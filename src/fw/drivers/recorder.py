"""
Stream the native control core's bounded binary log queue to the laptop.
"""

import asyncio
import json
import os
import struct
import time
import flight_rt
from . import actuators
from .calibration import THRUST_TABLE, SENSOR_TO_BODY
from .log_format import SAMPLE_FORMAT, HEADER_FORMAT, RECORD_SIZE, BATCH_SIZE, MAX_BATCH, SAMPLE_HZ, STOP_REASONS

connected = False
metadata = None
reason = None
session = events_dropped = 0
events = []


def status():
    state = flight_rt.record_status()
    return {
        "connected": connected,
        "active": state["active"],
        "ready": connected and metadata is None,
        "samples": state["samples"],
        "dropped": state["dropped"],
    }


def start(settings, servo_offsets):
    global metadata, reason, session, events_dropped
    # Imported here because imu imports this module through control.
    from .imu import DLPF
    session = int.from_bytes(os.urandom(4), "little")
    started = flight_rt.record_start(session)
    runtime = flight_rt.snapshot()
    events_dropped = 0
    events.clear()
    reason = None
    metadata = {
        "type": "start",
        "session": session,
        "version": 1,
        "sample_format": SAMPLE_FORMAT,
        "record_size": RECORD_SIZE,
        "target_hz": SAMPLE_HZ,
        "batch_ms": 1000 * BATCH_SIZE // SAMPLE_HZ,
        "imu_dlpf": DLPF,
        "pwm_hz": dict(actuators.PWM_HZ),
        "started_ticks_us": started,
        "settings": dict(settings),
        "gyro_bias_rad_s": list(runtime["gyro_bias"]),
        "pose_zero_deg": list(runtime["pose_zero"]),
        "calibrated": runtime["calibrated"],
        "control_core": runtime["core"],
        "network_core": runtime["vm_core"],
        "sensor_to_body": list(SENSOR_TO_BODY),
        "servo_offsets": dict(servo_offsets),
        "thrust_table": THRUST_TABLE,
    }


def stop(message=None):
    global reason
    if message is not None:
        reason = message
    flight_rt.record_stop()


def settings_changed(settings):
    global events_dropped
    state = flight_rt.record_status()
    if not state["active"]:
        return
    if len(events) == 8:
        events.pop(0)
        events_dropped += 1
        flight_rt.set_log_flags(2)
    events.append({
        "type": "settings",
        "time_us": state["time_us"],
        "settings": dict(settings),
    })


async def stream(ws):
    """
    A stalled socket can fill the queue; it cannot stall the control task.
    """
    global connected, metadata
    if connected:
        await ws.close()
        return

    connected = True
    announced = None
    heartbeat_ms = time.ticks_ms()
    try:
        await ws.send('{"type":"ready"}')
        while True:
            if metadata is not None and announced != session:
                await ws.send(json.dumps(metadata))
                announced = session

            state = flight_rt.record_status()
            if events:
                await ws.send(json.dumps(events.pop(0)))
            elif state["pending"] >= BATCH_SIZE or (state["pending"] and not state["active"]):
                data = flight_rt.read_log(min(state["pending"], MAX_BATCH))
                count = len(data) // RECORD_SIZE
                header = struct.pack(HEADER_FORMAT, b"FLG1", session, count, RECORD_SIZE)
                await ws.send(header + data)
                flight_rt.consume_log(count)
            elif metadata is not None and not state["active"] and not state["pending"]:
                await ws.send(json.dumps({
                    "type": "stop",
                    "session": session,
                    "reason": reason or STOP_REASONS[state["reason"]] or "Disabled",
                    "time_us": state["time_us"],
                    "samples": state["samples"],
                    "dropped": state["dropped"],
                    "events_dropped": events_dropped,
                }))
                metadata = None
            elif time.ticks_diff(time.ticks_ms(), heartbeat_ms) >= 1000:
                await ws.send('{"type":"ready"}')
                heartbeat_ms = time.ticks_ms()

            await asyncio.sleep_ms(5 if state["active"] or state["pending"] else 100)
    finally:
        connected = False
