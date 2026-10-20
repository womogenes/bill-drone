"""
MicroPython firmware for everything
"""

from machine import ADC, Pin, freq
import config
from config import WIFI_SSID, WIFI_PASSWORD

import network
import time
import asyncio
import socket
import flight_rt

freq(240_000_000)

# Set up wireless connection

network.hostname("bill-esp32-drone")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.config(pm=network.WLAN.PM_NONE)

direct_peer = getattr(config, "DIRECT_CONTROLLER_MAC", None)
if direct_peer is not None:
    wifi.disconnect()
    wifi.config(channel=getattr(config, "DIRECT_CHANNEL", 6))
    lan_addr = None
    # Allocate the radio buffers before other modules fragment the heap.
    from drivers import direct
    direct_link = direct.Link(direct_peer)
    print("Direct ESP-NOW controller enabled")
else:
    print("Connecting to wifi...")
    wifi.connect(WIFI_SSID, WIFI_PASSWORD)

    while not wifi.isconnected():
        time.sleep(0.1)

    lan_addr = wifi.ifconfig()[0]
    print(f"Connected! LAN address: {lan_addr}")


from drivers import actuators, control, recorder, protocol, ota

actuators.init()
control.load_gains()


## ===============
## Sensor readings
## ===============


# Initialize load cell sensor
from drivers.load_cell import LoadCell
from drivers import imu

load_cell = LoadCell()
imu.init()

adc = ADC(Pin(35), atten=ADC.ATTN_6DB)
adc.block().init(bits=12)
VOLTAGE_DIVIDER_RATIO = (100_000 + 10_000) / 10_000


def get_sensor_data():
    """
    Read sensors yay
    """
    state = flight_rt.snapshot()
    control.sync(state)
    force = load_cell.read()
    voltage = adc.read_uv() * VOLTAGE_DIVIDER_RATIO / 1_000_000
    flight_rt.set_voltage(round(voltage * 1000))

    return {
        "force": force,
        "force_age_ms": (
            time.ticks_diff(time.ticks_ms(), load_cell.last_read_ms)
            if load_cell.last_read_ms is not None
            else None
        ),
        "force_sequence": load_cell.sequence,
        "actuators": actuators.read(state),
        "control": control.settings,
        "control_status": control.status,
        "logging": recorder.status(),
        "adc_raw": adc.read_u16(),
        "adc_voltage": voltage,
        "imu": imu.read(state),
    }


## ====================
## Main networking loop
## ====================

# Post address to kv.wfeng.dev
# We do this by GETting kv.wfeng.dev/_/key/value

SEND_IP_ADDR = True
if SEND_IP_ADDR and lan_addr:
    import requests

    print("Sending IP address...")
    res = requests.get(f"http://kv.wfeng.dev/_/bill-esp32-drone-addr/{lan_addr}")
    print(f"Sending IP address got code:", res.status_code)
    res.close()

# Load the web framework after Wi-Fi has allocated its native buffers.
from microdot import Microdot
from microdot.websocket import with_websocket

app = Microdot()
command_owner = None


@app.get("/")
async def md_index(req):
    return "online"


async def receive_commands(ws):
    """
    Dispatch servo and throttle commands.
    """
    global command_owner

    while True:
        # Buffered frames can complete without yielding; give other clients a turn.
        await asyncio.sleep_ms(0)
        command = protocol.decode_command(await ws.receive())
        op = command["command"]

        if op == "stop":
            control.stop("E-stop triggered.")
            command_owner = None
            continue

        if op == "heartbeat":
            if command_owner is ws:
                flight_rt.heartbeat()
            continue

        if command_owner is not None and command_owner is not ws:
            continue

        command_owner = ws
        flight_rt.heartbeat()

        if op == "calibrate_imu":
            imu.calibrate()

        elif imu.calibration_count:
            continue

        elif op == "zero_servo":
            control.zero_servo(command["side"])

        elif op == "control":
            if command["settings"].get("enabled") and (
                imu.read() is None
                or time.ticks_diff(time.ticks_ms(), imu.last_read_ms) > 100
            ):
                raise ValueError("IMU is offline")

            control.configure(command["settings"])

        else:
            if control.settings["enabled"]:
                continue

            commands = {
                "servo": actuators.set_servo,
                "throttle": actuators.set_throttle,
            }
            commands[op](command["side"], command["value"])


TELEMETRY_HZ = 24
telemetry_packet = None
telemetry_reason = ""
telemetry_subscribers = []


async def sample_telemetry():
    """
    Read and encode one snapshot for all dashboards.
    """
    global telemetry_packet, telemetry_reason
    while True:
        started = time.ticks_ms()
        if telemetry_subscribers:
            telemetry_packet = protocol.encode_reading(get_sensor_data())
            telemetry_reason = control.status["reason"]
            for ready in telemetry_subscribers:
                ready.set()

        elapsed = time.ticks_diff(time.ticks_ms(), started)
        await asyncio.sleep_ms(max(0, round(1000 / TELEMETRY_HZ) - elapsed))


async def send_sensor_data(ws):
    """
    Send the newest snapshot; a slow client never queues stale readings.
    """
    ready = asyncio.Event()
    telemetry_subscribers.append(ready)
    previous_reason = None
    try:
        while True:
            await ready.wait()
            ready.clear()
            packet, reason = telemetry_packet, telemetry_reason
            if reason != previous_reason:
                await ws.send(b"R" + reason.encode())
                previous_reason = reason
            await ws.send(packet)
    finally:
        telemetry_subscribers.remove(ready)


@app.route("/ws")
@with_websocket
async def md_websocket(req, ws):
    """
    Fires on a new websocket connection
    """
    global command_owner
    req.sock[1].s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    await serve_connection(ws)


async def serve_connection(ws):
    global command_owner
    tasks = [
        asyncio.create_task(send_sensor_data(ws)),
        asyncio.create_task(receive_commands(ws)),
    ]

    try:
        await asyncio.gather(*tasks)

    finally:
        if command_owner is ws:
            control.stop("Controller disconnected")
            command_owner = None

        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


async def receive_log_heartbeat(ws):
    while True:
        await asyncio.wait_for_ms(ws.receive(), 3000)


@app.route("/log")
@with_websocket
async def flight_log(req, ws):
    req.sock[1].s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    tasks = [asyncio.create_task(recorder.stream(ws)), asyncio.create_task(receive_log_heartbeat(ws))]
    try:
        await asyncio.gather(*tasks)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


async def main():
    """
    Serve commands and telemetry while the native core handles sampling and control.
    """
    try:
        tasks = [sample_telemetry(), control.save_gains(), imu.save_calibration()]
        if direct_peer is not None:
            tasks.append(direct.run(direct_link, serve_connection, recorder.stream))
        else:
            tasks.extend([app.start_server(port=80), ota.confirm_boot(app)])
        await asyncio.gather(*tasks)

    finally:
        control.stop()
        imu.stop()


if __name__ == "__main__":
    print("Starting app...")
    asyncio.run(main())
