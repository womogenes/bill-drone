"""
MicroPython firmware for everything
"""

from machine import ADC, Pin, freq
from config import WIFI_SSID, WIFI_PASSWORD

import network
import time
import asyncio
import json

freq(240_000_000)

# Set up wireless connection

network.hostname("bill-esp32-drone")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.config(pm=network.WLAN.PM_NONE)

print("Connecting to wifi...")
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

while not wifi.isconnected():
    time.sleep(0.1)

lan_addr = wifi.ifconfig()[0]
print(f"Connected! LAN address: {lan_addr}")


from drivers import actuators, control

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
    force = load_cell.read()

    return {
        "force": force,
        "force_age_ms": (
            time.ticks_diff(time.ticks_ms(), load_cell.last_read_ms)
            if load_cell.last_read_ms is not None
            else None
        ),
        "force_sequence": load_cell.sequence,
        "actuators": actuators.read(),
        "control": control.settings,
        "control_status": control.status,
        "adc_raw": adc.read_u16(),
        "adc_voltage": adc.read_uv() * VOLTAGE_DIVIDER_RATIO / 1_000_000,
        "imu": imu.read(),
    }


## ====================
## Main networking loop
## ====================

# Post address to kv.wfeng.dev
# We do this by GETting kv.wfeng.dev/_/key/value

SEND_IP_ADDR = True
if SEND_IP_ADDR:
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
last_command_ms = 0


@app.get("/")
async def md_index(req):
    return "online"


async def receive_commands(ws):
    """
    Dispatch servo and throttle commands.
    """
    global command_owner, last_command_ms

    while True:
        command = json.loads(await ws.receive())
        op = command["command"]

        if op == "stop":
            control.stop("Stopped from dashboard")
            command_owner = None
            continue

        if op == "heartbeat":
            if command_owner is ws:
                last_command_ms = time.ticks_ms()
            continue

        if command_owner is not None and command_owner is not ws:
            continue

        command_owner = ws
        last_command_ms = time.ticks_ms()

        if op == "calibrate_imu":
            imu.calibrate()

        elif imu.calibration_count:
            continue

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


async def heartbeat_esc():
    """
    Stop outputs when their controlling connection stops sending commands.
    """
    global command_owner

    while True:
        now = time.ticks_ms()
        if command_owner is not None and time.ticks_diff(now, last_command_ms) >= 500:
            control.stop("Controller heartbeat lost")
            command_owner = None

        if control.settings["enabled"] and time.ticks_diff(now, imu.last_read_ms) > 100:
            control.stop("IMU samples stopped")

        await asyncio.sleep_ms(50)


TELEMETRY_HZ = 24


async def send_sensor_data(ws):
    """
    Stream sensor readings at a target of 24 Hz.
    """
    while True:
        started = time.ticks_ms()
        readings = get_sensor_data()
        await ws.send(json.dumps(readings))

        elapsed = time.ticks_diff(time.ticks_ms(), started)
        await asyncio.sleep_ms(max(0, round(1000 / TELEMETRY_HZ) - elapsed))


@app.route("/ws")
@with_websocket
async def md_websocket(req, ws):
    """
    Fires on a new websocket connection
    """
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


async def main():
    """
    Sample the IMU independently of dashboard connections.
    """
    try:
        await asyncio.gather(
            imu.sample(), heartbeat_esc(), control.save_gains(), app.start_server(port=80)
        )

    finally:
        control.stop()


if __name__ == "__main__":
    print("Starting app...")
    asyncio.run(main())
