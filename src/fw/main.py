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


@app.get("/")
async def md_index(req):
    return "online"


async def receive_commands(ws, last_command):
    """
    Dispatch servo and throttle commands.
    """
    while True:
        command = json.loads(await ws.receive())
        last_command[0] = time.ticks_ms()
        op = command["command"]
        if op == "heartbeat":
            continue
        if op == "stop":
            control.stop()
        elif op == "control":
            if command["settings"].get("enabled") and (
                imu.read() is None
                or time.ticks_diff(time.ticks_ms(), imu.last_read_ms) > 100
            ):
                raise ValueError("IMU is offline")
            control.configure(command["settings"])
        else:
            if op == "throttle" and control.settings["enabled"]:
                control.stop()
                continue
            commands = {
                "servo": actuators.set_servo,
                "throttle": actuators.set_throttle,
            }
            commands[op](command["side"], command["value"])


async def watch_commands(last_command):
    """
    Stop ESCs on command silence without closing the telemetry connection.
    """
    while True:
        now = time.ticks_ms()
        if time.ticks_diff(now, last_command[0]) >= 500 or (
            control.settings["enabled"] and time.ticks_diff(now, imu.last_read_ms) > 100
        ):
            control.stop()
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
    last_command = [time.ticks_ms()]
    tasks = [
        asyncio.create_task(send_sensor_data(ws)),
        asyncio.create_task(receive_commands(ws, last_command)),
        asyncio.create_task(watch_commands(last_command)),
    ]

    try:
        await asyncio.gather(*tasks)

    finally:
        control.stop()
        for task in tasks:
            task.cancel()


async def main():
    """
    Sample the IMU independently of dashboard connections.
    """
    try:
        await asyncio.gather(imu.sample(), app.start_server(port=80))
    finally:
        control.stop()


if __name__ == "__main__":
    print("Starting app...")
    asyncio.run(main())
