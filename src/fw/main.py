"""
MicroPython firmware for everything
"""

from machine import PWM, Pin
from config import WIFI_SSID, WIFI_PASSWORD

import network
import time
import asyncio
import json

# Set up wireless connection

network.hostname("bill-esp32-drone")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)

print("Connecting to wifi...")
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

while not wifi.isconnected():
    time.sleep(0.1)

lan_addr = wifi.ifconfig()[0]
print(f"Connected! LAN address: {lan_addr}")


## =========
## ESC LOGIC
## ==========

ESC_PIN = 25
pwm = PWM(Pin(ESC_PIN), freq=50, duty_u16=0)
pwm.duty_ns(1_000_000)

def set_throttle(throttle):
    """
    Set throttle from 0 to 1000 (1000–2000 us pulses)
    """
    pwm.duty_ns((1000 + throttle) * 1000)


## ===============
## Sensor readings
## ===============


# Initialize load cell sensor
from drivers.load_cell import LoadCell
load_cell = LoadCell()

def get_sensor_data():
    """
    Read sensors yay
    """
    return {
        "force": load_cell.read(),
    }



## ====================
## Main networking loop
## ====================

# Post address to kv.wfeng.dev
# We do this by GETting kv.wfeng.dev/_/key/value

SEND_IP_ADDR = False
if SEND_IP_ADDR:
    import requests
    print("Sending IP address...")
    res = requests.get(f"https://kv.wfeng.dev/_/bill-esp32-drone-addr/{lan_addr}")
    print(f"Sending IP address got code:", res.status_code)
    res.close()

# Load the web framework after Wi-Fi has allocated its native buffers.
from microdot import Microdot
from microdot.websocket import with_websocket

app = Microdot()

@app.get("/")
async def md_index(req):
    return "online"


async def receive_commands(ws):
    """
    Get throttle commands, etc
    """
    while True:
        throttle = int(await ws.receive())
        set_throttle(throttle)


async def send_sensor_data(ws):
    """
    Continually send sensor data (w some delay)
    """
    while True:
        readings = get_sensor_data()
        await ws.send(json.dumps(readings))
        await asyncio.sleep_ms(100)


@app.route("/ws")
@with_websocket
async def md_websocket(req, ws):
    """
    Fires on a new websocket connection
    """
    tasks = [
        asyncio.create_task(send_sensor_data(ws)),
        asyncio.create_task(receive_commands(ws)),
    ]

    try:
        await asyncio.gather(*tasks)

    finally:
        # Ok cleanup is actually important here
        set_throttle(0)
        for task in tasks:
            task.cancel()


if __name__ == "__main__":

    print("Starting app...")
    app.run(port=80)
