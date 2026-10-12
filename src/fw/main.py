"""
MicroPython firmware for everything
"""

from machine import ADC, PWM, Pin
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

# Physical wiring: positions counted from the top of each header.
LEFT_ESC_PIN = 14      # 12th from top left
LEFT_SERVO_PIN = 26    # 10th from top left
RIGHT_ESC_PIN = 19     # 8th from top right
RIGHT_SERVO_PIN = 18   # 9th from top right

# The existing single-pair commands control the left side.
# Right-side pins are reserved here; their PWM outputs are not initialized yet.
pwm = PWM(Pin(LEFT_ESC_PIN), freq=50, duty_u16=0)
pwm.duty_ns(1_000_000)

def set_throttle(throttle):
    """
    Set throttle from 0 to 1000 (1000–2000 us pulses)
    """
    pwm.duty_ns((1000 + throttle) * 1000)


## Left servo: MG995, GPIO26

SERVO_LIMIT = 60
servo = PWM(Pin(LEFT_SERVO_PIN), freq=50, duty_u16=0)
servo_angle = 0

def set_servo(angle):
    """
    Set degrees relative to center, with 0 at 1500 us.
    """
    global servo_angle
    if not -SERVO_LIMIT <= angle <= SERVO_LIMIT:
        raise ValueError("Servo angle must be between -%s and %s" % (SERVO_LIMIT, SERVO_LIMIT))

    servo.duty_ns(round(1_500_000 + angle * 1_000_000 / 90))
    servo_angle = angle

set_servo(0)


## ===============
## Sensor readings
## ===============


# Initialize load cell sensor
from drivers.load_cell import LoadCell
from drivers import imu
load_cell = LoadCell()
imu.init()

shunt = ADC(Pin(34))
shunt.init(atten=ADC.ATTN_0DB, width=ADC.WIDTH_12BIT)

def get_sensor_data():
    """
    Read sensors yay
    """
    force = load_cell.read()
    return {
        "force": force,
        "force_age_ms": (
            time.ticks_diff(time.ticks_ms(), load_cell.last_read_ms)
            if load_cell.last_read_ms is not None else None
        ),
        "force_sequence": load_cell.sequence,
        "servo_angle": servo_angle,
        "shunt_raw": shunt.read(),
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
    Dispatch servo and throttle commands.
    """
    commands = {"servo": set_servo, "throttle": set_throttle}
    while True:
        command = json.loads(await ws.receive())
        commands[command["command"]](command["value"])


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
