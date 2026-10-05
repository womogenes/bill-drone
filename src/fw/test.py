"""
Test program. Run with

mpremote resume mount . run test.py repl
"""

from config import WIFI_SSID, WIFI_PASSWORD

import network
import time
import requests

network.hostname("bill-esp32-drone")
wifi = network.WLAN(network.STA_IF)
wifi.active(True)

print("Connecting to wifi...")
wifi.connect(WIFI_SSID, WIFI_PASSWORD)

while not wifi.isconnected():
    time.sleep(0.1)

lan_addr = wifi.ifconfig()[0]
print(f"Connected! LAN address: {lan_addr}")

# Post address to kv.wfeng.dev
# We do this by GETting kv.wfeng.dev/_/key/value
print("Sending IP address...")
res = requests.get(f"https://kv.wfeng.dev/_/bill-esp32-drone-addr/{lan_addr}")
print(f"Sending IP address got code:", res.status_code)
