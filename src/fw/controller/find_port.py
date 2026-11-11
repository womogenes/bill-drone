# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5"]
# ///
"""
Finds the controller's USB serial device by listening: a controller sends its link report in the bridge's framing every 250 ms, whether or not it hears a drone, and nothing else on USB does. The device names follow plug order (the controller came up as each of /dev/ttyUSB0 and 1 on 2026-10-30 and 31), and the boards' USB chips all carry the same serial number, so neither names it. No board is reset: until 2026-11-10 this asked each board's bootloader for its MAC, which reset every ESP32 on USB, the drone too, each time the bridge's service restarted.

The bridge (bridge.py) looks for its controller with this and keeps looking while there is none. Alone it prints the device, or nothing when no controller is heard: uv run src/fw/controller/find_port.py
"""

import glob
import time

import serial

from wire import SerialDecoder, unpack

BAUD = 921600


def open_port(path, baud=BAUD):
    """The device opened for the bridge: non-blocking, locked against a second bridge, its reset lines released (pyserial's defaults hold the board in reset)."""
    port = serial.Serial()
    port.port, port.baudrate = path, baud
    port.timeout = port.write_timeout = 0
    port.exclusive = True
    port.dtr = port.rts = False
    port.open()
    return port


def listen(path, seconds=1.0, baud=BAUD):
    """The device, open, if a controller's packet arrives on it within seconds; else None (another board, a controller still booting, or a device something else holds)."""
    try:
        port = open_port(path, baud)
    except (OSError, serial.SerialException):
        return None
    decoder = SerialDecoder()
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            for packet in decoder.feed(port.read(4096)):
                try:
                    unpack(packet)
                except ValueError:
                    continue
                return port
            time.sleep(0.02)
    except (OSError, serial.SerialException):
        pass
    port.close()
    return None


def candidates():
    return sorted(glob.glob("/dev/ttyUSB*"))


if __name__ == "__main__":
    for path in candidates():
        port = listen(path)
        if port is not None:
            port.close()
            print(path)
            break
