# /// script
# requires-python = ">=3.11"
# dependencies = ["esptool>=4.7,<6"]
# ///
"""
Prints the USB serial device of the ESP32 with the given MAC (the controller's by default), for the bridge's service (sw/bill-drone-bridge.service): the device names follow plug order, and the controller came up as each of /dev/ttyUSB0 and 1 on 2026-10-30 and 31. Asking each board for its MAC resets it, the drone too when it is on USB.

uv run src/fw/controller/find_port.py [--mac 04:b2:47:54:80:4c]
"""

import argparse
import glob
import subprocess
import sys

CONTROLLER = "04:b2:47:54:80:4c"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--mac", default=CONTROLLER)
    args = parser.parse_args()
    for port in sorted(glob.glob("/dev/ttyUSB*")):
        result = subprocess.run([sys.executable, "-m", "esptool", "--no-stub", "--port", port, "--after", "hard_reset", "read_mac"],
                                capture_output=True, text=True, timeout=30)
        if args.mac in (result.stdout + result.stderr).lower():
            print(port)
            return
    sys.exit(f"No ESP32 with MAC {args.mac} on /dev/ttyUSB*")


if __name__ == "__main__":
    main()
