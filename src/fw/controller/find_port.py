# /// script
# requires-python = ">=3.11"
# dependencies = ["esptool>=4.7,<6"]
# ///
"""
Prints the USB serial device of the ESP32 with the given MAC (by default any controller in fw/pairs.h), for the bridge's service (sw/bill-drone-bridge.service): the device names follow plug order, and the controller came up as each of /dev/ttyUSB0 and 1 on 2026-10-30 and 31. Asking each board for its MAC resets it, the drone too when it is on USB.

uv run src/fw/controller/find_port.py [--mac aa:bb:cc:dd:ee:ff]
"""

import argparse
import glob
import re
import subprocess
import sys
from pathlib import Path

PAIRS = Path(__file__).parent.parent / "pairs.h"


def controllers():
    """The controllers' MACs in fw/pairs.h: the second of each pair's two."""
    pairs = re.findall(r"\{\{([^}]*)\}, \{([^}]*)\}\}", PAIRS.read_text())
    return [":".join(byte.strip()[2:] for byte in controller.split(",")) for _, controller in pairs]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--mac")
    args = parser.parse_args()
    wanted = [args.mac.lower()] if args.mac else controllers()
    for port in sorted(glob.glob("/dev/ttyUSB*")):
        result = subprocess.run([sys.executable, "-m", "esptool", "--no-stub", "--port", port, "--after", "hard_reset", "read_mac"],
                                capture_output=True, text=True, timeout=30)
        if any(mac in (result.stdout + result.stderr).lower() for mac in wanted):
            print(port)
            return
    sys.exit(f"No ESP32 with MAC {' or '.join(wanted)} on /dev/ttyUSB*")


if __name__ == "__main__":
    main()
