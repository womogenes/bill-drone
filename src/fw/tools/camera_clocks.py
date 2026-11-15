# /// script
# requires-python = ">=3.11"
# dependencies = ["websockets>=15,<17"]
# ///
"""
The camera's image-sensor clocks and drives through the drone (K C, fw/esp32cam/src/clocks.h), for finding a setting that keeps the camera's harmonics off the GPS. Nothing is kept: the camera's own defaults come back at its next restart (or --reset).

uv run tools/camera_clocks.py                      # the current mode's clocks as the hardware has them
uv run tools/camera_clocks.py --mode fpv           # FPV's (hw after its last start, else the next start's plan)
uv run tools/camera_clocks.py --mode tag --pin-drive 0 --pad-drive 0   # weakest XCLK pin and sensor pads in tag mode
uv run tools/camera_clocks.py --mode tag --xclk 16000000               # XCLK 160/10 MHz, the sensor's PLL keeping SYSCLK and PCLK
uv run tools/camera_clocks.py --reset              # restarts the camera (K X): every override gone

A setting restarts the camera in its current mode at once; the other mode takes it at its next start. The reply: mode (t or f), XCLK in MHz and its divider of 160 MHz (+b/a when fractional, which dithers), PLLCLK, SYSCLK and PCLK in MHz, XCLK pin and sensor pad drives (0-3), and hw (read back after the start) or plan. Run it with the drone disarmed: the camera stops for about a second.
"""

import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ota_direct import Drone  # noqa: E402

MODES = {"tag": 0, "fpv": 1}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=MODES)
    parser.add_argument("--xclk", type=int, default=0, help="Hz (6-27 MHz; 0 the mode's default)")
    parser.add_argument("--fractional", action="store_true", help="allow the divider's fraction (dithered)")
    parser.add_argument("--pin-drive", type=int, choices=range(4), help="XCLK pin drive, gpio_drive_cap_t (default 2)")
    parser.add_argument("--pad-drive", type=int, choices=range(4), help="sensor pads 0-3 = 1x-4x (driver 3)")
    parser.add_argument("--multiplier", type=int, default=0, help="sensor PLL multiplier (0 keeps the mode's PLLCLK)")
    parser.add_argument("--prediv", type=int, default=0, choices=range(4), help="with --multiplier: 1, 1.5, 2, 3")
    parser.add_argument("--pclk-divider", type=int, default=0, help="0 the mode's own (tag 8, FPV 10)")
    parser.add_argument("--reset", action="store_true", help="restart the camera, clearing every override")
    parser.add_argument("--bridge", default="ws://127.0.0.1:8765/update")
    args = parser.parse_args()
    drone = Drone(args.bridge)
    if args.reset:
        drone.request(b"K", b"X", patience=10)
        time.sleep(4)
        print(drone.request(b"K", b"C", patience=20).decode(errors="replace"))
        return
    setting = args.xclk or args.fractional or args.pin_drive is not None or args.pad_drive is not None or args.multiplier or args.pclk_divider
    if setting and args.mode is None:
        parser.error("a setting needs --mode")
    body = b"" if args.mode is None else bytes([MODES[args.mode]])
    if setting:
        drive = lambda value: 0xFF if value is None else value
        body += struct.pack("<IBBBBBB", args.xclk, int(args.fractional), drive(args.pin_drive), drive(args.pad_drive), args.multiplier, args.prediv, args.pclk_divider)
    print(drone.request(b"K", b"C" + body).decode(errors="replace"))
    if setting:
        # The camera restarts in about a second; then the hardware's values.
        time.sleep(2.5)
        print(drone.request(b"K", b"C" + bytes([MODES[args.mode]])).decode(errors="replace"))


if __name__ == "__main__":
    main()
