"""
The camera off or on from the laptop, through the drone (its J 4 and J 5 requests: the camera's EN line held low, or released to boot): uv run -m sw.camera_power off|on. Only with the drone disarmed. Off stops the camera's chip, radio and image sensor until on or a drone restart, for example to see whether it is what disturbs the GPS (uv run -m sw.gps_remote watch).
"""

import argparse

from fw.tools.ota_direct import Drone


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("state", choices=("off", "on"))
    parser.add_argument("--bridge", default="ws://127.0.0.1:8765/update")
    args = parser.parse_args()
    print(Drone(args.bridge).request(b"J", b"\x04" if args.state == "off" else b"\x05").decode())


if __name__ == "__main__":
    main()
