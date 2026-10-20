"""
Update the inactive native firmware slot through authenticated WebREPL.
"""

import argparse
import getpass
import hashlib
from pathlib import Path
import time

import flash_web


def raw(ws):
    ws.write(b"\x01", flash_web.webrepl.WEBREPL_FRAME_TXT)
    ws.until(b"raw REPL; CTRL-B to exit\r\n>")


def normal(ws):
    ws.write(b"\x02", flash_web.webrepl.WEBREPL_FRAME_TXT)
    ws.until(b">>> ")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("image", type=Path, help="micropython.bin application image; never merged firmware.bin")
    parser.add_argument("--repl", action="store_true")
    args = parser.parse_args()
    image = args.image.read_bytes()
    if image[:1] != b"\xe9":
        parser.error("Expected an ESP application image")
    size = len(image)
    del image
    password = getpass.getpass("WebREPL password: ")
    ws = flash_web.connect(args.host, password)
    ws.s.settimeout(30)
    started = time.monotonic()
    try:
        helper = flash_web.FIRMWARE / "drivers/ota.py"
        remote = "/drivers/ota.py"
        hashes = flash_web.remote_hashes(ws, [remote])
        if hashes.get(remote) != hashlib.sha256(helper.read_bytes()).hexdigest():
            flash_web.put(ws, helper, remote)
        raw(ws)
        ws.execute(
            'from drivers import control, imu\ncontrol.stop("Firmware update")\n'
            'imu.stop()\nfrom drivers import ota\n'
            f"upload = ota.mount({size})\n"
        )
        normal(ws)
        flash_web.put(ws, args.image, "/ota/firmware.bin")
        raw(ws)
        ws.execute("upload.finish()")
        print(f"Wrote and checked {size:,} bytes in chunks in {time.monotonic() - started:.1f}s; restarting")
        normal(ws)
        flash_web.reset(ws)
    except BaseException:
        print("Update failed. Motors are stopped; the previous image remains selected unless verification completed.")
        print("Reset the ESP to resume. No automatic reset is attempted after a failed transfer.")
        raise
    finally:
        ws.s.close()

    if args.repl:
        time.sleep(2)
        deadline = time.monotonic() + 30
        while True:
            try:
                ws = flash_web.connect(args.host, password)
                break
            except (OSError, AssertionError):
                if time.monotonic() > deadline:
                    raise TimeoutError("ESP did not reconnect; inspect USB startup") from None
                time.sleep(1)
        try:
            ws.s.settimeout(None)
            flash_web.webrepl.do_repl(ws)
        finally:
            ws.s.close()


if __name__ == "__main__":
    main()
