"""
Flash changed application sectors, checking only the chunks written.
"""

import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import shutil
import time

import esptool
from esptool import cmds

SECTOR_SIZE = 4096
CHUNK_SIZE = 64 * 1024
APP_ADDRESS = 0x10000
APP_SIZE = 0x1F0000


def changed_chunks(current, previous):
    """
    Combine adjacent changed sectors into writes of at most 64 KiB.
    """
    end = (len(current) + SECTOR_SIZE - 1) // SECTOR_SIZE * SECTOR_SIZE
    current = current.ljust(end, b"\xff")
    covered = 0
    if previous is not None:
        covered = (len(previous) + SECTOR_SIZE - 1) // SECTOR_SIZE * SECTOR_SIZE
        previous = previous.ljust(end, b"\xff")
    chunks = []
    start = None

    for offset in range(0, end, SECTOR_SIZE):
        changed = (
            offset >= covered
            or current[offset:offset + SECTOR_SIZE] != previous[offset:offset + SECTOR_SIZE]
        )
        if start is not None and (not changed or offset - start == CHUNK_SIZE):
            chunks.append((start, current[start:offset]))
            start = None
        if changed and start is None:
            start = offset

    if start is not None:
        chunks.append((start, current[start:end]))
    return chunks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("image", type=Path)
    args = parser.parse_args()

    image = args.image.resolve()
    lock = image.with_name(".flash.lock").open("w")
    fcntl.flock(lock, fcntl.LOCK_EX)
    previous = image.with_name("last-flashed.bin")
    manifest = image.with_name("last-flashed.json")
    pending = image.with_name("flashing.bin")
    # Freeze the input so another build can run during the transfer.
    with image.with_name(".build.lock").open("w") as build_lock:
        fcntl.flock(build_lock, fcntl.LOCK_SH)
        shutil.copyfile(image, pending)
    current = pending.read_bytes()
    if not 0 < len(current) <= APP_SIZE:
        parser.error("Image does not fit the factory application partition")

    started = time.monotonic()
    with esptool.detect_chip(args.port) as connection:
        chip = connection.CHIP_NAME
        mac = ":".join(f"{byte:02x}" for byte in connection.read_mac())
        if chip != "ESP32":
            parser.error(f"This application targets ESP32; connected chip is {chip}")
        print(f"Connected {chip} {mac}", flush=True)

        old = None
        if previous.exists() and manifest.exists():
            saved = json.loads(manifest.read_text())
            candidate = previous.read_bytes()
            if (
                saved.get("mac") == mac
                and saved.get("chip") == chip
                and saved["sha256"] == hashlib.sha256(candidate).hexdigest()
            ):
                old = candidate

        chunks = changed_chunks(current, old)
        if not chunks:
            print("Local successful-flash manifest matches this ESP and build; no writes")
            cmds.reset_chip(connection)
            pending.unlink()
            return

        # An interrupted upload invalidates the baseline for the next invocation.
        manifest.unlink(missing_ok=True)
        target = cmds.run_stub(connection)
        if args.baud != 115200:
            target.change_baud(args.baud)
        cmds.attach_flash(target)
        print(f"Writing {sum(len(data) for offset, data in chunks)} bytes in {len(chunks)} chunks", flush=True)
        cmds.write_flash(target, [(APP_ADDRESS + offset, data) for offset, data in chunks])
        cmds.reset_chip(target)

    # This is the local build used for the next comparison; no ESP backup is read.
    pending.replace(previous)
    saved = {
        "port": args.port,
        "chip": chip,
        "mac": mac,
        "sha256": hashlib.sha256(current).hexdigest(),
        "bytes": len(current),
        "written_at": time.time(),
    }
    temporary = manifest.with_suffix(".tmp")
    temporary.write_text(json.dumps(saved) + "\n")
    temporary.replace(manifest)
    lock.close()
    print(f"Changed chunks written and checked in {time.monotonic() - started:.1f}s")


if __name__ == "__main__":
    main()
