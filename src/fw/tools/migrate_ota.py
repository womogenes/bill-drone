"""
Move a USB-connected drone from the factory layout to the dual-slot OTA layout.

1. Interrupt the application and export every file, checked by SHA-256, into a
   private local directory that is kept afterwards.
2. After a typed confirmation, erase NVS, otadata and the new filesystem region,
   then write the OTA bootloader, partition table and runtime (slot ota_0).
3. Let MicroPython format the new 320 KiB filesystem, restore every file with
   main.py last, verify each hash, and restart.

A failure before step 2 changes nothing on the ESP. A failure after it leaves the
export on disk; rerun with --restore EXPORT to repeat step 3.
"""

import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import time

import esptool
from esptool import cmds
from mpremote.transport_serial import SerialTransport

from upload_usb import HASH_CODE, enter_repl

FIRMWARE = Path(__file__).resolve().parents[1]
VFS_SIZE = 0x50000
BLOCK_SIZE = 4096
EXPECTED = {
    "nvs": (1, 0x02, 0x9000, 0x4000),
    "otadata": (1, 0x00, 0xD000, 0x2000),
    "ota_0": (0, 0x10, 0x10000, 0x1D0000),
    "ota_1": (0, 0x11, 0x1E0000, 0x1D0000),
    "vfs": (1, 0x81, 0x3B0000, VFS_SIZE),
}
REQUIRED = ["config.py", "main.py", "flight-tuning.json", "imu-calibration.json"]
LIST_CODE = """
def walk(path):
    for entry in os.ilistdir(path or "/"):
        child = path + "/" + entry[0] if path else entry[0]
        if entry[1] == 0x4000:
            walk(child)
        else:
            print(repr((child, os.stat(child)[6], file_hash(child))))
walk("")
"""


def validate_layout(build):
    table = (build / "partition_table/partition-table.bin").read_bytes()
    partitions = {}
    for index in range(0, len(table), 32):
        magic, kind, subtype, offset, size, label, flags = struct.unpack("<HBBII16sI", table[index:index + 32])
        if magic != 0x50AA:
            break
        partitions[label.rstrip(b"\0").decode()] = (kind, subtype, offset, size)
    for name, expected in EXPECTED.items():
        if partitions.get(name) != expected:
            raise SystemExit(f"Unexpected {name} partition in {build}: {partitions.get(name)}")
    if not 0 < (build / "micropython.bin").stat().st_size <= EXPECTED["ota_0"][3]:
        raise SystemExit("Runtime does not fit the OTA application slot")
    if not 0 < (build / "bootloader/bootloader.bin").stat().st_size <= 0x7000:
        raise SystemExit("Bootloader overlaps the partition table")


def repl(port):
    board = SerialTransport(port, timeout=1)
    enter_repl(board)
    board.exec("import os, esp32\n" + HASH_CODE)
    return board


def running_label(board):
    return board.exec("print(esp32.Partition(esp32.Partition.RUNNING).info()[4])").decode().strip()


def export(board, directory):
    if running_label(board) != "factory":
        raise SystemExit("Drone does not run the factory layout; it may already be migrated")
    board.exec("import flight_rt")
    entries = [ast.literal_eval(line) for line in board.exec(LIST_CODE).decode().splitlines()]
    names = [entry[0] for entry in entries]
    missing = [name for name in REQUIRED if name not in names]
    if missing:
        raise SystemExit(f"Drone filesystem lacks {missing}; nothing was changed")
    estimate = sum(math.ceil(size / BLOCK_SIZE) * BLOCK_SIZE + BLOCK_SIZE for name, size, digest in entries)
    if estimate + 65536 > VFS_SIZE:
        raise SystemExit(f"Files need about {estimate + 65536} of {VFS_SIZE} bytes; nothing was changed")

    directory.mkdir(mode=0o700, parents=True)
    for name, size, digest in entries:
        data = board.fs_readfile(name)
        if hashlib.sha256(data).hexdigest() != digest:
            raise SystemExit(f"Export of {name} failed verification; nothing was changed")
        target = directory / "files" / name
        target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        target.write_bytes(data)
    (directory / "manifest.json").write_text(json.dumps(entries, indent=1) + "\n")
    print(f"Exported {len(entries)} files ({sum(entry[1] for entry in entries)} bytes) to {directory}")


def flash(port, build):
    with esptool.detect_chip(port) as connection:
        if connection.CHIP_NAME != "ESP32":
            raise SystemExit(f"Connected chip is {connection.CHIP_NAME}")
        target = cmds.run_stub(connection)
        cmds.attach_flash(target)
        for name in ("nvs", "otadata", "vfs"):
            cmds.erase_region(target, *EXPECTED[name][2:])
        cmds.write_flash(target, [
            (0x1000, str(build / "bootloader/bootloader.bin")),
            (0x8000, str(build / "partition_table/partition-table.bin")),
            (0x10000, str(build / "micropython.bin")),
        ])
        cmds.reset_chip(target)


def restore(board, directory):
    if running_label(board) != "ota_0":
        raise SystemExit("Drone did not boot ota_0 after flashing")
    entries = json.loads((directory / "manifest.json").read_text())
    entries.sort(key=lambda entry: entry[0] == "main.py")
    for name, size, digest in entries:
        parts = name.split("/")[:-1]
        for index in range(1, len(parts) + 1):
            board.exec(f"try:\n os.mkdir({'/'.join(parts[:index])!r})\nexcept OSError as error:\n if error.errno != 17: raise")
        board.fs_writefile(name + ".upload", (directory / "files" / name).read_bytes(), chunk_size=2048)
        if board.exec(f"print(file_hash({name + '.upload'!r}))").decode().strip() != digest:
            raise SystemExit(f"Restore of {name} failed verification; rerun with --restore {directory}")
        board.exec(f"os.rename({name + '.upload'!r}, {name!r})")
    print(f"Restored and verified {len(entries)} files")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="The drone's USB serial device, never the controller's")
    parser.add_argument("--build", type=Path, default=FIRMWARE / "native/build-runtime-ota")
    parser.add_argument("--export", type=Path,
                        default=Path.home() / ".local/state/bill-drone" / time.strftime("fs-%Y%m%dT%H%M%S"))
    parser.add_argument("--restore", type=Path, help="Only restore this earlier export onto a migrated drone")
    args = parser.parse_args()
    validate_layout(args.build)

    if args.restore is None:
        board = repl(args.port)
        try:
            export(board, args.export)
        finally:
            board.close()
        answer = input(f"Erase NVS and rewrite bootloader, partition table and app on {args.port}? Type MIGRATE: ")
        if answer != "MIGRATE":
            raise SystemExit("Cancelled; nothing on the ESP was changed")
        flash(args.port, args.build)
        time.sleep(3)
    board = repl(args.port)
    try:
        restore(board, args.restore or args.export)
        board.exec_raw_no_follow("import machine; machine.reset()")
    finally:
        board.close()
    print(f"Migrated. Keep {args.restore or args.export} until tuning and calibration are confirmed.")


if __name__ == "__main__":
    os.umask(0o077)
    main()
