"""
Upload changed application files over one USB connection.
"""

import argparse
import ast
import hashlib
from pathlib import Path
import time

from mpremote.transport import TransportError
from mpremote.transport_serial import SerialTransport
from serial.tools import list_ports, miniterm

from mpy import compile_driver


FIRMWARE = Path(__file__).resolve().parents[1]
HASH_CODE = """
import os, hashlib, binascii
def file_hash(path):
    try:
        stream = open(path, "rb")
    except OSError as error:
        if error.errno == 2:
            return None
        raise
    digest = hashlib.sha256()
    with stream:
        while True:
            chunk = stream.read(2048)
            if not chunk:
                break
            digest.update(chunk)
    return binascii.hexlify(digest.digest()).decode()
"""


def select_port(requested):
    if requested != "auto":
        return requested

    ports = [port.device for port in list_ports.comports() if port.vid is not None]
    if len(ports) != 1:
        raise SystemExit(f"Choose --port explicitly; USB serial ports: {ports}")
    return ports[0]


def enter_repl(board):
    try:
        board.enter_raw_repl(timeout_overall=2)
        return
    except TransportError:
        print("Resetting ESP to enter REPL...")

    # EN reset, with GPIO0 released; interrupt boot before the application starts.
    board.serial.dtr = False
    board.serial.rts = True
    time.sleep(0.1)
    board.serial.rts = False
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        board.serial.write(b"\x03")
        time.sleep(0.05)
        board.serial.read(board.serial.in_waiting)

    board.enter_raw_repl(timeout_overall=3)


def upload(board, files):
    paths = [path.relative_to(FIRMWARE).as_posix() for path in files]
    board.exec(HASH_CODE)
    hashes = ast.literal_eval(
        board.exec(f"print({{path: file_hash(path) for path in {paths!r}}})").decode()
    )
    changed = 0
    transferred = 0

    for source, path in zip(files, paths):
        data = source.read_bytes()
        digest = hashlib.sha256(data).hexdigest()
        if hashes[path] != digest:
            for parent in reversed(Path(path).parents):
                if parent == Path("."):
                    continue
                board.exec(
                    f"try:\n os.mkdir({parent.as_posix()!r})\n"
                    "except OSError as error:\n"
                    " if error.errno != 17: raise"
                )

            temporary = path + ".upload"
            board.fs_writefile(temporary, data, chunk_size=2048)
            actual = board.exec(f"print(file_hash({temporary!r}))").decode().strip()
            if actual != digest:
                raise RuntimeError(f"Upload verification failed: {path}")

            board.exec(f"os.rename({temporary!r}, {path!r})")
            print(f"Updated {path} ({len(data)} bytes)")
            changed += 1
            transferred += len(data)

        if path.endswith(".mpy"):
            # Remove shadowing source even when the bytecode already matches.
            board.exec(
                f"try:\n os.remove({path[:-4] + '.py'!r})\n"
                "except OSError as error:\n"
                " if error.errno != 2: raise"
            )

    return changed, transferred


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto")
    parser.add_argument("--repl", action="store_true", help="Show boot output and terminal")
    parser.add_argument("files", nargs="*", help="Paths relative to src/fw")
    args = parser.parse_args()

    names = args.files or [
        "config.py",
        "main.py",
        *sorted(str(path.relative_to(FIRMWARE)) for path in (FIRMWARE / "drivers").glob("*.py")),
        "drivers/flight_native.mpy",
    ]
    files = [FIRMWARE / compile_driver(name) for name in names]
    for path in files:
        path.resolve().relative_to(FIRMWARE)
        if not path.is_file():
            parser.error(f"Missing file: {path}")

    started = time.monotonic()
    board = SerialTransport(select_port(args.port), timeout=1)
    try:
        enter_repl(board)
        changed, transferred = upload(board, files)
        print(
            f"{changed}/{len(files)} files updated, {transferred} bytes, "
            f"{time.monotonic() - started:.1f}s"
        )
        board.exec_raw_no_follow("import machine; machine.reset()")

        if args.repl:
            print("Use Ctrl-] to exit.")
            terminal = miniterm.Miniterm(board.serial, echo=False, eol="lf")
            terminal.exit_character = chr(29)
            terminal.raw = True
            terminal.start()
            terminal.join(True)
            terminal.stop()
            terminal.join()
    finally:
        board.close()


if __name__ == "__main__":
    main()
