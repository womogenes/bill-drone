"""
Upload changed Python files over one WebREPL connection.
"""

import argparse
import getpass
import hashlib
import json
from pathlib import Path
import socket
import struct
import sys
import time

FIRMWARE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FIRMWARE / "lib/webrepl"))
import webrepl_cli as webrepl


class Connection(webrepl.websocket):
    def write(self, data, frame=webrepl.WEBREPL_FRAME_BIN):
        size = len(data)
        header = struct.pack(">BB", frame, size) if size < 126 else struct.pack(">BBH", frame, 126, size)
        self.s.sendall(header + data)

    def until(self, suffix):
        result = bytearray()
        deadline = time.monotonic() + 10
        while not result.endswith(suffix):
            if time.monotonic() > deadline:
                raise TimeoutError("ESP did not finish its REPL response")
            result.extend(self.read(1, text_ok=True))
        return bytes(result)

    def execute(self, source):
        data = source.encode()
        for offset in range(0, len(data), 256):
            self.write(data[offset:offset + 256], webrepl.WEBREPL_FRAME_TXT)
            time.sleep(0.01)
        self.write(b"\x04", webrepl.WEBREPL_FRAME_TXT)
        response = self.until(b"\x04>")
        if not response.startswith(b"OK"):
            raise RuntimeError("ESP did not accept the command")
        output, error = response[2:-2].split(b"\x04", 1)
        if error.strip():
            raise RuntimeError(error.decode())
        return output


def connect(host, password):
    connection = socket.create_connection((host, 8266), timeout=5)
    try:
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        connection.sendall(
            b"GET / HTTP/1.1\r\nHost: esp\r\nConnection: Upgrade\r\n"
            b"Upgrade: websocket\r\nSec-WebSocket-Key: foo\r\n\r\n"
        )
        with connection.makefile("rb", buffering=0) as response:
            if b" 101 " not in response.readline():
                raise ConnectionError("ESP refused the WebREPL handshake")
            while True:
                line = response.readline()
                if line == b"\r\n":
                    break
                if not line:
                    raise ConnectionError("ESP closed the WebREPL handshake")
        ws = Connection(connection)
        webrepl.login(ws, password)
        webrepl.get_ver(ws)
        return ws
    except BaseException:
        connection.close()
        raise


def put(ws, path, destination):
    name = destination.encode()
    if len(name) > 64:
        raise ValueError(f"WebREPL filename exceeds 64 bytes: {destination}")
    request = struct.pack(webrepl.WEBREPL_REQ_S, b"WA", webrepl.WEBREPL_PUT_FILE,
                          0, 0, path.stat().st_size, len(name), name)
    ws.write(request[:10])
    ws.write(request[10:])
    if webrepl.read_resp(ws):
        raise OSError(f"Cannot open {destination} on ESP")
    with path.open("rb") as source:
        while data := source.read(1024):
            ws.write(data)
    if webrepl.read_resp(ws):
        raise OSError(f"Upload failed: {destination}")


def remote_hashes(ws, names):
    ws.write(b"\r\x03\x03", webrepl.WEBREPL_FRAME_TXT)
    ws.until(b">>> ")
    ws.write(b"\x01", webrepl.WEBREPL_FRAME_TXT)
    ws.until(b"raw REPL; CTRL-B to exit\r\n>")
    # Interrupting main() stops native sampling/control before file changes.
    source = '''import os, hashlib, binascii, json
result = {}
for name in NAMES:
    parent = name.rsplit("/", 1)[0]
    directory = ""
    for part in parent.split("/"):
        if part:
            directory += "/" + part
            try:
                os.mkdir(directory)
            except OSError as error:
                if error.args[0] != 17:
                    raise
    try:
        file = open(name, "rb")
    except OSError as error:
        if error.args[0] != 2:
            raise
        continue
    digest = hashlib.sha256()
    with file:
        while True:
            block = file.read(1024)
            if not block:
                break
            digest.update(block)
    result[name] = binascii.hexlify(digest.digest()).decode()
print(json.dumps(result))
'''.replace("NAMES", repr(names))
    hashes = json.loads(ws.execute(source))
    ws.write(b"\x02", webrepl.WEBREPL_FRAME_TXT)
    ws.until(b">>> ")
    return hashes


def reset(ws):
    # Use raw REPL acknowledgement before closing the connection.
    ws.write(b"\x01", webrepl.WEBREPL_FRAME_TXT)
    ws.until(b"raw REPL; CTRL-B to exit\r\n>")
    ws.write(b"import machine, time; time.sleep_ms(250); machine.reset()\x04", webrepl.WEBREPL_FRAME_TXT)
    ws.until(b"OK")
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("files", nargs="*", help="Paths relative to src/fw; default: application and compiled libraries")
    parser.add_argument("--repl", action="store_true", help="Reconnect to terminal after reset")
    parser.add_argument("--no-reset", action="store_true", help="Leave the ESP at its REPL after upload")
    args = parser.parse_args()
    files = [FIRMWARE / name for name in args.files] if args.files else [
        FIRMWARE / "config.py", FIRMWARE / "main.py",
        *sorted((FIRMWARE / "drivers").glob("*.py")),
        FIRMWARE / "drivers/flight_native.mpy",
        *sorted((FIRMWARE / "lib/microdot").glob("*.mpy")),
    ]
    for path in files:
        path.resolve().relative_to(FIRMWARE)
    names = ["/" + path.relative_to(FIRMWARE).as_posix() for path in files]
    digests = [hashlib.sha256(path.read_bytes()).hexdigest() for path in files]
    password = getpass.getpass("WebREPL password: ")
    started = time.monotonic()
    ws = connect(args.host, password)
    try:
        hashes = remote_hashes(ws, names)
        changed = 0
        for path, name, digest in zip(files, names, digests):
            if hashes.get(name) != digest:
                put(ws, path, name)
                print(f"Uploaded {name}")
                changed += 1
        print(f"{changed} updated, {len(files) - changed} unchanged; {time.monotonic() - started:.1f}s")
        if not args.no_reset:
            reset(ws)
        elif args.repl:
            ws.s.settimeout(None)
            webrepl.do_repl(ws)
    finally:
        ws.s.close()

    if args.repl and not args.no_reset:
        time.sleep(2)
        deadline = time.monotonic() + 25
        while True:
            try:
                ws = connect(args.host, password)
                break
            except (OSError, AssertionError):
                if time.monotonic() > deadline:
                    raise TimeoutError("ESP did not reconnect after reset") from None
                time.sleep(1)
        try:
            ws.s.settimeout(None)
            webrepl.do_repl(ws)
        finally:
            ws.s.close()


if __name__ == "__main__":
    main()
