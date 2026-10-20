"""
Python file updates over the direct link. The native request service (native/runtime/radio.c) reassembles each request, refuses writes unless the drone is idle and caches the reply per request id; native image updates and resets never reach Python.
"""

import hashlib
import os
import struct

buffer = bytearray(1024)


def file_hash(path):
    digest = hashlib.sha256()
    try:
        stream = open(path, "rb")
    except OSError as error:
        if error.errno == 2:
            return bytes(32)
        raise
    with stream:
        while True:
            count = stream.readinto(buffer)
            if not count:
                return digest.digest()
            digest.update(memoryview(buffer)[:count])


def handle(request):
    """
    F: SHA-256 of each newline-separated path. P: append data at an offset to <path>.upload. M: check <path>.upload's hash and rename it. Y: up to 900 bytes of a file from an offset. L: a directory's names. K: remove a file if it exists.
    """
    op, body = request[0], request[1:]
    if op == ord("L"):
        return "\n".join(os.listdir(bytes(body).decode())).encode()
    if op == ord("K"):
        try:
            os.remove(bytes(body).decode())
        except OSError as error:
            if error.errno != 2:
                raise
        return b""
    if op == ord("Y"):
        offset = struct.unpack_from("<I", body)[0]
        with open(bytes(body[4:]).decode(), "rb") as stream:
            stream.seek(offset)
            return stream.read(900)
    if op == ord("F"):
        return b"".join(file_hash(path) for path in bytes(body).decode().split("\n"))
    if op == ord("P"):
        offset, length = struct.unpack_from("<IB", body)
        path = bytes(body[5:5 + length]).decode() + ".upload"
        if offset == 0:
            parts = path.split("/")[:-1]
            for index in range(1, len(parts) + 1):
                try:
                    os.mkdir("/".join(parts[:index]))
                except OSError as error:
                    if error.errno != 17:
                        raise
        elif os.stat(path)[6] != offset:
            raise ValueError(f"{path} is not {offset} bytes long")
        with open(path, "ab" if offset else "wb") as stream:
            stream.write(body[5 + length:])
        return b""
    expected, path = bytes(body[:32]), bytes(body[32:]).decode()
    if file_hash(path + ".upload") != expected:
        raise ValueError(f"{path} verification failed")
    os.rename(path + ".upload", path)
    if path.endswith(".mpy"):
        # MicroPython imports a .py before the .mpy beside it.
        try:
            os.remove(path[:-4] + ".py")
        except OSError as error:
            if error.errno != 2:
                raise
    return b""
