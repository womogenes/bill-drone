"""
Stream an authenticated WebREPL upload into the inactive application slot.
"""

import asyncio
import esp32
import io
import os

BLOCK_SIZE = 4096


class Image(io.IOBase):
    def __init__(self, partition, size):
        self.partition = partition
        self.size = size
        self.received = 0
        self.block = 0
        self.buffer = bytearray()
        self.readback = bytearray(BLOCK_SIZE)
        self.finished_upload = False

    def write(self, data):
        if self.finished_upload or self.received + len(data) > self.size:
            raise OSError("Unexpected OTA data")
        self.received += len(data)
        self.buffer.extend(data)
        while len(self.buffer) >= BLOCK_SIZE:
            self.write_block(self.buffer[:BLOCK_SIZE])
            del self.buffer[:BLOCK_SIZE]
        return len(data)

    def write_block(self, data):
        self.partition.writeblocks(self.block, data)
        self.partition.readblocks(self.block, self.readback)
        if self.readback != data:
            raise ValueError("OTA chunk readback mismatch")
        self.block += 1

    def ioctl(self, request, argument):
        if request == 4:  # MicroPython stream close.
            self.finished_upload = True
        return 0

    def finish(self):
        if not self.finished_upload or self.received != self.size:
            raise ValueError("OTA upload is incomplete")
        if self.buffer:
            self.buffer.extend(b"\xff" * (BLOCK_SIZE - len(self.buffer)))
            self.write_block(self.buffer)
            self.buffer = bytearray()

        # ESP-IDF validates boot compatibility when selecting the new slot.
        self.partition.set_boot()


class Upload:
    def __init__(self, size):
        running = esp32.Partition(esp32.Partition.RUNNING)
        try:
            self.partition = running.get_next_update()
        except OSError:
            raise ValueError("No OTA partition; migrate over USB first")
        info = self.partition.info()
        if info[2] == running.info()[2] or not 0x10 <= info[1] <= 0x1F:
            raise ValueError("No inactive OTA partition; migrate over USB first")
        if not 0 < size <= info[3]:
            raise ValueError("Image exceeds OTA partition")
        self.image = Image(self.partition, size)
        self.opened = False

    def mount(self, readonly, mkfs):
        pass

    def umount(self):
        pass

    def stat(self, path):
        if path in ("", "/"):
            return (0x4000, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        raise OSError(2)

    def open(self, path, mode):
        if path.lstrip("/") != "firmware.bin" or mode != "wb" or self.opened:
            raise OSError("OTA accepts one firmware.bin upload")
        self.opened = True
        return self.image

    def finish(self):
        self.image.finish()
        os.umount("/ota")


def mount(size):
    upload = Upload(size)
    os.mount(upload, "/ota")
    return upload


async def confirm_boot(app):
    """
    Accept an OTA image after native initialization and server startup succeed.
    """
    subtype = esp32.Partition(esp32.Partition.RUNNING).info()[1]
    if not 0x10 <= subtype <= 0x1F:
        return
    while app.server is None:
        await asyncio.sleep_ms(20)
    await asyncio.sleep_ms(1000)
    esp32.Partition.mark_app_valid_cancel_rollback()
