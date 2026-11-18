# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial>=3.5"]
# ///
"""
Finds the controller's USB serial device by listening: a controller sends its link report in the bridge's framing (a CRC-checked COBS frame with the DC1 header) once a second, whether or not it hears a drone, and nothing else on USB does (the FPV receivers send FPVR/FPVS beacons and raw shards, the drone's flight controller a text console). The device names follow plug order (the controller came up as each of /dev/ttyUSB0, 1 and 3 between 2026-10-30 and 2026-11-14), and the boards' USB chips all carry the same serial number (0001), so neither names it; /dev/serial/by-path names the physical USB port, which the bridge prefers when it looks again. No board is reset and nothing is written: until 2026-11-10 this asked each board's bootloader for its MAC, which reset every ESP32 on USB, the drone too, each time the bridge's service restarted.

A device another process holds is passed over without being opened: one with a flock (an esptool upload, another bridge) or one any other process has open (the dashboard's FPV receivers and its 5 s look at every new port, a serial monitor on the flight controller's console). Opening it would assert its reset lines for a moment and, while the other reader runs, split the bytes between the two of us, so the dashboard lost video and the bridge heard no whole packet.

The bridge (bridge.py) looks for its controller with this whenever it has none. Alone it prints the device, or nothing when no controller is heard: uv run src/fw/controller/find_port.py
"""

import fcntl
import glob
import os
import struct
import termios
import time

import serial

from wire import SerialDecoder, unpack

BAUD = 921600
# How long a probe listens (s): the relay sends its link report once a second (fw/link.h LINK_REPORT_US, checked every 50 ms), and opening the port flushes what came before, so one second could fall between two reports.
LISTEN_S = 1.5
# Physical USB ports by their place on the bus; the same device has a usb- and a usbv2- name.
BY_PATH = "/dev/serial/by-path"


class QuietSerial(serial.Serial):
    """
    A serial port whose DTR and RTS are released together. Linux asserts both when a port opens, which leaves an ESP32's auto-reset circuit (and an S3's USB serial) running; pyserial then clears DTR and RTS in two requests, and between them DTR off with RTS on pulls EN low, a reset. One TIOCMBIC for both (a single CP2102 or CDC control request) never passes through that state.
    """

    def release_lines(self):
        fcntl.ioctl(self.fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))

    def _update_dtr_state(self):
        if self._dtr_state or self._rts_state:
            super()._update_dtr_state()
        else:
            self.release_lines()

    def _update_rts_state(self):
        if self._dtr_state or self._rts_state:
            super()._update_rts_state()
        else:
            self.release_lines()


def open_port(path, baud=BAUD):
    """The device opened for the bridge: non-blocking, locked against a second bridge and the dashboard's receiver search (which skips flocked ports), its reset lines released (pyserial's defaults hold the board in reset)."""
    port = QuietSerial()
    port.port, port.baudrate = path, baud
    port.timeout = port.write_timeout = 0
    port.exclusive = True
    port.dtr = port.rts = False
    port.open()
    return port


def listen(path, seconds=LISTEN_S, baud=BAUD, busy=False):
    """The device, open, if a controller's packet arrives on it within seconds; else None (another board, a controller still booting, or a device something else holds: with busy, one that cannot be opened raises instead)."""
    try:
        port = open_port(path, baud)
    except (OSError, serial.SerialException):
        if busy:
            raise
        return None
    decoder = SerialDecoder()
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            for packet in decoder.feed(port.read(4096)):
                try:
                    unpack(packet)
                except ValueError:
                    continue
                return port
            time.sleep(0.02)
    except (OSError, serial.SerialException):
        pass
    port.close()
    return None


def candidates():
    """Every USB serial device, by its /dev name: the by-path links' targets and every ttyUSB and ttyACM, each once."""
    devices = {os.path.realpath(path) for path in glob.glob(f"{BY_PATH}/*")}
    devices |= {os.path.realpath(path) for path in glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")}
    return sorted(device for device in devices if os.path.exists(device))


def by_path(device):
    """The device's physical USB port names (BY_PATH links to it), sorted; empty for one with none (a pty in the tests)."""
    device = os.path.realpath(device)
    return sorted(path for path in glob.glob(f"{BY_PATH}/*") if os.path.realpath(path) == device)


def holder(path):
    """
    Why another process holds the device, or None: "locked" (a flock on it in /proc/locks, by any other process) or "open" (another process has it open, from /proc/*/fd; processes of other users are not visible and do not count).
    """
    try:
        info = os.stat(path)
        device = os.path.realpath(path)
    except OSError:
        return None
    me = os.getpid()
    try:
        with open("/proc/locks", encoding="latin-1") as file:
            for line in file:
                fields = line.split()
                if len(fields) < 6 or fields[1] != "FLOCK":
                    continue
                pid, (major, minor, inode) = fields[4], fields[5].split(":")
                if int(inode) == info.st_ino and int(major, 16) == os.major(info.st_dev) and int(minor, 16) == os.minor(info.st_dev) and pid != str(me):
                    return "locked"
    except (OSError, ValueError):
        pass
    for directory in glob.glob("/proc/[0-9]*/fd"):
        if directory == f"/proc/{me}/fd":
            continue
        try:
            for name in os.listdir(directory):
                try:
                    if os.readlink(f"{directory}/{name}") == device:
                        return "open"
                except OSError:
                    continue
        except OSError:
            continue
    return None


if __name__ == "__main__":
    for path in candidates():
        if holder(path) is not None:
            continue
        port = listen(path)
        if port is not None:
            port.close()
            print(path)
            break
