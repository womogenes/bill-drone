"""
FPV over the radio received by a USB Wi-Fi adapter in monitor mode (a TP-Link Archer T2U Plus, RTL8821AU, on the vendor driver morrownr/8821au-20210708 through DKMS, which heard more than the kernel's rtw88_8821au: 77% of beacons at -57 dBm against 55%), in place of the ESP32 receiver (fw/fpv_rx) and its 98 KB/s USB link.

sudo python3 src/sw/fpv_monitor.py [--interface wlan1] [--channel 11]   (or the bill-fpv-monitor service, src/sw/fpv-monitor.service)

Needs root, so it does as little as it can and the dashboard server (sw/dashboard/app/api/camera/route.ts) does the rest: it turns the interface into a monitor on the channel the camera parks on (fw/esp32cam/src/ota.h radioChannel), with NetworkManager told to leave it alone, and passes every camera shard it hears (broadcast vendor action frames with BSSID FPVRAW, fw/esp32cam/src/fpv.h) to 127.0.0.1:5602 as FPVC, the signal (dBm, signed byte) and the shard from its FPV2 header; FPVM every 500 ms says the monitor is running, which turns the dashboard's FPV to the radio.
"""

import argparse
import socket
import struct
import subprocess
import time

BSSID = b"FPVRAW"
ACTION = b"\x7f" + b"FPV"
DASHBOARD = ("127.0.0.1", 5602)
# Radiotap fields before the antenna signal (5): their sizes and alignments.
FIELDS = {0: (8, 8), 1: (1, 1), 2: (1, 1), 3: (4, 2), 4: (2, 2), 5: (1, 1)}
FLAG_FCS = 0x10
RETUNE_S = 3.0


def monitor(interface, channel):
    subprocess.run(["nmcli", "device", "set", interface, "managed", "no"], check=False)
    subprocess.run(["ip", "link", "set", interface, "down"], check=True)
    subprocess.run(["iw", "dev", interface, "set", "type", "monitor"], check=True)
    subprocess.run(["ip", "link", "set", interface, "up"], check=True)
    subprocess.run(["iw", "dev", interface, "set", "channel", str(channel), "HT20"], check=True)


def radiotap(header):
    """The flags byte and the antenna signal (dBm) from a radiotap header, 0 when absent."""
    present = struct.unpack_from("<I", header, 4)[0]
    offset = 8
    word = present
    while word & 0x80000000:
        word = struct.unpack_from("<I", header, offset)[0]
        offset += 4
    values = {}
    for field in range(6):
        if not present & (1 << field):
            continue
        size, align = FIELDS[field]
        offset = (offset + align - 1) // align * align
        values[field] = header[offset:offset + size]
        offset += size
    flags = values.get(1, b"\0")[0]
    signal = struct.unpack("b", values[5])[0] if 5 in values else 0
    return flags, signal


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--interface", default="wlan1")
    parser.add_argument("--channel", type=int, default=11)
    args = parser.parse_args()
    monitor(args.interface, args.channel)
    raw = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(3))
    raw.bind((args.interface, 0))
    raw.settimeout(0.5)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    hello, heard, tuned = 0.0, time.monotonic(), time.monotonic()
    print(f"monitoring {args.interface} on channel {args.channel}", flush=True)
    while True:
        now = time.monotonic()
        if now - hello >= 0.5:
            out.sendto(b"FPVM", DASHBOARD)
            hello = now
        # The adapter sometimes keeps reporting the channel while hearing nothing from the camera on it (2026-10-30, twice), and setting the same channel again does not help; tuning away and back does. So without a camera chunk for RETUNE_S it goes through another channel (harmless when the camera is simply not streaming).
        # After a reboot the adapter came back managed (2026-10-31, NetworkManager first), where setting a channel only failed busy: then monitor mode is set again.
        if now - heard > RETUNE_S and now - tuned > RETUNE_S:
            if "type monitor" not in subprocess.run(["iw", "dev", args.interface, "info"], capture_output=True, text=True).stdout:
                monitor(args.interface, args.channel)
            subprocess.run(["iw", "dev", args.interface, "set", "channel", str(1 if args.channel != 1 else 6), "HT20"], check=False)
            subprocess.run(["iw", "dev", args.interface, "set", "channel", str(args.channel), "HT20"], check=False)
            tuned = now
        try:
            packet = raw.recv(4096)
        except socket.timeout:
            continue
        length = struct.unpack_from("<H", packet, 2)[0]
        frame = packet[length:]
        if len(frame) < 28 + 16 or frame[0] != 0xD0 or frame[16:22] != BSSID or frame[24:28] != ACTION:
            continue
        flags, signal = radiotap(packet[:length])
        chunk = frame[28:-4] if flags & FLAG_FCS else frame[28:]
        if chunk[:4] == b"FPV2":
            heard = time.monotonic()
            out.sendto(b"FPVC" + struct.pack("b", signal) + chunk, DASHBOARD)


if __name__ == "__main__":
    main()
