"""
ESP-NOW envelopes and zero-delimited USB frames.
"""

import binascii
import struct

HEADER = "<3sBIIIBB"
HEADER_SIZE = 18
PAYLOAD_SIZE = 232


def pack(channel, session, sequence, token, payload, index=0, count=1):
    return struct.pack(HEADER, b"DC1", channel, session, sequence, token, index, count) + payload


def unpack(packet):
    if len(packet) < HEADER_SIZE or len(packet) > 250:
        raise ValueError("Invalid direct packet size")
    magic, channel, session, sequence, token, index, count = struct.unpack_from(HEADER, packet)
    if magic != b"DC1" or not 0 <= channel <= 6 or not 0 <= index < count <= 32:
        raise ValueError("Invalid direct packet header")
    return channel, session, sequence, token, index, count, packet[HEADER_SIZE:]


def serial_encode(packet):
    data = packet + struct.pack("<I", binascii.crc32(packet) & 0xFFFFFFFF)
    encoded = bytearray([0])
    position = 0
    code = 1
    for value in data:
        if value == 0:
            encoded[position] = code
            position = len(encoded)
            encoded.append(0)
            code = 1
        else:
            encoded.append(value)
            code += 1
            if code == 255:
                encoded[position] = code
                position = len(encoded)
                encoded.append(0)
                code = 1
    encoded[position] = code
    encoded.append(0)
    return encoded


class SerialDecoder:
    def __init__(self):
        self.pending = bytearray()
        self.discard = False

    def feed(self, data):
        packets = []
        for value in data:
            if value:
                if not self.discard:
                    self.pending.append(value)
                    if len(self.pending) > 260:
                        self.pending = bytearray()
                        self.discard = True
                continue
            if not self.discard and self.pending:
                try:
                    decoded = bytearray()
                    offset = 0
                    while offset < len(self.pending):
                        code = self.pending[offset]
                        end = offset + code
                        if end > len(self.pending):
                            raise ValueError("Truncated COBS block")
                        decoded.extend(self.pending[offset + 1:end])
                        offset = end
                        if code < 255 and offset < len(self.pending):
                            decoded.append(0)
                    if len(decoded) < HEADER_SIZE + 4:
                        raise ValueError("Truncated USB packet")
                    packet = bytes(decoded[:-4])
                    crc = struct.unpack("<I", decoded[-4:])[0]
                    if binascii.crc32(packet) & 0xFFFFFFFF == crc:
                        unpack(packet)
                        packets.append(packet)
                except (ValueError, IndexError):
                    pass
            self.pending = bytearray()
            self.discard = False
        return packets
