#pragma once
#include <stddef.h>
#include <stdint.h>

// ESP32 is little-endian. The Python receiver uses the same explicit layout.
struct __attribute__((packed)) PosePacket {
    char magic[4] = {'T', 'A', 'G', '1'};
    uint32_t sequence = 0;
    uint32_t captureUs = 0; // Camera boot clock, wraps every ~72 minutes.
    uint32_t ageUs = 0;     // Capture-to-ready time; no clock sync required.
    uint16_t tagId = 0;
    uint16_t valid = 0;
    float position[3] = {}; // Camera origin in tag frame, metres.
    float rotation[9] = {}; // Camera -> tag rotation, row-major.
    float margin = 0;
    float reprojectionPx = 0;
    uint16_t reserved = 0; // Keep DMA transactions a multiple of four bytes.
    uint16_t checksum = 0;
};
static_assert(sizeof(PosePacket) == 80, "Wire format changed");

// CRC-16/CCITT-FALSE over all bytes before checksum (including magic).
inline uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc ^= uint16_t(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

