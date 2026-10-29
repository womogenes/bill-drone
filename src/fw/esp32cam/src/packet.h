#pragma once
#include <stddef.h>
#include <stdint.h>

// ESP32 is little-endian. The drone's receiver (fw/drone/main/camera.c) uses the same explicit layout.
struct __attribute__((packed)) PosePacket {
    char magic[4] = {'T', 'A', 'G', '3'};
    uint32_t sequence = 0;
    uint32_t captureUs = 0; // CAM clock (wraps every ~72 minutes): readout of the tag's centre row, else the frame's.
    uint32_t ageUs = 0;     // Capture-to-ready time; no clock sync required.
    uint16_t tagId = 0;
    uint16_t valid = 0;     // 0 none, 1 unambiguous, 2 ambiguous tilt resolved by continuity.
    float position[3] = {}; // Camera origin in tag frame, metres.
    float rotation[9] = {}; // Camera -> tag rotation, row-major.
    float margin = 0;
    float reprojectionPx = 0;
    float flow[2] = {};           // Image shift from the previous frame, pixels right and down.
    uint32_t flowIntervalUs = 0;  // Between the two frames' readouts; 0 when there is no flow.
    float flowResidualPx = 0;     // Median deviation of the matched blocks from the median shift.
    uint16_t flowInliers = 0;     // Blocks that agreed.
    int16_t flowOffsetUs = 0;     // This frame's centre-row readout minus captureUs.
    float tagPosition[3] = {};    // Tag centre in the camera frame, metres: well determined even when the tilt is not.
    float altPosition[3] = {};    // The other planar solution, as position and rotation.
    float altRotation[9] = {};
    float altReprojectionPx = 0;
    uint16_t trusted = 0;         // 1: the pose fields hold both solutions, though valid may be 0 (ambiguous tilt).
    uint16_t checksum = 0;
};
static_assert(sizeof(PosePacket) == 164, "Wire format changed");

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


// Firmware updates through the drone (update.h): the drone sends a request, header then body then a CRC-16 over both, and the camera answers each with one reply.
struct __attribute__((packed)) UpdateRequest {
    char magic[4];          // CUP1
    uint32_t id = 0;        // echoed in the reply
    uint8_t op = 0;
    uint8_t reserved = 0;
    uint16_t length = 0;    // of the body that follows
};
static_assert(sizeof(UpdateRequest) == 12, "Update request header changed");

struct __attribute__((packed)) UpdateReply {
    char magic[4] = {'C', 'U', 'R', '1'};
    uint32_t id = 0;
    uint8_t status = 0;     // 0 done, 1 failed
    uint8_t length = 0;     // of text
    char text[52] = {};     // the answer, or why it failed
    uint16_t checksum = 0;
};
static_assert(sizeof(UpdateReply) == 64, "Update reply changed");
