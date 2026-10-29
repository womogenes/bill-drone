#ifndef CAMERA_LINK_H
#define CAMERA_LINK_H

// The TAG3 packet and the byte-stream parser for the camera link, with no ESP-IDF dependencies so the host test (fw/drone/test/camera_link_test.c) runs the same code as camera.c.
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t sequence, capture_us, age_us;
    uint16_t tag_id, valid;
    float position[3], rotation[9], margin, reprojection_px, flow[2];
    uint32_t flow_interval_us;
    float flow_residual_px;
    uint16_t flow_inliers;
    int16_t flow_offset_us;
    float tag_position[3], alt_position[3], alt_rotation[9], alt_reprojection_px;
    uint16_t trusted, checksum;
} CameraPacket;
_Static_assert(sizeof(CameraPacket) == 164, "TAG3 layout");

// CRC-16/CCITT-FALSE, as in the camera's packet.h.
static inline uint16_t camera_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i] << 8;
        for (int bit = 0; bit < 8; bit++) crc = crc & 0x8000 ? crc << 1 ^ 0x1021 : crc << 1;
    }
    return crc;
}

// The bytes since the last packet that could still start one. skipped counts bytes thrown away (the camera's boot messages, line noise, the remains of a damaged packet); corrupt counts full-length candidates whose CRC failed.
typedef struct {
    uint8_t bytes[sizeof(CameraPacket)];
    size_t have;
    uint32_t skipped, corrupt;
} CameraParser;

// Drops the first n bytes, then any more until what is left could be the start of "TAG3".
static inline void camera_parser_drop(CameraParser *parser, size_t n) {
    for (;;) {
        memmove(parser->bytes, parser->bytes + n, parser->have - n);
        parser->have -= n;
        parser->skipped += n;
        n = 0;
        while (n < parser->have && memcmp(parser->bytes + n, "TAG3", parser->have - n < 4 ? parser->have - n : 4)) n++;
        if (!n) return;
    }
}

// Consumes input from *data (advancing *data and *length) until a packet with the magic, full length and a matching CRC is complete: copies it to packet and returns true. Returns false once the input is used up, keeping a partial packet for the next call. After a CRC failure the search resumes one byte past the false start, so a real packet that began inside it is still found.
static inline bool camera_parse(CameraParser *parser, const uint8_t **data, size_t *length, CameraPacket *packet) {
    while (*length) {
        // Outside a packet, skip quickly to the next possible start.
        if (!parser->have) {
            const uint8_t *start = memchr(*data, 'T', *length);
            size_t gap = start ? (size_t)(start - *data) : *length;
            parser->skipped += gap;
            *data += gap;
            *length -= gap;
            if (!*length) return false;
        }
        size_t take = sizeof(CameraPacket) - parser->have;
        if (take > *length) take = *length;
        memcpy(parser->bytes + parser->have, *data, take);
        parser->have += take;
        *data += take;
        *length -= take;
        size_t valid_prefix = parser->have < 4 ? parser->have : 4;
        if (memcmp(parser->bytes, "TAG3", valid_prefix)) {
            camera_parser_drop(parser, 1);
            continue;
        }
        if (parser->have < sizeof(CameraPacket)) continue;
        if (camera_crc16(parser->bytes, offsetof(CameraPacket, checksum)) ==
            (uint16_t)(parser->bytes[sizeof(CameraPacket) - 2] | parser->bytes[sizeof(CameraPacket) - 1] << 8)) {
            memcpy(packet, parser->bytes, sizeof(CameraPacket));
            parser->have = 0;
            return true;
        }
        parser->corrupt++;
        camera_parser_drop(parser, 1);
    }
    return false;
}

// The camera raises SYNC as it queues each packet, then sends it: a packet completed within CAMERA_SYNC_WINDOW_US of the newest unused rising edge belongs to that edge (the 0.8 ms transfer plus this drone's UART and task latency; packets are about 34 ms apart). edges counts rising edges, edge_us is the newest one's time and used_edges the count already matched; all times are the low 32 bits of esp_timer. Returns whether the packet is dated, and how long before now its edge was.
#define CAMERA_SYNC_WINDOW_US 10000
static inline bool camera_sync(uint32_t now_us, uint32_t edges, uint32_t edge_us, uint32_t *used_edges, uint32_t *since_edge_us) {
    *since_edge_us = now_us - edge_us;
    if (edges == *used_edges || *since_edge_us > CAMERA_SYNC_WINDOW_US) return false;
    *used_edges = edges;
    return true;
}

// Camera firmware updates through this link (fw/esp32cam/src/update.h): a request is this header, the body and a CRC-16 over both; the camera answers each with one CameraUpdateReply.
typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t id;
    uint8_t op, reserved;
    uint16_t length;
} CameraUpdateRequest;
_Static_assert(sizeof(CameraUpdateRequest) == 12, "CUP1 layout");

typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t id;
    uint8_t status, length;
    char text[52];
    uint16_t checksum;
} CameraUpdateReply;
_Static_assert(sizeof(CameraUpdateReply) == 64, "CUR1 layout");

typedef struct {
    uint8_t bytes[sizeof(CameraUpdateReply)];
    size_t have;
} CameraReplyParser;

// Consumes input byte by byte until a reply with the magic and a matching CRC is complete: copies it to reply and returns true, leaving the rest of the input for the next call.
static inline bool camera_reply_parse(CameraReplyParser *parser, const uint8_t **data, size_t *length, CameraUpdateReply *reply) {
    while (*length) {
        uint8_t b = **data;
        (*data)++;
        (*length)--;
        if (parser->have < 4) {
            if (b == (uint8_t)"CUR1"[parser->have]) parser->bytes[parser->have++] = b;
            else parser->have = b == 'C' ? (parser->bytes[0] = b, 1) : 0;
            continue;
        }
        parser->bytes[parser->have++] = b;
        if (parser->have < sizeof(CameraUpdateReply)) continue;
        parser->have = 0;
        if (camera_crc16(parser->bytes, offsetof(CameraUpdateReply, checksum)) ==
            (uint16_t)(parser->bytes[sizeof(CameraUpdateReply) - 2] | parser->bytes[sizeof(CameraUpdateReply) - 1] << 8)) {
            memcpy(reply, parser->bytes, sizeof(CameraUpdateReply));
            return true;
        }
    }
    return false;
}

#endif
