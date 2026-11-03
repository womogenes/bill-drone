// u-blox UBX framing for the GPS (gps.c): the frame parser, UBX-NAV-PVT's fields and UBX-CFG-VALSET requests. Header-only and free of ESP-IDF, so the host test (test/gps_ubx_test.c, make gps-test) runs the same code.
#ifndef GPS_UBX_H
#define GPS_UBX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define UBX_SYNC1 0xb5
#define UBX_SYNC2 0x62
// The longest payload kept: NAV-SAT is 8 + 12 bytes per satellite, about 600 for 50.
#define UBX_MOST 1024
#define UBX_NAV 0x01
#define UBX_NAV_PVT 0x07
#define UBX_PVT_SIZE 92
#define UBX_ACK 0x05
#define UBX_CFG 0x06
#define UBX_CFG_VALSET 0x8a

typedef struct {
    uint8_t message_class, id;
    uint16_t length;
    uint8_t payload[UBX_MOST];
} UbxFrame;

// Bytes that could not start a frame (the module's NMEA text before it is configured, noise) are counted in skipped; frames whose checksum failed, or longer than UBX_MOST, in corrupt.
typedef struct {
    int state;
    uint16_t at;
    uint8_t check_a, check_b;
    UbxFrame frame;
    uint32_t skipped, corrupt;
} UbxParser;

static inline void ubx_checksum(const uint8_t *data, size_t length, uint8_t *a, uint8_t *b) {
    for (size_t i = 0; i < length; i++) {
        *a += data[i];
        *b += *a;
    }
}

// Consumes bytes until a frame completes (true, with *data and *length advanced past it) or the input runs out (false).
static inline bool ubx_parse(UbxParser *parser, const uint8_t **data, size_t *length, UbxFrame *out) {
    while (*length) {
        uint8_t byte = **data;
        (*data)++;
        (*length)--;
        UbxFrame *frame = &parser->frame;
        switch (parser->state) {
        case 0:
            if (byte == UBX_SYNC1) parser->state = 1;
            else parser->skipped++;
            break;
        case 1:
            if (byte == UBX_SYNC2) parser->state = 2;
            else if (byte == UBX_SYNC1) parser->skipped++;
            else {
                parser->skipped += 2;
                parser->state = 0;
            }
            break;
        case 2:
            frame->message_class = byte;
            parser->check_a = parser->check_b = 0;
            ubx_checksum(&byte, 1, &parser->check_a, &parser->check_b);
            parser->state = 3;
            break;
        case 3:
            frame->id = byte;
            ubx_checksum(&byte, 1, &parser->check_a, &parser->check_b);
            parser->state = 4;
            break;
        case 4:
            frame->length = byte;
            ubx_checksum(&byte, 1, &parser->check_a, &parser->check_b);
            parser->state = 5;
            break;
        case 5:
            frame->length |= byte << 8;
            ubx_checksum(&byte, 1, &parser->check_a, &parser->check_b);
            parser->at = 0;
            if (frame->length > UBX_MOST) {
                parser->corrupt++;
                parser->state = 0;
            } else parser->state = frame->length ? 6 : 7;
            break;
        case 6:
            frame->payload[parser->at++] = byte;
            ubx_checksum(&byte, 1, &parser->check_a, &parser->check_b);
            if (parser->at == frame->length) parser->state = 7;
            break;
        case 7:
            if (byte == parser->check_a) parser->state = 8;
            else {
                parser->corrupt++;
                parser->state = byte == UBX_SYNC1;
            }
            break;
        default:
            parser->state = 0;
            if (byte == parser->check_b) {
                *out = *frame;
                return true;
            }
            parser->corrupt++;
            parser->state = byte == UBX_SYNC1;
            break;
        }
    }
    return false;
}

static inline uint32_t ubx_u32(const uint8_t *p) {
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

// The NAV-PVT fields the drone uses, in the message's own units.
typedef struct {
    // GPS time of week (ms); fix_type 0 none, 2 2D, 3 3D; flags bit 0 gnssFixOK, bits 6-7 carrier solution.
    uint32_t itow_ms;
    uint8_t fix_type, flags, satellites;
    // Degrees times 1e7; height above mean sea level (mm).
    int32_t longitude, latitude, height_mm;
    // North, east, down (mm/s).
    int32_t velocity_mm_s[3];
    // Horizontal, vertical and speed accuracy estimates (mm, mm/s).
    uint32_t horizontal_accuracy_mm, vertical_accuracy_mm, speed_accuracy_mm_s;
} UbxPvt;

static inline bool ubx_pvt(const UbxFrame *frame, UbxPvt *pvt) {
    if (frame->message_class != UBX_NAV || frame->id != UBX_NAV_PVT || frame->length != UBX_PVT_SIZE) return false;
    const uint8_t *p = frame->payload;
    pvt->itow_ms = ubx_u32(p);
    pvt->fix_type = p[20];
    pvt->flags = p[21];
    pvt->satellites = p[23];
    pvt->longitude = (int32_t)ubx_u32(p + 24);
    pvt->latitude = (int32_t)ubx_u32(p + 28);
    pvt->height_mm = (int32_t)ubx_u32(p + 36);
    pvt->horizontal_accuracy_mm = ubx_u32(p + 40);
    pvt->vertical_accuracy_mm = ubx_u32(p + 44);
    for (int i = 0; i < 3; i++) pvt->velocity_mm_s[i] = (int32_t)ubx_u32(p + 48 + 4 * i);
    pvt->speed_accuracy_mm_s = ubx_u32(p + 68);
    return true;
}

// One configuration item: the key's size bits (28-30) give the value's width, 1 to 4 bytes.
typedef struct {
    uint32_t key, value;
} UbxSetting;

// A UBX-CFG-VALSET frame applying settings to the RAM layer (until the module loses power), written into out; returns its length, 0 if it does not fit.
static inline size_t ubx_valset(const UbxSetting *settings, int count, uint8_t *out, size_t size) {
    size_t length = 4;
    for (int i = 0; i < count; i++) {
        int bits = settings[i].key >> 28 & 7;
        length += 4 + (bits == 4 ? 4 : bits == 3 ? 2 : 1);
    }
    if (length + 8 > size) return 0;
    uint8_t *p = out;
    *p++ = UBX_SYNC1;
    *p++ = UBX_SYNC2;
    *p++ = UBX_CFG;
    *p++ = UBX_CFG_VALSET;
    *p++ = length & 0xff;
    *p++ = length >> 8;
    *p++ = 0;
    *p++ = 1;
    *p++ = 0;
    *p++ = 0;
    for (int i = 0; i < count; i++) {
        int bits = settings[i].key >> 28 & 7, width = bits == 4 ? 4 : bits == 3 ? 2 : 1;
        for (int b = 0; b < 4; b++) *p++ = settings[i].key >> (8 * b) & 0xff;
        for (int b = 0; b < width; b++) *p++ = settings[i].value >> (8 * b) & 0xff;
    }
    uint8_t a = 0, b = 0;
    ubx_checksum(out + 2, p - out - 2, &a, &b);
    *p++ = a;
    *p++ = b;
    return p - out;
}

#endif
