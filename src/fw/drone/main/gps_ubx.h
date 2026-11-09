// u-blox UBX framing for the GPS (gps.c): the frame parser, UBX-NAV-PVT's fields, UBX-CFG-VALSET and VALGET requests and their answers, and the UBX-NAV-SAT and MON-RF summaries. Header-only and free of ESP-IDF, so the host test (test/gps_ubx_test.c, make gps-test) runs the same code.
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
#define UBX_NAV_SAT 0x35
#define UBX_ACK 0x05
#define UBX_ACK_ACK 0x01
#define UBX_CFG 0x06
#define UBX_CFG_VALSET 0x8a
#define UBX_CFG_VALGET 0x8b
#define UBX_MON 0x0a
#define UBX_MON_RF 0x38

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

// One configuration item: the key's size bits (28-30) give the value's width, 1 to 8 bytes.
typedef struct {
    uint32_t key;
    uint64_t value;
} UbxSetting;

static inline int ubx_width(uint32_t key) {
    int bits = key >> 28 & 7;
    return bits == 5 ? 8 : bits == 4 ? 4 : bits == 3 ? 2 : 1;
}

// A UBX frame with the given payload, written into out; returns its length, 0 if it does not fit.
static inline size_t ubx_frame(uint8_t message_class, uint8_t id, const uint8_t *payload, size_t length, uint8_t *out, size_t size) {
    if (length + 8 > size) return 0;
    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = message_class;
    out[3] = id;
    out[4] = length & 0xff;
    out[5] = length >> 8;
    if (length) memmove(out + 6, payload, length);
    uint8_t a = 0, b = 0;
    ubx_checksum(out + 2, length + 4, &a, &b);
    out[6 + length] = a;
    out[7 + length] = b;
    return length + 8;
}

// A UBX-CFG-VALSET frame applying settings to the RAM layer (until the module loses power), written into out; returns its length, 0 if it does not fit.
static inline size_t ubx_valset(const UbxSetting *settings, int count, uint8_t *out, size_t size) {
    size_t length = 4;
    for (int i = 0; i < count; i++) length += 4 + ubx_width(settings[i].key);
    if (length + 8 > size) return 0;
    uint8_t *p = out + 6;
    *p++ = 0;
    *p++ = 1;
    *p++ = 0;
    *p++ = 0;
    for (int i = 0; i < count; i++) {
        for (int b = 0; b < 4; b++) *p++ = settings[i].key >> (8 * b) & 0xff;
        for (int b = 0; b < ubx_width(settings[i].key); b++) *p++ = settings[i].value >> (8 * b) & 0xff;
    }
    return ubx_frame(UBX_CFG, UBX_CFG_VALSET, out + 6, length, out, size);
}

// A UBX-CFG-VALGET poll of the settings' keys in the RAM layer, written into out; returns its length, 0 if it does not fit.
static inline size_t ubx_valget(const UbxSetting *settings, int count, uint8_t *out, size_t size) {
    size_t length = 4 + 4 * count;
    if (length + 8 > size) return 0;
    uint8_t *p = out + 6;
    memset(p, 0, 4);
    for (int i = 0; i < count; i++)
        for (int b = 0; b < 4; b++) p[4 + 4 * i + b] = settings[i].key >> (8 * b) & 0xff;
    return ubx_frame(UBX_CFG, UBX_CFG_VALGET, p, length, out, size);
}

// Whether a UBX-CFG-VALGET reply holds every one of the settings at its value (keys in any order).
static inline bool ubx_valget_matches(const UbxFrame *frame, const UbxSetting *settings, int count) {
    if (frame->message_class != UBX_CFG || frame->id != UBX_CFG_VALGET || frame->length < 4 || frame->payload[0] != 1) return false;
    int found = 0;
    for (size_t at = 4; at + 4 <= frame->length;) {
        uint32_t key = ubx_u32(frame->payload + at);
        int width = ubx_width(key);
        if (at + 4 + width > frame->length) return false;
        uint64_t value = 0;
        for (int b = 0; b < width; b++) value |= (uint64_t)frame->payload[at + 4 + b] << (8 * b);
        for (int i = 0; i < count; i++)
            if (settings[i].key == key) {
                if (settings[i].value != value) return false;
                found++;
            }
        at += 4 + width;
    }
    return found == count;
}

// An acknowledgement of a configuration request: true for UBX-ACK-ACK or ACK-NAK of a CFG message, *id that message's ID and *acked whether it was accepted.
static inline bool ubx_ack(const UbxFrame *frame, uint8_t *id, bool *acked) {
    if (frame->message_class != UBX_ACK || frame->length != 2 || frame->id > UBX_ACK_ACK || frame->payload[0] != UBX_CFG) return false;
    *id = frame->payload[1];
    *acked = frame->id == UBX_ACK_ACK;
    return true;
}

// One satellite from UBX-NAV-SAT: the GNSS (0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS), its number (an SBAS one's PRN), C/N0 (dB-Hz), elevation (degrees, beyond +-90 unknown) and flags: bits 0-2 the signal quality (4 code locked, 5-7 carrier locked), 3 used in the solution, 4-5 health (1 healthy, 2 not), 6 differential corrections available, 7 SBAS corrections used.
typedef struct __attribute__((packed)) {
    uint8_t gnss, sv, cno;
    int8_t elevation;
    uint8_t flags;
} UbxSatellite;

// The satellites heard (C/N0 above 0) or used, at most most of them, from a UBX-NAV-SAT; *known is how many it listed in all.
static inline bool ubx_sat(const UbxFrame *frame, uint32_t *itow_ms, UbxSatellite *out, int most, int *count, int *known) {
    if (frame->message_class != UBX_NAV || frame->id != UBX_NAV_SAT || frame->length < 8) return false;
    const uint8_t *p = frame->payload;
    int listed = p[5];
    if (frame->length != 8 + 12 * listed) return false;
    *itow_ms = ubx_u32(p);
    *known = listed;
    *count = 0;
    for (int i = 0; i < listed && *count < most; i++) {
        const uint8_t *s = p + 8 + 12 * i;
        uint32_t flags = ubx_u32(s + 8);
        if (!s[2] && !(flags & 8)) continue;
        out[(*count)++] = (UbxSatellite){.gnss = s[0], .sv = s[1], .cno = s[2], .elevation = (int8_t)s[3], .flags = (flags & 0x7f) | (flags >> 16 & 1) << 7};
    }
    return true;
}

// The L1 band's RF health from UBX-MON-RF: jamming 0 unknown or monitor off, 1 ok, 2 warning, 3 critical; antenna status 0 init, 1 unknown, 2 ok, 3 short, 4 open; antenna power 0 off, 1 on, 2 unknown; noise per millisecond; AGC count (0-8191); the CW jamming indicator (cwSuppression, 0 none to 255 strong).
typedef struct {
    uint8_t jamming, antenna_status, antenna_power, cw_jamming;
    uint16_t noise_per_ms, agc_count;
} UbxRf;

static inline bool ubx_rf(const UbxFrame *frame, UbxRf *rf) {
    if (frame->message_class != UBX_MON || frame->id != UBX_MON_RF || frame->length < 28 || frame->length != 4 + 24 * frame->payload[1]) return false;
    const uint8_t *b = frame->payload + 4;
    *rf = (UbxRf){.jamming = b[1] & 3, .antenna_status = b[2], .antenna_power = b[3], .cw_jamming = b[16],
                  .noise_per_ms = b[12] | b[13] << 8, .agc_count = b[14] | b[15] << 8};
    return true;
}

#endif
