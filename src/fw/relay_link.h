// The radio relay over the drone's wire to the camera (radio_via_camera, fw/drone/main/radio.c and fw/esp32cam/src/relay.h): the drone's ESP-NOW packets go out through the camera's radio and its u.FL dipole, and what the camera hears comes back the same way. Each frame is "CRL1", a type, a flags byte, the payload length (u16), the payload and a CRC-16/CCITT-FALSE over all of it, so it shares the wire with TAG3 packets and CUP1/CUR1 update traffic; each parser skips the others' bytes. The magic is not the controller relay's RLY1 link report (fw/link.h), which is a different thing on the radio. Types: S drone -> camera, a packet to send to the controller (flags: the rate, a fw/link.h link_rates index); R camera -> drone, a packet heard (the sender's MAC, its RSSI and the noise floor as int8 dBm, then the packet); A camera -> drone, one send finished (payload: 0 delivered, 1 failed), one per S in order. Plain C with no ESP-IDF dependencies, so the host test (fw/drone/test/relay_link_test.c) and the camera's C++ share it.
#ifndef RELAY_LINK_H
#define RELAY_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define RELAY_HEADER 8
// An ESP-NOW packet (250 bytes) after the sender's MAC, RSSI and noise floor.
#define RELAY_PAYLOAD (8 + 250)
#define RELAY_FRAME (RELAY_HEADER + RELAY_PAYLOAD + 2)

static inline uint16_t relay_crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int bit = 0; bit < 8; bit++) crc = crc & 0x8000 ? (uint16_t)(crc << 1 ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// Writes one frame into out (at least RELAY_FRAME bytes) and returns its length, or 0 if the payload is too long.
static inline size_t relay_frame(uint8_t *out, uint8_t type, uint8_t flags, const uint8_t *payload, size_t length) {
    if (length > RELAY_PAYLOAD) return 0;
    memcpy(out, "CRL1", 4);
    out[4] = type;
    out[5] = flags;
    out[6] = (uint8_t)(length & 0xff);
    out[7] = (uint8_t)(length >> 8);
    if (length) memcpy(out + RELAY_HEADER, payload, length);
    uint16_t crc = relay_crc16(out, RELAY_HEADER + length);
    out[RELAY_HEADER + length] = (uint8_t)(crc & 0xff);
    out[RELAY_HEADER + length + 1] = (uint8_t)(crc >> 8);
    return RELAY_HEADER + length + 2;
}

// bytes holds what could still be or contain a frame; done is the length of the frame returned last, removed at the next call. frames counts good ones and corrupt those whose CRC failed or whose length was impossible.
typedef struct {
    uint8_t bytes[RELAY_FRAME];
    size_t have, done;
    uint32_t frames, corrupt;
} RelayParser;

// Drops the first n bytes, then any more until what is left could be the start of "CRL1".
static inline void relay_parser_drop(RelayParser *parser, size_t n) {
    while (n < parser->have && memcmp(parser->bytes + n, "CRL1", parser->have - n < 4 ? parser->have - n : 4)) n++;
    if (n > parser->have) n = parser->have;
    memmove(parser->bytes, parser->bytes + n, parser->have - n);
    parser->have -= n;
}

// The length of a good frame at the start of bytes, or 0 while one could still be completing; false starts and damaged frames are dropped one byte past their start, so a frame that began inside one is still found.
static inline size_t relay_parser_check(RelayParser *parser) {
    while (parser->have) {
        size_t prefix = parser->have < 4 ? parser->have : 4;
        if (memcmp(parser->bytes, "CRL1", prefix)) {
            relay_parser_drop(parser, 1);
            continue;
        }
        if (parser->have < RELAY_HEADER) return 0;
        size_t payload = parser->bytes[6] | (size_t)parser->bytes[7] << 8;
        size_t total = RELAY_HEADER + payload + 2;
        if (payload > RELAY_PAYLOAD) {
            parser->corrupt++;
            relay_parser_drop(parser, 1);
            continue;
        }
        if (parser->have < total) return 0;
        if (relay_crc16(parser->bytes, total - 2) == (uint16_t)(parser->bytes[total - 2] | parser->bytes[total - 1] << 8)) {
            parser->frames++;
            return total;
        }
        parser->corrupt++;
        relay_parser_drop(parser, 1);
    }
    return 0;
}

// Consumes input from *data (advancing *data and *length) until a frame with the magic, a possible length and a matching CRC is complete: returns its total length (header, payload and CRC, at the start of parser->bytes until the next call). Returns 0 once the input is used up, keeping a partial frame for the next call.
static inline size_t relay_parse(RelayParser *parser, const uint8_t **data, size_t *length) {
    for (;;) {
        if (parser->done) {
            memmove(parser->bytes, parser->bytes + parser->done, parser->have - parser->done);
            parser->have -= parser->done;
            parser->done = 0;
        }
        size_t total = relay_parser_check(parser);
        if (total) {
            parser->done = total;
            return total;
        }
        if (!*length) return 0;
        // Outside a frame, skip quickly to the next possible start.
        if (!parser->have) {
            const uint8_t *start = (const uint8_t *)memchr(*data, 'C', *length);
            size_t gap = start ? (size_t)(start - *data) : *length;
            *data += gap;
            *length -= gap;
            if (!*length) return 0;
        }
        parser->bytes[parser->have++] = **data;
        (*data)++;
        (*length)--;
    }
}

#endif
