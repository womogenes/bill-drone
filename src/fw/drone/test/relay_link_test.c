// Host test of the radio relay's frames (fw/relay_link.h): make camera-test from src/fw. The stream mixes relay frames of every length with TAG3 packets, CUR1 replies, the camera's boot text, random noise, false "CRL1" starts and damaged frames, and is fed in pieces of random size to both the relay parser and the TAG3 parser, as the drone's receive task does. Every intact frame and packet must come out once, in order, and nothing else.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "camera_link.h"
#include "../../relay_link.h"

static uint8_t stream[1 << 21];
static size_t stream_length;
static uint32_t frames[8192], packets[8192];
static size_t frame_count, packet_count;

static void put(const void *data, size_t length) {
    assert(stream_length + length <= sizeof(stream));
    memcpy(stream + stream_length, data, length);
    stream_length += length;
}

// A frame whose payload starts with its number, then filler that contains both magics.
static size_t make_frame(uint8_t *out, uint32_t number, size_t length) {
    uint8_t payload[RELAY_PAYLOAD];
    for (size_t i = 0; i < length; i++) payload[i] = (uint8_t)(number * 7 + i);
    if (length >= 4) memcpy(payload, &number, 4);
    if (length >= 12) memcpy(payload + 4, "CRL1TAG3", 8);
    return relay_frame(out, number % 2 ? 'R' : 'S', number % 7, payload, length);
}

static void put_packet(uint32_t sequence) {
    CameraPacket packet;
    memset(&packet, 0, sizeof(packet));
    memcpy(packet.magic, "TAG3", 4);
    packet.sequence = sequence;
    memcpy(&packet.margin, "CRL1", 4);
    packet.checksum = camera_crc16((const uint8_t *)&packet, offsetof(CameraPacket, checksum));
    put(&packet, sizeof(packet));
    packets[packet_count++] = sequence;
}

int main(void) {
    srand(1);
    uint8_t frame[RELAY_FRAME];
    assert(!relay_frame(frame, 'S', 0, frame, RELAY_PAYLOAD + 1));
    put("ESP-ROM:esp32s3-20210327\r\nRLY", 29);
    uint32_t number = 1, sequence = 1;
    for (int i = 0; i < 6000; i++) {
        int kind = rand() % 10;
        if (kind < 5) {
            // Lengths 4 (an answer's minimum here) to the full 256, numbered for checking.
            size_t total = make_frame(frame, number, 4 + rand() % (RELAY_PAYLOAD - 3));
            put(frame, total);
            frames[frame_count++] = number++;
        } else if (kind < 7) {
            put_packet(sequence++);
        } else if (kind == 7) {
            // A damaged frame: one byte changed after the magic.
            size_t total = make_frame(frame, 0xdead0000 + i, 4 + rand() % 200);
            frame[4 + rand() % (total - 4)] ^= 1 + rand() % 255;
            put(frame, total);
        } else if (kind == 8) {
            // A truncated frame: the start of one, then the next real one picks up.
            make_frame(frame, 0xbeef0000 + i, 100);
            put(frame, RELAY_HEADER + rand() % 50);
        } else {
            uint8_t noise[40];
            for (size_t k = 0; k < sizeof(noise); k++) noise[k] = rand();
            memcpy(noise + rand() % 30, "CRL1", 4);
            put(noise, sizeof(noise));
        }
    }
    // A complete frame closes the stream so the last truncated one is resolved.
    put(frame, make_frame(frame, number, 20));
    frames[frame_count++] = number++;

    RelayParser relay = {0};
    CameraParser camera = {0};
    size_t got_frames = 0, decoded = 0, got_packets = 0, offset = 0;
    while (offset < stream_length) {
        size_t piece = 1 + rand() % 300;
        if (piece > stream_length - offset) piece = stream_length - offset;
        const uint8_t *data = stream + offset;
        size_t length = piece, total;
        while ((total = relay_parse(&relay, &data, &length))) {
            uint32_t got;
            memcpy(&got, relay.bytes + RELAY_HEADER, 4);
            // The numbered frames must come out in order, each one real.
            while (got_frames < frame_count && frames[got_frames] != got) got_frames++;
            assert(got_frames < frame_count);
            got_frames++;
            decoded++;
        }
        data = stream + offset;
        length = piece;
        CameraPacket packet;
        while (camera_parse(&camera, &data, &length, &packet)) {
            assert(got_packets < packet_count && packets[got_packets] == packet.sequence);
            got_packets++;
        }
        offset += piece;
    }
    assert(got_frames == frame_count);
    // The parser resumes one byte past each false start, so a frame that began inside a truncated or damaged one is still found.
    assert(decoded == frame_count);
    assert(got_packets == packet_count);
    printf("relay link: %zu of %zu frames (%u decoded, %u corrupt) and %zu packets through\n", decoded, frame_count, relay.frames, relay.corrupt, packet_count);
    return 0;
}
