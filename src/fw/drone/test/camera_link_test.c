// Host test of the camera link parser and SYNC matching (main/camera_link.h): make camera-test from src/fw. The stream mixes real packets with what the wire carries at camera power-up (the ROM and bootloader's 115200-baud text as a 2 Mbaud receiver decodes it), random noise, false "TAG3" starts, truncated and damaged packets, and is fed in pieces of random size. Every intact packet must come out once, in order, and nothing else.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "camera_link.h"

static uint8_t stream[1 << 20];
static size_t stream_length;
static uint32_t expected[4096];
static size_t expected_count;

static void put(const void *data, size_t length) {
    assert(stream_length + length <= sizeof(stream));
    memcpy(stream + stream_length, data, length);
    stream_length += length;
}

static CameraPacket make_packet(uint32_t sequence) {
    CameraPacket packet;
    memset(&packet, 0, sizeof(packet));
    memcpy(packet.magic, "TAG3", 4);
    packet.sequence = sequence;
    packet.capture_us = sequence * 33974u;
    packet.age_us = 36000 + sequence % 1000;
    packet.valid = 1;
    packet.trusted = 1;
    for (int i = 0; i < 3; i++) packet.position[i] = sequence * 0.001f + i;
    packet.rotation[0] = packet.rotation[4] = packet.rotation[8] = 1;
    // Put the magic inside the payload too, as a float or counter might.
    memcpy(&packet.margin, "TAG3", 4);
    packet.checksum = camera_crc16((const uint8_t *)&packet, offsetof(CameraPacket, checksum));
    return packet;
}

static void put_packet(uint32_t sequence) {
    CameraPacket packet = make_packet(sequence);
    put(&packet, sizeof(packet));
    expected[expected_count++] = sequence;
}

// What a 2 Mbaud UART receiver makes of text sent at 115200: the line is sampled at the receiver's rate, a low after idle is a start bit, eight data bits are sampled mid-bit, and the byte is kept even when the stop bit is wrong (a framing error), as the ESP32 does.
static void put_slow_text(const char *text) {
    static uint8_t line[1 << 20];
    size_t samples = 0;
    const double ratio = 2000000.0 / 115200.0;
    for (const char *c = text; *c; c++) {
        int bits[10] = {0};
        for (int b = 0; b < 8; b++) bits[1 + b] = (*c >> b) & 1;
        bits[9] = 1;
        for (int b = 0; b < 10; b++) {
            size_t end = (size_t)((c - text) * 10 * ratio + (b + 1) * ratio);
            while (samples < end) line[samples++] = bits[b];
        }
    }
    for (int i = 0; i < 40; i++) line[samples++] = 1;
    for (size_t i = 0; i + 10 <= samples;) {
        if (line[i]) {
            i++;
            continue;
        }
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++) byte |= line[i + 1 + b] << b;
        put(&byte, 1);
        i += 10;
        while (i < samples && !line[i]) i++;
    }
}

static void put_noise(size_t length) {
    for (size_t i = 0; i < length; i++) {
        uint8_t byte = rand() % 4 == 0 ? "TAG3"[rand() % 4] : (uint8_t)rand();
        put(&byte, 1);
    }
}

static void test_stream(void) {
    srand(1);
    put_slow_text("ets Jun  8 2016 00:22:57\r\n\r\nrst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\r\n"
                  "configsip: 0, SPIWP:0xee\r\nclk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00\r\n"
                  "mode:DIO, clock div:1\r\nload:0x3fff0030,len:1184\r\nentry 0x400805f0\r\n");
    uint32_t sequence = 0;
    for (int round = 0; round < 400; round++) {
        put_packet(sequence++);
        switch (round % 8) {
        case 1:
            put_noise(1 + rand() % 300);
            break;
        case 2: {
            // A packet cut short by a reset: its tail never arrives.
            CameraPacket packet = make_packet(100000 + round);
            put(&packet, 20 + rand() % 140);
            break;
        }
        case 3: {
            // One damaged byte: the packet must be refused.
            CameraPacket packet = make_packet(200000 + round);
            ((uint8_t *)&packet)[4 + rand() % 158] ^= 1 << (rand() % 8);
            put(&packet, sizeof(packet));
            break;
        }
        case 4:
            put("TAG3", 4);
            put("TA", 2);
            break;
        case 5:
            put_slow_text("Brownout detector was triggered\r\n");
            break;
        default:
            break;
        }
    }
    CameraParser parser = {0};
    size_t found = 0;
    for (size_t at = 0; at < stream_length;) {
        size_t piece = 1 + rand() % 400;
        if (piece > stream_length - at) piece = stream_length - at;
        const uint8_t *data = stream + at;
        size_t length = piece;
        CameraPacket packet;
        while (camera_parse(&parser, &data, &length, &packet)) {
            assert(found < expected_count);
            if (packet.sequence != expected[found]) {
                fprintf(stderr, "packet %zu: sequence %u, expected %u\n", found, packet.sequence, expected[found]);
                exit(1);
            }
            CameraPacket reference = make_packet(packet.sequence);
            assert(!memcmp(&packet, &reference, sizeof(packet)));
            found++;
        }
        assert(length == 0);
        at += piece;
    }
    if (found != expected_count) {
        fprintf(stderr, "found %zu of %zu packets\n", found, expected_count);
        exit(1);
    }
    printf("stream: %zu bytes, %zu packets recovered, %u bytes skipped, %u corrupt candidates\n", stream_length, found,
           parser.skipped, parser.corrupt);
    assert(parser.corrupt >= 50);
}

static void test_sync(void) {
    uint32_t used = 0, since;
    // No edge yet.
    assert(!camera_sync(1000, 0, 0, &used, &since));
    // An edge 900 us before the packet completes dates it, once.
    assert(camera_sync(10900, 1, 10000, &used, &since) && since == 900 && used == 1);
    assert(!camera_sync(44900, 1, 10000, &used, &since));
    // An edge too long ago is somebody else's (a missed packet, or the camera's boot) and is not used.
    assert(!camera_sync(100000, 2, 50000, &used, &since));
    // Across the 32-bit wrap.
    assert(camera_sync(500, 3, 0xffffff00u, &used, &since) && since == 756);
    printf("sync: ok\n");
}

static CameraUpdateReply make_reply(uint32_t id, const char *text) {
    CameraUpdateReply reply = {.magic = {'C', 'U', 'R', '1'}, .id = id, .length = (uint8_t)strlen(text)};
    memcpy(reply.text, text, reply.length);
    reply.checksum = camera_crc16((const uint8_t *)&reply, offsetof(CameraUpdateReply, checksum));
    return reply;
}

static void test_replies(void) {
    // Replies between pose packets, one damaged and one split across reads, in a stream that starts with a false "CU".
    stream_length = 0;
    put("CUCU", 4);
    put_packet(1);
    CameraUpdateReply first = make_reply(7, "ota_0 ota_1 1310720 0123456789abcdef"), damaged = make_reply(8, "bad"), second = make_reply(9, "");
    put(&first, sizeof(first));
    damaged.text[0] ^= 1;
    put(&damaged, sizeof(damaged));
    put_packet(2);
    put(&second, sizeof(second));
    CameraReplyParser parser = {0};
    uint32_t ids[4];
    size_t found = 0;
    for (size_t at = 0; at < stream_length; at += 37) {
        const uint8_t *data = stream + at;
        size_t length = stream_length - at < 37 ? stream_length - at : 37;
        CameraUpdateReply reply;
        while (camera_reply_parse(&parser, &data, &length, &reply)) {
            assert(found < 4);
            if (reply.id == 7) assert(!memcmp(&reply, &first, sizeof(reply)));
            ids[found++] = reply.id;
        }
    }
    assert(found == 2 && ids[0] == 7 && ids[1] == 9);
    printf("replies: ok\n");
}

int main(void) {
    test_stream();
    test_sync();
    test_replies();
    return 0;
}
