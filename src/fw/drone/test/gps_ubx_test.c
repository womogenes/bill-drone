// Host test of the GPS's UBX code (main/gps_ubx.h): make gps-test from src/fw. The reference frames come from pyubx2 1.2 (an independent implementation): the drone's configuration request must match its bytes exactly, and its NAV-PVT must decode to the values it was built from, alone and inside a stream of NMEA text, noise, false starts and damaged copies fed in pieces of random size.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "gps_ubx.h"

static const char valset_hex[] =
    "b562068a2600000100000100524000c201000100741001020074100007009120010100213064002100112008e263";
static const char pvt_hex[] =
    "b56201075c0015cd5b07ea070b02141e05071e0000002efbffff03010011327c24b7d27353166eb2000032090100d2040000290900"
    "0088ffffff54010000c8ffffff680100004d61bc005900000051b345007b0000000000000000000000000000006821";

static size_t unhex(const char *hex, uint8_t *out) {
    size_t n = 0;
    for (; hex[2 * n]; n++) sscanf(hex + 2 * n, "%2hhx", &out[n]);
    return n;
}

static void check_pvt(const UbxFrame *frame) {
    UbxPvt pvt;
    assert(ubx_pvt(frame, &pvt));
    assert(pvt.itow_ms == 123456789 && pvt.fix_type == 3 && (pvt.flags & 1) && pvt.satellites == 17);
    assert(pvt.longitude == -1222345678 && pvt.latitude == 374567890 && pvt.height_mm == 67890);
    assert(pvt.horizontal_accuracy_mm == 1234 && pvt.vertical_accuracy_mm == 2345 && pvt.speed_accuracy_mm_s == 89);
    assert(pvt.velocity_mm_s[0] == -120 && pvt.velocity_mm_s[1] == 340 && pvt.velocity_mm_s[2] == -56);
}

int main(void) {
    static const UbxSetting configuration[] = {
        {0x40520001, 115200}, {0x10740001, 1}, {0x10740002, 0}, {0x20910007, 1}, {0x30210001, 100}, {0x20110021, 8},
    };
    uint8_t expected[128], built[128];
    size_t expected_length = unhex(valset_hex, expected);
    size_t built_length = ubx_valset(configuration, 6, built, sizeof(built));
    assert(built_length == expected_length && !memcmp(built, expected, built_length));
    assert(!ubx_valset(configuration, 6, built, 40));

    uint8_t pvt[128];
    size_t pvt_length = unhex(pvt_hex, pvt);
    assert(pvt_length == 100);

    static uint8_t stream[200000];
    size_t length = 0;
    int intact = 0;
    srand(1);
    for (int round = 0; round < 1000; round++) {
        switch (rand() % 6) {
        case 0: {
            const char *nmea = "$GNGGA,203005.00,3727.40734,N,12214.07407,W,1,17,1.23,67.9,M,-29.9,M,,*5B\r\n";
            memcpy(stream + length, nmea, strlen(nmea));
            length += strlen(nmea);
            break;
        }
        case 1:
            for (int i = rand() % 50; i > 0; i--) stream[length++] = rand();
            break;
        case 2:
            // A false start: the sync bytes, then a length too long to be a frame.
            stream[length++] = UBX_SYNC1;
            stream[length++] = UBX_SYNC2;
            stream[length++] = 1;
            stream[length++] = 7;
            stream[length++] = 0xff;
            stream[length++] = 0x7f;
            break;
        case 3: {
            // A damaged copy: one payload byte changed, so its checksum fails.
            memcpy(stream + length, pvt, pvt_length);
            stream[length + 6 + rand() % 92] ^= 1 + rand() % 255;
            length += pvt_length;
            break;
        }
        default:
            memcpy(stream + length, pvt, pvt_length);
            length += pvt_length;
            intact++;
        }
    }

    UbxParser parser = {0};
    int found = 0;
    for (size_t at = 0; at < length;) {
        size_t piece = 1 + rand() % 150;
        if (piece > length - at) piece = length - at;
        const uint8_t *data = stream + at;
        size_t left = piece;
        UbxFrame frame;
        while (ubx_parse(&parser, &data, &left, &frame)) {
            check_pvt(&frame);
            found++;
        }
        at += piece;
    }
    // A random byte run can hold a frame only by an astronomically unlikely checksum match.
    assert(found == intact);
    assert(parser.corrupt > 0 && parser.skipped > 0);
    printf("gps ubx: %d frames of %zu bytes, %u corrupt, %u bytes skipped\n", found, length, parser.corrupt, parser.skipped);
    return 0;
}
