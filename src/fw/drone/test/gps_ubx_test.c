// Host test of the GPS's UBX code (main/gps_ubx.h): make gps-test from src/fw. The reference frames come from pyubx2 (an independent implementation; 1.2, and 1.4.1 for the diagnostics): the drone's configuration and VALGET requests must match its bytes exactly, the module's VALGET, ACK, NAV-SAT and MON-RF replies must read as built, and its NAV-PVT must decode to the values it was built from, alone and inside a stream of NMEA text, noise, false starts and damaged copies fed in pieces of random size.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "gps_ubx.h"

static const char valset_hex[] =
    "b562068a2600000100000100524000c201000100741001020074100007009120010100213064002100112008e263";
static const char pvt_hex[] =
    "b56201075c0015cd5b07ea070b02141e05071e0000002efbffff03010011327c24b7d27353166eb2000032090100d2040000290900"
    "0088ffffff54010000c8ffffff680100004d61bc005900000051b345007b0000000000000000000000000000006821";

// From pyubx2 1.4.1: a VALSET and VALGET of the SBAS group (an 8-byte key among them) and CFG-RATE-MEAS; the module's VALGET reply with the keys in another order (built by hand, read back by pyubx2); an ACK-NAK of a VALSET; a NAV-SAT of three satellites (GPS 12 used with SBAS corrections, Galileo 5 unheard, BeiDou 30 used); a MON-RF block (jamming warning, noise 87, AGC 5000, CW indicator 42).
static const char sbas_valset_hex[] = "b562068a200000010000040036100103003610010600365000a80000000000000100213064003095";
static const char sbas_valget_hex[] = "b562068b140000000000040036100300361006003650010021301613";
static const char sbas_values_hex[] = "b562068b2000010000000100213064000600365000a80000000000000400361001030036100131b3";
static const char nak_hex[] = "b56205000200068a97bc";
static const char sat_hex[] =
    "b56201352c0000ca5b0701030000000c293778001e004f000100020500a50000000001000000031e2114fa0088ff1c00000084ed";
static const char rf_hex[] = "b5620a381c0000010000000202010000000000000000570088132afd7804760000006fe0";

static size_t unhex(const char *hex, uint8_t *out) {
    size_t n = 0;
    for (; hex[2 * n]; n++) sscanf(hex + 2 * n, "%2hhx", &out[n]);
    return n;
}

// One whole frame parsed out of its bytes.
static UbxFrame parse_hex(const char *hex) {
    static uint8_t bytes[1100];
    size_t length = unhex(hex, bytes);
    const uint8_t *data = bytes;
    UbxParser parser = {0};
    UbxFrame frame;
    assert(ubx_parse(&parser, &data, &length, &frame) && length == 0);
    return frame;
}

static void check_diagnostics(void) {
    static const UbxSetting sbas[] = {{0x10360004, 1}, {0x10360003, 1}, {0x50360006, 0xa800}, {0x30210001, 100}};
    uint8_t expected[128], built[128];
    size_t length = unhex(sbas_valset_hex, expected);
    assert(ubx_valset(sbas, 4, built, sizeof(built)) == length && !memcmp(built, expected, length));
    length = unhex(sbas_valget_hex, expected);
    assert(ubx_valget(sbas, 4, built, sizeof(built)) == length && !memcmp(built, expected, length));
    assert(!ubx_valget(sbas, 4, built, 20));

    UbxFrame values = parse_hex(sbas_values_hex);
    assert(ubx_valget_matches(&values, sbas, 4));
    assert(ubx_valget_matches(&values, sbas + 2, 1));
    static const UbxSetting other_mask[] = {{0x50360006, 0x72bc8}};
    static const UbxSetting missing[] = {{0x10360004, 1}, {0x10310025, 0}};
    assert(!ubx_valget_matches(&values, other_mask, 1));
    assert(!ubx_valget_matches(&values, missing, 2));

    uint8_t id;
    bool acked = true;
    UbxFrame nak = parse_hex(nak_hex);
    assert(ubx_ack(&nak, &id, &acked) && id == UBX_CFG_VALSET && !acked);
    assert(!ubx_ack(&values, &id, &acked));

    UbxFrame sat = parse_hex(sat_hex);
    UbxSatellite satellites[2];
    uint32_t itow;
    int count, known;
    assert(ubx_sat(&sat, &itow, satellites, 2, &count, &known));
    assert(itow == 123456000 && known == 3 && count == 2);
    assert(satellites[0].gnss == 0 && satellites[0].sv == 12 && satellites[0].cno == 41 && satellites[0].elevation == 55 && satellites[0].flags == 0xcf);
    assert(satellites[1].gnss == 3 && satellites[1].sv == 30 && satellites[1].cno == 33 && satellites[1].elevation == 20 && satellites[1].flags == 0x1c);
    assert(ubx_sat(&sat, &itow, satellites, 1, &count, &known) && count == 1);
    assert(!ubx_rf(&sat, &(UbxRf){0}));

    UbxRf rf;
    UbxFrame monitor = parse_hex(rf_hex);
    assert(ubx_rf(&monitor, &rf));
    assert(rf.jamming == 2 && rf.antenna_status == 2 && rf.antenna_power == 1 && rf.noise_per_ms == 87 && rf.agc_count == 5000 && rf.cw_jamming == 42);

    uint8_t poll[8];
    assert(ubx_frame(UBX_NAV, UBX_NAV_SAT, NULL, 0, poll, sizeof(poll)) == 8 && poll[6] == 0x36 && poll[7] == 0xa3);
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

    check_diagnostics();

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
