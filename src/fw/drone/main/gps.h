#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>
#include "gps_ubx.h"

// The newest position and velocity solution, stamped on this drone's esp_timer clock when its last byte arrived.
typedef struct {
    UbxPvt pvt;
    int64_t received_us;
} GpsFix;

typedef struct {
    // NAV-PVT messages decoded; corrupt and skipped as UbxParser; configured: the module answered at GPS_BAUD after the last configuration (0 while searching for its baud).
    uint32_t messages, corrupt, skipped;
    bool configured;
    // The newest fix, received_us -1 before the first.
    GpsFix fix;
} GpsStats;

// Satellites kept from each NAV-SAT: as many as fit one radio message after the report's header (gps_report).
#define GPS_SATELLITES 40
// How each configuration group (gps.c groups: signals, SBAS, interference monitor, output) went at the module's last configuration, 2 bits per group at bit 2i of GpsSky.config.
enum { GPS_PENDING, GPS_SET, GPS_REFUSED, GPS_UNANSWERED };

// The module's own diagnostics, polled every second while it is configured: the satellites heard or used (UBX-NAV-SAT, its GPS time of week and how many it listed in all) and the RF health (UBX-MON-RF), each stamped on arrival (-1 before the first).
typedef struct {
    int64_t sat_us, rf_us;
    uint32_t sat_itow_ms;
    uint8_t known, count, config;
    UbxSatellite satellites[GPS_SATELLITES];
    UbxRf rf;
} GpsSky;

// Beitian BE-220 (u-blox M10) on UART1: its TX into GPIO23, its RX from GPIO5 (both on the board's right side, as wired on 2026-11-02; GPIO5 is a strapping pin whose boot-time pulses the module ignores). The task finds the module's baud, sets 115200 with NAV-PVT only at 10 Hz in the airborne (4 g) model, then checks and sets its signals (GPS, Galileo, BeiDou B1C, QZSS and WAAS), SBAS corrections and interference monitor, polls its diagnostics every second, and searches again after 3 s without a solution. Call from a core-0 task.
void gps_start(void);
// The newest fix if it is at most max_age_us old.
bool gps_fix(GpsFix *fix, uint32_t max_age_us);
void gps_stats(GpsStats *stats);
void gps_sky(GpsSky *sky);
// The diagnostics as the radio's GPS report (bridge.py decode_gps_report): "GPD1", then the sky's fields with ages in ms (0xffffffff before the first) and its satellites; returns the length, at most 231 bytes.
size_t gps_report(uint8_t *out, size_t size, int64_t now);
// The tunnel for exploring the module from the laptop (the drone's N request): gps_send writes raw bytes to the module (UBX polls and configuration); gps_take copies out the UBX frames other than NAV-PVT received since the last take, whole, as the module sent them, and returns their length (frames that did not fit the 4 KB buffer are dropped and counted in *dropped). Frames are kept only within 5 s of the tunnel's last use, or until gps_tunnel_hold's time runs out, so the drone's own polls do not fill it while nobody takes them.
void gps_send(const uint8_t *bytes, size_t length);
size_t gps_take(uint8_t *out, size_t size, uint32_t *dropped);
void gps_tunnel_hold(uint32_t ms);

#endif
