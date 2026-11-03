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

// Beitian BE-220 (u-blox M10) on UART1: its TX into GPIO23, its RX from GPIO17 (both on the board's right side, 2026-11-02). The task finds the module's baud, sets 115200 with NAV-PVT only at 10 Hz in the airborne (4 g) model, and searches again after 3 s without a solution. Call from a core-0 task.
void gps_start(void);
// The newest fix if it is at most max_age_us old.
bool gps_fix(GpsFix *fix, uint32_t max_age_us);
void gps_stats(GpsStats *stats);

#endif
