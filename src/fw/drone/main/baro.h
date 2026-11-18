#ifndef BARO_H
#define BARO_H

#include <stdbool.h>
#include <stdint.h>

// One compensated reading: when it was read (us, esp_timer; the chip finished the measurement within the poll before), pressure (Pa), the chip's temperature (°C) and the pressure altitude in the standard atmosphere (m; only its changes mean height).
typedef struct {
    int64_t read_us;
    float pressure, temperature, altitude;
} BaroSample;

typedef struct {
    // address: where the chip answered (0x76 with SDO low, 0x77 high; 0 not found, or lost and searching); chip: its ID (0x50 BMP388, 0x60 BMP390).
    uint8_t address, chip;
    // Readings kept; failed bus transfers; readings dropped as out of range or a jump; searches that found nothing.
    uint32_t samples, errors, rejected, searches;
    // The newest kept reading, read_us -1 before the first.
    BaroSample sample;
} BaroStats;

// Bosch BMP388 (CJMCU-388 board) on I2C1, its own bus: SDA GPIO17, SCL GPIO15 (the IMU's bus, GPIO19 and 22, stays the IMU's alone), CSB to 3.3 V for I2C. The task looks for it at both addresses every second until it answers, configures it (soft reset, calibration, pressure x8 and temperature x1 oversampling, IIR coefficient 3, 50 Hz normal mode) and polls it every 10 ms for new measurements; after repeated failed reads it searches again. Without it nothing else changes. Call from a core-0 task.
void baro_start(void);
// The newest reading if it is at most max_age_us old.
bool baro_sample(BaroSample *sample, uint32_t max_age_us);
void baro_stats(BaroStats *stats);

#endif
