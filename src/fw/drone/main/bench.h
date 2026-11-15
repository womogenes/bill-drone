// The compute and scheduling benchmark's hooks in the control task (test build only: make bench defines BENCH; flight_rt.c includes this, and bench_flight.c at its end, so the benchmark sees the task's own state). Every stage of the tick is timed in CPU cycles (CCOUNT, core 1's own counter) from MARK to MARK, the lock's waits and holds per caller, and the task's wake-up against the FreeRTOS tick that wakes it. A run (radio request b, flight_runtime_bench) flies the real tick, armed, on a simulated plant or a recording's sensor data in a sandbox: the actuators' pulses go to a shadow instead of the hardware, and the state is put back as it was afterwards.
#ifndef BENCH_H
#define BENCH_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "baro.h"
#include "gps.h"

// The tick's stages in order; a MARK(stage) gives the stage every cycle since the last mark.
enum { S_IMU, S_ATTITUDE, S_INDI, S_FLIP, S_INPUTS, S_EKF_QUEUE, S_EKF, S_ESTIMATE, S_RECOVER, S_HOVER, S_SETPOINT, S_CONTROL, S_OUTPUT, S_REST, S_LOG, STAGES };

static void bench_mark(int stage);
static void bench_lock(const char *site);
static void bench_unlock(void);
static void bench_wake(void);
static void bench_sensors(esp_err_t *result, uint8_t *data, int16_t *sensor, int32_t *sum, int *samples);
static void bench_begin(void);
static void bench_end(const void *tick);
static void bench_after(void);
static void bench_start(void);
static bool bench_shadow(int index, float us);
static bool bench_gps_fix(GpsFix *fix, uint32_t max_age_us);
static bool bench_baro_sample(BaroSample *sample, uint32_t max_age_us);

#define MARK(stage) bench_mark(stage)
#define LOCK() bench_lock(__func__)
#define UNLOCK() bench_unlock()
#define BENCH_WAKE() bench_wake()
#define BENCH_SENSORS(result, data, sensor, sum, samples) bench_sensors(result, data, sensor, sum, samples)
#define BENCH_BEGIN() bench_begin()
#define BENCH_END(t) bench_end(t)
#define BENCH_AFTER() bench_after()
#define BENCH_START() bench_start()
#define BENCH_PULSE(index, us) \
    if (bench_shadow(index, us)) return
// The sandbox's GPS and barometer stand in for the real ones while a run flies.
#define gps_fix(fix, max_age_us) bench_gps_fix(fix, max_age_us)
#define baro_sample(sample, max_age_us) bench_baro_sample(sample, max_age_us)

#endif
