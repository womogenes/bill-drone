#ifndef RUNTIME_H
#define RUNTIME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Bytes of the status message the Python application fills (drivers/protocol.py APP_FORMAT).
#define STATUS_APP 24

// The drone's periodic status message on direct channel 0; decoded by controller/bridge.py.
typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t time_ms, target_sequence;
    // 1 IMU online, 2 calibrating, 4 calibrated, 8 armed, 16 recording, 32 recorder subscribed, 64 throttle latched.
    uint16_t flags;
    // Native stop reason (drivers/log_format.py STOP_REASONS); age of the application block in 10 ms units.
    uint8_t reason, app_age;
    float accel[3], gyro[3], roll, pitch, temperature, servo[2];
    uint16_t throttle[2];
    // Roll and pitch (degrees), yaw rate (degrees/s), collective throttle.
    float target[4];
    uint32_t log_samples, log_dropped;
    uint8_t app[STATUS_APP];
} Status;
_Static_assert(sizeof(Status) == 112, "Status layout");

typedef struct {
    uint32_t session, started_us, samples, dropped, elapsed_us, pending;
    bool active, calibrated;
    uint8_t reason;
    float bias[3], pose_zero[2];
} LogState;

// One binary flight-log record (drivers/log_format.py SAMPLE_FORMAT).
#define RECORD_SIZE 48

bool flight_runtime_idle(void);
// Returns why arming was refused (arming.h), or NULL once armed and recording. expected_generation is
// flight_runtime_stop_generation() from when the request arrived.
// arm_sequence is the arm packet's laptop sequence.
const char *flight_runtime_arm(bool recorder, bool calibration_unsaved, uint32_t expected_generation,
                               uint32_t arm_sequence);
uint32_t flight_runtime_stop_generation(void);
// Brackets every flash and NVS write: begin fails unless disarmed with zero throttle; outputs stay zero until end.
bool flight_runtime_begin_maintenance(void);
void flight_runtime_end_maintenance(void);
void flight_runtime_log(LogState *log);
size_t flight_runtime_read_log(uint8_t *output, size_t count);
void flight_runtime_consume_log(size_t count);
// Collective, roll, pitch, yaw rate, then servo angles and throttles applied while disarmed. False if rejected.
bool flight_runtime_targets(const float *targets);
// sequence: the STOP packet's laptop sequence.
void flight_runtime_command_stop(uint32_t sequence);
// Copies the settings store's tunable values into the controller.
void flight_runtime_apply_settings(void);
const char *flight_runtime_calibrate(void);
// True once per finished calibration: gyro bias (rad/s), pose zero (degrees) and the sensor frame it belongs to.
bool flight_runtime_take_calibration(float *bias, float *pose_zero, float *sensor_frame);
const char *flight_runtime_zero_servo(int side, float *offset, char *error, size_t size);
void flight_runtime_status(Status *status);

#endif
