#ifndef RUNTIME_H
#define RUNTIME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Control and log rate; must divide the 100 Hz FreeRTOS tick. Records per FLG1 batch: one batch every 40 ms.
#define SAMPLE_HZ 100
#define LOG_BATCH 4
// MPU6050 DLPF_CFG: 3 is 42 Hz gyro bandwidth with 4.8 ms delay, 2 is 98 Hz with 2.8 ms but aliases vibration above the 50 Hz Nyquist rate.
#define IMU_DLPF 3
// MG995 analog servos stay at the standard 50 Hz. ESC refresh is independent; 50 and 400 Hz both use 1000–2200 us pulses.
#define SERVO_HZ 100
#define ESC_HZ 400
// Raw sensor axes to body X forward, Y left, Z up: body axis i is sensor_to_body[i] times sensor axis sensor_axis[i]. The replacement IMU is turned -90° about body Y, so its X points up and its Z forward.
static const int sensor_axis[3] = {2, 1, 0};
static const int sensor_to_body[3] = {1, -1, 1};

// The drone's periodic status message on direct channel 0; decoded by controller/bridge.py STATUS_FORMAT.
typedef struct __attribute__((packed)) {
    char magic[4];
    uint32_t time_ms, target_sequence;
    // 1 IMU online, 2 calibrating, 4 calibrated, 8 armed, 16 recording, 32 recorder subscribed, 64 throttle latched, 128 force valid.
    uint16_t flags;
    // Native stop reason (controller/log_format.py STOP_REASONS).
    uint8_t reason;
    float accel[3], gyro[3], roll, pitch, temperature, servo[2];
    uint16_t throttle[2];
    // Roll and pitch (degrees), yaw rate (degrees/s), collective throttle.
    float target[4];
    uint32_t log_samples, log_dropped;
    // Load cell raw counts, the age of that reading and the count of readings; battery ADC (16-bit scale) and volts.
    int32_t force;
    uint32_t force_age_ms, force_sequence;
    uint16_t adc_raw, settings_version;
    float voltage;
} Status;
_Static_assert(sizeof(Status) == 107, "Status layout");

typedef struct {
    uint32_t session, started_us, samples, dropped, elapsed_us, pending;
    bool active, calibrated;
    uint8_t reason;
    float bias[3], pose_zero[2];
} LogState;

// One binary flight-log record (controller/log_format.py SAMPLE_FORMAT), and the longest metadata JSON.
#define RECORD_SIZE 48
#define META 3072

// Outputs at zero throttle and servos at their offsets; before the radio, so a stop always has outputs to stop.
void flight_runtime_outputs(void);
// The IMU and the control task on core 1.
void flight_runtime_start(void);
void flight_runtime_set_voltage(uint16_t millivolts);
bool flight_runtime_idle(void);
// Returns why arming was refused (arming.h), or NULL once armed and recording. expected_generation is flight_runtime_stop_generation() from when the request arrived. arm_sequence is the arm packet's laptop sequence.
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

// The ESP-NOW link and request service (radio.c).
void radio_start(void);
// True while the controller has been heard in the last 2 s.
bool radio_connected(void);
// Replaces the metadata JSON sent to the recorder after each start; false while the previous one is being sent.
bool radio_log_meta(const char *data, size_t length);

// The load cell and battery fields of the status message (main.c).
void sensors_status(Status *status);

#endif
