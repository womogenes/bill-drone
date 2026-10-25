#include <math.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "soc/ledc_struct.h"
#include "flight_math.h"
#include "runtime.h"
#include "settings.h"
#include "arming.h"
#include "camera.h"
#include "hover.h"

#define LOG_CAPACITY 200
#define DEG (180.0f / 3.14159265358979323846f)
// MPU6050 on I2C: SDA, SCL and address.
#define SDA 19
#define SCL 22
#define MPU 0x68
// Motor watchdog: the laptop's targets must keep arriving.
#define HEARTBEAT_MS 500
// Tolerate one missed control tick; a second consecutive miss stops the motors.
#define MAX_SAMPLE_MS (2500 / SAMPLE_HZ)
// Degrees.
#define TIP_LIMIT 45
#define SERVO_LIMIT 60
#define THROTTLE_MAX 1200
#define PWM_BITS 16
// Streamed target limits: degrees and degrees per second.
#define TARGET_TILT 15
#define TARGET_RATE 45
// Battery smoothing for thrust compensation (s), and the furthest the smoothed voltage may sit from thrust_voltage, as a ratio, before it is treated as an ADC fault and clamped.
#define VOLTAGE_TAU 0.3f
#define VOLTAGE_RANGE 0.15f
// Oldest camera sighting hover uses (us), and the highest throttle it may ask for (the thrust table's rising end).
#define HOVER_POSE_US 150000
#define HOVER_THROTTLE 1100

typedef struct __attribute__((packed)) {
    uint32_t elapsed, sequence;
    int16_t raw[7], attitude[2];
    uint16_t throttle[2];
    int16_t servo[2], target[3];
    uint16_t base, control_us, flags, voltage;
} Record;
_Static_assert(sizeof(Record) == 48, "Flight record layout");

// Output channels, in this order: left servo, left ESC, right servo, right ESC (index side * 2 + ESC).
static const int pins[4] = {14, 25, 16, 18};
// LEDC ticks per microsecond of pulse.
static const float ticks[4] = {
    (1 << PWM_BITS) * SERVO_HZ / 1e6f, (1 << PWM_BITS) * ESC_HZ / 1e6f,
    (1 << PWM_BITS) * SERVO_HZ / 1e6f, (1 << PWM_BITS) * ESC_HZ / 1e6f,
};

typedef struct {
    float accel_scale[3], gyro_scale[3], gravity_tau, powered_tau, tip_limit;
} Config;

typedef struct {
    // latched: manual throttle stays zero after a stop until the laptop sends zero throttle.
    // servo_held: a zeroed servo ignores streamed angles until the laptop sends zero for it.
    // calibration_ready: a finished calibration waits for the radio's update task to store it.
    bool online, enabled, calibrated, latched, servo_held[2], calibration_ready;
    int reason, calibration_left, calibration_count;
    float accel[3], gyro[3], bias[3], calibration_sum[3], gravity_sum[3];
    float temperature, servo[2], offsets[2], accel_offset[3], accel_gain[3];
    int throttle[2];
    Attitude attitude, pose_zero;
    Controller controller;
    uint64_t sample_us, heartbeat_us;
} State;

static Config config;
static State state;
static Command command;
static Record records[LOG_CAPACITY];
static uint32_t log_head, log_tail, log_samples, log_dropped, log_elapsed, log_session;
static uint64_t log_started;
// running: the control task has started; until then targets, arming and calibration are refused.
static bool log_active, running, sensor_configured;
// maintenance: a flash or NVS write is in progress, so outputs are held at zero and arming is refused.
static bool maintenance;
// Counts every stop; an arm request carries the count from when it arrived.
static uint32_t stop_generation;
// The newest laptop STOP since boot (each boot is a new radio session).
static StopBarrier stop_barrier;
static uint16_t voltage_mv;
// Smoothed battery volts, for thrust compensation; 0 until the first reading.
static float battery_v;
// The streamed hover mode (hover.h), the hover loop, and the command actually flown this tick (hover replaces base and targets), which the recording logs.
static int hover_mode;
static HoverState hover;
static Command flown;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

static float limit(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

// Caller holds lock so an emergency stop cannot race a later duty write.
// Latches a duty update without waiting for the previous transition.
static void pulse(int index, float us) {
    LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[index].conf1.duty_start = 0;
    LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[index].duty.duty = (uint32_t)lroundf(us * ticks[index]) << 4;
    LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[index].conf1.duty_start = 1;
}

static void write_servo(int side) {
    pulse(side * 2, 1500 + (state.servo[side] + state.offsets[side]) * 1000 / 90);
}

static void stop_locked(int reason) {
    stop_generation++;
    command.enabled = command.base = 0;
    state.enabled = false;
    state.latched = true;
    state.reason = reason;
    state.controller = (Controller){0};
    for (int side = 0; side < 2; side++) {
        state.throttle[side] = 0;
        pulse(side * 2 + 1, 1000);
    }
    if (log_active) log_elapsed = esp_timer_get_time() - log_started;
    log_active = false;
}

static esp_err_t write_mpu(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    return i2c_master_write_to_device(I2C_NUM_0, MPU, data, sizeof(data), pdMS_TO_TICKS(10));
}

// Sign bits (x 1, y 2, z 4) of the sensor-to-body mapping, plus its axis order from bit 3; 0 there is the identity order.
static float frame(void) {
    return (config.accel_scale[0] < 0) | (config.accel_scale[1] < 0) << 1 | (config.accel_scale[2] < 0) << 2 |
           sensor_axis[0] << 3 | (sensor_axis[1] ^ 1) << 5 | (sensor_axis[2] ^ 2) << 7;
}

static Attitude relative_attitude(const State *value) {
    float roll = value->attitude.roll - value->pose_zero.roll;
    float pitch = value->attitude.pitch - value->pose_zero.pitch;
    return (Attitude){atan2f(sinf(roll), cosf(roll)), atan2f(sinf(pitch), cosf(pitch))};
}

static void capture(uint64_t now, const int16_t *raw, uint32_t duration) {
    log_elapsed = now - log_started;
    uint32_t sequence = log_samples++;
    if (log_head - log_tail == LOG_CAPACITY) {
        log_dropped++;
        return;
    }
    Record *record = &records[log_head++ % LOG_CAPACITY];
    record->elapsed = log_elapsed;
    record->sequence = sequence;
    memcpy(record->raw, raw, sizeof(record->raw));
    Attitude pose = relative_attitude(&state);
    record->attitude[0] = lroundf(pose.roll * DEG * 100);
    record->attitude[1] = lroundf(pose.pitch * DEG * 100);
    for (int side = 0; side < 2; side++) {
        record->throttle[side] = state.throttle[side];
        record->servo[side] = lroundf(state.servo[side] * 100);
    }
    record->target[0] = lroundf(flown.target_roll * DEG * 100);
    record->target[1] = lroundf(flown.target_pitch * DEG * 100);
    record->target[2] = lroundf(flown.target_yaw * DEG * 100);
    record->base = lroundf(flown.base);
    record->control_us = duration > 65535 ? 65535 : duration;
    // Bit 0 armed, bits 1-2 the hover mode, bit 3 hover sees the tag, bit 4 a hover landing touched down.
    record->flags = state.enabled | hover_mode << 1 | (hover.have_pose && (int64_t)now - hover.seen_us <= 250000) << 3 | hover.landed << 4;
    record->voltage = voltage_mv;
}

static void runtime_task(void *arg) {
    TickType_t deadline = xTaskGetTickCount();
    TickType_t period = configTICK_RATE_HZ / SAMPLE_HZ;
    uint64_t previous = 0;
    while (true) {
        uint8_t reg = 0x3a, data[15];
        esp_err_t result = sensor_configured
            ? i2c_master_write_read_device(I2C_NUM_0, MPU, &reg, 1,
                                          data, sizeof(data), pdMS_TO_TICKS(10))
            : ESP_ERR_INVALID_STATE;
        uint64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&lock);
        bool recording = log_active;
        if ((state.enabled || state.throttle[0] || state.throttle[1]) &&
            now - state.heartbeat_us > HEARTBEAT_MS * 1000ULL) stop_locked(2);

        if (result != ESP_OK) {
            state.online = false;
            state.calibration_left = 0;
            if (state.enabled) stop_locked(3);
            else if (!state.reason) state.reason = 3;
            previous = 0;
            portEXIT_CRITICAL(&lock);
            vTaskDelayUntil(&deadline, period);
            continue;
        }
        // The MPU updates at 1 kHz, so every control tick sees a new sample unless the sensor stalled.
        if (!(data[0] & 1)) {
            if (state.sample_us && now - state.sample_us > MAX_SAMPLE_MS * 1000ULL) {
                state.online = false;
                if (state.enabled) stop_locked(5);
            }
            portEXIT_CRITICAL(&lock);
            vTaskDelayUntil(&deadline, period);
            continue;
        }

        int16_t sensor[7], raw[7];
        for (int i = 0; i < 7; i++) sensor[i] = (int16_t)((data[1 + i * 2] << 8) | data[2 + i * 2]);
        // Reordered to body axes, so the log's raw values need only the signs.
        raw[3] = sensor[3];
        for (int i = 0; i < 3; i++) {
            raw[i] = sensor[sensor_axis[i]];
            raw[4 + i] = sensor[4 + sensor_axis[i]];
        }
        float dt = previous ? (now - previous) / 1000000.0f : 1.0f / SAMPLE_HZ;
        if (voltage_mv) {
            float volts = voltage_mv / 1000.0f;
            battery_v = battery_v ? battery_v + (volts - battery_v) * (1 - expf(-dt / VOLTAGE_TAU)) : volts;
        }
        if (state.enabled && previous && now - previous > MAX_SAMPLE_MS * 1000ULL) stop_locked(5);
        for (int i = 0; i < 3; i++) {
            state.accel[i] = (raw[i] * config.accel_scale[i] - state.accel_offset[i]) * state.accel_gain[i];
            state.gyro[i] = raw[4 + i] * config.gyro_scale[i];
            if (state.calibration_left) {
                state.calibration_sum[i] += state.gyro[i];
                state.gravity_sum[i] += state.accel[i];
            }
        }
        if (state.calibration_left && --state.calibration_left == 0) {
            float bias[3];
            for (int i = 0; i < 3; i++) bias[i] = state.calibration_sum[i] / state.calibration_count;
            float pose_zero[] = {state.pose_zero.roll * DEG, state.pose_zero.pitch * DEG};
            // A result the settings table cannot hold is dropped: the IMU stays uncalibrated, as calibrate() left it.
            if (settings_calibration_fits(bias, pose_zero)) {
                memcpy(state.bias, bias, sizeof(bias));
                state.attitude = (Attitude){atan2f(state.gravity_sum[1], state.gravity_sum[2]),
                                            atan2f(-state.gravity_sum[0], hypotf(state.gravity_sum[1], state.gravity_sum[2]))};
                state.controller = (Controller){0};
                state.calibrated = true;
                state.calibration_ready = true;
            }
        }
        for (int i = 0; i < 3; i++) state.gyro[i] -= state.bias[i];
        if (!previous) {
            state.attitude.roll = atan2f(state.accel[1], state.accel[2]);
            state.attitude.pitch = atan2f(-state.accel[0], hypotf(state.accel[1], state.accel[2]));
        }
        Motion motion = {state.accel[0], state.accel[1], state.accel[2],
                         state.gyro[0], state.gyro[1], state.gyro[2], dt,
                         state.throttle[0] || state.throttle[1] ? config.powered_tau : config.gravity_tau};
        flight_attitude(&state.attitude, &motion);
        state.online = true;
        state.temperature = raw[3] / 340.0f + 36.53f;
        state.sample_us = now;
        previous = now;
        uint64_t started = esp_timer_get_time();
        Attitude pose = relative_attitude(&state);
        if (state.enabled && (fabsf(pose.roll) > config.tip_limit ||
                              fabsf(pose.pitch) > config.tip_limit)) stop_locked(4);
        // Measured specific force along vertical, in g: the accelerometer onto the IMU attitude's up axis.
        float vertical = -state.accel[0] * sinf(state.attitude.pitch) +
                         cosf(state.attitude.pitch) * (state.accel[1] * sinf(state.attitude.roll) + state.accel[2] * cosf(state.attitude.roll));
        if (state.enabled) {
            command.roll = pose.roll;
            command.pitch = pose.pitch;
            command.gx = state.gyro[0]; command.gy = state.gyro[1]; command.gz = state.gyro[2];
            command.dt = dt;
            // Propeller thrust at a fixed throttle goes with the square of battery voltage, so the table's newtons are scaled to the measured voltage: a sagging pack gets more throttle for the same force.
            Command scaled = command;
            if (hover_mode && hover.bias > 0.5f) {
                CameraPose camera;
                HoverPose seen, *tag = NULL;
                // Every frame whose tag corners the camera trusted, whether or not it settled the tilt: hover flies on the tag centre, which the tilt does not affect, and takes the wall's normal (the tag's z axis in the camera frame) only from an unambiguous tilt.
                if (command.hover_camera && camera_candidates(&camera, HOVER_POSE_US)) {
                    seen = (HoverPose){.sequence = camera.sequence, .capture_us = camera.capture_us,
                                       .tag = {camera.tag_position_m[0], camera.tag_position_m[1], camera.tag_position_m[2]}};
                    if (camera.valid == 1) memcpy(seen.normal, &camera.rotation[6], sizeof(seen.normal));
                    tag = &seen;
                }
                float flying = command.base > 0 ? 2 * flight_interpolate(command.base, &flight_thrust[0][0], flight_thrust_count, 0) : 0;
                if (hover.engaged) flying = hover.force;
                // Centring climbs looking for the tag, which only makes sense with the camera on.
                HoverGains gains = command.hover;
                if (!command.hover_camera) gains.center = 0;
                gains.pitch_trim = command.pitch_trim;
                HoverOutput out;
                // Heading rate about vertical from the body rates (positive left).
                float heading_rate = (state.gyro[1] * sinf(pose.roll) + state.gyro[2] * cosf(pose.roll)) / cosf(pose.pitch);
                hover_update(&hover, &gains, hover_mode, tag, now, dt, pose.roll, pose.pitch, heading_rate, vertical, flying, &out);
                scaled.base = out.force > 0 ? fminf(HOVER_THROTTLE, flight_interpolate(out.force / 2 / cosf(command.pitch_trim), &flight_thrust[0][0], flight_thrust_count, 1)) : 0;
                scaled.target_roll = out.roll;
                scaled.target_pitch = out.pitch;
                scaled.target_yaw = out.yaw_rate;
                // Once flying on the tag, hover damps the velocity itself, measured by the camera and dead-reckoned through losses: flight_math's damping is off, since its leaky integral of attitude error would cancel any steady tilt hover asks for, and its velocity follows hover's, so it would resume from the real velocity.
                if (out.tracking) {
                    state.controller.velocity[0] = out.velocity[0];
                    state.controller.velocity[1] = out.velocity[1];
                    scaled.velocity_damping = 0;
                }
            }
            flown = scaled;
            if (command.thrust_voltage > 0 && battery_v > 0) {
                float ratio = limit(battery_v / command.thrust_voltage, 1 - VOLTAGE_RANGE, 1 + VOLTAGE_RANGE);
                scaled.left_thrust_scale *= ratio * ratio;
                scaled.right_thrust_scale *= ratio * ratio;
            }
            flight_control(&state.controller, &scaled, &flight_thrust[0][0], flight_thrust_count);
            state.servo[0] = limit(state.controller.left_angle * DEG, -SERVO_LIMIT, SERVO_LIMIT);
            state.servo[1] = limit(state.controller.right_angle * DEG, -SERVO_LIMIT, SERVO_LIMIT);
            state.throttle[0] = lroundf(limit(state.controller.left, 0, THROTTLE_MAX));
            state.throttle[1] = lroundf(limit(state.controller.right, 0, THROTTLE_MAX));
            for (int side = 0; side < 2; side++) {
                write_servo(side);
                pulse(side * 2 + 1, 1000 + state.throttle[side]);
            }
        }
        if (!state.enabled || !hover_mode) {
            flown = command;
            hover_rest(&hover, vertical, state.throttle[0] || state.throttle[1], dt);
        }
        uint32_t duration = esp_timer_get_time() - started;
        if (recording) capture(now, raw, duration);
        portEXIT_CRITICAL(&lock);
        vTaskDelayUntil(&deadline, period);
    }
}

// The update service writes flash only while this holds: flash writes stall this core's code fetches.
bool flight_runtime_idle(void) {
    portENTER_CRITICAL(&lock);
    bool idle = !state.enabled && !command.enabled && !state.throttle[0] && !state.throttle[1];
    portEXIT_CRITICAL(&lock);
    return idle;
}

// A target packet from the laptop is the controller heartbeat; the 500 ms watchdog stops the motors when they cease.
bool flight_runtime_targets(const float *targets) {
    for (int i = 0; i < 9; i++) {
        if (!isfinite(targets[i])) return false;
    }
    if (targets[0] < 0 || fabsf(targets[1]) > TARGET_TILT || fabsf(targets[2]) > TARGET_TILT ||
        fabsf(targets[3]) > TARGET_RATE || targets[6] < 0 || targets[7] < 0 ||
        !(targets[8] == HOVER_OFF || targets[8] == HOVER_HOLD || targets[8] == HOVER_LAND || targets[8] == HOVER_LAND_FAST))
        return false;
    portENTER_CRITICAL(&lock);
    bool active = running && targets[0] <= THROTTLE_MAX && targets[6] <= THROTTLE_MAX && targets[7] <= THROTTLE_MAX &&
                  fabsf(targets[4]) <= SERVO_LIMIT && fabsf(targets[5]) <= SERVO_LIMIT;
    if (active) {
        state.heartbeat_us = esp_timer_get_time();
        command.base = targets[0];
        command.target_roll = targets[1] / DEG;
        command.target_pitch = targets[2] / DEG;
        command.target_yaw = targets[3] / DEG;
        hover_mode = targets[8];
        if (!targets[6] && !targets[7]) state.latched = false;
        // During maintenance the disarmed outputs stay where they are: zero throttle, servos still.
        if (!state.enabled && !maintenance) {
            for (int side = 0; side < 2; side++) {
                if (!targets[4 + side]) state.servo_held[side] = false;
                if (!state.servo_held[side]) state.servo[side] = targets[4 + side];
                write_servo(side);
                state.throttle[side] = state.latched ? 0 : lroundf(targets[6 + side]);
                pulse(side * 2 + 1, 1000 + state.throttle[side]);
            }
        }
    }
    portEXIT_CRITICAL(&lock);
    return active;
}

// The only way to arm: an explicit request after any stop, with the streamed collective at zero. Arming starts a recording, so it also needs the laptop's recorder and the previous recording fully sent.
uint32_t flight_runtime_stop_generation(void) {
    portENTER_CRITICAL(&lock);
    uint32_t generation = stop_generation;
    portEXIT_CRITICAL(&lock);
    return generation;
}

// Every check, including the stop count from when the request arrived, runs under the lock that stops take, so a stop at any point after the request arrived refuses it.
const char *flight_runtime_arm(bool recorder, bool calibration_unsaved, uint32_t expected_generation,
                               uint32_t arm_sequence) {
    portENTER_CRITICAL(&lock);
    ArmCheck check = {
        .running = running, .online = state.online, .calibrated = state.calibrated,
        .calibrating = state.calibration_left > 0, .enabled = state.enabled,
        .throttle = state.throttle[0] || state.throttle[1], .base = command.base != 0, .recorder = recorder,
        .log_pending = log_head != log_tail, .calibration_ready = state.calibration_ready,
        .calibration_unsaved = calibration_unsaved, .maintenance = maintenance,
        .expected_generation = expected_generation, .generation = stop_generation,
        .arm_sequence = arm_sequence, .barrier = stop_barrier,
    };
    const char *error = arm_refusal(&check);
    // Hover takes off by itself, so it is switched on only after arming.
    if (!error && hover_mode) error = "Hover mode must be off to arm";
    if (!error) {
        command.enabled = 1;
        state.enabled = true;
        state.reason = 0;
        flight_reset(&state.controller, &command);
        state.heartbeat_us = esp_timer_get_time();
        log_head = log_tail = log_samples = log_dropped = log_elapsed = 0;
        log_session = esp_random() | 1;
        log_started = esp_timer_get_time();
        log_active = true;
    }
    portEXIT_CRITICAL(&lock);
    return error;
}

void flight_runtime_log(LogState *log) {
    portENTER_CRITICAL(&lock);
    *log = (LogState){
        .session = log_session, .started_us = log_started, .samples = log_samples, .dropped = log_dropped,
        .elapsed_us = log_active ? esp_timer_get_time() - log_started : log_elapsed,
        .pending = log_head - log_tail, .active = log_active, .reason = state.reason, .calibrated = state.calibrated,
        .bias = {state.bias[0], state.bias[1], state.bias[2]},
        .pose_zero = {state.pose_zero.roll * DEG, state.pose_zero.pitch * DEG},
    };
    portEXIT_CRITICAL(&lock);
}

// Copies up to count of the oldest records without removing them; the radio consumes them once acknowledged.
size_t flight_runtime_read_log(uint8_t *output, size_t count) {
    portENTER_CRITICAL(&lock);
    if (count > log_head - log_tail) count = log_head - log_tail;
    for (size_t i = 0; i < count; i++) memcpy(output + i * sizeof(Record), &records[(log_tail + i) % LOG_CAPACITY], sizeof(Record));
    portEXIT_CRITICAL(&lock);
    return count;
}

void flight_runtime_consume_log(size_t count) {
    portENTER_CRITICAL(&lock);
    log_tail += count;
    portEXIT_CRITICAL(&lock);
}

static void accel_correction(const float *values) {
    for (int i = 0; i < 3; i++) {
        state.accel_offset[i] = values[SETTING_accel_offset_x + i];
        state.accel_gain[i] = values[SETTING_accel_gain_x + i];
    }
}

void flight_runtime_apply_settings(void) {
    float values[SETTINGS];
    settings_copy(values);
    portENTER_CRITICAL(&lock);
    settings_apply(&command, values);
    accel_correction(values);
    state.pose_zero = (Attitude){values[SETTING_pose_zero_roll] / DEG, values[SETTING_pose_zero_pitch] / DEG};
    portEXIT_CRITICAL(&lock);
}

// Stops the motors and averages two seconds of gyro for the bias, stored once it completes; gravity only re-seeds the attitude estimate.
const char *flight_runtime_calibrate(void) {
    portENTER_CRITICAL(&lock);
    bool active = running;
    if (active) {
        stop_locked(6);
        state.calibrated = state.calibration_ready = false;
        memset(state.calibration_sum, 0, sizeof(state.calibration_sum));
        memset(state.gravity_sum, 0, sizeof(state.gravity_sum));
        state.calibration_left = state.calibration_count = 2 * SAMPLE_HZ;
    }
    portEXIT_CRITICAL(&lock);
    return active ? NULL : "IMU offline";
}

bool flight_runtime_take_calibration(float *bias, float *pose_zero, float *sensor_frame) {
    portENTER_CRITICAL(&lock);
    bool ready = state.calibration_ready;
    state.calibration_ready = false;
    memcpy(bias, state.bias, sizeof(state.bias));
    pose_zero[0] = state.pose_zero.roll * DEG;
    pose_zero[1] = state.pose_zero.pitch * DEG;
    *sensor_frame = frame();
    portEXIT_CRITICAL(&lock);
    return ready;
}

// Keeps the current pulse and makes the servo's commanded angle its new zero, only if the settings table can hold the new offset; otherwise nothing changes. error is scratch space for the message.
const char *flight_runtime_zero_servo(int side, float *offset, char *error, size_t size) {
    int index = side ? SETTING_servo_offset_right : SETTING_servo_offset_left;
    portENTER_CRITICAL(&lock);
    bool busy = state.enabled || state.throttle[0] || state.throttle[1];
    *offset = state.offsets[side] + state.servo[side];
    bool fits = settings_valid(index, *offset);
    if (!busy && fits) {
        state.offsets[side] = *offset;
        state.servo[side] = 0;
        state.servo_held[side] = true;
    }
    portEXIT_CRITICAL(&lock);
    // Formatting allocates, so the message is built outside the lock.
    if (busy) return "Stop the motors before zeroing a servo";
    return fits ? NULL : settings_validate(index, *offset, error, size);
}

// Holds the outputs at zero for a flash write: succeeds only while disarmed with zero throttle.
bool flight_runtime_begin_maintenance(void) {
    portENTER_CRITICAL(&lock);
    bool ok = !state.enabled && !state.throttle[0] && !state.throttle[1];
    if (ok) maintenance = true;
    portEXIT_CRITICAL(&lock);
    return ok;
}

// Manual throttle stays latched at zero until the laptop streams zero throttle again.
void flight_runtime_end_maintenance(void) {
    portENTER_CRITICAL(&lock);
    maintenance = false;
    state.latched = true;
    portEXIT_CRITICAL(&lock);
}

// A laptop STOP: stops at once, whatever its sequence, and raises the barrier arm packets must be newer than.
void flight_runtime_command_stop(uint32_t sequence) {
    portENTER_CRITICAL(&lock);
    stop_locked(1);
    stop_barrier_update(&stop_barrier, sequence);
    portEXIT_CRITICAL(&lock);
}

void flight_runtime_status(Status *status) {
    portENTER_CRITICAL(&lock);
    State copy = state;
    float target[] = {command.target_roll * DEG, command.target_pitch * DEG, command.target_yaw * DEG, command.base};
    bool recording = log_active;
    uint32_t samples = log_samples, dropped = log_dropped;
    portEXIT_CRITICAL(&lock);
    Attitude pose = relative_attitude(&copy);
    bool fresh = copy.online && esp_timer_get_time() - copy.sample_us < 100000;
    status->flags = fresh | (copy.calibration_left > 0 || copy.calibration_ready) << 1 | copy.calibrated << 2 | copy.enabled << 3 |
                    recording << 4 | copy.latched << 6;
    status->reason = copy.reason;
    for (int i = 0; i < 3; i++) {
        status->accel[i] = copy.accel[i];
        status->gyro[i] = copy.gyro[i] * DEG;
    }
    status->roll = pose.roll * DEG;
    status->pitch = pose.pitch * DEG;
    status->temperature = copy.temperature;
    for (int side = 0; side < 2; side++) {
        status->servo[side] = copy.servo[side];
        status->throttle[side] = copy.throttle[side];
    }
    memcpy(status->target, target, sizeof(target));
    status->log_samples = samples;
    status->log_dropped = dropped;
}

void flight_runtime_set_voltage(uint16_t millivolts) {
    portENTER_CRITICAL(&lock);
    voltage_mv = millivolts;
    portEXIT_CRITICAL(&lock);
}

void flight_runtime_outputs(void) {
    float values[SETTINGS];
    settings_copy(values);
    state.offsets[0] = values[SETTING_servo_offset_left];
    state.offsets[1] = values[SETTING_servo_offset_right];
    for (int kind = 0; kind < 2; kind++) {
        ledc_timer_config_t timer = {.speed_mode = LEDC_HIGH_SPEED_MODE, .duty_resolution = PWM_BITS,
                                     .timer_num = kind, .freq_hz = kind ? ESC_HZ : SERVO_HZ, .clk_cfg = LEDC_AUTO_CLK};
        ESP_ERROR_CHECK(ledc_timer_config(&timer));
    }
    for (int index = 0; index < 4; index++) {
        int side = index / 2, esc = index % 2;
        float us = esc ? 1000 : 1500 + state.offsets[side] * 1000 / 90;
        ledc_channel_config_t channel = {.gpio_num = pins[index], .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = index,
                                         .timer_sel = esc, .duty = lroundf(us * ticks[index])};
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
}

// Wakes the MPU6050 with 1 kHz output, so each control tick reads a sample at most 1 ms old; the DLPF is the anti-alias filter.
static bool configure_sensor(void) {
    i2c_config_t bus = {.mode = I2C_MODE_MASTER, .sda_io_num = SDA, .scl_io_num = SCL,
                        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
                        .master.clk_speed = 400000};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &bus));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    bool ok = write_mpu(0x6b, 1) == ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t registers[][2] = {
        {0x19, 0}, {0x1a, IMU_DLPF}, {0x1b, 0}, {0x1c, 0}, {0x37, 0x20}, {0x38, 1},
    };
    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
        if (write_mpu(registers[i][0], registers[i][1]) != ESP_OK) ok = false;
    }
    return ok;
}

void flight_runtime_start(void) {
    // Accelerometer at ±2 g (16384 per g), gyro at ±250 °/s (131 per °/s), both mapped to body axes.
    for (int i = 0; i < 3; i++) {
        config.accel_scale[i] = sensor_to_body[i] / 16384.0f;
        config.gyro_scale[i] = sensor_to_body[i] * 3.14159265358979323846 / (180 * 131);
    }
    // Equivalent to retaining 0.99 / 0.9998 of gyro integration every 10 ms.
    config.gravity_tau = -0.01 / log(0.99);
    config.powered_tau = -0.01 / log(0.9998);
    config.tip_limit = TIP_LIMIT / DEG;
    float values[SETTINGS];
    settings_copy(values);
    // A calibration is valid only in the sensor frame it was taken in. Arming and the status both read state.calibrated, so a rejected calibration blocks arming.
    float pose_zero[2];
    if (settings_calibration(frame(), state.bias, pose_zero)) {
        state.pose_zero = (Attitude){pose_zero[0] / DEG, pose_zero[1] / DEG};
        state.calibrated = true;
    }
    // The fixed model; the settings store supplies every tunable value.
    command.model.servo_min = -SERVO_LIMIT / DEG;
    command.model.servo_max = SERVO_LIMIT / DEG;
    command.model.left_servo_direction = 1;
    command.model.right_servo_direction = -1;
    settings_apply(&command, values);
    accel_correction(values);
    state.heartbeat_us = esp_timer_get_time();
    // An initial I2C timeout can leave the sensor idle; one fresh driver install and configuration recovers it.
    sensor_configured = configure_sensor();
    if (!sensor_configured) {
        ESP_ERROR_CHECK(i2c_driver_delete(I2C_NUM_0));
        sensor_configured = configure_sensor();
    }
    portENTER_CRITICAL(&lock);
    stop_locked(0);
    for (int side = 0; side < 2; side++) write_servo(side);
    running = true;
    portEXIT_CRITICAL(&lock);
    if (xTaskCreatePinnedToCore(runtime_task, "flight", 4096, NULL, 5, NULL, 1) != pdPASS) abort();
}
