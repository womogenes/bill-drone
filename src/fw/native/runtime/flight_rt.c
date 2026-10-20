#include <math.h>
#include <stdint.h>
#include <string.h>
#include "py/runtime.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "driver/i2c.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "../flight_math.h"
#include "runtime.h"
#include "settings.h"
#include "arming.h"

#define LOG_CAPACITY 200
#define TABLE_CAPACITY 64
#define DEG (180.0f / 3.14159265358979323846f)
// Streamed target limits: degrees and degrees per second.
#define TARGET_TILT 15
#define TARGET_RATE 45

typedef struct __attribute__((packed)) {
    uint32_t elapsed, sequence;
    int16_t raw[7], attitude[2];
    uint16_t throttle[2];
    int16_t servo[2], target[3];
    uint16_t base, control_us, flags, voltage;
} Record;
_Static_assert(sizeof(Record) == 48, "Flight record layout");

typedef struct {
    int sda, scl, address, sample_hz, heartbeat_ms, throttle_max, max_sample_ms, dlpf;
    float accel_scale[3], gyro_scale[3], temperature_scale, temperature_offset;
    float gravity_tau, powered_tau, tip_limit, servo_limit;
    uintptr_t pwm[4];
    float ticks[4];
} Config;

typedef struct {
    // latched: manual throttle stays zero after a stop until the laptop sends zero throttle.
    // servo_held: a zeroed servo ignores streamed angles until the laptop sends zero for it.
    // calibration_ready: a finished calibration waits for the radio's update task to store it.
    bool online, enabled, calibrated, latched, servo_held[2], calibration_ready;
    int reason, core, calibration_left, calibration_count;
    float accel[3], gyro[3], bias[3], calibration_sum[3], gravity_sum[3];
    float temperature, servo[2], offsets[2];
    int throttle[2];
    Attitude attitude, pose_zero;
    Controller controller;
    uint64_t sample_us, heartbeat_us, samples, total_control_us;
    uint32_t max_control_us;
} State;

static Config config;
static State state;
static Command command;
static float thrust[TABLE_CAPACITY * 2];
static size_t thrust_count;
static Record records[LOG_CAPACITY];
static uint32_t log_head, log_tail, log_samples, log_dropped, log_elapsed, log_session;
static uint64_t log_started;
static bool log_active, running, i2c_installed, sensor_configured;
// maintenance: a flash or NVS write is in progress, so outputs are held at zero and arming is refused.
static bool maintenance;
// Counts every stop; an arm request carries the count from when it arrived.
static uint32_t stop_generation;
// The newest laptop STOP since boot (each boot is a new radio session).
static StopBarrier stop_barrier;
static uint16_t voltage_mv;
static TaskHandle_t task;
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

static float limit(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

// Caller holds lock so an emergency stop cannot race a later duty write.
static void pulse(int index, float us) {
    if (!config.pwm[index]) return;
    volatile uint32_t *reg = (volatile uint32_t *)config.pwm[index];
    reg[3] &= 0x7fffffff;
    reg[2] = (uint32_t)lroundf(us * config.ticks[index]) << 4;
    reg[3] |= 1U << 31;
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
    return i2c_master_write_to_device(I2C_NUM_0, config.address, data, sizeof(data), pdMS_TO_TICKS(10));
}

// Sign bits (x 1, y 2, z 4) of the sensor-to-body mapping, as the IMU scales from Python carry it.
static float frame(void) {
    return (config.accel_scale[0] < 0) | (config.accel_scale[1] < 0) << 1 | (config.accel_scale[2] < 0) << 2;
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
    record->target[0] = lroundf(command.target_roll * DEG * 100);
    record->target[1] = lroundf(command.target_pitch * DEG * 100);
    record->target[2] = lroundf(command.target_yaw * DEG * 100);
    record->base = command.base;
    record->control_us = duration > 65535 ? 65535 : duration;
    record->flags = state.enabled;
    record->voltage = voltage_mv;
}

static void runtime_task(void *arg) {
    TickType_t deadline = xTaskGetTickCount();
    TickType_t period = configTICK_RATE_HZ / config.sample_hz;
    uint64_t previous = 0;
    while (true) {
        portENTER_CRITICAL(&lock);
        bool active = running;
        portEXIT_CRITICAL(&lock);
        if (!active) break;

        uint8_t reg = 0x3a, data[15];
        esp_err_t result = sensor_configured
            ? i2c_master_write_read_device(I2C_NUM_0, config.address, &reg, 1,
                                          data, sizeof(data), pdMS_TO_TICKS(10))
            : ESP_ERR_INVALID_STATE;
        uint64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&lock);
        if (!running) {
            portEXIT_CRITICAL(&lock);
            break;
        }
        bool recording = log_active;
        if ((state.enabled || state.throttle[0] || state.throttle[1]) &&
            now - state.heartbeat_us > config.heartbeat_ms * 1000ULL) stop_locked(2);

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
            if (state.sample_us && now - state.sample_us > config.max_sample_ms * 1000ULL) {
                state.online = false;
                if (state.enabled) stop_locked(5);
            }
            portEXIT_CRITICAL(&lock);
            vTaskDelayUntil(&deadline, period);
            continue;
        }

        int16_t raw[7];
        for (int i = 0; i < 7; i++) raw[i] = (int16_t)((data[1 + i * 2] << 8) | data[2 + i * 2]);
        float dt = previous ? (now - previous) / 1000000.0f : 1.0f / config.sample_hz;
        if (state.enabled && previous && now - previous > config.max_sample_ms * 1000ULL) stop_locked(5);
        for (int i = 0; i < 3; i++) {
            state.accel[i] = raw[i] * config.accel_scale[i];
            state.gyro[i] = raw[4 + i] * config.gyro_scale[i];
            if (state.calibration_left) {
                state.calibration_sum[i] += state.gyro[i];
                state.gravity_sum[i] += state.accel[i];
            }
        }
        if (state.calibration_left && --state.calibration_left == 0) {
            float bias[3];
            for (int i = 0; i < 3; i++) bias[i] = state.calibration_sum[i] / state.calibration_count;
            Attitude zero = {atan2f(state.gravity_sum[1], state.gravity_sum[2]),
                             atan2f(-state.gravity_sum[0], hypotf(state.gravity_sum[1], state.gravity_sum[2]))};
            float pose_zero[] = {zero.roll * DEG, zero.pitch * DEG};
            // A result the settings table cannot hold is dropped: the IMU stays uncalibrated, as calibrate() left it.
            if (settings_calibration_fits(bias, pose_zero)) {
                memcpy(state.bias, bias, sizeof(bias));
                state.pose_zero = zero;
                state.attitude = state.pose_zero;
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
        state.temperature = raw[3] * config.temperature_scale + config.temperature_offset;
        state.sample_us = now;
        state.core = xPortGetCoreID();
        previous = now;
        uint64_t started = esp_timer_get_time();
        Attitude pose = relative_attitude(&state);
        if (state.enabled && (fabsf(pose.roll) > config.tip_limit ||
                              fabsf(pose.pitch) > config.tip_limit)) stop_locked(4);
        if (state.enabled) {
            command.roll = pose.roll;
            command.pitch = pose.pitch;
            command.gx = state.gyro[0]; command.gy = state.gyro[1]; command.gz = state.gyro[2];
            command.dt = dt;
            flight_control(&state.controller, &command, thrust, thrust_count);
            state.servo[0] = limit(state.controller.left_angle * DEG, -config.servo_limit, config.servo_limit);
            state.servo[1] = limit(state.controller.right_angle * DEG, -config.servo_limit, config.servo_limit);
            state.throttle[0] = lroundf(limit(state.controller.left, 0, config.throttle_max));
            state.throttle[1] = lroundf(limit(state.controller.right, 0, config.throttle_max));
            for (int side = 0; side < 2; side++) {
                write_servo(side);
                pulse(side * 2 + 1, 1000 + state.throttle[side]);
            }
        }
        uint32_t duration = esp_timer_get_time() - started;
        state.samples++;
        state.total_control_us += duration;
        if (duration > state.max_control_us) state.max_control_us = duration;
        if (recording) capture(now, raw, duration);
        portEXIT_CRITICAL(&lock);
        vTaskDelayUntil(&deadline, period);
    }
    portENTER_CRITICAL(&lock);
    task = NULL;
    portEXIT_CRITICAL(&lock);
    vTaskDelete(NULL);
}

void flight_runtime_stop(void) {
    portENTER_CRITICAL(&lock);
    running = false;
    stop_locked(1);
    portEXIT_CRITICAL(&lock);
    while (true) {
        portENTER_CRITICAL(&lock);
        bool alive = task != NULL;
        portEXIT_CRITICAL(&lock);
        if (!alive) break;
        vTaskDelay(1);
    }
    if (i2c_installed) {
        i2c_driver_delete(I2C_NUM_0);
        i2c_installed = false;
    }
    memset(config.pwm, 0, sizeof(config.pwm));
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
    for (int i = 0; i < 8; i++) {
        if (!isfinite(targets[i])) return false;
    }
    if (targets[0] < 0 || fabsf(targets[1]) > TARGET_TILT || fabsf(targets[2]) > TARGET_TILT ||
        fabsf(targets[3]) > TARGET_RATE || targets[6] < 0 || targets[7] < 0)
        return false;
    portENTER_CRITICAL(&lock);
    // The limits come from start(), which clears running while it changes them.
    bool active = running && targets[0] <= config.throttle_max && targets[6] <= config.throttle_max &&
                  targets[7] <= config.throttle_max && fabsf(targets[4]) <= config.servo_limit &&
                  fabsf(targets[5]) <= config.servo_limit;
    if (active) {
        state.heartbeat_us = esp_timer_get_time();
        command.base = targets[0];
        command.target_roll = targets[1] / DEG;
        command.target_pitch = targets[2] / DEG;
        command.target_yaw = targets[3] / DEG;
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

// The only way to arm: an explicit request after any stop, with the streamed collective at zero. Arming starts a
// recording, so it also needs the laptop's recorder and the previous recording fully sent.
uint32_t flight_runtime_stop_generation(void) {
    portENTER_CRITICAL(&lock);
    uint32_t generation = stop_generation;
    portEXIT_CRITICAL(&lock);
    return generation;
}

// Every check, including the stop count from when the request arrived, runs under the lock that stops take, so a
// stop at any point after the request arrived refuses it.
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

void flight_runtime_apply_settings(void) {
    float values[SETTINGS];
    settings_copy(values);
    portENTER_CRITICAL(&lock);
    settings_apply(&command, values);
    portEXIT_CRITICAL(&lock);
}

// Stops the motors and averages two seconds of gyro and gravity; the result is stored once it completes.
const char *flight_runtime_calibrate(void) {
    portENTER_CRITICAL(&lock);
    bool active = running;
    if (active) {
        stop_locked(6);
        state.calibrated = state.calibration_ready = false;
        memset(state.calibration_sum, 0, sizeof(state.calibration_sum));
        memset(state.gravity_sum, 0, sizeof(state.gravity_sum));
        state.calibration_left = state.calibration_count = 2 * config.sample_hz;
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

// Keeps the current pulse and makes the servo's commanded angle its new zero, only if the settings table can hold
// the new offset; otherwise nothing changes. error is scratch space for the message.
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

static mp_obj_t field(mp_obj_t dict, qstr key) {
    return mp_obj_dict_get(dict, MP_OBJ_NEW_QSTR(key));
}
static void floats(mp_obj_t object, float *values, size_t expected) {
    size_t count; mp_obj_t *items;
    mp_obj_get_array(object, &count, &items);
    if (count != expected) mp_raise_ValueError(MP_ERROR_TEXT("Invalid vector length"));
    for (size_t i = 0; i < count; i++) {
        values[i] = mp_obj_get_float(items[i]);
        if (!isfinite(values[i])) mp_raise_ValueError(MP_ERROR_TEXT("Expected finite values"));
    }
}

// Gains, trims and model only: arming, setpoints and stops never come from here.
static mp_obj_t configure(mp_obj_t buffer, mp_obj_t table) {
    mp_buffer_info_t a, b;
    mp_get_buffer_raise(buffer, &a, MP_BUFFER_READ);
    mp_get_buffer_raise(table, &b, MP_BUFFER_READ);
    if (a.len != sizeof(Command) || b.len < 16 || b.len % 8 || b.len > sizeof(thrust))
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid controller buffers"));
    Command next; memcpy(&next, a.buf, sizeof(next));
    for (size_t i = 0; i < sizeof(next) / sizeof(float); i++) {
        float value; memcpy(&value, (uint8_t *)&next + i * sizeof(float), sizeof(float));
        if (!isfinite(value)) mp_raise_ValueError(MP_ERROR_TEXT("Invalid controller value"));
    }
    float values[SETTINGS];
    settings_copy(values);
    portENTER_CRITICAL(&lock);
    next.enabled = command.enabled;
    next.base = command.base;
    next.target_roll = command.target_roll;
    next.target_pitch = command.target_pitch;
    next.target_yaw = command.target_yaw;
    command = next;
    // Python supplies the thrust table and the fixed model; tunable values come from the settings store.
    settings_apply(&command, values);
    memcpy(thrust, b.buf, b.len);
    thrust_count = b.len / 8;
    portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(configure_obj, configure);


static mp_obj_t start(mp_obj_t settings) {
    flight_runtime_stop();
    Config next = {0};
#define INT_FIELD(name) next.name = mp_obj_get_int(field(settings, MP_QSTR_##name))
#define FLOAT_FIELD(name) next.name = mp_obj_get_float(field(settings, MP_QSTR_##name))
    INT_FIELD(max_sample_ms); INT_FIELD(dlpf);
    INT_FIELD(sda); INT_FIELD(scl); INT_FIELD(address); INT_FIELD(sample_hz); INT_FIELD(heartbeat_ms); INT_FIELD(throttle_max);
    FLOAT_FIELD(temperature_scale); FLOAT_FIELD(temperature_offset); FLOAT_FIELD(gravity_tau); FLOAT_FIELD(powered_tau);
    FLOAT_FIELD(tip_limit); FLOAT_FIELD(servo_limit);
#undef INT_FIELD
#undef FLOAT_FIELD
    floats(field(settings, MP_QSTR_accel_scale), next.accel_scale, 3);
    floats(field(settings, MP_QSTR_gyro_scale), next.gyro_scale, 3);
    size_t count; mp_obj_t *items;
    mp_obj_get_array(field(settings, MP_QSTR_pwm), &count, &items);
    // The control period is a whole number of RTOS ticks; the tick ISR paces it with microsecond jitter.
    if (count != 4 || next.sample_hz < 1 || next.sample_hz > configTICK_RATE_HZ ||
        configTICK_RATE_HZ % next.sample_hz || next.dlpf < 1 || next.dlpf > 6 ||
        next.max_sample_ms * next.sample_hz <= 1000)
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid runtime configuration"));
    for (int i = 0; i < 4; i++) {
        size_t n; mp_obj_t *pair;
        mp_obj_get_array(items[i], &n, &pair);
        if (n != 2) mp_raise_ValueError(MP_ERROR_TEXT("Invalid PWM configuration"));
        next.pwm[i] = mp_obj_get_int_truncated(pair[0]);
        next.ticks[i] = mp_obj_get_float(pair[1]);
        if (next.pwm[i] < 0x3ff59000 || next.pwm[i] > 0x3ff5908c ||
            (next.pwm[i] - 0x3ff59000) % 20 || !(next.ticks[i] > 0))
            mp_raise_ValueError(MP_ERROR_TEXT("Invalid LEDC channel"));
    }
    config = next;
    memset(&state, 0, sizeof(state));
    float values[SETTINGS];
    settings_copy(values);
    state.offsets[0] = values[SETTING_servo_offset_left];
    state.offsets[1] = values[SETTING_servo_offset_right];
    // A calibration is valid only in the sensor frame it was taken in. Arming and the status both read
    // state.calibrated, so a rejected calibration blocks arming.
    float pose_zero[2];
    if (settings_calibration(frame(), state.bias, pose_zero)) {
        state.pose_zero = (Attitude){pose_zero[0] / DEG, pose_zero[1] / DEG};
        state.calibrated = true;
    }
    settings_apply(&command, values);
    command.enabled = command.base = 0;
    state.heartbeat_us = esp_timer_get_time();
    i2c_config_t bus = {.mode = I2C_MODE_MASTER, .sda_io_num = config.sda, .scl_io_num = config.scl,
                        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
                        .master.clk_speed = 400000};
    esp_err_t error = i2c_param_config(I2C_NUM_0, &bus);
    if (error == ESP_OK) error = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (error != ESP_OK) mp_raise_OSError(error);
    i2c_installed = true;
    sensor_configured = write_mpu(0x6b, 1) == ESP_OK;
    mp_hal_delay_ms(100);
    // 1 kHz output so each control tick reads a sample at most 1 ms old; the DLPF is the anti-alias filter.
    uint8_t registers[][2] = {
        {0x19, 0}, {0x1a, config.dlpf}, {0x1b, 0}, {0x1c, 0}, {0x37, 0x20}, {0x38, 1},
    };
    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
        if (write_mpu(registers[i][0], registers[i][1]) != ESP_OK) sensor_configured = false;
    }
    portENTER_CRITICAL(&lock);
    stop_locked(0);
    for (int side = 0; side < 2; side++) write_servo(side);
    running = true;
    portEXIT_CRITICAL(&lock);
    if (xTaskCreatePinnedToCore(runtime_task, "flight", 4096, NULL, 5, &task, 1) != pdPASS) {
        flight_runtime_stop();
        mp_raise_OSError(MP_ENOMEM);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(start_obj, start);

static mp_obj_t stop(size_t n, const mp_obj_t *args) {
    int reason = n ? mp_obj_get_int(args[0]) : 1;
    portENTER_CRITICAL(&lock); stop_locked(reason); portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(stop_obj, 0, 1, stop);
static void put(mp_obj_t dict, qstr key, mp_obj_t value) { mp_obj_dict_store(dict, MP_OBJ_NEW_QSTR(key), value); }
static mp_obj_t float_tuple(const float *values, int n, float scale) {
    mp_obj_t items[3];
    for (int i = 0; i < n; i++) items[i] = mp_obj_new_float(values[i] * scale);
    return mp_obj_new_tuple(n, items);
}
static mp_obj_t snapshot(void) {
    State copy;
    portENTER_CRITICAL(&lock); copy = state; portEXIT_CRITICAL(&lock);
    mp_obj_t dict = mp_obj_new_dict(0);
    put(dict, MP_QSTR_online, mp_obj_new_bool(copy.online));
    put(dict, MP_QSTR_calibrated, mp_obj_new_bool(copy.calibrated));
    put(dict, MP_QSTR_calibrating, mp_obj_new_bool(copy.calibration_left > 0));
    put(dict, MP_QSTR_accel, float_tuple(copy.accel, 3, 1));
    put(dict, MP_QSTR_gyro, float_tuple(copy.gyro, 3, DEG));
    put(dict, MP_QSTR_gyro_bias, float_tuple(copy.bias, 3, 1));
    Attitude pose = relative_attitude(&copy);
    put(dict, MP_QSTR_roll, mp_obj_new_float(pose.roll * DEG));
    put(dict, MP_QSTR_pitch, mp_obj_new_float(pose.pitch * DEG));
    float zero[] = {copy.pose_zero.roll, copy.pose_zero.pitch};
    put(dict, MP_QSTR_pose_zero, float_tuple(zero, 2, DEG));
    put(dict, MP_QSTR_temperature, mp_obj_new_float(copy.temperature));
    put(dict, MP_QSTR_servo, float_tuple(copy.servo, 2, 1));
    put(dict, MP_QSTR_offsets, float_tuple(copy.offsets, 2, 1));
    mp_obj_t throttles[] = {mp_obj_new_int(copy.throttle[0]), mp_obj_new_int(copy.throttle[1])};
    put(dict, MP_QSTR_throttle, mp_obj_new_tuple(2, throttles));
    put(dict, MP_QSTR_enabled, mp_obj_new_bool(copy.enabled));
    put(dict, MP_QSTR_reason, mp_obj_new_int(copy.reason));
    put(dict, MP_QSTR_age_ms, mp_obj_new_int_from_ull(copy.sample_us ? (esp_timer_get_time() - copy.sample_us) / 1000 : 0));
    put(dict, MP_QSTR_samples, mp_obj_new_int_from_ull(copy.samples));
    put(dict, MP_QSTR_total_control_us, mp_obj_new_int_from_ull(copy.total_control_us));
    put(dict, MP_QSTR_max_control_us, mp_obj_new_int_from_uint(copy.max_control_us));
    put(dict, MP_QSTR_core, mp_obj_new_int(copy.core));
    put(dict, MP_QSTR_vm_core, mp_obj_new_int(xPortGetCoreID()));
    put(dict, MP_QSTR_sample_hz, mp_obj_new_int(config.sample_hz));
    return dict;
}
static MP_DEFINE_CONST_FUN_OBJ_0(snapshot_obj, snapshot);
static mp_obj_t reset(void) {
    portENTER_CRITICAL(&lock); state.controller = (Controller){0}; flight_reset(&state.controller, &command); portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(reset_obj, reset);
static mp_obj_t set_voltage(mp_obj_t value) {
    int voltage = mp_obj_get_int(value);
    portENTER_CRITICAL(&lock); voltage_mv = voltage; portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(set_voltage_obj, set_voltage);
static mp_obj_t shutdown(void) { flight_runtime_stop(); return mp_const_none; }
static MP_DEFINE_CONST_FUN_OBJ_0(shutdown_obj, shutdown);

static const mp_rom_map_elem_t globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_flight_rt)},
#define EXPORT(name) {MP_ROM_QSTR(MP_QSTR_##name), MP_ROM_PTR(&name##_obj)}
    EXPORT(configure), EXPORT(start), EXPORT(stop), EXPORT(snapshot), EXPORT(reset), EXPORT(set_voltage), EXPORT(shutdown),
#undef EXPORT
};
static MP_DEFINE_CONST_DICT(globals, globals_table);
const mp_obj_module_t flight_rt_module = {.base = {&mp_type_module}, .globals = (mp_obj_dict_t *)&globals};
MP_REGISTER_MODULE(MP_QSTR_flight_rt, flight_rt_module);
