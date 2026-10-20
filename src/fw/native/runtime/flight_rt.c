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
#include "esp_timer.h"
#include "../flight_math.h"

#define LOG_CAPACITY 200
#define LOG_BATCH 16
#define TABLE_CAPACITY 64
#define DEG (180.0f / 3.14159265358979323846f)

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
    bool online, enabled, calibrated;
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
static uint32_t log_head, log_tail, log_samples, log_dropped, log_elapsed;
static uint64_t log_started;
static bool log_active, running, i2c_installed, sensor_configured;
static uint16_t voltage_mv, log_flags;
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
    command.enabled = command.base = 0;
    state.enabled = false;
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
    record->flags = state.enabled | log_flags;
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
        if (previous && now - previous > config.max_sample_ms * 1000ULL) stop_locked(5);
        for (int i = 0; i < 3; i++) {
            state.accel[i] = raw[i] * config.accel_scale[i];
            state.gyro[i] = raw[4 + i] * config.gyro_scale[i];
            if (state.calibration_left) {
                state.calibration_sum[i] += state.gyro[i];
                state.gravity_sum[i] += state.accel[i];
            }
        }
        if (state.calibration_left && --state.calibration_left == 0) {
            for (int i = 0; i < 3; i++) state.bias[i] = state.calibration_sum[i] / state.calibration_count;
            state.pose_zero.roll = atan2f(state.gravity_sum[1], state.gravity_sum[2]);
            state.pose_zero.pitch = atan2f(-state.gravity_sum[0], hypotf(state.gravity_sum[1], state.gravity_sum[2]));
            state.attitude = state.pose_zero;
            state.controller = (Controller){0};
            state.calibrated = true;
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
    portENTER_CRITICAL(&lock);
    bool arm = next.enabled && !state.enabled;
    bool invalid = arm && (next.base != 0 || state.throttle[0] || state.throttle[1] ||
                           !state.online || !state.calibrated || state.calibration_left || !running);
    if (!invalid) {
        if (state.enabled && !next.enabled) stop_locked(1);
        command = next;
        memcpy(thrust, b.buf, b.len);
        thrust_count = b.len / 8;
        state.enabled = next.enabled != 0;
        if (arm) { state.reason = 0; state.controller = (Controller){0}; state.heartbeat_us = esp_timer_get_time(); }
    }
    portEXIT_CRITICAL(&lock);
    if (invalid) mp_raise_ValueError(MP_ERROR_TEXT("Enable requires calibrated online IMU and zero throttle"));
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
    float offsets[2]; floats(field(settings, MP_QSTR_servo_offsets), offsets, 2);
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
    memcpy(state.offsets, offsets, sizeof(offsets));
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

static mp_obj_t manual(size_t n, const mp_obj_t *args) {
    int side = mp_obj_get_int(args[0]), kind = mp_obj_get_int(args[1]);
    float value = mp_obj_get_float(args[2]);
    if (side < 0 || side > 1 || kind < 0 || kind > 1 || !isfinite(value) ||
        (kind == 0 && fabsf(value) > config.servo_limit) || (kind == 1 && (value < 0 || value > config.throttle_max)))
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid actuator command"));
    portENTER_CRITICAL(&lock);
    bool enabled = state.enabled;
    if (!enabled) {
        if (kind == 0) { state.servo[side] = value; write_servo(side); }
        else { state.throttle[side] = lroundf(value); pulse(side * 2 + 1, 1000 + state.throttle[side]); }
    }
    portEXIT_CRITICAL(&lock);
    if (enabled) mp_raise_ValueError(MP_ERROR_TEXT("Disable control before manual commands"));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(manual_obj, 3, 3, manual);

static mp_obj_t zero_servo(mp_obj_t value) {
    int side = mp_obj_get_int(value);
    if (side < 0 || side > 1) mp_raise_ValueError(MP_ERROR_TEXT("Invalid side"));
    portENTER_CRITICAL(&lock);
    bool busy = state.enabled || state.throttle[0] || state.throttle[1];
    if (!busy) { state.offsets[side] += state.servo[side]; state.servo[side] = 0; }
    float offset = state.offsets[side];
    portEXIT_CRITICAL(&lock);
    if (busy) mp_raise_ValueError(MP_ERROR_TEXT("Stop motors before zeroing servo"));
    return mp_obj_new_float(offset);
}
static MP_DEFINE_CONST_FUN_OBJ_1(zero_servo_obj, zero_servo);

static mp_obj_t restore_calibration(mp_obj_t gyro_bias, mp_obj_t pose_zero) {
    float bias[3], pose[2];
    floats(gyro_bias, bias, 3);
    floats(pose_zero, pose, 2);
    if (fabsf(pose[0]) > 180 || fabsf(pose[1]) > 90)
        mp_raise_ValueError(MP_ERROR_TEXT("Invalid calibration pose"));
    portENTER_CRITICAL(&lock);
    bool busy = state.enabled || state.throttle[0] || state.throttle[1] || state.calibration_left;
    if (!busy) {
        memcpy(state.bias, bias, sizeof(bias));
        state.pose_zero = (Attitude){pose[0] / DEG, pose[1] / DEG};
        if (state.online) {
            state.attitude.roll = atan2f(state.accel[1], state.accel[2]);
            state.attitude.pitch = atan2f(-state.accel[0], hypotf(state.accel[1], state.accel[2]));
        }
        state.controller = (Controller){0};
        state.calibrated = true;
    }
    portEXIT_CRITICAL(&lock);
    if (busy) mp_raise_ValueError(MP_ERROR_TEXT("Stop before restoring calibration"));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(restore_calibration_obj, restore_calibration);

static mp_obj_t set_offsets(mp_obj_t values) {
    float offsets[2]; floats(values, offsets, 2);
    portENTER_CRITICAL(&lock);
    memcpy(state.offsets, offsets, sizeof(offsets));
    portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(set_offsets_obj, set_offsets);
static mp_obj_t heartbeat(void) {
    portENTER_CRITICAL(&lock); state.heartbeat_us = esp_timer_get_time(); portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(heartbeat_obj, heartbeat);
static mp_obj_t stop(size_t n, const mp_obj_t *args) {
    int reason = n ? mp_obj_get_int(args[0]) : 1;
    portENTER_CRITICAL(&lock); stop_locked(reason); portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(stop_obj, 0, 1, stop);
static mp_obj_t calibrate(mp_obj_t count) {
    int samples = mp_obj_get_int(count);
    if (samples < 1) mp_raise_ValueError(MP_ERROR_TEXT("Calibration requires samples"));
    portENTER_CRITICAL(&lock);
    stop_locked(6);
    state.calibrated = false;
    memset(state.calibration_sum, 0, sizeof(state.calibration_sum));
    memset(state.gravity_sum, 0, sizeof(state.gravity_sum));
    state.calibration_left = state.calibration_count = samples;
    portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(calibrate_obj, calibrate);
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
static mp_obj_t record_start(mp_obj_t session) {
    (void)mp_obj_get_int_truncated(session);
    portENTER_CRITICAL(&lock);
    log_head = log_tail = log_samples = log_dropped = log_elapsed = 0;
    log_flags = 0;
    log_started = esp_timer_get_time(); log_active = true;
    uint32_t started = log_started & ((1U << 30) - 1);
    portEXIT_CRITICAL(&lock);
    return mp_obj_new_int_from_uint(started);
}
static MP_DEFINE_CONST_FUN_OBJ_1(record_start_obj, record_start);
static mp_obj_t record_stop(void) {
    portENTER_CRITICAL(&lock);
    if (log_active) log_elapsed = esp_timer_get_time() - log_started;
    log_active = false;
    portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(record_stop_obj, record_stop);
static mp_obj_t record_status(void) {
    portENTER_CRITICAL(&lock);
    bool active = log_active;
    uint32_t samples = log_samples, dropped = log_dropped;
    uint32_t elapsed = log_active ? esp_timer_get_time() - log_started : log_elapsed;
    int reason = state.reason;
    uint32_t pending = log_head - log_tail;
    portEXIT_CRITICAL(&lock);
    mp_obj_t dict = mp_obj_new_dict(0);
    put(dict, MP_QSTR_active, mp_obj_new_bool(active));
    put(dict, MP_QSTR_samples, mp_obj_new_int_from_uint(samples));
    put(dict, MP_QSTR_dropped, mp_obj_new_int_from_uint(dropped));
    put(dict, MP_QSTR_time_us, mp_obj_new_int_from_uint(elapsed));
    put(dict, MP_QSTR_reason, mp_obj_new_int(reason));
    put(dict, MP_QSTR_pending, mp_obj_new_int_from_uint(pending));
    return dict;
}
static MP_DEFINE_CONST_FUN_OBJ_0(record_status_obj, record_status);
static mp_obj_t read_log(mp_obj_t value) {
    int count = mp_obj_get_int(value);
    if (count < 1 || count > LOG_BATCH) mp_raise_ValueError(MP_ERROR_TEXT("Read 1 to 16 records"));
    Record output[LOG_BATCH];
    portENTER_CRITICAL(&lock);
    if ((uint32_t)count > log_head - log_tail) count = log_head - log_tail;
    for (int i = 0; i < count; i++) output[i] = records[(log_tail + i) % LOG_CAPACITY];
    portEXIT_CRITICAL(&lock);
    return mp_obj_new_bytes((uint8_t *)output, count * sizeof(Record));
}
static MP_DEFINE_CONST_FUN_OBJ_1(read_log_obj, read_log);
static mp_obj_t consume_log(mp_obj_t value) {
    uint32_t count = mp_obj_get_int(value);
    portENTER_CRITICAL(&lock);
    bool valid = count <= log_head - log_tail;
    if (valid) log_tail += count;
    portEXIT_CRITICAL(&lock);
    if (!valid) mp_raise_ValueError(MP_ERROR_TEXT("Invalid log count"));
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(consume_log_obj, consume_log);
static mp_obj_t set_log_flags(mp_obj_t value) {
    uint16_t flags = mp_obj_get_int(value);
    portENTER_CRITICAL(&lock); log_flags = flags & ~1U; portEXIT_CRITICAL(&lock);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(set_log_flags_obj, set_log_flags);
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
    EXPORT(configure), EXPORT(start), EXPORT(manual), EXPORT(zero_servo), EXPORT(set_offsets),
    EXPORT(heartbeat), EXPORT(stop), EXPORT(calibrate), EXPORT(restore_calibration), EXPORT(snapshot), EXPORT(record_start),
    EXPORT(record_stop), EXPORT(record_status), EXPORT(read_log), EXPORT(consume_log),
    EXPORT(set_log_flags), EXPORT(reset), EXPORT(set_voltage), EXPORT(shutdown),
#undef EXPORT
};
static MP_DEFINE_CONST_DICT(globals, globals_table);
const mp_obj_module_t flight_rt_module = {.base = {&mp_type_module}, .globals = (mp_obj_dict_t *)&globals};
MP_REGISTER_MODULE(MP_QSTR_flight_rt, flight_rt_module);
