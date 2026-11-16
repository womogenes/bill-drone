// Host test of the control task: flight_rt.c and the modules it flies (flight_math, hover, ekf, guard, flip, recover, settings) on stubbed ESP-IDF (test/host), its IMU fed from a crude closed-loop plant, with every runtime request the radio makes fuzzed between ticks: arming, targets in every hover mode, link dropouts, stops, flips (flip_refusal forced open at random, so flips start from any state), level-and-hold, tumbles, IMU faults, test patterns, calibration, ESC calibration, maintenance, settings changes. Each tick's outputs (the four PWM duties, the status message, the log state and every drained log record) and every request's answer go into a per-tick digest, so two builds of the firmware can be compared tick for tick: make flight-test BASE=<git ref>. Usage: flight_rt_test SEED TICKS [TRACE]; prints the final digest and coverage counts.
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "soc/ledc_struct.h"

#include "baro.h"
#include "camera.h"
#include "gps.h"
#include "hover.h"
#include "runtime.h"
#include "settings.h"

// The firmware's flip_refusal under another name, so the test's own (below) can let a flip start from anywhere.
#define flip_refusal flip_refusal_firmware
#include "flip.c"
#undef flip_refusal

int host_lock_depth;
ledc_dev_t LEDC;
static int64_t clock_us = 1000000;
static void (*task)(void *);
static jmp_buf finished;
static long tick, ticks;
static FILE *trace;

// xorshift64*: the test's own random stream, identical for any two builds as long as their outputs are.
static uint64_t seed_state;
static uint64_t next(void) {
    seed_state ^= seed_state >> 12;
    seed_state ^= seed_state << 25;
    seed_state ^= seed_state >> 27;
    return seed_state * 2685821657736338717ULL;
}
static double uniform(double low, double high) { return low + (high - low) * (next() >> 11) * (1.0 / 9007199254740992.0); }
static bool chance(double p) { return uniform(0, 1) < p; }
// A hash of a few integers, for stubs whose answers must not depend on how often they are called.
static uint64_t mix(uint64_t a, uint64_t b) {
    uint64_t x = a * 0x9E3779B97F4A7C15ULL ^ (b + 0x632BE59BD9B4E019ULL);
    x ^= x >> 31;
    x *= 0xBF58476D1CE4E5B9ULL;
    return x ^ x >> 29;
}
static double hashed(uint64_t a, uint64_t b) { return (mix(a, b) >> 11) * (1.0 / 9007199254740992.0); }

// FNV-1a over everything the firmware puts out, per tick and overall.
static uint64_t digest = 14695981039346656037ULL, tick_digest;
static void feed(const void *data, size_t size) {
    const uint8_t *p = data;
    for (size_t i = 0; i < size; i++) {
        digest = (digest ^ p[i]) * 1099511628211ULL;
        tick_digest = (tick_digest ^ p[i]) * 1099511628211ULL;
    }
}
static void feed_text(const char *label, const char *text) {
    feed(label, strlen(label));
    if (text) feed(text, strlen(text));
    else feed("(null)", 6);
    if (trace && text) fprintf(trace, "  %s: %s\n", label, text);
}
static void feed_int(const char *label, long value) {
    feed(label, strlen(label));
    feed(&value, sizeof(value));
}

// ESP-IDF stand-ins.
int64_t esp_timer_get_time(void) { return clock_us; }
static uint32_t random_calls;
uint32_t esp_random(void) { return (uint32_t)mix(0xE5, random_calls++); }
TickType_t xTaskGetTickCount(void) { return (TickType_t)(clock_us / 10000); }
void vTaskDelay(TickType_t delay) { clock_us += delay * 10000LL; }
BaseType_t xTaskCreatePinnedToCore(void (*function)(void *), const char *name, uint32_t stack, void *arg, int priority, TaskHandle_t *handle, int core) {
    (void)name, (void)stack, (void)arg, (void)priority, (void)handle, (void)core;
    task = function;
    return pdPASS;
}
esp_err_t ledc_timer_config(const ledc_timer_config_t *config) { (void)config; return ESP_OK; }
esp_err_t ledc_channel_config(const ledc_channel_config_t *config) {
    LEDC.channel_group[0].channel[config->channel].duty.duty = config->duty << 4;
    return ESP_OK;
}
esp_err_t i2c_param_config(int port, const i2c_config_t *config) { (void)port, (void)config; return ESP_OK; }
esp_err_t i2c_driver_install(int port, int mode, size_t rx, size_t tx, int flags) { (void)port, (void)mode, (void)rx, (void)tx, (void)flags; return ESP_OK; }
esp_err_t i2c_driver_delete(int port) { (void)port; return ESP_OK; }
static int fifo_resets;
esp_err_t i2c_master_write_to_device(int port, uint8_t address, const uint8_t *data, size_t length, TickType_t wait) {
    (void)port, (void)address, (void)length, (void)wait;
    if (data[0] == 0x6a && data[1] == 0x44) fifo_resets++;
    return ESP_OK;
}

// The IMU's next answers, prepared by the plant before each tick: the data registers (accelerometer, temperature, gyro, in sensor axes and counts), INT_STATUS's ready bit, and the FIFO's samples; faults set by the fuzz.
static struct {
    esp_err_t registers, fifo_count, fifo_data;
    bool ready;
    int16_t newest[7];
    int samples;
    int16_t fifo[40][6];
    int bad_count;
} imu;

static void put16(uint8_t *at, int16_t value) {
    at[0] = (uint16_t)value >> 8;
    at[1] = value & 0xff;
}

esp_err_t i2c_master_write_read_device(int port, uint8_t address, const uint8_t *write, size_t write_length, uint8_t *read, size_t read_length, TickType_t wait) {
    (void)port, (void)address, (void)write_length, (void)wait;
    memset(read, 0, read_length);
    switch (write[0]) {
    case 0x3a:
        if (imu.registers) return imu.registers;
        read[0] = imu.ready;
        for (size_t i = 0; i < 7 && 1 + 2 * i + 1 < read_length; i++) put16(read + 1 + 2 * i, imu.newest[i]);
        return ESP_OK;
    case 0x72: {
        if (imu.fifo_count) return imu.fifo_count;
        int bytes = imu.bad_count ? imu.bad_count : imu.samples * 12;
        read[0] = bytes >> 8;
        read[1] = bytes & 0xff;
        return ESP_OK;
    }
    case 0x74:
        if (imu.fifo_data) return imu.fifo_data;
        for (size_t k = 0; k < read_length / 12; k++)
            for (int i = 0; i < 6; i++) put16(read + k * 12 + i * 2, imu.fifo[k][i]);
        return ESP_OK;
    case 0x75:
        read[0] = 0x68;
        return ESP_OK;
    case 0x6b:
        read[0] = 1;
        return ESP_OK;
    }
    return ESP_OK;
}

// The plant: a crude rigid body on the firmware's thrust table, enough to keep the controllers in their usual states (on the ground, lifting off, hovering, tilted, tumbling).
static struct {
    double roll, pitch, yaw, rate[3], x, y, z, velocity[3];
    double thrust[2], tilt[2];
    double rest_pitch, gust[3];
    bool grounded;
} plant = {.grounded = true};
#define MASS 1.05
#define G 9.81

static double table_thrust(double throttle) {
    if (throttle <= 0) return 0;
    for (size_t i = 1; i < flight_thrust_count; i++)
        if (throttle <= flight_thrust[i][0]) return flight_thrust[i - 1][1] + (throttle - flight_thrust[i - 1][0]) / (flight_thrust[i][0] - flight_thrust[i - 1][0]) * (flight_thrust[i][1] - flight_thrust[i - 1][1]);
    return flight_thrust[flight_thrust_count - 1][1];
}

// What the four PWM channels say: each motor's throttle and each servo's angle (degrees, as pulsed, offsets included).
static void outputs(double *throttle, double *servo) {
    for (int side = 0; side < 2; side++) {
        double us = (LEDC.channel_group[0].channel[side * 2 + 1].duty.duty >> 4) / ((1 << 16) * ESC_HZ / 1e6);
        throttle[side] = us > 1000.5 ? (us - 1000) * 1200 / 1000 : 0;
        double servo_us = (LEDC.channel_group[0].channel[side * 2].duty.duty >> 4) / ((1 << 16) * SERVO_HZ / 1e6);
        servo[side] = (servo_us - 1500) * 90 / 1000;
    }
}

static int16_t counts(double value, double scale) {
    double x = round(value * scale);
    return x > 32767 ? 32767 : x < -32768 ? -32768 : (int16_t)x;
}

static void step_plant(double dt) {
    double throttle[2], servo[2];
    outputs(throttle, servo);
    for (int side = 0; side < 2; side++) {
        plant.thrust[side] += (1.3 * table_thrust(throttle[side]) - plant.thrust[side]) * (1 - exp(-dt / 0.06));
        double tilt = (side ? -servo[side] : servo[side]) * M_PI / 180;
        plant.tilt[side] += (tilt - plant.tilt[side]) * (1 - exp(-dt / 0.08));
    }
    for (int i = 0; i < 3; i++) plant.gust[i] += (uniform(-1, 1) * 3 - plant.gust[i]) * 0.05;
    double left = plant.thrust[0], right = plant.thrust[1];
    double forward = left * sin(plant.tilt[0]) + right * sin(plant.tilt[1]);
    double up = left * cos(plant.tilt[0]) + right * cos(plant.tilt[1]);
    double accel[3] = {
        (left - right) * 0.172 / 0.012 - 0.4 * plant.rate[0] + plant.gust[0],
        forward * 0.025 / 0.0046 - 0.6 * plant.rate[1] + plant.gust[1] + 0.3,
        (right * sin(plant.tilt[1]) - left * sin(plant.tilt[0])) * 0.172 / 0.0107 * 0.5 - 0.5 * plant.rate[2] + plant.gust[2],
    };
    // A tumble now and then: the gyro's full scale reached, a tip past any limit.
    if (!plant.grounded && chance(0.0003))
        for (int i = 0; i < 3; i++) plant.rate[i] += uniform(-14, 14);
    for (int i = 0; i < 3; i++) plant.rate[i] += accel[i] * dt;
    plant.roll += plant.rate[0] * dt;
    plant.pitch += plant.rate[1] * dt;
    plant.yaw += plant.rate[2] * dt;
    plant.roll = atan2(sin(plant.roll), cos(plant.roll));
    plant.pitch = atan2(sin(plant.pitch), cos(plant.pitch));
    double cr = cos(plant.roll), cp = cos(plant.pitch);
    double vertical = up * cr * cp / MASS - G;
    double horizontal[2] = {(up * sin(plant.pitch) + forward) / MASS, up * sin(plant.roll) / MASS};
    if (plant.grounded) {
        if (vertical > 0.2) {
            plant.grounded = false;
        } else {
            plant.roll += (0 - plant.roll) * 0.2;
            plant.pitch += (plant.rest_pitch - plant.pitch) * 0.2;
            for (int i = 0; i < 3; i++) plant.rate[i] = 0;
            plant.velocity[0] = plant.velocity[1] = plant.velocity[2] = 0;
            plant.z = 0;
            vertical = 0;
            horizontal[0] = horizontal[1] = 0;
        }
    }
    if (!plant.grounded) {
        plant.velocity[2] += vertical * dt;
        plant.velocity[0] += (horizontal[0] - 0.3 * plant.velocity[0]) * dt;
        plant.velocity[1] += (horizontal[1] - 0.3 * plant.velocity[1]) * dt;
        plant.x += plant.velocity[0] * dt;
        plant.y += plant.velocity[1] * dt;
        plant.z += plant.velocity[2] * dt;
        if (plant.z <= 0) {
            plant.z = 0;
            plant.grounded = true;
            plant.rest_pitch = uniform(-0.2, 0.2);
        }
    }
    // The IMU: specific force (g) and body rates, in body axes, then sensor axes and counts.
    double specific[3];
    if (plant.grounded) {
        specific[0] = -sin(plant.pitch);
        specific[1] = sin(plant.roll) * cp;
        specific[2] = cr * cp;
    } else {
        specific[0] = -0.3 * plant.velocity[0] / G;
        specific[1] = -0.3 * plant.velocity[1] / G;
        specific[2] = up / MASS / G;
    }
    double vibration = (throttle[0] + throttle[1]) > 0 ? 0.08 : 0.002;
    imu.samples = 9 + (int)(next() % 3);
    for (int k = 0; k < imu.samples; k++) {
        double body[6];
        for (int i = 0; i < 3; i++) {
            body[i] = specific[i] + uniform(-vibration, vibration);
            body[3 + i] = plant.rate[i] + uniform(-0.01, 0.01) + (i == 1 ? 0.02 : 0);
        }
        for (int i = 0; i < 3; i++) {
            imu.fifo[k][sensor_axis[i]] = counts(sensor_to_body[i] * body[i], ACCEL_COUNTS_PER_G);
            imu.fifo[k][3 + sensor_axis[i]] = counts(sensor_to_body[i] * body[3 + i], 180 / M_PI * GYRO_COUNTS_PER_DEG_S);
        }
    }
    for (int i = 0; i < 3; i++) {
        imu.newest[i] = imu.fifo[imu.samples - 1][i];
        imu.newest[4 + i] = imu.fifo[imu.samples - 1][3 + i];
    }
    imu.newest[3] = (int16_t)((30 - 36.53) * 340);
}

// The sensors on core 0, as functions of the plant's state when each was last sampled, so their answers do not depend on when or how often the control task asks.
static struct { double x, y, z, velocity[3]; int64_t at; } gps_seen, baro_seen;
bool gps_fix(GpsFix *fix, uint32_t max_age_us) {
    if (!gps_seen.at || clock_us - gps_seen.at > (int64_t)max_age_us) return false;
    memset(fix, 0, sizeof(*fix));
    fix->received_us = gps_seen.at;
    fix->pvt.fix_type = hashed(gps_seen.at, 1) < 0.95 ? 3 : 2;
    fix->pvt.flags = 1;
    fix->pvt.speed_accuracy_mm_s = hashed(gps_seen.at, 2) < 0.95 ? 200 : 900;
    fix->pvt.horizontal_accuracy_mm = 1500;
    fix->pvt.latitude = 374000000 + (int32_t)(gps_seen.x / 0.0111);
    fix->pvt.longitude = -1220000000 + (int32_t)(gps_seen.y / 0.0088);
    fix->pvt.height_mm = 10000 + (int32_t)(gps_seen.z * 1000);
    for (int i = 0; i < 3; i++) fix->pvt.velocity_mm_s[i] = (int32_t)((i == 2 ? -1 : 1) * gps_seen.velocity[i] * 1000 + (hashed(gps_seen.at, 3 + i) - 0.5) * 100);
    return true;
}
bool baro_sample(BaroSample *sample, uint32_t max_age_us) {
    if (!baro_seen.at || clock_us - baro_seen.at > (int64_t)max_age_us) return false;
    *sample = (BaroSample){.read_us = baro_seen.at, .pressure = 101000, .temperature = 25, .altitude = 30 + baro_seen.z + (hashed(baro_seen.at, 7) - 0.5) * 0.4};
    return true;
}
static int64_t frame_us(void) { return clock_us / 33333 * 33333; }
bool camera_candidates(CameraPose *pose, uint32_t max_age_us) {
    int64_t at = frame_us();
    if (hashed(at, 11) < 0.3 || clock_us - at > (int64_t)max_age_us) return false;
    memset(pose, 0, sizeof(*pose));
    pose->sequence = (uint32_t)(at / 33333);
    pose->capture_us = at - 20000;
    pose->valid = hashed(at, 12) < 0.5 ? 1 : 2;
    pose->rotation[0] = pose->rotation[4] = pose->rotation[8] = 1;
    pose->tag_position_m[0] = (float)(hashed(at, 13) - 0.5);
    pose->tag_position_m[1] = (float)(0.3 - plant.z * 0.1);
    pose->tag_position_m[2] = (float)(1.8 + hashed(at, 14) * 0.2);
    return true;
}
bool camera_newest(uint32_t *sequence, int64_t *centre_us) {
    int64_t at = frame_us();
    *sequence = (uint32_t)(at / 33333);
    *centre_us = at - 20000;
    return at > 2000000;
}

// Flips start where the firmware's checks would refuse them (FORCED_FLIP of the requests while armed), so the flip, its recovery and every owner hand-over meet states a real flight would not reach.
static bool force_flip;
const char *flip_refusal(const FlipCheck *c) {
    if (force_flip && c->armed) return NULL;
    return flip_refusal_firmware(c);
}

// Coverage: ticks armed, in each flip phase and recovery stage, and stop reasons seen.
static long armed_ticks, phase_ticks[4], stage_ticks[4], reasons[16], modes[4], flips_started, levels_taken, arms, link_gaps, sensor_faults, patterns_played;

static void check_unlocked(const char *where) {
    if (host_lock_depth) {
        fprintf(stderr, "lock held after %s (depth %d) at tick %ld\n", where, host_lock_depth, tick);
        exit(2);
    }
}

// The radio's side: what the laptop streams and asks, as a session state machine.
static struct {
    int phase;
    double phase_until, base, roll, pitch, yaw, servo[2], throttle[2], nudge[3];
    int mode;
    bool streaming, recorder;
    double gap_until;
    uint32_t sequence;
} laptop = {.recorder = true};

static void set_setting(int index, float value) {
    settings_change(&index, &value, 1, clock_us);
}

static void randomize_settings(void) {
    static const struct { int index; float values[5]; int count; } choices[] = {
        {SETTING_hover_camera, {0, 1}, 2}, {SETTING_hover_gps_tau, {0, 0.5f}, 2}, {SETTING_hover_baro_tau, {0, 1}, 2}, {SETTING_flip_angle, {0, 30, 60, 90, 45}, 5},
        {SETTING_recover_enable, {0, 1}, 2}, {SETTING_auto_trim, {0, 0.5f}, 2}, {SETTING_pitch_damping_tau, {0, 0.03f}, 2}, {SETTING_tip_limit, {45, 80}, 2},
        {SETTING_thrust_voltage, {0, 16}, 2}, {SETTING_hover_drag, {0, 1}, 2}, {SETTING_integrate_throttle, {580, 350}, 2}, {SETTING_roll_target_rate, {0, 360}, 2},
        {SETTING_hover_climb, {0.3f, 3}, 2}, {SETTING_yaw_i, {0, 2}, 2}, {SETTING_pitch_speed, {0, 2}, 2}, {SETTING_failsafe_speed, {0, 2}, 2},
        {SETTING_hover_gps_calibrate, {0, 5}, 2}, {SETTING_roll_bias, {0, 0.5f, -0.5f}, 3}, {SETTING_hover_mass, {0.9f, 1.05f}, 2}, {SETTING_left_thrust_scale, {1, 1.3f}, 2},
        {SETTING_right_thrust_scale, {1, 1.3f}, 2}, {SETTING_indi, {0, 0.5f, 1}, 3}, {SETTING_indi_pitch, {0, 0.25f, 1}, 3}, {SETTING_pitch_predict, {0, 0.5f, 1}, 3}, {SETTING_roll_authority, {8, 14.5f}, 2}, {SETTING_yaw_authority, {11.35f, 16}, 2},
    };
    for (size_t i = 0; i < sizeof(choices) / sizeof(choices[0]); i++)
        if (chance(0.5)) set_setting(choices[i].index, choices[i].values[next() % choices[i].count]);
    flight_runtime_apply_settings();
    check_unlocked("apply_settings");
}

static void load_pattern(void) {
    uint8_t body[9 + 6 * (2 + 2 * 10)];
    float after = (float)uniform(0, 3);
    uint16_t mask = (uint16_t)(next() & 0x3ff), count = (uint16_t)(1 + next() % 6);
    int channels = __builtin_popcount(mask);
    static const float limit[10] = {20, 20, 30, 100, 5, 5, 100, 100, 0.5f, 0.5f};
    memcpy(body, &after, 4);
    body[4] = (uint8_t)(next() % 3);
    memcpy(body + 5, &mask, 2);
    memcpy(body + 7, &count, 2);
    uint8_t *p = body + 9;
    for (int s = 0; s < count; s++) {
        uint16_t length = (uint16_t)(5 + next() % 60);
        memcpy(p, &length, 2);
        p += 2;
        for (int i = 0; i < 10; i++) {
            if (!(mask >> i & 1)) continue;
            int16_t value = (int16_t)(uniform(-1, 1) * limit[i] * 100);
            memcpy(p, &value, 2);
            p += 2;
        }
    }
    char text[200];
    feed_text("pattern", flight_runtime_pattern(body, 9 + count * (2 + 2 * channels), text, sizeof(text)));
}

// Between ticks: what the radio and the laptop do, in the order the fuzz draws.
static void requests(void) {
    double t = clock_us / 1e6;
    if (t >= laptop.phase_until) {
        laptop.phase = laptop.phase == 1 ? 0 : 1;
        laptop.phase_until = t + (laptop.phase ? uniform(4, 40) : uniform(0.5, 4));
        if (laptop.phase == 1) {
            laptop.base = 0;
            laptop.mode = 0;
            laptop.roll = laptop.pitch = laptop.yaw = 0;
            laptop.streaming = true;
            feed_int("idle", flight_runtime_idle());
            arms++;
            uint32_t generation = flight_runtime_stop_generation();
            const char *answer = flight_runtime_arm(laptop.recorder || chance(0.9), chance(0.05), chance(0.97) ? generation : generation - 1, ++laptop.sequence);
            feed_text("arm", answer);
            // The pilot fixes what the refusal says and tries again next session.
            if (answer && strstr(answer, "Calibrate")) feed_text("calibrate", flight_runtime_calibrate());
            if (answer && strstr(answer, "ESC")) feed_text("esc calibration", flight_runtime_esc_calibration(0));
        } else {
            if (chance(0.7)) {
                flight_runtime_command_stop(++laptop.sequence);
                feed_text("stop", "");
            } else {
                laptop.mode = chance(0.5) ? HOVER_LAND : HOVER_LAND_FAST;
            }
        }
    }
    LogState log;
    flight_runtime_log(&log);
    bool armed = false;
    Status status;
    memset(&status, 0, sizeof(status));
    flight_runtime_status(&status);
    armed = status.flags >> 3 & 1;
    if (laptop.phase == 1 && armed) {
        // Flying: the collective, the tilts and the hover mode wander; flips, level-and-hold and link gaps now and then.
        if (chance(0.02)) laptop.mode = chance(0.6) ? HOVER_HOLD : chance(0.5) ? HOVER_OFF : chance(0.5) ? HOVER_LAND : HOVER_LAND_FAST;
        if (laptop.mode == HOVER_OFF) laptop.base += (uniform(560, 760) - laptop.base) * 0.05;
        else laptop.base = chance(0.01) ? 0 : laptop.base;
        if (chance(0.05)) {
            laptop.roll = chance(0.5) ? 0 : uniform(-15, 15);
            laptop.pitch = chance(0.5) ? 0 : uniform(-15, 15);
            laptop.yaw = chance(0.7) ? 0 : uniform(-45, 45);
            for (int i = 0; i < 3; i++) laptop.nudge[i] = chance(0.7) ? 0 : uniform(-1.5, 1.5);
        }
        if (chance(0.004)) {
            force_flip = chance(0.7);
            if (chance(0.8)) laptop.mode = HOVER_HOLD;
            const char *answer = flight_runtime_flip();
            flips_started += !answer;
            feed_text("flip", answer);
            force_flip = false;
        }
        if (chance(0.002)) {
            const char *answer = flight_runtime_level();
            levels_taken += !answer;
            feed_text("level", answer);
        }
        if (chance(0.002)) {
            laptop.gap_until = t + uniform(0.2, 3.5);
            link_gaps++;
        }
        if (chance(0.0002)) {
            flight_runtime_command_stop(++laptop.sequence);
            feed_text("stop", "");
        }
    } else if (!armed) {
        laptop.base = laptop.roll = laptop.pitch = laptop.yaw = 0;
        laptop.mode = HOVER_OFF;
        if (chance(0.01)) laptop.servo[next() % 2] = chance(0.5) ? 0 : uniform(-20, 20);
        laptop.throttle[0] = laptop.throttle[1] = chance(0.003) ? uniform(0, 300) : 0;
        if (chance(0.002)) randomize_settings();
        if (chance(0.002)) load_pattern();
        if (chance(0.0005)) {
            float offset;
            char error[100];
            feed_text("zero servo", flight_runtime_zero_servo((int)(next() % 2), &offset, error, sizeof(error)));
            feed(&offset, sizeof(offset));
        }
        if (chance(0.0005)) feed_text("esc calibration", flight_runtime_esc_calibration((int)(next() % 3)));
        if (chance(0.002)) feed_text("esc calibration", flight_runtime_esc_calibration(0));
        // The laptop calibrates an uncalibrated IMU (one a sensor fault interrupted, too) before it tries to arm again.
        if (chance(0.0005) || (!(status.flags >> 2 & 1) && !(status.flags >> 1 & 1) && chance(0.01))) feed_text("calibrate", flight_runtime_calibrate());
        if (chance(0.001)) feed_text("record", flight_runtime_record(chance(0.5), true));
        if (chance(0.001) && flight_runtime_begin_maintenance()) {
            feed_text("maintenance", "");
            flight_runtime_end_maintenance();
        }
    }
    if (chance(0.00003)) feed_text("calibrate", flight_runtime_calibrate());
    if (chance(0.001)) {
        feed_text("apply", "");
        flight_runtime_apply_settings();
    }
    if (chance(0.002)) flight_runtime_set_voltage(chance(0.2) ? 0 : (uint16_t)uniform(chance(0.1) ? 11000 : 14800, 16800));
    if (chance(0.0005)) feed_text("level", flight_runtime_level());
    if (chance(0.0005)) {
        force_flip = false;
        feed_text("flip", flight_runtime_flip());
    }
    feed_int("camera hold", flight_runtime_camera_hold());
    // The laptop's targets, unless the link is down: a target packet is the heartbeat.
    if (t >= laptop.gap_until && laptop.streaming) {
        float targets[12] = {(float)laptop.base, (float)laptop.roll, (float)laptop.pitch, (float)laptop.yaw, (float)laptop.servo[0], (float)laptop.servo[1], (float)laptop.throttle[0], (float)laptop.throttle[1], (float)laptop.mode, (float)laptop.nudge[0], (float)laptop.nudge[1], (float)laptop.nudge[2]};
        if (chance(0.001)) targets[1] = 99;
        feed_int("targets", flight_runtime_targets(targets));
    }
    // The radio's update loop: a finished calibration or a self-trim is taken once disarmed.
    float bias[3], pose_zero[2], frame, trimmed[2];
    if (flight_runtime_take_calibration(bias, pose_zero, &frame)) {
        feed("calibration", 11);
        feed(bias, sizeof(bias));
        feed(pose_zero, sizeof(pose_zero));
        feed(&frame, sizeof(frame));
    }
    if (flight_runtime_take_trim(&trimmed[0], &trimmed[1])) {
        feed("trim", 4);
        feed(trimmed, sizeof(trimmed));
        int indices[2] = {SETTING_roll_bias, SETTING_pitch_bias};
        if (settings_change(indices, trimmed, 2, clock_us)) flight_runtime_apply_settings();
    }
    check_unlocked("requests");
}

// The end of a tick (the task's vTaskDelayUntil): digest what it put out, drain the log as the radio does, step the plant and the sensors, then the requests.
void vTaskDelayUntil(TickType_t *previous, TickType_t period) {
    (void)previous, (void)period;
    check_unlocked("tick");
    tick_digest = 14695981039346656037ULL;
    uint32_t duty[4];
    for (int i = 0; i < 4; i++) duty[i] = LEDC.channel_group[0].channel[i].duty.duty;
    feed(duty, sizeof(duty));
    Status status;
    memset(&status, 0, sizeof(status));
    flight_runtime_status(&status);
    feed(&status, sizeof(status));
    LogState log;
    memset(&log, 0, sizeof(log));
    flight_runtime_log(&log);
    feed_int("session", log.session);
    feed_int("samples", log.samples);
    feed_int("dropped", log.dropped);
    feed_int("elapsed", log.elapsed_us);
    feed_int("pending", log.pending);
    feed_int("active", log.active);
    feed_int("reason", log.reason);
    feed(log.bias, sizeof(log.bias));
    feed(log.pose_zero, sizeof(log.pose_zero));
    uint8_t records[16 * RECORD_SIZE];
    size_t drained = 0, n;
    while ((n = flight_runtime_read_log(records, 0, 16)) > 0) {
        feed(records, n * RECORD_SIZE);
        for (size_t k = 0; k < n; k++) {
            uint16_t flags, guard;
            memcpy(&flags, records + k * RECORD_SIZE + 64, 2);
            memcpy(&guard, records + k * RECORD_SIZE + 108 - 20, 2);
            modes[flags >> 1 & 3]++;
            patterns_played += flags >> 8 & 1;
        }
        flight_runtime_consume_log(n);
        drained += n;
    }
    bool armed = status.flags >> 3 & 1;
    armed_ticks += armed;
    phase_ticks[status.flags >> 14 & 3]++;
    stage_ticks[status.reason >> 6 & 3]++;
    reasons[status.reason & 15]++;
    if (trace)
        fprintf(trace, "%ld %016llx duty %u %u %u %u flags %04x reason %02x roll %.4f pitch %.4f thr %u %u servo %.3f %.3f drained %zu\n", tick, (unsigned long long)tick_digest, duty[0], duty[1], duty[2], duty[3], status.flags,
                status.reason, status.roll, status.pitch, status.throttle[0], status.throttle[1], status.servo[0], status.servo[1], drained);
    if (++tick >= ticks) longjmp(finished, 1);
    tick_digest = 14695981039346656037ULL;
    int64_t dt_us = chance(0.0003) ? 30000 : 9500 + (int64_t)(next() % 1000);
    clock_us += dt_us;
    step_plant(dt_us / 1e6);
    if (clock_us - gps_seen.at >= 100000 || !gps_seen.at) gps_seen = (typeof(gps_seen)){plant.x, plant.y, plant.z, {plant.velocity[0], plant.velocity[1], plant.velocity[2]}, clock_us};
    if (clock_us - baro_seen.at >= 20000 || !baro_seen.at) baro_seen = (typeof(baro_seen)){plant.x, plant.y, plant.z, {0, 0, 0}, clock_us};
    // IMU faults: a failed or stalled read now and then, sometimes long enough for the task to set the IMU up again; a FIFO out of step.
    static int fault_left, fault_kind;
    if (!fault_left && chance(0.0003)) {
        fault_left = chance(0.1) ? 120 : 1 + (int)(next() % 4);
        fault_kind = (int)(next() % 2);
        sensor_faults++;
    }
    imu.registers = fault_left && fault_kind == 0 ? ESP_FAIL : ESP_OK;
    imu.ready = !(fault_left && fault_kind == 1);
    if (fault_left) fault_left--;
    imu.fifo_count = chance(0.001) ? ESP_FAIL : ESP_OK;
    imu.fifo_data = chance(0.001) ? ESP_FAIL : ESP_OK;
    imu.bad_count = chance(0.002) ? (chance(0.5) ? 7 : 12 * 30) : 0;
    if (chance(0.001)) imu.fifo[imu.samples - 1][0] ^= 0x4000;
    requests();
    feed(&tick_digest, 0);
    if (trace && tick_digest != 14695981039346656037ULL) fprintf(trace, "%ld requests %016llx\n", tick, (unsigned long long)tick_digest);
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s SEED TICKS [TRACE]\n", argv[0]);
        return 1;
    }
    seed_state = mix(strtoull(argv[1], NULL, 10), 1) | 1;
    ticks = strtol(argv[2], NULL, 10);
    trace = argc > 3 ? fopen(argv[3], "w") : NULL;
    settings_load();
    flight_runtime_outputs();
    randomize_settings();
    flight_runtime_start();
    check_unlocked("start");
    flight_runtime_set_voltage(16000);
    step_plant(0.01);
    laptop.phase_until = 5;
    feed_text("calibrate", flight_runtime_calibrate());
    if (!setjmp(finished)) task(NULL);
    printf("digest %016llx after %ld ticks; armed %ld, flip phases %ld/%ld/%ld, recovery stages %ld/%ld, hover modes %ld/%ld/%ld/%ld, flips %ld, levels %ld, arms %ld, link gaps %ld, sensor faults %ld (fifo resets %d), pattern ticks %ld; stop reasons",
           (unsigned long long)digest, tick, armed_ticks, phase_ticks[1], phase_ticks[2], phase_ticks[3], stage_ticks[1], stage_ticks[2], modes[0], modes[1], modes[2], modes[3], flips_started, levels_taken, arms, link_gaps, sensor_faults, fifo_resets, patterns_played);
    for (int i = 0; i < 16; i++)
        if (reasons[i]) printf(" %d:%ld", i, reasons[i]);
    printf("\n");
    if (trace) fclose(trace);
    return 0;
}
