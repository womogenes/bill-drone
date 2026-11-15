// The compute and scheduling benchmark (test build only, BENCH; bench.h): included at the end of flight_rt.c, so it times the control task's own code and state.
// Timing: every metric is a histogram of CPU cycles (log-spaced, BIN_SUB bins an octave, so a percentile is within about 3%; the minimum and maximum are exact). The stages are the tick's work between MARKs under the lock; the rest is the task's scheduling: how late it wakes after the FreeRTOS tick that wakes it, the period between wake-ups, the IMU's I2C reads, its wait for the lock and the whole tick from wake-up to the lock's release. Every LOCK site records its wait and hold too.
// A run (flight_runtime_bench 's') flies the real tick armed, in a sandbox, for a number of real 10 ms ticks: the control task's state is saved, the settings the run needs are applied to the command alone (nothing is stored), and the sensors are a crude simulated plant closed around the shadowed outputs (as test/flight_rt_test.c's) or a recording's (bench_replay.h, tools/bench_replay.py). The PWM outputs never change while it runs: pulse() writes a shadow. At the end the saved state comes back and the outputs are written as the disarmed drone has them. Refused with a pack connected.
#include "esp_cpu.h"
#include "esp_freertos_hooks.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "mpc.h"
#if __has_include("bench_replay.h")
#include "bench_replay.h"
#define HAVE_REPLAY 1
#endif

enum { M_COMPUTE = STAGES, M_TICK, M_I2C, M_WAIT, M_LATE, M_PERIOD, M_MPC, M_HOLD_OTHER, M_WAIT_OTHER, METRICS };
static const char *const metric_names[METRICS] = {
    "imu", "attitude", "indi", "flip", "inputs", "ekf_queue", "ekf", "estimate", "recover", "hover", "setpoint", "control", "output", "rest", "log",
    "compute", "tick", "i2c", "lock_wait", "wake_late", "period", "mpc", "other_hold", "other_wait",
};

// Values in units of 16 cycles; below BIN_SUB units each its own bin, above that BIN_SUB bins an octave, up to 2^20 units (70 ms at 240 MHz). On the heap (bench_start): static RAM is full.
#define BIN_SUB 16
#define BINS 272
typedef struct {
    uint32_t n, min, max;
    uint64_t sum;
    uint16_t count[BINS];
} Histogram;
static Histogram *histograms;

static int bin_of(uint32_t cycles) {
    uint32_t v = cycles >> 4;
    if (v < BIN_SUB) return v;
    int e = 31 - __builtin_clz(v);
    int b = (e - 3) * BIN_SUB + ((v >> (e - 4)) & (BIN_SUB - 1));
    return b < BINS ? b : BINS - 1;
}

// The lowest value (cycles) a bin holds.
static uint32_t bin_low(int b) {
    if (b < BIN_SUB) return (uint32_t)b << 4;
    int e = b / BIN_SUB + 3;
    return ((uint32_t)(BIN_SUB + b % BIN_SUB) << (e - 4)) << 4;
}

// Nothing before bench_start: the radio takes the lock first.
static void record(int metric, uint32_t cycles) {
    if (!histograms) return;
    Histogram *h = &histograms[metric];
    if (!h->n || cycles < h->min) h->min = cycles;
    if (cycles > h->max) h->max = cycles;
    h->n++;
    h->sum += cycles;
    uint16_t *c = &h->count[bin_of(cycles)];
    if (*c < UINT16_MAX) (*c)++;
}

// The q-quantile (cycles): the middle of the bin it falls in (about 4% wide), within the exact extremes.
static uint32_t quantile(const Histogram *h, float q) {
    if (!h->n) return 0;
    uint32_t target = (uint32_t)ceilf(q * h->n), seen = 0;
    if (target < 1) target = 1;
    for (int b = 0; b < BINS; b++) {
        seen += h->count[b];
        if (seen >= target) {
            uint32_t value = b < BIN_SUB ? bin_low(b) + 8 : (bin_low(b) + bin_low(b + 1)) / 2;
            return value < h->min ? h->min : value > h->max ? h->max : value;
        }
    }
    return h->max;
}

static uint32_t cycles(void) { return esp_cpu_get_cycle_count(); }

// The tick's stages: cycles each got this tick and whether it ran.
static uint32_t stage_cycles[STAGES], stage_last, stage_hit, tick_start;

static void bench_mark(int stage) {
    uint32_t now = cycles();
    stage_cycles[stage] += now - stage_last;
    stage_hit |= 1u << stage;
    stage_last = now;
}

// The lock, per call site (the function's name): calls, the longest wait and hold, the summed holds (cycles, each on its caller's core).
#define SITES 40
static struct {
    const char *site;
    uint32_t count, wait_max, hold_max;
    uint64_t hold_sum;
} sites[SITES];
static const char *holder;
static uint32_t held_since, holder_wait;

static void bench_lock(const char *site) {
    uint32_t start = cycles();
    xSemaphoreTake(lock, portMAX_DELAY);
    held_since = cycles();
    holder = site;
    holder_wait = held_since - start;
}

// Recorded while still held, which is what guards the tables.
static void bench_unlock(void) {
    uint32_t hold = cycles() - held_since;
    bool control = xPortGetCoreID() == 1;
    record(control ? M_WAIT : M_WAIT_OTHER, holder_wait);
    if (!control) record(M_HOLD_OTHER, hold);
    for (int i = 0; i < SITES; i++) {
        if (sites[i].site && sites[i].site != holder) continue;
        sites[i].site = holder;
        sites[i].count++;
        if (holder_wait > sites[i].wait_max) sites[i].wait_max = holder_wait;
        if (hold > sites[i].hold_max) sites[i].hold_max = hold;
        sites[i].hold_sum += hold;
        break;
    }
    xSemaphoreGive(lock);
}

// Scheduling, measured outside the lock and recorded at the next tick's start: the FreeRTOS tick interrupt on core 1 (its cycle count), this wake-up and the last, and what the last tick took.
static volatile uint32_t tick_interrupt;
static uint32_t woke, previous_woke, pending[METRICS];
static uint32_t pending_mask;

static void IRAM_ATTR bench_tick_hook(void) { tick_interrupt = cycles(); }

static void later(int metric, uint32_t value) {
    pending[metric] = value;
    pending_mask |= 1u << (metric - STAGES);
}

// A run: its scenario and length, the tick it is at, whether it records a log, and what it saw (coverage).
enum { RUN_FLIGHT = 1, RUN_REPLAY, RUN_WORST };
static struct {
    bool requested, active, record, mpc, started_log, reinstall, warm;
    // The cache test (w): the Kalman filter's and the hover's tick run on copies four times in a row (us each).
    float warm_ekf[4], warm_hover[4];
    int scenario, ticks, tick, disarmed;
    char error[48];
    // Coverage: ticks armed, in the hold flying, in each flip phase, recovering, with a GPS fix, a barometer reading and both, with a saturated sensor; flips and kicks; the highest the hold got (m); the flight task's least free stack (bytes).
    int armed, holding, phase[4], recovering, gps, baro, both, saturated, flips, kicks;
    float top;
    uint32_t stack_free;
} run;

// The actuators' pulses while a run flies (us), which the plant reads.
static float shadow_us[4];

static bool bench_shadow(int index, float us) {
    if (!run.active) return false;
    shadow_us[index] = us;
    return true;
}

// The plant (test/flight_rt_test.c's, in float): a rigid body on the thrust table, its IMU (10 FIFO samples a tick), and when the GPS and barometer last sampled it.
static struct {
    float roll, pitch, yaw, rate[3], x, y, z, velocity[3], thrust[2], tilt[2], gust[3];
    bool grounded;
    int16_t newest[7];
    int32_t sum[6];
    bool saturated;
    struct { float x, y, z, velocity[3]; int64_t at; } gps, baro;
} plant;
static uint32_t random_state = 0x9e3779b9;

static float uniform(float low, float high) {
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return low + (high - low) * (random_state >> 8) * (1.0f / 16777216);
}

static int16_t counts(float value, float scale, bool *saturated) {
    float x = roundf(value * scale);
    if (x >= 32767 || x <= -32768) *saturated = true;
    return x > 32767 ? 32767 : x < -32768 ? -32768 : (int16_t)x;
}

#define PLANT_MASS 1.05f
#define PLANT_SAMPLES 10

static void step_plant(float dt) {
    float throttle[2], servo[2];
    for (int side = 0; side < 2; side++) {
        float us = shadow_us[side * 2 + 1];
        throttle[side] = us > 1000.5f ? (us - 1000) * THROTTLE_MAX / (ESC_TOP_US - 1000) : 0;
        float servo_us = shadow_us[side * 2];
        servo[side] = servo_us > 0 ? (servo_us - 1500) * 90 / 1000 - state.offsets[side] : 0;
        plant.thrust[side] += (1.3f * (throttle[side] > 0 ? thrust(throttle[side]) : 0) - plant.thrust[side]) * (1 - expf(-dt / 0.06f));
        float tilt = (side ? -servo[side] : servo[side]) / DEG;
        plant.tilt[side] += (tilt - plant.tilt[side]) * (1 - expf(-dt / 0.08f));
    }
    for (int i = 0; i < 3; i++) plant.gust[i] += (uniform(-1, 1) * 3 - plant.gust[i]) * 0.05f;
    float left = plant.thrust[0], right = plant.thrust[1];
    float forward = left * sinf(plant.tilt[0]) + right * sinf(plant.tilt[1]);
    float up = left * cosf(plant.tilt[0]) + right * cosf(plant.tilt[1]);
    float accel[3] = {
        (left - right) * 0.172f / 0.012f - 0.4f * plant.rate[0] + plant.gust[0],
        forward * 0.025f / 0.0046f - 0.6f * plant.rate[1] + plant.gust[1] + 0.3f,
        (right * sinf(plant.tilt[1]) - left * sinf(plant.tilt[0])) * 0.172f / 0.0107f * 0.5f - 0.5f * plant.rate[2] + plant.gust[2],
    };
    for (int i = 0; i < 3; i++) plant.rate[i] += accel[i] * dt;
    plant.roll = atan2f(sinf(plant.roll + plant.rate[0] * dt), cosf(plant.roll + plant.rate[0] * dt));
    plant.pitch = atan2f(sinf(plant.pitch + plant.rate[1] * dt), cosf(plant.pitch + plant.rate[1] * dt));
    plant.yaw += plant.rate[2] * dt;
    float cr = cosf(plant.roll), cp = cosf(plant.pitch);
    float vertical = up * cr * cp / PLANT_MASS - GRAVITY;
    float horizontal[2] = {(up * sinf(plant.pitch) + forward) / PLANT_MASS, up * sinf(plant.roll) / PLANT_MASS};
    if (plant.grounded) {
        if (vertical > 0.2f) {
            plant.grounded = false;
        } else {
            plant.roll *= 0.8f;
            plant.pitch *= 0.8f;
            for (int i = 0; i < 3; i++) plant.rate[i] = plant.velocity[i] = 0;
            plant.z = 0;
        }
    }
    if (!plant.grounded) {
        plant.velocity[2] += vertical * dt;
        plant.velocity[0] += (horizontal[0] - 0.3f * plant.velocity[0]) * dt;
        plant.velocity[1] += (horizontal[1] - 0.3f * plant.velocity[1]) * dt;
        plant.x += plant.velocity[0] * dt;
        plant.y += plant.velocity[1] * dt;
        plant.z += plant.velocity[2] * dt;
        if (plant.z <= 0) {
            plant.z = 0;
            plant.grounded = true;
        }
    }
    float specific[3];
    if (plant.grounded) {
        specific[0] = -sinf(plant.pitch);
        specific[1] = sinf(plant.roll) * cp;
        specific[2] = cr * cp;
    } else {
        specific[0] = -0.3f * plant.velocity[0] / GRAVITY;
        specific[1] = -0.3f * plant.velocity[1] / GRAVITY;
        specific[2] = up / PLANT_MASS / GRAVITY;
    }
    float vibration = throttle[0] + throttle[1] > 0 ? 0.08f : 0.002f;
    memset(plant.sum, 0, sizeof(plant.sum));
    plant.saturated = false;
    int16_t sample[6];
    for (int k = 0; k < PLANT_SAMPLES; k++) {
        for (int i = 0; i < 3; i++) {
            sample[sensor_axis[i]] = counts(sensor_to_body[i] * (specific[i] + uniform(-vibration, vibration)), ACCEL_COUNTS_PER_G, &plant.saturated);
            // The calibration's bias added back, so the gyro the tick takes it off from reads the plant's rate.
            sample[3 + sensor_axis[i]] = counts(sensor_to_body[i] * (plant.rate[i] + state.bias[i] + uniform(-0.01f, 0.01f)), 180 * GYRO_COUNTS_PER_DEG_S / 3.14159265f, &plant.saturated);
        }
        for (int i = 0; i < 6; i++) plant.sum[i] += sample[i];
    }
    for (int i = 0; i < 3; i++) {
        plant.newest[i] = sample[i];
        plant.newest[4 + i] = sample[3 + i];
    }
    plant.newest[3] = (int16_t)((30 - 36.53f) * 340);
    int64_t now = esp_timer_get_time();
    bool every = run.scenario == RUN_WORST;
    if (every || now - plant.gps.at >= 100000) plant.gps = (typeof(plant.gps)){plant.x, plant.y, plant.z, {plant.velocity[0], plant.velocity[1], plant.velocity[2]}, now};
    if (every || now - plant.baro.at >= 20000) plant.baro = (typeof(plant.baro)){plant.x, plant.y, plant.z, {0, 0, 0}, now};
}

static void reset_plant(void) {
    memset(&plant, 0, sizeof(plant));
    plant.grounded = true;
    for (int i = 0; i < 4; i++) shadow_us[i] = i % 2 ? ESC_STOP_US : 1500 + state.offsets[i / 2] * 1000 / 90;
    step_plant(0.01f);
}

// The replay's sensors at a tick: the record's IMU (in body order, put back in the sensor's), its barometer reading every other tick and a GPS solution every tenth from its Kalman filter's velocity and position.
static int replay_index(void) {
#ifdef HAVE_REPLAY
    return run.tick % REPLAY_TICKS;
#else
    return 0;
#endif
}

static void replay_sensors(void) {
#ifdef HAVE_REPLAY
    const ReplayTick *r = &replay[replay_index()];
    plant.saturated = false;
    // The recording's accelerometer read 16384 counts a g (±2 g); the firmware's reads ACCEL_COUNTS_PER_G.
    for (int i = 0; i < 3; i++) {
        plant.newest[sensor_axis[i]] = (int16_t)lroundf(r->raw[i] * ACCEL_COUNTS_PER_G / 16384);
        plant.newest[4 + sensor_axis[i]] = r->raw[4 + i];
    }
    plant.newest[3] = r->raw[3];
    for (int i = 0; i < 7; i++) plant.saturated |= r->raw[i] >= 32767 || r->raw[i] <= -32768;
    for (int i = 0; i < 3; i++) {
        plant.sum[i] = plant.newest[i] * PLANT_SAMPLES;
        plant.sum[3 + i] = plant.newest[4 + i] * PLANT_SAMPLES;
    }
    int64_t now = esp_timer_get_time();
    if (run.tick % 10 == 0)
        plant.gps = (typeof(plant.gps)){r->position[0] / 100.0f, -r->position[1] / 100.0f, r->height / 1000.0f, {r->velocity[0] / 1000.0f, -r->velocity[1] / 1000.0f, r->velocity[2] / 1000.0f}, now};
    if (run.tick % 2 == 0 && r->baro_mm != INT32_MIN) plant.baro = (typeof(plant.baro)){0, 0, r->baro_mm / 1000.0f - 30, {0, 0, 0}, now};
#endif
}

#undef gps_fix
#undef baro_sample
static bool bench_gps_fix(GpsFix *fix, uint32_t max_age_us) {
    if (!run.active) return gps_fix(fix, max_age_us);
    int64_t now = esp_timer_get_time();
    if (!plant.gps.at || now - plant.gps.at > (int64_t)max_age_us) return false;
    memset(fix, 0, sizeof(*fix));
    fix->received_us = plant.gps.at;
    fix->pvt.fix_type = 3;
    fix->pvt.flags = 1;
    fix->pvt.satellites = 12;
    fix->pvt.speed_accuracy_mm_s = 200;
    fix->pvt.horizontal_accuracy_mm = 1500;
    fix->pvt.vertical_accuracy_mm = 2500;
    fix->pvt.latitude = 377000000 + (int32_t)(plant.gps.x / 0.01112f);
    fix->pvt.longitude = -1223900000 + (int32_t)(plant.gps.y / 0.0088f);
    fix->pvt.height_mm = 10000 + (int32_t)(plant.gps.z * 1000);
    for (int i = 0; i < 3; i++) fix->pvt.velocity_mm_s[i] = (int32_t)((i == 2 ? -1 : 1) * plant.gps.velocity[i] * 1000 + uniform(-50, 50));
    return true;
}

static bool bench_baro_sample(BaroSample *sample, uint32_t max_age_us) {
    if (!run.active) return baro_sample(sample, max_age_us);
    int64_t now = esp_timer_get_time();
    if (!plant.baro.at || now - plant.baro.at > (int64_t)max_age_us) return false;
    *sample = (BaroSample){.read_us = plant.baro.at, .pressure = 101000, .temperature = 25, .altitude = 30 + plant.baro.z + uniform(-0.2f, 0.2f)};
    return true;
}

// The control task's state a run changes, saved before and put back after.
#define SNAPSHOT(X) X(state) X(command) X(flown) X(hover) X(ekf) X(filtered) X(guard) X(flip) X(maneuver) X(recovery) X(indi) X(indi_fade) X(indi_share) \
    X(nudge) X(battery_v) X(baro_altitude) X(trim) X(held) X(link_landing) X(servo_pack) X(held_from) X(hover_mode) X(requested_mode) \
    X(link_land_us) X(voltage_mv) X(stop_generation)
#define FIELD(name) typeof(name) name;
#define SAVE(name) memcpy(&saved->name, &name, sizeof(name));
#define RESTORE(name) memcpy(&name, &saved->name, sizeof(name));
typedef struct {
    SNAPSHOT(FIELD)
    int excite_count;
} Snapshot;
static Snapshot *saved;

static void reset_statistics(void) {
    if (histograms) memset(histograms, 0, METRICS * sizeof(Histogram));
    memset(sites, 0, sizeof(sites));
    pending_mask = 0;
    previous_woke = 0;
}

static void arm_sandbox(uint64_t now) {
    command.enabled = 1;
    state.enabled = true;
    state.reason = STOP_NONE;
    flight_reset(&state.controller, &command);
    rewind_pattern();
    flip_reset(&flip);
    recover_reset(&recovery);
    held = false;
    hover_mode = requested_mode = HOVER_OFF;
    state.heartbeat_us = now;
}

static void begin_run(void) {
    run.requested = false;
    saved = heap_caps_malloc(sizeof(Snapshot), MALLOC_CAP_8BIT);
    if (!saved) {
        snprintf(run.error, sizeof(run.error), "No memory for the %d-byte snapshot", (int)sizeof(Snapshot));
        return;
    }
    SNAPSHOT(SAVE)
    saved->excite_count = excite.count;
    excite.count = 0;
    run.active = true;
    float values[SETTINGS];
    settings_copy(values);
    // As flown on 2026-11-15 07:02 UTC (the drone's own settings then; the attitude is the quaternion's alone now), with the 60-degree burst and self-trim off.
    values[SETTING_indi] = 0.5f;
    values[SETTING_indi_pitch] = 0;
    values[SETTING_recover_enable] = 1;
    values[SETTING_flip_angle] = 60;
    values[SETTING_hover_camera] = 0;
    values[SETTING_hover_gps_tau] = 0.5f;
    values[SETTING_hover_baro_tau] = 2;
    values[SETTING_auto_trim] = 0;
    settings_apply(&command, values);
    voltage_mv = 16000;
    battery_v = 16;
    reset_plant();
    uint64_t now = esp_timer_get_time();
    arm_sandbox(now);
    run.started_log = run.record && !log_active && log_head == log_tail;
    if (run.started_log) start_log();
    reset_statistics();
}

static void end_run(void) {
    run.active = false;
    SNAPSHOT(RESTORE)
    excite.count = saved->excite_count;
    rewind_pattern();
    free(saved);
    saved = NULL;
    if (run.started_log && log_active && !log_ends) log_ends = esp_timer_get_time() + 200000;
    // The outputs as the disarmed drone has them (the hardware never saw the run's).
    for (int side = 0; side < 2; side++) {
        write_servo(side);
        pulse(side * 2 + 1, ESC_STOP_US);
    }
}

// The laptop's part, each tick before it runs: the heartbeat and the hold's mode, the burst and the kicks the scenario asks for, and the pack's voltage.
static void script(uint64_t now) {
    float seconds = run.tick / (float)SAMPLE_HZ;
    // The worst case flies on after a stop (a tip cut, a failed recovery): back on the ground and armed again a second later.
    if (run.scenario == RUN_WORST && !state.enabled && run.disarmed >= SAMPLE_HZ) {
        reset_plant();
        arm_sandbox(now);
        run.disarmed = 0;
    }
    state.heartbeat_us = now;
    voltage_mv = 16000;
    command.base = 0;
    command.target_roll = command.target_pitch = command.target_yaw = 0;
    nudge[0] = nudge[1] = nudge[2] = 0;
    int mode = run.scenario == RUN_FLIGHT && seconds >= 27 ? HOVER_LAND : HOVER_HOLD;
    requested_mode = mode;
    if (held && requested_mode != held_from) held = false;
    if (!link_landing && !held && state.enabled) hover_mode = mode;
    bool burst = false;
#ifdef HAVE_REPLAY
    if (run.scenario == RUN_REPLAY) burst = replay_index() == REPLAY_BURST;
#endif
    if (run.scenario == RUN_FLIGHT) burst = run.tick == 1200;
    if (run.scenario == RUN_WORST) burst = run.tick % 600 == 500;
    if (burst && state.enabled && !flip_active(&flip) && !recover_active(&recovery)) {
        flip.done = false;
        Attitude pose = relative_attitude(&state);
        flip_begin(&flip, &state.controller, hover.integral, pose.q, hover.force, hover.climb_rate, hover.height, command.flip_angle, command.hover.ceiling);
        run.flips++;
    }
    bool kick = (run.scenario == RUN_FLIGHT && run.tick == 2000) || (run.scenario == RUN_WORST && run.tick >= 800 && run.tick % 300 == 0);
    if (kick && !plant.grounded) {
        float size = run.scenario == RUN_WORST ? 40 : 8;
        plant.rate[0] += size;
        plant.rate[1] -= size * 0.6f;
        plant.rate[2] += size * 0.3f;
        run.kicks++;
    }
}

static void bench_wake(void) {
    uint32_t now = cycles();
    if (previous_woke) {
        later(M_PERIOD, now - previous_woke);
        later(M_LATE, now - tick_interrupt);
    }
    previous_woke = woke = now;
    if (run.reinstall && !run.active) {
        // The IMU's I2C driver installed from this task, so its interrupt is on core 1 instead of core 0's (flight_runtime_start runs on core 0).
        i2c_driver_delete(I2C_NUM_0);
        sensor_configured = configure_sensor(false);
        run.reinstall = false;
        previous_woke = 0;
    }
}

static void bench_sensors(esp_err_t *result, uint8_t *data, int16_t *sensor, int32_t *sum, int *samples) {
    later(M_I2C, cycles() - woke);
    if (!run.active) return;
    if (run.scenario == RUN_REPLAY) replay_sensors();
    *result = ESP_OK;
    data[0] = 1;
    memcpy(sensor, plant.newest, sizeof(plant.newest));
    memcpy(sum, plant.sum, sizeof(plant.sum));
    *samples = PLANT_SAMPLES;
}

// The same work four times in a row on copies, so only the first meets a cold cache: the Kalman filter's update and the hold's hover_update, both disarmed on the drone's own state.
static void warm_test(void) {
    run.warm = false;
    Ekf *copy = heap_caps_malloc(sizeof(Ekf), MALLOC_CAP_8BIT);
    HoverState *hold = heap_caps_malloc(sizeof(HoverState), MALLOC_CAP_8BIT);
    if (!copy || !hold) {
        free(copy);
        free(hold);
        return;
    }
    float mhz = esp_rom_get_cpu_ticks_per_us();
    EkfSample sample = {.time_us = esp_timer_get_time(), .gyro = {state.gyro[0], state.gyro[1], state.gyro[2]}, .accel = {state.accel[0] * GRAVITY, state.accel[1] * GRAVITY, state.accel[2] * GRAVITY}, .dt = 0.01f};
    EkfOutput out;
    HoverGains gains = command.hover;
    HoverDrag drag = {.specific = {0, 0, 1}};
    HoverOutput result;
    for (int k = 0; k < 4; k++) {
        memcpy(copy, &ekf, sizeof(Ekf));
        uint32_t start = cycles();
        ekf_update(copy, &sample, &out);
        run.warm_ekf[k] = (cycles() - start) / mhz;
        memcpy(hold, &hover, sizeof(HoverState));
        start = cycles();
        hover_update(hold, &gains, HOVER_HOLD, NULL, sample.time_us, 0.01f, 0, 0, 0, 1, 10, &drag, NULL, NULL, &result);
        run.warm_hover[k] = (cycles() - start) / mhz;
    }
    free(copy);
    free(hold);
}

static void bench_begin(void) {
    for (int metric = STAGES; metric < METRICS; metric++)
        if (pending_mask >> (metric - STAGES) & 1) record(metric, pending[metric]);
    pending_mask = 0;
    if (run.requested) begin_run();
    if (run.warm) warm_test();
    if (run.active) script(esp_timer_get_time());
    memset(stage_cycles, 0, sizeof(stage_cycles));
    stage_hit = 0;
    stage_last = tick_start = cycles();
}

static void bench_end(const void *tick) {
    const Tick *t = tick;
    uint32_t now = cycles();
    for (int stage = 0; stage < STAGES; stage++)
        if (stage_hit >> stage & 1) record(stage, stage_cycles[stage]);
    record(M_COMPUTE, now - tick_start);
    if (!run.active) return;
    run.armed += state.enabled;
    run.holding += state.enabled && hover_mode == HOVER_HOLD && hover.engaged && !hover.landed && hover.height > 0.3f;
    run.phase[flip_phase(&flip) & 3]++;
    run.recovering += recover_active(&recovery);
    run.gps += t->have_fix;
    run.baro += t->have_baro;
    run.both += t->have_fix && t->have_baro;
    run.saturated += plant.saturated;
    if (state.enabled && hover.height > run.top) run.top = hover.height;
    run.disarmed = state.enabled ? 0 : run.disarmed + 1;
    if (++run.tick >= run.ticks || (run.scenario != RUN_WORST && run.disarmed > 2 * SAMPLE_HZ)) end_run();
}

// After the lock: the whole tick, the MPC as a flip would call it each tick (timed, not flown), and the plant's next sample.
static Mpc mpc;
static void bench_after(void) {
    uint32_t now = cycles();
    later(M_TICK, now - woke);
    if (!run.active) return;
    if (state.enabled && run.mpc) {
        static const FlipAuthority authority = {13.5f, 7.08f, 4.66f, 10.0f, {0, 0, 0}, {0, 0, 0}};
        float wanted[MPC_HORIZON][MPC_OUTPUTS];
        float sweep = 60 * sinf(run.tick * 0.12f);
        for (int k = 0; k < MPC_HORIZON; k++) {
            wanted[k][0] = sweep;
            wanted[k][1] = 5 * state.gyro[1];
            wanted[k][2] = 0;
            wanted[k][3] = 11;
        }
        FlipOutput out;
        uint32_t start = cycles();
        mpc_allocate(&mpc, &mpc_defaults, &authority, (const float (*)[MPC_OUTPUTS])wanted, 2.6f, 12.3f, 0.01f, &out);
        later(M_MPC, cycles() - start);
        float tilt[2] = {out.left_tilt, out.right_tilt}, force[2] = {out.left, out.right};
        mpc_sent(&mpc, &mpc_defaults, tilt, force, 0.01f);
    }
    uint32_t free_stack = uxTaskGetStackHighWaterMark(NULL);
    if (!run.stack_free || free_stack < run.stack_free) run.stack_free = free_stack;
    if (run.scenario != RUN_REPLAY) step_plant(1.0f / SAMPLE_HZ);
}

static void bench_start(void) {
    histograms = heap_caps_calloc(METRICS, sizeof(Histogram), MALLOC_CAP_8BIT);
    if (!histograms) abort();
    esp_register_freertos_tick_hook_for_cpu(bench_tick_hook, 1);
    mpc_reset(&mpc);
}

// The b request: 'r' clears the statistics; 's' scenario (1 flight, 2 replay, 3 worst), ticks (u16) and flags (1: a flight log, streamed as in flight; 2: the MPC timed after each tick, which takes core 1's time from the tick's schedule) starts a run, disarmed with no pack only; 'q' the run's progress and coverage; 'm' page the statistics (METRICS_PAGE metrics a page: n, then min, median, 99th percentile, 99.9th, max and mean in us); 'l' the lock's sites; 'i' installs the IMU's I2C driver again from the control task (its interrupt then on core 1); 'w' the cache test (warm_test, its results in q's).
#define METRICS_PAGE 12
const char *flight_runtime_bench(const uint8_t *body, size_t length, char *text, size_t size, size_t *written) {
    if (!length) return "Expected r, s, q, m, l or i";
    float mhz = esp_rom_get_cpu_ticks_per_us();
    int n = 0;
    if (body[0] == 'r' || body[0] == 'i' || body[0] == 'w') {
        LOCK();
        bool idle = !run.active && !run.requested;
        if (idle && body[0] == 'r') reset_statistics();
        if (idle && body[0] == 'i') run.reinstall = true;
        if (idle && body[0] == 'w') run.warm = true;
        UNLOCK();
        if (!idle) return "A run is under way";
        n = snprintf(text, size, "{\"ok\":1}");
    } else if (body[0] == 's') {
        if (length < 5) return "Expected scenario, ticks (u16) and record";
        int scenario = body[1], ticks = body[2] | body[3] << 8;
        if (scenario < RUN_FLIGHT || scenario > RUN_WORST || ticks < 1) return "Scenario 1-3 and at least one tick";
#ifndef HAVE_REPLAY
        if (scenario == RUN_REPLAY) return "Built without bench_replay.h";
#endif
        LOCK();
        const char *error = !running || !quiet() || maintenance || command.enabled ? "Disarm and zero throttle first"
                            : battery_v > NO_PACK_V                              ? "A pack is connected: the run is for the bench on USB alone"
                            : run.active || run.requested                        ? "A run is under way"
                                                                                 : NULL;
        if (!error) {
            memset(&run, 0, sizeof(run));
            run.scenario = scenario;
            run.ticks = ticks;
            run.record = body[4] & 1;
            run.mpc = body[4] >> 1 & 1;
            run.requested = true;
        }
        UNLOCK();
        if (error) return error;
        n = snprintf(text, size, "{\"ok\":1}");
    } else if (body[0] == 'q') {
        LOCK();
        typeof(run) copy = run;
        UNLOCK();
        n = snprintf(text, size,
                     "{\"active\":%d,\"requested\":%d,\"scenario\":%d,\"tick\":%d,\"of\":%d,\"error\":\"%s\",\"armed\":%d,\"holding\":%d,\"phases\":[%d,%d,%d,%d],\"recovering\":%d,"
                     "\"gps\":%d,\"baro\":%d,\"both\":%d,\"saturated\":%d,\"flips\":%d,\"kicks\":%d,\"top\":%.2f,\"warm_ekf\":[%.1f,%.1f,%.1f,%.1f],\"warm_hover\":[%.1f,%.1f,%.1f,%.1f],\"stack_free\":%u,\"heap_free\":%u,\"heap_low\":%u}",
                     copy.active, copy.requested, copy.scenario, copy.tick, copy.ticks, copy.error, copy.armed, copy.holding, copy.phase[0], copy.phase[1], copy.phase[2],
                     copy.phase[3], copy.recovering, copy.gps, copy.baro, copy.both, copy.saturated, copy.flips, copy.kicks, copy.top, copy.warm_ekf[0], copy.warm_ekf[1], copy.warm_ekf[2], copy.warm_ekf[3], copy.warm_hover[0], copy.warm_hover[1], copy.warm_hover[2], copy.warm_hover[3], (unsigned)copy.stack_free,
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    } else if (body[0] == 'm') {
        int page = length > 1 ? body[1] : 0, first = page * METRICS_PAGE;
        if (first >= METRICS) return "No such page";
        float table[METRICS_PAGE][7];
        int count = 0;
        if (!histograms) return "Not started";
        LOCK();
        for (int metric = first; metric < METRICS && count < METRICS_PAGE; metric++, count++) {
            const Histogram *h = &histograms[metric];
            float *row = table[count];
            row[0] = h->n;
            row[1] = h->min / mhz;
            row[2] = quantile(h, 0.5f) / mhz;
            row[3] = quantile(h, 0.99f) / mhz;
            row[4] = quantile(h, 0.999f) / mhz;
            row[5] = h->max / mhz;
            row[6] = h->n ? h->sum / h->n / mhz : 0;
        }
        UNLOCK();
        n = snprintf(text, size, "{\"mhz\":%.0f,\"page\":%d,\"pages\":%d,\"metrics\":{", mhz, page, (METRICS + METRICS_PAGE - 1) / METRICS_PAGE);
        for (int i = 0; i < count && n > 0 && (size_t)n < size; i++) {
            const float *row = table[i];
            n += snprintf(text + n, size - n, "%s\"%s\":[%.0f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", i ? "," : "", metric_names[first + i], row[0], row[1], row[2], row[3], row[4], row[5], row[6]);
        }
        if (n > 0 && (size_t)n < size) n += snprintf(text + n, size - n, "}}");
    } else if (body[0] == 'l') {
        typeof(sites) copy;
        LOCK();
        memcpy(copy, sites, sizeof(sites));
        UNLOCK();
        n = snprintf(text, size, "{\"mhz\":%.0f,\"sites\":[", mhz);
        for (int i = 0; i < SITES && copy[i].site && n > 0 && (size_t)n < size; i++)
            n += snprintf(text + n, size - n, "%s[\"%s\",%u,%.1f,%.1f,%.1f]", i ? "," : "", copy[i].site, (unsigned)copy[i].count, copy[i].wait_max / mhz, copy[i].hold_max / mhz,
                          copy[i].count ? (float)(copy[i].hold_sum / copy[i].count) / mhz : 0);
        if (n > 0 && (size_t)n < size) n += snprintf(text + n, size - n, "]}");
    } else {
        return "Expected r, s, q, m, l or i";
    }
    if (n <= 0 || (size_t)n >= size) return "Reply does not fit";
    *written = n;
    return NULL;
}
