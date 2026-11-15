#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
#include "baro.h"
#include "camera.h"
#include "ekf.h"
#include "gps.h"
#include "guard.h"
#include "flip.h"
#include "hover.h"
#include "indi.h"
#include "recover.h"

// Samples kept while the radio catches up: 5 s at 100 Hz (2 s until 2026-11-14). 500 records take 56 KB; with 200 the drone had 129 KB of internal RAM left at its lowest (radio.c I reply), so this leaves about 95 KB.
#define LOG_CAPACITY 500
#define DEG (180.0f / 3.14159265358979323846f)
// One g as the Kalman filter and the hover take it (m/s²).
#define GRAVITY 9.81f
// MPU6050 on I2C: SDA, SCL and address.
#define SDA 19
#define SCL 22
#define MPU 0x68
// Motor watchdog: the laptop's targets must keep arriving.
#define HEARTBEAT_MS 500
// A flight that loses the laptop lands itself, in any mode, instead of stopping the motors in the air (2026-10-29 07:28 dropped from 0.4 m): the landing needs nothing from the laptop. The controller's airborne time, not the hover's height estimate (outdoors, without the tag, only the IMU's), says it is flying. When the laptop returns it keeps landing until the laptop asks to fly: hover, or manual collective at liftoff (integrate_throttle); a reconnecting dashboard's zero collective does not end it. The watchdog still stops a flight not yet off the ground, and a link-loss landing that has not touched down within LINK_LAND_S (s; 60 m at LINK_LAND_RATE).
#define LINK_LAND_S 60.0f
// In the hover's hold a lost link is first waited out this long (s) with the hold flying on as it was, the laptop's tilt dropped: on 2026-11-12 the link dropped for under a second at a time at big roll angles and low on the ground (the drone's antenna is shadowed), and each landing it set off risked the backward tip-over at touchdown. Other modes land at once.
#define LINK_HOLD_S 2.0f
// Without the tag the landing holds its velocity on the rotor-drag model (hover.h drag, in 1/s), at least this much: with nothing damping the drift, a no-tag landing tipped over in 18 of 48 simulated runs, with the drag model at 1 in 2 (sw/sim/landing_test.py --no-camera, 2026-10-30).
#define LINK_LAND_DRAG 1.0f
// The link-loss landing descends at least this fast (m/s), whatever hover_land_rate (the pilot's L) is: down in seconds instead of a minute, so it drifts less sideways, and without the tag or a barometer the climb-rate estimate can be 0.3 m/s off (it read level while the drone rose 6 m in 16 s on 2026-11-08 01:56 UTC), which at 0.2 m/s could leave the drone climbing. 1 m/s at touchdown is about a 5 cm drop.
#define LINK_LAND_RATE 1.0f
// Tolerate one missed control tick; a second consecutive miss stops the motors.
#define MAX_SAMPLE_MS (2500 / SAMPLE_HZ)
// The IMU is set up again from scratch (a fresh driver, a device reset, the configuration) after this many bad control ticks in a row with the motors stopped: on 2026-11-08 19:11 PST every read failed from power-up for 11 minutes until a reset, and "IMU offline" before the first outdoor flights also needed a restart. Never with the motors enabled: a failure there has already stopped them.
#define SENSOR_RETRY_TICKS SAMPLE_HZ
// Degrees. The tip cut is the setting tip_limit (settings.def).
#define SERVO_LIMIT 60
// Banked past bank_tilt the collective's thrust is divided by the cosine of the setpoint's tilt, at most this many times: at 60 degrees holding height needs twice the thrust (the motors give about 2.2 times the weight).
#define BANK_THRUST_MAX 2.0f
#define THROTTLE_MAX 1200
// The ESC pulse for a stopped motor (us), below the 1000 us that calibration teaches as zero: an ESC calibrated at exactly 1000 us can read that, with pulse jitter, as just above its minimum and spin.
#define ESC_STOP_US 950
#define PWM_BITS 16
// Streamed target limits: degrees and degrees per second.
#define TARGET_TILT 15
#define TARGET_RATE 45
// Battery smoothing for thrust compensation (s), and the furthest the smoothed voltage may sit from thrust_voltage, as a ratio, before it is treated as an ADC fault and clamped.
#define VOLTAGE_TAU 0.3f
#define VOLTAGE_RANGE 0.15f
// Arming is refused with the pack under ARM_MIN_V (about 3.5 V a cell on 4S): on 2026-11-02 the drone armed at 9.9 V after a half hour powered on the bench and sagged to 6.3 V in the hop. Below NO_PACK_V there is no pack (the bench on USB alone), which stays allowed.
#define ARM_MIN_V 14.0f
// Self-trim learns nothing from a hop that ends with the pack under TRIM_MIN_V (3 V a cell under load on 4S): starved motors split the thrust differently, and on 2026-11-02 a hop at 6.3 V moved roll_bias from 0.4 to 3.2, which rolled the next liftoff left.
#define TRIM_MIN_V 12.0f
#define NO_PACK_V 5.0f
// Oldest camera sighting hover uses (us).
#define HOVER_POSE_US 150000

typedef struct __attribute__((packed)) {
    uint32_t elapsed, sequence;
    int16_t raw[7], attitude[2];
    uint16_t throttle[2];
    int16_t servo[2], target[3];
    uint16_t base, control_us, flags, voltage;
    // The hover loop (hover.h HoverState), in the yaw-turned level frame (forward, left, up at yaw 0) unless body axes are named: the tag position it last took (mm) and that sighting's age (ms; 65535 none or older); horizontal velocity (mm/s); height above the engage point (mm) and climb rate (mm/s); height target and range target (mm) and place on the circle (arc, centidegrees); gyro heading (centidegrees); trim (mm/s²; body forward, left); the drag model's velocity (mm/s; body forward, left).
    int16_t tag[3];
    uint16_t tag_age;
    int16_t velocity[2], height, climb, height_target, range_target, arc, yaw, trim[2], drag_velocity[2];
    // The barometer's newest reading (baro_input): pressure altitude in the standard atmosphere (mm), INT32_MIN without one in the last BARO_MAX_AGE_US.
    int32_t baro;
    // The guards (guard.h): bits 0-1 the fly-away failsafe's phase (FAILSAFE_*), bit 2 set off by distance rather than speed, bits 3-5 why the test pattern stopped (PATTERN_*, 0 while it plays or never did), bits 6-7 the flip's phase (flip.h flip_phase).
    uint16_t guard;
    // The Kalman filter's estimate (ekf.h), whether or not the hold flies it: roll and pitch as attitude above and yaw (centidegrees; from north once the heading is known, until then from where the nose pointed at the start), velocity north, west and up (mm/s), position north and west of where it last rested (cm) and height above it (mm), the standard deviations of its horizontal velocity (cm/s) and height (cm; 255 at most), and flags: bit 0 the heading is known, bit 1 resting still, bits 2 and 3 good enough for the hold to fly horizontally and vertically (ekf_estimate), bit 4 the hold flew on it this tick (hover.h gains ekf), bits 8-15 the GPS solutions it has refused in a row (255 at most).
    int16_t ekf_attitude[3], ekf_velocity[3], ekf_position[2], ekf_height;
    uint8_t ekf_sd[2];
    uint16_t ekf_flags;
    // The camera's newest dated frame (camera_newest): its sequence's low 16 bits, and how long before this sample its centre row was read (10 us; 65535 none, or older than that), which ties the camera's black-box frames to the IMU (sw/vio/blackbox_dataset.py).
    uint16_t camera_sequence, camera_age;
    // That frame's camera -> tag rotation (camera.h CameraPose rotation) as a unit quaternion w, x, y, z (32767 = 1), when the frame is a trusted sighting whose tilt the camera decided; all zero otherwise. With the tag flat on the floor it is the attitude's ground truth for checking the attitude estimate and the Kalman filter indoors (sw/flight_log.py tag_q*), which the tag's position alone cannot give.
    int16_t tag_rotation[4];
} Record;
_Static_assert(sizeof(Record) == RECORD_SIZE, "Flight record layout");

// Output channels, in this order: left servo, left ESC, right servo, right ESC (index side * 2 + ESC).
static const int pins[4] = {14, 25, 16, 18};
// LEDC ticks per microsecond of pulse.
static const float ticks[4] = {
    (1 << PWM_BITS) * SERVO_HZ / 1e6f, (1 << PWM_BITS) * ESC_HZ / 1e6f,
    (1 << PWM_BITS) * SERVO_HZ / 1e6f, (1 << PWM_BITS) * ESC_HZ / 1e6f,
};

typedef struct {
    float accel_scale[3], gyro_scale[3], gravity_tau;
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
// The last gyro readings (body rad/s) with their times, for taking the drone's rotation out of the camera's flow (flow_input).
#define GYRO_HISTORY 16
static struct { int64_t us[GYRO_HISTORY]; float rate[GYRO_HISTORY][3]; int next; } gyro_history;
// The tracking camera's focal length in its full-resolution pixels (fw/esp32cam/src/config.h fx, fy at 320x240), and the flow it is trusted with.
#define FLOW_FOCAL_PX 370.0f
#define FLOW_MIN_INLIERS 6
#define FLOW_MAX_RESIDUAL_PX 2.0f
#define FLOW_MAX_AGE_US 150000

// GPS solutions the hover takes: a 3D fix inside the module's accuracy masks, its speed known to GPS_MAX_SPEED_ACCURACY (m/s) and its position to GPS_MAX_ACCURACY (m), arriving no later than GPS_MAX_AGE_US. NAV-PVT comes out about GPS_LATENCY_US after the epoch it describes (the u-blox M10's solution time at 10 Hz plus the 100 bytes at 115200).
#define GPS_MAX_SPEED_ACCURACY 0.6f
#define GPS_MAX_ACCURACY 8.0f
#define GPS_MAX_AGE_US 300000
#define GPS_LATENCY_US 60000
// The flip asks for a fix the hold took in the last FLIP_FIX_US (hover.c GPS_STALE_US).
#define FLIP_FIX_US 500000
#define EARTH_RADIUS 6371000.0

// The newest GPS solution as the Kalman filter and the hover take it (once each): north and east of the first fix taken since boot (m), up above sea level. False without a new, good fix.
static bool gps_input(HoverGps *out) {
    static int64_t taken_us = -1;
    static int32_t latitude0, longitude0;
    static float east_scale;
    GpsFix fix;
    if (!gps_fix(&fix, GPS_MAX_AGE_US) || fix.received_us == taken_us) return false;
    const UbxPvt *p = &fix.pvt;
    if (p->fix_type != 3 || !(p->flags & 1) || p->speed_accuracy_mm_s > GPS_MAX_SPEED_ACCURACY * 1000 || p->horizontal_accuracy_mm > GPS_MAX_ACCURACY * 1000)
        return false;
    if (taken_us < 0) {
        latitude0 = p->latitude;
        longitude0 = p->longitude;
        east_scale = cos(p->latitude * 1e-7 * M_PI / 180);
    }
    taken_us = fix.received_us;
    const float metres = EARTH_RADIUS * M_PI / 180 * 1e-7;
    *out = (HoverGps){.capture_us = fix.received_us - GPS_LATENCY_US,
                      .velocity = {p->velocity_mm_s[0] / 1000.0f, p->velocity_mm_s[1] / 1000.0f, -p->velocity_mm_s[2] / 1000.0f},
                      .position = {(p->latitude - latitude0) * metres, (p->longitude - longitude0) * metres * east_scale, p->height_mm / 1000.0f}};
    return true;
}

// Barometer readings the hover takes (once each) and the log records: at most BARO_MAX_AGE_US old. A reading gives the pressure about BARO_LATENCY_US before it was read: the IIR filter's lag (three 20 ms samples at coefficient 3), half the 19 ms measurement and half the 10 ms poll.
#define BARO_MAX_AGE_US 100000
#define BARO_LATENCY_US 75000
// The newest reading's altitude this tick (m), NaN without a fresh one.
static float baro_altitude = NAN;

static bool baro_input(HoverBaro *out) {
    static int64_t taken_us = -1;
    BaroSample sample;
    bool fresh = baro_sample(&sample, BARO_MAX_AGE_US);
    baro_altitude = fresh ? sample.altitude : NAN;
    if (!fresh || sample.read_us == taken_us) return false;
    taken_us = sample.read_us;
    *out = (HoverBaro){.capture_us = sample.read_us - BARO_LATENCY_US, .altitude = sample.altitude};
    return true;
}

// The camera's newest flow as the hover takes it: its shift turned into an angular rate (rad/s; right, down) less the part the drone's own rotation over the interval explains (a rotation rate W in camera axes sweeps the image centre at -W_down right and +W_right down). False without a new, trusted flow.
static bool flow_input(const HoverGains *gains, HoverFlow *out) {
    static uint32_t sequence;
    CameraFlow flow;
    if (!camera_flow(&flow, &sequence, FLOW_MAX_AGE_US) || flow.inliers < FLOW_MIN_INLIERS || flow.residual_px > FLOW_MAX_RESIDUAL_PX ||
        flow.interval_us == 0 || flow.interval_us > 100000)
        return false;
    float mean[3] = {0, 0, 0};
    int count = 0;
    for (int k = 0; k < GYRO_HISTORY; k++) {
        int64_t t = gyro_history.us[k];
        if (t > flow.capture_us - (int64_t)flow.interval_us && t <= flow.capture_us) {
            for (int i = 0; i < 3; i++) mean[i] += gyro_history.rate[k][i];
            count++;
        }
    }
    if (!count) return false;
    for (int i = 0; i < 3; i++) mean[i] /= count;
    float camera[3];
    hover_body_to_camera(gains, mean, camera);
    float interval = flow.interval_us / 1e6f;
    out->capture_us = flow.capture_us - (int64_t)flow.interval_us / 2;
    out->rate[0] = flow.shift[0] / FLOW_FOCAL_PX / interval + camera[1];
    out->rate[1] = flow.shift[1] / FLOW_FOCAL_PX / interval - camera[0];
    return true;
}
static Command command;
// One motor's thrust (N) at a throttle on the firmware's table, and the throttle for a thrust on its rising curve, at most THROTTLE_MAX.
static float thrust(float throttle) {
    return flight_interpolate(throttle, &flight_thrust[0][0], flight_thrust_count, 0);
}
static float throttle_for(float force) {
    return fminf(THROTTLE_MAX, flight_interpolate(force, &flight_thrust[0][0], flight_thrust_count, 1));
}
// The collective throttle for a total thrust (N) along the thrust axis at pitch_trim, as the hover, the flip and the recovery's leveling ask for it.
static float collective_throttle(float force) {
    return throttle_for(force / 2 / cosf(command.pitch_trim));
}
// The controller's tilt setpoint for bank_tilt: its roll, or its pitch beyond pitch_trim, whichever is larger (radians).
static float bank_angle(const Controller *c) {
    return fmaxf(fabsf(c->setpoint_roll), fabsf(c->setpoint_pitch - command.pitch_trim));
}
static Record records[LOG_CAPACITY];
static uint32_t log_head, log_tail, log_samples, log_dropped, log_elapsed, log_session;
static uint64_t log_started;
// A stop keeps the recording going for POST_ROLL_US, so a touchdown and what follows it (a tip-over after the motors stop) are in the log; log_ends is when it closes (0: no stop pending).
#define POST_ROLL_US 1500000
static uint64_t log_ends;
// running: the control task has started; until then targets, arming and calibration are refused.
static bool log_active, running, sensor_configured;
// maintenance: a flash or NVS write is in progress, so outputs are held at zero and arming is refused.
static bool maintenance;
// Counts every stop; an arm request carries the count from when it arrived.
static uint32_t stop_generation;
// The newest laptop STOP since boot (each boot is a new radio session).
static StopBarrier stop_barrier;
static uint16_t voltage_mv;
// Servo bench test (flight_runtime_servo_test): disarmed with zero throttle, each servo in turn steps +amp and back to 0, then -amp and back, holding each for TEST_STEP_US; after each step the peak gyro deviation from the level before it is taken over TEST_WINDOW_US. peak[side][axis] sums those peaks (pitch rate, yaw rate) over steps[side] steps.
#define TEST_STEP_US 600000
#define TEST_WINDOW_US 450000
static struct {
    bool active;
    int index, count;
    float amp;
    uint64_t step_us;
    float before[2], recent[2], top[2], peak[2][2];
    int steps[2];
} servo_test;
// Smoothed battery volts, for thrust compensation; 0 until the first reading.
static float battery_v;
// The streamed hover mode (hover.h), the hover loop, and the command actually flown this tick (hover replaces base and targets), which the recording logs.
static int hover_mode;
// The link-loss landing (LINK_LAND_S) under way since link_land_us; a stop ends it, and so does the laptop asking to fly again. requested_mode is the hover mode the laptop last sent, which the landing overrides.
static bool link_landing;
static int requested_mode;
static uint64_t link_land_us;
static HoverState hover;
// The Kalman filter (ekf.h) and its newest estimate: it runs every tick, armed or not, beside the estimates above, and the hold flies on it only where the hover_ekf setting says (hover.h gains ekf).
static Ekf ekf;
static EkfOutput filtered;
// The gyro bias the attitude estimate takes off with attitude_bias (rad/s; ekf_attitude_bias), zero until the filter first pins one and again when a calibration restarts it.
static float lent_bias[3];
// The fly-away failsafe and the test pattern's limits (guard.h).
static Guard guard;
// The roll flip (flip.h), started only by flight_runtime_flip.
static FlipRun flip;
// Tumble recovery and the pilot's level-and-hold (recover.h). held: since a recovery or a level-and-hold began, the hover holds whatever mode the laptop streams, until the laptop asks for another mode than held_from, the one it streamed then (the pilot's L, or manual); the laptop's tilt is dropped only while the recovery and its grace run, so the pilot steers the hold again after it.
static RecoverRun recovery;
// INDI (indi.h; settings.def indi) and the share of its correction taken now: from INDI_START_S after the controller's airborne count began it fades in to the setting over INDI_FADE_S, and out as fast while the collective is under integrate_throttle with neither the flip nor a recovery flying the motors, or the hover has landed: the ground holds the drone against what INDI would take as unmodeled, and asking ever more against it is an integrator winding up.
#define INDI_START_S 0.5f
#define INDI_FADE_S 0.5f
static Indi indi;
static float indi_fade;
static bool held;
static int held_from;
static Command flown;
// The laptop's hold-point nudge for hover (m; toward the tag, left, up), from the targets.
#define NUDGE_MAX 1.5f
static float nudge[3];
// Self-trim: through a hop, the attitude controller's total roll and pitch command (rad/s²) summed over steady samples (from TRIM_SETTLE s after collective reached integrate_throttle, body rates under TRIM_RATE, no test pattern, not landing: the floor's wash and the touchdown are not the airframe's trim), as sw/learn_bias.py does from the log. At the stop, with at least TRIM_SAMPLES of them, each bias moves auto_trim of the way to the mean, by at most TRIM_STEP; radio.c stores it once disarmed. So a moved battery costs one hop instead of a retune.
// The hold's own trim is not carried over (hover_trim_start stays as set): it learns its push in flight, and a carried one was the last flight's room, CoM or misfit. On 2026-11-01 a sideways 0.9 m/s², learned in a hover that kept losing the tag, rolled every later liftoff 5 degrees left; a forward one had pitched liftoffs nose-up the hour before.
#define TRIM_SETTLE 1.0f
#define TRIM_RATE (15 / DEG)
#define TRIM_SAMPLES 100
#define TRIM_STEP 3.0f
static struct { float sum[2], bias[2]; int count; bool ready; } trim;
// The identification test pattern (P request, flight_runtime_pattern; kept in RAM only, so a restart clears it): piecewise-constant steps added to up to EXCITE_CHANNELS channels, played while collective is up from `after` seconds past collective first reaching integrate_throttle, `repeat` times (0 = until disarmed).
// Channels: roll and pitch targets (degrees), yaw-rate target (degrees/s), collective (throttle), left and right servo tilt (degrees, positive forward), left and right motor (throttle), and the hover hold point's speed toward the tag and to the drone's left (m/s). The targets and collective add before the controller, so the log's targets hold them; the servos and motors add after it, and the fit adds them back from the pattern (the recording's events) and the segment logged in Record flags. The hold point's speed moves it as a held arrow does (on top of the laptop's nudge, within NUDGE_MAX), so a pattern can fly steady speeds for the speed moment's fit.
#define EXCITE_CHANNELS 10
#define EXCITE_SEGMENTS 224
// Each channel's largest value (fw/controller/log_format.py PATTERN_LIMITS holds the same; test_bridge.py checks). The roll and pitch targets go to 60 degrees (10 until 2026-11-12, which refused the roll-step ladder's 20-35 degree steps), as far as pattern_tilt may be set: the pattern's tilt guard (guard.c) and the tip cut are what bound the flown tilt.
static const float excite_limit[EXCITE_CHANNELS] = {60, 60, 60, 150, 8, 8, 150, 150, 1, 1};
typedef struct {
    uint16_t ticks;
    int16_t value[EXCITE_CHANNELS];
} ExciteSegment;
static struct {
    ExciteSegment segment[EXCITE_SEGMENTS];
    int count, repeat;
    float after;
    // Playback: the segment, ticks into it and whole passes so far; this tick's segment and whether it played.
    int index, tick, played, now;
    bool active;
    // How far the hold-point channels have moved the hold point (m; toward the tag, left).
    float hold[2];
} excite;
// The control task's state, shared with the radio's requests on core 0, behind one mutex rather than a critical section: the tick (the IMU's sample, the Kalman filter, the hover, the controller and the log record; 2-3.5 ms armed by the records' control_us on 2026-11-14) and each request still run one at a time exactly as before, but interrupts stay on, on core 1 through the tick and on core 0 while a request waits for it (a critical section kept core 1's interrupts off for the whole tick and core 0's while a request spun on the lock, which the camera's 2 Mbaud UART and the radio share). Taken only from tasks, never from an interrupt or inside another critical section; created by flight_runtime_outputs, before the radio starts.
static StaticSemaphore_t lock_buffer;
static SemaphoreHandle_t lock;
#define LOCK() xSemaphoreTake(lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(lock)

static bool spinning(void) { return state.throttle[0] || state.throttle[1]; }

// Disarmed with both motors stopped.
static bool quiet(void) { return !state.enabled && !spinning(); }

static void rewind_pattern(void) {
    excite.index = excite.tick = excite.played = excite.now = 0;
    excite.active = false;
    excite.hold[0] = excite.hold[1] = 0;
    guard_pattern_reset(&guard);
}

// The hold-point nudge hover flies (m; toward the tag, left, up): the laptop's and the test pattern's.
static float held_nudge(int axis) {
    return nudge[axis] + (axis < 2 ? excite.hold[axis] : 0);
}

// ESC calibration (flight_runtime_esc_calibration): 0 off, 1 the top of the range (ESC_TOP_US), 2 its bottom (1000 us), on both ESCs while disarmed, until ESC_CALIBRATION_US after it began. Teaching the ESCs this range is what makes the thrust table's throttles mean the same thrust on both, flight to flight. The Nuofany 40A ESCs (from 2026-11-14, Hobbywing Skywalker-40A clones) were taught 1000-2000 us on the thrust stand at 50 Hz; the Flylin 30As before them took 1000-2200 us.
#define ESC_TOP_US 2000
#define ESC_CALIBRATION_US 120000000LL
static int esc_calibration;
static int64_t esc_calibration_us;

// The ESC pulse (us) for a throttle: THROTTLE_MAX spans 1000 us to ESC_TOP_US, so a throttle is the same fraction of the ESC's range whatever range it was taught (the thrust table's throttles kept their meaning across the ESC swap: the stand's 40A with a 10" prop matched the table within about 5% from mid throttle up, 2026-11-14), or ESC_STOP_US for a stopped motor; during ESC calibration its stage's pulse instead.
static float esc_us(int throttle) {
    if (esc_calibration && esp_timer_get_time() - esc_calibration_us > ESC_CALIBRATION_US) esc_calibration = 0;
    if (esc_calibration) return esc_calibration == 1 ? ESC_TOP_US : 1000;
    return throttle > 0 ? 1000 + throttle * (ESC_TOP_US - 1000.0f) / THROTTLE_MAX : ESC_STOP_US;
}

// Caller holds lock so an emergency stop cannot race a later duty write.
// Latches a duty update without waiting for the previous transition.
static void pulse(int index, float us) {
    LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[index].conf1.duty_start = 0;
    LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[index].duty.duty = (uint32_t)lroundf(us * ticks[index]) << 4;
    LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[index].conf1.duty_start = 1;
}

// On USB alone (no pack: battery_v under NO_PACK_V, or not read yet) the servos get no pulses, so they are not commanded and do not hold against load on the USB supply (servos drawing power browned out the board on 2026-11-14); the commanded angle is sent again when a pack is connected (servo_pack).
static bool servo_pack;

static void write_servo(int side) {
    pulse(side * 2, servo_pack ? 1500 + (state.servo[side] + state.offsets[side]) * 1000 / 90 : 0);
}

// Why the motors stopped, as the status and the log carry it (fw/controller/log_format.py STOP_REASONS; the tip cut, 4, and a failed recovery, 10, are recover.h's).
enum { STOP_NONE, STOP_LAPTOP, STOP_LINK, STOP_IMU, STOP_TIP, STOP_LATE, STOP_CALIBRATION, STOP_LANDED, STOP_LINK_LANDED, STOP_FAILSAFE_LANDED };

static void stop_locked(int reason) {
    if (state.enabled && trim.count >= TRIM_SAMPLES && !(battery_v > NO_PACK_V && battery_v < TRIM_MIN_V)) {
        // Roll only: its trim is the motors' thrust split, the airframe's own. The pitch trim is wherever the CoM sat this flight, which the next need not share (the battery moves between flights, and slid within them on 2026-10-31, when a carried pitch_bias kicked the nose at the next liftoff); the controller finds it in flight instead (Controller tilt, the pitch integrator), and pitch_bias stays as set.
        float old[2] = {command.roll_bias, command.pitch_bias};
        trim.bias[0] = clampf(old[0] + clampf(command.auto_trim * (trim.sum[0] / trim.count - old[0]), -TRIM_STEP, TRIM_STEP), -command.integral_limit, command.integral_limit);
        trim.bias[1] = old[1];
        trim.ready = true;
    }
    trim.sum[0] = trim.sum[1] = 0;
    trim.count = 0;
    stop_generation++;
    command.enabled = command.base = 0;
    state.enabled = false;
    state.latched = true;
    state.reason = reason;
    link_landing = false;
    flip_cancel(&flip);
    recover_reset(&recovery);
    held = false;
    state.controller = (Controller){0};
    for (int side = 0; side < 2; side++) {
        state.throttle[side] = 0;
        pulse(side * 2 + 1, ESC_STOP_US);
    }
    if (log_active && !log_ends) log_ends = esp_timer_get_time() + POST_ROLL_US;
}

static esp_err_t write_mpu(uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    return i2c_master_write_to_device(I2C_NUM_0, MPU, data, sizeof(data), pdMS_TO_TICKS(10));
}

// The MPU6050's FIFO holds every 1 kHz sample (accelerometer then gyro, big-endian, FIFO_SAMPLE bytes), and each tick takes the mean of those since the last: one sample every 10 ms let the rotors' vibration near 100 Hz (5500-6600 rpm with the 10-inch props) alias to 0-10 Hz, where the height and attitude loops act; the 10 ms mean nulls 100 Hz and its multiples, and the gyro's mean is the rotation over the tick. More than FIFO_MOST samples (a missed tick, an overflow, startup) resets the FIFO, and the tick falls back to the single newest sample. Its reads may take two FreeRTOS ticks (20 ms): at the 100 Hz tick a shorter timeout rounds to none, and every read failed (2026-10-31).
#define FIFO_SAMPLE 12
#define FIFO_MOST 16
// A FIFO read cut short (an I2C timeout, a brownout) or an overflow that wraps mid-sample leaves the stream out of step with FIFO_SAMPLE, and every later read takes gyro bytes for the accelerometer's and back (2026-11-01 after a prop swap: accel ~0 g, gyro -125 deg/s, 1 g's counts). The newest FIFO sample must match the registers' sample to within FIFO_ACCEL_SLACK and FIFO_GYRO_SLACK counts (0.75 g, 100 deg/s: vibration between the two reads stays well under, any misaligned order is off by about 1 g); otherwise, and after a failed read or a partial sample, the FIFO is reset and the tick takes the registers' sample.
#define FIFO_ACCEL_SLACK 12288
#define FIFO_GYRO_SLACK ((int)(100 * GYRO_COUNTS_PER_DEG_S))
// One big-endian value from the FIFO.
static int16_t fifo_value(const uint8_t *at) {
    return (int16_t)((at[0] << 8) | at[1]);
}

static int read_fifo(int32_t sum[6], const int16_t newest[7]) {
    uint8_t reg = 0x72, count[2];
    if (i2c_master_write_read_device(I2C_NUM_0, MPU, &reg, 1, count, 2, pdMS_TO_TICKS(20)) != ESP_OK) return 0;
    int bytes = (count[0] << 8) | count[1], samples = bytes / FIFO_SAMPLE;
    static uint8_t data[FIFO_MOST * FIFO_SAMPLE];
    reg = 0x74;
    bool good = samples <= FIFO_MOST && !(bytes % FIFO_SAMPLE);
    if (!samples && good) return 0;
    good = good && i2c_master_write_read_device(I2C_NUM_0, MPU, &reg, 1, data, samples * FIFO_SAMPLE, pdMS_TO_TICKS(20)) == ESP_OK;
    for (int i = 0; good && i < 6; i++)
        good = abs(fifo_value(data + (samples - 1) * FIFO_SAMPLE + i * 2) - newest[i < 3 ? i : i + 1]) <= (i < 3 ? FIFO_ACCEL_SLACK : FIFO_GYRO_SLACK);
    if (!good) {
        write_mpu(0x6a, 0x44);
        return 0;
    }
    for (int k = 0; k < samples; k++)
        for (int i = 0; i < 6; i++) sum[i] += fifo_value(data + k * FIFO_SAMPLE + i * 2);
    return samples;
}

// The rounded mean of samples values summing to sum.
static int16_t mean(int32_t sum, int samples) {
    return (int16_t)((sum + (sum >= 0 ? samples / 2 : -samples / 2)) / samples);
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

// Hover's rotor-drag inputs: the specific force turned from the IMU into the level reference's body axes (by pose_zero, the IMU's mounting on the frame), and the thrust vector's forward tilt the controller last commanded (each servo's physical tilt from its trim, weighted by its motor's thrust).
static HoverDrag drag_inputs(const State *value, const Command *c) {
    const float *a = value->accel;
    float zr = value->pose_zero.roll, zp = value->pose_zero.pitch;
    float y = a[1] * cosf(zr) - a[2] * sinf(zr), z = a[1] * sinf(zr) + a[2] * cosf(zr);
    HoverDrag drag = {.specific = {a[0] * cosf(zp) + z * sinf(zp), y, -a[0] * sinf(zp) + z * cosf(zp)}};
    float left = thrust(value->controller.left), right = thrust(value->controller.right);
    float left_tilt = c->model.left_servo_direction * (value->controller.left_angle - c->left_trim);
    float right_tilt = c->model.right_servo_direction * (value->controller.right_angle - c->right_trim);
    float forward = left * sinf(left_tilt) + right * sinf(right_tilt), up = left * cosf(left_tilt) + right * cosf(right_tilt);
    drag.tilt = up > 0 ? atan2f(forward, up) : 0;
    return drag;
}

// A value in thousandths (mm, mm/s, mm/s²) or hundredths (centidegrees) of its unit, saturated to int16.
static int16_t scaled(float value, float scale) {
    float x = value * scale;
    return x >= INT16_MAX ? INT16_MAX : x <= INT16_MIN ? INT16_MIN : (int16_t)lroundf(x);
}

// The guards' and the flip's state as the record's guard field and the status's flags carry them: bits 0-1 the fly-away failsafe's phase (FAILSAFE_*), bit 2 set off by distance rather than speed, bits 3-5 why the test pattern stopped (PATTERN_*), bits 6-7 the flip's phase (flip.h flip_phase).
static int guard_bits(void) {
    return guard.failsafe | guard.far << 2 | guard.stopped << 3 | flip_phase(&flip) << 6;
}

// A row-major rotation matrix as a unit quaternion (w, x, y, z, w >= 0), from its largest diagonal term so it stays exact near half turns: a camera looking down at a floor tag sits near one.
static void rotation_quaternion(const float *m, float *q) {
    float trace = m[0] + m[4] + m[8];
    if (trace > 0) {
        float s = 2 * sqrtf(1 + trace);
        q[0] = s / 4, q[1] = (m[7] - m[5]) / s, q[2] = (m[2] - m[6]) / s, q[3] = (m[3] - m[1]) / s;
    } else if (m[0] > m[4] && m[0] > m[8]) {
        float s = 2 * sqrtf(1 + m[0] - m[4] - m[8]);
        q[0] = (m[7] - m[5]) / s, q[1] = s / 4, q[2] = (m[1] + m[3]) / s, q[3] = (m[2] + m[6]) / s;
    } else if (m[4] > m[8]) {
        float s = 2 * sqrtf(1 + m[4] - m[0] - m[8]);
        q[0] = (m[2] - m[6]) / s, q[1] = (m[1] + m[3]) / s, q[2] = s / 4, q[3] = (m[5] + m[7]) / s;
    } else {
        float s = 2 * sqrtf(1 + m[8] - m[0] - m[4]);
        q[0] = (m[3] - m[1]) / s, q[1] = (m[2] + m[6]) / s, q[2] = (m[5] + m[7]) / s, q[3] = s / 4;
    }
    float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) * (q[0] < 0 ? -1 : 1);
    for (int i = 0; i < 4; i++) q[i] /= norm;
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
    // Bit 0 armed, bits 1-2 the hover mode, bit 3 hover sees the tag, bit 4 a hover landing touched down, bit 5 the drag model ran, bit 6 hover flies on the tag (tracking), bit 7 the range and arc targets are set, bit 8 the test pattern played this tick and bits 9-15 its segment (modulo 128).
    record->flags = state.enabled | hover_mode << 1 | (hover.have_pose && (int64_t)now - hover.seen_us <= 250000) << 3 | hover.landed << 4 |
                    hover.dragging << 5 | hover.tracking << 6 | hover.have_setpoint << 7 | excite.active << 8 | (excite.now & 127) << 9;
    record->voltage = voltage_mv;
    for (int i = 0; i < 3; i++) record->tag[i] = scaled(hover.position[i], 1000);
    int64_t age = ((int64_t)now - hover.capture_us) / 1000;
    record->tag_age = !hover.have_pose ? UINT16_MAX : age < 0 ? 0 : age >= UINT16_MAX ? UINT16_MAX : age;
    for (int i = 0; i < 2; i++) {
        record->velocity[i] = scaled(hover.velocity[i], 1000);
        record->trim[i] = scaled(hover.carry[i], 1000);
        record->drag_velocity[i] = scaled(hover.drag_velocity[i], 1000);
    }
    record->height = scaled(hover.height, 1000);
    record->climb = scaled(hover.climb_rate, 1000);
    record->height_target = scaled(hover.target, 1000);
    // The targets as nudged (hover.c): the range to hold and the place on the circle.
    float range = hover.setpoint - held_nudge(0);
    record->range_target = scaled(range, 1000);
    record->arc = scaled((hover.arc - (range > 0.1f ? held_nudge(1) / range : 0)) * DEG, 100);
    record->yaw = scaled(hover.yaw * DEG, 100);
    record->baro = isfinite(baro_altitude) ? (int32_t)lroundf(baro_altitude * 1000) : INT32_MIN;
    record->guard = guard_bits() | recovery.stage << 8 | (recover_active(&recovery) ? recovery.cause : 0) << 10;
    record->ekf_attitude[0] = scaled((filtered.roll - state.pose_zero.roll) * DEG, 100);
    record->ekf_attitude[1] = scaled((filtered.pitch - state.pose_zero.pitch) * DEG, 100);
    record->ekf_attitude[2] = scaled(filtered.yaw * DEG, 100);
    for (int i = 0; i < 3; i++) record->ekf_velocity[i] = scaled(filtered.velocity[i], 1000);
    for (int i = 0; i < 2; i++) record->ekf_position[i] = scaled(filtered.position[i], 100);
    record->ekf_height = scaled(filtered.height, 1000);
    record->ekf_sd[0] = fminf(filtered.velocity_sd * 100, 255);
    record->ekf_sd[1] = fminf(filtered.height_sd * 100, 255);
    record->ekf_flags = filtered.aligned | filtered.still << 1 | hover.estimate.horizontal << 2 | hover.estimate.vertical << 3 | hover.flown << 4 |
                        (filtered.rejected > 255 ? 255 : filtered.rejected) << 8;
    uint32_t camera_sequence;
    int64_t centre_us;
    int64_t camera_age = camera_newest(&camera_sequence, &centre_us) ? ((int64_t)now - centre_us) / 10 : UINT16_MAX;
    record->camera_sequence = camera_sequence & 0xffff;
    record->camera_age = camera_age < 0 ? 0 : camera_age > UINT16_MAX ? UINT16_MAX : camera_age;
    CameraPose seen;
    float q[4] = {0};
    if (camera_candidates(&seen, HOVER_POSE_US) && seen.valid && seen.sequence == camera_sequence) rotation_quaternion(seen.rotation, q);
    for (int i = 0; i < 4; i++) record->tag_rotation[i] = scaled(q[i], 32767);
}

static void reconfigure_sensor(void);

// One control tick's values that its stages hand on: the sample's time and interval, whether the laptop's link is stale, this tick's GPS fix and barometer reading for the hover, the attitude from level, the specific force along vertical (g), and what the flip and the recovery ask of the actuators.
typedef struct {
    uint64_t now;
    float dt, vertical;
    bool stale, have_fix, have_baro;
    HoverGps gps;
    HoverBaro baro;
    Attitude pose;
    FlipOutput flipped;
    RecoverOutput recovered;
} Tick;

// The laptop's link: a lost one starts the link-loss landing in the air (LINK_LAND_S) or stops the motors; a returning one that asks to fly ends the landing. Returns whether the link is stale (no targets for HEARTBEAT_MS).
static bool watch_link(uint64_t now) {
    // now was read before the lock, so a target accepted on the network core in between leaves heartbeat_us just after it; signed, that is a fresh heartbeat rather than an unsigned wrap to a huge age.
    bool stale = (int64_t)(now - state.heartbeat_us) > HEARTBEAT_MS * 1000LL;
    // Airborne, as the link-loss landing takes it. Flying in the hold, a stale link is lost only once it has been gone LINK_HOLD_S and a recovery ended that long ago (the hold it hands to is waited out too, as sw/sim/flip_firmware.py flies it); until then it neither lands nor stops.
    bool flying = state.controller.airborne > 0 && !hover.landed && hover.bias > 0.5f;
    bool waited = (int64_t)(now - state.heartbeat_us) > (int64_t)(LINK_HOLD_S * 1e6f) && !(recovery.since >= 0 && recovery.since < LINK_HOLD_S);
    bool lost = flying && hover_mode == HOVER_HOLD ? waited : stale;
    if (link_landing && !stale && (requested_mode == HOVER_HOLD || (requested_mode == HOVER_OFF && command.base >= command.integrate_throttle))) {
        link_landing = false;
        hover_mode = requested_mode;
    }
    // A flip that has begun its spin finishes first (flip.h): it needs nothing from the laptop and hands back within a few seconds; one still climbing ends here.
    if (state.enabled && stale) flip_abort(&flip, &hover.integral);
    if (state.enabled && (flip_active(&flip) || recover_active(&recovery))) {
        // The landing or the stop below, once it is done.
    } else if (state.enabled && (link_landing || (lost && flying))) {
        if (!link_landing) {
            link_landing = true;
            link_land_us = now;
        }
        hover_mode = HOVER_LAND;
        if (now - link_land_us > (uint64_t)(LINK_LAND_S * 1e6f)) stop_locked(STOP_LINK);
    } else if (!quiet() && lost) {
        stop_locked(STOP_LINK);
    }
    return stale;
}

// The tick's IMU sample in body axes: the mean of the FIFO's samples in place of the newest (the temperature stays the newest). The log takes it rounded to counts (raw); the estimates take it unrounded (body): at ±2000 deg/s a count is 0.061 deg/s, more than the resting gyro's noise (0.03-0.07 deg/s RMS), so a rounded mean would quantize the calibrated bias and leave the attitude estimate up to a degree or so off in flight (half a count over its 50 s powered time constant).
static void imu_sample(int16_t *sensor, const int32_t *sum, int samples, int16_t *raw, float *body) {
    float fine[7];
    for (int i = 0; i < 7; i++) fine[i] = sensor[i];
    if (samples) {
        for (int i = 0; i < 3; i++) {
            sensor[i] = mean(sum[i], samples);
            sensor[4 + i] = mean(sum[3 + i], samples);
            fine[i] = (float)sum[i] / samples;
            fine[4 + i] = (float)sum[3 + i] / samples;
        }
    }
    // Reordered to body axes, so the log's raw values need only the signs.
    raw[3] = sensor[3];
    for (int i = 0; i < 3; i++) {
        raw[i] = sensor[sensor_axis[i]];
        raw[4 + i] = sensor[4 + sensor_axis[i]];
        body[i] = fine[sensor_axis[i]];
        body[4 + i] = fine[4 + sensor_axis[i]];
    }
}

// Calibrated accelerometer (g) and gyro (rad/s, less its bias) into state, with a gyro calibration's sums and its result once complete, and the gyro history the camera's flow needs.
static void take_imu(const float *body, uint64_t now) {
    for (int i = 0; i < 3; i++) {
        state.accel[i] = (body[i] * config.accel_scale[i] - state.accel_offset[i]) * state.accel_gain[i];
        state.gyro[i] = body[4 + i] * config.gyro_scale[i];
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
            // The gyro's zero has just moved, so the filter starts over from this rest.
            ekf_reset(&ekf);
            memset(lent_bias, 0, sizeof(lent_bias));
        }
    }
    for (int i = 0; i < 3; i++) state.gyro[i] -= state.bias[i];
    gyro_history.us[gyro_history.next] = now;
    memcpy(gyro_history.rate[gyro_history.next], state.gyro, sizeof(state.gyro));
    gyro_history.next = (gyro_history.next + 1) % GYRO_HISTORY;
}

// INDI's tick (indi.h), before the flip, the recovery and the controller, which take its unmodeled acceleration on their own models: the controller's (Model, which the recovery's authority shares) and the flip's. Off and reset while disarmed.
static void incremental(float dt) {
    if (!state.enabled) {
        indi_reset(&indi);
        indi_fade = 0;
        return;
    }
    bool owned = flip_active(&flip) || recover_active(&recovery);
    bool flying = command.indi > 0 && !hover.landed && state.controller.airborne > INDI_START_S && (owned || flown.base >= command.integrate_throttle);
    indi_fade = clampf(indi_fade + (flying ? dt : -dt) * command.indi / INDI_FADE_S, 0, command.indi);
    const Model *m = &command.model;
    const FlipAuthority *a = &flip.authority;
    IndiModel models[INDI_MODELS] = {
        [INDI_CONTROLLER] = {m->arm / m->roll_inertia, m->pivot_height / m->pitch_inertia, m->com_x / m->pitch_inertia, m->arm / m->yaw_inertia, m->reaction_torque_ratio / m->yaw_inertia},
        [INDI_FLIP] = {a->roll, a->pitch, 0, a->yaw, 0},
    };
    indi_update(&indi, state.gyro, models, dt);
    flip.authority.indi = indi_fade;
    memcpy(flip.authority.unmodeled, indi.unmodeled[INDI_FLIP], sizeof(flip.authority.unmodeled));
}

// The attitude estimate and the flip's own (it flies on the gyro, and at its hand-back the estimate takes its attitude), then the Kalman filter, which takes every tick, GPS solution and barometer reading, flying or not, and hands the hold its estimate. Returns when the timed part of the tick (the log's control_us) began.
static uint64_t estimate(Tick *t, bool first) {
    if (first) {
        state.attitude.roll = atan2f(state.accel[1], state.accel[2]);
        state.attitude.pitch = atan2f(-state.accel[0], hypotf(state.accel[1], state.accel[2]));
    }
    // Banked past bank_tilt (last tick's setpoint, from pitch_trim), and through a flip or a recovery and their grace after, the accelerometer reads the thrust, not gravity: the gyro alone carries the attitude.
    bool banked = command.bank_tilt > 0 && spinning() && bank_angle(&state.controller) > command.bank_tilt;
    float bias[3] = {0, 0, 0};
    if (command.attitude_bias > 0) memcpy(bias, lent_bias, sizeof(bias));
    Motion motion = {state.accel[0], state.accel[1], state.accel[2],
                     state.gyro[0] - bias[0], state.gyro[1] - bias[1], state.gyro[2] - bias[2], t->dt,
                     banked || flip_gyro_only(&flip) || recover_gyro_only(&recovery) ? INFINITY : spinning() ? command.attitude_tau : config.gravity_tau};
    flight_attitude(&state.attitude, &motion);
    incremental(t->dt);
    if (state.enabled) flip_tick(&flip, state.gyro, state.accel, t->dt, &state.attitude, &state.controller, &hover.integral, &t->flipped);
    state.online = true;
    state.sample_us = t->now;
    // The barometer's newest reading, taken once this tick by whichever of the hover's calls runs.
    t->have_baro = baro_input(&t->baro);
    uint64_t started = esp_timer_get_time();
    t->have_fix = gps_input(&t->gps);
    if (t->have_fix) ekf_gps(&ekf, &t->gps);
    if (t->have_baro) ekf_baro(&ekf, &t->baro);
    EkfSample sample = {.time_us = t->now, .gyro = {state.gyro[0], state.gyro[1], state.gyro[2]},
                        .accel = {state.accel[0] * GRAVITY, state.accel[1] * GRAVITY, state.accel[2] * GRAVITY}, .dt = t->dt, .spinning = spinning()};
    ekf_update(&ekf, &sample, &filtered);
    ekf_attitude_bias(&filtered, lent_bias);
    HoverEstimate estimate;
    ekf_estimate(&filtered, &estimate);
    hover_estimate(&hover, &estimate);
    t->pose = relative_attitude(&state);
    return started;
}

// The tip cut, or with recover_enable a recovery in its place (recover.h); a level-and-hold starts here too. Whatever was flying, the hold takes over after a recovery: a test pattern stops for good, a link-loss landing gives way to it (and starts again after it if the link is still lost).
static void tip_or_recover(Tick *t) {
    bool landing = hover_mode > HOVER_HOLD || link_landing;
    RecoverInput input = {
        .enabled = command.recover_enable >= 1,
        .tipped = !flip_tip_exempt(&flip) && (fabsf(t->pose.roll) > command.tip_limit || fabsf(t->pose.pitch) > command.tip_limit),
        .allowed = state.controller.airborne >= RECOVER_AIRBORNE_S && !hover.landed && !hover.probing && !(hover_mode && hover.engaged && hover.lifted < 0) &&
                   !(landing && hover.height < RECOVER_LAND_HEIGHT) && recovery.recoveries < RECOVER_MAX,
        .gyro = state.gyro, .accel = state.accel, .dt = t->dt, .tip_limit = command.tip_limit,
        .flip_active = flip_active(&flip), .flip_gyro_only = flip_gyro_only(&flip), .flip_phase = flip_phase(&flip),
        .climb_rate = hover.climb_rate, .collective = 2 * thrust(fmaxf(flown.base, 0)),
        .authority = {command.model.arm / command.model.roll_inertia, command.model.pivot_height / command.model.pitch_inertia, command.model.arm / command.model.yaw_inertia, command.hover.mass * GRAVITY, indi_fade,
                      {indi.unmodeled[INDI_CONTROLLER][0], indi.unmodeled[INDI_CONTROLLER][1], indi.unmodeled[INDI_CONTROLLER][2]}},
    };
    bool before = recover_active(&recovery);
    recover_tick(&recovery, &recover_defaults, &input, &state.attitude, &state.controller, &hover.integral, &t->recovered);
    if (t->recovered.cancel_flip) {
        flip_cancel(&flip);
        t->flipped = (FlipOutput){0};
    }
    if (!before && recover_active(&recovery)) {
        if (!held) held_from = requested_mode;
        held = true;
        hover_mode = HOVER_HOLD;
        link_landing = false;
        if (excite.count && !guard.stopped) guard.stopped = PATTERN_RECOVERY;
    }
    if (t->recovered.cut) stop_locked(t->recovered.cut);
    t->pose = relative_attitude(&state);
}

// The hover's setpoint for the attitude controller (setpoint's collective, tilt and yaw rate), its fly-away failsafe and its landing's disarm. Returns whether it flies on the tag (tracking), when the controller takes its measured velocity and leaves the damping to it.
static bool hover_setpoint(Tick *t, Command *setpoint) {
    CameraPose camera;
    HoverPose seen, *tag = NULL;
    // Every frame whose tag corners the camera trusted, whether or not it settled the tilt: hover flies on the tag centre, which the tilt does not affect, and takes the wall's normal (the tag's z axis in the camera frame) only from an unambiguous tilt.
    if (command.hover_camera && camera_candidates(&camera, HOVER_POSE_US)) {
        seen = (HoverPose){.sequence = camera.sequence, .capture_us = camera.capture_us,
                           .tag = {camera.tag_position_m[0], camera.tag_position_m[1], camera.tag_position_m[2]}};
        if (camera.valid == 1) memcpy(seen.normal, &camera.rotation[6], sizeof(seen.normal));
        tag = &seen;
    }
    float flying = hover.engaged ? hover.force : command.base > 0 ? 2 * thrust(command.base) : 0;
    // Centring climbs looking for the tag, which only makes sense with the camera on.
    HoverGains gains = command.hover;
    if (!command.hover_camera) gains.center = 0;
    gains.pitch_trim = command.pitch_trim;
    // The pattern's hold-point speed takes effect a tick late: it plays after hover, with the rest of the pattern.
    gains.nudge_forward = held_nudge(0);
    gains.nudge_left = held_nudge(1);
    gains.nudge_up = held_nudge(2);
    // Without the tag the laptop's tilt (the arrows, A and D) flies the drone directly (on the tag they move the hold point instead); a link-loss landing ignores what the lost laptop last sent.
    bool steered = !(command.hover_camera || link_landing || t->stale || recover_gyro_only(&recovery));
    gains.stick_pitch = steered ? command.target_pitch : 0;
    gains.stick_roll = steered ? command.target_roll : 0;
    if (link_landing && !hover.tracking) gains.drag = fmaxf(gains.drag, LINK_LAND_DRAG);
    if (link_landing) gains.land_rate = fmaxf(gains.land_rate, LINK_LAND_RATE);
    HoverOutput out;
    // Heading rate about vertical from the body rates (positive left).
    float heading_rate = (state.gyro[1] * sinf(t->pose.roll) + state.gyro[2] * cosf(t->pose.roll)) / cosf(t->pose.pitch);
    HoverDrag drag = drag_inputs(&state, &command);
    HoverFlow flow;
    bool have_flow = gains.flow_depth > 0 && flow_input(&gains, &flow);
    bool have_gps = gains.gps_tau > 0 && t->have_fix;
    // The fly-away failsafe may ignore the stick, brake harder or land (guard.c).
    int mode = guard_hover(&guard, &command.guard, &hover, &gains, hover_mode, t->now, t->dt);
    // The failsafe ends a flip that is still climbing; one past that holds until it is done, its estimators running on (its outputs are not flown meanwhile).
    if (guard.failsafe != FAILSAFE_OFF) flip_abort(&flip, &hover.integral);
    if (flip_active(&flip) || recover_active(&recovery)) mode = HOVER_HOLD;
    hover_update(&hover, &gains, mode, tag, t->now, t->dt, t->pose.roll, t->pose.pitch, heading_rate, t->vertical, flying, &drag,
                 have_flow ? &flow : NULL, have_gps ? &t->gps : NULL, t->have_baro ? &t->baro : NULL, &out);
    // A finished landing disarms, ending the session like a stop, so the laptop can take off again (H or Enable) instead of being left armed with hover holding thrust at zero.
    if (hover.landed && out.force <= 0 && !flip_active(&flip) && !recover_active(&recovery)) stop_locked(link_landing ? STOP_LINK_LANDED : guard.failsafe == FAILSAFE_LAND ? STOP_FAILSAFE_LANDED : STOP_LANDED);
    setpoint->base = out.force > 0 ? collective_throttle(out.force) : 0;
    setpoint->target_roll = out.roll;
    setpoint->target_pitch = out.pitch;
    // The tag steers the heading, and the laptop's yaw (A and D) overrides it while held, as in manual flight (and always without the camera, where nothing else steers it): on 2026-11-14 a liftoff yaw of 20-30 degrees turned the camera off the tag and the keys could not turn it back. Before the tag is first seen the hold then keeps the heading the keys left it at, not the one it started with.
    bool pilot_yaw = command.target_yaw != 0;
    setpoint->target_yaw = command.hover_camera && !pilot_yaw ? out.yaw_rate : command.target_yaw;
    if (pilot_yaw) hover.yaw_hold = hover.yaw;
    // Once flying on the tag, hover damps the velocity itself, measured by the camera and dead-reckoned through losses: flight_math's damping is off, since its leaky integral of attitude error would cancel any steady tilt hover asks for, and its velocity follows hover's, so it would resume from the real velocity.
    if (!out.tracking) return false;
    state.controller.velocity[0] = out.velocity[0];
    state.controller.velocity[1] = out.velocity[1];
    setpoint->velocity_damping = 0;
    return true;
}

// The identification test pattern's step this tick (add, in each channel's unit), whether it plays, and its targets and collective added to the setpoint; the servos and motors add after the controller (fly).
static void play_pattern(const Tick *t, Command *setpoint, float *add) {
    excite.active = excite.count && !guard.stopped && setpoint->base > 0 && state.controller.airborne > excite.after && (!excite.repeat || excite.played < excite.repeat) && !flip_gyro_only(&flip) &&
                    !recover_gyro_only(&recovery);
    // A landing of any kind (the laptop's, the link-loss one) or the fly-away failsafe in any phase, its brake included, ends the pattern for good (stop reason landing): on 2026-11-12 01:59 UTC the link dropped after the 50-degree step and the -55-degree step played into the link-loss descent at 2.4 m, which came down from it at 1.5 m.
    if (excite.active && !guard.stopped && (link_landing || hover_mode > HOVER_HOLD || guard.failsafe != FAILSAFE_OFF)) guard.stopped = PATTERN_LANDING;
    // The pattern stops for good (until the next arming) once it strays, loses the tag, tilts too far or the pilot steers (guard.c); the hold keeps flying, its hold point where the pattern left it.
    if (excite.active && guard_pattern(&guard, &command.guard, hover_mode && hover.engaged ? &hover : NULL, t->now, t->pose.roll, t->pose.pitch, command.target_roll, command.target_pitch, nudge, excite.hold)) excite.active = false;
    if (!excite.active) return;
    const ExciteSegment *segment = &excite.segment[excite.index];
    for (int i = 0; i < EXCITE_CHANNELS; i++) add[i] = segment->value[i] / 100.0f;
    excite.now = excite.index;
    if (++excite.tick >= segment->ticks) {
        excite.tick = 0;
        if (++excite.index >= excite.count) {
            excite.index = 0;
            excite.played++;
        }
    }
    setpoint->target_roll += add[0] / DEG;
    setpoint->target_pitch += add[1] / DEG;
    setpoint->target_yaw += add[2] / DEG;
    setpoint->base = fmaxf(setpoint->base + add[3], 0);
    for (int i = 0; i < 2; i++) excite.hold[i] = clampf(excite.hold[i] + add[8 + i] / SAMPLE_HZ, -NUDGE_MAX, NUDGE_MAX);
}

// Self-trim's sums (trim): through a hop's steady samples, the attitude controller's total roll and pitch command, as flight_math.c makes it (the roll integrator holds its bias, while the pitch bias adds on top of the pitch integrator, fully faded in by now).
static void sum_trim(const Tick *t, float base) {
    const Controller *c = &state.controller;
    if (command.auto_trim > 0 && !excite.active && !flip_gyro_only(&flip) && !recover_gyro_only(&recovery) && hover_mode <= HOVER_HOLD && base >= command.integrate_throttle && c->airborne > TRIM_SETTLE &&
        fabsf(state.gyro[0]) < TRIM_RATE && fabsf(state.gyro[1]) < TRIM_RATE) {
        trim.sum[0] += command.roll_p * (c->setpoint_roll - t->pose.roll) + c->roll_i - command.roll_damping * state.gyro[0];
        trim.sum[1] += command.pitch_p * (c->setpoint_pitch - t->pose.pitch) + c->pitch_i +
                       clampf(command.pitch_bias, -command.integral_limit, command.integral_limit) - command.pitch_damping * state.gyro[1];
        trim.count++;
    }
}

// An armed tick's actuators. The attitude controller always runs; its setpoint comes from, in rising priority, the laptop (manual) or the hover, the test pattern's steps on top, the flip's climb and arrest collective, and the recovery's leveling. The actuators' owner is then the flip's spin and righting or the recovery's righting while either flies the motors and servos itself (flip.h, recover.h; the controller's outputs unused, and it takes over at the hand-back), otherwise the controller. Through the flip, a recovery and their graces each motor's throttle is then slewed and capped (flip_limit); the pattern's servo and motor steps go on last.
static void fly(Tick *t) {
    command.roll = t->pose.roll;
    command.pitch = t->pose.pitch;
    command.gx = state.gyro[0]; command.gy = state.gyro[1]; command.gz = state.gyro[2];
    command.dt = t->dt;
    Command setpoint = command;
    if (hover_mode != HOVER_HOLD) flip_abort(&flip, &hover.integral);
    bool hovering = hover_mode && hover.bias > 0.5f;
    // The pitch speed moment compensation needs a measured speed: the hover's while it flies on the tag; flight_math's own dead reckoning from g tan(pitch) reads a held trim pitch as speed.
    bool tracking = hovering && hover_setpoint(t, &setpoint);
    float add[EXCITE_CHANNELS] = {0};
    play_pattern(t, &setpoint, add);
    if (!tracking) setpoint.pitch_speed = 0;
    // Banked past bank_tilt (last tick's setpoint, as the attitude estimate), the collective's thrust grows by 1 / cos(tilt) so the bank holds its height. Not under the hover, whose thrust is already divided by the cosine of the measured tilt (hover.c, at most twice): both made it up to four times.
    const Controller *bank = &state.controller;
    if (command.bank_tilt > 0 && setpoint.base > 0 && !hovering && bank_angle(bank) > command.bank_tilt) {
        float lift = cosf(bank->setpoint_roll) * cosf(bank->setpoint_pitch - command.pitch_trim);
        setpoint.base = throttle_for(thrust(setpoint.base) * fminf(1 / fmaxf(lift, 1 / BANK_THRUST_MAX), BANK_THRUST_MAX));
    }
    if (t->flipped.collective_set) setpoint.base = collective_throttle(t->flipped.collective);
    if (t->recovered.level) {
        setpoint.target_roll = setpoint.target_pitch = 0;
        setpoint.base = collective_throttle(t->recovered.motors.collective);
    }
    setpoint.indi = indi_fade;
    memcpy(setpoint.unmodeled, indi.unmodeled[INDI_CONTROLLER], sizeof(setpoint.unmodeled));
    flown = setpoint;
    // Propeller thrust at a fixed throttle goes with the square of battery voltage, so the table's newtons are scaled to the measured voltage: a sagging pack gets more throttle for the same force.
    if (command.thrust_voltage > 0 && battery_v > 0) {
        float ratio = clampf(battery_v / command.thrust_voltage, 1 - VOLTAGE_RANGE, 1 + VOLTAGE_RANGE);
        setpoint.left_thrust_scale *= ratio * ratio;
        setpoint.right_thrust_scale *= ratio * ratio;
    }
    flight_control(&state.controller, &setpoint, &flight_thrust[0][0], flight_thrust_count);
    float throttle[2];
    if (t->flipped.owned || t->recovered.owned) {
        flip_actuators(t->recovered.owned ? &t->recovered.motors : &t->flipped, &command, setpoint.left_thrust_scale, setpoint.right_thrust_scale, THROTTLE_MAX, SERVO_LIMIT, throttle, state.servo);
    } else {
        throttle[0] = lroundf(clampf(state.controller.left, 0, THROTTLE_MAX));
        throttle[1] = lroundf(clampf(state.controller.right, 0, THROTTLE_MAX));
        state.servo[0] = clampf(state.controller.left_angle * DEG, -SERVO_LIMIT, SERVO_LIMIT);
        state.servo[1] = clampf(state.controller.right_angle * DEG, -SERVO_LIMIT, SERVO_LIMIT);
    }
    flip_limit(&flip, recover_gyro_only(&recovery), throttle, t->dt);
    for (int side = 0; side < 2; side++) state.throttle[side] = lroundf(throttle[side]);
    sum_trim(t, setpoint.base);
    if (excite.active) {
        state.servo[0] = clampf(state.servo[0] + command.model.left_servo_direction * add[4], -SERVO_LIMIT, SERVO_LIMIT);
        state.servo[1] = clampf(state.servo[1] + command.model.right_servo_direction * add[5], -SERVO_LIMIT, SERVO_LIMIT);
        // A stopped motor stays stopped.
        for (int side = 0; side < 2; side++)
            if (state.throttle[side]) state.throttle[side] = lroundf(clampf(state.throttle[side] + add[6 + side], 1, THROTTLE_MAX));
    }
    // What INDI's actuator estimate follows: each assembly's physical forward tilt and each motor's thrust as sent.
    float tilt[2], force[2], scale[2] = {setpoint.left_thrust_scale, setpoint.right_thrust_scale}, trims[2] = {command.left_trim, command.right_trim};
    float direction[2] = {command.model.left_servo_direction, command.model.right_servo_direction};
    for (int side = 0; side < 2; side++) {
        tilt[side] = direction[side] * (state.servo[side] / DEG - trims[side]);
        force[side] = state.throttle[side] > 0 ? thrust(state.throttle[side]) * scale[side] : 0;
    }
    indi_sent(&indi, tilt, force);
    for (int side = 0; side < 2; side++) {
        write_servo(side);
        pulse(side * 2 + 1, esc_us(state.throttle[side]));
    }
}

// The servo bench test's tick (flight_runtime_servo_test), disarmed with zero throttle only: the gyro's level just before a step, then its largest deviation in the window after; the next step once this one has been held TEST_STEP_US.
static void servo_test_tick(uint64_t now) {
    if (!quiet()) {
        servo_test.active = false;
        return;
    }
    for (int axis = 0; axis < 2; axis++) {
        float rate = state.gyro[axis + 1];
        if (now - servo_test.step_us < TEST_WINDOW_US) servo_test.top[axis] = fmaxf(servo_test.top[axis], fabsf(rate - servo_test.before[axis]));
        else servo_test.recent[axis] += (rate - servo_test.recent[axis]) * 0.2f;
    }
    if (now - servo_test.step_us < TEST_STEP_US) return;
    int side = servo_test.index / 4 % 2;
    if (servo_test.index > 0) {
        int last = (servo_test.index - 1) / 4 % 2;
        for (int axis = 0; axis < 2; axis++) servo_test.peak[last][axis] += servo_test.top[axis];
        servo_test.steps[last]++;
    }
    if (servo_test.index >= servo_test.count) {
        servo_test.active = false;
        state.servo[0] = state.servo[1] = 0;
    } else {
        static const float pattern[4] = {1, 0, -1, 0};
        state.servo[side] = pattern[servo_test.index % 4] * servo_test.amp;
        state.servo[1 - side] = 0;
        for (int axis = 0; axis < 2; axis++) {
            servo_test.before[axis] = servo_test.recent[axis];
            servo_test.top[axis] = 0;
        }
        servo_test.step_us = now;
        servo_test.index++;
    }
    write_servo(0);
    write_servo(1);
}

static void runtime_task(void *arg) {
    TickType_t deadline = xTaskGetTickCount();
    TickType_t period = configTICK_RATE_HZ / SAMPLE_HZ;
    uint64_t previous = 0;
    // Bad ticks in a row (SENSOR_RETRY_TICKS), and whether the stop reason shown is a failure with the motors already stopped, which a recovered sensor clears.
    int bad_ticks = 0;
    bool idle_failure = false;
    while (true) {
        uint8_t reg = 0x3a, data[15] = {0};
        esp_err_t result = sensor_configured
            ? i2c_master_write_read_device(I2C_NUM_0, MPU, &reg, 1,
                                          data, sizeof(data), pdMS_TO_TICKS(10))
            : ESP_ERR_INVALID_STATE;
        int16_t sensor[7], raw[7];
        for (int i = 0; i < 7; i++) sensor[i] = (int16_t)((data[1 + i * 2] << 8) | data[2 + i * 2]);
        int32_t sum[6] = {0};
        int samples = result == ESP_OK ? read_fifo(sum, sensor) : 0;
        Tick t = {.now = esp_timer_get_time()};
        LOCK();
        bool recording = log_active;
        t.stale = watch_link(t.now);
        // A failed read, or no new sample (the MPU updates at 1 kHz, so every control tick sees one unless the sensor stalled), ends the tick here; enough failed or stalled ticks in a row while disarmed set the IMU up again.
        bool failed = result != ESP_OK;
        if (failed || !(data[0] & 1)) {
            bool stalled = failed || !state.sample_us || t.now - state.sample_us > MAX_SAMPLE_MS * 1000ULL;
            if (failed) {
                state.online = false;
                state.calibration_left = 0;
                if (state.enabled) stop_locked(STOP_IMU);
                else if (!state.reason) {
                    state.reason = STOP_IMU;
                    idle_failure = true;
                }
                previous = 0;
            } else if (state.sample_us && stalled) {
                state.online = false;
                if (state.enabled) stop_locked(STOP_LATE);
            }
            bool retry = stalled && ++bad_ticks >= SENSOR_RETRY_TICKS && !state.enabled;
            UNLOCK();
            if (retry) {
                reconfigure_sensor();
                bad_ticks = 0;
                deadline = xTaskGetTickCount();
            }
            vTaskDelayUntil(&deadline, period);
            continue;
        }
        bad_ticks = 0;
        if (idle_failure) {
            if (state.reason == STOP_IMU) state.reason = STOP_NONE;
            idle_failure = false;
        }
        float body[7];
        imu_sample(sensor, sum, samples, raw, body);
        t.dt = previous ? (t.now - previous) / 1000000.0f : 1.0f / SAMPLE_HZ;
        if (voltage_mv) {
            float volts = voltage_mv / 1000.0f;
            battery_v = battery_v ? battery_v + (volts - battery_v) * (1 - expf(-t.dt / VOLTAGE_TAU)) : volts;
        }
        if ((battery_v > NO_PACK_V) != servo_pack) {
            servo_pack = battery_v > NO_PACK_V;
            write_servo(0);
            write_servo(1);
        }
        if (state.enabled && previous && t.now - previous > MAX_SAMPLE_MS * 1000ULL) stop_locked(STOP_LATE);
        take_imu(body, t.now);
        state.temperature = raw[3] / 340.0f + 36.53f;
        uint64_t started = estimate(&t, !previous);
        previous = t.now;
        if (state.enabled) tip_or_recover(&t);
        // Measured specific force along vertical, in g: the accelerometer onto the IMU attitude's up axis.
        t.vertical = -state.accel[0] * sinf(state.attitude.pitch) +
                     cosf(state.attitude.pitch) * (state.accel[1] * sinf(state.attitude.roll) + state.accel[2] * cosf(state.attitude.roll));
        if (state.enabled) fly(&t);
        if (servo_test.active) servo_test_tick(t.now);
        if (!state.enabled || !hover_mode) {
            flown = command;
            hover_rest(&hover, t.vertical, spinning(), t.dt, t.have_baro ? &t.baro : NULL);
            guard.failsafe = FAILSAFE_OFF;
        }
        uint32_t duration = esp_timer_get_time() - started;
        if (recording) capture(t.now, raw, duration);
        if (log_ends && t.now >= log_ends) {
            log_elapsed = t.now - log_started;
            log_active = false;
            log_ends = 0;
        }
        UNLOCK();
        vTaskDelayUntil(&deadline, period);
    }
}

const char *flight_runtime_esc_calibration(int stage) {
    if (stage < 0 || stage > 2) return "ESC calibration stage must be 0, 1 or 2";
    LOCK();
    const char *error = state.enabled || spinning() ? "Disarm first" : NULL;
    if (!error) {
        esc_calibration = stage;
        esc_calibration_us = esp_timer_get_time();
        for (int side = 0; side < 2; side++) {
            state.throttle[side] = 0;
            pulse(side * 2 + 1, esc_us(0));
        }
    }
    UNLOCK();
    return error;
}

// The update service writes flash only while this holds: flash writes stall this core's code fetches.
bool flight_runtime_idle(void) {
    LOCK();
    bool idle = quiet() && !command.enabled;
    UNLOCK();
    return idle;
}

// Whether the hover holds on the camera's tag sightings (hover_camera): while it does, nothing may switch the camera out of tag mode.
bool flight_runtime_camera_hold(void) {
    LOCK();
    bool camera = command.hover_camera != 0;
    UNLOCK();
    return camera;
}

// A target packet from the laptop is the controller heartbeat; the 500 ms watchdog stops the motors when they cease.
bool flight_runtime_targets(const float *targets) {
    for (int i = 0; i < 12; i++) {
        if (!isfinite(targets[i])) return false;
    }
    // The hold-point nudge (m), at most NUDGE_MAX each way.
    if (fabsf(targets[9]) > NUDGE_MAX || fabsf(targets[10]) > NUDGE_MAX || fabsf(targets[11]) > NUDGE_MAX) return false;
    if (targets[0] < 0 || fabsf(targets[1]) > TARGET_TILT || fabsf(targets[2]) > TARGET_TILT ||
        fabsf(targets[3]) > TARGET_RATE || targets[6] < 0 || targets[7] < 0 ||
        !(targets[8] == HOVER_OFF || targets[8] == HOVER_HOLD || targets[8] == HOVER_LAND || targets[8] == HOVER_LAND_FAST))
        return false;
    LOCK();
    bool active = running && targets[0] <= THROTTLE_MAX && targets[6] <= THROTTLE_MAX && targets[7] <= THROTTLE_MAX &&
                  fabsf(targets[4]) <= SERVO_LIMIT && fabsf(targets[5]) <= SERVO_LIMIT;
    if (active) {
        state.heartbeat_us = esp_timer_get_time();
        command.base = targets[0];
        command.target_roll = targets[1] / DEG;
        command.target_pitch = targets[2] / DEG;
        command.target_yaw = targets[3] / DEG;
        requested_mode = targets[8];
        if (held && requested_mode != held_from) held = false;
        // Flying in the hold or a landing, a switch to manual with the collective under liftoff lands instead of stopping the motors in the air: on 2026-11-14 at 0.48 m the stream that came back after a controller restart carried mode 0 and zero collective (the dashboard had dropped to manual), and the drone fell and tumbled. Stopping on purpose is the stop request (Space), and manual flight is still taken with the collective raised; the landing ends as any does, or by asking for the hold again.
        int mode = requested_mode;
        bool flying = state.enabled && state.controller.airborne > 0 && !hover.landed && hover.bias > 0.5f;
        if (flying && hover_mode != HOVER_OFF && mode == HOVER_OFF && command.base < command.integrate_throttle) mode = HOVER_LAND;
        if (!link_landing && !held) hover_mode = mode;
        nudge[0] = targets[9];
        nudge[1] = targets[10];
        nudge[2] = targets[11];
        if (!targets[6] && !targets[7]) state.latched = false;
        // During maintenance the disarmed outputs stay where they are: zero throttle, servos still.
        if (!state.enabled && !maintenance) {
            for (int side = 0; side < 2; side++) {
                if (!targets[4 + side]) state.servo_held[side] = false;
                // A servo test owns the servos; streamed throttle aborts it (the control loop checks).
                if (!state.servo_held[side] && !servo_test.active) state.servo[side] = targets[4 + side];
                write_servo(side);
                state.throttle[side] = state.latched ? 0 : lroundf(targets[6 + side]);
                pulse(side * 2 + 1, esc_us(state.throttle[side]));
            }
        }
    }
    UNLOCK();
    return active;
}

uint32_t flight_runtime_stop_generation(void) {
    LOCK();
    uint32_t generation = stop_generation;
    UNLOCK();
    return generation;
}

// A new recording session from an empty log; with no recording open there is no stop's post-roll pending (log_ends 0).
static void start_log(void) {
    log_head = log_tail = log_samples = log_dropped = log_elapsed = 0;
    log_session = esp_random() | 1;
    log_started = esp_timer_get_time();
    log_active = true;
    log_ends = 0;
}

// The only way to arm: an explicit request after any stop, with the streamed collective at zero. Arming starts a recording, so it also needs the laptop's recorder and the previous recording fully sent. Every check, including the stop count from when the request arrived, runs under the lock that stops take, so a stop at any point after the request arrived refuses it.
const char *flight_runtime_arm(bool recorder, bool calibration_unsaved, uint32_t expected_generation,
                               uint32_t arm_sequence) {
    LOCK();
    ArmCheck check = {
        .running = running, .online = state.online, .calibrated = state.calibrated,
        .calibrating = state.calibration_left > 0, .enabled = state.enabled,
        .throttle = spinning(), .base = command.base != 0, .recorder = recorder,
        .log_pending = log_head != log_tail, .calibration_ready = state.calibration_ready,
        .calibration_unsaved = calibration_unsaved, .maintenance = maintenance,
        .expected_generation = expected_generation, .generation = stop_generation,
        .arm_sequence = arm_sequence, .barrier = stop_barrier,
    };
    const char *error = arm_refusal(&check);
    // Hover takes off by itself, so it is switched on only after arming.
    if (!error && hover_mode) error = "Hover mode must be off to arm";
    if (!error && esc_calibration) error = "Finish the ESC calibration first";
    if (!error && battery_v > NO_PACK_V && battery_v < ARM_MIN_V) error = "Battery under 14.0 V: charge or swap it";
    if (!error) {
        command.enabled = 1;
        state.enabled = true;
        state.reason = STOP_NONE;
        flight_reset(&state.controller, &command);
        rewind_pattern();
        flip_reset(&flip);
        recover_reset(&recovery);
        held = false;
        state.heartbeat_us = esp_timer_get_time();
        start_log();
    }
    UNLOCK();
    return error;
}

const char *flight_runtime_record(bool on, bool recorder) {
    const char *error = NULL;
    LOCK();
    if (state.enabled) {
        error = "Armed: the flight is already recording";
    } else if (!on) {
        if (log_active) log_elapsed = esp_timer_get_time() - log_started;
        log_active = false;
        log_ends = 0;
    } else if (log_active) {
        // Already recording: a repeated request changes nothing.
    } else if (!recorder) {
        error = "Recorder not subscribed or previous recording still open";
    } else if (log_head != log_tail) {
        error = "Previous recording not yet sent";
    } else {
        state.reason = STOP_LAPTOP;
        start_log();
    }
    UNLOCK();
    return error;
}

void flight_runtime_log(LogState *log) {
    LOCK();
    *log = (LogState){
        .session = log_session, .started_us = log_started, .samples = log_samples, .dropped = log_dropped,
        .elapsed_us = log_active ? esp_timer_get_time() - log_started : log_elapsed,
        .pending = log_head - log_tail, .active = log_active, .reason = state.reason, .calibrated = state.calibrated,
        .bias = {state.bias[0], state.bias[1], state.bias[2]},
        .pose_zero = {state.pose_zero.roll * DEG, state.pose_zero.pitch * DEG},
    };
    UNLOCK();
}

// Copies up to count of the oldest records after the first skip without removing them; the radio consumes them once acknowledged. The radio reads a backlog one batch at a time, so the lock is never held for more than a batch's copy while the control task waits on it.
size_t flight_runtime_read_log(uint8_t *output, size_t skip, size_t count) {
    LOCK();
    size_t pending = log_head - log_tail;
    if (skip > pending) skip = pending;
    if (count > pending - skip) count = pending - skip;
    for (size_t i = 0; i < count; i++) memcpy(output + i * sizeof(Record), &records[(log_tail + skip + i) % LOG_CAPACITY], sizeof(Record));
    UNLOCK();
    return count;
}

void flight_runtime_consume_log(size_t count) {
    LOCK();
    log_tail += count;
    UNLOCK();
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
    LOCK();
    settings_apply(&command, values);
    accel_correction(values);
    state.pose_zero = (Attitude){values[SETTING_pose_zero_roll] / DEG, values[SETTING_pose_zero_pitch] / DEG};
    UNLOCK();
}

// Stops the motors and averages two seconds of gyro for the bias, stored once it completes; gravity only re-seeds the attitude estimate.
const char *flight_runtime_calibrate(void) {
    LOCK();
    bool active = running;
    if (active) {
        stop_locked(STOP_CALIBRATION);
        state.calibrated = state.calibration_ready = false;
        memset(state.calibration_sum, 0, sizeof(state.calibration_sum));
        memset(state.gravity_sum, 0, sizeof(state.gravity_sum));
        state.calibration_left = state.calibration_count = 2 * SAMPLE_HZ;
    }
    UNLOCK();
    return active ? NULL : "IMU offline";
}

// Hands over the self-trim a stop left, once, while disarmed.
bool flight_runtime_take_trim(float *roll_bias, float *pitch_bias) {
    LOCK();
    bool taken = trim.ready && !state.enabled;
    if (taken) {
        trim.ready = false;
        *roll_bias = trim.bias[0];
        *pitch_bias = trim.bias[1];
    }
    UNLOCK();
    return taken;
}

bool flight_runtime_take_calibration(float *bias, float *pose_zero, float *sensor_frame) {
    LOCK();
    bool ready = state.calibration_ready;
    state.calibration_ready = false;
    memcpy(bias, state.bias, sizeof(state.bias));
    pose_zero[0] = state.pose_zero.roll * DEG;
    pose_zero[1] = state.pose_zero.pitch * DEG;
    *sensor_frame = frame();
    UNLOCK();
    return ready;
}

// Keeps the current pulse and makes the servo's commanded angle its new zero, only if the settings table can hold the new offset; otherwise nothing changes. error is scratch space for the message.
const char *flight_runtime_zero_servo(int side, float *offset, char *error, size_t size) {
    int index = side ? SETTING_servo_offset_right : SETTING_servo_offset_left;
    LOCK();
    bool busy = !quiet();
    *offset = state.offsets[side] + state.servo[side];
    bool fits = settings_valid(index, *offset);
    if (!busy && fits) {
        state.offsets[side] = *offset;
        state.servo[side] = 0;
        state.servo_held[side] = true;
    }
    UNLOCK();
    // Formatting allocates, so the message is built outside the lock.
    if (busy) return "Stop the motors before zeroing a servo";
    return fits ? NULL : settings_validate(index, *offset, error, size);
}

// Holds the outputs at zero for a flash write: succeeds only while disarmed with zero throttle.
bool flight_runtime_begin_maintenance(void) {
    LOCK();
    bool ok = quiet();
    if (ok) maintenance = true;
    UNLOCK();
    return ok;
}

// Manual throttle stays latched at zero until the laptop streams zero throttle again.
void flight_runtime_end_maintenance(void) {
    LOCK();
    maintenance = false;
    state.latched = true;
    UNLOCK();
}

// A laptop STOP: stops at once, whatever its sequence, and raises the barrier arm packets must be newer than.
void flight_runtime_command_stop(uint32_t sequence) {
    LOCK();
    stop_locked(STOP_LAPTOP);
    stop_barrier_update(&stop_barrier, sequence);
    UNLOCK();
}

// The laptop's flip request: starts the flip (flip.h) at once if flip_refusal finds nothing against it, checked under the lock against this tick's state.
const char *flight_runtime_flip(void) {
    LOCK();
    int64_t now = esp_timer_get_time();
    Attitude pose = relative_attitude(&state);
    FlipCheck check = {
        .enabled = command.flip_enable >= 1, .armed = state.enabled, .done = flip.done,
        .holding = hover_mode == HOVER_HOLD && hover.engaged, .camera = command.hover_camera != 0, .gps = command.hover.gps_tau > 0,
        .aligned = hover.aligned, .located = hover.gps_us && now - hover.gps_us <= FLIP_FIX_US, .ekf = command.hover.ekf > 0,
        .landing = link_landing, .failsafe = guard.failsafe != FAILSAFE_OFF, .pattern = excite.active,
        .airborne = state.controller.airborne, .height = hover.height, .climb_rate = hover.climb_rate, .speed = hypotf(hover.gps_fix[0], hover.gps_fix[1]),
        .roll = pose.roll, .pitch = pose.pitch - command.pitch_trim, .rate = {state.gyro[0], state.gyro[1], state.gyro[2]},
        .min_height = command.flip_height, .battery = battery_v,
    };
    const char *error = recover_gyro_only(&recovery) ? "Recovering" : flip_refusal(&check);
    if (!error) flip_begin(&flip, &command, &state.controller, hover.integral, hover.force, state.attitude.roll, state.attitude.pitch, hover.climb_rate);
    UNLOCK();
    return error;
}

// The laptop's level-and-hold (radio.c h): from any mode, ends a test pattern or a flip, levels (righting first if far over, recover.h) and holds; needs nothing more from the laptop once taken. Refused disarmed or on the ground.
const char *flight_runtime_level(void) {
    LOCK();
    const char *error = !state.enabled ? "Not armed" : state.controller.airborne <= 0 || hover.landed ? "On the ground" : NULL;
    if (!error) {
        // A flip still climbing just ends (its height integral given back); one past that the recovery takes over at its next tick.
        flip_abort(&flip, &hover.integral);
        recover_abort(&recovery);
    }
    UNLOCK();
    return error;
}

void flight_runtime_status(Status *status) {
    LOCK();
    State copy = state;
    int guarded = guard_bits();
    // A recovery's stage and cause in the reason's top bits (bits 6-7 the stage, 4-5 the cause less one), as runtime.h Status has it.
    int recovering = recover_active(&recovery) ? recovery.stage << 6 | (recovery.cause - 1) << 4 : 0;
    float target[] = {command.target_roll * DEG, command.target_pitch * DEG, command.target_yaw * DEG, command.base};
    bool recording = log_active;
    uint32_t samples = log_samples, dropped = log_dropped;
    UNLOCK();
    Attitude pose = relative_attitude(&copy);
    bool fresh = copy.online && esp_timer_get_time() - copy.sample_us < 100000;
    status->flags = fresh | (copy.calibration_left > 0 || copy.calibration_ready) << 1 | copy.calibrated << 2 | copy.enabled << 3 |
                    recording << 4 | copy.latched << 6 | guarded << 8;
    status->reason = copy.reason | recovering;
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

// Starts the servo bench test: cycles × (left then right) × 4 steps of amp degrees. Refused unless disarmed with zero throttle.
const char *flight_runtime_servo_test(float amp, int cycles) {
    if (!(amp > 0 && amp <= 15) || cycles < 1 || cycles > 50) return "Expected amp 0-15 degrees and 1-50 cycles";
    LOCK();
    bool idle = running && quiet() && !maintenance;
    if (idle) {
        memset(&servo_test, 0, sizeof(servo_test));
        servo_test.amp = amp;
        servo_test.count = cycles * 8;
        servo_test.step_us = esp_timer_get_time();
        servo_test.active = true;
    }
    UNLOCK();
    return idle ? NULL : "Disarm and zero throttle first";
}

// A test pattern (see excite): float32 after (s), uint8 repeat (0 = until disarmed), uint16 mask of the channels given, uint16 segment count, then per segment uint16 ticks (control samples) and an int16 value (hundredths of the channel's unit) for each channel in mask, all little-endian. A count of 0 clears it. Only while disarmed.
const char *flight_runtime_pattern(const uint8_t *body, size_t length, char *text, size_t size) {
    if (length < 9) return "Expected after, repeat, mask and count";
    float after;
    uint16_t count, mask;
    memcpy(&after, body, 4);
    int repeat = body[4];
    memcpy(&mask, body + 5, 2);
    memcpy(&count, body + 7, 2);
    if (mask >> EXCITE_CHANNELS) return "Unknown channel in mask";
    int channels = __builtin_popcount(mask);
    if (!(after >= 0 && after <= 60)) return "after must be 0-60 s";
    if (count > EXCITE_SEGMENTS) return "Too many segments";
    if (length != 9 + (size_t)count * (2 + 2 * channels)) return "Length does not match mask and count";
    static ExciteSegment staged[EXCITE_SEGMENTS];
    memset(staged, 0, sizeof(staged));
    const uint8_t *p = body + 9;
    float seconds = 0;
    for (int s = 0; s < count; s++) {
        memcpy(&staged[s].ticks, p, 2);
        p += 2;
        if (!staged[s].ticks) return "Segments need at least one tick";
        seconds += staged[s].ticks / (float)SAMPLE_HZ;
        for (int i = 0; i < EXCITE_CHANNELS; i++) {
            if (!(mask >> i & 1)) continue;
            memcpy(&staged[s].value[i], p, 2);
            p += 2;
            if (fabsf(staged[s].value[i] / 100.0f) > excite_limit[i]) return "A value is past its channel's limit";
        }
    }
    LOCK();
    bool idle = running && quiet() && !maintenance;
    if (idle) {
        memcpy(excite.segment, staged, sizeof(staged));
        excite.count = count;
        excite.repeat = repeat;
        excite.after = after;
        rewind_pattern();
    }
    UNLOCK();
    if (!idle) return "Disarm and zero throttle first";
    int n = snprintf(text, size, "{\"segments\":%d,\"seconds\":%.2f,\"repeat\":%d,\"after\":%.2f}", count, seconds, repeat, after);
    return n > 0 && (size_t)n < size ? NULL : "Reply does not fit";
}

// The test's progress and, per side, the mean peak pitch and yaw rate deviation after a step (deg/s).
size_t flight_runtime_servo_result(char *text, size_t size) {
    LOCK();
    typeof(servo_test) copy = servo_test;
    UNLOCK();
    float mean[2][2];
    for (int side = 0; side < 2; side++)
        for (int axis = 0; axis < 2; axis++) mean[side][axis] = copy.steps[side] ? copy.peak[side][axis] / copy.steps[side] * DEG : 0;
    int n = snprintf(text, size, "{\"active\":%d,\"done\":%d,\"of\":%d,\"amp\":%.1f,\"left\":{\"steps\":%d,\"pitch_rate\":%.2f,\"yaw_rate\":%.2f},\"right\":{\"steps\":%d,\"pitch_rate\":%.2f,\"yaw_rate\":%.2f}}",
                     copy.active, copy.index, copy.count, copy.amp, copy.steps[0], mean[0][0], mean[0][1], copy.steps[1], mean[1][0], mean[1][1]);
    return n > 0 && (size_t)n < size ? (size_t)n : 0;
}

void flight_runtime_set_voltage(uint16_t millivolts) {
    LOCK();
    voltage_mv = millivolts;
    UNLOCK();
}

void flight_runtime_outputs(void) {
    lock = xSemaphoreCreateMutexStatic(&lock_buffer);
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
        float us = esc ? ESC_STOP_US : 1500 + state.offsets[side] * 1000 / 90;
        ledc_channel_config_t channel = {.gpio_num = pins[index], .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = index,
                                         .timer_sel = esc, .duty = lroundf(us * ticks[index])};
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }
}

// Wakes the MPU6050 with 1 kHz output into its FIFO (accelerometer and gyro), which each control tick averages (read_fifo); the DLPF only has to clear what the 10 ms mean does not. With reset, the device is reset to its defaults first.
static bool configure_sensor(bool reset) {
    i2c_config_t bus = {.mode = I2C_MODE_MASTER, .sda_io_num = SDA, .scl_io_num = SCL,
                        .sda_pullup_en = GPIO_PULLUP_ENABLE, .scl_pullup_en = GPIO_PULLUP_ENABLE,
                        .master.clk_speed = 400000};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &bus));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    if (reset) {
        write_mpu(0x6b, 0x80);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    bool ok = write_mpu(0x6b, 1) == ESP_OK;
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t registers[][2] = {
        {0x19, 0}, {0x1a, IMU_DLPF}, {0x1b, GYRO_FS_SEL << 3}, {0x1c, 0}, {0x37, 0x20}, {0x38, 1}, {0x23, 0x78}, {0x6a, 0x44},
    };
    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
        if (write_mpu(registers[i][0], registers[i][1]) != ESP_OK) ok = false;
    }
    return ok;
}

// After SENSOR_RETRY_TICKS bad ticks: a fresh driver install, a device reset and the configuration, with what the sensor answered on the serial console (WHO_AM_I 0x68 for an MPU6050; PWR_MGMT_1 0x01 once awake; INT_STATUS's bit 0 set when a sample is ready), so a sensor that reads but never samples is told from one that does not answer.
static void reconfigure_sensor(void) {
    i2c_driver_delete(I2C_NUM_0);
    sensor_configured = configure_sensor(true);
    uint8_t reg = 0x75, who = 0, power = 0, ready = 0;
    esp_err_t answered = i2c_master_write_read_device(I2C_NUM_0, MPU, &reg, 1, &who, 1, pdMS_TO_TICKS(20));
    reg = 0x6b;
    i2c_master_write_read_device(I2C_NUM_0, MPU, &reg, 1, &power, 1, pdMS_TO_TICKS(20));
    vTaskDelay(pdMS_TO_TICKS(10));
    reg = 0x3a;
    i2c_master_write_read_device(I2C_NUM_0, MPU, &reg, 1, &ready, 1, pdMS_TO_TICKS(20));
    printf("IMU set up again: configured %d, %s, WHO_AM_I 0x%02x, PWR_MGMT_1 0x%02x, INT_STATUS 0x%02x\n", sensor_configured, esp_err_to_name(answered), who, power, ready);
}

void flight_runtime_start(void) {
    // Accelerometer at ±2 g (16384 per g), gyro at runtime.h's full scale (GYRO_COUNTS_PER_DEG_S), both mapped to body axes.
    for (int i = 0; i < 3; i++) {
        config.accel_scale[i] = sensor_to_body[i] / 16384.0f;
        config.gyro_scale[i] = sensor_to_body[i] * 3.14159265358979323846 / (180 * GYRO_COUNTS_PER_DEG_S);
    }
    // Equivalent to retaining 0.99 of gyro integration every 10 ms; with the motors spinning, settings.def attitude_tau.
    config.gravity_tau = -0.01 / log(0.99);
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
    sensor_configured = configure_sensor(false);
    if (!sensor_configured) {
        ESP_ERROR_CHECK(i2c_driver_delete(I2C_NUM_0));
        sensor_configured = configure_sensor(false);
    }
    LOCK();
    stop_locked(STOP_NONE);
    for (int side = 0; side < 2; side++) write_servo(side);
    running = true;
    UNLOCK();
    if (xTaskCreatePinnedToCore(runtime_task, "flight", 8192, NULL, 5, NULL, 1) != pdPASS) abort();
}
