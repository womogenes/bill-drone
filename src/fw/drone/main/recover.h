#pragma once

#include <stdbool.h>

#include "flight_math.h"
#include "flip.h"

// Tumble recovery and the pilot's level-and-hold (recover.c, fw/FLIP.md): instead of the tip cut stopping the motors in the air, right the thrust axis with its own gains and the motors flown directly (flip_allocate), the attitude estimate on the gyro alone, then level under the attitude controller while the collective stops the vertical speed, then hand back to the hover's hold. Righting needs more than the attitude controller's gains and its thrust: a drone far over has its pitch authority only from tilted thrust.
// The righting's gains: the one attitude law (attitude_rates) toward upright at the drone's own heading, the rate toward up at most max_rate, sqrt(2 brake angle) and kp angle; k_roll and k_pitch (1/s) turn rate errors into angular accelerations, kd_yaw damps yaw; servo_max bounds the common tilt (radians). The collective holds the height along up (kv per s against the vertical speed) times floor when inverted to all of it upright, and each motor keeps at least idle of the weight, the flip's FLIP_FLOOR: a motor commanded to throttle 1 mid-spin on 2026-11-12 took 0.1 s or more to come back (an ESC may stop at 1001 us). flight_rt.c slews and caps the throttles as the flip's (flip_limit) while a recovery and its grace run. Righting ends within right_angle of upright and turning under right_rate (rad/s) on roll and pitch; it fails after right_time (s). Leveling ends once within level_angle, under level_rate and under ARREST_SPEED of vertical speed for SETTLE_S, or after level_time (s).
typedef struct {
    float kp, brake, max_rate, k_roll, k_pitch, kd_yaw, servo_max;
    float kv, floor, idle;
    float right_angle, right_rate, right_time;
    float level_angle, level_rate, level_time;
} RecoverParams;

extern const RecoverParams recover_defaults;

// The stages, as the flight record's guard bits 8-9 and the status reason's bits 6-7 hold them (fw/controller/log_format.py RECOVERY_STAGES): righting owns the motors; leveling flies the attitude controller at level with the collective arresting the vertical speed.
enum { RECOVER_IDLE, RECOVER_RIGHT, RECOVER_LEVEL };
// What started it (guard bits 10-12, log_format RECOVERY_CAUSES): past tip_limit outside a flip, a flip's arrest or grace past tip_limit, the gyro at its full scale, the pilot's level-and-hold.
enum { RECOVER_NONE, RECOVER_TILT, RECOVER_FLIP, RECOVER_SATURATED, RECOVER_ABORT };
// Stop reasons (flight_rt.c state.reason, log_format STOP_REASONS): the tip cut, and a righting that did not come upright within right_time.
#define RECOVER_TIP_REASON 4
#define RECOVER_FAILED_REASON 10

// The barometer's readings the recovery keeps for its rate: more than half a second of them (recover.c BARO_RATE_S) at up to 100 readings a second (baro.c measures about every 20 ms).
#define RECOVER_BARO_READINGS 64

typedef struct {
    int stage, cause, recoveries;
    // How long this recovery has run and righted (s), the thrust the motors have by now (N), the vertical speed it flies on (m/s; from the hold's climb rate at the start, then the specific force along q's up while measured, otherwise the barometer's rate or none), how long it has stayed settled (s), the time since it ended (s; negative before one), and whether the pilot asked for level-and-hold.
    float time, righting, thrust, vertical, settled, since;
    bool abort;
    // The attitude controller and the hover's height integral as the last ordinary tick left them (neither in a flip nor recovering), given back at the hand-back and the end, so neither carries what the tumble wound into them.
    Controller saved;
    float height_integral;
    // Whether vertical is still the accelerometer's integral (recover.c measured), how long since the accelerometer or the gyro last read at their full scale (s; INFINITY never this arming), and this run's clock (s).
    bool measured;
    float clipped, clock;
    // The barometer's newest readings (RECOVER_BARO_READINGS of them, each its height above the hold's ground in m and its arrival on clock), and from those within BARO_RATE_S of the newest the line fitted through them: its rate (m/s, NaN until readings cover the window) and its height now (m).
    float baro_height[RECOVER_BARO_READINGS], baro_at[RECOVER_BARO_READINGS];
    int baro_next, baro_count;
    float baro_rate, baro_fit;
} RecoverRun;

// One tick's inputs. enabled: the setting recover_enable. tipped: past tip_limit outside the flip's exemption, as the tip cut has always tested it. allowed: airborne, not landing near the ground or touched down, and fewer than RECOVER_MAX recoveries this arming (flight_rt.c), so a tumble may be recovered rather than cut. Rates in rad/s, the specific force in g (body forward, left, up), angles in radians.
typedef struct {
    bool enabled, tipped, allowed;
    const float *gyro, *accel;
    float dt, tip_limit;
    // The burst (flip.h): running, its phase (flip_phase: it keeps the nose level, so past BURST_LOST_PITCH of nose up or down while it turns it is lost, and the recovery takes over then) and whether the attitude estimate is on the gyro alone for it (flip_gyro_only).
    bool flip_active, flip_gyro_only;
    int flip_phase;
    // The hold's climb rate (m/s), the collective being flown (N) and the airframe's authority (flip.h; weight in N).
    float climb_rate, collective;
    FlipAuthority authority;
    // The barometer: its newest reading's height above the ground the hold learned with the motors off (m; raw, not the hold's fused height; NaN without a fresh reading or a ground), whether that reading is new this tick, and how old its pressure is when it arrives (s; flight_rt.c BARO_LATENCY_US). The hold's ceiling (hover_ceiling, m) and its barometer time constant (hover_baro_tau, s; 0 off).
    float baro;
    bool baro_new;
    float baro_latency, ceiling, hold_tau;
} RecoverInput;

typedef struct {
    // cut: the tip cut applies, with this stop reason. cancel_flip: a recovery took over a running flip. owned: righting flies motors (thrusts, tilts) itself; level: the attitude controller flies level with motors.collective (N) as the collective.
    int cut;
    bool cancel_flip, owned, level;
    FlipOutput motors;
    // The most collective (N) the barometer allows: INFINITY unless its raw reading is more than RECOVER_CEILING_MARGIN over the ceiling (recover.c ceiling_cap). With recover_enable flight_rt.c caps every collective it flies with it, the hold's included; the recovery's own already are.
    float cap;
    // reseed: a recovery whose vertical speed was no longer the accelerometer's ended, and the hold, whose height and climb rate integrate the same accelerometer, takes height (m) and climb_rate (m/s) from the barometer's line instead.
    bool reseed;
    float height, climb_rate;
} RecoverOutput;

// A new arming: nothing recovered yet.
void recover_reset(RecoverRun *run);
// The pilot's level-and-hold: the next tick starts a recovery whatever the setting, ending a flip and leveling.
void recover_abort(RecoverRun *run);
bool recover_active(const RecoverRun *run);
// The attitude estimate on the gyro alone: while it runs and RECOVER_GRACE after.
bool recover_gyro_only(const RecoverRun *run);
// One control tick, where the tip cut has always been tested (after the attitude estimate and the burst's tick), on the body attitude q (w, x, y, z): starts a recovery on a trigger it is allowed, runs one, and says when the tip cut applies instead. At the hand-back to leveling the controller gets back its integrators and dead-reckoned velocity, and at its end the hover gets back its height integral.
void recover_tick(RecoverRun *run, const RecoverParams *p, const RecoverInput *in, const float *q, Controller *controller, float *height_integral, RecoverOutput *out);
// After a recovery ended, the estimate stays on the gyro this long (s): the accelerometer still reads the arrest's thrust.
#define RECOVER_GRACE 1.0f
// Recoveries per arming before the tip cut applies again: one that keeps tipping back over is not recovering.
#define RECOVER_MAX 3
// No recovery until the attitude controller has been airborne this long (s): a takeoff that tips over still has its feet on the ground, where driving the motors would only roll it along the floor.
#define RECOVER_AIRBORNE_S 1.0f
// The user's rule for drone 1's height (2026-11-15): it holds at hover_ceiling (5 m at most) and may stray this far above it (m) briefly; past that, by the barometer's raw reading, no collective may climb (recover.c ceiling_cap).
#define RECOVER_CEILING_MARGIN 1.0f
// A landing recovers only above this height by the hold's estimate (m): below it the drone is at or near the ground, and the barometer may read 0.9 m high at contact (hover.c LAND_FINAL_HEIGHT's note).
#define RECOVER_LAND_HEIGHT 1.0f
