#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "flight_math.h"

// The burst (flip.c, fw/FLIP.md): from a settled GPS hold, one reference rotation about the drone's forward axis at its heading, out to flip_angle and back, tracked by the one attitude law (attitude_error, attitude_rates) with the reference's rate and acceleration fed forward, the motors and servos flown by flip_allocate; then, within right_angle of upright and turning under handoff_rate (or after timeout), the attitude controller takes it back while the collective arrests the vertical speed.
// Its parameters (sw/sim/tune_flip.py, CMA-ES on burst60 and burst90 on the 2026-11-14 ESCs' plant and the 2026-11-15 bursts' reproducing plant; sw/sim/models/NOTES.md "Quaternion attitude"): thrusts as fractions of the weight, rates in rad/s, angles in radians.
typedef struct {
    // The hand-back: within right_angle of upright with every body rate under handoff_rate, or timeout (s) after the start regardless. Then the collective stops the vertical speed (kv per s) until it has stayed under ARREST_DONE (flip.c) for ARREST_HOLD, at most arrest_time (s).
    float right_angle, handoff_rate, timeout, kv, arrest_time;
    // The motors while the burst owns them: each makes at least floor and at most ceiling of the weight, so neither idles (an idle prop coasts, its ESC may stop it, and its spin-up later is a current spike and a desync risk) nor sits at full throttle; servo_max bounds the common tilt. flip_limit: each throttle moves at most slew a second and stays under throttle_cap, from the start until FLIP_GRACE after the hand-back.
    float floor, ceiling, servo_max, slew, throttle_cap;
    // The rotors' spin momentum fed forward (FlipAllocation gyro_ff, yaw_ff).
    float gyro_ff, yaw_ff;
    // The reference: its mean angular acceleration (rad/s²), the top roll rate a whole turn would have (a burst's is this times the square root of its share of a turn: the rate it can brake from in its angle), how far ahead its acceleration is fed forward (s; the motors' and the servos' lag), and the tilt error (rad) at which it stops to wait for the drone (flip.c track).
    float ref_accel, ref_rate, ref_lead, track_slack;
    // The attitude law tracking it: track_kp (1/s) the tilt error into rate, no faster than it can brake from (sqrt(2 brake angle)) nor max_rate; track_heading (1/s) the heading's; track_roll, track_pitch and track_yaw (1/s) the rate errors into angular accelerations.
    float track_kp, brake, max_rate, track_heading, track_roll, track_pitch, track_yaw;
} FlipParams;

extern const FlipParams flip_defaults;
// The burst's and the recovery's authority (FlipAuthority, angular acceleration per newton of differential or tilted thrust), measured rather than taken from the settings' model (pivot_height 0.02 m, pitch_inertia 0.0023, which the attitude controller keeps): roll, the arm over the roll inertia on the raised pivots (0.172 / 0.01275, sw/sim/models/airframe_2026-11-12_raised.json); pitch and yaw as the 2026-11-15 bursts measured them (sw/sim/models/airframe_2026-11-15_bursts.json, NOTES.md "The bursts' plant"): the tilt pitches 16.8 per N per radian, 1.9 times the settings' model's 8.7, and the tilted thrust yaws 11.34, the slipstream leaving 74% of it at the arm.
#define FLIP_ROLL_AUTHORITY 13.5f
#define FLIP_PITCH_AUTHORITY 16.8f
#define FLIP_YAW_AUTHORITY 11.34f
// The least thrust each motor keeps while the burst or a recovery (recover.c) owns the motors, of the weight: recover_defaults' idle, and the floor below which sw/sim/tune_flip.py does not search flip_defaults' own.
#define FLIP_FLOOR 0.2f
// The height the burst never takes the drone over (m; the user's fixed cap: only the user changes it), whatever hover_ceiling says, and the margin it keeps under it.
#define FLIP_CEILING 5.0f
#define FLIP_ROOM_MARGIN 0.3f

enum { FLIP_IDLE, FLIP_TURN, FLIP_ARREST };

typedef struct {
    int stage;
    // q: the attitude (w, x, y, z; body to level), the estimate's own each tick; heading: its heading at the start, about which the reference turns; time since the start (s); turned: the furthest roll on the gyro since the start (radians), rolled its running sum; thrust: what the motors have by now (N); vertical: the vertical speed (m/s; the hold's climb rate at the start, then the specific force along up integrated); risen: the height since the start (m); arrested: how long the arrest has stayed under ARREST_DONE (s).
    float q[4], heading, time, turned, rolled, thrust, vertical, risen, arrested;
    // How far it rolls (radians; flip_angle) and how far the reference's clock has run (s); room: the height from the hold's at the start to the lower of hover_ceiling and FLIP_CEILING (m), past which, less FLIP_ROOM_MARGIN and what the vertical speed still carries it (flip.c capped), the collective may not lift.
    float turn, elapsed, room;
    // √T_right - √T_left as the motors have it by now (flip_allocate, for yaw_ff).
    float spread;
} Flip;

// The airframe's authority: angular acceleration (rad/s²) per newton of differential thrust (roll), per newton of collective per radian of common tilt (pitch), per newton per radian of differential tilt (yaw); and the weight (N).
// indi and unmodeled: per axis (roll, pitch, yaw) the share INDI takes (0 = off) of the unmodeled angular acceleration it found on this authority (indi.h; rad/s²), which the allocation takes off what it is asked; the rotors' gyroscopic feedforwards (gyro_ff in pitch, yaw_ff in yaw) give way by the same share, as the measured acceleration has them.
typedef struct {
    float roll, pitch, yaw, weight, indi[3], unmodeled[3];
} FlipAuthority;

// One tick's commands: while owned the burst flies the motors itself (thrusts in N, each assembly's physical forward tilt in radians); while arresting (collective_set) it only sets the collective (N) under the attitude controller. handed_back marks the tick it gives the attitude back.
typedef struct {
    bool owned, collective_set, handed_back;
    float left, right, left_tilt, right_tilt, collective;
} FlipOutput;

// The motors while the burst or the recovery owns them: servo_max bounds the common tilt (radians), each motor makes at least floor and at most ceiling of the weight.
// gyro_ff: the pitch the rotors' spin momentum gives a rolling drone with one rotor faster than the other (-ω × H: ω_roll H (√T_right - √T_left) over the pitch inertia), fed forward from the thrusts asked for, rad/s² per rad/s per √N; 0 (recover.c) leaves it out.
// yaw_ff: the yaw the rotors' spin momentum gives the body as their speeds part (-dH_z/dt) and as the pitch rate turns their tilted difference (ω_pitch H_x), fed forward from the thrusts asked for through the motors' lag, rad/s² per √N/s: the rotors' momentum per √N over the yaw inertia; 0 leaves it out.
typedef struct {
    float servo_max, floor, ceiling, gyro_ff, yaw_ff;
} FlipAllocation;

// Wanted angular accelerations (rad/s²) into thrusts and tilts: roll by differential thrust, pitch by common tilt, yaw by differential tilt, the collective (N) as asked where the motors' floor and ceiling leave room. thrust: what the motors have by now (N), followed here with their lag; spread: flip_allocate's yaw_ff state (NULL without it); pitch_rate (rad/s) for yaw_ff.
void flip_allocate(const FlipAllocation *r, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float pitch_rate, float *thrust, float *spread, float dt, FlipOutput *out);

// How flight_rt.c flies it (sw/sim/flip_firmware.py flies the same functions). It starts only on the laptop's flip request (radio.c f), once per arming, and only when flip_refusal finds nothing against it. Once it has begun nothing but a stop or a recovery ends it early: it hands back within timeout and arrests within arrest_time after, and the link-loss landing and a landing asked for wait until then. While it runs and for FLIP_GRACE after the hand-back the attitude estimate runs on the gyro alone and the tip cut is off (the recovery watches it instead, recover.h): the accelerometer still reads the arrest's thrust, and the attitude controller needs a moment to bring the last of the tilt in.
#define FLIP_GRACE 1.0f

// Everything the flip request depends on, read under the runtime lock in one go (flight_rt.c flight_runtime_flip). Angles from level (pitch beyond pitch_trim) in radians, rates in rad/s, heights in m, speeds in m/s, the battery in V.
typedef struct {
    // flip_angle set, armed, one already flown since arming.
    bool enabled, armed, done;
    // The GPS hold: the hover holding (mode HOVER_HOLD, engaged), not on the camera, the GPS hold on (hover_gps_tau), its heading pinned and a fix in the last GPS_STALE_US; no link-loss landing, fly-away failsafe or test pattern.
    bool holding, camera, gps, aligned, located, landing, failsafe, pattern;
    // The attitude controller's airborne time (flight_math.h Controller airborne), the hold's height above where it engaged and its climb rate, the GPS's ground speed, the attitude and body rates.
    float airborne, height, climb_rate, speed, roll, pitch, rate[3];
    // flip_height, and the smoothed battery voltage (0 before the first reading).
    float min_height, battery;
} FlipCheck;

// Why the flip is refused, or NULL.
const char *flip_refusal(const FlipCheck *check);

// The burst as flown: whether one has been flown since arming, the time since its hand-back (s; negative before one), the attitude controller and the hover's height integral as they were when it began (given back at the hand-back and at its end, so neither carries what the burst wound into them), its parameters (flip_reset: flip_defaults) and the throttles flip_limit last let out.
typedef struct {
    Flip flip;
    bool done;
    float since;
    Controller saved;
    float height_integral;
    const FlipParams *params;
    float throttle[2];
} FlipRun;

// A new arming: none yet.
void flip_reset(FlipRun *run);
// Starts it from the body attitude q (w, x, y, z), the controller as it is, the hover's height integral (hover.h HoverState integral), the collective now (N), the hold's climb rate (m/s) and height (m), turning angle (radians) under the lower of ceiling (m, hover_ceiling) and FLIP_CEILING.
void flip_begin(FlipRun *run, const Controller *controller, float height_integral, const float *q, float collective, float climb_rate, float height, float angle, float ceiling);
// The motors stopped, or a recovery took over: it ends at once.
void flip_cancel(FlipRun *run);
bool flip_active(const FlipRun *run);
// The attitude estimate on the gyro alone and the tip cut off: while it runs and FLIP_GRACE after its hand-back.
bool flip_gyro_only(const FlipRun *run);
// For the flight record and status (flight_rt.c, log_format.py FLIP_PHASES): 0 none, 2 turning (it owns the motors), 3 arresting (1, the whole flip's climb, is no longer flown).
int flip_phase(const FlipRun *run);
// One control tick, right after the attitude estimate: q the body attitude, gyro the body rates (rad/s), accel the specific force (g, body axes), a the authority with INDI's shares as this tick has them. At the hand-back the controller gets back its integrators and dead-reckoned velocity from before, and at its end the hover its height integral.
void flip_tick(FlipRun *run, const FlipAuthority *a, const float *q, const float *gyro, const float *accel, float dt, Controller *controller, float *height_integral, FlipOutput *out);
// Every control tick, on the throttles about to be sent (flight_rt.c, after the attitude controller and flip_actuators): from the burst's start until FLIP_GRACE after its hand-back, and while recovering holds (a recovery and its grace, recover_gyro_only), each moves at most params->slew a second from the last tick's and stays under params->throttle_cap, so neither motor jumps between idle and full throttle (the ESCs' current and voltage spikes); a stopped motor (0) stays stopped. Otherwise it only follows them.
void flip_limit(FlipRun *run, bool recovering, float *throttle, float dt);
// While the burst or a recovery owns the motors: its thrusts (N) as throttles through the thrust table and each motor's thrust scale (as flight_control's, battery compensation included), a running motor never stopped, and its tilts as servo angles (degrees, as flight_rt.c's state.servo, within servo_limit).
void flip_actuators(const FlipOutput *out, const Command *command, float left_scale, float right_scale, float throttle_max, float servo_limit, float *throttle, float *servo);
