#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "flight_math.h"

// The roll flip (flip.c), as sw/sim/flip.py flies it: punch up level, spin about roll on differential thrust, right the thrust axis as recover.py does, then hand back to the attitude controller and stop the vertical speed.
// The maneuver's parameters, from sw/sim/models/flip_firmware_params_2026-11-13_robust.json (sw/sim/tune_flip.py --robust on the raised-pivot airframe, for the worst rotor_momentum band over 0-0.05; before it flip_firmware_params_2026-11-12.json and flip_params_2026-11-12_bands.json): thrusts as fractions of the weight, rates in rad/s, angles in radians.
typedef struct {
    float climb_thrust, climb_time, spin_rate, k_spin, spin_thrust, handoff;
    // Righting (recover.py): the rate toward up is at most max_rate, sqrt(2 brake angle) and kp angle; k_roll and k_pitch (1/s) turn rate errors into angular accelerations, kd_yaw damps yaw; the collective is flip_thrust of the weight; servo_max bounds the common tilt; past roll_over from up with the error mostly in pitch it rolls the long way round.
    float kp, brake, max_rate, k_roll, k_pitch, kd_yaw, flip_thrust, right_angle, servo_max, roll_over;
    // After the hand-back the collective stops the vertical speed the flip left (kv per s, recover.py's arrest) until it has stayed under ARREST_DONE (flip.c), at most arrest_time (s).
    float kv, arrest_time;
    // Hand the attitude back regardless after this long (s).
    float timeout;
    // The motors while the flip owns them (2026-11-12, after the first flip): each makes at least floor and at most ceiling of the weight, so neither idles (an idle prop coasts, its ESC may stop it, and its spin-up later is a current spike and a desync risk) and neither sits at full throttle; the righting hands back only once every body rate is under handoff_rate (rad/s) as well as within right_angle of upright.
    float floor, ceiling, handoff_rate;
    // flip_limit: each throttle moves at most slew a second (throttle units: the full range in no less than 1200 / slew s), at most throttle_cap, from the flip's start until FLIP_GRACE after its hand-back.
    float slew, throttle_cap;
    // The spin's collective is spin_thrust plus spin_lift times the cosine of the turn so far, of the weight: more while the thrust points up, less while it points down.
    float spin_lift;
    // The spin's pitch and yaw rate gains (1/s; before 2026-11-13 k_spin and 0.3 k_spin), and the rotors' gyroscopic pitch fed forward in the spin and the righting (gyro_ff, rad/s² per rad/s of roll per √N of thrust difference: the rotors' momentum per √N over the pitch inertia, FlipRighting gyro_ff).
    float spin_pitch, spin_yaw, gyro_ff;
    // The rotors' spin momentum's yaw fed forward in the spin and the righting (FlipRighting yaw_ff). 2026-11-15: yaw_ff 1.6 (rotor_momentum 0.018 over the yaw inertia; APC's prop geometry gives about 0.014, the flight fits 0.006-0.03, and 1.6 did best on the worst of the bands below 0.03) with spin_yaw and kd_yaw four times the robust retune's (sw/sim/models/NOTES.md "Yaw in the flip").
    float yaw_ff;
    // The quaternion flip (settings.def quaternion; flip.c track), from sw/sim tuning on the 2026-11-14 ESCs' plant and the 2026-11-15 burst's reproducing plant (sw/sim/models/NOTES.md "Quaternion attitude"): the reference rotation's mean angular acceleration (rad/s²), and the one attitude law's gains tracking it: track_kp (1/s) the tilt error into rate (bounded by brake and max_rate as the righting's), track_heading (1/s) the heading's, and track_roll, track_pitch and track_yaw (1/s) the rate errors into angular accelerations.
    float ref_accel, track_kp, track_heading, track_roll, track_pitch, track_yaw;
    // How far ahead the reference's acceleration is fed forward (s): the motors' and the servos' lag; and the tilt error (rad) at which the reference stops to wait for the drone (flip.c track).
    float ref_lead, track_slack;
    // The reference's top roll rate for a whole turn (rad/s; a shorter turn's is this times the square root of its share, as spin_rate's).
    float ref_rate;
} FlipParams;

extern const FlipParams flip_defaults;
// The flip's own authority (FlipAuthority, angular acceleration per newton of tilted or differential thrust), from the airframe with the pivots raised 20 mm (sw/sim/models/airframe_2026-11-12_raised.json at geometry_2026-11-12_raised.json), not from the settings' model, which the attitude controller keeps and which predates the raise (pivot_height 0.02 m, pitch_inertia 0.0023: 8.7 per N per radian of pitch, 16.1 of yaw): roll, the arm over the roll inertia (0.172 / 0.01275); pitch, the tilt's effective lever above the CoM (the pivot 39.1 mm above it less the slipstream's 2 mm, 37.2 mm) over the pitch inertia (0.00525); yaw, the 30% of the tilted thrust the slipstream leaves at the arm over the yaw inertia (0.302 x 0.172 / 0.01115). The Euler flip's gains are tuned against these.
#define FLIP_ROLL_AUTHORITY 13.5f
#define FLIP_PITCH_AUTHORITY 7.08f
#define FLIP_YAW_AUTHORITY 4.66f
// With the quaternion setting, the flip's and the recovery's righting's (flight_rt.c) pitch and yaw authority as the 2026-11-15 bursts measured them (sw/sim/models/airframe_2026-11-15_bursts.json, sw/sim/models/NOTES.md "The bursts' plant"): the tilt's pitch 16.8, 2.4 times FLIP_PITCH_AUTHORITY and 1.9 times the settings' model's 8.7, which the recovery flew; the tilted thrust's yaw 11.3, the slipstream leaving 74% of it at the arm rather than the hover fit's 30%. Allocating against half the pitch authority there was, the 07:03 UTC burst60 asked twice the tilt it meant (25-30 degrees back while the motors were low), and when the roll's brake brought the right motor to the cap the nose went over.
#define FLIP_MEASURED_PITCH_AUTHORITY 16.8f
#define FLIP_MEASURED_YAW_AUTHORITY 11.34f
// The least thrust each motor keeps while the flip or a recovery (recover.c) owns the motors, of the weight: recover_defaults' idle, and the floor below which sw/sim/tune_flip.py does not search flip_defaults' own (0.2645 since the robust retune).
#define FLIP_FLOOR 0.2f
// The height the quaternion flip never takes the drone over (m; the user's fixed cap: only the user changes it), whatever hover_ceiling says, and the margin it keeps under it.
#define FLIP_CEILING 5.0f
#define FLIP_ROOM_MARGIN 0.3f
// The most room the quaternion flip's climb takes above where it starts (m), whatever the ceiling leaves: from a 3 m hold under the 5 m ceiling the 2026-11-15 07:03 UTC burst60 climbed to 4.85 m before it turned, and fell from about 4 m when it failed. With FLIP_ROOM_MARGIN it rises about half a meter. The quaternion bursts no longer climb at all (flip_step).
#define FLIP_CLIMB_ROOM 0.8f

enum { FLIP_IDLE, FLIP_CLIMB, FLIP_SPIN, FLIP_RIGHT, FLIP_ARREST };

typedef struct {
    int stage;
    // q: the attitude (w, x, y, z; body to level, the heading at the start as zero), from the attitude estimate at the start and the gyro since; turned: the roll counted from the gyro since the spin began (radians); thrust: what the motors have by now (N); vertical: the vertical speed (m/s; the hold's climb rate at the start, then the specific force along its own up); arrested: how long it has stayed under ARREST_DONE (s).
    float q[4];
    float time, turned, thrust, vertical, arrested;
    // How far it rolls (radians): 2 pi for the flip; 60, 90 or 30 degrees for the staged tests (flip_enable 2, 3 and 4, flip_turn): a short burst of the spin and the righting's brake at once, back the short way, so the drone peaks near 45-60 or 80-90 degrees with its thrust still mostly holding it up, and the motor split, the rotors' momentum, the floor and the brake are tested with margin. The spin aims at spin_rate times the square root of its share of a whole turn (the rate it can brake from in that angle) and ends with handoff's share of it left (handoff / 2 pi of the turn).
    float turn;
    // √T_right - √T_left as the motors have it by now (flip_allocate, for yaw_ff).
    float spread;
    // The quaternion flip (1; the command's quaternion when it began): q is then the attitude estimate's own each tick (flip_tick), and from the climb's end one reference rotation about the forward axis at the heading the climb ended on is tracked (flip.c track) in place of the spin's and the righting's laws; elapsed counts from that end (s).
    float quaternion, heading, elapsed;
    // The quaternion flip's height cap: the room (m) from the hold's height at the start to the lower of hover_ceiling and FLIP_CEILING, and the height risen since (m, from its own vertical speed). The climb ends early, and the reference's collective is held to what keeps the thrust from lifting, once the height risen plus what the vertical speed still carries it (v² / 2g) reaches the room less FLIP_ROOM_MARGIN.
    float room, risen;
    // The quaternion flip's roll on the gyro since the reference began (radians); turned is its furthest.
    float rolled;
} Flip;

// The airframe's authority as the firmware's model has it: angular acceleration (rad/s²) per newton of differential thrust (roll), per newton of collective per radian of common tilt (pitch), per newton per radian of differential tilt (yaw); and the weight (N).
// indi and unmodeled: per axis (roll, pitch, yaw) the share INDI takes (0 = off) of the unmodeled angular acceleration it found on this authority (indi.h; rad/s²), which the allocation takes off what it is asked; the rotors' gyroscopic feedforwards (gyro_ff in pitch, yaw_ff in yaw) give way by the same share, as the measured acceleration has them.
typedef struct {
    float roll, pitch, yaw, weight, indi[3], unmodeled[3];
} FlipAuthority;

// One tick's commands: while owned the flip flies the motors itself (thrusts in N, each assembly's physical forward tilt in radians); while climbing or arresting (collective_set) it only sets the collective (N) under the attitude controller. handed_back marks the tick it gives the attitude back, with the attitude it has then (roll and pitch, radians, as flight_attitude's), which the estimate takes.
typedef struct {
    bool owned, collective_set, handed_back;
    float left, right, left_tilt, right_tilt, collective, roll, pitch;
} FlipOutput;

// The righting the flip's last stage and the tumble recovery (recover.c) both fly, and the motors while either owns them: the rate toward up is at most max_rate, sqrt(2 brake angle) and kp angle; k_roll and k_pitch (1/s) turn rate errors into angular accelerations, kd_yaw damps yaw; past roll_over from up with the error mostly in pitch it rolls the long way round; servo_max bounds the common tilt (radians); each motor makes at least floor and at most ceiling of the weight.
// gyro_ff: the pitch the rotors' spin momentum gives a rolling drone with one rotor faster than the other (-ω × H: ω_roll H (√T_right - √T_left) over the pitch inertia), fed forward from the thrusts asked for, rad/s² per rad/s per √N; 0 (recover.c) leaves it out.
typedef struct {
// yaw_ff: the yaw the rotors' spin momentum gives the body as their speeds part (-dH_z/dt) and as the pitch rate turns their tilted difference (ω_pitch H_x), fed forward from the thrusts asked for through the motors' lag, rad/s² per √N/s: the rotors' momentum per √N over the yaw inertia; 0 leaves it out.
    float kp, brake, max_rate, k_roll, k_pitch, kd_yaw, roll_over, servo_max, floor, ceiling, gyro_ff, yaw_ff;
} FlipRighting;

// Wanted angular accelerations (rad/s²) into thrusts and tilts: roll by differential thrust, pitch by common tilt, yaw by differential tilt, the collective (N) as asked where the motors' floor and ceiling leave room. thrust: what the motors have by now (N), followed here with their lag.
void flip_allocate(const FlipRighting *r, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float pitch_rate, float *thrust, float *spread, float dt, FlipOutput *out);
// Turn the thrust axis toward up about the axis between them, no faster than it can brake from, at this collective (N); far over with the error mostly in pitch, roll the long way round. up: the level frame's up in body axes (quaternion_up); gyro: body rates (rad/s).
void flip_righting(const FlipRighting *r, const FlipAuthority *a, const float *up, const float *gyro, float collective, float *thrust, float *spread, float dt, FlipOutput *out);

// roll and pitch: the attitude estimate when the flip starts (radians); vertical: the climb rate then (m/s).
void flip_start(Flip *flip, float hover_thrust, float roll, float pitch, float vertical);
// gyro: body rates (rad/s; roll positive right side down, pitch nose down, yaw left); accel: the measured specific force in body axes (g; forward, left, up).
void flip_step(Flip *flip, const FlipParams *p, const FlipAuthority *a, const float *gyro, const float *accel, float dt, FlipOutput *out);

// How flight_rt.c flies it (sw/sim/flip_firmware.py flies the same functions). The flip starts only on the laptop's flip request (radio.c f), once per arming, and only when flip_refusal finds nothing against it. While it runs and for FLIP_GRACE after the hand-back the attitude estimate runs on the gyro alone, and from the spin until FLIP_GRACE after the hand-back the tip cut is off. A link loss, a landing (the pilot's or the fly-away failsafe's) or the failsafe ends it while it only climbs (flip_abort); once the spin has begun nothing but a stop ends it early: the righting is the way back, it hands back within timeout of the start whatever happens and arrests within arrest_time after, and the link-loss landing and a landing asked for wait until then. A STOP stops the motors as always.
// The attitude estimate is gyro-only and the tip cut off for this long after the hand-back (s): the accelerometer still reads the arrest's thrust, and the attitude controller needs a moment to bring the last of the righting's tilt in.
#define FLIP_GRACE 1.0f

// Everything the flip request depends on, read under the runtime lock in one go (flight_rt.c flight_runtime_flip). Angles from level (pitch beyond pitch_trim) in radians, rates in rad/s, heights in m, speeds in m/s, the battery in V.
typedef struct {
    // The setting flip_enable, armed, a flip already flown since arming.
    bool enabled, armed, done;
    // The GPS hold: the hover holding (mode HOVER_HOLD, engaged), not on the camera, the GPS hold on (hover_gps_tau), its heading pinned and a fix in the last GPS_STALE_US; the hold on its own estimates (hover_ekf 0: the Kalman filter's attitude would take the spin's thrust for gravity); no link-loss landing, fly-away failsafe or test pattern.
    bool holding, camera, gps, aligned, located, ekf, landing, failsafe, pattern;
    // The attitude controller's airborne time (flight_math.h Controller airborne: since the collective first reached integrate_throttle; hover.h lifted stops counting after the takeoff), the hold's height above where it engaged and its climb rate, the GPS's ground speed, the attitude and body rates.
    float airborne, height, climb_rate, speed, roll, pitch, rate[3];
    // flip_height, and the smoothed battery voltage (0 before the first reading).
    float min_height, battery;
    // The whole flip asked for (flip_enable 1) with the quaternion setting on: refused, as the quaternion flip flies only the bursts (sw/sim/models/NOTES.md "Quaternion attitude": under the 5 m cap it has no room to climb first, and it reached the ground from 4.5 m in every simulated run).
    bool whole_quaternion;
} FlipCheck;

// The roll a flip_enable setting asks for (radians): 1 the whole flip, 2 a 60-degree burst and back, 3 a 90-degree one; flip_begin takes it from the command.
float flip_turn(float enable);
// Why the flip is refused, or NULL.
const char *flip_refusal(const FlipCheck *check);

// The flip as flown: the maneuver, the authority it was started with, whether one has been flown since arming, the time since its hand-back (s; negative before one), and what the attitude controller and the hover's height loop held when it began, given back at the hand-back and at its end.
typedef struct {
    Flip flip;
    FlipAuthority authority;
    bool done;
    float since;
    Controller saved;
    float height_integral;
    // The maneuver's parameters (flip_reset: flip_defaults), and the throttles flip_limit last let out.
    const FlipParams *params;
    float throttle[2];
} FlipRun;

// A new arming: no flip yet.
void flip_reset(FlipRun *run);
// Starts the flip from the attitude estimate (radians, as flight_attitude keeps it), the controller as it is, the hover's height integral (hover.h HoverState integral), collective (N), climb rate (m/s) and height (m, the hold's; the quaternion flip's cap), with the authority of command's model and hover mass.
void flip_begin(FlipRun *run, const Command *command, const Controller *controller, float height_integral, float collective, float roll, float pitch, float climb_rate, float height);
// The motors stopped: the flip ends at once.
void flip_cancel(FlipRun *run);
// A guard while it only climbs (a link loss, a landing, the fly-away failsafe): the flip ends there, the hover's height integral given back, and the attitude controller, which flew the climb, carries on. False, changing nothing, once the spin has begun.
bool flip_abort(FlipRun *run, float *height_integral);
bool flip_active(const FlipRun *run);
// The attitude estimate on the gyro alone: while the flip runs and FLIP_GRACE after its hand-back.
bool flip_gyro_only(const FlipRun *run);
// No tip cut: from the spin to FLIP_GRACE after the hand-back.
bool flip_tip_exempt(const FlipRun *run);
// For the flight record and status (flight_rt.c, log_format.py FLIP_PHASES): 0 none, 1 climbing, 2 spinning or righting (the flip owns the motors), 3 arresting.
int flip_phase(const FlipRun *run);
// One control tick, right after the attitude estimate: the flip's step. With the quaternion flip the estimate is the flip's attitude and nothing is handed back to it but the integrators; otherwise, while the estimate is on the gyro alone (flip_gyro_only: the climb to FLIP_GRACE after the hand-back) it takes the roll and pitch of the flip's own quaternion every tick, at the hand-back the controller takes its integrators and dead-reckoned velocity from before the flip; at its end the hover's height integral is given back too, so neither carries what the flip wound into them.
void flip_tick(FlipRun *run, const float *gyro, const float *accel, float dt, Attitude *attitude, Controller *controller, float *height_integral, FlipOutput *out);
// Every control tick, on the throttles about to be sent (flight_rt.c, after the attitude controller and flip_actuators): from the flip's start until FLIP_GRACE after its hand-back, and while recovering holds (flight_rt.c: a recovery and its grace, recover_gyro_only), each moves at most params->slew a second from the last tick's and stays under params->throttle_cap, so neither motor jumps between idle and full throttle (the ESCs' current and voltage spikes; sw/sim/recover_firmware.py: unlimited, the recovery's tuned righting stepped a throttle by up to 1000 in one tick); a stopped motor (0) stays stopped. Otherwise it only follows them.
void flip_limit(FlipRun *run, bool recovering, float *throttle, float dt);
// While the flip owns the motors: its thrusts (N) as throttles through the thrust table and each motor's thrust scale (as flight_control's, battery compensation included), a running motor never stopped, and its tilts as servo angles (degrees, as flight_rt.c's state.servo, within servo_limit).
void flip_actuators(const FlipOutput *out, const Command *command, float left_scale, float right_scale, float throttle_max, float servo_limit, float *throttle, float *servo);
