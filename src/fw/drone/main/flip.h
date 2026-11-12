#pragma once

#include <stdbool.h>

#include "flight_math.h"

// The roll flip (flip.c), as sw/sim/flip.py flies it: punch up level, spin about roll on differential thrust, right the thrust axis as recover.py does, then hand back to the attitude controller and stop the vertical speed.
// The maneuver's parameters, from sw/sim/models/flip_params_2026-11-12_bands.json (tune_recover.py --flip --bands): thrusts as fractions of the weight, rates in rad/s, angles in radians.
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
} FlipParams;

extern const FlipParams flip_defaults;

enum { FLIP_IDLE, FLIP_CLIMB, FLIP_SPIN, FLIP_RIGHT, FLIP_ARREST };

typedef struct {
    int stage;
    // q: the attitude (w, x, y, z; body to level, the heading at the start as zero), from the attitude estimate at the start and the gyro since; turned: the roll counted from the gyro since the spin began (radians); thrust: what the motors have by now (N); vertical: the vertical speed (m/s; the hold's climb rate at the start, then the specific force along its own up); arrested: how long it has stayed under ARREST_DONE (s).
    float q[4];
    float time, turned, thrust, vertical, arrested;
} Flip;

// The airframe's authority as the firmware's model has it: angular acceleration (rad/s²) per newton of differential thrust (roll), per newton of collective per radian of common tilt (pitch), per newton per radian of differential tilt (yaw); and the weight (N).
typedef struct {
    float roll, pitch, yaw, weight;
} FlipAuthority;

// One tick's commands: while owned the flip flies the motors itself (thrusts in N, each assembly's physical forward tilt in radians); while climbing or arresting (collective_set) it only sets the collective (N) under the attitude controller. handed_back marks the tick it gives the attitude back, with the attitude it has then (roll and pitch, radians, as flight_attitude's), which the estimate takes.
typedef struct {
    bool owned, collective_set, handed_back;
    float left, right, left_tilt, right_tilt, collective, roll, pitch;
} FlipOutput;

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
} FlipCheck;

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
// Starts the flip from the attitude estimate (radians, as flight_attitude keeps it), the controller as it is, the hover's height integral (hover.h HoverState integral), collective (N) and climb rate (m/s), with the authority of command's model and hover mass.
void flip_begin(FlipRun *run, const Command *command, const Controller *controller, float height_integral, float collective, float roll, float pitch, float climb_rate);
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
// One control tick, right after the attitude estimate: the flip's step, and at the hand-back the estimate takes its attitude and the controller its integrators and dead-reckoned velocity from before the flip; at its end the hover's height integral is given back too, so neither carries what the flip wound into them.
void flip_tick(FlipRun *run, const float *gyro, const float *accel, float dt, Attitude *attitude, Controller *controller, float *height_integral, FlipOutput *out);
// Every control tick, on the throttles about to be sent (flight_rt.c, after the attitude controller and flip_actuators): from the flip's start until FLIP_GRACE after its hand-back each moves at most params->slew a second from the last tick's and stays under params->throttle_cap, so neither motor jumps between idle and full throttle (the ESCs' current and voltage spikes); a stopped motor (0) stays stopped. Otherwise it only follows them.
void flip_limit(FlipRun *run, float *throttle, float dt);
// While the flip owns the motors: its thrusts (N) as throttles through the thrust table and each motor's thrust scale (as flight_control's, battery compensation included), a running motor never stopped, and its tilts as servo angles (degrees, as flight_rt.c's state.servo, within servo_limit).
void flip_actuators(const FlipOutput *out, const Command *command, float left_scale, float right_scale, float throttle_max, float servo_limit, float *throttle, float *servo);
