#ifndef FLIGHT_MATH_H
#define FLIGHT_MATH_H

#include <math.h>
#include <stddef.h>

#include "guard.h"
#include "hover.h"

// Angles/rates are radians. Shared by the drone and the host simulation (sw/sim/flight.py).
// q: the attitude (w, x, y, z; body to level, its heading the gyro's own), of which roll and pitch are read (quaternion_euler).
typedef struct { float roll, pitch, q[4]; } Attitude;
typedef struct { float ax, ay, az, gx, gy, gz, dt, gravity_tau; } Motion;
// comp_dir/comp_ref: each servo's last commanded direction (+1, -1, or 0 before it moved) and its extreme since. airborne counts seconds since collective first reached integrate_throttle (0 before); setpoint_* is the ramped roll/pitch target (with trim); velocity is the hover's measured (rightward, forward) velocity in m/s while it flies on the tag, for the speed moment (pitch_speed).
// upright: 1 from arming at a tilted rest until the drone first comes level; meanwhile the servos also tilt back by most of the pitch from level, so the thrust points up, easing off as the nose comes up (flight_math.c).
// yaw_i: the yaw-rate integrator (rad/s²), trimming out steady twist such as the motors' reaction torque at unequal throttles.
// damped_gy: the pitch rate the pitch damping acts on, low-passed over pitch_damping_tau.
typedef struct { float roll_i, pitch_i, left, right, left_angle, right_angle, comp_dir[2], comp_ref[2], airborne, setpoint_roll, setpoint_pitch, velocity[2], upright, yaw_i, damped_gy; } Controller;
typedef struct {
    float arm, pivot_height, com_x;
    float roll_inertia, pitch_inertia, yaw_inertia, reaction_torque_ratio;
    float servo_min, servo_max, left_servo_direction, right_servo_direction;
} Model;
typedef struct {
    float roll, pitch, gx, gy, gz, dt;
    float enabled, target_roll, target_pitch, target_yaw, base;
    float roll_p, roll_i, roll_damping, pitch_p, pitch_i, pitch_damping;
    float yaw_damping, integral_limit, pitch_trim, left_trim, right_trim;
    // Integrator starting values and a constant yaw term, in rad/s², for known steady disturbances.
    float roll_bias, pitch_bias, yaw_bias;
    // Per-motor thrust multipliers; requested newtons are divided by these before the throttle lookup.
    float left_thrust_scale, right_thrust_scale;
    // Added to each servo command in the direction it last moved, to take up the servo's deadband and gear play.
    float servo_compensation;
    // Collective throttle below which the integrators hold their biases (on the ground).
    float integrate_throttle;
    // Fastest change of the roll/pitch setpoint, rad/s: after liftoff it moves from the ground attitude to the target.
    float target_rate;
    Model model;
    // Battery volts the thrust table is taken to hold at (0 = off): the runtime scales the thrust scales by (measured / this)² before flight_control, which does not read it. Last, so the host simulation's buffer layout is unchanged.
    float thrust_voltage;
    // Yaw-rate integral gain (1/s), runtime-only like the fields below it; the integrator shares integral_limit.
    float yaw_i;
    // Speed moment compensation (rad/s² per m/s of forward speed, 0 = off): added to the pitch command against the nose-up moment forward speed makes (flight_math.c). Runtime-only; flight_rt.c passes it only while the hover flies on the tag.
    float pitch_speed;
    // Hover and landing (hover.h), and whether they use the camera (0 = IMU only). Also runtime-only.
    HoverGains hover;
    float hover_camera;
    // Self-trim (flight_rt.c trim): the fraction of the way roll_bias and pitch_bias move toward a hop's steady command at each stop (0 = off). Runtime-only.
    float auto_trim;
    // Time constant of the low-pass on the pitch rate the pitch damping acts on (s, 0 = the raw rate; flight_math.c). Runtime-only, last.
    float pitch_damping_tau;
    // Fastest change of the roll setpoint alone (rad/s, 0 = target_rate): roll answers differential thrust far faster than pitch answers the servos, and at target_rate 30 deg/s a full roll stick reversal took a second in the setpoint alone (2026-11-08 outdoor flights). Runtime-only, last.
    float roll_target_rate;
    // The fly-away failsafe and the test pattern's limits (guard.h). Runtime-only, last.
    GuardGains guard;
    // The tilt past which flight_rt.c stops the motors (radians; settings.def tip_limit). Runtime-only, last.
    float tip_limit;
    // How fast the attitude estimate pulls toward the accelerometer with the motors spinning (s; settings.def attitude_tau). Runtime-only, last.
    float attitude_tau;
    // The burst (flip.h): the angle it rolls to and back (radians, 0 = the flip request refused) and the least height it starts from (m). Runtime-only, last.
    float flip_angle, flip_height;
    // Tumble recovery in place of the tip cut in flight (recover.h; settings.def recover_enable). Runtime-only, last.
    float recover_enable;
    // Incremental nonlinear dynamic inversion (indi.h): the settings indi (roll and yaw) and indi_pitch, and per axis (roll, pitch, yaw) the share now taken (indi_share, the settings faded in by flight_rt.c; 0 = off, the controller as before) of the unmodeled angular acceleration INDI found on this controller's model (unmodeled, rad/s²), which flight_control takes off what it asks. Runtime-only, last.
    float indi, indi_pitch, indi_share[3], unmodeled[3];
    // The attitude (w, x, y, z; body to level, from pose_zero, as roll and pitch are taken): the controller's roll and pitch errors are the tilt-prioritized error (attitude_error) against the setpoint's roll and pitch at its own heading. Runtime-only, last.
    float attitude[4];
} Command;

// Rows of (throttle, thrust in N); flight_control takes it flattened.
extern const float flight_thrust[][2];
extern const size_t flight_thrust_count;

void flight_attitude(Attitude *state, const Motion *motion);
// The attitude at rest from the accelerometer (g, body axes), the heading zero.
void flight_attitude_start(Attitude *state, const float *accel);
void flight_reset(Controller *state, const Command *command);
// Throttle to thrust (reverse 0) or thrust to throttle on the rising curve (reverse 1), on rows of (throttle, N).
float flight_interpolate(float value, const float *table, size_t count, int reverse);
void flight_control(Controller *state, const Command *command, const float *table, size_t count);
// Gives back the integrators and velocity a maneuver saved when it took over (the flip's and the recovery's hand-back).
void flight_restore(Controller *state, const Controller *saved);

// Quaternions (w, x, y, z) of the attitude, body to level: x forward, y left, z up, as the gyro's axes.
// From flight_attitude's roll and pitch (about forward, then left; positive right side down and nose down), the heading zero; turned by the body rates over dt (normalized); the level frame's up in body axes; and back to roll and pitch, pitch within ±90 degrees.
void quaternion_from_euler(float *q, float roll, float pitch);
void quaternion_rotate(float *q, const float *gyro, float dt);
void quaternion_up(const float *q, float *up);
void quaternion_euler(const float *q, float *roll, float *pitch);
// a ⊗ b into out (which may be either).
void quaternion_multiply(const float *a, const float *b, float *out);
// The heading: q's turn about the level frame's up once the tilt is taken out (the twist of a swing-twist split, radians), defined everywhere but upside down; and the attitude at a heading with roll and pitch as quaternion_from_euler takes them.
float quaternion_heading(const float *q);
void quaternion_target(float *q, float heading, float roll, float pitch);
// The attitude rolled by angle about its own forward axis (the flip's reference).
void quaternion_roll(float *q, const float *from, float angle);
// The tilt-prioritized attitude error (Brescianini and D'Andrea, "Tilt-prioritized quaternion control of quadrotors", 2018): q's rotation onto target split into the tilt first, then the turn about the thrust axis. In q's body axes (radians): error[0] and error[1] the rotation about a horizontal body axis that brings the thrust axis onto target's (angle times unit axis, always the short way; exactly upside down, about forward), error[2] the turn about the thrust axis that remains, within ±180 degrees. turn (may be NULL): a vector in target's body axes turned into q's (the reference's body rates).
void attitude_error(const float *q, const float *target, float *error, float *turn);
// The one attitude law's rate loop, as the flip and the recovery fly it: the body rates wanted are the reference's (rate, in q's body axes) plus the error's, the tilt's at kp per s but no faster than it can brake from (sqrt(2 brake angle)) nor max_rate, the heading's at kp_heading per s; the angular acceleration asked (rad/s², body axes) is gain times the rate error plus the reference's own (accel; NULL for none).
typedef struct {
    float kp, brake, max_rate, kp_heading, gain[3];
} AttitudeGains;
void attitude_rates(const AttitudeGains *g, const float *error, const float *rate, const float *accel, const float *gyro, float *wanted);

#endif
