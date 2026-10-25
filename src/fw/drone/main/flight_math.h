#ifndef FLIGHT_MATH_H
#define FLIGHT_MATH_H

#include <stddef.h>

#include "hover.h"

// Angles/rates are radians. Shared by the drone and the host simulation (sw/sim/flight.py).
typedef struct { float roll, pitch; } Attitude;
typedef struct { float ax, ay, az, gx, gy, gz, dt, gravity_tau; } Motion;
// comp_dir/comp_ref: each servo's last commanded direction (+1, -1, or 0 before it moved) and its extreme since. airborne counts seconds since collective first reached integrate_throttle (0 before); setpoint_* is the ramped roll/pitch target (with trim); velocity is the dead-reckoned (rightward, forward) velocity in m/s from g * tan(tilt).
// upright: 1 from arming at a tilted rest until the drone first comes level; meanwhile the servos also tilt back by most of the pitch from level, so the thrust points up, easing off as the nose comes up (flight_math.c).
typedef struct { float roll_i, pitch_i, left, right, left_angle, right_angle, comp_dir[2], comp_ref[2], airborne, setpoint_roll, setpoint_pitch, velocity[2], upright; } Controller;
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
    // Velocity damping: target tilt per m/s of dead-reckoned velocity (rad per m/s, 0 = off), and that estimate's leak time (s).
    float velocity_damping, velocity_leak;
    Model model;
    // Battery volts the thrust table is taken to hold at (0 = off): the runtime scales the thrust scales by (measured / this)² before flight_control, which does not read it. Last, so the host simulation's buffer layout is unchanged.
    float thrust_voltage;
    // Hover and landing (hover.h), and whether they use the camera (0 = IMU only). Also runtime-only.
    HoverGains hover;
    float hover_camera;
} Command;

// Rows of (throttle, thrust in N); flight_control takes it flattened.
extern const float flight_thrust[][2];
extern const size_t flight_thrust_count;

void flight_attitude(Attitude *state, const Motion *motion);
void flight_reset(Controller *state, const Command *command);
// Throttle to thrust (reverse 0) or thrust to throttle on the rising curve (reverse 1), on rows of (throttle, N).
float flight_interpolate(float value, const float *table, size_t count, int reverse);
void flight_control(Controller *state, const Command *command, const float *table, size_t count);

#endif
