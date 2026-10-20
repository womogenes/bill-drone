#ifndef FLIGHT_MATH_H
#define FLIGHT_MATH_H

#include <stddef.h>

// Angles/rates are radians. Shared by the drone and the host simulation (sw/sim/flight.py).
typedef struct { float roll, pitch; } Attitude;
typedef struct { float ax, ay, az, gx, gy, gz, dt, gravity_tau; } Motion;
typedef struct { float roll_i, pitch_i, left, right, left_angle, right_angle; } Controller;
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
    Model model;
} Command;

// Rows of (throttle, thrust in N); flight_control takes it flattened.
extern const float flight_thrust[][2];
extern const size_t flight_thrust_count;

void flight_attitude(Attitude *state, const Motion *motion);
void flight_reset(Controller *state, const Command *command);
void flight_control(Controller *state, const Command *command, const float *table, size_t count);

#endif
