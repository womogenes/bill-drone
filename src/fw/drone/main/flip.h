#pragma once

#include <stdbool.h>

// The roll flip (flip.c), as sw/sim/flip.py flies it: punch up level, spin about roll on differential thrust, brake the last of the turn, then hand back to the attitude controller.
// The maneuver's parameters, from sw/sim/models/flip_params_2026-11-12_bands.json (tune_recover.py --flip --bands): thrusts as fractions of the weight, rates in rad/s, angles in radians.
typedef struct {
    float climb_thrust, climb_time, spin_rate, k_spin, spin_thrust, handoff;
    // Braking the turn's last part (recover.py's righting): the rate toward level is at most max_rate, sqrt(2 brake angle) and kp angle; k_roll and k_pitch (1/s) turn rate errors into angular accelerations, kd_yaw damps yaw; the collective is flip_thrust of the weight; servo_max bounds the common tilt.
    float kp, brake, max_rate, k_roll, k_pitch, kd_yaw, flip_thrust, right_angle, servo_max;
    // After the hand-back the collective stops the vertical speed the flip left (kv per s, recover.py's arrest) until it has stayed under ARREST_DONE (flip.c), at most arrest_time (s).
    float kv, arrest_time;
    // Hand the attitude back regardless after this long (s).
    float timeout;
} FlipParams;

extern const FlipParams flip_defaults;

enum { FLIP_IDLE, FLIP_CLIMB, FLIP_SPIN, FLIP_BRAKE, FLIP_ARREST };

typedef struct {
    int stage;
    // vertical: the vertical speed since the flip began (m/s, from the measured specific force along the estimated up; the drone hovered when it began); arrested: how long it has stayed under ARREST_DONE (s).
    float time, turned, thrust, vertical, arrested;
    // The roll left at the hand-back, from where the turn started (radians, negative short of a whole turn).
    float landing_roll;
} Flip;

// The airframe's authority as the firmware's model has it: angular acceleration (rad/s²) per newton of differential thrust (roll), per newton of collective per radian of common tilt (pitch), per newton per radian of differential tilt (yaw); and the weight (N).
typedef struct {
    float roll, pitch, yaw, weight;
} FlipAuthority;

// One tick's commands: while owned the flip flies the motors itself (thrusts in N, each assembly's physical forward tilt in radians); while climbing or arresting (collective_set) it only sets the collective (N) under the attitude controller; handed_back marks the tick it gives the attitude back.
typedef struct {
    bool owned, collective_set, handed_back;
    float left, right, left_tilt, right_tilt, collective;
} FlipOutput;

void flip_start(Flip *flip, float hover_thrust);
// gyro: body rates (rad/s; roll positive right side down, pitch nose down, yaw left); vertical_g: the measured specific force along the estimated up (g).
void flip_step(Flip *flip, const FlipParams *p, const FlipAuthority *a, const float *gyro, float vertical_g, float dt, FlipOutput *out);
