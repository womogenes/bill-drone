#pragma once

#include <stdbool.h>

#include "flip.h"

// A model-predictive allocation (prototype; not flown, nothing in the firmware calls it yet): in place of flip_allocate's instant inversion, it plans the thrusts and the servo commands over the next MPC_HORIZON ticks against a model of what each actuator will do with them (each servo's dead time, its horn's second-order loop and slew, the ±60 degree stops; each motor's dead tick, lag, floor and ceiling) and of what the body gets from that (each motor's thrust along its tilted shaft, the assemblies' swing reaction), linearized each tick about the trajectory the last plan predicts (a linear time-varying model, one real-time iteration a tick), so that a thrust step does not land on servos still tilted for the thrust before it (the 2026-11-15 07:03 UTC burst60: the right motor jumped to the cap at the roll's brake with the servos about 12 degrees back, 60-100 ms behind; about 40-50 rad/s² nose up). The wanted angular accelerations are tracked with roll and pitch far ahead of the collective, the collective ahead of yaw, plus a small tilt effort and slew penalty; the condensed box-constrained QP is solved by projected gradient on the Jacobi-scaled problem, warm-started from the last plan shifted by a tick, then polished on the active set it finds. sw/sim/mpc (make_mpc.py, band_mpc.py) flies it in place of flip_allocate in the burst sims; sw/sim/mpc/bench_mpc.c times it.

// Ticks planned (10 ms each): long enough for the servos' dead time (MpcConfig servo_delay) and their horns' settling after it.
#define MPC_HORIZON 15
// The most servo dead time modeled (ticks), the history held.
#define MPC_DELAY_MAX 12
// The plan's moves (move blocking): each motor's command over ticks [0, 1), [1, 3), [3, 7), [7, MPC_HORIZON - 1) (its dead tick leaves the last tick unseen), the servos' common and differential tilt over [0, 1), [1, 3), [3, MPC_HORIZON - delay) (the dead time leaves the rest unseen; held at the last block).
#define MPC_MOTOR_BLOCKS 4
#define MPC_SERVO_BLOCKS 3
// The decisions: the left motor's blocks, the right's, the servos' common tilt's, their differential's.
#define MPC_INPUTS (2 * MPC_MOTOR_BLOCKS + 2 * MPC_SERVO_BLOCKS)
// What is tracked each tick: roll, pitch and yaw acceleration (rad/s²) and the total thrust (N).
#define MPC_OUTPUTS 4

typedef struct {
    // Each servo's dead time (ticks, at most MPC_DELAY_MAX - 1 and MPC_HORIZON - 4), the horn's loop (rad/s, damping ratio), its speed limit (rad/s), and the bounds on the common tilt and the differential (radians; their sum within the ±60 degree stops).
    int servo_delay[2];
    float servo_wn, servo_zeta, servo_slew, common_max, differential_max;
    // The motors' lag (s) after one dead tick.
    float motor_tau;
    // The assemblies' swing reaction (rad/s² of body pitch per rad/s² of tilt, indi.c ASSEMBLY_RATIO) and the tilt's pitch authority as a factor on FlipAuthority's (the flip regime fits 1.8-2.6).
    float assembly_ratio, pitch_gain;
    // The tracking weights: roll, pitch and yaw (per (rad/s²)²) and the total thrust (per N²), per tick; pitch and yaw at early times theirs over the ticks before a servo command sent now can act (the dead time: there only the thrust moves them, through the tilts already on their way).
    float weight[MPC_OUTPUTS], early;
    // Penalties per tick: the common tilt's size (per rad²), the differential's (per rad²), each move of a motor's command (per N²) and a servo's (per rad²) from the block before (the first from what was sent last).
    float tilt_effort, differential_effort, thrust_slew, tilt_slew;
    // Projected-gradient iterations a solve and real-time iterations (relinearizations) a tick.
    int iterations, passes;
} MpcConfig;

extern const MpcConfig mpc_defaults;

typedef struct {
    // The actuators as estimated from what was sent: each servo command (physical forward tilt, radians) of the last MPC_DELAY_MAX ticks (newest at next - 1), each horn's tilt and rate (rad, rad/s), the motors' thrust asked last tick and their thrust by now (N).
    float sent_tilt[MPC_DELAY_MAX][2];
    int next;
    float tilt[2], tilt_rate[2], sent_thrust[2], thrust[2];
    bool sent;
    // The last plan (MPC_INPUTS, as the decisions are laid out) and whether it was made last tick (else the next starts from holding what was sent).
    float plan[MPC_INPUTS];
    bool planned, fresh;
    // The last solve, for the record: the cost, the iterations it took and whether the polish held.
    float cost;
    int iterations;
    bool polished;
} Mpc;

void mpc_reset(Mpc *m);
// Every tick, the actuators as sent (each assembly's physical forward tilt, radians; each motor's thrust, N, zero for a stopped motor), whoever chose them: the estimate steps a tick on.
void mpc_sent(Mpc *m, const MpcConfig *c, const float *tilt, const float *thrust, float dt);
// The thrusts (N; left, right) the motors would have 1..MPC_HORIZON ticks ahead were asked[j] (left, right) sent j ticks from now: what an instant split delivers through the motors' dead tick and lag.
void mpc_motor_response(const Mpc *m, const MpcConfig *c, const float (*asked)[2], float dt, float (*thrust)[2]);
// Each assembly's tilt (radians; left, right) 1..MPC_HORIZON ticks ahead were left and right sent from now on, through the servos' dead time (what was sent before plays out first) and their horns, and the swing reaction's body pitch acceleration (rad/s²) through each of those ticks.
void mpc_servo_response(const Mpc *m, const MpcConfig *c, float left, float right, float dt, float (*tilt)[2], float *swing);
// The allocation: target[k] what is wanted k + 1 ticks ahead (roll, pitch and yaw acceleration, rad/s², and the total thrust, N), each motor between floor and ceiling (N). Writes this tick's thrusts (N) and tilts (radians) into out (left, right, left_tilt, right_tilt, collective).
void mpc_allocate(Mpc *m, const MpcConfig *c, const FlipAuthority *a, const float (*target)[MPC_OUTPUTS], float floor, float ceiling, float dt, FlipOutput *out);
