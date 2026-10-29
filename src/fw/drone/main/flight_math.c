#include <math.h>
#include <stddef.h>

#include "flight_math.h"

// (throttle, thrust in N). Rescaled 2026-10-28 for both ESCs recalibrated to 1000-2200 us (throttle 0-1200), from the median of three load-cell sweeps taken when the ESCs ran from about 200 to 1100 (sw/sim/models/NOTES.md): each throttle c now gives what 200 + 0.75 c gave then. Not yet re-measured; recordings before 2026-10-28 06:40 UTC used the old range.
const float flight_thrust[][2] = {
    {0, 0.000000f}, {50, 0.569017f}, {100, 0.949508f}, {150, 1.241442f}, {200, 1.544789f}, {250, 1.852657f},
    {300, 2.154418f}, {350, 2.459095f}, {400, 2.775708f}, {450, 3.140096f}, {500, 3.467688f}, {550, 3.821599f},
    {600, 4.264942f}, {650, 4.740267f}, {700, 5.232667f}, {750, 5.753304f}, {800, 6.313341f}, {850, 6.957856f},
    {900, 7.588432f}, {950, 8.207068f}, {1000, 8.815765f}, {1050, 9.411621f}, {1100, 10.024490f}, {1150, 10.633263f},
    {1200, 11.216827f},
};
const size_t flight_thrust_count = sizeof(flight_thrust) / sizeof(flight_thrust[0]);

static float clamp(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

// Calibration rows are (throttle, thrust). Reverse lookup uses the rising curve.
float flight_interpolate(float value, const float *table, size_t count, int reverse) {
    if (value <= table[reverse]) return table[1 - reverse];
    for (size_t i = 1; i < count; i++) {
        float x = table[2 * i + reverse];
        if (reverse && x < table[2 * (i - 1) + reverse]) {
            return table[2 * (i - 1)];
        }
        if (value <= x) {
            float x0 = table[2 * (i - 1) + reverse];
            float y0 = table[2 * (i - 1) + 1 - reverse];
            float y1 = table[2 * i + 1 - reverse];
            return y0 + (value - x0) / (x - x0) * (y1 - y0);
        }
    }
    return table[2 * (count - 1) + 1 - reverse];
}

void flight_attitude(Attitude *state, const Motion *m) {
    float roll = state->roll;
    float pitch = state->pitch;
    float sr = sinf(roll), cr = cosf(roll);
    float roll_rate = m->gx + tanf(pitch) * (m->gy * sr + m->gz * cr);
    float pitch_rate = m->gy * cr - m->gz * sr;
    float gravity_roll = atan2f(m->ay, m->az);
    float gravity_pitch = atan2f(-m->ax, sqrtf(m->ay * m->ay + m->az * m->az));
    float weight = 1.0f - expf(-m->dt / m->gravity_tau);

    roll += roll_rate * m->dt;
    pitch += pitch_rate * m->dt;
    state->roll = roll + weight * (gravity_roll - roll);
    state->pitch = pitch + weight * (gravity_pitch - pitch);
}

#define MIN_THRUST 0.3f
// A servo command reverses direction only after moving this far back, so gyro noise cannot toggle the compensation.
#define COMPENSATION_HYSTERESIS (0.3f * 3.14159265f / 180)
// Most tilt velocity damping may add to the roll/pitch target.
#define VELOCITY_TILT (8.0f * 3.14159265f / 180)
// The integrators see at most this much error, so a large transient (a tilted takeoff, a gust) winds them up only slowly while a steady error of any size is still trimmed out.
#define INTEGRATE_ERROR (8.0f * 3.14159265f / 180)
// Seconds over which the pitch bias fades in once collective reaches integrate_throttle.
#define PITCH_BIAS_RAMP 0.3f
// A takeoff from a rest more than UPRIGHT_START from level tilts the servos back by UPRIGHT_GAIN times the pitch from level (at most UPRIGHT_MAX), scaled down to nothing as the righting rotation approaches UPRIGHT_RATE, until the drone first comes within UPRIGHT_DONE of level. Tuned in the simulator from 12-30° rests with sliding and gripping nose contacts (sw/sim/sweep.py ground_pitch, feet).
#define UPRIGHT_START (2.0f * 3.14159265f / 180)
#define UPRIGHT_DONE (2.0f * 3.14159265f / 180)
#define UPRIGHT_MAX (35.0f * 3.14159265f / 180)
#define UPRIGHT_GAIN 0.7f
#define UPRIGHT_RATE (30.0f * 3.14159265f / 180)
// The upright phase also ends this long after collective reaches integrate_throttle, so a drone that lifts off without rotating is still levelled by the normal loop (s).
#define UPRIGHT_HOLD 0.4f

// The roll integrator starts from roll_bias so liftoff already counters a known disturbance. The pitch bias is instead faded in over PITCH_BIAS_RAMP after collective reaches integrate_throttle (flight_control): applied while the feet still hold the drone it only waits to kick the nose at liftoff, so it arrives just as the drone lifts.
void flight_reset(Controller *state, const Command *c) {
    state->roll_i = clamp(c->roll_bias, -c->integral_limit, c->integral_limit);
    state->pitch_i = 0;
    state->comp_dir[0] = state->comp_dir[1] = state->comp_ref[0] = state->comp_ref[1] = 0;
    state->airborne = 0;
    state->setpoint_roll = c->roll;
    state->setpoint_pitch = c->pitch;
    state->velocity[0] = state->velocity[1] = 0;
    state->yaw_i = 0;
    // Tilt from true level, not from pitch_trim: the upright takeoff points the thrust straight up on the ground, while the trim is the attitude to fly at.
    state->upright = fabsf(c->pitch - c->target_pitch) > UPRIGHT_START;
}

static float compensate(Controller *state, int side, float angle, float amount) {
    float *direction = &state->comp_dir[side], *extreme = &state->comp_ref[side];
    if (*direction > 0 ? angle > *extreme : angle > *extreme + COMPENSATION_HYSTERESIS) {
        *direction = 1;
        *extreme = angle;
    } else if (*direction < 0 ? angle < *extreme : angle < *extreme - COMPENSATION_HYSTERESIS) {
        *direction = -1;
        *extreme = angle;
    }
    return angle + *direction * amount;
}

// Armed without usable thrust: the servos wait at trim, tilted back for a tilted rest as the upright takeoff will want them, so they are already there when thrust comes.
static void hold_servos(Controller *state, const Command *c) {
    const Model *model = &c->model;
    float upright = state->upright ? clamp(UPRIGHT_GAIN * (c->pitch - c->target_pitch), -UPRIGHT_MAX, UPRIGHT_MAX) : 0;
    state->left_angle = clamp(c->left_trim - model->left_servo_direction * upright, model->servo_min, model->servo_max);
    state->right_angle = clamp(c->right_trim - model->right_servo_direction * upright, model->servo_min, model->servo_max);
}

void flight_control(Controller *state, const Command *c, const float *table, size_t count) {
    if (!c->enabled || c->base == 0) {
        flight_reset(state, c);
        state->left = state->right = state->left_angle = state->right_angle = 0;
        if (c->enabled) hold_servos(state, c);
        return;
    }

    float trim = c->pitch_trim;
    float force = 2 * flight_interpolate(c->base, table, count, 0) * cosf(trim);
    // Without thrust, atan2(horizontal, ~0) saturates the servos on gyro noise.
    if (force < 2 * MIN_THRUST) {
        flight_reset(state, c);
        state->left = state->right = c->base;
        hold_servos(state, c);
        return;
    }
    const Model *model = &c->model;
    // On the ground the setpoint follows the measured attitude, so a sloped surface is no error. Once collective first reaches integrate_throttle it ramps to the target at target_rate: liftoff levels off instead of stepping.
    if (c->base >= c->integrate_throttle || state->airborne) state->airborne += c->dt;
    if (!state->airborne) {
        state->setpoint_roll = c->roll;
        state->setpoint_pitch = c->pitch;
    } else {
        // Dead-reckoned horizontal velocity: tilt accelerates the vehicle at g * tan(tilt); the leak bounds attitude-error drift. Positive roll (right side down) moves right; positive pitch (nose down) moves forward.
        float keep = expf(-c->dt / c->velocity_leak);
        state->velocity[0] = keep * state->velocity[0] + 9.81f * tanf(c->roll) * c->dt;
        state->velocity[1] = keep * state->velocity[1] + 9.81f * tanf(c->pitch) * c->dt;
        float brake_roll = clamp(-c->velocity_damping * state->velocity[0], -VELOCITY_TILT, VELOCITY_TILT);
        float brake_pitch = clamp(-c->velocity_damping * state->velocity[1], -VELOCITY_TILT, VELOCITY_TILT);
        float step = c->target_rate * c->dt;
        state->setpoint_roll += clamp(c->target_roll + brake_roll - state->setpoint_roll, -step, step);
        state->setpoint_pitch += clamp(c->target_pitch + trim + brake_pitch - state->setpoint_pitch, -step, step);
    }
    // Resting tilted (no front legs), forward-pointing thrust would shove the drone along the floor and lever it over its nose. Until it first comes level the servos also tilt back by most of the pitch from level: the thrust points nearly up, and acting ahead of the back feet it lifts the nose. That back tilt is far more torque than the drone needs once it is pivoting or airborne, and the servos lag ~0.1 s, so it is withdrawn as the nose comes up (gone at UPRIGHT_RATE): the righting rotation stays slow enough for the pitch damping to stop at level. Meanwhile the setpoint follows the attitude, so the pitch P term and integrator stay out of it and the hand-over is smooth. Only pitch can be vectored this way.
    float tilt = c->pitch - c->target_pitch;
    if (state->upright && (fabsf(tilt) <= UPRIGHT_DONE || state->airborne > UPRIGHT_HOLD)) state->upright = 0;
    float upright = 0;
    if (state->upright) {
        float rising = tilt > 0 ? -c->gy : c->gy;
        upright = clamp(UPRIGHT_GAIN * tilt, -UPRIGHT_MAX, UPRIGHT_MAX) * clamp(1 - rising / UPRIGHT_RATE, 0, 1);
        state->setpoint_pitch = c->pitch;
    }
    float roll_error = state->setpoint_roll - c->roll;
    float pitch_error = state->setpoint_pitch - c->pitch;
    // Below integrate_throttle the feet are still down: integrating an error the ground holds would only wind up.
    if (c->base < c->integrate_throttle) {
        state->roll_i = clamp(c->roll_bias, -c->integral_limit, c->integral_limit);
        state->pitch_i = 0;
        state->yaw_i = 0;
    } else {
        state->yaw_i = clamp(state->yaw_i + c->yaw_i * (c->target_yaw - c->gz) * c->dt, -c->integral_limit, c->integral_limit);
        float roll_seen = clamp(roll_error, -INTEGRATE_ERROR, INTEGRATE_ERROR);
        float pitch_seen = clamp(pitch_error, -INTEGRATE_ERROR, INTEGRATE_ERROR);
        state->roll_i = clamp(state->roll_i + c->roll_i * roll_seen * c->dt, -c->integral_limit, c->integral_limit);
        state->pitch_i = clamp(state->pitch_i + c->pitch_i * pitch_seen * c->dt, -c->integral_limit, c->integral_limit);
    }

    float roll_torque = model->roll_inertia * (c->roll_p * roll_error + state->roll_i - c->roll_damping * c->gx);
    float pitch_bias = clamp(c->pitch_bias, -c->integral_limit, c->integral_limit) * clamp(state->airborne / PITCH_BIAS_RAMP, 0, 1);
    float pitch_torque = model->pitch_inertia * (c->pitch_p * pitch_error + state->pitch_i + pitch_bias - c->pitch_damping * c->gy);
    float yaw_torque = model->yaw_inertia * (c->yaw_damping * (c->target_yaw - c->gz) + c->yaw_bias + state->yaw_i);

    float left_vertical = fmaxf(0, (force + roll_torque / model->arm) / 2);
    float right_vertical = fmaxf(0, (force - roll_torque / model->arm) / 2);
    float forward = (pitch_torque - model->com_x * force) / model->pivot_height;
    yaw_torque -= model->reaction_torque_ratio * (right_vertical - left_vertical);
    float left_horizontal = (forward - yaw_torque / model->arm) / 2;
    float right_horizontal = (forward + yaw_torque / model->arm) / 2;

    float left_angle = model->left_servo_direction * (atan2f(left_horizontal, left_vertical) - upright) + c->left_trim;
    float right_angle = model->right_servo_direction * (atan2f(right_horizontal, right_vertical) - upright) + c->right_trim;
    state->left_angle = clamp(compensate(state, 0, left_angle, c->servo_compensation), model->servo_min, model->servo_max);
    state->right_angle = clamp(compensate(state, 1, right_angle, c->servo_compensation), model->servo_min, model->servo_max);
    state->left = roundf(flight_interpolate(hypotf(left_horizontal, left_vertical) / c->left_thrust_scale, table, count, 1));
    state->right = roundf(flight_interpolate(hypotf(right_horizontal, right_vertical) / c->right_thrust_scale, table, count, 1));
}
