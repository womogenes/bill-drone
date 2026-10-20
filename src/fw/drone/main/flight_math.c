#include <math.h>
#include <stddef.h>

#include "flight_math.h"

// (throttle, thrust in N): the median of three calibrated sweeps in sw/slop/comparison.csv. Throttle 200 is treated as approximately zero thrust, from the bench observation; thrust declines above 1100.
const float flight_thrust[][2] = {
    {0, 0.0f}, {200, 0.0f}, {250, 0.758689175f}, {300, 1.140325966f}, {350, 1.544788528f}, {400, 1.955279404f},
    {450, 2.353557228f}, {500, 2.775707658f}, {550, 3.261558474f}, {600, 3.673818136f}, {650, 4.264941843f},
    {700, 4.898708952f}, {750, 5.566624474f}, {800, 6.313341249f}, {850, 7.172694844f}, {900, 8.00416899f},
    {950, 8.815765343f}, {1000, 9.61023936f}, {1050, 10.438741463f}, {1100, 11.216827129f}, {1150, 11.052149476f},
    {1200, 10.992227314f},
};
const size_t flight_thrust_count = sizeof(flight_thrust) / sizeof(flight_thrust[0]);

static float clamp(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

// Calibration rows are (throttle, thrust). Reverse lookup uses the rising curve.
static float interpolate(float value, const float *table, size_t count, int reverse) {
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

// Integrators start from the configured bias so liftoff already counters a known disturbance.
void flight_reset(Controller *state, const Command *c) {
    state->roll_i = clamp(c->roll_bias, -c->integral_limit, c->integral_limit);
    state->pitch_i = clamp(c->pitch_bias, -c->integral_limit, c->integral_limit);
}

void flight_control(Controller *state, const Command *c, const float *table, size_t count) {
    if (!c->enabled || c->base == 0) {
        flight_reset(state, c);
        state->left = state->right = state->left_angle = state->right_angle = 0;
        return;
    }

    const Model *model = &c->model;
    float trim = c->pitch_trim;
    float force = 2 * interpolate(c->base, table, count, 0) * cosf(trim);
    // Without thrust, atan2(horizontal, ~0) saturates the servos on gyro noise.
    if (force < 2 * MIN_THRUST) {
        flight_reset(state, c);
        state->left = state->right = c->base;
        state->left_angle = clamp(c->left_trim, model->servo_min, model->servo_max);
        state->right_angle = clamp(c->right_trim, model->servo_min, model->servo_max);
        return;
    }
    float roll_error = c->target_roll - c->roll;
    float pitch_error = c->target_pitch + trim - c->pitch;
    state->roll_i = clamp(state->roll_i + c->roll_i * roll_error * c->dt,
                          -c->integral_limit, c->integral_limit);
    state->pitch_i = clamp(state->pitch_i + c->pitch_i * pitch_error * c->dt,
                           -c->integral_limit, c->integral_limit);

    float roll_torque = model->roll_inertia * (c->roll_p * roll_error + state->roll_i - c->roll_damping * c->gx);
    float pitch_torque = model->pitch_inertia * (c->pitch_p * pitch_error + state->pitch_i - c->pitch_damping * c->gy);
    float yaw_torque = model->yaw_inertia * (c->yaw_damping * (c->target_yaw - c->gz) + c->yaw_bias);

    float left_vertical = fmaxf(0, (force + roll_torque / model->arm) / 2);
    float right_vertical = fmaxf(0, (force - roll_torque / model->arm) / 2);
    float forward = (pitch_torque - model->com_x * force) / model->pivot_height;
    yaw_torque -= model->reaction_torque_ratio * (right_vertical - left_vertical);
    float left_horizontal = (forward - yaw_torque / model->arm) / 2;
    float right_horizontal = (forward + yaw_torque / model->arm) / 2;

    state->left_angle = clamp(model->left_servo_direction * atan2f(left_horizontal, left_vertical) + c->left_trim,
                              model->servo_min, model->servo_max);
    state->right_angle = clamp(model->right_servo_direction * atan2f(right_horizontal, right_vertical) + c->right_trim,
                               model->servo_min, model->servo_max);
    state->left = roundf(interpolate(hypotf(left_horizontal, left_vertical) / c->left_thrust_scale, table, count, 1));
    state->right = roundf(interpolate(hypotf(right_horizontal, right_vertical) / c->right_thrust_scale, table, count, 1));
}
