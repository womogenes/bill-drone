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

#define HALF_TURN 3.14159265f

void quaternion_from_euler(float *q, float roll, float pitch) {
    // flight_attitude's roll and pitch (about forward, then left; positive right side down and nose down) as a quaternion, the heading zero.
    float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

void quaternion_up(const float *q, float *up) {
    // The level frame's z row.
    up[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    up[1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    up[2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
}

void quaternion_euler(const float *q, float *roll, float *pitch) {
    *roll = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    *pitch = asinf(clampf(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1));
}

// The body rates turn the attitude: q += q (0, w) dt / 2, normalized.
void quaternion_rotate(float *q, const float *w, float dt) {
    float h = dt / 2;
    float dw = -q[1] * w[0] - q[2] * w[1] - q[3] * w[2], dx = q[0] * w[0] + q[2] * w[2] - q[3] * w[1];
    float dy = q[0] * w[1] + q[3] * w[0] - q[1] * w[2], dz = q[0] * w[2] + q[1] * w[1] - q[2] * w[0];
    q[0] += dw * h;
    q[1] += dx * h;
    q[2] += dy * h;
    q[3] += dz * h;
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) q[i] /= n;
}

void quaternion_multiply(const float *a, const float *b, float *out) {
    float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
    out[0] = w;
    out[1] = x;
    out[2] = y;
    out[3] = z;
}

float quaternion_heading(const float *q) {
    // q = (cos h/2, 0, 0, sin h/2) ⊗ (a, b, c, 0): the turn about the level frame's up, then a tilt about a horizontal body axis.
    return 2 * atan2f(q[3], q[0]);
}

void quaternion_target(float *q, float heading, float roll, float pitch) {
    float tilt[4], turn[4] = {cosf(heading / 2), 0, 0, sinf(heading / 2)};
    quaternion_from_euler(tilt, roll, pitch);
    quaternion_multiply(turn, tilt, q);
}

void quaternion_roll(float *q, const float *from, float angle) {
    float turn[4] = {cosf(angle / 2), sinf(angle / 2), 0, 0};
    quaternion_multiply(from, turn, q);
}

void attitude_error(const float *q, const float *target, float *error, float *turn) {
    // The rotation from q to target in q's body axes, q⁻¹ ⊗ target, the short way (w ≥ 0).
    float inverse[4] = {q[0], -q[1], -q[2], -q[3]}, e[4];
    quaternion_multiply(inverse, target, e);
    if (e[0] < 0)
        for (int i = 0; i < 4; i++) e[i] = -e[i];
    // Split as the tilt about a horizontal body axis (a, b, c, 0), then the turn about the thrust axis (w, 0, 0, z) / a: Brescianini and D'Andrea's q_e,red ⊗ q_e,yaw.
    float a = hypotf(e[0], e[3]);
    if (a > 1e-6f) {
        float b = (e[0] * e[1] - e[2] * e[3]) / a, c = (e[0] * e[2] + e[1] * e[3]) / a, s = hypotf(b, c);
        float angle = 2 * atan2f(s, a);
        error[0] = s > 1e-9f ? angle * b / s : 2 * b;
        error[1] = s > 1e-9f ? angle * c / s : 2 * c;
        error[2] = 2 * atan2f(e[3], e[0]);
    } else {
        // Upside down from the target: the half turn about the horizontal axis (e[1], e[2]) itself, with no turn about the thrust axis.
        float s = fmaxf(hypotf(e[1], e[2]), 1e-9f);
        error[0] = HALF_TURN * e[1] / s;
        error[1] = HALF_TURN * e[2] / s;
        error[2] = 0;
    }
    if (turn) {
        // v + 2 w (u × v) + 2 u × (u × v), u the vector part.
        float u[3] = {e[1], e[2], e[3]}, v[3] = {turn[0], turn[1], turn[2]};
        float t[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
        float tt[3] = {u[1] * t[2] - u[2] * t[1], u[2] * t[0] - u[0] * t[2], u[0] * t[1] - u[1] * t[0]};
        for (int i = 0; i < 3; i++) turn[i] = v[i] + 2 * e[0] * t[i] + 2 * tt[i];
    }
}

void attitude_rates(const AttitudeGains *g, const float *error, const float *rate, const float *accel, const float *gyro, float *wanted) {
    float angle = hypotf(error[0], error[1]);
    float speed = angle > 1e-6f ? fminf(fminf(g->kp, sqrtf(2 * g->brake / angle)), g->max_rate / angle) : g->kp;
    float want[3] = {speed * error[0], speed * error[1], g->kp_heading * error[2]};
    for (int i = 0; i < 3; i++) wanted[i] = g->gain[i] * (rate[i] + want[i] - gyro[i]) + (accel ? accel[i] : 0);
}

// The attitude estimate: q turned by the body rates, then the body turned about the axis between the up it has and the accelerometer's, toward the accelerometer's, by 1 - exp(-dt / gravity_tau) of the angle between them; that axis is horizontal, so the heading stays the gyro's. Roll and pitch are q's (pitch within ±90 degrees).
void flight_attitude(Attitude *state, const Motion *m) {
    float *q = state->q;
    float gyro[3] = {m->gx, m->gy, m->gz};
    quaternion_rotate(q, gyro, m->dt);
    float weight = 1.0f - expf(-m->dt / m->gravity_tau);
    float norm = sqrtf(m->ax * m->ax + m->ay * m->ay + m->az * m->az);
    if (weight > 0 && norm > 0) {
        float up[3], a[3] = {m->ax / norm, m->ay / norm, m->az / norm};
        quaternion_up(q, up);
        // The body turn that takes the measured up onto the estimate's, a × up, by weight of the angle between them: q ⊗ that turn reads the measured up.
        float c[3] = {a[1] * up[2] - a[2] * up[1], a[2] * up[0] - a[0] * up[2], a[0] * up[1] - a[1] * up[0]};
        float s = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
        if (s > 1e-9f) {
            float half = weight * atan2f(s, a[0] * up[0] + a[1] * up[1] + a[2] * up[2]) / 2, k = sinf(half) / s;
            float turn[4] = {cosf(half), k * c[0], k * c[1], k * c[2]};
            quaternion_multiply(q, turn, q);
        }
    }
    quaternion_euler(q, &state->roll, &state->pitch);
}

void flight_attitude_start(Attitude *state, const float *accel) {
    state->roll = atan2f(accel[1], accel[2]);
    state->pitch = atan2f(-accel[0], hypotf(accel[1], accel[2]));
    quaternion_from_euler(state->q, state->roll, state->pitch);
}

#define MIN_THRUST 0.3f
// A servo command reverses direction only after moving this far back, so gyro noise cannot toggle the compensation.
#define COMPENSATION_HYSTERESIS (0.3f * 3.14159265f / 180)
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
    state->roll_i = clampf(c->roll_bias, -c->integral_limit, c->integral_limit);
    state->pitch_i = 0;
    state->comp_dir[0] = state->comp_dir[1] = state->comp_ref[0] = state->comp_ref[1] = 0;
    state->airborne = 0;
    state->setpoint_roll = c->roll;
    state->setpoint_pitch = c->pitch;
    state->velocity[0] = state->velocity[1] = 0;
    state->yaw_i = 0;
    state->damped_gy = c->gy;
    // Tilt from true level, not from pitch_trim nor the target: the upright takeoff points the thrust straight up on the ground, while the trim is the attitude to fly at, and hover's target on the floor is the resting attitude itself (hover.c TAKEOFF_EASE_S), which zeroed the vectoring on every hover takeoff.
    state->upright = fabsf(c->pitch) > UPRIGHT_START;
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
    float upright = state->upright ? clampf(UPRIGHT_GAIN * c->pitch, -UPRIGHT_MAX, UPRIGHT_MAX) : 0;
    state->left_angle = clampf(c->left_trim - model->left_servo_direction * upright, model->servo_min, model->servo_max);
    state->right_angle = clampf(c->right_trim - model->right_servo_direction * upright, model->servo_min, model->servo_max);
}

// The forward thrusts (N; the two's sum, forward, and the right's over the left's, twist) within what each servo's stops allow at these vertical thrusts: pitch before yaw, so a servo at its stop gives up the twist first and the common tilt only past what no twist could give (Smeur et al.'s prioritized allocation, with roll and the collective already set by the motors). upright: the tilt back the upright takeoff adds (radians).
static void fit_tilts(const Command *c, float upright, float left_vertical, float right_vertical, float *forward, float *twist) {
    const Model *model = &c->model;
    float vertical[2] = {left_vertical, right_vertical}, trim[2] = {c->left_trim, c->right_trim}, direction[2] = {model->left_servo_direction, model->right_servo_direction};
    float low[2], high[2];
    for (int side = 0; side < 2; side++) {
        // The servo's stops as the tilt atan2(forward, vertical) has them (flight_control's angle = direction (tilt - upright) + trim), short of 80 degrees for the tangent.
        float a = direction[side] * (model->servo_min - trim[side]) + upright, b = direction[side] * (model->servo_max - trim[side]) + upright;
        low[side] = vertical[side] * tanf(clampf(fminf(a, b), -1.4f, 1.4f));
        high[side] = vertical[side] * tanf(clampf(fmaxf(a, b), -1.4f, 1.4f));
    }
    float half = clampf(*forward, low[0] + low[1], high[0] + high[1]) / 2;
    *forward = 2 * half;
    *twist = 2 * clampf(*twist / 2, fmaxf(half - high[0], low[1] - half), fminf(half - low[0], high[1] - half));
}

void flight_restore(Controller *state, const Controller *saved) {
    state->roll_i = saved->roll_i;
    state->pitch_i = saved->pitch_i;
    state->yaw_i = saved->yaw_i;
    state->velocity[0] = saved->velocity[0];
    state->velocity[1] = saved->velocity[1];
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
        float step = c->target_rate * c->dt;
        float roll_step = c->roll_target_rate > 0 ? c->roll_target_rate * c->dt : step;
        state->setpoint_roll += clampf(c->target_roll - state->setpoint_roll, -roll_step, roll_step);
        state->setpoint_pitch += clampf(c->target_pitch + trim - state->setpoint_pitch, -step, step);
    }
    // Resting tilted (no front legs), forward-pointing thrust would shove the drone along the floor and lever it over its nose. Until it first comes level the servos also tilt back by most of the pitch from level: the thrust points nearly up, and acting ahead of the back feet it lifts the nose. That back tilt is far more torque than the drone needs once it is pivoting or airborne, and the servos lag ~0.1 s, so it is withdrawn as the nose comes up (gone at UPRIGHT_RATE): the righting rotation stays slow enough for the pitch damping to stop at level. Meanwhile the setpoint follows the attitude, so the pitch P term and integrator stay out of it and the hand-over is smooth. Only pitch can be vectored this way. On the forward legs (2026-11-11) the tilted thrust's horizontal part pulls the body one way while the gripping feet hold it, a couple that pivots it further from level, faster than the servos' correction can stop: hover takeoffs, which had no vectoring, pitched 20-30 degrees past their -12 to +8 degree rests before the feet were 4 cm up (2026-11-12 00:53-01:31 UTC; sw/sim/takeoff_test.py).
    float tilt = c->pitch;
    if (state->upright && (fabsf(tilt) <= UPRIGHT_DONE || state->airborne > UPRIGHT_HOLD)) state->upright = 0;
    float upright = 0;
    if (state->upright) {
        float rising = tilt > 0 ? -c->gy : c->gy;
        upright = clampf(UPRIGHT_GAIN * tilt, -UPRIGHT_MAX, UPRIGHT_MAX) * clampf(1 - rising / UPRIGHT_RATE, 0, 1);
        state->setpoint_pitch = c->pitch;
    }
    // The setpoint's roll and pitch as an attitude at the drone's own heading (yaw is flown as a rate), and the tilt-prioritized error onto it in body axes, which near level is the Euler angles' errors and stays one rotation at any tilt.
    float target[4], error[3];
    quaternion_target(target, quaternion_heading(c->attitude), state->setpoint_roll, state->setpoint_pitch);
    attitude_error(c->attitude, target, error, NULL);
    float roll_error = error[0], pitch_error = error[1];
    // Below integrate_throttle the feet are still down: integrating an error the ground holds would only wind up.
    if (c->base < c->integrate_throttle) {
        state->roll_i = clampf(c->roll_bias, -c->integral_limit, c->integral_limit);
        state->pitch_i = 0;
        state->yaw_i = 0;
    } else {
        state->yaw_i = clampf(state->yaw_i + c->yaw_i * (c->target_yaw - c->gz) * c->dt, -c->integral_limit, c->integral_limit);
        float roll_seen = clampf(roll_error, -INTEGRATE_ERROR, INTEGRATE_ERROR);
        float pitch_seen = clampf(pitch_error, -INTEGRATE_ERROR, INTEGRATE_ERROR);
        state->roll_i = clampf(state->roll_i + c->roll_i * roll_seen * c->dt, -c->integral_limit, c->integral_limit);
        state->pitch_i = clampf(state->pitch_i + c->pitch_i * pitch_seen * c->dt, -c->integral_limit, c->integral_limit);
    }

    float roll_torque = model->roll_inertia * (c->roll_p * roll_error + state->roll_i - c->roll_damping * c->gx);
    float pitch_bias = clampf(c->pitch_bias, -c->integral_limit, c->integral_limit) * clampf(state->airborne / PITCH_BIAS_RAMP, 0, 1);
    // Speed moment (pitch_speed, 0 = off): forward speed pitches this airframe nose-up (the rotors flap back; on the hover sim's model a backward drift likewise kept the nose falling through the braking tilt, as at 2026-11-02 04:55 UTC). The torque builds with the speed, so it lags the tilt that made it by the speed's own time constant, and the attitude loop meets it only as error; cancelled here from the forward velocity. flight_rt.c sets pitch_speed only while the hover flies on the tag, when velocity is the hover's measured one, not the g tan(pitch) dead reckoning above. With 0 the torque is exactly as before.
    float speed_torque = c->pitch_speed * state->velocity[1];
    // The pitch damping acts on the rate low-passed over pitch_damping_tau: above pitch_damping 6 the raw rate's damping drove a 2.6-2.8 Hz shake (2026-11-02 04:36 UTC, pitch rate 67 deg/s RMS there at pitch_damping 8), where the servo's lag and the assembly's rising tilt authority leave the loop without phase, while the 0.6-0.9 Hz swing of the 2026-11-08 hops wants more damping than 6 gives. The filter gives up damping above its corner to keep it below.
    float smooth = c->pitch_damping_tau > 0 ? c->dt / (c->pitch_damping_tau + c->dt) : 1;
    state->damped_gy += (c->gy - state->damped_gy) * smooth;
    float pitch_torque = model->pitch_inertia * (c->pitch_p * pitch_error + state->pitch_i + pitch_bias - c->pitch_damping * state->damped_gy + speed_torque);
    float yaw_torque = model->yaw_inertia * (c->yaw_damping * (c->target_yaw - c->gz) + c->yaw_bias + state->yaw_i);
    // INDI (indi.h): its share of what the model leaves out is taken off; the integrators trim whatever it leaves.
    if (c->indi_share[0] > 0) roll_torque -= model->roll_inertia * c->indi_share[0] * c->unmodeled[0];
    if (c->indi_share[1] > 0) pitch_torque -= model->pitch_inertia * c->indi_share[1] * c->unmodeled[1];
    if (c->indi_share[2] > 0) yaw_torque -= model->yaw_inertia * c->indi_share[2] * c->unmodeled[2];

    // The motors' vertical thrusts: the collective plus and less the roll differential, each kept from going negative (past the table's top the reverse lookup and the runtime's clamp drop the excess). The collective is never raised to keep the differential (no air mode): near the ground that would lift a drone the hover is setting down.
    float differential = roll_torque / model->arm;
    float left_vertical = fmaxf(0, (force + differential) / 2);
    float right_vertical = fmaxf(0, (force - differential) / 2);
    float forward = (pitch_torque - model->com_x * force) / model->pivot_height;
    yaw_torque -= model->reaction_torque_ratio * (right_vertical - left_vertical);
    float twist = yaw_torque / model->arm;
    if (c->indi_share[0] > 0 || c->indi_share[1] > 0 || c->indi_share[2] > 0) fit_tilts(c, upright, left_vertical, right_vertical, &forward, &twist);
    float left_horizontal = (forward - twist) / 2;
    float right_horizontal = (forward + twist) / 2;

    float left_angle = model->left_servo_direction * (atan2f(left_horizontal, left_vertical) - upright) + c->left_trim;
    float right_angle = model->right_servo_direction * (atan2f(right_horizontal, right_vertical) - upright) + c->right_trim;
    left_angle = compensate(state, 0, left_angle, c->servo_compensation);
    right_angle = compensate(state, 1, right_angle, c->servo_compensation);
    state->left_angle = clampf(left_angle, model->servo_min, model->servo_max);
    state->right_angle = clampf(right_angle, model->servo_min, model->servo_max);
    state->left = roundf(flight_interpolate(hypotf(left_horizontal, left_vertical) / c->left_thrust_scale, table, count, 1));
    state->right = roundf(flight_interpolate(hypotf(right_horizontal, right_vertical) / c->right_thrust_scale, table, count, 1));
}
