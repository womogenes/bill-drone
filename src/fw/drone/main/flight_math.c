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
    // Past 90 degrees of pitch the same attitude is the pitch's supplement with the roll half a turn round: kept within ±90 degrees of pitch, so an estimate carried through the vertical never hands the attitude controller the other form's error (2026-11-12 02:27: roll 136, pitch 159 is roll -44, pitch 21, which it flew as 140 degrees of error). Never reached below 90 degrees.
    if (fabsf(pitch) > HALF_TURN / 2) {
        pitch = copysignf(HALF_TURN, pitch) - pitch;
        roll += roll > 0 ? -HALF_TURN : HALF_TURN;
    }
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
        // Dead-reckoned horizontal velocity: tilt accelerates the vehicle at g * tan(tilt); the leak bounds attitude-error drift. Positive roll (right side down) moves right; positive pitch (nose down) moves forward.
        float keep = expf(-c->dt / c->velocity_leak);
        state->velocity[0] = keep * state->velocity[0] + 9.81f * tanf(c->roll) * c->dt;
        state->velocity[1] = keep * state->velocity[1] + 9.81f * tanf(c->pitch) * c->dt;
        float brake_roll = clampf(-c->velocity_damping * state->velocity[0], -VELOCITY_TILT, VELOCITY_TILT);
        float brake_pitch = clampf(-c->velocity_damping * state->velocity[1], -VELOCITY_TILT, VELOCITY_TILT);
        float step = c->target_rate * c->dt;
        float roll_step = c->roll_target_rate > 0 ? c->roll_target_rate * c->dt : step;
        state->setpoint_roll += clampf(c->target_roll + brake_roll - state->setpoint_roll, -roll_step, roll_step);
        state->setpoint_pitch += clampf(c->target_pitch + trim + brake_pitch - state->setpoint_pitch, -step, step);
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
    float roll_error = state->setpoint_roll - c->roll;
    float pitch_error = state->setpoint_pitch - c->pitch;
    float roll_i = state->roll_i, pitch_i = state->pitch_i, yaw_i = state->yaw_i;
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
    if (c->indi > 0) {
        // INDI (indi.h): its share of what the model leaves out is taken off; the integrators trim whatever it leaves.
        roll_torque -= model->roll_inertia * c->indi * c->unmodeled[0];
        pitch_torque -= model->pitch_inertia * c->indi * c->unmodeled[1];
        yaw_torque -= model->yaw_inertia * c->indi * c->unmodeled[2];
    }

    // The motors' vertical thrusts: the collective plus and less the roll differential, each kept from going negative. With mix_ceiling (0 = off, as before, when the reverse lookup and the runtime's clamp drop whatever is past the table's top) neither motor is asked for more than the table gives at that throttle (at its thrust scale): the collective gives way before the roll differential, which is cut only by what no collective could give, and the roll integrator does not wind further into a cut at either end. The collective is never raised to keep the differential (no air mode): near the ground that would lift a drone the hover is setting down.
    float lift = force, differential = roll_torque / model->arm;
    if (c->mix_ceiling > 0) {
        float top_left = flight_interpolate(c->mix_ceiling, table, count, 0) * c->left_thrust_scale;
        float top_right = flight_interpolate(c->mix_ceiling, table, count, 0) * c->right_thrust_scale;
        differential = clampf(differential, -top_right, top_left);
        lift = fminf(lift, fminf(2 * top_left - differential, 2 * top_right + differential));
    }
    float left_vertical = fmaxf(0, (lift + differential) / 2);
    float right_vertical = fmaxf(0, (lift - differential) / 2);
    // The differential lost to the limits (N; more than rounding).
    float cut = roll_torque / model->arm - (left_vertical - right_vertical);
    bool windup = c->mix_ceiling > 0 && c->base >= c->integrate_throttle;
    if (windup && fabsf(cut) > 1e-3f && cut * (state->roll_i - roll_i) > 0) state->roll_i = roll_i;
    float forward = (pitch_torque - model->com_x * force) / model->pivot_height;
    yaw_torque -= model->reaction_torque_ratio * (right_vertical - left_vertical);
    float twist = yaw_torque / model->arm;
    if (c->indi > 0) fit_tilts(c, upright, left_vertical, right_vertical, &forward, &twist);
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
    if (c->mix_ceiling > 0) {
        state->left = fminf(state->left, c->mix_ceiling);
        state->right = fminf(state->right, c->mix_ceiling);
    }
    if (windup) {
        // A servo at its stop: the pitch and yaw integrators do not wind further into it (each side's tilt forward past its stop; pitch tilts both forward, yaw the right forward and the left back).
        float left_past = model->left_servo_direction * (left_angle - state->left_angle), right_past = model->right_servo_direction * (right_angle - state->right_angle);
        float pitch_step = state->pitch_i - pitch_i, yaw_step = state->yaw_i - yaw_i;
        if (pitch_step * left_past > 0 || pitch_step * right_past > 0) state->pitch_i = pitch_i;
        if (yaw_step * right_past > 0 || yaw_step * left_past < 0) state->yaw_i = yaw_i;
    }
}
