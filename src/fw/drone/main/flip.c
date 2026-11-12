#include "flip.h"

#include <math.h>

#define TWO_PI 6.28318531f
// How fast the flip's idea of the motors' thrust follows what it asks (s; the motors' lag).
#define FLIP_MOTOR_TAU 0.1f
#define GRAVITY 9.81f
// The arrest ends once the vertical speed has stayed under ARREST_DONE (m/s) for ARREST_HOLD (s).
#define ARREST_DONE 0.2f
#define ARREST_HOLD 0.3f
#define ARREST_FLOOR 0.6f
#define DEGREE (3.14159265f / 180)
// The flip request (flip_refusal) wants the hold settled this long after liftoff (s), within FLIP_LEVEL of level and turning under FLIP_STILL on every axis, its climb rate under FLIP_CLIMB_RATE and the GPS's ground speed under FLIP_SPEED (m/s), and the pack at FLIP_MIN_V or more: the climb asks for 2.1 times the weight, and a sagging pack gives less (3.7 V a cell under load on 4S).
#define FLIP_SETTLE_S 5.0f
#define FLIP_LEVEL (10 * DEGREE)
#define FLIP_STILL (30 * DEGREE)
#define FLIP_CLIMB_RATE 0.3f
#define FLIP_SPEED 1.0f
#define FLIP_MIN_V 14.8f

const FlipParams flip_defaults = {
    .climb_thrust = 1.9829f, .climb_time = 0.35f, .spin_rate = 11.3753f, .k_spin = 12.0f, .spin_thrust = 0.4f, .handoff = 1.9829f,
    .kp = 2.9482f, .brake = 9.0993f, .max_rate = 6.7122f, .k_roll = 7.2914f, .k_pitch = 20.0f, .kd_yaw = 2.0828f,
    .flip_thrust = 1.7758f, .right_angle = 0.6f, .servo_max = 0.4927f, .roll_over = 2.8798f, .kv = 15.0f, .arrest_time = 2.0f, .timeout = 2.0f,
    .floor = 0.2f, .ceiling = 1.1647f, .handoff_rate = 3.0f, .slew = 10000.0f, .throttle_cap = 1100.0f, .spin_lift = 0.0147f,
};

static float clampf(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

void flip_attitude(float *q, float roll, float pitch) {
    // flight_attitude's roll and pitch (about forward, then left; positive right side down and nose down) as a quaternion, the heading zero.
    float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

void flip_up(const float *q, float *up) {
    // The level frame's z row.
    up[0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    up[1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    up[2] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
}

void flip_euler(const float *q, float *roll, float *pitch) {
    *roll = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    *pitch = asinf(clampf(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1));
}

void flip_start(Flip *flip, float hover_thrust, float roll, float pitch, float vertical) {
    *flip = (Flip){.stage = FLIP_CLIMB, .thrust = hover_thrust, .vertical = vertical, .turn = TWO_PI};
    flip_attitude(flip->q, roll, pitch);
}

// The body rates turn the attitude: q += q (0, w) dt / 2, normalized.
void flip_rotate(float *q, const float *w, float dt) {
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

// recover.py's allocation, from the thrust the motors have by now.
void flip_allocate(const FlipRighting *r, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float *thrust, float dt, FlipOutput *out) {
    // Each motor between floor and ceiling of the weight: the difference within what that allows, then the collective as asked where the difference leaves room.
    float least = r->floor * a->weight, most = fmaxf(r->ceiling * a->weight, least);
    float diff = clampf(roll / a->roll, least - most, most - least);
    float total = clampf(fmaxf(collective, fabsf(diff) + 2 * least), fabsf(diff) + 2 * least, 2 * most - fabsf(diff));
    out->left = (total + diff) / 2;
    out->right = (total - diff) / 2;
    *thrust += (total - *thrust) * (1 - expf(-dt / FLIP_MOTOR_TAU));
    float have = fmaxf(*thrust, 0.5f), limit = sinf(r->servo_max);
    float common = asinf(clampf(pitch / (a->pitch * have), -limit, limit));
    float split = clampf(yaw / (a->yaw * have * cosf(common)), -0.3f, 0.3f);
    out->left_tilt = common - split;
    out->right_tilt = common + split;
    out->collective = total;
}

// recover.py's righting.
void flip_righting(const FlipRighting *r, const FlipAuthority *a, const float *up, const float *gyro, float collective, float *thrust, float dt, FlipOutput *out) {
    float sine = hypotf(up[0], up[1]), angle = atan2f(sine, up[2]);
    float axis[2] = {sine > 1e-6f ? -up[1] / sine : 1, sine > 1e-6f ? up[0] / sine : 0};
    if (up[2] < cosf(r->roll_over) && fabsf(up[0]) > fabsf(up[1])) {
        angle = atan2f(fabsf(up[1]), up[2]);
        axis[0] = up[1] > 0 ? -1 : 1;
        axis[1] = 0;
    }
    float rate = fminf(fminf(r->max_rate, sqrtf(2 * r->brake * angle)), r->kp * angle);
    flip_allocate(r, a, r->k_roll * (axis[0] * rate - gyro[0]), r->k_pitch * (axis[1] * rate - gyro[1]), -r->kd_yaw * gyro[2], collective, thrust, dt, out);
}

static FlipRighting righting(const FlipParams *p) {
    return (FlipRighting){p->kp, p->brake, p->max_rate, p->k_roll, p->k_pitch, p->kd_yaw, p->roll_over, p->servo_max, p->floor, p->ceiling};
}

// The collective that stops the vertical speed, at least ARREST_FLOOR of the weight: the attitude controller flies the arrest and needs thrust to do it (a flip or burst that climbed had cut it to 0.15 of the weight and tipped over in the sim).
static float arrest(const FlipParams *p, const FlipAuthority *a, float vertical) {
    return clampf(a->weight * (1 - p->kv * vertical / GRAVITY), ARREST_FLOOR * a->weight, 2.2f * a->weight);
}

void flip_step(Flip *flip, const FlipParams *p, const FlipAuthority *a, const float *gyro, const float *accel, float dt, FlipOutput *out) {
    *out = (FlipOutput){0};
    if (flip->stage == FLIP_IDLE) return;
    flip->time += dt;
    float *q = flip->q;
    flip_rotate(q, gyro, dt);
    // Up in body axes: the body's forward and left axes' height.
    float up[3];
    flip_up(q, up);
    flip->vertical += ((up[0] * accel[0] + up[1] * accel[1] + up[2] * accel[2]) - 1) * GRAVITY * dt;
    if (flip->stage == FLIP_ARREST) {
        out->collective_set = true;
        out->collective = arrest(p, a, flip->vertical);
        flip->arrested = fabsf(flip->vertical) < ARREST_DONE ? flip->arrested + dt : 0;
        if (flip->arrested >= ARREST_HOLD || flip->time > p->arrest_time) flip->stage = FLIP_IDLE;
        return;
    }
    if (flip->stage == FLIP_CLIMB) {
        out->collective_set = true;
        out->collective = p->climb_thrust * a->weight;
        flip->thrust = out->collective;
        if (flip->time >= p->climb_time) flip->stage = FLIP_SPIN;
        return;
    }
    out->owned = true;
    FlipRighting r = righting(p);
    if (flip->stage == FLIP_SPIN) {
        flip->turned += gyro[0] * dt;
        // Keep the roll axis level: the nose's height (up's forward component, positive nose up) driven to zero through whichever of pitch and yaw tilts it now.
        float nose = up[0];
        float pitch_rate = p->kp * nose * up[2], yaw_rate = -p->kp * nose * up[1];
        flip_allocate(&r, a, p->k_spin * (p->spin_rate * sqrtf(flip->turn / TWO_PI) - gyro[0]), p->k_spin * (pitch_rate - gyro[1]), p->k_spin * 0.3f * (yaw_rate - gyro[2]), fmaxf(p->spin_thrust + p->spin_lift * cosf(flip->turned), 0) * a->weight, &flip->thrust, dt, out);
        if (flip->turned >= flip->turn * (1 - p->handoff / TWO_PI)) flip->stage = FLIP_RIGHT;
    } else {
        flip_righting(&r, a, up, gyro, p->flip_thrust * a->weight, &flip->thrust, dt, out);
        bool slow = fabsf(gyro[0]) < p->handoff_rate && fabsf(gyro[1]) < p->handoff_rate && fabsf(gyro[2]) < p->handoff_rate;
        if ((up[2] > cosf(p->right_angle) && slow) || flip->time > p->timeout) {
            // Back to the attitude controller, with the attitude as flight_attitude keeps it.
            flip->stage = FLIP_ARREST;
            flip->time = 0;
            out->owned = false;
            out->handed_back = true;
            flip_euler(q, &out->roll, &out->pitch);
            out->collective_set = true;
            out->collective = arrest(p, a, flip->vertical);
        }
    }
    if (flip->stage == FLIP_SPIN && flip->time > p->timeout) flip->stage = FLIP_RIGHT;
}

float flip_turn(float enable) {
    return enable >= 3 ? TWO_PI / 4 : enable >= 2 ? TWO_PI / 6 : TWO_PI;
}

const char *flip_refusal(const FlipCheck *c) {
    if (!c->enabled) return "Flip is off (setting flip_enable)";
    if (!c->armed) return "Not armed";
    if (c->done) return "One flip per arming: land and arm again";
    if (!c->holding || c->landing) return "Flip only from the hover hold";
    if (c->camera || !c->gps) return "Flip only from the GPS hold (hover_camera 0, hover_gps_tau on)";
    if (c->ekf) return "Flip only with the hold on its own estimates (hover_ekf 0)";
    if (!c->aligned || !c->located) return "GPS hold not settled: heading not pinned or no fresh fix";
    if (c->failsafe) return "Fly-away failsafe active";
    if (c->pattern) return "Test pattern playing";
    if (c->airborne < FLIP_SETTLE_S) return "Not airborne long enough";
    if (c->height < c->min_height) return "Below flip_height";
    if (fabsf(c->climb_rate) > FLIP_CLIMB_RATE) return "Still climbing or sinking";
    if (c->speed > FLIP_SPEED) return "Moving over the ground";
    if (fabsf(c->roll) > FLIP_LEVEL || fabsf(c->pitch) > FLIP_LEVEL) return "Not level";
    if (fabsf(c->rate[0]) > FLIP_STILL || fabsf(c->rate[1]) > FLIP_STILL || fabsf(c->rate[2]) > FLIP_STILL) return "Not still";
    if (c->battery < FLIP_MIN_V) return "Battery under 14.8 V";
    return NULL;
}

void flip_reset(FlipRun *run) {
    *run = (FlipRun){.since = -1, .params = &flip_defaults};
}

void flip_begin(FlipRun *run, const Command *command, const Controller *controller, float height_integral, float collective, float roll, float pitch, float climb_rate) {
    const Model *m = &command->model;
    run->authority = (FlipAuthority){m->arm / m->roll_inertia, m->pivot_height / m->pitch_inertia, m->arm / m->yaw_inertia, command->hover.mass * GRAVITY};
    run->done = true;
    run->since = -1;
    run->saved = *controller;
    run->height_integral = height_integral;
    flip_start(&run->flip, collective, roll, pitch, climb_rate);
    run->flip.turn = flip_turn(command->flip_enable);
}

void flip_cancel(FlipRun *run) {
    run->flip.stage = FLIP_IDLE;
    run->since = -1;
}

bool flip_abort(FlipRun *run, float *height_integral) {
    if (run->flip.stage != FLIP_CLIMB) return false;
    run->flip.stage = FLIP_IDLE;
    *height_integral = run->height_integral;
    return true;
}

bool flip_active(const FlipRun *run) {
    return run->flip.stage != FLIP_IDLE;
}

static bool grace(const FlipRun *run) {
    return run->since >= 0 && run->since < FLIP_GRACE;
}

bool flip_gyro_only(const FlipRun *run) {
    return flip_active(run) || grace(run);
}

bool flip_tip_exempt(const FlipRun *run) {
    return run->flip.stage == FLIP_SPIN || run->flip.stage == FLIP_RIGHT || grace(run);
}

int flip_phase(const FlipRun *run) {
    int stage = run->flip.stage;
    return stage == FLIP_CLIMB ? 1 : stage == FLIP_SPIN || stage == FLIP_RIGHT ? 2 : stage == FLIP_ARREST ? 3 : 0;
}

void flip_tick(FlipRun *run, const float *gyro, const float *accel, float dt, Attitude *attitude, Controller *controller, float *height_integral, FlipOutput *out) {
    if (run->since >= 0) run->since += dt;
    bool active = flip_active(run);
    flip_step(&run->flip, run->params, &run->authority, gyro, accel, dt, out);
    if (out->handed_back) {
        attitude->roll = out->roll;
        attitude->pitch = out->pitch;
        controller->roll_i = run->saved.roll_i;
        controller->pitch_i = run->saved.pitch_i;
        controller->yaw_i = run->saved.yaw_i;
        controller->velocity[0] = run->saved.velocity[0];
        controller->velocity[1] = run->saved.velocity[1];
        run->since = 0;
    }
    if (active && !flip_active(run)) *height_integral = run->height_integral;
}

void flip_limit(FlipRun *run, float *throttle, float dt) {
    flip_limit_with(run, false, throttle, dt);
}

void flip_limit_with(FlipRun *run, bool recovering, float *throttle, float dt) {
    const FlipParams *p = run->params;
    bool limited = flip_active(run) || grace(run) || recovering;
    for (int side = 0; side < 2; side++) {
        if (limited && throttle[side] > 0) {
            float last = run->throttle[side], step = p->slew * dt;
            throttle[side] = fminf(clampf(throttle[side], last - step, last + step), p->throttle_cap);
        }
        run->throttle[side] = throttle[side];
    }
}

void flip_actuators(const FlipOutput *out, const Command *c, float left_scale, float right_scale, float throttle_max, float servo_limit, float *throttle, float *servo) {
    float thrust[2] = {fmaxf(out->left, 0) / left_scale, fmaxf(out->right, 0) / right_scale};
    float tilt[2] = {c->left_trim + c->model.left_servo_direction * out->left_tilt, c->right_trim + c->model.right_servo_direction * out->right_tilt};
    for (int side = 0; side < 2; side++) {
        throttle[side] = clampf(roundf(flight_interpolate(thrust[side], &flight_thrust[0][0], flight_thrust_count, 1)), 1, throttle_max);
        servo[side] = clampf(tilt[side] / DEGREE, -servo_limit, servo_limit);
    }
}
