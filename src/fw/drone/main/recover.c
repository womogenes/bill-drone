#include "recover.h"

#include <math.h>

#define GRAVITY 9.81f
#define DEGREE (3.14159265f / 180)
// How fast the righting's idea of the motors' thrust follows what it asks (s; the motors' lag, as flip.c's).
#define MOTOR_TAU 0.1f
// The gyro's full scale is 2000 deg/s (runtime.h GYRO_FS_SEL); a tick's mean this close to it has clipped samples in it, and the attitude carried on the gyro is no longer the drone's.
#define SATURATED (1900 * DEGREE)
// Leveling counts as settled under this vertical speed (m/s) held for SETTLE_S (s).
#define ARREST_SPEED 0.3f
#define SETTLE_S 0.3f

const RecoverParams recover_defaults = {
    .kp = 2.4309f, .brake = 15.4237f, .max_rate = 10.3931f, .k_roll = 7.4738f, .k_pitch = 2.1245f, .kd_yaw = 2.005f, .roll_over = 165 * DEGREE, .servo_max = 0.8003f,
    .kv = 3.0f, .floor = 0.3f, .idle = 0.1f,
    .right_angle = 30 * DEGREE, .right_rate = 1.5f, .right_time = 2.5f,
    .level_angle = 10 * DEGREE, .level_rate = 0.5f, .level_time = 2.0f,
};

static float clampf(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

void recover_reset(RecoverRun *run) {
    *run = (RecoverRun){.q = {1, 0, 0, 0}, .since = -1};
}

void recover_abort(RecoverRun *run) {
    run->abort = true;
}

bool recover_active(const RecoverRun *run) {
    return run->stage != RECOVER_IDLE;
}

static bool grace(const RecoverRun *run) {
    return run->since >= 0 && run->since < RECOVER_GRACE;
}

bool recover_gyro_only(const RecoverRun *run) {
    return recover_active(run) || grace(run);
}

// flight_attitude's roll and pitch (about forward, then left) as a quaternion, the heading zero.
static void seed(float *q, float roll, float pitch) {
    float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    q[0] = cr * cp;
    q[1] = sr * cp;
    q[2] = cr * sp;
    q[3] = -sr * sp;
}

// The body rates turn the attitude: q += q (0, w) dt / 2, normalized.
static void turn(float *q, const float *w, float dt) {
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

// Wanted angular accelerations into thrusts and tilts: roll by differential thrust (each motor kept at idle or more), pitch by common tilt, yaw by differential tilt, from the thrust the motors have by now.
static void allocate(RecoverRun *run, const RecoverParams *p, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float dt, FlipOutput *out) {
    float diff = roll / a->roll, idle = p->idle * a->weight;
    float total = fmaxf(collective, fabsf(diff) + 2 * idle);
    out->left = (total + diff) / 2;
    out->right = (total - diff) / 2;
    run->thrust += (total - run->thrust) * (1 - expf(-dt / MOTOR_TAU));
    float have = fmaxf(run->thrust, 0.5f), limit = sinf(p->servo_max);
    float common = asinf(clampf(pitch / (a->pitch * have), -limit, limit));
    float split = clampf(yaw / (a->yaw * have * cosf(common)), -0.3f, 0.3f);
    out->left_tilt = common - split;
    out->right_tilt = common + split;
    out->collective = total;
}

// The collective that holds the height along up and stops the vertical speed (N).
static float arrest(const RecoverParams *p, const FlipAuthority *a, float vertical, float upright) {
    return clampf(a->weight * (1 - p->kv * vertical / GRAVITY) / fmaxf(upright, 0.5f), 0.3f * a->weight, 2.0f * a->weight);
}

static void begin(RecoverRun *run, const RecoverInput *in, int cause, RecoverOutput *out) {
    run->stage = RECOVER_RIGHT;
    run->cause = cause;
    run->recoveries++;
    run->time = run->righting = run->settled = 0;
    run->since = -1;
    run->thrust = in->collective;
    run->vertical = in->climb_rate;
    out->cancel_flip = in->flip_active;
}

void recover_tick(RecoverRun *run, const RecoverParams *p, const RecoverInput *in, Attitude *attitude, Controller *controller, float *height_integral, RecoverOutput *out) {
    *out = (RecoverOutput){0};
    float dt = in->dt;
    const float *w = in->gyro;
    if (run->since >= 0) run->since += dt;
    bool ordinary = !recover_active(run) && !in->flip_gyro_only && !grace(run);
    if (ordinary) {
        seed(run->q, attitude->roll, attitude->pitch);
        run->saved = *controller;
        run->height_integral = *height_integral;
    } else {
        turn(run->q, w, dt);
    }
    float *q = run->q;
    // Up in body axes (the level frame's z row), and the tilt from it.
    float up[3] = {2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])};
    float tilt = acosf(clampf(up[2], -1, 1));
    bool saturated = fabsf(w[0]) > SATURATED || fabsf(w[1]) > SATURATED || fabsf(w[2]) > SATURATED;
    // After the flip's hand-back (its arrest and grace, when the tip cut is off for it), a thrust axis past tip_limit is the flip failing, as on 2026-11-12 02:27:15 UTC.
    bool after_flip = in->flip_gyro_only && in->flip_phase != 1 && in->flip_phase != 2;
    if (run->abort) {
        run->abort = false;
        if (!recover_active(run)) begin(run, in, RECOVER_ABORT, out);
    } else if (!recover_active(run)) {
        int cause = saturated ? RECOVER_SATURATED : in->tipped ? RECOVER_TILT : after_flip && tilt > in->tip_limit ? RECOVER_FLIP : RECOVER_NONE;
        if (cause && in->enabled && in->allowed) begin(run, in, cause, out);
        else if (in->tipped) out->cut = RECOVER_TIP_REASON;
        if (!recover_active(run)) return;
    }
    run->time += dt;
    run->vertical += ((up[0] * in->accel[0] + up[1] * in->accel[1] + up[2] * in->accel[2]) - 1) * GRAVITY * dt;
    const FlipAuthority *a = &in->authority;
    bool slow = fabsf(w[0]) < p->right_rate && fabsf(w[1]) < p->right_rate;
    if (run->stage == RECOVER_LEVEL && tilt > in->tip_limit) run->stage = RECOVER_RIGHT;
    if (run->stage == RECOVER_RIGHT && tilt < p->right_angle && slow) {
        // To the attitude controller, which ran on meanwhile with its outputs unused: the integrators and dead-reckoned velocity from before.
        run->stage = RECOVER_LEVEL;
        run->settled = 0;
        controller->roll_i = run->saved.roll_i;
        controller->pitch_i = run->saved.pitch_i;
        controller->yaw_i = run->saved.yaw_i;
        controller->velocity[0] = run->saved.velocity[0];
        controller->velocity[1] = run->saved.velocity[1];
    }
    // The attitude estimate flies q while this runs: Euler angles from it stay sane at any attitude.
    attitude->roll = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
    attitude->pitch = asinf(clampf(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1));
    if (run->stage == RECOVER_RIGHT) {
        run->righting += dt;
        if (run->righting > p->right_time) {
            run->stage = RECOVER_IDLE;
            out->cut = RECOVER_FAILED_REASON;
            return;
        }
        // Turn the thrust axis toward up about the axis between them, no faster than it can brake from; far over with the error mostly in pitch, roll the long way round.
        float sine = hypotf(up[0], up[1]), angle = atan2f(sine, up[2]);
        float axis[2] = {sine > 1e-6f ? -up[1] / sine : 1, sine > 1e-6f ? up[0] / sine : 0};
        if (up[2] < cosf(p->roll_over) && fabsf(up[0]) > fabsf(up[1])) {
            angle = atan2f(fabsf(up[1]), up[2]);
            axis[0] = up[1] > 0 ? -1 : 1;
            axis[1] = 0;
        }
        float rate = fminf(fminf(p->max_rate, sqrtf(2 * p->brake * angle)), p->kp * angle);
        float collective = arrest(p, a, run->vertical, up[2]) * (p->floor + (1 - p->floor) * clampf(up[2], 0, 1));
        allocate(run, p, a, p->k_roll * (axis[0] * rate - w[0]), p->k_pitch * (axis[1] * rate - w[1]), -p->kd_yaw * w[2], collective, dt, &out->motors);
        out->owned = true;
        return;
    }
    out->level = true;
    out->motors.collective = arrest(p, a, run->vertical, up[2]);
    run->thrust = out->motors.collective;
    bool settled = tilt < p->level_angle && fabsf(w[0]) < p->level_rate && fabsf(w[1]) < p->level_rate && fabsf(run->vertical) < ARREST_SPEED;
    run->settled = settled ? run->settled + dt : 0;
    if (run->settled >= SETTLE_S || run->time > p->level_time + run->righting) {
        run->stage = RECOVER_IDLE;
        run->since = 0;
        *height_integral = run->height_integral;
    }
}
