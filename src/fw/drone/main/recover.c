#include "recover.h"

#include <math.h>

#define GRAVITY 9.81f
#define DEGREE (3.14159265f / 180)
// The gyro's full scale is 2000 deg/s (runtime.h GYRO_FS_SEL); a tick's mean this close to it has clipped samples in it, and the attitude carried on the gyro is no longer the drone's.
#define SATURATED (1900 * DEGREE)
// Leveling counts as settled under this vertical speed (m/s) held for SETTLE_S (s).
#define ARREST_SPEED 0.3f
#define SETTLE_S 0.3f

// The righting's gains from sw/sim/recover_firmware.py --tune --wide (CMA-ES, 60 generations of 20 on the tip, tilt and tumble events at 2 and 3 m, the plant's uncertain terms widened; sw/sim/models/recover_firmware_params.json). The flip's righting gains (flip.c), which they started from, used a fifth of this pitch gain and recovered 30% of the tip events. The tuned righting keeps the collective up even inverted (floor near 1): pitch authority comes from tilted thrust, and without it pitched-over drones could not turn in time. right_time and the leveling's were not tuned.
const RecoverParams recover_defaults = {
    .kp = 4.8131f, .brake = 137.6662f, .max_rate = 24.8436f, .k_roll = 20.0008f, .k_pitch = 14.0902f, .kd_yaw = 0.6032f, .roll_over = 2.045f, .servo_max = 0.8919f,
    .kv = 7.2089f, .floor = 0.9841f, .idle = 0.1443f,
    .right_angle = 1.0079f, .right_rate = 2.0123f, .right_time = 2.5f,
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
        flip_attitude(run->q, attitude->roll, attitude->pitch);
        run->saved = *controller;
        run->height_integral = *height_integral;
    } else {
        flip_rotate(run->q, w, dt);
    }
    float *q = run->q;
    // Up in body axes, and the tilt from it.
    float up[3];
    flip_up(q, up);
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
    flip_euler(q, &attitude->roll, &attitude->pitch);
    if (run->stage == RECOVER_RIGHT) {
        run->righting += dt;
        if (run->righting > p->right_time) {
            run->stage = RECOVER_IDLE;
            out->cut = RECOVER_FAILED_REASON;
            return;
        }
        // The flip's righting (flip.h flip_righting) at the recovery's gains, each motor at idle of the weight or more and no ceiling.
        FlipRighting r = {p->kp, p->brake, p->max_rate, p->k_roll, p->k_pitch, p->kd_yaw, p->roll_over, p->servo_max, p->idle, INFINITY};
        float collective = arrest(p, a, run->vertical, up[2]) * (p->floor + (1 - p->floor) * clampf(up[2], 0, 1));
        flip_righting(&r, a, up, w, collective, &run->thrust, dt, &out->motors);
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
