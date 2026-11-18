#include "recover.h"
#include "runtime.h"

#include <math.h>

// The gyro's full scale is 2000 deg/s (runtime.h GYRO_FS_SEL); a tick's mean this close to it has clipped samples in it, and the attitude carried on the gyro is no longer the drone's.
#define SATURATED (1900 * DEGREE)
// Leveling counts as settled under this vertical speed (m/s) held for SETTLE_S (s).
#define ARREST_SPEED 0.3f
#define SETTLE_S 0.3f
// The vertical speed is checked. A tick's mean within the gyro's SATURATED share (95%) of the accelerometer's full scale (runtime.h ACCEL_FULL_SCALE_G) has clipped samples in it. Then the specific force integrated into the vertical speed is no longer the drone's, and the error stays in the integral: righting at full throttle asks up to 2.9 g along the thrust axis and a ground strike more, so on 2026-11-15's lost burst90 in the sim leveling believed a 10 m/s fall while the drone climbed at 11 m/s, and the hold it handed to coasted to 26 m. A jolt or a tumble that stays inside the scale integrates correctly (the 1 kHz samples' mean keeps the impulse, and up is the gyro's, which is good until the gyro saturates too), so saturation of either sensor is the test.
#define ACCEL_SATURATED (0.95f * ACCEL_FULL_SCALE_G)
// The hold's climb rate integrates the same accelerometer, and only its barometer filter (hover.c take_baro, a triple pole at 1 / hover_baro_tau) takes out what a saturation left in it: (1 + x + x² / 2) e^-x of it after x time constants, 6% after HOLD_SETTLE. A recovery starts from the hold's climb rate only that long after the last saturation; without the barometer, never after one.
#define HOLD_SETTLE 6
// The barometer's rate is the slope of a line through its readings over BARO_RATE_S (s). At its 0.09 m RMS in flight (ekf.c: 2026-11-09 under 10 degrees of tilt) and 50 readings a second the slope's noise is 0.09 / sqrt(25 × 0.5² / 12) = 0.12 m/s, well under ARREST_SPEED, so leveling can judge settling on it; it is the rate at the window's middle, BARO_RATE_S / 2 plus the reading's latency ago. The Kalman filter's vertical speed is no alternative: ekf.c integrates the same accelerometer and does not know when it clips.
#define BARO_RATE_S 0.5f
// The arrest's gain on that late rate: a first-order loop on the vertical speed with gain k (1/s) and a delay L crosses over at k rad/s and loses k L of phase there, so k L = π / 6 keeps 60 degrees of margin (kv's 7.2 /s on the barometer's 0.33 s would leave none).
#define BARO_PHASE (PI / 6)

// The righting's gains from sw/sim/recover_firmware.py --tune --wide (CMA-ES, 60 generations of 20 on the tip, tilt and tumble events at 2 and 3 m, the plant's uncertain terms widened; recover_firmware_params.json). The flip's righting gains (flip.c), which they started from, used a fifth of this pitch gain and recovered 30% of the tip events. The tuned righting keeps the collective up even inverted (floor near 1): pitch authority comes from tilted thrust, and without it pitched-over drones could not turn in time. right_time and the leveling's were not tuned, and idle is the flip's floor (FLIP_FLOOR 0.2; the tuning's 0.144 made no difference in the sim).
const RecoverParams recover_defaults = {
    .kp = 4.8131f, .brake = 137.6662f, .max_rate = 24.8436f, .k_roll = 20.0008f, .k_pitch = 14.0902f, .kd_yaw = 0.6032f, .servo_max = 0.8919f,
    .kv = 7.2089f, .floor = 0.9841f, .idle = FLIP_FLOOR,
    .right_angle = 1.0079f, .right_rate = 2.0123f, .right_time = 2.5f,
    .level_angle = 10 * DEGREE, .level_rate = 0.5f, .level_time = 2.0f,
};

void recover_reset(RecoverRun *run) {
    *run = (RecoverRun){.since = -1, .measured = true, .clipped = INFINITY, .baro_rate = NAN, .baro_fit = NAN};
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

// The collective that holds the height along up and stops the vertical speed at gain kv (N): above hover's weight over the cosine only for a descent.
static float arrest(const FlipAuthority *a, float vertical, float kv, float upright) {
    return clampf(a->weight * (1 - kv * vertical / GRAVITY) / fmaxf(upright, 0.5f), 0.3f * a->weight, 2.0f * a->weight);
}

// The arrest's gain on the barometer's rate (BARO_PHASE), never more than kv.
static float baro_gain(const RecoverParams *p, const RecoverInput *in) {
    return fminf(p->kv, BARO_PHASE / (BARO_RATE_S / 2 + in->baro_latency));
}

// The barometer's readings, kept every tick whether or not a recovery runs, and the line through those within BARO_RATE_S of now: its rate and its height now, ahead by the time since the window's middle and the latency. Neither until a kept reading is older than the window (so the window is full) and while the newest is fresh.
static void track_baro(RecoverRun *run, const RecoverInput *in) {
    if (in->baro_new && isfinite(in->baro)) {
        run->baro_height[run->baro_next] = in->baro;
        run->baro_at[run->baro_next] = run->clock;
        run->baro_next = (run->baro_next + 1) % RECOVER_BARO_READINGS;
        if (run->baro_count < RECOVER_BARO_READINGS) run->baro_count++;
    }
    run->baro_rate = run->baro_fit = NAN;
    if (!isfinite(in->baro)) return;
    float oldest = 0, mean_age = 0, mean_height = 0;
    int count = 0;
    for (int k = 0; k < run->baro_count; k++) {
        float age = run->clock - run->baro_at[k];
        oldest = fmaxf(oldest, age);
        if (age > BARO_RATE_S) continue;
        mean_age += age;
        mean_height += run->baro_height[k];
        count++;
    }
    if (oldest <= BARO_RATE_S || count < 2) return;
    mean_age /= count;
    mean_height /= count;
    float spread = 0, product = 0;
    for (int k = 0; k < run->baro_count; k++) {
        float age = run->clock - run->baro_at[k];
        if (age > BARO_RATE_S) continue;
        spread += (age - mean_age) * (age - mean_age);
        product += (age - mean_age) * (run->baro_height[k] - mean_height);
    }
    if (spread <= 0) return;
    // Height falls with age for a climb.
    run->baro_rate = -product / spread;
    run->baro_fit = mean_height + run->baro_rate * (mean_age + in->baro_latency);
}

// The barometer's backstop, on its raw reading whatever the estimates say: more than RECOVER_CEILING_MARGIN over the ceiling, the collective may at most stop the climb the barometer's rate shows (the arrest at that rate, at its gain), and without a rate hold hover's thrust. Never more than hover, and less while it still climbs; below the margin, no cap.
static float ceiling_cap(const RecoverRun *run, const RecoverParams *p, const RecoverInput *in, float upright) {
    if (!(in->baro > in->ceiling + RECOVER_CEILING_MARGIN)) return INFINITY;
    return arrest(&in->authority, isfinite(run->baro_rate) ? fmaxf(run->baro_rate, 0) : 0, baro_gain(p, in), upright);
}

static void begin(RecoverRun *run, const RecoverInput *in, int cause, RecoverOutput *out) {
    run->stage = RECOVER_RIGHT;
    run->cause = cause;
    run->recoveries++;
    run->time = run->righting = run->settled = 0;
    run->since = -1;
    run->thrust = in->collective;
    run->vertical = in->climb_rate;
    run->measured = isinf(run->clipped) || (in->hold_tau > 0 && run->clipped > HOLD_SETTLE * in->hold_tau);
    out->cancel_flip = in->flip_active;
}

// The nose's angle from level past which a running burst is lost (radians; RecoverInput flip_phase).
#define BURST_LOST_PITCH (30 * DEGREE)

void recover_tick(RecoverRun *run, const RecoverParams *p, const RecoverInput *in, const float *q, Controller *controller, float *height_integral, RecoverOutput *out) {
    *out = (RecoverOutput){0};
    float dt = in->dt;
    const float *w = in->gyro;
    if (run->since >= 0) run->since += dt;
    run->clock += dt;
    bool saturated = fabsf(w[0]) > SATURATED || fabsf(w[1]) > SATURATED || fabsf(w[2]) > SATURATED;
    bool clipping = saturated || fabsf(in->accel[0]) >= ACCEL_SATURATED || fabsf(in->accel[1]) >= ACCEL_SATURATED || fabsf(in->accel[2]) >= ACCEL_SATURATED;
    run->clipped = clipping ? 0 : run->clipped + dt;
    track_baro(run, in);
    bool ordinary = !recover_active(run) && !in->flip_gyro_only && !grace(run);
    if (ordinary) {
        run->saved = *controller;
        run->height_integral = *height_integral;
    }
    // Up in body axes, and the tilt from it.
    float up[3];
    quaternion_up(q, up);
    float tilt = acosf(clampf(up[2], -1, 1));
    out->cap = ceiling_cap(run, p, in, up[2]);
    // After the flip's hand-back (its arrest and grace, when the tip cut is off for it), a thrust axis past tip_limit is the flip failing, as on 2026-11-12 02:27:15 UTC.
    bool after_flip = in->flip_gyro_only && in->flip_phase != 2;
    // A burst lost while it flies: its nose past BURST_LOST_PITCH (on 2026-11-15 07:03 UTC the quaternion burst60's nose went to -33 degrees at the roll's brake, then over +78 as roll passed upside down, and the flip, which hands back only once upright or at its timeout, flew on until the pilot's stop at 4 m).
    bool lost = in->flip_phase == 2 && fabsf(up[0]) > sinf(BURST_LOST_PITCH);
    if (run->abort) {
        run->abort = false;
        if (!recover_active(run)) begin(run, in, RECOVER_ABORT, out);
    } else if (!recover_active(run)) {
        int cause = saturated ? RECOVER_SATURATED : in->tipped ? RECOVER_TILT : (after_flip && tilt > in->tip_limit) || lost ? RECOVER_FLIP : RECOVER_NONE;
        if (cause && in->enabled && in->allowed) begin(run, in, cause, out);
        else if (in->tipped) out->cut = RECOVER_TIP_REASON;
        if (!recover_active(run)) return;
    }
    run->time += dt;
    if (clipping) run->measured = false;
    float kv = p->kv;
    if (run->measured) {
        run->vertical += ((up[0] * in->accel[0] + up[1] * in->accel[1] + up[2] * in->accel[2]) - 1) * GRAVITY * dt;
    } else {
        // The integral no longer the drone's: the barometer's rate at the gain its lag allows, or without one no vertical speed, so the collective holds hover's thrust and nothing it cannot measure makes it climb.
        run->vertical = isfinite(run->baro_rate) ? run->baro_rate : 0;
        kv = baro_gain(p, in);
    }
    const FlipAuthority *a = &in->authority;
    bool slow = fabsf(w[0]) < p->right_rate && fabsf(w[1]) < p->right_rate;
    // Leveling goes back to righting only beyond both right_angle and tip_limit: with tip_limit under right_angle (45 against 58 degrees by default) a drone between them was handed to leveling and back every tick, its integrators restored and its settling restarted each time.
    if (run->stage == RECOVER_LEVEL && tilt > fmaxf(in->tip_limit, p->right_angle)) run->stage = RECOVER_RIGHT;
    if (run->stage == RECOVER_RIGHT && tilt < p->right_angle && slow) {
        // To the attitude controller, which ran on meanwhile with its outputs unused: the integrators and dead-reckoned velocity from before.
        run->stage = RECOVER_LEVEL;
        run->settled = 0;
        flight_restore(controller, &run->saved);
    }
    if (run->stage == RECOVER_RIGHT) {
        run->righting += dt;
        if (run->righting > p->right_time) {
            run->stage = RECOVER_IDLE;
            out->cut = RECOVER_FAILED_REASON;
            return;
        }
        // The one attitude law toward upright at the drone's own heading (the tilt error the short way), at the recovery's gains, each motor at idle of the weight or more (the burst's floor), no ceiling but flip_limit's throttle cap, and no gyroscopic or yaw feedforward (tuned for the burst only). Without a roll-over: in sw/sim/recover_firmware.py (64 events a scenario from 3 m) the long way round past it recovered tip 93.8%, tilt 76.6%, tumble 40.6% against 93.8%, 79.7% and 42.2% without.
        FlipAllocation r = {p->servo_max, p->idle, INFINITY, 0, 0};
        float collective = fminf(arrest(a, run->vertical, kv, up[2]) * (p->floor + (1 - p->floor) * clampf(up[2], 0, 1)), out->cap);
        float target[4], error[3], none[3] = {0, 0, 0}, wanted[3];
        quaternion_target(target, quaternion_heading(q), 0, 0);
        attitude_error(q, target, error, NULL);
        AttitudeGains g = {p->kp, p->brake, p->max_rate, 0, {p->k_roll, p->k_pitch, p->kd_yaw}};
        float seen[3];
        flip_predicted(a, error, w, seen);
        attitude_rates(&g, error, none, NULL, seen, wanted);
        flip_allocate(&r, a, wanted[0], wanted[1], wanted[2], collective, w[1], &run->thrust, NULL, dt, &out->motors);
        out->owned = true;
        return;
    }
    out->level = true;
    out->motors.collective = fminf(arrest(a, run->vertical, kv, up[2]), out->cap);
    run->thrust = out->motors.collective;
    bool settled = tilt < p->level_angle && fabsf(w[0]) < p->level_rate && fabsf(w[1]) < p->level_rate && fabsf(run->vertical) < ARREST_SPEED;
    run->settled = settled ? run->settled + dt : 0;
    if (run->settled >= SETTLE_S || run->time > p->level_time + run->righting) {
        run->stage = RECOVER_IDLE;
        run->since = 0;
        *height_integral = run->height_integral;
        if (!run->measured && isfinite(run->baro_rate)) {
            out->reseed = true;
            out->height = run->baro_fit;
            out->climb_rate = run->baro_rate;
        }
    }
}
