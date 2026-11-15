// The sim's switch between flip.c's greedy allocation and the MPC (fw/drone/main/mpc.c), without touching flip.c: make_mpc.py builds the harness's library from a copy of flip.c whose flip_allocate is renamed flip_allocate_greedy and from recover.c compiled with flip_allocate as flip_allocate_greedy (the recovery keeps the greedy allocation), and this file's flip_allocate takes the flip's calls. mpc_mode 0 passes them through unchanged; 1 allocates with the MPC for the lane mpc_hook_select chose (its actuator estimate fed every tick by mpc_hook_sent) tracking the wanted accelerations and collective over the whole horizon; 2 the same with the roll and the collective tracked as flip_allocate's split would deliver them through the motors' lag (no lead on the motors, which the flip's gains were tuned without); 3 everything tracked as flip_allocate's own commands would deliver it through the actuators' lags at the thrust it allocated for (the greedy plan unless thrust moves under tilted servos or a bound binds).
#include <math.h>
#include <string.h>

#include "flip.h"
#include "mpc.h"

#define LANES 256
// flip.c's FLIP_MOTOR_TAU: how fast the flip's idea of the motors' thrust follows what it asks.
#define FLIP_MOTOR_TAU 0.1f

void flip_allocate_greedy(const FlipRighting *r, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float pitch_rate, float *thrust, float *spread, float dt, FlipOutput *out);

int mpc_mode;
// With mode 3, whether the roll reference looks ahead along the quaternion burst's reference profile (flip.c track's feedforward), so the plan sees the brake coming.
int mpc_preview;
MpcConfig mpc_config;
// Per lane: the MPC's state; the lane flying now, its throttle cap in newtons and its flip.
static Mpc lanes[LANES];
static Mpc *current;
static float cap;
static const FlipRun *flying;
// Over the run: MPC calls, projected-gradient iterations, polished solves.
long mpc_calls, mpc_iterations, mpc_polished;
// The last call, for the harness's trace: the roll, pitch and yaw wanted (rad/s²) and the collective (N) after INDI, then the thrusts (N) and tilts (rad) out, and each servo's and motor's estimate (rad, N).
float mpc_last[12];

void mpc_hook_init(int mode) {
    memset(lanes, 0, sizeof(lanes));
    mpc_config = mpc_defaults;
    mpc_mode = mode;
    current = NULL;
    mpc_calls = mpc_iterations = mpc_polished = 0;
}

void mpc_hook_select(int lane, float ceiling, const FlipRun *run) {
    current = lane >= 0 && lane < LANES ? &lanes[lane] : NULL;
    cap = ceiling;
    flying = run;
}

#define TWO_PI 6.28318531f
#define HALF_PI 1.57079633f

// flip.c's profile (static there), the reference's angular acceleration only: up to speed on a half sine averaging accel, coasting, braking the same way at turn, and for a burst (back) the same way back.
static float profile_accel(float turn, float speed, float accel, bool back, float t) {
    float top = fminf(speed, sqrtf(accel * turn)), ramp = top / accel, coast = (turn - top * ramp) / top, leg = 2 * ramp + coast, sign = 1;
    if (back && t >= leg) {
        t -= leg;
        sign = -1;
    }
    if (t >= leg || t < 0) return 0;
    float u = t < ramp ? t : leg - t, x = HALF_PI * 2 * fminf(u, ramp) / ramp, a = HALF_PI * accel * sinf(x);
    return sign * (t < ramp ? a : t < ramp + coast ? 0 : -a);
}

// The change in the reference's roll acceleration k + 1 ticks ahead from now (rad/s²), while a quaternion flip's reference runs; zero otherwise.
static void preview(float dt, float *change) {
    for (int k = 0; k < MPC_HORIZON; k++) change[k] = 0;
    if (!mpc_preview || !flying || !(flying->flip.quaternion > 0) || flying->flip.stage != FLIP_SPIN) return;
    const Flip *f = &flying->flip;
    const FlipParams *p = flying->params;
    float speed = p->ref_rate * sqrtf(fminf(f->turn / TWO_PI, 1));
    bool back = f->turn < TWO_PI;
    float now = profile_accel(f->turn, speed, p->ref_accel, back, f->elapsed + p->ref_lead);
    for (int k = 0; k < MPC_HORIZON; k++) change[k] = profile_accel(f->turn, speed, p->ref_accel, back, f->elapsed + p->ref_lead + (k + 1) * dt) - now;
}

void mpc_hook_sent(int lane, const float *tilt, const float *thrust, float dt) {
    if (lane >= 0 && lane < LANES) mpc_sent(&lanes[lane], &mpc_config, tilt, thrust, dt);
}

static void mpc_record(const FlipOutput *out) {
    float tail[8] = {out->left, out->right, out->left_tilt, out->right_tilt, current ? current->tilt[0] : 0, current ? current->tilt[1] : 0, current ? current->thrust[0] : 0, current ? current->thrust[1] : 0};
    memcpy(mpc_last + 4, tail, sizeof(tail));
}

void flip_allocate(const FlipRighting *r, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float pitch_rate, float *thrust, float *spread, float dt, FlipOutput *out) {
    mpc_last[0] = roll - a->indi[0] * a->unmodeled[0], mpc_last[1] = pitch - a->indi[1] * a->unmodeled[1], mpc_last[2] = yaw - a->indi[2] * a->unmodeled[2], mpc_last[3] = collective;
    if (!mpc_mode || !current) {
        flip_allocate_greedy(r, a, roll, pitch, yaw, collective, pitch_rate, thrust, spread, dt, out);
        mpc_record(out);
        return;
    }
    roll -= a->indi[0] * a->unmodeled[0];
    pitch -= a->indi[1] * a->unmodeled[1];
    yaw -= a->indi[2] * a->unmodeled[2];
    float keep = expf(-dt / FLIP_MOTOR_TAU);
    if (spread && r->yaw_ff != 0) {
        // As flip_allocate's yaw_ff, from the thrusts the last plan asked for this tick (the warm start's).
        const float *plan = current->plan;
        float left = current->planned ? plan[0] : current->sent_thrust[0], right = current->planned ? plan[MPC_MOTOR_BLOCKS] : current->sent_thrust[1];
        float wanted = sqrtf(fmaxf(right, 0)) - sqrtf(fmaxf(left, 0));
        if (isnan(*spread)) *spread = wanted;
        float rate = (wanted - *spread) / FLIP_MOTOR_TAU, common = (current->tilt[0] + current->tilt[1]) / 2;
        *spread += (wanted - *spread) * (1 - keep);
        yaw += r->yaw_ff * (rate - pitch_rate * sinf(common) * *spread);
    }
    float least = r->floor * a->weight, most = fminf(fmaxf(r->ceiling * a->weight, least), cap > 0 ? cap : INFINITY);
    float target[MPC_HORIZON][MPC_OUTPUTS];
    if (mpc_mode == 3) {
        // flip_allocate's own commands as the reference: its split through the motors' lag for the roll and the collective, and for pitch and yaw what its tilts give through the servos' dead time and horns (the swing reaction included) at the thrust it allocated for (have), not at the thrust the motors will have; so the plan is the greedy one unless the motors' thrust moves under tilts not yet where they will be (or a bound binds), and then the MPC trades the roll and collective against the pitch and yaw that would bring.
        float diff = clampf(roll / a->roll, least - most, most - least);
        float total = clampf(fmaxf(collective, fabsf(diff) + 2 * least), fabsf(diff) + 2 * least, 2 * most - fabsf(diff));
        float have = fmaxf(*thrust + (total - *thrust) * (1 - keep), 0.5f), limit = sinf(r->servo_max);
        float common = asinf(clampf(pitch / (a->pitch * have), -limit, limit));
        float differential = clampf(yaw / (a->yaw * have * cosf(common)), -0.3f, 0.3f);
        float thrusts[MPC_HORIZON][2], tilts[MPC_HORIZON][2], swing[MPC_HORIZON], asked[MPC_HORIZON][2], change[MPC_HORIZON];
        // The split now, and ahead with the reference's coming roll acceleration (mpc_preview).
        preview(dt, change);
        for (int k = 0; k < MPC_HORIZON; k++) {
            float d = clampf((roll + change[k]) / a->roll, least - most, most - least), t = clampf(fmaxf(collective, fabsf(d) + 2 * least), fabsf(d) + 2 * least, 2 * most - fabsf(d));
            asked[k][0] = (t + d) / 2, asked[k][1] = (t - d) / 2;
        }
        mpc_motor_response(current, &mpc_config, (const float (*)[2])asked, dt, thrusts);
        mpc_servo_response(current, &mpc_config, common - differential, common + differential, dt, tilts, swing);
        for (int k = 0; k < MPC_HORIZON; k++) {
            float sl = sinf(tilts[k][0]), sr = sinf(tilts[k][1]);
            target[k][0] = a->roll * (thrusts[k][0] - thrusts[k][1]);
            target[k][1] = a->pitch * mpc_config.pitch_gain * have / 2 * (sl + sr) + swing[k];
            target[k][2] = a->yaw * have / 2 * (sr - sl);
            target[k][3] = thrusts[k][0] + thrusts[k][1];
        }
    } else if (mpc_mode == 2) {
        // The roll and the collective as flip_allocate's split would deliver them through the motors' lag (so unconstrained and uncoupled the plan is the greedy one), pitch and yaw as wanted.
        float diff = clampf(roll / a->roll, least - most, most - least);
        float total = clampf(fmaxf(collective, fabsf(diff) + 2 * least), fabsf(diff) + 2 * least, 2 * most - fabsf(diff));
        float thrusts[MPC_HORIZON][2], asked[MPC_HORIZON][2];
        for (int k = 0; k < MPC_HORIZON; k++) asked[k][0] = (total + diff) / 2, asked[k][1] = (total - diff) / 2;
        mpc_motor_response(current, &mpc_config, (const float (*)[2])asked, dt, thrusts);
        for (int k = 0; k < MPC_HORIZON; k++) target[k][0] = a->roll * (thrusts[k][0] - thrusts[k][1]), target[k][1] = pitch, target[k][2] = yaw, target[k][3] = thrusts[k][0] + thrusts[k][1];
    } else {
        for (int k = 0; k < MPC_HORIZON; k++) target[k][0] = roll, target[k][1] = pitch, target[k][2] = yaw, target[k][3] = collective;
    }
    mpc_allocate(current, &mpc_config, a, (const float (*)[MPC_OUTPUTS])target, least, most, dt, out);
    mpc_calls++;

    mpc_iterations += current->iterations;
    mpc_polished += current->polished;
    *thrust += (out->collective - *thrust) * (1 - keep);
    mpc_record(out);
}
