#include "mpc.h"

#include <math.h>
#include <string.h>

#define N MPC_HORIZON
#define MB MPC_MOTOR_BLOCKS
#define SB MPC_SERVO_BLOCKS
#define NU MPC_INPUTS
#define NY (MPC_OUTPUTS * MPC_HORIZON)
// Substeps a tick for the horns, as indi.c's SERVO_STEPS.
#define STEPS 4

// The servos as indi.c has them (both sides' mean of sw/sim/models/airframe_2026-11-12_raised.json), 80 ms of dead time (the flip regime's 60-110 ms; the hover's 60), the common tilt within flip_defaults' servo_max and the differential within flip_allocate's 0.3 rad (so both servos stay inside their 60 degree stops); the motors' 66 ms lag (indi.c MOTOR_TAU); the swing reaction at indi.c's ASSEMBLY_RATIO; the weights from sw/sim/mpc runs: roll and pitch alike, the total thrust about two hundred times cheaper per newton than the motors' difference is in roll, yaw a fifth of roll.
const MpcConfig mpc_defaults = {
    .servo_delay = {8, 8}, .servo_wn = 37.5f, .servo_zeta = 0.87f, .servo_slew = 4.97f, .common_max = 0.5092f, .differential_max = 0.3f,
    .motor_tau = 0.066f, .assembly_ratio = 0.27f, .pitch_gain = 1.0f,
    .weight = {1.0f, 1.0f, 0.2f, 1.0f}, .early = 1.0f,
    .tilt_effort = 20.0f, .differential_effort = 20.0f, .thrust_slew = 0.5f, .tilt_slew = 20.0f,
    .iterations = 15, .passes = 1,
};

void mpc_reset(Mpc *m) {
    memset(m, 0, sizeof(*m));
}

// Where each block starts (ticks): a motor's (the last ends at N - 1) and a servo's (the last ends at its count, N less its dead time).
static const int MOTOR_START[MB + 1] = {0, 1, 3, 7, N - 1};
static const int SERVO_START[SB] = {0, 1, 3};

static int motor_block(int tick) {
    int b = MB - 1;
    while (b > 0 && tick < MOTOR_START[b]) b--;
    return b;
}

static int servo_block(int tick) {
    int b = SB - 1;
    while (b > 0 && tick < SERVO_START[b]) b--;
    return b;
}

static int delay_of(const MpcConfig *c, int side) {
    int d = c->servo_delay[side];
    return d < 0 ? 0 : d > MPC_DELAY_MAX - 1 ? MPC_DELAY_MAX - 1 : d > N - 4 ? N - 4 : d;
}

// One horn tick toward goal (STEPS substeps of the second-order loop under the slew limit); clamped[step] marks a substep at the limit.
static void horn(const MpcConfig *c, float goal, float *x, float *v, float dt, bool *clamped) {
    float h = dt / STEPS, w2 = c->servo_wn * c->servo_wn, z2 = 2 * c->servo_zeta * c->servo_wn;
    for (int step = 0; step < STEPS; step++) {
        float next = *v + (w2 * (goal - *x) - z2 * *v) * h;
        bool limited = fabsf(next) > c->servo_slew;
        if (clamped) clamped[step] = limited;
        *v = limited ? copysignf(c->servo_slew, next) : next;
        *x += *v * h;
    }
}

void mpc_sent(Mpc *m, const MpcConfig *c, const float *tilt, const float *thrust, float dt) {
    if (!m->sent) {
        m->sent = true;
        for (int k = 0; k < MPC_DELAY_MAX; k++) memcpy(m->sent_tilt[k], tilt, sizeof(m->sent_tilt[k]));
        memcpy(m->tilt, tilt, sizeof(m->tilt));
        memcpy(m->thrust, thrust, sizeof(m->thrust));
        memcpy(m->sent_thrust, thrust, sizeof(m->sent_thrust));
    }
    float keep = expf(-dt / c->motor_tau);
    memcpy(m->sent_tilt[m->next], tilt, sizeof(m->sent_tilt[0]));
    m->next = (m->next + 1) % MPC_DELAY_MAX;
    for (int side = 0; side < 2; side++) {
        // Through this tick the motor follows what was sent the tick before (its dead tick), the horn what was sent its dead time ago.
        m->thrust[side] = m->sent_thrust[side] + (m->thrust[side] - m->sent_thrust[side]) * keep;
        m->sent_thrust[side] = thrust[side];
        float goal = m->sent_tilt[(m->next - 1 - delay_of(c, side) + 2 * MPC_DELAY_MAX) % MPC_DELAY_MAX][side];
        horn(c, goal, &m->tilt[side], &m->tilt_rate[side], dt, NULL);
    }
    // A plan carries over only to the tick right after the one it was made in.
    if (!m->fresh) m->planned = false;
    m->fresh = false;
}

// The decisions' layout: motor side s block b at s * MB + b; the servos' common tilt block b at 2 MB + b and their differential (half the right's less the left's, as flip_allocate's) at 2 MB + SB + b.
static int motor_index(int side, int b) {
    return side * MB + b;
}

static int servo_index(int kind, int b) {
    return 2 * MB + kind * SB + b;
}

// A side's servo command from a block's common and differential tilt.
static float side_tilt(const float *u, int side, int b) {
    return u[servo_index(0, b)] + (side ? 1 : -1) * u[servo_index(1, b)];
}

// The trajectory a plan gives and its sensitivities: thrust, tilt and tilt rate at ticks 0..N; the outputs at ticks 1..N; and G, the outputs' change per unit of each decision.
typedef struct {
    float thrust[2][N + 1], tilt[2][N + 1], rate[2][N + 1];
    bool clamped[2][N][STEPS];
    float y[NY], g[NY][NU];
} Rollout;

static void rollout(const Mpc *m, const MpcConfig *c, const FlipAuthority *a, const float *u, float dt, Rollout *r) {
    float keep = expf(-dt / c->motor_tau);
    for (int side = 0; side < 2; side++) {
        int delay = delay_of(c, side);
        r->thrust[side][0] = m->thrust[side];
        r->tilt[side][0] = m->tilt[side];
        r->rate[side][0] = m->tilt_rate[side];
        for (int j = 0; j < N; j++) {
            float asked = j == 0 ? m->sent_thrust[side] : u[motor_index(side, motor_block(j - 1))];
            r->thrust[side][j + 1] = asked + (r->thrust[side][j] - asked) * keep;
            float goal = j < delay ? m->sent_tilt[(m->next - delay + j + 2 * MPC_DELAY_MAX) % MPC_DELAY_MAX][side] : side_tilt(u, side, servo_block(j - delay));
            float x = r->tilt[side][j], v = r->rate[side][j];
            horn(c, goal, &x, &v, dt, r->clamped[side][j]);
            r->tilt[side][j + 1] = x;
            r->rate[side][j + 1] = v;
        }
    }
    memset(r->g, 0, sizeof(r->g));
    float roll = a->roll, pitch = a->pitch * c->pitch_gain, yaw = a->yaw, swing = c->assembly_ratio / dt;
    // The outputs' partials in each tick's thrusts, tilts and tilt rates (static, as this file's other large arrays: the flight task has 8 KB of stack).
    static float d_thrust[N][MPC_OUTPUTS][2], d_tilt[N][MPC_OUTPUTS][2];
    for (int k = 1; k <= N; k++) {
        float *y = &r->y[(k - 1) * MPC_OUTPUTS];
        float tl = r->thrust[0][k], tr = r->thrust[1][k], cl = cosf(r->tilt[0][k]), sl = sinf(r->tilt[0][k]), cr = cosf(r->tilt[1][k]), sr = sinf(r->tilt[1][k]);
        float accel = (r->rate[0][k] - r->rate[0][k - 1]) + (r->rate[1][k] - r->rate[1][k - 1]);
        y[0] = roll * (tl * cl - tr * cr);
        y[1] = pitch * (tl * sl + tr * sr) - swing * accel;
        y[2] = yaw * (tr * sr - tl * sl);
        y[3] = tl + tr;
        float (*dthrust)[2] = d_thrust[k - 1], (*dx)[2] = d_tilt[k - 1];
        dthrust[0][0] = roll * cl, dthrust[0][1] = -roll * cr, dx[0][0] = -roll * tl * sl, dx[0][1] = roll * tr * sr;
        dthrust[1][0] = pitch * sl, dthrust[1][1] = pitch * sr, dx[1][0] = pitch * tl * cl, dx[1][1] = pitch * tr * cr;
        dthrust[2][0] = -yaw * sl, dthrust[2][1] = yaw * sr, dx[2][0] = -yaw * tl * cl, dx[2][1] = yaw * tr * cr;
        dthrust[3][0] = 1, dthrust[3][1] = 1, dx[3][0] = 0, dx[3][1] = 0;
    }
    // Each motor block: its thrust's sensitivity, linear (one dead tick, then the lag).
    for (int side = 0; side < 2; side++) {
        for (int b = 0; b < MB; b++) {
            int col = motor_index(side, b);
            float s = 0;
            for (int j = 0; j < N; j++) {
                bool in = j >= 1 && motor_block(j - 1) == b;
                s = keep * s + (in ? 1 - keep : 0);
                for (int o = 0; o < MPC_OUTPUTS; o++) r->g[j * MPC_OUTPUTS + o][col] = d_thrust[j][o][side] * s;
            }
        }
    }
    // Each servo block: the horn's tangent along the trajectory (none through a substep at the slew limit), into the tilt's and the swing's partials.
    float h = dt / STEPS, w2 = c->servo_wn * c->servo_wn, z2 = 2 * c->servo_zeta * c->servo_wn;
    for (int side = 0; side < 2; side++) {
        int delay = delay_of(c, side);
        for (int b = 0; b < SB; b++) {
            // The side's tilt per unit of its own command, into the common's column as it is and the differential's with the side's sign.
            int common = servo_index(0, b), differential = servo_index(1, b);
            float sign = side ? 1 : -1, x = 0, v = 0;
            for (int j = 0; j < N; j++) {
                float goal = j >= delay && servo_block(j - delay) == b ? 1 : 0, before = v;
                for (int step = 0; step < STEPS; step++) {
                    v = r->clamped[side][j][step] ? 0 : v + (w2 * (goal - x) - z2 * v) * h;
                    x += v * h;
                }
                float *g = r->g[j * MPC_OUTPUTS];
                for (int o = 0; o < MPC_OUTPUTS; o++) {
                    float part = d_tilt[j][o][side] * x - (o == 1 ? swing * (v - before) : 0);
                    g[o * NU + common] += part;
                    g[o * NU + differential] += sign * part;
                }
            }
        }
    }
}

// Cholesky solve of the n by n system (row stride NU) in place: false if it is not positive definite.
static bool cholesky_solve(float *h, float *x, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
            float s = h[i * NU + j];
            for (int k = 0; k < j; k++) s -= h[i * NU + k] * h[j * NU + k];
            if (i == j) {
                if (s <= 1e-9f) return false;
                h[i * NU + i] = sqrtf(s);
            } else {
                h[i * NU + j] = s / h[j * NU + j];
            }
        }
    }
    for (int i = 0; i < n; i++) {
        float s = x[i];
        for (int k = 0; k < i; k++) s -= h[i * NU + k] * x[k];
        x[i] = s / h[i * NU + i];
    }
    for (int i = n - 1; i >= 0; i--) {
        float s = x[i];
        for (int k = i + 1; k < n; k++) s -= h[k * NU + i] * x[k];
        x[i] = s / h[i * NU + i];
    }
    return true;
}

// min ½ zᵀHz + gᵀz over low ≤ z ≤ high (H unit-diagonal after scaling): accelerated projected gradient from z (the warm start, inside the box), then the active set it ends on solved exactly, kept if it is feasible and its bounds' multipliers have the right sign.
static void solve(const float h[NU][NU], const float *g, const float *low, const float *high, int iterations, float *z, int *used, bool *polished) {
    float step = 0;
    for (int i = 0; i < NU; i++) {
        float row = 0;
        for (int j = 0; j < NU; j++) row += fabsf(h[i][j]);
        step = fmaxf(step, row);
    }
    step = 1 / step;
    float y[NU], last[NU], t = 1;
    memcpy(y, z, sizeof(y));
    int it = 0;
    for (; it < iterations; it++) {
        memcpy(last, z, sizeof(last));
        float moved = 0;
        for (int i = 0; i < NU; i++) {
            float grad = g[i];
            for (int j = 0; j < NU; j++) grad += h[i][j] * y[j];
            z[i] = clampf(y[i] - step * grad, low[i], high[i]);
            moved = fmaxf(moved, fabsf(z[i] - last[i]));
        }
        float next = (1 + sqrtf(1 + 4 * t * t)) / 2, momentum = (t - 1) / next;
        for (int i = 0; i < NU; i++) y[i] = z[i] + momentum * (z[i] - last[i]);
        t = next;
        if (moved < 1e-4f) {
            it++;
            break;
        }
    }
    *used = it;
    // The polish: the free variables solved with the bound ones held, up to three passes of moving a violator onto its bound.
    *polished = false;
    int bound[NU];
    for (int i = 0; i < NU; i++) bound[i] = z[i] <= low[i] + 1e-6f ? -1 : z[i] >= high[i] - 1e-6f ? 1 : 0;
    for (int pass = 0; pass < 3; pass++) {
        int free[NU], n = 0;
        static float sub[NU * NU];
        float x[NU], fixed[NU];
        for (int i = 0; i < NU; i++) {
            fixed[i] = bound[i] < 0 ? low[i] : bound[i] > 0 ? high[i] : 0;
            if (!bound[i]) free[n++] = i;
        }
        for (int a = 0; a < n; a++) {
            float s = -g[free[a]];
            for (int j = 0; j < NU; j++) s -= h[free[a]][j] * fixed[j];
            x[a] = s;
            for (int b = 0; b < n; b++) sub[a * NU + b] = h[free[a]][free[b]];
        }
        if (!cholesky_solve(sub, x, n)) return;
        float candidate[NU];
        memcpy(candidate, fixed, sizeof(candidate));
        bool feasible = true;
        for (int a = 0; a < n; a++) {
            candidate[free[a]] = x[a];
            if (x[a] < low[free[a]] - 1e-6f) bound[free[a]] = -1, feasible = false;
            else if (x[a] > high[free[a]] + 1e-6f) bound[free[a]] = 1, feasible = false;
        }
        if (!feasible) continue;
        // The bound ones' multipliers: the gradient must push each against its bound.
        bool optimal = true;
        for (int i = 0; i < NU; i++) {
            if (!bound[i]) continue;
            float grad = g[i];
            for (int j = 0; j < NU; j++) grad += h[i][j] * candidate[j];
            if ((bound[i] < 0 && grad < -1e-5f) || (bound[i] > 0 && grad > 1e-5f)) optimal = false;
        }
        if (optimal) {
            memcpy(z, candidate, sizeof(candidate));
            *polished = true;
        }
        return;
    }
}

void mpc_motor_response(const Mpc *m, const MpcConfig *c, const float (*asked)[2], float dt, float (*thrust)[2]) {
    float keep = expf(-dt / c->motor_tau);
    for (int side = 0; side < 2; side++) {
        float t = m->thrust[side];
        for (int j = 0; j < N; j++) {
            float now = j == 0 ? m->sent_thrust[side] : asked[j - 1][side];
            t = now + (t - now) * keep;
            thrust[j][side] = t;
        }
    }
}

void mpc_servo_response(const Mpc *m, const MpcConfig *c, float left, float right, float dt, float (*tilt)[2], float *swing) {
    float asked[2] = {left, right};
    for (int j = 0; j < N; j++) swing[j] = 0;
    for (int side = 0; side < 2; side++) {
        int delay = delay_of(c, side);
        float x = m->tilt[side], v = m->tilt_rate[side];
        for (int j = 0; j < N; j++) {
            float before = v;
            horn(c, j < delay ? m->sent_tilt[(m->next - delay + j + 2 * MPC_DELAY_MAX) % MPC_DELAY_MAX][side] : asked[side], &x, &v, dt, NULL);
            tilt[j][side] = x;
            swing[j] -= c->assembly_ratio * (v - before) / dt;
        }
    }
}

void mpc_allocate(Mpc *m, const MpcConfig *c, const FlipAuthority *a, const float (*target)[MPC_OUTPUTS], float floor, float ceiling, float dt, FlipOutput *out) {
    float u[NU], low[NU], high[NU], last[NU];
    ceiling = fmaxf(ceiling, floor);
    const float *sent = m->sent_tilt[(m->next - 1 + MPC_DELAY_MAX) % MPC_DELAY_MAX];
    last[servo_index(0, 0)] = (sent[0] + sent[1]) / 2;
    last[servo_index(1, 0)] = (sent[1] - sent[0]) / 2;
    int count = N - (delay_of(c, 0) > delay_of(c, 1) ? delay_of(c, 0) : delay_of(c, 1));
    for (int kind = 0; kind < 2; kind++) {
        for (int b = 0; b < SB; b++) {
            int i = servo_index(kind, b);
            high[i] = kind ? c->differential_max : c->common_max;
            low[i] = -high[i];
            u[i] = m->planned ? m->plan[servo_index(kind, servo_block(SERVO_START[b] + 1 < count ? SERVO_START[b] + 1 : count - 1))] : last[servo_index(kind, 0)];
        }
    }
    for (int side = 0; side < 2; side++) {
        last[motor_index(side, 0)] = m->sent_thrust[side];
        for (int b = 0; b < MB; b++) {
            int i = motor_index(side, b);
            low[i] = floor;
            high[i] = ceiling;
            // The warm start: the last plan a tick on (each block from the tick after its start), or what was sent, held.
            u[i] = m->planned ? m->plan[motor_index(side, motor_block(MOTOR_START[b] + 1 < N - 1 ? MOTOR_START[b] + 1 : N - 2))] : last[motor_index(side, 0)];
        }
    }
    for (int i = 0; i < NU; i++) u[i] = clampf(u[i], low[i], high[i]);
    // Each block's length in ticks, for the per-tick penalties.
    float length[NU];
    for (int side = 0; side < 2; side++) {
        for (int b = 0; b < MB; b++) length[motor_index(side, b)] = MOTOR_START[b + 1] - MOTOR_START[b];
        for (int b = 0; b < SB; b++) length[servo_index(side, b)] = (b + 1 < SB ? SERVO_START[b + 1] : count) - SERVO_START[b];
    }
    static Rollout r;
    int passes = c->passes < 1 ? 1 : c->passes;
    for (int pass = 0; pass < passes; pass++) {
        rollout(m, c, a, u, dt, &r);
        static float h[NU][NU];
        float g[NU];
        memset(h, 0, sizeof(h));
        memset(g, 0, sizeof(g));
        // The tracking: Gᵀ W G and Gᵀ W (y - wanted).
        for (int k = 0; k < N; k++) {
            for (int o = 0; o < MPC_OUTPUTS; o++) {
                const float *row = r.g[k * MPC_OUTPUTS + o];
                float w = c->weight[o] * ((o == 1 || o == 2) && k < N - count ? c->early : 1), e = w * (r.y[k * MPC_OUTPUTS + o] - target[k][o]);
                for (int i = 0; i < NU; i++) {
                    if (row[i] == 0) continue;
                    float wi = w * row[i];
                    g[i] += e * row[i];
                    for (int j = i; j < NU; j++) h[i][j] += wi * row[j];
                }
            }
        }
        // The penalties, per tick of each block: the common and the differential tilt's size, and each move from the block before (the first from what was sent).
        for (int kind = 0; kind < 2; kind++) {
            for (int b = 0; b < SB; b++) {
                int i = servo_index(kind, b);
                float w = (kind ? c->differential_effort : c->tilt_effort) * length[i];
                h[i][i] += w;
                g[i] += w * u[i];
            }
        }
        for (int side = 0; side < 2; side++) {
            for (int kind = 0; kind < 2; kind++) {
                int blocks = kind ? SB : MB;
                float w = kind ? c->tilt_slew : c->thrust_slew;
                for (int b = 0; b < blocks; b++) {
                    int i = kind ? servo_index(side, b) : motor_index(side, b);
                    float before = b ? u[i - 1] : last[kind ? servo_index(side, 0) : motor_index(side, 0)], d = u[i] - before;
                    h[i][i] += w;
                    g[i] += w * d;
                    if (b) {
                        h[i - 1][i - 1] += w;
                        h[i - 1][i] -= w;
                        g[i - 1] -= w * d;
                    }
                }
            }
        }
        for (int i = 0; i < NU; i++)
            for (int j = 0; j < i; j++) h[i][j] = h[j][i];
        // Jacobi scaling: z = δ / d, d = 1 / √H_ii.
        float d[NU], z[NU], zl[NU], zh[NU], gs[NU];
        for (int i = 0; i < NU; i++) d[i] = h[i][i] > 1e-9f ? 1 / sqrtf(h[i][i]) : 1;
        for (int i = 0; i < NU; i++) {
            for (int j = 0; j < NU; j++) h[i][j] *= d[i] * d[j];
            gs[i] = g[i] * d[i];
            zl[i] = (low[i] - u[i]) / d[i];
            zh[i] = (high[i] - u[i]) / d[i];
            z[i] = 0;
        }
        solve((const float (*)[NU])h, gs, zl, zh, c->iterations, z, &m->iterations, &m->polished);
        float cost = 0;
        for (int i = 0; i < NU; i++) {
            float hz = 0;
            for (int j = 0; j < NU; j++) hz += h[i][j] * z[j];
            cost += z[i] * (0.5f * hz + gs[i]);
            u[i] = clampf(u[i] + d[i] * z[i], low[i], high[i]);
        }
        m->cost = cost;
    }
    memcpy(m->plan, u, sizeof(m->plan));
    m->planned = m->fresh = true;
    out->left = u[motor_index(0, 0)];
    out->right = u[motor_index(1, 0)];
    out->left_tilt = side_tilt(u, 0, 0);
    out->right_tilt = side_tilt(u, 1, 0);
    out->collective = out->left + out->right;
}
