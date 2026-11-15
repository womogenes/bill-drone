// mpc.c's checks and timing on the host: the rollout's Jacobian against finite differences, the solve against a long projected-gradient run, and the time a call takes. cc -O2 -iquote ../../../fw/drone/main test_mpc.c -lm -o test_mpc && ./test_mpc (from sw/sim/mpc).
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "../../../fw/drone/main/mpc.c"

static double now(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

// A state like the burst's brake: the servos 12 degrees back and slewing forward, the right motor rising.
static void brake_state(Mpc *m, const MpcConfig *c) {
    mpc_reset(m);
    float tilt[2] = {-0.35f, -0.3f}, thrust[2] = {4.0f, 5.0f};
    for (int k = 0; k < 20; k++) mpc_sent(m, c, tilt, thrust, 0.01f);
    for (int k = 0; k < 4; k++) {
        float t2[2] = {-0.35f + 0.08f * k, -0.3f + 0.08f * k}, f2[2] = {4.0f, 5.0f + 1.5f * k};
        mpc_sent(m, c, t2, f2, 0.01f);
    }
}

int main(void) {
    MpcConfig c = mpc_defaults;
    FlipAuthority a = {13.5f, 7.08f, 4.66f, 10.0f, {0, 0, 0}, {0, 0, 0}};
    Mpc m;
    brake_state(&m, &c);
    float u[NU];
    for (int i = 0; i < NU; i++) u[i] = i < 2 * MB ? 6.0f + 0.3f * i : 0.05f * (i - 2 * MB) - 0.2f;
    static Rollout base, moved;
    rollout(&m, &c, &a, u, 0.01f, &base);
    double worst = 0, scale = 0;
    for (int col = 0; col < NU; col++) {
        float step = col < 2 * MB ? 1e-2f : 1e-3f, v[NU];
        memcpy(v, u, sizeof(v));
        v[col] += step;
        rollout(&m, &c, &a, v, 0.01f, &moved);
        for (int row = 0; row < NY; row++) {
            double fd = (moved.y[row] - base.y[row]) / step;
            worst = fmax(worst, fabs(fd - base.g[row][col]));
            scale = fmax(scale, fabs(fd));
        }
    }
    printf("Jacobian: largest finite-difference mismatch %.3g against a largest entry %.3g\n", worst, scale);

    float wanted[MPC_HORIZON][MPC_OUTPUTS];
    for (int k = 0; k < MPC_HORIZON; k++) wanted[k][0] = -60, wanted[k][1] = 5, wanted[k][2] = 0, wanted[k][3] = 11;
    FlipOutput out;
    int sizes[] = {10, 30, 100, 2000};
    for (int s = 0; s < 4; s++) {
        c.iterations = sizes[s];
        brake_state(&m, &c);
        mpc_allocate(&m, &c, &a, (const float (*)[MPC_OUTPUTS])wanted, 2.6f, 12.3f, 0.01f, &out);
        printf("iterations %4d: used %3d polished %d cost %.4f; thrusts %.2f %.2f N, tilts %.1f %.1f deg\n", sizes[s], m.iterations, m.polished, m.cost, out.left, out.right, out.left_tilt * 57.3f, out.right_tilt * 57.3f);
    }

    // The time a call takes from a warm start, as the flip calls it every tick.
    c = mpc_defaults;
    brake_state(&m, &c);
    int calls = 20000;
    double start = now();
    long iterations = 0, polished = 0;
    for (int k = 0; k < calls; k++) {
        float w = -60 + 120.0f * (k % 50) / 50;
        for (int j = 0; j < MPC_HORIZON; j++) wanted[j][0] = w;
        mpc_allocate(&m, &c, &a, (const float (*)[MPC_OUTPUTS])wanted, 2.6f, 12.3f, 0.01f, &out);
        iterations += m.iterations;
        polished += m.polished;
        float tilt[2] = {out.left_tilt, out.right_tilt}, thrust[2] = {out.left, out.right};
        mpc_sent(&m, &c, tilt, thrust, 0.01f);
    }
    double each = (now() - start) / calls;
    printf("per call %.1f us (%d calls; %.1f iterations, %.0f%% polished)\n", each * 1e6, calls, (double)iterations / calls, 100.0 * polished / calls);
    return 0;
}
