#include "flip.h"

#include <math.h>

#define TWO_PI 6.28318531f
// How fast the flip's idea of the motors' thrust follows what it asks (s; the motors' lag).
#define FLIP_MOTOR_TAU 0.1f
#define GRAVITY 9.81f
// The arrest ends once the vertical speed has stayed under ARREST_DONE (m/s) for ARREST_HOLD (s).
#define ARREST_DONE 0.2f
#define ARREST_HOLD 0.3f

const FlipParams flip_defaults = {
    .climb_thrust = 2.1362f, .climb_time = 0.1064f, .spin_rate = 15.9605f, .k_spin = 5.322f, .spin_thrust = 0.4672f, .handoff = 1.9173f,
    .kp = 2.4309f, .brake = 15.4237f, .max_rate = 10.3931f, .k_roll = 7.4738f, .k_pitch = 2.1245f, .kd_yaw = 2.005f,
    .flip_thrust = 1.4874f, .right_angle = 0.656f, .servo_max = 0.8003f, .roll_over = 2.8798f, .kv = 7.9681f, .arrest_time = 2.0f, .timeout = 2.0f,
};

static float clampf(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

void flip_start(Flip *flip, float hover_thrust, float roll, float pitch) {
    // flight_attitude's roll and pitch (about forward, then left; positive right side down and nose down) as a quaternion, the heading zero.
    float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    *flip = (Flip){.stage = FLIP_CLIMB, .q = {cr * cp, sr * cp, cr * sp, -sr * sp}, .thrust = hover_thrust};
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

// Wanted angular accelerations into thrusts and tilts (recover.py's allocation): roll by differential thrust, pitch by common tilt, yaw by differential tilt, from the thrust the motors have by now.
static void allocate(Flip *flip, const FlipParams *p, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float dt, FlipOutput *out) {
    float diff = roll / a->roll;
    float total = fmaxf(collective, fabsf(diff));
    out->left = (total + diff) / 2;
    out->right = (total - diff) / 2;
    flip->thrust += (total - flip->thrust) * (1 - expf(-dt / FLIP_MOTOR_TAU));
    float have = fmaxf(flip->thrust, 0.5f), limit = sinf(p->servo_max);
    float common = asinf(clampf(pitch / (a->pitch * have), -limit, limit));
    float split = clampf(yaw / (a->yaw * have * cosf(common)), -0.3f, 0.3f);
    out->left_tilt = common - split;
    out->right_tilt = common + split;
    out->collective = total;
}

// The collective that stops the vertical speed.
static float arrest(const FlipParams *p, const FlipAuthority *a, float vertical) {
    return clampf(a->weight * (1 - p->kv * vertical / GRAVITY), 0.15f * a->weight, 2.2f * a->weight);
}

void flip_step(Flip *flip, const FlipParams *p, const FlipAuthority *a, const float *gyro, const float *accel, float dt, FlipOutput *out) {
    *out = (FlipOutput){0};
    if (flip->stage == FLIP_IDLE) return;
    flip->time += dt;
    float *q = flip->q;
    turn(q, gyro, dt);
    // Up in body axes (the level frame's z row), the body's forward and left axes' height.
    float up[3] = {2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])};
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
    if (flip->stage == FLIP_SPIN) {
        flip->turned += gyro[0] * dt;
        // Keep the roll axis level: the nose's height (up's forward component, positive nose up) driven to zero through whichever of pitch and yaw tilts it now.
        float nose = up[0];
        float pitch_rate = p->kp * nose * up[2], yaw_rate = -p->kp * nose * up[1];
        allocate(flip, p, a, p->k_spin * (p->spin_rate - gyro[0]), p->k_spin * (pitch_rate - gyro[1]), p->k_spin * 0.3f * (yaw_rate - gyro[2]), p->spin_thrust * a->weight, dt, out);
        if (flip->turned >= TWO_PI - p->handoff) flip->stage = FLIP_RIGHT;
    } else {
        // recover.py's righting: turn the thrust axis toward up about the axis between them, no faster than it can brake from; far over with the error mostly in pitch, roll the long way round.
        float sine = hypotf(up[0], up[1]), angle = atan2f(sine, up[2]);
        float axis[2] = {sine > 1e-6f ? -up[1] / sine : 1, sine > 1e-6f ? up[0] / sine : 0};
        if (up[2] < cosf(p->roll_over) && fabsf(up[0]) > fabsf(up[1])) {
            angle = atan2f(fabsf(up[1]), up[2]);
            axis[0] = up[1] > 0 ? -1 : 1;
            axis[1] = 0;
        }
        float rate = fminf(fminf(p->max_rate, sqrtf(2 * p->brake * angle)), p->kp * angle);
        allocate(flip, p, a, p->k_roll * (axis[0] * rate - gyro[0]), p->k_pitch * (axis[1] * rate - gyro[1]), -p->kd_yaw * gyro[2], p->flip_thrust * a->weight, dt, out);
        if (up[2] > cosf(p->right_angle) || flip->time > p->timeout) {
            // Back to the attitude controller, with the attitude as flight_attitude keeps it.
            flip->stage = FLIP_ARREST;
            flip->time = 0;
            out->owned = false;
            out->handed_back = true;
            out->roll = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
            out->pitch = asinf(clampf(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1));
            out->collective_set = true;
            out->collective = arrest(p, a, flip->vertical);
        }
    }
    if (flip->stage == FLIP_SPIN && flip->time > p->timeout) flip->stage = FLIP_RIGHT;
}
