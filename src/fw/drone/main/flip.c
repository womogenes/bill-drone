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
    .flip_thrust = 1.4874f, .right_angle = 0.656f, .servo_max = 0.8003f, .kv = 7.9681f, .arrest_time = 2.0f, .timeout = 2.0f,
};

static float clampf(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

void flip_start(Flip *flip, float hover_thrust) {
    *flip = (Flip){.stage = FLIP_CLIMB, .thrust = hover_thrust};
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

void flip_step(Flip *flip, const FlipParams *p, const FlipAuthority *a, const float *gyro, float vertical_g, float dt, FlipOutput *out) {
    *out = (FlipOutput){0};
    if (flip->stage == FLIP_IDLE) return;
    flip->time += dt;
    flip->vertical += (vertical_g - 1) * GRAVITY * dt;
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
    flip->turned += gyro[0] * dt;
    out->owned = true;
    if (flip->stage == FLIP_SPIN) {
        allocate(flip, p, a, p->k_spin * (p->spin_rate - gyro[0]), -p->k_spin * gyro[1], -p->k_spin * 0.3f * gyro[2], p->spin_thrust * a->weight, dt, out);
        if (flip->turned >= TWO_PI - p->handoff) flip->stage = FLIP_BRAKE;
    } else {
        // What is left of the turn to level, braked as fast as the drone can stop from it.
        float angle = fmaxf(TWO_PI - flip->turned, 0);
        float rate = fminf(fminf(p->max_rate, sqrtf(2 * p->brake * angle)), p->kp * angle);
        allocate(flip, p, a, p->k_roll * (rate - gyro[0]), -p->k_pitch * gyro[1], -p->kd_yaw * gyro[2], p->flip_thrust * a->weight, dt, out);
    }
    // Back to the attitude controller once the thrust is within right_angle of up, or on the timeout.
    if ((flip->stage == FLIP_BRAKE && TWO_PI - flip->turned < p->right_angle) || flip->time > p->timeout) {
        flip->landing_roll = flip->turned - TWO_PI;
        flip->stage = FLIP_ARREST;
        flip->time = 0;
        out->owned = false;
        out->handed_back = true;
        out->collective_set = true;
        out->collective = arrest(p, a, flip->vertical);
    }
}
