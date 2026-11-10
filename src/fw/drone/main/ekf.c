#include <math.h>
#include <string.h>

#include "ekf.h"

#define GRAVITY 9.81f
#define PI 3.14159265f

// Where each state's error sits in the covariance.
enum { TILT = 0, VELOCITY = 3, POSITION = 6, GYRO_BIAS = 9, ACCEL_BIAS = 12, BARO_OFFSET = 15, BARO_TRANSIENT = 16 };

// What the IMU's readings are worth: the gyro's and accelerometer's noise (rad/s and m/s² per root Hz; the accelerometer's is the airframe's vibration, 1 m/s² a sample at 100 Hz as sw/sim/gps_truth.py) and how fast their biases and the barometer's offset wander (per root second).
#ifndef EKF_GYRO_NOISE
#define EKF_GYRO_NOISE 0.002f
#endif
#ifndef EKF_ACCEL_NOISE
#define EKF_ACCEL_NOISE 0.1f
#endif
#ifndef EKF_GYRO_WALK
#define EKF_GYRO_WALK 1e-4f
#endif
#ifndef EKF_ACCEL_WALK
#define EKF_ACCEL_WALK 0.01f
#endif
#ifndef EKF_BARO_WALK
#define EKF_BARO_WALK 0.03f
#endif
// The start's uncertainty: tilt (rad), velocity (m/s), gyro bias (rad/s), accelerometer bias across and along the thrust axis (m/s²).
#define START_TILT 0.05f
#define START_VELOCITY 0.1f
#define START_GYRO_BIAS 0.005f
#define START_ACCEL_BIAS 0.1f
#define START_ACCEL_BIAS_UP 0.3f
#define MAX_GYRO_BIAS 0.1f
#define MAX_ACCEL_BIAS 1.5f
// A solution's standard deviations: horizontal velocity (m/s), climb rate (m/s; near the ground with the motors on the receiver read 0.5-1.2 m/s down through the liftoffs of 2026-11-09), position (m; its error wanders over many solutions, so each is worth less than its scatter says).
#ifndef EKF_GPS_VELOCITY_SD
#define EKF_GPS_VELOCITY_SD 0.1f
#endif
#ifndef EKF_GPS_CLIMB_SD
#define EKF_GPS_CLIMB_SD 0.3f
#endif
#ifndef EKF_GPS_CLIMB_NEAR_SD
#define EKF_GPS_CLIMB_NEAR_SD 1.5f
#endif
#ifndef EKF_GPS_POSITION_SD
#define EKF_GPS_POSITION_SD 2.0f
#endif
// The barometer's (m), and near the ground with the motors on (under EKF_NEAR m above where the drone last rested), where the props' wash moves the pressure: 0.3-2.9 m either way for the first 0.4 s after the liftoffs of 2026-11-09 (hover.c take_baro), and for all of it at once, so each of those readings is worth little.
#ifndef EKF_BARO_SD
#define EKF_BARO_SD 0.12f
#endif
#ifndef EKF_BARO_NEAR_SD
#define EKF_BARO_NEAR_SD 2.5f
#endif
#ifndef EKF_NEAR
#define EKF_NEAR 0.5f
#endif
// The barometer's transient: the air moving over the drone shifts the pressure it reads for a few tenths of a second at a time, which no height change explains (2026-11-09 05:53 UTC at 24 s: 1.2 m high for 0.5 s banked 20-30 degrees and sinking at 0.7 m/s by the GPS and the accelerometer both). It fades over EKF_BARO_TRANSIENT_S (s) and is as wide as EKF_BARO_TRANSIENT_SD (m) plus EKF_BARO_TILTED (m per rad² of tilt) with the motors on: over the 17 flights of 2026-11-09 the barometer less the GPS climb rate's integral, each less its own 2 s median, was 0.09 m RMS under 10 degrees of tilt, 0.16 at 17-23 and 0.30 past 23.
#ifndef EKF_BARO_TRANSIENT_S
#define EKF_BARO_TRANSIENT_S 0.5f
#endif
#ifndef EKF_BARO_TRANSIENT_SD
#define EKF_BARO_TRANSIENT_SD 0.1f
#endif
#ifndef EKF_BARO_TILTED
#define EKF_BARO_TILTED 1.6f
#endif
// A measurement further than this many standard deviations from the state is refused. EKF_REJECTS GPS solutions refused in a row (1.5 s of them) reset the horizontal velocity to the GPS and loosen the tilt, whose error is what most often carries the velocity off (RESET_TILT, rad), and the heading (RESET_YAW); twice within RESET_US the heading is searched for again. As many position fixes refused move home to them. EKF_BARO_REJECTS barometer readings refused (2 s) in the air loosen the height by RESET_HEIGHT (m) for the barometer to set, since the barometer is all that knows the height for long; on the ground they move the offset.
#define GATE 5.0f
#ifndef EKF_REJECTS
#define EKF_REJECTS 15
#endif
#ifndef EKF_BARO_REJECTS
#define EKF_BARO_REJECTS 100
#endif
#define RESET_HEIGHT 2.0f
#define RESET_US 10000000
#define RESET_YAW 0.26f
#define RESET_TILT 0.17f
#define RESET_VELOCITY 2.0f
#define RESET_POSITION 10.0f
// Rest. Still: motors off, turning under STILL_GYRO (rad/s) and weighing within STILL_ACCEL (m/s²) of gravity for STILL_S. Then the velocity is zero (REST_SD, m/s), the position is where it came to rest (LATCH_SD, m) and the gyro reads its own bias (STILL_GYRO_SD, rad/s; loose, since a drone held steady in the hand passes for still). The GPS's position there is averaged over HOME_TAU (s), a stir of under REST_KEPT (s) before the motors start keeps the resting place, and the ground's height follows the position over GROUND_TAU (s) whenever the motors are off.
#define STILL_GYRO 0.03f
#define STILL_ACCEL 0.5f
#define STILL_S 1.0f
#ifndef EKF_REST_SD
#define EKF_REST_SD 0.02f
#endif
#define LATCH_SD 0.05f
#define STILL_GYRO_SD 0.02f
#define HOME_TAU 5.0f
#define REST_KEPT 5.0f
#define GROUND_TAU 1.0f
// Without the GPS's horizontal velocity (heading not yet known, no solution for GPS_STALE_US, indoors) the horizontal specific force is taken as zero, which over time it is: the accelerometer's pull on the attitude in flight_math.c. Its standard deviation (m/s²) with the motors off, where only carrying accelerates it, and on, where the drone's own accelerations do. The horizontal velocity, which nothing measures then, is held within EKF_CARRY_SD or EKF_FREE_SD (m/s) of zero once a second so its uncertainty stays finite.
#define GPS_STALE_US 500000
#ifndef EKF_LEVEL_SD
#define EKF_LEVEL_SD 0.6f
#endif
#ifndef EKF_LEVEL_FLYING_SD
#define EKF_LEVEL_FLYING_SD 10.0f
#endif
#define EKF_CARRY_SD 1.5f
#define EKF_FREE_SD 5.0f
// Flown by the hold (ekf_estimate): the horizontal velocity's standard deviation up to which the hold flies the filter's heading, velocity and travel (m/s), and the height's for its height and climb rate (m). With the GPS the velocity's is 0.08-0.1; without it 0.3 after 2 s, 0.45 after 3 s and 0.7 after 5 s (2026-11-09 05:53 and 06:38 UTC replays, its real error much the same), so the hold goes back to its own estimates, which the drag model and the servos bound, some 2 s into a GPS outage. In the sim (sw/sim/outdoor.py --gps-outage 8,5) the two velocities' errors grow alike for 3 s (to 0.4 m/s) and the filter's faster after (0.65 against 0.5 m/s at 5 s, 90th percentile 1.5 against 0.8); handing back at 0.5 m/s let the outage carry the drone 6.0 m from takeoff at the 90th percentile against 4.5 on the hold's own. The height's is 0.1-0.2 in flight.
#define EKF_FLOWN_VELOCITY_SD 0.3f
#define EKF_FLOWN_HEIGHT_SD 0.5f
// The heading search, as hover.c learn_heading: GPS velocity and the state's own averaged over blocks, each pair of blocks' changes multiplied, the products kept over HEADING_TAU (s). Sure once their scatter puts the angle within HEADING_SURE (rad) with HEADING_BLOCKS products' worth of evidence; HEADING_NOISE is the scatter's floor as a share of the changes, and HEADING_FLOOR the least uncertainty the pinned heading starts with (rad).
#define HEADING_BLOCK_US 400000
#define HEADING_GAP_US 500000
#define HEADING_TAU 30.0f
#define HEADING_SURE 0.1745f
#define HEADING_BLOCKS 2
#define HEADING_NOISE 0.08f
#define HEADING_STEADY_TAU 5.0f
#define HEADING_STEADY_PRIOR 3.0f
#define HEADING_FLOOR 0.05f
#define MAX_DT 0.05f

static float clamp(float value, float low, float high) {
    return value < low ? low : value > high ? high : value;
}

// a ⊗ b into out (which may be either), normalized.
static void multiply(const float *a, const float *b, float *out) {
    float q[4] = {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    };
    float size = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) out[i] = q[i] / size;
}

static void rotation(const float *q, float R[3][3]) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z);
    R[0][1] = 2 * (x * y - w * z);
    R[0][2] = 2 * (x * z + w * y);
    R[1][0] = 2 * (x * y + w * z);
    R[1][1] = 1 - 2 * (x * x + z * z);
    R[1][2] = 2 * (y * z - w * x);
    R[2][0] = 2 * (x * z - w * y);
    R[2][1] = 2 * (y * z + w * x);
    R[2][2] = 1 - 2 * (x * x + y * y);
}

// One tick of dead reckoning: the attitude turned by the gyro, and the velocity and position moved by the specific force (the mean of it turned by the attitude before and after) and gravity. R is the attitude after and force that mean (world axes, m/s²).
static void advance(float *q, float *v, float *p, const float *gyro_bias, const float *accel_bias, const EkfSample *sample, float dt, float R[3][3], float *force) {
    float a[3], before[3][3], turn[4] = {1, 0, 0, 0}, angle = 0;
    for (int i = 0; i < 3; i++) {
        a[i] = sample->accel[i] - accel_bias[i];
        turn[1 + i] = (sample->gyro[i] - gyro_bias[i]) * dt / 2;
        angle += turn[1 + i] * turn[1 + i];
    }
    turn[0] = 1 - angle / 2;
    rotation(q, before);
    multiply(q, turn, q);
    rotation(q, R);
    for (int i = 0; i < 3; i++) {
        force[i] = 0;
        for (int j = 0; j < 3; j++) force[i] += 0.5f * (before[i][j] + R[i][j]) * a[j];
        float acceleration = force[i] - (i == 2 ? GRAVITY : 0);
        p[i] += v[i] * dt + 0.5f * acceleration * dt * dt;
        v[i] += acceleration * dt;
    }
}

// The state forward one tick and its covariance with it, P = F P Fᵀ + Q. F is the identity but for four blocks (tilt from gyro bias, velocity from tilt and from accelerometer bias, position from velocity), so it is applied in place: rows then columns, each in an order that reads only what is not yet changed.
static void predict(Ekf *ekf, const EkfSample *sample, float dt) {
    float R[3][3], (*P)[EKF_STATES] = ekf->P;
    advance(ekf->q, ekf->v, ekf->p, ekf->gyro_bias, ekf->accel_bias, sample, dt, R, ekf->force);
    const float *f = ekf->force;
    // The velocity's error from a tilt error: tilt × force.
    float cross[3][3] = {{0, f[2], -f[1]}, {-f[2], 0, f[0]}, {f[1], -f[0], 0}};
    for (int c = 0; c < EKF_STATES; c++) {
        for (int i = 0; i < 3; i++) P[POSITION + i][c] += dt * P[VELOCITY + i][c];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[VELOCITY + i][c] += dt * (cross[i][j] * P[TILT + j][c] - R[i][j] * P[ACCEL_BIAS + j][c]);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[TILT + i][c] -= dt * R[i][j] * P[GYRO_BIAS + j][c];
    }
    for (int r = 0; r < EKF_STATES; r++) {
        for (int i = 0; i < 3; i++) P[r][POSITION + i] += dt * P[r][VELOCITY + i];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[r][VELOCITY + i] += dt * (cross[i][j] * P[r][TILT + j] - R[i][j] * P[r][ACCEL_BIAS + j]);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) P[r][TILT + i] -= dt * R[i][j] * P[r][GYRO_BIAS + j];
    }
    for (int i = 0; i < 3; i++) {
        P[TILT + i][TILT + i] += EKF_GYRO_NOISE * EKF_GYRO_NOISE * dt;
        P[VELOCITY + i][VELOCITY + i] += EKF_ACCEL_NOISE * EKF_ACCEL_NOISE * dt;
        P[GYRO_BIAS + i][GYRO_BIAS + i] += EKF_GYRO_WALK * EKF_GYRO_WALK * dt;
        P[ACCEL_BIAS + i][ACCEL_BIAS + i] += EKF_ACCEL_WALK * EKF_ACCEL_WALK * dt;
    }
    P[BARO_OFFSET][BARO_OFFSET] += EKF_BARO_WALK * EKF_BARO_WALK * dt;
    // The barometer's transient fades, and is driven so that left alone it settles as wide as the tilt (R's last term is its cosine) says.
    float keep = 1 - dt / EKF_BARO_TRANSIENT_S, wide = EKF_BARO_TRANSIENT_SD + (sample->spinning ? EKF_BARO_TILTED * 2 * (1 - R[2][2]) : 0);
    ekf->baro_transient *= keep;
    for (int i = 0; i < EKF_STATES; i++) {
        P[BARO_TRANSIENT][i] *= keep;
        P[i][BARO_TRANSIENT] *= keep;
    }
    P[BARO_TRANSIENT][BARO_TRANSIENT] += 2 * wide * wide * dt / EKF_BARO_TRANSIENT_S;
    // Rounding leaves the two halves a little apart.
    for (int i = 0; i < EKF_STATES; i++)
        for (int j = i + 1; j < EKF_STATES; j++) P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
}

// One measurement of a combination of states (count of them, at the indexes at with the weights h): innovation is the measured value less the state's, sd the measurement's standard deviation. Refused (false) when the innovation is more than gate standard deviations of what the state and the measurement allow between them; ratio, if given, receives the innovation in those.
static bool fuse(Ekf *ekf, int count, const int *at, const float *h, float innovation, float sd, float gate, float *ratio) {
    float ph[EKF_STATES], variance = sd * sd;
    for (int i = 0; i < EKF_STATES; i++) {
        ph[i] = 0;
        for (int k = 0; k < count; k++) ph[i] += ekf->P[i][at[k]] * h[k];
    }
    for (int k = 0; k < count; k++) variance += h[k] * ph[at[k]];
    if (ratio) *ratio = innovation / sqrtf(variance);
    if (gate > 0 && innovation * innovation > gate * gate * variance) return false;
    float change[EKF_STATES];
    for (int i = 0; i < EKF_STATES; i++) {
        float gain = ph[i] / variance;
        change[i] = gain * innovation;
        for (int j = 0; j < EKF_STATES; j++) ekf->P[i][j] -= gain * ph[j];
    }
    // The correction into the state: the tilt's as a small rotation in world axes.
    float turn[4] = {1, change[TILT] / 2, change[TILT + 1] / 2, change[TILT + 2] / 2};
    multiply(turn, ekf->q, ekf->q);
    for (int i = 0; i < 3; i++) {
        ekf->v[i] += change[VELOCITY + i];
        ekf->p[i] += change[POSITION + i];
        ekf->gyro_bias[i] = clamp(ekf->gyro_bias[i] + change[GYRO_BIAS + i], -MAX_GYRO_BIAS, MAX_GYRO_BIAS);
        ekf->accel_bias[i] = clamp(ekf->accel_bias[i] + change[ACCEL_BIAS + i], -MAX_ACCEL_BIAS, MAX_ACCEL_BIAS);
    }
    ekf->baro_offset += change[BARO_OFFSET];
    ekf->baro_transient += change[BARO_TRANSIENT];
    return true;
}

// A measurement of one state.
static bool fuse_one(Ekf *ekf, int at, float innovation, float sd, float gate, float *ratio) {
    float one = 1;
    return fuse(ekf, 1, &at, &one, innovation, sd, gate, ratio);
}

// One state's error made independent of the others, with this standard deviation.
static void forget(Ekf *ekf, int at, float sd) {
    for (int i = 0; i < EKF_STATES; i++) ekf->P[at][i] = ekf->P[i][at] = 0;
    ekf->P[at][at] = sd * sd;
}

// The world's axes turned about up by angle (rad, counterclockwise): the attitude, the horizontal velocity and position, where the drone rested, and their errors' covariance with them.
static void turn_world(Ekf *ekf, float angle) {
    float c = cosf(angle), s = sinf(angle), (*P)[EKF_STATES] = ekf->P;
    float about[4] = {cosf(angle / 2), 0, 0, sinf(angle / 2)};
    multiply(about, ekf->q, ekf->q);
    float *vectors[3] = {ekf->v, ekf->p, ekf->latch};
    for (int k = 0; k < 3; k++) {
        float x = vectors[k][0], y = vectors[k][1];
        vectors[k][0] = c * x - s * y;
        vectors[k][1] = s * x + c * y;
    }
    for (int block = 0; block < 9; block += 3) {
        for (int i = 0; i < EKF_STATES; i++) {
            float x = P[block][i], y = P[block + 1][i];
            P[block][i] = c * x - s * y;
            P[block + 1][i] = s * x + c * y;
        }
        for (int i = 0; i < EKF_STATES; i++) {
            float x = P[i][block], y = P[i][block + 1];
            P[i][block] = c * x - s * y;
            P[i][block + 1] = s * x + c * y;
        }
    }
}

static void restart_search(Ekf *ekf) {
    ekf->block_count = 0;
    ekf->block_us = 0;
    ekf->search_us = 0;
    memset(ekf->steady, 0, sizeof(ekf->steady));
    ekf->steady_weight = HEADING_STEADY_PRIOR;
    ekf->turn[0] = ekf->turn[1] = ekf->turn_weight = ekf->turn_square[0] = ekf->turn_square[1] = ekf->turn_inertial = 0;
    ekf->heading_sd = PI;
}

// The heading search (see hover.c learn_heading, whose arithmetic this is): true once the evidence is sure, with the angle the world's axes are to be turned by in evidence. Velocities as north + i west, so the GPS's change times the conjugate of the state's has that angle. The state's is the specific force summed (reckoned), not its velocity, which the level measurement holds near zero.
static bool search(Ekf *ekf, const HoverGps *fix) {
    if (!ekf->search_us || fix->capture_us - ekf->search_us > HEADING_GAP_US) {
        ekf->block_count = 0;
        ekf->block_us = 0;
        ekf->reckoned[0] = ekf->reckoned[1] = 0;
        memset(ekf->steady, 0, sizeof(ekf->steady));
        ekf->steady_weight = HEADING_STEADY_PRIOR;
    }
    ekf->search_us = fix->capture_us;
    if (!ekf->block_count) {
        memset(ekf->block, 0, sizeof(ekf->block));
        ekf->block_start_us = fix->capture_us;
    }
    float values[4] = {fix->velocity[0], -fix->velocity[1], ekf->reckoned[0], ekf->reckoned[1]};
    for (int i = 0; i < 4; i++) ekf->block[i] += values[i];
    ekf->block_count++;
    if (fix->capture_us - ekf->block_start_us < HEADING_BLOCK_US) return false;
    float mean[4];
    for (int i = 0; i < 4; i++) mean[i] = ekf->block[i] / ekf->block_count;
    int64_t middle = (ekf->block_start_us + fix->capture_us) / 2;
    bool sure = false;
    if (ekf->block_us) {
        float span = (middle - ekf->block_us) / 1e6f, keep = expf(-span / HEADING_TAU);
        // Each change less its steady part, as hover.c's (HEADING_STEADY_TAU there).
        float change[4];
        ekf->steady_weight = ekf->steady_weight * expf(-span / HEADING_STEADY_TAU) + 1;
        float share = 1 / ekf->steady_weight, scale = sqrtf(1 - share);
        for (int i = 0; i < 4; i++) {
            float beyond = mean[i] - ekf->last[i] - ekf->steady[i];
            ekf->steady[i] += beyond * share;
            change[i] = beyond * scale;
        }
        float cn = change[0], cw = change[1], ax = change[2], ay = change[3];
        float x = cn * ax + cw * ay, y = cw * ax - cn * ay;
        ekf->turn[0] = ekf->turn[0] * keep + x;
        ekf->turn[1] = ekf->turn[1] * keep + y;
        ekf->turn_weight = ekf->turn_weight * keep * keep + x * x + y * y;
        ekf->turn_square[0] = ekf->turn_square[0] * keep * keep + x * x - y * y;
        ekf->turn_square[1] = ekf->turn_square[1] * keep * keep + 2 * x * y;
        ekf->turn_inertial = ekf->turn_inertial * keep * keep + ax * ax + ay * ay;
        float size = hypotf(ekf->turn[0], ekf->turn[1]);
        ekf->evidence = atan2f(ekf->turn[1], ekf->turn[0]);
        float across = 0.5f * (ekf->turn_weight - (ekf->turn_square[0] * cosf(2 * ekf->evidence) + ekf->turn_square[1] * sinf(2 * ekf->evidence)));
        across = fmaxf(across, HEADING_NOISE * HEADING_NOISE * ekf->turn_inertial);
        ekf->heading_sd = size > 0 ? fminf(sqrtf(fmaxf(across, 0)) / size, PI) : PI;
        sure = size * size >= HEADING_BLOCKS * ekf->turn_weight && ekf->heading_sd < HEADING_SURE;
    }
    memcpy(ekf->last, mean, sizeof(mean));
    ekf->block_us = middle;
    ekf->block_count = 0;
    return sure;
}

// The horizontal velocity and position let go, for the next solution to set.
static void release(Ekf *ekf) {
    for (int i = 0; i < 2; i++) {
        forget(ekf, VELOCITY + i, RESET_VELOCITY);
        forget(ekf, POSITION + i, RESET_POSITION);
    }
}

// How much of the ground's standard deviation (near) a measurement has, the rest being its own (far): all of it on the ground with the motors on, none from EKF_NEAR up, and in between falling off as the square, so the height does not step where the two meet.
static float near_ground(const Ekf *ekf, const EkfTick *tick, float near, float far) {
    if (!tick->sample.spinning) return far;
    float low = clamp(1 - (ekf->p[2] - ekf->ground) / EKF_NEAR, 0, 1);
    return far + (near - far) * low * low;
}

// A GPS solution at the state's own time. At rest its position is where home is; otherwise its climb rate corrects the vertical, and once the heading is known its horizontal velocity and position the rest.
static void take_gps(Ekf *ekf, const HoverGps *fix, const EkfTick *tick) {
    if (ekf->gps_us && fix->capture_us <= ekf->gps_us) return;
    float span = ekf->gps_us ? clamp((fix->capture_us - ekf->gps_us) / 1e6f, 0, 1) : 0;
    ekf->gps_us = fix->capture_us;
    float place[2] = {fix->position[0], -fix->position[1]}, velocity[2] = {fix->velocity[0], -fix->velocity[1]};
    if (tick->still) {
        float pull = ekf->located ? span / (span + HOME_TAU) : 1;
        for (int i = 0; i < 2; i++) ekf->home[i] += (place[i] - ekf->home[i]) * pull;
        ekf->located = true;
        return;
    }
    fuse_one(ekf, VELOCITY + 2, fix->velocity[2] - ekf->v[2], near_ground(ekf, tick, EKF_GPS_CLIMB_NEAR_SD, EKF_GPS_CLIMB_SD), GATE, &ekf->gps_innovation[2]);
    // Home not seen at rest is where this solution is, the heading known or not: the solution's position needs none, and the state's is turned with the world when it is found.
    if (!ekf->located) {
        memcpy(ekf->home, place, sizeof(place));
        ekf->latch[0] = ekf->p[0];
        ekf->latch[1] = ekf->p[1];
        ekf->located = true;
    }
    if (!ekf->aligned) {
        if (!search(ekf, fix)) return;
        // Pinned: the world turned to north, the heading as uncertain as the evidence, and the horizontal velocity and position, dead-reckoned until now, left to this solution.
        turn_world(ekf, ekf->evidence);
        forget(ekf, TILT + 2, fmaxf(ekf->heading_sd, HEADING_FLOOR));
        release(ekf);
        restart_search(ekf);
        ekf->aligned = true;
        ekf->rejected = ekf->place_rejected = 0;
    }
    bool refused = false;
    for (int i = 0; i < 2; i++)
        if (!fuse_one(ekf, VELOCITY + i, velocity[i] - ekf->v[i], EKF_GPS_VELOCITY_SD, GATE, &ekf->gps_innovation[i])) refused = true;
    if (refused) {
        if (++ekf->rejected < EKF_REJECTS) return;
        // The state and the GPS have parted: once, the velocity is the GPS's again and the tilt and heading loosened; again soon after, the heading is wrong.
        ekf->rejected = 0;
        release(ekf);
        for (int i = 0; i < 2; i++) ekf->P[TILT + i][TILT + i] += RESET_TILT * RESET_TILT;
        if (ekf->reset_us && fix->capture_us - ekf->reset_us < RESET_US) {
            ekf->aligned = false;
            restart_search(ekf);
        } else {
            ekf->P[TILT + 2][TILT + 2] += RESET_YAW * RESET_YAW;
        }
        ekf->reset_us = fix->capture_us;
        return;
    }
    ekf->rejected = 0;
    ekf->horizontal_us = fix->capture_us;
    refused = false;
    float innovation[2];
    for (int i = 0; i < 2; i++) {
        innovation[i] = place[i] - ekf->home[i] + ekf->latch[i] - ekf->p[i];
        if (!fuse_one(ekf, POSITION + i, innovation[i], EKF_GPS_POSITION_SD, GATE, NULL)) refused = true;
    }
    if (!refused) {
        ekf->place_rejected = 0;
    } else if (++ekf->place_rejected >= EKF_REJECTS) {
        // The receiver's position has jumped and stayed: measured from there on.
        for (int i = 0; i < 2; i++) ekf->home[i] = place[i] + ekf->latch[i] - ekf->p[i];
        ekf->place_rejected = 0;
    }
}

// A barometer reading at the state's own time: the height plus the offset and the transient.
static void take_baro(Ekf *ekf, const HoverBaro *baro, const EkfTick *tick) {
    if (ekf->baro_us && baro->capture_us <= ekf->baro_us) return;
    ekf->baro_us = baro->capture_us;
    float innovation = baro->altitude - ekf->p[2] - ekf->baro_offset - ekf->baro_transient;
    bool lost = ekf->baro_rejected >= EKF_BARO_REJECTS;
    if (!ekf->baro_started || (lost && !tick->sample.spinning)) {
        ekf->baro_offset += innovation;
        forget(ekf, BARO_OFFSET, 1);
        ekf->baro_started = true;
        ekf->baro_rejected = 0;
        return;
    }
    if (lost) ekf->P[POSITION + 2][POSITION + 2] += RESET_HEIGHT * RESET_HEIGHT;
    int at[3] = {POSITION + 2, BARO_OFFSET, BARO_TRANSIENT};
    float h[3] = {1, 1, 1};
    bool taken = fuse(ekf, 3, at, h, innovation, near_ground(ekf, tick, EKF_BARO_NEAR_SD, EKF_BARO_SD), lost ? 0 : GATE, &ekf->baro_innovation);
    ekf->baro_rejected = taken ? 0 : ekf->baro_rejected + 1;
}

// Still on the ground: no velocity, the place it came to rest at, and a gyro reading only its bias.
static void rest(Ekf *ekf, const EkfSample *sample) {
    if (!ekf->latched) {
        memcpy(ekf->latch, ekf->p, sizeof(ekf->latch));
        ekf->latched = true;
        ekf->located = false;
    }
    for (int i = 0; i < 3; i++) {
        fuse_one(ekf, VELOCITY + i, -ekf->v[i], EKF_REST_SD, 0, NULL);
        fuse_one(ekf, POSITION + i, ekf->latch[i] - ekf->p[i], LATCH_SD, 0, NULL);
        fuse_one(ekf, GYRO_BIAS + i, sample->gyro[i] - ekf->gyro_bias[i], STILL_GYRO_SD, 0, NULL);
    }
}

// No horizontal velocity from the GPS: the horizontal specific force is zero but for sd (m/s²), which holds the tilt (not the heading: the force's own direction says nothing of it) and the accelerometer's bias, and now and then the horizontal velocity is near zero.
static void level(Ekf *ekf, const EkfSample *sample) {
    float R[3][3], sd = sample->spinning ? EKF_LEVEL_FLYING_SD : EKF_LEVEL_SD;
    for (int i = 0; i < 2; i++) {
        rotation(ekf->q, R);
        float force = 0;
        for (int j = 0; j < 3; j++) force += R[i][j] * (sample->accel[j] - ekf->accel_bias[j]);
        // The force along north grows with a tilt about west by the vertical force, the one along west shrinks with a tilt about north.
        int at[4] = {TILT + 1 - i, ACCEL_BIAS, ACCEL_BIAS + 1, ACCEL_BIAS + 2};
        float h[4] = {i ? -ekf->force[2] : ekf->force[2], -R[i][0], -R[i][1], -R[i][2]};
        fuse(ekf, 4, at, h, -force, sd, 0, NULL);
    }
    if (++ekf->bound < 100) return;
    ekf->bound = 0;
    for (int i = 0; i < 2; i++) fuse_one(ekf, VELOCITY + i, -ekf->v[i], sample->spinning ? EKF_FREE_SD : EKF_CARRY_SD, 0, NULL);
}

// The state taken one tick on, and every measurement captured by then.
static void step(Ekf *ekf, const EkfTick *tick) {
    const EkfSample *sample = &tick->sample;
    float dt = clamp(sample->dt, 0, MAX_DT);
    predict(ekf, sample, dt);
    ekf->time_us = sample->time_us;
    for (int i = 0; i < 2; i++) ekf->reckoned[i] += ekf->force[i] * dt;
    if (!sample->spinning) ekf->ground += (ekf->p[2] - ekf->ground) * dt / (dt + GROUND_TAU);
    // The motors starting more than REST_KEPT after the drone was last seen to rest (carried, set down and armed at once): the position is from here, and home where the next GPS solution is. Sooner than that it only stirred (the servos centering at arming rock it), and where it rested stands.
    if (sample->spinning && !ekf->spun && ekf->since_rest > REST_KEPT) {
        memcpy(ekf->latch, ekf->p, sizeof(ekf->latch));
        ekf->located = false;
    }
    ekf->spun = sample->spinning;
    if (tick->still) {
        rest(ekf, sample);
        ekf->since_rest = 0;
    } else {
        ekf->latched = false;
        ekf->since_rest += dt;
    }
    int kept = 0;
    for (int i = 0; i < ekf->baro_count; i++) {
        if (ekf->baro[i].capture_us <= ekf->time_us + 5000) take_baro(ekf, &ekf->baro[i], tick);
        else ekf->baro[kept++] = ekf->baro[i];
    }
    ekf->baro_count = kept;
    kept = 0;
    for (int i = 0; i < ekf->gps_count; i++) {
        if (ekf->gps[i].capture_us <= ekf->time_us + 5000) take_gps(ekf, &ekf->gps[i], tick);
        else ekf->gps[kept++] = ekf->gps[i];
    }
    ekf->gps_count = kept;
    bool aided = ekf->aligned && ekf->horizontal_us && ekf->time_us - ekf->horizontal_us <= GPS_STALE_US;
    if (!aided && !tick->still) level(ekf, sample);
}

void ekf_reset(Ekf *ekf) {
    memset(ekf, 0, sizeof(*ekf));
}

void ekf_gps(Ekf *ekf, const HoverGps *fix) {
    if (ekf->gps_count == EKF_PENDING) memmove(ekf->gps, ekf->gps + 1, --ekf->gps_count * sizeof(*fix));
    ekf->gps[ekf->gps_count++] = *fix;
}

void ekf_baro(Ekf *ekf, const HoverBaro *baro) {
    if (ekf->baro_count == EKF_PENDING) memmove(ekf->baro, ekf->baro + 1, --ekf->baro_count * sizeof(*baro));
    ekf->baro[ekf->baro_count++] = *baro;
}

// The first sample: level from the accelerometer (flight_math.c's angles: the specific force at rest is straight up), facing along the world's first axis, at the origin.
static void start(Ekf *ekf, const EkfSample *sample) {
    const float *a = sample->accel;
    float roll = atan2f(a[1], a[2]), pitch = atan2f(-a[0], hypotf(a[1], a[2]));
    float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    float q[4] = {cp * cr, cp * sr, sp * cr, -sp * sr};
    memcpy(ekf->q, q, sizeof(q));
    for (int i = 0; i < 3; i++) {
        ekf->P[VELOCITY + i][VELOCITY + i] = START_VELOCITY * START_VELOCITY;
        ekf->P[GYRO_BIAS + i][GYRO_BIAS + i] = START_GYRO_BIAS * START_GYRO_BIAS;
        ekf->P[ACCEL_BIAS + i][ACCEL_BIAS + i] = i < 2 ? START_ACCEL_BIAS * START_ACCEL_BIAS : START_ACCEL_BIAS_UP * START_ACCEL_BIAS_UP;
    }
    ekf->P[TILT][TILT] = ekf->P[TILT + 1][TILT + 1] = START_TILT * START_TILT;
    ekf->P[BARO_OFFSET][BARO_OFFSET] = 1;
    ekf->P[BARO_TRANSIENT][BARO_TRANSIENT] = EKF_BARO_TRANSIENT_SD * EKF_BARO_TRANSIENT_SD;
    ekf->heading_sd = PI;
    ekf->time_us = sample->time_us;
    ekf->started = true;
}

void ekf_update(Ekf *ekf, const EkfSample *sample, EkfOutput *output) {
    if (!ekf->started) {
        start(ekf, sample);
    } else {
        float rate = 0, weight = 0;
        for (int i = 0; i < 3; i++) {
            float turning = sample->gyro[i] - ekf->gyro_bias[i];
            rate += turning * turning;
            weight += sample->accel[i] * sample->accel[i];
        }
        bool quiet = !sample->spinning && rate < STILL_GYRO * STILL_GYRO && fabsf(sqrtf(weight) - GRAVITY) < STILL_ACCEL;
        ekf->still_time = quiet ? ekf->still_time + sample->dt : 0;
        EkfTick *tick = &ekf->ticks[ekf->next];
        tick->sample = *sample;
        tick->still = ekf->still_time >= STILL_S;
        ekf->next = (ekf->next + 1) % EKF_HISTORY;
        ekf->waiting++;
        while (ekf->waiting > EKF_DELAY) {
            step(ekf, &ekf->ticks[(ekf->next - ekf->waiting + EKF_HISTORY) % EKF_HISTORY]);
            ekf->waiting--;
        }
    }
    // The state carried through the ticks it has yet to take, with the biases it has now.
    float q[4], v[3], p[3], R[3][3], force[3];
    memcpy(q, ekf->q, sizeof(q));
    memcpy(v, ekf->v, sizeof(v));
    memcpy(p, ekf->p, sizeof(p));
    for (int k = ekf->waiting; k > 0; k--) {
        const EkfSample *s = &ekf->ticks[(ekf->next - k + EKF_HISTORY) % EKF_HISTORY].sample;
        advance(q, v, p, ekf->gyro_bias, ekf->accel_bias, s, clamp(s->dt, 0, MAX_DT), R, force);
    }
    rotation(q, R);
    output->roll = atan2f(R[2][1], R[2][2]);
    output->pitch = -asinf(clamp(R[2][0], -1, 1));
    output->yaw = atan2f(R[1][0], R[0][0]);
    for (int i = 0; i < 3; i++) {
        output->velocity[i] = v[i];
        output->position[i] = p[i];
        output->gyro_bias[i] = ekf->gyro_bias[i];
        output->accel_bias[i] = ekf->accel_bias[i];
        output->gps_innovation[i] = ekf->gps_innovation[i];
    }
    for (int i = 0; i < 2; i++) output->position[i] -= ekf->latch[i];
    output->height = p[2] - ekf->ground;
    output->baro_offset = ekf->baro_offset;
    output->baro_transient = ekf->baro_transient;
    output->tilt_sd = sqrtf(fmaxf(ekf->P[TILT][TILT] + ekf->P[TILT + 1][TILT + 1], 0));
    output->yaw_sd = sqrtf(fmaxf(ekf->P[TILT + 2][TILT + 2], 0));
    output->velocity_sd = sqrtf(fmaxf(ekf->P[VELOCITY][VELOCITY] + ekf->P[VELOCITY + 1][VELOCITY + 1], 0));
    output->position_sd = sqrtf(fmaxf(ekf->P[POSITION][POSITION] + ekf->P[POSITION + 1][POSITION + 1], 0));
    output->height_sd = sqrtf(fmaxf(ekf->P[POSITION + 2][POSITION + 2], 0));
    output->aligned = ekf->aligned;
    output->still = ekf->waiting ? ekf->ticks[(ekf->next - 1 + EKF_HISTORY) % EKF_HISTORY].still : false;
    output->heading_sd = ekf->heading_sd;
    output->baro_innovation = ekf->baro_innovation;
    output->rejected = ekf->rejected;
}

void ekf_estimate(const EkfOutput *output, HoverEstimate *estimate) {
    *estimate = (HoverEstimate){.yaw = output->yaw, .velocity = {output->velocity[0], output->velocity[1], output->velocity[2]},
                                .position = {output->position[0], output->position[1]}, .height = output->height,
                                .horizontal = output->aligned && output->velocity_sd < EKF_FLOWN_VELOCITY_SD,
                                .vertical = output->height_sd > 0 && output->height_sd < EKF_FLOWN_HEIGHT_SD};
}

int ekf_size(void) {
    return sizeof(Ekf);
}
