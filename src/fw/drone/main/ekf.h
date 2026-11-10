#ifndef EKF_H
#define EKF_H

#include <stdbool.h>
#include <stdint.h>

#include "hover.h"

// One Kalman filter for the drone's attitude, velocity and position from the IMU, the GPS and the barometer (an error-state extended Kalman filter), in place of the separate estimates in flight_math.c (attitude: gyro pulled toward the accelerometer) and hover.c (velocity, heading, height: each sensor pulled in over its own time constant). No ESP-IDF, so the host replays recordings through it and the simulation flies it (sw/sim/ekf.py).
// Axes: the body's are the IMU's (x forward, y left, z up), the world's north, west and up, so with roll, pitch and yaw all zero the nose points north; roll, pitch and yaw are flight_math.c's (positive roll right side down, positive pitch nose down, positive yaw a left turn).
// The states: attitude, velocity (m/s), position (m), the gyro's bias (rad/s, body), the accelerometer's bias (m/s², body), the barometer's offset (its pressure altitude less the position's height, m) and its transient (what moving air adds to its reading for a moment, m).
// Late measurements: a GPS solution describes the drone 60-120 ms before it arrives and a barometer reading 75 ms, so the filter itself runs EKF_DELAY ticks behind the newest IMU sample, where each measurement meets the state of the moment it measured, and the output is that state carried forward through the IMU samples since.
// Heading: nothing on the drone measures it, so until it is known the world's axes are turned from north by an unknown angle and the GPS's horizontal velocity and position are not used (the attitude then leans on the accelerometer as flight_math.c's does). The angle is found as hover.c finds it, from the GPS velocity's changes against the filter's own; then the state is turned to north once (aligned) and the filter refines the heading itself whenever the drone accelerates horizontally.

#define EKF_STATES 17
#define EKF_DELAY 10
#define EKF_HISTORY 16
#define EKF_PENDING 8

// One tick of the IMU: when it was read, the gyro (rad/s, the boot calibration's bias removed) and the accelerometer's specific force (m/s²) averaged over the dt (s) before it, and whether the motors are turning.
typedef struct {
    int64_t time_us;
    float gyro[3], accel[3], dt;
    bool spinning;
} EkfSample;

// The estimate at the newest sample. roll, pitch, yaw (radians; yaw counterclockwise from north once aligned, before that from where the nose pointed at the start); velocity and position north, west and up (m/s, m; north and west from where the drone last rested or its motors started, up from where the filter started); height above where it last rested (m); the biases, the barometer's offset and transient; one standard deviation of the tilt, yaw, horizontal velocity, horizontal position and height; whether the heading is known and the evidence's standard deviation before it is (radians); whether it is resting still; and the last GPS velocity and barometer innovations over their expected standard deviations.
typedef struct {
    float roll, pitch, yaw;
    float velocity[3], position[3], height;
    float gyro_bias[3], accel_bias[3], baro_offset, baro_transient;
    float tilt_sd, yaw_sd, velocity_sd, position_sd, height_sd;
    bool aligned, still;
    float heading_sd;
    float gps_innovation[3], baro_innovation;
    int rejected;
} EkfOutput;

typedef struct {
    EkfSample sample;
    bool still;
} EkfTick;

typedef struct {
    // The state EKF_DELAY ticks back and its time: attitude (body to world, w x y z), velocity, position, biases, the barometer's offset and transient, and the covariance of their errors (attitude error as a small rotation in world axes, then velocity, position, gyro bias, accelerometer bias, barometer offset and transient). force is the specific force of the last tick taken, in world axes (m/s²).
    float q[4], v[3], p[3], gyro_bias[3], accel_bias[3], baro_offset, baro_transient;
    float P[EKF_STATES][EKF_STATES], force[3];
    int64_t time_us;
    bool started, aligned, located, latched, spun, baro_started;
    // The newest ticks (waiting of them not yet taken into the state), and the GPS solutions and barometer readings waiting for the state to reach their capture times.
    EkfTick ticks[EKF_HISTORY];
    int next, waiting;
    HoverGps gps[EKF_PENDING];
    HoverBaro baro[EKF_PENDING];
    int gps_count, baro_count;
    // Rest: how long the drone has been still (s), where the state was when it came to rest (latch) and the GPS's position there (home: north, west, m), and the height of the ground it last rested on (m). The GPS's position is measured from home and the state's from latch, so the position is from where the drone last rested (or, not seen resting for since_rest s, from where its motors started: spun is whether they were turning at the last tick).
    float still_time, since_rest, latch[3], home[2], ground;
    // Capture times of the last GPS solution taken, the last used horizontally and the last barometer reading; when the horizontal state was last reset to the GPS; ticks since the unaided velocity was last bounded.
    int64_t gps_us, horizontal_us, baro_us, reset_us;
    int bound;
    // The heading search (as hover.c learn_heading): the horizontal specific force summed since the search began (reckoned, m/s: the velocity the IMU alone would give), the block being averaged (GPS north and west, reckoned's two), the last block's means and middle, and the evidence (the summed products of their changes, the products' weight and squares, reckoned's changes squared, the angle they give and its standard deviation).
    float reckoned[2], block[4], last[4], turn[2], turn_weight, turn_square[2], turn_inertial, evidence, heading_sd;
    int block_count;
    int64_t block_start_us, block_us, search_us;
    // Diagnostics: the last innovations over their standard deviations, and how many GPS solutions (horizontal velocity, position) and barometer readings in a row the gate has refused.
    float gps_innovation[3], baro_innovation;
    int rejected, place_rejected, baro_rejected;
} Ekf;

// Start over (also the first call): the next sample sets the attitude from the accelerometer, so the drone should be at rest.
void ekf_reset(Ekf *ekf);
// A GPS solution (hover.h: capture time, velocity north, east, up, position north, east from any fixed point) and a barometer reading, queued until the filter reaches their capture.
void ekf_gps(Ekf *ekf, const HoverGps *fix);
void ekf_baro(Ekf *ekf, const HoverBaro *baro);
// One tick: the newest IMU sample in, the estimate at it out.
void ekf_update(Ekf *ekf, const EkfSample *sample, EkfOutput *output);
// The estimate as the hold flies it (hover.h hover_estimate, gains ekf): horizontal while the heading is known and the velocity's standard deviation is under EKF_FLOWN_VELOCITY_SD, vertical while the height's is under EKF_FLOWN_HEIGHT_SD.
void ekf_estimate(const EkfOutput *output, HoverEstimate *estimate);
// The structure's size, for hosts that allocate it without the header.
int ekf_size(void);

#endif
