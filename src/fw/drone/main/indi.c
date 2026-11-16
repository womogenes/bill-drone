#include "indi.h"

#include <math.h>
#include <string.h>

#include "flight_math.h"

// The servos (sw/sim/models/airframe_2026-11-12_raised.json, both sides' mean): the horn's second-order loop (rad/s, damping ratio) and its speed limit (rad/s, 285 deg/s), stepped SERVO_STEPS times a tick.
#define SERVO_WN 37.5f
#define SERVO_ZETA 0.87f
#define SERVO_SLEW 4.97f
#define SERVO_STEPS 4
// Swinging a motor assembly turns the body the other way (rad/s² of body pitch per rad/s² of the two assemblies' summed tilt acceleration): fitted with the pitch authority (sw/sim/authority.py, 0.106). The CAD's inertia ratio gives 0.049 (sw/sim/airframe.py resolve J, 2.59e-4 kg m² with the raised pivots' lever, over Iyy 5.25e-3); the link's spring and the servo's load add to it. In the first instant of a servo move this is several times the tilted thrust's own pitch, and left out, INDI takes it as unmodeled and swings the servos harder against it.
#define ASSEMBLY_RATIO 0.106f
// The motors' lag (s) after their dead time (indi.h INDI_MOTOR_DELAY), fitted with it.
#define MOTOR_TAU 0.035f
// The low-pass both the measured and the modeled acceleration go through (Hz; second-order Butterworth): the gyro's difference at 100 Hz is noise above it, and the motor assemblies ring on their links at about 11 Hz (link_k over J), which the servo estimate leaves out; at 15 Hz the sim's hovers shook at that ring (sw/sim/models/NOTES.md "INDI").
#define FILTER_HZ 8.0f

// The low-pass on what the drone's pitch acceleration has beyond the predictor's model (s): the speed moment that cancels the tilt below 1 Hz and the CoM's push are in it, and the gyro difference's noise and the links' 11 Hz ring are not.
#define PREDICT_TAU 0.05f

// The most unmodeled acceleration taken (rad/s²; roll, pitch, yaw): a CoM 10 mm off is about 40 in pitch.
static const float LIMIT[3] = {40, 60, 15};

void indi_reset(Indi *indi) {
    memset(indi, 0, sizeof(*indi));
}

void indi_sent(Indi *indi, const float *tilt, const float *thrust) {
    if (!indi->sent) {
        indi->sent = true;
        // The first tick: as if this had always been sent.
        for (int k = 0; k < INDI_SERVO_DELAY; k++) {
            memcpy(indi->sent_tilt[k], tilt, sizeof(indi->sent_tilt[k]));
            memcpy(indi->sent_thrust[k], thrust, sizeof(indi->sent_thrust[k]));
        }
        memcpy(indi->tilt, tilt, sizeof(indi->tilt));
        memcpy(indi->thrust, thrust, sizeof(indi->thrust));
    }
    memcpy(indi->sent_tilt[indi->next], tilt, sizeof(indi->sent_tilt[0]));
    memcpy(indi->sent_thrust[indi->next], thrust, sizeof(indi->sent_thrust[0]));
    indi->next = (indi->next + 1) % INDI_SERVO_DELAY;
}

// A second-order Butterworth low-pass at FILTER_HZ for this tick length: b0 (b1 is twice it, b2 equal), a1, a2.
typedef struct {
    float b0, a1, a2;
} LowPass;

static LowPass low_pass_for(float dt) {
    float k = tanf(PI * FILTER_HZ * dt), norm = 1 / (1 + 1.41421356f * k + k * k);
    return (LowPass){k * k * norm, 2 * (k * k - 1) * norm, (1 - 1.41421356f * k + k * k) * norm};
}

// One step of it (transposed direct form II); a starting filter is settled at its input.
static float low_pass(const LowPass *f, float *s, float x, bool start) {
    float b0 = f->b0, a1 = f->a1, a2 = f->a2;
    if (start) {
        s[0] = x * (1 - b0);
        s[1] = x * (b0 - a2);
    }
    float y = b0 * x + s[0];
    s[0] = 2 * b0 * x - a1 * y + s[1];
    s[1] = b0 * x - a2 * y;
    return y;
}

// One tick of an assembly's servo toward goal (the second-order loop with its slew), returning its angular acceleration over the tick.
static float servo_step(float *tilt, float *tilt_rate, float goal, float dt) {
    float h = dt / SERVO_STEPS, before = *tilt_rate;
    for (int step = 0; step < SERVO_STEPS; step++) {
        float push = SERVO_WN * SERVO_WN * (goal - *tilt) - 2 * SERVO_ZETA * SERVO_WN * *tilt_rate;
        *tilt_rate = clampf(*tilt_rate + push * h, -SERVO_SLEW, SERVO_SLEW);
        *tilt += *tilt_rate * h;
    }
    return (*tilt_rate - before) / dt;
}

// The angular accelerations (roll, pitch, yaw) model gives for these tilts and thrusts with the assemblies swinging at swing (the two's angular acceleration summed).
static void model_accel(const Model *model, const float *tilt, const float *thrust, float swing, float *accel) {
    float vertical[2], forward[2];
    for (int side = 0; side < 2; side++) {
        vertical[side] = thrust[side] * cosf(tilt[side]);
        forward[side] = thrust[side] * sinf(tilt[side]);
    }
    accel[0] = model->roll * (vertical[0] - vertical[1]);
    accel[1] = model->pitch * (forward[0] + forward[1]) - ASSEMBLY_RATIO * swing;
    accel[2] = model->yaw * (forward[1] - forward[0]);
}

// What the motors were sent k ticks after the servos' oldest goal (sent_tilt[next]): the motors' input k ticks from now is the one at INDI_SERVO_DELAY - INDI_MOTOR_DELAY + k, the newest once that runs past what was sent.
static const float *motor_goal(const Indi *indi, int k) {
    int at = INDI_SERVO_DELAY - INDI_MOTOR_DELAY + k;
    return indi->sent_thrust[(indi->next + (at < INDI_SERVO_DELAY ? at : INDI_SERVO_DELAY - 1)) % INDI_SERVO_DELAY];
}

// One tick of each motor's lag toward goal.
static void motor_step(float *thrust, const float *goal, float keep) {
    for (int side = 0; side < 2; side++) thrust[side] = goal[side] + (thrust[side] - goal[side]) * keep;
}

void indi_update(Indi *indi, const float *gyro, const Model *model, float dt) {
    // Nothing sent yet (disarmed): nothing to compare.
    if (!indi->sent) {
        memset(indi->unmodeled, 0, sizeof(indi->unmodeled));
        return;
    }
    bool start = !indi->started;
    indi->started = true;
    // The actuators by now: the servos from what was sent INDI_SERVO_DELAY ticks ago, the motors from INDI_MOTOR_DELAY ticks ago.
    const float *goal = indi->sent_tilt[indi->next];
    float swing = 0;
    for (int side = 0; side < 2; side++) swing += servo_step(&indi->tilt[side], &indi->tilt_rate[side], goal[side], dt);
    motor_step(indi->thrust, motor_goal(indi, 0), expf(-dt / MOTOR_TAU));
    // The measured angular acceleration: the filtered gyro's change over the tick. The gyro is the tick's mean rate, so its change is the acceleration half a tick before each reading's middle, a tick before now: it is set against the model's estimate from the last tick.
    LowPass filter = low_pass_for(dt);
    float rate[3], accel[3];
    for (int axis = 0; axis < 3; axis++) {
        float filtered = low_pass(&filter, indi->filter[axis], gyro[axis], start);
        rate[axis] = start ? 0 : (filtered - indi->gyro[axis]) / dt;
        indi->gyro[axis] = filtered;
    }
    model_accel(model, indi->tilt, indi->thrust, swing, accel);
    for (int axis = 0; axis < 3; axis++) {
        float left = start ? 0 : rate[axis] - indi->model[axis];
        indi->model[axis] = low_pass(&filter, indi->filter[3 + axis], accel[axis], start);
        indi->unmodeled[axis] = clampf(left, -LIMIT[axis], LIMIT[axis]);
        if (axis == 1) indi->beyond += (left - indi->beyond) * dt / (PREDICT_TAU + dt);
    }
}

void indi_predict(const Indi *indi, const Model *model, float pitch_rate, float dt, float *rate, float *angle) {
    *rate = *angle = 0;
    if (!indi->started) return;
    float tilt[2] = {indi->tilt[0], indi->tilt[1]}, tilt_rate[2] = {indi->tilt_rate[0], indi->tilt_rate[1]}, thrust[2] = {indi->thrust[0], indi->thrust[1]};
    float keep = expf(-dt / MOTOR_TAU);
    // The goals of the next ticks, oldest first: sent_tilt[next] went in this tick, and this tick's command comes in after them.
    for (int k = 1; k < INDI_SERVO_DELAY; k++) {
        const float *goal = indi->sent_tilt[(indi->next + k) % INDI_SERVO_DELAY];
        float swing = 0, accel[3];
        for (int side = 0; side < 2; side++) swing += servo_step(&tilt[side], &tilt_rate[side], goal[side], dt);
        motor_step(thrust, motor_goal(indi, k), keep);
        model_accel(model, tilt, thrust, swing, accel);
        float before = *rate;
        *rate += (accel[1] + indi->beyond) * dt;
        *angle += (pitch_rate + (before + *rate) / 2) * dt;
    }
}
