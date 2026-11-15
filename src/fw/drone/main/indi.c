#include "indi.h"

#include <math.h>
#include <string.h>

#include "flight_math.h"

// The servos (sw/sim/models/airframe_2026-11-12_raised.json, both sides' mean): the horn's second-order loop (rad/s, damping ratio) and its speed limit (rad/s, 285 deg/s), stepped SERVO_STEPS times a tick.
#define SERVO_WN 37.5f
#define SERVO_ZETA 0.87f
#define SERVO_SLEW 4.97f
#define SERVO_STEPS 4
// Swinging a motor assembly turns the body the other way (rad/s² of body pitch per rad/s² of the assembly's tilt). The CAD's inertia ratio gives 0.091 (sw/sim/airframe.py resolve J, 4.78e-4 kg m², over Iyy 5.25e-3); the 2026-11-15 hovers fit 0.27 with INDI_SERVO_DELAY's 60 ms (the link's spring and the servo's load add to it). In the first instant of a servo move this is several times the tilted thrust's own pitch, and left out, INDI takes it as unmodeled and swings the servos harder against it. 0.18 is where the servos' dead-time predictor (indi_predict) foretells the drone's pitch rate as large as it comes above 4 Hz (sw/sim/predict_check.py on the 2026-11-15 hovers): at 0.27, the fit to 1.5-4 Hz alone, it foretold 1.5-2 times the drone's motion there, and in the sim a predictor that overstates the fast reaction feeds it back as an 8 Hz oscillation; at 0.18 its 1.5-4 Hz prediction keeps slope 1.07-1.25 and correlation 0.88-0.96.
#define ASSEMBLY_RATIO 0.18f
// The motors' lag (s; the same fit's motor_tau). Their 11 ms dead time is the tick between sending and the next update.
#define MOTOR_TAU 0.066f
// The low-pass both the measured and the modeled acceleration go through (Hz; second-order Butterworth): the gyro's difference at 100 Hz is noise above it, and the motor assemblies ring on their links at about 11 Hz (link_k over J), which the servo estimate leaves out; at 15 Hz the sim's hovers shook at that ring (sw/sim/models/NOTES.md "INDI").
#define FILTER_HZ 8.0f

// The predictor's pitch per the controller's model's (indi_predict): over 2026-11-15's hovers the pitch rate's change in the next 50 ms against the model's at 1.5-4 Hz came out 0.64-0.79 with correlation 0.92-0.99 (the model's pitch_inertia is a third of the CAD's; the tilt's authority there is 1.57 times the CAD's, sw/sim/models/NOTES.md "One plant"). With the drone's own model settings (pivot_height 0.02) the prediction is then 0.8-0.93 of the drone's change there (sw/sim/predict_check.py); pitch_predict's shares from the sims are on this scale.
#define PREDICT_AUTHORITY 0.7f
// The low-pass on what the drone's pitch acceleration has beyond the predictor's model (s): the speed moment that cancels the tilt below 1 Hz and the CoM's push are in it, and the gyro difference's noise and the links' 11 Hz ring are not.
#define PREDICT_TAU 0.05f

// The most unmodeled acceleration taken (rad/s²; roll, pitch, yaw): a CoM 10 mm off is about 40 in pitch on the controller's model.
static const float LIMIT[3] = {40, 60, 15};

void indi_reset(Indi *indi) {
    memset(indi, 0, sizeof(*indi));
}

void indi_sent(Indi *indi, const float *tilt, const float *thrust) {
    if (!indi->sent) {
        indi->sent = true;
        // The first tick: as if this had always been sent.
        for (int k = 0; k < INDI_SERVO_DELAY; k++) memcpy(indi->sent_tilt[k], tilt, sizeof(indi->sent_tilt[k]));
        memcpy(indi->tilt, tilt, sizeof(indi->tilt));
        memcpy(indi->thrust, thrust, sizeof(indi->thrust));
    }
    memcpy(indi->sent_tilt[indi->next], tilt, sizeof(indi->sent_tilt[0]));
    indi->next = (indi->next + 1) % INDI_SERVO_DELAY;
    memcpy(indi->sent_thrust, thrust, sizeof(indi->sent_thrust));
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
static void model_accel(const IndiModel *model, const float *tilt, const float *thrust, float swing, float *accel) {
    float vertical[2], forward[2];
    for (int side = 0; side < 2; side++) {
        vertical[side] = thrust[side] * cosf(tilt[side]);
        forward[side] = thrust[side] * sinf(tilt[side]);
    }
    accel[0] = model->roll * (vertical[0] - vertical[1]);
    accel[1] = model->pitch * (forward[0] + forward[1]) + model->lift_pitch * (vertical[0] + vertical[1]) - ASSEMBLY_RATIO * swing;
    accel[2] = model->yaw * (forward[1] - forward[0]) + model->reaction * (vertical[1] - vertical[0]);
}

void indi_update(Indi *indi, const float *gyro, const IndiModel *models, float dt) {
    // Nothing sent yet (disarmed): nothing to compare.
    if (!indi->sent) {
        memset(indi->unmodeled, 0, sizeof(indi->unmodeled));
        return;
    }
    bool start = !indi->started;
    indi->started = true;
    // The actuators by now: the servos from what was sent INDI_SERVO_DELAY ticks ago, the motors from last tick's.
    const float *goal = indi->sent_tilt[indi->next];
    float keep = expf(-dt / MOTOR_TAU), swing = 0;
    for (int side = 0; side < 2; side++) {
        swing += servo_step(&indi->tilt[side], &indi->tilt_rate[side], goal[side], dt);
        indi->thrust[side] = indi->sent_thrust[side] + (indi->thrust[side] - indi->sent_thrust[side]) * keep;
    }
    // The measured angular acceleration: the filtered gyro's change over the tick. The gyro is the tick's mean rate, so its change is the acceleration half a tick before each reading's middle, a tick before now: it is set against the models' estimate from the last tick.
    LowPass filter = low_pass_for(dt);
    float rate[3];
    for (int axis = 0; axis < 3; axis++) {
        float filtered = low_pass(&filter, indi->filter[axis], gyro[axis], start);
        rate[axis] = start ? 0 : (filtered - indi->gyro[axis]) / dt;
        indi->gyro[axis] = filtered;
    }
    for (int m = 0; m < INDI_MODELS; m++) {
        float accel[3];
        model_accel(&models[m], indi->tilt, indi->thrust, swing, accel);
        for (int axis = 0; axis < 3; axis++) {
            float before = indi->model[m][axis];
            if (m == INDI_CONTROLLER && axis == 1) {
                float seen = start ? 0 : rate[1] - PREDICT_AUTHORITY * before;
                indi->beyond += (seen - indi->beyond) * dt / (PREDICT_TAU + dt);
            }
            indi->model[m][axis] = low_pass(&filter, indi->filter[3 + 3 * m + axis], accel[axis], start);
            indi->unmodeled[m][axis] = start ? 0 : clampf(rate[axis] - before, -LIMIT[axis], LIMIT[axis]);
        }
    }
}

void indi_predict(const Indi *indi, const IndiModel *model, float pitch_rate, float dt, float *rate, float *angle) {
    *rate = *angle = 0;
    if (!indi->started) return;
    float tilt[2] = {indi->tilt[0], indi->tilt[1]}, tilt_rate[2] = {indi->tilt_rate[0], indi->tilt_rate[1]}, thrust[2] = {indi->thrust[0], indi->thrust[1]};
    float keep = expf(-dt / MOTOR_TAU);
    // The goals of the next ticks, oldest first: sent_tilt[next] went in this tick, and this tick's command comes in after them.
    for (int k = 1; k < INDI_SERVO_DELAY; k++) {
        const float *goal = indi->sent_tilt[(indi->next + k) % INDI_SERVO_DELAY];
        float swing = 0, accel[3];
        for (int side = 0; side < 2; side++) {
            swing += servo_step(&tilt[side], &tilt_rate[side], goal[side], dt);
            thrust[side] = indi->sent_thrust[side] + (thrust[side] - indi->sent_thrust[side]) * keep;
        }
        model_accel(model, tilt, thrust, swing, accel);
        float before = *rate;
        *rate += (PREDICT_AUTHORITY * accel[1] + indi->beyond) * dt;
        *angle += (pitch_rate + (before + *rate) / 2) * dt;
    }
}
