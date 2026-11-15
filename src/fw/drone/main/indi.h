#pragma once
#include <stdbool.h>

// Incremental nonlinear dynamic inversion (settings.def indi): the angular acceleration the gyro measures, less what the actuators as they are by now should give, is what the controller's model leaves out (the CoM's offset, the rotors' gyroscopic moments, the slipstream, wind, an effectiveness wrong by a factor); the controller asks its allocation for the acceleration it wants less that, so the drone gets what is wanted whatever the model misses. The actuators are estimated from what was sent (each servo's dead time, second-order loop and slew, each motor's dead time and lag: sw/sim/airframe.py's fits), and both sides go through the same low-pass, so the comparison is of like with like. Smeur, de Croon and Chu, "Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles" (2016).

// How the actuators turn the body (rad/s² per newton), the inverse of what the allocation assumes: roll per newton of the left motor's vertical thrust over the right's, pitch per newton of the two's forward thrust and per newton of their vertical thrust (the CoM's offset ahead of the pivots), yaw per newton of the right's forward thrust over the left's and per newton of the right's vertical over the left's (the props' reaction).
typedef struct {
    float roll, pitch, lift_pitch, yaw, reaction;
} IndiModel;

// The two allocations' models: the attitude controller's (flight_math.c, from the settings' Model; the recovery's righting shares it, recover.c's authority) and the flip's (flip.c, its own authority).
enum { INDI_CONTROLLER, INDI_FLIP, INDI_MODELS };

// Ticks of servo dead time held (sw/sim/airframe.py servo_delay: 0.040-0.043 s at 100 Hz).
#define INDI_SERVO_DELAY 4

typedef struct {
    // What was sent each of the last ticks (physical forward tilt, radians, newest at next - 1) and the motors' thrust asked last tick (N).
    float sent_tilt[INDI_SERVO_DELAY][2], sent_thrust[2];
    int next;
    // The actuators as estimated by now: each assembly's forward tilt (radians) and its rate (rad/s), each motor's thrust (N).
    float tilt[2], tilt_rate[2], thrust[2];
    // The low-pass's state (two per channel): the gyro's three axes, then each model's three accelerations; the filtered gyro and each model's filtered acceleration at the last tick.
    float filter[3 + 3 * INDI_MODELS][2];
    float gyro[3], model[INDI_MODELS][3];
    // Whether anything has been sent since the reset, and the filters started.
    bool sent, started;
    // The unmodeled angular acceleration (rad/s²; roll, pitch, yaw) by each model, bounded; the allocation takes its share (settings.def indi) off what it is asked.
    float unmodeled[INDI_MODELS][3];
} Indi;

void indi_reset(Indi *indi);
// The actuators as sent this tick: each assembly's physical forward tilt (radians) and each motor's thrust (N; zero for a stopped motor).
void indi_sent(Indi *indi, const float *tilt, const float *thrust);
// The tick's gyro (rad/s, body roll, pitch, yaw) against the actuators as estimated through each model.
void indi_update(Indi *indi, const float *gyro, const IndiModel *models, float dt);
