#pragma once
#include <stdbool.h>

#include "flight_math.h"

// Incremental nonlinear dynamic inversion (settings.def indi): the angular acceleration the gyro measures, less what the actuators as they are by now should give, is what the controller's model leaves out (the CoM's offset, the rotors' gyroscopic moments, the slipstream, wind, an effectiveness wrong by a factor); the controller asks its allocation for the acceleration it wants less that, so the drone gets what is wanted whatever the model misses. The actuators are estimated from what was sent (each servo's dead time, second-order loop and slew, each motor's dead time and lag), and the model and these together are fitted to foretell the drone's body rates 50 ms ahead (sw/sim/authority.py), and both sides go through the same low-pass, so the comparison is of like with like. Smeur, de Croon and Chu, "Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles" (2016).

// Ticks of servo dead time held: 6 (60 ms) fits the hovers of 2026-11-15 (06:29-06:34 UTC): with the horn at SERVO_WN, the modeled pitch acceleration explains 83% of the measured below 2 Hz and 98% in 2-4 Hz, against 31% and 86% at sw/sim/airframe.py's 40 ms (servo_delay 0.040-0.043), where INDI at 0.25-0.5 doubled the 3 Hz pitch shake.
#define INDI_SERVO_DELAY 6
// Ticks of the motors' dead time (the tick to the next update and the ESCs' and props' start), ahead of their lag (indi.c MOTOR_TAU): 4 and 35 ms foretell the roll rate 50 ms ahead best on the new ESCs' flights (sw/sim/authority.py), against 1 and 66 ms before. At most INDI_SERVO_DELAY: the motors' commands are held in the same ring as the servos'.
#define INDI_MOTOR_DELAY 4

typedef struct {
    // What was sent each of the last ticks (newest at next - 1): each assembly's physical forward tilt (radians) and each motor's thrust (N).
    float sent_tilt[INDI_SERVO_DELAY][2], sent_thrust[INDI_SERVO_DELAY][2];
    int next;
    // The actuators as estimated by now: each assembly's forward tilt (radians) and its rate (rad/s), each motor's thrust (N).
    float tilt[2], tilt_rate[2], thrust[2];
    // The low-pass's state (two per channel): the gyro's three axes, then the model's three accelerations; the filtered gyro and the model's filtered acceleration at the last tick.
    float filter[6][2];
    float gyro[3], model[3];
    // Whether anything has been sent since the reset, and the filters started.
    bool sent, started;
    // The unmodeled angular acceleration (rad/s²; roll, pitch, yaw), bounded; the controller's, the burst's and the recovery's allocations take its share (settings.def indi) off what they are asked.
    float unmodeled[3];
    // The pitch acceleration the drone has beyond the model (rad/s², low-passed over indi.c PREDICT_TAU): the speed moment, the CoM, wind, which indi_predict takes to hold over its horizon.
    float beyond;
} Indi;

void indi_reset(Indi *indi);
// The actuators as sent this tick: each assembly's physical forward tilt (radians) and each motor's thrust (N; zero for a stopped motor).
void indi_sent(Indi *indi, const float *tilt, const float *thrust);
// The tick's gyro (rad/s, body roll, pitch, yaw) against the actuators as estimated through the airframe's model (flight_math.h Model: settings.def roll_authority, pitch_authority, yaw_authority).
void indi_update(Indi *indi, const float *gyro, const Model *model, float dt);
// A Smith predictor for the servos' dead time (Normey-Rico and Camacho, "Dead-time compensators: a survey", 2008): how much the pitch rate (rad/s) and pitch (radians, with the horizon's travel at the measured pitch_rate) will have changed by the time a servo command sent now first moves the assemblies, from the commands already on their way (INDI_SERVO_DELAY - 1 ticks of them, stepped through the servos' estimate) on the model's pitch, with what the drone has beyond that held as it is now. After indi_update in the same tick.
void indi_predict(const Indi *indi, const Model *model, float pitch_rate, float dt, float *rate, float *angle);
