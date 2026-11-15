#include "flip.h"

#include <math.h>

#define TWO_PI (2 * PI)
#define HALF_PI (PI / 2)
// How fast the burst's idea of the motors' thrust follows what it asks (s; the motors' lag).
#define FLIP_MOTOR_TAU 0.1f
// The arrest ends once the vertical speed has stayed under ARREST_DONE (m/s) for ARREST_HOLD (s). Its collective is at least ARREST_FLOOR of the weight: the attitude controller flies the arrest and needs thrust to do it (an arrest cut to 0.15 of the weight tipped over in the sim), and at most ARREST_MOST, under the 2.8 times the weight the motors give at the thrust table's top (2 × 11.2 N at the 1.3 thrust scale against 1.05 kg), so the attitude controller keeps a margin of differential thrust.
#define ARREST_DONE 0.2f
#define ARREST_HOLD 0.3f
#define ARREST_FLOOR 0.6f
#define ARREST_MOST 2.2f
// The flip request (flip_refusal) wants the hold settled this long after liftoff (s), within FLIP_LEVEL of level and turning under FLIP_STILL on every axis, its climb rate under FLIP_CLIMB_RATE and the GPS's ground speed under FLIP_SPEED (m/s), and the pack at FLIP_MIN_V or more (3.7 V a cell under load on 4S: a sagging pack gives less of the thrust the turn's brake asks).
#define FLIP_SETTLE_S 5.0f
#define FLIP_LEVEL (10 * DEGREE)
#define FLIP_STILL (30 * DEGREE)
#define FLIP_CLIMB_RATE 0.3f
#define FLIP_SPEED 1.0f
#define FLIP_MIN_V 14.8f
// The differential tilt for yaw is at most this (radians): a wider bound (0.5 rad, which servo_max's common tilt and SERVO_LIMIT's 60 degrees would allow) left the bursts' yaw as it was and cost the turn (rotor_momentum 0.04-0.05 with the CoM 15.5 mm left: 79% clean to 60%): the rotors' yaw is more than the tilt can take back, and the wider tilt couples into pitch (sw/sim/models/NOTES.md "Yaw in the flip").
#define DIFFERENTIAL_TILT 0.3f

const FlipParams flip_defaults = {
    .right_angle = 0.5831f, .handoff_rate = 3.0f, .timeout = 1.9751f, .kv = 15.0f, .arrest_time = 2.0f,
    .floor = 0.2645f, .ceiling = 1.2286f, .servo_max = 0.5092f, .slew = 10000.0f, .throttle_cap = 1100.0f,
    .gyro_ff = 8.8625f, .yaw_ff = 1.6f,
    .ref_accel = 26.47f, .ref_rate = 11.42f, .ref_lead = 0.0701f, .track_slack = 1.212f,
    .track_kp = 6.525f, .brake = 8.0696f, .max_rate = 8.5717f, .track_heading = 0.384f, .track_roll = 13.16f, .track_pitch = 2.904f, .track_yaw = 2.984f,
};

// Each motor between floor and ceiling of the weight: the difference within what that allows, then the collective as asked where the difference leaves room. Each motor's thrust (N) into left and right.
static void split(const FlipAllocation *r, const FlipAuthority *a, float roll, float collective, float *left, float *right) {
    float least = r->floor * a->weight, most = fmaxf(r->ceiling * a->weight, least);
    float diff = clampf(roll / a->roll, least - most, most - least);
    float total = clampf(fmaxf(collective, fabsf(diff) + 2 * least), fabsf(diff) + 2 * least, 2 * most - fabsf(diff));
    *left = (total + diff) / 2;
    *right = (total - diff) / 2;
}

// The wanted pitch acceleration less the rotors' gyroscopic pitch at these thrusts (FlipAllocation gyro_ff).
static float gyroscopic(const FlipAllocation *r, const FlipAuthority *a, float roll, float pitch, float collective, float roll_rate) {
    if (r->gyro_ff == 0) return pitch;
    float left, right;
    split(r, a, roll, collective, &left, &right);
    return pitch - r->gyro_ff * roll_rate * (sqrtf(fmaxf(right, 0)) - sqrtf(fmaxf(left, 0)));
}

// From the thrust the motors have by now. spread follows √T_right - √T_left as the motors' lag lets it: its rate of change is the rotors' spin momentum turning over between them, which the body takes in yaw as the rotors speed apart in the roll (-dH_z/dt), and with the shafts tilted the pitch rate turns their difference into yaw too (ω_pitch H_x); yaw_ff feeds both forward.
void flip_allocate(const FlipAllocation *r, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float pitch_rate, float *thrust, float *spread, float dt, FlipOutput *out) {
    roll -= a->indi[0] * a->unmodeled[0];
    pitch -= a->indi[1] * a->unmodeled[1];
    yaw -= a->indi[2] * a->unmodeled[2];
    split(r, a, roll, collective, &out->left, &out->right);
    float total = out->left + out->right, keep = expf(-dt / FLIP_MOTOR_TAU);
    *thrust += (total - *thrust) * (1 - keep);
    float have = fmaxf(*thrust, 0.5f), limit = sinf(r->servo_max);
    float common = asinf(clampf(pitch / (a->pitch * have), -limit, limit));
    if (spread && r->yaw_ff != 0) {
        float wanted = sqrtf(fmaxf(out->right, 0)) - sqrtf(fmaxf(out->left, 0)), rate;
        // From the first tick it owns the motors: the split the hold left them in is no change.
        if (isnan(*spread)) *spread = wanted;
        rate = (wanted - *spread) / FLIP_MOTOR_TAU;
        *spread += (wanted - *spread) * (1 - keep);
        yaw += r->yaw_ff * (rate - pitch_rate * sinf(common) * *spread);
    }
    float differential = clampf(yaw / (a->yaw * have * cosf(common)), -DIFFERENTIAL_TILT, DIFFERENTIAL_TILT);
    out->left_tilt = common - differential;
    out->right_tilt = common + differential;
    out->collective = total;
}

// The collective that stops the vertical speed (kv per s), between ARREST_FLOOR and ARREST_MOST of the weight.
static float arrest(const FlipParams *p, const FlipAuthority *a, float vertical) {
    return clampf(a->weight * (1 - p->kv * vertical / GRAVITY), ARREST_FLOOR * a->weight, ARREST_MOST * a->weight);
}

// At the height cap (Flip room): the height risen and what the vertical speed still carries it, braking at what ARREST_FLOOR of the weight leaves of gravity, reach the room less the margin.
static bool capped(const Flip *flip) {
    float rising = fmaxf(flip->vertical, 0);
    return flip->risen + rising * rising / (2 * (1 - ARREST_FLOOR) * GRAVITY) >= flip->room - FLIP_ROOM_MARGIN;
}

// The reference's roll t into it (radians), its rate and its acceleration: up to speed with the acceleration a half sine averaging accel (so it starts and ends at zero, which the motors' lag can follow), coasting, and braking the same way to a stop at turn (up to the speed it can brake from in it), then back to zero the same way. True once it is back.
static bool profile(float turn, float speed, float accel, float t, float *angle, float *rate, float *acc) {
    float top = fminf(speed, sqrtf(accel * turn)), ramp = top / accel, coast = (turn - top * ramp) / top, leg = 2 * ramp + coast;
    float sign = 1, start = 0;
    if (t >= leg) {
        t -= leg;
        sign = -1;
        start = turn;
    }
    if (t >= leg) {
        *angle = *rate = *acc = 0;
        return true;
    }
    // A ramp u into it: the half sine's acceleration, the speed it has given and the angle it has covered.
    float u = t < ramp ? t : leg - t, x = HALF_PI * 2 * fminf(u, ramp) / ramp;
    float a = HALF_PI * accel * sinf(x), v = top / 2 * (1 - cosf(x)), s = top / 2 * (fminf(u, ramp) - ramp * sinf(x) / (2 * HALF_PI));
    if (t < ramp) {
        *angle = s;
        *rate = v;
        *acc = a;
    } else if (t < ramp + coast) {
        *angle = top * ramp / 2 + top * (t - ramp);
        *rate = top;
        *acc = 0;
    } else {
        *angle = turn - s;
        *rate = v;
        *acc = -a;
    }
    *angle = start + sign * *angle;
    *rate *= sign;
    *acc *= sign;
    return false;
}

// One tick of the turn: the reference (the attitude at the start's heading rolled by the profile's angle about its forward axis) tracked by the one attitude law with its rate and acceleration fed forward, through flip_allocate. The collective holds the height: its vertical part is the weight (asking for no more than the weight in all sank a banked burst), and at the height cap, while still rising, ARREST_FLOOR of it, so the climb brakes under the cap. Hands back once the reference is back, within right_angle of upright and under handoff_rate, or at timeout.
static void track(Flip *flip, const FlipParams *p, const FlipAuthority *a, const float *up, const float *gyro, float dt, FlipOutput *out) {
    float angle, rate, accel, ahead, unused;
    float speed = p->ref_rate * sqrtf(fminf(flip->turn / TWO_PI, 1));
    bool done = profile(flip->turn, speed, p->ref_accel, flip->elapsed, &angle, &rate, &accel);
    // The acceleration is fed forward ref_lead ahead, for the motors' and the servos' lag.
    profile(flip->turn, speed, p->ref_accel, flip->elapsed + p->ref_lead, &ahead, &unused, &accel);
    flip->rolled += gyro[0] * dt;
    flip->turned = fmaxf(flip->turned, flip->rolled);
    float level[4], reference[4], error[3], axis[3] = {1, 0, 0};
    quaternion_target(level, flip->heading, 0, 0);
    quaternion_roll(reference, level, angle);
    // The reference's forward axis in the drone's body axes, along which its rate and acceleration are fed forward.
    attitude_error(flip->q, reference, error, axis);
    // The reference waits for a drone that falls behind it (a reference governor): its time runs at 1 less the tilt error's share of track_slack, its rate and acceleration scaled with it, so an authority short of the model's slows the turn instead of leaving the drone short of it.
    float pace = clampf(1 - hypotf(error[0], error[1]) / p->track_slack, 0, 1);
    flip->elapsed += dt * pace;
    rate *= pace;
    accel *= pace * pace;
    float rates[3] = {rate * axis[0], rate * axis[1], rate * axis[2]}, accels[3] = {accel * axis[0], accel * axis[1], accel * axis[2]}, wanted[3];
    AttitudeGains g = {p->track_kp, p->brake, p->max_rate, p->track_heading, {p->track_roll, p->track_pitch, p->track_yaw}};
    attitude_rates(&g, error, rates, accels, gyro, wanted);
    float collective = a->weight / fmaxf(up[2], 0.5f);
    if (capped(flip) && flip->vertical > 0) collective = ARREST_FLOOR * a->weight / fmaxf(up[2], 0.5f);
    FlipAllocation r = {p->servo_max, p->floor, p->ceiling, p->gyro_ff * (1 - a->indi[1]), p->yaw_ff * (1 - a->indi[2])};
    flip_allocate(&r, a, wanted[0], gyroscopic(&r, a, wanted[0], wanted[1], collective, gyro[0]), wanted[2], collective, gyro[1], &flip->thrust, &flip->spread, dt, out);
    out->owned = true;
    bool slow = fabsf(gyro[0]) < p->handoff_rate && fabsf(gyro[1]) < p->handoff_rate && fabsf(gyro[2]) < p->handoff_rate;
    if ((done && up[2] > cosf(p->right_angle) && slow) || flip->time > p->timeout) {
        flip->stage = FLIP_ARREST;
        flip->time = 0;
        out->owned = false;
        out->handed_back = true;
        out->collective_set = true;
        out->collective = arrest(p, a, flip->vertical);
    }
}

const char *flip_refusal(const FlipCheck *c) {
    if (!c->enabled) return "Flip is off (setting flip_angle)";
    if (!c->armed) return "Not armed";
    if (c->done) return "One flip per arming: land and arm again";
    if (!c->holding || c->landing) return "Flip only from the hover hold";
    if (c->camera || !c->gps) return "Flip only from the GPS hold (hover_camera 0, hover_gps_tau on)";
    if (!c->aligned || !c->located) return "GPS hold not settled: heading not pinned or no fresh fix";
    if (c->failsafe) return "Fly-away failsafe active";
    if (c->pattern) return "Test pattern playing";
    if (c->airborne < FLIP_SETTLE_S) return "Not airborne long enough";
    if (c->height < c->min_height) return "Below flip_height";
    if (fabsf(c->climb_rate) > FLIP_CLIMB_RATE) return "Still climbing or sinking";
    if (c->speed > FLIP_SPEED) return "Moving over the ground";
    if (fabsf(c->roll) > FLIP_LEVEL || fabsf(c->pitch) > FLIP_LEVEL) return "Not level";
    if (fabsf(c->rate[0]) > FLIP_STILL || fabsf(c->rate[1]) > FLIP_STILL || fabsf(c->rate[2]) > FLIP_STILL) return "Not still";
    if (c->battery < FLIP_MIN_V) return "Battery under 14.8 V";
    return NULL;
}

void flip_reset(FlipRun *run) {
    *run = (FlipRun){.since = -1, .params = &flip_defaults};
}

void flip_begin(FlipRun *run, const Controller *controller, float height_integral, const float *q, float collective, float climb_rate, float height, float angle, float ceiling) {
    run->done = true;
    run->since = -1;
    run->saved = *controller;
    run->height_integral = height_integral;
    run->flip = (Flip){.stage = FLIP_TURN, .thrust = collective, .vertical = climb_rate, .turn = angle, .spread = NAN,
                       .room = fminf(ceiling, FLIP_CEILING) - height};
    for (int i = 0; i < 4; i++) run->flip.q[i] = q[i];
}

void flip_cancel(FlipRun *run) {
    run->flip.stage = FLIP_IDLE;
    run->since = -1;
}

bool flip_active(const FlipRun *run) {
    return run->flip.stage != FLIP_IDLE;
}

bool flip_gyro_only(const FlipRun *run) {
    return flip_active(run) || (run->since >= 0 && run->since < FLIP_GRACE);
}

int flip_phase(const FlipRun *run) {
    return run->flip.stage == FLIP_TURN ? 2 : run->flip.stage == FLIP_ARREST ? 3 : 0;
}

void flip_tick(FlipRun *run, const FlipAuthority *a, const float *q, const float *gyro, const float *accel, float dt, Controller *controller, float *height_integral, FlipOutput *out) {
    *out = (FlipOutput){0};
    if (run->since >= 0) run->since += dt;
    Flip *flip = &run->flip;
    if (flip->stage == FLIP_IDLE) return;
    const FlipParams *p = run->params;
    for (int i = 0; i < 4; i++) flip->q[i] = q[i];
    // The reference turns about the heading the drone has at its first tick.
    if (flip->stage == FLIP_TURN && flip->time == 0) flip->heading = quaternion_heading(q);
    flip->time += dt;
    flip->risen += flip->vertical * dt;
    float up[3];
    quaternion_up(flip->q, up);
    flip->vertical += ((up[0] * accel[0] + up[1] * accel[1] + up[2] * accel[2]) - 1) * GRAVITY * dt;
    if (flip->stage == FLIP_ARREST) {
        out->collective_set = true;
        out->collective = arrest(p, a, flip->vertical);
        flip->arrested = fabsf(flip->vertical) < ARREST_DONE ? flip->arrested + dt : 0;
        if (flip->arrested >= ARREST_HOLD || flip->time > p->arrest_time) {
            flip->stage = FLIP_IDLE;
            *height_integral = run->height_integral;
        }
        return;
    }
    track(flip, p, a, up, gyro, dt, out);
    if (out->handed_back) {
        flight_restore(controller, &run->saved);
        run->since = 0;
    }
}

void flip_limit(FlipRun *run, bool recovering, float *throttle, float dt) {
    const FlipParams *p = run->params;
    bool limited = flip_gyro_only(run) || recovering;
    for (int side = 0; side < 2; side++) {
        if (limited && throttle[side] > 0) {
            float last = run->throttle[side], step = p->slew * dt;
            throttle[side] = fminf(clampf(throttle[side], last - step, last + step), p->throttle_cap);
        }
        run->throttle[side] = throttle[side];
    }
}

void flip_actuators(const FlipOutput *out, const Command *c, float left_scale, float right_scale, float throttle_max, float servo_limit, float *throttle, float *servo) {
    float thrust[2] = {fmaxf(out->left, 0) / left_scale, fmaxf(out->right, 0) / right_scale};
    float tilt[2] = {c->left_trim + c->model.left_servo_direction * out->left_tilt, c->right_trim + c->model.right_servo_direction * out->right_tilt};
    for (int side = 0; side < 2; side++) {
        throttle[side] = clampf(roundf(flight_interpolate(thrust[side], &flight_thrust[0][0], flight_thrust_count, 1)), 1, throttle_max);
        servo[side] = clampf(tilt[side] / DEGREE, -servo_limit, servo_limit);
    }
}
