#include "flip.h"

#include <math.h>

#define TWO_PI 6.28318531f
#define HALF_PI 1.57079633f
// How fast the flip's idea of the motors' thrust follows what it asks (s; the motors' lag).
#define FLIP_MOTOR_TAU 0.1f
#define GRAVITY 9.81f
// The arrest ends once the vertical speed has stayed under ARREST_DONE (m/s) for ARREST_HOLD (s).
#define ARREST_DONE 0.2f
#define ARREST_HOLD 0.3f
#define ARREST_FLOOR 0.6f
#define DEGREE (3.14159265f / 180)
// The flip request (flip_refusal) wants the hold settled this long after liftoff (s), within FLIP_LEVEL of level and turning under FLIP_STILL on every axis, its climb rate under FLIP_CLIMB_RATE and the GPS's ground speed under FLIP_SPEED (m/s), and the pack at FLIP_MIN_V or more: the climb asks for 2.1 times the weight, and a sagging pack gives less (3.7 V a cell under load on 4S).
#define FLIP_SETTLE_S 5.0f
#define FLIP_LEVEL (10 * DEGREE)
#define FLIP_STILL (30 * DEGREE)
#define FLIP_CLIMB_RATE 0.3f
#define FLIP_SPEED 1.0f
#define FLIP_MIN_V 14.8f
// The flip's own authority (FlipAuthority), from the airframe with the pivots raised 20 mm (sw/sim/models/airframe_2026-11-12_raised.json at geometry_2026-11-12_raised.json), not from the settings' model, which the attitude controller keeps and which predates the raise (pivot_height 0.02 m, pitch_inertia 0.0023: 8.7 per N per radian of pitch, 16.1 of yaw): roll, the arm over the roll inertia (0.172 / 0.01275); pitch, the tilt's effective lever above the CoM (the pivot 39.1 mm above it less the slipstream's 2 mm, 37.2 mm) over the pitch inertia (0.00525); yaw, the 30% of the tilted thrust the slipstream leaves at the arm over the yaw inertia (0.302 x 0.172 / 0.01115). The flip's gains are tuned against these.
#define FLIP_ROLL_AUTHORITY 13.5f
#define FLIP_PITCH_AUTHORITY 7.08f
#define FLIP_YAW_AUTHORITY 4.66f

const FlipParams flip_defaults = {
    .climb_thrust = 1.8954f, .climb_time = 0.35f, .spin_rate = 11.9178f, .k_spin = 11.9756f, .spin_thrust = 0.8693f, .handoff = 1.9648f,
    .kp = 4.0f, .brake = 8.0696f, .max_rate = 8.5717f, .k_roll = 7.7428f, .k_pitch = 8.525f, .kd_yaw = 2.128f,
    .flip_thrust = 1.7828f, .right_angle = 0.5831f, .servo_max = 0.5092f, .roll_over = 2.8798f, .kv = 15.0f, .arrest_time = 2.0f, .timeout = 1.9751f,
    .floor = 0.2645f, .ceiling = 1.2286f, .handoff_rate = 3.0f, .slew = 10000.0f, .throttle_cap = 1100.0f, .spin_lift = 0.6312f,
    .spin_pitch = 8.217f, .spin_yaw = 9.34f, .gyro_ff = 8.8625f, .yaw_ff = 1.6f,
    .ref_accel = 26.47f, .track_kp = 6.525f, .track_heading = 0.384f, .track_roll = 13.16f, .track_pitch = 2.904f, .track_yaw = 2.984f, .ref_lead = 0.0701f, .track_slack = 1.212f, .ref_rate = 11.42f,
};

void flip_start(Flip *flip, float hover_thrust, float roll, float pitch, float vertical) {
    *flip = (Flip){.stage = FLIP_CLIMB, .thrust = hover_thrust, .vertical = vertical, .turn = TWO_PI, .spread = NAN};
    quaternion_from_euler(flip->q, roll, pitch);
}

// Each motor between floor and ceiling of the weight: the difference within what that allows, then the collective as asked where the difference leaves room. Each motor's thrust (N) into left and right.
static void split(const FlipRighting *r, const FlipAuthority *a, float roll, float collective, float *left, float *right) {
    float least = r->floor * a->weight, most = fmaxf(r->ceiling * a->weight, least);
    float diff = clampf(roll / a->roll, least - most, most - least);
    float total = clampf(fmaxf(collective, fabsf(diff) + 2 * least), fabsf(diff) + 2 * least, 2 * most - fabsf(diff));
    *left = (total + diff) / 2;
    *right = (total - diff) / 2;
}

// The wanted pitch acceleration less the rotors' gyroscopic pitch at these thrusts (FlipRighting gyro_ff).
static float gyroscopic(const FlipRighting *r, const FlipAuthority *a, float roll, float pitch, float collective, float roll_rate) {
    if (r->gyro_ff == 0) return pitch;
    float left, right;
    split(r, a, roll, collective, &left, &right);
    return pitch - r->gyro_ff * roll_rate * (sqrtf(fmaxf(right, 0)) - sqrtf(fmaxf(left, 0)));
}

// recover.py's allocation, from the thrust the motors have by now. spread follows √T_right - √T_left as the motors' lag lets it (NULL without yaw_ff): its rate of change is the rotors' spin momentum turning over between them, which the body takes in yaw as the rotors speed apart in the flip's roll (-dH_z/dt), and with the shafts tilted the pitch rate turns their difference into yaw too (ω_pitch H_x); yaw_ff feeds both forward.
void flip_allocate(const FlipRighting *r, const FlipAuthority *a, float roll, float pitch, float yaw, float collective, float pitch_rate, float *thrust, float *spread, float dt, FlipOutput *out) {
    if (a->indi > 0) {
        roll -= a->indi * a->unmodeled[0];
        pitch -= a->indi * a->unmodeled[1];
        yaw -= a->indi * a->unmodeled[2];
    }
    split(r, a, roll, collective, &out->left, &out->right);
    float total = out->left + out->right, keep = expf(-dt / FLIP_MOTOR_TAU);
    *thrust += (total - *thrust) * (1 - keep);
    float have = fmaxf(*thrust, 0.5f), limit = sinf(r->servo_max);
    float common = asinf(clampf(pitch / (a->pitch * have), -limit, limit));
    if (spread && r->yaw_ff != 0) {
        float wanted = sqrtf(fmaxf(out->right, 0)) - sqrtf(fmaxf(out->left, 0)), rate;
        // From the first tick the flip owns the motors: the split the hold left them in is no change.
        if (isnan(*spread)) *spread = wanted;
        rate = (wanted - *spread) / FLIP_MOTOR_TAU;
        *spread += (wanted - *spread) * (1 - keep);
        yaw += r->yaw_ff * (rate - pitch_rate * sinf(common) * *spread);
    }
    // A wider bound (0.5 rad, which servo_max's common tilt and SERVO_LIMIT's 60 degrees would allow) left the bursts' yaw as it was and cost the flip (rotor_momentum 0.04-0.05 with the CoM 15.5 mm left: 79% clean to 60%): the rotors' yaw is more than the tilt can take back, and the wider tilt couples into pitch (sw/sim/models/NOTES.md "Yaw in the flip").
    float differential = clampf(yaw / (a->yaw * have * cosf(common)), -0.3f, 0.3f);
    out->left_tilt = common - differential;
    out->right_tilt = common + differential;
    out->collective = total;
}

// recover.py's righting.
void flip_righting(const FlipRighting *r, const FlipAuthority *a, const float *up, const float *gyro, float collective, float *thrust, float *spread, float dt, FlipOutput *out) {
    float sine = hypotf(up[0], up[1]), angle = atan2f(sine, up[2]);
    float axis[2] = {sine > 1e-6f ? -up[1] / sine : 1, sine > 1e-6f ? up[0] / sine : 0};
    if (up[2] < cosf(r->roll_over) && fabsf(up[0]) > fabsf(up[1])) {
        angle = atan2f(fabsf(up[1]), up[2]);
        axis[0] = up[1] > 0 ? -1 : 1;
        axis[1] = 0;
    }
    float rate = fminf(fminf(r->max_rate, sqrtf(2 * r->brake * angle)), r->kp * angle);
    float roll = r->k_roll * (axis[0] * rate - gyro[0]);
    flip_allocate(r, a, roll, gyroscopic(r, a, roll, r->k_pitch * (axis[1] * rate - gyro[1]), collective, gyro[0]), -r->kd_yaw * gyro[2], collective, gyro[1], thrust, spread, dt, out);
}

static FlipRighting righting(const FlipParams *p) {
    return (FlipRighting){p->kp, p->brake, p->max_rate, p->k_roll, p->k_pitch, p->kd_yaw, p->roll_over, p->servo_max, p->floor, p->ceiling, p->gyro_ff, p->yaw_ff};
}

// The collective that stops the vertical speed, at least ARREST_FLOOR of the weight: the attitude controller flies the arrest and needs thrust to do it (a flip or burst that climbed had cut it to 0.15 of the weight and tipped over in the sim).
static float arrest(const FlipParams *p, const FlipAuthority *a, float vertical) {
    return clampf(a->weight * (1 - p->kv * vertical / GRAVITY), ARREST_FLOOR * a->weight, 2.2f * a->weight);
}

// The quaternion flip at its height cap (Flip room): the height risen and what the vertical speed still carries it, braking at what ARREST_FLOOR of the weight leaves of gravity, reach the room less the margin.
static bool capped(const Flip *flip) {
    float rising = fmaxf(flip->vertical, 0);
    return flip->risen + rising * rising / (2 * (1 - ARREST_FLOOR) * GRAVITY) >= flip->room - FLIP_ROOM_MARGIN;
}

// The quaternion flip's reference roll t into it (radians), its rate and its acceleration: up to speed with the acceleration a half sine averaging accel (so it starts and ends at zero, which the motors' lag can follow), coasting, and braking the same way to a stop at turn (a shorter turn, up to the speed it can brake from in it); a burst (back) then comes back to zero the same way. True once it has stopped at its end.
static bool profile(float turn, float speed, float accel, bool back, float t, float *angle, float *rate, float *acc) {
    float top = fminf(speed, sqrtf(accel * turn)), ramp = top / accel, coast = (turn - top * ramp) / top, leg = 2 * ramp + coast;
    float sign = 1, start = 0;
    if (back && t >= leg) {
        t -= leg;
        sign = -1;
        start = turn;
    }
    if (t >= leg) {
        *angle = back ? 0 : turn;
        *rate = *acc = 0;
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

// The quaternion flip from the climb's end: one reference rotation, the attitude at the climb's heading rolled by the profile's angle about its forward axis, out to the whole turn (the flip) or out and back (a burst), at ref_rate times the square root of the turn's share of a whole one; the one attitude law (attitude_error, attitude_rates) tracks it with the reference's rate and acceleration fed forward, through flip_allocate as the staged laws were. The flip's collective is the spin's (spin_thrust plus spin_lift times the reference's cosine) while the reference moves and flip_thrust once it has stopped, its vertical part at most the weight, a burst's what holds its height; then, within right_angle of upright and under handoff_rate, or at timeout, it hands back as the staged flip does. No roll-over, no separate spin, righting or Euler hand-back.
static void track(Flip *flip, const FlipParams *p, const FlipAuthority *a, const FlipRighting *r, const float *up, const float *gyro, float dt, FlipOutput *out) {
    float angle, rate, accel;
    float speed = p->ref_rate * sqrtf(fminf(flip->turn / TWO_PI, 1)), ahead, unused;
    bool done = profile(flip->turn, speed, p->ref_accel, flip->turn < TWO_PI, flip->elapsed, &angle, &rate, &accel);
    // The acceleration is fed forward ref_lead ahead, for the motors' and the servos' lag.
    profile(flip->turn, speed, p->ref_accel, flip->turn < TWO_PI, flip->elapsed + p->ref_lead, &ahead, &unused, &accel);
    // The roll turned (on the gyro since the reference began) at its furthest, as the staged spin counts it.
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
    // The thrust's vertical part is never more than the weight: a burst holds its height with exactly that (asking for no more than the weight in all sank it while banked), the flip has its schedule within it; and at the height cap, while still rising, at most ARREST_FLOOR of it, so the climb brakes under it.
    float hold = a->weight / fmaxf(up[2], 0.5f);
    float collective = flip->turn < TWO_PI ? hold : fminf(fmaxf(done ? p->flip_thrust : p->spin_thrust + p->spin_lift * cosf(angle), 0) * a->weight, hold);
    if (capped(flip) && flip->vertical > 0) collective = fminf(collective, ARREST_FLOOR * a->weight / fmaxf(up[2], 0.5f));
    flip_allocate(r, a, wanted[0], gyroscopic(r, a, wanted[0], wanted[1], collective, gyro[0]), wanted[2], collective, gyro[1], &flip->thrust, &flip->spread, dt, out);
    flip->stage = done ? FLIP_RIGHT : FLIP_SPIN;
    bool slow = fabsf(gyro[0]) < p->handoff_rate && fabsf(gyro[1]) < p->handoff_rate && fabsf(gyro[2]) < p->handoff_rate;
    if ((done && up[2] > cosf(p->right_angle) && slow) || flip->time > p->timeout) {
        flip->stage = FLIP_ARREST;
        flip->time = 0;
        out->owned = false;
        out->handed_back = true;
        quaternion_euler(flip->q, &out->roll, &out->pitch);
        out->collective_set = true;
        out->collective = arrest(p, a, flip->vertical);
    }
}

void flip_step(Flip *flip, const FlipParams *p, const FlipAuthority *a, const float *gyro, const float *accel, float dt, FlipOutput *out) {
    *out = (FlipOutput){0};
    if (flip->stage == FLIP_IDLE) return;
    flip->time += dt;
    float *q = flip->q;
    // The quaternion flip's q is the attitude estimate's, set by flip_tick.
    if (!(flip->quaternion > 0)) quaternion_rotate(q, gyro, dt);
    else flip->risen += flip->vertical * dt;
    // Up in body axes: the body's forward and left axes' height.
    float up[3];
    quaternion_up(q, up);
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
        if (flip->time >= p->climb_time || (flip->quaternion > 0 && capped(flip))) flip->stage = FLIP_SPIN;
        return;
    }
    out->owned = true;
    FlipRighting r = righting(p);
    r.gyro_ff *= 1 - a->indi;
    r.yaw_ff *= 1 - a->indi;
    // A staged burst (a turn short of the whole flip) never turns its thrust far from up, so it asks for no more than the weight: the flip's spin and righting collectives took a 60-degree burst 7 m up.
    float burst = flip->turn < TWO_PI ? a->weight : INFINITY;
    if (flip->quaternion > 0) {
        track(flip, p, a, &r, up, gyro, dt, out);
        return;
    }
    if (flip->stage == FLIP_SPIN) {
        flip->turned += gyro[0] * dt;
        // Keep the roll axis level: the nose's height (up's forward component, positive nose up) driven to zero through whichever of pitch and yaw tilts it now.
        float nose = up[0];
        float pitch_rate = p->kp * nose * up[2], yaw_rate = -p->kp * nose * up[1];
        float roll = p->k_spin * (p->spin_rate * sqrtf(flip->turn / TWO_PI) - gyro[0]), collective = fminf(fmaxf(p->spin_thrust + p->spin_lift * cosf(flip->turned), 0) * a->weight, burst);
        flip_allocate(&r, a, roll, gyroscopic(&r, a, roll, p->spin_pitch * (pitch_rate - gyro[1]), collective, gyro[0]), p->spin_yaw * (yaw_rate - gyro[2]), collective, gyro[1], &flip->thrust, &flip->spread, dt, out);
        if (flip->turned >= flip->turn * (1 - p->handoff / TWO_PI)) flip->stage = FLIP_RIGHT;
    } else {
        flip_righting(&r, a, up, gyro, fminf(p->flip_thrust * a->weight, burst), &flip->thrust, &flip->spread, dt, out);
        bool slow = fabsf(gyro[0]) < p->handoff_rate && fabsf(gyro[1]) < p->handoff_rate && fabsf(gyro[2]) < p->handoff_rate;
        if ((up[2] > cosf(p->right_angle) && slow) || flip->time > p->timeout) {
            // Back to the attitude controller, with the attitude as flight_attitude keeps it.
            flip->stage = FLIP_ARREST;
            flip->time = 0;
            out->owned = false;
            out->handed_back = true;
            quaternion_euler(q, &out->roll, &out->pitch);
            out->collective_set = true;
            out->collective = arrest(p, a, flip->vertical);
        }
    }
    if (flip->stage == FLIP_SPIN && flip->time > p->timeout) flip->stage = FLIP_RIGHT;
}

float flip_turn(float enable) {
    return enable >= 3 ? TWO_PI / 4 : enable >= 2 ? TWO_PI / 6 : TWO_PI;
}

const char *flip_refusal(const FlipCheck *c) {
    if (!c->enabled) return "Flip is off (setting flip_enable)";
    if (!c->armed) return "Not armed";
    if (c->done) return "One flip per arming: land and arm again";
    if (c->whole_quaternion) return "The whole flip is not flown with quaternion 1 (bursts only: flip_enable 2 or 3)";
    if (!c->holding || c->landing) return "Flip only from the hover hold";
    if (c->camera || !c->gps) return "Flip only from the GPS hold (hover_camera 0, hover_gps_tau on)";
    if (c->ekf) return "Flip only with the hold on its own estimates (hover_ekf 0)";
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

void flip_begin(FlipRun *run, const Command *command, const Controller *controller, float height_integral, float collective, float roll, float pitch, float climb_rate, float height) {
    run->authority = (FlipAuthority){.roll = FLIP_ROLL_AUTHORITY, .pitch = FLIP_PITCH_AUTHORITY, .yaw = FLIP_YAW_AUTHORITY, .weight = command->hover.mass * GRAVITY};
    run->done = true;
    run->since = -1;
    run->saved = *controller;
    run->height_integral = height_integral;
    flip_start(&run->flip, collective, roll, pitch, climb_rate);
    run->flip.turn = flip_turn(command->flip_enable);
    run->flip.quaternion = command->quaternion > 0;
    run->flip.room = fminf(command->hover.ceiling, FLIP_CEILING) - height;
}

void flip_cancel(FlipRun *run) {
    run->flip.stage = FLIP_IDLE;
    run->since = -1;
}

bool flip_abort(FlipRun *run, float *height_integral) {
    if (run->flip.stage != FLIP_CLIMB) return false;
    run->flip.stage = FLIP_IDLE;
    *height_integral = run->height_integral;
    return true;
}

bool flip_active(const FlipRun *run) {
    return run->flip.stage != FLIP_IDLE;
}

static bool grace(const FlipRun *run) {
    return run->since >= 0 && run->since < FLIP_GRACE;
}

bool flip_gyro_only(const FlipRun *run) {
    return flip_active(run) || grace(run);
}

bool flip_tip_exempt(const FlipRun *run) {
    return run->flip.stage == FLIP_SPIN || run->flip.stage == FLIP_RIGHT || grace(run);
}

int flip_phase(const FlipRun *run) {
    int stage = run->flip.stage;
    return stage == FLIP_CLIMB ? 1 : stage == FLIP_SPIN || stage == FLIP_RIGHT ? 2 : stage == FLIP_ARREST ? 3 : 0;
}

void flip_tick(FlipRun *run, const float *gyro, const float *accel, float dt, Attitude *attitude, Controller *controller, float *height_integral, FlipOutput *out) {
    if (run->since >= 0) run->since += dt;
    bool active = flip_active(run);
    bool quaternion = run->flip.quaternion > 0;
    if (quaternion && active) {
        // The quaternion flip flies the attitude estimate itself (on the gyro alone from the climb), at the heading the climb ends on.
        for (int i = 0; i < 4; i++) run->flip.q[i] = attitude->q[i];
        if (run->flip.stage == FLIP_CLIMB) run->flip.heading = quaternion_heading(run->flip.q);
    }
    flip_step(&run->flip, run->params, &run->authority, gyro, accel, dt, out);
    if (out->handed_back) {
        if (!quaternion) {
            attitude->roll = out->roll;
            attitude->pitch = out->pitch;
        }
        flight_restore(controller, &run->saved);
        run->since = 0;
    }
    if (active && !flip_active(run)) *height_integral = run->height_integral;
    if (quaternion) return;
    // While the estimate is on the gyro alone it is the flip's own quaternion's, through the arrest and the grace too, where the attitude controller flies: Euler angles carried on the gyro jump at 90 degrees of pitch (02:27: roll 58 to 163), and from a quaternion they come back with pitch within ±90 degrees, unambiguous within 90 degrees of upright.
    if (!active && grace(run)) quaternion_rotate(run->flip.q, gyro, dt);
    if (flip_gyro_only(run)) quaternion_euler(run->flip.q, &attitude->roll, &attitude->pitch);
}

void flip_limit(FlipRun *run, bool recovering, float *throttle, float dt) {
    const FlipParams *p = run->params;
    bool limited = flip_active(run) || grace(run) || recovering;
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
