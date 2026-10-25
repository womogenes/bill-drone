#include <math.h>
#include <string.h>

#include "hover.h"

#define GRAVITY 9.81f
// A tag unseen this long is lost: the range, place and heading holds stop, horizontal velocity is dead-reckoned and damped, and height runs on the IMU alone.
#define LOST_US 250000
// How fast the camera corrects the velocity (s); sightings further apart than GAP_US restart it.
#define VELOCITY_TAU 0.4f
#define GAP_US 200000
// Without the tag the dead-reckoned velocity leaks toward zero over this long (s), as the trim never quite matches the push it cancels.
#define DRIFT_TAU 3.0f
// The body-fixed trim's integration time (s) and its limit (m/s², about 3 degrees of tilt).
#define TRIM_TAU 4.0f
#define MAX_TRIM 0.5f
// Each sighting with a settled tilt moves the wall-normal estimate this fraction of the way to its own.
#define NORMAL_WEIGHT 0.1f
// The place on the circle follows that estimate over this long (s), at most at the approach speed.
#define ARC_TAU 2.0f
// How fast the camera pulls the IMU's height and climb rate toward its own (s).
#define CAMERA_TAU 0.15f
// Without the camera the integrated climb rate leaks over this long (s), so an accelerometer bias cannot run away: airborne in hold toward the last climb-rate command, so a long climb keeps the speed it really has; on the ground (where the drone cannot follow the command) and landing toward zero, so touchdown still shows as a stalled descent.
#define LEAK_TAU 10.0f
// Resting specific force smoothing (s).
#define BIAS_TAU 0.5f
// Fastest commanded climb or descent from the height error (m/s), most vertical acceleration asked for (m/s²), and the integral's limit (m/s²).
#define MAX_CLIMB 0.5f
#define MAX_VERTICAL 3.0f
#define MAX_INTEGRAL 3.0f
// Landing: thrust below this fraction of weight while the climb rate stays within TOUCHDOWN_RATE (m/s) for TOUCHDOWN_S means the ground holds the drone (in the air that thrust would show as falling); thrust then falls to zero over LANDED_RAMP (s).
#define TOUCHDOWN_FRACTION 0.85f
#define TOUCHDOWN_RATE 0.1f
#define TOUCHDOWN_S 0.3f
#define LANDED_RAMP 0.5f
// Below this height (m) a hold is still on the ground taking off: the IMU climb rate leaks toward zero, and centring neither tilts nor turns and lets the range target follow the drone. The tag-height target never asks for less than MIN_HEIGHT above the engage point (m).
#define AIRBORNE 0.05f
#define MIN_HEIGHT 0.2f
// Rotor drag: the servos' dead time beyond the tick the runtime's tilt is already late by (us; sw/sim/sweep.py FITTED has 70 ms), how slowly the camera teaches the drag model its offset (s), and that offset's limit (m/s).
#define SERVO_DELAY_US 60000
#define OFFSET_TAU 5.0f
#define MAX_OFFSET 1.0f
// The integrated acceleration's bias the drag model may find (m/s², about 6 degrees of attitude error).
#define MAX_BIAS 1.0f

static float clamp(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

static float wrap(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

void hover_rest(HoverState *state, float vertical_g, bool spinning, float dt) {
    float bias = state->bias, climb_rate = 0, height = 0;
    if (!spinning) {
        bias = bias ? bias + (vertical_g - bias) * (1 - expf(-dt / BIAS_TAU)) : vertical_g;
    } else if (bias) {
        // Manual flight: keep the climb rate and height current (leaking like the no-camera hover), so a hover or landing taken over mid-flight starts from the real climb, not from rest.
        climb_rate = (state->climb_rate + (vertical_g - bias) * GRAVITY * dt) * expf(-dt / LEAK_TAU);
        height = state->height + climb_rate * dt;
    }
    memset(state, 0, sizeof(*state));
    state->bias = bias;
    state->climb_rate = climb_rate;
    state->height = height;
}

// The newest recorded tick no later than time (the oldest kept if none is).
static int recorded(const HoverState *state, int64_t time_us) {
    int pick = (state->next + HOVER_HISTORY - 1) % HOVER_HISTORY;
    for (int i = 1; i < HOVER_HISTORY; i++) {
        int k = (state->next + HOVER_HISTORY - 1 - i) % HOVER_HISTORY;
        if (state->history_us[pick] <= time_us || !state->history_us[k]) break;
        pick = k;
    }
    return pick;
}

// A camera-frame vector in the level frame turned by yaw (forward, left, up at yaw 0), with the attitude at capture.
static void level(const HoverState *state, const HoverGains *g, int64_t capture_us, const float *in, float *out) {
    const float *then = state->history[recorded(state, capture_us)];
    float roll = then[0], pitch = then[1], yaw = then[2];
    // Camera (right, down, forward), tilted up by cam_pitch, to body (forward, left, up).
    float ca = cosf(g->cam_pitch), sa = sinf(g->cam_pitch);
    float right = in[0], down = in[1], ahead = in[2];
    float x = down * sa + ahead * ca, y = -right, z = ahead * sa - down * ca;
    // Level the body: roll about forward (right side down), then pitch about left (nose down).
    float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
    float y1 = cr * y - sr * z, z1 = sr * y + cr * z;
    float x2 = cp * x + sp * z1, z2 = -sp * x + cp * z1;
    // Turn by the heading so the drone's own turning does not look like travel.
    float cy = cosf(yaw), sy = sinf(yaw);
    out[0] = cy * x2 - sy * y1;
    out[1] = sy * x2 + cy * y1;
    out[2] = z2;
}

static void take(HoverState *state, const HoverGains *g, const HoverPose *pose) {
    float position[3];
    level(state, g, pose->capture_us, pose->tag, position);
    // A settled tilt gives the wall's normal: its heading in the level frame, smoothed over many frames, is where the drone should be looking from. A sighting or two with the wrong tilt only nudges it.
    if (pose->normal[0] || pose->normal[1] || pose->normal[2]) {
        float normal[3];
        level(state, g, pose->capture_us, pose->normal, normal);
        float heading = atan2f(normal[1], normal[0]);
        state->normal = state->have_normal ? wrap(state->normal + NORMAL_WEIGHT * wrap(heading - state->normal)) : heading;
        state->have_normal = true;
    }
    int64_t interval_us = pose->capture_us - state->capture_us;
    if (state->have_pose && interval_us > 0 && interval_us <= GAP_US) {
        float interval = interval_us / 1e6f, keep = expf(-interval / VELOCITY_TAU);
        // The tag stands still, so its apparent motion is the drone's, reversed: the average velocity over the interval, compared with the estimate at its middle. The horizontal estimate runs on tilt between sightings, so the correction moves it and its recorded past alike.
        float *then = state->history[recorded(state, pose->capture_us - interval_us / 2)];
        for (int i = 0; i < 2; i++) {
            float correction = (1 - keep) * ((state->position[i] - position[i]) / interval - then[3 + i]);
            state->velocity[i] += correction;
            for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3 + i] += correction;
        }
        state->velocity[2] = keep * state->velocity[2] + (1 - keep) * (state->position[2] - position[2]) / interval;
    } else {
        // Without a recent sighting to compare, the dead-reckoned velocity stands if there is one (always, with the drag model bounding it).
        if (!state->tracking && !g->drag) {
            state->velocity[0] = state->velocity[1] = 0;
            for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3] = state->history[k][4] = 0;
        }
        state->velocity[2] = 0;
        // First sight (or after a gap): the camera's current height is the IMU's current height.
        state->tag_height = state->height + position[2];
    }
    memcpy(state->position, position, sizeof(state->position));
    state->sequence = pose->sequence;
    state->capture_us = pose->capture_us;
    state->have_pose = true;
}

void hover_update(HoverState *state, const HoverGains *g, int mode, const HoverPose *pose, int64_t now_us, float dt,
                  float roll, float pitch, float yaw_rate, float vertical_g, float force, const HoverDrag *drag, HoverOutput *output) {
    float weight = g->mass * GRAVITY;
    bool dragging = g->drag > 0 && drag;
    if (!state->engaged) {
        state->engaged = true;
        // Continue from the thrust being flown, so taking over a manual flight does not step.
        state->integral = force > 0.5f * weight ? clamp(force / g->mass - GRAVITY, -MAX_INTEGRAL, MAX_INTEGRAL) : 0;
        state->force = force;
        // Asked to land without enough thrust to be flying: it is already down.
        if (mode != HOVER_HOLD && force < 0.5f * weight) state->landed = true;
    }
    if (state->landed) {
        state->force = fmaxf(0, state->force - weight / LANDED_RAMP * dt);
        *output = (HoverOutput){.force = state->force};
        return;
    }
    // Heading from the gyro; horizontal velocity from tilt (g tan(tilt) less the push the trim cancels, once airborne, as on the ground a resting tilt pushes nothing); both recorded with the attitude.
    state->yaw = wrap(state->yaw + yaw_rate * dt);
    float cy = cosf(state->yaw), sy = sinf(state->yaw);
    // With the drag model, the measured specific force turned level instead, less the acceleration bias the drag model has found in it: it carries the real thrust tilt and the drag, and its level-reference error cancels against the attitude's (both are relative to pose_zero); what remains is mostly the attitude estimate's own drift.
    if (state->height >= AIRBORNE) {
        float ahead, left;
        if (dragging) {
            const float *f = drag->specific;
            float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
            ahead = GRAVITY * (cp * f[0] + sp * (sr * f[1] + cr * f[2])) - state->drag_bias[0];
            left = GRAVITY * (cr * f[1] - sr * f[2]) - state->drag_bias[1];
        } else {
            ahead = GRAVITY * tanf(pitch - g->pitch_trim) - state->trim[0];
            left = -GRAVITY * tanf(roll) - state->trim[1];
        }
        state->velocity[0] += (ahead * cy - left * sy) * dt;
        state->velocity[1] += (ahead * sy + left * cy) * dt;
    }
    float *now = state->history[state->next];
    state->history_us[state->next] = now_us;
    now[0] = roll;
    now[1] = pitch;
    now[2] = state->yaw;
    now[3] = state->velocity[0];
    now[4] = state->velocity[1];
    now[5] = dragging ? drag->tilt : 0;
    state->next = (state->next + 1) % HOVER_HISTORY;

    // Rotor drag: the body's horizontal specific force less the thrust vector's forward part (at the tilt the servos have reached) is -drag times the airspeed. It is noisy (vibration, the pitch loop's servo motion), so it pulls the estimate slowly, and the past with it, as the camera's correction does; a steady disagreement is the integrated acceleration's bias, which it learns (a critically damped second-order filter, settling over a few drag_tau). The camera, when it sees the tag, measures the velocity itself: then the disagreement is the drag model's own offset (servo zero, accelerometer bias, mounting) instead, which it teaches below.
    bool seeing = state->have_pose && now_us - state->seen_us <= LOST_US;
    float measured[2] = {0, 0};
    if (dragging && state->height >= AIRBORNE) {
        const float *f = drag->specific;
        float tilt = state->history[recorded(state, now_us - SERVO_DELAY_US)][5] + g->drag_tilt;
        measured[0] = -GRAVITY * (f[0] - f[2] * tanf(tilt)) / g->drag - state->drag_offset[0];
        measured[1] = -GRAVITY * f[1] / g->drag - state->drag_offset[1];
        float tau = fmaxf(g->drag_tau, dt), pull = 1 - expf(-dt / tau);
        float x = measured[0] * cy - measured[1] * sy, y = measured[0] * sy + measured[1] * cy;
        float error[2] = {x - state->velocity[0], y - state->velocity[1]};
        for (int i = 0; i < 2; i++) {
            float correction = error[i] * pull;
            state->velocity[i] += correction;
            for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3 + i] += correction;
            state->place[i] += state->velocity[i] * dt;
        }
        if (!seeing) {
            float learn = dt / (4 * tau * tau);
            state->drag_bias[0] = clamp(state->drag_bias[0] - (error[0] * cy + error[1] * sy) * learn, -MAX_BIAS, MAX_BIAS);
            state->drag_bias[1] = clamp(state->drag_bias[1] - (-error[0] * sy + error[1] * cy) * learn, -MAX_BIAS, MAX_BIAS);
        }
    }

    // IMU: integrate vertical acceleration; without the camera, leak the climb rate.
    state->climb_rate += (vertical_g - state->bias) * GRAVITY * dt;
    if (pose && (!state->have_pose || pose->sequence != state->sequence)) {
        if (!seeing) state->have_pose = state->have_setpoint = false;
        take(state, g, pose);
        state->seen_us = now_us;
        seeing = state->found = true;
        // The camera corrects the IMU: the tag is position[2] above the camera.
        float pull = 1 - expf(-dt / CAMERA_TAU);
        state->height += (state->tag_height - state->position[2] - state->height) * pull;
        state->climb_rate += (state->velocity[2] - state->climb_rate) * pull;
    } else if (!seeing) {
        float leak = mode == HOVER_HOLD && state->height >= AIRBORNE ? state->command : 0;
        state->climb_rate += (leak - state->climb_rate) * (1 - expf(-dt / LEAK_TAU));
        if (!dragging) {
            float keep = expf(-dt / DRIFT_TAU);
            state->velocity[0] *= keep;
            state->velocity[1] *= keep;
        }
    }
    if (seeing && dragging && state->height >= AIRBORNE) {
        // The camera measures the velocity: whatever the drag model reads beyond it, in body axes, is its offset (servo zero, accelerometer bias, mounting).
        float forward = state->velocity[0] * cy + state->velocity[1] * sy, left = -state->velocity[0] * sy + state->velocity[1] * cy;
        state->drag_offset[0] = clamp(state->drag_offset[0] + (measured[0] - forward) * dt / OFFSET_TAU, -MAX_OFFSET, MAX_OFFSET);
        state->drag_offset[1] = clamp(state->drag_offset[1] + (measured[1] - left) * dt / OFFSET_TAU, -MAX_OFFSET, MAX_OFFSET);
        state->place[0] = state->place[1] = 0;
    }
    state->height += state->climb_rate * dt;

    // Climb-rate command: hold moves the target height toward its goal at climb_rate; landing descends at a set rate.
    float command;
    if (mode == HOVER_HOLD) {
        // The goal is climb; centring goes to the tag's height instead, climbs toward the ceiling to bring the tag into view until it has been flown on (a glimpse from the ground does not count), and holds the height reached when it is lost.
        float goal = g->climb;
        if (g->center) goal = seeing ? clamp(state->tag_height, MIN_HEIGHT, g->ceiling) : state->tracking ? fmaxf(g->climb, state->target) : g->ceiling;
        state->target += clamp(goal - state->target, -g->climb_rate * dt, g->climb_rate * dt);
        command = clamp(g->kz * (state->target - state->height), -MAX_CLIMB, fminf(MAX_CLIMB, g->climb_rate));
    } else {
        command = -g->land_rate * (mode == HOVER_LAND_FAST ? 3 : 1);
        state->target = state->height;
    }
    state->command = command;
    float error = command - state->climb_rate;
    state->integral = clamp(state->integral + g->kiz * error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    float up = clamp(g->kdz * error + state->integral, -MAX_VERTICAL, MAX_VERTICAL);
    state->force = g->mass * (GRAVITY + up) / fmaxf(0.5f, cosf(roll) * cosf(pitch));

    // On the ground a descent command winds thrust down, since the climb rate cannot follow.
    if (mode != HOVER_HOLD) {
        bool held = state->force < TOUCHDOWN_FRACTION * weight && fabsf(state->climb_rate) < TOUCHDOWN_RATE;
        state->low_time = held ? state->low_time + dt : 0;
        if (state->low_time >= TOUCHDOWN_S) state->landed = true;
    }

    *output = (HoverOutput){.force = state->force};
    float forward, left;
    if (!seeing) {
        // Tag lost after flying on it: keep damping the dead-reckoned velocity, with the trim learned on the tag. Before the tag is first flown on, the attitude controller's own damping holds the drone, unless the drag model keeps the velocity: then the hold flies on it from takeoff, and also returns to the place where the tag was last seen (or where it took off), as far as the dead reckoning knows it.
        if ((!state->tracking && !dragging) || state->height < AIRBORNE) return;
        float ax = -g->kd * state->velocity[0], ay = -g->kd * state->velocity[1];
        if (dragging) {
            ax -= g->kp * state->place[0];
            ay -= g->kp * state->place[1];
        }
        forward = state->trim[0] + ax * cy + ay * sy;
        left = state->trim[1] - ax * sy + ay * cy;
    } else {
        // The tag relative to the drone now: the sighting moved on by the drone's velocity over its age, then turned into the body's heading.
        float lead = clamp((now_us - state->capture_us) / 1e6f, 0, 0.1f);
        float px = state->position[0] - state->velocity[0] * lead, py = state->position[1] - state->velocity[1] * lead;
        float range = fmaxf(hypotf(px, py), 0.1f);
        float bearing = atan2f(px * sy - py * cy, px * cy + py * sy);
        // Where on the circle about the tag: the heading of the line from the drone to the tag, in the yaw-turned frame.
        float around = atan2f(py, px);
        if (!state->have_setpoint) {
            state->setpoint = range;
            state->bearing = g->center ? 0 : bearing;
            state->arc = around;
            state->have_setpoint = true;
        }
        // Centring: on the ground the targets follow the drone and nothing is commanded, since noisy camera velocity would only tilt it on its feet; airborne in hold the range target moves toward distance at the approach speed, so a large offset is flown out rather than lunged at, and the loop follows its rate; the place on the circle moves the same way toward square in front of the tag once the wall's normal is known; landing holds them where they are.
        float rate = 0, arc_rate = 0;
        if (g->center && mode == HOVER_HOLD) {
            if (state->height < AIRBORNE) {
                state->setpoint = range;
                state->arc = around;
                return;
            }
            rate = clamp(g->distance - state->setpoint, -g->approach * dt, g->approach * dt) / dt;
            state->setpoint += rate * dt;
            if (state->have_normal) {
                // Smoothly rather than at full speed, as the estimate moves a little with each sighting.
                float fastest = g->approach / range;
                arc_rate = clamp(wrap(state->normal - state->arc) / ARC_TAU, -fastest, fastest);
                state->arc = wrap(state->arc + arc_rate * dt);
            }
        }
        // Toward the tag: close the range error and damp the closing speed toward the target's rate. Across it (positive left): damp the sideways speed toward the target's rate, and pull weakly toward the place on the circle, so the drone neither drifts around the tag nor fights the range and heading loops for it. Moving left turns the line to the tag right (smaller around).
        float ux = px / range, uy = py / range;
        float closing = state->velocity[0] * ux + state->velocity[1] * uy;
        float sideways = -state->velocity[0] * uy + state->velocity[1] * ux;
        float hold = g->kp * (range - state->setpoint), place = g->kside * range * wrap(around - state->arc);
        float toward = hold - g->kd * (closing + rate), across = place - g->kd * (sideways + arc_rate * range);
        float ax = toward * ux - across * uy;
        float ay = toward * uy + across * ux;
        forward = ax * cy + ay * sy;
        left = -ax * sy + ay * cy;
        // A steady push fixed to the body (level-reference error, thrust-vector trim) would leave steady range and place errors: their position terms, in body axes, slowly build a trim that takes it over.
        float hx = hold * ux - place * uy, hy = hold * uy + place * ux;
        state->trim[0] = clamp(state->trim[0] + (hx * cy + hy * sy) * dt / TRIM_TAU, -MAX_TRIM, MAX_TRIM);
        state->trim[1] = clamp(state->trim[1] + (-hx * sy + hy * cy) * dt / TRIM_TAU, -MAX_TRIM, MAX_TRIM);
        forward += state->trim[0];
        left += state->trim[1];
        // Tag right of the bearing target: turn right, a negative yaw rate.
        output->yaw_rate = clamp(-g->kyaw * wrap(bearing - state->bearing), -g->yaw_rate, g->yaw_rate);
        state->tracking = true;
    }
    output->tracking = true;
    output->velocity[0] = state->velocity[0] * sy - state->velocity[1] * cy;
    output->velocity[1] = state->velocity[0] * cy + state->velocity[1] * sy;
    output->pitch = clamp(atanf(forward / GRAVITY), -g->tilt, g->tilt);
    output->roll = clamp(atanf(-left / GRAVITY), -g->tilt, g->tilt);
}
