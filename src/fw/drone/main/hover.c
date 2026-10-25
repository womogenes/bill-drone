#include <math.h>
#include <string.h>

#include "hover.h"

#define GRAVITY 9.81f
// A tag unseen this long is lost: the horizontal and heading holds stop and height runs on the IMU alone.
#define LOST_US 250000
// Camera velocity smoothing (s); poses further apart than GAP_US restart it.
#define VELOCITY_TAU 0.08f
#define GAP_US 200000
// How fast the camera pulls the IMU's height and climb rate toward its own (s).
#define CAMERA_TAU 0.15f
// Without the camera the integrated climb rate leaks toward zero over this long (s), so an accelerometer bias cannot run away.
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

static float clamp(float value, float low, float high) {
    return fmaxf(low, fminf(high, value));
}

static float wrap(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

float hover_heading(const float *forward) {
    return atan2f(forward[0], forward[2]);
}

void hover_rest(HoverState *state, float vertical_g, float dt) {
    float bias = state->bias ? state->bias + (vertical_g - state->bias) * (1 - expf(-dt / BIAS_TAU)) : vertical_g;
    memset(state, 0, sizeof(*state));
    state->bias = bias;
}

static void take(HoverState *state, const HoverPose *pose) {
    float interval = (pose->capture_us - state->capture_us) / 1e6f;
    if (state->have_pose && interval > 0 && pose->capture_us - state->capture_us <= GAP_US) {
        float keep = expf(-interval / VELOCITY_TAU);
        for (int i = 0; i < 3; i++) {
            float raw = (pose->position[i] - state->position[i]) / interval;
            state->velocity[i] = keep * state->velocity[i] + (1 - keep) * raw;
        }
    } else {
        memset(state->velocity, 0, sizeof(state->velocity));
        // First sight (or after a gap): the camera's current height is the IMU's current height.
        state->zero_y = pose->position[1] + state->height;
    }
    memcpy(state->position, pose->position, sizeof(state->position));
    state->facing = hover_heading(pose->forward);
    state->sequence = pose->sequence;
    state->capture_us = pose->capture_us;
    state->have_pose = true;
}

void hover_update(HoverState *state, const HoverGains *g, int mode, const HoverPose *pose, int64_t now_us, float dt,
                  float roll, float pitch, float vertical_g, float force, HoverOutput *output) {
    float weight = g->mass * GRAVITY;
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
        *output = (HoverOutput){0, 0, 0, state->force};
        return;
    }

    // IMU: integrate vertical acceleration; without the camera, leak the climb rate.
    state->climb_rate += (vertical_g - state->bias) * GRAVITY * dt;
    bool seeing = state->have_pose && now_us - state->seen_us <= LOST_US;
    if (pose && (!state->have_pose || pose->sequence != state->sequence)) {
        if (!seeing) state->have_pose = state->have_setpoint = false;
        take(state, pose);
        state->seen_us = now_us;
        seeing = true;
        if (!state->have_setpoint) {
            state->setpoint[0] = pose->position[0];
            state->setpoint[1] = pose->position[2];
            state->heading = state->facing;
            state->have_setpoint = true;
        }
        // The camera corrects the IMU: tag y points down, so height is zero_y - y.
        float pull = 1 - expf(-dt / CAMERA_TAU);
        state->height += (state->zero_y - pose->position[1] - state->height) * pull;
        state->climb_rate += (-state->velocity[1] - state->climb_rate) * pull;
    } else if (!seeing) {
        state->climb_rate *= expf(-dt / LEAK_TAU);
    }
    state->height += state->climb_rate * dt;

    // Climb-rate command: hold climbs to the target height; landing descends at a set rate.
    float command;
    if (mode == HOVER_HOLD) {
        state->target = fminf(g->climb, state->target + g->climb_rate * dt);
        command = clamp(g->kz * (state->target - state->height), -MAX_CLIMB, fminf(MAX_CLIMB, g->climb_rate));
    } else {
        command = -g->land_rate * (mode == HOVER_LAND_FAST ? 3 : 1);
        state->target = state->height;
    }
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

    *output = (HoverOutput){0, 0, 0, state->force};
    if (!seeing) return;
    // Horizontal hold in the tag frame (x along the wall, z toward it), turned into body forward/right by the camera's heading; the camera position is extrapolated over its age.
    float lead = clamp((now_us - state->capture_us) / 1e6f, 0, 0.1f);
    float x = state->position[0] + state->velocity[0] * lead, z = state->position[2] + state->velocity[2] * lead;
    float ax = -g->kp * (x - state->setpoint[0]) - g->kd * state->velocity[0];
    float az = -g->kp * (z - state->setpoint[1]) - g->kd * state->velocity[2];
    float forward = ax * sinf(state->facing) + az * cosf(state->facing);
    float right = ax * cosf(state->facing) - az * sinf(state->facing);
    output->pitch = clamp(atanf(forward / GRAVITY), -g->tilt, g->tilt);
    output->roll = clamp(atanf(right / GRAVITY), -g->tilt, g->tilt);
    // Turned right of the engage heading: turn left, a positive yaw rate.
    output->yaw_rate = clamp(g->kyaw * wrap(state->facing - state->heading), -g->yaw_rate, g->yaw_rate);
}
