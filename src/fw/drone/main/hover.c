#include <math.h>
#include <string.h>

#include "hover.h"

#define GRAVITY 9.81f
// A tag unseen this long is lost: the range, place and heading holds stop, horizontal velocity is dead-reckoned and damped, and height runs on the IMU alone.
#define LOST_US 250000
// How fast the camera corrects the velocity (s); sightings further apart than GAP_US restart it.
#define VELOCITY_TAU 0.4f
#define GAP_US 200000
// The push observer: each camera velocity correction, spread over PUSH_TAU (s), moves the push estimate (critically damped with VELOCITY_TAU at 4 VELOCITY_TAU), up to MAX_PUSH (m/s²).
#define PUSH_TAU 1.6f
#define MAX_PUSH 3.0f
// Without the tag the dead-reckoned velocity leaks toward zero over this long (s), as the trim never quite matches the push it cancels.
#define DRIFT_TAU 3.0f
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
// Landing. Touchdown goes by the accelerometer, not by the height or climb-rate estimate (the tag is about 2 m away, so a few degrees of attitude error moves the camera's height 3.5 cm per degree, and near the floor the estimates read 10-30 cm off either way: 2026-10-29 07:20 "descended" to -0.31 m on the floor at hover thrust, 08:15 fired on a height rule in the air at -0.12 m), nor by hover_mass (06:07 held at 0.88 of hover_mass weight and fired at 0.47 m as the descent began). Its references are the thrust the hold actually flew and the vertical specific force it read meanwhile, both smoothed over HOLD_TAU (s).
#ifndef HOLD_TAU
#define HOLD_TAU 1.0f
#endif
// A probe winds thrust down from where it is at PROBE_RATE (fraction of the hold's thrust per s), whatever the climb-rate loop asks, and watches the accelerometer: in the air the vertical specific force follows the thrust down (after the motors' dead time and lag, taken as a lag of PROBE_LAG (s), a little long, so the air shows first), on the floor the ground takes the weight and it stays put. It is in the air as soon as the specific force over the last RECENT_TAU (s) is PROBE_FALL (g) under the hold's, or the vertical speed gained since the probe began reaches PROBE_DEPTH (m/s) down; it is on the floor once the thrust lost would have cost the air PROBE_DEPTH of vertical speed while the accelerometer saw under PROBE_SHARE of that: touchdown. The two references guard each other: the speed gained is counted from the specific force at the probe's start, which a descent still speeding up reads low (in the simulator that alone called touchdown at 0.13 m), and the hold's specific force can be a percent or two of g off the floor's, well inside PROBE_FALL.
#ifndef RECENT_TAU
#define RECENT_TAU 0.1f
#endif
#ifndef PROBE_RATE
#define PROBE_RATE 1.0f
#endif
#ifndef PROBE_LAG
#define PROBE_LAG 0.15f
#endif
#ifndef PROBE_FALL
#define PROBE_FALL 0.08f
#endif
#ifndef PROBE_DEPTH
#define PROBE_DEPTH 0.2f
#endif
#ifndef PROBE_SHARE
#define PROBE_SHARE 0.5f
#endif
// A probe that finds air gives back the thrust it started from and waits PROBE_WAIT (s) before the next. The sink it caused (up to MAX_SINK, m/s) fades from the climb-rate command over SINK_TAU (s) instead of being caught at once, which with a climb-rate estimate that already reads low near the floor carried the simulated drone back up.
#ifndef PROBE_WAIT
#define PROBE_WAIT 1.0f
#endif
#ifndef SINK_TAU
#define SINK_TAU 1.0f
#endif
#ifndef MAX_SINK
#define MAX_SINK 0.1f
#endif
// A probe starts once the descent command has eased in and the descent has stalled (climb rate under half the descent rate for STALL_S), or the accelerometer shows the descent stopped by something (the speed gained over the last IMPULSE_TAU (s) jumps up by IMPULSE of the descent rate, and by at least IMPULSE_MIN (m/s): slower contacts are not told from the hold's own corrections), or the height estimate is below LOW_HEIGHT (m), or at the latest once the landing has run as long as the descent from its starting height should take, plus LAND_MARGIN of that and LAND_SLACK (s). All of them can be wrong in the air, which a probe only costs a short sink.
#ifndef STALL_S
#define STALL_S 0.2f
#endif
#ifndef IMPULSE_TAU
#define IMPULSE_TAU 0.1f
#endif
#ifndef IMPULSE
#define IMPULSE 0.4f
#endif
#ifndef IMPULSE_MIN
#define IMPULSE_MIN 0.08f
#endif
#ifndef LOW_HEIGHT
#define LOW_HEIGHT 0.0f
#endif
#ifndef LAND_MARGIN
#define LAND_MARGIN 0.1f
#endif
#ifndef LAND_SLACK
#define LAND_SLACK 0.3f
#endif
// A landing integrates its climb rate from the specific force the hold read rather than the resting one (motors off, they differ by a percent or two of g, and 0.01 g is 0.1 m/s of climb rate per second wherever the camera does not hold it).
#ifndef LAND_BIAS
#define LAND_BIAS 1
#endif
// After touchdown thrust falls from the hold's to zero over LANDED_RAMP (s) with the attitude targets at the attitude the floor gives (LANDED_ATTITUDE 1) rather than level, and a shock or tilt past the contact limits below still cuts it at once (06:08 pitched over to 42 degrees nose-down as thrust faded over 0.5 s with the servos at their 60 degree limits).
#ifndef LANDED_ATTITUDE
#define LANDED_ATTITUDE 1
#endif
#ifndef LANDED_RAMP
#define LANDED_RAMP 0.25f
#endif
// Contact: a landing that hits harder than LAND_SHOCK (specific force along vertical, g), or tilts past LAND_TILT from level, stops the motors at once. With no legs the drone touches down on its body, and thrust above the first contact point levers it over (2026-10-28 05:01: 12 degrees nose-down at contact, 1.6 g and over its nose 0.15 s later). Hovering stays under 1.35 g; a tilt alone is no proof of contact (05:05 pitched to 15 degrees in the air at 0.7 m, and a cut there drops the drone), so LAND_TILT is only for a drone already going over.
// The tilt cut applies only near the floor: under LAND_TILT_HEIGHT (m) by the height estimate, which reads 10-30 cm off there, or once a probe is feeling for it. On 2026-11-02 04:16 UTC a landing pressed at 0.9 m, mid-way through a 24-degree pitch swing, cut both motors 0.1 s later.
#ifndef LAND_TILT_HEIGHT
#define LAND_TILT_HEIGHT 0.3f
#endif
#ifndef LAND_TILT
#define LAND_TILT (25.0f * 3.14159265f / 180)
#endif
#ifndef LAND_SHOCK
#define LAND_SHOCK 1.45f
#endif
// The descent command ramps in over LAND_EASE (s), so pressing land does not step collective down.
#define LAND_EASE 0.5f
// Above this nudge speed (m/s) the hold point counts as moving.
#ifndef NUDGE_MOVING
#define NUDGE_MOVING 0.02f
#endif
// The nudge's speed is smoothed over this long (s): the laptop moves it in small steps every 50 ms.
#ifndef NUDGE_TAU
#define NUDGE_TAU 0.2f
#endif
// A nudge that moves further than this in one tick (m) was wrapped by the laptop, not flown: without the tag only its speed matters, so the dashboard wraps the forward and left nudge around its limit to keep a held arrow flying, and the jump counts as no motion.
#define NUDGE_JUMP 0.5f
// Below this height (m) a hold is still on the ground taking off: the IMU climb rate leaks toward zero, and centring neither tilts nor turns and lets the range target follow the drone. The tag-height target never asks for less than MIN_HEIGHT above the engage point (m).
#define AIRBORNE 0.05f
#define MIN_HEIGHT 0.2f
// With the drag model the velocity counts from when the collective carries this share of the weight, before the height does: the IMU height crosses AIRBORNE about half a second after the feet leave the floor, and on 2026-10-31 a takeoff from the drone's 5° nose-up rest was moving back at 0.8 m/s by then, unseen and so never braked. Below it the feet hold the drone and the velocity is zero.
#define LIFTING 0.8f
// Takeoff from wherever the drone rests: it has no legs, and its body sits several degrees off level (3-8 degrees nose-down on 2026-11-07). While the floor holds it the attitude targets are the resting attitude, so the servos do not tilt the thrust against the floor, and from liftoff they ease to the hold's own over TAKEOFF_EASE_S (s). Levelling at once from an 8 degree nose-down rest kicked the pitch at 45 degrees/s at liftoff (2026-11-08 00:39 UTC), and the hold, blind before the tag, answered it with a 10 degree lean back. Liftoff is the height estimate passing LIFTED_HEIGHT (m), 0.25-0.4 s after the motors start in that day's hops (the collective is at hover thrust from the first tick, so it cannot tell), or TAKEOFF_WAIT_S (s) after engaging, whichever comes first.
#define TAKEOFF_EASE_S 0.6f
#define LIFTED_HEIGHT 0.005f
#define TAKEOFF_WAIT_S 1.5f
// Rotor drag: the servos' dead time beyond the tick the runtime's tilt is already late by (us; sw/sim/sweep.py FITTED has 70 ms), how slowly the camera teaches the drag model its offset (s), and that offset's limit (m/s).
#define SERVO_DELAY_US 60000
#define OFFSET_TAU 5.0f
#define MAX_OFFSET 1.0f
// The drag model's offset learned from the GPS is mostly the wind.
#define MAX_WIND 10.0f
// The integrated acceleration's bias the drag model may find (m/s², about 6 degrees of attitude error).
#define MAX_BIAS 1.0f
// After a stick is let go the hold brakes for up to BRAKE_S (s), or until the speed estimate is under BRAKED (m/s), before it holds a place: the place is then where the drone stopped.
#define BRAKE_S 2.0f
#define BRAKED 0.15f
// Speed from the servos (gains speed_tilt): the tilt and pitch are smoothed over SPEED_SMOOTH (s; 0.5 fitted best against the camera's speed, against 0.1-1), the offset is first the mean over the first SPEED_TAKEOFF_S airborne (plus speed_takeoff) and then follows the camera's speed over SPEED_LEARN (s) while it sees the tag.
#define SPEED_SMOOTH 0.5f
#define SPEED_TAKEOFF_S 2.0f
#define SPEED_LEARN 5.0f
// After the tag is found again following a loss, the hold's position terms ease in over this long (us): the drift while blind arrives as one jump in the sighting, and at full gain it commanded the tilt limit on both axes at once (2026-11-02 05:13 UTC: 0.5 m after 0.7 s lost, pitch -10 and roll +10, and the overshoot to 17 degrees made the pilot stop it).
#define REACQUIRE_EASE_US 1000000
// GPS (take_gps): a fix older than this no longer stands in for the drag model and the speed from the servos (both read airspeed, which outdoors is not the ground speed the hold needs).
#define GPS_STALE_US 500000
// The heading's evidence fades over HEADING_TAU (s); it moves the heading (over HEADING_PULL, s) once the faded sum of |inertial change| |GPS change| passes HEADING_EVIDENCE (m²/s²) and the changes agree in direction (the evidence's magnitude at least HEADING_COHERENCE of that sum).
#define HEADING_TAU 30.0f
#define HEADING_PULL 3.0f
#define HEADING_EVIDENCE 0.3f
#define HEADING_COHERENCE 0.6f

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
    // Camera (right, down, forward), rolled by cam_roll and tilted up by cam_pitch, to body (forward, left, up).
    float cc = cosf(g->cam_roll), sc = sinf(g->cam_roll);
    float right = cc * in[0] - sc * in[1], down = sc * in[0] + cc * in[1], ahead = in[2];
    float ca = cosf(g->cam_pitch), sa = sinf(g->cam_pitch);
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
            // A velocity the tilt keeps getting wrong the same way is a push it does not know about: learn it (only once flying on the tag, so the takeoff's ground contact does not teach it).
            if (state->tracking) state->push[i] = clamp(state->push[i] + correction / PUSH_TAU, -MAX_PUSH, MAX_PUSH);
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

void hover_body_to_camera(const HoverGains *g, const float *body, float *camera) {
    // level()'s mounting, undone: body to the rolled camera's (right, down, ahead), then the roll taken off.
    float ca = cosf(g->cam_pitch), sa = sinf(g->cam_pitch);
    float right = -body[1], down = body[0] * sa - body[2] * ca, ahead = body[0] * ca + body[2] * sa;
    float cc = cosf(g->cam_roll), sc = sinf(g->cam_roll);
    camera[0] = cc * right + sc * down;
    camera[1] = -sc * right + cc * down;
    camera[2] = ahead;
}

// Flow without the tag: the velocity across the camera (right and down, the scene at flow_depth) corrects the estimate along those directions in the level frame, and the climb rate; the sideways part teaches the drag model's offset, which is how the wind is learned, and a steady disagreement in the climb rate is the vertical acceleration's bias, which it learns (critically damped, as the drag model's bias).
// A flow arrives once a camera frame (about 22 a second), not every tick, so each corrects over the time since the previous one: corrected per tick, flow_tau acted about 4.5 times longer than set, and without the bias the height still drifted (in the sim 7 m RMS in 30 s without flow, 0.2 m with both, 2026-10-31).
static void take_flow(HoverState *state, const HoverGains *g, const HoverFlow *flow, float cy, float sy, float dt, bool dragging) {
    static const float axes[2][3] = {{1, 0, 0}, {0, 1, 0}};
    float interval = state->flow_us && flow->capture_us > state->flow_us ? fminf((flow->capture_us - state->flow_us) / 1e6f, 0.1f) : dt;
    state->flow_us = flow->capture_us;
    float tau = fmaxf(g->flow_tau, interval), pull = 1 - expf(-interval / tau), climb_error = 0;
    for (int a = 0; a < 2; a++) {
        float direction[3];
        level(state, g, flow->capture_us, axes[a], direction);
        // Moving along +axis makes the scene sweep the other way: rate = -velocity / distance.
        float measured = -g->flow_depth * flow->rate[a];
        float estimate = state->velocity[0] * direction[0] + state->velocity[1] * direction[1] + state->climb_rate * direction[2];
        float error = (measured - estimate) * pull;
        climb_error += (measured - estimate) * direction[2];
        for (int i = 0; i < 2; i++) {
            state->velocity[i] += error * direction[i];
            for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3 + i] += error * direction[i];
        }
        state->climb_rate += error * direction[2];
    }
    state->flow_bias = clamp(state->flow_bias - climb_error * interval / (4 * tau * tau), -MAX_BIAS, MAX_BIAS);
    if (dragging) {
        // The sideways speed the flow now gives, against what the drag model reads (as with the tag below; forward stays the drag model's own until the camera measures expansion).
        float left = -state->velocity[0] * sy + state->velocity[1] * cy;
        state->drag_offset[1] = clamp(state->drag_offset[1] + (state->drag_velocity[1] - left) * dt / OFFSET_TAU, -MAX_OFFSET, MAX_OFFSET);
    }
}

// GPS without the tag: its velocity (turned into the hold's frame by the heading) corrects the estimate as it was at the fix's capture, and the past since, over gps_tau; a steady disagreement is the measured acceleration's bias, which it learns (critically damped, as the drag model's). Its velocity changes against the inertial ones give the heading. Its position moves the dead reckoning and the hold's place with it over gps_place_tau, and its climb rate (and height, with gps_height_tau) the vertical as the flow does.
static void take_gps(HoverState *state, const HoverGains *g, const HoverGps *fix, int64_t now_us, float cy, float sy, float dt) {
    bool first = !state->gps_us || fix->capture_us <= state->gps_us;
    float span = first ? dt : fminf((fix->capture_us - state->gps_us) / 1e6f, 0.5f);
    const float *then = state->history[recorded(state, fix->capture_us)];
    if (!first) {
        // Inertial change times the GPS change's conjugate, the GPS's as north - i east: their product's angle is the heading.
        float ax = then[6] - state->gps_inertial[0], ay = then[7] - state->gps_inertial[1];
        float cn = fix->velocity[0] - state->gps_velocity[0], ce = -(fix->velocity[1] - state->gps_velocity[1]);
        float keep = expf(-span / HEADING_TAU);
        state->turn[0] = state->turn[0] * keep + ax * cn + ay * ce;
        state->turn[1] = state->turn[1] * keep + ay * cn - ax * ce;
        state->turn_weight = state->turn_weight * keep + hypotf(ax, ay) * hypotf(cn, ce);
        if (state->turn_weight > HEADING_EVIDENCE && hypotf(state->turn[0], state->turn[1]) > HEADING_COHERENCE * state->turn_weight)
            state->heading = wrap(state->heading + wrap(atan2f(state->turn[1], state->turn[0]) - state->heading) * (1 - expf(-span / HEADING_PULL)));
    }
    state->gps_velocity[0] = fix->velocity[0];
    state->gps_velocity[1] = fix->velocity[1];
    state->gps_inertial[0] = then[6];
    state->gps_inertial[1] = then[7];
    state->gps_us = fix->capture_us;
    // North and east into the hold's frame: x + i y = (north - i east) e^(i heading).
    float ch = cosf(state->heading), sh = sinf(state->heading);
    float tau = fmaxf(g->gps_tau, span), pull = 1 - expf(-span / tau);
    float error[2] = {fix->velocity[0] * ch + fix->velocity[1] * sh - then[3], fix->velocity[0] * sh - fix->velocity[1] * ch - then[4]};
    for (int i = 0; i < 2; i++) {
        state->velocity[i] += error[i] * pull;
        for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3 + i] += error[i] * pull;
    }
    // The bias is in body axes, where the drag model keeps it (a steady level-reference or mounting error turns with the drone).
    float learn = span / (4 * tau * tau);
    state->drag_bias[0] = clamp(state->drag_bias[0] - (error[0] * cy + error[1] * sy) * learn, -MAX_BIAS, MAX_BIAS);
    state->drag_bias[1] = clamp(state->drag_bias[1] - (-error[0] * sy + error[1] * cy) * learn, -MAX_BIAS, MAX_BIAS);
    float climb_error = fix->velocity[2] - state->climb_rate;
    state->climb_rate += climb_error * pull;
    state->flow_bias = clamp(state->flow_bias - climb_error * learn, -MAX_BIAS, MAX_BIAS);
    // Position: the first fix ties the GPS to the dead reckoning; later ones correct it (back-dated by the fix's age) and move the place, which is relative to the hold point, by the same amount.
    float age = clamp((now_us - fix->capture_us) / 1e6f, 0, 0.5f);
    if (first) {
        state->origin[0] = fix->position[0] - (state->track[0] * ch + state->track[1] * sh);
        state->origin[1] = fix->position[1] - (state->track[0] * sh - state->track[1] * ch);
        state->origin[2] = fix->position[2] - state->height;
    } else {
        float n = fix->position[0] - state->origin[0], e = fix->position[1] - state->origin[1];
        if (g->gps_place_tau > 0) {
            float place_pull = 1 - expf(-span / fmaxf(g->gps_place_tau, span));
            float shift[2] = {(n * ch + e * sh - (state->track[0] - state->velocity[0] * age)) * place_pull, (n * sh - e * ch - (state->track[1] - state->velocity[1] * age)) * place_pull};
            for (int i = 0; i < 2; i++) {
                state->track[i] += shift[i];
                state->place[i] += shift[i];
            }
        }
        if (g->gps_height_tau > 0) state->height += (fix->position[2] - state->origin[2] - state->climb_rate * age - state->height) * (1 - expf(-span / fmaxf(g->gps_height_tau, span)));
    }
}

static void hold(HoverState *state, const HoverGains *g, int mode, const HoverPose *pose, int64_t now_us, float dt,
                 float roll, float pitch, float yaw_rate, float vertical_g, float force, const HoverDrag *drag, const HoverFlow *flow,
                 const HoverGps *gps, HoverOutput *output) {
    float weight = g->mass * GRAVITY;
    bool dragging = g->drag > 0 && drag;
    if (!state->engaged) {
        state->engaged = true;
        bool grounded = force < 0.5f * weight;
        state->rest[0] = grounded ? roll : 0;
        state->rest[1] = grounded ? pitch : 0;
        state->lifted = grounded ? -TAKEOFF_WAIT_S : 0;
        state->flow_bias = 0;
        state->flow_us = 0;
        state->gps_us = 0;
        state->heading = g->gps_heading;
        state->turn[0] = state->turn[1] = state->turn_weight = 0;
        state->inertial[0] = state->inertial[1] = state->track[0] = state->track[1] = 0;
        // Continue from the thrust being flown, so taking over a manual flight does not step.
        state->integral = force > 0.5f * weight ? clamp(force / g->mass - GRAVITY, -MAX_INTEGRAL, MAX_INTEGRAL) : 0;
        state->force = force;
        // The carried-over trim is the push the last hold cancelled: start the push estimate from it (yaw is zero at engage, so body and yaw-turned axes agree), and the body trim from zero.
        state->push[0] = clamp(-g->trim_start_x, -MAX_PUSH, MAX_PUSH);
        state->push[1] = clamp(-g->trim_start_y, -MAX_PUSH, MAX_PUSH);
        // The touchdown reference until a hold has flown long enough to give its own: the thrust being flown (or the weight from mass) at the resting specific force.
        state->hold_force = force > 0.5f * weight ? force : weight;
        state->hold_g = state->recent_g = state->bias > 0.5f ? state->bias : 1;
        // Asked to land without enough thrust to be flying: it is already down.
        if (mode != HOVER_HOLD && force < 0.5f * weight) state->landed = true;
    }
    if (state->landed) {
        state->dragging = false;
        if (fabsf(roll) > LAND_TILT || fabsf(pitch) > LAND_TILT || vertical_g > LAND_SHOCK) state->force = 0;
        state->force = fmaxf(0, state->force - state->hold_force / LANDED_RAMP * dt);
        *output = (HoverOutput){.roll = LANDED_ATTITUDE ? roll : 0, .pitch = LANDED_ATTITUDE ? pitch : 0, .force = state->force};
        return;
    }
    // Heading from the gyro; horizontal velocity from tilt (g tan(tilt) less the push the trim cancels, once airborne, as on the ground a resting tilt pushes nothing); both recorded with the attitude.
    state->yaw = wrap(state->yaw + yaw_rate * dt);
    float cy = cosf(state->yaw), sy = sinf(state->yaw);
    // Taking off with the drag model: not yet AIRBORNE by the height, but the collective (last tick's) already lifting.
    bool lifting = dragging && state->height < AIRBORNE && state->force >= LIFTING * weight;
    // With the drag model, the measured specific force turned level instead, less the acceleration bias the drag model has found in it: it carries the real thrust tilt and the drag, and its level-reference error cancels against the attitude's (both are relative to pose_zero); what remains is mostly the attitude estimate's own drift.
    if (state->height >= AIRBORNE || lifting) {
        float ahead, left;
        if (dragging) {
            const float *f = drag->specific;
            float cr = cosf(roll), sr = sinf(roll), cp = cosf(pitch), sp = sinf(pitch);
            ahead = GRAVITY * (cp * f[0] + sp * (sr * f[1] + cr * f[2])) - state->drag_bias[0];
            left = GRAVITY * (cr * f[1] - sr * f[2]) - state->drag_bias[1];
        } else {
            ahead = GRAVITY * tanf(pitch - g->pitch_trim + g->thrust_tilt * state->thrust_tilt);
            left = -GRAVITY * tanf(roll);
        }
        // Plus the push learned from the camera (the trim is already in the attitude flown, so it is not subtracted again).
        float push[2] = {dragging ? 0 : state->push[0], dragging ? 0 : state->push[1]};
        state->velocity[0] += (ahead * cy - left * sy + push[0]) * dt;
        state->velocity[1] += (ahead * sy + left * cy + push[1]) * dt;
        if (dragging) {
            state->inertial[0] += (ahead * cy - left * sy) * dt;
            state->inertial[1] += (ahead * sy + left * cy) * dt;
        }
        // Airborne, the drag model's correction below moves the place.
        if (lifting) {
            state->place[0] += state->velocity[0] * dt;
            state->place[1] += state->velocity[1] * dt;
        }
    } else if (dragging) {
        state->velocity[0] = state->velocity[1] = 0;
        state->place[0] = state->place[1] = 0;
    }
    float *now = state->history[state->next];
    state->history_us[state->next] = now_us;
    now[0] = roll;
    now[1] = pitch;
    now[2] = state->yaw;
    now[3] = state->velocity[0];
    now[4] = state->velocity[1];
    now[5] = drag ? drag->tilt : 0;
    now[6] = state->inertial[0];
    now[7] = state->inertial[1];
    state->next = (state->next + 1) % HOVER_HISTORY;
    // The thrust tilt the servos have reached, beyond pitch_trim's (which flies the body at pitch_trim so a level thrust needs a tilt of -pitch_trim), learned only in the air.
    if (g->thrust_tilt > 0 && drag && state->height >= AIRBORNE) {
        float reached = state->history[recorded(state, now_us - SERVO_DELAY_US)][5] + g->pitch_trim;
        state->thrust_tilt += (reached - state->thrust_tilt) * (1 - expf(-dt / fmaxf(g->thrust_tilt_tau, dt)));
    }

    // Rotor drag: the body's horizontal specific force less the thrust vector's forward part (at the tilt the servos have reached) is -drag times the airspeed. It is noisy (vibration, the pitch loop's servo motion), so it pulls the estimate slowly, and the past with it, as the camera's correction does; a steady disagreement is the integrated acceleration's bias, which it learns (a critically damped second-order filter, settling over a few drag_tau). The camera, when it sees the tag, measures the velocity itself: then the disagreement is the drag model's own offset (servo zero, accelerometer bias, mounting) instead, which it teaches below.
    bool seeing = state->have_pose && now_us - state->seen_us <= LOST_US;
    // A fresh GPS fix measures the ground speed: it then stands in for the drag model's and the servos' airspeed readings.
    bool located = g->gps_tau > 0 && state->gps_us && now_us - state->gps_us <= GPS_STALE_US && !seeing;
    float measured[2] = {0, 0};
    state->dragging = dragging && state->height >= AIRBORNE;
    if (state->dragging) {
        const float *f = drag->specific;
        float tilt = state->history[recorded(state, now_us - SERVO_DELAY_US)][5] + g->drag_tilt;
        measured[0] = -GRAVITY * (f[0] - f[2] * tanf(tilt)) / g->drag - state->drag_offset[0];
        measured[1] = -GRAVITY * f[1] / g->drag - state->drag_offset[1];
        memcpy(state->drag_velocity, measured, sizeof(measured));
        float tau = fmaxf(g->drag_tau, dt), pull = 1 - expf(-dt / tau);
        float x = measured[0] * cy - measured[1] * sy, y = measured[0] * sy + measured[1] * cy;
        float error[2] = {x - state->velocity[0], y - state->velocity[1]};
        if (located) pull = error[0] = error[1] = 0;
        for (int i = 0; i < 2; i++) {
            float correction = error[i] * pull;
            state->velocity[i] += correction;
            for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3 + i] += correction;
            state->place[i] += state->velocity[i] * dt;
        }
        if (!seeing && !located) {
            float learn = dt / (4 * tau * tau);
            state->drag_bias[0] = clamp(state->drag_bias[0] - (error[0] * cy + error[1] * sy) * learn, -MAX_BIAS, MAX_BIAS);
            state->drag_bias[1] = clamp(state->drag_bias[1] - (-error[0] * sy + error[1] * cy) * learn, -MAX_BIAS, MAX_BIAS);
        }
    }

    // IMU: integrate vertical acceleration (LAND_BIAS); without the camera, leak the climb rate.
    state->climb_rate += ((vertical_g - (LAND_BIAS && mode != HOVER_HOLD ? state->hold_g : state->bias)) * GRAVITY - state->flow_bias) * dt;
    if (pose && (!state->have_pose || pose->sequence != state->sequence)) {
        // Reacquiring the tag in the air keeps the targets: they are relative to the tag, so a drift while it was lost is flown back rather than adopted. On the ground they restart from the new sighting.
        if (!seeing) {
            state->have_pose = false;
            if (state->height < AIRBORNE) state->have_setpoint = false;
            if (state->tracking) state->reacquired_us = now_us;
        }
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
    // From engaging, not only once airborne: without the tag the height is the IMU's alone and drifts (in the sim it sank 2 m in 10 s, so the hold never counted as airborne), and the vertical flow is what keeps the climb rate honest. On the ground the flow reads zero, which is right.
    if (flow && !seeing && g->flow_depth > 0 && state->engaged) take_flow(state, g, flow, cy, sy, dt, state->dragging);
    if (gps && !seeing && g->gps_tau > 0) {
        take_gps(state, g, gps, now_us, cy, sy, dt);
        located = true;
    }
    if (located && state->dragging) {
        // What the drag model reads beyond the GPS's ground speed, in body axes, is the wind (with its own offset): learned as the camera teaches the offset, and allowed a wind's size.
        float forward = state->velocity[0] * cy + state->velocity[1] * sy, left = -state->velocity[0] * sy + state->velocity[1] * cy;
        state->drag_offset[0] = clamp(state->drag_offset[0] + (state->drag_velocity[0] - forward) * dt / OFFSET_TAU, -MAX_WIND, MAX_WIND);
        state->drag_offset[1] = clamp(state->drag_offset[1] + (state->drag_velocity[1] - left) * dt / OFFSET_TAU, -MAX_WIND, MAX_WIND);
    }
    if (seeing && dragging && state->height >= AIRBORNE) {
        // The camera measures the velocity: whatever the drag model reads beyond it, in body axes, is its offset (servo zero, accelerometer bias, mounting).
        float forward = state->velocity[0] * cy + state->velocity[1] * sy, left = -state->velocity[0] * sy + state->velocity[1] * cy;
        state->drag_offset[0] = clamp(state->drag_offset[0] + (measured[0] - forward) * dt / OFFSET_TAU, -MAX_OFFSET, MAX_OFFSET);
        state->drag_offset[1] = clamp(state->drag_offset[1] + (measured[1] - left) * dt / OFFSET_TAU, -MAX_OFFSET, MAX_OFFSET);
        state->place[0] = state->place[1] = 0;
    }
    // Speed from the servos: the thrust tilt the pitch loop holds against the forward speed's pitching moment. The rotor drag the drag model reads is a twentieth of what it assumes on this airframe (0.04/s measured), so without the tag the estimate barely saw the drone moving: on 2026-11-02 02:46 UTC it read under 0.2 m/s while the drone flew back about 3 m in 4.5 s, which this measure showed from liftoff.
    state->speed = NAN;
    if (g->speed_tilt > 0 && drag && (state->height >= AIRBORNE || state->force >= LIFTING * weight)) {
        float smooth = 1 - expf(-dt / SPEED_SMOOTH), raw = drag->tilt - g->speed_pitch * pitch;
        state->speed_raw = state->speed_time > 0 ? state->speed_raw + (raw - state->speed_raw) * smooth : raw;
        float forward = state->velocity[0] * cy + state->velocity[1] * sy;
        if ((seeing && state->tracking) || located) {
            // The camera measures the speed: the rest of the tilt is the offset (CoM, servo zero).
            float offset = state->speed_raw - g->speed_tilt * forward;
            state->speed_offset = state->speed_ready ? state->speed_offset + (offset - state->speed_offset) * dt / SPEED_LEARN : offset;
            state->speed_ready = true;
        } else if (!state->speed_ready && state->speed_time < SPEED_TAKEOFF_S) {
            state->speed_sum += raw;
            if (state->speed_time + dt >= SPEED_TAKEOFF_S) {
                state->speed_offset = state->speed_sum * dt / (state->speed_time + dt) + g->speed_takeoff;
                state->speed_ready = true;
            }
        }
        state->speed_time += dt;
        if (state->speed_ready) {
            state->speed = (state->speed_raw - state->speed_offset) / g->speed_tilt;
            if (!seeing && !located) {
                float correction = (state->speed - forward) * (1 - expf(-dt / fmaxf(g->speed_tau, dt)));
                state->velocity[0] += correction * cy;
                state->velocity[1] += correction * sy;
                for (int k = 0; k < HOVER_HISTORY; k++) {
                    state->history[k][3] += correction * cy;
                    state->history[k][4] += correction * sy;
                }
            }
        }
    }
    state->height += state->climb_rate * dt;
    state->track[0] += state->velocity[0] * dt;
    state->track[1] += state->velocity[1] * dt;
    float smooth = 1 - expf(-dt / NUDGE_TAU), nudge[3] = {g->nudge_forward, g->nudge_left, g->nudge_up};
    for (int i = 0; i < 3; i++) {
        float moved = nudge[i] - state->nudge[i];
        if (fabsf(moved) > NUDGE_JUMP) moved = 0;
        state->nudge_rate[i] += (moved / fmaxf(dt, 1e-4f) - state->nudge_rate[i]) * smooth;
        state->nudge[i] = nudge[i];
    }

    // Climb-rate command: hold moves the target height toward its goal at climb_rate; landing descends at a set rate.
    float command;
    if (mode == HOVER_HOLD) {
        // The goal is climb; centring goes to the tag's height instead, climbs toward the ceiling to bring the tag into view until it has been flown on (a glimpse from the ground does not count), and holds the height reached when it is lost.
        // The height nudge raises the goal once flying (the takeoff climb and the search for the tag keep theirs), and its speed feeds the climb command.
        // Without the tag, W and S move the height goal the same way once the drag model holds the drone (state->dragging: airborne).
        float goal = g->climb, up = state->tracking || state->dragging ? g->nudge_up : 0;
        if (g->center) goal = seeing ? clamp(state->tag_height + up, MIN_HEIGHT, g->ceiling) : state->tracking ? fmaxf(g->climb, state->target) : g->ceiling;
        else goal += up;
        state->target += clamp(goal - state->target, -g->climb_rate * dt, g->climb_rate * dt);
        command = clamp(g->kz * (state->target - state->height) + (state->tracking || state->dragging ? state->nudge_rate[2] : 0), -MAX_CLIMB, fminf(MAX_CLIMB, g->climb_rate));
    } else {
        float rate = -g->land_rate * (mode == HOVER_LAND_FAST ? 3 : 1);
        command = fmaxf(rate, fminf(state->command, 0) + rate / LAND_EASE * dt);
        state->target = state->height;
    }
    state->command = command;
    // Landing: less the sink a probe that found air left (SINK_TAU).
    float error = command + (mode != HOVER_HOLD ? state->sink : 0) - state->climb_rate;
    state->integral = clamp(state->integral + g->kiz * error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    float up = clamp(g->kdz * error + state->integral, -MAX_VERTICAL, MAX_VERTICAL);
    float lean = fmaxf(0.5f, cosf(roll) * cosf(pitch));
    state->force = g->mass * (GRAVITY + up) / lean;

    bool floor_near = state->probing || state->height < LAND_TILT_HEIGHT;
    if (mode != HOVER_HOLD && ((floor_near && (fabsf(roll) > LAND_TILT || fabsf(pitch) > LAND_TILT)) || vertical_g > LAND_SHOCK)) {
        state->landed = true;
        state->force = 0;
        *output = (HoverOutput){.force = 0};
        return;
    }
    if (mode == HOVER_HOLD) {
        // The touchdown reference: the thrust the hold flies once off the ground, and the specific force it reads there (its accelerometer bias in flight).
        if (state->height >= AIRBORNE) {
            float pull = 1 - expf(-dt / HOLD_TAU);
            state->hold_force += (state->force - state->hold_force) * pull;
            state->hold_g += (vertical_g - state->hold_g) * pull;
        }
        state->land_time = state->stall_time = state->impulse = state->probe_wait = state->sink = 0;
        state->probing = false;
    } else {
        // The vertical speed the accelerometer gains this tick, against the hold's specific force.
        float rate = g->land_rate * (mode == HOVER_LAND_FAST ? 3 : 1), gained = (vertical_g - state->hold_g) * GRAVITY * dt;
        if (state->land_time == 0) state->land_height = state->height;
        state->land_time += dt;
        // Only once the descent command has eased in: before that the drone is not meant to be moving yet.
        state->stall_time = state->land_time > LAND_EASE && fabsf(state->climb_rate) < 0.5f * rate ? state->stall_time + dt : 0;
        state->impulse = state->impulse * expf(-dt / IMPULSE_TAU) + gained;
        state->probe_wait = fmaxf(0, state->probe_wait - dt);
        state->sink *= expf(-dt / SINK_TAU);
        float due = (LAND_EASE + fmaxf(state->land_height, 0) / fmaxf(rate, 0.01f)) * (1 + LAND_MARGIN) + LAND_SLACK;
        bool stopped = state->land_time > LAND_EASE && state->impulse > fmaxf(IMPULSE * rate, IMPULSE_MIN);
        bool low = state->land_time > LAND_EASE && state->height < LOW_HEIGHT;
        if (!state->probing && state->probe_wait <= 0 && (state->stall_time >= STALL_S || stopped || low || state->land_time >= due)) {
            state->probing = true;
            state->ceiling = state->force;
            state->probe_start = state->probe_force = state->force * lean;
            state->probe_g = state->recent_g;
            state->probe_velocity = state->probe_expected = 0;
        }
        if (state->probing) {
            // In the air the specific force would follow the thrust (lagged by the motors) from where the probe began.
            state->ceiling -= PROBE_RATE * state->hold_force * dt;
            if (state->force > state->ceiling) {
                // The climb-rate loop cannot wind up against the probe.
                state->force = fmaxf(0, state->ceiling);
                state->integral = fminf(state->integral, state->force * lean / g->mass - GRAVITY);
            }
            state->probe_force += (state->force * lean - state->probe_force) * (1 - expf(-dt / PROBE_LAG));
            state->probe_expected += (state->probe_force - state->probe_start) / state->hold_force * GRAVITY * dt;
            state->probe_velocity += (vertical_g - state->probe_g) * GRAVITY * dt;
            bool falling = state->recent_g < state->hold_g - PROBE_FALL || state->probe_velocity <= -PROBE_DEPTH;
            if (!falling && state->probe_expected <= -PROBE_DEPTH && state->probe_velocity > PROBE_SHARE * state->probe_expected) {
                state->landed = true;
            } else if (falling || state->probe_expected <= -PROBE_DEPTH) {
                // In the air.
                state->probing = false;
                state->probe_wait = PROBE_WAIT;
                state->stall_time = state->impulse = 0;
                state->sink = clamp(state->probe_velocity, -MAX_SINK, 0);
                state->force = fmaxf(state->force, state->probe_start / lean);
                state->integral = clamp(fmaxf(state->integral, state->force * lean / g->mass - GRAVITY), -MAX_INTEGRAL, MAX_INTEGRAL);
            }
        }
    }
    state->recent_g += (vertical_g - state->recent_g) * (1 - expf(-dt / RECENT_TAU));

    *output = (HoverOutput){.force = state->force};
    // Until the tag is first seen, hold the heading the hover started with (the gyro's, which drifts only slowly): only the tag steers the heading, so a yaw kick at liftoff turned the camera away from it for good (2026-11-02 04:47 UTC: 58 degrees/s, 30 degrees counterclockwise, the tag never seen and the drone at its ceiling looking for it).
    if (!state->found) output->yaw_rate = clamp(-g->kyaw * state->yaw, -g->yaw_rate, g->yaw_rate);
    float forward, left;
    if (!seeing) {
        // Tag lost after flying on it: keep damping the dead-reckoned velocity, with the trim learned on the tag. Before the tag is first flown on, the attitude controller's own damping holds the drone, unless the drag model keeps the velocity: then the hold flies on it from takeoff, and also returns to the place where the tag was last seen (or where it took off), as far as the dead reckoning knows it.
        if ((!state->tracking && !dragging) || (state->height < AIRBORNE && !lifting)) return;
        // With the drag model the arrows fly the drone: the nudge's speed (toward the drone's heading and to its left) moves the hold point, and the velocity damping aims at that speed, so holding an arrow flies at it and letting go stops where the hold point stopped.
        float vx = 0, vy = 0;
        if (dragging) {
            vx = state->nudge_rate[0] * cy - state->nudge_rate[1] * sy;
            vy = state->nudge_rate[0] * sy + state->nudge_rate[1] * cy;
            state->place[0] -= vx * dt;
            state->place[1] -= vy * dt;
        }
        // A held stick flies its tilt outright on its axis (the other axis still holds); let go, the hold brakes, then holds where the drone stopped.
        bool stick = dragging && (g->stick_pitch || g->stick_roll);
        if (stick)
            state->braking = BRAKE_S;
        else if (state->braking > 0)
            state->braking = hypotf(state->velocity[0], state->velocity[1]) < BRAKED ? 0 : state->braking - dt;
        if (stick || state->braking > 0) state->place[0] = state->place[1] = 0;
        float ax = -g->kd * (state->velocity[0] - vx) - state->push[0], ay = -g->kd * (state->velocity[1] - vy) - state->push[1];
        if (dragging) {
            ax -= g->kp * state->place[0];
            ay -= g->kp * state->place[1];
        }
        forward = state->trim[0] + ax * cy + ay * sy;
        left = state->trim[1] - ax * sy + ay * cy;
        if (dragging && g->stick_pitch) forward = GRAVITY * tanf(g->stick_pitch);
        if (dragging && g->stick_roll) left = -GRAVITY * tanf(g->stick_roll);
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
            // distance 0 keeps the range from takeoff.
            rate = g->distance > 0 ? clamp(g->distance - state->setpoint, -g->approach * dt, g->approach * dt) / dt : 0;
            state->setpoint += rate * dt;
            if (state->have_normal && g->square) {
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
        // The nudged hold point: nearer the tag by nudge_forward, and nudge_left to the drone's left (seen from the drone, the tag then sits that far further round to the right).
        float ease = state->reacquired_us ? clamp((now_us - state->reacquired_us) / (float)REACQUIRE_EASE_US, 0, 1) : 1;
        float hold = ease * g->kp * (range - (state->setpoint - g->nudge_forward)), place = ease * g->kside * range * wrap(around - (state->arc - g->nudge_left / range));
        // The velocity damping aims at the nudged hold point's own speed.
        float toward = hold - g->kd * (closing + rate - state->nudge_rate[0]), across = place - g->kd * (sideways + arc_rate * range - state->nudge_rate[1]);
        // Cancel the learned push outright, in the yaw-turned frame, so a turn does not swing it sideways as a body-fixed trim would.
        float ax = toward * ux - across * uy - state->push[0];
        float ay = toward * uy + across * ux - state->push[1];
        forward = ax * cy + ay * sy;
        left = -ax * sy + ay * cy;
        // A steady push fixed to the body (level-reference error, thrust-vector trim) would leave steady range and place errors: their position terms, in body axes, slowly build a trim that takes it over.
        // It does not grow while the tilt it would add is already at the limit (anti-windup): a trim wound up while the attitude lagged would otherwise carry the drone past its place once it got there.
        float hx = hold * ux - place * uy, hy = hold * uy + place * ux;
        float step[2] = {(hx * cy + hy * sy) * dt / fmaxf(g->trim_tau, dt), (-hx * sy + hy * cy) * dt / fmaxf(g->trim_tau, dt)};
        float limit = GRAVITY * tanf(g->tilt);
        float total[2] = {forward + state->trim[0], left + state->trim[1]};
        bool moving = hypotf(state->nudge_rate[0], state->nudge_rate[1]) > NUDGE_MOVING;
        for (int i = 0; i < 2; i++) {
            // Nor while the hold point is being moved: the error then is the move, not a steady push.
            if (moving) continue;
            if (fabsf(total[i]) < limit || step[i] * total[i] < 0) state->trim[i] = clamp(state->trim[i] + step[i], -g->trim_max, g->trim_max);
        }
        forward += state->trim[0];
        left += state->trim[1];
        // Tag right of the bearing target: turn right, a negative yaw rate.
        output->yaw_rate = clamp(-g->kyaw * wrap(bearing - state->bearing), -g->yaw_rate, g->yaw_rate);
        state->tracking = true;
    }
    output->tracking = true;
    state->carry[0] = clamp(state->trim[0] - (state->push[0] * cy + state->push[1] * sy), -g->trim_max, g->trim_max);
    state->carry[1] = clamp(state->trim[1] - (-state->push[0] * sy + state->push[1] * cy), -g->trim_max, g->trim_max);
    output->velocity[0] = state->velocity[0] * sy - state->velocity[1] * cy;
    output->velocity[1] = state->velocity[0] * cy + state->velocity[1] * sy;
    output->pitch = clamp(atanf(forward / GRAVITY) - g->thrust_tilt * state->thrust_tilt, -g->tilt, g->tilt);
    // Keep the tag in view: leaning nose-down turns the camera down and moves the tag up the picture, so the pitch is limited to put the tag (as last seen, relative to level) no further than view from the picture's centre. On 2026-11-02 04:55 UTC a tag 9 degrees above level left the top of the camera's 18 degrees as the body pitched past 11 degrees, and the hold, blind, swung out of control.
    if (g->view > 0 && state->have_pose) {
        float elevation = atan2f(state->position[2], fmaxf(hypotf(state->position[0], state->position[1]), 0.1f));
        // The body flies at this pitch plus pitch_trim (the attitude controller adds it), and that is what turns the camera.
        // It only holds back a lean that would lose the tag, never asks for one: level stays allowed. On 2026-11-02 08:04 UTC the liftoff's nose-up caught the tag 19 degrees above level from the floor, the band lay wholly nose-up, the pitch was pinned at -10.75 degrees, and the drone drove backward blind on that stale sighting.
        float low = fminf(-g->view - elevation + g->cam_pitch - g->pitch_trim, 0), high = fmaxf(g->view - elevation + g->cam_pitch - g->pitch_trim, 0);
        output->pitch = clamp(output->pitch, low, high);
    }
    output->roll = clamp(atanf(-left / GRAVITY), -g->tilt, g->tilt);
}

void hover_update(HoverState *state, const HoverGains *g, int mode, const HoverPose *pose, int64_t now_us, float dt,
                  float roll, float pitch, float yaw_rate, float vertical_g, float force, const HoverDrag *drag, const HoverFlow *flow,
                  const HoverGps *gps, HoverOutput *output) {
    hold(state, g, mode, pose, now_us, dt, roll, pitch, yaw_rate, vertical_g, force, drag, flow, gps, output);
    if (state->landed || state->lifted >= TAKEOFF_EASE_S) return;
    // Negative while on the floor, counting up to the wait; zero from liftoff, counting the ease.
    state->lifted = state->lifted < 0 && state->height >= LIFTED_HEIGHT ? 0 : state->lifted + dt;
    // The share of the resting attitude still in the targets: all of it on the floor, fading to none over the ease.
    float share = state->lifted < 0 ? 1 : fmaxf(0, 1 - state->lifted / TAKEOFF_EASE_S);
    output->roll += share * (state->rest[0] - output->roll);
    output->pitch += share * (state->rest[1] - output->pitch);
}
