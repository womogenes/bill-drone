#include <math.h>
#include <string.h>

#include "guard.h"

// hover.c's own limits, repeated here: a tag unseen this long is lost (LOST_US), and a GPS fix older than this no longer corrects the velocity (GPS_STALE_US).
#define GUARD_LOST_US 250000
#define GUARD_GPS_US 500000
// A nudge that moves less than this in a tick (m) is not the pilot's: the laptop streams it in 1-2 cm steps every 50 ms while an arrow is held.
#define NUDGE_STILL 0.001f
// A pattern started on the tag stops once the tag has been unseen this long (us), not at the hold's own LOST_US: on 2026-11-02 the hold lost the tag past 250 ms 26 times in 84 s on it (six flights), and 8 times past 1 s, which at 250 ms would have stopped most 8 s patterns while the hold dead-reckoned through them.
#define PATTERN_LOST_US 1000000
// Braking ends once the speed estimate is under FAILSAFE_SLOW (m/s); then the hold's own brake (hover.c BRAKE_S, BRAKED) settles it and it holds where it stopped.
#define FAILSAFE_SLOW 0.5f
// While the failsafe brakes, the hover's braking is kept at least this (s), so its place follows the drone and only the velocity loop acts.
#define FAILSAFE_BRAKING 1.0f
// The failsafe's hold lands if the drone gets this far from where the hold began (m): the GPS's position wanders 1-2 m, a hold that is not holding goes further.
#define FAILSAFE_DRIFT 3.0f
// The failsafe's landing descends at least this fast (m/s), as the link-loss landing does (flight_rt.c LINK_LAND_RATE): down in seconds, so it drifts less on the way.
#define FAILSAFE_LAND_RATE 1.0f

int guard_hover(Guard *guard, const GuardGains *g, HoverState *hover, HoverGains *gains, int mode, int64_t now_us, float dt) {
    // A new hover (hover_rest cleared the last): a new failsafe.
    if (!hover->engaged) {
        guard->failsafe = FAILSAFE_OFF;
        guard->far = false;
        guard->phase_time = guard->fast = guard->quiet = 0;
    }
    // The laptop's horizontal input: a tilt (free flight without the tag) or a hold-point nudge moving (the arrows on the tag, a speed on the GPS hold, a pattern's hold-point channels).
    bool input = gains->stick_pitch || gains->stick_roll || fabsf(gains->nudge_forward - hover->nudge[0]) > NUDGE_STILL || fabsf(gains->nudge_left - hover->nudge[1]) > NUDGE_STILL;
    guard->quiet = input ? 0 : guard->quiet + dt;
    bool seeing = hover->have_pose && now_us - hover->seen_us <= GUARD_LOST_US;
    bool located = gains->gps_tau > 0 && hover->gps_us && now_us - hover->gps_us <= GUARD_GPS_US;
    bool flying = hover->engaged && !hover->landed && hover->lifted >= 0;
    // Only the GPS-corrected estimate is trusted: the drag model alone read under 0.2 m/s while the drone flew 3 m in 4.5 s (2026-11-02 02:46 UTC).
    bool watching = mode == HOVER_HOLD && flying && located && !seeing;
    float speed = hypotf(hover->velocity[0], hover->velocity[1]), distance = hypotf(hover->track[0], hover->track[1]);
    guard->fast = watching && !input && g->speed > 0 && speed > g->speed ? guard->fast + dt : 0;
    guard->phase_time += dt;
    int was = guard->failsafe;
    switch (guard->failsafe) {
    case FAILSAFE_OFF:
        if (watching && (guard->fast >= g->time || (g->distance > 0 && distance > g->distance && guard->quiet >= g->time))) {
            guard->failsafe = FAILSAFE_BRAKE;
            guard->far = guard->fast < g->time;
        }
        break;
    case FAILSAFE_BRAKE:
        // A landing asked for while it brakes (the pilot's, or the link-loss one) is the failsafe's: still braking hard and at least FAILSAFE_LAND_RATE. On 2026-11-09 the pilot pressed land at 05:55:49 and 06:39:10 UTC, 3 and 4.5 s after the failsafe would have fired, into a 0.5 m/s descent at 3.5-4.7 m/s.
        if (!flying) guard->failsafe = FAILSAFE_OFF;
        else if (mode != HOVER_HOLD) guard->failsafe = FAILSAFE_LAND;
        else if (speed < FAILSAFE_SLOW) guard->failsafe = FAILSAFE_HOLD;
        // Not slow in time, it lands. Holding instead unless still over failsafe_speed (with a 5 m FAILSAFE_DRIFT) let more drones fly off in sw/sim/outdoor.py (2026-11-09: at 3 m/s of wind 1 of 48 past 10 m against none, worst distance from takeoff p90 26 m against 22, 8 lost against 2).
        else if (guard->phase_time >= g->land) guard->failsafe = FAILSAFE_LAND;
        break;
    case FAILSAFE_HOLD:
        // The pilot takes over by touching the stick, or by asking for a landing; flying off again from the hold lands.
        if (mode != HOVER_HOLD || !flying || input) guard->failsafe = FAILSAFE_OFF;
        else if (guard->fast >= g->time || hypotf(hover->track[0] - guard->anchor[0], hover->track[1] - guard->anchor[1]) > FAILSAFE_DRIFT) guard->failsafe = FAILSAFE_LAND;
        break;
    default:
        // Landing: until touchdown, or the hover is switched off (manual flight).
        break;
    }
    if (guard->failsafe != was) {
        guard->phase_time = guard->fast = 0;
        guard->anchor[0] = hover->track[0];
        guard->anchor[1] = hover->track[1];
    }
    if (guard->failsafe == FAILSAFE_OFF) return mode;
    // The laptop's stick and horizontal nudge are ignored (the nudge held where it was, so its speed fades to none).
    gains->stick_pitch = gains->stick_roll = 0;
    gains->nudge_forward = hover->nudge[0];
    gains->nudge_left = hover->nudge[1];
    if (guard->failsafe == FAILSAFE_HOLD) return mode;
    // Braking, and landing after a brake that failed: the velocity loop alone (the place follows the drone), up to the failsafe's tilt.
    hover->braking = fmaxf(hover->braking, FAILSAFE_BRAKING);
    gains->tilt = fmaxf(gains->tilt, g->tilt);
    if (guard->failsafe == FAILSAFE_BRAKE) return mode;
    gains->land_rate = fmaxf(gains->land_rate, FAILSAFE_LAND_RATE);
    return mode == HOVER_LAND_FAST ? HOVER_LAND_FAST : HOVER_LAND;
}

void guard_pattern_reset(Guard *guard) {
    guard->started = guard->on_tag = guard->located = false;
    guard->stopped = PATTERN_PLAYING;
}

int guard_pattern(Guard *guard, const GuardGains *g, const HoverState *hover, int64_t now_us, float roll, float pitch, float stick_roll, float stick_pitch, const float *nudge, const float *shift) {
    if (guard->stopped) return guard->stopped;
    bool engaged = hover && hover->engaged, seeing = engaged && hover->have_pose && now_us - hover->seen_us <= GUARD_LOST_US;
    bool lost = !(engaged && hover->have_pose && now_us - hover->seen_us <= PATTERN_LOST_US);
    // Where the drone is: on the tag the tag relative to it (yaw-turned level frame, m; between sightings moved on by the hover's velocity, as hover.c leads it), else the hover's dead-reckoned (GPS-corrected when it has the GPS) track and height.
    float here[3] = {0, 0, 0};
    if ((guard->started ? guard->on_tag : seeing) && !lost) {
        float age = fminf(fmaxf((now_us - hover->capture_us) / 1e6f, 0), PATTERN_LOST_US / 1e6f);
        here[0] = hover->position[0] - hover->velocity[0] * age;
        here[1] = hover->position[1] - hover->velocity[1] * age;
        here[2] = hover->position[2];
    } else if (engaged) {
        here[0] = hover->track[0], here[1] = hover->track[1], here[2] = hover->height;
    }
    if (!guard->started) {
        guard->started = true;
        guard->on_tag = seeing;
        guard->located = engaged;
        memcpy(guard->origin, here, sizeof(here));
        guard->nudge[0] = nudge[0];
        guard->nudge[1] = nudge[1];
    }
    // Where the pattern means it to be: its start, moved by its own hold-point channels (sw/excite.py speed flies 1.5 m out and back on purpose). On the tag toward it and to the left of the line to it, so the tag relative to the drone moves the other way; without it along the heading and to its left.
    float expected[3];
    memcpy(expected, guard->origin, sizeof(expected));
    if (guard->on_tag) {
        float range = fmaxf(hypotf(guard->origin[0], guard->origin[1]), 0.1f), ux = guard->origin[0] / range, uy = guard->origin[1] / range;
        expected[0] -= shift[0] * ux - shift[1] * uy;
        expected[1] -= shift[0] * uy + shift[1] * ux;
    } else if (engaged) {
        float cy = cosf(hover->yaw), sy = sinf(hover->yaw);
        expected[0] += shift[0] * cy - shift[1] * sy;
        expected[1] += shift[0] * sy + shift[1] * cy;
    }
    float dx = here[0] - expected[0], dy = here[1] - expected[1], dz = here[2] - expected[2];
    if (stick_roll || stick_pitch || fabsf(nudge[0] - guard->nudge[0]) > NUDGE_STILL || fabsf(nudge[1] - guard->nudge[1]) > NUDGE_STILL) guard->stopped = PATTERN_STICK;
    else if (g->pattern_tilt > 0 && (fabsf(roll) > g->pattern_tilt || fabsf(pitch) > g->pattern_tilt)) guard->stopped = PATTERN_TILT;
    else if (guard->on_tag && lost) guard->stopped = PATTERN_LOST;
    // Only where the pattern started with a position (in the hover), and while it still has one (a manual flight after has none).
    else if (g->pattern_distance > 0 && guard->located && engaged && sqrtf(dx * dx + dy * dy + dz * dz) > g->pattern_distance) guard->stopped = PATTERN_FAR;
    return guard->stopped;
}
