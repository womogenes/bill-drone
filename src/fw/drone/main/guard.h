#ifndef GUARD_H
#define GUARD_H

#include <stdbool.h>
#include <stdint.h>

#include "hover.h"

// Safety guards around the hover (hover.c) and the test pattern (flight_rt.c excite), kept out of hover.c so the hold can be redesigned without them. No ESP-IDF, so the host simulation runs them too (sw/sim/guard.py).
// The fly-away failsafe: airborne in the hover hold without the tag, on the GPS, a drone moving faster than speed for time with no horizontal input from the laptop, or further than distance from where the hover engaged (its first fix) once untouched for time, both by the GPS itself, brakes at the hold's own velocity loop with the stick ignored and at least tilt (FAILSAFE_BRAKE), holds where it stopped once under FAILSAFE_SLOW (FAILSAFE_HOLD), and lands (FAILSAFE_LAND) if it is not that slow within land seconds, a landing is asked for while it brakes, or it flies off again from the hold. A brake without the hold's GPS heading cannot aim: the hold flies its calibration pulses to find it, and with none after FAILSAFE_SEARCH the failsafe lands. On 2026-11-09 05:55:44 and 06:39:08 UTC the GPS hold flew off at 4-6 m/s with nobody on the sticks.
enum { FAILSAFE_OFF, FAILSAFE_BRAKE, FAILSAFE_HOLD, FAILSAFE_LAND };
// Why a test pattern stopped before its end (0: it did not): it moved further than pattern_distance from where it started (moved by the pattern's own hold-point channels), lost the tag it started on, tilted past pattern_tilt, or the laptop's stick or arrows moved.
enum { PATTERN_PLAYING, PATTERN_FAR, PATTERN_LOST, PATTERN_TILT, PATTERN_STICK, PATTERN_LANDING };

// Settings (settings.def failsafe_* and pattern_*), SI units and radians; a speed, distance or pattern limit of 0 turns that check off.
typedef struct {
    float speed, distance, time, land, tilt;
    float pattern_distance, pattern_tilt;
} GuardGains;

typedef struct {
    // The failsafe's phase, whether distance (not speed) set it off, how long the phase has run (s), how long the speed has been past the limit with no input (s), how long since the last horizontal input (s), and where its hold began (m; north, east of the first fix, hover.h gps_fix).
    int failsafe;
    bool far;
    float phase_time, fast, quiet, anchor[2];
    // The test pattern: whether it has started, why it stopped (PATTERN_*), whether it started on the tag, where it started (on the tag the tag relative to the drone, else the hover's track and height, m) and the laptop's nudge then (m; forward, left).
    bool started, on_tag, located;
    int stopped;
    float origin[3], nudge[2];
    // How long the failsafe's brake has had no heading to aim by (s).
    float blind;
} Guard;

// Each hover tick, before hover_update: the fly-away failsafe. Returns the hover mode to fly (HOVER_LAND once it lands) and changes gains (the copy the runtime passes to hover_update) to ignore the laptop's stick and nudge and brake harder; hover is read, and its braking and searching set while the failsafe brakes. mode is the requested hover mode.
int guard_hover(Guard *guard, const GuardGains *g, HoverState *hover, HoverGains *gains, int mode, int64_t now_us, float dt);
// Each tick a test pattern plays: PATTERN_PLAYING while it may go on, else why it must stop (sticky until guard_pattern_reset). hover is the hover's state while the hover flies (NULL otherwise: only the tilt and the stick are checked), roll and pitch the measured attitude from level (radians), stick_roll and stick_pitch the laptop's tilt (radians), nudge its hold-point nudge and shift how far the pattern's own hold-point channels have moved the hold point (m; forward or toward the tag, left).
int guard_pattern(Guard *guard, const GuardGains *g, const HoverState *hover, int64_t now_us, float roll, float pitch, float stick_roll, float stick_pitch, const float *nudge, const float *shift);
void guard_pattern_reset(Guard *guard);

#endif
