#ifndef HOVER_H
#define HOVER_H

#include <stdbool.h>
#include <stdint.h>

// Hover and landing: height from the IMU's vertical acceleration, corrected by the camera when it sees the tag, which also holds horizontal position and heading. No ESP-IDF, so the host can replay recordings through it.
// Camera tag frame (fw/esp32cam/README.md): an upright tag on a wall in front of the forward-facing camera; x right along the wall as seen facing it, y down, z into the wall; the camera has negative z.
// Body conventions (flight_math.c): positive pitch (nose down) accelerates forward, positive roll (right side down) accelerates right, positive yaw rate turns left.

enum { HOVER_OFF, HOVER_HOLD, HOVER_LAND, HOVER_LAND_FAST };

// Tunable, in SI units and radians.
typedef struct {
    // Horizontal position loop (camera only): m/s² per m of error and per m/s of velocity; the steepest tilt it may command.
    float kp, kd, tilt;
    // Height loop: climb rate per m of height error (1/s), then m/s² per m/s of climb-rate error and per m of its integral; the vehicle mass that, through the thrust table, gives hover thrust (kg).
    float kz, kdz, kiz, mass;
    // Height above the engage point to climb to (m) and the fastest climb (m/s); landing descent rate (m/s), tripled for HOVER_LAND_FAST.
    float climb, climb_rate, land_rate;
    // Heading loop (camera only): yaw rate per radian of heading error, and the fastest yaw rate it may command.
    float kyaw, yaw_rate;
    // Tag hold (camera only): center 1 flies square in front of the tag at distance (m) from the wall and at its height, facing it, moving the horizontal setpoint there at approach (m/s); center 0 holds the position and heading where the tag was first seen at the height climb. ceiling is the most height above the engage point centring may ask for, and the height a takeoff climbs toward while looking for the tag (m).
    float center, distance, approach, ceiling;
} HoverGains;

// One camera pose: the camera origin in the tag frame (m), the camera's forward axis in the tag frame (rotation * (0, 0, 1)), and its capture time.
typedef struct {
    uint32_t sequence;
    int64_t capture_us;
    float position[3], forward[3];
} HoverPose;

typedef struct {
    // engaged: running since the mode left HOVER_OFF; landed: a landing touched down and thrust is ramping to zero; found: the tag has been seen since engaging.
    bool engaged, landed, have_pose, have_setpoint, found;
    // Vertical specific force at rest, in g, learned while not engaged.
    float bias;
    // Height above the engage point (m) and climb rate (m/s), from the IMU and the camera; the height target; the last climb-rate command (m/s); the climb-rate integral (m/s²); the last thrust (N); how long thrust has been low while landing (s).
    float height, climb_rate, target, command, integral, force, low_time;
    // Camera: the newest pose's sequence, capture time and position, its velocity, the camera height that is zero height, the horizontal setpoint, the heading where the tag was first seen and the newest heading.
    uint32_t sequence;
    int64_t capture_us, seen_us;
    float position[3], velocity[3], zero_y, setpoint[2], heading, facing;
} HoverState;

// For the attitude controller: roll/pitch targets and yaw rate (radians), and collective thrust in newtons.
typedef struct {
    float roll, pitch, yaw_rate, force;
} HoverOutput;

// Camera heading in the tag's horizontal plane: 0 facing straight at the wall, positive turned right.
float hover_heading(const float *forward);
// While not hovering: forget the flight and learn the resting vertical specific force (g).
void hover_rest(HoverState *state, float vertical_g, float dt);
// One control tick. mode is HOVER_HOLD, HOVER_LAND or HOVER_LAND_FAST; pose is the newest camera pose or NULL (camera disabled or no tag); vertical_g is the measured specific force along vertical, in g; force is the collective thrust in N being flown now, so taking over mid-flight is smooth.
void hover_update(HoverState *state, const HoverGains *gains, int mode, const HoverPose *pose, int64_t now_us, float dt,
                  float roll, float pitch, float vertical_g, float force, HoverOutput *output);

#endif
