#ifndef HOVER_H
#define HOVER_H

#include <stdbool.h>
#include <stdint.h>

// Hover and landing: height from the IMU's vertical acceleration, corrected by the camera when it sees the tag, which also holds the range and bearing to the tag. No ESP-IDF, so the host can replay recordings through it.
// The camera's tag rotation is ambiguous and noisy at range (fw/esp32cam/README.md), so the hold flies on the tag centre in the camera frame: turned level by the IMU's roll and pitch, it gives range, bearing and height to the tag; the gyro's yaw separates the drone's own turning from its travel, and tilt dead-reckons the velocity between sightings. Only the rotations the camera settled outright steer, slowly, where on the circle about the tag the drone sits.
// Body conventions (flight_math.c): positive pitch (nose down) accelerates forward, positive roll (right side down) accelerates right, positive yaw rate turns left.

enum { HOVER_OFF, HOVER_HOLD, HOVER_LAND, HOVER_LAND_FAST };

// Tunable, in SI units and radians.
typedef struct {
    // Horizontal loop (camera only): m/s² per m of range error and per m/s of velocity (closing and sideways); the steepest tilt it may command.
    float kp, kd, tilt;
    // m/s² per m away from the hold's place on the circle about the tag (camera only): weak, as the wall's normal behind it is known only roughly.
    float kside;
    // Height loop: climb rate per m of height error (1/s), then m/s² per m/s of climb-rate error and per m of its integral; the vehicle mass that, through the thrust table, gives hover thrust (kg).
    float kz, kdz, kiz, mass;
    // Height above the engage point to climb to (m) and the fastest climb (m/s); landing descent rate (m/s), tripled for HOVER_LAND_FAST.
    float climb, climb_rate, land_rate;
    // Heading loop (camera only): yaw rate per radian of bearing error, and the fastest yaw rate it may command.
    float kyaw, yaw_rate;
    // Tag hold (camera only): center 1 faces the tag, holds distance (m) from it (0: the range at takeoff) and flies at its height, moving the range target there at approach (m/s); center 0 holds the range and bearing where the tag was first seen at the height climb. ceiling is the most height above the engage point centring may ask for, and the height a takeoff climbs toward while looking for the tag (m).
    float center, distance, approach, ceiling;
    // Centring only: 1 moves the place on the circle toward square in front of the tag as the wall's normal is learned, 0 keeps the place where the tag was first seen (the normal from a small tag at range can be several degrees off).
    float square;
    // The camera's tilt up from the body's forward axis, and its roll about its own axis (pi when mounted upside down).
    float cam_pitch, cam_roll;
    // Rotor drag: the horizontal specific force the rotors' flapping and drag give per m/s of airspeed (1/s; 0 = off, leaving the tilt dead reckoning of before), the forward thrust tilt the servo commands miss (servo zero and mounting), and how slowly the drag-model velocity corrects the estimate (s). On, the velocity estimate runs on the measured specific force turned level and is pulled toward the drag model's, so without the camera it stays bounded instead of drifting, and the hold flies on it from takeoff.
    float drag, drag_tilt, drag_tau;
    // The body-fixed trim that takes over a steady range or place error: its integration time (s) and its limit (m/s²; 1 is about 6 degrees of tilt).
    float trim_tau, trim_max;
    // Thrust tilt: on a tilt-rotor the motors' tilt moves the drone as the body's pitch does, and a steady tilt (holding the body against a CoM offset) would push it along unseen. thrust_tilt (0 = off, 1 = all of it) counts the servos' steady thrust tilt beyond the one pitch_trim expects, low-passed over thrust_tilt_tau (s), in the velocity dead reckoning and takes it off the pitch the hold asks for, so the thrust points where the hold wants.
    float thrust_tilt, thrust_tilt_tau;
    // The trim a hold starts from (m/s²; body forward, left): flight_rt.c's self-trim carries each hop's learned trim over, so the steady push need not be relearned from zero every hop.
    float trim_start_x, trim_start_y;
    // Not settings: the laptop's nudge of the hold point (m; toward the tag, to the drone's left, and up), streamed with the targets (the dashboard's arrows and W/S in hover).
    float nudge_forward, nudge_left, nudge_up;
    // Not a setting: the attitude controller's pitch_trim (radians, filled in by the runtime). The drone flies level at this pitch (a tail-heavy drone hovers nose-up with its servos tilted forward), so horizontal acceleration follows the pitch beyond it.
    float pitch_trim;
} HoverGains;

// One camera sighting: the tag centre in the camera frame (m; right, down, forward), the wall's normal into it in the camera frame when the camera settled the tag's tilt (zero otherwise), and its capture time.
typedef struct {
    uint32_t sequence;
    int64_t capture_us;
    float tag[3], normal[3];
} HoverPose;

// The accelerometer's specific force in the level reference's body axes (g; forward, left, up: the IMU turned by pose_zero) and the thrust vector's tilt forward of the body's up axis that the servos were last commanded to (radians, thrust-weighted over both sides); hover delays the tilt by the servos' dead time.
typedef struct {
    float specific[3], tilt;
} HoverDrag;

#define HOVER_HISTORY 24

typedef struct {
    // engaged: running since the mode left HOVER_OFF; landed: a landing touched down and thrust is ramping to zero; found: the tag has been seen since engaging; tracking: the horizontal hold has flown on the tag since, so the velocity estimate is good.
    bool engaged, landed, have_pose, have_setpoint, have_normal, found, tracking;
    // Vertical specific force at rest, in g, learned while not engaged.
    float bias;
    // Height above the engage point (m) and climb rate (m/s), from the IMU and the camera; the height target; the last climb-rate command (m/s); the climb-rate integral (m/s²); the last thrust (N); how long thrust has been low while landing (s).
    float height, climb_rate, target, command, integral, force, low_time;
    // Heading integrated from the gyro (positive left, drifting), and the attitude, heading, horizontal velocity and commanded thrust tilt of recent ticks, so a sighting is levelled with the attitude it was captured at and compared with the velocity of its time, and the drag model sees the tilt the servos have reached.
    float yaw;
    int64_t history_us[HOVER_HISTORY];
    float history[HOVER_HISTORY][6];
    int next;
    // Camera: the newest sighting's sequence and capture time; the tag relative to the drone in the level frame turned by yaw (forward, left, up at yaw 0) and the drone's velocity in it; the tag's height above the engage point; the range target (m) and the bearing target (radians, positive right); the heading of the line to the tag that holds the drone's place on the circle about it, and the smoothed heading of the wall's normal, both in the yaw-turned frame.
    uint32_t sequence;
    int64_t capture_us, seen_us;
    float position[3], velocity[3], tag_height, setpoint, bearing, arc, normal;
    // Horizontal acceleration trim in body axes (m/s²; forward, left).
    float trim[2];
    // Rotor drag: the drag-model velocity's offset learned while the camera measures the velocity (m/s; body forward, left), the integrated acceleration's bias learned from the drag model without it (m/s²; body forward, left), and the place dead-reckoned since the tag was last seen (m; yaw-turned frame), which the hold returns to without it.
    float drag_offset[2], drag_bias[2], place[2];
    // The drag model's velocity this tick (m/s; body forward, left, after its offset), for the flight log; dragging: it ran this tick (drag on and airborne).
    float drag_velocity[2];
    bool dragging;
    // The servos' steady thrust tilt beyond the one pitch_trim expects (radians, positive forward), as thrust_tilt counts it.
    float thrust_tilt;
    // How long a landing's descent has stalled (s).
    float stall_time;
    // The hold-point nudge last seen and how fast it is moving (m, m/s; toward the tag, left, up), so a moving hold point is flown at its speed instead of lagging it by kd / kp.
    float nudge[3], nudge_rate[3];
    // The horizontal acceleration the tilt does not explain (m/s²; yaw-turned frame: room wash, level-reference error, unmodelled thrust tilt), learned from the camera's velocity corrections; the hold cancels it directly.
    float push[2];
    // What a later hold should start its trim from (m/s²; body forward, left): the trim plus the push's opposite turned into the body, for flight_rt.c's carry-over.
    float carry[2];
} HoverState;

// For the attitude controller: roll/pitch targets and yaw rate (radians), and collective thrust in newtons. While tracking is set hover damps the horizontal velocity itself, from the tag or, briefly lost, dead-reckoned since, and velocity is that estimate in body axes (m/s; right, forward).
typedef struct {
    float roll, pitch, yaw_rate, force;
    bool tracking;
    float velocity[2];
} HoverOutput;

// While not hovering: forget the hover. With the motors off, learn the resting vertical specific force (g); spinning (manual flight), keep integrating the climb rate and height so a later hover or landing starts from them.
void hover_rest(HoverState *state, float vertical_g, bool spinning, float dt);
// One control tick. mode is HOVER_HOLD, HOVER_LAND or HOVER_LAND_FAST; pose is the newest camera sighting or NULL (camera disabled or no tag); roll and pitch are the measured attitude from level and yaw_rate the measured heading rate (rad/s, positive left); vertical_g is the measured specific force along vertical, in g; force is the collective thrust in N being flown now, so taking over mid-flight is smooth; drag is what the rotor-drag model needs (used when gains->drag is set).
void hover_update(HoverState *state, const HoverGains *gains, int mode, const HoverPose *pose, int64_t now_us, float dt,
                  float roll, float pitch, float yaw_rate, float vertical_g, float force, const HoverDrag *drag, HoverOutput *output);

#endif
