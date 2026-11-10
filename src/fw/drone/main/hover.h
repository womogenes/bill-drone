#ifndef HOVER_H
#define HOVER_H

#include <stdbool.h>
#include <stdint.h>

// Hover and landing: height from the IMU's vertical acceleration, corrected by the camera when it sees the tag (and without it by the barometer, GPS or flow when they are on), which also holds the range and bearing to the tag. No ESP-IDF, so the host can replay recordings through it.
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
    // Optical flow without the tag (flight_rt.c flow_input): the scene's assumed distance along the camera's axis (m; 0 = off) turns the flow's translational rate into velocity across the camera, which pulls the estimate over flow_tau (s) and teaches the drag model its sideways offset (the wind, as the camera teaches it on the tag). Holding still needs only zero flow, so the distance sets the correction's strength, not where it settles.
    float flow_depth, flow_tau;
    // Speed from the servos: forward airspeed pitches this airframe (rotor flapping), so the pitch loop holds it with a steady thrust tilt, tilt = speed_pitch * pitch + speed_tilt * forward speed + offset (fitted on the camera's speed over five tag hovers, 2026-10-31 and 11-01: speed_tilt 5.0 degrees per m/s, speed_pitch -0.54, the servo compensation's share; 0.19 m/s error on flights left out of the fit). speed_tilt (radians per m/s; 0 = off) turns it on; the offset (CoM and servo zero) is learned from the camera while it sees the tag, else from the first SPEED_TAKEOFF_S airborne plus speed_takeoff (radians: the climb reads that much lower); without the tag it pulls the forward velocity estimate over speed_tau (s).
    float speed_tilt, speed_pitch, speed_takeoff, speed_tau;
    // The dashboard's arrows on the tag move the hold point at this speed (m/s); kept here so it is set like the rest. The hold flies at the point's speed (nudge_rate).
    float nudge_speed;
    // The furthest from the picture's centre the hold's pitch may put the tag, as last seen (radians; 0 = no limit): the camera sees about 18 degrees either way and the body overshoots.
    float view;
    // GPS without the tag (hover.c take_gps): its velocity pulls the estimate over gps_tau (s; 0 = off), its position the hold's place over gps_place_tau and its height the height over gps_height_tau (s; 0 = not used); gps_heading is the first guess at where the drone's nose points at engage (radians clockwise from true north): the hold takes the GPS's velocity and position only once the flight's own accelerations against the GPS's have pinned the heading (hover.c HEADING_SURE), so the guess no longer steers anything. Firmware before that (2026-11-09, 4320732 and earlier) trusted it from the first fix and rarely corrected it: there point the nose at it (0 = north) for takeoff.
    float gps_tau, gps_place_tau, gps_height_tau, gps_heading;
    // Barometer without the tag (hover.c take_baro): its height above the ground it read with the motors off pulls the height and climb rate over baro_tau (s; 0 = off, the IMU's height alone as before). On, the hold also never asks for more height than ceiling, so a height above it descends.
    float baro_tau;
    // Not settings: the laptop's nudge of the hold point (m; toward the tag, to the drone's left, and up), streamed with the targets (the dashboard's arrows and W/S in hover).
    float nudge_forward, nudge_left, nudge_up;
    // Not a setting: the attitude controller's pitch_trim (radians, filled in by the runtime). The drone flies level at this pitch (a tail-heavy drone hovers nose-up with its servos tilted forward), so horizontal acceleration follows the pitch beyond it.
    float pitch_trim;
    // Not settings: the laptop's tilt without the tag (radians; pitch positive nose down, roll positive right side down), filled in by the runtime; on the tag the arrows move the hold point instead (nudge). While one is held its axis flies that tilt outright instead of holding; once released the hold brakes and holds where the drone stopped.
    float stick_pitch, stick_roll;
    // Thrust direction (hover.c THRUST_DIRECTION_TAU): the share of the motors' mean forward tilt that moves the drone as the same body pitch does (0 = off; the airframe model's tilt_push is 0.66). On, the hold's tilt (within tilt) aims the thrust, body pitch plus this share of the tilt the servos reached, and a slow correction finds the body pitch that gives it (within THRUST_BODY_MAX), in place of thrust_tilt's take on the pitch asked for: at 4 m/s on 2026-11-09 05:55 UTC the forward speed's nose-up moment held the motors 27 degrees forward, so the -15 degree body pitch asked for braked with a near vertical thrust. Last, so copies of hover.c from before it keep their layout (sw/sim/landing_test.py --source).
    float thrust_direction;
    // GPS heading calibration (hover.c CALIBRATE_S): the tilt of the pulses the hold flies while the GPS's heading is not yet pinned (radians; 0 = off): forward and back, then left and back, a pulse pair moving the simulated drone about 0.15 m at 5 degrees. The liftoff alone usually pins the heading (2-13 s after engaging on the 2026-11-09 replays, 2.6 s median in the simulator); the pulses are for a takeoff too gentle to.
    float gps_calibrate;
    // The Kalman filter (ekf.c, hover_estimate): 0 = the hold flies its own estimates and the filter is only logged; 1 = without the tag the hold flies the filter's heading, horizontal velocity and travel wherever the filter is sure of them (HoverEstimate horizontal), its own estimates running underneath for where it is not; 2 = and, while holding, the filter's height and climb rate from liftoff (a landing keeps the hold's own, carried on from the filter's).
    float ekf;
    // The fastest the hold flies back to its hold point without the tag (m/s; 0 = no limit): the place's pull is capped where the velocity damping meets it at this speed (the offset counts as at most return_speed × kd / kp), so a hold point meters away is flown back to and not lunged at. Uncapped the pull asks for kp / kd of speed per meter (1.3 m/s at kp 2, kd 1.5), more than the drone can stop from: in sw/sim/outdoor.py the hold points left 3-7 m behind by the drift before the heading was pinned were flown back to at 2.5-4.3 m/s and overshot until the fly-away failsafe landed the drone.
    float return_speed;
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

// One camera flow measurement: its frame's capture time and the scene's translational angular rate in camera axes (rad/s; right, down), the drone's own rotation over the interval taken out.
typedef struct {
    int64_t capture_us;
    float rate[2];
} HoverFlow;

// One GPS solution: when it was measured (us, this clock), velocity (m/s; north, east, up) and position (m; north and east of any fixed origin, and up).
typedef struct {
    int64_t capture_us;
    float velocity[3], position[3];
} HoverGps;

// One barometer reading: when the pressure it gives was measured (us, this clock) and the pressure altitude (m in the standard atmosphere; only its changes matter).
typedef struct {
    int64_t capture_us;
    float altitude;
} HoverBaro;

// The Kalman filter's estimate (ekf.c ekf_estimate) for a hold flown on it (gains ekf): the nose's heading (radians counterclockwise from north), the velocity (m/s; north, west, up), the position (m; north, west, from any fixed point) and the height above the ground it last rested on (m); horizontal while its heading is known and its velocity sure enough to fly on, vertical while its height is.
typedef struct {
    float yaw, velocity[3], position[2], height;
    bool horizontal, vertical;
} HoverEstimate;

#define HOVER_HISTORY 24

typedef struct {
    // engaged: running since the mode left HOVER_OFF; landed: a landing touched down and thrust is ramping to zero; found: the tag has been seen since engaging; tracking: the horizontal hold has flown on the tag since, so the velocity estimate is good.
    bool engaged, landed, have_pose, have_setpoint, have_normal, found, tracking;
    // Vertical specific force at rest, in g, learned while not engaged.
    float bias;
    // Height above the engage point (m) and climb rate (m/s), from the IMU and the camera; the height target; the last climb-rate command (m/s); the climb-rate integral (m/s²); the last thrust (N).
    float height, climb_rate, target, command, integral, force;
    // Heading integrated from the gyro (positive left, drifting), and the attitude, heading, horizontal velocity, commanded thrust tilt and inertial velocity (state->inertial) of recent ticks, so a sighting is levelled with the attitude it was captured at and compared with the velocity of its time, and the drag model sees the tilt the servos have reached.
    float yaw;
    int64_t history_us[HOVER_HISTORY];
    float history[HOVER_HISTORY][8];
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
    // Landing (hover.c, the probe): the thrust the hold flew (N) and the vertical specific force it read (g), both smoothed, and the specific force over the last moments (g); how long the landing has run (s) and the height it started from (m); how long its descent has stalled (s); the vertical speed the accelerometer gained over the last moments (m/s). probing: thrust is being wound down under ceiling (N) to feel for the floor, from the vertical thrust and specific force it began at (N, g), with the vertical thrust the motors have reached (N), the vertical speed the accelerometer has gained since and what the air would have taken (m/s); then the wait before another probe after one found air (s), and the sink it left (m/s, fading).
    float hold_force, hold_g, recent_g, land_time, land_height, stall_time, impulse;
    bool probing;
    float ceiling, probe_start, probe_g, probe_force, probe_velocity, probe_expected, probe_wait, sink;
    // The hold-point nudge last seen and how fast it is moving (m, m/s; toward the tag, left, up), so a moving hold point is flown at its speed instead of lagging it by kd / kp.
    float nudge[3], nudge_rate[3];
    // The horizontal acceleration the tilt does not explain (m/s²; yaw-turned frame: room wash, level-reference error, unmodelled thrust tilt), learned from the camera's velocity corrections; the hold cancels it directly.
    float push[2];
    // What a later hold should start its trim from (m/s²; body forward, left): the trim plus the push's opposite turned into the body, for flight_rt.c's carry-over.
    float carry[2];
    // Optical flow (hover.c take_flow): the vertical acceleration's bias it has found (m/s²), and the previous flow's capture time (us, 0 before the first), so each flow corrects over the time it covers.
    float flow_bias;
    int64_t flow_us;
    // Since a stick was let go, how long the hold may still take to brake (s; hover.c BRAKE_S, ended early once under BRAKED): meanwhile the place follows the drone, so it holds where it stops, not where the stick was let go.
    float braking;
    // Speed from the servos (gains speed_tilt): the thrust tilt less speed_pitch times the pitch, smoothed (radians); its offset, and the sum and time of the takeoff window that first sets it; the forward speed it gives (m/s, NaN until the offset is known).
    float speed_raw, speed_offset, speed_sum, speed_time, speed;
    bool speed_ready;
    // When the tag was last found again after being lost (us; hover.c REACQUIRE_EASE_US).
    int64_t reacquired_us;
    // The horizontal acceleration measured (the specific force turned level) integrated since engaging, uncorrected (m/s; yaw-turned frame): its changes against the GPS's velocity changes give the heading. The dead-reckoned position the GPS corrects (m; yaw-turned frame, from the first fix).
    float inertial[2], track[2];
    // GPS (take_gps): the newest fix's capture time (us, 0 before the first since engaging); the hold frame's heading (radians clockwise from north) and the evidence for it, the sum of inertial velocity changes times the GPS's conjugate (m²/s², a complex number as x, y) and the sum of their squared magnitudes (m⁴/s⁴), both fading; the previous block's mean GPS velocity (m/s; north, east) and inertial velocity; the first fix's position (m; north, east, and up less the height then).
    int64_t gps_us;
    float heading, turn[2], turn_weight, gps_velocity[2], gps_inertial[2], origin[3];
    // Takeoff (hover.c TAKEOFF_EASE_S): the attitude the drone rested at when the hover engaged on the ground (radians; roll, pitch; zero when engaged in the air), and the time since liftoff (s; negative before it, counting up from -TAKEOFF_WAIT_S).
    float rest[2], lifted;
    // Whether the last tick's vertical specific force was past LAND_SHOCK (hover.c): a touchdown jolt must last two ticks.
    bool shocked;
    // Barometer (hover.c take_baro): the ground's pressure altitude (m), smoothed while the motors are off (hover_rest) and kept through the flight, and whether there is one; the newest reading's capture time (us; 0 before the first since engaging); how much longer after liftoff it waits before it is used (s); and whether the last reading was used.
    float baro_ground;
    bool baro_grounded;
    int64_t baro_us;
    float baro_wait;
    bool baro_used;
    // Unused since hover.c's SETTLE_S (easing landings to the resting attitude) was taken out on 2026-11-09: it did not stop the tip-overs at touchdown. Kept so the layout stays as deployed.
    float settle;
    // How long a landing has waited to descend while the hold brakes (s; hover.c LAND_WAIT_S).
    float land_wait;
    // Thrust direction (gains thrust_direction): the body pitch the attitude controller was last asked for (radians, from pitch_trim, after the takeoff easing) and how far it must sit from the thrust direction aimed (radians, positive nose down).
    float flown_pitch, direction;
    // Whether this landing's climb rate has yet been under half its descent rate (hover.c: until then a slow climb rate is no stall).
    bool descended;
    // GPS heading (take_gps): the current block's sums of the GPS velocity (north, east) and inertial velocity (x, y) over its fixes and their count, its first fix's capture time and the previous block's middle (us, 0 for none); the heading evidence's standard deviation (radians); whether the heading is pinned (the GPS steers the hold only then); the time the calibration pulses have run (s).
    float heading_sum[4];
    int heading_count;
    int64_t heading_start_us, heading_block_us;
    float turn_square[2], heading_sigma;
    bool aligned;
    float calibrate_time;
    // The dead reckoning at the first fix (m; yaw-turned frame), where origin's position sits.
    float anchor[2];
    // The heading evidence's inertial changes' squared magnitudes summed (m²/s², fading as turn_weight), for its noise floor.
    float heading_inertial;
    // The Kalman filter's estimate for the coming tick (hover_estimate; estimated until hover_update has taken it), whether the last tick flew its horizontal part (flown), its position at the last tick, for the travel since, and the dead reckoning where its position is zero (flown_anchor, m, yaw-turned frame; flown_tied once known).
    HoverEstimate estimate;
    bool estimated, flown, flown_tied;
    float flown_position[2], flown_anchor[2];
    // The heading evidence's steady part: the fading mean of the blocks' changes (GPS north, east; inertial x, y; m/s) and the blocks it is worth, taken off each change before the product.
    float heading_steady[4], heading_steady_weight;
    // The newest fix as the GPS gave it, which needs no heading (m/s north, east; m north, east of the first fix), for the fly-away failsafe (guard.c); and whether the failsafe is braking, when the calibration pulses run unasked until the heading is pinned.
    float gps_fix[4];
    bool searching;
} HoverState;

// For the attitude controller: roll/pitch targets and yaw rate (radians), and collective thrust in newtons. While tracking is set hover damps the horizontal velocity itself, from the tag or, briefly lost, dead-reckoned since, and velocity is that estimate in body axes (m/s; right, forward).
typedef struct {
    float roll, pitch, yaw_rate, force;
    bool tracking;
    float velocity[2];
} HoverOutput;

// While not hovering: forget the hover. With the motors off, learn the resting vertical specific force (g) and, from baro (the newest barometer reading or NULL, each passed once), the ground's pressure altitude; spinning (manual flight), keep integrating the climb rate and height so a later hover or landing starts from them.
void hover_rest(HoverState *state, float vertical_g, bool spinning, float dt, const HoverBaro *baro);
// One control tick. mode is HOVER_HOLD, HOVER_LAND or HOVER_LAND_FAST; pose is the newest camera sighting or NULL (camera disabled or no tag); roll and pitch are the measured attitude from level and yaw_rate the measured heading rate (rad/s, positive left); vertical_g is the measured specific force along vertical, in g; force is the collective thrust in N being flown now, so taking over mid-flight is smooth; drag is what the rotor-drag model needs (used when gains->drag is set).
// A body vector (forward, left, up) in camera axes (right, down, forward): the inverse of the mounting the hover turns sightings by (cam_roll, cam_pitch), for taking the drone's own rotation out of the flow.
void hover_body_to_camera(const HoverGains *gains, const float *body, float *camera);
// The Kalman filter's estimate for the next hover_update (gains ekf), passed every tick the filter has one.
void hover_estimate(HoverState *state, const HoverEstimate *estimate);
// flow is the newest camera flow measurement, gps the newest GPS solution and baro the newest barometer reading, or NULL (each is passed once).
void hover_update(HoverState *state, const HoverGains *gains, int mode, const HoverPose *pose, int64_t now_us, float dt,
                  float roll, float pitch, float yaw_rate, float vertical_g, float force, const HoverDrag *drag, const HoverFlow *flow,
                  const HoverGps *gps, const HoverBaro *baro, HoverOutput *output);

#endif
