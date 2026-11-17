#include <math.h>
#include <string.h>

#include "hover.h"

// How fast the camera corrects the velocity (s); sightings further apart than GAP_US restart it.
#define VELOCITY_TAU 0.4f
#define GAP_US 200000
// The push observer: each camera velocity correction, spread over PUSH_TAU (s), moves the push estimate (critically damped with VELOCITY_TAU at 4 VELOCITY_TAU), up to MAX_PUSH (m/s²).
#define PUSH_TAU 1.6f
#define MAX_PUSH 3.0f
// With the drag model the velocity runs on the measured specific force, which already carries every real push (a thrust tilt the servo command does not show, the room's flow), so what the camera keeps correcting is that level acceleration's own error (the attitude estimate against the accelerometer): the corrections teach it as the drag model's bias, which the dead reckoning takes off, instead of the push, which it does not, so the push only ever grew. On 2026-11-11 with rubber bands on the servos the level acceleration read 0.1-0.4 m/s² forward, the push ran to -2.9 m/s² within 25 s, and the hold, cancelling it outright, leaned the drone at the tag 1-1.5 m inside its range target (hold_replay.py, band_hold.py).
// Without the tag the dead-reckoned velocity leaks toward zero over this long (s), as the trim never quite matches the push it cancels.
#define DRIFT_TAU 3.0f
// How fast the camera pulls the IMU's height and climb rate toward its own (s).
#define CAMERA_TAU 0.15f
// Without the camera the integrated climb rate leaks over this long (s), so an accelerometer bias cannot run away: airborne in hold toward the last climb-rate command, so a long climb keeps the speed it really has; on the ground (where the drone cannot follow the command) and landing toward zero, so touchdown still shows as a stalled descent.
#define LEAK_TAU 10.0f
// Resting specific force smoothing (s).
#define BIAS_TAU 0.5f
// Fastest commanded descent from the height error (m/s; a climb goes to hover_climb_rate, which MAX_CLIMB capped at 0.5 m/s until 2026-11-12, so the 0.6 set that day made no difference), most vertical acceleration asked for (m/s²), and the integral's limit (m/s²).
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
// A probe starts once the descent command has eased in and the descent has stalled (climb rate under half the descent rate for STALL_S), or the accelerometer shows the descent stopped by something (the speed gained over the last IMPULSE_TAU (s) jumps up by IMPULSE of the descent rate, and by at least IMPULSE_MIN (m/s): slower contacts are not told from the hold's own corrections), or the height estimate is below LOW_HEIGHT (m), or at the latest once the landing has run as long as the descent from its starting height should take, plus LAND_MARGIN of that and LAND_SLACK (s). All of them can be wrong in the air, which a probe only costs a short sink. IMPULSE was 0.4 until 2026-11-12, when a soft outdoor contact (01:11 UTC) gained 0.18 m/s over IMPULSE_TAU against 0.2 at the 0.5 m/s rate and the first probe waited 1.5 s for the "due" start; the descents' own peaks that night were 0.01-0.08 m/s.
#ifndef STALL_S
#define STALL_S 0.2f
#endif
#ifndef IMPULSE_TAU
#define IMPULSE_TAU 0.1f
#endif
#ifndef IMPULSE
#define IMPULSE 0.3f
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
// After touchdown thrust falls from the hold's to zero over LANDED_RAMP (s) with the attitude targets at the attitude the floor gives rather than level, and a shock or tilt past the contact limits below still cuts it at once (06:08 pitched over to 42 degrees nose-down as thrust faded over 0.5 s with the servos at their 60 degree limits).
#ifndef LANDED_RAMP
#define LANDED_RAMP 0.25f
#endif
// Contact: a landing that hits harder than LAND_SHOCK (specific force along vertical, g), or tilts past LAND_TILT from level, stops the motors at once. With no legs the drone touches down on its body, and thrust above the first contact point levers it over (2026-10-28 05:01: 12 degrees nose-down at contact, 1.6 g and over its nose 0.15 s later). Hovering stays under 1.35 g; a tilt alone is no proof of contact (05:05 pitched to 15 degrees in the air at 0.7 m, and a cut there drops the drone), so LAND_TILT is only for a drone already going over.
// The tilt cut applies only near the floor: under LAND_TILT_HEIGHT (m) by the height estimate, which reads 10-30 cm off there, or once a probe is feeling for it. On 2026-11-02 04:16 UTC a landing pressed at 0.9 m, mid-way through a 24-degree pitch swing, cut both motors 0.1 s later.
#ifndef LAND_TILT_HEIGHT
#define LAND_TILT_HEIGHT 0.3f
#endif
#ifndef LAND_TILT
#define LAND_TILT (25.0f * DEGREE)
#endif
// Landing blind (no tag in view, the barometer on): the height is then the barometer's, and it cannot find the floor. Against the floor each 2026-11-09 landing came down on (baro_floor.py) the floor itself had moved -0.48 to +0.64 m from the takeoff's (weather, wind pressure), and the last 0.3 s before contact read a median 0.27 m high on top, so the height read -0.51 to +0.89 m at contact. So blind, neither the tilt cut nor a probe goes by the height: the tilt cut waits for a probe's evidence of the floor (BLIND_FELT of PROBE_DEPTH lost to the air with the accelerometer under PROBE_SHARE of it), the "low" start is the camera's only, and the "due" start allows BLIND_HEIGHT more height. In sw/sim/landing_test.py a probe in the air (most started by a stalled descent) made the tilt cut live and the drone's 20-30 degree swings did the rest: blind landings cut in the air fell from 35 to 1 of 181 with --outdoor (the one at 38 degrees of pitch, losing control) and from 77 to 0 of 175 with --no-camera --baro; replayed on the 11 landings of that night, probes started in the air in 9 before and in none now.
#ifndef BLIND_FELT
#define BLIND_FELT 0.5f
#endif
#ifndef BLIND_HEIGHT
#define BLIND_HEIGHT 0.5f
#endif
// Nor is a blind touchdown declared while the climb rate still reads a descent over BLIND_CLIMB (m/s, or half the land rate if more): ground effect then holds the specific force up as the probe winds thrust down. 2026-11-09 06:10 UTC: a probe started 0.25 m up on the barometer's height reading under the takeoff floor, declared touchdown 0.12 s before contact at 77 % of the hold's thrust with the climb rate at -0.89 m/s, and the drone dropped the last 10 cm (replayed: landing_test --replay ... --contact 81.89). The probe then goes on feeling until the accelerometer shows air (falling) or the descent stops. Without the barometer (the IMU's height, camera lost) landings go on as before: in landing_test's camera rig the IMU-only height drifted down below the floor while the drone rose, and only the "low" start ended those landings.
#ifndef BLIND_CLIMB
#define BLIND_CLIMB 0.2f
#endif
// But on the floor the barometer reads low with the rotors spinning (2026-11-12 01:09 and 01:11 UTC: 1.3-1.6 m below its reading just before contact, so the height estimate "descended" to -1.8 m and the climb rate read -0.3 to -0.7 m/s for 2 s on the floor), which held those touchdowns off until the probe had wound thrust to nothing. So a probe that has taken away BLIND_DEEP (m/s) of the air's vertical speed while the accelerometer saw under PROBE_SHARE of it and no fall is on the floor whatever the climb rate reads: that is about a third of the hold's thrust, which no ground effect makes up in the air (the 06:10 touchdown above came at 77 %, and the cushion at the rotors' height on the feet is about 5 %).
#ifndef BLIND_DEEP
#define BLIND_DEEP 0.6f
#endif
// The Kalman filter lifts that veto. On soft ground the first touch is too soft for LAND_SHOCK and the barometer's false descent in the wash held the climb rate at -0.3 to -0.4 m/s on the grass (2026-11-15 07:13 UTC: contact at about 27.8 s at 0.88-1.17 g, the height estimate to -1.0 m, and every probe vetoed by "sinking" until the drone's rocking on the grass read as a fall: the motors ran at hover thrust 2 s until the pilot's stop). Near the floor the filter barely takes the barometer (ekf.c EKF_NEAR), and there it read -0.45 m/s at contact and -0.01 to +0.03 from 0.2 s after. But near the floor in the air it read as little: -0.04 to -0.17 m/s for 1.4 s at 06:49 (40.0-41.4 s, pitch swinging 8 degrees, until contact at 41.75 s), and through zero to +0.12 at 06:47 (40.3-41.4 s, a second before contact at 42.4 s), so stillness alone is not the floor. What contact adds is a stop: the filter's climb rate coming within EKF_STILL (m/s) of zero from a descent at least EKF_STOP (m/s) faster than EKF_RECOVER (m/s²) and the thrust over the hold's would slow it (its lowest climb rate recovering at that rate), and staying there for EKF_STILL_S (s). The stalls above slowed at 0.2-0.3 m/s², the contacts at 1.4-3; over the 81 landings recorded on 2026-11-09, 11-12 and 11-15 UTC such a stop began only 0.03-0.2 s after a contact, never in the air. The thrust's part is the hold braking: in landing_test --outdoor --tray --ekf, without it, the hold catching the sink a probe in the air had left stopped a 0.5 m/s descent at 1.1 m/s² with up to a quarter more thrust than the hold's, and 7 of 3318 landings were called on that 0.3-1.2 m up. Once such a stop has lasted EKF_STILL_S, a probe whose accelerometer saw under PROBE_SHARE of the air's loss (probe_settled) is on the floor whatever the hold's climb rate reads. The stop does not start a probe of its own: in the same simulator a filter step of 0.1 m/s on a drone pitched 22 degrees in the wind passed for one, and the probe it started made the blind tilt cut live 1.4 m up (seed 12046), which is what BLIND_FELT is for. The filter staying within EKF_STILL while the probe takes away PROBE_DEPTH of the air's speed is a floor test of its own: it integrates the same accelerometer, against gravity rather than the specific force at the probe's start, which after a jolt reads the jolt's fading back to 1 g as a fall (probe_velocity). Replayed with the filter as logged (landing_test --replay ... --contact 27.8), 07:13 touches down 0.85 s after contact, at the end of the first probe after it, at 68 % of the hold's thrust.
#ifndef EKF_STILL
#define EKF_STILL 0.1f
#endif
#ifndef EKF_STOP
#define EKF_STOP 0.15f
#endif
#ifndef EKF_RECOVER
#define EKF_RECOVER 1.0f
#endif
#ifndef EKF_STILL_S
#define EKF_STILL_S 0.3f
#endif
#ifndef LAND_SHOCK
#define LAND_SHOCK 1.45f
#endif
// The descent command ramps in over LAND_EASE (s), so pressing land does not step collective down.
#define LAND_EASE 0.5f
// Held L (HOVER_LAND_FAST) descends this many times hover_land_rate: 2 m/s at the 1 m/s set on 2026-11-08 (was 3x of 0.2). Faster is not safer: about 1 kg on two 10-inch rotors has an induced velocity near 6.4 m/s, and a straight descent beyond roughly a quarter of it (1.5-2 m/s) enters the vortex ring state, where lift and control drop out unpredictably. Where the height knows the floor (LAND_DESCENT below) it only sets how fast held L may come down above LAND_FINAL_HEIGHT.
#define LAND_FAST_FACTOR 2
// Above LAND_FINAL_HEIGHT (m, by the height estimate) a landing comes down faster, at up to LAND_DESCENT (m/s), braking at LAND_BRAKE (m/s²) so it reaches the set rate by LAND_FINAL_HEIGHT; the touchdown itself stays at hover_land_rate (the tip-overs of 2026-11-12 touched down at 0.85-1 m/s, the upright ones at 0.5). Only on a height that knows the floor, the camera's or the barometer's, and the barometer read up to 0.9 m above the floor at contact on 2026-11-09, so the final height leaves that and more; 1.5 m until 2026-11-14, when the 5 m landings spent 3 s of their 8 at the set rate below it (the user found the bottom too slow) and the barometer-and-IMU height read 0.0-0.14 m at contact, so 1.0 m (the 2026-11-09 offset with 10 cm to spare). LAND_DESCENT 0 (or under the set rate) keeps one rate throughout, as before 2026-11-12, when a landing from 3 m at 0.5 m/s took 6 s to the cut (01:55 UTC).
#ifndef LAND_DESCENT
#define LAND_DESCENT 1.2f
#endif
#ifndef LAND_BRAKE
#define LAND_BRAKE 0.8f
#endif
#ifndef LAND_FINAL_HEIGHT
#define LAND_FINAL_HEIGHT 1.0f
#endif
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
// The GPS's climb rate is not taken until TAKEOFF_GPS_S (s) after liftoff, nor while the barometer holds the height: at the 2026-11-09 GPS-held liftoffs it read 0.5-1.2 m/s down for 0.3-0.5 s while the drone climbed (05:45, 05:50, 06:19, 06:25 UTC), and at hover_gps_tau 0.5 it held the climb-rate estimate near zero while the drone climbed at up to 1.5 m/s, so the climb loop kept 0.07-0.14 g of excess thrust and 7 of 12 such takeoffs reached 1.15-1.62 m against a 0.8 m goal (none of the 7 without the GPS hold did; gps_truth.py). In flight it lags 40-180 ms and is 0.18-0.39 m/s RMS off, where the barometer tracks the accelerometer to 0.07-0.13 m over 4 s. Replayed on those 12 takeoffs (height_replay.py) the height estimate's error over the first 2.5 s fell from 0.12-0.40 m RMS (worst 0.85) to 0.04-0.21 (worst 0.40); in sw/sim/outdoor.py (06:19 settings, the false descent modelled) the climb's peak over the height it settles at fell from a median of 0.12-0.19 m (90% 0.41-0.53) to 0.02-0.07 (90% 0.16-0.32, the rest the barometer's own wander).
#ifndef TAKEOFF_GPS_S
#define TAKEOFF_GPS_S 2.0f
#endif
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
// A landing waits to descend (holding its height) while the horizontal hold brakes a speed over LAND_WAIT_SPEED (m/s), for at most LAND_WAIT_S (s) from its start, unless already under LAND_TILT_HEIGHT: B pressed in fast flight began descending at once while still braking (asked for after the 2026-11-08 outdoor flights), so it stops first, then comes down. The wait is not landing time (land_time stays 0), so the probes' timing, LAND_EASE and the "due" timeout all start from the descent.
#ifndef LAND_WAIT_SPEED
#define LAND_WAIT_SPEED 0.5f
#endif
// It was off (0) as deployed 2026-11-09: forced on every landing (LAND_WAIT_SPEED 0.02) it declared touchdowns in the air in landing_test. Waiting held only a zero climb rate (the height target followed the height), so the climb rate's drift moved the drone (--outdoor seed 48: up 0.25 m in the 2 s), and when the descent began late the stall start took the descent not yet under way for a stall and probed in the air, where the probe found "floor" (seed 48 at 1.33 m). Now the wait holds the height, a stall needs the descent first (descended), and the blind rules above keep a probe in the air from cutting: forced on with --outdoor, touchdowns in the air 4 to 0 and cuts in the air 32 to 2 of 181 (both on airframes losing control in 2-3 m/s of wind, which the 2 s more airborne meets a little more often: 29 against 22 crashed or fell over without the wait), contact speed at most 0.98 m/s against 4.51; with the drag model and no GPS (--no-camera --baro --set hover_drag=1) no landing in the air either way.
#ifndef LAND_WAIT_S
#define LAND_WAIT_S 2.0f
#endif
// Speed from the servos (gains speed_tilt): the tilt and pitch are smoothed over SPEED_SMOOTH (s; 0.5 fitted best against the camera's speed, against 0.1-1), the offset is first the mean over the first SPEED_TAKEOFF_S airborne (plus speed_takeoff) and then follows the camera's speed over SPEED_LEARN (s) while it sees the tag.
#define SPEED_SMOOTH 0.5f
#define SPEED_TAKEOFF_S 2.0f
#define SPEED_LEARN 5.0f
// After the tag is found again following a loss, the hold's position terms ease in over this long (us): the drift while blind arrives as one jump in the sighting, and at full gain it commanded the tilt limit on both axes at once (2026-11-02 05:13 UTC: 0.5 m after 0.7 s lost, pitch -10 and roll +10, and the overshoot to 17 degrees made the pilot stop it).
#define REACQUIRE_EASE_US 1000000
// The heading's evidence: the GPS velocity and the inertial velocity are averaged over blocks of fixes spanning HEADING_BLOCK_US, and each block's change from the previous one (inertial times the GPS's conjugate) is summed, fading over HEADING_TAU (s; the gyro heading drifts under 10 degrees a minute, gps_truth.py). Fix-to-fix changes (0.1 s) were mostly GPS noise: on 2026-11-09 06:34 UTC the evidence pointed within 10 degrees of the truth from 15 s on, but their magnitudes' agreement stayed at 0.34-0.54, under the 0.6 that gate needed, so the heading never left its first guess (off 49 degrees all flight; 05:45, 05:52, 06:14, 06:33, 06:38 alike, heading_replay.py). A gap longer than HEADING_GAP_US starts the blocks over.
#define HEADING_BLOCK_US 400000
#define HEADING_GAP_US 500000
#define HEADING_TAU 30.0f
// The blocks' changes carry a steady part that is no evidence: an attitude estimate a few degrees off (the gyro's in-flight bias against the accelerometer's slow pull) is a constant acceleration in the inertial velocity that the GPS's does not have, 0.9 m/s² in sw/sim/outdoor.py's seed 39, whose products with the GPS's noise then outweighed two cycles of calibration pulses: the heading was never pinned and the drone drifted 27 m at 1.4 m/s. So each change counts less its mean over HEADING_STEADY_TAU (s), the GPS's and the inertial alike (a steady real acceleration is in both and turns out with the same angle).
#ifndef HEADING_STEADY_TAU
#define HEADING_STEADY_TAU 5.0f
#endif
// The steady part starts as this many blocks at zero, so the first changes after engaging (the takeoff's own acceleration, which pins most flights) still count nearly whole: started empty, the 2026-11-09 flights replayed pinned at a median 4.9 s instead of 2.3; with 3 blocks, 2.5 s (heading_replay.py).
#ifndef HEADING_STEADY_PRIOR
#define HEADING_STEADY_PRIOR 3.0f
#endif
// The evidence's standard deviation (radians): the root of the products' summed squared scatter across its direction, but at least what GPS velocity changes HEADING_NOISE (m/s per axis) off would scatter them, over the sum's magnitude. Under HEADING_SURE, with the sum worth at least HEADING_BLOCKS products of its average size, the heading is pinned (taken outright the first time, then followed over HEADING_PULL, s) and the GPS steers the hold; over HEADING_LOST it lets go again. Counting the whole magnitudes as noise (the root of half their squares over the sum) pinned all ten 2026-11-09 flights within 2 degrees of the truth, but only at 21-42 s; the scatter alone with 4 products' worth, at 4-18 s, but in sw/sim/outdoor.py a liftoff kick's two or three large changes then outweighed everything after, and a drone drifting off at 1.8 m/s without the GPS waited 6 s. With the floor at 0.15 m/s the simulator's pins came later (median 5.5 s) and 28 of 96 still-air runs flew off before or around them; at 0.08, 2.6 s and 1 (the replays pin alike at either).
#ifndef HEADING_SURE
#define HEADING_SURE (10.0f * DEGREE)
#endif
#define HEADING_LOST (25.0f * DEGREE)
#define HEADING_PULL 1.0f
#ifndef HEADING_BLOCKS
#define HEADING_BLOCKS 2.0f
#endif
#ifndef HEADING_NOISE
#define HEADING_NOISE 0.08f
#endif
// Heading calibration pulses (gains gps_calibrate): after CALIBRATE_AFTER_S (s) of hold without a pinned heading, slots of CALIBRATE_S (s): tilt forward, back, rest, left, right, rest, at most CALIBRATE_CYCLES times; a pulse pair under way is finished once the heading is pinned. In sw/sim/outdoor.py (2026-11-09) one pair moved the drone 0.14-0.18 m at 5 degrees (0.08-0.09 at 3), but four cycles never pinned 0.65-1.3 m, as the hold has no position to return to before the GPS steers it.
#define CALIBRATE_AFTER_S 2.0f
#define CALIBRATE_S 0.5f
#define CALIBRATE_CYCLES 2
// The pulses' tilt at least, while the fly-away failsafe brakes without a heading (radians; state searching).
#ifndef CALIBRATE_SEARCH
#define CALIBRATE_SEARCH 0.0873f
#endif
// Barometer (take_baro): the ground's pressure altitude is smoothed over BARO_GROUND_TAU (s) while the motors are off. Readings are not used until BARO_SETTLE_S (s) after liftoff, while the rotors' wash and ground effect change fastest. Near the floor they stay in: weighting them out under half a metre left the hold to the IMU there, and in the sim an accelerometer reading high then held drones on the floor that thought they were climbing, and low holds stalled (sw/sim/outdoor.py and landing_test.py --baro, 2026-11-08). Measured on 2026-11-09 against the accelerometer (baro_floor.py): with the motors spinning on the floor it reads -0.13 to +0.14 m (no steady wash), in the first 0.4 s after liftoff 0.3-2.9 m off either way, and in a landing's last 0.3 s a median 0.27 m above the floor it then rests on, small beside the floor's own move of -0.48 to +0.64 m over a flight. Touchdown goes by the accelerometer, and blind the height starts no probe (BLIND_FELT). A reading older than BARO_STALE_US no longer stands in for the climb rate's leak.
#define BARO_GROUND_TAU 1.0f
#ifndef BARO_SETTLE_S
#define BARO_SETTLE_S 1.0f
#endif
#define BARO_STALE_US 300000
// The filter's time constant is at least this many readings, so its discrete steps stay well inside stability.
#define BARO_MIN_STEPS 10
// Thrust direction (gains thrust_direction): the correction from the hold's tilt to the body pitch moves over THRUST_DIRECTION_TAU (s), and the body pitch asked for stays within THRUST_BODY_MAX (radians), short of flight_rt.c's tip cut (settings.def tip_limit, 45 degrees by default). In sw/sim/outdoor.py (2026-11-09, 06:38 settings) 0.3-0.7 s fed a 0.3-0.4 Hz swing of the hold itself (pitch RMS 12-16 degrees in still air against 5.5 off), 1.5-2.5 s held as well without it.
#ifndef THRUST_DIRECTION_TAU
#define THRUST_DIRECTION_TAU 1.5f
#endif
#ifndef THRUST_BODY_MAX
#define THRUST_BODY_MAX (35.0f * DEGREE)
#endif

static float wrap(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

void hover_rest(HoverState *state, float vertical_g, bool spinning, float dt, const HoverBaro *baro) {
    float bias = state->bias, climb_rate = 0, height = 0, ground = state->baro_ground;
    bool grounded = state->baro_grounded;
    int64_t baro_us = state->baro_us;
    if (!spinning) {
        bias = bias ? bias + (vertical_g - bias) * (1 - expf(-dt / BIAS_TAU)) : vertical_g;
        // The ground's pressure altitude, over the time each reading covers; spinning, it stays as it was before the motors started.
        if (baro) {
            float span = baro_us && baro->capture_us > baro_us ? fminf((baro->capture_us - baro_us) / 1e6f, 0.1f) : 0;
            ground = grounded ? ground + (baro->altitude - ground) * (1 - expf(-span / BARO_GROUND_TAU)) : baro->altitude;
            grounded = true;
            baro_us = baro->capture_us;
        }
    } else if (bias) {
        // Manual flight: keep the climb rate and height current (leaking like the no-camera hover), so a hover or landing taken over mid-flight starts from the real climb, not from rest.
        climb_rate = (state->climb_rate + (vertical_g - bias) * GRAVITY * dt) * expf(-dt / LEAK_TAU);
        height = state->height + climb_rate * dt;
    }
    memset(state, 0, sizeof(*state));
    state->bias = bias;
    state->climb_rate = climb_rate;
    state->height = height;
    state->baro_ground = ground;
    state->baro_grounded = grounded;
    state->baro_us = baro_us;
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
    int64_t interval_us = pose->capture_us - state->capture_us;
    if (state->have_pose && interval_us > 0 && interval_us <= GAP_US) {
        float interval = interval_us / 1e6f, keep = expf(-interval / VELOCITY_TAU);
        // The tag stands still, so its apparent motion is the drone's, reversed: the average velocity over the interval, compared with the estimate at its middle. The horizontal estimate runs on tilt between sightings, so the correction moves it and its recorded past alike.
        float *then = state->history[recorded(state, pose->capture_us - interval_us / 2)];
        float correction[2];
        bool biased = state->dragging;
        for (int i = 0; i < 2; i++) {
            correction[i] = (1 - keep) * ((state->position[i] - position[i]) / interval - then[3 + i]);
            state->velocity[i] += correction[i];
            for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3 + i] += correction[i];
            // A velocity the tilt keeps getting wrong the same way is a push it does not know about: learn it (only once flying on the tag, so the takeoff's ground contact does not teach it).
            if (state->tracking && !biased) state->push[i] = clampf(state->push[i] + correction[i] / PUSH_TAU, -MAX_PUSH, MAX_PUSH);
        }
        // With the drag model it is the level acceleration's bias instead, in body axes as the drag model keeps it.
        if (state->tracking && biased) {
            float cy = cosf(state->yaw), sy = sinf(state->yaw);
            state->drag_bias[0] = clampf(state->drag_bias[0] - (correction[0] * cy + correction[1] * sy) / PUSH_TAU, -MAX_BIAS, MAX_BIAS);
            state->drag_bias[1] = clampf(state->drag_bias[1] - (-correction[0] * sy + correction[1] * cy) / PUSH_TAU, -MAX_BIAS, MAX_BIAS);
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

// The heading from the GPS: the inertial velocity (the measured acceleration integrated in the hold's frame) and the GPS's (north, east) change alike up to the turn between the frames, so each block's change of the one times the other's conjugate points at the heading.
static void learn_heading(HoverState *state, const HoverGps *fix, const float *then) {
    if (!state->gps_us || fix->capture_us - state->gps_us > HEADING_GAP_US) {
        state->heading_count = 0;
        state->heading_block_us = 0;
        memset(state->heading_steady, 0, sizeof(state->heading_steady));
        state->heading_steady_weight = HEADING_STEADY_PRIOR;
    }
    if (!state->heading_count) {
        memset(state->heading_sum, 0, sizeof(state->heading_sum));
        state->heading_start_us = fix->capture_us;
    }
    float values[4] = {fix->velocity[0], fix->velocity[1], then[6], then[7]};
    for (int i = 0; i < 4; i++) state->heading_sum[i] += values[i];
    state->heading_count++;
    if (fix->capture_us - state->heading_start_us < HEADING_BLOCK_US) return;
    float mean[4];
    for (int i = 0; i < 4; i++) mean[i] = state->heading_sum[i] / state->heading_count;
    int64_t middle = (state->heading_start_us + fix->capture_us) / 2;
    if (state->heading_block_us) {
        // Inertial change times the GPS change's conjugate, the GPS's as north - i east: their product's angle is the heading.
        float span = (middle - state->heading_block_us) / 1e6f, keep = expf(-span / HEADING_TAU);
        // Each change less its steady part (the mean before this block; the product scaled as a running covariance's).
        float change[4] = {mean[0] - state->gps_velocity[0], mean[1] - state->gps_velocity[1], mean[2] - state->gps_inertial[0], mean[3] - state->gps_inertial[1]};
        state->heading_steady_weight = state->heading_steady_weight * expf(-span / HEADING_STEADY_TAU) + 1;
        float share = 1 / state->heading_steady_weight, scale = sqrtf(1 - share);
        for (int i = 0; i < 4; i++) {
            float beyond = change[i] - state->heading_steady[i];
            state->heading_steady[i] += beyond * share;
            change[i] = beyond * scale;
        }
        float ax = change[2], ay = change[3], cn = change[0], ce = -change[1];
        float x = ax * cn + ay * ce, y = ay * cn - ax * ce;
        state->turn[0] = state->turn[0] * keep + x;
        state->turn[1] = state->turn[1] * keep + y;
        state->turn_weight = state->turn_weight * keep * keep + x * x + y * y;
        state->turn_square[0] = state->turn_square[0] * keep * keep + x * x - y * y;
        state->turn_square[1] = state->turn_square[1] * keep * keep + 2 * x * y;
        float size = hypotf(state->turn[0], state->turn[1]), evidence = atan2f(state->turn[1], state->turn[0]);
        // The products' scatter across the evidence's direction: half of (|p|² less the real part of p² turned back by twice its angle).
        state->heading_inertial = state->heading_inertial * keep * keep + ax * ax + ay * ay;
        float across = 0.5f * (state->turn_weight - (state->turn_square[0] * cosf(2 * evidence) + state->turn_square[1] * sinf(2 * evidence)));
        across = fmaxf(across, HEADING_NOISE * HEADING_NOISE * state->heading_inertial);
        state->heading_sigma = size > 0 ? fminf(sqrtf(fmaxf(across, 0)) / size, PI) : PI;
        // As many products as the evidence is worth: a lone block or two scatter little by chance.
        bool enough = size * size >= HEADING_BLOCKS * state->turn_weight;
        if (!state->aligned && enough && state->heading_sigma < HEADING_SURE) {
            state->heading = evidence;
            state->aligned = true;
        } else if (state->aligned && state->heading_sigma > HEADING_LOST) {
            state->aligned = false;
        } else if (state->aligned) {
            state->heading = wrap(state->heading + wrap(evidence - state->heading) * (1 - expf(-span / HEADING_PULL)));
        }
    }
    state->gps_velocity[0] = mean[0];
    state->gps_velocity[1] = mean[1];
    state->gps_inertial[0] = mean[2];
    state->gps_inertial[1] = mean[3];
    state->heading_block_us = middle;
    state->heading_count = 0;
}

// GPS without the tag: its velocity (turned into the hold's frame by the heading) corrects the estimate as it was at the fix's capture, and the past since, over gps_tau; a steady disagreement is the measured acceleration's bias, which it learns (critically damped, as the drag model's). Its velocity changes against the inertial ones give the heading. Its position moves the dead reckoning and the hold's place with it over gps_place_tau, and its climb rate the vertical.
// The horizontal parts wait for the heading to be pinned: turned by a wrong one the GPS's correction and the hold's braking point partly sideways (2026-11-09 06:33-06:38 UTC flew 49-57 degrees off all flight, and 06:38 drifted wildly in under 0.35 m/s of wind), so until then the hold runs on the drag model and the servos' speed as it does without the GPS.
static void take_gps(HoverState *state, const HoverGains *g, const HoverGps *fix, int64_t now_us, float cy, float sy, float dt) {
    bool first = !state->gps_us || fix->capture_us <= state->gps_us;
    float span = first ? dt : fminf((fix->capture_us - state->gps_us) / 1e6f, 0.5f);
    const float *then = state->history[recorded(state, fix->capture_us)];
    if (fix->capture_us > state->gps_us) learn_heading(state, fix, then);
    state->gps_us = fix->capture_us;
    state->gps_fix[0] = fix->velocity[0];
    state->gps_fix[1] = fix->velocity[1];
    // North and east into the hold's frame: x + i y = (north - i east) e^(i heading).
    float ch = cosf(state->heading), sh = sinf(state->heading);
    float tau = fmaxf(g->gps_tau, span), pull = 1 - expf(-span / tau);
    float learn = span / (4 * tau * tau);
    if (state->aligned) {
        float error[2] = {fix->velocity[0] * ch + fix->velocity[1] * sh - then[3], fix->velocity[0] * sh - fix->velocity[1] * ch - then[4]};
        for (int i = 0; i < 2; i++) {
            state->velocity[i] += error[i] * pull;
            for (int k = 0; k < HOVER_HISTORY; k++) state->history[k][3 + i] += error[i] * pull;
        }
        // The bias is in body axes, where the drag model keeps it (a steady level-reference or mounting error turns with the drone).
        state->drag_bias[0] = clampf(state->drag_bias[0] - (error[0] * cy + error[1] * sy) * learn, -MAX_BIAS, MAX_BIAS);
        state->drag_bias[1] = clampf(state->drag_bias[1] - (-error[0] * sy + error[1] * cy) * learn, -MAX_BIAS, MAX_BIAS);
    }
    bool barometric = g->baro_tau > 0 && state->baro_used && state->baro_us && now_us - state->baro_us <= BARO_STALE_US;
    if (state->lifted >= TAKEOFF_GPS_S && !barometric) {
        float climb_error = fix->velocity[2] - state->climb_rate;
        state->climb_rate += climb_error * pull;
        state->accel_bias = clampf(state->accel_bias - climb_error * learn, -MAX_BIAS, MAX_BIAS);
    }
    // Position: the first fix ties the GPS to the dead reckoning (its position and the dead reckoning's then, turned by the heading of each later fix, so a heading pinned later still measures from where the GPS was first taken: tying again at the pinning kept the drift flown before it, 2.6-3.4 m median in sw/sim/outdoor.py); later ones correct it (back-dated by the fix's age) and move the place, which is relative to the hold point, by the same amount.
    float age = clampf((now_us - fix->capture_us) / 1e6f, 0, 0.5f);
    if (first) {
        state->origin[0] = fix->position[0];
        state->origin[1] = fix->position[1];
        state->origin[2] = fix->position[2] - state->height;
        state->anchor[0] = state->track[0];
        state->anchor[1] = state->track[1];
        state->gps_fix[2] = state->gps_fix[3] = 0;
    } else {
        float n = fix->position[0] - state->origin[0], e = fix->position[1] - state->origin[1];
        state->gps_fix[2] = n;
        state->gps_fix[3] = e;
        if (g->gps_place_tau > 0 && state->aligned) {
            float place_pull = 1 - expf(-span / fmaxf(g->gps_place_tau, span));
            float x = n * ch + e * sh + state->anchor[0], y = n * sh - e * ch + state->anchor[1];
            float shift[2] = {(x - (state->track[0] - state->velocity[0] * age)) * place_pull, (y - (state->track[1] - state->velocity[1] * age)) * place_pull};
            for (int i = 0; i < 2; i++) {
                state->track[i] += shift[i];
                state->place[i] += shift[i];
            }
        }
    }
}

// Barometer without the tag: its height above the ground corrects the height, the climb rate and the vertical acceleration's bias (accel_bias, which the GPS also learns) as they were at the reading's capture, as a critically damped third-order complementary filter over baro_tau (the IMU carries what changes faster, the barometer what changes slower), so the height cannot drift away as the IMU's alone did (2026-11-08 01:56 UTC: it read 0.4-0.8 m while the drone climbed to about 6 m). Without the bias a steady accelerometer error b would leave the climb rate 2 baro_tau b off. A hover engaged in the air (from manual flight, or a link-loss landing) takes the barometer's height outright at its first reading, so it starts from the true height instead of the IMU's.
static void take_baro(HoverState *state, const HoverGains *g, const HoverBaro *baro, int64_t now_us) {
    bool first = !state->baro_us;
    float span = first || baro->capture_us <= state->baro_us ? 0 : fminf((baro->capture_us - state->baro_us) / 1e6f, 0.1f);
    state->baro_us = baro->capture_us;
    // No ground was read with the motors off (the barometer came up in flight): the ground is where the height estimate puts it now.
    if (!state->baro_grounded) {
        state->baro_ground = baro->altitude - state->height;
        state->baro_grounded = true;
    }
    state->baro_used = state->baro_wait <= 0;
    if (!state->baro_used) return;
    float above = baro->altitude - state->baro_ground, age = clampf((now_us - baro->capture_us) / 1e6f, 0, 0.5f);
    // Only a hover engaged in the air has its first reading here: on the floor the first comes while it waits.
    if (first) {
        state->height = above + state->climb_rate * age;
        return;
    }
    if (span <= 0) return;
    float error = above - (state->height - state->climb_rate * age), tau = fmaxf(g->baro_tau, BARO_MIN_STEPS * span);
    // A triple pole at -1 / tau.
    state->height += 3 * error * span / tau;
    state->climb_rate += 3 * error * span / (tau * tau);
    state->accel_bias = clampf(state->accel_bias - error * span / (tau * tau * tau), -MAX_BIAS, MAX_BIAS);
}

// Heading calibration (gains gps_calibrate): while the GPS is on and fresh but the heading not yet pinned, hands-off in hold, the hold flies pulses of a known tilt so the accelerometer and the GPS both see a clean velocity change. The pulses are added to the hold's tilt, which brakes the speed they build (at most g tan(tilt) CALIBRATE_S, 0.43 m/s at 5 degrees): aiming the hold's damping at that speed as well doubled the tilt (11-13 degrees beyond the hold's at 5) and, never pinned, moved the simulated drone 1.1-1.3 m median over four cycles (sw/sim/outdoor.py, 2026-11-09). Without them the 2026-11-09 flights replayed pin it 2-13 s after engaging (the liftoff alone usually does it; a still hover can take 10 s and more, heading_replay.py), and until then the hold has no GPS.
static void calibrate(HoverState *state, const HoverGains *g, int mode, int64_t now_us, float dt, bool stick, float *forward, float *left) {
    bool fresh = state->gps_us && now_us - state->gps_us <= GPS_STALE_US;
    // The failsafe braking (guard.c sets searching, and starts calibrate_time over): without a heading its brake cannot aim, so the pulses run whatever the setting, at once, and until the heading is pinned or the failsafe gives up and lands.
    bool searching = state->searching;
    if ((g->gps_calibrate <= 0 && !searching) || g->gps_tau <= 0 || !fresh || mode != HOVER_HOLD || state->height < AIRBORNE) return;
    // The pilot flying: the pulses wait until the hold is hands-off again for CALIBRATE_AFTER_S.
    if (!searching && (stick || state->braking > 0)) {
        state->calibrate_time = 0;
        return;
    }
    float phase = state->calibrate_time - (searching ? 0 : CALIBRATE_AFTER_S);
    int slot = phase >= 0 ? (int)(phase / CALIBRATE_S) : -1;
    // Pinned: finish the pulse pair under way, then stop (and start the wait over, should the heading be lost again).
    if (state->aligned && (slot < 0 || slot % 3 == 2 || state->calibrate_time == 0)) {
        state->calibrate_time = 0;
        return;
    }
    if (!searching && slot >= 6 * CALIBRATE_CYCLES) return;
    state->calibrate_time += dt;
    if (slot < 0 || slot % 3 == 2) return;
    float push = GRAVITY * tanf(searching ? fmaxf(g->gps_calibrate, CALIBRATE_SEARCH) : g->gps_calibrate), extra = slot % 3 == 0 ? push : -push;
    if (slot % 6 < 3) *forward += extra;
    else *left += extra;
}

static void hold(HoverState *state, const HoverGains *g, int mode, const HoverPose *pose, int64_t now_us, float dt,
                 float roll, float pitch, float yaw_rate, float vertical_g, float force, const HoverDrag *drag,
                 const HoverGps *gps, const HoverBaro *baro, HoverOutput *output) {
    float weight = g->mass * GRAVITY;
    bool dragging = g->drag > 0 && drag;
    bool estimated = state->estimated;
    state->estimated = false;
    if (!state->engaged) {
        state->engaged = true;
        bool grounded = force < 0.5f * weight;
        state->rest[0] = grounded ? roll : 0;
        state->rest[1] = grounded ? pitch : 0;
        state->lifted = grounded ? -TAKEOFF_WAIT_S : 0;
        state->accel_bias = 0;
        state->gps_us = 0;
        state->heading = 0;
        state->turn[0] = state->turn[1] = state->turn_weight = state->turn_square[0] = state->turn_square[1] = state->heading_inertial = 0;
        state->heading_count = 0;
        state->heading_block_us = 0;
        state->heading_sigma = PI;
        state->aligned = false;
        state->calibrate_time = 0;
        state->searching = false;
        memset(state->gps_fix, 0, sizeof(state->gps_fix));
        state->inertial[0] = state->inertial[1] = state->track[0] = state->track[1] = 0;
        state->baro_us = 0;
        state->baro_wait = grounded ? BARO_SETTLE_S : 0;
        state->baro_used = false;
        // Continue from the thrust being flown, so taking over a manual flight does not step.
        state->integral = force > 0.5f * weight ? clampf(force / g->mass - GRAVITY, -MAX_INTEGRAL, MAX_INTEGRAL) : 0;
        state->force = force;
        // The carried-over trim is the push the last hold cancelled: start the push estimate from it (yaw is zero at engage, so body and yaw-turned axes agree), and the body trim from zero.
        state->push[0] = clampf(-g->trim_start_x, -MAX_PUSH, MAX_PUSH);
        state->push[1] = clampf(-g->trim_start_y, -MAX_PUSH, MAX_PUSH);
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
        *output = (HoverOutput){.roll = roll, .pitch = pitch, .force = state->force};
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
    // Thrust direction: the body pitch beyond pitch_trim plus the share of the servos' reached tilt beyond pitch_trim's that moves the drone. The correction follows what the last body pitch asked for gave, so it integrates the hold's aim less the thrust direction while the body pitch is free, and stops winding up once a limit holds it.
    if (g->thrust_direction > 0 && drag && state->height >= AIRBORNE) {
        float reached = state->history[recorded(state, now_us - SERVO_DELAY_US)][5] + g->pitch_trim;
        float thrust = pitch - g->pitch_trim + g->thrust_direction * reached;
        state->direction += (state->flown_pitch - thrust - state->direction) * (1 - expf(-dt / THRUST_DIRECTION_TAU));
        state->direction = clampf(state->direction, -THRUST_BODY_MAX, THRUST_BODY_MAX);
    }

    // Rotor drag: the body's horizontal specific force less the thrust vector's forward part (at the tilt the servos have reached) is -drag times the airspeed. It is noisy (vibration, the pitch loop's servo motion), so it pulls the estimate slowly, and the past with it, as the camera's correction does; a steady disagreement is the integrated acceleration's bias, which it learns (a critically damped second-order filter, settling over a few drag_tau). The camera, when it sees the tag, measures the velocity itself: then the disagreement is the drag model's own offset (servo zero, accelerometer bias, mounting) instead, which it teaches below.
    bool seeing = state->have_pose && now_us - state->seen_us <= LOST_US;
    // A fresh GPS fix measures the ground speed: once the heading is pinned it stands in for the drag model's and the servos' airspeed readings.
    bool located = g->gps_tau > 0 && state->gps_us && now_us - state->gps_us <= GPS_STALE_US && !seeing && state->aligned;
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
            state->drag_bias[0] = clampf(state->drag_bias[0] - (error[0] * cy + error[1] * sy) * learn, -MAX_BIAS, MAX_BIAS);
            state->drag_bias[1] = clampf(state->drag_bias[1] - (-error[0] * sy + error[1] * cy) * learn, -MAX_BIAS, MAX_BIAS);
        }
    }

    // IMU: integrate vertical acceleration (a landing from the specific force the hold read); without the camera, leak the climb rate.
    state->climb_rate += ((vertical_g - (mode != HOVER_HOLD ? state->hold_g : state->bias)) * GRAVITY - state->accel_bias) * dt;
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
        // Not while the barometer holds the climb rate, which the leak would only pull toward the command.
        bool barometric = g->baro_tau > 0 && state->baro_used && state->baro_us && now_us - state->baro_us <= BARO_STALE_US;
        float leak = mode == HOVER_HOLD && state->height >= AIRBORNE ? state->command : 0;
        if (!barometric) state->climb_rate += (leak - state->climb_rate) * (1 - expf(-dt / LEAK_TAU));
        if (!dragging) {
            float keep = expf(-dt / DRIFT_TAU);
            state->velocity[0] *= keep;
            state->velocity[1] *= keep;
        }
    }
    if (gps && !seeing && g->gps_tau > 0) {
        take_gps(state, g, gps, now_us, cy, sy, dt);
        located = state->aligned;
    }
    if (state->lifted >= 0) state->baro_wait = fmaxf(0, state->baro_wait - dt);
    if (baro && !seeing && g->baro_tau > 0) take_baro(state, g, baro, now_us);
    if (located && state->dragging) {
        // What the drag model reads beyond the GPS's ground speed, in body axes, is the wind (with its own offset): learned as the camera teaches the offset, and allowed a wind's size.
        float forward = state->velocity[0] * cy + state->velocity[1] * sy, left = -state->velocity[0] * sy + state->velocity[1] * cy;
        state->drag_offset[0] = clampf(state->drag_offset[0] + (state->drag_velocity[0] - forward) * dt / OFFSET_TAU, -MAX_WIND, MAX_WIND);
        state->drag_offset[1] = clampf(state->drag_offset[1] + (state->drag_velocity[1] - left) * dt / OFFSET_TAU, -MAX_WIND, MAX_WIND);
    }
    if (seeing && dragging && state->height >= AIRBORNE) {
        // The camera measures the velocity: whatever the drag model reads beyond it, in body axes, is its offset (servo zero, accelerometer bias, mounting).
        float forward = state->velocity[0] * cy + state->velocity[1] * sy, left = -state->velocity[0] * sy + state->velocity[1] * cy;
        state->drag_offset[0] = clampf(state->drag_offset[0] + (measured[0] - forward) * dt / OFFSET_TAU, -MAX_OFFSET, MAX_OFFSET);
        state->drag_offset[1] = clampf(state->drag_offset[1] + (measured[1] - left) * dt / OFFSET_TAU, -MAX_OFFSET, MAX_OFFSET);
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

    // Climb-rate command: hold moves the target height toward its goal at climb_rate; landing descends at a set rate, once the hold has braked (LAND_WAIT_S). Only the horizontal hold brakes, so without it (neither the tag nor the drag model) there is nothing to wait for.
    bool waiting = mode != HOVER_HOLD && state->land_time == 0 && state->land_wait < LAND_WAIT_S && (state->tracking || state->dragging) && !state->probing && state->height >= LAND_TILT_HEIGHT && hypotf(state->velocity[0], state->velocity[1]) > LAND_WAIT_SPEED;
    // A landing's height knows the floor: the camera's, or the barometer's.
    bool floored = seeing || (g->baro_tau > 0 && state->baro_used && state->baro_us && now_us - state->baro_us <= BARO_STALE_US);
    float command;
    if (mode == HOVER_HOLD) {
        // The goal is climb; centring goes to the tag's height instead, climbs toward the ceiling to bring the tag into view until it has been flown on (a glimpse from the ground does not count), and holds the height reached when it is lost.
        // The height nudge raises the goal once flying (the takeoff climb and the search for the tag keep theirs), and its speed feeds the climb command.
        // Without the tag, W and S move the height goal the same way once airborne (2026-11-14: before the tag was flown on they did nothing), the search's climb to the ceiling included.
        bool aloft = state->tracking || state->dragging || state->height >= AIRBORNE;
        float goal = g->climb, up = aloft ? g->nudge_up : 0;
        if (g->center) goal = seeing ? clampf(state->tag_height + up, MIN_HEIGHT, g->ceiling) : state->tracking ? fmaxf(g->climb, state->target) : clampf(g->ceiling + up, MIN_HEIGHT, g->ceiling);
        else goal += up;
        // With the barometer the height is the true one above the ground, so the ceiling holds for every goal and the nudge: above it the hold descends.
        bool capped = g->baro_tau > 0;
        if (capped) goal = fminf(goal, g->ceiling);
        state->target += clampf(goal - state->target, -g->climb_rate * dt, g->climb_rate * dt);
        if (capped) state->target = fminf(state->target, g->ceiling);
        float rising = aloft ? state->nudge_rate[2] : 0;
        if (capped && state->target >= g->ceiling) rising = fminf(rising, 0);
        command = clampf(g->kz * (state->target - state->height) + rising, -MAX_CLIMB, g->climb_rate);
    } else {
        float rate = -g->land_rate * (mode == HOVER_LAND_FAST ? LAND_FAST_FACTOR : 1);
        // Held L too touches down at the set rate where the height knows the floor, and only comes down faster above it (2026-11-12 01:55 UTC: held L met the ground at 1 m/s, 5 degrees nose-down, and went over backward).
        float fastest = fmaxf(LAND_DESCENT, -rate);
        if (floored && LAND_DESCENT > 0) rate = -fminf(fastest, sqrtf(g->land_rate * g->land_rate + 2 * LAND_BRAKE * fmaxf(0, state->height - LAND_FINAL_HEIGHT)));
        if (waiting) {
            // Waiting, the height the hold was holding is held (kz), not only a zero climb rate.
            command = clampf(g->kz * (state->target - state->height), -MAX_CLIMB, fminf(MAX_CLIMB, g->climb_rate));
        } else {
            command = fmaxf(rate, fminf(state->command, 0) + rate / LAND_EASE * dt);
            state->target = state->height;
        }
    }
    state->command = command;
    // Landing: less the sink a probe that found air left (SINK_TAU).
    float error = command + (mode != HOVER_HOLD ? state->sink : 0) - state->climb_rate;
    state->integral = clampf(state->integral + g->kiz * error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    float up = clampf(g->kdz * error + state->integral, -MAX_VERTICAL, MAX_VERTICAL);
    float lean = fmaxf(0.5f, cosf(roll) * cosf(pitch));
    state->force = g->mass * (GRAVITY + up) / lean;

    bool felt = state->probing && state->probe_expected <= -BLIND_FELT * PROBE_DEPTH && state->probe_velocity > PROBE_SHARE * state->probe_expected;
    bool blind = !seeing && g->baro_tau > 0;
    bool floor_near = blind ? felt : state->probing || state->height < LAND_TILT_HEIGHT;
    // A jolt is a touchdown only if it lasts two ticks: on 2026-11-08 01:54 UTC one reading of 1.5 g (a tick that fell back to a single raw IMU sample, rotor vibration and all, 7 ms off the 10 ms rhythm) cut the motors 0.8 m up, 0.37 s into a landing; the averaged readings in that flight never passed 1.38 g.
    bool jolted = vertical_g > LAND_SHOCK && state->shocked;
    state->shocked = vertical_g > LAND_SHOCK;
    if (mode != HOVER_HOLD && ((floor_near && (fabsf(roll) > LAND_TILT || fabsf(pitch) > LAND_TILT)) || jolted)) {
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
        state->land_time = state->stall_time = state->impulse = state->probe_wait = state->sink = state->land_wait = state->filter_still = state->filter_low = 0;
        state->probing = state->descended = false;
    } else if (waiting) {
        state->land_wait += dt;
    } else {
        // The vertical speed the accelerometer gains this tick, against the hold's specific force.
        // The rate it touches down at (the descent command's final one).
        float rate = g->land_rate * (mode == HOVER_LAND_FAST && !(floored && LAND_DESCENT > 0) ? LAND_FAST_FACTOR : 1), gained = (vertical_g - state->hold_g) * GRAVITY * dt;
        if (state->land_time == 0) state->land_height = state->height;
        state->land_time += dt;
        // Only once the descent command has eased in: before that the drone is not meant to be moving yet.
        // And only once the drone has descended at half the rate: a descent that has not yet begun is no stall.
        state->descended = state->descended || state->climb_rate < -0.5f * rate;
        state->stall_time = state->land_time > LAND_EASE && state->descended && fabsf(state->climb_rate) < 0.5f * rate ? state->stall_time + dt : 0;
        state->impulse = state->impulse * expf(-dt / IMPULSE_TAU) + gained;
        state->probe_wait = fmaxf(0, state->probe_wait - dt);
        state->sink *= expf(-dt / SINK_TAU);
        float due = (LAND_EASE + (fmaxf(state->land_height, 0) + (blind ? BLIND_HEIGHT : 0)) / fmaxf(rate, 0.01f)) * (1 + LAND_MARGIN) + LAND_SLACK;
        bool stopped = state->land_time > LAND_EASE && state->impulse > fmaxf(IMPULSE * rate, IMPULSE_MIN);
        bool low = !blind && state->land_time > LAND_EASE && state->height < LOW_HEIGHT;
        // The filter's stop (EKF_STILL_S), while it is sure of its height.
        if (estimated && state->estimate.vertical) {
            float up = state->estimate.velocity[2];
            state->filter_still = fabsf(up) <= EKF_STILL && (state->filter_still > 0 || state->filter_low <= -EKF_STOP) ? state->filter_still + dt : 0;
            // A stop the thrust explains is the hold braking, not the floor: the lowest climb rate recovers faster by what the thrust asks over the hold's.
            float braking = state->hold_force > 0 ? fmaxf(0, state->force / state->hold_force - 1) * GRAVITY : 0;
            state->filter_low = fminf(up, state->filter_low + (EKF_RECOVER + braking) * dt);
        } else {
            state->filter_still = state->filter_low = 0;
        }
        if (!state->probing && state->probe_wait <= 0 && (state->stall_time >= STALL_S || stopped || low || state->land_time >= due)) {
            state->probing = true;
            state->ceiling = state->force;
            state->probe_start = state->probe_force = state->force * lean;
            // A probe started by a jolt (stopped) judges air and floor against no more than the specific force its thrust explains in the air (probe_settled): against the jolt itself the jolt's fading back to 1 g reads as a fall, and it found air on the floor (2026-11-12 01:09 UTC: started 0.02 s after contact at 1.45 g, "air" 0.18 s later, the motors on the floor 2.3 s more). What it felt (the blind tilt cut) still goes against the specific force before it: in landing_test --outdoor the settled reference made the tilt cut live in the air on drones already losing control.
            state->probe_g = state->recent_g;
            state->probe_air_g = stopped ? fminf(state->recent_g, state->hold_g * state->probe_start / state->hold_force) : state->recent_g;
            state->probe_velocity = state->probe_settled = state->probe_expected = 0;
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
            state->probe_settled += (vertical_g - state->probe_air_g) * GRAVITY * dt;
            bool falling = state->recent_g < state->hold_g - PROBE_FALL || state->probe_settled <= -PROBE_DEPTH;
            bool sinking = blind && state->climb_rate < -fmaxf(0.5f * rate, BLIND_CLIMB);
            if (!falling && (!sinking || state->filter_still >= EKF_STILL_S || state->probe_expected <= -BLIND_DEEP) && state->probe_expected <= -PROBE_DEPTH && state->probe_settled > PROBE_SHARE * state->probe_expected) {
                state->landed = true;
            } else if (falling || (!sinking && state->probe_expected <= -PROBE_DEPTH)) {
                // In the air.
                state->probing = false;
                state->probe_wait = PROBE_WAIT;
                state->stall_time = state->impulse = 0;
                state->sink = clampf(state->probe_velocity, -MAX_SINK, 0);
                state->force = fmaxf(state->force, state->probe_start / lean);
                state->integral = clampf(fmaxf(state->integral, state->force * lean / g->mass - GRAVITY), -MAX_INTEGRAL, MAX_INTEGRAL);
            }
        }
    }
    state->recent_g += (vertical_g - state->recent_g) * (1 - expf(-dt / RECENT_TAU));

    *output = (HoverOutput){.force = state->force};
    // Until the tag is first seen, hold the heading the hover started with (the gyro's, which drifts only slowly): only the tag steers the heading, so a yaw kick at liftoff turned the camera away from it for good (2026-11-02 04:47 UTC: 58 degrees/s, 30 degrees counterclockwise, the tag never seen and the drone at its ceiling looking for it).
    if (!state->found) output->yaw_rate = clampf(-g->kyaw * wrap(state->yaw - state->yaw_hold), -g->yaw_rate, g->yaw_rate);
    float forward, left;
    if (!seeing) {
        // Tag lost after flying on it: keep damping the dead-reckoned velocity, with the trim learned on the tag. Before the tag is first flown on, the attitude controller's own damping holds the drone, unless the drag model keeps the velocity: then the hold flies on it from takeoff, and also returns to the place where the tag was last seen (or where it took off), as far as the dead reckoning knows it.
        if (state->height < AIRBORNE && !lifting) return;
        if (!state->tracking && !dragging) {
            // Nothing to hold on yet (the tag not flown on, no drag model): the arrows lean the drone the nudge's way, hover_tilt at nudge_speed, so a drone that lost the tag at liftoff can be flown back to it (2026-11-14: a liftoff's nose-down kick took the tag out of view and the arrows did nothing); let go, the attitude controller levels it.
            float speed = fmaxf(g->nudge_speed, 0.1f);
            output->pitch = clampf(g->tilt * state->nudge_rate[0] / speed, -g->tilt, g->tilt);
            output->roll = clampf(-g->tilt * state->nudge_rate[1] / speed, -g->tilt, g->tilt);
            return;
        }
        // The arrows fly the drone: the nudge's speed (toward the drone's heading and to its left) is what the velocity damping aims at, so holding an arrow flies at it; with the drag model it also moves the hold point, so letting go stops where the hold point stopped (without it, on the tag's dead reckoning, where the drone stopped).
        float vx = state->nudge_rate[0] * cy - state->nudge_rate[1] * sy, vy = state->nudge_rate[0] * sy + state->nudge_rate[1] * cy;
        if (dragging) {
            // That speed changes by at most nudge_accel a second (hover.h), so a stick let go at speed moves the hold point on as far as the drone can stop in, instead of stopping it where the stick was let go and asking for kd times the speed at once; the fly-away failsafe's brake, which holds the nudge, slows along it too.
            float dx = vx - state->reference[0], dy = vy - state->reference[1], change = hypotf(dx, dy), most = g->nudge_accel * dt;
            if (g->nudge_accel > 0 && change > most) {
                vx = state->reference[0] + dx * most / change;
                vy = state->reference[1] + dy * most / change;
            }
            state->reference[0] = vx;
            state->reference[1] = vy;
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
            // A hold point far away is flown back to at return_speed, not lunged at (hover.h).
            float far = hypotf(state->place[0], state->place[1]), reach = g->return_speed * g->kd / fmaxf(g->kp, 1e-3f);
            float pull = g->return_speed > 0 && far > reach ? g->kp * reach / far : g->kp;
            ax -= pull * state->place[0];
            ay -= pull * state->place[1];
        }
        forward = state->trim[0] + ax * cy + ay * sy;
        left = state->trim[1] - ax * sy + ay * cy;
        if (dragging) calibrate(state, g, mode, now_us, dt, stick, &forward, &left);
        if (dragging && g->stick_pitch) forward = GRAVITY * tanf(g->stick_pitch);
        if (dragging && g->stick_roll) left = -GRAVITY * tanf(g->stick_roll);
    } else {
        // The tag relative to the drone now: the sighting moved on by the drone's velocity over its age, then turned into the body's heading.
        float lead = clampf((now_us - state->capture_us) / 1e6f, 0, 0.1f);
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
        // Centring: on the ground the targets follow the drone and nothing is commanded, since noisy camera velocity would only tilt it on its feet; airborne in hold the range target moves toward distance at the approach speed, so a large offset is flown out rather than lunged at, and the loop follows its rate; landing holds them where they are.
        float rate = 0;
        if (g->center && mode == HOVER_HOLD) {
            if (state->height < AIRBORNE) {
                state->setpoint = range;
                state->arc = around;
                return;
            }
            // distance 0 keeps the range from takeoff.
            rate = g->distance > 0 ? clampf(g->distance - state->setpoint, -g->approach * dt, g->approach * dt) / dt : 0;
            state->setpoint += rate * dt;
        }
        // Toward the tag: close the range error and damp the closing speed toward the target's rate. Across it (positive left): damp the sideways speed toward the target's rate, and pull weakly toward the place on the circle, so the drone neither drifts around the tag nor fights the range and heading loops for it. Moving left turns the line to the tag right (smaller around).
        float ux = px / range, uy = py / range;
        float closing = state->velocity[0] * ux + state->velocity[1] * uy;
        float sideways = -state->velocity[0] * uy + state->velocity[1] * ux;
        // The nudged hold point: nearer the tag by nudge_forward, and nudge_left to the drone's left (seen from the drone, the tag then sits that far further round to the right).
        float ease = state->reacquired_us ? clampf((now_us - state->reacquired_us) / (float)REACQUIRE_EASE_US, 0, 1) : 1;
        float hold = ease * g->kp * (range - (state->setpoint - g->nudge_forward)), place = ease * g->kside * range * wrap(around - (state->arc - g->nudge_left / range));
        // The velocity damping aims at the nudged hold point's own speed.
        float toward = hold - g->kd * (closing + rate - state->nudge_rate[0]), across = place - g->kd * (sideways - state->nudge_rate[1]);
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
            if (fabsf(total[i]) < limit || step[i] * total[i] < 0) state->trim[i] = clampf(state->trim[i] + step[i], -g->trim_max, g->trim_max);
        }
        forward += state->trim[0];
        left += state->trim[1];
        // Tag right of the bearing target: turn right, a negative yaw rate.
        output->yaw_rate = clampf(-g->kyaw * wrap(bearing - state->bearing), -g->yaw_rate, g->yaw_rate);
        state->tracking = true;
    }
    output->tracking = true;
    state->carry[0] = clampf(state->trim[0] - (state->push[0] * cy + state->push[1] * sy), -g->trim_max, g->trim_max);
    state->carry[1] = clampf(state->trim[1] - (-state->push[0] * sy + state->push[1] * cy), -g->trim_max, g->trim_max);
    output->velocity[0] = state->velocity[0] * sy - state->velocity[1] * cy;
    output->velocity[1] = state->velocity[0] * cy + state->velocity[1] * sy;
    if (g->thrust_direction > 0)
        output->pitch = clampf(clampf(atanf(forward / GRAVITY), -g->tilt, g->tilt) + state->direction, -THRUST_BODY_MAX, THRUST_BODY_MAX);
    else
        output->pitch = clampf(atanf(forward / GRAVITY) - g->thrust_tilt * state->thrust_tilt, -g->tilt, g->tilt);
    // Keep the tag in view: leaning nose-down turns the camera down and moves the tag up the picture, so the pitch is limited to put the tag (as last seen, relative to level) no further than view from the picture's centre. On 2026-11-02 04:55 UTC a tag 9 degrees above level left the top of the camera's 18 degrees as the body pitched past 11 degrees, and the hold, blind, swung out of control.
    if (g->view > 0 && state->have_pose) {
        float elevation = atan2f(state->position[2], fmaxf(hypotf(state->position[0], state->position[1]), 0.1f));
        // The body flies at this pitch plus pitch_trim (the attitude controller adds it), and that is what turns the camera.
        // It only holds back a lean that would lose the tag, never asks for one: level stays allowed. On 2026-11-02 08:04 UTC the liftoff's nose-up caught the tag 19 degrees above level from the floor, the band lay wholly nose-up, the pitch was pinned at -10.75 degrees, and the drone drove backward blind on that stale sighting.
        float low = fminf(-g->view - elevation + g->cam_pitch - g->pitch_trim, 0), high = fmaxf(g->view - elevation + g->cam_pitch - g->pitch_trim, 0);
        output->pitch = clampf(output->pitch, low, high);
    }
    output->roll = clampf(atanf(-left / GRAVITY), -g->tilt, g->tilt);
}

void hover_estimate(HoverState *state, const HoverEstimate *estimate) {
    state->estimate = *estimate;
    state->estimated = true;
}

void hover_update(HoverState *state, const HoverGains *g, int mode, const HoverPose *pose, int64_t now_us, float dt,
                  float roll, float pitch, float yaw_rate, float vertical_g, float force, const HoverDrag *drag,
                  const HoverGps *gps, const HoverBaro *baro, HoverOutput *output) {
    hold(state, g, mode, pose, now_us, dt, roll, pitch, yaw_rate, vertical_g, force, drag, gps, baro, output);
    if (!state->landed && state->lifted < fmaxf(TAKEOFF_EASE_S, TAKEOFF_GPS_S)) {
        // Negative while on the floor, counting up to the wait; zero from liftoff, counting the ease and the GPS's wait.
        state->lifted = state->lifted < 0 && state->height >= LIFTED_HEIGHT ? 0 : state->lifted + dt;
        // The share of the resting attitude still in the targets: all of it on the floor, fading to none over the ease.
        float share = state->lifted < 0 ? 1 : fmaxf(0, 1 - state->lifted / TAKEOFF_EASE_S);
        output->roll += share * (state->rest[0] - output->roll);
        output->pitch += share * (state->rest[1] - output->pitch);
    }
    state->flown_pitch = output->pitch;
}
