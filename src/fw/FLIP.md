# Roll flip firmware (william/flip-ready)

The roll flip from the GPS hold, behind a setting and a one-shot request, on drone 1. It builds clean (`nice -n 19 taskset -c 0-3 make -C src/fw build`) and has only been flown in the simulator. Nothing here has flown.

## What changed

**Gyro at ±2000 deg/s** (runtime.h `GYRO_FS_SEL 3`, 16.4 counts per deg/s; ±250 until now). The flip spins at about 950 deg/s, and with ±250 every simulated flip hit the ground.
- The estimates now take the FIFO's unrounded tick mean. One count is now 0.061 deg/s, more than the resting gyro's noise (0.03-0.07 deg/s RMS), so the rounded mean would have quantized the calibrated bias. Half a count of bias over the 50 s powered time constant is about 1.5 degrees of attitude error in flight.
- The log still records the rounded counts. The metadata carries `gyro_counts_per_deg_s`, which sw/flight_log.py and the dashboard's flight report read; older recordings, which lack it, decode at 131 as before.
- The stored gyro bias is in rad/s and carries over, but recalibrate anyway (step 2).
- FIFO_GYRO_SLACK stays 100 deg/s, now in the new counts.
- The accelerometer stays at ±2 g. The flip's 2.1 g climb clips it for 0.1 s; the sims below include that clipping.

**Settings** (settings.def; every new one defaults to today's behavior):
- `tip_limit`: 20-80 degrees, default 45 as the constant was.
- `bank_tilt`: 0-80 degrees, default 0 = off. Past it the attitude estimate runs on the gyro alone, and in manual flight the collective grows by 1/cos(tilt), at most 2x.
- `flip_enable`: 0 or 1, default 0, which refuses every request.
- `flip_height`: 2.5-5 m, default 3.
- The caps on `hover_climb` and `hover_ceiling` go from 1.5 and 3 m to 3 and 3.5 m. Their defaults are unchanged.
- The settings blob is 1966 of its 2048 bytes, and a static assert now keeps it inside.

**Test pattern limits.** The roll and pitch channels go from 10 to 60 degrees: the drone refused the ladder's 20-35 degree steps. `pattern_tilt` (at most 60) and the tip cut bound what is flown. fw/controller/log_format.py `PATTERN_LIMITS` holds the same limits:
- the bridge refuses a pattern past them before sending it;
- `sweep.run` refuses to fly one;
- test_bridge.py checks the Python limits against flight_rt.c.

**The flip** (flip.c, flip.h, wired in flight_rt.c):
- **Trigger.** Request `f` from the bridge, sent by the dashboard's Flip button. The bridge tries it for 1 s rather than the usual 6, so a late copy cannot start a flip seconds after the click.
- **Once per arming.** At most one flip per arming. A retry that misses the reply cache is refused for that reason.
- **Entry checks** (flip.c `flip_refusal`; the refusal text comes back as the request's error):
  - flip_enable is 1;
  - armed;
  - in the hover hold, not a link-loss landing;
  - on the GPS hold: hover_camera 0, hover_gps_tau above 0, heading pinned, a fix in the last 0.5 s;
  - hover_ekf 0: the Kalman filter's attitude would take the spin's thrust for gravity;
  - no fly-away failsafe and no test pattern playing;
  - 5 s airborne;
  - the hold's height at least flip_height;
  - climb rate under 0.3 m/s and GPS ground speed under 1 m/s;
  - within 10 degrees of level (pitch from pitch_trim), turning under 30 deg/s on every axis;
  - the pack at 14.8 V or more.
- **Sequence.** Climb at 2.1 times the weight for 0.11 s under the attitude controller. Spin on differential thrust. Right the thrust axis. Hand back to the attitude controller within 37 degrees of upright. Arrest the vertical speed, then return to the hold.
- **During the flip:**
  - The attitude estimate runs on the gyro alone, and for 1 s (FLIP_GRACE) after the hand-back.
  - The tip cut is off from the spin until 1 s after the hand-back.
  - The spin keeps a running motor at throttle 1 or more, never stopped.
  - The hover's estimators keep running in HOVER_HOLD, but their outputs are not flown.
- **At the hand-back** the attitude controller gets back the integrators and dead-reckoned velocity it had before the flip. The hover's height integral comes back when the arrest ends.
- **Aborts and timeouts:**
  - While the flip is still climbing, a link loss, a landing request, manual mode or the fly-away failsafe ends it, and the attitude controller carries on.
  - Once the spin has begun, only STOP ends it early, and STOP cuts the motors as always.
  - The flip hands back at most 2 s after it starts, and arrests within 2 s after that.
  - A link-loss landing or a requested landing waits until the flip is done.
- **Logging.** The flight record's guard bits 6-7 and the status flags' bits 14-15 carry the phase: 1 climbing, 2 spinning or righting, 3 arresting. The bridge reports it as `control_status.flip`, and sw/flight_log.py as the `flip` column.
- **Dashboard.** A Flip button sits beside Enable. It is enabled only while armed in hover with flip_enable 1. While a flip runs, the status line under the controls reads `Flip: climbing`, `spinning` or `arresting`; the line keeps its height. The gains dialog gets tip_limit, bank_tilt, flip_enable and flip_height.

**Review fixes beyond the flip:**
- The bank collective no longer applies under the hover. hover.c already divides its thrust by the cosine of the measured tilt (at most 2x), so the two together asked for up to 4 times the thrust.
- bank_tilt's test now measures pitch from pitch_trim.
- landing_test.py's tray clips the gyro at the new ±2000 deg/s.

## Verified, in the simulator

**The flip as wired** (sw/sim/flip_firmware.py, `--n 256`). The setup:
- The plant: drone 1 as of 2026-11-09 in rigid.py, randomized as recover.randomization.
- Settings: 2026-11-11 23:35's, with `flight_steps flip` on top.
- Sensors: the GPS and barometer as sweep.run has them, the gyro at ±2000, the accelerometer at ±2 g, calm air.
- The flight: each drone takes off in the hold, climbs to 3 m, and from 25 s on presses Flip once a second until the drone accepts.

The results:

| band tilt offset | plants that hold without a flip | flips started | clean | touched the ground | tip cut | clean among the plants that hold | height lost median (90th, max) | lowest above the ground median (10th, min) | back within 0.3 m of the start height | carried over the ground median (90th) |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 217 of 256 | 225 | 97.3% | 0 | 0 | 99.5% | 0.71 m (1.14, 1.60) | 2.23 m (1.59, 0.92) | 2.5 s (90th 3.6) | 0.48 m (0.98) |
| -2.5° | 216 | 223 | 95.5% | 0 | 0 | 98.1% | 0.70 m (1.16, 1.64) | 2.23 m (1.66, 0.90) | 2.5 s (3.6) | 0.52 m (0.97) |
| -5° (with 0.025 g on the accelerometer) | 212 | 221 | 95.5% | 0 | 0 | 96.7% | 0.71 m (1.19, 1.71) | 2.21 m (1.57, 0.83) | 2.5 s (3.7) | 0.61 m (1.07) |

- **Refusals.** The flips not started (31-35 of 256) were refused every time, mostly "Not level" or "Not still": these are the plants whose attitude loop does not hold anyway. A few were refused for "Fly-away failsafe active" or an unpinned heading.
- **Not clean.** The runs that were not clean never touched the ground and never hit the tip cut. They ended outside 10 degrees of upright or still turning.
- **The old gyro range.** At ±250 deg/s (`--gyro-range 250`, 32 plants at -2.5°) every flip hit the ground.
- **Compared with the unwired port.** The port flown without the hold lost 1.1 m. Here it loses 0.7 m, probably because the climb starts from the hold's own collective, the arrest starts from the hold's climb rate, and the hold flies the drone back afterwards.

**Nothing else changes with the flip off** (flip_enable 0). These sims match william/cook exactly:
- the tag hold with and without the bands: `band_hold --seeds 8 --seconds 20`;
- the outdoor GPS hold: `outdoor.py --gps --baro 2`, 16 runs at 0 and 2 m/s.

**The gyro range in an ordinary hover.** The same outdoor runs, with the gyro read in whole counts:
- 131 counts per deg/s clipped at ±250 and 16.4 clipped at ±2000 print the same held, lost and failsafe counts;
- the distances and tilts match to within 0.1 m and 0.1 degree.

That sim has no gyro bias, so the bias calibration's resolution is argued above (the unrounded mean), not simulated.

**Roll-step ladder** (tilt_ladder.py, `--bands`, 1 m/s wind, 8 runs, 0.4 s steps at roll_target_rate 360):

| amplitudes | settings | lost | pattern guard stops | true peak roll median (max) at the largest step | height lost max | sideways speed max | furthest from the hold point max |
|---|---|---|---|---|---|---|---|
| 20, 30, 35 | `flight_steps ladder` (1.5 m) | 0 | 0 | 32.5° (36.3) | 0.37 m | 2.6 m/s | 2.0 m |
| 40, 45, 50, 55 | `flight_steps ladder55` (3 m, tip_limit 65, pattern_tilt 58) | 0 | 1 (distance) | 48.8° (54.9) | 0.41 m | 4.4 m/s | 3.5 m |

**Firmware checks:**
- The firmware builds clean.
- test_bridge.py passes and checks the pattern limits against the firmware.
- The dashboard passes tsc and eslint.

## Untested

- **Nothing has flown.**
  - In the simulator, flight_rt.c's tick order and glue are reimplemented in Python around the shared C (flip.c, flight_math.c, hover.c, guard.c): the order, the guards, the collective mapping and the actuator mapping.
  - Not run at all: the radio's `f`, the bridge's flip request and the dashboard button (tsc and eslint pass). The status and record bits are untested end to end.
- **Hardware unknowns:**
  - The MPU6050's noise and bias at ±2000 on this board.
  - How the ESCs behave at throttle 1 (1001 µs) mid-spin.
  - A real pack's thrust at 2.1 times the weight.
  - The bands' spring torque at the servo stops (the sim has a constant tilt offset).
  - The raised pivots and forward legs (the plant is the 2026-11-09 airframe).
  - Everything past 35 degrees in the model is extrapolated.
- **Not simulated:**
  - Wind: rigid.py has none, so fly in calm air only.
  - The Kalman filter during a flip: the flip is refused with hover_ekf above 0.
  - Link loss during a flip: logic only.
  - The barometer's transient from the spin's wash.
- **Open risks seen in flight on 2026-11-12, not addressed here:**
  - The landing's motor cut comes about 2 s late: the height estimate keeps descending past the ground.
  - Takeoffs from a resting tilt over about 5 degrees kick to 20-40 degrees.
  - Keep both in mind for every flight below; land on level ground and take off level.

## Flight checklist

One step per flight, in order. Do not move to the next step until the current one is clean. Settings go through the running bridge with `uv run -m sw.flight_steps <step>` (prints the change) and `--apply` (stores it and checks what came back); store them only while disarmed. Calm air only (under about 1 m/s) from step 3 on. Before every flight: the battery charged (the flip refuses under 14.8 V), a clear area of at least 10 m around, and Space or STOP ready.

### 1. Flash

On the ground, disarmed, with the props off or the drone held down. Run `nice -n 19 taskset -c 0-3 make -C src/fw deploy` from the branch: it builds and sends the image through the running bridge.

Check the drone afterwards:
- It boots the new image, and the dashboard's gains dialog shows Tilt and flip with tip_limit 45, bank_tilt 0, flip_enable 0 and flip_height 3.
- `uv run -m sw.flight_steps noflip` lists flip_enable 0 and tip_limit 45 as already stored.

### 2. Recalibrate on the new gyro range, and check a resting log

No flight. With the drone still and level, press Calibrate on the dashboard and wait until it is saved. Then record about 30 s disarmed and at rest (the dashboard's record, or request O).

The resting log should show:
- the metadata's `gyro_counts_per_deg_s` at 16.4;
- each gyro axis (`gx_deg_s`, `gy_deg_s`, `gz_deg_s` in sw/flight_log.py) with a mean within ±0.1 deg/s of zero and noise about 0.05-0.1 deg/s RMS;
- roll and pitch drifting under 0.2 degrees over the 30 s.

If any of these is off, stop here: the gyro or its calibration is wrong.

### 3. Ordinary hover

The hold as last flown (the tag hold indoors, or `flight_steps kd` outdoors), 30-60 s, then land.

Compare it with the last flights on the old firmware:
- roll and pitch RMS, the pitch swing;
- no new shake in the gyro;
- the attitude estimate at liftoff;
- the landing.

The control loop sees the same rates; only the gyro's resolution changed.

### 4. Roll-step ladder 20/30/35 at 1.5 m

The ladder's settings and pattern:
- Settings: `uv run -m sw.flight_steps ladder --apply` (the GPS hold as cleared on 2026-11-12: hover_tilt 10, hover_kd 1.5, hover_thrust_direction 0.8, pitch_damping 6, hover_climb 1.5, hover_ceiling 3, pattern_tilt 42, pattern_distance 4, roll_target_rate 360, hover_gps_calibrate 5).
- Pattern: `uv run -m sw.sim.tilt_ladder --amplitudes 20,30,35 --width 0.4 --roll-rate 360 --dry > ladder35.json`, then `uv run -m sw.excite file ladder35.json` while disarmed. The old firmware refused this pattern ("A value is past its channel's limit"); this one takes up to 60 degrees.
- The pattern starts 14 s after the collective first reaches integrate_throttle.

In the sim (above): none lost, a true peak of 32.5 degrees (max 36), up to 2 m to the side, at most 0.4 m of height lost per step.

Stop the flight if:
- the pattern guard does not end a step that passes 42 degrees;
- the drone goes more than 4 m to the side;
- the hold does not settle between steps.

### 5. Hold at 3 m, the flip's height

- Settings: `uv run -m sw.flight_steps high --apply` (hover_climb 3, hover_ceiling 3.5 on the ladder's hold).
- Take off, hold for 60 s with no pattern, land.

Check:
- the hold's height against the GPS's and the barometer's in the log;
- the heading pins (flags), with no failsafe;
- how long the hold takes to come back down, and the late motor cut at touchdown.

Expect the climb to 3 m at 0.3 m/s to take about 10 s.

### 6. Roll steps past 45 degrees

- Settings: `uv run -m sw.flight_steps ladder55 --apply` (the 3 m hold, tip_limit 65, pattern_tilt 58).
- Fly it over two flights. First `--amplitudes 40,45`, then on a later flight `--amplitudes 50,55`, each from `uv run -m sw.sim.tilt_ladder --amplitudes ... --width 0.4 --roll-rate 360 --dry > file.json` and `sw.excite file`.

In the sim (40-55): none lost, a true peak of 48.8 degrees median (max 55) at the 55-degree step, up to 4.4 m/s and 3.5 m sideways, at most 0.4 m of height lost. One run in 8 had its pattern stopped by pattern_distance.

Stop if:
- a step's true roll passes 58 degrees;
- the drone loses more than 1 m of height;
- the hold does not bring it back between steps.

Afterwards run `uv run -m sw.flight_steps high --apply` (tip_limit stays at 65 until the next step resets it).

### 7. First flip

**Settings:**
- `uv run -m sw.flight_steps flip --apply`: the 3 m GPS hold of step 5, plus tip_limit 45, flip_enable 1, flip_height 2.5.
- Nothing else changes. hover_ekf must stay 0.
- No test pattern loaded: run `uv run -m sw.excite clear` while disarmed.

**Before arming:**
- Calm air, under 1 m/s.
- The pack fresh (over 15.5 V at rest).
- At least 10 m clear around and nobody under the drone.

**The flight:**
- Take off in the hold. Wait until it has held 3 m for about 10 s with the heading pinned.
- Hands off the sticks, then press Flip once.
- If the drone refuses, the reason shows as the request's error (for example "Not still", "Below flip_height" or "GPS hold not settled"). Wait and press again. One flip per arming.

**What it does:**
- The status line shows Flip: climbing for 0.1 s, then spinning for about 0.5-1 s, then arresting for up to 2 s, then the hold again.
- In the sim it loses about 0.7 m (90th percentile 1.2, worst 1.7), comes back to its height in about 2.5 s, and drifts about 0.5 m (up to 1.1).

**Abort (STOP, Space):**
- It stays upside down or keeps spinning for more than about 2 s.
- It falls below about 1 m.
- After the arrest it does not hold, or it oscillates.
- It flies off sideways faster than the hold can brake.

STOP cuts the motors. From 3 m a cut drone falls, so stop only when the flip has clearly failed. Once spinning, the flip finishes by itself within 2 s whatever the link does.

**Afterwards:**
- Land.
- Run `uv run -m sw.flight_steps noflip --apply`, so flip_enable goes back to 0.
- Look at the recording: the `flip` column (1, 2, 3), the gyro during the spin (it should peak near 900 deg/s, well under 2000), the accelerometer clipping at 2 g, the true height from the GPS and barometer, and the hand-back roll and pitch against the estimate after the grace.

# Tumble recovery and Level (william/recovery)

Two safety features for aggressive maneuvers on drone 1, after the first real flip (2026-11-12 02:27:04 recording) came out of its roll pitching at 380-600 deg/s, tumbled and had to be cut at about 1.3 m. They build clean and have only been flown in the simulator. Nothing here has flown.

## What it does

- **Tumble recovery** (`recover_enable`, default 0 = the tip cut as before). In flight, instead of stopping the motors, a recovery starts when:
  - the attitude passes `tip_limit` outside a flip (where the tip cut fired);
  - after a flip's hand-back (its arrest and the 1 s grace, when the tip cut is off for it) the thrust axis passes `tip_limit`;
  - the gyro reads 1900 deg/s or more on any axis (its full scale is 2000).
- **Righting** owns the motors: it flies its own quaternion on the gyro alone (the Euler estimate breaks down near 90 degrees of pitch), turns the thrust axis toward up on differential thrust (roll) and the servos' common tilt (pitch), rolling the long way round when far over and mostly pitched, with each motor kept at 14% of the weight or more (a motor commanded to throttle 1 mid-spin on 2026-11-12 took 0.1 s or more to come back).
- **Leveling**, once within 58 degrees of upright and turning under 115 deg/s: the attitude controller at level, its integrators as before the event, the collective stopping the vertical speed. Once within 10 degrees, still and under 0.3 m/s vertically for 0.3 s (or after 2 s) it hands to the hover's hold. The estimate stays on the gyro for 1 s more.
- **Level** (dashboard button between Flip and the STOP square; request `h`): from any mode, with or without `recover_enable`, ends a test pattern or a flip and starts the same recovery (straight to leveling when already near upright), then the hold. It needs nothing more from the laptop once taken: a lost link afterwards is the hold's, waited out 2 s then landed. STOP stays the motor cut.
- **After either**, the hover holds (GPS or tag hold, whatever is set) whatever mode the laptop streams until the pilot picks another mode (L lands, manual takes the stick back); a hold engaged from manual flight climbs to `hover_climb` as pressing H in the air does. The laptop's tilt is dropped only while the recovery and its grace run. The dashboard's mode label shows the mode it sends, not this hold.
- **The tip cut still applies:**
  - with `recover_enable` 0;
  - before the attitude controller has been airborne 1 s, or in a hover takeoff before liftoff: a takeoff that tips over still has its feet down, and driving the motors there rolls it along the ground;
  - during a landing (the pilot's, the failsafe's or the link-loss one) below 1 m by the hold's height (the barometer may read 0.9 m high at contact), and once the landing's touchdown probe has started or it has touched down;
  - after 3 recoveries in one arming;
  - and a righting that has not come within 58 degrees of upright in 2.5 s stops the motors with the new stop reason 10, Recovery failed.
- **Logging.** The flight record's guard bits 8-9 carry the stage (1 righting, 2 leveling) and bits 10-12 the cause (1 tip, 2 flip, 3 gyro, 4 Level); sw/flight_log.py gives them as the `recovery` and `recovery_cause` columns. The status carries them in its reason byte's top bits; the bridge reports `control_status.recovery` and `recovery_cause`, and the dashboard's status line reads `Recovering (tipped): righting` and the like without moving anything. A test pattern a recovery stops records pattern stop 6, recovery.
- **Settings.** One new setting, `recover_enable` (0 or 1), in the gains dialog under Tilt and flip. The righting's gains are constants in recover.c, tuned in the simulator (below).

## Verified, in the simulator

sw/sim/recover_firmware.py flies recover.c, flip.c, flight_math.c, hover.c and guard.c in sw/sim/flip_firmware.py's harness, ticked as flight_rt.c ticks them, on drone 1's plant (2026-11-09) randomized as recover.randomization, from the 2026-11-11 23:35 settings with the GPS hold. At 14 s after liftoff each holding drone is put into a drawn attitude and rates (the estimate with it), 256 runs per row. Recovered means never cut, never touched the ground, and upright, still and above 0.5 m 6 s later; "of holding plants" leaves out the 20% of drawn plants whose attitude loop does not hold even without the event.

**Wide plant** (`--wide`): the uncertain terms widened as the flip failure suggests: roll authority 0.4-5 times the fit, the props' gyroscopic coupling (missing from rigid.py) up to ±0.03 N·m·s/√N, an ESC deadband up to throttle 60 that stops a motor (0.16 s restart), motor lag 1.5 times.

| event | hold | recovered, nominal plant (of holding plants) | recovered, wide plant (of holding) | touched the ground, nominal / wide | height lost median (90th), nominal | hand-back to the hold median |
|---|---|---|---|---|---|---|
| tip: 45-75° over, ~100 deg/s | 2 m | 76% (88%) | 77% (83%) | 5% / 4% | 0.16 m (1.0) | 1.8 s |
| | 3 m | 77% (90%) | 78% (84%) | 3% / 3% | 0.17 m (1.2) | 1.8 s |
| tilt: 60-150° over, at rest | 2 m | 65% (76%) | 64% (70%) | 10% / 11% | 0.60 m (1.7) | 1.9 s |
| | 3 m | 67% (78%) | 69% (75%) | 5% / 5% | 0.59 m (1.7) | 1.9 s |
| tumble: 90-180° over, pitching 400-600 deg/s, sinking to 2 m/s | 2 m | 25% (29%) | 15% (16%) | 62% / 71% | 1.85 m (2.2) | 2.2 s |
| | 3 m | 39% (46%) | 26% (28%) | 42% / 55% | 2.47 m (3.2) | 2.1 s |
| Level with recover_enable 0: 20-60° over, up to 100 deg/s | 2 m | 79% (91%) | 85% (91%) | 3% / 1% | 0.16 m (0.85) | 1.6 s |
| | 3 m | 79% (92%) | 84% (90%) | 5% / 1% | 0.16 m (0.87) | 1.7 s |

- **Today's firmware** cuts every one of these: 100% fall from 2-3 m.
- **The flip's righting gains**, which the recovery started from (`--params` with flip.c's), recovered 28-31% of the tip events, 7-10% of the tilts and none of the tumbles: their pitch gain was a fifth of the tuned one, and the servos barely moved against a 584 deg/s pitch rate.
- **Link-loss landings** (`landing`: 60-150° over at 0.5-2.5 m during a 1 m/s descent): above 1 m 61-68% recovered (nominal), 61-72% (wide), then landed again after the hold's 2 s; below 1 m 55 of 63 and 56 of 64 were cut as designed (the rest had the hold's height reading 1 m or more).
- **The flip.** On the nominal plant, recovery on or off gives the same 213 flips, 95.3% clean, and no recovery started in any. On the wide plant the flip fails as it did on the drone: 39.7% clean with recovery off (47.7% tip cut, 59.8% touched the ground, 43.9% holding at the end); with it on 39.3% clean, 57.5% touched, 48.6% holding at the end. The recovery does not rescue most failed flips at 3 m: they fall too far before it can act.
- **Nothing else changes with recover_enable 0.** With the setting off and no Level, recover_tick only tracks its quaternion and returns the tip cut exactly where flight_rt.c tested it. The flip's runs (flip_firmware, 32 plants with and without the flip) are bit-identical to william/cook. hover.c, guard.c, flight_math.c, ekf.c, flip.c and every other sim's code are unchanged (guard.h gains an enum value, flight_math.h's Command a last field), so the hover, tag hold, landing and outdoor sims are the same programs.
- **Replayed on 2026-11-12T02-27-04** (sw/sim/recover_replay.py, open loop): the recovery would have started 0.15 s after the flip's hand-back, at 2 m by the hold, sinking 3.6 m/s, 48 degrees over by its quaternion and pitching nose-down at 584 deg/s, with the servos at their 51-degree limit tilting back and the collective at its limit of twice the weight. Whether that stops the pitch in time is not known: the simulator mispredicted this flip. The replay also shows what drove the tumble: at 11.19 s the Euler estimate went from roll 58 to 163 at 90 degrees of pitch, and from there wrote the attitude the other way round (roll 136, pitch 159 is roll -44, pitch 21), which the attitude controller flew as 140 degrees of error.

## What real data tests each assumption, and what does not

- **The gyro carries the attitude through a tumble** (no accelerometer). Tested only up to 600 deg/s, in the replay: the quaternion and the Euler estimate (both on the gyro then) agree to a degree until the Euler one's singularity. Past the gyro's 2000 deg/s nothing can: the attitude is then unknown, and in the air the accelerometer cannot tell (it reads the thrust, not gravity, whatever the attitude). After a saturation trigger the recovery flies on an attitude that may be wrong by tens of degrees.
- **Roll authority, pitch authority and their lags at large tilts and rates.** Fitted only within 35 degrees of level and at hover rates. The flip's spin reached 540 deg/s on the drone against the simulator's 950, so the roll authority at low thrust is likely at the low end of the wide range; the tuned gains hold up there (wide plant column) except in tumbles.
- **The props' gyroscopic coupling.** Not in the identified model at all; its size (±0.03) is a physical estimate. During the real flip the pitch rate grew to -160 deg/s while rolling at 340 deg/s with the motors at 464 and 34 throttle, consistent with it, but not a measurement of it.
- **ESC behavior near throttle 1 and restarts.** Not measured; the recovery keeps every motor at 14% of the weight to stay out of it.
- **The ground.** The simulator's contact is rough, and a recovery that meets the ground drives its motors into it until the 2.5 s timeout or the end of the righting. A drone tumbling from under about 2 m mostly hits the ground first either way.

## Enabling and testing it

Flash `william/recovery` as in step 1 above. `recover_enable` stays 0 until set; the Level button works without it.

### A. On the ground, props off

1. Props off, drone in hand or on a stand. `uv run -m sw.flight_steps recover --apply` (tip_limit 30, recover_enable 1).
2. Arm, stay in manual, raise the collective past integrate_throttle (580) and hold it there more than 1 s (the recovery waits for 1 s airborne).
3. Tilt the drone by hand past 30 degrees in roll: the status line shows `Recovering (tipped): righting`, the low side's motor speeds up and the high side's slows (never stopping). Tilt it nose-down past 30: both servos tilt back (nose-up), with both motors up. Hold it tilted: after 2.5 s the motors stop with "Recovery failed".
4. Arm again and repeat, bringing it back level within a second: `leveling`, then the drone's hold (the recording's hover mode, flags bits 1-2, goes to 1; the dashboard's mode label still shows what it sends). STOP.
5. Arm, collective up past 580 for 1 s, press Level: `Recovering (level): leveling`, then the hold. STOP.
6. In the recording: the `recovery` (1, 2) and `recovery_cause` (1, 4) columns, the throttles and servos against the gyro's sign, stop reason 10 on the held tilt.
7. `uv run -m sw.flight_steps norecover --apply` before flying anything else.

### B. In flight, from a modest tilt

Calm air, the 3 m GPS hold as cleared in steps 4-5 above, 10 m clear, STOP ready.

1. **Level in a plain hold.** Settings `high`, recover_enable 0. Hold 3 m, press Level once: leveling for a moment, then the hold, as before. Then press L to land: the hold gives way to the landing.
2. **Level during a roll step.** Settings `ladder` at 3 m (`high`), a 30-degree step (`uv run -m sw.sim.tilt_ladder --amplitudes 30 --width 0.4 --roll-rate 360 --dry`). Press Level as the step starts: the pattern stops (pattern stop 6) and the drone levels and holds. In the sim, 20-60-degree Levels hand back to the hold in 1.6 s median and lose 0.16 m median.
3. **Recovery from a 35-degree step.** `uv run -m sw.flight_steps recover --apply` (tip_limit 30, recover_enable 1) and a 35-degree step. It trips the recovery at 30 degrees: `Recovering (tipped)`, then the hold. Afterwards `norecover`.
4. Only then consider larger tilts, and fly the flip with recover_enable 1 only after the flip itself has been refitted (william/flip-fit): on the plant the real flip suggests, recovery saved few failed flips in the sim.

Abort a test with STOP if the righting does not turn the drone toward upright within half a second, or it falls below about 1 m.
