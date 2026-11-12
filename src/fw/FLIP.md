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
