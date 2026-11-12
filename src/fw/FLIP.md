# Roll flip firmware (william/flip-ready)

The roll flip from the GPS hold, behind a setting and a one-shot request, on drone 1. It builds clean (`nice -n 19 taskset -c 0-3 make -C src/fw build`) and has only been flown in the simulator. Nothing here has flown.

## After the first flip (2026-11-12 02:27 UTC, william/flip-fit)

**Do not fly the full flip on this firmware until stages A-D below have passed.** The first flip (step 7 below, as flown) failed: the spin rolled at 550 deg/s instead of the sim's 950 while the pitch swung to -36 degrees, the righting began with the pitch rate at +380 deg/s and rising, the drone went over the top in pitch and the pilot stopped it at 1.3 m; it crashed and the left ESC burnt at an internal joint. src/sw/sim/models/NOTES.md "The first flip, replayed" has the replay and the fit; this section has what the sim assumes, what changed in flip.c, the retuned flip's sim results and the staged tests.

### What the sim assumed, and what real data tests

The sim was checked against the same day's roll-step ladders to 55 degrees and matched them, and predicted the flip 95-97% clean. The ladders keep both motors near hover (L 450-980, R 450-870) at under 230 deg/s, so they test only near-hover terms; the flip cut one motor to idle and spun at 550 deg/s. Every term the flip leans on, with what real data tests it:

**Motors and ESCs**

| assumption | in the flip | tested by real data | status |
|---|---|---|---|
| Thrust against throttle: the 2026-10-04 bench table (static, 9-inch prop, rescaled by guess after the ESC recalibration) times each flight's hover trim | climb 960-1000, spin L 380-650 and R 1-100, righting and arrest 750-1200 | hovers pin it at 540-680, the PRBS hovers its slope there | low end (under 300) and top (over 1000) untested; full throttle against the 30 A ESC never measured |
| Differential thrust gives the table's difference (differential_slope 1) | one motor near idle | ±1.5-3° PRBS hovers and the ladders, both motors near hover | the flip's roll was 2x short of the model's; unresolved between a coasting cut motor, a regime-dependent loss and a weak left ESC (NOTES) |
| Thrust rises with 0.01 s dead time and a 0.066 s lag | climb, righting | fitted from hover attitude data, never on the bench | the flip's step sizes untested |
| A falling thrust follows the same lag | the spin's cut to throttle 1 | nothing (no hover or ladder cuts a motor) | the flip says a cut motor coasts, halving from 5 N in about 0.21 s; one flight |
| What the ESC does at throttle 1 (1001 µs): idles or stops | the spin's slow motor | nothing | untested |
| The pack holds its voltage | climb at 2.1 g, arrest at 1200 | the log sagged to 14.0 V in the arrest | rigid.py holds it constant |

**Rotors and airframe**

| assumption | in the flip | tested by real data | status |
|---|---|---|---|
| The rotors' spin angular momentum acts only through the tilt rate (prop_momentum) | spinning at 5-10 rad/s with one rotor fast and one slow, -ω × H pitches the drone | the hover fit's prop_momentum 0.0154 (the same H, seen through the tilt rate) | missing from the sim until now; the flip's pitch swing and its reversal at the righting are its signature; value 0.003-0.05 from the replay, about 0.02 from physics; the old flip's outcome hangs on it |
| Roll damping (inflow_damping 0.046) | 10 rad/s | hover fit, under 1 rad/s | flip fits 0.03-0.3 depending on structure |
| Yaw from roll rate (yaw_from_roll -0.023, a lumped hover term) | 0.2 N·m of yaw at the flip's roll rate | hover fit | the flip contradicts it (+0.002 to +0.009); the righting's yaw is still not reproduced |
| Prop reaction torque (prop_torque -0.0044) | one rotor carries the whole reaction | hover fit | untested in the flip's regime |
| Pitch authority of the tilt (tilt arm, wash, pitch_wake) at large tilts and on one motor | the spin's and righting's pitch control | hovers within ±8° of servo | untested past 10°; the pivots raised 2 cm on 2026-11-11 are not in the model (the CAD-estimated geometry with them does not hold a hover in the sim) |
| The hover's attitude and speed moments fade to nothing past 35° | the whole flip | hovers within 35° | assumption |
| Rotors in non-axial flow, descending through their own wake, inverted | the second half of the spin, the arrest | nothing | not modelled |
| Inertias (pitch 2x CAD from the hover fits) | the gyroscopic coupling goes as H over the inertia | hover attitude fits | the raised pivots changed them; not measured |
| The bands as a constant 2.5° tilt offset | servos at large angles | the accelerometer in band hovers | the spring at large tilt untested |
| Servo dynamics (slew 285 deg/s) | commands of ±25-55° | hover-sized commands | untested at the flip's amplitudes |

**Sensors and estimator**

| assumption | in the flip | tested by real data | status |
|---|---|---|---|
| Gyro at ±2000 deg/s, scale within a few % | the turn is counted from the gyro; the arrest clipped at -2000 | resting logs only | scale at high rates untested; 3% is 11° over a turn |
| Accelerometer at ±2 g | the climb clips; the arrest's vertical speed is integrated from it | nothing past 2 g | the arrest's vertical speed is wrong while it clips |
| The IMU sits at the CoM | at 10 rad/s an IMU 2 cm off the roll axis reads 0.2 g of centripetal | nothing | not modelled |
| The barometer gives the height through the flip | the hold's height after it | 02:27: the barometer read a 1 m climb while the drone fell 1.2 m | wrong during the flip |
| The gyro-only attitude through the flip and 1 s after | the hand-back's roll and pitch | 02:27: consistent with itself, no truth | untested |

**Timing**: the 100 Hz loop, the motors' 0.01 s and the servos' 0.04 s dead times and the gyro as the tick's mean are the same in the sim and tested at hover rates only.

**flip.c's own assumptions** (now changed, below): its authority comes from the firmware's model (pivot_height 0.02 m, pitch_inertia 0.0023, the numbers from before the pivots were raised); it has no model of the rotors' momentum; it handed back within 37° of upright whatever the rates (at 02:27 with the pitch rate at +380 deg/s); it sent the slow motor to throttle 1; its throttle steps were unlimited (right motor 1 → 853 → 1200 in 0.1 s at the righting).

### What changed in flip.c

- **Motor limits** (FlipParams floor, ceiling, slew, throttle_cap; the new flip_limit, which flight_rt.c calls every tick on the throttles about to be sent):
  - While the flip owns the motors each makes at least 0.2 of the weight (about 2.1 N, throttle 230 at the 1.3 thrust scale): above the throttle-150 region where the first flip's cut motor coasted, so the ESC keeps driving it, and both rotors keep spinning so their momenta partly cancel. Nothing goes to throttle 1 or 0 except on STOP.
  - From the flip's start until 1 s after its hand-back (the arrest included, where the attitude controller flies the motors) each throttle moves at most 10000 units a second, the full range in no less than 0.12 s (about twice the fitted 0.066 s motor lag), and stays under 1100 (92%), so no motor sits at 95% or more. A stopped motor stays stopped.
- **Hand-back** only once every body rate is under handoff_rate (at most 3 rad/s in the search) as well as within right_angle (at most 0.6 rad) of upright, or at the 2 s timeout. At 02:27 it handed back at 37° with the pitch rate at +380 deg/s; the attitude controller works in Euler angles, and at 11.19 s, pitched through 90°, the estimate jumped from roll 58°, pitch 87° to roll 163°, pitch 93° (the same attitude): the controller flew that as about 140° of error, put the servos to +55/-50 and both motors to 1200, and the pitch rate went from +100 to -1200 deg/s in 0.15 s. The flip's own righting works on its quaternion; the hand-back limits keep the Euler controller to a drone nearly level and slow. The tilt after the hand-back is reported by the sims below.
- **The arrest's collective** is at least 0.6 of the weight (ARREST_FLOOR; 0.15 before): the attitude controller flies the arrest and needs the thrust. A flip or burst that had climbed cut it to 0.15 and tipped over in the sim.
- **spin_lift**: the spin's collective may follow the cosine of the turn (the final tuning left it near 0).
- **Staged bursts**: flip_enable 2 and 3 fly a burst of the spin aimed at 60 or 90 degrees and brake it at once, back the short way (flip_turn); the spin aims at spin_rate times the square root of the turn's share of a full one. Same entry checks, once per arming. Whether they are less risky than the flip is in the sim results below.
- **The parameters** (flip_defaults, models/flip_firmware_params_2026-11-12.json; radians and fractions of the weight) were retuned by sw/sim/tune_flip.py on the fitted plant and its uncertainty (below), with the rate loops' gains bounded where the motors' lag and the slew make them chatter.
- **recover.c** (william/recovery) keeps its own quaternion and repeats about 40 lines of this righting (the thrust axis turned toward up about the axis between them, no faster than it can brake from, and the allocation into thrusts and tilts). Once both branches are in, the righting and allocate() should be one function both call, with the floor, ceiling and flip_limit's slew applying to both.
- Unchanged: the API the tumble-recovery work calls (flip_abort, flip_active, flip_gyro_only, flip_tip_exempt), the climb, the gyro-integrated attitude, the arrest's logic, the entry checks.
- settings.def: hover_climb and hover_ceiling may go to 5 and 5.5 m; flip_enable takes 0-3.

### The retuned flip in the sim

sw/sim/tune_flip.py (CMA-ES, 16 candidates a generation on 64 fresh plants each, a quarter of them the staged bursts; drone-v0) on the fitted plant (models/airframe_2026-11-12_flip.json) with every term the flip leans on that nothing real tests drawn over its whole plausible range per batch of 4 runs (sw/sim/flip_firmware.py `uncertain`): rotor_momentum 0.006-0.04, motor_coast 0.05-0.3 s, inflow_damping 0.03-0.3, yaw_from_roll -0.025 to +0.01, prop_torque ±0.015, the motors' lag 0.04-0.1 s, the table's top at 70-100%, the ESC stopping at throttle 1 or not, the differential thrust's share far from hover 0.45-1.1, the tilt's pitch authority 0.7-1.4, inertia 0.8-1.3, drag, disturbances, the gyro's scale ±3% and the bands' offset 0 to -5 degrees. It took four searches: left free, the search climbed metres first or turned slowly for seconds (now costed), handed back fast at 57 degrees for the Euler controller to finish (now bounded), and drove the rate loops to chatter with the throttles swinging 200 ↔ 1100 every 0.1 s (gains now bounded). The final parameters are flip.c's flip_defaults and models/flip_firmware_params_2026-11-12.json: a 0.35 s climb at 1.98 of the weight, a spin at up to 11.4 rad/s (650 deg/s) on k_spin 12 with 0.4 of the weight of collective (each motor at least 0.2), handoff with 114 degrees left, a gentler righting (kp 2.9, brake 9.1, max_rate 6.7 rad/s, k_roll 7.3), hand-back within 34 degrees of upright and under 3 rad/s.

From a 5 m hold (flip_height 4.5; the tip cut at 45; the 2026-11-12 02:27 settings), the same 512 drawn plants (seed 90000) for every row; height lost is from the flip's start to its lowest in 4 s; "time past 90" is with the thrust pointing below the horizon; the current is modeled (I = 31 A (T / 14.4 N)^1.5, pinned at SunnySky's X2212 980KV table on a 10x4.7 at 14.8 V; not measured on this drone):

| | clean | touched the ground | tip cut | height lost median (90th, worst) | past 90° median (90th) | tilt after the hand-back median (90th) | peak rates roll/pitch/yaw median (deg/s) | at ≥95% throttle | current peak median (max), mean |
|---|---|---|---|---|---|---|---|---|---|
| the flip as flown on 2026-11-12 (old flip_defaults, no motor limits) | 47.0% | 52.6% | 30.2% | 4.68 m (5.18, 5.61) | 0.38 s (2.78) | 70° (180) | 745/564/574 | 0.13 s median, 2.90 max | 23 A (31), 10.1 A |
| the same with the motor limits only (floor 0.2, slew, cap 1100) | 33.9% | 66.1% | 32.5% | 4.85 m (5.23, 5.70) | 0.54 s (2.70) | 49° (179) | 749/699/525 | 0 | 22 A (30), 10.7 A |
| **retuned** | **92.5%** | **7.1%** | 3.2% | **0.00 m (3.74, 5.58)** | 0.32 s (0.53) | 33° (49) | 609/224/302 | **0** | 20 A (29), 10.8 A |
| retuned, without the slew limit | 94.4% | 5.4% | 2.8% | 0.00 m (2.78, 5.58) | 0.31 s (0.49) | 33° (45) | 597/231/307 | 0 | 20 A (29), 10.8 A |
| retuned, the left motor at 80% of the right's (a weak left ESC), 256 | 77.7% | 19.2% | 10.9% | 0.01 m (4.92, 5.37) | 0.36 s (0.86) | 39° (131) | 572/199/235 | 0 | 22 A (28), 11.1 A |
| retuned, from 3 m instead of 5 | 87.4% | 11.8% | 5.1% | 0.00 m (2.79, 3.60) | 0.32 s (0.53) | 33° (51) | 615/224/304 | 0 | 20 A (30), 10.8 A |
| retuned, the fitted plant as saved (64, sensor noise only) | 100% | 0 | 0 | 0.00 m (0.02, 0.21) | 0.26 s | 34° (38) | 691/272/258 | 0 | 21 A (22), 9.9 A |
| retuned, airframe_2026-11-09 as saved (64) | 100% | 0 | 0 | 0.00 m (0.02, 0.22) | 0.26 s | 35° (39) | 709/177/231 | 0 | 22 A (25), 10.2 A |
| retuned, recover.randomization about the fitted plant (the old, narrower spread; 167 of 256 plants hold well enough to start) | 97.6% | 0 | 0 | 0.00 m (0.04, 0.26) | 0.26 s | 28° (38) | 754/327/307 | 0 | 22 A (30), 10.0 A |

- **It climbs first**: 1.6-1.8 m median above the start, then loses little below it; the hold brings it back down. Hand-back 1.1-1.5 s after the start (90th 2.0, the timeout).
- **The slew limit** (the full throttle range in no less than 0.12 s) costs about 2 points of clean flips and 1.7 of ground hits; the cap at 1100 keeps every motor under 95% throttle (the old flip ran one there for 0.13 s median, 2.9 s worst). The throttles travel about 5200 units a flip, against 11000 before the gains were bounded.
- **Current**: 20 A peak median (29 max) and 10.8 A mean through the flip, against the motor's 26 A for 30 s and 385 W, the old ESC's 30 A and the new 40 A (55 A burst). A 1-2 s flip is far from the motor's thermal limit; the peaks are at the climb and the righting. The current model is unmeasured (stage A2).
- **What decides it** (sw/sim/flip_sensitivity.py, 508 runs): the rotors' momentum. With rotor_momentum up to 0.022 the retuned flip is 99% clean with at most 1% on the ground; from 0.022 to 0.04, 71% and 28%. Then the roll damping (inflow_damping over 0.146: 78% clean, 22% ground). No other drawn term moves it by more than about 8 points. The old flip with the limits was 56% clean at the low third of rotor_momentum and 2% at the top. A weak left ESC (80%) costs 15 points.

**The staged bursts against the flip** (the same retuned parameters; a burst is the flip's climb and spin, the spin cut short at about two thirds of 60 or 90 degrees (42° or 63°) and braked at once by the righting, back the short way):

| | clean | touched | height lost worst | past 90° median (90th) | peak roll / pitch / yaw rate median (90th) deg/s | climbed median |
|---|---|---|---|---|---|---|
| 60° burst (flip_enable 2), 256 drawn plants | 100% | 0 | 0.01 m | 0 (0) | 232 / 110 / 70 (276 / 193 / 154) | 2.8 m |
| 60° burst, the fitted plant (32) | 100% | 0 | 0.01 m | 0 | 287 / 133 / 71 | 3.4 m |
| 90° burst (flip_enable 3), 256 drawn | 100% | 0 | 3.13 m | 0.20 s (0.32) | 284 / 124 / 76 (340 / 240 / 199) | 1.8 m |
| 90° burst, the fitted plant (32) | 100% | 0 | 0.01 m | 0.16 s | 352 / 143 / 77 | 1.7 m |
| the flip, 512 drawn | 92.5% | 7.1% | 5.58 m | 0.32 s (0.53) | 609 / 224 / 302 (801 / 712 / 656) | 1.8 m |

The 60° burst never passes 90 degrees and has never touched the ground in the drawn plants; the 90° burst passes it for 0.2 s. Both are lower risk than the flip in this sim. Rolls to 90 or 180 degrees and back at flip rates are not: an earlier tuning (aggressive righting, hand-back at 57 degrees) flew a 90-degree roll at 53% clean on the fitted plant itself, worse than its own flip, as the user expected: braking mid-turn on its side and coming back needs two accelerations where the flip needs one each way. The bursts here are slower (230-350 deg/s against the flip's 600-800) and spend almost no time past 90 degrees. They still exercise every term the flip's failure turned on: the one-sided thrust split with the slow motor at its floor, the rotors' momentum (the pitch swing grows with the angle turned, so a burst shows it at a sixth to a quarter of the flip's size, with the gyro's 2000 deg/s and 0.06 deg/s resolution well able to see 100-150 deg/s), the motors' fall and rise, the brake and the hand-back.

**Not reproduced and not modelled**: the real arrest's final tumble (both motors at 1200, the servos at ±55, the gyro clipping); the rotors in their own wake while inverted or descending; the IMU's lever arm; the barometer through the flip; a failing ESC beyond a constant 80%.

### Staged tests before another flip

Each stage has a prediction from the sim and pass/fail numbers; do not go on until a stage passes. Calm air (under 1 m/s), a fresh pack, 10 m clear, Space or STOP ready. After each flying stage, replay its recording with `uv run -m sw.sim.flip_replay --recording <name> --start <t> --end <t>` (on airframe_2026-11-12_flip.json) and refit with `flip_replay.fit`: the stage passes only if the replay matches with the model's terms in the ranges the flip is robust over.

**A. Bench (props on, the drone clamped to the load cells and HX711, outdoors or with the props well clear; the new ESCs fitted).** No flight. Measure each motor alone, then both:
- **A1, the motors' steps.** From hover throttle (about 600): up to 1000 and back, down to the flip's floor (throttle 230) and back, and down to throttle 1 and back up. Prediction: thrust rises with about 0.01 s dead time and a 0.066 s lag, and falls the same when driven to 230; cut to 1 it coasts, halving from about 5 N in about 0.21 s, and whether it stops or idles at 1 is unknown (the sim draws both). Pass: rise and driven-fall lags 0.04-0.10 s (the drawn range), a cut motor's half-time 0.05-0.3 s. The HX711 at 80 Hz resolves this; average several steps.
- **A2, the table's top and the current.** 1100 (the flip's cap) for 1 s on each motor with an inline current meter. Prediction: about 13 N (the table's 10.0 N at 1100 times the 1.3 thrust scale) and about 25 A at 16 V. Pass: at least 70% of the predicted thrust (the drawn range's floor) and the current under the ESC's continuous rating. Refit the thrust table from a hover afterwards as planned for the new ESCs.
- **A3, the rotors' momentum, the term the flip hangs on.** The rotor speed at hover thrust (5 N) from the blade-pass frequency in a phone recording next to the prop (or an optical tachometer), and the prop's inertia from a bifilar swing (or its mass and length: a 15 g 10-inch prop is about 7e-5 kg·m²; add the motor bell's). H/√T = I Ω / √5. Prediction 0.018 (physics about 0.02, 630 rad/s, 6000 rpm, blade pass about 200 Hz). Pass: under 0.022, where the retuned flip is 99% clean in the sim. Over it, retune on the measured value before stage D2.
- **A4, left against right.** Each motor's thrust at hover throttle (the model has the right 11% stronger), and both with the props and motors swapped side to side if the split is unclear: a gain moves with the motor, a sideways CoM offset stays.

**B. Hover at 5 m on the new ESCs** (`uv run -m sw.flight_steps high5 --apply`, about 60 s). Pass: the hold settles within 0.3 m of 5 m, the L-R throttle split within ±30 of the last good flights' after the new ESCs' trim, no failsafe, a normal landing.

**D1. 60° burst at 5 m** (`flight_steps burst60 --apply`, then Flip once). Prediction on the fitted plant: climbs about 3.4 m above the start (the hold comes back down), rolls at most 290 deg/s, turning about 42° in the spin, pitch rate at most about 130 deg/s, yaw rate about 70 deg/s, back upright and handed back 1.3 s after the start, never past 90°; over the drawn plants 100% clean, none on the ground, roll rate 232 deg/s median (90th 276), pitch 110 (90th 193). Pass: roll rate peak 180-350 deg/s, pitch and yaw rate peaks under 250, the tilt under 75°, upright and handed back by 2 s, no motor at 1100 for more than 0.3 s, and the replay's fitted rotor_momentum under 0.022. Abort (STOP) only if it is past 90° and still turning away after 1 s, or below 2 m.

**D2. 90° burst at 5 m** (`flight_steps burst90 --apply`). Prediction on the fitted plant: roll at most 350 deg/s, about 0.16 s past 90°, pitch rate at most 145 deg/s, handed back 1.3 s after the start; over the drawn plants 100% clean, none on the ground, past 90° for 0.20 s median (0.32 at the 90th). Pass: roll rate peak 250-450 deg/s, pitch under 300, past 90° under 0.4 s, upright and handed back by 2 s, and the replay again under 0.022 for rotor_momentum, with the cut motor's fall and the roll damping within the drawn ranges.

**E. The flip at 5 m** (`flight_steps flip --apply`), only after A3, D1 and D2 pass and their replays agree. Prediction on the fitted plant: climbs about 1.6 m, spins at up to 690 deg/s, about 0.26 s past 90°, pitch rate at most about 270 deg/s, handed back 1.1 s after the start within 34° of upright, loses no height below the start; over the drawn plants 92.5% clean, 7.1% on the ground (most where rotor_momentum is over 0.022, which A3 and D are there to rule out). Pass: roll peak 550-800 deg/s, pitch under 400, upright and still within 2.5 s, lowest point above 3 m. Abort as step 7 had it, and do not stop it in the spin: from 5 m a cut drone falls.

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

### 7. First flip (flown 2026-11-12 02:27 UTC and failed; superseded by "After the first flip" above)

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
