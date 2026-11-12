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
- **One righting for the flip and the recovery** (william/merge-sim): flip.c's flip_righting and flip_allocate, with the gains and motor limits in a FlipRighting, and the quaternion helpers (flip_attitude, flip_rotate, flip_up, flip_euler); recover.c calls them with its own gains, the flip's floor (FLIP_FLOOR, 0.2 of the weight) and no ceiling but the throttle cap, and flight_rt.c slews and caps the recovery's throttles as the flip's (flip_limit_with) while a recovery and its grace run. Details and the trade-off are under "Merged with the retuned flip" below.
- **The attitude estimate through the flip** is the flip's own quaternion's roll and pitch every tick while the estimate is on the gyro alone (the climb to 1 s after the hand-back, the arrest included), no longer Euler angles carried on the gyro, which jumped at 90 degrees of pitch at 02:27.
- Unchanged: the API the tumble-recovery work calls (flip_abort, flip_active, flip_gyro_only, flip_tip_exempt, flip_cancel, flip_actuators, flip_limit), the climb, the arrest's logic, the entry checks.
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
  - The attitude estimate runs on the gyro alone, and for 1 s (FLIP_GRACE) after the hand-back; since william/merge-sim it is the flip's quaternion's roll and pitch throughout.
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

# Tumble recovery and Level (william/recovery)

Two safety features for aggressive maneuvers on drone 1, after the first real flip (2026-11-12 02:27:04 recording) came out of its roll pitching at 380-600 deg/s, tumbled and had to be cut at about 1.3 m. They build clean and have only been flown in the simulator. Nothing here has flown.

## What it does

- **Tumble recovery** (`recover_enable`, default 0 = the tip cut as before). In flight, instead of stopping the motors, a recovery starts when:
  - the attitude passes `tip_limit` outside a flip (where the tip cut fired);
  - after a flip's hand-back (its arrest and the 1 s grace, when the tip cut is off for it) the thrust axis passes `tip_limit`;
  - the gyro reads 1900 deg/s or more on any axis (its full scale is 2000).
- **Righting** owns the motors: it flies its own quaternion on the gyro alone (the Euler estimate breaks down near 90 degrees of pitch), turns the thrust axis toward up on differential thrust (roll) and the servos' common tilt (pitch), rolling the long way round when far over and mostly pitched, with each motor kept at 20% of the weight or more, the flip's floor (a motor commanded to throttle 1 mid-spin on 2026-11-12 took 0.1 s or more to come back), and the throttles slewed and capped as the flip's (the full range in no less than 0.12 s, at most 1100).
- **Leveling**, once within 58 degrees of upright and turning under 115 deg/s: the attitude controller at level, on the righting's quaternion's roll and pitch (within 90 degrees, so unambiguous), its integrators as before the event, the collective stopping the vertical speed. Once within 10 degrees, still and under 0.3 m/s vertically for 0.3 s (or after 2 s) it hands to the hover's hold. The estimate stays on the gyro for 1 s more.
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

As tuned on william/recovery, before the merge ("Merged with the retuned flip" below has the merged firmware, on the fitted plant too). sw/sim/recover_firmware.py flies recover.c, flip.c, flight_math.c, hover.c and guard.c in sw/sim/flip_firmware.py's harness, ticked as flight_rt.c ticks them, on drone 1's plant (2026-11-09) randomized as recover.randomization, from the 2026-11-11 23:35 settings with the GPS hold. At 14 s after liftoff each holding drone is put into a drawn attitude and rates (the estimate with it), 256 runs per row. Recovered means never cut, never touched the ground, and upright, still and above 0.5 m 6 s later; "of holding plants" leaves out the 20% of drawn plants whose attitude loop does not hold even without the event.

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

## Merged with the retuned flip (william/merge-sim)

william/flip-fit and william/recovery were merged into one firmware and one simulator harness. Nothing here has flown either.

**What changed**
- **One righting.** flip.c's flip_righting (the thrust axis turned toward up about the axis between them, no faster than it can brake from, the long way round when far over and mostly pitched) and flip_allocate (into thrusts and tilts, each motor between a floor and a ceiling of the weight) are what both the flip's last stage and recover.c fly, each with its own gains (FlipRighting). recover.c lost its copies, and both carry their attitude with the same quaternion helpers (flip_attitude, flip_rotate, flip_up, flip_euler). flip.h's and recover.h's existing functions are unchanged; flip_limit_with is new.
- **The recovery's motors follow the flip's rules.** Each keeps at least the flip's floor, 0.2 of the weight (FLIP_FLOOR; it was 0.144: about a point fewer recoveries from tilts at rest in the sim, nothing elsewhere). flight_rt.c slews and caps the throttles while a recovery and its 1 s grace run, as through a flip (the full range in no less than 0.12 s, at most 1100); before, only a recovery inside the flip's window was limited, and one that took over a flip lost the limit the tick it cancelled it. The recovery has no ceiling of its own (the flip's 1.16 of the weight is one of its tuned gains); the cap bounds it.
- **No ambiguous attitude reaches the attitude controller.** While the estimate is on the gyro alone for the flip (its climb to 1 s after the hand-back, the arrest included) it is the flip's quaternion's roll and pitch every tick, as it already was through a recovery, and now through the recovery's grace too. From a quaternion pitch comes back within ±90 degrees, so a drone within 90 degrees of upright has both angles under 90: the arrest hands back within 34 degrees, and the leveling runs only within right_angle (58 degrees) or tip_limit (at most 80) of upright, beyond both it is righting again. flight_attitude keeps pitch within ±90 degrees as a backstop. On the 02:27 recording's gyro from the hand-back (open loop), the recorded estimate went to roll 135, pitch 128 at 11.27 s; the new flight_attitude gives roll -47, pitch 51 and the flip's quaternion roll -49, pitch 54 (the same attitude; the Euler one still errs by up to about 30 degrees around the vertical itself, the quaternion does not).
- **The simulator** (sw/sim/flip_firmware.py) is flip-fit's harness with the recovery's in it, ticked as the merged flight_rt.c: flip_tick, recover_tick (the tip cut or the recovery), the failsafe and landings, the hover, the controller, flip_actuators for the flip's or the righting's motors, flip_limit_with. The plant is flip-fit's: recover_firmware.py now flies the airframe fitted to the first flip with every untested term drawn (flip_firmware.uncertain) by default; `--airframe airframe_2026-11-09.json --randomize wide` is the plant the recovery was tuned on. The recovery's `--gyroscopic` push now adds to the airframe's constant pushes instead of replacing them, and is for airframes without rotor_momentum (the fitted one has it).

**The flip, before and after** (tune_flip.py --check, the same plants as above, seed 90000, from 5 m; before is william/cook a99e6fe):

| | clean | touched | tip cut | height lost median (90th, worst) | past 90° median (90th) | tilt after the hand-back median (90th) |
|---|---|---|---|---|---|---|
| retuned, 512 drawn, before | 92.5% | 7.1% | 3.2% | 0.00 m (3.74, 5.58) | 0.32 s (0.53) | 33° (49) |
| retuned, 512 drawn, after | 92.5% | 7.1% | 3.0% | 0.00 m (3.74, 5.58) | 0.32 s (0.53) | 33° (49) |
| retuned, 512 drawn, after, recover_enable 1 | 92.5% | 7.1% | 2.0% | 0.00 m (3.68, 5.58) | 0.32 s (0.53) | 31° (41) |
| as flown on 2026-11-12, 512 drawn, before | 47.0% | 52.6% | 30.2% | 4.68 m (5.18, 5.61) | 0.38 s (2.78) | 70° (180) |
| as flown, after | 47.6% | 52.0% | 28.0% | 4.67 m (5.18, 5.61) | 0.38 s (2.74) | 70° (180) |
| 60° burst, 256 drawn, before / after | 100% / 100% | 0 / 0 | 0 / 0 | 0.00 m (0.01, 0.01) both | 0 | 33° (43) both |
| 90° burst, 256 drawn, before / after | 100% / 100% | 0 / 0 | 0 / 0 | 0.00 m (0.46, 3.13) both | 0.20 s (0.32) both | 33° (41) both |
| the fitted plant as saved, 64, before / after | 100% / 100% | 0 / 0 | 0 / 0 | 0.00 m (0.02, 0.21 / 0.20) | 0.26 s | 34° (38) both |

The merged harness and the shared righting alone printed exactly what a99e6fe did; the small changes in the flip as flown come from the attitude estimate through its arrest (now the quaternion's). Recovery on does not save the retuned flip's failures from 5 m (the same 7.1% touch the ground), nor from 3 m on the same plants (recover_firmware flip: 89.1% clean and 10.5% touched either way; 77.7% holding at the end with it, 76.6% without).

**The recovery on the fitted plant** (recover_firmware.py, 256 runs per row, the plant drawn over every untested term; every drawn plant holds without the event, so "of holding plants" is the same number; recovered means never cut, never on the ground, upright, still and above 0.5 m 6 s after the event):

| event | hold | recovered | touched the ground | height lost median (90th) | hand-back to the hold median |
|---|---|---|---|---|---|
| tip: 45-75° over, ~100 deg/s | 2 m | 94.1% | 0.4% | 0.19 m (0.65) | 1.6 s |
| | 3 m | 94.5% | 0 | 0.20 m (0.70) | 1.6 s |
| tilt: 60-150° over, at rest | 2 m | 66.0% | 20.7% | 0.98 m (2.00) | 1.7 s |
| | 3 m | 74.6% | 7.4% | 0.97 m (2.52) | 1.7 s |
| tumble: 90-180° over, pitching 400-600 deg/s, sinking to 2 m/s | 2 m | 25.8% | 70.7% | 1.89 m (2.26) | 2.0 s |
| | 3 m | 42.6% | 50.8% | 2.68 m (3.22) | 1.9 s |
| Level with recover_enable 0: 20-60° over, up to 100 deg/s | 2 m | 96.5% | 0 | 0.14 m (0.54) | 1.5 s |
| | 3 m | 96.5% | 0 | 0.15 m (0.55) | 1.5 s |
| link-loss landing, 60-150° over at 0.5-2.5 m | 2 m | above 1 m 73.1% of 186; below it 60 of 70 cut as designed | 71.5% | 1.14 m | 1.8 s |
| | 3 m | above 1 m 68.3% of 186; 60 of 70 cut | 53.9% | 0.93 m | 1.7 s |

On the plant the recovery was tuned on (airframe_2026-11-09, recover.randomization), the same firmware recovers 70-71% of tips (84-85% of holding plants; on william/recovery 76-77%, 88-90%), 61-63% of tilts (73-77%; 65-67%, 76-78%), 23-36% of tumbles (29-44%; 25-39%, 29-46%) and 74% of Levels (88-89%; 79%, 91-92%). Most of the difference is not the merge: flip-fit's changes to the plant's code (rigid_step.c, airframe.py) alone moved the william/recovery harness and firmware to 71.5-72.3% of tips, 62.9-64.1% of tilts, 25.4-39.8% of tumbles and 74.6-75.0% of Levels, run for run what the merged harness gives before the motor changes below. The rest is the slew and cap. With `--wide` on that plant (roll authority 0.4-5 times, the gyroscopic push, a deadband, slower motors) the merged firmware recovers 71-72% of tips (william/recovery 77-78%), 55-60% of tilts (64-69%), 16-27% of tumbles (15-26%) and 81% of Levels (84-85%); the merged harness before the motor and attitude changes already gave 56-62% of tilts there, flip-fit's plant code and the gyroscopic push no longer replacing the airframe's constant pushes accounting for most of it.

**The slew limit's trade-off** (the fitted plant, 256 runs per row, floor 0.2; unlimited is as william/recovery flew it outside a flip):

| event | hold | unlimited: recovered / largest throttle step in a tick (max) / at ≥95% throttle median (90th) | slewed 0.12 s full range, cap 1100 (the firmware) | slewed 0.06 s full range |
|---|---|---|---|---|
| tip | 2 m | 93.8% / 258 / 0 (0.03 s) | 94.1% / 100 (178 after the grace) / 0 | 93.4% / 200 / 0 |
| | 3 m | 94.5% | 94.5% | 93.8% |
| tilt | 2 m | 71.1% / 1004 / 0.06 s (0.16) | 66.0% / 100 / 0 | 69.1% / 200 / 0 |
| | 3 m | 77.3% | 74.6% | 75.8% |
| tumble | 2 m | 25.4% / 1004 / 0.10 s (0.29) | 25.8% / 100 / 0 | 25.4% / 200 / 0 |
| | 3 m | 42.6% | 42.6% | 41.8% |
| Level | 2 m | 96.9% | 96.5% | 96.9% |
| | 3 m | 97.3% | 96.5% | 96.9% |

Unlimited, the tuned righting is close to bang-bang: in tilts and tumbles it steps a throttle by up to 1000 in one 10 ms tick and holds a motor at 95% or more for 0.06-0.10 s median, what the new ESCs should not see after the old left one burnt. Slewed as the flip's, no step exceeds 100 a tick and no motor reaches 95%, for 3-5 points fewer recoveries from 60-150° at rest and none fewer from tips, tumbles or Level, where the drone falls too far before authority matters. Halving the slew time (0.06 s, about the motors' lag) gives back half of the tilts' loss. On the plant the recovery was tuned on, whose roll authority spans 0.8-5 times the fit, the floor, slew and attitude changes together cost 1-4 points of tips, tilts and tumbles and triple the rightings that time out (Recovery failed, stop reason 10: 9 to 27-29 of 256 tips, 11 to 39 tilts), the drone then cut in the air rather than drifting down tilted. The gains were tuned unlimited; retuning them slewed (recover_firmware.py --tune, which now flies the firmware's limit) is the way to win the tilts back.

**What these results rest on**: the fitted plant is one flight's fit (the first flip), and the sim matched every gentle flight while mispredicting that flip; its untested terms are drawn over their plausible ranges, but nothing past 35 degrees of tilt, at the rates of a tumble or near the ground in a righting has been flown or measured. The recovery's numbers at large tilts rest on the righting's roll and pitch authority there, the rotors' momentum (rotor_momentum, drawn 0.006-0.04), the gyro carrying the attitude through a tumble, and the ground contact. The slew's cost rests on the motors' modeled lag (0.04-0.1 s drawn); with slower real motors the slew matters less, with faster ones more. The ESC benefit is not modeled at all: the sim has no current spikes, desyncs or burnt joints, so the limit is chosen for what it removes (single-tick jumps between idle and full throttle, time at full throttle) at the cost the sim shows.

**Unchanged with flip_enable 0 and recover_enable 0** (byte-identical output against a99e6fe): sweep.py's nominal hover, band_hold.py's tag holds (four cases, 16 seeds), landing_test.py's 200 landings, outdoor.py (16 runs per wind), takeoff_test.py, and flip_firmware.py --no-flip's GPS holds on both plants (64 each at three band offsets). The only change outside flip.c and recover.c these programs run is flight_attitude's backstop, which acts only past 90 degrees of pitch.

## What real data tests each assumption, and what does not

- **The gyro carries the attitude through a tumble** (no accelerometer). Tested only up to 600 deg/s, in the replay: the quaternion and the Euler estimate (both on the gyro then) agree to a degree until the Euler one's singularity. Past the gyro's 2000 deg/s nothing can: the attitude is then unknown, and in the air the accelerometer cannot tell (it reads the thrust, not gravity, whatever the attitude). After a saturation trigger the recovery flies on an attitude that may be wrong by tens of degrees.
- **Roll authority, pitch authority and their lags at large tilts and rates.** Fitted only within 35 degrees of level and at hover rates. The flip's spin reached 540 deg/s on the drone against the simulator's 950, so the roll authority at low thrust is likely at the low end of the wide range; the tuned gains hold up there (wide plant column) except in tumbles.
- **The props' gyroscopic coupling.** Not in the identified model at all; its size (±0.03) is a physical estimate. During the real flip the pitch rate grew to -160 deg/s while rolling at 340 deg/s with the motors at 464 and 34 throttle, consistent with it, but not a measurement of it.
- **ESC behavior near throttle 1 and restarts.** Not measured; the recovery keeps every motor at 20% of the weight (the flip's floor) to stay out of it.
- **The ground.** The simulator's contact is rough, and a recovery that meets the ground drives its motors into it until the 2.5 s timeout or the end of the righting. A drone tumbling from under about 2 m mostly hits the ground first either way.

## Enabling and testing it

Flash `william/merge-sim` (the recovery with the retuned flip) as in step 1 above. `recover_enable` stays 0 until set; the Level button works without it.

### A. On the ground, props off

1. Props off, drone in hand or on a stand. `uv run -m sw.flight_steps recover --apply` (tip_limit 30, recover_enable 1).
2. Arm, stay in manual, raise the collective past integrate_throttle (580) and hold it there more than 1 s (the recovery waits for 1 s airborne).
3. Tilt the drone by hand past 30 degrees in roll: the status line shows `Recovering (tipped): righting`, the low side's motor speeds up and the high side's slows (never stopping, and ramping: the full throttle range takes at least 0.12 s, and no motor goes past 1100). Tilt it nose-down past 30: both servos tilt back (nose-up), with both motors up. Hold it tilted: after 2.5 s the motors stop with "Recovery failed".
4. Arm again and repeat, bringing it back level within a second: `leveling`, then the drone's hold (the recording's hover mode, flags bits 1-2, goes to 1; the dashboard's mode label still shows what it sends). STOP.
5. Arm, collective up past 580 for 1 s, press Level: `Recovering (level): leveling`, then the hold. STOP.
6. In the recording: the `recovery` (1, 2) and `recovery_cause` (1, 4) columns, the throttles and servos against the gyro's sign, stop reason 10 on the held tilt.
7. `uv run -m sw.flight_steps norecover --apply` before flying anything else.

### B. In flight, from a modest tilt

Calm air, the 3 m GPS hold as cleared in steps 4-5 above, 10 m clear, STOP ready.

1. **Level in a plain hold.** Settings `high`, recover_enable 0. Hold 3 m, press Level once: leveling for a moment, then the hold, as before. Then press L to land: the hold gives way to the landing.
2. **Level during a roll step.** Settings `ladder` at 3 m (`high`), a 30-degree step (`uv run -m sw.sim.tilt_ladder --amplitudes 30 --width 0.4 --roll-rate 360 --dry`). Press Level as the step starts: the pattern stops (pattern stop 6) and the drone levels and holds. In the sim, 20-60-degree Levels hand back to the hold in 1.6 s median and lose 0.16 m median.
3. **Recovery from a 35-degree step.** `uv run -m sw.flight_steps recover --apply` (tip_limit 30, recover_enable 1) and a 35-degree step. It trips the recovery at 30 degrees: `Recovering (tipped)`, then the hold. Afterwards `norecover`.
4. Only then consider larger tilts, and fly the flip with recover_enable 1 only after stages A-D of "Staged tests before another flip" have passed: on the fitted plant the recovery saved none of the retuned flip's failures in the sim (the same 7.1% on the ground from 5 m with it on or off).

Abort a test with STOP if the righting does not turn the drone toward upright within half a second, or it falls below about 1 m.
