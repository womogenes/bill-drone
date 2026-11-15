# Drone 1's sim models: what is current

The notes up to 2026-11-15 (every fit, replay, retune and the incidents behind them, 860 lines) are on `william/archive-2026-11-15-indi` in this file, with the figures they cite; `git log` has each change's evidence. What follows is the state after the consolidation of 2026-11-15. The plant's physics (rigid.py, rigid_step.c, airframe.py and the airframe and geometry files) belongs to the modeling work and is described there as it changes.

## The plants

- **Raised pivots** (`airframe_2026-11-12_raised.json` at `geometry_2026-11-12_raised.json`): drone 1 since the servo pivots were raised 20 mm on 2026-11-11, refitted (sw/sim/identify_airframe.py) on the hovers flown after it. The hover-fit plant: flip_firmware.AIRFRAME, the default of flip_esc.py, recover_firmware.py, tune_flip.py and scorecard_closed.py.
- **The bursts' plant** (`airframe_2026-11-15_bursts.json`): the three bursts of 2026-11-15 (05:29 burst90 with the rubber bands, 07:03 burst60 and 07:13 burst30 without), replayed open loop on all axes by sw/sim/burst_replay.py, fit the tilt's pitch authority at about 2 times the hover fit's (tilt_authority 1.95), the servos 40-47 ms late, yaw's tilt authority 0.74 of the geometric and the forward speed's pitch moment past 1.5 m/s (speed_range 8, speed_moment2). The hovers had said 1 because their small swings dilute the regression (errors in variables): their spans with swings past 4 degrees fit 2.2-2.9. Closed loop only this plant shakes at the drone's 2.3-2.6 Hz and diverges at pitch_damping 8 as the drone shook past 6, and it reproduces the 07:03 burst60's nose-down swing (burst60_repro_2026-11-15.png). The firmware's FLIP_PITCH_AUTHORITY 16.8 and FLIP_YAW_AUTHORITY 11.34 come from it.
- **Bands off** (`airframe_2026-11-15_bandsoff.json`): the raised plant's pitch path refitted on the INDI-free hovers of 2026-11-15 from 06:29 UTC, with the rubber bands off.
- The hover sims (sweep.py, landing_test.py, takeoff_test.py, outdoor.py) keep the older plants they were calibrated on, so their outputs stay comparable from change to change.
- **The scorecard** (sw/sim/scorecard.py, models/scorecard/): every drone 1 recording inventoried (inventory.json), each post-raise flight scored by k-step-ahead prediction against the do-nothing prediction, per-flight CoM terms, held-out totals per plant and paired --compare; scorecard_closed.py reruns flights closed loop on the firmware's C. On the post-raise flights (open loop total / held out): hover fit 1.020 / 1.033, bursts plant 1.120 / 1.207, bands-off 1.015 / 1.099.

## The new ESCs

From 2026-11-14 the ESCs are Nuofany 40A calibrated to 1000-2000 µs. The thrust stand (sw/recordings/stand/2026-11-14T18-07-54; flip_esc.STAND and STAND_VOLTS) against the firmware's table times the 1.3 thrust scale: the stand gives 0.41-0.78 of it below hover throttle and 1.04-1.17 from 840 up. The pack sags as V0 - k T^1.5 per motor (16.31 V, 0.0160 V/N^1.5; flip_esc --sag). flip_esc's "now" case flies the stand's table with the CoM 15.5 mm left (the 2026-11-15 hovers' 8-10% split) at the drone's thrust scale and hover_mass.

## Quaternion attitude

The estimate is one quaternion (flight_math.c flight_attitude: the gyro turns it, the accelerometer pulls its up toward its own about a horizontal axis, so the heading stays the gyro's); the controller's roll and pitch errors are the tilt-prioritized error (Brescianini and D'Andrea) onto the setpoint at the drone's own heading, near level the Euler errors. The burst and the recovery fly the same estimate and the same error law in a rate loop (attitude_rates). Its cost on the host was 1.08 µs a tick against the Euler path's 0.52; on the ESP32 an estimated 10-25 µs of the 10 ms tick, to be measured on the flight controller.

## INDI

settings.def indi takes a share of the angular acceleration the gyro measures beyond the actuators' models (indi.c). In roll and yaw 0.5 steadies the hover (pitch rate RMS 7.8 -> 3.4 deg/s in the sim, roll 1.2 -> 0.7 degrees on the drone); much over 0.5 rings the assemblies' links at about 11 Hz. In pitch (indi_pitch) it doubled the drone's 2.6-3 Hz hover shake on 2026-11-15, and in the consolidated firmware, which takes indi_pitch for the hold, the burst and the recovery alike, 0.5 drops the bursts to 30-70% clean and the recovery to about 25% (below). Keep indi_pitch 0.

## Yaw in the flip

The bursts' yaw peaks are the rotors' spin momentum changing hands as the motors' split reverses (-dH_z/dt, H_z = rotor_momentum (√T_right - √T_left)); the differential tilt alone cannot take it back. yaw_ff 1.6 (rotor_momentum 0.018 over the yaw inertia; flight fits put rotor_momentum at 0.006-0.03, the prop's geometry at about 0.014) feeds it forward, judged on the worst rotor_momentum band below 0.03.

## Consolidated firmware in the sim (2026-11-15)

flip_esc "now" (32 runs per rotor_momentum band 0-0.01 ... 0.04-0.05, from the 3 m hold under a 5 m ceiling, indi 0.5, recover_enable 1, seed 95000) and recover_firmware (64 events a scenario from the 3 m hold, indi 0.5), the merged branch before the consolidation (pitch taking indi's share while the burst or the recovery flew) against the consolidated firmware (indi_pitch everywhere; ±8 g accelerometer):

| | before, raised | after, raised | before, bursts plant | after, bursts plant |
|---|---|---|---|---|
| burst30 clean | 100% | 100% | 92-100% | 92-100% |
| burst60 clean | 100% | 100% | 92-96% | 88-92% |
| burst90 clean (touched) | 97% (3%) | 78-97% (3-19%) | 81-92% (0-13%) | 56-76% (16-36%) |
| recovery tip / tilt / tumble / landing / abort | 98 / 81 / 56 / 47 / 91% | 100 / 91 / 47 / 41 / 98% | 53 / 49 / 40 / 23 / 67% | 53 / 56 / 22 / 30 / 67% |
| indi_pitch 0.5 everywhere: bursts 30/60/90 clean | | 33-67 / 29-42 / 37-52% | | 66-72 / 56-78 / 38-59% |

The ±8 g accelerometer on its own (same firmware otherwise): recovery tip 97 -> 100%, tilt 86 -> 91%, tumble 61 -> 47% on the raised plant (the ±2 g clip had put tumbles on the barometer's gentler arrest; the arrest gain kv 7.2 on the accelerometer's integral is the next thing to look at), bursts within a run or two. The hover sims: outdoor (the GPS and barometer, the raised plant, 32 runs at 0/2/4 m/s: held 32/32/26) and takeoff_test are unchanged; landing_test (400 indoor landings) ends a little later after contact (median 0.46 -> 0.62 s, 90th 1.65 -> 1.76; none in the air) since the Kalman filter now runs beside the hover from a 5 s rest and its touchdown is always on; sweep's nominal free flight no longer has velocity damping (8 by default, 0 on the drone since late October).
