# Pre-registered: the first hovers and bursts with the servos' dead-time predictor

Written 2026-11-15 19:21 UTC (11:21 AM Pacific), before any flight with pitch_predict, on branch william/predictor. Do not edit after the flight: compare the flight with it and add the comparison below the line. The predictor is fw/drone/main/indi.c indi_predict (settings.def pitch_predict; models/NOTES.md "Servo dead-time predictor").

## What is predicted

- **Firmware**: william/predictor (william/consolidate 90c5c11 with the timing fix 68596c7 and the plant c144e10 merged, plus the predictor), flown by its own sw/sim/flip_firmware.py.
- **Plants**: two, because they disagree about the servos and the flight decides between them. "Fit" is models/airframe_2026-11-15_plant.json as fitted (servo dead time 75/55 ms, wn 33.3, zeta 0.22): it has the drone's shake amplitude (11 deg/s in 1.5-4 Hz against the logs' 12-17) but 0.4 Hz fast. "Servo" is the same plant with the servo the logs support (55 ms both, wn 37.5, zeta 0.87; sw/sim/predict_check.py foretells the drone's pitch rate 50 ms ahead with correlation 0.92-0.98 on that servo model and 0.63-0.74 on the fit's): it shakes at 2.43-2.57 Hz like the drone but at about half the drone's amplitude.
- **Settings**: the drone's at 07:13 UTC (pitch_p 36, indi 0.5, indi_pitch 0, recover_enable 1, hover_climb 3, flip_height 2.5) with A: pitch_predict 0.5, pitch_damping 6 (as now) or B: pitch_predict 0.875, pitch_damping 4.5. Hovers: the 07:12, 06:30, 06:58 and 05:18 hovers rerun (sw/sim/scorecard_closed.py, 2 runs each). Bursts: from the 3 m GPS hold, 96 runs each, the pre-registered burst30's draws (tilt_authority 1.45-1.70, rotor_momentum 0-0.007, flap_lever 0.11-0.18, the CoM ±1.5 mm), the servo dead times drawn 65-85/45-65 ms (fit) or 45-65 ms both (servo).

## Hover: pitch rate (deg/s RMS) by band, median over the reruns (10-90%)

| | 1.5-4 Hz (the shake) | 4-10 Hz | below 1.5 Hz |
|---|---|---|---|
| the drone now (logs, pitch_predict 0) | 12.5-16.6 | | |
| fit plant, now | 11.3 (11.1-11.6) | 0.8 | 13.3 |
| servo plant, now | 7.0 (5.8-7.6) | 0.5 | 14.4 |
| **A**, fit plant | **17.9 (12.5-19.6)**: worse | 0.9 | 13.4 |
| **A**, servo plant | **3.1 (2.4-3.9)** | 1.2 | 12.7 |
| **B**, fit plant | 2.8 (1.3-3.0) | 5.3 (90th 5.5, a peak near 8 Hz) | 16.2 |
| **B**, servo plant | 2.1 (0.9-2.5) | 2.0 | 15.0 |

A hover with A tells the plants apart: the shake grows toward 18 deg/s if the fit's servo is the drone's and falls toward 3 if the logs' servo model is. Neither plant tips over with A or B in any rerun.

## burst30 (flip_angle 30), 96 runs each

| | clean | roll peak | nose up | nose down | hand-back | height lost |
|---|---|---|---|---|---|---|
| A, fit | 100% | 32.5 (32.0-33.0) | 17.8 (12.3-26.1) | -10.8 (-15.4 to -7.0) | 0.66 s | 0.26 m (0.17-0.34) |
| A, servo | 100% | 32.3 (31.8-33.0) | 19.3 (11.4-29.9) | -10.2 (-15.1 to -5.5) | 0.66 s | 0.25 m (0.17-0.34) |
| B, fit | 100% | 32.5 (31.9-33.1) | 18.4 (11.8-27.5) | -13.3 (-19.1 to -6.2) | 0.66 s | 0.25 m |
| B, servo | 92% (4 refused) | 32.3 (31.6-32.8) | 19.9 (11.5-30.2) | -14.9 (-21.2 to -6.7) | 0.66 s | 0.25 m |

Without the predictor (pitch_predict 0, damping 6) the nose went up 26 degrees (fit 15-37, servo 17-39), and on the servo plant 5 of 96 bursts were refused and 74% clean.

## burst60 (flip_angle 60): still not to be flown

| | clean | touched | nose up |
|---|---|---|---|
| now, fit / servo | 32% / 5% | 54% / 69% | 86 / 85 deg |
| A, fit / servo | 53% / 92% | 42% / 6% | 76 / 66 deg |
| B, fit / servo | 74% / 92% | 18% / 0% | 67 / 61 deg |

The predictor helps the hold catch the nose after the hand-back, but the nose still goes up 60-80 degrees once speed and the 60-degree roll meet (the rotors' in-plane force; models/NOTES.md "One plant"). That needs its own fix before burst60.

## How to score it

From the hover with A: the 1.5-4 Hz pitch-rate RMS over the settled hold (sw/sim/scorecard_closed.py's band RMS on the recording). Under 6 deg/s supports the servo plant; over 12 supports the fit plant and says B is the setting to try. For burst30, as in 2026-11-15T11-12Z_burst30.md: roll peak and hand-back within the bands, the nose's largest excursion within 11-30 degrees, height lost 0.1-0.5 m.

---
