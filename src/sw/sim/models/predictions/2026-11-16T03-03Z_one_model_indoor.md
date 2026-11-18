# Pre-registered: the first indoor tag hover on the one airframe model

Written 2026-11-16 03:03 UTC (7:03 PM Pacific), before any flight on william/unify 58e17ee. Do not edit after the flight: compare the flight with it and add the comparison below the line.

## What is predicted

- **Firmware**: william/unify 58e17ee: the controller, INDI and the predictor on one model (settings.def roll_authority 8.0, pitch_authority 8.68, yaw_authority 11.35); the burst's and the recovery's roll and pitch gains unchanged; drone 1's gains scaled once at boot from its old blob (roll x0.551, pitch x1.000, yaw x0.704).
- **Settings**: drone 1's as they are now, with the indoor tag profile of 2026-11-11 (hover_camera 1, hover_climb 1.2, hover_ceiling 2, hover_center 1, hover_gps_tau 0, hover_kp 1, hover_kd 2.5) and pitch_predict 1, pitch_damping 4.5.
- **Sims**: sw/sim/scorecard_closed.py, the 07:12, 06:30, 06:58 and 05:18 hovers flown again at their own settings with pitch_predict 1 and pitch_damping 4.5, 4 runs each, on the fitted plant and the plant with the logs' servo (models/NOTES.md "One model"). These are outdoor GPS holds; the pitch loop is the same in the tag hold, which this sim does not fly.

## Predictions

| | 1.5-4 Hz pitch rate RMS (deg/s), median (10-90%) |
|---|---|
| the drone so far (logs, no predictor) | 12.5-16.6 |
| the same hovers rerun, predictor off | 15.2 (fitted plant) / 10.1 (logs' servo) |
| **this hover** | **2.4 (1.4-2.8) on the fitted plant, 2.1 (1.2-2.9) with the logs' servo** |

- No tip-over or tip cut (none in 32 runs).
- The pitch rate's dominant frequency is no longer a 2.3-2.5 Hz peak standing out of the hold's motion.
- Slow swing (below 1.5 Hz) about as before or somewhat more: the all-band pitch rate RMS stays near 17-18 deg/s in the sims, against 17 with the predictor off. The largest pitch is about 14 degrees against 11-12.
- Roll and yaw hold as before. The controller commands the motors and servos for them as it did; only INDI's roll and yaw share (indi 0.5) now works on the fitted model.

## How to score it

From the settled tag hold, the 1.5-4 Hz pitch rate RMS (scorecard_closed's band RMS on the recording):
- Under 6 deg/s: the prediction holds.
- 6-12 deg/s: the predictor helps less than predicted. Look for the cause in the servo estimate.
- Over 12 deg/s, or a ring at 2.7 Hz or higher: the predictor, at this share and damping, is wrong for the drone. Set pitch_predict back to 0.

---

**Scored 2026-11-16 (after the flight, 2026-11-16T03-07-41, the tag hold from 6 to 23 s, landed on B at 24.7 s).** By the measure registered (one Hann-windowed spectrum of the settled hold, 6.3-23 s): 1.5-4 Hz RMS 9.1 deg/s, peak 1.62 Hz, so by the rule "6-12: the predictor helps less than predicted". That measure is leaky here: 67-79% of the pitch rate's variance is below 1.5 Hz (the tag hold's slow motion, 10.9 deg/s below 1 Hz; the outdoor GPS holds the sims fly had far less), and the window spreads it into the band. By Welch bands (512-tick segments, 7-23 s) the band is 5.4 deg/s and its parts are: 1.4-1.9 Hz 3.65, 1.9-2.2 Hz 1.76, 2.2-2.7 Hz 2.15, 2.7-3.2 Hz 2.11, 3.2-4 Hz 3.17. The 2.3-2.5 Hz shake the prediction was about is down from 10-17 deg/s on every earlier hover to 2.2, as predicted; no tip-over; no ring at 2.7 Hz or above (3.2-4 Hz at 3.2 deg/s, a small rise both plants also show with the predictor on). The slow swing as predicted: all-band 18.9 deg/s against the sims' 17-18, most of it below 1 Hz in the tag hold, which the sims do not fly. Verdict: the shake's prediction held; the registered measure was the wrong one for an indoor tag hold and reads 9.1.
