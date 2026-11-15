# Bursts, tumble recovery and Level

The history of the whole flip (2026-11-12), its retunes, the Euler attitude and the staged tests is on `william/archive-2026-11-15-indi` (this file before the consolidation) and in `git log`. This is what the firmware flies now.

## The burst (flip.c)

The Flip request (radio.c `f`, the dashboard's Flip button) starts one burst per arming. From a settled GPS hold the drone rolls about its own forward axis to `flip_angle` degrees and back: one reference rotation (a half-sine acceleration ramp, a coast, a braked stop, then back the same way) tracked by the one attitude law (flight_math.c `attitude_error`, `attitude_rates`) with the reference's rate and acceleration fed forward. The motors and servos are flown directly (`flip_allocate`: roll by differential thrust, pitch by common tilt, yaw by differential tilt), each motor between `floor` and `ceiling` of the weight, the throttles slewed and capped (`flip_limit`). The collective's vertical part holds the weight; under the height cap (the lower of `hover_ceiling` and 5 m, less 0.3 m and what the climb still carries) it brakes. Within `right_angle` of upright and turning under `handoff_rate` (or at `timeout`) it hands back to the attitude controller, which gets its integrators back, while the collective arrests the vertical speed; the hold takes over after that.

The reference waits for a drone that falls behind it (a reference governor): an authority short of the model's slows the turn instead of leaving the drone short.

The authority (flip.h `FLIP_*_AUTHORITY`) is measured, not the settings' model's: roll 13.5 (the raised pivots), pitch 16.8 and yaw 11.34 per N (the 2026-11-15 bursts). The recovery flies the same authority, and INDI's unmodeled acceleration for both comes from that one model.

The request is refused unless armed, holding on the GPS (not the camera) with the heading pinned and a fresh fix, airborne 5 s, at `flip_height` or higher, within 10 degrees of level, turning under 30 deg/s, the climb rate under 0.3 m/s, the ground speed under 1 m/s, the pack at 14.8 V or more, no landing, failsafe or test pattern, and no recovery running.

## Tumble recovery and Level (recover.c)

With `recover_enable` 1 a drone past `tip_limit` in flight, a burst whose nose goes past 30 degrees from level while it turns, one past `tip_limit` in its arrest or grace, or a gyro at its full scale is righted instead of cut: the one attitude law toward upright at the drone's own heading, at the recovery's gains (`recover_defaults`), each motor at least 0.2 of the weight. Within `right_angle` and slow it hands to the attitude controller, leveling while the collective arrests the vertical speed, then to the hold. Not on the ground, not landing below 1 m, not in the first second airborne, at most 3 times an arming; a righting that is not upright in 2.5 s cuts (stop reason 10).

The vertical speed is the accelerometer's integral until either sensor reads within 95% of its full scale (±2000 deg/s, ±8 g since the consolidation); then it is the barometer's rate at the gain its lag allows, and the hold is handed the barometer's height. Over `hover_ceiling` + 1 m by the raw barometer, no collective may climb.

Level (radio.c `h`, the dashboard's Level button) starts the same recovery whatever `recover_enable` says.

## Settings

| setting | use |
|---|---|
| `flip_angle` | degrees, 0 refuses the request (default); 30, 60 and 90 have been simulated |
| `flip_height` | least hold height for the request (m) |
| `recover_enable` | 1: recovery in place of the tip cut |
| `tip_limit` | the tilt past which the cut or the recovery comes (degrees) |
| `indi`, `indi_pitch` | INDI's shares (roll and yaw; pitch), for the hold, the burst and the recovery alike. Keep `indi_pitch` 0: at 0.5 everywhere the sim's bursts fell to 30-70% clean and the recovery to about 25% |
| `hover_climb`, `hover_ceiling` | the hold's height and ceiling, 5 m at most |

## In the simulator (consolidated firmware, 32 runs per rotor_momentum band 0-0.05, from the 3 m hold, indi 0.5, indi_pitch 0, recover_enable 1)

`uv run -m sw.sim.flip_esc now --kind burst30|burst60|burst90 [--airframe airframe_2026-11-15_bursts.json] [--sag]` and `uv run -m sw.sim.recover_firmware --heights 3`:

| | hover-fit plant (raised) | bursts plant |
|---|---|---|
| burst30 clean | 100% | 92-100% |
| burst60 clean | 100% | 88-92% |
| burst90 clean (touched) | 78-97% (3-19%) | 56-76% (16-36%) |
| recovery tip / tilt / tumble / abort | 100 / 91 / 47 / 98% | 53 / 56 / 22 / 67% |

burst90 did better while pitch took INDI's roll share during the burst (97% hover-fit, 81-92% bursts plant), a regime the consolidation removed; burst30 and burst60 are unchanged by it. Fly burst30, then burst60; burst90 is not cleared.

## Checklist

1. Flash (`make -C src/fw deploy` from the branch), disarmed. The dashboard's settings dialog lists `flip_angle`, not `flip_enable`. Calibrate, and record 30 s at rest: `accel_counts_per_g` 4096 and `gyro_counts_per_deg_s` 16.4 in the metadata, the gyro's mean within 0.1 deg/s.
2. A plain GPS hold at 3 m, 30-60 s: the pitch shake no worse than the last flights' (2.3-2.5 Hz, 12-19 deg/s).
3. Level once in the hold, then land.
4. `recover_enable` 1, `tip_limit` 30, a 35-degree roll step: the recovery takes it and hands back to the hold.
5. `flip_angle` 30 from the 3 m hold under a 5 m ceiling, calm air, 10 m clear, STOP ready. Then 60.
