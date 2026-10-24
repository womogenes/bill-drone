# Bicopter simulation

## Code and operation

Firmware and simulation share the C attitude estimator, controller and thrust table in `fw/drone/main/flight_math.c`; `sw/sim/flight.py` wraps it for the simulation and adds thrust conversion and gyro calibration. The controller receives accelerometer and gyro readings. It uses fixed geometry and inertia estimates; position and altitude are recorded for evaluation.

## Thrust calibration

The thrust table uses three October 4 sweeps (the CSVs and `compare.py` are kept on the `claude/slop-archive` branch under `src/sw/slop/`), calibrated with the 1.62 kg bottle comparison. `compare.py` excludes the two confirmed glitches at throttle 750. It calculates a median at each throttle for each run; the shared table uses the median across runs and interpolates between those values.

Throttle 200 is treated as approximately zero thrust, based on the bench observation. The curve retains the measured decline above 1100. Commands below 200 produce zero modeled thrust; commands above 1200 hold the last table value. Converting thrust back to throttle uses the rising branch, capped at 1100. Both motors use the same curve. Battery voltage and propeller configuration were not recorded in those CSVs.

## Sensor and actuator response

The MPU settings match `fw/drone/main/flight_rt.c`: 100 Hz sampling, ±2 g acceleration, ±250°/s gyro range, and DLPF setting 3 (44 Hz accel / 42 Hz gyro bandwidth). The [InvenSense specification, sections 6.1–6.2](https://www.haoyuelectronics.com/Attachment/GY-521/mpu6050.pdf) gives noise densities of 0.005°/s/√Hz for the gyro and 400 µg/√Hz for acceleration. The simulation estimates RMS noise as density × √bandwidth and approximates sensor latency with a 5 ms filter. It also applies ADC quantization and clipping.

Gyro bias is `[0.02, 1.10, -1.95]` °/s, from an earlier stationary dashboard observation. The simulation averages the first 200 stationary samples to estimate that bias. Accelerometer offsets are zero because tilt and bias were not measured independently. Powered logs contain both sensor noise and mechanical motion. Motor vibration, temperature drift, and sampling jitter are omitted.

Actuator parameters are estimates:

- Thrust-command delay: 20 ms, followed by a 50 ms response time constant.
- Servo play: ±0.5°, implemented by holding the target until the command takes up the slack. Load-driven movement inside the gear clearance is omitted.
- Servo PWM is 100 Hz (was 50 Hz until 2026-10-21). Servo command filter: 60 ms; stiffness: 5 N·m/rad; damping: 0.15 N·m·s/rad.
- Servo torque limit: ±0.8 N·m; travel: ±60°.
- Maximum thrust: approximately 11.22 N per motor, from the calibrated curve.
- Propeller reaction torque: ±0.015 N·m per newton of thrust, with opposite spins. The actual left/right spin assignment still needs confirmation.

The thrust sweeps contain settled readings after a one-second wait, so they do not identify motor response timing. Battery sag, aerodynamic drag, and propeller gyroscopic effects are omitted. Propeller geometry supplies approximate inertia and a visual outline. Ground contact uses simple component shapes and foot pads. The standalone XML viewer includes actuator lag; `sweep.py` adds command delay, backlash and sensor noise.

## Controller tuning

Roll P/damping are 20/6; pitch P/damping are 20/10. Yaw-rate damping is 4. Roll and pitch integral gains are 3, with correction limited to ±3 rad/s² and cleared when stopped. Pitch trim is −3.6°; left/right servo trims are +0.9°/−0.9°. These values were selected from simulated trajectories and need hardware checks.

Accelerometer correction has an approximately 1 s time constant while unpowered and 50 s while powered. Slow correction reduces the effect of horizontal acceleration being interpreted as tilt. Residual gyro bias can still cause drift. Collective returns to nominal after the takeoff pulses; the controller does not hold altitude or position.

## Geometry and mass

`bicopter.xml` specifies each component's mass. MuJoCo computes mesh inertia from solid geometry and scales it to the assigned mass. This approximates printed infill as uniformly distributed. The mass budget totals 900 g; COM height remains a prediction until it is measured on the assembled drone.

| Component                     |     Mass used | Source                                                       |
| ----------------------------- | ------------: | ------------------------------------------------------------ |
| One battery                   |         193 g | Components document; CAD envelope 74 × 36 × 35 mm below tube |
| Two motors                    |     57 g each | SunnySky X2212 V3, components document                       |
| Two servos                    |     55 g each | MG995, components document                                   |
| Two propellers                |     12 g each | Estimate; 9-inch diameter from CAD/components document       |
| Two cradles                   |  16.84 g each | CAD solid-PLA estimate                                       |
| Two fixed motor mounts        |     40 g each | Estimate from frame/support/saddle CAD volumes               |
| Electronics deck              |          24 g | Estimate; CAD solid-PLA equivalent 29.89 g                   |
| Two landing frames            |     35 g each | Estimate; current CAD solid-PLA equivalent 46.31 g each      |
| Two chassis side frames       |  16.56 g each | Recorded slicer estimate                                     |
| Hatch / bridge / camera mount | 17 / 14 / 8 g | Estimates from CAD volumes                                   |
| Carbon tube                   |          25 g | Estimate; 420 × 10.5 × 10 mm outside                         |
| Two ESCs                      |     25 g each | Estimate                                                     |
| Electronics / camera          |     55 / 10 g | Estimates including perfboard                                |
| Wiring and fasteners          |       39.20 g | Residual to match reported 900 g, placed near central deck   |


Sources: [components document](https://docs.google.com/document/d/1pgYeX1OccuEGhHezcuSxMdpLn0v6e36mq_4py6jGJlA/edit), `hw/drone_chassis/ASSEMBLY.md`, and `hw/drone_chassis/stl/side_frames_and_landing_legs.txt`.

CAD X maps to body Y; CAD −Y maps to body X. Motor axes are at body Y=±172 mm. Tilt pivots are 37.2 mm above the tube and propellers are 94 mm above it. Servo cases are at Y=±121.5 mm, Z=47.55 mm. The electronics deck replaces the old cap, so the combined bridge/cap STL is omitted.

The fixed mounts use connected components from `hw/end_attachment/stl/Servo mount.stl` (the October 8 print layout). Assembly coordinates were recovered as follows:

- Frame: unchanged.
- Saddle: translated `(0, 65, -14.4)` mm.
- Pivot support: translated `(0, -80, 5)` mm.
- Retainer: subtract `(120, 60, -26.3)` mm, then rotate +90° about Y.

The resulting meshes are `meshes/fixed_mount.stl` and `meshes/pivot_retainer.stl`. The moving cradle references `motor_cradle_m2.stl` directly. In those local CAD coordinates, the motor center is X=−6.5 mm and the pivot is at Z=32 mm. Mesh placement preserves the body-frame motor spacing. `flight.py` uses the resulting COM, pivot-height, and inertia estimates.

## Recorded simulation results — 2026-10-18

After integer ESC-command rounding, 60-second runs with seeds 1, 7, and 19 stayed upright. Maximum absolute roll/pitch after takeoff was approximately 5.21°, 5.19°, and 5.18°. Seed 7 ended near `[8.66, 29.13, 19.02]` m. These runs still climb and drift. The omitted aerodynamic drag limits the usefulness of long-distance trajectory predictions.

The older response fits used data collected before the blocking PWM updates were fixed. Their delay/lag pairs of 0.08/0.28 s, 0.10/0.24 s, and 0.16/0.12 s all caused the simulated drone to tip over. Those fits include the old software timing and rig dynamics; current motor response needs a separate measurement. The shared controller and CAD-mesh changes were checked without flashing firmware.

## Measured plant (2026-10-21)

`bicopter.xml` carries the bench deck (about 5 g, as boxes) and an explicit inertial on the drone body: the geoms place the parts, while mass and inertia are set so the vehicle matches measurements. The CoM is 17.2 mm above the tube (hang test with three string lines, 17.0 mm with the old deck, plus the lighter deck), roughly centred fore/aft, so the tilt pivots sit 20.0 mm above it. Pitch inertia is 0.00157 kg·m², from flight fits that showed pitch responding about 1.4x the controller's model (0.020 m pivot, 0.0023 kg·m²). Total mass is 881 g, estimated from the 900 g weighing minus the old deck; weigh it again to confirm. Other fitted plant values: thrust about 1.09x the table at 16 V (liftoff near throttle 620-650), the left motor about 6% weak (roll_bias 3.7 rad/s² holds it), servo dead time about 40 ms (bench, battery-powered servos) plus about 60 ms response. Servo play is still unmeasured. `sweep.py`'s NOMINAL plant uses the same values.

## Sweep plant (2026-10-22)

`sweep.py` runs a 6-DOF rigid body: thrust and servo tilt in the body frame, gravity, quadratic drag (0.03 N/(m/s)², a rough estimate), and four spring-damper feet at the CAD landing-leg contact points (x ±55 mm, y ±78 mm, 104 mm below the CoM; 2000 N/m and 30 N·s/m per foot, friction 0.5). The controller sees the firmware attitude estimator fed with simulated, noisy gyro and accelerometer readings instead of the true attitude. A plant's `ramp` raises collective to T over that many seconds on the ground. On the 2026-10-21 liftoff and free-flight sweeps this matches the earlier attitude-only plant to about 1° of peak pitch and the same stable/unstable cases: the feet cannot tip the drone before liftoff (rocking about a foot takes about 0.5 N·m; a 10 rad/s² pitch push is 0.016 N·m), and the estimator error is small next to the servo lag. Collective has no altitude hold, so after the climb pulse the drone keeps climbing or sinking; free flight starts at 50 m so the floor never ends a run.

## Airborne fit from the Oct 20–21 recordings (2026-10-23)

Per Bill, the Oct 20 08:25–11:17Z runs sat on the floor (legs touching, collective 700–710 at about 14.5 V is below liftoff on that pack); the Oct 21 02:xx and 06:xx runs were free. The 00:32Z files (no `sensor_to_body` in their header; the firmware then used [1, −1, −1]) never reach collective 600. `sw/fit_airborne.py` fits only the airborne windows, from liftoff to 35° or impact:

| Session | Effectiveness vs controller model | Delay | Steady pitch push | R² |
| --- | ---: | ---: | ---: | ---: |
| Oct 21 02:xx | 1.42x | 90 ms | −4.0 rad/s² (nose-up) | 0.71 |
| Oct 21 06:xx (new ESCs) | 1.04x | 130 ms | +2.8 rad/s² (nose-down) | 0.54 |

The sweep plant with the 06:xx values and the flown gains (pitch 10/10/5, `pitch_bias` +2.4, `velocity_damping` 8) pitches past 30° about 0.6 s after liftoff, matching the recorded tip-overs (21–35° within 0.4–0.6 s). With pitch 15/30/7 the peak pitch in the first second is 15–17° at bias −7, 4–5° at −3, and 13–16° at 0; on the 02:xx plant bias −7 gives 34–39°. The push changed sign between sessions, so bias −3 suits the current (06:xx) hardware and 0 is the safe middle.
