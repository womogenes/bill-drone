# Bicopter simulation

## Code and operation

`sw.sim.slop.simulation.run()` calls the controller at 100 Hz and renders a 10 FPS
GIF. The notebook runs a timed takeoff sequence. The firmware takes collective
throttle from the dashboard.

Firmware and simulation use `fw/drivers/flight.py` for attitude estimation and
motor commands, and `calibration.py` for thrust conversion and gyro calibration.
The controller receives accelerometer and gyro readings. It uses fixed geometry
and inertia estimates; position and altitude are recorded for evaluation.

`control.py` manages settings, shutdown conditions, and PWM output. `imu.py` reads
the MPU6050. Tuning is saved in `flight-tuning.json`; the older `control-gains.json`
has different gain units and is ignored. Flight trim and physical servo-center
offsets in `actuators.py` are separate adjustments to check on the assembled drone.

## Thrust calibration

The thrust table uses three October 4 sweeps in `sw/slop/`, calibrated with the
1.62 kg bottle comparison. `compare.py` excludes the two confirmed glitches at
throttle 750. It calculates a median at each throttle for each run; the shared
table uses the median across runs and interpolates between those values.

Throttle 200 is treated as approximately zero thrust, based on the bench
observation. The curve retains the measured decline above 1100. Commands below
200 produce zero modeled thrust; commands above 1200 hold the last table value.
Converting thrust back to throttle uses the rising branch, capped at 1100.
Both motors use the same curve. Battery voltage and propeller configuration were
not recorded in those CSVs.

## Sensor and actuator response

The MPU settings match `fw/drivers/imu.py`: 100 Hz sampling, ±2 g acceleration,
±250°/s gyro range, and DLPF setting 3 (44 Hz accel / 42 Hz gyro bandwidth).
The [InvenSense specification, sections 6.1–6.2](https://www.haoyuelectronics.com/Attachment/GY-521/mpu6050.pdf)
gives noise densities of 0.005°/s/√Hz for the gyro and 400 µg/√Hz for acceleration.
The simulation estimates RMS noise as density × √bandwidth and approximates
sensor latency with a 5 ms filter. It also applies ADC quantization and clipping.

Gyro bias is `[0.02, 1.10, -1.95]` °/s, from an earlier stationary dashboard
observation. The simulation averages the first 200 stationary samples to estimate
that bias. Accelerometer offsets are zero because tilt and bias were not measured
independently. Powered logs contain both sensor noise and mechanical motion.
Motor vibration, temperature drift, and sampling jitter are omitted.

Actuator parameters are estimates:

- Thrust-command delay: 20 ms, followed by a 50 ms response time constant.
- Servo play: ±0.5°, implemented by holding the target until the command takes up
  the slack. Load-driven movement inside the gear clearance is omitted.
- Servo command filter: 60 ms; stiffness: 5 N·m/rad; damping: 0.15 N·m·s/rad.
- Servo torque limit: ±0.8 N·m; travel: ±60°.
- Maximum thrust: approximately 11.22 N per motor, from the calibrated curve.
- Propeller reaction torque: ±0.015 N·m per newton of thrust, with opposite spins.
  The actual left/right spin assignment still needs confirmation.

The thrust sweeps contain settled readings after a one-second wait, so they do
not identify motor response timing. Battery sag, aerodynamic drag, and propeller
gyroscopic effects are omitted. Propeller geometry supplies approximate inertia
and a visual outline. Ground contact uses simple component shapes and foot pads.
The standalone XML viewer includes actuator lag; `simulation.py` adds command
delay, backlash, sensor noise, and gyro calibration.

## Controller tuning

Roll P/damping are 20/6; pitch P/damping are 20/10. Yaw-rate damping is 4.
Roll and pitch integral gains are 3, with correction limited to ±3 rad/s² and
cleared when stopped. Pitch trim is −3.6°; left/right servo trims are +0.9°/−0.9°.
These values were selected from simulated trajectories and need hardware checks.

Accelerometer correction has an approximately 1 s time constant while unpowered
and 50 s while powered. Slow correction reduces the effect of horizontal
acceleration being interpreted as tilt. Residual gyro bias can still cause drift.
Collective returns to nominal after the takeoff pulses; the controller does not
hold altitude or position.

## Geometry and mass

`bicopter.xml` specifies each component's mass. MuJoCo computes mesh inertia from
solid geometry and scales it to the assigned mass. This approximates printed
infill as uniformly distributed. The mass budget totals 900 g; COM height remains
a prediction until it is measured on the assembled drone.

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


Sources: [components document](https://docs.google.com/document/d/1pgYeX1OccuEGhHezcuSxMdpLn0v6e36mq_4py6jGJlA/edit),
`hw/drone_chassis/ASSEMBLY.md`, and `hw/drone_chassis/stl/side_frames_and_landing_legs.txt`.

CAD X maps to body Y; CAD −Y maps to body X. Motor axes are at body Y=±172 mm.
Tilt pivots are 37.2 mm above the tube and propellers are 94 mm above it.
Servo cases are at Y=±121.5 mm, Z=47.55 mm. The electronics deck replaces the old
cap, so the combined bridge/cap STL is omitted.

The fixed mounts use connected components from `hw/end_attachment/stl/Servo mount.stl`
(the October 8 print layout). Assembly coordinates were recovered as follows:

- Frame: unchanged.
- Saddle: translated `(0, 65, -14.4)` mm.
- Pivot support: translated `(0, -80, 5)` mm.
- Retainer: subtract `(120, 60, -26.3)` mm, then rotate +90° about Y.

The resulting meshes are `meshes/fixed_mount.stl` and `meshes/pivot_retainer.stl`.
The moving cradle references `motor_cradle_m2.stl` directly. In those local CAD
coordinates, the motor center is X=−6.5 mm and the pivot is at Z=32 mm. Mesh
placement preserves the body-frame motor spacing. `flight.py` uses the resulting
COM, pivot-height, and inertia estimates.

## Recorded simulation results — 2026-10-18

After integer ESC-command rounding, 60-second runs with seeds 1, 7, and 19 stayed
upright. Maximum absolute roll/pitch after takeoff was approximately 5.21°, 5.19°,
and 5.18°. Seed 7 ended near `[8.66, 29.13, 19.02]` m. These runs still climb and
drift. The omitted aerodynamic drag limits the usefulness of long-distance
trajectory predictions.

The older response fits used data collected before the blocking PWM updates were
fixed. Their delay/lag pairs of 0.08/0.28 s, 0.10/0.24 s, and 0.16/0.12 s all caused
the simulated drone to tip over. Those fits include the old software timing and
rig dynamics; current motor response needs a separate measurement. The shared
controller and CAD-mesh changes were checked without flashing firmware.
