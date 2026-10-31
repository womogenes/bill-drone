# Downward rangefinder plan

Why: in Free mode (hover_camera 0) the height is integrated vertical acceleration, and today it drifted about 1.1 m in 10 s. The sim study of 2026-10-31 shows forward-camera optical flow only bounds the climb rate: with the current firmware the best flow settings still leave about 0.2 m RMS height error at 2-3 m scenes and 0.4 m outdoors, and the drone touched the floor in most 30 s runs at hover_climb 0.3. A downward range measures the height itself, so the error stops growing.

## Part choice

Figures are approximate datasheet values; check the current datasheet before buying.

| Part | Range indoors | Range in sunlight | Field of view | Rate | Bus | Supply | Mass | Price |
|---|---|---|---|---|---|---|---|---|
| ST VL53L1X (breakout) | 4 m long mode, 1.3 m short mode (dark) | about 0.7 m long mode, 1.3 m short mode at the datasheet's "strong ambient"; direct sun is brighter, so plan on under 1 m | 27 deg, ROI down to about 15 deg | up to 50 Hz (20-33 ms budget) | I2C 0x29 | 3.3 V, about 20 mA | about 0.5-1 g | 5-15 USD |
| ST VL53L4CD | 1.3 m | about 1 m (short range, good ambient immunity) | 18 deg | up to 100 Hz | I2C 0x29 | 3.3 V | about 0.5-1 g | 5-15 USD |
| Benewake TFmini-S | 12 m (90 % reflectivity), 4 m (10 %) | about 7 m on bright ground in strong sun | 2 deg | 1-1000 Hz (100 default) | UART 115200 (or I2C) | 5 V, about 140 mA | 5 g | about 40 USD |
| Flow + ToF module (MicoAir MTF-01 class) | about 8 m | worse than TFmini, better than VL53L1X | narrow ToF plus a downward flow camera | 50-100 Hz | UART | 5 V | about 3-5 g | 20-30 USD |

Recommendation: VL53L1X in short mode for now (all flying so far is indoors at 0.3-1 m): it is the cheapest and lightest, its 27 deg cone averages over floor texture and is forgiving of tilt, and 1.3 m covers the hover heights in use. VL53L4CD has the same footprint and bus but only 1.3 m in the dark, so it gains nothing here. For outdoors buy a TFmini-S: its 2 deg beam reaches about 7 m in sun, at the cost of 5 V power and a 10 cm blind zone (it reads invalid on the floor). If the forward drift is the bigger problem, the downward flow + ToF module is worth the extra integration: a downward flow sensor sees forward motion, which the forward camera cannot (the study shows 0.7 m/s RMS forward velocity error with or without camera flow).

## Wiring

Pins in use (fw/drone/main): servos and ESCs 14, 25, 16, 18 (flight_rt.c:63); MPU6050 I2C SDA 19, SCL 22 at 400 kHz on I2C_NUM_0 (flight_rt.c:22-23, 1004-1008); camera UART2 RX 34, TX 27, SYNC 13 (camera.c:17-20), camera IO0 4 and EN 23 (camera.c:23-24); HX711 32, 33 and LED 2 (main.c:17-20); battery ADC 35. camera.c:298 lists the free pins as 4, 5, 12, 15, 17, 21, 23, 26; without 4 and 23 that leaves 5, 12, 15, 17, 21, 26. Avoid 12 (strapping pin for flash voltage: a pull-up there can stop the board booting); 5 and 15 are strapping pins too but their I2C pull-ups would hold them at their default high level.

Sharing the IMU bus works electrically: the VL53L1X is at 0x29 and the MPU6050 at 0x68. The tick's MPU reads (15-byte read, FIFO count, about 10 samples of 12 bytes) take about 3.5 ms of the 10 ms tick at 400 kHz; a VL53L1X read (data-ready poll, status, range, clear interrupt, about 30 bytes) adds about 0.8 ms at 50 Hz, done in runtime_task right after read_fifo (flight_rt.c:368) so nothing contends for the bus. It is not recommended, though: a sensor on a lead to the underside that hangs SDA would take down the attitude IMU in flight, and flight_rt.c already needs a driver reinstall to recover the MPU from one I2C timeout (flight_rt.c:1045-1048). Better is the ESP32's second controller: I2C_NUM_1 with SDA on GPIO 21 and SCL on GPIO 26, optional XSHUT on 17 and the data-ready interrupt (GPIO1) on 5, read by a core-0 task that publishes the newest reading under a lock, as camera_flow does for the flow (camera.c:274). For a TFmini-S use UART1 (UART0 is the console, UART2 the camera) with RX on 26 and TX on 21, powered from the 5 V BEC; its 3.3 V TTL output is safe for the ESP32.

Mount it on the underside, pointing straight down along body z, clear of the prop wash and the body's resting contact points, and note its offset from the CoM (x, y, z) for the tilt compensation. The VL53L1X needs a clear window (no tape or film over it) or a cover-glass calibration.

## Firmware integration

Input: a new `HoverRange` (capture_us at the middle of the timing budget, distance along the sensor axis in m, valid flag) passed to hover_update next to the flow (hover.h:116-118, the call at flight_rt.c:508, sw/sim/hover.py update, and a sim plant["range"] with noise, rate, latency and maximum range in sw/sim/sweep.py).

Reference: in hover_rest (hover.c:136) learn the resting range (the sensor's height above the floor on the ground) as the resting specific force is learned, so height 0 stays the engage point.

Measurement: height_measured = distance * cos(roll) * cos(pitch) - rest_range, plus the lever-arm correction for the sensor's offset from the CoM (x sin(pitch) - y sin(roll), a few mm per 10 degrees). Compare it with the height at capture time, not now: add the height to the history ring (hover.c:312-319, a 7th entry) and look it up with recorded() as the tag path does.

Correction: where the IMU integrates the climb rate (hover.c:354) and before the camera's height correction (hover.c:355-367), a critically damped three-state complementary filter on the innovation e = height_measured - height_then: height += 3 e dt / tau, climb_rate += 3 e dt / tau^2, and a new vertical acceleration bias += e dt / tau^3 that the integration at hover.c:354 subtracts, with tau about 0.3 s (setting hover_range_tau). The learned bias is what keeps the height good through dropouts. While the range is valid, skip the climb-rate leak (hover.c:368-371) and let the range override the tag's height (the tag's height errs 3.5 cm per degree of attitude error, hover.c:31).

Out of range: reject a reading whose status is not OK (sigma, signal or wrap-around fail on the VL53L1X), under 0.04 m or over the maximum (1.2 m short mode, 3 m long mode indoors, 7 m TFmini-S), older than 100 ms, with tilt over 20 degrees (the cone hits the floor off axis), or with |e| over 0.3 m. A run of rejected |e| outliers lasting over 0.5 s is a new floor level (a table, a step): re-reference rest_range by the jump instead of flying it. With no valid range, fall back to the IMU (with the learned bias) and the flow as now. In hold, clamp the height target below the sensor's maximum range minus 0.2 m so the drone does not climb out of it.

Landing: a valid range within 2 cm of rest_range is extra touchdown evidence; the probe already has the hook (LOW_HEIGHT, hover.c:77 and 446, now 0), which could be set from the range instead.

Settings and log: hover_range (0 off, 1 on), hover_range_tau and hover_range_max in settings.def; the range, its status and the learned vertical bias in the recorded sample (RECORD_SIZE 80 and sample_format change, so the recording reader needs the new field).

## Rough effort

Hardware (breakout, four wires to GPIO 21 and 26, a printed mount): 1-2 h. VL53L1X driver: port ST's ULD API (BSD licence; four I2C read/write functions to supply) on I2C_NUM_1 with a core-0 read task, 3-4 h (a TFmini-S UART frame parser instead: 1-2 h). hover.c fusion, history, settings, log fields: 4-6 h. Sim range model and a Free-mode test with flight_accel_bias: 2-3 h. Bench checks (tilt compensation on a level, floor materials, the rest reference) and 2-3 test flights: half a day. About 2-3 days in all.
