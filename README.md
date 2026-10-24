# Drone

We're building a bicopter!

Still in the very early stages of this, but hope to get results quickly.

## Links

- Live main doc (components, etc). Plese read this to get a sense of what components we are using, before claiming you don't have enough information on our components list. Doc: https://docs.google.com/document/d/1pgYeX1OccuEGhHezcuSxMdpLn0v6e36mq_4py6jGJlA/edit?tab=t.0

## Notes

- We are willing to do a lot of integration and coding. Do not say "this is impossible / potentially possible" --- if it's possible with engineering work, it's possible. Don't ask for photos of things if you can figure it out with enough thinking/research.
- We are bold. A 3.7V battery will definitely power an ESP32-CAM unless experiments show otherwise.
- When you give `uv run` commands, assume we're in `src/`. So, for example, `uv run --project src python src/sw/sensors/camera_readings.py` is too verbose. Instead, just say `uv run sw/sensors/camera_readings.py`.
- Do not keep tests checked in.
- We are lean. We are scrappy. We should keep the absolute minimum amount possible in this repo that is required to do a thing. Do not maintain backwards compatability. Do not write defensive code; prefer to fail loudly. For example, the guard clause here:

```py
def get_frame():
"""
Return a single cv2 frame from the camera
"""
ok, frame = camera.read()
if not ok:
    raise RuntimeError("Could not read a frame from go2rtc")
return frame
```

is a bad pattern and must be avoided. We should let it fail loudly.

## Code style

- Python docstrings should have a new line after the first """ and before the last """
- Do not column-wrap in Markdown

## Project structure

- `src/` contains software and firmware.
- `src/lib/` contains external software and firmware. This includes things like ESC control etc. These should largely be unmodified.
- `src/sw` contains software and firmware.
- `src/hw` is empty right now but will contain things like PCB schematics etc.

## Local setup

Static laptop is streaming go2rtc over port 8555. On my tailnet, that's `http://100.64.0.6:1984/`.

## Drone link

One laptop service, `src/fw/controller/bridge.py`, owns the controller's USB port (see [HOST.md](src/fw/controller/HOST.md)). It serves the dashboard, records flights and relays updates; nothing else talks to the drone. The drone runs one ESP-IDF C application, `src/fw/drone/`: `main/radio.c` for the link and `main/flight_rt.c` for control.

| Direct channel | Direction | Content |
| -------------- | --------- | ------- |
| 0 | drone | Status every 50 ms (`runtime.h` `Status`): IMU, attitude, outputs, targets, armed state, stop reason, last target sequence, recorder state, load cell, battery and settings version. It also carries the boot session and time token that make the link fresh. |
| 1 | laptop | Targets, sent immediately when controls change and every 50 ms while connected: collective, roll, pitch, yaw rate, and the servo angles and throttles used while disarmed. The newest wins; a late one is dropped. Each accepted packet is the controller heartbeat, so the native 500 ms watchdog stops the motors when they stop arriving. |
| 2, 3 | drone | The flight log: `LGS1` start, `FLG1` batches, `LGE1` stop, and the metadata JSON. One message is in flight at a time; it is resent until acknowledged. |
| 4, 5 | laptop | Recorder subscription (every second) and log acknowledgements. |
| 6 | both | Id-tagged requests with cached replies: `X` stop, `A` arm, `G` calibrate, `L` level reference, `Q` accel correction, `Z` zero a servo, `T` tune, `S` read settings, and the update ops. |

Stop preempts everything: the drone applies it as it arrives, even during another request, and the service repeats it every 50 ms until the drone answers. Arming needs an explicit `A` after any stop, a calibrated IMU, zero streamed collective, a subscribed recorder and the previous recording fully sent. Target packets never arm. After a stop, manual throttle stays at zero until the laptop sends zero throttle. A zeroed servo holds still until its streamed angle returns to zero.

The dashboard's `/ws` carries JSON: readings from the service, and `targets`, `stop`, `arm`, `calibrate`, `zero` and `tune` messages from the browser. Closing it stops the drone.

Each armed session becomes one recording, `src/sw/recordings/<time>_<session>.bin` and `.jsonl`, even across link hiccups. The first JSON line holds the session, calibration and metadata. Gaps in sample sequence, log stalls, resumed streams and the stop record are written as events. A bounded worker queue handles file writes and syncs them once per second without blocking control forwarding. The drone keeps 2 s of samples while the link stalls. Decode a recording with `uv run -m sw.flight_log sw/recordings/<flight>.bin`.

Sampling and control run at 100 Hz. Log batches of four samples go out every 40 ms. The rate and batch size are defined in `src/fw/drone/main/runtime.h`; the laptop decodes records with `src/fw/controller/log_format.py`.

IMU calibration averages two seconds of gyro readings, then saves the gyro bias in NVS; gravity re-seeds the attitude estimate. The level reference (`pose_zero`) is set separately with `make level` from two readings 180° apart on the same spot, which cancels floor tilt. Startup restores calibration when its sensor coordinate frame matches the firmware. Arming requires saved calibration; the calibration button stays busy until saving finishes. Recordings include calibration validity, gyro bias, and `pose_zero_deg`.

## Tuning from a terminal

With the service running, local tools read status and change gains, trims, feedforward and model parameters directly. The drone validates them and saves them after edits pause.

```sh
curl localhost:8765/status
curl localhost:8765/settings
curl localhost:8765/schema
curl -X POST localhost:8765/settings -H 'Content-Type: application/json' -d '{"settings":{"pitch_p":45}}'
```

## Firmware

The drone's tunable settings, defaults and ranges are defined in `src/fw/drone/main/settings.def`. The estimator, controller and thrust table are in `src/fw/drone/main/flight_math.c`; the host simulation runs the same C through `src/sw/sim/flight.py`, which compiles it on first use.

`app_main` (`main/main.c`) loads the settings from NVS, sets the outputs to zero throttle and centred servos, starts the radio and then the control task. Wi-Fi, the radio and the update service run on core 0. `main/flight_rt.c` runs sensor reads, control, PWM, the 500 ms motor watchdog and binary log capture on core 1. It owns a bounded log queue. `main/settings.c` owns settings, servo offsets and calibration, committing them to NVS while disarmed. `main/camera.c` receives AprilTag poses and image flow from the ESP32-CAM over UART with a SYNC line (`src/fw/esp32cam/README.md`); it is built but not started, so nothing uses vision yet. The main loop reads the load cell and battery every 50 ms and refreshes the recording metadata when the settings change.

Run these from `src/fw`:

| Change | Command |
| --- | --- |
| Build | `make` |
| Install over the radio | `make deploy` |
| USB recovery | `make usb PORT=/dev/ttyUSB1` |

Builds use ESP-IDF 5.5.2 from `$IDF_PATH` or `~/esp/esp-idf`. The partition table has two 1,900,544-byte application slots, NVS and an unused 320 KiB data partition on the 4 MiB ESP32. Bootloader rollback is enabled.

Our code is linked last in each flash segment and padded to a 4 KiB boundary (`src/fw/drone/main/flight.lf`), with its constants kept out of the shared string pool, so ESP-IDF keeps its addresses when our code changes. A small edit typically changes a handful of sectors (app header, our code and rodata, image hash); growing our code past a 4 KiB boundary rewrites more.

`make usb` is for recovery only: it writes the image to `ota_0` and resets the OTA selection to it, leaving the bootloader and NVS alone. Opening the drone's serial port resets it.

### Radio updates

With the service running, `make deploy` updates the drone over ESP-NOW through `ws://127.0.0.1:8765/update` (`src/fw/tools/ota_direct.py`). The service only adds fragmentation and pacing. The controller MAC and channel are compiled into `radio.c`. Requests travel on direct channel 6. Each one carries a random id; the drone answers each id once and replays the reply to retries. The drone refuses every write and reset while armed or with a throttle nonzero, holds the outputs at zero during flash writes, and does not touch the native watchdog.

The CLI keeps the last image it installed in each slot in `src/fw/drone/installed/` and diffs against it, so the drone hashes nothing in bulk. The drone reports each slot's ELF SHA-256 prefix from its app description; an unknown or different slot counts as unknown. Sectors already right in the inactive slot are skipped, sectors the running image has are copied on the drone (checked against an 8-byte SHA-256 prefix), and only the rest cross the radio, each checked by hash and read back after writing. ESP-IDF then checks the image while selecting it. The drone restarts, and the CLI confirms the new slot once the drone answers over the link again. An image still unconfirmed 90 s after boot restarts while disarmed, and the bootloader rolls back to the previous one. Starting a write also confirms the running image, so the fallback slot is never the one being overwritten. After flashing a slot by another route, the next radio update to that slot sends it whole.

Measured on the drone: the first image (173 sectors, all sent) installed, restarted and confirmed in 28 s, about 6 KB/s with flash writes. A one-line C change (5 sectors sent, 168 copied) took 20 s including the build, 14.5 s of it to install, restart and confirm.
