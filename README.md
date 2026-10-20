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

One laptop service, `src/fw/controller/bridge.py`, owns the controller's USB port (see [HOST.md](src/fw/controller/HOST.md)). It serves the dashboard, records flights and relays updates; nothing else talks to the drone. The drone side is native: `src/fw/native/runtime/radio.c` with `flight_rt.c`.

| Direct channel | Direction | Content |
| -------------- | --------- | ------- |
| 0 | drone | Status every 50 ms (`runtime.h` `Status`): IMU, attitude, outputs, targets, armed state, stop reason, last target sequence, recorder state, and the application's block (force, battery, settings version). It also carries the boot session and time token that make the link fresh. |
| 1 | laptop | Targets, sent immediately when controls change and every 50 ms while connected: collective, roll, pitch, yaw rate, and the servo angles and throttles used while disarmed. The newest wins; a late one is dropped. Each accepted packet is the controller heartbeat, so the native 500 ms watchdog stops the motors when they stop arriving. |
| 2, 3 | drone | The flight log: `LGS1` start, `FLG1` batches, `LGE1` stop, and the application's metadata JSON. One message is in flight at a time; it is resent until acknowledged. |
| 4, 5 | laptop | Recorder subscription (every second) and log acknowledgements. |
| 6 | both | Id-tagged requests with cached replies: `X` stop, `A` arm, `G` calibrate, `Z` zero a servo, `T` tune, `S` read settings, and the update ops. |

Stop preempts everything: the drone applies it as it arrives, even during another request, and the service repeats it every 50 ms until the drone answers. Arming needs an explicit `A` after any stop, a calibrated IMU, zero streamed collective, a subscribed recorder and the previous recording fully sent. Target packets never arm. After a stop, manual throttle stays at zero until the laptop sends zero throttle. A zeroed servo holds still until its streamed angle returns to zero.

The dashboard's `/ws` carries JSON: readings from the service, and `targets`, `stop`, `arm`, `calibrate`, `zero` and `tune` messages from the browser. Closing it stops the drone.

Each armed session becomes one recording, `src/sw/recordings/<time>_<session>.bin` and `.jsonl`, even across link hiccups. The first JSON line holds the session, calibration and metadata. Gaps in sample sequence, log stalls, resumed streams and the stop record are written as events. A bounded worker queue handles file writes and syncs them once per second without blocking control forwarding. The drone keeps 2 s of samples while the link stalls. Decode a recording with `uv run -m sw.flight_log sw/recordings/<flight>.bin`.

Sampling and control run at 100 Hz. Log batches of four samples go out every 40 ms. The rate and batch size are defined in `src/fw/drivers/log_format.py`.

IMU calibration averages two seconds of gyro and gravity readings, then saves gyro bias and the zero-pose reference in NVS. Startup restores calibration when its sensor coordinate frame matches the firmware. Arming requires saved calibration; the calibration button stays busy until saving finishes. Recordings include calibration validity, gyro bias, and `pose_zero_deg`.

## Tuning from a terminal

With the service running, local tools read status and change gains, trims, feedforward and model parameters directly. The drone validates them and saves them after edits pause.

```sh
curl localhost:8765/status
curl localhost:8765/settings
curl localhost:8765/schema
curl -X POST localhost:8765/settings -H 'Content-Type: application/json' -d '{"settings":{"pitch_p":45}}'
```

## Native math

The drone's tunable settings, defaults and ranges are defined in `src/fw/native/runtime/settings.def`. The host simulation configures its controller through `src/fw/drivers/flight.py`; both run the estimator/controller math in `src/fw/native/flight_native.c`.

The custom firmware runs MicroPython, Wi-Fi and the native radio on core 0. `src/fw/native/runtime/flight_rt.c` runs sensor reads, control, PWM, the 500 ms motor watchdog and binary log capture on core 1. It owns a bounded log queue. Native `settings.c` owns settings, servo offsets and calibration, committing them to NVS while disarmed. Python handles the load cell, battery ADC and one-time import of the old JSON settings.

Run these from `src/fw`:

| Change                     | Command                                                            |
| -------------------------- | ------------------------------------------------------------------ |
| Python over USB            | `make flash`                                                       |
| One file                   | `make upload FILES="main.py"`                                      |
| Python over the radio      | `make flash_direct`                                                |
| Native firmware over radio | `make flash_runtime_direct`                                        |
| Build native firmware only | `make ota_runtime`                                                 |

`flash` resets and opens the REPL; `upload` resets and exits. Both reuse one connection and skip unchanged files, and compile drivers to `.mpy` (removing a `.py` that would shadow it). USB uploads automatically select a port only when exactly one USB serial device is present. With both boards connected, set `ESP_PORT` to the drone's port explicitly. File updates preserve saved tuning.

Native builds use ESP-IDF 5.5.2 from `$IDF_PATH` or `~/esp/esp-idf`, reuse Ninja's build directory, and initialize submodules on the first build. Factory and OTA builds share a lock before changing their MicroPython checkout. Build dates are reproducible, so crossing midnight does not trigger recompilation.

Use `flash_runtime_direct` for native application updates. It selects the inactive OTA slot, preserves the filesystem and NVS, and verifies changed sectors. The old factory-address USB uploader has been retired because it could overwrite the wrong OTA slot.

Our C code is linked last in each flash segment and padded to a 4 KiB boundary (`src/fw/native/runtime/flight.lf`, archive `libzflight.a`), so MicroPython and ESP-IDF keep their addresses when it changes. A small edit to `flight_rt.c` or `flight_native.c` typically changes 3–7 sectors (app header, our code and rodata, image hash). Growing our code past a 4 KiB boundary, resizing static arrays (moves the heap start) or adding a qstr rewrites more, up to about 50 sectors; new qstrs and `MP_ERROR_TEXT` wording can renumber MicroPython-wide tables.

Python-only updates do not rebuild the runtime. For simulation, run `make -C src/fw/native host` and restart the notebook kernel after rebuilding.

### Optional native OTA layout

`make ota_runtime` builds a separate image in `src/fw/native/build-runtime-ota/`. The default `runtime` build keeps the existing factory partition layout. The OTA variant has two 1,900,544-byte application slots and a 320 KiB filesystem on the 4 MiB ESP32. Bootloader rollback is enabled.

`make flash_runtime_direct` uploads the native application to the inactive OTA slot. The new boot must be confirmed before the next restart; an unconfirmed boot rolls back to the previous application.

### Direct-link updates

With the service running, `make flash_runtime_direct` and `make flash_direct` (optionally `FILES=…`) update the drone over ESP-NOW through `ws://127.0.0.1:8765/update`. The service only adds fragmentation and pacing.

The drone's radio link and update service are native (`src/fw/native/runtime/radio.c`). The firmware starts them before `boot.py` and `main.py`, so the drone keeps reporting status, stops, and accepts firmware updates and resets even when the Python application is broken or has crashed. The controller MAC and channel are compiled into `radio.c`. Requests travel on direct channel 6. Each one carries a random id; the drone answers each id once and replays the reply to retries. The drone refuses every write and reset while the native runtime is armed or a throttle is nonzero, and does not touch the native watchdog.

- Firmware: the CLI keeps the last image it installed in each slot in `native/build-runtime-ota/installed/` and diffs against it, so the drone hashes nothing in bulk. The drone reports each slot's ELF SHA-256 prefix from its app description; an unknown or different slot counts as unknown. Sectors already right in the inactive slot are skipped, sectors the running image has are copied on the drone (checked against an 8-byte SHA-256 prefix), and only the rest cross the radio, each checked by hash and read back after writing. ESP-IDF then checks the whole image while selecting it. The drone restarts, and the CLI confirms the new slot once the drone answers over the link again. Without that confirmation, the next reset rolls back. Starting a write also confirms the running image, so the fallback slot is never the one being overwritten. After flashing a slot by another route, the next direct update to that slot sends it whole.
- Files: changed files only, written as `<path>.upload`, hash-checked, renamed, then a reset. The CLI removes a `.py` that would shadow an uploaded `.mpy`. The native service hands these requests to `src/fw/drivers/update.py`, so they need the Python application running. This also works on the factory layout.

Expect roughly 25 KB/s on the radio (about 150 ms per sector, including the flash write). A small C change (measured: 8 sectors sent, 3 copied) takes about 10 s to build and 3–4 s to install, restart and confirm. A full image (430 sectors) takes about 70 s.
