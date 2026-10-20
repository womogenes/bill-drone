# ESP32-CAM AprilTag pose

One firmware handles onboard tag16h5 pose estimation, SPI delivery, Wi-Fi preview, lens calibration, and passwordless OTA. It targets the AI Thinker ESP32-CAM with a driver-detected sensor (OV3660 on this board), 4 MB flash, and PSRAM. The flight controller runs separate MicroPython firmware.

## Wiring

| Signal | ESP32-CAM | Main ESP32 | Direction |
|---|---:|---:|---|
| SCLK | GPIO14 | GPIO27 | Main → CAM |
| MOSI | GPIO13 | GPIO26 | Main → CAM |
| MISO | GPIO12 | GPIO34 | CAM → main |
| CS | GPIO15 | GPIO23 | Main → CAM |
| READY | GPIO2 | GPIO4 | CAM → main |

Signals are 3.3 V; share ground through the power wiring. Leave the camera's microSD slot unused. Main GPIO34 must remain an input without pull-up, preserving camera GPIO12's low boot strap. Main GPIO4 uses a pull-down, keeping camera GPIO2 low for serial flashing. CS idles high. Keep the signal wires short.

## Flash and update

Copy `src/wifi.example.h` to git-ignored `src/wifi.h` and set network credentials. The local header has been populated from the repository `.env`. OTA has no password. From `src/`, build and initially flash **the camera's verified programmer port**:

```sh
uvx --from platformio==6.1.18 pio run -d fw/esp32cam
uvx --from platformio==6.1.18 pio run -d fw/esp32cam -t upload --upload-port /dev/ttyUSB_CAM
```

Ground camera GPIO0 and reset/power-cycle for serial flashing; release GPIO0 and reset afterward. Identify the board before selecting a port: USB numbers can move, and these two adapters have identical USB serial numbers. The CAM identified on 2026-10-19 has MAC `58:2A:BD:79:2B:9C`, physical path `/dev/serial/by-path/pci-0000:c1:00.3-usb-0:1:1.0-port0` (then USB1). The main controller was USB0 and printed `Starting app...`. Reconfirm if wiring changes.

Subsequent firmware updates:

```sh
uvx --from platformio==6.1.18 pio run -d fw/esp32cam -e ota -t upload
```

Hostname: `bill-esp32cam.local`; override with `--upload-port <camera-ip>` when mDNS is unavailable. The first flash installs two 1.25 MiB OTA slots. Updates pause processing and reboot the CAM; update while the flight controller is disarmed.

## Dashboard calibration and preview

Open **Camera** (`/camera`) in the dashboard and connect. Enter the printed **tag16h5 ID (0–29)** and **outer black-square width in millimetres**, excluding the white margin. Capture **25–40 sharp views of the same flat tag**, varying both tilt axes, distance, and position across the image while keeping all corners visible. Press Space or click Capture view; Space is ignored while typing in inputs. A single tag has only four corners, so varied views are especially important.

**Calibrate and save** runs `calibration.py` with `src/.venv/bin/python`, fits the lens, writes `src/config.h`, and sends the calibration to the CAM's `/config` endpoint. The CAM applies it immediately and stores it in NVS across reboots. Saved NVS calibration takes precedence over compiled defaults. No reflashing is needed to calibrate. A failed camera connection leaves the local fit saved, but the dashboard reports failure until it is applied to the board.

The fit uses focal lengths, principal point, and two radial distortion coefficients; tangential distortion and k3 are fixed to zero because a single tag supplies few constraints. Reprojection RMS must be below 1 px; a small RMS alone does not prove accuracy. Check measured distances after calibration and recalibrate after changing focus, lens, resolution, crop, or orientation. Both detection and preview use the same upright 320×240 hardware-JPEG capture, with hmirror=0 and vflip=1. These settings rotate the previous image 180 degrees. The dashboard applies no additional display flip. Calibration is stored separately for this orientation; recapture views after switching to it. The detector decodes JPEG into grayscale; calibration uses the same decoded image geometry.

Preview is MJPEG on camera port 81, proxied through the dashboard. It targets up to 24 fps. After processing a frame, the detector offers its compressed JPEG to a single latest-frame slot using a try-lock; it never waits for video. A low-priority task on the other core sends it without re-encoding. Slow viewers drop frames. Without a viewer, no preview copying or streaming runs. The dashboard shows actual preview fps and detector Hz. Once calibrated, onboard pose processing runs on every captured frame; its rate takes priority over reaching 24 preview fps. SPI reception can be absent without stalling capture, detection, preview, or OTA.

## MicroPython integration

Copy `receiver.py` to the main controller as `drivers/tag_pose.py`. Add:

```python
from machine import Pin, SPI
from drivers.tag_pose import TagPose

cs = Pin(23, Pin.OUT, value=1)
spi = SPI(1, baudrate=4_000_000, polarity=0, phase=1,
          sck=Pin(27), mosi=Pin(26), miso=Pin(34))
tag_pose = TagPose(spi, cs, Pin(4, Pin.IN, Pin.PULL_DOWN), tag_id=0)

async def sample_tag():
    while True:
        tag_pose.poll()
        await asyncio.sleep_ms(10)
```

Add `sample_tag()` to the existing asyncio gather. Set its tag ID to match the camera. Parsing stays outside the control callback; control reads `tag_pose.latest`, a dictionary or `None`, and uses each sequence once. It clears on loss, CRC errors, stale data, or repeated sequence numbers. Startup packets already waiting on READY and packets after polling stalls are discarded because their queue age is unknown. Expiry is checked even when polling stops.

## Coordinate frames and packet

`point_tag = rotation * point_camera + position_m`. Position is the camera origin in metres in the tag frame. Rotation is a row-major 3×3 matrix, camera → tag. Camera axes are image-right, image-down, and through the lens. Tag axes are right, down, and into the printed tag in its canonical orientation. A camera facing the tag has negative tag-frame Z. Measure the camera mounting transform before body pose hold: `T_tag_body = T_tag_camera * inverse(T_body_camera)`.

SPI mode 1, MSB-first, 4 MHz, exactly 80 bytes per READY. Packet fields are little-endian integers and float32; Python format `<4sIIIHH14fHH`: magic `TAG1`, sequence, capture time (µs), age (µs), tag ID, valid flag, position[3], rotation[9], decode margin, reprojection RMS, reserved zero, CRC-16/CCITT-FALSE over 78 bytes. Capture time is the driver's first-DMA timestamp on the CAM clock. The receiver adds polling delay and transport time; both sides default to 250 ms expiry.

`main.cpp` owns capture/detection/SPI; `pose.h` owns geometry; `web.h` owns preview and calibration persistence; `ota.h` owns Wi-Fi/OTA. `build.py` places the library's pose solver in flash to fit Wi-Fi and camera interrupt code in IRAM. Vendor code is pinned and unchanged. Bench-test calibrated accuracy and main-controller loop timing before using vision for pose hold.
