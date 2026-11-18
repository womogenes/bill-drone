# Firmware

The ESP32 runs one ESP-IDF C application in `drone/`: control loop, IMU, radio link, settings, recording, load cell and battery ADC.

[note to agents: do not modify this file. if you want to modify, ask user]

## Pinouts

**ESCs and servos**

- Left ESC: GPIO25
- Left servo: GPIO14
- Right ESC: GPIO 18
- Right servo: GPIO 16

**Sensors**

- ADC: GPIO35 (6th from top left)
- Load cell (HX711): DT GPIO32, SCK GPIO33
- IMU:
  - SDA: GPIO19
  - SCL: GPIO22
  - VCC: 3.3V
  - INT: GPIO21

**Camera**

- SCLK: GPIO27 (GPIO14 on CAM)
- MOSI: GPIO26 (GPIO13 on CAM)
- MISO: GPIO34 (GPIO12 on CAM)
- CS: GPIO23 (GPIO15 on CAM)
- READY: GPIO4 (GPIO2 on CAM)
- VCC: 6V
- GND: GND

## Updates

Run the [laptop service](controller/HOST.md), then build and update through the controller radio:

```sh
cd src/fw
make deploy
```

Over USB, stop any program using the drone's serial port and select that port explicitly:

```sh
make usb PORT=/dev/ttyUSB1
```

USB device names can change; identify the drone before flashing. Updates preserve NVS, which holds tuning, servo offsets and IMU calibration. Skip backups and whole-image verification. See the [repository README](../../README.md) for the protocol and build details.

## Radio through the camera

The drone's own antenna sits under the GPS and loses 15-25 dB. With the setting `radio_via_camera` at 1 the drone's ESP-NOW link goes through the S3 camera's radio and u.FL dipole instead: the drone keeps its own Wi-Fi off and sends and receives over its wire to the camera (`relay_link.h`: `CRL1` frames with a length and CRC-16 beside the TAG3 packets and update traffic), and the camera's station takes the drone's MAC on channel 6, so the controller and laptop need no change. Send outcomes come back from the camera into the drone's link counts (LNK1 tx_ok, tx_fail, latency) and each packet heard comes with the camera's RSSI and noise floor, so the channel 7 link reports, which travel through the relay too, describe the camera's radio; their flags gain 4 in relay mode, and radio_rate still picks the rate (radio_power does not apply: the camera keeps its own power, K T). Following channels (radio_follow) and adopting another controller are off in relay mode, as the camera stays on channel 6 and hears only the paired controller; the fallback below brings them back. a camera that stops answering frees it after 100 ms, and its silence ends the link like any other link loss (the link-loss landing). In relay mode the camera joins no network and refuses FPV, the radio preview and its access point (K H, A, V, P, Q), which would leave channel 6 or take its airtime.

Both modes stay in both images; the setting chooses at boot and either mode can be reached from the other over the link in use. At every boot the drone tells the camera its mode (K R), and the camera keeps it in flash and restarts into a changed one, with the drone's MAC for the relay or its own MAC and Wi-Fi otherwise. To switch, with the drone disarmed: tune `radio_via_camera` to 1 (or 0) from the dashboard or a T request, then restart the drone (R). The drone comes back in the new mode within a few seconds and the camera follows about a second later. The I reply's `radio` says `camera` or `own`; the camera's K S begins with `relay` in relay mode.

Failsafes: in relay mode a disarmed drone that hears nothing from the controller for 30 s tells the camera to leave relay mode and brings up its own radio until the next restart (the setting stays 1, so a power cycle tries the relay again); a camera in relay mode that gets no frames from the drone for 20 s drops the mode and restarts with its own MAC, so the two boards never transmit with one MAC for long. Both images must carry the relay (drone and camera from the same commit) before the setting is turned on; an older camera refuses K R, and in relay mode the drone then falls back after 30 s.
