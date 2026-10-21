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
