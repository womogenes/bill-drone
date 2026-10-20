# Firmware

The ESP32 runs the control loop, IMU sampling and radio link in native C. MicroPython handles the remaining application code, including the load cell, battery ADC and file updates.

[note to agents: do not modify this file. if you want to modify, ask user]

## Pinouts

**ESCs and servos**

- Left ESC: GPIO25
- Left servo: GPIO14
- Right ESC: GPIO 18
- Right servo: GPIO 16

**Sensors**

- ADC: GPIO35 (6th from top left)
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

## Updates

Run the [laptop service](controller/HOST.md), then update through the controller radio:

```sh
cd src/fw
make flash_direct          # changed Python files, then restart
make flash_runtime_direct  # build and update the native OTA application
```

For Python updates over USB, stop any program using the drone's serial port and select that port explicitly:

```sh
make upload ESP_PORT=/dev/ttyUSB1
# Or one file:
make upload ESP_PORT=/dev/ttyUSB1 FILES="main.py"
```

USB device names can change; identify the drone before uploading. Both upload paths compile drivers to bytecode and check changed files. Native updates preserve the filesystem and NVS, which holds tuning, servo offsets and IMU calibration. Skip backups and whole-image verification.

The drone requires the custom runtime built in `native/runtime/`. The stock MicroPython image lacks its control, radio and settings modules. See the [repository README](../../README.md) for the protocol, build process and one-time factory-to-OTA migration.
