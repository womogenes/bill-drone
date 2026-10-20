# USB radio controller

The controller ESP32 runs the ESP-IDF relay in `native/`. It forwards packets between USB serial at 921600 baud and the drone over ESP-NOW on Wi-Fi channel 6. The laptop supplies commands and heartbeat freshness.

## Build and install

Stop the laptop service before opening the controller's USB port. Identify the controller explicitly; both boards use CP2102 adapters and their `/dev/ttyUSBn` names can change.

```sh
# From the repository root:
make -C src/fw/controller/native build
make -C src/fw/controller/native flash PORT=/dev/ttyUSB0
```

The build uses ESP-IDF from `$IDF_PATH` or `~/esp/esp-idf`. Flashing writes the relay application at `0x10000` and checks the written data. It requires the existing factory bootloader and partition table on the controller. It is not a drone firmware update or a fresh-board provisioning command.

Start the [laptop service](HOST.md) after installation. The controller exposes no MicroPython REPL.

## Pairing

Both endpoints use fixed station MAC addresses and channel 6:

| Setting | Source |
| --- | --- |
| Drone station MAC | `native/main/relay.c`: `drone_mac` |
| Controller station MAC | `../native/runtime/radio.c`: `peer` |
| Controller radio channel | `native/main/relay.c`: `CHANNEL` |
| Drone radio channel | `../native/runtime/radio.c`: `WIFI_CHANNEL` |

Replacing either board requires updating its peer's MAC and rebuilding that peer's firmware. Keep the channels equal. Inspect a replacement ESP's MAC with `esptool read-mac` while its serial port is free.

## Transport

ESP-NOW packets are at most 250 bytes. The envelope `<3sBIIIBB>` contains `DC1`, channel, drone boot session, sequence, drone time token, fragment index and fragment count. USB wraps each envelope in COBS with a little-endian CRC32 and a zero delimiter. `wire.py` implements framing for the laptop service; the native relay implements the same framing in C.

| Channel | Content |
| --- | --- |
| 0 | Status |
| 1 | Flight targets |
| 2 | Binary recording |
| 3 | Recording metadata |
| 4 | Recorder subscription |
| 5 | Recording acknowledgement |
| 6 | Requests and replies, including arm, stop, settings and updates |

The relay bounds its outgoing queue to eight packets and permits four sends awaiting callbacks. It does not generate targets, refresh their timestamps, or arm the drone.

Firmware updates verify written chunks or changed files only. Skip firmware backups and whole-image verification.
