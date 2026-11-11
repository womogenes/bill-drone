# USB radio controller

The controller ESP32 runs the ESP-IDF relay in `native/`. It forwards packets between USB serial at 921600 baud and the drone over ESP-NOW on Wi-Fi channel 6. The relay sends to the drone at Espressif's 500 kbps long-range rate (`WIFI_PHY_MODE_LR`), and the drone answers at the default 1 Mbps; both receive either, so a relay or drone on firmware from before 2026-10-29 can still hear the other. Long range gave fewer retries and lower latency on a channel shared with a building's Wi-Fi (median request round trip 41 to 30 ms, 95th percentile 158 to 82 ms), but on the drone's side it left too little airtime for the 100 Hz log stream (10-30% of samples dropped), so the drone stayed at the default rate. The laptop supplies commands and heartbeat freshness.

## Build and install

Keep one serial owner: stop the laptop service before opening the controller's USB port, and restart it afterwards. Opening a serial port can reset its ESP. Identify the controller explicitly: both boards use CP2102 adapters that report serial number `0001`, so `/dev/serial/by-id` is ambiguous and `/dev/ttyUSBn` names can change. Check the MAC (`esptool read-mac`) after replacing or reconnecting a board; the current ones are in the pairing table's sources below.

```sh
# From the repository root:
make -C src/fw/controller/native build
make -C src/fw/controller/native flash PORT=/dev/ttyUSB0
```

The build uses ESP-IDF from `$IDF_PATH` or `~/esp/esp-idf`. Flashing writes the relay application at `0x10000` and checks the written data. It requires the existing factory bootloader and partition table on the controller. It is not a drone firmware update or a fresh-board provisioning command.

Start the [laptop service](HOST.md) after installation. The controller exposes no MicroPython REPL.

## Pairing

Each drone and its controller are a line in `../pairs.h` (their station MACs). At boot each board looks up its own MAC there and talks only to its partner, so the same images serve every pair; a board that is not listed stops at boot and prints its MAC on USB. The drone's MAC is also its ID in every recording (the start record's `drone`). Both ends use channel 6:

| Setting | Source |
| --- | --- |
| Drone and controller MACs | `../pairs.h` |
| Controller radio channel | `native/main/relay.c`: `CHANNEL` |
| Drone radio channel | `../drone/main/radio.c`: `WIFI_CHANNEL` |

A new or replaced board needs its line in `../pairs.h` and both boards' firmware rebuilt and flashed. A replaced controller needs only itself flashed over USB: its relay beacons to the drone its table names, and a disarmed drone that has heard nothing from its own controller for 5 s takes the beaconing relay as its controller and stores it (`../drone/main/radio.c` adopt; drone firmware from 2026-11-10), after which `make deploy` gives the drone the new table. Keep the channels equal, or let the drone follow (Radio options below). Inspect a board's MAC with `esptool read-mac` while its serial port is free. `find_port.py` finds any listed controller.

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
| 7 | Link reports (`../link.h`): LNK1 from the drone once a second, RLY1 from the relay once a second over USB and every 250 ms to the drone as its beacon; RCF1 from the laptop sets the relay's radio and is not forwarded |

The relay bounds its outgoing queue to eight packets and permits four sends awaiting callbacks. It does not generate targets, refresh their timestamps, or arm the drone.

Firmware updates verify written chunks only. Skip firmware backups and whole-image verification.

## Link quality

Each end counts, per second, the other's packets it heard, their RSSI and the noise floor, the other's sequence numbers that never arrived, the longest silence, and its own sends acknowledged, failed (no acknowledgement after the MAC's retries) and refused, with the slowest send-to-callback time (`../link.h` LinkReport). The bridge adds the status messages it received of the drone's 20, the longest gap between them and its link timeouts, shows all three as `reading.link` (`GET /status`, the dashboard's radio item) and writes them into each recording as `link` events, with a `dropout` event for every status gap of 0.3 s or more. Firmware and bridges from before 2026-11-09 ignore channel 7, so either side can be updated first; the report is then missing from the other.

On 2026-11-08 PST outdoors the status stream (drone to laptop) lost 0.1-0.3% of its 10 Hz GPS solutions within 10 m of the laptop and 25-77% beyond 15 m, camera on or off; 30 dropouts of 0.4 s or more (median 1.0 s, longest 8.3 s), all but three at 14-50 m (the camera on for 21 of them, when the flights also went farther). The drone and both ESP32s have PCB antennas.

## Radio options

| Option | Drone | Controller | Default |
| --- | --- | --- | --- |
| Sending rate (`../link.h` link_rates: 0 1 Mbps, 1 LR 500 kbps, 2 LR 250 kbps, 3 2 Mbps, 4 5.5 Mbps, 5 11 Mbps, 6 11g 6 Mbps) | setting `radio_rate` | `RATE`, or `POST /radio {"rate": i}` | drone 0, controller 1 |
| Maximum transmit power (dBm, 0 = the driver's own, 20 dBm) | setting `radio_power` | `POWER`, or `POST /radio {"power": dBm}` | 0 on both |
| Channel (1, 6 or 11) | follows the controller with setting `radio_follow` 1 | `CHANNEL`, or `POST /radio {"channel": c}` | 6, follow off |

Both ends enable 802.11b/g/n and Espressif long range together, so each hears every rate and either can change its own sending rate alone; an LR-only protocol set would gain no sensitivity and would stop hearing the other's 802.11b frames, so neither uses it. A rate or power change on the drone persists (NVS); `POST /radio` changes last until the controller resets, so unplugging it undoes them. The bridge refuses `POST /radio` while the drone is armed, and another channel than 6 unless the drone reports `radio_follow` on: a following drone that is disarmed and has not heard its controller for 2 s tries 6, 1 and 11 for 0.75 s each until it hears the relay's beacon. A drone on 11 shares the camera's FPV channel.

If a mismatch still breaks the link: reset or replug the controller (back to its compiled channel, rate and power), or `POST /radio {"channel": 6}`; a drone that still answers nothing can be set back over USB with `make -C src/fw usb PORT=...` (its settings stay; with `radio_follow` on it finds any of the three channels by itself).
