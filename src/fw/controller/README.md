# Direct controller

The laptop uses USB to reach a second ESP32. That ESP32 relays ESP-NOW packets directly to the drone on channel 6. The dashboard's commands, telemetry and flight recordings keep their existing formats.

## Install

Use MicroPython with `espnow` on the USB controller. Identify its station MAC before installing:

```sh
mpremote exec 'import network; w=network.WLAN(network.STA_IF); w.active(True); print(w.config("mac").hex())'
```

From this directory, upload the controller files:

```sh
mpremote fs cp wire.py controller_config.py main.py : + reset
```

`controller_config.py` contains the drone's station MAC and radio channel. On the drone, add these settings to its existing `config.py`, substituting the controller's MAC:

```python
DIRECT_CONTROLLER_MAC = bytes.fromhex("001122334455")
DIRECT_CHANNEL = 6
```

Upload the drone's `main.py`, `drivers/direct.py`, and `drivers/direct_wire.py` together. `direct_wire.py` points to the shared `controller/wire.py` source. Direct mode disconnects the router; use USB to change the drone's configuration afterward. Remove `DIRECT_CONTROLLER_MAC` to return to router mode.

Start the laptop bridge as described in [HOST.md](HOST.md), then choose **USB controller** in the dashboard. Keep the normal dashboard server running for recording to disk.

The controller maps buffered UART1 onto GPIO1/GPIO3, the USB serial pins. It reserves its USB serial port at 115200 baud while running. For subsequent uploads, reset it and interrupt `main.py` during the two-second startup window. Only one program can own its USB port.

## Transport

Radio packets fit the ESP-NOW v1 250-byte limit. Header `<3sBIIIBB>` contains `DC1`, channel, drone boot session, sequence, drone time token, fragment index and fragment count. Channels are beacon (0), dashboard (1), log binary (2), log JSON (3), log subscription/heartbeat (4), and log acknowledgement (5). USB wraps the same packet in COBS with a little-endian CRC32 and zero delimiter.

The drone sends its time token to its paired controller every 50 ms. Commands must echo a token less than 200 ms old, match the current boot session, and advance their channel sequence. The bridge forwards browser heartbeats; it never generates flight-control heartbeats. The native 500 ms motor failsafe remains active.

The bridge holds at most one unacknowledged dashboard command; later commands wait up to 200 ms behind it, and Stop discards them. While a command is unacknowledged, each browser heartbeat resends its exact packet instead of a new heartbeat. The drone acknowledges each applied command on channel 5 with payload `01` and the command's sequence, and acknowledges a repeated sequence again without reapplying it. A command unacknowledged after 200 ms stops the drone and closes the dashboard connection. After any Stop, commands are ignored until a Stop is acknowledged; heartbeats resend that Stop.

Telemetry uses the latest readings. Log messages use stop-and-wait acknowledgement, fragmentation, bounded retries and duplicate suppression. A missing log acknowledgement releases the recorder after two seconds. Command and heartbeat reception runs separately from log retries.

Firmware updates verify changed chunks or files only; do not run whole-image verification.

## Native relay

`native/` is an ESP-IDF 5.5 replacement for `main.py` with the same wire behaviour: ESP-NOW callbacks feed queues, a UART event task decodes USB frames, and at most four sends await their callback with up to eight queued (oldest dropped). It never generates packets itself.

```sh
make -C native        # build, then write only the app partition at 0x10000
```

It reuses MicroPython's bootloader, partition table and NVS, so returning to MicroPython means reflashing MicroPython. `PORT` defaults to the controller's USB path (topology 1-1). Restart the bridge after flashing.
