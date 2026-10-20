# Direct controller handoff — 2026-10-20

The direct-link implementation is installed, but the end-to-end recording/control bench has **not passed**. Keep motor power disconnected while completing it.

## Hardware and deployment

| Device | USB topology / port at handoff | Station MAC | Firmware |
| --- | --- | --- | --- |
| Drone | `1-2.1`, `/dev/ttyUSB1` | `b0:3f:d3:52:5c:60` | Existing custom MicroPython with native control on core 1 |
| Controller | `1-1`, `/dev/ttyUSB0` | `04:b2:47:54:80:4c` | Stock ESP32 MicroPython 1.29.0 plus this directory's relay |

Both CP2102 devices report serial number `0001`. Their `/dev/serial/by-id` alias is ambiguous. Recheck topology and MAC after reconnection; ports can change.

Controller files uploaded successfully: `main.py`, `wire.py`, `controller_config.py`. The relay uses buffered **UART1 mapped to GPIO1/GPIO3 at 115200 baud**, which are the existing USB bridge pins. A two-second startup delay allows Ctrl-C after reset for updates. Keep the serial port open during operation; opening it was observed to restart this board despite initializing DTR/RTS false.

Large controller flash writes failed partway through. Independently verified 64 KiB ROM-flasher writes completed. The controller's old filesystem was unreadable; only its VFS boot sector was cleared, allowing MicroPython to initialize the VFS. No drone flash or filesystem erase was done by this controller task.

**Never run whole-image verification again. Do not create firmware backups.** Verify changed chunks/files only. The user's prohibition arrived after the initial installation verification.

The drone is configured for direct mode, peer `04b24754804c`, channel 6. Direct mode disconnects the router. USB is required for its recovery/configuration. Deployed drone files include `config.py`, `main.py`, `drivers/direct.py`, and `drivers/direct_wire.py`; the final upload included Stop ACKs, the command-loop fairness yield, and radio send gating.

The final USB check explicitly stopped native control and reported `enabled=False`, `throttle=(0, 0)`. The saved calibration file remained present with sensor frame `[1, -1, -1]`, gyro bias `[0.058022192, -0.02545916, -0.075941816]` rad/s, and pose zero `[0.119175928, -7.08157236]` degrees. The normal direct-mode application was then restarted. No powered testing was performed during this investigation.

Temporary socket/scheduler profiling code was removed from the deployed application. The dashboard heartbeat was restored to its original one-byte format.

## Laptop

Two user services are running: `bill-drone-dashboard.service` and `bill-drone-controller.service`. The controller service is transient:

```sh
systemctl --user status bill-drone-controller.service
journalctl --user -u bill-drone-controller.service -n 30
```

Command used by the service:

```sh
uv run src/fw/controller/bridge.py --port /dev/ttyUSB0
```

It serves `ws://127.0.0.1:8765/ws` and `/log`. The normal dashboard server remains responsible for writing recordings. Choose **USB controller** in the dashboard or use `?controller=usb`. Only one client may own each bridge endpoint; close the dashboard's control connection before using an independent bench client.

## Verified

- Controller radio: correct MAC/channel, direct MAC acknowledgement from drone; 35 received / 35 unique beacons over a two-second RAM-only check, zero reported radio drops.
- Buffered UART1 at 115200: after startup, consecutive two-second captures contained 35, 34 and 35 CRC-valid unique beacon frames, with no invalid bytes in those steady windows. Earlier short high-baud captures included startup output and cannot establish a high-baud error rate.
- Local pseudo-terminal/WebSocket integration: commands, fragmented metadata, binary logs, duplicate log suppression, acknowledgements, reconnection without replay, stale-command rejection.
- Stop latch: dropped first stop is retried in response to actual browser heartbeats; explicit matching stop ACK releases it. Stopped telemetry, old ACKs and wrong-session ACKs cannot release it.
- Firmware Python compilation and dashboard typecheck/scoped lint passed.

## Unresolved

The parent's latest full bench, `/tmp/bench_direct_bridge.py`, was rejected with WebSocket 1013 (`Direct link unavailable or endpoint in use`) before enabling. A final passive connection received the same rejection. `ss` showed no existing clients on port 8765, so the remaining failure is radio availability rather than endpoint ownership. An earlier service instance observed a drone restart at 00:45:11, then a radio-link timeout at 00:45:39. The latest service was restarted at 00:46:54. Its configured baud is 115200. This is not a successful flight-readiness result.

Next checks should expose bridge counters before changing more code: last beacon age/session/sequence, bytes received, valid/invalid serial frames and endpoint owners. Correlate those with controller and drone ESP-NOW TX/response/RX/drop counters. Then repeat the zero-throttle recording + heartbeat-loss bench. `/tmp/bench_direct_bridge.py` was prepared by the parent task.

Every non-heartbeat command is now acknowledged; see README "Transport". This bridge and drone code is compiled and tested against local pseudo-terminal/mock harnesses only; it has not been uploaded or bench-tested. Upload the drone before restarting the new bridge: a drone that only acknowledges Stop makes the new bridge stop and disconnect on every other command.


ESP-NOW `send(..., False)` can still retry internally for up to two seconds on `ESP_ERR_ESPNOW_NO_MEM`. Both relay and drone now gate sends on matching TX/response counters; log fragments yield while waiting. This reduces queue pressure but is not a proof that every native send call is nonblocking.

## Earlier Wi-Fi investigation

The user's cutoff recording is `src/sw/recordings/2026-10-20T07-06-00-641Z_162998185.bin` and its `.jsonl` sidecar. It stopped after 7.623 seconds with `Controller heartbeat lost`, 382 samples, zero drops, and valid saved calibration. Roll/pitch stayed within 0.6°. Subsequent recordings include diagnostic zero-throttle runs; do not mistake them for powered flights.

The parent reproduced missing heartbeats with the actual dashboard: browser heartbeat send times continued at roughly 49 ms intervals while the drone stopped accepting them for seconds. Native control continued on its other core.

Adding explicit asyncio fairness yields reduced the measured maximum VM loop interval from 3210 ms to 210 ms. Individual instrumented calls were bounded in that follow-up: socket read ≤13.5 ms, write ≤10.1 ms, sensor snapshot ≤15.5 ms, poll ≤49.5 ms. A 792 ms heartbeat gap still occurred, with roughly 1.3 KiB TCP not-sent data and retransmission backoff reaching 9.376 seconds. The fairness fix therefore did not fully resolve the router/TCP path. Preserve the native 500 ms motor watchdog.

Calibration persistence and the native calibration arming gate were verified in an earlier five-second, 255-sample recording at 50 Hz with zero drops. That verification predates this direct bridge and does not validate the bridge.

Useful temporary traces/scripts on this machine:

- `/tmp/heartbeat-link.jsonl`: original laptop send/receive and TCP timing capture.
- `/tmp/heartbeat-fairness-before.jsonl` and `/tmp/heartbeat-fairness-after.jsonl`: bounded owner/observer reproduction.
- `/tmp/install_link_profile.py` and `/tmp/socket_timing_snippet.py`: temporary instrumentation generator, not production code.
- `/tmp/bench_direct_bridge.py`: pending 60-second zero-throttle, recording, and heartbeat-loss test.

The temporary bench script needs its cleanup made tolerant of a rejected/closed connection before reusing it; the first failed handshake also raised while trying to send its cleanup stop. No Enable was sent in that failed attempt.

## Source state

`drivers/direct_wire.py` is a symlink to `controller/wire.py`. Upload tools were fixed to retain the remote alias `drivers/direct_wire.py` rather than resolving it into `controller/wire.py`.

Controller source SHA256 at the last successful upload (not a fresh device readback):

- `main.py`: `7840dc3957452fdf40780221418e2e1759171c8fe57f625a3dada7ef4eaf1450`
- `wire.py`: `21292ae3b92150fccb0f42ec6bba7f9af9bbad608c5ae605f4e0eab1a0c5d10e`
- `controller_config.py`: `6442b1e3cc9bd77d57bb20dfe05c354aeccccf1e3a2df1792b3827d36074040b`

The source now contains generic command acknowledgement (not yet deployed). The controller relay is unchanged. No powered commands or flight tests were performed by this controller task.
