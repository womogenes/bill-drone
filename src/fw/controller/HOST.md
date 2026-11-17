# Laptop service

Connect the native controller's USB cable and run this from the repository root:

```sh
uv run src/fw/controller/bridge.py
```

The service finds the controller on any USB serial device by its packets (`find_port.py`; `--port` only names a device to try first) and owns that port at 921600 baud, locked so the dashboard's FPV receiver search passes it over. Stop any previous service instance before starting another. It listens on `127.0.0.1:8765`.

For flight tests, run the dashboard's production build in another terminal:

```sh
cd src/sw/dashboard
pnpm build
pnpm start
```

Open `http://localhost:3000`.

## Background services

To run both in the background instead, start them from the repository root. These are transient units: they last until logout or reboot, and `systemctl --user restart` fails with "Unit … not found" once they are gone, so after a reboot start them again with these two commands rather than restarting them. The dashboard needs a build first only after its code changed (`cd src/sw/dashboard && pnpm build`).

```sh
systemd-run --user --unit=bill-drone-controller -p Restart=on-failure --working-directory=$PWD $(which uv) run src/fw/controller/bridge.py
systemd-run --user --unit=bill-drone-dashboard -p Restart=on-failure -p TimeoutStopSec=5 --working-directory=$PWD/src/sw/dashboard $PWD/src/sw/dashboard/node_modules/.bin/next start
```

Check that the drone is heard with `curl -s 127.0.0.1:8765/status`: `"online": true`. If it stays `false` for more than about 10 s with the drone powered and both LEDs blinking, the controller came up stuck (seen after reboots and replugs); reset it through its serial port, which esptool does when it reads the MAC, and start the bridge again:

```sh
systemctl --user stop bill-drone-controller
bash -c 'source ~/esp/esp-idf/export.sh >/dev/null && esptool.py --port /dev/ttyUSB0 read_mac'
systemctl --user start bill-drone-controller
```

If the dashboard unit fails with `start-limit-hit`, something else already holds port 3000 (a `pnpm start` left in a terminal): `ss -ltnp | grep :3000` shows it; stop it, then `systemctl --user reset-failed bill-drone-dashboard` and start the unit again.

The dashboard unit runs Next directly: pnpm ignores the stop signal, so a `pnpm start` unit takes systemd's full 90 s timeout to stop. While the units exist, restart them with `systemctl --user restart bill-drone-controller bill-drone-dashboard` and read the bridge's log with `journalctl --user -u bill-drone-controller -f`. Rebuild and restart the dashboard after code changes, with the drone disarmed. Development mode's hot reload can close the control connection.

| Endpoint            | Use                                                                                                                   |
| ------------------- | --------------------------------------------------------------------------------------------------------------------- |
| WebSocket `/ws`     | Dashboard status, targets and commands                                                                                |
| WebSocket `/update` | Firmware updates through `tools/ota_direct.py` and the dashboard's camera requests, any number of clients at once           |
| GET `/status`       | Current connection and sensor state                                                                                   |
| GET `/settings`     | Current cached settings                                                                                               |
| GET `/schema`       | Native setting names, defaults, ranges and write permissions                                                          |
| POST `/settings`    | Change settings with `{"settings": {...}}`                                                                            |
| POST `/radio`       | The controller's sending rate and power until it resets, with `{"rate": i, "power": dBm}` (README "Radio options")    |
| POST `/pattern`     | Load the identification test pattern (built by `uv run -m sw.excite`), recorded at the start of every later recording |

The service records armed sessions directly into `src/sw/recordings/`; `--recordings` changes the directory. Each session has a binary recording and a JSONL event file. `src/sw/flight_log.py` reads them.

The browser sends changed targets immediately and repeats the current targets every 50 ms. The service forwards them with a recent drone time token. The drone rejects stale packets and stops the motors after 500 ms without fresh targets. Closing the dashboard sends Stop, which repeats until acknowledged. After a lost link or drone restart, the dashboard reconnects with zero targets and requires an explicit Enable.

After a USB error, or 5 s without a packet from the controller (its relay sends a link report every second), the service closes the port and looks on every USB serial device once a second, first the controller's last physical USB socket (`/dev/serial/by-path`), so a re-plug that renames it (ttyUSB0 to ttyUSB3 on 2026-11-14) needs no restart. It never opens a device another process holds (a flock, or open in the dashboard as an FPV receiver or in a serial monitor) and writes nothing while it looks. `GET /status` gives the device in use (`port`), how often it was lost and found again (`rescans`) and `usb` (its by-path name, seconds since its last packet, the last loss and the devices the last search passed over). Check `GET /status` for drone availability before opening flight controls.
