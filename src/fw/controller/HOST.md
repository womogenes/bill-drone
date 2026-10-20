# Laptop service

Connect the native controller's USB cable and run this from the repository root:

```sh
uv run src/fw/controller/bridge.py --port /dev/ttyUSB0
```

Select the controller's actual port. The service owns that port at 921600 baud; close serial terminals and stop any previous service instance before starting another. It listens on `127.0.0.1:8765`.

For flight tests, run the dashboard's production build in another terminal:

```sh
cd src/sw/dashboard
pnpm build
pnpm start
```

Open `http://localhost:3000`. Rebuild and restart the dashboard after code changes, with the drone disarmed. Development mode's hot reload can close the control connection.

| Endpoint | Use |
| --- | --- |
| WebSocket `/ws` | Dashboard status, targets and commands |
| WebSocket `/update` | Firmware updates through `tools/ota_direct.py` |
| GET `/status` | Current connection and sensor state |
| GET `/settings` | Current cached settings |
| GET `/schema` | Native setting names, defaults, ranges and write permissions |
| POST `/settings` | Change settings with `{"settings": {...}}` |

The service records armed sessions directly into `src/sw/recordings/`; `--recordings` changes the directory. Each session has a binary recording and a JSONL event file. `src/sw/flight_log.py` reads them.

The browser sends changed targets immediately and repeats the current targets every 50 ms. The service forwards them with a recent drone time token. The drone rejects stale packets and stops the motors after 500 ms without fresh targets. Closing the dashboard sends Stop, which repeats until acknowledged. After a lost link or drone restart, the dashboard reconnects with zero targets and requires an explicit Enable.

After a USB error, the service closes the old connection and retries the selected port once per second. If Linux assigns a different port after reconnection, restart the service with that port. Check `GET /status` for drone availability before opening flight controls.
