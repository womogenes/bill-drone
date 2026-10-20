# Laptop bridge

After flashing and pairing the controller ESP, connect its USB cable and run:

```sh
uv run src/fw/controller/bridge.py --port /dev/ttyUSB0
```

Run the dashboard normally. Select **USB controller** in its header, or open `http://localhost:3000/?controller=usb`. Select **Router** to use the original Wi-Fi connection.

The bridge serves the existing `/ws` controls and `/log` recording stream at `127.0.0.1:8765`. The dashboard and its server should run on this same laptop. Flight recordings keep using the dashboard server and its recordings directory.

The bridge forwards browser heartbeats without generating them. Each outgoing packet echoes a recent drone time token; the drone rejects stale packets. Radio failures close the local connections; the dashboard reconnects when the radio returns. Reconnection never enables control or replays commands. Restart the bridge after a USB error or unplugging the controller.

Stop and Disable stay latched until the drone acknowledges a Stop command from that request. While waiting, incoming browser heartbeats resend Stop; other control commands are discarded. Earlier acknowledgements cannot release a new Stop.
