# Controller maintenance

Use [README.md](README.md) for relay installation and pairing, and [HOST.md](HOST.md) for the laptop service. The earlier MicroPython relay was retired because it dropped channel-6 requests used by the current firmware. `wire.py` remains the laptop's framing implementation.

The installed pair was identified on 2026-10-20:

| Device | USB topology | Station MAC |
| --- | --- | --- |
| Drone | `1-2.1` | `b0:3f:d3:52:5c:60` |
| Controller | `1-1` | `04:b2:47:54:80:4c` |

Both CP2102 adapters report serial number `0001`, so their `/dev/serial/by-id` alias is ambiguous. Recheck topology and MAC after replacing or reconnecting a board. Opening a serial port can reset its ESP.

Keep one serial owner. Stop the laptop service before flashing the controller, then restart it. Drone updates use the service's `/update` endpoint; they do not require opening the controller's serial port independently.

Historical router and MicroPython relay investigations are recorded in the repository's root `HANDOFF.md`. Those traces predate the native radio and consolidated laptop service and do not establish the current firmware's flight readiness. Retain the native 500 ms watchdog and explicit re-arming.
