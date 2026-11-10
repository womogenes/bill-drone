"""
When the fly-away failsafe and the test pattern's limits (fw/drone/main/guard.c) would have fired on a recorded flight: uv run -m sw.sim.failsafe_replay sw/recordings/<flight>.bin [--set name=value,...] [--plot out.png]

Open loop: each record's hover state as logged (the GPS-corrected velocity, the mode, liftoff) and the GPS's position from the recording's gps events (as the distance from where the hover engaged, which hover.c's track follows over hover_gps_place_tau), with the laptop's stick and nudge from its command events, go through the firmware's guard_hover on the recording's settings (the failsafe's at settings.def's defaults unless --set). The drone did not brake or land as the failsafe would have, so after it first fires the phases only show what the logged flight would have set off; it prints each phase change with the logged speed and distance there. Where the recording's test pattern played (flags bit 8), the pattern's limits run on the logged attitude and the same position (without the tag), and it prints when and why they would have stopped it (the pattern's own hold-point channels are taken as unused).
"""

import argparse
import math
from datetime import datetime
from pathlib import Path

import numpy as np

from sw.flight_log import events as recording_events
from sw.flight_log import read_flight
from sw.flight_log import settings as recording_settings
from sw.sim import hover as H
from sw.sim.guard import PHASES, STOPS, Guard


def stamp(text):
    return datetime.fromisoformat(text.replace("Z", "+00:00")).timestamp()


def replay(path, overrides=None):
    """Per record: time (s), mode, logged speed (m/s), GPS distance from the engage point (m), whether the laptop's horizontal input was on, the failsafe's phase, and why the test pattern stopped (0 while it plays or never did)."""
    data, _ = read_flight(path)
    events = recording_events(path)
    settings = recording_settings(path) | (overrides or {})
    gains = H.gains(settings)
    guard = Guard(settings)
    state = H.State()
    # The recording's wall clock: its name's UTC time is the first record's.
    name = path.name
    start = stamp(f"{name[:10]}T{name[11:13]}:{name[14:16]}:{name[17:19]}.{name[20:23]}Z")
    t = (data.esp_time_us.to_numpy() - data.esp_time_us.iloc[0]) / 1e6
    fixes = [e for e in events if e["type"] == "gps" and e.get("ok")]
    fix_t = np.array([stamp(e["received_at"]) - start for e in fixes])
    lat0 = fixes[0]["latitude"]
    north = np.array([e["latitude"] for e in fixes]) * 111195.0
    east = np.array([e["longitude"] for e in fixes]) * 111195.0 * math.cos(math.radians(lat0))
    commands = [e for e in events if e["type"] == "command"]
    command_t = np.array([stamp(e["received_at"]) - start for e in commands])
    flags = data["flags"].to_numpy().astype(int)
    rows, origin, engaged_at = [], None, None
    for k in range(len(data)):
        mode = flags[k] >> 1 & 3
        now_us = round((t[k] + 1) * 1e6)
        if not mode:
            state = H.State()
            origin = engaged_at = None
            rows.append((t[k], 0, 0.0, 0.0, False, 0, guard.state.stopped))
            continue
        j = np.searchsorted(fix_t, t[k]) - 1
        if origin is None:
            origin, engaged_at = (north[max(j, 0)], east[max(j, 0)]), t[k]
        c = commands[max(np.searchsorted(command_t, t[k]) - 1, 0)]
        tick = H.Gains.from_buffer_copy(gains)
        tick.stick_pitch, tick.stick_roll = math.radians(c["pitch"]), math.radians(c["roll"])
        tick.nudge_forward, tick.nudge_left, tick.nudge_up = c["nudge"]
        state.velocity[0], state.velocity[1] = data.hover_vx_m_s.iloc[k], data.hover_vy_m_s.iloc[k]
        state.track[0], state.track[1] = (north[j] - origin[0], east[j] - origin[1]) if j >= 0 else (0.0, 0.0)
        state.gps_us = round((fix_t[j] + 1) * 1e6) if j >= 0 else 0
        state.height = data.hover_height_m.iloc[k]
        state.lifted = 0.0 if state.height >= 0.005 or t[k] - engaged_at >= 1.5 else -1.0
        state.landed = bool(flags[k] >> 4 & 1)
        input_on = bool(tick.stick_pitch or tick.stick_roll or abs(tick.nudge_forward - state.nudge[0]) > 0.001 or abs(tick.nudge_left - state.nudge[1]) > 0.001)
        guard.hover(state, tick, mode, now_us, 0.01)
        # As hover.c leaves them for the next tick: engaged, and the nudge as flown.
        state.engaged = True
        state.nudge[0], state.nudge[1] = tick.nudge_forward, tick.nudge_left
        if flags[k] >> 8 & 1:
            state.yaw = math.radians(data.hover_yaw_deg.iloc[k])
            guard.pattern(state, now_us, math.radians(data.roll_deg.iloc[k]), math.radians(data.pitch_deg.iloc[k]), math.radians(c["roll"]), math.radians(c["pitch"]), c["nudge"][:2], (0.0, 0.0))
        rows.append((t[k], mode, math.hypot(state.velocity[0], state.velocity[1]), math.hypot(state.track[0], state.track[1]), input_on, guard.state.failsafe, guard.state.stopped))
    return np.array(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file", type=Path)
    parser.add_argument("--set", default="", help="setting overrides, name=value,...")
    parser.add_argument("--plot", type=Path, help="save speed, distance, input and phase against time")
    args = parser.parse_args()
    overrides = {k: float(v) for k, v in (item.split("=") for item in args.set.split(",") if item)}
    rows = replay(args.file, overrides)
    t, mode, speed, distance, active, phase, stopped = rows.T
    print(args.file.name)
    for k in np.flatnonzero(np.diff(phase) != 0) + 1:
        print(f"  {t[k]:6.2f} s: failsafe {PHASES[int(phase[k])]:5s} (logged speed {speed[k]:.1f} m/s, {distance[k]:.1f} m from the engage point, mode {int(mode[k])})")
    for k in np.flatnonzero(np.diff(stopped) != 0) + 1:
        print(f"  {t[k]:6.2f} s: the test pattern would have stopped: {STOPS[int(stopped[k])]} (logged speed {speed[k]:.1f} m/s)")
    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        flying = mode > 0
        figure, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
        axes[0].plot(t[flying], speed[flying], label="hover speed estimate (GPS-corrected)")
        axes[0].axhline(overrides.get("failsafe_speed", 2.0), color="grey", ls="--", lw=0.8, label="failsafe_speed")
        axes[0].set_ylabel("m/s")
        axes[1].plot(t[flying], distance[flying], color="C1", label="GPS distance from engage point")
        axes[1].axhline(overrides.get("failsafe_distance", 10.0), color="grey", ls="--", lw=0.8, label="failsafe_distance")
        axes[1].set_ylabel("m")
        axes[1].set_xlabel("s from the recording's start")
        colors = {1: "tab:orange", 2: "tab:green", 3: "tab:red"}
        for ax in axes:
            ymax = ax.get_ylim()[1]
            ax.fill_between(t, 0, ymax, where=active > 0, color="tab:blue", alpha=0.15, step="post", label="laptop stick or nudge")
            for value, color in colors.items():
                ax.fill_between(t, 0, ymax, where=phase == value, color=color, alpha=0.25, step="post", label=f"failsafe {PHASES[value]}")
            if stopped.any():
                ax.axvline(t[np.argmax(stopped > 0)], color="k", lw=1, ls=":", label=f"test pattern stopped ({STOPS[int(stopped.max())]})")
            ax.legend(loc="upper left", fontsize=8)
        axes[0].set_title(f"{args.file.name[:19]}: the failsafe replayed open loop on the logged flight")
        figure.tight_layout()
        figure.savefig(args.plot, dpi=120)
        print(f"Saved {args.plot}")


if __name__ == "__main__":
    main()
