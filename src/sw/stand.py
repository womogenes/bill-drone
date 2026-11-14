"""
The thrust stand (fw/stand): runs a throttle profile on one motor while recording its load cells and the laptop's microphone, then lines them up: uv run -m sw.stand <command>

  run PROFILE [--port P] [--max 1100] [--cells 1|2] [--no-audio]   fly a profile on the stand and save it under sw/recordings/stand/<UTC time>/
  scale --mass GRAMS [--cell 0|1]                                    with that mass on the cell, store its grams per count (~/.config/bill-drone/stand.json)
  esc-cal                                                            teach a new ESC the drone's pulse range, 1000-2200 us (props off)
  analyze RUN_DIR [--blades 2]                                       thrust, rpm from the audio, and each step's timing, as a plot and CSV beside the run

Profiles (throttles are the drone's, 0-1200; FLIP.md stage A): steps (A1: hover 600 to 1000, to the flip's floor near 290 and to 1, each and back, 1.5 s apiece), full (A2: 1100 for 1 s from hover), table (0 to the cap in steps of 100, 2 s each, for the thrust table).

The stand stamps each load-cell reading with its own clock; the laptop stamps every line as it arrives and maps the stand's clock onto its own by the earliest arrivals (the least delayed). The audio starts on the laptop's clock with pw-record's own buffering unknown, so analyze shifts the audio by the lag that best matches rpm² to thrust (thrust goes as rpm²), within half a second.
"""

import argparse
import json
import subprocess
import sys
import threading
import time
import wave
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import serial

ROOT = Path(__file__).resolve().parent / "recordings" / "stand"
CONFIG = Path.home() / ".config" / "bill-drone" / "stand.json"
BAUD = 921600
AUDIO_RATE = 48000
# Commands go out this often; the stand stops the motor after 300 ms without one.
COMMAND_S = 0.02
# Zero throttle first: the ESC arms and the cells settle, and the mean is the tare.
TARE_S = 3.0


def segments(profile, cap):
    """The profile as (seconds, throttle) holds, ramped in and out through hover so no step starts from rest."""
    ramp = [(1.0, 300), (1.0, 600)]
    if profile == "steps":
        body = [(3.0, 600), (1.5, 1000), (1.5, 600), (1.5, 290), (1.5, 600), (1.5, 1), (1.5, 600)]
    elif profile == "full":
        body = [(3.0, 600), (1.0, 1100), (2.0, 600)]
    elif profile == "table":
        body = [(2.0, throttle) for throttle in range(100, cap + 1, 100)] + [(2.0, 600)]
    else:
        raise SystemExit(f"unknown profile {profile}")
    return ramp + [(seconds, min(throttle, cap)) for seconds, throttle in body] + [(1.0, 300)]


def open_port(name):
    # DTR and RTS released before opening: asserted, the board's auto-reset holds it in reset.
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = name, BAUD, 0.05
    port.dtr = port.rts = False
    port.open()
    return port


def read_lines(port, out, stop):
    """Every line from the stand with the laptop's arrival time (monotonic s) until stop is set."""
    buffer = b""
    while not stop.is_set():
        chunk = port.read(4096)
        now = time.monotonic()
        if not chunk:
            continue
        buffer += chunk
        *lines, buffer = buffer.split(b"\n")
        for line in lines:
            out.write(f"{now:.6f} {line.decode(errors='replace').strip()}\n")


def run(args):
    plan = segments(args.profile, args.max)
    total = TARE_S + sum(seconds for seconds, _ in plan)
    print(f"{args.profile}: {total:.0f} s, throttles {[throttle for _, throttle in plan]}, cap {args.max}")
    if input("Stand clamped, props clear of everything, battery on? Type go: ").strip() != "go":
        return
    name = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%S")
    directory = ROOT / name
    directory.mkdir(parents=True)
    port = open_port(args.port)
    time.sleep(0.5)
    port.write(f"C {(1 << args.cells) - 1}\n".encode())
    audio = None
    if not args.no_audio:
        audio_started = time.monotonic()
        audio = subprocess.Popen(["pw-record", "--rate", str(AUDIO_RATE), "--channels", "1", "--format", "s16", str(directory / "audio.wav")])
    stop = threading.Event()
    with open(directory / "lines.txt", "w") as out:
        reader = threading.Thread(target=read_lines, args=(port, out, stop), daemon=True)
        reader.start()
        commands = []
        started = time.monotonic()
        try:
            schedule = [(TARE_S, 0)] + plan
            for seconds, throttle in schedule:
                end = time.monotonic() + seconds
                commands.append((time.monotonic(), throttle))
                while time.monotonic() < end:
                    port.write(f"T {throttle}\n".encode())
                    time.sleep(COMMAND_S)
        except KeyboardInterrupt:
            print("stopped")
        finally:
            for _ in range(5):
                port.write(b"T 0\n")
                time.sleep(COMMAND_S)
            time.sleep(0.3)
            stop.set()
            reader.join()
            port.close()
            if audio:
                audio.terminate()
                audio.wait()
    meta = {"profile": args.profile, "cap": args.max, "cells": args.cells, "started_monotonic": started, "commands": commands,
            "audio": None if audio is None else {"file": "audio.wav", "rate": AUDIO_RATE, "started_monotonic": audio_started},
            "scale": load_config().get("grams_per_count", {})}
    (directory / "meta.json").write_text(json.dumps(meta, indent=1))
    print(f"saved {directory}")


def load_config():
    return json.loads(CONFIG.read_text()) if CONFIG.exists() else {}


def parse(directory):
    """The stand's lines as arrays: per cell (laptop time s, raw), the ESC pulses (laptop time s, us), on the laptop's clock."""
    cells, pulses, offsets = {0: [], 1: []}, [], []
    for line in (directory / "lines.txt").read_text().splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        arrived, kind = float(parts[0]), parts[1]
        try:
            if kind == "F":
                cell, stand_us, raw = int(parts[2]), int(parts[3]), int(parts[4])
                cells[cell].append((stand_us, raw))
            elif kind == "E":
                stand_us, pulse = int(parts[2]), int(parts[3])
                pulses.append((stand_us, pulse))
            elif kind == "H":
                stand_us = int(parts[2])
            else:
                continue
        except (ValueError, IndexError):
            continue
        offsets.append(arrived - stand_us / 1e6)
    # The least-delayed lines set the clock offset; the stand's crystal drifts by tens of ppm, under a millisecond a minute.
    offset = np.percentile(offsets, 1)
    to_laptop = lambda rows: np.array([(us / 1e6 + offset, value) for us, value in rows]) if rows else np.zeros((0, 2))
    return {cell: to_laptop(rows) for cell, rows in cells.items()}, to_laptop(pulses)


def scale(args):
    port = open_port(args.port)
    port.write(f"C {1 << args.cell}\n".encode())
    def mean_counts(seconds):
        values, end = [], time.monotonic() + seconds
        buffer = b""
        while time.monotonic() < end:
            buffer += port.read(4096)
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                parts = line.split()
                if len(parts) == 4 and parts[0] == b"F" and int(parts[1]) == args.cell:
                    values.append(int(parts[3]))
        if not values:
            raise SystemExit("no readings: check the HX711 wiring and the cell number")
        return np.mean(values)
    input(f"Cell {args.cell} empty. Enter to measure the zero.")
    zero = mean_counts(3.0)
    input(f"Put the {args.mass} g mass on cell {args.cell}. Enter to measure.")
    loaded = mean_counts(3.0)
    port.close()
    grams_per_count = args.mass / (loaded - zero)
    config = load_config()
    config.setdefault("grams_per_count", {})[str(args.cell)] = grams_per_count
    CONFIG.parent.mkdir(parents=True, exist_ok=True)
    CONFIG.write_text(json.dumps(config, indent=1))
    print(f"cell {args.cell}: {grams_per_count:.6g} g per count ({loaded - zero:.0f} counts for {args.mass} g), saved to {CONFIG}")


def esc_cal(args):
    if input("Props OFF the motor, battery DISCONNECTED from the ESC. Type go: ").strip() != "go":
        return
    port = open_port(args.port)
    stop = threading.Event()
    def hold(pulse):
        while not stop.is_set():
            port.write(f"P {pulse}\n".encode())
            time.sleep(COMMAND_S)
    for pulse, prompt in ((2200, "Sending the top (2200 us). Connect the battery, wait for the ESC's top-confirm beeps, then press Enter."),
                          (1000, "Sending the bottom (1000 us). Wait for the confirm beeps, then press Enter.")):
        stop.clear()
        thread = threading.Thread(target=hold, args=(pulse,), daemon=True)
        thread.start()
        input(prompt)
        stop.set()
        thread.join()
    port.write(b"T 0\n")
    port.close()
    print("done: the ESC now takes 1000-2200 us, the drone's range")


def rpm_track(path, blades):
    """The rotation rate (rpm) over time (s from the audio's start) from the blade-pass tone: the strongest peak of the harmonic sum in 30-400 Hz (900-12000 rpm at two blades), refined between bins; NaN where no tone stands out (motor stopped)."""
    from scipy.signal import stft
    with wave.open(str(path)) as audio:
        rate = audio.getframerate()
        samples = np.frombuffer(audio.readframes(audio.getnframes()), dtype=np.int16).astype(float)
    # 85 ms windows every 10 ms: short enough to time a step, with the peak refined between the 12 Hz bins.
    frequencies, times, spectrum = stft(samples, rate, nperseg=4096, noverlap=4096 - rate // 100)
    power = np.abs(spectrum)
    band = (frequencies >= 30) & (frequencies <= 400)
    # The fundamental's power plus its second and third harmonics', which keeps the tracker off a lone harmonic.
    summed = np.zeros((band.sum(), power.shape[1]))
    for harmonic in (1, 2, 3):
        index = np.clip(np.searchsorted(frequencies, frequencies[band] * harmonic), 0, len(frequencies) - 1)
        summed += np.log1p(power[index])
    peak = np.clip(np.argmax(summed, axis=0), 1, band.sum() - 2)
    columns = np.arange(summed.shape[1])
    left, middle, right = summed[peak - 1, columns], summed[peak, columns], summed[peak + 1, columns]
    shift = 0.5 * (left - right) / np.where(left - 2 * middle + right == 0, -1e-9, left - 2 * middle + right)
    step = frequencies[1] - frequencies[0]
    blade_pass = frequencies[band][peak] + np.clip(shift, -0.5, 0.5) * step
    # A tone: its peak well above the band's median in that frame.
    tonal = middle > np.median(summed, axis=0) + 3.0
    return times, np.where(tonal, blade_pass * 60 / blades, np.nan)


def analyze(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    directory = Path(args.run)
    meta = json.loads((directory / "meta.json").read_text())
    cells, pulses = parse(directory)
    start = meta["started_monotonic"]
    per_count = {int(cell): value for cell, value in meta.get("scale", {}).items()}
    thrust = {}
    for cell, rows in cells.items():
        if not len(rows):
            continue
        t = rows[:, 0] - start
        tare = rows[t < TARE_S - 0.5, 1].mean()
        thrust[cell] = (t, (rows[:, 1] - tare) * per_count.get(cell, 1.0))
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    if len(pulses):
        axes[0].step(pulses[:, 0] - start, pulses[:, 1] - 1000, where="post", color="#555")
    axes[0].set_ylabel("throttle (0-1200)")
    for cell, (t, grams) in thrust.items():
        axes[1].plot(t, grams, lw=1, label=f"cell {cell}")
    axes[1].set_ylabel("thrust (g)" if per_count else "thrust (counts, not scaled)")
    axes[1].legend(loc="upper right")
    rows = []
    if meta.get("audio") and (directory / "audio.wav").exists():
        audio_t, rpm = rpm_track(directory / "audio.wav", args.blades)
        audio_t = audio_t + meta["audio"]["started_monotonic"] - start
        lag = 0.0
        if thrust:
            # The audio's buffering delay: the lag within ±0.5 s at which rpm² best follows the thrust.
            t, grams = next(iter(thrust.values()))
            grid = np.arange(max(t[0], audio_t[0]) + 0.5, min(t[-1], audio_t[-1]) - 0.5, 0.01)
            force = np.interp(grid, t, grams)
            lags = np.arange(-0.5, 0.5, 0.005)
            def score(shift):
                heard = np.interp(grid, audio_t - shift, rpm ** 2)
                valid = ~np.isnan(heard)
                return np.corrcoef(force[valid], heard[valid])[0, 1] if valid.sum() > 100 else np.nan
            scores = [score(shift) for shift in lags]
            lag = float(lags[int(np.nanargmax(scores))])
            print(f"audio lag {lag * 1000:.0f} ms (rpm² against thrust correlation {np.nanmax(scores):.2f})")
        axes[2].plot(audio_t - lag, rpm, lw=1, color="#1f4e9c")
        rows = np.column_stack([audio_t - lag, rpm])
        np.savetxt(directory / "rpm.csv", rows, delimiter=",", header="seconds,rpm", comments="", fmt="%.4f")
    axes[2].set_ylabel(f"rpm (audio, {args.blades} blades)")
    axes[2].set_xlabel("seconds from the start of the run")
    # Each throttle change's response: when the first cell's thrust got 63% of the way to its new level.
    if thrust and len(pulses):
        t, grams = next(iter(thrust.values()))
        changes = pulses[1:]
        print("step                        63% time (ms)")
        for i, (when, pulse) in enumerate(changes):
            when -= start
            until = changes[i + 1][0] - start if i + 1 < len(changes) else t[-1]
            before = grams[(t > when - 0.4) & (t < when)]
            after = grams[(t > until - 0.4) & (t < until)]
            span = grams[(t >= when) & (t < until)]
            span_t = t[(t >= when) & (t < until)]
            if len(before) < 2 or len(after) < 2 or len(span) < 2:
                continue
            target = before.mean() + 0.63 * (after.mean() - before.mean())
            crossed = np.flatnonzero((span - target) * np.sign(after.mean() - before.mean()) >= 0)
            if crossed.size:
                print(f"  to throttle {int(pulse) - 1000:4d} at {when:6.2f} s  {(span_t[crossed[0]] - when) * 1000:6.0f}")
    for axis in axes:
        axis.grid(alpha=0.3)
    fig.suptitle(f"Stand run {directory.name} UTC, profile {meta['profile']}")
    fig.tight_layout()
    fig.savefig(directory / "stand.png", dpi=110)
    for cell, (t, grams) in thrust.items():
        np.savetxt(directory / f"thrust_{cell}.csv", np.column_stack([t, grams]), delimiter=",", header="seconds,thrust", comments="", fmt="%.4f")
    print(f"saved {directory / 'stand.png'}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    commands = parser.add_subparsers(dest="command", required=True)
    flying = commands.add_parser("run")
    flying.add_argument("profile", choices=("steps", "full", "table"))
    flying.add_argument("--max", type=int, default=1100, help="throttle cap (0-1200; the flip's cap is 1100)")
    flying.add_argument("--cells", type=int, choices=(1, 2), default=1)
    flying.add_argument("--no-audio", action="store_true")
    weighing = commands.add_parser("scale")
    weighing.add_argument("--mass", type=float, required=True, help="the known mass (g)")
    weighing.add_argument("--cell", type=int, choices=(0, 1), default=0)
    commands.add_parser("esc-cal")
    looking = commands.add_parser("analyze")
    looking.add_argument("run")
    looking.add_argument("--blades", type=int, default=2)
    for sub in (flying, weighing, commands.choices["esc-cal"]):
        sub.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()
    {"run": run, "scale": scale, "esc-cal": esc_cal, "analyze": analyze}[args.command](args)


if __name__ == "__main__":
    main()
