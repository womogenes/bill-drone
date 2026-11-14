"""
The thrust stand (fw/stand): runs a throttle profile on one motor while recording its load cells and the laptop's microphone, then lines them up: uv run -m sw.stand <command>

  run PROFILE [--port P] [--max 1100] [--cells 1|2] [--no-audio]   fly a profile on the stand and save it under sw/recordings/stand/<UTC time>/
  scale --mass GRAMS [--cell 0|1]                                    with that mass on the cell, pulling the way the prop does, store its lift per count (~/.config/bill-drone/stand.json; signed: lift is (count - tare) times it)
  esc-cal [--top 2000] [--hz 400]                                    teach a new ESC its pulse range, 1000 us to --top (props off); --hz 50 for ESCs that only read servo-rate signals
  spin [--throttle 50] [--seconds 2] [--hz 400]                      spin the motor gently to check the signal reaches the ESC (props off)
  flash [--port wifi] [--image fw/stand/build/stand.bin]               send new stand firmware over Wi-Fi (build it with make -C fw/stand build); the stand restarts into it
  analyze RUN_DIR [--blades 2]                                       thrust, rpm from the audio, and each step's timing, as a plot and CSV beside the run

Profiles (throttles are the drone's, 0-1200; FLIP.md stage A): steps (A1: hover 600 to 1000, to the flip's floor near 290 and to 1, each and back, 1.5 s apiece), full (A2: 1100 for 1 s from hover), table (0 to the cap in steps of 100, 2 s each, for the thrust table).

--port is wifi (the default: the stand found by its broadcast on the local network), a host or host:port, or a serial device such as /dev/ttyUSB1.

The stand stamps each load-cell reading with its own clock; the laptop stamps every line as it arrives and maps the stand's clock onto its own by the earliest arrivals (the least delayed). The audio starts on the laptop's clock with pw-record's own buffering unknown, so analyze shifts the audio by the lag at which the sound's changes line up with the throttle's, or failing that the lag that best matches rpm² to thrust (thrust goes as rpm²).
"""

import argparse
import json
import socket
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


TCP_PORT = 3333
BEACON_PORT = 3334
UPDATE_PORT = 3335


class WifiPort:
    """The stand over Wi-Fi (fw/stand's TCP server), with the serial port's write, read, reset_input_buffer and close. A dropped connection is reopened on the next call; the stand stops the motor by itself while commands are missing."""

    def __init__(self, host, tcp_port):
        self.address = (host, tcp_port)
        self.sock = None
        self.connect()

    def connect(self):
        if self.sock:
            self.sock.close()
        self.sock = socket.create_connection(self.address, timeout=3)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(0.05)

    def reconnect(self, error):
        print(f"\n!! stand connection lost ({error}); reconnecting")
        while True:
            try:
                self.connect()
                return
            except OSError:
                time.sleep(0.5)

    def write(self, data):
        try:
            self.sock.sendall(data)
        except OSError as error:
            self.reconnect(error)

    def read(self, size):
        try:
            data = self.sock.recv(size)
        except (socket.timeout, BlockingIOError):
            return b""
        except OSError as error:
            self.reconnect(error)
            return b""
        if not data:
            self.reconnect("closed by the stand")
        return data

    def reset_input_buffer(self):
        while self.read(65536):
            pass

    def close(self):
        self.sock.close()


def find_stand(seconds=5.0):
    """The stand's address from its UDP broadcast."""
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("", BEACON_PORT))
    listener.settimeout(seconds)
    try:
        while True:
            parts = listener.recv(100).split()
            if len(parts) == 3 and parts[0] == b"STAND":
                return parts[1].decode(), int(parts[2])
    except socket.timeout:
        raise SystemExit(f"no stand broadcast in {seconds:.0f} s: is it powered and on the network? (or pass --port HOST or --port /dev/ttyUSB1)")
    finally:
        listener.close()


def open_port(name):
    if not name.startswith("/dev/"):
        host, _, tcp_port = name.partition(":")
        address = find_stand() if name == "wifi" else (host, int(tcp_port or TCP_PORT))
        port = WifiPort(*address)
        print(f"stand at {address[0]} over Wi-Fi")
        time.sleep(0.3)
        port.reset_input_buffer()
        return port
    # DTR and RTS released before opening: asserted, the board's auto-reset holds it in reset. What the port buffered while closed is dropped.
    port = serial.Serial()
    port.port, port.baudrate, port.timeout = name, BAUD, 0.05
    port.dtr = port.rts = False
    port.open()
    time.sleep(0.5)
    port.reset_input_buffer()
    return port


def read_lines(port, out, stop, seen=None):
    """Every line from the stand with the laptop's arrival time (monotonic s) until stop is set; seen, if given, collects cell 0's counts and the supply volts as (arrival, value) for the run's printout."""
    buffer = b""
    while not stop.is_set():
        chunk = port.read(4096)
        now = time.monotonic()
        if not chunk:
            continue
        buffer += chunk
        *lines, buffer = buffer.split(b"\n")
        for line in lines:
            text = line.decode(errors="replace").strip()
            out.write(f"{now:.6f} {text}\n")
            parts = text.split()
            if seen is None:
                continue
            try:
                if len(parts) == 4 and parts[0] == "F" and parts[1] == "0":
                    seen["cell"].append((now, int(parts[3])))
                elif len(parts) == 3 and parts[0] == "V":
                    seen["volts"].append((now, int(parts[2]) / 1000))
            except ValueError:
                pass


def mean_since(rows, since):
    values = [value for when, value in rows if when >= since]
    return sum(values) / len(values) if values else None


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
        seen = {"cell": [], "volts": []}
        reader = threading.Thread(target=read_lines, args=(port, out, stop, seen), daemon=True)
        reader.start()
        commands = []
        started = time.monotonic()
        # Each step's held thrust and supply as it ends, from its last half second: thrust from the tare's counts, roughly; analyze does it properly.
        per_count = load_config().get("grams_per_count", {}).get("0", 0)
        tare = None
        try:
            schedule = [(TARE_S, 0)] + plan
            for step, (seconds, throttle) in enumerate(schedule):
                begun = time.monotonic()
                print(f"{begun - started:5.1f} s  {step + 1:2d}/{len(schedule)}  throttle {throttle:4d}" + ("  (tare)" if step == 0 else ""), end="", flush=True)
                end = begun + seconds
                commands.append((begun, throttle))
                while time.monotonic() < end:
                    port.write(f"T {throttle}\n".encode())
                    time.sleep(COMMAND_S)
                counts = mean_since(seen["cell"], end - 0.5)
                volts = mean_since(seen["volts"], end - 0.5)
                if step == 0:
                    tare = mean_since(seen["cell"], end - 2.0)
                thrust = "" if counts is None or tare is None else f"{(counts - tare) * per_count:7.0f} g" if per_count else f"{counts - tare:9.0f} counts"
                print(f"  {thrust}" + (f"  {volts:6.2f} V" if volts is not None else "") + ("" if seen["cell"] and seen["cell"][-1][0] > end - 0.5 else "  no cell readings"))
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
    """The stand's lines as arrays: per cell (laptop time s, raw), the ESC pulses (laptop time s, us) and the supply (laptop time s, V), on the laptop's clock."""
    cells, pulses, supply, offsets = {0: [], 1: []}, [], [], []
    for line in (directory / "lines.txt").read_text().splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        arrived, kind = float(parts[0]), parts[1]
        try:
            if kind == "F":
                cell, stand_us, raw = int(parts[2]), int(parts[3]), int(parts[4])
                # The same reading can arrive more than once just after the port opens (seen on 2026-11-13): one per conversion time.
                if not cells[cell] or cells[cell][-1][0] != stand_us:
                    cells[cell].append((stand_us, raw))
            elif kind == "E":
                stand_us, pulse = int(parts[2]), int(parts[3])
                pulses.append((stand_us, pulse))
            elif kind == "V":
                stand_us = int(parts[2])
                supply.append((stand_us, int(parts[3]) / 1000))
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
    return {cell: to_laptop(rows) for cell, rows in cells.items()}, to_laptop(pulses), to_laptop(supply)


def scale(args):
    port = open_port(args.port)
    port.write(f"C {1 << args.cell}\n".encode())
    def mean_counts(seconds):
        values, seen, end = [], set(), time.monotonic() + seconds
        buffer = b""
        while time.monotonic() < end:
            buffer += port.read(4096)
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                parts = line.split()
                if len(parts) == 4 and parts[0] == b"F" and int(parts[1]) == args.cell and parts[2] not in seen:
                    seen.add(parts[2])
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
    port.write(f"Z {args.hz}\n".encode())
    stop = threading.Event()
    reasons = {1: "power-on", 3: "software", 4: "panic", 5: "interrupt watchdog", 6: "task watchdog", 9: "brownout"}
    def hold(pulse):
        buffer = b""
        while not stop.is_set():
            port.write(f"P {pulse}\n".encode())
            time.sleep(COMMAND_S)
            buffer += port.read(4096)
            *lines, buffer = buffer.split(b"\n")
            for line in lines:
                parts = line.split()
                # A reset drops the pulse for the half second the board takes to boot: an ESC powered up in that gap never sees the top and arms as usual instead.
                if parts[:1] == [b"R"] and len(parts) == 2:
                    print(f"\n!! the stand reset ({reasons.get(int(parts[1]), parts[1].decode())}); if that was as the battery went on, the ESC missed the top. Fix the reset and run esc-cal again.")
                elif parts[:1] == [b"E"] and len(parts) >= 3:
                    print(f"   stand sending {int(parts[2])} us" + (f" at {int(parts[3])} Hz" if len(parts) > 3 else ""))
    for pulse, prompt in ((args.top, f"Sending the top ({args.top} us). Connect the battery, wait for the ESC's top-confirm beeps, then press Enter."),
                          (1000, "Sending the bottom (1000 us). Wait for the confirm beeps, then press Enter.")):
        stop.clear()
        thread = threading.Thread(target=hold, args=(pulse,), daemon=True)
        thread.start()
        input(prompt)
        stop.set()
        thread.join()
    port.write(b"T 0\n")
    port.close()
    print(f"done: if the ESC confirmed both, it now takes 1000-{args.top} us" + ("" if args.top == 2200 else " (the drone's firmware assumes 1000-2200 us; its throttle range has to follow)"))


def spin(args):
    if input(f"Props OFF, motor free to turn. Spins at throttle {args.throttle} ({1000 + args.throttle} us) for {args.seconds} s at {args.hz} Hz. Type go: ").strip() != "go":
        return
    port = open_port(args.port)
    port.write(f"Z {args.hz}\n".encode())
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        port.write(b"T 0\n")
        time.sleep(COMMAND_S)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        port.write(f"T {args.throttle}\n".encode())
        time.sleep(COMMAND_S)
    for _ in range(10):
        port.write(b"T 0\n")
        time.sleep(COMMAND_S)
    port.close()
    print("done: did the motor turn?")


def flash(args):
    image = Path(args.image).read_bytes()
    host = find_stand()[0] if args.port == "wifi" else args.port.partition(":")[0]
    print(f"sending {len(image)} bytes to the stand at {host}")
    with socket.create_connection((host, UPDATE_PORT), timeout=30) as connection:
        connection.sendall(len(image).to_bytes(4, "little") + image)
        answer = connection.makefile().readline().strip()
    if answer != "OK":
        raise SystemExit(f"update refused: {answer or 'no answer'}")
    print("written; the stand is restarting into it")
    # Back on the network a few seconds after its restart.
    print(f"stand back at {find_stand(15.0)[0]}")


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


def spectrum(path):
    """The audio's spectrogram, 0-4 kHz, in dB: frequencies (Hz), times (s from the audio's start) and power, 170 ms windows every 20 ms."""
    from scipy.signal import spectrogram
    with wave.open(str(path)) as audio:
        rate = audio.getframerate()
        samples = np.frombuffer(audio.readframes(audio.getnframes()), dtype=np.int16).astype(float)
    frequencies, times, power = spectrogram(samples, rate, nperseg=8192, noverlap=8192 - rate // 50)
    band = frequencies <= 4000
    return frequencies[band], times, 10 * np.log10(power[band] + 1e-12)


def step_lag(frequencies, times, power, changes, lags):
    """The audio's delay found from the motor's own sound, for runs whose thrust says nothing (props off): the lag at which the sound's changes (positive spectral flux over 300-4000 Hz, each tone moving when the speed does) best line up with the throttle's changes (s, on the run's clock)."""
    flux = np.clip(np.diff(power[frequencies >= 300], axis=1), 0, None).sum(axis=0)
    flux_t = times[1:]
    flux = (flux - np.median(flux)) / (np.std(flux) + 1e-9)
    def score(shift):
        # Each change as a 60 ms bump where the sound should move.
        expected = sum(np.exp(-0.5 * ((flux_t - shift - when) / 0.06) ** 2) for when in changes)
        return float(np.dot(flux, expected))
    scores = [score(shift) for shift in lags]
    return float(lags[int(np.argmax(scores))]), max(scores) / (np.sqrt(len(changes)) + 1e-9)


# An HX711 with its bridge disconnected reads near zero with little noise (2026-11-14: about 1,590 ± 10); a healthy one here rests near 256,000.
NO_SIGNAL_COUNTS = 20000
SATURATED = 8388000
CELLS_4S = 4


def run_checks(directory, meta, cells, pulses, supply, thrust):
    """What went wrong in a run, as (ok, label, detail): the link, the stand's resets, the load cell, the supply and the audio. The dashboard's stand page shows these."""
    start = meta["started_monotonic"]
    commands = meta["commands"]
    found = []
    arrivals, stand_us, restarts = [], [], 0
    for line in (directory / "lines.txt").read_text().splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[1] in "FEHV":
            try:
                arrived, us = float(parts[0]), int(parts[2] if parts[1] != "F" else parts[3])
            except ValueError:
                continue
            if stand_us and us < stand_us[-1] - 1_000_000:
                restarts += 1
            arrivals.append(arrived)
            stand_us.append(us)
    running = [when for when, throttle in commands if throttle > 0]
    if arrivals and running:
        inside = np.array([when for when in arrivals if running[0] <= when <= commands[-1][0] + 1.0])
        gaps = np.diff(inside) if len(inside) > 1 else np.zeros(1)
        worst = int(np.argmax(gaps))
        found.append((gaps[worst] < 0.3, "Link", f"longest silence {gaps[worst] * 1000:.0f} ms" + (f" at {inside[worst] - start:.1f} s (the motor stops after 300 ms without a command)" if gaps[worst] >= 0.3 else "")))
    if restarts:
        found.append((False, "Stand", f"restarted {restarts} times during the run: power or brownout"))
    # A stop pulse while the schedule still asked for thrust: the stand's watchdog cut the motor.
    commanded = lambda when: next((throttle for at, throttle in reversed(commands) if at <= when), 0)
    cut = [when - start for when, pulse in pulses if pulse <= 950 and when < commands[-1][0] and commanded(when) > 0]
    if cut:
        found.append((False, "Motor", f"stopped by the stand's watchdog at {', '.join(f'{when:.1f}' for when in cut[:5])} s: commands didn't arrive"))
    rows = cells[0]
    if not len(rows):
        found.append((False, "Load cell", "no readings: check the HX711's DT, SCK and power"))
    else:
        raw = rows[:, 1]
        seconds = rows[-1, 0] - rows[0, 0]
        rate = len(raw) / seconds if seconds > 0 else 0
        peak = float(np.max(thrust[0][1])) if 0 in thrust else 0.0
        if abs(np.mean(raw)) < NO_SIGNAL_COUNTS and np.std(raw) < 200:
            found.append((False, "Load cell", f"no signal (reads {np.mean(raw):,.0f} ± {np.std(raw):.0f}): check the cell's four wires to the HX711"))
        elif np.any(np.abs(raw) >= SATURATED):
            found.append((False, "Load cell", "saturated: overloaded or a wire off"))
        elif running and peak < 20:
            found.append((False, "Load cell", f"no thrust seen (peak {peak:.0f} g): props off, or the cell isn't loaded"))
        elif rate < 8:
            found.append((False, "Load cell", f"slow: {rate:.1f} readings/s"))
        else:
            found.append((True, "Load cell", f"{rate:.1f} readings/s, peak {peak:,.0f} g" + ("" if meta.get("scale") else " in counts: run sw.stand scale")))
    if not len(supply):
        found.append((False, "Supply", "not recorded (stand firmware before 2026-11-14)"))
    else:
        volts = supply[:, 1]
        rest = volts[supply[:, 0] < (running[0] if running else supply[-1, 0])]
        rest_v = float(np.median(rest)) if len(rest) else float(np.median(volts))
        low = float(np.percentile(volts, 1))
        if rest_v < 5:
            found.append((False, "Supply", f"reads {rest_v:.1f} V: no pack on the GPIO35 divider"))
        elif rest_v > 16.8 + 0.4:
            found.append((False, "Supply", f"{rest_v:.2f} V is above a full 4S: check the divider"))
        else:
            ok = rest_v / CELLS_4S >= 3.6 and low / CELLS_4S >= 3.3
            found.append((ok, "Supply", f"{rest_v:.2f} V at rest ({rest_v / CELLS_4S:.2f} V a cell), {low:.2f} V lowest under load ({low / CELLS_4S:.2f} V a cell)"))
    if not meta.get("audio") or not (directory / "audio.wav").exists():
        found.append((False, "Audio", "not recorded"))
    return found


def analyze(args):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    directory = Path(args.run)
    meta = json.loads((directory / "meta.json").read_text())
    cells, pulses, supply = parse(directory)
    start = meta["started_monotonic"]
    per_count = {int(cell): value for cell, value in meta.get("scale", {}).items()}
    thrust = {}
    for cell, rows in cells.items():
        if not len(rows):
            continue
        t = rows[:, 0] - start
        tare = rows[t < TARE_S - 0.5, 1].mean()
        grams = (rows[:, 1] - tare) * per_count.get(cell, 1.0)
        # Lift reads positive whichever way the scale's mass loaded the cell.
        thrust[cell] = (t, grams if np.mean(grams[t >= TARE_S]) >= 0 else -grams)
    checks = run_checks(directory, meta, cells, pulses, supply, thrust)
    for ok, label, detail in checks:
        print(f"check {'ok' if ok else 'FAIL'} {label}: {detail}")
    cell_ok = any(ok for ok, label, _ in checks if label == "Load cell")
    # A supply panel when the run recorded the voltage (stands wired on GPIO35 since 2026-11-14).
    panels = 5 if len(supply) else 4
    fig, axes = plt.subplots(panels, 1, figsize=(11, 3 * panels), sharex=True, gridspec_kw={"height_ratios": [1] * (panels - 1) + [2]})
    if len(supply):
        axes[2].plot(supply[:, 0] - start, supply[:, 1], lw=1, color="#555")
        axes[2].set_ylabel("supply (V)")
        axes = [axes[0], axes[1], axes[3], axes[4], axes[2]]
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
        frequencies, times, power = spectrum(directory / "audio.wav")
        times = times + meta["audio"]["started_monotonic"] - start
        # The audio's buffering delay, from the sound's own changes against the throttle's: the motor's tones move the moment its speed does.
        lag, strength = 0.0, 0.0
        if len(pulses) > 1:
            lag, strength = step_lag(frequencies, times, power, pulses[1:, 0] - start, np.arange(-1.0, 1.0, 0.005))
            print(f"audio lag {lag * 1000:.0f} ms from the throttle steps in the sound (match strength {strength:.1f})")
        if thrust:
            # The lag at which rpm² best follows the thrust: it includes the load cell's own delay (the HX711 averages over each conversion), so it is the fallback, and against the steps' lag it measures that delay.
            t, grams = next(iter(thrust.values()))
            grid = np.arange(max(t[0], audio_t[0]) + 0.5, min(t[-1], audio_t[-1]) - 0.5, 0.01)
            force = np.interp(grid, t, grams)
            lags = np.arange(-0.5, 0.5, 0.005)
            def score(shift):
                heard = np.interp(grid, audio_t - shift, rpm ** 2)
                valid = ~np.isnan(heard)
                return np.corrcoef(force[valid], heard[valid])[0, 1] if valid.sum() > 100 else np.nan
            scores = [score(shift) for shift in lags]
            if not np.all(np.isnan(scores)) and np.nanmax(scores) > 0.5:
                followed = float(lags[int(np.nanargmax(scores))])
                print(f"rpm² follows thrust at audio lag {followed * 1000:.0f} ms (correlation {np.nanmax(scores):.2f})" + (f": the load cell trails the sound by {(lag - followed) * 1000:.0f} ms" if strength >= 3 else ""))
                if strength < 3:
                    lag = followed
        floor = np.percentile(power, 20)
        axes[3].pcolormesh(times - lag, frequencies, power, shading="auto", cmap="magma", vmin=floor)
        if len(pulses):
            for when in pulses[1:, 0] - start:
                axes[3].axvline(when, color="#7fd3ff", lw=0.6, alpha=0.6)
        axes[3].set_ylabel("sound (Hz)")
        axes[2].plot(audio_t - lag, rpm, lw=1, color="#1f4e9c")
        rows = np.column_stack([audio_t - lag, rpm])
        np.savetxt(directory / "rpm.csv", rows, delimiter=",", header="seconds,rpm", comments="", fmt="%.4f")
    axes[2].set_ylabel(f"rpm (audio, {args.blades} blades)")
    axes[3].set_xlabel("seconds from the start of the run")
    # Each throttle change's response: when the first cell's thrust got 63% of the way to its new level.
    if thrust and cell_ok and len(pulses):
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
    if len(supply) and len(pulses):
        print("supply at the end of each step (V)")
        for i, (when, pulse) in enumerate(pulses[1:]):
            until = pulses[i + 2][0] if i + 2 < len(pulses) else supply[-1, 0]
            held = supply[(supply[:, 0] > until - 0.5) & (supply[:, 0] < until), 1]
            if len(held):
                print(f"  throttle {int(pulse) - 1000:4d}  {held.mean():6.2f}")
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
    calibrating = commands.add_parser("esc-cal")
    calibrating.add_argument("--top", type=int, default=2000, help="the top pulse (us); many generic ESCs take only up to 2000")
    calibrating.add_argument("--hz", type=int, default=400, help="PWM rate, 50-400")
    spinning = commands.add_parser("spin")
    spinning.add_argument("--throttle", type=int, default=50)
    spinning.add_argument("--seconds", type=float, default=2.0)
    spinning.add_argument("--hz", type=int, default=400)
    updating = commands.add_parser("flash")
    updating.add_argument("--image", default=str(Path(__file__).resolve().parent.parent / "fw" / "stand" / "build" / "stand.bin"))
    looking = commands.add_parser("analyze")
    looking.add_argument("run")
    looking.add_argument("--blades", type=int, default=2)
    for sub in (flying, weighing, calibrating, spinning, updating):
        sub.add_argument("--port", default="wifi")
    args = parser.parse_args()
    {"run": run, "scale": scale, "esc-cal": esc_cal, "spin": spin, "flash": flash, "analyze": analyze}[args.command](args)


if __name__ == "__main__":
    main()
