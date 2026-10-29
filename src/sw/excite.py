"""
Identification test patterns for the drone (fw/drone/main/flight_rt.c excite), loaded through the laptop service: uv run -m sw.excite <kind> [channel amplitudes] [options]

  prbs     each channel steps between ±amplitude on its own random bits, redrawn every --hold seconds, for --seconds
  doublet  each channel +amplitude for --width seconds, then -amplitude for --width, then zero for --rest
  chirp    each channel a ±amplitude square wave sweeping --low to --high Hz over --seconds
  file     a pattern JSON file as POST /pattern takes it ({"after", "repeat", "segments": [{"ticks", "<channel>": value}]})
  clear    remove the pattern

Channels (fw/controller/log_format.py PATTERN_CHANNELS): --roll, --pitch (target, degrees), --yaw (yaw-rate target, degrees/s), --base (collective, throttle), --left-servo, --right-servo (degrees, positive forward), --left-motor, --right-motor (throttle).
The pattern starts --after seconds past collective first reaching integrate_throttle and plays --repeat times (0 = until disarmed). It lasts until the drone restarts; disarm first. --dry prints it without sending.
Patterns are plain lists of segments, so other shapes are a few lines with segments() and send() from Python.
"""

import argparse
import json
import math
import random
import sys
import urllib.request
from pathlib import Path

from fw.controller.log_format import PATTERN_CHANNELS

SAMPLE_HZ = 100
URL = "http://127.0.0.1:8765/pattern"


def segments(values, hold):
    """Segments from per-tick channel values ([tick] of {channel: value}), merging equal neighbours; hold scales ticks (1 = one control sample each)."""
    out = []
    for value in values:
        value = {key: round(v, 2) for key, v in value.items() if v}
        if out and {k: v for k, v in out[-1].items() if k != "ticks"} == value:
            out[-1]["ticks"] += hold
        else:
            out.append({"ticks": hold, **value})
    return out


def prbs(amplitudes, hold, seconds, seed):
    rng = random.Random(seed)
    ticks = max(1, round(hold * SAMPLE_HZ))
    return segments([{key: a * rng.choice((-1, 1)) for key, a in amplitudes.items()} for _ in range(max(1, round(seconds / hold)))], ticks)


def doublet(amplitudes, width, rest):
    ticks = max(1, round(width * SAMPLE_HZ))
    out = segments([amplitudes, {key: -a for key, a in amplitudes.items()}], ticks)
    return out + ([{"ticks": round(rest * SAMPLE_HZ)}] if rest > 0 else [])


def chirp(amplitudes, low, high, seconds):
    n = round(seconds * SAMPLE_HZ)
    # The sweep's phase (cycles) at each tick, exponential in frequency.
    k = math.log(high / low) / seconds if high != low else 0
    phase = [low * (math.expm1(k * i / SAMPLE_HZ) / k if k else i / SAMPLE_HZ) for i in range(n)]
    return segments([{key: a * (1 if math.sin(2 * math.pi * p) >= 0 else -1) for key, a in amplitudes.items()} for p in phase], 1)


def send(pattern, url=URL):
    request = urllib.request.Request(url, json.dumps(pattern).encode(), {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=10) as response:
            return json.loads(response.read())
    except urllib.error.HTTPError as error:
        raise SystemExit(f"{error.code}: {error.read().decode()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("kind", choices=("prbs", "doublet", "chirp", "file", "clear"))
    parser.add_argument("path", nargs="?", type=Path, help="the pattern file, for file")
    for name in PATTERN_CHANNELS:
        parser.add_argument("--" + name.replace("_", "-"), type=float, default=0.0)
    parser.add_argument("--after", type=float, default=2.0)
    parser.add_argument("--repeat", type=int, default=0)
    parser.add_argument("--hold", type=float, default=0.05)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--width", type=float, default=0.3)
    parser.add_argument("--rest", type=float, default=1.0)
    parser.add_argument("--low", type=float, default=0.5)
    parser.add_argument("--high", type=float, default=8.0)
    parser.add_argument("--dry", action="store_true")
    parser.add_argument("--url", default=URL)
    args = parser.parse_args()
    amplitudes = {name: getattr(args, name) for name in PATTERN_CHANNELS if getattr(args, name)}
    if args.kind in ("prbs", "doublet", "chirp") and not amplitudes:
        parser.error("give at least one channel amplitude")
    if args.kind == "file":
        pattern = json.loads(args.path.read_text())
    else:
        built = {"prbs": lambda: prbs(amplitudes, args.hold, args.seconds, args.seed), "doublet": lambda: doublet(amplitudes, args.width, args.rest),
                 "chirp": lambda: chirp(amplitudes, args.low, args.high, args.seconds), "clear": lambda: []}[args.kind]()
        pattern = {"after": args.after, "repeat": args.repeat, "segments": built}
    print(f"{len(pattern['segments'])} segments, {sum(s['ticks'] for s in pattern['segments']) / SAMPLE_HZ:.2f} s", file=sys.stderr)
    if args.dry:
        print(json.dumps(pattern))
    else:
        print(json.dumps(send(pattern, args.url)))


if __name__ == "__main__":
    main()
