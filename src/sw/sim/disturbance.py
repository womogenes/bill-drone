"""
Disturbances for the host simulation (sw/sim/sweep.py plant["disturbances"]): any sum of terms on six channels, horizontal and vertical forces in the world frame (forward toward the tag, left, up; m/s²) and torques about the body axes (roll, pitch, yaw; rad/s²).

A term is a dict with "channel" and "kind":
  constant  {"value"}: a steady push or torque (the room's wash, a thrust misalignment the airframe fit cannot see)
  drift     {"sd", "tau"}: a random force that wanders with that standard deviation and correlation time (s), an Ornstein-Uhlenbeck process
  white     {"sd"}: independent each control tick (fast jitter that only shakes)
  sine      {"amplitude", "period", "phase"}: a periodic force (phase in radians)
Terms on one channel add. sw/sim/fit_disturbances.py fits constants and drifts from flight logs.
"""

import math
import random

CHANNELS = ("forward", "left", "up", "roll", "pitch", "yaw")


class Disturbances:
    def __init__(self, terms, seed):
        self.terms = [dict(term) for term in terms or ()]
        for term in self.terms:
            if term["channel"] not in CHANNELS:
                raise ValueError(f"unknown disturbance channel {term['channel']!r}")
            term["state"] = 0.0
        self.rng = random.Random(seed + 7000)
        self.time = 0.0

    def step(self, dt):
        """The six channels' values for the next dt seconds."""
        self.time += dt
        out = [0.0] * len(CHANNELS)
        for term in self.terms:
            kind = term["kind"]
            if kind == "constant":
                value = term["value"]
            elif kind == "drift":
                keep = math.exp(-dt / term["tau"])
                term["state"] = keep * term["state"] + term["sd"] * math.sqrt(1 - keep * keep) * self.rng.gauss(0, 1)
                value = term["state"]
            elif kind == "white":
                value = term["sd"] * self.rng.gauss(0, 1)
            elif kind == "sine":
                value = term["amplitude"] * math.sin(2 * math.pi * self.time / term["period"] + term.get("phase", 0.0))
            else:
                raise ValueError(f"unknown disturbance kind {kind!r}")
            out[CHANNELS.index(term["channel"])] += value
        return out
