"""
Liftoff with an unmodelled steady pitch push: uv run -m sw.sim.slop.liftoff
Runs sw.sim.sweep's ground-start plant at T (720) with ±push rad/s², servo lag 80–120 ms and servo effect 1.0–1.4x, and prints peak pitch in the first second, settling (last |pitch| > 5°) and late pitch RMS per case.
"""

import itertools

from sw.sim.sweep import NOMINAL, run

BASE = {"roll_p": 40, "roll_i": 10, "roll_damping": 10, "yaw_damping": 10, "integral_limit": 20, "servo_compensation": 0, "velocity_damping": 0}
# com_x 0: NOMINAL's 2 mm already is a ~+10.7 rad/s² push; here the push is the explicit disturbance instead.
LIFT = NOMINAL | {"com_x": 0, "liftoff": True}
GAINS = [
    {"pitch_p": 10, "pitch_i": 10, "pitch_damping": 5, "pitch_bias": 0},
    {"pitch_p": 10, "pitch_i": 30, "pitch_damping": 7, "pitch_bias": 0},
    {"pitch_p": 10, "pitch_i": 30, "pitch_damping": 7, "pitch_bias": -7},
    {"pitch_p": 15, "pitch_i": 30, "pitch_damping": 7, "pitch_bias": -7},
]

if __name__ == "__main__":
    for gains in GAINS:
        print(gains)
        for push, delay, effect in itertools.product((6, 10, 14, -10), (0.02, 0.06), (1.0, 1.4)):
            plant = LIFT | {"disturbance": push, "servo_delay": delay, "height": NOMINAL["height"] * effect}
            r = run(BASE | gains, plant, seconds=6)
            lag = round((delay + plant["servo_tau"]) * 1000)
            if not r["stable"]:
                print(f"  push {push:+3} lag {lag} ms effect {effect}: tipped past 45° at {r['fail_s']:.2f} s")
                continue
            print(f"  push {push:+3} lag {lag} ms effect {effect}: peak {r['peak_pitch']:5.1f}°  settled {r['settle_s']:4.2f} s  late rms {r['pitch_rms']:.1f}°")
