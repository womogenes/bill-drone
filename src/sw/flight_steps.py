"""
The 2026-11-12 outdoor session's settings, one step per flight, stored through the running bridge: uv run -m sw.flight_steps [step] [--apply]

Without a step it lists them; with one it prints what that step stores against what the drone has now, and --apply stores it. The steps from "high" on need the firmware of william/flip-ready (src/fw/FLIP.md): older firmware refuses hover_climb past 1.5 and does not know tip_limit or flip_enable. Each outdoor step holds the ones before it, so any step can be applied from today's indoor settings. The sims behind each are in sw/sim/models/handoff_2026-11-12.md and NOTES.md. "indoor" puts the tag hold's values back.
"""

import argparse
import json
import urllib.request

BRIDGE = "http://127.0.0.1:8765/settings"
INDOOR = {"hover_camera": 1, "hover_center": 1, "hover_gps_tau": 0, "hover_tilt": 10, "hover_thrust_direction": 0, "hover_kd": 2.5}
GPS = {"hover_camera": 0, "hover_center": 0, "hover_gps_tau": 0.5, "hover_gps_place_tau": 2}
LADDER = GPS | {"hover_tilt": 10, "hover_thrust_direction": 0.8, "hover_kd": 1.5, "pitch_damping": 6, "hover_climb": 1.5, "hover_ceiling": 3, "pattern_tilt": 42, "pattern_distance": 4,
                "hover_gps_calibrate": 5, "roll_target_rate": 360}
HIGH = LADDER | {"hover_climb": 3, "hover_ceiling": 3.5}
# The staged tests after the first flip (src/fw/FLIP.md "After the first flip"): the hold 5 m up, the tip cut back at 45 (the flip is exempt while it runs), from 4.5 m.
HIGH5 = LADDER | {"hover_climb": 5, "hover_ceiling": 5.5, "tip_limit": 45, "flip_height": 4.5}
STEPS = {
    "indoor": ("the tag hold as flown on 2026-11-11", INDOOR),
    "gps": ("the GPS hold on the indoor gains: calm air only (held 32/28/13 of 32 in 0/1/2 m/s)", GPS),
    "tilt": ("hover_tilt 20 (32/32/31/22 at 0/1/2/3 m/s)", GPS | {"hover_tilt": 20}),
    "direction": ("hover_thrust_direction 0.8, not flown before (32/31/32/30)", GPS | {"hover_tilt": 20, "hover_thrust_direction": 0.8}),
    "kd": ("hover_kd 1.5 (32/32/32/31)", GPS | {"hover_tilt": 20, "hover_thrust_direction": 0.8, "hover_kd": 1.5}),
    "ladder": ("the roll-step ladder's test settings on the full GPS hold, 1.5 m up, as cleared on 2026-11-12 (hover_tilt 10: 20 made the hold's braking violent in flight); play the pattern from sw/sim/tilt_ladder.py --dry with sw/excite.py",
               LADDER),
    # From here on the firmware of william/flip-ready (src/fw/FLIP.md): the tip cut and the flip are settings, and the hold may climb to 3 m.
    "high": ("the ladder's hold 3 m up (hover_climb 3, hover_ceiling 3.5), the flip's height; no pattern", HIGH),
    "ladder55": ("roll steps to 55 degrees past the 45-degree tip cut: tip_limit 65, pattern_tilt 58, 3 m up; play tilt_ladder.py --amplitudes 40,45,50,55 --dry", HIGH | {"tip_limit": 65, "pattern_tilt": 58}),
    # From here on the firmware of william/flip-fit (src/fw/FLIP.md "After the first flip"): the hold may climb to 5 m and flip_enable 2 and 3 are the staged partial rolls.
    "high5": ("the ladder's hold 5 m up (hover_climb 5, hover_ceiling 5.5), no flip: check the height and the hold up there", HIGH5 | {"flip_enable": 0}),
    "burst60": ("stage D1: one burst of the flip's spin braked at once, aimed at 60 degrees and back, from the 5 m hold (the dashboard's Flip button, once per arming)", HIGH5 | {"flip_enable": 2}),
    "burst90": ("stage D2: the same aimed at 90 degrees, faster", HIGH5 | {"flip_enable": 3}),
    "flip": ("stage E: one roll flip from the 5 m hold, the tip cut back at 45", HIGH5 | {"flip_enable": 1}),
    "noflip": ("the flip off again (flip_enable 0) and the tip cut at 45", {"flip_enable": 0, "tip_limit": 45}),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("step", nargs="?", choices=list(STEPS))
    parser.add_argument("--apply", action="store_true", help="store the step's settings through the bridge")
    args = parser.parse_args()
    if not args.step:
        for name, (what, _) in STEPS.items():
            print(f"{name:10} {what}")
        return
    what, update = STEPS[args.step]
    current = json.load(urllib.request.urlopen(BRIDGE, timeout=5))["settings"]
    print(f"{args.step}: {what}")
    for name, value in update.items():
        print(f"  {name:24} {current.get(name)} -> {value}")
    if args.apply:
        request = urllib.request.Request(BRIDGE, json.dumps({"settings": update}).encode(), {"Content-Type": "application/json"})
        stored = json.load(urllib.request.urlopen(request, timeout=5))["settings"]
        wrong = {name: stored.get(name) for name, value in update.items() if stored.get(name) is None or abs(stored[name] - value) > 1e-3}
        print("stored" if not wrong else f"NOT stored as asked: {wrong}")


if __name__ == "__main__":
    main()
