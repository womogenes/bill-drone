"""
The burst harness with the MPC allocation (fw/drone/main/mpc.c) switchable in place of flip.c's greedy flip_allocate, flip.c itself untouched: python -m sw.sim.mpc.make_mpc HARNESS OUT (from src), HARNESS sw/sim/flip_firmware.py or a copy patched from it (the sagging pack's ffsag.py), OUT a module next to this file (sw/sim/mpc/ffmpc.py).

The library is built from a copy of flip.c with its flip_allocate renamed flip_allocate_greedy, recover.c compiled with flip_allocate as flip_allocate_greedy (the recovery keeps the greedy allocation), flight_math.c, mpc.c and mpc_hook.c, whose flip_allocate takes the flip's calls. fly() reads options["mpc"]: absent or None flies flip.c's allocation exactly; a dict flies the MPC with those MpcConfig fields over mpc_defaults, and "mode" (mpc_hook.c mpc_mode: 2, the default, the roll and the collective as the greedy split delivers them through the motors' lag; 1 everything as wanted over the whole horizon; 3 the greedy plan's own response as the reference) and "preview" (mode 3: the roll reference looks ahead along the quaternion burst's reference profile). Each lane's MPC is fed what was sent every tick (mpc_hook_sent) and told the throttle cap in newtons at the lane's thrust scale (mpc_hook_select). Each row gains nose_up and nose_down (degrees, the body's forward axis above and below the horizon at its furthest while the flip owns the motors) and mpc (the run's mean projected-gradient iterations and polished share).
"""

import re
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent


def build(main, out):
    """The harness's library at out from the firmware sources in main."""
    with tempfile.TemporaryDirectory() as folder:
        build_in(main, out, Path(folder))


def build_in(main, out, work):
    source = (main / "flip.c").read_text()
    renamed, count = re.subn(r"^void flip_allocate\(", "void flip_allocate_greedy(", source, flags=re.M)
    assert count == 1
    (work / "flip_greedy.c").write_text(renamed)
    common = ["cc", "-O2", "-fPIC", "-iquote", str(main), "-c"]
    objects = []
    for name, path, extra in (("flip", work / "flip_greedy.c", []), ("recover", main / "recover.c", ["-Dflip_allocate=flip_allocate_greedy"]), ("flight_math", main / "flight_math.c", []), ("mpc", main / "mpc.c", []), ("hook", HERE / "mpc_hook.c", [])):
        target = work / f"{name}.o"
        subprocess.run(common + extra + [str(path), "-o", str(target)], check=True)
        objects.append(str(target))
    subprocess.run(["cc", "-shared", *objects, "-o", str(out), "-lm"], check=True)


def patch(src):
    def swap(old, new, count=1):
        nonlocal src
        assert src.count(old) == count, old
        src = src.replace(old, new)

    src = re.sub(r"^MAIN = .*$", 'MAIN = Path(__file__).resolve().parents[3] / "fw/drone/main"', src, count=1, flags=re.M)
    src = re.sub(r'^LIBRARY = .*$', 'LIBRARY = Path(__file__).with_name("libflip_mpc.so")', src, count=1, flags=re.M)
    swap('SOURCES = [MAIN / "flip.c", MAIN / "recover.c", MAIN / "flight_math.c"]', 'SOURCES = [MAIN / "flip.c", MAIN / "recover.c", MAIN / "flight_math.c", MAIN / "mpc.c", MAIN / "mpc.h", Path(__file__).with_name("mpc_hook.c"), Path(__file__).with_name("make_mpc.py")]')
    swap('    subprocess.run(["cc", "-O2", "-fPIC", "-shared", "-iquote", str(MAIN), *map(str, SOURCES), "-o", str(scratch), "-lm"], check=True)\n',
         '    from sw.sim.mpc.make_mpc import build\n    build(MAIN, scratch)\n')
    swap("DEFAULTS = Params.in_dll(library, \"flip_defaults\")\n", '''DEFAULTS = Params.in_dll(library, "flip_defaults")


class MpcConfig(ctypes.Structure):
    _fields_ = [("servo_delay", ctypes.c_int * 2)] + [(name, ctypes.c_float) for name in ("servo_wn", "servo_zeta", "servo_slew", "common_max", "differential_max", "motor_tau", "assembly_ratio", "pitch_gain")] + [("weight", ctypes.c_float * 4), ("early", ctypes.c_float)] + [(name, ctypes.c_float) for name in ("tilt_effort", "differential_effort", "thrust_slew", "tilt_slew")] + [("iterations", ctypes.c_int), ("passes", ctypes.c_int)]


MPC_CONFIG = MpcConfig.in_dll(library, "mpc_config")
MPC_LAST = (ctypes.c_float * 12).in_dll(library, "mpc_last")
library.mpc_hook_init.argtypes = [ctypes.c_int]
library.mpc_hook_select.argtypes = [ctypes.c_int, ctypes.c_float, ctypes.c_void_p]
library.mpc_hook_sent.argtypes = [ctypes.c_int, ctypes.c_float * 2, ctypes.c_float * 2, ctypes.c_float]
''')
    swap("    options = job[12] if len(job) > 12 else {}\n", '''    options = job[12] if len(job) > 12 else {}
    # options["mpc"]: None or absent, flip.c's allocation; a dict, the MPC with those MpcConfig fields over mpc_defaults.
    mpc = options.get("mpc")
    library.mpc_hook_init(0 if mpc is None else int(mpc.get("mode", 3)))
    ctypes.c_int.in_dll(library, "mpc_preview").value = int((mpc or {}).get("preview", 0))
    for key, value in (mpc or {}).items():
        if key in ("mode", "preview"):
            continue
        if isinstance(value, (list, tuple)):
            getattr(MPC_CONFIG, key)[:] = value
        else:
            setattr(MPC_CONFIG, key, value)
''')
    swap("                library.flip_tick(ctypes.byref(run),", "                library.mpc_hook_select(i, flight.thrust(params.throttle_cap) * scales[i], ctypes.addressof(run))\n                library.flip_tick(ctypes.byref(run),")
    swap('            lane["indi"].sent(tilt[i], [flight.thrust(t) * scales[i] if t > 0 else 0.0 for t in (left, right)])\n',
         '            lane["indi"].sent(tilt[i], [flight.thrust(t) * scales[i] if t > 0 else 0.0 for t in (left, right)])\n'
         '            library.mpc_hook_sent(i, (ctypes.c_float * 2)(*tilt[i]), (ctypes.c_float * 2)(*[flight.thrust(t) * scales[i] if t > 0 else 0.0 for t in (left, right)]), R.DT)\n')
    swap('                    lane["rates"] = np.maximum(lane.get("rates", np.zeros(3)), np.degrees(np.abs(sim.omega[i])))\n',
         '                    lane["rates"] = np.maximum(lane.get("rates", np.zeros(3)), np.degrees(np.abs(sim.omega[i])))\n'
         '                    nose = math.degrees(math.asin(max(-1.0, min(1.0, float(R.rotation(sim.quat[i:i + 1])[0, 2, 0])))))\n'
         '                    lane["nose_up"], lane["nose_down"] = max(lane.get("nose_up", 0.0), nose), min(lane.get("nose_down", 0.0), nose)\n'
         '                    if options.get("mpc_trace"):\n'
         '                        lane.setdefault("mpc_trace", []).append((round(now - lane["started"], 2), nose, *np.degrees(sim.omega[i]), *sim.state.motor[i], *np.degrees(sim.tilt[i]), *lane["mpc_last"]))\n')
    swap('lane["indi"].sent(tilt[i], [', 'lane["mpc_last"] = list(MPC_LAST)\n            lane["indi"].sent(tilt[i], [')
    swap('"past90": lane.get("past90", 0.0),', '"past90": lane.get("past90", 0.0), "mpc_trace": lane.get("mpc_trace"), "nose_up": lane.get("nose_up", 0.0), "nose_down": lane.get("nose_down", 0.0),')
    swap("    rows = []\n    for i, lane in enumerate(lanes):\n", '''    rows = []
    calls = ctypes.c_long.in_dll(library, "mpc_calls").value
    mpc_stats = (ctypes.c_long.in_dll(library, "mpc_iterations").value / max(calls, 1), ctypes.c_long.in_dll(library, "mpc_polished").value / max(calls, 1))
    for i, lane in enumerate(lanes):
''')
    swap('        row = {"held": upright', '        row = {"mpc": mpc_stats, "held": upright')
    return src


if __name__ == "__main__":
    Path(sys.argv[2]).write_text(patch(Path(sys.argv[1]).read_text()))
