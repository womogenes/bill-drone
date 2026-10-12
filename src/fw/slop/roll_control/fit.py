"""
Fit an unstable roll plant with motor lag and delay to recorded responses.

Model: roll'' = b*m - d*roll' + g*roll + bias; tau*m' = u(t-delay)-m.
u = (left throttle - right throttle)/2. All fitted b, d, g are nonnegative.
The final third of each recording is held out from coefficient fitting.
"""

import itertools
import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).parent
DT = 0.02
WINDOW = 10


def load(path):
    obj = json.loads(path.read_text())
    if isinstance(obj, dict) and "count" in obj:
        trace = np.fromfile(path.with_suffix(".bin"), dtype="<f4").reshape(-1, 6)
        rows = [
            dict(
                received_s=float(t),
                imu=dict(roll=float(a), gyro=[float(v)]),
                actuators=dict(
                    left=dict(throttle=float(l)), right=dict(throttle=float(r))
                ),
                control=dict(
                    enabled=bool(l or r),
                    kp=obj["kp"],
                    ki=obj["ki"],
                    kd=obj["kd"],
                    base_throttle=obj["base"],
                    target=0,
                ),
            )
            for t, a, v, l, r, i in trace
        ]
    else:
        rows = obj if isinstance(obj, list) else obj["readings"]
    rows = [r for r in rows if r["imu"] and r["control"]["enabled"]]
    if len(rows) < 15:
        return None
    t = np.array([r["received_s"] for r in rows])
    roll = np.array([r["imu"]["roll"] for r in rows])
    rate = np.array([r["imu"]["gyro"][0] for r in rows])
    u = np.array(
        [
            (r["actuators"]["left"]["throttle"] - r["actuators"]["right"]["throttle"])
            / 2
            for r in rows
        ]
    )
    grid = np.arange(t[0], t[-1], DT)
    roll, rate, u = [np.interp(grid, t, x) for x in (roll, rate, u)]
    # Do not fit interpolated gaps, startup, or samples near an endstop.
    index = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    valid = (np.abs(roll) < 30) & (t[index] - t[index - 1] < 0.2) & (grid > grid[0] + 1)
    mask = np.convolve(valid.astype(int), np.ones(WINDOW + 1), "valid") == WINDOW + 1
    if np.count_nonzero(mask) < 10:
        return None
    return dict(
        name=path.stem,
        t=grid,
        roll=roll,
        rate=rate,
        u=u,
        mask=mask,
        settings=rows[0]["control"],
        received_hz=(len(t) - 1) / (t[-1] - t[0]),
    )


def integral(x):
    return np.convolve((x[:-1] + x[1:]) / 2, np.ones(WINDOW) * DT, "valid")


def design(run, delay, tau):
    delayed = np.interp(run["t"] - delay, run["t"], run["u"])
    motor = delayed.copy()
    if tau:
        alpha = 1 - np.exp(-DT / tau)
        for i in range(1, len(motor)):
            motor[i] = motor[i - 1] + alpha * (delayed[i] - motor[i - 1])
    x = np.column_stack(
        [
            integral(motor),
            -integral(run["rate"]),
            integral(run["roll"]),
            np.full(len(motor) - WINDOW, WINDOW * DT),
        ]
    )
    y = run["rate"][WINDOW:] - run["rate"][:-WINDOW]
    return x, y


def constrained_fit(x, y):
    best = None
    for bits in itertools.product([False, True], repeat=3):
        cols = list(np.flatnonzero(bits)) + [3]
        values = np.linalg.lstsq(x[:, cols], y, rcond=None)[0]
        if np.any(values[:-1] < 0):
            continue
        coef = np.zeros(4)
        coef[cols] = values
        loss = np.mean((x @ coef - y) ** 2)
        if best is None or loss < best[0]:
            best = loss, coef
    return best[1]


runs = [
    run
    for p in sorted(ROOT.glob("*.json"))
    if p.name == "baseline.json" or p.stem[:8].isdigit()
    if (run := load(p)) is not None
]
models = []
for delay in np.arange(0, 0.401, 0.02):
    for tau in np.arange(0, 0.401, 0.04):
        train_x, train_y, test_x, test_y = [], [], [], []
        for run in runs:
            x, y = design(run, delay, tau)
            split = int(len(y) * 2 / 3)
            train = run["mask"] & (np.arange(len(y)) < split)
            test = run["mask"] & (np.arange(len(y)) >= split)
            train_x.append(x[train] / np.sqrt(max(1, train.sum())))
            train_y.append(y[train] / np.sqrt(max(1, train.sum())))
            test_x.append(x[test] / np.sqrt(max(1, test.sum())))
            test_y.append(y[test] / np.sqrt(max(1, test.sum())))
        x, y = np.concatenate(train_x), np.concatenate(train_y)
        xv, yv = np.concatenate(test_x), np.concatenate(test_y)
        coef = constrained_fit(x, y)
        models.append(
            dict(
                delay=float(delay),
                tau=float(tau),
                b=float(coef[0]),
                d=float(coef[1]),
                g=float(coef[2]),
                bias=float(coef[3]),
                train_rmse=float(np.sqrt(np.sum((x @ coef - y) ** 2) / len(runs))),
                validation_rmse=float(
                    np.sqrt(np.sum((xv @ coef - yv) ** 2) / len(runs))
                ),
            )
        )
models.sort(key=lambda m: m["validation_rmse"])
best = models[0]
result = dict(
    model=__doc__,
    best=best,
    near_best=[
        m for m in models if m["validation_rmse"] < best["validation_rmse"] * 1.05
    ],
    recordings=[
        dict(
            name=r["name"],
            settings=r["settings"],
            received_hz=r["received_hz"],
            roll_min=float(min(r["roll"])),
            roll_max=float(max(r["roll"])),
        )
        for r in runs
    ],
)
(ROOT / "fit-result.json").write_text(json.dumps(result, indent=2))
fig, axs = plt.subplots(len(runs), 3, figsize=(14, 3 * len(runs)), squeeze=False)
for r, ax in zip(runs, axs):
    t = r["t"]
    ax[0].plot(t, r["roll"])
    ax[0].set_ylabel("Roll (degrees)")
    ax[0].set_title(r["name"])
    ax[1].plot(t, r["u"])
    ax[1].set_ylabel("Differential throttle")
    x, y = design(r, best["delay"], best["tau"])
    pred = x @ np.array([best[k] for k in ("b", "d", "g", "bias")])
    ax[2].plot(t[WINDOW:], y, label="Measured")
    ax[2].plot(t[WINDOW:], pred, label="Model")
    ax[2].set_ylabel("Rate change over 0.2 s (deg/s)")
    ax[2].legend()
    for a in ax:
        a.set_xlabel("Time (s)")
        a.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(ROOT / "fit.png", dpi=150)
print(json.dumps(result, indent=2)[:6000])
