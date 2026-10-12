"""
Search PID gains against all near-best fitted plants, including actuator delay.
These simulated results need a hardware validation run.
"""

import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).parent
fitted = json.loads((ROOT / "fit-result.json").read_text())["near_best"]
# Deliberate robustness scenarios, not statistical confidence intervals.
models = [
    dict(m, b=m["b"] * scale, delay=m["delay"] + 0.04, tau=m["tau"] * 1.2)
    for m in fitted
    for scale in (0.8, 1, 1.2)
]
kp, ki, kd = np.meshgrid(
    np.arange(0.8, 2.51, 0.1),
    np.arange(0, 0.61, 0.05),
    np.arange(0.3, 1.61, 0.05),
    indexing="ij",
)
kp, ki, kd = [x.ravel() for x in (kp, ki, kd)]
worst = np.zeros(len(kp))
worst_peak = np.zeros(len(kp))
for model, initial in [(m, a) for m in models for a in (-49.0, 49.0)]:
    dt = 0.04
    steps = 625
    delay = round(model["delay"] / dt)
    angle = np.full(len(kp), initial)
    rate = np.zeros(len(kp))
    motor = np.zeros(len(kp))
    integral = np.zeros(len(kp))
    history = np.zeros((delay + 1, len(kp)))
    cost = np.zeros(len(kp))
    peak = np.zeros(len(kp))
    entered = np.zeros(len(kp), dtype=bool)
    for i in range(steps):
        target = 0 if i < 250 else (4 if i < 425 else -4)
        error = target - angle
        candidate = integral + ki * error * dt
        output = kp * error + candidate - kd * rate
        accept = (
            (np.abs(output) <= 200)
            | ((output > 200) & (error < 0))
            | ((output < -200) & (error > 0))
        )
        integral = np.where(accept, candidate, integral)
        u = np.clip(kp * error + integral - kd * rate, -200, 200)
        history[i % (delay + 1)] = u
        delayed = history[(i - delay) % (delay + 1)]
        motor = (
            delayed
            if model["tau"] == 0
            else motor + (1 - np.exp(-dt / model["tau"])) * (delayed - motor)
        )
        rate += dt * (
            model["b"] * motor - model["d"] * rate + model["g"] * angle + model["bias"]
        )
        angle = angle + dt * rate
        hit = np.abs(angle) > 49
        rate = np.where(hit, 0, rate)
        angle = np.clip(angle, -49, 49)
        entered |= np.abs(angle) < 30
        peak = np.maximum(peak, np.where(entered, np.abs(angle), 0))
        cost += dt * ((1 + i * dt * 0.2) * error**2 + 0.2 * rate**2 + 0.005 * u**2)
    cost += np.maximum(peak - 30, 0) ** 2 * 1000
    worst = np.maximum(worst, cost)
    worst_peak = np.maximum(worst_peak, peak)
indices = np.argsort(worst)[:10]
result = [
    dict(
        kp=round(float(kp[i]), 2),
        ki=round(float(ki[i]), 2),
        kd=round(float(kd[i]), 2),
        worst_cost=float(worst[i]),
        worst_peak_after_entry=float(worst_peak[i]),
    )
    for i in indices
]
result = dict(
    status="simulation only; no hardware recommendation",
    candidates_within_30_degrees=int(np.count_nonzero(worst_peak <= 30)),
    stress_scenarios="input gain 0.8/1/1.2 times fitted, +40 ms delay, 1.2 times motor lag",
    candidates=result,
)
(ROOT / "tuning-result.json").write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
