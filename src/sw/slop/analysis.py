"""
Sweep ESC throttle from 200 through 1200 and save a timestamped CSV and plot.png.
The current firmware maps throttle 1200 to a 2200 us PWM pulse.
"""

import argparse
import json
from pathlib import Path
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import requests
from websocket import create_connection


def main():
    parser = argparse.ArgumentParser(
        description="Sweep throttle 200–1200 in steps of 50; save a timestamped CSV and plot.png beside this script."
    )
    parser.add_argument(
        "--bottle-attached",
        type=float,
        nargs="?",
        const=1.62,
        default=None,
        metavar="KG",
        help="Record an attached bottle's mass in kg (flag alone: 1.62 kg; omitted: no bottle).",
    )
    args = parser.parse_args()
    bottle_attached = args.bottle_attached is not None
    bottle_mass_kg = args.bottle_attached if bottle_attached else 0.0

    response = requests.get("https://kv.wfeng.dev/bill-esp32-drone-addr", timeout=5)
    response.raise_for_status()
    esp_ip = response.text.strip()
    print(f"Connecting to ESP at {esp_ip}...")
    ws = create_connection(f"ws://{esp_ip}/ws", timeout=5)
    readings = []

    try:
        for throttle in range(200, 1201, 50):
            print(f"Setting throttle to {throttle}")
            ws.send(str(throttle))

            # Drain incoming telemetry while the motor and load cell settle.
            settle_until = time.monotonic() + 1
            while time.monotonic() < settle_until:
                ws.recv()

            for _ in range(30):
                data = json.loads(ws.recv())
                readings.append({"throttle": throttle, "force": data["force"]})
    finally:
        try:
            ws.send("0")
        finally:
            ws.close()

    df = pd.DataFrame(readings)
    df["bottle_attached"] = bottle_attached
    df["bottle_mass_kg"] = bottle_mass_kg
    csv_path = Path(__file__).with_name(time.strftime("readings_%Y-%m-%d_%H-%M-%S.csv"))
    df.to_csv(csv_path, index=False)
    print(f"Saved {csv_path}")

    m, b = np.polyfit(df.throttle, df.force, 1)
    x = np.linspace(df.throttle.min(), df.throttle.max(), 100)

    fig, ax = plt.subplots()
    ax.scatter(df.throttle, df.force, s=12, alpha=0.5, label="Readings")
    ax.plot(x, m * x + b, color="red", label=f"Fit: y = {m:.1f}x + {b:.0f}")
    ax.set(xlabel="Throttle", ylabel="Load cell reading (raw counts)",
           title="Static motor test — " + (f"{bottle_mass_kg:g} kg bottle attached" if bottle_attached else "no bottle"))
    ax.legend()
    fig.tight_layout()
    output = Path(__file__).with_name("plot.png")
    fig.savefig(output, dpi=150)
    plt.close(fig)
    print(f"Saved {output}")


if __name__ == "__main__":
    main()
