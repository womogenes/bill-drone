"""
Compare three recorded sweeps using the original 1.62 kg calibration scale.
Exclude the two confirmed 750-throttle glitches in the 0.97 kg run.
Report force changes relative to each run at throttle 200.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parent
BOTTLE_KG = 1.62
GRAVITY = 9.80665


def main():
    paths = {
        "no_bottle": ROOT / "readings_2026-10-04_22-32-57.csv",
        "bottle": ROOT / "readings_2026-10-04_22-40-47.csv",
        "bottle_097": ROOT / "readings_2026-10-04_22-51-32.csv",
    }
    runs = {name: pd.read_csv(path) for name, path in paths.items()}
    # CSV lines 339–340: duplicate -260280 readings amid ~120000 counts.
    # Preserve the raw CSV; exclude only these confirmed rows from analysis.
    runs["bottle_097"] = runs["bottle_097"].drop(index=[337, 338])
    masses = {name: runs[name].bottle_mass_kg.iloc[0] for name in ("bottle", "bottle_097")}
    table = pd.DataFrame()
    for name, df in runs.items():
        grouped = df.groupby("throttle").force
        table[f"{name}_median"] = grouped.median()
        table[f"{name}_q25"] = grouped.quantile(0.25)
        table[f"{name}_q75"] = grouped.quantile(0.75)
        table[f"{name}_samples"] = grouped.size()

    table["bottle_shift_counts"] = table.bottle_median - table.no_bottle_median
    table["signed_counts_per_kg"] = table.bottle_shift_counts / BOTTLE_KG
    signed_scale = table.signed_counts_per_kg.median()
    scale = abs(signed_scale)
    for name, mass in masses.items():
        table[f"{name}_offset_per_kg"] = (table.no_bottle_median - table[f"{name}_median"]) / mass
    for name in runs:
        baseline = table.loc[200, f"{name}_median"]
        table[f"{name}_relative_kgf"] = (table[f"{name}_median"] - baseline) / scale
        table[f"{name}_relative_N"] = table[f"{name}_relative_kgf"] * GRAVITY
    table.to_csv(ROOT / "comparison.csv", index_label="throttle")

    fig, axes = plt.subplots(1, 3, figsize=(16, 5), constrained_layout=True)
    colors = {"no_bottle": "tab:blue", "bottle": "tab:orange", "bottle_097": "tab:green"}
    labels = {"no_bottle": "No bottle", **{name: f"{mass:g} kg bottle" for name, mass in masses.items()}}
    for name, df in runs.items():
        color = colors[name]
        axes[0].scatter(df.throttle, df.force, s=7, alpha=0.2, color=color)
        axes[0].plot(table.index, table[f"{name}_median"], color=color,
                     marker=".", label=labels[name])
        axes[2].plot(table.index, table[f"{name}_relative_kgf"], color=color,
                     marker=".", label=labels[name])
        baseline = table.loc[200, f"{name}_median"]
        axes[2].fill_between(
            table.index,
            (table[f"{name}_q25"] - baseline) / scale,
            (table[f"{name}_q75"] - baseline) / scale,
            color=color, alpha=0.2,
        )

    axes[0].set(title=f"{sum(map(len, runs.values())):,} retained samples + step medians", ylabel="Raw load-cell counts")
    axes[0].legend()
    separation = -table.bottle_shift_counts
    for name in masses:
        axes[1].plot(table.index, table[f"{name}_offset_per_kg"], marker=".",
                     color=colors[name], label=labels[name])
    axes[1].axhline(scale, color="black", linestyle="--",
                    label=f"Original scale: {scale:,.0f}")
    axes[1].set(title="Bottle offset per kilogram",
                ylabel="(No-bottle median − bottle median) / mass (counts/kg)")
    axes[1].legend()
    axes[2].set(title="Approximate force change from throttle 200",
                ylabel="Force change (kgf; increasing-count direction)")
    axes[2].legend()
    secondary = axes[2].secondary_yaxis(
        "right", functions=(lambda x: x * GRAVITY, lambda x: x / GRAVITY)
    )
    secondary.set_ylabel("Force change (N)")
    for ax in axes:
        ax.set_xlabel("Throttle command")
        ax.grid(alpha=0.2)
    fig.suptitle("Three sweeps — two confirmed glitches excluded; original 1.62 kg calibration")
    fig.savefig(ROOT / "comparison.png", dpi=160)
    plt.close(fig)

    lines = [
        "Bottle calibration comparison",
        "",
        *[f"{name}: {path.name}; {len(runs[name])} samples" for name, path in paths.items()],
        "21 throttle settings, 200 through 1200; originally 30 samples per setting per run.",
        "Excluded only CSV lines 339–340 (data indices 337–338) in the 0.97 kg run:",
        "both throttle=750, force=-260280; the other 28 readings span 117198 to 122262.",
        "Raw CSVs are unchanged. 1,888 samples retained; no other filtering applied.",
        "Medians and interquartile ranges summarize each step.",
        "The older CSV has no bottle metadata and is treated as the no-bottle run.",
        "",
        f"Assumed bottle mass: {BOTTLE_KG} kg ({BOTTLE_KG * GRAVITY:.4f} N).",
        f"Median matched-throttle bottle shift: {table.bottle_shift_counts.median():,.1f} counts.",
        f"Signed bottle-direction sensitivity: {signed_scale:,.1f} counts/kg.",
        f"Sensitivity magnitude: {scale:,.1f} counts/kgf; {scale / GRAVITY:,.1f} counts/N.",
        f"Bottle separation spans {separation.min():,.1f} to {separation.max():,.1f} counts",
        f"({100 * (separation.max() / separation.min() - 1):.1f}% increase from minimum to maximum).",
        "This systematic variation is not explained by a single constant calibration offset.",
        "Possible contributors include differing motor thrust between runs, mounting/load-path",
        "changes, sensor nonlinearity, or drift. These recordings cannot distinguish them.",
        "",
        "Force changes use (counts - own-run median at throttle 200) / sensitivity magnitude.",
        "Positive is increasing counts, opposite the bottle's load direction.",
        "These are approximate changes, NOT absolute thrust: throttle 200 is not a motor-off tare.",
    ]
    for name in runs:
        peak = table[f"{name}_relative_kgf"].idxmax()
        lines.append(
            f"{name}: peak step median at throttle {peak}; change from throttle 200 = "
            f"{table.loc[peak, f'{name}_relative_kgf']:.3f} kgf "
            f"({table.loc[peak, f'{name}_relative_N']:.3f} N)."
        )
    for name, mass in masses.items():
        offset = table[f"{name}_offset_per_kg"]
        lines.append(
            f"{mass:g} kg bottle: inferred sensitivity magnitude spans {offset.min():,.1f} "
            f"to {offset.max():,.1f} counts/kg; median {offset.median():,.1f}."
        )
    lines.extend([
        "All three median curves peak at 1100 and decline at 1150 and 1200.",
        "All force curves retain the original 1.62 kg calibration scale for direct comparison.",
        "Firmware command 1000 means 2000 us; 1100 and 1200 mean 2100 and 2200 us.",
        "The observed peak alone does not establish an ESC limit or an optimal operating point.",
        "For a cleaner calibration, compare bottle on/off with the motor stopped and mounting unchanged.",
    ])
    report = "\n".join(lines) + "\n"
    (ROOT / "comparison.txt").write_text(report)
    print(report)


if __name__ == "__main__":
    main()
