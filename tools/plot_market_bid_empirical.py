#!/usr/bin/env python3
"""Figures and paired summaries for the empirical bidding protocol."""

import argparse
import json
from pathlib import Path
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from market_bid_empirical import ROOT, save


def apply_figure_style():
    # Mechanics from figure-style/SKILL.md; local to avoid a runtime skill dependency.
    plt.rcParams.update({"font.family": "sans-serif", "font.size": 10, "axes.labelsize": 10,
        "axes.titlesize": 11, "legend.fontsize": 9, "xtick.labelsize": 9, "ytick.labelsize": 9,
        "axes.spines.top": False, "axes.spines.right": False, "xtick.direction": "out",
        "ytick.direction": "out", "legend.frameon": False, "savefig.dpi": 300,
        "pdf.fonttype": 42, "axes.titlelocation": "left"})


def export(fig, path):
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()
    width, height = fig.canvas.get_width_height()
    for ax in fig.axes:
        box = ax.get_tightbbox(renderer)
        if box.x0 < -1 or box.y0 < -1 or box.x1 > width+1 or box.y1 > height+1:
            raise RuntimeError("Figure content outside canvas")
    for suffix in ("png", "pdf"):
        fig.savefig(path.with_suffix("."+suffix))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT/"output/market-bids")
    args = parser.parse_args()
    out = args.output
    summary = json.loads((out/"empirical_summary.json").read_text())
    runs = json.loads((out/"clearing_summary.json").read_text())
    if len(runs) != 27 or not all(r["valid"] for r in runs):
        raise ValueError("Figures require all 27 verified runs; failures cannot enter summaries")
    aggregates = {}
    for direction, info in summary["directions"].items():
        n = sum(d["paired_units"] for d in info["daily_scores"])
        aggregates[direction] = {"paired_unit_days": n}
        for model in ("persistence", "common_train", "common_expost"):
            aggregates[direction][model] = {
                "mae_aud_mwh": sum(d["paired_units"]*d[model]["mae_aud_mwh"] for d in info["daily_scores"])/n,
                "rmse_aud_mwh": (sum(d["paired_units"]*d[model]["rmse_aud_mwh"]**2 for d in info["daily_scores"])/n)**.5}
    lookup = {(r["day"], r["load_fraction"], r["method"]): r for r in runs}
    aggregates["clearing"] = {"runs": len(runs),
        "max_power_residual_mw": max(r["power_residual_mw"] for r in runs),
        "max_cost_relative_error": max(r["cost_relative_error"] for r in runs),
        "max_lmp_interval_error": max(r["lmp_interval_error"] for r in runs),
        "runtime_min_sec": min(r["wall_sec"] for r in runs),
        "runtime_max_sec": max(r["wall_sec"] for r in runs)}
    for method in ("unit_persistence", "common_train"):
        paired = [(r, lookup[(r["day"], r["load_fraction"], "actual_snapshot")]) for r in runs if r["method"] == method]
        aggregates["clearing"][method] = {"price_mae_artificial_units_mwh": statistics.mean(abs(r["lmp"]-a["lmp"]) for r, a in paired),
            "day_bid_cost_mae_artificial_units": statistics.mean(abs(r["day_cost"]-a["day_cost"]) for r, a in paired)}
    save(out/"aggregate_summary.json", aggregates)

    apply_figure_style()
    labels = {"persistence": "Unit persistence", "common_train": "Common factor (training)",
              "common_expost": "Common factor (hindsight)"}
    colors = {"persistence": "#707070", "common_train": "#007D86", "common_expost": "#AF4B77"}
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.8), layout="constrained")
    for ax, direction in zip(axes, ("GEN", "LOAD")):
        scores = summary["directions"][direction]["daily_scores"]
        for model in labels:
            ax.plot(range(3), [d[model]["rmse_aud_mwh"] for d in scores],
                    marker="s" if model == "persistence" else "o", markersize=7 if model == "persistence" else 4,
                    markerfacecolor="white" if model == "persistence" else colors[model],
                    linestyle="--" if model == "persistence" else "-", zorder=3 if model == "persistence" else 2,
                    label=labels[model], color=colors[model])
        ax.set_xticks(range(3), [f"09-{i:02d}\nn={s['paired_units']}" for i, s in enumerate(scores, 1)])
        ax.set(title="Generation offers" if direction == "GEN" else "Load-side energy bids",
               ylabel="Bid-curve RMSE (AUD/MWh)", xlabel="Final noon snapshot / paired DUIDs")
        ax.margins(.1)
    handles, texts = axes[0].get_legend_handles_labels()
    fig.legend(handles, texts, loc="outside upper center", ncol=3)
    export(fig, out/"bid_errors")

    methods = {"actual_snapshot": ("Actual bid shapes", "#B44835", "o"),
               "unit_persistence": ("Unit persistence", "#707070", "s"),
               "common_train": ("Common factor (training)", "#007D86", "^")}
    fig, axes = plt.subplots(1, 3, figsize=(10.2, 3.8), layout="constrained", sharey=True)
    for ax, day in zip(axes, summary["test_days"]):
        for method, (label, color, marker) in methods.items():
            rows = sorted((r for r in runs if r["day"] == day and r["method"] == method), key=lambda r: r["load_fraction"])
            ax.plot([r["load_fraction"]*100 for r in rows], [r["lmp"] for r in rows],
                    label=label, color=color, marker=marker, linestyle="-" if method == "actual_snapshot" else "--")
        ax.set_xticks([35, 65, 90])
        ax.set(title=f"{day[:4]}-{day[4:6]}-{day[6:]} / {rows[0]['active_units']} units",
               xlabel="Load / available capacity (%)")
        ax.axhline(0, color="#aaaaaa", linewidth=.6, zorder=0)
        ax.margins(.08)
    axes[0].set_ylabel("Model LMP (artificial units/MWh)")
    handles, texts = axes[0].get_legend_handles_labels()
    fig.legend(handles, texts, loc="outside upper center", ncol=3)
    export(fig, out/"clearing_prices")

    extracted = json.loads((out/"extracted.json").read_text())
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.8), layout="constrained", sharey=True)
    for ax, duid in zip(axes, ("GEN:TUMUT3", "GEN:MURRAY")):
        for day, color in zip(("20260831", "20260901", "20260903"), ("#707070", "#007D86", "#B44835")):
            row = extracted[day]["records"][duid]
            ax.stairs(row["prices_aud_mwh"], list(range(0, 101, 10)), baseline=None,
                      label=f"{day[4:6]}-{day[6:]}", color=color, linewidth=1.8,
                      linestyle="--" if day == "20260831" else "-", zorder=3 if day == "20260831" else 2)
        ax.set(title=duid.removeprefix("GEN:"), xlabel="Normalized offered capacity (%)")
        ax.margins(.06)
    axes[0].set_ylabel("Mean price in capacity decile (AUD/MWh)")
    handles, texts = axes[0].get_legend_handles_labels()
    fig.legend(handles, texts, loc="outside upper center", ncol=3)
    export(fig, out/"individual_bid_shapes")
    print(json.dumps(aggregates, indent=2))


if __name__ == "__main__":
    main()
