#!/usr/bin/env python3
"""Generate explanatory figures for the Yunnan week-ahead benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = ROOT / "docs" / "yunnan_ieee24_118_results.json"
DEFAULT_OUTPUT = ROOT / "docs" / "images" / "yunnan_week_ahead"

COLORS = {
    "blue": "#2878B5",
    "green": "#2A9D6F",
    "red": "#D1495B",
    "amber": "#E39C37",
    "purple": "#7B61A8",
    "gray": "#69737D",
    "light": "#E9EEF2",
    "ink": "#263238",
}


def configure() -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "axes.edgecolor": "#AAB3BA",
        "axes.linewidth": .8,
        "axes.grid": True,
        "grid.color": "#D9E0E5",
        "grid.linewidth": .7,
        "grid.alpha": .8,
        "legend.frameon": False,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
    })


def save(fig, path: Path) -> None:
    fig.savefig(path, dpi=180, bbox_inches="tight", pad_inches=.12)
    plt.close(fig)


def draw_box(ax, xy, width, height, text, color, subtitle=None):
    box = FancyBboxPatch(
        xy, width, height, boxstyle="round,pad=0.02,rounding_size=0.025",
        linewidth=1.2, edgecolor=color, facecolor=color + "18",
    )
    ax.add_patch(box)
    x, y = xy
    ax.text(x + width / 2, y + height * .60, text, ha="center", va="center",
            color=COLORS["ink"], weight="medium")
    if subtitle:
        ax.text(x + width / 2, y + height * .28, subtitle, ha="center", va="center",
                color=COLORS["gray"], fontsize=8.5)
    return box


def arrow(ax, start, end, color=None):
    ax.add_patch(FancyArrowPatch(
        start, end, arrowstyle="-|>", mutation_scale=13,
        linewidth=1.2, color=color or COLORS["gray"],
        connectionstyle="arc3,rad=0",
    ))


def plot_information_architecture(data, output: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 5.1))
    ax.set_xlim(0, 12); ax.set_ylim(0, 5.1); ax.axis("off")

    draw_box(ax, (.3, 3.55), 2.25, 1.05, "Hidden IEEE-118 truth",
             COLORS["red"], "benchmark only")
    draw_box(ax, (3.15, 3.55), 2.25, 1.05, "Boundary observations",
             COLORS["amber"], "price + HVDC envelope")
    draw_box(ax, (6.05, 3.55), 2.25, 1.05, "Market simulation",
             COLORS["purple"], "local UC / reserve / DC plan")
    draw_box(ax, (9.15, 3.55), 2.25, 1.05, "Stage-2 scenarios",
             COLORS["purple"], "tight / loose")

    draw_box(ax, (.3, .65), 2.25, 1.05, "Yunnan local history",
             COLORS["blue"], "inflow + wind + solar + load")
    draw_box(ax, (3.15, .65), 2.25, 1.05, "Probabilistic forecast",
             COLORS["blue"], "joint physical trajectories")
    draw_box(ax, (6.05, .65), 2.25, 1.05, "Stage-3 scenarios",
             COLORS["green"], "forecast errors + outages")
    draw_box(ax, (8.95, 1.95), 2.65, 1.05, "Week-ahead optimizer",
             COLORS["green"], "maintenance / boundary / DR")

    arrow(ax, (2.55, 4.08), (3.15, 4.08))
    arrow(ax, (5.40, 4.08), (6.05, 4.08))
    arrow(ax, (8.30, 4.08), (9.15, 4.08))
    arrow(ax, (2.55, 1.18), (3.15, 1.18))
    arrow(ax, (5.40, 1.18), (6.05, 1.18))
    arrow(ax, (10.28, 3.55), (10.28, 3.02))
    arrow(ax, (8.30, 1.18), (8.95, 2.27))

    ax.plot([2.85, 2.85], [2.05, 4.65], color=COLORS["red"], linestyle="--", linewidth=1.2)
    ax.text(2.72, 2.35, "information boundary", rotation=90, ha="right", va="center",
            color=COLORS["red"], fontsize=9)
    ax.text(.35, 4.83, "Not observable by Yunnan", color=COLORS["red"], fontsize=9)
    ax.text(3.18, 4.83, "Yunnan-observable interface", color=COLORS["green"], fontsize=9)
    save(fig, output / "01_information_architecture.png")


def plot_scenario_tree(data, output: Path) -> None:
    tree = data["uncertainty_quantification"]["joint_tree"]
    fig, ax = plt.subplots(figsize=(11, 5.2))
    ax.set_xlim(0, 10.8); ax.set_ylim(-.4, 4.75); ax.axis("off")

    root = (1.0, 1.95)
    market_pos = {"tight": (4.0, 3.15), "loose": (4.0, .75)}
    leaf_pos = {
        "tight_dry": (8.0, 3.60), "tight_normal": (8.0, 2.45),
        "loose_normal": (8.0, 1.15), "loose_wet": (8.0, .05),
    }
    draw_box(ax, (.15, 1.55), 1.7, .8, "D-day decision", COLORS["green"], "shared x")

    market_colors = {"tight": COLORS["red"], "loose": COLORS["blue"]}
    for market, pos in market_pos.items():
        probability = next(row["market_probability"] for row in tree if row["market"] == market)
        draw_box(ax, (pos[0] - .9, pos[1] - .4), 1.8, .8,
                 f"Market: {market}", market_colors[market], f"P = {probability:.2f}")
        arrow(ax, (1.85, 1.95), (pos[0] - .9, pos[1]))

    for row in tree:
        x, y = leaf_pos[row["leaf"]]
        color = COLORS["amber"] if row["physical"] in ("dry", "wet") else COLORS["green"]
        draw_box(ax, (x - 1.05, y - .35), 2.1, .7,
                 row["physical"].title(), color,
                 f"P|M={row['conditional_probability']:.3f}  joint={row['joint_probability']:.2f}")
        mx, my = market_pos[row["market"]]
        arrow(ax, (mx + .9, my), (x - 1.05, y))

    ax.text(3.15, 4.48, "Stage 2: simulated market result", color=COLORS["gray"])
    ax.text(6.95, 4.48, "Stage 3: forecast-error realization", color=COLORS["gray"])
    save(fig, output / "02_conditional_scenario_tree.png")


def plot_forecast_trajectories(data, output: Path) -> None:
    scenarios = data["uncertainty_quantification"]["physical_scenarios"]
    days = np.arange(1, 8)
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.8), sharex=True)
    fields = [
        ("load_factor_by_day", "Load multiplier"),
        ("wind_factor_by_day", "Wind multiplier"),
        ("solar_factor_by_day", "Solar multiplier"),
        ("inflow_factor_by_day", "Inflow multiplier"),
    ]
    palette = [COLORS["red"], COLORS["amber"], COLORS["blue"], COLORS["green"]]
    for ax, (field, title) in zip(axes.flat, fields):
        for scenario, color in zip(scenarios, palette):
            label = f"{scenario['parent_market']}/{scenario['name']}"
            ax.plot(days, scenario[field], marker="o", markersize=3.5,
                    linewidth=1.8, color=color, label=label)
        ax.axhline(1.0, color=COLORS["gray"], linewidth=.9, linestyle="--")
        ax.set_title(title)
        ax.set_xticks(days, [f"D+{d}" for d in days])
        ax.set_ylabel("factor")
        ax.spines[["top", "right"]].set_visible(False)
    axes[0, 0].legend(ncol=2, fontsize=8, loc="best")
    fig.tight_layout()
    save(fig, output / "03_physical_forecast_trajectories.png")


def scheme_name(name: str) -> str:
    return {
        "逐日滚动基线": "Rolling DA",
        "确定性周前瞻": "Deterministic WA",
        "随机周前瞻": "Stochastic WA",
    }.get(name, name)


def add_bar_labels(ax, bars, fmt="{:.2f}"):
    for bar in bars:
        value = bar.get_height()
        ax.annotate(fmt.format(value),
                    (bar.get_x() + bar.get_width() / 2, value),
                    xytext=(0, 4), textcoords="offset points",
                    ha="center", va="bottom", fontsize=8.5)


def plot_scheme_effectiveness(data, output: Path) -> None:
    schemes = data["schemes"]
    names = [scheme_name(s["name"]) for s in schemes]
    colors = [COLORS["gray"], COLORS["blue"], COLORS["green"]]
    metrics = [
        ("expected_eens_gwh", "Expected unserved energy", "GWh"),
        ("expected_curtailment_gwh", "Renewable curtailment", "GWh"),
        ("expected_export_gwh", "HVDC export", "GWh"),
        ("expected_terminal_water_gwh", "Terminal water energy", "GWh"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.0))
    for ax, (field, title, unit) in zip(axes.flat, metrics):
        values = [s[field] for s in schemes]
        bars = ax.bar(names, values, color=colors, width=.64)
        add_bar_labels(ax, bars, "{:.2f}")
        ax.set_title(title); ax.set_ylabel(unit)
        ax.tick_params(axis="x", rotation=12)
        ax.spines[["top", "right"]].set_visible(False)
        ax.margins(y=.16)
    fig.tight_layout()
    save(fig, output / "04_scheme_effectiveness.png")


def plot_leaf_risk(data, output: Path) -> None:
    baseline = data["schemes"][0]["leaves"]
    stochastic = data["schemes"][2]["leaves"]
    leaves = [row["leaf"] for row in data["uncertainty_quantification"]["joint_tree"]]
    labels = [name.replace("_", "\n") for name in leaves]
    x = np.arange(len(leaves)); width = .36
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    for ax, field, title in [
        (axes[0], "eens_gwh", "Unserved energy by leaf"),
        (axes[1], "curtailment_gwh", "Curtailment by leaf"),
    ]:
        bvals = [baseline[name][field] for name in leaves]
        svals = [stochastic[name][field] for name in leaves]
        b1 = ax.bar(x - width / 2, bvals, width, color=COLORS["gray"], label="Rolling DA")
        b2 = ax.bar(x + width / 2, svals, width, color=COLORS["green"], label="Stochastic WA")
        add_bar_labels(ax, b1, "{:.2f}"); add_bar_labels(ax, b2, "{:.2f}")
        ax.set_title(title); ax.set_ylabel("GWh")
        ax.set_xticks(x, labels); ax.spines[["top", "right"]].set_visible(False)
        ax.margins(y=.20)
    axes[0].legend()
    fig.tight_layout()
    save(fig, output / "05_leaf_risk_comparison.png")


def plot_package_frontier(data, output: Path) -> None:
    packages = data["package_evaluations"]
    schemes = {s["name"]: s["package"] for s in data["schemes"]}
    fig, ax = plt.subplots(figsize=(10.5, 5.6))
    feasible = [p for p in packages if p["risk_feasible"]]
    infeasible = [p for p in packages if not p["risk_feasible"]]
    for rows, color, label, marker in [
        (infeasible, COLORS["red"], "Risk-budget violation", "x"),
        (feasible, COLORS["blue"], "Risk feasible", "o"),
    ]:
        if rows:
            ax.scatter(
                [p["expected_curtailment_gwh"] for p in rows],
                [p["expected_objective_musd"] for p in rows],
                s=[35 + p["dr_reserve_mw"] * .35 for p in rows],
                color=color, marker=marker, alpha=.82, label=label,
            )
    selected = {
        schemes["确定性周前瞻"]: (COLORS["amber"], "Deterministic choice"),
        schemes["随机周前瞻"]: (COLORS["green"], "Stochastic choice"),
    }
    for package, (color, label) in selected.items():
        row = next(p for p in packages if p["package"] == package)
        ax.scatter(row["expected_curtailment_gwh"], row["expected_objective_musd"],
                   s=150, facecolor="none", edgecolor=color, linewidth=2.2, label=label)
        ax.annotate(package, (row["expected_curtailment_gwh"], row["expected_objective_musd"]),
                    xytext=(7, 7), textcoords="offset points", fontsize=8.5)
    ax.set_xlabel("Expected renewable curtailment (GWh)")
    ax.set_ylabel("Expected risk-adjusted objective (M$)")
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(ncol=2, loc="best")
    save(fig, output / "06_package_risk_frontier.png")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    data = json.loads(args.input.read_text())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    configure()
    plot_information_architecture(data, args.output_dir)
    plot_scenario_tree(data, args.output_dir)
    plot_forecast_trajectories(data, args.output_dir)
    plot_scheme_effectiveness(data, args.output_dir)
    plot_leaf_risk(data, args.output_dir)
    plot_package_frontier(data, args.output_dir)
    print(f"Wrote 6 figures to {args.output_dir}")


if __name__ == "__main__":
    main()
