#!/usr/bin/env python3
"""Generate vector manuscript figures from the versioned evidence data."""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.lines import Line2D


ROOT = Path(__file__).resolve().parent
DATA_PATH = ROOT / "figure_data.json"

BLUE = "#0072B2"
ORANGE = "#D55E00"
GREEN = "#009E73"
GOLD = "#E69F00"
PURPLE = "#7A5195"
DARK = "#252525"
MID = "#6B6B6B"
LIGHT = "#D9D9D9"
PALE_BLUE = "#DCECF5"
PALE_ORANGE = "#F7E2D8"
PALE_GREEN = "#DCEFE8"
PALE_GOLD = "#F7EBCF"
PALE_GRAY = "#F2F2F2"


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 8.5,
            "axes.titlesize": 9.5,
            "axes.labelsize": 8.5,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "legend.fontsize": 7.7,
            "axes.linewidth": 0.7,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.03,
        }
    )


def panel_title(ax: plt.Axes, label: str, title: str) -> None:
    ax.text(
        0.0,
        1.03,
        label,
        transform=ax.transAxes,
        fontsize=11,
        fontweight="bold",
        va="bottom",
        color=DARK,
    )
    ax.text(
        0.07,
        1.03,
        title,
        transform=ax.transAxes,
        fontsize=9.5,
        va="bottom",
        color=DARK,
    )


def add_box(
    ax: plt.Axes,
    xy: tuple[float, float],
    width: float,
    height: float,
    text: str,
    facecolor: str,
    edgecolor: str = DARK,
    fontsize: float = 8.2,
    linewidth: float = 0.8,
) -> None:
    box = patches.Rectangle(
        xy,
        width,
        height,
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=linewidth,
    )
    ax.add_patch(box)
    ax.text(
        xy[0] + width / 2,
        xy[1] + height / 2,
        text,
        ha="center",
        va="center",
        fontsize=fontsize,
        color=DARK,
        linespacing=1.2,
    )


def add_arrow(
    ax: plt.Axes,
    start: tuple[float, float],
    end: tuple[float, float],
    color: str = MID,
    linewidth: float = 1.0,
) -> None:
    ax.annotate(
        "",
        xy=end,
        xytext=start,
        arrowprops={
            "arrowstyle": "-|>",
            "color": color,
            "linewidth": linewidth,
            "shrinkA": 0,
            "shrinkB": 0,
        },
    )


def figure_problem_method() -> None:
    fig, axes = plt.subplots(1, 3, figsize=(11.2, 3.55))
    fig.subplots_adjust(wspace=0.16)

    ax = axes[0]
    ax.set(xlim=(0, 10), ylim=(0, 10))
    ax.axis("off")
    panel_title(ax, "a", "Limit-induced mode switching")
    add_box(ax, (0.2, 7.0), 2.6, 1.2, "Unconstrained\nconverter", PALE_BLUE)
    add_box(ax, (3.7, 7.0), 2.6, 1.2, "Inspect current,\npower and droop", PALE_GRAY)
    add_arrow(ax, (2.8, 7.6), (3.7, 7.6), BLUE)
    add_box(ax, (7.2, 8.2), 2.5, 1.05, "Voltage-control\nequations", PALE_BLUE)
    add_box(ax, (7.2, 6.5), 2.5, 1.05, "Current-limit\nequations", PALE_ORANGE)
    add_box(ax, (7.2, 4.8), 2.5, 1.05, "Droop-saturated\nequations", PALE_GOLD)
    add_arrow(ax, (6.3, 7.6), (7.2, 8.72))
    add_arrow(ax, (6.3, 7.6), (7.2, 7.02))
    add_arrow(ax, (6.3, 7.6), (7.2, 5.32))
    ax.text(5.0, 5.2, "external if--else", ha="center", color=ORANGE)
    ax.plot([1.2, 8.7], [3.7, 3.7], color=LIGHT, linewidth=0.8)
    add_box(ax, (0.5, 1.45), 2.6, 1.25, "Changing equation\nsemantics", PALE_ORANGE, ORANGE)
    add_box(ax, (3.7, 1.45), 2.6, 1.25, "Mode-order\nsensitivity", PALE_ORANGE, ORANGE)
    add_box(ax, (6.9, 1.45), 2.6, 1.25, "Sparse-pattern\nrebuilds", PALE_ORANGE, ORANGE)

    ax = axes[1]
    ax.set(xlim=(0, 10), ylim=(0, 10))
    ax.axis("off")
    panel_title(ax, "b", "Unified explicit converter model")
    ax.plot([0.6, 2.0], [7.2, 7.2], color=BLUE, linewidth=2.0)
    ax.plot([7.6, 9.2], [7.2, 7.2], color=ORANGE, linewidth=2.0)
    ax.text(0.55, 7.75, "AC network", ha="left", color=BLUE)
    ax.text(9.25, 7.75, "DC network", ha="right", color=ORANGE)
    add_box(ax, (3.7, 6.0), 2.6, 2.3, "VSC\nenergy\nbalance", PALE_GREEN, GREEN, 8.7, 1.0)
    add_arrow(ax, (2.0, 7.2), (3.7, 7.2), BLUE, 1.4)
    add_arrow(ax, (6.3, 7.2), (7.6, 7.2), ORANGE, 1.4)
    ax.text(2.8, 7.55, r"$P_{ac},Q_{ac}$", ha="center")
    ax.text(6.95, 7.55, r"$P_{dc}$", ha="center")
    source = patches.Circle((1.65, 4.25), 0.55, facecolor=PALE_BLUE, edgecolor=BLUE, linewidth=1.0)
    ax.add_patch(source)
    ax.text(1.65, 4.25, r"$E_c$", ha="center", va="center", fontsize=9)
    ax.plot([2.2, 3.7], [4.25, 4.25], color=DARK, linewidth=1.0)
    ax.text(2.95, 4.6, r"$Z_{v,c}$", ha="center")
    ax.plot([3.7, 3.7], [4.25, 6.0], color=DARK, linewidth=1.0)
    ax.text(1.35, 2.85, "GFM Norton source\nbehind virtual impedance", ha="center", color=MID, fontsize=7.7)
    add_box(
        ax,
        (4.2, 2.25),
        5.45,
        2.35,
        "$z_c=(P_{ac},Q_{ac},P_{dc},E_r,E_i,\\lambda)$\n\ncurrent disk  |  P/Q priority\nVdc droop  |  GFM current limit",
        PALE_GRAY,
        DARK,
        8.2,
    )
    ax.text(
        5.0,
        0.72,
        "shared state semantics across\nPF | quasi-steady | OPF replay | transient initialization",
        ha="center",
        fontsize=7.3,
        color=GREEN,
    )

    ax = axes[2]
    ax.set(xlim=(0, 10), ylim=(0, 10))
    ax.axis("off")
    panel_title(ax, "c", "Structure-preserving Newton solve")
    add_box(ax, (0.25, 7.7), 2.6, 1.15, "NCP residual", PALE_BLUE, BLUE)
    add_box(ax, (3.7, 7.7), 2.6, 1.15, "Selected generalized\nJacobian", PALE_GREEN, GREEN)
    add_box(ax, (7.15, 7.7), 2.6, 1.15, "Globalized step", PALE_GOLD, GOLD)
    add_arrow(ax, (2.95, 8.18), (3.65, 8.18))
    add_arrow(ax, (6.35, 8.18), (7.05, 8.18))
    ax.text(2.0, 6.1, r"$J=[A\ B;\ C\ D]$", fontsize=12, ha="center")
    ax.text(7.0, 6.1, r"$D=\mathrm{blkdiag}(D_1,\ldots,D_m)$", fontsize=8.6, ha="center")
    add_arrow(ax, (5.0, 5.45), (5.0, 4.5), GREEN, 1.4)
    add_box(ax, (1.1, 3.05), 6.7, 1.35, "$S=A-BD^{-1}C$\nlocal $6\\times6$ solves; no dense inverse", PALE_GREEN, GREEN, 8.8)
    add_arrow(ax, (7.8, 3.72), (9.15, 3.72), GREEN, 1.4)
    ax.text(9.25, 4.05, "reduced sparse\nnetwork solve", ha="center", fontsize=7.8)
    ax.text(5.0, 1.95, r"fixed coordinates  |  $O(m)$ local work", color=GREEN, fontsize=7.5, ha="center")
    ax.text(5.0, 1.3, "rcond and backward-error guards", color=GREEN, fontsize=7.5, ha="center")
    ax.text(5.0, 0.45, r"certificate failure $\Rightarrow$ complete sparse-LU fallback", ha="center", color=ORANGE, fontsize=7.7)

    fig.savefig(ROOT / "problem_method_overview.pdf")
    plt.close(fig)


def figure_priority_geometry() -> None:
    reference = (0.9, 0.8)
    radius = math.hypot(*reference)
    solutions = [
        ("Magnitude", (reference[0] / radius, reference[1] / radius), "radial scaling"),
        ("P-first", (reference[0], math.sqrt(1.0 - reference[0] ** 2)), "preserve active current"),
        ("Q-first", (math.sqrt(1.0 - reference[1] ** 2), reference[1]), "preserve reactive current"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.35), sharex=True, sharey=True)
    theta = [2.0 * math.pi * i / 360 for i in range(361)]
    circle_x = [math.cos(value) for value in theta]
    circle_y = [math.sin(value) for value in theta]

    for index, (ax, (title, solution, note)) in enumerate(zip(axes, solutions)):
        panel_title(ax, chr(ord("a") + index), title)
        ax.fill(circle_x, circle_y, color=PALE_BLUE, alpha=0.65)
        ax.plot(circle_x, circle_y, color=BLUE, linewidth=1.2)
        ax.axhline(0.0, color=LIGHT, linewidth=0.7)
        ax.axvline(0.0, color=LIGHT, linewidth=0.7)
        ax.plot([0.0, reference[0]], [0.0, reference[1]], color=MID, linewidth=0.9, linestyle="--")
        ax.scatter(*reference, marker="x", color=ORANGE, s=42, linewidth=1.5, zorder=4)
        ax.scatter(*solution, marker="o", color=GREEN, s=34, edgecolor="white", linewidth=0.6, zorder=5)
        ax.annotate("request", reference, xytext=(5, 7), textcoords="offset points", color=ORANGE)
        ax.annotate(
            "limited solution",
            solution,
            xytext=(-5, -14),
            textcoords="offset points",
            color=GREEN,
            ha="center",
        )
        if title == "P-first":
            ax.plot([solution[0], solution[0]], [solution[1], reference[1]], color=GREEN, linewidth=1.1)
        elif title == "Q-first":
            ax.plot([solution[0], reference[0]], [solution[1], solution[1]], color=GREEN, linewidth=1.1)
        ax.text(0.04, 0.92, note, transform=ax.transAxes, color=MID, fontsize=7.7)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(-0.1, 1.23)
        ax.set_ylim(-0.1, 1.23)
        ax.set_xlabel(r"$P_{ac}/(V I^{max})$")
        if index == 0:
            ax.set_ylabel(r"$Q_{ac}/(V I^{max})$")
        ax.set_xticks([0.0, 0.5, 1.0])
        ax.set_yticks([0.0, 0.5, 1.0])
        for spine in ax.spines.values():
            spine.set_color(MID)

    handles = [
        Line2D([0], [0], marker="x", color="none", markeredgecolor=ORANGE, markersize=6, label="unconstrained request"),
        Line2D([0], [0], marker="o", color="none", markerfacecolor=GREEN, markeredgecolor="white", markersize=6, label="NCP solution"),
        Line2D([0], [0], color=BLUE, linewidth=1.2, label="current circle"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.subplots_adjust(bottom=0.17, top=0.9, wspace=0.2)
    fig.savefig(ROOT / "priority_geometry.pdf")
    plt.close(fig)


def label_vertical_bars(ax: plt.Axes, bars) -> None:
    for bar in bars:
        value = bar.get_height()
        ax.annotate(
            f"{value:.0f}",
            (bar.get_x() + bar.get_width() / 2, value),
            xytext=(0, 3),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=7.4,
            color=DARK,
        )


def figure_benchmark_results(data: dict) -> None:
    cases = data["benchmarks"]
    labels = [case["label"] for case in cases]
    x = list(range(len(cases)))
    width = 0.34
    fig, axes = plt.subplots(2, 2, figsize=(10.6, 6.15))
    fig.subplots_adjust(hspace=0.42, wspace=0.28)

    ax = axes[0, 0]
    panel_title(ax, "a", "Newton-system dimension")
    full = ax.bar([v - width / 2 for v in x], [c["full_dimension"] for c in cases], width, color=MID, label="full LU")
    reduced = ax.bar([v + width / 2 for v in x], [c["reduced_dimension"] for c in cases], width, color=BLUE, label="Schur reduced")
    label_vertical_bars(ax, full)
    label_vertical_bars(ax, reduced)
    ax.set_ylabel("unknowns")
    ax.set_xticks(x, labels)
    ax.legend(frameon=False, loc="upper left")

    ax = axes[0, 1]
    panel_title(ax, "b", "Input structural nonzeros")
    full = ax.bar([v - width / 2 for v in x], [c["full_structural_nnz"] for c in cases], width, color=MID, label="full LU")
    reduced = ax.bar([v + width / 2 for v in x], [c["reduced_structural_nnz"] for c in cases], width, color=GREEN, label="Schur reduced")
    label_vertical_bars(ax, full)
    label_vertical_bars(ax, reduced)
    ax.set_ylabel("structural nnz")
    ax.set_xticks(x, labels)
    ax.legend(frameon=False, loc="upper left")

    ax = axes[1, 0]
    panel_title(ax, "c", "Measured Schur timing change")
    linear = ax.bar([v - width / 2 for v in x], [c["linear_change_pct"] for c in cases], width, color=PURPLE, label="linear solve")
    wall = ax.bar([v + width / 2 for v in x], [c["wall_change_pct"] for c in cases], width, color=GOLD, label="end to end")
    ax.axhline(0.0, color=DARK, linewidth=0.8)
    for bars in (linear, wall):
        for bar in bars:
            value = bar.get_height()
            ax.annotate(
                f"{value:+.1f}%",
                (bar.get_x() + bar.get_width() / 2, value),
                xytext=(0, 4 if value >= 0 else -12),
                textcoords="offset points",
                ha="center",
                va="bottom" if value >= 0 else "top",
                fontsize=7.6,
                color=DARK,
            )
    ax.set_ylabel("change from full LU (%)")
    ax.set_xticks(x, labels)
    ax.set_ylim(-24, 50)
    ax.legend(frameon=False, loc="upper right")
    ax.text(0.02, 0.04, "negative is faster", transform=ax.transAxes, color=GREEN)

    ax = axes[1, 1]
    panel_title(ax, "d", "Certified Schur attempts")
    accepted = [c["schur_accepted"] for c in cases]
    regular = [c["local_regular_rejections"] for c in cases]
    backward = [c["full_backward_error_rejections"] for c in cases]
    y = list(range(len(cases)))
    ax.barh(y, accepted, color=GREEN)
    ax.barh(y, regular, left=accepted, color=ORANGE)
    left = [accepted[i] + regular[i] for i in range(len(cases))]
    ax.barh(y, backward, left=left, color=PURPLE)
    for row, case in enumerate(cases):
        ax.text(case["schur_attempts"] + 0.5, row, f'{case["schur_attempts"]} attempts', va="center")
        ax.text(accepted[row] / 2, row, str(accepted[row]), va="center", ha="center", color="white", fontweight="bold")
        if regular[row] > 0:
            ax.text(accepted[row] + regular[row] / 2, row, str(regular[row]), va="center", ha="center", color="white", fontweight="bold")
        if backward[row] > 0:
            ax.text(left[row] + backward[row] / 2, row, str(backward[row]), va="center", ha="center", color="white", fontweight="bold")
    ax.set_yticks(y, labels)
    ax.set_xlabel("attempt count")
    ax.set_xlim(0, max(c["schur_attempts"] for c in cases) + 8)
    ax.invert_yaxis()

    for ax in axes.flat:
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(axis="y", color=LIGHT, linewidth=0.6, alpha=0.7)
        ax.set_axisbelow(True)

    protocol = data["benchmark_protocol"]
    fig.text(
        0.5,
        0.005,
        f'Release/KLU; {protocol["warmups"]} warmups and {protocol["repeats"]} alternating repetitions; medians reported.',
        ha="center",
        fontsize=7.6,
        color=MID,
    )
    fig.savefig(ROOT / "benchmark_results.pdf")
    plt.close(fig)


def main() -> None:
    configure_style()
    with DATA_PATH.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    figure_problem_method()
    figure_priority_geometry()
    figure_benchmark_results(data)


if __name__ == "__main__":
    main()
