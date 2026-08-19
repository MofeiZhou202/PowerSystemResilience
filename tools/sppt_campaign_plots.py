#!/usr/bin/env python3
"""Generate manuscript figures from the reproducible SPPT campaign CSV files."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


FAULT_LABELS = {
    "missing_vsc_ac_terminal": "VSC AC\nterminal",
    "missing_vsc_dc_terminal": "VSC DC\nterminal",
    "missing_ac_reference": "AC angle\nreference",
    "missing_dc_support": "DC voltage\nsupport",
    "invalid_converter_efficiency": "Converter\nefficiency",
    "duplicate_ac_bus_identity": "AC-bus\nidentity",
}
DETECTOR_LABELS = {
    "validation": "Boundary validation",
    "solver_only": "Convergence only",
    "full_sppt": "Full SPPT",
}
EDIT_LABELS = {
    "plausible_load_error": "Load edit",
    "plausible_converter_setpoint_error": "VSC setpoint edit",
}
AGENT_LABELS = {
    "scale_loads_10pct": "Scale loads\n+10%",
    "no_op": "Unchanged\nmodel",
    "hallucinate_vsc_ac_terminal": "Invalid VSC\nAC terminal",
    "hallucinate_vsc_dc_terminal": "Invalid VSC\nDC terminal",
    "remove_dc_voltage_support": "Remove DC\nsupport",
    "invalid_vsc_efficiency": "Efficiency\nabove unity",
}
COLORS = {
    "blue": "#356E9F",
    "orange": "#D8872A",
    "green": "#3B8A68",
    "red": "#B84A3A",
    "slate": "#637181",
    "light": "#E8EDF2",
    "ink": "#26323F",
}
THREE_PHASE_LABELS = {
    "abc-balanced-radial": "Balanced radial",
    "abc-unbalanced-lateral": "Unbalanced lateral",
    "abc-meshed-der": "Unbalanced meshed with DER",
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["DejaVu Serif"],
            "font.size": 8.2,
            "axes.labelsize": 8.5,
            "axes.titlesize": 9.2,
            "axes.titleweight": "bold",
            "xtick.labelsize": 7.6,
            "ytick.labelsize": 7.6,
            "legend.fontsize": 7.5,
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "savefig.bbox": "tight",
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.edgecolor": COLORS["slate"],
            "axes.linewidth": 0.7,
            "xtick.color": COLORS["ink"],
            "ytick.color": COLORS["ink"],
            "text.color": COLORS["ink"],
            "axes.labelcolor": COLORS["ink"],
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def save_figure(figure: plt.Figure, output_dir: Path, stem: str) -> None:
    for suffix in ("pdf", "png"):
        figure.savefig(output_dir / f"{stem}.{suffix}")
    plt.close(figure)


def plot_public_feeders(manifest_rows: list[dict[str, str]], output_dir: Path) -> None:
    systems: dict[str, dict[str, str]] = {}
    converter_counts: dict[str, int] = {}
    for row in manifest_rows:
        systems.setdefault(row["system"], row)
        converter_counts[row["system"]] = converter_counts.get(row["system"], 0) + 1

    assert len(systems) == 7
    assert all(count == 2 for count in converter_counts.values())
    rows = sorted(systems.values(), key=lambda row: int(row["ac_buses"]))
    assert [int(row["ac_buses"]) for row in rows] == [32, 78, 330, 823, 1105, 1976, 6986]
    assert all(int(row["import_skipped"]) == 0 for row in rows)
    assert all(int(row["dc_buses"]) == 2 and int(row["dc_branches"]) == 1 for row in rows)

    labels = [row["system"].replace("-ACDC", "") for row in rows]
    buses = np.array([int(row["ac_buses"]) for row in rows])
    branches = np.array([int(row["ac_branches"]) for row in rows])
    warnings = np.array([int(row["import_warnings"]) for row in rows])
    positions = np.arange(len(rows))

    figure = plt.figure(figsize=(7.15, 5.55), layout="constrained")
    grid = figure.add_gridspec(2, 2, height_ratios=[1.55, 1.15], width_ratios=[1.22, 1.18])
    scale_axis = figure.add_subplot(grid[0, :])
    warning_axis = figure.add_subplot(grid[1, 0])
    overlay_axis = figure.add_subplot(grid[1, 1])

    bar_height = 0.32
    scale_axis.barh(
        positions + bar_height / 2,
        buses,
        height=bar_height,
        color=COLORS["blue"],
        label="Imported AC buses",
    )
    scale_axis.barh(
        positions - bar_height / 2,
        branches,
        height=bar_height,
        color=COLORS["orange"],
        label="Imported AC branches",
    )
    scale_axis.set_xscale("log")
    scale_axis.set_xlim(15, 12000)
    scale_axis.set_yticks(positions, labels)
    scale_axis.set_xlabel("Imported positive-sequence network size (log scale)")
    scale_axis.set_title("(a) Public GridLAB-D distribution-feeder ladder", loc="left")
    scale_axis.grid(axis="x", which="both", color=COLORS["light"], linewidth=0.7)
    scale_axis.set_axisbelow(True)
    scale_axis.legend(frameon=False, ncols=2, loc="lower right")
    for position, value in zip(positions, buses):
        scale_axis.text(value * 1.06, position + bar_height / 2, f"{value:,}", va="center", fontsize=7.1)

    warning_axis.bar(positions, warnings, color=COLORS["slate"], width=0.62)
    warning_axis.set_xticks(positions, labels, rotation=32, ha="right")
    warning_axis.set_ylabel("Importer warnings (count)")
    warning_axis.set_title("(b) Declared conversion scope", loc="left")
    warning_axis.grid(axis="y", color=COLORS["light"], linewidth=0.7)
    warning_axis.set_axisbelow(True)
    for position, value in zip(positions, warnings):
        warning_axis.text(position, value + 1.1, str(value), ha="center", va="bottom", fontsize=7.1)
    warning_axis.text(
        0.01,
        0.96,
        "0 skipped objects in every feeder",
        transform=warning_axis.transAxes,
        va="top",
        fontsize=7.4,
        color=COLORS["green"],
    )

    overlay_axis.set_title("(c) Fully disclosed deterministic AC/DC overlay", loc="left")
    overlay_axis.set_xlim(0, 10)
    overlay_axis.set_ylim(0, 5.2)
    overlay_axis.axis("off")
    feeder = FancyBboxPatch(
        (0.6, 4.12), 8.8, 0.62, boxstyle="round,pad=0.08", facecolor="#E7F0F7", edgecolor=COLORS["blue"]
    )
    overlay_axis.add_patch(feeder)
    overlay_axis.text(5.0, 4.43, "Imported AC feeder: source and largest-load buses", ha="center", va="center", fontsize=7.1)
    for x_value, label, color in ((2.2, "PQ VSC", COLORS["orange"]), (7.8, "VDC-Q VSC", COLORS["green"])):
        box = FancyBboxPatch(
            (x_value - 0.82, 2.82), 1.64, 0.68, boxstyle="round,pad=0.08", facecolor="white", edgecolor=color
        )
        overlay_axis.add_patch(box)
        overlay_axis.text(x_value, 3.16, label, ha="center", va="center", fontsize=7.3)
    for x_value, label in ((3.25, "DC-P"), (6.75, "DC-V: 1.02 p.u.")):
        node = FancyBboxPatch(
            (x_value - 0.92, 1.80), 1.84, 0.62, boxstyle="round,pad=0.07",
            facecolor="#F2EAF5", edgecolor="#76558E", linewidth=1.0
        )
        overlay_axis.add_patch(node)
        overlay_axis.text(x_value, 2.11, label, ha="center", va="center", fontsize=6.6)
    arrow_style = dict(arrowstyle="-", color=COLORS["ink"], linewidth=1.2)
    for start, end in (((2.2, 4.12), (2.2, 3.50)), ((7.8, 4.12), (7.8, 3.50)), ((2.2, 2.82), (2.72, 2.42)), ((7.28, 2.42), (7.8, 2.82))):
        overlay_axis.add_patch(FancyArrowPatch(start, end, **arrow_style))
    overlay_axis.plot([3.25, 3.25, 6.75, 6.75], [1.80, 1.46, 1.46, 1.80], color=COLORS["ink"], linewidth=1.2)
    overlay_axis.text(5.0, 0.92, r"DC line: $r=0.02$ p.u.; both VSCs: $\eta=0.99$", ha="center", fontsize=6.9)
    overlay_axis.text(5.0, 0.22, r"$P_{\rm PQ}=\mathrm{clamp}(0.02P_{\rm load},\,0.001S_b,\,0.05S_b)$", ha="center", fontsize=6.8)

    save_figure(figure, output_dir, "sppt_public_feeder_variants")


def plot_detection(summary_rows: list[dict[str, str]], output_dir: Path) -> None:
    faults = list(FAULT_LABELS)
    detectors = list(DETECTOR_LABELS)
    lookup = {(row["detector"], row["fault"]): row for row in summary_rows}
    rates = np.zeros((len(detectors), len(faults)))
    samples = np.zeros_like(rates, dtype=int)
    localized = np.zeros_like(rates, dtype=int)
    for row_index, detector in enumerate(detectors):
        for column_index, fault in enumerate(faults):
            row = lookup[(detector, fault)]
            rates[row_index, column_index] = 100.0 * float(row["detection_rate"])
            samples[row_index, column_index] = int(row["samples"])
            localized[row_index, column_index] = int(row["localized"])

    assert np.all(rates[2] == 100.0)
    assert np.all(localized[2] == 175)
    assert rates[0, faults.index("missing_dc_support")] == 0.0
    assert np.all(samples[1] == 75)

    colormap = LinearSegmentedColormap.from_list("sppt_rate", ["#F5F6F7", "#D9E8F1", COLORS["blue"]])
    figure, axis = plt.subplots(figsize=(7.15, 2.75), layout="constrained")
    image = axis.imshow(rates, cmap=colormap, vmin=0, vmax=100, aspect="auto")
    axis.set_xticks(np.arange(len(faults)), [FAULT_LABELS[fault] for fault in faults])
    axis.set_yticks(np.arange(len(detectors)), [DETECTOR_LABELS[detector] for detector in detectors])
    axis.tick_params(top=False, bottom=False, left=False)
    for row_index in range(len(detectors)):
        for column_index in range(len(faults)):
            rate = rates[row_index, column_index]
            color = "white" if rate >= 70 else COLORS["ink"]
            axis.text(
                column_index,
                row_index,
                f"{rate:.0f}%\n$n={samples[row_index, column_index]}$",
                ha="center",
                va="center",
                fontsize=7.4,
                color=color,
            )
    axis.set_title("Structural-fault detection across public distribution-feeder variants", loc="left")
    colorbar = figure.colorbar(image, ax=axis, fraction=0.028, pad=0.02)
    colorbar.set_label("Detection rate (%)")
    axis.text(
        0.0,
        -0.28,
        "Full SPPT also localized 175/175 injections in every class; convergence-only scope: 32, 78, and 330 AC buses.",
        transform=axis.transAxes,
        fontsize=7.3,
        va="top",
    )
    save_figure(figure, output_dir, "sppt_structural_detection")


def quantile_triplet(values: list[float]) -> tuple[float, float, float]:
    array = np.asarray(values, dtype=float)
    return (
        float(np.median(array)),
        float(np.quantile(array, 0.95, method="nearest")),
        float(np.max(array)),
    )


def plot_intent_impacts(sample_rows: list[dict[str, str]], output_dir: Path) -> None:
    impact_rows = [
        row
        for row in sample_rows
        if row["fault"] in EDIT_LABELS and row["solver_attempted"] == "1" and row["solver_converged"] == "1"
    ]
    grouped = {
        fault: [row for row in impact_rows if row["fault"] == fault]
        for fault in EDIT_LABELS
    }
    assert all(len(rows) == 100 for rows in grouped.values())

    metrics = [
        ("max_ac_voltage_error_pu", r"Max $|\Delta V_{\rm ac}|$ (p.u.)", "(a) AC-voltage impact"),
        ("max_dc_voltage_error_pu", r"Max $|\Delta V_{\rm dc}|$ (p.u.)", "(b) DC-voltage impact"),
        ("max_branch_active_error_mw", r"Max branch $|\Delta P|$ (MW)", "(c) AC-branch-flow impact"),
        ("max_converter_active_error_mw", r"Max VSC $|\Delta P|$ (MW)", "(d) Converter-transfer impact"),
    ]
    statistic_labels = ["Median", "95th percentile", "Maximum"]
    statistic_markers = ["o", "s", "^"]
    statistic_colors = [COLORS["blue"], COLORS["orange"], COLORS["red"]]
    expected = {
        ("plausible_load_error", "max_ac_voltage_error_pu"): (0.000308932, 0.001651885, 0.004239302),
        ("plausible_load_error", "max_branch_active_error_mw"): (0.005237, 0.427329, 0.526453),
        ("plausible_converter_setpoint_error", "max_ac_voltage_error_pu"): (0.0001820835, 0.001828083, 0.001915688),
        ("plausible_converter_setpoint_error", "max_dc_voltage_error_pu"): (0.000037060, 0.000058440, 0.000063221),
        ("plausible_converter_setpoint_error", "max_branch_active_error_mw"): (0.0191505, 0.029956, 0.032028),
        ("plausible_converter_setpoint_error", "max_converter_active_error_mw"): (0.018897, 0.029803, 0.032231),
    }

    statistics: dict[tuple[str, str], tuple[float, float, float]] = {}
    for fault, rows in grouped.items():
        for field, _, _ in metrics:
            statistics[(fault, field)] = quantile_triplet([float(row[field]) for row in rows])
    for key, expected_triplet in expected.items():
        assert np.allclose(statistics[key], expected_triplet, rtol=2e-5, atol=1e-9), (key, statistics[key])

    figure, axes = plt.subplots(2, 2, figsize=(7.15, 5.25), layout="constrained")
    x_positions = np.arange(len(EDIT_LABELS))
    for axis, (field, ylabel, title) in zip(axes.flat, metrics):
        for statistic_index, (label, marker, color) in enumerate(
            zip(statistic_labels, statistic_markers, statistic_colors)
        ):
            values = [statistics[(fault, field)][statistic_index] for fault in EDIT_LABELS]
            offsets = x_positions + (statistic_index - 1) * 0.13
            axis.scatter(offsets, values, s=31, marker=marker, color=color, label=label, zorder=3)
        all_values = [value for fault in EDIT_LABELS for value in statistics[(fault, field)]]
        maximum = max(all_values)
        axis.set_ylim(0, maximum * 1.18 if maximum > 0 else 1.0)
        axis.set_xticks(x_positions, [EDIT_LABELS[fault] for fault in EDIT_LABELS])
        axis.set_ylabel(ylabel)
        axis.set_title(title, loc="left")
        axis.grid(axis="y", color=COLORS["light"], linewidth=0.7)
        axis.set_axisbelow(True)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="outside upper center", ncols=3, frameon=False)
    figure.text(
        0.5,
        -0.02,
        "100 converged edits per class on the 32-, 78-, 330-, and 823-bus variants; zero values denote no response in that metric.",
        ha="center",
        fontsize=7.3,
    )
    save_figure(figure, output_dir, "sppt_intent_error_impacts")


def plot_agent_campaign(agent_rows: list[dict[str, str]], output_dir: Path) -> None:
    assert len(agent_rows) == 42
    systems = sorted({row["system"] for row in agent_rows}, key=lambda name: int(next(
        row["ac_buses"] for row in agent_rows if row["system"] == name
    )))
    actions = list(AGENT_LABELS)
    lookup = {(row["system"], row["action"]): row for row in agent_rows}
    verdict = np.zeros((len(systems), len(actions)))
    recovery = np.zeros_like(verdict)
    for system_index, system in enumerate(systems):
        for action_index, action in enumerate(actions):
            row = lookup[(system, action)]
            expected = row["expected_admissible"] == "1"
            accepted = row["accepted"] == "1"
            committed = row["committed"] == "1"
            assert accepted == expected
            assert committed == expected
            assert row["trajectory_sound"] == "1"
            assert row["recovery_accepted"] == "1"
            verdict[system_index, action_index] = 1 if accepted else -1
            recovery[system_index, action_index] = 1

    figure, axes = plt.subplots(1, 2, figsize=(7.15, 3.35), layout="constrained",
                               gridspec_kw={"width_ratios": [1.18, 1.0]})
    cmap = LinearSegmentedColormap.from_list("agent", [COLORS["red"], "#F4F5F6", COLORS["green"]])
    axes[0].imshow(verdict, cmap=cmap, vmin=-1, vmax=1, aspect="auto")
    axes[0].set_title("(a) Assessment of proposed modifications", loc="left")
    axes[0].set_xticks(range(len(actions)), [AGENT_LABELS[action] for action in actions])
    axes[0].set_yticks(range(len(systems)), [name.replace("-ACDC", "") for name in systems])
    axes[0].tick_params(axis="x", rotation=27)
    for row_index in range(len(systems)):
        for column_index in range(len(actions)):
            axes[0].text(column_index, row_index,
                         "A" if verdict[row_index, column_index] > 0 else "R",
                         ha="center", va="center", fontsize=7.0, fontweight="bold", color="white")

    stages = ["Proposed\nchange", "Physical\nchecks", "Model\nselection", "Power\nflow", "Device\nassociation", "Following\ncheck"]
    positions = np.arange(len(stages))
    axes[1].plot(positions, np.ones(len(stages)), marker="o", linewidth=2,
                 color=COLORS["green"])
    axes[1].plot(positions, np.zeros(len(stages)), linewidth=1.5,
                 color=COLORS["slate"], linestyle="--")
    refused_colors = [COLORS["orange"], COLORS["red"], COLORS["blue"],
                      COLORS["slate"], COLORS["slate"], COLORS["green"]]
    refused_markers = ["s", "X", "o", "x", "x", "o"]
    for position, color, marker in zip(positions, refused_colors, refused_markers):
        axes[1].scatter(position, 0, color=color, marker=marker, s=29, zorder=3)
    axes[1].set_xticks(positions, stages)
    axes[1].set_yticks([0, 1], ["28 refused\nmodel retained", "14 accepted\nmodel adopted"])
    axes[1].set_xlim(-0.25, len(stages) - 0.75)
    axes[1].set_ylim(-0.28, 1.28)
    axes[1].grid(axis="x", color=COLORS["light"], linewidth=0.7)
    axes[1].set_title("(b) Sequential physical assessment", loc="left")
    axes[1].tick_params(axis="x", labelsize=6.7)
    axes[1].text(2, 0.16, "accepted model unchanged", ha="center", fontsize=6.7,
                 color=COLORS["blue"])
    save_figure(figure, output_dir, "sppt_agent_public_campaign")


def plot_three_phase_stress(stress_rows: list[dict[str, str]], output_dir: Path) -> None:
    assert len(stress_rows) == 18
    assert set(row["case"] for row in stress_rows) == set(THREE_PHASE_LABELS)
    series_colors = [COLORS["blue"], COLORS["orange"], COLORS["green"]]
    series_markers = ["o", "s", "^"]

    figure, axes = plt.subplots(1, 3, figsize=(7.15, 2.75), layout="constrained")
    for (case, label), color, marker in zip(THREE_PHASE_LABELS.items(), series_colors, series_markers):
        rows = sorted((row for row in stress_rows if row["case"] == case), key=lambda row: float(row["scale"]))
        scales = np.array([float(row["scale"]) for row in rows])
        ac_vmin = np.array([float(row["abc_vmin"]) for row in rows])
        vuf = np.array([float(row["abc_vuf_pct"]) for row in rows])
        dc_ok = np.array([row["boundary_converged"] == "yes" for row in rows])
        dc_vmin = np.array([float(row["vdc_min"]) for row in rows])

        axes[0].plot(scales, ac_vmin, color=color, marker=marker, linewidth=1.5, markersize=4, label=label)
        axes[1].plot(scales, vuf, color=color, marker=marker, linewidth=1.5, markersize=4, label=label)
        axes[2].plot(scales[dc_ok], dc_vmin[dc_ok], color=color, marker=marker, linewidth=1.5, markersize=4, label=label)
        if np.any(~dc_ok):
            axes[2].scatter(scales[~dc_ok], np.full(np.count_nonzero(~dc_ok), 0.24),
                            color=COLORS["red"], marker="x", s=28, linewidth=1.2, zorder=4)

    panel_settings = [
        (axes[0], "(a) AC phase-domain response", r"Minimum $|V_{abc}|$ (p.u.)", (0.94, 1.005)),
        (axes[1], "(b) Voltage unbalance", "Maximum VUF (%)", (-0.02, 0.90)),
        (axes[2], "(c) Coupled DC boundary", r"Minimum $v_{dc}$ (p.u.)", (0.22, 0.91)),
    ]
    for axis, title, ylabel, ylim in panel_settings:
        axis.set_title(title, loc="left")
        axis.set_xlabel("Load multiplier")
        axis.set_ylabel(ylabel)
        axis.set_xticks([0.5, 1.0, 1.5, 2.0])
        axis.set_ylim(*ylim)
        axis.grid(color=COLORS["light"], linewidth=0.7)
        axis.set_axisbelow(True)
    axes[2].text(1.98, 0.245, "not converged", ha="right", va="bottom", color=COLORS["red"], fontsize=6.8)
    handles, labels = axes[0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="outside upper center", ncols=3, frameon=False)
    save_figure(figure, output_dir, "sppt_three_phase_stress")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_dir", type=Path, help="Directory containing campaign CSV files")
    parser.add_argument("output_dir", type=Path, help="Destination for PDF and PNG figures")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    manifest = read_csv(args.input_dir / "sppt_public_benchmark_manifest.csv")
    summary = read_csv(args.input_dir / "sppt_fault_campaign_summary.csv")
    samples = read_csv(args.input_dir / "sppt_fault_campaign_samples.csv")
    agents = read_csv(args.input_dir / "sppt_agent_public_campaign.csv")
    stress = read_csv(args.input_dir / "sppt_three_phase_hybrid_pf_stress.csv")
    assert len(samples) == 1575

    configure_style()
    plot_public_feeders(manifest, args.output_dir)
    plot_detection(summary, args.output_dir)
    plot_intent_impacts(samples, args.output_dir)
    plot_agent_campaign(agents, args.output_dir)
    plot_three_phase_stress(stress, args.output_dir)
    print(f"Generated five SPPT figure pairs in {args.output_dir}")


if __name__ == "__main__":
    main()
