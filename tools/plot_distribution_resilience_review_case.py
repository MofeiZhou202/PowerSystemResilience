#!/usr/bin/env python3
"""Plot the generated distribution-resilience review benchmark outputs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finish_figure(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path, bbox_inches="tight", metadata={"Creator": "HySim review benchmark"})
    plt.close(fig)


def plot_service(results: Path, figures: Path, report: dict) -> None:
    rows = read_csv(results / "service_curve.csv")
    hour = [float(row["hour"]) for row in rows]
    demand = [float(row["demand_mw"]) for row in rows]
    served = [float(row["served_mw"]) for row in rows]
    critical_shed = [float(row["critical_shed_mw"]) for row in rows]
    dynamic_service = [0.0 for _ in rows]
    if report["dynamic_certificate"]["proof_valid"]:
        dynamic_service = served

    fig, ax = plt.subplots(figsize=(7.1, 3.55))
    ax.step(hour, demand, where="post", color="#303030", linewidth=1.5, label="Demand")
    ax.step(hour, served, where="post", color="#187b59", linewidth=2.0, label="MIP-served load")
    ax.step(
        hour,
        dynamic_service,
        where="post",
        color="#b33a3a",
        linewidth=1.8,
        linestyle="--",
        zorder=4,
        label="DAE-admissible service credit",
    )
    ax.fill_between(
        hour,
        critical_shed,
        step="post",
        color="#e9a23b",
        alpha=0.35,
        label="Critical-load shed",
    )
    ax.set(xlabel="Event time (h)", ylabel="Power (MW)", xlim=(0, max(hour) + 1), ylim=(-0.10, None))
    ax.grid(axis="y", alpha=0.25)
    ax.legend(ncol=1, frameon=False, fontsize=8, loc="center right")
    finish_figure(fig, figures / "review_case_service_trajectory.pdf")


def plot_weak_links(results: Path, figures: Path) -> None:
    hazard = read_csv(results / "hazard_branch_risk.csv")
    weak = read_csv(results / "weak_link_ranking.csv")
    weak_by_key = {("AC", int(row["branch_index"])): row for row in weak}
    labels = [f"{row['domain']}-{row['branch_index']}" for row in hazard]
    fragility = [float(row["peak_failure_probability"]) for row in hazard]
    fmea = [
        float(weak_by_key.get((row["domain"], int(row["branch_index"])), {}).get("fmea_eens_mwh_yr", 0.0))
        for row in hazard
    ]
    positions = list(range(len(labels)))

    fig, ax_probability = plt.subplots(figsize=(7.1, 3.55))
    width = 0.38
    ax_probability.bar(
        [position - width / 2 for position in positions],
        fragility,
        width,
        color="#2d6f9f",
        label="Peak conditional failure probability",
    )
    ax_probability.set_ylabel("Conditional failure probability")
    ax_probability.set_ylim(0.0, 1.08)
    ax_probability.set_xticks(positions, labels)
    ax_probability.set_xlabel("Domain-qualified branch ID")
    ax_probability.grid(axis="y", alpha=0.2)

    ax_eens = ax_probability.twinx()
    ax_eens.bar(
        [position + width / 2 for position in positions],
        fmea,
        width,
        color="#c05a3d",
        label="N-1 FMEA EENS contribution",
    )
    ax_eens.set_ylabel("EENS contribution (MWh/yr)")
    handles_a, labels_a = ax_probability.get_legend_handles_labels()
    handles_b, labels_b = ax_eens.get_legend_handles_labels()
    ax_probability.legend(
        handles_a + handles_b,
        labels_a + labels_b,
        ncol=2,
        frameon=False,
        fontsize=8,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.17),
    )
    finish_figure(fig, figures / "review_case_weak_link_evidence.pdf")


def plot_held_out(results: Path, figures: Path) -> None:
    rows = read_csv(results / "held_out_risk.csv")
    feasible = [
        row
        for row in rows
        if row["baseline_feasible"] == "1" and row["improved_feasible"] == "1"
    ]
    baseline = sorted(float(row["baseline_unserved_mwh"]) for row in feasible)
    intervention = sorted(float(row["improved_unserved_mwh"]) for row in feasible)
    probability = [(index + 0.5) / len(feasible) for index in range(len(feasible))]

    fig, ax = plt.subplots(figsize=(7.1, 3.55))
    ax.step(
        probability,
        baseline,
        where="mid",
        color="#59636e",
        linewidth=1.8,
        label="Baseline",
    )
    ax.step(
        probability,
        intervention,
        where="mid",
        color="#187b59",
        linewidth=1.8,
        label="DC grid-forming storage",
    )
    ax.axvline(0.95, color="#b33a3a", linestyle="--", linewidth=1.0, label="95% quantile")
    ax.set(
        xlabel="Empirical cumulative probability",
        ylabel="Event ENS (MWh)",
        xlim=(0, 1),
        ylim=(0, None),
    )
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False, fontsize=8)
    finish_figure(fig, figures / "review_case_held_out_risk.pdf")


def plot_risk_convergence(results: Path, figures: Path) -> None:
    rows = read_csv(results / "risk_convergence.csv")
    samples = [int(row["paired_feasible_samples"]) for row in rows]
    baseline_mean = [float(row["baseline_mean_unserved_mwh"]) for row in rows]
    improved_mean = [float(row["improved_mean_unserved_mwh"]) for row in rows]
    baseline_cvar = [float(row["baseline_cvar95_unserved_mwh"]) for row in rows]
    improved_cvar = [float(row["improved_cvar95_unserved_mwh"]) for row in rows]

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 3.25), sharex=True)
    for ax, baseline, intervention, title in (
        (axes[0], baseline_mean, improved_mean, "Mean ENS"),
        (axes[1], baseline_cvar, improved_cvar, "CVaR95 ENS"),
    ):
        ax.plot(samples, baseline, marker="o", color="#59636e", label="Baseline")
        ax.plot(samples, intervention, marker="s", color="#187b59", label="Intervention")
        ax.set_title(title, fontsize=9)
        ax.set_xlabel("Paired feasible samples")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("Event ENS (MWh)")
    axes[1].legend(frameon=False, fontsize=8)
    finish_figure(fig, figures / "review_case_risk_convergence.pdf")


def plot_literature_duration_capacity(results: Path, figures: Path) -> None:
    rows = read_csv(results / "literature_duration_capacity.csv")
    by_resource: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_resource.setdefault(row["resource"], []).append(row)

    styles = {
        "CHP microgrids": ("#2d6f9f", "o"),
        "PV-battery microgrids": ("#c05a3d", "s"),
    }
    fig, axes = plt.subplots(1, 2, figsize=(7.1, 3.25), sharex=True)
    for resource, resource_rows in by_resource.items():
        resource_rows.sort(key=lambda row: float(row["service_duration_hr"]))
        duration = [float(row["service_duration_hr"]) for row in resource_rows]
        capacity = [float(row["reported_additional_capacity_mw"]) for row in resource_rows]
        nameplate = [float(row["reported_nameplate_capacity_mw"]) for row in resource_rows]
        normalized = [100.0 * value / base for value, base in zip(capacity, nameplate)]
        color, marker = styles[resource]
        axes[0].plot(duration, capacity, marker=marker, color=color, label=resource)
        axes[1].plot(duration, normalized, marker=marker, color=color, label=resource)

    axes[0].set(ylabel="Additional available capacity (MW)", title="Reported values")
    axes[1].set(ylabel="Share of reported nameplate (%)", title="Review normalization")
    for ax in axes:
        ax.set(xlabel="Reserve-service duration (h)", xticks=[1, 5, 10, 15, 20, 24])
        ax.grid(alpha=0.25)
    axes[0].legend(frameon=False, fontsize=8)
    finish_figure(fig, figures / "literature_duration_capacity.pdf")


def plot_transition_verification_funnel(figures: Path, report: dict) -> None:
    transitions = report["dynamic_certificate"]["transitions"]
    blocked = report["cyber_blocked_certificate"]["transitions"]
    values = [
        len(transitions),
        sum(bool(row["executable"]) for row in transitions),
        sum(bool(row["proof_valid"]) for row in transitions),
        sum(bool(row["executable"]) for row in blocked),
    ]
    labels = ["MIP transitions", "Nominally executable", "DAE certified", "Cyber-outage executable"]
    colors = ["#59636e", "#2d6f9f", "#b33a3a", "#c05a3d"]

    fig, axes = plt.subplots(1, 2, figsize=(7.1, 3.25))
    positions = list(range(len(values)))
    axes[0].bar(positions, values, color=colors, width=0.68)
    axes[0].set_xticks(positions, labels, rotation=24, ha="right")
    axes[0].set(ylabel="Transition count", ylim=(0, max(values) + 0.6), title="Cross-layer verification funnel")
    axes[0].grid(axis="y", alpha=0.25)
    for position, value in zip(positions, values):
        axes[0].text(position, value + 0.08, str(value), ha="center", va="bottom", fontsize=9)

    transition_labels = [f"{row['from_step']} to {row['to_step']}" for row in transitions]
    event_counts = [int(row["event_count"]) for row in transitions]
    status_colors = ["#b33a3a" if row["label"] == "failed" else "#e9a23b" for row in transitions]
    axes[1].barh(transition_labels, event_counts, color=status_colors)
    axes[1].invert_yaxis()
    axes[1].set(xlabel="Mapped event count", title="DAE outcome by transition")
    axes[1].grid(axis="x", alpha=0.25)
    for index, row in enumerate(transitions):
        axes[1].text(event_counts[index] + 0.08, index, row["label"], va="center", fontsize=8)
    axes[1].set_xlim(0, max(event_counts) + 1.8)
    fig.subplots_adjust(wspace=0.46, bottom=0.28)
    finish_figure(fig, figures / "review_case_transition_verification.pdf")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path)
    parser.add_argument("figures", type=Path)
    args = parser.parse_args()
    args.figures.mkdir(parents=True, exist_ok=True)
    with (args.results / "review_case_results.json").open(encoding="utf-8") as stream:
        report = json.load(stream)
    plot_service(args.results, args.figures, report)
    plot_weak_links(args.results, args.figures)
    plot_held_out(args.results, args.figures)
    plot_risk_convergence(args.results, args.figures)
    plot_literature_duration_capacity(args.results, args.figures)
    plot_transition_verification_funnel(args.figures, report)


if __name__ == "__main__":
    main()
