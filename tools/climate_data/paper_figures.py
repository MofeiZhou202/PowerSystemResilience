#!/usr/bin/env python3
"""Create frozen paper tables and figures for the future-weather case study."""

from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import climate_data as cd
import future_weather_batch as future


DEFAULT_DEFINITION = future.DEFAULT_DEFINITION
VAR_METHOD = "seasonal_var_block_ecdf_v2"
KNN_METHOD = "seasonal_multivariate_knn_analog_k10_frozen_v1"
PDF_METADATA = {
    "Creator": "HySim-XJTU-HRPES paper_figures.py",
    "CreationDate": None,
    "ModDate": None,
}


def read_csv(path: Path) -> List[Dict[str, Any]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def paired_rows(
    rows: Iterable[Dict[str, Any]],
    key_fields: Sequence[str],
    value_fields: Sequence[str],
) -> List[Dict[str, Any]]:
    lookup = {}
    for row in rows:
        key = tuple(str(row[field]) for field in key_fields)
        method = str(row["method"])
        lookup[(key, method)] = row
    keys = sorted({key for key, _ in lookup})
    paired = []
    for key in keys:
        if (key, VAR_METHOD) not in lookup or (key, KNN_METHOD) not in lookup:
            raise cd.ContractError(f"Incomplete KNN/VAR pair: {key}")
        baseline = lookup[(key, VAR_METHOD)]
        candidate = lookup[(key, KNN_METHOD)]
        item = dict(zip(key_fields, key))
        for field in value_fields:
            var_value = float(baseline[field])
            knn_value = float(candidate[field])
            item[f"VAR_{field}"] = var_value
            item[f"KNN_{field}"] = knn_value
            item[f"delta_{field}"] = knn_value - var_value
        paired.append(item)
    return paired


def write_rows(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        raise cd.ContractError(f"Cannot write empty table: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def grouped_boxplot(
    axes: Sequence[Any],
    paired: List[Dict[str, Any]],
    metrics: Sequence[Tuple[str, str]],
    experiments: Sequence[str],
) -> None:
    colors = ["#277DA1", "#D1495B"]
    for axis, (field, label) in zip(axes, metrics):
        groups = [
            [
                float(row[f"delta_{field}"])
                for row in paired
                if row["experiment"] == experiment
            ]
            for experiment in experiments
        ]
        plot = axis.boxplot(groups, patch_artist=True, widths=0.58, showfliers=False)
        for patch, color in zip(plot["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.72)
        axis.axhline(0.0, color="#333333", linewidth=0.9)
        axis.set_xticks(range(1, len(experiments) + 1), experiments)
        axis.set_title(label, fontsize=9)
        axis.tick_params(labelsize=8)
        axis.grid(axis="y", color="#dddddd", linewidth=0.6)


def plot_weather_deltas(
    paired: List[Dict[str, Any]], output: Path, experiments: Sequence[str]
) -> None:
    metrics = [
        ("tasmax_q99_degC", "99th percentile Tmax (degC)"),
        ("pr_q99_mm_day", "99th percentile precipitation (mm/day)"),
        ("maximum_three_day_precipitation_mm", "Maximum 3-day precipitation (mm)"),
        ("hot_dry_days_per_year", "Hot-dry days per year"),
        (
            "maximum_three_day_mean_compound_stress",
            "Maximum 3-day compound stress",
        ),
        ("tasmax_mean_degC", "Mean Tmax (degC)"),
    ]
    figure, axes = plt.subplots(2, 3, figsize=(10.5, 6.1), constrained_layout=True)
    grouped_boxplot(axes.ravel(), paired, metrics, experiments)
    figure.suptitle("KNN K=10 minus VAR: paired future-weather statistics", fontsize=12)
    figure.savefig(output, dpi=300)
    figure.savefig(output.with_suffix(".pdf"), metadata=PDF_METADATA)
    plt.close(figure)


def plot_resilience_deltas(
    paired: List[Dict[str, Any]], output: Path, experiments: Sequence[str]
) -> None:
    metrics = [
        ("total_shed_mwh", "Unserved energy (MWh)"),
        ("resilience_index", "Resilience index"),
        ("peak_shed_mw", "Peak shed (MW)"),
        ("total_demand_mwh", "Event demand (MWh)"),
    ]
    figure, axes = plt.subplots(1, 4, figsize=(11.0, 3.2), constrained_layout=True)
    grouped_boxplot(axes.ravel(), paired, metrics, experiments)
    figure.suptitle("KNN K=10 minus VAR: fixed resilience-case outcomes", fontsize=12)
    figure.savefig(output, dpi=300)
    figure.savefig(output.with_suffix(".pdf"), metadata=PDF_METADATA)
    plt.close(figure)


def plot_representative_pair(
    results: List[Dict[str, Any]], paired: List[Dict[str, Any]], output: Path
) -> Dict[str, Any]:
    selected = max(paired, key=lambda row: abs(float(row["delta_total_shed_mwh"])))
    key_fields = ("model", "experiment", "site", "member")
    key = tuple(str(selected[field]) for field in key_fields)
    lookup = {
        (
            str(row["model"]),
            str(row["experiment"]),
            str(row["site"]),
            str(row["member"]),
            str(row["method"]),
        ): row
        for row in results
    }
    var = lookup[(*key, VAR_METHOD)]
    knn = lookup[(*key, KNN_METHOD)]
    figure, axes = plt.subplots(3, 1, figsize=(8.4, 6.8), sharex=True, constrained_layout=True)
    for row, label, color in (
        (var, "VAR", "#277DA1"),
        (knn, "KNN K=10", "#D1495B"),
    ):
        hours = [float(step["hour"]) for step in row["steps"]]
        axes[0].plot(hours, [float(step["load_multiplier"]) for step in row["steps"]], label=label, color=color)
        axes[1].plot(hours, [float(step["pv_multiplier"]) for step in row["steps"]], label=f"{label} PV", color=color, linestyle="-")
        axes[1].plot(hours, [float(step["wind_multiplier"]) for step in row["steps"]], label=f"{label} wind", color=color, linestyle="--")
        axes[2].plot(hours, [float(step["shed_mw"]) for step in row["steps"]], label=label, color=color)
    axes[0].set_ylabel("Load multiplier")
    axes[1].set_ylabel("Availability")
    axes[2].set_ylabel("Shed (MW)")
    axes[2].set_xlabel("Event hour")
    for axis in axes:
        axis.grid(color="#dddddd", linewidth=0.6)
        axis.legend(fontsize=8, ncol=2)
    figure.suptitle(
        f"Largest paired EENS difference: {selected['model']} {selected['experiment']} member {selected['member']}",
        fontsize=11,
    )
    figure.savefig(output, dpi=300)
    figure.savefig(output.with_suffix(".pdf"), metadata=PDF_METADATA)
    plt.close(figure)
    return selected


def summarize_deltas(
    rows: List[Dict[str, Any]], value_fields: Sequence[str]
) -> Dict[str, Any]:
    output = {}
    for field in value_fields:
        values = np.asarray([float(row[f"delta_{field}"]) for row in rows])
        output[field] = {
            "pairs": int(values.size),
            "mean_KNN_minus_VAR": float(np.mean(values)),
            "median_KNN_minus_VAR": float(np.median(values)),
            "minimum": float(np.min(values)),
            "maximum": float(np.max(values)),
            "negative_pair_count": int(np.count_nonzero(values < 0.0)),
            "positive_pair_count": int(np.count_nonzero(values > 0.0)),
        }
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--definition", type=Path, default=DEFAULT_DEFINITION)
    parser.add_argument("--batch-report", type=Path)
    parser.add_argument("--mapping-provenance", type=Path)
    parser.add_argument("--case-bundle", type=Path)
    parser.add_argument("--weather-summary", type=Path)
    parser.add_argument("--resilience-results", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    definition = cd.load_json(args.definition)
    future.validate_definition(definition, study)
    study_dir = cd.REPO_ROOT / study["data_root"] / "processed/future_weather_study"
    batch_path = args.batch_report or (
        cd.REPO_ROOT / study["data_root"] / "provenance/future_weather_batch.json"
    )
    mapping_path = (
        args.mapping_provenance or study_dir / "weather_to_grid_provenance.json"
    )
    bundle_path = args.case_bundle or study_dir / "resilience_case_bundle.json"
    weather_path = args.weather_summary or study_dir / "future_weather_member_summary.csv"
    resilience_path = args.resilience_results or study_dir / "resilience_results.json"
    output_dir = args.output_dir or study_dir / "paper"
    outputs = [
        output_dir / "weather_generator_future_deltas.png",
        output_dir / "weather_generator_future_deltas.pdf",
        output_dir / "resilience_case_deltas.png",
        output_dir / "resilience_case_deltas.pdf",
        output_dir / "representative_resilience_event.png",
        output_dir / "representative_resilience_event.pdf",
        output_dir / "weather_paired_deltas.csv",
        output_dir / "resilience_paired_deltas.csv",
        output_dir / "paper_results_summary.json",
    ]
    if not args.overwrite and any(path.exists() for path in outputs):
        raise cd.ContractError("Refusing to overwrite paper outputs")
    generation = definition["weather_generation"]
    definition_sha256 = cd.sha256_file(args.definition)
    batch = cd.load_json(batch_path)
    expected_tasks = int(generation["expected_state_method_tasks"])
    expected_members = int(generation["expected_generated_members"])
    if (
        batch.get("status") != "complete"
        or batch.get("executed") is not True
        or int(batch.get("task_count", -1)) != expected_tasks
        or int(batch.get("tasks_completed", -1)) != expected_tasks
        or int(batch.get("member_count", -1)) != expected_members
        or batch.get("failures") != []
    ):
        raise cd.ContractError("Future-weather batch is incomplete or has failures")
    if batch.get("definition_sha256") != definition_sha256:
        raise cd.ContractError("Future-weather batch definition checksum changed")
    mapping = cd.load_json(mapping_path)
    expected_cases = (
        len(cd.models_for_set(study, generation["model_set"]))
        * len(generation["experiments"])
        * len(generation["methods"])
        * int(generation["members_per_state"])
    )
    if (
        mapping.get("definition_sha256") != definition_sha256
        or mapping.get("batch_report_sha256") != cd.sha256_file(batch_path)
        or int(mapping.get("weather_member_count", -1)) != expected_members
        or int(mapping.get("resilience_case_count", -1)) != expected_cases
        or mapping.get("case_bundle_sha256") != cd.sha256_file(bundle_path)
        or mapping.get("weather_summary_sha256") != cd.sha256_file(weather_path)
    ):
        raise cd.ContractError("Weather-to-grid provenance chain is inconsistent")
    weather_rows = read_csv(weather_path)
    weather_fields = [
        "tasmax_mean_degC",
        "tasmax_q99_degC",
        "pr_q99_mm_day",
        "maximum_three_day_precipitation_mm",
        "hot_dry_days_per_year",
        "maximum_three_day_mean_compound_stress",
    ]
    weather_paired = paired_rows(
        weather_rows,
        ("model", "experiment", "site", "member"),
        weather_fields,
    )
    resilience_payload = cd.load_json(resilience_path)
    if (
        resilience_payload.get("failed_case_count") != 0
        or int(resilience_payload.get("case_count", -1)) != expected_cases
        or len(resilience_payload.get("results", [])) != expected_cases
        or resilience_payload.get("algorithm_tuned_from_weather_results") is not False
    ):
        raise cd.ContractError("Resilience results are incomplete, failed, or tuned")
    resilience_rows = resilience_payload["results"]
    resilience_fields = [
        "total_shed_mwh",
        "weighted_unserved_mwh",
        "resilience_index",
        "peak_shed_mw",
        "total_demand_mwh",
    ]
    resilience_paired = paired_rows(
        resilience_rows,
        ("model", "experiment", "site", "member"),
        resilience_fields,
    )
    expected_weather_pairs = int(
        generation["expected_generated_members"]
    ) // 2
    expected_resilience_pairs = int(resilience_payload["case_count"]) // 2
    if len(weather_paired) != expected_weather_pairs:
        raise cd.ContractError("Future-weather paired table is incomplete")
    if len(resilience_paired) != expected_resilience_pairs:
        raise cd.ContractError("Resilience paired table is incomplete")
    output_dir.mkdir(parents=True, exist_ok=True)
    write_rows(outputs[6], weather_paired)
    write_rows(outputs[7], resilience_paired)
    experiments = generation["experiments"]
    plot_weather_deltas(weather_paired, outputs[0], experiments)
    plot_resilience_deltas(resilience_paired, outputs[2], experiments)
    representative = plot_representative_pair(
        resilience_rows, resilience_paired, outputs[4]
    )
    summary = {
        "schema_version": "1.0",
        "operation": "future_weather_generator_paper_results",
        "definition_sha256": definition_sha256,
        "upstream_artifacts": [
            {
                "role": "future_weather_batch_report",
                "path": cd.path_text(batch_path),
                "sha256": cd.sha256_file(batch_path),
            },
            {
                "role": "weather_to_grid_provenance",
                "path": cd.path_text(mapping_path),
                "sha256": cd.sha256_file(mapping_path),
            },
            {
                "role": "resilience_case_bundle",
                "path": cd.path_text(bundle_path),
                "sha256": cd.sha256_file(bundle_path),
            },
            {
                "role": "future_weather_member_summary",
                "path": cd.path_text(weather_path),
                "sha256": cd.sha256_file(weather_path),
            },
            {
                "role": "fixed_resilience_results",
                "path": cd.path_text(resilience_path),
                "sha256": cd.sha256_file(resilience_path),
            },
        ],
        "implementation_artifacts": [
            {
                "path": "tools/climate_data/future_weather_batch.py",
                "sha256": cd.sha256_file(
                    cd.REPO_ROOT / "tools/climate_data/future_weather_batch.py"
                ),
            },
            {
                "path": "tools/climate_data/weather_to_grid.py",
                "sha256": cd.sha256_file(
                    cd.REPO_ROOT / "tools/climate_data/weather_to_grid.py"
                ),
            },
            {
                "path": "tools/weather_resilience_case_study.cpp",
                "sha256": cd.sha256_file(
                    cd.REPO_ROOT / "tools/weather_resilience_case_study.cpp"
                ),
            },
            {
                "path": "tools/climate_data/paper_figures.py",
                "sha256": cd.sha256_file(
                    cd.REPO_ROOT / "tools/climate_data/paper_figures.py"
                ),
            },
        ],
        "weather_summary_sha256": cd.sha256_file(weather_path),
        "resilience_results_sha256": cd.sha256_file(resilience_path),
        "weather_pairs": len(weather_paired),
        "resilience_pairs": len(resilience_paired),
        "weather_delta_summary": summarize_deltas(weather_paired, weather_fields),
        "resilience_delta_summary": summarize_deltas(
            resilience_paired, resilience_fields
        ),
        "representative_pair": representative,
        "figures": [
            {"path": cd.path_text(path), "sha256": cd.sha256_file(path)}
            for path in outputs[:6]
        ],
        "paired_tables": [
            {"path": cd.path_text(path), "sha256": cd.sha256_file(path)}
            for path in outputs[6:8]
        ],
        "significance_claim": False,
        "automatic_method_selection": False,
        "generator_probability_weights": None,
        "GCM_probability_weights": None,
        "SSP_probability_weights": None,
        "interpretation": definition["reporting"]["interpretation"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    cd.write_json(outputs[8], summary, args.overwrite)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except cd.ContractError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2)
