#!/usr/bin/env python3
"""Evaluate how generated paths preserve source-GCM climate change signals."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import climate_data as cd
import future_weather_batch as future
import weather_generator as wg
import weather_to_grid as mapping


DEFAULT_ANALYSIS = cd.TOOL_DIR / "manifests/climate_signal_fidelity.json"
PDF_METADATA = {
    "Creator": "HySim-XJTU-HRPES climate_signal_fidelity.py",
    "CreationDate": None,
    "ModDate": None,
}
METHOD_LABELS = {
    "seasonal_var_block_ecdf_v2": "VAR",
    "seasonal_multivariate_knn_analog_k10_frozen_v1": "KNN K=10",
}
METRIC_LABELS = {
    "tasmax_mean_degC": "Mean Tmax (degC)",
    "tasmax_q99_degC": "99th-percentile Tmax (degC)",
    "pr_mean_mm_day": "Mean precipitation (mm/day)",
    "pr_q99_mm_day": "99th-percentile precipitation (mm/day)",
    "mean_annual_rx3day_mm": "Mean annual RX3day (mm)",
    "tasmax_lag1_correlation": "Tmax lag-1 correlation",
    "pr_lag1_correlation": "Precipitation lag-1 correlation",
    "tasmax_hurs_correlation": "Tmax-humidity correlation",
    "sfcWind_rsds_correlation": "Wind-radiation correlation",
    "hot_days_per_year": "Hot days per year",
    "hot_dry_days_per_year": "Hot-dry days per year",
    "hot_dry_low_wind_days_per_year": "Hot-dry-low-wind days per year",
    "mean_annual_max_hot_spell_days": "Mean annual maximum hot spell (days)",
    "mean_annual_max_dry_spell_days": "Mean annual maximum dry spell (days)",
    "maximum_three_day_mean_compound_stress": "Maximum 3-day compound stress",
    "wet_day_fraction": "Wet-day fraction",
}


def validate_analysis(
    analysis: Dict[str, Any], definition: Dict[str, Any], study: Dict[str, Any]
) -> None:
    future.validate_definition(definition, study)
    if analysis.get("future_experiments") != definition["weather_generation"][
        "experiments"
    ]:
        raise cd.ContractError("Climate-signal experiments differ from future study")
    if analysis.get("metrics") != list(METRIC_LABELS):
        raise cd.ContractError("Climate-signal metric contract changed")
    reporting = analysis["reporting"]
    if reporting.get("automatic_generator_selection") is not False:
        raise cd.ContractError("Climate-signal analysis cannot select a generator")
    if reporting.get("member_level_significance_claim") is not False:
        raise cd.ContractError("Member-level significance claims are not permitted")
    for key in (
        "GCM_probability_weights",
        "SSP_probability_weights",
        "generator_probability_weights",
    ):
        if reporting.get(key) is not None:
            raise cd.ContractError(f"Climate-signal analysis cannot assign {key}")
    for path_key, hash_key in (
        ("batch_report", "batch_report_sha256"),
        ("outer_result_snapshot", "outer_result_snapshot_sha256"),
        ("unit_score_table", "unit_score_table_sha256"),
    ):
        path = cd.REPO_ROOT / analysis[path_key]
        if not path.is_file() or cd.sha256_file(path) != analysis[hash_key]:
            raise cd.ContractError(f"Frozen climate-signal input changed: {path}")


def output_arrays(matrix: np.ndarray) -> Dict[str, np.ndarray]:
    tasmax = np.asarray(matrix[:, 0], dtype=float)
    dtr = np.maximum(np.asarray(matrix[:, 1], dtype=float), 0.0)
    return {
        "tasmax": tasmax,
        "tasmin": tasmax - dtr,
        "hurs": np.clip(np.asarray(matrix[:, 2], dtype=float), 0.0, 100.0),
        "sfcWind": np.maximum(np.asarray(matrix[:, 3], dtype=float), 0.0),
        "rsds": np.maximum(np.asarray(matrix[:, 4], dtype=float), 0.0),
        "pr": np.maximum(np.asarray(matrix[:, 5], dtype=float), 0.0),
    }


def correlation(left: np.ndarray, right: np.ndarray) -> float:
    if left.size != right.size or left.size < 2:
        raise cd.ContractError("Correlation arrays are incomplete")
    if float(np.std(left)) == 0.0 or float(np.std(right)) == 0.0:
        return 0.0
    return float(np.corrcoef(left, right)[0, 1])


def maximum_run(mask: np.ndarray) -> int:
    best = 0
    current = 0
    for value in mask:
        current = current + 1 if bool(value) else 0
        best = max(best, current)
    return best


def annual_slices(length: int) -> Iterable[slice]:
    if length % 365 != 0:
        raise cd.ContractError("Climate-signal inputs must use complete 365-day years")
    for start in range(0, length, 365):
        yield slice(start, start + 365)


def metric_values(
    arrays: Dict[str, np.ndarray],
    hot_threshold: float,
    low_wind_threshold: float,
    dry_threshold: float,
    definition: Dict[str, Any],
) -> Dict[str, float]:
    lengths = {np.asarray(values).size for values in arrays.values()}
    if len(lengths) != 1:
        raise cd.ContractError("Climate-signal variables are misaligned")
    length = lengths.pop()
    years = length // 365
    hot = arrays["tasmax"] > hot_threshold
    dry = arrays["pr"] < dry_threshold
    low_wind = arrays["sfcWind"] < low_wind_threshold
    annual_rx3 = []
    annual_hot_spell = []
    annual_dry_spell = []
    for selected in annual_slices(length):
        precipitation = arrays["pr"][selected]
        annual_rx3.append(
            float(np.max(np.convolve(precipitation, np.ones(3), mode="valid")))
        )
        annual_hot_spell.append(maximum_run(hot[selected]))
        annual_dry_spell.append(maximum_run(dry[selected]))
    stress = mapping.compound_stress(
        arrays["tasmax"],
        arrays["hurs"],
        arrays["sfcWind"],
        arrays["pr"],
        definition,
    )
    return {
        "tasmax_mean_degC": float(np.mean(arrays["tasmax"])),
        "tasmax_q99_degC": float(np.quantile(arrays["tasmax"], 0.99)),
        "pr_mean_mm_day": float(np.mean(arrays["pr"])),
        "pr_q99_mm_day": float(np.quantile(arrays["pr"], 0.99)),
        "mean_annual_rx3day_mm": float(np.mean(annual_rx3)),
        "tasmax_lag1_correlation": correlation(
            arrays["tasmax"][:-1], arrays["tasmax"][1:]
        ),
        "pr_lag1_correlation": correlation(arrays["pr"][:-1], arrays["pr"][1:]),
        "tasmax_hurs_correlation": correlation(arrays["tasmax"], arrays["hurs"]),
        "sfcWind_rsds_correlation": correlation(
            arrays["sfcWind"], arrays["rsds"]
        ),
        "hot_days_per_year": float(np.count_nonzero(hot) / years),
        "hot_dry_days_per_year": float(np.count_nonzero(hot & dry) / years),
        "hot_dry_low_wind_days_per_year": float(
            np.count_nonzero(hot & dry & low_wind) / years
        ),
        "mean_annual_max_hot_spell_days": float(np.mean(annual_hot_spell)),
        "mean_annual_max_dry_spell_days": float(np.mean(annual_dry_spell)),
        "maximum_three_day_mean_compound_stress": float(
            np.max(np.convolve(stress, np.ones(3) / 3.0, mode="valid"))
        ),
        "wet_day_fraction": float(np.mean(arrays["pr"] >= dry_threshold)),
    }


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    if not rows:
        raise cd.ContractError(f"Cannot write empty table: {path}")
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def aggregate_units(
    rows: List[Dict[str, Any]], metrics: List[str]
) -> List[Dict[str, Any]]:
    groups: Dict[Tuple[str, ...], List[Dict[str, Any]]] = defaultdict(list)
    key_fields = ("model", "experiment", "site", "method")
    for row in rows:
        groups[tuple(str(row[field]) for field in key_fields)].append(row)
    output = []
    for key in sorted(groups):
        members = groups[key]
        item: Dict[str, Any] = dict(zip(key_fields, key))
        item["member_count"] = len(members)
        for metric in metrics:
            generated = np.asarray(
                [float(row[f"generated_{metric}"]) for row in members]
            )
            historical = float(members[0][f"historical_{metric}"])
            source_future = float(members[0][f"source_future_{metric}"])
            item[f"historical_{metric}"] = historical
            item[f"source_future_{metric}"] = source_future
            item[f"source_signal_{metric}"] = source_future - historical
            item[f"generated_mean_{metric}"] = float(np.mean(generated))
            item[f"generated_std_{metric}"] = float(np.std(generated, ddof=1))
            item[f"generated_signal_{metric}"] = float(np.mean(generated) - historical)
            item[f"signal_error_{metric}"] = float(np.mean(generated) - source_future)
        output.append(item)
    return output


def method_summary(
    unit_rows: List[Dict[str, Any]], metrics: List[str]
) -> Dict[str, Any]:
    output = {}
    for method in METHOD_LABELS:
        selected = [row for row in unit_rows if row["method"] == method]
        metric_summary = {}
        for metric in metrics:
            errors = np.asarray(
                [float(row[f"signal_error_{metric}"]) for row in selected]
            )
            metric_summary[metric] = {
                "units": int(errors.size),
                "mean_bias": float(np.mean(errors)),
                "mean_absolute_error": float(np.mean(np.abs(errors))),
                "median_absolute_error": float(np.median(np.abs(errors))),
                "maximum_absolute_error": float(np.max(np.abs(errors))),
                "negative_unit_count": int(np.count_nonzero(errors < 0.0)),
                "positive_unit_count": int(np.count_nonzero(errors > 0.0)),
            }
        output[method] = {
            "label": METHOD_LABELS[method],
            "method_units": len(selected),
            "metrics": metric_summary,
        }
    return output


def save_figure(figure: Any, path: Path) -> None:
    figure.savefig(path, dpi=300)
    figure.savefig(path.with_suffix(".pdf"), metadata=PDF_METADATA)
    plt.close(figure)


def plot_signal_scatter(unit_rows: List[Dict[str, Any]], output: Path) -> None:
    metrics = (
        "tasmax_mean_degC",
        "tasmax_q99_degC",
        "mean_annual_rx3day_mm",
        "hot_days_per_year",
        "hot_dry_days_per_year",
        "mean_annual_max_hot_spell_days",
    )
    colors = {
        "seasonal_var_block_ecdf_v2": "#277DA1",
        "seasonal_multivariate_knn_analog_k10_frozen_v1": "#D1495B",
    }
    figure, axes = plt.subplots(2, 3, figsize=(10.5, 6.6), constrained_layout=True)
    for axis, metric in zip(axes.ravel(), metrics):
        all_values = []
        for method in METHOD_LABELS:
            selected = [row for row in unit_rows if row["method"] == method]
            source = np.asarray(
                [float(row[f"source_signal_{metric}"]) for row in selected]
            )
            generated = np.asarray(
                [float(row[f"generated_signal_{metric}"]) for row in selected]
            )
            all_values.extend(source.tolist())
            all_values.extend(generated.tolist())
            axis.scatter(
                source,
                generated,
                s=18,
                alpha=0.72,
                color=colors[method],
                label=METHOD_LABELS[method],
            )
        lower = min(all_values)
        upper = max(all_values)
        padding = max((upper - lower) * 0.06, 1e-6)
        axis.plot(
            [lower - padding, upper + padding],
            [lower - padding, upper + padding],
            color="#333333",
            linewidth=0.9,
            linestyle="--",
        )
        axis.set_xlim(lower - padding, upper + padding)
        axis.set_ylim(lower - padding, upper + padding)
        axis.set_title(METRIC_LABELS[metric], fontsize=9)
        axis.set_xlabel("Source GCM change", fontsize=8)
        axis.set_ylabel("Generated change", fontsize=8)
        axis.tick_params(labelsize=7)
        axis.grid(color="#dddddd", linewidth=0.6)
    axes[0, 0].legend(fontsize=8)
    figure.suptitle(
        "Preservation of 2031-2060 minus 1985-2014 climate signals", fontsize=12
    )
    save_figure(figure, output)


def plot_signal_errors(unit_rows: List[Dict[str, Any]], output: Path) -> None:
    metrics = (
        "tasmax_q99_degC",
        "pr_q99_mm_day",
        "mean_annual_rx3day_mm",
        "tasmax_lag1_correlation",
        "pr_lag1_correlation",
        "hot_dry_days_per_year",
        "hot_dry_low_wind_days_per_year",
        "maximum_three_day_mean_compound_stress",
    )
    colors = ("#277DA1", "#D1495B")
    figure, axes = plt.subplots(2, 4, figsize=(11.0, 6.0), constrained_layout=True)
    for axis, metric in zip(axes.ravel(), metrics):
        groups = [
            [
                float(row[f"signal_error_{metric}"])
                for row in unit_rows
                if row["method"] == method
            ]
            for method in METHOD_LABELS
        ]
        plot = axis.boxplot(groups, patch_artist=True, widths=0.58, showfliers=False)
        for patch, color in zip(plot["boxes"], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.72)
        axis.axhline(0.0, color="#333333", linewidth=0.9)
        axis.set_xticks((1, 2), ("VAR", "KNN"))
        axis.set_title(METRIC_LABELS[metric], fontsize=8)
        axis.tick_params(labelsize=7)
        axis.grid(axis="y", color="#dddddd", linewidth=0.6)
    figure.suptitle(
        "Generated minus source-GCM change-signal error across 48 climate states",
        fontsize=12,
    )
    save_figure(figure, output)


def plot_outer_evaluation(unit_score_path: Path, output: Path) -> None:
    rows = read_csv(unit_score_path)
    models = ("CanESM5", "CNRM-CM6-1", "IPSL-CM6A-LR", "MIROC6", "NorESM2-MM", "FGOALS-g3")
    sites = ("yulin", "xian", "hanzhong", "ankang")
    lookup = {
        (row["model"], row["site"], row["variant"]): row for row in rows
    }
    heatmap = np.asarray(
        [
            [
                float(lookup[(model, site, "knn_k10")]["composite"])
                - float(lookup[(model, site, "full")]["composite"])
                for site in sites
            ]
            for model in models
        ]
    )
    categories = (
        "annual_extremes_and_compound_events",
        "daily_persistence",
        "marginal_distribution",
        "multivariate_dependence",
        "seasonal_cycle",
    )
    category_labels = ("Extremes", "Persistence", "Marginals", "Dependence", "Seasonality")
    category_delta = np.asarray(
        [
            np.mean(
                [
                    float(row[category])
                    for row in rows
                    if row["variant"] == "knn_k10"
                ]
            )
            - np.mean(
                [float(row[category]) for row in rows if row["variant"] == "full"]
            )
            for category in categories
        ]
    )
    figure, axes = plt.subplots(
        1, 2, figsize=(9.8, 4.4), gridspec_kw={"width_ratios": (1.45, 1.0)}, constrained_layout=True
    )
    bound = float(np.max(np.abs(heatmap)))
    image = axes[0].imshow(heatmap, cmap="RdBu_r", vmin=-bound, vmax=bound, aspect="auto")
    axes[0].set_xticks(range(len(sites)), ("Yulin", "Xi'an", "Hanzhong", "Ankang"))
    axes[0].set_yticks(range(len(models)), models)
    axes[0].set_title("Composite loss: KNN minus VAR", fontsize=10)
    axes[0].tick_params(labelsize=8)
    for row_index in range(len(models)):
        for column_index in range(len(sites)):
            axes[0].text(
                column_index,
                row_index,
                f"{heatmap[row_index, column_index]:+.4f}",
                ha="center",
                va="center",
                fontsize=7,
            )
    figure.colorbar(image, ax=axes[0], fraction=0.046, pad=0.04)
    bar_colors = ["#277DA1" if value < 0.0 else "#D1495B" for value in category_delta]
    axes[1].barh(range(len(categories)), category_delta, color=bar_colors, alpha=0.82)
    axes[1].axvline(0.0, color="#333333", linewidth=0.9)
    axes[1].set_yticks(range(len(categories)), category_labels)
    axes[1].invert_yaxis()
    axes[1].set_xlabel("Mean category loss difference", fontsize=8)
    axes[1].set_title("Category decomposition", fontsize=10)
    axes[1].tick_params(labelsize=8)
    axes[1].grid(axis="x", color="#dddddd", linewidth=0.6)
    figure.suptitle("GCM-level outer evaluation", fontsize=12)
    save_figure(figure, output)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--definition", type=Path, default=future.DEFAULT_DEFINITION)
    parser.add_argument("--analysis", type=Path, default=DEFAULT_ANALYSIS)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--progress", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    definition = cd.load_json(args.definition)
    analysis = cd.load_json(args.analysis)
    validate_analysis(analysis, definition, study)
    output_dir = args.output_dir or (
        cd.REPO_ROOT
        / study["data_root"]
        / "processed/future_weather_study/climate_signal_fidelity"
    )
    outputs = {
        "member_table": output_dir / "member_signal_fidelity.csv",
        "unit_table": output_dir / "unit_signal_fidelity.csv",
        "source_inventory": output_dir / "source_input_inventory.json",
        "scatter_png": output_dir / "climate_signal_scatter.png",
        "scatter_pdf": output_dir / "climate_signal_scatter.pdf",
        "error_png": output_dir / "climate_signal_error.png",
        "error_pdf": output_dir / "climate_signal_error.pdf",
        "outer_png": output_dir / "outer_evaluation_detail.png",
        "outer_pdf": output_dir / "outer_evaluation_detail.pdf",
        "report": output_dir / "climate_signal_fidelity_report.json",
    }
    if not args.overwrite and any(path.exists() for path in outputs.values()):
        raise cd.ContractError("Refusing to overwrite climate-signal outputs")
    output_dir.mkdir(parents=True, exist_ok=True)
    batch_path = cd.REPO_ROOT / analysis["batch_report"]
    batch = cd.load_json(batch_path)
    if batch.get("status") != "complete" or batch.get("failures") != []:
        raise cd.ContractError("Future-weather batch is incomplete")
    metrics = list(analysis["metrics"])
    dry_threshold = float(analysis["thresholds"]["dry_day_mm"])
    source_cache: Dict[Tuple[str, str, str], Dict[str, np.ndarray]] = {}
    metric_cache: Dict[Tuple[str, str, str], Dict[str, float]] = {}
    threshold_cache: Dict[Tuple[str, str], Tuple[float, float]] = {}
    source_inventory = []

    def source_arrays(model: str, experiment: str, site: str) -> Dict[str, np.ndarray]:
        key = (model, experiment, site)
        if key not in source_cache:
            matrix, _, inputs, coordinates = wg.load_conditioning_data(
                study, model, experiment, site
            )
            source_cache[key] = output_arrays(matrix)
            source_inventory.append(
                {
                    "model": model,
                    "experiment": experiment,
                    "site": site,
                    "coordinates": coordinates,
                    "inputs": inputs,
                }
            )
        return source_cache[key]

    def thresholds(model: str, site: str) -> Tuple[float, float]:
        key = (model, site)
        if key not in threshold_cache:
            historical = source_arrays(model, analysis["reference_experiment"], site)
            threshold_cache[key] = (
                float(np.quantile(historical["tasmax"], 0.90)),
                float(np.quantile(historical["sfcWind"], 0.10)),
            )
        return threshold_cache[key]

    def source_metrics(model: str, experiment: str, site: str) -> Dict[str, float]:
        key = (model, experiment, site)
        if key not in metric_cache:
            hot_threshold, low_wind_threshold = thresholds(model, site)
            metric_cache[key] = metric_values(
                source_arrays(model, experiment, site),
                hot_threshold,
                low_wind_threshold,
                dry_threshold,
                definition,
            )
        return metric_cache[key]

    member_rows = []
    records = [record for record in batch["records"] if record["valid"]]
    for record_index, record in enumerate(records, start=1):
        model = record["model"]
        experiment = record["experiment"]
        site = record["site"]
        method = record["method"]
        historical = source_metrics(model, analysis["reference_experiment"], site)
        source_future = source_metrics(model, experiment, site)
        hot_threshold, low_wind_threshold = thresholds(model, site)
        state_directory = cd.REPO_ROOT / record["state_directory"]
        for member in range(int(record["outputs"]["member_count"])):
            member_path = state_directory / f"member={member:04d}.nc"
            arrays, _ = mapping.read_member(member_path)
            generated = metric_values(
                arrays,
                hot_threshold,
                low_wind_threshold,
                dry_threshold,
                definition,
            )
            row: Dict[str, Any] = {
                "model": model,
                "experiment": experiment,
                "site": site,
                "method": method,
                "member": member,
                "source_weather": cd.path_text(member_path),
                "source_weather_sha256": cd.sha256_file(member_path),
                "historical_hot_threshold_degC": hot_threshold,
                "historical_low_wind_threshold_m_s": low_wind_threshold,
            }
            for metric in metrics:
                row[f"historical_{metric}"] = historical[metric]
                row[f"source_future_{metric}"] = source_future[metric]
                row[f"source_signal_{metric}"] = (
                    source_future[metric] - historical[metric]
                )
                row[f"generated_{metric}"] = generated[metric]
                row[f"generated_signal_{metric}"] = (
                    generated[metric] - historical[metric]
                )
                row[f"signal_error_{metric}"] = (
                    generated[metric] - source_future[metric]
                )
            member_rows.append(row)
        if args.progress:
            print(
                f"{record_index}/{len(records)} {method} {model} {experiment} {site}",
                flush=True,
            )
    expected_members = int(analysis["expected_generated_members"])
    if len(member_rows) != expected_members:
        raise cd.ContractError(
            f"Expected {expected_members} member rows, found {len(member_rows)}"
        )
    unit_rows = aggregate_units(member_rows, metrics)
    if len(unit_rows) != int(analysis["expected_method_units"]):
        raise cd.ContractError("Climate-signal method-unit inventory is incomplete")
    write_csv(outputs["member_table"], member_rows)
    write_csv(outputs["unit_table"], unit_rows)
    source_inventory.sort(key=lambda row: (row["model"], row["experiment"], row["site"]))
    cd.write_json(
        outputs["source_inventory"],
        {
            "schema_version": "1.0",
            "operation": "climate_signal_source_input_inventory",
            "state_count": len(source_inventory),
            "states": source_inventory,
        },
        args.overwrite,
    )
    plot_signal_scatter(unit_rows, outputs["scatter_png"])
    plot_signal_errors(unit_rows, outputs["error_png"])
    plot_outer_evaluation(cd.REPO_ROOT / analysis["unit_score_table"], outputs["outer_png"])
    figure_records = [
        {"path": cd.path_text(path), "sha256": cd.sha256_file(path)}
        for key, path in outputs.items()
        if key.endswith("_png") or key.endswith("_pdf")
    ]
    report = {
        "schema_version": "1.0",
        "operation": "future_weather_climate_signal_fidelity",
        "analysis_id": analysis["analysis_id"],
        "analysis_definition": cd.path_text(args.analysis),
        "analysis_definition_sha256": cd.sha256_file(args.analysis),
        "future_study_definition_sha256": cd.sha256_file(args.definition),
        "batch_report_sha256": cd.sha256_file(batch_path),
        "source_inventory": cd.path_text(outputs["source_inventory"]),
        "source_inventory_sha256": cd.sha256_file(outputs["source_inventory"]),
        "member_table": cd.path_text(outputs["member_table"]),
        "member_table_sha256": cd.sha256_file(outputs["member_table"]),
        "unit_table": cd.path_text(outputs["unit_table"]),
        "unit_table_sha256": cd.sha256_file(outputs["unit_table"]),
        "generated_member_count": len(member_rows),
        "method_unit_count": len(unit_rows),
        "climate_state_count": len(unit_rows) // len(METHOD_LABELS),
        "metric_count": len(metrics),
        "metric_labels": METRIC_LABELS,
        "method_summary": method_summary(unit_rows, metrics),
        "figures": figure_records,
        "automatic_generator_selection": False,
        "member_level_significance_claim": False,
        "interpretation": analysis["reporting"]["interpretation"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    cd.write_json(outputs["report"], report, args.overwrite)
    print(
        json.dumps(
            {
                "generated_member_count": report["generated_member_count"],
                "method_unit_count": report["method_unit_count"],
                "metric_count": report["metric_count"],
                "report": cd.path_text(outputs["report"]),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except cd.ContractError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2)
