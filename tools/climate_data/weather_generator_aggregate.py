#!/usr/bin/env python3
"""Aggregate weather-generator evaluation reports without discarding units."""

from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

import climate_data as cd


DEFAULT_EVALUATION_ID = "shaanxi_weather_generator_state_count_sensitivity_v1"
DEFAULT_MODELS = ("EC-Earth3", "MPI-ESM1-2-HR", "GFDL-ESM4")


def summarize_units(
    rows: List[Dict[str, Any]], reference_variant: str
) -> List[Dict[str, Any]]:
    metadata_fields = {"model", "site", "variant", "diagnostic_rank"}
    categories = [key for key in rows[0] if key not in metadata_fields]
    variants = sorted({str(row["variant"]) for row in rows})
    units = sorted({(str(row["model"]), str(row["site"])) for row in rows})
    lookup = {
        (str(row["model"]), str(row["site"]), str(row["variant"])): row for row in rows
    }
    if any((*unit, reference_variant) not in lookup for unit in units):
        raise cd.ContractError("Reference variant is missing from one or more units")
    summaries = []
    for variant in variants:
        selected = [lookup[(*unit, variant)] for unit in units]
        values = np.asarray([float(row["composite"]) for row in selected])
        deltas = np.asarray(
            [
                float(lookup[(*unit, variant)]["composite"])
                - float(lookup[(*unit, reference_variant)]["composite"])
                for unit in units
            ]
        )
        by_model = {}
        for model in sorted({unit[0] for unit in units}):
            model_values = np.asarray(
                [float(row["composite"]) for row in selected if row["model"] == model]
            )
            by_model[model] = {
                "sites": int(model_values.size),
                "mean": float(np.mean(model_values)),
                "minimum": float(np.min(model_values)),
                "maximum": float(np.max(model_values)),
            }
        category_scores = {}
        for category in categories:
            category_values = np.asarray([float(row[category]) for row in selected])
            category_scores[category] = {
                "mean": float(np.mean(category_values)),
                "standard_deviation_across_units": float(
                    np.std(category_values, ddof=1)
                ),
                "minimum": float(np.min(category_values)),
                "maximum": float(np.max(category_values)),
            }
        summaries.append(
            {
                "variant": variant,
                "units": len(selected),
                "unit_weighting": "equal_weight_per_gcm_site_unit",
                "composite": {
                    "mean": float(np.mean(values)),
                    "standard_deviation_across_units": float(np.std(values, ddof=1)),
                    "minimum": float(np.min(values)),
                    "maximum": float(np.max(values)),
                },
                "category_scores": category_scores,
                "delta_from_reference": {
                    "reference_variant": reference_variant,
                    "mean": float(np.mean(deltas)),
                    "minimum": float(np.min(deltas)),
                    "maximum": float(np.max(deltas)),
                },
                "unit_rank_one_count": int(
                    sum(int(row["diagnostic_rank"]) == 1 for row in selected)
                ),
                "by_model": by_model,
            }
        )
    ordered = sorted(
        summaries, key=lambda item: (item["composite"]["mean"], item["variant"])
    )
    for rank, item in enumerate(ordered, start=1):
        item["aggregate_diagnostic_rank"] = rank
    return ordered


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--evaluation-id", default=DEFAULT_EVALUATION_ID)
    parser.add_argument("--models", nargs="+", default=list(DEFAULT_MODELS))
    parser.add_argument(
        "--model-set", choices=cd.MODEL_SET_CHOICES, default="pilot"
    )
    parser.add_argument("--sites", nargs="+")
    parser.add_argument("--reference-variant", default="state_k1")
    parser.add_argument("--aggregate-id", default="method_development_gcms_four_sites")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    allowed_models = set(cd.models_for_set(study, args.model_set))
    unexpected_models = sorted(set(args.models) - allowed_models)
    if unexpected_models:
        raise cd.ContractError(
            f"Models are outside the {args.model_set} set: {unexpected_models}"
        )
    sites = args.sites or [site["id"] for site in cd.iter_sites(study)]
    root = cd.REPO_ROOT / study["data_root"] / "processed/weather_generator_evaluation"
    rows = []
    report_records = []
    definition_hash = None
    for model in args.models:
        for site in sites:
            report_path = (
                root
                / f"evaluation={args.evaluation_id}"
                / f"model={model}"
                / f"site={site}"
                / "evaluation_report.json"
            )
            if not report_path.exists():
                raise cd.ContractError(f"Missing evaluation report: {report_path}")
            report = cd.load_json(report_path)
            if report.get("evaluation_id") != args.evaluation_id:
                raise cd.ContractError(f"Evaluation ID mismatch: {report_path}")
            if report.get("model_set", "pilot") != args.model_set:
                raise cd.ContractError(f"Evaluation model-set mismatch: {report_path}")
            current_hash = report["definition_sha256"]
            if definition_hash is None:
                definition_hash = current_hash
            elif definition_hash != current_hash:
                raise cd.ContractError("Evaluation reports use different definitions")
            score_path = cd.REPO_ROOT / report["member_score_table"]
            if cd.sha256_file(score_path) != report["member_score_table_sha256"]:
                raise cd.ContractError(f"Member-score checksum mismatch: {score_path}")
            report_records.append(
                {
                    "model": model,
                    "site": site,
                    "path": cd.path_text(report_path),
                    "sha256": cd.sha256_file(report_path),
                    "member_score_table_sha256": report["member_score_table_sha256"],
                }
            )
            for summary in report["variant_summary"]:
                row = {
                    "model": model,
                    "site": site,
                    "variant": summary["variant"],
                    "diagnostic_rank": int(summary["diagnostic_rank"]),
                    "composite": float(summary["category_scores"]["composite"]["mean"]),
                }
                for category, values in summary["category_scores"].items():
                    row[category] = float(values["mean"])
                rows.append(row)
    summaries = summarize_units(rows, args.reference_variant)
    output_dir = args.output_dir or (
        root / f"evaluation={args.evaluation_id}" / f"aggregate={args.aggregate_id}"
    )
    unit_path = output_dir / "unit_scores.csv"
    report_path = output_dir / "aggregate_report.json"
    if not args.overwrite and (unit_path.exists() or report_path.exists()):
        raise cd.ContractError("Refusing to overwrite aggregate evaluation outputs")
    output_dir.mkdir(parents=True, exist_ok=True)
    with unit_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    if args.model_set == "outer":
        status = "frozen_method_gcm_outer_evaluation_not_observational_validation"
    elif "frozen_confirmation" in args.evaluation_id:
        status = "frozen_method_confirmation_not_observational_validation"
    else:
        status = "method_development_aggregation_not_final_performance_or_observational_validation"
    output = {
        "schema_version": "1.0",
        "operation": "aggregate_weather_generator_evaluation_units",
        "evaluation_id": args.evaluation_id,
        "status": status,
        "model_set": args.model_set,
        "models": args.models,
        "sites": sites,
        "unit_count": len(args.models) * len(sites),
        "unit_semantics": "GCM-site units are retained and are not asserted to be independent identically distributed observations.",
        "aggregation": "equal weight for each GCM-site unit after equal weighting of folds within each unit",
        "reference_variant": args.reference_variant,
        "evaluation_definition_sha256": definition_hash,
        "input_reports": report_records,
        "unit_score_table": cd.path_text(unit_path),
        "unit_score_table_sha256": cd.sha256_file(unit_path),
        "variant_summary": summaries,
        "automatic_method_selection_completed": False,
        "significance_claim": False,
        "gcm_or_ssp_probabilities_assigned": False,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    cd.write_json(report_path, output, args.overwrite)
    print(
        json.dumps(
            {
                "status": output["status"],
                "unit_count": output["unit_count"],
                "ranking": [
                    {
                        "rank": item["aggregate_diagnostic_rank"],
                        "variant": item["variant"],
                        "mean": item["composite"]["mean"],
                        "rank_one_units": item["unit_rank_one_count"],
                    }
                    for item in summaries
                ],
                "report": cd.path_text(report_path),
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
