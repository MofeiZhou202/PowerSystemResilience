#!/usr/bin/env python3
"""Blocked historical validation and ablation of the weather generator."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import platform
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import xarray as xr

import climate_data as cd
import gcm_screen
import weather_generator as wg
import weather_generator_knn as knn


DEFAULT_DEFINITION = cd.TOOL_DIR / "manifests/weather_generator_evaluation.json"


def validate_definition(
    definition: Dict[str, Any], study: Dict[str, Any], score_definition: Dict[str, Any]
) -> None:
    expected_period = {
        "start": study["periods"]["reference"]["start"],
        "end": study["periods"]["reference"]["end"],
    }
    if definition.get("period") != expected_period:
        raise cd.ContractError("Generator evaluation period does not match the study")
    cross_validation = definition["cross_validation"]
    years = int(expected_period["end"][:4]) - int(expected_period["start"][:4]) + 1
    if int(cross_validation["block_years"]) * int(cross_validation["folds"]) != years:
        raise cd.ContractError(
            "Blocked folds must partition the complete reference period"
        )
    if cross_validation.get("exclude_transitions_across_removed_blocks") is not True:
        raise cd.ContractError("Removed-block transitions must be excluded")
    variants = definition.get("variants", [])
    identifiers = [variant["id"] for variant in variants]
    if len(identifiers) != len(set(identifiers)) or "full" not in identifiers:
        raise cd.ContractError("Evaluation variants must be unique and include full")
    if int(definition["members_per_fold"]) < 2:
        raise cd.ContractError("At least two generated members per fold are required")
    if score_definition["selection_rule"].get("automatic_exclusion") is not False:
        raise cd.ContractError(
            "Evaluation score definition cannot automatically exclude GCMs"
        )
    freeze = definition.get("development_freeze")
    if freeze:
        evidence_path = cd.REPO_ROOT / freeze["evidence_report"]
        if not evidence_path.exists():
            raise cd.ContractError("Frozen-method development evidence is missing")
        if cd.sha256_file(evidence_path) != freeze["evidence_report_sha256"]:
            raise cd.ContractError(
                "Frozen-method development evidence checksum changed"
            )
        if freeze["selected_variant"] not in identifiers:
            raise cd.ContractError(
                "Frozen selected variant is absent from confirmation"
            )
    access = definition.get("gcm_access")
    if access:
        model_set = access.get("model_set")
        allowed = access.get("allowed_gcms")
        if model_set not in cd.MODEL_SET_CHOICES:
            raise cd.ContractError("Evaluation gcm_access has an invalid model set")
        if allowed != cd.models_for_set(study, model_set):
            raise cd.ContractError(
                "Evaluation allowed_gcms must exactly match its declared model set"
            )
    outer_freeze = definition.get("outer_evaluation_freeze")
    if outer_freeze:
        if outer_freeze.get("outer_gcms") != cd.models_for_set(study, "outer"):
            raise cd.ContractError("Outer freeze GCM list differs from the study")
        for phase in ("development", "confirmation"):
            path = cd.REPO_ROOT / outer_freeze[f"{phase}_report"]
            if not path.exists():
                raise cd.ContractError(f"Outer freeze {phase} evidence is missing")
            if cd.sha256_file(path) != outer_freeze[f"{phase}_report_sha256"]:
                raise cd.ContractError(
                    f"Outer freeze {phase} evidence checksum changed"
                )
        catalog_path = cd.REPO_ROOT / outer_freeze["catalog_audit"]
        if not catalog_path.exists():
            raise cd.ContractError("Outer GCM catalog audit is missing")
        if cd.sha256_file(catalog_path) != outer_freeze["catalog_audit_sha256"]:
            raise cd.ContractError("Outer GCM catalog audit checksum changed")
        catalog_audit = cd.load_json(catalog_path)
        if not catalog_audit.get("valid"):
            raise cd.ContractError("Outer GCM catalog audit did not pass")
        frozen_variants = outer_freeze.get("frozen_variants", [])
        if frozen_variants != identifiers:
            raise cd.ContractError(
                "Outer evaluation variants differ from the frozen comparison"
            )
    amendment = definition.get("protocol_amendment")
    if amendment:
        original_path = cd.REPO_ROOT / amendment["original_protocol"]
        if not original_path.exists():
            raise cd.ContractError("Original outer protocol is missing")
        if cd.sha256_file(original_path) != amendment["original_protocol_sha256"]:
            raise cd.ContractError("Original outer protocol checksum changed")
        if amendment.get("methods_hyperparameters_scores_seeds_and_aggregation_changed") is not False:
            raise cd.ContractError("Calendar amendment cannot change the methods")
        allowed = definition["gcm_access"]["allowed_gcms"]
        if amendment.get("excluded_model") in allowed:
            raise cd.ContractError("Excluded calendar-incompatible GCM remains allowed")
        if amendment.get("replacement_model") not in allowed:
            raise cd.ContractError("Calendar-compatible replacement GCM is not allowed")
        qualification_name = amendment.get("replacement_data_qualification")
        if qualification_name:
            qualification_path = cd.REPO_ROOT / qualification_name
            if not qualification_path.exists():
                raise cd.ContractError("Replacement GCM data qualification is missing")
            if (
                cd.sha256_file(qualification_path)
                != amendment["replacement_data_qualification_sha256"]
            ):
                raise cd.ContractError(
                    "Replacement GCM data qualification checksum changed"
                )
            qualification = cd.load_json(qualification_path)
            if qualification.get("model") != amendment.get("replacement_model"):
                raise cd.ContractError(
                    "Replacement GCM data qualification model differs from amendment"
                )
            if qualification.get("generator_scores_computed_before_qualification") != 0:
                raise cd.ContractError(
                    "Replacement GCM was scored before its data qualification"
                )
            if qualification.get("method_or_hyperparameter_change") is not False:
                raise cd.ContractError(
                    "Replacement data qualification cannot change the methods"
                )


def matrix_to_output(matrix: np.ndarray) -> Dict[str, np.ndarray]:
    return {
        "tasmax": matrix[:, 0],
        "tasmin": matrix[:, 0] - matrix[:, 1],
        "hurs": matrix[:, 2],
        "sfcWind": matrix[:, 3],
        "rsds": matrix[:, 4],
        "pr": matrix[:, 5],
    }


def sample_monthly_iid(
    fitted: wg.FittedGenerator, months: np.ndarray, seed: int
) -> Dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    transformed = np.empty((months.size, len(wg.VARIABLES)), dtype=float)
    for month in range(1, 13):
        selected = np.flatnonzero(months == month)
        count = int(fitted.marginal_counts[month - 1])
        for column in range(len(wg.VARIABLES)):
            indices = rng.integers(count, size=selected.size)
            transformed[selected, column] = fitted.sorted_marginals[
                month - 1, column, indices
            ]
    output = matrix_to_output(transformed)
    output["hurs"] = np.clip(output["hurs"], 0.0, 100.0)
    output["sfcWind"] = np.maximum(output["sfcWind"], 0.0)
    output["rsds"] = np.maximum(output["rsds"], 0.0)
    output["pr"] = np.maximum(output["pr"], 0.0)
    return output


def fit_variant(
    training: np.ndarray,
    training_months: np.ndarray,
    transition_valid: np.ndarray,
    baseline_definition: Dict[str, Any],
    variant: Dict[str, Any],
) -> Any:
    if variant.get("generator") == "seasonal_multivariate_knn_analog":
        return knn.fit_generator(training, training_months, transition_valid)
    definition = copy.deepcopy(baseline_definition)
    definition["weather_state_model"]["states"] = int(variant["state_count"])
    return wg.fit_generator(
        training,
        training_months,
        definition,
        diagonal_autoregression=bool(variant["diagonal_autoregression"]),
        transition_valid=transition_valid,
    )


def sample_variant(
    fitted: Any,
    months: np.ndarray,
    baseline_definition: Dict[str, Any],
    variant: Dict[str, Any],
    seed: int,
    sampling_cache: Dict[Any, Any],
) -> Dict[str, np.ndarray]:
    if variant.get("generator") == "seasonal_multivariate_knn_analog":
        analog_definition = copy.deepcopy(sampling_cache["knn_definition"])
        analog_definition["analog_model"]["neighbor_count"] = int(
            variant["neighbor_count"]
        )
        analog_definition["analog_model"]["rank_weight_exponent"] = float(
            variant["rank_weight_exponent"]
        )
        if "season_trees" not in sampling_cache:
            sampling_cache["season_trees"] = knn.build_season_trees(fitted)
        output, _ = knn.sample_generator(
            fitted,
            months,
            analog_definition,
            seed,
            sampling_cache["season_trees"],
        )
        return output
    if bool(variant["monthly_iid"]):
        return sample_monthly_iid(fitted, months, seed)
    output, _ = wg.sample_generator(
        fitted,
        months,
        baseline_definition,
        seed,
        block_length_override=int(variant["block_length_days"]),
        independent_residual_columns=bool(variant["independent_residual_columns"]),
        residual_candidate_cache=sampling_cache,
    )
    return output


def summarize_rows(
    rows: List[Dict[str, Any]], variants: List[Dict[str, Any]], categories: List[str]
) -> List[Dict[str, Any]]:
    fold_means: Dict[Tuple[str, int], Dict[str, float]] = {}
    for variant in variants:
        variant_id = variant["id"]
        fold_starts = sorted(
            {
                int(row["fold_start_year"])
                for row in rows
                if row["variant"] == variant_id
            }
        )
        for fold_start in fold_starts:
            selected = [
                row
                for row in rows
                if row["variant"] == variant_id
                and int(row["fold_start_year"]) == fold_start
            ]
            fold_means[(variant_id, fold_start)] = {
                category: float(np.mean([float(row[category]) for row in selected]))
                for category in categories
            }
    full_composite = {
        fold: values["composite"]
        for (variant, fold), values in fold_means.items()
        if variant == "full"
    }
    summaries = []
    for variant in variants:
        variant_id = variant["id"]
        folds = sorted(fold for name, fold in fold_means if name == variant_id)
        category_summary = {}
        for category in categories:
            values = np.asarray(
                [fold_means[(variant_id, fold)][category] for fold in folds]
            )
            category_summary[category] = {
                "mean": float(np.mean(values)),
                "standard_deviation": float(np.std(values, ddof=1)),
                "minimum": float(np.min(values)),
                "maximum": float(np.max(values)),
            }
        deltas = np.asarray(
            [
                fold_means[(variant_id, fold)]["composite"] - full_composite[fold]
                for fold in folds
            ]
        )
        summaries.append(
            {
                "variant": variant_id,
                "description": variant["description"],
                "folds": len(folds),
                "category_scores": category_summary,
                "composite_delta_from_full": {
                    "mean": float(np.mean(deltas)),
                    "minimum": float(np.min(deltas)),
                    "maximum": float(np.max(deltas)),
                },
            }
        )
    ordered = sorted(
        summaries,
        key=lambda item: (
            item["category_scores"]["composite"]["mean"],
            item["variant"],
        ),
    )
    for rank, item in enumerate(ordered, start=1):
        item["diagnostic_rank"] = rank
    return ordered


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--definition", type=Path, default=DEFAULT_DEFINITION)
    parser.add_argument(
        "--baseline-definition", type=Path, default=wg.LEGACY_DEFINITION
    )
    parser.add_argument(
        "--score-definition", type=Path, default=gcm_screen.DEFAULT_DEFINITION
    )
    parser.add_argument("--knn-definition", type=Path, default=knn.DEFAULT_DEFINITION)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--model-set", choices=cd.MODEL_SET_CHOICES, default="pilot"
    )
    parser.add_argument("--site", required=True)
    parser.add_argument("--members", type=int)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    definition = cd.load_json(args.definition)
    baseline_definition = cd.load_json(args.baseline_definition)
    score_definition = cd.load_json(args.score_definition)
    knn_definition = cd.load_json(args.knn_definition)
    validate_definition(definition, study, score_definition)
    wg.validate_definition(baseline_definition)
    knn.validate_definition(knn_definition)
    outer_freeze = definition.get("outer_evaluation_freeze")
    if outer_freeze:
        if (
            cd.sha256_file(args.baseline_definition)
            != outer_freeze["baseline_definition_sha256"]
        ):
            raise cd.ContractError("Frozen outer baseline definition checksum changed")
        if (
            cd.sha256_file(args.knn_definition)
            != outer_freeze["knn_definition_sha256"]
        ):
            raise cd.ContractError("Frozen outer KNN definition checksum changed")
    started = time.perf_counter()
    cd.require_model_in_set(study, args.model, args.model_set)
    access = definition.get("gcm_access")
    if access:
        if args.model_set != access["model_set"]:
            raise cd.ContractError(
                f"Evaluation requires --model-set {access['model_set']}"
            )
        if args.model not in access["allowed_gcms"]:
            raise cd.ContractError(f"Model {args.model} is outside the evaluation gate")
    freeze = definition.get("development_freeze")
    if freeze and args.model not in freeze["confirmation_gcms_not_used_for_selection"]:
        raise cd.ContractError(
            f"Model {args.model} is outside the frozen confirmation set"
        )
    selection_guard = definition.get("selection_guard", {})
    development_gcms = selection_guard.get("development_gcms")
    if development_gcms and args.model not in development_gcms:
        raise cd.ContractError(f"Model {args.model} is outside the development set")
    if args.site not in {site["id"] for site in cd.iter_sites(study)}:
        raise cd.ContractError(f"Unknown pilot site {args.site}")
    members = args.members or int(definition["members_per_fold"])
    if members < 2:
        raise cd.ContractError("Evaluation requires at least two members per fold")
    matrix, months, inputs, coordinates = wg.load_conditioning_data(
        study, args.model, "historical", args.site
    )
    start_year = int(definition["period"]["start"][:4])
    end_year = int(definition["period"]["end"][:4])
    years = np.repeat(np.arange(start_year, end_year + 1), 365)
    original_indices = np.arange(matrix.shape[0])
    block_years = int(definition["cross_validation"]["block_years"])
    variants = definition["variants"]
    category_names = [*score_definition["categories"], "composite"]
    rows: List[Dict[str, Any]] = []
    fit_diagnostics = []
    for fold_start in range(start_year, end_year + 1, block_years):
        held = (years >= fold_start) & (years < fold_start + block_years)
        keep = ~held
        training_indices = original_indices[keep]
        transition_valid = np.diff(training_indices) == 1
        held_matrix = matrix[held]
        held_months = months[held]
        held_years = years[held]
        held_output = matrix_to_output(held_matrix)
        for variant in variants:
            fitted = fit_variant(
                matrix[keep],
                months[keep],
                transition_valid,
                baseline_definition,
                variant,
            )
            fit_record = {
                "fold_start_year": fold_start,
                "variant": variant["id"],
                "generator": variant.get("generator", "var_block"),
                "training_days": int(np.count_nonzero(keep)),
                "valid_training_transitions": int(np.count_nonzero(transition_valid)),
            }
            if isinstance(fitted, knn.FittedAnalogGenerator):
                fit_record["transition_records"] = int(fitted.predecessors.shape[0])
            else:
                fit_record["fitted_spectral_radius"] = fitted.fitted_spectral_radius
                fit_record["effective_spectral_radius"] = (
                    fitted.effective_spectral_radius
                )
            fit_diagnostics.append(fit_record)
            sampling_cache: Dict[Any, Any] = {"knn_definition": knn_definition}
            for member in range(members):
                seed = wg.derive_member_seed(
                    int(definition["base_seed"]),
                    f"{definition['evaluation_id']}:{fold_start}",
                    args.model,
                    "historical",
                    args.site,
                    member,
                )
                generated = sample_variant(
                    fitted,
                    held_months,
                    baseline_definition,
                    variant,
                    seed,
                    sampling_cache,
                )
                scores = gcm_screen.score_pair(
                    held_output,
                    generated,
                    held_months,
                    held_years,
                    score_definition,
                )
                rows.append(
                    {
                        "model": args.model,
                        "site": args.site,
                        "variant": variant["id"],
                        "fold_start_year": fold_start,
                        "fold_end_year": fold_start + block_years - 1,
                        "member": member,
                        "seed": seed,
                        **scores,
                    }
                )
    summaries = summarize_rows(rows, variants, category_names)
    output_root = args.output_root or (
        cd.REPO_ROOT / study["data_root"] / "processed/weather_generator_evaluation"
    )
    output_directory = (
        output_root
        / f"evaluation={definition['evaluation_id']}"
        / f"model={args.model}"
        / f"site={args.site}"
    )
    score_path = output_directory / "member_scores.csv"
    report_path = output_directory / "evaluation_report.json"
    if not args.overwrite and (score_path.exists() or report_path.exists()):
        raise cd.ContractError("Refusing to overwrite generator-evaluation outputs")
    output_directory.mkdir(parents=True, exist_ok=True)
    with score_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "schema_version": "1.0",
        "operation": "blocked_weather_generator_ablation_evaluation",
        "evaluation_id": definition["evaluation_id"],
        "status": "held_out_gcm_distribution_evaluation_not_observational_validation",
        "model_set": args.model_set,
        "conditioning_state": {
            "model": args.model,
            "experiment": "historical",
            "site": args.site,
            "epoch": f"{start_year}-{end_year}",
        },
        "members_per_fold": members,
        "member_seed_pairing": "common_within_fold_and_member_across_variants",
        "definition": cd.path_text(args.definition),
        "definition_sha256": cd.sha256_file(args.definition),
        "baseline_definition_sha256": cd.sha256_file(args.baseline_definition),
        "score_definition_sha256": cd.sha256_file(args.score_definition),
        "knn_definition_sha256": cd.sha256_file(args.knn_definition),
        "input_records": inputs,
        "coordinates": coordinates,
        "member_score_table": cd.path_text(score_path),
        "member_score_table_sha256": cd.sha256_file(score_path),
        "fit_diagnostics": fit_diagnostics,
        "variant_summary": summaries,
        "automatic_method_selection_completed": False,
        "gcm_or_ssp_probabilities_assigned": False,
        "interpretation_limits": definition["interpretation_limits"],
        "software_versions": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "xarray": xr.__version__,
        },
        "execution_time_seconds": time.perf_counter() - started,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    cd.write_json(report_path, report, args.overwrite)
    console_summary = {
        "status": report["status"],
        "rows": len(rows),
        "execution_time_seconds": report["execution_time_seconds"],
        "report": cd.path_text(report_path),
    }
    if args.quiet:
        console_summary["ranking"] = [
            {
                "rank": item["diagnostic_rank"],
                "variant": item["variant"],
                "composite": item["category_scores"]["composite"]["mean"],
            }
            for item in summaries
        ]
    else:
        console_summary["variant_summary"] = summaries
    print(json.dumps(console_summary, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except cd.ContractError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2)
