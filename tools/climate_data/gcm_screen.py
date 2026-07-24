#!/usr/bin/env python3
"""Historical NEX-GDDP-CMIP6 screening against an ERA5-Land reference."""

from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import xarray as xr

import climate_data as cd


DEFAULT_DEFINITION = cd.TOOL_DIR / "manifests" / "gcm_skill_screening.json"


def validate_definition(definition: Dict[str, Any], study: Dict[str, Any]) -> None:
    expected_period = {
        "start": study["periods"]["reference"]["start"],
        "end": study["periods"]["reference"]["end"],
    }
    if definition.get("period") != expected_period:
        raise cd.ContractError("GCM screen period does not match the study reference")
    weights = [float(spec["weight"]) for spec in definition["categories"].values()]
    if any(weight < 0 for weight in weights) or not np.isclose(sum(weights), 1.0):
        raise cd.ContractError(
            "GCM screen category weights must be nonnegative and sum to one"
        )
    quantiles = [float(value) for value in definition["quantiles"]]
    if quantiles != sorted(set(quantiles)) or not all(
        0 < value < 1 for value in quantiles
    ):
        raise cd.ContractError(
            "GCM screen quantiles must be unique, ordered, and inside (0,1)"
        )
    if definition["selection_rule"].get("automatic_exclusion") is not False:
        raise cd.ContractError(
            "Reanalysis-only screening cannot automatically exclude GCMs"
        )


def average_ranks(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    sorted_values = values[order]
    ranks = np.empty(values.size, dtype=float)
    start = 0
    while start < values.size:
        end = start + 1
        while end < values.size and sorted_values[end] == sorted_values[start]:
            end += 1
        ranks[order[start:end]] = (start + end - 1) / 2.0
        start = end
    return ranks


def spearman_matrix(matrix: np.ndarray) -> np.ndarray:
    ranked = np.column_stack(
        [average_ranks(matrix[:, column]) for column in range(matrix.shape[1])]
    )
    return np.corrcoef(ranked, rowvar=False)


def lag_one_correlation(values: np.ndarray, years: np.ndarray) -> float:
    adjacent = years[1:] == years[:-1]
    left = values[:-1][adjacent]
    right = values[1:][adjacent]
    if left.size < 2 or np.std(left) == 0 or np.std(right) == 0:
        return 0.0
    return float(np.corrcoef(left, right)[0, 1])


def monthly_thresholds(
    values: np.ndarray, months: np.ndarray, quantile: float
) -> np.ndarray:
    return np.asarray(
        [np.quantile(values[months == month], quantile) for month in range(1, 13)]
    )


def annual_features(
    arrays: Dict[str, np.ndarray],
    months: np.ndarray,
    years: np.ndarray,
    reference: Dict[str, np.ndarray],
) -> Dict[str, np.ndarray]:
    hot_threshold = monthly_thresholds(reference["tasmax"], months, 0.9)
    warm_threshold = monthly_thresholds(reference["tasmin"], months, 0.9)
    wind_threshold = monthly_thresholds(reference["sfcWind"], months, 0.1)
    hot = arrays["tasmax"] > hot_threshold[months - 1]
    warm = arrays["tasmin"] > warm_threshold[months - 1]
    dry = arrays["pr"] < 1.0
    low_wind = arrays["sfcWind"] < wind_threshold[months - 1]
    features = {
        name: []
        for name in (
            "txx",
            "tnx",
            "rx1day",
            "cdd",
            "hot_days",
            "warm_night_days",
            "hot_dry_days",
            "hot_dry_low_wind_days",
        )
    }
    for year in sorted(set(int(value) for value in years)):
        selected = years == year
        features["txx"].append(float(np.max(arrays["tasmax"][selected])))
        features["tnx"].append(float(np.max(arrays["tasmin"][selected])))
        features["rx1day"].append(float(np.max(arrays["pr"][selected])))
        features["cdd"].append(float(cd.maximum_run_length(dry[selected])))
        features["hot_days"].append(float(np.count_nonzero(hot[selected])))
        features["warm_night_days"].append(float(np.count_nonzero(warm[selected])))
        features["hot_dry_days"].append(
            float(np.count_nonzero(hot[selected] & dry[selected]))
        )
        features["hot_dry_low_wind_days"].append(
            float(np.count_nonzero(hot[selected] & dry[selected] & low_wind[selected]))
        )
    return {name: np.asarray(values, dtype=float) for name, values in features.items()}


def score_pair(
    reference: Dict[str, np.ndarray],
    candidate: Dict[str, np.ndarray],
    months: np.ndarray,
    years: np.ndarray,
    definition: Dict[str, Any],
) -> Dict[str, float]:
    variables = definition["variables"]
    quantiles = definition["quantiles"]
    scales = {
        variable: max(
            float(
                np.quantile(reference[variable], 0.95)
                - np.quantile(reference[variable], 0.05)
            ),
            float(definition["normalization_floors"][variable]),
        )
        for variable in variables
    }
    marginal = np.mean(
        [
            np.mean(
                np.abs(
                    np.quantile(candidate[variable], quantiles)
                    - np.quantile(reference[variable], quantiles)
                )
            )
            / scales[variable]
            for variable in variables
        ]
    )
    seasonal = np.mean(
        [
            np.sqrt(
                np.mean(
                    [
                        (
                            np.mean(candidate[variable][months == month])
                            - np.mean(reference[variable][months == month])
                        )
                        ** 2
                        for month in range(1, 13)
                    ]
                )
            )
            / scales[variable]
            for variable in variables
        ]
    )
    reference_matrix = np.column_stack([reference[name] for name in variables])
    candidate_matrix = np.column_stack([candidate[name] for name in variables])
    difference = np.abs(
        spearman_matrix(candidate_matrix) - spearman_matrix(reference_matrix)
    )
    dependence = float(np.mean(difference[np.triu_indices(len(variables), 1)]))
    persistence = float(
        np.mean(
            [
                abs(
                    lag_one_correlation(candidate[variable], years)
                    - lag_one_correlation(reference[variable], years)
                )
                for variable in variables
            ]
        )
    )
    reference_features = annual_features(reference, months, years, reference)
    candidate_features = annual_features(candidate, months, years, reference)
    feature_errors = []
    for name, reference_values in reference_features.items():
        scale = max(
            float(
                np.quantile(reference_values, 0.95)
                - np.quantile(reference_values, 0.05)
            ),
            1.0,
        )
        feature_errors.append(
            abs(float(np.mean(candidate_features[name]) - np.mean(reference_values)))
            / scale
        )
    extremes = float(np.mean(feature_errors))
    categories = {
        "marginal_distribution": float(marginal),
        "seasonal_cycle": float(seasonal),
        "multivariate_dependence": dependence,
        "daily_persistence": persistence,
        "annual_extremes_and_compound_events": extremes,
    }
    categories["composite"] = float(
        sum(
            categories[name] * float(spec["weight"])
            for name, spec in definition["categories"].items()
        )
    )
    return categories


def load_era5(
    study: Dict[str, Any], site: str
) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray]:
    root = cd.REPO_ROOT / study["data_root"]
    path = (
        root
        / "processed/daily/era5_land"
        / f"site={site}"
        / "reference_period=1985-2014"
        / f"daily_era5_land_{site}_1985_2014.nc"
    )
    with xr.open_dataset(path) as dataset:
        dates = np.asarray(dataset.time.values)
        date_text = np.asarray([str(value)[:10] for value in dates])
        months = np.asarray([int(value[5:7]) for value in date_text])
        years = np.asarray([int(value[:4]) for value in date_text])
        keep = np.asarray([value[5:] != "02-29" for value in date_text])
        arrays = {
            variable: np.asarray(dataset[variable].values, dtype=float)[keep]
            for variable in ("tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr")
        }
    return arrays, months[keep], years[keep]


def load_nex(
    study: Dict[str, Any], model: str, site: str
) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray, list]:
    root = cd.REPO_ROOT / study["data_root"]
    arrays = {
        name: [] for name in ("tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr")
    }
    months = []
    years = []
    inputs = []
    for year in range(1985, 2015):
        path = (
            root
            / "processed/daily/nex_gddp_cmip6"
            / f"model={model}"
            / "experiment=historical"
            / f"site={site}"
            / f"year={year}"
            / f"daily_nex_gddp_cmip6_{model}_historical_{site}_{year}_v2.0.nc"
        )
        with xr.open_dataset(path) as dataset:
            date_text = [str(value)[:10] for value in np.asarray(dataset.time.values)]
            keep = np.asarray([value[5:] != "02-29" for value in date_text])
            months.extend(
                int(value[5:7]) for value, selected in zip(date_text, keep) if selected
            )
            years.extend(
                int(value[:4]) for value, selected in zip(date_text, keep) if selected
            )
            for variable in arrays:
                arrays[variable].append(
                    np.asarray(dataset[variable].values, dtype=float)[keep]
                )
        inputs.append({"path": cd.path_text(path), "sha256": cd.sha256_file(path)})
    return (
        {name: np.concatenate(parts) for name, parts in arrays.items()},
        np.asarray(months, dtype=int),
        np.asarray(years, dtype=int),
        inputs,
    )


def rank_models(scores: Dict[str, float]) -> Dict[str, int]:
    ordered = sorted(scores, key=lambda model: (scores[model], model))
    return {model: rank + 1 for rank, model in enumerate(ordered)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--definition", type=Path, default=DEFAULT_DEFINITION)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    definition = cd.load_json(args.definition)
    validate_definition(definition, study)
    models = study["climate_states"]["pilot_gcms"]
    sites = [site["id"] for site in cd.iter_sites(study)]
    references = {site: load_era5(study, site) for site in sites}
    candidates = {}
    input_records = {}
    site_rows = []
    for model in models:
        for site in sites:
            arrays, months, years, inputs = load_nex(study, model, site)
            reference, reference_months, reference_years = references[site]
            if not np.array_equal(months, reference_months) or not np.array_equal(
                years, reference_years
            ):
                raise cd.ContractError(
                    f"Aligned historical calendars differ for {model} {site}"
                )
            candidates[(model, site)] = arrays
            input_records[f"{model}:{site}"] = inputs
            scores = score_pair(reference, arrays, months, years, definition)
            site_rows.append({"model": model, "site": site, **scores})
    model_scores = {}
    category_names = [*definition["categories"], "composite"]
    for model in models:
        rows = [row for row in site_rows if row["model"] == model]
        model_scores[model] = {
            category: float(np.mean([row[category] for row in rows]))
            for category in category_names
        }
    full_ranks = rank_models(
        {model: model_scores[model]["composite"] for model in models}
    )
    block_ranks = {model: [] for model in models}
    for block_start in range(1985, 2015, 5):
        fold_scores = {}
        for model in models:
            values = []
            for site in sites:
                reference, months, years = references[site]
                keep = (years < block_start) | (years >= block_start + 5)
                values.append(
                    score_pair(
                        {name: array[keep] for name, array in reference.items()},
                        {
                            name: array[keep]
                            for name, array in candidates[(model, site)].items()
                        },
                        months[keep],
                        years[keep],
                        definition,
                    )["composite"]
                )
            fold_scores[model] = float(np.mean(values))
        ranks = rank_models(fold_scores)
        for model in models:
            block_ranks[model].append(ranks[model])
    summaries = []
    for model in models:
        summaries.append(
            {
                "model": model,
                **model_scores[model],
                "reference_screening_rank": full_ranks[model],
                "leave_five_year_block_out_rank_min": min(block_ranks[model]),
                "leave_five_year_block_out_rank_median": float(
                    np.median(block_ranks[model])
                ),
                "leave_five_year_block_out_rank_max": max(block_ranks[model]),
                "provisional_method_development_shortlist": full_ranks[model] <= 3,
                "continues_to_independent_observation_gate": True,
            }
        )
    summaries.sort(key=lambda row: row["reference_screening_rank"])
    output_dir = (
        args.output_dir or cd.REPO_ROOT / study["data_root"] / "processed/gcm_screening"
    )
    csv_path = output_dir / "nex_gddp_cmip6_historical_reference_scores.csv"
    report_path = (
        cd.REPO_ROOT
        / study["data_root"]
        / "provenance/gcm_screening/nex_gddp_cmip6_historical_reference_screen.json"
    )
    if not args.overwrite and (csv_path.exists() or report_path.exists()):
        raise cd.ContractError("Refusing to overwrite existing GCM screening outputs")
    output_dir.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(site_rows[0]))
        writer.writeheader()
        writer.writerows(site_rows)
    report = {
        "schema_version": "1.0",
        "screening_id": definition["screening_id"],
        "status": "reanalysis_reference_screening_not_observational_validation",
        "definition": definition,
        "site_score_table": cd.path_text(csv_path),
        "site_score_table_sha256": cd.sha256_file(csv_path),
        "model_summary": summaries,
        "input_records": input_records,
        "independent_observation_validation_completed": False,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    cd.write_json(report_path, report, args.overwrite)
    print(
        json.dumps(
            {
                "status": report["status"],
                "model_summary": summaries,
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
