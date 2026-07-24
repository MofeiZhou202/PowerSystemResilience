#!/usr/bin/env python3
"""Season-conditioned multivariate KNN analog weather generator."""

from __future__ import annotations

import argparse
import io
import json
import platform
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import scipy
import xarray as xr
from scipy.spatial import cKDTree

import climate_data as cd
import weather_generator as wg


DEFAULT_DEFINITION = cd.TOOL_DIR / "manifests/weather_generator_knn_analog.json"


@dataclass
class FittedAnalogGenerator:
    sorted_marginals: np.ndarray
    marginal_counts: np.ndarray
    predecessors: np.ndarray
    successors: np.ndarray
    successor_months: np.ndarray
    transition_source_indices: np.ndarray


def validate_definition(definition: Dict[str, Any]) -> None:
    if definition.get("model_variables") != list(wg.VARIABLES):
        raise cd.ContractError("KNN analog variables do not match the code contract")
    if definition["conditioning"].get("pool_gcms_or_ssps") is not False:
        raise cd.ContractError("KNN analog must not pool GCM or SSP states")
    analog = definition["analog_model"]
    if int(analog["neighbor_count"]) < 1:
        raise cd.ContractError("KNN neighbor count must be positive")
    if float(analog["rank_weight_exponent"]) <= 0.0:
        raise cd.ContractError("KNN rank-weight exponent must be positive")
    if (
        analog.get("successor_sampling")
        != "sample_complete_multivariate_successor_vector"
    ):
        raise cd.ContractError("KNN analog must sample complete successor vectors")


def fit_generator(
    values: np.ndarray,
    months: np.ndarray,
    transition_valid: Optional[np.ndarray] = None,
) -> FittedAnalogGenerator:
    gaussian, marginals, counts = wg.fit_monthly_marginals(values, months)
    if transition_valid is None:
        transition_valid = np.ones(values.shape[0] - 1, dtype=bool)
    if transition_valid.shape != (values.shape[0] - 1,):
        raise cd.ContractError("KNN transition mask has the wrong shape")
    if not np.any(transition_valid):
        raise cd.ContractError("KNN transition library is empty")
    return FittedAnalogGenerator(
        sorted_marginals=marginals,
        marginal_counts=counts,
        predecessors=gaussian[:-1][transition_valid],
        successors=gaussian[1:][transition_valid],
        successor_months=months[1:][transition_valid].copy(),
        transition_source_indices=np.flatnonzero(transition_valid) + 1,
    )


def build_season_trees(
    fitted: FittedAnalogGenerator,
) -> Dict[int, Tuple[np.ndarray, cKDTree]]:
    successor_seasons = np.asarray(
        [wg.season(int(month)) for month in fitted.successor_months]
    )
    trees = {}
    for season in range(4):
        positions = np.flatnonzero(successor_seasons == season)
        if positions.size == 0:
            raise cd.ContractError(f"KNN transition library lacks season {season}")
        trees[season] = (positions, cKDTree(fitted.predecessors[positions]))
    return trees


def rank_probabilities(count: int, exponent: float) -> np.ndarray:
    weights = np.arange(1, count + 1, dtype=float) ** (-exponent)
    return weights / np.sum(weights)


def transformed_to_output(transformed: np.ndarray) -> Dict[str, np.ndarray]:
    tasmax = transformed[:, 0]
    dtr = np.maximum(transformed[:, 1], 0.0)
    return {
        "tasmax": tasmax,
        "tasmin": tasmax - dtr,
        "hurs": np.clip(transformed[:, 2], 0.0, 100.0),
        "sfcWind": np.maximum(transformed[:, 3], 0.0),
        "rsds": np.maximum(transformed[:, 4], 0.0),
        "pr": np.maximum(transformed[:, 5], 0.0),
    }


def sample_generator(
    fitted: FittedAnalogGenerator,
    months: np.ndarray,
    definition: Dict[str, Any],
    seed: int,
    season_trees: Optional[Dict[int, Tuple[np.ndarray, cKDTree]]] = None,
) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
    analog = definition["analog_model"]
    neighbor_count = int(analog["neighbor_count"])
    exponent = float(analog["rank_weight_exponent"])
    rng = np.random.default_rng(seed)
    trees = season_trees or build_season_trees(fitted)
    gaussian = np.empty((months.size, len(wg.VARIABLES)), dtype=float)
    selected_transitions = np.empty(months.size, dtype=int)
    initial_candidates = np.flatnonzero(fitted.successor_months == months[0])
    if initial_candidates.size == 0:
        raise cd.ContractError("KNN initialization month has no historical vectors")
    initial = int(initial_candidates[int(rng.integers(initial_candidates.size))])
    gaussian[0] = fitted.successors[initial]
    selected_transitions[0] = initial
    for index in range(1, months.size):
        positions, tree = trees[wg.season(int(months[index]))]
        k = min(neighbor_count, positions.size)
        distances, local = tree.query(gaussian[index - 1], k=k, workers=1)
        distances = np.atleast_1d(distances)
        local = np.atleast_1d(local)
        global_positions = positions[local]
        order = np.lexsort((global_positions, distances))
        neighbors = global_positions[order]
        probabilities = rank_probabilities(k, exponent)
        selected = int(neighbors[int(rng.choice(k, p=probabilities))])
        gaussian[index] = fitted.successors[selected]
        selected_transitions[index] = selected
    transformed = wg.inverse_monthly_marginals(
        gaussian, months, fitted.sorted_marginals, fitted.marginal_counts
    )
    return transformed_to_output(transformed), selected_transitions


def save_fit(path: Path, fitted: FittedAnalogGenerator) -> None:
    arrays = {
        "marginal_counts": fitted.marginal_counts,
        "predecessors": fitted.predecessors,
        "sorted_marginals": fitted.sorted_marginals,
        "successor_months": fitted.successor_months,
        "successors": fitted.successors,
        "transition_source_indices": fitted.transition_source_indices,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(arrays):
            buffer = io.BytesIO()
            np.lib.format.write_array(buffer, arrays[name], allow_pickle=False)
            entry = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            entry.create_system = 3
            entry.external_attr = 0o600 << 16
            archive.writestr(entry, buffer.getvalue())


def load_fit(path: Path) -> FittedAnalogGenerator:
    with np.load(path, allow_pickle=False) as arrays:
        return FittedAnalogGenerator(
            sorted_marginals=arrays["sorted_marginals"],
            marginal_counts=arrays["marginal_counts"],
            predecessors=arrays["predecessors"],
            successors=arrays["successors"],
            successor_months=arrays["successor_months"],
            transition_source_indices=arrays["transition_source_indices"],
        )


def write_member(
    path: Path,
    values: Dict[str, np.ndarray],
    selected: np.ndarray,
    dates: list,
    attributes: Dict[str, Any],
) -> None:
    dataset = xr.Dataset(
        data_vars={
            **{
                name: (
                    "time",
                    values[name].astype("float32"),
                    {"units": wg.UNITS[name]},
                )
                for name in wg.OUTPUT_VARIABLES
            },
            "analog_transition_index": (
                "time",
                selected.astype("int32"),
                {
                    "long_name": "zero-based selected fitted transition record",
                    "units": "1",
                },
            ),
        },
        coords={"time": dates},
        attrs=attributes,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    encoding = {
        name: {"zlib": True, "complevel": 4}
        for name in (*wg.OUTPUT_VARIABLES, "analog_transition_index")
    }
    encoding["time"] = {"calendar": "365_day"}
    dataset.to_netcdf(path, encoding=encoding)
    dataset.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--definition", type=Path, default=DEFAULT_DEFINITION)
    parser.add_argument("--model", required=True)
    parser.add_argument(
        "--model-set", choices=cd.MODEL_SET_CHOICES, default="pilot"
    )
    parser.add_argument(
        "--experiment", choices=("historical", "ssp245", "ssp585"), required=True
    )
    parser.add_argument("--site", required=True)
    parser.add_argument("--members", type=int, default=1)
    parser.add_argument("--base-seed", type=int, default=20260724)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--reuse-fit", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    definition = cd.load_json(args.definition)
    validate_definition(definition)
    cd.require_model_in_set(study, args.model, args.model_set)
    if args.site not in {site["id"] for site in cd.iter_sites(study)}:
        raise cd.ContractError(f"Unknown pilot site {args.site}")
    if args.members < 1:
        raise cd.ContractError("At least one KNN member is required")
    start_year, end_year = wg.period_years(study, args.experiment)
    method = definition["method_id"]
    root = args.output_root or (
        cd.REPO_ROOT / study["data_root"] / "processed/weather_generator"
    )
    state_directory = (
        root
        / f"method={method}"
        / f"model={args.model}"
        / f"experiment={args.experiment}"
        / f"site={args.site}"
        / f"epoch={start_year}-{end_year}"
    )
    parameter_path = state_directory / "parameters.npz"
    fit_report_path = state_directory / "fit_provenance.json"
    member_paths = [
        state_directory / f"member={member:04d}.nc" for member in range(args.members)
    ]
    member_reports = [path.with_suffix(".provenance.json") for path in member_paths]
    definition_hash = cd.sha256_file(args.definition)
    conditioning_state = {
        "model": args.model,
        "experiment": args.experiment,
        "site": args.site,
        "epoch": f"{start_year}-{end_year}",
    }
    if args.reuse_fit:
        if not parameter_path.exists() or not fit_report_path.exists():
            raise cd.ContractError("--reuse-fit requires KNN parameters and provenance")
        wg.ensure_absent([*member_paths, *member_reports], args.overwrite)
        fit_report = cd.load_json(fit_report_path)
        wg.verify_reusable_fit(
            fit_report, conditioning_state, definition_hash, parameter_path
        )
        fitted = load_fit(parameter_path)
        inputs = fit_report["inputs"]
        coordinates = fit_report["coordinates"]
        parameter_hash = fit_report["parameter_archive_sha256"]
    else:
        wg.ensure_absent(
            [parameter_path, fit_report_path, *member_paths, *member_reports],
            args.overwrite,
        )
        matrix, months, inputs, coordinates = wg.load_conditioning_data(
            study, args.model, args.experiment, args.site
        )
        fitted = fit_generator(matrix, months)
        save_fit(parameter_path, fitted)
        parameter_hash = cd.sha256_file(parameter_path)
        fit_report = {
            "schema_version": "1.0",
            "operation": "fit_seasonal_multivariate_knn_analog",
            "method_id": method,
            "method_role": definition["method_role"],
            "model_set": args.model_set,
            "conditioning_state": conditioning_state,
            "probability_semantics": study["probability_semantics"],
            "definition": cd.path_text(args.definition),
            "definition_sha256": definition_hash,
            "inputs": inputs,
            "coordinates": coordinates,
            "fitting_days": int(matrix.shape[0]),
            "transition_records": int(fitted.predecessors.shape[0]),
            "parameter_archive": cd.path_text(parameter_path),
            "parameter_archive_sha256": parameter_hash,
            "software_versions": {
                "python": platform.python_version(),
                "numpy": np.__version__,
                "scipy": scipy.__version__,
                "xarray": xr.__version__,
            },
            "created_utc": datetime.now(timezone.utc).isoformat(),
        }
        cd.write_json(fit_report_path, fit_report, args.overwrite)
    fit_report_hash = cd.sha256_file(fit_report_path)
    dates, target_months = wg.noleap_dates(start_year, end_year)
    trees = build_season_trees(fitted)
    summaries = []
    for member, (member_path, report_path) in enumerate(
        zip(member_paths, member_reports)
    ):
        seed = wg.derive_member_seed(
            args.base_seed, method, args.model, args.experiment, args.site, member
        )
        values, selected = sample_generator(
            fitted, target_months, definition, seed, trees
        )
        diagnostics = wg.output_diagnostics(values)
        selected_months = fitted.successor_months[selected]
        season_match = bool(
            np.all(
                [
                    wg.season(int(source)) == wg.season(int(target))
                    for source, target in zip(selected_months[1:], target_months[1:])
                ]
            )
        )
        if not season_match:
            raise cd.ContractError("Generated KNN transition crossed season condition")
        write_member(
            member_path,
            values,
            selected,
            dates,
            {
                "study_id": study["study_id"],
                "method_id": method,
                "method_role": definition["method_role"],
                "model_set": args.model_set,
                "model": args.model,
                "experiment": args.experiment,
                "site": args.site,
                "epoch": f"{start_year}-{end_year}",
                "calendar": "365_day",
                "member": member,
                "member_seed": str(seed),
                "parameter_archive_sha256": parameter_hash,
                **coordinates,
            },
        )
        report = {
            "schema_version": "1.0",
            "operation": "sample_seasonal_multivariate_knn_analog",
            "conditioning_state": conditioning_state,
            "model_set": args.model_set,
            "member": member,
            "base_seed": args.base_seed,
            "member_seed": seed,
            "definition_sha256": definition_hash,
            "parameter_archive_sha256": parameter_hash,
            "fit_provenance": cd.path_text(fit_report_path),
            "fit_provenance_sha256": fit_report_hash,
            "input_file_sha256": inputs,
            "software_versions": fit_report["software_versions"],
            "output": cd.path_text(member_path),
            "output_sha256": cd.sha256_file(member_path),
            "days": len(dates),
            "calendar": "365_day",
            "selected_transition_season_match": season_match,
            "unique_transition_records_used": int(np.unique(selected).size),
            "diagnostics": diagnostics,
            "created_utc": datetime.now(timezone.utc).isoformat(),
        }
        cd.write_json(report_path, report, args.overwrite)
        summaries.append(
            {"member": member, "seed": seed, "output": cd.path_text(member_path)}
        )
    print(
        json.dumps(
            {
                "status": "knn_analog_fit_and_sampling_complete",
                "conditioning_state": conditioning_state,
                "fit_report": cd.path_text(fit_report_path),
                "members": summaries,
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
