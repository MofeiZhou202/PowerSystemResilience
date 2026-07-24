#!/usr/bin/env python3
"""Fit and sample the first climate-state-conditioned weather-generator baseline."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import platform
import zipfile
from dataclasses import dataclass
from datetime import datetime, timezone
from math import erf, sqrt
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import cftime
import numpy as np
import xarray as xr

import climate_data as cd


LEGACY_DEFINITION = cd.TOOL_DIR / "manifests" / "weather_generator_baseline.json"
DEFAULT_DEFINITION = (
    cd.TOOL_DIR / "manifests" / "weather_generator_baseline_frozen.json"
)
VARIABLES = ("tasmax", "dtr", "hurs", "sfcWind", "rsds", "pr")
OUTPUT_VARIABLES = ("tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr")
UNITS = {
    "tasmax": "degC",
    "tasmin": "degC",
    "hurs": "%",
    "sfcWind": "m s-1",
    "rsds": "W m-2",
    "pr": "mm day-1",
}


@dataclass
class FittedGenerator:
    sorted_marginals: np.ndarray
    marginal_counts: np.ndarray
    centroids: np.ndarray
    initial_probabilities: np.ndarray
    transition_probabilities: np.ndarray
    coefficients: np.ndarray
    residuals: np.ndarray
    residual_months: np.ndarray
    residual_source_indices: np.ndarray
    fitted_spectral_radius: float
    effective_spectral_radius: float


def validate_definition(definition: Dict[str, Any]) -> None:
    if definition.get("model_variables") != list(VARIABLES):
        raise cd.ContractError(
            "Weather-generator variables do not match the code contract"
        )
    if definition["conditioning"].get("pool_gcms_or_ssps") is not False:
        raise cd.ContractError("The baseline must not pool GCM or SSP states")
    if definition["calendar"].get("generated_calendar") != "365_day":
        raise cd.ContractError("The baseline output calendar must be 365_day")
    if int(definition["weather_state_model"]["states"]) < 1:
        raise cd.ContractError("At least one weather state is required")
    if int(definition["innovation_model"]["block_length_days"]) < 1:
        raise cd.ContractError("Residual block length must be positive")
    freeze = definition.get("freeze_evidence")
    if freeze:
        for prefix in ("development", "confirmation"):
            path = cd.REPO_ROOT / freeze[f"{prefix}_report"]
            if not path.exists():
                raise cd.ContractError(f"Frozen baseline {prefix} evidence is missing")
            if cd.sha256_file(path) != freeze[f"{prefix}_report_sha256"]:
                raise cd.ContractError(
                    f"Frozen baseline {prefix} evidence checksum changed"
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
        ranks[order[start:end]] = 0.5 * (start + end - 1)
        start = end
    return ranks


def normal_ppf(probabilities: np.ndarray) -> np.ndarray:
    values = np.asarray(probabilities, dtype=float)
    if np.any((values <= 0.0) | (values >= 1.0)):
        raise cd.ContractError("Normal quantile probabilities must be inside (0,1)")
    a = (
        -39.6968302866538,
        220.946098424521,
        -275.928510446969,
        138.357751867269,
        -30.6647980661472,
        2.50662827745924,
    )
    b = (
        -54.4760987982241,
        161.585836858041,
        -155.698979859887,
        66.8013118877197,
        -13.2806815528857,
    )
    c = (
        -0.00778489400243029,
        -0.322396458041136,
        -2.40075827716184,
        -2.54973253934373,
        4.37466414146497,
        2.93816398269878,
    )
    d = (0.00778469570904146, 0.32246712907004, 2.445134137143, 3.75440866190742)
    output = np.empty_like(values)
    lower = values < 0.02425
    upper = values > 1.0 - 0.02425
    central = ~(lower | upper)
    if np.any(lower):
        q = np.sqrt(-2.0 * np.log(values[lower]))
        output[lower] = (
            ((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]
        ) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    if np.any(upper):
        q = np.sqrt(-2.0 * np.log(1.0 - values[upper]))
        output[upper] = -(
            (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5])
            / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
        )
    if np.any(central):
        q = values[central] - 0.5
        r = q * q
        output[central] = (
            (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5])
            * q
            / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0)
        )
    return output


def fit_monthly_marginals(
    values: np.ndarray, months: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if values.ndim != 2 or values.shape[1] != len(VARIABLES):
        raise cd.ContractError("Marginal input must be an n by 6 matrix")
    counts = np.asarray([np.count_nonzero(months == month) for month in range(1, 13)])
    if np.any(counts == 0):
        raise cd.ContractError("Every calendar month needs fitting observations")
    sorted_values = np.full((12, len(VARIABLES), int(np.max(counts))), np.nan)
    gaussian = np.empty_like(values, dtype=float)
    for month in range(1, 13):
        selected = np.flatnonzero(months == month)
        for column in range(len(VARIABLES)):
            sample = values[selected, column]
            if not np.all(np.isfinite(sample)):
                raise cd.ContractError("Non-finite value in marginal fitting data")
            sorted_values[month - 1, column, : sample.size] = np.sort(sample)
            probabilities = (average_ranks(sample) + 0.5) / sample.size
            gaussian[selected, column] = normal_ppf(probabilities)
    return gaussian, sorted_values, counts


def inverse_monthly_marginals(
    gaussian: np.ndarray,
    months: np.ndarray,
    sorted_values: np.ndarray,
    counts: np.ndarray,
) -> np.ndarray:
    output = np.empty_like(gaussian, dtype=float)
    probabilities = np.asarray(
        [
            [0.5 * (1.0 + erf(float(value) / sqrt(2.0))) for value in row]
            for row in gaussian
        ],
        dtype=float,
    )
    for month in range(1, 13):
        selected = np.flatnonzero(months == month)
        count = int(counts[month - 1])
        indices = np.minimum(
            np.floor(probabilities[selected] * count).astype(int), count - 1
        )
        for column in range(len(VARIABLES)):
            sample = sorted_values[month - 1, column, :count]
            output[selected, column] = sample[indices[:, column]]
    return output


def deterministic_kmeans(
    values: np.ndarray, states: int, maximum_iterations: int, tolerance: float
) -> Tuple[np.ndarray, np.ndarray]:
    if states > values.shape[0]:
        raise cd.ContractError("More weather states than fitting days")
    mean = np.mean(values, axis=0)
    centers = [values[int(np.argmax(np.sum((values - mean) ** 2, axis=1)))].copy()]
    while len(centers) < states:
        distances = np.min(
            np.stack([np.sum((values - center) ** 2, axis=1) for center in centers]),
            axis=0,
        )
        centers.append(values[int(np.argmax(distances))].copy())
    centroids = np.asarray(centers)
    labels = np.zeros(values.shape[0], dtype=int)
    for _ in range(maximum_iterations):
        distances = np.stack(
            [np.sum((values - center) ** 2, axis=1) for center in centroids], axis=1
        )
        new_labels = np.argmin(distances, axis=1)
        new_centroids = centroids.copy()
        for state in range(states):
            if np.any(new_labels == state):
                new_centroids[state] = np.mean(values[new_labels == state], axis=0)
        shift = float(np.max(np.abs(new_centroids - centroids)))
        labels, centroids = new_labels, new_centroids
        if shift <= tolerance:
            break
    order = sorted(range(states), key=lambda state: tuple(centroids[state]))
    remap = np.empty(states, dtype=int)
    remap[order] = np.arange(states)
    return remap[labels], centroids[order]


def fit_state_probabilities(
    labels: np.ndarray,
    months: np.ndarray,
    states: int,
    smoothing: float,
    transition_valid: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    if transition_valid is None:
        transition_valid = np.ones(labels.size - 1, dtype=bool)
    if transition_valid.shape != (labels.size - 1,):
        raise cd.ContractError("Weather-state transition mask has the wrong shape")
    initial = np.full((12, states), smoothing, dtype=float)
    transitions = np.full((12, states, states), smoothing, dtype=float)
    for month in range(1, 13):
        initial[month - 1] += np.bincount(labels[months == month], minlength=states)
    for index in range(1, labels.size):
        if transition_valid[index - 1]:
            transitions[months[index] - 1, labels[index - 1], labels[index]] += 1.0
    initial /= np.sum(initial, axis=1, keepdims=True)
    transitions /= np.sum(transitions, axis=2, keepdims=True)
    return initial, transitions


def fit_state_var(
    gaussian: np.ndarray,
    labels: np.ndarray,
    states: int,
    ridge_penalty: float,
    maximum_spectral_radius: float,
    diagonal_autoregression: bool = False,
    transition_valid: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, float, float]:
    dimension = gaussian.shape[1]
    if transition_valid is None:
        transition_valid = np.ones(gaussian.shape[0] - 1, dtype=bool)
    if transition_valid.shape != (gaussian.shape[0] - 1,):
        raise cd.ContractError("VAR transition mask has the wrong shape")
    previous = gaussian[:-1][transition_valid]
    current_labels = labels[1:][transition_valid]
    state_design = np.eye(states)[current_labels]
    response = gaussian[1:][transition_valid]
    if not np.all(np.isfinite(gaussian)) or not np.all(np.isfinite(response)):
        raise cd.ContractError("Non-finite Gaussian value in VAR fitting data")
    if diagonal_autoregression:
        coefficients = np.zeros((dimension + states, dimension), dtype=float)
        for column in range(dimension):
            column_design = np.column_stack((previous[:, column], state_design))
            augmented_design = np.vstack(
                (
                    column_design,
                    np.sqrt(ridge_penalty) * np.eye(1 + states),
                )
            )
            augmented_response = np.concatenate(
                (response[:, column], np.zeros(1 + states))
            )
            solution = np.linalg.lstsq(
                augmented_design, augmented_response, rcond=None
            )[0]
            coefficients[column, column] = solution[0]
            coefficients[dimension:, column] = solution[1:]
    else:
        design = np.column_stack((previous, state_design))
        augmented_design = np.vstack(
            (design, np.sqrt(ridge_penalty) * np.eye(dimension + states))
        )
        augmented_response = np.vstack(
            (response, np.zeros((dimension + states, dimension)))
        )
        coefficients = np.linalg.lstsq(
            augmented_design, augmented_response, rcond=None
        )[0]
    fitted_radius = float(np.max(np.abs(np.linalg.eigvals(coefficients[:dimension].T))))
    if fitted_radius > maximum_spectral_radius:
        coefficients[:dimension] *= maximum_spectral_radius / fitted_radius
        adjusted = response - row_matrix_product(previous, coefficients[:dimension])
        for state in range(states):
            selected = current_labels == state
            coefficients[dimension + state] = np.mean(adjusted[selected], axis=0)
    effective_radius = float(
        np.max(np.abs(np.linalg.eigvals(coefficients[:dimension].T)))
    )
    prediction = row_matrix_product(previous, coefficients[:dimension])
    prediction += coefficients[dimension:][current_labels]
    residuals = response - prediction
    return coefficients, residuals, fitted_radius, effective_radius


def row_matrix_product(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Small dense row products without platform BLAS floating-flag leakage."""
    return np.sum(left[..., :, np.newaxis] * right, axis=-2)


def fit_generator(
    values: np.ndarray,
    months: np.ndarray,
    definition: Dict[str, Any],
    diagonal_autoregression: bool = False,
    transition_valid: Optional[np.ndarray] = None,
) -> FittedGenerator:
    gaussian, marginals, counts = fit_monthly_marginals(values, months)
    state_spec = definition["weather_state_model"]
    states = int(state_spec["states"])
    labels, centroids = deterministic_kmeans(
        gaussian,
        states,
        int(state_spec["maximum_iterations"]),
        float(state_spec["convergence_tolerance"]),
    )
    initial, transitions = fit_state_probabilities(
        labels,
        months,
        states,
        float(state_spec["laplace_smoothing"]),
        transition_valid,
    )
    dynamic = definition["dynamic_dependence_model"]
    coefficients, residuals, fitted_radius, effective_radius = fit_state_var(
        gaussian,
        labels,
        states,
        float(dynamic["ridge_penalty"]),
        float(dynamic["maximum_spectral_radius"]),
        diagonal_autoregression,
        transition_valid,
    )
    return FittedGenerator(
        marginals,
        counts,
        centroids,
        initial,
        transitions,
        coefficients,
        residuals,
        months[1:][
            np.ones(months.size - 1, dtype=bool)
            if transition_valid is None
            else transition_valid
        ].copy(),
        np.flatnonzero(
            np.ones(months.size - 1, dtype=bool)
            if transition_valid is None
            else transition_valid
        )
        + 1,
        fitted_radius,
        effective_radius,
    )


def season(month: int) -> int:
    if month in (12, 1, 2):
        return 0
    if month in (3, 4, 5):
        return 1
    if month in (6, 7, 8):
        return 2
    return 3


def sample_residual_blocks(
    fitted: FittedGenerator,
    target_months: np.ndarray,
    block_length: int,
    rng: np.random.Generator,
    candidate_cache: Optional[Dict[Tuple[int, int], np.ndarray]] = None,
) -> np.ndarray:
    sampled = np.empty((target_months.size, fitted.residuals.shape[1]), dtype=float)
    source_seasons = np.asarray(
        [season(int(month)) for month in fitted.residual_months]
    )
    target_seasons = np.asarray([season(int(month)) for month in target_months])
    if candidate_cache is None:
        candidate_cache = {}
    cursor = 0
    while cursor < target_months.size:
        current_season = target_seasons[cursor]
        season_end = cursor + 1
        while (
            season_end < target_months.size
            and target_seasons[season_end] == current_season
        ):
            season_end += 1
        length = min(block_length, season_end - cursor)
        cache_key = (int(current_season), length)
        if cache_key not in candidate_cache:
            candidate_cache[cache_key] = np.asarray(
                [
                    start
                    for start in range(0, fitted.residuals.shape[0] - length + 1)
                    if np.all(source_seasons[start : start + length] == current_season)
                    and np.all(
                        np.diff(fitted.residual_source_indices[start : start + length])
                        == 1
                    )
                ],
                dtype=int,
            )
        candidates = candidate_cache[cache_key]
        if candidates.size == 0:
            raise cd.ContractError(
                f"No residual block available for season {current_season}"
            )
        source = int(candidates[int(rng.integers(candidates.size))])
        sampled[cursor : cursor + length] = fitted.residuals[source : source + length]
        cursor += length
    return sampled


def sample_generator(
    fitted: FittedGenerator,
    months: np.ndarray,
    definition: Dict[str, Any],
    seed: int,
    block_length_override: Optional[int] = None,
    independent_residual_columns: bool = False,
    residual_candidate_cache: Optional[Dict[Tuple[int, int], np.ndarray]] = None,
) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
    rng = np.random.default_rng(seed)
    states = fitted.centroids.shape[0]
    labels = np.empty(months.size, dtype=int)
    labels[0] = int(rng.choice(states, p=fitted.initial_probabilities[months[0] - 1]))
    for index in range(1, months.size):
        labels[index] = int(
            rng.choice(
                states,
                p=fitted.transition_probabilities[months[index] - 1, labels[index - 1]],
            )
        )
    block_length = (
        int(definition["innovation_model"]["block_length_days"])
        if block_length_override is None
        else block_length_override
    )
    if independent_residual_columns:
        residuals = np.empty((months.size, fitted.residuals.shape[1]), dtype=float)
        for column in range(fitted.residuals.shape[1]):
            sampled = sample_residual_blocks(
                fitted, months, block_length, rng, residual_candidate_cache
            )
            residuals[:, column] = sampled[:, column]
    else:
        residuals = sample_residual_blocks(
            fitted, months, block_length, rng, residual_candidate_cache
        )
    dimension = len(VARIABLES)
    gaussian = np.empty((months.size, dimension), dtype=float)
    gaussian[0] = fitted.centroids[labels[0]] + residuals[0]
    autoregression = fitted.coefficients[:dimension]
    state_intercepts = fitted.coefficients[dimension:]
    for index in range(1, months.size):
        gaussian[index] = (
            row_matrix_product(gaussian[index - 1], autoregression)
            + state_intercepts[labels[index]]
            + residuals[index]
        )
    transformed = inverse_monthly_marginals(
        gaussian, months, fitted.sorted_marginals, fitted.marginal_counts
    )
    tasmax = transformed[:, 0]
    dtr = np.maximum(transformed[:, 1], 0.0)
    output = {
        "tasmax": tasmax,
        "tasmin": tasmax - dtr,
        "hurs": np.clip(transformed[:, 2], 0.0, 100.0),
        "sfcWind": np.maximum(transformed[:, 3], 0.0),
        "rsds": np.maximum(transformed[:, 4], 0.0),
        "pr": np.maximum(transformed[:, 5], 0.0),
    }
    return output, labels


def noleap_dates(start_year: int, end_year: int) -> Tuple[list, np.ndarray]:
    dates = []
    months = []
    month_lengths = (31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31)
    for year in range(start_year, end_year + 1):
        for month, days in enumerate(month_lengths, start=1):
            for day in range(1, days + 1):
                dates.append(cftime.DatetimeNoLeap(year, month, day, 12))
                months.append(month)
    return dates, np.asarray(months, dtype=int)


def derive_member_seed(
    base_seed: int, method: str, model: str, experiment: str, site: str, member: int
) -> int:
    material = f"{base_seed}|{method}|{model}|{experiment}|{site}|{member}".encode()
    return int.from_bytes(hashlib.sha256(material).digest()[:8], "big")


def period_years(study: Dict[str, Any], experiment: str) -> Tuple[int, int]:
    period = cd.nex_period_for_experiment(study, experiment)
    return int(period["start"][:4]), int(period["end"][:4])


def load_conditioning_data(
    study: Dict[str, Any], model: str, experiment: str, site: str
) -> Tuple[np.ndarray, np.ndarray, list, Dict[str, float]]:
    start_year, end_year = period_years(study, experiment)
    data_root = cd.REPO_ROOT / study["data_root"]
    arrays = {name: [] for name in OUTPUT_VARIABLES}
    months = []
    inputs = []
    coordinates: Dict[str, float] = {}
    for year in range(start_year, end_year + 1):
        directory = (
            data_root
            / "processed/daily/nex_gddp_cmip6"
            / f"model={model}"
            / f"experiment={experiment}"
            / f"site={site}"
            / f"year={year}"
        )
        matches = list(directory.glob("daily_nex_gddp_cmip6_*.nc"))
        if len(matches) != 1:
            raise cd.ContractError(f"Expected one harmonized NEX file in {directory}")
        path = matches[0]
        with xr.open_dataset(path) as dataset:
            text_dates = np.asarray([str(value)[:10] for value in dataset.time.values])
            keep = np.asarray([value[5:] != "02-29" for value in text_dates])
            months.extend(int(value[5:7]) for value in text_dates[keep])
            for name in OUTPUT_VARIABLES:
                values = np.asarray(dataset[name].values, dtype=float)[keep]
                arrays[name].append(values)
            candidate_coordinates = {
                "latitude": float(dataset.latitude.values),
                "longitude": float(dataset.longitude.values),
            }
            if coordinates and coordinates != candidate_coordinates:
                raise cd.ContractError(
                    "Selected NEX grid cell changes within the epoch"
                )
            coordinates = candidate_coordinates
        inputs.append({"path": cd.path_text(path), "sha256": cd.sha256_file(path)})
    combined = {name: np.concatenate(parts) for name, parts in arrays.items()}
    if np.any(combined["tasmin"] > combined["tasmax"]):
        raise cd.ContractError("Conditioning input has tasmin greater than tasmax")
    matrix = np.column_stack(
        (
            combined["tasmax"],
            combined["tasmax"] - combined["tasmin"],
            combined["hurs"],
            combined["sfcWind"],
            combined["rsds"],
            combined["pr"],
        )
    )
    expected = (end_year - start_year + 1) * 365
    if matrix.shape != (expected, len(VARIABLES)):
        raise cd.ContractError(
            f"Expected {(expected, len(VARIABLES))}, found {matrix.shape}"
        )
    return matrix, np.asarray(months, dtype=int), inputs, coordinates


def save_fit(path: Path, fitted: FittedGenerator) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {
        "centroids": fitted.centroids,
        "coefficients": fitted.coefficients,
        "effective_spectral_radius": np.asarray(fitted.effective_spectral_radius),
        "fitted_spectral_radius": np.asarray(fitted.fitted_spectral_radius),
        "initial_probabilities": fitted.initial_probabilities,
        "marginal_counts": fitted.marginal_counts,
        "residual_months": fitted.residual_months,
        "residual_source_indices": fitted.residual_source_indices,
        "residuals": fitted.residuals,
        "sorted_marginals": fitted.sorted_marginals,
        "transition_probabilities": fitted.transition_probabilities,
    }
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(arrays):
            buffer = io.BytesIO()
            np.lib.format.write_array(
                buffer, np.asarray(arrays[name]), allow_pickle=False
            )
            entry = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
            entry.compress_type = zipfile.ZIP_DEFLATED
            entry.create_system = 3
            entry.external_attr = 0o600 << 16
            archive.writestr(entry, buffer.getvalue())


def load_fit(path: Path) -> FittedGenerator:
    with np.load(path, allow_pickle=False) as arrays:
        residuals = arrays["residuals"]
        residual_source_indices = (
            arrays["residual_source_indices"]
            if "residual_source_indices" in arrays.files
            else np.arange(1, residuals.shape[0] + 1)
        )
        return FittedGenerator(
            sorted_marginals=arrays["sorted_marginals"],
            marginal_counts=arrays["marginal_counts"],
            centroids=arrays["centroids"],
            initial_probabilities=arrays["initial_probabilities"],
            transition_probabilities=arrays["transition_probabilities"],
            coefficients=arrays["coefficients"],
            residuals=residuals,
            residual_months=arrays["residual_months"],
            residual_source_indices=residual_source_indices,
            fitted_spectral_radius=float(arrays["fitted_spectral_radius"]),
            effective_spectral_radius=float(arrays["effective_spectral_radius"]),
        )


def write_member(
    path: Path,
    values: Dict[str, np.ndarray],
    labels: np.ndarray,
    dates: list,
    attributes: Dict[str, Any],
) -> None:
    dataset = xr.Dataset(
        data_vars={
            **{
                name: ("time", values[name].astype("float32"), {"units": UNITS[name]})
                for name in OUTPUT_VARIABLES
            },
            "weather_state": (
                "time",
                labels.astype("int8"),
                {
                    "long_name": "unlabelled statistical weather-state cluster",
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
        for name in (*OUTPUT_VARIABLES, "weather_state")
    }
    encoding["time"] = {"calendar": "365_day"}
    dataset.to_netcdf(path, encoding=encoding)
    dataset.close()


def output_diagnostics(values: Dict[str, np.ndarray]) -> Dict[str, Any]:
    checks = {
        "all_finite": all(np.all(np.isfinite(value)) for value in values.values()),
        "tasmin_not_greater_than_tasmax": bool(
            np.all(values["tasmin"] <= values["tasmax"])
        ),
        "hurs_inside_0_100": bool(
            np.all((values["hurs"] >= 0.0) & (values["hurs"] <= 100.0))
        ),
        "nonnegative_wind_radiation_precipitation": bool(
            np.all(values["sfcWind"] >= 0.0)
            and np.all(values["rsds"] >= 0.0)
            and np.all(values["pr"] >= 0.0)
        ),
    }
    if not all(checks.values()):
        raise cd.ContractError(f"Generated-member quality gate failed: {checks}")
    return {
        "checks": checks,
        "variables": {
            name: {"minimum": float(np.min(value)), "maximum": float(np.max(value))}
            for name, value in values.items()
        },
    }


def ensure_absent(paths: Iterable[Path], overwrite: bool) -> None:
    existing = [str(path) for path in paths if path.exists()]
    if existing and not overwrite:
        raise cd.ContractError(f"Refusing to overwrite existing outputs: {existing}")


def verify_reusable_fit(
    report: Dict[str, Any],
    expected_state: Dict[str, str],
    definition_hash: str,
    parameter_path: Path,
) -> None:
    if report.get("conditioning_state") != expected_state:
        raise cd.ContractError("Reusable fit conditioning state does not match request")
    if report.get("definition_sha256") != definition_hash:
        raise cd.ContractError("Reusable fit method definition checksum has changed")
    if report.get("parameter_archive_sha256") != cd.sha256_file(parameter_path):
        raise cd.ContractError("Reusable parameter archive checksum mismatch")
    input_records = report.get("inputs", [])
    if len(input_records) != 30:
        raise cd.ContractError("Reusable fit must retain all 30 annual input records")
    for record in input_records:
        path = Path(record["path"])
        if not path.is_absolute():
            path = cd.REPO_ROOT / path
        if not path.exists() or cd.sha256_file(path) != record["sha256"]:
            raise cd.ContractError(f"Reusable fit input checksum mismatch: {path}")


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
        raise cd.ContractError("At least one member is required")
    start_year, end_year = period_years(study, args.experiment)
    method = definition["method_id"]
    root = (
        args.output_root
        or cd.REPO_ROOT / study["data_root"] / "processed/weather_generator"
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
            raise cd.ContractError("--reuse-fit requires parameters and fit provenance")
        ensure_absent([*member_paths, *member_reports], args.overwrite)
        fit_report = cd.load_json(fit_report_path)
        verify_reusable_fit(
            fit_report, conditioning_state, definition_hash, parameter_path
        )
        fitted = load_fit(parameter_path)
        inputs = fit_report["inputs"]
        coordinates = fit_report["coordinates"]
        parameter_hash = fit_report["parameter_archive_sha256"]
    else:
        ensure_absent(
            [parameter_path, fit_report_path, *member_paths, *member_reports],
            args.overwrite,
        )
        matrix, months, inputs, coordinates = load_conditioning_data(
            study, args.model, args.experiment, args.site
        )
        fitted = fit_generator(matrix, months, definition)
        save_fit(parameter_path, fitted)
        parameter_hash = cd.sha256_file(parameter_path)
        fit_report = {
            "schema_version": "1.0",
            "operation": "fit_weather_generator_baseline",
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
            "parameter_archive": cd.path_text(parameter_path),
            "parameter_archive_sha256": parameter_hash,
            "fitted_spectral_radius": fitted.fitted_spectral_radius,
            "effective_spectral_radius": fitted.effective_spectral_radius,
            "software_versions": {
                "python": platform.python_version(),
                "numpy": np.__version__,
                "xarray": xr.__version__,
                "cftime": cftime.__version__,
            },
            "created_utc": datetime.now(timezone.utc).isoformat(),
        }
        cd.write_json(fit_report_path, fit_report, args.overwrite)
    fit_report_hash = cd.sha256_file(fit_report_path)
    dates, target_months = noleap_dates(start_year, end_year)
    summaries = []
    for member, (member_path, report_path) in enumerate(
        zip(member_paths, member_reports)
    ):
        seed = derive_member_seed(
            args.base_seed, method, args.model, args.experiment, args.site, member
        )
        values, labels = sample_generator(fitted, target_months, definition, seed)
        diagnostics = output_diagnostics(values)
        write_member(
            member_path,
            values,
            labels,
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
                "probability_semantics": "conditional_on_model_experiment_site_epoch",
                "member": member,
                "member_seed": str(seed),
                "parameter_archive_sha256": parameter_hash,
                **coordinates,
            },
        )
        report = {
            "schema_version": "1.0",
            "operation": "sample_weather_generator_baseline",
            "conditioning_state": fit_report["conditioning_state"],
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
                "status": "baseline_fit_and_sampling_complete",
                "conditioning_state": fit_report["conditioning_state"],
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
