#!/usr/bin/env python3
"""Apply the frozen weather-to-grid mapping and build one resilience bundle."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import xarray as xr

import climate_data as cd
import future_weather_batch as future


DEFAULT_DEFINITION = future.DEFAULT_DEFINITION


def logistic(values: np.ndarray, midpoint: float, scale: float) -> np.ndarray:
    if scale <= 0.0:
        raise cd.ContractError("Logistic scale must be positive")
    z = np.clip((values - midpoint) / scale, -50.0, 50.0)
    return 1.0 / (1.0 + np.exp(-z))


def compound_stress(
    tasmax: np.ndarray,
    hurs: np.ndarray,
    wind: np.ndarray,
    precipitation: np.ndarray,
    definition: Dict[str, Any],
) -> np.ndarray:
    components = definition["weather_to_grid"]["compound_stress"]["components"]
    inputs = {
        "heat": tasmax,
        "humidity": hurs,
        "wind": wind,
        "precipitation": precipitation,
    }
    result = np.zeros_like(tasmax, dtype=float)
    weight_sum = 0.0
    for name, values in inputs.items():
        spec = components[name]
        weight = float(spec["weight"])
        result += weight * logistic(
            values,
            float(spec["logistic_midpoint"]),
            float(spec["logistic_scale"]),
        )
        weight_sum += weight
    if not math.isclose(weight_sum, 1.0, abs_tol=1e-12):
        raise cd.ContractError("Compound-stress weights must sum to one")
    return result


def event_window_start(stress: np.ndarray, days: int) -> Tuple[int, np.ndarray]:
    if days < 1 or stress.size < days:
        raise cd.ContractError("Event window is longer than the weather path")
    rolling = np.convolve(stress, np.ones(days, dtype=float) / days, mode="valid")
    start = int(np.argmax(rolling))
    return start, rolling


def wind_capacity_factor(speed: np.ndarray, spec: Dict[str, Any]) -> np.ndarray:
    cut_in = float(spec["cut_in_m_s"])
    rated = float(spec["rated_m_s"])
    cut_out = float(spec["cut_out_m_s"])
    result = np.zeros_like(speed, dtype=float)
    ramp = (speed >= cut_in) & (speed < rated)
    result[ramp] = (speed[ramp] ** 3 - cut_in**3) / (rated**3 - cut_in**3)
    result[(speed >= rated) & (speed < cut_out)] = 1.0
    return np.clip(result, 0.0, 1.0)


def reconstruct_event_profiles(
    arrays: Dict[str, np.ndarray], start: int, definition: Dict[str, Any]
) -> Dict[str, np.ndarray]:
    mapping = definition["weather_to_grid"]
    days = int(mapping["event_window_days"])
    selected = slice(start, start + days)
    hours = np.arange(24, dtype=float)
    hourly_temperature = []
    hourly_humidity = []
    hourly_ghi = []
    hourly_wind = []
    solar_shape = np.maximum(0.0, np.sin(np.pi * (hours - 6.0) / 12.0))
    solar_shape /= float(np.mean(solar_shape))
    wind_spec = mapping["wind"]
    hub_ratio = (
        float(wind_spec["hub_height_m"])
        / float(wind_spec["measurement_height_m"])
    ) ** float(wind_spec["shear_exponent"])
    for offset in range(days):
        index = start + offset
        mean_temperature = 0.5 * (
            arrays["tasmax"][index] + arrays["tasmin"][index]
        )
        amplitude = 0.5 * (
            arrays["tasmax"][index] - arrays["tasmin"][index]
        )
        hourly_temperature.append(
            mean_temperature + amplitude * np.cos(2.0 * np.pi * (hours - 15.0) / 24.0)
        )
        hourly_humidity.append(np.full(24, arrays["hurs"][index], dtype=float))
        hourly_ghi.append(arrays["rsds"][index] * solar_shape)
        hourly_wind.append(
            np.full(24, arrays["sfcWind"][index] * hub_ratio, dtype=float)
        )
    temperature = np.concatenate(hourly_temperature)
    humidity = np.concatenate(hourly_humidity)
    ghi = np.concatenate(hourly_ghi)
    hub_wind = np.concatenate(hourly_wind)
    load_spec = mapping["load"]
    base_profile = np.asarray(load_spec["base_hourly_profile"], dtype=float)
    base_profile /= float(np.mean(base_profile))
    tiled_base = np.tile(base_profile, days)
    weather_load = (
        1.0
        + float(load_spec["cooling_slope_per_degC"])
        * np.maximum(
            temperature - float(load_spec["cooling_balance_temperature_degC"]),
            0.0,
        )
        + float(load_spec["heating_slope_per_degC"])
        * np.maximum(
            float(load_spec["heating_balance_temperature_degC"]) - temperature,
            0.0,
        )
        + float(load_spec["humidity_slope_per_percent"])
        * np.maximum(
            humidity - float(load_spec["humidity_threshold_percent"]), 0.0
        )
    )
    load_limits = [float(value) for value in load_spec["multiplier_limits"]]
    load = np.clip(tiled_base * weather_load, *load_limits)
    pv_spec = mapping["pv"]
    cell_temperature = temperature + (
        float(pv_spec["nominal_operating_cell_temperature_degC"]) - 20.0
    ) / 800.0 * ghi
    pv = ghi / float(pv_spec["reference_irradiance_W_m2"]) * (
        1.0
        + float(pv_spec["temperature_coefficient_per_degC"])
        * (cell_temperature - 25.0)
    )
    pv_limits = [float(value) for value in pv_spec["capacity_factor_limits"]]
    pv = np.clip(pv, *pv_limits)
    wind = wind_capacity_factor(hub_wind, wind_spec)
    return {
        "temperature_degC": temperature,
        "humidity_percent": humidity,
        "GHI_W_m2": ghi,
        "hub_wind_m_s": hub_wind,
        "load_multiplier": load,
        "pv_capacity_factor": pv,
        "wind_capacity_factor": wind,
        "selected_daily_tasmax": arrays["tasmax"][selected],
        "selected_daily_pr": arrays["pr"][selected],
    }


def read_member(path: Path) -> Tuple[Dict[str, np.ndarray], np.ndarray]:
    with xr.open_dataset(path) as dataset:
        required = ("tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr")
        missing = sorted(set(required) - set(dataset.data_vars))
        if missing:
            raise cd.ContractError(f"{path} lacks generated variables {missing}")
        arrays = {
            name: np.asarray(dataset[name].values, dtype=float) for name in required
        }
        dates = np.asarray([str(value)[:10] for value in dataset.time.values])
    lengths = {values.size for values in arrays.values()} | {dates.size}
    if len(lengths) != 1:
        raise cd.ContractError(f"Generated member arrays are misaligned: {path}")
    return arrays, dates


def member_summary(
    arrays: Dict[str, np.ndarray], stress: np.ndarray, years: int
) -> Dict[str, float]:
    max_three_day_pr = float(
        np.max(np.convolve(arrays["pr"], np.ones(3), mode="valid"))
    )
    hot_dry = (arrays["tasmax"] >= 35.0) & (arrays["pr"] < 1.0)
    return {
        "tasmax_mean_degC": float(np.mean(arrays["tasmax"])),
        "tasmax_q99_degC": float(np.quantile(arrays["tasmax"], 0.99)),
        "pr_q99_mm_day": float(np.quantile(arrays["pr"], 0.99)),
        "maximum_three_day_precipitation_mm": max_three_day_pr,
        "hot_dry_days_per_year": float(np.count_nonzero(hot_dry) / years),
        "maximum_three_day_mean_compound_stress": float(
            np.max(np.convolve(stress, np.ones(3) / 3.0, mode="valid"))
        ),
    }


def build_case(
    member_path: Path,
    method: str,
    model: str,
    experiment: str,
    site: str,
    member: int,
    definition: Dict[str, Any],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    arrays, dates = read_member(member_path)
    stress = compound_stress(
        arrays["tasmax"],
        arrays["hurs"],
        arrays["sfcWind"],
        arrays["pr"],
        definition,
    )
    days = int(definition["weather_to_grid"]["event_window_days"])
    start, rolling = event_window_start(stress, days)
    profiles = reconstruct_event_profiles(arrays, start, definition)
    event_stress = float(rolling[start])
    fault_spec = definition["weather_to_grid"]["faults"]
    faults = []
    for branch, outage_start, base_repair in zip(
        fault_spec["AC_branch_indices"],
        fault_spec["outage_start_hours"],
        fault_spec["base_repair_hours"],
    ):
        faults.append(
            {
                "ac_branch_index": int(branch),
                "outage_start_hr": float(outage_start),
                "repair_duration_hr": float(base_repair)
                * (1.0 + 2.0 * event_stress),
                "name": f"fixed_weather_stress_branch_{branch}",
            }
        )
    case_id = f"{model}:{experiment}:{site}:{method}:member-{member:04d}"
    case = {
        "id": case_id,
        "model": model,
        "experiment": experiment,
        "site": site,
        "method": method,
        "member": member,
        "source_weather": cd.path_text(member_path),
        "source_weather_sha256": cd.sha256_file(member_path),
        "event_start_date": str(dates[start]),
        "event_end_date": str(dates[start + days - 1]),
        "event_compound_stress": event_stress,
        "load_profile": profiles["load_multiplier"].tolist(),
        "pv_profile": profiles["pv_capacity_factor"].tolist(),
        "wind_profile": profiles["wind_capacity_factor"].tolist(),
        "faults": faults,
        "weather_diagnostics": {
            "peak_temperature_degC": float(np.max(profiles["temperature_degC"])),
            "peak_GHI_W_m2": float(np.max(profiles["GHI_W_m2"])),
            "peak_hub_wind_m_s": float(np.max(profiles["hub_wind_m_s"])),
            "event_precipitation_mm": float(
                np.sum(profiles["selected_daily_pr"])
            ),
            "peak_load_multiplier": float(np.max(profiles["load_multiplier"])),
            "mean_pv_capacity_factor": float(
                np.mean(profiles["pv_capacity_factor"])
            ),
            "mean_wind_capacity_factor": float(
                np.mean(profiles["wind_capacity_factor"])
            ),
        },
    }
    years = arrays["tasmax"].size // 365
    summary = {
        "model": model,
        "experiment": experiment,
        "site": site,
        "method": method,
        "member": member,
        **member_summary(arrays, stress, years),
    }
    return case, summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--definition", type=Path, default=DEFAULT_DEFINITION)
    parser.add_argument("--batch-report", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    definition = cd.load_json(args.definition)
    future.validate_definition(definition, study)
    batch_path = args.batch_report or (
        cd.REPO_ROOT / study["data_root"] / "provenance/future_weather_batch.json"
    )
    batch = cd.load_json(batch_path)
    if batch.get("status") != "complete":
        raise cd.ContractError("Future weather batch is not complete")
    if batch.get("definition_sha256") != cd.sha256_file(args.definition):
        raise cd.ContractError("Future weather batch definition checksum changed")
    generation = definition["weather_generation"]
    case_site = definition["weather_to_grid"]["case_site"]
    output_dir = args.output_dir or (
        cd.REPO_ROOT / study["data_root"] / "processed/future_weather_study"
    )
    bundle_path = output_dir / "resilience_case_bundle.json"
    summary_path = output_dir / "future_weather_member_summary.csv"
    provenance_path = output_dir / "weather_to_grid_provenance.json"
    if not args.overwrite and any(
        path.exists() for path in (bundle_path, summary_path, provenance_path)
    ):
        raise cd.ContractError("Refusing to overwrite weather-to-grid outputs")
    cases = []
    summaries = []
    for record in batch["records"]:
        if not record["valid"] or record["disposition"] == "planned":
            continue
        directory = cd.REPO_ROOT / record["state_directory"]
        for member in range(int(generation["members_per_state"])):
            member_path = directory / f"member={member:04d}.nc"
            case, summary = build_case(
                member_path,
                record["method"],
                record["model"],
                record["experiment"],
                record["site"],
                member,
                definition,
            )
            summaries.append(summary)
            if record["site"] == case_site:
                cases.append(case)
    expected_summaries = int(generation["expected_generated_members"])
    expected_cases = (
        len(cd.models_for_set(study, "outer"))
        * len(generation["experiments"])
        * len(generation["methods"])
        * int(generation["members_per_state"])
    )
    if len(summaries) != expected_summaries or len(cases) != expected_cases:
        raise cd.ContractError(
            f"Weather-to-grid inventory mismatch: {len(summaries)} summaries, {len(cases)} cases"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    bundle = {
        "schema_version": "1.0",
        "operation": "fixed_weather_to_grid_resilience_case_bundle",
        "study_id": definition["study_id"],
        "definition": cd.path_text(args.definition),
        "definition_sha256": cd.sha256_file(args.definition),
        "batch_report": cd.path_text(batch_path),
        "batch_report_sha256": cd.sha256_file(batch_path),
        "network": definition["resilience_case"]["network"],
        "network_sha256": cd.sha256_file(
            cd.REPO_ROOT / definition["resilience_case"]["network"]
        ),
        "resilience_case": definition["resilience_case"],
        "case_count": len(cases),
        "cases": cases,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    cd.write_json(bundle_path, bundle, args.overwrite)
    fieldnames = list(summaries[0])
    with summary_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)
    provenance = {
        "schema_version": "1.0",
        "operation": "weather_to_grid_mapping_and_future_weather_summary",
        "definition_sha256": cd.sha256_file(args.definition),
        "batch_report_sha256": cd.sha256_file(batch_path),
        "weather_member_count": len(summaries),
        "resilience_case_count": len(cases),
        "case_bundle": cd.path_text(bundle_path),
        "case_bundle_sha256": cd.sha256_file(bundle_path),
        "weather_summary": cd.path_text(summary_path),
        "weather_summary_sha256": cd.sha256_file(summary_path),
        "mapping_interpretation": definition["reporting"]["interpretation"],
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    cd.write_json(provenance_path, provenance, args.overwrite)
    print(json.dumps(provenance, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except cd.ContractError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2)
