#!/usr/bin/env python3
"""Reproducible data staging for the Shaanxi climate-scenario pilot.

The default commands are offline: they validate the study contract, estimate
volume, and render exact request records. Network retrieval is opt-in.
"""

from __future__ import annotations

import argparse
import calendar
import concurrent.futures
import csv
import hashlib
import json
import math
import re
import shutil
import sys
import tempfile
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile
from datetime import date, datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


TOOL_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOL_DIR.parent.parent
DEFAULT_STUDY = TOOL_DIR / "manifests" / "shaanxi_pilot.json"
DEFAULT_SOURCES = TOOL_DIR / "manifests" / "sources.json"
DEFAULT_HISTORICAL_DIAGNOSTICS = TOOL_DIR / "manifests" / "historical_diagnostics.json"
DEFAULT_OBSERVATION_VALIDATION = TOOL_DIR / "manifests" / "observation_validation.json"
DEFAULT_HOURLY_HEAT_STRESS = TOOL_DIR / "manifests" / "hourly_heat_stress.json"

ERA5_TIMESERIES_DATASET_ID = "reanalysis-era5-land-timeseries"
ERA5_TIMESERIES_GROUPS = {
    "temperature_humidity": ["2m_temperature", "2m_dewpoint_temperature"],
    "precipitation": ["total_precipitation"],
    "solar_radiation": ["surface_solar_radiation_downwards"],
    "wind": ["10m_u_component_of_wind", "10m_v_component_of_wind"],
    "soil_moisture": [
        "volumetric_soil_water_level_1",
        "volumetric_soil_water_level_2",
        "volumetric_soil_water_level_3",
    ],
}
MODEL_SET_CHOICES = ("pilot", "outer", "all")
REALIZATION_PATTERN = re.compile(r"^r\d+i\d+p\d+f\d+$")


class ContractError(ValueError):
    pass


def models_for_set(study: Dict[str, Any], model_set: str = "pilot") -> List[str]:
    if model_set not in MODEL_SET_CHOICES:
        raise ContractError(
            f"Unknown model set {model_set!r}; expected one of {MODEL_SET_CHOICES}"
        )
    states = study.get("climate_states", {})
    pilot = list(states.get("pilot_gcms", []))
    outer = list(states.get("outer_evaluation_gcms", []))
    if model_set == "pilot":
        return pilot
    if model_set == "outer":
        return outer
    return pilot + outer


def require_model_in_set(
    study: Dict[str, Any], model: str, model_set: str = "pilot"
) -> None:
    if model not in models_for_set(study, model_set):
        raise ContractError(f"Model {model} is outside the {model_set} GCM set")


def nex_realization(study: Dict[str, Any], model: str) -> str:
    realizations = study.get("climate_states", {}).get("gcm_realizations", {})
    realization = realizations.get(model) if isinstance(realizations, dict) else None
    if not isinstance(realization, str) or not REALIZATION_PATTERN.fullmatch(realization):
        raise ContractError(f"Model {model} lacks a valid declared NEX realization")
    return realization


def nex_request_calendar(study: Dict[str, Any], model: str) -> str:
    overrides = study.get("climate_states", {}).get(
        "nex_request_calendar_overrides", {}
    )
    if not isinstance(overrides, dict):
        raise ContractError("nex_request_calendar_overrides must be an object")
    calendar_name = overrides.get(model, "standard")
    if calendar_name not in {"standard", "365_day", "noleap", "360_day"}:
        raise ContractError(
            f"Model {model} has unsupported request calendar {calendar_name!r}"
        )
    return calendar_name


def load_json(path: Path) -> Dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            value = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise ContractError(f"Cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ContractError(f"Top-level JSON value must be an object: {path}")
    return value


def write_json(path: Path, value: Any, overwrite: bool = False) -> None:
    if path.exists() and not overwrite:
        raise ContractError(f"Refusing to overwrite existing file: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=True, indent=2, sort_keys=True)
        handle.write("\n")


def parse_iso_day(value: str, field: str) -> date:
    try:
        return date.fromisoformat(value)
    except (TypeError, ValueError) as exc:
        raise ContractError(f"{field} must be an ISO date, got {value!r}") from exc


def validate_bbox(bbox: Any, field: str) -> Tuple[float, float, float, float]:
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise ContractError(f"{field} must be [west, south, east, north]")
    if not all(isinstance(value, (int, float)) for value in bbox):
        raise ContractError(f"{field} contains a non-numeric coordinate")
    west, south, east, north = (float(value) for value in bbox)
    if not (-180 <= west < east <= 180 and -90 <= south < north <= 90):
        raise ContractError(f"{field} is not a valid EPSG:4326 bounding box")
    return west, south, east, north


def contains(bbox: Sequence[float], lon: float, lat: float) -> bool:
    west, south, east, north = bbox
    return west <= lon <= east and south <= lat <= north


def source_index(sources: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    rows = sources.get("sources")
    if not isinstance(rows, list) or not rows:
        raise ContractError("sources.json must contain a non-empty sources array")
    indexed: Dict[str, Dict[str, Any]] = {}
    for position, row in enumerate(rows):
        if not isinstance(row, dict) or not isinstance(row.get("id"), str):
            raise ContractError(f"sources[{position}] requires a string id")
        if row["id"] in indexed:
            raise ContractError(f"Duplicate source id: {row['id']}")
        indexed[row["id"]] = row
    return indexed


def resolve_nex_native_units(
    source: Dict[str, Any], model: str, variable: str, declared_units: str
) -> Tuple[str, Optional[Dict[str, Any]]]:
    """Resolve one explicitly registered upstream metadata omission."""
    variable_spec = source["variables"][variable]
    if declared_units in variable_spec["accepted_units"]:
        return declared_units, None
    matches = [
        repair
        for repair in source.get("metadata_repairs", [])
        if repair.get("model") == model
        and repair.get("variable") == variable
        and repair.get("match_declared_units") == declared_units
    ]
    if len(matches) > 1:
        raise ContractError(
            f"Multiple NEX metadata repairs match {model}/{variable} units {declared_units!r}"
        )
    if not matches:
        return declared_units, None
    repair = matches[0]
    effective_units = str(repair.get("effective_units", ""))
    if effective_units not in variable_spec["accepted_units"]:
        raise ContractError(
            f"NEX metadata repair {repair.get('id')!r} resolves to unsupported units "
            f"{effective_units!r}"
        )
    return effective_units, repair


def validate_contract(study: Dict[str, Any], sources: Dict[str, Any]) -> Dict[str, Any]:
    errors: List[str] = []
    warnings: List[str] = []

    def check(condition: bool, message: str) -> None:
        if not condition:
            errors.append(message)

    check(study.get("schema_version") == "1.0", "Unsupported study schema_version")
    check(sources.get("schema_version") == "1.0", "Unsupported sources schema_version")
    try:
        source_map = source_index(sources)
    except ContractError as exc:
        source_map = {}
        errors.append(str(exc))

    spatial = study.get("spatial_design", {})
    try:
        acquisition = validate_bbox(
            spatial.get("acquisition_extent"), "spatial_design.acquisition_extent"
        )
    except ContractError as exc:
        acquisition = (-180.0, -90.0, 180.0, 90.0)
        errors.append(str(exc))

    site_ids: set = set()
    zone_ids: set = set()
    registered_sites = []
    zones = spatial.get("zones")
    check(
        isinstance(zones, list) and len(zones or []) >= 3,
        "At least three climate zones are required",
    )
    for zone_pos, zone in enumerate(zones or []):
        zone_id = zone.get("id") if isinstance(zone, dict) else None
        check(
            isinstance(zone_id, str) and bool(zone_id), f"zones[{zone_pos}] requires id"
        )
        if isinstance(zone_id, str):
            check(zone_id not in zone_ids, f"Duplicate zone id: {zone_id}")
            zone_ids.add(zone_id)
        try:
            zone_bbox = validate_bbox(zone.get("bbox"), f"zones[{zone_pos}].bbox")
        except (AttributeError, ContractError) as exc:
            errors.append(str(exc))
            continue
        sites = zone.get("sites", [])
        check(
            isinstance(sites, list) and bool(sites),
            f"Zone {zone_id} has no representative site",
        )
        for site_pos, site in enumerate(sites):
            site_id = site.get("id") if isinstance(site, dict) else None
            check(
                isinstance(site_id, str) and bool(site_id),
                f"Zone {zone_id} site {site_pos} requires id",
            )
            if isinstance(site_id, str):
                check(site_id not in site_ids, f"Duplicate site id: {site_id}")
                site_ids.add(site_id)
            lon = site.get("longitude") if isinstance(site, dict) else None
            lat = site.get("latitude") if isinstance(site, dict) else None
            check(
                isinstance(lon, (int, float)) and isinstance(lat, (int, float)),
                f"Site {site_id} needs numeric coordinates",
            )
            if isinstance(lon, (int, float)) and isinstance(lat, (int, float)):
                registered_sites.append((site_id, float(lon), float(lat)))
                check(
                    contains(zone_bbox, float(lon), float(lat)),
                    f"Site {site_id} lies outside its zone bbox",
                )
                check(
                    contains(acquisition, float(lon), float(lat)),
                    f"Site {site_id} lies outside acquisition extent",
                )

    try:
        bulk_extent = validate_bbox(
            spatial.get("bulk_nex_request_extent"),
            "spatial_design.bulk_nex_request_extent",
        )
        for site_id, lon, lat in registered_sites:
            check(
                contains(bulk_extent, lon, lat),
                f"Site {site_id} lies outside bulk NEX request extent",
            )
    except ContractError as exc:
        errors.append(str(exc))
    bulk_strategy = spatial.get("bulk_nex_request_strategy", {})
    check(
        isinstance(bulk_strategy.get("measured_content_length_bytes"), int)
        and bulk_strategy["measured_content_length_bytes"] > 0,
        "Bulk NEX strategy requires a positive measured_content_length_bytes",
    )

    periods = study.get("periods", {})
    parsed_periods: Dict[str, Tuple[date, date]] = {}
    for period_id, period in periods.items() if isinstance(periods, dict) else []:
        try:
            start = parse_iso_day(period.get("start"), f"periods.{period_id}.start")
            end = parse_iso_day(period.get("end"), f"periods.{period_id}.end")
            check(start <= end, f"Period {period_id} starts after it ends")
            parsed_periods[period_id] = (start, end)
        except (AttributeError, ContractError) as exc:
            errors.append(str(exc))
    check(
        "reference" in parsed_periods and "future_pilot" in parsed_periods,
        "Reference and future_pilot periods are required",
    )

    states = study.get("climate_states", {})
    allowed_ssps = {"ssp126", "ssp245", "ssp370", "ssp585"}
    pilot_ssps = states.get("pilot_ssps", [])
    check(
        isinstance(pilot_ssps, list) and bool(pilot_ssps),
        "pilot_ssps must be non-empty",
    )
    for ssp in pilot_ssps if isinstance(pilot_ssps, list) else []:
        check(ssp in allowed_ssps, f"Unsupported SSP id: {ssp}")
    gcms = states.get("pilot_gcms", [])
    check(
        isinstance(gcms, list) and len(gcms) >= 3,
        "The pilot must include at least three GCMs",
    )
    outer_gcms = states.get("outer_evaluation_gcms", [])
    check(
        isinstance(outer_gcms, list) and len(outer_gcms) >= 3,
        "The outer evaluation must include at least three GCMs",
    )
    if isinstance(gcms, list) and isinstance(outer_gcms, list):
        check(len(gcms) == len(set(gcms)), "pilot_gcms contains duplicates")
        check(
            len(outer_gcms) == len(set(outer_gcms)),
            "outer_evaluation_gcms contains duplicates",
        )
        overlap = sorted(set(gcms) & set(outer_gcms))
        check(not overlap, f"Pilot and outer GCM sets overlap: {overlap}")
        declared_models = gcms + outer_gcms
    else:
        declared_models = []
    realizations = states.get("gcm_realizations", {})
    check(
        isinstance(realizations, dict),
        "gcm_realizations must map each declared GCM to one realization",
    )
    if isinstance(realizations, dict):
        check(
            set(realizations) == set(declared_models),
            "gcm_realizations keys must exactly match pilot and outer GCMs",
        )
        for model, realization in realizations.items():
            check(
                isinstance(realization, str)
                and REALIZATION_PATTERN.fullmatch(realization) is not None,
                f"Invalid realization for {model}: {realization!r}",
            )
    calendar_overrides = states.get("nex_request_calendar_overrides", {})
    check(
        isinstance(calendar_overrides, dict),
        "nex_request_calendar_overrides must be an object",
    )
    if isinstance(calendar_overrides, dict):
        check(
            set(calendar_overrides).issubset(set(declared_models)),
            "Request calendar overrides contain an undeclared GCM",
        )
        for model, calendar_name in calendar_overrides.items():
            check(
                calendar_name in {"standard", "365_day", "noleap", "360_day"},
                f"Invalid request calendar for {model}: {calendar_name!r}",
            )
    roles = states.get("weather_generator_evaluation_roles", {})
    if isinstance(roles, dict):
        development = roles.get("method_development", [])
        confirmation = roles.get("frozen_confirmation", [])
        outer_role = roles.get("outer_evaluation", [])
        check(
            set(development) | set(confirmation) == set(gcms),
            "Development and confirmation roles must partition pilot_gcms",
        )
        check(
            not (set(development) & set(confirmation)),
            "Development and confirmation GCM roles must be disjoint",
        )
        check(
            outer_role == outer_gcms,
            "The outer evaluation role must preserve outer_evaluation_gcms order",
        )
    else:
        errors.append("weather_generator_evaluation_roles must be an object")
    check(
        states.get("selection_status") != "final",
        "Pilot GCM list must not be represented as final before skill screening",
    )

    variable_ids: set = set()
    variables = study.get("variables")
    check(
        isinstance(variables, list) and bool(variables), "variables must be non-empty"
    )
    for position, variable in enumerate(variables or []):
        variable_id = variable.get("id") if isinstance(variable, dict) else None
        check(
            isinstance(variable_id, str) and bool(variable_id),
            f"variables[{position}] requires id",
        )
        if not isinstance(variable_id, str):
            continue
        check(variable_id not in variable_ids, f"Duplicate variable id: {variable_id}")
        variable_ids.add(variable_id)
        check(
            bool(variable.get("target_units")),
            f"Variable {variable_id} lacks target_units",
        )
        check(
            bool(variable.get("target_frequency")),
            f"Variable {variable_id} lacks target_frequency",
        )
        if variable.get("derived_from"):
            for dependency in variable["derived_from"]:
                check(
                    any(v.get("id") == dependency for v in variables),
                    f"Variable {variable_id} has unknown dependency {dependency}",
                )
        elif variable.get("required"):
            check(
                variable.get("era5_land") is not None,
                f"Required variable {variable_id} lacks ERA5-Land mapping",
            )
            check(
                variable.get("nex_gddp_cmip6") is not None,
                f"Required variable {variable_id} lacks NEX-GDDP-CMIP6 mapping",
            )

    for required_source in ("era5_land", "nex_gddp_cmip6", "cma_station", "cn05_1"):
        check(
            required_source in source_map,
            f"Missing source definition: {required_source}",
        )
    for source_id, source in source_map.items():
        for field in (
            "title",
            "provider",
            "dataset_id",
            "access_method",
            "credential_requirement",
            "license_note",
            "version_policy",
            "pilot_status",
        ):
            check(field in source, f"Source {source_id} lacks {field}")
    nex_source = source_map.get("nex_gddp_cmip6", {})
    repair_keys = []
    repair_ids = []
    for position, repair in enumerate(nex_source.get("metadata_repairs", [])):
        prefix = f"NEX metadata repair {position}"
        check(isinstance(repair, dict), f"{prefix} must be an object")
        if not isinstance(repair, dict):
            continue
        repair_id = repair.get("id")
        model = repair.get("model")
        variable = repair.get("variable")
        effective_units = repair.get("effective_units")
        check(isinstance(repair_id, str) and bool(repair_id), f"{prefix} lacks id")
        check(model in declared_models, f"{prefix} has undeclared model {model!r}")
        variable_spec = nex_source.get("variables", {}).get(variable, {})
        check(bool(variable_spec), f"{prefix} has unknown variable {variable!r}")
        check(
            effective_units in variable_spec.get("accepted_units", []),
            f"{prefix} has unsupported effective units {effective_units!r}",
        )
        check(
            isinstance(repair.get("evidence"), list) and bool(repair.get("evidence")),
            f"{prefix} requires evidence",
        )
        repair_ids.append(repair_id)
        repair_keys.append(
            (model, variable, repair.get("match_declared_units"))
        )
    check(len(repair_ids) == len(set(repair_ids)), "Duplicate NEX metadata repair id")
    check(
        len(repair_keys) == len(set(repair_keys)),
        "Duplicate NEX metadata repair match key",
    )

    gates = study.get("quality_gates", {})
    missing_limit = gates.get("maximum_missing_fraction")
    check(
        isinstance(missing_limit, (int, float)) and 0 <= missing_limit < 1,
        "Invalid maximum_missing_fraction",
    )
    check(
        gates.get("require_sha256") is True, "SHA-256 provenance gate must be enabled"
    )
    check(
        gates.get("require_query_record") is True,
        "Query provenance gate must be enabled",
    )
    check(
        gates.get("require_expected_time_coverage") is True,
        "Expected time-coverage gate must be enabled",
    )

    semantics = study.get("probability_semantics", {})
    check(
        semantics.get("ssp_probabilities") is None,
        "SSP probabilities must remain unset in the pilot",
    )
    check(
        semantics.get("gcm_probabilities") is None,
        "GCM probabilities must remain unset in the pilot",
    )

    if spatial.get("mode") == "representative_points":
        warnings.append(
            "Representative-point pilot cannot support asset-scale exposure claims."
        )
    if (
        source_map.get("nex_gddp_cmip6", {}).get("pilot_status")
        != "bulk_catalog_coverage_verified"
    ):
        warnings.append(
            "NEX-GDDP-CMIP6 bulk model/experiment/variable coverage remains to be audited."
        )
    return {
        "valid": not errors,
        "errors": errors,
        "warnings": warnings,
        "counts": {
            "zones": len(zone_ids),
            "sites": len(site_ids),
            "variables": len(variable_ids),
            "pilot_gcms": len(gcms) if isinstance(gcms, list) else 0,
            "outer_evaluation_gcms": (
                len(outer_gcms) if isinstance(outer_gcms, list) else 0
            ),
            "all_gcms": len(declared_models),
            "pilot_ssps": len(pilot_ssps) if isinstance(pilot_ssps, list) else 0,
        },
    }


def iter_sites(study: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    for zone in study["spatial_design"]["zones"]:
        for site in zone["sites"]:
            result = dict(site)
            result["zone_id"] = zone["id"]
            yield result


def days_in_period(period: Dict[str, str]) -> int:
    return (
        parse_iso_day(period["end"], "period.end")
        - parse_iso_day(period["start"], "period.start")
    ).days + 1


def years_in_period(period: Dict[str, str]) -> range:
    start = parse_iso_day(period["start"], "period.start")
    end = parse_iso_day(period["end"], "period.end")
    return range(start.year, end.year + 1)


def expected_days_in_calendar_year(year: int, calendar_name: str) -> int:
    if calendar_name in {"365_day", "noleap"}:
        return 365
    if calendar_name == "360_day":
        return 360
    return 366 if calendar.isleap(year) else 365


def calendar_time_values_equal(left: Any, right: Any, np: Any) -> bool:
    left_values = np.asarray(left).reshape(-1)
    right_values = np.asarray(right).reshape(-1)
    if left_values.shape != right_values.shape:
        return False
    left_text = np.asarray(
        [f"{getattr(value, 'calendar', 'native')}|{value}" for value in left_values]
    )
    right_text = np.asarray(
        [f"{getattr(value, 'calendar', 'native')}|{value}" for value in right_values]
    )
    return bool(np.array_equal(left_text, right_text))


def clip_relative_humidity(values: Any, np: Any) -> Tuple[Any, int]:
    array = np.asarray(values, dtype=float)
    clipped_count = int(np.count_nonzero((array < 0.0) | (array > 100.0)))
    return np.clip(array, 0.0, 100.0), clipped_count


def required_source_variables(
    study: Dict[str, Any], source_id: str, include_optional: bool
) -> List[str]:
    values: List[str] = []
    for variable in study["variables"]:
        if not variable.get("required") and not include_optional:
            continue
        mapping = variable.get(source_id)
        if not isinstance(mapping, dict):
            continue
        candidates = mapping.get("inputs") or [mapping.get("variable")]
        for candidate in candidates:
            if candidate and candidate not in values:
                values.append(candidate)
    return values


def era5_timeseries_groups(
    study: Dict[str, Any], include_optional: bool
) -> Dict[str, List[str]]:
    required = set(required_source_variables(study, "era5_land", include_optional))
    groups = {
        group: [variable for variable in variables if variable in required]
        for group, variables in ERA5_TIMESERIES_GROUPS.items()
    }
    assigned = {variable for variables in groups.values() for variable in variables}
    unassigned = sorted(required - assigned)
    if unassigned:
        raise ContractError(
            "ERA5-Land time-series variables lack a declared storage group: "
            + ", ".join(unassigned)
        )
    return {group: variables for group, variables in groups.items() if variables}


def path_text(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO_ROOT.resolve()).as_posix()
    except ValueError:
        return path.resolve().as_posix()


def build_plan(
    study: Dict[str, Any],
    include_optional: bool,
    include_extension: bool,
    model_set: str = "pilot",
) -> Dict[str, Any]:
    data_root = REPO_ROOT / study["data_root"]
    sites = list(iter_sites(study))
    records: List[Dict[str, Any]] = []
    era_groups = era5_timeseries_groups(study, include_optional)
    reference = study["periods"]["reference"]
    era_values = 0
    for site in sites:
        period_id = f"{reference['start'][:4]}-{reference['end'][:4]}"
        for group, variables in era_groups.items():
            target = (
                data_root
                / "raw"
                / "era5_land_periods"
                / f"site={site['id']}"
                / f"period={period_id}"
                / f"era5_land_timeseries_{site['id']}_{period_id}_{group}.zip"
            )
            values = days_in_period(reference) * 24 * len(variables)
            era_values += values
            records.append(
                {
                    "source": "era5_land",
                    "dataset_id": ERA5_TIMESERIES_DATASET_ID,
                    "site": site["id"],
                    "zone": site["zone_id"],
                    "period": period_id,
                    "variable_group": group,
                    "variables": variables,
                    "target": path_text(target),
                    "estimated_values": values,
                    "status": "reference_period_point_timeseries_request_ready",
                }
            )

    nex_variables = required_source_variables(study, "nex_gddp_cmip6", include_optional)
    model_experiments: List[Tuple[str, Dict[str, str]]] = [("historical", reference)]
    for ssp in study["climate_states"]["pilot_ssps"]:
        model_experiments.append((ssp, study["periods"]["future_pilot"]))
    if include_extension:
        for ssp in study["climate_states"].get("extension_ssps", []):
            model_experiments.append((ssp, study["periods"]["future_extension"]))
    nex_values = 0
    selected_models = models_for_set(study, model_set)
    for model in selected_models:
        for experiment, period in model_experiments:
            period_id = f"{period['start'][:4]}-{period['end'][:4]}"
            for variable in nex_variables:
                target = (
                    data_root
                    / "raw"
                    / "nex_gddp_cmip6"
                    / f"model={model}"
                    / f"experiment={experiment}"
                    / f"variable={variable}"
                    / f"nex_gddp_cmip6_{model}_{experiment}_{variable}_{period_id}_sites.nc"
                )
                values = days_in_period(period) * len(sites)
                nex_values += values
                records.append(
                    {
                        "source": "nex_gddp_cmip6",
                        "dataset_id": "NEX-GDDP-CMIP6",
                        "model": model,
                        "experiment": experiment,
                        "period": period_id,
                        "variable": variable,
                        "sites": [site["id"] for site in sites],
                        "target": path_text(target),
                        "estimated_values": values,
                        "status": "catalog_verified_retrieval_pending",
                    }
                )
    total_values = era_values + nex_values
    nex_object_count = (
        len(selected_models)
        * len(model_experiments)
        * len(nex_variables)
        * len(list(years_in_period(reference)))
    )
    measured_bytes = study["spatial_design"]["bulk_nex_request_strategy"][
        "measured_content_length_bytes"
    ]
    return {
        "study_id": study["study_id"],
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": {
            "include_optional_variables": include_optional,
            "include_extension_period": include_extension,
            "model_set": model_set,
            "models": selected_models,
            "spatial_mode": study["spatial_design"]["mode"],
            "site_count": len(sites),
        },
        "estimate": {
            "era5_land_values": era_values,
            "nex_gddp_cmip6_values": nex_values,
            "total_values": total_values,
            "uncompressed_float32_mib": round(total_values * 4 / 1024 / 1024, 2),
            "planning_multiplier_note": "Allow 2-5x for coordinates, metadata, chunk overhead, and intermediate products.",
            "nex_corridor_annual_request_count": nex_object_count,
            "nex_equivalent_point_request_count": nex_object_count * len(sites),
            "nex_corridor_transfer_mib_from_single_object_extrapolation": round(
                nex_object_count * measured_bytes / 1024 / 1024, 2
            ),
        },
        "record_count": len(records),
        "records": records,
    }


def era5_timeseries_request(
    site: Dict[str, Any],
    year: int,
    variables: Sequence[str],
    end_year: Optional[int] = None,
) -> Dict[str, Any]:
    final_year = end_year if end_year is not None else year
    return {
        "variable": list(variables),
        "location": {
            "longitude": float(site["longitude"]),
            "latitude": float(site["latitude"]),
        },
        "date": [f"{year}-01-01/{final_year}-12-31"],
        "data_format": "netcdf",
    }


def extract_single_netcdf(archive: Path, target: Path) -> str:
    try:
        with zipfile.ZipFile(archive) as bundle:
            members = [
                member
                for member in bundle.infolist()
                if not member.is_dir() and member.filename.lower().endswith(".nc")
            ]
            if len(members) != 1:
                raise ContractError(
                    f"Expected exactly one NetCDF member in {archive}, found {len(members)}"
                )
            member = members[0]
            target.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                dir=target.parent,
                prefix=target.name + ".",
                suffix=".part",
                delete=False,
            ) as temporary:
                temporary_name = Path(temporary.name)
                with bundle.open(member) as source:
                    shutil.copyfileobj(source, temporary)
            temporary_name.replace(target)
            return member.filename
    except (OSError, zipfile.BadZipFile) as exc:
        raise ContractError(
            f"Invalid ERA5-Land response archive {archive}: {exc}"
        ) from exc


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def nex_catalog_url(
    model: str, experiment: str, realization: str, variable: str
) -> str:
    base = "https://ds.nccs.nasa.gov/thredds/catalog/AMES/NEX/GDDP-CMIP6"
    return f"{base}/{model}/{experiment}/{realization}/{variable}/catalog.xml"


def resolve_nex_dataset(catalog_xml: bytes, year: int, product_version: str) -> str:
    try:
        root = ET.fromstring(catalog_xml)
    except ET.ParseError as exc:
        raise ContractError(f"Invalid THREDDS catalog XML: {exc}") from exc
    suffix = f"_{year}_v{product_version}.nc"
    matches = []
    for element in root.iter():
        url_path = element.attrib.get("urlPath")
        if url_path and url_path.endswith(suffix):
            matches.append(url_path)
    if len(matches) != 1:
        raise ContractError(
            f"Expected one NEX dataset ending {suffix}, found {len(matches)}"
        )
    return matches[0]


def nex_point_request_url(
    url_path: str,
    variable: str,
    site: Dict[str, Any],
    year: int,
    calendar_name: str = "standard",
) -> str:
    base = "https://ds.nccs.nasa.gov/thredds/ncss/grid/"
    query = urllib.parse.urlencode(
        {
            "var": variable,
            "latitude": site["latitude"],
            "longitude": site["longitude"],
            "time_start": f"{year}-01-01T12:00:00Z",
            "time_end": (
                f"{year}-12-30T12:00:00Z"
                if calendar_name == "360_day"
                else f"{year}-12-31T12:00:00Z"
            ),
            "accept": "netcdf4",
            "addLatLon": "true",
        }
    )
    return base + urllib.parse.quote(url_path, safe="/-_.") + "?" + query


def nex_corridor_request_url(
    url_path: str,
    variable: str,
    bbox: Sequence[float],
    year: int,
    calendar_name: str = "standard",
) -> str:
    west, south, east, north = bbox
    base = "https://ds.nccs.nasa.gov/thredds/ncss/grid/"
    query = urllib.parse.urlencode(
        {
            "var": variable,
            "north": north,
            "west": west,
            "east": east,
            "south": south,
            "disableProjSubset": "on",
            "horizStride": 1,
            "time_start": f"{year}-01-01T12:00:00Z",
            "time_end": (
                f"{year}-12-30T12:00:00Z"
                if calendar_name == "360_day"
                else f"{year}-12-31T12:00:00Z"
            ),
            "timeStride": 1,
            "accept": "netcdf4",
            "addLatLon": "true",
        }
    )
    return base + urllib.parse.quote(url_path, safe="/-_.") + "?" + query


def nex_period_for_experiment(study: Dict[str, Any], experiment: str) -> Dict[str, str]:
    if experiment == "historical":
        return study["periods"]["reference"]
    if experiment in study["climate_states"]["pilot_ssps"]:
        return study["periods"]["future_pilot"]
    if experiment in study["climate_states"].get("extension_ssps", []):
        return study["periods"]["future_extension"]
    raise ContractError(
        f"Experiment {experiment} is outside the declared study contract"
    )


def command_validate(args: argparse.Namespace) -> int:
    report = validate_contract(load_json(args.study), load_json(args.sources))
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


def command_plan(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    report = validate_contract(study, load_json(args.sources))
    if not report["valid"]:
        raise ContractError("Contract is invalid; run validate for details")
    plan = build_plan(
        study, args.include_optional, args.include_extension, args.model_set
    )
    if args.output:
        write_json(args.output, plan, args.overwrite)
        print(f"Wrote {plan['record_count']} records to {args.output}")
    else:
        print(
            json.dumps(
                {k: v for k, v in plan.items() if k != "records"},
                indent=2,
                sort_keys=True,
            )
        )
    return 0


def command_era5_requests(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    report = validate_contract(study, load_json(args.sources))
    if not report["valid"]:
        raise ContractError("Contract is invalid; run validate for details")
    data_root = REPO_ROOT / study["data_root"]
    request_root = (
        args.output_dir or data_root / "provenance" / "requests" / "era5_land"
    )
    client = None
    if args.execute:
        try:
            import cdsapi  # type: ignore
        except ImportError as exc:
            raise ContractError(
                "--execute requires cdsapi; install it in a dedicated climate-data environment"
            ) from exc
        client = cdsapi.Client()
    count = 0
    retrieved = 0
    groups = era5_timeseries_groups(study, args.include_optional)
    if args.group:
        if args.group not in groups:
            raise ContractError(
                f"Variable group {args.group} is not available for this request scope"
            )
        groups = {args.group: groups[args.group]}
    for site in iter_sites(study):
        for year in years_in_period(study["periods"]["reference"]):
            if args.year and year != args.year:
                continue
            if args.site and site["id"] != args.site:
                continue
            for group, variables in groups.items():
                request = era5_timeseries_request(site, year, variables)
                archive_target = (
                    data_root
                    / "raw"
                    / "era5_land"
                    / f"site={site['id']}"
                    / f"year={year}"
                    / f"era5_land_timeseries_{site['id']}_{year}_{group}.zip"
                )
                netcdf_target = (
                    archive_target.parent
                    / f"era5_land_timeseries_{site['id']}_{year}_{group}.nc"
                )
                record = {
                    "schema_version": "1.0",
                    "study_id": study["study_id"],
                    "source": "era5_land",
                    "dataset_id": ERA5_TIMESERIES_DATASET_ID,
                    "site": site,
                    "year": year,
                    "variable_group": group,
                    "query": request,
                    "archive_target": path_text(archive_target),
                    "netcdf_target": path_text(netcdf_target),
                    "request_created_utc": datetime.now(timezone.utc).isoformat(),
                    "retrieval_status": "planned",
                }
                request_path = (
                    request_root
                    / f"site={site['id']}"
                    / f"year={year}"
                    / f"era5_land_timeseries_{site['id']}_{year}_{group}.request.json"
                )
                if args.execute:
                    if archive_target.exists() or netcdf_target.exists():
                        raise ContractError(
                            "Raw targets are immutable; choose a new versioned path: "
                            f"{archive_target} or {netcdf_target}"
                        )
                    archive_target.parent.mkdir(parents=True, exist_ok=True)
                    temporary_name: Optional[str] = None
                    try:
                        with tempfile.NamedTemporaryFile(
                            dir=archive_target.parent,
                            prefix=archive_target.name + ".",
                            suffix=".part",
                            delete=False,
                        ) as temporary:
                            temporary_name = temporary.name
                        client.retrieve(
                            ERA5_TIMESERIES_DATASET_ID, request, temporary_name
                        )
                        Path(temporary_name).replace(archive_target)
                        archive_member = extract_single_netcdf(
                            archive_target, netcdf_target
                        )
                    except Exception as exc:
                        raise ContractError(
                            f"ERA5-Land retrieval failed for {site['id']} {year} {group}: {exc}"
                        ) from exc
                    finally:
                        if temporary_name and Path(temporary_name).exists():
                            Path(temporary_name).unlink()
                    record["retrieval_status"] = "retrieved_unvalidated"
                    record["retrieved_utc"] = datetime.now(timezone.utc).isoformat()
                    record["archive_member"] = archive_member
                    record["archive_bytes"] = archive_target.stat().st_size
                    record["archive_sha256"] = sha256_file(archive_target)
                    record["netcdf_bytes"] = netcdf_target.stat().st_size
                    record["netcdf_sha256"] = sha256_file(netcdf_target)
                    retrieved += 1
                write_json(request_path, record, args.overwrite)
                count += 1
    print(
        json.dumps(
            {
                "request_records": count,
                "retrieved": retrieved,
                "request_root": path_text(request_root),
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def split_era5_period_netcdf(
    study: Dict[str, Any],
    site: Dict[str, Any],
    group: str,
    period_file: Path,
    start_year: int,
    end_year: int,
) -> List[Dict[str, Any]]:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "ERA5-Land period partitioning requires xarray, numpy, and a NetCDF backend"
        ) from exc
    period_sha256 = sha256_file(period_file)
    data_root = REPO_ROOT / study["data_root"]
    staged: List[Tuple[Path, Path, Dict[str, Any]]] = []
    records: List[Dict[str, Any]] = []
    try:
        with xr.open_dataset(period_file) as dataset:
            time_name = coordinate_name(dataset, ("time", "valid_time"))
            if not time_name:
                raise ContractError(f"{period_file} lacks a time coordinate")
            actual_time = np.asarray(dataset[time_name].values).astype("datetime64[ns]")
            expected_time = np.arange(
                np.datetime64(f"{start_year}-01-01T00:00"),
                np.datetime64(f"{end_year + 1}-01-01T00:00"),
                np.timedelta64(1, "h"),
            ).astype("datetime64[ns]")
            if not np.array_equal(actual_time, expected_time):
                raise ContractError(
                    f"{period_file} is not the complete ordered {start_year}-{end_year} UTC hourly axis"
                )
            for year in range(start_year, end_year + 1):
                target = (
                    data_root
                    / "raw"
                    / "era5_land"
                    / f"site={site['id']}"
                    / f"year={year}"
                    / f"era5_land_timeseries_{site['id']}_{year}_{group}.nc"
                )
                year_data = dataset.sel(
                    {time_name: slice(f"{year}-01-01", f"{year}-12-31T23:00:00")}
                ).load()
                expected_hours = (366 if calendar.isleap(year) else 365) * 24
                if int(year_data.sizes[time_name]) != expected_hours:
                    raise ContractError(
                        f"Annual partition {site['id']} {year} {group} has "
                        f"{year_data.sizes[time_name]} hours, expected {expected_hours}"
                    )
                if target.exists():
                    with xr.open_dataset(target) as existing:
                        existing_time_name = coordinate_name(
                            existing, ("time", "valid_time")
                        )
                        equal = bool(existing_time_name) and set(
                            existing.data_vars
                        ) == set(year_data.data_vars)
                        if equal:
                            equal = np.array_equal(
                                np.asarray(existing[existing_time_name].values).astype(
                                    "datetime64[ns]"
                                ),
                                np.asarray(year_data[time_name].values).astype(
                                    "datetime64[ns]"
                                ),
                            )
                        if equal:
                            for variable in year_data.data_vars:
                                if not np.array_equal(
                                    np.asarray(existing[variable].values),
                                    np.asarray(year_data[variable].values),
                                    equal_nan=True,
                                ):
                                    equal = False
                                    break
                        if not equal:
                            raise ContractError(
                                f"Existing annual raw file differs from period response: {target}"
                            )
                    records.append(
                        {
                            "year": year,
                            "target": path_text(target),
                            "status": "existing_annual_file_verified_equal",
                            "sha256": sha256_file(target),
                        }
                    )
                    year_data.close()
                    continue
                target.parent.mkdir(parents=True, exist_ok=True)
                year_data.attrs.update(
                    {
                        "source_period_file": path_text(period_file),
                        "source_period_sha256": period_sha256,
                        "temporal_partition_year": year,
                        "partition_operation": "lossless_hourly_time_slice_no_value_transformation",
                    }
                )
                with tempfile.NamedTemporaryFile(
                    dir=target.parent,
                    prefix=target.name + ".",
                    suffix=".part",
                    delete=False,
                ) as temporary:
                    temporary_path = Path(temporary.name)
                year_data.to_netcdf(temporary_path, engine="netcdf4")
                year_data.close()
                record = {
                    "year": year,
                    "target": path_text(target),
                    "status": "staged",
                }
                staged.append((temporary_path, target, record))
        for temporary_path, target, record in staged:
            temporary_path.replace(target)
            record.update(
                {
                    "status": "partitioned",
                    "bytes": target.stat().st_size,
                    "sha256": sha256_file(target),
                }
            )
            records.append(record)
    finally:
        for temporary_path, _, _ in staged:
            if temporary_path.exists():
                temporary_path.unlink()
    return sorted(records, key=lambda record: int(record["year"]))


def command_era5_period(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    report = validate_contract(study, load_json(args.sources))
    if not report["valid"]:
        raise ContractError("Contract is invalid; run validate for details")
    reference_years = set(years_in_period(study["periods"]["reference"]))
    if args.start_year > args.end_year:
        raise ContractError("--start-year must not exceed --end-year")
    if not set(range(args.start_year, args.end_year + 1)).issubset(reference_years):
        raise ContractError(
            "Requested ERA5-Land period is outside the reference period"
        )
    sites = {site["id"]: site for site in iter_sites(study)}
    site = sites[args.site]
    groups = era5_timeseries_groups(study, include_optional=False)
    variables = groups[args.group]
    period_id = f"{args.start_year}-{args.end_year}"
    data_root = REPO_ROOT / study["data_root"]
    target_root = (
        data_root
        / "raw"
        / "era5_land_periods"
        / f"site={args.site}"
        / f"period={period_id}"
    )
    stem = f"era5_land_timeseries_{args.site}_{period_id}_{args.group}"
    archive_target = target_root / f"{stem}.zip"
    netcdf_target = target_root / f"{stem}.nc"
    request = era5_timeseries_request(
        site, args.start_year, variables, end_year=args.end_year
    )
    record: Dict[str, Any] = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "source": "era5_land",
        "dataset_id": ERA5_TIMESERIES_DATASET_ID,
        "site": site,
        "period": period_id,
        "variable_group": args.group,
        "query": request,
        "archive_target": path_text(archive_target),
        "netcdf_target": path_text(netcdf_target),
        "request_created_utc": datetime.now(timezone.utc).isoformat(),
        "retrieval_status": "planned",
    }
    request_path = (
        data_root
        / "provenance"
        / "requests"
        / "era5_land_periods"
        / f"site={args.site}"
        / f"period={period_id}"
        / f"{stem}.request.json"
    )
    if args.execute:
        try:
            import cdsapi  # type: ignore
        except ImportError as exc:
            raise ContractError(
                "--execute requires cdsapi; install it in a dedicated climate-data environment"
            ) from exc
        if archive_target.exists() or netcdf_target.exists():
            raise ContractError(
                "Period raw targets are immutable; existing target: "
                f"{archive_target if archive_target.exists() else netcdf_target}"
            )
        target_root.mkdir(parents=True, exist_ok=True)
        temporary_name: Optional[str] = None
        try:
            with tempfile.NamedTemporaryFile(
                dir=target_root,
                prefix=archive_target.name + ".",
                suffix=".part",
                delete=False,
            ) as temporary:
                temporary_name = temporary.name
            cdsapi.Client().retrieve(
                ERA5_TIMESERIES_DATASET_ID, request, temporary_name
            )
            Path(temporary_name).replace(archive_target)
            archive_member = extract_single_netcdf(archive_target, netcdf_target)
            annual_partitions = split_era5_period_netcdf(
                study,
                site,
                args.group,
                netcdf_target,
                args.start_year,
                args.end_year,
            )
        except Exception as exc:
            raise ContractError(
                f"ERA5-Land period retrieval failed for {args.site} {period_id} "
                f"{args.group}: {exc}"
            ) from exc
        finally:
            if temporary_name and Path(temporary_name).exists():
                Path(temporary_name).unlink()
        record.update(
            {
                "retrieval_status": "retrieved_partitioned_unvalidated",
                "retrieved_utc": datetime.now(timezone.utc).isoformat(),
                "archive_member": archive_member,
                "archive_bytes": archive_target.stat().st_size,
                "archive_sha256": sha256_file(archive_target),
                "netcdf_bytes": netcdf_target.stat().st_size,
                "netcdf_sha256": sha256_file(netcdf_target),
                "annual_partitions": annual_partitions,
            }
        )
    write_json(request_path, record, args.overwrite)
    if not getattr(args, "quiet", False):
        print(json.dumps(record, indent=2, sort_keys=True))
    return 0


def command_era5_site_period(args: argparse.Namespace) -> int:
    completed = []
    for group in ("temperature_humidity", "precipitation", "solar_radiation", "wind"):
        child = argparse.Namespace(
            study=args.study,
            sources=args.sources,
            site=args.site,
            group=group,
            start_year=args.start_year,
            end_year=args.end_year,
            execute=args.execute,
            overwrite=args.overwrite,
            quiet=True,
        )
        command_era5_period(child)
        completed.append(group)
    summary = {
        "site": args.site,
        "start_year": args.start_year,
        "end_year": args.end_year,
        "groups_completed": completed,
        "executed": args.execute,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def command_nex_request(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    report = validate_contract(study, load_json(args.sources))
    if not report["valid"]:
        raise ContractError("Contract is invalid; run validate for details")
    require_model_in_set(study, args.model, getattr(args, "model_set", "all"))
    variables = required_source_variables(
        study, "nex_gddp_cmip6", include_optional=False
    )
    if args.variable not in variables:
        raise ContractError(f"Variable {args.variable} is outside the required NEX set")
    period = nex_period_for_experiment(study, args.experiment)
    if args.year not in years_in_period(period):
        raise ContractError(
            f"Year {args.year} is outside the declared period for {args.experiment}"
        )
    sites = {site["id"]: site for site in iter_sites(study)}
    site = sites[args.site]
    source = sources["nex_gddp_cmip6"]
    realization = nex_realization(study, args.model)
    version = source["pilot_product_version"]
    catalog_url = nex_catalog_url(
        args.model, args.experiment, realization, args.variable
    )
    url_path = getattr(args, "resolved_url_path", None)
    if url_path is None:
        try:
            with urllib.request.urlopen(catalog_url, timeout=args.timeout) as response:
                catalog_xml = response.read()
        except OSError as exc:
            raise ContractError(
                f"Cannot read NEX catalog {catalog_url}: {exc}"
            ) from exc
        url_path = resolve_nex_dataset(catalog_xml, args.year, version)
    request_calendar = nex_request_calendar(study, args.model)
    request_url = nex_point_request_url(
        url_path, args.variable, site, args.year, request_calendar
    )
    data_root = REPO_ROOT / study["data_root"]
    target_name = f"nex_gddp_cmip6_{args.model}_{args.experiment}_{args.variable}_{args.site}_{args.year}_v{version}.nc"
    target = (
        data_root
        / "raw"
        / "nex_gddp_cmip6"
        / f"model={args.model}"
        / f"experiment={args.experiment}"
        / f"variable={args.variable}"
        / f"site={args.site}"
        / f"year={args.year}"
        / target_name
    )
    record_path = (
        data_root
        / "provenance"
        / "requests"
        / "nex_gddp_cmip6"
        / f"model={args.model}"
        / f"experiment={args.experiment}"
        / f"variable={args.variable}"
        / f"site={args.site}"
        / f"{Path(target_name).stem}.request.json"
    )
    record: Dict[str, Any] = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "source": "nex_gddp_cmip6",
        "dataset_id": source["dataset_id"],
        "product_version": version,
        "model": args.model,
        "experiment": args.experiment,
        "realization": realization,
        "request_calendar": request_calendar,
        "variable": args.variable,
        "year": args.year,
        "site": site,
        "catalog_url": catalog_url,
        "source_url_path": url_path,
        "query_url": request_url,
        "target": path_text(target),
        "request_created_utc": datetime.now(timezone.utc).isoformat(),
        "retrieval_status": "planned",
    }
    if args.execute:
        if target.exists():
            raise ContractError(
                f"Raw targets are immutable; choose a new versioned path: {target}"
            )
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary_name: Optional[str] = None
        try:
            with urllib.request.urlopen(request_url, timeout=args.timeout) as response:
                content_type = response.headers.get_content_type()
                if content_type not in {
                    "application/x-netcdf",
                    "application/netcdf",
                    "application/octet-stream",
                }:
                    raise ContractError(
                        f"NEX endpoint returned unexpected content type: {content_type}"
                    )
                with tempfile.NamedTemporaryFile(
                    dir=target.parent,
                    prefix=target.name + ".",
                    suffix=".part",
                    delete=False,
                ) as temporary:
                    temporary_name = temporary.name
                    shutil.copyfileobj(response, temporary)
            Path(temporary_name).replace(target)
        finally:
            if temporary_name and Path(temporary_name).exists():
                Path(temporary_name).unlink()
        record["retrieval_status"] = "retrieved_unvalidated"
        record["retrieved_utc"] = datetime.now(timezone.utc).isoformat()
        record["bytes"] = target.stat().st_size
        record["sha256"] = sha256_file(target)
    write_json(record_path, record, args.overwrite)
    if not getattr(args, "quiet", False):
        print(json.dumps(record, indent=2, sort_keys=True))
    return 0


def command_nex_bundle(args: argparse.Namespace) -> int:
    completed = []
    for variable in args.variables:
        child = argparse.Namespace(
            study=args.study,
            sources=args.sources,
            model=args.model,
            experiment=args.experiment,
            variable=variable,
            site=args.site,
            year=args.year,
            execute=args.execute,
            overwrite=args.overwrite,
            timeout=args.timeout,
            model_set=args.model_set,
        )
        command_nex_request(child)
        completed.append(variable)
    print(
        json.dumps(
            {
                "model": args.model,
                "experiment": args.experiment,
                "site": args.site,
                "year": args.year,
                "variables_completed": completed,
                "executed": args.execute,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def nex_site_raw_target(
    data_root: Path,
    model: str,
    experiment: str,
    variable: str,
    site: str,
    year: int,
    version: str,
) -> Path:
    name = f"nex_gddp_cmip6_{model}_{experiment}_{variable}_{site}_{year}_v{version}.nc"
    return (
        data_root
        / "raw"
        / "nex_gddp_cmip6"
        / f"model={model}"
        / f"experiment={experiment}"
        / f"variable={variable}"
        / f"site={site}"
        / f"year={year}"
        / name
    )


def extract_nex_corridor_sites(
    corridor: Path,
    study: Dict[str, Any],
    source: Dict[str, Any],
    model: str,
    experiment: str,
    variable: str,
    year: int,
) -> List[Dict[str, Any]]:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "NEX corridor extraction requires xarray, numpy, and a NetCDF backend"
        ) from exc
    version = source["pilot_product_version"]
    data_root = REPO_ROOT / study["data_root"]
    outputs = []
    with xr.open_dataset(corridor) as dataset:
        if variable not in dataset.data_vars:
            raise ContractError(f"{corridor} does not contain {variable}")
        latitude_name = coordinate_name(dataset, ("latitude", "lat"))
        longitude_name = coordinate_name(dataset, ("longitude", "lon"))
        if not latitude_name or not longitude_name or "time" not in dataset.coords:
            raise ContractError(f"{corridor} lacks time/latitude/longitude coordinates")
        time_values = np.asarray(dataset["time"].values)
        calendar_name = dataset["time"].encoding.get("calendar") or dataset[
            "time"
        ].attrs.get("calendar", "standard")
        expected_days = expected_days_in_calendar_year(year, str(calendar_name))
        expected_last_day = (
            f"{year}-12-30" if calendar_name == "360_day" else f"{year}-12-31"
        )
        if (
            time_values.size != expected_days
            or str(time_values[0])[:10] != f"{year}-01-01"
            or str(time_values[-1])[:10] != expected_last_day
        ):
            raise ContractError(
                f"{corridor} does not contain the complete {year} {calendar_name} daily axis"
            )
        array = dataset[variable]
        declared_units = str(array.attrs.get("units", ""))
        variable_spec = source["variables"][variable]
        units, metadata_repair = resolve_nex_native_units(
            source, model, variable, declared_units
        )
        if units not in variable_spec["accepted_units"]:
            raise ContractError(
                f"{corridor} has unsupported {variable} units {declared_units!r}"
            )
        values = np.asarray(array.values, dtype=float)
        if values.ndim != 3 or "time" not in array.dims:
            raise ContractError(
                f"{corridor} {variable} must be a three-dimensional time/latitude/longitude field"
            )
        missing_fraction = float(np.isnan(values).mean())
        if missing_fraction > study["quality_gates"]["maximum_missing_fraction"]:
            raise ContractError(
                f"{corridor} missing fraction {missing_fraction:.6g} exceeds gate"
            )
        lower, upper = (float(value) for value in variable_spec["physical_range"])
        tolerance = float(variable_spec.get("physical_range_tolerance", 0.0))
        minimum = float(np.nanmin(values))
        maximum = float(np.nanmax(values))
        if minimum < lower - tolerance or maximum > upper + tolerance:
            raise ContractError(
                f"{corridor} range [{minimum:.6g}, {maximum:.6g}] exceeds "
                f"[{lower}, {upper}] with tolerance {tolerance}"
            )
        for site in iter_sites(study):
            selected = array.sel(
                {
                    latitude_name: float(site["latitude"]),
                    longitude_name: float(site["longitude"]),
                },
                method="nearest",
            )
            selected_values = np.asarray(selected.values, dtype=float).reshape(-1)
            if selected_values.size != expected_days:
                raise ContractError(
                    f"Corridor selection for {site['id']} has {selected_values.size} days"
                )
            selected_latitude = float(np.asarray(selected[latitude_name].values).item())
            selected_longitude = normalize_longitude(
                float(np.asarray(selected[longitude_name].values).item())
            )
            target = nex_site_raw_target(
                data_root,
                model,
                experiment,
                variable,
                site["id"],
                year,
                version,
            )
            disposition = "created"
            if target.exists():
                with xr.open_dataset(target) as existing:
                    if variable not in existing or "time" not in existing.coords:
                        raise ContractError(
                            f"Existing immutable NEX point target is incompatible: {target}"
                        )
                    existing_values = np.asarray(
                        existing[variable].values, dtype=float
                    ).reshape(-1)
                    same = np.array_equal(
                        existing_values, selected_values
                    ) and calendar_time_values_equal(
                        existing["time"].values, time_values, np
                    )
                    existing_latitude_name = coordinate_name(
                        existing, ("latitude", "lat")
                    )
                    existing_longitude_name = coordinate_name(
                        existing, ("longitude", "lon")
                    )
                    if existing_latitude_name and existing_longitude_name:
                        same = (
                            same
                            and math.isclose(
                                float(
                                    np.asarray(
                                        existing[existing_latitude_name].values
                                    ).reshape(-1)[0]
                                ),
                                selected_latitude,
                                abs_tol=1e-9,
                            )
                            and math.isclose(
                                normalize_longitude(
                                    float(
                                        np.asarray(
                                            existing[existing_longitude_name].values
                                        ).reshape(-1)[0]
                                    )
                                ),
                                selected_longitude,
                                abs_tol=1e-9,
                            )
                        )
                    else:
                        same = False
                if not same:
                    raise ContractError(
                        f"Existing immutable NEX point target differs from corridor extraction: {target}"
                    )
                disposition = "verified_reused"
            else:
                point_variable_attrs = dict(array.attrs)
                if metadata_repair is not None:
                    point_variable_attrs.update(
                        {
                            "units": units,
                            "source_declared_units": declared_units,
                            "metadata_repair_id": metadata_repair["id"],
                        }
                    )
                point = xr.Dataset(
                    data_vars={
                        variable: (
                            "time",
                            selected_values,
                            point_variable_attrs,
                        )
                    },
                    coords={
                        "time": time_values,
                        "latitude": selected_latitude,
                        "longitude": selected_longitude,
                        "site": site["id"],
                    },
                    attrs={
                        "source_dataset": "NEX-GDDP-CMIP6",
                        "source_product_version": version,
                        "source_calendar": str(calendar_name),
                        "model": model,
                        "experiment": experiment,
                        "realization": nex_realization(study, model),
                        "extraction_method": "nearest_grid_point_from_annual_corridor",
                        "source_corridor_sha256": sha256_file(corridor),
                    },
                )
                target.parent.mkdir(parents=True, exist_ok=True)
                temporary_name: Optional[str] = None
                try:
                    with tempfile.NamedTemporaryFile(
                        dir=target.parent,
                        prefix=target.name + ".",
                        suffix=".part",
                        delete=False,
                    ) as temporary:
                        temporary_name = temporary.name
                    point.to_netcdf(
                        temporary_name,
                        encoding={variable: {"zlib": True, "complevel": 4}},
                    )
                    Path(temporary_name).replace(target)
                finally:
                    point.close()
                    if temporary_name and Path(temporary_name).exists():
                        Path(temporary_name).unlink()
            outputs.append(
                {
                    "site": site["id"],
                    "selected_grid_point": {
                        "latitude": selected_latitude,
                        "longitude": selected_longitude,
                    },
                    "target": path_text(target),
                    "bytes": target.stat().st_size,
                    "sha256": sha256_file(target),
                    "disposition": disposition,
                    "unit_resolution": {
                        "source_declared_units": declared_units,
                        "effective_units": units,
                        "metadata_repair_id": (
                            metadata_repair["id"] if metadata_repair else None
                        ),
                    },
                }
            )
    return outputs


def command_nex_corridor(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    report = validate_contract(study, load_json(args.sources))
    if not report["valid"]:
        raise ContractError("Contract is invalid; run validate for details")
    require_model_in_set(study, args.model, getattr(args, "model_set", "all"))
    variables = required_source_variables(
        study, "nex_gddp_cmip6", include_optional=False
    )
    if args.variable not in variables:
        raise ContractError(f"Variable {args.variable} is outside the required NEX set")
    period = nex_period_for_experiment(study, args.experiment)
    if args.year not in years_in_period(period):
        raise ContractError(
            f"Year {args.year} is outside the declared period for {args.experiment}"
        )
    source = sources["nex_gddp_cmip6"]
    version = source["pilot_product_version"]
    realization = nex_realization(study, args.model)
    catalog_url = nex_catalog_url(
        args.model, args.experiment, realization, args.variable
    )
    url_path = getattr(args, "resolved_url_path", None)
    if url_path is None:
        try:
            with urllib.request.urlopen(catalog_url, timeout=args.timeout) as response:
                catalog_xml = response.read()
        except OSError as exc:
            raise ContractError(
                f"Cannot read NEX catalog {catalog_url}: {exc}"
            ) from exc
        url_path = resolve_nex_dataset(catalog_xml, args.year, version)
    bbox = validate_bbox(
        study["spatial_design"]["bulk_nex_request_extent"],
        "spatial_design.bulk_nex_request_extent",
    )
    request_calendar = nex_request_calendar(study, args.model)
    request_url = nex_corridor_request_url(
        url_path, args.variable, bbox, args.year, request_calendar
    )
    data_root = REPO_ROOT / study["data_root"]
    target_name = (
        f"nex_gddp_cmip6_{args.model}_{args.experiment}_{args.variable}_corridor_"
        f"{args.year}_v{version}.nc"
    )
    target = (
        data_root
        / "raw"
        / "nex_gddp_cmip6"
        / f"model={args.model}"
        / f"experiment={args.experiment}"
        / f"variable={args.variable}"
        / "scope=corridor"
        / f"year={args.year}"
        / target_name
    )
    record_path = (
        data_root
        / "provenance"
        / "requests"
        / "nex_gddp_cmip6"
        / f"model={args.model}"
        / f"experiment={args.experiment}"
        / f"variable={args.variable}"
        / "scope=corridor"
        / f"{Path(target_name).stem}.request.json"
    )
    record: Dict[str, Any] = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "source": "nex_gddp_cmip6",
        "dataset_id": source["dataset_id"],
        "product_version": version,
        "model": args.model,
        "experiment": args.experiment,
        "realization": realization,
        "request_calendar": request_calendar,
        "variable": args.variable,
        "year": args.year,
        "bbox_west_south_east_north": list(bbox),
        "catalog_url": catalog_url,
        "source_url_path": url_path,
        "query_url": request_url,
        "target": path_text(target),
        "request_created_utc": datetime.now(timezone.utc).isoformat(),
        "retrieval_status": "planned",
    }
    if args.execute:
        if target.exists():
            record["retrieval_status"] = "existing_raw_validated_and_reused"
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary_name: Optional[str] = None
            try:
                with urllib.request.urlopen(
                    request_url, timeout=args.timeout
                ) as response:
                    content_type = response.headers.get_content_type()
                    if content_type not in {
                        "application/x-netcdf",
                        "application/netcdf",
                        "application/octet-stream",
                    }:
                        raise ContractError(
                            f"NEX endpoint returned unexpected content type: {content_type}"
                        )
                    with tempfile.NamedTemporaryFile(
                        dir=target.parent,
                        prefix=target.name + ".",
                        suffix=".part",
                        delete=False,
                    ) as temporary:
                        temporary_name = temporary.name
                        shutil.copyfileobj(response, temporary)
                Path(temporary_name).replace(target)
            finally:
                if temporary_name and Path(temporary_name).exists():
                    Path(temporary_name).unlink()
            record["retrieval_status"] = "retrieved_and_validated"
            record["retrieved_utc"] = datetime.now(timezone.utc).isoformat()
        record["bytes"] = target.stat().st_size
        record["sha256"] = sha256_file(target)
        record["site_extractions"] = extract_nex_corridor_sites(
            target,
            study,
            source,
            args.model,
            args.experiment,
            args.variable,
            args.year,
        )
    write_json(record_path, record, args.overwrite)
    if not getattr(args, "quiet", False):
        print(json.dumps(record, indent=2, sort_keys=True))
    return 0


def resolve_nex_catalog_group(
    task: Tuple[str, str, str, List[int], str, str, int],
) -> Dict[str, Any]:
    model, experiment, variable, years, realization, version, timeout = task
    catalog_url = nex_catalog_url(model, experiment, realization, variable)
    result: Dict[str, Any] = {
        "model": model,
        "experiment": experiment,
        "variable": variable,
        "catalog_url": catalog_url,
    }
    try:
        with urllib.request.urlopen(catalog_url, timeout=timeout) as response:
            catalog_xml = response.read()
        result["url_paths"] = {
            year: resolve_nex_dataset(catalog_xml, year, version) for year in years
        }
        result["valid"] = True
    except (OSError, ContractError, TypeError, ValueError) as exc:
        result.update({"valid": False, "error": str(exc), "years": years})
    return result


def execute_nex_annual_task(
    task: Tuple[Path, Path, str, str, str, int, str, int],
) -> Dict[str, Any]:
    study_path, sources_path, model, experiment, variable, year, url_path, timeout = (
        task
    )
    child = argparse.Namespace(
        study=study_path,
        sources=sources_path,
        model=model,
        experiment=experiment,
        variable=variable,
        year=year,
        execute=True,
        overwrite=True,
        timeout=timeout,
        quiet=True,
        resolved_url_path=url_path,
    )
    try:
        command_nex_corridor(child)
        return {
            "valid": True,
            "model": model,
            "experiment": experiment,
            "variable": variable,
            "year": year,
        }
    except (OSError, ContractError, TypeError, ValueError) as exc:
        return {
            "valid": False,
            "model": model,
            "experiment": experiment,
            "variable": variable,
            "year": year,
            "error": str(exc),
        }


def command_nex_pilot_download(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    if args.workers < 1:
        raise ContractError("nex-pilot-download workers must be at least one")
    report = validate_contract(study, load_json(args.sources))
    if not report["valid"]:
        raise ContractError("Contract is invalid; run validate for details")
    source = sources["nex_gddp_cmip6"]
    selected_models = models_for_set(study, args.model_set)
    if args.model:
        require_model_in_set(study, args.model, args.model_set)
    models = [args.model] if args.model else selected_models
    experiments = (
        [args.experiment]
        if args.experiment
        else ["historical", *study["climate_states"]["pilot_ssps"]]
    )
    variables = (
        [args.variable]
        if args.variable
        else required_source_variables(study, "nex_gddp_cmip6", include_optional=False)
    )
    groups = []
    object_count = 0
    for model in models:
        for experiment in experiments:
            years = list(years_in_period(nex_period_for_experiment(study, experiment)))
            if args.start_year is not None:
                years = [year for year in years if year >= args.start_year]
            if args.end_year is not None:
                years = [year for year in years if year <= args.end_year]
            if not years:
                continue
            for variable in variables:
                groups.append(
                    (
                        model,
                        experiment,
                        variable,
                        years,
                        nex_realization(study, model),
                        source["pilot_product_version"],
                        args.timeout,
                    )
                )
                object_count += len(years)
    if not groups:
        raise ContractError("The requested NEX filters select no declared pilot years")
    summary: Dict[str, Any] = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "operation": "nex_gddp_cmip6_resumable_corridor_pilot_download",
        "selection": {
            "models": models,
            "experiments": experiments,
            "variables": variables,
            "start_year": args.start_year,
            "end_year": args.end_year,
        },
        "catalog_count": len(groups),
        "annual_corridor_object_count": object_count,
        "workers": args.workers,
        "executed": args.execute,
    }
    if not args.execute:
        summary["status"] = "planned_not_executed"
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0
    resolved_groups = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
        for resolved in executor.map(resolve_nex_catalog_group, groups):
            resolved_groups.append(resolved)
            if args.progress:
                state = "OK" if resolved["valid"] else "FAIL"
                print(
                    f"CATALOG {state} {resolved['model']} {resolved['experiment']} {resolved['variable']}",
                    flush=True,
                )
    catalog_failures = [group for group in resolved_groups if not group["valid"]]
    annual_tasks = []
    for group in resolved_groups:
        if not group["valid"]:
            continue
        for year, url_path in group["url_paths"].items():
            annual_tasks.append(
                (
                    args.study,
                    args.sources,
                    group["model"],
                    group["experiment"],
                    group["variable"],
                    int(year),
                    url_path,
                    args.timeout,
                )
            )

    annual_results = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.workers) as executor:
        for result in executor.map(execute_nex_annual_task, annual_tasks):
            annual_results.append(result)
            if args.progress:
                state = "OK" if result["valid"] else "FAIL"
                print(
                    f"OBJECT {state} {result['model']} {result['experiment']} "
                    f"{result['variable']} {result['year']}",
                    flush=True,
                )
    annual_failures = [result for result in annual_results if not result["valid"]]
    summary.update(
        {
            "status": "complete"
            if not catalog_failures and not annual_failures
            else "incomplete",
            "catalog_failures": catalog_failures,
            "annual_objects_completed": len(annual_results) - len(annual_failures),
            "annual_object_failures": annual_failures,
            "finished_utc": datetime.now(timezone.utc).isoformat(),
        }
    )
    output = args.output or (
        REPO_ROOT
        / study["data_root"]
        / "provenance"
        / "nex_gddp_cmip6_download_last_run.json"
    )
    write_json(output, summary, overwrite=True)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "complete" else 2


def command_harmonize_era5(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "harmonize-era5 requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    if args.year not in years_in_period(study["periods"]["reference"]):
        raise ContractError(f"Year {args.year} is outside the reference period")
    sites = {site["id"]: site for site in iter_sites(study)}
    site = sites[args.site]
    data_root = REPO_ROOT / study["data_root"]
    source = sources["era5_land"]
    groups = era5_timeseries_groups(study, include_optional=False)
    reference_time = None
    selected_latitude = None
    selected_longitude = None
    arrays: Dict[str, Any] = {}
    input_records = []
    for group, variables in groups.items():
        path = (
            data_root
            / "raw"
            / "era5_land"
            / f"site={args.site}"
            / f"year={args.year}"
            / f"era5_land_timeseries_{args.site}_{args.year}_{group}.nc"
        )
        if not path.exists():
            raise ContractError(f"Missing raw ERA5-Land input: {path}")
        with xr.open_dataset(path) as dataset:
            time_name = coordinate_name(dataset, ("time", "valid_time"))
            if not time_name:
                raise ContractError(f"{path} lacks a time coordinate")
            time_values = np.asarray(dataset[time_name].values).astype("datetime64[ns]")
            if reference_time is None:
                reference_time = time_values
            elif not np.array_equal(reference_time, time_values):
                raise ContractError(f"Time axis mismatch for ERA5-Land group {group}")
            latitude_name = coordinate_name(dataset, ("latitude", "lat"))
            longitude_name = coordinate_name(dataset, ("longitude", "lon"))
            if not latitude_name or not longitude_name:
                raise ContractError(f"{path} lacks latitude/longitude coordinates")
            latitude = float(np.asarray(dataset[latitude_name].values).reshape(-1)[0])
            longitude = normalize_longitude(
                float(np.asarray(dataset[longitude_name].values).reshape(-1)[0])
            )
            if selected_latitude is None:
                selected_latitude = latitude
                selected_longitude = longitude
            elif not (
                math.isclose(selected_latitude, latitude, abs_tol=1e-9)
                and math.isclose(selected_longitude, longitude, abs_tol=1e-9)
            ):
                raise ContractError(
                    f"Selected grid cell mismatch for ERA5-Land group {group}"
                )
            group_variables = []
            for request_name in variables:
                spec = source["variables"][request_name]
                matches = [
                    name for name in spec["netcdf_names"] if name in dataset.data_vars
                ]
                if len(matches) != 1:
                    raise ContractError(
                        f"Expected one NetCDF variable for {request_name} in {path}, found {matches}"
                    )
                name = matches[0]
                array = dataset[name]
                units = str(array.attrs.get("units", ""))
                if units not in spec["accepted_units"]:
                    raise ContractError(
                        f"{request_name} units {units!r} are not in {spec['accepted_units']}"
                    )
                values = np.asarray(array.values, dtype=float).reshape(-1)
                if values.size != time_values.size:
                    raise ContractError(
                        f"Value/time length mismatch for {request_name}"
                    )
                arrays[request_name] = values
                group_variables.append(
                    {"request_name": request_name, "netcdf_name": name, "units": units}
                )
            input_records.append(
                {
                    "variable_group": group,
                    "path": path_text(path),
                    "sha256": sha256_file(path),
                    "variables": group_variables,
                }
            )
    expected_hours = (366 if calendar.isleap(args.year) else 365) * 24
    expected_time = np.arange(
        np.datetime64(f"{args.year}-01-01T00:00"),
        np.datetime64(f"{args.year + 1}-01-01T00:00"),
        np.timedelta64(1, "h"),
    ).astype("datetime64[ns]")
    if reference_time is None or not np.array_equal(reference_time, expected_time):
        actual = 0 if reference_time is None else len(reference_time)
        raise ContractError(
            f"ERA5-Land time axis is not the complete ordered {args.year} UTC hourly axis "
            f"({actual} values, expected {expected_hours})"
        )
    days = expected_hours // 24
    temperature_c = arrays["2m_temperature"] - 273.15
    dewpoint_c = arrays["2m_dewpoint_temperature"] - 273.15
    relative_humidity = 100.0 * np.exp(
        17.625 * dewpoint_c / (243.04 + dewpoint_c)
        - 17.625 * temperature_c / (243.04 + temperature_c)
    )
    rh_clipped_count = int(
        np.count_nonzero((relative_humidity < 0) | (relative_humidity > 100))
    )
    relative_humidity = np.clip(relative_humidity, 0.0, 100.0)
    wind_speed = np.hypot(
        arrays["10m_u_component_of_wind"],
        arrays["10m_v_component_of_wind"],
    )
    precipitation = arrays["total_precipitation"]
    radiation = arrays["surface_solar_radiation_downwards"]
    precipitation_tolerance = float(
        source["variables"]["total_precipitation"]["physical_range_tolerance"]
    )
    radiation_tolerance = float(
        source["variables"]["surface_solar_radiation_downwards"][
            "physical_range_tolerance"
        ]
    )
    if float(np.nanmin(precipitation)) < -precipitation_tolerance:
        raise ContractError(
            "Precipitation is below its declared packing-noise tolerance"
        )
    if float(np.nanmin(radiation)) < -radiation_tolerance:
        raise ContractError(
            "Solar radiation is below its declared packing-noise tolerance"
        )
    precipitation_negative_count = int(np.count_nonzero(precipitation < 0))
    radiation_negative_count = int(np.count_nonzero(radiation < 0))
    precipitation = np.clip(precipitation, 0.0, None)
    radiation = np.clip(radiation, 0.0, None)
    output_arrays = {
        "tasmax": temperature_c.reshape(days, 24).max(axis=1),
        "tasmin": temperature_c.reshape(days, 24).min(axis=1),
        "hurs": relative_humidity.reshape(days, 24).mean(axis=1),
        "sfcWind": wind_speed.reshape(days, 24).mean(axis=1),
        "rsds": radiation.reshape(days, 24).sum(axis=1) / 86400.0,
        "pr": precipitation.reshape(days, 24).sum(axis=1) * 1000.0,
    }
    target_units = {
        "tasmax": "degC",
        "tasmin": "degC",
        "hurs": "%",
        "sfcWind": "m s-1",
        "rsds": "W m-2",
        "pr": "mm day-1",
    }
    physical_ranges = {
        "tasmax": (-100.0, 70.0),
        "tasmin": (-100.0, 70.0),
        "hurs": (0.0, 100.0),
        "sfcWind": (0.0, 100.0),
        "rsds": (0.0, 1500.0),
        "pr": (0.0, 2000.0),
    }
    diagnostics = {}
    data_variables = {}
    for variable, values in output_arrays.items():
        minimum = float(np.nanmin(values))
        maximum = float(np.nanmax(values))
        missing_fraction = float(np.isnan(values).mean())
        lower, upper = physical_ranges[variable]
        if minimum < lower or maximum > upper:
            raise ContractError(
                f"{variable} harmonized range [{minimum:.6g}, {maximum:.6g}] exceeds [{lower}, {upper}]"
            )
        if missing_fraction > study["quality_gates"]["maximum_missing_fraction"]:
            raise ContractError(
                f"{variable} missing fraction {missing_fraction:.6g} exceeds gate"
            )
        diagnostics[variable] = {
            "units": target_units[variable],
            "minimum": minimum,
            "maximum": maximum,
            "missing_fraction": missing_fraction,
        }
        data_variables[variable] = (
            "time",
            values.astype("float32"),
            {"units": target_units[variable]},
        )
    daily_time = np.arange(
        np.datetime64(f"{args.year}-01-01T12:00"),
        np.datetime64(f"{args.year + 1}-01-01T12:00"),
        np.timedelta64(1, "D"),
    ).astype("datetime64[ns]")
    output_dataset = xr.Dataset(
        data_vars=data_variables,
        coords={
            "time": daily_time,
            "latitude": selected_latitude,
            "longitude": selected_longitude,
            "site": args.site,
            "zone": site["zone_id"],
        },
        attrs={
            "study_id": study["study_id"],
            "source_dataset": ERA5_TIMESERIES_DATASET_ID,
            "aggregation_day": "UTC",
            "daily_time_anchor": "12:00:00 UTC",
            "probability_semantics": "historical_reanalysis_no_scenario_probability",
            "created_utc": datetime.now(timezone.utc).isoformat(),
        },
    )
    output_name = f"daily_era5_land_{args.site}_{args.year}.nc"
    output = (
        data_root
        / "processed"
        / "daily"
        / "era5_land"
        / f"site={args.site}"
        / f"year={args.year}"
        / output_name
    )
    if output.exists() and not args.overwrite:
        raise ContractError(f"Refusing to overwrite harmonized output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    encoding = {variable: {"zlib": True, "complevel": 4} for variable in output_arrays}
    output_dataset.to_netcdf(output, encoding=encoding)
    output_dataset.close()
    provenance = {
        "schema_version": "1.0",
        "operation": "era5_land_hourly_to_daily_harmonization",
        "study_id": study["study_id"],
        "site": site,
        "year": args.year,
        "selected_grid_point": {
            "longitude": selected_longitude,
            "latitude": selected_latitude,
        },
        "inputs": input_records,
        "transformations": {
            variable["id"]: variable["era5_land"]["aggregation"]
            for variable in study["variables"]
            if variable.get("era5_land") and variable.get("required")
        },
        "numerical_adjustments": {
            "negative_precipitation_values_clipped_to_zero": precipitation_negative_count,
            "negative_radiation_values_clipped_to_zero": radiation_negative_count,
            "relative_humidity_values_clipped_to_0_100": rh_clipped_count,
        },
        "diagnostics": diagnostics,
        "output": path_text(output),
        "output_sha256": sha256_file(output),
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    provenance_path = output.with_suffix(output.suffix + ".provenance.json")
    write_json(provenance_path, provenance, args.overwrite)
    if not getattr(args, "quiet", False):
        print(json.dumps(provenance, indent=2, sort_keys=True))
    return 0


def command_harmonize_era5_range(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    reference_years = set(years_in_period(study["periods"]["reference"]))
    if args.start_year > args.end_year:
        raise ContractError("--start-year must not exceed --end-year")
    requested_years = list(range(args.start_year, args.end_year + 1))
    if not set(requested_years).issubset(reference_years):
        raise ContractError(
            "Requested harmonization range is outside the reference period"
        )
    data_root = REPO_ROOT / study["data_root"]
    completed = []
    for year in requested_years:
        harmonize_args = argparse.Namespace(
            study=args.study,
            sources=args.sources,
            site=args.site,
            year=year,
            overwrite=args.overwrite,
            quiet=True,
        )
        command_harmonize_era5(harmonize_args)
        output = (
            data_root
            / "processed"
            / "daily"
            / "era5_land"
            / f"site={args.site}"
            / f"year={year}"
            / f"daily_era5_land_{args.site}_{year}.nc"
        )
        qc_output = (
            data_root
            / "provenance"
            / "qc"
            / "daily_era5_land"
            / f"site={args.site}"
            / f"daily_era5_land_{args.site}_{year}.json"
        )
        qc_args = argparse.Namespace(
            study=args.study,
            file=output,
            expected_year=year,
            output=qc_output,
            overwrite=args.overwrite,
            quiet=True,
        )
        if command_qc_daily(qc_args) != 0:
            raise ContractError(f"Canonical daily QC failed for {args.site} {year}")
        completed.append(
            {
                "year": year,
                "output": path_text(output),
                "sha256": sha256_file(output),
                "qc": path_text(qc_output),
            }
        )
    summary = {
        "schema_version": "1.0",
        "operation": "era5_land_reference_period_harmonization",
        "site": args.site,
        "start_year": args.start_year,
        "end_year": args.end_year,
        "completed_years": len(completed),
        "records": completed,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    summary_path = (
        data_root
        / "provenance"
        / "harmonization"
        / f"era5_land_{args.site}_{args.start_year}_{args.end_year}.json"
    )
    write_json(summary_path, summary, args.overwrite)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def command_combine_era5_reference(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "combine-era5-reference requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    data_root = REPO_ROOT / study["data_root"]
    reference = study["periods"]["reference"]
    start_year = int(reference["start"][:4])
    end_year = int(reference["end"][:4])
    sites = list(iter_sites(study))
    if args.site:
        sites = [site for site in sites if site["id"] == args.site]
    records = []
    for site in sites:
        annual_datasets = []
        inputs = []
        try:
            for year in range(start_year, end_year + 1):
                path = (
                    data_root
                    / "processed"
                    / "daily"
                    / "era5_land"
                    / f"site={site['id']}"
                    / f"year={year}"
                    / f"daily_era5_land_{site['id']}_{year}.nc"
                )
                if not path.exists():
                    raise ContractError(f"Missing annual canonical input: {path}")
                with xr.open_dataset(path) as dataset:
                    annual_datasets.append(dataset.load())
                inputs.append(
                    {"year": year, "path": path_text(path), "sha256": sha256_file(path)}
                )
            combined = xr.concat(
                annual_datasets,
                dim="time",
                data_vars="minimal",
                coords="minimal",
                compat="equals",
            )
            expected_time = np.arange(
                np.datetime64(f"{start_year}-01-01T12:00"),
                np.datetime64(f"{end_year + 1}-01-01T12:00"),
                np.timedelta64(1, "D"),
            ).astype("datetime64[ns]")
            actual_time = np.asarray(combined["time"].values).astype("datetime64[ns]")
            if not np.array_equal(actual_time, expected_time):
                raise ContractError(
                    f"Combined daily time axis is incomplete for {site['id']}"
                )
            if bool((combined["tasmin"] > combined["tasmax"]).any().item()):
                raise ContractError(
                    f"tasmin exceeds tasmax in combined {site['id']} data"
                )
            combined.attrs.update(
                {
                    "reference_period": f"{start_year}-{end_year}",
                    "annual_files_combined": end_year - start_year + 1,
                    "combination_operation": "time_concat_no_value_transformation",
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                }
            )
            output = (
                data_root
                / "processed"
                / "daily"
                / "era5_land"
                / f"site={site['id']}"
                / f"reference_period={start_year}-{end_year}"
                / f"daily_era5_land_{site['id']}_{start_year}_{end_year}.nc"
            )
            if output.exists() and not args.overwrite:
                raise ContractError(f"Refusing to overwrite combined output: {output}")
            output.parent.mkdir(parents=True, exist_ok=True)
            encoding = {
                variable: {"zlib": True, "complevel": 4}
                for variable in combined.data_vars
            }
            combined.to_netcdf(output, encoding=encoding)
            combined.close()
            record = {
                "site": site,
                "reference_period": f"{start_year}-{end_year}",
                "days": len(expected_time),
                "inputs": inputs,
                "output": path_text(output),
                "output_bytes": output.stat().st_size,
                "output_sha256": sha256_file(output),
            }
            provenance_path = output.with_suffix(output.suffix + ".provenance.json")
            write_json(
                provenance_path,
                {
                    "schema_version": "1.0",
                    "operation": "era5_land_annual_daily_time_concat",
                    **record,
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                },
                args.overwrite,
            )
            records.append(record)
        finally:
            for dataset in annual_datasets:
                dataset.close()
    summary = {
        "schema_version": "1.0",
        "operation": "era5_land_reference_period_combination",
        "reference_period": f"{start_year}-{end_year}",
        "sites_completed": len(records),
        "records": records,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    summary_path = (
        data_root
        / "provenance"
        / "harmonization"
        / f"era5_land_combined_{start_year}_{end_year}.json"
    )
    write_json(summary_path, summary, args.overwrite)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def command_harmonize_nex(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "harmonize-nex requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    require_model_in_set(study, args.model, getattr(args, "model_set", "all"))
    period = nex_period_for_experiment(study, args.experiment)
    if args.year not in years_in_period(period):
        raise ContractError(
            f"Year {args.year} is outside the declared period for {args.experiment}"
        )
    sites = {site["id"]: site for site in iter_sites(study)}
    site = sites[args.site]
    nex_source = sources["nex_gddp_cmip6"]
    version = nex_source["pilot_product_version"]
    variables = required_source_variables(
        study, "nex_gddp_cmip6", include_optional=False
    )
    data_root = REPO_ROOT / study["data_root"]
    input_records = []
    arrays = {}
    reference_time = None
    reference_calendar = None
    selected_latitude = None
    selected_longitude = None
    for variable in variables:
        name = f"nex_gddp_cmip6_{args.model}_{args.experiment}_{variable}_{args.site}_{args.year}_v{version}.nc"
        path = (
            data_root
            / "raw"
            / "nex_gddp_cmip6"
            / f"model={args.model}"
            / f"experiment={args.experiment}"
            / f"variable={variable}"
            / f"site={args.site}"
            / f"year={args.year}"
            / name
        )
        if not path.exists():
            raise ContractError(f"Missing raw NEX input: {path}")
        with xr.open_dataset(path) as dataset:
            if variable not in dataset.data_vars:
                raise ContractError(f"{path} does not contain {variable}")
            if "time" not in dataset.coords:
                raise ContractError(f"{path} does not contain a time coordinate")
            time_values = np.asarray(dataset["time"].values)
            calendar_name = dataset["time"].encoding.get("calendar") or dataset[
                "time"
            ].attrs.get("calendar", "standard")
            if reference_time is None:
                reference_time = time_values
                reference_calendar = str(calendar_name)
            elif not np.array_equal(reference_time, time_values):
                raise ContractError(f"Time axis mismatch for {variable}")
            elif str(calendar_name) != reference_calendar:
                raise ContractError(f"Calendar mismatch for {variable}")
            latitude_name = coordinate_name(dataset, ("latitude", "lat"))
            longitude_name = coordinate_name(dataset, ("longitude", "lon"))
            if not latitude_name or not longitude_name:
                raise ContractError(f"{path} lacks latitude/longitude coordinates")
            latitude = float(np.asarray(dataset[latitude_name].values).reshape(-1)[0])
            longitude = normalize_longitude(
                float(np.asarray(dataset[longitude_name].values).reshape(-1)[0])
            )
            if selected_latitude is None:
                selected_latitude = latitude
                selected_longitude = longitude
            elif not (
                math.isclose(selected_latitude, latitude, abs_tol=1e-9)
                and math.isclose(selected_longitude, longitude, abs_tol=1e-9)
            ):
                raise ContractError(f"Selected grid cell mismatch for {variable}")
            array = dataset[variable]
            if array.ndim != 1 or not ({"obs", "time"} & set(array.dims)):
                raise ContractError(
                    f"{variable} is not a one-dimensional NCSS point or corridor-extracted time series"
                )
            values = np.asarray(array.values, dtype=float).reshape(-1)
            if values.size != time_values.size:
                raise ContractError(f"Value/time length mismatch for {variable}")
            declared_units = str(
                array.attrs.get(
                    "source_declared_units", array.attrs.get("units", "")
                )
            )
            units, metadata_repair = resolve_nex_native_units(
                nex_source, args.model, variable, declared_units
            )
            stored_units = str(array.attrs.get("units", ""))
            if stored_units != units:
                raise ContractError(
                    f"{path} stores {variable} units {stored_units!r}; expected {units!r}"
                )
            arrays[variable] = values
            input_records.append(
                {
                    "variable": variable,
                    "path": path_text(path),
                    "sha256": sha256_file(path),
                    "source_declared_units": declared_units,
                    "effective_native_units": units,
                    "metadata_repair_id": (
                        metadata_repair["id"] if metadata_repair else None
                    ),
                }
            )
    expected_days = expected_days_in_calendar_year(
        args.year, str(reference_calendar or "standard")
    )
    if reference_time is None or len(reference_time) != expected_days:
        raise ContractError(
            f"Expected {expected_days} aligned daily values, found {0 if reference_time is None else len(reference_time)}"
        )
    canonical_humidity, humidity_clipped_count = clip_relative_humidity(
        arrays["hurs"], np
    )
    output_arrays = {
        "tasmax": arrays["tasmax"] - 273.15,
        "tasmin": arrays["tasmin"] - 273.15,
        "hurs": canonical_humidity,
        "sfcWind": arrays["sfcWind"],
        "rsds": arrays["rsds"],
        "pr": arrays["pr"] * 86400.0,
    }
    if bool(np.any(output_arrays["tasmin"] > output_arrays["tasmax"])):
        raise ContractError("Cross-variable gate failed: tasmin exceeds tasmax")
    target_units = {
        "tasmax": "degC",
        "tasmin": "degC",
        "hurs": "%",
        "sfcWind": "m s-1",
        "rsds": "W m-2",
        "pr": "mm day-1",
    }
    physical_ranges = {
        "tasmax": (-100.0, 70.0),
        "tasmin": (-100.0, 70.0),
        "hurs": (0.0, 100.5),
        "sfcWind": (0.0, 100.0),
        "rsds": (0.0, 1500.0),
        "pr": (0.0, 2000.0),
    }
    diagnostics = {}
    data_variables = {}
    for variable, values in output_arrays.items():
        minimum = float(np.nanmin(values))
        maximum = float(np.nanmax(values))
        missing_fraction = float(np.isnan(values).mean())
        lower, upper = physical_ranges[variable]
        if minimum < lower or maximum > upper:
            raise ContractError(
                f"{variable} harmonized range [{minimum:.6g}, {maximum:.6g}] exceeds [{lower}, {upper}]"
            )
        if missing_fraction > study["quality_gates"]["maximum_missing_fraction"]:
            raise ContractError(
                f"{variable} missing fraction {missing_fraction:.6g} exceeds gate"
            )
        diagnostics[variable] = {
            "units": target_units[variable],
            "minimum": minimum,
            "maximum": maximum,
            "missing_fraction": missing_fraction,
        }
        data_variables[variable] = (
            "time",
            values.astype("float32"),
            {"units": target_units[variable]},
        )
    output_dataset = xr.Dataset(
        data_vars=data_variables,
        coords={
            "time": reference_time,
            "latitude": selected_latitude,
            "longitude": selected_longitude,
            "site": args.site,
            "zone": site["zone_id"],
        },
        attrs={
            "study_id": study["study_id"],
            "source_dataset": "NEX-GDDP-CMIP6",
            "source_product_version": version,
            "model": args.model,
            "experiment": args.experiment,
            "realization": nex_realization(study, args.model),
            "source_calendar": reference_calendar,
            "probability_semantics": "conditional_on_model_experiment_epoch",
            "created_utc": datetime.now(timezone.utc).isoformat(),
        },
    )
    output_name = f"daily_nex_gddp_cmip6_{args.model}_{args.experiment}_{args.site}_{args.year}_v{version}.nc"
    output = (
        data_root
        / "processed"
        / "daily"
        / "nex_gddp_cmip6"
        / f"model={args.model}"
        / f"experiment={args.experiment}"
        / f"site={args.site}"
        / f"year={args.year}"
        / output_name
    )
    if output.exists() and not args.overwrite:
        raise ContractError(f"Refusing to overwrite harmonized output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    encoding = {variable: {"zlib": True, "complevel": 4} for variable in variables}
    output_dataset.to_netcdf(output, encoding=encoding)
    output_dataset.close()
    provenance = {
        "schema_version": "1.0",
        "operation": "nex_daily_unit_harmonization",
        "study_id": study["study_id"],
        "climate_state": {
            "model": args.model,
            "experiment": args.experiment,
            "year": args.year,
        },
        "site": site,
        "inputs": input_records,
        "transformations": study["harmonization"]["nex_gddp_cmip6"],
        "harmonization_counts": {
            "hurs_values_clipped_to_0_100_percent": humidity_clipped_count
        },
        "diagnostics": diagnostics,
        "output": path_text(output),
        "output_sha256": sha256_file(output),
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    provenance_path = output.with_suffix(output.suffix + ".provenance.json")
    write_json(provenance_path, provenance, args.overwrite)
    if not getattr(args, "quiet", False):
        print(json.dumps(provenance, indent=2, sort_keys=True))
    return 0


def execute_nex_harmonize_task(
    task: Tuple[Path, Path, str, str, str, int, str, bool],
) -> Dict[str, Any]:
    study_path, sources_path, model, experiment, site, year, version, overwrite = task
    study = load_json(study_path)
    data_root = REPO_ROOT / study["data_root"]
    output = (
        data_root
        / "processed"
        / "daily"
        / "nex_gddp_cmip6"
        / f"model={model}"
        / f"experiment={experiment}"
        / f"site={site}"
        / f"year={year}"
        / f"daily_nex_gddp_cmip6_{model}_{experiment}_{site}_{year}_v{version}.nc"
    )
    provenance_path = output.with_suffix(output.suffix + ".provenance.json")
    state = {"model": model, "experiment": experiment, "site": site, "year": year}
    if output.exists() and provenance_path.exists() and not overwrite:
        try:
            provenance = load_json(provenance_path)
            if provenance.get("output_sha256") == sha256_file(output):
                return {"valid": True, "disposition": "verified_reused", **state}
        except ContractError:
            pass
        return {
            "valid": False,
            "error": "Existing canonical output or provenance failed checksum verification",
            **state,
        }
    child = argparse.Namespace(
        study=study_path,
        sources=sources_path,
        model=model,
        experiment=experiment,
        site=site,
        year=year,
        overwrite=overwrite,
        quiet=True,
    )
    try:
        command_harmonize_nex(child)
        return {"valid": True, "disposition": "created", **state}
    except (OSError, ContractError, TypeError, ValueError) as exc:
        return {"valid": False, "error": str(exc), **state}


def command_harmonize_nex_pilot(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    if args.workers < 1:
        raise ContractError("harmonize-nex-pilot workers must be at least one")
    selected_models = models_for_set(study, args.model_set)
    if args.model:
        require_model_in_set(study, args.model, args.model_set)
    models = [args.model] if args.model else selected_models
    experiments = (
        [args.experiment]
        if args.experiment
        else ["historical", *study["climate_states"]["pilot_ssps"]]
    )
    sites = [args.site] if args.site else [site["id"] for site in iter_sites(study)]
    version = sources["nex_gddp_cmip6"]["pilot_product_version"]
    tasks = []
    for model in models:
        for experiment in experiments:
            years = list(years_in_period(nex_period_for_experiment(study, experiment)))
            if args.start_year is not None:
                years = [year for year in years if year >= args.start_year]
            if args.end_year is not None:
                years = [year for year in years if year <= args.end_year]
            for site in sites:
                for year in years:
                    tasks.append(
                        (
                            args.study,
                            args.sources,
                            model,
                            experiment,
                            site,
                            year,
                            version,
                            args.overwrite,
                        )
                    )
    if not tasks:
        raise ContractError("The requested harmonization filters select no pilot data")
    results = []
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.workers) as executor:
        for result in executor.map(execute_nex_harmonize_task, tasks):
            results.append(result)
            if args.progress:
                state = "OK" if result["valid"] else "FAIL"
                print(
                    f"{state} {result['model']} {result['experiment']} "
                    f"{result['site']} {result['year']} ",
                    flush=True,
                )
    failures = [result for result in results if not result["valid"]]
    dispositions = {}
    for result in results:
        disposition = result.get("disposition", "failed")
        dispositions[disposition] = dispositions.get(disposition, 0) + 1
    summary = {
        "schema_version": "1.0",
        "operation": "nex_gddp_cmip6_pilot_harmonization",
        "status": "complete" if not failures else "incomplete",
        "task_count": len(tasks),
        "completed": len(tasks) - len(failures),
        "dispositions": dispositions,
        "failures": failures,
        "workers": args.workers,
        "finished_utc": datetime.now(timezone.utc).isoformat(),
    }
    output = args.output or (
        REPO_ROOT
        / study["data_root"]
        / "provenance"
        / "nex_gddp_cmip6_harmonization_last_run.json"
    )
    write_json(output, summary, overwrite=True)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if not failures else 2


def audit_nex_catalog_task(
    task: Tuple[str, str, str, List[int], str, str, int],
) -> Dict[str, Any]:
    model, experiment, variable, expected_years, realization, version, timeout = task
    catalog_url = nex_catalog_url(model, experiment, realization, variable)
    record: Dict[str, Any] = {
        "model": model,
        "experiment": experiment,
        "realization": realization,
        "variable": variable,
        "catalog_url": catalog_url,
        "expected_years": expected_years,
    }
    try:
        with urllib.request.urlopen(catalog_url, timeout=timeout) as response:
            catalog_xml = response.read()
        root = ET.fromstring(catalog_xml)
        paths = [
            element.attrib["urlPath"]
            for element in root.iter()
            if "urlPath" in element.attrib
        ]
        available_years = []
        observed_versions = set()
        for path in paths:
            match = re.search(r"_(\d{4})_v([^_/]+)\.nc$", path)
            if match:
                observed_versions.add(match.group(2))
                if match.group(2) == version:
                    available_years.append(int(match.group(1)))
        missing_years = sorted(set(expected_years) - set(available_years))
        record.update(
            {
                "valid": not missing_years,
                "missing_years": missing_years,
                "available_year_count_for_version": len(set(available_years)),
                "observed_versions": sorted(observed_versions),
            }
        )
    except (OSError, ET.ParseError) as exc:
        record.update(
            {"valid": False, "error": str(exc), "missing_years": expected_years}
        )
    return record


def command_nex_audit(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    report = validate_contract(study, load_json(args.sources))
    if not report["valid"]:
        raise ContractError("Contract is invalid; run validate for details")
    source = sources["nex_gddp_cmip6"]
    version = source["pilot_product_version"]
    variables = required_source_variables(
        study, "nex_gddp_cmip6", include_optional=False
    )
    experiments = ["historical"] + list(study["climate_states"]["pilot_ssps"])
    tasks = []
    for model in models_for_set(study, args.model_set):
        for experiment in experiments:
            years = list(years_in_period(nex_period_for_experiment(study, experiment)))
            for variable in variables:
                tasks.append(
                    (
                        model,
                        experiment,
                        variable,
                        years,
                        nex_realization(study, model),
                        version,
                        args.timeout,
                    )
                )
    records: List[Dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
        for record in executor.map(audit_nex_catalog_task, tasks):
            records.append(record)
            if args.progress:
                state = "OK" if record["valid"] else "FAIL"
                print(
                    f"{state} {record['model']} {record['experiment']} {record['variable']}",
                    flush=True,
                )
    failed = [record for record in records if not record["valid"]]
    audit = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "source": "nex_gddp_cmip6",
        "product_version": version,
        "checked_utc": datetime.now(timezone.utc).isoformat(),
        "valid": not failed,
        "catalog_count": len(records),
        "valid_catalog_count": len(records) - len(failed),
        "failed_catalog_count": len(failed),
        "records": records,
    }
    output = (
        args.output
        or REPO_ROOT
        / study["data_root"]
        / "provenance"
        / "nex_gddp_cmip6_catalog_audit.json"
    )
    write_json(output, audit, args.overwrite)
    print(
        json.dumps(
            {key: value for key, value in audit.items() if key != "records"},
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if audit["valid"] else 2


def coordinate_name(dataset: Any, candidates: Sequence[str]) -> Optional[str]:
    for name in candidates:
        if name in dataset.coords:
            return name
    return None


def normalize_longitude(value: float) -> float:
    return value - 360.0 if value > 180.0 else value


def command_qc_netcdf(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "qc-netcdf requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    if args.source not in sources:
        raise ContractError(f"Unknown source: {args.source}")
    source = sources[args.source]
    expected = source.get("variables", {})
    gates = study["quality_gates"]
    errors: List[str] = []
    warnings: List[str] = []
    variables: Dict[str, Any] = {}
    time_summary: Dict[str, Any] = {}
    spatial_summary: Dict[str, Any] = {}
    with xr.open_dataset(args.file) as dataset:
        time_name = coordinate_name(dataset, ("time", "valid_time"))
        if not time_name:
            errors.append("No time coordinate found")
        else:
            time = dataset[time_name]
            calendar_name = time.encoding.get("calendar") or time.attrs.get("calendar")
            if gates.get("require_declared_calendar") and not calendar_name:
                errors.append("Time calendar is not declared in attributes or encoding")
            elif calendar_name and calendar_name not in gates.get(
                "accepted_calendars", []
            ):
                errors.append(f"Unsupported calendar: {calendar_name}")
            values = np.asarray(time.values)
            time_summary = {
                "count": int(values.size),
                "calendar": calendar_name,
                "first": str(values[0]) if values.size else None,
                "last": str(values[-1]) if values.size else None,
            }
            if values.size > 1:
                comparable = (
                    values.astype("datetime64[ns]").astype("int64")
                    if np.issubdtype(values.dtype, np.datetime64)
                    else values
                )
                if gates.get("require_monotonic_unique_time") and not bool(
                    np.all(comparable[1:] > comparable[:-1])
                ):
                    errors.append(
                        "Time coordinate is not strictly increasing and unique"
                    )
            if args.expected_year:
                expected_year = int(args.expected_year)
                calendar_key = str(calendar_name or "standard")
                expected_days = expected_days_in_calendar_year(
                    expected_year, calendar_key
                )
                source_frequency = source.get("temporal_resolution")
                expected_count = expected_days * (
                    24 if source_frequency == "hourly" else 1
                )
                time_summary["expected_year"] = expected_year
                time_summary["expected_count"] = expected_count
                if int(values.size) != expected_count:
                    errors.append(
                        f"Time count {values.size} does not match expected {expected_count} for {expected_year}"
                    )
                if values.size and (
                    str(values[0])[:4] != str(expected_year)
                    or str(values[-1])[:4] != str(expected_year)
                ):
                    errors.append(
                        f"Time coverage is not confined to expected year {expected_year}"
                    )
            else:
                warnings.append(
                    "No expected year was supplied; temporal completeness was not checked"
                )
        recognized = 0
        for request_name, spec in expected.items():
            matches = [
                name
                for name in spec.get("netcdf_names", [])
                if name in dataset.data_vars
            ]
            if not matches:
                continue
            recognized += 1
            name = matches[0]
            array = dataset[name]
            units = str(array.attrs.get("units", ""))
            accepted_units = spec.get("accepted_units", [])
            if gates.get("reject_unit_conflicts") and units not in accepted_units:
                errors.append(f"{name}: units {units!r} not in {accepted_units}")
            missing = float(array.isnull().mean().compute().item())
            minimum = float(array.min(skipna=True).compute().item())
            maximum = float(array.max(skipna=True).compute().item())
            if not math.isfinite(missing):
                errors.append(f"{name}: missing fraction is not finite")
            elif missing > float(gates["maximum_missing_fraction"]):
                errors.append(f"{name}: missing fraction {missing:.6g} exceeds gate")
            physical_range = spec.get("physical_range")
            if physical_range:
                lower = float(physical_range[0])
                upper = float(physical_range[1])
                tolerance = float(spec.get("physical_range_tolerance", 0.0))
                outside_nominal = minimum < lower or maximum > upper
                outside_tolerance = (
                    minimum < lower - tolerance or maximum > upper + tolerance
                )
                if outside_tolerance:
                    errors.append(
                        f"{name}: range [{minimum:.6g}, {maximum:.6g}] exceeds physical gate "
                        f"{physical_range} with absolute tolerance {tolerance:.6g}"
                    )
                elif outside_nominal:
                    warnings.append(
                        f"{name}: range [{minimum:.6g}, {maximum:.6g}] crosses nominal physical "
                        f"gate {physical_range} within declared packing-noise tolerance "
                        f"{tolerance:.6g}; preserve raw values and clip only during harmonization"
                    )
            variables[name] = {
                "request_name": request_name,
                "units": units,
                "missing_fraction": missing,
                "minimum": minimum,
                "maximum": maximum,
                "shape": list(array.shape),
            }
        if recognized == 0:
            errors.append(f"No recognized {args.source} variables found")
        latitude_name = coordinate_name(dataset, ("latitude", "lat"))
        longitude_name = coordinate_name(dataset, ("longitude", "lon"))
        extent = study["spatial_design"]["acquisition_extent"]
        if latitude_name and longitude_name:
            latitudes = np.asarray(dataset[latitude_name].values, dtype=float)
            longitudes = np.vectorize(normalize_longitude)(
                np.asarray(dataset[longitude_name].values, dtype=float)
            )
            west, south, east, north = extent
            spatial_summary = {
                "latitude_min": float(latitudes.min()),
                "latitude_max": float(latitudes.max()),
                "longitude_min": float(longitudes.min()),
                "longitude_max": float(longitudes.max()),
            }
            tolerance = 0.3
            if (
                latitudes.min() < south - tolerance
                or latitudes.max() > north + tolerance
            ):
                errors.append(
                    "Latitude coordinates extend beyond the Shaanxi buffered acquisition support"
                )
            if (
                longitudes.min() < west - tolerance
                or longitudes.max() > east + tolerance
            ):
                errors.append(
                    "Longitude coordinates extend beyond the Shaanxi buffered acquisition support"
                )
        else:
            warnings.append(
                "Latitude/longitude coordinates were not both found; spatial support was not checked"
            )
    report = {
        "valid": not errors,
        "file": path_text(args.file),
        "source": args.source,
        "sha256": sha256_file(args.file),
        "bytes": args.file.stat().st_size,
        "errors": errors,
        "warnings": warnings,
        "time": time_summary,
        "spatial": spatial_summary,
        "variables": variables,
        "checked_utc": datetime.now(timezone.utc).isoformat(),
    }
    if args.output:
        write_json(args.output, report, args.overwrite)
    if not getattr(args, "quiet", False):
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


def command_subset_netcdf(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "subset-netcdf requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    data_root = (REPO_ROOT / study["data_root"]).resolve()
    output = args.output.resolve()
    allowed_roots = [
        (data_root / "interim").resolve(),
        (data_root / "processed").resolve(),
    ]
    if not any(
        root == output.parent or root in output.parents for root in allowed_roots
    ):
        raise ContractError(
            "Subset output must be under data/climate_shaanxi/interim or processed"
        )
    if output.exists() and not args.overwrite:
        raise ContractError(f"Refusing to overwrite existing subset: {output}")
    sites = list(iter_sites(study))
    with xr.open_dataset(args.file) as dataset:
        latitude_name = coordinate_name(dataset, ("latitude", "lat"))
        longitude_name = coordinate_name(dataset, ("longitude", "lon"))
        if not latitude_name or not longitude_name:
            raise ContractError(
                "Cannot subset without latitude and longitude coordinates"
            )
        source_lons = np.asarray(dataset[longitude_name].values, dtype=float)
        uses_360 = bool(source_lons.size and np.nanmax(source_lons) > 180.0)
        lat_index = xr.DataArray(
            [site["latitude"] for site in sites],
            dims="site",
            coords={"site": [site["id"] for site in sites]},
        )
        lon_values = [
            site["longitude"] % 360.0 if uses_360 else site["longitude"]
            for site in sites
        ]
        lon_index = xr.DataArray(
            lon_values, dims="site", coords={"site": [site["id"] for site in sites]}
        )
        subset = dataset.sel(
            {latitude_name: lat_index, longitude_name: lon_index}, method="nearest"
        )
        selected_lats = np.asarray(subset[latitude_name].values, dtype=float)
        selected_lons = np.asarray(subset[longitude_name].values, dtype=float)
        distances = []
        for position, site in enumerate(sites):
            selected_lon = normalize_longitude(float(selected_lons[position]))
            distance = math.hypot(
                float(selected_lats[position]) - site["latitude"],
                selected_lon - site["longitude"],
            )
            distances.append(distance)
            if distance > args.max_distance_degrees:
                raise ContractError(
                    f"Nearest grid cell for {site['id']} is {distance:.3f} degrees away"
                )
        subset = subset.assign_coords(
            zone=("site", [site["zone_id"] for site in sites])
        )
        subset.attrs.update(
            {
                "study_id": study["study_id"],
                "subset_method": "nearest_grid_cell_to_registered_representative_points",
                "source_file_sha256": sha256_file(args.file),
                "created_utc": datetime.now(timezone.utc).isoformat(),
            }
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        subset.to_netcdf(output)
    provenance = {
        "schema_version": "1.0",
        "operation": "representative_point_subset",
        "source": path_text(args.file),
        "source_sha256": sha256_file(args.file),
        "output": path_text(output),
        "output_sha256": sha256_file(output),
        "sites": [site["id"] for site in sites],
        "nearest_cell_distance_degrees": dict(
            zip((site["id"] for site in sites), distances)
        ),
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    provenance_path = output.with_suffix(output.suffix + ".provenance.json")
    write_json(provenance_path, provenance, args.overwrite)
    print(json.dumps(provenance, indent=2, sort_keys=True))
    return 0


def command_qc_daily(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "qc-daily requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    required_units = {
        "tasmax": "degC",
        "tasmin": "degC",
        "hurs": "%",
        "sfcWind": "m s-1",
        "rsds": "W m-2",
        "pr": "mm day-1",
    }
    errors = []
    diagnostics = {}
    with xr.open_dataset(args.file) as dataset:
        missing_variables = sorted(set(required_units) - set(dataset.data_vars))
        if missing_variables:
            errors.append(f"Missing canonical variables: {missing_variables}")
        if "time" not in dataset.coords:
            errors.append("Missing time coordinate")
            time_summary = {}
        else:
            values = np.asarray(dataset["time"].values)
            calendar_name = dataset["time"].encoding.get("calendar") or dataset[
                "time"
            ].attrs.get("calendar", "standard")
            expected_count = expected_days_in_calendar_year(
                args.expected_year, str(calendar_name)
            )
            time_summary = {
                "count": int(values.size),
                "expected_count": expected_count,
                "first": str(values[0]) if values.size else None,
                "last": str(values[-1]) if values.size else None,
                "calendar": calendar_name,
            }
            if values.size != expected_count:
                errors.append(
                    f"Time count {values.size} does not match expected {expected_count}"
                )
            if values.size and (
                str(values[0])[:4] != str(args.expected_year)
                or str(values[-1])[:4] != str(args.expected_year)
            ):
                errors.append(f"Time coverage does not match {args.expected_year}")
            if values.size > 1:
                if not bool(np.all(values[1:] > values[:-1])):
                    errors.append("Time coordinate is not strictly increasing")
        for variable, expected_units in required_units.items():
            if variable not in dataset.data_vars:
                continue
            array = dataset[variable]
            units = str(array.attrs.get("units", ""))
            missing_fraction = float(array.isnull().mean().compute().item())
            diagnostics[variable] = {
                "units": units,
                "minimum": float(array.min(skipna=True).compute().item()),
                "maximum": float(array.max(skipna=True).compute().item()),
                "missing_fraction": missing_fraction,
            }
            if units != expected_units:
                errors.append(
                    f"{variable}: units {units!r} do not match {expected_units!r}"
                )
            if missing_fraction > study["quality_gates"]["maximum_missing_fraction"]:
                errors.append(
                    f"{variable}: missing fraction {missing_fraction:.6g} exceeds gate"
                )
        if "tasmin" in dataset and "tasmax" in dataset:
            violations = int((dataset["tasmin"] > dataset["tasmax"]).sum().item())
            if violations:
                errors.append(f"tasmin exceeds tasmax on {violations} days")
        source_dataset = dataset.attrs.get("source_dataset")
        required_attributes = {"study_id": study["study_id"]}
        if source_dataset == "NEX-GDDP-CMIP6":
            required_attributes["probability_semantics"] = (
                "conditional_on_model_experiment_epoch"
            )
        elif source_dataset == ERA5_TIMESERIES_DATASET_ID:
            required_attributes.update(
                {
                    "probability_semantics": "historical_reanalysis_no_scenario_probability",
                    "aggregation_day": "UTC",
                }
            )
        else:
            errors.append(f"Unsupported or missing source_dataset: {source_dataset!r}")
        for name, expected_value in required_attributes.items():
            if dataset.attrs.get(name) != expected_value:
                errors.append(
                    f"Attribute {name} is {dataset.attrs.get(name)!r}, expected {expected_value!r}"
                )
        state = {
            "source_dataset": source_dataset,
            "model": dataset.attrs.get("model"),
            "experiment": dataset.attrs.get("experiment"),
            "year": args.expected_year,
            "site": str(np.asarray(dataset.coords.get("site", "")).item()),
            "zone": str(np.asarray(dataset.coords.get("zone", "")).item()),
        }
        required_state_fields = ["site", "zone"]
        if source_dataset == "NEX-GDDP-CMIP6":
            required_state_fields.extend(["model", "experiment"])
        for field in required_state_fields:
            if not state[field]:
                errors.append(f"Missing climate-state coordinate or attribute: {field}")
    report = {
        "valid": not errors,
        "file": path_text(args.file),
        "sha256": sha256_file(args.file),
        "errors": errors,
        "state": state,
        "time": time_summary,
        "variables": diagnostics,
        "checked_utc": datetime.now(timezone.utc).isoformat(),
    }
    if args.output:
        write_json(args.output, report, args.overwrite)
    if not getattr(args, "quiet", False):
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


def command_audit_era5_reference(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    data_root = REPO_ROOT / study["data_root"]
    reference = study["periods"]["reference"]
    start_year = int(reference["start"][:4])
    end_year = int(reference["end"][:4])
    period_id = f"{start_year}-{end_year}"
    groups = era5_timeseries_groups(study, include_optional=False)
    errors = []
    site_records = []
    period_inventory = []
    total_annual_group_files = 0
    total_daily_files = 0
    total_qc_passed = 0
    total_station_days = 0
    for site in iter_sites(study):
        site_periods = []
        for group in groups:
            stem = f"era5_land_timeseries_{site['id']}_{period_id}_{group}"
            request_path = (
                data_root
                / "provenance"
                / "requests"
                / "era5_land_periods"
                / f"site={site['id']}"
                / f"period={period_id}"
                / f"{stem}.request.json"
            )
            if not request_path.exists():
                errors.append(
                    f"Missing period request record: {path_text(request_path)}"
                )
                continue
            request_record = load_json(request_path)
            if (
                request_record.get("retrieval_status")
                != "retrieved_partitioned_unvalidated"
            ):
                errors.append(
                    f"Unexpected retrieval status in {path_text(request_path)}: "
                    f"{request_record.get('retrieval_status')!r}"
                )
            for target_key, hash_key in (
                ("archive_target", "archive_sha256"),
                ("netcdf_target", "netcdf_sha256"),
            ):
                target = REPO_ROOT / request_record[target_key]
                if not target.exists():
                    errors.append(f"Missing period file: {path_text(target)}")
                elif sha256_file(target) != request_record.get(hash_key):
                    errors.append(f"Checksum mismatch: {path_text(target)}")
            partitions = request_record.get("annual_partitions", [])
            if len(partitions) != end_year - start_year + 1:
                errors.append(
                    f"{site['id']} {group} has {len(partitions)} annual partitions"
                )
            verified_partitions = 0
            for partition in partitions:
                target = REPO_ROOT / partition["target"]
                if not target.exists():
                    errors.append(f"Missing annual partition: {path_text(target)}")
                elif sha256_file(target) != partition.get("sha256"):
                    errors.append(f"Annual checksum mismatch: {path_text(target)}")
                else:
                    verified_partitions += 1
            total_annual_group_files += verified_partitions
            period_record = {
                "variable_group": group,
                "request_record": path_text(request_path),
                "archive_sha256": request_record.get("archive_sha256"),
                "netcdf_sha256": request_record.get("netcdf_sha256"),
                "annual_partitions_verified": verified_partitions,
            }
            site_periods.append(period_record)
            period_inventory.append({"site": site["id"], **period_record})
        harmonization_path = (
            data_root
            / "provenance"
            / "harmonization"
            / f"era5_land_{site['id']}_{start_year}_{end_year}.json"
        )
        if not harmonization_path.exists():
            errors.append(
                f"Missing harmonization summary: {path_text(harmonization_path)}"
            )
            daily_records = []
        else:
            daily_records = load_json(harmonization_path).get("records", [])
        if len(daily_records) != end_year - start_year + 1:
            errors.append(
                f"{site['id']} has {len(daily_records)} daily records, expected 30"
            )
        grid_point = None
        for daily_record in daily_records:
            output = REPO_ROOT / daily_record["output"]
            qc_path = REPO_ROOT / daily_record["qc"]
            if not output.exists():
                errors.append(f"Missing daily file: {path_text(output)}")
                continue
            output_sha256 = sha256_file(output)
            if output_sha256 != daily_record.get("sha256"):
                errors.append(f"Daily checksum mismatch: {path_text(output)}")
                continue
            total_daily_files += 1
            if not qc_path.exists():
                errors.append(f"Missing daily QC: {path_text(qc_path)}")
                continue
            qc = load_json(qc_path)
            if not qc.get("valid") or qc.get("sha256") != output_sha256:
                errors.append(f"Invalid daily QC: {path_text(qc_path)}")
                continue
            total_qc_passed += 1
            total_station_days += int(qc.get("time", {}).get("count", 0))
            if grid_point is None:
                provenance_path = output.with_suffix(output.suffix + ".provenance.json")
                if provenance_path.exists():
                    grid_point = load_json(provenance_path).get("selected_grid_point")
        site_records.append(
            {
                "site": site,
                "selected_grid_point": grid_point,
                "period_groups_verified": len(site_periods),
                "daily_files_verified": len(daily_records),
                "harmonization_summary": path_text(harmonization_path),
            }
        )
    combined_summary_path = (
        data_root
        / "provenance"
        / "harmonization"
        / f"era5_land_combined_{start_year}_{end_year}.json"
    )
    combined_verified = 0
    if not combined_summary_path.exists():
        errors.append(f"Missing combined summary: {path_text(combined_summary_path)}")
    else:
        combined_records = load_json(combined_summary_path).get("records", [])
        for combined_record in combined_records:
            output = REPO_ROOT / combined_record["output"]
            if not output.exists():
                errors.append(f"Missing combined file: {path_text(output)}")
            elif sha256_file(output) != combined_record.get("output_sha256"):
                errors.append(f"Combined checksum mismatch: {path_text(output)}")
            elif int(combined_record.get("days", 0)) != days_in_period(reference):
                errors.append(f"Combined day count mismatch: {path_text(output)}")
            else:
                combined_verified += 1
    report = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "operation": "era5_land_reference_period_completeness_audit",
        "valid": not errors,
        "reference_period": period_id,
        "expected": {
            "sites": 4,
            "period_requests": 16,
            "annual_group_files": 480,
            "daily_files": 120,
            "daily_qc_reports": 120,
            "combined_reference_files": 4,
            "station_days": days_in_period(reference) * 4,
        },
        "observed": {
            "sites": len(site_records),
            "period_requests": len(period_inventory),
            "annual_group_files": total_annual_group_files,
            "daily_files": total_daily_files,
            "daily_qc_reports_passed": total_qc_passed,
            "combined_reference_files": combined_verified,
            "station_days": total_station_days,
        },
        "sites": site_records,
        "period_inventory": period_inventory,
        "errors": errors,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    output = (
        args.output
        or data_root / "provenance" / "era5_land_reference_period_audit.json"
    )
    write_json(output, report, args.overwrite)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


def boolean_run_lengths(values: Sequence[bool]) -> List[Tuple[int, int, int]]:
    runs: List[Tuple[int, int, int]] = []
    start: Optional[int] = None
    for index, value in enumerate(values):
        if value and start is None:
            start = index
        elif not value and start is not None:
            runs.append((start, index, index - start))
            start = None
    if start is not None:
        runs.append((start, len(values), len(values) - start))
    return runs


def maximum_run_length(values: Sequence[bool]) -> int:
    runs = boolean_run_lengths(values)
    return max((length for _, _, length in runs), default=0)


def masked_mean_or_blank(values: Any, selected: Any, np: Any) -> Any:
    sample = np.asarray(values, dtype=float)[np.asarray(selected, dtype=bool)]
    return float(np.mean(sample)) if sample.size else ""


def theil_sen_slope(years: Sequence[float], values: Sequence[float]) -> float:
    slopes = []
    for left in range(len(years)):
        if not math.isfinite(float(values[left])):
            continue
        for right in range(left + 1, len(years)):
            if not math.isfinite(float(values[right])):
                continue
            delta_year = float(years[right]) - float(years[left])
            if delta_year:
                slopes.append((float(values[right]) - float(values[left])) / delta_year)
    if not slopes:
        return float("nan")
    slopes.sort()
    middle = len(slopes) // 2
    if len(slopes) % 2:
        return float(slopes[middle])
    return float((slopes[middle - 1] + slopes[middle]) / 2.0)


def noleap_day_indices(months: Sequence[int], days: Sequence[int]) -> List[int]:
    offsets = (0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334)
    month_lengths = (31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31)
    if len(months) != len(days):
        raise ContractError("Month and day arrays must have identical lengths")
    result = []
    for month, day in zip(months, days):
        if not 1 <= int(month) <= 12:
            raise ContractError(f"Invalid month in time axis: {month}")
        if not 1 <= int(day) <= month_lengths[int(month) - 1]:
            raise ContractError(f"Invalid month/day in time axis: {month}/{day}")
        if month == 2 and day == 29:
            result.append(offsets[1] + 27)
        else:
            result.append(offsets[month - 1] + day - 1)
    return result


def calendar_day_percentile_thresholds(
    values: Any,
    day_indices: Any,
    months: Any,
    days: Any,
    quantile: float,
    window_days: int,
    np: Any,
) -> Any:
    if window_days <= 0 or window_days % 2 == 0:
        raise ContractError(
            "Calendar-day percentile window must be a positive odd number"
        )
    if not 0.0 <= quantile <= 1.0:
        raise ContractError("Calendar-day percentile quantile must be in [0, 1]")
    value_array = np.asarray(values, dtype=float)
    day_array = np.asarray(day_indices, dtype=int)
    month_array = np.asarray(months, dtype=int)
    date_array = np.asarray(days, dtype=int)
    if not (value_array.size == day_array.size == month_array.size == date_array.size):
        raise ContractError("Calendar-day percentile inputs must have equal lengths")
    if np.any((day_array < 0) | (day_array >= 365)):
        raise ContractError("No-leap day indices must be in [0, 364]")
    keep = ~((month_array == 2) & (date_array == 29))
    training_values = value_array[keep]
    training_days = day_array[keep]
    half_window = window_days // 2
    thresholds = np.empty(365, dtype=float)
    for target in range(365):
        distance = np.abs(training_days - target)
        circular_distance = np.minimum(distance, 365 - distance)
        sample = training_values[circular_distance <= half_window]
        sample = sample[np.isfinite(sample)]
        if sample.size == 0:
            raise ContractError(f"No finite samples for calendar day {target + 1}")
        thresholds[target] = float(np.quantile(sample, quantile, method="linear"))
    return thresholds


def relative_humidity_from_temperature_dewpoint(
    temperature_c: Any, dewpoint_c: Any, np: Any
) -> Any:
    return 100.0 * np.exp(
        17.625 * dewpoint_c / (243.04 + dewpoint_c)
        - 17.625 * temperature_c / (243.04 + temperature_c)
    )


def stull_wet_bulb_temperature(
    temperature_c: Any, relative_humidity_percent: Any, np: Any
) -> Any:
    humidity = np.asarray(relative_humidity_percent, dtype=float)
    temperature = np.asarray(temperature_c, dtype=float)
    return (
        temperature * np.arctan(0.151977 * np.sqrt(humidity + 8.313659))
        + np.arctan(temperature + humidity)
        - np.arctan(humidity - 1.676331)
        + 0.00391838 * humidity**1.5 * np.arctan(0.023101 * humidity)
        - 4.686035
    )


def shade_apparent_temperature(
    temperature_c: Any, dewpoint_c: Any, wind_speed_m_s: Any, np: Any
) -> Any:
    vapour_pressure_hpa = 6.105 * np.exp(
        17.27
        * np.asarray(dewpoint_c, dtype=float)
        / (237.7 + np.asarray(dewpoint_c, dtype=float))
    )
    return (
        np.asarray(temperature_c, dtype=float)
        + 0.33 * vapour_pressure_hpa
        - 0.70 * np.asarray(wind_speed_m_s, dtype=float)
        - 4.00
    )


def daily_maximum_with_context(
    metric: Any, contexts: Dict[str, Any], np: Any
) -> Dict[str, Any]:
    metric_matrix = np.asarray(metric, dtype=float).reshape(-1, 24)
    finite = np.isfinite(metric_matrix)
    valid_hours = finite.sum(axis=1)
    safe = np.where(finite, metric_matrix, -np.inf)
    hour = np.argmax(safe, axis=1)
    maximum = np.take_along_axis(safe, hour[:, None], axis=1).reshape(-1)
    no_valid_hour = valid_hours == 0
    maximum[no_valid_hour] = np.nan
    result: Dict[str, Any] = {
        "maximum": maximum,
        "hour_utc": np.where(no_valid_hour, -1, hour),
        "valid_hours": valid_hours,
    }
    for name, values in contexts.items():
        matrix = np.asarray(values, dtype=float).reshape(-1, 24)
        selected = np.take_along_axis(matrix, hour[:, None], axis=1).reshape(-1)
        selected[no_valid_hour] = np.nan
        result[name] = selected
    return result


def haversine_distance_km(
    longitude_a: float, latitude_a: float, longitude_b: float, latitude_b: float
) -> float:
    radius_km = 6371.0088
    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)
    delta_latitude = latitude_b_rad - latitude_a_rad
    delta_longitude = math.radians(longitude_b - longitude_a)
    haversine = math.sin(delta_latitude / 2.0) ** 2 + (
        math.cos(latitude_a_rad)
        * math.cos(latitude_b_rad)
        * math.sin(delta_longitude / 2.0) ** 2
    )
    return radius_km * 2.0 * math.asin(min(1.0, math.sqrt(haversine)))


def observation_value(value: Any, field: str, row_number: int) -> Optional[float]:
    text = "" if value is None else str(value).strip()
    if not text or text.lower() in {"nan", "na", "null"}:
        return None
    try:
        parsed = float(text)
    except ValueError as exc:
        raise ContractError(
            f"Observation row {row_number} field {field} is not numeric: {value!r}"
        ) from exc
    if not math.isfinite(parsed):
        raise ContractError(
            f"Observation row {row_number} field {field} is not finite: {value!r}"
        )
    return parsed


def load_reference_grid_point(study: Dict[str, Any], site_id: str) -> Dict[str, float]:
    audit_path = (
        REPO_ROOT
        / study["data_root"]
        / "provenance"
        / "era5_land_reference_period_audit.json"
    )
    audit = load_json(audit_path)
    if audit.get("valid") is not True:
        raise ContractError(f"ERA5-Land reference audit is not valid: {audit_path}")
    for record in audit.get("sites", []):
        if record.get("site", {}).get("id") == site_id:
            point = record.get("selected_grid_point", {})
            try:
                return {
                    "longitude": float(point["longitude"]),
                    "latitude": float(point["latitude"]),
                }
            except (KeyError, TypeError, ValueError) as exc:
                raise ContractError(
                    f"ERA5-Land audit has no valid grid point for {site_id}"
                ) from exc
    raise ContractError(f"ERA5-Land audit has no site record for {site_id}")


def validate_observation_table(
    file_path: Path,
    source_id: str,
    site_id: str,
    study: Dict[str, Any],
    definition: Dict[str, Any],
    provenance: Dict[str, Any],
    reference_grid: Dict[str, float],
) -> Dict[str, Any]:
    errors: List[str] = []
    required_provenance = definition["licensing"]["required_provenance"]
    for field in required_provenance:
        if field not in provenance or provenance[field] in (None, "", []):
            errors.append(f"Missing observation provenance field: {field}")
    file_digest = sha256_file(file_path)
    if provenance.get("file_sha256") != file_digest:
        errors.append("Observation provenance file_sha256 does not match the CSV")
    if provenance.get("source_id") != source_id:
        errors.append(
            f"Observation provenance source_id is {provenance.get('source_id')!r}, expected {source_id!r}"
        )
    if provenance.get("target_site") != site_id:
        errors.append(
            f"Observation provenance target_site is {provenance.get('target_site')!r}, expected {site_id!r}"
        )
    if source_id not in definition["permitted_sources"]:
        errors.append(f"Observation source is not permitted: {source_id}")
    if provenance.get("canonical_units") != definition["canonical_units"]:
        errors.append(
            "Observation provenance canonical_units do not match the contract"
        )
    try:
        parse_iso_day(provenance.get("download_date"), "provenance.download_date")
    except ContractError as exc:
        errors.append(str(exc))
    approved_raw = provenance.get("approved_quality_flags", [])
    if not isinstance(approved_raw, list):
        errors.append("Observation provenance approved_quality_flags must be an array")
        approved_raw = []
    approved_flags = {
        str(value).strip() for value in approved_raw if str(value).strip()
    }
    if not approved_flags:
        errors.append("Observation provenance has no approved quality flags")
    meanings = provenance.get("quality_flag_meanings", {})
    if not isinstance(meanings, dict) or any(
        flag not in meanings for flag in approved_flags
    ):
        errors.append("Every approved quality flag requires a documented meaning")
    transformation_history = provenance.get("transformation_history", [])
    if not isinstance(transformation_history, list) or not all(
        isinstance(value, str) and value.strip() for value in transformation_history
    ):
        errors.append(
            "Observation provenance transformation_history must be an array of non-empty strings"
        )

    contract = definition["observation_table_contract"]
    required_columns = set(contract["required_columns"])
    required_variables = list(definition["required_variables"])
    optional_variables = list(definition["optional_variables"])
    rows_by_date: Dict[date, Dict[str, Any]] = {}
    duplicate_dates: List[str] = []
    location_ids = set()
    metadata_rows: List[Tuple[float, float, float]] = []
    flag_counts: Dict[str, int] = {}
    with file_path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        columns = set(reader.fieldnames or [])
        missing_columns = sorted(required_columns - columns)
        if missing_columns:
            errors.append(
                f"Observation table lacks required columns: {missing_columns}"
            )
        available_variables = [
            variable
            for variable in [*required_variables, *optional_variables]
            if variable in columns
        ]
        for row_number, row in enumerate(reader, start=2):
            try:
                observed_date = parse_iso_day(row.get("date"), f"row {row_number} date")
                longitude = observation_value(
                    row.get("longitude"), "longitude", row_number
                )
                latitude = observation_value(
                    row.get("latitude"), "latitude", row_number
                )
                elevation = observation_value(
                    row.get("elevation_m"), "elevation_m", row_number
                )
                if longitude is None or latitude is None or elevation is None:
                    raise ContractError(
                        f"Observation row {row_number} has incomplete location metadata"
                    )
                if not -180.0 <= longitude <= 180.0 or not -90.0 <= latitude <= 90.0:
                    raise ContractError(
                        f"Observation row {row_number} has invalid coordinates"
                    )
                parsed_values = {
                    variable: observation_value(row.get(variable), variable, row_number)
                    for variable in available_variables
                }
            except ContractError as exc:
                errors.append(str(exc))
                continue
            if observed_date in rows_by_date:
                duplicate_dates.append(observed_date.isoformat())
                continue
            location_id = str(row.get("location_id", "")).strip()
            location_ids.add(location_id)
            metadata_rows.append((longitude, latitude, elevation))
            quality_flag = str(row.get("quality_flag", "")).strip()
            flag_counts[quality_flag] = flag_counts.get(quality_flag, 0) + 1
            rows_by_date[observed_date] = {
                "quality_flag": quality_flag,
                "values": parsed_values,
            }
    if duplicate_dates:
        errors.append(
            f"Observation table has {len(duplicate_dates)} duplicate dates; first is {duplicate_dates[0]}"
        )
    if len(location_ids) != 1 or "" in location_ids:
        errors.append(
            f"Observation table must contain exactly one non-empty location_id, found {sorted(location_ids)}"
        )
    location_id = next(iter(location_ids), None) if len(location_ids) == 1 else None
    declared_raw = provenance.get("station_or_grid_ids", [])
    if not isinstance(declared_raw, list):
        errors.append("Observation provenance station_or_grid_ids must be an array")
        declared_raw = []
    declared_ids = {str(value) for value in declared_raw}
    if location_id and location_id not in declared_ids:
        errors.append(
            "Observation location_id is absent from provenance station_or_grid_ids"
        )
    if metadata_rows:
        longitude, latitude, elevation = metadata_rows[0]
        if any(
            not (
                math.isclose(row[0], longitude, abs_tol=1e-8)
                and math.isclose(row[1], latitude, abs_tol=1e-8)
                and math.isclose(row[2], elevation, abs_tol=1e-6)
            )
            for row in metadata_rows[1:]
        ):
            errors.append("Observation location metadata changes within the file")
    else:
        longitude = latitude = elevation = float("nan")
        errors.append("Observation table has no readable data rows")

    distance_km = (
        haversine_distance_km(
            longitude,
            latitude,
            float(reference_grid["longitude"]),
            float(reference_grid["latitude"]),
        )
        if math.isfinite(longitude) and math.isfinite(latitude)
        else float("nan")
    )
    maximum_distance = float(
        definition["spatial_matching"]["maximum_station_to_grid_distance_km"]
    )
    if math.isfinite(distance_km) and distance_km > maximum_distance:
        errors.append(
            f"Observation-to-ERA5 grid distance {distance_km:.3f} km exceeds {maximum_distance:.3f} km"
        )
    try:
        reference_elevation = float(provenance["reference_grid_elevation_m"])
    except (KeyError, TypeError, ValueError):
        reference_elevation = float("nan")
    if not math.isfinite(reference_elevation):
        errors.append("Observation provenance reference_grid_elevation_m is not finite")
    elevation_difference = (
        elevation - reference_elevation
        if math.isfinite(elevation) and math.isfinite(reference_elevation)
        else float("nan")
    )

    preferred = definition["preferred_overlap_period"]
    preferred_start = parse_iso_day(preferred["start"], "preferred_overlap.start")
    preferred_end = parse_iso_day(preferred["end"], "preferred_overlap.end")
    dates = sorted(rows_by_date)
    if dates:
        start_year = max(preferred_start.year, dates[0].year)
        end_year = min(preferred_end.year, dates[-1].year)
    else:
        start_year, end_year = 1, 0
    overlap_years = max(0, end_year - start_year + 1)
    expected_dates: List[date] = []
    if overlap_years:
        cursor = date(start_year, 1, 1)
        end_cursor = date(end_year, 12, 31)
        while cursor <= end_cursor:
            expected_dates.append(cursor)
            cursor += timedelta(days=1)
    seasons = {
        "DJF": {12, 1, 2},
        "MAM": {3, 4, 5},
        "JJA": {6, 7, 8},
        "SON": {9, 10, 11},
    }
    gates = definition["coverage_gates"]
    overall_limit = float(gates["maximum_missing_fraction_overall"])
    seasonal_limit = float(gates["maximum_missing_fraction_per_season"])
    annual_limit = float(gates["maximum_missing_fraction_per_year"])

    def has_value(observed_date: date, variable: str) -> bool:
        row = rows_by_date.get(observed_date)
        return bool(
            row
            and row["quality_flag"] in approved_flags
            and row["values"].get(variable) is not None
        )

    coverage: Dict[str, Any] = {}
    variables_to_report = [
        variable
        for variable in [*required_variables, *optional_variables]
        if any(variable in row["values"] for row in rows_by_date.values())
    ]
    for variable in variables_to_report:
        total = len(expected_dates)
        valid = sum(has_value(value, variable) for value in expected_dates)
        overall_missing = 1.0 - valid / total if total else 1.0
        seasonal_missing = {}
        for season, months in seasons.items():
            selected = [value for value in expected_dates if value.month in months]
            season_valid = sum(has_value(value, variable) for value in selected)
            seasonal_missing[season] = (
                1.0 - season_valid / len(selected) if selected else 1.0
            )
        coverage[variable] = {
            "valid_days": valid,
            "expected_days": total,
            "missing_fraction_overall": overall_missing,
            "missing_fraction_by_season": seasonal_missing,
        }
        if variable in required_variables and overall_missing > overall_limit:
            errors.append(
                f"{variable} overall missing fraction {overall_missing:.6g} exceeds {overall_limit:.6g}"
            )
        for season, fraction in seasonal_missing.items():
            if variable in required_variables and fraction > seasonal_limit:
                errors.append(
                    f"{variable} {season} missing fraction {fraction:.6g} exceeds {seasonal_limit:.6g}"
                )
    for variable in required_variables:
        if variable not in variables_to_report:
            errors.append(f"Required observation variable is unavailable: {variable}")

    valid_years = []
    annual_coverage: Dict[str, Any] = {}
    for year in range(start_year, end_year + 1):
        selected = [value for value in expected_dates if value.year == year]
        fractions = {
            variable: 1.0
            - sum(has_value(value, variable) for value in selected) / len(selected)
            for variable in required_variables
        }
        annual_coverage[str(year)] = fractions
        if all(value <= annual_limit for value in fractions.values()):
            valid_years.append(year)
    if len(valid_years) < int(definition["minimum_overlap_years"]):
        errors.append(
            f"Only {len(valid_years)} calendar years pass annual coverage; "
            f"at least {definition['minimum_overlap_years']} are required"
        )

    approved_rows = [
        (observed_date, row)
        for observed_date, row in rows_by_date.items()
        if start_year <= observed_date.year <= end_year
        and row["quality_flag"] in approved_flags
    ]
    negative_precipitation = sum(
        row["values"].get("pr") is not None and row["values"]["pr"] < 0.0
        for _, row in approved_rows
    )
    temperature_order_violations = sum(
        row["values"].get("tasmin") is not None
        and row["values"].get("tasmax") is not None
        and row["values"]["tasmin"] > row["values"]["tasmax"]
        for _, row in approved_rows
    )
    if negative_precipitation:
        errors.append(
            f"Observation table has {negative_precipitation} approved negative precipitation values"
        )
    if temperature_order_violations:
        errors.append(
            f"Observation table has {temperature_order_violations} approved days with tasmin > tasmax"
        )
    undocumented_flags = (
        sorted(set(flag_counts) - set(meanings))
        if isinstance(meanings, dict)
        else sorted(flag_counts)
    )
    if undocumented_flags:
        errors.append(
            f"Observation table contains undocumented quality flags: {undocumented_flags}"
        )
    wet_days = sum(
        row["values"].get("pr") is not None and row["values"]["pr"] >= 1.0
        for _, row in approved_rows
    )
    if wet_days < int(gates["minimum_valid_wet_days_for_precipitation_quantiles"]):
        errors.append(
            f"Only {wet_days} approved wet days are available for precipitation quantiles"
        )

    return {
        "schema_version": "1.0",
        "validation_id": definition["validation_id"],
        "valid": not errors,
        "status": "observation_admission_passed"
        if not errors
        else "observation_admission_failed",
        "source_id": source_id,
        "target_site": site_id,
        "file": path_text(file_path),
        "file_sha256": file_digest,
        "location": {
            "location_id": location_id,
            "longitude": longitude if math.isfinite(longitude) else None,
            "latitude": latitude if math.isfinite(latitude) else None,
            "elevation_m": elevation if math.isfinite(elevation) else None,
            "era5_grid_longitude": float(reference_grid["longitude"]),
            "era5_grid_latitude": float(reference_grid["latitude"]),
            "distance_to_era5_grid_km": distance_km
            if math.isfinite(distance_km)
            else None,
            "reference_grid_elevation_m": reference_elevation
            if math.isfinite(reference_elevation)
            else None,
            "elevation_minus_reference_grid_m": elevation_difference
            if math.isfinite(elevation_difference)
            else None,
        },
        "time": {
            "evaluation_start": f"{start_year:04d}-01-01" if overlap_years else None,
            "evaluation_end": f"{end_year:04d}-12-31" if overlap_years else None,
            "calendar_years": overlap_years,
            "years_passing_annual_coverage": valid_years,
        },
        "quality_flags": {
            "approved": sorted(approved_flags),
            "observed_counts": flag_counts,
        },
        "coverage": coverage,
        "annual_missing_fraction": annual_coverage,
        "wet_days": wet_days,
        "cross_variable_checks": {
            "negative_precipitation_values": negative_precipitation,
            "tasmin_above_tasmax_days": temperature_order_violations,
        },
        "provenance": provenance,
        "errors": errors,
        "interpretation_limit": "Admission checks data fitness only; they do not constitute ERA5 bias evaluation or climate attribution.",
    }


def command_qc_observations(args: argparse.Namespace) -> int:
    study = load_json(args.study)
    definition = load_json(args.validation_contract)
    provenance = load_json(args.provenance)
    reference_grid = load_reference_grid_point(study, args.site)
    report = validate_observation_table(
        args.file,
        args.source,
        args.site,
        study,
        definition,
        provenance,
        reference_grid,
    )
    report["checked_utc"] = datetime.now(timezone.utc).isoformat()
    if args.output:
        write_json(args.output, report, args.overwrite)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


def command_derive_era5_heat_stress(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "derive-era5-heat-stress requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    sources = source_index(load_json(args.sources))
    definition = load_json(args.heat_stress_definition)
    reference = study["periods"]["reference"]
    reference_start = int(reference["start"][:4])
    reference_end = int(reference["end"][:4])
    start_year = args.start_year or reference_start
    end_year = args.end_year or reference_end
    if start_year > end_year or not (
        reference_start <= start_year <= end_year <= reference_end
    ):
        raise ContractError("Heat-stress range must lie inside the reference period")
    period_id = f"{start_year}-{end_year}"
    source = sources["era5_land"]
    data_root = REPO_ROOT / study["data_root"]
    sites = list(iter_sites(study))
    if args.site:
        sites = [site for site in sites if site["id"] == args.site]
    output_paths = {
        site["id"]: data_root
        / "processed"
        / "daily"
        / "era5_land_heat_stress"
        / f"site={site['id']}"
        / f"reference_period={period_id}"
        / f"daily_era5_land_heat_stress_{site['id']}_{period_id.replace('-', '_')}.nc"
        for site in sites
    }
    existing = [path for path in output_paths.values() if path.exists()]
    existing.extend(
        path.with_suffix(path.suffix + ".provenance.json")
        for path in output_paths.values()
        if path.with_suffix(path.suffix + ".provenance.json").exists()
    )
    if existing and not args.overwrite:
        raise ContractError(
            "Refusing to overwrite heat-stress outputs: "
            + ", ".join(path_text(path) for path in existing)
        )
    wet_bulb_definition = definition["wet_bulb_temperature"]
    temperature_limits = [
        float(value) for value in wet_bulb_definition["valid_air_temperature_degC"]
    ]
    humidity_limits = [
        float(value) for value in wet_bulb_definition["valid_relative_humidity_percent"]
    ]
    records = []
    for site in sites:
        annual_datasets = []
        inputs = []
        selected_latitude = None
        selected_longitude = None
        total_hours = 0
        valid_wet_bulb_hours = 0
        rh_clipped_count = 0
        try:
            for year in range(start_year, end_year + 1):
                group_paths = {
                    "temperature_humidity": data_root
                    / "raw"
                    / "era5_land"
                    / f"site={site['id']}"
                    / f"year={year}"
                    / f"era5_land_timeseries_{site['id']}_{year}_temperature_humidity.nc",
                    "wind": data_root
                    / "raw"
                    / "era5_land"
                    / f"site={site['id']}"
                    / f"year={year}"
                    / f"era5_land_timeseries_{site['id']}_{year}_wind.nc",
                }
                arrays: Dict[str, Any] = {}
                reference_time = None
                for group, path in group_paths.items():
                    if not path.exists():
                        raise ContractError(
                            f"Missing ERA5-Land heat-stress input: {path}"
                        )
                    with xr.open_dataset(path) as dataset:
                        time_name = coordinate_name(dataset, ("time", "valid_time"))
                        if not time_name:
                            raise ContractError(f"{path} lacks a time coordinate")
                        time_values = np.asarray(dataset[time_name].values).astype(
                            "datetime64[ns]"
                        )
                        if reference_time is None:
                            reference_time = time_values
                        elif not np.array_equal(reference_time, time_values):
                            raise ContractError(
                                f"Temperature/humidity and wind axes differ for {site['id']} {year}"
                            )
                        latitude_name = coordinate_name(dataset, ("latitude", "lat"))
                        longitude_name = coordinate_name(dataset, ("longitude", "lon"))
                        if not latitude_name or not longitude_name:
                            raise ContractError(f"{path} lacks latitude/longitude")
                        latitude = float(
                            np.asarray(dataset[latitude_name].values).reshape(-1)[0]
                        )
                        longitude = normalize_longitude(
                            float(
                                np.asarray(dataset[longitude_name].values).reshape(-1)[
                                    0
                                ]
                            )
                        )
                        if selected_latitude is None:
                            selected_latitude = latitude
                            selected_longitude = longitude
                        elif not (
                            math.isclose(selected_latitude, latitude, abs_tol=1e-9)
                            and math.isclose(
                                float(selected_longitude), longitude, abs_tol=1e-9
                            )
                        ):
                            raise ContractError(
                                f"ERA5-Land grid cell changes for {site['id']} {year}"
                            )
                        request_variables = (
                            ["2m_temperature", "2m_dewpoint_temperature"]
                            if group == "temperature_humidity"
                            else ["10m_u_component_of_wind", "10m_v_component_of_wind"]
                        )
                        for request_name in request_variables:
                            spec = source["variables"][request_name]
                            matches = [
                                name
                                for name in spec["netcdf_names"]
                                if name in dataset.data_vars
                            ]
                            if len(matches) != 1:
                                raise ContractError(
                                    f"Expected one {request_name} variable in {path}, found {matches}"
                                )
                            array = dataset[matches[0]]
                            units = str(array.attrs.get("units", ""))
                            if units not in spec["accepted_units"]:
                                raise ContractError(
                                    f"{request_name} units {units!r} are unsupported"
                                )
                            values = np.asarray(array.values, dtype=float).reshape(-1)
                            if values.size != time_values.size:
                                raise ContractError(
                                    f"{request_name} value/time length mismatch"
                                )
                            arrays[request_name] = values
                    inputs.append(
                        {
                            "year": year,
                            "group": group,
                            "path": path_text(path),
                            "sha256": sha256_file(path),
                        }
                    )
                expected_time = np.arange(
                    np.datetime64(f"{year}-01-01T00:00"),
                    np.datetime64(f"{year + 1}-01-01T00:00"),
                    np.timedelta64(1, "h"),
                ).astype("datetime64[ns]")
                if reference_time is None or not np.array_equal(
                    reference_time, expected_time
                ):
                    raise ContractError(
                        f"Incomplete ERA5-Land hourly axis for {site['id']} {year}"
                    )
                temperature_c = arrays["2m_temperature"] - 273.15
                dewpoint_c = arrays["2m_dewpoint_temperature"] - 273.15
                relative_humidity = relative_humidity_from_temperature_dewpoint(
                    temperature_c, dewpoint_c, np
                )
                rh_clipped_count += int(
                    np.count_nonzero(
                        (relative_humidity < 0.0) | (relative_humidity > 100.0)
                    )
                )
                relative_humidity = np.clip(relative_humidity, 0.0, 100.0)
                wind_speed = np.hypot(
                    arrays["10m_u_component_of_wind"],
                    arrays["10m_v_component_of_wind"],
                )
                valid_stull = (
                    (temperature_c >= temperature_limits[0])
                    & (temperature_c <= temperature_limits[1])
                    & (relative_humidity >= humidity_limits[0])
                    & (relative_humidity <= humidity_limits[1])
                )
                wet_bulb = np.where(
                    valid_stull,
                    stull_wet_bulb_temperature(temperature_c, relative_humidity, np),
                    np.nan,
                )
                apparent_temperature = shade_apparent_temperature(
                    temperature_c, dewpoint_c, wind_speed, np
                )
                wet_bulb_daily = daily_maximum_with_context(
                    wet_bulb,
                    {
                        "temperature": temperature_c,
                        "relative_humidity": relative_humidity,
                        "wind_speed": wind_speed,
                    },
                    np,
                )
                apparent_daily = daily_maximum_with_context(
                    apparent_temperature,
                    {
                        "temperature": temperature_c,
                        "relative_humidity": relative_humidity,
                        "wind_speed": wind_speed,
                    },
                    np,
                )
                temperature_daily = daily_maximum_with_context(
                    temperature_c,
                    {
                        "relative_humidity": relative_humidity,
                        "wind_speed": wind_speed,
                    },
                    np,
                )
                total_hours += int(expected_time.size)
                valid_wet_bulb_hours += int(np.count_nonzero(valid_stull))
                daily_time = np.arange(
                    np.datetime64(f"{year}-01-01T12:00"),
                    np.datetime64(f"{year + 1}-01-01T12:00"),
                    np.timedelta64(1, "D"),
                ).astype("datetime64[ns]")
                annual_datasets.append(
                    xr.Dataset(
                        data_vars={
                            "twmax": (
                                "time",
                                wet_bulb_daily["maximum"].astype("float32"),
                                {"units": "degC"},
                            ),
                            "twmax_hour_utc": (
                                "time",
                                wet_bulb_daily["hour_utc"].astype("int8"),
                                {"units": "hour"},
                            ),
                            "valid_stull_hours": (
                                "time",
                                wet_bulb_daily["valid_hours"].astype("int8"),
                                {"units": "hour"},
                            ),
                            "tas_at_twmax": (
                                "time",
                                wet_bulb_daily["temperature"].astype("float32"),
                                {"units": "degC"},
                            ),
                            "hurs_at_twmax": (
                                "time",
                                wet_bulb_daily["relative_humidity"].astype("float32"),
                                {"units": "%"},
                            ),
                            "sfcWind_at_twmax": (
                                "time",
                                wet_bulb_daily["wind_speed"].astype("float32"),
                                {"units": "m s-1"},
                            ),
                            "atmax": (
                                "time",
                                apparent_daily["maximum"].astype("float32"),
                                {"units": "degC"},
                            ),
                            "atmax_hour_utc": (
                                "time",
                                apparent_daily["hour_utc"].astype("int8"),
                                {"units": "hour"},
                            ),
                            "tas_at_atmax": (
                                "time",
                                apparent_daily["temperature"].astype("float32"),
                                {"units": "degC"},
                            ),
                            "hurs_at_atmax": (
                                "time",
                                apparent_daily["relative_humidity"].astype("float32"),
                                {"units": "%"},
                            ),
                            "sfcWind_at_atmax": (
                                "time",
                                apparent_daily["wind_speed"].astype("float32"),
                                {"units": "m s-1"},
                            ),
                            "hurs_at_tasmax": (
                                "time",
                                temperature_daily["relative_humidity"].astype(
                                    "float32"
                                ),
                                {"units": "%"},
                            ),
                            "sfcWind_at_tasmax": (
                                "time",
                                temperature_daily["wind_speed"].astype("float32"),
                                {"units": "m s-1"},
                            ),
                        },
                        coords={"time": daily_time},
                    )
                )
            combined = xr.concat(annual_datasets, dim="time")
            expected_daily_time = np.arange(
                np.datetime64(f"{start_year}-01-01T12:00"),
                np.datetime64(f"{end_year + 1}-01-01T12:00"),
                np.timedelta64(1, "D"),
            ).astype("datetime64[ns]")
            if not np.array_equal(
                np.asarray(combined["time"].values).astype("datetime64[ns]"),
                expected_daily_time,
            ):
                raise ContractError(
                    f"Heat-stress daily axis is incomplete for {site['id']}"
                )
            month_values = np.asarray(
                [int(str(value)[:7].split("-")[1]) for value in expected_daily_time]
            )
            warm_season = np.isin(
                month_values, definition["humid_heat_event"]["season_months"]
            )
            warm_missing = float(
                np.isnan(np.asarray(combined["twmax"].values)[warm_season]).mean()
            )
            if warm_missing > study["quality_gates"]["maximum_missing_fraction"]:
                raise ContractError(
                    f"{site['id']} warm-season twmax missing fraction {warm_missing:.6g} exceeds gate"
                )
            combined = combined.assign_coords(
                latitude=selected_latitude,
                longitude=selected_longitude,
                site=site["id"],
                zone=site["zone_id"],
            )
            combined.attrs.update(
                {
                    "study_id": study["study_id"],
                    "source_dataset": ERA5_TIMESERIES_DATASET_ID,
                    "derivation_id": definition["derivation_id"],
                    "reference_period": period_id,
                    "aggregation_day": "UTC",
                    "probability_semantics": "historical_reanalysis_no_scenario_probability",
                    "wet_bulb_method": wet_bulb_definition["method"],
                    "apparent_temperature_method": definition["apparent_temperature"][
                        "method"
                    ],
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                }
            )
            output = output_paths[site["id"]]
            output.parent.mkdir(parents=True, exist_ok=True)
            combined.to_netcdf(
                output,
                encoding={
                    variable: {"zlib": True, "complevel": 4}
                    for variable in combined.data_vars
                },
            )
            combined.close()
            record = {
                "site": site,
                "selected_grid_point": {
                    "longitude": selected_longitude,
                    "latitude": selected_latitude,
                },
                "period": period_id,
                "days": len(expected_daily_time),
                "hours": total_hours,
                "valid_stull_hours": valid_wet_bulb_hours,
                "outside_stull_domain_hours": total_hours - valid_wet_bulb_hours,
                "relative_humidity_values_clipped_to_0_100": rh_clipped_count,
                "warm_season_twmax_missing_fraction": warm_missing,
                "inputs": inputs,
                "output": path_text(output),
                "output_sha256": sha256_file(output),
                "definition": definition,
            }
            provenance_path = output.with_suffix(output.suffix + ".provenance.json")
            write_json(
                provenance_path,
                {
                    "schema_version": "1.0",
                    "operation": "era5_land_concurrent_hourly_heat_stress_derivation",
                    **record,
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                },
                args.overwrite,
            )
            record["provenance"] = path_text(provenance_path)
            records.append(record)
        finally:
            for dataset in annual_datasets:
                dataset.close()
    summary = {
        "derivation_id": definition["derivation_id"],
        "status": definition["source_role"],
        "period": period_id,
        "sites_completed": len(records),
        "records": records,
        "independent_observation_validation_completed": False,
    }
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def command_diagnose_era5_reference(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "diagnose-era5-reference requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    definition = load_json(args.diagnostics)
    heat_stress_definition = load_json(args.heat_stress_definition)
    reference = study["periods"]["reference"]
    if definition.get("reference_period") != {
        "start": reference["start"],
        "end": reference["end"],
    }:
        raise ContractError(
            "Historical diagnostic period does not match the study contract"
        )
    percentile = definition["percentile_method"]
    humid_heat_definition = heat_stress_definition["humid_heat_event"]
    humid_heat_months = [int(value) for value in humid_heat_definition["season_months"]]
    window_days = int(percentile["window_days"])
    absolute = definition["absolute_thresholds"]
    wet_day_mm = float(absolute["wet_day_mm"])
    start_date = parse_iso_day(reference["start"], "periods.reference.start")
    end_date = parse_iso_day(reference["end"], "periods.reference.end")
    if (start_date.month, start_date.day) != (1, 1) or (
        end_date.month,
        end_date.day,
    ) != (12, 31):
        raise ContractError("Historical diagnostics require complete calendar years")
    start_year = start_date.year
    end_year = end_date.year
    period_id = f"{start_year}-{end_year}"
    data_root = REPO_ROOT / study["data_root"]
    output_root = (
        args.output_dir
        or data_root / "processed" / "diagnostics" / "era5_land_reference"
    )
    annual_csv = (
        output_root / f"era5_land_annual_indices_{period_id.replace('-', '_')}.csv"
    )
    report_path = (
        data_root
        / "provenance"
        / "diagnostics"
        / f"era5_land_reference_diagnostics_{period_id.replace('-', '_')}.json"
    )
    threshold_paths = {
        site["id"]: output_root
        / "thresholds"
        / f"era5_land_{site['id']}_{period_id}_calendar_day_thresholds.nc"
        for site in iter_sites(study)
    }
    existing_outputs = [
        path
        for path in [*threshold_paths.values(), annual_csv, report_path]
        if path.exists()
    ]
    if existing_outputs and not args.overwrite:
        rendered = ", ".join(path_text(path) for path in existing_outputs)
        raise ContractError(f"Refusing to overwrite diagnostic outputs: {rendered}")
    output_root.mkdir(parents=True, exist_ok=True)
    annual_rows: List[Dict[str, Any]] = []
    site_reports = []
    for site in iter_sites(study):
        input_path = (
            data_root
            / "processed"
            / "daily"
            / "era5_land"
            / f"site={site['id']}"
            / f"reference_period={period_id}"
            / f"daily_era5_land_{site['id']}_{period_id.replace('-', '_')}.nc"
        )
        if not input_path.exists():
            raise ContractError(
                f"Missing combined ERA5-Land reference file: {input_path}"
            )
        with xr.open_dataset(input_path) as dataset:
            time_values = np.asarray(dataset["time"].values).astype("datetime64[D]")
            date_strings = np.datetime_as_string(time_values, unit="D")
            expected_dates = np.arange(
                np.datetime64(reference["start"]),
                np.datetime64(reference["end"]) + np.timedelta64(1, "D"),
                dtype="datetime64[D]",
            )
            if not np.array_equal(time_values, expected_dates):
                raise ContractError(
                    f"{input_path} does not contain the exact continuous reference-period daily axis"
                )
            years = np.asarray([int(value[:4]) for value in date_strings], dtype=int)
            months = np.asarray([int(value[5:7]) for value in date_strings], dtype=int)
            days = np.asarray([int(value[8:10]) for value in date_strings], dtype=int)
            day_indices = np.asarray(noleap_day_indices(months, days), dtype=int)
            arrays = {
                variable: np.asarray(dataset[variable].values, dtype=float)
                for variable in ("tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr")
            }
            for variable, values in arrays.items():
                if values.ndim != 1 or values.size != time_values.size:
                    raise ContractError(
                        f"{input_path} variable {variable} is not a complete one-dimensional daily series"
                    )
                if not np.all(np.isfinite(values)):
                    raise ContractError(
                        f"{input_path} variable {variable} contains missing values"
                    )
            heat_stress_path = (
                data_root
                / "processed"
                / "daily"
                / "era5_land_heat_stress"
                / f"site={site['id']}"
                / f"reference_period={period_id}"
                / f"daily_era5_land_heat_stress_{site['id']}_{period_id.replace('-', '_')}.nc"
            )
            if not heat_stress_path.exists():
                raise ContractError(
                    f"Missing concurrent hourly heat-stress product: {heat_stress_path}"
                )
            with xr.open_dataset(heat_stress_path) as heat_dataset:
                heat_time = np.asarray(heat_dataset["time"].values).astype(
                    "datetime64[D]"
                )
                if not np.array_equal(heat_time, time_values):
                    raise ContractError(
                        f"Heat-stress and canonical daily axes differ for {site['id']}"
                    )
                if (
                    heat_dataset.attrs.get("derivation_id")
                    != heat_stress_definition["derivation_id"]
                ):
                    raise ContractError(
                        f"Heat-stress derivation id mismatch for {site['id']}"
                    )
                heat_variables = (
                    "twmax",
                    "atmax",
                    "tas_at_twmax",
                    "hurs_at_twmax",
                    "sfcWind_at_twmax",
                    "hurs_at_tasmax",
                    "sfcWind_at_tasmax",
                )
                for variable in heat_variables:
                    if variable not in heat_dataset:
                        raise ContractError(
                            f"{heat_stress_path} lacks heat-stress variable {variable}"
                        )
                    values = np.asarray(heat_dataset[variable].values, dtype=float)
                    if values.ndim != 1 or values.size != time_values.size:
                        raise ContractError(
                            f"{heat_stress_path} variable {variable} has an invalid shape"
                        )
                    arrays[variable] = values
            thresholds = {
                "tasmax_q90": calendar_day_percentile_thresholds(
                    arrays["tasmax"],
                    day_indices,
                    months,
                    days,
                    float(percentile["temperature_hot_quantile"]),
                    window_days,
                    np,
                ),
                "tasmin_q90": calendar_day_percentile_thresholds(
                    arrays["tasmin"],
                    day_indices,
                    months,
                    days,
                    float(percentile["warm_night_quantile"]),
                    window_days,
                    np,
                ),
                "sfcWind_q10": calendar_day_percentile_thresholds(
                    arrays["sfcWind"],
                    day_indices,
                    months,
                    days,
                    float(percentile["low_wind_quantile"]),
                    window_days,
                    np,
                ),
            }
            humid_heat_selected = np.isin(months, humid_heat_months)
            humid_heat_values = arrays["twmax"][humid_heat_selected]
            humid_heat_values = humid_heat_values[np.isfinite(humid_heat_values)]
            if not humid_heat_values.size:
                raise ContractError(
                    f"No valid warm-season twmax values for {site['id']}"
                )
            humid_heat_q90 = float(
                np.quantile(
                    humid_heat_values,
                    float(percentile["humid_heat_quantile"]),
                    method="linear",
                )
            )
            wet_values = arrays["pr"][arrays["pr"] >= wet_day_mm]
            if wet_values.size == 0:
                raise ContractError(f"No wet days found for {site['id']}")
            wet_day_q95 = float(
                np.quantile(
                    wet_values,
                    float(percentile["wet_day_precipitation_quantile"]),
                    method="linear",
                )
            )
            threshold_path = threshold_paths[site["id"]]
            threshold_path.parent.mkdir(parents=True, exist_ok=True)
            threshold_dataset = xr.Dataset(
                data_vars={
                    name: ("day_of_year", values.astype("float32"))
                    for name, values in thresholds.items()
                },
                coords={"day_of_year": np.arange(1, 366, dtype=int)},
                attrs={
                    "study_id": study["study_id"],
                    "diagnostic_id": definition["diagnostic_id"],
                    "reference_period": period_id,
                    "window_days": window_days,
                    "february_29_rule": percentile["february_29_rule"],
                    "wet_day_pr_q95_mm": wet_day_q95,
                    "humid_heat_twmax_q90_degC": humid_heat_q90,
                    "humid_heat_season_months": ",".join(
                        str(value) for value in humid_heat_months
                    ),
                    "heat_stress_source_file": path_text(heat_stress_path),
                    "heat_stress_source_sha256": sha256_file(heat_stress_path),
                    "source_file": path_text(input_path),
                    "source_sha256": sha256_file(input_path),
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                },
            )
            threshold_dataset.to_netcdf(
                threshold_path,
                encoding={name: {"zlib": True, "complevel": 4} for name in thresholds},
            )
            threshold_dataset.close()
            mapped = {name: values[day_indices] for name, values in thresholds.items()}
            monthly_climatology = []
            for month in range(1, 13):
                selected = months == month
                row: Dict[str, Any] = {"month": month}
                for variable in ("tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr"):
                    values = arrays[variable]
                    sample = values[selected]
                    row[variable] = {
                        "mean": float(np.mean(sample)),
                        "q05": float(np.quantile(sample, 0.05)),
                        "q50": float(np.quantile(sample, 0.50)),
                        "q95": float(np.quantile(sample, 0.95)),
                    }
                monthly_climatology.append(row)
            site_rows = []
            for year in range(start_year, end_year + 1):
                selected = years == year
                expected_days = 366 if calendar.isleap(year) else 365
                if int(selected.sum()) != expected_days:
                    raise ContractError(
                        f"{input_path} has {int(selected.sum())} days for {year}, expected {expected_days}"
                    )
                tx = arrays["tasmax"][selected]
                tn = arrays["tasmin"][selected]
                rh = arrays["hurs"][selected]
                wind = arrays["sfcWind"][selected]
                solar = arrays["rsds"][selected]
                precipitation = arrays["pr"][selected]
                twmax = arrays["twmax"][selected]
                atmax = arrays["atmax"][selected]
                tas_at_twmax = arrays["tas_at_twmax"][selected]
                hurs_at_twmax = arrays["hurs_at_twmax"][selected]
                wind_at_twmax = arrays["sfcWind_at_twmax"][selected]
                hurs_at_tasmax = arrays["hurs_at_tasmax"][selected]
                wind_at_tasmax = arrays["sfcWind_at_tasmax"][selected]
                year_months = months[selected]
                tx_threshold = mapped["tasmax_q90"][selected]
                tn_threshold = mapped["tasmin_q90"][selected]
                wind_threshold = mapped["sfcWind_q10"][selected]
                hot = tx > tx_threshold
                warm_night = tn > tn_threshold
                low_wind = wind < wind_threshold
                dry = precipitation < float(absolute["dry_day_mm"])
                wet = precipitation >= wet_day_mm
                heatwave_runs = [run for run in boolean_run_lengths(hot) if run[2] >= 3]
                wsdi_runs = [run for run in boolean_run_lengths(hot) if run[2] >= 6]
                humid_heat = np.isin(year_months, humid_heat_months) & (
                    twmax > humid_heat_q90
                )
                humid_heat_runs = [
                    run
                    for run in boolean_run_lengths(humid_heat)
                    if run[2] >= int(humid_heat_definition["minimum_consecutive_days"])
                ]
                humid_heat_days = sum(run[2] for run in humid_heat_runs)
                humid_heat_mask = np.zeros_like(humid_heat, dtype=bool)
                for start, end, _ in humid_heat_runs:
                    humid_heat_mask[start:end] = True
                humid_heat_severity = float(
                    np.sum(twmax[humid_heat_mask] - humid_heat_q90)
                )
                warm_season = np.isin(year_months, humid_heat_months)
                heatwave_days = sum(run[2] for run in heatwave_runs)
                heatwave_severity = sum(
                    float(np.sum(tx[start:end] - tx_threshold[start:end]))
                    for start, end, _ in heatwave_runs
                )
                rx5day = float(
                    np.max(np.convolve(precipitation, np.ones(5), mode="valid"))
                )
                row = {
                    "site": site["id"],
                    "zone": site["zone_id"],
                    "year": year,
                    "days": int(selected.sum()),
                    "tasmax_mean_degC": float(np.mean(tx)),
                    "tasmin_mean_degC": float(np.mean(tn)),
                    "hurs_mean_percent": float(np.mean(rh)),
                    "sfcWind_mean_m_s": float(np.mean(wind)),
                    "rsds_mean_W_m2": float(np.mean(solar)),
                    "pr_total_mm": float(np.sum(precipitation)),
                    "txx_degC": float(np.max(tx)),
                    "tnn_degC": float(np.min(tn)),
                    "rx1day_mm": float(np.max(precipitation)),
                    "rx5day_mm": rx5day,
                    "r10mm_days": int(
                        np.count_nonzero(
                            precipitation >= float(absolute["heavy_precipitation_mm"])
                        )
                    ),
                    "r20mm_days": int(
                        np.count_nonzero(
                            precipitation
                            >= float(absolute["very_heavy_precipitation_mm"])
                        )
                    ),
                    "r95p_total_mm": float(
                        np.sum(precipitation[precipitation > wet_day_q95])
                    ),
                    "tx90p_days": int(np.count_nonzero(hot)),
                    "tx90p_percent": float(np.mean(hot) * 100.0),
                    "tn90p_days": int(np.count_nonzero(warm_night)),
                    "tn90p_percent": float(np.mean(warm_night) * 100.0),
                    "low_wind10p_days": int(np.count_nonzero(low_wind)),
                    "grid_heatwave_events": len(heatwave_runs),
                    "grid_heatwave_days": heatwave_days,
                    "grid_heatwave_max_duration_days": max(
                        (run[2] for run in heatwave_runs), default=0
                    ),
                    "grid_heatwave_degree_days_above_q90": heatwave_severity,
                    "wsdi_days": sum(run[2] for run in wsdi_runs),
                    "cdd_days": maximum_run_length(dry),
                    "cwd_days": maximum_run_length(wet),
                    "hot_dry_days": int(np.count_nonzero(hot & dry)),
                    "hot_dry_low_wind_days": int(
                        np.count_nonzero(hot & dry & low_wind)
                    ),
                    "twmax_warm_season_mean_degC": float(np.mean(twmax[warm_season])),
                    "twx_warm_season_degC": float(np.max(twmax[warm_season])),
                    "atx_warm_season_degC": float(np.max(atmax[warm_season])),
                    "humid_heat_events": len(humid_heat_runs),
                    "humid_heat_days": humid_heat_days,
                    "humid_heat_max_duration_days": max(
                        (run[2] for run in humid_heat_runs), default=0
                    ),
                    "humid_heat_degree_days_above_q90": humid_heat_severity,
                    "humid_heat_tas_mean_degC": masked_mean_or_blank(
                        tas_at_twmax, humid_heat_mask, np
                    ),
                    "humid_heat_hurs_mean_percent": masked_mean_or_blank(
                        hurs_at_twmax, humid_heat_mask, np
                    ),
                    "humid_heat_wind_mean_m_s": masked_mean_or_blank(
                        wind_at_twmax, humid_heat_mask, np
                    ),
                    "hurs_at_tasmax_mean_percent": float(np.mean(hurs_at_tasmax)),
                    "sfcWind_at_tasmax_mean_m_s": float(np.mean(wind_at_tasmax)),
                    "hot_warm_night_days": int(np.count_nonzero(hot & warm_night)),
                }
                annual_rows.append(row)
                site_rows.append(row)
            trend_metrics = {}
            trend_years = [float(row["year"]) for row in site_rows]
            for metric in (
                "tasmax_mean_degC",
                "tasmin_mean_degC",
                "pr_total_mm",
                "txx_degC",
                "tnn_degC",
                "rx1day_mm",
                "rx5day_mm",
                "tx90p_percent",
                "tn90p_percent",
                "grid_heatwave_days",
                "cdd_days",
                "hot_dry_days",
                "hot_dry_low_wind_days",
                "twmax_warm_season_mean_degC",
                "twx_warm_season_degC",
                "atx_warm_season_degC",
                "humid_heat_days",
            ):
                metric_values = [float(row[metric]) for row in site_rows]
                trend_metrics[metric] = {
                    "theil_sen_slope_per_decade": theil_sen_slope(
                        trend_years, metric_values
                    )
                    * 10.0,
                    f"first_decade_mean_{start_year}_{start_year + 9}": float(
                        np.mean(metric_values[:10])
                    ),
                    f"last_decade_mean_{end_year - 9}_{end_year}": float(
                        np.mean(metric_values[-10:])
                    ),
                    "last_minus_first_decade": float(
                        np.mean(metric_values[-10:]) - np.mean(metric_values[:10])
                    ),
                    "inference": "descriptive_only",
                }
            site_reports.append(
                {
                    "site": site,
                    "input": path_text(input_path),
                    "input_sha256": sha256_file(input_path),
                    "threshold_file": path_text(threshold_path),
                    "threshold_sha256": sha256_file(threshold_path),
                    "wet_day_pr_q95_mm": wet_day_q95,
                    "humid_heat_twmax_q90_degC": humid_heat_q90,
                    "heat_stress_input": path_text(heat_stress_path),
                    "heat_stress_input_sha256": sha256_file(heat_stress_path),
                    "monthly_climatology": monthly_climatology,
                    "trend_screening": trend_metrics,
                }
            )
    fieldnames = ["site", "zone", "year", "days"] + sorted(
        set(annual_rows[0]) - {"site", "zone", "year", "days"}
    )
    with annual_csv.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(annual_rows)
    report = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "diagnostic_id": definition["diagnostic_id"],
        "status": definition["source_role"],
        "definition": definition,
        "annual_table": path_text(annual_csv),
        "annual_table_sha256": sha256_file(annual_csv),
        "annual_rows": len(annual_rows),
        "sites": site_reports,
        "independent_observation_validation_completed": False,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    write_json(report_path, report, args.overwrite)
    print(
        json.dumps(
            {
                "diagnostic_id": report["diagnostic_id"],
                "status": report["status"],
                "annual_rows": report["annual_rows"],
                "annual_table": report["annual_table"],
                "report": path_text(report_path),
                "sites": [site["site"]["id"] for site in site_reports],
                "independent_observation_validation_completed": False,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


def command_summarize_daily(args: argparse.Namespace) -> int:
    try:
        import numpy as np  # type: ignore
        import xarray as xr  # type: ignore
    except ImportError as exc:
        raise ContractError(
            "summarize-daily requires xarray, numpy, and a NetCDF backend"
        ) from exc
    study = load_json(args.study)
    thresholds = study["compound_diagnostic_smoke_test"]
    records = []
    state_keys = set()
    for path in args.files:
        with xr.open_dataset(path) as dataset:
            required = {"tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr"}
            missing = sorted(required - set(dataset.data_vars))
            if missing:
                raise ContractError(f"{path} lacks canonical variables {missing}")
            state = {
                "model": dataset.attrs.get("model"),
                "experiment": dataset.attrs.get("experiment"),
                "site": str(np.asarray(dataset.coords.get("site", "")).item()),
                "year": int(str(np.asarray(dataset["time"].values)[0])[:4]),
            }
            state_key = tuple(state.values())
            if state_key in state_keys:
                raise ContractError(f"Duplicate climate-state slice: {state}")
            state_keys.add(state_key)
            metrics = {}
            for variable in sorted(required):
                values = np.asarray(dataset[variable].values, dtype=float)
                metrics[variable] = {
                    "mean": float(np.mean(values)),
                    "q05": float(np.quantile(values, 0.05)),
                    "q50": float(np.quantile(values, 0.50)),
                    "q95": float(np.quantile(values, 0.95)),
                }
            hot = dataset["tasmax"] >= thresholds["tasmax_hot_day_threshold_degC"]
            warm_night = (
                dataset["tasmin"] >= thresholds["tasmin_warm_night_threshold_degC"]
            )
            dry = dataset["pr"] < thresholds["dry_day_threshold_mm_day-1"]
            low_wind = dataset["sfcWind"] < thresholds["low_wind_threshold_m_s-1"]
            counts = {
                "hot_days": int(hot.sum().item()),
                "warm_nights": int(warm_night.sum().item()),
                "dry_days": int(dry.sum().item()),
                "low_wind_days": int(low_wind.sum().item()),
                "hot_dry_days": int((hot & dry).sum().item()),
                "hot_dry_low_wind_days": int((hot & dry & low_wind).sum().item()),
            }
            records.append(
                {
                    "state": state,
                    "source": path_text(path),
                    "sha256": sha256_file(path),
                    "metrics": metrics,
                    "diagnostic_counts": counts,
                }
            )
    report = {
        "schema_version": "1.0",
        "study_id": study["study_id"],
        "purpose": "compound_diagnostic_pipeline_smoke_test",
        "interpretation_limit": "Single-year slices must not be interpreted as long-term climate-change signals or used to rank SSPs.",
        "probability_semantics": "No cross-state probabilities or pooled statistics are computed.",
        "thresholds": thresholds,
        "records": records,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    if args.output:
        write_json(args.output, report, args.overwrite)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--study", type=Path, default=DEFAULT_STUDY)
    result.add_argument("--sources", type=Path, default=DEFAULT_SOURCES)
    commands = result.add_subparsers(dest="command", required=True)

    validate = commands.add_parser(
        "validate", help="Validate manifests without network access"
    )
    validate.set_defaults(function=command_validate)

    plan = commands.add_parser(
        "plan", help="Estimate volume and produce deterministic raw targets"
    )
    plan.add_argument("--include-optional", action="store_true")
    plan.add_argument("--include-extension", action="store_true")
    plan.add_argument("--model-set", choices=MODEL_SET_CHOICES, default="pilot")
    plan.add_argument("--output", type=Path)
    plan.add_argument("--overwrite", action="store_true")
    plan.set_defaults(function=command_plan)

    era5 = commands.add_parser(
        "era5-requests", help="Render ERA5-Land requests; retrieval is opt-in"
    )
    era5.add_argument("--output-dir", type=Path)
    era5.add_argument("--site", choices=["yulin", "xian", "hanzhong", "ankang"])
    era5.add_argument("--year", type=int)
    era5.add_argument("--group", choices=sorted(ERA5_TIMESERIES_GROUPS))
    era5.add_argument("--include-optional", action="store_true")
    era5.add_argument("--execute", action="store_true")
    era5.add_argument("--overwrite", action="store_true")
    era5.set_defaults(function=command_era5_requests)

    era5_period = commands.add_parser(
        "era5-period",
        help="Retrieve one ERA5-Land site/group period and losslessly partition years",
    )
    era5_period.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    era5_period.add_argument(
        "--group",
        required=True,
        choices=["temperature_humidity", "precipitation", "solar_radiation", "wind"],
    )
    era5_period.add_argument("--start-year", required=True, type=int)
    era5_period.add_argument("--end-year", required=True, type=int)
    era5_period.add_argument("--execute", action="store_true")
    era5_period.add_argument("--overwrite", action="store_true")
    era5_period.set_defaults(function=command_era5_period)

    era5_site_period = commands.add_parser(
        "era5-site-period",
        help="Retrieve all four ERA5-Land groups for one site reference period",
    )
    era5_site_period.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    era5_site_period.add_argument("--start-year", required=True, type=int)
    era5_site_period.add_argument("--end-year", required=True, type=int)
    era5_site_period.add_argument("--execute", action="store_true")
    era5_site_period.add_argument("--overwrite", action="store_true")
    era5_site_period.set_defaults(function=command_era5_site_period)

    harmonize_era5 = commands.add_parser(
        "harmonize-era5",
        help="Align four ERA5-Land groups and aggregate one site-year to daily data",
    )
    harmonize_era5.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    harmonize_era5.add_argument("--year", required=True, type=int)
    harmonize_era5.add_argument("--overwrite", action="store_true")
    harmonize_era5.set_defaults(function=command_harmonize_era5)

    harmonize_era5_range = commands.add_parser(
        "harmonize-era5-range",
        help="Harmonize and independently QC an ERA5-Land site-year range",
    )
    harmonize_era5_range.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    harmonize_era5_range.add_argument("--start-year", required=True, type=int)
    harmonize_era5_range.add_argument("--end-year", required=True, type=int)
    harmonize_era5_range.add_argument("--overwrite", action="store_true")
    harmonize_era5_range.set_defaults(function=command_harmonize_era5_range)

    heat_stress = commands.add_parser(
        "derive-era5-heat-stress",
        help="Derive concurrent hourly wet-bulb and apparent-temperature diagnostics",
    )
    heat_stress.add_argument("--site", choices=["yulin", "xian", "hanzhong", "ankang"])
    heat_stress.add_argument("--start-year", type=int)
    heat_stress.add_argument("--end-year", type=int)
    heat_stress.add_argument(
        "--heat-stress-definition", type=Path, default=DEFAULT_HOURLY_HEAT_STRESS
    )
    heat_stress.add_argument("--overwrite", action="store_true")
    heat_stress.set_defaults(function=command_derive_era5_heat_stress)

    combine_era5 = commands.add_parser(
        "combine-era5-reference",
        help="Concatenate annual canonical ERA5-Land files into continuous site series",
    )
    combine_era5.add_argument("--site", choices=["yulin", "xian", "hanzhong", "ankang"])
    combine_era5.add_argument("--overwrite", action="store_true")
    combine_era5.set_defaults(function=command_combine_era5_reference)

    nex = commands.add_parser(
        "nex-request",
        help="Resolve and optionally retrieve one annual NEX point series",
    )
    nex.add_argument(
        "--model",
        required=True,
    )
    nex.add_argument("--model-set", choices=MODEL_SET_CHOICES, default="pilot")
    nex.add_argument(
        "--experiment",
        required=True,
        choices=["historical", "ssp126", "ssp245", "ssp370", "ssp585"],
    )
    nex.add_argument(
        "--variable",
        required=True,
        choices=["tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr"],
    )
    nex.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    nex.add_argument("--year", required=True, type=int)
    nex.add_argument("--execute", action="store_true")
    nex.add_argument("--overwrite", action="store_true")
    nex.add_argument("--timeout", type=int, default=120)
    nex.set_defaults(function=command_nex_request)

    bundle = commands.add_parser(
        "nex-bundle",
        help="Resolve and optionally retrieve several annual NEX point variables",
    )
    bundle.add_argument(
        "--model",
        required=True,
    )
    bundle.add_argument("--model-set", choices=MODEL_SET_CHOICES, default="pilot")
    bundle.add_argument(
        "--experiment",
        required=True,
        choices=["historical", "ssp126", "ssp245", "ssp370", "ssp585"],
    )
    bundle.add_argument(
        "--variables",
        nargs="+",
        required=True,
        choices=["tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr"],
    )
    bundle.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    bundle.add_argument("--year", required=True, type=int)
    bundle.add_argument("--execute", action="store_true")
    bundle.add_argument("--overwrite", action="store_true")
    bundle.add_argument("--timeout", type=int, default=120)
    bundle.set_defaults(function=command_nex_bundle)

    corridor = commands.add_parser(
        "nex-corridor",
        help="Retrieve one annual NEX corridor and extract all four registered sites",
    )
    corridor.add_argument(
        "--model",
        required=True,
    )
    corridor.add_argument("--model-set", choices=MODEL_SET_CHOICES, default="pilot")
    corridor.add_argument(
        "--experiment",
        required=True,
        choices=["historical", "ssp126", "ssp245", "ssp370", "ssp585"],
    )
    corridor.add_argument(
        "--variable",
        required=True,
        choices=["tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr"],
    )
    corridor.add_argument("--year", required=True, type=int)
    corridor.add_argument("--execute", action="store_true")
    corridor.add_argument("--overwrite", action="store_true")
    corridor.add_argument("--timeout", type=int, default=120)
    corridor.set_defaults(function=command_nex_corridor)

    pilot_download = commands.add_parser(
        "nex-pilot-download",
        help="Plan or execute the resumable annual-corridor NEX pilot archive",
    )
    pilot_download.add_argument(
        "--model",
    )
    pilot_download.add_argument(
        "--model-set", choices=MODEL_SET_CHOICES, default="pilot"
    )
    pilot_download.add_argument(
        "--experiment", choices=["historical", "ssp245", "ssp585"]
    )
    pilot_download.add_argument(
        "--variable", choices=["tasmax", "tasmin", "hurs", "sfcWind", "rsds", "pr"]
    )
    pilot_download.add_argument("--start-year", type=int)
    pilot_download.add_argument("--end-year", type=int)
    pilot_download.add_argument("--workers", type=int, default=4)
    pilot_download.add_argument("--timeout", type=int, default=120)
    pilot_download.add_argument("--progress", action="store_true")
    pilot_download.add_argument("--execute", action="store_true")
    pilot_download.add_argument("--output", type=Path)
    pilot_download.set_defaults(function=command_nex_pilot_download)

    harmonize = commands.add_parser(
        "harmonize-nex",
        help="Align and convert six raw NEX point variables into one daily dataset",
    )
    harmonize.add_argument(
        "--model",
        required=True,
    )
    harmonize.add_argument("--model-set", choices=MODEL_SET_CHOICES, default="pilot")
    harmonize.add_argument(
        "--experiment",
        required=True,
        choices=["historical", "ssp126", "ssp245", "ssp370", "ssp585"],
    )
    harmonize.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    harmonize.add_argument("--year", required=True, type=int)
    harmonize.add_argument("--overwrite", action="store_true")
    harmonize.set_defaults(function=command_harmonize_nex)

    harmonize_pilot = commands.add_parser(
        "harmonize-nex-pilot",
        help="Harmonize all selected NEX pilot model/experiment/site/year files",
    )
    harmonize_pilot.add_argument(
        "--model",
    )
    harmonize_pilot.add_argument(
        "--model-set", choices=MODEL_SET_CHOICES, default="pilot"
    )
    harmonize_pilot.add_argument(
        "--experiment", choices=["historical", "ssp245", "ssp585"]
    )
    harmonize_pilot.add_argument(
        "--site", choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    harmonize_pilot.add_argument("--start-year", type=int)
    harmonize_pilot.add_argument("--end-year", type=int)
    harmonize_pilot.add_argument("--workers", type=int, default=4)
    harmonize_pilot.add_argument("--progress", action="store_true")
    harmonize_pilot.add_argument("--overwrite", action="store_true")
    harmonize_pilot.add_argument("--output", type=Path)
    harmonize_pilot.set_defaults(function=command_harmonize_nex_pilot)

    audit = commands.add_parser(
        "nex-audit",
        help="Audit v2.0 annual catalog completeness for all pilot climate states",
    )
    audit.add_argument("--output", type=Path)
    audit.add_argument("--model-set", choices=MODEL_SET_CHOICES, default="pilot")
    audit.add_argument("--workers", type=int, default=4)
    audit.add_argument("--timeout", type=int, default=120)
    audit.add_argument("--progress", action="store_true")
    audit.add_argument("--overwrite", action="store_true")
    audit.set_defaults(function=command_nex_audit)

    qc = commands.add_parser(
        "qc-netcdf", help="Check a staged NetCDF file and emit SHA-256 provenance"
    )
    qc.add_argument("file", type=Path)
    qc.add_argument("--source", required=True, choices=["era5_land", "nex_gddp_cmip6"])
    qc.add_argument("--expected-year", type=int)
    qc.add_argument("--output", type=Path)
    qc.add_argument("--overwrite", action="store_true")
    qc.set_defaults(function=command_qc_netcdf)

    daily_qc = commands.add_parser(
        "qc-daily",
        help="Independently validate a canonical six-variable daily dataset",
    )
    daily_qc.add_argument("file", type=Path)
    daily_qc.add_argument("--expected-year", type=int, required=True)
    daily_qc.add_argument("--output", type=Path)
    daily_qc.add_argument("--overwrite", action="store_true")
    daily_qc.set_defaults(function=command_qc_daily)

    observation_qc = commands.add_parser(
        "qc-observations",
        help="Gate a licensed canonical CMA/CN05.1 daily observation table",
    )
    observation_qc.add_argument("file", type=Path)
    observation_qc.add_argument(
        "--source", required=True, choices=["cma_station", "cn05_1"]
    )
    observation_qc.add_argument(
        "--site", required=True, choices=["yulin", "xian", "hanzhong", "ankang"]
    )
    observation_qc.add_argument("--provenance", required=True, type=Path)
    observation_qc.add_argument(
        "--validation-contract", type=Path, default=DEFAULT_OBSERVATION_VALIDATION
    )
    observation_qc.add_argument("--output", type=Path)
    observation_qc.add_argument("--overwrite", action="store_true")
    observation_qc.set_defaults(function=command_qc_observations)

    audit_era5 = commands.add_parser(
        "audit-era5-reference",
        help="Verify period archives, annual partitions, daily files, and QC reports",
    )
    audit_era5.add_argument("--output", type=Path)
    audit_era5.add_argument("--overwrite", action="store_true")
    audit_era5.set_defaults(function=command_audit_era5_reference)

    diagnose_era5 = commands.add_parser(
        "diagnose-era5-reference",
        help="Compute predeclared ERA5-Land baseline and compound-event diagnostics",
    )
    diagnose_era5.add_argument(
        "--diagnostics", type=Path, default=DEFAULT_HISTORICAL_DIAGNOSTICS
    )
    diagnose_era5.add_argument(
        "--heat-stress-definition", type=Path, default=DEFAULT_HOURLY_HEAT_STRESS
    )
    diagnose_era5.add_argument("--output-dir", type=Path)
    diagnose_era5.add_argument("--overwrite", action="store_true")
    diagnose_era5.set_defaults(function=command_diagnose_era5_reference)

    summary = commands.add_parser(
        "summarize-daily",
        help="Summarize canonical slices without pooling climate-state probabilities",
    )
    summary.add_argument("files", nargs="+", type=Path)
    summary.add_argument("--output", type=Path)
    summary.add_argument("--overwrite", action="store_true")
    summary.set_defaults(function=command_summarize_daily)

    subset = commands.add_parser(
        "subset-netcdf", help="Subset a gridded NetCDF file to the four pilot sites"
    )
    subset.add_argument("file", type=Path)
    subset.add_argument("--output", type=Path, required=True)
    subset.add_argument("--max-distance-degrees", type=float, default=0.3)
    subset.add_argument("--overwrite", action="store_true")
    subset.set_defaults(function=command_subset_netcdf)
    return result


def main() -> int:
    args = parser().parse_args()
    try:
        return int(args.function(args))
    except ContractError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
