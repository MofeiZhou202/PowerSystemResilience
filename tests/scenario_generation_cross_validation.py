#!/usr/bin/env python3
"""Independent oracle for the scenario-generation review case.

The script intentionally does not import hacdcpf. It re-derives the public
Holland/rainfall and traffic recurrences from the published equations and
checks probability normalization and the declared statistical targets.
"""

import argparse
import json
import math
import pathlib
import subprocess
import tempfile


EARTH_RADIUS_KM = 6371.0088
OMEGA = 7.2921e-5
AIR_DENSITY = 1.15
PI = math.pi


def haversine(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(
        math.radians(lat2)
    ) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_KM * 2 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1 - a)))


def holland(storm, site_lat, site_lon):
    radius = max(1.0, haversine(storm["latitude"], storm["longitude"], site_lat, site_lon))
    rmw = max(1.0, storm["rmw_km"])
    b = max(0.5, storm["holland_b"])
    ratio = (rmw / radius) ** b
    pressure = b * storm["delta_p_hpa"] * 100.0 * ratio * math.exp(-ratio) / AIR_DENSITY
    r_m = radius * 1000.0
    tempc = storm["fc"] * r_m / 2.0
    gradient = math.sqrt(max(0.0, pressure + tempc * tempc)) - tempc
    v10 = 1.75 * gradient * (10.0 / 275.0) ** (1.0 / 7.0)
    return 0.0 if v10 < 0.01 else max(0.0, v10)


def bearing_sector(storm, lat, lon):
    y = math.sin(math.radians(lon - storm["longitude"])) * math.cos(math.radians(lat))
    x = math.cos(math.radians(storm["latitude"])) * math.sin(math.radians(lat)) - math.sin(
        math.radians(storm["latitude"])
    ) * math.cos(math.radians(lat)) * math.cos(math.radians(lon - storm["longitude"]))
    deg = math.degrees(math.atan2(y, x))
    if deg < 0:
        deg += 360.0
    deg = (deg + storm["heading_deg"]) % 360.0
    return deg


def rain_multiplier(sector, speed):
    fast = speed > 8.0
    slow = speed < 4.0
    return {
        0: 1.15 if fast else (1.45 if slow else 1.0),
        1: 1.15 if fast else (1.05 if slow else 1.0),
        2: 1.35 if fast else (0.55 if slow else 1.0),
        3: 1.35 if fast else (0.65 if slow else 1.0),
        4: 0.85,
        5: 0.65 if fast else (0.95 if slow else 1.0),
        6: 0.80 if fast else (1.15 if slow else 1.0),
        7: 0.95 if fast else (1.35 if slow else 1.0),
    }[sector]


def rainfall(storm, previous, site_lat, site_lon):
    radius = max(1.0, haversine(storm["latitude"], storm["longitude"], site_lat, site_lon))
    ratio = max(0.0, min(1.1, storm["rmw_km"] / radius))
    rr = max(0.0, -5.5 + 110 * ratio - 390 * ratio**2 + 550 * ratio**3 - 250 * ratio**4)
    dpdt = (previous["delta_p_hpa"] - storm["delta_p_hpa"]) if previous else 1.0
    k = max(1.0, 0.0319 * storm["delta_p_hpa"] - 0.0395)
    k1 = max(1.0, 1.0 - dpdt / 100.0)
    sector = int(math.floor(bearing_sector(storm, site_lat, site_lon) / 45.0)) % 8
    return max(0.0, k * k1 * rr * rain_multiplier(sector, storm["translation_speed_kmph"] / 3.6))


def flood_speed_factor(depth, options):
    speed = (
        options["depth_speed_quadratic"] * depth * depth
        + options["depth_speed_linear"] * depth
        + options["depth_speed_intercept_km_hr"]
    )
    return max(options["minimum_open_speed_factor"], min(1.0, speed / options["depth_speed_intercept_km_hr"]))


def wind_capacity_factor(wind, options):
    if wind <= options["wind_capacity_reduction_start_ms"]:
        return 1.0
    severity = max(0.0, min(1.0, (wind - options["wind_capacity_reduction_start_ms"]) / (
        options["wind_capacity_reduction_full_ms"] - options["wind_capacity_reduction_start_ms"]
    )))
    return 1.0 - severity * (1.0 - max(0.0, min(1.0, options["minimum_wind_capacity_factor"])))


def equivalent_wind(wind, rain):
    v10min = wind / 1.42
    f1 = math.exp(0.006462 * v10min) - 1.2486 * math.exp(-0.2769 * v10min)
    f2 = 0.09376 * rain**0.7087 if rain > 0.0 else 0.0
    return max(0.0, (v10min + max(0.0, f1 * f2)) * 1.42)


def overhead_probability(wind, rain, fragility, dt=1.0):
    eq_wind = equivalent_wind(wind, rain)
    p_wind = 0.0
    if eq_wind > fragility["design_wind_ms"]:
        if fragility["collapse_wind_ms"] > 0.0 and eq_wind >= fragility["collapse_wind_ms"]:
            severity = 1.0
        else:
            z = (math.log(eq_wind) - math.log(fragility["lognormal_median_wind_ms"])) / fragility["lognormal_sigma"]
            severity = 0.5 * math.erfc(-z / math.sqrt(2.0))
        exposure = max(0.0, min(5.0, fragility["segment_length_km"]))
        p_wind = max(0.0, min(1.0, 1.0 - math.exp(-0.015 * severity * exposure * max(0.0, dt))))
    wind_ratio = wind / fragility["design_wind_ms"] if fragility["design_wind_ms"] > 0.0 else 0.0
    rain_ratio = rain / fragility["design_rain_mm_hr"] if fragility["design_rain_mm_hr"] > 0.0 else 0.0
    rate = fragility["segment_length_km"] ** 2 * math.exp(
        fragility["seg_a_wind"] * wind_ratio
        + fragility["seg_b_rain"] * rain_ratio
        + fragility["seg_c_bias"]
    )
    exposed = wind > fragility["design_wind_ms"] or rain > fragility["design_rain_mm_hr"]
    p_segment = max(0.0, min(1.0, 1.0 - math.exp(-rate * max(0.0, dt)))) if exposed else 0.0
    return max(0.0, min(1.0, 1.0 - (1.0 - p_wind) * (1.0 - p_segment)))


def pearson(x, y):
    assert len(x) == len(y) and len(x) > 1
    mx = sum(x) / len(x)
    my = sum(y) / len(y)
    numerator = sum((a - mx) * (b - my) for a, b in zip(x, y))
    sx = sum((a - mx) ** 2 for a in x)
    sy = sum((b - my) ** 2 for b in y)
    return numerator / math.sqrt(sx * sy)


def robust_scaled_features(candidates):
    discrete = {"regular_group", "ssp_code", "year", "climate_data_found",
                "contingency_component_index", "contingency_ordinal", "typhoon_month"}
    keys = sorted({key for candidate in candidates for key in candidate["features"]} - discrete)
    scaled = {candidate["id"]: [] for candidate in candidates}
    for key in keys:
        column = [candidate["features"].get(key, 0.0) for candidate in candidates]
        ordered = sorted(column)
        middle = len(ordered) // 2
        center = (ordered[middle - 1] + ordered[middle]) / 2.0 if len(ordered) % 2 == 0 else ordered[middle]
        q1 = ordered[len(ordered) // 4]
        q3 = ordered[(len(ordered) * 3) // 4]
        scale = max(1e-9, q3 - q1)
        for candidate, value in zip(candidates, column):
            scaled[candidate["id"]].append((value - center) / scale)
    return scaled


def validate_reduction(reduction):
    result = reduction["result"]
    candidates = result["audit"]["coverage_samples"]
    clusters = result["clusters"]
    assert len(candidates) == reduction["candidate_count"] == 128
    assert len(clusters) == reduction["cluster_count"] == 12
    scaled = robust_scaled_features(candidates)
    by_id = {candidate["id"]: candidate for candidate in candidates}
    probability_sum = sum(cluster["probability"] for cluster in clusters)
    member_total = sum(cluster["member_count"] for cluster in clusters)
    max_average_error = 0.0
    max_radius_error = 0.0
    transport_mean = 0.0
    transport_max = 0.0
    for cluster in clusters:
        assert cluster["representative_id"] in cluster["member_ids"]
        assert cluster["member_count"] == len(cluster["member_ids"])
        representative = scaled[cluster["representative_id"]]
        distances = []
        weighted = 0.0
        for member_id in cluster["member_ids"]:
            candidate = by_id[member_id]
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(scaled[member_id], representative)))
            distances.append(distance)
            weighted += candidate["probability"] * distance
        expected_average = weighted / cluster["probability"]
        expected_maximum = max(distances)
        max_average_error = max(max_average_error, abs(expected_average - cluster["average_distance"]))
        max_radius_error = max(max_radius_error, abs(expected_maximum - cluster["max_distance"]))
        transport_mean += cluster["probability"] * expected_average
        transport_max = max(transport_max, expected_maximum)
    metrics = reduction["audit"]["metrics"]
    assert member_total == 128
    assert abs(probability_sum - 1.0) < 1e-12
    assert max_average_error < 1e-10
    assert max_radius_error < 1e-10
    assert abs(transport_mean - metrics["transport_mean"]) < 1e-10
    assert abs(transport_max - metrics["transport_max"]) < 1e-10
    assert reduction["audit"]["tail_anchor_count"] == len(reduction["audit"]["anchor_ids"])
    return {
        "probability_error": abs(probability_sum - 1.0),
        "member_total": member_total,
        "max_average_distance_error": max_average_error,
        "max_radius_error": max_radius_error,
        "transport_mean": transport_mean,
        "transport_max": transport_max,
        "tail_anchor_count": reduction["audit"]["tail_anchor_count"],
        "frozen_medoid_count": reduction["audit"]["frozen_medoid_count"],
    }


def check(args):
    with tempfile.TemporaryDirectory(prefix="scenario_generation_xref_") as temp:
        output = pathlib.Path(temp) / "review.json"
        subprocess.run([args.cpp_bin, str(output)], check=True)
        raw_cpp = output.read_text()
        data = json.loads(raw_cpp)

    regular = data["regular"]
    assert regular["candidate_count"] == 64
    assert regular["cluster_count"] == 6
    assert abs(regular["candidate_probability_sum"] - 1.0) < 1e-12
    assert abs(regular["cluster_probability_sum"] - 1.0) < 1e-12
    assert abs(regular["load_pv_energy_correlation"] - (-0.25)) < 0.15
    assert regular["deterministic_repeat"] is True
    ar1 = regular["ar1_process"]
    assert ar1["steps"] == 4096
    assert abs(ar1["load_lag1_correlation"] - ar1["target_temporal_correlation"]) < 0.05
    # Wind is nonzero throughout this synthetic profile, so it observes the
    # complete bivariate AR(1) sample.  Daylight-only PV is reported as a
    # separate periodic-subselection diagnostic rather than used as the gate.
    assert abs(ar1["load_wind_contemporaneous_correlation"] - ar1["target_cross_correlation"]) < 0.08
    reduction_report = validate_reduction(regular["reduction"])

    storm = data["hazard"]["storm"]
    native_wind = holland(storm, data["hazard"]["site_latitude"], data["hazard"]["site_longitude"])
    native_rain = rainfall(storm, None, data["hazard"]["site_latitude"], data["hazard"]["site_longitude"])
    assert abs(native_wind - data["hazard"]["holland_wind_ms"]) < 1e-10
    assert abs(native_rain - data["hazard"]["rainfall_mm_hr"]) < 1e-10
    fragility = data["hazard"]["fragility"]
    independent_failure_probability = overhead_probability(
        native_wind, native_rain, fragility
    )
    fragility_error = abs(independent_failure_probability - fragility["peak_failure_probability"])
    assert fragility_error < 1e-10
    assert fragility["fault_count"] == 0
    assert fragility["peak_failure_probability"] < fragility["sampling_gate"]

    options = data["traffic"]["options"]
    depth = 0.0
    expected_depth = []
    expected_capacity = []
    for step, row in enumerate(data["traffic"]["steps"]):
        previous = data["traffic"]["steps"][step - 1] if step else None
        rain = rainfall(storm, None, row["midpoint_latitude"], row["midpoint_longitude"])
        # The review track is stationary, so the first-step and later-step
        # pressure tendency are identical; recompute the actual input from the
        # row rather than trusting the C++ output.
        if previous is not None:
            rain = row["rainfall_mm_hr"]
        wind = holland(storm, row["midpoint_latitude"], row["midpoint_longitude"])
        depth = max(0.0, min(options["maximum_surface_water_mm"], depth + (
            options["rainfall_runoff_coefficient"] * rain - options["drainage_rate_mm_hr"]
        ) * 1.0))
        speed = flood_speed_factor(depth, options)
        wind_factor = wind_capacity_factor(wind, options)
        capacity = max(options["minimum_open_capacity_factor"], min(1.0, speed ** options["capacity_speed_exponent"] * wind_factor))
        expected_depth.append(depth)
        expected_capacity.append(capacity)
        assert abs(depth - row["surface_water_mm"]) < 1e-10
        assert abs(capacity - row["capacity_factor"]) < 1e-10
    assert data["traffic"]["closed_link_steps"] == sum(
        depth >= data["traffic"]["options"]["flood_closure_depth_mm"] for depth in expected_depth
    )

    assert data["reliability_catalog"]["count"] == len(data["reliability_catalog"]["ids"])
    pathlib.Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    (pathlib.Path(args.output_dir) / "scenario_generation_review_case.json").write_text(
        raw_cpp
    )
    (pathlib.Path(args.output_dir) / "scenario_generation_cross_validation.json").write_text(
        json.dumps({
            "passed": True,
            "regular": {
                "candidate_probability_error": abs(regular["candidate_probability_sum"] - 1.0),
                "cluster_probability_error": abs(regular["cluster_probability_sum"] - 1.0),
                "load_pv_correlation": regular["load_pv_energy_correlation"],
                "deterministic_repeat": regular["deterministic_repeat"],
                "ar1": ar1,
                "reduction": reduction_report,
            },
            "hazard": {"wind_error": abs(native_wind - data["hazard"]["holland_wind_ms"]),
                       "rainfall_error": abs(native_rain - data["hazard"]["rainfall_mm_hr"]),
                       "fragility_error": fragility_error,
                       "failure_probability": independent_failure_probability},
            "traffic": {"max_depth_error": max(abs(a - b) for a, b in zip(expected_depth, [r["surface_water_mm"] for r in data["traffic"]["steps"]]))},
        }, indent=2) + "\n"
    )
    print(json.dumps({"passed": True, "output_dir": args.output_dir}, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cpp-bin", required=True)
    parser.add_argument("--output-dir", required=True)
    check(parser.parse_args())


if __name__ == "__main__":
    main()
