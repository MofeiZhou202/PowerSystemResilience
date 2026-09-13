#!/usr/bin/env python3
"""Check stable-hardware LP accuracy and latency distributions."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def load(path: Path) -> Any:
    with path.open("r", encoding="utf-8-sig") as stream:
        return json.load(stream)


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    args = parser.parse_args()
    result = load(args.result)
    baseline = load(args.baseline)
    if baseline.get("schema_version") != 1:
        raise SystemExit("unsupported LP release baseline schema")

    config = str(baseline["thread_budget"])
    actual = result.get(config)
    if not isinstance(actual, dict):
        raise SystemExit(f"result has no thread configuration {config}")
    failures: list[str] = []
    runs = actual.get("runs")
    accurate = actual.get("accurate")
    minimum_runs = int(baseline["minimum_accurate_runs"])
    if not isinstance(runs, int) or runs < minimum_runs or accurate != runs:
        failures.append(
            f"accuracy: measured {accurate}/{runs}, required all of at least {minimum_runs}"
        )

    def compare(label: str, measured: Any, reference: Any, ratio: float) -> None:
        if not finite(measured):
            failures.append(f"{label}: measured value is not finite: {measured!r}")
            return
        ceiling = float(reference) * ratio
        print(
            f"{label}: measured={float(measured):.3f} ms, "
            f"baseline={float(reference):.3f} ms, ceiling={ceiling:.3f} ms"
        )
        if float(measured) > ceiling:
            failures.append(
                f"{label}: {float(measured):.3f} ms exceeds {ceiling:.3f} ms"
            )

    case_limits = baseline["case_limits"]
    for case, expected in baseline["cases"].items():
        measured = actual.get("cases", {}).get(case)
        if not isinstance(measured, dict):
            failures.append(f"{case}: missing distribution")
            continue
        compare(
            f"{case}.median",
            measured.get("median"),
            expected["median_ms"],
            float(case_limits["median_ratio"]),
        )
        compare(
            f"{case}.p95",
            measured.get("p95"),
            expected["p95_ms"],
            float(case_limits["p95_ratio"]),
        )

    aggregate = actual.get("aggregate", {})
    aggregate_limits = baseline["aggregate_limits"]
    compare(
        "aggregate.median",
        aggregate.get("median"),
        baseline["aggregate"]["median_ms"],
        float(aggregate_limits["median_ratio"]),
    )
    compare(
        "aggregate.p95",
        aggregate.get("p95"),
        baseline["aggregate"]["p95_ms"],
        float(aggregate_limits["p95_ratio"]),
    )

    for case, expected_iterations in baseline["fixed_iteration_sets"].items():
        measured_iterations = actual.get("iterations", {}).get(case)
        if measured_iterations != expected_iterations:
            failures.append(
                f"{case}: iteration set {measured_iterations!r} != {expected_iterations!r}"
            )

    if failures:
        print("LP release gate failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("LP release gate passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
