#!/usr/bin/env python3
"""Validate benchmark correctness and conservative runtime envelopes."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--tolerance", type=float, default=0.0)
    args = parser.parse_args()

    if not math.isfinite(args.tolerance) or args.tolerance < 0.0:
        parser.error("--tolerance must be finite and non-negative")

    results = load_json(args.result)
    baseline = load_json(args.baseline)
    if not isinstance(results, list):
        raise SystemExit("result must be a JSON array")
    if baseline.get("schema_version") != 1 or not isinstance(baseline.get("entries"), list):
        raise SystemExit("unsupported performance baseline schema")

    by_key = {}
    for row in results:
        key = (row.get("case"), row.get("config"))
        if key in by_key:
            raise SystemExit(f"duplicate benchmark result: {key}")
        by_key[key] = row

    failures: list[str] = []
    for expected in baseline["entries"]:
        key = (expected.get("case"), expected.get("config"))
        actual = by_key.get(key)
        if actual is None:
            failures.append(f"missing result {key}")
            continue
        if not actual.get("success", False):
            failures.append(f"{key}: solver failed with status={actual.get('status')!r}")
            continue

        objective = actual.get("objective")
        expected_objective = expected.get("objective")
        objective_tol = expected.get("objective_abs_tol", 0.0)
        if not finite_number(objective) or abs(float(objective) - float(expected_objective)) > objective_tol:
            failures.append(
                f"{key}: objective {objective!r} outside {expected_objective} +/- {objective_tol}"
            )

        gap = actual.get("gap")
        if not finite_number(gap) or float(gap) > float(expected.get("max_gap", 0.0)):
            failures.append(f"{key}: gap {gap!r} exceeds {expected.get('max_gap')}")

        runtime = actual.get("runtime_ms")
        runtime_ceiling = float(expected["max_runtime_ms"]) * (1.0 + args.tolerance)
        if not finite_number(runtime) or float(runtime) > runtime_ceiling:
            failures.append(f"{key}: runtime {runtime!r} ms exceeds {runtime_ceiling:.3f} ms")

    if failures:
        print("performance gate failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print(f"performance gate passed for {len(baseline['entries'])} benchmark rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
