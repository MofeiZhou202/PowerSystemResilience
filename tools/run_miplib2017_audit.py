#!/usr/bin/env python3
"""Run the MIPLIB benchmark and freeze a reproducible audit report."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import itertools
import json
import math
import os
import pathlib
import platform
import random
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Optional


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def command_output(args: list[str], cwd: pathlib.Path) -> Optional[str]:
    try:
        completed = subprocess.run(
            args,
            cwd=cwd,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return completed.stdout.strip()


def git_manifest(repo: pathlib.Path) -> dict[str, Any]:
    commit = command_output(["git", "rev-parse", "HEAD"], repo)
    diff = subprocess.run(
        ["git", "diff", "--binary", "HEAD"],
        cwd=repo,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    ).stdout
    untracked_raw = subprocess.run(
        ["git", "ls-files", "--others", "--exclude-standard", "-z"],
        cwd=repo,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
    ).stdout
    untracked: list[dict[str, Any]] = []
    for raw_name in untracked_raw.split(b"\0"):
        if not raw_name:
            continue
        name = os.fsdecode(raw_name)
        path = repo / name
        if path.is_file():
            untracked.append(
                {"path": name, "sha256": sha256_file(path), "size": path.stat().st_size}
            )

    state = hashlib.sha256()
    state.update((commit or "unknown").encode())
    state.update(diff)
    for entry in sorted(untracked, key=lambda item: item["path"]):
        state.update(entry["path"].encode())
        state.update(entry["sha256"].encode())
    return {
        "commit": commit,
        "dirty": bool(diff or untracked),
        "tracked_diff_sha256": hashlib.sha256(diff).hexdigest(),
        "untracked_files": untracked,
        "source_state_sha256": state.hexdigest(),
    }


def cmake_manifest(repo: pathlib.Path) -> dict[str, Any]:
    cache_path = repo / "CMakeCache.txt"
    values: dict[str, str] = {}
    if cache_path.is_file():
        for line in cache_path.read_text(errors="replace").splitlines():
            if line.startswith("//") or line.startswith("#") or "=" not in line:
                continue
            key_and_type, value = line.split("=", 1)
            key = key_and_type.split(":", 1)[0]
            if key in {
                "CMAKE_BUILD_TYPE",
                "CMAKE_CXX_COMPILER",
                "CMAKE_CXX_COMPILER_VERSION",
                "CMAKE_CXX_FLAGS",
                "CMAKE_CXX_FLAGS_RELEASE",
            }:
                values[key] = value
    compiler = values.get("CMAKE_CXX_COMPILER")
    compiler_version = None
    if compiler:
        compiler_version = command_output([compiler, "--version"], repo)
    return {
        "cache_sha256": sha256_file(cache_path) if cache_path.is_file() else None,
        "values": values,
        "compiler_version_output": compiler_version,
    }


def cpu_manifest(repo: pathlib.Path) -> dict[str, Any]:
    brand = None
    model = None
    memory = None
    if sys.platform == "darwin":
        brand = command_output(["sysctl", "-n", "machdep.cpu.brand_string"], repo)
        hardware = command_output(
            ["system_profiler", "SPHardwareDataType", "-detailLevel", "mini"], repo
        )
        if hardware:
            for line in hardware.splitlines():
                key, separator, value = line.strip().partition(":")
                if not separator:
                    continue
                if key == "Chip":
                    brand = brand or value.strip()
                elif key == "Model Identifier":
                    model = value.strip()
                elif key == "Memory":
                    memory = value.strip()
    elif pathlib.Path("/proc/cpuinfo").is_file():
        for line in pathlib.Path("/proc/cpuinfo").read_text(errors="replace").splitlines():
            if line.lower().startswith("model name") and ":" in line:
                brand = line.split(":", 1)[1].strip()
                break
    return {
        "brand": brand or platform.processor() or None,
        "model": model,
        "memory": memory,
        "architecture": platform.machine(),
        "logical_cpus": os.cpu_count(),
        "platform": platform.platform(),
        "python": platform.python_version(),
    }


def instance_name(path: pathlib.Path) -> str:
    name = path.name
    if name.endswith(".gz"):
        name = name[:-3]
    if name.endswith(".mps"):
        name = name[:-4]
    return name


def resolve_instance_paths(data_dir: pathlib.Path, names: Iterable[str]) -> dict[str, pathlib.Path]:
    wanted = set(names)
    resolved: dict[str, pathlib.Path] = {}
    for path in data_dir.rglob("*"):
        if not path.is_file() or not (path.name.endswith(".mps") or path.name.endswith(".mps.gz")):
            continue
        name = instance_name(path)
        if name not in wanted:
            continue
        if name in resolved:
            raise RuntimeError(f"duplicate instance name {name}: {resolved[name]} and {path}")
        resolved[name] = path.resolve()
    missing = wanted - resolved.keys()
    if missing:
        raise RuntimeError("missing selected instances: " + ", ".join(sorted(missing)))
    return resolved


def finite_number(value: Any) -> Optional[float]:
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def benchmark_solved(row: dict[str, Any]) -> bool:
    reference_status = row.get("reference_status") or ""
    return bool(
        (row.get("proven") and row.get("audit", {}).get("passed") and
         (not reference_status or row.get("reference_match")))
        or (row.get("proven") and not row.get("has_solution") and row.get("reference_match"))
    )


def audited_pdi(row: dict[str, Any]) -> Optional[float]:
    available = row.get("statistics_available", {}).get("bound_events", False)
    value = finite_number(row.get("primal_dual_integral_sec"))
    if not available or value is None or value < 0.0:
        return None
    events = row.get("bound_events", [])
    if not isinstance(events, list):
        return None
    for event in events:
        if finite_number(event.get("primal_bound")) is not None:
            if not event.get("incumbent_audit", {}).get("passed", False):
                return None
    return value


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def paired_analysis(report: dict[str, Any]) -> dict[str, Any]:
    solvers = list(report.get("solvers", []))
    rows = report.get("results", [])
    penalty_ms = 10.0 * float(report["time_limit_sec"]) * 1000.0
    by_key: dict[tuple[str, int, str], dict[str, Any]] = {}
    for row in rows:
        by_key[(row["instance"], int(row["repeat"]), row["solver"])] = row

    comparisons: list[dict[str, Any]] = []
    for solver_a, solver_b in itertools.combinations(solvers, 2):
        instance_log_ratios: list[float] = []
        paired_repetitions = 0
        for instance in sorted({row["instance"] for row in rows}):
            repeated_log_ratios: list[float] = []
            repeats = sorted({int(row["repeat"]) for row in rows if row["instance"] == instance})
            for repeat in repeats:
                row_a = by_key.get((instance, repeat, solver_a))
                row_b = by_key.get((instance, repeat, solver_b))
                if row_a is None or row_b is None:
                    continue
                time_a = float(row_a["solve_ms"]) if benchmark_solved(row_a) else penalty_ms
                time_b = float(row_b["solve_ms"]) if benchmark_solved(row_b) else penalty_ms
                repeated_log_ratios.append(
                    math.log(max(time_a, 1e-9) / max(time_b, 1e-9))
                )
            if repeated_log_ratios:
                paired_repetitions += len(repeated_log_ratios)
                instance_log_ratios.append(
                    sum(repeated_log_ratios) / len(repeated_log_ratios)
                )

        mean_log = (
            sum(instance_log_ratios) / len(instance_log_ratios)
            if instance_log_ratios else None
        )
        interval = None
        if len(instance_log_ratios) >= 2:
            pair_seed = int.from_bytes(
                hashlib.sha256(f"{report['seed']}:{solver_a}:{solver_b}".encode()).digest()[:8],
                "big",
            )
            rng = random.Random(pair_seed)
            bootstrap = []
            for _ in range(10000):
                sample = [
                    instance_log_ratios[rng.randrange(len(instance_log_ratios))]
                    for _ in instance_log_ratios
                ]
                bootstrap.append(math.exp(sum(sample) / len(sample)))
            interval = [percentile(bootstrap, 0.025), percentile(bootstrap, 0.975)]
        comparisons.append(
            {
                "solver_a": solver_a,
                "solver_b": solver_b,
                "paired_instances": len(instance_log_ratios),
                "paired_repetitions": paired_repetitions,
                "bootstrap_unit": "instance",
                "metric": "PAR-10 time ratio a_over_b",
                "geometric_mean_ratio": math.exp(mean_log) if mean_log is not None else None,
                "bootstrap_95_percentile_ci": interval,
            }
        )
    pdi_summaries: list[dict[str, Any]] = []
    for solver in solvers:
        values = [
            value
            for row in rows
            if row.get("solver") == solver
            for value in [audited_pdi(row)]
            if value is not None
        ]
        pdi_summaries.append(
            {
                "solver": solver,
                "available_runs": len(values),
                "attempts": sum(row.get("solver") == solver for row in rows),
                "mean_sec": sum(values) / len(values) if values else None,
                "median_sec": percentile(values, 0.5) if values else None,
            }
        )

    pdi_comparisons: list[dict[str, Any]] = []
    for solver_a, solver_b in itertools.combinations(solvers, 2):
        instance_log_ratios: list[float] = []
        paired_repetitions = 0
        for instance in sorted({row["instance"] for row in rows}):
            repeated_log_ratios: list[float] = []
            repeats = sorted(
                {int(row["repeat"]) for row in rows if row["instance"] == instance}
            )
            for repeat in repeats:
                row_a = by_key.get((instance, repeat, solver_a))
                row_b = by_key.get((instance, repeat, solver_b))
                if row_a is None or row_b is None:
                    continue
                pdi_a = audited_pdi(row_a)
                pdi_b = audited_pdi(row_b)
                if pdi_a is None or pdi_b is None:
                    continue
                repeated_log_ratios.append(
                    math.log(max(pdi_a, 1e-9) / max(pdi_b, 1e-9))
                )
            if repeated_log_ratios:
                paired_repetitions += len(repeated_log_ratios)
                instance_log_ratios.append(
                    sum(repeated_log_ratios) / len(repeated_log_ratios)
                )

        mean_log = (
            sum(instance_log_ratios) / len(instance_log_ratios)
            if instance_log_ratios else None
        )
        interval = None
        if len(instance_log_ratios) >= 2:
            pair_seed = int.from_bytes(
                hashlib.sha256(
                    f"pdi:{report['seed']}:{solver_a}:{solver_b}".encode()
                ).digest()[:8],
                "big",
            )
            rng = random.Random(pair_seed)
            bootstrap = []
            for _ in range(10000):
                sample = [
                    instance_log_ratios[rng.randrange(len(instance_log_ratios))]
                    for _ in instance_log_ratios
                ]
                bootstrap.append(math.exp(sum(sample) / len(sample)))
            interval = [percentile(bootstrap, 0.025), percentile(bootstrap, 0.975)]
        pdi_comparisons.append(
            {
                "solver_a": solver_a,
                "solver_b": solver_b,
                "paired_instances": len(instance_log_ratios),
                "paired_repetitions": paired_repetitions,
                "bootstrap_unit": "instance",
                "metric": "fixed-horizon PDI ratio a_over_b",
                "geometric_mean_ratio":
                    math.exp(mean_log) if mean_log is not None else None,
                "bootstrap_95_percentile_ci": interval,
            }
        )

    pdi_available_runs = sum(item["available_runs"] for item in pdi_summaries)
    return {
        "paired_par10_comparisons": comparisons,
        "primal_dual_integral": {
            "unit": "seconds",
            "horizon_sec": float(report["time_limit_sec"]),
            "pre_bound_penalty_gap": 1.0,
            "primal_event_policy":
                "Every primal event must carry an original-space-audited incumbent.",
            "solver_summaries": pdi_summaries,
            "paired_comparisons": pdi_comparisons,
        } if pdi_available_runs else None,
        "primal_dual_integral_unavailable_reason": None if pdi_available_runs else
            "No run exposed a complete audited primal/dual bound-event stream.",
    }


def build_manifest(
    repo: pathlib.Path,
    executable: pathlib.Path,
    command: list[str],
    report: dict[str, Any],
    source: dict[str, Any],
) -> dict[str, Any]:
    data_dir = pathlib.Path(report["data_dir"])
    if not data_dir.is_absolute():
        data_dir = repo / data_dir
    solution_file = pathlib.Path(report.get("solution_file") or "")
    if solution_file and not solution_file.is_absolute():
        solution_file = repo / solution_file

    names = sorted({row["instance"] for row in report.get("results", [])})
    paths = resolve_instance_paths(data_dir.resolve(), names)
    rows_by_name = {row["instance"]: row for row in report.get("results", [])}
    instances = []
    for name in names:
        path = paths[name]
        row = rows_by_name[name]
        instances.append(
            {
                "name": name,
                "path": str(path),
                "sha256": sha256_file(path),
                "size": path.stat().st_size,
                "reference_status": row.get("reference_status") or None,
                "reference_objective": finite_number(row.get("reference_objective")),
                "rows": row.get("rows"),
                "columns": row.get("columns"),
                "nonzeros": row.get("nonzeros"),
                "variable_types": {
                    "integers": row.get("integers"),
                    "binaries": row.get("binaries"),
                    "semicontinuous": row.get("semicontinuous"),
                    "semiinteger": row.get("semiinteger"),
                },
            }
        )

    relevant_environment = {
        key: value
        for key, value in sorted(os.environ.items())
        if key.startswith("MIPSOLVERS_")
        or key.startswith("HACDCPF_")
        or key in {"OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS",
                   "VECLIB_MAXIMUM_THREADS"}
    }
    manifest = {
        "schema": "miplib2017-audit-manifest-v1",
        "created_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "command": command,
        "working_directory": str(repo),
        "executable": {
            "path": str(executable),
            "sha256": sha256_file(executable),
            "size": executable.stat().st_size,
        },
        "source": source,
        "build": cmake_manifest(repo),
        "host": cpu_manifest(repo),
        "solver_versions": report.get("solver_versions", {}),
        "compiler_id_from_binary": report.get("compiler_id"),
        "environment": relevant_environment,
        "solution_file": {
            "path": str(solution_file.resolve()) if solution_file else None,
            "sha256": sha256_file(solution_file) if solution_file.is_file() else None,
            "size": solution_file.stat().st_size if solution_file.is_file() else None,
        },
        "instances": instances,
        "experiment": {
            "solvers": report.get("solvers"),
            "repeats": report.get("repeats"),
            "execution_order": report.get("execution_order"),
            "threads": report.get("threads"),
            "seed": report.get("seed"),
            "time_limit_sec": report.get("time_limit_sec"),
            "hard_timeout_grace_sec": report.get("hard_timeout_grace_sec"),
            "gap": report.get("gap"),
            "max_nodes_native": report.get("max_nodes_native"),
        },
    }
    canonical = json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()
    manifest["manifest_payload_sha256"] = hashlib.sha256(canonical).hexdigest()
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", default="./tests/miplib2017_benchmark")
    parser.add_argument("--repo-root", default=".")
    parser.add_argument("--json", required=True, dest="json_path")
    parser.add_argument("--csv", required=True, dest="csv_path")
    parser.add_argument("benchmark_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()

    repo = pathlib.Path(args.repo_root).resolve()
    executable = pathlib.Path(args.benchmark)
    if not executable.is_absolute():
        executable = (repo / executable).resolve()
    if not executable.is_file():
        parser.error(f"benchmark executable does not exist: {executable}")

    benchmark_args = list(args.benchmark_args)
    if benchmark_args and benchmark_args[0] == "--":
        benchmark_args.pop(0)
    forbidden = {"--json", "--csv", "--worker-instance", "--worker-output", "--worker-solver"}
    if forbidden.intersection(benchmark_args):
        parser.error("output and worker options are owned by the audit wrapper")

    def option_value(flag: str) -> Optional[str]:
        try:
            return benchmark_args[benchmark_args.index(flag) + 1]
        except (ValueError, IndexError):
            return None

    repeat_text = option_value("--repeat")
    if repeat_text is None or int(repeat_text) < 3:
        parser.error("audit experiments require --repeat 3 or greater")

    source = git_manifest(repo)
    json_path = pathlib.Path(args.json_path)
    csv_path = pathlib.Path(args.csv_path)
    if not json_path.is_absolute():
        json_path = repo / json_path
    if not csv_path.is_absolute():
        csv_path = repo / csv_path

    with tempfile.TemporaryDirectory(prefix="miplib_audit_") as temp_name:
        temp_dir = pathlib.Path(temp_name)
        raw_json = temp_dir / "raw.json"
        raw_csv = temp_dir / "raw.csv"
        command = [
            str(executable),
            *benchmark_args,
            "--json", str(raw_json),
            "--csv", str(raw_csv),
        ]
        completed = subprocess.run(command, cwd=repo, check=False)
        if completed.returncode != 0:
            return completed.returncode

        report = json.loads(raw_json.read_text())
        report["immutable_manifest"] = build_manifest(
            repo, executable, command, report, source
        )
        report["statistical_analysis"] = paired_analysis(report)

        json_path.parent.mkdir(parents=True, exist_ok=True)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(
            "w", dir=json_path.parent, delete=False, encoding="utf-8"
        ) as stream:
            json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            staged_json = pathlib.Path(stream.name)
        os.replace(staged_json, json_path)
        shutil.copyfile(raw_csv, csv_path)

    print(f"Audit JSON: {json_path}")
    print(f"Raw CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
