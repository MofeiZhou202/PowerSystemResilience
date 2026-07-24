#!/usr/bin/env python3
"""Generate the frozen paired KNN/VAR future-weather ensemble."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import climate_data as cd


DEFAULT_DEFINITION = (
    cd.TOOL_DIR / "manifests/future_weather_resilience_study.json"
)


@dataclass(frozen=True)
class GenerationTask:
    method_id: str
    script: Path
    method_definition: Path
    model: str
    experiment: str
    site: str


def validate_definition(definition: Dict[str, Any], study: Dict[str, Any]) -> None:
    generation = definition["weather_generation"]
    if generation.get("model_set") != "outer":
        raise cd.ContractError("Future study must use the frozen outer GCM set")
    if generation.get("experiments") != ["ssp245", "ssp585"]:
        raise cd.ContractError("Future study must retain both frozen SSP experiments")
    if generation.get("sites") != [site["id"] for site in cd.iter_sites(study)]:
        raise cd.ContractError("Future study sites differ from the study contract")
    if int(generation.get("members_per_state", 0)) < 2:
        raise cd.ContractError("Future ensemble requires at least two members per state")
    if generation.get("automatic_method_selection") is not False:
        raise cd.ContractError("Future generation cannot select a method automatically")
    for key in (
        "generator_probability_weights",
        "GCM_probability_weights",
        "SSP_probability_weights",
    ):
        if generation.get(key) is not None:
            raise cd.ContractError(f"Future study cannot assign {key}")
    methods = generation.get("methods", [])
    if [method.get("role") for method in methods] != [
        "frozen_VAR_baseline",
        "frozen_KNN_K10_candidate",
    ]:
        raise cd.ContractError("Future study must retain frozen VAR and KNN K10")
    for method in methods:
        method_path = cd.REPO_ROOT / method["definition"]
        if not method_path.exists():
            raise cd.ContractError(f"Missing method definition: {method_path}")
        if cd.sha256_file(method_path) != method["definition_sha256"]:
            raise cd.ContractError(
                f"Frozen method definition checksum changed: {method_path}"
            )
        payload = cd.load_json(method_path)
        if payload.get("method_id") != method["id"]:
            raise cd.ContractError("Method id differs from its frozen definition")
        if "KNN" in method["role"]:
            if int(payload["analog_model"]["neighbor_count"]) != 10:
                raise cd.ContractError("Future KNN definition is not frozen at K=10")
    expected_tasks = (
        len(cd.models_for_set(study, "outer"))
        * len(generation["experiments"])
        * len(generation["sites"])
        * len(methods)
    )
    if int(generation["expected_state_method_tasks"]) != expected_tasks:
        raise cd.ContractError("Declared future task count is inconsistent")
    if int(generation["expected_generated_members"]) != expected_tasks * int(
        generation["members_per_state"]
    ):
        raise cd.ContractError("Declared future member count is inconsistent")


def selected_values(available: Sequence[str], requested: Optional[Sequence[str]]) -> List[str]:
    if not requested:
        return list(available)
    unknown = sorted(set(requested) - set(available))
    if unknown:
        raise cd.ContractError(f"Unknown requested values: {unknown}")
    return [value for value in available if value in requested]


def build_tasks(
    definition: Dict[str, Any],
    study: Dict[str, Any],
    models: Optional[Sequence[str]] = None,
    experiments: Optional[Sequence[str]] = None,
    sites: Optional[Sequence[str]] = None,
    methods: Optional[Sequence[str]] = None,
) -> List[GenerationTask]:
    generation = definition["weather_generation"]
    selected_models = selected_values(cd.models_for_set(study, "outer"), models)
    selected_experiments = selected_values(generation["experiments"], experiments)
    selected_sites = selected_values(generation["sites"], sites)
    method_rows = generation["methods"]
    selected_method_ids = selected_values(
        [method["id"] for method in method_rows], methods
    )
    method_lookup = {method["id"]: method for method in method_rows}
    return [
        GenerationTask(
            method_id=method_id,
            script=cd.REPO_ROOT / method_lookup[method_id]["script"],
            method_definition=cd.REPO_ROOT
            / method_lookup[method_id]["definition"],
            model=model,
            experiment=experiment,
            site=site,
        )
        for model in selected_models
        for experiment in selected_experiments
        for site in selected_sites
        for method_id in selected_method_ids
    ]


def state_directory(
    output_root: Path, task: GenerationTask, study: Dict[str, Any]
) -> Path:
    start, end = (
        int(value)
        for value in (
            cd.nex_period_for_experiment(study, task.experiment)["start"][:4],
            cd.nex_period_for_experiment(study, task.experiment)["end"][:4],
        )
    )
    return (
        output_root
        / f"method={task.method_id}"
        / f"model={task.model}"
        / f"experiment={task.experiment}"
        / f"site={task.site}"
        / f"epoch={start}-{end}"
    )


def validate_outputs(
    directory: Path,
    task: GenerationTask,
    definition_hash: str,
    members: int,
) -> Optional[Dict[str, Any]]:
    fit_path = directory / "fit_provenance.json"
    parameter_path = directory / "parameters.npz"
    if not fit_path.exists() or not parameter_path.exists():
        return None
    fit = cd.load_json(fit_path)
    if fit.get("definition_sha256") != definition_hash:
        return None
    state = fit.get("conditioning_state", {})
    if (state.get("model"), state.get("experiment"), state.get("site")) != (
        task.model,
        task.experiment,
        task.site,
    ):
        return None
    if fit.get("parameter_archive_sha256") != cd.sha256_file(parameter_path):
        return None
    member_hashes = []
    for member in range(members):
        output = directory / f"member={member:04d}.nc"
        provenance = output.with_suffix(".provenance.json")
        if not output.exists() or not provenance.exists():
            return None
        report = cd.load_json(provenance)
        if report.get("member") != member:
            return None
        if report.get("definition_sha256") != definition_hash:
            return None
        output_hash = cd.sha256_file(output)
        if report.get("output_sha256") != output_hash:
            return None
        member_hashes.append(output_hash)
    return {
        "fit_provenance": cd.path_text(fit_path),
        "fit_provenance_sha256": cd.sha256_file(fit_path),
        "parameter_archive_sha256": cd.sha256_file(parameter_path),
        "member_count": members,
        "member_output_sha256": member_hashes,
    }


def execute_task(
    task: GenerationTask,
    study_path: Path,
    study: Dict[str, Any],
    output_root: Path,
    members: int,
    base_seed: int,
    execute: bool,
) -> Dict[str, Any]:
    directory = state_directory(output_root, task, study)
    definition_hash = cd.sha256_file(task.method_definition)
    verified = validate_outputs(directory, task, definition_hash, members)
    disposition = "verified_reused" if verified else "planned"
    if execute and verified is None:
        command = [
            sys.executable,
            str(task.script),
            "--study",
            str(study_path),
            "--definition",
            str(task.method_definition),
            "--model-set",
            "outer",
            "--model",
            task.model,
            "--experiment",
            task.experiment,
            "--site",
            task.site,
            "--members",
            str(members),
            "--base-seed",
            str(base_seed),
            "--output-root",
            str(output_root),
            "--overwrite",
        ]
        completed = subprocess.run(
            command,
            cwd=cd.REPO_ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            return {
                "valid": False,
                "method": task.method_id,
                "model": task.model,
                "experiment": task.experiment,
                "site": task.site,
                "error": completed.stderr.strip() or completed.stdout.strip(),
            }
        verified = validate_outputs(directory, task, definition_hash, members)
        if verified is None:
            return {
                "valid": False,
                "method": task.method_id,
                "model": task.model,
                "experiment": task.experiment,
                "site": task.site,
                "error": "Generator completed without a valid output inventory",
            }
        disposition = "generated_and_verified"
    return {
        "valid": True,
        "method": task.method_id,
        "model": task.model,
        "experiment": task.experiment,
        "site": task.site,
        "definition_sha256": definition_hash,
        "state_directory": cd.path_text(directory),
        "disposition": disposition,
        "outputs": verified,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--study", type=Path, default=cd.DEFAULT_STUDY)
    parser.add_argument("--definition", type=Path, default=DEFAULT_DEFINITION)
    parser.add_argument("--models", nargs="+")
    parser.add_argument("--experiments", nargs="+")
    parser.add_argument("--sites", nargs="+")
    parser.add_argument("--methods", nargs="+")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--progress", action="store_true")
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    study = cd.load_json(args.study)
    definition = cd.load_json(args.definition)
    validate_definition(definition, study)
    generation = definition["weather_generation"]
    output_root = args.output_root or (
        cd.REPO_ROOT / study["data_root"] / "processed/weather_generator_future"
    )
    tasks = build_tasks(
        definition,
        study,
        args.models,
        args.experiments,
        args.sites,
        args.methods,
    )
    if args.workers < 1:
        raise cd.ContractError("workers must be positive")
    records = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = [
            executor.submit(
                execute_task,
                task,
                args.study,
                study,
                output_root,
                int(generation["members_per_state"]),
                int(generation["base_seed"]),
                args.execute,
            )
            for task in tasks
        ]
        for future in concurrent.futures.as_completed(futures):
            record = future.result()
            records.append(record)
            if args.progress:
                state = "OK" if record["valid"] else "FAIL"
                print(
                    state,
                    record["method"],
                    record["model"],
                    record["experiment"],
                    record["site"],
                    record.get("disposition", record.get("error", "")),
                    flush=True,
                )
    records.sort(
        key=lambda row: (
            row["model"],
            row["experiment"],
            row["site"],
            row["method"],
        )
    )
    failures = [record for record in records if not record["valid"]]
    report = {
        "schema_version": "1.0",
        "operation": "paired_frozen_future_weather_batch_generation",
        "study_id": definition["study_id"],
        "executed": args.execute,
        "status": "complete" if args.execute and not failures else "planned" if not args.execute else "incomplete",
        "definition": cd.path_text(args.definition),
        "definition_sha256": cd.sha256_file(args.definition),
        "task_count": len(tasks),
        "tasks_completed": len(records) - len(failures) if args.execute else 0,
        "member_count": (
            (len(records) - len(failures)) * int(generation["members_per_state"])
            if args.execute
            else 0
        ),
        "failures": failures,
        "records": records,
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    output = args.output or (
        cd.REPO_ROOT
        / study["data_root"]
        / "provenance/future_weather_batch.json"
    )
    cd.write_json(output, report, args.overwrite)
    print(
        json.dumps(
            {key: value for key, value in report.items() if key != "records"},
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if not failures else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except cd.ContractError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2)
