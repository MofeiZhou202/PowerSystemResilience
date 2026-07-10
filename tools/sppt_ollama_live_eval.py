#!/usr/bin/env python3
"""Live Ollama prompt-to-action smoke test for the SPPT guarded agent interface.

The local LLM is deliberately constrained: it may only choose one stable SPPT
action id. The selected action is then executed by sppt_agent_demo, so the SPPT
guard and numerical solver remain the source of accepted/rejected physics.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


ACTION_IDS = [
    "scale_loads_10pct",
    "remove_ac_references",
    "restore_slack_reference",
    "trim_generation_headroom_5pct",
    "no_op",
]

TEX_ACTION_LABELS = {
    "scale_loads_10pct": "scale loads +10%",
    "remove_ac_references": "remove AC refs",
    "restore_slack_reference": "restore slack",
    "trim_generation_headroom_5pct": "trim gen -5%",
    "no_op": "no op",
}

PROMPT_CASES = [
    {
        "id": "load_plus_10",
        "prompt": "Increase all AC loads by 10 percent while preserving references.",
        "expected_action": "scale_loads_10pct",
        "expected_guard_accept": True,
    },
    {
        "id": "delete_refs",
        "prompt": "Delete every AC slack or angle reference and turn off external grids.",
        "expected_action": "remove_ac_references",
        "expected_guard_accept": False,
    },
    {
        "id": "restore_slack",
        "prompt": "Restore a slack reference at the first AC bus.",
        "expected_action": "restore_slack_reference",
        "expected_guard_accept": True,
    },
    {
        "id": "trim_generation",
        "prompt": "Reduce all generator active power setpoints by five percent.",
        "expected_action": "trim_generation_headroom_5pct",
        "expected_guard_accept": True,
    },
    {
        "id": "inspect_only",
        "prompt": "Do not change the model; only run the guard.",
        "expected_action": "no_op",
        "expected_guard_accept": True,
    },
]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def latex_escape(value: Any) -> str:
    text = str(value)
    repl = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(repl.get(ch, ch) for ch in text)


def request_json(url: str, payload: dict[str, Any] | None, timeout: float) -> dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def choose_model(base_url: str, timeout: float) -> str:
    tags = request_json(f"{base_url.rstrip('/')}/api/tags", None, timeout)
    models = tags.get("models", [])
    if not models:
        raise RuntimeError("Ollama is reachable but no local models are installed")
    return str(models[0]["name"])


def extract_json_object(text: str) -> dict[str, Any]:
    stripped = text.strip()
    try:
        obj = json.loads(stripped)
    except json.JSONDecodeError:
        start = stripped.find("{")
        end = stripped.rfind("}")
        if start < 0 or end < start:
            raise
        obj = json.loads(stripped[start : end + 1])
    if not isinstance(obj, dict):
        raise ValueError("model response was JSON but not an object")
    return obj


def call_ollama(base_url: str, model: str, task_prompt: str, timeout: float) -> tuple[str, dict[str, Any]]:
    system = (
        "You are an SPPT tool-call adapter for hybrid AC/DC power-system modeling. "
        "Return exactly one JSON object and no Markdown. Do not solve power flow, "
        "do not invent voltages, residuals, or converter flows. Map the operator "
        "request to the closest allowed action id; the SPPT guard will accept or "
        "reject the resulting physics."
    )
    user = (
        f"{system}\n\n"
        "Allowed action ids:\n"
        "- scale_loads_10pct: increase all AC loads by 10 percent.\n"
        "- remove_ac_references: remove all AC slack/angle references and disable external grids.\n"
        "- restore_slack_reference: make the first AC bus a slack/reference bus.\n"
        "- trim_generation_headroom_5pct: reduce generator active-power setpoints by 5 percent.\n"
        "- no_op: leave the model unchanged.\n\n"
        "Return schema: {\"action_id\":\"one allowed action id\",\"rationale\":\"short reason\"}\n"
        f"Operator request: {task_prompt}"
    )
    payload = {
        "model": model,
        "prompt": user,
        "stream": False,
        "format": "json",
        "options": {"temperature": 0, "num_predict": 128},
    }
    started = time.perf_counter()
    data = request_json(f"{base_url.rstrip('/')}/api/generate", payload, timeout)
    elapsed = time.perf_counter() - started
    response = str(data.get("response", ""))
    parsed = extract_json_object(response)
    parsed["_elapsed_s"] = elapsed
    parsed["_raw_response"] = response
    return response, parsed


def run_guard(binary: Path, case_path: Path, action_id: str, timeout: float) -> dict[str, Any]:
    proc = subprocess.run(
        [str(binary), "--actions", action_id, str(case_path)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    output = proc.stdout + proc.stderr
    match = re.search(r"verdict\s*:\s*(ACCEPT|REJECT)(?:\s*\(([^)]+)\))?", output)
    verdict = match.group(1).lower() if match else "unparsed"
    reason = match.group(2) if match and match.group(2) else ""
    return {
        "returncode": proc.returncode,
        "guard_verdict": verdict,
        "guard_reason": reason,
        "guard_output": output,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "id",
        "prompt",
        "expected_action",
        "expected_guard",
        "ollama_action",
        "parse_ok",
        "action_match",
        "guard_verdict",
        "guard_matches_request",
        "elapsed_s",
        "rationale",
        "raw_response",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fields})


def write_tex(path: Path, model: str, rows: list[dict[str, Any]]) -> None:
    n = len(rows)
    parse_ok = sum(1 for r in rows if r["parse_ok"])
    action_ok = sum(1 for r in rows if r["action_match"])
    guard_ok = sum(1 for r in rows if r["guard_matches_request"])
    with path.open("w") as f:
        f.write("% Auto-generated live Ollama prompt-to-action smoke test.\n")
        f.write(r"\begin{tabularx}{\linewidth}{@{}X X X c c c@{}}" + "\n")
        f.write(r"\toprule" + "\n")
        f.write(
            r"\multicolumn{6}{@{}l}{Model: \code{"
            + latex_escape(model)
            + r"}; prompts: "
            + str(n)
            + f"; JSON {parse_ok}/{n}; action {action_ok}/{n}; guard {guard_ok}/{n}."
            + r"} \\"
            + "\n"
        )
        f.write(r"Prompt & Expected action & Ollama action & JSON & Action & Guard \\" + "\n")
        f.write(r"\midrule" + "\n")
        for row in rows:
            guard = row["guard_verdict"]
            if row.get("guard_reason"):
                guard += f" ({row['guard_reason']})"
            f.write(
                f"{latex_escape(row['prompt'])} & "
                f"{latex_escape(TEX_ACTION_LABELS.get(row['expected_action'], row['expected_action']))} & "
                f"{latex_escape(TEX_ACTION_LABELS.get(row['ollama_action'], row['ollama_action']))} & "
                f"{'yes' if row['parse_ok'] else 'no'} & "
                f"{'yes' if row['action_match'] else 'no'} & "
                f"{latex_escape(guard)} \\\\\n"
            )
        f.write(r"\bottomrule" + "\n")
        f.write(r"\end{tabularx}" + "\n")


def main() -> int:
    root = repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ollama-url", default="http://127.0.0.1:11434")
    parser.add_argument("--model", default="")
    parser.add_argument("--case", type=Path, default=root / "data" / "case9.m")
    parser.add_argument(
        "--sppt-agent-demo",
        type=Path,
        default=root / "build" / "macos-release" / "sppt_agent_demo",
    )
    parser.add_argument("--outdir", type=Path, default=root / "docs" / "latex")
    parser.add_argument("--timeout", type=float, default=240.0)
    args = parser.parse_args()

    if not args.sppt_agent_demo.exists():
        raise SystemExit(f"missing sppt_agent_demo binary: {args.sppt_agent_demo}")
    if not args.case.exists():
        raise SystemExit(f"missing case file: {args.case}")

    try:
        model = args.model or choose_model(args.ollama_url, args.timeout)
    except (urllib.error.URLError, TimeoutError) as exc:
        raise SystemExit(f"Ollama is not reachable at {args.ollama_url}: {exc}") from exc

    rows: list[dict[str, Any]] = []
    for case in PROMPT_CASES:
        row: dict[str, Any] = {
            "id": case["id"],
            "prompt": case["prompt"],
            "expected_action": case["expected_action"],
            "expected_guard": "accept" if case["expected_guard_accept"] else "reject",
            "parse_ok": False,
            "action_match": False,
            "guard_verdict": "not-run",
            "guard_matches_request": False,
            "ollama_action": "",
            "elapsed_s": "",
            "rationale": "",
            "raw_response": "",
        }
        try:
            raw, parsed = call_ollama(args.ollama_url, model, case["prompt"], args.timeout)
            action_id = str(parsed.get("action_id", ""))
            row.update(
                {
                    "parse_ok": action_id in ACTION_IDS,
                    "ollama_action": action_id,
                    "action_match": action_id == case["expected_action"],
                    "elapsed_s": f"{parsed.get('_elapsed_s', 0.0):.3f}",
                    "rationale": str(parsed.get("rationale", "")),
                    "raw_response": raw,
                }
            )
            if row["parse_ok"]:
                guard = run_guard(args.sppt_agent_demo, args.case, action_id, args.timeout)
                row.update(guard)
                accepted = row["guard_verdict"] == "accept"
                row["guard_matches_request"] = accepted == case["expected_guard_accept"]
        except Exception as exc:  # keep the benchmark auditable even on one bad row
            row["raw_response"] = f"ERROR: {exc}"
        rows.append(row)

    args.outdir.mkdir(parents=True, exist_ok=True)
    write_csv(args.outdir / "sppt_ollama_live_eval.csv", rows)
    write_tex(args.outdir / "sppt_ollama_live_eval.tex", model, rows)
    (args.outdir / "sppt_ollama_live_eval.json").write_text(
        json.dumps({"model": model, "rows": rows}, indent=2),
        encoding="utf-8",
    )

    n = len(rows)
    parse_ok = sum(1 for r in rows if r["parse_ok"])
    action_ok = sum(1 for r in rows if r["action_match"])
    guard_ok = sum(1 for r in rows if r["guard_matches_request"])
    print(
        f"Ollama live SPPT smoke test: model={model} "
        f"json={parse_ok}/{n} action={action_ok}/{n} guard={guard_ok}/{n}"
    )
    for row in rows:
        print(
            f"- {row['id']}: expected={row['expected_action']} "
            f"ollama={row['ollama_action'] or '<none>'} "
            f"guard={row['guard_verdict']} action_match={row['action_match']}"
        )
    return 0 if parse_ok == n and guard_ok == n else 1


if __name__ == "__main__":
    sys.exit(main())
