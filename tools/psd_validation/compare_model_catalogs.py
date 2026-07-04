#!/usr/bin/env python3
"""Generate the HACDCPF vs PowerSimulationsDynamics model crosswalk report."""

from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_CROSSWALK = SCRIPT_DIR / "psd_hacdcpf_model_crosswalk.json"
DEFAULT_CATALOG_CPP = REPO_ROOT / "src" / "dynamics" / "DynamicModelCatalog.cpp"


CSV_COLUMNS = [
    "domain",
    "psd_slot",
    "psd_model",
    "hacdcpf_slot",
    "hacdcpf_model",
    "status",
    "comparison_level",
    "trace_case",
    "catalog_present",
    "notes",
]


def parse_hacdcpf_catalog(path: Path) -> Dict[str, Dict[str, str]]:
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(
        r'c\.push_back\(model\(\s*"(?P<name>[^"]+)"\s*,\s*"(?P<standard>[^"]*)"\s*,'
        r'\s*"(?P<display>[^"]*)"\s*,\s*"(?P<role>[^"]*)"',
        re.DOTALL,
    )
    catalog: Dict[str, Dict[str, str]] = {}
    for match in pattern.finditer(text):
        catalog[match.group("name")] = {
            "standard": match.group("standard"),
            "display": match.group("display"),
            "role": match.group("role"),
        }
    loop_pattern = re.compile(
        r'for\s*\(const char\*\s+\w+\s*:\s*\{(?P<names>[^}]+)\}\)\s*\{(?P<body>.*?)\n\s*\}',
        re.DOTALL,
    )
    model_call_pattern = re.compile(
        r'c\.push_back\(model\(\s*\w+\s*,\s*"(?P<standard>[^"]*)".*?,\s*"(?P<role>[^"]*)"',
        re.DOTALL,
    )
    for loop in loop_pattern.finditer(text):
        call = model_call_pattern.search(loop.group("body"))
        if not call:
            continue
        names = re.findall(r'"([^"]+)"', loop.group("names"))
        for name in names:
            catalog[name] = {
                "standard": call.group("standard"),
                "display": name,
                "role": call.group("role"),
            }
    return catalog


def load_crosswalk(path: Path) -> Dict[str, object]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if data.get("format") != "hacdcpf_psd_model_crosswalk.v1":
        raise ValueError(f"unexpected crosswalk format in {path}")
    if not isinstance(data.get("rows"), list):
        raise ValueError(f"crosswalk rows missing in {path}")
    return data


def annotate_rows(rows: Iterable[Dict[str, str]], catalog: Dict[str, Dict[str, str]]):
    annotated = []
    for row in rows:
        item = dict(row)
        model = item.get("hacdcpf_model", "")
        item["catalog_present"] = "yes" if model and model in catalog else ("n/a" if not model else "no")
        if model and model in catalog:
            item["hacdcpf_catalog_role"] = catalog[model]["role"]
            item["hacdcpf_catalog_standard"] = catalog[model]["standard"]
        else:
            item["hacdcpf_catalog_role"] = ""
            item["hacdcpf_catalog_standard"] = ""
        annotated.append(item)
    return annotated


def write_csv(path: Path, rows: List[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json(path: Path, payload: Dict[str, object], rows: List[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    out = dict(payload)
    out["rows"] = rows
    out["summary"] = summary(rows)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=2, sort_keys=True)
        handle.write("\n")


def clean_cell(value: object) -> str:
    text = "" if value is None else str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def summary(rows: List[Dict[str, str]]) -> Dict[str, object]:
    by_status = Counter(row.get("status", "") for row in rows)
    by_domain = Counter(row.get("domain", "") for row in rows)
    stale = [
        row
        for row in rows
        if row.get("hacdcpf_model") and row.get("catalog_present") == "no"
    ]
    return {
        "total_rows": len(rows),
        "by_status": dict(sorted(by_status.items())),
        "by_domain": dict(sorted(by_domain.items())),
        "stale_hacdcpf_model_refs": len(stale),
    }


def write_markdown(path: Path, payload: Dict[str, object], rows: List[Dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    stats = summary(rows)
    status_legend = payload.get("status_legend", {})
    level_legend = payload.get("comparison_levels", {})
    lines = [
        "# HACDCPF vs PowerSimulationsDynamics.jl Model Crosswalk",
        "",
        f"Generated from `{DEFAULT_CROSSWALK.name}`. Crosswalk updated: {payload.get('updated', 'unknown')}.",
        "",
        "This is a model-by-model and controller-by-controller comparison. It is not an equivalence claim; it tells the validation harness which pairs can be compared today and which pairs should only preserve metadata.",
        "",
        "## Summary",
        "",
        f"- Rows: {stats['total_rows']}",
    ]
    for status, count in stats["by_status"].items():
        lines.append(f"- {status}: {count}")
    if stats["stale_hacdcpf_model_refs"]:
        lines.append(f"- Stale HACDCPF model references: {stats['stale_hacdcpf_model_refs']}")
    lines.extend(["", "## Status Legend", ""])
    for key, text in status_legend.items():
        lines.append(f"- `{key}`: {text}")
    lines.extend(["", "## Comparison Levels", ""])
    for key, text in level_legend.items():
        lines.append(f"- `{key}`: {text}")
    lines.extend(
        [
            "",
            "## Crosswalk",
            "",
            "| Domain | PSD Slot | PSD Model / Controller | HACDCPF Slot | HACDCPF Model | Status | Level | Trace Case | Catalog | Notes |",
            "|---|---|---|---|---|---|---|---|---|---|",
        ]
    )
    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                clean_cell(row.get(col, ""))
                for col in [
                    "domain",
                    "psd_slot",
                    "psd_model",
                    "hacdcpf_slot",
                    "hacdcpf_model",
                    "status",
                    "comparison_level",
                    "trace_case",
                    "catalog_present",
                    "notes",
                ]
            )
            + " |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--crosswalk", type=Path, default=DEFAULT_CROSSWALK)
    parser.add_argument("--catalog-cpp", type=Path, default=DEFAULT_CATALOG_CPP)
    parser.add_argument("--out-md", type=Path)
    parser.add_argument("--out-csv", type=Path)
    parser.add_argument("--out-json", type=Path)
    parser.add_argument("--fail-on-stale", action="store_true")
    parser.add_argument("--print-summary", action="store_true")
    args = parser.parse_args()

    payload = load_crosswalk(args.crosswalk)
    catalog = parse_hacdcpf_catalog(args.catalog_cpp)
    rows = annotate_rows(payload["rows"], catalog)
    stats = summary(rows)

    if args.out_md:
        write_markdown(args.out_md, payload, rows)
    if args.out_csv:
        write_csv(args.out_csv, rows)
    if args.out_json:
        write_json(args.out_json, payload, rows)

    if args.print_summary or not (args.out_md or args.out_csv or args.out_json):
        print(json.dumps(stats, indent=2, sort_keys=True))

    if args.fail_on_stale and stats["stale_hacdcpf_model_refs"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
