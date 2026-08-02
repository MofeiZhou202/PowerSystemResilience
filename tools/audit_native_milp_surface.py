#!/usr/bin/env python3
"""Reproduce the Native MILP public-surface and placeholder census."""

from __future__ import annotations

import argparse
import collections
import json
import pathlib
import re
import sys
from typing import Iterable


SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
MARKER_RE = re.compile(
    r"\b(?:TODO|FIXME|HACK|XXX)\b|scaffold|not[ -]implemented|"
    r"\bplaceholders?\b|\bfake(?:d)?\b|\bstub(?:s|bed)?\b|\bunimplemented\b",
    re.IGNORECASE,
)
GETENV_RE = re.compile(r"\b(?:(?:std|boost)::)?getenv\s*\(")
ROADMAP_RE = re.compile(r"\bP[0-9]+(?:\.[0-9]+)?\b")
UNQUALIFIED_PERFORMANCE_CLAIM_RE = re.compile(
    r"\b(?:faster|fastest|speed-?up)\b|\bmeasured\s+(?:on|to|rationale)\b",
    re.IGNORECASE,
)


def source_files(roots: Iterable[pathlib.Path]) -> list[pathlib.Path]:
    files: list[pathlib.Path] = []
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*"):
            if path.is_file() and path.suffix.lower() in SOURCE_SUFFIXES:
                if "extern" not in path.parts:
                    files.append(path)
    return sorted(set(files))


def strip_comments_and_literals(text: str) -> str:
    pattern = re.compile(
        r"//[^\n]*|/\*.*?\*/|R\"([^\s\\()]*)\(.*?\)\1\"|"
        r"\"(?:\\.|[^\"\\])*\"|(?<![A-Za-z0-9_])'(?:\\.|[^'\\])*'",
        re.DOTALL,
    )
    return pattern.sub(lambda match: "\n" * match.group(0).count("\n"), text)


def struct_body(text: str, name: str) -> str:
    cleaned = strip_comments_and_literals(text)
    match = re.search(rf"\bstruct\s+{re.escape(name)}\s*\{{", cleaned)
    if match is None:
        raise ValueError(f"struct {name} not found")
    start = match.end()
    depth = 1
    for index in range(start, len(cleaned)):
        char = cleaned[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return cleaned[start:index]
    raise ValueError(f"unterminated struct {name}")


def top_level_statements(body: str) -> list[str]:
    statements: list[str] = []
    start = 0
    brace_depth = 0
    paren_depth = 0
    bracket_depth = 0
    angle_depth = 0
    for index, char in enumerate(body):
        if char == "{":
            brace_depth += 1
        elif char == "}":
            brace_depth -= 1
        elif char == "(":
            paren_depth += 1
        elif char == ")":
            paren_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]":
            bracket_depth -= 1
        elif char == "<":
            angle_depth += 1
        elif char == ">" and angle_depth:
            angle_depth -= 1
        elif (
            char == ";"
            and brace_depth == 0
            and paren_depth == 0
            and bracket_depth == 0
            and angle_depth == 0
        ):
            statements.append(body[start:index].strip())
            start = index + 1
    return statements


def declared_fields(header: pathlib.Path, struct_name: str) -> list[str]:
    fields: list[str] = []
    for statement in top_level_statements(struct_body(header.read_text(), struct_name)):
        declaration = statement.split("=", 1)[0].split("{", 1)[0].strip()
        if not declaration or "(" in declaration or declaration.startswith(
            ("using ", "typedef ", "static ")
        ):
            continue
        match = re.search(r"([A-Za-z_]\w*)\s*$", declaration)
        if match is not None:
            fields.append(match.group(1))
    if len(fields) != len(set(fields)):
        raise ValueError(f"duplicate extracted fields in {struct_name}")
    return fields


def reference_counts(fields: list[str], corpus: str) -> dict[str, int]:
    observed = collections.Counter(
        re.findall(r"(?:\.|->)\s*([A-Za-z_]\w*)", corpus)
    )
    return {field: observed[field] for field in fields}


def mutation_counts(fields: list[str], corpus: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    mutating_methods = (
        "assign|clear|emplace|emplace_back|erase|insert|pop_back|"
        "push_back|resize|swap"
    )
    for field in fields:
        member = rf"(?:\.|->)\s*{re.escape(field)}\b"
        suffix = rf"{member}\s*(?:\+\+|--|(?:<<|>>|[+\-*/%&|^])?=(?!=))"
        prefix = rf"(?:\+\+|--)[^;\n]{{0,120}}{member}"
        method_call = rf"{member}\s*\.\s*(?:{mutating_methods})\s*\("
        erase_if = rf"\b(?:std::)?erase_if\s*\(\s*[^,;\n]*{member}\s*,"
        counts[field] = sum(
            len(re.findall(pattern, corpus))
            for pattern in (suffix, prefix, method_call, erase_if)
        )
    return counts


def line_hits(paths: Iterable[pathlib.Path], pattern: re.Pattern[str]) -> list[str]:
    hits: list[str] = []
    for path in paths:
        for line_number, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
            if pattern.search(line):
                hits.append(f"{path}:{line_number}:{line.strip()}")
    return hits


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=".")
    parser.add_argument("--json", dest="json_path")
    args = parser.parse_args()

    repo = pathlib.Path(args.repo_root).resolve()
    usage_paths = source_files(
        repo / name for name in ("include", "src", "benchmark", "tests")
    )
    corpus = "\n".join(
        strip_comments_and_literals(path.read_text(errors="replace"))
        for path in usage_paths
    )
    production_paths = source_files(repo / name for name in ("include", "src"))
    production_corpus = "\n".join(
        strip_comments_and_literals(path.read_text(errors="replace"))
        for path in production_paths
    )

    options_header = repo / "include/mipsolvers/engine/bc/options.hpp"
    stats_header = repo / "include/mipsolvers/engine/bc/stats.hpp"
    option_fields = declared_fields(options_header, "BCOptions")
    stats_fields = declared_fields(stats_header, "BCStats")
    option_counts = reference_counts(option_fields, corpus)
    stats_counts = reference_counts(stats_fields, corpus)
    production_option_counts = reference_counts(option_fields, production_corpus)
    production_stat_mutations = mutation_counts(stats_fields, production_corpus)

    native_paths = source_files(
        (
            repo / "include/mipsolvers/engine/bc",
            repo / "include/mipsolvers/engine/detail",
            repo / "src/engine/solver/native/milp/bc",
        )
    )
    marker_hits = line_hits(native_paths, MARKER_RE)
    roadmap_hits = line_hits(native_paths, ROADMAP_RE)
    native_claim_paths = sorted(
        set(native_paths + [repo / "src/engine/solver/native/native_adapters.cpp"])
    )
    performance_claim_hits = line_hits(
        native_claim_paths, UNQUALIFIED_PERFORMANCE_CLAIM_RE
    )
    direct_getenv_hits: list[str] = []
    for path in native_paths:
        cleaned = strip_comments_and_literals(path.read_text(errors="replace"))
        for line_number, line in enumerate(cleaned.splitlines(), 1):
            if GETENV_RE.search(line):
                direct_getenv_hits.append(f"{path}:{line_number}:{line.strip()}")

    report = {
        "schema": "native-milp-surface-census-v1",
        "options": {
            "declared_fields": len(option_fields),
            "zero_reference_fields": sorted(
                field for field, count in option_counts.items() if count == 0
            ),
            "zero_production_reference_fields": sorted(
                field
                for field, count in production_option_counts.items()
                if count == 0
            ),
        },
        "stats": {
            "declared_fields": len(stats_fields),
            "zero_reference_fields": sorted(
                field for field, count in stats_counts.items() if count == 0
            ),
            "zero_explicit_production_mutation_fields": sorted(
                field
                for field, count in production_stat_mutations.items()
                if count == 0
            ),
        },
        "native_scope": {
            "direct_getenv_hits": direct_getenv_hits,
            "placeholder_marker_hits": marker_hits,
            "roadmap_marker_hits": roadmap_hits,
            "unqualified_performance_claim_hits": performance_claim_hits,
        },
    }

    encoded = json.dumps(report, indent=2, sort_keys=True)
    print(encoded)
    if args.json_path:
        output = pathlib.Path(args.json_path)
        if not output.is_absolute():
            output = repo / output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded + "\n")

    failed = any(
        (
            report["options"]["zero_reference_fields"],
            report["options"]["zero_production_reference_fields"],
            report["stats"]["zero_reference_fields"],
            report["stats"]["zero_explicit_production_mutation_fields"],
            direct_getenv_hits,
            marker_hits,
            roadmap_hits,
            performance_claim_hits,
        )
    )
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
