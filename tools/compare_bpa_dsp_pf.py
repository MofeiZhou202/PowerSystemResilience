#!/usr/bin/env python3
"""Compare a HySim GUI power-flow JSON report with a DSP .pf report.

The script uses only the Python standard library.  It matches buses by BPA
name plus base voltage, aligns voltage angles at the slack bus, and then uses
the matched buses to identify generators and AC branches.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import re
import statistics
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Callable, Iterable


NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?"
BUS_RE = re.compile(
    rf"^\s*(?P<seq>\d+)\s+(?P<label>.+?)\s+"
    rf"(?P<vm>{NUMBER})\s+(?P<kv>{NUMBER})\s+(?P<angle>{NUMBER})\s*$"
)
GEN_RE = re.compile(
    rf"^\s*(?P<seq>\d+)\s+(?P<label>.+?)\s+'(?P<generator_id>[^']*)'\s+"
    rf"(?P<p>{NUMBER})\s+(?P<q>{NUMBER})\s+(?P<qmax>{NUMBER})\s+"
    rf"(?P<qmin>{NUMBER})\s+(?P<actual_vm>{NUMBER})\s+"
    rf"(?P<control_vm>{NUMBER})(?:\s+.*)?$"
)
DECIMAL_TOKEN_RE = re.compile(
    r"^[+-]?(?:\d+\.\d*|\.\d+)(?:[Ee][+-]?\d+)?$"
)

BUS_MARKER = ">>>\u8282\u70b9\u7535\u538b"
GEN_MARKER = ">>>\u53d1\u7535\u673a\u51fa\u529b"
BRANCH_MARKER = ">>>\u652f\u8def\u6f6e\u6d41"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare bus, generator, and AC-branch power-flow results from "
            "a HySim GUI JSON report and a DSP .pf report."
        )
    )
    parser.add_argument("hysim_json", type=Path, help="HySim GUI PF result JSON")
    parser.add_argument("dsp_pf", type=Path, help="DSP power-flow .pf report")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="Output directory (default: comparison_<json>_vs_<pf>)",
    )
    parser.add_argument(
        "--pf-encoding",
        default="auto",
        help="DSP PF encoding (default: auto; normally GB18030/GBK)",
    )
    parser.add_argument(
        "--angle-align",
        choices=("slack", "median", "island", "none"),
        default="slack",
        help="Voltage-angle reference alignment (default: slack; island aligns each AC island)",
    )
    parser.add_argument("--vm-tol", type=float, default=1e-3, help="Bus Vm tolerance in pu")
    parser.add_argument(
        "--angle-tol-deg", type=float, default=0.1, help="Bus angle tolerance in degrees"
    )
    parser.add_argument("--p-tol-mw", type=float, default=0.5, help="P tolerance in MW")
    parser.add_argument("--q-tol-mvar", type=float, default=0.5, help="Q tolerance in Mvar")
    parser.add_argument(
        "--fail-on-tolerance",
        action="store_true",
        help="Return exit code 2 when any matched row exceeds a tolerance",
    )
    return parser.parse_args()


def read_dsp_text(path: Path, requested_encoding: str) -> tuple[str, str]:
    data = path.read_bytes()
    encodings = (
        (requested_encoding,)
        if requested_encoding != "auto"
        else ("utf-8-sig", "gb18030", "gbk")
    )
    errors: list[str] = []
    for encoding in encodings:
        try:
            return data.decode(encoding), encoding
        except UnicodeDecodeError as exc:
            errors.append(f"{encoding}: {exc}")
    raise ValueError("Cannot decode DSP PF report; " + "; ".join(errors))


def collect_table(
    lines: list[str], marker: str, parser: Callable[[str], dict[str, Any] | None]
) -> list[dict[str, Any]]:
    try:
        start = next(i for i, line in enumerate(lines) if line.startswith(marker))
    except StopIteration as exc:
        raise ValueError(f"DSP PF section not found: {marker}") from exc

    rows: list[dict[str, Any]] = []
    for line in lines[start + 1 :]:
        if line.startswith(">>>"):
            break
        parsed = parser(line)
        if parsed is not None:
            rows.append(parsed)
        elif rows and not line.strip():
            break
    if not rows:
        raise ValueError(f"DSP PF section is empty or unsupported: {marker}")
    return rows


def parse_bus_line(line: str) -> dict[str, Any] | None:
    match = BUS_RE.match(line)
    if not match:
        return None
    return {
        "seq": int(match.group("seq")),
        "label": match.group("label").strip(),
        "vm_pu": float(match.group("vm")),
        "voltage_kv": float(match.group("kv")),
        "va_deg": float(match.group("angle")),
    }


def parse_generator_line(line: str) -> dict[str, Any] | None:
    match = GEN_RE.match(line)
    if not match:
        return None
    return {
        "bus_seq": int(match.group("seq")),
        "label": match.group("label").strip(),
        "generator_id": match.group("generator_id").strip(),
        "p_mw": float(match.group("p")),
        "q_mvar": float(match.group("q")),
        "qmax_mvar": float(match.group("qmax")),
        "qmin_mvar": float(match.group("qmin")),
        "actual_vm_pu": float(match.group("actual_vm")),
        "control_vm_pu": float(match.group("control_vm")),
    }


def _find_endpoint(
    line: str, start: int, bus_by_seq: dict[int, dict[str, Any]]
) -> tuple[int, str, int] | None:
    seq_match = re.match(r"\s*(\d+)\s+", line[start:])
    if not seq_match:
        return None
    seq = int(seq_match.group(1))
    bus = bus_by_seq.get(seq)
    if bus is None:
        return None
    label = bus["label"]
    label_start = start + seq_match.end()
    position = line.find(label, label_start)
    if position < 0 or line[label_start:position].strip():
        return None
    return seq, label, position + len(label)


def make_branch_parser(
    bus_by_seq: dict[int, dict[str, Any]]
) -> Callable[[str], dict[str, Any] | None]:
    def parse_branch_line(line: str) -> dict[str, Any] | None:
        first = _find_endpoint(line, 0, bus_by_seq)
        if first is None:
            return None
        from_seq, from_label, after_first = first
        second = _find_endpoint(line, after_first, bus_by_seq)
        if second is None:
            return None
        to_seq, to_label, after_second = second
        tokens = line[after_second:].split()
        try:
            type_position = next(i for i, token in enumerate(tokens) if token in ("L", "T"))
        except StopIteration:
            return None

        before_type = tokens[:type_position]
        if not before_type:
            return None
        circuit = ""
        flow_start = 0
        if not DECIMAL_TOKEN_RE.match(before_type[0]):
            circuit = before_type[0]
            flow_start = 1
        if len(before_type) < flow_start + 6:
            return None
        try:
            flows = [float(value) for value in before_type[flow_start : flow_start + 6]]
            parameters = [float(value) for value in tokens[type_position + 1 : type_position + 5]]
        except ValueError:
            return None
        if len(parameters) != 4:
            return None
        return {
            "from_seq": from_seq,
            "from_label": from_label,
            "to_seq": to_seq,
            "to_label": to_label,
            "circuit": circuit,
            "pf_mw": flows[0],
            "qf_mvar": flows[1],
            "pt_mw": flows[2],
            "qt_mvar": flows[3],
            "p_loss_mw": flows[4],
            "q_loss_mvar": flows[5],
            "branch_type": tokens[type_position],
            "r_pu": parameters[0],
            "x_pu": parameters[1],
            "b1_or_tap1": parameters[2],
            "b2_or_tap2": parameters[3],
        }

    return parse_branch_line


def parse_dsp_pf(path: Path, requested_encoding: str) -> dict[str, Any]:
    text, encoding = read_dsp_text(path, requested_encoding)
    lines = text.splitlines()
    buses = collect_table(lines, BUS_MARKER, parse_bus_line)
    bus_by_seq = {row["seq"]: row for row in buses}
    generators = collect_table(lines, GEN_MARKER, parse_generator_line)
    branches = collect_table(lines, BRANCH_MARKER, make_branch_parser(bus_by_seq))
    convergence = re.search(
        r"POWER FLOW SOLUTION IS REACHED IN\s+(\d+)\s+ITERATIONS", text, re.IGNORECASE
    )
    return {
        "encoding": encoding,
        "converged": convergence is not None,
        "iterations": int(convergence.group(1)) if convergence else None,
        "buses": buses,
        "generators": generators,
        "branches": branches,
    }


def load_hysim_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8-sig") as stream:
        report = json.load(stream)
    required = ("geo_buses", "geo_gen", "geo_ac_branches")
    missing = [name for name in required if not isinstance(report.get(name), list)]
    if missing:
        raise ValueError("HySim JSON is missing result arrays: " + ", ".join(missing))
    return report


def normalize_label(value: Any) -> str:
    return "".join(str(value).split()).casefold()


def base_voltage_variants(base_kv: float) -> set[str]:
    variants: set[str] = set()
    for digits in range(0, 7):
        value = f"{base_kv:.{digits}f}"
        variants.add(value)
        if "." in value:
            stripped = value.rstrip("0")
            variants.add(stripped)
            variants.add(stripped.rstrip("."))
    variants.add(f"{base_kv:.12g}")
    if math.isclose(base_kv, round(base_kv), abs_tol=1e-9):
        variants.add(f"{int(round(base_kv))}.")
    for value in list(variants):
        if value.startswith("0."):
            variants.add(value[1:])
        elif value.startswith("-0."):
            variants.add("-." + value[3:])
    return {value for value in variants if value not in ("", "-")}


def bus_aliases(bus: dict[str, Any]) -> set[str]:
    name = normalize_label(bus.get("name", ""))
    base_kv = float(bus.get("base_kv", 0.0))
    return {name + normalize_label(value) for value in base_voltage_variants(base_kv)}


def match_buses(
    hysim_buses: list[dict[str, Any]], dsp_buses: list[dict[str, Any]]
) -> tuple[dict[int, dict[str, Any]], list[dict[str, Any]]]:
    alias_map: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for bus in hysim_buses:
        for alias in bus_aliases(bus):
            alias_map[alias].append(bus)

    matches: dict[int, dict[str, Any]] = {}
    used_ids: set[int] = set()
    unmatched: list[dict[str, Any]] = []
    for dsp_bus in dsp_buses:
        label = normalize_label(dsp_bus["label"])
        candidates = [
            bus
            for bus in alias_map.get(label, [])
            if int(bus["id"]) not in used_ids
        ]
        if len(candidates) != 1:
            fallback: list[dict[str, Any]] = []
            for bus in hysim_buses:
                bus_id = int(bus["id"])
                if bus_id in used_ids:
                    continue
                name = normalize_label(bus.get("name", ""))
                if not label.startswith(name):
                    continue
                suffix = label[len(name) :]
                try:
                    parsed_base = float(suffix)
                except ValueError:
                    continue
                if math.isclose(parsed_base, float(bus["base_kv"]), rel_tol=0.0, abs_tol=1e-6):
                    fallback.append(bus)
            candidates = fallback
        if len(candidates) == 1:
            bus = candidates[0]
            matches[int(dsp_bus["seq"])] = bus
            used_ids.add(int(bus["id"]))
        else:
            unmatched.append(
                {
                    "category": "bus",
                    "side": "dsp",
                    "identity": dsp_bus["label"],
                    "reason": "no_unique_name_and_base_voltage_match",
                }
            )

    for bus in hysim_buses:
        if int(bus["id"]) not in used_ids:
            unmatched.append(
                {
                    "category": "bus",
                    "side": "hysim",
                    "identity": f"{bus.get('name', '')}@{bus.get('base_kv', '')}kV",
                    "reason": "not_present_in_dsp_bus_table",
                }
            )
    return matches, unmatched


def choose_angle_offset(
    mode: str,
    raw_deltas: list[tuple[dict[str, Any], dict[str, Any], float]],
) -> tuple[float, str]:
    if mode == "none" or not raw_deltas:
        return 0.0, "none"
    if mode == "slack":
        for hysim_bus, _dsp_bus, delta in raw_deltas:
            if str(hysim_bus.get("bus_type", "")).casefold() == "slack":
                return delta, "slack"
    return statistics.median(delta for _hysim, _dsp, delta in raw_deltas), "median"


def ac_island_by_bus(
    hysim_buses: list[dict[str, Any]], hysim_branches: list[dict[str, Any]]
) -> dict[int, int]:
    adjacency: dict[int, set[int]] = {
        int(bus["id"]): set() for bus in hysim_buses if bus.get("type", "AC") == "AC"
    }
    for branch in hysim_branches:
        if branch.get("in_service", True) is False:
            continue
        from_bus = int(branch["from"])
        to_bus = int(branch["to"])
        if from_bus not in adjacency or to_bus not in adjacency:
            continue
        adjacency[from_bus].add(to_bus)
        adjacency[to_bus].add(from_bus)

    component_by_bus: dict[int, int] = {}
    component = 0
    for start in adjacency:
        if start in component_by_bus:
            continue
        queue = deque([start])
        component_by_bus[start] = component
        while queue:
            current = queue.popleft()
            for neighbour in adjacency[current]:
                if neighbour not in component_by_bus:
                    component_by_bus[neighbour] = component
                    queue.append(neighbour)
        component += 1
    return component_by_bus


def compare_buses(
    hysim_buses: list[dict[str, Any]],
    dsp_buses: list[dict[str, Any]],
    matches: dict[int, dict[str, Any]],
    angle_mode: str,
    vm_tol: float,
    angle_tol: float,
    component_by_bus: dict[int, int] | None = None,
) -> tuple[list[dict[str, Any]], float, str, list[dict[str, Any]]]:
    raw_deltas: list[tuple[dict[str, Any], dict[str, Any], float]] = []
    for dsp_bus in dsp_buses:
        hysim_bus = matches.get(int(dsp_bus["seq"]))
        if hysim_bus is None:
            continue
        hysim_angle = math.degrees(float(hysim_bus.get("va_rad", 0.0)))
        raw_deltas.append((hysim_bus, dsp_bus, hysim_angle - float(dsp_bus["va_deg"])))
    island_offsets: dict[int, tuple[float, str]] = {}
    if angle_mode == "island":
        grouped: dict[int, list[tuple[dict[str, Any], dict[str, Any], float]]] = defaultdict(list)
        for item in raw_deltas:
            grouped[(component_by_bus or {}).get(int(item[0]["id"]), -1)].append(item)
        for component, items in grouped.items():
            island_offsets[component] = choose_angle_offset("slack", items)
        angle_offset, angle_source = 0.0, "per-island slack/median"
    else:
        angle_offset, angle_source = choose_angle_offset(angle_mode, raw_deltas)

    rows: list[dict[str, Any]] = []
    for hysim_bus, dsp_bus, raw_angle_delta in raw_deltas:
        vm_delta = float(hysim_bus["vm_pu"]) - float(dsp_bus["vm_pu"])
        island = (component_by_bus or {}).get(int(hysim_bus["id"]), -1)
        row_offset = island_offsets.get(island, (angle_offset, angle_source))[0]
        aligned_angle_delta = raw_angle_delta - row_offset
        rows.append(
            {
                "match_status": "matched",
                "dsp_seq": dsp_bus["seq"],
                "dsp_label": dsp_bus["label"],
                "hysim_bus_id": hysim_bus["id"],
                "hysim_name": hysim_bus.get("name", ""),
                "base_kv": hysim_bus.get("base_kv", ""),
                "bus_type": hysim_bus.get("bus_type", ""),
                "dsp_vm_pu": dsp_bus["vm_pu"],
                "hysim_vm_pu": hysim_bus["vm_pu"],
                "delta_vm_pu": vm_delta,
                "dsp_va_deg": dsp_bus["va_deg"],
                "hysim_va_deg": raw_angle_delta + float(dsp_bus["va_deg"]),
                "raw_delta_va_deg": raw_angle_delta,
                "angle_island": island,
                "angle_offset_deg": row_offset,
                "aligned_delta_va_deg": aligned_angle_delta,
                "within_tolerance": abs(vm_delta) <= vm_tol
                and abs(aligned_angle_delta) <= angle_tol,
            }
        )
    offset_rows = [
        {"island": component, "offset_deg": value[0], "source": value[1]}
        for component, value in sorted(island_offsets.items())
    ]
    return rows, angle_offset, angle_source, offset_rows


def compare_generators(
    hysim_generators: list[dict[str, Any]],
    hysim_bus_by_id: dict[int, dict[str, Any]],
    dsp_generators: list[dict[str, Any]],
    bus_matches: dict[int, dict[str, Any]],
    vm_tol: float,
    p_tol: float,
    q_tol: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    queues: dict[int, deque[dict[str, Any]]] = defaultdict(deque)
    for generator in hysim_generators:
        queues[int(generator["bus"])].append(generator)
    used_indices: set[int] = set()
    rows: list[dict[str, Any]] = []
    unmatched: list[dict[str, Any]] = []

    for dsp_generator in dsp_generators:
        bus = bus_matches.get(int(dsp_generator["bus_seq"]))
        if bus is None or not queues[int(bus["id"])]:
            unmatched.append(
                {
                    "category": "generator",
                    "side": "dsp",
                    "identity": dsp_generator["label"],
                    "reason": "no_generator_at_matched_bus",
                }
            )
            continue
        generator = queues[int(bus["id"])].popleft()
        used_indices.add(int(generator["index"]))
        actual_vm = float(bus["vm_pu"])
        delta_p = float(generator["pg_mw"]) - float(dsp_generator["p_mw"])
        delta_q = float(generator["qg_mvar"]) - float(dsp_generator["q_mvar"])
        delta_vm = actual_vm - float(dsp_generator["actual_vm_pu"])
        rows.append(
            {
                "match_status": "matched",
                "dsp_bus_seq": dsp_generator["bus_seq"],
                "dsp_label": dsp_generator["label"],
                "dsp_generator_id": dsp_generator["generator_id"],
                "hysim_generator_index": generator["index"],
                "hysim_bus_id": bus["id"],
                "hysim_name": generator.get("name", bus.get("name", "")),
                "dsp_p_mw": dsp_generator["p_mw"],
                "hysim_p_mw": generator["pg_mw"],
                "delta_p_mw": delta_p,
                "dsp_q_mvar": dsp_generator["q_mvar"],
                "hysim_q_mvar": generator["qg_mvar"],
                "delta_q_mvar": delta_q,
                "dsp_actual_vm_pu": dsp_generator["actual_vm_pu"],
                "hysim_actual_vm_pu": actual_vm,
                "delta_vm_pu": delta_vm,
                "hysim_setpoint_vm_pu": generator.get("vg_pu", ""),
                "hysim_pmax_mw": generator.get("pmax_mw", ""),
                "dsp_qmax_mvar": dsp_generator["qmax_mvar"],
                "dsp_qmin_mvar": dsp_generator["qmin_mvar"],
                "within_tolerance": abs(delta_p) <= p_tol
                and abs(delta_q) <= q_tol
                and abs(delta_vm) <= vm_tol,
            }
        )

    for generator in hysim_generators:
        if int(generator["index"]) in used_indices:
            continue
        is_zero_capacity_record = (
            abs(float(generator.get("pg_mw", 0.0))) <= 1e-9
            and abs(float(generator.get("qg_mvar", 0.0))) <= 1e-9
            and float(generator.get("pmax_mw", 0.0)) > 0.0
        )
        bus = hysim_bus_by_id.get(int(generator["bus"]), {})
        reason = (
            "hysim_zero_output_capacity_record_not_listed_by_dsp"
            if is_zero_capacity_record
            else "not_present_in_dsp_generator_table"
        )
        unmatched.append(
            {
                "category": "generator",
                "side": "hysim",
                "identity": f"{generator.get('name', '')}@{bus.get('base_kv', '')}kV",
                "reason": reason,
            }
        )
        rows.append(
            {
                "match_status": reason,
                "dsp_bus_seq": "",
                "dsp_label": "",
                "dsp_generator_id": "",
                "hysim_generator_index": generator["index"],
                "hysim_bus_id": generator["bus"],
                "hysim_name": generator.get("name", ""),
                "dsp_p_mw": "",
                "hysim_p_mw": generator.get("pg_mw", ""),
                "delta_p_mw": "",
                "dsp_q_mvar": "",
                "hysim_q_mvar": generator.get("qg_mvar", ""),
                "delta_q_mvar": "",
                "dsp_actual_vm_pu": "",
                "hysim_actual_vm_pu": bus.get("vm_pu", ""),
                "delta_vm_pu": "",
                "hysim_setpoint_vm_pu": generator.get("vg_pu", ""),
                "hysim_pmax_mw": generator.get("pmax_mw", ""),
                "dsp_qmax_mvar": "",
                "dsp_qmin_mvar": "",
                "within_tolerance": "not_compared",
            }
        )
    return rows, unmatched


def json_branch_type(branch: dict[str, Any]) -> str:
    return "T" if bool(branch.get("is_transformer")) else "L"


def branch_orientation(
    branch: dict[str, Any], from_bus: int, to_bus: int, branch_type: str
) -> str | None:
    if json_branch_type(branch) != branch_type:
        return None
    json_from = int(branch["from"])
    json_to = int(branch["to"])
    if json_from == from_bus and json_to == to_bus:
        return "direct"
    if json_from == to_bus and json_to == from_bus:
        return "reversed"
    return None


def compare_branches(
    hysim_branches: list[dict[str, Any]],
    dsp_branches: list[dict[str, Any]],
    bus_matches: dict[int, dict[str, Any]],
    p_tol: float,
    q_tol: float,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    assignments: dict[int, tuple[int, str]] = {}
    used_json: set[int] = set()

    # The importer and DSP normally preserve the same branch order.  Use it
    # when identities agree so parallel circuits remain deterministic.
    for dsp_position, dsp_branch in enumerate(dsp_branches):
        from_bus = bus_matches.get(int(dsp_branch["from_seq"]))
        to_bus = bus_matches.get(int(dsp_branch["to_seq"]))
        if from_bus is None or to_bus is None or dsp_position >= len(hysim_branches):
            continue
        orientation = branch_orientation(
            hysim_branches[dsp_position],
            int(from_bus["id"]),
            int(to_bus["id"]),
            dsp_branch["branch_type"],
        )
        if orientation:
            assignments[dsp_position] = (dsp_position, orientation)
            used_json.add(dsp_position)

    direct_queues: dict[tuple[int, int, str], deque[int]] = defaultdict(deque)
    for position, branch in enumerate(hysim_branches):
        if position not in used_json:
            direct_queues[(int(branch["from"]), int(branch["to"]), json_branch_type(branch))].append(
                position
            )

    for dsp_position, dsp_branch in enumerate(dsp_branches):
        if dsp_position in assignments:
            continue
        from_bus = bus_matches.get(int(dsp_branch["from_seq"]))
        to_bus = bus_matches.get(int(dsp_branch["to_seq"]))
        if from_bus is None or to_bus is None:
            continue
        from_id = int(from_bus["id"])
        to_id = int(to_bus["id"])
        branch_type = dsp_branch["branch_type"]
        direct = direct_queues[(from_id, to_id, branch_type)]
        reverse = direct_queues[(to_id, from_id, branch_type)]
        if direct:
            position = direct.popleft()
            assignments[dsp_position] = (position, "direct")
            used_json.add(position)
        elif reverse:
            position = reverse.popleft()
            assignments[dsp_position] = (position, "reversed")
            used_json.add(position)

    rows: list[dict[str, Any]] = []
    unmatched: list[dict[str, Any]] = []
    for dsp_position, dsp_branch in enumerate(dsp_branches):
        assignment = assignments.get(dsp_position)
        if assignment is None:
            unmatched.append(
                {
                    "category": "branch",
                    "side": "dsp",
                    "identity": f"{dsp_branch['from_label']}->{dsp_branch['to_label']}",
                    "reason": "no_endpoint_and_type_match",
                }
            )
            continue
        json_position, orientation = assignment
        branch = hysim_branches[json_position]
        if orientation == "direct":
            hp_f, hq_f = float(branch["pf_mw"]), float(branch["qf_mvar"])
            hp_t, hq_t = float(branch["pt_mw"]), float(branch["qt_mvar"])
        else:
            hp_f, hq_f = float(branch["pt_mw"]), float(branch["qt_mvar"])
            hp_t, hq_t = float(branch["pf_mw"]), float(branch["qf_mvar"])
        deltas = {
            "delta_pf_mw": hp_f - float(dsp_branch["pf_mw"]),
            "delta_qf_mvar": hq_f - float(dsp_branch["qf_mvar"]),
            "delta_pt_mw": hp_t - float(dsp_branch["pt_mw"]),
            "delta_qt_mvar": hq_t - float(dsp_branch["qt_mvar"]),
        }
        rows.append(
            {
                "match_status": "matched",
                "dsp_position": dsp_position,
                "dsp_from_seq": dsp_branch["from_seq"],
                "dsp_from_label": dsp_branch["from_label"],
                "dsp_to_seq": dsp_branch["to_seq"],
                "dsp_to_label": dsp_branch["to_label"],
                "dsp_circuit": dsp_branch["circuit"],
                "branch_type": dsp_branch["branch_type"],
                "hysim_position": json_position,
                "hysim_index": branch.get("index", json_position),
                "hysim_name": branch.get("name", ""),
                "orientation": orientation,
                "dsp_pf_mw": dsp_branch["pf_mw"],
                "hysim_pf_mw": hp_f,
                "delta_pf_mw": deltas["delta_pf_mw"],
                "dsp_qf_mvar": dsp_branch["qf_mvar"],
                "hysim_qf_mvar": hq_f,
                "delta_qf_mvar": deltas["delta_qf_mvar"],
                "dsp_pt_mw": dsp_branch["pt_mw"],
                "hysim_pt_mw": hp_t,
                "delta_pt_mw": deltas["delta_pt_mw"],
                "dsp_qt_mvar": dsp_branch["qt_mvar"],
                "hysim_qt_mvar": hq_t,
                "delta_qt_mvar": deltas["delta_qt_mvar"],
                "dsp_p_loss_mw": dsp_branch["p_loss_mw"],
                "hysim_p_loss_mw": hp_f + hp_t,
                "dsp_q_loss_mvar": dsp_branch["q_loss_mvar"],
                "hysim_q_loss_mvar": hq_f + hq_t,
                "within_tolerance": max(
                    abs(deltas["delta_pf_mw"]), abs(deltas["delta_pt_mw"])
                )
                <= p_tol
                and max(abs(deltas["delta_qf_mvar"]), abs(deltas["delta_qt_mvar"]))
                <= q_tol,
            }
        )

    for position, branch in enumerate(hysim_branches):
        if position not in used_json:
            unmatched.append(
                {
                    "category": "branch",
                    "side": "hysim",
                    "identity": branch.get("name", str(position)),
                    "reason": "not_present_in_dsp_branch_table",
                }
            )
    return rows, unmatched


def numeric_values(rows: Iterable[dict[str, Any]], field: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        value = row.get(field)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def metric_stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean_abs": None, "rmse": None, "p95_abs": None, "max_abs": None}
    absolute = sorted(abs(value) for value in values)
    p95_index = min(len(absolute) - 1, math.ceil(0.95 * len(absolute)) - 1)
    return {
        "count": len(values),
        "mean_abs": sum(absolute) / len(absolute),
        "rmse": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95_abs": absolute[p95_index],
        "max_abs": absolute[-1],
    }


def category_summary(rows: list[dict[str, Any]], fields: list[str]) -> dict[str, Any]:
    matched = [row for row in rows if row.get("match_status") == "matched"]
    within = sum(row.get("within_tolerance") is True for row in matched)
    return {
        "matched": len(matched),
        "within_tolerance": within,
        "outside_tolerance": len(matched) - within,
        "metrics": {field: metric_stats(numeric_values(matched, field)) for field in fields},
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def format_value(value: Any) -> str:
    if isinstance(value, float):
        return f"{value:.8g}"
    return str(value)


def html_table(rows: list[dict[str, Any]], columns: list[str], limit: int = 50) -> str:
    head = "".join(f"<th>{html.escape(column)}</th>" for column in columns)
    body = []
    for row in rows[:limit]:
        cells = "".join(
            f"<td>{html.escape(format_value(row.get(column, '')))}</td>" for column in columns
        )
        body.append(f"<tr>{cells}</tr>")
    return f"<table><thead><tr>{head}</tr></thead><tbody>{''.join(body)}</tbody></table>"


def write_html_report(
    path: Path,
    summary: dict[str, Any],
    bus_rows: list[dict[str, Any]],
    generator_rows: list[dict[str, Any]],
    branch_rows: list[dict[str, Any]],
    unmatched: list[dict[str, Any]],
) -> None:
    top_buses = sorted(
        bus_rows,
        key=lambda row: max(
            abs(float(row.get("delta_vm_pu", 0.0))),
            abs(float(row.get("aligned_delta_va_deg", 0.0))),
        ),
        reverse=True,
    )
    matched_generators = [row for row in generator_rows if row.get("match_status") == "matched"]
    top_generators = sorted(
        matched_generators,
        key=lambda row: max(abs(float(row["delta_p_mw"])), abs(float(row["delta_q_mvar"]))),
        reverse=True,
    )
    top_branches = sorted(
        branch_rows,
        key=lambda row: max(
            abs(float(row["delta_pf_mw"])),
            abs(float(row["delta_qf_mvar"])),
            abs(float(row["delta_pt_mw"])),
            abs(float(row["delta_qt_mvar"])),
        ),
        reverse=True,
    )
    counts = summary["counts"]
    overview = [
        {"item": "HySim convergence", "value": summary["hysim"]["converged"]},
        {"item": "DSP convergence", "value": summary["dsp"]["converged"]},
        {"item": "Matched buses", "value": counts["matched_buses"]},
        {"item": "Matched generators", "value": counts["matched_generators"]},
        {"item": "Matched branches", "value": counts["matched_branches"]},
        {"item": "Unmatched records", "value": counts["unmatched_records"]},
        {"item": "Angle offset (deg)", "value": summary["angle_alignment"]["offset_deg"]},
    ]
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>HySim vs DSP PF comparison</title>
<style>
body{{font-family:Segoe UI,Arial,sans-serif;margin:24px;color:#1f2933;background:#fff}}
h1{{font-size:24px;margin:0 0 8px}} h2{{font-size:18px;margin-top:28px}}
p{{color:#52606d}} table{{border-collapse:collapse;width:100%;font-size:12px}}
th,td{{border:1px solid #d9e2ec;padding:6px 8px;text-align:right;white-space:nowrap}}
th{{background:#f0f4f8;position:sticky;top:0}} th:first-child,td:first-child{{text-align:left}}
.links a{{margin-right:16px}} .scroll{{overflow:auto;max-height:520px}}
</style></head><body>
<h1>HySim vs DSP power-flow comparison</h1>
<p>Angles are aligned by {html.escape(summary['angle_alignment']['source'])}. DSP bus values are rounded to four decimals and branch values to two decimals in the PF report.</p>
<div class="links"><a href="bus_comparison.csv">Bus CSV</a><a href="generator_comparison.csv">Generator CSV</a><a href="branch_comparison.csv">Branch CSV</a><a href="unmatched.csv">Unmatched CSV</a><a href="summary.json">Summary JSON</a></div>
<h2>Overview</h2>{html_table(overview, ['item', 'value'])}
<h2>Largest bus differences</h2><div class="scroll">{html_table(top_buses, ['dsp_label','bus_type','dsp_vm_pu','hysim_vm_pu','delta_vm_pu','dsp_va_deg','hysim_va_deg','aligned_delta_va_deg','within_tolerance'])}</div>
<h2>Largest generator differences</h2><div class="scroll">{html_table(top_generators, ['dsp_label','dsp_p_mw','hysim_p_mw','delta_p_mw','dsp_q_mvar','hysim_q_mvar','delta_q_mvar','within_tolerance'])}</div>
<h2>Largest branch differences</h2><div class="scroll">{html_table(top_branches, ['dsp_from_label','dsp_to_label','branch_type','dsp_pf_mw','hysim_pf_mw','delta_pf_mw','dsp_qf_mvar','hysim_qf_mvar','delta_qf_mvar','within_tolerance'])}</div>
<h2>Unmatched records</h2><div class="scroll">{html_table(unmatched, ['category','side','identity','reason'], 200)}</div>
</body></html>"""
    path.write_text(document, encoding="utf-8")


def run(args: argparse.Namespace) -> tuple[Path, dict[str, Any]]:
    hysim = load_hysim_json(args.hysim_json)
    dsp = parse_dsp_pf(args.dsp_pf, args.pf_encoding)
    hysim_buses = hysim["geo_buses"]
    hysim_generators = hysim["geo_gen"]
    hysim_branches = hysim["geo_ac_branches"]
    hysim_bus_by_id = {int(bus["id"]): bus for bus in hysim_buses}

    bus_matches, bus_unmatched = match_buses(hysim_buses, dsp["buses"])
    component_by_bus = ac_island_by_bus(hysim_buses, hysim_branches)
    bus_rows, angle_offset, angle_source, island_offsets = compare_buses(
        hysim_buses,
        dsp["buses"],
        bus_matches,
        args.angle_align,
        args.vm_tol,
        args.angle_tol_deg,
        component_by_bus,
    )
    generator_rows, generator_unmatched = compare_generators(
        hysim_generators,
        hysim_bus_by_id,
        dsp["generators"],
        bus_matches,
        args.vm_tol,
        args.p_tol_mw,
        args.q_tol_mvar,
    )
    branch_rows, branch_unmatched = compare_branches(
        hysim_branches,
        dsp["branches"],
        bus_matches,
        args.p_tol_mw,
        args.q_tol_mvar,
    )
    unmatched = bus_unmatched + generator_unmatched + branch_unmatched

    bus_summary = category_summary(bus_rows, ["delta_vm_pu", "aligned_delta_va_deg"])
    generator_summary = category_summary(
        generator_rows, ["delta_p_mw", "delta_q_mvar", "delta_vm_pu"]
    )
    branch_summary = category_summary(
        branch_rows,
        ["delta_pf_mw", "delta_qf_mvar", "delta_pt_mw", "delta_qt_mvar"],
    )
    zero_capacity_only = sum(
        item["reason"] == "hysim_zero_output_capacity_record_not_listed_by_dsp"
        for item in generator_unmatched
    )
    summary = {
        "inputs": {
            "hysim_json": str(args.hysim_json.resolve()),
            "dsp_pf": str(args.dsp_pf.resolve()),
            "dsp_encoding": dsp["encoding"],
        },
        "hysim": {
            "converged": bool(hysim.get("converged")),
            "iterations": hysim.get("iterations"),
            "residual": hysim.get("residual"),
            "method": hysim.get("method"),
        },
        "dsp": {"converged": dsp["converged"], "iterations": dsp["iterations"]},
        "angle_alignment": {
            "requested": args.angle_align,
            "source": angle_source,
            "offset_deg": angle_offset,
            "island_offsets": island_offsets,
            "definition": "HySim angle minus DSP angle; subtracted before comparison",
        },
        "tolerances": {
            "vm_pu": args.vm_tol,
            "angle_deg": args.angle_tol_deg,
            "p_mw": args.p_tol_mw,
            "q_mvar": args.q_tol_mvar,
        },
        "counts": {
            "hysim_buses": len(hysim_buses),
            "dsp_buses": len(dsp["buses"]),
            "matched_buses": bus_summary["matched"],
            "hysim_generators": len(hysim_generators),
            "dsp_generators": len(dsp["generators"]),
            "matched_generators": generator_summary["matched"],
            "hysim_zero_output_capacity_records_not_listed_by_dsp": zero_capacity_only,
            "hysim_branches": len(hysim_branches),
            "dsp_branches": len(dsp["branches"]),
            "matched_branches": branch_summary["matched"],
            "unmatched_records": len(unmatched),
        },
        "bus_comparison": bus_summary,
        "generator_comparison": generator_summary,
        "branch_comparison": branch_summary,
        "notes": [
            "DSP PF bus values are rounded to 4 decimals and branch P/Q values to 2 decimals.",
            "HySim-only generators with Pg=Qg=0 and Pmax>0 are capacity records, not missing DSP output rows.",
        ],
    }

    output_dir = args.output_dir or Path(
        f"comparison_{args.hysim_json.stem}_vs_{args.dsp_pf.stem}"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "bus_comparison.csv", bus_rows)
    write_csv(output_dir / "generator_comparison.csv", generator_rows)
    write_csv(output_dir / "branch_comparison.csv", branch_rows)
    write_csv(output_dir / "unmatched.csv", unmatched)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_html_report(
        output_dir / "comparison_report.html",
        summary,
        bus_rows,
        generator_rows,
        branch_rows,
        unmatched,
    )
    return output_dir, summary


def main() -> int:
    args = parse_args()
    try:
        output_dir, summary = run(args)
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    counts = summary["counts"]
    print(f"Report directory: {output_dir.resolve()}")
    print(
        f"Buses: {counts['matched_buses']}/{counts['dsp_buses']} matched; "
        f"Generators: {counts['matched_generators']}/{counts['dsp_generators']} matched; "
        f"Branches: {counts['matched_branches']}/{counts['dsp_branches']} matched"
    )
    print(
        "Outside tolerance: "
        f"buses={summary['bus_comparison']['outside_tolerance']}, "
        f"generators={summary['generator_comparison']['outside_tolerance']}, "
        f"branches={summary['branch_comparison']['outside_tolerance']}"
    )
    if args.fail_on_tolerance and any(
        summary[name]["outside_tolerance"] > 0
        for name in ("bus_comparison", "generator_comparison", "branch_comparison")
    ):
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
