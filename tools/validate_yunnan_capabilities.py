#!/usr/bin/env python3
"""Run a bounded production-HTTP capability audit on the Yunnan BPA JSON.

The report separates numerical success, honest model limitation, failure, and
missing-input applicability. It never invents DC, three-phase, reliability,
market, dynamics, GIS, or time-series data to make an endpoint appear usable.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable


ROOT = Path(__file__).resolve().parent.parent


class Client:
    def __init__(self, base_url: str):
        self.base_url = base_url.rstrip("/")

    def request(self, method: str, path: str, payload: Any = None,
                timeout: float = 60.0) -> tuple[int, Any]:
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            self.base_url + path,
            data=body,
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as response:
                raw = response.read()
                return response.status, json.loads(raw) if raw else {}
        except urllib.error.HTTPError as exc:
            raw = exc.read()
            try:
                data = json.loads(raw) if raw else {"error": str(exc)}
            except json.JSONDecodeError:
                data = {"error": raw.decode("utf-8", errors="replace")}
            return exc.code, data

    def get(self, path: str, timeout: float = 30.0) -> tuple[int, Any]:
        return self.request("GET", path, timeout=timeout)

    def post(self, path: str, payload: Any = None,
             timeout: float = 60.0) -> tuple[int, Any]:
        return self.request("POST", path, payload or {}, timeout)

    def cancel_and_wait(self, timeout: float = 30.0) -> bool:
        try:
            self.post("/api/session/cancel", {}, timeout=5.0)
        except Exception:
            pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                _, status = self.get("/api/session/status", timeout=2.0)
                if not status.get("busy", False):
                    return True
            except Exception:
                pass
            time.sleep(0.1)
        return False


@dataclass(frozen=True)
class Probe:
    probe_id: str
    domain: str
    path: str
    payload: dict[str, Any]
    timeout_s: float = 60.0
    applicability: Callable[[dict[str, Any]], tuple[bool, str]] | None = None
    kind: str = "generic"
    limitation: str = ""
    prepare: tuple[tuple[str, dict[str, Any], float], ...] = ()


def rows_at(model: dict[str, Any], *path: str) -> list[Any]:
    value: Any = model
    for key in path:
        if not isinstance(value, dict):
            return []
        value = value.get(key, [])
    return value if isinstance(value, list) else []


def build_inventory(model: dict[str, Any]) -> dict[str, Any]:
    branches = rows_at(model, "ac", "branches")
    generators = rows_at(model, "ac", "generators")
    loads = rows_at(model, "ac", "loads")
    buses = rows_at(model, "ac", "buses")
    return {
        "name": model.get("name", ""),
        "base_mva": model.get("base_mva"),
        "ac_buses": len(buses),
        "ac_branches": len(branches),
        "generators": len(generators),
        "loads": len(loads),
        "shunts": len(rows_at(model, "ac", "shunts")),
        "transformers_2w": len(rows_at(model, "ac", "transformers_2w")),
        "transformer_like_branches": sum(
            str(row.get("name", "")).startswith("T_")
            or float(row.get("sn_mva", 0.0) or 0.0) > 0.0
            or float(row.get("vn_hv_kv", 0.0) or 0.0) > 0.0
            for row in branches if isinstance(row, dict)
        ),
        "switches": len(rows_at(model, "ac", "switches")),
        "circuit_breakers": len(rows_at(model, "ac", "circuit_breakers")),
        "storage": len(rows_at(model, "ac", "storage")),
        "renewables": len(rows_at(model, "ac", "renewable_gens")),
        "external_grids": len(rows_at(model, "ac", "external_grids")),
        "dc_buses": len(rows_at(model, "dc", "buses")),
        "vsc_converters": len(rows_at(model, "vsc_converters")),
        "lcc_converters": len(rows_at(model, "lcc_converters")),
        "three_phase_buses": len(rows_at(model, "three_phase_ac", "buses")),
        "branches_with_failure_rate": sum(
            float(row.get("failure_rate", 0.0) or 0.0) > 0.0
            for row in branches if isinstance(row, dict)
        ),
        "loads_with_customers": sum(
            float(row.get("num_customers", 0.0) or 0.0) > 0.0
            for row in loads if isinstance(row, dict)
        ),
        "generators_with_cost": sum(
            any(abs(float(row.get(key, 0.0) or 0.0)) > 0.0
                for key in ("cost_c0", "cost_c1", "cost_c2"))
            for row in generators if isinstance(row, dict)
        ),
        "generators_with_dynamic_model": sum(
            bool(row.get("dynamic_model"))
            for row in generators if isinstance(row, dict)
        ),
        "buses_with_gis": sum(
            any(abs(float(row.get(key, 0.0) or 0.0)) > 0.0
                for key in ("latitude", "longitude"))
            for row in buses if isinstance(row, dict)
        ),
        "switchable_shunts": sum(
            bool(row.get("switchable", False))
            or int(row.get("num_steps", 0) or 0) > 0
            for row in rows_at(model, "ac", "shunts")
            if isinstance(row, dict)
        ),
        "load_profiles": sum(
            row.get("profile_id") is not None and int(row.get("profile_id")) >= 0
            for row in loads if isinstance(row, dict)
        ),
        "generator_profiles": sum(
            row.get("profile_id") is not None and int(row.get("profile_id")) >= 0
            for row in generators if isinstance(row, dict)
        ),
        "first_ac_bus": buses[0].get("index") if buses else None,
    }


def require_any(*keys: str, reason: str) -> Callable[[dict[str, Any]], tuple[bool, str]]:
    def check(inventory: dict[str, Any]) -> tuple[bool, str]:
        return (any(inventory.get(key, 0) for key in keys), reason)
    return check


def require_all(*keys: str, reason: str) -> Callable[[dict[str, Any]], tuple[bool, str]]:
    def check(inventory: dict[str, Any]) -> tuple[bool, str]:
        return (all(inventory.get(key, 0) for key in keys), reason)
    return check


def pf_probe(method: str, timeout_s: float = 60.0,
             options: dict[str, Any] | None = None,
             probe_id: str | None = None) -> Probe:
    return Probe(
        probe_id or f"pf_{method}",
        "Power flow",
        "/api/session/pf",
        {"method": method, "response_detail": "compact",
         "options": options or {"max_iter": 200}},
        timeout_s,
        kind="pf",
    )


def build_probes(inventory: dict[str, Any]) -> list[Probe]:
    first_bus = inventory["first_ac_bus"]
    no_dc = require_any("dc_buses", reason="requires authored DC buses")
    no_tp = require_any("three_phase_buses", reason="requires phase-domain buses")
    no_dynamics = require_any(
        "generators_with_dynamic_model", reason="requires authored dynamic models")
    no_market = require_any(
        "generators_with_cost", reason="requires authored generator cost curves")
    no_reliability = require_all(
        "branches_with_failure_rate", "loads_with_customers",
        reason="requires authored failure rates and customer counts")
    no_reconfiguration = require_any(
        "switches", "circuit_breakers", reason="requires controllable topology devices")
    no_gis = require_any("buses_with_gis", reason="requires authored GIS coordinates")
    no_ies = require_any(
        "storage", "renewables", reason="requires integrated-energy assets")
    no_profiles = require_any(
        "load_profiles", "generator_profiles",
        reason="requires authored operating profiles")

    probes = [
        pf_probe("ac_newton", options={"max_iter": 200,
                 "enable_pv_pq_conversion": False}, probe_id="pf_ac_screen"),
        pf_probe("ac_newton", options={"max_iter": 200,
                 "enable_pv_pq_conversion": True}, probe_id="pf_ac_q_limits"),
        pf_probe("pure_ac"),
        pf_probe("fdpf"),
        pf_probe("dc"),
        pf_probe("hybrid_linearized"),
        pf_probe("adaptive"),
        pf_probe("islanded"),
        pf_probe("distributed_slack"),
        pf_probe("helm", timeout_s=90.0),
        pf_probe("homotopy", timeout_s=90.0),
        pf_probe("newton_krylov", timeout_s=90.0),
        Probe("topology", "Graph and topology", "/api/session/topology", {},
              kind="generic"),
        Probe("network_reduction", "Graph and topology",
              "/api/session/network_reduction", {}, timeout_s=90.0),
        Probe("sppt_guard", "Projection certificate", "/api/session/sppt_guard", {},
              kind="generic"),
        Probe("short_circuit_all", "Short circuit", "/api/session/sc",
              {"options": {"compute_all_buses": True}}, timeout_s=90.0,
              kind="short_circuit"),
        Probe("short_circuit_detailed", "Short circuit",
              "/api/session/sc_detailed", {"fault_bus_ids": [first_bus]},
              timeout_s=90.0, kind="short_circuit"),
        Probe("harmonics_penetration", "Harmonics", "/api/session/harmonics",
              {"options": {"ac_orders": [5, 7, 11, 13],
                           "dc_orders": [2, 6],
                           "auto_nic_from_vscs": True,
                           "include_load_impedance": True}},
              timeout_s=90.0, kind="generic",
              limitation="No authored harmonic sources; zero penetration is an input boundary."),
        Probe("opf_dc", "Optimal power flow", "/api/session/opf",
              {"solver": "dc", "network_model": "balanced_aggregate",
               "constraints": {"branch_limits": False},
               "options": {"max_iterations": 100, "load_shedding": True,
                           "compute_lmp": True}},
              timeout_s=120.0, kind="opf",
              limitation="No authored cost curves; objective economics are not evidential."),
        Probe("opf_parity_phase_one_off", "Optimal power flow",
              "/api/session/opf",
              {"solver": "parity", "network_model": "balanced_aggregate",
               "constraints": {"branch_limits": False},
               "options": {"max_iterations": 100, "ac_pf_warm_start": True,
                           "enable_phase_one": False}},
              timeout_s=180.0, kind="opf",
              limitation="No authored cost curves; this audits feasibility and solver work only."),
        Probe("opf_parity_phase_one_on", "Optimal power flow",
              "/api/session/opf",
              {"solver": "parity", "network_model": "balanced_aggregate",
               "constraints": {"branch_limits": False},
               "options": {"max_iterations": 100, "ac_pf_warm_start": True,
                           "enable_phase_one": True,
                           "phase_one_time_limit_ms": 2000.0,
                           "phase_one_max_iterations": 5,
                           "phase_one_max_factorizations": 7,
                           "phase_one_max_backtracks": 8}},
              timeout_s=180.0, kind="opf",
              limitation="No authored cost curves; this audits Phase-I handoff and solver work only."),
        Probe("time_series_pf_4step", "Time series",
              "/api/session/run_ts_pf",
              {"num_steps": 4, "step_duration_hr": 1.0, "skip_uc": True,
               "run_opf": False}, timeout_s=120.0, kind="time_series",
              limitation="No authored profiles; repeated base point is a plumbing test only."),
        Probe("carbon_flow", "Carbon analysis", "/api/session/run_carbon", {},
              timeout_s=60.0,
              limitation="No authored carbon factors; default/fallback factors are not case evidence.",
              prepare=(("/api/session/pf",
                        {"method": "ac_newton", "response_detail": "compact",
                         "options": {"enable_pv_pq_conversion": True}}, 60.0),)),
        Probe("hosting_capacity", "Hosting capacity",
              "/api/session/run_hosting_capacity", {}, timeout_s=60.0,
              applicability=require_any(
                  "transformers_2w", "transformer_like_branches",
                  reason="requires transformer capacity metadata")),
        Probe("reactive_power_optimization", "Reactive power optimization",
              "/api/session/run_rpo", {}, timeout_s=120.0,
              applicability=require_any(
                  "switchable_shunts", "switches",
                  reason="requires controllable taps or switched shunts")),
        Probe("network_reconfiguration", "Network reconfiguration",
              "/api/session/run_reconfig",
              {"options": {"enable_pf": True, "loss_aware": True,
                           "solver": "auto"}}, timeout_s=120.0,
              applicability=no_reconfiguration),
        Probe("reliability", "Reliability", "/api/session/run_reliability",
              {"method": "fmea"}, timeout_s=180.0,
              applicability=no_reliability),
        Probe("distribution_resilience", "Resilience",
              "/api/session/run_distribution_resilience",
              {"default_fault_count": 1, "horizon_hours": 4,
               "mip_solver": "HiGHS"}, timeout_s=180.0,
              applicability=no_reconfiguration),
        Probe("scenario_generation", "Scenario generation",
              "/api/session/generate_scenarios",
              {"regular_clusters": 2, "reliability_clusters": 2,
               "resilience_clusters": 2}, timeout_s=90.0,
              limitation="No authored stochastic/time-series inputs; generated defaults are not case evidence."),
        Probe("counterfactual_planning", "Planning analysis",
              "/api/session/run_counterfactual_planning", {}, timeout_s=180.0,
              limitation="No authored planning portfolio; backend default measures are a workflow probe."),
        Probe("multidimensional_weak_links", "Planning analysis",
              "/api/session/run_multidimensional_weak_links",
              {"entities": [], "options": {"mode": "planning", "top_k": 10,
                                             "minimum_dimensions": 1}},
              timeout_s=60.0, applicability=no_reliability),
        Probe("typhoon_faults", "Typhoon resilience",
              "/api/session/generate_typhoon_faults", {}, timeout_s=90.0,
              applicability=no_gis),
        Probe("transient", "Dynamics", "/api/session/run_transient",
              {"solver_type": "heun", "t_end_s": 0.05, "dt_s": 0.01,
               "run_power_flow_initialization": True}, timeout_s=120.0,
              applicability=no_dynamics),
        Probe("small_signal", "Dynamics", "/api/session/small_signal", {},
              timeout_s=120.0, applicability=no_dynamics),
        Probe("market", "Market", "/api/session/run_market_clearing",
              {"num_steps": 4, "offer_segments": 2,
               "network_constraints": True, "run_ac_validation": False},
              timeout_s=180.0, applicability=no_market),
        Probe("real_time_market", "Market",
              "/api/session/run_real_time_market", {}, timeout_s=120.0,
              applicability=no_market),
        Probe("repeated_market_game", "Market",
              "/api/session/run_repeated_market_game", {}, timeout_s=120.0,
              applicability=no_market),
        Probe("annual_production", "Time series",
              "/api/session/run_annual_sim", {}, timeout_s=180.0,
              applicability=no_profiles),
        Probe("lifecycle_simulation", "Lifecycle",
              "/api/session/run_lifecycle_sim", {}, timeout_s=180.0,
              applicability=no_ies),
        Probe("lifecycle_comparison", "Lifecycle",
              "/api/session/run_lifecycle_compare", {}, timeout_s=180.0,
              applicability=no_ies),
        Probe("integrated_energy", "Integrated energy",
              "/api/session/run_campus_ies", {}, timeout_s=120.0,
              applicability=no_ies),
        Probe("ev_traffic", "EV and traffic", "/api/session/run_ev_traffic", {},
              timeout_s=120.0,
              applicability=require_any(
                  "renewables", reason="requires an authored traffic/charging system")),
        Probe("dc_short_circuit", "Short circuit", "/api/session/dc_sc", {},
              applicability=no_dc),
        Probe("three_phase_pf", "Power flow", "/api/session/pf",
              {"method": "three_phase", "response_detail": "compact"},
              applicability=no_tp, kind="pf"),
        Probe("three_phase_hybrid_pf", "Power flow", "/api/session/pf",
              {"method": "three_phase_hybrid", "response_detail": "compact"},
              applicability=no_tp, kind="pf"),
        Probe("three_phase_harmonics", "Harmonics",
              "/api/session/harmonics_3ph", {}, applicability=no_tp),
    ]
    return probes


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def summarize(probe: Probe, status_code: int, data: Any,
              elapsed_s: float) -> tuple[str, dict[str, Any]]:
    if not isinstance(data, dict):
        return "FAIL", {"http_status": status_code, "error": "non-object response"}
    error = data.get("error")
    summary: dict[str, Any] = {
        "http_status": status_code,
        "wall_ms": round(elapsed_s * 1000.0, 3),
    }
    if error or status_code >= 400:
        summary["error"] = str(error or data.get("status") or "HTTP failure")[:500]
        return "REJECTED" if status_code in (400, 409, 422) else "FAIL", summary

    if probe.kind == "pf":
        reactive = data.get("reactive_limits") or {}
        timing = data.get("timing") or {}
        summary.update({
            "converged": data.get("converged"),
            "method_actual": data.get("method_actual", data.get("method")),
            "iterations": data.get("iterations"),
            "residual": data.get("residual"),
            "solver_ms": timing.get("solve_ms"),
            "presentation_ms": timing.get("presentation_ms"),
            "total_before_serialization_ms": timing.get("total_before_serialize_ms"),
            "q_requested": reactive.get("enforcement_requested"),
            "q_certified": reactive.get("certified"),
            "q_max_violation_pu": reactive.get("max_violation_pu"),
            "pv_to_pq": reactive.get("pv_to_pq_switches"),
            "pq_to_pv": reactive.get("pq_to_pv_switches"),
            "active_set_cycle": reactive.get("active_set_cycle_detected"),
        })
        outcome = "PASS" if data.get("converged") is True else "FAIL"
        if reactive.get("enforcement_requested") and not reactive.get("certified"):
            outcome = "FAIL"
        return outcome, summary

    if probe.kind == "opf":
        profiling = data.get("ipm_profiling") or data.get("profiling") or {}
        timing = data.get("timing") or {}
        summary.update({
            "converged": data.get("converged"),
            "status": data.get("status"),
            "solver_backend": data.get("solver_backend"),
            "iterations": data.get("iterations"),
            "objective": data.get("objective"),
            "primal_residual": data.get("primal_residual"),
            "dual_residual": data.get("dual_residual"),
            "core_solve_ms": timing.get("core_solve_ms"),
            "total_before_serialization_ms": timing.get("total_before_serialize_ms"),
            "phase_one_runtime_ms": profiling.get("phase_one_runtime_ms"),
            "phase_one_termination": profiling.get("phase_one_termination"),
            "phase_one_constraint_violation":
                profiling.get("phase_one_constraint_violation"),
            "phase_one_in_handoff_corridor":
                profiling.get("phase_one_in_handoff_corridor"),
            "phase_two_start_accepted": profiling.get("phase_two_start_accepted"),
            "factorization_calls": profiling.get("factorization_calls"),
        })
        outcome = "PASS" if data.get("converged") is True else "FAIL"
        return ("LIMITED" if outcome == "PASS" and probe.limitation else outcome,
                summary)

    if probe.kind == "short_circuit":
        rows = data.get("bus_results") or data.get("results") or []
        summary["result_count"] = len(rows) if isinstance(rows, list) else None
        summary["model_scope"] = data.get("model_scope")
    elif probe.kind == "time_series":
        summary.update({"num_steps": data.get("num_steps"),
                        "num_converged": data.get("num_converged"),
                        "status": data.get("status"),
                        "vm_min": data.get("vm_min"),
                        "vm_max": data.get("vm_max"),
                        "profile_audit": data.get("profile_audit")})
        if data.get("num_steps") and data.get("num_converged") != data.get("num_steps"):
            return "FAIL", summary
    else:
        summary["keys"] = list(data)[:20]
        for key in ("ok", "success", "converged", "status", "model_scope"):
            if key in data:
                summary[key] = data[key]
        if probe.probe_id == "topology":
            summary.update({key: data.get(key) for key in (
                "n_buses", "n_branches", "n_ac_islands", "n_dc_islands",
                "is_connected", "is_radial", "cycle_count")})
        elif probe.probe_id == "network_reduction":
            summary.update({key: data.get(key) for key in (
                "before", "after", "n_buses_eliminated",
                "n_branches_eliminated", "reduction_pct_buses")})
        elif probe.probe_id == "sppt_guard":
            summary.update({key: data.get(key) for key in (
                "accepted", "validation_ok", "well_posed", "reason")})
        elif probe.probe_id == "harmonics_penetration":
            summary.update({key: data.get(key) for key in (
                "base_pf_converged", "max_ac_thd_pct", "max_ac_thd_bus",
                "max_dc_thd_pct", "message")})
        elif probe.probe_id == "carbon_flow":
            summary.update({key: data.get(key) for key in (
                "matrix_solved", "matrix_rank", "matrix_relative_residual",
                "max_node_power_balance_error_mw", "execution_time_sec")})
        elif probe.probe_id == "hosting_capacity":
            summary["transformer_count"] = len(data.get("transformers") or [])
            summary["standard"] = data.get("standard")
        elif probe.probe_id == "scenario_generation":
            summary["scenario_summary"] = data.get("summary")
    if probe.limitation:
        summary["limitation"] = probe.limitation
        return "LIMITED", summary
    return "PASS", summary


def markdown_report(case_path: Path, inventory: dict[str, Any],
                    results: list[dict[str, Any]]) -> str:
    counts: dict[str, int] = {}
    for row in results:
        counts[row["outcome"]] = counts.get(row["outcome"], 0) + 1
    lines = [
        "# 云南案例功能模块实证",
        "",
        f"> 输入：`{case_path}`；所有执行项调用生产 `run_gui_server` HTTP 路由。",
        "> `LIMITED` 表示接口运行但案例缺少该模块的专属物理数据；",
        "> `NOT_APPLICABLE` 表示没有伪造缺失资产来强行求解。",
        "",
        "## 资产与数据边界",
        "",
        "```json",
        json.dumps(inventory, ensure_ascii=False, indent=2),
        "```",
        "",
        "## 汇总",
        "",
        " / ".join(f"{key}={value}" for key, value in sorted(counts.items())),
        "",
        "## 模块矩阵",
        "",
        "| 模块 | 能力域 | 结果 | 墙钟 | 证据/边界 |",
        "|---|---|---:|---:|---|",
    ]
    for row in results:
        summary = row.get("summary", {})
        wall = summary.get("wall_ms")
        wall_text = f"{wall:.1f} ms" if finite(wall) else "-"
        if row["outcome"] == "NOT_APPLICABLE":
            detail = row.get("reason", "")
        elif summary.get("error"):
            detail = summary["error"]
            if "cancel_recovered" in summary:
                detail += f"; cancel_recovered={summary['cancel_recovered']}"
        else:
            fields = []
            for key in ("converged", "method_actual", "iterations", "residual",
                        "solver_ms", "pv_to_pq", "pq_to_pv", "q_certified",
                        "q_max_violation_pu", "core_solve_ms",
                        "factorization_calls", "phase_one_runtime_ms",
                        "phase_one_termination", "phase_one_constraint_violation",
                        "phase_one_in_handoff_corridor",
                        "phase_two_start_accepted", "num_converged", "num_steps",
                        "vm_min", "vm_max", "result_count", "status",
                        "n_buses", "n_branches", "n_ac_islands", "is_connected",
                        "n_buses_eliminated", "n_branches_eliminated",
                        "reduction_pct_buses", "accepted", "validation_ok",
                        "base_pf_converged", "max_ac_thd_pct", "matrix_solved",
                        "matrix_relative_residual", "transformer_count", "standard",
                        "limitation"):
                if summary.get(key) is not None:
                    fields.append(f"{key}={summary[key]}")
            detail = "; ".join(fields) or ", ".join(summary.get("keys", [])[:8])
        detail = str(detail).replace("|", "\\|").replace("\n", " ")
        lines.append(
            f"| `{row['probe_id']}` | {row['domain']} | {row['outcome']} | "
            f"{wall_text} | {detail} |"
        )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-url", default="http://127.0.0.1:8088")
    parser.add_argument("--case", default=str(ROOT / "data" / "云南案例.json"))
    parser.add_argument("--out-json",
                        default=str(ROOT / "output" / "yunnan_capability_audit.json"))
    parser.add_argument("--out-md",
                        default=str(ROOT / "output" / "yunnan_capability_audit.md"))
    parser.add_argument("--only", default="",
                        help="Comma-separated probe ids; empty runs the full matrix")
    parser.add_argument("--merge-inputs", default="",
                        help="Comma-separated audit JSON files to merge without HTTP calls")
    args = parser.parse_args()

    case_path = Path(args.case).resolve()
    model = json.loads(case_path.read_text(encoding="utf-8"))
    inventory = build_inventory(model)
    selected = {item.strip() for item in args.only.split(",") if item.strip()}
    probes = [p for p in build_probes(inventory)
              if not selected or p.probe_id in selected]

    if args.merge_inputs:
        merged: dict[str, dict[str, Any]] = {}
        for raw_path in args.merge_inputs.split(","):
            source = Path(raw_path.strip())
            if not source:
                continue
            source_report = json.loads(source.read_text(encoding="utf-8"))
            for row in source_report.get("results", []):
                merged[row["probe_id"]] = row
        ordered_ids = [probe.probe_id for probe in build_probes(inventory)]
        results = [merged[probe_id] for probe_id in ordered_ids
                   if probe_id in merged]
        results.extend(row for probe_id, row in merged.items()
                       if probe_id not in ordered_ids)
        report = {
            "schema": "yunnan_capability_audit_v1",
            "case": str(case_path),
            "base_url": args.base_url,
            "inventory": inventory,
            "results": results,
        }
        out_json = Path(args.out_json)
        out_md = Path(args.out_md)
        out_json.parent.mkdir(parents=True, exist_ok=True)
        out_md.parent.mkdir(parents=True, exist_ok=True)
        out_json.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        out_md.write_text(markdown_report(case_path, inventory, results),
                          encoding="utf-8")
        print(f"Merged {len(results)} probes into {out_json} and {out_md}")
        return 0

    client = Client(args.base_url)
    print(f"Loading {case_path} into {args.base_url}", flush=True)
    code, loaded = client.post(
        "/api/session/load_json_string",
        {"json_string": json.dumps(model, ensure_ascii=False)},
        timeout=120.0,
    )
    if code >= 400 or loaded.get("error"):
        raise RuntimeError(f"case load failed: HTTP {code}: {loaded.get('error')}")

    results: list[dict[str, Any]] = []
    for probe in probes:
        if probe.applicability is not None:
            applicable, reason = probe.applicability(inventory)
            if not applicable:
                row = {"probe_id": probe.probe_id, "domain": probe.domain,
                       "outcome": "NOT_APPLICABLE", "reason": reason}
                results.append(row)
                print(f"[NOT_APPLICABLE] {probe.probe_id}: {reason}", flush=True)
                continue

        started = time.monotonic()
        try:
            for path, payload, timeout_s in probe.prepare:
                prepare_code, prepare_data = client.post(
                    path, payload, timeout=timeout_s)
                if prepare_code >= 400 or prepare_data.get("error"):
                    raise RuntimeError(
                        f"preparation {path} failed: HTTP {prepare_code}: "
                        f"{prepare_data.get('error', prepare_data.get('status'))}"
                    )
            code, data = client.post(
                probe.path, probe.payload, timeout=probe.timeout_s)
            elapsed = time.monotonic() - started
            outcome, summary = summarize(probe, code, data, elapsed)
        except (TimeoutError, socket.timeout) as exc:
            elapsed = time.monotonic() - started
            recovered = client.cancel_and_wait()
            outcome = "TIMEOUT"
            summary = {"wall_ms": round(elapsed * 1000.0, 3),
                       "timeout_s": probe.timeout_s,
                       "cancel_recovered": recovered,
                       "error": str(exc)}
        except Exception as exc:
            elapsed = time.monotonic() - started
            outcome = "FAIL"
            summary = {"wall_ms": round(elapsed * 1000.0, 3),
                       "error": str(exc)[:500]}
        row = {"probe_id": probe.probe_id, "domain": probe.domain,
               "outcome": outcome, "summary": summary}
        results.append(row)
        print(f"[{outcome}] {probe.probe_id}: {json.dumps(summary, ensure_ascii=False)}",
              flush=True)

    report = {
        "schema": "yunnan_capability_audit_v1",
        "case": str(case_path),
        "base_url": args.base_url,
        "inventory": inventory,
        "results": results,
    }
    out_json = Path(args.out_json)
    out_md = Path(args.out_md)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
    out_md.write_text(markdown_report(case_path, inventory, results),
                      encoding="utf-8")
    print(f"Wrote {out_json} and {out_md}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
