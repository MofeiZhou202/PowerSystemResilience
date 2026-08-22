#!/usr/bin/env python3
"""Independent SCUC and multi-snapshot PF cross-validation for time_series.

The HySim driver supplies only HySim/GridLAB-D observations. This script rebuilds
the SCUC equations with SciPy/HiGHS, proves the four-period optimum by exhaustive
commitment enumeration, and solves the six AC snapshots with OpenDSS.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path
from typing import Any

import numpy as np
import opendssdirect as dss
from scipy.optimize import Bounds, LinearConstraint, linprog, milp
from scipy.sparse import coo_matrix, vstack


REPO = Path(__file__).resolve().parents[2]
DEFAULT_JSON = REPO / "external_data/time_series_validation/cross_engine_matrix.json"
DEFAULT_MD = REPO / "external_data/time_series_validation/cross_engine_matrix.md"


class Model:
    def __init__(self, case: dict[str, Any]) -> None:
        self.case = case
        self.generators = case["generator"]
        self.storage = case["storage"]
        self.load = np.asarray(case["load_mw"], dtype=float)
        self.g = len(self.generators)
        self.t = len(self.load)
        self.pg0 = 0
        self.u0 = self.pg0 + self.g * self.t
        self.su0 = self.u0 + self.g * self.t
        self.p0 = self.su0 + self.g * self.t
        self.ch0 = self.p0 + self.t
        self.soc0 = self.ch0 + self.t
        self.z0 = self.soc0 + self.t
        self.n = self.z0 + self.t

    def gt(self, offset: int, g: int, t: int) -> int:
        return offset + g * self.t + t

    def build(self, fixed_commit: np.ndarray | None = None,
              relax_storage_mode: bool = False) -> tuple[np.ndarray, np.ndarray,
                                                          np.ndarray, np.ndarray,
                                                          np.ndarray, np.ndarray,
                                                          np.ndarray]:
        """Rebuild equations from Wood--Wollenberg--Sheble, Ch. 4.

        Storage uses p=p_dis-p_ch and the exact SOC row from
        docs/modules/time_series/chapters/source_equivalent_model.tex.
        """
        c = np.zeros(self.n)
        lb = np.full(self.n, -np.inf)
        ub = np.full(self.n, np.inf)
        integrality = np.zeros(self.n, dtype=int)
        for gi, gen in enumerate(self.generators):
            for ti in range(self.t):
                ip = self.gt(self.pg0, gi, ti)
                iu = self.gt(self.u0, gi, ti)
                isu = self.gt(self.su0, gi, ti)
                c[ip] = gen["cost_c1"]
                c[iu] = gen["cost_c0"]
                c[isu] = 1.0
                lb[ip], ub[ip] = 0.0, gen["pmax_mw"]
                lb[iu], ub[iu] = 0.0, 1.0
                lb[isu], ub[isu] = 0.0, gen["startup_cost"]
                integrality[iu] = 1
                if fixed_commit is not None:
                    lb[iu] = ub[iu] = float(fixed_commit[gi, ti])
                    integrality[iu] = 0
        st = self.storage
        for ti in range(self.t):
            lb[self.p0 + ti], ub[self.p0 + ti] = (-st["p_charge_max_mw"],
                                                   st["p_discharge_max_mw"])
            lb[self.ch0 + ti], ub[self.ch0 + ti] = 0.0, st["p_charge_max_mw"]
            lb[self.soc0 + ti], ub[self.soc0 + ti] = st["soc_min"], st["soc_max"]
            lb[self.z0 + ti], ub[self.z0 + ti] = 0.0, 1.0
            integrality[self.z0 + ti] = 0 if relax_storage_mode else 1
        lb[self.soc0 + self.t - 1] = ub[self.soc0 + self.t - 1] = st["soc_init"]

        eq_rows: list[int] = []
        eq_cols: list[int] = []
        eq_vals: list[float] = []
        beq: list[float] = []
        row = 0
        for ti in range(self.t):
            for gi in range(self.g):
                eq_rows.append(row); eq_cols.append(self.gt(self.pg0, gi, ti)); eq_vals.append(1.0)
            eq_rows.append(row); eq_cols.append(self.p0 + ti); eq_vals.append(1.0)
            beq.append(float(self.load[ti])); row += 1
        cp = 1.0 / (st["eta_discharge"] * st["energy_mwh"])
        cc = (1.0 / st["eta_discharge"] - st["eta_charge"]) / st["energy_mwh"]
        for ti in range(self.t):
            eq_rows.extend([row, row]); eq_cols.extend([self.soc0 + ti, self.p0 + ti]); eq_vals.extend([1.0, cp])
            eq_rows.append(row); eq_cols.append(self.ch0 + ti); eq_vals.append(cc)
            if ti == 0:
                beq.append(st["soc_init"])
            else:
                eq_rows.append(row); eq_cols.append(self.soc0 + ti - 1); eq_vals.append(-1.0)
                beq.append(0.0)
            row += 1
        aeq = coo_matrix((eq_vals, (eq_rows, eq_cols)), shape=(row, self.n)).tocsr()

        in_rows: list[int] = []
        in_cols: list[int] = []
        in_vals: list[float] = []
        bineq: list[float] = []
        row = 0
        def add(coeff: dict[int, float], rhs: float) -> None:
            nonlocal row
            for col, val in coeff.items():
                in_rows.append(row); in_cols.append(col); in_vals.append(val)
            bineq.append(rhs); row += 1

        for gi, gen in enumerate(self.generators):
            for ti in range(self.t):
                ip, iu = self.gt(self.pg0, gi, ti), self.gt(self.u0, gi, ti)
                add({ip: 1.0, iu: -gen["pmax_mw"]}, 0.0)
                add({ip: -1.0, iu: gen["pmin_mw"]}, 0.0)
            ramp = gen["ramp_mw_per_period"]
            for ti in range(1, self.t):
                now, prev = self.gt(self.pg0, gi, ti), self.gt(self.pg0, gi, ti - 1)
                add({now: 1.0, prev: -1.0}, ramp)
                add({prev: 1.0, now: -1.0}, ramp)
            initial = 1.0 if gen["initial_on"] else 0.0
            startup = gen["startup_cost"]
            add({self.gt(self.su0, gi, 0): -1.0,
                 self.gt(self.u0, gi, 0): startup}, startup * initial)
            for ti in range(1, self.t):
                add({self.gt(self.su0, gi, ti): -1.0,
                     self.gt(self.u0, gi, ti): startup,
                     self.gt(self.u0, gi, ti - 1): -startup}, 0.0)
            up = int(math.ceil(gen["min_up_hr"]))
            if up >= 2:
                for ti in range(1, self.t):
                    for k in range(1, min(up - 1, self.t - 1 - ti) + 1):
                        add({self.gt(self.u0, gi, ti): 1.0,
                             self.gt(self.u0, gi, ti - 1): -1.0,
                             self.gt(self.u0, gi, ti + k): -1.0}, 0.0)
            down = int(math.ceil(gen["min_down_hr"]))
            if down >= 2:
                for ti in range(1, self.t):
                    for k in range(1, min(down - 1, self.t - 1 - ti) + 1):
                        add({self.gt(self.u0, gi, ti - 1): 1.0,
                             self.gt(self.u0, gi, ti): -1.0,
                             self.gt(self.u0, gi, ti + k): 1.0}, 1.0)
        for ti in range(self.t):
            p, ch, z = self.p0 + ti, self.ch0 + ti, self.z0 + ti
            add({p: -1.0, ch: -1.0}, 0.0)
            add({p: 1.0, ch: 1.0, z: -st["p_discharge_max_mw"]}, 0.0)
            add({ch: 1.0, z: st["p_charge_max_mw"]}, st["p_charge_max_mw"])
        aine = coo_matrix((in_vals, (in_rows, in_cols)), shape=(row, self.n)).tocsr()
        return c, lb, ub, integrality, aeq, np.asarray(beq), aine, np.asarray(bineq)


def solve_scipy_milp(model: Model) -> dict[str, Any]:
    c, lb, ub, integ, aeq, beq, aine, bineq = model.build()
    constraints = [LinearConstraint(aeq, beq, beq),
                   LinearConstraint(aine, -np.inf, bineq)]
    result = milp(c, integrality=integ, bounds=Bounds(lb, ub),
                  constraints=constraints, options={"mip_rel_gap": 1e-10})
    if not result.success:
        raise RuntimeError(f"SciPy/HiGHS MILP failed: {result.message}")
    return solution_record(model, result.x, float(result.fun), str(result.message))


def solve_enumeration(model: Model) -> dict[str, Any]:
    """Enumerate commitments; the storage-mode LP relaxation is exact here.

    With positive marginal generation cost, eta_c*eta_d<1, no must-run minimum
    output, and cyclic SOC, simultaneous charge/discharge is strictly dominated:
    reducing both flows preserves net injection while reducing losses and required
    generation. Therefore relaxing z is exact for this frozen benchmark.
    """
    best: tuple[float, np.ndarray, np.ndarray] | None = None
    feasible = 0
    for mask in range(1 << (model.g * model.t)):
        commitment = np.zeros((model.g, model.t), dtype=int)
        for gi in range(model.g):
            for ti in range(model.t):
                commitment[gi, ti] = (mask >> (gi * model.t + ti)) & 1
        c, lb, ub, _, aeq, beq, aine, bineq = model.build(commitment, True)
        result = linprog(c, A_ub=aine, b_ub=bineq, A_eq=aeq, b_eq=beq,
                         bounds=list(zip(lb, ub)), method="highs")
        if not result.success:
            continue
        feasible += 1
        if best is None or result.fun < best[0] - 1e-9:
            best = (float(result.fun), result.x, commitment)
    if best is None:
        raise RuntimeError("Commitment enumeration found no feasible schedule")
    record = solution_record(model, best[1], best[0], "global optimum by 256 commitment LPs")
    record["enumerated_commitments"] = 1 << (model.g * model.t)
    record["feasible_commitments"] = feasible
    record["commitment"] = best[2].tolist()
    return record


def solution_record(model: Model, x: np.ndarray, objective: float, status: str) -> dict[str, Any]:
    dispatch = [[float(x[model.gt(model.pg0, gi, ti)]) for ti in range(model.t)]
                for gi in range(model.g)]
    commit = [[int(round(x[model.gt(model.u0, gi, ti)])) for ti in range(model.t)]
              for gi in range(model.g)]
    storage = [float(x[model.p0 + ti]) for ti in range(model.t)]
    soc = [float(x[model.soc0 + ti]) for ti in range(model.t)]
    balance = [sum(dispatch[gi][ti] for gi in range(model.g)) + storage[ti] - model.load[ti]
               for ti in range(model.t)]
    return {"objective": objective, "status": status, "gen_dispatch_mw": dispatch,
            "commitment": commit, "storage_dispatch_mw": storage, "storage_soc": soc,
            "max_balance_residual_mw": max(abs(v) for v in balance)}


def opendss_snapshot(step: dict[str, Any]) -> dict[str, Any]:
    base_mva, base_kv = 10.0, 12.47
    zbase = base_kv * base_kv / base_mva
    r, x = 0.012 * zbase, 0.032 * zbase
    dss.Basic.ClearAll()
    commands = [
        f"New Circuit.tsxref basekv={base_kv} pu=1 phases=3 bus1=sourcebus mvasc3=1e9 mvasc1=1e9",
        ("New Line.L1 bus1=sourcebus.1.2.3 bus2=loadbus.1.2.3 phases=3 "
         f"r1={r:.15g} x1={x:.15g} r0={r:.15g} x0={x:.15g} c1=0 c0=0 length=1 units=km"),
        ("New Load.L1 bus1=loadbus.1.2.3 phases=3 conn=wye model=1 "
         f"kv={base_kv} kw={1000*step['load_p_mw']:.15g} kvar={1000*step['load_q_mvar']:.15g}"),
        "Set mode=snapshot controlmode=off maxiterations=100",
        "Solve",
    ]
    for command in commands:
        dss.Text.Command(command)
    if not dss.Solution.Converged():
        raise RuntimeError(f"OpenDSS failed at time step {step['step']}")
    dss.Circuit.SetActiveBus("loadbus")
    # OpenDSSDirect 0.9.4 exposes VMagAngle in physical V/deg for this circuit.
    # Convert phase-to-neutral voltage to pu on the authored line-to-line base;
    # Kersting, Distribution System Modeling and Analysis, 4th ed., Sec. 2.3.
    va = dss.Bus.VMagAngle()
    vm_pu = float(va[0]) / (base_kv * 1000.0 / math.sqrt(3.0))
    dss.Circuit.SetActiveElement("Line.L1")
    powers = dss.CktElement.Powers()
    pf_kw = sum(powers[2 * i] for i in range(3))
    qf_kvar = sum(powers[2 * i + 1] for i in range(3))
    return {"vm_load_pu": vm_pu, "va_load_deg": float(va[1]),
            "pf_mw": pf_kw / 1000.0, "qf_mvar": qf_kvar / 1000.0,
            "engine_version": dss.Basic.Version()}


def compare(payload: dict[str, Any]) -> dict[str, Any]:
    uc = payload["uc"]
    model = Model(uc)
    scipy_result = solve_scipy_milp(model)
    enum_result = solve_enumeration(model)
    hysim = uc["hysim"]
    objectives = [hysim["objective"], scipy_result["objective"], enum_result["objective"]]
    objective_spread = max(objectives) - min(objectives)
    objective_rel = objective_spread / max(1.0, abs(enum_result["objective"]))
    commitment_equal = (hysim["gen_commit"] == scipy_result["commitment"] ==
                        enum_result["commitment"])
    uc_gate = bool(hysim["feasible"] and objective_rel <= 1e-8 and commitment_equal and
                   scipy_result["max_balance_residual_mw"] <= 1e-8 and
                   enum_result["max_balance_residual_mw"] <= 1e-8)

    pf_rows = []
    max_odss_vm = max_odss_va = max_odss_p = max_odss_q = 0.0
    max_gld_vm = max_gld_va = max_gld_p = max_gld_q = 0.0
    for step in payload["pf"]["steps"]:
        oracle = opendss_snapshot(step)
        hvm = float(step["hysim_vm_pu"][1])
        hva = math.degrees(float(step["hysim_va_rad"][1]))
        branch = step["hysim_branch"]
        errors = {"vm_pu": abs(hvm - oracle["vm_load_pu"]),
                  "va_deg": abs(hva - oracle["va_load_deg"]),
                  "p_mw": abs(branch["pf_mw"] - oracle["pf_mw"]),
                  "q_mvar": abs(branch["qf_mvar"] - oracle["qf_mvar"])}
        max_odss_vm = max(max_odss_vm, errors["vm_pu"])
        max_odss_va = max(max_odss_va, errors["va_deg"])
        max_odss_p = max(max_odss_p, errors["p_mw"])
        max_odss_q = max(max_odss_q, errors["q_mvar"])
        for item in step["gridlabd_items"]:
            if item["kind"] == "bus_vm_pu": max_gld_vm = max(max_gld_vm, item["abs_error"])
            elif item["kind"] == "bus_va_deg": max_gld_va = max(max_gld_va, item["abs_error"])
            elif item["kind"] in ("branch_pf_mw", "branch_loss_p_mw"): max_gld_p = max(max_gld_p, item["abs_error"])
            elif item["kind"] in ("branch_qf_mvar", "branch_loss_q_mvar"): max_gld_q = max(max_gld_q, item["abs_error"])
        pf_rows.append({"step": step["step"], "scale": step["scale"],
                        "hysim": {"vm_load_pu": hvm, "va_load_deg": hva,
                                  "pf_mw": branch["pf_mw"], "qf_mvar": branch["qf_mvar"]},
                        "opendss": oracle, "abs_error": errors,
                        "gridlabd_equivalence_passed": step["gridlabd_equivalence_passed"]})
    opendss_gate = bool(max_odss_vm <= 5e-4 and max_odss_p <= 2e-2 and max_odss_q <= 2e-2)
    gridlabd_gate = bool(payload["pf"]["all_gridlabd_passed"])
    return {
        "schema": "hysim-time-series-cross-validation-v1",
        "uc": {"hysim": hysim, "scipy_highs": scipy_result,
               "commitment_enumeration": enum_result,
               "objective_abs_spread": objective_spread,
               "objective_relative_spread": objective_rel,
               "commitment_equal": commitment_equal, "passed": uc_gate},
        "pf": {"case": payload["pf"]["case"], "steps": pf_rows,
               "opendss_max_abs_error": {"vm_pu": max_odss_vm, "va_deg": max_odss_va,
                                         "p_mw": max_odss_p, "q_mvar": max_odss_q},
               "gridlabd_max_abs_error": {"vm_pu": max_gld_vm, "va_deg": max_gld_va,
                                           "p_mw": max_gld_p, "q_mvar": max_gld_q},
               "opendss_passed": opendss_gate, "gridlabd_passed": gridlabd_gate,
               "gridlabd_executable": payload["pf"]["gridlabd_executable"]},
        "passed": uc_gate and opendss_gate and gridlabd_gate,
        "scope": {"uc": "deterministic copper-plate SCUC with one lossy cyclic BESS",
                  "pf": "balanced positive-sequence constant-power snapshots",
                  "exclusions": ["OpenDSS/GridLAB-D do not validate commitment or lifecycle economics",
                                 "This matrix is not an unbalanced annual feeder certification"]},
    }


def markdown(report: dict[str, Any]) -> str:
    uc = report["uc"]
    pf = report["pf"]
    lines = ["# Time-series cross-engine matrix", "",
             "## Unit commitment", "",
             "| Oracle | Objective | Commitment | Max balance residual (MW) |",
             "|---|---:|---|---:|",
             f"| HySim/HiGHS | {uc['hysim']['objective']:.10f} | `{uc['hysim']['gen_commit']}` | n/a |",
             f"| SciPy/HiGHS rebuild | {uc['scipy_highs']['objective']:.10f} | `{uc['scipy_highs']['commitment']}` | {uc['scipy_highs']['max_balance_residual_mw']:.3e} |",
             f"| 256-sequence enumeration | {uc['commitment_enumeration']['objective']:.10f} | `{uc['commitment_enumeration']['commitment']}` | {uc['commitment_enumeration']['max_balance_residual_mw']:.3e} |",
             "", f"Objective relative spread: `{uc['objective_relative_spread']:.3e}`; pass: `{uc['passed']}`.", "",
             "## Six-step AC snapshot comparison", "",
             "| Step | Scale | HySim Vm (pu) | OpenDSS Vm (pu) | |dVm| | GridLAB-D pass |",
             "|---:|---:|---:|---:|---:|---|"]
    for row in pf["steps"]:
        lines.append(f"| {row['step']} | {row['scale']:.2f} | {row['hysim']['vm_load_pu']:.9f} | "
                     f"{row['opendss']['vm_load_pu']:.9f} | {row['abs_error']['vm_pu']:.3e} | "
                     f"{row['gridlabd_equivalence_passed']} |")
    lines += ["", f"OpenDSS maxima: `{json.dumps(pf['opendss_max_abs_error'])}`.",
              f"GridLAB-D maxima: `{json.dumps(pf['gridlabd_max_abs_error'])}`.", "",
              f"Overall pass: **{report['passed']}**.", "",
              "The external engines validate the per-step AC network snapshots only; they are not used as UC or lifecycle oracles."]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cpp-bin", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=DEFAULT_JSON)
    parser.add_argument("--markdown", type=Path, default=DEFAULT_MD)
    args = parser.parse_args()
    run = subprocess.run([str(args.cpp_bin)], capture_output=True, text=True,
                         check=False, timeout=300)
    if run.returncode != 0:
        raise RuntimeError(f"HySim/GridLAB-D driver failed ({run.returncode}): {run.stderr[-4000:]}")
    report = compare(json.loads(run.stdout))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text(markdown(report), encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "output": str(args.out),
                      "uc_objective_relative_spread": report["uc"]["objective_relative_spread"],
                      "opendss_max_abs_error": report["pf"]["opendss_max_abs_error"],
                      "gridlabd_max_abs_error": report["pf"]["gridlabd_max_abs_error"]}, indent=2))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
