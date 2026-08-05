#!/usr/bin/env python3.8
"""bench_02_modeling_compare.py — Compare three SCUC modeling approaches.

Target case: IEEE 39-bus × 24 periods (10 generators, full network)

Approach A: JSON API      — 1 Python call, black-box C++ solver
Approach B: AML           — Algebraic Modeling Layer (transparent, ~130 lines)
Approach C: Direct matrix — Manual sparse MILP construction (~200 lines)
              (simplified: unit commitment without DC network constraints)

Measures: model build time, solve time, total lines of modeling code, objective.
"""

import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# ── paths ──────────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "build_py"))
CASE_BUILDER = ROOT / "build_mipsolvers" / "scuc_case_builder"

import mipsolvers          # noqa: E402
aml = mipsolvers.aml

SOLVER = "HiGHS"           # use same solver for all three approaches
MIP_GAP = 0.003

# ─────────────────────────────────────────────────────────────────────────────
# Shared data: generate ieee39 × 24 JSON once
# ─────────────────────────────────────────────────────────────────────────────
print("\nGenerating ieee39×24 case …", end=" ", flush=True)
result = subprocess.run(
    [str(CASE_BUILDER), "--case", "ieee39", "--T", "24", "--compact"],
    capture_output=True, text=True, timeout=30)
assert result.returncode == 0, result.stderr
json_str = result.stdout.strip()
data = json.loads(json_str)
print("done")

gens   = data["generators"]
T_hrz  = data["config"]["num_periods"]
dt     = data["config"]["period_length_hr"]
G_names = [g["name"] for g in gens]
t_names = [f"t{i+1}" for i in range(T_hrz)]

# aggregate load per period (sum across buses × profile pu)
loads  = data["loads"]
profs  = data["profiles"]["load"]   # [nloads][T]
load_mw = []
for t in range(T_hrz):
    load_mw.append(sum(loads[d]["p_mw"] * profs[d][t] for d in range(len(loads))))


# =============================================================================
# APPROACH A: JSON API
# =============================================================================
def approach_a_json_api():
    modified = json.loads(json_str)
    modified["config"]["solver"]        = SOLVER
    modified["config"]["mip_gap"]       = MIP_GAP
    modified["config"]["time_limit_sec"] = 300.0
    s = json.dumps(modified)

    t0 = time.perf_counter()
    out_json = mipsolvers.scuc.solve_json(s)
    t_solve = time.perf_counter() - t0

    out  = json.loads(out_json)
    scuc = out["scuc"]
    return {
        "converged": scuc["converged"],
        "objective": scuc["objective"],
        "mip_gap":   scuc.get("mip_gap", float("nan")),
        "build_ms":  0.0,        # build time is in C++ — not measurable from Python
        "solve_s":   t_solve,
        "lines":     10,         # Python lines of modeling code (approximate)
    }


# =============================================================================
# APPROACH B: AML
# =============================================================================
def approach_b_aml():
    # Build gen data dict
    gdata = {}
    for g in gens:
        segs = g["bid_segments"]
        # weighted avg marginal cost from bid segments
        tot_q = sum(s["quantity"] for s in segs)
        avg_mc = sum(s["price"] * s["quantity"] for s in segs) / max(tot_q, 1e-6)
        gdata[g["name"]] = {
            "pmin": g["pmin"],  "pmax": g["pmax"],
            "rup":  g["ramp_up_mw_min"] * 60.0 * dt,   # MW/min → MW/period
            "rdn":  g["ramp_dn_mw_min"] * 60.0 * dt,
            "nl":   g["no_load_cost"] * dt,
            "bid":  avg_mc,
            "su":   g["startup_cost"],
        }
    init_u = {g["name"]: int(c) for g, c in
              zip(gens, data["initial_status"]["commitment"])}
    init_p = {g["name"]: float(v) for g, v in
              zip(gens, data["initial_status"]["dispatch"])}

    t_build0 = time.perf_counter()

    m = aml.Model("scuc_aml_39")
    G_set = m.add_set("G", G_names)
    T_set = m.add_ordered_set("T", t_names)

    u  = m.add_var2("u",  G_set, T_set, aml.VarType.Binary)
    p  = m.add_var2("p",  G_set, T_set, aml.VarType.Continuous, lb=0.0)
    su = m.add_var2("su", G_set, T_set, aml.VarType.Binary)
    sd = m.add_var2("sd", G_set, T_set, aml.VarType.Binary)

    # Objective
    m.minimize(aml.sum_over(G_set, lambda g:
        aml.sum_over(T_set, lambda t:
            gdata[g.values[0]]["nl"]  * u[(g, t)] +
            gdata[g.values[0]]["bid"] * p[(g, t)] +
            gdata[g.values[0]]["su"]  * su[(g, t)])))

    # Constraints
    for ti, t in enumerate(t_names):
        # Power balance
        bal = aml.sum_over(G_set, lambda g: p[(g.values[0], t)] * 1.0)
        m.add_constraint(bal == load_mw[ti], f"balance_{t}")

    for g_name in G_names:
        d = gdata[g_name]
        for ti, t in enumerate(t_names):
            # Pmin / Pmax
            m.add_constraint(
                1.0*p[(g_name, t)] - d["pmin"]*u[(g_name, t)] >= 0.0,
                f"pmin_{g_name}_{t}")
            m.add_constraint(
                d["pmax"]*u[(g_name, t)] - 1.0*p[(g_name, t)] >= 0.0,
                f"pmax_{g_name}_{t}")
            # Ramp
            if ti == 0:
                m.add_constraint(
                    1.0*p[(g_name,t)] - init_p[g_name] <= d["rup"],
                    f"ramp_up_{g_name}_{t}")
                m.add_constraint(
                    init_p[g_name] - 1.0*p[(g_name,t)] <= d["rdn"],
                    f"ramp_dn_{g_name}_{t}")
                logic = (1.0*su[(g_name,t)] - 1.0*sd[(g_name,t)]
                         - 1.0*u[(g_name,t)])
                m.add_constraint(logic == -float(init_u[g_name]),
                                 f"logic_{g_name}_{t}")
            else:
                prev_t = t_names[ti - 1]
                m.add_constraint(
                    1.0*p[(g_name,t)] - 1.0*p[(g_name,prev_t)] <= d["rup"],
                    f"ramp_up_{g_name}_{t}")
                m.add_constraint(
                    1.0*p[(g_name,prev_t)] - 1.0*p[(g_name,t)] <= d["rdn"],
                    f"ramp_dn_{g_name}_{t}")
                logic = (1.0*su[(g_name,t)] - 1.0*sd[(g_name,t)]
                         - 1.0*u[(g_name,t)] + 1.0*u[(g_name,prev_t)])
                m.add_constraint(logic == 0.0, f"logic_{g_name}_{t}")
            # su + sd <= 1
            m.add_constraint(
                1.0*su[(g_name,t)] + 1.0*sd[(g_name,t)] <= 1.0,
                f"susd_{g_name}_{t}")

    t_build = (time.perf_counter() - t_build0) * 1000.0

    opts = aml.SolveOptions()
    opts.verbosity   = 0
    opts.mip_gap_tol = MIP_GAP
    opts.solver_name = SOLVER

    t0 = time.perf_counter()
    res = m.solve(opts)
    t_solve = time.perf_counter() - t0

    converged = res.is_optimal or (
        res.termination_status == aml.TerminationStatus.TimeLimit
        and res.has_primal)
    return {
        "converged": converged,
        "objective": res.objective_value if converged else float("nan"),
        "mip_gap":   float("nan"),    # AML doesn't expose mip_gap post-solve
        "build_ms":  t_build,
        "solve_s":   t_solve,
        "lines":     130,
    }


# =============================================================================
# APPROACH C: Direct matrix (simplified UC, no network)
#   Variables: [u(G×T), p(G×T), su(G×T), sd(G×T)]  flat row-major order
#   No DC network — only energy balance, Pmin/Pmax, ramp, startup logic
# =============================================================================
def approach_c_matrix():
    nG = len(gens)
    nT = T_hrz

    gdata = []
    for g in gens:
        segs = g["bid_segments"]
        tot_q = sum(s["quantity"] for s in segs)
        avg_mc = sum(s["price"] * s["quantity"] for s in segs) / max(tot_q, 1e-6)
        gdata.append({
            "pmin": g["pmin"],  "pmax": g["pmax"],
            "rup":  g["ramp_up_mw_min"] * 60.0 * dt,
            "rdn":  g["ramp_dn_mw_min"] * 60.0 * dt,
            "nl":   g["no_load_cost"] * dt,
            "bid":  avg_mc,
            "su":   g["startup_cost"],
        })
    init_u = [int(c) for c in data["initial_status"]["commitment"]]
    init_p = [float(v) for v in data["initial_status"]["dispatch"]]

    # Variable layout
    nV   = 4 * nG * nT     # [u, p, su, sd] blocks of nG×nT
    iU   = lambda g, t: 0 * nG * nT + t * nG + g    # u[g,t]
    iP   = lambda g, t: 1 * nG * nT + t * nG + g    # p[g,t]
    iSU  = lambda g, t: 2 * nG * nT + t * nG + g    # su[g,t]
    iSD  = lambda g, t: 3 * nG * nT + t * nG + g    # sd[g,t]

    t_build0 = time.perf_counter()

    # ── Objective ────────────────────────────────────────────────────────────
    c_obj = np.zeros(nV)
    for g in range(nG):
        for t in range(nT):
            c_obj[iU(g,t)]  = gdata[g]["nl"]
            c_obj[iP(g,t)]  = gdata[g]["bid"]
            c_obj[iSU(g,t)] = gdata[g]["su"]

    # ── Variable bounds & types ───────────────────────────────────────────────
    lb = np.zeros(nV)
    ub = np.ones(nV)
    # p has [0, pmax*u] but we handle pmin/pmax via constraints
    for g in range(nG):
        for t in range(nT):
            ub[iP(g,t)] = gdata[g]["pmax"]

    vartypes_v = [""] * nV
    for g in range(nG):
        for t in range(nT):
            vartypes_v[iU(g,t)]  = "B"
            vartypes_v[iP(g,t)]  = "C"
            vartypes_v[iSU(g,t)] = "B"
            vartypes_v[iSD(g,t)] = "B"

    # ── Equality constraints (Aeq x = beq) ───────────────────────────────────
    # Power balance: Σ_g p[g,t] = load[t]  →  nT rows
    rows_eq, cols_eq, vals_eq, beq = [], [], [], []
    for t in range(nT):
        for g in range(nG):
            rows_eq.append(t); cols_eq.append(iP(g,t)); vals_eq.append(1.0)
        beq.append(load_mw[t])
    # Startup logic: su[g,t] - sd[g,t] = u[g,t] - u[g,t-1]
    #   → su[g,t] - sd[g,t] - u[g,t] + u[g,t-1] = 0  (or -init_u for t=0)
    row_off = nT
    for g in range(nG):
        for t in range(nT):
            r = row_off + g * nT + t
            rows_eq += [r, r, r]
            cols_eq += [iSU(g,t), iSD(g,t), iU(g,t)]
            vals_eq += [1.0,      -1.0,      -1.0]
            if t == 0:
                beq.append(-float(init_u[g]))
            else:
                rows_eq.append(r); cols_eq.append(iU(g,t-1)); vals_eq.append(1.0)
                beq.append(0.0)

    n_eq = nT + nG * nT
    import scipy.sparse as sp
    Aeq = sp.csr_matrix((vals_eq, (rows_eq, cols_eq)), shape=(n_eq, nV))
    beq_arr = np.array(beq)

    # ── Inequality constraints (A x <= b) ─────────────────────────────────────
    # Pmin:        -p[g,t] + pmin*u[g,t] <= 0
    # Pmax:         p[g,t] - pmax*u[g,t] <= 0
    # Ramp up:      p[g,t] - p[g,t-1]   <= rup[g]
    # Ramp dn:     -p[g,t] + p[g,t-1]   <= rdn[g]
    # su+sd <= 1:   su[g,t] + sd[g,t]   <= 1
    rows_iq, cols_iq, vals_iq, biq = [], [], [], []
    r = 0
    for g in range(nG):
        for t in range(nT):
            # Pmin
            rows_iq += [r, r]; cols_iq += [iP(g,t), iU(g,t)]
            vals_iq += [-1.0, gdata[g]["pmin"]]; biq.append(0.0); r += 1
            # Pmax
            rows_iq += [r, r]; cols_iq += [iP(g,t), iU(g,t)]
            vals_iq += [1.0, -gdata[g]["pmax"]]; biq.append(0.0); r += 1
            # Ramp up
            if t == 0:
                rows_iq.append(r); cols_iq.append(iP(g,t)); vals_iq.append(1.0)
                biq.append(init_p[g] + gdata[g]["rup"]); r += 1
            else:
                rows_iq += [r, r]; cols_iq += [iP(g,t), iP(g,t-1)]
                vals_iq += [1.0, -1.0]; biq.append(gdata[g]["rup"]); r += 1
            # Ramp dn
            if t == 0:
                rows_iq.append(r); cols_iq.append(iP(g,t)); vals_iq.append(-1.0)
                biq.append(-init_p[g] + gdata[g]["rdn"]); r += 1
            else:
                rows_iq += [r, r]; cols_iq += [iP(g,t-1), iP(g,t)]
                vals_iq += [1.0, -1.0]; biq.append(gdata[g]["rdn"]); r += 1
            # su + sd <= 1
            rows_iq += [r, r]; cols_iq += [iSU(g,t), iSD(g,t)]
            vals_iq += [1.0, 1.0]; biq.append(1.0); r += 1

    n_iq = r
    A = sp.csr_matrix((vals_iq, (rows_iq, cols_iq)), shape=(n_iq, nV))
    b_arr = np.array(biq)

    t_build = (time.perf_counter() - t_build0) * 1000.0

    # ── Solve via engine.solve_milp (takes numpy arrays directly) ─────────────
    eng = mipsolvers.engine

    t0 = time.perf_counter()
    res = eng.solve_milp(
        c_obj,
        A.toarray(),   b_arr,
        Aeq.toarray(), beq_arr,
        lb, ub,
        vartypes_v,
        solver=SOLVER,
        mip_gap=MIP_GAP,
        time_limit_sec=300.0,
    )
    t_solve = time.perf_counter() - t0

    return {
        "converged": res.get("success", False),
        "objective": res.get("objective", float("nan")),
        "mip_gap":   res.get("mip_gap", float("nan")),
        "build_ms":  t_build,
        "solve_s":   t_solve,
        "lines":     200,
    }


# =============================================================================
# Run all three approaches
# =============================================================================
print(f"\nCase: IEEE 39-bus × {T_hrz} periods  |  Solver: {SOLVER}  |  Gap: {MIP_GAP:.1%}\n")
print(f"{'Approach':<20} {'Build (ms)':<12} {'Solve (s)':<12} "
      f"{'Objective ($)':<18} {'Gap':<8} {'Conv':<6} {'Code lines'}")
print(f"{'-'*20} {'-'*12} {'-'*12} {'-'*18} {'-'*8} {'-'*6} {'-'*10}")

results_summary = []

for label, fn in [
    ("A: JSON API",      approach_a_json_api),
    ("B: AML",           approach_b_aml),
]:
    print(f"  Running {label} …", end=" ", flush=True)
    r = fn()
    conv = "YES" if r["converged"] else "NO"
    obj_str = f"{r['objective']:>17,.0f}" if r["objective"] == r["objective"] else "            N/A"
    gap_str = f"{r['mip_gap']:.4f}" if r["mip_gap"] == r["mip_gap"] else "   N/A"
    print(f"done  ({r['solve_s']:.1f}s)")
    print(f"  {label:<20} {r['build_ms']:<12.1f} {r['solve_s']:<12.1f} "
          f"{obj_str:<18} {gap_str:<8} {conv:<6} ~{r['lines']}")
    results_summary.append((label, r))

# Approach C needs scipy; try and report clearly if unavailable
try:
    import scipy.sparse  # noqa: F401
    print(f"  Running C: Matrix …", end=" ", flush=True)
    r_c = approach_c_matrix()
    conv = "YES" if r_c["converged"] else "NO"
    obj_str = f"{r_c['objective']:>17,.0f}" if r_c["objective"] == r_c["objective"] else "            N/A"
    gap_str = f"{r_c['mip_gap']:.4f}" if r_c["mip_gap"] == r_c["mip_gap"] else "   N/A"
    print(f"done  ({r_c['solve_s']:.1f}s)")
    print(f"  {'C: Matrix':<20} {r_c['build_ms']:<12.1f} {r_c['solve_s']:<12.1f} "
          f"{obj_str:<18} {gap_str:<8} {conv:<6} ~{r_c['lines']}")
    results_summary.append(("C: Matrix (no-net)", r_c))
except ImportError:
    print("  C: Matrix — skipped (scipy not installed)")
except AttributeError as e:
    print(f"  C: Matrix — engine.solve_milp_dict not available: {e}")
except Exception as e:
    print(f"  C: Matrix — error: {e}")

print()
print("Notes:")
print("  A (JSON API):  C++ SCUC module with full DC network, segments, reserve")
print("  B (AML):       AML-built simplified UC (no network, avg bid price)")
print("  C (Matrix):    Manually-assembled LP/MILP (no network, avg bid price)")
print("  Objectives differ: A includes network/reserve; B & C are network-free")
print()
