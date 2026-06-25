#!/usr/bin/env python3
"""Drive the running run_gui_server time-series PF/OPF for the nansha case and
report convergence + which hours fail. Replicates the web frontend flow:
  load_json_string -> set_ts_config -> run_ts_pf
A failing hour leaves pf.vm empty, so vm_min==0 identifies it.
"""
import json, sys, urllib.request, urllib.error

BASE = "http://127.0.0.1:8088"
# Optional 4th CLI arg overrides the case file (e.g. a browser-exported canvas dump),
# so we can run the EXACT system the web UI ran and compare against disk.
CASE = "d:/luosipeng/Code_Script/HybridACDCDistribtutionSystemsSimulation/external_data/classical_example/nansha_full_network.json"
if len(sys.argv) > 4 and sys.argv[4].strip():
    CASE = sys.argv[4].strip()
IRR  = "d:/luosipeng/Code_Script/HybridACDCDistribtutionSystemsSimulation/external_data/profiles/nansha_network/nansha_full_network_irradiance_8760.json"
LOADS= "d:/luosipeng/Code_Script/HybridACDCDistribtutionSystemsSimulation/external_data/profiles/nansha_network/nansha_full_network_loads_8760.json"

IRR_ID = 2
LOAD_BASE = 100

def post(path, body, timeout=1800):
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(BASE+path, data=data,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))

def main():
    num_steps = int(sys.argv[1]) if len(sys.argv) > 1 else 24
    run_opf = (sys.argv[2].lower() in ("1","true","opf")) if len(sys.argv) > 2 else True
    skip_uc = (sys.argv[3].lower() in ("1","true")) if len(sys.argv) > 3 else True

    with open(CASE, encoding="utf-8") as f:
        case_str = f.read()
    st, j = post("/api/session/load_json_string", {"json_string": case_str})
    if st != 200:
        print("load failed:", j); return
    print(f"loaded: buses_ac={j.get('num_ac_buses', j.get('n_ac_bus','?'))} "
          f"gens={j.get('num_generators','?')} vsc={j.get('num_vsc','?')}")

    with open(IRR, encoding="utf-8") as f:
        irr = json.load(f)
    with open(LOADS, encoding="utf-8") as f:
        loads = json.load(f)

    irr_vals = irr["values"][:num_steps]
    profiles = [{"id": IRR_ID, "name": "irradiance", "values": irr_vals}]
    load_map = []
    for i, lp in enumerate(loads["load_profiles"]):
        pid = LOAD_BASE + i
        nom = float(lp.get("p_mw_nominal", 0) or 0)
        mw = lp.get("p_mw_values", [])[:num_steps]
        if abs(nom) > 1e-12:
            scaled = [v/nom for v in mw]
        else:
            scaled = mw
        profiles.append({"id": pid, "name": lp.get("name", f"load_{i}"), "values": scaled})
        if isinstance(lp.get("load_index"), int):
            load_map.append({"load_index": lp["load_index"], "profile_id": pid})
        elif isinstance(lp.get("bus"), int):
            load_map.append({"bus": lp["bus"], "profile_id": pid})

    st, j = post("/api/session/set_ts_config", {
        "num_steps": num_steps, "step_duration_hr": 1.0,
        "profiles": profiles, "load_profile_map": load_map,
        "assign_all_pv_to": IRR_ID,
    })
    if st != 200:
        print("set_ts_config failed:", j); return
    print(f"ts_config: steps={j['num_steps']} profiles={j['num_profiles']} "
          f"loads_mapped={j.get('num_loads_mapped')} materialized={j.get('num_loads_materialized')}")

    st, j = post("/api/session/run_ts_pf", {
        "num_steps": num_steps, "skip_uc": skip_uc, "run_opf": run_opf})
    if st != 200:
        print("run_ts_pf failed:", j); return

    ns = j["num_steps"]
    nconv = j["num_converged"]
    nopf = j["num_opf_converged"]
    print(f"\n=== RESULT: PF {nconv}/{ns}  OPF {nopf}/{ns}  run_opf={run_opf} skip_uc={skip_uc} ===")
    vmin = j.get("vm_min", [])
    vmax = j.get("vm_max", [])
    losses = j.get("losses_mw", [])
    # Save per-step diagnostics for offline analysis
    with open("scripts/tspf_diag.json", "w") as f:
        json.dump({"num_steps": ns, "num_converged": nconv, "num_opf_converged": nopf,
                   "vm_min": vmin, "vm_max": vmax, "losses_mw": losses}, f)
    # Heuristic failure detection: empty vm OR abnormal voltage (non-converged)
    fails = []
    for t in range(ns):
        if t >= len(vmin): continue
        vlo, vhi = vmin[t], vmax[t]
        if (vlo == 0.0 and vhi == 0.0) or vlo < 0.70 or vhi > 1.30:
            fails.append(t)
    print(f"suspected failing hours (vm empty or <0.70/>1.30): {len(fails)}")
    if fails:
        print("first 40:", fails[:40])
    # Distribution of vm_min/vm_max
    nz = [v for v in vmin if v > 0]
    if nz:
        sv = sorted(nz)
        print(f"vm_min: global_min={min(nz):.4f} p1={sv[len(sv)//100]:.4f} median={sv[len(sv)//2]:.4f}")
        svx = sorted(vmax)
        print(f"vm_max: median={svx[len(svx)//2]:.4f} p99={svx[len(svx)*99//100]:.4f} global_max={max(vmax):.4f}")
    return fails

if __name__ == "__main__":
    main()
