#!/usr/bin/env python3
"""Build a small time-series profile set containing ONLY the failing hours
(plus a few known-good hours as controls), and run it through the live server
for fast iteration. Reuses validate_tspf's POST + scaling logic inline.
"""
import json, urllib.request, urllib.error

BASE = "http://127.0.0.1:8088"
ROOT = "d:/luosipeng/Code_Script/HybridACDCDistribtutionSystemsSimulation"
CASE = f"{ROOT}/external_data/classical_example/nansha_full_network.json"
IRR  = f"{ROOT}/external_data/profiles/nansha_network/nansha_full_network_irradiance_8760.json"
LOADS= f"{ROOT}/external_data/profiles/nansha_network/nansha_full_network_loads_8760.json"
IRR_ID, LOAD_BASE = 2, 100

# 25 failing hours from the full run + 3 control hours (known good: peak 5966, 0, 12)
FAIL = [145,295,458,799,1039,1323,1638,1642,1803,1810,2864,6585,6775,7197,8046,8208,8550]
CTRL = [0, 12, 5966]
HOURS = FAIL + CTRL

def post(path, body, timeout=600):
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(BASE+path, data=data,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode("utf-8"))

def main():
    with open(CASE, encoding="utf-8") as f:
        case_str = f.read()
    st, j = post("/api/session/load_json_string", {"json_string": case_str})
    if st != 200:
        print("load failed:", j); return

    with open(IRR, encoding="utf-8") as f:
        irr = json.load(f)
    with open(LOADS, encoding="utf-8") as f:
        loads = json.load(f)

    n = len(HOURS)
    irr_vals = [irr["values"][h] for h in HOURS]
    profiles = [{"id": IRR_ID, "name": "irradiance", "values": irr_vals}]
    load_map = []
    for i, lp in enumerate(loads["load_profiles"]):
        pid = LOAD_BASE + i
        nom = float(lp.get("p_mw_nominal", 0) or 0)
        full = lp.get("p_mw_values", [])
        mw = [full[h] for h in HOURS]
        scaled = [v/nom for v in mw] if abs(nom) > 1e-12 else mw
        profiles.append({"id": pid, "name": lp.get("name", f"l{i}"), "values": scaled})
        if isinstance(lp.get("load_index"), int):
            load_map.append({"load_index": lp["load_index"], "profile_id": pid})
        elif isinstance(lp.get("bus"), int):
            load_map.append({"bus": lp["bus"], "profile_id": pid})

    st, j = post("/api/session/set_ts_config", {
        "num_steps": n, "step_duration_hr": 1.0,
        "profiles": profiles, "load_profile_map": load_map, "assign_all_pv_to": IRR_ID})
    if st != 200:
        print("set_ts_config failed:", j); return

    st, j = post("/api/session/run_ts_pf", {"num_steps": n, "skip_uc": True, "run_opf": True})
    if st != 200:
        print("run failed:", j); return

    losses = j.get("losses_mw", [])
    print(f"PF {j['num_converged']}/{j['num_steps']}  OPF {j['num_opf_converged']}/{j['num_steps']}")
    still_fail = []
    for k, h in enumerate(HOURS):
        ok = k < len(losses) and losses[k] > 0.0
        tag = "CTRL" if h in CTRL else "fail?"
        if not ok and h in FAIL:
            still_fail.append(h)
        if not ok:
            print(f"  h{h} [{tag}]: STILL FAILS (loss=0)")
    print(f"\n{len(still_fail)}/{len(FAIL)} originally-failing hours still fail")

if __name__ == "__main__":
    main()
