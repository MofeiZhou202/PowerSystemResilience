#!/usr/bin/env python3
"""Adversarial market workflow experiments using sourced AEMO shapes.

See docs/modules/market/aemo_validation.md for equations and admission bounds.
"""
import copy
import json
import math
import platform
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from market_bid_empirical import ROOT, save, sha256
from independent_audit import audit, indexed, enumerate_uc, audit_completed_hour
from aemo_data import OUT


def series(x):
    return [x]*98


def empty(schema):
    if "enum" in schema:
        return schema["enum"][0]
    if schema["type"] == "object":
        return {k: empty(v) for k, v in schema["properties"].items()}
    if schema["type"] == "array":
        return [empty(schema["items"]) for _ in range(schema["minItems"])]
    if schema["type"] == "string":
        return "Authored physical boundary for AEMO input validation"
    return max(0, schema.get("minimum", 0))


class Experiment:
    def __init__(self, base):
        self.base, self.checks, self.runs = base, [], []

    def api(self, route, body=None, expected=200):
        req = urllib.request.Request(self.base+"/api/session/"+route,
              data=None if body is None else json.dumps(body, allow_nan=False).encode(),
              headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=180) as response:
                code, data = response.status, json.load(response)
        except urllib.error.HTTPError as error:
            code, data = error.code, json.load(error)
        if code != expected:
            raise RuntimeError(f"{route}: {code} != {expected}: {str(data)[:600]}")
        return data

    def record(self, name, passed, evidence):
        self.checks.append({"name": name, "passed": bool(passed), "evidence": evidence})
        save(OUT/"campaign_checks.json", self.checks)
        print(name, "PASS" if passed else "FAIL", str(evidence)[:300], flush=True)

    def fixture(self, action="example"):
        s = self.api("southern_market")
        return self.api("southern_market", {"action": action, "revision": s["revision"]})["boundary"]

    def boundary(self, b):
        s = self.api("southern_market")
        return self.api("southern_market", {"action": "save", "revision": s["revision"], "boundary": b})

    def solve(self, name, b, expect_feasible=True):
        s = self.boundary(b)
        result = self.api("run_southern_market", {"revision": s["revision"]})
        save(OUT/"runs"/(name+".json"), {"input": b, "result": result})
        self.runs.append({"name": name, "status": result["status"], "runtime_sec": result["runtime_sec"]})
        if expect_feasible:
            report = audit(b, result)
            self.record(name+"/independent_physics", report["passed"], report)
        else:
            self.record(name+"/expected_failure", not result["schedule_feasible"], {"status": result["status"]})
        return result


def network_case(seed, schema, profile, p1, p2):
    b = copy.deepcopy(seed)
    b["source"] = "AEMO NSW TOTALDEMAND shape / sampled GEN bids; artificial currency AUD/100; authored 2-bus physics"
    b["execution"].update(ac_security="schedule_only", solver="highs", mip_gap=.01)
    load = profile["TOTALDEMAND"]["quarter_hour_mw"]
    d = [100*x/max(load) for x in load]
    d += [min(d), max(d)]
    b["areas"][0]["load_mw"] = d
    b["buses"][0]["load_mw"] = series(0)
    node = copy.deepcopy(b["buses"][0]); node.update(id=2, load_mw=d)
    b["buses"].append(node)
    g = b["generators"][0]
    g.update(must_on=series(1), initial_power_mw=0, segments=[{"quantity_mw": 200, "price_per_mwh": p1}])
    second = copy.deepcopy(g); second.update(id=2, bus=2, name="AEMO positive bid high sample")
    second["segments"][0]["price_per_mwh"] = p2
    b["generators"].append(second)
    line = empty(schema["properties"]["branches"]["items"])
    line.update(id=10, from_bus=1, to_bus=2, available=series(1), x_pu=.1, tap=1,
                min_mw=series(-30), max_mw=series(30), rate_mva=series(100))
    b["branches"] = [line]
    return b


def run_network(e, b, p1, p2):
    base = e.solve("network_congested", b)
    powers = indexed(base["sced"]["generators"])
    prices = indexed(base["lmp"]["buses"])
    error = max(abs(powers[1]["power_mw"][t]-30) for t in range(98))
    error = max(error, max(abs(prices[i]["lmp_per_mwh"][t]-p) for i, p in ((1, p1), (2, p2)) for t in range(98)))
    e.record("two_bus_merit_and_lmp", error <= 1e-5, {"max_error": error, "source_prices": [p1, p2]})
    wider = copy.deepcopy(b); wider["branches"][0].update(min_mw=series(-150), max_mw=series(150))
    result = e.solve("network_relaxed", wider)
    e.record("feasible_set_inclusion", result["scuc"]["objective"] <= base["scuc"]["objective"]+1e-5,
             {"congested": base["scuc"]["objective"], "relaxed": result["scuc"]["objective"]})
    perturb = copy.deepcopy(b); perturb["areas"][0]["load_mw"][0] += .001
    result = e.solve("network_load_derivative", perturb)
    derivative = (result["sced"]["objective"]-base["sced"]["objective"])/.00025
    e.record("lmp_finite_difference", abs(derivative-p2) <= 1e-4, {"derivative": derivative, "lmp": p2})
    outage = copy.deepcopy(b); outage["branches"][0]["available"][0] = 0
    e.solve("network_line_outage", outage)
    deficit = copy.deepcopy(outage); deficit["generators"].pop()
    e.solve("isolated_load_strict_failure", deficit, False)
    deficit["execution"].update(balance_policy="diagnostic", balance_penalty_per_mwh=100000)
    r = e.solve("isolated_load_diagnostic", deficit)
    actual = indexed(r["sced"]["buses"])[2]["deficit_mw"][0]
    expected = deficit["areas"][0]["load_mw"][0]
    e.record("delta_p_is_deficit_not_equation_residual", abs(actual-expected)<1e-5, {"expected_mw": expected, "actual_mw": actual})
    overloaded = copy.deepcopy(deficit); overloaded["branches"][0]["available"][0] = 1
    r = e.solve("transmission_soft_overload", overloaded)
    expected = overloaded["areas"][0]["load_mw"][0]-30
    e.record("delta_pij_is_limit_slack", abs(r["sced"]["branches"][0]["slack_plus_mw"][0]-expected)<1e-5,
             {"expected_overload_mw": expected, "actual": r["sced"]["branches"][0]["slack_plus_mw"][0]})
    scaled = copy.deepcopy(b)
    for g in scaled["generators"]:
        g["segments"][0]["price_per_mwh"] *= 1.5
    r = e.solve("price_scale", scaled)
    e.record("positive_price_scale", abs(r["sced"]["objective"]-1.5*base["sced"]["objective"])<1e-5 and
             abs(r["lmp"]["buses"][1]["lmp_per_mwh"][0]-1.5*p2)<1e-5, {"scale": 1.5})
    permutation = copy.deepcopy(b)
    for table in ("generators", "buses", "branches"):
        permutation[table].reverse()
    for bus in permutation["buses"]:
        bus["id"] += 100
    for g in permutation["generators"]:
        g["id"] += 200; g["bus"] += 100
    for line in permutation["branches"]:
        line["id"] += 300; line["from_bus"] += 100; line["to_bus"] += 100
    r = e.solve("stable_id_permutation", permutation)
    e.record("stable_id_permutation_equivalence", abs(r["sced"]["objective"]-base["sced"]["objective"])<1e-5, {})
    for solver in ("native", "gurobi"):
        other = copy.deepcopy(b); other["execution"]["solver"] = solver
        r = e.solve("backend_"+solver, other)
        e.record("backend_equivalence_"+solver, abs(r["sced"]["objective"]-base["sced"]["objective"])<1e-5, {})
    other = copy.deepcopy(b); other["execution"]["formulation"] = "reference"
    r = e.solve("reference_formulation", other)
    e.record("reference_compact_equivalence", abs(r["scuc"]["objective"]-base["scuc"]["objective"])<1e-5, {})
    return base


def run_uc(e, seed, p1, p2, profile):
    b = copy.deepcopy(seed); b["execution"].update(ac_security="schedule_only", mip_gap=.000001)
    x = profile["TOTALDEMAND"]["quarter_hour_mw"]
    selected = [x.index(min(x)), x.index(max(x)), x.index(min(x)), 95]
    load = [20+100*(x[k]-min(x))/(max(x)-min(x)) for k in selected]
    b["areas"][0]["load_mw"] = load+[0]*94
    b["buses"][0]["load_mw"] = load+[0]*94
    seedgen = copy.deepcopy(b["generators"][0]); b["generators"] = []
    caps, costs = [80, 140], [p2*10, p2]
    no_load = [p2, 40*p2]
    for i in range(2):
        g = copy.deepcopy(seedgen)
        g.update(id=i+1, initial_on=0, initial_power_mw=0, initial_state_minutes=1440,
                 must_off=[0]*4+[1]*94, pmax_mw=series(caps[i]), startup_cost=[costs[i]]*3,
                 minimum_cost_per_hour=no_load[i],
                 min_up_minutes=30 if i == 0 else 15,
                 segments=[{"quantity_mw": caps[i], "price_per_mwh": [p1, p2][i]}])
        b["generators"].append(g)
    oracle = enumerate_uc(load, caps, [p1, p2], costs, [2, 1], no_load)
    r = e.solve("uc_full_enumeration", b)
    e.record("uc_global_objective_oracle", abs(r["scuc"]["objective"]-oracle["objective"])/max(1,abs(oracle["objective"]))<1e-6,
             {"oracle": oracle, "actual_objective": r["scuc"]["objective"], "demand": load,
              "source_slots": selected, "scope": "Reordered extrema stress sequence, not chronological replay"})
    for solver in ("native", "gurobi"):
        other = copy.deepcopy(b); other["execution"]["solver"] = solver
        result = e.solve("uc_enumeration_"+solver, other)
        e.record("uc_global_"+solver, abs(result["scuc"]["objective"]-oracle["objective"])/max(1,abs(oracle["objective"]))<1e-6,
                 {"actual_objective": result["scuc"]["objective"], "oracle": oracle["objective"]})


def mixed_case(e, profile):
    b = e.fixture("demo")
    b["execution"].update(ac_security="schedule_only", solver="highs", time_limit_sec=60)
    source = "AEMO NSW regional demand/UIGF shapes and GEN prices; authored demo network/reservoir/SOC; AUD/100 artificial currency"
    extracted = json.loads((ROOT/"output/market-bids/extracted.json").read_text())["20260901"]["records"]
    reference = [extracted[k] for k in ("GEN:TUMUT3", "GEN:MURRAY", "GEN:ER02", "GEN:ER03")]
    for i, g in enumerate(b["generators"]):
        cap = max(g["pmax_mw"])
        g["segments"] = [{"quantity_mw": cap/10, "price_per_mwh": p/100} for p in reference[i%4]["prices_aud_mwh"]]
        g["source"] = source+"; DUID="+reference[i%4]["duid"]
        if g["kind"] in ("wind", "solar"):
            field = "SS_WIND_UIGF" if g["kind"] == "wind" else "SS_SOLAR_UIGF"
            values = profile[field]["quarter_hour_mw"]
            x = [cap*v/max(values) for v in values]
            g["forecast_mw"] = x+[min(x), max(x)]
    load = profile["TOTALDEMAND"]["quarter_hour_mw"]
    shape = [v/(sum(load)/96) for v in load]; shape += [min(shape), max(shape)]
    for table in ("buses", "areas"):
        for row in b[table]:
            row["load_mw"] = [p*f for p, f in zip(row["load_mw"], shape)]
    b["source"] = source
    reservoir = b["reservoirs"][0]
    members = reservoir["generators"]
    if len(members) != 4:
        raise ValueError("Expected four shared hydro members in demo")
    downstream = copy.deepcopy(reservoir)
    reservoir["generators"] = members[:2]
    downstream.update(id=reservoir["id"]+1, generators=members[2:], upstream=reservoir["id"],
                      lag_slots=1, source="Authored one-slot cascade, AEMO does not supply reservoir physics")
    b["reservoirs"].append(downstream)
    return b


def mutations(e, b, r):
    changes = {
        "generation_1mw": lambda x: x["sced"]["generators"][0]["power_mw"].__setitem__(0, x["sced"]["generators"][0]["power_mw"][0]+1),
        "line_1mw": lambda x: x["sced"]["branches"][0]["power_mw"].__setitem__(0, x["sced"]["branches"][0]["power_mw"][0]+1),
        "soc_1mwh": lambda x: x["sced"]["storage"][0]["energy_mwh"].__setitem__(0, x["sced"]["storage"][0]["energy_mwh"][0]+1),
        "reservoir_001m": lambda x: x["sced"]["reservoirs"][0]["level_m"].__setitem__(0, x["sced"]["reservoirs"][0]["level_m"][0]+.01),
        "release_1m3_s": lambda x: x["sced"]["reservoirs"][0]["release_m3_s"].__setitem__(0, x["sced"]["reservoirs"][0]["release_m3_s"][0]+1),
        "missing_unit": lambda x: x["sced"]["generators"].pop(),
        "duplicate_id": lambda x: x["sced"]["generators"][1].update(id=x["sced"]["generators"][0]["id"]),
        "nan_power": lambda x: x["sced"]["generators"][0]["power_mw"].__setitem__(0, float("nan")),
        "truncated_series": lambda x: x["sced"]["generators"][0]["power_mw"].pop(),
        "false_energy_total": lambda x: x["sced"].update(day_generation_mwh=x["sced"]["day_generation_mwh"]+1),
        "false_bid_cost": lambda x: x["sced"].update(day_energy_bid_cost=x["sced"]["day_energy_bid_cost"]+100),
        "false_feasibility": lambda x: x.update(schedule_feasible=False),
    }
    for name, change in changes.items():
        altered = copy.deepcopy(r); change(altered)
        report = audit(b, altered)
        e.record("mutation/"+name, not report["passed"], report["errors"])


def rolling(e, b, profiles):
    e.boundary(b)
    s = e.api("market_operation")
    config = {"horizon": "week", "start_date": "2026-08-28", "penalty_per_mwh": 100000,
              "explain": False, "days": []}
    preview = e.api("market_operation", {"action": "start", "revision": s["revision"], "config": config})
    config = preview["job"]["config"]
    means = [sum(profiles[d]["NSW1"]["TOTALDEMAND"]["quarter_hour_mw"])/96 for d in sorted(profiles)]
    for i, change in enumerate(config["days"]):
        change["load_scale"] = means[min(i, 6)]/means[0]
    # No eighth official day downloaded: explicitly persistence, not hidden source data.
    r = e.api("market_operation", {"action": "start", "revision": s["revision"], "config": config})
    days = []
    for i in range(7):
        r = e.api("market_operation", {"action": "step", "run_id": r["run_id"], "day": i})
        if r["job"]["completed_days"] != i+1:
            raise RuntimeError("Weekly solve failed: "+str(r["job"].get("error")))
        days = r["job"]["days"]
        e.record(f"rolling/day{i}/execution_points", len(days[-1]["periods"]) == 96 and len(days[-1]["lookahead"]["points"]) == 2, {})
        for point in days[-1]["lookahead"]["points"]:
            expected = sum(a["load_mw"][point["source_slot"]] for a in b["areas"])*config["days"][i+1]["load_scale"]
            e.record(f"rolling/day{i}/lookahead{point['target_slot']}", abs(expected-point["load_mw"]) < 1e-5,
                     {"expected_mw": expected, "actual_mw": point["load_mw"]})
        ledger = days[-1]["analysis"]["settlement"]
        periods = ledger.get("periods", [])
        residual = max((abs(p["load_payment_cny"]-p["generation_receipt_cny"]-p["storage_receipt_cny"]-p["diagnostic_slack_receipt_cny"]-p["line_rent_cny"]) for p in periods), default=math.inf)
        e.record(f"rolling/day{i}/cash_identity", len(periods) == 96 and residual < 1e-5, {"max_cash_residual": residual})
        nodes = indexed(days[-1]["nodes"]); generators = indexed(days[-1]["resources"]["generators"])
        recomputed = 0
        for t, money in enumerate(periods):
            load_cash = sum(n["load_mw"][t]*n["lmp_per_mwh"][t]*.25 for n in nodes.values())
            gen_cash = sum(generators[g["id"]]["power_mw"][t]*nodes[g["bus"]]["lmp_per_mwh"][t]*.25 for g in b["generators"])
            rent = sum(l["power_mw"][t]*(nodes[l["to_bus"]]["lmp_per_mwh"][t]-nodes[l["from_bus"]]["lmp_per_mwh"][t])*.25 for l in days[-1]["lines"])
            recomputed = max(recomputed, abs(money["load_payment_cny"]-load_cash), abs(money["generation_receipt_cny"]-gen_cash), abs(money["line_rent_cny"]-rent))
        e.record(f"rolling/day{i}/price_quantity_cash_reconstruction", recomputed < 1e-5, {"max_money_error": recomputed})
    save(OUT/"rolling_week.json", r)
    e.api("market_operation", {"action": "step", "run_id": r["run_id"], "day": 0}, 409)
    e.record("rolling/repeated_day_rejected", True, {})


def realtime(e, b, profiles):
    e.boundary(b)
    s = e.api("southern_realtime"); c = s["config"]
    c["steps"] = 4
    c["boundary"]["execution"].update(solver="highs", ac_security="schedule_only")
    values = profiles["20260901"]["NSW1"]["TOTALDEMAND"]["five_minute_mw"][132:204]
    factor = [x/(sum(values)/len(values)) for x in values]
    for table in ("areas", "buses"):
        for row in c["boundary"][table]:
            row["load_mw"] = [row["load_mw"][0]*v for v in factor]
    c["boundary"]["branches"][0]["available"][6] = 0
    s = e.api("southern_realtime", {"action": "save", "revision": s["revision"], "config": c})
    for i in range(4):
        s = e.api("southern_realtime", {"action": "step", "revision": s["revision"], "run_id": s["run_id"]})
        job = s["job"]
        if job["completed_steps"] != i+1:
            raise RuntimeError("Realtime failed: "+str(job))
        run = job["runs"][-1]; result = run["dispatch"]
        report = audit(result["boundary_snapshot"], result)
        e.record(f"realtime/{i}/physical_equations", report["passed"], report)
        e.record(f"realtime/{i}/grids", len(result["sced"]["generators"][0]["power_mw"]) == 24 and len(result["lmp"]["buses"][0]["lmp_per_mwh"]) == 8 and run["executed_points"] == 3, {})
        if i:
            prev = job["runs"][-2]["dispatch"]["sced"]
            error = 0
            for table, initial, output in (("storage", "initial_mwh", "energy_mwh"), ("reservoirs", "initial_level_m", "level_m"), ("generators", "initial_power_mw", "power_mw")):
                old = indexed(prev[table])
                for row in result["boundary_snapshot"][table]:
                    error = max(error, abs(row[initial]-old[row["id"]][output][2]))
            e.record(f"realtime/{i}/executed_state_only", error < 1e-5, {"max_state_difference": error})
    save(OUT/"realtime_hour.json", s)
    report = audit_completed_hour(s["job"], [n["id"] for n in b["buses"]])
    e.record("realtime/hour_price_after_four_quarters", report["passed"], report)
    e.api("southern_realtime", {"action": "step", "revision": s["revision"], "run_id": s["run_id"]+1}, 409)
    bad = copy.deepcopy(c); bad["boundary"]["generators"][0]["segments"][0]["price_per_mwh"] += 1
    e.api("southern_realtime", {"action": "save", "revision": s["revision"], "config": bad}, 400)
    e.record("realtime/stale_and_rebid_rejected", True, {})


def ancillary(e, b):
    e.boundary(b)
    s = e.api("yunnan_ancillary"); c = s["config"]
    c.update(cmin_mw=2, load_ratio=0, renewable_ratio=0)
    source = json.loads((OUT/"aemo_noon_fcas_bids.json").read_text())
    samples = sorted((float(v["price"][f"PRICEBAND{k}"]), key, k) for key, v in source.items() for k in range(1, 11)
                     if float(v["quantity"][f"BANDAVAIL{k}"]) > 0 and float(v["price"][f"PRICEBAND{k}"]) > 0)
    mapped = []
    for i, g in enumerate(c["agc_units"]):
        price, duid, band = samples[min(len(samples)-1, len(samples)//3+i)]
        value = min(5, max(.1, price/100))
        g["price_per_mw"] = [value]*24
        mapped.append({"agc_id": g["id"], "source_duid_product": duid, "band": band, "original_price": price,
                       "mapped_price": value, "scope": "clipped /100 artificial Yunnan rule input, not FCAS mileage equivalence"})
    s = e.api("yunnan_ancillary", {"action": "run", "revision": s["revision"], "config": c})
    save(OUT/"ancillary_day.json", {"mapping": mapped, "state": s})
    result = s["result"]
    if result["status"] == "ancillary_capacity_shortfall":
        # An energy-only UC can turn off providers. The 50% supplier share cap
        # then makes a single remaining AGC insufficient for any positive demand.
        e.record("ancillary/energy_uc_capacity_shortfall_gate", result["prices_valid"] is False and "sced" not in result,
                 {"status": result["status"], "shortage_hours": [h["hour"] for h in result["ancillary"]["hours"] if h["shortage_mw"] > 0]})
        save(OUT/"ancillary_capacity_shortfall.json", {"mapping": mapped, "state": s})
        fixed = copy.deepcopy(b)
        for g in fixed["generators"]:
            if g["kind"] in ("thermal", "hydro"):
                g["must_on"] = series(1)
        fixed["source"] += "; counterfactual authored must-on AGC providers"
        s = e.boundary(fixed)
        # Keep out-of-range price mapping in the original failure evidence;
        # admissible affine mapping is a separate, explicitly authored experiment.
        for g, m in zip(c["agc_units"], mapped):
            g["price_per_mw"] = [round(min(8, 3+m["original_price"]/100), 1)]*24
        s = e.api("yunnan_ancillary", {"action": "run", "revision": s["revision"], "config": c})
        result, b = s["result"], fixed
        save(OUT/"ancillary_fixed_provider_day.json", s)
        report = audit(b, result); e.record("ancillary/fixed_provider_physics", report["passed"], report)
    else:
        report = audit(b, result); e.record("ancillary/physical_equations", report["passed"], report)
    before, after = indexed(result["scuc"]["generators"]), indexed(result["sced"]["generators"])
    frozen = max(abs(after[i][f][t]-before[i][f][t]) for i in before for f in ("online", "primary_reserve_mw") for t in range(98))
    capacity_error = 0
    for g in c["agc_units"]:
        for t in range(96):
            bid = next(x for x in result["ancillary"]["hours"][t//4]["bids"] if x["id"] == g["id"])
            for field in ("secondary_up_mw", "secondary_down_mw"):
                capacity_error = max(capacity_error, abs(sum(after[m["generator_id"]][field][t] for m in g["members"])-bid["award_mw"]))
    e.record("ancillary/fixed_uc_primary_and_awards", frozen < 1e-5 and capacity_error < 1e-5,
             {"frozen_residual": frozen, "award_residual": capacity_error})
    safe = True
    for g in c["agc_units"]:
        for m in g["members"]:
            row = after[m["generator_id"]]
            for t in range(98):
                if m["safe_intervals_mw"] and row["stable"][t] > .5:
                    lo = row["power_mw"][t]-row["secondary_down_mw"][t]
                    hi = row["power_mw"][t]+row["secondary_up_mw"][t]+row["primary_reserve_mw"][t]
                    safe = safe and any(a-1e-5 <= lo and hi <= z+1e-5 for a, z in m["safe_intervals_mw"])
    e.record("ancillary/hydro_connected_safe_band", safe, {})
    c["workflow"]["stage"] = "intraday"
    intraday = e.api("yunnan_ancillary", {"action": "intraday", "revision": s["revision"], "day_ahead_id": s["day_ahead_id"], "config": c})
    save(OUT/"ancillary_intraday.json", intraday)
    e.record("ancillary/intraday_commitment_frozen", intraday["result"]["commitment_solution"] == result["commitment_solution"], {})
    bad = copy.deepcopy(c); bad["agc_units"][0]["price_per_mw"][0] += .1
    e.api("yunnan_ancillary", {"action": "intraday", "revision": s["revision"], "day_ahead_id": s["day_ahead_id"], "config": bad}, 400)
    e.record("ancillary/sealed_quote_rejected", True, {})


def scenarios(e, b, profiles, mixed):
    e.boundary(b)
    s = e.api("market_forecast"); c = s["defaults"]
    c.update(sample_count=2, seed=20260901, temporal_rho=0)
    c["operation"].update(start_date="2026-08-28", explain=False)
    means = [sum(profiles[d]["NSW1"]["TOTALDEMAND"]["quarter_hour_mw"])/96 for d in sorted(profiles)]
    factors = [v/means[0] for v in means]
    for m in c["marginals"]:
        m.update(distribution="fixed", lower=1, upper=1, center=[1]*8)
    c["marginals"][0].update(distribution="uniform", lower=min(factors), upper=max(factors), center=[sum(factors)/7]*8)
    r = e.api("market_forecast", {"action": "generate", "revision": s["revision"], "config": c})
    original = [x["config"]["days"] for x in r["job"]["scenarios"]]
    repeat = e.api("market_forecast", {"action": "generate", "revision": s["revision"], "config": c})
    e.record("forecast/seed_reproducibility", original == [x["config"]["days"] for x in repeat["job"]["scenarios"]], {})
    r = repeat
    for scenario in range(2):
        for day in range(7):
            r = e.api("market_forecast", {"action": "step", "run_id": r["run_id"], "scenario": scenario, "day": day})
    job = r["job"]
    stats = job["statistics"]
    expected = sum(sum(d["deficit_mwh"] for d in x["days"]) for x in job["scenarios"])/2
    e.record("forecast/independent_sample_mean", stats["complete_scenarios"] == 2 and abs(stats["week_deficit_mwh"]["mean"]-expected)<1e-5,
             {"expected_deficit_mean": expected, "actual": stats["week_deficit_mwh"]["mean"],
              "scope": "Seven observed daily load means set a research uniform range; not calibrated forecast errors"})
    save(OUT/"forecast_week.json", r)
    e.boundary(mixed)
    s = e.api("market_study"); config = s["defaults"]
    config["operation"].update(start_date="2026-09-01", explain=False, posthoc_ac_audit=True)
    config["inflow_scales"] = [0, 1]
    normal = config["faults"][0]
    fault = copy.deepcopy(normal); fault.update(name="Authored branch outage", branch_outages=[mixed["branches"][0]["id"]])
    config["faults"].append(fault)
    r = e.api("market_study", {"action": "generate", "revision": s["revision"], "config": config})
    for scenario in range(4):
        r = e.api("market_study", {"action": "step", "run_id": r["run_id"], "scenario": scenario, "day": 0})
    save(OUT/"fault_inflow_study.json", r)
    e.record("study/factorial_coverage", len(r["job"]["scenarios"]) == 4 and r["job"]["statistics"]["complete_scenarios"] == 4, {})
    for i, scenario in enumerate(r["job"]["scenarios"]):
        day = scenario["days"][0]
        analysis = day["analysis"]
        e.record(f"study/{i}/ac_and_settlement_scope", analysis["settlement"]["formal_settlement_eligible"] is False and
                 len(analysis["ac_audit"]["periods"]) == 98,
                 {"ac_audit": analysis["ac_audit"].get("status"), "cash_status": analysis["settlement"]["status"]})
        delta = sum(sum(l["overload_mw"]) for l in day["lines"])*.25
        e.record(f"study/{i}/delta_pij_integral", abs(delta-day["overload_mwh"]) < 1e-5, {"integral_mwh": delta})


def main():
    profiles = json.loads((OUT/"aemo_profiles.json").read_text())
    official = json.loads((OUT/"official_data_audit.json").read_text())
    if not official["identities_pass"]:
        raise SystemExit("Official identities need investigation before transfer")
    binary = ROOT/"build/macos-release/run_gui_server"
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0)); port = sock.getsockname()[1]
    with (OUT/"campaign_server.log").open("w") as log:
        server = subprocess.Popen([str(binary), "--host", "127.0.0.1", "--port", str(port)], cwd=ROOT, stdout=log, stderr=log)
        e = Experiment(f"http://127.0.0.1:{port}")
        try:
            for _ in range(150):
                try:
                    state = e.api("southern_market"); break
                except OSError:
                    time.sleep(.1)
            seed = e.fixture()
            records = json.loads((ROOT/"output/market-bids/extracted.json").read_text())["20260901"]["records"]
            quotes = sorted((float(v["daily_record"][f"PRICEBAND{k}"]), v["duid"], k) for v in records.values() if v["direction"] == "GEN" for k in range(1, 11)
                            if float(v["period_record"][f"BANDAVAIL{k}"]) > 0 and 10 < float(v["daily_record"][f"PRICEBAND{k}"]) < 1000)
            low, high = quotes[len(quotes)//4], quotes[3*len(quotes)//4]
            p1, p2 = low[0]/100, high[0]/100
            if not p2 > p1:
                raise ValueError("Need separated empirical price samples")
            save(OUT/"campaign_manifest.json", {"binary": str(binary), "sha256": sha256(binary),
                 "quote_samples": [low, high], "price_scale": .01, "data_summary": official,
                 "python": sys.version, "platform": platform.platform(),
                 "source_manifest_sha256": sha256(ROOT/"external_data/market_validation/source_manifest.json"),
                 "harness_sha256": {str(p.relative_to(ROOT)): sha256(p) for p in Path(__file__).parent.glob("*.py")},
                 "build_scope": "Existing macos-release binary, not rebuilt; dependency worktree/pin guard unresolved",
                 "repository_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
                 "repository_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()})
            profile = profiles["20260901"]["NSW1"]
            b = network_case(seed, state["schema"], profile, p1, p2)
            run_network(e, b, p1, p2)
            run_uc(e, seed, p1, p2, profile)
            mixed = mixed_case(e, profile)
            r = e.solve("mixed_resources_aemo_shapes", mixed)
            mutations(e, mixed, r)
            dry = copy.deepcopy(mixed)
            for h in dry["reservoirs"]:
                h["inflow_m3_s"] = [0]*98
            e.solve("mixed_dry_inflow", dry)
            rolling(e, b, profiles)
            realtime(e, mixed, profiles)
            ancillary(e, mixed)
            scenarios(e, b, profiles, mixed)
        finally:
            save(OUT/"campaign_runs.json", e.runs)
            server.terminate()
            try:
                server.wait(timeout=10)
            except subprocess.TimeoutExpired:
                server.kill(); server.wait()
    if not all(c["passed"] for c in e.checks):
        raise SystemExit("Adversarial checks found discrepancies; see campaign_checks.json")
    print(f"Passed {len(e.checks)} checks", flush=True)


if __name__ == "__main__":
    main()
