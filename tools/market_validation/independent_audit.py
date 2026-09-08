"""Independent output equations and mutation detectors, not solver self-reports.

Coverage/tolerances: docs/modules/market/aemo_validation.md, experiments 3-8.
"""
import itertools
import math


def indexed(rows):
    result = {}
    for row in rows:
        if row["id"] in result:
            raise ValueError("Duplicate stable ID")
        result[row["id"]] = row
    return result


def audit_completed_hour(job, bus_ids, hour=0):
    """Four executed quarter prices, not the seven forecast prices per window."""
    try:
        prices = {i: {} for i in bus_ids}
        for run in job["runs"]:
            minute = run["start_minute"]
            if minute//60 != hour or run["executed_points"] != 3:
                continue
            if minute % 15 or not run["dispatch"]["prices_valid"]:
                raise ValueError("Invalid executed price interval")
            for i, row in indexed(run["dispatch"]["lmp"]["buses"]).items():
                if i not in prices or minute in prices[i]:
                    raise ValueError("Duplicate or unexpected executed price")
                price = row["lmp_per_mwh"][0]
                if not math.isfinite(price):
                    raise ValueError("Nonfinite executed price")
                prices[i][minute] = price
        reported = indexed([{**p, "id": p["bus_id"]} for p in job["hourly_prices"] if p["hour"] == hour])
        if not prices or reported.keys() != prices.keys():
            raise ValueError("Missing or unexpected hourly node prices")
        residual = 0
        for i, values in prices.items():
            if set(values) != {hour*60+q*15 for q in range(4)} or reported[i]["observed_quarters"] != 4:
                raise ValueError("Incomplete four-quarter hour")
            value = reported[i]["price_per_mwh"]
            if value is None or not math.isfinite(value):
                raise ValueError("Missing hourly price")
            residual = max(residual, abs(value-sum(values.values())/4))
        return {"passed": residual <= 1e-6, "max_price_error": residual, "nodes": len(prices)}
    except (ValueError, KeyError, TypeError, IndexError) as error:
        return {"passed": False, "error": str(error)}


def audit(boundary, result):
    errors, maxima = [], {}

    def check(name, error, tolerance=1e-5):
        if not math.isfinite(error):
            errors.append({"check": name, "error": "nonfinite"})
            return
        maxima[name] = max(maxima.get(name, 0), abs(error))
        if abs(error) > tolerance and len(errors) < 50:
            errors.append({"check": name, "error": error, "tolerance": tolerance})

    try:
        if result.get("schedule_feasible") is not True:
            raise ValueError("No admissible schedule")
        b = result.get("effective_boundary", boundary)
        s = result["sced"]
        if b["dc_links"] or b["dc_hubs"] or b["external_schedules"] or b["trades"]:
            raise ValueError("AC audit cannot silently omit DC/external/trade components")
        n = len(b["periods"])
        inputs, outputs = {}, {}
        fields = {"generators": ["power_mw", "online", "primary_reserve_mw"],
                  "buses": ["deficit_mw", "surplus_mw"], "branches": ["power_mw", "slack_plus_mw", "slack_minus_mw"],
                  "storage": ["discharge_mw", "charge_mw", "energy_mwh"],
                  "reservoirs": ["level_m", "release_m3_s", "spill_m3_s"], "controllable_loads": ["reduction_mw"]}
        for table, names in fields.items():
            inputs[table], outputs[table] = indexed(b[table]), indexed(s[table])
            if inputs[table].keys() != outputs[table].keys():
                raise ValueError("Missing or extra result IDs: " + table)
            for row in outputs[table].values():
                for field in names:
                    if len(row[field]) != n or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in row[field]):
                        raise ValueError("Nonfinite or incomplete series: " + table + "." + field)
        for t in range(n):
            duration = b["periods"][t]["duration_hr"]
            balance = {i: -row["load_mw"][t] for i, row in inputs["buses"].items()}
            for i, g in inputs["generators"].items():
                row = outputs["generators"][i]
                p, u = row["power_mw"][t], row["online"][t]
                check("commitment_integrality", u-round(u))
                cap = g["pmax_mw"][t]*g["available"][t]*(1-g["must_off"][t])
                if g["kind"] in ("wind", "solar", "renewable"):
                    cap = min(cap, g["forecast_mw"][t])
                check("generator_capacity", max(-p, p-cap, p-g["pmax_mw"][t]*u, 0))
                balance[g["bus"]] += p
            for i, g in inputs["controllable_loads"].items():
                p = outputs["controllable_loads"][i]["reduction_mw"][t]
                check("load_reduction_bound", max(-p, p-g["max_reduction_mw"][t]*g["available"][t], 0))
                balance[g["bus"]] += p
            for i, g in inputs["storage"].items():
                row = outputs["storage"][i]
                dis, ch, energy = (row[f][t] for f in ("discharge_mw", "charge_mw", "energy_mwh"))
                eta = math.sqrt(g["roundtrip_efficiency"])
                previous = g["initial_mwh"] if t == 0 else row["energy_mwh"][t-1]
                check("storage_soc_mwh", energy-previous+duration*(dis/eta+ch*eta))
                check("storage_energy_bounds", max(g["min_mwh"][t]-energy, energy-g["max_mwh"][t], 0))
                check("storage_power_signs_bounds", max(-dis, ch, dis-g["discharge_max_mw"]*g["available"][t], -ch-g["charge_max_mw"]*g["available"][t], 0))
                check("storage_exclusivity", min(abs(ch), abs(dis)))
                balance[g["bus"]] += dis+ch
            for i, g in inputs["branches"].items():
                row = outputs["branches"][i]
                p = row["power_mw"][t]
                if not g["available"][t]:
                    check("outage_flow", p)
                check("line_slack_plus", row["slack_plus_mw"][t]-max(0, p-g["max_mw"][t]*g["available"][t]))
                check("line_slack_minus", row["slack_minus_mw"][t]-max(0, g["min_mw"][t]*g["available"][t]-p))
                balance[g["from_bus"]] -= p
                balance[g["to_bus"]] += p
            for i, row in outputs["buses"].items():
                check("slack_nonnegative", max(-row["deficit_mw"][t], -row["surplus_mw"][t], 0))
                check("nodal_balance_mw", balance[i]+row["deficit_mw"][t]-row["surplus_mw"][t])
            for i, g in inputs["reservoirs"].items():
                row = outputs["reservoirs"][i]
                members = g.get("generators", [g.get("generator")])
                power = sum(outputs["generators"][k]["power_mw"][t] for k in members)
                check("shared_water_release_m3_s", row["release_m3_s"][t]-row["spill_m3_s"][t]-power*g["water_m3_mwh"]/3600)
                parent, upstream = g["upstream"], 0
                if parent != -1:
                    offset = t-g["lag_slots"]
                    upstream = (inputs["reservoirs"][parent]["release_history_m3_s"][offset]
                                if offset < 0 else outputs["reservoirs"][parent]["release_m3_s"][offset])
                previous = g["initial_level_m"] if t == 0 else row["level_m"][t-1]
                expected = previous+3600*duration*(g["inflow_m3_s"][t]+upstream-row["release_m3_s"][t])/g["area_m2"]
                check("reservoir_level_m", row["level_m"][t]-expected, 1e-7)
                check("reservoir_level_bounds", max(g["min_level_m"][t]-row["level_m"][t], row["level_m"][t]-g["max_level_m"][t], 0), 1e-7)
        if n == 98:
            # Realized energy is 96 points; two outlook points must not be billed.
            energy = sum(g["power_mw"][t]*b["periods"][t]["duration_hr"] for g in outputs["generators"].values() for t in range(96))
            check("day_generation_energy_mwh", s["day_generation_mwh"]-energy)
            bid_cost = 0
            for i, g in inputs["generators"].items():
                for t in range(96):
                    row = outputs["generators"][i]
                    if g["pmin_mw"][t] != 0 or any(g["startup_curves_mw"]) or g["shutdown_curve_mw"]:
                        raise ValueError("Independent bid-integral audit requires zero minimum and no startup/shutdown trajectory")
                    remaining = row["power_mw"][t]
                    for segment in g["segments"]:
                        used = min(max(0, remaining), segment["quantity_mw"])
                        bid_cost += used*segment["price_per_mwh"]*b["periods"][t]["weight_hr"]
                        remaining -= used
            check("day_bid_integral_relative_error", (s["day_energy_bid_cost"]-bid_cost)/max(1, abs(bid_cost)), 1e-6)
            for i, g in inputs["storage"].items():
                check("storage_day_terminal", outputs["storage"][i]["energy_mwh"][95]-g["terminal_mwh"])
            for i, g in inputs["controllable_loads"].items():
                energy = sum(outputs["controllable_loads"][i]["reduction_mw"][t]*b["periods"][t]["duration_hr"] for t in range(96))
                check("demand_day_energy_cap", max(0, energy-g["max_day_reduction_mwh"]))
    except (ValueError, KeyError, TypeError, IndexError) as error:
        errors.append({"check": "structure_or_scope", "error": str(error)})
    return {"passed": not errors, "maxima": maxima, "errors": errors}


def enumerate_uc(demand, caps, prices, starts, minimum_up, no_load=(0, 0)):
    """Complete finite action oracle: 2 units x 4 decisions, fixed-off tail."""
    best, best_bits, feasible = math.inf, None, 0
    for bits in itertools.product((0, 1), repeat=2*len(demand)):
        on = [bits[2*t:2*t+2] for t in range(len(demand))]
        valid, cost = True, 0
        for t, load in enumerate(demand):
            if sum(caps[g]*on[t][g] for g in range(2)) < load-1e-9:
                valid = False
                break
            for g in range(2):
                cost += .25*no_load[g]*on[t][g]
                if on[t][g] and (t == 0 or not on[t-1][g]):
                    if t+minimum_up[g] > len(demand) or any(not on[k][g] for k in range(t, t+minimum_up[g])):
                        valid = False
                    cost += starts[g]
            left = load
            for g in sorted(range(2), key=lambda g: prices[g]):
                p = min(left, caps[g]*on[t][g])
                left -= p
                cost += .25*p*prices[g]
        if valid:
            feasible += 1
            if cost < best:
                best, best_bits = cost, bits
    if best_bits is None:
        raise ValueError("No feasible commitment in exhaustive oracle")
    return {"objective": best, "commitment": best_bits, "feasible_combinations": feasible, "enumerated": 2**(2*len(demand))}
