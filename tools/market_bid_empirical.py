#!/usr/bin/env python3
"""Reproduce docs/modules/market/empirical_bidding_analysis.md with official bids.

Raw quotes are AUD/MWh. Solver inputs use artificial currency = AUD / 100;
schema CNY labels do not imply an exchange rate or Southern price calibration.
"""

import argparse
import copy
import csv
import datetime as dt
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import socket
import statistics
import subprocess
import sys
import time
import urllib.request
import urllib.error
import zipfile
from collections import Counter


ROOT = Path(__file__).resolve().parents[1]
FILES = [
    "20260828_0000000534982753", "20260829_0000000535142632",
    "20260830_0000000535298928", "20260831_0000000535463922",
    "20260901_0000000535637491", "20260902_0000000535804688",
    "20260903_0000000535974210",
]
URL = "https://nemweb.com.au/Reports/CURRENT/Bidmove_Complete/"


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def quantile(values, p):
    a = sorted(values)
    if not a:
        return None
    pos = (len(a) - 1) * p
    i = int(pos)
    return a[i] + (a[min(i + 1, len(a) - 1)] - a[i]) * (pos - i)


def curve(prices, quantities, maxavail, direction="GEN"):
    """Protocol integral: clip availability in price order, keep signed prices."""
    if any(not math.isfinite(x) for x in prices + quantities + [maxavail]):
        raise ValueError("Nonfinite offer")
    if any(q < 0 for q in quantities) or maxavail < 0:
        raise ValueError("Negative availability")
    if any(b < a for a, b in zip(prices, prices[1:])):
        raise ValueError("Unordered bid prices")
    capacity = min(maxavail, sum(quantities))
    if capacity <= 0:
        return None
    if direction not in ("GEN", "LOAD"):
        raise ValueError("Unsupported energy direction")
    kept = [0.0] * len(quantities)
    remaining = capacity
    order = range(len(quantities)) if direction == "GEN" else reversed(range(len(quantities)))
    for k in order:
        kept[k] = min(quantities[k], remaining)
        remaining -= kept[k]
    steps = []
    used = 0.0
    for p, q in zip(prices, kept):
        q = min(q, capacity - used)
        if q > 0:
            steps.append((used, used + q, p))
            used += q
    means = []
    for k in range(10):
        lo, hi = capacity * k / 10, capacity * (k + 1) / 10
        means.append(sum(max(0, min(hi, b) - max(lo, a)) * p
                         for a, b, p in steps) / (hi - lo))
    # Ordered-curve integration is monotone analytically. Bound roundoff by
    # 64 machine epsilons (at most 10 overlaps); do not repair real reversals.
    for k in range(1, len(means)):
        if means[k] < means[k-1]:
            if means[k-1]-means[k] > 64*sys.float_info.epsilon*max(1, abs(means[k-1])):
                raise ValueError("Quantile integration broke price monotonicity")
            means[k] = means[k-1]
    residual = abs(sum(means) * capacity / 10 - sum((b-a)*p for a, b, p in steps))
    if residual > 1e-8 * max(1, sum(abs((b-a)*p) for a, b, p in steps)):
        raise ValueError("Offer integral changed")
    error = sum(max(0, min(capacity*(k+1)/10, b)-max(capacity*k/10, a)) * abs(p-m)
                for k, m in enumerate(means) for a, b, p in steps) / capacity
    return {"capacity_mw": capacity, "prices_aud_mwh": means,
            "mapping_mae_aud_mwh": error,
            "negative_capacity_fraction": sum(b-a for a, b, p in steps if p < 0)/capacity,
            "tail_1000_capacity_fraction": sum(b-a for a, b, p in steps if p > 1000)/capacity}


def parse_archive(path):
    headers, daily, periods = {}, {}, {}
    counts = Counter()
    with zipfile.ZipFile(path) as archive:
        members = archive.namelist()
        if len(members) != 1 or not members[0].upper().endswith(".CSV"):
            raise ValueError("Expected exactly one MMS CSV")
        with archive.open(members[0]) as raw:
            for row in csv.reader(io.TextIOWrapper(raw, encoding="utf-8-sig")):
                if row[0] == "I":
                    headers[row[2]] = row[4:]
                    continue
                if row[0] != "D":
                    continue
                table = row[2]
                fields = headers[table]
                if len(fields) != len(row) - 4:
                    raise ValueError("MMS column mismatch")
                counts[f"{table}:all"] += 1
                # Restrict before constructing dictionaries for the large FCAS tables.
                if row[4 + fields.index("BIDTYPE")] != "ENERGY":
                    continue
                d = dict(zip(fields, row[4:]))
                counts[f"{table}:ENERGY:{d['DIRECTION']}"] += 1
                key = (d["SETTLEMENTDATE"], d["DUID"], d["DIRECTION"])
                if table == "BIDDAYOFFER_D":
                    if key in daily:
                        raise ValueError("Multiple daily versions: snapshot protocol cannot select silently")
                    daily[key] = d
                elif table == "BIDPEROFFER_D" and d["PERIODID"] == "96":
                    if key in periods:
                        raise ValueError("Multiple period versions: need explicit as-of reconstruction")
                    periods[key] = d
    records, unavailable = {}, {}
    for key, p in sorted(periods.items()):
        if key not in daily:
            raise ValueError(f"No matching daily price: {key}")
        d = daily[key]
        # AEMO Elec10, BIDDAYOFFER_D primary key: applicable SETTLEMENTDATE.
        # BIDSETTLEMENTDATE is the submitted date and may differ across carried bids.
        if p["BIDSETTLEMENTDATE"] != d["BIDSETTLEMENTDATE"]:
            counts["different_submitted_dates"] += 1
        expected_interval = key[0][:10] + " 12:00:00"
        if p["INTERVAL_DATETIME"] != expected_interval:
            raise ValueError("Unexpected interval clock")
        if p["OFFERDATE"] > p["INTERVAL_DATETIME"] or d["OFFERDATE"] > p["INTERVAL_DATETIME"]:
            raise ValueError("Bid timestamp after target interval")
        if p["DIRECTION"] not in ("GEN", "LOAD"):
            counts["excluded_direction"] += 1
            continue
        x = curve([float(d[f"PRICEBAND{k}"]) for k in range(1, 11)],
                  [float(p[f"BANDAVAIL{k}"]) for k in range(1, 11)], float(p["MAXAVAIL"]), p["DIRECTION"])
        if x is None:
            counts[f"zero_capacity:{p['DIRECTION']}"] += 1
            unavailable[p["DIRECTION"] + ":" + p["DUID"]] = {"daily_record": d, "period_record": p}
            continue
        records[p["DIRECTION"] + ":" + p["DUID"]] = {
            **x, "duid": p["DUID"], "direction": p["DIRECTION"],
            "participant": d["PARTICIPANTID"], "daily_record": d, "period_record": p}
    return {"records": records, "zero_capacity_records": unavailable, "counts": dict(counts), "headers": headers}


def positive_factor(x, y):
    # Constrained one-parameter least squares; derivation in the experiment protocol.
    denom = sum(a*a for a in x)
    if denom == 0:
        raise ValueError("Unidentifiable common factor: all baseline bids are zero")
    return max(0, sum(a*b for a, b in zip(x, y)) / denom)


def score(actual, predicted):
    if len(actual) != len(predicted) or not actual:
        raise ValueError("Unpaired or empty scores")
    return {"mae_aud_mwh": statistics.mean(abs(a-b) for a, b in zip(actual, predicted)),
            "rmse_aud_mwh": math.sqrt(statistics.mean((a-b)**2 for a, b in zip(actual, predicted)))}


def empirical(data):
    days = sorted(data)
    train, test = days[:4], days[4:]
    anchor = data[train[-1]]["records"]
    summary = {"train_days": train, "test_days": test, "directions": {}, "daily_counts": {}}
    for day, parsed in data.items():
        summary["daily_counts"][day] = parsed["counts"]
    for direction in ("GEN", "LOAD"):
        keys = sorted(k for k in anchor if k.startswith(direction + ":") and
                      all(k in data[d]["records"] for d in train))
        base = [x for k in keys for x in anchor[k]["prices_aud_mwh"]]
        coefficients = [positive_factor(base, [x for k in keys for x in data[d]["records"][k]["prices_aud_mwh"]]) for d in train]
        fitted = statistics.mean(coefficients)
        daily_scores = []
        for day in test:
            paired = [k for k in keys if k in data[day]["records"]]
            x = [p for k in paired for p in anchor[k]["prices_aud_mwh"]]
            y = [p for k in paired for p in data[day]["records"][k]["prices_aud_mwh"]]
            oracle = positive_factor(x, y)
            daily_scores.append({"day": day, "paired_units": len(paired), "missing_units": len(keys)-len(paired),
                "common_factor_expost": oracle, "persistence": score(y, x),
                "common_train": score(y, [fitted*p for p in x]),
                "common_expost": score(y, [oracle*p for p in x]),
                "changed_curve_fraction": statistics.mean(any(abs(a-b)>1e-7 for a, b in zip(anchor[k]["prices_aud_mwh"], data[day]["records"][k]["prices_aud_mwh"])) for k in paired)})
        all_records = [v for d in days for v in data[d]["records"].values() if v["direction"] == direction]
        raw_active_prices = [float(v["daily_record"][f"PRICEBAND{k}"]) for v in all_records for k in range(1, 11)
                             if float(v["period_record"][f"BANDAVAIL{k}"]) > 0]
        summary["directions"][direction] = {"training_units": len(keys), "unit_days": len(all_records),
            "duids": len({v["duid"] for v in all_records}), "common_factor_train": fitted,
            "raw_positive_quantity_band_price_quantiles_aud_mwh": {str(p): quantile(raw_active_prices, p) for p in (0, .05, .5, .95, 1)},
            "equal_unit_day_negative_capacity_fraction": statistics.mean(v["negative_capacity_fraction"] for v in all_records),
            "equal_unit_day_tail_1000_capacity_fraction": statistics.mean(v["tail_1000_capacity_fraction"] for v in all_records),
            "mapping_mae_aud_mwh": statistics.mean(v["mapping_mae_aud_mwh"] for v in all_records),
            "daily_scores": daily_scores}
    eligible = [k for k in anchor if k.startswith("GEN:") and all(k in data[d]["records"] for d in train)]
    selected = sorted(eligible, key=lambda k: (-statistics.mean(data[d]["records"][k]["capacity_mw"] for d in train), k))[:20]
    if len(selected) != 20:
        raise ValueError("Need 20 training units")
    summary["selected_generation_duids"] = selected
    return summary


def merit(curves, demand):
    offers = sorted((p/100, i, k) for i, row in enumerate(curves) for k, p in enumerate(row))
    if not 0 < demand < len(offers)*10:
        raise ValueError("Demand must lie strictly inside available capacity")
    remaining, hourly_cost = demand, 0.0
    for p, _, _ in offers:
        q = min(10, remaining)
        hourly_cost += p*q
        remaining -= q
    # The subgradient at a step boundary is an interval, not a unique price.
    left = offers[math.ceil(demand/10)-1][0]
    right = offers[math.floor(demand/10)][0]
    return {"day_cost": hourly_cost*24, "price_interval": [left, right]}


def api(base, route, body=None):
    request = urllib.request.Request(base + route, data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=180) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"{route}: HTTP {error.code}: {error.read().decode()}") from error


def simulations(data, summary, out, server_path, solver):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    base = f"http://127.0.0.1:{port}"
    rows = []
    with (out/"server.log").open("w") as log:
        server = subprocess.Popen([str(server_path), "--host", "127.0.0.1", "--port", str(port)], cwd=ROOT, stdout=log, stderr=log)
        try:
            for _ in range(150):
                if server.poll() is not None:
                    raise RuntimeError("Experiment server exited")
                try:
                    state = api(base, "/api/session/southern_market")
                    break
                except OSError:
                    time.sleep(.1)
            else:
                raise RuntimeError("Experiment server did not start")
            api(base, "/api/session/southern_market", {"action": "example", "revision": state["revision"]})
            state = api(base, "/api/session/southern_market")
            template = state["boundary"]
            anchor = data[summary["train_days"][-1]]["records"]
            cohort = summary["selected_generation_duids"]
            common = summary["directions"]["GEN"]["common_factor_train"]
            for day in summary["test_days"]:
                selected = [k for k in cohort if k in data[day]["records"]]
                unavailable = [k for k in cohort if k not in data[day]["records"]]
                if not selected or any(k not in data[day]["zero_capacity_records"] for k in unavailable):
                    raise ValueError("Missing selected unit without a documented zero-capacity record")
                actual = [data[day]["records"][k]["prices_aud_mwh"] for k in selected]
                frozen = [anchor[k]["prices_aud_mwh"] for k in selected]
                for method, curves in (("actual_snapshot", actual), ("unit_persistence", frozen),
                                       ("common_train", [[p*common for p in row] for row in frozen])):
                    for load_fraction in (.35, .65, .9):
                        demand = 100 * len(selected) * load_fraction
                        boundary = copy.deepcopy(template)
                        boundary["name"] = f"AEMO controlled transfer {day} {method} {load_fraction}"
                        boundary["source"] = "Empirical curve shapes; 100 MW/unit; prices AUD/100 artificial currency, NOT CNY calibration; no source physical limits"
                        boundary["execution"].update(solver=solver, ac_security="schedule_only", time_limit_sec=60, mip_gap=.01)
                        boundary["areas"][0]["load_mw"] = [demand]*98
                        boundary["buses"][0]["load_mw"] = [demand]*98
                        generator = copy.deepcopy(template["generators"][0])
                        boundary["generators"] = []
                        for i, (key, prices) in enumerate(zip(selected, curves)):
                            g = copy.deepcopy(generator)
                            g.update(id=i+1, name=key, source=boundary["source"], must_on=[1]*98,
                                     pmax_mw=[100]*98, initial_power_mw=demand/len(selected),
                                     segments=[{"quantity_mw": 10, "price_per_mwh": p/100} for p in prices])
                            boundary["generators"].append(g)
                        state = api(base, "/api/session/southern_market")
                        save(out/"last_attempted_boundary.json", boundary)
                        api(base, "/api/session/southern_market", {"action": "save", "revision": state["revision"], "boundary": boundary})
                        state = api(base, "/api/session/southern_market")
                        start = time.monotonic()
                        result = api(base, "/api/session/run_southern_market", {"revision": state["revision"]})
                        elapsed = time.monotonic() - start
                        tag = f"{day}-{method}-{int(load_fraction*100)}"
                        save(out/"runs"/(tag+".json"), {"input": boundary, "result": result})
                        oracle = merit(curves, demand)
                        row = {"day": day, "method": method, "load_fraction": load_fraction,
                               "active_units": len(selected), "zero_capacity_units": unavailable,
                               "wall_sec": elapsed, "status": result.get("status"), "oracle": oracle}
                        try:
                            dispatch = result["sced"]
                            row["day_cost"] = dispatch["day_energy_bid_cost"]
                            row["cost_relative_error"] = abs(row["day_cost"]-oracle["day_cost"])/max(1, abs(oracle["day_cost"]))
                            row["power_residual_mw"] = max(abs(sum(g["power_mw"][t] for g in dispatch["generators"])-demand) for t in range(98))
                            prices = result["lmp"]["buses"][0]["lmp_per_mwh"]
                            row["lmp"] = prices[0]
                            lo, hi = oracle["price_interval"]
                            row["lmp_interval_error"] = max(max(lo-p, p-hi, 0) for p in prices)
                            row["solver"] = dispatch.get("solver")
                            row["valid"] = (result.get("schedule_feasible") is True and result.get("prices_valid") is True and
                                            row["cost_relative_error"] <= 1e-6 and row["power_residual_mw"] <= 1e-5 and row["lmp_interval_error"] <= 1e-5)
                        except (KeyError, TypeError) as error:
                            row.update(valid=False, error=str(error))
                        rows.append(row)
                        save(out/"clearing_summary.json", rows)
                        print(tag, row, flush=True)
        finally:
            server.terminate()
            try:
                server.wait(timeout=10)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true", help="Download missing official archives")
    parser.add_argument("--parse-only", action="store_true")
    parser.add_argument("--solver", choices=["highs", "native", "gurobi"], default="highs")
    parser.add_argument("--server", type=Path, default=ROOT/"build/macos-release/run_gui_server")
    parser.add_argument("--output", type=Path, default=ROOT/"output/market-bids")
    args = parser.parse_args()
    rawdir, out = ROOT/"external_data/market_bids", args.output
    rawdir.mkdir(parents=True, exist_ok=True)
    out.mkdir(parents=True, exist_ok=True)
    data, files = {}, []
    for name in FILES:
        filename = "PUBLIC_BIDMOVE_COMPLETE_" + name + ".zip"
        path = rawdir/filename
        if not path.exists() and args.download:
            urllib.request.urlretrieve(URL + filename, path)
        digest = sha256(path)
        day = name[:8]
        print("Parsing", day, flush=True)
        data[day] = parse_archive(path)
        files.append({"url": URL + filename, "file": str(path.relative_to(ROOT)), "sha256": digest,
                      "bytes": path.stat().st_size, "local_mtime_utc": dt.datetime.fromtimestamp(path.stat().st_mtime, dt.timezone.utc).isoformat()})
    save(out/"extracted.json", data)
    manifest = {"processed_at_utc": dt.datetime.now(dt.timezone.utc).isoformat(), "files": files,
                "python": platform.python_version(), "platform": platform.platform(),
                "repository_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                "worktree_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
                "scope": "Final noon bid snapshots; chronological descriptive holdout, NOT real-time-available forecast backtest"}
    if not args.parse_only:
        manifest["server_sha256"] = sha256(args.server)
        manifest["solver_requested"] = args.solver
    save(out/"manifest.json", manifest)
    summary = empirical(data)
    save(out/"empirical_summary.json", summary)
    if not args.parse_only:
        rows = simulations(data, summary, out, args.server, args.solver)
        if len(rows) != 27 or not all(row["valid"] for row in rows):
            raise SystemExit("Clearing verification failed; retained all evidence")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
