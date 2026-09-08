"""Official MMS sources and independent identities; see aemo_validation.md."""
import argparse
import csv
import datetime as dt
import io
import json
from pathlib import Path
import sys
import urllib.request
import zipfile
from collections import Counter

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from market_bid_empirical import ROOT, FILES, save, sha256

RAW = ROOT / "external_data/market_validation"
OUT = ROOT / "output/market-validation"


def sources(download=False):
    base = "https://nemweb.com.au/Reports/"
    dictionary = base+"CURRENT/MMSDataModelReport/Electricity/Electricity%20Data%20Model%20Report_files/"
    items = [(RAW/f"PUBLIC_DISPATCHIS_{name[:8]}.zip",
              base+f"ARCHIVE/DispatchIS_Reports/PUBLIC_DISPATCHIS_{name[:8]}.zip") for name in FILES]
    unit = "PUBLIC_NEXT_DAY_DISPATCH_20260901_0000000535633836.zip"
    items += [(RAW/unit, base+"CURRENT/Next_Day_Dispatch/"+unit)]
    items += [(RAW/f"Elec{n}.html", dictionary+f"Elec{n}.htm") for n in ("20", "20_1", "22")]
    items += [(ROOT/"external_data/market_bids/aemo_bid_data_model.html", dictionary+"Elec10.htm")]
    items += [(ROOT/"external_data/market_bids"/f"PUBLIC_BIDMOVE_COMPLETE_{name}.zip",
               base+f"CURRENT/Bidmove_Complete/PUBLIC_BIDMOVE_COMPLETE_{name}.zip") for name in FILES]
    pin_path = RAW/"source_manifest.json"
    pins = {r["file"]: r["sha256"] for r in json.loads(pin_path.read_text())} if pin_path.exists() else {}
    manifest = []
    for path, url in items:
        if not path.exists() and download:
            path.parent.mkdir(parents=True, exist_ok=True)
            temp = path.with_suffix(path.suffix+".part")
            try:
                with urllib.request.urlopen(url, timeout=120) as response, temp.open("wb") as output:
                    while block := response.read(1024*1024):
                        output.write(block)
                if path.suffix == ".zip" and not zipfile.is_zipfile(temp):
                    raise ValueError(f"Not an archive: {url}")
                temp.replace(path)
            finally:
                temp.unlink(missing_ok=True)
        if not path.exists():
            raise FileNotFoundError(f"Missing {path}; use --download (upstream rolling archives may expire)")
        relative, digest = str(path.relative_to(ROOT)), sha256(path)
        if relative in pins and pins[relative] != digest:
            raise ValueError(f"Source differs from recorded SHA256: {relative}")
        manifest.append({"file": relative, "url": url, "sha256": digest, "bytes": path.stat().st_size})
    save(pin_path, manifest)
    return manifest


def mms(archive):
    """Handle both a daily ZIP of interval ZIPs and a ZIP containing one CSV."""
    with zipfile.ZipFile(archive) as z:
        for name in sorted(z.namelist()):
            if name.lower().endswith(".zip"):
                yield from mms(io.BytesIO(z.read(name)))
                continue
            if not name.lower().endswith(".csv"):
                raise ValueError(f"Unexpected archive member {name}")
            headers = {}
            with z.open(name) as raw:
                for row in csv.reader(io.TextIOWrapper(raw, encoding="utf-8-sig")):
                    if row[0] == "I":
                        headers[row[2]] = row[4:]
                    elif row[0] == "D":
                        h = headers[row[2]]
                        if len(h) != len(row)-4 or len(set(h)) != len(h):
                            raise ValueError("Invalid or ambiguous MMS columns")
                        yield row[2], dict(zip(h, row[4:]))


def prepare(download=False):
    regions, prices, cases, constraints = {}, {}, {}, Counter()
    manifest, examples = sources(download), []
    for name in FILES:
        day = name[:8]
        path = RAW/f"PUBLIC_DISPATCHIS_{day}.zip"
        for table, row in mms(path):
            if table not in ("REGIONSUM", "PRICE", "CASE_SOLUTION", "CONSTRAINT"):
                continue
            if row["INTERVENTION"] != "0":
                constraints["intervention_rows_excluded"] += 1
                continue
            time = row["SETTLEMENTDATE"]
            if row["RUNNO"] != "1":
                raise ValueError("Unresolved dispatch run version")
            if table in ("REGIONSUM", "PRICE"):
                dest = regions if table == "REGIONSUM" else prices
                key = (time, row["REGIONID"])
                if key in dest:
                    raise ValueError("Duplicate applicable region/price snapshot")
                dest[key] = row
            elif table == "CASE_SOLUTION":
                if time in cases:
                    raise ValueError("Duplicate case solution")
                cases[time] = row
            else:
                constraints["records"] += 1
                if float(row["VIOLATIONDEGREE"]) > 1e-6:
                    constraints["positive_violation_records"] += 1
                    if len(examples) < 20:
                        examples.append(row)
    if len(cases) != 2016 or len(regions) != 10080 or set(regions) != set(prices):
        raise ValueError("Incomplete seven-day panel")
    max_balance, max_renewable, max_energy_error = 0, 0, 0
    profiles = {}
    for date in [x[:8] for x in FILES]:
        start = dt.datetime.strptime(date, "%Y%m%d")
        profiles[date] = {}
        for region in ("NSW1", "QLD1", "SA1", "TAS1", "VIC1"):
            rows, price_rows = [], []
            for k in range(288):
                timestamp = (start+dt.timedelta(minutes=5*(k+1))).strftime("%Y/%m/%d %H:%M:%S")
                r, p = regions[(timestamp, region)], prices[(timestamp, region)]
                rows.append(r); price_rows.append(p)
                # AEMO DISPATCHREGIONSUM: NETINTERCHANGE is positive net export.
                residual = float(r["DISPATCHABLEGENERATION"])-float(r["DISPATCHABLELOAD"])-float(r["NETINTERCHANGE"])-float(r["TOTALDEMAND"])
                max_balance = max(max_balance, abs(residual))
                for total, a, b in (("UIGF", "SS_SOLAR_UIGF", "SS_WIND_UIGF"),
                                    ("SEMISCHEDULE_CLEAREDMW", "SS_SOLAR_CLEAREDMW", "SS_WIND_CLEAREDMW")):
                    max_renewable = max(max_renewable, abs(float(r[total])-float(r[a])-float(r[b])))
            converted = {}
            for field in ("TOTALDEMAND", "SS_SOLAR_UIGF", "SS_WIND_UIGF", "SS_SOLAR_CLEAREDMW", "SS_WIND_CLEAREDMW"):
                x = [float(r[field]) for r in rows]
                q = [sum(x[k:k+3])/3 for k in range(0, 288, 3)]
                residual = abs(sum(x)/12-sum(q)/4)/max(1, abs(sum(x)/12))
                max_energy_error = max(max_energy_error, residual)
                converted[field] = {"five_minute_mw": x, "quarter_hour_mw": q}
            converted["prices"] = price_rows
            profiles[date][region] = converted
    summary = {"intervals": len(cases), "region_intervals": len(regions),
        "region_balance_max_mw": max_balance, "wind_solar_sum_max_mw": max_renewable,
        "aggregation_relative_energy_error": max_energy_error,
        "constraint_counts": dict(constraints), "constraint_violation_examples": examples,
        "case_solution_status_counts": dict(Counter(r["SOLUTIONSTATUS"] for r in cases.values())),
        "intervals_with_generic_violation": sum(float(r["TOTALGENERICVIOLATION"]) > 1e-6 for r in cases.values()),
        "negative_rrp_region_intervals": sum(float(r["RRP"]) < 0 for r in prices.values()),
        "scope": "Published dispatch intervals, not metered settlement or original forecast information sets"}
    unitfile = RAW/"PUBLIC_NEXT_DAY_DISPATCH_20260901_0000000535633836.zip"
    unit_counts, noon, award_excess = Counter(), {}, 0.0
    discrepancies = []
    for table, row in mms(unitfile):
        if table != "UNIT_SOLUTION" or row["INTERVENTION"] != "0":
            continue
        unit_counts["records"] += 1
        for product in ("RAISEREG", "LOWERREG", "RAISE5MIN", "LOWER5MIN", "RAISE6SEC", "LOWER6SEC", "RAISE60SEC", "LOWER60SEC", "RAISE1SEC", "LOWER1SEC"):
            if row[product] == "" or row[product+"ACTUALAVAILABILITY"] == "":
                unit_counts["missing_award_or_availability"] += 1
                continue
            excess = float(row[product])-float(row[product+"ACTUALAVAILABILITY"])
            award_excess = max(award_excess, excess)
            unit_counts["fcas_checks"] += 1
            if excess > .05:
                unit_counts["fcas_exceeds_actual_availability"] += 1
                discrepancies.append({"product": product, "excess_mw": excess, "record": row})
        if row["SETTLEMENTDATE"] == "2026/09/01 12:00:00":
            if row["DUID"] in noon:
                raise ValueError("Duplicate noon dispatch unit")
            noon[row["DUID"]] = row
    summary["unit_dispatch"] = {**unit_counts, "max_fcas_award_excess_mw": award_excess, "noon_units": len(noon)}
    summary["fcas_discrepancies"] = discrepancies
    summary["fcas_comparison_status"] = "unresolved_field_semantics" if discrepancies else "no_excess_observed"
    summary["identities_scope"] = "Regional balance, wind/solar sums and time aggregation only; excludes FCAS comparison"
    summary["identities_pass"] = max_balance <= .05 and max_renewable <= .05 and max_energy_error <= 1e-10
    save(OUT/"official_data_audit.json", summary)
    save(OUT/"aemo_profiles.json", profiles)
    save(OUT/"aemo_noon_unit_dispatch.json", noon)
    save(OUT/"sources.json", manifest)
    # These are original FCAS capacity-price inputs, not Yunnan mileage bids.
    bidsfile = ROOT/"external_data/market_bids"/("PUBLIC_BIDMOVE_COMPLETE_"+FILES[4]+".zip")
    fcday, fcper = {}, {}
    for table, row in mms(bidsfile):
        if row["BIDTYPE"] not in ("RAISEREG", "LOWERREG") or row["DIRECTION"] != "GEN":
            continue
        key = (row["SETTLEMENTDATE"], row["DUID"], row["BIDTYPE"], row["DIRECTION"])
        if table == "BIDDAYOFFER_D":
            if key in fcday:
                raise ValueError("Ambiguous FCAS daily snapshot")
            fcday[key] = row
        elif table == "BIDPEROFFER_D" and row["PERIODID"] == "96":
            if key in fcper or row["INTERVAL_DATETIME"] != "2026/09/01 12:00:00":
                raise ValueError("Ambiguous FCAS interval or unexpected clock")
            fcper[key] = row
    joined = {k[1]+":"+k[2]: {"price": fcday[k], "quantity": v} for k, v in fcper.items() if float(v["MAXAVAIL"]) > 0}
    save(OUT/"aemo_noon_fcas_bids.json", joined)
    print(json.dumps({k: v for k, v in summary.items() if k not in ("constraint_violation_examples", "fcas_discrepancies")}, indent=2), flush=True)
    return profiles, summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true", help="Download missing official sources, check pinned hashes")
    prepare(parser.parse_args().download)
