#!/usr/bin/env python3
"""Summarize saved evidence without rerunning or relabeling failed security checks."""
import json
import re
from collections import Counter

from aemo_data import OUT, ROOT
from market_bid_empirical import save, sha256


def read(name):
    return json.loads((OUT/name).read_text())


def main():
    checks = read("campaign_checks.json")
    maxima = {}
    for check in checks:
        for name, value in (check["evidence"].get("maxima", {}) if isinstance(check["evidence"], dict) else {}).items():
            maxima[name] = max(maxima.get(name, 0), value)
    studies = []
    for scenario in read("fault_inflow_study.json")["job"]["scenarios"]:
        audit = scenario["days"][0]["analysis"]["ac_audit"]
        violations = [v for p in audit["periods"] for v in p["violations"]]
        grouped = {}
        for v in violations:
            kind = v["name"].split("/")[0]
            grouped[kind] = max(grouped.get(kind, 0), v["excess"])
        studies.append({"name": scenario["name"], "status": audit["status"],
                        "nonconverged_periods": sum(not p["converged"] for p in audit["periods"]),
                        "violation_counts": dict(Counter(v["name"].split("/")[0] for v in violations)),
                        "max_excess_by_type": grouped,
                        "formal_settlement_eligible": scenario["days"][0]["analysis"]["settlement"]["formal_settlement_eligible"]})
    regressions = []
    for stem, binary in (("southern", "test_southern_market"), ("generic", "test_market_simulation"), ("forecast", "test_market_forecast")):
        path = OUT/f"{stem}-regression.txt"
        match = re.search(r"All tests passed \((\d+) assertions in (\d+) test cases\)", path.read_text())
        if not match:
            raise ValueError(f"No successful regression evidence in {path}")
        exe = ROOT/"build/macos-release/tests"/binary
        regressions.append({"binary": str(exe.relative_to(ROOT)), "binary_sha256": sha256(exe),
                            "assertions": int(match[1]), "test_cases": int(match[2]),
                            "log_sha256": sha256(path), "rebuilt_this_campaign": False})
    result = {"checks": len(checks), "passed": sum(c["passed"] for c in checks),
              "failed_checks": [c for c in checks if not c["passed"]], "equation_maxima": maxima,
              "mutation_checks": [c["name"] for c in checks if c["name"].startswith("mutation/")],
              "ac_studies": studies, "cpp_regressions": regressions,
              "browser_logs": {s: (OUT/f"{s}-gui.txt").read_text().strip() for s in ("generic", "forecast", "realtime", "ancillary")},
              "official_data": {k: v for k, v in read("official_data_audit.json").items() if not k.endswith("examples") and k != "fcas_discrepancies"},
              "artifact_sha256": {p.name: sha256(p) for p in sorted(OUT.glob("*.json")) if p.name != "validation_summary.json"},
              "scope": "Finite AEMO input-transfer and synthetic rule regressions; not NEM replay, AC certification or all-function empirical validation"}
    save(OUT/"validation_summary.json", result)
    print(f"Saved {result['passed']}/{result['checks']} checks; {sum(r['test_cases'] for r in regressions)} cached C++ tests; {len(studies)} AC studies")


if __name__ == "__main__":
    main()
