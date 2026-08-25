#!/usr/bin/env python3
"""Independent equation oracle for the reliability parameter resolver.

This script does not import or link hacdcpf. It re-derives the Billinton & Allan
(1996) alternating-renewal closed forms that resolve_reliability_params applies —
component unavailability U = lambda / (lambda + mu) with mu = H / r, the
MTBF/MTTF -> lambda conversions, the forced-outage-rate identities
lambda = f / ((1 - f) r) * H, the calendar-basis correction
lambda = f_cal / (1 - f_cal r / H), and the active-on-demand equivalence
lambda_active = nu_demand * p_demand — from the raw failure-mode inputs emitted
by validate_reliability_xref, and checks the resolved canonical parameters.

The governing rules are documented in
docs/modules/reliability/chapters/theory_reliability_foundations.tex and mirrored
in src/reliability/reliability_assessment.cpp (resolve_reliability_params).

Usage:
    run_cross_validation.py --executable <bin> --out <json> --markdown <md>
Exit status is 1 if any resolved field exceeds the tolerance.
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

TOLERANCE = 1e-9
FIELDS = ("lambda_per_year", "repair_hr", "unavailability", "mttf_hr",
          "calendar_frequency_per_year", "lambda_active_per_year")


def _valid(value):
    return math.isfinite(value) and value > 0.0


def expected_params(raw, policy):
    """Re-derive the canonical parameters from first principles."""
    hours = policy["hours_per_year"]
    hours = hours if (math.isfinite(hours) and hours > 0.0) else 8760.0

    has_lambda = _valid(raw["failure_rate_per_year"])
    has_mttr_hr = _valid(raw["mttr_hr"])
    has_mtbf = _valid(raw["mtbf_hours"])
    has_mttr_hours = _valid(raw["mttr_hours"])
    has_mttf = _valid(raw["mttf_hours"])
    forced = raw["forced_outage_rate"]
    has_for = math.isfinite(forced) and 0.0 < forced < 1.0
    p_demand = raw["probability_per_demand"]
    has_pdemand = math.isfinite(p_demand) and 0.0 < p_demand <= 1.0
    has_demandfreq = _valid(raw["demand_frequency_per_year"])
    has_cyber = _valid(raw["cyber_recovery_hr"])

    out = {name: 0.0 for name in FIELDS}

    def finalize(frequency, repair, calendar_input=False):
        lam = frequency
        if calendar_input and repair > 0.0:
            calendar_u = frequency * repair / hours
            lam = frequency / (1.0 - calendar_u)
        out["lambda_per_year"] = lam
        out["repair_hr"] = repair
        if repair > 0.0:
            mu = hours / repair
            out["unavailability"] = lam / (lam + mu) if (lam + mu) > 0.0 else 0.0
        out["mttf_hr"] = hours / lam if lam > 0.0 else 0.0
        out["calendar_frequency_per_year"] = (
            frequency if calendar_input else (1.0 - out["unavailability"]) * lam)

    active_repair = (raw["cyber_recovery_hr"] if has_cyber
                     else raw["mttr_hr"] if has_mttr_hr
                     else raw["mttr_hours"] if has_mttr_hours else 0.0)
    active_case = raw["is_active"] and has_pdemand and has_demandfreq
    may_default = policy["default_policy"] != "strict"
    active_usable = active_case and (not raw["active_params_are_template"] or may_default)

    if active_usable:
        lambda_active = raw["demand_frequency_per_year"] * raw["probability_per_demand"]
        out["lambda_active_per_year"] = lambda_active
        finalize(lambda_active, active_repair)
        out["calendar_frequency_per_year"] = lambda_active
    elif has_lambda and has_mttr_hr:
        finalize(raw["failure_rate_per_year"], raw["mttr_hr"],
                 policy["failure_rate_basis"] == "calendar")
    elif has_mttf:
        lam = hours / raw["mttf_hours"]
        repair = raw["mttr_hours"] if has_mttr_hours else (raw["mttr_hr"] if has_mttr_hr else 0.0)
        if repair > 0.0:
            finalize(lam, repair)
            out["unavailability"] = repair / (raw["mttf_hours"] + repair)
        else:
            out["lambda_per_year"] = lam
            out["mttf_hr"] = raw["mttf_hours"]
    elif has_mtbf:
        mttf = raw["mtbf_hours"]
        if policy["mtbf_convention"] == "cycle" and has_mttr_hours:
            mttf = max(raw["mtbf_hours"] - raw["mttr_hours"], 0.0)
        lam = hours / mttf if mttf > 0.0 else 0.0
        if has_mttr_hours:
            finalize(lam, raw["mttr_hours"])
            out["unavailability"] = raw["mttr_hours"] / (mttf + raw["mttr_hours"])
        else:
            out["lambda_per_year"] = lam
            out["mttf_hr"] = mttf
    elif has_for and (has_mttr_hr or has_mttr_hours):
        mttr = raw["mttr_hr"] if has_mttr_hr else raw["mttr_hours"]
        f = forced
        out["unavailability"] = f
        out["repair_hr"] = mttr
        out["lambda_per_year"] = f / ((1.0 - f) * mttr) * hours
        out["calendar_frequency_per_year"] = f * hours / mttr
        out["mttf_hr"] = mttr * (1.0 - f) / f
    elif has_for:
        out["unavailability"] = forced
    elif has_lambda:
        out["lambda_per_year"] = raw["failure_rate_per_year"]
        out["calendar_frequency_per_year"] = (
            raw["failure_rate_per_year"]
            if policy["failure_rate_basis"] == "calendar" else 0.0)
        out["mttf_hr"] = hours / raw["failure_rate_per_year"]

    return out


def validate_case(case):
    expected = expected_params(case["raw"], case["policy"])
    resolved = case["resolved"]
    return {name: abs(expected[name] - resolved[name]) for name in FIELDS}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--markdown", required=True)
    args = parser.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    raw = out.with_name("reliability_resolver_solver_output.json")
    subprocess.run([args.executable, str(raw)], check=True)
    payload = json.loads(raw.read_text())

    summary = {case["name"]: validate_case(case) for case in payload["cases"]}
    observed = [value for row in summary.values() for value in row.values()]
    worst = max(observed) if observed else 0.0
    report = {
        "schema": "hysim-reliability-resolver-cross-validation-v1",
        "tolerance": TOLERANCE,
        "checks": summary,
        "max_error": worst,
        "passed": worst <= TOLERANCE,
        "scope": ["failure-mode-parameter-resolution"],
        "limitations": [
            "Covers the deterministic parameter resolver (lambda/repair/"
            "unavailability/MTTF/frequency); system-level EENS/SAIDI from the "
            "Monte-Carlo and FMEA engines are validated by their own suites.",
            "The oracle re-derives the same published closed forms; it is an "
            "independent-implementation check, not a second physical model.",
        ],
    }
    out.write_text(json.dumps(report, indent=2) + "\n")

    lines = ["# Reliability parameter-resolver cross-validation", "",
             f"Tolerance: `{TOLERANCE:.1e}`; worst error: `{worst:.3e}`; "
             f"overall: `{'PASS' if report['passed'] else 'FAIL'}`.", "",
             "| Case | lambda | repair | U | MTTF | cal.freq |",
             "|---|---:|---:|---:|---:|---:|"]
    for name, row in summary.items():
        lines.append(
            f"| {name} | {row['lambda_per_year']:.2e} | {row['repair_hr']:.2e} | "
            f"{row['unavailability']:.2e} | {row['mttf_hr']:.2e} | "
            f"{row['calendar_frequency_per_year']:.2e} |")
    lines += ["", "Scope: deterministic Billinton & Allan failure-mode parameter "
              "resolution, independently re-derived.", ""]
    Path(args.markdown).write_text("\n".join(lines))

    if not report["passed"]:
        raise SystemExit("independent reliability-resolver oracle failed")


if __name__ == "__main__":
    main()
