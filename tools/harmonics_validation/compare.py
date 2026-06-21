"""Cross-validate the C++ harmonic power-flow solver against independent references.

Pipeline:
  1. Run the C++ driver `validate_harmonics_xref` on case.json -> cpp_result.json.
     (Located in build*/tests or via --cpp-driver.)
  2. Solve the SAME case with the independent numpy reference (always available).
  3. Solve the SAME case with OpenDSS (OpenDSSDirect.py) if installed.
  4. Compare per-order bus voltage magnitudes and bus THD, and report PASS/FAIL.

Exit code 0 = all available comparisons within tolerance, 1 = mismatch / error.
OpenDSS being absent is a SKIP, not a failure (the numpy check still runs).
"""

import argparse
import glob
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

import reference_numpy            # noqa: E402
import validate_opendss          # noqa: E402


def find_cpp_driver(explicit):
    if explicit:
        return explicit
    pats = [
        os.path.join(ROOT, "build_rel", "tests", "validate_harmonics_xref"),
        os.path.join(ROOT, "build", "tests", "validate_harmonics_xref"),
        os.path.join(ROOT, "build*", "**", "validate_harmonics_xref"),
    ]
    for p in pats:
        hits = glob.glob(p, recursive=True)
        hits = [h for h in hits if os.access(h, os.X_OK) and os.path.isfile(h)]
        if hits:
            return hits[0]
    return None


def run_cpp(driver, case_path, out_path):
    subprocess.run([driver, case_path, out_path], check=True, cwd=HERE)
    with open(out_path) as f:
        return json.load(f)


def cpp_to_maps(cpp):
    """cpp_result.json -> {bus: {'thd': x, 'orders': {h: mag}}}."""
    out = {}
    for b in cpp["buses"]:
        out[int(b["bus"])] = {
            "thd": b["thd_pct"],
            "orders": {int(o["order"]): o["mag"] for o in b["orders"]},
        }
    return out


def ref_to_maps(ref):
    out = {}
    for b, d in ref["buses"].items():
        out[int(b)] = {
            "thd": d["thd_pct"],
            "orders": {int(h): d["orders"][h]["mag"] for h in d["orders"]},
        }
    return out


def compare(name, cpp, other, orders, abs_tol=None, pct_tol=None):
    """Compare per-order magnitudes + THD. Returns (ok, max_abs, max_pct)."""
    max_abs, max_pct = 0.0, 0.0
    rows = []
    for bus in sorted(cpp):
        if bus not in other:
            continue
        for h in orders:
            a = cpp[bus]["orders"].get(h)
            b = other[bus]["orders"].get(h)
            if a is None or b is None:
                continue
            d = abs(a - b)
            p = d / abs(a) * 100.0 if abs(a) > 1e-12 else 0.0
            max_abs = max(max_abs, d)
            max_pct = max(max_pct, p)
            rows.append((bus, h, a, b, d, p))
        # THD
        ta, tb = cpp[bus]["thd"], other[bus]["thd"]
        dthd = abs(ta - tb)
        pthd = dthd / abs(ta) * 100.0 if abs(ta) > 1e-12 else 0.0
        max_pct = max(max_pct, pthd)
        rows.append((bus, "THD", ta, tb, dthd, pthd))

    ok = True
    if abs_tol is not None:
        ok = ok and max_abs <= abs_tol
    if pct_tol is not None:
        ok = ok and max_pct <= pct_tol

    print(f"\n=== C++ vs {name} ===")
    print(f"{'bus':>4} {'ord':>4} {'C++':>14} {name[:12]:>14} {'|Δ|':>11} {'%':>8}")
    for bus, h, a, b, d, p in rows:
        print(f"{bus:>4} {str(h):>4} {a:14.6e} {b:14.6e} {d:11.3e} {p:8.3f}")
    tol_str = []
    if abs_tol is not None:
        tol_str.append(f"|Δ|<={abs_tol:g}")
    if pct_tol is not None:
        tol_str.append(f"%<={pct_tol:g}")
    print(f"max |Δ| = {max_abs:.3e}, max % = {max_pct:.4f}  "
          f"[{' and '.join(tol_str)}]  -> {'PASS' if ok else 'FAIL'}")
    return ok, max_abs, max_pct


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default=os.path.join(HERE, "case.json"))
    ap.add_argument("--cpp-driver", default=None)
    ap.add_argument("--out", default=os.path.join(HERE, "cpp_result.json"))
    args = ap.parse_args()

    with open(args.case) as f:
        case = json.load(f)
    orders = list(case["ac_orders"])
    abs_tol = float(case.get("tol_numpy_abs", 1e-7))
    pct_tol = float(case.get("tol_opendss_pct", 2.0))

    driver = find_cpp_driver(args.cpp_driver)
    if not driver:
        print("ERROR: validate_harmonics_xref not built. Build target "
              "'validate_harmonics_xref' (cmake --build build_rel --target "
              "validate_harmonics_xref), or pass --cpp-driver.", file=sys.stderr)
        return 1
    print(f"C++ driver: {driver}")
    cpp = cpp_to_maps(run_cpp(driver, args.case, args.out))

    # 1) Independent numpy reference (authoritative, always available).
    npy = ref_to_maps(reference_numpy.solve_reference(case))
    ok_np, _, _ = compare("numpy", cpp, npy, orders, abs_tol=abs_tol)

    # 2) OpenDSS cross-check (optional).
    ok_dss = True
    dss = validate_opendss.solve_opendss(case)
    if dss is None:
        print("\n=== C++ vs OpenDSS === SKIPPED (OpenDSSDirect.py not installed)")
    else:
        dssm = ref_to_maps(dss)
        ok_dss, _, _ = compare(f"OpenDSS", cpp, dssm, orders, pct_tol=pct_tol)

    overall = ok_np and ok_dss
    print(f"\nOVERALL: {'PASS' if overall else 'FAIL'}")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
