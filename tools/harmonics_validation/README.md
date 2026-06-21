# Harmonic Power-Flow — External Cross-Validation

Independent validation of `hacdcpf::harmonics::solve_harmonic_power_flow` against
two separate code bases solving the **identical** network defined in
[`case.json`](case.json):

| Reference | Code base | Role | Tolerance |
|-----------|-----------|------|-----------|
| **numpy** (`reference_numpy.py`) | Independent Python/numpy re-implementation of the documented nodal model | Authoritative, always runs | `\|Δ\|` ≤ `1e-7` (abs, pu) |
| **OpenDSS** (`validate_opendss.py`) | The OpenDSS engine via `OpenDSSDirect.py` | External-tool cross-check (optional) | ≤ `2%` (rel) |

Why this is a real external check: the C++ solver, the numpy reference, and the
OpenDSS model are three independent implementations. Agreement to ~`1e-7` with the
numpy reference pins the linear core, the Norton current-injection convention, the
branch-flow recovery, and the THD formula; agreement with OpenDSS additionally
validates against a widely-used third-party harmonics engine.

## The canonical case (`case.json`)

A two-bus radial feeder — SLACK source behind a sub-transient reactance `x''` →
series line → a nonlinear load modelled as a Norton harmonic current source
(orders 5/7/11/13). `include_load_impedance=false` and `run_base_power_flow=false`
so the load is a pure current injector and the fundamental reference is flat
(`|V₁| = 1`), which maps cleanly onto all three solvers. `case.json` is the single
source of truth read by every solver.

## Modelling map (so the three agree)

* Per-order impedance `Z(h) = r + j·h·x`; slack grounded by `y_src(h)=1/(j·h·x'')`.
* OpenDSS uses a self-consistent single-phase SI with `Z_base = 1 Ω`
  (`basekv = 1 kV`, `I_base = 1000 A`), so a pu impedance equals its ohm value and
  `AllBusMagPu` reads back directly in pu. The slack is a `Vsource` with `pu=0`
  (a harmonic short behind `x''`); the load is an `Isource`; each order is a
  `set mode=direct` solve at `h·f₁`, where OpenDSS scales reactances by `h`.

## Run it

```bash
# 1. Build the C++ driver (emits the solver's result for case.json):
cmake --build build_rel --target validate_harmonics_xref

# 2. Set up Python and run the comparison:
cd tools/harmonics_validation
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt          # numpy required; OpenDSSDirect.py optional
python compare.py
```

`compare.py` locates the driver under `build_rel/tests` or `build/tests`
automatically (or pass `--cpp-driver <path>`), runs all three solvers, prints a
per-order table for each comparison, and exits non-zero on any mismatch. If
`OpenDSSDirect.py` is not installed the OpenDSS comparison is **skipped** (not
failed) and the numpy check still runs.

## Files

| File | Purpose |
|------|---------|
| `case.json` | Shared canonical case (single source of truth) |
| `validate_harmonics_xref.cpp` | C++ driver → `cpp_result.json` (built as a CMake target) |
| `reference_numpy.py` | Independent numpy nodal solver |
| `validate_opendss.py` | OpenDSS (OpenDSSDirect.py) cross-check |
| `compare.py` | Orchestrates the three solvers and asserts agreement |
| `requirements.txt` | Python dependencies |
