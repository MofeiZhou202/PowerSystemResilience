# Harmonic Power-Flow — Correctness Verification

This note records the correctness verification of the harmonic power-flow (HPF)
module (`src/harmonics_power_flow/harmonics_power_flow.cpp`, API in
`include/hacdcpf/analysis/harmonics_power_flow.hpp`) implementing the hybrid AC/DC
"harmonic penetration" study from `docs/harmonic_*.md`.

## Summary

The solver was reviewed against the design docs and standard HPF theory, the unit
suite was strengthened, one real bug was fixed, and the solver was cross-validated
against two independent external implementations. **The single-phase linear core,
the Norton current-injection convention, branch-flow recovery, and the THD/IHD
formulas now agree with both an independent numpy reference and the OpenDSS engine
to ~1e-13 pu.**

## 1. Code review findings

A line-by-line review of the implementation against the documented equations found
the foundational solver mathematically sound and consistent with the docs.
Confirmed-correct items (with the tests that pin them):

* Per-order impedance `Z(h)=r+j·h·x`, line charging `j·h·b`, shunt `g+j·h·b`,
  source grounding `1/(r+j·h·x'')`, skin-effect `R(h)` models.
* Current injection "positive into network"; `from→to` branch current `ys·(Vf−Vt)`.
* THD/IHD: `sqrt(Σ_{h≠fund}|V_h|²)/|V_fund|·100`.
* Three-phase sequence-by-order routing `((h%3))` → pos/neg/zero, and the
  symmetrical-component injection/read-out.
* NIC operating point `I_ac,1 = conj(S_ac)/conj(V_ac,1)`, `I_dc,0 = P_dc/V_dc,0`.
* Newton Jacobian `Ĵ = Ŷ − ∂Î_res/∂V̂`; the real/imag (2N) form for
  non-holomorphic constant-power loads; bilinear frequency-mixing derivatives.
* Frequency-scan driving-point impedance `Z_dp=(Ŷ⁻¹)_kk`; K-factor, TDD, `I²R(h)`
  losses.

## 2. Bug fixed

**DC branch-flow current ignored the ripple inductance.** The DC branch-flow loop
computed `|(Vf−Vt)/r|` (resistance only) while the network solve uses
`Z(r)=r_pu+j·r·x_pu` when `dc_ripple_model.branch_x_pu` is set, so the reported DC
branch current was overstated at higher ripple orders. Fixed to use the same
order-dependent impedance as `build_dc_ybus` (`harmonics_power_flow.cpp`, DC
branch-flow loop). Pinned by a new test: in a radial DC feeder KCL forces the
branch current to equal the injected ripple current exactly (1.0 pu); the pre-fix
formula returned 2.6 pu.

Reviewed but **not** bugs: the three-phase balanced negative-sequence rotation is
correct; `ThreePhaseLoad` has no `scaling` field (so none is dropped).

## 3. Test coverage added

`tests/test_harmonics_power_flow.cpp` grew from 45 to 50 cases (315 → 344
assertions), closing previously-untested paths:

1. DC branch flow with ripple inductance (pins the §2 fix).
2. Three-phase load-as-shunt-impedance path (no prior 3-phase test enabled it) —
   analytic positive-sequence value + strict attenuation.
3. NIC operating point derived from a **converged base power flow** (the
   `vsc_transfers` path; every other NIC test hand-fed setpoints) on
   `build_ieee14_acdc()`.
4. Single-phase NIC Norton output admittance `y_out_ac` stamping (analytic
   `|V|=1.2` vs `1.5` ideal).
5. THD with a depressed fundamental voltage (denominator robustness).

## 4. Documented simplifications

Annotated in the header so users see them: the load conductance `P/V²` is not
frequency/skin-scaled; the NIC `P_dc = −P_ac` lossless guess (now also surfaced in
`HPFResult::message` when taken); `solve_harmonic_power_flow_3ph` covers the AC
network only and ignores `auto_nic_from_vscs`.

## 5. External cross-validation

`tools/harmonics_validation/` validates the solver on a canonical two-bus case
(`case.json`, the single source of truth) against two independent code bases:

| Reference | Result |
|-----------|--------|
| Independent numpy nodal solver | max \|Δ\| = **2.8e-17** pu |
| OpenDSS engine (OpenDSSDirect.py 0.9.4) | max \|Δ\| = **6.0e-13** pu |

A notable finding during this work: OpenDSS `Line` elements carry a default
frequency-dependent / earth-return impedance model, so the apples-to-apples
comparison models the series branch as an OpenDSS `Reactor` (`Z(h)=r+j·h·x`),
matching the solver and the docs.

Reproduce:

```bash
cmake --build build_rel --target validate_harmonics_xref
cd tools/harmonics_validation
python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt
python compare.py        # -> OVERALL: PASS
```

## 6. Scope

The implemented solver is the documented **"Level 1" direct-nodal penetration**
method, not the paper's full hybrid-parameter Newton; this is by design (see the
header preamble). The Newton family (nonlinear, non-holomorphic constant-power,
cross-order mixing, hybrid AC/DC bilinear) is covered by the unit suite's
reduces-to-linear and analytic-fixed-point tests.
