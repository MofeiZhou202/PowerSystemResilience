# Development Status

Updated: 2026-08-16

This is the living handoff for verified build state and active engineering work.
Update it in place; do not create dated copies. Source, registered tests, and
the current Git worktrees remain authoritative.

## Verified baseline

| Scope | Result |
|---|---|
| Required MIPSolvers source | Current pin `3bf1e66749e3b3e0bbd57696a7d4f43ecf09218c`; the clean sibling worktree matches it and includes the PF KLU numeric-refactor interface/adapter. The PF regressions below used the same three-file dependency content before it was committed. The earlier general full-regression baseline was established at `60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0`. |
| `full-dev` regression | 1430/1430 registered tests completed without failure; 3 condition-dependent tests skipped |
| `macos-release` regression | 1425/1425 registered tests completed without failure; 3 condition-dependent tests skipped |
| Graph ASan/UBSan subset | 28 cases, 113 assertions passed after the iterative Tarjan fix |
| Reliability ASan/UBSan | Complete three-stage suite 25 cases/1339 assertions; `case33mg_acdc` 477 assertions and five consecutive parallel repeats passed; `test_1_no_sop` 79 assertions |
| Market ASan/UBSan | Complete suite 22 cases/845 assertions; focused initial root-cut case 1/42 |
| Other sanitizer subsets | Thread pool 4 cases/6 assertions; `test_hacdcpf` 26/89; `test_acopf_dcopf_crossval` 13/119; `test_power_flow_math_audit` 42/231 |

## Balanced VSC current-limit NCP

The production unified Newton path now has an opt-in fixed six-state local
block per supported `PQ_MODE`, `VDC_Q`, or grid-connected `AC_GRID_FORMING`
converter. The shared `(Pac,Qac,Pdc,Er,Ei,lambda)` layout uses identity internal-
voltage slots for power-port modes and an explicit internal voltage behind
virtual impedance for GFM. It enforces the AC current disk,
Magnitude/P-first/Q-first policy, energy balance, and saturated Vdc-droop
command. Network and local generalized Jacobians, including terminal-angle
derivatives, are analytic and retain one fixed sparse layout across activation.

The complete Release `test_vsc_limit_ncp` target passes 45 cases and 540
assertions. It covers three priorities, the full local/terminal Jacobian
finite-difference check, two simultaneous GFM limits, zero-radius priority
ties, weak-grid and infeasible points, three-step quasi-steady replay, balanced
OPF dispatch replay, two-level OpenDSS validation, legacy power-port oracles,
nonbinding equivalence, Vdc saturation, structural admission, standard/JPC
round trips, stable identity, shared parameter precedence, the production
demo, sole/multiple slackless GFM islands, adaptive island extraction, and the
case300/ACTIVSg2000 families. Exact FB now satisfies the biactive origin rather
than adding a hidden epsilon. Optional smooth FB/CHKS continuation covers all
three power-port priorities and all three current-limited GFM priorities; the
measured solves took 16--25 iterations with 11 continuation updates, retained
one Jacobian pattern build, and returned exact residual certificates between
`1.0e-12` and `3.6e-10`. The three-phase hybrid OPF target
passes 12 cases/152 assertions, including rich-to-phase GFM parameter mapping.

The focused transient initialization contract passes 4 cases and 51
assertions. Its primary 15-assertion case certifies the stable-ID PF-to-dynamic
Norton seed before network trimming: internal voltage, current, and power
errors are each at most `1e-8`. A companion executable guard proves that the
legacy authored fallback resolves identically across PF and dynamics, while an
explicit transient profile override invalidates that continuity certificate
and emits a warning. This seed certificate is distinct from the later
full-dynamic equilibrium, which may move when dynamic device equivalents do
not reproduce the PF slack dispatch. The added case starts without a terminal
SLACK and certifies the stable GFM Norton seed through dynamic construction.

The full `tools/gui_api_e2e.py` run passed all new production GFM/NCP HTTP
checks, including the Schur-policy request/effective-options round trip. The
`gfm_norton_limit_demo` built-in case loads with 2 AC buses, 2 DC
buses, and 2 VSCs; it preserves all four authored Norton fields through GUI
editing and returns stable Canvas references, local certificates, and the GFM
validity flags. The complete script finished 68/69 checks; its unrelated
existing hybrid Auto OPF check reached Ipopt's iteration limit with violation
`1.51e-4`. No GFM/NCP HTTP check failed.

Two versioned MTDC/PQ-Vdc-Q benchmark builders prevent pure-AC evidence from
being misreported as converter-limit scalability. The registered case300 solve
has 300 AC buses, 6 DC buses, and 6 explicit VSC blocks; ACTIVSg2000 has 2000
AC buses, 8 DC buses, and 8 explicit blocks. Each has four simultaneous
power-port limits. Separate GFM builders solve that real MTDC benchmark first,
retain its voltage state as an explicit continuation seed, and add one
nonbinding Norton GFM block; their focused tests each pass 13 assertions. They
prove fixed sparse structure on real hybrid networks, not GFM flat-start
robustness, large-scale simultaneous GFM binding, or GPU acceleration.

The fixed two-warmup/five-repeat Release/KLU Schur benchmark was rerun after
centralizing the numerical policy. Forced case300 Schur reduced dimension
`640 -> 604` and structural nonzeros `4820 -> 4502`, but increased median
linear time `40.9%` and wall time `15.7%`; the production default therefore
keeps this case on full LU. ACTIVSg2000 reduced `4054 -> 4006` and
`29806 -> 29382`, with median linear and wall reductions of `14.0%` and
`12.5%`. The Schur enable flag, network-dimension admission, local `rcond`,
and backward-error tolerances now round-trip through the production PF HTTP
configuration. Invalid dimensionless tolerances restore named defaults; the
global machine epsilon remains read-only.

The admitted GFM scope is balanced positive-sequence steady/quasi-steady
operation. A slackless island retains all terminal `Vm/Va` variables and is
anchored by one or more authored fixed internal GFM Norton phasors; no terminal
SLACK is manufactured. Strict coordination still requires physical DC-side
voltage/power support, and the local Newton method does not claim uniqueness
across high- and low-voltage basins. Balanced OPF does not put the
GFM internal-voltage/priority NCP inside its KKT system: the production PF
`post_pf` replay preserves GFM mode and provides the certificate. OpenDSS
independently reproduces the nonbinding Thevenin root; at a binding point it
freezes the HySim internal voltage and re-solves the circuit with native
terminal-power KCL residual at most `1e-6 pu`, which is not presented as an
independent mode-selection oracle. GPU assembly/factorization remains
unimplemented. The concrete authored/canonical/solver/result/replay ownership
rules are in `docs/model_data_semantics_contract.md`; the equations and
numerical boundaries are in `docs/vsc_limit_ncp_power_flow_contract.md`.

The repository-wide semantics guard `test_model_semantics_contract` passes 5
cases and 1531 assertions. It pairs all 43 component I/O collections with
runtime identity/unit/sign/model-fidelity contracts, covers all 22 top-level
production source modules, and locks shared `DCStorage`/`StaticGeneratorDC`
execution views plus additive bus/component demand. This audit found and fixed
loss of `StaticGeneratorDC.cost_c1` in both SolverData construction paths. The
existing component I/O registry also passes 14 cases and 373 assertions.

## Trial edition integration

Current `main` reimplements the two `trial_design` commits on top of the newer
reliability and GUI work. The backend owns an embedded edition profile,
indicator-to-analysis plan, and fail-closed method/path policy. Known disabled
routes return `TRIAL_FEATURE_DISABLED`; an unclassified future API route is
also denied. The GUI consumes the profile and plan, removes disabled controls,
avoids the eager dynamics-schema request, and keeps each solver execution
explicit. Windows packaging requires a clean pinned MIPSolvers checkout, Trial
CTest success, required runtime/data presence, and an extracted-package startup
smoke test before hashing the manifest and ZIP.

The Trial verification below used the earlier MIPSolvers `7b4cba8` pin. That
commit guards the optional CHOLMOD augmented-factor calls and restores the
no-SuiteSparse build. Its clean Release build with SuiteSparse, SuperLU, MKL,
Ipopt, Gurobi, and PaPILO disabled passed `test_ipm_solver`: 30 cases and 188
assertions. The current dependency upgrade and OPF verification are recorded
separately below. No claim is made for a Windows package yet.

A clean Debug Trial build against that checkout rebuilt `test_edition_profile`
and `run_gui_server`. All five registered Trial tests passed in 8.19 seconds:
three C++ profile/policy/plan cases, the fail-closed HTTP E2E, and the
Playwright GUI E2E. Manual browser inspection at 1440x1000 and 390x844 found no
page-level horizontal overflow, toolbar overlap, or clipped controls; the
mobile indicator-plan band was expanded so all nine backend-ordered links end
above the workspace. A separate clean full-edition Debug build passed the full
profile case, `runtime_api_v1_e2e`, `reliability_workflow_e2e`, and
`reliability_dimension_validation_e2e` in 57.93 seconds. JavaScript syntax,
Python AST parsing, preset JSON parsing, required package-data presence, and
`git diff --check` passed. PowerShell is unavailable on this macOS host, so the
Windows preset, staged DLL startup, manifest, and ZIP creation remain
unverified.

The two full CTest results establish the normal build baseline. They do not
claim that every sanitizer entry point is green.

## OPF Phase I/II integration and performance

The checked-in dependency pin is MIPSolvers `3bf1e66`; its clean `main`
contains the audited central warm-start and NativeLCQP cooperative-deadline
contracts used by this HySim change, plus the guarded KLU numeric-refactor
adapter used by PF. Bounded Phase I fronts balanced Parity Native IPM (pure AC
and hybrid AC/DC) and monolithic three-phase hybrid NativeIPM. Pure AC keeps
the state/basic PF-Newton basis. Hybrid models first eliminate converter
`Pdc` directions and solve the DC-conductance/converter Schur block, then use
the same state/basic fallback when the global admission test passes. It applies sparse
constraint Newton steps in assembled OPF coordinates, then performs one
equality-dual and positive inequality-dual/slack fit. A declared state/basic
square Jacobian uses SparseLU; unusual component mixes or active nonlinear
inequalities fall back to direct SparseQR. No dense Jacobian, `J J^T`, barrier
schedule, filter, Hessian solve, second-order correction, elastic variable, or
Ipopt call is part of finite-budget Phase I.

The coordinated defaults are `mu0 = 0.1`, admission ratio `eta_a = 1.0`,
primal handoff ratio `eta_p = 0.1`, and centrality tolerance `eta_c = 0.5`.
Thus Phase I is attempted only when the original-coordinate violation is at
most `eta_a * mu0 = 1e-1`; its finite-accuracy handoff target is
`max(final_primal_tolerance, eta_p * mu0) = 1e-2`. Phase II's final primal,
dual, and complementarity tolerances are unchanged. A handoff may therefore be
inside the Phase I corridor without satisfying the final solution tolerance;
`phase_one_in_handoff_corridor` and `phase_one_primal_feasible` report these
facts separately.

MIPSolvers accepts the handoff only under filter globalization with complete,
finite primal/equality-dual/inequality-dual/slack vectors, positive inequality
duals and slacks, original and perturbed primal residuals within the corridor,
`mu0 <= 0.1`, centrality
`max_i |s_i z_i / mu0 - 1| <= eta_c`, and maximum inequality dual at most
`1e4`. Full dual residual is diagnostic rather than an admission gate because
Phase I deliberately leaves the reduced gradient to Phase II. Rejection is
all-or-nothing: every warm vector is cleared and ordinary interiorization is
used. Diagnostics include the audit residuals and reason plus first accepted
step lengths and barrier-objective change. Fixed-variable reduction preserves
the nonlinear, nonfixed-lower-bound, then nonfixed-upper-bound row order.

Iteration, total primal-plus-dual factorization, and per-iteration backtrack
caps are hard work bounds. The wall-clock deadline is cooperative because an
in-flight sparse factorization cannot be interrupted. The corridor-aware slack
floor `max(2e-10, (epsilon_p - violation) / 2)` keeps `s_i z_i = mu0` without
creating the extreme multipliers observed with a uniform `2e-10` floor. That
original construction increased Phase II from 17 to 35 iterations on case30
and from 13 to 35 on the hybrid microgrid; the revised scaling removed the
regression.

The reproducible `phase_one_restoration_benchmark` protocol uses two warmups
and five timed repetitions. On AppleClang 21, arm64 macOS, Release, case30 AC
improved from `4.458 ms` off to `4.225 ms` on (`1.055x`), while Phase II fell
from 17 to 16 iterations and from 16 to 15 factorizations; Phase I reduced
violation `3.106e-2 -> 4.030e-4` with centrality `2.22e-16`. The 360-bus
three-phase case has 1080 phase nodes and 2175 variables: its `1e-3` point was
already inside the corridor, so Phase I used no primal-restoration iteration
and one dual-fit factorization, reached dual-fit residual `8.33e-15`, and had a
`5.438 ms` median.

Admission prevents wasted full-model sparse work outside the local basin. The
Hybrid Schur step is audited by its own DC-balance/converter/reference norm but
may not bypass the global admission threshold. At the production default, the
microgrid (`0.144 > 0.1`) and case300 ACDC (`1.129 > 0.1`) therefore retain
their exact cold Phase-II starts after the bounded structural probe. With the
research setting `phase_one_admission_mu_factor = 2`, the microgrid Schur block
is accepted and state/basic Newton reaches `1.689e-5`; the central handoff is
certified. The same bypass was deliberately rejected for case300: it incurred
about 160 ms of full-model QR while only reaching `1.100`, so the global gate
remains authoritative.

DCOPF now has a separate affine structural start for NativeLCQP: each energized
component is power-balanced within generator/load-shed bounds, one angle
reference is selected, and a reduced Laplacian projects angles/flows before the
QP IPM. MIPSolvers `QPModel::x0` audits size/finiteness and reports actual use
and initial primal residual; rejection is exactly the deterministic cold
initialization. In the same two-warmup/five-repeat Release protocol, case57
improved from 29 to 18 iterations and `1.934` to `1.284` ms (`1.506x`), case118
from 43 to 18 iterations and `5.841` to `2.766` ms (`2.112x`), and case300
from 68 to 19 iterations and `33.360` to `10.636` ms (`3.137x`). The
original equality residuals were respectively `6.11e-15`, `1.53e-14`, and
`9.13e-14`; solver initial residual was `0.01` after positive bound-slack
initialization, versus cold `2.60`, `4.03`, and `12.00`. The residual,
iteration, and end-to-end predictions therefore held on all three cases.

Large pure-AC Parity solves can additionally compose those structures as an
opt-in dispatch Phase I: ordinary AC PF is evaluated first, and only a poor
full-model initial dual residual triggers a five-iteration compact NativeLCQP
DCOPF. The DC dispatch and reduced-Laplacian angles are replayed through AC PF;
the candidate is admitted only after evaluating the actual Parity initial
primal/dual metrics, and every failure retains the original formulation and PF
point. This is a dispatch warm start rather than a central-state certificate:
on `case9241pegase`, candidate dual residual improved `838.166 -> 236.753`
while the maximum full-model primal metric increased `1.601e-3 -> 1.506e-1`.
The final Native IPM therefore remains responsible for the unchanged KKT
certificate.

The fixed Release/KLU protocol used one warmup and three timed repetitions on
`case9241pegase`. Ordinary AC-PF start measured `12.657 s` median, 77
iterations, and 76 Phase-II factorizations; the structured dispatch start
measured `11.088 s`, 56 iterations, and 55 factorizations. End-to-end median
fell `12.4%` and Phase-II factorizations fell `27.6%`, exceeding the fixed
10%/15% acceptance thresholds. Objectives were `315911.571480` and
`315911.571406`; final primal/dual residuals were respectively
`6.49e-8/4.66e-7` and `5.78e-8/3.05e-7`. On `case13659pegase`, baseline dual
residual `1.970` passed the prefilter, so no DC work ran; the guard retained
138 iterations, 137 factorizations, objective `386116.837520`, primal
`2.05e-7`, and dual `5.89e-9` in `21.506 s`.

The DC Phase-I default wall budget is `2000 ms` end to end: canonical
projection, compact formulation, connected-component/reduced-Laplacian seed,
symbolic analysis, and numeric iterations are all charged. Sparse
factorizations are cooperative indivisible units, so diagnostics report any
overshoot. On case9241 the DC work used `1638 ms`, did not exhaust the budget,
and performed one NativeLCQP symbolic analysis. Baseline and candidate share
one Parity formulation (`parity_formulation_builds=1`); only mutable authored
operating-point seeds are refreshed before candidate slack construction. On
case13659 the prefilter performed zero DC symbolic analyses.

The 20k-class validation uses the 25,000-bus ACTIVSg25k case (4,834
generators and 32,230 AC branches) with Release/KLU on the same AppleClang 21
arm64 host. The fixed protocol parses once, then runs one warmup and three
timed solves. The ordinary AC-PF start samples were `183.967`, `182.383`, and
`183.808 s` (median `183.808 s`); every solve converged in 66 iterations and
65 Phase-II factorizations to objective `6252070.20694`, primal
`3.239e-7`, and dual `1.004e-7`. The process peak RSS was `5.13 GiB`.

With only the default 2-second DC Phase-I budget changed, samples were
`186.588`, `184.993`, and `187.476 s` (median `186.588 s`, 1.51% slower).
The DC path performed one symbolic analysis and two iterations, exhausted the
budget in `2064 ms` with `64 ms` cooperative overshoot, produced no admissible
iterate, and retained the exact baseline start. Phase II therefore remained
66 iterations/65 factorizations with bit-identical objective and KKT metrics;
`parity_formulation_builds` remained one. Full-solve peak RSS was `4.48 GiB`;
the difference from the baseline peak is allocator/run variability and is not
claimed as a memory reduction.

A predeclared budget sweep explains why increasing the DC-IPM allowance is not
the remedy for this case. A 5-second embedded run completed five DC iterations
but still produced no iterate within the `1e-2` handoff tolerance. DC-only
residuals were `3.639e-2` after
`10.55 s/10` iterations, `1.146e-2` after `20.27 s/17`, and `8.767e-4`
after `31.05 s/25`; peak RSS rose from `2.12` to `3.84` and `5.49 GiB`.
The 30-second embedded run reached that last DC residual but its AC replay
changed the actual Parity primal metric from `3.929e-4` to `9.786e-2` and
improved dual only `468.107 -> 448.664` (4.15%, below the fixed 20% gate).
It was correctly rejected and finished in `215.012 s`, 16.98% slower than the
baseline, with unchanged Phase-II work.

The original `O(n^1.5)` nested-dissection estimate predicted 45--90 seconds
from case9241 and was violated. Re-derivation found a machine/cost-model and
input-assumption error rather than implementation infidelity: bus count alone
does not determine KLU fill, elimination-tree separators, generator/bound-row
density, or memory traffic across PEGASE and ACTIVS families. The benchmark
now reports buses, generators, branches, controllable DC budget/iterations,
all handoff diagnostics, and POSIX peak RSS. The verified conclusion is that
the current solver can complete this 25k ACOPF with a final KKT certificate,
but the current compact DC Phase I does not accelerate ACTIVSg25k. Future 20k
work should target a cheaper dispatch/dual predictor or symbolic reuse across
repeated operating points, not a more accurate auxiliary DC IPM.

The first zero-factorization dual-predictor probe is now implemented as an
experimental, default-off option. It forms one active/reactive balance price
per conductive AC component from the median stationarity of interior
generators. The candidate reuses the Jacobians that Phase II already assembles,
performs no symbolic or numeric factorization, and is accepted only if both the
raw and multiplier-normalized full stationarity residual improve by at least
`phase_one_dispatch_dual_min_improvement` (default 5%). Rejection leaves the
ordinary zero equality-dual start bit-for-bit intact. The JSON and `/api/v1`
diagnostics report attempted/accepted, runtime, both residual pairs, and status.

On ACTIVSg25k, the single Release/KLU research probe cost `1.978 ms` but changed
raw stationarity only `47278.840 -> 47276.850` and normalized stationarity only
`468.10733 -> 468.08762`, a `0.0042%` improvement. The candidate was therefore
rejected; the solve completed in `184.344 s` with the unchanged 66 iterations,
65 Phase-II factorizations, objective, and final KKT residuals. Case30 also
worsened and was rejected; case118 improved about 2% but remained below the
fixed 5% production gate. This falsifies the prediction that an island-common
partial dual would remove one KKT factorization on the 25k case. The dominant
stationarity entries lie outside the free-generator price subspace, especially
in bound/inequality and uncovered variable families.

Current research evidence supports the following priority order:

| Direction | Extra factorization | Memory lifetime | Valid after data/topology change | Measured/expected value |
|---|---:|---|---|---|
| Component dispatch/dual predictor | 0 | `O(nb + ng)` for one initialization | Recompute after either change | Dual-only probe is negligible on ACTIVSg25k; a primal dispatch reshape still needs AC-PF replay and end-to-end validation |
| AC-PF adjoint dual seed | One transpose solve only if the final PF factorization is retained; otherwise one new factorization | PF reduced Jacobian and factors must survive into OPF | Numeric reuse only for unchanged values; symbolic reuse requires unchanged PV/PQ and topology pattern | Deliberately excluded: the reduced PV/PQ PF Jacobian does not directly cover the full OPF equality/bound multiplier layout |
| Repeated-solve continuation plus prepared symbolic cache | 0 extra symbolic analyses after a compatible first solve; numeric KKT factorization remains per IPM iteration | Prepared formulation, maps, sparse pattern/order, and last complete `(x,lambda,z,mu)` state persist across solves | Full continuation requires identical layout; symbolic ordering survives parameter changes with fixed pattern; topology/layout changes invalidate it | Highest priority for time series/contingency batches because HySim already validates and reuses a complete continuation state within one compatible formulation |

That highest-priority path is now implemented as the non-copyable, movable
`opf::PreparedACOPFSession`. Its exact compatibility key covers variable and
constraint blocks, bound-row finiteness/order, component maps, reference rows,
limited-device row membership, energy-router port order, and Ybus/Gdc sparse
patterns. LCC stable identity, AC/DC terminals, resolved commutation bus, and
energy-router port stable identity are included because they select KKT row,
column, or public mapping semantics. Identical inputs reuse the owning
`parity::Problem`; compatible load,
cost, setpoint, and limit changes refresh canonical numeric `SolverData` and
derived coefficients without rebuilding OPF maps. A failed structural audit
rebuilds the formulation, clears the KKT symbolic cache, and prevents opaque
state reuse on that solve. The four continuation blocks are all-or-nothing and
still pass the native positivity, centrality, and residual admission checks.
Each sparse backend additionally keys its symbolic analysis by an FNV-1a scan
of every compressed KKT coordinate, not only dimension and nonzero count; an
equal-size, equal-nnz graph with different coordinates is therefore reanalyzed.

On the fixed case118 Release/KLU protocol, three standalone Phase-I solves
measured `22.199/16.407/14.786 ms` (median `16.407 ms`), 27 iterations, 26
factorizations, and 2 symbolic analyses. One prepared session measured
`18.524/7.163/6.760 ms` (median `7.163 ms`), with the repeated solve at 2
iterations, 1 factorization, and 0 analyses. The prepared
`0,+0.5,-0.5,+2,-2%` load sequence measured
`24.907/8.833/7.859/8.374/8.434 ms` (median `8.434 ms`); the final -2% point
used 7 iterations, 6 factorizations, and 0 analyses. Thus the fixed prediction
of at least 25% Phase-II iteration reduction is met on this oracle. Final
objectives and KKT residuals remain within the existing solver tolerances.
After the exact KKT-coordinate cache audit, a two-warmup/ten-repeat case118
prepared-session check measured `6.121 ms` median, 2 iterations, 1 fresh
numeric factorization, and 0 symbolic analyses. This corrected the initial
three-sample timing concern (`8.501 ms` median): the longer sequence showed no
stable regression relative to the earlier `7.163 ms` result.

The 20k-class acceptance run used ACTIVSg25k (25,000 buses, 4,834 generators,
32,230 branches) with `prepared-power-flow`, no warmup, and two solves in one
Release/KLU session. The first solve took `184645.248 ms`, 66 iterations, and
65 factorizations. The compatible repeated solve took `8666.249 ms`, 4
iterations, 3 fresh numeric factorizations, and zero symbolic analyses: a
`95.31%` wall-clock reduction while improving final primal/dual residuals to
`3.389e-8/2.092e-8`. Its objective `6252069.93711` is within the established
large-case tolerance of the first-solve/reference objective. Peak process RSS
was `4709.6 MiB`; therefore the session closes the repeated-solve time target
at 25k but does not close the memory-efficiency target for larger batches.

Numeric-factor reuse remains deliberately unsupported; ordinary
`factorize()` is not labelled as reuse. The current Eigen SuiteSparse adapters and MIPSolvers
MUMPS wrapper expose persistent symbolic analysis followed by fresh numeric
factorization, but no distinct cross-solve factor object with pre-use matrix
drift and post-use backward-residual/refinement audit. Enabling
`prepared_numeric_refactor` therefore returns an explicit `unsupported` status
and performs ordinary numeric factorization. AC-PF adjoint dual seeding also
remains excluded: its reduced PV/PQ Jacobian does not match the full OPF
equality/bound multiplier layout, and without a retained PF factor it adds a
factorization rather than removing one.

The literature check is consistent with these measurements. Baker,
*Learning Warm-Start Points for AC Optimal Power Flow* (2019,
doi:10.1109/MLSP.2019.8918690), and Cao et al., *Fast and explainable
warm-start point learning for AC Optimal Power Flow using decision tree*
(2023, doi:10.1016/j.ijepes.2023.109369), predict primal operating points but
require offline data and distribution-shift controls. Park et al., *Compact
Optimization Learning for AC Optimal Power Flow* (arXiv:2301.08840), reports
learned warm starts up to 30,000 buses, again with an offline model. More
directly relevant, Taheri and Molzahn, *Not All Warm Starts Help: Benchmarking
Primal-Dual Initializations for ACOPF Algorithms* (arXiv:2606.08984), tests
systems up to 30,000 buses and finds most partial primal-plus-dual restarts
slower or less reliable; complete coverage is the robust case, and DC seeding
loses statistical significance after presolve cost. WARP
(arXiv:2605.05728) likewise identifies the complete `(x,lambda,z,mu)` state as
the useful IPM target. Gondosiswanto and Pulsipher
(arXiv:2606.04725) demonstrate the analogous parametric-NLP benefit of keeping
structure-dependent symbolic work across repeated solves. These results do not
justify a learned model in the production path today; they justify a prepared
OPF session with exact compatibility keys, complete-state audit, and symbolic
cache invalidation before attempting numeric refactor reuse.

The formal runs are reproducible with:

```bash
cmake --build build/macos-release --target opf_numerical_benchmark -j4
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode power-flow \
  --warmups 1 --repeats 3 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode structured-power-flow \
  --warmups 1 --repeats 3 --max-iterations 300 \
  --dc-max-iterations 5 --dc-time-limit-ms 2000
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode power-flow-phase-one \
  --warmups 0 --repeats 1 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case118.m --mode prepared-session \
  --warmups 0 --repeats 3 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case118.m --mode prepared-sweep \
  --warmups 0 --repeats 5 --max-iterations 300
./build/macos-release/opf_numerical_benchmark \
  external_data/matpower/case_ACTIVSg25k.m --mode prepared-power-flow \
  --warmups 0 --repeats 2 --max-iterations 300
```

The enhanced diagnostic target rebuilt in both `macos-release` and the local
ASan/UBSan configuration. Focused Release and sanitizer runs each passed the
four prepared-session cases with 46 assertions and the eight combined
warm-start/Phase-I cases with 120 assertions. These include Hybrid AC/DC
numeric refresh and LCC numeric-versus-structural mapping coverage. The 25k
timing protocol itself was Release-only and preceded the final KKT-coordinate
fingerprint hardening; it was not rerun after that `O(nnz)` safety scan. No 25k
sanitizer or full CTest claim is made.

Library JSON, the GUI response, and `/api/v1` now preserve the Phase-I
baseline/candidate residuals, budgets, admission status, and warm-start-only
DC contract. The AC request surface accepts bounded central Phase-I and large
pure-AC DC-dispatch controls; callers must still opt into `ac_pf_warm_start`,
and the DC path defaults to pure AC networks with at least 5000 buses.

A barrier sweep put case30 outside admission at `mu0 = 0.03` and `0.01`,
supporting the retained `mu0 = 0.1` default. Current verification passed
MIPSolvers Native IPM 34 cases/246 assertions, HySim
three-phase hybrid OPF 11/145, and ACOPF/DCOPF cross-validation 13/119. The
complete OPF backend binary passed 22/24 cases and 639/644 assertions; both
failed cases are Ipopt or Auto-to-Ipopt iteration-limit stalls that bypass
Native Phase I. The focused `/api/v1` runtime E2E passed. The earlier rebuilt
GUI/API E2E passed 68/69 checks; its sole failure is the same Auto-to-Ipopt
showcase stall. DCOPF remains affine and pays only
the connected-component/reduced-Laplacian projection when NativeLCQP is
selected. Focused ASan/UBSan verification
passed balanced Phase I 3 cases/45 assertions, three-phase Native coverage 5
cases/54 assertions (6 Ipopt-dependent cases skipped by that build), and
ACOPF/DCOPF cross-validation 13 cases/119 assertions. No full CTest, Windows,
or browser-layout E2E claim is made for this increment.

## DER control and reliability-method comparison

The unified reliability request now applies an optional calculation-copy DER
control scenario: authored roles, forced grid-following, or promotion of
eligible controllable resources to grid-forming. Every result exposes a stable-
ID device audit with authored/effective role, reference capability,
anti-islanding and black-start credit. This is a steady-state reliability
sensitivity model; dynamic synchronization, current limiting and protection-
FRT trajectories remain explicit limitations. Disabling black-start removes
the explicit FMEA storage credit, while methods that still use a composite GFM/
black-start flag disclose that boundary.

The GUI adds deterministic, Monte Carlo and full six-method comparison sets.
Each production result carries a system fingerprint and normalized calculation
basis. The comparison backend rejects mismatched bases, omits unavailable
metrics, reports min/median/max, coefficient of variation and normalized range,
and compares weak-component rankings by stable-ID Spearman correlation, top-5
Jaccard overlap and consensus reciprocal rank. The default reporting horizon is
8760 hours across the compared methods.

Focused `macos-release` verification rebuilt `run_gui_server`. The three
registered GFL/GFM physical cases passed, `reliability_control_state_test`
passed, and the extended `reliability_workflow_e2e` passed in 8.90 seconds. The
latter used the built-in cyber-physical case to verify the per-device control
audit and same-basis component-FMEA versus failure-mode-FMEA comparison,
including statistic bounds and stable consensus identities. JavaScript syntax
and `git diff --check` passed. No complete CTest or sanitizer run was performed
for this increment.

The reliability calculation-guide refresh reused the existing
`macos-release/tests/run_gui_server` binary without rebuilding it. A direct
`reliability_workflow_e2e.mjs` run completed successfully and reproduced the
documented cyber-physical EENS sequences `1.0000005 -> 4.30000075`,
`0.10000005 -> 4.400001`, and `10.000005 -> 20.000005 MWh/year`. Direct
production-API capture for `dist33_microgrid_der` returned the coupled-model
baseline EENS/LOLE/SAIDI of `4.276339 MWh/year`, `1.261167 h/year`, and
`72.861707 min/customer-year`; promoting 11 eligible resources to steady-state
GFM reduced those values to `0.010627`, `0.027833`, and `0.172683`. These
Dist33 values are worked-case evidence, not registered cross-version numeric
thresholds. The validity flags and directional checks remain the enforced
contracts. No full CTest or sanitizer run was performed for this documentation
refresh.

## Reliability configuration enhancement

The dirty worktree now registers `reliability_dimension_validation_e2e`, which
uses the production unified reliability route for seven physical,
information, and intelligent scenarios on both `dist33_microgrid_der` and
`comprehensive_hybrid_acdc`. The registered `macos-release` CTest passed 1/1
in 0.79 seconds. Load scale 1.3 changed EENS from `0.149400` to `0.194220`
and from `15.162140` to `35.964740 MWh/year`; disabling six restoration
resource classes changed it to `1.656000` and `21.312060`. With automatic and
manual endpoints of `0.014940/0.334800` and `15.449294/267.580000`, the
factorized intelligent probability was exactly `0.6500736`, effective
automation probability was `0.58506624`, and measured joint EENS was
`0.1476607125` and `120.0668358520`, matching the affine total-expectation
identity within the predeclared `1e-8` tolerance.

The comprehensive case retained a signed duration attribution of
`-0.251539 MWh/year` and a control attribution of `+104.869081 MWh/year`.
This is not clamped: the derivation explicitly permits non-monotone
counterfactual attributions when a longer switching window shortens the
fixed-MTTR repair window or changes topology. The response localized negative
duration terms to `HV-Line-110kV` and `F3-Line-15-16`; total endpoint ordering
and the exact decomposition still passed. Both cases declare the Level-1
independence boundary, no joint class probabilities, and no protection/FRT
coupling. The test reused the existing `macos-release/tests/run_gui_server`
binary; CMake reconfiguration disclosed that local MIPSolvers HEAD
`82f4583d4f9070549c75a1ffa5d8d3bac8d9dc6f` differs from the recorded pin
`60f8bc4e4eeb58c239f83b7ff0fde1be75cd05b0`, so no reproducible rebuild or
full-suite claim is made for this increment.

The current dirty HySim worktree adds a model-bound reliability/protection
configuration across `failure_mode`, the GUI server, and `/xjtu/`. The catalog
now includes previously absent AC/DC/three-phase buses, transformers, bus-load,
regulator, dedicated DC storage, LCC, and three-phase component families. User
overrides are sparse and use stable component `.index` identities; explicit
protection zones and all backend schema fields round trip through the GUI.
Unsupported steady-state consequences and protection-zone targets remain
diagnostics rather than fabricated zero-impact support.

Focused verification used the existing Debug build directory with the local
dependency dirty-check override; no Release result is claimed:

```bash
cmake -S . -B /private/tmp/hysim_reliability_config_debug \
  -DCMAKE_BUILD_TYPE=Debug \
  -DHACDCDSS_SKIP_MIPSOLVERS_DIRTY_CHECK=ON \
  -DHACDCPF_ENABLE_ETAP=OFF \
  -DHACDCPF_ENABLE_IPOPT=OFF \
  -DHACDCPF_ENABLE_OPENDSS=OFF
cmake --build /private/tmp/hysim_reliability_config_debug \
  --target test_reliability_resolver run_gui_server -j4
```

`test_reliability_resolver` passed 53 cases and 347 assertions. Registered
CTest `reliability_configuration_e2e` passed in 40.09 seconds and verified
transformer and AC-branch overrides, resolved-value display for every returned
mode and all 12 numeric fields, sparse GUI save, all mode/protection fields,
invalid-input rejection, duplicate-name identity, same-model preservation,
component-array and replacement-model reset, explicit non-application by other
methods, custom protection probabilities, semantic property grouping across 27
mapped Canvas component kinds, a non-generator property-panel edit, and a
390 px layout after overriding a persisted 1180 px panel width. The focused
`topology_transformer_link_e2e` passed in 3.71 seconds and verified that both
standalone and branch-backed transformer rows select the correct Canvas glyph.
The automated comprehensive case covers the core AC/DC equipment families;
manual GUI inspection of that case showed 121 components, 266 modes, and all
3192 numeric cells populated with finite resolved values.
`node --check` passed for the changed JavaScript files. Browser inspection at
1440x1000 and 390x844 found no
page-level horizontal overflow; the large mode table uses bounded independent
scrolling; the calculation principles, all five execution stages, and the
protection editor remain reachable. Only the focused `macos-release` targets
described below were rebuilt; no complete Release/full-dev build, full CTest,
or sanitizer run was performed for this enhancement.

Three-stage restoration now consumes the same session protection rows without
claiming that failure-mode overrides apply. For each initiating fault it splits
frequency into successful-reclose, primary-cleared, backup-cleared, and
unresolved scenarios; configured clearing times enter Stage 1, the backup zone
expands the outage set, and primary-plus-backup failure blocks restoration.
Successful reclose is excluded from sustained IEEE 1366 indices, while the
result audits initiating, transient, and sustained frequencies and their exact
conservation. The GUI renders this protection/recovery audit and the governing
formula from the response.

Focused `macos-release` verification now passes 35 regular
`test_three_stage_reliability` cases with 1454 assertions. In addition to exact
`r=0.8` scaling of sustained EENS to 20% of baseline, primary/backup frequency
conservation, backup-zone expansion, and configured clearing times, the suite
now verifies coupled AC/DC nodal balance and voltage/radial constraints,
bidirectional VSC and DC-DC efficiency, DC line limits, four DC DER fault
families, AC/DC/mobile-storage stage chronology, mobile-storage transit state,
VPP PCC-boundary outages, and equal-number AC/DC bus-ID isolation. A direct
`build_dist33_microgrid_der()` regression returns `ok=true` with the coupled
scope and all branch-flow/voltage/radial/DC/VSC validity flags true. The
registered `reliability_configuration_e2e` and
`reliability_default_policy_e2e` passed in 39.39 and 7.16 seconds after
checking both unified and legacy API protection fields, strict-policy rejection,
unified fallback execution, and the empty-configuration JSON contract. A
direct-C++ timing probe on the
same arm64 macOS 26.5.2 host, Apple clang 21, `-O3 -DNDEBUG`, commit `493f6351`
used two warmups plus five measured repetitions per variant. Across three
independent runs, median no-configuration/primary/primary-plus-backup times were
`0.473--0.488 / 0.477--0.480 / 0.630--0.648 ms`; the two-scenario/single-
scenario ratio was `1.320--1.353x`, within the predeclared approximately `2x`
upper-order prediction because parsing, assembly, and healthy-state solves are
shared fixed costs. The hidden diagnostic is reproducible with
`./build/macos-release/tests/test_three_stage_reliability
'Protection scenario runtime probe'`; it is excluded from regular CTest.
Manual inspection of the rebuilt GUI loaded `dist33_tie_demo`, selected the
three-stage consequence model, and displayed its equation and five-stage
workflow. The follow-up browser regression closes a discovered policy mismatch:
`missing_only` now runs with the backend's declared unified fallback values,
while `case_data_only` alone blocks zero-coverage input and offers the explicit
`Apply parameter library and run` path. Selecting three-stage restoration now
defaults the GUI fault loop to serial execution, avoiding queue amplification
against the process-serialized solver while retaining an explicit parallel
opt-in. Earlier Dist33 GUI evidence in this paragraph predated the coupled DC
model and is superseded by the current source-level and workflow contracts
below. The obsolete empty-result text claiming that
the reliability and resilience backend interfaces were pending was removed.

The same dirty worktree now adds a reliability calculation workflow to the GUI:
method-specific principles and equations, five persistent execution stages,
catalog/protection coverage, and response-backed limitation states. Because the
unified endpoint has no stage-progress stream, the GUI labels enumeration,
consequence mapping, and solving as one backend interval instead of inventing
percent completion. The registered `reliability_workflow_e2e` uses the built-in
`cyber_physical_reliability_demo` case and passed with these directional EENS
checks (MWh/year): physical load scale `1.0 -> 1.5`,
`1.0000005 -> 4.30000075`; information availability `1 -> 0`,
`0.10000005 -> 4.400001`; dominant passive-mode failure frequency doubled,
`10.000005 -> 20.000005`. Its completed GUI stage states were
`complete/complete/limited/complete/complete`; the limited consequence state
was backed by runtime model declarations, and 390 px viewport overflow was
zero. After rebuilding the focused `macos-release` targets,
the registered `reliability_workflow_e2e` passed in 11.14 seconds and exercises
real three-stage requests for both
Dist33 variants: `dist33_microgrid_der` uses
`coupled-acdc-lindistflow-restoration-milp`, returns `ok=true`, signed VSC
dispatch, and true branch-flow/voltage/radial/DC/restoration validity flags;
`dist33_tie_demo` remains an exact AC case. The rebuilt registered
`reliability_configuration_e2e` and `reliability_default_policy_e2e` also
passed in 50.17 and 9.78 seconds. Both unified and legacy
three-stage routes execute the core solver on a dedicated 4 MiB-stack worker;
the workflow wall time is `1.21x` the previously recorded 9.20-second run,
inside the predeclared `3x` AC/DC-model expansion budget. This is an end-to-end
runtime comparison, not a solver-only microbenchmark. An additional
response-construction failure was traced under LLDB to a nullable
`const char*` passed to nlohmann JSON after a successful solve. Both routes now
construct an explicit JSON string or JSON null, closing the observed
`strlen(nullptr)` process termination for empty custom configurations. The
rebuilt `test_reliability_resolver` remained at 53 cases and 347 assertions. The broad
`gui_scale_features_e2e` transformer-link assertion also passed, but that run
was not green overall because two later, unrelated comprehensive-OPF checks did
not converge; no green result is claimed for that suite.

## Model parameter explorer

The dirty worktree extends `/api/session/parameter_library` and the Model
Parameters GUI with backend-owned presentation metadata. All 52 underlying
rules return symbols, quantities, model roles, equations, typical-range
provenance, and an equivalent-circuit family. Typical screening ranges are
explicitly separate from editable hard validation bounds. The backend now owns
a 44-entry catalog covering every physical/system family serialized by
`HybridPowerSystem`; the selector no longer depends on which families happen
to have standard-completion rules.

The current-system snapshot flattens all fields from `hacdcpf::io::to_json`,
uses domain-qualified stable identities, and groups them by engineering
semantics. Resolved failure modes are attached to their physical component by
`reliability_kind + component .index`; the legacy reliability rule groups stay
in the 52-rule JSON contract but are hidden as standalone models. The GUI
therefore shows 24 physical standard rules in the overview, complete read-only
instance fields, and reliability parameters within each matching instance.
Unknown typical/hard ranges are reported as unpublished/unregistered rather
than synthesized. Existing save/import/export/validate/apply behavior remains
intact; authored instance values remain editable through Canvas properties.

Focused Debug verification rebuilt `run_gui_server`. The registered
`parameter_contract_e2e` passed in 8.99 seconds with 44 catalog models, no
standalone reliability pseudo-models, complete field-contract checks, stable
Canvas selection, profile save round trips, and mobile overflow checks. The
registered `reliability_configuration_e2e` passed in 38.63 seconds. Real
browser inspection of `comprehensive_hybrid_acdc` found 108 stable instances;
the selected generator exposed 76 grouped rows, including 41 resolved
reliability rows, with no visible overlap. `node --check` passed for
`web/js/app.js` and the E2E script. No Release/full-dev build, full CTest, or
sanitizer run was performed for this GUI enhancement.

The same regression exercises every mappable instance family present in the
distribution case, same-number AC/DC buses, consecutive AC branches,
VSC/storage, transformer aliases, current-value refresh, unsaved profile-edit
preservation, and module/tab stability. Selecting a Canvas glyph while Model
Parameters is active follows the exact domain-qualified instance without
rebuilding or losing unsaved profile edits.

## GUI workspace density

The dirty worktree now provides a workspace-first GUI layout modeled on common
engineering simulation tools: standard and compact densities, independent
component-library/context-ribbon/inspector/console docks, a focus mode that
preserves the active module and inspector tab, and persisted layout state.
Desktop first load uses compact density with a collapsed console. At 720 px and
below the component library, contextual ribbon, console, and desktop-only
dependency chips start collapsed while global element location and the active
right-side parameter/result view remain reachable.

The registered Debug CTest selection passed 5/5:
`top_toolbar_semantics_test`, `workspace_layout_e2e`,
`topology_transformer_link_e2e`, `parameter_contract_e2e`, and
`reliability_configuration_e2e` (54.21 seconds total). At 1440x1000, focus mode
increased the measured Canvas area from 959x649 to 1115x919 while preserving the
short-circuit module, topology tab, and console contents. The layout round trip
survived reload; a real 390x844 Chromium check reported zero page-level
horizontal overflow and showed no overlapping controls. JavaScript syntax and
`git diff --check` passed. No Release/full-dev build, full CTest, or sanitizer
run was performed for this frontend-only change.

## Active module audit

The living [module code audit](module_code_audit.md) records the current
source-backed findings and audit depth. AUD-001 through AUD-011 are closed.
Focused runtime contracts now cover `graph/`, `scenario_generation/`,
`carbon_analysis/`, `integrated_energy/`, and `sppt/`; the canonical links are
in [docs/README.md](README.md).

Current-source Debug verification rebuilt all affected targets. The focused
and contract suites passed: typhoon traffic/catalog 5 cases/35 assertions,
campus IES 6/106, harmonics 52/359, EV Formulation D 15/196,
graph/Kron/round-trip 55/461, scenario generation/schema 12/105,
carbon snapshot/annual/GEC 41/574, SPPT 33/213, and the shared
resilience/reliability executable 39/360. `run_gui_server` also rebuilt and
linked successfully.

AUD-011 is closed by running StrictHiGHS restoration B&C calls on a fresh
joined thread while keeping Native on the caller thread, whose larger stack is
required by recursive sub-MIPs. The same-process regression completed five
Native-to-StrictHiGHS cycles. A 1000-cycle packaged-task/jthread benchmark
measured 0.0145--0.0187 ms fixed overhead per cycle, below the 10 ms threshold.
No full CTest, complete sanitizer suite, or external-engine cross-validation
was run specifically for this closure pass.

## Closed investigation

The HySim worktree currently contains uncommitted dependency, documentation,
graph, reliability, and graph-test changes. Re-run `git status` before relying
on this list.

- `src/graph/topology_analysis.cpp` fixes an invalidated stack-frame reference
  in iterative Tarjan traversal and preserves the exact parent edge for
  parallel-edge correctness. Focused ASan/UBSan tests pass.
- `src/reliability/three_stage_reliability.cpp` serializes process-global HiGHS
  and native B&C solve state across contingency workers; the native fallback
  uses one internal thread. Solver-bearing reliability workers request a 4 MiB
  POSIX stack through the optional `ThreadPool` stack-size parameter.
- At that closure point, the sibling MIPSolvers worktree was clean at
  `5a5fad0`. Earlier local HiGHS experiments are not pending worktree state.

The apparent `HPresolve::changeImplColUpper` container corruption in
`case33mg_acdc` was a downstream symptom of worker stack overflow, not a
presolve data-structure defect. On Darwin, the default pthread stack is about
512 KiB, while sanitizer instrumentation expanded the largest observed solver
frame to about 340 KiB. The same case completed on the main thread, and the
parallel case became stable after provisioning a 4 MiB worker stack. Five
consecutive sanitizer repeats completed in 414.63 seconds. The resulting
case33mg metrics were SAIFI 0.7833, SAIDI 10.87, and EENS 588.5; the case33bw
cross-check produced SAIFI 0.7787, SAIDI 10.60, and EENS 570.3.

## Performance closure

An earlier pre-cleanup MIPSolvers Release experiment compared the modified
binary with a clean export of the pinned commit on the same arm64 host using an
A/B/C sandwich: 24 NETLIB cases, Native and HiGHS, 3 repeats, single-threaded
HiGHS, and a 30 second solve limit. Every run was accurate (72/72 for each
solver in each leg). This is retained as historical performance evidence, not
as a statement that the current sibling worktree is dirty.

| Leg | Native geometric mean |
|---|---:|
| Clean A | 3.761956 ms |
| Dirty B | 3.761257 ms |
| Clean C | 3.763764 ms |
| `sqrt(A * C)` control | 3.762860 ms |

Dirty versus sandwich control was -0.0426%, within the predeclared absolute
1% acceptance threshold. After timing fields were removed, the JSON run
records matched exactly across A/B/C, including status, objective, feasibility,
iterations, and dual-pivot counts. The raw reports are
`/private/tmp/hysim_native_dual_{control_a,experiment_b,control_c}_repeat3.{csv,json}`.

## PV/PQ reactive-limit switching

Balanced Newton PF now performs generator Q-limit conversion only around
fixed-active-set converged points. All violated PV buses enter their Q bounds
in one batch. PQ-to-PV restoration is audited at most once and requires the
configured hold count plus the existing voltage-direction margin. Converged
active-set signatures, outer-budget exhaustion, final Q violation, and an
explicit certificate are returned. The GUI defaults to fast screening with
conversion disabled and labels Q-limit certification separately.

Smooth NCP uses CHKS median smoothing for PV/PQ and priority projections plus
Kanzow smooth-FB for magnitude-priority VSC blocks. Continuation owns a local
`SolverData` copy, initializes `mu` before the first evaluation, reassembles
residual and Jacobian after every reduction, and cannot terminate until
`ncp_mu_min` is reached. VSC continuation can be enabled without the generator
PV/PQ NCP option. Public converter certificates are re-evaluated against the
exact `mu=0` equations.

Release verification rebuilt `test_power_flow_math_audit`,
`matpower_pf_compare`, and `run_gui_server`. The full math audit passed 50 cases
and 305 assertions. The ASan/UBSan build passed its configured 42 cases and 260
assertions; LeakSanitizer is unavailable in the macOS runtime. JavaScript syntax
and `git diff --check` passed.

On `case6515rte`, Q-limit enforcement converged in 9 Newton iterations and 8
factorizations, with 74 PV-to-PQ switches, no restoration, no repeated active
set, and a certified maximum Q violation of `4.22e-11 pu`. The ordinary-layout
five-repeat Release run measured `56.4--61.2 ms` with a `58.7 ms` median; the
fixed-layout comparison is recorded in the flat-start follow-up below.
Fast screening previously used 4 iterations and 3 factorizations with a median
of approximately 21.2 ms.

The original `data/云南案例.json` exposed a distinct input/projection defect,
not active-set oscillation. It contains 587 legacy BPA zero-data L-card ties
encoded as physical `x=1e-4 pu` branches. Conflicting PV setpoints across those
rows produced `O(Delta V/x)` reactive circulation; the old solve reached about
`410 pu` residual and `8736 pu` maximum Q violation without repeating an active
set. BPA import now marks exact-zero L cards as ideal connectivity. JSON round
trip preserves that provenance, and a strict BPA-schema migration recognizes
the legacy encoding while excluding ordinary short physical lines. Canonical
projection performs 585 effective merges, reducing 4512 authored buses to 3927
canonical buses before Ybus and PV/PQ construction.

With the unmodified Yunnan JSON loaded through the rebuilt Release GUI server,
Q-limit enforcement converged and certified in 19 total Newton iterations and
5 outer solves: 218 PV-to-PQ entries, 3 restorations, 215 final limited buses,
no repeated active set or cycle, `7.28e-12` electrical residual, and
`2.27e-13 pu` maximum Q violation. Cold solver/presentation/total-before-
serialization timings were `71.9/224.2/296.8 ms`; five warm compact Q-limit
solves had a `47.3 ms` median. Q-disabled screening measured `18.3 ms`.

Release verification additionally passed `test_io_json` (41 cases, 398
assertions, one data-dependent skip), `test_bpa_io` (25/885),
`test_component_models_math_audit` (15/111), `test_power_flow_math_audit`
(49/304), and `test_advanced_pf` (30/218). The rebuilt `run_gui_server` is the
binary used for the full Yunnan request.

The production-HTTP Yunnan capability audit is reproducible with
`tools/validate_yunnan_capabilities.py`; its merged machine-readable and human
reports are `output/yunnan_capability_audit.json` and `.md`. It covers 46
probes without inventing missing case data: 14 passed, 6 ran with an explicit
input limitation, 20 were correctly classified not applicable, 3 returned a
numerical non-convergence, and 3 exhausted a declared wall budget.

The PF comparison confirms that the fixed active-set Newton path is the right
production default for this case. Screening used `16.0 ms` solver time and 7
iterations. Q-certified Newton used `42.6 ms`, 19 total iterations, 218
PV-to-PQ entries, 3 restorations, and no cycle. Distributed slack (`1081 ms`),
HELM (`443 ms`), and Newton-Krylov (`378 ms`) converged but were materially
slower. FDPF stopped after 1000 iterations without convergence; explicit
homotopy stopped after 110 iterations at residual `1.35` and an uncertified Q
violation. DC and hybrid-linearized PF remain useful screening models, not
substitutes for the nonlinear Q certificate.

Parity OPF with AC-PF warm start converged in 37 iterations and 36
factorizations both with and without central Phase I. Phase I measured
`0.66 ms`, observed initial constraint violation `0.2199 > 0.1`, terminated
`outside-phase-one-admission`, and correctly left the ordinary Phase-II start
unchanged. Core solve time was `676.7 ms` with Phase I and `675.3 ms` without;
the difference is noise. DCOPF completed in `217 ms` core time. Because the
case has no authored generator cost curves, these runs certify feasibility and
solver work, not economic fidelity.

Topology, projection guard, network reduction, harmonics zero-injection,
hosting capacity, carbon-flow plumbing, and scenario generation all returned
normally. The system is one connected AC island with 673 cycles. General graph
reduction eliminated 143 buses/branches in `159 ms`; SPPT admission passed.
Hosting capacity evaluated 2428 BPA transformer branches in `27.7 ms`.
Carbon tracing solved a rank-3927 canonical matrix with zero reported relative
residual in `11.35 s`, but the result uses fallback factors because no authored
carbon factors exist. Scenario generation took `2.36 s` and produced 6344
default reliability contingencies; those defaults are workflow evidence, not
Yunnan stochastic-data evidence.

The Yunnan short-circuit and carbon-flow scale failures are closed in the dirty
worktree. Detailed sequence matrices are sparse, KLU factors are shared across
batch fault locations, selected inverse columns replace full `Zbus` matrices,
and inverse diagonals use bounded RHS blocks with cooperative cancellation.
The GUI all-bus route now uses its documented positive-sequence overview model;
the selected-fault route computes complete fault-point duties and the full
remaining-voltage profile without unused non-fault current diagonals. Release
HTTP wall times were `74.2 ms` for 4512 overview rows and `29.7 ms` for one
detailed fault, versus both former paths exceeding 90 seconds. Carbon tracing
and nodal intensity now remain sparse throughout; the same case fell from
`11.35 s` to `1.469 s`, with rank 3927, relative residual zero, finite condition
estimate `5.173`, and all validity flags true. Focused Release tests passed 523
short-circuit assertions, 8 IEC assertions, and 594 carbon assertions.

Two scale failures remain open. Default counterfactual planning
exceeded 180 seconds and ignored cancellation because it nests baseline, five
measure, and up to ten pair evaluations across economic, carbon, reliability,
and resilience models. It needs a shared prepared assessment session, bounded
dimension selection, and cancellation at every clone/measure/pair boundary.
Finally, the four-step default time-series profile returned voltage arrays but
zero converged steps; all four minima hit `0.05 pu`. The case has no authored
load or generator profiles, so the default `0.4--0.5` load multipliers are not
an admissible Yunnan operating trajectory and must not be presented as a
successful production simulation.

The GUI/API E2E passed all PF Q-certificate and fast-screening assertions, but
the complete run was 70/71 because the separate Hybrid Auto OPF check reached
the Ipopt iteration limit. Browser E2E passed the new default/control assertion
and all large-system PF checks; it remained non-green because two existing
multiscale comprehensive-OPF assertions failed. No all-suite green claim is
made for those OPF paths.

The 82,000-bus `case_SyntheticUSA.m` exposed a separate GUI orchestration
regression. The `ac_newton` route detected three solvable AC islands, ran the
adaptive island solver, and then ran a second full-network Newton solely to
populate display/cache data. The second solve has been removed. Adaptive
results now aggregate per-island Q-limit certificates and profiling and are
used directly for branch-flow reconstruction, compact presentation, and PF
cache state. The exact Release GUI request with PV/PQ conversion enabled
converged in 2.58 seconds with 4,820 PV-to-PQ switches, 76 restorations, no
repeated active set, and maximum Q violation `1.34e-11 pu`. The former double
solve took approximately 5.80 seconds on the same server. Focused Release
adaptive tests passed 30 cases/218 assertions; the ASan/UBSan integration and
adaptive selection passed 4 cases/23 assertions.

## Power-flow flat-start robustness follow-up

The committed PF follow-up closes two correctness defects from the large-case
PF evaluation. Reactive-limit certification now uses
`max(1e-10, PowerFlowOptions::tol)`: the limited-bus Q violation is the
remaining Q-row mismatch and cannot be certified more tightly than the root
that produced it. The focused regression deliberately converges with a
violation above `1e-10` and below `1e-3`, and remains certified. The optional
Levenberg-Marquardt recovery no longer factorizes `J^T J + lambda I` through
the symbolic analysis for `J`; it owns a separate sparse solver, analyzes its
normal-equation pattern once, reuses that pattern across damping attempts, and
reports its analyses, factorizations, solves, and elapsed linear time.

Default line-search Newton now detects only high-residual stagnation: the best
residual must fail to improve by 10% over 15 iterations while remaining above
`1.0`. Near-root damped progress retains the caller's full iteration budget.
For a structurally closed numerical failure, the public default escalation is
now deterministic: direct fixed-layout Newton, semi-smooth NCP, linearized-DC
angle seed plus fixed-layout active set, DC-seeded NCP, then homotopy as the
final fallback. Internal retries own their `SolverData` and `NewtonSolver`, do
not recurse through the facade, preserve the caller's Q-enforcement semantics,
and aggregate their work/provenance in `SolverProfiling`. LCC transformer tap
control retains its dedicated outer loop and is not bypassed by direct retries.

`enable_fixed_pv_pq_layout` is now the C++ default. It embeds every non-slack
AC bus's `Vm/Q` coordinates in one stable canonical bus-position order. PV rows
become `Vm` identity rows while Q-limited rows retain physical Q balance; the
outer active-set timing and certificate semantics are unchanged. Semi-smooth
NCP remains mutually exclusive and wins with an explicit warning when both
options are requested. Row nonzero indices are precomputed once, making
active-row replacement `O(nnz(row))` rather than a full compressed-matrix scan.

On AppleClang 21 arm64 macOS, Release/KLU, working tree `3cc92658373a`,
`case6515rte` reduced Jacobian pattern rebuilds/analyses from `3/3` to `1/1`
with unchanged 9 Newton iterations, 8 factorizations, 74 PV-to-PQ switches,
and Q certificate (`4.22e-11 pu`); five-repeat median solve time was
`58.7 -> 40.9 ms` (`30%`). `case_ACTIVSg10k` reduced `10/10 -> 1/1` with
unchanged 26 iterations, 25 factorizations, 1151 entries, 40 restorations,
and certificate; its median was `261.1 -> 152.0 ms` (`42%`). The default fixed
layout does not claim to reduce the number of outer active-set solves.

The hard flat-start acceptance set is closed without homotopy tuning. Release/
KLU default solves converged and Q-certified as follows: `case1888rte` used the
DC-angle/fixed-active-set stage in 21 iterations, `case3375wp` used semi-smooth
NCP in 9, `case6468rte` used DC-angle/fixed in 21, `case6515rte` used
DC-angle/fixed in 30, and `case_ACTIVSg10k` used DC-angle/fixed in 29. None
reached homotopy. Flat-start `case118` stayed on direct Newton and converged in
18 iterations both with and without the outer ladder; warm `case3375wp` stayed
at 7 iterations and warm `ACTIVSg10k` at 26, with zero escalation attempts.

`PreparedPowerFlowSession` now caches canonical projection, `SolverData`, the
fixed Jacobian pattern, and symbolic analysis across compatible solves. A
same-layout authored load/setpoint change refreshes numeric values without
projection or assembly; topology/network-parameter changes rebuild
conservatively. On `case6515rte`, five Release repeats had medians of
`28.962 ms` through the ordinary facade and `16.463 ms` through one prepared
session (`43.2%` lower); prepared repeats reported zero projection, assembly,
and symbolic-analysis work.

The PF linear path now requests KLU numeric refactorization only after an
existing factorization with an unchanged pattern. Every candidate solve is
guarded by a normwise backward-error audit and falls back to fresh pivoting on
failure or excess error. On warm `case_ACTIVSg10k`, a five-repeat Release
protocol reduced median total time from `154.032` to `104.024 ms` (`32.5%`)
and final-solve linear time from `98.549` to `54.699 ms` (`44.5%`); all 24
refactors were accepted and maximum backward error was `2.29e-17`. A zero
tolerance regression forces every candidate through the full-factorization
fallback and still converges.

Profiling now covers residual evaluation, Jacobian scaling, active-set scans,
projection, assembly, result derivation, solver-core/facade totals, unclassified
remainders, prepared-session reuse, and escalation provenance. The W4 sweep is
also closed: FDPF voltage division uses the configured floor, GFM derivatives
use `min_vm_pu`, the Newton-Krylov Schur preconditioner reuses symbolic analysis,
and the standalone DC solver uses an analytic sparse Jacobian/SparseLU instead
of dense finite differences and `FullPivLU`.

Release/KLU focused verification passed `test_power_flow_math_audit` 50 cases/
305 assertions, `test_advanced_pf` 30/218, `test_homotopy_continuation` 5/36,
`test_newton_krylov` 5/30, `test_hacdcpf` 27/117, and the flat-start/KLU
MATPOWER filter 3/45. ASan/UBSan with SuiteSparse disabled and leak detection
disabled passed 43/261, 30/218, 5/36, 5/30, the prepared-session filter 1/28,
and the MATPOWER filter 3/41. The complete `macos-release` CTest then ran all
1492 registered tests: 1482 passed, 7 failed, and 3 condition-dependent tests
were skipped. Every PF/MATPOWER test passed. The seven failures were confined
to existing OPF/RPO boundaries: multiscale/case300/GUI Auto Ipopt iteration
limits, one authored-order OPF value at `10.0100145` just outside a `10.0 +/-
0.01` assertion, the GUI multiscale/Auto OPF checks, and the case300 RPO
cross-validation tolerance. No all-suite green claim is made. The main
repository implementation is at `684de1ee`; the clean MIPSolvers dependency is
at the pinned `3bf1e66`.

## Fast orientation

```bash
git status --short --branch
git -C ../MIPSolvers status --short --branch
cmake --build --preset full-dev
ctest --preset full-dev
cmake --build --preset macos-release
ctest --preset macos-release
```

Use [README.md](../README.md) for the user-facing capability baseline,
[AGENTS.md](../AGENTS.md) for architecture and invariants, and
[docs/README.md](README.md) for topic documentation.
