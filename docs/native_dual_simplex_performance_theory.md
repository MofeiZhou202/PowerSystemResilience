# Native dual simplex performance: theory and experiment contracts

## 1. Scope and acceptance criterion

This document derives the theory guiding the remaining performance work on
the Native dual simplex (`src/engine/kernel/lp_kernel/native_dual/`). It
plays the same role for the dual iteration that
`docs/native_primal_pricing_theory.md` plays for the primal cleanup: every
future change must implement a contract stated here, and no benchmark win
substitutes for the invariants.

The project-wide target is unchanged: 72/72 accurate runs over the 24-case
NETLIB suite and a geometric speedup greater than 1.0 over one-thread HiGHS
Simplex in the repeated Release benchmark. After E1, the 2026-08-01 ratio is
0.785x (`reports/netlib_e1_shift_start_repeat3.json`), with 72/72 accurate
runs.

Changes divide into two acceptance classes:

- **Class P (path-identical).** Every floating-point operand, operation
  order, branch outcome, counter, and published decision is unchanged; only
  wall time changes. Verified by the differential path contract (identical
  pivots, rebuilds, phase iterations, and BFRT telemetry on the six recorded
  contract cases).
- **Class A (algorithmic).** Pivot paths may change by design. Verified by
  1,166 mathematical assertions, 224 NETLIB regression assertions, 72/72
  benchmark accuracy, no new fallback/cleanup/phase-transition class in
  telemetry, and a re-recorded path contract afterward.

## 2. Measured decomposition of the HiGHS gap

Total solve time factors as

\[
  T = \sum_{s\in\{DI,DII,PI,PII,PC\}} N_s\bar c_s
    + N_Rc_R + c_{\text{init}}.
\]

with \(N_I, N_{II}\) the dual Phase I/II pivot counts, \(\bar c\) the mean
pivot costs, and \(N_R, c_R\) reinversion count and cost. E1 makes that
decomposition incomplete: restoring the original cost may invoke primal
cleanup, and a failed dual path may invoke a full primal Phase I/II fallback.
The complete iteration identity is

\[
  N = N_{DI} + N_{DII} + N_{PI} + N_{PII} + N_{PC}.
\]

`MIPSOLVERS_DS_PROFILE` now publishes every term. The E1/legacy diagnostic is:

| case/policy | dual I | dual II | primal I | primal II | primal cleanup | total |
|---|---:|---:|---:|---:|---:|---:|
| grow7 / E1 | 0 | 179 | 0 | 0 | 173 | 352 |
| grow7 / Dual-I | 21 | 464 | 0 | 0 | 0 | 485 |
| fit1p / E1 | 0 | 0 | 0 | 0 | 541 | 541 |
| fit1p / Dual-I | 223 | 636 | 0 | 0 | 0 | 859 |
| stocfor1 / E1 | 0 | 54 | 0 | 0 | 39 | 93 |
| stocfor1 / Dual-I | 125 | 9 | 0 | 0 | 0 | 134 |
| d2q06c / E1 | 0 | 2,057 | 0 | 0 | 7,797 | 9,854 |
| d2q06c / Dual-I | 1,423 | 4,186 | 0 | 0 | 9 | 5,618 |
| degen2 / E1 | 0 | 569 | 0 | 0 | 333 | 902 |
| degen2 / Dual-I | 134 | 470 | 0 | 0 | 0 | 604 |
| degen3 / E1 | 0 | 1,861 | 0 | 0 | 1,442 | 3,303 |
| degen3 / Dual-I | 465 | 1,617 | 0 | 0 | 0 | 2,082 |
| pilot4 / E1 | 0 | 356 | 0 | 0 | 693 | 1,049 |
| pilot4 / Dual-I | 90 | 427 | 0 | 0 | 0 | 517 |
| grow22 / E1 | 0 | 810 | 0 | 0 | 533 | 1,343 |
| grow22 / Dual-I | 66 | 660 | 357 | 529 | 0 | 1,612 |

The resulting regimes are:

- **E1 success** (`grow7`, `fit1p`, `stocfor1`, `sc205`): shifted Phase II
  plus cleanup costs fewer pivots and improves suite time.
- **Original-cost cleanup domination** (`d2q06c`, `degen2`, `degen3`,
  `pilot4`): E1 removes Dual Phase I but creates a larger primal cleanup bill.
  This is a start-policy problem, not a dual ratio-test problem.
- **Per-iteration cost** (`scsd8`, `25fv47`): substantial wall gaps remain
  after accounting for paths. Density-adaptive kernels remain candidates.
- **Resolved reconstruction defect** (`grow22`): the former numerical
  fallback came from reusing an audit across a changed nonbasic side vector.
  Section 6.5 gives the dependency proof and retained correction.

Any proposed change must name the regime it addresses and the case set whose
statistics it is required to move.

## 3. Dual degeneracy: perturbation and minimum-step theory

### 3.1 Degenerate dual steps and stalling

In the internal maximization convention, a dual pivot moves the duals by
step \(\theta_D \ge 0\) along the direction defined by leaving row \(p\).
The step is bounded by the first breakpoint
\(\theta_j = g_j / \alpha_j\) over eligible columns, where
\(g_j = \max(0, -s_j r_j)\) is the dual margin. A pivot is **degenerate**
when the binding margin is zero: \(\theta_D = 0\), the dual objective does
not improve, and only the basis changes. Long runs of degenerate pivots
(stalling) are the classical explanation for iteration counts far above the
nondegenerate bound; `sc205` (162 of 230 steps degenerate) is a textbook
instance.

Two independent, composable remedies exist; both are Class A.

### 3.2 Cost perturbation

Perturb the working cost \(c_j \to c_j + \delta_j\) before Phase II so that
exact ties in the margins \(g_j\) become resolved at magnitude
\(|\delta_j|\).

**Correct feasibility-preservation lemma.** For a fixed basis,

\[
  r_N = c_N-A_N^TB^{-T}c_B,
  \qquad
  \Delta r_N=\delta_N-A_N^TB^{-T}\delta_B.
\]

Let \(s_j\) be the stored move direction, so dual feasibility is
\(s_jr_j\le\tau_D\). Preservation requires
\(s_j(r_j+\Delta r_j)\le\tau_D\). The simpler sign rule
\(s_j\delta_j\le0\) is sufficient only when basic costs are not perturbed
(\(\delta_B=0\)). Since the production perturbation visits enterable basic
and nonbasic columns, its contract must be stated on projected reduced-cost
changes \(\Delta r\), followed by reconstruction and audit. A raw per-column
cost sign is not a proof for the current implementation.

**Correct magnitude contract.** A bound on \(|\delta_j|\) alone cannot bound
the original-cost reduced-cost defect because the basis projection may
amplify \(\delta_B\). The required terminal quantity is

\[
  \|\delta_N-A_N^TB^{-T}\delta_B\|_\infty.
\]

Nor is cleanup pivot count bounded by the number of active perturbations:
simplex may require multiple basis exchanges to recover original-cost
optimality. Cleanup pivots remain an observed acceptance statistic, not a
theorem.

**Current implementation and diagnosis.** Native uses structural base
\(5\cdot10^{-7}\max|c|\) (with fourth-root compression) and logical base
\(10^{-12}\). The local HiGHS implementation uses the same base constants,
subject to its multiplier and boxed-rate adjustment. Therefore the former
claim that Native is one or two decades below HiGHS, and the proposed
\(10^{-5}\) parameter grid, are rejected. E2 is blocked until a trace shows a
remaining production-path degeneracy problem after E1; no perturbation-scale
experiment is currently authorized.

### 3.3 Minimum-step (EXPAND-style) hypothesis

Perturbation resolves exact ties; it does not prevent stalls where many
margins are merely *near* zero. An EXPAND-style mechanism may admit bounded
working infeasibility to escape such stalls, but it cannot guarantee a
strictly positive stable step when no eligible pivot exists.

- maintain a working dual tolerance \(\tau_k = \tau_0 (\beta + k\gamma)\)
  with \(\beta \approx 0.5\), \(\gamma\) chosen so \(\tau_k\) grows by less
  than one decade between reinversions;
- Harris pass 1 computes the relaxed bound with \(\tau_k\); pass 2 admits
  any stable pivot whose ratio is within the relaxed bound and then takes a
  step of at least \(\theta_{\min} = \gamma\tau_0 / |\alpha_{\max}|\), even
  when the exact minimum ratio is smaller — the overshoot is a deliberate,
  bounded dual infeasibility absorbed by the working tolerance;
- at INVERT, reconstruct and audit the working reduced costs and reset the
  expansion budget only if the reconstructed state satisfies its contract.

**Required contract before implementation.** Define the per-pivot and
cumulative working dual-infeasibility budget, prove how every overshoot changes
reduced costs, specify the no-stable-pivot outcome, and audit reconstruction at
INVERT. An overshoot is not automatically equivalent to a cost shift. The
terminal audit must still use original costs and the unexpanded tolerance.
This experiment remains blocked.

## 4. Phase-I avoidance: the cost-shifted dual start

### 4.1 Why the subproblem Phase I stalls on small models

The legacy diagnostic cold start runs the HiGHS-style subproblem Phase I: in
homogeneous coordinates \(z = x - x_0\), a lower-only column gets the box
\([0,1]\) and a boxed or fixed column gets \([0,0]\). On models dominated by
boxed/fixed columns (`sc205`, `stocfor1`), almost every subproblem variable
is frozen at a width-0 box: the Phase-I polyhedron is maximally degenerate
by construction, and the measured 70% degenerate-step share on `sc205` is
the direct consequence. HiGHS avoids this cost most of the time because its
perturbed start is already dual feasible and Phase I is rarely entered.

### 4.2 The shift-start alternative and its bound

**Lemma (universal dual-feasible start).** For any basis \(B\) and any
bound-side selection, shifting the working cost of every dual-infeasible
one-sided column by \(\sigma_j = -r_j\) (exactly zeroing its reduced cost)
and flipping the side of every dual-infeasible boxed column produces a
working-cost iterate that is dual feasible. The number of shifts is bounded
by the number of dual-infeasible one-sided columns of the initial basis,
and each shift is removed by the mandatory original-cost cleanup already
required of every result.

Dual simplex then starts directly in Phase II against primal
infeasibilities, with the original-cost defect re-introduced only at the
audited cleanup — precisely the architecture this project already validated
("cost-shifted dual crash", doc of 2026-07-31, change 1) and for which a
unit test still exists (`test_dual_simplex.cpp:210`,
`initialize_cost_shifted_dual_start`).

### 4.3 E1 result and policy diagnosis

E1 is implemented on the production cold path. It passes 1,166 mathematical
assertions, 224 NETLIB regression assertions, and 72/72 repeated benchmark
runs. Native geometric mean improves from 4.840 ms with E1 disabled to
4.395 ms with E1 enabled; the run-local ratio versus HiGHS is 0.785x.
`MIPSOLVERS_DUAL_SHIFT_START=off` retains the legacy path for diagnosis.

The universal-start lemma proves dual feasibility, not performance. For E1,

\[
  N_{E1}=N_{DII}^{work}+N_{PC}^{original},
\]

whereas the alternative includes Dual Phase I/II and may include primal
fallback stages. E1 wins only when the complete measured bill is smaller.
The initial shift count does not bound \(N_{PC}\): one shift produces 224
cleanup pivots on `sc205`, four produce 693 on `pilot4`, and 1,651 produce
7,797 on `d2q06c`. Consequently the proposed shift-count-only policy
predicate is rejected. No hybrid selector is authorized until a pre-solve
feature predicts cleanup work outside these 24 training cases.

## 5. Boxed-model long steps: diagnosis closed

The BFRT breakpoint rule remains the minimizer of the piecewise-linear dual
objective along its chosen pivotal row. Before E1, `grow7` required 485 Native
pivots versus 353 in HiGHS, motivating objective-gain and entering-column
traces. E1 changes Native to 179 working-cost dual pivots plus 173 original-
cost primal cleanup pivots: 352 total versus HiGHS's 353.

Thus the excess was caused by the cold-start/phase path, not demonstrated
BFRT step quality. The planned objective-gain and first-50-entering traces are
cancelled because their precondition (a remaining pivot excess) is false.
`grow22` also does not reopen this diagnosis: after correcting the stale
reconstruction-audit dependency, both the E1 and forced Dual-Phase-I paths
reach the published optimum. The former fallback was not evidence against
BFRT group selection. No ratio-test change is authorized.

## 6. Per-iteration cost theory

E1 invalidates the former `fit1p` and `d2q06c` kernel diagnosis. `fit1p` now
runs entirely as 541 primal cleanup pivots and is faster than HiGHS on the
formal run (about 16.5 versus 18.3 ms). `d2q06c` is dominated by 7,797 primal
cleanup pivots, so optimizing dual PRICE cannot address its production-path
bottleneck. The remaining dual-kernel targets are `scsd8` and `25fv47`, where
wall time remains high after path effects are accounted.

### 6.1 PRICE representation experiments (Class A, rejected)

Row-wise PRICE costs
\(C_r \approx c_s \sum_{i \in \text{supp}(\pi)} \text{nnz}(A_{i\cdot})\)
with a stamped-accumulator constant \(c_s\) that includes a data-dependent
branch per term. Column-wise PRICE against a dense scatter of \(\pi\) costs
\(C_c \approx c_d\,\text{nnz}(A) + c_g n\) with a branchless streaming
constant \(c_d\) (no stamps or touched list). The inequality \(c_d<c_s\) is
an empirical hypothesis, not a structural guarantee.
Support density is only a proxy for the row work and is invalid on matrices
with nonuniform row lengths. Any future selector must instead use the exact
\(W_r=\sum_{i\in\operatorname{supp}(\pi)}\operatorname{nnz}(A_{i\cdot})\).
An online per-solve estimator of \(c_s\), serial CSC cost, and parallel CSC
cost was implemented without density or size thresholds, but the calibration
and per-pivot clock reads regressed the fixed 24-case cohort by 4.1%. It is
rejected. Production PRICE therefore remains CSR-only. The forced CSC kernels
remain solely for controlled measurement and rounding-envelope tests.

**Determinism contract.** Column-wise accumulation fixes the summation order
to A's column storage order — deterministic, but different from the row-wise
order, so results differ in rounding and paths may change: Class A. A future
selector must not make the algorithmic path depend on noisy wall-clock reads.
HiGHS-style row-wise sparse-to-dense result handling tied to structural output
saturation was also tested: its O(n) materialization/export cost exceeded the
saved stamp work and regressed the cohort by 2.4%. E3 is closed unless a new
kernel removes that traffic term rather than retuning the switch condition.

Three Class-P traffic reductions were then tested without dispatch rules.
Shrinking the BFRT candidate records and delaying taboo lookup removed hot
record fields and non-Harris hash probes, but an A/B/A run on `d2q06c`
regressed its median by 2.1%. Reusing `result.index` as PRICE's touched list
made index writes non-increasing, yet the fixed cohort regressed from 4.277 to
4.453 ms and from 0.737x to 0.720x versus HiGHS. Finally, a HiGHS-inspired
value-marked accumulator removed every per-term stamp read and the entire
stamp array. A signed-zero marker preserved unique indices across exact
cancellation, unlike a literal `value == 0` first-touch test. Its cohort result
was still 4.329 ms and 0.733x. All three implementations were removed.

This sharpens the requirement for a future PRICE kernel: a byte-count proof is
necessary but not sufficient. It must also reduce the dynamic instruction
stream or replace irregular accesses with demonstrably cheaper locality. A
removed load that introduces a branch, sign test, compaction dependency, or
less favorable cache ownership is not presumed beneficial.

### 6.2 Dense scan layout above the density threshold (classification open)

The BFRT candidate scan and commit traversals are index-indirect walks of a
dense packed vector. A natural dense column-order scan changes enumeration and
floating-point order, so it is Class A. Preserving packed support order may be
Class P, but retains the indirect enumeration and therefore does not inherit a
branchless dense scan's claimed locality automatically. No strict traffic
reduction is proved. Prototype both kernels and classify the retained form by
its actual operation order.

### 6.3 Reinversion interval (Class A, measurement-gated)

Under the diagnostic model that solve cost grows linearly by \(u\) after each
Forrest-Tomlin update, rebuild cost \(R\) gives average overhead
\(R/T + u(T{-}1)/2\), minimized at \(T^* = \sqrt{2R/u}\). Real fill and solve
cost need not be linear, so \(T^*\) is a fitted estimator, not a general
simplex theorem. The current policy is the clamp
\(\max(50, \min(200, m/4))\) plus HFactor fill advice; on `grow7` the
effective scheduled interval is 50 pivots. The E4 estimator has the following
contract:

1. one observation is all indexed HFactor solves performed by one dual minor
   iteration, including pivotal BTRAN/FTRAN, BFRT FTRAN, and DSE FTRAN;
2. observations are averaged at each exact `updates_since_rebuild` age before
   OLS, so frequently visited ages do not dominate the slope;
3. Dual Phase I and Phase II are fitted separately;
4. wall \(R\) is the full `major_rebuild` cost. Synthetic \(R\) is HFactor's
   build tick only and is reported as `R_build_tick`; dense reconstruction
   has no synthetic-clock value;
5. only `UpdateLimit` reinversions are scheduled samples. Numerical trouble,
   terminal certification, and HFactor `SyntheticWork` fill advice remain
   safety events and cannot be delayed by an interval policy.

Three-repeat measurements make the distinction between evidence types
important. Synthetic-work slopes are deterministic and fit Phase II strongly
(`scsd8`/`25fv47`/`degen3`/`d2q06c` \(R^2=0.94\)-0.98), proving that
solve work grows approximately linearly with update age. However, HFactor's
fixed synthetic weights are not calibrated to the relative build/solve wall
cost on this machine, so synthetic \(T^*\) is not an absolute interval
prediction. The wall estimator is the decision metric; synthetic work
validates its shape.

Measured Phase-II `(current interval, T_wall, T_tick)` values are
approximately `scsd8 (99, 89, 52)`, `25fv47 (168, 94, 54)`,
`degen3 (200, 141, 93)`, `d2q06c (200, 133, 72)`, and
`grow7 (50, 72, 48)`. No wall estimate differs from the current interval by
the required factor of two. E4 therefore retains the existing policy. A future
measurement may reopen it only if the same gate is met on cases totaling more
than 10% of suite time; the gate must not be replaced by a model-size or
case-specific threshold.

### 6.4 Dependency-sliced INVERT (E6, Class P)

For fixed basis \(B\), nonbasic side vector \(s\), and working cost \(c\), a
reconstruction is the product of two independent maps:

\[
 x_B(s)=B^{-1}(b-A_Nx_N(s)),\qquad
 r(c)=c-A^TB^{-T}c_B.
\]

The objective \(c^Tx\) depends on both maps but is a dot product after they
have been updated. `major_rebuild` formerly evaluated both maps after each
of these transitions:

1. INVERT, with \((B,s,c)\) fixed: full primal and dual reconstruction;
2. boxed-column side classification, changing only \(s\): full reconstruction;
3. one-sided cost shifts, changing only \(c\): another full reconstruction.

The dependency graph proves that transition 2 requires only the primal map
and transition 3 only the dual map. This removes one BTRAN plus dense
\(A^Ty\) PRICE on every rebuild, and, when a cost shift occurs, one FTRAN plus
canonical-residual correction. It does not change pivot selection, tolerances,
rebuild scheduling, perturbation magnitude, or any model-dependent policy.

The floating-point claim is stronger than real-arithmetic equivalence. The
retained primal-only and dual-only blocks execute the same statements in the
same order as the old full calls. A skipped block's inputs are bit-identical
to its preceding execution: side classification does not write `basis`,
`factor`, or `cost`, while cost shifting does not write `basis`, `factor`,
`bounds`, or `move`. Therefore re-executing the skipped block could only
overwrite its output with the same bits. Factor solves mutate diagnostics but
not the factorization; every failure-producing solve writes its own diagnostic
record before it can be read.

The implementation keeps `rebuild_membership`, heap invalidation, objective
recalculation, and the final mathematical audit on every slice. Acceptance is
conditional on identical iteration/rebuild/phase telemetry across the path
contract, all mathematical and NETLIB accuracy gates, and a lower measured
rebuild bucket. Any pivot-path change rejects E6 rather than being explained
as benchmark noise.

### 6.5 Reconstruction audit dependency and iterative refinement (correctness contract)

The primal reconstruction map is

\[
  x_B(B,s)=B^{-1}(b-A_Nx_N(s)).
\]

Therefore a canonical-residual audit is reusable only while both the factor
generation (B) and the nonbasic side vector (s) are unchanged. The former
rate-limit key used only the factor rebuild count. During `major_rebuild`, the
first reconstruction audited the old side vector; boxed-column side
classification then changed (s), but the second reconstruction incorrectly
treated the old audit as current. `grow22` exposed the missing dependency with
a deterministic (6.1467\times10^{-8}) residual against the unchanged
(10^{-8}) limit.

Bound-side reconstruction now forces an exact long-double canonical residual
and defect correction before the final audit. Correction is genuine iterative
refinement, not a tolerance relaxation: each step solves (B\delta=r), updates
(x_B\leftarrow x_B+\delta), and recomputes (r=b-Ax) in long double. It
succeeds only at the original feasibility limit, continues only while the
infinity norm strictly decreases, and otherwise fails closed. The universal
safety cap is `numeric_limits<double>::digits`, derived from machine precision
rather than a model, density, or case threshold.

The fixed 24x3 cohort is now 72/72 successful and accurate. `grow22` reaches
the published objective with relative error (2.72\times10^{-12}) and
normalized primal violation (5.04\times10^{-9}). The corrected cohort
geometric means are 4.788 ms Native and 3.589 ms HiGHS, or 0.750x. Earlier
69/72 geometric means included a 4.7 ms fail-fast run for `grow22` and are not
valid complete-solve performance baselines.

### 6.6 DSE support-intersection extraction (Class P)

Let $d=|\operatorname{supp}(B^{-1}a_q)|$,
$r=|\operatorname{supp}(B^{-1}B^{-T}e_p)|$, and let the basis order be
$m$. The Goldfarb-Reid recurrence reads the auxiliary vector \(\rho\) only
at the $d$ pivotal-column positions. The former implementation nevertheless
exported all $r$ entries from HFactor's dense-backed `HVector`, zeroed an
$m$-entry `dense_rho`, scattered the $r$ entries into it, and then gathered
the $d$ required values.

HiGHS keeps the FTRAN-DSE result in `HVector::array` and updates weights on the
pivotal-column support. Native now uses the same ownership principle without
copying HiGHS' density thresholds: the backend retains the already captured
pivotal FTRAN pattern and extracts \(\rho\) from the new FTRAN's existing dense
backing in exactly that order. Since $d\le m$, the new $d$-entry contiguous
output replaces at least the old $m$-entry zero materialization, while the
packed \(\rho\) export and its $r$-entry scatter disappear completely. No
stamp table, hash lookup, density selector, or model-size threshold is added.

The recurrence loop, long-double expression order, clipping, and weight commit
order are unchanged. A differential test compares every extracted coordinate
with ordinary packed FTRAN lookup under a permuted basis. On the fixed 24x3
cohort, A/B/A native geometric means were 4.256 / 4.234 / 4.263 ms; removing
runtime from the JSON made every native run bit-identical in status,
iterations, objective, and audit fields. The approximately 0.6% improvement is
small but passes the predeclared suite gate and is retained.

### 6.7 BFRT RHS ownership experiment (Class A, rejected)

For flip set (F), let
(L=\sum_{j\in F}\operatorname{nnz}(A_j)) and let (S\le L) be the number
of unique touched rows. Three threshold-free kernels successively reduced the
old RHS construction from (O(L)) pair materialization plus
(O(L\log L)) sorting to stamped (O(L+S\log S)), first-touch (O(L+S)),
and finally direct collection into HFactor's consumer-owned `HVector`. The
last form also deleted indexed-RHS export and the consumer scatter, exactly
the ownership transformation retained for DSE.

All forms passed 72/72 accuracy. Their native geometric means were within
measurement noise of, or slower than, the 4.851 ms control: 4.861/4.836 ms
for the bracketing sorted-stamp runs, 4.849 ms first-touch, and 4.858 ms for
direct HVector ownership. Several pivot paths changed because equal-row terms
were no longer ordered by an unstable sort, so a neutral wall result is not a
Class-P justification. All BFRT RHS variants were removed.

This separates a structural proof from a bottleneck proof. Deleting traffic
is necessary to propose a scalar kernel, but the fixed cohort must still show
that the deleted traffic lies on a material critical path. BFRT RHS does not;
therefore no density, size, or case selector is justified for this family.

### 6.8 What is settled

Hyper-sparse triangular solves are active on the hot path (`solveHyper` in
profiles; the indexed solve path always passes true density). Exact DSE
beats Devex at every size in this codebase (validated 2026-08-01:
cold-start Devex raised `d2q06c` 5,618 → 8,519 pivots, geomean 4.963 →
5.262 ms). The postcondition and reduced-cost commit traversals may not be
algebraically merged (rejected experiment of 2026-08-01, bitwise
transactional equivalence failure). These questions are closed unless new
evidence is recorded here.

## 7. The dominating-bound relaxation lemma

The 2026-08-01 ratio-test short-circuit generalizes:

**Lemma.** Let a branch predicate \(P(e)\) be monotone in a computed
quantity \(e \ge 0\), and let \(U\) be any cheaply computable value with
\(e \le U\) guaranteed against all rounding (proved by construction, e.g. a
factor-2-guarded norm product). If \(P(U)\) is false, \(P(e)\) is false,
and the branch may be decided without computing \(e\) — bit-exactly,
provided no other consumer reads the value of \(e\) on that path. Monotone
rounding (fl is order-preserving) extends the argument to predicates of the
form \(fl(x + e) > 0\).

Application inventory: BFRT nonpositive-pivot skip and Phase-II positive
certification (implemented 2026-08-01, `State::bfrt_error_coef`); candidate
sites must document (a) the dominance proof with its rounding guard and
(b) the audit that no consumer of the exact value survives on the
short-circuited path. Sites that fail (b) — e.g. the Phase-I capacity
evidence — must keep the exact evaluation, as the implementation already
does.

## 8. Required invariant tests

Before enabling each experiment:

1. projected reduced-cost perturbation per column class (lower-only,
   upper-only, boxed, fixed) never violates the current iterate's dual
   feasibility after reconstruction;
2. perturbation removal restores `working_cost_is_original` and the
   original-space audit on an intentionally perturbed optimum;
3. any EXPAND proposal defines a cumulative infeasibility budget and proves
   its reconstruction/reset audit at INVERT;
4. shift-start lemma: constructed bases with known dual-infeasible column
   sets produce exactly the predicted shift count and a dual-feasible
   working start (extend the existing `initialize_cost_shifted_dual_start`
   test);
5. experimental PRICE: all representations agree to the documented rounding
   envelope on randomized sparse fixtures; any future selector is a pure
   function of structural work/state rather than wall time or a density cutoff;
6. reinversion-interval adaptation never overrides a numerical-trouble or
   fill-advice rebuild (safety triggers are not schedulable);
7. every dominating-bound site carries a test that drives the cheap bound
   below/above the exact value near the boundary and asserts identical
   branch outcomes;
8. staged telemetry satisfies
   `dualI + dualII + primalI + primalII + primalCleanup == iterations` on
   every terminal path.

## 9. Staged experiment plan

Ordered by expected payoff over risk; each stage passes its gates before
the next starts, and each result — retained or rejected — is recorded in
`docs/native_dual_simplex_optimization_2026-07-31.md`.

| Stage | Change | Class | Regime / target cases | Gate statistic |
|---|---|---|---|---|
| D1 | Complete dual/primal phase telemetry | P | all fallbacks | phase sum equals total iterations — complete |
| E1 | Cost-shifted dual start | A | suite-wide | 72/72, 4.395 ms, 0.785x — retained |
| D2 | Attribute `grow7` pivot excess | diagnostic | `grow7`, `grow22` | cold-path cause established — closed |
| E2 | Perturbation + EXPAND | A | unassigned | blocked on corrected mathematical contract and new evidence |
| E3 | PRICE representation switching | A | suite-wide | rejected: online model +4.1%, row switch +2.4% |
| E4 | Reinversion interval measurement | diagnostic | 3 + small models | complete: wall gate not met; policy unchanged |
| E5 | Dense branchless scan layout | P/A | 3 | measured kernel cost plus the applicable path contract |
| E6 | Dependency-sliced INVERT | P | all | 72/72 identical paths, 4.220 ms, 0.791x — retained |

The next performance experiment must use the post-E1 production path. Legacy
`fit1p`/`d2q06c` dual-kernel profiles did not justify E3 because those cases
now spend their dominant work in primal cleanup.

The performance target remains: a repeated full-suite run with 72/72
accurate results and geometric speedup greater than 1.0x versus HiGHS. No
stage is credited against the target except by that measurement.

E6 does not meet that terminal target. In the interleaved 24-case, three-repeat
run, Native geometric mean moved from 4.287 ms to 4.220 ms (1.6% lower) and
the ratio to one-thread HiGHS moved from 0.785x to 0.791x. All 72 Native
`(case, repeat, iterations, status, objective)` records are identical before
and after. Raw results are
`reports/netlib_e6_{before,after}_repeat3.{csv,json}`.

## 11. Reliability branching and reusable strong-branch states

The complete derivation, failure-state algebra, candidate loop, reuse
contracts, test matrix, implementation phases, and acceptance gates are now
recorded in
`docs/native_milp_reliability_branching_theory_and_plan.md`. The remainder of
this section is the shorter historical summary of the first implementation.

Let the parent relaxation value be \(z\), let \(x_j=k+f\), and let the two
child relaxation values be \(z^-\) and \(z^+\). A pseudocost observation is a
cost per unit displacement, not the complete child gain:

\[
  p_j^- = \frac{\max(0,z^- - z)}{f},\qquad
  p_j^+ = \frac{\max(0,z^+ - z)}{1-f}.
\]

The predicted gains are therefore \(q_j^-=fp_j^-\) and
\(q_j^+=(1-f)p_j^+\). Storing \(z^\pm-z\) directly and multiplying by the
distance again during scoring makes the estimate quadratic in \(f\), so
observations made at different nodes are not comparable. Root probes, tree
strong probes, and solved children now share one normalization contract. The
distance is captured before propagation; later domain tightening may improve
the child bound but must not redefine the branch displacement.

Reliability branching spends a strong-branch LP only when the provisional
pseudocost winner is itself unreliable. Candidates are ordered by the same
priority and pseudocost relation used for final selection. After observations
are incorporated, final selection again covers every eligible candidate; a
probed subset is never given unconditional precedence over a reliable global
candidate.

For a probe domain \(D_p\) and post-propagation child domain \(D_c\), the
probe objective and proof status are consumed as the child result only when
\(D_p=D_c\) and the local-cut structure agrees. If propagation gives
\(D_c\subset D_p\), the old objective is not published as the solved child
objective: only the primal seed and an independently owned vendored-HiGHS
basis may warm the required re-solve. Native basis operations retain a pointer
to their temporary matrix and are detached. Thus neither sibling state nor a
parent proof snapshot is mutated by probe reuse.

Telemetry separates pseudocost regret from observed strong-score regret:

\[
  R_{pc}=\max_{i\in C_{prio}}s_i-s_j,
  \qquad
  R_{sb}=\max_{i\in C_{probed}}\hat s_i-\hat s_j.
\]

The second quantity is sampled only when the selected variable has a complete
down/up probe pair; unknown scores are never treated as zero. Cache exact/warm
hits and avoided duplicate LPs are reported separately and do not affect any
branch decision.

## 10. Persistent incremental LP state for branch-and-cut

For a fixed standard-form matrix and basis,

\[
  x_B=B^{-1}(b-A_Nx_N),\qquad
  r_N=c_N-A_N^TB^{-T}c_B.
\]

A tree-node bound change modifies `b`, nonbasic bound sides, and the objective
constant, but not `A`, `c`, or `B`. Therefore the inherited reduced costs are
unchanged: a dual-feasible parent basis remains dual feasible and only primal
feasibility must be repaired. This is exactly a dual-simplex reoptimization;
rebuilding, rescaling, and passing the matrix again adds work without changing
the mathematical problem.

Appending a direct `<=` cut adds one implicit row slack. With that slack basic,
the extended basis is block triangular around the parent basis. Existing dual
feasibility is preserved while violation of the new cut appears as primal
infeasibility, again selecting dual simplex. Equality-row insertion is not
treated as tail append because the native row order is `[ineq | eq]`.

The mutable HiGHS handle is a sequential workspace, not a node snapshot. A
node snapshot contains immutable bounds and basis status. A transaction saves
`(row count, bounds, basis)`; rejection deletes appended rows, restores bounds
and basis, then reruns that exact state to reconstruct its numeric
factorization. Parallel workers do not use this shared-handle path. These are
ownership and algebraic contracts, not model-size thresholds or case-specific
parameters.

## 11. Branch-free BFRT prefilter model

The Phase-II cheap BFRT classifier has seven mutually exclusive paths:
basic, fixed, positive stability-prefiltered, nonpositive cheap-rejected,
nonpositive exact-required, positive cheap-certified, and positive
exact-required. This partition is semantic, not a density selector. A two-lane
NEON kernel evaluates the same strict comparisons with masks. The standalone
proof kernel emitted `(signed_alpha, range, flags)`; production subsequently
removed the derived range stream as described in section 17. Exact-required
lanes remain owned by the scalar `dot_error_bound` stage.

For (N=N_b+N_m+N_a), the audited array-byte models of the standalone scalar
evaluation record and compact SIMD prefilter are

\[
 M_s=48N+5N_b+6N_m+38N_a,\qquad M_v=55N,
\]

so (M_s-M_v=31N_a-2N_b-N_m). This is deliberately not claimed as an
all-distribution theorem: gathering inactive lanes can lose up to two bytes
per basic lane. A future production PRICE/BFRT ownership contract can make the
reduction unconditional by omitting basic and fixed columns when PRICE emits
the packed BFRT support. That structural filter has no size or density
threshold; with (N=N_a), the compact stage deletes exactly (31N) output
bytes relative to the conservative 48-byte evaluation record.

On Apple Clang 21 AArch64 at `-O3`, the scalar disassembly gives path costs

\[
 I_s=18+37N_b+40N_m+56N_{pp}+56N_{nr}+57N_{ne}
       +58N_{pc}+58N_{pe}.
\]

The SIMD main body is 80 instructions per two lanes, including one loop branch
and no candidate-dependent control transfer, so for even (N),
(I_v=24+40N). These constants are enabled only for the audited compiler and
architecture; odd tails and other toolchains report the instruction gate as
unavailable. This is an exact dynamic count from static basic-block sizes plus
runtime category multiplicities, not a predicate-count proxy and not a
hardware-counter claim.

The deterministic (2^{20})-lane fixture measured (M_s=86,974,187) and
(M_v=57,671,680) bytes (33.691% lower), and (I_s=58,083,022) versus
(I_v=41,943,064) instructions (27.788% lower). Five serial runs, each using
seven alternating-order medians, gave speedups from 1.326x to 1.393x with a
1.349x median. Boundary fixtures cover odd tails, all-basic/all-fixed inputs,
signed zero, equality at both thresholds, and nonfinite pivots, ranges, error
coefficients, row norms, and tolerance.

The production ownership boundary now retains the complete pivotal row because
Devex and reduced-cost commit still require it. During the same PRICE export,
the producer also records the packed positions of nonbasic movable entries;
this is a four-byte index, not a copied column/value pair. Phase-II BFRT reads
only this active stream, runs the mask prefilter, evaluates `dot_error_bound`
only for `needs_exact`, and merges directly in original PRICE order. The old
`BfrtScanEvaluation[]` materialization is absent on this path. Dual Phase I
keeps its original scalar exact evaluation.

The complete implementation passed every correctness gate and a fixed 24x3
A/B/A. Control/experiment/control native geometric means were 5.140, 4.803,
and 5.187 ms, so the experiment is 7.5% faster than the geometric mean of its
bracketing controls. All three runs were 72/72 accurate, and their JSON records
were identical after deleting runtime. A final Native+HiGHS run measured 4.780
ms versus 3.727 ms, or 0.780x. The temporary experiment switch was removed;
there is no case, size, or density selector in production.

## 12. Structural exact DSE initialization

For dual steepest-edge pricing, the cold weight of basis row \(i\) is

\[
  w_i=\lVert B^{-T}e_i\rVert_2^2.
\]

If every basis column is a singleton in its assigned row, then
\(B=\operatorname{diag}(d_1,\ldots,d_m)\) under the maintained row-to-basis
ordering and therefore \(w_i=1/d_i^2\). This is complete exact DSE, not an
approximation or a model selector. It requires one pass over the basis columns
and no BTRAN. Production takes this analytic path when the structural
precondition holds and otherwise starts Devex. Explicit benchmark modes can
still force Devex or compute checked full weights from the current factor. No
size, density, case, or timing threshold participates in the production
decision.

The fixed NETLIB cohort established that all 72 cold starts satisfy the
structural precondition. Structural and full exact DSE consequently had zero
initialization solves and identical pivot count, status, objective, and
audited feasibility on every run. Against explicit Devex, mean pivots fell
from 824.7 to 639.0, a 22.52% reduction. Initialization increased only from
0.00057 ms to 0.00164 ms in the bracketed measurement.

The 24x3 A/B/A geometric means were 4.677 ms Devex, 4.341 ms structural DSE,
and 4.621 ms Devex. Structural DSE reduced elapsed time by 6.61% against the
4.649 ms geometric mean of the controls even though its geometric kernel cost
per pivot was 3.59% higher. The gain is therefore a pivot-count effect, as the
edge-weight theory predicts, rather than evidence for a faster minor kernel.
All three brackets were 72/72 accurate.

The production promotion was then measured with separate fixed binaries:
explicit Devex control, structural-exact production, and the same Devex
control again. Native geometric means were 4.674, 4.369, and 4.664 ms. The
geometric mean of the controls was 4.669 ms, so production structural DSE was
6.42% faster; the controls differed by only 0.21%. Mean pivots again changed
from 824.7 to 639.0, while mean initialization time remained 0.00142 ms with
zero BTRAN. All 216 native runs were accurate and Optimal. In the experiment
bracket HiGHS measured 3.559 ms, so Native/HiGHS improved from the first
control's 0.766x to 0.814x. Raw data are
`reports/netlib_structural_production_{A_devex,B_exact,C_devex}_repeat3.{csv,json}`.

The same identity also applies to an uncached warm basis: its proof depends on
the current basis matrix, not on how that basis was obtained. Production now
uses analytic weights for a proved assigned-row-singleton cold or warm basis.
A non-singleton basis falls back to Devex, a matching warm DSE cache still has
priority, and the rejected certified CHUZR experiment is not selected by
production.

An exact terminal audit initially exposed one deterministic `grow22` failure:
primal cleanup rebuilt the factor with a backward-stable FTRAN, but deferred
the long-double canonical residual check until termination. A primal major
INVERT is now a certification boundary: reconstruction immediately evaluates
the exact canonical residual and performs monotone defect correction if the
fixed feasibility contract requires it. No tolerance changed. This restored
72/72 accuracy without changing the 810-pivot DSE path.

## 13. Certified exact CHUZR experiment

The updated DSE heap is advisory: recomputing only its selected row does not
prove that row maximizes the current exact DSE merit. A benchmark-only selector
tested whether closing this gap improves the four cases where structural exact
DSE initialization increased pivots. For each infeasible basis row (i), let
(y_i=B^{-T}e_i), (b_i) be basis column (i), and let the checked BTRAN
contract be

\[
  \lVert e_i-B^Ty_i\rVert_\infty
  \le c(\lVert B^T\rVert_\infty\lVert y_i\rVert_\infty+1).
\]

Including the rounding envelope of the residual dot product gives

\[
  \lVert y_i\rVert_2 \ge
  \frac{1-c}{\lVert b_i\rVert_2+(c+\gamma_m)\lVert B^T\rVert_\infty}.
\]

This lower bound on the DSE weight gives an upper bound on merit. Candidates
are processed in descending upper-bound order with checked BTRAN, and the
search stops only when the best evaluated merit dominates every remaining
upper bound, with row index as the deterministic tie break. Taboo priority is
preserved. No empirical threshold is involved.

The targeted results reject this as a production direction:

| Case | Devex pivots | Structural pivots | Certified pivots | Certified BTRAN | Structural ms | Certified ms |
|---|---:|---:|---:|---:|---:|---:|
| `grow22` | 732 | 810 | 806 | 33,444 | 57.19 | 321.59 |
| `fit1d` | 67 | 70 | 70 | 604 | 6.61 | 7.06 |
| `recipe` | 35 | 36 | 36 | 138 | 0.13 | 0.25 |
| `stocfor2` | 1,041 | 1,054 | 1,061 | 125,306 | 18.60 | 1,802.75 |

All 36 targeted runs were accurate, and the brute-force inverse-row oracle
test passed. Across the four-case cohort, CertifiedExact's geometric mean was
31.460 ms versus 5.529 ms for Devex and 5.499 ms for StructuralExact. Exact
selection did not recover the Devex pivot path on any case; on `stocfor2` it
made it worse. The loose but proof-safe bound required about 41.5 and 118.1
BTRAN per pivot on `grow22` and `stocfor2`. Therefore the DSE regressions are
an algorithmic merit-choice effect, not evidence that cached weights need more
eager correction. The mode remains benchmark-only. A full 24x3 timing gate is
intentionally not run because the predeclared targeted gate failed. Raw data
are `reports/netlib_certified_dse_targeted_repeat3.{csv,json}`.

## 14. CHUZR producer/consumer decomposition

Deleting an intermediate index stream is not automatically a traffic win. A
prototype refreshed CHUZR heap entries directly while traversing the
16-byte `pair<int,double>` primal transaction. Although it removed a four-byte
row write and later read, it regressed `d2q06c` by 0.23% in A/B/A. The compact
row stream shortens the commit dependency chain and is a better heap-consumer
layout, so the fusion was removed.

Decomposition exposed a strictly dominant subgraph: the transaction always
appends the leaving row, hence the compact stream always contains it. The old
consumer nevertheless scanned the whole stream with `std::find` before an
unreachable fallback insertion. Removing only that scan retains the compact
layout and identical heap operation order while deleting (O(k)) comparisons
per pivot for (k) changed primal rows. The full 24x3 B/A/B was 72/72 accurate
in every bracket and byte-identical after timing removal. Its effect is below
stable wall-clock resolution; retention follows from instruction dominance,
not a timing or model-dependent selector.

## 15. DSE transaction stream ownership

The pivotal-column FTRAN already owns the ordered row support consumed by the
Goldfarb-Reid recurrence. The former transaction nevertheless materialized a
16-byte `{row,value}` record per updated row and read it back after the factor
update. The row was duplicated state: `direction.index` remains valid until
commit. Production now materializes only the eight-byte nonpivotal values plus
one pivotal value, validates the complete update before mutation, and commits
the values against the original direction stream. This deletes at least 12
bytes of transaction traffic per nonpivotal update without changing update
order, arithmetic, failure boundaries, or pivots.

On the fixed 24x3 gate all 72 runs were accurate and Optimal, with the same
639.0 mean pivots. Native measured 4.355 ms versus 3.542 ms for HiGHS (0.813x).
The previous production measurement was 4.369 ms, so the difference is below
stable wall-clock resolution and is not claimed as a speedup. Retention follows
from strict stream dominance and identical discrete paths. Raw data are
`reports/netlib_dse_value_stream_full_repeat3.{csv,json}`.

## 16. Rejected PRICE-owned absolute-dot stream

A stronger ownership experiment accumulated
`sum_i |row_ep_i A_ij|` beside every signed PRICE accumulator, allowing BFRT to
evaluate its unchanged `gamma_k * absolute_dot + 256*eps*absolute_dot` bound
without CSC column rereads or dense row materialization. It was correct, but
not dominant: every PRICE matrix term gained an absolute-value accumulator RMW
and dependency, while only `needs_exact` columns avoided their second read.
The changed summation order also moved `d2q06c` from 5,609 to 5,678 pivots.

A fixed targeted B/A/B measured 857.9 ms control, 898.2 ms experiment, and
855.1 ms control. Relative to the 856.5 ms control geometric mean, the fused
path regressed by 4.9%, and kernel time per pivot rose from about 0.150 to
0.156 ms. The production wiring was removed. This falsifies the general rule
that producer fusion is beneficial when it adds work to every source item to
save work only for a consumer subset. Raw data are
`reports/netlib_price_abs_d2q_{control_a,experiment_b,control_c}.{csv,json}`.

## 17. BFRT derived-range ownership

The Phase-II SIMD prefilter formerly materialized `upper[j]-lower[j]` for every
active PRICE lane, then the ordered merge immediately read that value once.
The range is derived from immutable bounds and is consumed only when signed
alpha is positive. Production now keeps only signed alpha and classification
flags in the prefilter workspace; the original-order merge computes the same
subtraction once inside its positive branch.

This removes one eight-byte range write and read for every active candidate,
and additionally removes two bound reads plus the subtraction for every
nonpositive candidate. It adds no threshold, changes no exact-error fallback,
capacity order, candidate order, or transaction boundary. `d2q06c` retained
5,609 pivots. The complete gate was 72/72 accurate for both Native and HiGHS,
with 639.0 mean Native pivots and 0.001445 ms mean DSE initialization. Native
measured 4.358 ms versus 3.502 ms for HiGHS (0.804x), statistically unchanged
from the preceding 4.355/3.542 ms gate. Retention follows from strict traffic
dominance; no wall-speedup claim is made. Raw data are
`reports/netlib_bfrt_deferred_range_full_repeat3.{csv,json}`.

A related reduced-cost transaction experiment reused `pivot_row.value` for
prevalidated commit values and made `rcUpdate` nearly disappear, but changed
`d2q06c` from 5,609 to 5,592 pivots. Moving the committed floating result into
the validation context changed its contraction/rounding context, violating the
discrete-path gate. The experiment was removed despite remaining accurate.

## 18. Minimal sufficient BFRT candidate state

For an admissible candidate let (s\in\{-1,1\}) be the leaving side,
(\sigma_j\in\{-1,1\}) the nonbasic move sign, and (p_j) the pivotal-row
coefficient. BFRT stores

\[
  \alpha_j=s\sigma_jp_j>0,
  \qquad \beta_j=\frac{m_j}{\alpha_j}.
\]

After `breakpoint=beta` is formed, margin has no downstream consumer. The
original pivot is also not independent state: (p_j=s\sigma_j\alpha_j).
Multiplication by the two signs is exact for every finite nonzero admitted
pivot, and move signs remain unchanged until the transaction is returned.
Therefore the minimal candidate record is `(col, alpha, breakpoint, range,
taboo metadata)`.

Production removes both copied doubles from `Candidate`, reducing its Apple
AArch64 layout from 56 to 40 bytes. The Phase-I scan record also drops its
copied pivot, while the Phase-II merge no longer rereads `pivot_row.value` just
to populate candidates. Unlike the earlier rejected compound experiment, this
does not defer taboo lookup or change any hash-probe dependency chain.

Positive and negative pivot reconstruction tests pass exactly. `d2q06c`
retained 5,609 pivots; its five-run BFRT bucket moved from about 0.16 to 0.15
seconds and ordering from about 0.00533 to 0.00497 seconds, which is supporting
evidence rather than a fleet speedup claim. The full gate was 72/72 accurate
for both solvers, with 639.0 mean Native pivots. Native measured 4.359 ms versus
3.568 ms for HiGHS (0.819x). Removing timing fields makes the previous and new
full reports identical. Raw data are
`reports/netlib_bfrt_minimal_candidate_full_repeat3.{csv,json}`.
