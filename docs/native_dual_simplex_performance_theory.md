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
  1,166 mathematical assertions, 220 NETLIB regression assertions, 72/72
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
- **Numerical fallback** (`grow22` with E1 disabled): Dual Phase II fails a
  canonical residual reconstruction check and falls back to primal Phase I/II.
  The formerly missing 886 pivots are now explicitly accounted.

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
assertions, 220 NETLIB regression assertions, and 72/72 repeated benchmark
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
`grow22` also does not reopen this diagnosis: with E1 disabled, its extra path
is an explicitly logged numerical fallback after canonical residual
reconstruction failure; with E1 enabled, 533 pivots are primal cleanup. Neither
is evidence against BFRT group selection. No ratio-test change is authorized.

## 6. Per-iteration cost theory

E1 invalidates the former `fit1p` and `d2q06c` kernel diagnosis. `fit1p` now
runs entirely as 541 primal cleanup pivots and is faster than HiGHS on the
formal run (about 16.5 versus 18.3 ms). `d2q06c` is dominated by 7,797 primal
cleanup pivots, so optimizing dual PRICE cannot address its production-path
bottleneck. The remaining dual-kernel targets are `scsd8` and `25fv47`, where
wall time remains high after path effects are accounted.

### 6.1 Density-adaptive PRICE (Class A)

Row-wise PRICE costs
\(C_r \approx c_s \sum_{i \in \text{supp}(\pi)} \text{nnz}(A_{i\cdot})\)
with a stamped-accumulator constant \(c_s\) that includes a data-dependent
branch per term. Column-wise PRICE against a dense scatter of \(\pi\) costs
\(C_c \approx c_d\,\text{nnz}(A) + c_g n\) with a branchless streaming
constant \(c_d\) (no stamps or touched list). The inequality \(c_d<c_s\) is
an empirical hypothesis, not a structural guarantee.
With uniform row counts the crossover is at support density
\(\rho^* \approx (c_d/c_s)\bigl(1 + c_g n / (c_d\,\text{nnz}(A))\bigr)\).
If a controlled microbenchmark establishes \(c_s \approx 3c_d\), the model
predicts \(\rho^* \approx 0.3\)–0.4, the range previously recorded on dense
dual paths such as `scsd8`. The switch must be taken per pivot from
\(|\text{supp}(\pi)|/m\).

**Determinism contract.** Column-wise accumulation fixes the summation
order to A's column storage order — deterministic, but different from the
row-wise order, so results differ in rounding and paths may change: Class A.
The constants \(c_s, c_d, c_g\) must be measured on this machine before the
threshold is frozen, and the threshold is a recorded constant, not an
environment variable.

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
effective interval is ~37 pivots and rebuild work is ~8% of the solve.
Before changing anything, measure \(R\) and \(u\) per case from the
existing profile (rebuild bucket; ftranU tick growth between rebuilds). If
the measured \(T^*\) disagrees with the effective interval by more than 2x
on cases totaling >10% of suite time, adopt the adaptive rule with the
clamp bounds recorded here.

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

### 6.5 What is settled

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
5. density-adaptive PRICE: both representations agree to the documented
   rounding envelope on randomized sparse fixtures, and the switch is a
   pure function of the recorded threshold;
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
| E3 | Density-adaptive PRICE | A | 3: `scsd8`, `25fv47` | PRICE bucket, per-pivot time at equal paths |
| E4 | Reinversion interval measurement, then \(\sqrt{2R/u}\) rule if warranted | A | 3 + small models | rebuild bucket share |
| E5 | Dense branchless scan layout | P/A | 3 | measured kernel cost plus the applicable path contract |
| E6 | Dependency-sliced INVERT | P | all | 72/72 identical paths, 4.220 ms, 0.791x — retained |

The next performance experiment must use the post-E1 production path. Legacy
`fit1p`/`d2q06c` dual-kernel profiles are not evidence for E3 because those
cases now spend their dominant work in primal cleanup.

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
