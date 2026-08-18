# SPPT Executable Runtime Contract

The Semantics-Preserving Projection Theory (SPPT) module is an executable
verification layer around projection, attribution, validation, and solver
results. The public umbrella is `hacdcpf/sppt/sppt.hpp`. It does not replace the
underlying physical solver and does not prove correctness beyond the tested
system, observable, tolerance, and backend.

## Entry points

| Area | Public API | Output |
|---|---|---|
| Metamorphic relations | `mr1_*` through `mr7_*`, MR3 transient support, and the registered MR8 test | `MetamorphicResult` with ID, observation, pass, residual, tolerance, detail. |
| Certificate corpus | `certify_case()`, `certify_corpus()`, `certify_systems()` | PF commuting, independent residual, attribution, and OPF-dual evidence plus CSV/LaTeX renderers. |
| Admissibility guard | `guard_system()` | Validation, well-posedness, and requested-observable attribution gates. |
| Agent loop | `run_agent_loop()` and stable action-ID helpers | Per-edit guard/commit/analysis trace and classification metrics. |
| Benchmark/ablation | `run_benchmark*()`, `run_ablation*()` | Timing/residual or mechanism-removal evidence. |
| Intelligent simulation | `evaluate_intelligent_simulation_contract()` | Reject, read-only admission, or mandatory human review. |

## Metamorphic outcome semantics

`passed=true` is meaningful only when `observation == Observed`. A relation
that cannot obtain its required observable must return `passed=false` and
`NotObserved`; MR3d follows this rule when either DC-OPF result lacks LMPs.
Non-convergence is a failure with diagnostic detail. A mathematically
inapplicable structural relation may explicitly report a trivial observed pass,
such as equipotentiality when there are no merge classes.

Residual units depend on the relation: voltage comparisons are per unit,
transient frequency terms use their underlying result units after the relation's
normalization, and LMP residuals use the DC-OPF LMP unit. Tolerances are caller
inputs and are not universal certification thresholds.

## Identity and attribution

SPPT compares authored and canonical systems only after projection mappings are
applied. Bus and component `.index` values are stable authored identities;
vector positions and graph indices remain internal. `ObservableAttribution`
records the requested observable, canonical entity count, attributed entity
count, recovery class, and reason. Guard acceptance requires total attribution
for every requested observable, not merely a successful projection.

The independent hybrid residual deliberately evaluates authored primitive
AC/DC/VSC equations without calling the production projection, assembly, or
residual evaluator. `supported=false` or `converged=false` is unavailable
evidence, not a zero residual.

## Guard and authority contract

`guard_system()` accepts only when boundary validation, reference
well-posedness, and attribution totality all pass. Its `reason` names the first
failed gate and `details` preserve the supporting diagnostics. The scripted
agent loop applies and analyzes only accepted edits and reports whether any
inadmissible state was committed.

The intelligent-simulation gate additionally checks solver and independent
residuals, approximation permission, declared uncertainty/provenance, and
action authority. It never executes physical control. Even complete evidence
for a physical-control request can produce only `RequireHumanReview`.

## Certificate and benchmark boundaries

Certificate rows distinguish PF convergence/pass, independent support/pass,
attribution totality, approximate merge count, and OPF dual
convergence/observation/pass. A file that cannot be loaded is retained as a
non-passing row with a note. CSV and LaTeX renderers serialize evidence; they do
not strengthen it.

Benchmark timings are best-of-N wall-clock measurements on the current process
and machine. They are performance observations, not deterministic limits.
Ablations intentionally remove individual safeguards and must not be used as
production analysis paths.

## Registered verification

- `test_sppt_metamorphic`: MR1-MR7, authored/canonical index contracts, AC/DC
  projection, and explicit MR3d not-observed behavior.
- `test_sppt_dae_wellposedness`: MR8 index-1 and transient MR3 behavior.
- `test_sppt_certificate`: corpus rendering and independent hybrid residuals.
- `test_sppt_guard`: validation/reference/attribution gates and metrics.
- `test_sppt_agent`: guarded edits, uncertainty, and authority decisions.
- `test_sppt_benchmark` and `test_sppt_ablation`: reporting and mechanism
  sensitivity.

