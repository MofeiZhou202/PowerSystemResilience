# Learning-to-Optimize Framework Analysis and Implementation Plan

This document analyzes how to implement a learning-to-optimize (L2O) framework
on top of the current MIPSolvers infrastructure. It is grounded in the tutorial
material under `docs/LearningtoOptimize/`, the existing engine and SCUC APIs,
and the present branch-and-cut, StrictHiGHS, warm-start, and benchmark paths.

The key conclusion is that MIPSolvers should implement L2O as a solver-guidance
framework, not as a solver replacement. Learned models should propose warm
starts, parameter choices, branch priorities, node-ordering scores, cut budgets,
or certified cut-family choices. Feasibility, bounds, optimality certificates,
and cut validity must remain under the deterministic optimization engine.

## Part I: Theoretical Analysis

### 1. Why L2O is applicable here

Learning-to-optimize is useful when future optimization instances are drawn from
a concentrated task distribution rather than from the space of all possible
optimization problems. MIPSolvers has exactly this structure in the SCUC and
market-clearing pipeline:

- The mathematical form is stable: unit commitment variables, dispatch
  variables, reserve variables, DC flow constraints, storage constraints, and
  piecewise bid segments appear repeatedly.
- The data varies in structured ways: demand profiles, renewable forecasts,
  generator outages, reserve requirements, bids, network limits, and horizon
  length vary while the modeling grammar remains mostly fixed.
- The solver workload repeats: the same or similar power-system instances are
  solved day after day and can be logged, replayed, and benchmarked.
- The infrastructure already extracts useful solver artifacts: incumbents,
  root cuts, root bases, pseudocost data, branch priorities, cut diagnostics,
  primal-dual gaps, and node/cut statistics.

This matches the tutorial's main prerequisite for successful L2O: the presence
of many previous examples from the same task distribution. It also fits the
tutorial's conservative MIP recommendation: neural networks should guide a
traditional solver, because direct neural prediction of an MILP solution has two
fundamental weaknesses. First, a neural output is not guaranteed feasible.
Second, even a good neural output does not certify optimality. A branch-and-cut
solver can repair, validate, and certify; a neural model can only prioritize.

### 2. Solver-guidance principle

For MILP, the correct L2O contract is:

```text
learned policy proposes -> solver validates or uses softly -> solver certifies
```

The learned policy may improve speed, but it must not be required for
correctness. This yields three safety rules.

1. Learned primal proposals are only incumbents or repair seeds.
   They may be rounded, fixed, or rejected. The solver's feasibility checks
   decide whether they enter the incumbent pool.

2. Learned branching, node selection, and cut-selection policies are advisory.
   They may change search order or budgets, but they must not delete feasible
   regions, falsify lower bounds, or mark a node proven when it is not proven.

3. Learned cuts cannot be arbitrary rows unless they are independently
   certified. A model may select from solver-generated valid cuts, select from
   certified domain cut families, or choose budgets. If a neural model directly
   predicts coefficients, a deterministic validity oracle must prove that the
   row is globally valid, locally valid, or lazy-valid before insertion.

This design retains the solver's mathematical guarantees: every accepted
incumbent is feasible under tolerances, every pruning decision depends on a
valid bound, and the final result remains a normal MILP result with a gap or
certificate.

### 3. The MILP graph representation

The tutorial presents the standard ML-for-MIP representation: a bipartite graph
with variable nodes, constraint nodes, and coefficient edges.

For a model

```text
min c'x
s.t. A x <= b, Aeq x = beq, l <= x <= u, x_j integer for j in I,
```

we construct:

- Variable nodes `v_j` with features such as objective coefficient, lower and
  upper bounds, variable type, current LP value, fractionality, reduced cost,
  pseudocosts, branch priority, block tag, and incumbent value if available.
- Row nodes `r_i` with features such as row sense, right-hand side, activity,
  slack, dual value, row density, row family, and validity scope.
- Edges `(r_i, v_j)` with coefficient `A_ij`, scaled coefficient, sign, and
  optional structural tags.

This representation is especially appropriate for MIPSolvers because `LPModel`
already stores sparse matrices in Eigen, and `MIPModel::UCGenHint` stores SCUC
block metadata. A graph neural network can generalize across different numbers
of buses, generators, rows, and columns because message passing shares weights
across nodes and edges.

For SCUC, a richer graph is usually better than a purely generic matrix graph.
The model should expose domain tags without violating solver generality:

- Generator-time variables: commitment `IG(g,t)`, startup `SU(g,t)`, shutdown
  `SD(g,t)`, dispatch `PG(g,t)`, reserve variables, and segment variables.
- Network structure: bus nodes, line/section constraints, generator-to-bus
  edges, load-to-bus data, renewable-to-bus data.
- Time structure: edges between adjacent periods for ramping, min-up/down,
  storage SOC, and DC-line ramping.
- Forecast context: load, wind, solar, reserve demand, and initial state.

A two-level representation is recommended:

- Generic MILP graph for general engine policies.
- SCUC-augmented graph for domain-specific policies that consume `UCGenHint`.

The generic policy can work on any MILP; the SCUC policy should be enabled only
when certified metadata exists.

### 4. Learning targets

The L2O surface should be divided by solver decision type. Each target has a
different risk level and implementation cost.

#### 4.1 Learned primal warm starts

Goal: quickly produce a good feasible or near-feasible incumbent.

For SCUC, the main discrete decision is the commitment schedule. A model can
predict commitment, startup, and shutdown patterns from demand, renewable,
generator, and network features. The solver then repairs the prediction by:

- fixing or biasing integer variables;
- solving a dispatch LP with fixed commitment;
- relaxing uncertain variables in a local neighborhood;
- using feasibility jump, RENS, LNS, or local branching;
- accepting the result only after feasibility and objective validation.

This is the lowest-risk L2O entry point. A bad warm start is ignored; a good
warm start improves primal bound, pruning, reduced-cost fixing, and time to
first incumbent.

The main losses are:

- Binary cross entropy on commitment/startup/shutdown labels.
- Cost-weighted classification loss, giving more weight to variables whose
  wrong value causes high startup, no-load, load-shed, or reserve cost.
- Feasibility-aware penalties for ramp, min-up/down, reserve, storage, and
  network violations in the predicted schedule.
- End-to-end repair objective: cost after deterministic repair.

#### 4.2 Learning to branch

Goal: choose the branching variable that reduces the branch-and-bound tree or
closes the primal-dual gap faster.

The tutorial describes three parameterizations:

- Per-variable MLP over local features.
- Feature-augmented MLP using neighborhood statistics.
- GNN over the full MILP bipartite graph.

The usual supervised label is strong branching score. Strong branching is too
expensive for every node on large SCUC cases, so labels should be sampled under
a budget. Practical labels include:

- Full or partial strong branching score at selected nodes.
- Reliability branching decisions.
- Pseudocost updates observed after actual branching.
- Retrospective labels from variables on branches that led to incumbent
  improvements or bound lifts.

Training objectives should prefer ranking over raw regression. The absolute
scale of strong branching scores varies by node and instance, while the solver
only needs the best candidate or a good top-k set. Pairwise ranking loss,
listwise cross entropy over candidates, or normalized score regression are
better than unnormalized mean squared error.

Deployment should blend the learned score with pseudocosts, not replace them:

```text
score(j) = solver_score(j) + lambda * learned_score(j)
```

The weight `lambda` should be configurable and bounded. On out-of-distribution
instances, the engine should reduce `lambda` or disable the policy.

#### 4.3 Learning to search

Goal: choose which active node to process next.

The theory separates two objectives:

- Best-first search improves the global lower bound as directly as possible.
- Depth-first or diving behavior often finds incumbents earlier.

Learning is most useful for primal-bound discovery: selecting nodes likely to
contain high-quality feasible solutions. Once a good incumbent exists, the
solver may switch back to best-first or hybrid search for proof.

Possible labels:

- Whether a node contains the final optimal solution.
- Whether a node is on a path to the first incumbent or best incumbent.
- Future reward from exploring the node: incumbent improvement, bound lift,
  or gap integral reduction within a fixed budget.

Node selection has high interaction effects because changing search order
changes the future tree. Therefore, a staged approach is best:

1. Offline imitation of successful baseline traces.
2. Limited early-tree advisory ordering.
3. Reinforcement learning only after deterministic tracing and replay exist.

#### 4.4 Learning to select cuts

Goal: select cuts that improve bounds enough to justify their LP cost.

The tutorial frames cut selection as assigning scores to candidate cuts. A cut
is useful if it improves the bound, reduces future nodes, or enables stronger
domain propagation. A cut is harmful if it makes LPs larger and slower without
moving the bound.

Useful features include:

- efficacy;
- density;
- parallelism with existing cuts;
- binary support ratio;
- violation at current LP point;
- cut family;
- row activity and dual information;
- estimated LP re-solve cost;
- historical acceptance and bound-lift rates by family.

The action space can be very large, so the first implementation should learn a
filter or budget policy rather than a combinatorial subset selector. For
example:

- choose per-family cut budgets;
- choose threshold multipliers for existing filters;
- accept/reject top-k candidates after deterministic pre-ranking;
- choose whether to keep root cuts for future warm starts.

For SCUC, learned cut selection should initially select among certified market
cut families and solver-generated valid cuts. It should not predict arbitrary
cut coefficients.

#### 4.5 Learning solver configuration

Goal: choose options before a run or before a phase of a run.

This is an attractive near-term target because `BCCallbacks::hyperparam_tuner`
and `BCPostSolveFn` already exist. A learned or rule-based tuner can choose:

- root LP solver mode;
- root cut rounds and separation budgets;
- heuristic effort;
- pseudocost reliability threshold;
- symmetry detection;
- cut-pool aging;
- feasibility-jump and LNS budgets;
- time allocation between primal heuristics and proof.

This can be formulated as contextual bandit or algorithm selection. Features
come from model size, density, objective ranges, integer counts, SCUC metadata,
prior run statistics, and time limit. Rewards can be negative solve time,
negative gap integral, final gap improvement, or time-to-first-incumbent.

This path is safe because option choices affect performance, not correctness,
assuming every option remains within solver-safe semantics.

#### 4.6 Learning reusable artifacts

Repeated or structurally similar solves can reuse solver artifacts:

- primal incumbents;
- root cuts;
- root simplex bases;
- pseudocost tables;
- branching priorities;
- option presets;
- learned feature normalization statistics.

This is not neural learning in the narrow sense, but it is learning from past
optimization runs. It is highly relevant to the current codebase because
`BCResult` already exposes StrictHiGHS root cuts, root basis, and pseudocost
initialization payloads. For SCUC, artifact matching should use a structural
fingerprint of the model: topology, variable layout, row families, horizon,
generator count, and row/column dimensions. Numeric forecasts can vary while
the structural fingerprint remains stable.

### 5. Reinforcement learning interpretation

Branching, node selection, and cut selection can all be formulated as Markov
decision processes:

- State: model graph, current LP relaxation solution, incumbent, bounds,
  active nodes, pseudocosts, cut pool, and phase statistics.
- Action: branching variable, node ordering, cut subset, or option choice.
- Transition: the solver performs deterministic or near-deterministic work and
  reaches the next decision point.
- Reward: negative primal-dual gap integral, bound lift per second, incumbent
  improvement, or final solve time.

This formulation is theoretically appealing but expensive in practice. RL
changes the data distribution during training and requires many solver runs.
The recommended roadmap is therefore:

1. Build deterministic trace collection.
2. Train supervised imitation policies.
3. Deploy advisory policies with fallbacks.
4. Use offline RL or contextual bandits on logged traces.
5. Only then attempt online RL in controlled benchmarks.

### 6. Generalization, distribution shift, and safeguards

The no-free-lunch principle is directly relevant. A policy trained on 24-hour
SCUC for IEEE-39 may not generalize to IEEE-118, a different horizon, or a
storage-heavy case. The framework needs explicit safeguards:

- Train/validation/test splits by topology, horizon, load profile, renewable
  penetration, and generator outage scenario.
- Out-of-distribution checks using feature ranges, graph size, row-family
  distribution, and structural fingerprints.
- Policy confidence thresholds and fallback to baseline heuristics.
- Inference time budgets per node or per root round.
- Deterministic seeding and trace replay for regression.
- Correctness tests proving that enabling policies does not change feasibility
  validation, cut validity, or final certificate semantics.

The correct acceptance condition for an L2O policy is not merely lower average
runtime. It should improve at least one target metric without hurting solver
robustness:

- time to first feasible incumbent;
- time to target gap;
- final gap under fixed time limit;
- primal-dual gap integral;
- nodes explored;
- LP solves;
- root bound lift per second;
- cut count and LP re-solve cost;
- final objective quality;
- policy inference overhead;
- failure rate and fallback rate.

### 7. Current infrastructure analysis

#### 7.1 Engine layer

The core engine already provides the important abstractions:

- `SolverEngine` dispatches LP, QP, NLP, MILP, and MINLP models to adapters.
- `LPModel` and `MIPModel` provide sparse matrices, variable metadata, integer
  indices, binary indices, initial solutions, branching priorities, and SCUC
  hints.
- `MIPModel::UCGenHint` describes generator-time blocks, column maps, demand,
  reserve requirements, segment columns, storage data, and certificates for
  dynamic SCUC cuts.
- `BCOptions` exposes many controllable search, cut, heuristic, parallel,
  StrictHiGHS, root-warm-start, and pseudocost options.
- `BCStats` records detailed runtime behavior suitable for post-solve labels.
- `BCResult` can return root cuts, root basis, and pseudocost initialization.
- `BCWarmStart` defines a generic warm-start payload with primal hints, duals,
  basis, and cut pool records.
- `BCCallbacks` defines a public hook surface for hyperparameter tuning,
  post-solve logging, branching prior, node selection, cut selection, and
  dynamic node cuts.

Important current-state caveat: not every callback is fully honored inside the
solver today. The extended MILP overload currently uses `hyperparam_tuner` and
`post_solve`; it passes callbacks into the inner branch-and-cut path; the native
branch selector consumes `branching_prior`; and the StrictHiGHS lifecycle
consumes `dynamic_node_cut`. The `BCWarmStart` payload, `node_selector`, and
`cut_selector` are API scaffolding or only partially wired. Separately,
`MIPModel::initial_solution`, `MIPModel::branching_priority`, and `BCOptions`
root-cut/root-basis/pseudocost warm-start fields are active paths and are
already useful for L2O.

#### 7.2 Native branch-and-cut and StrictHiGHS

The repository has two important MILP execution modes:

- Native branch-and-cut with native scheduling, cuts, propagation, and optional
  vendored HiGHS LP kernel.
- StrictHiGHS mode, which delegates the full MIP lifecycle to embedded HiGHS
  while applying MIPSolvers' production options, branching priorities, root
  artifacts, MIP starts, and dynamic node-cut callback bridge.

For L2O, these modes imply different integration options:

- Static branching priorities can be injected into StrictHiGHS today.
- Dynamic learned branching inside StrictHiGHS still requires a new HiGHS branch
  callback or a deeper modification of the vendored HiGHS state machine. The
  native branch-and-cut selector now consumes dynamic priors directly.
- Dynamic certified cuts can be injected through the StrictHiGHS node-cut
  callback when `UCGenHint` certificates are present.
- Native branch-and-cut is easier to extend for learned branching, node
  ordering, and cut selection because the search loop and cut filters are under
  this repository's control.

#### 7.3 SCUC layer

SCUC is the best first domain for L2O because it has structured repeated
instances and a mature model builder:

- JSON input/output for datasets and external workflows.
- `scuc_solve` CLI and Python JSON API.
- `build_scuc_mip()` for constructing the raw MILP without solving.
- Case builders for 3-bus, 6-bus, IEEE-39, and IEEE-118 systems.
- Built-in initial state, load, wind, solar, storage, reserve, and network
  data structures.
- Existing SCUC-specific market cuts and dynamic-cut metadata.
- Per-variable branching priorities for commitment and transition variables.

The strongest initial L2O target is SCUC commitment warm-starting, followed by
solver-configuration tuning and static branch-priority learning. These can be
implemented without changing the solver's proof logic.

#### 7.4 Python and AML layer

Python is the natural training environment. The repository already provides:

- `mipsolvers.scuc` Python bindings for SCUC structs, JSON solve, and full
  pipeline solve.
- `mipsolvers.engine.solve_lp()` and `solve_milp()` thin wrappers from NumPy
  arrays.
- AML Python bindings for building optimization models and reading solutions,
  duals, and reduced costs.

The Python layer now exposes the first L2O surfaces needed for training and
evaluation, including SCUC case builders, generator-time features, model
fingerprints, detailed branch-and-cut stats, warm-start construction, solver
configuration policies, artifact reuse, and learned SCUC branching priorities.
Remaining surfaces still needed for deeper learning are:

- `build_scuc_mip()`;
- branch/cut/node traces;
- callback registration or policy inference handles.

The implementation should add a Python-side `mipsolvers.l2o` layer rather than
force all training logic into C++.

#### 7.5 Benchmark layer

The benchmark runner already builds SCUC MILPs, runs multiple StrictHiGHS and
native configurations, audits solutions, records nodes, gaps, objectives,
bounds, and JSON output. This is a strong base for L2O evaluation. The missing
piece is trace-level data: per-node, per-candidate, per-cut, and per-phase
events.

## Part II: Implementation Plan

### Phase 0: Stabilize the L2O contract and documentation

Objectives:

- Make the current public hook contract explicit.
- Separate active hooks from planned hooks.
- Define correctness rules for L2O policies.

Tasks:

1. Update engine documentation to use canonical callback names:
   `hyperparam_tuner`, `branching_prior`, `node_selector`, `cut_selector`,
   `dynamic_node_cut`, and `post_solve`.
2. Document which callback paths are active today and which are scaffolding.
3. Define policy safety modes:
   `Off`, `TraceOnly`, `Advisory`, `Guarded`, and `Experimental`.
4. Define policy metadata: name, version, model hash, feature schema version,
   training distribution, inference budget, and fallback policy.

Deliverable:

- A documented L2O contract that no learned policy can bypass feasibility,
  bound, or cut-validity checks.

### Phase 1: Build trace and dataset infrastructure

Objectives:

- Collect reproducible training data from existing solvers.
- Avoid coupling training to live solver internals too early.

Recommended layout:

```text
include/mipsolvers/l2o/
  feature_schema.hpp
  trace_event.hpp
  trace_writer.hpp
  model_fingerprint.hpp
  policy.hpp

src/l2o/
  feature_schema.cpp
  trace_writer.cpp
  model_fingerprint.cpp
  policy.cpp

python/mipsolvers/l2o/
  datasets.py
  featurize.py
  train_warm_start.py
  train_branching.py
  evaluate.py
  export.py

examples/l2o/
  collect_scuc_traces.py
  train_scuc_warm_start.py
  evaluate_scuc_policy.py
```

Trace event types:

- `instance_start`: dimensions, hash, SCUC metadata, options.
- `root_lp`: LP status, objective, fractional count, reduced-cost summary,
  dual summary, root bound.
- `branch_decision`: candidate variables, features, solver score, selected
  variable, branch direction, pseudocosts, optional strong-branch labels.
- `node_selected`: active node summaries, chosen node, node depth, lower bound,
  estimate, incumbent state.
- `cut_round`: generated cut candidates, cut features, accepted mask, bound
  lift, LP re-solve time, rejected reasons.
- `incumbent`: objective, source, node, depth, repair stats.
- `solve_end`: final stats, artifacts, status, failure reason if any.

Storage formats:

- JSONL for easy debugging and small traces.
- Compressed binary or Parquet-like format for large training runs later.
- Sparse graph data as CSR arrays: row starts, indices, values, row features,
  variable features, edge features, candidate masks.

Implementation notes:

- Start with post-solve and root-level traces, then add node-level traces.
- Use deterministic seeds and record all options.
- Sample expensive labels such as strong branching under explicit budgets.
- Include `model_fingerprint` so artifacts can be reused only on compatible
  model layouts.

### Phase 2: Expose training surfaces to Python

Objectives:

- Let Python generate data, train models, and evaluate policies.
- Keep production solver logic in C++.

Tasks:

1. Add Python bindings for SCUC case builders:
   `build_3bus_case`, `build_6bus_case`, `build_ieee39_case`,
   `build_ieee118_case`, and `scuc_input_to_json`.
2. Expose `build_scuc_mip()` or a safer feature-export wrapper for SCUC MILPs.
3. Expose `BCOptions` fields needed by benchmark and L2O experiments.
4. Expose detailed `BCStats` and `BCResult` artifacts in a stable Python dict.
5. Make `mipsolvers.engine.solve_milp()` honor `mip_gap` and `time_limit_sec`;
   the current Python wrapper accepts these arguments but does not pass them
   into the engine options.
6. Add a direct `solve_milp_bc()` Python binding for research runs with
   `BCOptions`, trace output path, and optional initial solution.

Deliverable:

- Python can generate SCUC datasets, solve baseline cases, export traces, and
  train models without writing one-off C++ benchmark code.

### Phase 3: Implement the first low-risk policy: SCUC warm-start learning

Objectives:

- Improve time to incumbent and pruning using a learned commitment seed.
- Avoid any changes to proof logic.

Pipeline:

1. Generate SCUC instances from case builders and perturbations:
   load profiles, renewable forecasts, outages, bids, reserve requirements,
   line limits, storage settings, and initial states.
2. Solve each instance with trusted baselines such as Gurobi, direct HiGHS,
   StrictHiGHS, or the current best native configuration.
3. Extract labels: commitment `IG(g,t)`, startup `SU(g,t)`, shutdown `SD(g,t)`,
   dispatch summaries, objective, gap, solve status.
4. Train a model that predicts binary commitment and transition variables.
5. Convert predictions into an `Eigen::VectorXd` initial solution:
   round confident binaries, leave uncertain variables repairable, initialize
   continuous dispatch from a deterministic economic dispatch or zeros.
6. Use the existing `MIPModel::initial_solution` path.
7. Let the solver validate and repair; reject infeasible starts.

Model options:

- Baseline MLP over generator-time features.
- Temporal model over each generator's horizon.
- SCUC graph model with generator, time, bus, and constraint edges.

Evaluation:

- acceptance rate of predicted starts;
- time to first incumbent;
- incumbent objective at fixed times;
- final gap under 5s, 30s, 60s, and 300s limits;
- solve-time overhead of prediction and repair;
- performance under topology and forecast shifts.

Why this phase should come first:

- It uses active infrastructure today.
- It cannot invalidate optimality certificates.
- It produces visible benefits even before branch/cut callbacks are fully wired.

Implemented Phase 3 API surface:

- C++ correctness boundary:
  `mipsolvers::l2o::make_scuc_commitment_warm_start()` converts predicted
  `[ng][T]` commitment, startup, and shutdown matrices into
  `MIPModel::initial_solution` using `MIPModel::UCGenHint` column maps. If
  startup or shutdown are not supplied, transitions are inferred from predicted
  commitment and the initial unit status.
- Python dataset surface:
  `mipsolvers.l2o.scuc_generator_time_features(input)` returns a dependency-free
  generator-time feature tensor for training, and
  `mipsolvers.l2o.scuc_commitment_labels(result)` returns NumPy labels from an
  `SCUCSolveResult`.
- Python policy-evaluation surface:
  `mipsolvers.l2o.make_scuc_commitment_warm_start(input, commitment, ...)`
  returns the initial-solution vector and a conversion report, while
  `mipsolvers.l2o.solve_scuc_mip(input, ...)` runs the comparable no-learned-
  warm-start baseline and
  `mipsolvers.l2o.solve_scuc_mip_with_warm_start(input, commitment, ...)`
  evaluates a predicted schedule through the existing C++ branch-and-cut path.
  The warm-start helpers can also preserve a caller-provided full incumbent via
  `initial_solution` before overwriting the learned `IG/SU/SD` entries.

### Phase 4: Learn solver configuration and artifact reuse

Objectives:

- Use current `hyperparam_tuner` and `post_solve` hooks productively.
- Learn option presets for model families and time limits.

Tasks:

1. Add a training table of `(features, options, stats)` from benchmark runs.
2. Define a compact feature vector using `BCInstanceFeatures`, SCUC metadata,
   structural fingerprint, previous-run stats, and time limit.
3. Train an algorithm selector or contextual bandit for option presets.
4. Deploy through `BCCallbacks::hyperparam_tuner`.
5. Record outcomes through `post_solve` for continuous improvement.
6. Add artifact cache logic for root cuts, root basis, and pseudocost init:
   only reuse artifacts when the structural fingerprint and dimensions match.

Candidate decisions:

- StrictHiGHS vs native branch-and-cut.
- Simplex vs IPM root settings.
- Root cut budgets and separation caps.
- Heuristic effort.
- Pseudocost reliability threshold.
- Symmetry detection.
- Root-cut, root-basis, and pseudocost warm-start reuse.

This phase directly targets current known bottlenecks in large SCUC cases:
root processing, root cuts, basis warm-starting, and pseudocost quality.

Implemented Phase 4 API surface:

- C++ policy and cache boundary:
  `mipsolvers::l2o::SolverConfigPolicyOptions` defines conservative option
  presets for SCUC-like short-budget and artifact-reuse runs;
  `mipsolvers::l2o::SolverArtifactCache` stores root cuts, root basis, and
  pseudocost artifacts keyed by the L2O structural fingerprint. Artifacts are
  deep-copied from `BCResult` and are applied only when the fingerprint and
  row/column dimensions validate against the current `MIPModel`.
- Callback deployment:
  `mipsolvers::l2o::make_solver_config_callbacks()` returns `BCCallbacks` with
  an active `hyperparam_tuner` and `post_solve` recorder. The tuner adjusts
  existing safe `BCOptions` knobs such as root separation caps, heuristic
  effort, presolve controls, pseudocost reliability, and deterministic
  evaluation settings. The recorder produces `(features, options, stats)` run
  records for later contextual-bandit or algorithm-selection training.
- Python evaluation surface:
  `mipsolvers.l2o.SolverArtifactCache()` holds reusable artifacts across solves,
  `mipsolvers.l2o.SolverConfigPolicyOptions()` exposes the policy knobs,
  `mipsolvers.l2o.scuc_solver_config_features(input, options, ...)` exports the
  compact feature dictionary, and
  `mipsolvers.l2o.solve_scuc_mip_with_config_policy(input, options, ...,
  artifact_cache=cache)` solves through the C++ branch-and-cut path with Phase 4
  tuning and optional artifact refresh.
- Reproducible numerical driver:
  `docs/tutorial/bench_06_l2o_phase4_config_artifacts.py` benchmarks the IEEE-118
  `T=4` wind+solar case by first populating a compatible artifact cache and then
  comparing baseline fixed-budget solves against Phase 4 configuration plus
  root-cut artifact reuse.

### Phase 5: Wire learned branching priorities and dynamic branching priors

Objectives:

- Move from static SCUC hand-coded priorities to learned priorities.
- Keep learned branching as a soft prior blended with pseudocosts.

Stage 5A: Static priority learning

1. Train a model to produce original-column priority scores before solve.
2. Write scores into `MIPModel::branching_priority`.
3. Use existing StrictHiGHS priority injection and native priority mapping.
4. Evaluate against the current hand-coded commitment/startup/shutdown
   priorities.

Stage 5B: Native dynamic branching prior

1. Finish wiring `BCCallbacks::branching_prior` in the native branch selector.
2. Add `BCBranchContext` features: node id, depth, LP solution, candidate set,
   incumbent, node lower bound, pseudocosts, and branch history.
3. Blend learned scores with solver scores under `BCOptions` controls:
   enabled flag, weight, max inference time, minimum candidate count, fallback.
4. Add trace-only mode that computes learned choices but does not use them.
5. Add regression tests proving no-op and zero-weight policies preserve
   baseline behavior.

Stage 5C: StrictHiGHS dynamic branching research path

- If needed, add a vendored HiGHS branch-candidate callback. This is a deeper
  change and should wait until static priorities and native dynamic priors
  demonstrate value.

Phase 5 theoretical objective: branch-regret learning

The neural model should not be trained merely to predict whether a commitment
variable is 0 or 1 in an incumbent solution. Incumbent labels are useful for
warm starts, but branching is a different decision problem: at a branch-and-cut
node, the solver must choose one variable from the current fractional candidate
set so that the resulting search tree proves the bound or finds a good incumbent
faster. A Phase 5 branching model should therefore learn a ranking over branch
candidates, not a standalone binary commitment classifier.

For a minimization MILP, consider a branch node `i` with LP lower bound `b_i`,
candidate set `C_i`, incumbent upper bound `U_i`, and candidate variable `j`.
If we temporarily branch on `j`, solve the two child LP relaxations, and obtain
child bounds `b_ij_down` and `b_ij_up`, define the one-step bound gains

```text
g_ij_down = max(0, min(b_ij_down, U_i) - b_i)
g_ij_up   = max(0, min(b_ij_up,   U_i) - b_i)
```

The useful branching score should reward both children becoming strong. A
robust strong-branching utility is

```text
u_ij = min(g_ij_down, g_ij_up) + beta * max(g_ij_down, g_ij_up)
```

with `0 <= beta <= 0.2`. The `min` term avoids choosing a variable where only
one side improves the bound, and the small `max` term breaks ties. If a child LP
is infeasible or pruned by the incumbent, its gain is capped by `U_i - b_i` or
by a configured large finite value. This target is closer to the real objective
than incumbent membership because it measures immediate proof progress.

Let the neural network produce a logit `z_ij = f_theta(phi_i, psi_ij)` for each
candidate, where `phi_i` are node/instance features and `psi_ij` are
candidate-variable features in original column space. Convert logits to a
distribution over the candidate set:

```text
p_ij(theta) = exp(z_ij / tau_model) / sum_{k in C_i} exp(z_ik / tau_model)
```

Convert expert strong-branching utilities to a soft target distribution:

```text
q_ij = exp(u_ij / tau_label) / sum_{k in C_i} exp(u_ik / tau_label)
```

The recommended primary training loss is the weighted listwise branch-regret
cross entropy

```text
L_rank(theta) = sum_i w_i * sum_{j in C_i} -q_ij * log p_ij(theta)
```

where `w_i` should emphasize important nodes. A practical weight is

```text
w_i = gap_i * depth_discount_i * reliability_i
gap_i = max(0, U_i - b_i) / max(1, abs(U_i))
depth_discount_i = 1 / sqrt(1 + depth_i)
```

and `reliability_i` is higher when all or most candidates were probed by strong
branching, lower when labels are sampled or delayed. This loss has the right
invariance: only the within-node ordering matters, so arbitrary scaling across
instances does not dominate training.

Two auxiliary losses are useful but should not replace the ranking loss:

```text
L_pair(theta) = sum_i w_i * sum_{j,k in C_i, u_ij > u_ik}
           max(0, margin - (z_ij - z_ik))

L_value(theta) = sum_i w_i * sum_{j in C_i}
            huber(normalize(z_ij), normalize(u_ij))
```

`L_pair` improves top-candidate separation. `L_value` calibrates scores for
blending with pseudocosts. The total objective should be

```text
L(theta) = L_rank(theta) + lambda_pair * L_pair(theta)
        + lambda_value * L_value(theta) + lambda_l2 * ||theta||_2^2
```

The current NumPy MLP benchmark implements the first branch-derived label path:
it trains generator-time `IG/SU/SD` scores from pseudocost-gain artifacts
exported after a branch-and-bound run. This is still a static distillation, not
the final dynamic candidate-set `L_rank` implementation above, but it is no
longer an incumbent warm-start proxy. The script keeps incumbent labels only as
an explicit fallback for smoke tests via `--label-source incumbent` or
`--allow-incumbent-fallback`.

Phase 5 dataset generation for training

The training set should be generated from solver behavior, not from hand-coded
feature weights. Each sample is a branch-node candidate set with expert utility
labels:

```text
sample_i = {
  instance_id, fingerprint, solver_path,
  node_id, depth, parent_bound, incumbent_bound,
  candidates_original_cols: [j1, j2, ...],
  candidate_features: [psi_ij1, psi_ij2, ...],
  node_features: phi_i,
  expert_utilities: [u_ij1, u_ij2, ...],
  baseline_choice_original_col,
  presolve_mapping_version
}
```

Dataset construction should follow this pipeline:

1. Sample a distribution of SCUC instances, not a single case. Use IEEE-39 and
  IEEE-118 horizons such as `T=4, 8, 12, 24`; wind-only, solar-only,
  wind+solar, and base cases; load/renewable perturbation seeds; and several
  solver budgets. Split train/validation/test by instance seed and scenario,
  not by node, to avoid leaking nearly identical branch nodes across splits.
2. Run the native branch-and-cut path in trace mode. At each selected node,
  record original-space candidate columns, LP values, fractionality, objective
  coefficient, variable type, SCUC role (`IG`, `SU`, `SD`, `PG`), generator and
  time indices when available, existing static priority, pseudocost statistics,
  node depth, parent bound, incumbent bound, global gap, and branch history.
  The presolve mapping must be stored so reduced-space solver internals can be
  reconstructed in original column space.
3. Generate expert labels. Preferred labels come from strong branching: for each
  candidate or a sampled top-K candidate subset, temporarily solve the down and
  up child LP relaxations under deterministic probe limits and compute `u_ij`.
  When probing every candidate is too expensive, mix full probing near the root
  with sampled probing deeper in the tree and store `reliability_i` so the loss
  can down-weight incomplete labels.
4. Add lower-quality fallback labels only when strong-branching labels are not
  available: baseline branch choices as imitation labels, pseudocost gain
  observations from actual children, or delayed labels based on subtree bound
  improvement and pruning. These fallback labels should be tagged by
  `label_source` and given smaller reliability weights. The implemented
  benchmark now uses this path by default with `label_source=pseudocost`: it
  reads original-column `pseudocostup`, `pseudocostdown`, sample counts,
  inference counts, and conflict scores from the C++ artifact, then forms
  observed labels

```text
u_j = min(pc_down_j, pc_up_j) + beta * max(pc_down_j, pc_up_j)
    + 0.05 * conflict_j
```

  followed by `log1p` normalization over observed entries. Entries without
  pseudocost evidence receive zero sample weight, so the neural loss is driven
  by variables touched by actual branch-and-bound behavior.
5. Store the dataset as JSONL metadata plus array blocks. JSONL is convenient
  for variable-size candidate sets and auditability; dense arrays or NPZ files
  are better for mini-batch training. Every row must include the structural
  fingerprint, solver version, random seed, presolve setting, and coordinate
  convention.
6. Train the dynamic candidate-ranker with `L_rank`. For StrictHiGHS, which
  currently only accepts static priorities, distill the dynamic labels into an
  original-column static score by aggregating node utilities:

```text
S_j = sum_{i: j in C_i} w_i * q_ij
```

  Then map `S_j` through `UCGenHint` to generator-time scores for `IG/SU/SD`.
  This static distillation is only a deployment approximation; the native
  dynamic callback should consume the candidate-ranker directly.
7. Validate with solver metrics, not only validation loss. A model is accepted
  only if fixed-budget validation solves improve or preserve gap, best bound,
  node count, or time-to-incumbent on held-out instances. Root-dominated gates
  should reject the policy as inconclusive, because they do not test branching.

Implemented Phase 5 API surface:

- C++ policy boundary:
  `mipsolvers::l2o::SCUCBranchingPolicyOptions` controls static priority
  installation, startup/shutdown inclusion, learned score scaling, dynamic
  prior weight, and fallback behavior. `make_scuc_branching_priorities()`
  converts a learned `[ng, T]` generator-time score matrix into an original
  column priority vector using `MIPModel::UCGenHint`. `apply_scuc_branching_priorities()`
  installs that vector on `MIPModel::branching_priority`.
- Dynamic native branch prior:
  `make_scuc_branching_callbacks()` returns `BCCallbacks` with a deterministic
  `branching_prior` callback. The native selector overloads in
  `choose_branch_var()` and `choose_branch_var_pseudocost()` evaluate the
  callback on the current `BCBranchContext`, blend finite callback scores with
  pseudocost or infeasibility scores, and fall back to the existing selector on
  missing, malformed, zero, or throwing callbacks. Static priority remains
  lexicographic, so the learned dynamic prior reranks candidates only within the
  safe priority tier selected by the solver. Callback-facing coordinates are
  original model columns: when PaPILO presolve removes variables, the native
  search maps reduced branch candidates back through `reduced_to_orig_col`
  before invoking the dynamic prior.
- Python evaluation surface:
  `mipsolvers.l2o.SCUCBranchingPolicyOptions()` exposes the policy knobs,
  `mipsolvers.l2o.make_scuc_branching_priorities(input, scores, ...)` returns
  the priority vector plus validation report, and
  `mipsolvers.l2o.solve_scuc_mip_with_branching_policy(input, scores, ...)`
  solves with installed static priorities and an advisory dynamic branch-prior
  callback when the native selector asks for scores.
- Regression coverage:
  `test_l2o_trace` now checks that learned SCUC scores map to commitment,
  startup, and shutdown columns, and that a dynamic prior reranks equal-priority
  branch candidates without affecting the no-callback fallback path.
- Reproducible numerical driver:
  `docs/tutorial/bench_07_l2o_phase5_branching.py` now trains or loads a
  dependency-free NumPy MLP before producing scores. Label solves are generated
  through `mipsolvers.l2o.solve_scuc_mip()` with artifact vectors enabled for
  pseudocost training. The default label source is branch-derived pseudocost
  gain evidence in original column space, converted into weighted soft labels
  for commitment, startup, and shutdown outputs. The branch score passed to C++
  is the maximum neural output probability, not a hand-weighted feature formula.
  The script persists the model artifact under
  `build_mipsolvers/l2o_phase5_branching_mlp.json`, supports `--reuse-model`,
  and gates the learned policy with a validation solve before using it. By
  default the gate rejects root-dominated validation runs with no branch-tree
  evidence.

Phase 5 numerical validation on IEEE-118, `T=24`, wind+solar:

The larger benchmark has 26,736 variables, 3,888 binary variables, 32,215
inequality rows, 2,784 equality rows, and 553,035 inequality nonzeros. Policy
installation produced 3,888 priority entries: 1,296 commitment priorities and
2,592 startup/shutdown priorities, with no missing UC columns. StrictHiGHS uses
the static priority vector; the dynamic `branching_prior` callback is installed
for native branch-and-cut but is not a HiGHS branch callback.

The previous hand-weighted score recipe has been removed from the benchmark.
The refreshed run trained a 21-input, 32-hidden-unit, 3-output MLP from
IEEE-118 `T=24` wind+solar branch-derived pseudocost labels. The label solve
used StrictHiGHS with a 120s budget, reached objective `166551.9551168032`, gap
`0.1396771321`, and explored 6 branch nodes. The exported pseudocost artifact
provided 3,664 observed UC label entries out of 3,888 possible entries, with
total label weight `1291.5`, pseudocost sample total `1.0`, inference total
`1.0`, and pseudocost cost total `4625.1480309906`. The loss was weighted soft
binary cross entropy on the pseudocost utility labels; after 160 epochs the
training loss was `0.4429196524` and the validation loss was `0.5710140892`.
On the same IEEE-118 `T=24` wind+solar case, the neural score range was
`[0.1991313019, 0.5757023770]` with mean `0.3581154565`, and policy
installation produced all 3,888 priority entries.

The 60s validation gate was root dominated: both baseline and neural-priority
runs had `nodes_explored=0`, objective `166551.9551168032`, and gap
`0.1415317491`. The strengthened gate therefore rejected the neural policy
because it had no branch-tree evidence. The accepted gated result falls back to
the baseline, while the raw neural diagnostic at 120s is reported separately
with the gate disabled to show branch-tree behavior without claiming it as an
accepted production policy.

| Run | Budget | Baseline gap | Neural/gated gap | Baseline solver s | Policy solver s | Nodes baseline/policy | Neural used |
|---|---:|---:|---:|---:|---:|---:|:---:|
| Gated validation result | 60 s | 0.1415317491 | 0.1415317491 | 60.026166 | 60.026166 | 0 / 0 | false |
| Raw neural diagnostic, gate disabled | 120 s | 0.1396771321 | 0.1404223474 | 120.022275 | 120.021611 | 5 / 17 | true |

The current neural model is therefore real and branch-derived, but not yet good
enough to deploy on the branchy `T=24` case. It changes branching behavior at
120s, increasing explored nodes from 5 to 17, but slightly worsens the bound gap
from `0.1396771321` to `0.1404223474`. This is exactly why the validation gate
must remain in place: weak neural artifacts should fall back instead of
perturbing production solves. The next modeling step is to collect explicit
candidate-set traces for `L_rank`: actual baseline branch decisions,
deterministic strong-branching probe scores, pseudocost gains by candidate and
node, and successful branch histories across multiple IEEE-118 seeds and
scenarios.

The earlier `T=4` smoke benchmark remains useful only as an API integration
check: it installs 648 priority entries but is root dominated in short budgets
and reports `nodes_explored=0`, so it cannot validate branching behavior.
Future Phase 5 experiments should combine the T=24 setup with Phase 4 artifact
reuse, broaden the pseudocost/strong-branching dataset, and compare nodes, bound
progress, and gap at equal wall-clock budgets on held-out instances.

Native-first L2O pivot after Phase 5:

The Phase 5 experiments make the bottleneck clearer. StrictHiGHS is a strong
production and validation path, but it is a weak teacher for learning policies
when the IEEE-118 `T=24` solve spends most of its budget in the root lifecycle
and often reaches the time limit with zero or very few branch nodes. That means
the run is expensive but produces too few branch, node, and cut decisions to
train a neural policy. Gurobi has the opposite issue: it is an excellent
end-to-end benchmark and can provide incumbent or objective labels, but its
internal branch, cut, and node decisions are black-box and cannot be traced in
the coordinate contract needed by MIPSolvers.

The L2O data-generation path should therefore pivot back to native
branch-and-cut with the self-developed LP kernel. In this mode MIPSolvers owns
the LP basis, reduced costs, fractional candidate set, cut candidates, node
queue, pseudocost updates, and presolve coordinate mapping. Even if this path is
not yet the fastest production solver, it is the right research teacher because
it can expose dense supervised labels for branching, cut selection, node
selection, and solver-state representation learning.

The intended solver roles are now:

- Native branch-and-cut with `BCOptions::lp_kernel_backend=LpKernelBackend::ExperimentalNative`:
  primary L2O trace generator and policy-integration workbench. It uses the
  in-repository dual-simplex/IPM machinery, gives full observability, and can
  call learned callbacks at every native decision point.
- StrictHiGHS: production-style validation baseline and artifact source for
  root cuts, basis, pseudocost summaries, and final fixed-budget comparisons.
- Gurobi: external end-to-end oracle for objective quality and incumbent labels
  when available, not the source of internal branch/cut/node traces.

The Phase 5 benchmark now exposes this distinction explicitly. Use

```bash
PYTHONPATH=build_mipsolvers /Users/tianyangzhao/.pyenv/versions/3.8.20/bin/python3.8 \
  docs/tutorial/bench_07_l2o_phase5_branching.py \
  --native --native-lp-kernel self --periods 24 --label-source pseudocost
```

for native self-kernel L2O experiments. The optional
`--native-lp-kernel highs` mode is a diagnostic bridge that keeps the native
branch-and-cut search owner but swaps the LP numerical oracle back to vendored
HiGHS. StrictHiGHS remains the default when `--native` is omitted.

### Phase 6: Wire learned cut selection and certified dynamic cuts

Objectives:

- Reduce LP burden from weak cuts while preserving useful bound lift.
- Use learned models only where validity is deterministic.
- Make native self-kernel branch-and-cut the first-class training path for cut
  data, because it exposes generated cuts, rejected cuts, basis state, row
  activity, and post-cut LP bound movement.

Rationale:

StrictHiGHS can prove useful production behavior, but the root-dominated
IEEE-118 runs do not yield enough labeled cut and tree decisions for neural
training. Native self-kernel runs are more instrumentable: every generated cut
candidate can be logged before deterministic filters, every accepted cut can be
matched to an LP re-solve, and every rejected cut can carry a reason. This is
the right setting for learning a cut selector, even if deployment later uses a
guarded subset of the policy in StrictHiGHS or a production native profile.

Native cut selector path:

1. Run the native branch-and-cut path with the self-developed LP kernel:
   `lp_kernel_backend=LpKernelBackend::ExperimentalNative`, `use_simplex_lp_nodes=true`, and
   deterministic single-thread settings during trace generation.
2. Wire `BCCallbacks::cut_selector` into the candidate-cut admission pipeline.
3. Expose `BCCutCandidate` features for every generated candidate: cut family,
   validity scope, violation, efficacy, density, support on binary variables,
   objective parallelism, row age, LP basis availability, depth, incumbent gap,
   and estimated re-solve cost.
4. Let the callback return an accept mask, per-family budget, or threshold
   multiplier. Start with per-family budgets because they are lower risk than
   selecting arbitrary individual cuts.
5. Keep deterministic validity, efficacy, numerical, and parallelism filters
   after the learned selector.
6. Add trace-only mode and no-op tests.
7. Store labels from actual bound movement:

```text
cut_reward = bound_lift_after_resolve / max(1, resolve_time_ms)
           - alpha * added_nonzeros
           - beta  * numerical_rejection_indicator
```

   The model should learn to preserve cuts with positive bound lift per LP cost,
   not merely cuts with large immediate violation.

StrictHiGHS and SCUC dynamic cut path:

1. Keep using `dynamic_node_cut` only for certified rows.
2. Allow a policy to choose which certified SCUC cut family to generate and
   how much budget to spend.
3. Do not allow arbitrary neural cut coefficients unless a deterministic
   validity oracle is implemented.
4. Use `ValidityScope` correctly: global cuts, local node cuts, and lazy
   constraints have different proof obligations.

Recommended first cut policy:

- Learn per-family budgets and thresholds from existing cut features and
  `BCStats` rather than selecting from all individual cuts. This captures much
  of the performance benefit with lower action-space complexity.
- Train and debug this policy only on native self-kernel traces first. Promote
  it to StrictHiGHS-style validation only after trace replay shows that the
  learned selector preserves or improves root bound lift and fixed-budget gap on
  held-out cases.

### Phase 7: Wire learned node selection

Objectives:

- Improve primal-bound discovery in the early tree.
- Preserve best-bound proof behavior later.

Tasks:

1. Wire `BCCallbacks::node_selector` into the native node queue.
2. Add open-node features: depth, lower bound, estimate, branch path summary,
   local domain deltas, fractional count, and incumbent distance features.
3. Add a policy schedule:
   use learned node ordering until first incumbent or until a node/time budget;
   then switch to hybrid or best-first.
4. Train from traces using labels from incumbent-improving paths or optimal
   solution containment when available.
5. Evaluate primal-bound improvement and final proof time separately.

Node selection should come after warm-start and branching because it has higher
interaction risk and requires richer traces.

### Phase 8: Add an inference runtime

Objectives:

- Support fast, deterministic policy inference in C++ without imposing heavy
  dependencies on default builds.

Recommended design:

- Define a C++ `L2OPolicy` interface with pure virtual methods:
  `score_branch_candidates`, `score_nodes`, `select_cuts`,
  `predict_warm_start`, and `tune_options`.
- Provide a `NoOpPolicy` and `RuleBasedPolicy` in core.
- Add optional model backends behind CMake flags:
  `MIPSOLVERS_BUILD_L2O=ON`, `MIPSOLVERS_L2O_ONNX=ON`.
- Use ONNX Runtime only as an optional dependency. The default build should
  compile without it.
- For small MLPs, consider a lightweight built-in JSON weight loader to avoid
  runtime dependency overhead.

Policy loading should validate:

- model file exists;
- feature schema version matches;
- expected input dimensions match;
- structural domain matches or OOD fallback is allowed;
- inference latency stays under configured budget.

### Phase 9: Testing and benchmarking

Correctness tests:

- No-op policy returns identical results to baseline where determinism allows.
- Zero-weight learned branch prior preserves baseline branch choices.
- Invalid warm starts are rejected.
- Invalid dynamic cuts are rejected or never generated.
- Policy inference exceptions trigger fallback, not solver failure.
- Artifact reuse rejects dimension or fingerprint mismatch.

Performance tests:

- SCUC 3-bus and 6-bus smoke tests for all policy modes.
- IEEE-39 validation for realistic medium cases.
- IEEE-118 benchmark for large-case behavior.
- Fixed time-limit comparisons at 5s, 30s, 60s, and 300s.
- Trace replay tests for branch/cut/node event schemas.

Metrics to report:

- objective;
- best bound;
- MIP gap;
- runtime;
- nodes explored;
- LP solves;
- root cuts and tree cuts;
- first incumbent time and node;
- incumbent improvements;
- root bound lift;
- policy inference time;
- fallback count;
- artifact reuse hit rate.

### Phase 10: Suggested milestone order

1. Documentation and API cleanup.
2. Python exposure of case builders, `BCOptions`, `BCStats`, and benchmark
   result dictionaries.
3. Trace writer and root/post-solve dataset collection.
4. SCUC warm-start predictor using existing `MIPModel::initial_solution`.
5. Hyperparameter tuner using existing `hyperparam_tuner` and `post_solve`.
6. Artifact cache for root cuts, basis, and pseudocost data.
7. Static learned branch priorities through `MIPModel::branching_priority`.
8. Native dynamic `branching_prior` wiring.
9. Native self-kernel L2O trace mode for branch, cut, and node decisions.
10. Native `cut_selector` wiring and cut-reward labels.
11. Native `node_selector` wiring.
12. Optional StrictHiGHS dynamic branching callback research.

This order delivers useful value early while postponing high-risk changes until
the data, telemetry, and fallback machinery are mature.

## Recommended Initial MVP

The first complete MVP should be:

```text
SCUC trace collection -> train commitment warm-start model -> export model ->
predict MIP start -> solve with StrictHiGHS/native -> compare fixed-time gaps
```

MVP scope:

- Python scripts generate perturbed SCUC cases from the built-in case builders.
- Baseline solvers generate labels and JSONL training records.
- A small model predicts commitment/startup/shutdown binaries.
- C++ or Python converts predictions into `MIPModel::initial_solution`.
- Existing solver validation accepts or rejects the incumbent.
- Benchmark runner reports time-to-incumbent, final gap, nodes, and overhead.

This MVP uses the strongest available infrastructure and avoids the hardest
callback work. It also creates the dataset needed for later branching, search,
and cut-selection policies.

## Open Engineering Decisions

1. Policy runtime: optional ONNX Runtime vs lightweight built-in MLP inference.
2. Trace storage: JSONL only initially vs adding a binary graph format early.
3. Python packaging: extend the current pybind module vs create a separate
   Python package that imports the compiled module.
4. StrictHiGHS dynamic branching: modify vendored HiGHS now or defer until
   native dynamic branching proves value.
5. Training source of truth: use Gurobi labels when available, or rely on
   HiGHS/StrictHiGHS/native traces for open-source reproducibility.
6. Artifact fingerprints: strict equality of structure vs tolerant matching
   for same topology with changed numeric profiles.

## Practical Risks

- A learned policy can overfit to case-builder distributions and regress on
  real market data.
- GNN inference can cost more than it saves if called at every node.
- Strong-branching labels can be biased by the baseline search distribution.
- Learned cut selection can reduce LP size but weaken bound progression.
- Dynamic cuts are correctness-sensitive; only certified cut families should be
  used initially.
- Parallel search can make traces less reproducible unless deterministic mode
  and fixed seeds are used.
- Current docs and headers have some naming drift around ML hooks; the public
  contract should be cleaned up before adding more policies.

## Final Recommendation

Implement L2O in MIPSolvers as a layered, guarded policy framework:

1. Use Python for data generation, training, and evaluation.
2. Use C++ for deterministic feature extraction, inference, validation, and
   solver integration.
3. Start with SCUC warm starts and option tuning because those paths are active
   and proof-safe.
4. Add learned branching, cut selection, and node selection only after trace
   infrastructure and fallback tests exist.
5. Treat every learned output as advisory unless a deterministic optimization
   certificate validates it.

This approach matches the theory in `docs/LearningtoOptimize/`, exploits the
current solver infrastructure, and keeps the core solver reliable while making
room for data-driven acceleration.
