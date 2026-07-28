# MIPSolvers Engine — Reference Documentation

The **MIPSolvers engine** is a unified C++ solver dispatcher that routes mathematical
programming problems to the best available solver backend. It supports linear systems,
LP, QP, NLP, MILP, and MINLP, and abstracts away solver-specific APIs behind a single
`SolverEngine` class.

---

## Table of Contents

1. [Quick Start](#1-quick-start)
2. [Problem Types](#2-problem-types)
3. [SolverEngine API](#3-solverengine-api)
4. [Solve Options](#4-solve-options)
5. [Result Structure](#5-result-structure)
6. [Solver Adapters](#6-solver-adapters)
7. [Native Branch-and-Cut Engine](#7-native-branch-and-cut-engine)
   - [BCOptions Reference](#71-bcoptions-reference)
   - [Branching and Node Selection](#72-branching-and-node-selection)
   - [Cut Generation](#73-cut-generation)
   - [Warm-Start](#74-warm-start)
   - [Cut Validity Scope](#75-cut-validity-scope)
   - [Dynamic Node Cuts](#76-dynamic-node-cuts)
   - [ML Hooks](#77-ml-hooks)
8. [Strategy Dispatcher](#8-strategy-dispatcher)
9. [Include Layout](#9-include-layout)

---

## 1. Quick Start

```cpp
#include "mipsolvers/engine/engine.hpp"

using namespace mipsolvers::engine;

// Build an LP
LPModel lp;
lp.sense = Sense::Minimize;
lp.c = Eigen::VectorXd::Ones(2);          // min x0 + x1
// Aeq * x = beq  (x0 + x1 = 1)
lp.Aeq.resize(1, 2);
lp.Aeq.insert(0, 0) = 1.0;
lp.Aeq.insert(0, 1) = 1.0;
lp.Aeq.makeCompressed();
lp.beq = Eigen::VectorXd::Constant(1, 1.0);
lp.vars = {{VarType::Continuous, 0.0, 1.0},
           {VarType::Continuous, 0.0, 1.0}};

// Solve
SolverEngine eng;
eng.register_default_adapters();
auto result = eng.solve_lp(lp);

if (result.stats.success) {
    std::cout << "objective = " << result.stats.objective << "\n";
    std::cout << "x = " << result.x.transpose() << "\n";
}
```

For MILP, mark variable types as `Binary` or `Integer` in `vars`, then use
`solve_milp` with a `MIPModel`:

```cpp
MIPModel mip;
mip.linear_part = lp;                    // same LPModel fields
mip.binary_idx  = {0};                   // variable 0 is binary
auto result = eng.solve_milp(mip);
```

---

## 2. Problem Types

All problem types live in `mipsolvers::engine` (include `problem_types.hpp`).

### `ProblemClass` enum

| Enumerator | Description |
|---|---|
| `LE` | Sparse linear system $Ax = b$ |
| `NLE` | Nonlinear system $F(x) = 0$ |
| `LP` | Linear program |
| `QP` | Convex quadratic program (LCQP) |
| `NLP` | Nonlinear program |
| `MILP` | Mixed-integer linear program |
| `MINLP` | Mixed-integer nonlinear program |
| `CONIC` | Conic program (LP/SOCP/SDP, cvxopt standard form) |

### `VarType` enum

| Enumerator | Meaning |
|---|---|
| `Continuous` | Real-valued variable |
| `Integer` | General integer |
| `Binary` | Binary (0/1) integer |

### `Sense` enum

| Enumerator | Meaning |
|---|---|
| `Minimize` | Minimize objective (default) |
| `Maximize` | Maximize objective |

### `VariableMeta`

Attached to every variable in a model via the `vars` vector.

```cpp
struct VariableMeta {
  VarType     type{VarType::Continuous};
  double      lb{-1e20};
  double      ub{ 1e20};
  std::string name;          // optional, for debugging
};
```

### `SparseLinSys`

```cpp
struct SparseLinSys {
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd b;         // Ax = b
};
```

### `LPModel`

$$\min_{x} \; c^\top x \quad \text{s.t.} \quad Ax \le b, \; A_{eq} x = b_{eq}, \; \ell \le x \le u$$

```cpp
struct LPModel {
  Sense   sense{Sense::Minimize};
  Eigen::VectorXd             c;       // objective coefficients (n)
  Eigen::SparseMatrix<double> A;       // inequality LHS  (m_ineq × n)
  Eigen::VectorXd             row_lhs; // optional per-row lower bound (m_ineq)
  Eigen::VectorXd             b;       // inequality RHS  (m_ineq)
  Eigen::SparseMatrix<double> Aeq;     // equality LHS    (m_eq × n)
  Eigen::VectorXd             beq;     // equality RHS    (m_eq)
  std::vector<VariableMeta>   vars;    // bounds and types (n)
};
```

`row_lhs` enables **ranged rows**: when `row_lhs[i]` is finite,
row `i` becomes `row_lhs[i] <= A[i,:]*x <= b[i]`.
When `row_lhs` is empty (or shorter than `A.rows()`), the corresponding row
lower bound is $-\infty$ (standard $\le$ row).

Helper functions:

```cpp
bool   lp_has_row_lhs(const LPModel& lp);
double lp_row_lhs_or_neg_inf(const LPModel& lp, int row);
bool   lp_row_has_finite_lhs(const LPModel& lp, int row);
```

### `QPModel`

$$\min_{x} \; \tfrac{1}{2} x^\top Q x + c^\top x \quad \text{s.t.} \quad Ax \le b, \; A_{eq} x = b_{eq}$$

```cpp
struct QPModel {
  Sense   sense{Sense::Minimize};
  Eigen::SparseMatrix<double> Q;    // Hessian (n×n, symmetric PSD)
  Eigen::VectorXd             c;
  Eigen::SparseMatrix<double> A;
  Eigen::VectorXd             b;
  Eigen::SparseMatrix<double> Aeq;
  Eigen::VectorXd             beq;
  std::vector<VariableMeta>   vars;
};
```

### `NLPModel`

NLP with callback functions for the objective, gradient, and constraint Jacobians.

```cpp
struct NLPModel {
  Sense sense{Sense::Minimize};
  std::function<double(const VectorXd&)>              f;
  std::function<void(const VectorXd&, VectorXd&)>     grad;
  std::function<void(const VectorXd&, SparseMatrix&)> hess;
  // Lagrangian Hessian: f_eff + lambda'*g + nu'*h_nonlin
  std::function<void(const VectorXd&, const VectorXd&,
                     const VectorXd*, SparseMatrix&)> lagrangian_hess;
  // Nonlinear inequality g(x) <= 0 and equality h(x) = 0
  std::function<void(const VectorXd&, VectorXd&)>     g, h;
  std::function<void(const VectorXd&, SparseMatrix&)> jac_g, jac_h;
  // Optional symbolic form for exportable solvers (e.g. Ipopt NL file)
  std::shared_ptr<SymExpr>          symbolic_objective;
  std::vector<SymbolicConstraint>   symbolic_constraints;
  std::vector<VariableMeta>         vars;
  Eigen::VectorXd                   x0;   // initial point
};
```

### `MIPModel`

MILP. Integer and binary variables are specified by index into the flat variable
list in `linear_part.vars`.

```cpp
struct MIPModel {
  LPModel          linear_part;
  std::vector<int> integer_idx;       // indices of integer variables
  std::vector<int> binary_idx;        // indices of binary variables

  Eigen::VectorXd  initial_solution;  // optional warm-start incumbent
  std::optional<UCGenHint> uc_hint;   // optional UC domain hint
  std::vector<int> branching_priority;// per-variable priority (higher = first)
};
```

#### `MIPModel::UCGenHint`

Optional metadata consumed by the native B&C engine when
`BCOptions::enable_domain_heuristics` is `true`. It describes the
unit-commitment structure so the engine can apply UC-specific branching
and fixing heuristics.

| Field | Type | Description |
|---|---|---|
| `ng` | `int` | Number of generators |
| `T` | `int` | Commitment time periods |
| `ig_start` | `int` | Start index of the $u_{g,t}$ (commitment) block |
| `su_start` | `int` | Start index of the $v_{g,t}$ (startup) block |
| `sd_start` | `int` | Start index of the $w_{g,t}$ (shutdown) block |
| `pg_start` | `int` | Start index of the $p_{g,t}$ (power) block (`-1` if unknown) |
| `min_up` | `vector<int>` | Minimum up-time per generator (hours) |
| `min_down` | `vector<int>` | Minimum down-time per generator (hours) |
| `ig0` | `vector<int>` | Initial commitment state (0 or 1) |
| `pmin`, `pmax`, `ramp` | `vector<double>` | Power limits and ramp rate |
| `ig_cols`, `su_cols`, `sd_cols`, `pg_cols` | `vector<int>` | Column maps into the reduced model (`-1` = eliminated) |

### `ConicModel`

$$\min_{x} \; c^\top x \quad \text{s.t.} \quad Gx + s = h, \; Ax = b, \; s \in \mathcal{K}$$

```cpp
struct ConicModel {
  Sense   sense{Sense::Minimize};
  Eigen::VectorXd             c;       // objective coefficients (n)
  Eigen::SparseMatrix<double> G;       // conic constraint matrix (dims.total() × n)
  Eigen::VectorXd             h;       // conic RHS (dims.total())
  Eigen::SparseMatrix<double> A;       // equality constraint matrix (m_eq × n)
  Eigen::VectorXd             b;       // equality RHS (m_eq)
  ConeDims                    dims;    // cone dims {l, q, s}; K = R^l_+ × ∏ Q^{q_i} × ∏ S^{s_i}
  std::vector<VariableMeta>   vars;
};
```

Rows of `G`/`h` are ordered by cone block: `[l | q blocks | svec-packed s blocks]`.
SDP blocks use the svec packing (column-major lower triangle, off-diagonals
scaled by $\sqrt{2}$, so $\operatorname{tr}(SZ) = \mathrm{svec}(S)^\top \mathrm{svec}(Z)$).
See `docs/conic_sdp.md` for the solver design and derivations.

### `MINLPModel`

```cpp
struct MINLPModel {
  NLPModel         nonlinear_part;
  std::vector<int> integer_idx;
  std::vector<int> binary_idx;
};
```

---

## 3. SolverEngine API

```cpp
#include "mipsolvers/engine/engine.hpp"
// or selectively:
#include "mipsolvers/engine/api/solver.hpp"

namespace mipsolvers::engine {

class SolverEngine {
public:
  // Constructor — pass false to skip automatic adapter registration
  explicit SolverEngine(bool register_defaults = true);

  // Adapter management
  void register_adapter(const SolverAdapterPtr& adapter);
  std::size_t register_default_adapters();   // returns count registered

  // Preference overrides
  void set_solver_preference(ProblemClass cls, const std::string& adapter_name);
  std::vector<std::string> list_solvers(ProblemClass cls) const;

  // Generic dispatch (type-safe variant)
  api::Result solve(const api::ProblemVariant& problem,
                    const SolveOptions& options = {}) const;

  // Typed entry points
  api::Result solve_le   (const SparseLinSys&  p, const SolveOptions& o = {}) const;
  api::Result solve_nle  (const NonlinearSystem& p, const SolveOptions& o = {}) const;
  api::Result solve_lp   (const LPModel&       p, const SolveOptions& o = {}) const;
  api::Result solve_qp   (const QPModel&       p, const SolveOptions& o = {}) const;
  api::Result solve_nlp  (const NLPModel&      p, const SolveOptions& o = {}) const;
  api::Result solve_milp (const MIPModel&      p, const SolveOptions& o = {}) const;
  api::Result solve_minlp(const MINLPModel&    p, const SolveOptions& o = {}) const;
  api::Result solve_conic(const ConicModel&    p, const SolveOptions& o = {}) const;
};

} // namespace mipsolvers::engine
```

`register_default_adapters()` registers (in order of preference):

| Adapter name | Problem classes |
|---|---|
| `NativeLinear` | LE |
| `NativeNewton` | NLE |
| `NativeIPMLPAdapter` | LP |
| `NativePDLPAdapter` | LP |
| `NativeLCQPAdapter` | QP |
| `NativeIPMAdapter` | NLP |
| `NativeNLPAdapter` | NLP |
| `NativeConicIPM` | CONIC |
| `StrictHiGHS` | MILP |
| `NativeBranchAndCut` | MILP, MINLP |
| `Gurobi` *(if available)* | LP, QP, MILP |
| `HiGHS` | LP, MILP |
| `Ipopt` | NLP |
| `SCIP` | MINLP |

---

## 4. Solve Options

```cpp
enum class StrategyPolicy {
  Auto,          // engine heuristic based on problem characteristics
  NativeFirst,   // prefer native adapters before external ones
  ExternalFirst, // prefer external (Gurobi, HiGHS, …) first
};

struct SolveOptions {
  std::string   preferred_solver;        // exact adapter name, or "" for auto
  bool          allow_fallback{true};    // try next adapter if primary fails
  StrategyPolicy strategy_policy{StrategyPolicy::Auto};
  // Per-class overrides (takes precedence over strategy_policy)
  std::map<ProblemClass, StrategyPolicy> class_strategy_policy;
};
```

**Adapter name strings** (case-sensitive):

| Name | Adapter |
|---|---|
| `"Gurobi"` | GurobiAdapter |
| `"HiGHS"` | HighsAdapter |
| `"StrictHiGHS"` | Embedded HiGHS state machine with the MIPSolvers production contract; large roots use IPM with crossover by default |
| `"SCIP"` | ScipAdapter (MINLP only) |
| `"Ipopt"` | IpoptAdapter |
| `"NativeBranchAndCut"` | NativeBranchAndCutAdapter |
| `"NativeIPMLPAdapter"` | Interior-point LP |
| `"NativePDLPAdapter"` | First-order LP (PDLP) |
| `"NativeLCQPAdapter"` | LCQP (convex QP) |
| `"NativeLinear"` | Dense linear system solver |
| `"NativeNewton"` | Newton NLE solver |
| `"NativeIPMAdapter"` | Interior-point NLP |
| `"NativeNLPAdapter"` | SQP-style NLP solver |
| `"NativeConicIPM"` | Conic (LP/SOCP/SDP) interior-point solver |

Example — force HiGHS for LP but Gurobi for MILP:

```cpp
SolveOptions opts;
opts.class_strategy_policy[ProblemClass::LP]   = StrategyPolicy::ExternalFirst;
opts.class_strategy_policy[ProblemClass::MILP] = StrategyPolicy::ExternalFirst;
opts.preferred_solver = "Gurobi";
auto result = eng.solve_milp(mip, opts);
```

---

## 5. Result Structure

```cpp
namespace mipsolvers::engine::api {

struct Stats {
  bool        success{false};
  int         iterations{0};
  double      objective{0.0};
  double      residual_inf{0.0};    // max constraint violation
  double      primal_feas{0.0};     // primal feasibility
  double      dual_feas{0.0};       // dual feasibility
  double      complementarity{0.0};
  double      mip_gap{0.0};         // relative gap (MILP only)
  double      runtime_sec{0.0};
  std::string status;               // human-readable status string
  std::string solver_name;          // adapter that produced the result
  int         cglp_cuts_added{0};   // native B&C only
  // Farkas infeasibility certificate (populated when status = "Infeasible")
  Eigen::VectorXd farkas_ray;
  Eigen::VectorXd farkas_ray_eq;
  bool            has_farkas_certificate{false};
};

struct Result {
  Eigen::VectorXd x;                // primal solution (n)
  Stats           stats;
  // Shadow prices (dual variables). Layout: [ineq duals | eq duals]
  // Populated for LP solves by: Gurobi, NativeBranchAndCut (LP path),
  // NativeIPMLPAdapter.  Empty for MILP results.
  Eigen::VectorXd constraint_duals;
  // Variable bound multipliers (LP certificates only)
  Eigen::VectorXd box_dual_lb;      // z_l[j] for x[j] >= lb[j]
  Eigen::VectorXd box_dual_ub;      // z_u[j] for x[j] <= ub[j]
};

} // namespace mipsolvers::engine::api
```

### Dual variable layout

`constraint_duals` is ordered as:

```
[ dual_0, …, dual_{m_ineq-1},   (A*x <= b rows)
  dual_{m_ineq}, …, dual_{m_ineq+m_eq-1} ]  (Aeq*x = beq rows)
```

This ordering is consistent across all adapters that populate duals.

> **Note:** The HiGHS file-based adapter does not return constraint duals — it
> parses only the primal solution from HiGHS's solution file.  If duals are
> required (e.g., for LMP computation), prefer `"Gurobi"`,
> `"NativeBranchAndCut"`, or `"NativeIPMLPAdapter"`.

---

## 6. Solver Adapters

All adapters implement `SolverAdapter`:

```cpp
class SolverAdapter {
public:
  virtual std::string name() const = 0;
  virtual bool supports(ProblemClass cls) const = 0;

  virtual SolveResult solve_le   (const SparseLinSys& prob) const;
  virtual SolveResult solve_nle  (const NonlinearSystem& prob) const;
  virtual SolveResult solve_lp   (const LPModel& prob) const;
  virtual SolveResult solve_qp   (const QPModel& prob) const;
  virtual SolveResult solve_nlp  (const NLPModel& prob) const;
  virtual SolveResult solve_milp (const MIPModel& prob) const;
  virtual SolveResult solve_minlp(const MINLPModel& prob) const;
  virtual SolveResult solve_conic(const ConicModel& prob) const;
};
```

### External adapters (`solver/external/adapters.hpp`)

#### `HighsAdapter`

Wraps the HiGHS binary via file-based I/O.

```cpp
class HighsAdapter : public SolverAdapter {
public:
  explicit HighsAdapter(std::string executable = "");
  bool available() const;           // true if HiGHS binary found on PATH
  const std::string& executable() const;
};
```

Supports: `LP`, `MILP`.  
Returns duals: **No** (primal solution only from HiGHS solution file).

#### `IpoptAdapter`

Wraps the Ipopt binary via NL file exchange.

Supports: `NLP`.  
Returns duals: No.

#### `ScipAdapter`

Wraps the SCIP binary via file-based I/O.

Supports: `MINLP`.  
Returns duals: No.

#### `GurobiAdapter`

In-process Gurobi via the native C API (`libgurobi`). Requires a valid
Gurobi licence at runtime (`GRB_LICENSE_FILE` or the standard Gurobi
search path).

```cpp
class GurobiAdapter : public SolverAdapter {
public:
  GurobiAdapter();
  bool available() const;   // true if licence found and initialised
};
```

Supports: `LP`, `QP`, `MILP`.  
Returns duals: **Yes** (LP duals via `constraint_duals`).

### Native adapters (`solver/native/native_adapters.hpp`)

| Adapter class | `name()` | Supports | Returns duals |
|---|---|---|---|
| `NativeLinearAdapter` | `"NativeLinear"` | LE | N/A |
| `NativeNewtonAdapter` | `"NativeNewton"` | NLE | No |
| `NativeNLPAdapter` | `"NativeNLPAdapter"` | NLP | No |
| `NativeBranchAndCutAdapter` | `"NativeBranchAndCut"` | MILP, MINLP | **Yes** (LP path) |

LP-specific adapters:

| Adapter class | `name()` | Notes |
|---|---|---|
| `NativeIPMLPAdapter` | `"NativeIPMLPAdapter"` | Mehrotra predictor-corrector IPM; returns duals |
| `NativePDLPAdapter` | `"NativePDLPAdapter"` | First-order (PDLP) for large sparse LP |
| `NativeLCQPAdapter` | `"NativeLCQPAdapter"` | Linearly-constrained QP |
| `NativeIPMAdapter` | `"NativeIPMAdapter"` | Interior-point NLP solver |

Conic adapters:

| Adapter class | `name()` | Notes |
|---|---|---|
| `NativeConicIPMAdapter` | `"NativeConicIPM"` | Mehrotra predictor-corrector conic IPM (LP/SOCP/SDP); returns duals (`constraint_duals = [z | y]`), see `docs/conic_sdp.md` |

---

## 7. Native Branch-and-Cut Engine

The native B&C engine (`bc/api.hpp`) can also be used directly, bypassing
`SolverEngine`:

```cpp
#include "mipsolvers/engine/bc/api.hpp"

BCOptions opt;
opt.max_nodes     = 100'000;
opt.gap_tol       = 1e-3;
opt.num_threads   = 4;
opt.use_feasibility_pump = true;

BCResult result = solve_milp_bc(mip, opt);
```

With warm-start and ML callbacks:

```cpp
BCWarmStart ws;
ws.primal_hints.push_back({initial_x, initial_obj, false});

BCCallbacks cbs;
cbs.hyper_tuner = [](const BCInstanceFeatures& feat, const BCOptions& base,
                     const BCStats* prev) -> BCOptions {
  BCOptions tuned = base;
  if (feat.n_bin > 500) tuned.branching = BranchingStrategy::Pseudocost;
  return tuned;
};

BCResult result = solve_milp_bc(mip, opt, ws, cbs);
```

### 7.1 BCOptions Reference

#### Core limits

| Field | Default | Description |
|---|---|---|
| `max_nodes` | 50 000 | Maximum B&B nodes before stopping |
| `max_lp_iter` | 500 | LP/NLP iteration limit per node |
| `time_limit_sec` | 120.0 | Wall-clock time limit (seconds) |
| `int_tol` | 1e-5 | Integrality tolerance |
| `gap_tol` | 1e-4 | Relative primal-dual gap tolerance |
| `lp_tol` | 1e-6 | LP relaxation convergence tolerance |

#### LP solver at nodes

| Field | Default | Description |
|---|---|---|
| `use_simplex_lp_nodes` | `true` | Use dual simplex for tree node LPs |
| `use_ipm_root` | `false` | Use IPM for root LP relaxation |
| `use_ipm_nodes` | `false` | Use IPM for all node LPs |
| `lp_kernel_backend` | `HiGHS` | LP numerical kernel; `ExperimentalNative` is development-only |
| `strict_highs_mip_contract` | `false` | Explicit full-MIP StrictHiGHS policy; independent from LP kernel selection |
| `highs_mip_lp_solver` | `"choose"` | StrictHiGHS root LP solver; production auto-policy may set large roots to `"ipm"` |
| `highs_mip_root_crossover` | `"on"` | Crossover policy for StrictHiGHS root IPM; keep `"on"` when following node LPs need simplex bases |
| `highs_strict_auto_ipm_root_for_large_models` | `true` | If the root solver is still `"choose"`, use IPM for large StrictHiGHS roots |
| `highs_strict_auto_ipm_root_min_cols` | 10 000 | Column threshold for StrictHiGHS root-IPM auto-selection |
| `highs_strict_auto_ipm_root_min_rows` | 10 000 | Row threshold for StrictHiGHS root-IPM auto-selection |
| `highs_strict_auto_ipm_root_min_time_sec` | 30.0 | Minimum MIP time limit for automatic root IPM; short caps keep simplex unless IPM is explicit |
| `simplex_factor_backend` | 0 | Factor backend: 0=UmfpackNative, 1=HiGHSSafe, 2=ForceFT, 3=ShortChain |

#### Heuristics

| Field | Default | Description |
|---|---|---|
| `use_feasibility_pump` | `true` | Feasibility pump primal heuristic |
| `use_progressive_rounding` | `true` | Progressive rounding heuristic |
| `use_papilo_presolve` | `true` | PaPILO presolve pass |
| `papilo_aggressive` | `false` | More aggressive PaPILO reduction |
| `enable_feasibility_jump` | `true` | Generic feasibility-jump heuristic |
| `feasibility_jump_max_flips` | 512 | Binary flips per feasibility-jump attempt |
| `accept_verified_warm_start_incumbent` | `true` | Accept caller-supplied incumbent |
| `enable_domain_heuristics` | `false` | Enable UC/SCUC-specific domain heuristics |

#### Solution and cut pools

| Field | Default | Description |
|---|---|---|
| `cut_pool_max_size` | 2 000 | Maximum global cut pool size |
| `cut_pool_max_age` | 50 | Evict cuts unused for this many nodes |
| `solution_pool_size` | 10 | Number of elite feasible solutions to retain |

#### Parallelism

| Field | Default | Description |
|---|---|---|
| `num_threads` | -1 | Worker threads (-1 = hardware concurrency) |
| `max_plunge_depth` | 10 | Max depth for synchronous DFS plunge in parallel mode |
| `auto_parallel_min_threads` | 2 | Minimum threads for auto-parallel |
| `auto_parallel_max_threads` | 0 | 0 = no cap |
| `deterministic_parallel` | `false` | Round-robin turn token for reproducible parallel search |
| `share_conflict_learning_across_threads` | `false` | Publish conflict clauses across explorers |
| `share_implications_across_threads` | `true` | Publish binary implications to shared graph (requires above) |
| `pseudocost_delta_merge` | `true` | Delta-aggregate pseudocost merge across threads |
| `parallel_delay_until_incumbent` | `true` | Don't launch parallel tree until a finite incumbent exists |
| `enable_work_stealing` | `true` | Work-stealing between parallel explorer threads |

#### Implied Bound Cuts

| Field | Default | Description |
|---|---|---|
| `enable_graph_implied_bound_cuts` | `true` | Separate violated two-term implied-bound cuts from the binary implication graph |
| `graph_implied_bound_max_cuts` | 128 | Maximum implied-bound cuts per round |
| `enable_dynamic_implied_bound_probing` | `true` | Local probing to discover dynamic implied bounds at nodes |
| `dynamic_implied_bound_probing_max_vars` | 8 | Max fractional binaries to probe per node |
| `dynamic_implied_bound_probing_max_rows` | 8 | Max rows generated per probing pass |

#### Reduced-Cost Learning

| Field | Default | Description |
|---|---|---|
| `enable_reduced_cost_fixing` | `true` | Apply reduced-cost domain fixing after LP solves |
| `enable_reduced_cost_conflict_learning` | `true` | Record cutoff conflicts from reduced-cost fixings |
| `enable_reduced_cost_fixing_resolve` | `true` | Re-solve LP after tightening bounds via fixing |
| `enable_reduced_cost_proof_cut_resolve` | `true` | Re-solve nodes after proof-frontier no-good rows are admitted |
| `reduced_cost_proof_cut_resolve_max_lps` | 4 | LP budget per node for proof-cut resolve |

#### Root Cut Quality

| Field | Default | Description |
|---|---|---|
| `root_cut_stall_tol` | 1e-4 | Stall tolerance: stop cutting if bound lift is below this |
| `root_cut_max_stalls` | 2 | Maximum consecutive stalled rounds before halting |
| `root_cut_reject_nonmoving_rows` | `true` | Reject root cuts that don't tighten the domain or bound |
| `root_cut_audit_validate_rows` | `true` | Validate cut rows after each separation round |
| `root_cut_audit_force_separation` | `false` | Force separation through size/fractionality gates (audit only) |
| `enable_objective_cutoff_conflict_cuts` | `true` | Objective-cutoff conflict cuts from cost-implied binary events |
| `enable_graph_implied_bound_cuts` | `true` | Two-term implied-bound cuts from binary implication graph |

#### LP Fallback and Auto-Tuning

| Field | Default | Description |
|---|---|---|
| `enable_lp_fallback` | `true` | Enable multi-phase LP failure recovery |
| `fallback_l1_retries` | 3 | Phase-1 retry count (perturbation-based) |
| `fallback_l3_retries` | 2 | Phase-3 retry count (tolerance-relaxation) |
| `ipm_auto_threshold` | 8 000 | Switch to IPM at root when `m > threshold` |
| `hybrid_ipm_threshold` | 15 000 | Hybrid IPM/simplex crossover for node LPs |
| `xlarge_ipm_only_threshold` | 50 000 | Use IPM-only above this size |

#### Large Neighborhood Search (LNS)

| Field | Default | Description |
|---|---|---|
| `enable_lns` | `true` | Enable LNS sub-MIP improvement heuristic |
| `lns_fix_ratio` | 0.80 | Fraction of binaries fixed during LNS dive |
| `lns_node_limit` | 500 | Node limit per LNS sub-MIP |
| `lns_time_limit` | 5.0 | Time limit per LNS call (seconds) |
| `lns_max_iters` | 3 | Maximum LNS improvement iterations |

### 7.2 Branching and Node Selection

```cpp
enum class BranchingStrategy {
  MostInfeasible,  // variable with fraction closest to 0.5
  Pseudocost,      // Benichou pseudocost estimates (default)
  FirstFractional, // first fractional variable (baseline/debug)
};

enum class NodeSelection {
  BestFirst,   // estimate-guided best-first
  DepthFirst,  // LIFO (fast incumbent, small memory)
  Hybrid,      // DFS until first incumbent, then best-first (default)
};
```

Set via `BCOptions::branching` and `BCOptions::node_sel`.

Variable priorities can be set in `MIPModel::branching_priority` (higher value
= branch first). This overrides the branching strategy for the selected
variables but does not disable pseudocost learning.

### 7.3 Cut Generation

Enabled cut families are selected through `BCOptions::cuts`:

```cpp
enum class CutType {
  None,
  IntRounding, MIR, Gomory, Cover,
  FlowCover, ImpliedBound,
  All,          // all of the above (default)
};
```

Cut-quality filters:

| Field | Default | Description |
|---|---|---|
| `root_cut_rounds` | 10 | Rounds of cutting at root node |
| `cuts_per_round` | 20 | Max cuts added per round |
| `max_cut_depth` | 0 | Generate cuts at tree nodes up to this depth (0 = root only) |
| `gmi_min_efficacy` | 1e-3 | Minimum normalised violation for GMI admission |
| `gmi_max_density` | 0.35 | Maximum non-zero ratio for admitted GMI cuts |
| `gmi_max_parallelism` | 0.90 | Reject near-parallel cuts above this cosine |

### 7.4 Warm-Start

```cpp
struct BCWarmStart {
  std::vector<BCPrimalHint> primal_hints;  // incumbent(s) to seed B&B
  Eigen::VectorXd           dual_row;      // row duals for LP warm-start
  Eigen::VectorXd           dual_col;      // reduced-cost / bound duals
  BCSimplexBasis            basis;         // simplex basis descriptor
  std::vector<BCCutRecord>  cut_pool;      // cut pool snapshot to reseed
  double                    best_bound;    // inherited lower bound
};

struct BCPrimalHint {
  Eigen::VectorXd x;
  double          obj{0.0};
  bool            verified{false};  // set true if caller checked feasibility
};
```

A primal hint with `verified = false` undergoes an internal LP feasibility
check before being accepted as an incumbent.

### 7.5 Cut Validity Scope

When using the dynamic-node-cut or other callback-based cut injection paths,
each cut can be annotated with a `ValidityScope` that controls how the engine
manages it:

```cpp
enum class ValidityScope {
  GlobalCut,       // Added to the global cut pool; shared across all nodes
  LocalNodeCut,    // Valid only for the current node and its subtree
  LazyConstraint,  // Lazy constraint; activates once a violating integer point is found
};
```

This enum is declared in `mipsolvers/engine/bc/enums.hpp`.

### 7.6 Dynamic Node Cuts

`BCCallbacks` includes a `dynamic_node_cut` hook for injecting user-generated
cuts at integer feasible nodes (lazy-constraint / user-cut style):

```cpp
struct BCDynamicNodeCutContext {
  int                    node_id;
  int                    depth;
  const Eigen::VectorXd& x;         // current LP/node solution
  const BCStats&         stats;
};

struct BCDynamicNodeCut {
  Eigen::SparseVector<double> coeff;  // row of coeff*x <= rhs
  double                      rhs;
  bool                        integral{false};   // cut is integrality-valid
  bool                        propagate{true};   // allow domain propagation
  std::string                 audit_family;      // diagnostic tag
  std::string                 audit_key;
  ValidityScope               validity_scope{ValidityScope::GlobalCut};
};

using BCDynamicNodeCutFn = std::function<int(
    const BCDynamicNodeCutContext& ctx,
    std::vector<BCDynamicNodeCut>& out_cuts)>;
```

Register it via `BCCallbacks::dynamic_node_cut`:

```cpp
BCCallbacks cbs;
cbs.dynamic_node_cut = [](const BCDynamicNodeCutContext& ctx,
                           std::vector<BCDynamicNodeCut>& cuts) -> int {
  // inspect ctx.x and add violated inequalities to cuts
  if (/* constraint violated */) {
    BCDynamicNodeCut cut;
    cut.coeff = /* sparse row */;
    cut.rhs   = /* rhs */;
    cut.validity_scope = ValidityScope::LazyConstraint;
    cuts.push_back(std::move(cut));
    return 1;
  }
  return 0;  // no cuts generated
};
BCResult result = solve_milp_bc(mip, opt, ws, cbs);
```

Return value is the number of cuts generated; return 0 to signal no violation.

### 7.7 ML Hooks

`BCCallbacks` provides extension points for learned policies:

| Hook | Signature | Description |
|---|---|---|
| `hyper_tuner` | `BCOptions(feat, base, prev_stats)` | Override BCOptions before the run |
| `node_selector` | `void(open_nodes, out_permutation)` | Reorder the open-node list |
| `cut_selector` | `void(candidates, budget, out_mask)` | Accept/reject cut candidates |
| `branching_advisor` | `BCBranchingPrior(node_id, fracs, features)` | Suggest branching variable |
| `incumbent_callback` | `void(x, obj, stats)` | Called each time a better incumbent is found |
| `termination_check` | `bool(stats)` | Custom early-termination condition |
| `dynamic_node_cut` | `int(ctx, out_cuts)` | Inject user cuts at integer feasible nodes |

---

## 8. Strategy Dispatcher

`strategy::StrategyDispatcher` is the internal routing layer used by
`SolverEngine`. It is rarely needed directly, but understanding it helps when
debugging solver selection.

Selection order when `preferred_solver` is `""` (Auto):

1. If the user set a per-class preference via `set_solver_preference`, that
   adapter is tried first.
2. Otherwise, `StrategyPolicy` governs the ordering:
   - `Auto` — LP/QP/MILP prefer an installed/licensed `Gurobi`; MILP then
     falls back to `StrictHiGHS`, direct `HiGHS`, and `NativeBranchAndCut`.
   - `NativeFirst` — all native adapters come before external ones.
   - `ExternalFirst` — external adapters (Gurobi, HiGHS) come before native.
3. If the chosen adapter fails and `allow_fallback = true`, the dispatcher
   tries the next candidate in order.

`list_solvers(ProblemClass)` returns the full ordered list for a given class,
which is useful for introspection:

```cpp
for (const auto& name : eng.list_solvers(ProblemClass::MILP))
    std::cout << name << "\n";
```

---

## 9. Include Layout

```
include/mipsolvers/engine/
  engine.hpp                    ← umbrella (includes all api/ headers)
  problem_types.hpp             ← all model structs
  branch_and_cut.hpp            ← BCOptions, BCResult (convenience)
  api/
    solver.hpp                  ← SolverEngine class
    problem.hpp                 ← ProblemVariant + problem_class()
    result.hpp                  ← api::Result, api::Stats
    options.hpp                 ← SolveOptions, StrategyPolicy
  solver/
    solver_adapter.hpp          ← SolverAdapter base class, SolveResult
    adapter_registry.hpp        ← AdapterRegistry
    external/
      adapters.hpp              ← HighsAdapter, IpoptAdapter, ScipAdapter, GurobiAdapter
    native/
      native_adapters.hpp       ← NativeLinear, NativeNewton, NativeBranchAndCut, …
  bc/
    api.hpp                     ← solve_milp_bc / solve_minlp_bc
    options.hpp                 ← BCOptions
    stats.hpp                   ← BCStats, BCRootStats
    warmstart.hpp               ← BCWarmStart, BCPrimalHint, …
    ml_hooks.hpp                ← BCCallbacks, BCInstanceFeatures, BCDynamicNodeCut, …
    enums.hpp                   ← BranchingStrategy, NodeSelection, CutType, ValidityScope
  strategy/
    dispatcher.hpp              ← StrategyDispatcher
```

---

## 10. Performance Notes (May 2026)

Eight targeted hot-path improvements were applied to the native B&C engine. All
benchmarks (unit tests, market simulations, and IEEE-39 cases) pass with identical
objective values and cut counts at equal or lower wall-clock times.

### 10.1 Changes

| # | File | Change | Asymptotic improvement |
|---|------|--------|------------------------|
| 1 | `dual_simplex.hpp` / `dual_simplex_api.cpp` | Added `aux_col_to_row` reverse-lookup vector to `StandardFormLP`; populated in `build_standard_form_lp` and `append_leq_rows_to_standard_form` | `standard_form_aux_col_row`: O(m) linear scan → O(1) |
| 2 | `bc_cuts.cpp` | `build_bounded_form_gmi_cut` / `build_bounded_form_mir_cut` accept optional `is_basic_hint`; `add_transformed_tableau_cuts` builds the vector once before the fractional-row loop | O(n_std) alloc+fill per cut → once per cut round |
| 3 | `bc_cuts.cpp` | Hoisted `efficacy_A_row` (RowMajor copy of `lp.A`) outside the `compute_efficacy` lambda; the lambda lazily refreshes only when new rows were added | O(nnz) copy per cut family (≤8/round) → at most one rebuild per round when rows grow |
| 4 | `bc_cuts.cpp` | Replaced `pool.erase(std::remove(...))` in `add_clique_cuts` greedy extension with swap-to-end + `pop_back` | O(P²) worst case → O(P) per seed |
| 5 | `bc_relaxation.cpp` | `highs_pass_lp_model` avoids redundant `Eigen::ColMajor` copy of `lp.A`/`lp.Aeq` when the matrices are already compressed | O(nnz) copy per LP solve eliminated in the common case |
| 6 | `branch_and_cut.cpp` | `append_projected_cutpool_upper_row`: `std::map<int,double>` → `std::unordered_map<int,double>` (with `reserve`) for cut-term merging | O(k log k) → O(k) per cut appended |
| 7 | `bc_utils.cpp` | `reduced_cost_fixing`: replaced `std::vector<char>(n)` with `std::unordered_set<int>` sized to the basis (m ≪ n) | O(n) allocation → O(m) allocation and lookup |
| 8 | `bc_utils.cpp` | `propagate_upper_side` (inside `propagate_node_domain_impl`): hoisted `std::vector<int> ids` outside the inner non-zero loop; `ids.clear()` reuses capacity | O(nnz_row) heap allocs per row visit → zero dynamic allocs after first row |

### 10.2 Post-fix benchmark (macOS M4, Release build)

| Case | Native B&C (s) | HiGHS (s) | Gurobi (s) | Obj ($) | Cuts |
|------|---------------|-----------|------------|---------|------|
| 3-bus T=6 | 0.029 | 0.017 | 0.003 | 746 317.1 | 32 |
| 6-bus T=8 wind+sto | 0.047 | 0.020 | 0.005 | 76 477.6 | 86 |
| IEEE-39 T=4 | 0.017 | 0.020 | 0.005 | 203 129.7 | 122 |
| IEEE-39 T=24 wind+solar | 0.087 | 0.080 | 0.071 | 891 465.7 | 898 |
| IEEE-39 T=24 full | 0.131 | 0.087 | 0.077 | 891 465.7 | 900 |

MIP gap = 0.000 % for all cases; 261 assertions across all test binaries pass.
