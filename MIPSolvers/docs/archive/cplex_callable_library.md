# CPLEX Callable Library integration and validation

## Model and claim

An `MIPModel` represents a linear mixed-integer program with objective `c'x`,
boxed variables, two-sided rows `lhs <= A x <= rhs`, equalities
`Aeq x = beq`, and explicit binary/general-integer index sets. The adapter maps
this model without algebraic reformulation to IBM ILOG CPLEX 22.1.1 and invokes
`CPXmipopt`.

`CPXcopylp` receives one column-compressed matrix. Each finite upper side is
emitted as `A_i x <= rhs_i`, each finite lower side as `A_i x >= lhs_i`, and
each equality as `Aeq_i x = beq_i`. The original objective sense and variable
order are passed unchanged, so primal objectives and dual bounds need no sign
conversion. Integer and binary columns are assigned with `CPXcopyctype`.

## Cost model and fixed validation

The conversion allocates `O(n+m+nnz)` storage and scans each matrix nonzero
once, emitting a second copy only when an inequality has two finite sides.
Predicted adapter import overhead is below 5% of import plus optimize time for
instances whose optimization phase lasts at least one second.

Fixed validation gates are:

1. Known-optimum models cover ranged rows, equalities, minimization,
   maximization, binary/general-integer types and a MIP start at `1e-4`
   objective tolerance.
2. A CPLEX-enabled Release build runs focused CPLEX tests, full engine API and
   MILP tests, plus a CPLEX-disabled build regression.
3. MIPLIB incumbents must pass the shared `1e-5` original-model row, bound,
   integrality and objective audit.
4. A wrong-sign result or import-overhead deviation greater than about 50% of
   the prediction triggers investigation in this order: implementation
   fidelity, machine/cost model, assumptions, theory.

References are the IBM ILOG CPLEX 22.1.1 Callable Library documentation for
`CPXcopylp`, `CPXcopyctype`, `CPXaddmipstarts`, `CPXmipopt`, solution status,
objective, best bound, relative gap and node count; Achterberg, *Constraint
Integer Programming* (2007), Sections 4.1-4.2; and Gleixner et al., "MIPLIB
2017", Mathematical Programming Computation 13 (2021), Section 3 and Appendix
A. The repository audit contract is in
`docs/miplib2017_benchmark_protocol_2026-08-25.md`.

## Result mapping

CPLEX optimal, optimal-with-tolerance, infeasible, unbounded and
infeasible-or-unbounded MIP statuses are proof-bearing. Only the two optimal
statuses are marked optimal, and the feasible/infeasible time-limit statuses
are marked timed out. Incumbent availability is determined independently with
`CPXgetobjval`; a finite objective is never used by itself as a proof claim.

The supported scope is linear MILP. The public model does not represent
semi-continuous or semi-integer domains. LP and QP CPLEX entry points are
outside the current adapter scope. One adapter instance owns one CPLEX
environment and must not be called concurrently.

## Windows source evidence

The source implementation came from `release/windows-self-contained` commit
`78939619`, based on parent `7fb0f10f`. Its validated environment was MSVC
19.44 x64 Release, `/MD`, IPO disabled, CPLEX Studio 22.1.1, one CPLEX thread,
seed zero, Gurobi disabled and source-built third-party dependencies.

The focused CPLEX slice passed 2 cases and 7 assertions. A corrected local
12-instance MIPLIB run produced 1 proven and 8 feasible CPLEX results with no
incumbent audit failures. Across runs lasting at least one second, maximum
CPLEX import fraction was 4.6122%, within the 5% gate. These were single-seed,
three-second runs without a strict process-level Windows deadline and do not
establish general performance parity with Gurobi or superiority over HiGHS.

Windows dynamically loads `cplex2211.dll` even though the build links the
`stat_mda/cplex2211.lib` import library. Omitting
`cplex/bin/x64_win64` from `PATH` caused process startup failure
`0xc0000135`; it is a deployment requirement, not a solver status.

## macOS main-port evidence

The main port started from `aef3be077ef2adf06286184fc8c4a5cf04138a74` on
2026-09-13. It imported only CPLEX capability, not the Windows commit's native
MILP scheduling or HiGHS benchmark timing changes. CMake added Apple Silicon
and Intel macOS layouts plus the frameworks required when the static library
is selected.

The validated build used AppleClang 21.0.0.21000334, arm64, Release, IPO off,
Gurobi off, and `/Applications/CPLEX_Studio2211`. CMake selected
`cplex/bin/arm64_osx/libcplex2211.dylib`; the installed interactive optimizer
reported version 22.1.1.0. `otool -L` confirmed the test executable's
`@rpath/libcplex2211.dylib` dependency.

Results:

| Validation | Result |
|---|---|
| Focused CPLEX API | 3 cases, 15 assertions, all passed |
| Full engine API | 30 cases, 175 assertions, all passed |
| Full MILP solver | 55 cases, 2,049 assertions, all passed |
| CPLEX-disabled registration | 1 case, 1 assertion, passed after rebuild |
| macOS static Callable Library | 3 focused cases passed; frameworks resolved |
| `mas74`, 3 s | incumbent audited; gap 7.41084%; import 0.100334 ms |
| `sct2`, 3 s | incumbent audited; gap 0.0224079%; import 0.198709 ms |

The import fractions were 0.00334% and 0.00662% of import plus optimize time,
well below the pre-registered 5% gate. Maximum audited row violation was
`4.17444e-14`. No mismatch protocol was triggered. These two short runs verify
the adapter and measurement path; they are not a solver ranking.
