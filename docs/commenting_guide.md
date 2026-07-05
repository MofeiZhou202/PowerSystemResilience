> Documentation Sync (2026-07-05)
> Scope: reviewed against current repository structure, CMake presets/options, and registered test targets.
> Status: implementation-backed reference.
> Source of truth: when text and implementation diverge, treat src/, include/, tests/, and CMake files as authoritative.

# Code Review Commenting Guide {#commenting_guide}

This project uses Doxygen comments as the source for the generated HTML
notebook.  Comments should explain the engineering contract and the equations a
reviewer needs, not restate each line of code.

## Header Comments

Use public headers for durable API contracts:

```cpp
/// Brief one-line purpose.
///
/// Mathematical model:
/// \f[
///   g(x) = 0,\qquad h(x) \le 0
/// \f]
///
/// @param sys Rich input network. The function canonicalizes it internally.
/// @param opt Numerical tolerances and model toggles.
/// @return Result vectors mapped back to user-facing component order.
```

Prefer `\f[ ... \f]` or `\f$ ... \f$` for formulas so Doxygen can render them
without treating LaTeX commands as unknown documentation commands.

## Implementation Comments

Use `.cpp` comments for local algorithm landmarks:

- variable layout and index maps;
- constraint rows and sign conventions;
- numerical safeguards such as clipping, regularization, or fallback logic;
- non-obvious formulas whose implementation is spread across several loops.

Do not add boilerplate comments to obvious getters, constructors, simple loops,
or one-line helpers unless they encode a convention that a reviewer could miss.

## Formula Style

Use the following notation consistently:

- \f$\theta_i\f$: AC voltage angle at bus `i`.
- \f$V_i\f$: AC voltage magnitude at bus `i`.
- \f$V^{dc}_k\f$: DC voltage at DC bus `k`.
- \f$P^g_i, Q^g_i\f$: generator injections.
- \f$P^{ac}_c, Q^{ac}_c, P^{dc}_c\f$: converter terminal powers.
- \f$G_{ij}, B_{ij}\f$: entries of the AC admittance matrix.
- \f$G^{dc}_{km}\f$: entries of the DC conductance matrix.

When a function returns residuals, document the sign convention.  For example,
use “generation minus demand minus network injection” or “left side minus right
side” explicitly.

## Batch Plan

For complete code review coverage, annotate in this order:

1. `include/hacdcpf/model/`: data structures and units.
2. `include/hacdcpf/projection/` and `src/model/`: rich-to-canonical mapping.
3. `include/hacdcpf/power_flow/` and `src/power_flow/`: PF equations and
   Jacobians.
4. `include/hacdcpf/optimal_power_flow/` and `src/optimal_power_flow/`: OPF
   objective, constraints, KKT system, and fallback paths.
5. `include/hacdcpf/graph/` and `src/graph/`: topology abstractions and graph
   algorithms.
6. `include/hacdcpf/io/` and `src/io/`: schema and unit conversion contracts.
