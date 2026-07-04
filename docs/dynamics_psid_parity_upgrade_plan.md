# Transient Dynamics Submodule — PSID Parity & Efficiency Upgrade

**Goal.** Bring the HACDCPF transient submodule (`src/dynamics`, `include/hacdcpf/dynamics`)
to functional parity with PowerSimulationsDynamics.jl (PSID) for the component test matrix in
`tools/psd_validation/psd_component_test_matrix.md` (39 groups, 27 currently blocked), and fix
the computational-efficiency problems that make the current implicit solver impractical at scale.

This document is **theoretical analysis first, then an executable plan**. It is a design
document, not a set of edits.

Reference source studied: `/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl` (PSID).

---

## 0. Executive summary

The test-matrix failures and the efficiency complaints have **one shared root cause**: our engine
is a *partitioned phasor* simulator (integrate device ODEs, then re-solve an algebraic network by
nested fixed-point iteration), whereas PSID is a *simultaneous DAE* simulator (one state vector
`[network voltages ; device states]`, one mass matrix, one sparse implicit solve per step).

The partitioned architecture is:
- **the performance limiter** — every right-hand-side evaluation runs a full nested network solve
  (up to `algebraic_network_max_iters = 20` sparse re-factorizations); the implicit integrators
  wrap that in a **dense finite-difference Jacobian** costing `(n+1)` RHS evaluations per Newton
  iteration → `O(n)` network solves per Newton iteration (`DynamicSolver.cpp` `numerical_state_jacobian`);
- **the correctness/parity limiter** — PSID's tests are defined against a `ResidualModel`
  (implicit DAE, IDA) and a `MassMatrixModel` (mass-matrix ODE, Rodas) plus an
  eigenvalue/participation `small_signal_analysis`. We expose none of these, so ~12 rows are
  `compare-limited` purely for lack of a matching *formulation*, and the multi-machine/eigenvalue
  rows (Test 35, 36) are `blocked-missing-formulation`;
- **the model-library limiter** — PSID composes each device from independent blocks
  (`Machine + Shaft + AVR + TurbineGov + PSS`, and `Converter + DCSource + Filter + FreqEstimator +
  InnerControl + OuterControl`), so its controller/model catalog is broad. Ours embeds controllers
  as machine-coupled add-ons and inverters as monolithic devices, which is why 9 rows are
  `blocked-missing-controller` and 12 are `blocked-missing-model`.

**Recommended path** (detail in Part III): do the **efficiency quick-wins first** (Phase 0 — they
are low-risk and independent), then execute the **mass-matrix DAE re-architecture** (Phase 1 — the
keystone that simultaneously fixes performance, enables `small_signal` (Phase 2), dynamic branches,
and formulation parity), then the **composed-device refactor** (Phase 3) and **model-library
expansion** (Phase 4). Multi-mass shaft and dynamic branches (Phase 5) follow naturally once the
DAE core exists.

---

# Part I — Theoretical analysis

## I.A Two paradigms: partitioned vs. simultaneous DAE

A power-system RMS/phasor model is a **differential-algebraic system**:

```
  ẋ = f(x, V, u, t)          device/controller differential states
  0 = g(x, V, u, t)          network Kirchhoff current balance  (I_inj(x,V) − Y_bus·V = 0)
```

There are two ways to integrate it.

**Partitioned (interleaved) — what HACDCPF does today.** At each step, alternate:
1. hold `x`, solve `g(x,V)=0` for `V` (a nonlinear algebraic solve — here a fixed-point/Gauss
   iteration with a sparse `Y_eff·V = I_eff` linear solve inside), then
2. hold `V`, integrate `ẋ = f(x,V)` one step.

This is simple and was the right Stage-1 choice (`docs/transient_simulation.md`). Its fatal cost is
that the network solve in step 1 is **re-run inside every derivative evaluation** (see
`DynamicSystem::evaluateDerivatives` → `solveNetwork`, `src/dynamics/DynamicSystem.cpp`), and the
implicit integrators need many derivative evaluations per step.

**Simultaneous DAE — what PSID does.** Stack everything into one unknown vector and one system:

```
  u = [ V_r ; V_i ; (branch states) ; (device states) ]          (PSID ordering)
  M · u̇ = F(u, t)                                                 (MassMatrixModel), or
  0 = R(u̇, u, t)                                                  (ResidualModel)
```

with a **diagonal mass matrix** `M` whose rows are `0` for algebraic unknowns (network voltages,
algebraic device states) and `1` for differential unknowns. The network equations become just
algebraic rows of the same residual — **there is no nested solve**. A single stiff DAE/ODE
integrator (IDA or Rodas) advances the whole vector with one sparse Jacobian factorization per
Newton iteration.

> PSID: `src/base/simulation_inputs.jl` `_make_mass_matrix` sets `M[1:2·n_bus, ·] = 0` (voltages are
> algebraic); `src/models/network_model.jl` returns `I_inj − Y_bus·V` as algebraic residual rows;
> `src/models/system.jl` `system_residual!` / `system_mass_matrix!` assemble the full vector.

**Why this matters beyond speed.** The simultaneous form is what makes PSID's *tests* well-defined:
- the **ResidualModel** `R(u̇,u,t)=0` is exactly the object IDA integrates and is the reference for
  `blocked-missing-formulation` rows;
- the Jacobian `∂R/∂u` of that same object is what `small_signal_analysis` reduces to an eigenvalue
  problem (Test 36 vs ANDES);
- **dynamic branches** are just extra differential rows (`L·dI/dt = V_from − V_to − R·I`) inserted
  into the same vector (`src/models/dynline_model.jl`) — impossible to express cleanly in a
  partitioned network solve, which is why Tests 10/11/25/27 are blocked.

## I.B Why the current formulation blocks parity

| Symptom in the matrix | Underlying formulation gap |
|---|---|
| 12 rows `compare-limited` "no PSD-equivalent ResidualModel/MassMatrixModel/small-signal gate" | We integrate `ẋ=f` with a *separate* network solve; there is no single residual object to compare, and no eigenvalues. |
| Test 35, 36 `blocked-missing-formulation` (multi-machine, eigenvalues vs ANDES) | No reduced-Jacobian/eigenvalue API. |
| Tests 10/11/25/27 (dynamic branches) `blocked-missing-model` | Network is algebraic-only; no differential branch states. |
| Test 03 `compare-failing` (SimpleMarconato δ drift ≈ 0.09 rad after bus 1–3 trip) | Partitioned angle/voltage coupling + a separate reference-frame convention diverge from PSID's simultaneous solve during a topology change. Likely an **initialization/frequency-reference** mismatch (see I.D). |

## I.C PSID formulation details (the target)

1. **Unknown vector & mass matrix.** `u = [V_r(1:n) ; V_i(1:n) ; branch ; device]`. `M` diagonal;
   `M=0` for the `2n` network rows and for algebraic device states, `M=1` for differential states
   (`_make_mass_matrix`, `_make_DAE_vector`).
2. **Network as algebraic rows.** `g = I_inj(x,V) − (Y_bus+Y_shunt)·V`, evaluated **matrix-free**
   over the sparse `Y_bus` in rectangular `[G −B; B G]` form (`network_model.jl`). No factorization
   in the RHS; the *solver's* Newton handles it.
3. **Two model wrappers.**
   - `ResidualModel`: `R = f(x,V) − M·u̇` (device rows) and `R = g` (network rows) → `IDA` (Sundials,
     sparse KLU). Best for large/stiff/DAE-with-index systems.
   - `MassMatrixModel`: `M·u̇ = [f ; g]` → `Rodas4/5` (Rosenbrock, mass-matrix aware). Best default
     for small/medium.
   (`simulation.jl` `_get_diffeq_problem`; `system.jl`.)
4. **Sparse Jacobian by AD, computed once, reused.** `JacobianFunctionWrapper`
   (`src/base/jacobian.jl`) uses `ForwardDiff.jacobian!` with chunking; sparsity discovered by a few
   random-perturbation passes (`sparse_retrieve_loop`), and the sparse `Jv` is handed to the solver
   as `jac_prototype`. For IDA the shifted Jacobian `J − γ·M` is formed in place.
5. **Small-signal reduction** (`src/base/small_signal.jl` `_reduce_jacobian`): partition the full
   Jacobian into differential/algebraic blocks and eliminate the algebraic variables by a Schur
   complement,
   ```
   A_reduced = M_dd^{-1} · ( f_x − f_y · g_y^{-1} · g_x )
   ```
   then `eigen(A_reduced)` → eigenvalues, damping `ζ = −Re(λ)/|λ|`, and participation factors
   `p_{ki} = |L_{ik}|·|R_{ki}| / Σ_i |L_{ik}|·|R_{ki}|`.

## I.D Composed-device architecture (ports / inner_vars)

PSID does **not** hard-wire a machine to its controllers. Each dynamic generator is a tuple of five
independently-authored blocks, coupled by a small fixed set of *inner variables*:

```
DynamicGenerator = Machine + Shaft + AVR + TurbineGov + PSS
  inner_vars (GEN_INNER_VARS_SIZE = 9): τe, τm, Vf, V_pss, VR_gen, VI_gen, ψd, ψq, Xad_Ifd
  flow:  Shaft.ω → {AVR, PSS, TG};  TG → τm → Shaft;  PSS → V_pss → AVR;
         AVR → Vf → Machine;  Machine → τe → Shaft;  Machine → (VR,VI) → AVR
```
```
DynamicInverter = Converter + DCSource + Filter + FreqEstimator(PLL) + InnerControl + OuterControl
  inner_vars (INV_INNER_VARS_SIZE = 25): md, mq, Vdc, Vr/Vi_filter, ω/θ_pll, Id/Iq_oc,
                                         Id/Iq_ic, Ir/Ii_cnv, Ir/Ii_filter, Vr/Vi_cnv, ...
  flow:  V_grid → Filter → PLL → OuterControl → InnerControl → Converter → Filter → I_inj
```
(`src/base/device_wrapper.jl`, `src/base/ports.jl`, `src/base/definitions.jl`, `src/models/device.jl`.)

Because blocks are independent, PSID's library is *combinatorial*: any of ~12 machines × ~10 AVRs ×
~10 governors × ~7 PSS. **Our current design is close in spirit but coarser**: my Layer-1 work added
a `MachineControlLink` so a `Governor`/`Exciter`/`PowerSystemStabilizer` writes the derivative of a
machine-owned actuator slot (`include/hacdcpf/dynamics/devices/MachineControlLink.hpp`,
`BasicDynamicDevices.cpp`). That is a *2-port* subset of PSID's port system (speed in, τm/Vf/Vs
out). It works for the wired controllers but does not yet expose the full inner-var bus (no ψd/ψq,
Xad_Ifd, VR/VI feedback), and inverters remain monolithic (`GridFollowingInverter`,
`GridFormingInverter`) rather than `Converter+Filter+PLL+Inner+Outer`. This is why the IBR rows
(VSM, VOC, REGCA/REECB/REPCA, DERA) are `blocked-missing-model` even though a GFL/GFM exists.

## I.E Computational-efficiency deep-dive (current engine)

Cost is dominated by **how often the network is solved**. Tracing the code
(`src/dynamics/DynamicSystem.cpp`, `src/dynamics/DynamicSolver.cpp`):

- `evaluateDerivatives(t,x,·)` calls `solveNetwork(t)` **once per call**.
- `solveNetwork` runs a fixed-point loop up to `algebraic_network_max_iters` (**default 20**),
  and **each iteration** constructs a fresh `SparseLinearSolver` and calls `solver.solve(Y_eff, …)`
  which **re-factorizes** `Y_eff` from triplets (no cached symbolic/numeric factorization; the base
  `Y_bus` is constant between events yet reassembled every time).
- The implicit integrators call `numerical_state_jacobian`, a **dense** `n×n` finite-difference
  Jacobian: `n` perturbed `evaluateDerivatives` calls (→ `n` network solves) + 1, per Newton
  iteration; the correction is a **dense** `colPivHouseholderQr` solve (`O(n³)`).

**Cost model per step (order of magnitude):**

```
 explicit Heun:   ~3  network solves         × (≤20 sparse factorizations each)
 explicit RK4:    ~5  network solves         × (≤20 …)
 Backward-Euler / Trapezoidal Newton:
     ≈ maxNewton × (n_states + 1 + lineSearch) network solves × (≤20 factorizations each)
     + maxNewton dense O(n³) solves
```

For even a modest `n_states` this is *hundreds to thousands* of sparse factorizations per step — the
implicit path is effectively unusable at scale, and the explicit path pays a 20× factorization tax
it does not need. Three independent problems, each independently fixable:

| # | Bottleneck (file:line) | Why it is expensive | Cheap high-impact fix |
|---|---|---|---|
| 1 | Re-factorization every algebraic iteration — `DynamicSystem::solveNetwork` builds a new `SparseLinearSolver` per iter (`DynamicSystem.cpp`), base `Y_bus` unchanged between events | 20× redundant symbolic+numeric LU of an unchanging pattern | **Cache the factorization**: factor the base `Y_bus` once per topology; within `solveNetwork` reuse a persistent solver, refactor only when device Norton stamps change the matrix (or use a constant-`Y` + current-injection split so only the RHS changes). |
| 2 | Dense FD state Jacobian — `numerical_state_jacobian` (`DynamicSolver.cpp`) | `O(n)` network solves per Newton iter + dense `O(n³)` solve; ignores `use_analytic_jacobian` | **Sparse Jacobian**: analytic per-device blocks (each device already knows its `∂f/∂x`), assembled sparse; solve with sparse LU/KLU. Eliminates the per-column network solve. |
| 3 | Over-tight algebraic loop — `algebraic_network_max_iters=20`, `tol=1e-10` (`DynamicSolverOptions.hpp`) | Most points converge in 3–5 iters | Early-exit on residual; drop default to ~6. (Interim only — Phase 1 removes the loop entirely.) |

**The structural cure (Phase 1)** subsumes #1–#3: once bus voltages are *states* and the network is
an algebraic residual, there is **no nested loop and no per-column network solve** — a single sparse
Jacobian is factored once per Newton iteration for the *entire* coupled system, exactly as in PSID.

---

# Part II — Gap analysis mapped to the test matrix

Grouping the 27 blocked rows by the upgrade that unblocks them:

| Root-cause bucket | Rows / models | Unblocked by |
|---|---|---|
| **Formulation** (no residual/mass-matrix/eigenvalue) | 12× `compare-limited`; Test 35, 36 (`blocked-missing-formulation`) | **Phase 1** (DAE core) + **Phase 2** (small-signal) |
| **Dynamic branches** | Tests 10, 11, 25, 27 | **Phase 1** + **Phase 5** (dynamic RL line rows) |
| **Missing machines** | Marconato (04), SimpleAF/AndersonFouad (05, 06), SauerPai (45), FiveMassShaft (07) | **Phase 3** (composed) + **Phase 5** (shaft) |
| **Marconato δ drift** | Test 03 `compare-failing` | **Phase 1** (simultaneous solve removes partitioned angle drift) + init/frequency-reference audit |
| **Missing AVRs** | AVRTypeI/II, AVRSimple (13, 17), ESAC1A (20), EXST1/EXAC1/SCRX/ESST1A/ST6B/ST8C (39/40/47/55/59/61) | **Phase 3** + **Phase 4** (AVR library) |
| **Missing governors** | TGTypeI/II (13, 12), GAST (21), HYGOV (31), DEGOV/PIDGOV/WPIDHY/TGSimple/DEGOV1 (57/58/60/62/63) | **Phase 3** + **Phase 4** (TG library) |
| **Missing PSS** | IEEEST/STAB1 (30, 41), PSS2A/B/C (52/53/54) | **Phase 3** + **Phase 4** (PSS library) |
| **Missing IBR** | VSM (08/09/10/11/27), VOC (44), DERA (42), REGCA/REECB/REPCA/RENA (29/43), PeriodicSource (28), CSVGN1 (49) | **Phase 3** (inverter composition) + **Phase 4** (IBR library) |

Two observations drive sequencing: (a) **Phase 1 alone converts ~12 `compare-limited` rows into
comparable rows** and fixes Test 03; (b) the model-library rows are cheap to add *once the composed
architecture exists*, and nearly impossible to add cleanly before it.

---

# Part III — Upgrade plan

Phases are ordered by dependency. Phase 0 is independent and should start immediately; Phase 1 is
the keystone; Phases 2–5 depend on Phase 1.

### Phase 0 — Efficiency quick-wins (independent, low-risk) — *days*
Decoupled from the re-architecture; buy immediate headroom and de-risk current tests.
- **P0.1 Cache the network factorization.** Give `DynamicSystem` a persistent solver keyed on
  topology version; refactor only when `rebuildBaseMatrices` runs (events) or when a device's
  stamped admittance changes. Files: `DynamicSystem.cpp` (`solveNetwork`, `assembleEffectiveMatrices`),
  `solvers/SparseLinearSolver.*`.
- **P0.2 Early-exit + lower default** `algebraic_network_max_iters` 20→6 with residual break.
  File: `DynamicSolverOptions.hpp`, `DynamicSystem.cpp`.
- **P0.3 Sparse Newton correction.** Replace `colPivHouseholderQr` (dense) with a sparse LU on the
  Newton system in the implicit steppers. File: `DynamicSolver.cpp`.
- *Exit criterion:* implicit-solver step count of sparse factorizations drops by ≥10×; all existing
  `test_transient_dynamics` cases still green.

### Phase 1 — Mass-matrix DAE core (keystone) — *weeks*
Re-architect the integration around a single state vector and mass matrix, mirroring PSID's
`MassMatrixModel` first (Rosenbrock-friendly; we already have a Rosenbrock-Euler stepper).
- **P1.1 Global unknown layout.** `u = [V_r ; V_i ; device states]` (add branch states in Phase 5).
  Introduce a `DaeLayout` that owns bus-voltage indices and each device's state range.
- **P1.2 Mass matrix.** Diagonal `M`: `0` for the `2·n_bus` network rows and algebraic device
  states, `1` for differential states. Devices declare, per local state, differential vs algebraic
  (extend `DynamicDevice` with a `stateMassMatrix()`/`isDifferential(local)` hook).
- **P1.3 Unified residual.** One function `F(u)` = device rows `f(x,V)` ⊕ network rows
  `I_inj(x,V) − Y_bus·V` (reuse existing device `computeDerivatives` and `stamp` to build both),
  evaluated **matrix-free** over the sparse `Y_bus` — no inner solve.
- **P1.4 Sparse Jacobian.** Assemble `∂F/∂u` sparse from (a) analytic per-device blocks and the
  network `∂g/∂V = −Y_bus` (constant), with a numerical-coloring fallback for devices lacking an
  analytic block. Factor once per Newton iteration (sparse LU/KLU).
- **P1.5 Steppers on the DAE.** Recast Backward-Euler/Trapezoidal/Rosenbrock to operate on
  `M·u̇ = F(u)` using P1.4's Jacobian; keep the partitioned Heun path available behind an option for
  back-compat and cross-checks.
- **P1.6 Initialization.** Extend the existing PF + equilibrium-trim
  (`DynamicSystem::initializeStatesFromPowerFlow`) to seed the network-voltage states and enforce
  `F(u₀) ≈ 0` (consistent DAE initial conditions). This is also where the **Test 03 δ-drift** must
  be closed (audit the frequency-reference/rotor-angle convention against PSID
  `src/base/frequency_reference.jl`).
- *Exit criterion:* a machine-only case reproduces the current Heun result on the new DAE path;
  implicit step cost is `~O(nnz)` per step; `dynamic_fast_dxdt_inf_norm` at t0 unchanged.

### Phase 2 — Small-signal / eigenvalue API — *~1 week after Phase 1*
- Reduce the Phase-1 Jacobian by Schur complement `A_red = M_dd^{-1}(f_x − f_y g_y^{-1} g_x)`;
  compute eigenvalues, damping, and participation factors (Eigen dense `eig` on the reduced system;
  the reduced system is small). New `src/dynamics/SmallSignal.{hpp,cpp}` + a
  `POST /api/session/small_signal` hook and CSV/JSON export. Mirrors PSID `small_signal.jl`.
- *Exit criterion:* eigenvalues of a GENROU+SEXS+TGOV1 case match PSID Test 36 within tolerance.

### Phase 3 — Composed-device architecture (ports / inner_vars) — *weeks*
Generalize `MachineControlLink` into a small **inner-variable bus** so blocks compose like PSID.
- **P3.1 Generator composition.** `DynamicGenerator = Machine + Shaft + AVR + TurbineGov + PSS`
  over a 9-slot inner-var struct (`τe, τm, Vf, V_pss, VR, VI, ψd, ψq, Xad_Ifd`). Refactor the
  current `SynchronousMachine`(+ attached controllers) onto it; Shaft becomes its own block
  (SingleMass now, FiveMass in Phase 5).
- **P3.2 Inverter composition.** `DynamicInverter = Converter + DCSource + Filter + FreqEstimator +
  InnerControl + OuterControl` over the 25-slot inverter inner-var struct. Re-express the existing
  GFL/GFM as (AverageConverter + RL/LCL Filter + Reduced/Kaura PLL + droop/PI outer + PI inner) so
  the compact models remain a special case.
- *Exit criterion:* current GFL/GFM and machine+controller tests pass unchanged on the composed
  runtime (behavioral equivalence), proving the refactor is non-regressive.

### Phase 4 — Model-library expansion (prioritized) — *ongoing, parallelizable*
With Phases 1 & 3 in place, blocks are additive. Suggested priority tied to the matrix:
1. **AVR:** AVRTypeI (Test 17), AVRTypeII/AVRSimple (13), then EXST1/ESST1A/ESAC1A/EXAC1/SCRX/ST6B/ST8C.
2. **Governor:** TGTypeI/II (13, 12), GAST (21), HYGOV (31), then DEGOV/PIDGOV/WPIDHY/DEGOV1/TGSimple.
3. **PSS:** IEEEST (30), STAB1 (41), PSS2A/B/C (52–54).
4. **Machines:** promote SimpleMarconato→Marconato (04), SimpleAF→AndersonFouad (05, 06), SauerPai (45).
5. **IBR:** VSM GFM (08–11), VOC (44), full REGCA/REECB/REPCA + RENA (29/43), DERA (42), CSVGN1 (49),
   PeriodicVariableSource (28).
Each new block ships with (a) a catalog entry in `DynamicModelCatalog.cpp` (already the single source
of truth, guarded by `test_dynamic_model_catalog`), (b) a builder mapping, and (c) a PSID trace test.

### Phase 5 — Dynamic branches & multi-mass shaft — *after Phase 1*
- **Dynamic RL lines:** add branch current states `L·dI/dt = V_from − V_to − R·I` as differential
  rows (mirror `dynline_model.jl`); a per-branch flag selects static (algebraic) vs dynamic. Unblocks
  Tests 10/11/25/27.
- **FiveMassShaft:** a 10-state shaft block in the Phase-3 composition. Unblocks Test 07.

---

## IV. Validation strategy

Adopt PSID's two-gate contract per component (the matrix already assumes it):
1. **Trace parity:** run the same perturbation, compare device signals (δ, ω, eq_p, ed_p, Vf, τm,
   PLL ω, …) against a stored PSID trace over 0–2 s within RMS/max tolerances. Reuse the existing
   `tools/psd_validation` harness and `device_output_series` test helpers.
2. **Small-signal parity:** compare the Phase-2 eigenvalues/damping against PSID (Test 36 vs ANDES).
Keep the current `test_transient_dynamics` suite as the non-regression gate through every phase.

## V. Risk & sequencing notes

- **Biggest risk is Phase 1 initialization** (consistent DAE `u₀`) and the Test-03 angle-reference
  audit; budget explicit time for it. The equilibrium-trim we already have is the right starting
  point.
- **Back-compat:** keep the partitioned Heun path selectable throughout; it is the oracle for
  validating the DAE path device-by-device.
- **Effort shape:** Phase 0 (days) → Phase 1 (the bulk) → Phases 2/5 (short) → Phases 3–4 (steady,
  parallelizable, matrix-driven).
- **Do not expand the model library before Phase 3.** Adding AVR/TG/PSS/IBR variants onto the
  current monolithic devices would create throwaway code and widen the surface that Phase 3 must
  rewrite.

## VI. The one decision to confirm

**Incremental (Phase 0 only) vs. re-architecture (Phase 0 → 1 → …).** Phase 0 makes the current
engine usable but will *never* reach PSID formulation/eigenvalue/dynamic-branch parity — those are
intrinsic to the simultaneous-DAE form. Recommendation: **commit to Phase 1** as the keystone; it is
the single change that converts the largest block of the matrix and removes the efficiency ceiling
at the same time. Everything else is additive library work on top of it.

---

### Appendix — key anchors

HACDCPF: `src/dynamics/DynamicSystem.cpp` (`solveNetwork`, `evaluateDerivatives`,
`rebuildBaseMatrices`, `assembleEffectiveMatrices`) · `src/dynamics/DynamicSolver.cpp`
(`numerical_state_jacobian`, steppers) · `include/hacdcpf/dynamics/DynamicSolverOptions.hpp` ·
`include/hacdcpf/dynamics/devices/{DynamicDevice,BasicDynamicDevices,MachineControlLink}.hpp` ·
`src/dynamics/DynamicModelCatalog.cpp` · `tools/psd_validation/psd_component_test_matrix.md`.

PSID: `src/base/{simulation,simulation_inputs,simulation_model,mass_matrix,jacobian,small_signal,
device_wrapper,ports,frequency_reference}.jl` · `src/models/{system,network_model,dynline_model,
device}.jl` · `src/models/generator_models/*` · `src/models/inverter_models/*`.
