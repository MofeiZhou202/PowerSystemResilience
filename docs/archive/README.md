# Documentation Archive

This directory contains historical evidence and theory retained for provenance.
Nothing below `docs/archive/` defines current runtime behavior. Verify every
claim against `include/`, `src/`, registered tests, and the active
[`docs/README.md`](../README.md) before reusing it.

Do not add archived documents back to active topic tables. If archived material
becomes implementation-backed, extract the verified behavior into a focused
living contract and leave the historical source here.

## Audits and diagnostics

| Material | Why archived |
|---|---|
| [`power_flow_math_audit.tex`](audits/power_flow_math_audit.tex) | Point-in-time defect audit; open findings now belong in the living module code audit and regression tests. |
| [`large_hybrid_ipm_diagnostics.md`](audits/large_hybrid_ipm_diagnostics.md) | Solver investigation and benchmark observations tied to a specific implementation state. |
| [`reactive_power_optimization_validation.md`](audits/reactive_power_optimization_validation.md) | Point-in-time validation record; the active OPF manual and tests own current behavior. |

## Theory and design references

| Area | Material | Boundary |
|---|---|---|
| Converter controls | [`Updated_GFL_GFM_Control_Modeling.md`](theory/Updated_GFL_GFM_Control_Modeling.md), [`multiple_converter.md`](theory/multiple_converter.md), [`multiple_converter_r1.md`](theory/multiple_converter_r1.md) | Target models and engineering proposals; implemented subsets vary by solver. |
| Harmonics | [`harmonic_general.md`](theory/harmonics/harmonic_general.md), [`harmonic_initial.md`](theory/harmonics/harmonic_initial.md), [`harmonic_nic.md`](theory/harmonics/harmonic_nic.md), [`harmonic_multiple_nic.md`](theory/harmonics/harmonic_multiple_nic.md), [`harmonic_three_phase.md`](theory/harmonics/harmonic_three_phase.md) | Research derivations and proposed extensions; use the active harmonic verification note for current coverage. |
| Dynamics | [`dynamics_electromechanical_transient_design.md`](theory/dynamics_electromechanical_transient_design.md), [`powersimulationsdynamics_interop.md`](theory/powersimulationsdynamics_interop.md) | Design assessment and optional interop proposal; use the active transient runtime contract for shipped behavior. |
| Reliability | [`cyber_physical_reliability_extension.md`](theory/cyber_physical_reliability_extension.md) | Fidelity ladder whose Level 1 has an implemented subset and Levels 2-4 remain proposals. |

## Legacy technical notebook

[`reference/technical_notebook/main.tex`](reference/technical_notebook/main.tex)
is a broad implementation snapshot retained because it contains useful
derivations and field dictionaries. Its duplicated status tables, test results,
gap audit, and roadmap sections can drift independently of focused contracts,
so the notebook is no longer an active handbook. The tracked rendered PDF was
removed; compile the source locally only when historical inspection requires it.

## Retention rules

- Keep sources only; do not commit generated PDFs or LaTeX intermediates.
- Do not update archived status claims to mimic current behavior.
- Fix broken archive links when files move, but put new runtime facts in active
  contracts.
- Delete an archived item only when it is fully duplicated and has no remaining
  provenance value.
