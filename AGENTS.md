# Project Instructions

This working tree is on the `main` branch and is intended to be merged later with the `luosipeng` branch.

## Merge and solver-safety constraints

- Do **not** modify `MIPSolvers` or vendored solver internals for normal feature work.
- Do **not** change core solver algorithms, branch-and-cut logic, power-flow kernels, or optimization adapters unless the user explicitly asks for solver-level changes.
- Keep changes for GUI features and resilience workflows localized to this repository's business/API/UI layers whenever possible.
- For distribution resilience work, prefer reusing or porting the proven reference implementation from `d:/ACDCHybridSimulation/dist_resilience_standalone` instead of rewriting solver logic.

## Resilience GUI expectations

- The Web resilience UI should keep AC and DC fault inputs separate.
- Generated typhoon/resilience scenario JSON may contain typed AC/DC faults; preserve `branch_type`/`branch_kind` through import, API request, assessment, and exported results.
- Switch-aware resilience defaults depend on the loaded case:
  - If the case has no AC switches, AC circuit breakers, or DC circuit breakers, default "consider switches" to unchecked.
  - If the case has any of those devices, default it to checked.
  - Once the user manually changes the checkbox, do not overwrite their choice automatically.
