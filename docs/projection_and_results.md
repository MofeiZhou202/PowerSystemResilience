# Projection and Result Attribution Contract

Updated: 2026-08-15

Rich engineering components are converted to canonical solver objects by
`projection::RichToCanonicalOperator`. Solver observables return to authored
identity through `projection::CanonicalToRichOperator`.

## Invariants

1. Public solve results and GUI payloads use authored-space identity.
2. Projection is idempotent for a model already carrying a valid projection
   certificate.
3. Merge and dead-island deletion maps are composed, not overwritten.
4. Intensive bus quantities such as voltage are broadcast across a merge.
5. Extensive quantities such as shedding are split using explicit
   participation semantics, never broadcast blindly.
6. Component attribution uses `canvas_type`, `canvas_index`, `position`,
   canonical sources, recovery class, and terminal sign convention.
7. Size mismatches are diagnostics; consumers must not silently pad result
   vectors into another index space.
8. Small impedance alone is not evidence of node identity. A physical branch
   is contracted only when exact switch/breaker semantics or explicit
   `ACBranch::ideal_connectivity` provenance places it on the merge whitelist.

## Operators and provenance

`ProjectionBundle` contains the canonical system, component mappings,
projection report, and certificate. `BusMergeMap`, branch provenance maps, and
component expansion records carry authored-to-canonical relationships.

An in-service `ACBranch` marked `ideal_connectivity` represents the equivalence
relation `u ~ v`, not a series admittance. Projection union-finds these rows
before Ybus assembly, aggregates extensive bus injections and control limits,
and records the authored-to-canonical bus map. Solved intensive values such as
voltage are broadcast back to every authored member. This exact contraction is
appropriate for source-declared ideal connectivity, including BPA zero-data L
cards. Unmarked low-impedance lines remain explicit branches, so their series
flow, charging, rating, and attribution are not discarded.

`RichResultAttribution` classifies recovered values as strong, approximate,
audit-only, or unsupported. A total rich identity does not imply that every
physical observable is solved; audit-only rows must say why a value is absent.

## Terminal power convention

Component terminal P/Q uses positive injection into the network. Network edge
diagnostics additionally expose from/to terminal outflow. Consumers must use
the row's declared `sign_convention` instead of inferring signs from a device
name.

PF, OPF post-PF, and TSPF Canvas frames use the same authored component keys.
Standalone transformer terminal flow can be recovered from solved AC phasors
and transformer parameters. Same-bus source circuit breakers use the recovered
external-grid injection (`same_bus_external_grid_cut`) so they are not forced
to zero by bus contraction.

## P/Q balance diagnostics

`power_balance_diagnostics.all_buses` reports, per AC/DC bus:

- net device injection;
- terminal export;
- active and reactive residual in MW/MVar and kW/kVar;
- detailed contributing terminals;
- explicit source/slack rows.

The GUI P/Q mode displays these residuals. A source correction used to close a
slack equation must be labeled; it must not hide an ordinary-bus mismatch.

## Verification

Projection metamorphic tests cover idempotence, merge/delete composition, and
result recovery. `tools/gui_api_e2e.py` additionally checks `power_system.json`
for Grid attribution, source-side and feeder CB P/Q, PF/OPF consistency, TSPF
frames, and Bus 1 P/Q closure.
