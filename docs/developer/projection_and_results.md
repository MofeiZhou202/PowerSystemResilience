# Projection and Result Attribution Contract

Updated: 2026-09-08

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

## Attribution lookup contract

`CanonicalToRichOperator::apply` builds call-local AC/DC voltage indexes and
stable-ID-to-position indexes; no index or operating-point value survives the
call. Voltage lookup retains PF-vector precedence, authored `vm_pu` fallback
for a missing vector entry, and the existing zero fallback for a missing bus.
That fallback is not evidence that a bus voltage was solved. Duplicate IDs
retain the first matching position; validation remains responsible for rejecting
invalid identities. OPF maps still refer to canonical container positions.

Result-row indexes are constructed lazily by component family after all rows
exist. Electrical lookup checks type, domain and stable ID. Result order,
source order, terminal signs and recovery classifications are unchanged.
`ProjectionReport` source records currently have no domain: provenance therefore
retains all same-type/same-ID matches, including across AC/DC domains. This is
an existing provenance limitation, not permission to mix electrical domains.

For B buses, C result rows, M source mappings and G generators, repeated
voltage/source/position scans previously included O(C*B + M*C + G^2) work.
Hash indexing changes these lookup terms to expected O(B + F*C + G + M + Q),
where F is the fixed number of queried component families and Q counts queries
and emitted matches. This bound excludes terminal-flow computation and other
remaining scans/copies in attribution. See AUD-097 in the
[audit ledger](../testing/module_code_audit.md) for measured scope and limitations.

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

For profiling the legacy full PF presentation path, run:

```sh
python3 tools/pf_presentation_profile.py --build --run
```

This developer tool generates an instrumented copy of `tests/run_gui_server.cpp`
under `output/pf-presentation-profile/` and links the retained incremental Release
overlays recorded by the previous attribution build. It requires that local
build provenance and baseline binary; it does not configure dependencies or
change production sources. Unique source anchors fail explicitly if the route
changes. It starts and stops private temporary servers, leaving existing
services untouched. The tool is a performance experiment, not a registered
CTest target or a clean-build validation.

The generated server adds experiment-only response headers for 19 disjoint
presentation stages, three optional lookup timers, result-row merge counts,
and scope lifetimes. These headers are not part of the production API or GUI.
The existing `Server-Timing` serialization metric measures `out.dump()`;
constructing JSON objects, copying them and destroying them are distinct costs.
The existing `presentation_ms` stops before local objects in `add_geo_data` are
destroyed; its remainder in `solve_ms` therefore must not be read as pure
numerical solver time. Handler lifetime additionally measures response-object
cleanup after `set_content`; time outside that scope still includes framework,
scheduling, socket and client costs, not an isolated network-transfer metric.

The protocol alternates baseline/coarse/scan modes with five fresh servers per
case/mode (case14, case118, ACTIVSg2000 and case9241pegase). Non-timing response
fields must match exactly after excluding `timing`, `profiling` and
`execution_time_sec`. Exact UTF-8 value sizes are extracted from the actual
top-level JSON wire text. Stage sums must agree with presentation within
max(1 ms, 1%); the predeclared large-case median wall overhead limit is 5%.
Per-call scan timing is kept separate from coarse measurement and includes
clock overhead. Measurements and residual boundaries are recorded in the
[audit ledger](../testing/module_code_audit.md).

Projection metamorphic tests cover idempotence, merge/delete composition, and
result recovery. `test_result_attribution` additionally checks reordered sparse
IDs, same-ID AC/DC components, canonical OPF positions, short/null/changing PF
inputs and ordered source mappings. `tools/attribution_performance_benchmark.py`
compares all serialized attribution fields against the baseline on synthetic
and MATPOWER cases. `tools/gui_api_e2e.py` additionally checks `power_system.json`
for Grid attribution, source-side and feeder CB P/Q, PF/OPF consistency, TSPF
frames, and Bus 1 P/Q closure.
