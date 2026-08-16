# Model, Data, and Result Semantics Contract

This living contract defines how one physical component keeps one meaning while
moving through rich-model input, canonical projection, numerical solvers,
results, replay, and dynamic initialization. Source and registered tests remain
authoritative. The executable registry in
`include/hacdcpf/model/model_semantics.hpp` extends the GFM reference pattern
to every rich component collection and every production source module.

## Semantic layers

| Layer | Owns | Must not own |
|---|---|---|
| Rich authored model | Stable component `.index`, engineering fields, units, control intent | Solver vector positions or graph indices |
| Canonical projection | Bus/branch merging and `ComponentMapping` | New device physics or silent unit conversion |
| Solver data | Domain-qualified positions, per-unit states, sparse coordinates | Public identity or presentation labels |
| Solver-local state | Iteration variables and active NCP branch | Authored setpoint mutation |
| Result certificate | Stable ID, units/index space, residuals, scope and validity | Uncertified inferred mode claims |
| Replay/initialization | Explicit source result plus mapping policy | Reinterpreting a field by module convention |

Every AC/DC lookup is domain-qualified. Component `.index`, vector position,
and graph node/edge index are distinct types in meaning even where represented
by `int`. Public results return stable component IDs after projection.

## Concrete type ownership

| Type or field family | Semantic role | Identity/index space | Mutation rule |
|---|---|---|---|
| `HybridPowerSystem`, `VSCConverter` | Persistent rich engineering model and control intent | Stable authored component/bus `.index` | I/O, GUI editing, and explicit workflows may author it |
| `BusMergeMap`, `BranchExpandMap`, `ComponentMapping` | Rich-to-canonical provenance | Stable IDs mapped to canonical positions and back | Projection owns; consumers treat as immutable evidence |
| `powerflow::SolverData` | Canonical numerical assembly and aggregated injections | Domain-qualified vector positions | Solver preparation may refresh values, never redefine public identity |
| `PhaseVSC` | Three-phase OPF/PF adapter DTO | Phase-node and DC-terminal positions | Rebuilt from the rich model; never replayed as authored data |
| `VSCConverterDynamicParams` | Transient runtime DTO | Stable component ID plus dynamic bus positions | Shared resolver first, explicit dynamic profile override second |
| `InitialState` | Numerical seed only | Canonical AC/DC vector order declared by `SolverData` | May initialize a solve; never changes setpoints or control mode |
| `VSCLimitStateResult` | Per-unit solver-local certificate | Stable converter `.index` | Produced only by the balanced PF NCP solve |
| `VSCTransfer` | Public engineering-unit transfer plus certificate projection | Stable authored converter and bus IDs | Derived from a result and projection maps; never inferred from bus net injection |
| `DynamicInitializationSummary` | PF-to-dynamic seed and trim certificate | Aggregate plus stable-ID matched devices | Reports comparison only; not another PF result |

The word “same” therefore means the same stable physical device and resolved
physical parameter, not the same C++ object instance or vector coordinate.
Adapter DTOs and result DTOs intentionally duplicate values for execution and
transport, but they do not acquire ownership of their meaning.

## Repository-wide executable inventory

`model::component_runtime_semantics()` contains one row for every collection in
the existing `io::component_io_mappings()` registry. Each row records stable
identity, domain-qualified terminals, units, sign convention, parameter owner,
result semantics, and one fidelity classification for PF, OPF, dynamics,
time-series, reliability/resilience, and market/carbon workflows. The allowed
classifications are `Exact`, `Equivalent`, `Aggregated`, `BoundaryInjection`,
`Projected`, `Unsupported`, and `NotApplicable`.

`model::module_data_contracts()` contains one row for every top-level `src/`
module. It records authoritative input, execution view, public output, identity
space, units, and fidelity boundary. This is an inventory and admission guard,
not a claim that all modules use identical equations. Shared physical fields,
units, signs, IDs, and precedence must agree; a reduced analysis may use a
different mathematical model only when its result scope declares that boundary.

Two legacy/native DC pairs have explicit shared views:

- `dc_storage_execution_view()` maps persistent `DCStorage` to the common
  active-power `Storage` execution schema with `q=0`; `materialize_dc_storage()`
  applies that view to an execution copy and is idempotent.
- `dc_static_generator_pf_view()` maps native `StaticGeneratorDC.p_set_mw` to
  the PF injection `StaticGenerator.p_mw`, preserving limits, controllability,
  marginal cost, and carbon factor. Technology, profile, reliability, and
  dynamic metadata intentionally remain owned by the native rich object.

Bus-level and component-level demand are additive in both AC and DC domains.
The presence of component loads changes the execution storage/ZIP calculation,
not whether authored bus demand exists. A consumer must never select one table
and silently discard or duplicate the other.

## GFM authored source of truth

`VSCConverter` owns the balanced/phase-domain/dynamic shared Norton parameters.
The resolver `model::resolve_gfm_norton_parameters` is the only precedence rule.

| Physical quantity | Preferred authored field | Legacy fallback | Resolved unit |
|---|---|---|---|
| Internal-voltage magnitude | `gfm_internal_voltage_set_pu` | `v_ac_set_pu` | pu on system voltage base |
| Internal-voltage angle | `gfm_internal_angle_set_deg` | `v_ac_angle_set_deg` | radians after resolution |
| Virtual resistance | `gfm_virtual_r_pu` | `r_conv_ac_pu` | pu on system impedance base |
| Virtual reactance | `gfm_virtual_x_pu` | `x_sc_pu` | pu on system impedance base |

A zero preferred value means “use the legacy fallback”; it is not a second
module-specific default. A nonzero explicit magnitude or angle selects the
explicit internal-reference pair, so an explicitly zero angle is represented
by setting the explicit magnitude and leaving the explicit angle at zero. PF,
three-phase OPF/PF adapters, transient model construction, and the GUI backend
all call the shared resolver.

Dynamic profiles are an explicit transient-only override applied after shared
resolution. If a profile changes the Norton seed, PF-to-transient certification
must fail its quantitative comparison and report a warning; it may not remain
marked continuous merely because the stable component ID matches.

## Control and equation semantics

`ConverterMode` is the stored control intent. Standard JSON uses canonical
strings. JPC keeps codes 1--3 backward compatible and appends 4=`AC_PV`,
5=`AC_GRID_FORMING`, and 6=`DC_V_DROOP_AC_V`. GUI editing and replay preserve
the enum; adapters may derive a module-specific role but may not overwrite the
authored enum in the stored system.

The same physical parameters do not imply identical equations in every domain:

| Consumer | Meaning of resolved `E∠δ, Zv` | Certification boundary |
|---|---|---|
| Balanced PF | Fixed reference plus solved six-state GFM Norton/current-limit NCP block | Exact PF NCP residual and current certificate |
| Balanced OPF | Not an endogenous KKT constraint | Engineering inequalities in OPF, then balanced post-PF replay certificate |
| Three-phase OPF | Positive-sequence GFM reference and initial internal voltage; phase-domain internal voltage remains an OPF state | Native phase-current/VUF constraints; not the balanced NCP |
| Quasi-steady time series | Authored GFM mode retained through dispatch replay | Per-step post-PF certificate |
| Transient initialization | Stable-ID PF Norton state seeds the dynamic device before network trim | Separate PF-seed error and full-dynamic trim residual |

These distinctions are declared in `model_scope`, `ValidityFlags`, or
`model_limitations`; sharing data does not authorize claiming mathematical
equivalence.

## Result ownership

`VSCLimitStateResult` is the solver certificate in per unit.
`VSCTransfer` is the public engineering transfer in MW/Mvar plus the same
certificate metadata. Both are keyed by stable converter `.index` and expose
internal voltage and terminal current as rectangular per-unit components.
`DynamicInitializationSummary` reports the subsequent seed comparison; it is
not another power-flow solution.

The following must stay synchronized in every serializer: `gfm_norton_model`,
priority, active status, current and margin, complementarity residual,
`internal_voltage_{real,imag}_pu`, `terminal_current_{real,imag}_pu`, and the
`vsc_gfm_norton_modelled`, `vsc_gfm_priority_limit_enforced`, and
`vsc_gfm_island_reference_modelled` validity flags, plus stable
`gfm_island_reference_vsc_indices` diagnostics where applicable.

## Executable guards

- `tests/test_vsc_limit_ncp.cpp` locks explicit-vs-legacy precedence, standard
  and JPC round trips, stable IDs, PF equations, oracle modes, and result flags.
- `tests/test_three_phase_hybrid_opf.cpp` locks all four resolved parameters at
  the rich-to-phase-domain adapter boundary.
- `tests/test_transient_dynamics.cpp` locks stable-ID PF-to-dynamic Norton seed
  errors to `1e-8`.
- `tools/gui_api_e2e.py` locks GUI edit round trips and HTTP result certificates.
- `tests/test_model_semantics_contract.cpp` requires one runtime-semantics row
  for every I/O component collection, one data contract for every `src/`
  module, complete workflow/fidelity fields, exact shared DC adapter mappings,
  idempotent DC storage materialization, equivalent SolverData constructors,
  and additive bus-plus-component load demand.

## Remaining structural debt

The production GUI backend still contains a local three-phase monolithic
assembly in addition to `build_three_phase_hybrid_model`. Its GFM fields now use
the shared resolver, but the duplicate topology/load/generator adapter should be
replaced by the library adapter in a dedicated refactor with response-parity
tests. Until then, new physical fields must enter the shared model resolver and
both adapters in the same change.

The large 300/2000-bus GFM builders are continuation datasets: the real MTDC
limit benchmark is solved first and its voltage state initializes an added
nonbinding GFM Norton block. They prove sparse structural scaling on real hybrid
networks, not flat-start robustness, multi-GFM binding scaling, or GPU support.
