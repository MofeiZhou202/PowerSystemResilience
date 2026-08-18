# IEC-CGE Distribution SVG Import and Export

## Implemented profile

`hacdcpf::io::from_svg_distribution` imports the annotated single-line SVG
dialect used by the feeder drawings in `data/`. The profile is identified by:

- SVG XML with `xmlns:cge="http://iec.ch/TC57/2005/SVG-schema#"`;
- equipment metadata in `cge:psr_ref` (`objectid`, `psrtype`, `classname`);
- equipment layers such as `ACLineSegment_Layer`, `Breaker_Layer`,
  `PowerTransformer_Layer`, and `BusbarSection_Layer`;
- electrical connectivity represented by line vertices and transformed device
  terminals, rather than CIM `Terminal` / `ConnectivityNode` references.

This is not an IEC 61970/61968 CIM network document. A decorative SVG without
`cge:psr_ref` equipment is rejected.

## Mapping contract

| SVG class | HACDCPF model | Binding |
| --- | --- | --- |
| `PWConductorSecPSR` / `PWCableSecPSR` | `ACBranch` | Object ID/name retained; polyline segments become branches. |
| `PWConnectLine` / `PWInnerLinkLine` | connectivity-node merge | Ideal connector; no branch impedance is invented. |
| `PWBusbarPSR` | connectivity-node merge | Endpoints and vertices lying on the busbar are merged. |
| breaker, disconnector, load-switch, fuse classes | `Switch` | Type retained; symbol variant `@1` maps to open state. |
| `PWBusCouplePSR` | `Switch` | Polyline endpoints are the two bus-coupler terminals. |
| `PWOPTransformerPSR` | `Transformer2W` + terminal `Load` | Label capacity is used when present; otherwise a configurable default is used. |
| `Substation` | synthetic `ExternalGrid` location | One source per substation-connected component. |

Text groups named `TXT-PD_<object-id>` are associated with equipment after
normalizing separator characters. Source `objectid` remains the fallback name,
so external stable identity is not replaced by vector position or graph index.

## Parameter and validity boundary

The source drawings do not contain a complete electrical asset record. The
import report therefore always uses `UnitAssertion::BestEffort` and records
coercions for:

- drawing-distance to line-length conversion;
- overhead/cable resistance and reactance defaults;
- transformer impedance defaults;
- transformer-terminal load factor and power factor;
- synthetic external-grid equivalents.

All values are configurable through `SvgDistributionImportOptions`. By default,
the importer then applies the shared standards-aware parameter completion only
to fields marked as SVG estimates. The report contains line-level suggestions,
confidence, changed-field counts, and national/industry/IEC references. Set
`auto_complete_parameters=false` to retain the initial SVG estimates. Strict mode
rejects the import because these inferences are unavoidable. Components not
connected to a substation through currently closed switches remain
`BusType::ISOLATED`; the importer does not add a source merely to make a dead
island appear supplied. Power-flow output consequently reports zero voltage for
those isolated buses.

Setting `auto_add_external_grids=false` disables source synthesis entirely and
marks every recovered component as isolated until the caller provides an
explicit source.

## Export contract

`hacdcpf::io::to_svg_distribution` and `save_svg_distribution` export the AC
distribution subset as a layered IEC-CGE SVG. The document contains
`cge:psr_ref` stable equipment identity and, by default, a
`hacdcpf:model` extension with electrical parameters needed for lossless
round-trip through this adapter. The exporter covers AC buses, branches,
switches, two-winding transformers, external grids, and transformer-terminal
loads. Standalone loads that cannot be embedded and non-AC assets are not
silently approximated: their counts and warnings are returned in
`SvgDistributionExportResult`.

The generated layout is deterministic from the AC graph and configurable with
`horizontal_spacing`, `vertical_spacing`, and `margin`. This is an engineering
single-line layout, not preservation of an imported drawing's original visual
coordinates.

## Runtime

The GUI Model IO toolbar exposes **导入配电SVG**. It posts:

```json
{
  "svg_string": "<svg ...>",
  "name": "feeder name"
}
```

to `POST /api/session/load_svg_distribution`. The response includes the normal
system summary plus `_svg_*` fields for source-object counts, recovered nodes,
unresolved objects, synthetic sources, isolated components/buses, warnings, and
model scope. `_svg_parameter_completion` contains the full standards-aware
completion report. After import, the ordinary session PF/OPF/analysis routes operate
on the resulting `HybridPowerSystem`.

The same toolbar exposes **导出配电SVG**. It synchronizes the canvas and posts to
`POST /api/session/export_svg_distribution`; the response contains the SVG text,
exported/omitted counts, warnings, and
`model_scope=ac_distribution_iec_cge_svg`. The browser downloads the result as
`*_distribution.svg`. Server-side layout and serialization run outside the
session mutex after taking a model snapshot.

The standards and assumptions crosswalk is maintained in
`distribution_parameter_completion_standards.md`.

## Regression fixtures

`tests/test_io_svg_distribution.cpp` imports and solves both checked-in files:

| Fixture | Recovered source lines | Transformers / estimated loads | Recovered buses | PF result |
| --- | ---: | ---: | ---: | --- |
| `张庄C503线-架空-丽水市.svg` | 30 | 11 / 11 | 68 | converged |
| `新区B259线-混合.svg` | 69 | 14 / 14 | 224 | converged |

The tests also require zero unresolved recognized objects, cable/overhead type
separation, explicit dead-island voltages, stable-ID and electrical-parameter
round-trip, layered SVG export, and rejection of plain SVG artwork. The GUI API
smoke test covers export followed by re-import.

## Security

The parser rejects `DOCTYPE` and `ENTITY` declarations, does not load external
resources, and ignores visual definitions outside the recognized metadata and
geometry subset. File-size and element-count caps remain a future hardening
item for untrusted public uploads.
