# Distribution Parameter Completion Standards Crosswalk

## Contract

The Model IO parameter-completion workflow is provenance-aware. It may replace
only fields that are missing or explicitly marked as importer/handbook
estimates. Manufacturer nameplates, utility asset records, and user-authored
parameters are not overwritten.

The implementation distinguishes three kinds of basis:

- `normative_parameter_table`: a numerical value is obtained from a standard
  table, such as conductor resistance at 20 C;
- `normative_calculation_method`: the input value is project-specific, while
  its conversion follows a standard method;
- `engineering_assumption`: the source file lacks the required asset data and
  a configurable screening value is used.

## Field Crosswalk

| HACDCPF field | Completion | Basis | Standard or source | Boundary |
| --- | --- | --- | --- | --- |
| `ACBranch.r_ohm_per_km` | conductor table plus temperature correction | normative table + calculation | GB/T 3956-2008 / IEC 60228:2004; GB/T 12706.1-2020 | Exact only when material, cross-section, and insulation are known. |
| `ACBranch.x_ohm_per_km` | overhead/cable configurable default | engineering assumption | DL/T 5220-2021, GB/T 1179-2017, GB/T 14049-2008, and GB/T 12706.1-2020 define the equipment/design context but do not prescribe one universal reactance. | Replace with geometry/manufacturer sequence impedance when available. |
| `ACBranch.rate_*_mva` | voltage and screening current density | engineering assumption | Product/design context from GB/T 1179-2017, GB/T 14049-2008, GB/T 12706.1-2020, and DL/T 5220-2021. | Not a substitute for installation-specific ampacity, ambient, grouping, or soil thermal calculations. |
| `Transformer2W.sn_mva` | capacity parsed from SVG label, otherwise configured default | source metadata or engineering assumption | GB/T 1094.1-2013 and GB/T 6451-2015 define nameplate/parameter context. | Manufacturer nameplate is authoritative. |
| `Transformer2W.vk_percent`, `vkr_percent` | configurable distribution-transformer defaults | engineering assumption | GB/T 1094.1-2013; GB/T 6451-2015 | Product series and nameplate must replace defaults for protection or loss studies. |
| `Transformer2W.pk_kw` | `Sn * vkr%` identity | calculation from inferred/nameplate values | GB/T 1094.1-2013 transformer loss/impedance definitions | Inherits the confidence of `vkr_percent`. |
| `ExternalGrid.r_pu`, `x_pu`, `r0_pu`, `x0_pu` | convert declared/configured fault level and R/X ratio | normative calculation method | GB/T 15544.1-2013 / IEC 60909-0:2016 | Synthetic `Ssc` and R/X remain project assumptions until utility fault-level data is supplied. |
| `ACBus.base_kv` | SVG import option or active parameter profile | standard voltage context | GB/T 156-2017 / IEC 60038 | The drawing/project nominal voltage is authoritative. |
| `ACBus.vmin_pu`, `vmax_pu` | active parameter profile | power-quality context | GB/T 12325-2008 | Study operating limits may be stricter and remain editable. |
| `ACSystem.freq_hz` | active parameter profile | frequency context | GB/T 15945-2008 | The interconnected system declaration is authoritative. |
| estimated transformer-terminal load | `Sn * load_factor * power_factor` | engineering assumption | No national standard is claimed for a universal feeder load factor. | Replace with AMI/SCADA/planning demand data before planning decisions. |

## Registered References

- GB/T 156-2017, *Standard voltages*.
- GB/T 1179-2017, *Round wire concentric lay overhead electrical stranded conductors*.
- GB/T 14049-2008, *Aerial insulated cables for rated voltages of 10 kV and 35 kV*.
- GB/T 3956-2008 / IEC 60228:2004, *Conductors of insulated cables*.
- GB/T 12706.1-2020, extruded-insulation power cables, Part 1.
- GB/T 1094.1-2013, *Power transformers - Part 1: General*.
- GB/T 6451-2015, technical parameters and requirements for oil-immersed power transformers.
- GB/T 12325-2008, *Power quality - Deviation of supply voltage*.
- GB/T 15945-2008, *Power quality - Frequency deviation for power system*.
- GB/T 15544.1-2013 / IEC 60909-0:2016, short-circuit current calculation.
- DL/T 5220-2021, design code for 10 kV and below overhead distribution lines.

The official National Public Service Platform for Standards is
<https://openstd.samr.gov.cn/>. DL/T applicability and current status should be
checked through the National Energy Administration at
<https://www.nea.gov.cn/> before using defaults in a regulated deliverable.

## SVG Automatic Completion

`SvgDistributionImportOptions::auto_complete_parameters` defaults to `true`.
The importer first recovers source-faithful identity and drawing geometry, marks
all invented electrical fields as `svg_geometry_estimate`, and then invokes the
shared design-handbook completion workflow. Setting the option to `false`
preserves the initial configurable SVG estimates.

The completion report returns every suggestion, confidence, action, standard
source, changed-field count, and the reference list. Strict import still rejects
the format because geometry scale, source strength, and demand remain inferred.
