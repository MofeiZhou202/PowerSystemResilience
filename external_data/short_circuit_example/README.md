# Classical Short-Circuit Examples

These files are small hand-calculable short-circuit benchmarks for checking the
IEC 60909 short-circuit modules and GUI.  Each JSON file is a normal
`HybridPowerSystem` case and includes an extra top-level
`expected_short_circuit` block.  The normal loader ignores that block; tests and
humans can use it as the hand calculation reference.

Use the values with `/api/session/sc_detailed` or the GUI's short-circuit module.
Set the GUI advanced options to the `options` listed in each check, especially
the voltage factor `c_factor`, because the GUI default may be different.

## Case List

### Hand-Calculable Cases

- `sc_hand_01_two_bus_source_line.json`  
  Two-bus 10 kV source plus one line. Checks base-current conversion and
  Thevenin impedance addition.

- `sc_hand_02_three_bus_radial_feeder.json`  
  Three-bus radial feeder. Checks that fault level decreases as feeder
  impedance increases downstream.

- `sc_hand_03_slg_low_zero_sequence.json`  
  Single grounded source with `Z0 < Z1`. Checks that SLG current can exceed
  three-phase current.

- `sc_hand_04_external_grid_max_min.json`  
  One-bus external-grid case using `s_sc_max_mva` and `s_sc_min_mva`. Checks
  IEC max/min source-data selection.

- `sc_hand_05_transformer_correction.json`  
  External grid plus two-winding transformer. Checks IEC transformer correction
  factor in canonical space.

- `sc_hand_06_converter_grid_following.json`  
  Current-limited inverter-based DG. Checks grid-following VSC current-source
  contribution.

- `sc_hand_07_converter_ac_grid_forming.json`  
  AC-grid-forming inverter-based DG. Checks voltage-source-behind-impedance
  contribution.

### Practical Stress Cases

These are not meant to be exact page-by-page hand calculations. They are
commissioning-style cases with embedded `practical_checks`: expected current
windows, source-contribution floors, branch-current evidence, max/min
relationships, and cross-fault ratios.

- `sc_practical_01_industrial_plant_110_20kv.json`  
  110/20 kV industrial plant with a rich two-winding transformer, synchronous
  CHP generator, large asynchronous motor, PV static generator, and max/min
  utility source data. Stresses canonical transformer and motor projection,
  source contribution attribution, and branch current output.

- `sc_practical_02_urban_meshed_10kv_feeder.json`  
  10 kV urban ring feeder with a closed tie, load motor fraction, and rooftop
  inverter DG. Stresses meshed transfer impedances, downstream attenuation, and
  multi-path branch current distribution.

- `sc_practical_03_hybrid_acdc_inverter_microgrid.json`  
  Hybrid AC/DC microgrid with one grid-following VSC and one AC-grid-forming
  VSC. Stresses the distinction between DC-side `grid_forming` current-limited
  behavior and AC-side `ac_grid_forming` voltage-source behavior.

- `sc_practical_04_ground_fault_low_z0_transformer_feeder.json`  
  Grounded 110/20 kV feeder with explicit grid, transformer, and cable
  zero-sequence data. Stresses SLG calculations where low `Z0` can make the
  single-phase ground current exceed the three-phase current.

## Formula Reminders

- Base current: `I_base[kA] = S_base[MVA] / (sqrt(3) * U_base[kV])`.
- Three-phase fault: `I_k'' = c / |Z1 + Zf| * I_base`.
- Single-line-ground fault: `I_k1'' = c / |(Z1 + Z2 + Z0 + 3Zf) / 3| * I_base`.
- External-grid short-circuit power: with explicit `s_sc_*`, `I_k''` should
  match `S_sc / (sqrt(3) * U_n)`.

## Validation Strategy

- Hand cases use `expected_short_circuit.checks` and compare selected numerical
  results directly.
- Practical cases use `expected_short_circuit.practical_checks` and validate
  engineering invariants. This is intentional: practical systems combine rich
  projection, sequence networks, source attribution, and current-limited devices,
  so robust ranges and relationships are more useful than brittle exact rows.
