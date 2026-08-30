# Transition-Aware Reliability Paper

This directory contains the IEEEtran manuscript:

> Transition Aware Reliability Assessment of Hybrid AC/DC Distribution
> Systems with Inverter Based Resources

## Current Scientific Story

The manuscript links protection--IBR transient simulation to conventional
post-fault distribution restoration. In the manuscript terminology, a
trajectory class is the restoration-entry state induced, for a fixed initiating
contingency, by the complete DAE trajectory's unavailable-VSC set.

1. The initiating contingency and operating state define the pre-fault system.
2. A balanced phasor-domain mass-matrix DAE determines whether the initiating
   fault causes additional VSC outages before restoration.
3. For a fixed initiating contingency, operating states with the same complete
   unavailable-VSC set form one trajectory class and share one
   restoration-entry state.
4. The trajectory partition is built progressively over the retained VSC
   status vector. Its number of levels equals the number of VSCs, rather than a
   prescribed three-level hierarchy; the final classes are finite, mutually
   exclusive, and complete over resolved states.
5. Loads and generation remain state-specific. The existing hybrid AC/DC
   restoration MILP, including its `sP/sQ` load-shedding variables, is solved
   for every operating state.
6. Restoration-entry states reduce repeated DAE calculations; they do not
   replace or reduce the number of restoration MILP calculations.
7. N-1, N-2, and N-3 entries are explicit simultaneous joint events. Marginal
   component failure rates are not multiplied to fabricate joint frequencies.

The corresponding cost model is

```text
T_direct = N * (t_DAE + t_MILP)
T_class  = T_library + N * (t_identification + t_MILP)
```

The Introduction and Case Studies follow the section structure and writing
pattern of the author's `TSG-01621-2022` paper:

```text
Introduction
  Motivation
  Literature Review
  Contributions

Case Studies
  Case Description
  Case 1: Necessity of Transient Modeling
  Case 2: Accuracy of Restoration-Entry State Identification
  Case 3: Computational Performance
  Case 4: Large-System Validation and Limitations
```

## Numerical Evidence

### Modified IEEE 33-Bus Hybrid AC/DC System

- 33 AC buses, 37 AC branches, five DC buses, five DC branches, and three
  VSCs.
- 8 N-1, 28 N-2, and 56 N-3 initiating events, evaluated at six operating
  states: 552/552 conditions resolved.
- Static and transition-aware EENS: `0.0905561618570` and
  `0.260830925306 MWh/yr`.
- The difference is 65.2817% of the transition-aware EENS; 366 higher-order
  conditions change restoration entry or consequence.
- 92 restoration-entry states; state ratio `0.166667`.
- Leave-one-operating-state-out entry-state accuracy: 100%; held-out
  restoration EENS error: zero.
- Exact grouping arithmetic error: `2.22e-16 MWh/yr`.
- Maximum post-event algebraic residual: `1.07e-13`.
- Two timing projections give 5.76x and 5.99x end-to-end speedup for 100000
  samples and a 99.448% reduction in DAE solves.
- Net saving is demonstrated, but the pre-registered 20x target fails.

### Modified IEEE 123-Bus Hybrid AC/DC System

- Balanced positive-sequence projection with 132 AC buses, 134 AC branches,
  seven DC buses, seven DC branches, and four VSCs.
- 10 N-1, 45 N-2, and 120 N-3 initiating events, evaluated at six operating
  states.
- 892/1050 conditions resolve. The remaining 158 fail in the DAE/Newton or
  event-time algebraic network solution.
- All 892 conditions that enter restoration complete the load-shedding MILP;
  there are no restoration-specific failures.
- Unresolved frequency: `0.0319160406627/yr`.
- The resolved-submeasure EENS values, `0.0578626786707` and
  `0.235781116183 MWh/yr`, are not feeder-wide indices.
- Leave-one-out uncovered frequency: `0.000761065845/yr`.
- The 8.81x--8.91x timing projections are diagnostic and are not admitted as
  net computational savings because the unresolved frequency is nonzero.

Both formal JSON reports pass the independent raw-record oracle. The case123
result is a numerical-closure diagnostic, not positive large-feeder
validation.

## Evidence Sources

- `code/`: source-backed manuscript audit bundle containing the traceability
  map, per-file hashes, repository provenance, implementation, tests, input
  data, and formal numerical reports.
- `build/macos-release/nk_case33_entry_class_final.json`
- `build/macos-release/nk_case123_entry_class_final.json`
- `tools/nk_acdc_reliability_study.cpp`
- `tools/validate_nk_acdc_reliability.py`
- `src/network_reconfiguration/topology_reconfiguration.cpp`
- `tests/test_reconfig_options.cpp`

The restoration model already supported active and reactive load shedding.
The relevant implementation is the `sP/sQ` bounds, objective terms, and nodal
power-balance constraints in
`src/network_reconfiguration/topology_reconfiguration.cpp`. The recent repair
corrected AC tie pruning under split AC/DC tree constraints; it did not add a
new load-shedding feature.

## Code Audit Bundle

The `code/snapshot/` directory preserves each audited file's repository-relative
path so that manuscript claims can be reviewed against the same module
boundaries used by HySim. Start with `code/TRACEABILITY.md`, which maps the
paper's equations, algorithms, cases, and admitted limitations to concrete
symbols and evidence. `code/FILESET.tsv` declares the extraction boundary,
`code/MANIFEST.tsv` records the SHA-256 hash of every copied file, and
`code/PROVENANCE.txt` records the HySim and MIPSolvers revisions and worktree
states from which the snapshot was made.

From `code/`, run `python3 audit_bundle.py verify` to detect a missing file,
source-to-snapshot drift, or a hash mismatch. Run `sync` only when deliberately
refreshing the audit snapshot after a source or evidence change. The bundle is
not a standalone HySim fork or build tree: production compilation and common
graph, power-flow, and solver infrastructure remain at the repository root and
in `../MIPSolvers`.

## Manuscript Files

- `main.tex`: title, abstract, Introduction, conclusion, and appendices.
- `sections_ii_iv.tex`: Section II and includes for Sections III--IV.
- `section_iii.tex`: reliability assessment formulation.
- `section_iv.tex`: controller-to-trajectory and equivalent-state method.
- `case_studies.tex`: four current numerical cases.
- `references.bib`: bibliography.
- `figures/generate_research_object.py`: editable Fig. 1 source.
- `figures/generate_partition_figure.m`: canonical MATLAB R2025b source for
  the Section III high-order event and trajectory-partition figure.
- `figures/generate_restoration_entry_state.py`: historical Python source for
  the former Section III trajectory-class figure; it is not the canonical
  renderer.
- `figures/generate_case_study_figures.m`: canonical MATLAB R2025b source for
  all three Section V figures: the case33 transient mechanism, case33 method
  validation, and case123 numerical-closure diagnostic. It reads the audited
  CSV files under `figures/source_data/` and the formal case33 event report and
  exports editable PDF/SVG plus 600-dpi PNG/TIFF files.
- The former Python figure scripts are retained only as historical data/plot
  preparation material; they are not the canonical renderer for Section V.

The main manuscript abstract, Introduction, Sections II--V, and conclusion use
the current N-1/N-2/N-3 restoration-entry-state story. The auxiliary highlights
and response letter remain historical working material and are not part of the
compiled manuscript.

## Reproduce Numerical Evidence

From the repository root:

```bash
cmake --build --preset macos-release --target nk_acdc_reliability_study

build/macos-release/nk_acdc_reliability_study \
  --case case33_acdc --max-order 3 --dt 0.005 --audit-only \
  --output build/macos-release/nk_case33_entry_class_final.json

build/macos-release/nk_acdc_reliability_study \
  --case case123_acdc --max-order 3 --dt 0.005 --audit-only \
  --output build/macos-release/nk_case123_entry_class_final.json

python3 tools/validate_nk_acdc_reliability.py \
  build/macos-release/nk_acdc_reliability_study \
  build/macos-release/nk_case33_entry_class_final.json

python3 tools/validate_nk_acdc_reliability.py \
  build/macos-release/nk_acdc_reliability_study \
  build/macos-release/nk_case123_entry_class_final.json
```

## Compile

Regenerate the Section V figures with MATLAB before compiling:

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch \
  "cd('docs/latex/paper/Progressive Class-Conditioned Frequency–Duration/figures'); \
   generate_case_study_figures"
```

Regenerate the Section III partition figure with MATLAB:

```bash
/Applications/MATLAB_R2025b.app/bin/matlab -batch \
  "cd('docs/latex/paper/Progressive Class-Conditioned Frequency–Duration/figures'); \
   generate_partition_figure"
```

```bash
python3 /Users/tianyangzhao/.codex/plugins/cache/openai-bundled/latex/0.2.6/scripts/compile_latex.py \
  'docs/latex/paper/Progressive Class-Conditioned Frequency–Duration/main.tex' \
  --compiler texlive --engine xelatex \
  --output-directory /tmp/hysim_transition_paper --json
```

The acceptance checks are zero undefined citations/references, zero overfull
boxes, and visual inspection of the rendered pages containing the rewritten
sections.
