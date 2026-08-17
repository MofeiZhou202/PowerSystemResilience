# Structure-Preserving Semismooth Hybrid AC/DC Power Flow Paper

This tracked IEEE Transactions-style manuscript contains the initial draft of:

> Structure-Preserving Semismooth Newton Method for Large-Scale Hybrid AC/DC
> Power Flow With Converter Limit-Induced Mode Switching

## Included Sections

- Introduction and research gap.
- Balanced positive-sequence hybrid AC/DC and explicit six-state VSC model.
- Exact and smoothed NCP equations and semismooth Newton algorithm.
- Exact local VSC-block Schur elimination and complexity model.
- BD-regularity boundary and conditional local convergence results.
- Preliminary production results from registered tests and the fixed Release
  Schur benchmark.

## Evidence Boundary

The draft reports only reproduced CPU/KLU results. It does not claim GPU
acceleration, large-scale simultaneous binding GFM converters, a complete
active-mode enumeration oracle, or an OPF KKT system containing the GFM
priority NCP. OpenDSS validates the nonbinding Thevenin root independently and
the binding circuit under a frozen HySim internal voltage; the latter is not an
independent mode-selection oracle.

## Reproduce

```sh
cmake --build build/macos-release --target test_vsc_limit_ncp \
  test_transient_dynamics test_three_phase_hybrid_opf \
  vsc_schur_benchmark -j4
./build/macos-release/tests/test_vsc_limit_ncp
./build/macos-release/tests/test_transient_dynamics \
  "[dynamics][gfm][initialization]"
./build/macos-release/tests/test_three_phase_hybrid_opf
./build/macos-release/vsc_schur_benchmark \
  --case all --warmups 2 --repeats 5
```

Compile the manuscript with the repository's available TeX Live installation:

```sh
python3 /Users/tianyangzhao/.codex/plugins/cache/openai-bundled/latex/0.2.4/scripts/compile_latex.py \
  "$PWD/docs/latex/paper/structure_preserving_semismooth_acdc_pf/main.tex" \
  --compiler texlive \
  --output-directory \
  "$PWD/docs/latex/paper/structure_preserving_semismooth_acdc_pf"
```

The compiled manuscript is retained as `main.pdf` in the paper directory.
LaTeX auxiliary files remain ignored build products.

## Reproduce Figures

The quantitative figure data are versioned separately from the plotting code
in `figures/figure_data.json`. Regenerate the vector figures with:

```sh
MPLCONFIGDIR=/tmp/hysim-matplotlib \
python3 figures/generate_figures.py
```

The script produces `problem_method_overview.pdf`, `priority_geometry.pdf`,
and `benchmark_results.pdf` beside the data file. The benchmark panels report
the same fixed-protocol values as `tools/vsc_schur_benchmark.cpp`,
`docs/development_status.md`, and the manuscript tables.
