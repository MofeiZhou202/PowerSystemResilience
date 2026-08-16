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
  --compiler texlive
```

Rendered PDFs and LaTeX intermediate files remain build products and are not
tracked.
