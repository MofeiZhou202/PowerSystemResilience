# Phase-Node Graph-Reduced Hybrid OPF Paper

This directory contains an IEEE Transactions-style initial manuscript draft.

## Status

- The compact phase-node hybrid AC/DC OPF formulation is drafted.
- Feasible-set, optimal-value, KKT, second-order, dual-recovery, and
  model/KKT-Schur commutativity results are stated and proved at manuscript
  draft level.
- Monolithic hybrid OPF numerical tables are intentionally marked `TBG`
  (to be generated).
- A reproducible AC-feeder phase-node reduction certificate is implemented for
  the official IEEE 13/34/123/8500 OpenDSS cases. It is kept separate from the
  still-TBG monolithic hybrid OPF ablation.
- The paper is not submission-ready until the monolithic solver, deterministic
  IEEE feeder hybridization tool, independent KKT checker, and all benchmark
  results are implemented and reviewed.

## Build

From the repository root:

```sh
python3 /Users/tianyangzhao/.codex/plugins/cache/openai-bundled/latex/0.2.4/scripts/compile_latex.py \
  "$PWD/docs/papers/phase_graph_reduced_hybrid_opf/main.tex" \
  --compiler texlive
```

The generated numerical pipeline should replace `results_tables.tex` while
preserving its labels and column semantics.

## Reduction Certificate

Configure the DSS C-API path, build the benchmark, and regenerate both the CSV
evidence and the LaTeX table:

```sh
cmake -S . -B build/macos-release \
  -DHACDCPF_ENABLE_OPENDSS=ON \
  -DHACDCPF_DSS_CAPI_ROOT="$PWD/.venv/lib/python3.9/site-packages/dss_python_backend"
cmake --build build/macos-release --target phase_graph_reduction_benchmark -j4
./build/macos-release/phase_graph_reduction_benchmark
```

Outputs:

- `output/benchmarks/phase_graph_reduction_case_audit.csv`
- `docs/papers/phase_graph_reduced_hybrid_opf/reduction_certificate_table.tex`

The certificate validates sparse phase-node Schur elimination, recovery, and
linear-solve timing on the original AC feeders. It does not contain DC
hybridization, VSC decisions, an OPF objective, or nonlinear KKT timing.
