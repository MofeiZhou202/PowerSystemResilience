# Vendored HiGHS HFactor

This directory contains a vendored copy of the HiGHS sparse-LU
basis factorization code (`HFactor`) plus the minimal transitive headers it
depends on. It builds as a standalone static library and is wrapped by
`mipsolvers::engine::HFactorBackend`. Local safety and integration changes are
kept small and visible in version control; this directory is not a pristine
upstream mirror.

## Source vintage
Vendored from the bundled HiGHS at `HiGHS/highs/` (HiGHS git
`dcc25308d`, 1.14.0). See `HConfig.h` in this directory for version stamp.

## Layout
Mirrors the HiGHS `highs/` source tree so `#include "util/HFactor.h"`
style includes work without modification:

```
highs_factor/
  HConfig.h             # checked-in standalone configuration
  extern/pdqsort/       # third-party sort used by HighsHash
  io/                   # HighsIO (logging)
  lp_data/              # HConst, HStruct, HighsAnalysis, HighsCallback*,
                        # HighsLp, HighsOptions, HighsStatus
  simplex/              # SimplexConst, SimplexStruct
  util/                 # HFactor + transitive: HVector, HSet, HighsSparseMatrix,
                        # HighsSort, HighsUtils, HighsTimer, HighsCDouble,
                        # HighsRandom, HighsHash, HighsMatrixUtils, stringutil,
                        # HighsSparseVectorSum, FactorTimer
```

## Current contract
- Builds as static library `mipsolvers_hfactor`.
- `HFactorBackend` exposes factorization, FTRAN, BTRAN, rank repair, and
  captured FT-update operations to the experimental native dual-simplex
  kernel.
- It is an experimental backend, not the production default and not evidence
  of parity with the complete HiGHS simplex implementation.
- Header-search root for HFactor sources is *this* directory, so they
  see `HConfig.h`, `util/...`, `io/...`, etc. as they would inside HiGHS.
