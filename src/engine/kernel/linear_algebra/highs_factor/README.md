# Vendored HiGHS HFactor (Phase 1 of U.7.118)

This directory contains a **read-only vendored copy** of the HiGHS sparse-LU
basis factorization code (`HFactor`) plus the minimal transitive headers it
depends on. The goal is to build `HFactor` as a standalone static library
without modifying any HiGHS source file, so future HiGHS upstream merges
remain trivial (re-`cp` the files).

## Source vintage
Vendored from the bundled HiGHS at `HiGHS/highs/` (HiGHS git
`dcc25308d`, 1.14.0). See `HConfig.h` in this directory for version stamp.

## Layout
Mirrors the HiGHS `highs/` source tree so `#include "util/HFactor.h"`
style includes work without modification:

```
highs_factor/
  HConfig.h             # vendored stub of build-generated HConfig
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

## Phase 1 contract (this turn)
- No integration with our LP kernel.
- No source modifications.
- Builds as static library `hacdcpf_hfactor`.
- Header-search root for HFactor sources is *this* directory, so they
  see `HConfig.h`, `util/...`, `io/...`, etc. as they would inside HiGHS.

## Future phases
- Phase 2: thin wrapper `HFactorBackend` mirroring `SparseLUFactor` API.
- Phase 3: integrate as `FactorBackendKind::HFactor` in dual_simplex.cpp.
- Phase 4: validate + bench + flip default ON for `B&C[Simplex]/FA`.

See `/memories/repo/u7-118-hfactor-port-plan.md` for full plan.
