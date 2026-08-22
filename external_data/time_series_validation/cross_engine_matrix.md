# Time-series cross-engine matrix

## Unit commitment

| Oracle | Objective | Commitment | Max balance residual (MW) |
|---|---:|---|---:|
| HySim/HiGHS | 9233.3500000000 | `[[1, 1, 1, 1], [0, 1, 1, 1]]` | n/a |
| SciPy/HiGHS rebuild | 9233.3500000000 | `[[1, 1, 1, 1], [0, 1, 1, 1]]` | 0.000e+00 |
| 256-sequence enumeration | 9233.3500000000 | `[[1, 1, 1, 1], [0, 1, 1, 1]]` | 7.105e-15 |

Objective relative spread: `0.000e+00`; pass: `True`.

## Six-step AC snapshot comparison

| Step | Scale | HySim Vm (pu) | OpenDSS Vm (pu) | |dVm| | GridLAB-D pass |
|---:|---:|---:|---:|---:|---|
| 0 | 0.65 | 0.998122169 | 0.998122169 | 4.188e-10 | True |
| 1 | 0.80 | 0.997687158 | 0.997687158 | 4.549e-10 | True |
| 2 | 1.00 | 0.997106165 | 0.997106165 | 4.112e-10 | True |
| 3 | 1.15 | 0.996669684 | 0.996669684 | 1.404e-10 | True |
| 4 | 1.30 | 0.996232570 | 0.996232570 | 6.587e-10 | True |
| 5 | 0.90 | 0.997396802 | 0.997396801 | 4.490e-10 | True |

OpenDSS maxima: `{"vm_pu": 6.587109746547526e-10, "va_deg": 4.505303607960087e-08, "p_mw": 6.642887351304694e-07, "q_mvar": 2.588633791766881e-07}`.
GridLAB-D maxima: `{"vm_pu": 5.847480752940726e-07, "va_deg": 4.165266190708783e-07, "p_mw": 4.814738312930444e-06, "q_mvar": 4.99294772415837e-07}`.

Overall pass: **True**.

The external engines validate the per-step AC network snapshots only; they are not used as UC or lifecycle oracles.
