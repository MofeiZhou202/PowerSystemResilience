# Power-Electronic Dynamics Cross-Validation

Generated from the live local checkout of PowerSimulationsDynamics.jl at
`/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl` with Julia 1.10.10.
HACDCPF uses `MassMatrixDae`; PSD uses `ResidualModel` with IDA. The disturbance
in this promoted gate is an active-power reference step from 0.5 to 0.7 pu at
1.0 s. The full 34-case manifest passed 1,069,529 assertions.

## Power-Electronic Results

| Control | Metric | RMS error | Max error | Interpretation |
|---|---:|---:|---:|---|
| VSM GFM | angle, relative rad | 0.022170 | 0.029745 | matched forming-angle response |
| VSM GFM | frequency, relative pu | 0.000072 | 0.000502 | strong agreement |
| VSM GFM | P, device pu | 0.011333 | 0.033907 | active-power reference reaches the Norton port |
| VOC GFM | angle, relative rad | 0.010788 | 0.014794 | good agreement |
| VOC GFM | internal voltage, pu | 0.012087 | 0.012268 | compact virtual-impedance model |
| VOC GFM | frequency, relative pu | 0.000104 | 0.000467 | strong agreement |
| Droop GFM | angle, relative rad | 0.021510 | 0.037001 | matched droop response |
| ReducedOrderPLL GFL | filtered P state, pu | 0.003472 | 0.027373 | matched controller state |
| ReducedOrderPLL GFL | PLL angle, relative rad | 0.010636 | 0.027029 | good agreement |
| ReducedOrderPLL GFL | PLL frequency, relative pu | 0.000500 | 0.007101 | event-localized deviation |
| ReducedOrderPLL GFL | terminal P, device pu | 0.007562 | 0.065743 | full outer/inner/LCL chain |
| ReducedOrderPLL GFL | terminal Q, device pu | 0.007542 | 0.043768 | full outer/inner/LCL chain |
| KauraPLL GFL | filtered P state, pu | 0.004606 | 0.036534 | matched controller state |
| KauraPLL GFL | PLL angle, relative rad | 0.010729 | 0.026628 | good agreement |
| KauraPLL GFL | PLL frequency, relative pu | 0.000316 | 0.002479 | strong agreement |
| KauraPLL GFL | terminal P, device pu | 0.008446 | 0.082164 | full outer/inner/LCL chain |
| KauraPLL GFL | terminal Q, device pu | 0.009774 | 0.067528 | full outer/inner/LCL chain |

Terminal-voltage magnitude RMS errors are below 0.00615 pu for all five controls.
GFL bus-angle relative RMS errors are below 0.0107 rad. The aggregate CSV contains
all 163 manifest metrics, including synchronous-machine and controller cases.

## Bugs Found And Corrected

1. The GFL fixture reused an unrelated hybrid DC network. It now matches PSD's
   two-bus OMIB topology and represents PSD's fixed DC source as an internal
   constant DC link. Both GFL power flows now converge; trim residuals are
   6.94e-8 for ReducedOrderPLL and 5.21e-8 for KauraPLL at the unchanged 1e-7
   tolerance.
2. Native GFM/GFL terminal power summed three phase-domain pu powers without
   converting back to the three-phase device base. Inverter feedback and
   telemetry now use the phase average, removing the exact 3x power error.
3. PSD terminal P/Q is returned on the 100 MVA system base while inverter
   controller states use the 2.75 MVA device base. The exporter now exposes
   explicit `p_device_pu` and `q_device_pu` signals.
4. An absent `is_not_reference` profile flag incorrectly locked every native
   GFM angle to the synchronous reference frame. The default now remains
   unlocked, and a regression requires VSM and droop power-reference steps to
   change terminal active power.

## Fidelity Boundary

PSD Tests 24 and 51 include ActivePowerPI, ReactivePowerPI, CurrentModeControl,
and a detailed LCL filter. HACDCPF now provides that same chain as an opt-in
**full-fidelity GFL mode** (outer PI + inner current PI + differential 6-state
LCL filter, wired P→Iq_ref / Q→Id_ref exactly as PSD). With it enabled, the
P/Q step-peak deviations collapse from 0.28/0.41 pu to **0.066/0.044 pu**
(test24) and **0.082/0.068 pu** (test51) vs PSD ResidualModel/IDA; the legacy
compact first-order REGC/REEC subset remains the default for backward
compatibility. The full-fidelity chain is stiff (ωb/lf ≈ 3.5e4 /s) and is
integrated with the A-stable MassMatrixDae solver; manifest max tolerances for
`p_device_pu`/`q_device_pu` are tightened from 0.30/0.45 to 0.15 accordingly.

The promoted gate currently covers the matched active-power reference
disturbance. Reactive-reference, source-voltage, impedance-switching, temporary
fault, and sequential-disturbance cases remain separate validation work and
must not be described as completed by this report.
