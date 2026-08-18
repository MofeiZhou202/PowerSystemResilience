# VSC Current-Limit NCP Power-Flow Contract

This living contract defines the balanced positive-sequence VSC current-limit
model used by the production unified Newton power-flow path. Source and
registered tests remain authoritative.

## Scope

The model is opt-in per in-service `VSCConverter` through
`enable_limit_ncp=true` and `i_ac_max_pu>0`. It supports `PQ_MODE`, `VDC_Q`,
and `AC_GRID_FORMING` in grid-connected or slackless AC islands. Every admitted converter owns the fixed
per-unit local state

\[
z_c=(P_{ac},Q_{ac},P_{dc},E_r,E_i,\lambda)_c.
\]

The powers enter AC-P, AC-Q, and DC-P balances directly. Results use the
stable converter `.index`, never vector position. `PQ_MODE` and `VDC_Q` use
identity rows `E_r=E_i=0`; GFM uses those slots for internal voltage behind
virtual impedance. This is a steady/quasi-steady post-limit operating point,
not the fast inner current loop, PWM, or EMT switching transient.

## Power-port equations

For `PQ_MODE`, `P_ref=P_set/S_base` and `Q_ref=Q_set/S_base`. For `VDC_Q`,

\[
P_{raw}=k_{vdc}(V_{dc}^2-(V_{dc}^{set})^2),\qquad
P_{ref}=\operatorname{clip}(P_{raw},P_{min},P_{max}).
\]

The `droop_p_min_mw`/`droop_p_max_mw` pair is used whenever either entry is
nonzero; only an all-zero droop pair falls back to `pmin_mw/pmax_mw`
(`effective_lower_mw`/`effective_upper_mw`,
`src/power_flow/vsc_limit_ncp.cpp:90-102`). The clip is applied only when the
effective upper bound exceeds the lower one; an unordered or zero-width
effective pair disables clamping and passes `P_ref=P_raw` through unclamped
(`vsc_limit_command`, `src/power_flow/vsc_limit_ncp.cpp:205-210`). The
saturated derivative is zero outside the open interval and `2*k_vdc*Vdc`
inside it.

### Droop sizing for authored benchmark data

For the production squared-voltage law, a station expected to regulate
`P_rated_pu` at a voltage-magnitude droop band `Delta V` uses

\[
k_{vdc}=\frac{P_{rated,pu}}{1-(1-\Delta V)^2}.
\]

`Defaults::kVdcDroopRatedVoltageDeviation=0.05` is the single source for the
standard authored-data band. In `case2000_acdc`, four fixed-P stations each
draw `0.5 pu` and four Vdc-Q stations share that transfer, hence each regulator
uses `k_vdc=0.5/[1-(1-0.05)^2]`. Converter and line losses may move the solved
voltage slightly beyond the nominal 5% point, while the registered case still
requires every DC voltage to remain inside the production `[0.9,1.1] pu`
qualification interval. This sizing is case-data construction, not an
iteration-dependent solver override.

The AC current disk is

\[
g(P,Q,V_m)=(V_m I_{max})^2-P^2-Q^2\ge0.
\]

Magnitude priority uses

\[
(1+\lambda)P-P_{ref}=0,\quad
(1+\lambda)Q-Q_{ref}=0,\quad
\Phi_{FB}(\lambda,g)=0.
\]

P-first and Q-first use median residuals for the remaining current-circle
radius. A finite generalized-Jacobian branch is selected at a zero-radius tie.
All priorities share

\[
P_{dc}+P_{ac}+P_{loss}(P_{ac},V_{dc})=0,
\]

with analytic loss derivatives from the selected production `LossModelType`.

The target complementarity function is the exact Fischer-Burmeister map

\[
\Phi_{FB}(a,b)=\sqrt{a^2+b^2}-a-b.
\]

In particular, `Phi_FB(0,0)=0`; no hidden epsilon perturbs the biactive
intersection. At the origin the implementation selects the symmetric Clarke
generalized derivative
`(1/sqrt(2)-1, 1/sqrt(2)-1)`. This is the default semismooth equation used for
the final certificate.

## Grid-forming equation family

For terminal voltage `V=Vm*exp(j*Va)`, internal voltage `E=Er+j*Ei`, and
virtual impedance `Zv=Rv+j*Xv`,

\[
I=(E-V)/Z_v,\qquad S_{ac}=VI^*=P_{ac}+jQ_{ac}.
\]

`gfm_internal_voltage_set_pu`, `gfm_internal_angle_set_deg`,
`gfm_virtual_r_pu`, and `gfm_virtual_x_pu` are shared by PF, OPF adapters, and
transient initialization. Zero authored values retain the legacy
`v_ac_set_pu`, `v_ac_angle_set_deg`, `r_conv_ac_pu`, and `x_sc_pu` fallbacks.
All consumers call `model::resolve_gfm_norton_parameters`; the complete
cross-module precedence, unit, identity, and result rules are defined in
`docs/model_data_semantics_contract.md`.

Magnitude limiting uses the two-dimensional fallback

\[
(1+\lambda)(E-V)=E^0-V,\qquad
0\le\lambda\perp I_{max}^2-|I|^2\ge0.
\]

P-first and Q-first operate in the terminal-voltage synchronous dq frame.
Generalized derivatives with respect to all six local states, terminal `Vm`,
terminal `Va`, and `Vdc` are analytic.

## Smooth continuation

When `RobustNonlinearOptions::enable_smooth_ncp` is enabled, VSC blocks use the
Kanzow smoothing radius

\[
\Phi_\mu(a,b)=\sqrt{a^2+b^2+2\mu^2}-a-b
\]

for magnitude priority and CHKS-smoothed min/max operators for P-first and
Q-first. This option applies to VSC limit blocks independently of the legacy
`enable_semi_smooth_newton` PV/PQ switch. The same `mu` schedule is shared when
both generator and converter complementarity equations are present.

`mu` is reduced from `ncp_mu0` to `ncp_mu_min`; after every reduction the
residual and Jacobian are reassembled at the unchanged state. The continuation
changes values only, so the six-state VSC layout and sparse coordinates remain
fixed. Public `VSCTransfer`/`VSCLimitState` certificates are re-evaluated with
`mu=0`, preventing a smooth surrogate residual from being reported as exact
complementarity.

## Sparse and structural contract

The network residual is `spec-calc`, while the stored matrix follows the
existing `d(calc-spec)/dx` convention. Local equations store `dF/dx` and use
right-hand side `-F`. Every enabled converter adds exactly six rows and six
columns. Switching activity changes values, never coordinates, so one symbolic
analysis is reused for numeric refactorizations.

Assembly cost remains

\[
O(nnz(Y_{ac})+nnz(G_{dc})+n_{VSC}),
\]

while factorization cost depends on fill. No GPU assembly or GPU sparse
factorization is implemented by this contract.

### Exact local-block Schur elimination

For `m` admitted VSC blocks, the selected generalized Newton matrix is
partitioned into network variables `x` and converter-local variables `z`:

\[
J=\begin{bmatrix}A&B\\C&D\end{bmatrix},\qquad
D=\operatorname{blkdiag}(D_1,\ldots,D_m),\quad D_i\in\mathbb R^{6\times6}.
\]

`VSCLocalSchurSolver` computes

\[
S=A-BD^{-1}C,\qquad
\widehat r_x=r_x-BD^{-1}r_z,
\]

solves `S dx = rhat_x`, and recovers

\[
dz=D^{-1}(r_z-Cdx).
\]

No dense inverse is formed. Each 6-by-6 factor solves the four right-hand
sides `[C_i,r_i]`. Every block couples only its terminal `Va/Vm/Vdc` columns
and its AC-P/AC-Q/DC-P rows, so one block contributes at most nine Schur update
coordinates. The reduced sparse pattern is fixed and symbolically analyzed
once.

With local size `s=6` and at most `c=3` terminal network coordinates, local
work is

\[
O\!\left(ms^3+ms^2(c+1)\right)=O(m),
\]

and the reduced dimension is exactly `n_x` instead of `n_x+6m`. The complete
fixed pattern has 36 local, 18 local-to-network, and 3 network-to-local entries
per converter. The reduced update adds at most 9 entries, so the input sparse
structure loses at least `48m` nonzeros. Sparse-LU fill and work remain graph-
and-ordering-dependent and are measured rather than assigned an `O(n^1.5)`
claim.

The production path requires `network_nvar >=
Defaults::kVSCSchurMinNetworkDimension` (currently 1000). Setting the threshold
to zero is a research ablation. The enable flag, admission threshold, local
`rcond`, and backward-error tolerances are configurable through both C++
`PowerFlowOptions` and the production PF HTTP `robust_nonlinear` object; the
effective values are returned by the GUI backend. Machine epsilon itself is a
read-only representation constant. Invalid dimensionless certificate
tolerances restore their named defaults at every production boundary.

This guard is evidence-based: forced Schur on
case300 reduces dimension `640 -> 604` and structural nonzeros `4820 -> 4502`
but increases the current fixed-protocol median linear/wall time by
`31.36%/32.94%` because local certificate overhead dominates a small KLU
factorization.
ACTIVSg2000 reduces `4054 -> 4006` and `29806 -> 29382`, with median
linear/wall reductions of `13.75%/10.55%`; it is admitted by the production
default. Relative to the preceding `13.85%/11.99%` result, the current
large-case speedups are lower by 0.10/1.44 percentage points; this small
regression does not change sign or trigger the fixed 50%-mismatch
re-derivation threshold.

Every candidate checks each selected `D_i` reciprocal condition estimate,
the reduced normwise backward error, and the reconstructed complete-system
backward error. `Defaults::kVSCSchurLocalRcondTol` is derived from the global
`NumericalConstants::kSqrtMachineEpsilon`; the complete-step default is
`Defaults::kVSCSchurBackwardErrorTol`. A rejected local block, reduced solve,
or reconstructed step falls back to the ordinary full sparse LU. Public
profiling separates attempts, accepted steps, rejection causes, dimensions,
structural nonzeros, accepted-block minimum `rcond`, accepted-step maximum
backward error, and semismooth residual-rate quotients.

### BD-regularity and local rate

For one selected generalized-Jacobian element, nonsingularity of every `D_i`
and of `S` implies nonsingularity of `J` by block Gaussian elimination. This is
the executable sufficient condition checked by the Schur path. It is not by
itself BD-regularity: BD-regularity requires every relevant element of the
B-subdifferential in a solution neighborhood to be nonsingular. Under that
stronger condition and semismooth residuals, semismooth Newton is locally
superlinear; under strong semismoothness the standard local rate is quadratic.

The runtime reports `||F_{k+1}||/||F_k||` and
`||F_{k+1}||/||F_k||^2` as measured local-rate evidence. It does not convert a
finite sample or one selected Jacobian certificate into a proof over the full
Clarke set. Degenerate P/Q-priority intersections can make a selected `D_i`
ill-conditioned; those iterations are expected to use the full-LU fallback.

Admission fails closed when the current limit is non-positive, the control
mode is unsupported, a GFM virtual impedance is zero, required AC-P/AC-Q/DC-P
rows are absent, or structural scanning finds an empty row/column. A
grid-connected GFM terminal remains a solved PQ bus. In an AC island without a
terminal SLACK/external grid, all bus `Vm/Va` states and all P/Q balances remain
in the Newton system. The authored fixed internal phasor `E*exp(j*delta*)`
inside the Norton equation breaks the global rotational null mode; no terminal
angle is fixed and no synthetic SLACK bus is created. Multiple fixed Norton
sources may share an island: their internal phasors and virtual impedances
determine circulating power and sharing, so they are not multiple rigid
terminal references. A GFM terminal already typed as `SLACK` is rejected.

`IslandInfo::has_ac_slack` retains its terminal-reference meaning.
`has_ac_angle_reference` additionally recognizes a fixed GFM internal phasor,
and `gfm_reference_vsc_indices` carries stable authored converter IDs.
Adaptive island extraction preserves those IDs and bus types. Strict hybrid
coordination separately requires a physical DC voltage/power-balancing source;
a bare `DC_V` bus label is not sufficient.

## Cross-module semantics

`PowerFlowResult::vsc_limit_states` and `VSCTransfer` expose the stable ID,
power/current data, priority, activation and droop status, complementarity
residual, `gfm_norton_model`, internal-voltage real/imaginary components, and
terminal-current real/imaginary components. A GFM solve sets
`vsc_gfm_norton_modelled` and `vsc_gfm_priority_limit_enforced` in
`ConverterModelScope`. Slackless solves additionally set
`vsc_gfm_island_reference_modelled` and return stable
`gfm_island_reference_vsc_indices` diagnostics.

Balanced OPF treats GFM converters as free P/Q ports with its selected
engineering inequalities. Internal voltage, virtual impedance, and priority
NCP are not OPF KKT constraints. Time-series and GUI `post_pf` replay preserves
the authored GFM mode and returns the production PF certificate; that replay
must not be described as endogenous OPF GFM-NCP optimization.

Transient initialization consumes the same stable-ID PF certificate and checks
the internal-voltage, current, and power Norton seed before dynamic network
re-solving. A later full-dynamic trim may move to another equilibrium because
dynamic sources and loads need not reproduce the PF slack dispatch exactly.

## Verification thresholds

`tests/test_vsc_limit_ncp.cpp` and `tests/test_transient_dynamics.cpp` enforce:

- local/oracle and nonbinding root errors at most `1e-9`;
- complementarity, current-circle, and energy residuals at most `1e-9 pu`;
- analytic GFM generalized-Jacobian finite-difference error at most `5e-7`;
- exact FB origin residual equal to zero and biactive full-block residual at
  most `2e-15`;
- smooth-FB and smoothed priority Jacobian finite-difference error at most
  `5e-7`;
- fixed layout with `jacobian_pattern_rebuilds == 1`;
- Schur/full-LU oracle step relative error at most `1e-10` and complete-system
  backward error at most `1e-12`;
- singular selected `D_i` rejection and production full-LU fallback;
- case300/case2000 Schur-to-full solved `Vm/Va/Vdc` difference at most `1e-8`;
- exact Schur dimension reductions of 36/48 and structural-nonzero reductions
  of at least 288/384 for case300/case2000;
- PF-to-transient Norton seed errors at most `1e-8`;
- OPF-to-PF replay certificate residual at most `1e-9`.

The suite covers all priorities, two simultaneous GFM limits, degeneracy,
weak-grid and infeasible cases, quasi-steady replay, OPF replay, transient
initialization, JSON/JPC round trips, stable IDs, a sole slackless GFM island,
multiple Norton sources in one slackless island, and adaptive multi-island
extraction without manufactured SLACK buses. Smooth-continuation integration
covers all three power-port priorities and all three current-limited GFM
priorities, with final `mu <= 1e-12` and exact certificate residual at most
`1e-9`.

These are local semismooth-Newton guarantees around the certified high-voltage
solution. The nonlinear island equations may also have low-voltage mathematical
roots; a sufficiently remote rotated seed has demonstrated a different basin.
The contract therefore does not claim global uniqueness or global convergence.

OpenDSS validation has two evidence levels. At the nonbinding point, an
equation-equivalent two-source Thevenin circuit is solved independently with
`Vm <= 1e-4 pu`, angle `<= 0.02 deg`, and `P/Q <= 1e-3 pu` tolerances. At a
binding point OpenDSS has no equivalent priority-NCP controller, so the HySim
internal voltage is frozen and OpenDSS independently re-solves the circuit;
its native terminal-power KCL residual must be at most `1e-6 pu`. The latter is
a frozen-circuit validation, not an independent mode-selection oracle.

`build_gfm_norton_limit_demo()` is the grid-connected flat-start production/GUI case. It has
an independent terminal AC angle reference, one VDC-Q station providing physical DC
voltage/energy support, and one explicit GFM Norton/current-limit NCP station.

`build_case300_acdc_vsc_limit_ncp()` and
`build_case2000_acdc_vsc_limit_ncp()` remain versioned MTDC/PQ-Vdc-Q CPU sparse
benchmarks: 300 AC + 6 DC + 6 VSC and 2000 AC + 8 DC + 8 VSC, each with four
simultaneous limits. `build_case300_acdc_gfm_limit_ncp()` and
`build_case2000_acdc_gfm_limit_ncp()` add one nonbinding GFM Norton block and
are certified from the corresponding solved MTDC state as explicit
continuation datasets. They prevent pure-AC scalability evidence from being
misreported, but do not prove flat-start robustness, large-scale simultaneous
GFM binding, or GPU acceleration.

The ordinary `build_case2000_acdc()` built-in is separately registered against
the non-NCP production hybrid Newton path. With GUI-equivalent `tol=1e-8`,
`max_iter=100`, converter coordination enabled, and automatic fallback
explicitly disabled, it must converge with residual at most `1e-8` and all DC
voltages in `[0.9,1.1] pu`. Pure-AC convergence is not accepted as this
certificate.

`vsc_schur_benchmark` is the non-CTest Release benchmark. Its fixed protocol is
two warmups and five alternating full/Schur repetitions. It labels case300 as
a forced research ablation and records why production admission excludes it.
On the current macOS 26.5.2 / Apple M4 Max clean Release/KLU build at repository
`1773aa0e75d7` and MIPSolvers `3bf1e66749e3`, case300 measured
`0.524958/0.689582 ms` full/Schur linear time and `2.522875/3.353792 ms`
full/Schur wall time. ACTIVSg2000 measured `31.979500/27.581291 ms` linear and
`47.372583/42.373500 ms` wall time, with 33 attempted, 19 accepted, and 14
fallback Schur steps.
The current KLU adapter returns `-1` for factor nonzeros/work, so the benchmark
sets `factor_statistics_available=false` rather than fabricating fill data.

## Mathematical references

- F. Facchinei and J.-S. Pang, *Finite-Dimensional Variational Inequalities
  and Complementarity Problems*, Vol. I, Springer, 2003, Sec. 9.1.
- L. Qi and J. Sun, "A nonsmooth version of Newton's method," *Mathematical
  Programming*, 58, 1993, pp. 353-367.
- A. Fischer, "A special Newton-type optimization method," *Optimization*,
  24, 1992, pp. 269-284.
- C. Kanzow, "Some noninterior continuation methods for linear
  complementarity problems," *SIAM Journal on Matrix Analysis and
  Applications*, 17(4), 1996, pp. 851-868.
- G. H. Golub and C. F. Van Loan, *Matrix Computations*, 4th ed., Johns
  Hopkins University Press, 2013, block Gaussian elimination.
- T. A. Davis, *Direct Methods for Sparse Linear Systems*, SIAM, 2006.
- N. J. Higham, *Accuracy and Stability of Numerical Algorithms*, 2nd ed.,
  SIAM, 2002, Secs. 2.2 and 7.1.
