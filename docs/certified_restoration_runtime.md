# Certified Restoration Runtime Contract

## Scope

The executable implementation is in
`include/hacdcpf/resilience/certified_restoration.hpp` and
`src/resilience/certified_restoration.cpp`. It solves a finite restoration
action catalog. It is not the internal decomposition of the full multi-period
AC-only LinDistFlow restoration MIP.

Each catalog action carries:

- an objective and dynamic event sequence;
- cyber path, authorization, and acknowledgement availability;
- MESS arrival deadline, available/required energy, target AC bus, dispatch,
  and GFM requirement.

The HiGHS master selects exactly one executable, unexcluded action. A master
solution is used as an upper bound only when its MIP gap is zero. L3-safe
actions update the incumbent. A no-good cut is added only for an L3 result with
`proof_valid=true` and label `unsafe`. An unresolved or failed oracle stops the
loop without a cut.

## Certificate quantities

For a trajectory snapshot, the native phase-domain residual evaluator returns

\[
r_f=\dot x-f(t,x,y),\qquad
r_g=\begin{bmatrix}
\Re(I_{ac}-Y_{ac}V_{abc})\\
\Im(I_{ac}-Y_{ac}V_{abc})\\
I_{dc}-G_{dc}V_{dc}
\end{bmatrix}.
\]

The differential coordinates are scaled as \(\widetilde x=D_x^{-1}x\).
Dense finite-difference Jacobians form the Schur-reduced local matrix

\[
A_D=D_x^{-1}(F_x-F_yG_y^{-1}G_x)D_x.
\]

For L1/L2, which currently use the same physical DAE at different integration
steps, each pair of accepted endpoints and endpoint fields defines a cubic
Hermite reconstruction \(\widehat x_H(t)\). Its continuous defect is

\[
\rho_H(t)=D_x^{-1}\left[\dot{\widehat x}_H(t)-
f(t,\widehat x_H(t),\psi(\widehat x_H(t)))\right],\qquad
R_x=\kappa_g e_0+\int\|\rho_H(t)\|_\infty dt+
\kappa_g\int\|r_g\|_\infty dt.
\]

The implementation samples this defect at two Gauss nodes per step. At a step
ending on an event, the right field is evaluated on the pre-event side; the
event map is applied before the next step. A state reset therefore appears in
the reconstruction defect unless separately enclosed by a verified reset
bound. The snapshot-based `r_f` remains a model-mismatch diagnostic.

The full-state indicator is retained for audit only:

\[
\eta=K_\Phi(T)R_x,\qquad
K_\Phi(T)=\sup_{0\le\tau\le T}\|\exp(A_D\tau)\|_\infty.
\]

Classification uses an output-directed bound. For security output \(q_j\),

\[
c_j^\mathsf{T}=(q_{j,x}-q_{j,y}G_y^{-1}G_x)D_x,
\quad
K_j(t,s)=\|c_j^\mathsf{T}(t)\Phi_D(t,s)\|_1,
\]

and

\[
\Delta_j(t)=K_j(t,0)e_0+
\int_0^tK_j(t,s)\|\rho_H(s)\|_\infty ds+
B_j\|r_g\|_{\infty,[0,t]},\qquad
B_j=\|q_{j,y}G_y^{-1}\|_1.
\]

The direct algebraic term is zero for COI frequency and is retained for AC
voltage. The sampled implementation obtains \(c_j\) by perturbing the scaled
state and resolving the network. RoCoF is the instantaneous inertia-weighted
derivative of the participating machine and virtual-speed states, avoiding a
step-dependent backward-difference definition.

`MarginCertificate` reports the local output sensitivity, output transition
gain, algebraic direct gain, bound method, and resulting `Delta_j`.
`MultiFidelityCertificate` separately reports the reconstruction-defect integral,
forcing method, full-state `eta`, and its construction method.

When the algebraic dimension exceeds `dense_jacobian_dimension_limit`, the
engine uses deterministic directional sensitivities. This path cannot form the
dense output semigroup and falls back to the full-state indicator.

All current Jacobians, transition gains, and output gradients are sampled
estimates. Therefore
`constants_are_global_bounds=false`, and L1/L2 labels are diagnostic rather
than analytic proofs even when their intervals separate zero. A formally
certified L1/L2 pruning implementation must replace the sampled quantities by
validated tube enclosures of \(G_y^{-1}\), \(A_D\), \(c_j\), reset maps, and
event-location error. Converter current-limit activation remains a discrete
safety gate and is not assigned a fictitious continuous `Delta_j`.

## Fidelity levels

- L1: configured full device model with a coarse integration step.
- L2: configured full device model with a finer integration step.
- L3: configured full DAE step and scenario/horizon-specific threshold oracle.

L1/L2 are numerical fidelities, not different physical model families. L3
requires successful simulation, qualified initialization, frequency/voltage/
RoCoF observations, and the configured converter-current observation gate.
`proof_valid` means the L3 classification inputs are complete; it does not mean
a global analytic stability proof.

## MESS contract

`MobileStorage::grid_forming` is the single per-device GFM capability field.
The strict restoration MIP permits `mess_root=1` only when both this field and
`allow_mess_black_start` are true. The certificate executability check uses the
same field.

For a route/SOC/GFM-qualified post-arrival action, the engine converts the
selected mobile asset into an AC `Storage` at the target bus before building the
dynamic model. The trajectory reports `mess_materialized_in_dae` and
`mess_dynamic_device_observed`. Travel dynamics are not integrated on the
sub-second DAE timeline; arrival and travel energy are master-level premises.

## Conditional catalog guarantee

Assume every master solve has zero MIP gap and each accepted L3 label is sound
for its declared scenario and horizon. Inductively, every no-good cut removes
only an unsafe action. If the next master objective is no larger than the best
safe incumbent, no remaining executable catalog action can improve that
incumbent. The returned action is then optimal over the encoded finite catalog.
The statement does not cover actions absent from the catalog or physics omitted
from L3.

## Reproduction

```sh
cmake --build build/macos-release --target \
  cyber_dynamic_safe_restoration_small \
  cyber_dynamic_safe_restoration_scale
./build/macos-release/cyber_dynamic_safe_restoration_small
./build/macos-release/cyber_dynamic_safe_restoration_scale
```

Outputs are under
`docs/latex/paper/cyber_dynamic_safe_restoration/results/small_baseline` and
`docs/latex/paper/cyber_dynamic_safe_restoration/results/scale_sentinels`.
