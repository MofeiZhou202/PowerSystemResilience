# Spatiotemporal multidimensional weak-link identification

## Scope

The `hacdcpf.multidimensional_weak_link.v1` module combines evidence from
economic operation, carbon flow, reliability, and resilience analyses. It is a
screening and decision-routing layer. It does not claim that an attributed
emission or outage contribution is the counterfactual benefit of an investment.

The GUI exposes two decision modes:

- `planning`: integrated upgrade, hardening/redundancy, loss reduction, and
  Pareto trade-off review;
- `operation`: coordinated dispatch, redispatch/demand response, emergency
  reconfiguration, and operational Pareto review.

## Mathematical foundation

The method is a non-compensatory multicriteria vulnerability screen. It combines
threshold normalization, Chebyshev bottleneck aggregation, max-min consensus,
Pareto partial ordering, and a separate temporal risk projection. It is not a
weighted-sum utility model: a severe reliability problem cannot be cancelled by
a favorable economic score.

Let `i` denote a spatial entity, `t` a time period, and
`k in {economic, carbon, reliability, resilience}` a dimension. Let `x_ik` be
the available raw pressure evidence and `tau_k > 0` its engineering target. The
dimensionless threshold-exceedance ratio is

`p_ik = max(x_ik, 0) / tau_k`.

`p_ik = 1` means the target has been reached and `p_ik > 1` means it is
exceeded. Define `K_i` as the set of available dimensions for entity `i`.
Missing dimensions are outside `K_i`; they are not imputed as zero.

Three non-compensatory statistics are calculated:

`severity_i = max(k in K_i) p_ik`,

`consensus_i = min(k in K_i) p_ik`,

`disagreement_i = severity_i - consensus_i`.

Severity is the Chebyshev (`L-infinity`) distance from the all-zero pressure
origin and identifies the worst bottleneck. Consensus is the max-min/fuzzy-AND
aggregation: it is high only when every available dimension is high.
Disagreement is the range seminorm and exposes cross-dimension imbalance.

These definitions provide three useful invariants:

1. monotonicity: increasing any available pressure cannot decrease severity;
2. non-compensation: reducing another dimension cannot hide the maximum pressure;
3. shared-weakness protection: one extreme dimension cannot increase consensus.

The classification is a deterministic decision partition. With the current
defaults `h=0.75`, `c=0.50`, and `g=0.60`:

- compatible: `|K_i| >= 3` and `consensus_i >= c`;
- conflict: `|K_i| >= 2`, `severity_i >= h`, and `disagreement_i >= g`;
- single-dimension: `severity_i >= h` after the previous tests;
- watch: evidence is available but the high-pressure condition is absent;
- insufficient: `|K_i|` is below the configured evidence requirement.

### Pareto weak frontier

Entities are Pareto-compared only when they have the same coverage set. Entity
`j` weakly dominates entity `i` when

`p_jk >= p_ik for every k in K_i`,

and the inequality is strict for at least one dimension. A Pareto weak link is
an entity not dominated by another comparable entity. This frontier means that
the entity cannot be dismissed as uniformly less weak; it does not mean that
the entity or a proposed investment is optimal.

The final ordering is lexicographic: Pareto membership first, then descending
severity, descending consensus, and a stable entity key. Temporal rows use the
same dimension normalization and are ordered by descending period severity.
The current implementation exposes two marginal views, entity-by-dimension and
period-by-dimension; it does not yet estimate a complete entity-by-time-by-
dimension tensor.

### What is and is not guaranteed

For fixed inputs and thresholds, the result is deterministic, monotone, and
reproducible. Coverage-aware Pareto comparison prevents an entity with missing
dimensions from dominating a fully observed entity. These are algorithmic
guarantees, not statistical guarantees about the physical system.

Result validity still requires converged source analyses, calibrated failure
and repair data, representative hazards and time series, and engineering
thresholds appropriate to the decision. The current GUI also uses relative
normalization for some evidence, including LMP deviation, branch carbon rank,
and peak-relative temporal cost/emissions. Those values support within-case
screening but should not be compared across unrelated cases without a common
absolute target.

A planning result should be accepted only after the following audit:

1. require four-dimensional coverage for an integrated conclusion;
2. reject or flag any unconverged source analysis and retain every evidence detail;
3. perturb each threshold and key model parameter and check frontier/rank stability;
4. repeat the screen across load, renewable, outage, maintenance, and hazard scenarios;
5. confirm proposed actions with the independent counterfactual benefit calculation below.

Statistical confidence intervals, probabilistic rank acceptability, and
scenario-frequency robustness are not yet part of schema v1. They are required
before interpreting a screen as a probabilistic assurance statement.

## Evidence contract

Every spatial entity and time period may provide zero to four dimension records:

```json
{
  "pressure": 1.15,
  "detail": "EENS contribution 1.15 MWh/yr / target 1.00 MWh/yr"
}
```

Pressure is dimensionless and always follows `higher_is_weaker`. A value of 1.0
means that the configured engineering threshold has been reached. Missing
evidence is unavailable, not zero. The GUI currently derives pressure from:

| Dimension | Spatial evidence | Temporal evidence |
| --- | --- | --- |
| Economic | branch loading and nodal LMP deviation | time-series OPF cost |
| Carbon | nodal intensity and branch-loss emissions | dynamic carbon emissions |
| Reliability | contingency/component and nodal EENS | reserved for chronological reliability traces |
| Resilience | bus-level weighted unserved energy | disaster-step weighted unserved energy |

## Ranking and relations

For available dimension `k`, the backend computes

`normalized_pressure[k] = max(raw_pressure[k], 0) / target_pressure[k]`.

The entity severity is the maximum available pressure. Consensus is the minimum
available pressure, so it cannot be increased by a single extreme dimension.
Disagreement is `max - min` over available dimensions.

The relation labels are:

- `compatible`: at least three dimensions are available and all exceed the
  compatibility threshold;
- `conflict`: at least two dimensions are available, one is high, and the
  cross-dimension gap exceeds the conflict threshold;
- `single_dimension`: high pressure without sufficient shared pressure;
- `watch`: evidence exists but no high-pressure condition is met;
- `insufficient_evidence`: fewer than the configured minimum dimensions.

Pareto dominance is evaluated only between entities with the same dimension
coverage mask. This prevents a two-dimension entity from dominating a
four-dimension entity merely because its missing dimensions were treated as
zero. The API returns the original evidence detail for auditability.

## Direct visual encoding

The GUI presents the spatial and temporal results as annotated pressure
heatmaps. Rows are ranked entities or critical periods, columns are the four
dimensions, and every available cell displays its exact normalized pressure.
The shared scale anchors `0.50`, `0.75`, and the engineering threshold `1.00`;
values above `1.50` are color-clipped but retain their exact text and hover
detail. Missing evidence is rendered separately and never receives a low-risk
color. Pareto entities are marked with a diamond and the relation label is kept
next to the entity name.

## Counterfactual planning

The `hacdcpf.counterfactual_planning.v1` assessment converts screening results
into auditable planning evidence. It applies each selected measure to an
independent clone of the same baseline system and re-runs the source analyses.
It never estimates investment benefit by scaling a weak-link score.

The built-in candidate generator exposes five explicit measure types:

| Type | Counterfactual model change |
| --- | --- |
| `line_capacity` | increase the selected AC branch rating and reduce its parallel-equivalent impedance by the same factor |
| `storage` | add grid-forming AC storage with explicit power, energy, SOC, dispatch and outage parameters |
| `tie_switch` | add a normally-open AC tie branch and remote switch between otherwise unconnected buses |
| `automation` | add or upgrade a feeder switch to remote automatic operation |
| `der` | add controllable zero-operational-carbon distributed generation with explicit capacity and operating-point capacity factor |

The API also accepts an explicit `measures` array, so a planning workflow can
target branch and bus indices identified by the weak-link screen instead of
using the automatic candidate locations. Candidate and pair counts are bounded
by `max_measures` and `max_pairs` because every row is a real system assessment.

For metric `k`, where lower is better, the signed single-measure benefit is

`B_k(a) = baseline_k - counterfactual_k(a)`.

A positive value is an improvement and a negative value is a deterioration.
The pairwise interaction is computed from a separate combined re-run:

`S_k(a,b) = B_k(a+b) - B_k(a) - B_k(b)`.

Positive `S` is synergy beyond additive benefits. Negative `S` indicates
redundancy or conflict. The JSON response retains absolute values, relative
values, units, solver diagnostics, application status and the equations used.
The GUI shows the four signed values in every pairwise-matrix cell; it does not
collapse economically beneficial but reliability-adverse combinations into a
single score.

The recomputed dimensions are:

- economic: annualized AC OPF objective, with an annualized monetized physical-loss fallback;
- carbon: annualized matrix carbon flow or proportional tracing result;
- reliability: deterministic distribution N-1 FMEA;
- resilience: heuristic sequential restoration under the same explicit branch-fault set for the baseline and every clone.

The current assessment is a planning comparison, not a multiyear capacity
expansion optimizer. CAPEX is reported separately from operating cost; storage
and DER use a configured representative operating point rather than an 8760-hour
investment dispatch; and automatic candidate generation currently proposes at
most one candidate per enabled type. These limitations are explicit so the
benefit and synergy matrix is not mistaken for a lifecycle NPV optimum.
