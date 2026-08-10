#pragma once

/// sppt/metamorphic.hpp
/// ====================
/// Executable metamorphic relations (MR1-MR8) realizing the theorems of
/// "Semantics-Preserving Projection Theory" (docs/latex/sppt_theory.tex).
///
/// Each relation turns a theorem into a computable pass/residual so the theory
/// is continuously falsified by the regression suite rather than asserted on
/// paper.  The mapping is:
///
///   MR1  Projection idempotence           Prop. 4.5   Pi(Pi(S)) ~= Pi(S)
///   MR2  Attribution round-trip           Def. 5.4    R_S(iota_S(x)) = x
///   MR3  Semantic preservation (PF)        Thm. 5.6    ||A_ref - R.A_can.Pi|| <= eps
///   MR3d Semantic preservation (OPF dual)  Thm. 5.6    LMP preserved through Pi
///   MR4  Merged-bus equipotential          Lem. 5.5    members share representative V
///   MR5  Merge / relabel invariance        Prop. 4.6   result invariant to relabeling
///   MR6  Compositionality                  Thm. 7.4    Pi distributes over gluing
///   MR7  Well-posedness gate               Cor. 6.5    missing reference => typed reject
///   MR8  DAE index-1 well-posedness         Thm. 6.6    algebraic sub-Jacobian nonsingular

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::sppt {

enum class MetamorphicObservation {
  Observed,
  NotObserved,
};

/// Outcome of a single metamorphic relation.
struct MetamorphicResult {
  std::string id;          ///< "MR1" .. "MR8"
  std::string name;        ///< human-readable relation name
  bool        passed{false};
  MetamorphicObservation observation{MetamorphicObservation::Observed};
  double      residual{0.0};    ///< measured residual / deviation (0 when N/A)
  double      tolerance{0.0};   ///< tolerance the residual was checked against
  std::string detail;      ///< diagnostic note (why it passed, failed, or was not observed)
};

// ── MR1: projection idempotence ──────────────────────────────────────────────
// Pi(Pi(S)) is structurally identical to Pi(S) (Prop. 4.5).
MetamorphicResult mr1_projection_idempotence(const HybridPowerSystem& sys);

// ── MR2: attribution round-trip ──────────────────────────────────────────────
// Lifting a rich-space bus field to canonical space and attributing it back with
// R_S recovers the original values on recoverable observables (Def. 5.4).
MetamorphicResult mr2_attribution_roundtrip(const HybridPowerSystem& sys,
                                            double tol = 1e-12);

// ── MR3: semantic preservation of power flow (Thm. 5.6) ──────────────────────
// || A_ref(S) - R_S(A_can(Pi(S))) || over bus voltages, where A_ref solves the
// authored system and A_can solves the explicitly projected system.
MetamorphicResult mr3_semantic_preservation_pf(const HybridPowerSystem& sys,
                                               double tol = 1e-6);

// ── MR3d: semantic preservation of OPF nodal prices (duals) ──────────────────
// Same commuting property applied to DC-OPF locational marginal prices.
MetamorphicResult mr3_semantic_preservation_opf_dual(const HybridPowerSystem& sys,
                                                     double tol = 1e-6);

// ── MR3t: semantic preservation of transient trajectories ────────────────────
// The commuting property applied to electromechanical transient simulation:
// final-snapshot observables (bus/COI frequency, voltage envelope, and the
// differential-state / DC-voltage vectors) agree between solving the authored
// system and solving the explicitly projected system.
MetamorphicResult mr3_semantic_preservation_transient(const HybridPowerSystem& sys,
                                                      double tol = 1e-6);

// ── MR4: merged-bus equipotential (Lem. 5.5) ─────────────────────────────────
// After a solve, every member of a zero-impedance merge class shares the
// representative's voltage.  Trivially passes when there are no merges.
MetamorphicResult mr4_merged_bus_equipotential(const HybridPowerSystem& sys,
                                               double tol = 1e-9);

// ── MR5: merge / relabel invariance (Prop. 4.6) ──────────────────────────────
// Permuting the AC-bus ordering (a relabeling) leaves the per-bus-id solution
// invariant once results are attributed back through R_S.
MetamorphicResult mr5_relabel_invariance(const HybridPowerSystem& sys,
                                         double tol = 1e-7);

// ── MR6: compositionality of projection (Thm. 7.4) ───────────────────────────
// With the empty interface, Pi distributes over disjoint union:
// Pi(S1 (+) S2) has bus/branch counts equal to Pi(S1) + Pi(S2).
MetamorphicResult mr6_compositionality(const HybridPowerSystem& s1,
                                       const HybridPowerSystem& s2);

// ── MR7: well-posedness gate (Cor. 6.5) ──────────────────────────────────────
// Removing an island's unique reference produces a typed rejection (a validation
// error or a solver diagnostic), never a silently "converged" wrong answer.
MetamorphicResult mr7_wellposedness_gate(const HybridPowerSystem& sys);

// ── Convenience: run the single-system relations (MR1,2,3,3d,4,5,7) ──────────
std::vector<MetamorphicResult> run_core_metamorphic_suite(const HybridPowerSystem& sys);

}  // namespace hacdcpf::sppt
