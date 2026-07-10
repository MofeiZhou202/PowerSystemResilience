#pragma once

/// sppt/ablation.hpp
/// =================
/// Pillar-4 ablation study (docs/latex/sppt_theory.tex): disabling each piece of
/// SPPT machinery and measuring the damage, to demonstrate its necessity.
///
///   Ablation A  remove provenance $\Att$   -> results become unattributable
///   Ablation B  remove role typing $\rho$  -> silent wrong-reference solves
///   Ablation C  remove the merge guard     -> ill-conditioned $Y_{bus}$

#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::sppt {

/// One case's response to all three ablations.
struct AblationRow {
  std::string case_name;

  // ── Ablation A: provenance removed ────────────────────────────────────────
  int    prov_unattributable_ablated{0};   ///< canonical entities with no device attribution
  int    prov_unattributable_guarded{0};   ///< with $\Att$: always 0
  double prov_fraction{0.0};               ///< ablated fraction of entities

  // ── Ablation B: role typing / wp gate removed ─────────────────────────────
  bool role_guard_rejects{false};  ///< SPPT gate catches the missing reference
  bool role_raw_silent{false};     ///< ablated raw solver returns "converged" anyway

  // ── Ablation C: merge fill-guard removed ──────────────────────────────────
  bool   merge_applicable{false};        ///< case has switch/CB elements to merge
  double merge_max_diag_merged{0.0};     ///< max |Y_ii| with the merge guard
  double merge_max_diag_unmerged{0.0};   ///< max |Y_ii| without it
  double merge_blowup{0.0};              ///< unmerged / merged
};

/// Aggregate study plus renderers.
struct AblationStudy {
  std::vector<AblationRow> rows;

  [[nodiscard]] int    total_unattributable_without_provenance() const;
  [[nodiscard]] int    silent_solves_without_role_typing() const;
  [[nodiscard]] double max_merge_blowup() const;

  [[nodiscard]] std::string to_csv() const;
  [[nodiscard]] std::string to_latex() const;
};

/// Run all three ablations on one system.
AblationRow run_ablation_case(const HybridPowerSystem& sys, std::string case_name);

/// Run the study over (display-name, file-path) pairs.  Unreadable cases are
/// recorded with a note rather than aborting.
AblationStudy run_ablation_study(const std::vector<std::pair<std::string, std::string>>& cases);

}  // namespace hacdcpf::sppt
