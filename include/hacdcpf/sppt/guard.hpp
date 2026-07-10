#pragma once

/// sppt/guard.hpp
/// ==============
/// The SPPT admissibility guard (Def. 8.7, Alg. 2 of docs/latex/sppt_theory.tex).
/// A model, or a model produced by an AI/agent edit, is SPPT-admissible iff it
///   (i)   passes boundary validation,
///   (ii)  is well-posed (every island keeps a unique/sufficient reference), and
///   (iii) is fully attributable (every canonical result maps back to a device).
/// The guard turns those three gates into a single Accept/Reject verdict so an
/// autonomous loop (Thm. 8.9) can never certify an ill-posed or unattributable
/// state.

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::sppt {

/// Verdict of the admissibility guard for one system.
struct GuardVerdict {
  bool accepted{false};
  std::string reason;            ///< empty when accepted; else the failing gate

  bool validation_ok{false};     ///< gate (i)
  bool well_posed{false};        ///< gate (ii)
  bool attribution_total{false}; ///< gate (iii)

  std::vector<std::string> details;  ///< human-readable gate diagnostics
};

/// Evaluate the three admissibility gates on \p sys (Alg. 2, lines 2-7).
GuardVerdict guard_system(const HybridPowerSystem& sys);

// ── LLM-in-the-loop evaluation (Pillar 5) ────────────────────────────────────

/// A labeled candidate model / edit with its ground-truth admissibility.
struct LabeledEdit {
  std::string name;
  HybridPowerSystem system;
  bool expected_admissible{true};
};

/// Confusion counts and derived guard-quality metrics over a labeled edit set.
/// Positive class = "admissible".
struct GuardMetrics {
  int tp{0};  ///< admissible & accepted
  int fp{0};  ///< inadmissible but accepted (a missed rejection)
  int tn{0};  ///< inadmissible & rejected (a correct catch)
  int fn{0};  ///< admissible but rejected (over-rejection)

  [[nodiscard]] int total() const { return tp + fp + tn + fn; }
  [[nodiscard]] double precision() const;   ///< tp / (tp + fp)
  [[nodiscard]] double recall() const;      ///< tp / (tp + fn)
  [[nodiscard]] double catch_rate() const;  ///< tn / (tn + fp): rejected inadmissibles
  [[nodiscard]] double accuracy() const;    ///< (tp + tn) / total
};

/// Run the guard over a labeled edit set and tally metrics.
GuardMetrics evaluate_guard(const std::vector<LabeledEdit>& edits);

}  // namespace hacdcpf::sppt
