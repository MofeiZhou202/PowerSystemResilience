#pragma once

/// sppt/certificate.hpp
/// ====================
/// The MR3 semantic-preservation certificate corpus (Thm. 5.6, Pillar 3 of the
/// verification program in docs/latex/sppt_theory.tex).  For each case it emits
/// the commuting residual || A_ref - R . A_can . Pi || for power flow and for
/// OPF nodal prices, and renders the corpus as CSV and as a \input-able LaTeX
/// table that becomes the platform's machine-checked correctness evidence.

#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::sppt {

struct IndependentResidualCertificate {
  bool supported{false};
  bool converged{false};
  double ac_residual{0.0};
  double dc_residual{0.0};
  double converter_balance_residual{0.0};
  double total_residual{0.0};
  std::string scope;
  std::string note;
};

/// Evaluate the converged state against authored primitive AC/DC/VSC equations.
/// This path intentionally does not call project_to_canonical_models,
/// make_solver_data, or the production residual evaluator.
IndependentResidualCertificate certify_independent_hybrid_residual(
    const HybridPowerSystem& authored,
    const PowerFlowResult& result,
    double tolerance = 1e-6);

/// One row of the certificate corpus.
struct CertificateRow {
  std::string case_name;
  int    n_ac_bus{0};
  int    n_dc_bus{0};

  bool   pf_converged{false};
  double pf_residual{0.0};       ///< MR3 power-flow commuting residual
  bool   pf_pass{false};

  bool   independent_supported{false};
  double independent_residual{0.0};
  bool   independent_pass{false};
  int    approximate_merges{0};
  bool   attribution_total{false};

  bool   opf_converged{false};
  double opf_dual_residual{0.0}; ///< MR3 OPF nodal-price commuting residual
  bool   opf_pass{false};

  double tolerance{0.0};
  std::string note;              ///< e.g. skip reason when a stage is N/A
};

/// The corpus and its renderers.
struct Certificate {
  double tolerance{1e-6};
  std::vector<CertificateRow> rows;

  [[nodiscard]] bool all_pass() const;
  [[nodiscard]] int  pass_count() const;

  /// Comma-separated values with a header row.
  [[nodiscard]] std::string to_csv() const;

  /// A booktabs `tabular` body, ready to \input into the SPPT paper.
  [[nodiscard]] std::string to_latex() const;
};

/// Certify a single already-loaded system.
CertificateRow certify_case(const HybridPowerSystem& sys,
                            std::string case_name,
                            double tol = 1e-6);

/// Certify a corpus given (display-name, file-path) pairs.  MATPOWER `.m` and
/// JSON `.json`/`.jpc` files are auto-detected by extension; unreadable cases
/// are recorded as a non-passing row with a note rather than aborting.
Certificate certify_corpus(const std::vector<std::pair<std::string, std::string>>& cases,
                           double tol = 1e-6);

/// Append already-constructed systems, including native hybrid AC/DC cases, to
/// an existing certificate corpus.
void certify_systems(Certificate& certificate,
                     const std::vector<std::pair<std::string, HybridPowerSystem>>& systems,
                     double tol = 1e-6);

}  // namespace hacdcpf::sppt
