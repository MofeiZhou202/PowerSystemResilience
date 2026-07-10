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

namespace hacdcpf::sppt {

/// One row of the certificate corpus.
struct CertificateRow {
  std::string case_name;
  int    n_ac_bus{0};
  int    n_dc_bus{0};

  bool   pf_converged{false};
  double pf_residual{0.0};       ///< MR3 power-flow commuting residual
  bool   pf_pass{false};

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

}  // namespace hacdcpf::sppt
