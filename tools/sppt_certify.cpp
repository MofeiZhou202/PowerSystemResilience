/// tools/sppt_certify.cpp
/// ======================
/// Emits the MR3 semantic-preservation certificate corpus (Thm. 5.6, Pillar 3
/// of docs/latex/sppt_theory.tex) across representative IEEE/MATPOWER cases as a
/// \input-able LaTeX table and a CSV, plus a console summary.
///
/// Usage:  sppt_certify [output_dir] [tolerance]
///   output_dir  where to write sppt_mr3_certificate.{tex,csv}
///               (default: <project_root>/docs/latex)
///   tolerance   MR3 residual tolerance (default: 1e-6)
///
/// Exit code 0 iff every case in the corpus passes.

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/sppt/certificate.hpp"

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "."
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;

int main(int argc, char** argv) {
  const fs::path root = HACDCPF_PROJECT_ROOT;
  const fs::path data = root / "data";
  const fs::path outdir = (argc > 1) ? fs::path(argv[1]) : (root / "docs" / "latex");
  const double tol = (argc > 2) ? std::stod(argv[2]) : 1e-6;

  // Curated corpus: Tier-1 IEEE cases that ship in data/.
  const std::vector<std::string> names = {
      "case5.m", "case9.m", "case14.m", "case30.m",
      "case57.m", "case118.m", "case300.m"};

  std::vector<std::pair<std::string, std::string>> cases;
  for (const auto& n : names) {
    const fs::path p = data / n;
    if (fs::exists(p)) cases.emplace_back(n, p.string());
  }
  if (cases.empty()) {
    std::cerr << "sppt_certify: no cases found under " << data << "\n";
    return 2;
  }

  const sppt::Certificate cert = sppt::certify_corpus(cases, tol);

  std::error_code ec;
  fs::create_directories(outdir, ec);
  {
    std::ofstream csv(outdir / "sppt_mr3_certificate.csv");
    csv << cert.to_csv();
  }
  {
    std::ofstream tex(outdir / "sppt_mr3_certificate.tex");
    tex << cert.to_latex();
  }

  std::cout << cert.to_csv() << "\n"
            << "MR3 certificate: " << cert.pass_count() << "/" << cert.rows.size()
            << " cases fully pass; all_pass="
            << (cert.all_pass() ? "yes" : "no") << "\n"
            << "Wrote " << (outdir / "sppt_mr3_certificate.tex").string()
            << " and .csv\n";

  return cert.all_pass() ? 0 : 1;
}
