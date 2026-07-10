/// test_sppt_certificate.cpp
/// =========================
/// The MR3 semantic-preservation certificate corpus (Thm. 5.6, Pillar 3).

#include <string>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/certificate.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;

namespace {
std::string data_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
}
}  // namespace

TEST_CASE("certify_case yields a passing PF certificate", "[sppt][certificate]") {
  for (const char* c : {"case9.m", "case14.m", "case30.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(c));
    const auto row = sppt::certify_case(sys, c, 1e-6);
    INFO(c << ": pf_residual=" << row.pf_residual
           << " opf_dual_residual=" << row.opf_dual_residual);
    CHECK(row.pf_converged);
    CHECK(row.pf_pass);
    CHECK(row.pf_residual < 1e-6);
  }
}

TEST_CASE("certify_corpus renders CSV and \\input-able LaTeX",
          "[sppt][certificate][corpus]") {
  const std::vector<std::pair<std::string, std::string>> cases = {
      {"case9", data_path("case9.m")},
      {"case14", data_path("case14.m")},
      {"case30", data_path("case30.m")},
  };
  const sppt::Certificate cert = sppt::certify_corpus(cases, 1e-6);

  REQUIRE(cert.rows.size() == cases.size());
  for (const auto& r : cert.rows) {
    INFO(r.case_name << ": pf_pass=" << r.pf_pass << " note=" << r.note);
    CHECK(r.pf_pass);
  }

  const std::string csv = cert.to_csv();
  CHECK(csv.find("case,n_ac_bus") != std::string::npos);
  CHECK(csv.find("case9") != std::string::npos);

  const std::string tex = cert.to_latex();
  CHECK(tex.find("\\begin{tabular}") != std::string::npos);
  CHECK(tex.find("\\bottomrule") != std::string::npos);

  CHECK(cert.all_pass());
}
