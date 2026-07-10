/// test_sppt_benchmark.cpp
/// =======================
/// Pillar-4 self-contained scale/accuracy benchmark: the SPPT pipeline solves and
/// preserves semantics across a case ladder (docs/latex/sppt_theory.tex).

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/benchmark.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

using namespace hacdcpf;

namespace {
std::string data_path(const std::string& name) {
  return std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
}
}  // namespace

TEST_CASE("Benchmark: SPPT pipeline solves and preserves semantics across a ladder",
          "[sppt][benchmark]") {
  for (const char* c : {"case9.m", "case14.m", "case30.m", "case57.m", "case118.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(c));
    const auto row = sppt::run_benchmark_case(sys, c, /*repeats=*/2);
    INFO(c << ": bus=" << row.n_ac_bus << " solve_ms=" << row.solve_ms
           << " residual=" << row.commuting_residual << " note=" << row.note);
    CHECK(row.n_ac_bus > 0);
    CHECK(row.converged);
    CHECK(row.solve_ms >= 0.0);
    CHECK(row.project_ms >= 0.0);
    CHECK(row.assemble_ms >= 0.0);
    CHECK(row.commuting_residual < 1e-6);   // accuracy: semantics preserved
  }
}

TEST_CASE("Benchmark renders CSV and LaTeX", "[sppt][benchmark][render]") {
  const std::vector<std::pair<std::string, std::string>> cases = {
      {"case9", data_path("case9.m")},
      {"case14", data_path("case14.m")},
  };
  const sppt::Benchmark bench = sppt::run_benchmark(cases, 2);
  REQUIRE(bench.rows.size() == 2);

  const std::string csv = bench.to_csv();
  CHECK(csv.find("case,n_ac_bus") != std::string::npos);
  const std::string tex = bench.to_latex();
  CHECK(tex.find("\\begin{tabular}") != std::string::npos);
  CHECK(tex.find("solve (ms)") != std::string::npos);
}
