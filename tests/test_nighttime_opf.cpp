/// @brief Quick smoke test: nansha_full_network OPF at nighttime (no PV).
///
/// Without the fix, solve_ac_opf immediately returns failure when
/// sys.ac.generators is empty (ng == 0). With the fix, external grids
/// with cost_c2 > 0 are injected as generator variables, so OPF converges.

#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"

namespace fs = std::filesystem;

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "../../"
#endif

static hacdcpf::HybridPowerSystem load_nansha() {
  const fs::path json_path =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "external_data/classical_example/nansha_full_network.json";
  REQUIRE(fs::exists(json_path));
  auto result = hacdcpf::io::try_load_json(json_path.string());
  REQUIRE(result.has_value());
  return std::move(result.value());
}

TEST_CASE("Nansha OPF: nighttime (no generators) converges via external grid cost",
          "[opf][nighttime][nansha]") {
  auto sys = load_nansha();

  // Verify precondition: nansha has no conventional generators
  INFO("Number of generators: " << sys.ac.generators.size());
  CHECK(sys.ac.generators.empty());

  // Verify precondition: external grids have cost set
  bool any_eg_cost = false;
  for (const auto& eg : sys.ac.external_grids)
    if (eg.cost_c2 > 0.0 || eg.cost_c1 > 0.0) any_eg_cost = true;
  INFO("Number of external grids with cost: checked");
  REQUIRE(any_eg_cost);

  // The loaded JSON already has no ac.generators — this is the nighttime condition.
  // External grids with cost_c2 > 0 must be promoted to OPF variables for convergence.

  hacdcpf::opf::ACOPFOptions opts;
  opts.allow_fallback = false; // strict: do not fall back to PF

  const auto result = hacdcpf::solve_ac_opf(sys, opts);

  INFO("OPF status: " << result.status);
  INFO("OPF iterations: " << result.iterations);
  for (const auto& h : result.infeasibility_hints)
    INFO("Hint: " << h);

  CHECK(result.converged);
}

TEST_CASE("Nansha OPF: daytime (PV present) also converges",
          "[opf][daytime][nansha]") {
  auto sys = load_nansha();

  // Leave renewable gen power as-is from the JSON (daytime representative)
  // Just ensure the base case runs — this was working before the fix too.

  hacdcpf::opf::ACOPFOptions opts;
  opts.allow_fallback = false;

  const auto result = hacdcpf::solve_ac_opf(sys, opts);

  INFO("OPF status: " << result.status);
  CHECK(result.converged);
}
