// Tests for DC island voltage-reference planning and the "regulate + warn"
// behavior of a PQ VSC converter feeding a reference-less DC island.
//
// Scenario (data/simple_case.json): an AC SLACK bus (external grid) coupled
// through one VSC converter (control_mode AC_PQ / PQ_MODE) to a single DC bus
// (DC_P) carrying a DC PV array.  In PQ mode the converter does not regulate
// Vdc, so the DC island has no voltage reference.  The solver must auto-promote
// the converter to VDC_Q, balance the island, and warn.

#include <algorithm>
#include <cmath>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

#ifndef HACDCPF_TEST_DATA_DIR
#define HACDCPF_TEST_DATA_DIR "../../data"
#endif

namespace {

bool any_warning_contains(const hacdcpf::PowerFlowResult& pf, const std::string& needle) {
  return std::any_of(pf.diagnostics.warnings.begin(), pf.diagnostics.warnings.end(),
                     [&](const std::string& w) { return w.find(needle) != std::string::npos; });
}

}  // namespace

TEST_CASE("DC island with PQ converter auto-promotes to regulate Vdc and warns",
          "[hybrid][converter][dc_island]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);

  REQUIRE_FALSE(sys.dc.buses.empty());
  REQUIRE_FALSE(sys.vsc_converters.empty());
  // Precondition: DC bus is a power bus (no voltage reference) and the converter
  // is in PQ mode — exactly the configuration that used to silently mis-balance.
  REQUIRE(sys.dc.buses.front().bus_type == hacdcpf::DCBusType::DC_P);
  REQUIRE(sys.vsc_converters.front().control_mode == hacdcpf::ConverterMode::PQ_MODE);

  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys);

  SECTION("solve converges by letting the converter regulate Vdc") {
    REQUIRE(pf.converged);
    REQUIRE_FALSE(pf.vdc.empty());
    // The promoted converter forms the Vdc reference with a stiff gain, so it
    // holds Vdc close to its setpoint and well inside the bus limits instead of
    // letting it drift up to absorb the PV surplus.
    REQUIRE(pf.vdc.front() >= sys.dc.buses.front().vmin_pu - 1e-6);
    REQUIRE(pf.vdc.front() <= sys.dc.buses.front().vmax_pu + 1e-6);
    REQUIRE(std::abs(pf.vdc.front() - 1.0) < 0.05);
  }

  SECTION("the promotion is recorded and warned") {
    REQUIRE(pf.diagnostics.promoted_vsc_indices.size() == 1);
    REQUIRE(pf.diagnostics.promoted_vsc_indices.front() == 0);
    REQUIRE(any_warning_contains(pf, "auto-promoted"));
  }

  SECTION("the stiff auto-promotion keeps Vdc within limits") {
    // Unlike the old weak-droop behavior, the promoted converter no longer
    // drives Vdc above the bus vmax, so no excursion warning is expected.
    REQUIRE(pf.vdc.front() <= sys.dc.buses.front().vmax_pu + 1e-6);
    REQUIRE_FALSE(any_warning_contains(pf, "exceeds vmax"));
  }
}

TEST_CASE("A genuine DC_V reference is pinned and the converter is not promoted",
          "[hybrid][converter][dc_island]") {
  const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/simple_case.json";
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::load_json(path);
  REQUIRE_FALSE(sys.dc.buses.empty());

  // Mark the DC bus as a true voltage reference: now the island has a reference,
  // so the PQ converter must be left alone (Vdc pinned, no auto-promotion).
  sys.dc.buses.front().bus_type = hacdcpf::DCBusType::DC_V;

  const hacdcpf::PowerFlowResult pf = hacdcpf::solve_power_flow(sys);

  REQUIRE(pf.converged);
  REQUIRE(pf.diagnostics.promoted_vsc_indices.empty());
  REQUIRE_FALSE(any_warning_contains(pf, "auto-promoted"));
  REQUIRE_FALSE(pf.vdc.empty());
  // Pinned DC_V bus holds its nominal voltage.
  REQUIRE(std::abs(pf.vdc.front() - 1.0) < 1e-3);
}
