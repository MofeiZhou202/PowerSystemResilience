/// @file test_typical_parameters.cpp
/// @brief Tests for the opt-in typical parameter library.

#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/typical_parameters.hpp"
#include "hacdcpf/validation/validate_system.hpp"

using namespace hacdcpf;
using Catch::Matchers::WithinAbs;

TEST_CASE("apply_typical_parameters fills sparse hybrid AC/DC model",
          "[model][typical_parameters]") {
  HybridPowerSystem sys;
  sys.base_mva = 0.0;
  sys.ac.base_mva = 0.0;
  sys.dc.base_mva = 0.0;
  sys.ac.freq_hz = 0.0;

  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  slack.vm_pu = 0.0;
  slack.base_kv = 0.0;
  slack.vmin_pu = 0.0;
  slack.vmax_pu = 0.0;

  ACBus load_bus;
  load_bus.index = 2;
  load_bus.bus_type = BusType::PQ;
  load_bus.base_kv = 0.0;
  sys.ac.buses = {slack, load_bus};

  ACBranch ac_line;
  ac_line.index = 1;
  ac_line.from_bus = 1;
  ac_line.to_bus = 2;
  ac_line.tap = 0.0;
  sys.ac.branches = {ac_line};

  Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 5.0;
  load.q_mvar = 1.0;
  sys.ac.loads = {load};

  Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.is_slack = true;
  gen.pg_mw = 5.0;
  sys.ac.generators = {gen};

  DCBus dc_ref;
  dc_ref.index = 1;
  dc_ref.bus_type = DCBusType::DC_V;
  DCBus dc_load;
  dc_load.index = 2;
  dc_load.bus_type = DCBusType::DC_P;
  sys.dc.buses = {dc_ref, dc_load};

  DCBranch dc_line;
  dc_line.index = 1;
  dc_line.from_bus = 1;
  dc_line.to_bus = 2;
  sys.dc.branches = {dc_line};

  DCLoad dcl;
  dcl.index = 1;
  dcl.bus = 2;
  dcl.p_mw = 1.0;
  sys.dc.loads = {dcl};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 2;
  vsc.bus_dc = 1;
  sys.vsc_converters = {vsc};

  Storage storage;
  storage.index = 1;
  storage.bus = 2;
  sys.ac.storage = {storage};

  Transformer2W tr;
  tr.index = 1;
  tr.hv_bus = 1;
  tr.lv_bus = 2;
  sys.ac.transformers_2w = {tr};

  const auto report = apply_typical_parameters(sys);
  CHECK(report.total() > 0);
  CHECK(report.electrical_fields > 0);
  CHECK(report.operational_limit_fields > 0);

  CHECK_THAT(sys.base_mva, WithinAbs(TypicalParameters::kBaseMva, 1e-12));
  CHECK_THAT(sys.ac.freq_hz, WithinAbs(TypicalParameters::kFreqHz, 1e-12));
  CHECK(sys.ac.buses[0].base_kv > 0.0);
  CHECK(sys.ac.branches[0].r_pu > 0.0);
  CHECK(sys.ac.branches[0].x_pu > 0.0);
  CHECK(sys.ac.branches[0].rate_a_mva > 0.0);
  CHECK(sys.ac.generators[0].pmax_mw >= 5.0);
  CHECK(sys.ac.generators[0].qmax_mvar > 0.0);
  CHECK(sys.ac.generators[0].qmin_mvar < 0.0);
  CHECK(sys.dc.buses[0].base_kv > 0.0);
  CHECK(sys.dc.branches[0].r_pu > 0.0);
  CHECK(sys.vsc_converters[0].pmax_mw > 0.0);
  CHECK(sys.vsc_converters[0].pmin_mw < 0.0);
  CHECK(sys.vsc_converters[0].eta > 0.0);
  CHECK(sys.vsc_converters[0].x_sc_pu > 0.0);
  CHECK(sys.ac.storage[0].pmax_mw > 0.0);
  CHECK(sys.ac.storage[0].pmin_mw < 0.0);
  CHECK(sys.ac.storage[0].e_rated_mwh > 0.0);
  CHECK(sys.ac.transformers_2w[0].sn_mva > 0.0);
  CHECK(sys.ac.transformers_2w[0].vk_percent > 0.0);

  const auto validation = validation::validate(sys, validation::ValidationLevel::Basic);
  CHECK(validation.ok());
}

TEST_CASE("apply_typical_parameters preserves explicit nonzero data",
          "[model][typical_parameters]") {
  HybridPowerSystem sys;
  sys.base_mva = 50.0;
  sys.ac.base_mva = 50.0;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.base_kv = 35.0;
  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.base_kv = 35.0;
  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.123;
  br.x_pu = 0.456;
  br.rate_a_mva = 12.0;
  sys.ac.branches = {br};

  VSCConverter c;
  c.index = 1;
  c.bus_ac = 1;
  c.bus_dc = 1;
  c.p_rated_mw = 3.0;
  c.pmax_mw = 4.0;
  c.pmin_mw = -2.0;
  c.qmax_mvar = 1.5;
  c.qmin_mvar = -1.5;
  c.eta = 0.965;
  sys.vsc_converters = {c};

  DCBus dc;
  dc.index = 1;
  dc.bus_type = DCBusType::DC_V;
  dc.base_kv = 1.5;
  sys.dc.buses = {dc};

  TypicalParameterOptions opts;
  opts.fill_reliability = false;
  const auto report = apply_typical_parameters(sys, opts);

  CHECK(report.reliability_fields == 0);
  CHECK_THAT(sys.ac.branches[0].r_pu, WithinAbs(0.123, 1e-12));
  CHECK_THAT(sys.ac.branches[0].x_pu, WithinAbs(0.456, 1e-12));
  CHECK_THAT(sys.ac.branches[0].rate_a_mva, WithinAbs(12.0, 1e-12));
  CHECK_THAT(sys.vsc_converters[0].pmax_mw, WithinAbs(4.0, 1e-12));
  CHECK_THAT(sys.vsc_converters[0].pmin_mw, WithinAbs(-2.0, 1e-12));
  CHECK_THAT(sys.vsc_converters[0].eta, WithinAbs(0.965, 1e-12));
}

TEST_CASE("JSON import stays raw until typical parameters are requested",
          "[model][typical_parameters][json]") {
  const std::string raw = R"({
    "name": "sparse-json",
    "ac": {
      "buses": [
        {"index": 1, "bus_type": "SLACK"},
        {"index": 2, "bus_type": "PQ"}
      ],
      "branches": [
        {"index": 1, "from_bus": 1, "to_bus": 2}
      ],
      "generators": [
        {"index": 1, "bus": 1, "is_slack": true}
      ]
    }
  })";

  auto sys = io::from_json(raw);
  REQUIRE(sys.ac.branches.size() == 1);
  CHECK_THAT(sys.ac.branches[0].r_pu, WithinAbs(0.0, 1e-12));
  CHECK_THAT(sys.ac.branches[0].x_pu, WithinAbs(0.0, 1e-12));

  auto filled = with_typical_parameters(sys);
  CHECK(filled.ac.branches[0].r_pu > 0.0);
  CHECK(filled.ac.branches[0].x_pu > 0.0);
}
