/// @file test_typical_parameters.cpp
/// @brief Tests for the opt-in typical parameter library.

#include <algorithm>
#include <cmath>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/typical_parameters.hpp"
#include "hacdcpf/validation/validate_system.hpp"

using namespace hacdcpf;
using Catch::Matchers::WithinAbs;

namespace {

HybridPowerSystem make_parameter_contract_system() {
  HybridPowerSystem sys;
  sys.base_mva = 0.0;
  sys.ac.base_mva = 0.0;
  sys.dc.base_mva = 0.0;
  sys.ac.freq_hz = 0.0;

  ACBus ac1;
  ac1.index = 1;
  ac1.bus_type = BusType::SLACK;
  ac1.base_kv = 0.0;
  ac1.vm_pu = 0.0;
  ac1.vmin_pu = 0.0;
  ac1.vmax_pu = 0.0;
  ACBus ac2 = ac1;
  ac2.index = 2;
  ac2.bus_type = BusType::PQ;
  sys.ac.buses = {ac1, ac2};

  ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  ACBranch transformer_branch = line;
  transformer_branch.index = 2;
  transformer_branch.sn_mva = 1.0;
  ACBranch overhead_line = line;
  overhead_line.index = 3;
  overhead_line.line_type = "OVERHEAD";
  ACBranch cable_line = line;
  cable_line.index = 4;
  cable_line.line_type = "CABLE";
  sys.ac.branches = {line, transformer_branch, overhead_line, cable_line};

  Transformer2W transformer;
  transformer.index = 1;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  sys.ac.transformers_2w = {transformer};

  Switch sw;
  sw.index = 1;
  sw.bus_from = 1;
  sw.bus_to = 2;
  sys.ac.switches = {sw};

  Generator generator;
  generator.index = 1;
  generator.bus = 1;
  sys.ac.generators = {generator};

  StaticGenerator static_generator;
  static_generator.index = 1;
  static_generator.bus = 1;
  sys.ac.static_generators = {static_generator};

  RenewableGen renewable;
  renewable.index = 1;
  renewable.bus = 1;
  sys.ac.renewable_gens = {renewable};

  PVSystem pv;
  pv.index = 1;
  pv.bus = 1;
  sys.ac.pv_systems = {pv};

  DCBus dc1;
  dc1.index = 1;
  dc1.base_kv = 0.0;
  DCBus dc2 = dc1;
  dc2.index = 2;
  sys.dc.buses = {dc1, dc2};
  DCBranch dc_line;
  dc_line.index = 1;
  dc_line.from_bus = 1;
  dc_line.to_bus = 2;
  sys.dc.branches = {dc_line};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 1;
  vsc.bus_dc = 1;
  vsc.p_rated_mw = 0.0;
  vsc.eta = 0.0;
  vsc.r_sc_pu = 0.0;
  vsc.x_sc_pu = 0.0;
  vsc.i_max_pu = 0.0;
  sys.vsc_converters = {vsc};

  DCDCConverter dcdc;
  dcdc.index = 1;
  dcdc.bus_in = 1;
  dcdc.bus_out = 2;
  dcdc.eta = 0.0;
  sys.dc.dcdc_converters = {dcdc};

  Storage storage;
  storage.index = 1;
  storage.bus = 1;
  storage.eta_charge = 0.0;
  storage.eta_discharge = 0.0;
  sys.ac.storage = {storage};

  Microgrid microgrid;
  microgrid.index = 1;
  microgrid.pcc_bus = 1;
  sys.microgrids = {microgrid};
  return sys;
}

double representative_parameter_value(const HybridPowerSystem& sys,
                                      const std::string& id) {
  if (id == "system.base_mva") return sys.base_mva;
  if (id == "system.frequency_hz") return sys.ac.freq_hz;
  if (id == "ac_bus.base_kv") return sys.ac.buses[0].base_kv;
  if (id == "ac_bus.vm_pu") return sys.ac.buses[0].vm_pu;
  if (id == "ac_bus.vmin_pu") return sys.ac.buses[0].vmin_pu;
  if (id == "ac_bus.vmax_pu") return sys.ac.buses[0].vmax_pu;
  if (id == "ac_branch.r_pu") return sys.ac.branches[0].r_pu;
  if (id == "ac_branch.x_pu") return sys.ac.branches[0].x_pu;
  if (id == "ac_branch.rate_a_mva") return sys.ac.branches[0].rate_a_mva;
  if (id == "transformer.r_pu") return sys.ac.branches[1].r_pu;
  if (id == "transformer.x_pu") return sys.ac.branches[1].x_pu;
  if (id == "transformer.sn_mva") return sys.ac.transformers_2w[0].sn_mva;
  if (id == "transformer.vk_percent") return sys.ac.transformers_2w[0].vk_percent;
  if (id == "transformer.vkr_percent") return sys.ac.transformers_2w[0].vkr_percent;
  if (id == "dc_bus.base_kv") return sys.dc.buses[0].base_kv;
  if (id == "dc_branch.r_pu") return sys.dc.branches[0].r_pu;
  if (id == "vsc.p_rated_mw") return sys.vsc_converters[0].p_rated_mw;
  if (id == "vsc.eta") return sys.vsc_converters[0].eta;
  if (id == "vsc.r_sc_pu") return sys.vsc_converters[0].r_sc_pu;
  if (id == "vsc.x_sc_pu") return sys.vsc_converters[0].x_sc_pu;
  if (id == "vsc.i_max_pu") return sys.vsc_converters[0].i_max_pu;
  if (id == "dcdc.eta") return sys.dc.dcdc_converters[0].eta;
  if (id == "storage.eta_charge") return sys.ac.storage[0].eta_charge;
  if (id == "storage.eta_discharge") return sys.ac.storage[0].eta_discharge;
  if (id == "reliability.ac_overhead.failure_rate") return sys.ac.branches[2].failure_rate;
  if (id == "reliability.ac_overhead.mttr_hr") return sys.ac.branches[2].mttr_hr;
  if (id == "reliability.ac_cable.failure_rate") return sys.ac.branches[3].failure_rate;
  if (id == "reliability.ac_cable.mttr_hr") return sys.ac.branches[3].mttr_hr;
  if (id == "reliability.ac_branch.failure_rate") return sys.ac.branches[0].failure_rate;
  if (id == "reliability.ac_branch.mttr_hr") return sys.ac.branches[0].mttr_hr;
  if (id == "reliability.transformer.mtbf_hours") return sys.ac.transformers_2w[0].mtbf_hours;
  if (id == "reliability.transformer.mttr_hours") return sys.ac.transformers_2w[0].mttr_hours;
  if (id == "reliability.switch.mtbf_hours") return sys.ac.switches[0].mtbf_hours;
  if (id == "reliability.switch.mttr_hours") return sys.ac.switches[0].mttr_hours;
  if (id == "reliability.generator.forced_outage_rate") return sys.ac.generators[0].forced_outage_rate;
  if (id == "reliability.generator.mttr_hr") return sys.ac.generators[0].mttr_hr;
  if (id == "reliability.static_generator.mtbf_hours") return sys.ac.static_generators[0].mtbf_hours;
  if (id == "reliability.static_generator.mttr_hours") return sys.ac.static_generators[0].mttr_hours;
  if (id == "reliability.renewable.mtbf_hours") return sys.ac.renewable_gens[0].mtbf_hours;
  if (id == "reliability.renewable.mttr_hours") return sys.ac.renewable_gens[0].mttr_hours;
  if (id == "reliability.pv.mtbf_hours") return sys.ac.pv_systems[0].mtbf_hours;
  if (id == "reliability.pv.mttr_hours") return sys.ac.pv_systems[0].mttr_hours;
  if (id == "reliability.storage.forced_outage_rate") return sys.ac.storage[0].forced_outage_rate;
  if (id == "reliability.storage.mttr_hr") return sys.ac.storage[0].mttr_hr;
  if (id == "reliability.vsc.forced_outage_rate") return sys.vsc_converters[0].forced_outage_rate;
  if (id == "reliability.vsc.mttr_hr") return sys.vsc_converters[0].mttr_hr;
  if (id == "reliability.dcdc.mtbf_hours") return sys.dc.dcdc_converters[0].mtbf_hours;
  if (id == "reliability.dcdc.mttr_hours") return sys.dc.dcdc_converters[0].mttr_hours;
  if (id == "reliability.dc_branch.mtbf_hours") return sys.dc.branches[0].mtbf_hours;
  if (id == "reliability.dc_branch.mttr_hours") return sys.dc.branches[0].mttr_hours;
  if (id == "reliability.microgrid.mtbf_hours") return sys.microgrids[0].mtbf_hours;
  if (id == "reliability.microgrid.mttr_hours") return sys.microgrids[0].mttr_hours;
  FAIL("No parameter-contract representative for " + id);
  return 0.0;
}

double alternate_rule_value(const StandardParameterRule& rule) {
  double candidate = rule.has_min && rule.has_max
                         ? rule.min_value + 0.37 * (rule.max_value - rule.min_value)
                         : rule.default_value * 1.17 + 0.01;
  if (std::abs(candidate - rule.default_value) < 1e-10) {
    candidate = rule.has_min && rule.has_max
                    ? rule.min_value + 0.63 * (rule.max_value - rule.min_value)
                    : rule.default_value + 0.1;
  }
  return candidate;
}

}  // namespace

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

TEST_CASE("standard parameter profiles expose editable validated metadata",
          "[model][parameter_library]") {
  const auto profiles = standard_parameter_profiles();
  REQUIRE(profiles.size() >= 2);

  auto library = make_standard_parameter_library("distribution_50hz");
  CHECK(library.profile_id == "distribution_50hz");
  CHECK(library.rules.size() >= 20);
  REQUIRE(library.find("transformer.x_pu") != nullptr);
  CHECK(library.find("transformer.x_pu")->unit == "pu");
  CHECK_FALSE(library.find("transformer.x_pu")->source.empty());
  CHECK(validate_standard_parameter_library(library).ok());

  auto* rule = library.find("ac_branch.r_pu");
  REQUIRE(rule != nullptr);
  rule->default_value = rule->max_value + 1.0;
  const auto invalid = validate_standard_parameter_library(library);
  CHECK_FALSE(invalid.ok());
  CHECK(invalid.error_count() == 1);
}

TEST_CASE("standard parameter library applies edited defaults only when requested",
          "[model][parameter_library][apply]") {
  HybridPowerSystem sys;
  sys.base_mva = 10.0;
  sys.ac.base_mva = 10.0;
  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  ACBus b2;
  b2.index = 2;
  sys.ac.buses = {b1, b2};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  sys.ac.branches = {branch};

  auto library = make_standard_parameter_library();
  REQUIRE(library.find("ac_branch.r_pu") != nullptr);
  REQUIRE(library.find("ac_branch.x_pu") != nullptr);
  library.find("ac_branch.r_pu")->default_value = 0.025;
  library.find("ac_branch.x_pu")->default_value = 0.055;

  CHECK_THAT(sys.ac.branches.front().r_pu, WithinAbs(0.0, 1e-12));
  const auto applied = apply_standard_parameter_library(sys, library);
  CHECK(applied.fields_changed > 0);
  CHECK_THAT(sys.ac.branches.front().r_pu, WithinAbs(0.025, 1e-12));
  CHECK_THAT(sys.ac.branches.front().x_pu, WithinAbs(0.055, 1e-12));
}

TEST_CASE("standard parameter reliability completion preserves authored data",
          "[model][parameter_library][reliability][missing_only]") {
  auto sys = make_parameter_contract_system();
  sys.ac.branches[0].failure_rate = 0.123;
  sys.ac.branches[0].mttr_hr = 17.0;
  sys.ac.generators[0].forced_outage_rate = 0.031;
  sys.ac.generators[0].mttr_hr = 29.0;
  sys.microgrids[0].mtbf_hours = 54321.0;
  sys.microgrids[0].mttr_hours = 13.0;

  const auto applied = apply_standard_parameter_library(
      sys, make_standard_parameter_library());
  CHECK(applied.fields_changed > 0);
  CHECK_THAT(sys.ac.branches[0].failure_rate, WithinAbs(0.123, 1e-12));
  CHECK_THAT(sys.ac.branches[0].mttr_hr, WithinAbs(17.0, 1e-12));
  CHECK_THAT(sys.ac.generators[0].forced_outage_rate,
             WithinAbs(0.031, 1e-12));
  CHECK_THAT(sys.ac.generators[0].mttr_hr, WithinAbs(29.0, 1e-12));
  CHECK_THAT(sys.microgrids[0].mtbf_hours, WithinAbs(54321.0, 1e-12));
  CHECK_THAT(sys.microgrids[0].mttr_hours, WithinAbs(13.0, 1e-12));
}

TEST_CASE("every registered standard parameter has an effective numerical override",
          "[model][parameter_library][contract][sensitivity]") {
  const auto baseline = make_standard_parameter_library();
  REQUIRE(baseline.rules.size() >= 52);

  for (const auto& baseline_rule : baseline.rules) {
    DYNAMIC_SECTION(baseline_rule.id) {
      auto library = baseline;
      auto* rule = library.find(baseline_rule.id);
      REQUIRE(rule != nullptr);
      const double alternate = alternate_rule_value(*rule);
      rule->default_value = alternate;
      REQUIRE(validate_standard_parameter_library(library).ok());

      auto sys = make_parameter_contract_system();
      const auto applied = apply_standard_parameter_library(sys, library);
      CAPTURE(baseline_rule.id, baseline_rule.default_value, alternate,
              applied.applied_rule_ids);
      CHECK(std::find(applied.applied_rule_ids.begin(),
                      applied.applied_rule_ids.end(),
                      baseline_rule.id) != applied.applied_rule_ids.end());
      CHECK_THAT(representative_parameter_value(sys, baseline_rule.id),
                 WithinAbs(alternate, 1e-12));
    }
  }
}

TEST_CASE("parameter validation rejects zero-impedance transformer imports",
          "[model][parameter_library][validation][lvnt]") {
  HybridPowerSystem sys;
  sys.name = "old LVNT import";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  ACBus source;
  source.index = 1;
  source.bus_type = BusType::SLACK;
  source.base_kv = 11.0;
  ACBus load;
  load.index = 2;
  load.base_kv = 0.4;
  sys.ac.buses = {source, load};
  ACBranch transformer;
  transformer.index = 1;
  transformer.name = "LVNT transformer";
  transformer.from_bus = 1;
  transformer.to_bus = 2;
  transformer.sn_mva = 0.5;
  transformer.vn_hv_kv = 11.0;
  transformer.vn_lv_kv = 0.4;
  sys.ac.branches = {transformer};

  ThreePhaseACSystem phase;
  phase.base_mva = 100.0;
  phase.base_freq_hz = 50.0;
  ThreePhaseACBus phase_source;
  phase_source.index = 1;
  phase_source.bus_type = BusType::SLACK;
  phase_source.base_kv = 11.0;
  ThreePhaseACBus phase_load;
  phase_load.index = 2;
  phase_load.base_kv = 0.4;
  phase.buses = {phase_source, phase_load};
  ThreePhaseACLine phase_line;
  phase_line.index = 1;
  phase_line.from_bus = 1;
  phase_line.to_bus = 2;
  phase.lines = {phase_line};
  sys.three_phase_ac = phase;

  const auto report = validate_component_parameters(sys);
  CHECK_FALSE(report.ok());
  const auto found = std::find_if(
      report.diagnostics.begin(), report.diagnostics.end(), [](const auto& item) {
        return item.code == "transformer_zero_series_impedance";
      });
  CHECK(found != report.diagnostics.end());
  const auto phase_found = std::find_if(
      report.diagnostics.begin(), report.diagnostics.end(), [](const auto& item) {
        return item.code == "three_phase_line_zero_series_impedance";
      });
  CHECK(phase_found != report.diagnostics.end());
}

TEST_CASE("parameter validation reports OPF decision-bound risks",
          "[model][parameter_library][validation][opf_bounds]") {
  HybridPowerSystem sys;
  sys.name = "invalid OPF bounds";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  MobileStorage mobile;
  mobile.index = 4;
  mobile.name = "Boundary BESS";
  mobile.bus = 1;
  mobile.pmin_mw = -0.5;
  mobile.pmax_mw = 0.5;
  mobile.p_mw = 0.8;
  mobile.qmin_mvar = 0.0;
  mobile.qmax_mvar = 0.0;
  mobile.q_mvar = 0.0;
  sys.mobile_storage.push_back(mobile);

  Generator generator;
  generator.index = 2;
  generator.name = "Reversed generator";
  generator.pmin_mw = 2.0;
  generator.pmax_mw = 1.0;
  generator.qmin_mvar = -1.0;
  generator.qmax_mvar = 1.0;
  sys.ac.generators.push_back(generator);

  const auto report = validate_component_parameters(sys);
  CHECK_FALSE(report.ok());
  const auto has_code = [&](const std::string& code) {
    return std::any_of(report.diagnostics.begin(), report.diagnostics.end(),
                       [&](const auto& item) { return item.code == code; });
  };
  CHECK(has_code("opf_decision_bounds_reversed"));
  CHECK(has_code("opf_decision_bounds_zero_width"));
  CHECK(has_code("opf_initial_value_outside_bounds"));
}

TEST_CASE("design handbook completion previews and applies CIM line parameters",
          "[model][parameter_library][design_handbook]") {
  HybridPowerSystem sys;
  sys.base_mva = 1.0;
  sys.ac.base_mva = 1.0;
  sys.ac.freq_hz = 50.0;
  ACBus b1;
  b1.index = 1;
  b1.base_kv = 0.4;
  b1.bus_type = BusType::SLACK;
  ACBus b2 = b1;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  sys.ac.buses = {b1, b2};

  ACBranch branch;
  branch.index = 7;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.name = "ZRC feeder";
  branch.length_km = 0.1;
  branch.r_ohm_per_km = 18.5 / 240.0;
  branch.x_ohm_per_km = 0.08;
  branch.r_pu = branch.r_ohm_per_km * branch.length_km / 0.16;
  branch.x_pu = branch.x_ohm_per_km * branch.length_km / 0.16;
  branch.conductor_model = "ZRC-YJV22-";
  branch.cross_section_mm2 = 240.0;
  branch.line_type = "低压电缆段";
  branch.parameter_source = "cim_model_cross_section_estimate";
  sys.ac.branches.push_back(branch);

  ThreePhaseACSystem phase;
  phase.base_mva = 1.0;
  phase.buses.resize(2);
  phase.buses[0].index = 1;
  phase.buses[0].base_kv = 0.4;
  phase.buses[1].index = 2;
  phase.buses[1].base_kv = 0.4;
  ThreePhaseACLine line;
  line.index = 7;
  line.from_bus = 1;
  line.to_bus = 2;
  line.length_km = 0.1;
  line.r1_pu = branch.r_pu;
  line.x1_pu = branch.x_pu;
  line.r0_pu = 3.0 * branch.r_pu;
  line.x0_pu = 3.0 * branch.x_pu;
  phase.lines.push_back(line);
  sys.three_phase_ac = phase;

  const double original_r = sys.ac.branches.front().r_ohm_per_km;
  const auto preview = complete_design_handbook_parameters(sys);
  REQUIRE(preview.candidates == 1);
  REQUIRE(preview.suggestions.size() == 1);
  CHECK(preview.fields_changed == 0);
  CHECK(sys.ac.branches.front().r_ohm_per_km == original_r);
  const auto& suggestion = preview.suggestions.front();
  CHECK(suggestion.conductor_material == "copper");
  CHECK(suggestion.insulation == "XLPE");
  CHECK_THAT(suggestion.r20_ohm_per_km, WithinAbs(0.0754, 1e-12));
  CHECK_THAT(suggestion.new_r_ohm_per_km, WithinAbs(0.09614254, 1e-8));
  CHECK_THAT(suggestion.new_x_ohm_per_km, WithinAbs(0.08, 1e-12));

  DesignHandbookCompletionOptions options;
  options.apply = true;
  const auto applied = complete_design_handbook_parameters(sys, options);
  CHECK(applied.fields_changed > 0);
  CHECK(applied.suggestions.front().applied);
  CHECK_THAT(sys.ac.branches.front().r_ohm_per_km,
             WithinAbs(0.09614254, 1e-8));
  REQUIRE(sys.three_phase_ac.has_value());
  CHECK_THAT(sys.three_phase_ac->lines.front().r1_pu,
             WithinAbs(sys.ac.branches.front().r_pu, 1e-12));
  CHECK_THAT(sys.three_phase_ac->lines.front().r0_pu,
             WithinAbs(3.0 * sys.ac.branches.front().r_pu, 1e-12));

  const auto restored = io::from_json(io::to_json(sys));
  REQUIRE(restored.ac.branches.size() == 1);
  CHECK(restored.ac.branches.front().conductor_model == "ZRC-YJV22-");
  CHECK(restored.ac.branches.front().cross_section_mm2 == 240.0);
  CHECK_FALSE(restored.ac.branches.front().cross_section_inferred);
  CHECK(restored.ac.branches.front().parameter_source ==
        "design_handbook_gbt3956_schneider_eig");
  CHECK_THAT(restored.ac.branches.front().r_ohm_per_km,
             WithinAbs(sys.ac.branches.front().r_ohm_per_km, 1e-12));
}

TEST_CASE("design handbook reports model and PSR geometry conflicts",
          "[model][parameter_library][design_handbook][conflict]") {
  HybridPowerSystem sys;
  sys.base_mva = 1.0;
  sys.ac.base_mva = 1.0;
  ACBus b1;
  b1.index = 1;
  b1.base_kv = 0.4;
  ACBus b2 = b1;
  b2.index = 2;
  sys.ac.buses = {b1, b2};
  ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.length_km = 0.05;
  branch.conductor_model = "BVV";
  branch.cross_section_mm2 = 120.0;
  branch.line_type = "低压架空线段";
  branch.parameter_source = "cim_model_cross_section_estimate";
  branch.parameters_inferred = true;
  sys.ac.branches.push_back(branch);

  const auto preview = complete_design_handbook_parameters(sys);
  REQUIRE(preview.suggestions.size() == 1);
  const auto& row = preview.suggestions.front();
  CHECK(row.conductor_material == "copper");
  CHECK(row.insulation == "PVC");
  CHECK(row.model_type_conflict);
  CHECK_FALSE(row.cross_section_inferred);
  CHECK(row.confidence == "medium");
  CHECK_THAT(row.new_r_ohm_per_km, WithinAbs(0.1830645, 1e-7));
  CHECK_THAT(row.new_x_ohm_per_km, WithinAbs(0.35, 1e-12));
  CHECK_FALSE(preview.warnings.empty());

  auto inferred = sys;
  inferred.ac.branches.front().conductor_model.clear();
  inferred.ac.branches.front().cross_section_mm2 = 35.0;
  inferred.ac.branches.front().cross_section_inferred = true;
  const auto inferred_preview = complete_design_handbook_parameters(inferred);
  REQUIRE(inferred_preview.suggestions.size() == 1);
  CHECK(inferred_preview.suggestions.front().cross_section_inferred);
  CHECK(inferred_preview.suggestions.front().confidence == "low");

  inferred.ac.branches.front().cross_section_inferred = false;
  const auto missing_model_preview =
      complete_design_handbook_parameters(inferred);
  REQUIRE(missing_model_preview.suggestions.size() == 1);
  CHECK_FALSE(missing_model_preview.suggestions.front().cross_section_inferred);
  CHECK(missing_model_preview.suggestions.front().confidence == "medium");
}

TEST_CASE("design handbook infers protection, sectionalizer, fuse, and tie bindings",
          "[model][parameter_library][design_handbook][switch_binding]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 10.0;
  for (int index = 1; index <= 10; ++index) {
    ACBus bus;
    bus.index = index;
    bus.base_kv = 10.0;
    bus.bus_type = (index == 1 || index == 6) ? BusType::SLACK : BusType::PQ;
    sys.ac.buses.push_back(bus);
  }
  ExternalGrid grid1;
  grid1.index = 1;
  grid1.bus = 1;
  ExternalGrid grid2 = grid1;
  grid2.index = 2;
  grid2.bus = 6;
  sys.ac.external_grids = {grid1, grid2};

  ACBranch feeder;
  feeder.index = 7;
  feeder.from_bus = 2;
  feeder.to_bus = 3;
  feeder.r_pu = 0.01;
  feeder.x_pu = 0.02;
  ACBranch lateral = feeder;
  lateral.index = 8;
  lateral.from_bus = 4;
  lateral.to_bus = 5;
  ACBranch isolated_section = feeder;
  isolated_section.index = 9;
  isolated_section.from_bus = 9;
  isolated_section.to_bus = 10;
  sys.ac.branches = {feeder, lateral, isolated_section};

  Transformer2W transformer;
  transformer.index = 3;
  transformer.hv_bus = 7;
  transformer.lv_bus = 8;
  transformer.sn_mva = 1.0;
  sys.ac.transformers_2w = {transformer};

  Switch breaker;
  breaker.index = 10;
  breaker.name = "feeder breaker";
  breaker.bus_from = 1;
  breaker.bus_to = 2;
  breaker.switch_type = SwitchType::CircuitBreaker;
  breaker.closed = true;
  Switch sectionalizer;
  sectionalizer.index = 11;
  sectionalizer.name = "sectionalizer";
  sectionalizer.bus_from = 3;
  sectionalizer.bus_to = 4;
  sectionalizer.switch_type = SwitchType::Sectionalizer;
  sectionalizer.closed = true;
  Switch fuse;
  fuse.index = 12;
  fuse.name = "dropout fuse";
  fuse.bus_from = 5;
  fuse.bus_to = 7;
  fuse.switch_type = SwitchType::Fuse;
  fuse.closed = true;
  Switch disconnector;
  disconnector.index = 14;
  disconnector.name = "maintenance disconnector";
  disconnector.bus_from = 5;
  disconnector.bus_to = 9;
  disconnector.switch_type = SwitchType::Disconnector;
  disconnector.closed = true;
  Switch tie;
  tie.index = 13;
  tie.name = "联络开关";
  tie.bus_from = 3;
  tie.bus_to = 6;
  tie.switch_type = SwitchType::LoadBreakSwitch;
  tie.closed = false;
  tie.normal_closed = false;
  tie.normal_state_explicit = true;
  tie.role = SwitchRole::Tie;
  sys.ac.switches = {breaker, sectionalizer, fuse, disconnector, tie};

  const auto preview = complete_design_handbook_parameters(sys);
  CHECK(preview.switches_scanned == 5);
  CHECK(preview.switch_binding_candidates == 5);
  CHECK(preview.switch_binding_fields_changed == 0);
  CHECK(sys.ac.switches[0].controlled_branch_index == -1);

  DesignHandbookCompletionOptions options;
  options.apply = true;
  const auto applied = complete_design_handbook_parameters(sys, options);
  CHECK(applied.switch_binding_fields_changed > 0);
  CHECK(sys.ac.switches[0].role == SwitchRole::Protection);
  CHECK(sys.ac.switches[0].controlled_element_type == "ac_branch");
  CHECK(sys.ac.switches[0].controlled_branch_index == 7);
  CHECK(sys.ac.switches[0].protection_zone_id == 7);
  CHECK(sys.ac.switches[1].role == SwitchRole::Sectionalizing);
  CHECK(sys.ac.switches[1].controlled_branch_index == 8);
  CHECK(sys.ac.switches[1].upstream_protective_switch_index == 10);
  CHECK(sys.ac.switches[1].sectionalizer_protection.upstream_switch_index == 10);
  CHECK(sys.ac.switches[2].controlled_element_type == "transformer_2w");
  CHECK(sys.ac.switches[2].controlled_element_index == 3);
  CHECK(sys.ac.switches[2].controlled_branch_index == -1);
  CHECK(sys.ac.switches[3].role == SwitchRole::Isolation);
  CHECK(sys.ac.switches[3].controlled_branch_index == 9);
  CHECK(sys.ac.switches[3].upstream_protective_switch_index == 10);
  CHECK(sys.ac.switches[4].role == SwitchRole::Tie);
  CHECK(sys.ac.switches[4].controlled_element_index == -1);
  CHECK(sys.ac.switches[4].synchronization_required);

  auto authored = sys;
  authored.ac.switches[0].controlled_element_type = "ac_branch";
  authored.ac.switches[0].controlled_element_index = 8;
  authored.ac.switches[0].controlled_branch_index = 8;
  authored.ac.switches[0].binding_inferred = false;
  authored.ac.switches[0].binding_source = "manufacturer";
  complete_design_handbook_parameters(authored, options);
  CHECK(authored.ac.switches[0].controlled_branch_index == 8);
  CHECK(authored.ac.switches[0].binding_source == "manufacturer");

  authored.ac.switches[0].binding_inferred = true;
  authored.ac.switches[0].binding_source = "old_inference";
  complete_design_handbook_parameters(authored, options);
  CHECK(authored.ac.switches[0].controlled_branch_index == 7);
  CHECK(authored.ac.switches[0].binding_source ==
        "design_handbook_topology_inference_v1");
}

TEST_CASE("design handbook finds upstream protection inside a source-less island",
          "[model][parameter_library][design_handbook][switch_binding]") {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 10.0;
  for (int index = 0; index <= 4; ++index) {
    ACBus bus;
    bus.index = index;
    bus.base_kv = 10.0;
    bus.bus_type = index == 0 ? BusType::SLACK : BusType::PQ;
    sys.ac.buses.push_back(bus);
  }
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 0;
  sys.ac.external_grids = {grid};

  ACBranch branch;
  branch.index = 8;
  branch.from_bus = 3;
  branch.to_bus = 4;
  branch.r_pu = 0.01;
  branch.x_pu = 0.02;
  sys.ac.branches = {branch};

  Switch breaker;
  breaker.index = 10;
  breaker.bus_from = 1;
  breaker.bus_to = 2;
  breaker.switch_type = SwitchType::CircuitBreaker;
  Switch disconnector;
  disconnector.index = 11;
  disconnector.bus_from = 2;
  disconnector.bus_to = 3;
  disconnector.switch_type = SwitchType::Disconnector;
  sys.ac.switches = {breaker, disconnector};

  DesignHandbookCompletionOptions options;
  options.apply = true;
  complete_design_handbook_parameters(sys, options);
  CHECK(sys.ac.switches[1].controlled_branch_index == 8);
  CHECK(sys.ac.switches[1].upstream_protective_switch_index == 10);
}
