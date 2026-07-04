#include <algorithm>
#include <map>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"

using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

const DynamicModelDescriptor& require_model(const std::string& name) {
  auto m = find_dynamic_model(name);
  REQUIRE(m.has_value());
  static thread_local DynamicModelDescriptor held;
  held = *m;
  return held;
}

// Build a 2-bus system with one non-slack machine carrying the given control
// blocks, run a short transient, and return the results so we can confirm the
// catalog parameter keys actually reach the device.
DynamicResults run_with_blocks(std::map<std::string, std::map<std::string, double>> blocks) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.02;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PV; b2.vm_pu = 1.0;
  sys.ac.buses = {b1, b2};
  ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2; br.r_pu = 0.01; br.x_pu = 0.05; br.tap = 1.0;
  sys.ac.branches = {br};
  Generator gs; gs.index = 1; gs.bus = 1; gs.is_slack = true; gs.vg_pu = 1.02; gs.pg_mw = 40;
  gs.xdpp_pu = 0.2; gs.qmax_mvar = 300; gs.qmin_mvar = -300;
  Generator g2; g2.index = 2; g2.bus = 2; g2.vg_pu = 1.0; g2.pg_mw = 50; g2.qg_mvar = 10;
  g2.xdpp_pu = 0.2; g2.inertia_h = 3.0; g2.pmax_mw = 200; g2.qmax_mvar = 300; g2.qmin_mvar = -300;
  g2.dynamic_model.model_name = "ClassicalMachine";
  for (auto& [type, params] : blocks) {
    hacdcpf::DynamicModelComponentProfile c;
    c.type = type;
    c.model = type == "governor" ? "TGOV1" : type == "exciter" ? "SEXS" : "PSS1A";
    c.parameters = params;
    g2.dynamic_model.components.push_back(std::move(c));
  }
  sys.ac.generators = {gs, g2};
  Load ld; ld.index = 1; ld.bus = 2; ld.p_mw = 30; ld.q_mvar = 10;
  sys.ac.loads = {ld};

  DynamicSolverOptions opt;
  opt.t_end_s = 0.1;
  opt.dt_s = 0.01;
  return hacdcpf::dynamics::run_transient_simulation(sys, opt);
}

double device_value(const DynamicResults& r, const std::string& type, const std::string& key) {
  REQUIRE_FALSE(r.snapshots.empty());
  for (const auto& d : r.snapshots.front().device_outputs) {
    if (d.type == type) {
      auto it = d.values.find(key);
      if (it != d.values.end()) return it->second;
    }
  }
  FAIL("device output not found: " + type + "/" + key);
  return 0.0;
}

}  // namespace

TEST_CASE("Dynamic model catalog covers the wired models", "[dynamics][catalog]") {
  const auto& catalog = dynamic_model_catalog();
  REQUIRE(catalog.size() >= 15);
  for (const char* name :
       {"GENROU", "ClassicalMachine", "TGOV1", "IEEEG1", "SEXS", "IEEET1", "PSS1A",
        "REGC_REEC_GFL_Subset", "GridFormingNortonDroop", "ReducedOrderPLL", "KauraPLL",
        "FixedFrequency", "FirstOrderDCDCConverter", "BatterySOCFirstOrder", "ZIP"}) {
    INFO("model " << name);
    CHECK(find_dynamic_model(name).has_value());
  }
}

TEST_CASE("Every block-composition model resolves in the catalog", "[dynamics][catalog]") {
  for (const auto& comp : dynamic_block_composition()) {
    for (const auto& slot : comp.slots) {
      for (const auto& model : slot.model_names) {
        INFO(comp.canvas_type << "/" << slot.slot << " -> " << model);
        CHECK(find_dynamic_model(model).has_value());
      }
      if (!slot.default_model.empty() && slot.default_model != "None") {
        INFO(comp.canvas_type << "/" << slot.slot << " default " << slot.default_model);
        CHECK(find_dynamic_model(slot.default_model).has_value());
      }
    }
  }
}

TEST_CASE("Catalog parameter bounds are self-consistent", "[dynamics][catalog]") {
  for (const auto& model : dynamic_model_catalog()) {
    for (const auto& p : model.parameters) {
      INFO(model.model_name << "/" << p.key);
      if (p.min_value && p.max_value) {
        CHECK(*p.min_value <= *p.max_value);
        CHECK(p.default_value >= *p.min_value);
        CHECK(p.default_value <= *p.max_value);
      }
      CHECK_FALSE(p.key.empty());
      CHECK_FALSE(p.label.empty());
    }
  }
}

TEST_CASE("Governor / exciter / PSS catalog keys are consumed by the builder",
          "[dynamics][catalog]") {
  // Set distinctive values via the catalog's canonical keys and confirm they
  // are echoed by the built device — proving the keys are not dead.
  const DynamicResults r = run_with_blocks({
      {"governor", {{"R", 0.037}}},
      {"exciter", {{"Ka", 42.0}}},
      {"pss", {{"Ks", 7.5}}},
  });
  REQUIRE(r.success);
  CHECK(device_value(r, "Governor", "droop_r") == Catch::Approx(0.037));
  CHECK(device_value(r, "Exciter", "ka") == Catch::Approx(42.0));
  CHECK(device_value(r, "PSS", "ks") == Catch::Approx(7.5));
}
