#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/analysis/hosting_capacity.hpp"

using Catch::Approx;
using namespace hacdcpf;
using namespace hacdcpf::analysis;

TEST_CASE("Hosting capacity aggregates bus and component demand",
          "[hosting_capacity]") {
  HybridPowerSystem sys;

  ACBus hv;
  hv.index = 1;
  hv.base_kv = 110.0;
  hv.area = 7;
  ACBus lv;
  lv.index = 2;
  lv.base_kv = 10.0;
  ACBus load_bus = lv;
  load_bus.index = 3;
  load_bus.pd_mw = 1.0;
  sys.ac.buses = {hv, lv, load_bus};

  Transformer2W transformer;
  transformer.index = 1;
  transformer.name = "T1";
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.sn_mva = 10.0;
  transformer.cap_power_factor = 1.0;
  transformer.cap_max_reverse_load_rate = 0.8;
  transformer.cap_registered_dr_mw = 2.0;
  transformer.cap_expected_new_storage_max_mw = 2.0;
  sys.ac.transformers_2w = {transformer};

  ACBranch feeder;
  feeder.index = 1;
  feeder.from_bus = 2;
  feeder.to_bus = 3;
  sys.ac.branches = {feeder};

  Load load;
  load.index = 1;
  load.bus = 3;
  load.p_mw = 2.0;
  load.scaling = 0.5;
  sys.ac.loads = {load};

  PVSystem pv;
  pv.index = 1;
  pv.bus = 3;
  pv.p_mw = 1.0;
  sys.ac.pv_systems = {pv};

  Storage storage;
  storage.index = 1;
  storage.bus = 3;
  storage.cap_charging_strategy = "static";
  storage.cap_static_charging_mw = 0.5;
  sys.ac.storage = {storage};

  const auto result = assess_hosting_capacity(sys);

  REQUIRE(result.transformers.size() == 1);
  REQUIRE(result.areas.size() == 1);
  const auto& row = result.transformers.front();
  CHECK(row.area == 7);
  CHECK(row.canvas_type == "transformer_2w");
  CHECK(row.canvas_index == 1);
  CHECK(row.hv_bus == 1);
  CHECK(row.lv_bus == 2);
  CHECK(row.supply_bus_count == 2);
  CHECK(row.supply_load_mw == Approx(2.0));
  CHECK(row.supply_existing_dr_mw == Approx(1.0));
  CHECK(row.supply_ess_charging_mw == Approx(0.5));
  CHECK(row.hosting_min_mw == Approx(10.5));
  CHECK(row.hosting_max_mw == Approx(12.5));
  CHECK(row.accessible_grid_min_mw == Approx(9.5));
  CHECK(row.accessible_reg_min_mw == Approx(7.5));
  CHECK(row.grade == "green");

  HostingCapacityResult export_result;
  export_result.transformers = result.transformers;
  const auto serialized = hosting_capacity_result_to_json(export_result);
  CHECK(serialized["transformers"][0].value("canvas_type", "") ==
        "transformer_2w");
  CHECK(serialized["transformers"][0].value("canvas_index", -1) == 1);
}

TEST_CASE("Hosting capacity maps branch-represented transformers to Canvas branches",
          "[hosting_capacity][canvas]") {
  HybridPowerSystem sys;
  ACBus hv;
  hv.index = 10;
  hv.base_kv = 110.0;
  hv.bus_type = BusType::SLACK;
  ACBus lv;
  lv.index = 20;
  lv.base_kv = 10.0;
  lv.pd_mw = 2.0;
  sys.ac.buses = {hv, lv};

  ACBranch transformer_branch;
  transformer_branch.index = 77;
  transformer_branch.from_bus = 10;
  transformer_branch.to_bus = 20;
  transformer_branch.sn_mva = 25.0;
  transformer_branch.rate_a_mva = 25.0;
  transformer_branch.in_service = true;
  sys.ac.branches = {transformer_branch};

  const auto result = assess_hosting_capacity(sys);
  REQUIRE(result.transformers.size() == 1);
  const auto& row = result.transformers.front();
  CHECK(row.canvas_type == "ac_branch");
  CHECK(row.canvas_index == 77);
  CHECK(row.hv_bus == 10);
  CHECK(row.lv_bus == 20);
}

TEST_CASE("Hosting capacity options reject negative engineering limits",
          "[hosting_capacity]") {
  const auto options = hosting_capacity_options_from_json({
      {"default_power_factor", 1.5},
      {"default_dr_max_output_coeff", -1.0},
      {"single_transformer_beta", -0.5},
      {"delta_UH_pct", -7.0},
  });

  CHECK(options.default_power_factor == Approx(1.0));
  CHECK(options.default_dr_max_output_coeff > 0.0);
  CHECK(options.single_transformer_beta == Approx(0.0));
  CHECK(options.delta_UH_pct == Approx(0.0));
}
