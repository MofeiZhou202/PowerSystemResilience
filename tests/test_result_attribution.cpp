#include <algorithm>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/projection/result_attribution.hpp"

using Catch::Matchers::WithinAbs;
using namespace hacdcpf;
using namespace hacdcpf::projection;

namespace {

const RichComponentResult& require_row(const RichResultAttribution& result,
                                       const std::string& type,
                                       const std::string& domain, int index) {
  const auto it = std::find_if(
      result.components.begin(), result.components.end(),
      [&](const RichComponentResult& row) {
        return row.component_type == type && row.domain == domain &&
               row.component_index == index;
      });
  REQUIRE(it != result.components.end());
  return *it;
}

double require_value(const RichComponentResult& row, const std::string& name) {
  const auto it = std::find_if(row.values.begin(), row.values.end(),
                               [&](const AttributedValue& value) {
                                 return value.name == name;
                               });
  REQUIRE(it != row.values.end());
  return it->value;
}

}  // namespace

TEST_CASE("SPPT attribution is total and keyed by rich identity",
          "[projection][attribution]") {
  HybridPowerSystem rich;
  rich.base_mva = rich.ac.base_mva = rich.dc.base_mva = 10.0;

  for (int index = 1; index <= 3; ++index) {
    ACBus bus;
    bus.index = index;
    bus.bus_type = index == 1 ? BusType::SLACK : BusType::PQ;
    bus.base_kv = 10.0;
    rich.ac.buses.push_back(bus);
  }
  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.bus_type = DCBusType::DC_V;
  dc_bus.base_kv = 1.0;
  rich.dc.buses.push_back(dc_bus);

  ExternalGrid grid;
  grid.index = 51;
  grid.bus = 1;
  grid.name = "Grid";
  rich.ac.external_grids.push_back(grid);

  Transformer2W transformer;
  transformer.index = 71;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.sn_mva = 5.0;
  transformer.vn_hv_kv = 10.0;
  transformer.vn_lv_kv = 10.0;
  transformer.vk_percent = 5.0;
  transformer.vkr_percent = 1.0;
  rich.ac.transformers_2w.push_back(transformer);

  CircuitBreaker breaker;
  breaker.index = 34;
  breaker.bus_from = 2;
  breaker.bus_to = 3;
  breaker.closed = true;
  breaker.z_ohm = 0.0;
  rich.ac.circuit_breakers.push_back(breaker);

  VSCConverter inactive_vsc;
  inactive_vsc.index = 80;
  inactive_vsc.bus_ac = 2;
  inactive_vsc.bus_dc = 1;
  inactive_vsc.in_service = false;
  rich.vsc_converters.push_back(inactive_vsc);
  VSCConverter vsc;
  vsc.index = 81;
  vsc.bus_ac = 3;
  vsc.bus_dc = 1;
  vsc.p_schedule_mw = 1.2;
  vsc.p_initial_mw = 0.8;
  rich.vsc_converters.push_back(vsc);

  DCStorage inactive_storage;
  inactive_storage.index = 90;
  inactive_storage.bus = 1;
  inactive_storage.in_service = false;
  rich.dc.dc_storage.push_back(inactive_storage);
  DCStorage storage;
  storage.index = 91;
  storage.bus = 1;
  storage.name = "DC BESS";
  rich.dc.dc_storage.push_back(storage);

  const auto projection = RichToCanonicalOperator::apply(rich);

  PowerFlowResult canonical_pf;
  canonical_pf.converged = true;
  canonical_pf.vm.assign(projection.canonical.ac.buses.size(), 1.0);
  canonical_pf.va.assign(projection.canonical.ac.buses.size(), 0.0);
  canonical_pf.vdc.assign(projection.canonical.dc.buses.size(), 1.0);
  canonical_pf.branch_flows.resize(projection.canonical.ac.branches.size());
  REQUIRE(projection.canonical.branch_expand_map.has_value());
  for (const auto& entry : projection.canonical.branch_expand_map->entries) {
    if (entry.origin_type != BranchOriginType::Transformer2W ||
        entry.origin_index != transformer.index)
      continue;
    const auto branch = std::find_if(
        projection.canonical.ac.branches.begin(),
        projection.canonical.ac.branches.end(),
        [&](const ACBranch& item) { return item.index == entry.branch_index; });
    REQUIRE(branch != projection.canonical.ac.branches.end());
    const auto position = static_cast<size_t>(std::distance(
        projection.canonical.ac.branches.begin(), branch));
    canonical_pf.branch_flows[position].pf_mw = 2.5;
    canonical_pf.branch_flows[position].pt_mw = -2.4;
  }

  PowerFlowResult rich_pf = canonical_pf;
  VSCTransfer transfer;
  transfer.index = 81;
  transfer.bus_ac = 3;
  transfer.bus_dc = 1;
  transfer.p_ac_mw = 1.5;
  transfer.q_ac_mvar = 0.2;
  transfer.p_dc_mw = -1.53;
  transfer.loss_mw = 0.03;
  rich_pf.vsc_transfers.push_back(transfer);

  opf::ACOPFResult opf;
  opf.pstor_mw = {3.25};
  opf.qstor_mvar = {0.0};
  opf.stor_map.push_back({1, 1});

  const auto result = CanonicalToRichOperator::apply(
      rich, projection, &opf, &canonical_pf, &rich_pf);

  CHECK_FALSE(result.coverage.total());
  CHECK(result.coverage.unsupported_components > 0);
  CHECK(result.coverage.rich_components ==
        static_cast<int>(result.components.size()));

  const auto& attributed_vsc = require_row(result, "vsc_converter", "ACDC", 81);
  REQUIRE(attributed_vsc.terminals.size() == 2);
  CHECK(attributed_vsc.terminals[0].name == "ac");
  CHECK(attributed_vsc.terminals[0].bus == 3);
  CHECK(attributed_vsc.terminals[1].name == "dc");
  CHECK(attributed_vsc.terminals[1].bus == 1);
  CHECK_THAT(require_value(attributed_vsc, "p_initial_mw"), WithinAbs(0.8, 1e-12));

  const auto& attributed_storage = require_row(result, "dc_storage", "DC", 91);
  CHECK_THAT(require_value(attributed_storage, "p_mw"), WithinAbs(3.25, 1e-12));
  REQUIRE(attributed_storage.terminals.size() == 1);
  CHECK(attributed_storage.terminals[0].bus == 1);
  CHECK_THAT(attributed_storage.terminals[0].p_mw, WithinAbs(3.25, 1e-12));
  CHECK(require_value(require_row(result, "dc_storage", "DC", 90), "p_mw") == 0.0);

  const auto& attributed_transformer =
      require_row(result, "transformer_2w", "AC", 71);
  REQUIRE(attributed_transformer.terminals.size() == 2);
  CHECK(attributed_transformer.terminals[0].bus == 1);
  CHECK(attributed_transformer.terminals[1].bus == 2);
  CHECK_THAT(attributed_transformer.terminals[0].p_mw, WithinAbs(2.5, 1e-12));
  CHECK_THAT(attributed_transformer.terminals[1].p_mw, WithinAbs(-2.4, 1e-12));

  const auto& attributed_breaker =
      require_row(result, "circuit_breaker", "AC", 34);
  CHECK(attributed_breaker.component_index == 34);
}

TEST_CASE("SPPT attribution restores charging from a merged self-loop line",
          "[projection][attribution][self-loop][charging]") {
  HybridPowerSystem rich;
  rich.base_mva = rich.ac.base_mva = 100.0;
  ACBus slack;
  slack.index = 1;
  slack.bus_type = BusType::SLACK;
  ACBus pq;
  pq.index = 2;
  pq.bus_type = BusType::PQ;
  rich.ac.buses = {slack, pq};

  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  rich.ac.external_grids = {grid};

  ACBranch line;
  line.index = 7;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.01;
  line.x_pu = 0.05;
  line.b_pu = 0.02;
  rich.ac.branches = {line};

  Switch bypass;
  bypass.index = 3;
  bypass.bus_from = 1;
  bypass.bus_to = 2;
  bypass.in_service = true;
  bypass.closed = true;
  rich.ac.switches = {bypass};

  const auto projection = RichToCanonicalOperator::apply(rich);
  REQUIRE(projection.canonical.bus_merge_map.has_value());
  REQUIRE(projection.canonical.ac.branches.empty());
  CHECK_THAT(projection.canonical.ac.buses.front().bs_mvar,
             WithinAbs(2.0, 1e-12));

  PowerFlowResult canonical_pf;
  canonical_pf.converged = true;
  canonical_pf.vm = {1.0};
  canonical_pf.va = {0.0};
  PowerFlowResult rich_pf;
  rich_pf.converged = true;
  rich_pf.vm = {1.0, 1.0};
  rich_pf.va = {0.0, 0.0};
  rich_pf.branch_flows.resize(1);

  const auto solved_pf = solve_power_flow(rich);
  REQUIRE(solved_pf.converged);
  REQUIRE(solved_pf.branch_flows.size() == 1);
  CHECK_THAT(solved_pf.branch_flows[0].qf_mvar, WithinAbs(-1.0, 1e-12));
  CHECK_THAT(solved_pf.branch_flows[0].qt_mvar, WithinAbs(-1.0, 1e-12));

  const auto result = CanonicalToRichOperator::apply(
      rich, projection, nullptr, &canonical_pf, &rich_pf);
  const auto& attributed_line = require_row(result, "ac_branch", "AC", 7);
  REQUIRE(attributed_line.terminals.size() == 2);
  CHECK_THAT(attributed_line.terminals[0].q_mvar, WithinAbs(-1.0, 1e-12));
  CHECK_THAT(attributed_line.terminals[1].q_mvar, WithinAbs(-1.0, 1e-12));
  CHECK(attributed_line.recovery == RecoveryClass::AuditOnly);
}

TEST_CASE("Attribution indexes preserve domain, positions and per-call voltage state",
          "[projection][attribution][indexed]") {
  HybridPowerSystem rich;
  ACBus ac90; ac90.index = 90; ac90.vm_pu = 1.02;
  ACBus ac10; ac10.index = 10; ac10.vm_pu = 1.03;
  DCBus dc10; dc10.index = 10; dc10.vm_pu = 0.96;
  rich.ac.buses = {ac90, ac10};
  rich.dc.buses = {dc10};
  Generator first; first.index = 71; first.bus = 10; first.pg_mw = 3.0;
  Generator second = first; second.index = 19; second.bus = 90;
  rich.ac.generators = {first, second};
  Storage ac; ac.index = 71; ac.bus = 10; ac.p_mw = 4.0;
  Storage dc = ac; dc.p_mw = 5.0;
  rich.ac.storage = {ac};
  rich.dc.storage = {dc};
  Load missing; missing.index = 71; missing.bus = 777;
  rich.ac.loads = {missing};

  ProjectionBundle bundle;
  bundle.canonical = rich;
  std::reverse(bundle.canonical.ac.buses.begin(), bundle.canonical.ac.buses.end());
  std::reverse(bundle.canonical.ac.generators.begin(), bundle.canonical.ac.generators.end());
  CHECK(CanonicalToRichOperator::ac_bus_reprojection_positions(rich, bundle) ==
        std::vector<int>{1, 0});
  bundle.canonical.ac.buses.pop_back();
  CHECK(CanonicalToRichOperator::ac_bus_reprojection_positions(rich, bundle) ==
        std::vector<int>{-1, 0});

  PowerFlowResult pf;
  pf.vm = {1.11}; // AC10 must use its authored value, not DC10's solved value.
  pf.vdc = {0.91};
  opf::ACOPFResult opf;
  opf.pg_mw = {20.0, 70.0};
  opf.qg_mvar = {2.0, 7.0};
  opf.pstor_mw = {8.0, 9.0};
  opf.qstor_mvar = {0.8, 0.9};
  opf.stor_map = {{0, 0}, {0, 1}};
  const auto result = CanonicalToRichOperator::apply(rich, bundle, &opf, nullptr, &pf);
  const auto& g = require_row(result, "generator", "AC", 71);
  CHECK(g.position == 0);
  CHECK(require_value(g, "p_mw") == 70.0);
  CHECK(require_value(g, "q_mvar") == 7.0);
  CHECK(g.terminals.front().v_pu == 1.03);
  CHECK(require_value(require_row(result, "ac_bus", "AC", 90), "vm_pu") == 1.11);
  CHECK(require_row(result, "load", "AC", 71).terminals.front().v_pu == 0.0);
  const auto& ac_result = require_row(result, "storage", "AC", 71);
  const auto& dc_result = require_row(result, "storage", "DC", 71);
  CHECK(ac_result.terminals.front().p_mw == 8.0);
  CHECK(dc_result.terminals.front().p_mw == 9.0);
  CHECK(ac_result.terminals.front().v_pu == 1.03);
  CHECK(dc_result.terminals.front().v_pu == 0.91);

  const auto authored = CanonicalToRichOperator::apply(rich, bundle, nullptr, nullptr, nullptr);
  CHECK(require_value(require_row(authored, "ac_bus", "AC", 90), "vm_pu") == 1.02);
  CHECK(require_value(require_row(authored, "dc_bus", "DC", 10), "vdc_pu") == 0.96);
  CHECK(require_value(require_row(authored, "generator", "AC", 71), "p_mw") == 3.0);
  pf.vm = {1.04, 1.05}; pf.vdc = {0.92};
  const auto replay = CanonicalToRichOperator::apply(rich, bundle, &opf, nullptr, &pf, {true});
  CHECK(require_row(replay, "storage", "AC", 71).terminals.front().p_mw == 4.0);
  CHECK(require_row(replay, "storage", "DC", 71).terminals.front().p_mw == 5.0);
  CHECK(require_row(replay, "generator", "AC", 71).terminals.front().v_pu == 1.05);
  CHECK(require_row(replay, "storage", "DC", 71).terminals.front().v_pu == 0.92);
  CHECK(result.components.size() == replay.components.size());
}

TEST_CASE("Attribution source index preserves mapping order and all matching identities",
          "[projection][attribution][indexed]") {
  HybridPowerSystem rich;
  CircuitBreaker ac; ac.index = 8; ac.in_service = false;
  DCCircuitBreaker dc; dc.index = 8; dc.in_service = false;
  rich.ac.circuit_breakers = {ac};
  rich.dc.dc_circuit_breakers = {dc};
  FlexibleLoad load; load.index = 8;
  rich.ac.flexible_loads = {load};
  ProjectionBundle bundle;
  bundle.canonical.projection_report.emplace();
  bundle.canonical.projection_report->mappings = {
      {"CircuitBreaker", "8", "ACBranch", "901", 0.25},
      {"Unknown", "8", "ACBranch", "999", 1.0},
      {"CircuitBreaker", "8", "ACBranch", "902", 0.75},
      {"FlexibleLoad", "8", "Load", "903", 1.0},
      {"Switch", "88", "ACBranch", "904", 1.0}};
  const auto result = CanonicalToRichOperator::apply(rich, bundle, nullptr, nullptr, nullptr);
  REQUIRE(result.components.size() == 3);
  CHECK(result.components.front().component_type == "flexible_load");
  for (const auto& domain : {"AC", "DC"}) {
    const auto& row = require_row(result, "circuit_breaker", domain, 8);
    REQUIRE(row.canonical_sources.size() == 2);
    CHECK(row.canonical_sources[0].component_index == 901);
    CHECK(row.canonical_sources[0].participation_factor == 0.25);
    CHECK(row.canonical_sources[1].component_index == 902);
  }
  const auto& row = require_row(result, "flexible_load", "AC", 8);
  REQUIRE(row.canonical_sources.size() == 1);
  CHECK(row.canonical_sources.front().component_index == 903);
  CHECK(CanonicalToRichOperator::apply({}, {}, nullptr, nullptr, nullptr).components.empty());
}
