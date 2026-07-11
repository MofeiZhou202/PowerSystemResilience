#include <algorithm>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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

  CHECK(result.coverage.total());
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
