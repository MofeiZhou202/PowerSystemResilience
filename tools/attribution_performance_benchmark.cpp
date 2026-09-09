// AUD-097: time the production attribution operator on fixed solved states.
#include <chrono>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sys/resource.h>
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/projection/result_attribution.hpp"
#include BENCH_SOURCE

using namespace hacdcpf;
using namespace hacdcpf::projection;
using json = nlohmann::json;

json serialize(const RichResultAttribution& result) {
  json rows = json::array();
  for (const auto& row : result.components) {
    json values = json::array(), terminals = json::array(), sources = json::array();
    for (const auto& v : row.values) values.push_back({v.name, v.value, v.unit});
    for (const auto& t : row.terminals)
      terminals.push_back({t.name, t.bus, t.is_dc, t.p_mw, t.q_mvar, t.v_pu,
                           t.has_p, t.has_q, t.has_v});
    for (const auto& s : row.canonical_sources)
      sources.push_back({s.component_type, s.component_index, s.participation_factor});
    rows.push_back({row.component_type, row.domain, row.component_index, row.position,
                    row.name, row.in_service, static_cast<int>(row.recovery),
                    row.recovery_reason, values, terminals, sources});
  }
  const auto& c = result.coverage;
  return {rows, {c.rich_components, c.attributed_components, c.strong_components,
                 c.approximate_components, c.audit_only_components, c.unsupported_components},
          result.diagnostics};
}

HybridPowerSystem balanced_fixture(int n) {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = sys.dc.base_mva = 100.0;
  for (int i = 1; i <= n; ++i) {
    ACBus bus;
    bus.index = i * 10;
    bus.bus_type = BusType::SLACK;
    bus.vm_pu = 1.0;
    bus.base_kv = 110.0;
    sys.ac.buses.push_back(bus);
    Generator gen;
    gen.index = i * 13;
    gen.bus = bus.index;
    gen.is_slack = true;
    gen.pg_mw = 1.0;
    gen.pmax_mw = 100.0;
    sys.ac.generators.push_back(gen);
    Load load;
    load.index = i * 17;
    load.bus = bus.index;
    load.p_mw = 1.0;
    sys.ac.loads.push_back(load);
    DCBus dc;
    dc.index = bus.index;
    dc.bus_type = DCBusType::DC_V;
    dc.vm_pu = 0.97;
    dc.base_kv = 10.0;
    sys.dc.buses.push_back(dc);
    if (i % 2 == 0) {
      Transformer2W transformer;
      transformer.index = i * 19;
      transformer.hv_bus = (i - 1) * 10;
      transformer.lv_bus = i * 10;
      transformer.sn_mva = 100.0;
      transformer.vn_hv_kv = transformer.vn_lv_kv = 110.0;
      transformer.vk_percent = 5.0;
      transformer.vkr_percent = 1.0;
      sys.ac.transformers_2w.push_back(transformer);
    }
  }
  std::reverse(sys.ac.buses.begin(), sys.ac.buses.end());
  std::reverse(sys.ac.generators.begin(), sys.ac.generators.end());
  return sys;
}

int main(int argc, char** argv) {
  if (argc != 4) return 2;
  const std::string fixture = argv[1];
  const auto sys = fixture == "synthetic" ? balanced_fixture(std::stoi(argv[2]))
                                          : io::parse_matpower(fixture);
  const auto projected = RichToCanonicalOperator::apply(sys);
  PowerFlowOptions options;
  options.max_iter = 100;
  const auto pf = solve_power_flow(sys, options);
  const auto canonical_pf = solve_power_flow(projected.canonical, options);
  if (!pf.converged || !canonical_pf.converged) return 3;
  // Batch sub-millisecond fixtures to reduce clock/scheduling noise.
  const int repetitions = sys.ac.buses.size() <= 118 ? 100 : 1;
  RichResultAttribution result;
  const auto start = std::chrono::steady_clock::now();
  for (int iteration = 0; iteration < repetitions; ++iteration)
    result = CanonicalToRichOperator::apply(sys, projected, nullptr, &canonical_pf, &pf);
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() / repetitions;
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  const double rss_mb = usage.ru_maxrss /
#ifdef __APPLE__
      1e6;
#else
      1e3;
#endif
  std::ofstream output(argv[3]);
  output << serialize(result).dump();
  json timing{{"seconds", seconds}, {"peak_rss_mb", rss_mb},
               {"repetitions", repetitions},
               {"rows", result.components.size()}, {"ac_buses", sys.ac.buses.size()},
               {"mappings", projected.canonical.projection_report ? projected.canonical.projection_report->mappings.size() : 0},
               {"pf_converged", pf.converged}, {"pf_residual", pf.residual}};
  std::cout << timing.dump() << '\n';
  return output ? 0 : 4;
}
