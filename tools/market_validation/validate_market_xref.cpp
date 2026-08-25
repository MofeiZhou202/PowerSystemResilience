// Market SCED/LMP + settlement independent cross-validation: evidence emitter.
//
// Runs the production day-ahead market (run_day_ahead_market) on two
// deterministic single-bus "copper-plate" cases with linear generator costs, so
// the fixed-commitment SCED locational marginal price and the settlement ledger
// have exact closed forms:
//   * LMP equals the marginal dispatched offer price (merit order);
//   * dispatch is the merit-order stack;
//   * on a single bus congestion rent is zero and the settlement is revenue
//     adequate (customer payment = resource revenue, cashflow residual 0).
//
// The companion oracle tools/market_validation/run_cross_validation.py does NOT
// link hacdcpf: it re-derives the merit-order dispatch/LMP from the emitted
// offers and demand and independently checks the revenue-adequacy identities.
//
// Usage: validate_market_xref <output.json>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/market/market_simulation.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace hacdcpf;

namespace {

ACBus slack_bus(int index) {
  ACBus bus;
  bus.index = index;
  bus.bus_type = BusType::SLACK;
  bus.in_service = true;
  bus.base_kv = 100.0;
  bus.vm_pu = 1.0;
  return bus;
}

Generator linear_gen(int index, int bus, double pmax, double marginal_cost,
                     double dispatch_hint) {
  Generator gen;
  gen.index = index;
  gen.bus = bus;
  gen.in_service = true;
  gen.pg_mw = dispatch_hint;  // seeds the initial commitment (>0 -> committed)
  gen.pmin_mw = 0.0;
  gen.pmax_mw = pmax;
  gen.cost_c2 = 0.0;
  gen.cost_c1 = marginal_cost;
  gen.cost_c0 = 0.0;
  return gen;
}

Load bus_load(int index, int bus, double p_mw) {
  Load load;
  load.index = index;
  load.bus = bus;
  load.in_service = true;
  load.p_mw = p_mw;
  load.profile_id = 0;
  return load;
}

market::MarketOptions copper_plate_options() {
  market::MarketOptions options;
  options.energy_offer_segments = 1;         // one block -> segment price = c1
  options.upward_reserve_fraction = 0.0;     // no reserve co-optimization
  options.value_of_lost_load_per_mwh = 10000.0;
  options.enable_network_constraints = false;
  options.run_ac_validation = false;         // pure economic dispatch, no PF
  options.uc_options.uc_solver = UCSolverChoice::Native;
  return options;
}

TimeSeriesData one_period_flat() {
  TimeSeriesData ts;
  ts.num_steps = 1;
  ts.step_duration_hr = 1.0;
  ts.profiles.push_back(TimeSeriesProfile{0, "flat", {1.0}});
  return ts;
}

json emit_case(const std::string& name, double demand_mw) {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  sys.ac.buses = {slack_bus(1)};
  // gen1 cheap (marginal 20), gen2 expensive (marginal 40); each 100 MW cap.
  sys.ac.generators = {linear_gen(1, 1, 100.0, 20.0, 50.0),
                       linear_gen(2, 1, 100.0, 40.0, 50.0)};
  sys.ac.loads = {bus_load(1, 1, demand_mw)};

  const market::MarketResult result =
      market::run_day_ahead_market(sys, one_period_flat(), copper_plate_options());

  json generators = json::array();
  const auto& period = result.pricing.at(0);
  for (std::size_t g = 0; g < sys.ac.generators.size(); ++g) {
    double offer_price = 0.0;
    if (g < result.offers.size() && !result.offers[g].energy_segments.empty()) {
      offer_price = result.offers[g].energy_segments.front().price_per_mwh;
    }
    generators.push_back({{"bus", sys.ac.generators[g].bus},
                          {"offer_price", offer_price},
                          {"pmax_mw", sys.ac.generators[g].pmax_mw},
                          {"dispatch_mw", period.generator_dispatch_mw.at(g)}});
  }

  json lmp = json::array();
  for (double value : period.lmp_per_mwh) lmp.push_back(value);

  const auto& s = result.settlement;
  return json{
      {"name", name},
      {"feasible", result.feasible},
      {"status", result.status},
      {"demand_mw", demand_mw},
      {"generators", generators},
      {"lmp_per_bus", lmp},
      {"total_load_shedding_mw",
       std::accumulate(period.load_shedding_mw.begin(),
                       period.load_shedding_mw.end(), 0.0)},
      {"settlement",
       {{"customer_energy_payment", s.customer_energy_payment},
        {"customer_total_payment", s.customer_total_payment},
        {"resource_energy_revenue", s.resource_energy_revenue},
        {"resource_total_revenue", s.resource_total_revenue},
        {"congestion_rent", s.congestion_rent},
        {"cashflow_residual", s.cashflow_residual}}}};
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: validate_market_xref <output.json>\n";
    return 2;
  }

  json out;
  out["schema"] = "hysim-market-sced-evidence-v1";
  // Case A: demand below the cheap unit's cap -> only gen1 marginal (LMP 20).
  // Case B: demand above it -> gen1 full, gen2 marginal (LMP 40).
  out["cases"] = json::array({emit_case("copper_plate_cheap_marginal", 60.0),
                              emit_case("copper_plate_peaker_marginal", 140.0)});

  const fs::path path(argv[1]);
  if (path.has_parent_path()) fs::create_directories(path.parent_path());
  std::ofstream os(path);
  if (!os) {
    std::cerr << "cannot open output: " << argv[1] << "\n";
    return 1;
  }
  os << out.dump(2) << "\n";
  return 0;
}
