// Hosting-capacity independent cross-validation: evidence emitter.
//
// Runs the production DL/T 2041-2025 equipment-level hosting-capacity assessment
// (assess_hosting_capacity) on deterministic single-transformer supply areas and
// dumps the supply-area aggregates and transformer parameters together with the
// resulting hosting-capacity interval and accessible-capacity figures.  The
// companion oracle tools/analysis_validation/run_cross_validation.py does NOT
// link hacdcpf: it re-derives the closed-form
//
//     S_d = max(0, (P - P_G + beta * n * S * cos(theta) + P_ESS + dP_ESS) / tau)
//
// and the accessible-capacity subtractions, and checks them against the reported
// figures (case 1 is additionally the hand-verified analytic anchor from
// tests/test_hosting_capacity.cpp).
//
// Usage: validate_hosting_xref <output.json>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/hosting_capacity.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::analysis;

namespace {

struct CaseSpec {
  std::string name;
  double sn_mva{10.0};
  double cap_power_factor{0.95};
  double cap_reverse_load_rate{0.0};  // 0 -> auto N-1 from n_parallel
  double cap_tau{1.0};
  double cap_registered_dr{0.0};
  double dpess_min{0.0};
  double dpess_max{0.0};
  int n_parallel{1};
  double load_pd_bus{0.0};   // bus-embedded load on the LV load bus
  double load_p{0.0};        // Load element p_mw
  double load_scaling{1.0};
  double pv_p{0.0};          // PVSystem -> counts as existing DR
  double storage_static{0.0};  // static storage charging -> P_ESS
};

json emit_case(const CaseSpec& spec) {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;

  ACBus hv;
  hv.index = 1;
  hv.base_kv = 110.0;
  hv.in_service = true;
  ACBus lv;
  lv.index = 2;
  lv.base_kv = 10.0;
  lv.in_service = true;
  ACBus load_bus;
  load_bus.index = 3;
  load_bus.base_kv = 10.0;
  load_bus.in_service = true;
  load_bus.pd_mw = spec.load_pd_bus;
  sys.ac.buses = {hv, lv, load_bus};

  Transformer2W tr;
  tr.index = 1;
  tr.name = spec.name;
  tr.hv_bus = 1;
  tr.lv_bus = 2;
  tr.sn_mva = spec.sn_mva;
  tr.n_parallel = spec.n_parallel;
  tr.cap_power_factor = spec.cap_power_factor;
  tr.cap_max_reverse_load_rate = spec.cap_reverse_load_rate;
  tr.cap_dr_max_output_coeff = spec.cap_tau;
  tr.cap_registered_dr_mw = spec.cap_registered_dr;
  tr.cap_expected_new_storage_min_mw = spec.dpess_min;
  tr.cap_expected_new_storage_max_mw = spec.dpess_max;
  sys.ac.transformers_2w = {tr};

  ACBranch feeder;
  feeder.index = 1;
  feeder.from_bus = 2;
  feeder.to_bus = 3;
  feeder.in_service = true;
  sys.ac.branches = {feeder};

  if (spec.load_p > 0.0) {
    Load load;
    load.index = 1;
    load.bus = 3;
    load.p_mw = spec.load_p;
    load.scaling = spec.load_scaling;
    load.in_service = true;
    sys.ac.loads = {load};
  }
  if (spec.pv_p > 0.0) {
    PVSystem pv;
    pv.index = 1;
    pv.bus = 3;
    pv.p_mw = spec.pv_p;
    sys.ac.pv_systems = {pv};
  }
  if (spec.storage_static > 0.0) {
    Storage storage;
    storage.index = 1;
    storage.bus = 3;
    storage.cap_charging_strategy = "static";
    storage.cap_static_charging_mw = spec.storage_static;
    sys.ac.storage = {storage};
  }

  const HostingCapacityResult result = assess_hosting_capacity(sys);
  const TransformerHostingResult& t = result.transformers.at(0);

  return json{
      {"name", spec.name},
      {"n_parallel", spec.n_parallel},
      {"dpess_min", spec.dpess_min},
      {"dpess_max", spec.dpess_max},
      {"sn_mva", t.sn_mva},
      {"power_factor", t.power_factor},
      {"beta", t.beta},
      {"beta_auto", t.beta_auto},
      {"tau_max", t.tau_max},
      {"supply_load_mw", t.supply_load_mw},
      {"supply_nondr_gen_mw", t.supply_nondr_gen_mw},
      {"supply_existing_dr_mw", t.supply_existing_dr_mw},
      {"supply_ess_charging_mw", t.supply_ess_charging_mw},
      {"registered_dr_mw", t.registered_dr_mw},
      {"hosting_min_mw", t.hosting_min_mw},
      {"hosting_max_mw", t.hosting_max_mw},
      {"accessible_grid_min_mw", t.accessible_grid_min_mw},
      {"accessible_grid_max_mw", t.accessible_grid_max_mw},
      {"accessible_reg_min_mw", t.accessible_reg_min_mw},
      {"accessible_reg_max_mw", t.accessible_reg_max_mw}};
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: validate_hosting_xref <output.json>\n";
    return 2;
  }

  json cases = json::array();

  // 1) Hand-verified analytic anchor (tests/test_hosting_capacity.cpp):
  //    P=2, DR=1, P_ESS=0.5, beta=0.8, cos=1, S=10, tau=1, dP_ESS in [0,2]
  //    -> S_d in [10.5, 12.5]; accessible grid/reg min 9.5 / 7.5.
  {
    CaseSpec s;
    s.name = "anchor_test_case";
    s.sn_mva = 10.0;
    s.cap_power_factor = 1.0;
    s.cap_reverse_load_rate = 0.8;
    s.cap_registered_dr = 2.0;
    s.dpess_max = 2.0;
    s.load_pd_bus = 1.0;
    s.load_p = 2.0;
    s.load_scaling = 0.5;
    s.pv_p = 1.0;
    s.storage_static = 0.5;
    cases.push_back(emit_case(s));
  }
  // 2) Auto-beta, single transformer (beta = 0.80), default cos=0.95, tau=1.
  {
    CaseSpec s;
    s.name = "auto_beta_single";
    s.sn_mva = 20.0;
    s.load_p = 3.0;
    cases.push_back(emit_case(s));
  }
  // 3) Two parallel transformers -> auto beta = (n-1)/n = 0.5, custom cos/tau.
  {
    CaseSpec s;
    s.name = "n_parallel_auto_beta";
    s.n_parallel = 2;
    s.sn_mva = 10.0;
    s.cap_power_factor = 0.9;
    s.cap_tau = 1.25;
    s.load_p = 4.0;
    cases.push_back(emit_case(s));
  }
  // 4) Storage interval with a negative lower bound -> S_d,min clamped at 0.
  {
    CaseSpec s;
    s.name = "ess_interval_clamp";
    s.sn_mva = 5.0;
    s.cap_power_factor = 1.0;
    s.cap_reverse_load_rate = 0.2;
    s.dpess_min = -3.0;
    s.dpess_max = 1.0;
    s.load_p = 1.0;
    cases.push_back(emit_case(s));
  }

  json out;
  out["schema"] = "hysim-hosting-capacity-evidence-v1";
  out["cases"] = cases;

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
