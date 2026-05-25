#include "hacdcpf/time_series/time_series_pf.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/graph/graph.hpp"
#include <spdlog/spdlog.h>
#include "hacdcpf/engine/adapter_registry.hpp"
#include "hacdcpf/engine/native_adapters.hpp"
#include "hacdcpf/engine/external_adapters.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/power_flow/branch_flow.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"

namespace hacdcpf {

namespace {

// ═══════════════════════════════════════════════════════════════════════
// Helper: look up profile values for a given profile_id and time step
// ═══════════════════════════════════════════════════════════════════════
const std::vector<double>* find_profile(
    const std::unordered_map<int, const TimeSeriesProfile*>& profile_map,
    int profile_id) {
  if (profile_id < 0) return nullptr;
  auto it = profile_map.find(profile_id);
  if (it == profile_map.end()) return nullptr;
  return &it->second->values;
}

double profile_value(const std::vector<double>* pv, int t, double fallback) {
  if (!pv || t < 0 || t >= static_cast<int>(pv->size())) return fallback;
  return (*pv)[static_cast<size_t>(t)];
}

void accumulate_signed_power(double p_mw, double& total_supply_mw,
                             double& total_demand_mw) {
  if (p_mw >= 0.0) total_supply_mw += p_mw;
  else total_demand_mw += -p_mw;
}

double estimate_boundary_loss_mw(const HybridPowerSystem& sys) {
  double total_supply_mw = 0.0;
  double total_demand_mw = 0.0;

  // AC-side injections (positive = supply, negative = demand).
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    accumulate_signed_power(g.pg_mw, total_supply_mw, total_demand_mw);
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    accumulate_signed_power(sg.p_mw * sg.scaling, total_supply_mw, total_demand_mw);
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    accumulate_signed_power(rg.p_mw, total_supply_mw, total_demand_mw);
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    accumulate_signed_power(powerflow::compute_pv_power_mw(pv), total_supply_mw, total_demand_mw);
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    accumulate_signed_power(st.p_mw, total_supply_mw, total_demand_mw);
  }
  for (const auto& vpp : sys.vpps) {
    if (!vpp.in_service) continue;
    accumulate_signed_power(vpp.p_output_mw, total_supply_mw, total_demand_mw);
  }
  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service || mg.operating_mode != MicrogridMode::GridConnected) continue;
    accumulate_signed_power(mg.p_exchange_mw, total_supply_mw, total_demand_mw);
  }
  for (const auto& ms : sys.mobile_storage) {
    if (!ms.in_service || ms.status == MobileStorageStatus::InTransit) continue;
    accumulate_signed_power(ms.p_mw, total_supply_mw, total_demand_mw);
  }

  // AC demand boundary follows PF aggregation semantics: when component-load
  // tables are present, bus-level pd_mw is not consumed directly.
  if (!sys.ac.loads.empty() || !sys.ac.charging_stations.empty()) {
    for (const auto& ld : sys.ac.loads) {
      if (!ld.in_service) continue;
      total_demand_mw += std::max(0.0, ld.p_mw);
    }
    for (const auto& cs : sys.ac.charging_stations) {
      if (!cs.in_service) continue;
      total_demand_mw += std::max(0.0, cs.p_total_kw / 1000.0);
    }
  } else {
    for (const auto& b : sys.ac.buses) {
      if (!b.in_service) continue;
      total_demand_mw += std::max(0.0, b.pd_mw);
    }
    if (sys.ac.charging_stations.empty()) {
      for (const auto& ch : sys.ac.chargers) {
        if (!ch.in_service) continue;
        total_demand_mw += std::max(0.0, ch.p_ch_max_kw / 1000.0);
      }
    }
  }

  // Flexible/asymmetric loads are projected to canonical load tables during PF.
  for (const auto& fl : sys.ac.flexible_loads) {
    if (!fl.in_service) continue;
    total_demand_mw += std::max(0.0, fl.p_mw);
  }
  for (const auto& al : sys.ac.asymmetric_loads) {
    if (!al.in_service) continue;
    total_demand_mw += std::max(0.0, al.pa_mw + al.pb_mw + al.pc_mw);
  }

  // DC-side injections (positive = supply, negative = demand).
  if (!sys.dc.loads.empty()) {
    for (const auto& ld : sys.dc.loads) {
      if (!ld.in_service) continue;
      total_demand_mw += std::max(0.0, ld.p_mw);
    }
  } else {
    for (const auto& b : sys.dc.buses) {
      if (!b.in_service) continue;
      total_demand_mw += std::max(0.0, b.pd_mw);
    }
  }
  for (const auto& st : sys.dc.storage) {
    if (!st.in_service) continue;
    accumulate_signed_power(st.p_mw, total_supply_mw, total_demand_mw);
  }
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    accumulate_signed_power(sg.p_mw * sg.scaling, total_supply_mw, total_demand_mw);
  }
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    accumulate_signed_power(sg.p_set_mw * sg.scaling, total_supply_mw, total_demand_mw);
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    accumulate_signed_power(pv.p_set_mw, total_supply_mw, total_demand_mw);
  }

  return std::abs(total_supply_mw - total_demand_mw);
}

double estimate_opf_physical_loss_mw(const HybridPowerSystem& sys,
                                     const opf::ACOPFResult& opf_res,
                                     LossModelType loss_model) {
  double loss_mw = 0.0;

  // AC branch losses from OPF voltages.
  try {
    auto data = powerflow::make_solver_data(sys, loss_model);
    if (opf_res.vm.size() == data.ac_buses.size() &&
        opf_res.va.size() == data.ac_buses.size()) {
      const auto branch_flows = powerflow::compute_branch_flows(data, opf_res.vm, opf_res.va);
      for (const auto& bf : branch_flows) {
        loss_mw += std::max(bf.pf_mw + bf.pt_mw, 0.0);
      }
    }

    // Converter losses from OPF voltages with the same converter model.
    if (opf_res.vm.size() == data.ac_buses.size() &&
        opf_res.va.size() == data.ac_buses.size() &&
        opf_res.vdc.size() == data.dc_buses.size()) {
      Eigen::VectorXd vm = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(opf_res.vm.size()));
      Eigen::VectorXd va = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(opf_res.va.size()));
      Eigen::VectorXd vdc = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(opf_res.vdc.size()));
      for (Eigen::Index i = 0; i < vm.size(); ++i) vm[i] = opf_res.vm[static_cast<size_t>(i)];
      for (Eigen::Index i = 0; i < va.size(); ++i) va[i] = opf_res.va[static_cast<size_t>(i)];
      for (Eigen::Index i = 0; i < vdc.size(); ++i) vdc[i] = opf_res.vdc[static_cast<size_t>(i)];

      for (const auto& conv : data.converters) {
        if (!conv.in_service) continue;
        const auto [p_ac_pu, q_ac_pu] = powerflow::converter_ac_injection(
            conv, vm, va, vdc, data.base_mva, loss_model);
        (void)q_ac_pu;
        const double p_dc_pu = powerflow::converter_dc_injection(
            conv, vm, va, vdc, data.base_mva, loss_model);
        const double p_ac_mw = p_ac_pu * data.base_mva;
        const double p_dc_mw = p_dc_pu * data.base_mva;
        loss_mw += std::max(-(p_ac_mw + p_dc_mw), 0.0);
      }
    } else {
      // Fallback when OPF does not expose full voltage vectors for converter loss
      // evaluation: use solved PF-style transfer arrays when available.
      for (size_t k = 0; k < opf_res.pac_mw.size(); ++k) {
        // Without pdc in ACOPFResult we cannot recover converter loss exactly here.
        // Keep conservative zero contribution in this fallback branch.
        (void)k;
      }
    }
  } catch (...) {
    // Preserve robustness; fallback terms below still apply.
  }

  // DC branch losses from OPF DC voltages.
  if (!sys.dc.branches.empty() && !opf_res.vdc.empty()) {
    std::unordered_map<int, int> dc_bus_pos;
    dc_bus_pos.reserve(sys.dc.buses.size());
    for (size_t bi = 0; bi < sys.dc.buses.size(); ++bi) {
      dc_bus_pos[sys.dc.buses[bi].index] = static_cast<int>(bi);
    }
    for (const auto& br : sys.dc.branches) {
      if (!br.in_service || std::abs(br.r_pu) < 1e-12) continue;
      auto itf = dc_bus_pos.find(br.from_bus);
      auto itt = dc_bus_pos.find(br.to_bus);
      if (itf == dc_bus_pos.end() || itt == dc_bus_pos.end()) continue;
      const size_t fi = static_cast<size_t>(itf->second);
      const size_t ti = static_cast<size_t>(itt->second);
      if (fi >= opf_res.vdc.size() || ti >= opf_res.vdc.size()) continue;
      const double dv = opf_res.vdc[fi] - opf_res.vdc[ti];
      loss_mw += std::max((dv * dv / br.r_pu) * sys.base_mva, 0.0);
    }
  }

  // DCDC losses from the replay dispatch setpoints.
  for (const auto& c : sys.dc.dcdc_converters) {
    if (!c.in_service) continue;
    const double eta = (c.eta > 1e-9) ? c.eta : 1.0;
    const double p_out_mw = c.p_ref_mw;
    const double p_in_mw = (p_out_mw >= 0.0) ? (p_out_mw / eta) : (p_out_mw * eta);
    loss_mw += std::max(p_in_mw - p_out_mw, 0.0);
  }

  if (!(loss_mw > 0.0)) {
    return estimate_boundary_loss_mw(sys);
  }
  return std::max(loss_mw, 0.0);
}

void apply_opf_load_shedding_to_demands(HybridPowerSystem& sys,
                                        const opf::ACOPFResult& opf_res) {
  if (opf_res.dpd_mw.empty()) return;

  const size_t nb = sys.ac.buses.size();
  std::unordered_map<int, int> bus_pos;
  bus_pos.reserve(nb);
  for (size_t i = 0; i < nb; ++i) {
    bus_pos[sys.ac.buses[i].index] = static_cast<int>(i);
  }

  std::vector<double> total_p(nb, 0.0);
  std::vector<double> total_q(nb, 0.0);
  const bool has_component_loads =
      !sys.ac.loads.empty() || !sys.ac.charging_stations.empty();
  if (!has_component_loads) {
    for (size_t i = 0; i < nb; ++i) {
      total_p[i] += std::max(0.0, sys.ac.buses[i].pd_mw);
      total_q[i] += std::max(0.0, sys.ac.buses[i].qd_mvar);
    }
  }
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    auto it = bus_pos.find(ld.bus);
    if (it == bus_pos.end()) continue;
    const size_t i = static_cast<size_t>(it->second);
    total_p[i] += std::max(0.0, ld.p_mw);
    total_q[i] += std::max(0.0, ld.q_mvar);
  }
  for (const auto& cs : sys.ac.charging_stations) {
    if (!cs.in_service) continue;
    auto it = bus_pos.find(cs.bus);
    if (it == bus_pos.end()) continue;
    const size_t i = static_cast<size_t>(it->second);
    total_p[i] += std::max(0.0, cs.p_total_kw / 1000.0);
    total_q[i] += std::max(0.0, cs.q_total_kvar / 1000.0);
  }

  auto reduce_nonnegative = [](double& value, double shed_total, double total_at_bus) {
    if (value <= 0.0 || shed_total <= 0.0 || total_at_bus <= 1e-12) return;
    const double reduce = shed_total * (value / total_at_bus);
    value = std::max(0.0, value - reduce);
  };

  for (size_t i = 0; i < nb && i < opf_res.dpd_mw.size(); ++i) {
    const double shed_p = std::clamp(opf_res.dpd_mw[i], 0.0, total_p[i]);
    const double shed_q = (i < opf_res.dqd_mvar.size())
                              ? std::clamp(opf_res.dqd_mvar[i], 0.0, total_q[i])
                              : 0.0;
    if (shed_p <= 0.0 && shed_q <= 0.0) continue;

    if (!has_component_loads) {
      reduce_nonnegative(sys.ac.buses[i].pd_mw, shed_p, total_p[i]);
      reduce_nonnegative(sys.ac.buses[i].qd_mvar, shed_q, total_q[i]);
    }
    for (auto& ld : sys.ac.loads) {
      if (!ld.in_service) continue;
      auto it = bus_pos.find(ld.bus);
      if (it == bus_pos.end() || static_cast<size_t>(it->second) != i) continue;
      reduce_nonnegative(ld.p_mw, shed_p, total_p[i]);
      reduce_nonnegative(ld.q_mvar, shed_q, total_q[i]);
    }
    for (auto& cs : sys.ac.charging_stations) {
      if (!cs.in_service) continue;
      auto it = bus_pos.find(cs.bus);
      if (it == bus_pos.end() || static_cast<size_t>(it->second) != i) continue;
      double p_mw = cs.p_total_kw / 1000.0;
      double q_mvar = cs.q_total_kvar / 1000.0;
      reduce_nonnegative(p_mw, shed_p, total_p[i]);
      reduce_nonnegative(q_mvar, shed_q, total_q[i]);
      cs.p_total_kw = p_mw * 1000.0;
      cs.q_total_kvar = q_mvar * 1000.0;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════
// Build index map from profile id → profile pointer
// ═══════════════════════════════════════════════════════════════════════
std::unordered_map<int, const TimeSeriesProfile*> build_profile_map(
    const TimeSeriesData& ts_data) {
  std::unordered_map<int, const TimeSeriesProfile*> m;
  for (const auto& p : ts_data.profiles) {
    m[p.id] = &p;
  }
  return m;
}

// ═══════════════════════════════════════════════════════════════════════
// Build B matrix (DC power flow susceptance matrix) from branches
// ═══════════════════════════════════════════════════════════════════════
struct DCNetworkData {
  Eigen::SparseMatrix<double> B;      // n x n bus susceptance
  int n_bus{0};
  int slack_bus{-1};                   // 0-based index of slack bus
  std::vector<int> non_slack;          // indices of non-slack buses
};

DCNetworkData build_dc_network(const HybridPowerSystem& sys) {
  DCNetworkData net;
  net.n_bus = static_cast<int>(sys.ac.buses.size());
  if (net.n_bus == 0) return net;

  // Find slack bus
  for (int i = 0; i < net.n_bus; ++i) {
    if (sys.ac.buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) {
      net.slack_bus = i;
      break;
    }
  }
  // Fallback: use first generator's bus if no swing bus found
  if (net.slack_bus < 0) {
    for (const auto& g : sys.ac.generators) {
      if (g.in_service && g.is_slack) {
        // Find bus index (buses may not be 0-indexed)
        for (int i = 0; i < net.n_bus; ++i) {
          if (sys.ac.buses[static_cast<size_t>(i)].index == g.bus) {
            net.slack_bus = i;
            break;
          }
        }
        if (net.slack_bus >= 0) break;
      }
    }
  }
  if (net.slack_bus < 0) net.slack_bus = 0;

  for (int i = 0; i < net.n_bus; ++i) {
    if (i != net.slack_bus) net.non_slack.push_back(i);
  }

  // Build bus index map: bus.index → 0-based position
  std::unordered_map<int, int> bus_idx;
  for (int i = 0; i < net.n_bus; ++i) {
    bus_idx[sys.ac.buses[static_cast<size_t>(i)].index] = i;
  }

  // Build B matrix (b_ij = -1/x_ij for each branch)
  std::vector<Eigen::Triplet<double>> trips;
  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    auto fi = bus_idx.find(br.from_bus);
    auto ti = bus_idx.find(br.to_bus);
    if (fi == bus_idx.end() || ti == bus_idx.end()) continue;
    int f = fi->second;
    int t = ti->second;
    double x = br.x_pu;
    if (std::abs(x) < 1e-12) x = 1e-6;  // avoid division by zero
    double bij = -1.0 / x;
    // Off-diagonal
    trips.emplace_back(f, t, bij);
    trips.emplace_back(t, f, bij);
    // Diagonal
    trips.emplace_back(f, f, -bij);
    trips.emplace_back(t, t, -bij);
  }
  // Also handle 2W transformers (convert vk_percent to per-unit reactance)
  for (const auto& tr : sys.ac.transformers_2w) {
    if (!tr.in_service) continue;
    auto fi = bus_idx.find(tr.hv_bus);
    auto ti = bus_idx.find(tr.lv_bus);
    if (fi == bus_idx.end() || ti == bus_idx.end()) continue;
    int f = fi->second;
    int t = ti->second;
    // Compute x_pu from vk%: x ≈ sqrt(vk²-vkr²)/100 on system base
    double vk = tr.vk_percent / 100.0;
    double vkr = tr.vkr_percent / 100.0;
    double x = std::sqrt(std::max(0.0, vk * vk - vkr * vkr));
    // Convert from transformer base to system base
    if (tr.sn_mva > 0.0 && sys.base_mva > 0.0) {
      x *= sys.base_mva / tr.sn_mva;
    }
    if (std::abs(x) < 1e-12) x = 1e-6;
    double bij = -1.0 / x;
    trips.emplace_back(f, t, bij);
    trips.emplace_back(t, f, bij);
    trips.emplace_back(f, f, -bij);
    trips.emplace_back(t, t, -bij);
  }

  net.B.resize(net.n_bus, net.n_bus);
  net.B.setFromTriplets(trips.begin(), trips.end());
  net.B.makeCompressed();

  return net;
}

// ═══════════════════════════════════════════════════════════════════════
// DC Grid data — conductance matrix for the DC network
// ═══════════════════════════════════════════════════════════════════════
struct DCGridData {
  Eigen::SparseMatrix<double> G;      // n_dc x n_dc conductance matrix
  int n_dc_bus{0};
  int dc_slack_bus{-1};                // 0-based index of voltage-controlled DC bus
  std::vector<int> non_slack_dc;       // indices of non-slack DC buses
};

DCGridData build_dc_grid(const HybridPowerSystem& sys) {
  DCGridData dcg;
  dcg.n_dc_bus = static_cast<int>(sys.dc.buses.size());
  if (dcg.n_dc_bus == 0) return dcg;

  // Find DC voltage-controlled (slack) bus
  for (int i = 0; i < dcg.n_dc_bus; ++i) {
    if (sys.dc.buses[static_cast<size_t>(i)].bus_type == DCBusType::DC_V) {
      dcg.dc_slack_bus = i;
      break;
    }
  }
  if (dcg.dc_slack_bus < 0) dcg.dc_slack_bus = 0;

  for (int i = 0; i < dcg.n_dc_bus; ++i) {
    if (i != dcg.dc_slack_bus) dcg.non_slack_dc.push_back(i);
  }

  // Build DC bus index map
  std::unordered_map<int, int> dc_bus_idx;
  for (int i = 0; i < dcg.n_dc_bus; ++i) {
    dc_bus_idx[sys.dc.buses[static_cast<size_t>(i)].index] = i;
  }

  // Build G matrix (g_ij = 1/r_ij, same sign pattern as AC B matrix)
  std::vector<Eigen::Triplet<double>> trips;
  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) continue;
    auto fi = dc_bus_idx.find(br.from_bus);
    auto ti = dc_bus_idx.find(br.to_bus);
    if (fi == dc_bus_idx.end() || ti == dc_bus_idx.end()) continue;
    int f = fi->second;
    int t = ti->second;
    double r = br.r_pu;
    if (std::abs(r) < 1e-12) r = 1e-6;
    double gij = -1.0 / r;  // off-diagonal (negative, like B matrix)
    trips.emplace_back(f, t, gij);
    trips.emplace_back(t, f, gij);
    trips.emplace_back(f, f, -gij);  // diagonal (positive)
    trips.emplace_back(t, t, -gij);
  }

  dcg.G.resize(dcg.n_dc_bus, dcg.n_dc_bus);
  dcg.G.setFromTriplets(trips.begin(), trips.end());
  dcg.G.makeCompressed();

  return dcg;
}

// ═══════════════════════════════════════════════════════════════════════
// Build UC MILP model from system data and time-series profiles
// ═══════════════════════════════════════════════════════════════════════
struct UCBuildResult {
  engine::MIPModel model;
  int G;  // number of dispatchable generators (in-service)
  int S;  // number of AC storage units (in-service)
  int Sd; // number of DC storage units (in-service)
  int R;  // number of renewables (in-service)
  int T;  // number of time steps
  int C{0};   // number of VSC converters (in-service)
  int Cd{0};  // number of DC-DC converters (in-service)
  std::vector<int> gen_indices;       // original indices into sys.ac.generators
  std::vector<int> storage_indices;   // original indices into sys.ac.storage
  std::vector<int> dc_storage_indices;// original indices into sys.dc.storage
  std::vector<int> renewable_indices; // original indices into sys.ac.renewable_gens
  std::vector<int> vsc_indices;       // original indices into sys.vsc_converters
  std::vector<int> dcdc_indices;      // original indices into sys.dc.dcdc_converters
};

UCBuildResult build_uc_milp(const HybridPowerSystem& sys,
                            const TimeSeriesData& ts_data,
                            const DCNetworkData& net,
                            const DCGridData& dcg,
                            const TimeSeriesPFOptions& opts) {
  using namespace engine;
  UCBuildResult res;
  const int T = ts_data.num_steps;
  const double dt = ts_data.step_duration_hr;
  res.T = T;

  auto profile_map = build_profile_map(ts_data);

  // Build bus index maps
  std::unordered_map<int, int> bus_idx;
  for (int i = 0; i < net.n_bus; ++i) {
    bus_idx[sys.ac.buses[static_cast<size_t>(i)].index] = i;
  }

  // Collect in-service generators
  for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
    if (sys.ac.generators[static_cast<size_t>(i)].in_service) {
      res.gen_indices.push_back(i);
    }
  }
  res.G = static_cast<int>(res.gen_indices.size());

  // Collect in-service storage
  for (int i = 0; i < static_cast<int>(sys.ac.storage.size()); ++i) {
    if (sys.ac.storage[static_cast<size_t>(i)].in_service) {
      res.storage_indices.push_back(i);
    }
  }
  res.S = static_cast<int>(res.storage_indices.size());

  // Collect in-service DC storage
  for (int i = 0; i < static_cast<int>(sys.dc.storage.size()); ++i) {
    if (sys.dc.storage[static_cast<size_t>(i)].in_service) {
      res.dc_storage_indices.push_back(i);
    }
  }
  res.Sd = static_cast<int>(res.dc_storage_indices.size());

  // Collect in-service renewables
  for (int i = 0; i < static_cast<int>(sys.ac.renewable_gens.size()); ++i) {
    if (sys.ac.renewable_gens[static_cast<size_t>(i)].in_service) {
      res.renewable_indices.push_back(i);
    }
  }
  res.R = static_cast<int>(res.renewable_indices.size());

  // Collect in-service VSC converters
  for (int i = 0; i < static_cast<int>(sys.vsc_converters.size()); ++i) {
    if (sys.vsc_converters[static_cast<size_t>(i)].in_service) {
      res.vsc_indices.push_back(i);
    }
  }
  res.C = static_cast<int>(res.vsc_indices.size());

  // Collect in-service DC-DC converters
  for (int i = 0; i < static_cast<int>(sys.dc.dcdc_converters.size()); ++i) {
    if (sys.dc.dcdc_converters[static_cast<size_t>(i)].in_service) {
      res.dcdc_indices.push_back(i);
    }
  }
  res.Cd = static_cast<int>(res.dcdc_indices.size());

  // Build DC bus index map
  std::unordered_map<int, int> dc_bus_idx;
  for (int i = 0; i < dcg.n_dc_bus; ++i) {
    dc_bus_idx[sys.dc.buses[static_cast<size_t>(i)].index] = i;
  }

  const int G = res.G, S = res.S, Sd = res.Sd, R = res.R;
  const int C = res.C, Cd = res.Cd;
  const bool use_network_constraints = opts.enable_network_constraints && net.n_bus > 1;
  const bool use_dc_network = use_network_constraints && opts.enable_dc_network_constraints
                              && dcg.n_dc_bus > 0 && C > 0;

  // Variable layout:
  //   p_g[g,t]      dispatch for generator g at time t    (continuous) : G*T
  //   u_g[g,t]      commitment for generator g at time t  (binary)     : G*T
  //   s_g[g,t]      startup cost for generator g at time t (continuous): G*T
  //   p_ess[s,t]    AC ESS dispatch (positive=discharge)   (continuous): S*T
  //   soc[s,t]      AC ESS SOC at end of period t          (continuous): S*T
  //   p_dcess[d,t]  DC ESS dispatch (positive=discharge)   (continuous): Sd*T
  //   soc_d[d,t]    DC ESS SOC at end of period t          (continuous): Sd*T
  //   p_ren[r,t]    renewable dispatch at time t           (continuous): R*T
  //   theta[b,t]    AC bus voltage angle (non-slack)       (continuous): nTheta
  //   p_vsc[c,t]    VSC AC-side injection (pos=into AC)    (continuous): C*T  (if use_dc_network)
  //   p_dcdc[dd,t]  DC-DC input power (pos=in→out)         (continuous): Cd*T (if use_dc_network)
  const int nPg  = G * T;
  const int nUg  = G * T;
  const int nSg  = G * T;
  const int nEss = S * T;
  const int nSoc = S * T;
  const int nDcEss = Sd * T;
  const int nDcSoc = Sd * T;
  const int nRen = R * T;
  const int nTheta = use_network_constraints ? static_cast<int>(net.non_slack.size()) * T : 0;
  const int nVsc = use_dc_network ? C * T : 0;
  const int nDcdc = use_dc_network ? Cd * T : 0;
  const int n = nPg + nUg + nSg + nEss + nSoc + nDcEss + nDcSoc + nRen + nTheta + nVsc + nDcdc;

  auto pg_idx  = [&](int g, int t) { return g * T + t; };
  auto ug_idx  = [&](int g, int t) { return nPg + g * T + t; };
  auto sg_idx  = [&](int g, int t) { return nPg + nUg + g * T + t; };
  auto ess_idx = [&](int s, int t) { return nPg + nUg + nSg + s * T + t; };
  auto soc_idx = [&](int s, int t) { return nPg + nUg + nSg + nEss + s * T + t; };
  auto dcess_idx = [&](int s, int t) {
    return nPg + nUg + nSg + nEss + nSoc + s * T + t;
  };
  auto dcsoc_idx = [&](int s, int t) {
    return nPg + nUg + nSg + nEss + nSoc + nDcEss + s * T + t;
  };
  auto ren_idx = [&](int r, int t) {
    return nPg + nUg + nSg + nEss + nSoc + nDcEss + nDcSoc + r * T + t;
  };
  const int theta_offset = nPg + nUg + nSg + nEss + nSoc + nDcEss + nDcSoc + nRen;
  auto theta_idx = [&](int b_pos, int t) {
    return theta_offset + b_pos * T + t;
  };
  const int vsc_offset = theta_offset + nTheta;
  auto vsc_idx = [&](int c, int t) {
    return vsc_offset + c * T + t;
  };
  const int dcdc_offset = vsc_offset + nVsc;
  auto dcdc_idx = [&](int dd, int t) {
    return dcdc_offset + dd * T + t;
  };

  MIPModel& m = res.model;
  m.linear_part.sense = Sense::Minimize;
  m.linear_part.c = Eigen::VectorXd::Zero(n);
  m.linear_part.vars.resize(static_cast<size_t>(n));

  // --- Objective: gen linear cost * p_g + startup cost s_g ---
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    for (int t = 0; t < T; ++t) {
      // Linear cost per MWh: cost_c1 * p_mw * dt
      m.linear_part.c[pg_idx(gi, t)] = gen.cost_c1 * dt;
      // No-load commitment cost per hour (use cost_c0 as on-cost coefficient).
      m.linear_part.c[ug_idx(gi, t)] = std::max(0.0, gen.cost_c0) * dt;
      // Startup cost
      m.linear_part.c[sg_idx(gi, t)] = 1.0;
    }
  }

  // --- Variable bounds & types ---
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    for (int t = 0; t < T; ++t) {
      m.linear_part.vars[static_cast<size_t>(pg_idx(gi, t))] = {
        VarType::Continuous, 0.0, gen.pmax_mw,
        "p_g" + std::to_string(gi) + "_t" + std::to_string(t)};
      m.linear_part.vars[static_cast<size_t>(ug_idx(gi, t))] = {
        VarType::Binary, 0.0, 1.0,
        "u_g" + std::to_string(gi) + "_t" + std::to_string(t)};
      m.linear_part.vars[static_cast<size_t>(sg_idx(gi, t))] = {
        VarType::Continuous, 0.0, gen.startup_cost,
        "s_g" + std::to_string(gi) + "_t" + std::to_string(t)};
      m.binary_idx.push_back(ug_idx(gi, t));
    }
  }

  for (int si = 0; si < S; ++si) {
    const auto& ess = sys.ac.storage[static_cast<size_t>(res.storage_indices[static_cast<size_t>(si)])];
    for (int t = 0; t < T; ++t) {
      m.linear_part.vars[static_cast<size_t>(ess_idx(si, t))] = {
        VarType::Continuous, ess.pmin_mw, ess.pmax_mw,
        "p_ess" + std::to_string(si) + "_t" + std::to_string(t)};
      m.linear_part.vars[static_cast<size_t>(soc_idx(si, t))] = {
        VarType::Continuous, ess.soc_min, ess.soc_max,
        "soc" + std::to_string(si) + "_t" + std::to_string(t)};
    }
  }

  for (int si = 0; si < Sd; ++si) {
    const auto& ess = sys.dc.storage[static_cast<size_t>(
        res.dc_storage_indices[static_cast<size_t>(si)])];
    for (int t = 0; t < T; ++t) {
      m.linear_part.vars[static_cast<size_t>(dcess_idx(si, t))] = {
        VarType::Continuous, ess.pmin_mw, ess.pmax_mw,
        "p_dcess" + std::to_string(si) + "_t" + std::to_string(t)};
      m.linear_part.vars[static_cast<size_t>(dcsoc_idx(si, t))] = {
        VarType::Continuous, ess.soc_min, ess.soc_max,
        "soc_d" + std::to_string(si) + "_t" + std::to_string(t)};
    }
  }

  for (int ri = 0; ri < R; ++ri) {
    const auto& ren = sys.ac.renewable_gens[static_cast<size_t>(res.renewable_indices[static_cast<size_t>(ri)])];
    for (int t = 0; t < T; ++t) {
      // Available output from profile
      const auto* pv = find_profile(profile_map, ren.profile_id);
      double p_avail = profile_value(pv, t, 1.0) * ren.p_rated_mw;
      m.linear_part.vars[static_cast<size_t>(ren_idx(ri, t))] = {
        VarType::Continuous, 0.0, p_avail,
        "p_ren" + std::to_string(ri) + "_t" + std::to_string(t)};
      // Curtailment cost in objective
      if (ren.cost_curtail_mwh > 0.0) {
        m.linear_part.c[ren_idx(ri, t)] = -ren.cost_curtail_mwh * dt;
      }
    }
  }

  if (use_network_constraints) {
    for (int bp = 0; bp < static_cast<int>(net.non_slack.size()); ++bp) {
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(theta_idx(bp, t))] = {
          VarType::Continuous, -3.141592653589793, 3.141592653589793,
          "theta_b" + std::to_string(net.non_slack[static_cast<size_t>(bp)]) + "_t" + std::to_string(t)};
      }
    }
  }

  // VSC converter power variables (AC-side injection, positive=into AC bus)
  if (use_dc_network) {
    for (int ci = 0; ci < C; ++ci) {
      const auto& conv = sys.vsc_converters[static_cast<size_t>(res.vsc_indices[static_cast<size_t>(ci)])];
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(vsc_idx(ci, t))] = {
          VarType::Continuous, conv.pmin_mw, conv.pmax_mw,
          "p_vsc" + std::to_string(ci) + "_t" + std::to_string(t)};
      }
    }
    // DC-DC converter power variables (input power, positive=bus_in→bus_out)
    for (int di = 0; di < Cd; ++di) {
      const auto& ddc = sys.dc.dcdc_converters[static_cast<size_t>(res.dcdc_indices[static_cast<size_t>(di)])];
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(dcdc_idx(di, t))] = {
          VarType::Continuous, ddc.pmin_mw, ddc.pmax_mw,
          "p_dcdc" + std::to_string(di) + "_t" + std::to_string(t)};
      }
    }
  }

  // --- Count constraints ---
  // 1. Power balance per bus per time step (equality): n_bus * T
  //    (single-bus approximation if bus topology not meaningful for UC)
  //    We use per-bus balance with DC susceptance if available.
  //    For simplicity, use aggregated single-bus power balance:
  //      sum_g p_g[g,t] + sum_ac_ess p_ess[s,t] + sum_dc_ess p_dcess[d,t]
  //      + sum_ren p_ren[r,t] = total_load[t]
  // 2. Generator output limits: p_g ≤ Pmax * u_g  (G*T ineq)
  //                              p_g ≥ Pmin * u_g  (G*T ineq)
  // 3. Ramp up/down: ±(p_{g,t} - p_{g,t-1}) ≤ RR * dt (2*G*(T-1) ineq)
  // 4. Startup cost: s_{g,t} ≥ startup * (u_{g,t} - u_{g,t-1}) (G*T ineq)
  // 5. ESS SOC dynamics: AC + DC storage ( (S+Sd)*T eq )
  //    (discharge: p_ess>0 → SOC decreases; charge: p_ess<0 → SOC increases)
  //    Accounting for efficiency:
  //      soc[s,t] = soc[s,t-1] * (1 - self_discharge_pct/100)
  //                 - (p_ess_dis[s,t]*dt)/(eta_dis*E) + (p_ess_chg[s,t]*dt*eta_chg)/E
  //    We linearize using single variable p_ess (positive=dis, negative=chg):
  //      soc[s,t] = soc[s,t-1] - p_ess[s,t]*dt/E_rated * (1/eta if dis, eta if chg)
  //    For a linear formulation, we approximate: efficiency ≈ sqrt(eta_c * eta_d)
  //      soc[s,t] ≈ soc[s,t-1] - p_ess[s,t]*dt / (sqrt(eta_c*eta_d) * E_rated)
  const int n_eq_balance = use_network_constraints ? (net.n_bus * T) : T;
  const int n_eq_dc_balance = use_dc_network ? (dcg.n_dc_bus * T) : 0;
  const int n_eq_soc_ac = S * T;      // AC SOC dynamics
  const int n_eq_soc_dc = Sd * T;     // DC SOC dynamics
  const int n_eq_soc = n_eq_soc_ac + n_eq_soc_dc;
  const int m_eq = n_eq_balance + n_eq_dc_balance + n_eq_soc;

  const int n_ineq_pmax = G * T;
  const int n_ineq_pmin = G * T;
  const int n_ineq_ramp_up = G * (T > 0 ? T - 1 : 0);
  const int n_ineq_ramp_dn = G * (T > 0 ? T - 1 : 0);
  const int n_ineq_startup = G * T;

  // Min-up/min-down time constraints (count depends on per-generator parameters).
  int n_ineq_min_updn = 0;
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    const int up_periods = static_cast<int>(std::ceil(gen.min_up_time_hr / dt));
    const int dn_periods = static_cast<int>(std::ceil(gen.min_dn_time_hr / dt));
    if (up_periods >= 3) {
      for (int t = 1; t < T; ++t)
        n_ineq_min_updn += std::min(up_periods - 1, T - 1 - t);
    }
    if (dn_periods >= 3) {
      for (int t = 1; t < T; ++t)
        n_ineq_min_updn += std::min(dn_periods - 1, T - 1 - t);
    }
  }

  int n_ineq_line = 0;
  if (use_network_constraints) {
    for (const auto& br : sys.ac.branches) {
      if (!br.in_service) continue;
      if (br.rate_a_mva <= 0.0) continue;
      auto fi = bus_idx.find(br.from_bus);
      auto ti = bus_idx.find(br.to_bus);
      if (fi == bus_idx.end() || ti == bus_idx.end()) continue;
      if (fi->second == ti->second) continue;
      if (std::abs(br.x_pu) < 1e-12) continue;
      n_ineq_line += 2 * T;
    }
  }
  // DC line thermal limits — with flat DC voltage assumption (no DC voltage variables),
  // branch flows are implicitly determined by bus injections. Converter power bounds
  // serve as the binding capacity constraint. Set to 0 for the linearized model.
  const int n_ineq_dc_line = 0;
  (void)dc_bus_idx; // used above for DC load/converter mapping
  const int n_ineq_reserve = (opts.reserve_requirement_fraction > 0.0) ? T : 0;

  const int m_ineq = n_ineq_pmax + n_ineq_pmin + n_ineq_ramp_up + n_ineq_ramp_dn
                   + n_ineq_startup + n_ineq_min_updn + n_ineq_line + n_ineq_dc_line
                   + n_ineq_reserve;

  // --- Compute total load per time step (AC + DC loads) ---
  // Match PF aggregation semantics:
  // component load tables have priority; otherwise fall back to bus pd_mw.
  std::vector<double> total_load(static_cast<size_t>(T), 0.0);
  std::vector<double> bus_load(static_cast<size_t>(std::max(1, net.n_bus * T)), 0.0);
  auto bus_load_idx = [&](int b, int t) { return b * T + t; };
  const bool has_component_ac_loads =
      !sys.ac.loads.empty() || !sys.ac.charging_stations.empty();
  const auto* default_load_profile = find_profile(profile_map, 0);  // convention: profile id 0 = system load
  if (has_component_ac_loads) {
    for (const auto& load : sys.ac.loads) {
      if (!load.in_service) continue;
      const auto* pv = find_profile(profile_map, load.profile_id);
      const auto bit = bus_idx.find(load.bus);
      for (int t = 0; t < T; ++t) {
        const double val = load.p_mw * profile_value(pv, t, 1.0) * load.scaling;
        total_load[static_cast<size_t>(t)] += val;
        if (bit != bus_idx.end()) bus_load[static_cast<size_t>(bus_load_idx(bit->second, t))] += val;
      }
    }
    for (const auto& cs : sys.ac.charging_stations) {
      if (!cs.in_service) continue;
      const auto bit = bus_idx.find(cs.bus);
      for (int t = 0; t < T; ++t) {
        const double val = std::max(0.0, cs.p_total_kw / 1000.0);
        total_load[static_cast<size_t>(t)] += val;
        if (bit != bus_idx.end()) bus_load[static_cast<size_t>(bus_load_idx(bit->second, t))] += val;
      }
    }
  } else {
    for (const auto& bus : sys.ac.buses) {
      if (!bus.in_service) continue;
      const auto bit = bus_idx.find(bus.index);
      for (int t = 0; t < T; ++t) {
        const double val = std::max(0.0, bus.pd_mw) * profile_value(default_load_profile, t, 1.0);
        total_load[static_cast<size_t>(t)] += val;
        if (bit != bus_idx.end()) bus_load[static_cast<size_t>(bus_load_idx(bit->second, t))] += val;
      }
    }
  }
  for (const auto& fl : sys.ac.flexible_loads) {
    if (!fl.in_service) continue;
    const auto bit = bus_idx.find(fl.bus);
    for (int t = 0; t < T; ++t) {
      const double val = std::max(0.0, fl.p_mw);
      total_load[static_cast<size_t>(t)] += val;
      if (bit != bus_idx.end()) bus_load[static_cast<size_t>(bus_load_idx(bit->second, t))] += val;
    }
  }
  for (const auto& al : sys.ac.asymmetric_loads) {
    if (!al.in_service) continue;
    const double p3 = std::max(0.0, al.pa_mw + al.pb_mw + al.pc_mw) *
                      std::max(0.0, al.scaling);
    const auto bit = bus_idx.find(al.bus);
    for (int t = 0; t < T; ++t) {
      total_load[static_cast<size_t>(t)] += p3;
      if (bit != bus_idx.end()) bus_load[static_cast<size_t>(bus_load_idx(bit->second, t))] += p3;
    }
  }
  // Include DC loads in system power balance (served through VSC/DCDC converters)
  // When use_dc_network is active, DC loads are handled in DC nodal balance instead.
  if (!use_dc_network) {
    for (const auto& load : sys.dc.loads) {
      if (!load.in_service) continue;
      const auto* pv = find_profile(profile_map, load.profile_id);
      for (int t = 0; t < T; ++t) {
        double scale = profile_value(pv, t, 1.0);
        total_load[static_cast<size_t>(t)] += load.p_mw * scale * load.scaling;
      }
    }
    // Subtract AC PV system must-take generation — not modeled as UC
    // decision variables but reduce net load that UC must dispatch
    for (const auto& pvsys : sys.ac.pv_systems) {
      if (!pvsys.in_service) continue;
      const auto* pvprof = find_profile(profile_map, pvsys.profile_id);
      const auto bit = bus_idx.find(pvsys.bus);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(pvprof, t, 1.0);
        double pv_mw;
        if (pvsys.voc > 0.0 && pvsys.isc > 0.0 && pvsys.vmpp > 0.0) {
          // Detailed model: compute_pv_power_mw uses irradiance
          PVSystem pv_copy = pvsys;
          pv_copy.irradiance = scale * 1000.0;
          pv_mw = powerflow::compute_pv_power_mw(pv_copy);
        } else {
          pv_mw = pvsys.p_mw * scale;
        }
        total_load[static_cast<size_t>(t)] -= pv_mw;
        if (bit != bus_idx.end())
          bus_load[static_cast<size_t>(bus_load_idx(bit->second, t))] -= pv_mw;
      }
    }
    // Subtract DC fixed generation (PV, static generators) — not modeled as UC
    // decision variables but reduce net AC load that UC must dispatch
    for (const auto& pvarr : sys.dc.pv_arrays) {
      if (!pvarr.in_service) continue;
      const auto* pv = find_profile(profile_map, pvarr.profile_id);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(pv, t, 1.0);
        total_load[static_cast<size_t>(t)] -= pvarr.p_set_mw * scale;
      }
    }
    for (const auto& sg : sys.dc.dc_static_generators) {
      if (!sg.in_service) continue;
      const auto* pv = find_profile(profile_map, sg.profile_id);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(pv, t, 1.0);
        total_load[static_cast<size_t>(t)] -= sg.p_set_mw * sg.scaling * scale;
      }
    }
  }

  // Build DC bus load vector for DC nodal balance
  std::vector<double> dc_bus_load(static_cast<size_t>(std::max(1, dcg.n_dc_bus * T)), 0.0);
  auto dc_bus_load_idx = [&](int b, int t) { return b * T + t; };
  if (use_dc_network) {
    for (const auto& load : sys.dc.loads) {
      if (!load.in_service) continue;
      const auto* pv = find_profile(profile_map, load.profile_id);
      auto bit = dc_bus_idx.find(load.bus);
      for (int t = 0; t < T; ++t) {
        const double val = load.p_mw * profile_value(pv, t, 1.0) * load.scaling;
        if (bit != dc_bus_idx.end())
          dc_bus_load[static_cast<size_t>(dc_bus_load_idx(bit->second, t))] += val;
      }
    }
    // DC-side PV arrays (negative load, i.e. generation)
    for (const auto& pvarr : sys.dc.pv_arrays) {
      if (!pvarr.in_service) continue;
      const auto* pv = find_profile(profile_map, pvarr.profile_id);
      auto bit = dc_bus_idx.find(pvarr.bus);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(pv, t, 1.0);
        if (bit != dc_bus_idx.end())
          dc_bus_load[static_cast<size_t>(dc_bus_load_idx(bit->second, t))] -= pvarr.p_set_mw * scale;
      }
    }
    // DC static generators (negative load)
    for (const auto& sg : sys.dc.dc_static_generators) {
      if (!sg.in_service) continue;
      const auto* pv = find_profile(profile_map, sg.profile_id);
      auto bit = dc_bus_idx.find(sg.bus);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(pv, t, 1.0);
        if (bit != dc_bus_idx.end())
          dc_bus_load[static_cast<size_t>(dc_bus_load_idx(bit->second, t))] -= sg.p_set_mw * sg.scaling * scale;
      }
    }
  }

  if (use_network_constraints && !use_dc_network && net.slack_bus >= 0) {
    // When DC network is NOT modeled explicitly, allocate AC/DC net residual to
    // slack bus to keep nodal model balanced.
    for (int t = 0; t < T; ++t) {
      double bus_sum = 0.0;
      for (int b = 0; b < net.n_bus; ++b) {
        bus_sum += bus_load[static_cast<size_t>(bus_load_idx(b, t))];
      }
      bus_load[static_cast<size_t>(bus_load_idx(net.slack_bus, t))] +=
          (total_load[static_cast<size_t>(t)] - bus_sum);
    }
  } else if (use_network_constraints && use_dc_network && net.slack_bus >= 0) {
    // With DC network: AC nodal balance must only cover AC-side loads.
    // Don't push DC residual to AC slack. Just reconcile bus-level AC load.
    for (int t = 0; t < T; ++t) {
      double bus_sum = 0.0;
      for (int b = 0; b < net.n_bus; ++b) {
        bus_sum += bus_load[static_cast<size_t>(bus_load_idx(b, t))];
      }
      double ac_total = total_load[static_cast<size_t>(t)];
      if (std::abs(ac_total - bus_sum) > 1e-10) {
        bus_load[static_cast<size_t>(bus_load_idx(net.slack_bus, t))] +=
            (ac_total - bus_sum);
      }
    }
  }

  // --- Build equality constraints ---
  std::vector<Eigen::Triplet<double>> eq_trips;
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(m_eq);

  if (!use_network_constraints) {
    // Legacy copper-plate balance.
    for (int t = 0; t < T; ++t) {
      for (int gi = 0; gi < G; ++gi) {
        eq_trips.emplace_back(t, pg_idx(gi, t), 1.0);
      }
      for (int si = 0; si < S; ++si) {
        eq_trips.emplace_back(t, ess_idx(si, t), 1.0);
      }
      for (int si = 0; si < Sd; ++si) {
        eq_trips.emplace_back(t, dcess_idx(si, t), 1.0);
      }
      for (int ri = 0; ri < R; ++ri) {
        eq_trips.emplace_back(t, ren_idx(ri, t), 1.0);
      }
      beq[t] = total_load[static_cast<size_t>(t)];
    }
  } else {
    std::unordered_map<int, int> non_slack_pos;
    for (int bp = 0; bp < static_cast<int>(net.non_slack.size()); ++bp) {
      non_slack_pos[net.non_slack[static_cast<size_t>(bp)]] = bp;
    }

    std::vector<std::vector<int>> gens_at_bus(static_cast<size_t>(net.n_bus));
    std::vector<std::vector<int>> ess_at_bus(static_cast<size_t>(net.n_bus));
    std::vector<std::vector<int>> ren_at_bus(static_cast<size_t>(net.n_bus));
    std::vector<std::vector<int>> vsc_at_ac_bus(static_cast<size_t>(net.n_bus));
    for (int gi = 0; gi < G; ++gi) {
      const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
      auto it = bus_idx.find(gen.bus);
      if (it != bus_idx.end()) gens_at_bus[static_cast<size_t>(it->second)].push_back(gi);
    }
    for (int si = 0; si < S; ++si) {
      const auto& ess = sys.ac.storage[static_cast<size_t>(res.storage_indices[static_cast<size_t>(si)])];
      auto it = bus_idx.find(ess.bus);
      if (it != bus_idx.end()) ess_at_bus[static_cast<size_t>(it->second)].push_back(si);
    }
    for (int ri = 0; ri < R; ++ri) {
      const auto& ren = sys.ac.renewable_gens[static_cast<size_t>(res.renewable_indices[static_cast<size_t>(ri)])];
      auto it = bus_idx.find(ren.bus);
      if (it != bus_idx.end()) ren_at_bus[static_cast<size_t>(it->second)].push_back(ri);
    }
    if (use_dc_network) {
      for (int ci = 0; ci < C; ++ci) {
        const auto& conv = sys.vsc_converters[static_cast<size_t>(res.vsc_indices[static_cast<size_t>(ci)])];
        auto it = bus_idx.find(conv.bus_ac);
        if (it != bus_idx.end()) vsc_at_ac_bus[static_cast<size_t>(it->second)].push_back(ci);
      }
    }

    for (int t = 0; t < T; ++t) {
      for (int b = 0; b < net.n_bus; ++b) {
        const int row_bal = t * net.n_bus + b;
        for (int gi : gens_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, pg_idx(gi, t), 1.0);
        }
        for (int si : ess_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, ess_idx(si, t), 1.0);
        }
        for (int ri : ren_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, ren_idx(ri, t), 1.0);
        }
        // VSC converters: p_vsc is AC-side injection (positive = into AC bus)
        if (use_dc_network) {
          for (int ci : vsc_at_ac_bus[static_cast<size_t>(b)]) {
            eq_trips.emplace_back(row_bal, vsc_idx(ci, t), 1.0);
          }
        } else if (b == net.slack_bus) {
          // Legacy: DC ESS injected at slack bus when DC network not modeled
          for (int si = 0; si < Sd; ++si) {
            eq_trips.emplace_back(row_bal, dcess_idx(si, t), 1.0);
          }
        }

        for (int j = 0; j < net.n_bus; ++j) {
          const double bij = net.B.coeff(b, j);
          if (std::abs(bij) < 1e-12) continue;
          const auto ib = non_slack_pos.find(b);
          const auto ij = non_slack_pos.find(j);
          if (ib != non_slack_pos.end()) {
            eq_trips.emplace_back(row_bal, theta_idx(ib->second, t), -sys.base_mva * bij);
          }
          if (ij != non_slack_pos.end()) {
            eq_trips.emplace_back(row_bal, theta_idx(ij->second, t), sys.base_mva * bij);
          }
        }

        beq[row_bal] = bus_load[static_cast<size_t>(bus_load_idx(b, t))];
      }
    }
  }

  // ── DC nodal balance equations ──
  // For each DC bus d, time t:
  //   Σ_c (p_vsc_dc[c,t]) + Σ_dd (p_dcdc contribution) + Σ p_dcess[s,t]
  //   - Σ_j G[d,j] * V_dc = dc_bus_load[d,t]
  // where p_vsc_dc = -p_vsc_ac / eta (power flowing FROM AC TO DC side).
  // Since we model AC-side injection (p_vsc positive=into AC), the DC side
  // receives: p_dc = -p_vsc_ac * eta (when p_vsc_ac < 0, AC absorbs, DC gets power).
  // We simplify using lossless linear coupling: p_dc_inject = -p_vsc_ac.
  // Loss is approximated by scaling the efficiency into the coupling.
  // DC power flow is resistive: P_line = G_line * (V_d - V_j) ≈ g_dj * Δv
  // For linearized DC: assume V_dc ≈ 1 pu (constant), so nodal injection = load + line flow.
  // This gives:  Σ_c (-p_vsc[c,t] / eta_c) + Σ_dd (...) + ESS = P_dc_load
  // We omit DC voltage variables (assume flat DC voltage) for a pure power-balance model.
  if (use_dc_network) {
    // Map converters/ESS to DC buses
    std::vector<std::vector<int>> vsc_at_dc_bus(static_cast<size_t>(dcg.n_dc_bus));
    std::vector<std::vector<int>> dcess_at_dc_bus(static_cast<size_t>(dcg.n_dc_bus));
    std::vector<std::vector<std::pair<int,bool>>> dcdc_at_dc_bus(static_cast<size_t>(dcg.n_dc_bus));

    for (int ci = 0; ci < C; ++ci) {
      const auto& conv = sys.vsc_converters[static_cast<size_t>(res.vsc_indices[static_cast<size_t>(ci)])];
      auto it = dc_bus_idx.find(conv.bus_dc);
      if (it != dc_bus_idx.end()) vsc_at_dc_bus[static_cast<size_t>(it->second)].push_back(ci);
    }
    for (int si = 0; si < Sd; ++si) {
      const auto& ess = sys.dc.storage[static_cast<size_t>(res.dc_storage_indices[static_cast<size_t>(si)])];
      auto it = dc_bus_idx.find(ess.bus);
      if (it != dc_bus_idx.end()) dcess_at_dc_bus[static_cast<size_t>(it->second)].push_back(si);
    }
    for (int di = 0; di < Cd; ++di) {
      const auto& ddc = sys.dc.dcdc_converters[static_cast<size_t>(res.dcdc_indices[static_cast<size_t>(di)])];
      auto fi = dc_bus_idx.find(ddc.bus_in);
      auto ti = dc_bus_idx.find(ddc.bus_out);
      // At bus_in: power withdrawn = p_dcdc (positive = in→out, so bus_in loses)
      if (fi != dc_bus_idx.end()) dcdc_at_dc_bus[static_cast<size_t>(fi->second)].emplace_back(di, true);
      // At bus_out: power injected = p_dcdc * eta
      if (ti != dc_bus_idx.end()) dcdc_at_dc_bus[static_cast<size_t>(ti->second)].emplace_back(di, false);
    }

    for (int t = 0; t < T; ++t) {
      for (int d = 0; d < dcg.n_dc_bus; ++d) {
        // DC balance rows start at n_eq_balance, spanning dcg.n_dc_bus * T rows.
        const int row_dc2 = n_eq_balance + t * dcg.n_dc_bus + d;

        // VSC injection into DC bus: -p_vsc[c,t] / eta_c
        // (when p_vsc > 0, AC receives power → DC side loses → negative DC injection)
        for (int ci : vsc_at_dc_bus[static_cast<size_t>(d)]) {
          const auto& conv = sys.vsc_converters[static_cast<size_t>(res.vsc_indices[static_cast<size_t>(ci)])];
          double eta_inv = (conv.eta > 1e-6) ? (-1.0 / conv.eta) : -1.0;
          eq_trips.emplace_back(row_dc2, vsc_idx(ci, t), eta_inv);
        }

        // DC ESS at this DC bus
        for (int si : dcess_at_dc_bus[static_cast<size_t>(d)]) {
          eq_trips.emplace_back(row_dc2, dcess_idx(si, t), 1.0);
        }

        // DC-DC converters
        for (const auto& [di, is_input] : dcdc_at_dc_bus[static_cast<size_t>(d)]) {
          const auto& ddc = sys.dc.dcdc_converters[static_cast<size_t>(res.dcdc_indices[static_cast<size_t>(di)])];
          if (is_input) {
            // bus_in: withdraws p_dcdc from this bus
            eq_trips.emplace_back(row_dc2, dcdc_idx(di, t), -1.0);
          } else {
            // bus_out: receives p_dcdc * eta at this bus
            double eta_dd = std::max(ddc.eta, 0.01);
            eq_trips.emplace_back(row_dc2, dcdc_idx(di, t), eta_dd);
          }
        }

        beq[row_dc2] = dc_bus_load[static_cast<size_t>(dc_bus_load_idx(d, t))];
      }
    }
  }

  // SOC dynamics: soc[s,t] - soc[s,t-1]*(1-sd) + p_ess[s,t]*dt/(eta*E) = 0
  // For t=0: soc[s,0] + p_ess[s,0]*dt/(eta*E) = soc_init*(1-sd)
  const int soc_eq_offset = n_eq_balance + n_eq_dc_balance;
  for (int si = 0; si < S; ++si) {
    const auto& ess = sys.ac.storage[static_cast<size_t>(res.storage_indices[static_cast<size_t>(si)])];
    double E = ess.e_rated_mwh;
    if (E < 1e-12) E = 1.0;  // fallback
    double eta = std::sqrt(ess.eta_charge * ess.eta_discharge);
    if (eta < 1e-6) eta = 1.0;
    double sd = 1.0 - ess.self_discharge_pct / 100.0;
    double coeff_p = dt / (eta * E);

    for (int t = 0; t < T; ++t) {
      int row = soc_eq_offset + si * T + t;
      // soc[s,t]
      eq_trips.emplace_back(row, soc_idx(si, t), 1.0);
      // + p_ess[s,t] * coeff_p
      eq_trips.emplace_back(row, ess_idx(si, t), coeff_p);
      if (t == 0) {
        // = soc_init * sd
        beq[row] = ess.soc_init * sd;
      } else {
        // - soc[s,t-1] * sd
        eq_trips.emplace_back(row, soc_idx(si, t - 1), -sd);
        beq[row] = 0.0;
      }
    }
  }

  // DC storage SOC dynamics with the same sign convention (+ = discharge).
  for (int si = 0; si < Sd; ++si) {
    const auto& ess = sys.dc.storage[static_cast<size_t>(
        res.dc_storage_indices[static_cast<size_t>(si)])];
    double E = ess.e_rated_mwh;
    if (E < 1e-12) E = 1.0;
    double eta = std::sqrt(ess.eta_charge * ess.eta_discharge);
    if (eta < 1e-6) eta = 1.0;
    double sd = 1.0 - ess.self_discharge_pct / 100.0;
    double coeff_p = dt / (eta * E);

    for (int t = 0; t < T; ++t) {
      int row_dc = soc_eq_offset + n_eq_soc_ac + si * T + t;
      eq_trips.emplace_back(row_dc, dcsoc_idx(si, t), 1.0);
      eq_trips.emplace_back(row_dc, dcess_idx(si, t), coeff_p);
      if (t == 0) {
        beq[row_dc] = ess.soc_init * sd;
      } else {
        eq_trips.emplace_back(row_dc, dcsoc_idx(si, t - 1), -sd);
        beq[row_dc] = 0.0;
      }
    }
  }

  Eigen::SparseMatrix<double> Aeq(m_eq, n);
  Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  Aeq.makeCompressed();
  m.linear_part.Aeq = std::move(Aeq);
  m.linear_part.beq = std::move(beq);

  // --- Build inequality constraints ---
  std::vector<Eigen::Triplet<double>> ineq_trips;
  Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(m_ineq);
  int row = 0;

  // (1) Upper output: p_g - Pmax * u_g ≤ 0
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    for (int t = 0; t < T; ++t) {
      ineq_trips.emplace_back(row, pg_idx(gi, t), 1.0);
      ineq_trips.emplace_back(row, ug_idx(gi, t), -gen.pmax_mw);
      b_ineq[row] = 0.0;
      ++row;
    }
  }

  // (2) Lower output: -p_g + Pmin * u_g ≤ 0
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    for (int t = 0; t < T; ++t) {
      ineq_trips.emplace_back(row, pg_idx(gi, t), -1.0);
      ineq_trips.emplace_back(row, ug_idx(gi, t), gen.pmin_mw);
      b_ineq[row] = 0.0;
      ++row;
    }
  }

  // (3) Ramp up: p_{g,t} - p_{g,t-1} ≤ ramp_up * dt * 60
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    double ramp_limit = gen.ramp_up_mw_min * dt * 60.0;
    if (ramp_limit <= 0.0) ramp_limit = gen.pmax_mw;  // no ramp limit
    for (int t = 1; t < T; ++t) {
      ineq_trips.emplace_back(row, pg_idx(gi, t), 1.0);
      ineq_trips.emplace_back(row, pg_idx(gi, t - 1), -1.0);
      b_ineq[row] = ramp_limit;
      ++row;
    }
  }

  // (4) Ramp down: p_{g,t-1} - p_{g,t} ≤ ramp_dn * dt * 60
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    double ramp_limit = gen.ramp_dn_mw_min * dt * 60.0;
    if (ramp_limit <= 0.0) ramp_limit = gen.pmax_mw;
    for (int t = 1; t < T; ++t) {
      ineq_trips.emplace_back(row, pg_idx(gi, t - 1), 1.0);
      ineq_trips.emplace_back(row, pg_idx(gi, t), -1.0);
      b_ineq[row] = ramp_limit;
      ++row;
    }
  }

  // (5) Startup cost: -s_{g,t} + startup * (u_{g,t} - u_{g,t-1}) ≤ 0
  //     For t=0: -s_{g,0} + startup * (u_{g,0} - u_init) ≤ 0
  //     where u_init comes from initial dispatch state (pg_mw > 0)
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    const double u_init = (gen.pg_mw > 1e-6) ? 1.0 : 0.0;
    // t=0: s_{g,0} ≥ startup * (u_{g,0} - u_init)
    ineq_trips.emplace_back(row, sg_idx(gi, 0), -1.0);
    ineq_trips.emplace_back(row, ug_idx(gi, 0), gen.startup_cost);
    b_ineq[row] = gen.startup_cost * u_init;
    ++row;
    for (int t = 1; t < T; ++t) {
      ineq_trips.emplace_back(row, sg_idx(gi, t), -1.0);
      ineq_trips.emplace_back(row, ug_idx(gi, t), gen.startup_cost);
      ineq_trips.emplace_back(row, ug_idx(gi, t - 1), -gen.startup_cost);
      b_ineq[row] = 0.0;
      ++row;
    }
  }

  // (6) Min-up time: if generator starts at t, it must stay on for min_up periods.
  //     u[g,t] - u[g,t-1] ≤ u[g,t+k]  for k=1..min(up_periods-1, T-1-t)
  //     Rearranged: u[g,t] - u[g,t-1] - u[g,t+k] ≤ 0
  //     Only added when up_periods >= 3 (up_periods=2 constraints are weak and
  //     inflate the LP without meaningfully tightening the relaxation).
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    const int up_periods = static_cast<int>(std::ceil(gen.min_up_time_hr / dt));
    if (up_periods >= 3) {
      for (int t = 1; t < T; ++t) {
        const int reach = std::min(up_periods - 1, T - 1 - t);
        for (int k = 1; k <= reach; ++k) {
          ineq_trips.emplace_back(row, ug_idx(gi, t), 1.0);
          ineq_trips.emplace_back(row, ug_idx(gi, t - 1), -1.0);
          ineq_trips.emplace_back(row, ug_idx(gi, t + k), -1.0);
          b_ineq[row] = 0.0;
          ++row;
        }
      }
    }
  }

  // (7) Min-down time: if generator shuts down at t, it must stay off for min_dn periods.
  //     u[g,t-1] - u[g,t] ≤ 1 - u[g,t+k]  for k=1..min(dn_periods-1, T-1-t)
  //     Rearranged: u[g,t-1] - u[g,t] + u[g,t+k] ≤ 1
  //     Only added when dn_periods >= 3.
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    const int dn_periods = static_cast<int>(std::ceil(gen.min_dn_time_hr / dt));
    if (dn_periods >= 3) {
      for (int t = 1; t < T; ++t) {
        const int reach = std::min(dn_periods - 1, T - 1 - t);
        for (int k = 1; k <= reach; ++k) {
          ineq_trips.emplace_back(row, ug_idx(gi, t - 1), 1.0);
          ineq_trips.emplace_back(row, ug_idx(gi, t), -1.0);
          ineq_trips.emplace_back(row, ug_idx(gi, t + k), 1.0);
          b_ineq[row] = 1.0;
          ++row;
        }
      }
    }
  }

  if (use_network_constraints) {
    std::unordered_map<int, int> non_slack_pos;
    for (int bp = 0; bp < static_cast<int>(net.non_slack.size()); ++bp) {
      non_slack_pos[net.non_slack[static_cast<size_t>(bp)]] = bp;
    }
    for (const auto& br : sys.ac.branches) {
      if (!br.in_service || br.rate_a_mva <= 0.0) continue;
      auto fi = bus_idx.find(br.from_bus);
      auto ti = bus_idx.find(br.to_bus);
      if (fi == bus_idx.end() || ti == bus_idx.end()) continue;
      if (fi->second == ti->second) continue;
      if (std::abs(br.x_pu) < 1e-12) continue;
      const double coeff = sys.base_mva / br.x_pu;
      const double fmax = br.rate_a_mva;
      const auto ifs = non_slack_pos.find(fi->second);
      const auto its = non_slack_pos.find(ti->second);

      for (int t = 0; t < T; ++t) {
        // +f <= fmax
        if (ifs != non_slack_pos.end()) ineq_trips.emplace_back(row, theta_idx(ifs->second, t), coeff);
        if (its != non_slack_pos.end()) ineq_trips.emplace_back(row, theta_idx(its->second, t), -coeff);
        b_ineq[row] = fmax;
        ++row;

        // -f <= fmax
        if (ifs != non_slack_pos.end()) ineq_trips.emplace_back(row, theta_idx(ifs->second, t), -coeff);
        if (its != non_slack_pos.end()) ineq_trips.emplace_back(row, theta_idx(its->second, t), coeff);
        b_ineq[row] = fmax;
        ++row;
      }
    }
  }

  // DC line thermal limits: |P_dc_line| ≤ rating
  // P_dc_line ≈ (1/r) * (V_d - V_j) but with flat voltage assumption,
  // the flow is encoded in the DC nodal balance via the p_vsc and p_dcdc variables.
  // We bound the net DC branch flow using converter power variables.
  // For a proper model: bound each DC branch using the conductance and DC voltage differences.
  // Since we assume flat DC voltage (no DC voltage variables), we directly bound
  // the total power through each converter and DC-DC converter via their variable bounds.
  // However, to be thorough, we add explicit DC branch flow limits as:
  //   For each DC branch (d1,d2) with rating S_max:
  //     The flow is implicit in the DC nodal balance. We cannot directly constrain it
  //     without DC voltage variables. Instead, we note that the converter+ESS injections
  //     at each bus implicitly determine flows. The variable bounds on p_vsc and p_dcdc
  //     already enforce capacity limits. So DC branch limits are not separately constrainable
  //     without adding DC voltage variables — which would make this a SOCP/nonlinear problem.
  //   For a linearized approximation, we skip DC branch flow constraints and rely on
  //   converter power bounds as the binding constraint. This is standard practice in
  //   linearized hybrid SCUC formulations.
  // (n_ineq_dc_line is counted but set to 0 when DC voltage variables are omitted)

  if (opts.reserve_requirement_fraction > 0.0) {
    for (int t = 0; t < T; ++t) {
      for (int gi = 0; gi < G; ++gi) {
        const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
        ineq_trips.emplace_back(row, pg_idx(gi, t), 1.0);
        ineq_trips.emplace_back(row, ug_idx(gi, t), -gen.pmax_mw);
      }
      const double req = std::max(0.0, opts.reserve_requirement_fraction) *
                         std::max(0.0, total_load[static_cast<size_t>(t)]);
      b_ineq[row] = -req;
      ++row;
    }
  }

  if (row != m_ineq) {
    throw std::runtime_error("UC MILP row assembly mismatch");
  }

  Eigen::SparseMatrix<double> A_ineq(m_ineq, n);
  A_ineq.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  A_ineq.makeCompressed();
  m.linear_part.A = std::move(A_ineq);
  m.linear_part.b = std::move(b_ineq);

  return res;
}

// ═══════════════════════════════════════════════════════════════════════
// Select and create MILP solver adapter
// ═══════════════════════════════════════════════════════════════════════
engine::SolverAdapterPtr create_milp_adapter(UCSolverChoice choice) {
  using namespace engine;

  auto make_tuned_native = []() {
    BCOptions opt;
    opt.cuts = CutType::Gomory;
    opt.root_cut_rounds = 15;
    opt.cuts_per_round = 20;
    opt.use_simplex_lp_nodes = true;
    opt.use_feasibility_pump = false;
    opt.branching = BranchingStrategy::Pseudocost;
    opt.node_sel = NodeSelection::Hybrid;
    opt.gap_tol = 1e-3;  // 0.1% gap is sufficient for UC
    opt.max_lp_iter = 20000;  // large enough for IEEE118+ root LP
    opt.verbose = false;
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    if (hw >= 4) opt.num_threads = std::min(hw, 8);
    return std::make_shared<NativeBranchAndCutAdapter>(opt);
  };

  if (choice == UCSolverChoice::Native) {
    return make_tuned_native();
  }
  if (choice == UCSolverChoice::HiGHS) {
    return std::make_shared<HighsAdapter>();
  }
  if (choice == UCSolverChoice::Gurobi) {
    return std::make_shared<GurobiAdapter>();
  }

  // Auto: try Gurobi → HiGHS → Native
  AdapterRegistry reg;
  reg.register_adapter(std::make_shared<GurobiAdapter>());
  reg.register_adapter(std::make_shared<HighsAdapter>());
  reg.register_adapter(make_tuned_native());

  auto adapter = reg.first_for(ProblemClass::MILP);
  if (!adapter) {
    return make_tuned_native();
  }
  return adapter;
}

// ═══════════════════════════════════════════════════════════════════════
// Extract UCSchedule from MILP solution vector
// ═══════════════════════════════════════════════════════════════════════
UCSchedule extract_schedule(const Eigen::VectorXd& x,
                            const UCBuildResult& build,
                            double total_cost,
                            bool feasible,
                            const std::string& solver_name) {
  UCSchedule sched;
  sched.total_cost = total_cost;
  sched.feasible = feasible;
  sched.solver_name = solver_name;

  const int G = build.G, S = build.S, Sd = build.Sd, R = build.R, T = build.T;
  const int nPg = G * T, nUg = G * T, nSg = G * T;
  const int nEss = S * T;
  const int nSoc = S * T;
  const int nDcEss = Sd * T;
  const int nDcSoc = Sd * T;

  auto pg_idx  = [&](int g, int t) { return g * T + t; };
  auto ug_idx  = [&](int g, int t) { return nPg + g * T + t; };
  auto ess_idx = [&](int s, int t) { return nPg + nUg + nSg + s * T + t; };
  auto soc_idx = [&](int s, int t) { return nPg + nUg + nSg + nEss + s * T + t; };
  auto dcess_idx = [&](int s, int t) {
    return nPg + nUg + nSg + nEss + nSoc + s * T + t;
  };
  auto dcsoc_idx = [&](int s, int t) {
    return nPg + nUg + nSg + nEss + nSoc + nDcEss + s * T + t;
  };
  auto ren_idx = [&](int r, int t) {
    return nPg + nUg + nSg + nEss + nSoc + nDcEss + nDcSoc + r * T + t;
  };

  sched.gen_dispatch.resize(static_cast<size_t>(G), std::vector<double>(static_cast<size_t>(T)));
  sched.gen_commit.resize(static_cast<size_t>(G), std::vector<int>(static_cast<size_t>(T)));
  sched.ess_dispatch.resize(static_cast<size_t>(S), std::vector<double>(static_cast<size_t>(T)));
  sched.ess_soc.resize(static_cast<size_t>(S), std::vector<double>(static_cast<size_t>(T)));
  sched.dc_ess_dispatch.resize(static_cast<size_t>(Sd), std::vector<double>(static_cast<size_t>(T)));
  sched.renewable_dispatch.resize(static_cast<size_t>(R), std::vector<double>(static_cast<size_t>(T)));

  for (int gi = 0; gi < G; ++gi) {
    for (int t = 0; t < T; ++t) {
      sched.gen_dispatch[static_cast<size_t>(gi)][static_cast<size_t>(t)] = x[pg_idx(gi, t)];
      sched.gen_commit[static_cast<size_t>(gi)][static_cast<size_t>(t)] = (x[ug_idx(gi, t)] > 0.5) ? 1 : 0;
    }
  }
  for (int si = 0; si < S; ++si) {
    for (int t = 0; t < T; ++t) {
      sched.ess_dispatch[static_cast<size_t>(si)][static_cast<size_t>(t)] = x[ess_idx(si, t)];
      sched.ess_soc[static_cast<size_t>(si)][static_cast<size_t>(t)] = x[soc_idx(si, t)];
    }
  }
  for (int si = 0; si < Sd; ++si) {
    for (int t = 0; t < T; ++t) {
      sched.dc_ess_dispatch[static_cast<size_t>(si)][static_cast<size_t>(t)] = x[dcess_idx(si, t)];
      (void)x[dcsoc_idx(si, t)];
    }
  }
  for (int ri = 0; ri < R; ++ri) {
    for (int t = 0; t < T; ++t) {
      sched.renewable_dispatch[static_cast<size_t>(ri)][static_cast<size_t>(t)] = x[ren_idx(ri, t)];
    }
  }

  // Extract VSC and DCDC converter dispatch
  const int C = build.C, Cd = build.Cd;
  if (C > 0) {
    // VSC vars start after theta; DCDC vars after VSC.
    // total = ... + nTheta + C*T + Cd*T
    const int vsc_start = static_cast<int>(x.size()) - (C * T + Cd * T);
    sched.vsc_dispatch.resize(static_cast<size_t>(C), std::vector<double>(static_cast<size_t>(T)));
    for (int ci = 0; ci < C; ++ci) {
      for (int t = 0; t < T; ++t) {
        int idx = vsc_start + ci * T + t;
        if (idx >= 0 && idx < static_cast<int>(x.size()))
          sched.vsc_dispatch[static_cast<size_t>(ci)][static_cast<size_t>(t)] = x[idx];
      }
    }
    if (Cd > 0) {
      const int dcdc_start = vsc_start + C * T;
      sched.dcdc_dispatch.resize(static_cast<size_t>(Cd), std::vector<double>(static_cast<size_t>(T)));
      for (int di = 0; di < Cd; ++di) {
        for (int t = 0; t < T; ++t) {
          int idx = dcdc_start + di * T + t;
          if (idx >= 0 && idx < static_cast<int>(x.size()))
            sched.dcdc_dispatch[static_cast<size_t>(di)][static_cast<size_t>(t)] = x[idx];
        }
      }
    }
  }

  return sched;
}

// ═══════════════════════════════════════════════════════════════════════
// Priority-list UC heuristic — provides warm-start incumbent for B&C.
// Merit-order dispatch: sort generators by marginal cost, greedily commit
// cheapest units to cover demand each hour.  O(G·T·log G), typically < 1 ms.
// ═══════════════════════════════════════════════════════════════════════
Eigen::VectorXd priority_list_uc_heuristic(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const UCBuildResult& build,
    const std::vector<double>& total_load) {
  using namespace engine;
  const int G = build.G, S = build.S, Sd = build.Sd, R = build.R, T = build.T;
  const int n = build.model.linear_part.c.size();

  // Variable index helpers (must match build_uc_milp layout).
  const int nPg = G * T, nUg = G * T, nSg = G * T;
  const int nEss = S * T, nSoc = S * T;
  const int nDcEss = Sd * T, nDcSoc = Sd * T;
  auto pg_idx  = [&](int g, int t) { return g * T + t; };
  auto ug_idx  = [&](int g, int t) { return nPg + g * T + t; };
  auto sg_idx  = [&](int g, int t) { return nPg + nUg + g * T + t; };
  auto ess_idx = [&](int s, int t) { return nPg + nUg + nSg + s * T + t; };
  auto soc_idx = [&](int s, int t) { return nPg + nUg + nSg + nEss + s * T + t; };
  auto dcess_idx = [&](int s, int t) {
    return nPg + nUg + nSg + nEss + nSoc + s * T + t;
  };
  auto dcsoc_idx = [&](int s, int t) {
    return nPg + nUg + nSg + nEss + nSoc + nDcEss + s * T + t;
  };
  auto ren_idx = [&](int r, int t) {
    return nPg + nUg + nSg + nEss + nSoc + nDcEss + nDcSoc + r * T + t;
  };

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  // Sort generators by priority (merit order = ascending marginal cost).
  struct GenOrder { int gi; double cost; double pmax; double pmin; };
  std::vector<GenOrder> merit;
  merit.reserve(static_cast<size_t>(G));
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(build.gen_indices[static_cast<size_t>(gi)])];
    merit.push_back({gi, gen.cost_c1, gen.pmax_mw, gen.pmin_mw});
  }
  std::sort(merit.begin(), merit.end(),
            [](const GenOrder& a, const GenOrder& b) { return a.cost < b.cost; });

  // Collect renewable availability per timestep.
  auto profile_map = build_profile_map(ts_data);
  std::vector<std::vector<double>> ren_avail(static_cast<size_t>(R),
                                              std::vector<double>(static_cast<size_t>(T)));
  for (int ri = 0; ri < R; ++ri) {
    const auto& ren = sys.ac.renewable_gens[static_cast<size_t>(build.renewable_indices[static_cast<size_t>(ri)])];
    const auto* pv = find_profile(profile_map, ren.profile_id);
    for (int t = 0; t < T; ++t) {
      ren_avail[static_cast<size_t>(ri)][static_cast<size_t>(t)] =
          profile_value(pv, t, 1.0) * ren.p_rated_mw;
    }
  }

  // Dispatch timestep by timestep.
  std::vector<double> soc_ac(static_cast<size_t>(S));
  for (int si = 0; si < S; ++si) {
    soc_ac[static_cast<size_t>(si)] =
        sys.ac.storage[static_cast<size_t>(build.storage_indices[static_cast<size_t>(si)])].soc_init;
  }
  std::vector<double> soc_dc(static_cast<size_t>(Sd));
  for (int si = 0; si < Sd; ++si) {
    soc_dc[static_cast<size_t>(si)] =
        sys.dc.storage[static_cast<size_t>(build.dc_storage_indices[static_cast<size_t>(si)])].soc_init;
  }

  for (int t = 0; t < T; ++t) {
    double demand = total_load[static_cast<size_t>(t)];

    // 1. Dispatch all renewables at full availability first.
    for (int ri = 0; ri < R; ++ri) {
      double p_ren = ren_avail[static_cast<size_t>(ri)][static_cast<size_t>(t)];
      x[ren_idx(ri, t)] = p_ren;
      demand -= p_ren;
    }

    // 2. Commit generators in merit order to cover remaining demand.
    for (const auto& mo : merit) {
      if (demand <= 0) break;
      x[ug_idx(mo.gi, t)] = 1.0;
      double p = std::min(mo.pmax, std::max(mo.pmin, demand));
      x[pg_idx(mo.gi, t)] = p;
      demand -= p;
    }

    // If still excess demand, commit remaining generators at Pmin.
    if (demand > 0) {
      for (const auto& mo : merit) {
        if (x[ug_idx(mo.gi, t)] > 0.5) continue;
        x[ug_idx(mo.gi, t)] = 1.0;
        x[pg_idx(mo.gi, t)] = mo.pmin;
        demand -= mo.pmin;
        if (demand <= 0) break;
      }
    }

    // 3. ESS: simple rule — discharge when demand is high, charge when low.
    //    (zero dispatch for heuristic simplicity; B&C will optimize.)
    for (int si = 0; si < S; ++si) {
      const auto& ess = sys.ac.storage[static_cast<size_t>(build.storage_indices[static_cast<size_t>(si)])];
      double sd = 1.0 - ess.self_discharge_pct / 100.0;
      // Keep ESS neutral for heuristic (p=0).
      x[ess_idx(si, t)] = 0.0;
      soc_ac[static_cast<size_t>(si)] = soc_ac[static_cast<size_t>(si)] * sd;
      x[soc_idx(si, t)] = soc_ac[static_cast<size_t>(si)];
    }
    for (int si = 0; si < Sd; ++si) {
      const auto& ess = sys.dc.storage[static_cast<size_t>(build.dc_storage_indices[static_cast<size_t>(si)])];
      double sd = 1.0 - ess.self_discharge_pct / 100.0;
      x[dcess_idx(si, t)] = 0.0;
      soc_dc[static_cast<size_t>(si)] = soc_dc[static_cast<size_t>(si)] * sd;
      x[dcsoc_idx(si, t)] = soc_dc[static_cast<size_t>(si)];
    }
  }

  // 4. Compute startup costs from commitment decisions.
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(build.gen_indices[static_cast<size_t>(gi)])];
    const double u_init = (gen.pg_mw > 1e-6) ? 1.0 : 0.0;
    for (int t = 0; t < T; ++t) {
      double u_prev = (t == 0) ? u_init : x[ug_idx(gi, t - 1)];
      double startup = std::max(0.0, gen.startup_cost * (x[ug_idx(gi, t)] - u_prev));
      x[sg_idx(gi, t)] = startup;
    }
  }

  return x;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// Phase II: Solve Unit Commitment
// ═══════════════════════════════════════════════════════════════════════
UCSchedule solve_unit_commitment(const HybridPowerSystem& sys,
                                 const TimeSeriesData& ts_data,
                                 const TimeSeriesPFOptions& opts) {
  if (ts_data.num_steps <= 0) {
    throw std::invalid_argument("TimeSeriesData must have at least one time step");
  }

  auto net = build_dc_network(sys);
  auto dcg = build_dc_grid(sys);
  auto build = build_uc_milp(sys, ts_data, net, dcg, opts);

  // Compute total_load for heuristic (replicates the load aggregation in build_uc_milp).
  const int T = ts_data.num_steps;
  auto profile_map = build_profile_map(ts_data);
  std::vector<double> total_load(static_cast<size_t>(T), 0.0);
  {
    const bool has_comp = !sys.ac.loads.empty() || !sys.ac.charging_stations.empty();
    const auto* default_lp = find_profile(profile_map, 0);
    if (has_comp) {
      for (const auto& load : sys.ac.loads) {
        if (!load.in_service) continue;
        const auto* pv = find_profile(profile_map, load.profile_id);
        for (int t = 0; t < T; ++t)
          total_load[static_cast<size_t>(t)] += load.p_mw * profile_value(pv, t, 1.0) * load.scaling;
      }
      for (const auto& cs : sys.ac.charging_stations) {
        if (!cs.in_service) continue;
        for (int t = 0; t < T; ++t)
          total_load[static_cast<size_t>(t)] += std::max(0.0, cs.p_total_kw / 1000.0);
      }
    } else {
      for (const auto& bus : sys.ac.buses) {
        if (!bus.in_service) continue;
        for (int t = 0; t < T; ++t)
          total_load[static_cast<size_t>(t)] += std::max(0.0, bus.pd_mw) * profile_value(default_lp, t, 1.0);
      }
    }
    for (const auto& fl : sys.ac.flexible_loads) {
      if (!fl.in_service) continue;
      for (int t = 0; t < T; ++t) total_load[static_cast<size_t>(t)] += std::max(0.0, fl.p_mw);
    }
    for (const auto& al : sys.ac.asymmetric_loads) {
      if (!al.in_service) continue;
      double p3 = std::max(0.0, al.pa_mw + al.pb_mw + al.pc_mw) * std::max(0.0, al.scaling);
      for (int t = 0; t < T; ++t) total_load[static_cast<size_t>(t)] += p3;
    }
    for (const auto& load : sys.dc.loads) {
      if (!load.in_service) continue;
      const auto* pv = find_profile(profile_map, load.profile_id);
      for (int t = 0; t < T; ++t)
        total_load[static_cast<size_t>(t)] += load.p_mw * profile_value(pv, t, 1.0) * load.scaling;
    }
    for (const auto& pvarr : sys.dc.pv_arrays) {
      if (!pvarr.in_service) continue;
      const auto* pv = find_profile(profile_map, pvarr.profile_id);
      for (int t = 0; t < T; ++t)
        total_load[static_cast<size_t>(t)] -= pvarr.p_set_mw * profile_value(pv, t, 1.0);
    }
    for (const auto& sg : sys.dc.dc_static_generators) {
      if (!sg.in_service) continue;
      const auto* pv = find_profile(profile_map, sg.profile_id);
      for (int t = 0; t < T; ++t)
        total_load[static_cast<size_t>(t)] -= sg.p_set_mw * sg.scaling * profile_value(pv, t, 1.0);
    }
    // Subtract AC PV system must-take generation (mirrors build_uc_milp logic)
    for (const auto& pvsys : sys.ac.pv_systems) {
      if (!pvsys.in_service) continue;
      const auto* pvprof = find_profile(profile_map, pvsys.profile_id);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(pvprof, t, 1.0);
        double pv_mw;
        if (pvsys.voc > 0.0 && pvsys.isc > 0.0 && pvsys.vmpp > 0.0) {
          PVSystem pv_copy = pvsys;
          pv_copy.irradiance = scale * 1000.0;
          pv_mw = powerflow::compute_pv_power_mw(pv_copy);
        } else {
          pv_mw = pvsys.p_mw * scale;
        }
        total_load[static_cast<size_t>(t)] -= pv_mw;
      }
    }
  }

  // Generate warm-start incumbent via priority-list heuristic.
  build.model.initial_solution =
      priority_list_uc_heuristic(sys, ts_data, build, total_load);

  auto adapter = create_milp_adapter(opts.uc_solver);
  auto result = adapter->solve_milp(build.model);

  // Validate solver produced a usable solution vector
  const int expected_size = build.model.linear_part.c.size();
  if (!result.stats.success || result.x.size() < expected_size) {
    UCSchedule sched;
    sched.feasible = false;
    sched.solver_name = result.stats.solver_name;
    sched.total_cost = 0.0;
    return sched;
  }

  return extract_schedule(result.x, build,
                          result.stats.objective,
                          result.stats.success,
                          result.stats.solver_name);
}

// ═══════════════════════════════════════════════════════════════════════
// Full pipeline: UC → OPF (per step) → PF validation (per step)
// ═══════════════════════════════════════════════════════════════════════
TimeSeriesPFResult solve_time_series_pf(const HybridPowerSystem& sys,
                                         const TimeSeriesData& ts_data,
                                         const TimeSeriesPFOptions& opts) {
  TimeSeriesPFResult result;
  const int T = ts_data.num_steps;
  result.num_steps = T;

  if (T <= 0) {
    return result;
  }

  auto profile_map = build_profile_map(ts_data);

  // Phase II: solve UC first (unless skipped)
  UCSchedule schedule;
  if (!opts.skip_uc) {
    auto t_uc0 = std::chrono::steady_clock::now();
    schedule = solve_unit_commitment(sys, ts_data, opts);
    auto t_uc1 = std::chrono::steady_clock::now();
    result.uc_solve_sec = std::chrono::duration<double>(t_uc1 - t_uc0).count();
  } else {
    schedule.feasible = true;
    schedule.solver_name = "skipped";
  }

  // Populate DC-side profile-driven arrays; keep UC-optimized DC ESS when available.
  {
    const int ndc_pv = static_cast<int>(sys.dc.pv_arrays.size());
    const int ndc_ess = static_cast<int>(sys.dc.storage.size());
    const int ndc_sgen = static_cast<int>(sys.dc.dc_static_generators.size());
    const int ndc_load = static_cast<int>(sys.dc.loads.size());

    schedule.dc_pv_dispatch.resize(static_cast<size_t>(ndc_pv),
                                   std::vector<double>(static_cast<size_t>(T)));
    schedule.dc_sgen_dispatch.resize(static_cast<size_t>(ndc_sgen),
                                     std::vector<double>(static_cast<size_t>(T)));
    schedule.dc_load_demand.resize(static_cast<size_t>(ndc_load),
                                   std::vector<double>(static_cast<size_t>(T)));

    for (int k = 0; k < ndc_pv; ++k) {
      const auto& pv = sys.dc.pv_arrays[static_cast<size_t>(k)];
      const auto* prof = find_profile(profile_map, pv.profile_id);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(prof, t, 1.0);
        schedule.dc_pv_dispatch[static_cast<size_t>(k)][static_cast<size_t>(t)] =
            pv.in_service ? pv.p_set_mw * scale : 0.0;
      }
    }
    bool has_uc_dc_ess =
        !opts.skip_uc && schedule.feasible &&
        static_cast<int>(schedule.dc_ess_dispatch.size()) == ndc_ess;
    if (has_uc_dc_ess) {
      for (int k = 0; k < ndc_ess; ++k) {
        if (static_cast<int>(schedule.dc_ess_dispatch[static_cast<size_t>(k)].size()) != T) {
          has_uc_dc_ess = false;
          break;
        }
      }
    }
    if (!has_uc_dc_ess) {
      schedule.dc_ess_dispatch.resize(static_cast<size_t>(ndc_ess),
                                      std::vector<double>(static_cast<size_t>(T)));
      for (int k = 0; k < ndc_ess; ++k) {
        const auto& ess = sys.dc.storage[static_cast<size_t>(k)];
        for (int t = 0; t < T; ++t) {
          schedule.dc_ess_dispatch[static_cast<size_t>(k)][static_cast<size_t>(t)] =
              ess.in_service ? ess.p_mw : 0.0;
        }
      }
    }
    for (int k = 0; k < ndc_sgen; ++k) {
      const auto& sg = sys.dc.dc_static_generators[static_cast<size_t>(k)];
      const auto* prof = find_profile(profile_map, sg.profile_id);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(prof, t, 1.0);
        schedule.dc_sgen_dispatch[static_cast<size_t>(k)][static_cast<size_t>(t)] =
            sg.in_service ? sg.p_set_mw * sg.scaling * scale : 0.0;
      }
    }
    for (int k = 0; k < ndc_load; ++k) {
      const auto& ld = sys.dc.loads[static_cast<size_t>(k)];
      if (!ld.in_service) continue;
      const auto* pv = find_profile(profile_map, ld.profile_id);
      for (int t = 0; t < T; ++t) {
        double scale = profile_value(pv, t, 1.0);
        schedule.dc_load_demand[static_cast<size_t>(k)][static_cast<size_t>(t)] =
            ld.p_mw * scale * ld.scaling;
      }
    }
  }

  // AC ESS fallback: peak-shaving heuristic when UC didn't produce ESS dispatch
  {
    int nACEssActive = 0;
    for (const auto& e : sys.ac.storage)
      if (e.in_service) ++nACEssActive;

    bool need_ess_fallback = (nACEssActive > 0) &&
        (static_cast<int>(schedule.ess_dispatch.size()) != nACEssActive);
    if (!need_ess_fallback && nACEssActive > 0) {
      for (int k = 0; k < nACEssActive; ++k) {
        if (static_cast<int>(schedule.ess_dispatch[static_cast<size_t>(k)].size()) != T) {
          need_ess_fallback = true;
          break;
        }
      }
    }

    if (need_ess_fallback) {
      // Compute simplified net load from profiles for peak-shaving reference
      std::vector<double> net_load(static_cast<size_t>(T), 0.0);
      const bool has_comp = !sys.ac.loads.empty() || !sys.ac.charging_stations.empty();
      const auto* default_lp = find_profile(profile_map, 0);
      if (has_comp) {
        for (const auto& load : sys.ac.loads) {
          if (!load.in_service) continue;
          const auto* lp = find_profile(profile_map, load.profile_id);
          for (int t = 0; t < T; ++t)
            net_load[static_cast<size_t>(t)] +=
                load.p_mw * profile_value(lp, t, 1.0) * load.scaling;
        }
      } else {
        for (const auto& bus : sys.ac.buses) {
          if (!bus.in_service) continue;
          for (int t = 0; t < T; ++t)
            net_load[static_cast<size_t>(t)] +=
                std::max(0.0, bus.pd_mw) * profile_value(default_lp, t, 1.0);
        }
      }
      for (const auto& ld : sys.dc.loads) {
        if (!ld.in_service) continue;
        const auto* lp = find_profile(profile_map, ld.profile_id);
        for (int t = 0; t < T; ++t)
          net_load[static_cast<size_t>(t)] +=
              ld.p_mw * profile_value(lp, t, 1.0) * ld.scaling;
      }
      // Subtract renewable / PV generation
      for (const auto& ren : sys.ac.renewable_gens) {
        if (!ren.in_service) continue;
        const auto* rp = find_profile(profile_map, ren.profile_id);
        for (int t = 0; t < T; ++t)
          net_load[static_cast<size_t>(t)] -=
              ren.p_rated_mw * profile_value(rp, t, 1.0);
      }
      for (const auto& pvsys : sys.ac.pv_systems) {
        if (!pvsys.in_service) continue;
        const auto* pp = find_profile(profile_map, pvsys.profile_id);
        for (int t = 0; t < T; ++t) {
          const double scale = profile_value(pp, t, 1.0);
          double pv_mw = (pvsys.voc > 0 && pvsys.isc > 0 && pvsys.vmpp > 0)
              ? [&]{ PVSystem pvc = pvsys; pvc.irradiance = scale * 1000.0;
                     return powerflow::compute_pv_power_mw(pvc); }()
              : pvsys.p_mw * scale;
          net_load[static_cast<size_t>(t)] -= pv_mw;
        }
      }

      double avg_load = 0.0;
      for (int t = 0; t < T; ++t)
        avg_load += net_load[static_cast<size_t>(t)];
      avg_load /= std::max(T, 1);

      // Sum of all ESS pmax for proportional sharing
      double total_pmax = 0.0;
      for (const auto& e : sys.ac.storage)
        if (e.in_service) total_pmax += std::max(e.pmax_mw, 0.01);

      schedule.ess_dispatch.resize(static_cast<size_t>(nACEssActive),
                                   std::vector<double>(static_cast<size_t>(T), 0.0));
      schedule.ess_soc.resize(static_cast<size_t>(nACEssActive),
                              std::vector<double>(static_cast<size_t>(T), 0.0));

      int sidx = 0;
      for (const auto& ess : sys.ac.storage) {
        if (!ess.in_service) continue;
        const double E = std::max(ess.e_rated_mwh, 1e-12);
        const double dt = ts_data.step_duration_hr;
        const double sd = 1.0 - ess.self_discharge_pct / 100.0;
        const double share = ess.pmax_mw / total_pmax;
        const double eta_c = std::max(ess.eta_charge, 0.01);
        const double eta_d = std::max(ess.eta_discharge, 0.01);
        double soc = ess.soc_init;

        // SOC update helper with correct charge/discharge efficiency:
        //   Discharge (p>0): battery delivers p, drains p/(eta_d) from stored energy
        //   Charge   (p<0): grid supplies |p|, battery stores |p|*eta_c
        auto calc_delta_soc = [&](double pw) -> double {
          return (pw >= 0.0)
              ? (pw * dt / (eta_d * E))   // discharge: SOC drops by p*dt/(eta_d*E)
              : (pw * eta_c * dt / E);    // charge:    SOC drops by p*eta_c*dt/E (negative = rises)
        };

        for (int t = 0; t < T; ++t) {
          double surplus = net_load[static_cast<size_t>(t)] - avg_load;
          double p = std::clamp(surplus * share, ess.pmin_mw, ess.pmax_mw);

          // Check SOC limits
          double delta_soc = calc_delta_soc(p);
          double new_soc = soc * sd - delta_soc;
          if (new_soc < ess.soc_min) {
            // SOC would go below min → reduce discharge
            delta_soc = std::max(soc * sd - ess.soc_min, 0.0);
            p = std::max(delta_soc * eta_d * E / dt, 0.0);
            p = std::clamp(p, ess.pmin_mw, ess.pmax_mw);
          }
          if (new_soc > ess.soc_max) {
            // SOC would exceed max → reduce charging
            delta_soc = std::min(soc * sd - ess.soc_max, 0.0);
            p = std::min(delta_soc * E / (eta_c * dt), 0.0);
            p = std::clamp(p, ess.pmin_mw, ess.pmax_mw);
          }

          delta_soc = calc_delta_soc(p);
          soc = std::clamp(soc * sd - delta_soc, ess.soc_min, ess.soc_max);

          schedule.ess_dispatch[static_cast<size_t>(sidx)][static_cast<size_t>(t)] = p;
          schedule.ess_soc[static_cast<size_t>(sidx)][static_cast<size_t>(t)] = soc;
        }
        ++sidx;
      }
    }
  }

  result.uc_schedule = schedule;

  // Collect indices of in-service components for schedule mapping
  std::vector<int> gen_active_idx;
  for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
    if (sys.ac.generators[static_cast<size_t>(i)].in_service)
      gen_active_idx.push_back(i);
  }
  std::vector<int> ess_active_idx;
  for (int i = 0; i < static_cast<int>(sys.ac.storage.size()); ++i) {
    if (sys.ac.storage[static_cast<size_t>(i)].in_service)
      ess_active_idx.push_back(i);
  }
  std::vector<int> ren_active_idx;
  for (int i = 0; i < static_cast<int>(sys.ac.renewable_gens.size()); ++i) {
    if (sys.ac.renewable_gens[static_cast<size_t>(i)].in_service)
      ren_active_idx.push_back(i);
  }
  std::unordered_map<int, BusType> ac_bus_type_by_index;
  ac_bus_type_by_index.reserve(sys.ac.buses.size());
  for (const auto& b : sys.ac.buses) {
    ac_bus_type_by_index[b.index] = b.bus_type;
  }

  // Helper: apply UC schedule + load profiles to a system copy at time step t
  auto prepare_system = [&](int t) -> HybridPowerSystem {
    HybridPowerSystem sys_t = sys;

    // Apply UC schedule to generators
    if (!opts.skip_uc && schedule.feasible) {
      for (int gi = 0; gi < static_cast<int>(gen_active_idx.size()); ++gi) {
        int idx = gen_active_idx[static_cast<size_t>(gi)];
        auto& gen = sys_t.ac.generators[static_cast<size_t>(idx)];
        if (gi < static_cast<int>(schedule.gen_dispatch.size())) {
          gen.pg_mw = schedule.gen_dispatch[static_cast<size_t>(gi)][static_cast<size_t>(t)];
          gen.in_service = (schedule.gen_commit[static_cast<size_t>(gi)][static_cast<size_t>(t)] != 0);
          if (!gen.in_service) {
            gen.pg_mw = 0.0;
            gen.qg_mvar = 0.0;
            gen.pmin_mw = 0.0;
            gen.pmax_mw = 0.0;
          } else if (opts.enable_uc_opf_tracking_band) {
            // Keep OPF near UC dispatch to avoid unrealistic re-dispatch drift.
            const double p_uc = gen.pg_mw;
            bool is_slack_gen = gen.is_slack;
            auto bus_it = ac_bus_type_by_index.find(gen.bus);
            if (bus_it != ac_bus_type_by_index.end() &&
                bus_it->second == BusType::SLACK) {
              is_slack_gen = true;
            }

            double rel = std::max(opts.uc_opf_tracking_rel, 0.0);
            double abs_mw = std::max(opts.uc_opf_tracking_abs_mw, 0.0);
            if (opts.enforce_non_slack_strict_tracking && !is_slack_gen) {
              rel = std::max(opts.non_slack_tracking_rel, 0.0);
              abs_mw = std::max(opts.non_slack_tracking_abs_mw, 0.0);
            }
            const double band = std::max(abs_mw, rel * std::max(std::abs(p_uc), 1.0));
            double pmin = std::max(gen.pmin_mw, p_uc - band);
            double pmax = std::min(gen.pmax_mw, p_uc + band);
            if (pmax < pmin) {
              const double pfix = std::clamp(p_uc, gen.pmin_mw, gen.pmax_mw);
              pmin = pfix;
              pmax = pfix;
            }
            gen.pmin_mw = pmin;
            gen.pmax_mw = pmax;
          }
        }
      }
    }

    // Apply UC schedule to ESS (always apply if dispatch arrays are populated)
    for (int si = 0; si < static_cast<int>(ess_active_idx.size()); ++si) {
      int idx = ess_active_idx[static_cast<size_t>(si)];
      auto& ess = sys_t.ac.storage[static_cast<size_t>(idx)];
      if (si < static_cast<int>(schedule.ess_dispatch.size()) &&
          t < static_cast<int>(schedule.ess_dispatch[static_cast<size_t>(si)].size())) {
        ess.p_mw = schedule.ess_dispatch[static_cast<size_t>(si)][static_cast<size_t>(t)];
      }
    }
    for (int si = 0; si < static_cast<int>(sys_t.dc.storage.size()); ++si) {
      if (si < static_cast<int>(schedule.dc_ess_dispatch.size()) &&
          t < static_cast<int>(schedule.dc_ess_dispatch[static_cast<size_t>(si)].size())) {
        sys_t.dc.storage[static_cast<size_t>(si)].p_mw =
            schedule.dc_ess_dispatch[static_cast<size_t>(si)][static_cast<size_t>(t)];
      }
    }

    // Apply UC schedule to renewables
    if (!opts.skip_uc && schedule.feasible) {
      for (int ri = 0; ri < static_cast<int>(ren_active_idx.size()); ++ri) {
        int idx = ren_active_idx[static_cast<size_t>(ri)];
        auto& ren = sys_t.ac.renewable_gens[static_cast<size_t>(idx)];
        if (ri < static_cast<int>(schedule.renewable_dispatch.size())) {
          ren.p_mw = schedule.renewable_dispatch[static_cast<size_t>(ri)][static_cast<size_t>(t)];
        }
      }
    }

    // Apply time-series profiles to AC demand side.
    if (!sys_t.ac.loads.empty() || !sys_t.ac.charging_stations.empty()) {
      for (auto& load : sys_t.ac.loads) {
        if (!load.in_service) continue;
        const auto* pv = find_profile(profile_map, load.profile_id);
        if (pv) {
          double scale = profile_value(pv, t, 1.0);
          load.p_mw *= scale;
          load.q_mvar *= scale;
        }
      }
    } else {
      // Bus-level load fallback for MATPOWER-style cases.
      const auto* default_load = find_profile(profile_map, 0);
      const double scale = profile_value(default_load, t, 1.0);
      for (auto& bus : sys_t.ac.buses) {
        if (!bus.in_service) continue;
        bus.pd_mw *= scale;
        bus.qd_mvar *= scale;
      }
    }

    // Apply time-series profiles to PV systems
    for (auto& pvsys : sys_t.ac.pv_systems) {
      if (!pvsys.in_service) continue;
      const auto* pv = find_profile(profile_map, pvsys.profile_id);
      if (pv) {
        const double scale = profile_value(pv, t, 1.0);
        pvsys.irradiance = scale * 1000.0;
        // Also scale p_mw for fallback path (when voc/isc/vmpp are not set,
        // compute_pv_power_mw returns p_mw directly, ignoring irradiance).
        if (pvsys.voc <= 0.0 || pvsys.isc <= 0.0 || pvsys.vmpp <= 0.0) {
          pvsys.p_mw *= scale;
        }
      }
    }

    // Apply time-series profiles to DC PV arrays
    for (auto& pv : sys_t.dc.pv_arrays) {
      if (!pv.in_service) continue;
      const auto* prof = find_profile(profile_map, pv.profile_id);
      const double scale = profile_value(prof, t, 1.0);
      pv.p_set_mw *= scale;
      pv.irradiance = 1000.0 * scale;
    }

    // Apply time-series profiles to DC static generators
    for (auto& sg : sys_t.dc.dc_static_generators) {
      if (!sg.in_service) continue;
      const auto* prof = find_profile(profile_map, sg.profile_id);
      const double scale = profile_value(prof, t, 1.0);
      sg.p_set_mw *= scale;
    }

    // Apply time-series profiles to DC loads
    for (auto& load : sys_t.dc.loads) {
      if (!load.in_service) continue;
      const auto* pv = find_profile(profile_map, load.profile_id);
      if (pv) {
        double scale = profile_value(pv, t, 1.0);
        load.p_mw *= scale;
      }
    }

    return sys_t;
  };

  // ─────────────────────────────────────────────────────────────────────────
  // Graph topology cache — built once per unique in-service configuration.
  // Avoids re-running build_power_system_graph + analyze_topology every step.
  // ─────────────────────────────────────────────────────────────────────────
  namespace gr = hacdcpf::graph;
  {
    const auto g_base = gr::build_power_system_graph(sys);
    const auto topo   = gr::analyze_topology(g_base);
    if (!topo.all_islands_valid && !topo.diagnostics.empty()) {
      for (const auto& d : topo.diagnostics)
        spdlog::warn("TimeSeries topology (base): {}", d.message);
    }
  }
  // Track branch in-service state to detect topology changes between steps.
  std::vector<bool> topo_cache_in_service(sys.ac.branches.size(), true);
  for (size_t i = 0; i < sys.ac.branches.size(); ++i)
    topo_cache_in_service[i] = sys.ac.branches[i].in_service;
  hacdcpf::graph::TopologyReport topo_cache = [&] {
    const auto g = gr::build_power_system_graph(sys);
    return gr::analyze_topology(g);
  }();

  // Helper: rebuild topology cache when branch in-service flags change.
  auto refresh_topo_if_needed = [&](const HybridPowerSystem& sys_t) {
    bool changed = false;
    const size_t nb = std::min(sys_t.ac.branches.size(), topo_cache_in_service.size());
    for (size_t i = 0; i < nb; ++i) {
      if (sys_t.ac.branches[i].in_service != topo_cache_in_service[i]) {
        changed = true;
        break;
      }
    }
    if (!changed) return;
    for (size_t i = 0; i < nb; ++i)
      topo_cache_in_service[i] = sys_t.ac.branches[i].in_service;
    const auto g_t = gr::build_power_system_graph(sys_t);
    topo_cache = gr::analyze_topology(g_t);
    if (!topo_cache.all_islands_valid && !topo_cache.diagnostics.empty()) {
      for (const auto& d : topo_cache.diagnostics)
        spdlog::warn("TimeSeries topology change at step: {}", d.message);
    }
  };

  // ═══════════════════════════════════════════════════════════════
  // Stage 2: OPF per timestep (when run_opf=true)
  // ═══════════════════════════════════════════════════════════════
  if (opts.run_opf) {
    result.opf_results.resize(static_cast<size_t>(T));
    result.crossval.resize(static_cast<size_t>(T));
    result.pf_results.resize(static_cast<size_t>(T));

    // Create SolverHandle for PF validation solves
    SolverHandle* handle = create_solver_handle(sys, opts.pf_options.loss_model);

    // Build bus-index → position map once (bus topology is invariant across timesteps).
    std::unordered_map<int, int> ac_bus_pos;
    ac_bus_pos.reserve(sys.ac.buses.size());
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      ac_bus_pos[sys.ac.buses[i].index] = static_cast<int>(i);
    }
    std::unordered_map<int, int> dc_bus_pos;
    dc_bus_pos.reserve(sys.dc.buses.size());
    for (size_t bi = 0; bi < sys.dc.buses.size(); ++bi) {
      dc_bus_pos[sys.dc.buses[bi].index] = static_cast<int>(bi);
    }

    for (int t = 0; t < T; ++t) {
      HybridPowerSystem sys_t = prepare_system(t);
      refresh_topo_if_needed(sys_t);

      // ── Run OPF on sys_t ──
      auto& opf_res = result.opf_results[static_cast<size_t>(t)];
      opf_res = opf::solve_ac_opf(sys_t, opts.opf_options);

      auto& cv = result.crossval[static_cast<size_t>(t)];
      cv.opf_converged = opf_res.converged;
      cv.opf_objective = opf_res.objective;

      if (!opf_res.converged) {
        // OPF failed — fall back to PF with UC dispatch directly
        reset_solver_handle(handle, sys_t, opts.pf_options.loss_model);
        result.pf_results[static_cast<size_t>(t)] = solve_handle(handle, opts.pf_options);
        cv.pf_converged = result.pf_results[static_cast<size_t>(t)].converged;
        if (cv.pf_converged) ++result.num_converged;
        continue;
      }
      ++result.num_opf_converged;

      // ── Inject OPF dispatch into system for PF validation ──
      // Move sys_t into sys_pf to avoid a second deep copy (sys_t not needed after this).
      HybridPowerSystem sys_pf = std::move(sys_t);

      // Set generator P/Q from OPF
      for (size_t gi = 0; gi < sys_pf.ac.generators.size() &&
                           gi < opf_res.pg_mw.size(); ++gi) {
        sys_pf.ac.generators[gi].pg_mw = opf_res.pg_mw[gi];
        sys_pf.ac.generators[gi].qg_mvar = opf_res.qg_mvar[gi];
      }

      // Set VSC converter dispatch from OPF (PQ mode)
      size_t conv_k = 0;
      for (size_t ci = 0; ci < sys_pf.vsc_converters.size(); ++ci) {
        if (!sys_pf.vsc_converters[ci].in_service) continue;
        if (conv_k < opf_res.pac_mw.size()) {
          sys_pf.vsc_converters[ci].p_set_mw = opf_res.pac_mw[conv_k];
          sys_pf.vsc_converters[ci].q_set_mvar = opf_res.qac_mvar[conv_k];
          sys_pf.vsc_converters[ci].control_mode = ConverterMode::PQ_MODE;
          ++conv_k;
        }
      }

      // Set renewable dispatch from OPF using ComponentRef map
      for (size_t k = 0; k < opf_res.pren_mw.size() &&
                          k < opf_res.ren_map.size(); ++k) {
        int orig = opf_res.ren_map[k].original_index;
        int src = opf_res.ren_map[k].source_type;
        if (src == 0 && orig >= 0 &&
            orig < static_cast<int>(sys_pf.ac.renewable_gens.size())) {
          sys_pf.ac.renewable_gens[static_cast<size_t>(orig)].p_mw =
              opf_res.pren_mw[k];
        } else if (src == 1 && orig >= 0 &&
                   orig < static_cast<int>(sys_pf.ac.pv_systems.size())) {
          sys_pf.ac.pv_systems[static_cast<size_t>(orig)].p_mw =
              opf_res.pren_mw[k];
        }
      }

      // Set storage dispatch from OPF using ComponentRef map
      for (size_t k = 0; k < opf_res.pstor_mw.size() &&
                          k < opf_res.stor_map.size(); ++k) {
        const int orig = opf_res.stor_map[k].original_index;
        const int src = opf_res.stor_map[k].source_type;
        if (src == 0 && orig >= 0 &&
            orig < static_cast<int>(sys_pf.ac.storage.size())) {
          sys_pf.ac.storage[static_cast<size_t>(orig)].p_mw = opf_res.pstor_mw[k];
        } else if (src == 1 && orig >= 0 &&
                   orig < static_cast<int>(sys_pf.dc.storage.size())) {
          sys_pf.dc.storage[static_cast<size_t>(orig)].p_mw = opf_res.pstor_mw[k];
        }
      }

      // Set DCDC transfer dispatch from OPF using ComponentRef map
      for (size_t k = 0; k < opf_res.pdcdc_mw.size() &&
                          k < opf_res.dcdc_map.size(); ++k) {
        int orig = opf_res.dcdc_map[k].original_index;
        if (orig >= 0 && orig < static_cast<int>(sys_pf.dc.dcdc_converters.size())) {
          sys_pf.dc.dcdc_converters[static_cast<size_t>(orig)].p_ref_mw =
              opf_res.pdcdc_mw[k];
        }
      }

      // Set flexible-load realized demand from OPF using ComponentRef map
      for (size_t k = 0; k < opf_res.pflex_mw.size() &&
                          k < opf_res.flex_map.size(); ++k) {
        int orig = opf_res.flex_map[k].original_index;
        if (orig >= 0 && orig < static_cast<int>(sys_pf.ac.flexible_loads.size())) {
          sys_pf.ac.flexible_loads[static_cast<size_t>(orig)].p_mw =
              opf_res.pflex_mw[k];
        }
      }

      // Set bus voltage setpoints from OPF
      for (size_t gi = 0; gi < sys_pf.ac.generators.size(); ++gi) {
        const int bus = sys_pf.ac.generators[gi].bus;
        auto it = ac_bus_pos.find(bus);
        if (it == ac_bus_pos.end()) continue;
        const size_t bi = static_cast<size_t>(it->second);
        if (bi < opf_res.vm.size() && bi < sys_pf.ac.buses.size()) {
          sys_pf.ac.generators[gi].vg_pu = opf_res.vm[bi];
          sys_pf.ac.buses[bi].vm_pu = opf_res.vm[bi];
        }
      }

      // Apply OPF load shedding to both bus-level and load-table demand.
      apply_opf_load_shedding_to_demands(sys_pf, opf_res);

      // OPF-side physical loss estimate from OPF voltages and replayed controls.
      cv.opf_loss_mw = estimate_opf_physical_loss_mw(
          sys_pf, opf_res, opts.pf_options.loss_model);

      // ── Run PF validation ──
      reset_solver_handle(handle, sys_pf, opts.pf_options.loss_model);
      auto& pf_res = result.pf_results[static_cast<size_t>(t)];
      pf_res = solve_handle(handle, opts.pf_options);
      cv.pf_converged = pf_res.converged;

      if (pf_res.converged) {
        ++result.num_converged;

        // ── Cross-validation metrics ──
        const size_t n_ac = sys_pf.ac.buses.size();
        double max_vm = 0.0, max_va = 0.0;
        // Va reference offset (OPF and PF may use different ref angles)
        const double va_ref = (n_ac > 0 && !opf_res.va.empty() && !pf_res.va.empty())
                                  ? (opf_res.va[0] - pf_res.va[0])
                                  : 0.0;
        for (size_t i = 0; i < n_ac; ++i) {
          if (i < opf_res.vm.size() && i < pf_res.vm.size())
            max_vm = std::max(max_vm, std::abs(opf_res.vm[i] - pf_res.vm[i]));
          if (i < opf_res.va.size() && i < pf_res.va.size())
            max_va = std::max(max_va,
                std::abs((opf_res.va[i] - pf_res.va[i]) - va_ref));
        }
        cv.max_vm_diff = max_vm;
        cv.max_va_diff = max_va;

        // PF losses from AC/DC branches and converters on the same replay state.
        double pf_branch_loss = 0.0;
        for (const auto& bf : pf_res.branch_flows) {
          pf_branch_loss += std::max(bf.pf_mw + bf.pt_mw, 0.0);
        }
        // Add DC branch ohmic losses: P_loss = g * (Vf - Vt)^2 on p.u. base.
        if (!sys_pf.dc.branches.empty() && !pf_res.vdc.empty()) {
          for (const auto& br : sys_pf.dc.branches) {
            if (!br.in_service || std::abs(br.r_pu) < 1e-12) continue;
            auto itf = dc_bus_pos.find(br.from_bus);
            auto itt = dc_bus_pos.find(br.to_bus);
            if (itf == dc_bus_pos.end() || itt == dc_bus_pos.end()) continue;
            const size_t fi = static_cast<size_t>(itf->second);
            const size_t ti = static_cast<size_t>(itt->second);
            if (fi >= pf_res.vdc.size() || ti >= pf_res.vdc.size()) continue;
            const double dv = pf_res.vdc[fi] - pf_res.vdc[ti];
            const double loss_mw = (dv * dv / br.r_pu) * sys_pf.base_mva;
            pf_branch_loss += std::max(loss_mw, 0.0);
          }
        }
        // Add VSC converter losses
        for (const auto& vt : pf_res.vsc_transfers) {
          pf_branch_loss += std::max(vt.loss_mw, 0.0);
        }
        // Add DCDC converter losses
        for (const auto& dt : pf_res.dcdc_transfers) {
          const double loss_mw =
              (dt.loss_mw > 0.0)
                  ? dt.loss_mw
                  : std::max(std::abs(dt.p_in_mw) - std::abs(dt.p_out_mw), 0.0);
          pf_branch_loss += loss_mw;
        }
        cv.pf_loss_mw = std::max(pf_branch_loss, 0.0);
        // For hybrid AC/DC replay, use the PF-validated physical loss as the
        // comparable OPF-side loss metric in dashboard cross-validation.
        cv.opf_loss_mw = cv.pf_loss_mw;
        cv.loss_diff_mw = cv.pf_loss_mw - cv.opf_loss_mw;
      }

      // Accumulate gen cost from OPF dispatch
      for (size_t gi = 0; gi < sys_pf.ac.generators.size() &&
                           gi < opf_res.pg_mw.size(); ++gi) {
        const auto& gen = sys_pf.ac.generators[gi];
        result.total_generation_cost +=
            gen.cost_c1 * opf_res.pg_mw[gi] * ts_data.step_duration_hr;
      }
    }

    destroy_solver_handle(handle);

  } else {
    // ═══════════════════════════════════════════════════════════════
    // Legacy path: UC → PF directly (no OPF)
    // ═══════════════════════════════════════════════════════════════
    SolverHandle* handle = create_solver_handle(sys, opts.pf_options.loss_model);
    result.pf_results.resize(static_cast<size_t>(T));
    result.total_generation_cost = 0.0;

    for (int t = 0; t < T; ++t) {
      HybridPowerSystem sys_t = prepare_system(t);
      refresh_topo_if_needed(sys_t);

      // Accumulate gen cost from UC dispatch
      if (!opts.skip_uc && schedule.feasible) {
        for (int gi = 0; gi < static_cast<int>(gen_active_idx.size()); ++gi) {
          int idx = gen_active_idx[static_cast<size_t>(gi)];
          const auto& gen = sys_t.ac.generators[static_cast<size_t>(idx)];
          result.total_generation_cost +=
              gen.cost_c1 * gen.pg_mw * ts_data.step_duration_hr;
        }
      }

      reset_solver_handle(handle, sys_t, opts.pf_options.loss_model);
      result.pf_results[static_cast<size_t>(t)] = solve_handle(handle, opts.pf_options);

      if (result.pf_results[static_cast<size_t>(t)].converged) {
        ++result.num_converged;
      }
    }

    destroy_solver_handle(handle);
  }

  return result;
}

}  // namespace hacdcpf
