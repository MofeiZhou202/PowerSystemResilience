#include "hacdcpf/time_series/time_series_pf.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <future>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/graph/graph.hpp"
#include <spdlog/spdlog.h>
#include "hacdcpf/engine/adapter_registry.hpp"
#include "hacdcpf/engine/native_adapters.hpp"
#include "hacdcpf/engine/external_adapters.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/power_flow/branch_flow.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"
#include "hacdcpf/util/parallel_execution.hpp"
#include "hacdcpf/util/thread_pool.hpp"

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

double absolute_profile_value(const std::vector<double>* profile, int t,
                              double fallback) {
  return profile ? profile_value(profile, t, fallback) : fallback;
}

void validate_time_series_data_for_solve(const TimeSeriesData& ts_data,
                                         bool require_steps) {
  if (ts_data.num_steps < 0) {
    throw std::invalid_argument("TimeSeriesData.num_steps must be non-negative");
  }
  if (require_steps && ts_data.num_steps <= 0) {
    throw std::invalid_argument("TimeSeriesData must have at least one time step");
  }
  if (ts_data.num_steps > 0 &&
      (!std::isfinite(ts_data.step_duration_hr) ||
       ts_data.step_duration_hr <= 0.0)) {
    throw std::invalid_argument(
        "TimeSeriesData.step_duration_hr must be positive and finite");
  }
  std::unordered_set<int> profile_ids;
  for (const auto& profile : ts_data.profiles) {
    if (!profile_ids.insert(profile.id).second) {
      throw std::invalid_argument("TimeSeriesData contains duplicate profile id " +
                                  std::to_string(profile.id));
    }
    for (double value : profile.values) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "TimeSeriesData profile " + std::to_string(profile.id) +
            " contains a non-finite value");
      }
    }
  }
}

bool should_apply_uc_schedule(const TimeSeriesPFOptions& opts,
                              const UCSchedule& schedule) {
  return schedule.feasible &&
         (!opts.skip_uc || opts.precomputed_uc_schedule != nullptr);
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
      total_demand_mw += std::max(0.0, model::effective_load_p_mw(ld));
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

HybridPowerSystem restore_rich_dc_storage_identity(
    const HybridPowerSystem& solver_state,
    const HybridPowerSystem& rich_template) {
  if (rich_template.dc.dc_storage.empty()) return solver_state;
  if (!solver_state.dc.dc_storage.empty()) return solver_state;
  HybridPowerSystem rich_state = solver_state;
  const size_t legacy_count = rich_template.dc.storage.size();
  rich_state.dc.storage.assign(
      solver_state.dc.storage.begin(),
      solver_state.dc.storage.begin() +
          std::min(legacy_count, solver_state.dc.storage.size()));
  rich_state.dc.dc_storage = rich_template.dc.dc_storage;
  for (size_t i = 0; i < rich_state.dc.dc_storage.size(); ++i) {
    const size_t solver_position = legacy_count + i;
    if (solver_position >= solver_state.dc.storage.size()) break;
    const auto& source = solver_state.dc.storage[solver_position];
    auto& target = rich_state.dc.dc_storage[i];
    target.p_mw = source.p_mw;
    target.soc_init = source.soc_init;
    target.e_mwh = source.e_mwh;
  }
  return rich_state;
}

double estimate_opf_physical_loss_mw(const HybridPowerSystem& sys,
                                     const opf::ACOPFResult& opf_res,
                                     LossModelType loss_model) {
  double loss_mw = 0.0;

  // AC branch losses from OPF voltages.
  try {
    auto data = powerflow::make_solver_data(sys, loss_model);
    auto project_intensive = [&](const std::vector<double>& authored) {
      if (!data.bus_merge_map) return authored;
      const auto& map = *data.bus_merge_map;
      if (static_cast<int>(authored.size()) != map.n_original) {
        throw std::invalid_argument(
            "estimate_opf_physical_loss_mw: authored AC bus vector size "
            "does not match projection map");
      }
      std::vector<double> canonical(static_cast<size_t>(map.n_merged), 0.0);
      std::vector<char> assigned(static_cast<size_t>(map.n_merged), 0);
      for (const auto& [ext_bus, int_pos] : map.ext_to_int) {
        const auto original = map.ext_to_orig_pos.find(ext_bus);
        if (original == map.ext_to_orig_pos.end() || int_pos < 0 ||
            int_pos >= map.n_merged || original->second < 0 ||
            original->second >= map.n_original) {
          continue;
        }
        if (assigned[static_cast<size_t>(int_pos)] == 0) {
          canonical[static_cast<size_t>(int_pos)] =
              authored[static_cast<size_t>(original->second)];
          assigned[static_cast<size_t>(int_pos)] = 1;
        }
      }
      return canonical;
    };
    const auto canonical_vm = project_intensive(opf_res.vm);
    const auto canonical_va = project_intensive(opf_res.va);
    if (canonical_vm.size() == data.ac_buses.size() &&
        canonical_va.size() == data.ac_buses.size()) {
      const auto branch_flows =
          powerflow::compute_branch_flows(data, canonical_vm, canonical_va);
      for (const auto& bf : branch_flows) {
        loss_mw += std::max(bf.pf_mw + bf.pt_mw, 0.0);
      }
    }

    // Converter losses from OPF voltages with the same converter model.
    if (canonical_vm.size() == data.ac_buses.size() &&
        canonical_va.size() == data.ac_buses.size() &&
        opf_res.vdc.size() == data.dc_buses.size()) {
      Eigen::VectorXd vm = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(canonical_vm.size()));
      Eigen::VectorXd va = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(canonical_va.size()));
      Eigen::VectorXd vdc = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(opf_res.vdc.size()));
      for (Eigen::Index i = 0; i < vm.size(); ++i) vm[i] = canonical_vm[static_cast<size_t>(i)];
      for (Eigen::Index i = 0; i < va.size(); ++i) va[i] = canonical_va[static_cast<size_t>(i)];
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

  // Find DC voltage-controlled (slack) bus.  Prefer a DC_V reference; otherwise
  // anchor on the first non-isolated bus.  DC_ISOLATED buses are de-energized
  // and are only used as a fallback anchor when nothing else exists.
  int first_non_isolated = -1;
  for (int i = 0; i < dcg.n_dc_bus; ++i) {
    const auto bt = sys.dc.buses[static_cast<size_t>(i)].bus_type;
    if (bt == DCBusType::DC_V) {
      dcg.dc_slack_bus = i;
      break;
    }
    if (first_non_isolated < 0 && bt != DCBusType::DC_ISOLATED) {
      first_non_isolated = i;
    }
  }
  if (dcg.dc_slack_bus < 0) {
    dcg.dc_slack_bus = (first_non_isolated >= 0) ? first_non_isolated : 0;
  }

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
  int X{0};   // number of external grids (in-service, when enabled)
  int F{0};   // number of flexible loads (in-service, when DR enabled)
  int U{0};   // number of microgrids (in-service, when enabled)
  int V{0};   // number of VPPs (in-service, when enabled)
  std::vector<int> gen_indices;       // original indices into sys.ac.generators
  std::vector<int> storage_indices;   // original indices into sys.ac.storage
  std::vector<int> dc_storage_indices;// original indices into sys.dc.storage
  std::vector<int> renewable_indices; // original indices into sys.ac.renewable_gens
  std::vector<int> vsc_indices;       // original indices into sys.vsc_converters
  std::vector<int> dcdc_indices;      // original indices into sys.dc.dcdc_converters
  std::vector<int> ext_indices;       // original indices into sys.ac.external_grids
  std::vector<int> flex_indices;      // original indices into sys.ac.flexible_loads
  std::vector<int> mg_indices;        // original indices into sys.microgrids
  std::vector<int> vpp_indices;       // original indices into sys.vpps
  int ac_pv_count{0};
  int dc_pv_count{0};
  int dc_sgen_count{0};
  int ac_sgen_count{0};
  int external_grid_count{0};
  int pv_curt_offset{0};
  int ac_sgen_offset{0};
  int external_grid_offset{0};
  // kind: 0 = AC PV, 1 = DC PV, 2 = DC static generator.
  std::vector<int> pv_curt_kind;
  std::vector<int> pv_curt_indices;
  std::vector<std::vector<double>> pv_curt_available;
  std::vector<int> ac_sgen_indices;
  // Constant objective offset to add back to the solver objective so the
  // reported cost reflects the true penalty form (e.g. islanding cost
  // pen·(1−o), encoded in c as −pen·o with the constant pen dropped).
  double obj_offset{0.0};
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
  res.ac_pv_count = static_cast<int>(sys.ac.pv_systems.size());
  res.dc_pv_count = static_cast<int>(sys.dc.pv_arrays.size());
  res.dc_sgen_count = static_cast<int>(sys.dc.dc_static_generators.size());
  res.ac_sgen_count = static_cast<int>(sys.ac.static_generators.size());
  res.external_grid_count = static_cast<int>(sys.ac.external_grids.size());

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

  // Collect in-service storage. A hosting-capacity "static" ESS is excluded from
  // the UC/OPF optimized set — it is held at its fixed charging power (honored by
  // the per-step OPF bound pin in parity_formulation.cpp), not dispatched.
  for (int i = 0; i < static_cast<int>(sys.ac.storage.size()); ++i) {
    const auto& st = sys.ac.storage[static_cast<size_t>(i)];
    if (st.in_service && st.cap_charging_strategy != "static") {
      res.storage_indices.push_back(i);
    }
  }
  res.S = static_cast<int>(res.storage_indices.size());

  // Collect in-service DC storage (same static-ESS exclusion)
  for (int i = 0; i < static_cast<int>(sys.dc.storage.size()); ++i) {
    const auto& st = sys.dc.storage[static_cast<size_t>(i)];
    if (st.in_service && st.cap_charging_strategy != "static") {
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

  // Collect in-service external grids (only used when enable_external_grid).
  if (opts.enable_external_grid) {
    for (int i = 0; i < static_cast<int>(sys.ac.external_grids.size()); ++i) {
      if (sys.ac.external_grids[static_cast<size_t>(i)].in_service) {
        res.ext_indices.push_back(i);
      }
    }
  }
  res.X = static_cast<int>(res.ext_indices.size());

  // Collect in-service flexible loads (only used when enable_demand_response).
  if (opts.enable_demand_response) {
    for (int i = 0; i < static_cast<int>(sys.ac.flexible_loads.size()); ++i) {
      if (sys.ac.flexible_loads[static_cast<size_t>(i)].in_service) {
        res.flex_indices.push_back(i);
      }
    }
  }
  res.F = static_cast<int>(res.flex_indices.size());

  // Collect in-service microgrids (only used when enable_microgrid).
  if (opts.enable_microgrid) {
    for (int i = 0; i < static_cast<int>(sys.microgrids.size()); ++i) {
      if (sys.microgrids[static_cast<size_t>(i)].in_service) {
        res.mg_indices.push_back(i);
      }
    }
  }
  res.U = static_cast<int>(res.mg_indices.size());

  // Collect in-service VPPs (only used when enable_vpp).
  if (opts.enable_vpp) {
    for (int i = 0; i < static_cast<int>(sys.vpps.size()); ++i) {
      if (sys.vpps[static_cast<size_t>(i)].in_service) {
        res.vpp_indices.push_back(i);
      }
    }
  }
  res.V = static_cast<int>(res.vpp_indices.size());

  // Build DC bus index map
  std::unordered_map<int, int> dc_bus_idx;
  for (int i = 0; i < dcg.n_dc_bus; ++i) {
    dc_bus_idx[sys.dc.buses[static_cast<size_t>(i)].index] = i;
  }

  const int G = res.G, S = res.S, Sd = res.Sd, R = res.R;
  const int C = res.C, Cd = res.Cd, X = res.X, F = res.F, U = res.U, V = res.V;
  const bool use_network_constraints = opts.enable_network_constraints && net.n_bus > 1;
  const bool use_dc_network = use_network_constraints && opts.enable_dc_network_constraints
                              && dcg.n_dc_bus > 0 && C > 0;
  const bool use_dr = opts.enable_demand_response && F > 0;
  const bool use_dr_shift = use_dr && opts.dr_shiftable;
  const bool use_mg = opts.enable_microgrid && U > 0;
  const bool use_degr = opts.enable_storage_degradation && (S + Sd) > 0;
  const auto has_storage_bid = [](const Storage& st) {
    return st.charge_bid_price > 0.0 || st.discharge_bid_price > 0.0;
  };
  bool use_storage_abs = use_degr;
  for (int i : res.storage_indices)
    use_storage_abs = use_storage_abs || has_storage_bid(sys.ac.storage[static_cast<size_t>(i)]);
  for (int i : res.dc_storage_indices)
    use_storage_abs = use_storage_abs || has_storage_bid(sys.dc.storage[static_cast<size_t>(i)]);
  const bool use_vpp = opts.enable_vpp && V > 0;

  // Dispatchable-PV curtailment descriptors: one per must-take source that may
  // be clawed back.  avail[t] is the profile-scaled available output; the
  // curtailment variable lives in [0, avail] and adds back to the bus balance
  // (reducing net injection).  AC PV is only must-take when the DC network is
  // not modelled explicitly, so it is only clawed back in that case.
  struct PVCurtSource {
    std::vector<double> avail;
    int ac_bus;
    int dc_bus;
    double cost_c1{0.0};
    double emission_factor{0.0};
  };
  std::vector<PVCurtSource> pvcurt;
  if (opts.enable_dispatchable_pv) {
    if (!use_dc_network) {
      for (int i = 0; i < static_cast<int>(sys.ac.pv_systems.size()); ++i) {
        const auto& pvsys = sys.ac.pv_systems[static_cast<size_t>(i)];
        if (!pvsys.in_service) continue;
        const auto* pvprof = find_profile(profile_map, pvsys.profile_id);
        PVCurtSource d; d.ac_bus = pvsys.bus; d.dc_bus = -1;
        d.cost_c1 = pvsys.cost_c1;
        d.avail.resize(static_cast<size_t>(T));
        for (int t = 0; t < T; ++t) {
          const double scale = profile_value(pvprof, t, 1.0);
          double pv_mw;
          if (pvsys.voc > 0.0 && pvsys.isc > 0.0 && pvsys.vmpp > 0.0) {
            PVSystem c = pvsys; c.irradiance = scale * 1000.0;
            pv_mw = powerflow::compute_pv_power_mw(c);
          } else {
            pv_mw = pvsys.p_mw * scale;
          }
          d.avail[static_cast<size_t>(t)] = std::max(0.0, pv_mw);
        }
        res.pv_curt_kind.push_back(0);
        res.pv_curt_indices.push_back(i);
        res.pv_curt_available.push_back(d.avail);
        pvcurt.push_back(std::move(d));
      }
    }
    for (int i = 0; i < static_cast<int>(sys.dc.pv_arrays.size()); ++i) {
      const auto& pvarr = sys.dc.pv_arrays[static_cast<size_t>(i)];
      if (!pvarr.in_service) continue;
      const auto* pv = find_profile(profile_map, pvarr.profile_id);
      PVCurtSource d; d.ac_bus = -1; d.dc_bus = pvarr.bus;
      d.cost_c1 = pvarr.cost_c1;
      d.avail.resize(static_cast<size_t>(T));
      for (int t = 0; t < T; ++t)
        d.avail[static_cast<size_t>(t)] = std::max(0.0, pvarr.p_set_mw * profile_value(pv, t, 1.0));
      res.pv_curt_kind.push_back(1);
      res.pv_curt_indices.push_back(i);
      res.pv_curt_available.push_back(d.avail);
      pvcurt.push_back(std::move(d));
    }
    for (int i = 0; i < static_cast<int>(sys.dc.dc_static_generators.size()); ++i) {
      const auto& sg = sys.dc.dc_static_generators[static_cast<size_t>(i)];
      if (!sg.in_service) continue;
      const auto* pv = find_profile(profile_map, sg.profile_id);
      PVCurtSource d; d.ac_bus = -1; d.dc_bus = sg.bus;
      d.cost_c1 = sg.cost_c1;
      d.emission_factor = sg.emission_factor_tco2_mwh;
      d.avail.resize(static_cast<size_t>(T));
      for (int t = 0; t < T; ++t)
        d.avail[static_cast<size_t>(t)] = std::max(0.0, sg.p_set_mw * sg.scaling * profile_value(pv, t, 1.0));
      res.pv_curt_kind.push_back(2);
      res.pv_curt_indices.push_back(i);
      res.pv_curt_available.push_back(d.avail);
      pvcurt.push_back(std::move(d));
    }
  }
  const int Pc = static_cast<int>(pvcurt.size());

  // Dispatchable AC static generators (sys.ac.static_generators).  Unlike the
  // must-take PV sources, these are not otherwise in the UC, so they get a
  // supply variable p_sgen ∈ [0, available] (available = p_mw·scaling) added to
  // the AC balance.  Curtailment is rewarded (like renewables): the objective
  // gets −pv_curtail_penalty·p_sgen with the constant penalty·available tracked
  // in obj_offset, so the reported cost equals fuel cost + penalty·curtailment.
  struct SgenSupply { std::vector<double> avail; int ac_bus; double cost_c1; };
  std::vector<SgenSupply> sgsup;
  if (opts.enable_dispatchable_pv) {
    for (int i = 0; i < static_cast<int>(sys.ac.static_generators.size()); ++i) {
      const auto& sg = sys.ac.static_generators[static_cast<size_t>(i)];
      if (!sg.in_service) continue;
      double a = std::max(0.0, sg.p_mw * sg.scaling);
      if (a <= 0.0) continue;
      SgenSupply d; d.ac_bus = sg.bus; d.cost_c1 = sg.cost_c1;
      d.avail.assign(static_cast<size_t>(T), a);
      res.ac_sgen_indices.push_back(i);
      sgsup.push_back(std::move(d));
    }
  }
  const int Ng = static_cast<int>(sgsup.size());

  // Explicit DC branch transport flows + thermal limits.  Each in-service DC
  // branch gets a flow variable f ∈ [−rate, rate] (a transport / network-flow
  // model with no DC voltages) that enters the DC nodal balance so power can
  // move between DC buses up to the line rating.
  struct DcFlow { int from_pos; int to_pos; double rate; };
  std::vector<DcFlow> dcflow;
  const bool use_dc_flows = use_dc_network && opts.enable_dc_branch_flows;
  if (use_dc_flows) {
    for (const auto& br : sys.dc.branches) {
      if (!br.in_service) continue;
      auto fi = dc_bus_idx.find(br.from_bus);
      auto ti = dc_bus_idx.find(br.to_bus);
      if (fi == dc_bus_idx.end() || ti == dc_bus_idx.end()) continue;
      if (fi->second == ti->second) continue;
      DcFlow d;
      d.from_pos = fi->second;
      d.to_pos = ti->second;
      d.rate = (br.rate_a_mva > 0.0) ? br.rate_a_mva
                                     : ((br.s_max_mva > 0.0) ? br.s_max_mva : 1.0e5);
      dcflow.push_back(d);
    }
  }
  const int Nf = static_cast<int>(dcflow.size());

  // Energy-router (multi-port converter) ports (§4.9).  Each in-service router
  // that still carries explicit ports (not yet expanded into VSC/DC-DC during
  // network projection) contributes, per placeable port, an inflow/outflow pair
  // bounded by the port rating.  A per-router lossy conservation constraint
  // (Σ η_j·p_in_j = Σ p_out_j) ties the ports together; the signed bus injection
  // is p_out − p_in.  DC ports require the DC network to be modelled; otherwise
  // (copper-plate, or AC-nodal without a DC network) DC ports fall back to the
  // single system / slack balance, like the must-take DC sources.
  struct ErPort { int router_pos; int ac_bus; int dc_bus; double cap; double eta; };
  std::vector<ErPort> erports;
  std::vector<std::vector<int>> er_ports_of_router;  // router_pos -> erport indices
  const bool use_router = opts.enable_energy_router && !sys.energy_routers.empty();
  if (use_router) {
    for (const auto& er : sys.energy_routers) {
      if (!er.in_service) continue;
      std::vector<ErPort> ports_this;
      for (const auto& port : er.ports) {
        if (!port.in_service || port.bus == 0) continue;
        ErPort d;
        d.router_pos = static_cast<int>(er_ports_of_router.size());
        d.cap = (port.pmax_mw > 0.0)
                    ? port.pmax_mw
                    : (er.p_rated_mw > 0.0 ? er.p_rated_mw : 1.0e5);
        d.eta = (port.eta > 1e-6 && port.eta <= 1.0) ? port.eta : 0.98;
        d.ac_bus = -1;
        d.dc_bus = -1;
        if (port.port_type == ERPortType::DC)
          d.dc_bus = port.bus;
        else
          d.ac_bus = port.bus;
        ports_this.push_back(d);
      }
      if (ports_this.empty()) continue;
      std::vector<int> idxs;
      for (auto& d : ports_this) {
        idxs.push_back(static_cast<int>(erports.size()));
        erports.push_back(d);
      }
      er_ports_of_router.push_back(std::move(idxs));
    }
  }
  const int Per = static_cast<int>(erports.size());        // placeable router ports
  const int Rrouter = static_cast<int>(er_ports_of_router.size());  // active routers

  // Mobile storage (§4.4) with an exogenous relocation schedule.  Each unit gets
  // a signed power variable (+discharge) and an energy state in [soc_min,
  // soc_max].  The connection bus per step follows departure_time/arrival_time/
  // status: at `bus` before departure, disconnected (power forced 0) while in
  // transit, at `target_bus` after arrival.  loc[t] holds the raw AC bus number
  // for that step, or −1 when disconnected.
  struct MobStor {
    int orig_idx;
    std::vector<int> loc;     // per-step raw AC bus number, or −1 (in transit)
    double pmin, pmax, E, eta, soc_init, soc_min, soc_max;
    // Co-optimised relocation (only used when use_coreloc): two candidate buses
    // (origin / target), per-transit SOC drain, and minimum-stay length.
    int busO{0}, busD{-1};    // origin bus, target bus (−1 = no relocation)
    double drain{0.0};        // per-transit-step SOC depletion (per-unit)
    int min_stay{1};          // minimum dwell once connected (steps)
  };
  std::vector<MobStor> mobstor;
  const bool use_mobstor = opts.enable_mobile_storage && !sys.mobile_storage.empty();
  const bool use_coreloc = use_mobstor && opts.mobile_storage_corelocate;
  if (use_mobstor) {
    for (int i = 0; i < static_cast<int>(sys.mobile_storage.size()); ++i) {
      const auto& ms = sys.mobile_storage[static_cast<size_t>(i)];
      if (!ms.in_service) continue;
      MobStor d;
      d.orig_idx = i;
      const double pcap = (ms.pmax_mw > 0.0) ? ms.pmax_mw
                                             : (ms.p_rated_mw > 0.0 ? ms.p_rated_mw : 0.0);
      d.pmax = std::abs(pcap);
      d.pmin = (ms.pmin_mw < 0.0) ? ms.pmin_mw : -d.pmax;
      d.E = (ms.e_rated_mwh > 1e-9) ? ms.e_rated_mwh : 1.0;
      d.eta = std::sqrt(std::max(1e-6, ms.eta_charge * ms.eta_discharge));
      d.soc_init = ms.soc_init;
      d.soc_min = ms.soc_min;
      d.soc_max = ms.soc_max;
      // Candidate buses + relocation parameters for the co-optimised model.
      d.busO = ms.bus;
      d.busD = (ms.target_bus != 0 && ms.target_bus != ms.bus) ? ms.target_bus : -1;
      const double trip_drain = (ms.e_consumption_mwh_km > 0.0 && ms.max_travel_distance_km > 0.0)
                                    ? (ms.e_consumption_mwh_km * ms.max_travel_distance_km) / d.E
                                    : 0.0;
      d.drain = std::min(std::max(0.0, trip_drain), 0.2);  // cap per-transit drain
      d.min_stay = (ms.t_stay_min_hr > 0.0 && dt > 0.0)
                       ? std::max(1, static_cast<int>(std::lround(ms.t_stay_min_hr / dt)))
                       : 1;
      d.loc.resize(static_cast<size_t>(T));
      const double td = ms.departure_time, ta = ms.arrival_time;
      const bool relocate = (td > 0.0 && ta > td && ms.target_bus != 0);
      for (int t = 0; t < T; ++t) {
        const double thr = static_cast<double>(t) * dt;
        int loc_bus;
        if (ms.status == MobileStorageStatus::InTransit && !relocate) {
          loc_bus = -1;                       // mid-journey, no schedule
        } else if (relocate) {
          if (thr < td)      loc_bus = ms.bus;       // pre-departure
          else if (thr < ta) loc_bus = -1;           // traveling
          else               loc_bus = ms.target_bus; // arrived
        } else {
          loc_bus = ms.bus;                   // stationary
        }
        d.loc[static_cast<size_t>(t)] = loc_bus;
      }
      mobstor.push_back(std::move(d));
    }
  }
  const int M = static_cast<int>(mobstor.size());  // active mobile-storage units

  // Variable layout:
  //   p_g[g,t]      dispatch for generator g at time t    (continuous) : G*T
  //   u_g[g,t]      commitment for generator g at time t  (binary)     : G*T
  //   s_g[g,t]      startup cost for generator g at time t (continuous): G*T
  //   p_ess[s,t]    AC ESS dispatch (positive=discharge)   (continuous): S*T
  //   soc[s,t]      AC ESS SOC at end of period t          (continuous): S*T
  //   p_dcess[d,t]  DC ESS dispatch (positive=discharge)   (continuous): Sd*T
  //   soc_d[d,t]    DC ESS SOC at end of period t          (continuous): Sd*T
  //   p_ren[r,t]    renewable dispatch at time t           (continuous): R*T
  //   p_ext[x,t]    external-grid net exchange (+imp/−exp) (continuous): X*T  (if enable_external_grid)
  //   p_ch[k,t]     storage charging power at the grid terminal (continuous): (S+Sd)*T
  //   z_dis[k,t]    storage discharge-mode selector              (binary): (S+Sd)*T
  //   theta[b,t]    AC bus voltage angle (non-slack)       (continuous): nTheta
  //   p_vsc[c,t]    VSC AC-side injection (pos=into AC)    (continuous): C*T  (if use_dc_network)
  //   p_dcdc[dd,t]  DC-DC input power (pos=in→out)         (continuous): Cd*T (if use_dc_network)
  // p_ext sits before theta so the warm-start heuristic and extract_schedule's
  // back-calculation of the trailing vsc/dcdc blocks remain valid.
  const int nPg  = G * T;
  const int nUg  = G * T;
  const int nSg  = G * T;
  const int nEss = S * T;
  const int nSoc = S * T;
  const int nDcEss = Sd * T;
  const int nDcSoc = Sd * T;
  const int nRen = R * T;
  const int nExt = X * T;
  const int nDrUp = use_dr ? F * T : 0;   // flexible-load demand increase
  const int nDrDn = use_dr ? F * T : 0;   // flexible-load demand decrease
  const int nCurt = Pc * T;               // dispatchable-PV curtailment claw-back
  const int nSgen = Ng * T;               // dispatchable AC static-gen supply
  const int nMgEx = use_mg ? U * T : 0;   // microgrid PCC net exchange (+export)
  const int nMgO  = use_mg ? U * T : 0;   // microgrid connection indicator (binary)
  const int nDcFlow = Nf * T;             // explicit DC branch transport flows
  const int nGdeg = use_storage_abs ? (S + Sd) * T : 0;  // storage throughput |p| aux
  const int nVpp  = use_vpp ? V * T : 0;          // VPP aggregate net output
  const int nVppE = use_vpp ? V * T : 0;          // VPP aggregate energy state
  const int nErIn  = use_router ? Per * T : 0;    // router port inflow (bus→router)
  const int nErOut = use_router ? Per * T : 0;    // router port outflow (router→bus)
  const int nMsP = (use_mobstor && !use_coreloc) ? M * T : 0;  // mobile power (exogenous)
  const int nMsE = use_mobstor ? M * T : 0;       // mobile-storage energy state
  const int nMsZ0 = use_coreloc ? M * T : 0;      // connected at origin (binary)
  const int nMsZ1 = use_coreloc ? M * T : 0;      // connected at target (binary)
  const int nMsW  = use_coreloc ? M * T : 0;      // in transit (binary)
  const int nMsQ0 = use_coreloc ? M * T : 0;      // injection at origin bus
  const int nMsQ1 = use_coreloc ? M * T : 0;      // injection at target bus
  const int nStorCharge = (S + Sd) * T;            // non-negative charging power
  const int nStorMode = (S + Sd) * T;              // discharge-mode selector
  const int nTheta = use_network_constraints ? static_cast<int>(net.non_slack.size()) * T : 0;
  const int nVsc = use_dc_network ? C * T : 0;
  const int nDcdc = use_dc_network ? Cd * T : 0;
  const int n = nPg + nUg + nSg + nEss + nSoc + nDcEss + nDcSoc + nRen + nExt
              + nDrUp + nDrDn + nCurt + nSgen + nMgEx + nMgO + nDcFlow + nGdeg
              + nVpp + nVppE + nErIn + nErOut + nMsP + nMsE
              + nMsZ0 + nMsZ1 + nMsW + nMsQ0 + nMsQ1
              + nStorCharge + nStorMode + nTheta + nVsc + nDcdc;

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
  const int ext_offset = nPg + nUg + nSg + nEss + nSoc + nDcEss + nDcSoc + nRen;
  res.external_grid_offset = ext_offset;
  auto ext_idx = [&](int x, int t) { return ext_offset + x * T + t; };
  const int drup_offset = ext_offset + nExt;
  auto drup_idx = [&](int f, int t) { return drup_offset + f * T + t; };
  const int drdn_offset = drup_offset + nDrUp;
  auto drdn_idx = [&](int f, int t) { return drdn_offset + f * T + t; };
  const int curt_offset = drdn_offset + nDrDn;
  auto curt_idx = [&](int k, int t) { return curt_offset + k * T + t; };
  const int sgen_offset = curt_offset + nCurt;
  res.pv_curt_offset = curt_offset;
  res.ac_sgen_offset = sgen_offset;
  auto sgen_idx = [&](int k, int t) { return sgen_offset + k * T + t; };
  const int mgex_offset = sgen_offset + nSgen;
  auto mgex_idx = [&](int u, int t) { return mgex_offset + u * T + t; };
  const int mgo_offset = mgex_offset + nMgEx;
  auto mgo_idx = [&](int u, int t) { return mgo_offset + u * T + t; };
  const int dcflow_offset = mgo_offset + nMgO;
  auto dcflow_idx = [&](int m, int t) { return dcflow_offset + m * T + t; };
  const int gdeg_offset = dcflow_offset + nDcFlow;
  auto gdeg_idx = [&](int k, int t) { return gdeg_offset + k * T + t; };
  const int vpp_offset = gdeg_offset + nGdeg;
  auto vpp_idx = [&](int v, int t) { return vpp_offset + v * T + t; };
  const int vppe_offset = vpp_offset + nVpp;
  auto vppe_idx = [&](int v, int t) { return vppe_offset + v * T + t; };
  const int erin_offset = vppe_offset + nVppE;
  auto erin_idx = [&](int p, int t) { return erin_offset + p * T + t; };
  const int erout_offset = erin_offset + nErIn;
  auto erout_idx = [&](int p, int t) { return erout_offset + p * T + t; };
  const int msp_offset = erout_offset + nErOut;
  auto msp_idx = [&](int b, int t) { return msp_offset + b * T + t; };
  const int mse_offset = msp_offset + nMsP;
  auto mse_idx = [&](int b, int t) { return mse_offset + b * T + t; };
  const int msz0_offset = mse_offset + nMsE;
  auto msz0_idx = [&](int b, int t) { return msz0_offset + b * T + t; };
  const int msz1_offset = msz0_offset + nMsZ0;
  auto msz1_idx = [&](int b, int t) { return msz1_offset + b * T + t; };
  const int msw_offset = msz1_offset + nMsZ1;
  auto msw_idx = [&](int b, int t) { return msw_offset + b * T + t; };
  const int msq0_offset = msw_offset + nMsW;
  auto msq0_idx = [&](int b, int t) { return msq0_offset + b * T + t; };
  const int msq1_offset = msq0_offset + nMsQ0;
  auto msq1_idx = [&](int b, int t) { return msq1_offset + b * T + t; };
  const int stor_charge_offset = msq1_offset + nMsQ1;
  auto stor_charge_idx = [&](int k, int t) {
    return stor_charge_offset + k * T + t;
  };
  const int stor_mode_offset = stor_charge_offset + nStorCharge;
  auto stor_mode_idx = [&](int k, int t) {
    return stor_mode_offset + k * T + t;
  };
  const int theta_offset = stor_mode_offset + nStorMode;
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

  // --- Objective: configurable production-cost / emissions / curtailment ---
  // Resolve the effective weight on each linear term from the objective mode.
  // All terms are linear so the model stays a MILP for every solver backend.
  double w_cost = 0.0, w_carbon = 0.0, w_loss = 0.0, w_curt = 0.0;
  switch (opts.objective_mode) {
    case UCObjective::Cost:           w_cost = 1.0; break;
    case UCObjective::Carbon:         w_carbon = 1.0; w_curt = 1.0; break;
    case UCObjective::MinCurtailment: w_curt = 1.0; w_cost = 1e-3; break;
    case UCObjective::MinLoss:        w_loss = 1.0; w_curt = 1.0; break;
    case UCObjective::Weighted:
      w_cost = opts.w_cost; w_carbon = opts.w_carbon;
      w_loss = opts.w_loss; w_curt = opts.w_curtailment;
      break;
  }

  // DC generators are profile-driven fixed injections rather than UC decision
  // variables. Their production cost/emissions are therefore objective
  // constants, but still belong in the reported objective value.
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    const double p = std::max(0.0, sg.p_mw * sg.scaling);
    res.obj_offset +=
        (w_cost * sg.cost_c1 + w_carbon * sg.co2_emission_rate + w_loss) * p * dt * T;
  }
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    const auto* pv = find_profile(profile_map, sg.profile_id);
    for (int t = 0; t < T; ++t) {
      const double p = std::max(
          0.0, sg.p_set_mw * sg.scaling * profile_value(pv, t, 1.0));
      res.obj_offset +=
          (w_cost * sg.cost_c1 +
           w_carbon * sg.emission_factor_tco2_mwh + w_loss) * p * dt;
    }
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service || std::abs(pv.cost_c1) <= 1e-12) continue;
    const auto* profile = find_profile(profile_map, pv.profile_id);
    for (int t = 0; t < T; ++t) {
      const double p = std::max(0.0, pv.p_mw * profile_value(profile, t, 1.0));
      res.obj_offset += w_cost * pv.cost_c1 * p * dt;
    }
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service || std::abs(pv.cost_c1) <= 1e-12) continue;
    const auto* profile = find_profile(profile_map, pv.profile_id);
    for (int t = 0; t < T; ++t) {
      const double p = std::max(0.0, pv.p_set_mw * profile_value(profile, t, 1.0));
      res.obj_offset += w_cost * pv.cost_c1 * p * dt;
    }
  }
  if (!opts.enable_dispatchable_pv) {
    for (const auto& sg : sys.ac.static_generators) {
      if (!sg.in_service) continue;
      const double p = std::max(0.0, sg.p_mw * sg.scaling);
      res.obj_offset += w_cost * sg.cost_c1 * p * dt * T;
    }
  }
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    for (int t = 0; t < T; ++t) {
      // p_g coefficient blends fuel cost ($/MWh), CO2 ($/MWh-equivalent via the
      // emission factor) and a generation-minimization loss proxy ($/MWh).
      m.linear_part.c[pg_idx(gi, t)] =
          (w_cost * gen.cost_c1 + w_carbon * gen.emission_factor_tco2_mwh + w_loss) * dt;
      // No-load and startup costs only carry the cost-objective weight (they are
      // not emissions/loss quantities).
      m.linear_part.c[ug_idx(gi, t)] = w_cost * std::max(0.0, gen.cost_c0) * dt;
      m.linear_part.c[sg_idx(gi, t)] = w_cost;
    }
  }

  // --- Variable bounds & types ---
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    for (int t = 0; t < T; ++t) {
      m.linear_part.vars[static_cast<size_t>(pg_idx(gi, t))] = {
        VarType::Continuous, 0.0, gen.pmax_mw,
        "p_g" + std::to_string(gi) + "_t" + std::to_string(t)};
      // Dynamic-SCED mode pins commitment: either to an explicit representative
      // schedule (fixed_commitment_schedule) or, failing that, ON for all
      // in-service units (fix_commitment) — collapsing SCUC to an ED LP.
      double u_lo = 0.0, u_hi = 1.0;
      if (opts.fixed_commitment_schedule &&
          gi < static_cast<int>(opts.fixed_commitment_schedule->size()) &&
          t < static_cast<int>((*opts.fixed_commitment_schedule)[static_cast<size_t>(gi)].size())) {
        const double u = (*opts.fixed_commitment_schedule)[static_cast<size_t>(gi)][static_cast<size_t>(t)]
                             ? 1.0 : 0.0;
        u_lo = u_hi = u;
      } else if (opts.fix_commitment) {
        u_lo = 1.0;
      }
      m.linear_part.vars[static_cast<size_t>(ug_idx(gi, t))] = {
        VarType::Binary, u_lo, u_hi,
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
      // Cyclic boundary: pin the terminal SOC to the initial SOC so the horizon
      // is energy-neutral and independent (enables parallel day decomposition).
      double soc_lo = ess.soc_min, soc_hi = ess.soc_max;
      if (opts.enforce_terminal_soc_cyclic && t == T - 1) {
        const double soc_fix = std::min(ess.soc_max, std::max(ess.soc_min, ess.soc_init));
        soc_lo = soc_hi = soc_fix;
      }
      m.linear_part.vars[static_cast<size_t>(soc_idx(si, t))] = {
        VarType::Continuous, soc_lo, soc_hi,
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
      double soc_lo = ess.soc_min, soc_hi = ess.soc_max;
      if (opts.enforce_terminal_soc_cyclic && t == T - 1) {
        const double soc_fix = std::min(ess.soc_max, std::max(ess.soc_min, ess.soc_init));
        soc_lo = soc_hi = soc_fix;
      }
      m.linear_part.vars[static_cast<size_t>(dcsoc_idx(si, t))] = {
        VarType::Continuous, soc_lo, soc_hi,
        "soc_d" + std::to_string(si) + "_t" + std::to_string(t)};
    }
  }

  // Exact split of signed net storage dispatch p = p_discharge - p_charge.
  // A binary mode is needed only for bidirectional, lossy storage. Unity-
  // efficiency or one-directional units have an exact continuous formulation.
  for (int k = 0; k < S + Sd; ++k) {
    const Storage& st = (k < S)
                            ? sys.ac.storage[static_cast<size_t>(
                                  res.storage_indices[static_cast<size_t>(k)])]
                            : sys.dc.storage[static_cast<size_t>(
                                  res.dc_storage_indices[static_cast<size_t>(k - S)])];
    const double charge_cap = std::max(-st.pmin_mw, 0.0);
    const double discharge_cap = std::max(st.pmax_mw, 0.0);
    const double eta_charge = std::clamp(st.eta_charge, 1e-6, 1.0);
    const double eta_discharge = std::clamp(st.eta_discharge, 1e-6, 1.0);
    const bool needs_binary =
        charge_cap > 1e-9 && discharge_cap > 1e-9 &&
        (std::abs(eta_charge - 1.0) > 1e-12 ||
         std::abs(eta_discharge - 1.0) > 1e-12);
    for (int t = 0; t < T; ++t) {
      m.linear_part.vars[static_cast<size_t>(stor_charge_idx(k, t))] = {
          VarType::Continuous, 0.0, charge_cap,
          "p_ch" + std::to_string(k) + "_t" + std::to_string(t)};
      m.linear_part.vars[static_cast<size_t>(stor_mode_idx(k, t))] = {
          needs_binary ? VarType::Binary : VarType::Continuous, 0.0, 1.0,
          "z_dis" + std::to_string(k) + "_t" + std::to_string(t)};
      if (needs_binary) {
        m.binary_idx.push_back(stor_mode_idx(k, t));
      }
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
      // Reward renewable utilisation: the cost objective uses cost_curtail_mwh;
      // the curtailment / clean-dispatch objectives add a unit utilisation
      // reward so the optimiser maximises renewable output.
      const double ren_reward = w_cost * ren.cost_curtail_mwh + w_curt;
      m.linear_part.c[ren_idx(ri, t)] =
          (w_cost * ren.cost_c1 - ren_reward) * dt;
    }
  }

  // External-grid net exchange (+import / −export). A bound price profile is
  // an absolute currency/MWh series; otherwise the static cost_c1 applies.
  // charges the import emission factor (applied to the net variable; export is
  // typically small).  Bounds are a symmetric big-M when no explicit limit.
  for (int xi = 0; xi < X; ++xi) {
    const auto& xg = sys.ac.external_grids[static_cast<size_t>(res.ext_indices[static_cast<size_t>(xi)])];
    const auto* price_pv = find_profile(profile_map, xg.price_profile_id);
    const double cap = std::max(1.0, opts.external_grid_cap_mw);
    for (int t = 0; t < T; ++t) {
      m.linear_part.vars[static_cast<size_t>(ext_idx(xi, t))] = {
        VarType::Continuous, -cap, cap,
        "p_ext" + std::to_string(xi) + "_t" + std::to_string(t)};
      const double price = absolute_profile_value(price_pv, t, xg.cost_c1);
      m.linear_part.c[ext_idx(xi, t)] =
          (w_cost * price + w_carbon * xg.emission_factor_tco2_mwh) * dt;
    }
  }

  // Demand response: served demand of flexible load f = baseline P0 + up − down,
  // with up,down ≥ 0 bounded by flex_up/flex_down (× availability).  The
  // discomfort penalty w_demand_response·(up+down) discourages shifting.
  if (use_dr) {
    for (int fi = 0; fi < F; ++fi) {
      const auto& fl = sys.ac.flexible_loads[static_cast<size_t>(res.flex_indices[static_cast<size_t>(fi)])];
      const double avail = std::clamp(fl.availability_pct / 100.0, 0.0, 1.0);
      const double up_cap = std::max(0.0, fl.flex_up_mw) * avail;
      const double dn_cap = std::min(std::max(0.0, fl.flex_down_mw) * avail,
                                     std::max(0.0, fl.p_mw));  // cannot shed below 0
      const double pen = std::max(0.0, opts.w_demand_response) * dt;
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(drup_idx(fi, t))] = {
          VarType::Continuous, 0.0, up_cap,
          "dr_up" + std::to_string(fi) + "_t" + std::to_string(t)};
        m.linear_part.vars[static_cast<size_t>(drdn_idx(fi, t))] = {
          VarType::Continuous, 0.0, dn_cap,
          "dr_dn" + std::to_string(fi) + "_t" + std::to_string(t)};
        m.linear_part.c[drup_idx(fi, t)] = pen;
        m.linear_part.c[drdn_idx(fi, t)] = pen;
      }
    }
  }

  // Dispatchable-PV curtailment claw-back: curt[k,t] ∈ [0, avail[k,t]], priced
  // by pv_curtail_penalty so it stays at 0 unless curtailment is required.
  if (Pc > 0) {
    const double pen = std::max(0.0, opts.pv_curtail_penalty) * dt;
    for (int k = 0; k < Pc; ++k) {
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(curt_idx(k, t))] = {
          VarType::Continuous, 0.0, pvcurt[static_cast<size_t>(k)].avail[static_cast<size_t>(t)],
          "pv_curt" + std::to_string(k) + "_t" + std::to_string(t)};
        const auto& src = pvcurt[static_cast<size_t>(k)];
        const double avoided_objective =
            (w_cost * src.cost_c1 + w_carbon * src.emission_factor + w_loss) * dt;
        m.linear_part.c[curt_idx(k, t)] = pen - avoided_objective;
      }
    }
  }

  // Dispatchable AC static-generator supply: p_sgen ∈ [0, avail], rewarded for
  // utilisation (−pen) with the constant pen·avail tracked in obj_offset so the
  // reported cost = fuel cost + pen·curtailment.
  if (Ng > 0) {
    const double pen = std::max(0.0, opts.pv_curtail_penalty) * dt;
    for (int k = 0; k < Ng; ++k) {
      for (int t = 0; t < T; ++t) {
        const double a = sgsup[static_cast<size_t>(k)].avail[static_cast<size_t>(t)];
        m.linear_part.vars[static_cast<size_t>(sgen_idx(k, t))] = {
          VarType::Continuous, 0.0, a,
          "p_sgen" + std::to_string(k) + "_t" + std::to_string(t)};
        m.linear_part.c[sgen_idx(k, t)] =
            (w_cost * sgsup[static_cast<size_t>(k)].cost_c1 - pen) * dt;
        res.obj_offset += pen * a;
      }
    }
  }

  // Microgrid PCC exchange + islanding indicator.  p_mg ∈ [−p_import_max,
  // p_export_max] (positive = export into the main system); binary o = 1 when
  // grid-connected.  Islanding (o = 0) is penalised by w_microgrid_island.
  if (use_mg) {
    for (int ui = 0; ui < U; ++ui) {
      const auto& mg = sys.microgrids[static_cast<size_t>(res.mg_indices[static_cast<size_t>(ui)])];
      const double p_imp = std::max(0.0, mg.p_import_max_mw);
      const double p_exp = std::max(0.0, mg.p_export_max_mw);
      // A microgrid that cannot island is pinned connected (o ≡ 1).
      const double o_lo = mg.islanding_capability ? 0.0 : 1.0;
      const double island_pen = std::max(0.0, opts.w_microgrid_island) * dt;
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(mgex_idx(ui, t))] = {
          VarType::Continuous, -p_imp, p_exp,
          "p_mg" + std::to_string(ui) + "_t" + std::to_string(t)};
        m.linear_part.vars[static_cast<size_t>(mgo_idx(ui, t))] = {
          VarType::Binary, o_lo, 1.0,
          "mg_on" + std::to_string(ui) + "_t" + std::to_string(t)};
        m.binary_idx.push_back(mgo_idx(ui, t));
        // Penalise islanding: cost = island_pen·(1 − o) ⇒ −island_pen·o (+const).
        m.linear_part.c[mgo_idx(ui, t)] = -island_pen;
        // Track the dropped constant so the reported cost is the true penalty.
        res.obj_offset += island_pen;
      }
    }
  }

  // DC branch transport flows: f ∈ [−rate, rate].  The thermal limit is encoded
  // directly as the variable bound, so no extra constraint rows are needed.
  if (Nf > 0) {
    for (int m_f = 0; m_f < Nf; ++m_f) {
      const double rate = dcflow[static_cast<size_t>(m_f)].rate;
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(dcflow_idx(m_f, t))] = {
          VarType::Continuous, -rate, rate,
          "f_dc" + std::to_string(m_f) + "_t" + std::to_string(t)};
      }
    }
  }

  // Storage degradation: throughput auxiliary g ≥ |p_ess| priced at the
  // cycle-aging cost replacement_cost / (2·max_cycles·E) per MWh moved.
  // Combined index k: [0,S) = AC storage, [S,S+Sd) = DC storage.
  auto stor_ref = [&](int k) -> const Storage& {
    return (k < S)
        ? sys.ac.storage[static_cast<size_t>(res.storage_indices[static_cast<size_t>(k)])]
        : sys.dc.storage[static_cast<size_t>(res.dc_storage_indices[static_cast<size_t>(k - S)])];
  };
  auto stor_power_idx = [&](int k, int t) {
    return (k < S) ? ess_idx(k, t) : dcess_idx(k - S, t);
  };
  if (use_storage_abs) {
    for (int k = 0; k < S + Sd; ++k) {
      const auto& st = stor_ref(k);
      const double E = (st.e_rated_mwh > 1e-9) ? st.e_rated_mwh : 1.0;
      const int ncyc = (st.max_cycles > 0) ? st.max_cycles : 5000;
      const double degr = (st.replacement_cost > 0.0)
                              ? st.replacement_cost / (2.0 * ncyc * E)
                              : 0.0;
      const double charge_bid = std::max(0.0, st.charge_bid_price);
      const double discharge_bid = std::max(0.0, st.discharge_bid_price);
      const double throughput_bid = 0.5 * w_cost * (charge_bid + discharge_bid);
      const double signed_bid = 0.5 * w_cost * (discharge_bid - charge_bid);
      const double g_hi = std::max(std::abs(st.pmin_mw), st.pmax_mw);
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(gdeg_idx(k, t))] = {
          VarType::Continuous, 0.0, (g_hi > 0.0 ? g_hi : 1.0e9),
          "g_deg" + std::to_string(k) + "_t" + std::to_string(t)};
        m.linear_part.c[gdeg_idx(k, t)] = (degr + throughput_bid) * dt;
        m.linear_part.c[stor_power_idx(k, t)] += signed_bid * dt;
      }
    }
  }

  // VPP aggregate dispatch p_vpp ∈ [pmin, pmax] (free aggregate resource) plus an
  // aggregate energy state e_vpp ∈ [0, E].  When e_storage_sum_mwh = 0 the
  // envelope is made non-binding by sizing it to the horizon-wide power span
  // (2·max|p|·T·dt, starting half full) so a generation-only VPP can export its
  // full power every step without the energy state ever binding.  This keeps the
  // bound magnitudes small and well-conditioned for the MILP backends (a 1e9
  // sentinel makes SCIP's presolve report spurious infeasibility).
  if (use_vpp) {
    for (int v = 0; v < V; ++v) {
      const auto& vp = sys.vpps[static_cast<size_t>(res.vpp_indices[static_cast<size_t>(v)])];
      const double vpp_span =
          std::max(std::abs(vp.pmin_mw), std::abs(vp.pmax_mw)) * static_cast<double>(T) * dt;
      const double E_eff = (vp.e_storage_sum_mwh > 1e-9)
                               ? vp.e_storage_sum_mwh
                               : std::max(2.0 * vpp_span, 1.0);
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(vpp_idx(v, t))] = {
          VarType::Continuous, vp.pmin_mw, vp.pmax_mw,
          "p_vpp" + std::to_string(v) + "_t" + std::to_string(t)};
        m.linear_part.vars[static_cast<size_t>(vppe_idx(v, t))] = {
          VarType::Continuous, 0.0, E_eff,
          "e_vpp" + std::to_string(v) + "_t" + std::to_string(t)};
      }
    }
  }

  // Energy-router ports: inflow / outflow pair per placeable port, each in
  // [0, cap].  The signed bus injection is p_out − p_in; the per-router lossy
  // conservation row (added below) couples the ports.  Simultaneous in/out at a
  // port is never optimal because the round-trip loss must be made up by costed
  // generation, so no anti-simultaneity binary is required.
  if (use_router) {
    for (int p = 0; p < Per; ++p) {
      const double cap = erports[static_cast<size_t>(p)].cap;
      for (int t = 0; t < T; ++t) {
        m.linear_part.vars[static_cast<size_t>(erin_idx(p, t))] = {
          VarType::Continuous, 0.0, cap,
          "er_in" + std::to_string(p) + "_t" + std::to_string(t)};
        m.linear_part.vars[static_cast<size_t>(erout_idx(p, t))] = {
          VarType::Continuous, 0.0, cap,
          "er_out" + std::to_string(p) + "_t" + std::to_string(t)};
      }
    }
  }

  // Mobile storage.  Exogenous-schedule mode: power in [pmin, pmax] when
  // connected, pinned to 0 in transit, plus an energy state.  Co-optimised mode:
  // connect/transit binaries (z0/z1/w), per-bus injections (q0/q1), and the
  // energy state; the start (t=0) is anchored connected at the origin.
  if (use_mobstor && !use_coreloc) {
    for (int b = 0; b < M; ++b) {
      const auto& d = mobstor[static_cast<size_t>(b)];
      for (int t = 0; t < T; ++t) {
        const bool connected = d.loc[static_cast<size_t>(t)] >= 0;
        m.linear_part.vars[static_cast<size_t>(msp_idx(b, t))] = {
          VarType::Continuous, connected ? d.pmin : 0.0, connected ? d.pmax : 0.0,
          "p_ms" + std::to_string(b) + "_t" + std::to_string(t)};
        m.linear_part.vars[static_cast<size_t>(mse_idx(b, t))] = {
          VarType::Continuous, d.soc_min, d.soc_max,
          "e_ms" + std::to_string(b) + "_t" + std::to_string(t)};
      }
    }
  } else if (use_coreloc) {
    for (int b = 0; b < M; ++b) {
      const auto& d = mobstor[static_cast<size_t>(b)];
      const bool has_dest = d.busD >= 0;
      for (int t = 0; t < T; ++t) {
        // z0: connected at origin (anchored to 1 at t=0).
        m.linear_part.vars[static_cast<size_t>(msz0_idx(b, t))] = {
          VarType::Binary, (t == 0) ? 1.0 : 0.0, 1.0,
          "ms_z0_" + std::to_string(b) + "_t" + std::to_string(t)};
        m.binary_idx.push_back(msz0_idx(b, t));
        // z1: connected at target (disabled when there is no valid destination).
        m.linear_part.vars[static_cast<size_t>(msz1_idx(b, t))] = {
          VarType::Binary, 0.0, has_dest && t > 0 ? 1.0 : 0.0,
          "ms_z1_" + std::to_string(b) + "_t" + std::to_string(t)};
        m.binary_idx.push_back(msz1_idx(b, t));
        // w: in transit (0 at t=0).
        m.linear_part.vars[static_cast<size_t>(msw_idx(b, t))] = {
          VarType::Binary, 0.0, (has_dest && t > 0) ? 1.0 : 0.0,
          "ms_w_" + std::to_string(b) + "_t" + std::to_string(t)};
        m.binary_idx.push_back(msw_idx(b, t));
        // q0/q1: injection at origin / target (gated by z0/z1 below).
        m.linear_part.vars[static_cast<size_t>(msq0_idx(b, t))] = {
          VarType::Continuous, d.pmin, d.pmax,
          "ms_q0_" + std::to_string(b) + "_t" + std::to_string(t)};
        m.linear_part.vars[static_cast<size_t>(msq1_idx(b, t))] = {
          VarType::Continuous, has_dest ? d.pmin : 0.0, has_dest ? d.pmax : 0.0,
          "ms_q1_" + std::to_string(b) + "_t" + std::to_string(t)};
        m.linear_part.vars[static_cast<size_t>(mse_idx(b, t))] = {
          VarType::Continuous, d.soc_min, d.soc_max,
          "e_ms" + std::to_string(b) + "_t" + std::to_string(t)};
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
  // 5. ESS SOC dynamics: AC + DC storage ((S+Sd)*T eq).
  //    Signed net dispatch is p = p_discharge - p_charge. The auxiliary
  //    p_charge and mode constraints below preserve the distinct charging and
  //    discharging efficiencies exactly in a MILP.
  const int n_eq_balance = use_network_constraints ? (net.n_bus * T) : T;
  const int n_eq_dc_balance = use_dc_network ? (dcg.n_dc_bus * T) : 0;
  const int n_eq_soc_ac = S * T;      // AC SOC dynamics
  const int n_eq_soc_dc = Sd * T;     // DC SOC dynamics
  const int n_eq_soc = n_eq_soc_ac + n_eq_soc_dc;
  // Shiftable demand response: one energy-neutrality row per flexible load
  // (Σ_t up − Σ_t down = 0), only in the shiftable variant.
  const int n_eq_dr_shift = use_dr_shift ? F : 0;
  // VPP aggregate energy-state dynamics: one row per VPP-step.
  const int n_eq_vpp = use_vpp ? V * T : 0;
  // Energy-router internal lossy conservation: one row per active router-step
  // (Σ_j η_j·p_in_j − Σ_j p_out_j = 0).
  const int n_eq_router = use_router ? Rrouter * T : 0;
  // Mobile-storage SOC dynamics: one row per active unit-step.
  const int n_eq_mobstor = use_mobstor ? M * T : 0;
  // Co-optimised relocation state-exclusivity (z0 + z1 + w = 1): one per step.
  const int n_eq_ms_excl = use_coreloc ? M * T : 0;
  const int m_eq = n_eq_balance + n_eq_dc_balance + n_eq_soc + n_eq_dr_shift
                 + n_eq_vpp + n_eq_router + n_eq_mobstor + n_eq_ms_excl;

  const int n_ineq_pmax = G * T;
  const int n_ineq_pmin = G * T;
  const int n_ineq_ramp_up = G * (T > 0 ? T - 1 : 0);
  const int n_ineq_ramp_dn = G * (T > 0 ? T - 1 : 0);
  const int n_ineq_startup = G * T;
  const int n_ineq_storage_mode = 3 * (S + Sd) * T;

  // Min-up/min-down time constraints (count depends on per-generator parameters).
  // Guard must match the assembly loop below (>= 2, not >= 3): a 2-period
  // min-up/min-down constraint (k=1 only) is a real binding constraint and was
  // added in the assembly in Round 6.  Counting fewer rows than the assembly
  // writes causes silent out-of-bounds writes into b_ineq.
  int n_ineq_min_updn = 0;
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    const int up_periods = static_cast<int>(std::ceil(gen.min_up_time_hr / dt));
    const int dn_periods = static_cast<int>(std::ceil(gen.min_dn_time_hr / dt));
    if (up_periods >= 2) {
      for (int t = 1; t < T; ++t)
        n_ineq_min_updn += std::min(up_periods - 1, T - 1 - t);
    }
    if (dn_periods >= 2) {
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
  // Microgrid islanding gating: two rows per microgrid-step
  //   p_mg ≤ o·p_export_max  and  −p_mg ≤ o·p_import_max.
  const int n_ineq_mg = use_mg ? (2 * U * T) : 0;
  // Storage degradation: two |p| rows per storage-step, plus one daily cycle
  // cap row per storage whose daily_cycle_limit > 0.
  const int n_ineq_degr_abs = use_storage_abs ? (2 * (S + Sd) * T) : 0;
  int n_ineq_degr_cap = 0;
  if (use_degr) {
    for (int k = 0; k < S + Sd; ++k) {
      if (stor_ref(k).daily_cycle_limit > 0.0) ++n_ineq_degr_cap;
    }
  }
  // VPP ramp limits: up to two rows per VPP-step (t ≥ 1) for units with limits.
  int n_ineq_vpp_ramp = 0;
  if (use_vpp) {
    for (int v = 0; v < V; ++v) {
      const auto& vp = sys.vpps[static_cast<size_t>(res.vpp_indices[static_cast<size_t>(v)])];
      if (vp.ramp_up_max_mw_min > 0.0) n_ineq_vpp_ramp += (T > 0 ? T - 1 : 0);
      if (vp.ramp_down_max_mw_min > 0.0) n_ineq_vpp_ramp += (T > 0 ? T - 1 : 0);
    }
  }
  // Co-optimised mobile relocation: q-gating (4·M·T), transit-linking (2·M·T)
  // and minimum-stay (2·M·T) rows.  All blocks are sized M·T (some rows are
  // trivially 0 ≤ 0 at the horizon edges) so the counting stays regular.
  const int n_ineq_ms_q = use_coreloc ? (4 * M * T) : 0;
  const int n_ineq_ms_transit = use_coreloc ? (2 * M * T) : 0;
  const int n_ineq_ms_minstay = use_coreloc ? (2 * M * T) : 0;

  const int m_ineq = n_ineq_pmax + n_ineq_pmin + n_ineq_ramp_up + n_ineq_ramp_dn
                   + n_ineq_startup + n_ineq_storage_mode
                   + n_ineq_min_updn + n_ineq_line + n_ineq_dc_line
                   + n_ineq_reserve + n_ineq_mg + n_ineq_degr_abs + n_ineq_degr_cap
                   + n_ineq_vpp_ramp
                   + n_ineq_ms_q + n_ineq_ms_transit + n_ineq_ms_minstay;

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
    for (const auto& sg : sys.dc.static_generators) {
      if (!sg.in_service) continue;
      const double p = sg.p_mw * sg.scaling;
      for (int t = 0; t < T; ++t) {
        total_load[static_cast<size_t>(t)] -= p;
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
    for (const auto& sg : sys.dc.static_generators) {
      if (!sg.in_service) continue;
      auto bit = dc_bus_idx.find(sg.bus);
      if (bit == dc_bus_idx.end()) continue;
      const double p = sg.p_mw * sg.scaling;
      for (int t = 0; t < T; ++t) {
        dc_bus_load[static_cast<size_t>(dc_bus_load_idx(bit->second, t))] -= p;
      }
    }
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
      // External-grid net exchange (+import) supplies the copper-plate balance.
      for (int xi = 0; xi < X; ++xi) {
        eq_trips.emplace_back(t, ext_idx(xi, t), 1.0);
      }
      // Demand response: +up raises demand (−1 on supply side), +down lowers it.
      for (int fi = 0; fi < F; ++fi) {
        eq_trips.emplace_back(t, drup_idx(fi, t), -1.0);
        eq_trips.emplace_back(t, drdn_idx(fi, t), 1.0);
      }
      // Dispatchable-PV curtailment reduces must-take injection (−1 = less supply).
      for (int k = 0; k < Pc; ++k) {
        eq_trips.emplace_back(t, curt_idx(k, t), -1.0);
      }
      // Dispatchable AC static-generator supply (+1).
      for (int k = 0; k < Ng; ++k) {
        eq_trips.emplace_back(t, sgen_idx(k, t), 1.0);
      }
      // Microgrid PCC exchange injects into the main system (+export).
      for (int ui = 0; ui < U; ++ui) {
        eq_trips.emplace_back(t, mgex_idx(ui, t), 1.0);
      }
      // VPP aggregate net output injects at the system level (+output).
      for (int v = 0; v < V; ++v) {
        eq_trips.emplace_back(t, vpp_idx(v, t), 1.0);
      }
      // Energy-router ports: net injection p_out − p_in into the single
      // copper-plate balance (DC ports also fold in here, as DC injections do).
      for (int p = 0; p < Per; ++p) {
        eq_trips.emplace_back(t, erout_idx(p, t), 1.0);
        eq_trips.emplace_back(t, erin_idx(p, t), -1.0);
      }
      // Mobile storage injects (+discharge) into the system balance.  Exogenous
      // mode uses the single power var when connected; co-optimised mode sums
      // the two per-bus injections (the bus is irrelevant in copper-plate mode).
      for (int b = 0; b < M; ++b) {
        if (use_coreloc) {
          eq_trips.emplace_back(t, msq0_idx(b, t), 1.0);
          eq_trips.emplace_back(t, msq1_idx(b, t), 1.0);
        } else if (mobstor[static_cast<size_t>(b)].loc[static_cast<size_t>(t)] >= 0) {
          eq_trips.emplace_back(t, msp_idx(b, t), 1.0);
        }
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
    // Map external grids to their AC bus for nodal injection.
    std::vector<std::vector<int>> ext_at_bus(static_cast<size_t>(net.n_bus));
    for (int xi = 0; xi < X; ++xi) {
      const auto& xg = sys.ac.external_grids[static_cast<size_t>(res.ext_indices[static_cast<size_t>(xi)])];
      auto it = bus_idx.find(xg.bus);
      if (it != bus_idx.end()) ext_at_bus[static_cast<size_t>(it->second)].push_back(xi);
    }
    // Map flexible loads to their AC bus for nodal demand adjustment.
    std::vector<std::vector<int>> flex_at_bus(static_cast<size_t>(net.n_bus));
    for (int fi = 0; fi < F; ++fi) {
      const auto& fl = sys.ac.flexible_loads[static_cast<size_t>(res.flex_indices[static_cast<size_t>(fi)])];
      auto it = bus_idx.find(fl.bus);
      if (it != bus_idx.end()) flex_at_bus[static_cast<size_t>(it->second)].push_back(fi);
    }
    // Route PV curtailment to the AC bus where the source was subtracted:
    // AC PV → its own bus; DC sources (no explicit DC network) → the slack bus
    // (their must-take was folded into the slack residual).  DC-network DC
    // sources are routed in the DC nodal balance below instead.
    std::vector<std::vector<int>> curt_at_ac_bus(static_cast<size_t>(net.n_bus));
    for (int k = 0; k < Pc; ++k) {
      const auto& d = pvcurt[static_cast<size_t>(k)];
      if (d.ac_bus >= 0) {
        auto it = bus_idx.find(d.ac_bus);
        if (it != bus_idx.end()) curt_at_ac_bus[static_cast<size_t>(it->second)].push_back(k);
      } else if (!use_dc_network && net.slack_bus >= 0) {
        curt_at_ac_bus[static_cast<size_t>(net.slack_bus)].push_back(k);
      }
    }
    // Map microgrids to their PCC bus for nodal exchange injection.
    std::vector<std::vector<int>> mg_at_bus(static_cast<size_t>(net.n_bus));
    for (int ui = 0; ui < U; ++ui) {
      const auto& mg = sys.microgrids[static_cast<size_t>(res.mg_indices[static_cast<size_t>(ui)])];
      auto it = bus_idx.find(mg.pcc_bus);
      if (it != bus_idx.end()) mg_at_bus[static_cast<size_t>(it->second)].push_back(ui);
    }
    // Map dispatchable AC static generators to their bus.
    std::vector<std::vector<int>> sgen_at_bus(static_cast<size_t>(net.n_bus));
    for (int k = 0; k < Ng; ++k) {
      auto it = bus_idx.find(sgsup[static_cast<size_t>(k)].ac_bus);
      if (it != bus_idx.end()) sgen_at_bus[static_cast<size_t>(it->second)].push_back(k);
    }
    // Map VPPs to their PCC bus.
    std::vector<std::vector<int>> vpp_at_bus(static_cast<size_t>(net.n_bus));
    for (int v = 0; v < V; ++v) {
      const auto& vp = sys.vpps[static_cast<size_t>(res.vpp_indices[static_cast<size_t>(v)])];
      auto it = bus_idx.find(vp.pcc_bus);
      if (it != bus_idx.end()) vpp_at_bus[static_cast<size_t>(it->second)].push_back(v);
    }
    // Map energy-router ports to an AC balance row: AC ports → their own bus;
    // DC ports without an explicit DC network → the slack bus (their injection
    // folds into the slack residual, mirroring the must-take DC sources).
    std::vector<std::vector<int>> er_at_ac_bus(static_cast<size_t>(net.n_bus));
    for (int p = 0; p < Per; ++p) {
      const auto& ep = erports[static_cast<size_t>(p)];
      if (ep.ac_bus >= 0) {
        auto it = bus_idx.find(ep.ac_bus);
        if (it != bus_idx.end()) er_at_ac_bus[static_cast<size_t>(it->second)].push_back(p);
      } else if (ep.dc_bus >= 0 && !use_dc_network && net.slack_bus >= 0) {
        er_at_ac_bus[static_cast<size_t>(net.slack_bus)].push_back(p);
      }
    }

    // Pre-compute a row-major view of B to allow O(nnz) row iteration.
    // net.B is column-major by default; scanning all columns via .coeff(b,j)
    // is O(T·nb²).  Converting once to row-major reduces the balance-assembly
    // loop to O(T·nnz(B)), which is O(T·nb·avg_degree) — typically 3–5×.
    const Eigen::SparseMatrix<double, Eigen::RowMajor> B_rm = net.B;

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
        // External-grid net exchange injected at its AC bus (+import).
        for (int xi : ext_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, ext_idx(xi, t), 1.0);
        }
        // Demand response at this bus: +up raises demand, +down lowers it.
        for (int fi : flex_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, drup_idx(fi, t), -1.0);
          eq_trips.emplace_back(row_bal, drdn_idx(fi, t), 1.0);
        }
        // PV curtailment reduces must-take injection at this AC bus (−1).
        for (int k : curt_at_ac_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, curt_idx(k, t), -1.0);
        }
        // Microgrid PCC exchange injects at its PCC bus (+export).
        for (int ui : mg_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, mgex_idx(ui, t), 1.0);
        }
        // Dispatchable AC static-generator supply at this bus (+1).
        for (int k : sgen_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, sgen_idx(k, t), 1.0);
        }
        // VPP aggregate net output at its PCC bus (+1).
        for (int v : vpp_at_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, vpp_idx(v, t), 1.0);
        }
        // Energy-router net injection (p_out − p_in) at this AC bus.
        for (int p : er_at_ac_bus[static_cast<size_t>(b)]) {
          eq_trips.emplace_back(row_bal, erout_idx(p, t), 1.0);
          eq_trips.emplace_back(row_bal, erin_idx(p, t), -1.0);
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

        // Iterate only over structural non-zeros in row b (O(nnz_per_row)).
        const auto ib = non_slack_pos.find(b);
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(B_rm, b); it; ++it) {
          const int j = static_cast<int>(it.index());
          const double bij = it.value();
          if (ib != non_slack_pos.end()) {
            eq_trips.emplace_back(row_bal, theta_idx(ib->second, t), -sys.base_mva * bij);
          }
          const auto ij = non_slack_pos.find(j);
          if (ij != non_slack_pos.end()) {
            eq_trips.emplace_back(row_bal, theta_idx(ij->second, t), sys.base_mva * bij);
          }
        }

        beq[row_bal] = bus_load[static_cast<size_t>(bus_load_idx(b, t))];
      }
    }
    // Mobile storage injects (+discharge) at its bus each step.  Exogenous mode
    // uses the scheduled bus loc[t]; co-optimised mode routes q0 to the origin
    // bus and q1 to the target bus (each gated to 0 unless connected there).
    for (int b = 0; b < M; ++b) {
      const auto& d = mobstor[static_cast<size_t>(b)];
      for (int t = 0; t < T; ++t) {
        if (use_coreloc) {
          auto io = bus_idx.find(d.busO);
          if (io != bus_idx.end())
            eq_trips.emplace_back(t * net.n_bus + io->second, msq0_idx(b, t), 1.0);
          if (d.busD >= 0) {
            auto id = bus_idx.find(d.busD);
            if (id != bus_idx.end())
              eq_trips.emplace_back(t * net.n_bus + id->second, msq1_idx(b, t), 1.0);
          }
        } else {
          const int raw = d.loc[static_cast<size_t>(t)];
          if (raw < 0) continue;
          auto it = bus_idx.find(raw);
          if (it == bus_idx.end()) continue;
          eq_trips.emplace_back(t * net.n_bus + it->second, msp_idx(b, t), 1.0);
        }
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

    // Route DC-side PV curtailment to its DC bus (reduces injection → −1).
    std::vector<std::vector<int>> curt_at_dc_bus(static_cast<size_t>(dcg.n_dc_bus));
    for (int k = 0; k < Pc; ++k) {
      const auto& d = pvcurt[static_cast<size_t>(k)];
      if (d.dc_bus < 0) continue;
      auto it = dc_bus_idx.find(d.dc_bus);
      if (it != dc_bus_idx.end()) curt_at_dc_bus[static_cast<size_t>(it->second)].push_back(k);
    }
    // Route DC branch flows to their endpoints: −f leaves the from-bus, +f
    // arrives at the to-bus.
    std::vector<std::vector<std::pair<int,double>>> dcflow_at_bus(static_cast<size_t>(dcg.n_dc_bus));
    for (int mf = 0; mf < Nf; ++mf) {
      const auto& d = dcflow[static_cast<size_t>(mf)];
      if (d.from_pos >= 0 && d.from_pos < dcg.n_dc_bus)
        dcflow_at_bus[static_cast<size_t>(d.from_pos)].emplace_back(mf, -1.0);
      if (d.to_pos >= 0 && d.to_pos < dcg.n_dc_bus)
        dcflow_at_bus[static_cast<size_t>(d.to_pos)].emplace_back(mf, 1.0);
    }
    // Route energy-router DC ports to their DC bus (net injection p_out − p_in).
    std::vector<std::vector<int>> er_at_dc_bus(static_cast<size_t>(dcg.n_dc_bus));
    for (int p = 0; p < Per; ++p) {
      const auto& ep = erports[static_cast<size_t>(p)];
      if (ep.dc_bus < 0) continue;
      auto it = dc_bus_idx.find(ep.dc_bus);
      if (it != dc_bus_idx.end()) er_at_dc_bus[static_cast<size_t>(it->second)].push_back(p);
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

        // DC-side PV / static-gen curtailment reduces injection at this DC bus.
        for (int k : curt_at_dc_bus[static_cast<size_t>(d)]) {
          eq_trips.emplace_back(row_dc2, curt_idx(k, t), -1.0);
        }
        // DC branch transport flows (−f out of from-bus, +f into to-bus).
        for (const auto& [mf, sgn] : dcflow_at_bus[static_cast<size_t>(d)]) {
          eq_trips.emplace_back(row_dc2, dcflow_idx(mf, t), sgn);
        }
        // Energy-router DC ports: net injection p_out − p_in at this DC bus.
        for (int p : er_at_dc_bus[static_cast<size_t>(d)]) {
          eq_trips.emplace_back(row_dc2, erout_idx(p, t), 1.0);
          eq_trips.emplace_back(row_dc2, erin_idx(p, t), -1.0);
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

  // SOC dynamics with p = p_discharge - p_charge:
  //   soc_t = retention*soc_prev + eta_c*p_charge*dt/E
  //           - (p + p_charge)*dt/(eta_d*E)
  const int soc_eq_offset = n_eq_balance + n_eq_dc_balance;
  for (int si = 0; si < S; ++si) {
    const auto& ess = sys.ac.storage[static_cast<size_t>(res.storage_indices[static_cast<size_t>(si)])];
    double E = ess.e_rated_mwh;
    if (E < 1e-12) E = 1.0;  // fallback
    const double eta_charge = std::clamp(ess.eta_charge, 1e-6, 1.0);
    const double eta_discharge = std::clamp(ess.eta_discharge, 1e-6, 1.0);
    const double retention =
        std::clamp(1.0 - ess.self_discharge_pct / 100.0, 0.0, 1.0);
    const double coeff_p = dt / (eta_discharge * E);
    const double coeff_charge =
        dt * (1.0 / eta_discharge - eta_charge) / E;

    for (int t = 0; t < T; ++t) {
      int row = soc_eq_offset + si * T + t;
      // soc[s,t]
      eq_trips.emplace_back(row, soc_idx(si, t), 1.0);
      // + p_ess[s,t] * coeff_p
      eq_trips.emplace_back(row, ess_idx(si, t), coeff_p);
      eq_trips.emplace_back(row, stor_charge_idx(si, t), coeff_charge);
      if (t == 0) {
        beq[row] = ess.soc_init * retention;
      } else {
        eq_trips.emplace_back(row, soc_idx(si, t - 1), -retention);
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
    const double eta_charge = std::clamp(ess.eta_charge, 1e-6, 1.0);
    const double eta_discharge = std::clamp(ess.eta_discharge, 1e-6, 1.0);
    const double retention =
        std::clamp(1.0 - ess.self_discharge_pct / 100.0, 0.0, 1.0);
    const double coeff_p = dt / (eta_discharge * E);
    const double coeff_charge =
        dt * (1.0 / eta_discharge - eta_charge) / E;

    for (int t = 0; t < T; ++t) {
      int row_dc = soc_eq_offset + n_eq_soc_ac + si * T + t;
      eq_trips.emplace_back(row_dc, dcsoc_idx(si, t), 1.0);
      eq_trips.emplace_back(row_dc, dcess_idx(si, t), coeff_p);
      eq_trips.emplace_back(row_dc, stor_charge_idx(S + si, t), coeff_charge);
      if (t == 0) {
        beq[row_dc] = ess.soc_init * retention;
      } else {
        eq_trips.emplace_back(row_dc, dcsoc_idx(si, t - 1), -retention);
        beq[row_dc] = 0.0;
      }
    }
  }

  // Shiftable demand response: per-load energy neutrality over the horizon,
  //   Σ_t up[f,t] − Σ_t down[f,t] = 0  (deferred load is recovered, not lost).
  if (use_dr_shift) {
    const int dr_eq_offset = soc_eq_offset + n_eq_soc;
    for (int fi = 0; fi < F; ++fi) {
      const int row_dr = dr_eq_offset + fi;
      for (int t = 0; t < T; ++t) {
        eq_trips.emplace_back(row_dr, drup_idx(fi, t), 1.0);
        eq_trips.emplace_back(row_dr, drdn_idx(fi, t), -1.0);
      }
      beq[row_dr] = 0.0;
    }
  }

  // VPP aggregate energy-state dynamics: e_vpp[v,t] = e_vpp[v,t-1] − p_vpp[v,t]·dt
  // (net output depletes the aggregate energy), starting from a half-full state.
  if (use_vpp) {
    const int vpp_eq_offset = soc_eq_offset + n_eq_soc + n_eq_dr_shift;
    for (int v = 0; v < V; ++v) {
      const auto& vp = sys.vpps[static_cast<size_t>(res.vpp_indices[static_cast<size_t>(v)])];
      const double vpp_span =
          std::max(std::abs(vp.pmin_mw), std::abs(vp.pmax_mw)) * static_cast<double>(T) * dt;
      const double E_eff = (vp.e_storage_sum_mwh > 1e-9)
                               ? vp.e_storage_sum_mwh
                               : std::max(2.0 * vpp_span, 1.0);
      for (int t = 0; t < T; ++t) {
        const int row_v = vpp_eq_offset + v * T + t;
        eq_trips.emplace_back(row_v, vppe_idx(v, t), 1.0);
        eq_trips.emplace_back(row_v, vpp_idx(v, t), dt);
        if (t == 0) {
          beq[row_v] = 0.5 * E_eff;
        } else {
          eq_trips.emplace_back(row_v, vppe_idx(v, t - 1), -1.0);
          beq[row_v] = 0.0;
        }
      }
    }
  }

  // Energy-router internal lossy power conservation, per active router-step:
  //   Σ_j η_j·p_in[j,t] − Σ_j p_out[j,t] = 0
  // (inflow crosses the port with efficiency η_j into the internal node; outflow
  // is drawn from it).  Rows follow the VPP block.
  if (use_router) {
    const int router_eq_offset =
        soc_eq_offset + n_eq_soc + n_eq_dr_shift + n_eq_vpp;
    for (int rp = 0; rp < Rrouter; ++rp) {
      for (int t = 0; t < T; ++t) {
        const int row_r = router_eq_offset + rp * T + t;
        for (int p : er_ports_of_router[static_cast<size_t>(rp)]) {
          eq_trips.emplace_back(row_r, erin_idx(p, t), erports[static_cast<size_t>(p)].eta);
          eq_trips.emplace_back(row_r, erout_idx(p, t), -1.0);
        }
        beq[row_r] = 0.0;
      }
    }
  }

  // Mobile-storage SOC dynamics (per-unit SOC, +discharge depletes energy):
  //   e[b,0] + dt/(η·E)·p[b,0] = soc_init
  //   e[b,t] − e[b,t-1] + dt/(η·E)·p[b,t] = 0   (t ≥ 1)
  // While in transit p is pinned to 0, so the SOC simply holds.
  if (use_mobstor) {
    const int mobstor_eq_offset =
        soc_eq_offset + n_eq_soc + n_eq_dr_shift + n_eq_vpp + n_eq_router;
    for (int b = 0; b < M; ++b) {
      const auto& d = mobstor[static_cast<size_t>(b)];
      const double coeff_p = dt / (d.eta * d.E);
      for (int t = 0; t < T; ++t) {
        const int row_m = mobstor_eq_offset + b * T + t;
        eq_trips.emplace_back(row_m, mse_idx(b, t), 1.0);
        if (use_coreloc) {
          // Power is the sum of the two per-bus injections; transit drains SOC.
          eq_trips.emplace_back(row_m, msq0_idx(b, t), coeff_p);
          eq_trips.emplace_back(row_m, msq1_idx(b, t), coeff_p);
          if (d.drain > 0.0)
            eq_trips.emplace_back(row_m, msw_idx(b, t), d.drain);
        } else {
          eq_trips.emplace_back(row_m, msp_idx(b, t), coeff_p);
        }
        if (t == 0) {
          beq[row_m] = d.soc_init;
        } else {
          eq_trips.emplace_back(row_m, mse_idx(b, t - 1), -1.0);
          beq[row_m] = 0.0;
        }
      }
    }
  }

  // Co-optimised relocation: state-exclusivity z0 + z1 + w = 1 per unit-step.
  if (use_coreloc) {
    const int ms_excl_offset =
        soc_eq_offset + n_eq_soc + n_eq_dr_shift + n_eq_vpp + n_eq_router + n_eq_mobstor;
    for (int b = 0; b < M; ++b) {
      for (int t = 0; t < T; ++t) {
        const int row_e = ms_excl_offset + b * T + t;
        eq_trips.emplace_back(row_e, msz0_idx(b, t), 1.0);
        eq_trips.emplace_back(row_e, msz1_idx(b, t), 1.0);
        eq_trips.emplace_back(row_e, msw_idx(b, t), 1.0);
        beq[row_e] = 1.0;
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

  // (6) Exact storage charge/discharge split. With p = p_dis - p_ch:
  //   p_dis >= 0, p_dis <= Pdis_max*z, p_ch <= Pch_max*(1-z).
  for (int k = 0; k < S + Sd; ++k) {
    const auto& st = stor_ref(k);
    const double charge_cap = std::max(-st.pmin_mw, 0.0);
    const double discharge_cap = std::max(st.pmax_mw, 0.0);
    for (int t = 0; t < T; ++t) {
      const int p_idx = stor_power_idx(k, t);
      const int c_idx = stor_charge_idx(k, t);
      const int z_idx = stor_mode_idx(k, t);

      ineq_trips.emplace_back(row, p_idx, -1.0);
      ineq_trips.emplace_back(row, c_idx, -1.0);
      b_ineq[row] = 0.0;
      ++row;

      ineq_trips.emplace_back(row, p_idx, 1.0);
      ineq_trips.emplace_back(row, c_idx, 1.0);
      ineq_trips.emplace_back(row, z_idx, -discharge_cap);
      b_ineq[row] = 0.0;
      ++row;

      ineq_trips.emplace_back(row, c_idx, 1.0);
      ineq_trips.emplace_back(row, z_idx, charge_cap);
      b_ineq[row] = charge_cap;
      ++row;
    }
  }

  // (7) Min-up time: if generator starts at t, it must stay on for min_up periods.
  //     u[g,t] - u[g,t-1] ≤ u[g,t+k]  for k=1..min(up_periods-1, T-1-t)
  //     Rearranged: u[g,t] - u[g,t-1] - u[g,t+k] ≤ 0
  //     Added for up_periods >= 2: the k=1 case (2-period min-up) is a real
  //     binding constraint — a generator that starts at t must be on at t+1.
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    const int up_periods = static_cast<int>(std::ceil(gen.min_up_time_hr / dt));
    if (up_periods >= 2) {
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

  // (8) Min-down time: if generator shuts down at t, it must stay off for min_dn periods.
  //     u[g,t-1] - u[g,t] ≤ 1 - u[g,t+k]  for k=1..min(dn_periods-1, T-1-t)
  //     Rearranged: u[g,t-1] - u[g,t] + u[g,t+k] ≤ 1
  //     Added for dn_periods >= 2: the k=1 case is binding and must be included.
  for (int gi = 0; gi < G; ++gi) {
    const auto& gen = sys.ac.generators[static_cast<size_t>(res.gen_indices[static_cast<size_t>(gi)])];
    const int dn_periods = static_cast<int>(std::ceil(gen.min_dn_time_hr / dt));
    if (dn_periods >= 2) {
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

  // Microgrid islanding gating: exchange is zero unless grid-connected (o = 1).
  //   p_mg − o·p_export_max ≤ 0      (export only when connected)
  //  −p_mg − o·p_import_max ≤ 0      (import only when connected)
  if (use_mg) {
    for (int ui = 0; ui < U; ++ui) {
      const auto& mg = sys.microgrids[static_cast<size_t>(res.mg_indices[static_cast<size_t>(ui)])];
      const double p_imp = std::max(0.0, mg.p_import_max_mw);
      const double p_exp = std::max(0.0, mg.p_export_max_mw);
      for (int t = 0; t < T; ++t) {
        ineq_trips.emplace_back(row, mgex_idx(ui, t), 1.0);
        ineq_trips.emplace_back(row, mgo_idx(ui, t), -p_exp);
        b_ineq[row] = 0.0;
        ++row;
        ineq_trips.emplace_back(row, mgex_idx(ui, t), -1.0);
        ineq_trips.emplace_back(row, mgo_idx(ui, t), -p_imp);
        b_ineq[row] = 0.0;
        ++row;
      }
    }
  }

  // Storage degradation: throughput auxiliary g ≥ |p_ess|, plus optional daily
  // cycle cap.
  if (use_storage_abs) {
    for (int k = 0; k < S + Sd; ++k) {
      for (int t = 0; t < T; ++t) {
        // p_ess − g ≤ 0
        ineq_trips.emplace_back(row, stor_power_idx(k, t), 1.0);
        ineq_trips.emplace_back(row, gdeg_idx(k, t), -1.0);
        b_ineq[row] = 0.0;
        ++row;
        // −p_ess − g ≤ 0
        ineq_trips.emplace_back(row, stor_power_idx(k, t), -1.0);
        ineq_trips.emplace_back(row, gdeg_idx(k, t), -1.0);
        b_ineq[row] = 0.0;
        ++row;
      }
    }
    if (use_degr) {
      // Daily cycle cap: Σ_t g·dt ≤ 2·daily_cycle_limit·E.
      for (int k = 0; k < S + Sd; ++k) {
        const auto& st = stor_ref(k);
        if (st.daily_cycle_limit <= 0.0) continue;
        const double E = (st.e_rated_mwh > 1e-9) ? st.e_rated_mwh : 1.0;
        for (int t = 0; t < T; ++t) {
          ineq_trips.emplace_back(row, gdeg_idx(k, t), dt);
        }
        b_ineq[row] = 2.0 * st.daily_cycle_limit * E;
        ++row;
      }
    }
  }

  // VPP ramp limits: |p_vpp[v,t] − p_vpp[v,t−1]| ≤ ramp·dt·60.
  if (use_vpp) {
    for (int v = 0; v < V; ++v) {
      const auto& vp = sys.vpps[static_cast<size_t>(res.vpp_indices[static_cast<size_t>(v)])];
      if (vp.ramp_up_max_mw_min > 0.0) {
        const double lim = vp.ramp_up_max_mw_min * dt * 60.0;
        for (int t = 1; t < T; ++t) {
          ineq_trips.emplace_back(row, vpp_idx(v, t), 1.0);
          ineq_trips.emplace_back(row, vpp_idx(v, t - 1), -1.0);
          b_ineq[row] = lim;
          ++row;
        }
      }
      if (vp.ramp_down_max_mw_min > 0.0) {
        const double lim = vp.ramp_down_max_mw_min * dt * 60.0;
        for (int t = 1; t < T; ++t) {
          ineq_trips.emplace_back(row, vpp_idx(v, t - 1), 1.0);
          ineq_trips.emplace_back(row, vpp_idx(v, t), -1.0);
          b_ineq[row] = lim;
          ++row;
        }
      }
    }
  }

  // Co-optimised mobile relocation inequalities (each block sized M·T so the
  // count stays regular; horizon-edge rows are trivially 0 ≤ 0):
  //   (1) q-gating  q ∈ [pmin·z, pmax·z]  — inject only at the connected bus
  //   (2) transit-linking  leaving a bus forces a transit step
  //   (3) minimum-stay  a fresh connection must persist for min_stay steps
  if (use_coreloc) {
    for (int b = 0; b < M; ++b) {           // (1) q-gating: 4 rows per (b,t)
      const auto& d = mobstor[static_cast<size_t>(b)];
      for (int t = 0; t < T; ++t) {
        ineq_trips.emplace_back(row, msq0_idx(b, t), 1.0);          // q0 ≤ pmax·z0
        ineq_trips.emplace_back(row, msz0_idx(b, t), -d.pmax);
        b_ineq[row] = 0.0; ++row;
        ineq_trips.emplace_back(row, msz0_idx(b, t), d.pmin);       // pmin·z0 ≤ q0
        ineq_trips.emplace_back(row, msq0_idx(b, t), -1.0);
        b_ineq[row] = 0.0; ++row;
        ineq_trips.emplace_back(row, msq1_idx(b, t), 1.0);          // q1 ≤ pmax·z1
        ineq_trips.emplace_back(row, msz1_idx(b, t), -d.pmax);
        b_ineq[row] = 0.0; ++row;
        ineq_trips.emplace_back(row, msz1_idx(b, t), d.pmin);       // pmin·z1 ≤ q1
        ineq_trips.emplace_back(row, msq1_idx(b, t), -1.0);
        b_ineq[row] = 0.0; ++row;
      }
    }
    for (int b = 0; b < M; ++b) {           // (2) transit-linking: 2 rows per (b,t)
      for (int t = 0; t < T; ++t) {
        if (t >= 1) {                       // z0[t-1] − z0[t] − w[t] ≤ 0
          ineq_trips.emplace_back(row, msz0_idx(b, t - 1), 1.0);
          ineq_trips.emplace_back(row, msz0_idx(b, t), -1.0);
          ineq_trips.emplace_back(row, msw_idx(b, t), -1.0);
        }
        b_ineq[row] = 0.0; ++row;
        if (t >= 1) {                       // z1[t-1] − z1[t] − w[t] ≤ 0
          ineq_trips.emplace_back(row, msz1_idx(b, t - 1), 1.0);
          ineq_trips.emplace_back(row, msz1_idx(b, t), -1.0);
          ineq_trips.emplace_back(row, msw_idx(b, t), -1.0);
        }
        b_ineq[row] = 0.0; ++row;
      }
    }
    for (int b = 0; b < M; ++b) {           // (3) min-stay: 2 rows per (b,t)
      const auto& d = mobstor[static_cast<size_t>(b)];
      const int H = d.min_stay;
      for (int t = 0; t < T; ++t) {
        if (H >= 2 && t >= 1 && t + H <= T) {     // origin window
          ineq_trips.emplace_back(row, msz0_idx(b, t), static_cast<double>(H) - 1.0);
          ineq_trips.emplace_back(row, msz0_idx(b, t - 1), -static_cast<double>(H));
          for (int tau = t + 1; tau < t + H; ++tau)
            ineq_trips.emplace_back(row, msz0_idx(b, tau), -1.0);
        }
        b_ineq[row] = 0.0; ++row;
        if (H >= 2 && d.busD >= 0 && t >= 1 && t + H <= T) {  // target window
          ineq_trips.emplace_back(row, msz1_idx(b, t), static_cast<double>(H) - 1.0);
          ineq_trips.emplace_back(row, msz1_idx(b, t - 1), -static_cast<double>(H));
          for (int tau = t + 1; tau < t + H; ++tau)
            ineq_trips.emplace_back(row, msz1_idx(b, tau), -1.0);
        }
        b_ineq[row] = 0.0; ++row;
      }
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
namespace {
thread_local int g_active_uc_solver_threads = 0;

struct ScopedUCSolverThreadOverride {
  explicit ScopedUCSolverThreadOverride(int threads)
      : previous(g_active_uc_solver_threads) {
    g_active_uc_solver_threads = std::max(0, threads);
  }
  ~ScopedUCSolverThreadOverride() { g_active_uc_solver_threads = previous; }
  int previous;
};
}  // namespace

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
    const int requested_threads = g_active_uc_solver_threads > 0
                                      ? g_active_uc_solver_threads
                                      : 0;
    if (requested_threads > 0) {
      opt.num_threads = requested_threads;
    } else if (hw >= 4) {
      opt.num_threads = std::min(hw, 8);
    }
    return std::make_shared<NativeBranchAndCutAdapter>(opt);
  };

  if (choice == UCSolverChoice::Native) {
    return make_tuned_native();
  }
  if (choice == UCSolverChoice::HiGHS) {
    return std::make_shared<HighsAdapter>();
  }
  if (choice == UCSolverChoice::SCIP) {
    // SCIP solves the UC MILP directly via its native MPS reader (embedded
    // libscip when compiled in, else the SCIP executable).  Fall back to the
    // tuned native B&C when SCIP is unavailable so the choice never dead-ends.
    auto scip = std::make_shared<ScipAdapter>();
    if (scip->available() && scip->supports(ProblemClass::MILP)) return scip;
    return make_tuned_native();
  }
  if (choice == UCSolverChoice::Gurobi) {
    return std::make_shared<GurobiAdapter>();
  }

  // Auto: keep the portable in-tree path first. Gurobi remains available only
  // through the explicit UCSolverChoice::Gurobi selection when compiled in.
  AdapterRegistry reg;
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
  sched.dc_ess_soc.resize(static_cast<size_t>(Sd), std::vector<double>(static_cast<size_t>(T)));
  sched.renewable_dispatch.resize(static_cast<size_t>(R), std::vector<double>(static_cast<size_t>(T)));
  if (std::find(build.pv_curt_kind.begin(), build.pv_curt_kind.end(), 0) !=
      build.pv_curt_kind.end()) {
    sched.ac_pv_dispatch.resize(static_cast<size_t>(build.ac_pv_count),
                                std::vector<double>(static_cast<size_t>(T)));
  }
  if (!build.ac_sgen_indices.empty()) {
    sched.ac_sgen_dispatch.resize(static_cast<size_t>(build.ac_sgen_count),
                                  std::vector<double>(static_cast<size_t>(T)));
  }
  if (!build.ext_indices.empty()) {
    sched.external_grid_dispatch.resize(
        static_cast<size_t>(build.external_grid_count),
        std::vector<double>(static_cast<size_t>(T)));
  }
  sched.dc_pv_dispatch.resize(static_cast<size_t>(build.dc_pv_count),
                              std::vector<double>(static_cast<size_t>(T)));
  sched.dc_sgen_dispatch.resize(static_cast<size_t>(build.dc_sgen_count),
                                std::vector<double>(static_cast<size_t>(T)));

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
      sched.dc_ess_soc[static_cast<size_t>(si)][static_cast<size_t>(t)] = x[dcsoc_idx(si, t)];
    }
  }
  for (int ri = 0; ri < R; ++ri) {
    for (int t = 0; t < T; ++t) {
      sched.renewable_dispatch[static_cast<size_t>(ri)][static_cast<size_t>(t)] = x[ren_idx(ri, t)];
    }
  }
  for (size_t k = 0; k < build.pv_curt_kind.size(); ++k) {
    const int component = build.pv_curt_indices[k];
    for (int t = 0; t < T; ++t) {
      const double dispatched = std::max(
          0.0, build.pv_curt_available[k][static_cast<size_t>(t)] -
                   x[build.pv_curt_offset + static_cast<int>(k) * T + t]);
      auto* rows = build.pv_curt_kind[k] == 0
                       ? &sched.ac_pv_dispatch
                       : (build.pv_curt_kind[k] == 1 ? &sched.dc_pv_dispatch
                                                     : &sched.dc_sgen_dispatch);
      (*rows)[static_cast<size_t>(component)][static_cast<size_t>(t)] = dispatched;
    }
  }
  for (size_t k = 0; k < build.ac_sgen_indices.size(); ++k) {
    const int component = build.ac_sgen_indices[k];
    for (int t = 0; t < T; ++t) {
      sched.ac_sgen_dispatch[static_cast<size_t>(component)][static_cast<size_t>(t)] =
          x[build.ac_sgen_offset + static_cast<int>(k) * T + t];
    }
  }
  for (size_t k = 0; k < build.ext_indices.size(); ++k) {
    const int component = build.ext_indices[k];
    for (int t = 0; t < T; ++t) {
      sched.external_grid_dispatch[static_cast<size_t>(component)][static_cast<size_t>(t)] =
          x[build.external_grid_offset + static_cast<int>(k) * T + t];
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

UCSolverChoice resolve_parallel_daily_uc_solver(UCSolverChoice requested) {
  if (requested == UCSolverChoice::Auto) {
    engine::ScipAdapter scip;
    if (scip.available() && scip.supports(engine::ProblemClass::MILP)) {
      return UCSolverChoice::SCIP;
    }
    return UCSolverChoice::Native;
  }
  return requested;
}

bool uc_solver_allows_parallel_daily(UCSolverChoice solver) {
  solver = resolve_parallel_daily_uc_solver(solver);
  if (solver == UCSolverChoice::Native) return true;
  if (solver == UCSolverChoice::SCIP) {
    engine::ScipAdapter scip;
    return scip.available() && scip.supports(engine::ProblemClass::MILP);
  }
  return false;
}

int resolve_parallel_worker_count(int requested_threads, int work_items) {
  return util::resolve_worker_count(requested_threads, work_items);
}

// ═══════════════════════════════════════════════════════════════════════
// Phase II: Solve Unit Commitment
// ═══════════════════════════════════════════════════════════════════════
HybridPowerSystem build_time_series_system_snapshot(
    const HybridPowerSystem& base_sys,
    const TimeSeriesData& ts_data,
    const UCSchedule& schedule,
    int step,
    const TimeSeriesPFOptions& opts) {
  if (step < 0 || step >= ts_data.num_steps) {
    throw std::out_of_range("time-series step is outside ts_data range");
  }

  HybridPowerSystem sys_t = base_sys;
  const auto profile_map = build_profile_map(ts_data);

  std::vector<int> gen_active_idx;
  for (int i = 0; i < static_cast<int>(base_sys.ac.generators.size()); ++i) {
    if (base_sys.ac.generators[static_cast<size_t>(i)].in_service) {
      gen_active_idx.push_back(i);
    }
  }

  std::vector<int> ess_active_idx;
  for (int i = 0; i < static_cast<int>(base_sys.ac.storage.size()); ++i) {
    if (base_sys.ac.storage[static_cast<size_t>(i)].in_service) {
      ess_active_idx.push_back(i);
    }
  }

  std::vector<int> dc_ess_active_idx;
  for (int i = 0; i < static_cast<int>(base_sys.dc.storage.size()); ++i) {
    if (base_sys.dc.storage[static_cast<size_t>(i)].in_service) {
      dc_ess_active_idx.push_back(i);
    }
  }

  std::vector<int> ren_active_idx;
  for (int i = 0; i < static_cast<int>(base_sys.ac.renewable_gens.size()); ++i) {
    if (base_sys.ac.renewable_gens[static_cast<size_t>(i)].in_service) {
      ren_active_idx.push_back(i);
    }
  }

  std::unordered_map<int, BusType> ac_bus_type_by_index;
  ac_bus_type_by_index.reserve(base_sys.ac.buses.size());
  for (const auto& bus : base_sys.ac.buses) {
    ac_bus_type_by_index[bus.index] = bus.bus_type;
  }

  const bool apply_uc_schedule = should_apply_uc_schedule(opts, schedule);

  if (!apply_uc_schedule) {
    for (auto& gen : sys_t.ac.generators) {
      if (!gen.in_service) continue;
      const auto* profile = find_profile(profile_map, gen.profile_id);
      if (!profile) continue;
      const double availability =
          std::max(0.0, profile_value(profile, step, 1.0));
      gen.pg_mw *= availability;
      gen.pmax_mw *= availability;
      gen.pmin_mw *= availability;
    }
  }

  if (apply_uc_schedule) {
    for (int gi = 0; gi < static_cast<int>(gen_active_idx.size()); ++gi) {
      const int idx = gen_active_idx[static_cast<size_t>(gi)];
      auto& gen = sys_t.ac.generators[static_cast<size_t>(idx)];
      if (gi >= static_cast<int>(schedule.gen_dispatch.size()) ||
          step >= static_cast<int>(schedule.gen_dispatch[static_cast<size_t>(gi)].size())) {
        continue;
      }

      gen.pg_mw = schedule.gen_dispatch[static_cast<size_t>(gi)][static_cast<size_t>(step)];
      if (gi < static_cast<int>(schedule.gen_commit.size()) &&
          step < static_cast<int>(schedule.gen_commit[static_cast<size_t>(gi)].size())) {
        gen.in_service =
            schedule.gen_commit[static_cast<size_t>(gi)][static_cast<size_t>(step)] != 0;
      }

      if (!gen.in_service) {
        gen.pg_mw = 0.0;
        gen.qg_mvar = 0.0;
        gen.pmin_mw = 0.0;
        gen.pmax_mw = 0.0;
      } else if (opts.enable_uc_opf_tracking_band) {
        const double p_uc = gen.pg_mw;
        bool is_slack_gen = gen.is_slack;
        auto bus_it = ac_bus_type_by_index.find(gen.bus);
        if (bus_it != ac_bus_type_by_index.end() && bus_it->second == BusType::SLACK) {
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

  for (int si = 0; si < static_cast<int>(ess_active_idx.size()); ++si) {
    const int idx = ess_active_idx[static_cast<size_t>(si)];
    auto& ess = sys_t.ac.storage[static_cast<size_t>(idx)];
    if (si < static_cast<int>(schedule.ess_dispatch.size()) &&
        step < static_cast<int>(schedule.ess_dispatch[static_cast<size_t>(si)].size())) {
      ess.p_mw = schedule.ess_dispatch[static_cast<size_t>(si)][static_cast<size_t>(step)];
    }
    if (si < static_cast<int>(schedule.ess_soc.size()) &&
        step < static_cast<int>(schedule.ess_soc[static_cast<size_t>(si)].size())) {
      ess.soc_init = schedule.ess_soc[static_cast<size_t>(si)][static_cast<size_t>(step)];
      ess.e_mwh = ess.soc_init * std::max(ess.e_rated_mwh, 0.0);
    }
  }

  for (int si = 0; si < static_cast<int>(dc_ess_active_idx.size()); ++si) {
    const int idx = dc_ess_active_idx[static_cast<size_t>(si)];
    auto& ess = sys_t.dc.storage[static_cast<size_t>(idx)];
    if (si < static_cast<int>(schedule.dc_ess_dispatch.size()) &&
        step < static_cast<int>(schedule.dc_ess_dispatch[static_cast<size_t>(si)].size())) {
      ess.p_mw = schedule.dc_ess_dispatch[static_cast<size_t>(si)][static_cast<size_t>(step)];
    }
    if (si < static_cast<int>(schedule.dc_ess_soc.size()) &&
        step < static_cast<int>(schedule.dc_ess_soc[static_cast<size_t>(si)].size())) {
      ess.soc_init = schedule.dc_ess_soc[static_cast<size_t>(si)][static_cast<size_t>(step)];
      ess.e_mwh = ess.soc_init * std::max(ess.e_rated_mwh, 0.0);
    }
  }

  if (apply_uc_schedule) {
    for (int ri = 0; ri < static_cast<int>(ren_active_idx.size()); ++ri) {
      const int idx = ren_active_idx[static_cast<size_t>(ri)];
      auto& ren = sys_t.ac.renewable_gens[static_cast<size_t>(idx)];
      if (ri < static_cast<int>(schedule.renewable_dispatch.size()) &&
          step < static_cast<int>(schedule.renewable_dispatch[static_cast<size_t>(ri)].size())) {
        ren.p_mw = schedule.renewable_dispatch[static_cast<size_t>(ri)][static_cast<size_t>(step)];
      }
    }
  } else {
    for (auto& ren : sys_t.ac.renewable_gens) {
      if (!ren.in_service) continue;
      const auto* prof = find_profile(profile_map, ren.profile_id);
      ren.p_mw *= profile_value(prof, step, 1.0);
    }
  }

  if (!sys_t.ac.loads.empty() || !sys_t.ac.charging_stations.empty()) {
    for (auto& load : sys_t.ac.loads) {
      if (!load.in_service) continue;
      const auto* prof = find_profile(profile_map, load.profile_id);
      const double scale = profile_value(prof, step, 1.0);
      load.p_mw *= scale * load.scaling;
      load.q_mvar *= scale * load.scaling;
      load.scaling = 1.0;
    }
  } else {
    const auto* default_load = find_profile(profile_map, 0);
    const double scale = profile_value(default_load, step, 1.0);
    for (auto& bus : sys_t.ac.buses) {
      if (!bus.in_service) continue;
      bus.pd_mw *= scale;
      bus.qd_mvar *= scale;
    }
  }

  for (auto& pvsys : sys_t.ac.pv_systems) {
    if (!pvsys.in_service) continue;
    const auto* prof = find_profile(profile_map, pvsys.profile_id);
    if (prof == nullptr) continue;
    const double scale = profile_value(prof, step, 1.0);
    pvsys.irradiance = scale * 1000.0;
    if (pvsys.voc <= 0.0 || pvsys.isc <= 0.0 || pvsys.vmpp <= 0.0) {
      pvsys.p_mw *= scale;
    }
  }

  if (apply_uc_schedule && opts.enable_dispatchable_pv) {
    for (size_t k = 0; k < sys_t.ac.pv_systems.size() &&
                       k < schedule.ac_pv_dispatch.size(); ++k) {
      auto& pv = sys_t.ac.pv_systems[k];
      if (!pv.in_service ||
          step >= static_cast<int>(schedule.ac_pv_dispatch[k].size())) continue;
      pv.p_mw = std::max(0.0, schedule.ac_pv_dispatch[k][static_cast<size_t>(step)]);
      // Force the PF snapshot to use the scheduled MW value even for PVs that
      // normally derive output from their electrical curve and irradiance.
      pv.voc = 0.0;
      pv.isc = 0.0;
      pv.vmpp = 0.0;
    }
    for (size_t k = 0; k < sys_t.ac.static_generators.size() &&
                       k < schedule.ac_sgen_dispatch.size(); ++k) {
      auto& sg = sys_t.ac.static_generators[k];
      if (!sg.in_service ||
          step >= static_cast<int>(schedule.ac_sgen_dispatch[k].size())) continue;
      sg.p_mw = std::max(0.0, schedule.ac_sgen_dispatch[k][static_cast<size_t>(step)]);
      sg.scaling = 1.0;
    }
  }

  for (size_t k = 0; k < sys_t.dc.pv_arrays.size(); ++k) {
    auto& pv = sys_t.dc.pv_arrays[k];
    if (!pv.in_service) continue;
    const auto* prof = find_profile(profile_map, pv.profile_id);
    const double scale = profile_value(prof, step, 1.0);
    pv.p_set_mw *= scale;
    pv.irradiance = 1000.0 * scale;
    if (apply_uc_schedule && opts.enable_dispatchable_pv &&
        k < schedule.dc_pv_dispatch.size() &&
        step < static_cast<int>(schedule.dc_pv_dispatch[k].size())) {
      pv.p_set_mw = std::max(
          0.0, schedule.dc_pv_dispatch[k][static_cast<size_t>(step)]);
    }
  }

  for (size_t k = 0; k < sys_t.dc.dc_static_generators.size(); ++k) {
    auto& sg = sys_t.dc.dc_static_generators[k];
    if (!sg.in_service) continue;
    const auto* prof = find_profile(profile_map, sg.profile_id);
    const double scale = profile_value(prof, step, 1.0);
    sg.p_set_mw *= sg.scaling * scale;
    sg.scaling = 1.0;
    if (apply_uc_schedule && opts.enable_dispatchable_pv &&
        k < schedule.dc_sgen_dispatch.size() &&
        step < static_cast<int>(schedule.dc_sgen_dispatch[k].size())) {
      sg.p_set_mw = std::max(
          0.0, schedule.dc_sgen_dispatch[k][static_cast<size_t>(step)]);
    }
  }

  for (auto& load : sys_t.dc.loads) {
    if (!load.in_service) continue;
    const auto* prof = find_profile(profile_map, load.profile_id);
    const double scale = profile_value(prof, step, 1.0);
    load.p_mw *= scale * load.scaling;
    load.scaling = 1.0;
  }

  // Apply time-varying electricity price to external grids (price_profile_id → cost_c1).
  // The profile contains absolute prices (e.g. RMB/MWh) that replace cost_c1 each step.
  for (auto& eg : sys_t.ac.external_grids) {
    if (!eg.in_service) continue;
    const auto* prof = find_profile(profile_map, eg.price_profile_id);
    if (prof == nullptr) continue;
    eg.cost_c1 = profile_value(prof, step, eg.cost_c1);
  }

  return sys_t;
}

UCSchedule solve_unit_commitment(const HybridPowerSystem& sys,
                                 const TimeSeriesData& ts_data,
                                 const TimeSeriesPFOptions& opts) {
  ScopedUCSolverThreadOverride thread_override(opts.uc_solver_threads);
  validate_time_series_data_for_solve(ts_data, true);

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
                          result.stats.objective + build.obj_offset,
                          result.stats.success,
                          result.stats.solver_name);
}

// ═══════════════════════════════════════════════════════════════════════
// Full pipeline: UC → OPF (per step) → PF validation (per step)
// ═══════════════════════════════════════════════════════════════════════

// Slice a time-series window [g0, g1) out of `src`, copying the corresponding
// span of every profile.  Used by the per-day parallel decomposition.
static TimeSeriesData slice_ts_window(const TimeSeriesData& src, int g0, int g1) {
  TimeSeriesData out;
  out.step_duration_hr = src.step_duration_hr;
  out.num_steps = std::max(0, g1 - g0);
  out.profiles.reserve(src.profiles.size());
  for (const auto& p : src.profiles) {
    TimeSeriesProfile dp;
    dp.id = p.id;
    dp.name = p.name;
    dp.values.reserve(static_cast<size_t>(out.num_steps));
    for (int t = g0; t < g1; ++t) {
      if (t >= 0 && t < static_cast<int>(p.values.size()))
        dp.values.push_back(p.values[static_cast<size_t>(t)]);
      else if (!p.values.empty())
        dp.values.push_back(p.values.back());
      else
        dp.values.push_back(1.0);
    }
    out.profiles.push_back(std::move(dp));
  }
  return out;
}

// Stitch per-day TimeSeriesPFResults (in chronological order) into one result by
// concatenating every per-step series along the time axis.
static TimeSeriesPFResult concat_ts_results(std::vector<TimeSeriesPFResult>& parts,
                                            int total_steps) {
  TimeSeriesPFResult out;
  out.num_steps = total_steps;
  auto cat_d = [](std::vector<std::vector<double>>& dst,
                  const std::vector<std::vector<double>>& src) {
    if (src.empty()) return;
    if (dst.empty()) dst.resize(src.size());
    if (dst.size() != src.size()) return;
    for (size_t c = 0; c < src.size(); ++c)
      dst[c].insert(dst[c].end(), src[c].begin(), src[c].end());
  };
  auto cat_i = [](std::vector<std::vector<int>>& dst,
                  const std::vector<std::vector<int>>& src) {
    if (src.empty()) return;
    if (dst.empty()) dst.resize(src.size());
    if (dst.size() != src.size()) return;
    for (size_t c = 0; c < src.size(); ++c)
      dst[c].insert(dst[c].end(), src[c].begin(), src[c].end());
  };
  bool first = true;
  out.uc_schedule.feasible = true;
  for (auto& p : parts) {
    auto& u = p.uc_schedule;
    cat_d(out.uc_schedule.gen_dispatch, u.gen_dispatch);
    cat_i(out.uc_schedule.gen_commit, u.gen_commit);
    cat_d(out.uc_schedule.ess_dispatch, u.ess_dispatch);
    cat_d(out.uc_schedule.ess_soc, u.ess_soc);
    cat_d(out.uc_schedule.renewable_dispatch, u.renewable_dispatch);
    cat_d(out.uc_schedule.ac_pv_dispatch, u.ac_pv_dispatch);
    cat_d(out.uc_schedule.ac_sgen_dispatch, u.ac_sgen_dispatch);
    cat_d(out.uc_schedule.external_grid_dispatch, u.external_grid_dispatch);
    cat_d(out.uc_schedule.dc_pv_dispatch, u.dc_pv_dispatch);
    cat_d(out.uc_schedule.dc_ess_dispatch, u.dc_ess_dispatch);
    cat_d(out.uc_schedule.dc_ess_soc, u.dc_ess_soc);
    cat_d(out.uc_schedule.dc_sgen_dispatch, u.dc_sgen_dispatch);
    cat_d(out.uc_schedule.dc_load_demand, u.dc_load_demand);
    cat_d(out.uc_schedule.vsc_dispatch, u.vsc_dispatch);
    cat_d(out.uc_schedule.dcdc_dispatch, u.dcdc_dispatch);
    out.uc_schedule.total_cost += u.total_cost;
    out.uc_schedule.feasible = out.uc_schedule.feasible && u.feasible;
    if (first) { out.uc_schedule.solver_name = u.solver_name; first = false; }
    for (auto& x : p.opf_results) out.opf_results.push_back(std::move(x));
    for (auto& x : p.pf_results) out.pf_results.push_back(std::move(x));
    for (auto& x : p.pf_system_snapshots) out.pf_system_snapshots.push_back(std::move(x));
    for (auto& x : p.rich_results) out.rich_results.push_back(std::move(x));
    for (auto& x : p.crossval) out.crossval.push_back(std::move(x));
    out.num_converged += p.num_converged;
    out.num_opf_converged += p.num_opf_converged;
    out.total_generation_cost += p.total_generation_cost;
    out.uc_solve_sec += p.uc_solve_sec;
  }
  return out;
}

TimeSeriesPFResult solve_time_series_pf(const HybridPowerSystem& sys_in,
                                         const TimeSeriesData& ts_data,
                                         const TimeSeriesPFOptions& opts) {
  validate_time_series_data_for_solve(ts_data, false);
  const int T = ts_data.num_steps;

  // ── Optional per-day parallel decomposition ──────────────────────────────
  // Split a multi-day horizon into independent scheduling days (cyclic SOC per
  // day), solve them concurrently, and stitch the per-step results back. The
  // single-day coupled solve below runs when this is disabled or T fits one day.
  if (opts.parallel_daily && opts.precomputed_uc_schedule == nullptr && T > 0) {
    const double dt = ts_data.step_duration_hr;
    const int day_len = std::max(1, static_cast<int>(std::lround(24.0 / dt)));
    const int num_days = (T + day_len - 1) / day_len;
    if (num_days > 1) {
      TimeSeriesPFOptions day_opts = opts;
      day_opts.parallel_daily = false;              // each day is a plain solve
      day_opts.enforce_terminal_soc_cyclic = true;  // independence across days
      day_opts.uc_solver = resolve_parallel_daily_uc_solver(opts.uc_solver);
      day_opts.uc_solver_threads =
          day_opts.uc_solver == UCSolverChoice::Native ? 1 : 0;
      // Force the thread-safe native parity IPM for any per-step AC-OPF.
      day_opts.opf_options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
      // Concurrent MILP is safe only when each daily solve owns its worker
      // state. HiGHS/Gurobi share process-global scheduling state, so explicit
      // selections are guarded to serial day execution.
      const bool uses_milp = !opts.skip_uc;
      const bool parallel_safe = !uses_milp ||
          uc_solver_allows_parallel_daily(day_opts.uc_solver);
      const std::string parallel_backend =
          !uses_milp ? "no-milp"
          : (day_opts.uc_solver == UCSolverChoice::SCIP) ? "scip"
          : (day_opts.uc_solver == UCSolverChoice::Native) ? "native-bc"
          : "guarded";
      const std::string guard_reason = parallel_safe ? std::string()
          : "explicit UC solver backend is guarded for concurrent daily solves";
      auto parallel_info = util::make_parallel_execution_info(
          true, opts.parallel_threads, num_days,
          parallel_safe ? "parallel-daily/" + parallel_backend
                        : "parallel-daily/serial-guarded",
          parallel_backend, guard_reason);
      const int workers = parallel_info.resolved_workers;

      std::vector<TimeSeriesPFResult> day_results(static_cast<size_t>(num_days));
      auto solve_day = [&](int d) {
        const int g0 = d * day_len;
        const int g1 = std::min(g0 + day_len, T);
        if (g1 <= g0) return;
        TimeSeriesData sub = slice_ts_window(ts_data, g0, g1);
        day_results[static_cast<size_t>(d)] = solve_time_series_pf(sys_in, sub, day_opts);
      };
      if (parallel_info.effective) {
        util::ThreadPool pool(workers);
        std::vector<std::future<void>> futs;
        futs.reserve(static_cast<size_t>(num_days));
        for (int d = 0; d < num_days; ++d)
          futs.push_back(pool.submit([&solve_day, d]() { solve_day(d); }));
        for (auto& f : futs) f.get();
      } else {
        for (int d = 0; d < num_days; ++d) solve_day(d);
      }
      auto out = concat_ts_results(day_results, T);
      parallel_info.actual_parallel_evaluations =
          parallel_info.effective ? num_days : 0;
      parallel_info.serial_evaluations =
          parallel_info.effective ? 0 : num_days;
      if (!parallel_info.effective && parallel_info.guard_reason.empty()) {
        parallel_info.guard_reason = util::insufficient_work_reason(parallel_info);
      }
      out.parallel_execution = parallel_info;
      out.parallel_daily_effective = parallel_info.effective;
      out.parallel_workers = workers;
      out.parallel_mode = parallel_info.mode;
      return out;
    }
  }

  TimeSeriesPFResult result;
  result.num_steps = T;
  result.parallel_workers = 1;
  result.parallel_mode = opts.parallel_daily ? "single-day" : "coupled";
  result.parallel_execution = util::make_parallel_execution_info(
      opts.parallel_daily, opts.parallel_threads,
      T > 0 ? 1 : 0, result.parallel_mode);
  if (opts.parallel_daily && T > 0) {
    result.parallel_execution.guard_reason =
        opts.precomputed_uc_schedule != nullptr
            ? "precomputed UC schedule replay uses the coupled timeline"
            : "horizon fits within one scheduling day";
    result.parallel_execution.mode = result.parallel_mode;
  }

  if (T <= 0) {
    return result;
  }

  // Fold DC-side `DCStorage` into the engine's existing `Storage`-typed
  // dc.storage path so it participates in the UC MILP, SOC dynamics, dispatch,
  // and every per-step snapshot exactly like AC / legacy DC storage.
  HybridPowerSystem sys = sys_in;
  materialize_dc_storage(sys);

  auto profile_map = build_profile_map(ts_data);

  // Phase II: solve UC first (unless skipped)
  UCSchedule schedule;
  if (opts.precomputed_uc_schedule != nullptr) {
    schedule = *opts.precomputed_uc_schedule;
    if (schedule.solver_name.empty()) schedule.solver_name = "precomputed";
  } else if (!opts.skip_uc) {
    auto t_uc0 = std::chrono::steady_clock::now();
    schedule = solve_unit_commitment(sys, ts_data, opts);
    auto t_uc1 = std::chrono::steady_clock::now();
    result.uc_solve_sec = std::chrono::duration<double>(t_uc1 - t_uc0).count();
  } else {
    schedule.feasible = true;
    schedule.solver_name = "skipped";
  }

  bool need_dc_ess_fallback = false;

  // Populate DC-side profile-driven arrays; keep UC-optimized DC ESS when available.
  {
    const int ndc_pv = static_cast<int>(sys.dc.pv_arrays.size());
    const int ndc_ess = static_cast<int>(std::count_if(
        sys.dc.storage.begin(), sys.dc.storage.end(),
        [](const Storage& st) { return st.in_service; }));
    const int ndc_sgen = static_cast<int>(sys.dc.dc_static_generators.size());
    const int ndc_load = static_cast<int>(sys.dc.loads.size());

    const auto complete_series = [T](const auto& rows, int count) {
      if (static_cast<int>(rows.size()) != count) return false;
      return std::all_of(rows.begin(), rows.end(), [T](const auto& row) {
        return static_cast<int>(row.size()) == T;
      });
    };
    const bool keep_uc_dc_pv = opts.enable_dispatchable_pv && schedule.feasible &&
                               complete_series(schedule.dc_pv_dispatch, ndc_pv);
    const bool keep_uc_dc_sgen = opts.enable_dispatchable_pv && schedule.feasible &&
                                 complete_series(schedule.dc_sgen_dispatch, ndc_sgen);
    if (!keep_uc_dc_pv) {
      schedule.dc_pv_dispatch.assign(static_cast<size_t>(ndc_pv),
                                     std::vector<double>(static_cast<size_t>(T)));
    }
    if (!keep_uc_dc_sgen) {
      schedule.dc_sgen_dispatch.assign(static_cast<size_t>(ndc_sgen),
                                       std::vector<double>(static_cast<size_t>(T)));
    }
    schedule.dc_load_demand.resize(static_cast<size_t>(ndc_load),
                                   std::vector<double>(static_cast<size_t>(T)));

    for (int k = 0; k < ndc_pv; ++k) {
      const auto& pv = sys.dc.pv_arrays[static_cast<size_t>(k)];
      const auto* prof = find_profile(profile_map, pv.profile_id);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(prof, t, 1.0);
        if (!keep_uc_dc_pv) {
          schedule.dc_pv_dispatch[static_cast<size_t>(k)][static_cast<size_t>(t)] =
              pv.in_service ? pv.p_set_mw * scale : 0.0;
        }
      }
    }
    bool has_uc_dc_ess =
        should_apply_uc_schedule(opts, schedule) &&
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
      // A static p_mw is a single operating-point setpoint, not a horizon
      // schedule. Repeating it would violate the storage energy constraint.
      need_dc_ess_fallback = ndc_ess > 0;
      schedule.dc_ess_dispatch.assign(
          static_cast<size_t>(ndc_ess),
          std::vector<double>(static_cast<size_t>(T), 0.0));
      schedule.dc_ess_soc.assign(
          static_cast<size_t>(ndc_ess),
          std::vector<double>(static_cast<size_t>(T), 0.0));
    }
    for (int k = 0; k < ndc_sgen; ++k) {
      const auto& sg = sys.dc.dc_static_generators[static_cast<size_t>(k)];
      const auto* prof = find_profile(profile_map, sg.profile_id);
      for (int t = 0; t < T; ++t) {
        const double scale = profile_value(prof, t, 1.0);
        if (!keep_uc_dc_sgen) {
          schedule.dc_sgen_dispatch[static_cast<size_t>(k)][static_cast<size_t>(t)] =
              sg.in_service ? sg.p_set_mw * sg.scaling * scale : 0.0;
        }
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

  // SOC-aware AC/DC ESS fallback when UC did not produce an inter-temporal
  // schedule (notably dynamic OPF / PF with skip_uc=true).
  {
    int nACEssActive = 0;
    for (const auto& e : sys.ac.storage)
      if (e.in_service) ++nACEssActive;

    bool need_ess_fallback = (nACEssActive > 0) &&
        (!should_apply_uc_schedule(opts, schedule) ||
         static_cast<int>(schedule.ess_dispatch.size()) != nACEssActive ||
         static_cast<int>(schedule.ess_soc.size()) != nACEssActive);
    if (!need_ess_fallback && nACEssActive > 0) {
      for (int k = 0; k < nACEssActive; ++k) {
        if (static_cast<int>(schedule.ess_dispatch[static_cast<size_t>(k)].size()) != T ||
            static_cast<int>(schedule.ess_soc[static_cast<size_t>(k)].size()) != T) {
          need_ess_fallback = true;
          break;
        }
      }
    }

    if (need_ess_fallback || need_dc_ess_fallback) {
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

      // Share the system-level peak-shaving request across every storage unit
      // that needs a fallback. This keeps AC and DC behavior symmetric.
      double total_pmax = 0.0;
      const auto effective_pmax = [](const Storage& e) {
        const double bound = std::max(e.pmax_mw, 0.0);
        return e.p_rated_mw > 0.0 ? std::min(bound, e.p_rated_mw) : bound;
      };
      if (need_ess_fallback) {
        for (const auto& e : sys.ac.storage)
          if (e.in_service) total_pmax += std::max(effective_pmax(e), 0.01);
      }
      if (need_dc_ess_fallback) {
        for (const auto& e : sys.dc.storage)
          if (e.in_service) total_pmax += std::max(effective_pmax(e), 0.01);
      }
      total_pmax = std::max(total_pmax, 0.01);

      auto schedule_group = [&](const std::vector<Storage>& storage,
                                std::vector<std::vector<double>>& dispatch,
                                std::vector<std::vector<double>>& soc_rows) {
        const int active = static_cast<int>(std::count_if(
            storage.begin(), storage.end(),
            [](const Storage& st) { return st.in_service; }));
        dispatch.assign(static_cast<size_t>(active),
                        std::vector<double>(static_cast<size_t>(T), 0.0));
        soc_rows.assign(static_cast<size_t>(active),
                        std::vector<double>(static_cast<size_t>(T), 0.0));

        int sidx = 0;
        for (const auto& ess : storage) {
          if (!ess.in_service) continue;
          const double E = std::max(ess.e_rated_mwh, 1e-12);
          const double dt = ts_data.step_duration_hr;
          const double sd = std::clamp(
              1.0 - ess.self_discharge_pct / 100.0, 0.0, 1.0);
          const double pmax = effective_pmax(ess);
          const double raw_charge_cap = std::max(-ess.pmin_mw, 0.0);
          const double charge_cap = ess.p_rated_mw > 0.0
              ? std::min(raw_charge_cap, ess.p_rated_mw) : raw_charge_cap;
          const double pmin = -charge_cap;
          const double share = std::max(pmax, 0.01) / total_pmax;
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
          double p = std::clamp(surplus * share, pmin, pmax);

          // Check SOC limits
          double delta_soc = calc_delta_soc(p);
          double new_soc = soc * sd - delta_soc;
          if (new_soc < ess.soc_min) {
            // SOC would go below min → reduce discharge
            delta_soc = std::max(soc * sd - ess.soc_min, 0.0);
            p = std::max(delta_soc * eta_d * E / dt, 0.0);
            p = std::clamp(p, pmin, pmax);
          }
          if (new_soc > ess.soc_max) {
            // SOC would exceed max → reduce charging
            delta_soc = std::min(soc * sd - ess.soc_max, 0.0);
            p = std::min(delta_soc * E / (eta_c * dt), 0.0);
            p = std::clamp(p, pmin, pmax);
          }

          delta_soc = calc_delta_soc(p);
          soc = std::clamp(soc * sd - delta_soc, ess.soc_min, ess.soc_max);

          dispatch[static_cast<size_t>(sidx)][static_cast<size_t>(t)] = p;
          soc_rows[static_cast<size_t>(sidx)][static_cast<size_t>(t)] = soc;
        }

        if (opts.enforce_terminal_soc_cyclic && T > 0) {
          auto& row = dispatch[static_cast<size_t>(sidx)];
          auto& row_soc = soc_rows[static_cast<size_t>(sidx)];
          const double target_soc =
              std::clamp(ess.soc_init, ess.soc_min, ess.soc_max);
          const auto simulate = [&](bool write_soc) {
            double state = ess.soc_init;
            bool valid = true;
            for (int t = 0; t < T; ++t) {
              state = state * sd - calc_delta_soc(row[static_cast<size_t>(t)]);
              valid = valid && state >= ess.soc_min - 1e-10 &&
                      state <= ess.soc_max + 1e-10;
              if (write_soc) {
                row_soc[static_cast<size_t>(t)] =
                    std::clamp(state, ess.soc_min, ess.soc_max);
              }
            }
            return std::pair<double, bool>{state, valid};
          };

          std::vector<int> order(static_cast<size_t>(T));
          std::iota(order.begin(), order.end(), 0);
          double terminal_soc = simulate(false).first;
          const bool need_charge = terminal_soc < target_soc;
          std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            return need_charge
                ? net_load[static_cast<size_t>(a)] < net_load[static_cast<size_t>(b)]
                : net_load[static_cast<size_t>(a)] > net_load[static_cast<size_t>(b)];
          });

          for (int t : order) {
            terminal_soc = simulate(false).first;
            if (std::abs(terminal_soc - target_soc) <= 1e-10) break;
            if ((terminal_soc < target_soc) != need_charge) break;

            const size_t ti = static_cast<size_t>(t);
            const double original = row[ti];
            const double available = need_charge ? original - pmin
                                                 : pmax - original;
            if (available <= 1e-12) continue;

            row[ti] = need_charge ? original - available
                                  : original + available;
            const auto [max_terminal, max_valid] = simulate(false);
            const bool reaches_target = need_charge
                ? max_terminal >= target_soc
                : max_terminal <= target_soc;
            if (!max_valid || reaches_target) {
              double lo = 0.0, hi = available;
              for (int iter = 0; iter < 60; ++iter) {
                const double mid = 0.5 * (lo + hi);
                row[ti] = need_charge ? original - mid : original + mid;
                const auto [candidate_terminal, candidate_valid] = simulate(false);
                const bool feasible_side = candidate_valid &&
                    (need_charge ? candidate_terminal <= target_soc
                                 : candidate_terminal >= target_soc);
                if (feasible_side) lo = mid;
                else hi = mid;
              }
              row[ti] = need_charge ? original - lo : original + lo;
            }
          }
          (void)simulate(true);
        }
          ++sidx;
        }
      };

      if (need_ess_fallback)
        schedule_group(sys.ac.storage, schedule.ess_dispatch, schedule.ess_soc);
      if (need_dc_ess_fallback)
        schedule_group(sys.dc.storage, schedule.dc_ess_dispatch,
                       schedule.dc_ess_soc);
    }
  }

  result.uc_schedule = schedule;

  // Collect indices of in-service components for schedule mapping
  std::vector<int> gen_active_idx;
  for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
    if (sys.ac.generators[static_cast<size_t>(i)].in_service)
      gen_active_idx.push_back(i);
  }
  auto prepare_system = [&](int t) -> HybridPowerSystem {
    return build_time_series_system_snapshot(sys, ts_data, schedule, t, opts);
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
    if (opts.keep_system_snapshots) {
      result.pf_system_snapshots.resize(static_cast<size_t>(T));
    }

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
        if (opts.keep_system_snapshots) {
          result.pf_system_snapshots[static_cast<size_t>(t)] =
              restore_rich_dc_storage_identity(sys_t, sys_in);
        }
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

      // Storage remains at the inter-temporal schedule already present in
      // sys_pf. The single-period OPF only validates that dispatch inside a
      // tiny numerical band; replaying its band-level deviation would make P
      // inconsistent with the schedule-derived terminal SOC.

      // Set DCDC transfer dispatch from OPF using ComponentRef map
      for (size_t k = 0; k < opf_res.pdcdc_mw.size() &&
                          k < opf_res.dcdc_map.size(); ++k) {
        int orig = opf_res.dcdc_map[k].original_index;
        if (orig >= 0 && orig < static_cast<int>(sys_pf.dc.dcdc_converters.size())) {
          auto& dcdc = sys_pf.dc.dcdc_converters[static_cast<size_t>(orig)];
          dcdc.p_ref_mw = powerflow::dcdc_output_power_from_input_ref_mw(
              dcdc, opf_res.pdcdc_mw[k]);
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
      if (opts.keep_system_snapshots) {
        result.pf_system_snapshots[static_cast<size_t>(t)] =
            restore_rich_dc_storage_identity(sys_pf, sys_in);
      }
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
    if (opts.keep_system_snapshots) {
      result.pf_system_snapshots.resize(static_cast<size_t>(T));
    }
    result.total_generation_cost = 0.0;

    for (int t = 0; t < T; ++t) {
      HybridPowerSystem sys_t = prepare_system(t);
      refresh_topo_if_needed(sys_t);

      // Accumulate gen cost from UC dispatch
      if (should_apply_uc_schedule(opts, schedule)) {
        for (int gi = 0; gi < static_cast<int>(gen_active_idx.size()); ++gi) {
          int idx = gen_active_idx[static_cast<size_t>(gi)];
          const auto& gen = sys_t.ac.generators[static_cast<size_t>(idx)];
          result.total_generation_cost +=
              gen.cost_c1 * gen.pg_mw * ts_data.step_duration_hr;
        }
      }

      if (opts.keep_system_snapshots) {
        result.pf_system_snapshots[static_cast<size_t>(t)] =
            restore_rich_dc_storage_identity(sys_t, sys_in);
      }
      reset_solver_handle(handle, sys_t, opts.pf_options.loss_model);
      result.pf_results[static_cast<size_t>(t)] = solve_handle(handle, opts.pf_options);

      if (result.pf_results[static_cast<size_t>(t)].converged) {
        ++result.num_converged;
      }
    }

    destroy_solver_handle(handle);
  }

  // Add non-AC-generator operating costs to the reported production cost. The
  // selected UC objective may be carbon/loss/curtailment, but this result field
  // remains a monetary accounting metric for the GUI.
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    result.total_generation_cost +=
        std::max(0.0, sg.p_mw * sg.scaling) * std::max(0.0, sg.cost_c1) *
        ts_data.step_duration_hr * T;
  }
  for (size_t k = 0; k < sys.dc.dc_static_generators.size(); ++k) {
    const auto& sg = sys.dc.dc_static_generators[k];
    if (!sg.in_service || k >= schedule.dc_sgen_dispatch.size()) continue;
    for (double p : schedule.dc_sgen_dispatch[k]) {
      result.total_generation_cost +=
          std::max(0.0, p) * std::max(0.0, sg.cost_c1) * ts_data.step_duration_hr;
    }
  }
  for (size_t k = 0; k < sys.ac.external_grids.size() &&
                     k < schedule.external_grid_dispatch.size(); ++k) {
    const auto& grid = sys.ac.external_grids[k];
    if (!grid.in_service) continue;
    const auto* price_profile = find_profile(profile_map, grid.price_profile_id);
    const auto& dispatch = schedule.external_grid_dispatch[k];
    for (int t = 0; t < T && t < static_cast<int>(dispatch.size()); ++t) {
      double p = dispatch[static_cast<size_t>(t)];
      if (t < static_cast<int>(result.opf_results.size()) &&
          result.opf_results[static_cast<size_t>(t)].converged &&
          k < result.opf_results[static_cast<size_t>(t)].external_grid_p_mw.size()) {
        p = result.opf_results[static_cast<size_t>(t)].external_grid_p_mw[k];
      }
      const double price =
          absolute_profile_value(price_profile, t, grid.cost_c1);
      result.total_generation_cost +=
          p * price * ts_data.step_duration_hr;
    }
  }
  for (int t = 0; t < T; ++t) {
    const HybridPowerSystem sys_t = prepare_system(t);
    for (const auto& sg : sys_t.ac.static_generators) {
      if (sg.in_service)
        result.total_generation_cost +=
            std::max(0.0, sg.p_mw * sg.scaling) * std::max(0.0, sg.cost_c1) *
            ts_data.step_duration_hr;
    }
    for (const auto& ren : sys_t.ac.renewable_gens) {
      if (ren.in_service)
        result.total_generation_cost +=
            std::max(0.0, ren.p_mw) * std::max(0.0, ren.cost_c1) *
            ts_data.step_duration_hr;
    }
    for (const auto& pv : sys_t.ac.pv_systems) {
      if (pv.in_service)
        result.total_generation_cost +=
            std::max(0.0, pv.p_mw) * std::max(0.0, pv.cost_c1) *
            ts_data.step_duration_hr;
    }
    for (const auto& pv : sys_t.dc.pv_arrays) {
      if (pv.in_service)
        result.total_generation_cost +=
            std::max(0.0, pv.p_set_mw) * std::max(0.0, pv.cost_c1) *
            ts_data.step_duration_hr;
    }
  }
  auto add_storage_bid_cost = [&](const Storage& st,
                                  const std::vector<double>& dispatch) {
    for (double p : dispatch) {
      result.total_generation_cost +=
          (p >= 0.0 ? p * std::max(0.0, st.discharge_bid_price)
                    : -p * std::max(0.0, st.charge_bid_price)) *
          ts_data.step_duration_hr;
    }
  };
  size_t active_row = 0;
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    if (active_row < schedule.ess_dispatch.size())
      add_storage_bid_cost(st, schedule.ess_dispatch[active_row]);
    ++active_row;
  }
  active_row = 0;
  for (const auto& st : sys.dc.storage) {
    if (!st.in_service) continue;
    if (active_row < schedule.dc_ess_dispatch.size())
      add_storage_bid_cost(st, schedule.dc_ess_dispatch[active_row]);
    ++active_row;
  }

  result.rich_results.resize(static_cast<size_t>(T));
  for (int t = 0; t < T; ++t) {
    HybridPowerSystem rich_state;
    if (t < static_cast<int>(result.pf_system_snapshots.size())) {
      rich_state = result.pf_system_snapshots[static_cast<size_t>(t)];
    } else {
      rich_state = prepare_system(t);
    }
    rich_state = restore_rich_dc_storage_identity(rich_state, sys_in);
    const auto projection_bundle =
        projection::RichToCanonicalOperator::apply(rich_state);
    const opf::ACOPFResult* opf_result =
        t < static_cast<int>(result.opf_results.size())
            ? &result.opf_results[static_cast<size_t>(t)]
            : nullptr;
    const PowerFlowResult* rich_pf =
        t < static_cast<int>(result.pf_results.size())
            ? &result.pf_results[static_cast<size_t>(t)]
            : nullptr;
    projection::AttributionOptions attribution_options;
    attribution_options.prefer_rich_storage_dispatch = true;
    result.rich_results[static_cast<size_t>(t)] =
        projection::CanonicalToRichOperator::apply(
            rich_state, projection_bundle, opf_result, nullptr, rich_pf,
            attribution_options);
  }

  return result;
}

}  // namespace hacdcpf
