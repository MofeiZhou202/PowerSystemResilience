// Distribution Power Flow — Backward-Forward Sweep implementation
// DistributionPowerFlow.jl PowerFlow module equivalent (C++20)
//
// Algorithm reference: Shirmohammadi et al., "A compensation-based power
// flow method for weakly meshed distribution and transmission networks"
// IEEE Trans. Power Systems, 1988.
//
// Three-phase BFS extension: Cheng & Shirmohammadi, "A three-phase power
// flow method for real-time distribution system analysis"
// IEEE Trans. Power Systems, 1995.

#include "hacdcpf/analysis/distribution_power_flow.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <complex>
#include <fstream>
#include <limits>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>

#include "hacdcpf/detail/logging.hpp"

#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"

#ifdef HACDCPF_HAVE_OPENDSS
#include "hacdcpf/io/dss_capi_adapter.hpp"
#include "hacdcpf/io/opendss_bridge.hpp"
#endif

namespace hacdcpf::analysis {

namespace {

constexpr double kPi = 3.14159265358979323846;

// -------------------------------------------------------------------------
// Build bus-ID → local-index map (0-indexed)
// -------------------------------------------------------------------------
std::unordered_map<int, int>
build_id_map(const std::vector<ACBus>& buses) {
  std::unordered_map<int, int> m;
  m.reserve(buses.size());
  for (int i = 0; i < static_cast<int>(buses.size()); ++i)
    m[buses[i].index] = i;
  return m;
}

// -------------------------------------------------------------------------
// BFS tree data
// -------------------------------------------------------------------------
struct BFSTree {
  std::vector<int> order;           // buses in BFS order (local indices)
  std::vector<int> parent_bus;      // parent_bus[i] = local idx of parent, -1 for root
  std::vector<int> parent_branch;   // index into branches[], -1 for root
  std::vector<bool> parent_is_from_side;  // true if parent maps to ACBranch.from_bus
};

BFSTree build_bfs_tree(const std::vector<ACBus>&   buses,
                       const std::vector<ACBranch>& branches,
                       const std::unordered_map<int, int>& id_map,
                       int root_local) {
  const int n = static_cast<int>(buses.size());

  // Adjacency list: adj[i] = {(neighbour_local, branch_idx)}
  std::vector<std::vector<std::pair<int, int>>> adj(n);
  for (int b = 0; b < static_cast<int>(branches.size()); ++b) {
    if (!branches[b].in_service) continue;
    auto it_f = id_map.find(branches[b].from_bus);
    auto it_t = id_map.find(branches[b].to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second, ti = it_t->second;
    adj[fi].emplace_back(ti, b);
    adj[ti].emplace_back(fi, b);
  }

  BFSTree tree;
  tree.order.reserve(n);
  tree.parent_bus.assign(n, -1);
  tree.parent_branch.assign(n, -1);
  tree.parent_is_from_side.assign(n, false);

  std::vector<bool> visited(n, false);
  std::queue<int> q;
  q.push(root_local);
  visited[root_local] = true;

  while (!q.empty()) {
    int u = q.front(); q.pop();
    tree.order.push_back(u);
    for (auto [v, b] : adj[u]) {
      if (!visited[v]) {
        visited[v] = true;
        tree.parent_bus[v] = u;
        tree.parent_branch[v] = b;
        tree.parent_is_from_side[v] = (branches[b].from_bus == buses[u].index);
        q.push(v);
      }
    }
  }
  return tree;
}

using Complex = std::complex<double>;

Complex branch_complex_tap(const ACBranch& branch) {
  const double tap_mag = (std::abs(branch.tap) < 1e-12) ? 1.0 : branch.tap;
  const double shift_rad = branch.shift_deg * (kPi / 180.0);
  return std::polar(tap_mag, shift_rad);
}

Complex branch_series_current(const ACBranch& branch,
                              Complex v_from,
                              Complex v_to) {
  const Complex z(branch.r_pu, branch.x_pu);
  if (std::abs(z) < 1e-20) {
    return {0.0, 0.0};
  }
  return (v_from / branch_complex_tap(branch) - v_to) / z;
}

Complex branch_current_from_bus_into_branch(const ACBranch& branch,
                                            Complex v_from,
                                            Complex v_to) {
  return branch_series_current(branch, v_from, v_to) /
         std::conj(branch_complex_tap(branch));
}

Complex branch_current_to_bus_into_branch(const ACBranch& branch,
                                          Complex v_from,
                                          Complex v_to) {
  return -branch_series_current(branch, v_from, v_to);
}

Complex branch_subtree_current_seen_from_parent(const ACBranch& branch,
                                                bool parent_is_from_side,
                                                Complex child_bus_current) {
  const Complex tap = branch_complex_tap(branch);
  return parent_is_from_side ? (child_bus_current / std::conj(tap))
                             : (std::conj(tap) * child_bus_current);
}

Complex branch_child_voltage_from_parent(const ACBranch& branch,
                                         bool parent_is_from_side,
                                         Complex parent_voltage,
                                         Complex child_bus_current) {
  const Complex z(branch.r_pu, branch.x_pu);
  const Complex tap = branch_complex_tap(branch);
  if (parent_is_from_side) {
    return parent_voltage / tap - z * child_bus_current;
  }
  return tap * (parent_voltage - z * std::conj(tap) * child_bus_current);
}

struct SinglePhaseBranchFlow {
  Complex current_from;
  Complex current_to;
  Complex power_from;
  Complex power_to;
  Complex loss;
};

SinglePhaseBranchFlow compute_single_phase_branch_flow(const ACBranch& branch,
                                                       Complex v_from,
                                                       Complex v_to,
                                                       double base_mva) {
  const Complex ifrom = branch_current_from_bus_into_branch(branch, v_from, v_to);
  const Complex ito = branch_current_to_bus_into_branch(branch, v_from, v_to);
  const Complex s_from = v_from * std::conj(ifrom) * base_mva;
  const Complex s_to = v_to * std::conj(ito) * base_mva;
  return {
      .current_from = ifrom,
      .current_to = ito,
      .power_from = s_from,
      .power_to = s_to,
      .loss = s_from + s_to,
  };
}

// -------------------------------------------------------------------------
// Compute net injection at each bus from component tables (or bus-level
// fallback). Returns S_net[i] = (Pd - Pg) / base_mva in complex form.
// -------------------------------------------------------------------------
std::vector<std::complex<double>>
compute_net_injection(const ACSystem& ac_sys,
                      const std::unordered_map<int, int>& id_map,
                      int root, int n, double base_mva,
                      bool include_shunts) {
  const auto& buses = ac_sys.buses;
  std::vector<std::complex<double>> S_net(n, {0.0, 0.0});

  // Load demand: use Load table if available, else bus-level pd/qd.
  if (!ac_sys.loads.empty()) {
    for (const auto& ld : ac_sys.loads) {
      if (!ld.in_service) continue;
      auto it = id_map.find(ld.bus);
      if (it == id_map.end()) continue;
      const int idx = it->second;
      // At initial/current iteration, use constant-power part.
      // ZIP model: S = S0 * (Pp + Ip*V + Zp*V^2), but BFS updates V each iter.
      // For BFS net injection we use S0 directly (constant-power approximation).
      S_net[idx] += std::complex<double>(ld.p_mw / base_mva, ld.q_mvar / base_mva);
    }
  } else {
    // Fallback: bus-level demand.
    for (int i = 0; i < n; ++i) {
      if (!buses[i].in_service) continue;
      S_net[i] = std::complex<double>(buses[i].pd_mw / base_mva, buses[i].qd_mvar / base_mva);
    }
  }

  // Charging station load (kW → MW).
  for (const auto& cs : ac_sys.charging_stations) {
    if (!cs.in_service) continue;
    auto it = id_map.find(cs.bus);
    if (it == id_map.end()) continue;
    const int idx = it->second;
    S_net[idx] += std::complex<double>((cs.p_total_kw / 1000.0) / base_mva,
                                       (cs.q_total_kvar / 1000.0) / base_mva);
  }

  // Shunt contribution (bus-level).
  if (include_shunts) {
    for (int i = 0; i < n; ++i) {
      if (!buses[i].in_service) continue;
      S_net[i] += std::complex<double>(buses[i].gs_mw / base_mva,
                                       -buses[i].bs_mvar / base_mva);
    }
    // Shunt table entries (separate from bus-level gs/bs).
    for (const auto& sh : ac_sys.shunts) {
      if (!sh.in_service) continue;
      auto it = id_map.find(sh.bus);
      if (it == id_map.end()) continue;
      const int idx = it->second;
      double gs = sh.gs_mw;
      double bs = sh.bs_mvar;
      if (sh.switchable && sh.n_steps > 0) {
        bs = sh.bs_per_step * sh.current_step;
      }
      S_net[idx] += std::complex<double>(gs / base_mva, -bs / base_mva);
    }
  }

  // Subtract generation: standard generators (skip slack).
  for (const auto& g : ac_sys.generators) {
    if (!g.in_service) continue;
    auto it = id_map.find(g.bus);
    if (it == id_map.end()) continue;
    const int gi = it->second;
    if (gi == root) continue;
    S_net[gi] -= std::complex<double>(g.pg_mw / base_mva, g.qg_mvar / base_mva);
  }

  // Subtract generation: static generators.
  for (const auto& sg : ac_sys.static_generators) {
    if (!sg.in_service) continue;
    auto it = id_map.find(sg.bus);
    if (it == id_map.end()) continue;
    const int idx = it->second;
    if (idx == root) continue;
    S_net[idx] -= std::complex<double>(sg.p_mw * sg.scaling / base_mva,
                                       sg.q_mvar * sg.scaling / base_mva);
  }

  // Subtract generation: renewable generators.
  for (const auto& rg : ac_sys.renewable_gens) {
    if (!rg.in_service) continue;
    auto it = id_map.find(rg.bus);
    if (it == id_map.end()) continue;
    const int idx = it->second;
    if (idx == root) continue;
    S_net[idx] -= std::complex<double>(rg.p_mw / base_mva, rg.q_mvar / base_mva);
  }

  // Subtract generation: PV systems (with power curve).
  for (const auto& pv : ac_sys.pv_systems) {
    if (!pv.in_service) continue;
    auto it = id_map.find(pv.bus);
    if (it == id_map.end()) continue;
    const int idx = it->second;
    if (idx == root) continue;
    const double p_mw = powerflow::compute_pv_power_mw(pv);
    S_net[idx] -= std::complex<double>(p_mw / base_mva, pv.q_mvar / base_mva);
  }

  // Subtract generation: storage (positive = discharge = generation).
  for (const auto& st : ac_sys.storage) {
    if (!st.in_service) continue;
    auto it = id_map.find(st.bus);
    if (it == id_map.end()) continue;
    const int idx = it->second;
    if (idx == root) continue;
    S_net[idx] -= std::complex<double>(st.p_mw / base_mva, st.q_mvar / base_mva);
  }

  return S_net;
}

// -------------------------------------------------------------------------
// Run the BFS inner loop (backward-forward sweep).
// Returns max|ΔV| residual and updates V in place.
// -------------------------------------------------------------------------
struct BFSInnerResult {
  double residual{0.0};
  int iterations{0};
  bool converged{false};
};

BFSInnerResult run_bfs_sweep(const std::vector<ACBranch>& branches,
                             const BFSTree& tree,
                             const std::vector<std::complex<double>>& S_net,
                             std::vector<std::complex<double>>& V,
                             int root, int n, int m,
                             const DPFOptions& opt) {
  std::vector<Complex> I_branch(m, {0.0, 0.0});
  double residual = std::numeric_limits<double>::max();
  int iter = 0;

  for (; iter < opt.max_iter; ++iter) {
    // Backward sweep: compute branch currents from net injection.
    for (int k = 1; k < static_cast<int>(tree.order.size()); ++k) {
      int j = tree.order[k];
      int b = tree.parent_branch[j];
      Complex Vj = V[j];
      if (std::abs(Vj) > 1e-12)
        I_branch[b] = std::conj(S_net[j] / Vj);
      else
        I_branch[b] = {0.0, 0.0};
    }
    // Accumulate child currents into parent branch (reverse BFS order).
    for (int k = static_cast<int>(tree.order.size()) - 1; k >= 1; --k) {
      int j = tree.order[k];
      int i = tree.parent_bus[j];
      int bj = tree.parent_branch[j];
      if (i != root) {
        int bi = tree.parent_branch[i];
        I_branch[bi] += branch_subtree_current_seen_from_parent(
            branches[bj], tree.parent_is_from_side[j], I_branch[bj]);
      }
    }

    // Forward sweep: update voltages.
    std::vector<std::complex<double>> V_new = V;
    V_new[root] = V[root];
    for (int k = 1; k < static_cast<int>(tree.order.size()); ++k) {
      int j = tree.order[k];
      int i = tree.parent_bus[j];
      int b = tree.parent_branch[j];
      V_new[j] = branch_child_voltage_from_parent(
          branches[b], tree.parent_is_from_side[j], V_new[i], I_branch[b]);
    }

    // Convergence check.
    residual = 0.0;
    for (int i = 0; i < n; ++i)
      residual = std::max(residual, std::abs(V_new[i] - V[i]));

    V = V_new;

    if (opt.verbose)
      HACDCPF_LOG_DEBUG("  DPF iter {}  residual = {}", iter + 1, residual);

    if (residual < opt.tol) { ++iter; break; }
  }

  return {residual, iter, residual < opt.tol};
}

struct SinglePhaseSolveSnapshot {
  DPFResult result;
  std::vector<Complex> voltages;
  std::unordered_map<int, int> id_map;
};

SinglePhaseSolveSnapshot solve_distribution_pf_ac_snapshot(
    const ACSystem& ac_sys,
    const DPFOptions& opt) {
  const auto& buses = ac_sys.buses;
  const auto& branches = ac_sys.branches;
  const double base_mva = (ac_sys.base_mva > 0.0) ? ac_sys.base_mva : 100.0;

  const int n = static_cast<int>(buses.size());
  const int m = static_cast<int>(branches.size());

  DPFResult result;
  result.vm_pu.assign(n, 1.0);
  result.va_deg.assign(n, 0.0);
  result.p_branch_mw.assign(m, 0.0);
  result.q_branch_mvar.assign(m, 0.0);

  if (n == 0) {
    result.control_converged = true;
    return {std::move(result), {}, {}};
  }

  const auto id_map = build_id_map(buses);

  int root = -1;
  for (int i = 0; i < n; ++i) {
    if (buses[i].bus_type == BusType::SLACK) {
      root = i;
      break;
    }
  }
  if (root < 0) {
    throw std::runtime_error("solve_distribution_pf: no SLACK bus found");
  }

  const BFSTree tree = build_bfs_tree(buses, branches, id_map, root);

  std::vector<std::complex<double>> S_net =
      compute_net_injection(ac_sys, id_map, root, n, base_mva, opt.include_shunts);

  std::vector<std::complex<double>> V(n, {1.0, 0.0});
  for (int i = 0; i < n; ++i) {
    V[i] = std::complex<double>(buses[i].vm_pu, 0.0);
  }
  V[root] = std::complex<double>(buses[root].vm_pu, 0.0);

  BFSInnerResult bfs_result;
  if (!opt.enforce_q_limits) {
    bfs_result = run_bfs_sweep(branches, tree, S_net, V, root, n, m, opt);
  } else {
    std::vector<bool> gen_q_clamped(ac_sys.generators.size(), false);

    for (int outer = 0; outer < opt.q_limit_max_outer_iter; ++outer) {
      bfs_result = run_bfs_sweep(branches, tree, S_net, V, root, n, m, opt);
      if (!bfs_result.converged) {
        break;
      }

      bool any_violation = false;
      for (size_t gi = 0; gi < ac_sys.generators.size(); ++gi) {
        const auto& g = ac_sys.generators[gi];
        if (!g.in_service || gen_q_clamped[gi]) continue;
        auto it = id_map.find(g.bus);
        if (it == id_map.end()) continue;
        const int idx = it->second;
        if (idx == root) continue;
        if (g.qmax_mvar == 0.0 && g.qmin_mvar == 0.0) continue;

        Complex Vbus = V[idx];
        double q_bus_calc = 0.0;
        for (int b = 0; b < m; ++b) {
          if (!branches[b].in_service) continue;
          auto it_f = id_map.find(branches[b].from_bus);
          auto it_t = id_map.find(branches[b].to_bus);
          if (it_f == id_map.end() || it_t == id_map.end()) continue;
          int fi = it_f->second;
          int ti = it_t->second;
          if (fi != idx && ti != idx) continue;

          const auto flow = compute_single_phase_branch_flow(
              branches[b], V[fi], V[ti], 1.0);
          const Complex I = (fi == idx) ? flow.current_from : flow.current_to;
          const Complex S = Vbus * std::conj(I);
          q_bus_calc += S.imag();
        }

        double q_demand = S_net[idx].imag() * base_mva + g.qg_mvar;
        double q_gen_required = (q_bus_calc * base_mva) + q_demand;

        if (q_gen_required > g.qmax_mvar) {
          const double q_clamped = g.qmax_mvar;
          S_net[idx].imag(S_net[idx].imag() - (q_clamped - g.qg_mvar) / base_mva);
          gen_q_clamped[gi] = true;
          any_violation = true;
          if (opt.verbose) {
            HACDCPF_LOG_DEBUG("  Q-limit: gen {} at bus {} clamped to Qmax={} MVAr",
                              gi, g.bus, g.qmax_mvar);
          }
        } else if (q_gen_required < g.qmin_mvar) {
          const double q_clamped = g.qmin_mvar;
          S_net[idx].imag(S_net[idx].imag() - (q_clamped - g.qg_mvar) / base_mva);
          gen_q_clamped[gi] = true;
          any_violation = true;
          if (opt.verbose) {
            HACDCPF_LOG_DEBUG("  Q-limit: gen {} at bus {} clamped to Qmin={} MVAr",
                              gi, g.bus, g.qmin_mvar);
          }
        }
      }

      if (!any_violation) {
        break;
      }

      for (int i = 0; i < n; ++i) {
        V[i] = std::complex<double>(buses[i].vm_pu, 0.0);
      }
      V[root] = std::complex<double>(buses[root].vm_pu, 0.0);
    }
  }

  result.converged = bfs_result.converged;
  result.iterations = bfs_result.iterations;
  result.residual = bfs_result.residual;
  result.control_converged = true;

  for (int i = 0; i < n; ++i) {
    result.vm_pu[i] = std::abs(V[i]);
    result.va_deg[i] = std::arg(V[i]) * (180.0 / kPi);
  }

  double total_p_loss = 0.0;
  double total_q_loss = 0.0;
  for (int b = 0; b < m; ++b) {
    if (!branches[b].in_service) continue;
    auto it_f = id_map.find(branches[b].from_bus);
    auto it_t = id_map.find(branches[b].to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second;
    int ti = it_t->second;

    const auto flow = compute_single_phase_branch_flow(branches[b], V[fi], V[ti], base_mva);
    result.p_branch_mw[b] = flow.power_from.real();
    result.q_branch_mvar[b] = flow.power_from.imag();
    total_p_loss += flow.loss.real();
    total_q_loss += flow.loss.imag();
  }
  result.total_p_loss_mw = total_p_loss;
  result.total_q_loss_mvar = total_q_loss;

  return {std::move(result), std::move(V), id_map};
}

double transformer_tap_pu(const Transformer2W& tr) {
  const double step = tr.tap_step_percent;
  if (std::abs(step) < 1e-12) {
    return 1.0;
  }
  return std::max(
      1e-6,
      1.0 + (static_cast<double>(tr.tap_pos - tr.tap_neutral) * step / 100.0));
}

int transformer_tap_number(const Transformer2W& tr) {
  return tr.tap_pos - tr.tap_neutral;
}

int transformer_winding_bus(const Transformer2W& tr, int winding) {
  if (winding == 1) return tr.hv_bus;
  if (winding == 2) return tr.lv_bus;
  throw std::runtime_error("Unsupported transformer winding " + std::to_string(winding));
}

bool tap_side_matches_winding(const Transformer2W& tr, int tap_winding) {
  return (tap_winding == 1 && tr.tap_side == 0) ||
         (tap_winding == 2 && tr.tap_side == 1);
}

int tap_pos_delta_for_raise_voltage(const Transformer2W& tr) {
  return (tr.tap_side == 1) ? 1 : -1;
}

int find_transformer_position_by_index(const std::vector<Transformer2W>& transformers,
                                       int transformer_index) {
  for (std::size_t pos = 0; pos < transformers.size(); ++pos) {
    if (transformers[pos].index == transformer_index) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error(
      "Missing Transformer2W with index " + std::to_string(transformer_index));
}

int find_branch_position_by_index(const ACSystem& ac, int branch_index) {
  for (std::size_t pos = 0; pos < ac.branches.size(); ++pos) {
    if (ac.branches[pos].index == branch_index) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error(
      "Missing projected branch index " + std::to_string(branch_index));
}

int find_projected_branch_position(const HybridPowerSystem& projected,
                                   BranchOriginType origin_type,
                                   int origin_index) {
  if (!projected.branch_expand_map.has_value()) {
    throw std::runtime_error("Projected system is missing BranchExpandMap");
  }
  for (const auto& entry : projected.branch_expand_map->entries) {
    if (entry.origin_type == origin_type && entry.origin_index == origin_index) {
      return find_branch_position_by_index(projected.ac, entry.branch_index);
    }
  }
  throw std::runtime_error(
      "Missing projected branch mapping for origin index " +
      std::to_string(origin_index));
}

int projected_bus_position_for_original_bus(const HybridPowerSystem& projected,
                                            int original_bus) {
  if (projected.bus_merge_map.has_value() && projected.bus_merge_map->has_merges()) {
    const auto it = projected.bus_merge_map->ext_to_int.find(original_bus);
    if (it == projected.bus_merge_map->ext_to_int.end()) {
      throw std::runtime_error(
          "Merged projected system is missing original bus " +
          std::to_string(original_bus));
    }
    return it->second;
  }

  for (std::size_t pos = 0; pos < projected.ac.buses.size(); ++pos) {
    if (projected.ac.buses[pos].index == original_bus) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error(
      "Projected system is missing bus " + std::to_string(original_bus));
}

double bus_base_voltage_volts(const ACBus& bus) {
  if (bus.base_kv <= 1e-9) {
    throw std::runtime_error("Bus " + std::to_string(bus.index) +
                             " is missing base_kv for regulator voltage control");
  }
  return bus.base_kv * 1000.0;
}

constexpr const char* kTransformer3WIsolatedTriangleTrialPathId =
    "transformer3w_isolated_triangle_dense_nr_trial";
constexpr const char* kTransformer3WSlackLeafEmbedTrialPathId =
    "transformer3w_slack_leaf_embed_dense_nr_trial";

struct SpecializedSinglePhasePathAttempt {
  bool eligible{false};
  bool applied{false};
  std::string path_id;
  std::string fail_close_reason;
  SinglePhaseSolveSnapshot snapshot;
};

SpecializedSinglePhasePathAttempt try_solve_transformer3w_isolated_triangle_snapshot(
    const HybridPowerSystem& projected,
    const DPFOptions& opt) {
  SpecializedSinglePhasePathAttempt attempt;
  attempt.path_id = kTransformer3WIsolatedTriangleTrialPathId;

  if (projected.bus_merge_map.has_value() &&
      projected.bus_merge_map->has_merges()) {
    return attempt;
  }
  if (!projected.branch_expand_map.has_value()) {
    return attempt;
  }
  if (opt.enforce_q_limits) {
    attempt.eligible = true;
    attempt.fail_close_reason = "q_limit_outer_loop_not_supported_for_local_triangle_trial";
    return attempt;
  }

  const auto& ac = projected.ac;
  if (ac.buses.size() < 3 || ac.buses.size() > 4 ||
      ac.branches.size() < 3 || ac.branches.size() > 4) {
    return attempt;
  }
  if (!ac.transformers_2w.empty() || !ac.switches.empty() || !ac.transformers_3w.empty()) {
    return attempt;
  }

  int transformer3w_origin = 0;
  std::set<int> pair_numbers;
  std::set<int> transformer_branch_positions;
  std::set<int> triangle_bus_ids;
  for (const auto& entry : projected.branch_expand_map->entries) {
    if (entry.origin_type != BranchOriginType::Transformer3W) {
      return attempt;
    }
    if (transformer3w_origin == 0) {
      transformer3w_origin = entry.origin_index;
    } else if (transformer3w_origin != entry.origin_index) {
      return attempt;
    }
    pair_numbers.insert(entry.pair_number);
    const int branch_pos =
        find_branch_position_by_index(ac, entry.branch_index);
    transformer_branch_positions.insert(branch_pos);
    triangle_bus_ids.insert(
        ac.branches[static_cast<std::size_t>(branch_pos)].from_bus);
    triangle_bus_ids.insert(
        ac.branches[static_cast<std::size_t>(branch_pos)].to_bus);
  }
  if (pair_numbers != std::set<int>({0, 1, 2})) {
    return attempt;
  }
  if (transformer_branch_positions.size() != 3 || triangle_bus_ids.size() != 3) {
    return attempt;
  }

  std::vector<int> extra_branch_positions;
  for (std::size_t pos = 0; pos < ac.branches.size(); ++pos) {
    if (!transformer_branch_positions.contains(static_cast<int>(pos))) {
      extra_branch_positions.push_back(static_cast<int>(pos));
    }
  }
  if (extra_branch_positions.size() > 1) {
    return attempt;
  }

  std::vector<int> extra_bus_ids;
  for (const auto& bus : ac.buses) {
    if (!triangle_bus_ids.contains(bus.index)) {
      extra_bus_ids.push_back(bus.index);
    }
  }
  if (extra_bus_ids.size() > 1) {
    return attempt;
  }
  const bool isolated_triangle_shape =
      extra_branch_positions.empty() && extra_bus_ids.empty();
  const bool slack_leaf_embed_shape =
      extra_branch_positions.size() == 1 && extra_bus_ids.size() == 1 &&
      ac.buses.size() == 4 && ac.branches.size() == 4;
  if (!isolated_triangle_shape && !slack_leaf_embed_shape) {
    return attempt;
  }

  int slack_count = 0;
  int slack_bus_id = 0;
  std::unordered_map<int, int> bus_degree;
  for (const auto& bus : ac.buses) {
    if (bus.bus_type == BusType::SLACK) {
      slack_count += 1;
      slack_bus_id = bus.index;
    } else if (bus.bus_type != BusType::PQ) {
      attempt.eligible = true;
      attempt.fail_close_reason = "non_pq_non_slack_bus_not_supported_for_local_triangle_trial";
      return attempt;
    }
    bus_degree.emplace(bus.index, 0);
  }
  if (slack_count != 1) {
    attempt.eligible = true;
    attempt.fail_close_reason = "local_triangle_trial_requires_exactly_one_slack_bus";
    return attempt;
  }

  attempt.eligible = true;

  std::set<std::pair<int, int>> undirected_edges;
  for (const auto& branch : ac.branches) {
    if (!branch.in_service) {
      attempt.eligible = true;
      attempt.fail_close_reason = "out_of_service_branch_present_in_local_triangle_trial";
      return attempt;
    }
    const auto edge = std::minmax(branch.from_bus, branch.to_bus);
    if (!undirected_edges.insert(edge).second) {
      attempt.eligible = true;
      attempt.fail_close_reason = "parallel_branch_not_supported_for_local_triangle_trial";
      return attempt;
    }
    bus_degree[branch.from_bus] += 1;
    bus_degree[branch.to_bus] += 1;
  }
  if (isolated_triangle_shape) {
    if (undirected_edges.size() != 3) {
      return attempt;
    }
    for (const auto& [bus_id, degree] : bus_degree) {
      (void)bus_id;
      if (degree != 2) {
        return attempt;
      }
    }
  } else {
    const int extra_branch_pos = extra_branch_positions.front();
    const auto& extra_branch = ac.branches[static_cast<std::size_t>(extra_branch_pos)];
    const int leaf_bus_id = extra_bus_ids.front();
    const bool connects_leaf =
        extra_branch.from_bus == leaf_bus_id || extra_branch.to_bus == leaf_bus_id;
    const int attached_triangle_bus =
        extra_branch.from_bus == leaf_bus_id ? extra_branch.to_bus : extra_branch.from_bus;
    if (!connects_leaf) {
      attempt.fail_close_reason =
          "embedded_transformer3w_trial_requires_single_leaf_bus_on_extra_branch";
      return attempt;
    }
    if (!triangle_bus_ids.contains(attached_triangle_bus)) {
      attempt.fail_close_reason =
          "embedded_transformer3w_trial_extra_branch_must_attach_to_triangle_bus";
      return attempt;
    }
    if (attached_triangle_bus != slack_bus_id) {
      attempt.fail_close_reason =
          "embedded_transformer3w_trial_requires_extra_branch_on_slack_side";
      return attempt;
    }
    if (bus_degree[leaf_bus_id] != 1) {
      attempt.fail_close_reason =
          "embedded_transformer3w_trial_requires_single_leaf_bus_degree";
      return attempt;
    }
    for (const int triangle_bus_id : triangle_bus_ids) {
      const int expected_degree =
          triangle_bus_id == slack_bus_id ? 3 : 2;
      if (bus_degree[triangle_bus_id] != expected_degree) {
        attempt.fail_close_reason =
            "embedded_transformer3w_trial_requires_one_slack_side_leaf_and_triangle_core";
        return attempt;
      }
    }
    attempt.path_id = kTransformer3WSlackLeafEmbedTrialPathId;
  }

  const double base_mva = (ac.base_mva > 0.0) ? ac.base_mva : 100.0;
  const auto id_map = build_id_map(ac.buses);
  int root = -1;
  for (int i = 0; i < static_cast<int>(ac.buses.size()); ++i) {
    if (ac.buses[static_cast<std::size_t>(i)].bus_type == BusType::SLACK) {
      root = i;
      break;
    }
  }
  if (root < 0) {
    attempt.fail_close_reason = "slack_bus_missing_after_local_triangle_detection";
    return attempt;
  }

  std::vector<int> pq_positions;
  for (int i = 0; i < static_cast<int>(ac.buses.size()); ++i) {
    if (i != root) {
      pq_positions.push_back(i);
    }
  }
  const std::vector<Complex> net_demand =
      compute_net_injection(ac, id_map, root, static_cast<int>(ac.buses.size()),
                            base_mva, opt.include_shunts);
  std::vector<Complex> specified_injection(net_demand.size(), Complex{0.0, 0.0});
  for (std::size_t i = 0; i < net_demand.size(); ++i) {
    specified_injection[i] = -net_demand[i];
  }

  const int n = static_cast<int>(ac.buses.size());
  std::vector<std::vector<Complex>> ybus(
      static_cast<std::size_t>(n),
      std::vector<Complex>(static_cast<std::size_t>(n), Complex{0.0, 0.0}));
  for (const auto& branch : ac.branches) {
    const int from = id_map.at(branch.from_bus);
    const int to = id_map.at(branch.to_bus);
    const Complex z(branch.r_pu, branch.x_pu);
    if (std::abs(z) < 1e-20) {
      attempt.fail_close_reason = "zero_impedance_branch_not_supported_for_local_triangle_trial";
      return attempt;
    }
    const Complex y = 1.0 / z;
    const Complex tap = branch_complex_tap(branch);
    ybus[static_cast<std::size_t>(from)][static_cast<std::size_t>(from)] +=
        y / (tap * std::conj(tap));
    ybus[static_cast<std::size_t>(from)][static_cast<std::size_t>(to)] +=
        -y / std::conj(tap);
    ybus[static_cast<std::size_t>(to)][static_cast<std::size_t>(from)] +=
        -y / tap;
    ybus[static_cast<std::size_t>(to)][static_cast<std::size_t>(to)] += y;
  }

  std::vector<Complex> voltages(static_cast<std::size_t>(n), Complex{1.0, 0.0});
  for (int i = 0; i < n; ++i) {
    voltages[static_cast<std::size_t>(i)] = std::polar(
        ac.buses[static_cast<std::size_t>(i)].vm_pu,
        ac.buses[static_cast<std::size_t>(i)].va_deg * (kPi / 180.0));
  }

  auto mismatch = [&](const std::vector<Complex>& v) {
    std::vector<double> residual;
    residual.reserve(2 * pq_positions.size());
    for (const int pos : pq_positions) {
      Complex current{0.0, 0.0};
      for (int col = 0; col < n; ++col) {
        current += ybus[static_cast<std::size_t>(pos)][static_cast<std::size_t>(col)] *
                   v[static_cast<std::size_t>(col)];
      }
      const Complex calculated = v[static_cast<std::size_t>(pos)] * std::conj(current);
      const Complex delta = calculated - specified_injection[static_cast<std::size_t>(pos)];
      residual.push_back(delta.real());
      residual.push_back(delta.imag());
    }
    return residual;
  };
  auto inf_norm = [](const std::vector<double>& values) {
    double max_abs = 0.0;
    for (const double value : values) {
      max_abs = std::max(max_abs, std::abs(value));
    }
    return max_abs;
  };

  double residual = std::numeric_limits<double>::max();
  int iterations = 0;
  bool converged = false;
  for (; iterations < opt.max_iter; ++iterations) {
    const std::vector<double> f = mismatch(voltages);
    residual = inf_norm(f);
    if (residual <= opt.tol) {
      converged = true;
      break;
    }

    const std::size_t state_dim = 2 * pq_positions.size();
    std::vector<std::vector<double>> augmented(
        state_dim, std::vector<double>(state_dim + 1, 0.0));
    const double eps = 1e-8;
    for (std::size_t k = 0; k < state_dim; ++k) {
      auto perturbed = voltages;
      const int bus_pos = pq_positions[k / 2];
      if ((k % 2) == 0) {
        perturbed[static_cast<std::size_t>(bus_pos)] += Complex{eps, 0.0};
      } else {
        perturbed[static_cast<std::size_t>(bus_pos)] += Complex{0.0, eps};
      }
      const std::vector<double> f_perturbed = mismatch(perturbed);
      for (std::size_t row = 0; row < state_dim; ++row) {
        augmented[row][k] = (f_perturbed[row] - f[row]) / eps;
      }
      augmented[k][state_dim] = -f[k];
    }

    for (std::size_t row = 0; row < state_dim; ++row) {
      augmented[row][state_dim] = -f[row];
    }

    for (std::size_t pivot = 0; pivot < state_dim; ++pivot) {
      std::size_t best = pivot;
      for (std::size_t row = pivot + 1; row < state_dim; ++row) {
        if (std::abs(augmented[row][pivot]) > std::abs(augmented[best][pivot])) {
          best = row;
        }
      }
      if (std::abs(augmented[best][pivot]) < 1e-12) {
        attempt.fail_close_reason = "local_triangle_trial_jacobian_singular";
        return attempt;
      }
      std::swap(augmented[pivot], augmented[best]);
      const double diag = augmented[pivot][pivot];
      for (std::size_t col = pivot; col <= state_dim; ++col) {
        augmented[pivot][col] /= diag;
      }
      for (std::size_t row = 0; row < state_dim; ++row) {
        if (row == pivot) {
          continue;
        }
        const double factor = augmented[row][pivot];
        for (std::size_t col = pivot; col <= state_dim; ++col) {
          augmented[row][col] -= factor * augmented[pivot][col];
        }
      }
    }

    double max_state_delta = 0.0;
    for (std::size_t k = 0; k < state_dim; ++k) {
      const int bus_pos = pq_positions[k / 2];
      const double delta = augmented[k][state_dim];
      max_state_delta = std::max(max_state_delta, std::abs(delta));
      if ((k % 2) == 0) {
        voltages[static_cast<std::size_t>(bus_pos)] += Complex{delta, 0.0};
      } else {
        voltages[static_cast<std::size_t>(bus_pos)] += Complex{0.0, delta};
      }
    }
    residual = max_state_delta;
    if (residual <= opt.tol) {
      converged = true;
      ++iterations;
      break;
    }
  }

  if (!converged) {
    attempt.fail_close_reason = "local_triangle_trial_not_converged";
    return attempt;
  }

  DPFResult result;
  result.vm_pu.assign(static_cast<std::size_t>(n), 1.0);
  result.va_deg.assign(static_cast<std::size_t>(n), 0.0);
  result.p_branch_mw.assign(ac.branches.size(), 0.0);
  result.q_branch_mvar.assign(ac.branches.size(), 0.0);
  result.specialized_path_applied = true;
  result.specialized_path_id = attempt.path_id;
  result.control_converged = true;
  result.converged = true;
  result.iterations = iterations;
  result.residual = residual;

  double total_p_loss = 0.0;
  double total_q_loss = 0.0;
  for (int i = 0; i < n; ++i) {
    result.vm_pu[static_cast<std::size_t>(i)] =
        std::abs(voltages[static_cast<std::size_t>(i)]);
    result.va_deg[static_cast<std::size_t>(i)] =
        std::arg(voltages[static_cast<std::size_t>(i)]) * (180.0 / kPi);
  }
  for (std::size_t branch_pos = 0; branch_pos < ac.branches.size(); ++branch_pos) {
    const auto& branch = ac.branches[branch_pos];
    const int from = id_map.at(branch.from_bus);
    const int to = id_map.at(branch.to_bus);
    const auto flow = compute_single_phase_branch_flow(
        branch,
        voltages[static_cast<std::size_t>(from)],
        voltages[static_cast<std::size_t>(to)],
        base_mva);
    result.p_branch_mw[branch_pos] = flow.power_from.real();
    result.q_branch_mvar[branch_pos] = flow.power_from.imag();
    total_p_loss += flow.loss.real();
    total_q_loss += flow.loss.imag();
  }
  result.total_p_loss_mw = total_p_loss;
  result.total_q_loss_mvar = total_q_loss;

  attempt.applied = true;
  attempt.snapshot = {
      .result = std::move(result),
      .voltages = std::move(voltages),
      .id_map = id_map,
  };
  return attempt;
}

}  // anonymous namespace

// =========================================================================
// solve_distribution_pf
// =========================================================================
DPFResult solve_distribution_pf(const ACSystem& ac_sys,
                                 const DPFOptions& opt) {
  return solve_distribution_pf_ac_snapshot(ac_sys, opt).result;
}

DPFResult solve_distribution_pf(const HybridPowerSystem& sys,
                                 const DPFOptions& opt) {
  auto unproject_if_needed = [](DPFResult& result, const HybridPowerSystem& projected) {
    if (projected.bus_merge_map && projected.bus_merge_map->has_merges()) {
      result.vm_pu = unproject_bus_vector(result.vm_pu, *projected.bus_merge_map);
      result.va_deg = unproject_bus_vector(result.va_deg, *projected.bus_merge_map);
    }
  };
  const bool has_enabled_regulator =
      std::any_of(sys.ac.regulator_controls.begin(), sys.ac.regulator_controls.end(),
                  [](const auto& control) { return control.enabled; });
  auto finalize_result = [&](DPFResult result, const HybridPowerSystem& projected) {
    unproject_if_needed(result, projected);
    return result;
  };

  if (!has_enabled_regulator) {
    const HybridPowerSystem projected = project_to_canonical_models(sys);
    const auto local_triangle_trial =
        try_solve_transformer3w_isolated_triangle_snapshot(projected, opt);
    DPFResult result;
    if (local_triangle_trial.applied) {
      result = local_triangle_trial.snapshot.result;
    } else {
      result = solve_distribution_pf_ac_snapshot(projected.ac, opt).result;
      if (local_triangle_trial.eligible) {
        result.specialized_path_id = local_triangle_trial.path_id;
        result.specialized_path_fail_close_reason =
            local_triangle_trial.fail_close_reason;
      }
    }
    unproject_if_needed(result, projected);
    return result;
  }

  HybridPowerSystem working = sys;
  std::vector<RegulatorControlTraceEntry> accumulated_trace;
  std::set<std::vector<int>> seen_tap_states;

  auto capture_tap_state = [&working]() {
    std::vector<int> state;
    state.reserve(working.ac.regulator_controls.size());
    for (const auto& control : working.ac.regulator_controls) {
      if (!control.enabled) continue;
      const int transformer_pos = find_transformer_position_by_index(
          working.ac.transformers_2w, control.transformer_index);
      state.push_back(working.ac.transformers_2w[static_cast<std::size_t>(transformer_pos)].tap_pos);
    }
    return state;
  };
  seen_tap_states.insert(capture_tap_state());

  DPFResult final_result;
  bool finalized = false;

  for (int control_iter = 1; control_iter <= opt.max_control_iter; ++control_iter) {
    const HybridPowerSystem projected = project_to_canonical_models(working);
    auto snapshot = solve_distribution_pf_ac_snapshot(projected.ac, opt);
    DPFResult current_result = snapshot.result;
    std::vector<RegulatorControlState> regulator_states;
    regulator_states.reserve(working.ac.regulator_controls.size());

    bool any_tap_changed = false;
    bool control_failed = !snapshot.result.converged;
    std::vector<std::size_t> iteration_trace_positions;

    for (const auto& control : working.ac.regulator_controls) {
      if (!control.enabled) continue;

      RegulatorControlState state;
      state.regulator_name = control.name;
      state.transformer_name = control.transformer_name;
      state.transformer_index = control.transformer_index;
      state.winding = control.winding;
      state.tap_winding = control.tap_winding;
      state.monitored_bus = control.monitored_bus;
      state.monitored_node = control.monitored_node;
      state.used_remote_bus = (control.monitored_bus != 0);
      state.used_line_drop_compensation =
          std::abs(control.r_volts) > 1e-12 || std::abs(control.x_volts) > 1e-12;
      state.target_vreg_volts = control.vreg_volts;
      state.band_volts = control.band_volts;
      state.ptratio = control.ptratio;
      state.remote_ptratio = (control.remote_ptratio > 0.0) ? control.remote_ptratio
                                                             : control.ptratio;
      state.ct_primary_amps = control.ct_primary_amps;
      state.r_volts = control.r_volts;
      state.x_volts = control.x_volts;
      state.max_tap_change = control.max_tap_change;
      state.control_iterations = control_iter;

      const std::size_t trace_pos = accumulated_trace.size();
      iteration_trace_positions.push_back(trace_pos);
      accumulated_trace.push_back(RegulatorControlTraceEntry{
          .regulator_name = control.name,
          .iteration = control_iter,
          .target_vreg_volts = control.vreg_volts,
          .band_half_volts = control.band_volts / 2.0,
          .decision_reason = "",
          .stop_reason = "",
      });
      auto& trace = accumulated_trace.back();

      try {
        const int transformer_pos = find_transformer_position_by_index(
            working.ac.transformers_2w, control.transformer_index);
        const auto& transformer =
            working.ac.transformers_2w[static_cast<std::size_t>(transformer_pos)];
        trace.current_tap_pos = transformer.tap_pos;
        trace.current_tap_number = transformer_tap_number(transformer);
        trace.current_tap_pu = transformer_tap_pu(transformer);

        if (control.reversible) {
          throw std::runtime_error("unsupported_reversible_control");
        }
        if (control.winding < 1 || control.winding > 2 ||
            control.tap_winding < 1 || control.tap_winding > 2) {
          throw std::runtime_error("unsupported_winding_reference");
        }
        if (control.monitored_node != 1) {
          throw std::runtime_error("unsupported_monitored_node");
        }
        if (!tap_side_matches_winding(transformer, control.tap_winding)) {
          throw std::runtime_error("tap_side_tap_winding_mismatch");
        }
        if (transformer.tap_max < transformer.tap_min ||
            std::abs(transformer.tap_step_percent) < 1e-12) {
          throw std::runtime_error("transformer_missing_discrete_tap_grid");
        }
        if (control.max_tap_change < 1) {
          throw std::runtime_error("unsupported_max_tap_change");
        }
        if (control.monitored_bus != 0 &&
            (std::abs(control.r_volts) > 1e-12 || std::abs(control.x_volts) > 1e-12)) {
          throw std::runtime_error("unsupported_remote_bus_with_ldc");
        }

        const int monitored_bus = (control.monitored_bus != 0)
                                      ? control.monitored_bus
                                      : transformer_winding_bus(transformer, control.winding);
        const int monitored_bus_pos =
            projected_bus_position_for_original_bus(projected, monitored_bus);
        const auto& monitored_ac_bus =
            projected.ac.buses[static_cast<std::size_t>(monitored_bus_pos)];
        const double monitored_base_volts = bus_base_voltage_volts(monitored_ac_bus);
        const Complex monitored_voltage =
            snapshot.voltages[static_cast<std::size_t>(monitored_bus_pos)] *
            monitored_base_volts;
        const double monitored_voltage_pu = std::abs(
            snapshot.voltages[static_cast<std::size_t>(monitored_bus_pos)]);
        const double monitored_voltage_volts = std::abs(monitored_voltage);
        const double effective_ptratio =
            (control.monitored_bus != 0 && control.remote_ptratio > 0.0)
                ? control.remote_ptratio
                : control.ptratio;
        if (effective_ptratio <= 1e-12) {
          throw std::runtime_error("invalid_ptratio");
        }

        Complex ldc_term{0.0, 0.0};
        if (std::abs(control.r_volts) > 1e-12 || std::abs(control.x_volts) > 1e-12) {
          if (control.ct_primary_amps <= 1e-12) {
            throw std::runtime_error("invalid_ct_primary_amps");
          }
          const int transformer_branch_pos = find_projected_branch_position(
              projected, BranchOriginType::Transformer2W, transformer.index);
          const auto& branch =
              projected.ac.branches[static_cast<std::size_t>(transformer_branch_pos)];
          auto it_f = snapshot.id_map.find(branch.from_bus);
          auto it_t = snapshot.id_map.find(branch.to_bus);
          if (it_f == snapshot.id_map.end() || it_t == snapshot.id_map.end()) {
            throw std::runtime_error("projected_branch_bus_lookup_failed");
          }
          const int fi = it_f->second;
          const int ti = it_t->second;
          const auto flow = compute_single_phase_branch_flow(
              branch,
              snapshot.voltages[static_cast<std::size_t>(fi)],
              snapshot.voltages[static_cast<std::size_t>(ti)],
              projected.ac.base_mva);
          const Complex winding_current_pu =
              (control.tap_winding == 1) ? flow.current_from : flow.current_to;
          const int current_bus =
              transformer_winding_bus(transformer, control.tap_winding);
          const int current_bus_pos =
              projected_bus_position_for_original_bus(projected, current_bus);
          const double base_current_amps =
              projected.ac.base_mva * 1000.0 /
              std::max(1e-9, projected.ac.buses[static_cast<std::size_t>(current_bus_pos)].base_kv);
          const Complex current_amps = winding_current_pu * base_current_amps;
          ldc_term = (current_amps / control.ct_primary_amps) *
                     Complex(control.r_volts, control.x_volts);
        }

        const Complex control_voltage = monitored_voltage / effective_ptratio + ldc_term;
        const double control_voltage_volts = std::abs(control_voltage);
        const double band_half = control.band_volts / 2.0;
        const double lower_band = control.vreg_volts - band_half;
        const double upper_band = control.vreg_volts + band_half;

        state.final_tap_pos = transformer.tap_pos;
        state.final_tap_number = transformer_tap_number(transformer);
        state.final_tap_pu = transformer_tap_pu(transformer);
        state.monitored_voltage_pu = monitored_voltage_pu;
        state.monitored_voltage_volts = monitored_voltage_volts;
        state.control_voltage_volts = control_voltage_volts;
        state.line_drop_compensation_real_volts = ldc_term.real();
        state.line_drop_compensation_imag_volts = ldc_term.imag();
        state.line_drop_compensation_magnitude_volts = std::abs(ldc_term);

        trace.monitored_voltage_pu = monitored_voltage_pu;
        trace.monitored_voltage_volts = monitored_voltage_volts;
        trace.control_voltage_volts = control_voltage_volts;
        trace.line_drop_compensation_real_volts = ldc_term.real();
        trace.line_drop_compensation_imag_volts = ldc_term.imag();
        trace.line_drop_compensation_magnitude_volts = std::abs(ldc_term);

        int next_tap_pos = transformer.tap_pos;
        if (control_voltage_volts < lower_band - 1e-6) {
          trace.decision_reason = "raise_voltage";
          const int tap_delta =
              tap_pos_delta_for_raise_voltage(transformer) *
              std::min(control.max_tap_change, std::abs(transformer.tap_max - transformer.tap_min));
          next_tap_pos = std::clamp(
              transformer.tap_pos + tap_delta, transformer.tap_min, transformer.tap_max);
          if (next_tap_pos == transformer.tap_pos) {
            throw std::runtime_error("tap_limit_reached_below_band");
          }
        } else if (control_voltage_volts > upper_band + 1e-6) {
          trace.decision_reason = "lower_voltage";
          const int tap_delta =
              -tap_pos_delta_for_raise_voltage(transformer) *
              std::min(control.max_tap_change, std::abs(transformer.tap_max - transformer.tap_min));
          next_tap_pos = std::clamp(
              transformer.tap_pos + tap_delta, transformer.tap_min, transformer.tap_max);
          if (next_tap_pos == transformer.tap_pos) {
            throw std::runtime_error("tap_limit_reached_above_band");
          }
        } else {
          trace.decision_reason = "within_band";
          trace.next_tap_pos = transformer.tap_pos;
          trace.next_tap_number = transformer_tap_number(transformer);
          trace.stop_reason = "within_band";
          state.converged = true;
          state.stop_reason = "within_band";
          regulator_states.push_back(state);
          continue;
        }

        trace.next_tap_pos = next_tap_pos;
        trace.next_tap_number = next_tap_pos - transformer.tap_neutral;
        trace.stop_reason = "continue_control_loop";
        state.converged = false;
        state.stop_reason = "tap_change_requested";

        working.ac.transformers_2w[static_cast<std::size_t>(transformer_pos)].tap_pos = next_tap_pos;
        any_tap_changed = true;
      } catch (const std::exception& ex) {
        control_failed = true;
        trace.decision_reason = "blocked";
        trace.next_tap_pos = trace.current_tap_pos;
        trace.next_tap_number = trace.current_tap_number;
        trace.stop_reason = ex.what();
        state.converged = false;
        state.stop_reason = ex.what();
      }

      regulator_states.push_back(state);
    }

    current_result.regulator_trace = accumulated_trace;
    current_result.regulator_states = regulator_states;

    if (control_failed) {
      current_result.control_converged = false;
      current_result.converged = false;
      final_result = finalize_result(std::move(current_result), projected);
      finalized = true;
      break;
    }

    if (!any_tap_changed) {
      current_result.control_converged = true;
      final_result = finalize_result(std::move(current_result), projected);
      finalized = true;
      break;
    }

    const auto next_tap_state = capture_tap_state();
    if (!seen_tap_states.insert(next_tap_state).second) {
      for (const auto trace_pos : iteration_trace_positions) {
        if (accumulated_trace[trace_pos].stop_reason == "continue_control_loop") {
          accumulated_trace[trace_pos].stop_reason = "cycle_detected";
        }
      }
      for (auto& state : current_result.regulator_states) {
        if (state.stop_reason == "tap_change_requested") {
          state.stop_reason = "cycle_detected";
          state.converged = false;
        }
      }
      current_result.regulator_trace = accumulated_trace;
      current_result.control_converged = false;
      current_result.converged = false;
      final_result = finalize_result(std::move(current_result), projected);
      finalized = true;
      break;
    }

    if (control_iter == opt.max_control_iter) {
      for (const auto trace_pos : iteration_trace_positions) {
        if (accumulated_trace[trace_pos].stop_reason == "continue_control_loop") {
          accumulated_trace[trace_pos].stop_reason = "max_control_iter_reached";
        }
      }
      for (auto& state : current_result.regulator_states) {
        if (state.stop_reason == "tap_change_requested") {
          state.stop_reason = "max_control_iter_reached";
          state.converged = false;
        }
      }
      current_result.regulator_trace = accumulated_trace;
      current_result.control_converged = false;
      current_result.converged = false;
      final_result = finalize_result(std::move(current_result), projected);
      finalized = true;
      break;
    }
  }

  if (!finalized) {
    throw std::runtime_error("solve_distribution_pf: regulator control loop did not finalize");
  }

  return final_result;
}

// =========================================================================
// Three-Phase BFS — abc-domain backward-forward sweep
// =========================================================================
//
// Mathematical formulation:
//
// Each bus carries a 3-vector of complex voltages V = [Va, Vb, Vc]^T.
// Each line has a 3×3 series impedance matrix Z_abc derived from
// sequence parameters via the Fortescue-to-abc transformation:
//
//   Z_abc = A · diag(Z0, Z1, Z1) · A^{-1}
//         = [ Zs  Zm  Zm ]      Zs = (Z0 + 2·Z1) / 3
//           [ Zm  Zs  Zm ]      Zm = (Z0 - Z1) / 3
//           [ Zm  Zm  Zs ]
//
// where Z1 = r1 + j·x1 (positive-sequence) and Z0 = r0 + j·x0 (zero-sequence).
// When Z0 = Z1 (balanced lines), Zm = 0 and the three phases decouple.
//
// Net injection per bus: S_abc[i] = [Sa, Sb, Sc] (load − generation, per-phase).
//
// Backward sweep:
//   I_abc[j] = conj(S_abc[j] ./ V_abc[j])    (per-phase injection)
//   Accumulate child branch currents into parent.
//
// Forward sweep:
//   V_abc[j] = V_abc[parent] − Z_abc · I_abc[branch]
//
// Convergence: max |ΔV| across all phases and buses.
// =========================================================================

namespace {

using C3 = std::array<std::complex<double>, 3>;   // per-phase voltage/current/power
using Z33 = std::array<std::array<std::complex<double>, 3>, 3>;  // 3×3 impedance

// Multiply 3×3 matrix by 3-vector: result = Z * I
C3 mat3_mul(const Z33& Z, const C3& I) {
  C3 out{};
  for (int r = 0; r < 3; ++r)
    out[r] = Z[r][0] * I[0] + Z[r][1] * I[1] + Z[r][2] * I[2];
  return out;
}

// Build 3×3 series impedance from sequence parameters:
//   Zs = (Z0 + 2*Z1) / 3,  Zm = (Z0 - Z1) / 3
Z33 build_z_abc(std::complex<double> Z1, std::complex<double> Z0) {
  std::complex<double> Zs = (Z0 + 2.0 * Z1) / 3.0;
  std::complex<double> Zm = (Z0 - Z1) / 3.0;
  Z33 Z{};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      Z[r][c] = (r == c) ? Zs : Zm;
  return Z;
}

struct ThreePhaseBFSTree {
  std::vector<int> order;
  std::vector<int> parent_bus;
  std::vector<int> parent_line;
};

ThreePhaseBFSTree build_tp_bfs_tree(
    const std::vector<ThreePhaseACBus>& buses,
    const std::vector<ThreePhaseACLine>& lines,
    const std::unordered_map<int, int>& id_map,
    int root_local) {
  const int n = static_cast<int>(buses.size());
  std::vector<std::vector<std::pair<int, int>>> adj(n);
  for (int b = 0; b < static_cast<int>(lines.size()); ++b) {
    if (!lines[b].in_service) continue;
    auto it_f = id_map.find(lines[b].from_bus);
    auto it_t = id_map.find(lines[b].to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    adj[it_f->second].emplace_back(it_t->second, b);
    adj[it_t->second].emplace_back(it_f->second, b);
  }

  ThreePhaseBFSTree tree;
  tree.order.reserve(n);
  tree.parent_bus.assign(n, -1);
  tree.parent_line.assign(n, -1);

  std::vector<bool> visited(n, false);
  std::queue<int> q;
  q.push(root_local);
  visited[root_local] = true;

  while (!q.empty()) {
    int u = q.front(); q.pop();
    tree.order.push_back(u);
    for (auto [v, b] : adj[u]) {
      if (!visited[v]) {
        visited[v] = true;
        tree.parent_bus[v] = u;
        tree.parent_line[v] = b;
        q.push(v);
      }
    }
  }
  return tree;
}

// Compute per-phase net injection at each bus.
// Returns S_net[bus_local] = {Sa, Sb, Sc} in per-unit (load − generation).
std::vector<C3> compute_tp_net_injection(
    const ThreePhaseACSystem& sys,
    const std::unordered_map<int, int>& id_map,
    int root, int n, double base_mva, bool include_shunts) {

  std::vector<C3> S_net(n, C3{{{0, 0}, {0, 0}, {0, 0}}});

  // Bus-level demand
  for (int i = 0; i < n; ++i) {
    if (!sys.buses[i].in_service) continue;
    const auto& b = sys.buses[i];
    S_net[i][0] += std::complex<double>(b.pd_a_mw / base_mva, b.qd_a_mvar / base_mva);
    S_net[i][1] += std::complex<double>(b.pd_b_mw / base_mva, b.qd_b_mvar / base_mva);
    S_net[i][2] += std::complex<double>(b.pd_c_mw / base_mva, b.qd_c_mvar / base_mva);
  }

  // Three-phase load table
  for (const auto& ld : sys.loads) {
    if (!ld.in_service) continue;
    auto it = id_map.find(ld.bus);
    if (it == id_map.end()) continue;
    const int idx = it->second;
    S_net[idx][0] += std::complex<double>(ld.p_a_mw / base_mva, ld.q_a_mvar / base_mva);
    S_net[idx][1] += std::complex<double>(ld.p_b_mw / base_mva, ld.q_b_mvar / base_mva);
    S_net[idx][2] += std::complex<double>(ld.p_c_mw / base_mva, ld.q_c_mvar / base_mva);
  }

  // Bus-level shunts
  if (include_shunts) {
    for (int i = 0; i < n; ++i) {
      if (!sys.buses[i].in_service) continue;
      const auto& b = sys.buses[i];
      S_net[i][0] += std::complex<double>(b.gs_a_mw / base_mva, -b.bs_a_mvar / base_mva);
      S_net[i][1] += std::complex<double>(b.gs_b_mw / base_mva, -b.bs_b_mvar / base_mva);
      S_net[i][2] += std::complex<double>(b.gs_c_mw / base_mva, -b.bs_c_mvar / base_mva);
    }
  }

  // Subtract generation (skip root/slack).  3-phase generators are balanced:
  // P/Q split equally across three phases.
  for (const auto& g : sys.generators) {
    if (!g.in_service) continue;
    auto it = id_map.find(g.bus);
    if (it == id_map.end()) continue;
    const int gi = it->second;
    if (gi == root) continue;
    std::complex<double> sg(g.p_mw / (3.0 * base_mva), g.q_mvar / (3.0 * base_mva));
    S_net[gi][0] -= sg;
    S_net[gi][1] -= sg;
    S_net[gi][2] -= sg;
  }

  return S_net;
}

// Run three-phase BFS inner loop.
struct ThreePhaseBFSInnerResult {
  double residual{0.0};
  int iterations{0};
  bool converged{false};
};

ThreePhaseBFSInnerResult run_tp_bfs_sweep(
    const std::vector<ThreePhaseACLine>& lines,
    const std::vector<Z33>& Z_abc,
    const ThreePhaseBFSTree& tree,
    const std::vector<C3>& S_net,
    std::vector<C3>& V,
    int root, int n, int m,
    const DPFOptions& opt) {
  (void)lines;

  std::vector<C3> I_branch(m, C3{{{0, 0}, {0, 0}, {0, 0}}});
  double residual = std::numeric_limits<double>::max();
  int iter = 0;

  for (; iter < opt.max_iter; ++iter) {
    // Zero branch currents
    for (int b = 0; b < m; ++b)
      I_branch[b] = C3{{{0, 0}, {0, 0}, {0, 0}}};

    // Backward sweep: compute per-phase injection currents
    for (int k = 1; k < static_cast<int>(tree.order.size()); ++k) {
      int j = tree.order[k];
      int b = tree.parent_line[j];
      for (int p = 0; p < 3; ++p) {
        if (std::abs(V[j][p]) > 1e-12)
          I_branch[b][p] = std::conj(S_net[j][p] / V[j][p]);
        else
          I_branch[b][p] = {0.0, 0.0};
      }
    }

    // Accumulate child currents into parent branch (reverse BFS order)
    for (int k = static_cast<int>(tree.order.size()) - 1; k >= 1; --k) {
      int j = tree.order[k];
      int i = tree.parent_bus[j];
      int bj = tree.parent_line[j];
      if (i != root) {
        int bi = tree.parent_line[i];
        for (int p = 0; p < 3; ++p)
          I_branch[bi][p] += I_branch[bj][p];
      }
    }

    // Forward sweep: update per-phase voltages using 3×3 impedance
    std::vector<C3> V_new = V;
    V_new[root] = V[root];  // root voltage is fixed
    for (int k = 1; k < static_cast<int>(tree.order.size()); ++k) {
      int j = tree.order[k];
      int i = tree.parent_bus[j];
      int b = tree.parent_line[j];
      C3 dV = mat3_mul(Z_abc[b], I_branch[b]);
      for (int p = 0; p < 3; ++p)
        V_new[j][p] = V_new[i][p] - dV[p];
    }

    // Convergence check: max |ΔV| across all phases and buses
    residual = 0.0;
    for (int i = 0; i < n; ++i)
      for (int p = 0; p < 3; ++p)
        residual = std::max(residual, std::abs(V_new[i][p] - V[i][p]));

    V = V_new;

    if (opt.verbose)
      HACDCPF_LOG_DEBUG("  3ph-DPF iter {}  residual = {}", iter + 1, residual);

    if (residual < opt.tol) { ++iter; break; }
  }

  return {residual, iter, residual < opt.tol};
}

}  // anonymous namespace (three-phase helpers)

// =========================================================================
// solve_three_phase_distribution_pf (ThreePhaseACSystem)
// =========================================================================
ThreePhaseDPFResult solve_three_phase_distribution_pf(
    const ThreePhaseACSystem& sys,
    const DPFOptions& opt) {

  const auto& buses = sys.buses;
  const auto& lines = sys.lines;
  const double base_mva = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;

  const int n = static_cast<int>(buses.size());
  const int m = static_cast<int>(lines.size());

  ThreePhaseDPFResult result;
  result.bus_voltages.resize(n);
  result.branch_powers.resize(m);

  if (n == 0) return result;

  // Bus ID → local index map
  std::unordered_map<int, int> id_map;
  id_map.reserve(n);
  for (int i = 0; i < n; ++i) id_map[buses[i].index] = i;

  // Find SLACK root bus
  int root = -1;
  for (int i = 0; i < n; ++i)
    if (buses[i].bus_type == BusType::SLACK) { root = i; break; }
  if (root < 0)
    throw std::runtime_error("solve_three_phase_distribution_pf: no SLACK bus found");

  // Build BFS tree
  const auto tree = build_tp_bfs_tree(buses, lines, id_map, root);

  // Compute per-phase net injection
  auto S_net = compute_tp_net_injection(sys, id_map, root, n, base_mva, opt.include_shunts);

  // Build 3×3 impedance matrices for each line
  std::vector<Z33> Z_abc(m);
  for (int b = 0; b < m; ++b) {
    std::complex<double> Z1(lines[b].r1_pu, lines[b].x1_pu);
    std::complex<double> Z0(lines[b].r0_pu, lines[b].x0_pu);
    // If zero-sequence is not specified, assume Z0 = Z1 (balanced)
    if (std::abs(Z0) < 1e-20) Z0 = Z1;
    Z_abc[b] = build_z_abc(Z1, Z0);
  }

  // Initialise per-phase voltages: balanced flat start
  // Phase a: 1∠0°, Phase b: 1∠−120°, Phase c: 1∠+120°
  const double deg2rad = kPi / 180.0;
  std::vector<C3> V(n);
  for (int i = 0; i < n; ++i) {
    double vm_a = buses[i].vm_a_pu;
    double vm_b = buses[i].vm_b_pu;
    double vm_c = buses[i].vm_c_pu;
    V[i][0] = std::polar(vm_a, buses[i].va_a_deg * deg2rad);
    V[i][1] = std::polar(vm_b, buses[i].va_b_deg * deg2rad);
    V[i][2] = std::polar(vm_c, buses[i].va_c_deg * deg2rad);
  }

  // Run three-phase BFS
  auto bfs_result = run_tp_bfs_sweep(lines, Z_abc, tree, S_net, V, root, n, m, opt);

  result.converged  = bfs_result.converged;
  result.iterations = bfs_result.iterations;
  result.residual   = bfs_result.residual;

  // Extract per-phase bus voltages
  const double rad2deg = 180.0 / kPi;
  for (int i = 0; i < n; ++i) {
    auto& bv = result.bus_voltages[i];
    bv.bus_id = buses[i].index;
    bv.vm_a_pu = std::abs(V[i][0]);  bv.va_a_deg = std::arg(V[i][0]) * rad2deg;
    bv.vm_b_pu = std::abs(V[i][1]);  bv.va_b_deg = std::arg(V[i][1]) * rad2deg;
    bv.vm_c_pu = std::abs(V[i][2]);  bv.va_c_deg = std::arg(V[i][2]) * rad2deg;
  }

  // Compute per-phase branch power and losses
  double p_loss_a = 0.0, q_loss_a = 0.0;
  double p_loss_b = 0.0, q_loss_b = 0.0;
  double p_loss_c = 0.0, q_loss_c = 0.0;

  for (int b = 0; b < m; ++b) {
    if (!lines[b].in_service) continue;
    auto it_f = id_map.find(lines[b].from_bus);
    auto it_t = id_map.find(lines[b].to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second, ti = it_t->second;

    auto& bp = result.branch_powers[b];
    bp.line_index = lines[b].index;

    // Per-phase branch current: I_abc = Z_abc^{-1} · (V_from - V_to)
    // For sending-end power: S_p = V_from_p · conj(I_p)
    // Since we have Z_abc as 3x3, compute I = Z^{-1} · dV using the
    // pre-computed impedance structure (Zs, Zm).
    // For efficiency, use the known inverse of the circulant matrix:
    //   Z^{-1} = A · diag(1/Z0, 1/Z1, 1/Z1) · A^{-1}
    //          = [ Ys  Ym  Ym ]   Ys = (1/Z0 + 2/Z1) / 3
    //            [ Ym  Ys  Ym ]   Ym = (1/Z0 - 1/Z1) / 3
    //            [ Ym  Ym  Ys ]
    std::complex<double> Z1(lines[b].r1_pu, lines[b].x1_pu);
    std::complex<double> Z0(lines[b].r0_pu, lines[b].x0_pu);
    if (std::abs(Z0) < 1e-20) Z0 = Z1;

    // Per-phase current via dV / Z (simplified using circulant inverse)
    C3 dV;
    for (int p = 0; p < 3; ++p)
      dV[p] = V[fi][p] - V[ti][p];

    std::complex<double> Ys = (1.0 / Z0 + 2.0 / Z1) / 3.0;
    std::complex<double> Ym = (1.0 / Z0 - 1.0 / Z1) / 3.0;
    C3 I_br;
    for (int p = 0; p < 3; ++p)
      I_br[p] = Ys * dV[p] + Ym * dV[(p + 1) % 3] + Ym * dV[(p + 2) % 3];

    // Sending-end power per phase
    auto S_a = V[fi][0] * std::conj(I_br[0]) * base_mva;
    auto S_b = V[fi][1] * std::conj(I_br[1]) * base_mva;
    auto S_c = V[fi][2] * std::conj(I_br[2]) * base_mva;

    bp.p_a_mw = S_a.real();  bp.q_a_mvar = S_a.imag();
    bp.p_b_mw = S_b.real();  bp.q_b_mvar = S_b.imag();
    bp.p_c_mw = S_c.real();  bp.q_c_mvar = S_c.imag();

    // Losses must include the full Zabc coupling.  Using only |I_p|^2 times the
    // diagonal self impedance drops the mutual-term cross products and
    // overstates losses on unbalanced coupled lines.
    for (int p = 0; p < 3; ++p) {
      const auto phase_loss = dV[p] * std::conj(I_br[p]) * base_mva;
      if (p == 0) { p_loss_a += phase_loss.real(); q_loss_a += phase_loss.imag(); }
      if (p == 1) { p_loss_b += phase_loss.real(); q_loss_b += phase_loss.imag(); }
      if (p == 2) { p_loss_c += phase_loss.real(); q_loss_c += phase_loss.imag(); }
    }
  }

  result.p_loss_a_mw = p_loss_a;   result.q_loss_a_mvar = q_loss_a;
  result.p_loss_b_mw = p_loss_b;   result.q_loss_b_mvar = q_loss_b;
  result.p_loss_c_mw = p_loss_c;   result.q_loss_c_mvar = q_loss_c;
  result.total_p_loss_mw   = p_loss_a + p_loss_b + p_loss_c;
  result.total_q_loss_mvar = q_loss_a + q_loss_b + q_loss_c;

  // Compute max voltage unbalance factor (VUF)
  // VUF = |V_neg| / |V_pos| × 100%
  // Using symmetrical components: V_pos = (Va + a·Vb + a²·Vc)/3
  //                              V_neg = (Va + a²·Vb + a·Vc)/3
  // where a = exp(j·2π/3)
  const std::complex<double> a_op = std::polar(1.0, 2.0 * kPi / 3.0);
  const std::complex<double> a2_op = std::polar(1.0, 4.0 * kPi / 3.0);
  double max_vuf = 0.0;
  for (int i = 0; i < n; ++i) {
    std::complex<double> V_pos = (V[i][0] + a_op * V[i][1] + a2_op * V[i][2]) / 3.0;
    std::complex<double> V_neg = (V[i][0] + a2_op * V[i][1] + a_op * V[i][2]) / 3.0;
    double vpos_mag = std::abs(V_pos);
    if (vpos_mag > 1e-12) {
      double vuf = std::abs(V_neg) / vpos_mag * 100.0;
      max_vuf = std::max(max_vuf, vuf);
    }
  }
  result.max_vuf_percent = max_vuf;

  return result;
}

// =========================================================================
// solve_three_phase_distribution_pf (HybridPowerSystem)
// =========================================================================
ThreePhaseDPFResult solve_three_phase_distribution_pf(
    const HybridPowerSystem& sys,
    const DPFOptions& opt) {
  if (!sys.three_phase_ac.has_value())
    throw std::runtime_error(
        "solve_three_phase_distribution_pf: no three_phase_ac subsystem present");
  return solve_three_phase_distribution_pf(sys.three_phase_ac.value(), opt);
}

namespace {

using Complex = std::complex<double>;

std::string lowercase_ascii_copy(std::string text) {
  std::transform(
      text.begin(),
      text.end(),
      text.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return text;
}

// Parse the clock number from a vector-group string and convert to degrees.
// Not gated on HACDCPF_HAVE_OPENDSS because case2jpc_phase uses it unconditionally.
double phase_shift_from_vector_group(const std::string& raw_group) {
  std::string digits;
  for (unsigned char raw : raw_group) {
    if (std::isdigit(raw)) {
      digits.push_back(static_cast<char>(raw));
    }
  }
  if (digits.empty()) {
    return 0.0;
  }

  const int clock = std::stoi(digits) % 12;
  const int normalized = ((12 - (clock % 12)) % 12);
  double shift_deg = static_cast<double>(normalized) * 30.0;
  if (shift_deg > 180.0) {
    shift_deg -= 360.0;
  }
  return shift_deg;
}

#ifdef HACDCPF_HAVE_OPENDSS
class LocalDSSContext {
 public:
  LocalDSSContext() {
    io::ScopedDSSFloatingPointEnv fp_env;
    ctx_ = ctx_New();
    if (ctx_ == nullptr) {
      throw std::runtime_error("Failed to create DSS C-API context");
    }
    error_ptr_ = ctx_Error_Get_NumberPtr(ctx_);
    ctx_DSS_Start(ctx_, 0);
    check("ctx_DSS_Start");
  }

  ~LocalDSSContext() {
    if (ctx_ != nullptr) {
      io::ScopedDSSFloatingPointEnv fp_env;
      ctx_Dispose(ctx_);
    }
  }

  LocalDSSContext(const LocalDSSContext&) = delete;
  LocalDSSContext& operator=(const LocalDSSContext&) = delete;

  const void* get() const { return ctx_; }

  void check(const char* what = nullptr) const {
    if (error_ptr_ == nullptr || *error_ptr_ == 0) return;

    const int32_t code = *error_ptr_;
    const char* description = ctx_Error_Get_Description(ctx_);
    *error_ptr_ = 0;

    std::string message =
        (description != nullptr) ? std::string(description) : "Unknown DSS error";
    if (what != nullptr && *what != '\0') {
      throw std::runtime_error(
          std::string(what) + " failed: " + message +
          " (code " + std::to_string(code) + ")");
    }
    throw std::runtime_error(
        message + " (code " + std::to_string(code) + ")");
  }

 private:
  const void* ctx_{nullptr};
  int32_t* error_ptr_{nullptr};
};

template <typename Fn, typename... Args>
std::vector<double> dss_get_double_array(
    const LocalDSSContext& api,
    Fn fn,
    Args... args) {
  double* values = nullptr;
  int32_t dims[4] = {0, 0, 0, 0};
  if constexpr (std::is_invocable_v<Fn, const void*, double**, int32_t*, Args...>) {
    fn(api.get(), &values, dims, args...);
  } else {
    fn(api.get(), &values, dims);
  }
  api.check();
  if (values == nullptr || dims[0] <= 0) {
    return {};
  }
  return {values, values + dims[0]};
}

template <typename Fn, typename... Args>
std::vector<int32_t> dss_get_int_array(
    const LocalDSSContext& api,
    Fn fn,
    Args... args) {
  int32_t* values = nullptr;
  int32_t dims[4] = {0, 0, 0, 0};
  if constexpr (std::is_invocable_v<Fn, const void*, int32_t**, int32_t*, Args...>) {
    fn(api.get(), &values, dims, args...);
  } else {
    fn(api.get(), &values, dims);
  }
  api.check();
  if (values == nullptr || dims[0] <= 0) {
    return {};
  }
  return {values, values + dims[0]};
}

template <typename Fn, typename... Args>
std::vector<std::string> dss_get_string_array(
    const LocalDSSContext& api,
    Fn fn,
    Args... args) {
  char** values = nullptr;
  int32_t dims[4] = {0, 0, 0, 0};
  if constexpr (std::is_invocable_v<Fn, const void*, char***, int32_t*, Args...>) {
    fn(api.get(), &values, dims, args...);
  } else {
    fn(api.get(), &values, dims);
  }
  api.check();

  std::vector<std::string> out;
  if (values == nullptr || dims[0] <= 0) {
    return out;
  }
  out.reserve(static_cast<std::size_t>(dims[0]));
  for (int32_t idx = 0; idx < dims[0]; ++idx) {
    out.emplace_back(values[idx] != nullptr ? values[idx] : "");
  }
  return out;
}

template <typename Fn>
double dss_get_double_value(
    const LocalDSSContext& api,
    Fn fn,
    const char* what) {
  const double value = fn(api.get());
  api.check(what);
  return value;
}

template <typename Fn>
int dss_get_int_value(
    const LocalDSSContext& api,
    Fn fn,
    const char* what) {
  const int value = static_cast<int>(fn(api.get()));
  api.check(what);
  return value;
}

std::string dss_get_string_value(
    const LocalDSSContext& api,
    const char* (*fn)(const void*),
    const char* what) {
  const char* value = fn(api.get());
  api.check(what);
  return (value != nullptr) ? std::string(value) : std::string();
}

void dss_run_command(const LocalDSSContext& api, const std::string& command) {
  io::ScopedDSSFloatingPointEnv fp_env;
  ctx_Text_Set_Command(api.get(), command.c_str());
  api.check("ctx_Text_Set_Command");
}

std::string quote_path_for_dss(const std::filesystem::path& path) {
  return "\"" + path.string() + "\"";
}

std::vector<std::string> load_dss_commands_with_continuations(
    const std::filesystem::path& dss_path) {
  auto trim_local = [](std::string text) {
    const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
    auto first = std::find_if(text.begin(), text.end(), not_space);
    if (first == text.end()) return std::string();
    auto last = std::find_if(text.rbegin(), text.rend(), not_space).base();
    return std::string(first, last);
  };

  std::ifstream input(dss_path);
  if (!input) {
    throw std::runtime_error("Failed to open DSS file: " + dss_path.string());
  }

  std::vector<std::string> commands;
  std::string current;
  std::string raw_line;
  bool in_block_comment = false;
  while (std::getline(input, raw_line)) {
    std::string uncommented;
    std::size_t pos = 0;
    while (pos < raw_line.size()) {
      if (in_block_comment) {
        const std::size_t end = raw_line.find("*/", pos);
        if (end == std::string::npos) {
          pos = raw_line.size();
          continue;
        }
        in_block_comment = false;
        pos = end + 2;
        continue;
      }
      const std::size_t start = raw_line.find("/*", pos);
      if (start == std::string::npos) {
        uncommented.append(raw_line.substr(pos));
        break;
      }
      uncommented.append(raw_line.substr(pos, start - pos));
      in_block_comment = true;
      pos = start + 2;
    }

    const std::string trimmed = trim_local(uncommented);
    if (trimmed.empty() || trimmed.front() == '!' ||
        (trimmed.size() >= 2 && trimmed[0] == '/' && trimmed[1] == '/')) {
      continue;
    }

    if (!trimmed.empty() && trimmed.front() == '~') {
      const std::string suffix = trim_local(trimmed.substr(1));
      if (!current.empty() && !suffix.empty()) {
        current.append(" ");
        current.append(suffix);
      }
      continue;
    }

    if (!current.empty()) {
      commands.push_back(current);
    }
    current = trimmed;
  }

  if (!current.empty()) {
    commands.push_back(std::move(current));
  }

  return commands;
}

void compile_master_dss(
    const LocalDSSContext& api,
    const std::filesystem::path& master_dss) {
  const std::filesystem::path absolute_master =
      std::filesystem::absolute(master_dss);
  if (!std::filesystem::exists(absolute_master)) {
    throw std::runtime_error("DSS master file not found: " + absolute_master.string());
  }

  ctx_DSS_ClearAll(api.get());
  api.check("ctx_DSS_ClearAll");
  ctx_DSS_Set_AllowChangeDir(api.get(), 1);
  api.check("ctx_DSS_Set_AllowChangeDir");
  ctx_DSS_Set_DataPath(api.get(), absolute_master.parent_path().string().c_str());
  api.check("ctx_DSS_Set_DataPath");

  // Use OpenDSS' native compile path for the actual circuit state.  A previous
  // manual replay of merged text commands was close for most elements, but it
  // changed IEEE13 Reg2 by one tap versus OpenDSS compile because compile has
  // additional parser/redirect/control side effects.
  dss_run_command(api, "compile " + quote_path_for_dss(absolute_master));
}

std::string dss_active_property(
    const LocalDSSContext& api,
    const char* property_name) {
  ctx_DSSProperty_Set_Name(api.get(), property_name);
  api.check("ctx_DSSProperty_Set_Name");
  const char* value = ctx_DSSProperty_Get_Val(api.get());
  api.check("ctx_DSSProperty_Get_Val");
  return (value != nullptr) ? std::string(value) : std::string();
}

std::string trim_ascii_copy(std::string text) {
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  auto first = std::find_if(text.begin(), text.end(), not_space);
  if (first == text.end()) return {};
  auto last = std::find_if(text.rbegin(), text.rend(), not_space).base();
  return std::string(first, last);
}

std::optional<double> parse_optional_double(std::string raw_text);

std::string strip_matching_quotes(std::string text) {
  text = trim_ascii_copy(std::move(text));
  if (text.size() >= 2) {
    const char first = text.front();
    const char last = text.back();
    if ((first == '"' && last == '"') ||
        (first == '\'' && last == '\'')) {
      return text.substr(1, text.size() - 2);
    }
  }
  return text;
}

std::string normalize_dss_object_name(std::string text) {
  text = lowercase_ascii_copy(strip_matching_quotes(std::move(text)));
  const std::size_t dot = text.find('.');
  if (dot != std::string::npos) {
    return text.substr(dot + 1);
  }
  return text;
}

std::vector<std::string> split_dss_command_tokens(const std::string& command) {
  std::vector<std::string> tokens;
  std::string current;
  int bracket_depth = 0;
  bool in_quotes = false;
  char quote_char = '\0';
  for (char ch : command) {
    if (in_quotes) {
      current.push_back(ch);
      if (ch == quote_char) {
        in_quotes = false;
        quote_char = '\0';
      }
      continue;
    }
    if (ch == '"' || ch == '\'') {
      in_quotes = true;
      quote_char = ch;
      current.push_back(ch);
      continue;
    }
    if (ch == '[') {
      ++bracket_depth;
      current.push_back(ch);
      continue;
    }
    if (ch == ']') {
      bracket_depth = std::max(0, bracket_depth - 1);
      current.push_back(ch);
      continue;
    }
    if (std::isspace(static_cast<unsigned char>(ch)) && bracket_depth == 0) {
      if (!current.empty()) {
        tokens.push_back(current);
        current.clear();
      }
      continue;
    }
    current.push_back(ch);
  }
  if (!current.empty()) {
    tokens.push_back(current);
  }
  return tokens;
}

std::vector<double> parse_dss_double_array(std::string raw_text) {
  raw_text = trim_ascii_copy(std::move(raw_text));
  if (!raw_text.empty() && raw_text.front() == '[' && raw_text.back() == ']') {
    raw_text = raw_text.substr(1, raw_text.size() - 2);
  }
  for (char& ch : raw_text) {
    if (ch == ',' || ch == '|') {
      ch = ' ';
    }
  }
  std::vector<double> values;
  std::stringstream ss(raw_text);
  std::string token;
  while (ss >> token) {
    if (const auto value = parse_optional_double(token)) {
      values.push_back(*value);
    }
  }
  return values;
}

struct DSSTransformerTapContract {
  std::array<double, 2> tap_pu{1.0, 1.0};
  std::array<bool, 2> has_explicit_tap{false, false};
};

void apply_transformer_tap_tokens(
    DSSTransformerTapContract& contract,
    const std::vector<std::string>& tokens,
    std::size_t start_index) {
  int active_winding = 2;
  for (std::size_t idx = start_index; idx < tokens.size(); ++idx) {
    const std::string token = tokens[idx];
    const std::size_t eq = token.find('=');
    if (eq == std::string::npos) continue;
    const std::string key = lowercase_ascii_copy(token.substr(0, eq));
    const std::string value = strip_matching_quotes(token.substr(eq + 1));
    if (key == "wdg" || key == "winding") {
      if (const auto parsed = parse_optional_double(value)) {
        const int winding = static_cast<int>(std::llround(*parsed));
        if (winding >= 1 && winding <= 2) {
          active_winding = winding;
        }
      }
      continue;
    }
    if (key == "tap") {
      if (const auto parsed = parse_optional_double(value)) {
        const std::size_t slot = static_cast<std::size_t>(active_winding - 1);
        contract.tap_pu[slot] = *parsed;
        contract.has_explicit_tap[slot] = true;
      }
      continue;
    }
    if (key == "taps") {
      const std::vector<double> taps = parse_dss_double_array(value);
      for (std::size_t tap_idx = 0; tap_idx < taps.size() && tap_idx < 2; ++tap_idx) {
        contract.tap_pu[tap_idx] = taps[tap_idx];
        contract.has_explicit_tap[tap_idx] = true;
      }
    }
  }
}

void collect_transformer_tap_contracts_from_file(
    const std::filesystem::path& dss_path,
    std::unordered_map<std::string, DSSTransformerTapContract>& contracts,
    std::set<std::filesystem::path>& visited_files);

void collect_transformer_tap_contracts_from_command(
    const std::string& command,
    const std::filesystem::path& base_dir,
    std::unordered_map<std::string, DSSTransformerTapContract>& contracts,
    std::set<std::filesystem::path>& visited_files) {
  const std::vector<std::string> tokens = split_dss_command_tokens(command);
  if (tokens.empty()) return;

  const std::string verb = lowercase_ascii_copy(tokens.front());
  if (verb == "redirect") {
    if (tokens.size() < 2) return;
    const std::filesystem::path nested_path =
        base_dir / strip_matching_quotes(tokens[1]);
    collect_transformer_tap_contracts_from_file(
        std::filesystem::absolute(nested_path),
        contracts,
        visited_files);
    return;
  }

  if (verb != "new" && verb != "edit") return;
  if (tokens.size() < 2) return;

  std::string object_spec = strip_matching_quotes(tokens[1]);
  std::string object_spec_lower = lowercase_ascii_copy(object_spec);
  if (verb == "new" && object_spec_lower.rfind("object=", 0) == 0) {
    object_spec = object_spec.substr(7);
    object_spec_lower = lowercase_ascii_copy(object_spec);
  }
  if (object_spec_lower.rfind("transformer.", 0) != 0) {
    return;
  }

  const std::string transformer_name =
      normalize_dss_object_name(object_spec);
  if (transformer_name.empty()) return;

  if (verb == "new") {
    DSSTransformerTapContract contract;
    for (std::size_t idx = 2; idx < tokens.size(); ++idx) {
      const std::string token = tokens[idx];
      const std::size_t eq = token.find('=');
      if (eq == std::string::npos) continue;
      const std::string key = lowercase_ascii_copy(token.substr(0, eq));
      if (key != "like") continue;
      const std::string like_name =
          normalize_dss_object_name(token.substr(eq + 1));
      const auto like_it = contracts.find(like_name);
      if (like_it != contracts.end()) {
        contract = like_it->second;
      }
      break;
    }
    apply_transformer_tap_tokens(contract, tokens, 2);
    contracts[transformer_name] = std::move(contract);
    return;
  }

  auto contract_it = contracts.find(transformer_name);
  if (contract_it == contracts.end()) {
    contract_it = contracts.emplace(transformer_name, DSSTransformerTapContract{}).first;
  }
  apply_transformer_tap_tokens(contract_it->second, tokens, 2);
}

void collect_transformer_tap_contracts_from_file(
    const std::filesystem::path& dss_path,
    std::unordered_map<std::string, DSSTransformerTapContract>& contracts,
    std::set<std::filesystem::path>& visited_files) {
  const std::filesystem::path absolute_path = std::filesystem::absolute(dss_path);
  if (!visited_files.insert(absolute_path).second) {
    return;
  }
  const std::vector<std::string> commands =
      load_dss_commands_with_continuations(absolute_path);
  for (const auto& command : commands) {
    collect_transformer_tap_contracts_from_command(
        command,
        absolute_path.parent_path(),
        contracts,
        visited_files);
  }
}

std::unordered_map<std::string, DSSTransformerTapContract>
collect_dss_transformer_tap_contracts(const std::filesystem::path& master_dss) {
  std::unordered_map<std::string, DSSTransformerTapContract> contracts;
  std::set<std::filesystem::path> visited_files;
  collect_transformer_tap_contracts_from_file(
      std::filesystem::absolute(master_dss),
      contracts,
      visited_files);
  return contracts;
}

std::string normalize_dss_length_units(
    std::string raw_units,
    const std::string& fallback) {
  std::string units = lowercase_ascii_copy(trim_ascii_copy(std::move(raw_units)));
  if (units.empty()) return fallback;
  if (units == "meter") return "m";
  if (units == "mile" || units == "miles") return "mi";
  if (units == "inch" || units == "inches") return "in";
  return units;
}

std::optional<double> parse_optional_double(std::string raw_text) {
  const std::string text = trim_ascii_copy(std::move(raw_text));
  if (text.empty()) return std::nullopt;
  try {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed == 0) return std::nullopt;
    return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<double> dss_active_property_double(
    const LocalDSSContext& api,
    const char* property_name) {
  return parse_optional_double(dss_active_property(api, property_name));
}

struct VsourceImpedanceProperties {
  std::optional<double> r1_ohm;
  std::optional<double> x1_ohm;
  std::optional<double> r0_ohm;
  std::optional<double> x0_ohm;
  std::optional<double> mvasc3;
  std::optional<double> mvasc1;
  std::optional<double> x1r1;
  std::optional<double> x0r0;
};

VsourceImpedanceProperties read_vsource_impedance_properties(
    const LocalDSSContext& api) {
  return {
      dss_active_property_double(api, "r1"),
      dss_active_property_double(api, "x1"),
      dss_active_property_double(api, "r0"),
      dss_active_property_double(api, "x0"),
      dss_active_property_double(api, "mvasc3"),
      dss_active_property_double(api, "mvasc1"),
      dss_active_property_double(api, "x1r1"),
      dss_active_property_double(api, "x0r0"),
  };
}

Complex impedance_from_magnitude_and_x_over_r(
    double magnitude_pu,
    double x_over_r) {
  if (!(magnitude_pu > 0.0) || !std::isfinite(magnitude_pu)) {
    return Complex(0.0, 0.0);
  }
  const double xr = std::max(0.0, x_over_r);
  const double r = magnitude_pu / std::sqrt(1.0 + xr * xr);
  return Complex(r, r * xr);
}

bool has_nonzero_impedance(const Complex& value) {
  return std::abs(value) > 1e-12;
}

std::optional<Complex> derive_positive_sequence_impedance_from_mvasc(
    const VsourceImpedanceProperties& props,
    double base_mva) {
  if (!props.mvasc3 || !props.x1r1 || *props.mvasc3 <= 0.0 || *props.x1r1 < 0.0 ||
      base_mva <= 0.0) {
    return std::nullopt;
  }
  const Complex z1 = impedance_from_magnitude_and_x_over_r(
      base_mva / *props.mvasc3,
      *props.x1r1);
  return has_nonzero_impedance(z1) ? std::optional<Complex>(z1) : std::nullopt;
}

std::optional<Complex> derive_zero_sequence_impedance_from_mvasc(
    const VsourceImpedanceProperties& props,
    double base_mva,
    const Complex& z1) {
  if (!props.mvasc1 || !props.x0r0 || *props.mvasc1 <= 0.0 || *props.x0r0 < 0.0 ||
      base_mva <= 0.0 || !has_nonzero_impedance(z1)) {
    return std::nullopt;
  }

  const double target_series_mag = 3.0 * base_mva / *props.mvasc1;
  if (!(target_series_mag > 0.0) || !std::isfinite(target_series_mag)) {
    return std::nullopt;
  }

  const double xr0 = *props.x0r0;
  const double norm = std::sqrt(1.0 + xr0 * xr0);
  const Complex direction(1.0 / norm, xr0 / norm);
  const Complex anchor = 2.0 * z1;
  const double linear = 2.0 * std::real(direction * std::conj(anchor));
  const double constant = std::norm(anchor) - target_series_mag * target_series_mag;
  double discriminant = linear * linear - 4.0 * constant;
  if (discriminant < -1e-10) {
    return std::nullopt;
  }
  discriminant = std::max(0.0, discriminant);
  const double sqrt_discriminant = std::sqrt(discriminant);
  const double root_hi = (-linear + sqrt_discriminant) / 2.0;
  const double root_lo = (-linear - sqrt_discriminant) / 2.0;

  double magnitude = -1.0;
  if (root_hi >= 0.0) magnitude = root_hi;
  if (root_lo >= 0.0) magnitude = std::max(magnitude, root_lo);
  if (!(magnitude > 0.0) || !std::isfinite(magnitude)) {
    return std::nullopt;
  }

  const Complex z0 = direction * magnitude;
  return has_nonzero_impedance(z0) ? std::optional<Complex>(z0) : std::nullopt;
}

PhaseMask phase_mask_from_node_numbers(const std::vector<int>& nodes) {
  std::uint8_t bits = 0;
  for (const int node : nodes) {
    if (node >= 1 && node <= 3) {
      bits |= static_cast<std::uint8_t>(1u << (node - 1));
    }
  }
  return bits == 0 ? PhaseMask::abc() : PhaseMask(bits);
}

struct ParsedDSSBusTerminal {
  std::string bus_name;
  std::vector<int> nodes;
  PhaseMask phase_mask{PhaseMask::abc()};
  bool has_ground_reference{false};
};

ParsedDSSBusTerminal parse_dss_bus_terminal(
    const std::string& raw_name,
    int default_phase_count = 3) {
  ParsedDSSBusTerminal parsed;
  std::string token;
  std::vector<std::string> parts;
  for (char ch : raw_name) {
    if (ch == '.') {
      parts.push_back(token);
      token.clear();
    } else {
      token.push_back(ch);
    }
  }
  parts.push_back(token);
  parsed.bus_name =
      parts.empty() ? lowercase_ascii_copy(raw_name)
                    : lowercase_ascii_copy(parts.front());
  for (std::size_t idx = 1; idx < parts.size(); ++idx) {
    if (parts[idx].empty()) continue;
    try {
      const int node = std::stoi(parts[idx]);
      if (node == 0) {
        parsed.has_ground_reference = true;
      }
      if (node >= 1 && node <= 3) {
        parsed.nodes.push_back(node);
      }
    } catch (const std::exception&) {
    }
  }
  if (parsed.nodes.empty()) {
    for (int phase = 1; phase <= std::clamp(default_phase_count, 1, 3); ++phase) {
      parsed.nodes.push_back(phase);
    }
  }
  std::sort(parsed.nodes.begin(), parsed.nodes.end());
  parsed.nodes.erase(std::unique(parsed.nodes.begin(), parsed.nodes.end()), parsed.nodes.end());
  parsed.phase_mask = phase_mask_from_node_numbers(parsed.nodes);
  return parsed;
}

std::vector<int> dss_phase_slots(PhaseMask mask) {
  std::vector<int> slots;
  for (int phase = 0; phase < 3; ++phase) {
    if (mask.has(phase)) slots.push_back(phase + 1);
  }
  return slots;
}

PhaseValueMatrix3 zero_phase_value_matrix() {
  PhaseValueMatrix3 matrix{};
  matrix.fill(0.0);
  return matrix;
}

PhaseValueMatrix3 expand_dss_phase_matrix(
    const std::vector<double>& raw_values,
    const std::vector<int>& phase_slots,
    double scale) {
  PhaseValueMatrix3 matrix = zero_phase_value_matrix();
  const int n = static_cast<int>(phase_slots.size());
  if (n == 0 || raw_values.empty()) return matrix;

  auto set_symmetric = [&](int row, int col, double value) {
    phase_matrix_set(matrix, phase_slots[static_cast<std::size_t>(row)] - 1,
                     phase_slots[static_cast<std::size_t>(col)] - 1, value);
    phase_matrix_set(matrix, phase_slots[static_cast<std::size_t>(col)] - 1,
                     phase_slots[static_cast<std::size_t>(row)] - 1, value);
  };

  if (static_cast<int>(raw_values.size()) == n * n) {
    for (int row = 0; row < n; ++row) {
      for (int col = 0; col < n; ++col) {
        phase_matrix_set(
            matrix,
            phase_slots[static_cast<std::size_t>(row)] - 1,
            phase_slots[static_cast<std::size_t>(col)] - 1,
            raw_values[static_cast<std::size_t>(row * n + col)] * scale);
      }
    }
    return matrix;
  }

  if (static_cast<int>(raw_values.size()) == n * (n + 1) / 2) {
    int index = 0;
    for (int row = 0; row < n; ++row) {
      for (int col = 0; col <= row; ++col) {
        set_symmetric(row, col, raw_values[static_cast<std::size_t>(index++)] * scale);
      }
    }
    return matrix;
  }

  if (static_cast<int>(raw_values.size()) == n) {
    for (int row = 0; row < n; ++row) {
      phase_matrix_set(
          matrix,
          phase_slots[static_cast<std::size_t>(row)] - 1,
          phase_slots[static_cast<std::size_t>(row)] - 1,
          raw_values[static_cast<std::size_t>(row)] * scale);
    }
    return matrix;
  }

  if (raw_values.size() == 1) {
    for (int row = 0; row < n; ++row) {
      phase_matrix_set(
          matrix,
          phase_slots[static_cast<std::size_t>(row)] - 1,
          phase_slots[static_cast<std::size_t>(row)] - 1,
          raw_values.front() * scale);
    }
  }
  return matrix;
}

double dss_length_to_km(double length, const std::string& units_raw) {
  const std::string units = normalize_dss_length_units(units_raw, "km");
  if (units.empty() || units == "km") return length;
  if (units == "m") return length / 1000.0;
  if (units == "cm") return length / 100000.0;
  if (units == "mm") return length / 1.0e6;
  if (units == "mi" || units == "mile" || units == "miles") return length * 1.609344;
  if (units == "kft") return length * 0.3048;
  if (units == "ft") return length * 0.0003048;
  if (units == "in" || units == "inch") return length * 0.0000254;
  return length;
}

double dss_impedance_to_ohm_per_km(double value, const std::string& units_raw) {
  const std::string units = normalize_dss_length_units(units_raw, "km");
  if (units == "km") return value;
  if (units == "m") return value * 1000.0;
  if (units == "cm") return value * 100000.0;
  if (units == "mm") return value * 1.0e6;
  if (units == "mi") return value / 1.609344;
  if (units == "kft") return value / 0.3048;
  if (units == "ft") return value / 0.0003048;
  if (units == "in") return value / 0.0000254;
  if (units == "none") return value;
  return value;
}

double dss_capacitance_to_nf_per_km(double value, const std::string& units_raw) {
  return dss_impedance_to_ohm_per_km(value, units_raw);
}

std::string dss_dimension_units_to_string(int code) {
  switch (code) {
    case 0: return "none";
    case 1: return "mi";
    case 2: return "kft";
    case 3: return "km";
    case 4: return "m";
    case 5: return "ft";
    case 6: return "in";
    case 7: return "cm";
    case 8: return "mm";
    default: return {};
  }
}

std::string dss_line_impedance_units(
    const LocalDSSContext& api,
    const std::string& line_units_raw) {
  const std::string line_units =
      normalize_dss_length_units(line_units_raw, "kft");
  // DSS C-API's active Lines.Rmatrix/Xmatrix/Cmatrix getters return values
  // converted to the active line's units, not the source LineCode units.  Use
  // the line units when they are explicit; fall back to the LineCode units only
  // for older/implicit DSS files where the line itself reports "none".
  if (line_units != "none") {
    return line_units;
  }
  const std::string linecode_name = trim_ascii_copy(
      dss_get_string_value(api, ctx_Lines_Get_LineCode, "ctx_Lines_Get_LineCode"));
  if (linecode_name.empty()) {
    return line_units;
  }

  ctx_LineCodes_Set_Name(api.get(), linecode_name.c_str());
  api.check("ctx_LineCodes_Set_Name");
  const std::string linecode_units = normalize_dss_length_units(
      dss_dimension_units_to_string(
          dss_get_int_value(api, ctx_LineCodes_Get_Units, "ctx_LineCodes_Get_Units")),
      "none");
  if (linecode_units == "none") {
    return line_units;
  }
  return linecode_units;
}

double bus_base_kv_from_dss(double kv_base_ln, PhaseMask mask) {
  if (kv_base_ln <= 0.0) return 0.0;
  return mask.count() == 3 ? kv_base_ln * std::sqrt(3.0) : kv_base_ln;
}

double bus_base_impedance_ohm(const ThreePhaseACBus& bus, double base_mva) {
  if (bus.base_kv <= 0.0 || base_mva <= 0.0) return 1.0;
  const double base_kv_ll =
      bus.phase_mask.count() == 3 ? bus.base_kv : bus.base_kv * std::sqrt(3.0);
  return (base_kv_ll * base_kv_ll) / base_mva;
}

std::string transformer_connection_token(bool is_delta, bool grounded_wye) {
  if (is_delta) return "D";
  return grounded_wye ? "YN" : "Y";
}

int transformer_clock_from_leadlag(const std::string& leadlag_raw) {
  const std::string leadlag = lowercase_ascii_copy(leadlag_raw);
  return (leadlag.find("lead") != std::string::npos) ? 11 : 1;
}

std::string infer_transformer_vector_group(
    bool hv_delta,
    bool lv_delta,
    bool hv_grounded_wye,
    bool lv_grounded_wye,
    int clock) {
  const std::string hv = transformer_connection_token(hv_delta, hv_grounded_wye);
  const std::string lv = transformer_connection_token(lv_delta, lv_grounded_wye);
  if (hv_delta == lv_delta && hv_grounded_wye == lv_grounded_wye) {
    clock = 0;
  }
  return hv + lv + std::to_string(clock);
}

struct OpenDSSYMatrixSolveResult {
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  std::unordered_map<std::string, std::array<std::optional<Complex>, 3>> voltage_map;
};

#endif

PhaseDomainConnectionType parse_connection_type(
    const std::string& vector_group,
    bool hv_side) {
  std::string letters;
  for (unsigned char raw : vector_group) {
    if (std::isalpha(raw)) {
      letters.push_back(static_cast<char>(std::toupper(raw)));
    }
  }

  std::size_t pos = 0;
  auto parse_side = [&](PhaseDomainConnectionType& connection) {
    if (pos >= letters.size()) {
      connection = PhaseDomainConnectionType::Unknown;
      return;
    }
    const char head = letters[pos++];
    if (head == 'D') {
      connection = PhaseDomainConnectionType::Delta;
      return;
    }
    if (head == 'Y') {
      const bool grounded = pos < letters.size() && letters[pos] == 'N';
      if (grounded) ++pos;
      connection = grounded ? PhaseDomainConnectionType::GroundedWye
                            : PhaseDomainConnectionType::Wye;
      return;
    }
    connection = PhaseDomainConnectionType::Unknown;
  };

  PhaseDomainConnectionType hv = PhaseDomainConnectionType::Unknown;
  PhaseDomainConnectionType lv = PhaseDomainConnectionType::Unknown;
  parse_side(hv);
  parse_side(lv);
  return hv_side ? hv : lv;
}

std::array<std::array<double, 3>, 3> zero_phase_matrix() {
  return {{
      {{0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
      {{0.0, 0.0, 0.0}},
  }};
}

std::array<std::array<double, 3>, 3> build_sequence_zabc_real(
    double r0_pu,
    double x0_pu,
    double r1_pu,
    double x1_pu,
    bool want_real) {
  const Complex z1(r1_pu, x1_pu);
  Complex z0(r0_pu, x0_pu);
  if (std::abs(z0) <= 1e-12) z0 = z1;

  const Complex zs = (z0 + 2.0 * z1) / 3.0;
  const Complex zm = (z0 - z1) / 3.0;

  auto matrix = zero_phase_matrix();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const Complex value = (row == col) ? zs : zm;
      matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
          want_real ? value.real() : value.imag();
    }
  }
  return matrix;
}

std::array<std::array<double, 3>, 3> build_line_r_matrix(
    const ThreePhaseACLine& line) {
  auto matrix = zero_phase_matrix();
  if (line.use_phase_matrix) {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
            phase_matrix_get(line.r_matrix_pu, row, col);
      }
    }
    return matrix;
  }
  return build_sequence_zabc_real(line.r0_pu, line.x0_pu, line.r1_pu, line.x1_pu, true);
}

std::array<std::array<double, 3>, 3> build_line_x_matrix(
    const ThreePhaseACLine& line) {
  auto matrix = zero_phase_matrix();
  if (line.use_phase_matrix) {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
            phase_matrix_get(line.x_matrix_pu, row, col);
      }
    }
    return matrix;
  }
  return build_sequence_zabc_real(line.r0_pu, line.x0_pu, line.r1_pu, line.x1_pu, false);
}

std::array<double, 3> build_line_b_vector(const ThreePhaseACLine& line) {
  std::array<double, 3> b{};
  if (line.use_phase_matrix) {
    for (int phase = 0; phase < 3; ++phase) {
      b[static_cast<std::size_t>(phase)] =
          phase_matrix_get(line.b_matrix_pu, phase, phase);
    }
    return b;
  }
  b.fill(line.b1_pu);
  return b;
}

double transformer_series_r_pu(
    const ThreePhaseTransformer& transformer,
    double base_mva) {
  if (transformer.sn_mva <= 0.0) return 0.0;
  return (transformer.vkr_percent / 100.0) * (base_mva / transformer.sn_mva);
}

double transformer_series_x_pu(
    const ThreePhaseTransformer& transformer,
    double base_mva) {
  if (transformer.sn_mva <= 0.0) return 0.0;
  const double z_mag =
      (transformer.vk_percent / 100.0) * (base_mva / transformer.sn_mva);
  const double r_pu = transformer_series_r_pu(transformer, base_mva);
  return std::sqrt(std::max(0.0, z_mag * z_mag - r_pu * r_pu));
}

std::set<int> build_regulated_transformer_index_set(
    const ThreePhaseACSystem& sys) {
  std::set<int> regulated;
  for (const auto& control : sys.regulator_controls) {
    if (control.enabled) {
      regulated.insert(control.transformer_index);
    }
  }
  return regulated;
}

void fill_bus_generation_fields(
    PhaseDomainBusRow& row,
    const ThreePhaseGenerator& generator) {
  row.pg_mw[0] += generator.p_a_mw;
  row.qg_mvar[0] += generator.q_a_mvar;
  row.pg_mw[1] += generator.p_b_mw;
  row.qg_mvar[1] += generator.q_b_mvar;
  row.pg_mw[2] += generator.p_c_mw;
  row.qg_mvar[2] += generator.q_c_mvar;

  const bool has_per_phase =
      std::abs(generator.p_a_mw) > 1e-12 || std::abs(generator.q_a_mvar) > 1e-12 ||
      std::abs(generator.p_b_mw) > 1e-12 || std::abs(generator.q_b_mvar) > 1e-12 ||
      std::abs(generator.p_c_mw) > 1e-12 || std::abs(generator.q_c_mvar) > 1e-12;
  if (has_per_phase) return;

  const double phase_count = std::max(1, generator.phase_mask.count());
  const double p_per_phase = generator.p_mw / phase_count;
  const double q_per_phase = generator.q_mvar / phase_count;
  for (int phase = 0; phase < 3; ++phase) {
    if (!generator.phase_mask.has(phase)) continue;
    row.pg_mw[static_cast<std::size_t>(phase)] += p_per_phase;
    row.qg_mvar[static_cast<std::size_t>(phase)] += q_per_phase;
  }
}

std::array<double, 3> transformer_phase_taps(const ThreePhaseTransformer& transformer) {
  const double tap =
      transformer.tap_step_percent == 0.0
          ? 1.0
          : std::max(
                1e-6,
                1.0 +
                    (static_cast<double>(transformer.tap_pos - transformer.tap_neutral) *
                     transformer.tap_step_percent / 100.0));
  return {tap, tap, tap};
}

int tap_position_from_ratio(
    const ThreePhaseTransformer& transformer,
    double tap_ratio_pu) {
  if (std::abs(transformer.tap_step_percent) <= 1e-12) {
    return transformer.tap_pos;
  }
  const double raw =
      (tap_ratio_pu - 1.0) * 100.0 / transformer.tap_step_percent +
      static_cast<double>(transformer.tap_neutral);
  const int discrete = static_cast<int>(std::llround(raw));
  return std::clamp(discrete, transformer.tap_min, transformer.tap_max);
}

void apply_reg_tap_overrides(
    ThreePhaseACSystem& sys,
    const std::unordered_map<std::string, double>& reg_taps) {
  if (reg_taps.empty()) return;

  std::unordered_map<int, double> transformer_taps;
  for (const auto& transformer : sys.transformers) {
    const std::string transformer_name = lowercase_ascii_copy(transformer.name);
    auto it = reg_taps.find(transformer_name);
    if (it != reg_taps.end()) {
      transformer_taps[transformer.index] = it->second;
    }
  }

  for (const auto& control : sys.regulator_controls) {
    const auto control_it = reg_taps.find(lowercase_ascii_copy(control.name));
    if (control_it != reg_taps.end()) {
      transformer_taps[control.transformer_index] = control_it->second;
      continue;
    }
    const auto transformer_it =
        reg_taps.find(lowercase_ascii_copy(control.transformer_name));
    if (transformer_it != reg_taps.end()) {
      transformer_taps[control.transformer_index] = transformer_it->second;
    }
  }

  for (auto& transformer : sys.transformers) {
    const auto it = transformer_taps.find(transformer.index);
    if (it == transformer_taps.end()) continue;
    transformer.tap_pos = tap_position_from_ratio(transformer, it->second);
  }
}

[[maybe_unused]] std::vector<PhaseDomainSparseEntry> expand_compact_ybus_to_full_entries(
    const ThreePhaseCompactPFData& compact,
    int bus_count) {
  std::vector<PhaseDomainSparseEntry> full_entries;
  full_entries.reserve(
      compact.ybus_entries.size() + static_cast<std::size_t>(bus_count * 3));
  std::vector<bool> diagonal_present(
      static_cast<std::size_t>(bus_count * 3),
      false);

  for (const auto& entry : compact.ybus_entries) {
    const PhaseNodeRef& row_ref = compact.indexer.node_ref(entry.row);
    const PhaseNodeRef& col_ref = compact.indexer.node_ref(entry.col);
    const int full_row = row_ref.bus_offset * 3 + row_ref.phase_index;
    const int full_col = col_ref.bus_offset * 3 + col_ref.phase_index;
    if (full_row == full_col) {
      diagonal_present[static_cast<std::size_t>(full_row)] = true;
    }
    full_entries.push_back(PhaseDomainSparseEntry{
        .row = full_row,
        .col = full_col,
        .value = entry.value,
    });
  }

  const Complex small_admittance(1e-6, 1e-6);
  for (int bus_offset = 0; bus_offset < static_cast<int>(compact.indexer.bus_phase_masks.size());
       ++bus_offset) {
    const PhaseMask mask = compact.indexer.bus_phase_mask(bus_offset);
    for (int phase = 0; phase < 3; ++phase) {
      if (mask.has(phase)) continue;
      const int full_index = bus_offset * 3 + phase;
      if (diagonal_present[static_cast<std::size_t>(full_index)]) continue;
      full_entries.push_back(PhaseDomainSparseEntry{
          .row = full_index,
          .col = full_index,
          .value = small_admittance,
      });
    }
  }

  return full_entries;
}

void populate_jpc_voltage_state(
    ThreePhaseJPCPhase& jpc,
    const ThreePhaseDPFResult& result) {
  jpc.success = result.converged;
  jpc.iterations = result.iterations;
  jpc.residual = result.residual;
  jpc.primary_solver = result.primary_solver;
  jpc.solver_used = result.solver_used;
  jpc.result_source = result.result_source;
  jpc.fallback_used = result.fallback_used;
  jpc.primary_solver_failed_reason = result.primary_solver_failed_reason;
  jpc.v_abc.assign(jpc.bus_abc.size() * 3, Complex(0.0, 0.0));

  std::unordered_map<int, std::size_t> bus_row_lookup;
  bus_row_lookup.reserve(jpc.bus_abc.size());
  for (std::size_t row = 0; row < jpc.bus_abc.size(); ++row) {
    bus_row_lookup[jpc.bus_abc[row].bus_id] = row;
  }

  for (const auto& voltage : result.bus_voltages) {
    const auto it = bus_row_lookup.find(voltage.bus_id);
    if (it == bus_row_lookup.end()) continue;
    auto& row = jpc.bus_abc[it->second];
    row.vm_pu = {voltage.vm_a_pu, voltage.vm_b_pu, voltage.vm_c_pu};
    row.va_deg = {voltage.va_a_deg, voltage.va_b_deg, voltage.va_c_deg};
    jpc.v_abc[it->second * 3 + 0] =
        std::polar(voltage.vm_a_pu, voltage.va_a_deg * kPi / 180.0);
    jpc.v_abc[it->second * 3 + 1] =
        std::polar(voltage.vm_b_pu, voltage.va_b_deg * kPi / 180.0);
    jpc.v_abc[it->second * 3 + 2] =
        std::polar(voltage.vm_c_pu, voltage.va_c_deg * kPi / 180.0);
  }
}

#ifdef HACDCPF_HAVE_OPENDSS
void populate_jpc_from_voltage_map(
    ThreePhaseJPCPhase& jpc,
    const std::unordered_map<std::string, std::array<std::optional<Complex>, 3>>& voltage_map,
    bool converged,
    int iterations,
    double residual) {
  jpc.success = converged;
  jpc.iterations = iterations;
  jpc.residual = residual;
  jpc.v_abc.assign(jpc.bus_abc.size() * 3, Complex(0.0, 0.0));

  for (std::size_t row_idx = 0; row_idx < jpc.bus_abc.size(); ++row_idx) {
    auto& row = jpc.bus_abc[row_idx];
    const auto original_has_phase = row.has_phase;
    row.has_phase = {false, false, false};
    row.vm_pu = {0.0, 0.0, 0.0};
    row.va_deg = {0.0, 0.0, 0.0};
    const auto bus_name_it = jpc.bus_id_to_name.find(row.bus_id);
    const std::string key =
        bus_name_it == jpc.bus_id_to_name.end()
            ? lowercase_ascii_copy(std::to_string(row.bus_id))
            : lowercase_ascii_copy(bus_name_it->second);
    const auto voltages_it = voltage_map.find(key);
    if (voltages_it == voltage_map.end()) continue;
    const int active_phase_count = static_cast<int>(std::count(
        original_has_phase.begin(),
        original_has_phase.end(),
        true));
    const double phase_base_kv =
        (active_phase_count == 3) ? row.base_kv / std::sqrt(3.0) : row.base_kv;
    const double phase_base_volts =
        (phase_base_kv > 0.0) ? phase_base_kv * 1000.0 : 0.0;
    for (int phase = 0; phase < 3; ++phase) {
      const auto& phase_voltage =
          voltages_it->second[static_cast<std::size_t>(phase)];
      if (!phase_voltage.has_value()) continue;
      const Complex voltage_pu =
          (phase_base_volts > 0.0) ? (*phase_voltage / phase_base_volts) : *phase_voltage;
      row.vm_pu[static_cast<std::size_t>(phase)] = std::abs(voltage_pu);
      row.va_deg[static_cast<std::size_t>(phase)] =
          std::arg(voltage_pu) * 180.0 / kPi;
      row.has_phase[static_cast<std::size_t>(phase)] = true;
      jpc.v_abc[row_idx * 3 + static_cast<std::size_t>(phase)] = voltage_pu;
    }
  }
}

[[maybe_unused]] void populate_jpc_from_opendss_snapshot(
    ThreePhaseJPCPhase& jpc,
    const io::OpenDSSSnapshotResult& snapshot) {
  std::unordered_map<std::string, std::array<std::optional<Complex>, 3>> voltage_map;
  for (const auto& node_voltage : snapshot.node_voltages) {
    if (node_voltage.node < 1 || node_voltage.node > 3) continue;
    voltage_map[node_voltage.bus_name][static_cast<std::size_t>(node_voltage.node - 1)] =
        std::polar(node_voltage.vm_pu, node_voltage.va_deg * kPi / 180.0);
  }
  populate_jpc_from_voltage_map(
      jpc,
      voltage_map,
      snapshot.converged,
      snapshot.control_oracle.control_iterations,
      0.0);
}

OpenDSSYMatrixSolveResult solve_opendss_ymatrix(
    const std::filesystem::path& master_dss,
    const std::unordered_map<std::string, double>& reg_taps,
    int max_iter,
    double tol,
    bool verbose) {
  LocalDSSContext api;
  compile_master_dss(api, master_dss);

  for (const auto& [name, tap_pu] : reg_taps) {
    dss_run_command(
        api,
        "Transformer." + name + ".wdg=2 Tap=" + std::to_string(tap_pu));
  }
  dss_run_command(api, "Set ControlMode=OFF");
  ctx_Solution_Solve(api.get());
  api.check("ctx_Solution_Solve");

  ctx_Solution_InitSnap(api.get());
  api.check("ctx_Solution_InitSnap");
  ctx_YMatrix_BuildYMatrixD(api.get(), 2, 1);
  api.check("ctx_YMatrix_BuildYMatrixD");

  const std::vector<std::string> node_order =
      dss_get_string_array(api, ctx_Circuit_Get_YNodeOrder);
  if (node_order.empty()) {
    throw std::runtime_error("solve_opendss_ymatrix: empty YNodeOrder");
  }

  auto build_sparse_y_matrix = [&]() {
    uint32_t n_bus = 0;
    uint32_t n_nz = 0;
    int32_t* col_ptr = nullptr;
    int32_t* row_idx = nullptr;
    double* c_vals = nullptr;
    ctx_YMatrix_GetCompressedYMatrix(
        api.get(),
        2,
        &n_bus,
        &n_nz,
        &col_ptr,
        &row_idx,
        &c_vals);
    api.check("ctx_YMatrix_GetCompressedYMatrix");

    const int matrix_dim = static_cast<int>(n_bus);
    if (matrix_dim != static_cast<int>(node_order.size())) {
      throw std::runtime_error(
          "solve_opendss_ymatrix: compressed matrix dimension does not match YNodeOrder");
    }
    if (col_ptr == nullptr || row_idx == nullptr || c_vals == nullptr) {
      throw std::runtime_error(
          "solve_opendss_ymatrix: compressed matrix buffers are null");
    }

    const int pointer_offset = (col_ptr[0] == 1) ? 1 : 0;
    std::vector<Eigen::Triplet<Complex>> triplets;
    triplets.reserve(static_cast<std::size_t>(n_nz));
    for (int col = 0; col < matrix_dim; ++col) {
      const int begin = col_ptr[col] - pointer_offset;
      const int end = col_ptr[col + 1] - pointer_offset;
      if (begin < 0 || end < begin || end > static_cast<int>(n_nz)) {
        throw std::runtime_error(
            "solve_opendss_ymatrix: invalid compressed column pointer range");
      }
      for (int pos = begin; pos < end; ++pos) {
        const int row = row_idx[pos] - pointer_offset;
        if (row < 0 || row >= matrix_dim) {
          throw std::runtime_error(
              "solve_opendss_ymatrix: invalid compressed row index");
        }
        const Complex value(c_vals[2 * pos], c_vals[2 * pos + 1]);
        if (std::abs(value) <= 1e-14) continue;
        triplets.emplace_back(row, col, value);
      }
    }

    Eigen::SparseMatrix<Complex> y_sparse(matrix_dim, matrix_dim);
    y_sparse.setFromTriplets(
        triplets.begin(),
        triplets.end(),
        [](const Complex& lhs, const Complex& rhs) { return lhs + rhs; });
    y_sparse.makeCompressed();
    return y_sparse;
  };

  auto factorize_y_matrix =
      [&](const Eigen::SparseMatrix<Complex>& y_sparse,
          Eigen::SparseLU<Eigen::SparseMatrix<Complex>>& lu) {
        lu.analyzePattern(y_sparse);
        lu.factorize(y_sparse);
        if (lu.info() != Eigen::Success) {
          throw std::runtime_error("solve_opendss_ymatrix: sparse LU factorization failed");
        }
      };

  double* v_buffer = nullptr;
  double* i_buffer = nullptr;
  ctx_YMatrix_getVpointer(api.get(), &v_buffer);
  api.check("ctx_YMatrix_getVpointer");
  ctx_YMatrix_getIpointer(api.get(), &i_buffer);
  api.check("ctx_YMatrix_getIpointer");
  if (v_buffer == nullptr) {
    throw std::runtime_error("solve_opendss_ymatrix: null V buffer");
  }
  if (i_buffer == nullptr) {
    throw std::runtime_error("solve_opendss_ymatrix: null I buffer");
  }

  {
    const std::vector<std::string> all_node_names =
        dss_get_string_array(api, ctx_Circuit_Get_AllNodeNames);
    const std::vector<double> all_bus_volts =
        dss_get_double_array(api, ctx_Circuit_Get_AllBusVolts);
    std::unordered_map<std::string, Complex> solved_node_voltage;
    const std::size_t available_pairs =
        std::min(all_node_names.size(), all_bus_volts.size() / 2);
    solved_node_voltage.reserve(available_pairs);
    for (std::size_t idx = 0; idx < available_pairs; ++idx) {
      solved_node_voltage.emplace(
          lowercase_ascii_copy(all_node_names[idx]),
          Complex(all_bus_volts[2 * idx], all_bus_volts[2 * idx + 1]));
    }

    v_buffer[0] = 0.0;
    v_buffer[1] = 0.0;
    for (std::size_t idx = 0; idx < node_order.size(); ++idx) {
      const auto it = solved_node_voltage.find(lowercase_ascii_copy(node_order[idx]));
      const Complex voltage = (it == solved_node_voltage.end()) ? Complex(0.0, 0.0) : it->second;
      const std::size_t buffer_slot = idx + 1;
      v_buffer[2 * buffer_slot] = voltage.real();
      v_buffer[2 * buffer_slot + 1] = voltage.imag();
    }
  }

  Eigen::SparseMatrix<Complex> y_sparse = build_sparse_y_matrix();
  Eigen::SparseLU<Eigen::SparseMatrix<Complex>> y_lu;
  factorize_y_matrix(y_sparse, y_lu);

  std::vector<double> last_vmag(node_order.size(), 0.0);
  OpenDSSYMatrixSolveResult result;
  Eigen::VectorXcd node_voltage = Eigen::VectorXcd::Zero(static_cast<int>(node_order.size()));

  for (int iter = 1; iter <= max_iter; ++iter) {
    ctx_YMatrix_ZeroInjCurr(api.get());
    api.check("ctx_YMatrix_ZeroInjCurr");
    ctx_YMatrix_GetSourceInjCurrents(api.get());
    api.check("ctx_YMatrix_GetSourceInjCurrents");
    ctx_YMatrix_GetPCInjCurr(api.get());
    api.check("ctx_YMatrix_GetPCInjCurr");

    if (ctx_YMatrix_Get_SystemYChanged(api.get()) != 0) {
      api.check("ctx_YMatrix_Get_SystemYChanged");
      ctx_YMatrix_BuildYMatrixD(api.get(), 2, 1);
      api.check("ctx_YMatrix_BuildYMatrixD");
      ctx_YMatrix_getVpointer(api.get(), &v_buffer);
      api.check("ctx_YMatrix_getVpointer");
      ctx_YMatrix_getIpointer(api.get(), &i_buffer);
      api.check("ctx_YMatrix_getIpointer");
      if (v_buffer == nullptr || i_buffer == nullptr) {
        throw std::runtime_error("solve_opendss_ymatrix: YMatrix buffers became null after rebuild");
      }
      y_sparse = build_sparse_y_matrix();
      factorize_y_matrix(y_sparse, y_lu);
    } else {
      api.check("ctx_YMatrix_Get_SystemYChanged");
    }

    if (ctx_YMatrix_Get_UseAuxCurrents(api.get()) != 0) {
      api.check("ctx_YMatrix_Get_UseAuxCurrents");
      ctx_YMatrix_AddInAuxCurrents(api.get(), 0);
      api.check("ctx_YMatrix_AddInAuxCurrents");
    } else {
      api.check("ctx_YMatrix_Get_UseAuxCurrents");
    }

    Eigen::VectorXcd node_current(static_cast<int>(node_order.size()));
    for (std::size_t idx = 0; idx < node_order.size(); ++idx) {
      const std::size_t buffer_slot = idx + 1;
      node_current(static_cast<int>(idx)) =
          Complex(i_buffer[2 * buffer_slot], i_buffer[2 * buffer_slot + 1]);
    }

    node_voltage = y_lu.solve(node_current);
    if (y_lu.info() != Eigen::Success ||
        node_voltage.size() != static_cast<int>(node_order.size())) {
      throw std::runtime_error("solve_opendss_ymatrix: linear solve failed");
    }

    v_buffer[0] = 0.0;
    v_buffer[1] = 0.0;
    for (std::size_t idx = 0; idx < node_order.size(); ++idx) {
      const std::size_t buffer_slot = idx + 1;
      v_buffer[2 * buffer_slot] = node_voltage(static_cast<int>(idx)).real();
      v_buffer[2 * buffer_slot + 1] = node_voltage(static_cast<int>(idx)).imag();
    }

    double max_change = 0.0;
    for (std::size_t idx = 0; idx < node_order.size(); ++idx) {
      const double vmag = std::abs(node_voltage(static_cast<int>(idx)));
      max_change = std::max(max_change, std::abs(vmag - last_vmag[idx]));
      last_vmag[idx] = vmag;
    }

    result.iterations = iter;
    result.residual = max_change;
    if (max_change < tol && iter > 1) {
      result.converged = true;
      break;
    }
  }

  for (std::size_t idx = 0; idx < node_order.size(); ++idx) {
    const ParsedDSSBusTerminal node = parse_dss_bus_terminal(node_order[idx], 1);
    if (node.nodes.empty()) continue;
    const int phase = node.nodes.front();
    if (phase < 1 || phase > 3) continue;
    result.voltage_map[node.bus_name][static_cast<std::size_t>(phase - 1)] =
        node_voltage(static_cast<int>(idx));
  }

  return result;
}
#endif

}  // namespace

std::unordered_map<std::string, double> get_opendss_regulator_taps(
    const std::filesystem::path& master_dss) {
  std::unordered_map<std::string, double> reg_taps;
#ifdef HACDCPF_HAVE_OPENDSS
  const io::OpenDSSSnapshotResult snapshot = io::solve_opendss_snapshot(master_dss);
  for (const auto& transformer : snapshot.transformer_states) {
    double tap_pu = 1.0;
    for (const auto& winding : transformer.winding_states) {
      if (winding.winding == 2) {
        tap_pu = winding.tap_pu;
        break;
      }
      tap_pu = winding.tap_pu;
    }
    reg_taps[lowercase_ascii_copy(transformer.name)] = tap_pu;
  }
  for (const auto& regcontrol : snapshot.regcontrol_results) {
    auto transformer_it =
        reg_taps.find(lowercase_ascii_copy(regcontrol.transformer_name));
    if (transformer_it != reg_taps.end()) {
      reg_taps[lowercase_ascii_copy(regcontrol.name)] = transformer_it->second;
    }
  }
#else
  (void)master_dss;
#endif
  return reg_taps;
}

OpenDSSSparseYMatrix build_opendss_sparse_y_matrix(
    const std::filesystem::path& master_dss,
    const std::unordered_map<std::string, double>& reg_taps) {
  OpenDSSSparseYMatrix snapshot;
#ifdef HACDCPF_HAVE_OPENDSS
  LocalDSSContext api;
  compile_master_dss(api, master_dss);

  for (const auto& [name, tap_pu] : reg_taps) {
    dss_run_command(
        api,
        "Transformer." + name + ".wdg=2 Tap=" + std::to_string(tap_pu));
  }
  dss_run_command(api, "Set ControlMode=OFF");
  ctx_Solution_InitSnap(api.get());
  api.check("ctx_Solution_InitSnap");
  ctx_YMatrix_BuildYMatrixD(api.get(), 2, 1);
  api.check("ctx_YMatrix_BuildYMatrixD");

  snapshot.node_order = dss_get_string_array(api, ctx_Circuit_Get_YNodeOrder);
  snapshot.dimension = static_cast<int>(snapshot.node_order.size());
  if (snapshot.dimension == 0) {
    throw std::runtime_error("build_opendss_sparse_y_matrix: empty YNodeOrder");
  }

  uint32_t n_bus = 0;
  uint32_t n_nz = 0;
  int32_t* col_ptr = nullptr;
  int32_t* row_idx = nullptr;
  double* c_vals = nullptr;
  ctx_YMatrix_GetCompressedYMatrix(
      api.get(),
      2,
      &n_bus,
      &n_nz,
      &col_ptr,
      &row_idx,
      &c_vals);
  api.check("ctx_YMatrix_GetCompressedYMatrix");

  const int matrix_dim = static_cast<int>(n_bus);
  if (matrix_dim != snapshot.dimension) {
    throw std::runtime_error(
        "build_opendss_sparse_y_matrix: compressed matrix dimension does not match YNodeOrder");
  }
  if (col_ptr == nullptr || row_idx == nullptr || c_vals == nullptr) {
    throw std::runtime_error(
        "build_opendss_sparse_y_matrix: compressed matrix buffers are null");
  }

  const int pointer_offset = (col_ptr[0] == 1) ? 1 : 0;
  snapshot.entries.reserve(static_cast<std::size_t>(n_nz));
  for (int col = 0; col < matrix_dim; ++col) {
    const int begin = col_ptr[col] - pointer_offset;
    const int end = col_ptr[col + 1] - pointer_offset;
    if (begin < 0 || end < begin || end > static_cast<int>(n_nz)) {
      throw std::runtime_error(
          "build_opendss_sparse_y_matrix: invalid compressed column pointer range");
    }
    for (int pos = begin; pos < end; ++pos) {
      const int row = row_idx[pos] - pointer_offset;
      if (row < 0 || row >= matrix_dim) {
        throw std::runtime_error(
            "build_opendss_sparse_y_matrix: invalid compressed row index");
      }
      const Complex value(c_vals[2 * pos], c_vals[2 * pos + 1]);
      if (std::abs(value) <= 1e-14) continue;
      snapshot.entries.push_back({
          .row = row,
          .col = col,
          .value = value,
      });
    }
  }
#else
  (void)master_dss;
  (void)reg_taps;
  throw std::runtime_error(
      "build_opendss_sparse_y_matrix requires HACDCPF_HAVE_OPENDSS");
#endif
  return snapshot;
}

ThreePhaseACSystem load_three_phase_system_from_opendss(
    const std::filesystem::path& master_dss,
    double base_mva) {
  #ifdef HACDCPF_HAVE_OPENDSS
  io::ScopedDSSFloatingPointEnv fp_env;
  #endif
  ThreePhaseACSystem sys;
  sys.base_mva = (base_mva > 0.0) ? base_mva : 1.0;
  sys.name = master_dss.filename().string();

#ifdef HACDCPF_HAVE_OPENDSS
  LocalDSSContext api;
  compile_master_dss(api, master_dss);
  ctx_Solution_Solve(api.get());
  api.check("ctx_Solution_Solve");
  sys.base_freq_hz = dss_get_double_value(
      api,
      ctx_Solution_Get_Frequency,
      "ctx_Solution_Get_Frequency");
  const auto transformer_tap_contracts =
      collect_dss_transformer_tap_contracts(master_dss);

  std::unordered_map<std::string, int> bus_id_by_name;
  const std::vector<std::string> bus_names =
      dss_get_string_array(api, ctx_Circuit_Get_AllBusNames);
  sys.buses.reserve(bus_names.size());
  for (const auto& raw_bus_name : bus_names) {
    const std::string bus_name = lowercase_ascii_copy(raw_bus_name);
    ctx_Circuit_SetActiveBus(api.get(), raw_bus_name.c_str());
    api.check("ctx_Circuit_SetActiveBus");

    const std::vector<int32_t> nodes_raw =
        dss_get_int_array(api, ctx_Bus_Get_Nodes);
    std::vector<int> nodes;
    nodes.reserve(nodes_raw.size());
    for (const int32_t value : nodes_raw) {
      if (value >= 1 && value <= 3) {
        nodes.push_back(static_cast<int>(value));
      }
    }
    std::sort(nodes.begin(), nodes.end());
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
    const PhaseMask phase_mask = phase_mask_from_node_numbers(nodes);

    ThreePhaseACBus bus;
    bus.index = static_cast<int>(sys.buses.size()) + 1;
    bus.name = bus_name;
    bus.phase_mask = phase_mask;
    bus.base_kv = bus_base_kv_from_dss(
        dss_get_double_value(api, ctx_Bus_Get_kVBase, "ctx_Bus_Get_kVBase"),
        phase_mask);

    const std::vector<double> pu_mag_angle =
        dss_get_double_array(api, ctx_Bus_Get_puVmagAngle);
    for (std::size_t pos = 0; pos < nodes.size() && (2 * pos + 1) < pu_mag_angle.size(); ++pos) {
      const int phase = nodes[pos] - 1;
      if (phase == 0) {
        bus.vm_a_pu = pu_mag_angle[2 * pos];
        bus.va_a_deg = pu_mag_angle[2 * pos + 1];
      } else if (phase == 1) {
        bus.vm_b_pu = pu_mag_angle[2 * pos];
        bus.va_b_deg = pu_mag_angle[2 * pos + 1];
      } else {
        bus.vm_c_pu = pu_mag_angle[2 * pos];
        bus.va_c_deg = pu_mag_angle[2 * pos + 1];
      }
    }

    bus_id_by_name[bus_name] = bus.index;
    sys.buses.push_back(std::move(bus));
  }

  auto find_bus = [&](const std::string& name) -> ThreePhaseACBus& {
    const auto it = bus_id_by_name.find(lowercase_ascii_copy(name));
    if (it == bus_id_by_name.end()) {
      throw std::runtime_error("Unknown OpenDSS bus " + name);
    }
    return sys.buses[static_cast<std::size_t>(it->second - 1)];
  };

  for (int has_line = ctx_Lines_Get_First(api.get());
       has_line != 0;
       has_line = ctx_Lines_Get_Next(api.get())) {
    api.check("ctx_Lines_Get_First/Next");
    const std::string line_name =
        lowercase_ascii_copy(dss_get_string_value(api, ctx_Lines_Get_Name, "ctx_Lines_Get_Name"));
    ctx_Circuit_SetActiveElement(api.get(), ("Line." + line_name).c_str());
    api.check("ctx_Circuit_SetActiveElement");
    if (ctx_CktElement_Get_Enabled(api.get()) == 0) {
      api.check("ctx_CktElement_Get_Enabled");
      continue;
    }
    api.check("ctx_CktElement_Get_Enabled");

    const int phases = dss_get_int_value(api, ctx_Lines_Get_Phases, "ctx_Lines_Get_Phases");
    const ParsedDSSBusTerminal from_terminal =
        parse_dss_bus_terminal(
            dss_get_string_value(api, ctx_Lines_Get_Bus1, "ctx_Lines_Get_Bus1"),
            phases);
    const ParsedDSSBusTerminal to_terminal =
        parse_dss_bus_terminal(
            dss_get_string_value(api, ctx_Lines_Get_Bus2, "ctx_Lines_Get_Bus2"),
            phases);

    ThreePhaseACLine line;
    line.index = static_cast<int>(sys.lines.size()) + 1;
    line.name = line_name;
    line.from_bus = bus_id_by_name.at(from_terminal.bus_name);
    line.to_bus = bus_id_by_name.at(to_terminal.bus_name);
    line.phase_mask = PhaseMask(
        static_cast<std::uint8_t>(from_terminal.phase_mask.bits & to_terminal.phase_mask.bits));
    if (line.phase_mask.empty()) {
      line.phase_mask = from_terminal.phase_mask;
    }

    const double length_value =
        dss_get_double_value(api, ctx_Lines_Get_Length, "ctx_Lines_Get_Length");
    const std::string line_units =
        normalize_dss_length_units(dss_active_property(api, "units"), "kft");
    const std::string impedance_units =
        dss_line_impedance_units(api, line_units);
    line.length_km = dss_length_to_km(length_value, line_units);
    const ThreePhaseACBus& from_bus =
        sys.buses[static_cast<std::size_t>(line.from_bus - 1)];
    const double z_base = bus_base_impedance_ohm(from_bus, sys.base_mva);
    const std::vector<int> phase_slots = dss_phase_slots(line.phase_mask);
    const std::vector<double> rmatrix = dss_get_double_array(api, ctx_Lines_Get_Rmatrix);
    const std::vector<double> xmatrix = dss_get_double_array(api, ctx_Lines_Get_Xmatrix);
    const std::vector<double> cmatrix = dss_get_double_array(api, ctx_Lines_Get_Cmatrix);
    const double impedance_len_base_km = dss_length_to_km(1.0, impedance_units);
    const double length_scale =
        (impedance_len_base_km > 0.0) ? (line.length_km / impedance_len_base_km) : 0.0;
    if (!rmatrix.empty() || !xmatrix.empty()) {
      line.use_phase_matrix = true;
      line.r_matrix_pu =
          expand_dss_phase_matrix(rmatrix, phase_slots, length_scale / z_base);
      line.x_matrix_pu =
          expand_dss_phase_matrix(xmatrix, phase_slots, length_scale / z_base);
      line.b_matrix_pu = expand_dss_phase_matrix(
          cmatrix,
          phase_slots,
          2.0 * kPi * sys.base_freq_hz * 1e-9 * length_scale * z_base);
    }

    line.r1_ohm_per_km =
        dss_impedance_to_ohm_per_km(
            dss_get_double_value(api, ctx_Lines_Get_R1, "ctx_Lines_Get_R1"),
            impedance_units);
    line.x1_ohm_per_km =
        dss_impedance_to_ohm_per_km(
            dss_get_double_value(api, ctx_Lines_Get_X1, "ctx_Lines_Get_X1"),
            impedance_units);
    line.r0_ohm_per_km =
        dss_impedance_to_ohm_per_km(
            dss_get_double_value(api, ctx_Lines_Get_R0, "ctx_Lines_Get_R0"),
            impedance_units);
    line.x0_ohm_per_km =
        dss_impedance_to_ohm_per_km(
            dss_get_double_value(api, ctx_Lines_Get_X0, "ctx_Lines_Get_X0"),
            impedance_units);
    line.c1_nf_per_km =
        dss_capacitance_to_nf_per_km(
            dss_get_double_value(api, ctx_Lines_Get_C1, "ctx_Lines_Get_C1"),
            impedance_units);
    line.c0_nf_per_km =
        dss_capacitance_to_nf_per_km(
            dss_get_double_value(api, ctx_Lines_Get_C0, "ctx_Lines_Get_C0"),
            impedance_units);
    line.r1_pu = line.r1_ohm_per_km * line.length_km / z_base;
    line.x1_pu = line.x1_ohm_per_km * line.length_km / z_base;
    line.r0_pu = line.r0_ohm_per_km * line.length_km / z_base;
    line.x0_pu = line.x0_ohm_per_km * line.length_km / z_base;
    line.b1_pu =
        2.0 * kPi * sys.base_freq_hz * line.c1_nf_per_km * 1e-9 * line.length_km * z_base;
    line.b0_pu =
        2.0 * kPi * sys.base_freq_hz * line.c0_nf_per_km * 1e-9 * line.length_km * z_base;
    sys.lines.push_back(std::move(line));
  }

  for (int has_load = ctx_Loads_Get_First(api.get());
       has_load != 0;
       has_load = ctx_Loads_Get_Next(api.get())) {
    api.check("ctx_Loads_Get_First/Next");
    const std::string load_name =
        lowercase_ascii_copy(dss_get_string_value(api, ctx_Loads_Get_Name, "ctx_Loads_Get_Name"));
    ctx_Circuit_SetActiveElement(api.get(), ("Load." + load_name).c_str());
    api.check("ctx_Circuit_SetActiveElement");
    if (ctx_CktElement_Get_Enabled(api.get()) == 0) {
      api.check("ctx_CktElement_Get_Enabled");
      continue;
    }
    api.check("ctx_CktElement_Get_Enabled");

    const int phases = dss_get_int_value(api, ctx_Loads_Get_Phases, "ctx_Loads_Get_Phases");
    const std::vector<std::string> bus_names_local =
        dss_get_string_array(api, ctx_CktElement_Get_BusNames, uint16_t{0});
    if (bus_names_local.empty()) continue;
    const ParsedDSSBusTerminal terminal =
        parse_dss_bus_terminal(bus_names_local.front(), phases);

    ThreePhaseLoad load;
    load.index = static_cast<int>(sys.loads.size()) + 1;
    load.name = load_name;
    load.bus = bus_id_by_name.at(terminal.bus_name);
    load.phase_mask = terminal.phase_mask;
    load.connection =
        (dss_get_int_value(api, ctx_Loads_Get_IsDelta, "ctx_Loads_Get_IsDelta") != 0)
            ? "delta"
            : "wye";
    const double load_rneut =
        dss_get_double_value(api, ctx_Loads_Get_Rneut, "ctx_Loads_Get_Rneut");
    const double load_xneut =
        dss_get_double_value(api, ctx_Loads_Get_Xneut, "ctx_Loads_Get_Xneut");
    const bool explicit_single_terminal_wye = bus_names_local.size() <= 1;
    load.grounded =
        (load.connection == "delta") ||
        explicit_single_terminal_wye ||
        terminal.has_ground_reference ||
        !(load_rneut < 0.0 && load_xneut < 0.0);
    load.r_neut_ohm = std::max(
        0.0,
        load_rneut);
    load.x_neut_ohm = std::max(
        0.0,
        load_xneut);
    load.vmin_pu =
        dss_get_double_value(api, ctx_Loads_Get_Vminpu, "ctx_Loads_Get_Vminpu");
    load.vmax_pu =
        dss_get_double_value(api, ctx_Loads_Get_Vmaxpu, "ctx_Loads_Get_Vmaxpu");

    const double p_total_mw =
        dss_get_double_value(api, ctx_Loads_Get_kW, "ctx_Loads_Get_kW") / 1000.0;
    const double q_total_mvar =
        dss_get_double_value(api, ctx_Loads_Get_kvar, "ctx_Loads_Get_kvar") / 1000.0;
    const int phase_count = std::max(1, load.phase_mask.count());
    const double p_per_phase = p_total_mw / static_cast<double>(phase_count);
    const double q_per_phase = q_total_mvar / static_cast<double>(phase_count);
    if (load.phase_mask.has(0)) { load.p_a_mw = p_per_phase; load.q_a_mvar = q_per_phase; }
    if (load.phase_mask.has(1)) { load.p_b_mw = p_per_phase; load.q_b_mvar = q_per_phase; }
    if (load.phase_mask.has(2)) { load.p_c_mw = p_per_phase; load.q_c_mvar = q_per_phase; }

    const std::vector<double> zipv = dss_get_double_array(api, ctx_Loads_Get_ZIPV);
    bool explicit_zip_loaded = false;
    if (zipv.size() >= 6) {
      const double p_sum = zipv[0] + zipv[1] + zipv[2];
      const double q_sum = zipv[3] + zipv[4] + zipv[5];
      if (p_sum > 1e-9 && q_sum > 1e-9) {
        load.p_const_z_percent = zipv[0] * 100.0;
        load.p_const_i_percent = zipv[1] * 100.0;
        load.p_const_p_percent = zipv[2] * 100.0;
        load.q_const_z_percent = zipv[3] * 100.0;
        load.q_const_i_percent = zipv[4] * 100.0;
        load.q_const_p_percent = zipv[5] * 100.0;
        explicit_zip_loaded = true;
      }
    }
    if (!explicit_zip_loaded) {
      const int model = dss_get_int_value(api, ctx_Loads_Get_Model, "ctx_Loads_Get_Model");
      if (model == 2) {
        load.const_z_percent = 100.0;
        load.const_i_percent = 0.0;
        load.const_p_percent = 0.0;
      } else if (model == 5) {
        load.const_z_percent = 0.0;
        load.const_i_percent = 100.0;
        load.const_p_percent = 0.0;
      }
    }
    sys.loads.push_back(std::move(load));
  }

  for (int has_cap = ctx_Capacitors_Get_First(api.get());
       has_cap != 0;
       has_cap = ctx_Capacitors_Get_Next(api.get())) {
    api.check("ctx_Capacitors_Get_First/Next");
    const std::string cap_name = lowercase_ascii_copy(
        dss_get_string_value(api, ctx_Capacitors_Get_Name, "ctx_Capacitors_Get_Name"));
    ctx_Circuit_SetActiveElement(api.get(), ("Capacitor." + cap_name).c_str());
    api.check("ctx_Circuit_SetActiveElement");
    if (ctx_CktElement_Get_Enabled(api.get()) == 0) {
      api.check("ctx_CktElement_Get_Enabled");
      continue;
    }
    api.check("ctx_CktElement_Get_Enabled");
    const std::vector<std::string> cap_bus_names =
        dss_get_string_array(api, ctx_CktElement_Get_BusNames, uint16_t{0});
    if (cap_bus_names.empty()) continue;

    const std::vector<int32_t> node_order =
        dss_get_int_array(api, ctx_CktElement_Get_NodeOrder);
    std::vector<int> phase_nodes;
    phase_nodes.reserve(node_order.size());
    for (const int32_t value : node_order) {
      if (value >= 1 && value <= 3) phase_nodes.push_back(static_cast<int>(value));
    }
    std::sort(phase_nodes.begin(), phase_nodes.end());
    phase_nodes.erase(std::unique(phase_nodes.begin(), phase_nodes.end()), phase_nodes.end());
    const ParsedDSSBusTerminal terminal =
        parse_dss_bus_terminal(cap_bus_names.front(), static_cast<int>(phase_nodes.size()));
    ThreePhaseACBus& bus = find_bus(terminal.bus_name);
    const int phase_count = std::max(1, terminal.phase_mask.count());
    const double q_per_phase_mvar =
        dss_get_double_value(api, ctx_Capacitors_Get_kvar, "ctx_Capacitors_Get_kvar") /
        1000.0 / static_cast<double>(phase_count);
    if (terminal.phase_mask.has(0)) bus.bs_a_mvar += q_per_phase_mvar;
    if (terminal.phase_mask.has(1)) bus.bs_b_mvar += q_per_phase_mvar;
    if (terminal.phase_mask.has(2)) bus.bs_c_mvar += q_per_phase_mvar;
  }

  std::unordered_map<std::string, int> transformer_index_by_name;
  auto get_or_create_internal_bus =
      [&](const std::string& raw_name,
          PhaseMask phase_mask,
          double base_kv) -> int {
        const std::string bus_name = lowercase_ascii_copy(raw_name);
        const auto it = bus_id_by_name.find(bus_name);
        if (it != bus_id_by_name.end()) {
          auto& bus = sys.buses[static_cast<std::size_t>(it->second - 1)];
          bus.phase_mask = PhaseMask(
              static_cast<std::uint8_t>(bus.phase_mask.bits | phase_mask.bits));
          if (bus.base_kv <= 0.0 && base_kv > 0.0) {
            bus.base_kv = base_kv;
          }
          return it->second;
        }

        ThreePhaseACBus bus;
        bus.index = static_cast<int>(sys.buses.size()) + 1;
        bus.name = bus_name;
        bus.phase_mask = phase_mask;
        bus.base_kv = base_kv;
        bus_id_by_name[bus.name] = bus.index;
        sys.buses.push_back(std::move(bus));
        return sys.buses.back().index;
      };

  auto add_three_winding_star_leg =
      [&](const std::string& leg_name,
          int from_bus,
          PhaseMask from_phase_mask,
          int star_bus,
          PhaseMask star_phase_mask,
          double from_kv,
          double star_kv,
          double sn_mva,
          bool from_delta,
          double tap_ratio,
          Complex z_star_pu) {
        ThreePhaseTransformer transformer;
        transformer.index = static_cast<int>(sys.transformers.size()) + 1;
        transformer.name = leg_name;
        transformer.sn_mva = sn_mva;
        transformer.vkr_percent =
            std::max(1e-6, z_star_pu.real()) * 100.0;
        transformer.vk_percent =
            std::max(1e-6, std::abs(z_star_pu)) * 100.0;
        const double effective_from_kv =
            from_kv * ((tap_ratio > 0.0) ? tap_ratio : 1.0);
        if (effective_from_kv >= star_kv) {
          transformer.hv_bus = from_bus;
          transformer.lv_bus = star_bus;
          transformer.hv_phase_mask = from_phase_mask;
          transformer.lv_phase_mask = star_phase_mask;
          transformer.vn_hv_kv = effective_from_kv;
          transformer.vn_lv_kv = star_kv;
          transformer.vector_group = infer_transformer_vector_group(
              from_delta,
              false,
              !from_delta,
              true,
              from_delta ? 1 : 0);
        } else {
          transformer.hv_bus = star_bus;
          transformer.lv_bus = from_bus;
          transformer.hv_phase_mask = star_phase_mask;
          transformer.lv_phase_mask = from_phase_mask;
          transformer.vn_hv_kv = star_kv;
          transformer.vn_lv_kv = effective_from_kv;
          transformer.vector_group = infer_transformer_vector_group(
              false,
              from_delta,
              true,
              !from_delta,
              from_delta ? 1 : 0);
        }
        transformer_index_by_name[transformer.name] = transformer.index;
        sys.transformers.push_back(std::move(transformer));
      };

  for (int has_transformer = ctx_Transformers_Get_First(api.get());
       has_transformer != 0;
       has_transformer = ctx_Transformers_Get_Next(api.get())) {
    api.check("ctx_Transformers_Get_First/Next");
    const std::string transformer_name = lowercase_ascii_copy(
        dss_get_string_value(api, ctx_Transformers_Get_Name, "ctx_Transformers_Get_Name"));
    ctx_Circuit_SetActiveElement(api.get(), ("Transformer." + transformer_name).c_str());
    api.check("ctx_Circuit_SetActiveElement");
    if (ctx_CktElement_Get_Enabled(api.get()) == 0) {
      api.check("ctx_CktElement_Get_Enabled");
      continue;
    }
    api.check("ctx_CktElement_Get_Enabled");

    const int windings = dss_get_int_value(
        api,
        ctx_Transformers_Get_NumWindings,
        "ctx_Transformers_Get_NumWindings");
    if (windings == 3) {
      const std::vector<std::string> transformer_bus_names =
          dss_get_string_array(api, ctx_CktElement_Get_BusNames, uint16_t{0});
      if (transformer_bus_names.size() < 3) {
        throw std::runtime_error(
            "load_three_phase_system_from_opendss: transformer " + transformer_name +
            " has incomplete 3-winding bus terminals");
      }

      std::array<ParsedDSSBusTerminal, 3> terminals = {
          parse_dss_bus_terminal(transformer_bus_names[0]),
          parse_dss_bus_terminal(transformer_bus_names[1]),
          parse_dss_bus_terminal(transformer_bus_names[2]),
      };
      std::array<bool, 3> is_delta{};
      std::array<double, 3> kvs{};
      std::array<double, 3> kvas{};
      std::array<double, 3> taps{1.0, 1.0, 1.0};
      std::array<double, 3> r_percent{};
      for (int winding = 1; winding <= 3; ++winding) {
        ctx_Transformers_Set_Wdg(api.get(), winding);
        api.check("ctx_Transformers_Set_Wdg");
        is_delta[static_cast<std::size_t>(winding - 1)] =
            dss_get_int_value(api, ctx_Transformers_Get_IsDelta, "ctx_Transformers_Get_IsDelta") != 0;
        kvs[static_cast<std::size_t>(winding - 1)] =
            dss_get_double_value(api, ctx_Transformers_Get_kV, "ctx_Transformers_Get_kV");
        kvas[static_cast<std::size_t>(winding - 1)] =
            dss_get_double_value(api, ctx_Transformers_Get_kVA, "ctx_Transformers_Get_kVA");
        taps[static_cast<std::size_t>(winding - 1)] =
            dss_get_double_value(api, ctx_Transformers_Get_Tap, "ctx_Transformers_Get_Tap");
        r_percent[static_cast<std::size_t>(winding - 1)] =
            dss_get_double_value(api, ctx_Transformers_Get_R, "ctx_Transformers_Get_R");
      }

      const Complex z_hl(
          (r_percent[0] + r_percent[1]) / 100.0,
          dss_get_double_value(api, ctx_Transformers_Get_Xhl, "ctx_Transformers_Get_Xhl") / 100.0);
      const Complex z_ht(
          (r_percent[0] + r_percent[2]) / 100.0,
          dss_get_double_value(api, ctx_Transformers_Get_Xht, "ctx_Transformers_Get_Xht") / 100.0);
      const Complex z_lt(
          (r_percent[1] + r_percent[2]) / 100.0,
          dss_get_double_value(api, ctx_Transformers_Get_Xlt, "ctx_Transformers_Get_Xlt") / 100.0);
      const Complex z_h = 0.5 * (z_hl + z_ht - z_lt);
      const Complex z_m = 0.5 * (z_hl + z_lt - z_ht);
      const Complex z_l = 0.5 * (z_ht + z_lt - z_hl);

      PhaseMask star_phase_mask = PhaseMask::none();
      for (const auto& terminal : terminals) {
        star_phase_mask = PhaseMask(
            static_cast<std::uint8_t>(star_phase_mask.bits | terminal.phase_mask.bits));
      }
      const std::string star_bus_name = "__t3w_" + transformer_name + "_star";
      const int star_bus = get_or_create_internal_bus(
          star_bus_name,
          star_phase_mask,
          kvs[0]);

      add_three_winding_star_leg(
          transformer_name + "_hv",
          bus_id_by_name.at(terminals[0].bus_name),
          terminals[0].phase_mask,
          star_bus,
          terminals[0].phase_mask,
          kvs[0],
          kvs[0],
          kvas[0] / 1000.0,
          is_delta[0],
          taps[0],
          z_h);
      add_three_winding_star_leg(
          transformer_name + "_mv",
          bus_id_by_name.at(terminals[1].bus_name),
          terminals[1].phase_mask,
          star_bus,
          terminals[1].phase_mask,
          kvs[1],
          kvs[0],
          kvas[1] / 1000.0,
          is_delta[1],
          taps[1],
          z_m);
      add_three_winding_star_leg(
          transformer_name + "_lv",
          bus_id_by_name.at(terminals[2].bus_name),
          terminals[2].phase_mask,
          star_bus,
          terminals[2].phase_mask,
          kvs[2],
          kvs[0],
          kvas[2] / 1000.0,
          is_delta[2],
          taps[2],
          z_l);
      continue;
    }
    if (windings != 2) {
      throw std::runtime_error(
          "load_three_phase_system_from_opendss: transformer " + transformer_name +
          " requires unsupported num_windings=" + std::to_string(windings));
    }

    const std::vector<std::string> transformer_bus_names =
        dss_get_string_array(api, ctx_CktElement_Get_BusNames, uint16_t{0});
    if (transformer_bus_names.size() < 2) continue;
    const ParsedDSSBusTerminal hv_terminal = parse_dss_bus_terminal(transformer_bus_names[0]);
    const ParsedDSSBusTerminal lv_terminal = parse_dss_bus_terminal(transformer_bus_names[1]);

    std::array<bool, 2> is_delta{false, false};
    std::array<bool, 2> grounded_wye{true, true};
    std::array<double, 2> kvs{};
    std::array<double, 2> kvas{};
    std::array<double, 2> taps{1.0, 1.0};
    std::array<double, 2> min_taps{1.0, 1.0};
    std::array<double, 2> max_taps{1.0, 1.0};
    std::array<int, 2> num_taps{};
    std::array<double, 2> r_percent{};
    for (int winding = 1; winding <= 2; ++winding) {
      ctx_Transformers_Set_Wdg(api.get(), winding);
      api.check("ctx_Transformers_Set_Wdg");
      is_delta[static_cast<std::size_t>(winding - 1)] =
          dss_get_int_value(api, ctx_Transformers_Get_IsDelta, "ctx_Transformers_Get_IsDelta") != 0;
      const bool explicit_ground_reference =
          (winding == 1) ? hv_terminal.has_ground_reference : lv_terminal.has_ground_reference;
      const double winding_rneut =
          dss_get_double_value(api, ctx_Transformers_Get_Rneut, "ctx_Transformers_Get_Rneut");
      const double winding_xneut =
          dss_get_double_value(api, ctx_Transformers_Get_Xneut, "ctx_Transformers_Get_Xneut");
      grounded_wye[static_cast<std::size_t>(winding - 1)] =
          is_delta[static_cast<std::size_t>(winding - 1)] ||
          explicit_ground_reference ||
          !(winding_rneut < 0.0 && winding_xneut < 0.0);
      kvs[static_cast<std::size_t>(winding - 1)] =
          dss_get_double_value(api, ctx_Transformers_Get_kV, "ctx_Transformers_Get_kV");
      kvas[static_cast<std::size_t>(winding - 1)] =
          dss_get_double_value(api, ctx_Transformers_Get_kVA, "ctx_Transformers_Get_kVA");
      taps[static_cast<std::size_t>(winding - 1)] =
          dss_get_double_value(api, ctx_Transformers_Get_Tap, "ctx_Transformers_Get_Tap");
      min_taps[static_cast<std::size_t>(winding - 1)] =
          dss_get_double_value(api, ctx_Transformers_Get_MinTap, "ctx_Transformers_Get_MinTap");
      max_taps[static_cast<std::size_t>(winding - 1)] =
          dss_get_double_value(api, ctx_Transformers_Get_MaxTap, "ctx_Transformers_Get_MaxTap");
      num_taps[static_cast<std::size_t>(winding - 1)] =
          dss_get_int_value(api, ctx_Transformers_Get_NumTaps, "ctx_Transformers_Get_NumTaps");
      r_percent[static_cast<std::size_t>(winding - 1)] =
          dss_get_double_value(api, ctx_Transformers_Get_R, "ctx_Transformers_Get_R");
    }

    if (const auto contract_it = transformer_tap_contracts.find(transformer_name);
        contract_it != transformer_tap_contracts.end()) {
      for (std::size_t tap_slot = 0; tap_slot < 2; ++tap_slot) {
        if (!contract_it->second.has_explicit_tap[tap_slot]) continue;
        taps[tap_slot] = contract_it->second.tap_pu[tap_slot];
      }
    }

    ThreePhaseTransformer transformer;
    transformer.index = static_cast<int>(sys.transformers.size()) + 1;
    transformer.name = transformer_name;
    transformer.hv_bus = bus_id_by_name.at(hv_terminal.bus_name);
    transformer.lv_bus = bus_id_by_name.at(lv_terminal.bus_name);
    transformer.hv_phase_mask = hv_terminal.phase_mask;
    transformer.lv_phase_mask = lv_terminal.phase_mask;
    transformer.sn_mva = std::min(kvas[0], kvas[1]) / 1000.0;
    transformer.vn_hv_kv = kvs[0];
    transformer.vn_lv_kv = kvs[1];
    transformer.vkr_percent = r_percent[0] + r_percent[1];
    transformer.vk_percent = std::hypot(
        transformer.vkr_percent,
        dss_get_double_value(api, ctx_Transformers_Get_Xhl, "ctx_Transformers_Get_Xhl"));
    const int clock =
        (is_delta[0] != is_delta[1])
            ? transformer_clock_from_leadlag(dss_active_property(api, "leadlag"))
            : 0;
    transformer.vector_group = infer_transformer_vector_group(
        is_delta[0],
        is_delta[1],
        grounded_wye[0],
        grounded_wye[1],
        clock);

    const bool tap_on_hv = std::abs(taps[0] - 1.0) > std::abs(taps[1] - 1.0);
    const int tap_side = tap_on_hv ? 0 : 1;
    const int tap_slot = tap_on_hv ? 0 : 1;
    transformer.tap_side = tap_side;
    const double tap_step =
        (num_taps[tap_slot] > 0)
            ? (max_taps[tap_slot] - min_taps[tap_slot]) /
                  static_cast<double>(num_taps[tap_slot])
            : 0.0;
    if (tap_step > 1e-9) {
      transformer.tap_step_percent = tap_step * 100.0;
      transformer.tap_min = 0;
      transformer.tap_max = num_taps[tap_slot];
      transformer.tap_neutral =
          static_cast<int>(std::llround((1.0 - min_taps[tap_slot]) / tap_step));
      transformer.tap_pos =
          static_cast<int>(std::llround((taps[tap_slot] - min_taps[tap_slot]) / tap_step));
      transformer.tap_pos =
          std::clamp(transformer.tap_pos, transformer.tap_min, transformer.tap_max);
    }
    transformer_index_by_name[transformer.name] = transformer.index;
    sys.transformers.push_back(std::move(transformer));
  }

  for (int has_vsource = ctx_Vsources_Get_First(api.get());
       has_vsource != 0;
       has_vsource = ctx_Vsources_Get_Next(api.get())) {
    api.check("ctx_Vsources_Get_First/Next");
    const std::string source_name =
        lowercase_ascii_copy(dss_get_string_value(api, ctx_Vsources_Get_Name, "ctx_Vsources_Get_Name"));
    ctx_Circuit_SetActiveElement(api.get(), ("Vsource." + source_name).c_str());
    api.check("ctx_Circuit_SetActiveElement");
    if (ctx_CktElement_Get_Enabled(api.get()) == 0) {
      api.check("ctx_CktElement_Get_Enabled");
      continue;
    }
    api.check("ctx_CktElement_Get_Enabled");

    const int phases = dss_get_int_value(api, ctx_Vsources_Get_Phases, "ctx_Vsources_Get_Phases");
    const std::vector<std::string> vsource_bus_names =
        dss_get_string_array(api, ctx_CktElement_Get_BusNames, uint16_t{0});
    if (vsource_bus_names.empty()) continue;
    const ParsedDSSBusTerminal terminal =
        parse_dss_bus_terminal(vsource_bus_names.front(), phases);

    ThreePhaseExternalGrid source;
    source.index = static_cast<int>(sys.external_grids.size()) + 1;
    source.name = source_name;
    source.bus = bus_id_by_name.at(terminal.bus_name);
    source.phase_mask = terminal.phase_mask;
    source.vm_pu =
        dss_get_double_value(api, ctx_Vsources_Get_pu, "ctx_Vsources_Get_pu");
    source.va_deg =
        dss_get_double_value(api, ctx_Vsources_Get_AngleDeg, "ctx_Vsources_Get_AngleDeg");
    const VsourceImpedanceProperties impedance_props =
        read_vsource_impedance_properties(api);

    ctx_Circuit_SetActiveBus(api.get(), terminal.bus_name.c_str());
    api.check("ctx_Circuit_SetActiveBus");
    const ThreePhaseACBus& source_bus =
        sys.buses[static_cast<std::size_t>(source.bus - 1)];
    const double z_base = bus_base_impedance_ohm(source_bus, sys.base_mva);
    if (impedance_props.r1_ohm || impedance_props.x1_ohm) {
      source.r1_pu = impedance_props.r1_ohm.value_or(0.0) / z_base;
      source.x1_pu = impedance_props.x1_ohm.value_or(0.0) / z_base;
      source.r2_pu = source.r1_pu;
      source.x2_pu = source.x1_pu;
    }
    if (impedance_props.r0_ohm || impedance_props.x0_ohm) {
      source.r0_pu = impedance_props.r0_ohm.value_or(0.0) / z_base;
      source.x0_pu = impedance_props.x0_ohm.value_or(0.0) / z_base;
    }
    if (impedance_props.mvasc3 && *impedance_props.mvasc3 > 0.0) {
      source.s_sc_max_mva = *impedance_props.mvasc3;
      if (impedance_props.x1r1 && *impedance_props.x1r1 > 0.0) {
        source.rx_max = 1.0 / *impedance_props.x1r1;
      }
    }
    if (impedance_props.mvasc1 && *impedance_props.mvasc1 > 0.0) {
      source.s_sc_min_mva = *impedance_props.mvasc1;
      if (impedance_props.x0r0 && *impedance_props.x0r0 > 0.0) {
        source.rx_min = 1.0 / *impedance_props.x0r0;
      }
    }

    if (!has_nonzero_impedance(Complex(source.r1_pu, source.x1_pu))) {
      if (const auto z1 =
              derive_positive_sequence_impedance_from_mvasc(impedance_props, sys.base_mva)) {
        source.r1_pu = z1->real();
        source.x1_pu = z1->imag();
        source.r2_pu = source.r1_pu;
        source.x2_pu = source.x1_pu;
      }
    }
    const std::vector<double> zsc1 = dss_get_double_array(api, ctx_Bus_Get_Zsc1);
    const std::vector<double> zsc0 = dss_get_double_array(api, ctx_Bus_Get_Zsc0);
    if (!has_nonzero_impedance(Complex(source.r1_pu, source.x1_pu)) && zsc1.size() >= 2) {
      source.r1_pu = zsc1[0] / z_base;
      source.x1_pu = zsc1[1] / z_base;
      source.r2_pu = source.r1_pu;
      source.x2_pu = source.x1_pu;
    }
    if (!has_nonzero_impedance(Complex(source.r0_pu, source.x0_pu))) {
      if (const auto z0 = derive_zero_sequence_impedance_from_mvasc(
              impedance_props,
              sys.base_mva,
              Complex(source.r1_pu, source.x1_pu))) {
        source.r0_pu = z0->real();
        source.x0_pu = z0->imag();
      }
    }
    if (!has_nonzero_impedance(Complex(source.r0_pu, source.x0_pu)) && zsc0.size() >= 2) {
      source.r0_pu = zsc0[0] / z_base;
      source.x0_pu = zsc0[1] / z_base;
    }
    sys.external_grids.push_back(source);

    ThreePhaseACBus& bus = find_bus(terminal.bus_name);
    bus.bus_type = BusType::SLACK;
    const std::array<double, 3> phase_offsets = {0.0, -120.0, 120.0};
    if (terminal.phase_mask.has(0)) {
      bus.vm_a_pu = source.vm_pu;
      bus.va_a_deg = source.va_deg + phase_offsets[0];
    }
    if (terminal.phase_mask.has(1)) {
      bus.vm_b_pu = source.vm_pu;
      bus.va_b_deg = source.va_deg + phase_offsets[1];
    }
    if (terminal.phase_mask.has(2)) {
      bus.vm_c_pu = source.vm_pu;
      bus.va_c_deg = source.va_deg + phase_offsets[2];
    }
  }

  for (int has_reg = ctx_RegControls_Get_First(api.get());
       has_reg != 0;
       has_reg = ctx_RegControls_Get_Next(api.get())) {
    api.check("ctx_RegControls_Get_First/Next");
    ThreePhaseRegulatorControl control;
    control.index = static_cast<int>(sys.regulator_controls.size()) + 1;
    control.name = lowercase_ascii_copy(
        dss_get_string_value(api, ctx_RegControls_Get_Name, "ctx_RegControls_Get_Name"));
    control.transformer_name = lowercase_ascii_copy(
        dss_get_string_value(api, ctx_RegControls_Get_Transformer, "ctx_RegControls_Get_Transformer"));
    const auto xfmr_it = transformer_index_by_name.find(control.transformer_name);
    if (xfmr_it == transformer_index_by_name.end()) {
      continue;
    }
    control.transformer_index = xfmr_it->second;
    control.winding = dss_get_int_value(api, ctx_RegControls_Get_Winding, "ctx_RegControls_Get_Winding");
    control.tap_winding = dss_get_int_value(api, ctx_RegControls_Get_TapWinding, "ctx_RegControls_Get_TapWinding");
    const ParsedDSSBusTerminal monitored =
        parse_dss_bus_terminal(
            dss_get_string_value(api, ctx_RegControls_Get_MonitoredBus, "ctx_RegControls_Get_MonitoredBus"),
            1);
    auto monitored_it = bus_id_by_name.find(monitored.bus_name);
    control.monitored_bus =
        monitored_it == bus_id_by_name.end() ? 0 : monitored_it->second;
    control.monitored_node =
        monitored.nodes.empty() ? 1 : monitored.nodes.front();
    control.vreg_volts = dss_get_double_value(api, ctx_RegControls_Get_ForwardVreg, "ctx_RegControls_Get_ForwardVreg");
    control.band_volts = dss_get_double_value(api, ctx_RegControls_Get_ForwardBand, "ctx_RegControls_Get_ForwardBand");
    control.ptratio = dss_get_double_value(api, ctx_RegControls_Get_PTratio, "ctx_RegControls_Get_PTratio");
    const std::string remote_ptratio = dss_active_property(api, "remoteptratio");
    if (!remote_ptratio.empty()) {
      try {
        control.remote_ptratio = std::stod(remote_ptratio);
      } catch (const std::exception&) {
        control.remote_ptratio = 0.0;
      }
    }
    control.ct_primary_amps = dss_get_double_value(api, ctx_RegControls_Get_CTPrimary, "ctx_RegControls_Get_CTPrimary");
    control.r_volts = dss_get_double_value(api, ctx_RegControls_Get_ForwardR, "ctx_RegControls_Get_ForwardR");
    control.x_volts = dss_get_double_value(api, ctx_RegControls_Get_ForwardX, "ctx_RegControls_Get_ForwardX");
    control.max_tap_change =
        dss_get_int_value(api, ctx_RegControls_Get_MaxTapChange, "ctx_RegControls_Get_MaxTapChange");
    control.reversible =
        dss_get_int_value(api, ctx_RegControls_Get_IsReversible, "ctx_RegControls_Get_IsReversible") != 0;
    sys.regulator_controls.push_back(std::move(control));
  }
#else
  (void)master_dss;
#endif

  return sys;
}

ThreePhaseJPCPhase case2jpc_phase(
    const ThreePhaseACSystem& sys,
    const std::unordered_map<std::string, double>& reg_taps) {
  ThreePhaseACSystem working = sys;
  apply_reg_tap_overrides(working, reg_taps);

  ThreePhaseJPCPhase jpc;
  jpc.base_mva = (working.base_mva > 0.0) ? working.base_mva : 100.0;
  jpc.base_freq_hz = working.base_freq_hz;
  jpc.bus_abc.reserve(working.buses.size());
  jpc.v_abc.assign(working.buses.size() * 3, Complex(0.0, 0.0));

  std::unordered_map<int, std::size_t> bus_row_lookup;
  bus_row_lookup.reserve(working.buses.size());

  for (std::size_t bus_row = 0; bus_row < working.buses.size(); ++bus_row) {
    const auto& bus = working.buses[bus_row];
    PhaseDomainBusRow row;
    row.bus_id = bus.index;
    row.bus_type = bus.bus_type;
    row.base_kv = bus.base_kv;
    row.vm_pu = {1.0, 1.0, 1.0};
    row.va_deg = {0.0, -120.0, 120.0};
    if (bus.bus_type == BusType::SLACK || bus.bus_type == BusType::PV) {
      row.vm_pu = {bus.vm_a_pu, bus.vm_b_pu, bus.vm_c_pu};
      row.va_deg = {bus.va_a_deg, bus.va_b_deg, bus.va_c_deg};
    }
    row.pd_mw = {bus.pd_a_mw, bus.pd_b_mw, bus.pd_c_mw};
    row.qd_mvar = {bus.qd_a_mvar, bus.qd_b_mvar, bus.qd_c_mvar};
    row.gs_mw = {bus.gs_a_mw, bus.gs_b_mw, bus.gs_c_mw};
    row.bs_mvar = {bus.bs_a_mvar, bus.bs_b_mvar, bus.bs_c_mvar};
    row.zone = bus.zone;
    row.area = bus.area;
    row.has_phase = {
        bus.phase_mask.has(0),
        bus.phase_mask.has(1),
        bus.phase_mask.has(2),
    };
    jpc.bus_id_to_name[bus.index] = bus.name;
    jpc.bus_name_to_id[bus.name] = bus.index;

    if (bus.bus_type == BusType::SLACK) {
      jpc.ref_bus_rows.push_back(static_cast<int>(bus_row));
    } else if (bus.bus_type == BusType::PV) {
      jpc.pv_bus_rows.push_back(static_cast<int>(bus_row));
    } else {
      jpc.pq_bus_rows.push_back(static_cast<int>(bus_row));
    }

    jpc.v_abc[bus_row * 3 + 0] =
        std::polar(row.vm_pu[0], row.va_deg[0] * kPi / 180.0);
    jpc.v_abc[bus_row * 3 + 1] =
        std::polar(row.vm_pu[1], row.va_deg[1] * kPi / 180.0);
    jpc.v_abc[bus_row * 3 + 2] =
        std::polar(row.vm_pu[2], row.va_deg[2] * kPi / 180.0);

    bus_row_lookup[bus.index] = jpc.bus_abc.size();
    jpc.bus_abc.push_back(row);
  }

  for (const auto& generator : working.generators) {
    if (!generator.in_service) continue;
    const auto it = bus_row_lookup.find(generator.bus);
    if (it == bus_row_lookup.end()) continue;
    fill_bus_generation_fields(jpc.bus_abc[it->second], generator);
  }

  const std::set<int> regulated_transformers =
      build_regulated_transformer_index_set(working);

  for (const auto& line : working.lines) {
    if (!line.in_service) continue;
    PhaseDomainBranchRow branch;
    branch.from_bus = line.from_bus;
    branch.to_bus = line.to_bus;
    branch.in_service = line.in_service;
    branch.branch_type = PhaseDomainBranchType::Line;
    branch.from_phase_mask = line.phase_mask;
    branch.to_phase_mask = line.phase_mask;
    branch.r_pu = build_line_r_matrix(line);
    branch.x_pu = build_line_x_matrix(line);
    branch.b_pu = build_line_b_vector(line);
    branch.element_name = line.name;
    branch.length_km = line.length_km;
    jpc.branch_abc.push_back(std::move(branch));
  }

  for (const auto& transformer : working.transformers) {
    if (!transformer.in_service) continue;
    PhaseDomainBranchRow branch;
    branch.from_bus = transformer.hv_bus;
    branch.to_bus = transformer.lv_bus;
    branch.in_service = transformer.in_service;
    branch.branch_type =
        regulated_transformers.contains(transformer.index)
            ? PhaseDomainBranchType::Regulator
            : PhaseDomainBranchType::Transformer;
    branch.from_phase_mask = transformer.hv_phase_mask;
    branch.to_phase_mask = transformer.lv_phase_mask;
    const double r_pu = transformer_series_r_pu(transformer, jpc.base_mva);
    const double x_pu = transformer_series_x_pu(transformer, jpc.base_mva);
    for (int phase = 0; phase < 3; ++phase) {
      branch.r_pu[static_cast<std::size_t>(phase)][static_cast<std::size_t>(phase)] = r_pu;
      branch.x_pu[static_cast<std::size_t>(phase)][static_cast<std::size_t>(phase)] = x_pu;
    }
    branch.tap_pu = transformer_phase_taps(transformer);
    const double shift_deg = phase_shift_from_vector_group(transformer.vector_group);
    branch.shift_deg = {shift_deg, shift_deg, shift_deg};
    branch.from_connection = parse_connection_type(transformer.vector_group, true);
    branch.to_connection = parse_connection_type(transformer.vector_group, false);
    branch.element_name = transformer.name;
    branch.vector_group = transformer.vector_group;
    branch.from_winding_topology = transformer.hv_winding_topology;
    branch.to_winding_topology = transformer.lv_winding_topology;
    jpc.branch_abc.push_back(std::move(branch));
  }

  for (const auto& load : working.loads) {
    if (!load.in_service) continue;
    if (lowercase_ascii_copy(load.connection) != "delta") continue;
    const auto bus_it = bus_row_lookup.find(load.bus);
    if (bus_it == bus_row_lookup.end()) continue;

    PhaseDomainDeltaLoadRow row;
    row.bus_row = static_cast<int>(bus_it->second);
    row.bus_id = load.bus;
    row.phase_mask = load.phase_mask;

    if (load.phase_mask.bits == PhaseMask::abc().bits) {
      row.line_power_mva = {
          Complex(load.p_a_mw, load.q_a_mvar),
          Complex(load.p_b_mw, load.q_b_mvar),
          Complex(load.p_c_mw, load.q_c_mvar),
      };
    } else if (load.phase_mask.bits == PhaseMask::ab().bits) {
      row.line_power_mva[0] =
          Complex(load.p_a_mw + load.p_b_mw, load.q_a_mvar + load.q_b_mvar);
    } else if (load.phase_mask.bits == PhaseMask::bc().bits) {
      row.line_power_mva[1] =
          Complex(load.p_b_mw + load.p_c_mw, load.q_b_mvar + load.q_c_mvar);
    } else if (load.phase_mask.bits == PhaseMask::ac().bits) {
      row.line_power_mva[2] =
          Complex(load.p_c_mw + load.p_a_mw, load.q_c_mvar + load.q_a_mvar);
    }

    jpc.delta_loads.push_back(std::move(row));
  }

  return jpc;
}

ThreePhaseJPCPhase case2jpc_phase(
    const HybridPowerSystem& sys,
    const std::unordered_map<std::string, double>& reg_taps) {
  if (!sys.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "case2jpc_phase: no three_phase_ac subsystem present");
  }
  return case2jpc_phase(*sys.three_phase_ac, reg_taps);
}

void makeYbus_phase(
    ThreePhaseJPCPhase& jpc,
    const ThreePhaseACSystem& sys,
    bool include_shunts) {
  jpc.ybus_abc_dim = static_cast<int>(sys.buses.size()) * 3;
  jpc.ybus_abc_entries = build_full_ybus_phase_entries(sys, include_shunts);
}

void makeYbus_phase(
    ThreePhaseJPCPhase& jpc,
    const HybridPowerSystem& sys,
    bool include_shunts) {
  if (!sys.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "makeYbus_phase: no three_phase_ac subsystem present");
  }
  makeYbus_phase(jpc, *sys.three_phase_ac, include_shunts);
}

ThreePhaseJPCPhase runpf_phase(
    const ThreePhaseACSystem& sys,
    const RunPFPhaseOptions& opt) {
  std::unordered_map<std::string, double> reg_taps = opt.reg_taps;
  if (reg_taps.empty() && !opt.dss_file_path.empty()) {
    reg_taps = get_opendss_regulator_taps(opt.dss_file_path);
  }

  ThreePhaseACSystem working = sys;
  apply_reg_tap_overrides(working, reg_taps);

  ThreePhaseJPCPhase jpc = case2jpc_phase(working, {});
  makeYbus_phase(jpc, working, opt.include_shunts);

  if (opt.algorithm == PhaseDomainSolverAlgorithm::OpenDSS) {
#ifdef HACDCPF_HAVE_OPENDSS
    if (opt.dss_file_path.empty()) {
      throw std::runtime_error(
          "runpf_phase: algorithm=OpenDSS requires dss_file_path");
    }
    const OpenDSSYMatrixSolveResult dss_result = solve_opendss_ymatrix(
        opt.dss_file_path,
        reg_taps,
        opt.max_iter,
        opt.tol,
        opt.verbose);
    populate_jpc_from_voltage_map(
        jpc,
        dss_result.voltage_map,
        dss_result.converged,
        dss_result.iterations,
        dss_result.residual);
    jpc.primary_solver = "opendss_reference";
    jpc.solver_used = "opendss_reference";
    jpc.result_source = "opendss_reference";
    jpc.fallback_used = false;
    jpc.primary_solver_failed_reason.clear();
    return jpc;
#else
    throw std::runtime_error(
        "runpf_phase: OpenDSS bridge is not enabled in this build");
#endif
  }

  ThreePhaseDPFResult result;
  if (opt.algorithm == PhaseDomainSolverAlgorithm::Newton) {
    ThreePhaseNROptions nr_opt;
    nr_opt.max_iter = opt.max_iter;
    nr_opt.max_control_iter = opt.max_control_iter;
    nr_opt.tol = opt.tol;
    nr_opt.verbose = opt.verbose;
    nr_opt.include_shunts = opt.include_shunts;
    result = solve_three_phase_nr(working, nr_opt);
  } else {
    ThreePhaseFixedPointOptions fp_opt;
    fp_opt.max_iter = opt.max_iter;
    fp_opt.tol = opt.tol;
    fp_opt.verbose = opt.verbose;
    fp_opt.include_shunts = opt.include_shunts;
    fp_opt.vmin_pu = opt.vmin_pu;
    if (opt.algorithm == PhaseDomainSolverAlgorithm::Compact) {
      result = solve_three_phase_compact_pf(working, fp_opt);
    } else {
      result = solve_three_phase_fixed_point(working, fp_opt);
    }
  }

  populate_jpc_voltage_state(jpc, result);
  return jpc;
}

ThreePhaseJPCPhase runpf_phase(
    const HybridPowerSystem& sys,
    const RunPFPhaseOptions& opt) {
  if (!sys.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "runpf_phase: no three_phase_ac subsystem present");
  }
  return runpf_phase(*sys.three_phase_ac, opt);
}

}  // namespace hacdcpf::analysis
