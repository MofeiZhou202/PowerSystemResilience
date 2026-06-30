/// analysis/dc_short_circuit.cpp
/// =============================
/// Resistive, stiff-source DC bolted-fault-level estimate.  See the header for
/// the modelling assumptions and limitations.

#include "hacdcpf/analysis/dc_short_circuit.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "hacdcpf/model/enums.hpp"

namespace hacdcpf::analysis {

namespace {

struct DCBreakerMatch {
  const DCCircuitBreaker* breaker{nullptr};
};

bool same_dc_terminals(int a_from, int a_to, int b_from, int b_to) {
  return (a_from == b_from && a_to == b_to) ||
         (a_from == b_to && a_to == b_from);
}

double dc_base_kv_for_edge(const DCSystem& dc, int from_bus, int to_bus) {
  for (const auto& b : dc.buses) {
    if ((b.index == from_bus || b.index == to_bus) && b.base_kv > 1e-9) {
      return b.base_kv;
    }
  }
  return 1.0;
}

double breaker_r_pu(const HybridPowerSystem& sys, const DCCircuitBreaker& cb,
                    const DCFaultOptions& opt) {
  const double base_mva = (sys.dc.base_mva > 1e-9) ? sys.dc.base_mva : sys.base_mva;
  const double base_kv = (cb.rated_voltage_kv > 1e-9)
      ? cb.rated_voltage_kv
      : dc_base_kv_for_edge(sys.dc, cb.bus_from, cb.bus_to);
  const double z_base = (base_kv * base_kv) / std::max(1e-9, base_mva);
  double r = (cb.r_ohm > 0.0 && z_base > 1e-12) ? cb.r_ohm / z_base : 0.0;
  if (r <= 1e-12) r = std::max(opt.min_closed_breaker_resistance_pu, 1e-12);
  return r;
}

void stamp_conductance(Eigen::MatrixXd& G, int fi, int ti, double r_pu,
                       int n_parallel = 1) {
  if (r_pu <= 1e-12) return;
  const double g = static_cast<double>(std::max(1, n_parallel)) / r_pu;
  G(fi, fi) += g;
  G(ti, ti) += g;
  G(fi, ti) -= g;
  G(ti, fi) -= g;
}

std::unordered_map<int, DCBreakerMatch>
match_breakers_to_branches(const DCSystem& dc) {
  std::unordered_map<int, DCBreakerMatch> out;
  for (const auto& br : dc.branches) {
    std::vector<const DCCircuitBreaker*> matches;
    for (const auto& cb : dc.dc_circuit_breakers) {
      if (!cb.in_service) continue;
      if (cb.element_id == br.index &&
          (cb.element_type.empty() || cb.element_type == "branch" ||
           cb.element_type == "dc_branch" || cb.element_type == "l" ||
           cb.element_type == "line" || cb.element_type == "dc_line")) {
        matches.push_back(&cb);
      }
    }
    if (matches.empty()) {
      for (const auto& cb : dc.dc_circuit_breakers) {
        if (!cb.in_service) continue;
        if (same_dc_terminals(cb.bus_from, cb.bus_to, br.from_bus, br.to_bus)) {
          matches.push_back(&cb);
        }
      }
    }
    if (matches.size() == 1) {
      out[br.index] = {matches.front()};
    }
  }
  return out;
}

}  // namespace

DCFaultResult dc_bus_fault_level(const HybridPowerSystem& sys, int dc_bus_id,
                                 const DCFaultOptions& opt) {
  DCFaultResult res;
  res.fault_bus_id = dc_bus_id;
  const auto& dc = sys.dc;

  // Collect energized DC buses and map bus index -> matrix position.
  std::unordered_map<int, int> pos;
  std::vector<const DCBus*> buses;
  for (const auto& b : dc.buses) {
    if (!b.in_service || b.bus_type == DCBusType::DC_ISOLATED) continue;
    pos[b.index] = static_cast<int>(buses.size());
    buses.push_back(&b);
  }
  const int n = static_cast<int>(buses.size());
  if (n == 0) { res.message = "no energized DC buses"; return res; }
  const auto kf_it = pos.find(dc_bus_id);
  if (kf_it == pos.end()) { res.message = "fault bus not found or de-energized"; return res; }
  const int kf = kf_it->second;

  // Resistive nodal conductance matrix (per-unit).
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(n, n);
  std::unordered_map<int, DCBreakerMatch> branch_breakers;
  if (opt.consider_dc_breakers && opt.dc_breakers_control_branches) {
    branch_breakers = match_breakers_to_branches(dc);
  }

  std::vector<char> breaker_used(dc.dc_circuit_breakers.size(), 0);
  for (const auto& br : dc.branches) {
    if (!br.in_service) continue;
    const auto f = pos.find(br.from_bus);
    const auto t = pos.find(br.to_bus);
    if (f == pos.end() || t == pos.end()) continue;
    double r_pu = br.r_pu;
    if (opt.consider_dc_breakers && opt.dc_breakers_control_branches) {
      auto bit = branch_breakers.find(br.index);
      if (bit != branch_breakers.end() && bit->second.breaker) {
        const auto& cb = *bit->second.breaker;
        auto cb_it = std::find_if(dc.dc_circuit_breakers.begin(),
                                  dc.dc_circuit_breakers.end(),
                                  [&](const DCCircuitBreaker& x) { return &x == &cb; });
        if (cb_it != dc.dc_circuit_breakers.end()) {
          breaker_used[static_cast<size_t>(std::distance(dc.dc_circuit_breakers.begin(), cb_it))] = 1;
        }
        if (!cb.closed) {
          ++res.dccb_open_count;
          ++res.dccb_blocked_branch_count;
          continue;
        }
        r_pu += breaker_r_pu(sys, cb, opt);
      }
    }
    if (r_pu <= 1e-12) continue;  // ideal short -> skip (avoids singularity)
    stamp_conductance(G, f->second, t->second, r_pu, br.n_parallel);
    ++res.dc_branch_edges_used;
  }

  if (opt.consider_dc_breakers && opt.add_unassigned_closed_breaker_edges) {
    for (size_t i = 0; i < dc.dc_circuit_breakers.size(); ++i) {
      const auto& cb = dc.dc_circuit_breakers[i];
      if (!cb.in_service) continue;
      if (breaker_used[i]) continue;
      if (!cb.closed) {
        ++res.dccb_open_count;
        continue;
      }
      const auto f = pos.find(cb.bus_from);
      const auto t = pos.find(cb.bus_to);
      if (f == pos.end() || t == pos.end()) continue;
      stamp_conductance(G, f->second, t->second, breaker_r_pu(sys, cb, opt));
      breaker_used[i] = 1;
      ++res.dccb_edges_used;
    }
  }

  // Voltage-reference (DC_V) buses act as ideal sources.
  std::vector<char> is_src(n, 0);
  std::vector<double> vsrc(n, 0.0);
  int nsrc = 0;
  for (int i = 0; i < n; ++i) {
    if (buses[i]->bus_type == DCBusType::DC_V) {
      is_src[i] = 1;
      ++nsrc;
      vsrc[i] = (opt.source_voltage_pu > 1e-9)
                    ? opt.source_voltage_pu
                    : (buses[i]->vm_pu > 1e-9 ? buses[i]->vm_pu : 1.0);
    }
  }
  if (nsrc == 0) { res.message = "no DC voltage-source (DC_V) bus"; return res; }
  if (is_src[kf]) {
    res.message = "fault bus is a stiff DC source (DC_V); source internal "
                  "impedance not modelled";
    return res;
  }

  // Reduce to the non-source buses with the sources grounded for the Thevenin
  // resistance: G_NN V_N = -G_NS V_S.
  std::vector<int> nmap;          // N-position -> matrix position
  std::vector<int> nidx(n, -1);   // matrix position -> N-position
  for (int i = 0; i < n; ++i) {
    if (!is_src[i]) { nidx[i] = static_cast<int>(nmap.size()); nmap.push_back(i); }
  }
  const int m = static_cast<int>(nmap.size());

  Eigen::MatrixXd Gnn = Eigen::MatrixXd::Zero(m, m);
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(m);
  for (int a = 0; a < m; ++a) {
    const int ia = nmap[a];
    for (int j = 0; j < n; ++j) {
      const double gij = G(ia, j);
      if (gij == 0.0) continue;
      if (is_src[j]) rhs(a) -= gij * vsrc[j];
      else Gnn(a, nidx[j]) += gij;
    }
  }

  Eigen::FullPivLU<Eigen::MatrixXd> lu(Gnn);
  if (!lu.isInvertible()) {
    res.message = "faulted DC island has no resistive path to a source";
    return res;
  }
  const Eigen::MatrixXd Zred = lu.inverse();
  const int ak = nidx[kf];
  const double zkk = Zred(ak, ak);                 // Thevenin resistance [pu]
  const Eigen::VectorXd vn = Zred * rhs;           // pre-fault voltages [pu]
  const double vpre = vn(ak);

  const double rtot = zkk + std::max(0.0, opt.fault_resistance_pu);
  if (rtot <= 1e-12) { res.message = "zero fault-path resistance"; return res; }

  const double base_mva = (dc.base_mva > 1e-9) ? dc.base_mva : sys.base_mva;
  const double base_kv = (buses[kf]->base_kv > 1e-9) ? buses[kf]->base_kv : 1.0;
  res.v_prefault_pu = vpre;
  res.r_thevenin_pu = zkk;
  res.i_fault_pu = vpre / rtot;
  res.i_fault_ka = res.i_fault_pu * (base_mva / base_kv);  // DC: P = V*I

  if (opt.consider_dc_breakers) {
    for (size_t i = 0; i < dc.dc_circuit_breakers.size(); ++i) {
      const auto& cb = dc.dc_circuit_breakers[i];
      DCBreakerDutyResult duty;
      duty.breaker_index = cb.index;
      duty.name = cb.name;
      duty.from_bus = cb.bus_from;
      duty.to_bus = cb.bus_to;
      duty.in_service = cb.in_service;
      duty.closed = cb.closed;
      duty.i_breaking_ka = cb.i_breaking_ka;
      if (cb.in_service && cb.closed) {
        duty.r_pu = breaker_r_pu(sys, cb, opt);
      }
      for (const auto& [branch_index, match] : branch_breakers) {
        if (match.breaker == &cb) {
          duty.controls_branch = true;
          duty.controlled_branch_index = branch_index;
          break;
        }
      }
      if (!cb.in_service) {
        duty.model = "out_of_service";
      } else if (!cb.closed) {
        duty.model = duty.controls_branch ? "open_blocks_branch" : "open_no_edge";
      } else if (duty.controls_branch) {
        duty.model = "closed_series_branch";
      } else if (breaker_used[i]) {
        duty.model = "closed_standalone_edge";
      } else {
        duty.model = "not_in_fault_graph";
      }
      if (cb.in_service && cb.closed && breaker_used[i]) {
        duty.i_duty_ka = res.i_fault_ka;
      }
      duty.breaking_rating_ok =
          (duty.i_breaking_ka <= 1e-12) || (duty.i_duty_ka <= duty.i_breaking_ka + 1e-9);
      res.breaker_duties.push_back(std::move(duty));
    }
  }
  res.solved = true;
  return res;
}

}  // namespace hacdcpf::analysis
