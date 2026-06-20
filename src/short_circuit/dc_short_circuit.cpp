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
  for (const auto& br : dc.branches) {
    if (!br.in_service) continue;
    const auto f = pos.find(br.from_bus);
    const auto t = pos.find(br.to_bus);
    if (f == pos.end() || t == pos.end()) continue;
    if (br.r_pu <= 1e-12) continue;  // ideal short -> skip (avoids singularity)
    const double g = static_cast<double>(std::max(1, br.n_parallel)) / br.r_pu;
    G(f->second, f->second) += g;
    G(t->second, t->second) += g;
    G(f->second, t->second) -= g;
    G(t->second, f->second) -= g;
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
  res.solved = true;
  return res;
}

}  // namespace hacdcpf::analysis
