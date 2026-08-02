#include "hacdcpf/power_flow/residual_evaluator.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"
#include "hacdcpf/power_flow/pf_injection_assembly.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

namespace {

void compute_ac_power_injections(
                                 const Eigen::SparseMatrix<std::complex<double>>& ybus,
                                 const Eigen::VectorXd& vm,
                                 const Eigen::VectorXd& va,
                                 Eigen::VectorXd& pcalc,
                                 Eigen::VectorXd& qcalc) {
  const int n = static_cast<int>(vm.size());
  pcalc = Eigen::VectorXd::Zero(n);
  qcalc = Eigen::VectorXd::Zero(n);
  Eigen::VectorXcd voltage(n);
  for (int i = 0; i < n; ++i) {
    voltage[i] = std::polar(vm[i], va[i]);
  }
  const Eigen::VectorXcd current = ybus * voltage;
  for (int i = 0; i < n; ++i) {
    const std::complex<double> injection = voltage[i] * std::conj(current[i]);
    pcalc[i] = injection.real();
    qcalc[i] = injection.imag();
  }
}

std::unordered_set<int> dc_reference_buses_for_residual(const SolverData& data) {
  const int ndc = static_cast<int>(data.dc_buses.size());
  std::unordered_set<int> references;
  if (ndc == 0) return references;

  std::unordered_map<int, int> pos_by_id;
  pos_by_id.reserve(static_cast<size_t>(ndc));
  for (int i = 0; i < ndc; ++i) {
    pos_by_id[data.dc_buses[static_cast<size_t>(i)].index] = i;
  }
  std::vector<std::vector<int>> adjacency(static_cast<size_t>(ndc));
  for (const auto& branch : data.dc_branches) {
    if (!branch.in_service) continue;
    const auto from = pos_by_id.find(branch.from_bus);
    const auto to = pos_by_id.find(branch.to_bus);
    if (from == pos_by_id.end() || to == pos_by_id.end()) continue;
    adjacency[static_cast<size_t>(from->second)].push_back(to->second);
    adjacency[static_cast<size_t>(to->second)].push_back(from->second);
  }

  std::vector<char> visited(static_cast<size_t>(ndc), 0);
  for (int start = 0; start < ndc; ++start) {
    if (visited[static_cast<size_t>(start)] != 0 ||
        data.dc_buses[static_cast<size_t>(start)].bus_type ==
            DCBusType::DC_ISOLATED) {
      continue;
    }
    std::vector<int> component;
    std::vector<int> stack{start};
    while (!stack.empty()) {
      const int bus = stack.back();
      stack.pop_back();
      if (visited[static_cast<size_t>(bus)] != 0) continue;
      visited[static_cast<size_t>(bus)] = 1;
      component.push_back(bus);
      for (int neighbor : adjacency[static_cast<size_t>(bus)]) {
        if (visited[static_cast<size_t>(neighbor)] == 0) stack.push_back(neighbor);
      }
    }

    int declared_reference = -1;
    bool converter_regulates_vdc = false;
    std::unordered_set<int> component_ids;
    for (int bus : component) {
      const auto& dc_bus = data.dc_buses[static_cast<size_t>(bus)];
      component_ids.insert(dc_bus.index);
      if (declared_reference < 0 && dc_bus.bus_type == DCBusType::DC_V) {
        declared_reference = bus;
      }
    }
    for (const auto& converter : data.converters) {
      if (!converter.in_service || component_ids.count(converter.bus_dc) == 0) continue;
      const bool promotable = converter.control_mode == ConverterMode::PQ_MODE;
      converter_regulates_vdc = converter_regulates_vdc || promotable ||
                                resolve_device_control_role(converter).is_dc_grid_forming;
    }
    if (declared_reference >= 0) {
      references.insert(declared_reference);
    } else if (!converter_regulates_vdc && !component.empty()) {
      // Mirrors Newton's explicit non-physical fallback when an island has no
      // voltage-forming device at all.
      references.insert(component.front());
    }
  }
  return references;
}

}  // namespace

ResidualBlocks evaluate_power_flow_residual(const SolverData& data,
                                            const Eigen::VectorXd& vm,
                                            const Eigen::VectorXd& va,
                                            const Eigen::VectorXd& vdc) {
  ResidualBlocks residual;

  const int n = static_cast<int>(data.ac_buses.size());
  const int ndc = static_cast<int>(data.dc_buses.size());

  const AcBusSets ac = classify_ac_buses(data);
  const std::unordered_set<int> dc_slacks =
      dc_reference_buses_for_residual(data);

  std::vector<int> dc_non_slack;
  dc_non_slack.reserve(static_cast<size_t>(std::max(0, ndc - 1)));
  for (int i = 0; i < ndc; ++i) {
    if (dc_slacks.count(i) != 0) continue;
    // Isolated DC buses are de-energized and held at fixed voltage, so they are
    // not part of the solved equation set (mirrors DCSolver::solve).
    if (data.dc_buses[static_cast<size_t>(i)].bus_type == DCBusType::DC_ISOLATED) {
      continue;
    }
    dc_non_slack.push_back(i);
  }

  const int np = static_cast<int>(ac.non_slack.size());
  const int nq = static_cast<int>(ac.pq.size());
  const int ndc_eq = static_cast<int>(dc_non_slack.size());

  residual.ac = Eigen::VectorXd::Zero(np + nq);
  residual.dc = Eigen::VectorXd::Zero(ndc_eq);
  residual.full = Eigen::VectorXd::Zero(np + nq + ndc_eq);

  Eigen::VectorXd pcalc = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qcalc = Eigen::VectorXd::Zero(n);
  if (n > 0) {
    compute_ac_power_injections(data.ybus, vm, va, pcalc, qcalc);
  }

  Eigen::VectorXd p_spec;
  Eigen::VectorXd q_spec;
  assemble_ac_injections(data, vm, va, vdc, p_spec, q_spec);

  for (int k = 0; k < np; ++k) {
    const int i = ac.non_slack[static_cast<size_t>(k)];
    residual.ac[k] = p_spec[i] - pcalc[i];
  }
  for (int k = 0; k < nq; ++k) {
    const int i = ac.pq[static_cast<size_t>(k)];
    residual.ac[np + k] = q_spec[i] - qcalc[i];
  }

  if (ndc_eq > 0) {
    Eigen::VectorXd pdc_spec = Eigen::VectorXd::Zero(ndc);
    assemble_dc_injections(data, vm, va, vdc, pdc_spec);

    const Eigen::VectorXd pdc_linear = data.gdc * vdc;
    const Eigen::VectorXd pdc_calc = vdc.array() * pdc_linear.array();
    for (int k = 0; k < ndc_eq; ++k) {
      const int i = dc_non_slack[static_cast<size_t>(k)];
      residual.dc[k] = pdc_spec[i] - pdc_calc[i];
    }
  }

  if (residual.ac.size() > 0) {
    residual.full.head(residual.ac.size()) = residual.ac;
  }
  if (residual.dc.size() > 0) {
    residual.full.tail(residual.dc.size()) = residual.dc;
  }

  return residual;
}

}  // namespace hacdcpf::powerflow
