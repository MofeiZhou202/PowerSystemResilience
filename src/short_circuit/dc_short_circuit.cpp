/// @file dc_short_circuit.cpp
/// Resistive quasi-static DC fault levels with sparse selected solves.

#include "hacdcpf/analysis/dc_short_circuit.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/model/enums.hpp"

namespace hacdcpf::analysis {
namespace {

constexpr double kIdealResistancePu = 1e-12;
// Backward-error admission threshold fixed by the short-circuit manual's
// numerical gate; see Higham, Accuracy and Stability (2nd ed.), sec. 7.2.
constexpr double kLinearResidualTolerance = 1e-9;

class DisjointSet {
 public:
  explicit DisjointSet(int n) : parent_(static_cast<size_t>(n)) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }
  int find(int value) {
    if (parent_[static_cast<size_t>(value)] != value)
      parent_[static_cast<size_t>(value)] = find(parent_[static_cast<size_t>(value)]);
    return parent_[static_cast<size_t>(value)];
  }
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) parent_[static_cast<size_t>(b)] = a;
  }

 private:
  std::vector<int> parent_;
};

bool same_dc_terminals(int a_from, int a_to, int b_from, int b_to) {
  return (a_from == b_from && a_to == b_to) ||
         (a_from == b_to && a_to == b_from);
}

double dc_base_kv_for_edge(const DCSystem& dc, int from_bus, int to_bus) {
  for (const auto& bus : dc.buses) {
    if ((bus.index == from_bus || bus.index == to_bus) && bus.base_kv > 1e-9)
      return bus.base_kv;
  }
  return 1.0;
}

double system_base_mva(const HybridPowerSystem& sys) {
  if (sys.dc.base_mva > 1e-9) return sys.dc.base_mva;
  if (sys.base_mva > 1e-9) return sys.base_mva;
  return 100.0;
}

double breaker_r_pu(const HybridPowerSystem& sys, const DCCircuitBreaker& cb,
                    const DCFaultOptions& opt) {
  const double base_kv = cb.rated_voltage_kv > 1e-9
      ? cb.rated_voltage_kv
      : dc_base_kv_for_edge(sys.dc, cb.bus_from, cb.bus_to);
  const double z_base = base_kv * base_kv / system_base_mva(sys);
  const double authored = cb.r_ohm > 0.0 ? cb.r_ohm / z_base : 0.0;
  return authored > kIdealResistancePu
      ? authored : opt.min_closed_breaker_resistance_pu;
}

void validate_options(const DCFaultOptions& opt) {
  const auto finite = [](double value) { return std::isfinite(value); };
  if (!finite(opt.fault_resistance_pu) || opt.fault_resistance_pu < 0.0)
    throw std::invalid_argument("DCFaultOptions.fault_resistance_pu must be finite and non-negative");
  if (!finite(opt.source_voltage_pu) || opt.source_voltage_pu < 0.0 ||
      opt.source_voltage_pu > 2.0)
    throw std::invalid_argument("DCFaultOptions.source_voltage_pu must be finite and in [0, 2]");
  if (!finite(opt.min_closed_breaker_resistance_pu) ||
      opt.min_closed_breaker_resistance_pu <= 0.0)
    throw std::invalid_argument("DCFaultOptions.min_closed_breaker_resistance_pu must be finite and positive");
}

struct Edge {
  int from{-1};
  int to{-1};
  double resistance_pu{0.0};
  int n_parallel{1};
  int branch_index{-1};
  int standalone_breaker_position{-1};
  std::vector<int> breaker_positions;
};

struct DCFaultContext {
  const HybridPowerSystem* sys{nullptr};
  DCFaultOptions options;
  std::unordered_map<int, int> bus_position;
  std::vector<const DCBus*> buses;
  std::vector<Edge> edges;
  std::vector<int> bus_to_super;
  std::vector<int> super_to_unknown;
  std::vector<char> super_is_source;
  std::vector<char> super_source_reachable;
  std::vector<double> super_source_voltage;
  std::vector<double> prefault_unknown_voltage;
  std::vector<int> breaker_to_edge;
  std::vector<int> breaker_controlled_branch;
  std::vector<char> breaker_open_blocks_branch;
  Eigen::SparseMatrix<double> gnn;
  std::unique_ptr<Eigen::SparseLU<Eigen::SparseMatrix<double>>> factor;
  bool factor_valid{false};
  std::string failure_message;
  int dc_branch_edges_used{0};
  int dccb_edges_used{0};
  int dccb_open_count{0};
  int dccb_blocked_branch_count{0};
};

std::vector<std::vector<int>> assign_breakers_to_branches(
    const DCSystem& dc, std::vector<int>& breaker_controlled_branch) {
  std::vector<std::vector<int>> assigned(dc.branches.size());
  breaker_controlled_branch.assign(dc.dc_circuit_breakers.size(), -1);
  for (size_t ci = 0; ci < dc.dc_circuit_breakers.size(); ++ci) {
    const auto& cb = dc.dc_circuit_breakers[ci];
    if (!cb.in_service) continue;
    std::vector<int> matches;
    const bool branch_kind = cb.element_type.empty() || cb.element_type == "branch" ||
        cb.element_type == "dc_branch" || cb.element_type == "l" ||
        cb.element_type == "line" || cb.element_type == "dc_line";
    if (branch_kind && cb.element_id != 0) {
      for (size_t bi = 0; bi < dc.branches.size(); ++bi)
        if (cb.element_id == dc.branches[bi].index) matches.push_back(static_cast<int>(bi));
    }
    if (matches.empty()) {
      for (size_t bi = 0; bi < dc.branches.size(); ++bi) {
        const auto& branch = dc.branches[bi];
        if (same_dc_terminals(cb.bus_from, cb.bus_to,
                              branch.from_bus, branch.to_bus))
          matches.push_back(static_cast<int>(bi));
      }
    }
    if (matches.size() > 1) {
      throw std::invalid_argument(
          "DCCircuitBreaker index " + std::to_string(cb.index) +
          " ambiguously matches multiple DCBranch rows; set element_type=dc_branch and element_id");
    }
    if (matches.size() == 1) {
      assigned[static_cast<size_t>(matches.front())].push_back(static_cast<int>(ci));
      breaker_controlled_branch[ci] = dc.branches[static_cast<size_t>(matches.front())].index;
    }
  }
  return assigned;
}

DCFaultContext build_context(const HybridPowerSystem& sys,
                             const DCFaultOptions& opt) {
  DCFaultContext context;
  context.sys = &sys;
  context.options = opt;
  const auto& dc = sys.dc;
  for (const auto& bus : dc.buses) {
    if (!bus.in_service || bus.bus_type == DCBusType::DC_ISOLATED) continue;
    if (!std::isfinite(bus.base_kv) || bus.base_kv <= 0.0 || !std::isfinite(bus.vm_pu))
      throw std::invalid_argument("energized DC buses require finite positive base_kv and finite vm_pu");
    if (!context.bus_position.emplace(bus.index, static_cast<int>(context.buses.size())).second)
      throw std::invalid_argument("duplicate energized DCBus index " + std::to_string(bus.index));
    context.buses.push_back(&bus);
  }
  if (context.buses.empty()) {
    context.failure_message = "no energized DC buses";
    return context;
  }

  std::vector<std::vector<int>> branch_breakers(dc.branches.size());
  context.breaker_controlled_branch.assign(dc.dc_circuit_breakers.size(), -1);
  if (opt.consider_dc_breakers && opt.dc_breakers_control_branches)
    branch_breakers = assign_breakers_to_branches(dc, context.breaker_controlled_branch);
  context.breaker_to_edge.assign(dc.dc_circuit_breakers.size(), -1);
  context.breaker_open_blocks_branch.assign(dc.dc_circuit_breakers.size(), 0);

  DisjointSet dsu(static_cast<int>(context.buses.size()));
  std::vector<Edge> candidate_edges;
  for (size_t bi = 0; bi < dc.branches.size(); ++bi) {
    const auto& branch = dc.branches[bi];
    if (!branch.in_service) continue;
    if (!std::isfinite(branch.r_pu) || branch.r_pu < 0.0)
      throw std::invalid_argument("DCBranch resistance must be finite and non-negative");
    const auto from = context.bus_position.find(branch.from_bus);
    const auto to = context.bus_position.find(branch.to_bus);
    if (from == context.bus_position.end() || to == context.bus_position.end()) continue;
    Edge edge;
    edge.from = from->second;
    edge.to = to->second;
    edge.resistance_pu = branch.r_pu;
    edge.n_parallel = std::max(1, branch.n_parallel);
    edge.branch_index = branch.index;
    bool blocked = false;
    for (int ci : branch_breakers[bi]) {
      const auto& cb = dc.dc_circuit_breakers[static_cast<size_t>(ci)];
      edge.breaker_positions.push_back(ci);
      if (!cb.closed) {
        blocked = true;
        context.breaker_open_blocks_branch[static_cast<size_t>(ci)] = 1;
        ++context.dccb_open_count;
      } else {
        edge.resistance_pu += breaker_r_pu(sys, cb, opt);
      }
    }
    if (blocked) {
      ++context.dccb_blocked_branch_count;
      continue;
    }
    if (edge.resistance_pu <= kIdealResistancePu) dsu.unite(edge.from, edge.to);
    candidate_edges.push_back(std::move(edge));
    ++context.dc_branch_edges_used;
  }

  if (opt.consider_dc_breakers && opt.add_unassigned_closed_breaker_edges) {
    for (size_t ci = 0; ci < dc.dc_circuit_breakers.size(); ++ci) {
      const auto& cb = dc.dc_circuit_breakers[ci];
      if (!cb.in_service || context.breaker_controlled_branch[ci] >= 0) continue;
      if (!cb.closed) {
        ++context.dccb_open_count;
        continue;
      }
      const auto from = context.bus_position.find(cb.bus_from);
      const auto to = context.bus_position.find(cb.bus_to);
      if (from == context.bus_position.end() || to == context.bus_position.end()) continue;
      Edge edge;
      edge.from = from->second;
      edge.to = to->second;
      edge.resistance_pu = breaker_r_pu(sys, cb, opt);
      edge.standalone_breaker_position = static_cast<int>(ci);
      edge.breaker_positions.push_back(static_cast<int>(ci));
      candidate_edges.push_back(std::move(edge));
      ++context.dccb_edges_used;
    }
  }

  std::unordered_map<int, int> root_to_super;
  context.bus_to_super.resize(context.buses.size());
  for (size_t i = 0; i < context.buses.size(); ++i) {
    const int root = dsu.find(static_cast<int>(i));
    auto [it, inserted] = root_to_super.emplace(root, static_cast<int>(root_to_super.size()));
    (void)inserted;
    context.bus_to_super[i] = it->second;
  }
  const int ns = static_cast<int>(root_to_super.size());
  context.super_is_source.assign(static_cast<size_t>(ns), 0);
  context.super_source_voltage.assign(static_cast<size_t>(ns), 0.0);
  for (size_t i = 0; i < context.buses.size(); ++i) {
    if (context.buses[i]->bus_type != DCBusType::DC_V) continue;
    const int super = context.bus_to_super[i];
    const double voltage = opt.source_voltage_pu > 0.0
        ? opt.source_voltage_pu
        : (context.buses[i]->vm_pu > 0.0 ? context.buses[i]->vm_pu : 1.0);
    if (context.super_is_source[static_cast<size_t>(super)] &&
        std::abs(context.super_source_voltage[static_cast<size_t>(super)] - voltage) > 1e-9)
      throw std::invalid_argument("ideal DC conductor joins voltage sources with inconsistent setpoints");
    context.super_is_source[static_cast<size_t>(super)] = 1;
    context.super_source_voltage[static_cast<size_t>(super)] = voltage;
  }

  context.edges.reserve(candidate_edges.size());
  for (auto edge : candidate_edges) {
    edge.from = context.bus_to_super[static_cast<size_t>(edge.from)];
    edge.to = context.bus_to_super[static_cast<size_t>(edge.to)];
    if (edge.from == edge.to && edge.resistance_pu <= kIdealResistancePu) continue;
    const int edge_position = static_cast<int>(context.edges.size());
    for (int ci : edge.breaker_positions)
      context.breaker_to_edge[static_cast<size_t>(ci)] = edge_position;
    context.edges.push_back(std::move(edge));
  }

  std::vector<std::vector<int>> adjacency(static_cast<size_t>(ns));
  for (const auto& edge : context.edges) {
    if (edge.from == edge.to || edge.resistance_pu <= kIdealResistancePu) continue;
    adjacency[static_cast<size_t>(edge.from)].push_back(edge.to);
    adjacency[static_cast<size_t>(edge.to)].push_back(edge.from);
  }
  context.super_source_reachable.assign(static_cast<size_t>(ns), 0);
  std::queue<int> queue;
  for (int i = 0; i < ns; ++i) {
    if (!context.super_is_source[static_cast<size_t>(i)]) continue;
    context.super_source_reachable[static_cast<size_t>(i)] = 1;
    queue.push(i);
  }
  while (!queue.empty()) {
    const int current = queue.front();
    queue.pop();
    for (int next : adjacency[static_cast<size_t>(current)]) {
      if (context.super_source_reachable[static_cast<size_t>(next)]) continue;
      context.super_source_reachable[static_cast<size_t>(next)] = 1;
      queue.push(next);
    }
  }

  context.super_to_unknown.assign(static_cast<size_t>(ns), -1);
  int unknown_count = 0;
  for (int i = 0; i < ns; ++i) {
    if (context.super_source_reachable[static_cast<size_t>(i)] &&
        !context.super_is_source[static_cast<size_t>(i)])
      context.super_to_unknown[static_cast<size_t>(i)] = unknown_count++;
  }
  if (unknown_count == 0) {
    context.failure_message = "no non-source DC buses have a resistive path to a voltage source";
    return context;
  }

  std::vector<Eigen::Triplet<double>> triplets;
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(unknown_count);
  for (const auto& edge : context.edges) {
    if (edge.from == edge.to || edge.resistance_pu <= kIdealResistancePu) continue;
    const double conductance = static_cast<double>(edge.n_parallel) / edge.resistance_pu;
    const int endpoints[2] = {edge.from, edge.to};
    for (int side = 0; side < 2; ++side) {
      const int here = endpoints[side];
      const int other = endpoints[1 - side];
      const int row = context.super_to_unknown[static_cast<size_t>(here)];
      if (row < 0) continue;
      triplets.emplace_back(row, row, conductance);
      const int col = context.super_to_unknown[static_cast<size_t>(other)];
      if (col >= 0) triplets.emplace_back(row, col, -conductance);
      else if (context.super_is_source[static_cast<size_t>(other)])
        rhs(row) += conductance * context.super_source_voltage[static_cast<size_t>(other)];
    }
  }
  context.gnn.resize(unknown_count, unknown_count);
  context.gnn.setFromTriplets(triplets.begin(), triplets.end());
  context.gnn.makeCompressed();
  context.factor = std::make_unique<Eigen::SparseLU<Eigen::SparseMatrix<double>>>();
  context.factor->analyzePattern(context.gnn);
  context.factor->factorize(context.gnn);
  context.factor_valid = context.factor->info() == Eigen::Success;
  if (!context.factor_valid) {
    context.failure_message = "sparse DC conductance factorization failed";
    return context;
  }
  const Eigen::VectorXd prefault = context.factor->solve(rhs);
  const double residual = (context.gnn * prefault - rhs).norm() / std::max(1.0, rhs.norm());
  if (context.factor->info() != Eigen::Success || !prefault.allFinite() ||
      !std::isfinite(residual) || residual > kLinearResidualTolerance) {
    context.factor_valid = false;
    context.failure_message = "sparse DC pre-fault solve failed the 1e-9 residual gate";
    return context;
  }
  context.prefault_unknown_voltage.assign(prefault.data(), prefault.data() + prefault.size());
  return context;
}

DCFaultResult solve_fault(DCFaultContext& context, int dc_bus_id) {
  DCFaultResult result;
  result.fault_bus_id = dc_bus_id;
  result.model_limitations = {
      "Resistive quasi-static fault level excludes converter current limiting, DC/DC blocking, capacitor discharge, cable inductance, battery/PV controls, and breaker opening transients.",
      "DC_V buses are ideal voltage sources; faults on their ideal-conductor supernode require explicit source impedance and are rejected."};
  result.dc_branch_edges_used = context.dc_branch_edges_used;
  result.dccb_edges_used = context.dccb_edges_used;
  result.dccb_open_count = context.dccb_open_count;
  result.dccb_blocked_branch_count = context.dccb_blocked_branch_count;
  const auto bus_it = context.bus_position.find(dc_bus_id);
  if (bus_it == context.bus_position.end()) {
    result.status = "invalid_fault_bus";
    result.message = "fault bus not found or de-energized";
    return result;
  }
  const int bus_position = bus_it->second;
  const int fault_super = context.bus_to_super[static_cast<size_t>(bus_position)];
  if (context.super_is_source[static_cast<size_t>(fault_super)]) {
    result.status = "unsupported_ideal_source_fault";
    result.message = "fault bus is electrically identical to an ideal DC voltage source";
    return result;
  }
  if (!context.super_source_reachable[static_cast<size_t>(fault_super)]) {
    result.status = "unsupplied_island";
    result.message = "faulted DC island has no resistive path to a source";
    return result;
  }
  if (!context.factor_valid) {
    result.status = "numerical_failure";
    result.message = context.failure_message;
    return result;
  }
  if (context.options.cancellation_requested && context.options.cancellation_requested())
    throw std::runtime_error("DC short-circuit analysis cancelled");

  const int fault_unknown = context.super_to_unknown[static_cast<size_t>(fault_super)];
  Eigen::VectorXd unit = Eigen::VectorXd::Zero(context.gnn.rows());
  unit(fault_unknown) = 1.0;
  const Eigen::VectorXd zcol = context.factor->solve(unit);
  const double residual = (context.gnn * zcol - unit).norm();
  result.max_linear_residual = residual;
  if (context.factor->info() != Eigen::Success || !zcol.allFinite() ||
      !std::isfinite(residual) || residual > kLinearResidualTolerance) {
    result.status = "numerical_failure";
    result.message = "sparse DC Thevenin solve failed the 1e-9 residual gate";
    return result;
  }
  const double r_thevenin = zcol(fault_unknown);
  const double v_prefault = context.prefault_unknown_voltage[static_cast<size_t>(fault_unknown)];
  const double total_resistance = r_thevenin + context.options.fault_resistance_pu;
  if (!std::isfinite(total_resistance) || total_resistance <= kIdealResistancePu) {
    result.status = "singular_fault_path";
    result.message = "zero fault-path resistance requires explicit source/internal impedance";
    return result;
  }
  result.v_prefault_pu = v_prefault;
  result.r_thevenin_pu = r_thevenin;
  result.i_fault_pu = v_prefault / total_resistance;
  const double base_mva = system_base_mva(*context.sys);
  const double fault_base_kv = context.buses[static_cast<size_t>(bus_position)]->base_kv;
  result.i_fault_ka = result.i_fault_pu * base_mva / fault_base_kv;

  std::vector<double> fault_voltage(context.super_is_source.size(), 0.0);
  for (size_t super = 0; super < fault_voltage.size(); ++super) {
    if (context.super_is_source[super]) {
      fault_voltage[super] = context.super_source_voltage[super];
    } else {
      const int unknown = context.super_to_unknown[super];
      if (unknown >= 0) {
        // Compensation theorem: V^f = V^0 - Z[:,k] I_f.
        // See IEC 61660-1 and the project DC derivation sec. 7.
        fault_voltage[super] = context.prefault_unknown_voltage[static_cast<size_t>(unknown)] -
                               zcol(unknown) * result.i_fault_pu;
      }
    }
  }

  const auto& breakers = context.sys->dc.dc_circuit_breakers;
  result.breaker_duties.reserve(breakers.size());
  for (size_t ci = 0; ci < breakers.size(); ++ci) {
    const auto& cb = breakers[ci];
    DCBreakerDutyResult duty;
    duty.breaker_index = cb.index;
    duty.name = cb.name;
    duty.from_bus = cb.bus_from;
    duty.to_bus = cb.bus_to;
    duty.in_service = cb.in_service;
    duty.closed = cb.closed;
    duty.i_breaking_ka = cb.i_breaking_ka;
    duty.controls_branch = context.breaker_controlled_branch[ci] >= 0;
    duty.controlled_branch_index = duty.controls_branch ? context.breaker_controlled_branch[ci] : 0;
    if (cb.in_service && cb.closed) duty.r_pu = breaker_r_pu(*context.sys, cb, context.options);
    const int edge_position = context.breaker_to_edge[ci];
    if (!cb.in_service) {
      duty.model = "out_of_service";
    } else if (!cb.closed) {
      duty.model = context.breaker_open_blocks_branch[ci]
          ? "open_blocks_series_chain" : "open_no_edge";
    } else if (edge_position >= 0) {
      const auto& edge = context.edges[static_cast<size_t>(edge_position)];
      duty.model = duty.controls_branch
          ? (edge.breaker_positions.size() > 1 ? "closed_series_branch_chain"
                                               : "closed_series_branch")
          : "closed_standalone_edge";
      if (edge.from != edge.to && edge.resistance_pu > kIdealResistancePu) {
        const double current_pu = static_cast<double>(edge.n_parallel) *
            std::abs(fault_voltage[static_cast<size_t>(edge.from)] -
                     fault_voltage[static_cast<size_t>(edge.to)]) / edge.resistance_pu;
        const double breaker_base_kv = cb.rated_voltage_kv > 1e-9
            ? cb.rated_voltage_kv
            : dc_base_kv_for_edge(context.sys->dc, cb.bus_from, cb.bus_to);
        duty.i_duty_ka = current_pu * base_mva / breaker_base_kv;
      }
    } else {
      duty.model = "not_in_fault_graph";
    }
    duty.breaking_rating_ok = duty.i_breaking_ka <= kIdealResistancePu ||
                              duty.i_duty_ka <= duty.i_breaking_ka + 1e-9;
    result.breaker_duties.push_back(std::move(duty));
  }
  result.solved = true;
  result.status = "solved";
  result.message = "ok";
  return result;
}

}  // namespace

std::vector<DCFaultResult> dc_bus_fault_levels(
    const HybridPowerSystem& sys, const std::vector<int>& dc_bus_ids,
    const DCFaultOptions& opt) {
  validate_options(opt);
  DCFaultContext context = build_context(sys, opt);
  std::vector<DCFaultResult> results;
  results.reserve(dc_bus_ids.size());
  for (int bus_id : dc_bus_ids) {
    if (opt.cancellation_requested && opt.cancellation_requested())
      throw std::runtime_error("DC short-circuit analysis cancelled");
    if (!context.failure_message.empty() && context.buses.empty()) {
      DCFaultResult result;
      result.fault_bus_id = bus_id;
      result.status = "invalid_network";
      result.message = context.failure_message;
      result.model_limitations = {
          "Resistive quasi-static fault level excludes electromagnetic transients and converter-control dynamics."};
      results.push_back(std::move(result));
    } else {
      results.push_back(solve_fault(context, bus_id));
    }
  }
  return results;
}

DCFaultResult dc_bus_fault_level(const HybridPowerSystem& sys, int dc_bus_id,
                                 const DCFaultOptions& opt) {
  auto results = dc_bus_fault_levels(sys, {dc_bus_id}, opt);
  return results.front();
}

}  // namespace hacdcpf::analysis
