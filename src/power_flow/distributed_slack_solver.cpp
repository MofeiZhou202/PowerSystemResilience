#include "hacdcpf/power_flow/distributed_slack_solver.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/pf_injection_assembly.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

namespace {

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

std::unordered_map<int, double> per_bus_generation_pu(const HybridPowerSystem& sys) {
  std::unordered_map<int, double> pg;
  pg.reserve(sys.ac.buses.size());
  for (const auto& bus : sys.ac.buses) {
    if (bus.in_service) pg.emplace(bus.index, 0.0);
  }
  for (const auto& g : sys.ac.generators) {
    const auto bus = pg.find(g.bus);
    if (!g.in_service || bus == pg.end()) {
      continue;
    }
    bus->second += g.pg_mw / sys.base_mva;
  }
  return pg;
}

void normalize_factors(std::vector<double>& factors) {
  for (double factor : factors) {
    if (!std::isfinite(factor) || factor < 0.0) {
      throw std::invalid_argument(
          "Distributed slack participation factors must be finite and non-negative.");
    }
  }
  double sum = std::accumulate(factors.begin(), factors.end(), 0.0);
  if (sum <= 0.0) {
    if (!factors.empty()) {
      const double eq = 1.0 / static_cast<double>(factors.size());
      std::fill(factors.begin(), factors.end(), eq);
    }
    return;
  }
  for (double& f : factors) {
    f /= sum;
  }
}

DistributedSlack sanitize_slack_cfg(const HybridPowerSystem& sys, DistributedSlack cfg) {
  if (cfg.participating_buses.empty()) {
    return create_participation_factors(sys, "capacity");
  }
  if (cfg.participation_factors.size() != cfg.participating_buses.size()) {
    cfg.participation_factors.assign(cfg.participating_buses.size(), 1.0);
  }
  normalize_factors(cfg.participation_factors);
  if (cfg.reference_bus == 0) {
    const auto reference = std::find_if(
        sys.ac.buses.begin(), sys.ac.buses.end(),
        [](const ACBus& bus) {
          return bus.in_service && bus.bus_type == BusType::SLACK;
        });
    if (reference == sys.ac.buses.end()) {
      throw std::invalid_argument(
          "Distributed slack requires an explicit reference_bus when no AC slack bus exists.");
    }
    cfg.reference_bus = reference->index;
  }
  const auto reference = std::find_if(
      sys.ac.buses.begin(), sys.ac.buses.end(),
      [&](const ACBus& bus) {
        return bus.in_service && bus.index == cfg.reference_bus;
      });
  if (reference == sys.ac.buses.end()) {
    throw std::invalid_argument(
        "Distributed slack reference_bus is not an in-service AC bus.");
  }
  return cfg;
}

void apply_reference_bus(HybridPowerSystem& sys, int reference_bus) {
  bool found = false;
  for (auto& bus : sys.ac.buses) {
    if (!bus.in_service) continue;
    if (bus.index == reference_bus) {
      bus.bus_type = BusType::SLACK;
      found = true;
    } else if (bus.bus_type == BusType::SLACK) {
      const bool has_generator = std::any_of(
          sys.ac.generators.begin(), sys.ac.generators.end(),
          [&](const Generator& generator) {
            return generator.in_service && generator.bus == bus.index;
          });
      bus.bus_type = has_generator ? BusType::PV : BusType::PQ;
    }
  }
  if (!found) {
    throw std::invalid_argument(
        "Distributed slack reference_bus is not an in-service AC bus.");
  }
  for (auto& generator : sys.ac.generators) {
    if (generator.in_service) {
      generator.is_slack = generator.bus == reference_bus;
    }
  }
}

void apply_power_flow_options(SolverData& data, const PowerFlowOptions& opt) {
  for (int index = 0; index < 3; ++index) {
    data.zip_pw[index] = opt.zip_pw[index];
    data.zip_qw[index] = opt.zip_qw[index];
  }
  data.enable_coupled_jacobian = opt.enable_coupled_jacobian;
  data.enable_augmented_equations = opt.enable_augmented_equations;
  data.enable_semi_smooth_newton = opt.enable_semi_smooth_newton;
}

struct ParticipationBounds {
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
};

std::unordered_map<int, ParticipationBounds> generator_delta_bounds(
    const HybridPowerSystem& sys,
    const DistributedSlack& cfg) {
  std::unordered_map<int, ParticipationBounds> bounds;
  for (int bus : cfg.participating_buses) {
    double scheduled = 0.0;
    double lower = 0.0;
    double upper = 0.0;
    bool found = false;
    for (const auto& generator : sys.ac.generators) {
      if (!generator.in_service || generator.bus != bus) continue;
      found = true;
      scheduled += generator.pg_mw;
      lower += generator.pmin_mw;
      upper += generator.pmax_mw;
    }
    if (!found) {
      bounds[bus] = {0.0, 0.0};
      continue;
    }
    ParticipationBounds bus_bounds{
        (lower - scheduled) / sys.base_mva,
        (upper - scheduled) / sys.base_mva};
    const auto cfg_limit = cfg.max_participation_p.find(bus);
    if (cfg_limit != cfg.max_participation_p.end()) {
      const double limit = std::abs(cfg_limit->second);
      bus_bounds.lower = std::max(bus_bounds.lower, -limit);
      bus_bounds.upper = std::min(bus_bounds.upper, limit);
    }
    bounds[bus] = bus_bounds;
  }
  return bounds;
}

double compute_total_participating_mismatch_pu(const SolverData& data,
                                               const PowerFlowResult& result) {
  const int n = static_cast<int>(data.ac_buses.size());
  const int ndc = static_cast<int>(data.dc_buses.size());
  if (n == 0 || result.vm.size() != static_cast<size_t>(n) ||
      result.va.size() != static_cast<size_t>(n)) {
    return 0.0;
  }

  Eigen::VectorXd vm(n);
  Eigen::VectorXd va(n);
  Eigen::VectorXd vdc = Eigen::VectorXd::Ones(ndc);
  for (int i = 0; i < n; ++i) {
    vm[i] = result.vm[static_cast<size_t>(i)];
    va[i] = result.va[static_cast<size_t>(i)];
  }
  for (int i = 0; i < ndc && i < static_cast<int>(result.vdc.size()); ++i) {
    vdc[i] = result.vdc[static_cast<size_t>(i)];
  }

  Eigen::VectorXd pcalc = Eigen::VectorXd::Zero(n);
  // P_i = sum_j Vi Vj (Gij cos(theta_ij) + Bij sin(theta_ij)).
  // AUD-096 in docs/testing/module_code_audit.md: visit only Ybus nonzeros;
  // column-major traversal preserves ascending-j accumulation for each row.
  for (int j = 0; j < data.ybus.outerSize(); ++j) {
    for (decltype(data.ybus)::InnerIterator entry(data.ybus, j); entry; ++entry) {
      const int i = static_cast<int>(entry.row());
      const double g = entry.value().real();
      const double b = entry.value().imag();
      const double t = va[i] - va[j];
      const double c = std::cos(t);
      const double s = std::sin(t);
      pcalc[i] += vm[i] * vm[j] * (g * c + b * s);
    }
  }

  Eigen::VectorXd psch;
  Eigen::VectorXd qsch;
  assemble_ac_injections(data, vm, va, vdc, psch, qsch);

  // Sum over the complete solved AC network. Non-slack equations cancel to
  // numerical tolerance; the remainder is the active power supplied by all
  // slack buses. Restricting this sum to participating buses silently returns
  // zero whenever the physical slack is not itself a participant.
  return (pcalc - psch).sum();
}

struct BoundedAllocation {
  std::unordered_map<int, double> by_bus;
  std::vector<int> hit_limits;
  double unallocated{0.0};
};

BoundedAllocation distribute_with_limits(
    double total_slack_pu,
    const DistributedSlack& slack_cfg,
    const std::unordered_map<int, ParticipationBounds>& bounds,
    const std::unordered_map<int, double>& already_allocated = {}) {
  BoundedAllocation result;
  auto& alloc = result.by_bus;
  const int n = static_cast<int>(slack_cfg.participating_buses.size());
  if (n == 0) {
    result.unallocated = total_slack_pu;
    return result;
  }

  std::vector<double> alpha = slack_cfg.participation_factors;
  if (static_cast<int>(alpha.size()) != n) {
    alpha.assign(static_cast<size_t>(n), 1.0);
  }
  normalize_factors(alpha);

  std::vector<char> active(static_cast<size_t>(n), 1);
  double remaining = total_slack_pu;

  for (int pass = 0; pass < n; ++pass) {
    double active_sum = 0.0;
    for (int i = 0; i < n; ++i) {
      if (active[static_cast<size_t>(i)] != 0) {
        active_sum += alpha[static_cast<size_t>(i)];
      }
    }
    if (active_sum <= 0.0) {
      break;
    }

    bool any_limited = false;
    for (int i = 0; i < n; ++i) {
      if (active[static_cast<size_t>(i)] == 0) {
        continue;
      }
      const int bus = slack_cfg.participating_buses[static_cast<size_t>(i)];
      const double proposed = remaining * (alpha[static_cast<size_t>(i)] / active_sum);
      ParticipationBounds available;
      if (const auto bound_it = bounds.find(bus); bound_it != bounds.end()) {
        available = bound_it->second;
      }
      const double prior = already_allocated.count(bus) != 0
                               ? already_allocated.at(bus)
                               : 0.0;
      available.lower -= prior;
      available.upper -= prior;
      if (proposed < available.lower || proposed > available.upper) {
        const double clipped = std::clamp(
            proposed, available.lower, available.upper);
        alloc[bus] = clipped;
        remaining -= clipped;
        active[static_cast<size_t>(i)] = 0;
        result.hit_limits.push_back(bus);
        any_limited = true;
      }
    }

    if (!any_limited) {
      break;
    }
  }

  double active_sum = 0.0;
  for (int i = 0; i < n; ++i) {
    if (active[static_cast<size_t>(i)] != 0) {
      active_sum += alpha[static_cast<size_t>(i)];
    }
  }
  if (active_sum > 0.0) {
    for (int i = 0; i < n; ++i) {
      if (active[static_cast<size_t>(i)] == 0) {
        continue;
      }
      const int bus = slack_cfg.participating_buses[static_cast<size_t>(i)];
      alloc[bus] = remaining * (alpha[static_cast<size_t>(i)] / active_sum);
    }
  }

  const double allocated = std::accumulate(
      alloc.begin(), alloc.end(), 0.0,
      [](double sum, const auto& entry) { return sum + entry.second; });
  result.unallocated = total_slack_pu - allocated;
  std::sort(result.hit_limits.begin(), result.hit_limits.end());
  result.hit_limits.erase(
      std::unique(result.hit_limits.begin(), result.hit_limits.end()),
      result.hit_limits.end());
  return result;
}

bool apply_generation_delta(HybridPowerSystem& sys, int bus, double delta_pu) {
  std::vector<Generator*> generators;
  for (auto& generator : sys.ac.generators) {
    if (generator.in_service && generator.bus == bus) {
      generators.push_back(&generator);
    }
  }
  double remaining_mw = delta_pu * sys.base_mva;
  for (int pass = 0; pass < 2 && std::abs(remaining_mw) > 1e-10; ++pass) {
    std::vector<double> room(generators.size(), 0.0);
    double total_room = 0.0;
    for (size_t index = 0; index < generators.size(); ++index) {
      const auto& generator = *generators[index];
      room[index] = remaining_mw >= 0.0
                        ? std::max(0.0, generator.pmax_mw - generator.pg_mw)
                        : std::max(0.0, generator.pg_mw - generator.pmin_mw);
      total_room += room[index];
    }
    if (total_room <= 1e-12) break;
    const double requested = std::abs(remaining_mw);
    for (size_t index = 0; index < generators.size(); ++index) {
      if (room[index] <= 0.0) continue;
      const double magnitude = std::min(
          room[index], requested * room[index] / total_room);
      const double change = std::copysign(magnitude, remaining_mw);
      generators[index]->pg_mw += change;
      remaining_mw -= change;
    }
  }
  return std::abs(remaining_mw) <= 1e-8;
}

}  // namespace

DistributedSlack create_participation_factors(const HybridPowerSystem& sys,
                                               const std::string& method,
                                               const std::vector<int>& participating_buses,
                                               const std::unordered_map<int, double>& droop_coeffs) {
  const auto bus_pg = per_bus_generation_pu(sys);

  std::vector<int> buses;
  if (participating_buses.empty()) {
    for (const auto& b : sys.ac.buses) {
      if (b.in_service &&
          (b.bus_type == BusType::SLACK || b.bus_type == BusType::PV) &&
          bus_pg.at(b.index) > 0.0) {
        buses.push_back(b.index);
      }
    }
  } else {
    std::unordered_set<int> seen;
    seen.reserve(participating_buses.size());
    for (int bus : participating_buses) {
      if (!bus_pg.contains(bus) || seen.count(bus) != 0) {
        continue;
      }
      seen.insert(bus);
      buses.push_back(bus);
    }
  }

  if (buses.empty()) {
    throw std::invalid_argument("No valid participating buses found for distributed slack.");
  }

  const std::string mode = to_lower(method);
  std::vector<double> raw;
  raw.reserve(buses.size());
  std::unordered_map<int, double> max_p;
  max_p.reserve(buses.size());

  for (int bus : buses) {
    const double capacity = std::max(bus_pg.at(bus), 1e-4);
    if (mode == "capacity") {
      raw.push_back(capacity);
    } else if (mode == "droop") {
      const auto it = droop_coeffs.find(bus);
      const double coeff = (it == droop_coeffs.end()) ? 0.0 : it->second;
      raw.push_back((coeff > 0.0) ? (1.0 / coeff) : capacity);
    } else if (mode == "equal") {
      raw.push_back(1.0);
    } else {
      throw std::invalid_argument(
          "Unknown distributed slack method: " + method +
          " (supported: capacity, droop, equal).");
    }

    double scheduled_mw = 0.0;
    double pmin_mw = 0.0;
    double pmax_mw = 0.0;
    for (const auto& generator : sys.ac.generators) {
      if (!generator.in_service || generator.bus != bus) continue;
      scheduled_mw += generator.pg_mw;
      pmin_mw += generator.pmin_mw;
      pmax_mw += generator.pmax_mw;
    }
    max_p[bus] = std::max(std::abs(pmin_mw - scheduled_mw),
                          std::abs(pmax_mw - scheduled_mw)) /
                 sys.base_mva;
  }

  normalize_factors(raw);

  DistributedSlack cfg;
  cfg.participating_buses = std::move(buses);
  cfg.participation_factors = std::move(raw);
  cfg.reference_bus = cfg.participating_buses.front();
  cfg.max_participation_p = std::move(max_p);
  return cfg;
}

DistributedSlackResult DistributedSlackSolver::solve_simplified(const HybridPowerSystem& sys,
                                                                const DistributedSlack& slack_cfg,
                                                                const PowerFlowOptions& opt) const {
  DistributedSlackResult out;
  const ConverterCoordinationReport coordination =
      evaluate_converter_coordination(sys, opt.enable_converter_coordination_check);
  out.diagnostics.converter_coordination = coordination;
  if (coordination.enabled && coordination.has_blocking_issue()) {
    out.diagnostics.termination_reason = "Converter coordination feasibility check failed";
    for (const auto& issue : coordination.issues) {
      out.diagnostics.warnings.push_back("[" + issue.rule_id + "] " + issue.message);
    }
    return out;
  }

  DistributedSlack cfg = sanitize_slack_cfg(sys, slack_cfg);
  HybridPowerSystem working = sys;
  apply_reference_bus(working, cfg.reference_bus);
  out.reference_bus_used = cfg.reference_bus;
  out.model_scope = "post-solve-distributed-slack-allocation";
  out.model_limitations.push_back(
      "Allocation is a post-solve report and does not redispatch or re-solve the network.");

  SolverData data = make_solver_data(working, opt.loss_model);
  apply_power_flow_options(data, opt);
  NewtonSolver solver;
  const PowerFlowResult base = solver.solve(data, opt, nullptr);
  out.diagnostics = base.diagnostics;
  out.diagnostics.converter_coordination = coordination;
  if (coordination.enabled) {
    for (const auto& issue : coordination.issues) {
      if (issue.severity == CoordinationSeverity::Warning ||
          issue.severity == CoordinationSeverity::Info) {
        out.diagnostics.warnings.push_back("[" + issue.rule_id + "] " + issue.message);
      }
    }
  }

  if (data.bus_merge_map && data.bus_merge_map->has_merges()) {
    out.vm = unproject_bus_vector(
        base.vm, *data.bus_merge_map, BusVectorSemantics::Intensive);
    out.va = unproject_bus_vector(
        base.va, *data.bus_merge_map, BusVectorSemantics::Intensive);
  } else {
    out.vm = base.vm;
    out.va = base.va;
  }
  out.vdc = base.vdc;
  out.converged = base.converged;
  out.iterations = base.iterations;
  out.residual = base.residual;
  out.hit_limits.clear();
  out.distributed_slack_p.clear();

  if (!base.converged || cfg.participating_buses.empty()) {
    return out;
  }

  const double total_slack =
      compute_total_participating_mismatch_pu(data, base);
  for (int i = 0; i < static_cast<int>(cfg.participating_buses.size()); ++i) {
    const int bus = cfg.participating_buses[static_cast<size_t>(i)];
    const double alpha = cfg.participation_factors[static_cast<size_t>(i)];
    out.distributed_slack_p[bus] = alpha * total_slack;
  }
  return out;
}

DistributedSlackResult DistributedSlackSolver::solve_full_jacobian(const HybridPowerSystem& sys,
                                                                   const DistributedSlack& slack_cfg,
                                                                   const PowerFlowOptions& opt) const {
  DistributedSlackResult out;
  const ConverterCoordinationReport coordination =
      evaluate_converter_coordination(sys, opt.enable_converter_coordination_check);
  out.diagnostics.converter_coordination = coordination;
  if (coordination.enabled && coordination.has_blocking_issue()) {
    out.diagnostics.termination_reason =
        "Converter coordination feasibility check failed";
    for (const auto& issue : coordination.issues) {
      out.diagnostics.warnings.push_back(
          "[" + issue.rule_id + "] " + issue.message);
    }
    return out;
  }

  DistributedSlack cfg = sanitize_slack_cfg(sys, slack_cfg);
  HybridPowerSystem working = sys;
  apply_reference_bus(working, cfg.reference_bus);
  const auto bounds = generator_delta_bounds(sys, cfg);

  out.reference_bus_used = cfg.reference_bus;
  out.model_scope = "iterative-newton-bounded-distributed-slack-redispatch";
  out.model_limitations.push_back(
      "The legacy solve_full_jacobian API name is retained for compatibility; "
      "the implemented algorithm is repeated full Newton redispatch, not one augmented Jacobian.");
  out.diagnostics.warnings.push_back(out.model_limitations.front());
  for (int bus : cfg.participating_buses) {
    out.distributed_slack_p[bus] = 0.0;
  }

  constexpr int kMaxRedispatchSolves = 20;
  const double redispatch_tol = std::max(1e-10, opt.tol);
  for (int outer = 0; outer < kMaxRedispatchSolves; ++outer) {
    SolverData data = make_solver_data(working, opt.loss_model);
    apply_power_flow_options(data, opt);
    NewtonSolver solver;
    const PowerFlowResult solved = solver.solve(data, opt, nullptr);
    out.diagnostics = solved.diagnostics;
    out.diagnostics.converter_coordination = coordination;
    out.diagnostics.warnings.push_back(out.model_limitations.front());
    if (coordination.enabled) {
      for (const auto& issue : coordination.issues) {
        if (issue.severity == CoordinationSeverity::Warning ||
            issue.severity == CoordinationSeverity::Info) {
          out.diagnostics.warnings.push_back(
              "[" + issue.rule_id + "] " + issue.message);
        }
      }
    }
    out.vm = data.bus_merge_map && data.bus_merge_map->has_merges()
                 ? unproject_bus_vector(
                       solved.vm, *data.bus_merge_map,
                       BusVectorSemantics::Intensive)
                 : solved.vm;
    out.va = data.bus_merge_map && data.bus_merge_map->has_merges()
                 ? unproject_bus_vector(
                       solved.va, *data.bus_merge_map,
                       BusVectorSemantics::Intensive)
                 : solved.va;
    out.vdc = solved.vdc;
    out.iterations += solved.iterations;
    out.residual = solved.residual;
    if (!solved.converged) {
      out.converged = false;
      return out;
    }

    const double remaining_slack =
        compute_total_participating_mismatch_pu(data, solved);
    out.unallocated_slack_pu = remaining_slack;
    if (std::abs(remaining_slack) <= redispatch_tol) {
      out.converged = true;
      out.unallocated_slack_pu = 0.0;
      return out;
    }

    BoundedAllocation allocation = distribute_with_limits(
        remaining_slack, cfg, bounds, out.distributed_slack_p);
    out.hit_limits.insert(out.hit_limits.end(),
                          allocation.hit_limits.begin(),
                          allocation.hit_limits.end());
    if (std::abs(allocation.unallocated) > redispatch_tol) {
      out.converged = false;
      out.unallocated_slack_pu = allocation.unallocated;
      out.diagnostics.converged = false;
      out.diagnostics.termination_reason =
          "Distributed slack generator limits leave unallocated active power";
      out.diagnostics.warnings.push_back(
          "The requested slack correction exceeds aggregate pmin/pmax and participation limits.");
      return out;
    }
    for (const auto& [bus, delta] : allocation.by_bus) {
      if (!apply_generation_delta(working, bus, delta)) {
        out.converged = false;
        out.unallocated_slack_pu = remaining_slack;
        out.diagnostics.converged = false;
        out.diagnostics.termination_reason =
            "Distributed slack generation update could not satisfy pmin/pmax";
        return out;
      }
      out.distributed_slack_p[bus] += delta;
    }
  }

  out.converged = false;
  out.diagnostics.converged = false;
  out.diagnostics.termination_reason =
      "Distributed slack redispatch did not absorb loss changes within 20 Newton solves";
  std::sort(out.hit_limits.begin(), out.hit_limits.end());
  out.hit_limits.erase(
      std::unique(out.hit_limits.begin(), out.hit_limits.end()),
      out.hit_limits.end());
  return out;
}

}  // namespace hacdcpf::powerflow
