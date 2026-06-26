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
#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

namespace {

std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

std::vector<double> per_bus_generation_pu(const HybridPowerSystem& sys) {
  const int n = static_cast<int>(sys.ac.buses.size());
  std::vector<double> pg(static_cast<size_t>(n + 1), 0.0);
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service || g.bus < 1 || g.bus > n) {
      continue;
    }
    pg[static_cast<size_t>(g.bus)] += g.pg_mw / sys.base_mva;
  }
  return pg;
}

void normalize_factors(std::vector<double>& factors) {
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
  if (cfg.reference_bus == 0 && !cfg.participating_buses.empty()) {
    cfg.reference_bus = cfg.participating_buses.front();
  }
  return cfg;
}

double compute_total_participating_mismatch_pu(const SolverData& data,
                                               const PowerFlowResult& result,
                                               const std::vector<int>& buses) {
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

  const Eigen::MatrixXcd ybus = Eigen::MatrixXcd(data.ybus);
  Eigen::VectorXd pcalc = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double g = ybus(i, j).real();
      const double b = ybus(i, j).imag();
      const double t = va[i] - va[j];
      const double c = std::cos(t);
      const double s = std::sin(t);
      pcalc[i] += vm[i] * vm[j] * (g * c + b * s);
    }
  }

  Eigen::VectorXd psch = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    const auto& bus = data.ac_buses[static_cast<size_t>(i)];
    psch[i] = data.pg[i] - bus.pd_mw / data.base_mva;
  }
  for (const auto& conv : data.converters) {
    if (!conv.in_service) {
      continue;
    }
    const int ac = conv.bus_ac - 1;
    if (ac >= 0 && ac < n) {
      const auto [pac, qac] =
          converter_ac_injection(conv, vm, va, vdc, data.base_mva, data.loss_model);
      (void)qac;
      psch[ac] += pac;
    }
  }

  double total = 0.0;
  for (int bus : buses) {
    const int idx = bus - 1;
    if (idx >= 0 && idx < n) {
      total += pcalc[idx] - psch[idx];
    }
  }
  return total;
}

std::unordered_map<int, double> distribute_with_limits(double total_slack_pu,
                                                       const DistributedSlack& slack_cfg,
                                                       std::vector<int>& hit_limits) {
  std::unordered_map<int, double> alloc;
  const int n = static_cast<int>(slack_cfg.participating_buses.size());
  if (n == 0) {
    return alloc;
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
      const auto it_lim = slack_cfg.max_participation_p.find(bus);
      const double lim =
          (it_lim == slack_cfg.max_participation_p.end()) ? std::numeric_limits<double>::infinity()
                                                          : std::abs(it_lim->second);
      if (std::abs(proposed) > lim) {
        const double clipped = std::copysign(lim, proposed);
        alloc[bus] = clipped;
        remaining -= clipped;
        active[static_cast<size_t>(i)] = 0;
        hit_limits.push_back(bus);
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

  std::sort(hit_limits.begin(), hit_limits.end());
  hit_limits.erase(std::unique(hit_limits.begin(), hit_limits.end()), hit_limits.end());
  return alloc;
}

}  // namespace

DistributedSlack create_participation_factors(const HybridPowerSystem& sys,
                                               const std::string& method,
                                               const std::vector<int>& participating_buses,
                                               const std::unordered_map<int, double>& droop_coeffs) {
  const int nac = static_cast<int>(sys.ac.buses.size());
  const std::vector<double> bus_pg = per_bus_generation_pu(sys);

  std::vector<int> buses;
  if (participating_buses.empty()) {
    for (int i = 1; i <= nac; ++i) {
      const auto& b = sys.ac.buses[static_cast<size_t>(i - 1)];
      if ((b.bus_type == BusType::SLACK || b.bus_type == BusType::PV) &&
          bus_pg[static_cast<size_t>(i)] > 0.0) {
        buses.push_back(i);
      }
    }
  } else {
    std::unordered_set<int> seen;
    seen.reserve(participating_buses.size());
    for (int bus : participating_buses) {
      if (bus < 1 || bus > nac || seen.count(bus) != 0) {
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
    const double capacity = std::max(bus_pg[static_cast<size_t>(bus)], 1e-4);
    if (mode == "capacity") {
      raw.push_back(capacity);
      max_p[bus] = 1.5 * capacity;
    } else if (mode == "droop") {
      const auto it = droop_coeffs.find(bus);
      const double coeff = (it == droop_coeffs.end()) ? 0.0 : it->second;
      raw.push_back((coeff > 0.0) ? (1.0 / coeff) : capacity);
      max_p[bus] = 1.5 * capacity;
    } else if (mode == "equal") {
      raw.push_back(1.0);
      max_p[bus] = 10.0;
    } else {
      throw std::invalid_argument(
          "Unknown distributed slack method: " + method +
          " (supported: capacity, droop, equal).");
    }
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

  SolverData data = make_solver_data(sys, opt.loss_model);
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
    out.vm = unproject_bus_vector(base.vm, *data.bus_merge_map);
    out.va = unproject_bus_vector(base.va, *data.bus_merge_map);
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
      compute_total_participating_mismatch_pu(data, base, cfg.participating_buses);
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
  DistributedSlackResult out = solve_simplified(sys, slack_cfg, opt);
  if (!out.converged) {
    return out;
  }

  DistributedSlack cfg = sanitize_slack_cfg(sys, slack_cfg);
  double total_slack = 0.0;
  for (const auto& kv : out.distributed_slack_p) {
    total_slack += kv.second;
  }

  std::vector<int> hit_limits;
  out.distributed_slack_p = distribute_with_limits(total_slack, cfg, hit_limits);
  out.hit_limits = std::move(hit_limits);
  return out;
}

}  // namespace hacdcpf::powerflow
