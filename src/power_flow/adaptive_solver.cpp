#include "hacdcpf/power_flow/adaptive_solver.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "hacdcpf/projection/result_attribution.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/util/thread_pool.hpp"

namespace hacdcpf::powerflow {

namespace {

constexpr double kPi = 3.14159265358979323846;

double bus_generation_mw(const HybridPowerSystem& sys, int bus) {
  double pg = 0.0;
  for (const auto& g : sys.ac.generators) {
    if (g.in_service && g.bus == bus) {
      pg += g.pg_mw;
    }
  }
  return pg;
}

int auto_select_swing_bus(const HybridPowerSystem& sys, const IslandInfo& island,
                          const std::unordered_map<int, int>& ac_id_to_pos) {
  if (island.ac_buses.empty()) {
    return 0;
  }
  for (int bus : island.ac_buses) {
    const auto it = ac_id_to_pos.find(bus);
    if (it != ac_id_to_pos.end() &&
        sys.ac.buses[static_cast<size_t>(it->second)].bus_type == BusType::SLACK) {
      return bus;
    }
  }

  int best_bus = 0;
  double best_pg = -1.0;
  for (int bus : island.ac_buses) {
    const double pg = bus_generation_mw(sys, bus);
    if (pg > best_pg) {
      best_pg = pg;
      best_bus = bus;
    }
  }
  if (best_bus > 0 && best_pg > 0.0) {
    return best_bus;
  }

  for (int bus : island.ac_buses) {
    const auto it = ac_id_to_pos.find(bus);
    if (it != ac_id_to_pos.end() &&
        sys.ac.buses[static_cast<size_t>(it->second)].bus_type == BusType::PV) {
      return bus;
    }
  }

  return island.ac_buses.front();
}

void map_island_result_back(const IslandInfo& island,
                            const PowerFlowResult& local,
                            const std::unordered_map<int, int>& ac_id_to_pos,
                            const std::unordered_map<int, int>& dc_id_to_pos,
                            AdaptiveSolveResult& global) {
  std::vector<int> ac_sorted = island.ac_buses;
  std::vector<int> dc_sorted = island.dc_buses;
  std::sort(ac_sorted.begin(), ac_sorted.end());
  std::sort(dc_sorted.begin(), dc_sorted.end());

  for (int i = 0; i < static_cast<int>(ac_sorted.size()) && i < static_cast<int>(local.vm.size()); ++i) {
    const auto it = ac_id_to_pos.find(ac_sorted[static_cast<size_t>(i)]);
    if (it == ac_id_to_pos.end()) continue;
    const int bus = it->second;
    if (bus >= 0 && bus < static_cast<int>(global.vm.size())) {
      global.vm[static_cast<size_t>(bus)] = local.vm[static_cast<size_t>(i)];
      global.va[static_cast<size_t>(bus)] = local.va[static_cast<size_t>(i)];
    }
  }
  for (int i = 0; i < static_cast<int>(dc_sorted.size()) && i < static_cast<int>(local.vdc.size()); ++i) {
    const auto it = dc_id_to_pos.find(dc_sorted[static_cast<size_t>(i)]);
    if (it == dc_id_to_pos.end()) continue;
    const int bus = it->second;
    if (bus >= 0 && bus < static_cast<int>(global.vdc.size())) {
      global.vdc[static_cast<size_t>(bus)] = local.vdc[static_cast<size_t>(i)];
    }
  }
}

void set_dead_island_zero(const IslandInfo& island,
                          const std::unordered_map<int, int>& ac_id_to_pos,
                          const std::unordered_map<int, int>& dc_id_to_pos,
                          AdaptiveSolveResult& result) {
  for (int bus : island.ac_buses) {
    const auto it = ac_id_to_pos.find(bus);
    if (it == ac_id_to_pos.end()) continue;
    const int idx = it->second;
    if (idx >= 0 && idx < static_cast<int>(result.vm.size())) {
      result.vm[static_cast<size_t>(idx)] = 0.0;
      result.va[static_cast<size_t>(idx)] = 0.0;
    }
  }
  for (int bus : island.dc_buses) {
    const auto it = dc_id_to_pos.find(bus);
    if (it == dc_id_to_pos.end()) continue;
    const int idx = it->second;
    if (idx >= 0 && idx < static_cast<int>(result.vdc.size())) {
      result.vdc[static_cast<size_t>(idx)] = 0.0;
    }
  }
}

void merge_island_diagnostics(const PowerFlowResult& local,
                              size_t island_index,
                              AdaptiveSolveResult& global) {
  global.profiling.jacobian_pattern_rebuilds +=
      local.profiling.jacobian_pattern_rebuilds;
  global.profiling.jacobian_analyze_calls +=
      local.profiling.jacobian_analyze_calls;
  global.profiling.factorization_calls += local.profiling.factorization_calls;
  global.profiling.linear_solve_calls += local.profiling.linear_solve_calls;
  global.profiling.regularization_attempts +=
      local.profiling.regularization_attempts;
  global.profiling.line_search_evaluations +=
      local.profiling.line_search_evaluations;
  global.profiling.rejected_steps += local.profiling.rejected_steps;
  global.profiling.pv_to_pq_switches += local.profiling.pv_to_pq_switches;
  global.profiling.pq_to_pv_switches += local.profiling.pq_to_pv_switches;
  global.profiling.pv_pq_outer_iterations +=
      local.profiling.pv_pq_outer_iterations;
  global.profiling.pv_pq_repeated_active_sets +=
      local.profiling.pv_pq_repeated_active_sets;
  global.profiling.smooth_ncp_continuation_updates +=
      local.profiling.smooth_ncp_continuation_updates;
  global.profiling.smooth_ncp_final_mu = std::max(
      global.profiling.smooth_ncp_final_mu,
      local.profiling.smooth_ncp_final_mu);
  global.profiling.converter_mode_switches +=
      local.profiling.converter_mode_switches;
  global.profiling.eval_jacobian_ms_total +=
      local.profiling.eval_jacobian_ms_total;
  global.profiling.linear_solve_ms_total +=
      local.profiling.linear_solve_ms_total;
  global.profiling.line_search_ms_total +=
      local.profiling.line_search_ms_total;
  global.profiling.raw_residual_norm = std::max(
      global.profiling.raw_residual_norm,
      local.profiling.raw_residual_norm);
  global.profiling.scaled_residual_norm = std::max(
      global.profiling.scaled_residual_norm,
      local.profiling.scaled_residual_norm);
  global.profiling.condition_estimate = std::max(
      global.profiling.condition_estimate,
      local.profiling.condition_estimate);

  global.reactive_limits.active_set_cycle_detected |=
      local.reactive_limits.active_set_cycle_detected;
  global.reactive_limits.outer_iteration_limit_reached |=
      local.reactive_limits.outer_iteration_limit_reached;
  global.reactive_limits.active_limited_buses +=
      local.reactive_limits.active_limited_buses;
  global.reactive_limits.max_violation_pu = std::max(
      global.reactive_limits.max_violation_pu,
      local.reactive_limits.max_violation_pu);
  if (global.reactive_limits.enforcement_requested) {
    global.reactive_limits.certified &= local.reactive_limits.certified;
  }

  for (const auto& warning : local.diagnostics.warnings) {
    global.diagnostics.warnings.push_back(
        "Island " + std::to_string(island_index) + ": " + warning);
  }
}

HybridPowerSystem apply_reactive_limit_overrides(
    const HybridPowerSystem& sys,
    const std::unordered_map<int, ReactiveLimit>& q_limits) {
  HybridPowerSystem working = sys;
  const double base_mva =
      working.ac.base_mva > 0.0 ? working.ac.base_mva : working.base_mva;
  if (!(base_mva > 0.0) || !std::isfinite(base_mva)) {
    throw std::runtime_error(
        "AdaptiveSolver reactive-limit overrides require positive base_mva");
  }
  for (const auto& [bus_id, limit] : q_limits) {
    if (!std::isfinite(limit.qmin) || !std::isfinite(limit.qmax) ||
        limit.qmin > limit.qmax) {
      throw std::runtime_error(
          "AdaptiveSolver reactive limit for AC bus " +
          std::to_string(bus_id) + " is invalid");
    }
    bool first = true;
    bool found = false;
    for (auto& generator : working.ac.generators) {
      if (!generator.in_service || generator.bus != bus_id) continue;
      found = true;
      generator.qmin_mvar = first ? limit.qmin * base_mva : 0.0;
      generator.qmax_mvar = first ? limit.qmax * base_mva : 0.0;
      first = false;
    }
    if (!found) {
      throw std::runtime_error(
          "AdaptiveSolver reactive limit references AC bus " +
          std::to_string(bus_id) + " without an in-service generator");
    }
  }
  return working;
}

}  // namespace

AdaptiveSolveResult AdaptiveSolver::solve(const HybridPowerSystem& sys,
                                          const PowerFlowOptions& opt,
                                          const std::unordered_map<int, ReactiveLimit>& q_limits) const {
  const HybridPowerSystem working =
      apply_reactive_limit_overrides(sys, q_limits);

  AdaptiveSolveResult out;
  const int nac = static_cast<int>(working.ac.buses.size());
  const int ndc = static_cast<int>(working.dc.buses.size());
  out.vm.assign(static_cast<size_t>(nac), 1.0);
  out.va.assign(static_cast<size_t>(nac), 0.0);
  out.vdc.assign(static_cast<size_t>(ndc), 1.0);
  out.converged = true;
  out.iterations = 0;
  out.residual = 0.0;
  out.reactive_limits.enforcement_requested =
      opt.enable_pv_pq_conversion || opt.enable_semi_smooth_newton;
  out.reactive_limits.certified =
      out.reactive_limits.enforcement_requested;

  for (int i = 0; i < nac; ++i) {
    out.vm[static_cast<size_t>(i)] = working.ac.buses[static_cast<size_t>(i)].vm_pu;
    out.va[static_cast<size_t>(i)] = working.ac.buses[static_cast<size_t>(i)].va_deg * kPi / 180.0;
  }
  for (int i = 0; i < ndc; ++i) {
    out.vdc[static_cast<size_t>(i)] = working.dc.buses[static_cast<size_t>(i)].vm_pu;
  }

  out.islands = detect_islands(working);

  // Single-island fast path: solve the whole system directly without
  // subsystem extraction to avoid any data loss.
  const bool single_island = (out.islands.size() == 1 && out.islands[0].has_generators);
  if (out.islands.empty() || single_island) {
    SolverData data = make_solver_data(working, opt.loss_model);
    NewtonSolver solver;
    PowerFlowResult r = solver.solve(data, opt, nullptr);
    if (!r.converged) {
      InitialState flat;
      flat.vm.assign(data.ac_buses.size(), 1.0);
      flat.va.assign(data.ac_buses.size(), 0.0);
      flat.vdc.assign(data.dc_buses.size(), 1.0);
      r = solver.solve(data, opt, &flat);
    }
    if (data.bus_merge_map && data.bus_merge_map->has_merges()) {
      out.vm = unproject_bus_vector(
          r.vm, *data.bus_merge_map, BusVectorSemantics::Intensive);
      out.va = unproject_bus_vector(
          r.va, *data.bus_merge_map, BusVectorSemantics::Intensive);
    } else {
      out.vm = r.vm;
      out.va = r.va;
    }
    out.vdc = r.vdc;
    out.converged = r.converged;
    out.iterations = r.iterations;
    out.residual = r.residual;
    out.diagnostics = r.diagnostics;
    out.profiling = r.profiling;
    out.reactive_limits = r.reactive_limits;
    return out;
  }

  // --- Parallel multi-island solving ---
  // Identify solvable islands and handle dead ones immediately.
  // Bus IDs may be non-contiguous, so map each real index to its array
  // position once and reuse it when writing per-island results back into the
  // global (position-indexed) voltage vectors.
  std::unordered_map<int, int> ac_id_to_pos;
  std::unordered_map<int, int> dc_id_to_pos;
  ac_id_to_pos.reserve(static_cast<size_t>(nac));
  dc_id_to_pos.reserve(static_cast<size_t>(ndc));
  for (int i = 0; i < nac; ++i) {
    ac_id_to_pos[working.ac.buses[static_cast<size_t>(i)].index] = i;
  }
  for (int i = 0; i < ndc; ++i) {
    dc_id_to_pos[working.dc.buses[static_cast<size_t>(i)].index] = i;
  }

  struct IslandTask {
    size_t island_idx;
    int slack_override;
  };
  std::vector<IslandTask> tasks;
  for (size_t ii = 0; ii < out.islands.size(); ++ii) {
    if (!out.islands[ii].has_generators) {
      set_dead_island_zero(out.islands[ii], ac_id_to_pos, dc_id_to_pos, out);
      continue;
    }
    int slack_override = 0;
    if (opt.enable_auto_swing_selection && !out.islands[ii].has_ac_slack) {
      slack_override = auto_select_swing_bus(working, out.islands[ii], ac_id_to_pos);
    }
    tasks.push_back({ii, slack_override});
  }

  // Helper: solve one island, returning its PowerFlowResult.
  auto solve_island = [&](const IslandTask& task,
                          const PowerFlowOptions& pf_opt) -> PowerFlowResult {
    const auto& island = out.islands[task.island_idx];
    NewtonSolver solver;
    HybridPowerSystem sub =
        extract_island_subsystem(working, island, task.slack_override);
    SolverData sub_data = make_solver_data_projected(
        projection::RichToCanonicalOperator::apply(std::move(sub)).canonical,
        pf_opt.loss_model);

    PowerFlowResult r = solver.solve(sub_data, pf_opt, nullptr);
    if (!r.converged) {
      InitialState flat;
      flat.vm.assign(sub_data.ac_buses.size(), 1.0);
      flat.va.assign(sub_data.ac_buses.size(), 0.0);
      flat.vdc.assign(sub_data.dc_buses.size(), 1.0);
      r = solver.solve(sub_data, pf_opt, &flat);
    }
    if (sub_data.bus_merge_map && sub_data.bus_merge_map->has_merges()) {
      r.vm = unproject_bus_vector(
          r.vm, *sub_data.bus_merge_map, BusVectorSemantics::Intensive);
      r.va = unproject_bus_vector(
          r.va, *sub_data.bus_merge_map, BusVectorSemantics::Intensive);
    }
    return r;
  };

  if (tasks.size() <= 1) {
    // Single solvable island: use Jacobian-level parallelism (ac_eval_threads).
    for (const auto& task : tasks) {
      PowerFlowResult r = solve_island(task, opt);
      if (!r.converged) out.converged = false;
      out.iterations += r.iterations;
      out.residual = std::max(out.residual, r.residual);
      merge_island_diagnostics(r, task.island_idx, out);
      map_island_result_back(out.islands[task.island_idx], r, ac_id_to_pos, dc_id_to_pos, out);
    }
  } else {
    // Multiple islands: solve concurrently via thread pool.
    // Force serial Jacobian eval per island to avoid nested-parallelism deadlock.
    PowerFlowOptions par_opt = opt;
    par_opt.ac_eval_threads = 1;

    auto& pool = util::ThreadPool::global();
    std::vector<std::future<PowerFlowResult>> futures;
    futures.reserve(tasks.size());
    for (size_t ti = 0; ti < tasks.size(); ++ti) {
      futures.push_back(pool.submit(
          [&, ti]() -> PowerFlowResult { return solve_island(tasks[ti], par_opt); }));
    }

    // Collect and merge results sequentially.
    for (size_t ti = 0; ti < tasks.size(); ++ti) {
      PowerFlowResult r = futures[ti].get();
      if (!r.converged) out.converged = false;
      out.iterations += r.iterations;
      out.residual = std::max(out.residual, r.residual);
      merge_island_diagnostics(r, tasks[ti].island_idx, out);
      map_island_result_back(out.islands[tasks[ti].island_idx], r, ac_id_to_pos, dc_id_to_pos, out);
    }
  }

  return out;
}

}  // namespace hacdcpf::powerflow
