#include "hacdcpf/power_flow/adaptive_solver.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <vector>

#include "hacdcpf/model/network_utils.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/solver_data.hpp"
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

int auto_select_swing_bus(const HybridPowerSystem& sys, const IslandInfo& island) {
  if (island.ac_buses.empty()) {
    return 0;
  }
  for (int bus : island.ac_buses) {
    if (bus >= 1 && bus <= static_cast<int>(sys.ac.buses.size()) &&
        sys.ac.buses[static_cast<size_t>(bus - 1)].bus_type == BusType::SLACK) {
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
    if (bus >= 1 && bus <= static_cast<int>(sys.ac.buses.size()) &&
        sys.ac.buses[static_cast<size_t>(bus - 1)].bus_type == BusType::PV) {
      return bus;
    }
  }

  return island.ac_buses.front();
}

void map_island_result_back(const IslandInfo& island,
                            const PowerFlowResult& local,
                            AdaptiveSolveResult& global) {
  std::vector<int> ac_sorted = island.ac_buses;
  std::vector<int> dc_sorted = island.dc_buses;
  std::sort(ac_sorted.begin(), ac_sorted.end());
  std::sort(dc_sorted.begin(), dc_sorted.end());

  for (int i = 0; i < static_cast<int>(ac_sorted.size()) && i < static_cast<int>(local.vm.size()); ++i) {
    const int bus = ac_sorted[static_cast<size_t>(i)] - 1;
    if (bus >= 0 && bus < static_cast<int>(global.vm.size())) {
      global.vm[static_cast<size_t>(bus)] = local.vm[static_cast<size_t>(i)];
      global.va[static_cast<size_t>(bus)] = local.va[static_cast<size_t>(i)];
    }
  }
  for (int i = 0; i < static_cast<int>(dc_sorted.size()) && i < static_cast<int>(local.vdc.size()); ++i) {
    const int bus = dc_sorted[static_cast<size_t>(i)] - 1;
    if (bus >= 0 && bus < static_cast<int>(global.vdc.size())) {
      global.vdc[static_cast<size_t>(bus)] = local.vdc[static_cast<size_t>(i)];
    }
  }
}

void set_dead_island_zero(const IslandInfo& island, AdaptiveSolveResult& result) {
  for (int bus : island.ac_buses) {
    const int idx = bus - 1;
    if (idx >= 0 && idx < static_cast<int>(result.vm.size())) {
      result.vm[static_cast<size_t>(idx)] = 0.0;
      result.va[static_cast<size_t>(idx)] = 0.0;
    }
  }
  for (int bus : island.dc_buses) {
    const int idx = bus - 1;
    if (idx >= 0 && idx < static_cast<int>(result.vdc.size())) {
      result.vdc[static_cast<size_t>(idx)] = 0.0;
    }
  }
}

}  // namespace

AdaptiveSolveResult AdaptiveSolver::solve(const HybridPowerSystem& sys,
                                          const PowerFlowOptions& opt,
                                          const std::unordered_map<int, ReactiveLimit>& q_limits) const {
  (void)q_limits;

  AdaptiveSolveResult out;
  const int nac = static_cast<int>(sys.ac.buses.size());
  const int ndc = static_cast<int>(sys.dc.buses.size());
  out.vm.assign(static_cast<size_t>(nac), 1.0);
  out.va.assign(static_cast<size_t>(nac), 0.0);
  out.vdc.assign(static_cast<size_t>(ndc), 1.0);
  out.converged = true;
  out.iterations = 0;
  out.residual = 0.0;

  for (int i = 0; i < nac; ++i) {
    out.vm[static_cast<size_t>(i)] = sys.ac.buses[static_cast<size_t>(i)].vm_pu;
    out.va[static_cast<size_t>(i)] = sys.ac.buses[static_cast<size_t>(i)].va_deg * kPi / 180.0;
  }
  for (int i = 0; i < ndc; ++i) {
    out.vdc[static_cast<size_t>(i)] = sys.dc.buses[static_cast<size_t>(i)].vm_pu;
  }

  out.islands = detect_islands(sys);

  // Single-island fast path: solve the whole system directly without
  // subsystem extraction to avoid any data loss.
  const bool single_island = (out.islands.size() == 1 && out.islands[0].has_generators);
  if (out.islands.empty() || single_island) {
    SolverData data = make_solver_data(sys, opt.loss_model);
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
      out.vm = unproject_bus_vector(r.vm, *data.bus_merge_map);
      out.va = unproject_bus_vector(r.va, *data.bus_merge_map);
    } else {
      out.vm = r.vm;
      out.va = r.va;
    }
    out.vdc = r.vdc;
    out.converged = r.converged;
    out.iterations = r.iterations;
    out.residual = r.residual;
    return out;
  }

  // --- Parallel multi-island solving ---
  // Identify solvable islands and handle dead ones immediately.
  struct IslandTask {
    size_t island_idx;
    int slack_override;
  };
  std::vector<IslandTask> tasks;
  for (size_t ii = 0; ii < out.islands.size(); ++ii) {
    if (!out.islands[ii].has_generators) {
      set_dead_island_zero(out.islands[ii], out);
      continue;
    }
    int slack_override = 0;
    if (opt.enable_auto_swing_selection && !out.islands[ii].has_ac_slack) {
      slack_override = auto_select_swing_bus(sys, out.islands[ii]);
    }
    tasks.push_back({ii, slack_override});
  }

  // Helper: solve one island, returning its PowerFlowResult.
  auto solve_island = [&](const IslandTask& task,
                          const PowerFlowOptions& pf_opt) -> PowerFlowResult {
    const auto& island = out.islands[task.island_idx];
    NewtonSolver solver;
    HybridPowerSystem sub = extract_island_subsystem(sys, island, task.slack_override);
    SolverData sub_data = make_solver_data_projected(
        project_to_canonical_models(std::move(sub)), pf_opt.loss_model);

    PowerFlowResult r = solver.solve(sub_data, pf_opt, nullptr);
    if (!r.converged) {
      InitialState flat;
      flat.vm.assign(sub_data.ac_buses.size(), 1.0);
      flat.va.assign(sub_data.ac_buses.size(), 0.0);
      flat.vdc.assign(sub_data.dc_buses.size(), 1.0);
      r = solver.solve(sub_data, pf_opt, &flat);
    }
    if (sub_data.bus_merge_map && sub_data.bus_merge_map->has_merges()) {
      r.vm = unproject_bus_vector(r.vm, *sub_data.bus_merge_map);
      r.va = unproject_bus_vector(r.va, *sub_data.bus_merge_map);
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
      map_island_result_back(out.islands[task.island_idx], r, out);
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
      map_island_result_back(out.islands[tasks[ti].island_idx], r, out);
    }
  }

  return out;
}

}  // namespace hacdcpf::powerflow
