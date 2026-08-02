// assignment_lp.cpp
// ──────────────────────────────────────────────────────────────────────────
// System-optimal LP/MILP route assignment for EV power-traffic coupling.
// Defines solve_system_optimal_assignment() declared in evpt_internal.hpp.

#include "evpt_internal.hpp"

#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace hacdcpf::evpt {

void solve_system_optimal_assignment(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& options,
    int T,
    double dt,
    const std::unordered_map<int, std::size_t>& link_pos,
    const std::vector<std::vector<double>>& link_capacity,
    const std::unordered_map<int, double>& charge_cap_kw,
    EVPowerTrafficResult& result,
    std::vector<std::vector<double>>& link_used,
    std::vector<EVChargingSession>& sessions,
    int& next_session_index) {
  SystemOptimalSolve cert;
  cert.is_mip = is_system_optimal_mip(options.assignment_model);

  std::vector<int> active_demand_positions;
  active_demand_positions.reserve(problem.demands.size());
  std::unordered_map<int, int> demand_eq_row;

  for (int di = 0; di < static_cast<int>(problem.demands.size()); ++di) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    if (demand.departure_step < 0 || demand.departure_step >= T ||
        demand.vehicles <= kTol) {
      continue;
    }
    demand_eq_row[di] = static_cast<int>(active_demand_positions.size());
    active_demand_positions.push_back(di);
    result.total_demand_vehicles += std::max(0.0, demand.vehicles);
  }

  const int n_eq = static_cast<int>(active_demand_positions.size());
  if (n_eq == 0) {
    result.optimization_solved = true;
    result.optimization_is_mip = cert.is_mip;
    result.optimization_proven_optimal = true;
    result.optimization_backend = cert.is_mip ? "MILP-empty" : "LP-empty";
    result.optimization_status = "empty";
    return;
  }

  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  std::vector<SystemOptimalColumn> cols;
  std::vector<double> obj;
  std::vector<engine::VariableMeta> vars;
  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<double> beq(static_cast<std::size_t>(n_eq), 0.0);

  auto add_col = [&](SystemOptimalColumn col,
                     double lb,
                     double ub,
                     double c,
                     const std::string& name,
                     bool integer) {
    const int idx = static_cast<int>(cols.size());
    cols.push_back(col);
    obj.push_back(c);
    vars.push_back({integer ? engine::VarType::Integer : engine::VarType::Continuous,
                    lb,
                    ub,
                    name});
    if (integer) mip.integer_idx.push_back(idx);
    return idx;
  };

  const bool requested_mip = is_system_optimal_mip(options.assignment_model);
  bool all_integer_demands = true;
  for (int di : active_demand_positions) {
    const double vehicles = problem.demands[static_cast<std::size_t>(di)].vehicles;
    if (std::abs(vehicles - std::round(vehicles)) > 1e-7) {
      all_integer_demands = false;
      break;
    }
  }
  const bool use_integer_vars = requested_mip && all_integer_demands;
  if (requested_mip && !all_integer_demands) {
    add_warning(result,
                "SystemOptimalMILP requested with noninteger demand; solving LP relaxation");
    cert.is_mip = false;
  }

  for (int di : active_demand_positions) {
    const auto& demand = problem.demands[static_cast<std::size_t>(di)];
    const int eq = demand_eq_row.at(di);
    beq[static_cast<std::size_t>(eq)] = demand.vehicles;
    const auto routes = candidate_routes(problem, demand);
    for (const auto* route : routes) {
      if (!route_can_deliver_requested_energy(*route, options, dt)) continue;
      if (!route_soc_feasible(*route, demand, problem, link_pos)) continue;
      const double route_cost =
          route_free_flow_generalized_cost(problem,
                                           options,
                                           *route,
                                           demand.departure_step,
                                           link_pos);
      if (!std::isfinite(route_cost)) continue;
      const int arrival_step =
          route_arrival_step(*route, link_pos, problem, demand.departure_step, dt);
      bool charging_window_inside_horizon = true;
      for (const auto& stop : route->charging_stops) {
        const int departure_step =
            arrival_step + std::max(1, stop.dwell_steps);
        if (arrival_step < 0 || arrival_step >= T || departure_step > T) {
          charging_window_inside_horizon = false;
          break;
        }
      }
      if (!charging_window_inside_horizon) continue;
      SystemOptimalColumn col;
      col.kind = SystemOptimalColumn::Kind::RouteFlow;
      col.demand_pos = di;
      col.route = route;
      col.departure_step = demand.departure_step;
      col.arrival_step = arrival_step;
      const int x_idx =
          add_col(col,
                  0.0,
                  demand.vehicles,
                  route_cost,
                  "x_d" + std::to_string(demand.index) + "_r" +
                      std::to_string(route->index) + "_t" +
                      std::to_string(demand.departure_step),
                  use_integer_vars);
      eq_trips.emplace_back(eq, x_idx, 1.0);
    }

    SystemOptimalColumn u_col;
    u_col.kind = SystemOptimalColumn::Kind::UnservedDemand;
    u_col.demand_pos = di;
    u_col.departure_step = demand.departure_step;
    const int u_idx =
        add_col(u_col,
                0.0,
                demand.vehicles,
                std::max(0.0, options.system_optimal_unserved_trip_penalty),
                "u_d" + std::to_string(demand.index) + "_t" +
                    std::to_string(demand.departure_step),
                use_integer_vars);
    eq_trips.emplace_back(eq, u_idx, 1.0);
  }

  const int n = static_cast<int>(cols.size());
  lp.c = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < n; ++j) lp.c[j] = obj[static_cast<std::size_t>(j)];
  lp.vars = std::move(vars);

  lp.Aeq.resize(n_eq, n);
  lp.beq = Eigen::VectorXd::Zero(n_eq);
  for (int r = 0; r < n_eq; ++r) lp.beq[r] = beq[static_cast<std::size_t>(r)];
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();

  std::map<std::pair<int, int>, int> road_rows;
  std::map<std::pair<int, int>, int> station_rows;
  std::map<int, int> gen_rows;
  std::vector<Eigen::Triplet<double>> ineq_trips;
  std::vector<double> b_vals;

  auto add_ineq_row = [&](double rhs) {
    const int row = static_cast<int>(b_vals.size());
    b_vals.push_back(rhs);
    return row;
  };

  auto road_row = [&](int step, int link_index, double rhs) {
    const auto key = std::make_pair(step, link_index);
    auto [it, inserted] = road_rows.emplace(key, -1);
    if (inserted) it->second = add_ineq_row(rhs);
    return it->second;
  };

  auto station_row = [&](int step, int station_id, double rhs) {
    const auto key = std::make_pair(step, station_id);
    auto [it, inserted] = station_rows.emplace(key, -1);
    if (inserted) it->second = add_ineq_row(rhs);
    return it->second;
  };

  auto gen_row = [&](int step, double rhs) {
    auto [it, inserted] = gen_rows.emplace(step, -1);
    if (inserted) it->second = add_ineq_row(rhs);
    return it->second;
  };

  const double base_load_mw = total_base_load_mw(problem.system);
  const double gen_cap_mw = total_generation_capacity_mw(problem.system, options);
  const double gen_ev_cap_kw =
      std::max(0.0, (gen_cap_mw - base_load_mw) * 1000.0);

  for (int j = 0; j < n; ++j) {
    const auto& col = cols[static_cast<std::size_t>(j)];
    if (col.kind != SystemOptimalColumn::Kind::RouteFlow || col.route == nullptr) {
      continue;
    }
    if (options.enforce_road_capacity) {
      for (int link_index : col.route->link_indices) {
        auto pos_it = link_pos.find(link_index);
        if (pos_it == link_pos.end()) continue;
        const double cap =
            link_capacity[static_cast<std::size_t>(col.departure_step)][pos_it->second];
        const int row = road_row(col.departure_step, link_index, cap);
        ineq_trips.emplace_back(row, j, 1.0);
      }
    }
    for (const auto& stop : col.route->charging_stops) {
      const double p_per_vehicle =
          full_energy_power_kw_per_vehicle(stop, options, dt);
      if (p_per_vehicle <= kTol) continue;
      const double station_cap =
          charge_cap_kw.count(stop.station_id)
              ? charge_cap_kw.at(stop.station_id)
              : options.default_station_power_kw;
      for (int offset = 0; offset < std::max(1, stop.dwell_steps); ++offset) {
        const int k = col.arrival_step + offset;
        if (k < 0 || k >= T) continue;
        const int sr = station_row(k, stop.station_id, station_cap);
        ineq_trips.emplace_back(sr, j, p_per_vehicle);
        if (options.enforce_generation_capacity) {
          const int gr = gen_row(k, gen_ev_cap_kw);
          ineq_trips.emplace_back(gr, j, p_per_vehicle);
        }
      }
    }
  }

  lp.A.resize(static_cast<int>(b_vals.size()), n);
  lp.b = Eigen::VectorXd::Zero(static_cast<int>(b_vals.size()));
  for (int r = 0; r < static_cast<int>(b_vals.size()); ++r) {
    lp.b[r] = b_vals[static_cast<std::size_t>(r)];
  }
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();

  cert.cols = cols;
  if (!cert.is_mip || (mip.integer_idx.empty() && mip.binary_idx.empty())) {
    engine::SimplexOptions simp_opt;
    simp_opt.max_iter = 10000;
    simp_opt.feasibility_tol = 1e-8;
    simp_opt.optimality_tol = 1e-8;
    simp_opt.verbose = false;
    auto lp_res = engine::solve_lp_with_basis(lp, simp_opt, nullptr);
    cert.x = lp_res.result.x;
    cert.solved = lp_res.result.stats.success;
    cert.backend = "NativeDualSimplex";
    cert.status = lp_res.result.stats.status;
    cert.objective = lp_res.result.stats.objective;
    cert.best_bound = cert.objective;
    cert.mip_gap = 0.0;
    cert.proven_optimal = cert.solved;
    cert.is_mip = false;
  } else {
    auto solve_with_native_bc = [&](const std::string& previous_failure) {
      engine::BCOptions bc_opt;
      bc_opt.max_nodes = std::max(1, options.system_optimal_max_nodes);
      bc_opt.time_limit_sec = std::max(0.0, options.system_optimal_time_limit_sec);
      bc_opt.gap_tol = std::max(0.0, options.system_optimal_mip_gap);
      bc_opt.verbose = false;
      bc_opt.use_simplex_lp_nodes = true;
      auto bc_res = engine::solve_milp_bc(mip, bc_opt);
      cert.x = bc_res.x;
      cert.solved = bc_res.stats.success;
      cert.backend = previous_failure.empty() ? "NativeB&C" : "HiGHS+NativeB&C";
      cert.status = previous_failure.empty()
          ? bc_res.stats.status
          : "HiGHS failed: " + previous_failure + "; NativeB&C: " +
                bc_res.stats.status;
      cert.objective = bc_res.stats.objective;
      cert.best_bound = bc_res.bc_stats.best_bound;
      cert.mip_gap = std::isfinite(bc_res.bc_stats.gap)
                         ? bc_res.bc_stats.gap
                         : bc_res.stats.mip_gap;
      cert.proven_optimal =
          cert.solved &&
          std::isfinite(cert.mip_gap) &&
          cert.mip_gap <= options.system_optimal_mip_gap + 1e-9;
    };

    engine::BCOptions highs_options;
    highs_options.max_nodes = std::max(1, options.system_optimal_max_nodes);
    highs_options.time_limit_sec =
        std::max(0.0, options.system_optimal_time_limit_sec);
    highs_options.gap_tol = std::max(0.0, options.system_optimal_mip_gap);
    engine::StrictHighsBranchAndCutAdapter highs(highs_options);
    {
      auto highs_res = highs.solve_milp(mip);
      cert.x = highs_res.x;
      cert.solved = highs_res.stats.success;
      cert.backend = "HiGHS";
      cert.status = highs_res.stats.status;
      cert.objective = highs_res.stats.objective;
      cert.mip_gap = highs_res.stats.mip_gap;
      cert.best_bound = inferred_minimization_bound(cert.objective, cert.mip_gap);
      cert.proven_optimal =
          cert.solved &&
          std::isfinite(cert.mip_gap) &&
          cert.mip_gap <= options.system_optimal_mip_gap + 1e-9;
      if (!cert.solved || cert.x.size() != n) {
        std::string reason =
            cert.status.empty() ? std::string("unsuccessful solve") : cert.status;
        if (cert.solved && cert.x.size() != n) {
          reason += " (solution vector has wrong size)";
        }
        solve_with_native_bc(reason);
      }
    }
  }

  result.optimization_solved = cert.solved;
  result.optimization_is_mip = cert.is_mip;
  result.optimization_proven_optimal = cert.solved && cert.proven_optimal;
  result.optimization_backend = cert.backend;
  result.optimization_status = cert.status;
  result.optimization_objective = cert.objective;
  result.optimization_best_bound = cert.best_bound;
  result.optimization_mip_gap = cert.mip_gap;

  if (!cert.solved || cert.x.size() != n) {
    result.status = cert.status.empty() ? "system-optimal solve failed" : cert.status;
    add_warning(result, "system-optimal LP/MILP solve failed");
    for (int di : active_demand_positions) {
      const auto& demand = problem.demands[static_cast<std::size_t>(di)];
      result.unmet_demand_vehicles += demand.vehicles;
      result.assignments.push_back({demand.index,
                                    0,
                                    demand.departure_step,
                                    demand.vehicles,
                                    0.0,
                                    0.0,
                                    false,
                                    "system-optimal solve failed"});
    }
    return;
  }

  std::vector<std::pair<int, double>> positive_route_cols;
  std::vector<std::pair<int, double>> positive_unserved_cols;
  for (int j = 0; j < n; ++j) {
    double value = cert.x[j];
    if (!std::isfinite(value) || value <= 1e-7) continue;
    if (cert.is_mip) value = std::round(value);
    const auto& col = cert.cols[static_cast<std::size_t>(j)];
    if (col.kind == SystemOptimalColumn::Kind::RouteFlow) {
      positive_route_cols.emplace_back(j, value);
      if (col.route != nullptr) {
        for (int link_index : col.route->link_indices) {
          if (auto pos_it = link_pos.find(link_index); pos_it != link_pos.end()) {
            link_used[static_cast<std::size_t>(col.departure_step)][pos_it->second] +=
                value;
          }
        }
      }
    } else {
      positive_unserved_cols.emplace_back(j, value);
    }
  }

  for (const auto& [j, vehicles] : positive_route_cols) {
    const auto& col = cert.cols[static_cast<std::size_t>(j)];
    if (col.route == nullptr) continue;
    const auto& demand = problem.demands[static_cast<std::size_t>(col.demand_pos)];
    const double route_cost =
        route_generalized_cost(problem,
                               options,
                               *col.route,
                               col.departure_step,
                               link_pos,
                               link_used[static_cast<std::size_t>(col.departure_step)],
                               link_capacity[static_cast<std::size_t>(col.departure_step)]);
    const double route_travel_time =
        route_travel_time_hr(problem,
                             *col.route,
                             link_pos,
                             link_used[static_cast<std::size_t>(col.departure_step)],
                             link_capacity[static_cast<std::size_t>(col.departure_step)]);
    result.assignments.push_back({demand.index,
                                  col.route->index,
                                  col.departure_step,
                                  vehicles,
                                  route_cost,
                                  route_travel_time,
                                  true,
                                  cert.is_mip ? "served-system-optimal-milp"
                                              : "served-system-optimal-lp"});
    result.served_demand_vehicles += vehicles;
    if (std::isfinite(route_travel_time)) {
      result.total_travel_time_hr += vehicles * route_travel_time;
    }
    if (std::isfinite(route_cost)) {
      result.total_assignment_cost += vehicles * route_cost;
    }

    for (const auto& stop : col.route->charging_stops) {
      if (col.arrival_step >= T) continue;
      EVChargingSession session;
      session.index = next_session_index++;
      session.station_id = stop.station_id;
      session.arrival_step = col.arrival_step;
      session.departure_step =
          std::min(T, col.arrival_step + std::max(1, stop.dwell_steps));
      session.vehicle_count = vehicles;
      const double e_req =
          std::max(0.0, stop.requested_energy_kwh_per_vehicle * vehicles);
      session.energy_initial_kwh = 0.0;
      session.energy_target_kwh = e_req;
      session.energy_min_kwh = 0.0;
      const double p_full = full_energy_power_kw_per_vehicle(stop, options, dt);
      session.max_charge_kw =
          std::isfinite(p_full)
              ? std::max(0.0, p_full * vehicles)
              : charging_power_limit(stop, options, vehicles);
      session.max_discharge_kw = discharge_power_limit(stop, vehicles);
      session.energy_max_kwh =
          std::max(e_req, e_req + session.max_discharge_kw * dt);
      session.eta_charge = options.default_charging_efficiency;
      session.eta_discharge = options.default_charging_efficiency;
      session.v2g_capable = stop.v2g_capable;
      session.source_demand_index = demand.index;
      session.source_route_index = col.route->index;
      sessions.push_back(session);
      result.total_requested_energy_kwh += e_req;
    }
  }

  for (const auto& [j, vehicles] : positive_unserved_cols) {
    const auto& col = cert.cols[static_cast<std::size_t>(j)];
    const auto& demand = problem.demands[static_cast<std::size_t>(col.demand_pos)];
    result.unmet_demand_vehicles += vehicles;
    result.assignments.push_back({demand.index,
                                  0,
                                  demand.departure_step,
                                  vehicles,
                                  0.0,
                                  0.0,
                                  false,
                                  "system-optimal unmet demand"});
  }
}

}  // namespace hacdcpf::evpt
