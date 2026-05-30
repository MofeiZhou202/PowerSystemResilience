// ctm_so_lp.cpp
// ──────────────────────────────────────────────────────────────────────────
// System-Optimal CTM LP (Formulation E) - Exact Node & Route Continuity
// ──────────────────────────────────────────────────────────────────────────
#include "evpt_internal.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>

namespace hacdcpf::evpt {

CTMSOLPResult solve_ctm_so_lp(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_opts,
    const CTMOptions& ctm_opts)
{
  if (ev_opts.auto_generate_routes) {
    EVPowerTrafficOptions local_opts = ev_opts;
    local_opts.auto_generate_routes = false;
    const EVPowerTrafficProblem generated =
        maybe_generate_candidate_routes(problem, ev_opts);
    return solve_ctm_so_lp(generated, local_opts, ctm_opts);
  }

  CTMSOLPResult result;
  result.solved = false;

  if (ev_opts.require_exact_mathematical_model) {
    result.infeasible = true;
    result.status =
        "unsupported: exact full EV power-traffic mathematical-model "
        "verification is not provided by SO-CTM LP";
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: SO-CTM LP certifies only the assembled traffic LP, not "
        "charging/V2G, DC-OPF, and equilibrium in one full model";
    return result;
  }

  const double dt_sim = ev_opts.time_step_hr;
  const int T_sim = ev_opts.num_steps;
  if (T_sim <= 0 || dt_sim <= kTol) {
    result.infeasible = true;
    result.status = "empty";
    return result;
  }

  const double dt_ctm = choose_ctm_dt(problem, ctm_opts, dt_sim);
  const int R = std::max(1, static_cast<int>(std::round(dt_sim / dt_ctm)));
  const int T_ctm = T_sim * R;

  std::unordered_map<int, std::size_t> link_pos;
  const int n_links = static_cast<int>(problem.traffic.links.size());
  for (int li = 0; li < n_links; ++li) {
    link_pos[problem.traffic.links[li].index] = li;
  }

  struct LinkCTM {
    int M;
    double vf_step;
    double w_step;
    double delta;
    double qmax_step;
    double njam;
  };
  std::vector<LinkCTM> lcp(static_cast<std::size_t>(n_links));
  for (int li = 0; li < n_links; ++li) {
    const auto& lnk = problem.traffic.links[li];
    const double len_km = std::max(kTol, lnk.length_km);
    const double vf_km_hr = link_free_flow_speed(lnk);
    // CFL: cell length δ = len/M must satisfy δ ≥ v_f·dt_ctm
    //  ⟹  M ≤ ⌊ len / (v_f · dt_ctm) ⌋
    // Clamp user-supplied n_cells_per_link down to the CFL ceiling so that
    // vehicles never propagate more than one cell per time step.
    const int M_user = std::max(1, ctm_opts.n_cells_per_link);
    const int M_cfl_max = (vf_km_hr > kTol && dt_ctm > kTol)
        ? std::max(1, static_cast<int>(std::floor(len_km / (vf_km_hr * dt_ctm))))
        : M_user;
    const int M = std::min(M_user, M_cfl_max);
    const double delta = len_km / M;
    const double vf_step = vf_km_hr * dt_ctm;
    const double qmax_hr = lnk.capacity_veh_per_hr > kTol ? lnk.capacity_veh_per_hr : vf_km_hr * 0.1;
    const double qmax_step = qmax_hr * dt_ctm;
    const double w_hr = [&]() {
      if (lnk.length_km > kTol) {
        double kj = lnk.jam_vehicles > kTol ? lnk.jam_vehicles / lnk.length_km : qmax_hr / vf_km_hr * 2.0;
        double kc = qmax_hr / vf_km_hr;
        if (kj > kc + kTol) return qmax_hr / (kj - kc);
      }
      double fb = ctm_opts.backward_wave_speed_fallback_km_hr;
      return fb > kTol ? fb : vf_km_hr * 0.25;
    }();
    const double w_step = std::max(kTol, w_hr) * dt_ctm;
    const double kj = lnk.jam_vehicles > kTol
        ? lnk.jam_vehicles / len_km
        : qmax_hr / vf_km_hr + qmax_hr / std::max(kTol, w_hr);
    const double njam = std::max(1.0, kj * delta);
    lcp[li] = {M, vf_step, w_step, delta, qmax_step, njam};
  }

  struct RouteDem {
    int d_idx;
    int r_idx;
    double vehicles;
    int k_dep;
    std::vector<int> path; 
  };
	  std::vector<RouteDem> active_routes;
	  const int n_demands = static_cast<int>(problem.demands.size());
	  double total_requested_vehicles = 0.0;
	  for (int d = 0; d < n_demands; ++d) {
	    const auto& demand = problem.demands[d];
	    if (demand.vehicles <= kTol) continue;
	    total_requested_vehicles += demand.vehicles;
	    int k_dep = demand.departure_step * R;
    if (k_dep < 0 || k_dep >= T_ctm) continue;
    for (const auto* r : candidate_routes(problem, demand)) {
      if (!route_soc_feasible(*r, demand, problem, link_pos)) continue;
      std::vector<int> path;
      for (int link_id : r->link_indices) {
        auto it = link_pos.find(link_id);
        if (it != link_pos.end()) path.push_back(static_cast<int>(it->second));
      }
	      if (!path.empty()) {
	        active_routes.push_back({demand.index, r->index, demand.vehicles, k_dep, path});
	      }
	    }
	  }
	  const double state_flow_ub = std::max(1.0, total_requested_vehicles);

  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  std::vector<engine::VariableMeta> vars;
  std::vector<double> obj_vec;
  std::vector<double> beq_vec;
  std::vector<double> bineq_vec;
  std::vector<Eigen::Triplet<double>> eq_trips;
  std::vector<Eigen::Triplet<double>> ineq_trips;

  auto add_var = [&](double lb, double ub, double cost, const std::string& name) -> int {
    const int idx = static_cast<int>(vars.size());
    vars.push_back({engine::VarType::Continuous, lb, ub, name});
    obj_vec.push_back(cost);
    return idx;
  };
  auto add_eq = [&](double rhs) -> int {
    const int row = static_cast<int>(beq_vec.size());
    beq_vec.push_back(rhs);
    return row;
  };
  auto add_ineq = [&](double rhs) -> int {
    const int row = static_cast<int>(bineq_vec.size());
    bineq_vec.push_back(rhs);
    return row;
  };

  std::unordered_map<int, std::unordered_map<int, int>> h_idx;
  for (const auto& ar : active_routes) {
    if (h_idx[ar.d_idx].count(ar.r_idx) == 0) {
      h_idx[ar.d_idx][ar.r_idx] = add_var(0.0, ar.vehicles, 0.0, 
        "h_d" + std::to_string(ar.d_idx) + "_r" + std::to_string(ar.r_idx));
    }
  }

  // ── Aggregate mode: per-link cell state variables ──────────────────────
  // When use_aggregate_link_variables = true, use ONE n_agg[li][m][k] per
  // link-cell-step for aggregate internal CTM dynamics instead of per-route
  // cell states. Per-route entry y[ari][pi][0][k] and exit y[ari][pi][M][k]
  // variables are still created for OD/node accounting; only the M-1 internal
  // boundary flow variables y[ari][pi][1..M-1][k] are replaced by shared
  // y_agg_int[li][m][k] (m=1..M-1).
  const bool use_agg = ctm_opts.use_aggregate_link_variables;

  // n_agg_idx[li][m][k] — aggregate cell occupancy (aggregate mode only)
  std::vector<std::vector<std::vector<int>>> n_agg_idx;
  // y_agg_int_idx[li][m][k] — aggregate internal boundary flow (m=1..M-1)
  std::vector<std::vector<std::vector<int>>> y_agg_int_idx;

  if (use_agg) {
    n_agg_idx.resize(static_cast<std::size_t>(n_links));
    y_agg_int_idx.resize(static_cast<std::size_t>(n_links));
    for (int li = 0; li < n_links; ++li) {
      int M = lcp[li].M;
      n_agg_idx[li].assign(M, std::vector<int>(T_ctm + 1, -1));
      y_agg_int_idx[li].assign(M + 1, std::vector<int>(T_ctm, -1)); // m=1..M-1 used
      for (int m = 0; m < M; ++m)
        for (int k = 0; k <= T_ctm; ++k)
          n_agg_idx[li][m][k] = add_var(0.0, state_flow_ub, dt_ctm,
            "n_a" + std::to_string(li) + "_m" + std::to_string(m) + "_k" + std::to_string(k));
      for (int m = 1; m < M; ++m) // internal boundaries only
        for (int k = 0; k < T_ctm; ++k)
          y_agg_int_idx[li][m][k] = add_var(0.0, state_flow_ub, 0.0,
            "yint_a" + std::to_string(li) + "_m" + std::to_string(m) + "_k" + std::to_string(k));
    }
  }

  std::vector<std::vector<std::vector<std::vector<int>>>> n_idx(active_routes.size());
  std::vector<std::vector<std::vector<std::vector<int>>>> y_idx(active_routes.size());
  std::vector<std::vector<std::vector<int>>> node_y_idx(active_routes.size());
  
  for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
    const auto& ar = active_routes[ari];
    n_idx[ari].resize(ar.path.size());
    y_idx[ari].resize(ar.path.size());
    node_y_idx[ari].resize(ar.path.size() > 1 ? ar.path.size() - 1 : 0);
    for (std::size_t pi = 0; pi < ar.path.size(); ++pi) {
      int li = ar.path[pi];
      int M = lcp[li].M;
      n_idx[ari][pi].assign(M, std::vector<int>(T_ctm + 1, -1));
      y_idx[ari][pi].assign(M + 1, std::vector<int>(T_ctm, -1)); // m=0 is entry, ..., m=M is exit
      
      if (!use_agg) {
        for (int m = 0; m < M; ++m) {
          for (int k = 0; k <= T_ctm; ++k) {
            n_idx[ari][pi][m][k] = add_var(0.0, state_flow_ub, dt_ctm,
              "n_r" + std::to_string(ari) + "_p" + std::to_string(pi) + "_m" + std::to_string(m) + "_k" + std::to_string(k));
          }
        }
      }
      // In aggregate mode, n_idx stays -1 (unused); n_agg_idx is used instead.

      for (int m = 0; m <= M; ++m) {
        for (int k = 0; k < T_ctm; ++k) {
          if (use_agg && m > 0 && m < M) continue; // internal flows replaced by y_agg_int
          y_idx[ari][pi][m][k] = add_var(0.0, state_flow_ub, 0.0,
            "y_r" + std::to_string(ari) + "_p" + std::to_string(pi) + "_m" + std::to_string(m) + "_k" + std::to_string(k));
        }
      }
      if (pi + 1 < ar.path.size()) {
        node_y_idx[ari][pi].assign(T_ctm, -1);
        for (int k = 0; k < T_ctm; ++k) {
          node_y_idx[ari][pi][k] = add_var(0.0, state_flow_ub, 0.0,
            "ynode_r" + std::to_string(ari) + "_p" + std::to_string(pi) + "_k" + std::to_string(k));
        }
      }
    }
  }

  std::unordered_map<int, std::vector<std::size_t>> demand_to_ari;
  for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
    demand_to_ari[active_routes[ari].d_idx].push_back(ari);
  }
  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    if (demand_to_ari.count(demand.index)) {
      int row = add_eq(demand.vehicles);
      for (std::size_t ari : demand_to_ari[demand.index]) {
        eq_trips.emplace_back(row, h_idx[demand.index][active_routes[ari].r_idx], 1.0);
      }
      if (ev_opts.allow_unserved_travel_demand) {
        double pen = ev_opts.value_of_time_per_hr * T_sim;
        int u_var = add_var(0.0, demand.vehicles, pen, "u_d" + std::to_string(demand.index));
        eq_trips.emplace_back(row, u_var, 1.0);
      }
    }
  }

  for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
    const auto& ar = active_routes[ari];
    int h_var = h_idx[ar.d_idx][ar.r_idx];

    // ── Origin injection constraints ─────────────────────────────────────
    // Flexible injection: vehicles cannot enter before k_dep, but the LP is
    // free to spread them across steps k >= k_dep (subject to shared
    // sending/receiving capacity).  The total injected equals h.
    // This avoids infeasibility when h > qmax_step for large demands.
    //
    //   y[ari][0][0][k] = 0          for k  < k_dep
    //   Σ_{k >= k_dep} y[ari][0][0][k] = h   (total injection = demand)
    for (int k = 0; k < ar.k_dep; ++k) {
      int row = add_eq(0.0);
      eq_trips.emplace_back(row, y_idx[ari][0][0][k], 1.0);
    }
    {
      int row = add_eq(0.0);
      for (int k = ar.k_dep; k < T_ctm; ++k)
        eq_trips.emplace_back(row, y_idx[ari][0][0][k], 1.0);
      eq_trips.emplace_back(row, h_var, -1.0);
    }

    for (std::size_t pi = 0; pi < ar.path.size(); ++pi) {
      int li = ar.path[pi];
      int M = lcp[li].M;

      if (use_agg) {
        // Aggregate mode: mass balance on n_agg[li][m][k].
        // Entry flow y[ari][pi][0][k] and exit flow y[ari][pi][M][k] are
        // per-route; internal flows are shared y_agg_int[li][m][k].
        // IC: n_agg[li][m][0] = 0  (shared, added once per li — see below)
        for (int m = 0; m < M; ++m) {
          for (int k = 0; k < T_ctm; ++k) {
            // n_agg[m][k+1] = n_agg[m][k] + y_in[m][k] - y_out[m+1][k]
            // y_in[m][k]:  m=0 → per-route entry y[ari][pi][0][k], else y_agg_int[li][m][k]
            // y_out[m+1][k]: m=M-1 → per-route exit y[ari][pi][M][k], else y_agg_int[li][m+1][k]
            // We only ADD the per-route entry/exit contribution here; the aggregate
            // balance rows for this (li,m,k) are built once in the aggregate block below.
            // Nothing needed per-ari in aggregate mode for mass balance rows.
            (void)m; (void)k; // silence unused-variable in loop-less path
            break; // skip inner — aggregate mass balance built per-link below
          }
          break; // only add once per li
        }
      } else {
        for (int m = 0; m < M; ++m) {
          int row_ic = add_eq(0.0);
          eq_trips.emplace_back(row_ic, n_idx[ari][pi][m][0], 1.0);
          
          for (int k = 0; k < T_ctm; ++k) {
            int row = add_eq(0.0);
            eq_trips.emplace_back(row, n_idx[ari][pi][m][k+1], 1.0);
            eq_trips.emplace_back(row, n_idx[ari][pi][m][k], -1.0);
            eq_trips.emplace_back(row, y_idx[ari][pi][m][k], -1.0);
            eq_trips.emplace_back(row, y_idx[ari][pi][m+1][k], 1.0);
          }
        }
      }
      
      if (pi + 1 < ar.path.size()) {
        for (int k = 0; k < T_ctm; ++k) {
          const int y_node = node_y_idx[ari][pi][k];
          int row_out = add_eq(0.0);
          eq_trips.emplace_back(row_out, y_idx[ari][pi][M][k], 1.0);
          eq_trips.emplace_back(row_out, y_node, -1.0);
          int row_in = add_eq(0.0);
          eq_trips.emplace_back(row_in, y_idx[ari][pi+1][0][k], 1.0);
          eq_trips.emplace_back(row_in, y_node, -1.0);
        }
      } else {
        for (int k = 0; k < T_ctm; ++k) {
          // Flow can exit the last link freely
        }
      }
    }
  }

  // ── Aggregate mode: per-link mass balance constraints ─────────────────
  // For each link li and each cell m, add:
  //   n_agg[li][m][0] = 0   (IC)
  //   n_agg[li][m][k+1] - n_agg[li][m][k] + y_out[m+1][k] - y_in[m][k] = 0
  // where y_in[m=0][k]  = Σ_{ari: li ∈ path[pi][0]} y[ari][pi][0][k]  (all route entries)
  //       y_out[m=M][k] = Σ_{ari: li ∈ path[pi][M]} y[ari][pi][M][k]  (all route exits)
  //       y_in/out for internal boundaries = y_agg_int[li][m][k]
  if (use_agg) {
    // Track which (ari,pi) tuples correspond to each link
    std::vector<std::vector<std::pair<std::size_t,std::size_t>>> link_route_pi(
        static_cast<std::size_t>(n_links));
    for (std::size_t ari = 0; ari < active_routes.size(); ++ari)
      for (std::size_t pi = 0; pi < active_routes[ari].path.size(); ++pi)
        link_route_pi[active_routes[ari].path[pi]].emplace_back(ari, pi);

    for (int li = 0; li < n_links; ++li) {
      int M = lcp[li].M;
      const auto& rp = link_route_pi[li];
      for (int m = 0; m < M; ++m) {
        // IC
        {
          int row_ic = add_eq(0.0);
          eq_trips.emplace_back(row_ic, n_agg_idx[li][m][0], 1.0);
        }
        for (int k = 0; k < T_ctm; ++k) {
          int row = add_eq(0.0);
          eq_trips.emplace_back(row, n_agg_idx[li][m][k+1], 1.0);
          eq_trips.emplace_back(row, n_agg_idx[li][m][k], -1.0);
          // y_in at boundary m (flow entering cell m from left)
          if (m == 0) {
            for (auto [ari2, pi2] : rp)
              if (y_idx[ari2][pi2][0][k] >= 0)
                eq_trips.emplace_back(row, y_idx[ari2][pi2][0][k], -1.0);
          } else {
            eq_trips.emplace_back(row, y_agg_int_idx[li][m][k], -1.0);
          }
          // y_out at boundary m+1 (flow leaving cell m to the right)
          if (m + 1 == M) {
            for (auto [ari2, pi2] : rp)
              if (y_idx[ari2][pi2][M][k] >= 0)
                eq_trips.emplace_back(row, y_idx[ari2][pi2][M][k], 1.0);
          } else {
            eq_trips.emplace_back(row, y_agg_int_idx[li][m+1][k], 1.0);
          }
        }
      }
    }
  }

  for (int li = 0; li < n_links; ++li) {
    const auto& lnk = problem.traffic.links[static_cast<std::size_t>(li)];
    const auto& p = lcp[li];
    double vf_rate = p.vf_step / p.delta;
    double w_rate = p.w_step / p.delta;

    for (int k = 0; k < T_ctm; ++k) {
      // ── Availability: if link is closed at this step, zero out all flows ──
      // availability_profile is indexed by simulation step (not CTM sub-step);
      // map CTM step k back to the corresponding simulation step k_sim = k/R.
      const int k_sim = k / R;
      bool avail = lnk.available;
      if (!lnk.availability_profile.empty() &&
          k_sim < static_cast<int>(lnk.availability_profile.size()))
        avail = lnk.availability_profile[static_cast<std::size_t>(k_sim)];
      if (!avail) {
        // Force all route flows on this link at this CTM step to zero.
        for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
          for (std::size_t pi = 0; pi < active_routes[ari].path.size(); ++pi) {
            if (active_routes[ari].path[pi] == li) {
              for (int m = 0; m <= p.M; ++m) {
                const int yv = y_idx[ari][pi][m][k];
                if (yv < 0) continue;  // aggregate mode: internal flows live in y_agg_int
                int row = add_eq(0.0);
                eq_trips.emplace_back(row, yv, 1.0);
              }
            }
          }
        }
        // In aggregate mode also zero out the shared internal boundary flows
        // on this closed link/step so cells cannot fill via the aggregate path.
        if (use_agg) {
          for (int m = 1; m < p.M; ++m) {
            const int yv = y_agg_int_idx[li][m][k];
            if (yv < 0) continue;
            int row = add_eq(0.0);
            eq_trips.emplace_back(row, yv, 1.0);
          }
        }
        continue;
      }

      for (int m = 0; m < p.M; ++m) {
        // Sending: sum_y <= sum_n * vf_rate and sum_y <= qmax
        int row_s1 = add_ineq(0.0);
        int row_s2 = add_ineq(p.qmax_step);

        if (use_agg) {
          // Aggregate: one n_agg variable per cell; flow is y_agg_int or per-route exit
          const int y_out_var = (m + 1 == p.M)
              ? -1  // per-route exits handled below
              : y_agg_int_idx[li][m+1][k];
          if (y_out_var >= 0) {
            ineq_trips.emplace_back(row_s1, y_out_var, 1.0);
            ineq_trips.emplace_back(row_s2, y_out_var, 1.0);
          } else {
            // Last cell: sum all per-route exit flows y[ari][pi][M][k] for this link
            for (std::size_t ari2 = 0; ari2 < active_routes.size(); ++ari2)
              for (std::size_t pi2 = 0; pi2 < active_routes[ari2].path.size(); ++pi2)
                if (active_routes[ari2].path[pi2] == li && y_idx[ari2][pi2][p.M][k] >= 0) {
                  ineq_trips.emplace_back(row_s1, y_idx[ari2][pi2][p.M][k], 1.0);
                  ineq_trips.emplace_back(row_s2, y_idx[ari2][pi2][p.M][k], 1.0);
                }
          }
          ineq_trips.emplace_back(row_s1, n_agg_idx[li][m][k], -vf_rate);
        } else {
          for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
            for (std::size_t pi = 0; pi < active_routes[ari].path.size(); ++pi) {
              if (active_routes[ari].path[pi] == li) {
                ineq_trips.emplace_back(row_s1, y_idx[ari][pi][m+1][k], 1.0);
                ineq_trips.emplace_back(row_s1, n_idx[ari][pi][m][k], -vf_rate);
                ineq_trips.emplace_back(row_s2, y_idx[ari][pi][m+1][k], 1.0);
              }
            }
          }
        }
        
        // Receiving: sum_y <= w_rate * (Njam - sum_n_{m+1})
        if (m + 1 < p.M) {
          int row_r = add_ineq(p.njam * w_rate);
          if (use_agg) {
            ineq_trips.emplace_back(row_r, y_agg_int_idx[li][m+1][k], 1.0);
            ineq_trips.emplace_back(row_r, n_agg_idx[li][m+1][k], w_rate);
          } else {
            for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
              for (std::size_t pi = 0; pi < active_routes[ari].path.size(); ++pi) {
                if (active_routes[ari].path[pi] == li) {
                  ineq_trips.emplace_back(row_r, y_idx[ari][pi][m+1][k], 1.0);
                  ineq_trips.emplace_back(row_r, n_idx[ari][pi][m+1][k], w_rate);
                }
              }
            }
          }
        }
      }
    }
  }

  // ── Explicit node movement constraints y_{a,b,k} ──────────────────────
  // For each physical movement a -> b and time step k, aggregate all
  // route-specific node variables sharing the same movement. Sending is
  // already enforced by the upstream last-cell y variable; here we enforce
  // downstream first-cell receiving and capacity on the merged movement.
  std::map<std::tuple<int, int, int>, std::vector<int>> movement_vars;
  std::map<std::pair<int, int>, int> movement_downstream_link;
  for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
    const auto& ar = active_routes[ari];
    for (std::size_t pi = 0; pi + 1 < ar.path.size(); ++pi) {
      const int up = ar.path[pi];
      const int dn = ar.path[pi + 1];
      movement_downstream_link[{up, dn}] = dn;
      for (int k = 0; k < T_ctm; ++k) {
        movement_vars[{up, dn, k}].push_back(node_y_idx[ari][pi][k]);
      }
    }
  }
  for (const auto& kv : movement_vars) {
    const auto [up, dn, k] = kv.first;
    (void)up;
    const auto& p_dn = lcp[dn];
    const double w_rate_dn = p_dn.w_step / p_dn.delta;
    int row_recv = add_ineq(p_dn.njam * w_rate_dn);
    int row_cap = add_ineq(p_dn.qmax_step);
    for (int yv : kv.second) {
      ineq_trips.emplace_back(row_recv, yv, 1.0);
      ineq_trips.emplace_back(row_cap, yv, 1.0);
    }
    if (use_agg) {
      ineq_trips.emplace_back(row_recv, n_agg_idx[dn][0][k], w_rate_dn);
    } else {
      for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
        for (std::size_t pi = 0; pi < active_routes[ari].path.size(); ++pi) {
          if (active_routes[ari].path[pi] == dn) {
            ineq_trips.emplace_back(row_recv, n_idx[ari][pi][0][k], w_rate_dn);
          }
        }
      }
    }
  }

  // Origin injection into each entry link shares the first-cell receiving
  // capacity with all routes injected at that link and step.
  for (int li = 0; li < n_links; ++li) {
    const auto& p = lcp[li];
    const double w_rate = p.w_step / p.delta;
    for (int k = 0; k < T_ctm; ++k) {
      int row_recv = add_ineq(p.njam * w_rate);
      int row_cap = add_ineq(p.qmax_step);
      bool any_entry = false;
      for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
        if (active_routes[ari].path.empty() || active_routes[ari].path[0] != li) continue;
        any_entry = true;
        ineq_trips.emplace_back(row_recv, y_idx[ari][0][0][k], 1.0);
        ineq_trips.emplace_back(row_cap, y_idx[ari][0][0][k], 1.0);
        if (!use_agg)
          ineq_trips.emplace_back(row_recv, n_idx[ari][0][0][k], w_rate);
      }
      if (use_agg && any_entry)
        ineq_trips.emplace_back(row_recv, n_agg_idx[li][0][k], w_rate);
      if (!any_entry) {
        bineq_vec[row_recv] = 1.0e20;
        bineq_vec[row_cap] = 1.0e20;
      }
    }
  }

  // Aggregate merge/origin receiving by downstream link.  This is the
  // node-level constraint that prevents two upstream movements plus origin
  // injection from independently consuming the same first-cell receiving
  // capacity of link b at step k.
  for (int li = 0; li < n_links; ++li) {
    const auto& p = lcp[li];
    const double w_rate = p.w_step / p.delta;
    for (int k = 0; k < T_ctm; ++k) {
      int row_recv = add_ineq(p.njam * w_rate);
      int row_cap = add_ineq(p.qmax_step);
      bool any_entry = false;

      for (const auto& kv : movement_vars) {
        const auto [up, dn, kk] = kv.first;
        (void)up;
        if (dn != li || kk != k) continue;
        any_entry = true;
        for (int yv : kv.second) {
          ineq_trips.emplace_back(row_recv, yv, 1.0);
          ineq_trips.emplace_back(row_cap, yv, 1.0);
        }
      }
      for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
        for (std::size_t pi = 0; pi < active_routes[ari].path.size(); ++pi) {
          if (active_routes[ari].path[pi] == li) {
            if (!use_agg)
              ineq_trips.emplace_back(row_recv, n_idx[ari][pi][0][k], w_rate);
          }
        }
        if (!active_routes[ari].path.empty() &&
            active_routes[ari].path[0] == li) {
          any_entry = true;
          ineq_trips.emplace_back(row_recv, y_idx[ari][0][0][k], 1.0);
          ineq_trips.emplace_back(row_cap, y_idx[ari][0][0][k], 1.0);
        }
      }
      if (use_agg && any_entry)
        ineq_trips.emplace_back(row_recv, n_agg_idx[li][0][k], w_rate);

      if (!any_entry) {
        bineq_vec[row_recv] = 1.0e20;
        bineq_vec[row_cap] = 1.0e20;
      }
    }
  }

  lp.c = Eigen::VectorXd::Map(obj_vec.data(), vars.size());
  lp.vars = std::move(vars);

  if (!beq_vec.empty()) {
    lp.Aeq.resize(static_cast<int>(beq_vec.size()), lp.vars.size());
    lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
    lp.Aeq.makeCompressed();
    lp.beq = Eigen::VectorXd::Map(beq_vec.data(), beq_vec.size());
  }

  if (!bineq_vec.empty()) {
    lp.A.resize(static_cast<int>(bineq_vec.size()), lp.vars.size());
    lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
    lp.A.makeCompressed();
    lp.b = Eigen::VectorXd::Map(bineq_vec.data(), bineq_vec.size());
  }

  engine::SolveResult solve_result;
  engine::HighsAdapter highs;
  if (highs.available()) {
    solve_result = highs.solve_lp(lp);
  }
  if (!solve_result.stats.success) {
    engine::SimplexOptions simp_opt;
    simp_opt.max_iter = 100000;
    simp_opt.feasibility_tol = 1e-8;
    simp_opt.optimality_tol = 1e-8;
    simp_opt.verbose = false;
    auto lp_result = engine::solve_lp_with_basis(lp, simp_opt, nullptr);
    solve_result = lp_result.result;
  }

  result.solved = solve_result.stats.success;
  result.infeasible = !result.solved;
  result.status = solve_result.stats.status;
  result.objective = 0.0;
  result.best_bound = 0.0;
  result.mip_gap = 0.0;

  if (!result.solved) return result;

  const auto& x = solve_result.x;
  if (x.size() == lp.c.size()) {
    result.objective = lp.c.dot(x);
    result.best_bound = result.objective;
  } else {
    result.objective = solve_result.stats.objective;
    result.best_bound = result.objective;
  }
  // best_bound is set to the primal objective.  The solver certifies
  // optimality internally, but this wrapper does not independently re-verify
  // dual feasibility from the LP certificate residuals.
  result.best_bound_is_primal_only = true;
  result.per_route_decomposition_used = !use_agg;
  // Closed-link availability rows now zero out both per-route entry/exit
  // y[ari][pi][m][k] (skipping -1 indices in aggregate mode) and the shared
  // internal y_agg_int[li][m][k] flows (Bug 5a fix).
  result.closed_link_handling_strict = true;
  const auto cert = compute_solver_certificate_residuals(mip, x);
  result.primal_max_violation = cert.primal_max_violation;
  result.integrality_max_violation = cert.integrality_max_violation;
  result.proven_optimal = result.solved &&
                          result.primal_max_violation <= 1.0e-7 &&
                          result.integrality_max_violation <= 1.0e-7;
  result.dynamic_constraints_enforced = true;
  result.node_flow_variables_enforced = !movement_vars.empty();
  result.n_node_flow_variables = 0;
  for (const auto& kv : movement_vars) {
    result.n_node_flow_variables += static_cast<int>(kv.second.size());
  }
  result.node_flow_conservation_max_violation = result.primal_max_violation;

  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    if (!h_idx.count(demand.index)) continue;
    for (const auto& kv : h_idx.at(demand.index)) {
      if (kv.second >= 0 && kv.second < static_cast<int>(x.size()))
        result.flow_by_demand_route[demand.index][kv.first] = x[kv.second];
    }
  }

  double tstt = 0.0;
  if (use_agg) {
    for (int li = 0; li < n_links; ++li) {
      for (int m = 0; m < lcp[li].M; ++m)
        for (int k = 0; k < T_ctm; ++k) {
          int idx = n_agg_idx[li][m][k];
          if (idx >= 0 && idx < static_cast<int>(x.size()))
            tstt += x[idx] * dt_ctm;
        }
    }
  } else {
    for (std::size_t ari = 0; ari < active_routes.size(); ++ari) {
      for (std::size_t pi = 0; pi < active_routes[ari].path.size(); ++pi) {
        int M = lcp[active_routes[ari].path[pi]].M;
        for (int m = 0; m < M; ++m) {
          for (int k = 0; k < T_ctm; ++k) {
            int idx = n_idx[ari][pi][m][k];
            if (idx >= 0 && idx < static_cast<int>(x.size())) {
              tstt += x[idx] * dt_ctm;
            }
          }
        }
      }
    }
  }
  result.so_tstt_hr = tstt;

  return result;
}

}  // namespace hacdcpf::evpt
