// ltm_so_lp.cpp
// ──────────────────────────────────────────────────────────────────────────
// System-Optimal LTM LP (Formulation E-LTM)
//
// Minimises total vehicle-hours subject to Link Transmission Model (LTM)
// cumulative count constraints (Newell–Daganzo formulation).
//
// Variables (monotone, non-decreasing):
//   N^in_{a,k}   cumulative inflow  [veh] at link a, step k
//   N^out_{a,k}  cumulative outflow [veh] at link a, step k
//   h_{d,r,k}    route entry flow [veh]   demand d, route r, departure k
//
// Objective: Minimize Σ_{a,k} (N^in_{a,k} - N^out_{a,k}) · dt  [veh·hr]
//
// LTM constraints (triangular FD):
//   Sending (free-flow, τ_ff steps lag):
//     N^out_{a,k} - N^out_{a,k-1}  ≤  N^in_{a,k-τ_ff} - N^out_{a,k-1}
//   Capacity:
//     N^out_{a,k} - N^out_{a,k-1}  ≤  q̄_a · dt
//   Receiving (backward wave, τ_bw steps lag):
//     N^in_{a,k}  - N^in_{a,k-1}   ≤  N^out_{a_up,k-τ_bw} + N^jam_a - N^in_{a,k-1}
//       (where N^jam_a = q̄_a * T * dt  is a large enough capacity limit)
//   Monotone:
//     N^in_{a,k+1}  ≥ N^in_{a,k}
//     N^out_{a,k+1} ≥ N^out_{a,k}
//   OD conservation: Σ_r h_{d,r,k} = D_{d,k}
//   Node movements:
//     y_{a,b,k} links upstream cumulative exits to downstream cumulative entries
//   Entry linking: N^in_{a,0→T} injected by route flows on entry links
// ──────────────────────────────────────────────────────────────────────────

#include "evpt_internal.hpp"

#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace hacdcpf::evpt {

LTMSOLPResult solve_ltm_so_lp(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_opts,
    const LTMSOLPOptions& ltm_opts)
{
  if (ev_opts.auto_generate_routes) {
    EVPowerTrafficOptions local_opts = ev_opts;
    local_opts.auto_generate_routes = false;
    const EVPowerTrafficProblem generated =
        maybe_generate_candidate_routes(problem, ev_opts);
    return solve_ltm_so_lp(generated, local_opts, ltm_opts);
  }

  LTMSOLPResult result;

  if (ev_opts.require_exact_mathematical_model ||
      ltm_opts.require_exact_mathematical_model) {
    result.infeasible = true;
    result.status =
        "unsupported: exact route-commodity SO-LTM mathematical-model "
        "verification is not available for this implementation path";
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: SO-LTM certifies only the assembled traffic LP; aggregate "
        "mode is a reduced representation and per-route mode is not the full "
        "joint power-traffic model";
    return result;
  }

  const int    T  = ev_opts.num_steps;
  const double dt = ev_opts.time_step_hr;
  if (T <= 0 || dt <= kTol) {
    result.infeasible = true;
    result.status = "empty";
    return result;
  }

  // ── Index maps ──────────────────────────────────────────────────────────
  std::unordered_map<int, std::size_t> link_pos;
  const int n_links  = static_cast<int>(problem.traffic.links.size());
  for (int li = 0; li < n_links; ++li)
    link_pos[problem.traffic.links[static_cast<std::size_t>(li)].index] =
        static_cast<std::size_t>(li);

  // ── Per-link LTM parameters ─────────────────────────────────────────────
  struct LinkLTM {
    double vf;      // free-flow speed [km/hr]
    double w;       // backward wave speed [km/hr]
    double qmax;    // capacity [veh/hr]
    double len;     // length [km]
    int    tau_ff;  // free-flow travel time [steps]
    int    tau_bw;  // backward wave time [steps]
    double njam;    // jam density * length = max vehicles on link
  };
  std::vector<LinkLTM> lltm(static_cast<std::size_t>(n_links));
  for (int li = 0; li < n_links; ++li) {
    const auto& lnk = problem.traffic.links[static_cast<std::size_t>(li)];
    const double vf  = link_free_flow_speed(lnk);
    const double len  = std::max(kTol, lnk.length_km);
    const double qmax = lnk.capacity_veh_per_hr > kTol ? lnk.capacity_veh_per_hr : vf * 0.1;
    const double w = [&]{
      const double kj = lnk.jam_vehicles > kTol ? lnk.jam_vehicles / len : qmax / vf * 2.0;
      const double kc = qmax / vf;
      if (kj > kc + kTol) return qmax / (kj - kc);
      const double fb = ltm_opts.backward_wave_speed_fallback_km_hr;
      return fb > kTol ? fb : vf * 0.25;
    }();
    const int tau_ff = std::max(1, static_cast<int>(std::round(len / (vf * dt))));
    const int tau_bw = std::max(1, static_cast<int>(std::round(len / (w  * dt))));
    const double kj   = qmax / w + qmax / vf;  // jam density [veh/km]
    const double njam = std::max(static_cast<double>(tau_ff + tau_bw) * qmax * dt,
                                  kj * len);
    lltm[static_cast<std::size_t>(li)] = {vf, w, qmax, len, tau_ff, tau_bw, njam};
  }

  // ── LP variables ──────────────────────────────────────────────────────

  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  struct ActiveRoute {
    int d_idx;
    int r_idx;
    int dep_step;
    double vehicles;
    std::vector<int> links;
  };
  std::vector<ActiveRoute> act_routes;
  for (const auto& d : problem.demands) {
    if (d.vehicles <= kTol || d.departure_step < 0 || d.departure_step >= T) continue;
    for (const auto* r : candidate_routes(problem, d)) {
      if (!route_soc_feasible(*r, d, problem, link_pos)) continue;
      act_routes.push_back({d.index, r->index, d.departure_step, d.vehicles, r->link_indices});
    }
  }

  std::vector<engine::VariableMeta> vars;
  std::vector<double> obj_vec;
  std::vector<Eigen::Triplet<double>> eq_trips, ineq_trips;
  std::vector<double> beq, bineq;

  auto add_var = [&](double lb, double ub, double cost, const std::string& nm) -> int {
    const int idx = static_cast<int>(vars.size());
    vars.push_back({engine::VarType::Continuous, lb, ub, nm});
    obj_vec.push_back(cost);
    return idx;
  };
  auto add_eq = [&](double rhs) -> int {
    beq.push_back(rhs); return static_cast<int>(beq.size()) - 1;
  };
  auto add_ineq = [&](double rhs) -> int {
    bineq.push_back(rhs); return static_cast<int>(bineq.size()) - 1;
  };

  // Variables
  std::unordered_map<int, std::unordered_map<int, int>> h_idx;

  // ── Aggregate mode: per-link Nin/Nout ─────────────────────────────────
  // When use_aggregate_link_variables = true, use ONE Nin_agg[li][k] and
  // Nout_agg[li][k] per link-step for the aggregate LTM sending/receiving
  // envelope. Per-route Nin/Nout are omitted; h_idx and Ynode are kept for OD
  // and node-movement accounting. This reduced mode is not a full per-route
  // FIFO certificate.
  const bool use_agg = ltm_opts.use_aggregate_link_variables;

  // n_in[ari][pi][k]
  using vec2d = std::vector<std::vector<int>>;
  std::vector<vec2d> Nin(act_routes.size());
  std::vector<vec2d> Nout(act_routes.size());
  std::vector<vec2d> Ynode(act_routes.size());

  // Aggregate Nin/Nout indexed by [li][k]
  std::vector<std::vector<int>> Nin_agg, Nout_agg;

  if (use_agg) {
    Nin_agg.resize(static_cast<std::size_t>(n_links));
    Nout_agg.resize(static_cast<std::size_t>(n_links));
    for (int li = 0; li < n_links; ++li) {
      Nin_agg[li].resize(T + 1, -1);
      Nout_agg[li].resize(T + 1, -1);
      for (int k = 0; k <= T; ++k) {
        Nin_agg[li][k] = add_var(0.0, 1e20, dt,
            "Nin_a" + std::to_string(li) + "_k" + std::to_string(k));
        Nout_agg[li][k] = add_var(0.0, 1e20, -dt,
            "Nout_a" + std::to_string(li) + "_k" + std::to_string(k));
      }
      // IC: Nin_agg[li][0] = 0, Nout_agg[li][0] = 0
      { int r = add_eq(0.0); eq_trips.emplace_back(r, Nin_agg[li][0], 1.0); }
      { int r = add_eq(0.0); eq_trips.emplace_back(r, Nout_agg[li][0], 1.0); }
    }
  }

  for (size_t ari = 0; ari < act_routes.size(); ++ari) {
    const auto& ar = act_routes[ari];
    h_idx[ar.d_idx][ar.r_idx] = add_var(0.0, ar.vehicles, 0.0,
        "h_d" + std::to_string(ar.d_idx) + "_r" + std::to_string(ar.r_idx));
    
    size_t np = ar.links.size();
    Nin[ari].resize(np);
    Nout[ari].resize(np);
    Ynode[ari].resize(np > 1 ? np - 1 : 0);
    for (size_t pi = 0; pi < np; ++pi) {
      Nin[ari][pi].resize(T + 1, -1);
      Nout[ari][pi].resize(T + 1, -1);
      if (!use_agg) {
        for (int k = 0; k <= T; ++k) {
          Nin[ari][pi][k] = add_var(0.0, 1e20, dt,
              "Nin_r" + std::to_string(ari) + "_p" + std::to_string(pi) + "_k" + std::to_string(k));
          Nout[ari][pi][k] = add_var(0.0, 1e20, -dt,
              "Nout_r" + std::to_string(ari) + "_p" + std::to_string(pi) + "_k" + std::to_string(k));
        }
      }
      // In aggregate mode, Nin/Nout[ari][pi][k] stay -1; Nin_agg/Nout_agg used instead.
      if (pi + 1 < np) {
        Ynode[ari][pi].resize(T + 1, -1);
        for (int k = 1; k <= T; ++k) {
          Ynode[ari][pi][k] = add_var(0.0, 1e20, 0.0,
              "Ynode_r" + std::to_string(ari) + "_p" + std::to_string(pi) + "_k" + std::to_string(k));
        }
      }
    }
  }

  // Demand constraints
  for (const auto& d : problem.demands) {
    if (d.vehicles <= kTol || d.departure_step < 0 || d.departure_step >= T) continue;
    int req = add_eq(d.vehicles);
    for (const auto& [ridx, hv] : h_idx[d.index]) {
      eq_trips.emplace_back(req, hv, 1.0);
    }
    if (ev_opts.allow_unserved_travel_demand) {
      double pen = ev_opts.value_of_time_per_hr * T;
      int uv = add_var(0.0, d.vehicles, pen, "u_d" + std::to_string(d.index));
      eq_trips.emplace_back(req, uv, 1.0);
    }
  }

  // Monotone & Node continuity
  for (size_t ari = 0; ari < act_routes.size(); ++ari) {
    const auto& ar = act_routes[ari];
    size_t np = ar.links.size();
    for (size_t pi = 0; pi < np; ++pi) {
      if (!use_agg) {
        for (int k = 0; k <= T; ++k) {
            if (k == 0) {
                int r1 = add_eq(0.0); eq_trips.emplace_back(r1, Nin[ari][pi][0], 1.0);
                int r2 = add_eq(0.0); eq_trips.emplace_back(r2, Nout[ari][pi][0], 1.0);
            } else {
                int r1 = add_ineq(0.0);
                ineq_trips.emplace_back(r1, Nin[ari][pi][k-1], 1.0);
                ineq_trips.emplace_back(r1, Nin[ari][pi][k], -1.0);
                
                int r2 = add_ineq(0.0);
                ineq_trips.emplace_back(r2, Nout[ari][pi][k-1], 1.0);
                ineq_trips.emplace_back(r2, Nout[ari][pi][k], -1.0);
            }
        }
        
        // Node continuity is expressed below through explicit movement
        // variables Ynode[ari][pi][k].  This replaces the older cumulative
        // identity Nin_next[k] = Nout_prev[k] so split/merge constraints can
        // be enforced per movement and per time step.
        if (pi > 0) {
            for (int k = 1; k <= T; ++k) {
                const int yv = Ynode[ari][pi - 1][k];
                int r_in = add_eq(0.0);
                eq_trips.emplace_back(r_in, Nin[ari][pi][k], 1.0);
                eq_trips.emplace_back(r_in, Nin[ari][pi][k - 1], -1.0);
                eq_trips.emplace_back(r_in, yv, -1.0);

                int r_out = add_eq(0.0);
                eq_trips.emplace_back(r_out, Nout[ari][pi - 1][k], 1.0);
                eq_trips.emplace_back(r_out, Nout[ari][pi - 1][k - 1], -1.0);
                eq_trips.emplace_back(r_out, yv, -1.0);
            }
        }
      } else {
        // Aggregate mode: node continuity via Ynode movement variables still
        // connects route-path segments (for multi-link routes, Ynode is still
        // per-route so the movement capacity can be enforced at each merge).
        // No per-route IC/monotone needed; aggregate IC/monotone added above.
        if (pi > 0) {
          for (int k = 1; k <= T; ++k) {
            // Ynode[ari][pi-1][k] = flow entering link pi at step k for route ari.
            // In aggregate mode, no per-route Nin/Nout to tie to, so the
            // node continuity ensures aggregate entry = sum of all movements.
            // We record Ynode vars for movement capacity enforcement (below).
            (void)Ynode[ari][pi - 1][k]; // registered by movement_vars loop below
          }
        }
      }
    }
    
    // Origin injection:
    // Per-route mode: Nin_{ari,0}[k]=0 for k<=dep_step; Nin_{ari,0}[T]=h
    // Aggregate mode:  Nin_agg[li_entry][T] += h  for each route; we add one
    //   aggregate injection equality: Nin_agg[entry_link][T] = Σ h_{ari: entry=entry_link}
    //   (handled after all routes processed, in a separate pass below)
    if (!use_agg && np > 0) {
        for (int k = 0; k <= ar.dep_step; ++k) {
            int r = add_eq(0.0);
            eq_trips.emplace_back(r, Nin[ari][0][k], 1.0);
        }
        // Total demand fully injected by horizon end: Nin[0][T] = h
        {
            int req = add_eq(0.0);
            eq_trips.emplace_back(req, Nin[ari][0][T], 1.0);
            eq_trips.emplace_back(req, h_idx[ar.d_idx][ar.r_idx], -1.0);
        }
    }
  }

  // Aggregate mode: total injected into each entry link = Σ h for routes entering it.
  // Nin_agg[li][T] = Σ_{routes with entry_link=li} h_{d,r}
  if (use_agg) {
    // Group active routes by their entry link
    std::unordered_map<int, std::vector<std::pair<int,int>>> entry_link_routes; // li -> [(d_idx,r_idx)]
    for (size_t ari = 0; ari < act_routes.size(); ++ari) {
      if (act_routes[ari].links.empty()) continue;
      int entry_id = act_routes[ari].links[0];
      auto it = link_pos.find(entry_id);
      if (it == link_pos.end()) continue;
      int li = static_cast<int>(it->second);
      entry_link_routes[li].emplace_back(act_routes[ari].d_idx, act_routes[ari].r_idx);
    }
    for (auto& [li, rv] : entry_link_routes) {
      int req = add_eq(0.0);
      eq_trips.emplace_back(req, Nin_agg[li][T], 1.0);
      for (auto [did, rid] : rv)
        eq_trips.emplace_back(req, h_idx[did][rid], -1.0);
    }
    // Aggregate monotone (already have IC from above)
    for (int li = 0; li < n_links; ++li) {
      for (int k = 1; k <= T; ++k) {
        int r1 = add_ineq(0.0);
        ineq_trips.emplace_back(r1, Nin_agg[li][k-1], 1.0);
        ineq_trips.emplace_back(r1, Nin_agg[li][k], -1.0);
        int r2 = add_ineq(0.0);
        ineq_trips.emplace_back(r2, Nout_agg[li][k-1], 1.0);
        ineq_trips.emplace_back(r2, Nout_agg[li][k], -1.0);
      }
    }

    // Couple aggregate cumulative counts to per-route node movements.
    // For each intermediate link li at step k >= 1, the cumulative inflow
    // increment must at least cover the Σ of all per-route Ynode movements
    // arriving at li during step k.  Combined with the LTM receiving envelope
    // and the LP's preference for small Nin (positive cost coefficient +dt),
    // this pins Nin_agg correctly at the lower bound for intermediate links.
    // Without this row, intermediate Nin_agg can stay 0 while Ynode flows
    // pass through, producing trivially low TSTT.
    //
    //   Nin_agg[li][k] - Nin_agg[li][k-1]  >=  Σ_{ari, pi>=1, ari.links[pi]==li_id} Ynode[ari][pi-1][k]
    {
      // Bucket Y_node movements by (li, k).
      std::vector<std::vector<std::vector<int>>> inflow_per_link(
          static_cast<std::size_t>(n_links));
      for (auto& v : inflow_per_link) v.assign(T + 1, {});
      for (size_t ari = 0; ari < act_routes.size(); ++ari) {
        const auto& ar = act_routes[ari];
        for (size_t pi = 1; pi < ar.links.size(); ++pi) {
          auto it = link_pos.find(ar.links[pi]);
          if (it == link_pos.end()) continue;
          int li = static_cast<int>(it->second);
          for (int k = 1; k <= T; ++k) {
            const int yv = Ynode[ari][pi - 1][k];
            if (yv >= 0) inflow_per_link[li][k].push_back(yv);
          }
        }
      }
      for (int li = 0; li < n_links; ++li) {
        for (int k = 1; k <= T; ++k) {
          if (inflow_per_link[li][k].empty()) continue;
          // Nin_agg[li][k-1] + Σ y_node <= Nin_agg[li][k]   →  rhs = 0
          int r = add_ineq(0.0);
          ineq_trips.emplace_back(r, Nin_agg[li][k - 1], 1.0);
          for (int yv : inflow_per_link[li][k])
            ineq_trips.emplace_back(r, yv, 1.0);
          ineq_trips.emplace_back(r, Nin_agg[li][k], -1.0);
        }
      }
    }
  }

  // Physical node movement constraints.  The Ynode variables above are
  // route-specific movement flows; these aggregate them by (upstream link,
  // downstream link, step) to enforce shared movement capacity and the
  // downstream link receiving envelope.
  std::map<std::tuple<int, int, int>, std::vector<int>> movement_vars;
  for (size_t ari = 0; ari < act_routes.size(); ++ari) {
    for (size_t pi = 0; pi + 1 < act_routes[ari].links.size(); ++pi) {
      const int up = act_routes[ari].links[pi];
      const int dn = act_routes[ari].links[pi + 1];
      for (int k = 1; k <= T; ++k) {
        movement_vars[{up, dn, k}].push_back(Ynode[ari][pi][k]);
      }
    }
  }
  for (const auto& kv : movement_vars) {
    const auto [up_link, dn_link, k] = kv.first;
    (void)up_link;
    auto dn_pos_it = link_pos.find(dn_link);
    if (dn_pos_it == link_pos.end()) continue;
    const auto& p = lltm[dn_pos_it->second];
    const double qdt = p.qmax * dt;

    // Shared downstream entry capacity for this movement.
    int rcap = add_ineq(qdt);
    for (int yv : kv.second) {
      ineq_trips.emplace_back(rcap, yv, 1.0);
    }

    // Shared downstream receiving envelope:
    // sum y_{a,b,k} + sum N_in_b[k-1] - sum N_out_b[k-tau_bw] <= Njam_b.
    int rrecv = add_ineq(p.njam);
    for (int yv : kv.second) {
      ineq_trips.emplace_back(rrecv, yv, 1.0);
    }
    const int k_lag_bw = std::max(0, k - p.tau_bw);
    if (use_agg) {
      auto dn_it = link_pos.find(dn_link);
      if (dn_it != link_pos.end()) {
        int dn_li = static_cast<int>(dn_it->second);
        ineq_trips.emplace_back(rrecv, Nin_agg[dn_li][k - 1], 1.0);
        ineq_trips.emplace_back(rrecv, Nout_agg[dn_li][k_lag_bw], -1.0);
      }
    } else {
      for (size_t ari = 0; ari < act_routes.size(); ++ari) {
        for (size_t pi = 0; pi < act_routes[ari].links.size(); ++pi) {
          if (act_routes[ari].links[pi] != dn_link) continue;
          ineq_trips.emplace_back(rrecv, Nin[ari][pi][k - 1], 1.0);
          ineq_trips.emplace_back(rrecv, Nout[ari][pi][k_lag_bw], -1.0);
        }
      }
    }
  }

  // Origin injection into an entry link also consumes the shared entry
  // capacity and receiving space of the first physical link.
  for (int li = 0; li < n_links; ++li) {
    const int link_id = problem.traffic.links[static_cast<std::size_t>(li)].index;
    const auto& p = lltm[static_cast<std::size_t>(li)];
    const double qdt = p.qmax * dt;
    for (int k = 1; k <= T; ++k) {
      int rcap = add_ineq(qdt);
      int rrecv = add_ineq(p.njam);
      bool any = false;
      const int k_lag_bw = std::max(0, k - p.tau_bw);
      if (use_agg) {
        // Check if any route starts at this link
        for (size_t ari = 0; ari < act_routes.size(); ++ari) {
          if (!act_routes[ari].links.empty() && act_routes[ari].links[0] == link_id) {
            any = true; break;
          }
        }
        if (any) {
          ineq_trips.emplace_back(rcap, Nin_agg[li][k], 1.0);
          ineq_trips.emplace_back(rcap, Nin_agg[li][k - 1], -1.0);
          ineq_trips.emplace_back(rrecv, Nin_agg[li][k], 1.0);
          ineq_trips.emplace_back(rrecv, Nout_agg[li][k_lag_bw], -1.0);
        }
      } else {
        for (size_t ari = 0; ari < act_routes.size(); ++ari) {
          if (act_routes[ari].links.empty() || act_routes[ari].links[0] != link_id) continue;
          any = true;
          ineq_trips.emplace_back(rcap, Nin[ari][0][k], 1.0);
          ineq_trips.emplace_back(rcap, Nin[ari][0][k - 1], -1.0);
          ineq_trips.emplace_back(rrecv, Nin[ari][0][k], 1.0);
          ineq_trips.emplace_back(rrecv, Nout[ari][0][k_lag_bw], -1.0);
        }
      }
      if (!any) {
        bineq[static_cast<std::size_t>(rcap)] = 1.0e20;
        bineq[static_cast<std::size_t>(rrecv)] = 1.0e20;
      }
    }
  }

  // Aggregate node receiving by downstream physical link.  Per-movement
  // rows above are useful diagnostics, but the shared node model needs one
  // row per downstream link and step so merges and origins compete for the
  // same receiving space.
  for (int li = 0; li < n_links; ++li) {
    const int link_id = problem.traffic.links[static_cast<std::size_t>(li)].index;
    const auto& p = lltm[static_cast<std::size_t>(li)];
    const double qdt = p.qmax * dt;
    for (int k = 1; k <= T; ++k) {
      int rcap = add_ineq(qdt);
      int rrecv = add_ineq(p.njam);
      bool any = false;
      const int k_lag_bw = std::max(0, k - p.tau_bw);

      for (const auto& kv : movement_vars) {
        const auto [up_link, dn_link, kk] = kv.first;
        (void)up_link;
        if (dn_link != link_id || kk != k) continue;
        any = true;
        for (int yv : kv.second) {
          ineq_trips.emplace_back(rcap, yv, 1.0);
          ineq_trips.emplace_back(rrecv, yv, 1.0);
        }
      }
      if (use_agg) {
        // Aggregate Nin/Nout for this link at this step
        ineq_trips.emplace_back(rrecv, Nin_agg[li][k - 1], 1.0);
        ineq_trips.emplace_back(rrecv, Nout_agg[li][k_lag_bw], -1.0);
        // Check if any route starts at this link for origin injection
        for (size_t ari = 0; ari < act_routes.size(); ++ari) {
          if (!act_routes[ari].links.empty() && act_routes[ari].links[0] == link_id) {
            any = true;
            ineq_trips.emplace_back(rcap, Nin_agg[li][k], 1.0);
            ineq_trips.emplace_back(rcap, Nin_agg[li][k - 1], -1.0);
            ineq_trips.emplace_back(rrecv, Nin_agg[li][k], 1.0);
            ineq_trips.emplace_back(rrecv, Nout_agg[li][k_lag_bw], -1.0);
            break; // aggregate: add once only
          }
        }
      } else {
        for (size_t ari = 0; ari < act_routes.size(); ++ari) {
          for (size_t pi = 0; pi < act_routes[ari].links.size(); ++pi) {
            if (act_routes[ari].links[pi] == link_id) {
              ineq_trips.emplace_back(rrecv, Nin[ari][pi][k - 1], 1.0);
              ineq_trips.emplace_back(rrecv, Nout[ari][pi][k_lag_bw], -1.0);
            }
          }
          if (!act_routes[ari].links.empty() &&
              act_routes[ari].links[0] == link_id) {
            any = true;
            ineq_trips.emplace_back(rcap, Nin[ari][0][k], 1.0);
            ineq_trips.emplace_back(rcap, Nin[ari][0][k - 1], -1.0);
            ineq_trips.emplace_back(rrecv, Nin[ari][0][k], 1.0);
            ineq_trips.emplace_back(rrecv, Nout[ari][0][k_lag_bw], -1.0);
          }
        }
      }

      if (!any) {
        bineq[static_cast<std::size_t>(rcap)] = 1.0e20;
        bineq[static_cast<std::size_t>(rrecv)] = 1.0e20;
      }
    }
  }

  // Link capacity
  for (int li = 0; li < n_links; ++li) {
      const auto& lnk = problem.traffic.links[static_cast<std::size_t>(li)];
      int l_idx = lnk.index;
      const auto& p = lltm[static_cast<std::size_t>(li)];
      double qdt = p.qmax * dt;
      
      for (int k = 1; k <= T; ++k) {
          // ── Availability: closed link → Nin[k] = Nin[k-1], Nout[k] = Nout[k-1] ──
          bool avail = lnk.available;
          if (!lnk.availability_profile.empty() &&
              k <= static_cast<int>(lnk.availability_profile.size()))
            avail = lnk.availability_profile[static_cast<std::size_t>(k - 1)];
          if (!avail) {
            if (use_agg) {
              // Nin_agg[li][k] = Nin_agg[li][k-1], Nout_agg[li][k] = Nout_agg[li][k-1]
              { int r = add_eq(0.0);
                eq_trips.emplace_back(r, Nin_agg[li][k], 1.0);
                eq_trips.emplace_back(r, Nin_agg[li][k-1], -1.0); }
              { int r = add_eq(0.0);
                eq_trips.emplace_back(r, Nout_agg[li][k], 1.0);
                eq_trips.emplace_back(r, Nout_agg[li][k-1], -1.0); }
            } else {
              for (size_t ari = 0; ari < act_routes.size(); ++ari) {
                for (size_t pi = 0; pi < act_routes[ari].links.size(); ++pi) {
                  if (act_routes[ari].links[pi] == l_idx) {
                    // Nin[k] = Nin[k-1]: no new vehicles can enter
                    { int r = add_eq(0.0);
                      eq_trips.emplace_back(r, Nin[ari][pi][k], 1.0);
                      eq_trips.emplace_back(r, Nin[ari][pi][k-1], -1.0); }
                    // Nout[k] = Nout[k-1]: no vehicles can exit
                    { int r = add_eq(0.0);
                      eq_trips.emplace_back(r, Nout[ari][pi][k], 1.0);
                      eq_trips.emplace_back(r, Nout[ari][pi][k-1], -1.0); }
                  }
                }
              }
            }
            continue;
          }

          // Sending: Sum Nout_k <= Sum Nin_{k-tau_ff}
          int rs = add_ineq(0.0);
          // Recv: Sum Nin_k <= Sum Nout_{k-tau_bw} + njam
          int rr = add_ineq(p.njam);
          // Cap out: Sum Nout_k - Sum Nout_{k-1} <= qdt
          int rcout = add_ineq(qdt);
          // Cap in: Sum Nin_k - Sum Nin_{k-1} <= qdt
          int rcin = add_ineq(qdt);

          if (use_agg) {
            int k_lag_ff = std::max(0, k - p.tau_ff);
            int k_lag_bw = std::max(0, k - p.tau_bw);
            ineq_trips.emplace_back(rs, Nout_agg[li][k], 1.0);
            ineq_trips.emplace_back(rs, Nin_agg[li][k_lag_ff], -1.0);
            ineq_trips.emplace_back(rr, Nin_agg[li][k], 1.0);
            ineq_trips.emplace_back(rr, Nout_agg[li][k_lag_bw], -1.0);
            ineq_trips.emplace_back(rcout, Nout_agg[li][k], 1.0);
            ineq_trips.emplace_back(rcout, Nout_agg[li][k-1], -1.0);
            ineq_trips.emplace_back(rcin, Nin_agg[li][k], 1.0);
            ineq_trips.emplace_back(rcin, Nin_agg[li][k-1], -1.0);
          } else {
            for (size_t ari = 0; ari < act_routes.size(); ++ari) {
                for (size_t pi = 0; pi < act_routes[ari].links.size(); ++pi) {
                    if (act_routes[ari].links[pi] == l_idx) {
                        int n_out_k = Nout[ari][pi][k];
                        int n_out_km1 = Nout[ari][pi][k-1];
                        int n_in_k = Nin[ari][pi][k];
                        int n_in_km1 = Nin[ari][pi][k-1];
                        
                        int k_lag_ff = std::max(0, k - p.tau_ff);
                        int n_in_lag = Nin[ari][pi][k_lag_ff];
                        
                        int k_lag_bw = std::max(0, k - p.tau_bw);
                        int n_out_lag = Nout[ari][pi][k_lag_bw];
                        
                        ineq_trips.emplace_back(rs, n_out_k, 1.0);
                        ineq_trips.emplace_back(rs, n_in_lag, -1.0);
                        
                        ineq_trips.emplace_back(rr, n_in_k, 1.0);
                        ineq_trips.emplace_back(rr, n_out_lag, -1.0);
                        
                        ineq_trips.emplace_back(rcout, n_out_k, 1.0);
                        ineq_trips.emplace_back(rcout, n_out_km1, -1.0);
                        
                        ineq_trips.emplace_back(rcin, n_in_k, 1.0);
                        ineq_trips.emplace_back(rcin, n_in_km1, -1.0);
                    }
                }
            }
          }
      }
  }

  lp.c = Eigen::VectorXd::Map(obj_vec.data(), vars.size());
  lp.vars = std::move(vars);

  if (!beq.empty()) {
    lp.Aeq.resize(beq.size(), lp.vars.size());
    lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
    lp.Aeq.makeCompressed();
    lp.beq = Eigen::VectorXd::Map(beq.data(), beq.size());
  }
  if (!bineq.empty()) {
    lp.A.resize(bineq.size(), lp.vars.size());
    lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
    lp.A.makeCompressed();
    lp.b = Eigen::VectorXd::Map(bineq.data(), bineq.size());
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
    simp_opt.optimality_tol  = 1e-8;
    simp_opt.verbose = false;
    auto lp_result = engine::solve_lp_with_basis(lp, simp_opt, nullptr);
    solve_result = lp_result.result;
  }

  result.solved     = solve_result.stats.success;
  result.infeasible = !result.solved;
  result.status     = solve_result.stats.status;
  result.objective  = result.solved ? solve_result.stats.objective : 0.0;
  // The LP objective is Σ_{a,k} (N^in_{a,k} - N^out_{a,k}) * dt, which is
  // exactly the total system travel time in vehicle-hours.
  result.so_tstt_hr = result.objective;
  result.best_bound = result.objective;
  result.mip_gap = 0.0;

  if (!result.solved) return result;
  const auto& x = solve_result.x;
  if (x.size() == lp.c.size()) {
    result.objective = lp.c.dot(x);
    result.so_tstt_hr = result.objective;
    result.best_bound = result.objective;
  }
  // best_bound equals the primal objective; no independent dual certificate.
  result.best_bound_is_primal_only = true;
  result.per_route_decomposition_used = !use_agg;
  // Intermediate-link Ynode→Nin_agg coupling is added unconditionally above
  // for aggregate mode (and per-route mode trivially enforces it via Nin
  // movement equalities); origin-timing strictness only holds in per-route mode.
  result.aggregate_intermediate_conservation_strict = true;
  result.aggregate_origin_timing_strict = !use_agg;
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
    if (!h_idx.count(demand.index)) continue;
    for (const auto& [ridx, hv] : h_idx.at(demand.index)) {
      if (hv >= 0 && hv < static_cast<int>(x.size()))
        result.flow_by_demand_route[demand.index][ridx] = x[hv];
    }
  }

  return result;
}

} // namespace hacdcpf::evpt
