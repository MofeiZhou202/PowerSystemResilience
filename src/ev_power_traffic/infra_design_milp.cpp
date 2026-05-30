// infra_design_milp.cpp
// ──────────────────────────────────────────────────────────────────────────
// Infrastructure Design MILP (Formulation G)
//
// Joint optimisation of:
//   - z_s ∈ {0,1}   : station open/close binary
//   - B̄_s ∈ ℤ₊       : number of plugs allocated (integer)
//   - h_{d,r} ≥ 0    : continuous route flow
//   - u_d   ≥ 0      : unserved demand
//
// Objective (minimise):
//   capital:  Σ_s c_plug · B̄_s
//   delay:    VOT · Σ_{d,r} h_{d,r} · t_{d,r}   (free-flow travel time proxy)
//   penalty:  M_out · Σ_d u_d
//
// Constraints:
//   OD balance:          Σ_r h_{d,r} + u_d = D_d
//   Station capacity:    Σ_{d,r: stop s} p_{drs} · h_{d,r} ≤ P^max_s · z_s
//   Plug admission:      Σ_{d,r: stop s} h_{d,r} ≤ B̄_s
//   Open indicator:      B̄_s ≤ B̄^max_s · z_s
//   Road capacity:       Σ_{d,r: link a, dep k} h_{d,r} ≤ Cap_a
//   Optional LTM layer:  time-expanded Nin/Nout/Ynode variables enforcing
//                       link transmission and node movement constraints
//
// Returns the optimal station configuration and route flows.
// ──────────────────────────────────────────────────────────────────────────

#include "evpt_internal.hpp"

#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace hacdcpf::evpt {

InfraDesignResult solve_infra_design_milp(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_opts,
    const InfraDesignMILPOptions& opts)
{
  if (ev_opts.auto_generate_routes) {
    EVPowerTrafficOptions local_opts = ev_opts;
    local_opts.auto_generate_routes = false;
    const EVPowerTrafficProblem generated =
        maybe_generate_candidate_routes(problem, ev_opts);
    return solve_infra_design_milp(generated, local_opts, opts);
  }

  InfraDesignResult result;

  if (ev_opts.require_exact_mathematical_model ||
      opts.require_exact_mathematical_model) {
    result.infeasible = true;
    result.status =
        "unsupported: exact infrastructure-design mathematical-model "
        "verification is not available for this reduced MILP";
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: infrastructure MILP uses documented objective/dynamic "
        "approximations rather than the full nonlinear power-traffic model";
    result.warnings.push_back(result.mathematical_model_verification_status);
    return result;
  }

  const int T  = ev_opts.num_steps;
  const double dt = ev_opts.time_step_hr;
  if (T <= 0 || dt <= kTol) {
    result.infeasible = true;
    result.status = "empty";
    return result;
  }

  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t i = 0; i < problem.traffic.links.size(); ++i)
    link_pos[problem.traffic.links[i].index] = i;

  engine::MIPModel mip;
  engine::LPModel& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;

  std::vector<engine::VariableMeta> vars;
  std::vector<double> obj_vec;
  std::vector<Eigen::Triplet<double>> eq_trips, ineq_trips;
  std::vector<double> beq_vec, bineq_vec;

  auto add_var = [&](double lb, double ub, double cost, const std::string& nm,
                     bool integer = false) -> int {
    const int idx = static_cast<int>(vars.size());
    vars.push_back({integer ? engine::VarType::Integer : engine::VarType::Continuous,
                    lb, ub, nm});
    obj_vec.push_back(cost);
    if (integer) mip.integer_idx.push_back(idx);
    return idx;
  };
  auto add_binary = [&](const std::string& nm) -> int {
    const int idx = static_cast<int>(vars.size());
    vars.push_back({engine::VarType::Binary, 0.0, 1.0, nm});
    obj_vec.push_back(0.0);
    mip.binary_idx.push_back(idx);
    return idx;
  };
  auto add_eq   = [&](double rhs) -> int {
    const int r = static_cast<int>(beq_vec.size()); beq_vec.push_back(rhs); return r; };
  auto add_ineq = [&](double rhs) -> int {
    const int r = static_cast<int>(bineq_vec.size()); bineq_vec.push_back(rhs); return r; };

  // ── Station variables z_s, B̄_s ─────────────────────────────────────────
  // Identify stations from routes
  std::unordered_set<int> station_ids;
  for (const auto& route : problem.routes)
    for (const auto& stop : route.charging_stops)
      station_ids.insert(stop.station_id);

  std::unordered_map<int, int> z_var, B_var;
  const double M_big = static_cast<double>(opts.max_plugs_per_station);
  for (int sid : station_ids) {
    z_var[sid] = add_binary("z_s" + std::to_string(sid));
    B_var[sid] = add_var(0.0, M_big, opts.capital_cost_per_plug,
                          "B_s" + std::to_string(sid), /*integer=*/true);
    // B̄_s ≤ B̄^max · z_s:  B_var - M_big * z_var ≤ 0
    { const int r = add_ineq(0.0);
      ineq_trips.emplace_back(r, B_var.at(sid),  1.0);
      ineq_trips.emplace_back(r, z_var.at(sid), -M_big); }
  }

  // ── Route flow and unserved variables ──────────────────────────────────
  const double vot = std::max(kTol, ev_opts.value_of_time_per_hr);
  const double pen = std::max(0.0, ev_opts.value_of_time_per_hr * static_cast<double>(T));

  std::unordered_map<int, std::unordered_map<int, int>> h_idx;
  std::unordered_map<int, int> od_eq_row;
  struct ActiveRouteFlow {
    int demand_index{0};
    int route_index{0};
    int h_var{-1};
    int dep_step{0};
    double vehicles{0.0};
    const RouteAlternative* route{nullptr};
  };
  std::vector<ActiveRouteFlow> active_route_flows;

  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    if (demand.departure_step < 0 || demand.departure_step >= T) continue;
    od_eq_row[demand.index] = add_eq(demand.vehicles);
    for (const auto* route : candidate_routes(problem, demand)) {
      if (!route_soc_feasible(*route, demand, problem, link_pos)) continue;
      const double cost_ff = vot * route_free_flow_time(*route, link_pos, problem);
      const int hv = add_var(0.0, demand.vehicles,
          cost_ff * opts.operating_cost_scale, "h_d" + std::to_string(demand.index) +
              "_r" + std::to_string(route->index));
      h_idx[demand.index][route->index] = hv;
      active_route_flows.push_back(
          {demand.index, route->index, hv, demand.departure_step,
           demand.vehicles, route});
      eq_trips.emplace_back(od_eq_row.at(demand.index), hv, 1.0);
    }
    const int uv = add_var(0.0, demand.vehicles, pen,
        "u_d" + std::to_string(demand.index));
    eq_trips.emplace_back(od_eq_row.at(demand.index), uv, 1.0);
  }

  // ── Station capacity and plug admission constraints ─────────────────────
  std::unordered_map<int, std::unordered_map<int, int>> stn_cap_row;  // [sid][step]
  std::unordered_map<int, int> stn_plug_row;                          // [sid]

  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    if (!h_idx.count(demand.index)) continue;
    for (const auto* route : candidate_routes(problem, demand)) {
      if (!h_idx.at(demand.index).count(route->index)) continue;
      const int hv = h_idx.at(demand.index).at(route->index);
      for (const auto& stop : route->charging_stops) {
        const int sid = stop.station_id;
        const double p_kw = std::max(0.0, ev_opts.default_route_stop_power_kw_per_vehicle);
        const int arr = route_arrival_step(*route, link_pos, problem,
                                           demand.departure_step, dt);
        for (int off = 0; off < std::max(1, stop.dwell_steps); ++off) {
          const int k = arr + off;
          if (k < 0 || k >= T) continue;
          // Power: Σ h · p_kw ≤ P^max_s · z_s
          if (!stn_cap_row.count(sid) || !stn_cap_row.at(sid).count(k)) {
            const double cap_kw = ev_opts.default_station_power_kw;
            const int r = add_ineq(0.0);  // rhs adjusted below
            stn_cap_row[sid][k] = r;
            // cap_kw * z_s on RHS: encode as -cap_kw * z_s on LHS
            ineq_trips.emplace_back(r, z_var.count(sid) ? z_var.at(sid) : -1, -cap_kw);
            bineq_vec.back() = 0.0;  // row: Σ p_kw*h - cap*z ≤ 0
          }
          ineq_trips.emplace_back(stn_cap_row.at(sid).at(k), hv, p_kw);
        }
        // Plug count: Σ h ≤ B̄_s
        if (!stn_plug_row.count(sid)) {
          const int r = add_ineq(0.0);
          stn_plug_row[sid] = r;
          if (B_var.count(sid))
            ineq_trips.emplace_back(r, B_var.at(sid), -1.0);
        }
        ineq_trips.emplace_back(stn_plug_row.at(sid), hv, 1.0);
      }
    }
  }

  // BPR aggregate flow/excess variable indices: populated by the road-capacity
  // block below and zero-ed out by the LTM block when LTM VHT costs are used.
  std::map<int, int> link_flow_var;   // link_id → variable index for f[li]
  std::map<int, int> link_excess_var; // link_id → variable index for e[li]

  // ── Road capacity constraints + BPR congestion (Beckmann function approx.) ──
  //
  // For the system-optimal infrastructure design we minimise the Beckmann
  // function W = Σ_a ∫₀^{f_a} t_a(v) dv rather than a static free-flow cost.
  // With the BPR delay t_a(v) = t0_a · (1 + α·(v/cap_a)^β) the exact integral
  // is nonlinear.  We use a piecewise-linear outer approximation with two
  // segments:
  //   [0, cap_a]   :  marginal cost  ≈  VOT · t0_a               (free-flow)
  //   [cap_a, ∞)   :  marginal cost  ≈  VOT · t0_a · (1 + α·β)  (at capacity)
  //
  // The excess flow e_a = max(0, f_a − cap_a) captures congestion above
  // capacity.  The penalty slope α·β = 0.15 × 4 = 0.6 matches the first-order
  // derivative of the BPR integral at v = cap_a.
  //
  // Note: the existing route-level free-flow cost (h_{d,r} · VOT · t_ff_r) is
  // replaced by the per-link aggregate (f_a · VOT · t0_a / cap_a per unit flow)
  // formulation below so that the two do not double-count.

  if (opts.include_road_capacity) {
    // Per-link aggregate flow f[li] = Σ_{d,r: a ∈ r} h_{d,r} (dep-step weighted)
    // We accumulate over ALL departure steps (routes are matched to link_indices).
    const double bpr_alpha = 0.15;
    const double bpr_beta  = 4.0;
    const double excess_slope = bpr_alpha * bpr_beta;  // ≈ 0.6

    // Step 1: collect link → (flow_row, excess_var) mappings
    std::map<int, int> link_flow_row;   // link_id → equality-row for f[li]
    // link_flow_var and link_excess_var are declared in outer scope

    for (std::size_t li = 0; li < problem.traffic.links.size(); ++li) {
      const auto& lnk = problem.traffic.links[li];
      const double cap = std::max(kTol, lnk.capacity_veh_per_hr * dt);
      const double t0  = lnk.free_flow_time_hr;
      if (t0 <= kTol) continue;

      // f[li]: aggregate route flow on link (cost = VOT * t0 for free-flow part)
      const double cff = vot * t0 * opts.operating_cost_scale;
      const int fv = add_var(0.0, cap * 10.0, cff,
                              "f_li" + std::to_string(lnk.index));
      link_flow_var[lnk.index]  = fv;

      // e[li]: excess flow above capacity (penalty slope = cff * excess_slope)
      const int ev_var = add_var(0.0, cap * 10.0, cff * excess_slope,
                                  "e_li" + std::to_string(lnk.index));
      link_excess_var[lnk.index] = ev_var;

      // Linking equality: f[li] = Σ h  (set up the row; filled below)
      const int fr = add_eq(0.0);
      link_flow_row[lnk.index] = fr;
      eq_trips.emplace_back(fr, fv, -1.0);   // −f[li] side

      // Excess lower bound: e[li] ≥ f[li] − cap  →  f[li] − e[li] ≤ cap
      { const int r = add_ineq(cap);
        ineq_trips.emplace_back(r, fv,    1.0);
        ineq_trips.emplace_back(r, ev_var, -1.0); }

      // Hard capacity ceiling (road capacity constraint):
      //   f[li] ≤ cap * headroom_factor (2× to allow for unserved demand)
      { const int r = add_ineq(cap * 2.0);
        ineq_trips.emplace_back(r, fv, 1.0); }
    }

    // Step 2: for each route, add h_{d,r} to f[li] for every link on the route
    // and zero out the old per-route free-flow cost (now captured by f[li]).
    for (const auto& demand : problem.demands) {
      if (demand.vehicles <= kTol) continue;
      if (!h_idx.count(demand.index)) continue;
      for (const auto* route : candidate_routes(problem, demand)) {
        if (!h_idx.at(demand.index).count(route->index)) continue;
        const int hv = h_idx.at(demand.index).at(route->index);
        // Remove old free-flow travel-time cost from h_{d,r} objective
        // (the per-link f[li] term now carries the full cost).
        obj_vec[static_cast<std::size_t>(hv)] = 0.0;
        for (int lid : route->link_indices) {
          if (!link_flow_row.count(lid)) continue;
          // f[li] += h_{d,r}  →  add +h on the f[li] row
          eq_trips.emplace_back(link_flow_row.at(lid), hv, 1.0);
        }
      }
    }
  }

  if (opts.include_ltm_dynamic_constraints) {
    result.dynamic_constraints_enforced = true;

    struct LinkLTM {
      double qmax{0.0};
      int tau_ff{1};
      int tau_bw{1};
      double njam{1.0};
    };
    std::vector<LinkLTM> ltm(static_cast<std::size_t>(problem.traffic.links.size()));
    for (std::size_t li = 0; li < problem.traffic.links.size(); ++li) {
      const auto& link = problem.traffic.links[li];
      const double vf = link_free_flow_speed(link);
      const double len = std::max(kTol, link.length_km);
      const double qmax = link.capacity_veh_per_hr > kTol
          ? link.capacity_veh_per_hr
          : vf * 0.1;
      const double kj = link.jam_vehicles > kTol
          ? link.jam_vehicles / len
          : qmax / vf * 2.0;
      const double kc = qmax / vf;
      const double w = kj > kc + kTol ? qmax / (kj - kc) : vf * 0.25;
      const int tau_ff = std::max(1, static_cast<int>(std::round(len / (vf * dt))));
      const int tau_bw = std::max(1, static_cast<int>(std::round(len / (w * dt))));
      const double njam = std::max(static_cast<double>(tau_ff + tau_bw) * qmax * dt,
                                   kj * len);
      ltm[li] = {qmax, tau_ff, tau_bw, std::max(1.0, njam)};
    }

    using Vec2 = std::vector<std::vector<int>>;
    std::vector<Vec2> Nin(active_route_flows.size());
    std::vector<Vec2> Nout(active_route_flows.size());
    std::vector<Vec2> Ynode(active_route_flows.size());
    double total_demand = 0.0;
    for (const auto& demand : problem.demands) {
      total_demand += std::max(0.0, demand.vehicles);
    }
    total_demand = std::max(1.0, total_demand);

    for (std::size_t ari = 0; ari < active_route_flows.size(); ++ari) {
      const auto* route = active_route_flows[ari].route;
      if (!route) continue;
      const std::size_t np = route->link_indices.size();
      Nin[ari].resize(np);
      Nout[ari].resize(np);
      Ynode[ari].resize(np > 1 ? np - 1 : 0);
      for (std::size_t pi = 0; pi < np; ++pi) {
        Nin[ari][pi].assign(static_cast<std::size_t>(T + 1), -1);
        Nout[ari][pi].assign(static_cast<std::size_t>(T + 1), -1);
        for (int k = 0; k <= T; ++k) {
          // VHT objective: VOT·dt·(N_in[k] − N_out[k]) summed over all k.
          // k=0 is the initial-condition (zero) — cost is zero.
          const double nin_cost  = (k > 0) ? vot * dt : 0.0;
          const double nout_cost = (k > 0) ? -vot * dt : 0.0;
          Nin[ari][pi][k] = add_var(0.0, total_demand, nin_cost,
              "dyn_Nin_a" + std::to_string(ari) + "_p" + std::to_string(pi) +
              "_k" + std::to_string(k));
          Nout[ari][pi][k] = add_var(0.0, total_demand, nout_cost,
              "dyn_Nout_a" + std::to_string(ari) + "_p" + std::to_string(pi) +
              "_k" + std::to_string(k));
        }
        if (pi + 1 < np) {
          Ynode[ari][pi].assign(static_cast<std::size_t>(T + 1), -1);
          for (int k = 1; k <= T; ++k) {
            Ynode[ari][pi][k] = add_var(0.0, total_demand, 0.0,
                "dyn_Ynode_a" + std::to_string(ari) + "_p" + std::to_string(pi) +
                "_k" + std::to_string(k));
          }
        }
      }
    }

    // LTM Nin/Nout variables carry the VHT objective (VOT·dt·ΔN).
    // Zero out the h_var free-flow costs and BPR f[li]/e[li] costs so that
    // travel time is not double-counted with the static cost proxies.
    for (const auto& arf : active_route_flows)
      obj_vec[static_cast<std::size_t>(arf.h_var)] = 0.0;
    for (const auto& kv : link_flow_var)
      obj_vec[static_cast<std::size_t>(kv.second)] = 0.0;
    for (const auto& kv : link_excess_var)
      obj_vec[static_cast<std::size_t>(kv.second)] = 0.0;

    for (std::size_t ari = 0; ari < active_route_flows.size(); ++ari) {
      const auto& arf = active_route_flows[ari];
      const auto* route = arf.route;
      if (!route || route->link_indices.empty()) continue;
      const std::size_t np = route->link_indices.size();
      for (std::size_t pi = 0; pi < np; ++pi) {
        int r0_in = add_eq(0.0);
        eq_trips.emplace_back(r0_in, Nin[ari][pi][0], 1.0);
        int r0_out = add_eq(0.0);
        eq_trips.emplace_back(r0_out, Nout[ari][pi][0], 1.0);
        for (int k = 1; k <= T; ++k) {
          int r_mono_in = add_ineq(0.0);
          ineq_trips.emplace_back(r_mono_in, Nin[ari][pi][k - 1], 1.0);
          ineq_trips.emplace_back(r_mono_in, Nin[ari][pi][k], -1.0);
          int r_mono_out = add_ineq(0.0);
          ineq_trips.emplace_back(r_mono_out, Nout[ari][pi][k - 1], 1.0);
          ineq_trips.emplace_back(r_mono_out, Nout[ari][pi][k], -1.0);
        }
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
      }

      for (int k = 0; k <= std::min(T, arf.dep_step); ++k) {
        int r = add_eq(0.0);
        eq_trips.emplace_back(r, Nin[ari][0][k], 1.0);
      }
      int r_full = add_eq(0.0);
      eq_trips.emplace_back(r_full, Nin[ari][0][T], 1.0);
      eq_trips.emplace_back(r_full, arf.h_var, -1.0);
    }

    for (std::size_t li = 0; li < problem.traffic.links.size(); ++li) {
      const auto& link = problem.traffic.links[li];
      const auto& p = ltm[li];
      const double qdt = p.qmax * dt;
      for (int k = 1; k <= T; ++k) {
        const bool avail = availability_value(link.availability_profile,
                                              k - 1, link.available);
        for (std::size_t ari = 0; ari < active_route_flows.size(); ++ari) {
          const auto* route = active_route_flows[ari].route;
          if (!route) continue;
          for (std::size_t pi = 0; pi < route->link_indices.size(); ++pi) {
            if (route->link_indices[pi] != link.index) continue;
            if (!avail) {
              int rin = add_eq(0.0);
              eq_trips.emplace_back(rin, Nin[ari][pi][k], 1.0);
              eq_trips.emplace_back(rin, Nin[ari][pi][k - 1], -1.0);
              int rout = add_eq(0.0);
              eq_trips.emplace_back(rout, Nout[ari][pi][k], 1.0);
              eq_trips.emplace_back(rout, Nout[ari][pi][k - 1], -1.0);
              continue;
            }
            const int k_ff = std::max(0, k - p.tau_ff);
            const int k_bw = std::max(0, k - p.tau_bw);
            int rsend = add_ineq(0.0);
            ineq_trips.emplace_back(rsend, Nout[ari][pi][k], 1.0);
            ineq_trips.emplace_back(rsend, Nin[ari][pi][k_ff], -1.0);
            int rrecv = add_ineq(p.njam);
            ineq_trips.emplace_back(rrecv, Nin[ari][pi][k], 1.0);
            ineq_trips.emplace_back(rrecv, Nout[ari][pi][k_bw], -1.0);
            int rcout = add_ineq(qdt);
            ineq_trips.emplace_back(rcout, Nout[ari][pi][k], 1.0);
            ineq_trips.emplace_back(rcout, Nout[ari][pi][k - 1], -1.0);
            int rcin = add_ineq(qdt);
            ineq_trips.emplace_back(rcin, Nin[ari][pi][k], 1.0);
            ineq_trips.emplace_back(rcin, Nin[ari][pi][k - 1], -1.0);
          }
        }
      }
    }

    std::map<std::tuple<int, int, int>, std::vector<int>> movement_vars;
    for (std::size_t ari = 0; ari < active_route_flows.size(); ++ari) {
      const auto* route = active_route_flows[ari].route;
      if (!route) continue;
      for (std::size_t pi = 0; pi + 1 < route->link_indices.size(); ++pi) {
        for (int k = 1; k <= T; ++k) {
          movement_vars[{route->link_indices[pi],
                         route->link_indices[pi + 1], k}]
              .push_back(Ynode[ari][pi][k]);
        }
      }
    }
    for (const auto& kv : movement_vars) {
      const auto [up, dn, k] = kv.first;
      (void)up;
      auto dn_it = link_pos.find(dn);
      if (dn_it == link_pos.end()) continue;
      const auto& p = ltm[dn_it->second];
      int rcap = add_ineq(p.qmax * dt);
      for (int yv : kv.second) ineq_trips.emplace_back(rcap, yv, 1.0);
      int rrecv = add_ineq(p.njam);
      for (int yv : kv.second) ineq_trips.emplace_back(rrecv, yv, 1.0);
      const int k_bw = std::max(0, k - p.tau_bw);
      for (std::size_t ari = 0; ari < active_route_flows.size(); ++ari) {
        const auto* route = active_route_flows[ari].route;
        if (!route) continue;
        for (std::size_t pi = 0; pi < route->link_indices.size(); ++pi) {
          if (route->link_indices[pi] != dn) continue;
          ineq_trips.emplace_back(rrecv, Nin[ari][pi][k - 1], 1.0);
          ineq_trips.emplace_back(rrecv, Nout[ari][pi][k_bw], -1.0);
        }
      }
    }

    for (std::size_t li = 0; li < problem.traffic.links.size(); ++li) {
      const int link_id = problem.traffic.links[li].index;
      const auto& p = ltm[li];
      for (int k = 1; k <= T; ++k) {
        int rcap = add_ineq(p.qmax * dt);
        int rrecv = add_ineq(p.njam);
        bool any = false;
        const int k_bw = std::max(0, k - p.tau_bw);

        for (const auto& kv : movement_vars) {
          const auto [up, dn, kk] = kv.first;
          (void)up;
          if (dn != link_id || kk != k) continue;
          any = true;
          for (int yv : kv.second) {
            ineq_trips.emplace_back(rcap, yv, 1.0);
            ineq_trips.emplace_back(rrecv, yv, 1.0);
          }
        }
        for (std::size_t ari = 0; ari < active_route_flows.size(); ++ari) {
          const auto* route = active_route_flows[ari].route;
          if (!route) continue;
          for (std::size_t pi = 0; pi < route->link_indices.size(); ++pi) {
            if (route->link_indices[pi] == link_id) {
              ineq_trips.emplace_back(rrecv, Nin[ari][pi][k - 1], 1.0);
              ineq_trips.emplace_back(rrecv, Nout[ari][pi][k_bw], -1.0);
            }
          }
          if (!route->link_indices.empty() &&
              route->link_indices[0] == link_id) {
            any = true;
            ineq_trips.emplace_back(rcap, Nin[ari][0][k], 1.0);
            ineq_trips.emplace_back(rcap, Nin[ari][0][k - 1], -1.0);
            ineq_trips.emplace_back(rrecv, Nin[ari][0][k], 1.0);
            ineq_trips.emplace_back(rrecv, Nout[ari][0][k_bw], -1.0);
          }
        }

        if (!any) {
          bineq_vec[static_cast<std::size_t>(rcap)] = 1.0e20;
          bineq_vec[static_cast<std::size_t>(rrecv)] = 1.0e20;
        }
      }
    }

    result.n_dynamic_flow_variables = 0;
    for (const auto& kv : movement_vars) {
      result.n_dynamic_flow_variables += static_cast<int>(kv.second.size());
    }
    for (const auto& n : Nin) {
      for (const auto& p : n) {
        result.n_dynamic_flow_variables += static_cast<int>(p.size());
      }
    }
    for (const auto& n : Nout) {
      for (const auto& p : n) {
        result.n_dynamic_flow_variables += static_cast<int>(p.size());
      }
    }
  }

  // ── Assemble and solve ─────────────────────────────────────────────────
  // n_vars is captured here, after all variables (including any BPR link-flow
  // and excess variables added in the road-capacity block) have been created.
  const int n_vars = static_cast<int>(vars.size());
  if (static_cast<int>(obj_vec.size()) != n_vars) {
    result.infeasible = true;
    result.status = "invalid model: objective/variable dimension mismatch";
    return result;
  }
  for (const auto& t : eq_trips) {
    if (t.col() < 0 || t.col() >= n_vars) {
      result.infeasible = true;
      result.status = "invalid model: equality triplet column out of range";
      return result;
    }
  }
  for (const auto& t : ineq_trips) {
    if (t.col() < 0 || t.col() >= n_vars) {
      result.infeasible = true;
      result.status = "invalid model: inequality triplet column out of range";
      return result;
    }
  }

  lp.c    = Eigen::VectorXd::Map(obj_vec.data(), n_vars);
  lp.vars = std::move(vars);

  if (!beq_vec.empty()) {
    lp.Aeq.resize(static_cast<int>(beq_vec.size()), n_vars);
    lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
    lp.Aeq.makeCompressed();
    lp.beq = Eigen::VectorXd::Map(beq_vec.data(), static_cast<Eigen::Index>(beq_vec.size()));
  }
  if (!bineq_vec.empty()) {
    lp.A.resize(static_cast<int>(bineq_vec.size()), n_vars);
    lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
    lp.A.makeCompressed();
    lp.b = Eigen::VectorXd::Map(bineq_vec.data(), static_cast<Eigen::Index>(bineq_vec.size()));
  }

  engine::BCOptions bc_opt;
  bc_opt.max_nodes       = 50000;
  bc_opt.time_limit_sec  = opts.time_limit_sec;
  bc_opt.gap_tol         = opts.mip_gap;
  bc_opt.use_simplex_lp_nodes = true;
  bc_opt.verbose         = false;
  auto bc_res = engine::solve_milp_bc(mip, bc_opt);

  result.solved     = bc_res.stats.success;
  result.infeasible = !result.solved;
  result.status     = bc_res.stats.status;
  result.objective  = result.solved ? bc_res.stats.objective : 0.0;
  result.best_bound = bc_res.bc_stats.best_bound;
  result.mip_gap = std::isfinite(bc_res.bc_stats.gap)
                       ? bc_res.bc_stats.gap
                       : bc_res.stats.mip_gap;

  if (!result.solved) return result;
  const auto& xv = bc_res.x;
  const auto cert = compute_solver_certificate_residuals(mip, xv);
  result.primal_max_violation = cert.primal_max_violation;
  result.integrality_max_violation = cert.integrality_max_violation;
  result.dynamic_flow_conservation_max_violation =
      opts.include_ltm_dynamic_constraints ? cert.primal_max_violation : 0.0;
  result.proven_optimal = result.solved &&
                          std::isfinite(result.mip_gap) &&
                          result.mip_gap <= opts.mip_gap + 1e-9 &&
                          result.primal_max_violation <= 1.0e-6 &&
                          result.integrality_max_violation <= 1.0e-6;

  for (int sid : station_ids) {
    result.station_open[sid]  = xv[z_var.at(sid)] > 0.5;
    result.station_plugs[sid] = static_cast<int>(std::round(xv[B_var.at(sid)]));
  }
  for (const auto& demand : problem.demands) {
    if (!h_idx.count(demand.index)) continue;
    for (const auto& [ridx, hv] : h_idx.at(demand.index))
      result.flow_by_demand_route[demand.index][ridx] = xv[hv];
  }

  return result;
}

}  // namespace hacdcpf::evpt
