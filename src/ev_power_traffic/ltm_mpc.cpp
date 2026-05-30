// ltm_mpc.cpp
// ──────────────────────────────────────────────────────────────────────────
// LTM Model Predictive Control (Formulation H) — exact LTM within horizon
//
// Receding-horizon LP control of EV charging station admission.
// At each simulation step k = 0…T-1, solve an LP over [k, k+H):
//
//   min  Σ_{k'=k}^{k+H-1} [ ω_Q · Σ_s Q_{s,k'}
//                           + ω_T · Σ_a (Nin_{a,k'} − Nout_{a,k'}) · dt
//                           − ω_V2G · Σ_s P^v2g_{s,k'} ]
//   s.t.
//     Queue balance:   Q_{s,k'+1} = Q_{s,k'} + b_{s,k'} − c_{s,k'}
//     Service:         c_{s,k'} ≤ Q_{s,k'},   c_{s,k'} ≤ μ_s
//     Admission:       0 ≤ b_{s,k'} ≤ B̄_s
//     LTM Nin/Nout per access link a (one per station):
//       Nin_{a,k'+1} ≥ Nin_{a,k'}              (monotone)
//       Nout_{a,k'+1} ≥ Nout_{a,k'}             (monotone)
//       Nout_{a,k'} ≤ Nin_{a,max(0,k'-τ_ff)}   (LTM sending)
//       Nin_{a,k'} ≤ Nout_{a,max(0,k'-τ_bw)} + N_jam_a   (LTM receiving)
//       Nin_{a,k'+1} − Nin_{a,k'} ≤ qmax_a · dt          (inflow cap)
//       Nout_{a,k'+1} − Nout_{a,k'} ≤ qmax_a · dt        (outflow cap)
//       b_{s,k'} ≤ Nout_{a,k'} − Nout_{a,k'-1}           (admission bounded
//                                                           by LTM arrivals)
//     V2G–SOC balance:
//       SOC_{s,k'+1} = SOC_{s,k'} + b_{s,k'} · E_dwell
//                     − c_{s,k'} · E_dwell − P^v2g_{s,k'} · dt
//       P^v2g_{s,k'} ≤ SOC_{s,k'} · η_discharge
//       SOC_{s,k'} ≥ 0
//
// Only first-step decisions b_{s,k} are applied; the loop repeats at k+1.
// ──────────────────────────────────────────────────────────────────────────

#include "evpt_internal.hpp"

#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace hacdcpf::evpt {

LTMMPCResult solve_ltm_mpc(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_opts,
    const LTMMPCOptions& mpc_opts)
{
  if (ev_opts.auto_generate_routes) {
    EVPowerTrafficOptions local_opts = ev_opts;
    local_opts.auto_generate_routes = false;
    const EVPowerTrafficProblem generated =
        maybe_generate_candidate_routes(problem, ev_opts);
    return solve_ltm_mpc(generated, local_opts, mpc_opts);
  }

  LTMMPCResult result;

  if (ev_opts.require_exact_mathematical_model ||
      mpc_opts.require_exact_mathematical_model) {
    result.solved = false;
    result.proven_optimal = false;
    result.status =
        "unsupported: exact full-network LTM-MPC mathematical-model "
        "verification is not available for the forecast-driven MPC path";
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: LTM-MPC is receding-horizon and forecast-driven, so it is "
        "not a 100% certificate for the full dynamic joint model";
    return result;
  }

  const int T  = ev_opts.num_steps;
  const double dt = ev_opts.time_step_hr;
  if (T <= 0 || dt <= kTol) {
    result.status = "empty";
    return result;
  }

  const int H = std::max(1, std::min(mpc_opts.horizon_steps, T));
  result.forecast_arrivals_enforced = mpc_opts.enforce_forecast_arrivals;
  result.forecast_arrivals =
      forecast_station_arrivals_from_demands(problem, ev_opts);

  // ── Build link index map ────────────────────────────────────────────────
  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t i = 0; i < problem.traffic.links.size(); ++i)
    link_pos[problem.traffic.links[i].index] = i;

  // ── Station model (one virtual "access link" per station) ───────────────
  // The access link represents the last road segment vehicles traverse before
  // arriving at the station.  Its LTM parameters approximate the network
  // approach conditions.  When a real link is identified from the routes
  // (last link before charging stop), we use its parameters; otherwise we
  // use a short default link.
  //
  // When mpc_opts.track_all_route_links = true, ALL links on the route leading
  // to each station are tracked with separate Nin/Nout LTM variables.  The VHT
  // objective sums ω_T·dt·(Nin[a,j]−Nout[a,j]) across all tracked links.
  // The admission constraint still uses only the final access link.

  struct LinkLTMParams {
    int    link_id{-1};
    double qmax_dt{1.0};
    double njam{10.0};
    int    tau_ff{1};
    int    tau_bw{1};
  };

  auto link_ltm_params_from_id = [&](int link_id) -> LinkLTMParams {
    LinkLTMParams p;
    p.link_id = link_id;
    if (link_id >= 0 && link_pos.count(link_id)) {
      const auto& lnk = problem.traffic.links[link_pos.at(link_id)];
      const double vf   = link_free_flow_speed(lnk);
      const double len  = std::max(kTol, lnk.length_km);
      const double qmax = lnk.capacity_veh_per_hr > kTol
                              ? lnk.capacity_veh_per_hr : vf * 0.1;
      const double kj   = lnk.jam_vehicles > kTol ? lnk.jam_vehicles / len
                                                   : qmax / vf * 2.0;
      const double kc   = qmax / vf;
      const double w    = kj > kc + kTol ? qmax / (kj - kc) : vf * 0.25;
      p.qmax_dt = qmax * dt;
      p.njam    = std::max(1.0, kj * len);
      p.tau_ff  = std::max(1, static_cast<int>(std::round(len / (vf * dt))));
      p.tau_bw  = std::max(1, static_cast<int>(std::round(len / (w  * dt))));
    } else {
      // Default short access link: 0.5 km, 50 km/hr free flow
      p.qmax_dt = 50.0 * 0.1 * dt;
      p.njam    = 10.0;
      p.tau_ff  = std::max(1, static_cast<int>(std::round(0.5 / (50.0 * dt))));
      p.tau_bw  = std::max(1, static_cast<int>(std::round(0.5 / (12.5 * dt))));
    }
    return p;
  };

  struct StationMPC {
    int id{0};
    double plug_cap{0.0};         // max simultaneous vehicles in queue / charge
    double service_rate{1.0};     // μ_s: vehicles served per step
    double v2g_max{0.0};          // max V2G power [kW]
    double q_init{0.0};           // initial queue (from prior horizon)
    // Access link (last on route) — used for admission constraint
    double qmax_dt{1.0};          // capacity * dt [vehicles/step]
    double njam{10.0};            // jam occupancy [vehicles]
    int    tau_ff{1};             // free-flow lag [steps]
    int    tau_bw{1};             // backward-wave lag [steps]
    double energy_per_veh{1.0};   // E_dwell: kWh charged per vehicle dwell
    double discharge_fraction{0.8}; // η_discharge: V2G limit = SOC * this
    // All route links (including access link) when track_all_route_links=true
    std::vector<LinkLTMParams> route_links;
  };
  std::vector<StationMPC> stations;
  std::unordered_set<int> seen_sids;
  for (const auto& route : problem.routes) {
    for (const auto& stop : route.charging_stops) {
      if (seen_sids.count(stop.station_id)) continue;
      seen_sids.insert(stop.station_id);
      StationMPC sm;
      sm.id           = stop.station_id;
      sm.plug_cap     = std::max(1.0, static_cast<double>(stop.dwell_steps));
      sm.service_rate = sm.plug_cap;
      sm.v2g_max      = stop.max_discharge_kw_per_vehicle;
      sm.q_init       = 0.0;
      sm.energy_per_veh =
          stop.max_discharge_kw_per_vehicle * dt *
          std::max(1.0, static_cast<double>(stop.dwell_steps));

      // Find the last link on this route before the stop to infer LTM params
      int access_link_id = -1;
      const auto& stop_idx_in_route = stop.station_id;  // used as a key below
      (void)stop_idx_in_route;
      if (!route.link_indices.empty())
        access_link_id = route.link_indices.back();

      const auto access_params = link_ltm_params_from_id(access_link_id);
      sm.qmax_dt = access_params.qmax_dt;
      sm.njam    = access_params.njam;
      sm.tau_ff  = access_params.tau_ff;
      sm.tau_bw  = access_params.tau_bw;

      // When track_all_route_links, collect LTM params for every route link
      if (mpc_opts.track_all_route_links) {
        for (int lid : route.link_indices)
          sm.route_links.push_back(link_ltm_params_from_id(lid));
      } else {
        // Single-link mode: just the access link
        sm.route_links.push_back(access_params);
      }
      sm.discharge_fraction = 0.8;
      stations.push_back(sm);
      result.admission[sm.id].assign(static_cast<std::size_t>(T), 0.0);
    }
  }
  const int n_s = static_cast<int>(stations.size());
  if (n_s == 0) {
    result.status = "no_stations";
    result.solved = true;
    return result;
  }

  // ── Persistent state across receding horizons ───────────────────────────
  std::vector<double> q_now(static_cast<std::size_t>(n_s), 0.0);
  // nin_now[si][li]: cumulative counts at current step k, per route link li
  // In single-link mode (track_all_route_links=false), li=0 is the access link
  std::vector<std::vector<double>> nin_now(static_cast<std::size_t>(n_s));
  std::vector<std::vector<double>> nout_now(static_cast<std::size_t>(n_s));
  for (int si = 0; si < n_s; ++si) {
    const int nl = static_cast<int>(stations[si].route_links.size());
    nin_now[si].assign(static_cast<std::size_t>(nl), 0.0);
    nout_now[si].assign(static_cast<std::size_t>(nl), 0.0);
  }
  // soc_now[si]: station energy storage [kWh]
  std::vector<double> soc_now(static_cast<std::size_t>(n_s), 0.0);
  std::vector<double> forecast_used(static_cast<std::size_t>(n_s), 0.0);

  double total_obj = 0.0;
  bool all_ok = true;

  for (int k = 0; k < T; ++k) {
    const int h_end = std::min(k + H, T);
    const int hlen  = h_end - k;

    // ── LP for horizon [k, k+hlen) ─────────────────────────────────────
    engine::MIPModel mip;
    engine::LPModel& lp = mip.linear_part;
    lp.sense = engine::Sense::Minimize;

    std::vector<engine::VariableMeta> vars;
    std::vector<double> obj_vec;
    std::vector<Eigen::Triplet<double>> eq_trips, ineq_trips;
    std::vector<double> beq_vec, bineq_vec;

    auto add_var = [&](double lb, double ub, double cost) -> int {
      const int idx = static_cast<int>(vars.size());
      vars.push_back({engine::VarType::Continuous, lb, ub, {}});
      obj_vec.push_back(cost);
      return idx;
    };
    auto add_eq   = [&](double rhs) -> int {
      const int r = static_cast<int>(beq_vec.size()); beq_vec.push_back(rhs); return r; };
    auto add_ineq = [&](double rhs) -> int {
      const int r = static_cast<int>(bineq_vec.size()); bineq_vec.push_back(rhs); return r; };

    const double w_q   = mpc_opts.weight_queue;
    const double w_t   = mpc_opts.weight_travel_time;
    const double w_v2g = mpc_opts.weight_v2g;

    // Variables per station: b[j], c[j], Q[j] (j=0..hlen), v2g[j], SOC[j]
    std::vector<std::vector<int>> b_v(n_s, std::vector<int>(hlen));
    std::vector<std::vector<int>> c_v(n_s, std::vector<int>(hlen));
    std::vector<std::vector<int>> Q_v(n_s, std::vector<int>(hlen + 1));
    std::vector<std::vector<int>> v2g_v(n_s, std::vector<int>(hlen));
    std::vector<std::vector<int>> soc_v(n_s, std::vector<int>(hlen + 1));
    // LTM cumulative counts Nin[si][li][j] and Nout[si][li][j] (j=0..hlen)
    // li indexes route_links[li]; li = route_links.size()-1 is always access link
    using vec2i = std::vector<std::vector<int>>;
    std::vector<vec2i> Nin_v(n_s);   // [si][li*(hlen+1) + j]
    std::vector<vec2i> Nout_v(n_s);

    for (int si = 0; si < n_s; ++si) {
      const auto& sm = stations[static_cast<std::size_t>(si)];
      const int nl = static_cast<int>(sm.route_links.size());
      const double qcap = sm.plug_cap * static_cast<double>(hlen + 1);

      // Q[0]: fixed to q_now
      Q_v[si].resize(hlen + 1);
      Q_v[si][0]   = add_var(0.0, qcap, 0.0);
      soc_v[si].resize(hlen + 1);
      soc_v[si][0] = add_var(0.0, qcap * sm.energy_per_veh, 0.0);

      // Nin/Nout per route link
      Nin_v[si].resize(static_cast<std::size_t>(nl), std::vector<int>(hlen + 1, -1));
      Nout_v[si].resize(static_cast<std::size_t>(nl), std::vector<int>(hlen + 1, -1));
      for (int li = 0; li < nl; ++li) {
        const auto& lp2 = sm.route_links[static_cast<std::size_t>(li)];
        Nin_v[si][li][0]  = add_var(0.0, nin_now[si][li]  + lp2.qmax_dt * hlen * 2, 0.0);
        Nout_v[si][li][0] = add_var(0.0, nout_now[si][li] + lp2.qmax_dt * hlen * 2, 0.0);
        for (int j = 0; j < hlen; ++j) {
          const double nin_ub  = nin_now[si][li]  + lp2.qmax_dt * (hlen + 1) * 2;
          const double nout_ub = nout_now[si][li] + lp2.qmax_dt * (hlen + 1) * 2;
          // VHT objective applies to all tracked route links
          Nin_v[si][li][j+1]  = add_var(0.0, nin_ub,  w_t * dt);
          Nout_v[si][li][j+1] = add_var(0.0, nout_ub, -w_t * dt);
        }
      }

      b_v[si].resize(hlen);
      c_v[si].resize(hlen);
      v2g_v[si].resize(hlen);
      for (int j = 0; j < hlen; ++j) {
        b_v[si][j]      = add_var(0.0, sm.plug_cap, 0.0);
        c_v[si][j]      = add_var(0.0, sm.service_rate, 0.0);
        v2g_v[si][j]    = add_var(0.0, sm.v2g_max * sm.plug_cap, -w_v2g);
        Q_v[si][j + 1]  = add_var(0.0, qcap, w_q);
        soc_v[si][j+1]  = add_var(0.0, qcap * sm.energy_per_veh, 0.0);
      }
    }
    const int n_vars = static_cast<int>(vars.size());

    for (int si = 0; si < n_s; ++si) {
      const auto& sm = stations[static_cast<std::size_t>(si)];
      const int nl = static_cast<int>(sm.route_links.size());
      // access link is always the last route link
      const int ali = nl - 1;

      // Fix Q[0], SOC[0], Nin[0], Nout[0] to current known values
      { auto fix = [&](int v, double val) {
          const int r = add_eq(val);
          eq_trips.emplace_back(r, v, 1.0); };
        fix(Q_v[si][0],    q_now[si]);
        fix(soc_v[si][0],  soc_now[si]);
        for (int li = 0; li < nl; ++li) {
          fix(Nin_v[si][li][0],  nin_now[si][li]);
          fix(Nout_v[si][li][0], nout_now[si][li]);
        }
      }

      for (int j = 0; j < hlen; ++j) {
        const int bi   = b_v[si][j];
        const int ci   = c_v[si][j];
        const int Qj   = Q_v[si][j];
        const int Qj1  = Q_v[si][j + 1];
        const int Vi   = v2g_v[si][j];
        const int Si   = soc_v[si][j];
        const int Si1  = soc_v[si][j + 1];
        const double E = sm.energy_per_veh;

        // Queue balance: Q[j+1] = Q[j] + b - c
        { const int r = add_eq(0.0);
          eq_trips.emplace_back(r, Qj1, 1.0);
          eq_trips.emplace_back(r, Qj, -1.0);
          eq_trips.emplace_back(r, bi, -1.0);
          eq_trips.emplace_back(r, ci,  1.0); }

        // Service ≤ queue: c ≤ Q[j]
        { const int r = add_ineq(0.0);
          ineq_trips.emplace_back(r, ci,  1.0);
          ineq_trips.emplace_back(r, Qj, -1.0); }

        // SOC balance: SOC[j+1] = SOC[j] + b·E − c·E − V2G·dt
        { const int r = add_eq(0.0);
          eq_trips.emplace_back(r, Si1,  1.0);
          eq_trips.emplace_back(r, Si,  -1.0);
          eq_trips.emplace_back(r, bi,  -E);
          eq_trips.emplace_back(r, ci,   E);
          eq_trips.emplace_back(r, Vi,   dt); }

        // V2G ≤ SOC · discharge_fraction (strict V2G-SOC linkage)
        { const int r = add_ineq(0.0);
          ineq_trips.emplace_back(r, Vi,  1.0);
          ineq_trips.emplace_back(r, Si, -sm.discharge_fraction); }

        // LTM constraints for ALL tracked route links
        for (int li = 0; li < nl; ++li) {
          const auto& lp2 = sm.route_links[static_cast<std::size_t>(li)];
          const int Ni   = Nin_v[si][li][j];
          const int Ni1  = Nin_v[si][li][j + 1];
          const int NOi  = Nout_v[si][li][j];
          const int NOi1 = Nout_v[si][li][j + 1];

          // LTM monotone: Nin[j+1] ≥ Nin[j], Nout[j+1] ≥ Nout[j]
          { int r = add_ineq(0.0);
            ineq_trips.emplace_back(r, Ni,   1.0);
            ineq_trips.emplace_back(r, Ni1, -1.0); }
          { int r = add_ineq(0.0);
            ineq_trips.emplace_back(r, NOi,   1.0);
            ineq_trips.emplace_back(r, NOi1, -1.0); }

          // LTM sending: Nout[j] ≤ Nin[j − τ_ff]
          { const int j_lag = j - lp2.tau_ff;
            const int Ni_lag = j_lag >= 0 ? Nin_v[si][li][j_lag] : Nin_v[si][li][0];
            const int r = add_ineq(j_lag < 0 ? nin_now[si][li] : 0.0);
            ineq_trips.emplace_back(r, NOi1, 1.0);
            ineq_trips.emplace_back(r, Ni_lag, -1.0); }

          // LTM receiving: Nin[j] ≤ Nout[j − τ_bw] + N_jam
          { const int j_lag = j - lp2.tau_bw;
            const int NO_lag = j_lag >= 0 ? Nout_v[si][li][j_lag] : Nout_v[si][li][0];
            const double rhs = lp2.njam + (j_lag < 0 ? nout_now[si][li] : 0.0);
            const int r = add_ineq(rhs);
            ineq_trips.emplace_back(r, Ni1,    1.0);
            ineq_trips.emplace_back(r, NO_lag, -1.0); }

          // LTM inflow/outflow capacity per step
          { int r = add_ineq(lp2.qmax_dt);
            ineq_trips.emplace_back(r, Ni1,  1.0);
            ineq_trips.emplace_back(r, Ni,  -1.0); }
          { int r = add_ineq(lp2.qmax_dt);
            ineq_trips.emplace_back(r, NOi1,  1.0);
            ineq_trips.emplace_back(r, NOi,  -1.0); }
        }

        // Admission bounded by vehicles completing ACCESS link (last link)
        //   b[j] ≤ Nout_access[j+1] − Nout_access[j]
        { const int NOi_a  = Nout_v[si][ali][j];
          const int NOi1_a = Nout_v[si][ali][j + 1];
          int r = add_ineq(0.0);
          ineq_trips.emplace_back(r, bi,     1.0);
          ineq_trips.emplace_back(r, NOi1_a, -1.0);
          ineq_trips.emplace_back(r, NOi_a,   1.0); }

        // Exogenous OD forecast bound (access link entry rate)
        if (mpc_opts.enforce_forecast_arrivals) {
          const int global_step = k + j;
          double forecast_remaining = 0.0;
          auto fit = result.forecast_arrivals.find(sm.id);
          if (fit != result.forecast_arrivals.end() &&
              global_step < static_cast<int>(fit->second.size())) {
            forecast_remaining =
                fit->second[static_cast<std::size_t>(global_step)];
          }
          if (j == 0) {
            forecast_remaining = std::max(
                0.0, forecast_remaining - forecast_used[si]);
          }
          // Constrain the access-link entry rate
          const int Ni_a  = Nin_v[si][ali][j];
          const int Ni1_a = Nin_v[si][ali][j + 1];
          const int r = add_ineq(forecast_remaining);
          ineq_trips.emplace_back(r, Ni1_a,  1.0);
          ineq_trips.emplace_back(r, Ni_a,  -1.0);
        }
      }
    }

    // Assemble LP
    lp.c    = Eigen::VectorXd::Map(obj_vec.data(), n_vars);
    lp.vars = std::move(vars);

    if (!beq_vec.empty()) {
      lp.Aeq.resize(static_cast<int>(beq_vec.size()), n_vars);
      lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
      lp.Aeq.makeCompressed();
      lp.beq = Eigen::VectorXd::Map(beq_vec.data(),
          static_cast<Eigen::Index>(beq_vec.size()));
    }
    if (!bineq_vec.empty()) {
      lp.A.resize(static_cast<int>(bineq_vec.size()), n_vars);
      lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
      lp.A.makeCompressed();
      lp.b = Eigen::VectorXd::Map(bineq_vec.data(),
          static_cast<Eigen::Index>(bineq_vec.size()));
    }

    engine::SolveResult solve_result;
    engine::HighsAdapter highs;
    if (highs.available()) {
      solve_result = highs.solve_lp(lp);
    }
    if (!solve_result.stats.success) {
      engine::SimplexOptions simp_opt;
      simp_opt.max_iter = 20000;
      simp_opt.feasibility_tol = 1e-8;
      simp_opt.optimality_tol  = 1e-8;
      simp_opt.verbose = false;
      auto lp_result = engine::solve_lp_with_basis(lp, simp_opt, nullptr);
      solve_result = lp_result.result;
    }

    const bool ok = solve_result.stats.success;
    if (!ok) { all_ok = false; }
    else {
      const auto cert =
          compute_solver_certificate_residuals(mip, solve_result.x);
      result.primal_max_violation =
          std::max(result.primal_max_violation, cert.primal_max_violation);
      result.integrality_max_violation =
          std::max(result.integrality_max_violation,
                   cert.integrality_max_violation);
      const auto& xv = solve_result.x;
      total_obj += xv.size() == lp.c.size() ? lp.c.dot(xv)
                                            : solve_result.stats.objective;
      for (int si = 0; si < n_s; ++si) {
        const auto& sm = stations[static_cast<std::size_t>(si)];
        const int nl  = static_cast<int>(sm.route_links.size());
        const int ali = nl - 1;
        const double b_k  = xv[b_v[si][0]];
        const double c_k  = xv[c_v[si][0]];
        const double dn_in  = xv[Nin_v[si][ali][1]]  - xv[Nin_v[si][ali][0]];
        const double dn_out = xv[Nout_v[si][ali][1]] - xv[Nout_v[si][ali][0]];
        const double dv2g   = xv[v2g_v[si][0]];
        const int sid = sm.id;
        result.admission[sid][static_cast<std::size_t>(k)] = b_k;
        if (mpc_opts.enforce_forecast_arrivals) {
          double forecast_k = 0.0;
          auto fit = result.forecast_arrivals.find(sid);
          if (fit != result.forecast_arrivals.end() &&
              k < static_cast<int>(fit->second.size())) {
            forecast_k = fit->second[static_cast<std::size_t>(k)];
          }
          result.forecast_bound_max_violation = std::max(
              result.forecast_bound_max_violation,
              std::max(0.0, dn_in - std::max(0.0, forecast_k - forecast_used[si])));
          forecast_used[si] += dn_in;
        }
        // Advance persistent state
        q_now  [si] = std::max(0.0, xv[Q_v[si][1]]);
        soc_now[si] = std::max(0.0, xv[soc_v[si][1]]);
        for (int li = 0; li < nl; ++li) {
          nin_now[si][li]  += xv[Nin_v[si][li][1]]  - xv[Nin_v[si][li][0]];
          nout_now[si][li] += xv[Nout_v[si][li][1]] - xv[Nout_v[si][li][0]];
        }
        (void)c_k; (void)dv2g; (void)dn_out;
      }
    }
  }

	  result.solved          = all_ok;
	  result.status          = all_ok ? "ok" : "partial";
	  result.total_objective = total_obj;
	  result.best_bound      = total_obj;
	  result.mip_gap         = 0.0;
	  result.proven_optimal  = result.solved &&
	                           result.primal_max_violation <= 1.0e-7 &&
	                           result.integrality_max_violation <= 1.0e-7;

	  return result;
	}

}  // namespace hacdcpf::evpt
