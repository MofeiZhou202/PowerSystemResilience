// ctm_due_vi.cpp — DUE-VI: Frank-Wolfe CTM-DUE (Formulation F)
//
// Implements a Frank-Wolfe iterative method for Dynamic User Equilibrium
// using CTM as the traffic model.  Improvements over a plain MSA step:
//
//   1. Departure-window support: vehicles in `departure_window_steps` are spread
//      over all window steps during initialisation and AON assignment.
//   2. Golden-section line search: reduces the step-size error relative to the
//      classical MSA step 1/(k+1) by approximately minimising TSTT(x + α·d)
//      over [0,1]. Controlled by CTMDUEVIOptions::line_search_steps (0 = MSA).
//   3. Wardrop and VI residual metrics: the final solution reports Wardrop
//      relative gap, VI gap, normalised VI gap, complementarity residual, and
//      demand-conservation residual at the final CTM costs. These certify
//      fixed-point residuals, not LP/MILP-style dual bounds.
//   4. Multi-departure result packing: final flows are summed over all departure
//      steps in departure_window_steps, not just the fixed departure_step.

#include "evpt_internal.hpp"

namespace hacdcpf::evpt {

// Forward declarations (defined in ctm_propagation.cpp)
extern CTMSimulationResult ctm_forward_pass(
    const EVPowerTrafficProblem&,
    const std::vector<std::vector<double>>&,
    const std::unordered_map<int, std::size_t>&,
    const std::unordered_map<int, int>&,
    const CTMOptions&,
    double, int);

extern double wardrop_relative_gap(
    const EVPowerTrafficProblem&,
    const std::vector<std::vector<double>>&,
    const CTMSimulationResult&,
    const std::unordered_map<int, int>&,
    const EVPowerTrafficOptions&,
    const DUEOptions&,
    int, int);

extern std::vector<std::vector<double>> auxiliary_assignment(
    const EVPowerTrafficProblem&,
    const CTMSimulationResult&,
    const std::unordered_map<int, int>&,
    const std::unordered_map<int, std::size_t>&,
    const EVPowerTrafficOptions&,
    const DUEOptions&,
    int, int,
    const std::unordered_map<int, std::vector<double>>*);

// ICV-specific helpers (defined in ctm_propagation.cpp)
extern std::vector<std::vector<double>> auxiliary_assignment_icv(
    const EVPowerTrafficProblem&,
    const CTMSimulationResult&,
    const std::unordered_map<int, int>&,
    const std::unordered_map<int, std::size_t>&,
    int, int);

extern double wardrop_relative_gap_icv(
    const EVPowerTrafficProblem&,
    const std::vector<std::vector<double>>&,
    const CTMSimulationResult&,
    const std::unordered_map<int, int>&,
    const std::unordered_map<int, std::size_t>&,
    int, int);

CTMDUEResult solve_ctm_due_vi(
    const EVPowerTrafficProblem& problem,
    const EVPowerTrafficOptions& ev_opts,
    const CTMOptions& ctm_opts,
    const CTMDUEVIOptions& vi_opts)
{
  if (ev_opts.auto_generate_routes) {
    EVPowerTrafficOptions local_opts = ev_opts;
    local_opts.auto_generate_routes = false;
    const EVPowerTrafficProblem generated =
        maybe_generate_candidate_routes(problem, ev_opts);
    return solve_ctm_due_vi(generated, local_opts, ctm_opts, vi_opts);
  }

  CTMDUEResult result;
  if (ev_opts.require_exact_mathematical_model ||
      vi_opts.require_exact_mathematical_model) {
    result.converged = false;
    result.vi_certificate_available = false;
    result.mathematical_model_verified = false;
    result.mathematical_model_verification_status =
        "unsupported: CTM-DUE/Frank-Wolfe reports fixed-point residuals, not a "
        "globally certified full VI/MPEC mathematical-model proof";
    return result;
  }

  const double dt_sim = ev_opts.time_step_hr;
  const int T_sim = ev_opts.num_steps;
  if (T_sim <= 0 || dt_sim <= kTol) { result.converged = false; return result; }

  std::unordered_map<int, std::size_t> link_pos;
  for (std::size_t i = 0; i < problem.traffic.links.size(); ++i)
    link_pos[problem.traffic.links[i].index] = i;
  std::unordered_map<int, int> route_pos_map;
  const int n_routes = static_cast<int>(problem.routes.size());
  for (int ri = 0; ri < n_routes; ++ri)
    route_pos_map[problem.routes[static_cast<std::size_t>(ri)].index] = ri;

  const double dt_ctm = choose_ctm_dt(problem, ctm_opts, dt_sim);
  const int R     = std::max(1, static_cast<int>(std::round(dt_sim / dt_ctm)));
  const int T_ctm = T_sim * R;

  // ── Departure-step helper: returns all CTM departure steps for a demand ──
  auto dep_steps_for = [&](const EVDemand& demand) -> std::vector<int> {
    if (!demand.departure_window_steps.empty()) {
      return demand.departure_window_steps;
    }
    return {demand.departure_step};
  };

  auto full_due_aon = [&](const CTMSimulationResult& ctm) {
    std::vector<std::vector<double>> aux(
        static_cast<std::size_t>(T_ctm),
        std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));
    for (const auto& demand : problem.demands) {
      if (demand.vehicles <= kTol) continue;
      double best_cost = std::numeric_limits<double>::infinity();
      int best_k = -1;
      int best_rp = -1;
      for (int dep : dep_steps_for(demand)) {
        const int k_dep = dep * R;
        if (k_dep < 0 || k_dep >= T_ctm) continue;
        for (const auto* route : candidate_routes(problem, demand)) {
          if (!route || !route_pos_map.count(route->index)) continue;
          if (!route_soc_feasible(*route, demand, problem, link_pos)) continue;
          auto tt_it = ctm.route_travel_time.find(route->index);
          if (tt_it == ctm.route_travel_time.end() ||
              k_dep >= static_cast<int>(tt_it->second.size())) continue;
          const double tt = tt_it->second[static_cast<std::size_t>(k_dep)];
          if (!std::isfinite(tt)) continue;
          const double arr_hr = dep * ev_opts.time_step_hr + tt;
          double cost = ev_opts.value_of_time_per_hr * tt + route->toll_cost +
              schedule_delay_cost(arr_hr, demand.desired_arrival_time_hr,
                                  demand.early_penalty_per_hr,
                                  demand.late_penalty_per_hr);
          for (const auto& stop : route->charging_stops) {
            const double price = station_price(problem, ev_opts, stop.station_id, dep);
            cost += ev_opts.station_energy_cost_weight * price *
                    stop.requested_energy_kwh_per_vehicle;
            if (stop.v2g_capable &&
                stop.requested_discharge_energy_kwh_per_vehicle > kTol) {
              cost -= ev_opts.station_energy_cost_weight * price *
                      stop.requested_discharge_energy_kwh_per_vehicle;
            }
          }
          if (cost < best_cost - kTol) {
            best_cost = cost;
            best_k = k_dep;
            best_rp = route_pos_map.at(route->index);
          }
        }
      }
      if (best_k >= 0 && best_rp >= 0) {
        aux[static_cast<std::size_t>(best_k)][static_cast<std::size_t>(best_rp)] +=
            demand.vehicles;
      }
    }
    return aux;
  };

  // ── Initialise: equal split across departure steps and candidate routes ──
  std::vector<std::vector<double>> x(
      static_cast<std::size_t>(T_ctm),
      std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));
  for (const auto& demand : problem.demands) {
    if (demand.vehicles <= kTol) continue;
    std::vector<int> rpos;
    for (const auto& route : problem.routes)
      if (route.origin_node == demand.origin_node &&
          route.destination_node == demand.destination_node &&
          route_pos_map.count(route.index))
        rpos.push_back(route_pos_map.at(route.index));
    if (rpos.empty()) continue;
    const auto& steps = dep_steps_for(demand);
    const double share = demand.vehicles /
        (static_cast<double>(steps.size()) * static_cast<double>(rpos.size()));
    for (int dep : steps) {
      const int k_dep = dep * R;
      if (k_dep < 0 || k_dep >= T_ctm) continue;
      for (int rp : rpos)
        x[static_cast<std::size_t>(k_dep)][static_cast<std::size_t>(rp)] += share;
    }
  }

  DUEOptions due_opts_aon;
  due_opts_aon.max_iterations = 1;
  due_opts_aon.due_mode = DUEMode::FullEndogenous;

  // ── Mixed-fleet setup (include_icv_in_due) ────────────────────────────
  const bool run_icv = vi_opts.include_icv_in_due &&
                       ctm_opts.enable_multiclass &&
                       !problem.icv_demands.empty();

  const double pce_ev  = run_icv ? ctm_opts.ev_pce  : 1.0;
  const double pce_icv = run_icv ? ctm_opts.icv_pce : 1.0;

  // ICV flow matrix (same dimensions as EV; indexed by route, shared route set)
  std::vector<std::vector<double>> x_icv;
  if (run_icv) {
    x_icv.assign(static_cast<std::size_t>(T_ctm),
                 std::vector<double>(static_cast<std::size_t>(n_routes), 0.0));
    // Initialise: equal split per ICV demand across candidate routes
    for (const auto& demand : problem.icv_demands) {
      if (demand.vehicles <= kTol) continue;
      const int k_dep_ctm = demand.departure_step * R;
      if (k_dep_ctm < 0 || k_dep_ctm >= T_ctm) continue;
      std::vector<int> rpos;
      for (const auto& route : problem.routes)
        if (route.origin_node == demand.origin_node &&
            route.destination_node == demand.destination_node &&
            route_pos_map.count(route.index))
          rpos.push_back(route_pos_map.at(route.index));
      if (rpos.empty()) continue;
      const double share = demand.vehicles / static_cast<double>(rpos.size());
      for (int rp : rpos)
        x_icv[static_cast<std::size_t>(k_dep_ctm)][static_cast<std::size_t>(rp)] += share;
    }
  }

  // Helper: build PCE-weighted combined flow for ctm_forward_pass
  auto make_combined_flow = [&](const std::vector<std::vector<double>>& x_ev_in,
                                 const std::vector<std::vector<double>>& x_icv_in)
      -> std::vector<std::vector<double>> {
    if (!run_icv) return x_ev_in;
    auto combined = x_ev_in;
    for (int k = 0; k < T_ctm; ++k)
      for (int ri = 0; ri < n_routes; ++ri)
        combined[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] =
            pce_ev  * x_ev_in [static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +
            pce_icv * x_icv_in[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
    return combined;
  };

  // ── TSTT computation from CTM route travel times × route flows ─────────
  // Used by the golden-section line search to evaluate the objective function.
  auto compute_tstt = [&](const std::vector<std::vector<double>>& flows,
                           const CTMSimulationResult& ctm) -> double {
    double tstt = 0.0;
    for (const auto& demand : problem.demands) {
      if (demand.vehicles <= kTol) continue;
      const auto& steps = dep_steps_for(demand);
      for (int dep : steps) {
        const int k_dep = dep * R;
        if (k_dep < 0 || k_dep >= T_ctm) continue;
        for (const auto& route : problem.routes) {
          if (route.origin_node != demand.origin_node ||
              route.destination_node != demand.destination_node) continue;
          if (!route_pos_map.count(route.index)) continue;
          const int rp = route_pos_map.at(route.index);
          const double f = flows[static_cast<std::size_t>(k_dep)]
                                [static_cast<std::size_t>(rp)];
          if (f <= kTol) continue;
          double tt = 0.0;
          auto it = ctm.route_travel_time.find(route.index);
          if (it != ctm.route_travel_time.end() &&
              dep < static_cast<int>(it->second.size()))
            tt = it->second[static_cast<std::size_t>(dep)];
          tstt += f * tt;
        }
      }
    }
    return tstt;
  };

  CTMSimulationResult ctm_res;
  for (int iter = 1; iter <= vi_opts.max_iterations; ++iter) {
    const auto combined = make_combined_flow(x, x_icv);
    ctm_res = ctm_forward_pass(problem, combined, link_pos, route_pos_map,
                                ctm_opts, dt_ctm, T_ctm);
    const double rg_ev = wardrop_relative_gap(problem, x, ctm_res,
                                               route_pos_map, ev_opts,
                                               due_opts_aon, T_ctm, R);
    double rg = rg_ev;
    if (run_icv) {
      const double rg_icv = wardrop_relative_gap_icv(problem, x_icv, ctm_res,
                                                      route_pos_map, link_pos,
                                                      T_ctm, R);
      rg = std::max(rg_ev, rg_icv);
    }
    result.history.push_back({iter, rg, rg, rg});
    result.iterations = iter;
    result.gap = rg;
    result.relative_gap = rg;
    if (rg < vi_opts.convergence_tol) { result.converged = true; break; }

    // AON auxiliary assignment (FW direction) — EV
    const auto d = full_due_aon(ctm_res);

    // Frank-Wolfe direction for EV: d_FW = d - x
    auto dir = x;
    for (int k = 0; k < T_ctm; ++k)
      for (int ri = 0; ri < n_routes; ++ri)
        dir[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] =
            d[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] -
            x[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];

    // ICV auxiliary assignment and direction
    std::vector<std::vector<double>> dir_icv;
    if (run_icv) {
      const auto d_icv = auxiliary_assignment_icv(problem, ctm_res,
                                                   route_pos_map, link_pos,
                                                   T_ctm, R);
      dir_icv = x_icv;
      for (int k = 0; k < T_ctm; ++k)
        for (int ri = 0; ri < n_routes; ++ri)
          dir_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] =
              d_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] -
              x_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
    }

    // ── Step-size selection ────────────────────────────────────────────
    double alpha;
    if (vi_opts.line_search_steps > 0) {
      // Golden-section line search: minimise TSTT(x + α·d_FW) over α ∈ [0,1].
      // Each iteration of the search requires one extra CTM forward pass.
      // For a unimodal TSTT the golden-section method reduces the interval by
      // the golden ratio at every evaluation, achieving O(φ^n) convergence.
      const double phi = 1.0 - 0.618033988749895;  // ≈ 0.382 (1 - golden ratio)
      double lo = 0.0, hi = 1.0;

      auto make_flow = [&](double a) {
        auto xt = x;
        for (int k = 0; k < T_ctm; ++k)
          for (int ri = 0; ri < n_routes; ++ri)
            xt[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +=
                a * dir[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
        if (!run_icv) return xt;
        // Combined: PCE-weighted including updated ICV
        auto xt_icv = x_icv;
        for (int k = 0; k < T_ctm; ++k)
          for (int ri = 0; ri < n_routes; ++ri)
            xt_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +=
                a * dir_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
        return make_combined_flow(xt, xt_icv);
      };

      double a1 = lo + phi * (hi - lo);
      double a2 = hi - phi * (hi - lo);
      auto xf1 = make_flow(a1);
      auto xf2 = make_flow(a2);
      auto cf1 = ctm_forward_pass(problem, xf1, link_pos, route_pos_map,
                                   ctm_opts, dt_ctm, T_ctm);
      auto cf2 = ctm_forward_pass(problem, xf2, link_pos, route_pos_map,
                                   ctm_opts, dt_ctm, T_ctm);
      double f1 = compute_tstt(xf1, cf1);
      double f2 = compute_tstt(xf2, cf2);

      for (int ls = 0; ls < vi_opts.line_search_steps; ++ls) {
        if (f1 < f2) {
          hi = a2; a2 = a1; f2 = f1;
          a1 = lo + phi * (hi - lo);
          xf1 = make_flow(a1);
          cf1 = ctm_forward_pass(problem, xf1, link_pos, route_pos_map,
                                  ctm_opts, dt_ctm, T_ctm);
          f1 = compute_tstt(xf1, cf1);
        } else {
          lo = a1; a1 = a2; f1 = f2;
          a2 = hi - phi * (hi - lo);
          xf2 = make_flow(a2);
          cf2 = ctm_forward_pass(problem, xf2, link_pos, route_pos_map,
                                  ctm_opts, dt_ctm, T_ctm);
          f2 = compute_tstt(xf2, cf2);
        }
      }
      alpha = (lo + hi) / 2.0;
    } else {
      // Fallback: classical MSA step 1/(k+1)
      alpha = 1.0 / static_cast<double>(iter + 1);
    }

    // ── Flow update: x ← x + α · d_FW ───────────────────────────────────
    for (int k = 0; k < T_ctm; ++k)
      for (int ri = 0; ri < n_routes; ++ri)
        x[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +=
            alpha * dir[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
    if (run_icv) {
      for (int k = 0; k < T_ctm; ++k)
        for (int ri = 0; ri < n_routes; ++ri)
          x_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)] +=
              alpha * dir_icv[static_cast<std::size_t>(k)][static_cast<std::size_t>(ri)];
    }
  }

  const auto final_combined = make_combined_flow(x, x_icv);
  result.final_ctm = ctm_forward_pass(problem, final_combined, link_pos, route_pos_map,
                                       ctm_opts, dt_ctm, T_ctm);
  result.full_due_enabled = true;

  result.ev_relative_gap = wardrop_relative_gap(problem, x, result.final_ctm,
                                                route_pos_map, ev_opts,
                                                due_opts_aon, T_ctm, R);
  result.icv_relative_gap = 0.0;
  if (run_icv) {
    result.icv_relative_gap = wardrop_relative_gap_icv(
        problem, x_icv, result.final_ctm, route_pos_map, link_pos, T_ctm, R);
  }
  // compute_due_vi_certificate operates on EV flows (problem.demands) only.
  // When ICVs participate, the VI residuals reflect the EV sub-game; they
  // do not certify the full mixed-fleet Wardrop fixed point.
  const auto vi_cert = compute_due_vi_certificate(problem, ev_opts,
                                                  result.final_ctm,
                                                  route_pos_map, x, R);
  result.vi_certificate_covers_ev_only = run_icv;
  result.vi_certificate_available = vi_cert.available;
  result.vi_gap = vi_cert.vi_gap;
  result.normalized_vi_gap = vi_cert.normalized_vi_gap;
  result.complementarity_max_violation =
      vi_cert.complementarity_max_violation;
  result.demand_conservation_max_violation =
      vi_cert.demand_conservation_max_violation;
  if (vi_cert.available) {
    // In mixed-fleet mode, take the worst gap across both vehicle classes so
    // result.relative_gap is not misleadingly tight.
    const double worst_gap = run_icv
        ? std::max({result.ev_relative_gap,
                    result.icv_relative_gap,
                    vi_cert.normalized_vi_gap})
        : std::max(result.ev_relative_gap, vi_cert.normalized_vi_gap);
    result.relative_gap = std::max(result.relative_gap, worst_gap);
    result.gap = result.relative_gap;
    if (!result.history.empty()) {
      result.history.back().gap = result.relative_gap;
      result.history.back().relative_gap = result.relative_gap;
    }
  }

  // ── Pack route flows summed over all departure steps ───────────────────
  // For demands with departure_window_steps, the converged flow matrix x
  // carries positive values at each window step.  Sum them all so the
  // result correctly reflects the full demand assignment.
  for (const auto& demand : problem.demands) {
    const auto& steps = dep_steps_for(demand);
    for (const auto& route : problem.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      if (!route_pos_map.count(route.index)) continue;
      const int rp = route_pos_map.at(route.index);
      double total_flow = 0.0;
      for (int dep : steps) {
        const int k_dep = dep * R;
        if (k_dep < 0 || k_dep >= T_ctm) continue;
        const double dep_flow = x[static_cast<std::size_t>(k_dep)]
                                 [static_cast<std::size_t>(rp)];
        result.flow_by_demand_departure_route[demand.index][dep][route.index] =
            dep_flow;
        total_flow += dep_flow;
      }
      if (total_flow > kTol)
        result.flow_by_demand_route[demand.index][route.index] = total_flow;
    }
  }
  return result;
}

}  // namespace hacdcpf::evpt
