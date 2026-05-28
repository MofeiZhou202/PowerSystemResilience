/// ctm_joint_welfare.cpp
/// =============================================================================
/// Formulation C: CTM-DUE jointly coupled with DC-OPF via iterative LMP price
/// coordination.
///
/// Algorithm
/// ----------
///   Outer loop (price-coordination, mirrors the JSW algorithm for Formulation A
///   but replaces the system-optimal LP inner step with CTM-DUE):
///
///   Initialise station prices π^{(0)}.
///   For i = 1 .. max_iterations:
///     (a) Traffic step  — run CTM-DUE with current prices π^{(i-1)}
///                          → DUE route flows x^{(i)}, session synthesis,
///                             smart-charging dispatch, station EV loads P^EV.
///     (b) Power step    — for each simulation step k:
///                          build system with EV loads P^EV_k;
///                          solve DC-OPF; extract LMPs λ^{(i)}_{b,k}.
///     (c) Price update  — π^{(i)}_{s,k} = α · (λ_{bus(s),k}/1000)
///                                         + (1-α) · π^{(i-1)}_{s,k}
///         (LMP is in $/MWh; divide by 1000 → $/kWh)
///     (d) Convergence   — stop when max |π^{(i)} - π^{(i-1)}| < tol.
///
///   Social welfare:
///     W = B_EV - C_gen - C_delay
///     B_EV   = Σ_d  WTP_d · vehicles_d                    (consumer benefit)
///     C_gen  = Σ_k  OPF_objective_k                       (dispatch cost)
///     C_delay= VOT · TSTT^{CTM}   where TSTT = Σ_{d,r} x_{d,r} · T^CTM_{r,k}
/// =============================================================================

#include "hacdcpf/ev_power_traffic/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"

namespace hacdcpf::evpt {

namespace {

constexpr double kWh_per_MWh = 1000.0;
constexpr double kW_per_MW   = 1000.0;

// ─────────────────────────────────────────────────────────────────────────────
// Build station_id → bus_number map from the power system model.
// ─────────────────────────────────────────────────────────────────────────────
std::unordered_map<int, int> station_bus_map(const EVPowerTrafficProblem& prob) {
  std::unordered_map<int, int> m;
  for (const auto& cs : prob.system.ac.charging_stations) {
    m[cs.index] = cs.bus;
  }
  return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// Write station prices back into problem.station_prices from the price matrix.
// ─────────────────────────────────────────────────────────────────────────────
void set_station_prices(EVPowerTrafficProblem& prob,
                        const std::unordered_map<int, std::vector<double>>& prices) {
  prob.station_prices.clear();
  for (const auto& [sid, vec] : prices) {
    StationPriceProfile p;
    p.station_id   = sid;
    p.price_per_kwh = vec;
    prob.station_prices.push_back(p);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Copy the base system and inject EV loads from the CTM dispatch result.
// station_ev_load_kw[station_id][step] → add to bus pd_mw at that step.
// ─────────────────────────────────────────────────────────────────────────────
HybridPowerSystem system_with_ctm_ev_loads(
    const HybridPowerSystem& base,
    const std::unordered_map<int, std::vector<double>>& ev_load_kw,
    const std::unordered_map<int, int>& sbus,
    int step) {
  HybridPowerSystem sys = base;
  for (const auto& [sid, load_vec] : ev_load_kw) {
    if (step < 0 || step >= static_cast<int>(load_vec.size())) continue;
    const double p_kw = load_vec[static_cast<std::size_t>(step)];
    if (!sbus.count(sid)) continue;
    const int bus_id = sbus.at(sid);
    const double ev_mw = p_kw / kW_per_MW;
    for (auto& b : sys.ac.buses) {
      if (b.index == bus_id) { b.pd_mw += ev_mw; break; }
    }
  }
  return sys;
}

// ─────────────────────────────────────────────────────────────────────────────
// Compute DUE total system travel time from CTM-DUE result.
// TSTT = Σ_{d,r} x_{d,r} · T^CTM_{r, k_dep_ctm}
// ─────────────────────────────────────────────────────────────────────────────
double compute_tstt_ctm(const EVPowerTrafficProblem& prob,
                        const CTMDUEResult& due_res,
                        int R) {
  double tstt = 0.0;
  for (const auto& demand : prob.demands) {
    const int k_dep_ctm = demand.departure_step * R;
    for (const auto& route : prob.routes) {
      if (route.origin_node != demand.origin_node ||
          route.destination_node != demand.destination_node) continue;
      double vehicles = 0.0;
      if (due_res.flow_by_demand_route.count(demand.index)) {
        const auto& rm = due_res.flow_by_demand_route.at(demand.index);
        if (rm.count(route.index)) vehicles = rm.at(route.index);
      }
      if (vehicles <= 0.0) continue;
      double tt = 0.0;
      if (due_res.final_ctm.route_travel_time.count(route.index)) {
        const auto& tv = due_res.final_ctm.route_travel_time.at(route.index);
        const int idx = std::min(k_dep_ctm, static_cast<int>(tv.size()) - 1);
        if (idx >= 0) tt = tv[static_cast<std::size_t>(idx)];
      }
      tstt += vehicles * tt;
    }
  }
  return tstt;
}

// ─────────────────────────────────────────────────────────────────────────────
// Compute gross EV consumer benefit = Σ_d WTP_d × vehicles_dispatched_d
// "dispatched" = vehicles that received non-zero energy in at least one session.
// ─────────────────────────────────────────────────────────────────────────────
double ev_gross_benefit(const EVPowerTrafficProblem& prob,
                        const CTMDUEResult& due_res) {
  std::unordered_map<int, double> wtp;
  for (const auto& d : prob.demands)
    wtp[d.index] = d.willingness_to_pay_per_vehicle;

  // Served vehicles per demand = demand flow that has at least one session
  // delivering energy.
  std::unordered_map<int, double> served;
  for (const auto& d : prob.demands) {
    double v = 0.0;
    if (due_res.flow_by_demand_route.count(d.index)) {
      for (const auto& [rid, x] : due_res.flow_by_demand_route.at(d.index))
        v += x;
    }
    served[d.index] = v;
  }

  double benefit = 0.0;
  for (const auto& [did, v] : served) {
    const double w = wtp.count(did) ? wtp.at(did) : 0.0;
    benefit += w * v;
  }
  return benefit;
}

}  // anonymous namespace


// =============================================================================
// simulate_ev_power_traffic_ctm_joint — Formulation C public entry point
// =============================================================================

CTMJointWelfareResult simulate_ev_power_traffic_ctm_joint(
    EVPowerTrafficProblem   problem,
    const CTMJointWelfareOptions& options) {

  CTMJointWelfareResult out;
  out.status = "initialised";

  const int    T     = options.evpt_opts.num_steps;
  const double alpha = std::clamp(options.price_update_step, 0.0, 1.0);

  // ── Collect station IDs ────────────────────────────────────────────────────
  std::vector<int> sta_ids;
  {
    std::unordered_map<int, bool> seen;
    auto add = [&](int id) {
      if (id && !seen.count(id)) { seen[id] = true; sta_ids.push_back(id); }
    };
    for (const auto& cs : problem.system.ac.charging_stations) add(cs.index);
    for (const auto& r  : problem.routes)
      for (const auto& s : r.charging_stops) add(s.station_id);
    std::sort(sta_ids.begin(), sta_ids.end());
  }

  const auto sbus = station_bus_map(problem);
  const double default_price = options.evpt_opts.default_station_price_per_kwh;

  // ── Initialise station prices from problem or default ──────────────────────
  std::unordered_map<int, std::vector<double>> cur_prices;
  for (int sid : sta_ids) {
    bool found = false;
    for (const auto& p : problem.station_prices) {
      if (p.station_id == sid && !p.price_per_kwh.empty()) {
        cur_prices[sid] = p.price_per_kwh;
        cur_prices[sid].resize(static_cast<std::size_t>(T), default_price);
        found = true;
        break;
      }
    }
    if (!found)
      cur_prices[sid].assign(static_cast<std::size_t>(T), default_price);
  }

  out.lmp_by_step.resize(static_cast<std::size_t>(T));

  // CTM time grid ratio (needed for TSTT computation)
  // We cannot compute R here without running the CTM, so we approximate as 1
  // and let each CTM-DUE call handle it internally.  TSTT is computed from
  // due_res.final_ctm.route_travel_time at departure step 0 (R=1 approximation).
  // For accurate TSTT we query it from the result via compute_tstt_ctm(R=1).

  // ── Outer iteration loop ───────────────────────────────────────────────────
  CTMDUEResult best_due;

  for (int iter = 1; iter <= options.max_iterations; ++iter) {

    // ── (a) CTM-DUE inner step with current prices ───────────────────────────
    set_station_prices(problem, cur_prices);

    EVPowerTrafficOptions evpt_opts = options.evpt_opts;
    const CTMDUEResult due_res = simulate_ev_power_traffic_ctm_due(
        problem, evpt_opts, options.ctm_opts, options.due_opts);

    // Estimate R from dt_sim / dt_ctm for TSTT computation.
    // Since dt_ctm is chosen inside simulate_ev_power_traffic_ctm_due, we
    // derive R from the final_ctm step count relative to T.
    const int T_ctm_final =
        static_cast<int>(due_res.final_ctm.step_link_results.size());
    const int R_est = (T > 0 && T_ctm_final >= T)
                          ? T_ctm_final / T
                          : 1;

    // ── (b) Power step: DC-OPF for each simulation time step ─────────────────
    std::unordered_map<int, std::vector<double>> new_prices = cur_prices;
    double total_gen_cost = 0.0;
    bool all_opf_ok = true;

    std::vector<opf::DCOPFResult> opf_step;  // only filled when use_dcopf_prices

    if (options.use_dcopf_prices) {
      opf_step.resize(static_cast<std::size_t>(T));
      for (int k = 0; k < T; ++k) {
        // Inject EV loads from CTM dispatch into the power system
        HybridPowerSystem sys_k = system_with_ctm_ev_loads(
            problem.system, due_res.station_ev_load_kw, sbus, k);

        opf::DCOPFOptions dcopf = options.dcopf_opts;
        dcopf.compute_lmp   = true;
        dcopf.load_shedding = true;
        const opf::DCOPFResult opf_r = opf::solve_dc_opf(sys_k, dcopf);
        opf_step[static_cast<std::size_t>(k)] = opf_r;

        if (opf_r.converged) {
          total_gen_cost += opf_r.objective;
        } else {
          all_opf_ok = false;
          continue;  // Keep previous prices for this step
        }

        // Build bus→LMP lookup for this step
        std::unordered_map<int, double> lmp_k;
        for (std::size_t bi = 0;
             bi < problem.system.ac.buses.size() && bi < opf_r.lmp.size();
             ++bi) {
          lmp_k[problem.system.ac.buses[bi].index] = opf_r.lmp[bi];
        }
        out.lmp_by_step[static_cast<std::size_t>(k)] = lmp_k;

        // ── (c) Price update: π_s^{new} = α·λ_{bus(s)}/1000 + (1-α)·π_s ──
        for (int sid : sta_ids) {
          const int bus = sbus.count(sid) ? sbus.at(sid) : 0;
          const double lmp_mwh = lmp_k.count(bus) ? lmp_k.at(bus) : 0.0;
          const double lmp_kwh = lmp_mwh / kWh_per_MWh;
          const double pi_old  = cur_prices.at(sid)[static_cast<std::size_t>(k)];
          new_prices[sid][static_cast<std::size_t>(k)] =
              alpha * lmp_kwh + (1.0 - alpha) * pi_old;
        }
      }
    } else {
      // No OPF: fixed prices.  Estimate generation cost from marginal costs.
      for (const auto& gen : problem.system.ac.generators) {
        if (!gen.in_service) continue;
        total_gen_cost += gen.cost_c1 * gen.pg_mw + gen.cost_c0;
      }
    }

    // ── (d) Convergence check ─────────────────────────────────────────────────
    double max_price_change = 0.0;
    for (int sid : sta_ids) {
      for (int k = 0; k < T; ++k) {
        const double dp = std::abs(
            new_prices.at(sid)[static_cast<std::size_t>(k)] -
            cur_prices.at(sid)[static_cast<std::size_t>(k)]);
        max_price_change = std::max(max_price_change, dp);
      }
    }
    cur_prices = new_prices;

    // ── Social welfare computation ────────────────────────────────────────────
    const double tstt    = compute_tstt_ctm(problem, due_res, R_est);
    const double b_ev    = ev_gross_benefit(problem, due_res);
    const double c_gen   = options.use_dcopf_prices
                               ? total_gen_cost
                               : std::numeric_limits<double>::quiet_NaN();
    const double c_dly   = options.evpt_opts.value_of_time_per_hr * tstt;
    const double welfare = b_ev - (std::isfinite(c_gen) ? c_gen : 0.0) - c_dly;

    CTMJointIterationRecord rec;
    rec.iteration          = iter;
    rec.social_welfare     = welfare;
    rec.ev_gross_benefit   = b_ev;
    rec.generation_cost    = std::isfinite(c_gen) ? c_gen : 0.0;
    rec.traffic_delay_cost = c_dly;
    rec.price_change       = max_price_change;
    rec.opf_converged      = all_opf_ok;
    rec.due_converged      = due_res.converged;
    rec.due_gap            = due_res.relative_gap;
    out.history.push_back(rec);

    if (options.verbose) {
      std::printf("  [CTM-C iter %2d]  W=%.4f  B_EV=%.4f  C_gen=%.4f  "
                  "C_dly=%.4f  Δπ=%.2e  DUE=%s(gap=%.2e)  OPF=%s\n",
                  iter, welfare, b_ev,
                  std::isfinite(c_gen) ? c_gen : 0.0,
                  c_dly, max_price_change,
                  due_res.converged ? "conv" : "no",
                  due_res.relative_gap,
                  all_opf_ok ? "ok" : "FAIL");
    }

    best_due = due_res;

    const bool price_converged =
        max_price_change < options.price_convergence_tol;
    const bool terminate =
        (iter == options.max_iterations) ||
        price_converged ||
        !options.use_dcopf_prices;   // fixed prices: run exactly one iteration

    if (terminate) {
      out.converged  = price_converged || !options.use_dcopf_prices;
      out.iterations = iter;
      out.status     = out.converged ? "converged" : "max_iterations_reached";

      out.social_welfare     = welfare;
      out.ev_gross_benefit   = b_ev;
      out.generation_cost    = std::isfinite(c_gen) ? c_gen : 0.0;
      out.traffic_delay_cost = c_dly;
      out.final_ctm_due      = due_res;
      out.opf_by_step        = std::move(opf_step);
      out.station_prices_per_step = cur_prices;
      break;
    }

  }  // end outer iteration

  return out;
}

}  // namespace hacdcpf::evpt
