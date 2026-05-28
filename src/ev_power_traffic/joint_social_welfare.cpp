/// joint_social_welfare.cpp
/// =============================================================================
/// Joint optimisation of power systems and traffic-EV systems targeting the
/// maximisation of social welfare.
///
/// Algorithm — Iterative LMP Coordination (Dantzig-Wolfe decomposition style)
/// ---------------------------------------------------------------------------
///   The problem couples two sub-problems:
///
///   1. Traffic / EV sub-problem (solved by SystemOptimalLP):
///      min  Σ_{d,r}  c_{d,r}(π) · x_{d,r}   s.t. flow balance + capacity
///      where  c_{d,r}(π) = VOT·travel_time_{d,r} + Σ_s π_{s,k} · E_s
///      and  π_{s,k}  is the nodal electricity price at station s at step k.
///
///   2. Power / OPF sub-problem (solved by DC-OPF at each step k):
///      min  Σ_g  (c2_g · Pg² + c1_g · Pg + c0_g)
///      s.t. DC power balance, branch limits, gen limits,
///           with EV loads  P^EV_{bus(s),k} = Σ_{d,r} x_{d,r} · p_{s}/1000 (MW)
///
///   At each outer iteration:
///     (a) Traffic step  — solve SystemOptimalLP with current π → x^{new}, P^EV
///     (b) Power step    — for each k: run DC-OPF with P^EV_k → LMP λ_{b,k}
///     (c) Price update  — π_{s,k}^{new} = α · (λ_{bus(s),k}/1000)
///                                         + (1-α) · π_{s,k}^{old}
///         (LMP is in $/MWh; divide by 1000 to convert to $/kWh)
///     (d) Convergence check: max |π^{new} - π^{old}| < tol
///
///   Social welfare at each iteration:
///     W = B_EV  -  C_gen  -  C_delay
///     B_EV   = Σ_d  WTP_d · served_d                    (consumer benefit)
///     C_gen  = Σ_k  OPF_objective_k                     (dispatch cost)
///     C_delay= VOT · Σ_{d,r} x_{d,r} · travel_time_{d,r} (congestion)
///
/// =============================================================================

#include "hacdcpf/ev_power_traffic/ev_power_traffic_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>

#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

namespace hacdcpf::evpt {

namespace {

constexpr double kWh_per_MWh = 1000.0;   // 1 MWh = 1000 kWh
constexpr double kW_per_MW   = 1000.0;   // 1 MW  = 1000 kW

// ─────────────────────────────────────────────────────────────────────────────
// Collect every (station_id → bus_number) pair referenced by routes/sessions.
// ─────────────────────────────────────────────────────────────────────────────
std::unordered_map<int, int> station_bus_map(const EVPowerTrafficProblem& prob) {
  std::unordered_map<int, int> m;
  for (const auto& cs : prob.system.ac.charging_stations) {
    m[cs.index] = cs.bus;
  }
  return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// Build StationPriceProfile list from a per-station, per-step price matrix.
// Replaces any existing entries in prob.station_prices.
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
// Inject EV station loads into a system copy so that DC-OPF sees them.
// Strategy: for each active charging station, add p_total_kw/1000 MW to its
// bus's pd_mw field (load demand).  Discharge (p_total_kw < 0) subtracts.
// ─────────────────────────────────────────────────────────────────────────────
HybridPowerSystem system_with_ev_loads(const HybridPowerSystem& base,
                                       const EVPowerTrafficStepResult& step) {
  HybridPowerSystem sys = base;
  for (const auto& ss : step.stations) {
    // Find bus for this station
    int bus_id = 0;
    for (const auto& cs : sys.ac.charging_stations) {
      if (cs.index == ss.station_id) { bus_id = cs.bus; break; }
    }
    if (bus_id == 0) continue;

    const double ev_mw = ss.p_ev_kw / kW_per_MW;   // kW → MW (pos=load, neg=gen)
    for (auto& b : sys.ac.buses) {
      if (b.index == bus_id) { b.pd_mw += ev_mw; break; }
    }
  }
  return sys;
}

// ─────────────────────────────────────────────────────────────────────────────
// Gross EV consumer benefit = Σ_d  WTP_d × served_d
// ─────────────────────────────────────────────────────────────────────────────
double ev_gross_benefit(const EVPowerTrafficProblem& prob,
                        const EVPowerTrafficResult& evpt) {
  // Build demand WTP lookup
  std::unordered_map<int, double> wtp;
  for (const auto& d : prob.demands) {
    wtp[d.index] = d.willingness_to_pay_per_vehicle;
  }

  double benefit = 0.0;
  for (const auto& a : evpt.assignments) {
    if (!a.served) continue;
    const double w = wtp.count(a.demand_index) ? wtp.at(a.demand_index) : 0.0;
    benefit += w * a.vehicles;
  }
  return benefit;
}

// ─────────────────────────────────────────────────────────────────────────────
// EV consumer surplus = B_EV - total charging payment
//   price_paid = Σ_d Σ_r  π_{s(r),k(d,r)} · E_s(r) · x_{d,r}
// ─────────────────────────────────────────────────────────────────────────────
double ev_consumer_surplus(const EVPowerTrafficProblem& prob,
                           const EVPowerTrafficResult& evpt,
                           const std::unordered_map<int, std::vector<double>>& prices) {
  // Build demand WTP
  std::unordered_map<int, double> wtp;
  for (const auto& d : prob.demands) {
    wtp[d.index] = d.willingness_to_pay_per_vehicle;
  }

  // Build route-stop price lookup
  // For each assignment (demand, route, step) → compute payment
  std::unordered_map<int, const RouteAlternative*> route_map;
  for (const auto& r : prob.routes) route_map[r.index] = &r;

  double surplus = 0.0;
  for (const auto& a : evpt.assignments) {
    if (!a.served) continue;
    const double w = wtp.count(a.demand_index) ? wtp.at(a.demand_index) : 0.0;
    double payment = 0.0;
    if (route_map.count(a.route_index)) {
      for (const auto& stop : route_map.at(a.route_index)->charging_stops) {
        if (!prices.count(stop.station_id)) continue;
        const auto& pvec = prices.at(stop.station_id);
        const int step = std::min(a.departure_step, static_cast<int>(pvec.size()) - 1);
        const double pi = step >= 0 ? pvec[static_cast<std::size_t>(step)] : 0.0;
        payment += pi * stop.requested_energy_kwh_per_vehicle;
      }
    }
    surplus += (w - payment) * a.vehicles;
  }
  return surplus;
}

// ─────────────────────────────────────────────────────────────────────────────
// Traffic delay cost = VOT × total_travel_time_hr
// ─────────────────────────────────────────────────────────────────────────────
double traffic_delay_cost(const EVPowerTrafficResult& evpt, double vot) {
  return vot * evpt.total_travel_time_hr;
}

}  // namespace


// =============================================================================
// solve_joint_social_welfare — public entry point
// =============================================================================

JointSocialWelfareResult solve_joint_social_welfare(
    EVPowerTrafficProblem   problem,
    const JointSocialWelfareOptions& options) {

  JointSocialWelfareResult out;
  out.status = "initialised";

  const int    T     = options.evpt_opts.num_steps;
  const double alpha = std::clamp(options.price_update_step, 0.0, 1.0);

  // ── Collect station IDs used by the traffic routes + initial sessions ──────
  std::vector<int> sta_ids;
  {
    std::unordered_map<int, bool> seen;
    auto add = [&](int id) {
      if (id && !seen.count(id)) { seen[id] = true; sta_ids.push_back(id); }
    };
    for (const auto& cs : problem.system.ac.charging_stations) add(cs.index);
    for (const auto& r  : problem.routes)
      for (const auto& s : r.charging_stops) add(s.station_id);
    for (const auto& s  : problem.initial_sessions) add(s.station_id);
    std::sort(sta_ids.begin(), sta_ids.end());
  }

  const auto sbus = station_bus_map(problem);

  // ── Initialise station prices to the default (or whatever is in problem) ───
  const double default_price = options.evpt_opts.default_station_price_per_kwh;
  std::unordered_map<int, std::vector<double>> cur_prices;
  for (int sid : sta_ids) {
    // Use existing profile if present; pad/trim to T steps.
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

  // ── Force SystemOptimalLP on the traffic sub-problem ─────────────────────
  EVPowerTrafficOptions evpt_opts = options.evpt_opts;
  evpt_opts.assignment_model = AssignmentModel::SystemOptimalLP;

  // ── Outer iteration loop ──────────────────────────────────────────────────
  EVPowerTrafficResult best_evpt;

  for (int iter = 1; iter <= options.max_iterations; ++iter) {

    // ── (a) Traffic step: LP assignment with current prices ──────────────────
    set_station_prices(problem, cur_prices);
    const EVPowerTrafficResult evpt = simulate_ev_power_traffic(problem, evpt_opts);

    if (!evpt.feasible) {
      out.status = "traffic-assignment infeasible at iteration " + std::to_string(iter);
      // Store what we have so far and break
      out.final_evpt = evpt;
      break;
    }

    // ── (b) Power step: DC-OPF for each time step ────────────────────────────
    std::unordered_map<int, std::vector<double>> new_prices = cur_prices;
    double total_gen_cost = 0.0;
    bool all_opf_ok = true;

    std::vector<opf::DCOPFResult> opf_step(static_cast<std::size_t>(T));

    if (options.use_dcopf_prices) {
      for (int k = 0; k < T; ++k) {
        // Build system with EV loads for step k
        const EVPowerTrafficStepResult& step_res =
            (k < static_cast<int>(evpt.steps.size()))
            ? evpt.steps[static_cast<std::size_t>(k)]
            : EVPowerTrafficStepResult{};

        HybridPowerSystem sys_k = system_with_ev_loads(problem.system, step_res);

        // Solve DC-OPF
        opf::DCOPFOptions dcopf = options.dcopf_opts;
        dcopf.compute_lmp   = true;
        dcopf.load_shedding = true;
        const opf::DCOPFResult opf_r = opf::solve_dc_opf(sys_k, dcopf);
        opf_step[static_cast<std::size_t>(k)] = opf_r;

        if (opf_r.converged) {
          total_gen_cost += opf_r.objective;
        } else {
          all_opf_ok = false;
          // Keep previous prices for this step
          continue;
        }

        // ── Build bus→LMP lookup for this step ──────────────────────────────
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
      }  // end step loop
    } else {
      // No OPF: generation cost estimated from generator marginal costs
      for (const auto& gen : problem.system.ac.generators) {
        if (!gen.in_service) continue;
        total_gen_cost += gen.cost_c1 * gen.pg_mw + gen.cost_c0;
      }
    }

    // ── (d) Convergence check ────────────────────────────────────────────────
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

    // ── Compute social welfare ───────────────────────────────────────────────
    const double b_ev   = ev_gross_benefit(problem, evpt);
    const double c_gen  = options.use_dcopf_prices
                              ? total_gen_cost
                              : std::numeric_limits<double>::quiet_NaN();
    const double c_dly  = traffic_delay_cost(evpt, evpt_opts.value_of_time_per_hr);
    const double welfare = b_ev - (std::isfinite(c_gen) ? c_gen : 0.0) - c_dly;

    JointIterationRecord rec;
    rec.iteration         = iter;
    rec.social_welfare    = welfare;
    rec.ev_gross_benefit  = b_ev;
    rec.generation_cost   = std::isfinite(c_gen) ? c_gen : 0.0;
    rec.traffic_delay_cost= c_dly;
    rec.price_change      = max_price_change;
    rec.opf_converged     = all_opf_ok;
    out.history.push_back(rec);

    if (options.verbose) {
      std::printf("  [JSW iter %2d]  W=%.4f  B_EV=%.4f  C_gen=%.4f  "
                  "C_dly=%.4f  Δπ=%.2e  opf=%s\n",
                  iter, welfare, b_ev,
                  std::isfinite(c_gen) ? c_gen : 0.0,
                  c_dly, max_price_change,
                  all_opf_ok ? "ok" : "FAIL");
    }

    best_evpt = evpt;
    if (iter == options.max_iterations ||
        max_price_change < options.price_convergence_tol) {
      out.converged  = (max_price_change < options.price_convergence_tol);
      out.iterations = iter;
      out.status     = out.converged ? "converged" : "max_iterations_reached";

      out.social_welfare     = welfare;
      out.ev_gross_benefit   = b_ev;
      out.ev_consumer_surplus= ev_consumer_surplus(problem, evpt, cur_prices);
      out.generation_cost    = std::isfinite(c_gen) ? c_gen : 0.0;
      out.traffic_delay_cost = c_dly;
      out.final_evpt         = evpt;
      out.opf_by_step        = std::move(opf_step);
      out.station_prices_per_step = cur_prices;
      break;
    }

  }  // end outer iteration

  return out;
}

}  // namespace hacdcpf::evpt
