#include "hacdcpf/power_models/scuc_builder.hpp"

#include <stdexcept>
#include <string>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

using namespace hacdcpf::aml;

// ─────────────────────────────────────────────────────────────────────────────
// Small helpers
// ─────────────────────────────────────────────────────────────────────────────
static double get_map(const std::map<std::string, double>& m,
                      const std::string& k, double default_val = 0.0) {
  const auto it = m.find(k);
  return it == m.end() ? default_val : it->second;
}

// ─────────────────────────────────────────────────────────────────────────────
// Builder
// ─────────────────────────────────────────────────────────────────────────────
SCUCResult solve_scuc(const SCUCData& data, const SolveOptions& opts) {
  if (data.generator_ids.empty())
    throw std::invalid_argument("SCUCData: generator_ids is empty");
  if (data.period_ids.empty())
    throw std::invalid_argument("SCUCData: period_ids is empty");

  Model m("scuc");

  // ── Sets ──────────────────────────────────────────────────────────────────
  auto& gens = m.add_set("generators");
  auto& T    = m.add_ordered_set("periods");   // OrderedSet enables prev()

  for (const auto& gid : data.generator_ids)
    gens.add_element(Key::scalar(gid));
  for (const auto& tid : data.period_ids)
    T.add_element(Key::scalar(tid));

  // ── Parameters ────────────────────────────────────────────────────────────
  auto& dem_p    = m.add_param("demand",       1, "MW");
  auto& cost_p   = m.add_param("cost",         1, "$/MWh");
  auto& pmin_p   = m.add_param("pmin",         1, "MW");
  auto& pmax_p   = m.add_param("pmax",         1, "MW");
  auto& ramp_u_p = m.add_param("ramp_up",      1, "MW/h");
  auto& ramp_d_p = m.add_param("ramp_dn",      1, "MW/h");
  auto& s_cost_p = m.add_param("startup_cost", 1, "$");
  auto& c_cost_p = m.add_param("commit_cost",  1, "$/h");

  // Load period parameters
  for (const auto& tid : data.period_ids) {
    Key tk = Key::scalar(tid);
    dem_p.set(tk, get_map(data.demand_MW, tid, 0.0));
  }
  // Load generator parameters
  for (const auto& gid : data.generator_ids) {
    Key gk = Key::scalar(gid);
    cost_p.set(gk,   get_map(data.cost_per_MWh,  gid, 1.0));
    pmin_p.set(gk,   get_map(data.pmin_MW,        gid, 0.0));
    pmax_p.set(gk,   get_map(data.pmax_MW,        gid, 1e6));
    ramp_u_p.set(gk, get_map(data.ramp_up_MW,     gid, 1e6));
    ramp_d_p.set(gk, get_map(data.ramp_dn_MW,     gid, 1e6));
    s_cost_p.set(gk, get_map(data.startup_cost,   gid, 0.0));
    c_cost_p.set(gk, get_map(data.commit_cost,    gid, 0.0));
  }

  // ── Variables ─────────────────────────────────────────────────────────────
  // u(g,t) ∈ {0,1}   commitment binary
  // p(g,t) ≥ 0       dispatch [MW]
  // s(g,t) ≥ 0       startup indicator [MW or 1; here treated as ≥ 0]
  auto& u_var = m.add_var("u", gens, T, VarType::Binary);
  auto& p_var = m.add_var("p", gens, T, VarType::Continuous, 0.0, 1e20);
  auto& s_var = m.add_var("s", gens, T, VarType::Continuous, 0.0, 1e20);

  // ── Objective ─────────────────────────────────────────────────────────────
  //  Σ_g Σ_t [ cost_g * p_{g,t} + startup_cost_g * s_{g,t} + commit_cost_g * u_{g,t} ]
  {
    LinearExpr obj;
    for (const auto& gk : gens.elements()) {
      const std::string& g = gk.values[0];
      for (const auto& tk : T.elements()) {
        const std::string& t = tk.values[0];
        Key gtk = Key::pair(g, t);
        obj += cost_p.get(gk)   * p_var(gtk);
        obj += s_cost_p.get(gk) * s_var(gtk);
        obj += c_cost_p.get(gk) * u_var(gtk);
      }
    }
    m.minimize(obj);
  }

  // ── Balance: Σ_g p_{g,t} = D_t  ∀ t ─────────────────────────────────────
  auto& balance = m.add_constraints("balance", T, [&](const Key& tk) {
    LinearExpr gen_sum;
    for (const auto& gk : gens.elements()) {
      Key gtk = Key::pair(gk.values[0], tk.values[0]);
      gen_sum += p_var(gtk);
    }
    return gen_sum == dem_p.get(tk);
  });
  (void)balance;  // possibly used for price extraction below

  // ── Per (generator, period) constraints ───────────────────────────────────
  for (const auto& gk : gens.elements()) {
    const std::string& g = gk.values[0];
    double pmax = pmax_p.get(gk);
    double pmin = pmin_p.get(gk);
    double ru   = ramp_u_p.get(gk);
    double rd   = ramp_d_p.get(gk);

    for (const auto& tk : T.elements()) {
      const std::string& t = tk.values[0];
      Key gtk = Key::pair(g, t);

      // Capacity upper: p_{g,t} ≤ pmax * u_{g,t}
      //   → p_{g,t} − pmax*u_{g,t} ≤ 0
      m.add_constraint(
          p_var(gtk) - pmax * u_var(gtk) <= 0.0,
          "cap_ub_" + g + "_" + t);

      // Capacity lower: p_{g,t} ≥ pmin * u_{g,t}
      //   → p_{g,t} − pmin*u_{g,t} ≥ 0
      m.add_constraint(
          p_var(gtk) - pmin * u_var(gtk) >= 0.0,
          "cap_lb_" + g + "_" + t);

      // Ramp / startup constraints require previous period
      auto prev_tk_opt = T.prev(tk);
      if (!prev_tk_opt.has_value()) continue;

      const std::string& tp = prev_tk_opt->values[0];
      Key gtk_prev = Key::pair(g, tp);

      // Ramp up: p_{g,t} − p_{g,t−1} ≤ R_up
      m.add_constraint(
          static_cast<LinearExpr>(p_var(gtk)) - p_var(gtk_prev) <= ru,
          "ramp_up_" + g + "_" + t);

      // Ramp down: p_{g,t−1} − p_{g,t} ≤ R_dn
      m.add_constraint(
          static_cast<LinearExpr>(p_var(gtk_prev)) - p_var(gtk) <= rd,
          "ramp_dn_" + g + "_" + t);

      // Startup: s_{g,t} ≥ u_{g,t} − u_{g,t−1}
      //   → u_{g,t} − u_{g,t−1} − s_{g,t} ≤ 0
      m.add_constraint(
          static_cast<LinearExpr>(u_var(gtk)) - u_var(gtk_prev) - s_var(gtk) <= 0.0,
          "startup_" + g + "_" + t);
    }
  }

  // ── Beta: Min-up constraints ──────────────────────────────────────────────
  // For each generator g with min_up T_up > 1:
  //   If unit starts at period t (u_{g,t} - u_{g,t-1} = 1),
  //   it must stay on for the next T_up-1 periods.
  //   Reformulated as:  Σ_{j=t}^{t+T_up-1} u_{g,j} ≥ T_up*(u_{g,t} - u_{g,t-1})
  //
  // For the first period we use initial_commitment as u_{g,t-1}.
  for (const auto& gk : gens.elements()) {
    const std::string& g = gk.values[0];
    auto mu_it = data.min_up_periods.find(g);
    if (mu_it == data.min_up_periods.end() || mu_it->second <= 1) continue;
    int T_up = mu_it->second;
    double u_init = get_map(data.initial_commitment, g, 0.0);

    const auto& all_periods = T.elements();
    const int nT = static_cast<int>(all_periods.size());

    for (int ti = 0; ti < nT; ++ti) {
      const std::string& t = all_periods[static_cast<std::size_t>(ti)].values[0];
      Key gtk = Key::pair(g, t);

      // u_prev: for ti=0 use initial_commitment, otherwise prev period
      LinearExpr u_cur = u_var(gtk);
      LinearExpr u_prev;
      if (ti == 0) {
        u_prev = LinearExpr::const_expr(u_init);
      } else {
        const std::string& tp = all_periods[static_cast<std::size_t>(ti-1)].values[0];
        u_prev = u_var(Key::pair(g, tp));
      }

      // Σ_{j=ti}^{min(ti+T_up-1, nT-1)} u_{g,j}
      LinearExpr u_sum;
      int window = std::min(ti + T_up - 1, nT - 1);
      for (int j = ti; j <= window; ++j) {
        const std::string& tj = all_periods[static_cast<std::size_t>(j)].values[0];
        u_sum += u_var(Key::pair(g, tj));
      }

      // u_sum ≥ T_up*(u_cur - u_prev)  →  u_sum - T_up*u_cur + T_up*u_prev ≥ 0
      // But only when window covers all T_up periods (or we truncate the window)
      // Standard: Σ ≥ (window-ti+1) * (u_t - u_{t-1}) if window < ti+T_up-1 (end of horizon)
      int actual_win = window - ti + 1;
      LinearExpr row = u_sum + (static_cast<double>(-actual_win)) * u_cur + (static_cast<double>(actual_win)) * u_prev;
      m.add_constraint(row >= 0.0, "min_up_" + g + "_" + t);
    }
  }

  // ── Beta: Min-down constraints ────────────────────────────────────────────
  // Similarly: if unit shuts down at t, must stay off for T_dn periods.
  //   Σ_{j=t}^{t+T_dn-1} (1 - u_{g,j}) ≥ T_dn*(u_{g,t-1} - u_{g,t})
  //   → −Σ u_{g,j} ≥ T_dn*(u_{g,t-1} - u_{g,t}) - T_dn
  //   → T_dn*u_{g,t} - T_dn*u_{g,t-1} - Σ u_{g,j} ≥ -T_dn
  for (const auto& gk : gens.elements()) {
    const std::string& g = gk.values[0];
    auto md_it = data.min_dn_periods.find(g);
    if (md_it == data.min_dn_periods.end() || md_it->second <= 1) continue;
    int T_dn = md_it->second;
    double u_init = get_map(data.initial_commitment, g, 0.0);

    const auto& all_periods = T.elements();
    const int nT = static_cast<int>(all_periods.size());

    for (int ti = 0; ti < nT; ++ti) {
      const std::string& t = all_periods[static_cast<std::size_t>(ti)].values[0];
      Key gtk = Key::pair(g, t);

      LinearExpr u_cur = u_var(gtk);
      LinearExpr u_prev;
      if (ti == 0) {
        u_prev = LinearExpr::const_expr(u_init);
      } else {
        const std::string& tp = all_periods[static_cast<std::size_t>(ti-1)].values[0];
        u_prev = u_var(Key::pair(g, tp));
      }

      int window = std::min(ti + T_dn - 1, nT - 1);
      int actual_win = window - ti + 1;

      LinearExpr u_sum;
      for (int j = ti; j <= window; ++j) {
        const std::string& tj = all_periods[static_cast<std::size_t>(j)].values[0];
        u_sum += u_var(Key::pair(g, tj));
      }

      // actual_win * (u_{t-1} - u_t) ≤ actual_win - Σ u_j
      // → actual_win*u_prev - actual_win*u_cur - (actual_win - Σ u_j) ≤ 0
      // i.e. Σ u_j - actual_win*u_cur + actual_win*u_prev ≤ actual_win
      // actually: Σ (1-u_j) ≥ actual_win*(u_{t-1}-u_t)
      //           actual_win - Σ u_j ≥ actual_win*u_prev - actual_win*u_cur
      //           actual_win*u_cur - actual_win*u_prev + actual_win - Σ u_j ≥ 0  -- wrong sign
      // Correct:
      //   Σ (1-u_j) ≥ actual_win*(u_{t-1}-u_t)
      //   actual_win - u_sum ≥ actual_win*u_prev - actual_win*u_cur
      //   u_sum + actual_win*u_prev - actual_win*u_cur ≤ actual_win
      LinearExpr row = u_sum + static_cast<double>(actual_win) * u_prev
                             + static_cast<double>(-actual_win) * u_cur;
      m.add_constraint(row <= static_cast<double>(actual_win),
                       "min_dn_" + g + "_" + t);
    }
  }

  // ── Beta: Spinning reserve ────────────────────────────────────────────────
  // Σ_g pmax_g * u_{g,t} ≥ demand_t + reserve_t   ∀ t with reserve > 0
  if (!data.spinning_reserve_MW.empty()) {
    for (const auto& tk : T.elements()) {
      const std::string& t = tk.values[0];
      auto res_it = data.spinning_reserve_MW.find(t);
      if (res_it == data.spinning_reserve_MW.end()) continue;
      double demand  = get_map(data.demand_MW, t, 0.0);
      double reserve = res_it->second;
      LinearExpr cap_sum;
      for (const auto& gk : gens.elements()) {
        const std::string& g = gk.values[0];
        double pm = pmax_p.get(gk);
        cap_sum += pm * u_var(Key::pair(g, t));
      }
      m.add_constraint(cap_sum >= demand + reserve, "reserve_" + t);
    }
  }

  // ── Solve ─────────────────────────────────────────────────────────────────
  m.check_bounds();
  auto sr = m.solve(opts);

  // ── Extract results ───────────────────────────────────────────────────────
  SCUCResult result;
  result.solve_result = sr;

  if (sr.has_primal()) {
    for (const auto& gk : gens.elements()) {
      const std::string& g = gk.values[0];
      for (const auto& tk : T.elements()) {
        const std::string& t = tk.values[0];
        Key gtk = Key::pair(g, t);
        auto gpair = std::make_pair(g, t);

        result.commitment[gpair]    = sr.var_value(u_var, gtk);
        result.dispatch_MW[gpair]   = sr.var_value(p_var, gtk);
        result.startup_event[gpair] = sr.var_value(s_var, gtk);
      }
    }

    // Balance duals (available only for LP relaxation / MIP LP)
    if (sr.has_duals()) {
      for (const auto& tk : T.elements()) {
        auto d = sr.dual(balance, tk);
        if (d.has_value()) {
          result.price_per_MWh[tk.values[0]] = *d;
        }
      }
    }
  }

  return result;
}

}  // namespace hacdcpf::power_models
