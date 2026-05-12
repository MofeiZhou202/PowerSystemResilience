#include "hacdcpf/power_models/dc_opf_builder.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

using namespace hacdcpf::aml;

DCOPFResult solve_dc_opf(const DCOPFData& data, const SolveOptions& opts) {
  // ── Validation ─────────────────────────────────────────────────────────
  if (data.bus_ids.empty()) {
    throw std::invalid_argument("DCOPFData: bus_ids is empty");
  }
  if (data.slack_bus.empty()) {
    throw std::invalid_argument("DCOPFData: slack_bus is empty");
  }
  {
    bool found = false;
    for (const auto& b : data.bus_ids) {
      if (b == data.slack_bus) { found = true; break; }
    }
    if (!found) {
      throw std::invalid_argument("DCOPFData: slack_bus '" +
                                  data.slack_bus + "' not in bus_ids");
    }
  }

  Model m("dc_opf");

  // ── Sets ────────────────────────────────────────────────────────────────
  auto& buses = m.add_set("buses", data.bus_ids);
  auto& gens_s = m.add_set("generators");
  auto& brs_s  = m.add_set("branches");

  for (const auto& [gid, bid, pmin, pmax, cost] : data.generators) {
    gens_s.add_element(Key::scalar(gid));
  }
  for (const auto& [fb, tb, susc, flim] : data.branches) {
    brs_s.add_element(Key::scalar(fb + "->" + tb));
  }

  // ── Parameters ──────────────────────────────────────────────────────────
  auto& pmin_p = m.add_param("pmin", 1, "MW");
  auto& pmax_p = m.add_param("pmax", 1, "MW");
  auto& cost_p = m.add_param("cost", 1, "$/MWh");
  auto& dem_p  = m.add_param("demand", 1, "MW");
  auto& susc_p = m.add_param("susc", 1, "1/ohm");
  auto& flim_p = m.add_param("flim", 1, "MW");

  for (const auto& [gid, bid, pmin, pmax, cost] : data.generators) {
    Key gk = Key::scalar(gid);
    pmin_p.set(gk, pmin);
    pmax_p.set(gk, pmax);
    cost_p.set(gk, cost);
  }
  for (const auto& [bus, demand] : data.demands_MW) {
    dem_p.set(Key::scalar(bus), demand);
  }
  for (const auto& [fb, tb, susc, flim] : data.branches) {
    Key brk = Key::scalar(fb + "->" + tb);
    susc_p.set(brk, susc);
    flim_p.set(brk, flim);
  }
  // Buses with no demand entry get 0 MW demand
  for (const auto& bus : data.bus_ids) {
    Key bk = Key::scalar(bus);
    if (data.demands_MW.find(bus) == data.demands_MW.end()) {
      dem_p.set(bk, 0.0);
    }
  }

  // ── Variables ────────────────────────────────────────────────────────────
  auto& p_gen = m.add_var("p_gen", gens_s, VarType::Continuous, 0.0, 1e20);
  // Use wide angle bounds; only the slack bus fix and balance equations
  // actually determine the angles.  Tight angle bounds (e.g. ±π) would
  // shadow line-flow limits and give wrong LMPs in congested cases.
  auto& theta = m.add_var("theta", buses, VarType::Continuous, -1e6, 1e6);

  // Set per-generator bounds from parameters
  for (const auto& [gid, bid, pmin, pmax, cost] : data.generators) {
    Key gk = Key::scalar(gid);
    p_gen.set_lb(gk, pmin);
    p_gen.set_ub(gk, pmax);
  }
  // Fix slack bus angle to 0
  theta.fix(Key::scalar(data.slack_bus), 0.0);

  // ── Objective: min Σ_g cost_g * p_g ─────────────────────────────────────
  m.minimize(sum(gens_s, [&](const Key& gk) {
    return cost_p.get(gk) * p_gen(gk);
  }));

  // ── Nodal balance: Σ_{g∈g(i)} p_g − P_d^i − net_flow_out = 0 ───────────
  auto& balance = m.add_constraints("balance", buses, [&](const Key& ik) {
    const std::string& bus = ik.values[0];
    LinearExpr lhs;

    // Add generation injections
    for (const auto& [gid, bid, pmin, pmax, cost] : data.generators) {
      if (bid == bus) {
        lhs += p_gen(Key::scalar(gid));
      }
    }
    // Subtract nodal demand
    lhs -= dem_p.get(ik);

    // Subtract net DC flows: outgoing positive, incoming negative
    // ℓ = (from, to): flow = b*(θ_from − θ_to)
    // If bus == from: subtract flow  → lhs -= b*(θ_bus − θ_to)
    // If bus == to  : add flow       → lhs += b*(θ_from − θ_bus)
    for (const auto& [fb, tb, susc, flim] : data.branches) {
      if (fb == bus) {
        // outgoing: subtract b*(θ_bus − θ_to)
        lhs -= susc * theta(Key::scalar(fb));
        lhs += susc * theta(Key::scalar(tb));
      }
      if (tb == bus) {
        // incoming: add b*(θ_from − θ_bus)
        lhs += susc * theta(Key::scalar(fb));
        lhs -= susc * theta(Key::scalar(tb));
      }
    }

    return lhs == 0.0;
  });

  // ── Flow limits ──────────────────────────────────────────────────────────
  m.add_constraints("flow_ub", brs_s, [&](const Key& brk) {
    const std::string& br = brk.values[0];
    const auto sep = br.find("->"); 
    const std::string fb = br.substr(0, sep);
    const std::string tb = br.substr(sep + 2);
    double flim = flim_p.get(brk);
    if (flim <= 0.0) {
      // No limit — use a trivially-satisfied constraint (1 <= infinity)
      return LinearExpr::const_expr(0.0) <= 1.0;
    }
    // b*(θ_from − θ_to) <= Pmax
    LinearExpr flow = susc_p.get(brk) * theta(Key::scalar(fb));
    flow -= susc_p.get(brk) * theta(Key::scalar(tb));
    return flow <= flim;
  });

  m.add_constraints("flow_lb", brs_s, [&](const Key& brk) {
    const std::string& br = brk.values[0];
    const auto sep = br.find("->");
    const std::string fb = br.substr(0, sep);
    const std::string tb = br.substr(sep + 2);
    double flim = flim_p.get(brk);
    if (flim <= 0.0) {
      return LinearExpr::const_expr(0.0) >= -1.0;
    }
    // b*(θ_from − θ_to) >= −Pmax
    LinearExpr flow = susc_p.get(brk) * theta(Key::scalar(fb));
    flow -= susc_p.get(brk) * theta(Key::scalar(tb));
    return flow >= -flim;
  });

  // ── Solve ─────────────────────────────────────────────────────────────────
  m.check_bounds();
  auto sr = m.solve(opts);

  // ── Extract results ───────────────────────────────────────────────────────
  DCOPFResult result;
  result.solve_result = sr;

  if (sr.has_primal()) {
    // Generator dispatch
    for (const auto& [gk, vid] : p_gen.key_to_id()) {
      result.gen_dispatch_MW[gk.values[0]] = sr.var_value(p_gen, gk);
    }
    // Voltage angles
    for (const auto& [bk, vid] : theta.key_to_id()) {
      result.voltage_angle_rad[bk.values[0]] = sr.var_value(theta, bk);
    }
    // Branch flows: flow_ℓ = b*(θ_from − θ_to)
    for (const auto& [fb, tb, susc, flim] : data.branches) {
      const std::string br_key = fb + "->" + tb;
      double th_f = result.voltage_angle_rad.count(fb) ?
                    result.voltage_angle_rad.at(fb) : 0.0;
      double th_t = result.voltage_angle_rad.count(tb) ?
                    result.voltage_angle_rad.at(tb) : 0.0;
      result.branch_flow_MW[br_key] = susc * (th_f - th_t);
    }
    // LMPs: dual(balance) = ∂cost/∂demand = marginal cost at each bus.
    // (Our balance is Σp_g - demand = 0; b_eq = demand;
    //  so dual = ∂obj*/∂demand = positive marginal cost.)
    if (sr.has_duals()) {
      for (const auto& [ik, cid] : balance.key_to_id()) {
        auto d = sr.dual(balance, ik);
        if (d.has_value()) {
          result.lmp_per_MWh[ik.values[0]] = *d;
        }
      }
    }
  }

  return result;
}

}  // namespace hacdcpf::power_models
