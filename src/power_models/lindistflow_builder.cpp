#include "hacdcpf/power_models/lindistflow_builder.hpp"

#include <stdexcept>
#include <string>

#include "hacdcpf/aml/aml.hpp"

namespace hacdcpf::power_models {

using namespace hacdcpf::aml;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────
static double ldf_get(const std::map<std::string, double>& m,
                      const std::string& k, double def = 0.0) {
  auto it = m.find(k);
  return it == m.end() ? def : it->second;
}

static std::string branch_key(const std::string& from, const std::string& to) {
  return from + "→" + to;
}

// ─────────────────────────────────────────────────────────────────────────────
// Builder
// ─────────────────────────────────────────────────────────────────────────────
LinDistFlowResult solve_lindistflow(const LinDistFlowData& data,
                                     const SolveOptions& opts) {
  if (data.bus_ids.empty())
    throw std::invalid_argument("LinDistFlowData: bus_ids is empty");
  if (data.root_bus.empty())
    throw std::invalid_argument("LinDistFlowData: root_bus is empty");
  if (data.branches.empty())
    throw std::invalid_argument("LinDistFlowData: branches is empty");

  Model m("lindistflow");

  // ── Sets ──────────────────────────────────────────────────────────────────
  auto& buses   = m.add_set("buses");
  auto& branches_set = m.add_set("branches");

  for (const auto& bid : data.bus_ids)
    buses.add_element(Key::scalar(bid));

  for (const auto& [from, to, r, x, smax] : data.branches) {
    (void)r; (void)x; (void)smax;
    branches_set.add_element(Key::scalar(branch_key(from, to)));
  }

  // ── Variables ─────────────────────────────────────────────────────────────
  // P_ij: real power flow on each branch
  auto& P_var = m.add_var("P", branches_set, VarType::Continuous, -1e20, 1e20);
  // Q_ij: reactive power flow on each branch
  auto& Q_var = m.add_var("Q", branches_set, VarType::Continuous, -1e20, 1e20);
  // v_i: squared voltage at each bus
  auto& v_var = m.add_var("v", buses, VarType::Continuous,
                            data.v_min_pu_sq, data.v_max_pu_sq);

  // ── Fix root voltage ───────────────────────────────────────────────────────
  {
    Key rk = Key::scalar(data.root_bus);
    v_var.set_lb(rk, data.root_v_sq_pu);
    v_var.set_ub(rk, data.root_v_sq_pu);
  }

  // Build adjacency: for each bus, outgoing and incoming branch ids
  // incoming_branch[bus_id] = branch_key that feeds INTO bus
  std::map<std::string, std::string> incoming_branch;   // non-root bus → branch key
  // outgoing_branches[bus_id] = list of branch keys leaving from bus
  std::map<std::string, std::vector<std::string>> outgoing_branches;

  for (const auto& [from, to, r, x, smax] : data.branches) {
    (void)r; (void)x; (void)smax;
    std::string bk = branch_key(from, to);
    incoming_branch[to]    = bk;
    outgoing_branches[from].push_back(bk);
  }

  // ── Thermal limits on branches ────────────────────────────────────────────
  for (const auto& [from, to, r, x, smax] : data.branches) {
    (void)r; (void)x;
    std::string bk = branch_key(from, to);
    Key bkey = Key::scalar(bk);
    P_var.set_lb(bkey, -smax);
    P_var.set_ub(bkey,  smax);
    Q_var.set_lb(bkey, -smax);
    Q_var.set_ub(bkey,  smax);
  }

  // ── Power balance at each non-root bus ────────────────────────────────────
  // Σ_j P_ij (outgoing) - P_ki (incoming) = p_dg_i - p_load_i
  // For radial networks each non-root bus has exactly one incoming branch.
  for (const auto& bk : buses.elements()) {
    const std::string& bus = bk.values[0];
    if (bus == data.root_bus) continue;

    double p_load = ldf_get(data.p_load_mw,   bus);
    double q_load = ldf_get(data.q_load_mvar, bus);

    // Real power balance
    {
      LinearExpr flow;
      // Subtract incoming P
      if (incoming_branch.count(bus)) {
        flow += (-1.0) * P_var(Key::scalar(incoming_branch.at(bus)));
      }
      // Add outgoing P
      auto out_it = outgoing_branches.find(bus);
      if (out_it != outgoing_branches.end()) {
        for (const auto& ob : out_it->second)
          flow += P_var(Key::scalar(ob));
      }
      // flow + p_load = 0  (no DG in basic form)
      m.add_constraint(flow == -p_load, "p_bal_" + bus);
    }

    // Reactive power balance
    {
      LinearExpr flow;
      if (incoming_branch.count(bus)) {
        flow += (-1.0) * Q_var(Key::scalar(incoming_branch.at(bus)));
      }
      auto out_it = outgoing_branches.find(bus);
      if (out_it != outgoing_branches.end()) {
        for (const auto& ob : out_it->second)
          flow += Q_var(Key::scalar(ob));
      }
      m.add_constraint(flow == -q_load, "q_bal_" + bus);
    }
  }

  // ── LinDistFlow voltage drop: v_j = v_i - 2*(r*P_ij + x*Q_ij) ───────────
  for (const auto& [from, to, r, x, smax] : data.branches) {
    (void)smax;
    std::string bk = branch_key(from, to);
    Key bkey = Key::scalar(bk);
    Key fkey = Key::scalar(from);
    Key tkey = Key::scalar(to);

    // v_j - v_i + 2*r*P_ij + 2*x*Q_ij = 0
    LinearExpr row =
        v_var(tkey)
        + (-1.0) * v_var(fkey)
        + (2.0 * r) * P_var(bkey)
        + (2.0 * x) * Q_var(bkey);
    m.add_constraint(row == 0.0, "vdrop_" + bk);
  }

  // ── Objective: minimise Σ_i (v_max - v_i)  (voltage regulation proxy) ────
  {
    LinearExpr obj;
    for (const auto& bk : buses.elements()) {
      const std::string& bus = bk.values[0];
      if (bus == data.root_bus) continue;
      obj += (-1.0) * v_var(Key::scalar(bus));
    }
    m.minimize(obj);  // maximise sum of voltages
  }

  // ── Solve ─────────────────────────────────────────────────────────────────
  m.check_bounds();
  auto sr = m.solve(opts);

  // ── Extract results ───────────────────────────────────────────────────────
  LinDistFlowResult result;
  result.solve_result = sr;

  if (sr.has_primal()) {
    for (const auto& bk : buses.elements()) {
      const std::string& bus = bk.values[0];
      result.voltage_sq_pu[bus] = sr.var_value(v_var, Key::scalar(bus));
    }
    for (const auto& [from, to, r, x, smax] : data.branches) {
      (void)r; (void)x; (void)smax;
      std::string bk = branch_key(from, to);
      Key bkey = Key::scalar(bk);
      result.branch_p_mw[bk]   = sr.var_value(P_var, bkey);
      result.branch_q_mvar[bk] = sr.var_value(Q_var, bkey);
    }
  }

  return result;
}

}  // namespace hacdcpf::power_models
