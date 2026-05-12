#include "hacdcpf/optimal_power_flow/dc_opf.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/solver/external/adapters.hpp"
#include "hacdcpf/engine/kernel/ipm/lcqp_solver.hpp"
#include "hacdcpf/engine/problem_types.hpp"

namespace hacdcpf::opf {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHugeBound = 1e20;

// --------------------------------------------------------------------------
// Build bus index map: bus_id (1-based) -> position (0-based)
// --------------------------------------------------------------------------
std::unordered_map<int, int> build_bus_map(const std::vector<ACBus>& buses) {
  std::unordered_map<int, int> m;
  m.reserve(buses.size());
  for (int i = 0; i < static_cast<int>(buses.size()); ++i) {
    m[buses[i].index] = i;
  }
  return m;
}

// --------------------------------------------------------------------------
// Find slack bus index (0-based)
// --------------------------------------------------------------------------
int find_slack_bus(const std::vector<ACBus>& buses) {
  for (int i = 0; i < static_cast<int>(buses.size()); ++i) {
    if (buses[i].bus_type == BusType::SLACK) {
      return i;
    }
  }
  return buses.empty() ? -1 : 0;
}

// --------------------------------------------------------------------------
// DC OPF Formulation Builder
// --------------------------------------------------------------------------
struct DCOPFFormulation {
  int nb{0};         // Number of buses
  int ng{0};         // Number of generators
  int nl{0};         // Number of branches
  int slack{0};      // Slack bus index
  int n_shed{0};     // Number of load shedding variables (0 or nb)
  
  // Variable layout in x = [theta; Pg; Pf; dpd]
  // theta: nb angles (slack fixed to 0)
  // Pg: ng generator outputs
  // Pf: nl branch flows (optional, helps with flow limits)
  // dpd: n_shed load shedding slack variables (one per bus)
  int nvar{0};
  int pf_offset{0};  // starting index of Pf variables
  int shed_offset{0}; // starting index of dpd variables
  
  // Index helpers
  int i_theta(int bus) const { return bus; }
  int i_pg(int gen) const { return nb + gen; }
  int i_pf(int br) const { return pf_offset + br; }
  int i_dpd(int bus) const { return shed_offset + bus; }
  
  // LP model (for linearized costs)
  engine::LPModel lp;
  
  // QP model (for true quadratic costs)
  engine::QPModel qp;
  
  // Generator map: gen_var -> original index in sys.ac.generators
  std::vector<int> gen_map;
  
  // Branch map: branch_var -> original index in sys.ac.branches
  std::vector<int> branch_map;
  
  // Bus map: bus_id -> position
  std::unordered_map<int, int> bus_map;
  
  // Store base_mva for result extraction
  double base_mva{100.0};
};

// --------------------------------------------------------------------------
// Build DC OPF LP formulation from HybridPowerSystem
// --------------------------------------------------------------------------
DCOPFFormulation build_dc_opf_lp(const HybridPowerSystem& sys,
                                  const DCOPFOptions& opt) {
  using Triplet = Eigen::Triplet<double>;
  
  DCOPFFormulation form;
  const auto& buses = sys.ac.buses;
  const auto& gens = sys.ac.generators;
  const auto& branches = sys.ac.branches;
  const double base_mva = std::max(sys.ac.base_mva, 1.0);
  
  form.nb = static_cast<int>(buses.size());
  form.nl = static_cast<int>(branches.size());
  form.slack = find_slack_bus(buses);
  form.bus_map = build_bus_map(buses);
  
  if (form.nb == 0 || form.slack < 0) {
    return form;
  }
  
  // Count active generators
  form.ng = 0;
  for (size_t gi = 0; gi < gens.size(); ++gi) {
    if (gens[gi].in_service) {
      form.gen_map.push_back(static_cast<int>(gi));
      ++form.ng;
    }
  }
  
  // Count active branches
  int n_active_br = 0;
  for (size_t bi = 0; bi < branches.size(); ++bi) {
    if (branches[bi].in_service) {
      form.branch_map.push_back(static_cast<int>(bi));
      ++n_active_br;
    }
  }
  
  // Variable layout: [theta(nb), Pg(ng), Pf(n_active_br), dpd(n_shed)]
  const bool include_pf = opt.include_branch_limits;
  const int n_pf_vars = include_pf ? n_active_br : 0;
  form.n_shed = opt.load_shedding ? form.nb : 0;
  form.pf_offset = form.nb + form.ng;
  form.shed_offset = form.nb + form.ng + n_pf_vars;
  form.nvar = form.nb + form.ng + n_pf_vars + form.n_shed;
  
  // --------------------------------------------------------------------------
  // Objective: min Σ c_i * Pg_i
  // For quadratic costs (c2*Pg^2 + c1*Pg + c0), we linearize using the 
  // marginal cost at the midpoint of the operating range:
  //   c_linear = c1 + 2*c2 * (Pmin + Pmax)/2 = c1 + c2*(Pmin + Pmax)
  // This provides a reasonable approximation for economic dispatch.
  // --------------------------------------------------------------------------
  form.lp.sense = engine::Sense::Minimize;
  form.lp.c = Eigen::VectorXd::Zero(form.nvar);
  
  for (int k = 0; k < form.ng; ++k) {
    const int gi = form.gen_map[k];
    const auto& gen = gens[gi];
    // Linearize quadratic cost around the midpoint
    double pg_mid = 0.5 * (gen.pmin_mw + gen.pmax_mw);
    double c_linear = gen.cost_c1 + 2.0 * gen.cost_c2 * pg_mid;
    // If c_linear is still zero (no cost data), use a small positive value
    // to ensure the problem has a meaningful objective
    if (std::abs(c_linear) < 1e-9) {
      c_linear = 1.0;  // Default marginal cost of $1/MWh
    }
    form.lp.c[form.i_pg(k)] = c_linear;
  }
  
  // --------------------------------------------------------------------------
  // Variable bounds
  // --------------------------------------------------------------------------
  form.lp.vars.resize(form.nvar);
  
  // Theta bounds: slack fixed to 0, others free (±π)
  for (int i = 0; i < form.nb; ++i) {
    if (i == form.slack) {
      form.lp.vars[form.i_theta(i)] = {engine::VarType::Continuous, 0.0, 0.0, ""};
    } else {
      form.lp.vars[form.i_theta(i)] = {engine::VarType::Continuous, -kPi, kPi, ""};
    }
  }
  
  // Pg bounds: Pmin ≤ Pg ≤ Pmax (in p.u.)
  for (int k = 0; k < form.ng; ++k) {
    const int gi = form.gen_map[k];
    const auto& gen = gens[gi];
    double pmin = gen.pmin_mw / base_mva;
    double pmax = gen.pmax_mw / base_mva;
    // Clamp to reasonable values
    pmin = std::max(pmin, -kHugeBound);
    pmax = std::min(pmax, kHugeBound);
    form.lp.vars[form.i_pg(k)] = {engine::VarType::Continuous, pmin, pmax, ""};
  }
  
  // Pf bounds: |Pf| ≤ rate_a (in p.u.)
  if (include_pf) {
    for (int k = 0; k < n_active_br; ++k) {
      const int bi = form.branch_map[k];
      const auto& br = branches[bi];
      double rate = br.rate_a_mva / base_mva;
      if (rate <= 1e-9) rate = kHugeBound;  // No limit
      rate *= opt.branch_limit_margin;
      form.lp.vars[form.i_pf(k)] = {engine::VarType::Continuous, -rate, rate, ""};
    }
  }
  
  // --------------------------------------------------------------------------
  // Compute net demand at each bus (needed for load shedding bounds + RHS)
  // --------------------------------------------------------------------------
  std::vector<double> pd_pu(form.nb, 0.0);
  for (int i = 0; i < form.nb; ++i) {
    pd_pu[i] = buses[i].pd_mw / base_mva;
  }
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    auto it = form.bus_map.find(ld.bus);
    if (it == form.bus_map.end()) continue;
    pd_pu[it->second] += (ld.p_mw * ld.scaling) / base_mva;
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    auto it = form.bus_map.find(sg.bus);
    if (it == form.bus_map.end()) continue;
    pd_pu[it->second] -= (sg.p_mw * sg.scaling) / base_mva;
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    auto it = form.bus_map.find(rg.bus);
    if (it == form.bus_map.end()) continue;
    pd_pu[it->second] -= rg.p_mw / base_mva;
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    auto it = form.bus_map.find(pv.bus);
    if (it == form.bus_map.end()) continue;
    pd_pu[it->second] -= pv.p_mw / base_mva;
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    auto it = form.bus_map.find(st.bus);
    if (it == form.bus_map.end()) continue;
    pd_pu[it->second] -= st.p_mw / base_mva;
  }

  // --------------------------------------------------------------------------
  // Load shedding variables: dpd (one per bus)
  // 0 ≤ dpd_i ≤ max(pd_pu_i, 0) — can only shed positive demand
  // Cost: VOLL × dpd_i × base_mva (in $/h)
  // --------------------------------------------------------------------------
  if (form.n_shed > 0) {
    // Compute VOLL: auto-determine from max generator marginal cost
    double voll_effective = opt.voll;
    if (voll_effective <= 0.0) {
      double max_marginal = 0.0;
      for (int k = 0; k < form.ng; ++k) {
        const int gi = form.gen_map[k];
        const auto& gen = gens[gi];
        double mc = gen.cost_c1 + 2.0 * gen.cost_c2 * gen.pmax_mw;
        max_marginal = std::max(max_marginal, mc);
      }
      voll_effective = std::max(100.0, 1.1 * max_marginal);
    }
    
    for (int i = 0; i < form.nb; ++i) {
      double ub = std::max(pd_pu[i], 0.01 / base_mva);
      form.lp.vars[form.i_dpd(i)] = {engine::VarType::Continuous, 0.0, ub, ""};
      // LP cost coefficient for dpd: VOLL × base_mva (converts pu to MW)
      form.lp.c[form.i_dpd(i)] = voll_effective * base_mva;
    }
  }

  // --------------------------------------------------------------------------
  // Equality constraints: Power balance at each bus (including slack)
  // Σ Pg_i - dpd_i - Σ Pd_i = Σ Pf_ij (for bus i)
  // 
  // In DC power flow: Pf_ij = (θ_i - θ_j) / x_ij  (ignore r, assume x >> r)
  // 
  // Formulation with explicit Pf variables:
  //   Constraint 1 (balance): Σ Pg - Pd = Σ Pf (at each bus)
  //   Constraint 2 (flow def): Pf_ij = (θ_i - θ_j) / x_ij
  // --------------------------------------------------------------------------
  const int n_eq = form.nb + (include_pf ? n_active_br : 0);
  
  std::vector<Triplet> eq_trips;
  eq_trips.reserve(static_cast<size_t>(n_eq) * 4);
  
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(n_eq);
  
  // --- Power balance equations for ALL buses ---
  // Row index for balance equations
  int row = 0;
  std::vector<int> balance_row(form.nb, -1);
  for (int i = 0; i < form.nb; ++i) {
    balance_row[i] = row++;
  }
  
  // RHS: Pd (demand is positive, net injection = Pg + net_import = Pd)
  // pd_pu was already computed above.
  for (int i = 0; i < form.nb; ++i) {
    beq[balance_row[i]] = pd_pu[i];
  }
  
  // Generator injection coefficients
  for (int k = 0; k < form.ng; ++k) {
    const int gi = form.gen_map[k];
    const auto& gen = gens[gi];
    auto it = form.bus_map.find(gen.bus);
    if (it == form.bus_map.end()) continue;
    const int bi = it->second;
    eq_trips.emplace_back(balance_row[bi], form.i_pg(k), 1.0);
  }
  
  // Load shedding coefficients: dpd reduces effective demand at each bus.
  // Power balance: Σ Pg - (Pd - dpd) = Σ Pf  →  Σ Pg + dpd = Pd + Σ Pf
  // So dpd enters the LHS with coefficient +1 (like generation).
  for (int i = 0; i < form.n_shed; ++i) {
    eq_trips.emplace_back(balance_row[i], form.i_dpd(i), 1.0);
  }
  
  if (include_pf) {
    // Flow injection coefficients: +Pf at from_bus, -Pf at to_bus
    for (int k = 0; k < n_active_br; ++k) {
      const int bri = form.branch_map[k];
      const auto& br = branches[bri];
      auto it_f = form.bus_map.find(br.from_bus);
      auto it_t = form.bus_map.find(br.to_bus);
      if (it_f == form.bus_map.end() || it_t == form.bus_map.end()) continue;
      const int fi = it_f->second;
      const int ti = it_t->second;
      
      // Pf positive = power from 'from' to 'to'
      // From bus exports: -Pf
      // To bus imports: +Pf
      eq_trips.emplace_back(balance_row[fi], form.i_pf(k), -1.0);
      eq_trips.emplace_back(balance_row[ti], form.i_pf(k), 1.0);
    }
    
    // --- Flow definition equations: Pf_ij - (θ_i - θ_j)/x = 0 ---
    for (int k = 0; k < n_active_br; ++k) {
      const int bri = form.branch_map[k];
      const auto& br = branches[bri];
      auto it_f = form.bus_map.find(br.from_bus);
      auto it_t = form.bus_map.find(br.to_bus);
      if (it_f == form.bus_map.end() || it_t == form.bus_map.end()) continue;
      const int fi = it_f->second;
      const int ti = it_t->second;
      
      // b = 1/x (susceptance)
      double x_pu = br.x_pu;
      if (std::abs(x_pu) < 1e-12) x_pu = 1e-6;  // Avoid division by zero
      const double b = 1.0 / x_pu;
      
      const int eq_row = row++;
      // Pf_k - b*(θ_f - θ_t) = 0
      eq_trips.emplace_back(eq_row, form.i_pf(k), 1.0);
      eq_trips.emplace_back(eq_row, form.i_theta(fi), -b);
      eq_trips.emplace_back(eq_row, form.i_theta(ti), b);
    }
  } else {
    // Without explicit Pf variables: incorporate branch flows directly in balance
    // Σ_j b_ij (θ_i - θ_j) = Pg_i - Pd_i for each bus i
    // This means: Σ_j b_ij θ_i - Σ_j b_ij θ_j = Σ_j b_ij θ_i - Σ_j b_ij θ_j
    //           = B_ii θ_i - Σ_{j≠i} B_ij θ_j
    // where B_ii = Σ_j b_ij,  B_ij = -b_ij
    
    // Build B matrix (bus susceptance matrix)
    std::vector<std::vector<std::pair<int, double>>> B_entries(form.nb);
    std::vector<double> B_diag(form.nb, 0.0);
    
    for (size_t bri = 0; bri < branches.size(); ++bri) {
      const auto& br = branches[bri];
      if (!br.in_service) continue;
      
      auto it_f = form.bus_map.find(br.from_bus);
      auto it_t = form.bus_map.find(br.to_bus);
      if (it_f == form.bus_map.end() || it_t == form.bus_map.end()) continue;
      const int fi = it_f->second;
      const int ti = it_t->second;
      
      double x_pu = br.x_pu;
      if (std::abs(x_pu) < 1e-12) x_pu = 1e-6;
      const double b = 1.0 / x_pu;
      
      B_diag[fi] += b;
      B_diag[ti] += b;
      B_entries[fi].emplace_back(ti, -b);
      B_entries[ti].emplace_back(fi, -b);
    }
    
    // Add B matrix entries to equality constraints
    for (int i = 0; i < form.nb; ++i) {
      if (i == form.slack) continue;
      const int eq_row = balance_row[i];
      
      // Diagonal: B_ii * θ_i
      eq_trips.emplace_back(eq_row, form.i_theta(i), B_diag[i]);
      
      // Off-diagonal: B_ij * θ_j
      for (const auto& [j, bij] : B_entries[i]) {
        if (j != form.slack) {
          eq_trips.emplace_back(eq_row, form.i_theta(j), bij);
        }
      }
    }
  }
  
  // Assemble equality constraint matrix
  form.lp.Aeq.resize(n_eq, form.nvar);
  form.lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  form.lp.Aeq.makeCompressed();
  form.lp.beq = beq;
  
  // --------------------------------------------------------------------------
  // Inequality constraints: Empty for basic DC OPF
  // Branch limits are handled via variable bounds on Pf
  // --------------------------------------------------------------------------
  form.lp.A.resize(0, form.nvar);
  form.lp.b.resize(0);
  
  return form;
}

// --------------------------------------------------------------------------
// Build DC OPF QP formulation (true quadratic costs)
// min  Σ (c2*Pg² + c1*Pg + c0)  -->  min 0.5*x'Qx + c'x
// s.t. same linear constraints as LP
// --------------------------------------------------------------------------
void build_dc_opf_qp(DCOPFFormulation& form, const HybridPowerSystem& sys) {
  using Triplet = Eigen::Triplet<double>;
  
  const auto& gens = sys.ac.generators;
  const double base_mva = std::max(sys.ac.base_mva, 1.0);
  form.base_mva = base_mva;
  
  // Copy constraint structure from LP model to QP model
  form.qp.sense = engine::Sense::Minimize;
  form.qp.A = form.lp.A;
  form.qp.b = form.lp.b;
  form.qp.Aeq = form.lp.Aeq;
  form.qp.beq = form.lp.beq;
  form.qp.vars = form.lp.vars;
  
  // Build quadratic cost: Q diagonal for generator Pg variables
  // Original cost: c2*Pg_MW² + c1*Pg_MW + c0
  // In p.u.: c2*(Pg_pu*Sbase)² + c1*(Pg_pu*Sbase) + c0
  //        = c2*Sbase²*Pg_pu² + c1*Sbase*Pg_pu + c0
  // So Q coefficient for Pg_pu: 2*c2*Sbase² (the 0.5 in 0.5*x'Qx cancels)
  // And c coefficient for Pg_pu: c1*Sbase
  
  std::vector<Triplet> Q_trips;
  form.qp.c = Eigen::VectorXd::Zero(form.nvar);
  
  for (int k = 0; k < form.ng; ++k) {
    const int gi = form.gen_map[k];
    const auto& gen = gens[gi];
    
    // Quadratic term: need 2*c2*Sbase² because obj = 0.5*x'Qx
    double q_coef = 2.0 * gen.cost_c2 * base_mva * base_mva;
    if (q_coef > 1e-12) {
      Q_trips.emplace_back(form.i_pg(k), form.i_pg(k), q_coef);
    }
    
    // Linear term: c1*Sbase
    double c_coef = gen.cost_c1 * base_mva;
    // Use small positive cost if no cost data
    if (std::abs(q_coef) < 1e-12 && std::abs(c_coef) < 1e-12) {
      c_coef = 1.0 * base_mva;  // Default $1/MWh
    }
    form.qp.c[form.i_pg(k)] = c_coef;
  }
  
  // Copy load shedding penalty costs from LP model (linear cost, no quadratic)
  for (int i = 0; i < form.n_shed; ++i) {
    form.qp.c[form.i_dpd(i)] = form.lp.c[form.i_dpd(i)];
  }
  
  form.qp.Q.resize(form.nvar, form.nvar);
  form.qp.Q.setFromTriplets(Q_trips.begin(), Q_trips.end());
  form.qp.Q.makeCompressed();
}

// --------------------------------------------------------------------------
// Compute QP objective value: 0.5*x'Qx + c'x + constant
// --------------------------------------------------------------------------
double compute_qp_objective(const DCOPFFormulation& form,
                            const Eigen::VectorXd& x,
                            const HybridPowerSystem& sys) {
  const auto& gens = sys.ac.generators;
  double obj = 0.0;
  
  // Quadratic + linear terms (already computed in optimization)
  obj = 0.5 * x.dot(form.qp.Q * x) + form.qp.c.dot(x);
  
  // Add constant terms (c0)
  for (int k = 0; k < form.ng; ++k) {
    const int gi = form.gen_map[k];
    obj += gens[gi].cost_c0;
  }
  
  return obj;
}

// --------------------------------------------------------------------------
// Extract results from LP solution
// --------------------------------------------------------------------------
DCOPFResult extract_dc_opf_result(const DCOPFFormulation& form,
                                   const engine::SolveResult& sol,
                                   const HybridPowerSystem& sys,
                                   double runtime_sec) {
  DCOPFResult result;
  const auto& branches = sys.ac.branches;
  const double base_mva = std::max(sys.ac.base_mva, 1.0);
  
  result.runtime_sec = runtime_sec;
  result.converged = sol.stats.success;
  result.iterations = sol.stats.iterations;
  result.objective = sol.stats.objective;
  result.status = sol.stats.status;
  result.solver_name = sol.stats.solver_name;
  
  if (!result.converged || sol.x.size() < form.nvar) {
    return result;
  }
  
  // Extract bus angles
  result.va.resize(form.nb);
  for (int i = 0; i < form.nb; ++i) {
    result.va[i] = sol.x[form.i_theta(i)];
  }
  
  // Extract generator dispatch (convert from p.u. to MW)
  const size_t n_gen = sys.ac.generators.size();
  result.pg_mw.assign(n_gen, 0.0);
  for (int k = 0; k < form.ng; ++k) {
    const int gi = form.gen_map[k];
    result.pg_mw[gi] = sol.x[form.i_pg(k)] * base_mva;
  }
  
  // Compute branch flows
  const size_t n_br = branches.size();
  result.pf_mw.assign(n_br, 0.0);
  
  const bool include_pf = !form.branch_map.empty() && form.pf_offset < form.shed_offset;
  
  if (include_pf) {
    // Extract from Pf variables
    for (size_t k = 0; k < form.branch_map.size(); ++k) {
      const int bi = form.branch_map[k];
      result.pf_mw[bi] = sol.x[form.i_pf(static_cast<int>(k))] * base_mva;
    }
  } else {
    // Compute from angles: Pf_ij = (θ_i - θ_j) / x_ij
    for (size_t bi = 0; bi < n_br; ++bi) {
      const auto& br = branches[bi];
      if (!br.in_service) continue;
      
      auto it_f = form.bus_map.find(br.from_bus);
      auto it_t = form.bus_map.find(br.to_bus);
      if (it_f == form.bus_map.end() || it_t == form.bus_map.end()) continue;
      
      const double theta_f = result.va[it_f->second];
      const double theta_t = result.va[it_t->second];
      double x_pu = br.x_pu;
      if (std::abs(x_pu) < 1e-12) x_pu = 1e-6;
      
      result.pf_mw[bi] = (theta_f - theta_t) / x_pu * base_mva;
    }
  }
  
  // Extract LMPs (dual variables on balance equations)
  // Note: Simplex solvers typically provide duals, but this depends on the backend
  result.lmp.assign(form.nb, 0.0);
  // TODO: Extract duals from solver if available
  
  // Extract load shedding results
  if (form.n_shed > 0) {
    result.load_shedding_mw.resize(form.nb, 0.0);
    result.total_load_shedding_mw = 0.0;
    for (int i = 0; i < form.nb; ++i) {
      double shed = sol.x[form.i_dpd(i)] * base_mva;
      if (shed < 1e-6) shed = 0.0;  // Clean up numerical noise
      result.load_shedding_mw[i] = shed;
      result.total_load_shedding_mw += shed;
    }
  }
  
  return result;
}

}  // namespace

// --------------------------------------------------------------------------
// Public API: solve_dc_opf
// --------------------------------------------------------------------------
DCOPFResult solve_dc_opf(const HybridPowerSystem& sys,
                         const DCOPFOptions& opt) {
  auto start_time = std::chrono::high_resolution_clock::now();
  
  DCOPFResult result;
  
  // Build LP formulation (constraint structure)
  DCOPFFormulation form = build_dc_opf_lp(sys, opt);
  
  if (form.nb == 0 || form.ng == 0) {
    result.status = "Empty system or no generators";
    result.converged = false;
    return result;
  }
  
  // Build QP model with true quadratic costs
  build_dc_opf_qp(form, sys);
  
  if (opt.verbose) {
    spdlog::info("DC OPF: {} buses, {} generators, {} branches, {} variables",
                 form.nb, form.ng, static_cast<int>(form.branch_map.size()),
                 form.nvar);
  }
  
  // Select solver backend
  engine::SolveResult sol;
  bool use_qp = false;  // Track if we used QP solver (for objective computation)
  
  // NativeQP: Use LCQP solver with true quadratic costs
  auto try_native_qp = [&]() -> bool {
    engine::LCQPOptions qp_opt;
    qp_opt.max_iter = opt.max_iterations;
    qp_opt.tol_primal = opt.feasibility_tol;
    qp_opt.tol_dual = opt.optimality_tol;
    qp_opt.tol_gap = opt.optimality_tol;
    qp_opt.verbose = opt.verbose;
    
    engine::NativeLCQPAdapter lcqp(qp_opt);
    sol = lcqp.solve_qp(form.qp);
    sol.stats.solver_name = "NativeLCQP";
    use_qp = true;
    return true;
  };
  
  // Gurobi QP: Use Gurobi with QP model
  auto try_gurobi_qp = [&]() -> bool {
    engine::GurobiAdapter gurobi;
    if (gurobi.available()) {
      sol = gurobi.solve_qp(form.qp);
      use_qp = true;
      return true;
    }
    return false;
  };
  
  // HiGHS: LP only (TODO: add QP support)
  auto try_highs = [&]() -> bool {
    engine::HighsAdapter highs;
    if (highs.available()) {
      sol = highs.solve_lp(form.lp);
      return true;
    }
    return false;
  };
  
  // Native simplex: LP with linearized costs
  auto try_native_simplex = [&]() -> bool {
    engine::SimplexOptions simp_opt;
    simp_opt.max_iter = opt.max_iterations;
    simp_opt.feasibility_tol = opt.feasibility_tol;
    simp_opt.optimality_tol = opt.optimality_tol;
    
    auto simp_result = engine::solve_lp_with_basis(form.lp, simp_opt, nullptr);
    sol.stats.success = simp_result.result.stats.success;
    sol.stats.iterations = simp_result.result.stats.iterations;
    sol.stats.objective = simp_result.result.stats.objective;
    sol.stats.status = simp_result.result.stats.success ? "Optimal" : "Not Optimal";
    sol.stats.solver_name = "NativeDualSimplex";
    sol.x = simp_result.result.x;
    return true;
  };
  
  bool solved = false;
  switch (opt.solver) {
    case DCOPFSolverBackend::NativeQP:
      solved = try_native_qp();
      // Fallback to native simplex if IPM did not converge.
      if (!sol.stats.success) {
        solved = try_native_simplex();
      }
      break;
      
    case DCOPFSolverBackend::Gurobi:
      solved = try_gurobi_qp();  // Prefer QP with Gurobi
      if (!solved) {
        result.status = "Gurobi not available";
        result.converged = false;
        return result;
      }
      break;
      
    case DCOPFSolverBackend::HiGHS:
      solved = try_highs();
      if (!solved) {
        result.status = "HiGHS not available";
        result.converged = false;
        return result;
      }
      break;
      
    case DCOPFSolverBackend::Native:
      solved = try_native_simplex();
      break;
      
    case DCOPFSolverBackend::Auto:
    default:
      // Try NativeQP first (best for quadratic costs), then Gurobi QP, then LP solvers
      solved = try_native_qp();
      if (!solved || !sol.stats.success) {
        solved = try_gurobi_qp();
      }
      if (!solved || !sol.stats.success) {
        solved = try_highs();
      }
      if (!solved || !sol.stats.success) {
        solved = try_native_simplex();
      }
      break;
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  double runtime_sec = std::chrono::duration<double>(end_time - start_time).count();
  
  // Extract results
  result = extract_dc_opf_result(form, sol, sys, runtime_sec);
  
  // For QP solvers, recompute the true quadratic objective including constant terms
  if (use_qp && result.converged && sol.x.size() >= form.nvar) {
    result.objective = compute_qp_objective(form, sol.x, sys);
  }
  
  if (opt.verbose) {
    spdlog::info("DC OPF: converged={} obj={:.4f} solver={} time={:.3f}s",
                 result.converged, result.objective, result.solver_name,
                 result.runtime_sec);
  }
  
  return result;
}

// --------------------------------------------------------------------------
// Public API: check_dc_opf_feasibility
// --------------------------------------------------------------------------
std::tuple<bool, double, std::string>
check_dc_opf_feasibility(const HybridPowerSystem& sys,
                         const DCOPFResult& result,
                         double tol) {
  const auto& buses = sys.ac.buses;
  const auto& gens = sys.ac.generators;
  const auto& branches = sys.ac.branches;
  const double base_mva = std::max(sys.ac.base_mva, 1.0);
  
  double max_viol = 0.0;
  std::string viol_desc;
  
  if (result.va.size() != buses.size()) {
    return {false, 1e20, "Result size mismatch"};
  }
  
  auto bus_map = build_bus_map(buses);
  
  // Check generator limits
  for (size_t gi = 0; gi < gens.size(); ++gi) {
    const auto& gen = gens[gi];
    if (!gen.in_service) continue;
    
    const double pg = result.pg_mw[gi];
    const double pmin = gen.pmin_mw;
    const double pmax = gen.pmax_mw;
    
    if (pg < pmin - tol * base_mva) {
      double viol = pmin - pg;
      if (viol > max_viol) {
        max_viol = viol;
        viol_desc = "Gen " + std::to_string(gi) + " Pmin violation: " +
                    std::to_string(pg) + " < " + std::to_string(pmin);
      }
    }
    if (pg > pmax + tol * base_mva) {
      double viol = pg - pmax;
      if (viol > max_viol) {
        max_viol = viol;
        viol_desc = "Gen " + std::to_string(gi) + " Pmax violation: " +
                    std::to_string(pg) + " > " + std::to_string(pmax);
      }
    }
  }
  
  // Check branch flow limits
  for (size_t bi = 0; bi < branches.size(); ++bi) {
    const auto& br = branches[bi];
    if (!br.in_service) continue;
    if (br.rate_a_mva <= 1e-9) continue;  // No limit
    
    const double pf = std::abs(result.pf_mw[bi]);
    if (pf > br.rate_a_mva + tol * base_mva) {
      double viol = pf - br.rate_a_mva;
      if (viol > max_viol) {
        max_viol = viol;
        viol_desc = "Branch " + std::to_string(bi) + " flow violation: " +
                    std::to_string(pf) + " > " + std::to_string(br.rate_a_mva);
      }
    }
  }
  
  // Check power balance (approximately)
  std::vector<double> p_net(buses.size(), 0.0);
  
  // Subtract demand
  for (size_t i = 0; i < buses.size(); ++i) {
    p_net[i] -= buses[i].pd_mw;
  }
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    auto it = bus_map.find(ld.bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] -= ld.p_mw * ld.scaling;
  }
  
  // Add generation
  for (size_t gi = 0; gi < gens.size(); ++gi) {
    if (!gens[gi].in_service) continue;
    auto it = bus_map.find(gens[gi].bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] += result.pg_mw[gi];
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    auto it = bus_map.find(sg.bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] += sg.p_mw * sg.scaling;
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    auto it = bus_map.find(rg.bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] += rg.p_mw;
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    auto it = bus_map.find(pv.bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] += pv.p_mw;
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    auto it = bus_map.find(st.bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] += st.p_mw;
  }
  
  // Subtract branch outflows, add inflows
  for (size_t bi = 0; bi < branches.size(); ++bi) {
    const auto& br = branches[bi];
    if (!br.in_service) continue;
    
    auto it_f = bus_map.find(br.from_bus);
    auto it_t = bus_map.find(br.to_bus);
    if (it_f == bus_map.end() || it_t == bus_map.end()) continue;
    
    // Positive Pf means power flows from 'from' to 'to'
    p_net[it_f->second] -= result.pf_mw[bi];
    p_net[it_t->second] += result.pf_mw[bi];
  }
  
  // Check balance at each bus (skip slack - it absorbs imbalance)
  int slack = find_slack_bus(buses);
  for (size_t i = 0; i < buses.size(); ++i) {
    if (static_cast<int>(i) == slack) continue;
    double imbalance = std::abs(p_net[i]);
    if (imbalance > tol * base_mva) {
      if (imbalance > max_viol) {
        max_viol = imbalance;
        viol_desc = "Bus " + std::to_string(i) + " power imbalance: " +
                    std::to_string(p_net[i]) + " MW";
      }
    }
  }
  
  bool feasible = (max_viol <= tol * base_mva);
  return {feasible, max_viol, viol_desc};
}

}  // namespace hacdcpf::opf
