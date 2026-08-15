#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "hacdcpf/graph/graph.hpp"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <spdlog/spdlog.h>

#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/solver/external/adapters.hpp"
#include "hacdcpf/engine/kernel/ipm/lcqp_solver.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/projection/result_attribution.hpp"

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
    auto [it, inserted] = m.emplace(buses[i].index, i);
    if (!inserted) {
      throw std::invalid_argument(
          "DC OPF: duplicate bus ID " + std::to_string(buses[i].index) +
          " at positions " + std::to_string(it->second) +
          " and " + std::to_string(i) +
          " — topology is invalid; fix the input before solving");
    }
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
  int pwl_offset{0};  // starting index of convex-combination lambda variables
  int pwl_segments_effective{0};
  
  // Index helpers
  int i_theta(int bus) const { return bus; }
  int i_pg(int gen) const { return nb + gen; }
  int i_pf(int br) const { return pf_offset + br; }
  int i_dpd(int bus) const { return shed_offset + bus; }
  int i_pwl(int gen, int point) const {
    return pwl_point_offset_by_gen[static_cast<size_t>(gen)] + point;
  }
  
  // LP model (for linearized costs)
  engine::LPModel lp;
  
  // QP model (for true quadratic costs)
  engine::QPModel qp;
  
  // Generator map: gen_var -> original index in sys.ac.generators
  std::vector<int> gen_map;
  
  // Branch map: branch_var -> original index in sys.ac.branches
  std::vector<int> branch_map;

  // Per active generator: offset/count of PWL lambda points, or -1/0 when the
  // generator has no convex quadratic term and remains linear in Pg.
  std::vector<int> pwl_point_offset_by_gen;
  std::vector<int> pwl_point_count_by_gen;
  
  // Bus map: bus_id -> position
  std::unordered_map<int, int> bus_map;

  // Connected-component structure used both to fix one angle reference per
  // island and to build the reduced-Laplacian NativeLCQP start.
  std::vector<int> component_of_bus;
  std::vector<std::vector<int>> component_buses;
  std::vector<int> component_references;
  std::vector<double> pd_pu;
  
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

  // Zimmerman et al. (2011), MATPOWER DC model: each energized connected
  // component has a one-dimensional angle nullspace, so fix exactly one
  // reference before forming any reduced Laplacian projection.
  std::vector<int> parent(static_cast<std::size_t>(form.nb));
  std::iota(parent.begin(), parent.end(), 0);
  const auto root = [&](int node) {
    int r = node;
    while (parent[static_cast<std::size_t>(r)] != r)
      r = parent[static_cast<std::size_t>(r)];
    while (parent[static_cast<std::size_t>(node)] != node) {
      const int next = parent[static_cast<std::size_t>(node)];
      parent[static_cast<std::size_t>(node)] = r;
      node = next;
    }
    return r;
  };
  for (const auto& br : branches) {
    if (!br.in_service) continue;
    const auto from = form.bus_map.find(br.from_bus);
    const auto to = form.bus_map.find(br.to_bus);
    if (from == form.bus_map.end() || to == form.bus_map.end()) continue;
    const int rf = root(from->second);
    const int rt = root(to->second);
    if (rf != rt) parent[static_cast<std::size_t>(rt)] = rf;
  }
  std::unordered_map<int, int> root_to_component;
  form.component_of_bus.resize(static_cast<std::size_t>(form.nb));
  for (int bus = 0; bus < form.nb; ++bus) {
    const int r = root(bus);
    auto [it, inserted] = root_to_component.emplace(
        r, static_cast<int>(root_to_component.size()));
    if (inserted) form.component_buses.emplace_back();
    form.component_of_bus[static_cast<std::size_t>(bus)] = it->second;
    form.component_buses[static_cast<std::size_t>(it->second)].push_back(bus);
  }
  form.component_references.reserve(form.component_buses.size());
  for (const auto& component : form.component_buses) {
    const auto slack_it = std::find(component.begin(), component.end(), form.slack);
    form.component_references.push_back(
        slack_it != component.end() ? form.slack : component.front());
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
  
  // Variable layout: [theta(nb), Pg(ng), Pf(n_active_br), dpd(n_shed), lambda]
  const bool include_pf = opt.include_branch_limits;
  const int n_pf_vars = include_pf ? n_active_br : 0;
  form.n_shed = opt.load_shedding ? form.nb : 0;
  form.pf_offset = form.nb + form.ng;
  form.shed_offset = form.nb + form.ng + n_pf_vars;
  form.pwl_offset = form.shed_offset + form.n_shed;
  form.pwl_point_offset_by_gen.assign(static_cast<size_t>(form.ng), -1);
  form.pwl_point_count_by_gen.assign(static_cast<size_t>(form.ng), 0);
  const int requested_segments = std::clamp(opt.pwl_segments, 1, 1000);
  int n_pwl_vars = 0;
  if (!opt.compact_quadratic_model) {
    for (int k = 0; k < form.ng; ++k) {
      const auto& gen = gens[static_cast<size_t>(form.gen_map[k])];
      if (gen.cost_c2 <= 1e-12 || !(gen.pmax_mw > gen.pmin_mw)) continue;
      const int point_count = requested_segments + 1;
      form.pwl_point_offset_by_gen[static_cast<size_t>(k)] =
          form.pwl_offset + n_pwl_vars;
      form.pwl_point_count_by_gen[static_cast<size_t>(k)] = point_count;
      n_pwl_vars += point_count;
      form.pwl_segments_effective = requested_segments;
    }
  }
  form.nvar = form.pwl_offset + n_pwl_vars;
  
  // --------------------------------------------------------------------------
  // Objective: min Σ c_i(Pg_i). Convex quadratic generator costs use a
  // lambda-form piecewise-linear interpolation with `pwl_segments` intervals;
  // linear costs stay directly on Pg. The QP builder below replaces this
  // objective with the exact quadratic while retaining equivalent Pg bounds.
  // --------------------------------------------------------------------------
  form.lp.sense = engine::Sense::Minimize;
  form.lp.c = Eigen::VectorXd::Zero(form.nvar);
  
  for (int k = 0; k < form.ng; ++k) {
    const int gi = form.gen_map[k];
    const auto& gen = gens[gi];
    if (form.pwl_point_count_by_gen[static_cast<size_t>(k)] > 0) continue;
    double c_linear = gen.cost_c1;
    // If c_linear is still zero (no cost data), use a small positive value
    // to ensure the problem has a meaningful objective
    if (std::abs(c_linear) < 1e-9) {
      c_linear = 1.0;  // Default marginal cost of $1/MWh
    }
    // Pg is a p.u. variable in the LP.  Convert the $/MWh marginal cost to
    // a coefficient on Pg_pu so the reported objective and equality duals stay
    // in MW-based engineering units.
    form.lp.c[form.i_pg(k)] = c_linear * base_mva;
  }
  
  // --------------------------------------------------------------------------
  // Variable bounds
  // --------------------------------------------------------------------------
  form.lp.vars.resize(form.nvar);
  
  // Theta bounds: one reference per connected component fixed to zero.
  std::vector<unsigned char> is_reference(static_cast<std::size_t>(form.nb), 0);
  for (int ref : form.component_references)
    is_reference[static_cast<std::size_t>(ref)] = 1;
  for (int i = 0; i < form.nb; ++i) {
    if (is_reference[static_cast<std::size_t>(i)] != 0) {
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

  // PWL lambda bounds and objective samples. Convexity plus minimization makes
  // the lower hull select adjacent breakpoints without binary/SOS2 variables.
  for (int k = 0; k < form.ng; ++k) {
    const int point_count =
        form.pwl_point_count_by_gen[static_cast<size_t>(k)];
    if (point_count == 0) continue;
    const auto& gen = gens[static_cast<size_t>(form.gen_map[k])];
    for (int p = 0; p < point_count; ++p) {
      const double alpha = static_cast<double>(p) /
                           static_cast<double>(point_count - 1);
      const double pg_mw = gen.pmin_mw + alpha * (gen.pmax_mw - gen.pmin_mw);
      const int col = form.i_pwl(k, p);
      form.lp.vars[col] = {engine::VarType::Continuous, 0.0, 1.0, ""};
      form.lp.c[col] = gen.cost_c2 * pg_mw * pg_mw +
                       gen.cost_c1 * pg_mw + gen.cost_c0;
    }
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
  form.pd_pu = pd_pu;

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
  int n_pwl_generators = 0;
  for (int count : form.pwl_point_count_by_gen)
    if (count > 0) ++n_pwl_generators;
  const int n_eq = form.nb + (include_pf ? n_active_br : 0) +
                   2 * n_pwl_generators;
  
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
    
    // Add B matrix entries to equality constraints.
    // The slack bus balance row is included (gen/shedding terms were added above)
    // and needs its susceptance flow contributions — but since θ_slack = 0 is
    // enforced via variable bounds, the diagonal term and any column for j==slack
    // evaluate to zero and can be omitted.  All other non-slack buses follow the
    // standard formulation (BUG-3 fix: was skipping the entire slack bus row).
    for (int i = 0; i < form.nb; ++i) {
      const int eq_row = balance_row[i];
      
      // Reference columns may remain in Aeq because their variable bounds fix
      // them to zero; retaining the full block keeps all components symmetric.
      eq_trips.emplace_back(eq_row, form.i_theta(i), B_diag[i]);
      
      // Off-diagonal: B_ij * θ_j  (skip j==slack since θ_slack = 0)
      for (const auto& [j, bij] : B_entries[i]) {
        eq_trips.emplace_back(eq_row, form.i_theta(j), bij);
      }
    }
  }

  // Convex-combination cost interpolation:
  //   sum_p lambda_gp = 1
  //   Pg_g = sum_p breakpoint_gp * lambda_gp
  for (int k = 0; k < form.ng; ++k) {
    const int point_count =
        form.pwl_point_count_by_gen[static_cast<size_t>(k)];
    if (point_count == 0) continue;
    const auto& gen = gens[static_cast<size_t>(form.gen_map[k])];
    const int convexity_row = row++;
    const int interpolation_row = row++;
    beq[convexity_row] = 1.0;
    eq_trips.emplace_back(interpolation_row, form.i_pg(k), 1.0);
    for (int p = 0; p < point_count; ++p) {
      const double alpha = static_cast<double>(p) /
                           static_cast<double>(point_count - 1);
      const double pg_pu =
          (gen.pmin_mw + alpha * (gen.pmax_mw - gen.pmin_mw)) / base_mva;
      eq_trips.emplace_back(convexity_row, form.i_pwl(k, p), 1.0);
      eq_trips.emplace_back(interpolation_row, form.i_pwl(k, p), -pg_pu);
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

struct DCStructuralWarmStart {
  Eigen::VectorXd x;
  bool built{false};
  int factorizations{0};
  double equality_residual{std::numeric_limits<double>::infinity()};
  std::string status{"not-built"};
};

DCStructuralWarmStart build_dc_structural_warm_start(
    const DCOPFFormulation& form,
    const HybridPowerSystem& sys) {
  DCStructuralWarmStart warm;
  if (form.nvar <= 0 || form.pd_pu.size() != static_cast<std::size_t>(form.nb)) {
    warm.status = "invalid-formulation";
    return warm;
  }

  warm.x = Eigen::VectorXd::Zero(form.nvar);
  const auto& gens = sys.ac.generators;
  const auto& branches = sys.ac.branches;
  const bool include_pf =
      !form.branch_map.empty() && form.pf_offset < form.shed_offset;

  std::vector<std::vector<int>> generators_by_component(
      form.component_buses.size());
  for (int k = 0; k < form.ng; ++k) {
    const auto bus_it = form.bus_map.find(
        gens[static_cast<std::size_t>(form.gen_map[static_cast<std::size_t>(k)])].bus);
    if (bus_it == form.bus_map.end()) continue;
    const int component =
        form.component_of_bus[static_cast<std::size_t>(bus_it->second)];
    generators_by_component[static_cast<std::size_t>(component)].push_back(k);
    const auto& bounds = form.lp.vars[static_cast<std::size_t>(form.i_pg(k))];
    warm.x[form.i_pg(k)] = std::clamp(
        gens[static_cast<std::size_t>(form.gen_map[static_cast<std::size_t>(k)])].pg_mw /
            form.base_mva,
        bounds.lb, bounds.ub);
  }

  // Per-island bounded balancing removes the Laplacian compatibility mode
  // before factorization: 1' (Pg + shed - Pd) = 0. This is the range
  // condition for a graph Laplacian (Zimmerman et al., 2011, DC model).
  for (int component = 0;
       component < static_cast<int>(form.component_buses.size()); ++component) {
    double demand = 0.0;
    for (int bus : form.component_buses[static_cast<std::size_t>(component)])
      demand += form.pd_pu[static_cast<std::size_t>(bus)];
    double generation = 0.0;
    for (int k : generators_by_component[static_cast<std::size_t>(component)])
      generation += warm.x[form.i_pg(k)];
    double remaining = demand - generation;

    if (remaining > 0.0) {
      for (int k : generators_by_component[static_cast<std::size_t>(component)]) {
        const int col = form.i_pg(k);
        const double room = form.lp.vars[static_cast<std::size_t>(col)].ub - warm.x[col];
        const double delta = std::min(remaining, std::max(0.0, room));
        warm.x[col] += delta;
        remaining -= delta;
        if (remaining <= 1e-12) break;
      }
      if (remaining > 1e-12 && form.n_shed > 0) {
        for (int bus : form.component_buses[static_cast<std::size_t>(component)]) {
          const int col = form.i_dpd(bus);
          const double room = form.lp.vars[static_cast<std::size_t>(col)].ub;
          const double delta = std::min(remaining, std::max(0.0, room));
          warm.x[col] = delta;
          remaining -= delta;
          if (remaining <= 1e-12) break;
        }
      }
    } else if (remaining < 0.0) {
      double excess = -remaining;
      for (int k : generators_by_component[static_cast<std::size_t>(component)]) {
        const int col = form.i_pg(k);
        const double room = warm.x[col] - form.lp.vars[static_cast<std::size_t>(col)].lb;
        const double delta = std::min(excess, std::max(0.0, room));
        warm.x[col] -= delta;
        excess -= delta;
        if (excess <= 1e-12) break;
      }
      remaining = -excess;
    }
    if (std::abs(remaining) > 1e-9) {
      warm.status = "component-power-balance-infeasible";
      return warm;
    }
  }

  for (int component = 0;
       component < static_cast<int>(form.component_buses.size()); ++component) {
    const auto& buses = form.component_buses[static_cast<std::size_t>(component)];
    const int reference =
        form.component_references[static_cast<std::size_t>(component)];
    if (buses.size() <= 1) continue;
    std::vector<int> reduced_index(static_cast<std::size_t>(form.nb), -1);
    int reduced_size = 0;
    for (int bus : buses) {
      if (bus != reference)
        reduced_index[static_cast<std::size_t>(bus)] = reduced_size++;
    }
    std::vector<Eigen::Triplet<double>> trips;
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(reduced_size);
    std::vector<double> supply(static_cast<std::size_t>(form.nb), 0.0);
    for (int bus : buses) {
      supply[static_cast<std::size_t>(bus)] =
          (form.n_shed > 0 ? warm.x[form.i_dpd(bus)] : 0.0) -
          form.pd_pu[static_cast<std::size_t>(bus)];
    }
    for (int k : generators_by_component[static_cast<std::size_t>(component)]) {
      const int gi = form.gen_map[static_cast<std::size_t>(k)];
      const int bus = form.bus_map.at(gens[static_cast<std::size_t>(gi)].bus);
      supply[static_cast<std::size_t>(bus)] += warm.x[form.i_pg(k)];
    }
    for (int bus : buses) {
      if (bus == reference) continue;
      const int row = reduced_index[static_cast<std::size_t>(bus)];
      rhs[row] = include_pf ? supply[static_cast<std::size_t>(bus)]
                            : -supply[static_cast<std::size_t>(bus)];
    }
    for (const auto& br : branches) {
      if (!br.in_service) continue;
      const auto from_it = form.bus_map.find(br.from_bus);
      const auto to_it = form.bus_map.find(br.to_bus);
      if (from_it == form.bus_map.end() || to_it == form.bus_map.end()) continue;
      const int from = from_it->second;
      const int to = to_it->second;
      if (form.component_of_bus[static_cast<std::size_t>(from)] != component ||
          form.component_of_bus[static_cast<std::size_t>(to)] != component)
        continue;
      double reactance = br.x_pu;
      if (std::abs(reactance) < 1e-12) reactance = 1e-6;
      const double b = 1.0 / reactance;
      const int rf = reduced_index[static_cast<std::size_t>(from)];
      const int rt = reduced_index[static_cast<std::size_t>(to)];
      if (rf >= 0) trips.emplace_back(rf, rf, b);
      if (rt >= 0) trips.emplace_back(rt, rt, b);
      if (rf >= 0 && rt >= 0) {
        trips.emplace_back(rf, rt, -b);
        trips.emplace_back(rt, rf, -b);
      }
    }
    Eigen::SparseMatrix<double> reduced_laplacian(reduced_size, reduced_size);
    reduced_laplacian.setFromTriplets(trips.begin(), trips.end());
    reduced_laplacian.makeCompressed();
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu;
    lu.analyzePattern(reduced_laplacian);
    lu.factorize(reduced_laplacian);
    ++warm.factorizations;
    if (lu.info() != Eigen::Success) {
      warm.status = "reduced-laplacian-factorization-failed";
      return warm;
    }
    const Eigen::VectorXd theta = lu.solve(rhs);
    if (lu.info() != Eigen::Success || !theta.allFinite()) {
      warm.status = "reduced-laplacian-solve-failed";
      return warm;
    }
    warm.x[form.i_theta(reference)] = 0.0;
    for (int bus : buses) {
      if (bus != reference)
        warm.x[form.i_theta(bus)] =
            theta[reduced_index[static_cast<std::size_t>(bus)]];
    }
  }

  if (include_pf) {
    for (int k = 0; k < static_cast<int>(form.branch_map.size()); ++k) {
      const auto& br = branches[static_cast<std::size_t>(form.branch_map[static_cast<std::size_t>(k)])];
      const int from = form.bus_map.at(br.from_bus);
      const int to = form.bus_map.at(br.to_bus);
      double reactance = br.x_pu;
      if (std::abs(reactance) < 1e-12) reactance = 1e-6;
      warm.x[form.i_pf(k)] =
          (warm.x[form.i_theta(from)] - warm.x[form.i_theta(to)]) / reactance;
    }
  }

  // The LP formulation retains PWL interpolation rows even when NativeLCQP
  // uses the exact quadratic objective. Populate the two adjacent breakpoints
  // so Aeq*x0=beq remains a complete contract.
  for (int k = 0; k < form.ng; ++k) {
    const int count = form.pwl_point_count_by_gen[static_cast<std::size_t>(k)];
    if (count == 0) continue;
    const auto& gen = gens[static_cast<std::size_t>(form.gen_map[static_cast<std::size_t>(k)])];
    const double lo = gen.pmin_mw / form.base_mva;
    const double hi = gen.pmax_mw / form.base_mva;
    const double position = hi > lo
        ? std::clamp((warm.x[form.i_pg(k)] - lo) / (hi - lo), 0.0, 1.0)
        : 0.0;
    const double scaled = position * static_cast<double>(count - 1);
    const int left = std::min(count - 1, static_cast<int>(std::floor(scaled)));
    const int right = std::min(count - 1, left + 1);
    const double right_weight = scaled - static_cast<double>(left);
    warm.x[form.i_pwl(k, left)] = 1.0 - right_weight;
    warm.x[form.i_pwl(k, right)] += right_weight;
  }

  // Fraction-free box projection is deliberate here: LCQP initializes its
  // own positive bound slacks. Phase I supplies a finite bounded primal point,
  // not a second barrier state (Nocedal--Wright, 2006, Section 16.1).
  for (int col = 0; col < form.nvar; ++col) {
    const auto& var = form.lp.vars[static_cast<std::size_t>(col)];
    if (std::isfinite(var.lb)) warm.x[col] = std::max(warm.x[col], var.lb);
    if (std::isfinite(var.ub)) warm.x[col] = std::min(warm.x[col], var.ub);
  }
  if (!warm.x.allFinite()) {
    warm.status = "nonfinite-projection";
    return warm;
  }
  warm.equality_residual = form.lp.Aeq.rows() > 0
      ? (form.lp.Aeq * warm.x - form.lp.beq).lpNorm<Eigen::Infinity>() : 0.0;
  warm.built = std::isfinite(warm.equality_residual);
  warm.status = warm.equality_residual <= 1e-9
      ? "component-balanced-reduced-laplacian"
      : "bounded-near-feasible-projection";
  return warm;
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

bool has_complete_duals(const DCOPFFormulation& form,
                        const engine::SolveResult& sol) {
  const int n_rows = form.lp.A.rows() + form.lp.Aeq.rows();
  return sol.constraint_duals.size() >= n_rows &&
         sol.box_dual_lb.size() >= form.nvar &&
         sol.box_dual_ub.size() >= form.nvar;
}

engine::SolveResult to_solve_result(const engine::api::Result& api_res) {
  engine::SolveResult out;
  out.x = api_res.x;
  out.stats.success = api_res.stats.success;
  out.stats.iterations = api_res.stats.iterations;
  out.stats.objective = api_res.stats.objective;
  out.stats.residual_inf = api_res.stats.residual_inf;
  out.stats.primal_feas = api_res.stats.primal_feas;
  out.stats.dual_feas = api_res.stats.dual_feas;
  out.stats.complementarity = api_res.stats.complementarity;
  out.stats.initial_primal_feas = api_res.stats.initial_primal_feas;
  out.stats.warm_start_used = api_res.stats.warm_start_used;
  out.stats.mip_gap = api_res.stats.mip_gap;
  out.stats.runtime_sec = api_res.stats.runtime_sec;
  out.stats.status = api_res.stats.status;
  out.stats.solver_name = api_res.stats.solver_name;
  out.stats.cglp_cuts_added = api_res.stats.cglp_cuts_added;
  out.stats.farkas_ray = api_res.stats.farkas_ray;
  out.stats.farkas_ray_eq = api_res.stats.farkas_ray_eq;
  out.stats.has_farkas_certificate = api_res.stats.has_farkas_certificate;
  out.constraint_duals = api_res.constraint_duals;
  out.box_dual_lb = api_res.box_dual_lb;
  out.box_dual_ub = api_res.box_dual_ub;
  return out;
}

// NOTE — congested LMP / branch_mu limitation:
// This function recovers dual variables by solving a supporting equality-form
// LP (the original LP formulation with the primal solution fixed as the
// starting basis).  The native simplex solver used here (solve_lp_with_basis)
// does NOT handle bounded-variable LP problems correctly: when
// include_branch_limits=true, the Pf branch-flow variables have active box
// constraints (lower/upper branch limits), and the simplex cannot recover their
// correct dual values (box_dual_lb / box_dual_ub).  As a result:
//   - LMP values (nodal prices) are valid and usable for economic dispatch.
//   - branch_mu_lower / branch_mu_upper are UNRELIABLE when
//     include_branch_limits=true and a branch limit is actually binding.
// Callers must check DCOPFOptions::include_branch_limits before using
// branch_mu for congestion analysis.  See also the test comment in
// test_acopf_dcopf_crossval.cpp and the documentation in 04_opf.tex.
void populate_missing_duals_from_supporting_lp(
    const DCOPFFormulation& form,
    engine::SolveResult& sol,
    bool use_qp,
    const DCOPFOptions& opt) {
  if (has_complete_duals(form, sol) || sol.x.size() < form.nvar) return;

  engine::LPModel support_lp = form.lp;
  if (use_qp) {
    support_lp.c = form.qp.Q * sol.x + form.qp.c;
  }

  engine::SimplexOptions simp_opt;
  simp_opt.max_iter = opt.max_iterations;
  simp_opt.feasibility_tol = opt.feasibility_tol;
  simp_opt.optimality_tol = opt.optimality_tol;

  auto cert = engine::solve_lp_with_basis(support_lp, simp_opt, nullptr);
  if (!cert.result.stats.success) return;

  const int n_rows = form.lp.A.rows() + form.lp.Aeq.rows();
  if (cert.result.constraint_duals.size() >= n_rows) {
    sol.constraint_duals = cert.result.constraint_duals;
  }
  if (cert.result.box_dual_lb.size() >= form.nvar) {
    sol.box_dual_lb = cert.result.box_dual_lb;
  }
  if (cert.result.box_dual_ub.size() >= form.nvar) {
    sol.box_dual_ub = cert.result.box_dual_ub;
  }
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

  // DC-OPF is a linearised AC+DC LP with no converter physics modelled.
  result.converter_model_scope.model_scope = "dc-opf:linear-no-converter-model";
  result.branch_mu_valid = false;
  result.branch_mu_validity_reason =
      "Branch-flow box duals are not KKT-certified by the current native supporting-LP extraction.";
  result.model_limitations.push_back(
      "DC OPF uses a lossless linearized AC network and does not model AC voltage magnitude, reactive power, or converter physics.");
  if (!sys.ac.storage.empty() || !sys.dc.storage.empty()) {
    result.model_limitations.push_back(
        "Storage dispatch is a fixed single-period injection; intertemporal SOC dynamics are outside snapshot DC OPF.");
  }

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
  
  // Extract LMPs and branch congestion prices from the solver certificate.
  // Constraint-dual layout is [inequality rows | equality rows]. The DC OPF
  // model stores Pg/Pf/dPd in p.u., so divide row and bound shadow prices by
  // base_mva to report $/MWh-style marginal values.
  const int ineq_rows = form.lp.A.rows();
  const int eq_rows = form.lp.Aeq.rows();
  if (sol.constraint_duals.size() >= ineq_rows + eq_rows) {
    result.lmp.assign(form.nb, 0.0);
    for (int i = 0; i < form.nb; ++i) {
      const int dual_idx = ineq_rows + i;
      if (dual_idx < sol.constraint_duals.size()) {
        result.lmp[i] = sol.constraint_duals[dual_idx] / base_mva;
      }
    }
    result.lmp_valid = true;
    result.lmp_validity_reason =
        "Nodal prices were recovered from the supporting LP balance-row duals.";
  } else {
    result.lmp_valid = false;
    result.lmp_validity_reason =
        "The selected backend did not provide a complete balance-row dual certificate.";
  }

  if (sol.box_dual_lb.size() >= form.nvar &&
      sol.box_dual_ub.size() >= form.nvar) {
    result.branch_mu_lower.assign(n_br, 0.0);
    result.branch_mu_upper.assign(n_br, 0.0);
    if (include_pf) {
      for (size_t k = 0; k < form.branch_map.size(); ++k) {
        const int bi = form.branch_map[k];
        const int v = form.i_pf(static_cast<int>(k));
        result.branch_mu_lower[bi] = sol.box_dual_lb[v] / base_mva;
        result.branch_mu_upper[bi] = sol.box_dual_ub[v] / base_mva;
      }
    }
    // H3: box duals for bounded Pf variables are unreliable whenever branch
    // limits are active (see comment at the LP formulation block above).
    // Always set branch_mu_valid = false until a proper KKT-based extraction
    // is implemented.
    result.branch_mu_valid = false;
  }
  
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
DCOPFResult solve_dc_opf(const HybridPowerSystem& sys_in,
                         const DCOPFOptions& opt) {
  const auto start_time = std::chrono::steady_clock::now();
  const bool bounded_phase_one = opt.accept_phase_one_iterate &&
      opt.phase_one_time_limit_ms > 0.0 &&
      std::isfinite(opt.phase_one_time_limit_ms);
  const auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_time).count();
  };
  const auto remaining_phase_one_sec = [&]() {
    if (!bounded_phase_one) return 0.0;
    return std::max(0.0,
                    (opt.phase_one_time_limit_ms - elapsed_ms()) / 1000.0);
  };

  DCOPFResult result;
  if (std::any_of(sys_in.lcc_converters.begin(),
                  sys_in.lcc_converters.end(),
                  [](const LCCConverter& lcc) { return lcc.in_service; })) {
    result.status =
        "DC OPF rejected: this AC-only linear formulation does not model LCC AC/DC coupling; use solve_ac_opf with ParityIPM or Ipopt.";
    result.solver_name = "none (unsupported LCC model)";
    result.converter_model_scope.model_scope = "dc-opf:rejected-lcc";
    result.model_limitations.push_back(
        "No optimization was run because DC OPF omits the physical DC network and LCC converter equations.");
    result.runtime_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time).count();
    return result;
  }
  // DC OPF must consume the same canonical topology as AC OPF/PF. In
  // particular, closed rich switches and circuit breakers merge their buses.
  ProjectionOptions projection_options;
  projection_options.strip_dead_islands = false;
  HybridPowerSystem sys =
      projection::RichToCanonicalOperator::apply(sys_in, projection_options)
          .canonical;
  const size_t original_generator_count = sys_in.ac.generators.size();
  const size_t external_source_offset = sys.ac.generators.size();
  std::vector<size_t> promoted_external_grid_indices;
  for (size_t i = 0; i < sys.ac.external_grids.size(); ++i) {
    const auto& grid = sys.ac.external_grids[i];
    if (!grid.in_service) continue;
    Generator source;
    source.index = static_cast<int>(sys.ac.generators.size());
    source.name = grid.name.empty() ? "ExternalGrid" + std::to_string(grid.index)
                                    : grid.name;
    source.bus = grid.bus;
    source.in_service = true;
    source.is_slack = true;
    source.pmin_mw = -1.0e5;
    source.pmax_mw = 1.0e5;
    source.qmin_mvar = -1.0e5;
    source.qmax_mvar = 1.0e5;
    source.cost_c2 = grid.cost_c2;
    source.cost_c1 = grid.cost_c1;
    source.cost_c0 = grid.cost_c0;
    sys.ac.generators.push_back(std::move(source));
    promoted_external_grid_indices.push_back(i);
  }
  auto separate_external_grid_dispatch = [&](DCOPFResult value) {
    value.external_grid_p_mw.assign(sys_in.ac.external_grids.size(), 0.0);
    for (size_t k = 0; k < promoted_external_grid_indices.size(); ++k) {
      const size_t source_row = external_source_offset + k;
      if (source_row < value.pg_mw.size()) {
        value.external_grid_p_mw[promoted_external_grid_indices[k]] =
            value.pg_mw[source_row];
      }
    }
    if (value.pg_mw.size() > original_generator_count)
      value.pg_mw.resize(original_generator_count);
    if (sys.bus_merge_map) {
      const auto& map = *sys.bus_merge_map;
      // A failed DC-OPF legitimately returns empty primal vectors.  Do not feed
      // those through the successful-solve projection contract: doing so masks
      // the original infeasibility with an unrelated size exception.
      if (value.va.size() == static_cast<size_t>(map.n_merged)) {
        value.va = unproject_bus_vector(
            value.va, map, BusVectorSemantics::Intensive);
      }
      if (value.lmp.size() == static_cast<size_t>(map.n_merged)) {
        value.lmp = unproject_bus_vector(
            value.lmp, map, BusVectorSemantics::Intensive);
      }
      if (value.load_shedding_mw.size() ==
          static_cast<size_t>(map.n_merged)) {
        value.load_shedding_mw =
            unproject_bus_vector(value.load_shedding_mw, map,
                                 BusVectorSemantics::Extensive);
      }
      std::vector<double> original_flows(sys_in.ac.branches.size(), 0.0);
      for (size_t i = 0; i < original_flows.size(); ++i) {
        auto it = map.branch_orig_to_proj.find(static_cast<int>(i));
        if (it != map.branch_orig_to_proj.end() && it->second >= 0 &&
            it->second < static_cast<int>(value.pf_mw.size())) {
          original_flows[i] = value.pf_mw[static_cast<size_t>(it->second)];
        }
      }
      value.pf_mw = std::move(original_flows);
    } else if (value.pf_mw.size() > sys_in.ac.branches.size()) {
      value.pf_mw.resize(sys_in.ac.branches.size());
    }
    return value;
  };

  // ── Graph topology pre-check ─────────────────────────────────────────────
  // Detect isolated load islands before the expensive LP build; pre-shed their
  // load and return infeasible immediately when no slack-connected island exists.
  double direct_shed_mw = 0.0;
  std::vector<double> direct_shed_by_bus(sys.ac.buses.size(), 0.0);
  const auto original_bus_pos = build_bus_map(sys.ac.buses);
  const HybridPowerSystem* sys_ptr = &sys;
  std::optional<HybridPowerSystem> sys_pruned;

  auto apply_direct_shed_to_result = [&]() {
    if (direct_shed_mw <= 1e-12) return;
    if (result.load_shedding_mw.size() < direct_shed_by_bus.size()) {
      result.load_shedding_mw.resize(direct_shed_by_bus.size(), 0.0);
    }
    for (size_t i = 0; i < direct_shed_by_bus.size(); ++i) {
      result.load_shedding_mw[i] += direct_shed_by_bus[i];
    }
    result.total_load_shedding_mw += direct_shed_mw;
  };

  if (!sys.ac.buses.empty()) {
    namespace gr = hacdcpf::graph;
    const auto g    = gr::build_power_system_graph(sys);
    const auto topo = gr::analyze_topology(g);
    if (!topo.all_islands_valid) {
      std::unordered_set<int> dispatchable_source_buses;
      for (const auto& gen : sys.ac.generators) {
        if (gen.in_service) dispatchable_source_buses.insert(gen.bus);
      }
      const auto island_has_dispatchable_source = [&](const gr::IslandInfo& island) {
        return std::any_of(
            island.ac_bus_ids.begin(), island.ac_bus_ids.end(),
            [&](int bus) { return dispatchable_source_buses.count(bus) != 0; });
      };
      std::unordered_set<int> dead_buses;
      for (const auto& isl : topo.islands) {
        if (isl.status == gr::IslandStatus::IsolatedLoad ||
            (isl.status == gr::IslandStatus::NoSlack &&
             !island_has_dispatchable_source(isl))) {
          dead_buses.insert(isl.ac_bus_ids.begin(), isl.ac_bus_ids.end());
        }
      }
      // P1a: DC-OPF formulation adds bus.pd_mw and ac.loads additively as
      // demand; direct-shed accounting must mirror the same convention to
      // avoid under-counting curtailment when both sources are present.
      for (const auto& b : sys.ac.buses) {
        if (!dead_buses.count(b.index)) continue;
        const double shed = std::max(0.0, b.pd_mw);
        auto it = original_bus_pos.find(b.index);
        if (it != original_bus_pos.end()) direct_shed_by_bus[it->second] += shed;
        direct_shed_mw += shed;
      }
      for (const auto& ld : sys.ac.loads) {
        if (!ld.in_service || !dead_buses.count(ld.bus)) continue;
        const double shed = std::max(0.0, ld.p_mw * ld.scaling);
        auto it = original_bus_pos.find(ld.bus);
        if (it != original_bus_pos.end()) direct_shed_by_bus[it->second] += shed;
        direct_shed_mw += shed;
      }
      const bool has_valid = std::any_of(
          topo.islands.begin(), topo.islands.end(),
          [&](const gr::IslandInfo& i) {
            return i.status == gr::IslandStatus::Valid ||
                   island_has_dispatchable_source(i);
          });
      if (!has_valid) {
        if (opt.verbose)
          spdlog::warn("DC OPF: no valid island — {:.2f} MW direct shed, infeasible",
                       direct_shed_mw);
        result.status = "Infeasible: no island with slack bus";
        result.converged = false;
        apply_direct_shed_to_result();
        return separate_external_grid_dispatch(std::move(result));
      }
      // Prune dead-bus loads from a local copy before building the formulation.
      sys_pruned = sys;
      for (auto& b : sys_pruned->ac.buses) {
        if (!dead_buses.count(b.index)) continue;
        b.pd_mw = 0.0;
        b.qd_mvar = 0.0;
      }
      for (auto& ld : sys_pruned->ac.loads) {
        if (!dead_buses.count(ld.bus)) continue;
        ld.in_service = false;
      }
      for (auto& gen : sys_pruned->ac.generators) {
        if (dead_buses.count(gen.bus)) gen.in_service = false;
      }
      for (auto& sg : sys_pruned->ac.static_generators) {
        if (dead_buses.count(sg.bus)) sg.in_service = false;
      }
      for (auto& rg : sys_pruned->ac.renewable_gens) {
        if (dead_buses.count(rg.bus)) rg.in_service = false;
      }
      for (auto& pv : sys_pruned->ac.pv_systems) {
        if (dead_buses.count(pv.bus)) pv.in_service = false;
      }
      for (auto& st : sys_pruned->ac.storage) {
        if (dead_buses.count(st.bus)) st.in_service = false;
      }
      for (auto& br : sys_pruned->ac.branches) {
        if (dead_buses.count(br.from_bus) || dead_buses.count(br.to_bus)) {
          br.in_service = false;
        }
      }
      sys_ptr = &(*sys_pruned);
      if (opt.verbose)
        spdlog::warn("DC OPF: {:.2f} MW in isolated islands pre-shed before LP solve",
                     direct_shed_mw);
    }
  }

  // Build LP formulation (constraint structure)
  DCOPFFormulation form;
  try {
    form = build_dc_opf_lp(*sys_ptr, opt);
  } catch (const std::invalid_argument& e) {
    spdlog::error("DC OPF: invalid topology — {}", e.what());
    result.status = std::string("Invalid topology: ") + e.what();
    result.converged = false;
    apply_direct_shed_to_result();
    return separate_external_grid_dispatch(std::move(result));
  }
  
  if (form.nb == 0 || form.ng == 0) {
    result.status = "Empty system or no generators";
    result.converged = false;
    apply_direct_shed_to_result();
    return separate_external_grid_dispatch(std::move(result));
  }
  
  // Build QP model with true quadratic costs
  build_dc_opf_qp(form, *sys_ptr);
  result.structural_warm_start_requested = opt.structural_warm_start;
  result.structural_warm_start_components =
      static_cast<int>(form.component_buses.size());
  DCStructuralWarmStart structural_start;
  if (opt.structural_warm_start) {
    structural_start = build_dc_structural_warm_start(form, *sys_ptr);
    result.structural_warm_start_built = structural_start.built;
    result.structural_warm_start_factorizations = structural_start.factorizations;
    result.structural_warm_start_residual = structural_start.equality_residual;
    result.structural_warm_start_status = structural_start.status;
    if (structural_start.built) form.qp.x0 = structural_start.x;
  }
  
  if (opt.verbose) {
    spdlog::info("DC OPF: {} buses, {} generators, {} branches, {} variables",
                 form.nb, form.ng, static_cast<int>(form.branch_map.size()),
                 form.nvar);
  }
  
  // Select solver backend
  engine::SolveResult sol;
  bool use_qp = false;  // Track if we used QP solver (for objective computation)
  
  // Track the actual backend chain for post-solve auditing.
  std::vector<std::string>& solver_chain = result.solver_chain;
  auto record_chain = [&](const char* backend, bool ok, const std::string& detail = "") {
    std::string entry = std::string(backend) + ":" + (ok ? "ok" : "fail");
    if (!detail.empty()) entry += "(" + detail + ")";
    solver_chain.push_back(std::move(entry));
  };

  // NativeQP: Use LCQP solver with true quadratic costs.
  // IMPORTANT: use_qp must reflect whether the *converged* solution comes
  // from a QP solver. If LCQP fails we must NOT leave use_qp=true, because a
  // subsequent LP fallback would otherwise be reported with QP objective
  // semantics.
  auto try_native_qp = [&]() -> bool {
    if (bounded_phase_one && remaining_phase_one_sec() <= 0.0) {
      sol.stats.solver_name = "NativeLCQP";
      sol.stats.status = "TimeLimit (before numeric solve)";
      result.phase_one_budget_exhausted = true;
      record_chain("NativeLCQP", false, sol.stats.status);
      return true;
    }
    engine::LCQPOptions qp_opt;
    qp_opt.max_iter = opt.max_iterations;
    qp_opt.tol_primal = opt.feasibility_tol;
    qp_opt.tol_dual = opt.optimality_tol;
    qp_opt.tol_gap = opt.optimality_tol;
    qp_opt.time_limit_sec = remaining_phase_one_sec();
    qp_opt.verbose = opt.verbose;
    
    engine::NativeLCQPAdapter lcqp(qp_opt);
    sol = lcqp.solve_qp(form.qp);
    sol.stats.solver_name = "NativeLCQP";
    result.native_qp_symbolic_analyze_calls =
        sol.stats.symbolic_analyze_calls;
    const bool native_time_limit = sol.stats.status == "TimeLimit";
    result.phase_one_budget_exhausted = native_time_limit ||
        (bounded_phase_one && elapsed_ms() >= opt.phase_one_time_limit_ms);
    if (!sol.stats.success && opt.accept_phase_one_iterate &&
        sol.x.size() == form.nvar && sol.x.allFinite()) {
      double residual = form.qp.Aeq.rows() > 0
          ? (form.qp.Aeq * sol.x - form.qp.beq)
                .lpNorm<Eigen::Infinity>()
          : 0.0;
      for (int col = 0; col < form.nvar; ++col) {
        const auto& variable = form.qp.vars[static_cast<std::size_t>(col)];
        if (std::isfinite(variable.lb))
          residual = std::max(residual, variable.lb - sol.x[col]);
        if (std::isfinite(variable.ub))
          residual = std::max(residual, sol.x[col] - variable.ub);
      }
      result.phase_one_iterate_residual = residual;
      const double tolerance =
          std::max(0.0, opt.phase_one_iterate_tolerance);
      if (std::isfinite(residual) && residual <= tolerance) {
        result.phase_one_warm_start_only = true;
        sol.stats.success = true;
        sol.stats.status = native_time_limit
            ? "Phase-I usable time-limited iterate (not DCOPF optimal)"
            : "Phase-I usable iterate (not DCOPF optimal)";
      }
    }
    use_qp = sol.stats.success;
    result.structural_warm_start_used = sol.stats.warm_start_used;
    result.solver_initial_primal_residual = sol.stats.initial_primal_feas;
    record_chain("NativeLCQP", sol.stats.success, sol.stats.status);
    return true;
  };
  
  // Gurobi QP: Use Gurobi with QP model
  auto try_gurobi_qp = [&]() -> bool {
    engine::GurobiAdapter gurobi;
    if (gurobi.available()) {
      sol = gurobi.solve_qp(form.qp);
      use_qp = sol.stats.success;
      record_chain("Gurobi-QP", sol.stats.success, sol.stats.status);
      return true;
    }
    record_chain("Gurobi-QP", false, "unavailable");
    return false;
  };
  
  // HiGHS is the preferred solver. HiGHS does not expose a native QP interface,
  // so SolverEngine routes solve_qp to whichever QP-capable backend is available
  // (Gurobi, NativeLCQP, etc.). The returned sol.stats.solver_name reflects the
  // actual backend used, not necessarily HiGHS. The "preferred_solver = HiGHS"
  // hint still causes HiGHS to handle LP/MILP sub-problems within that backend
  // where applicable.
  auto try_highs = [&]() -> bool {
    if (!opt.phase_one_linear_relaxation) {
      // Single QP attempt through SolverEngine with HiGHS preference.
      // A previous version had two back-to-back calls with identical options;
      // the first success path skipped solver_chain recording and the duplicate
      // call was redundant.  Now: one call, record result, fall through to LP.
      engine::SolverEngine eng;
      engine::SolveOptions solve_opt;
      solve_opt.preferred_solver = "HiGHS";
      solve_opt.allow_fallback = true;
      auto qp_res = to_solve_result(eng.solve_qp(form.qp, solve_opt));
      if (qp_res.stats.success) {
        sol = qp_res;
        use_qp = true;
        record_chain("HiGHS-routed-QP", true, sol.stats.status);
        return true;
      }
      record_chain("HiGHS-routed-QP", false, qp_res.stats.status);
    }

    // Stott, Jardim & Alsac, IEEE TPS 2009, "DC Power Flow Revisited": the
    // linear network model preserves component balance and branch congestion.
    // For Phase I its PWL cost is sufficient; Phase II certifies the nonlinear
    // AC objective/KKT point, so paying for an exact QP here is unnecessary.
    engine::HighsAdapter highs;
    if (highs.available()) {
      sol = highs.solve_lp(form.lp);
      // LP fallback: objective and duals reflect linear cost model only.
      use_qp = false;
      sol.stats.solver_name = "HiGHS-LP(QP-fallback)";
      record_chain("HiGHS-LP", sol.stats.success, sol.stats.status);
      return true;
    }
    record_chain("HiGHS-LP", false, "unavailable");
    return false;
  };
  
  // Native simplex: LP with linearized costs. Must reset use_qp=false because
  // this path is taken as a fallback from NativeQP when the QP fails to
  // converge; without the reset, downstream code would recompute the
  // objective using the QP form against an LP-feasible (but not QP-optimal)
  // point, producing inconsistent objective/dual semantics.
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
    sol.constraint_duals = simp_result.result.constraint_duals;
    sol.box_dual_lb = simp_result.result.box_dual_lb;
    sol.box_dual_ub = simp_result.result.box_dual_ub;
    use_qp = false;
    record_chain("NativeDualSimplex", sol.stats.success, sol.stats.status);
    return true;
  };
  
  bool solved = false;
  switch (opt.solver) {
    case DCOPFSolverBackend::NativeQP:
      solved = try_native_qp();
      // Warm-start-only Phase I has a strict wall budget; an unconstrained
      // simplex fallback would violate it and cannot improve the AC certificate.
      if (!sol.stats.success && !opt.accept_phase_one_iterate) {
        solved = try_native_simplex();
      }
      break;
      
    case DCOPFSolverBackend::Gurobi:
      solved = try_gurobi_qp();  // Prefer QP with Gurobi
      if (!solved || !sol.stats.success) {
        solved = try_native_qp();
      }
      if (!solved || !sol.stats.success) {
        solved = try_highs();
      }
      if (!solved || !sol.stats.success) {
        solved = try_native_simplex();
      }
      break;
      
    case DCOPFSolverBackend::HiGHS:
      solved = try_highs();
      if (!solved) {
        result.status = "HiGHS not available";
        result.converged = false;
        apply_direct_shed_to_result();
        return separate_external_grid_dispatch(std::move(result));
      }
      break;
      
    case DCOPFSolverBackend::Native:
      solved = try_native_simplex();
      break;
      
    case DCOPFSolverBackend::Auto:
    default:
      // Prefer an installed/licensed Gurobi, then use packaged QP/LP solvers.
      solved = try_gurobi_qp();
      if (!solved || !sol.stats.success) {
        solved = try_native_qp();
      }
      if (!solved || !sol.stats.success) {
        solved = try_highs();
      }
      if (!solved || !sol.stats.success) {
        solved = try_native_simplex();
      }
      break;
  }
  
  const auto end_time = std::chrono::steady_clock::now();
  double runtime_sec = std::chrono::duration<double>(end_time - start_time).count();
  if (bounded_phase_one) {
    result.phase_one_budget_exhausted =
        result.phase_one_budget_exhausted ||
        runtime_sec * 1000.0 >= opt.phase_one_time_limit_ms;
    result.phase_one_budget_overshoot_ms = std::max(
        0.0, runtime_sec * 1000.0 - opt.phase_one_time_limit_ms);
  }
  result.structural_warm_start_used =
      use_qp && sol.stats.success && sol.stats.warm_start_used;
  result.solver_initial_primal_residual =
      use_qp && sol.stats.success
          ? sol.stats.initial_primal_feas
          : std::numeric_limits<double>::infinity();

  // Supporting LP for dual extraction (LMPs) — skip when not requested.
  if (opt.compute_lmp) {
    populate_missing_duals_from_supporting_lp(form, sol, use_qp, opt);
  }
  
  // Extract results. extract_dc_opf_result overwrites *result*, so preserve
  // the solver_chain we accumulated during the fallback sequence and restore
  // it afterwards.
  std::vector<std::string> chain_snapshot = std::move(solver_chain);
  const bool structural_requested = result.structural_warm_start_requested;
  const bool structural_built = result.structural_warm_start_built;
  const bool structural_used = result.structural_warm_start_used;
  const int structural_components = result.structural_warm_start_components;
  const int structural_factorizations = result.structural_warm_start_factorizations;
  const double structural_residual = result.structural_warm_start_residual;
  const double solver_initial_residual = result.solver_initial_primal_residual;
  const bool phase_one_warm_start_only = result.phase_one_warm_start_only;
  const bool phase_one_budget_exhausted =
      result.phase_one_budget_exhausted;
  const double phase_one_budget_overshoot_ms =
      result.phase_one_budget_overshoot_ms;
  const int native_qp_symbolic_analyze_calls =
      result.native_qp_symbolic_analyze_calls;
  const double phase_one_iterate_residual = result.phase_one_iterate_residual;
  std::string structural_status = std::move(result.structural_warm_start_status);
  result = extract_dc_opf_result(form, sol, *sys_ptr, runtime_sec);
  result.solver_chain = std::move(chain_snapshot);
  result.structural_warm_start_requested = structural_requested;
  result.structural_warm_start_built = structural_built;
  result.structural_warm_start_used = structural_used;
  result.structural_warm_start_components = structural_components;
  result.structural_warm_start_factorizations = structural_factorizations;
  result.structural_warm_start_residual = structural_residual;
  result.solver_initial_primal_residual = solver_initial_residual;
  result.structural_warm_start_status = std::move(structural_status);
  result.phase_one_warm_start_only = phase_one_warm_start_only;
  result.phase_one_budget_exhausted = phase_one_budget_exhausted;
  result.phase_one_budget_overshoot_ms = phase_one_budget_overshoot_ms;
  result.native_qp_symbolic_analyze_calls =
      native_qp_symbolic_analyze_calls;
  result.phase_one_iterate_residual = phase_one_iterate_residual;
  if (phase_one_warm_start_only) {
    result.converged = false;
    result.model_limitations.push_back(
        "The NativeQP iteration-limited primal is certified only for Phase-I warm-start use; it is not a DCOPF optimum.");
  }
  if (!opt.compute_lmp) {
    result.lmp.clear();
    result.lmp_valid = false;
    result.lmp_validity_reason =
        "Nodal-price extraction was disabled by DCOPFOptions::compute_lmp.";
  }

  // For QP solvers, recompute the true quadratic objective including constant
  // terms. `objective_model` records which cost model the reported objective
  // corresponds to, disambiguating QP-vs-LP semantics across fallback paths.
  if (use_qp && sol.stats.success && sol.x.size() >= form.nvar) {
    result.objective = compute_qp_objective(form, sol.x, *sys_ptr);
    result.objective_model = "QP";
    result.pwl_segments_effective = 0;
  } else {
    result.objective_model = form.pwl_segments_effective > 0 ? "LP-PWL" : "LP";
    result.pwl_segments_effective = form.pwl_segments_effective;
  }
  
  if (opt.verbose) {
    spdlog::info("DC OPF: converged={} obj={:.4f} solver={} time={:.3f}s",
                 result.converged, result.objective, result.solver_name,
                 result.runtime_sec);
  }
  
  apply_direct_shed_to_result();
  return separate_external_grid_dispatch(std::move(result));
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

  // Add back load shedding — the OPF formulation includes a dpd[i] variable
  // that reduces the effective demand at each bus.  Omitting this step makes
  // every OPF solution that exercised load shedding appear infeasible.
  if (!result.load_shedding_mw.empty()) {
    for (size_t i = 0; i < buses.size() && i < result.load_shedding_mw.size(); ++i) {
      p_net[i] += result.load_shedding_mw[i];
    }
  }

  // Add generation
  for (size_t gi = 0; gi < gens.size(); ++gi) {
    if (!gens[gi].in_service) continue;
    auto it = bus_map.find(gens[gi].bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] += result.pg_mw[gi];
  }
  for (size_t i = 0; i < sys.ac.external_grids.size() &&
                     i < result.external_grid_p_mw.size(); ++i) {
    const auto& grid = sys.ac.external_grids[i];
    if (!grid.in_service) continue;
    auto it = bus_map.find(grid.bus);
    if (it == bus_map.end()) continue;
    p_net[it->second] += result.external_grid_p_mw[i];
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
  
  // Check balance at each bus, including slack.  The LP formulation includes
  // the slack-bus balance row; skipping it here would hide global imbalance.
  for (size_t i = 0; i < buses.size(); ++i) {
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
