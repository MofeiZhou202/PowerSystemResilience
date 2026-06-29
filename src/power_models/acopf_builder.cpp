#include "hacdcpf/power_models/ac_pf_model_builder.hpp"
#include "hacdcpf/power_models/branch_admittance.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hacdcpf/aml/aml.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"

namespace hacdcpf::power_models {

using namespace hacdcpf::aml;

// ── Internal constants ────────────────────────────────────────────────────
namespace {
constexpr double kPi           = 3.14159265358979323846;
constexpr double kDegToRad     = kPi / 180.0;
constexpr double kHuge         = 1e4;   // used as bound when unset
constexpr double kMinImpedance = 1e-6;  // skip zero-impedance branches in data conversion

}  // namespace

// ════════════════════════════════════════════════════════════════════════════
// to_acopf_data — HybridPowerSystem → ACOPFData
// ════════════════════════════════════════════════════════════════════════════
ACOPFData to_acopf_data(const HybridPowerSystem& sys) {
  ACOPFData d;
  d.base_mva = sys.base_mva > 0.0 ? sys.base_mva : 100.0;
  const double Sb = d.base_mva;

  // Build bus-index → bus-id map (1-based bus.index)
  std::unordered_map<int, std::string> idx_to_id;
  idx_to_id.reserve(sys.ac.buses.size());

  for (const auto& b : sys.ac.buses) {
    if (!b.in_service) continue;
    const std::string id = b.name.empty() ? ("B" + std::to_string(b.index)) : b.name;
    idx_to_id[b.index] = id;

    ACOPFBusData bd;
    bd.id       = id;
    bd.pd_pu    = b.pd_mw  / Sb;
    bd.qd_pu    = b.qd_mvar / Sb;
    bd.gs_pu    = b.gs_mw  / Sb;
    bd.bs_pu    = b.bs_mvar / Sb;
    bd.vm_min   = (b.vmin_pu > 0.0)  ? b.vmin_pu : 0.9;
    bd.vm_max   = (b.vmax_pu > 0.0)  ? b.vmax_pu : 1.1;
    bd.vm0      = (b.vm_pu  > 0.0)   ? b.vm_pu   : 1.0;
    bd.va0_rad  = b.va_deg * kDegToRad;
    bd.is_ref   = (b.bus_type == BusType::SLACK);
    d.buses.push_back(std::move(bd));
  }

  // Generators (in-service only)
  for (std::size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
    const auto& g = sys.ac.generators[gi];
    if (!g.in_service) continue;
    auto it = idx_to_id.find(g.bus);
    if (it == idx_to_id.end()) continue;

    const std::string gid = g.name.empty()
        ? ("G" + std::to_string(g.index))
        : g.name;

    ACOPFGenData gd;
    gd.id        = gid;
    gd.bus_id    = it->second;
    gd.pg_min_pu = g.pmin_mw  / Sb;
    gd.pg_max_pu = std::max(g.pmax_mw / Sb, gd.pg_min_pu);
    gd.qg_min_pu = (std::isfinite(g.qmin_mvar) && g.qmin_mvar > -1e9)
                       ? g.qmin_mvar / Sb : -kHuge;
    gd.qg_max_pu = (std::isfinite(g.qmax_mvar) && g.qmax_mvar < 1e9)
                       ? g.qmax_mvar / Sb :  kHuge;
    gd.cost_c2   = g.cost_c2;
    gd.cost_c1   = g.cost_c1;
    gd.cost_c0   = g.cost_c0;
    gd.pg0_pu    = std::clamp(g.pg_mw  / Sb, gd.pg_min_pu, gd.pg_max_pu);
    gd.qg0_pu    = std::clamp(g.qg_mvar / Sb, gd.qg_min_pu, gd.qg_max_pu);

    // If generator is the slack, it is also a ref-bus generator
    if (g.is_slack) {
      // Mark the bus as reference if not already
      for (auto& bd : d.buses) {
        if (bd.id == it->second) { bd.is_ref = true; break; }
      }
    }
    d.generators.push_back(std::move(gd));
  }

  // External grids with cost participate in OPF as dispatchable slack sources.
  // They already anchor V/θ via is_ref; here we add their cost-curve entries so
  // the IPM Hessian is non-singular even when there are no conventional generators.
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    if (eg.cost_c2 == 0.0 && eg.cost_c1 == 0.0) continue;
    auto it = idx_to_id.find(eg.bus);
    if (it == idx_to_id.end()) continue;

    const double pmax = (eg.s_sc_max_mva > 0.0) ? eg.s_sc_max_mva : 1000.0;
    ACOPFGenData gd;
    gd.id        = eg.name.empty() ? ("EG" + std::to_string(eg.index)) : eg.name;
    gd.bus_id    = it->second;
    gd.pg_min_pu = -pmax / Sb;
    gd.pg_max_pu =  pmax / Sb;
    gd.qg_min_pu = -pmax / Sb;
    gd.qg_max_pu =  pmax / Sb;
    gd.cost_c2   = eg.cost_c2;
    gd.cost_c1   = eg.cost_c1;
    gd.cost_c0   = eg.cost_c0;
    gd.pg0_pu    = 0.0;
    gd.qg0_pu    = 0.0;
    d.generators.push_back(std::move(gd));
  }

  // Branches (in-service, finite impedance)
  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    auto fi = idx_to_id.find(br.from_bus);
    auto ti = idx_to_id.find(br.to_bus);
    if (fi == idx_to_id.end() || ti == idx_to_id.end()) continue;

    // Skip zero-impedance (switch / tie) branches
    const double z2 = br.r_pu * br.r_pu + br.x_pu * br.x_pu;
    if (z2 < kMinImpedance * kMinImpedance) continue;

    const std::string brid = br.name.empty()
        ? (fi->second + "->" + ti->second + "_" + std::to_string(br.index))
        : br.name;

    ACOPFBranchData brd;
    brd.id        = brid;
    brd.from_bus  = fi->second;
    brd.to_bus    = ti->second;
    brd.r_pu      = br.r_pu;
    brd.x_pu      = br.x_pu;
    brd.bc_pu     = br.b_pu;
    brd.tap       = (br.tap > 0.0) ? br.tap : 1.0;
    brd.shift_deg = br.shift_deg;
    brd.rate_a_pu = (br.rate_a_mva > 0.0) ? (br.rate_a_mva / Sb) : 0.0;
    d.branches.push_back(std::move(brd));
  }

  // Ensure at least one reference bus exists (use bus 0 if none labelled SLACK)
  bool has_ref = false;
  for (const auto& bd : d.buses) {
    if (bd.is_ref) { has_ref = true; break; }
  }
  if (!has_ref && !d.buses.empty()) {
    d.buses[0].is_ref = true;
  }

  return d;
}

// ════════════════════════════════════════════════════════════════════════════
// solve_acopf — AML NonlinearExpr AC OPF
// ════════════════════════════════════════════════════════════════════════════
ACOPFBuilderResult solve_acopf(const ACOPFData& data,
                                const SolveOptions& opts_in) {
  if (data.buses.empty()) {
    throw std::invalid_argument("ACOPFData: bus list is empty");
  }

  // ── Choose solver ──────────────────────────────────────────────────────
  // Default: NativeIPM. NativeNLP is unsafe for AC OPF (can corrupt heap on
  // failure), so fallback is disabled. The caller may override solver_name.
  SolveOptions opts = opts_in;
  if (opts.solver_name.empty()) {
    opts.solver_name = "NativeIPM";
  }
  // Never fall back to NativeNLP — it lacks a line-search and can corrupt
  // memory on ill-conditioned AC OPF problems.
  opts.allow_fallback = false;

  const double Sb = (data.base_mva > 0.0) ? data.base_mva : 100.0;

  // ── Build bus / gen index maps ─────────────────────────────────────────
  // bus_id → position in data.buses
  std::unordered_map<std::string, std::size_t> bus_pos;
  bus_pos.reserve(data.buses.size());
  for (std::size_t i = 0; i < data.buses.size(); ++i) {
    bus_pos[data.buses[i].id] = i;
  }

  // ── AML model ─────────────────────────────────────────────────────────
  Model m("acopf");

  // ── Sets ──────────────────────────────────────────────────────────────
  auto& bus_set = m.add_set("buses");
  for (const auto& bd : data.buses) {
    bus_set.add_element(Key::scalar(bd.id));
  }
  auto& gen_set = m.add_set("gens");
  for (const auto& gd : data.generators) {
    gen_set.add_element(Key::scalar(gd.id));
  }

  // ── Variables ─────────────────────────────────────────────────────────
  // Voltage magnitude: Vm ∈ [vm_min, vm_max]
  auto& Vm = m.add_var("Vm", bus_set, VarType::Continuous, 0.9, 1.1);
  // Voltage angle: Va ∈ [−π, π]
  auto& Va = m.add_var("Va", bus_set, VarType::Continuous, -kPi, kPi);
  // Active generation (pu)
  auto& Pg = m.add_var("Pg", gen_set, VarType::Continuous, 0.0, kHuge);
  // Reactive generation (pu)
  auto& Qg = m.add_var("Qg", gen_set, VarType::Continuous, -kHuge, kHuge);

  // Per-bus bounds; reference bus angle pinned via equality constraint (not .fix)
  int slack_idx = -1;  // index into data.buses for the reference bus
  for (std::size_t i = 0; i < data.buses.size(); ++i) {
    const auto& bd = data.buses[i];
    Key bk = Key::scalar(bd.id);
    Vm.set_lb(bk, bd.vm_min);
    Vm.set_ub(bk, bd.vm_max);
    if (bd.is_ref) {
      slack_idx = static_cast<int>(i);
    }
  }

  // Per-gen bounds
  for (const auto& gd : data.generators) {
    Key gk = Key::scalar(gd.id);
    Pg.set_lb(gk, gd.pg_min_pu);
    Pg.set_ub(gk, gd.pg_max_pu);
    Qg.set_lb(gk, gd.qg_min_pu);
    Qg.set_ub(gk, gd.qg_max_pu);
  }

  // ── Objective: min Σ_g [c2·(Pg·Sb)² + c1·(Pg·Sb) + c0] ───────────────
  // Build as NonlinearExpr; pure linear / zero-c2 gens handled uniformly.
  NonlinearExpr obj = m.nl_const(0.0);
  for (const auto& gd : data.generators) {
    Key gk = Key::scalar(gd.id);
    VarRef pg_ref = Pg(gk);
    NonlinearExpr nlpg = m.nl_var(pg_ref);

    // f_g = c2·(Sb·Pg)² + c1·(Sb·Pg) + c0
    //     = c2·Sb²·Pg² + c1·Sb·Pg + c0
    const double a2 = gd.cost_c2 * Sb * Sb;
    const double a1 = gd.cost_c1 * Sb;
    const double a0 = gd.cost_c0;

    NonlinearExpr fg = m.nl_const(a0);
    if (std::abs(a1) > 0.0) {
      fg = m.nl_add(fg, m.nl_mul(m.nl_const(a1), nlpg));
    }
    if (std::abs(a2) > 0.0) {
      fg = m.nl_add(fg, m.nl_mul(m.nl_const(a2), m.nl_sq(nlpg)));
    }
    obj = m.nl_add(obj, fg);
  }
  m.minimize(obj);

  // ── Warm start ────────────────────────────────────────────────────────
  {
    std::vector<double> x0;
    x0.reserve(2 * data.buses.size() + 2 * data.generators.size());
    // Order: all Vm, all Va, all Pg, all Qg  (matches AML variable registration order)
    for (const auto& bd : data.buses) x0.push_back(bd.vm0);
    for (const auto& bd : data.buses) x0.push_back(bd.va0_rad);
    for (const auto& gd : data.generators) x0.push_back(gd.pg0_pu);
    for (const auto& gd : data.generators) x0.push_back(gd.qg0_pu);
    m.set_nlp_x0(x0);
  }

  // ── Precompute branch admittances ─────────────────────────────────────
  struct BranchAdm {
    std::size_t f_idx, t_idx;
    BranchAdmittance Y;
    double rate_a_pu;
    std::string id;
  };
  std::vector<BranchAdm> bra;
  bra.reserve(data.branches.size());
  for (const auto& br : data.branches) {
    auto fi = bus_pos.find(br.from_bus);
    auto ti = bus_pos.find(br.to_bus);
    if (fi == bus_pos.end() || ti == bus_pos.end()) continue;
    BranchAdm ba;
    ba.f_idx    = fi->second;
    ba.t_idx    = ti->second;
    ba.Y        = branch_admittance(br.r_pu, br.x_pu, br.bc_pu,
                                     br.tap, br.shift_deg * kDegToRad);
    ba.rate_a_pu = br.rate_a_pu;
    ba.id        = br.id;
    bra.push_back(std::move(ba));
  }

  // ── Per-bus NonlinearExpr: build P and Q balance expressions ──────────
  // P_bal[i] = sum_gen_at_i(Pg) - Pd_i - Gs_i*Vm_i² - sum_branches(Pf or Pt)
  // Q_bal[i] = sum_gen_at_i(Qg) - Qd_i + Bs_i*Vm_i² - sum_branches(Qf or Qt)

  const std::size_t nb = data.buses.size();

  // Helper: nl_Vm[i], nl_Va[i]
  std::vector<NonlinearExpr> nl_Vm;
  std::vector<NonlinearExpr> nl_Va;
  nl_Vm.reserve(nb);
  nl_Va.reserve(nb);
  for (const auto& bd : data.buses) {
    nl_Vm.push_back(m.nl_var(Vm(Key::scalar(bd.id))));
    nl_Va.push_back(m.nl_var(Va(Key::scalar(bd.id))));
  }

  // Initialise P and Q accumulator per bus
  std::vector<NonlinearExpr> P_acc, Q_acc;
  P_acc.reserve(nb);
  Q_acc.reserve(nb);
  for (std::size_t i = 0; i < nb; ++i) {
    const auto& bd = data.buses[i];
    // Start: generation shunt contributions − load − shunt injection
    // P_acc[i] = -Pd_i - Gs_i*Vm_i²
    NonlinearExpr p = m.nl_mul(m.nl_const(-bd.gs_pu), m.nl_sq(nl_Vm[i]));
    if (std::abs(bd.pd_pu) > 0.0) {
      p = m.nl_sub(p, m.nl_const(bd.pd_pu));
    }
    // Q_acc[i] = -Qd_i + Bs_i*Vm_i²
    NonlinearExpr q = m.nl_mul(m.nl_const(bd.bs_pu), m.nl_sq(nl_Vm[i]));
    if (std::abs(bd.qd_pu) > 0.0) {
      q = m.nl_sub(q, m.nl_const(bd.qd_pu));
    }
    P_acc.push_back(p);
    Q_acc.push_back(q);
  }

  // Add generator injections
  for (const auto& gd : data.generators) {
    auto bit = bus_pos.find(gd.bus_id);
    if (bit == bus_pos.end()) continue;
    std::size_t i = bit->second;
    Key gk = Key::scalar(gd.id);
    P_acc[i] = m.nl_add(P_acc[i], m.nl_var(Pg(gk)));
    Q_acc[i] = m.nl_add(Q_acc[i], m.nl_var(Qg(gk)));
  }

  // Subtract branch power flows from each bus's accumulator
  // For branch (f,t):
  //   Pf = Gff·Vf² + Vf·Vt·(Gft·cos(θft) + Bft·sin(θft))
  //   Qf = −Bff·Vf² + Vf·Vt·(Gft·sin(θft) − Bft·cos(θft))
  //   Pt = Gtt·Vt² + Vf·Vt·(Gtf·cos(θtf) + Btf·sin(θtf))
  //   Qt = −Btt·Vt² + Vf·Vt·(Gtf·sin(θtf) − Btf·cos(θtf))
  // P_acc[f] -= Pf,  P_acc[t] -= Pt
  // Q_acc[f] -= Qf,  Q_acc[t] -= Qt

  // Save flows for thermal constraints (avoids recomputing duplicate expressions)
  struct BranchFlow { NonlinearExpr Pf, Qf, Pt, Qt; };
  std::vector<BranchFlow> branch_flows;
  branch_flows.reserve(bra.size());

  for (const auto& ba : bra) {
    const std::size_t f = ba.f_idx;
    const std::size_t t = ba.t_idx;
    const auto& Y = ba.Y;

    NonlinearExpr Vf = nl_Vm[f];
    NonlinearExpr Vt = nl_Vm[t];
    NonlinearExpr θf = nl_Va[f];
    NonlinearExpr θt = nl_Va[t];

    NonlinearExpr θft = m.nl_sub(θf, θt);          // θf − θt
    NonlinearExpr θtf = m.nl_sub(θt, θf);          // θt − θf
    NonlinearExpr VfVt = m.nl_mul(Vf, Vt);         // Vf · Vt
    NonlinearExpr Vf2 = m.nl_sq(Vf);               // Vf²
    NonlinearExpr Vt2 = m.nl_sq(Vt);               // Vt²

    // From-side power (Pf, Qf)
    // Gff·Vf² + VfVt·(Gft·cos(θft)+Bft·sin(θft))
    NonlinearExpr Pf = m.nl_add(
        m.nl_mul(m.nl_const(Y.Gff), Vf2),
        m.nl_mul(VfVt,
            m.nl_add(
                m.nl_mul(m.nl_const(Y.Gft), m.nl_cos(θft)),
                m.nl_mul(m.nl_const(Y.Bft), m.nl_sin(θft)))));
    // −Bff·Vf² + VfVt·(Gft·sin(θft)−Bft·cos(θft))
    NonlinearExpr Qf = m.nl_add(
        m.nl_mul(m.nl_const(-Y.Bff), Vf2),
        m.nl_mul(VfVt,
            m.nl_sub(
                m.nl_mul(m.nl_const(Y.Gft), m.nl_sin(θft)),
                m.nl_mul(m.nl_const(Y.Bft), m.nl_cos(θft)))));

    // To-side power (Pt, Qt)
    NonlinearExpr Pt = m.nl_add(
        m.nl_mul(m.nl_const(Y.Gtt), Vt2),
        m.nl_mul(VfVt,
            m.nl_add(
                m.nl_mul(m.nl_const(Y.Gtf), m.nl_cos(θtf)),
                m.nl_mul(m.nl_const(Y.Btf), m.nl_sin(θtf)))));
    NonlinearExpr Qt = m.nl_add(
        m.nl_mul(m.nl_const(-Y.Btt), Vt2),
        m.nl_mul(VfVt,
            m.nl_sub(
                m.nl_mul(m.nl_const(Y.Gtf), m.nl_sin(θtf)),
                m.nl_mul(m.nl_const(Y.Btf), m.nl_cos(θtf)))));

    P_acc[f] = m.nl_sub(P_acc[f], Pf);
    Q_acc[f] = m.nl_sub(Q_acc[f], Qf);
    P_acc[t] = m.nl_sub(P_acc[t], Pt);
    Q_acc[t] = m.nl_sub(Q_acc[t], Qt);

    branch_flows.push_back({Pf, Qf, Pt, Qt});
  }

  // ── Add power balance equality constraints ─────────────────────────────
  for (std::size_t i = 0; i < nb; ++i) {
    const std::string& bid = data.buses[i].id;
    m.add_nl_constraint("pb_" + bid, P_acc[i], CompareOp::Equal, 0.0);
    m.add_nl_constraint("qb_" + bid, Q_acc[i], CompareOp::Equal, 0.0);
  }

  // ── Reference bus angle constraint: Va[slack] = 0 ─────────────────────
  // Using an equality constraint rather than fixing bounds keeps the IPM feasible.
  if (slack_idx >= 0) {
    m.add_nl_constraint(
        "va_ref_" + data.buses[static_cast<std::size_t>(slack_idx)].id,
        nl_Va[static_cast<std::size_t>(slack_idx)],
        CompareOp::Equal, 0.0);
  }

  // ── Thermal limit inequality constraints (optional) ────────────────────
  // Pf²+Qf² ≤ smax²  (both from-side and to-side)
  // Reuse the flow expressions computed in the balance loop (same ExprIds).
  for (std::size_t k = 0; k < bra.size(); ++k) {
    const auto& ba = bra[k];
    if (ba.rate_a_pu <= 0.0) continue;
    const auto& fl = branch_flows[k];
    const double smax2 = ba.rate_a_pu * ba.rate_a_pu;
    // from-side: Pf²+Qf² ≤ smax²  ↔  Pf²+Qf² − smax² ≤ 0
    m.add_nl_constraint(
        "sf_" + ba.id,
        m.nl_sub(m.nl_add(m.nl_sq(fl.Pf), m.nl_sq(fl.Qf)), m.nl_const(smax2)),
        CompareOp::LessEq, 0.0);
    // to-side
    m.add_nl_constraint(
        "st_" + ba.id,
        m.nl_sub(m.nl_add(m.nl_sq(fl.Pt), m.nl_sq(fl.Qt)), m.nl_const(smax2)),
        CompareOp::LessEq, 0.0);
  }

  // ── Solve ─────────────────────────────────────────────────────────────
  auto sr = m.solve(opts);

  // ── Pack result ────────────────────────────────────────────────────────
  ACOPFBuilderResult res;
  res.solve_result = sr;

  if (!sr.has_primal()) {
    return res;
  }

  // Bus results
  for (const auto& bd : data.buses) {
    Key bk = Key::scalar(bd.id);
    const double vm = sr.var_value(Vm, bk);
    const double va = sr.var_value(Va, bk);
    res.vm_pu[bd.id]  = vm;
    res.va_rad[bd.id] = va;
  }

  // Generator results
  double obj_check = 0.0;
  for (const auto& gd : data.generators) {
    Key gk = Key::scalar(gd.id);
    const double pg_pu = sr.var_value(Pg, gk);
    const double qg_pu = sr.var_value(Qg, gk);
    res.pg_mw  [gd.id] = pg_pu * Sb;
    res.qg_mvar[gd.id] = qg_pu * Sb;
    const double pg_mw = pg_pu * Sb;
    obj_check += gd.cost_c2 * pg_mw * pg_mw + gd.cost_c1 * pg_mw + gd.cost_c0;
  }
  res.obj_per_h = obj_check;

  // Branch flows + violations
  for (const auto& ba : bra) {
    const std::size_t f = ba.f_idx;
    const std::size_t t = ba.t_idx;
    const auto& Y = ba.Y;

    const double vm_f = res.vm_pu.at(data.buses[f].id);
    const double vm_t = res.vm_pu.at(data.buses[t].id);
    const double va_f = res.va_rad.at(data.buses[f].id);
    const double va_t = res.va_rad.at(data.buses[t].id);
    const double θft  = va_f - va_t;

    const double Pf = Y.Gff * vm_f * vm_f
        + vm_f * vm_t * (Y.Gft * std::cos(θft) + Y.Bft * std::sin(θft));
    const double Qf = -Y.Bff * vm_f * vm_f
        + vm_f * vm_t * (Y.Gft * std::sin(θft) - Y.Bft * std::cos(θft));

    res.pf_mw [ba.id] = Pf * Sb;
    res.qf_mw [ba.id] = Qf * Sb;
  }

  // Constraint violation metrics
  {
    std::unordered_map<std::string, double> pg_pu_map, qg_pu_map;
    for (const auto& gd : data.generators) {
      pg_pu_map[gd.id] = res.pg_mw.at(gd.id) / Sb;
      qg_pu_map[gd.id] = res.qg_mvar.at(gd.id) / Sb;
    }

    for (std::size_t i = 0; i < nb; ++i) {
      const auto& bd = data.buses[i];
      const double vm = res.vm_pu.at(bd.id);
      double p_inj = -bd.pd_pu - bd.gs_pu * vm * vm;
      double q_inj = -bd.qd_pu + bd.bs_pu * vm * vm;

      for (const auto& gd : data.generators) {
        if (gd.bus_id == bd.id) {
          p_inj += pg_pu_map.at(gd.id);
          q_inj += qg_pu_map.at(gd.id);
        }
      }
      for (const auto& ba : bra) {
        const auto& Y = ba.Y;
        if (ba.f_idx == i) {
          const double vm_f = res.vm_pu.at(data.buses[ba.f_idx].id);
          const double vm_t = res.vm_pu.at(data.buses[ba.t_idx].id);
          const double θft  = res.va_rad.at(data.buses[ba.f_idx].id)
                            - res.va_rad.at(data.buses[ba.t_idx].id);
          p_inj -= Y.Gff*vm_f*vm_f + vm_f*vm_t*(Y.Gft*std::cos(θft)+Y.Bft*std::sin(θft));
          q_inj -= (-Y.Bff*vm_f*vm_f + vm_f*vm_t*(Y.Gft*std::sin(θft)-Y.Bft*std::cos(θft)));
        } else if (ba.t_idx == i) {
          const double vm_f = res.vm_pu.at(data.buses[ba.f_idx].id);
          const double vm_t = res.vm_pu.at(data.buses[ba.t_idx].id);
          const double θtf  = res.va_rad.at(data.buses[ba.t_idx].id)
                            - res.va_rad.at(data.buses[ba.f_idx].id);
          p_inj -= Y.Gtt*vm_t*vm_t + vm_f*vm_t*(Y.Gtf*std::cos(θtf)+Y.Btf*std::sin(θtf));
          q_inj -= (-Y.Btt*vm_t*vm_t + vm_f*vm_t*(Y.Gtf*std::sin(θtf)-Y.Btf*std::cos(θtf)));
        }
      }
      res.max_p_viol_pu = std::max(res.max_p_viol_pu, std::abs(p_inj));
      res.max_q_viol_pu = std::max(res.max_q_viol_pu, std::abs(q_inj));
    }

    // Thermal violations
    for (const auto& ba : bra) {
      if (ba.rate_a_pu <= 0.0) continue;
      const double pf_pu = res.pf_mw.count(ba.id) ? res.pf_mw.at(ba.id) / Sb : 0.0;
      const double qf_pu = res.qf_mw.count(ba.id) ? res.qf_mw.at(ba.id) / Sb : 0.0;
      const double sf_pu = std::hypot(pf_pu, qf_pu);
      if (sf_pu > ba.rate_a_pu) {
        res.max_thermal_viol_pu = std::max(res.max_thermal_viol_pu,
                                            sf_pu - ba.rate_a_pu);
      }
    }
  }

  return res;
}

}  // namespace hacdcpf::power_models
