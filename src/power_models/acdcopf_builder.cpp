/// acdcopf_builder.cpp
/// ====================
/// Hybrid AC/DC Optimal Power Flow via the AML NonlinearExpr DAG.
///
/// Extends the pure-AC acopf_builder with:
///   • DC bus voltage variables  Vdc_k
///   • DC branch power flows     Pdc_branch = (Vdc_i - Vdc_j) / r_ij
///   • DC bus power balance      Σ branches − Pdc_conv = 0
///   • VSC AC injection vars     Pac_c, Qac_c  (free within bounds)
///   • VSC loss coupling         Pac + Pdc + a + b·Iac + c·Iac² = 0
///   • VDC_Q bus slack           Vdc[k] = Vdc_set  (equality)
///
/// The AC OPF core (branches, balance, thermal) is unchanged.

#include "hacdcpf/power_models/hybrid_opf_model_builder.hpp"
#include "hacdcpf/power_models/branch_admittance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

#include "hacdcpf/model/enums/converter_enums.hpp"

namespace hacdcpf::power_models {

using namespace hacdcpf::aml;

namespace {
static constexpr double kDegToRad = M_PI / 180.0;
static constexpr double kHuge     = 1e6;
static constexpr double kPi       = M_PI;
static constexpr double kEpsIac   = 1e-6;   // numerical regularisation inside sqrt
}  // namespace

// ════════════════════════════════════════════════════════════════════════════
// to_acdcopf_data
// ════════════════════════════════════════════════════════════════════════════

ACDCOPFData to_acdcopf_data(const HybridPowerSystem& sys) {
  ACDCOPFData d;
  d.ac = to_acopf_data(sys);   // reuse existing pure-AC converter

  const double Sb = d.ac.base_mva;

  // ── DC buses ────────────────────────────────────────────────────────────
  std::unordered_map<int, std::string> dc_idx_to_id;
  for (const auto& b : sys.dc.buses) {
    // Isolated DC buses are de-energized and removed from the solve, exactly
    // like out-of-service buses: no Vdc variable, no balance constraint, and
    // their loads are not served.  DC branches that reference them are dropped
    // below because their endpoint never enters dc_idx_to_id.
    if (!b.in_service || b.bus_type == DCBusType::DC_ISOLATED) continue;
    const std::string id = b.name.empty()
        ? ("DC" + std::to_string(b.index))
        : ("DC_" + b.name);
    dc_idx_to_id[b.index] = id;

    ACDCOPFDCBusData bd;
    bd.id       = id;
    bd.vdc_min  = (b.vmin_pu > 0.0) ? b.vmin_pu : 0.9;
    bd.vdc_max  = (b.vmax_pu > 0.0) ? b.vmax_pu : 1.1;
    bd.vdc0     = (b.vm_pu   > 0.0) ? b.vm_pu   : 1.0;
    bd.pd_pu    = 0.0;
    for (const auto& ld : sys.dc.loads) {
      if (ld.in_service && ld.bus == b.index) {
        bd.pd_pu += ld.p_mw / Sb;
      }
    }
    // VDC_Q bus: Vdc fixed (will be identified from converter control modes below)
    d.dc_buses.push_back(std::move(bd));
  }

  // ── DC branches ─────────────────────────────────────────────────────────
  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) continue;
    auto fi = dc_idx_to_id.find(br.from_bus);
    auto ti = dc_idx_to_id.find(br.to_bus);
    if (fi == dc_idx_to_id.end() || ti == dc_idx_to_id.end()) continue;
    const double r = (br.r_pu > 0.0) ? br.r_pu : 1e-4;

    ACDCOPFDCBranchData brd;
    brd.id       = br.name.empty()
        ? (fi->second + "->" + ti->second + "_" + std::to_string(br.index))
        : br.name;
    brd.from_bus = fi->second;
    brd.to_bus   = ti->second;
    brd.r_pu     = r;
    d.dc_branches.push_back(std::move(brd));
  }

  // ── VSC converters ───────────────────────────────────────────────────────
  // Build AC bus id map from the already-constructed ac data
  std::unordered_map<int, std::string> ac_idx_to_id;
  for (const auto& b : sys.ac.buses) {
    if (!b.in_service) continue;
    const std::string id = b.name.empty()
        ? ("B" + std::to_string(b.index))
        : b.name;
    ac_idx_to_id[b.index] = id;
  }

  // Mark VDC_Q DC buses
  for (const auto& conv : sys.vsc_converters) {
    if (!conv.in_service) continue;
    if (conv.control_mode == ConverterMode::VDC_Q) {
      auto it = dc_idx_to_id.find(conv.bus_dc);
      if (it == dc_idx_to_id.end()) continue;
      for (auto& bd : d.dc_buses) {
        if (bd.id == it->second) {
          bd.is_vdc_slack = true;
          bd.vdc_set = (conv.v_dc_set_pu > 0.0) ? conv.v_dc_set_pu : 1.0;
          bd.vdc0    = bd.vdc_set;
          break;
        }
      }
    }
  }

  for (std::size_t ki = 0; ki < sys.vsc_converters.size(); ++ki) {
    const auto& conv = sys.vsc_converters[ki];
    if (!conv.in_service) continue;
    auto ai = ac_idx_to_id.find(conv.bus_ac);
    auto di = dc_idx_to_id.find(conv.bus_dc);
    if (ai == ac_idx_to_id.end() || di == dc_idx_to_id.end()) continue;

    const std::string cid = conv.name.empty()
        ? ("VSC" + std::to_string(conv.index))
        : conv.name;

    ACDCOPFConverterData cd;
    cd.id         = cid;
    cd.ac_bus_id  = ai->second;
    cd.dc_bus_id  = di->second;
    cd.pac_min_pu = conv.pmin_mw / Sb;
    cd.pac_max_pu = conv.pmax_mw / Sb;
    cd.qac_min_pu = (std::isfinite(conv.qmin_mvar) && conv.qmin_mvar > -1e9)
                      ? conv.qmin_mvar / Sb : -kHuge;
    cd.qac_max_pu = (std::isfinite(conv.qmax_mvar) && conv.qmax_mvar <  1e9)
                      ? conv.qmax_mvar / Sb :  kHuge;
    cd.pac0_pu    = std::clamp(conv.p_set_mw / Sb,
                               cd.pac_min_pu, cd.pac_max_pu);
    cd.qac0_pu    = std::clamp(conv.q_set_mvar / Sb,
                               cd.qac_min_pu, cd.qac_max_pu);
    // Loss model coefficients
    cd.a_pu   = conv.loss_mw / Sb;
    cd.b_loss = conv.loss_percent / 100.0;
    cd.c_loss = 1.0 - std::clamp(conv.eta, 0.0, 1.0);
    cd.is_vdc_slack = (conv.control_mode == ConverterMode::VDC_Q);
    d.converters.push_back(std::move(cd));
  }

  return d;
}

// ════════════════════════════════════════════════════════════════════════════
// solve_acdcopf
// ════════════════════════════════════════════════════════════════════════════

ACDCOPFBuilderResult solve_acdcopf(const ACDCOPFData& data,
                                    const aml::SolveOptions& opts_in) {
  const ACOPFData& ac = data.ac;
  if (ac.buses.empty()) {
    throw std::invalid_argument("ACDCOPFData: AC bus list is empty");
  }

  SolveOptions opts = opts_in;
  if (opts.solver_name.empty()) opts.solver_name = "NativeIPM";
  opts.allow_fallback = false;

  const double Sb = (ac.base_mva > 0.0) ? ac.base_mva : 100.0;

  // ── Index maps ────────────────────────────────────────────────────────
  std::unordered_map<std::string, std::size_t> ac_bus_pos, dc_bus_pos;
  ac_bus_pos.reserve(ac.buses.size());
  for (std::size_t i = 0; i < ac.buses.size(); ++i)
    ac_bus_pos[ac.buses[i].id] = i;
  dc_bus_pos.reserve(data.dc_buses.size());
  for (std::size_t i = 0; i < data.dc_buses.size(); ++i)
    dc_bus_pos[data.dc_buses[i].id] = i;

  // ── AML model ─────────────────────────────────────────────────────────
  Model m("acdcopf");

  // ── Sets ──────────────────────────────────────────────────────────────
  auto& ac_bus_set = m.add_set("ac_buses");
  for (const auto& bd : ac.buses)  ac_bus_set.add_element(Key::scalar(bd.id));
  auto& gen_set = m.add_set("gens");
  for (const auto& gd : ac.generators) gen_set.add_element(Key::scalar(gd.id));
  auto& dc_bus_set = m.add_set("dc_buses");
  for (const auto& bd : data.dc_buses) dc_bus_set.add_element(Key::scalar(bd.id));
  auto& conv_set = m.add_set("converters");
  for (const auto& cd : data.converters) conv_set.add_element(Key::scalar(cd.id));

  // ── AC variables ──────────────────────────────────────────────────────
  auto& Vm = m.add_var("Vm", ac_bus_set, VarType::Continuous, 0.9, 1.1);
  auto& Va = m.add_var("Va", ac_bus_set, VarType::Continuous, -kPi, kPi);
  auto& Pg = m.add_var("Pg", gen_set,    VarType::Continuous, 0.0, kHuge);
  auto& Qg = m.add_var("Qg", gen_set,    VarType::Continuous, -kHuge, kHuge);

  int slack_idx = -1;
  for (std::size_t i = 0; i < ac.buses.size(); ++i) {
    const auto& bd = ac.buses[i];
    Key bk = Key::scalar(bd.id);
    Vm.set_lb(bk, bd.vm_min);
    Vm.set_ub(bk, bd.vm_max);
    if (bd.is_ref) slack_idx = static_cast<int>(i);
  }
  for (const auto& gd : ac.generators) {
    Key gk = Key::scalar(gd.id);
    Pg.set_lb(gk, gd.pg_min_pu);
    Pg.set_ub(gk, gd.pg_max_pu);
    Qg.set_lb(gk, gd.qg_min_pu);
    Qg.set_ub(gk, gd.qg_max_pu);
  }

  // ── DC bus voltage variables ───────────────────────────────────────────
  auto& Vdc = m.add_var("Vdc", dc_bus_set, VarType::Continuous, 0.9, 1.1);
  for (const auto& bd : data.dc_buses) {
    Key dk = Key::scalar(bd.id);
    Vdc.set_lb(dk, bd.vdc_min);
    Vdc.set_ub(dk, bd.vdc_max);
  }

  // ── VSC converter AC power variables ──────────────────────────────────
  // Pac: active power injected INTO the AC bus (positive = generator convention)
  // Qac: reactive power injected INTO the AC bus
  auto& Pac = m.add_var("Pac", conv_set, VarType::Continuous, -kHuge, kHuge);
  auto& Qac = m.add_var("Qac", conv_set, VarType::Continuous, -kHuge, kHuge);
  for (const auto& cd : data.converters) {
    Key ck = Key::scalar(cd.id);
    Pac.set_lb(ck, cd.pac_min_pu);
    Pac.set_ub(ck, cd.pac_max_pu);
    Qac.set_lb(ck, cd.qac_min_pu);
    Qac.set_ub(ck, cd.qac_max_pu);
  }

  // ── Objective ─────────────────────────────────────────────────────────
  NonlinearExpr obj = m.nl_const(0.0);
  for (const auto& gd : ac.generators) {
    Key gk = Key::scalar(gd.id);
    NonlinearExpr nlpg = m.nl_var(Pg(gk));
    const double a2 = gd.cost_c2 * Sb * Sb;
    const double a1 = gd.cost_c1 * Sb;
    const double a0 = gd.cost_c0;
    NonlinearExpr fg = m.nl_const(a0);
    if (std::abs(a1) > 0.0)
      fg = m.nl_add(fg, m.nl_mul(m.nl_const(a1), nlpg));
    if (std::abs(a2) > 0.0)
      fg = m.nl_add(fg, m.nl_mul(m.nl_const(a2), m.nl_sq(nlpg)));
    obj = m.nl_add(obj, fg);
  }
  m.minimize(obj);

  // ── Warm start ────────────────────────────────────────────────────────
  {
    std::vector<double> x0;
    x0.reserve(2*ac.buses.size() + 2*ac.generators.size()
               + data.dc_buses.size() + 2*data.converters.size());
    for (const auto& bd : ac.buses)       x0.push_back(bd.vm0);
    for (const auto& bd : ac.buses)       x0.push_back(bd.va0_rad);
    for (const auto& gd : ac.generators)  x0.push_back(gd.pg0_pu);
    for (const auto& gd : ac.generators)  x0.push_back(gd.qg0_pu);
    for (const auto& bd : data.dc_buses)  x0.push_back(bd.vdc0);
    for (const auto& cd : data.converters) x0.push_back(cd.pac0_pu);
    for (const auto& cd : data.converters) x0.push_back(cd.qac0_pu);
    m.set_nlp_x0(x0);
  }

  // ── Precompute AC branch admittances ──────────────────────────────────
  struct BranchAdm {
    std::size_t f_idx, t_idx;
    BranchAdmittance Y;
    double rate_a_pu;
    std::string id;
  };
  std::vector<BranchAdm> bra;
  bra.reserve(ac.branches.size());
  for (const auto& br : ac.branches) {
    auto fi = ac_bus_pos.find(br.from_bus);
    auto ti = ac_bus_pos.find(br.to_bus);
    if (fi == ac_bus_pos.end() || ti == ac_bus_pos.end()) continue;
    BranchAdm ba;
    ba.f_idx    = fi->second;
    ba.t_idx    = ti->second;
    ba.Y        = branch_admittance(br.r_pu, br.x_pu, br.bc_pu,
                                     br.tap, br.shift_deg * kDegToRad);
    ba.rate_a_pu = br.rate_a_pu;
    ba.id        = br.id;
    bra.push_back(std::move(ba));
  }

  // ── Build AC nl_Vm / nl_Va helpers ────────────────────────────────────
  const std::size_t nb_ac = ac.buses.size();
  std::vector<NonlinearExpr> nl_Vm, nl_Va;
  nl_Vm.reserve(nb_ac); nl_Va.reserve(nb_ac);
  for (const auto& bd : ac.buses) {
    nl_Vm.push_back(m.nl_var(Vm(Key::scalar(bd.id))));
    nl_Va.push_back(m.nl_var(Va(Key::scalar(bd.id))));
  }

  // ── AC bus power balance accumulators ─────────────────────────────────
  std::vector<NonlinearExpr> P_acc(nb_ac, m.nl_const(0.0));
  std::vector<NonlinearExpr> Q_acc(nb_ac, m.nl_const(0.0));
  for (std::size_t i = 0; i < nb_ac; ++i) {
    const auto& bd = ac.buses[i];
    NonlinearExpr p = m.nl_mul(m.nl_const(-bd.gs_pu), m.nl_sq(nl_Vm[i]));
    if (std::abs(bd.pd_pu) > 0.0)
      p = m.nl_sub(p, m.nl_const(bd.pd_pu));
    NonlinearExpr q = m.nl_mul(m.nl_const(bd.bs_pu), m.nl_sq(nl_Vm[i]));
    if (std::abs(bd.qd_pu) > 0.0)
      q = m.nl_sub(q, m.nl_const(bd.qd_pu));
    P_acc[i] = p; Q_acc[i] = q;
  }

  // Generator injections
  for (const auto& gd : ac.generators) {
    auto bit = ac_bus_pos.find(gd.bus_id);
    if (bit == ac_bus_pos.end()) continue;
    std::size_t i = bit->second;
    Key gk = Key::scalar(gd.id);
    P_acc[i] = m.nl_add(P_acc[i], m.nl_var(Pg(gk)));
    Q_acc[i] = m.nl_add(Q_acc[i], m.nl_var(Qg(gk)));
  }

  // Subtract AC branch flows
  struct BranchFlow { NonlinearExpr Pf, Qf, Pt, Qt; };
  std::vector<BranchFlow> branch_flows;
  branch_flows.reserve(bra.size());
  for (const auto& ba : bra) {
    const std::size_t f = ba.f_idx, t = ba.t_idx;
    const auto& Y = ba.Y;
    NonlinearExpr Vf = nl_Vm[f], Vt = nl_Vm[t];
    NonlinearExpr θf = nl_Va[f], θt = nl_Va[t];
    NonlinearExpr θft = m.nl_sub(θf, θt);
    NonlinearExpr θtf = m.nl_sub(θt, θf);
    NonlinearExpr VfVt = m.nl_mul(Vf, Vt);
    NonlinearExpr Vf2  = m.nl_sq(Vf), Vt2 = m.nl_sq(Vt);

    NonlinearExpr Pf = m.nl_add(
        m.nl_mul(m.nl_const(Y.Gff), Vf2),
        m.nl_mul(VfVt, m.nl_add(
            m.nl_mul(m.nl_const(Y.Gft), m.nl_cos(θft)),
            m.nl_mul(m.nl_const(Y.Bft), m.nl_sin(θft)))));
    NonlinearExpr Qf = m.nl_add(
        m.nl_mul(m.nl_const(-Y.Bff), Vf2),
        m.nl_mul(VfVt, m.nl_sub(
            m.nl_mul(m.nl_const(Y.Gft), m.nl_sin(θft)),
            m.nl_mul(m.nl_const(Y.Bft), m.nl_cos(θft)))));
    NonlinearExpr Pt = m.nl_add(
        m.nl_mul(m.nl_const(Y.Gtt), Vt2),
        m.nl_mul(VfVt, m.nl_add(
            m.nl_mul(m.nl_const(Y.Gtf), m.nl_cos(θtf)),
            m.nl_mul(m.nl_const(Y.Btf), m.nl_sin(θtf)))));
    NonlinearExpr Qt = m.nl_add(
        m.nl_mul(m.nl_const(-Y.Btt), Vt2),
        m.nl_mul(VfVt, m.nl_sub(
            m.nl_mul(m.nl_const(Y.Gtf), m.nl_sin(θtf)),
            m.nl_mul(m.nl_const(Y.Btf), m.nl_cos(θtf)))));

    P_acc[f] = m.nl_sub(P_acc[f], Pf);
    Q_acc[f] = m.nl_sub(Q_acc[f], Qf);
    P_acc[t] = m.nl_sub(P_acc[t], Pt);
    Q_acc[t] = m.nl_sub(Q_acc[t], Qt);
    branch_flows.push_back({Pf, Qf, Pt, Qt});
  }

  // ── Subtract VSC AC injections from AC balance ─────────────────────────
  // Pac_c is injected INTO the AC bus (positive = into AC)
  for (const auto& cd : data.converters) {
    auto bit = ac_bus_pos.find(cd.ac_bus_id);
    if (bit == ac_bus_pos.end()) continue;
    std::size_t i = bit->second;
    Key ck = Key::scalar(cd.id);
    // Pac injected into bus i: subtract from "generation minus load" balance
    // P_acc[i] += Pac_c (Pac already in pu with sign: + = into bus)
    P_acc[i] = m.nl_add(P_acc[i], m.nl_var(Pac(ck)));
    Q_acc[i] = m.nl_add(Q_acc[i], m.nl_var(Qac(ck)));
  }

  // ── Add AC equality constraints ────────────────────────────────────────
  for (std::size_t i = 0; i < nb_ac; ++i) {
    const std::string& bid = ac.buses[i].id;
    m.add_nl_constraint("pb_" + bid, P_acc[i], CompareOp::Equal, 0.0);
    m.add_nl_constraint("qb_" + bid, Q_acc[i], CompareOp::Equal, 0.0);
  }
  if (slack_idx >= 0) {
    m.add_nl_constraint(
        "va_ref_" + ac.buses[static_cast<std::size_t>(slack_idx)].id,
        nl_Va[static_cast<std::size_t>(slack_idx)],
        CompareOp::Equal, 0.0);
  }

  // ── AC thermal limits ─────────────────────────────────────────────────
  for (std::size_t k = 0; k < bra.size(); ++k) {
    const auto& ba = bra[k];
    if (ba.rate_a_pu <= 0.0) continue;
    const auto& fl = branch_flows[k];
    const double smax2 = ba.rate_a_pu * ba.rate_a_pu;
    m.add_nl_constraint("sf_" + ba.id,
        m.nl_sub(m.nl_add(m.nl_sq(fl.Pf), m.nl_sq(fl.Qf)), m.nl_const(smax2)),
        CompareOp::LessEq, 0.0);
    m.add_nl_constraint("st_" + ba.id,
        m.nl_sub(m.nl_add(m.nl_sq(fl.Pt), m.nl_sq(fl.Qt)), m.nl_const(smax2)),
        CompareOp::LessEq, 0.0);
  }

  // ── DC bus balance accumulators ────────────────────────────────────────
  const std::size_t nb_dc = data.dc_buses.size();
  std::vector<NonlinearExpr> Pdc_acc(nb_dc, m.nl_const(0.0));

  // Subtract fixed DC loads from DC balance
  for (std::size_t i = 0; i < nb_dc; ++i) {
    if (std::abs(data.dc_buses[i].pd_pu) > 0.0)
      Pdc_acc[i] = m.nl_const(-data.dc_buses[i].pd_pu);
  }

  // ── DC nl_Vdc helpers ─────────────────────────────────────────────────
  std::vector<NonlinearExpr> nl_Vdc;
  nl_Vdc.reserve(nb_dc);
  for (const auto& bd : data.dc_buses)
    nl_Vdc.push_back(m.nl_var(Vdc(Key::scalar(bd.id))));

  // Inject DC branch flows: P_ij = (Vdc_i - Vdc_j) / r_ij  (linear in Vdc)
  // Branch contributes +P_ij to bus i, −P_ij to bus j (sending convention).
  for (const auto& br : data.dc_branches) {
    auto fi = dc_bus_pos.find(br.from_bus);
    auto ti = dc_bus_pos.find(br.to_bus);
    if (fi == dc_bus_pos.end() || ti == dc_bus_pos.end()) continue;
    const std::size_t f = fi->second, t = ti->second;
    const double inv_r = 1.0 / br.r_pu;
    // P_ij = (Vdc_f - Vdc_t) / r
    NonlinearExpr P_br = m.nl_mul(m.nl_const(inv_r),
                                   m.nl_sub(nl_Vdc[f], nl_Vdc[t]));
    // DC balance: generation − consumption = 0
    // Sending bus f: loses P_ij  → subtract
    Pdc_acc[f] = m.nl_sub(Pdc_acc[f], P_br);
    // Receiving bus t: gains P_ij → add
    Pdc_acc[t] = m.nl_add(Pdc_acc[t], P_br);
  }

  // ── VSC converter coupling ────────────────────────────────────────────
  // For converter c with AC bus a and DC bus d:
  //   Pac_c flows INTO the AC bus (already subtracted above).
  //   Pdc_c flows INTO the DC bus (so we add it to Pdc_acc[d]).
  //   Power balance: Pac_c + Pdc_c + a_c + b_c·Iac_c + c_c·Iac_c² = 0
  //   where Iac_c = sqrt(Pac_c² + Qac_c² + ε) / Vm[a]
  //   → Pdc_c = −Pac_c − a_c − b_c·Iac_c − c_c·Iac_c²
  //
  // We express Pdc_c as a NonlinearExpr and add it directly to Pdc_acc[d].
  // Then we have NO separate equality constraint for the converter — the DC bus
  // balance implicitly encodes it.

  for (const auto& cd : data.converters) {
    auto ai = ac_bus_pos.find(cd.ac_bus_id);
    auto di = dc_bus_pos.find(cd.dc_bus_id);
    if (ai == ac_bus_pos.end() || di == dc_bus_pos.end()) continue;
    const std::size_t d_idx = di->second;
    Key ck = Key::scalar(cd.id);

    NonlinearExpr nl_Pac = m.nl_var(Pac(ck));
    NonlinearExpr nl_Qac = m.nl_var(Qac(ck));
    NonlinearExpr nl_Vm_ac = nl_Vm[ai->second];

    // Iac² = (Pac² + Qac² + ε) / Vm²
    NonlinearExpr Iac2 = m.nl_div(
        m.nl_add(m.nl_add(m.nl_sq(nl_Pac), m.nl_sq(nl_Qac)), m.nl_const(kEpsIac * kEpsIac)),
        m.nl_sq(nl_Vm_ac));
    // Iac  = sqrt(Iac²)
    NonlinearExpr Iac = m.nl_sqrt(Iac2);

    // ploss = a + b·Iac + c·Iac²
    NonlinearExpr ploss = m.nl_const(cd.a_pu);
    if (std::abs(cd.b_loss) > 1e-12)
      ploss = m.nl_add(ploss, m.nl_mul(m.nl_const(cd.b_loss), Iac));
    if (std::abs(cd.c_loss) > 1e-12)
      ploss = m.nl_add(ploss, m.nl_mul(m.nl_const(cd.c_loss), Iac2));

    // Pdc_c = −Pac_c − ploss  (flows into DC bus, positive convention)
    NonlinearExpr Pdc_c = m.nl_neg(m.nl_add(nl_Pac, ploss));
    Pdc_acc[d_idx] = m.nl_add(Pdc_acc[d_idx], Pdc_c);
  }

  // ── DC bus balance equality constraints ───────────────────────────────
  for (std::size_t i = 0; i < nb_dc; ++i) {
    const std::string& did = data.dc_buses[i].id;
    m.add_nl_constraint("pdc_bal_" + did, Pdc_acc[i], CompareOp::Equal, 0.0);
  }

  // ── VDC slack bus equality: Vdc[k] = Vdc_set ──────────────────────────
  for (const auto& bd : data.dc_buses) {
    if (!bd.is_vdc_slack) continue;
    m.add_nl_constraint(
        "vdc_set_" + bd.id,
        nl_Vdc[dc_bus_pos.at(bd.id)],
        CompareOp::Equal, bd.vdc_set);
  }

  // ── Solve ─────────────────────────────────────────────────────────────
  auto sr = m.solve(opts);

  // ── Pack result ───────────────────────────────────────────────────────
  ACDCOPFBuilderResult res;
  res.ac_result.solve_result = sr;

  if (!sr.has_primal()) return res;

  // AC bus results
  for (const auto& bd : ac.buses) {
    Key bk = Key::scalar(bd.id);
    res.ac_result.vm_pu [bd.id] = sr.var_value(Vm, bk);
    res.ac_result.va_rad[bd.id] = sr.var_value(Va, bk);
  }

  // Generator results + objective
  double obj_val = 0.0;
  for (const auto& gd : ac.generators) {
    Key gk = Key::scalar(gd.id);
    const double pg_pu = sr.var_value(Pg, gk);
    const double qg_pu = sr.var_value(Qg, gk);
    res.ac_result.pg_mw  [gd.id] = pg_pu * Sb;
    res.ac_result.qg_mvar[gd.id] = qg_pu * Sb;
    const double pg_mw = pg_pu * Sb;
    obj_val += gd.cost_c2 * pg_mw * pg_mw + gd.cost_c1 * pg_mw + gd.cost_c0;
  }
  res.ac_result.obj_per_h = obj_val;

  // DC bus results
  for (const auto& bd : data.dc_buses) {
    Key dk = Key::scalar(bd.id);
    res.vdc_pu[bd.id] = sr.var_value(Vdc, dk);
  }

  // Converter results
  for (const auto& cd : data.converters) {
    Key ck = Key::scalar(cd.id);
    const double pac_pu = sr.var_value(Pac, ck);
    const double qac_pu = sr.var_value(Qac, ck);
    res.pac_mw  [cd.id] = pac_pu * Sb;
    res.qac_mvar[cd.id] = qac_pu * Sb;

    // Back-compute Pdc from loss model
    const double vm_ac = res.ac_result.vm_pu.at(cd.ac_bus_id);
    const double iac = std::sqrt(pac_pu * pac_pu + qac_pu * qac_pu + kEpsIac * kEpsIac)
                       / std::max(vm_ac, 1e-6);
    const double ploss = cd.a_pu + cd.b_loss * iac + cd.c_loss * iac * iac;
    res.pdc_mw[cd.id] = -(pac_pu + ploss) * Sb;
  }

  // ── AC constraint violation metrics ────────────────────────────────────
  // Recompute nodal balance mismatch from the solution
  {
    std::unordered_map<std::string, double> pg_pu_map, qg_pu_map;
    for (const auto& gd : ac.generators) {
      pg_pu_map[gd.id] = res.ac_result.pg_mw .at(gd.id) / Sb;
      qg_pu_map[gd.id] = res.ac_result.qg_mvar.at(gd.id) / Sb;
    }
    for (std::size_t i = 0; i < nb_ac; ++i) {
      const auto& bd = ac.buses[i];
      const double vm = res.ac_result.vm_pu.at(bd.id);
      double p_inj = -bd.pd_pu - bd.gs_pu * vm * vm;
      double q_inj = -bd.qd_pu + bd.bs_pu * vm * vm;
      for (const auto& gd : ac.generators)
        if (gd.bus_id == bd.id) { p_inj += pg_pu_map.at(gd.id); q_inj += qg_pu_map.at(gd.id); }
      // Subtract converter injections
      for (const auto& cd : data.converters) {
        if (cd.ac_bus_id == bd.id) {
          p_inj += res.pac_mw.at(cd.id) / Sb;
          q_inj += res.qac_mvar.at(cd.id) / Sb;
        }
      }
      for (const auto& ba : bra) {
        const auto& Y = ba.Y;
        if (ba.f_idx == i) {
          const double vmf = res.ac_result.vm_pu.at(ac.buses[ba.f_idx].id);
          const double vmt = res.ac_result.vm_pu.at(ac.buses[ba.t_idx].id);
          const double θft = res.ac_result.va_rad.at(ac.buses[ba.f_idx].id)
                           - res.ac_result.va_rad.at(ac.buses[ba.t_idx].id);
          p_inj -= Y.Gff*vmf*vmf + vmf*vmt*(Y.Gft*std::cos(θft)+Y.Bft*std::sin(θft));
          q_inj -= (-Y.Bff*vmf*vmf + vmf*vmt*(Y.Gft*std::sin(θft)-Y.Bft*std::cos(θft)));
        } else if (ba.t_idx == i) {
          const double vmf = res.ac_result.vm_pu.at(ac.buses[ba.f_idx].id);
          const double vmt = res.ac_result.vm_pu.at(ac.buses[ba.t_idx].id);
          const double θtf = res.ac_result.va_rad.at(ac.buses[ba.t_idx].id)
                           - res.ac_result.va_rad.at(ac.buses[ba.f_idx].id);
          p_inj -= Y.Gtt*vmt*vmt + vmf*vmt*(Y.Gtf*std::cos(θtf)+Y.Btf*std::sin(θtf));
          q_inj -= (-Y.Btt*vmt*vmt + vmf*vmt*(Y.Gtf*std::sin(θtf)-Y.Btf*std::cos(θtf)));
        }
      }
      res.ac_result.max_p_viol_pu = std::max(res.ac_result.max_p_viol_pu, std::abs(p_inj));
      res.ac_result.max_q_viol_pu = std::max(res.ac_result.max_q_viol_pu, std::abs(q_inj));
    }
  }

  // ── DC constraint violation ────────────────────────────────────────────
  for (std::size_t i = 0; i < nb_dc; ++i) {
    const auto& bd = data.dc_buses[i];
    double pbal = -bd.pd_pu;
    const double vdc_i = res.vdc_pu.at(bd.id);
    // DC branch flows
    for (const auto& br : data.dc_branches) {
      if (br.from_bus == bd.id) {
        const double vdc_j = res.vdc_pu.at(br.to_bus);
        pbal -= (vdc_i - vdc_j) / br.r_pu;
      } else if (br.to_bus == bd.id) {
        const double vdc_j = res.vdc_pu.at(br.from_bus);
        pbal += (vdc_j - vdc_i) / br.r_pu;
      }
    }
    // Converter injections into DC bus
    for (const auto& cd : data.converters) {
      if (cd.dc_bus_id == bd.id) {
        pbal += res.pdc_mw.at(cd.id) / Sb;
      }
    }
    res.max_dc_p_viol_pu = std::max(res.max_dc_p_viol_pu, std::abs(pbal));
  }

  return res;
}

}  // namespace hacdcpf::power_models
