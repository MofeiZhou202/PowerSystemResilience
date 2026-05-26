// =============================================================================
// three_stage_reliability.cpp
//
// Native C++ three-stage fault-recovery reliability evaluator.  The previous
// implementation was a Julia process bridge; this version keeps the public JSON
// schema but evaluates the staged load restoration with the embedded MIPSolvers
// C++ MILP engine.
// =============================================================================

#include "hacdcpf/analysis/three_stage_reliability.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "hacdcpf/engine/branch_and_cut.hpp"
#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/io/json_io.hpp"

namespace fs = std::filesystem;
namespace hacdcpf::analysis {
namespace {

constexpr double kTauSwitchHr = 1.0 / 60.0;
constexpr double kTauTrippingHr = 1.0 / 30.0;
constexpr double kTauRepairHr = 1.0;
constexpr double kReliabilityVoll = 10.0;
constexpr double kDefaultFailureRate = 0.1;

// DC bus IDs are shifted by kDCBusOffset throughout the NativeCase model so
// that DC bus k is represented as (k + kDCBusOffset) in the c.buses / c.loads /
// c.sources vectors.  This prevents ID collisions when AC bus i and DC bus i
// both exist in the hybrid system (e.g. AC bus 1 vs DC bus 1).  The offset is
// purely internal; it is never exposed to the caller.
constexpr int kDCBusOffset = 1'000'000;
struct LoadPoint {
  int bus{0};
  double p_kw{0.0};
  double customers{1.0};
};

struct SourcePoint {
  int bus{0};
  double p_kw{0.0};
};

struct FaultLine {
  int id{0};
  bool ac{true};
  int index{0};
  int from_bus{0};
  int to_bus{0};
  bool normally_in_service{true};
  double failure_rate{0.0};
};

struct NativeCase {
  HybridPowerSystem sys;
  std::vector<int> buses;
  std::vector<LoadPoint> loads;
  std::vector<SourcePoint> sources;
  std::vector<FaultLine> faults;
};

std::string read_file_text(const fs::path& path) {
  std::ifstream in(path);
  if (!in) return {};
  return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void add_bus(std::vector<int>& buses, int bus) {
  if (bus <= 0) return;
  if (std::find(buses.begin(), buses.end(), bus) == buses.end()) buses.push_back(bus);
}

std::unordered_map<int, int> bus_position_map(const std::vector<int>& buses) {
  std::unordered_map<int, int> out;
  out.reserve(buses.size());
  for (int i = 0; i < static_cast<int>(buses.size()); ++i) out[buses[i]] = i;
  return out;
}

double load_customers(const HybridPowerSystem& sys, const Load& ld) {
  if (ld.n_customers > 0) return static_cast<double>(ld.n_customers);
  for (const auto& b : sys.ac.buses) {
    if (b.index == ld.bus && b.n_customers > 0) return static_cast<double>(b.n_customers);
  }
  return std::max(1.0, ld.p_mw * 10.0);
}

double bus_customers(const ACBus& b) {
  if (b.n_customers > 0) return static_cast<double>(b.n_customers);
  return std::max(1.0, b.pd_mw * 10.0);
}

void add_source(std::vector<SourcePoint>& sources, int bus, double p_mw) {
  if (bus <= 0 || p_mw <= 1e-9) return;
  sources.push_back({bus, p_mw * 1000.0});
}

NativeCase build_native_case(const HybridPowerSystem& sys) {
  NativeCase c;
  c.sys = sys;

  for (const auto& b : sys.ac.buses) {
    if (b.in_service) add_bus(c.buses, b.index);
  }
  for (const auto& b : sys.dc.buses) {
    if (b.in_service) add_bus(c.buses, b.index + kDCBusOffset);
  }

  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service) continue;
    add_bus(c.buses, ld.bus);
    c.loads.push_back({ld.bus, std::max(0.0, ld.p_mw * ld.scaling * 1000.0), load_customers(sys, ld)});
  }
  for (const auto& b : sys.ac.buses) {
    if (!b.in_service || b.pd_mw <= 1e-9) continue;
    c.loads.push_back({b.index, b.pd_mw * 1000.0, bus_customers(b)});
  }
  for (const auto& ld : sys.dc.loads) {
    if (!ld.in_service) continue;
    add_bus(c.buses, ld.bus + kDCBusOffset);
    c.loads.push_back({ld.bus + kDCBusOffset, std::max(0.0, ld.p_mw * ld.scaling * 1000.0),
                       std::max(1.0, ld.p_mw * 10.0)});
  }
  for (const auto& b : sys.dc.buses) {
    if (!b.in_service || b.pd_mw <= 1e-9) continue;
    c.loads.push_back({b.index + kDCBusOffset, b.pd_mw * 1000.0,
                       b.n_customers > 0 ? static_cast<double>(b.n_customers) : std::max(1.0, b.pd_mw * 10.0)});
  }

  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    add_source(c.sources, eg.bus, eg.s_sc_max_mva > 0.0 ? eg.s_sc_max_mva : 1.0e4);
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    add_source(c.sources, g.bus, g.pmax_mw > 0.0 ? g.pmax_mw : std::max(0.0, g.pg_mw));
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    double p = sg.pmax_mw > 0.0 ? sg.pmax_mw : (sg.p_rated_mw > 0.0 ? sg.p_rated_mw : sg.p_mw * sg.scaling);
    add_source(c.sources, sg.bus, p);
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    add_source(c.sources, rg.bus, rg.p_rated_mw > 0.0 ? rg.p_rated_mw * rg.capacity_factor : rg.p_mw);
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    add_source(c.sources, pv.bus, pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw);
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    double p = st.pmax_mw > 0.0 ? st.pmax_mw : st.p_rated_mw;
    add_source(c.sources, st.bus, p);
  }
  for (const auto& g : sys.dc.dc_static_generators) {
    if (!g.in_service) continue;
    add_source(c.sources, g.bus + kDCBusOffset, g.pmax_mw > 0.0 ? g.pmax_mw : g.p_set_mw * g.scaling);
  }
  for (const auto& g : sys.dc.static_generators) {
    if (!g.in_service) continue;
    double p = g.pmax_mw > 0.0 ? g.pmax_mw : (g.p_rated_mw > 0.0 ? g.p_rated_mw : g.p_mw * g.scaling);
    add_source(c.sources, g.bus + kDCBusOffset, p);
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    add_source(c.sources, pv.bus + kDCBusOffset, pv.p_set_mw);
  }
  for (const auto& b : sys.dc.buses) {
    if (b.in_service && b.bus_type == DCBusType::DC_V) add_source(c.sources, b.index + kDCBusOffset, 1.0e4);
  }

  int id = 1;
  for (int i = 0; i < static_cast<int>(sys.ac.branches.size()); ++i) {
    const auto& br = sys.ac.branches[i];
    add_bus(c.buses, br.from_bus);
    add_bus(c.buses, br.to_bus);
    if (!br.in_service) continue;  // out-of-service branches cannot fail
    c.faults.push_back({id++, true, i, br.from_bus, br.to_bus, br.in_service,
                        br.failure_rate > 0.0 ? br.failure_rate : kDefaultFailureRate});
  }
  for (int i = 0; i < static_cast<int>(sys.dc.branches.size()); ++i) {
    const auto& br = sys.dc.branches[i];
    add_bus(c.buses, br.from_bus + kDCBusOffset);
    add_bus(c.buses, br.to_bus + kDCBusOffset);
    if (!br.in_service) continue;  // out-of-service branches cannot fail
    double lambda = br.mtbf_hours > 0.0 ? 8760.0 / br.mtbf_hours : kDefaultFailureRate;
    // from_bus/to_bus stored without offset — informational only, not used in stage_components
    c.faults.push_back({id++, false, i, br.from_bus, br.to_bus, br.in_service, lambda});
  }

  std::sort(c.buses.begin(), c.buses.end());
  c.buses.erase(std::unique(c.buses.begin(), c.buses.end()), c.buses.end());
  return c;
}

struct DSU {
  std::vector<int> p;
  explicit DSU(int n) : p(n) { std::iota(p.begin(), p.end(), 0); }
  int find(int x) { return p[x] == x ? x : p[x] = find(p[x]); }
  void unite(int a, int b) {
    a = find(a); b = find(b);
    if (a != b) p[b] = a;
  }
};

// stage_components() — build connected-component map for the given fault stage.
//
// MODEL SCOPE (connectivity-only, no physical constraints):
//   Stage 1 — fault island: fault branch open, all other in-service branches/switches
//             in their nominal state.
//   Stage 2 — switching: fault branch stays open; ALL other in-service branches and
//             switches are closed (best-case connectivity restoration).
//             LIMITATION: this does not model radial-topology constraints, maximum
//             switch-operation counts, power-flow limits, or voltage limits.
//             It is a pure reachability model — if a source can reach a load bus via
//             any path, the load is "restorable".  Physical feasibility of the
//             resulting topology is NOT checked.
//   Stage 3 — repair: fault branch restored; all originally-in-service elements on.
//
// SOP/VSC dispatch is NOT modelled here; VSC converters are treated as lossless
// graph edges (always connected when in_service).  Actual SOP dispatch setpoints
// are NOT co-optimised; psop vectors in FaultDetail are filled with zeros.
std::unordered_map<int, int> stage_components(const NativeCase& c,
                                              const FaultLine& fault,
                                              int stage) {
  auto pos = bus_position_map(c.buses);
  DSU dsu(static_cast<int>(c.buses.size()));
  auto connect = [&](int a, int b) {
    auto ia = pos.find(a), ib = pos.find(b);
    if (ia != pos.end() && ib != pos.end()) dsu.unite(ia->second, ib->second);
  };
  for (int i = 0; i < static_cast<int>(c.sys.ac.branches.size()); ++i) {
    const auto& br = c.sys.ac.branches[i];
    bool on = br.in_service;
    if (fault.ac && i == fault.index && stage < 3) on = false;
    if (!on && stage >= 2 && br.in_service && !(fault.ac && i == fault.index)) on = true;
    if (stage >= 3 && br.in_service) on = true;  // repair restores only originally-in-service branches
    if (on) connect(br.from_bus, br.to_bus);
  }
  for (const auto& sw : c.sys.ac.switches) {
    bool on = sw.in_service && sw.closed;
    if (stage >= 2 && sw.in_service) on = true;
    if (on) connect(sw.bus_from, sw.bus_to);
  }
  for (const auto& tr : c.sys.ac.transformers_2w) {
    if (tr.in_service) connect(tr.hv_bus, tr.lv_bus);
  }
  for (const auto& tr : c.sys.ac.transformers_3w) {
    if (!tr.in_service) continue;
    connect(tr.hv_bus, tr.mv_bus);
    connect(tr.hv_bus, tr.lv_bus);
  }
  for (int i = 0; i < static_cast<int>(c.sys.dc.branches.size()); ++i) {
    const auto& br = c.sys.dc.branches[i];
    bool on = br.in_service;
    if (!fault.ac && i == fault.index && stage < 3) on = false;
    if (!on && stage >= 2 && br.in_service && !(!fault.ac && i == fault.index)) on = true;
    if (stage >= 3 && br.in_service) on = true;  // repair restores only originally-in-service branches
    if (on) connect(br.from_bus + kDCBusOffset, br.to_bus + kDCBusOffset);
  }
  for (const auto& vsc : c.sys.vsc_converters) {
    // VSC crosses domains: AC side has no offset, DC side has kDCBusOffset
    if (vsc.in_service) connect(vsc.bus_ac, vsc.bus_dc + kDCBusOffset);
  }
  for (const auto& dc : c.sys.dc.dcdc_converters) {
    if (dc.in_service) connect(dc.bus_in + kDCBusOffset, dc.bus_out + kDCBusOffset);
  }

  std::unordered_map<int, int> out;
  for (int i = 0; i < static_cast<int>(c.buses.size()); ++i) out[c.buses[i]] = dsu.find(i);
  return out;
}

struct StageSolve {
  double shed_kw{0.0};
  std::vector<double> shed_by_load;
  std::string status{"unknown"};
  double objective{0.0};
};

StageSolve solve_stage_milp(const NativeCase& c, const FaultLine& fault, int stage) {
  StageSolve out;
  const int nd = static_cast<int>(c.loads.size());
  out.shed_by_load.assign(c.loads.size(), 0.0);
  if (nd == 0) {
    out.status = "success";
    return out;
  }

  const auto comp = stage_components(c, fault, stage);
  std::unordered_map<int, double> comp_cap;
  for (const auto& s : c.sources) {
    auto it = comp.find(s.bus);
    if (it != comp.end()) comp_cap[it->second] += s.p_kw;
  }

  // ── Analytic shortcut ──────────────────────────────────────────────────────
  // If every connected component has sufficient capacity to serve all its loads,
  // the optimal all-or-nothing solution is trivially zero shed — skip B&C.
  {
    std::unordered_map<int, double> comp_demand;
    for (int i = 0; i < nd; ++i) {
      const double li = std::max(0.0, c.loads[i].p_kw);
      auto it = comp.find(c.loads[i].bus);
      if (it != comp.end()) comp_demand[it->second] += li;
      // Loads on isolated buses always shed — any isolation disqualifies shortcut.
      else if (li > 1e-6) goto full_milp;
    }
    {
      bool feasible = true;
      for (const auto& [cid, demand] : comp_demand) {
        const auto ci = comp_cap.find(cid);
        if (ci == comp_cap.end() || ci->second < demand - 1e-6) {
          feasible = false;
          break;
        }
      }
      if (feasible) {
        out.status = "success (analytical)";
        out.shed_kw = 0.0;
        std::fill(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
        return out;
      }
    }
  }
  full_milp:

  engine::MIPModel mip;
  auto& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(nd);
  lp.vars.resize(nd);
  for (int i = 0; i < nd; ++i) {
    const double li = std::max(0.0, c.loads[i].p_kw);
    // z_i ∈ {0,1}: 1 = load i completely shed (not served), 0 = fully served.
    // Discrete all-or-nothing restorability is the standard assumption in
    // distribution reliability assessment (IEC/DL-T 836, IEEE Std 1366).
    lp.vars[i] = {engine::VarType::Binary, 0.0, 1.0, "z_" + std::to_string(i)};
    lp.c[i] = li;  // minimize Σ z_i · load_i (total shed kW)
    mip.binary_idx.push_back(i);  // required: native B&C takes LP fast-path when binary_idx is empty
  }
  // Loads whose bus is unreachable (isolated) must be completely shed.
  for (int i = 0; i < nd; ++i) {
    if (comp.find(c.loads[i].bus) == comp.end()) {
      lp.vars[i].lb = 1.0;
    }
  }

  std::unordered_map<int, std::vector<int>> loads_by_comp;
  for (int i = 0; i < nd; ++i) {
    auto it = comp.find(c.loads[i].bus);
    if (it == comp.end()) continue;
    loads_by_comp[it->second].push_back(i);
  }

  std::vector<Eigen::Triplet<double>> trips;
  Eigen::VectorXd b(static_cast<int>(loads_by_comp.size()));
  int row = 0;
  for (const auto& [cid, ids] : loads_by_comp) {
    double demand = 0.0;
    for (int i : ids) demand += std::max(0.0, c.loads[i].p_kw);
    const double cap = comp_cap[cid];
    const double min_shed = std::max(0.0, demand - cap);
    // Constraint: Σ z_i · load_i ≥ min_shed  →  -Σ z_i · load_i ≤ -min_shed
    for (int i : ids) trips.emplace_back(row, i, -std::max(0.0, c.loads[i].p_kw));
    b[row] = -min_shed;
    ++row;
  }
  lp.A.resize(row, nd);
  lp.A.setFromTriplets(trips.begin(), trips.end());
  lp.A.makeCompressed();
  lp.b = b;
  lp.Aeq.resize(0, nd);
  lp.beq.resize(0);

  engine::BCOptions opt;
  opt.max_nodes = 512;
  opt.time_limit_sec = 5.0;
  opt.gap_tol = 1e-9;
  opt.verbose = false;
  opt.use_simplex_lp_nodes = true;
  auto res = engine::solve_milp_bc(mip, opt);
  if (!res.stats.success || res.x.size() != nd) {
    out.status = res.stats.status.empty() ? "failed" : res.stats.status;
    for (int i = 0; i < nd; ++i) out.shed_by_load[i] = c.loads[i].p_kw;
  } else {
    out.status = "success";
    out.objective = res.stats.objective;
    // Post-solve constraint verification: Ax ≤ b
    if (lp.A.rows() > 0) {
      Eigen::VectorXd Ax = lp.A * res.x;
      double ineq_viol = (Ax - lp.b).cwiseMax(0.0).maxCoeff();
      if (ineq_viol > 1e-6) {
        spdlog::warn("[三阶段可靠性] MIP 后验约束违约: ineq_viol={:.2e}", ineq_viol);
        out.status = "success (constraint violation detected)";
      }
    }
    // Map binary z_i back to shed kW: z_i > 0.5 → load completely shed.
    for (int i = 0; i < nd; ++i) {
      const double li = std::max(0.0, c.loads[i].p_kw);
      out.shed_by_load[i] = (res.x[i] > 0.5) ? li : 0.0;
    }
  }
  out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
  return out;
}

void fill_summary(const NativeCase& c, ThreeStageReliabilityResult& r) {
  r.nb_ac = static_cast<int>(c.sys.ac.buses.size());
  r.nb_dc = static_cast<int>(c.sys.dc.buses.size());
  r.nb = r.nb_ac + r.nb_dc;
  r.nl_ac = static_cast<int>(c.sys.ac.branches.size());
  r.nl_dc = static_cast<int>(c.sys.dc.branches.size());
  r.nl_vsc = static_cast<int>(c.sys.vsc_converters.size());
  r.nl_sop = r.nl_vsc;  // each VSC converter is treated as one SOP port
  r.sop_config.clear();
  for (const auto& vsc : c.sys.vsc_converters) {
    ThreeStageSopConfig sc;
    sc.id        = vsc.index;
    sc.node_a    = vsc.bus_ac;
    sc.node_b    = vsc.bus_dc;
    sc.pmax_kw   = vsc.pmax_mw * 1000.0;
    sc.qmax_kw   = vsc.qmax_mvar * 1000.0;
    sc.efficiency = (vsc.eta > 0.0 && vsc.eta <= 1.0) ? vsc.eta : 0.98;
    r.sop_config.push_back(sc);
  }
  r.nl = r.nl_ac + r.nl_dc + r.nl_vsc;
  r.nd = static_cast<int>(c.loads.size());
  r.ng = static_cast<int>(c.sources.size());
  r.nmg = static_cast<int>(c.sys.microgrids.size());
}

void run_native_case(const NativeCase& c, ThreeStageReliabilityResult& r) {
  fill_summary(c, r);
  r.nodal_eens_kwh_yr.assign(c.loads.size(), 0.0);
  r.nodal_cif.assign(c.loads.size(), 0.0);
  r.nodal_cid_min.assign(c.loads.size(), 0.0);
  r.faults.clear();

  double total_customers = 0.0;
  for (const auto& ld : c.loads) total_customers += std::max(1.0, ld.customers);
  total_customers = std::max(1.0, total_customers);

  double weighted_interruptions = 0.0;
  double weighted_duration_min = 0.0;
  double max_pls = -1.0;
  int worst = 0;

  for (const auto& fault : c.faults) {
    auto s1 = solve_stage_milp(c, fault, 1);
    auto s2 = solve_stage_milp(c, fault, 2);
    auto s3 = solve_stage_milp(c, fault, 3);

    ThreeStageFaultDetail d;
    d.line_id = fault.id;
    // Accept both "success" (MILP path) and "success (analytical)" (shortcut
    // path) as successful outcomes — they both mean zero or modelled load shed.
    auto is_ok = [](const std::string& s) { return s.rfind("success", 0) == 0; };
    d.status = (is_ok(s1.status) && is_ok(s2.status) && is_ok(s3.status)) ? "success" : "failed";
    d.pls_stage1 = s1.shed_kw;
    d.pls_stage2 = s2.shed_kw;
    d.pls_stage3 = s3.shed_kw;
    d.pls_total = d.pls_stage1 + d.pls_stage2 + d.pls_stage3;
    d.objective = d.pls_stage1 * kTauSwitchHr + d.pls_stage2 * kTauTrippingHr + d.pls_stage3 * kTauRepairHr;
    // SOP power vectors: zero-filled. Full SOP dispatch optimization is not
    // yet implemented; the binary load-shedding MILP above does not dispatch
    // SOP active-power flows. A dedicated OPF-based restoration model is
    // needed to co-optimize SOP setpoints with load pickup decisions.
    d.psop1.assign(r.nl_sop, 0.0);
    d.psop2.assign(r.nl_sop, 0.0);
    d.psop3.assign(r.nl_sop, 0.0);
    r.faults.push_back(d);

    if (d.pls_total > max_pls) {
      max_pls = d.pls_total;
      worst = fault.id;
    }

    for (size_t i = 0; i < c.loads.size(); ++i) {
      const double ens = fault.failure_rate *
          (s1.shed_by_load[i] * kTauSwitchHr +
           s2.shed_by_load[i] * kTauTrippingHr +
           s3.shed_by_load[i] * kTauRepairHr);
      r.nodal_eens_kwh_yr[i] += ens;
      r.eens_kwh_yr += ens;
      const bool interrupted = s1.shed_by_load[i] > 1e-6 || s2.shed_by_load[i] > 1e-6 || s3.shed_by_load[i] > 1e-6;
      if (interrupted) {
        const double cust = std::max(1.0, c.loads[i].customers);
        const double duration_min = ((s1.shed_by_load[i] > 1e-6 ? kTauSwitchHr : 0.0) +
                                     (s2.shed_by_load[i] > 1e-6 ? kTauTrippingHr : 0.0) +
                                     (s3.shed_by_load[i] > 1e-6 ? kTauRepairHr : 0.0)) * 60.0;
        r.nodal_cif[i] += fault.failure_rate;
        r.nodal_cid_min[i] += fault.failure_rate * duration_min;
        weighted_interruptions += fault.failure_rate * cust;
        weighted_duration_min += fault.failure_rate * duration_min * cust;
      }
    }
  }

  r.worst_line = worst > 0 ? worst : (r.nl_ac + r.nl_dc > 0 ? 1 : 0);
  r.saifi = weighted_interruptions / total_customers;
  r.saidi_min = weighted_duration_min / total_customers;
  r.eens_cost = r.eens_kwh_yr * kReliabilityVoll;
  r.ok = true;
}

}  // anonymous namespace

ThreeStageReliabilityResult run_three_stage_reliability(
    const fs::path& case_json,
    const ThreeStageReliabilityOptions& options) {
  (void)options;
  ThreeStageReliabilityResult result;
  if (!fs::exists(case_json)) {
    result.error = "case JSON does not exist: " + case_json.string();
    return result;
  }
  const std::string text = read_file_text(case_json);
  if (text.empty()) {
    result.error = "case JSON is empty or unreadable: " + case_json.string();
    return result;
  }
  try {
    HybridPowerSystem sys = io::from_json(text);
    NativeCase c = build_native_case(sys);
    if (c.loads.empty()) {
      result.error = "case JSON contains no load points";
      return result;
    }
    if (c.faults.empty()) {
      result.error = "case JSON contains no AC/DC branch contingencies";
      return result;
    }
    run_native_case(c, result);
  } catch (const std::exception& e) {
    result.error = std::string("failed to evaluate native three-stage reliability: ") + e.what();
  }
  return result;
}

ThreeStageReliabilityResult run_three_stage_reliability_from_string(
    const std::string& case_json_text,
    const ThreeStageReliabilityOptions& options) {
  (void)options;
  ThreeStageReliabilityResult result;
  if (case_json_text.empty()) {
    result.error = "case JSON text is empty";
    return result;
  }
  try {
    HybridPowerSystem sys = io::from_json(case_json_text);
    NativeCase c = build_native_case(sys);
    if (c.loads.empty()) {
      result.error = "case JSON contains no load points";
      return result;
    }
    if (c.faults.empty()) {
      result.error = "case JSON contains no AC/DC branch contingencies";
      return result;
    }
    run_native_case(c, result);
  } catch (const std::exception& e) {
    result.error = std::string("failed to evaluate native three-stage reliability: ") + e.what();
  }
  return result;
}

}  // namespace hacdcpf::analysis
