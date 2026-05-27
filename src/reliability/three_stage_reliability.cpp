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
#include <climits>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include "hacdcpf/engine/branch_and_cut.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/effective_capacity.hpp"

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
    const double cap = hacdcpf::model::effective_capacity_mw(g);
    if (cap > 0.0) add_source(c.sources, g.bus, cap);
  }
  for (const auto& sg : sys.ac.static_generators) {
    // Effective fixed-injection capacity must honour `scaling`; without it,
    // a unit scheduled out (scaling=0) would still appear at full nameplate
    // power in the connectivity model and over-state restoration headroom.
    const double cap = hacdcpf::model::effective_capacity_mw(sg);
    if (cap > 0.0) add_source(c.sources, sg.bus, cap);
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
    const double cap = hacdcpf::model::effective_capacity_mw(st);
    if (cap > 0.0) add_source(c.sources, st.bus, cap);
  }
  for (const auto& g : sys.dc.dc_static_generators) {
    const double cap = hacdcpf::model::effective_capacity_mw(g);
    if (cap > 0.0) add_source(c.sources, g.bus + kDCBusOffset, cap);
  }
  for (const auto& g : sys.dc.static_generators) {
    // Same scaling-aware treatment as the AC counterpart above.
    const double cap = hacdcpf::model::effective_capacity_mw(g);
    if (cap > 0.0) add_source(c.sources, g.bus + kDCBusOffset, cap);
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

// ─── LinDistFlow MILP — stages 1, 2, 3 ───────────────────────────────────────
//
// Implements the exact mathematical model:
//
//   Objective (eq. 3):   min Σ_i w_i · p^sh_i
//
//   C0  (eq. 3a):  Energisation/load-pickup coupling:
//                    P^d_i - p^sh_i ≤ P^d_i y_i, y_s = 1 for source buses
//
//   C1  (eq. 4'):  LinDistFlow active power balance at every AC bus i:
//                    Σ_{j:(j,i)∈L} P_ji - Σ_{j:(i,j)∈L} P_ij + p_g,i + p^sh_i = P^d_i
//                    0 ≤ p_g,i ≤ P^g,max_i
//
//   C2  (eq. 5):   Reactive power balance (q^sh_i = q_d,i · p^sh_i / p_d,i):
//                    Σ_{j:(j,i)∈L} Q_ji - Σ_{j:(i,j)∈L} Q_ij + q_g,i + q^sh_i = Q^d_i
//                    0 ≤ q_g,i ≤ Q^g,max_i
//
//   C3  (eq. 6'):  LinDistFlow voltage drop (linearised, losses dropped):
//                    v_j = v_i - 2(r_ij P_ij + x_ij Q_ij)
//                  Enforced via Big-M on z_ij:
//                    v_j - v_i + 2(r P + x Q) ≤  M(1 - z_ij)
//                    v_j - v_i + 2(r P + x Q) ≥ -M(1 - z_ij)
//
//   C4  (eq. 7–8): Branch active/reactive flow limits with Big-M:
//                    -z_ij · S̄_ij ≤ P_ij ≤ z_ij · S̄_ij
//                    -z_ij · S̄_ij ≤ Q_ij ≤ z_ij · S̄_ij
//
//   C5  (eq. 9):   Voltage bounds:   V̲² ≤ v_i ≤ V̄²
//
//   C6  (eq. 10–11): Strict energized radial forest:
//                    Σ f_in - Σ f_out = y_i, ∀i∉S
//                    Σ f_in - Σ f_out ≤ 0, ∀i∈S
//                    |f_ij| ≤ (|B|-1) z_ij, z_ij ≤ y_i, z_ij ≤ y_j
//                    Σ z_ij ≤ Σ y_i - |S|
//
//   C7  (eq. 12):  Forced-open failed branch:   z_k = 0
//
//   C8  (eq. 13):  Normally-closed non-switch branches stay closed:
//                    z_ij = 1  ∀(i,j) ∉ (L^NO ∪ {k})
//                  Stage 1: all switches remain at nominal position (no switching allowed).
//                  Stage 2: normally-open switches are free (0 ≤ z_ij ≤ 1, binary) subject to C9.
//                  Stage 3: fault branch is restored; all z_ij fixed at nominal (all n.c. = 1).
//
//   C9  (eq. 14):  Switch count:   Σ_{(i,j)∈L^NO} z_ij ≤ K^sw
//
//   C10 (eq. 15):  Load shed bounds:   0 ≤ p^sh_i ≤ p_d,i
//
// MODEL SCOPE: AC branches and buses only.  DC loads are handled by the
// connectivity fallback (capacity ≥ demand per component).  VSC/DCDC elements
// are graph edges in that fallback, not dispatch variables in this MILP.
// ─────────────────────────────────────────────────────────────────────────────

// Big-M constant for voltage-drop linearisation (in pu²).
// The voltage span is at most [V̲², V̄²] ≈ [0.81, 1.21]; 2r·P + 2x·Q is
// bounded in magnitude by 2·(r+x)·S̄.  A value of 2.0 pu² is conservative
// enough for any realistic distribution system.
constexpr double kBigMVoltage = 2.0;

struct StageSolve {
  double shed_kw{0.0};
  std::vector<double> shed_by_load;
  std::string status{"unknown"};
  double objective{0.0};
  double mip_gap{0.0};
  bool proven_optimal{true};
};

double vsc_transfer_capacity_kw(const VSCConverter& vsc) {
  const double capacity_mw = std::max({std::abs(vsc.pmax_mw),
                                       std::abs(vsc.pmin_mw),
                                       std::abs(vsc.p_set_mw),
                                       vsc.p_rated_mw});
  if (!std::isfinite(capacity_mw) || capacity_mw <= 0.0) return 0.0;
  const double eta = (vsc.eta > 0.0 && vsc.eta <= 1.0) ? vsc.eta : 1.0;
  return capacity_mw * eta * 1000.0;
}

// solve_stage_milp() — LinDistFlow MILP for stages 1, 2, and 3.
//
// The MILP is built over the AC sub-network only.  DC buses/branches/VSC are
// excluded from the LinDistFlow equations (they have no r_pu / x_pu / voltage
// bounds in the DC model).  DC loads that map to the same AC bus as a VSC
// converter are implicitly covered by the AC power balance.  Purely DC-only
// loads on buses with no AC equivalent are handled by the connectivity/capacity
// fallback at the end of this function.
StageSolve solve_stage_milp(const NativeCase& c, const FaultLine& fault, int stage,
                            int max_sw_ops = INT_MAX) {
  StageSolve out;
  const int nd_total = static_cast<int>(c.loads.size());
  out.shed_by_load.assign(c.loads.size(), 0.0);
  if (nd_total == 0) {
    out.status = "success";
    return out;
  }

  // ── Build index maps for the AC sub-network ──────────────────────────────
  const HybridPowerSystem& sys = c.sys;

  // AC buses: map bus_id → position in ac_buses vector (0-based)
  const int n_bus = static_cast<int>(sys.ac.buses.size());
  std::unordered_map<int, int> ac_bus_pos;
  ac_bus_pos.reserve(static_cast<size_t>(n_bus));
  for (int i = 0; i < n_bus; ++i) ac_bus_pos[sys.ac.buses[i].index] = i;

  // AC candidate edges: all physical branches plus standalone switch elements.
  // Closed/in-service edges are nominally closed.  Out-of-service branches and
  // open switches are normally-open candidates that may close only in Stage 2.
  struct BrInfo {
    int idx_global;   // index in sys.ac.branches
    int from_pos;     // position in ac_buses
    int to_pos;
    double r_pu;
    double x_pu;
    double s_max_mw;  // thermal limit in MW (= rate_a_mva, or default)
    bool normally_open;  // true if this is a normally-open switch
    bool failed;         // true if this is the faulted branch
  };
  // Default branch rating: 2× total system demand (ensures feasibility when
  // no explicit rating is given, without making Big-M constraints too loose).
  double total_demand_mw = 0.0;
  for (const auto& ld : c.loads) total_demand_mw += std::max(0.0, ld.p_kw) / 1000.0;
  const double default_rate_mw = std::max(10.0, 2.0 * total_demand_mw);

  std::vector<BrInfo> ac_branches;
  ac_branches.reserve(sys.ac.branches.size() + sys.ac.switches.size());
  auto undirected_key = [](int a, int b) -> long long {
    if (a > b) std::swap(a, b);
    return (static_cast<long long>(a) << 32) ^ static_cast<unsigned int>(b);
  };
  std::unordered_map<long long, bool> branch_pair_seen;
  for (int b = 0; b < static_cast<int>(sys.ac.branches.size()); ++b) {
    const auto& br = sys.ac.branches[b];
    auto it_f = ac_bus_pos.find(br.from_bus);
    auto it_t = ac_bus_pos.find(br.to_bus);
    if (it_f == ac_bus_pos.end() || it_t == ac_bus_pos.end()) continue;
    branch_pair_seen[undirected_key(br.from_bus, br.to_bus)] = true;

    bool has_open_switch = false;
    for (const auto& sw : sys.ac.switches) {
      if (!sw.in_service) continue;
      if (!sw.closed &&
          ((sw.bus_from == br.from_bus && sw.bus_to == br.to_bus) ||
           (sw.bus_from == br.to_bus   && sw.bus_to == br.from_bus))) {
        has_open_switch = true;
        break;
      }
    }
    const bool is_no_switch = !br.in_service || has_open_switch;
    const bool is_failed = (fault.ac && b == fault.index);
    const double s_max = br.rate_a_mva > 1e-9 ? br.rate_a_mva : default_rate_mw;
    ac_branches.push_back({b, it_f->second, it_t->second,
                           std::max(1e-6, br.r_pu),
                           std::max(1e-6, br.x_pu),
                           s_max,
                           is_no_switch,
                           is_failed});
  }
  for (const auto& sw : sys.ac.switches) {
    if (!sw.in_service) continue;
    if (branch_pair_seen.count(undirected_key(sw.bus_from, sw.bus_to))) continue;
    auto it_f = ac_bus_pos.find(sw.bus_from);
    auto it_t = ac_bus_pos.find(sw.bus_to);
    if (it_f == ac_bus_pos.end() || it_t == ac_bus_pos.end()) continue;
    ac_branches.push_back({-1, it_f->second, it_t->second,
                           1e-6, 1e-6, default_rate_mw,
                           !sw.closed,
                           false});
  }
  const int n_br = static_cast<int>(ac_branches.size());

  // ── Identify source buses (S set in radiality constraint C6) ────────────
  // A bus is a source if it has an external grid, generator, static generator,
  // storage, or renewable gen attached with positive capacity.
  std::vector<bool> is_source(static_cast<size_t>(n_bus), false);
  // p_gen[i]: maximum active injection in MW at AC bus i (sum over all generators)
  std::vector<double> p_gen_max(static_cast<size_t>(n_bus), 0.0);
  // q_gen[i]: maximum reactive injection in Mvar (used in reactive balance C2)
  std::vector<double> q_gen_max(static_cast<size_t>(n_bus), 0.0);

  auto mark_source = [&](int bus, double p_mw) {
    auto it = ac_bus_pos.find(bus);
    if (it == ac_bus_pos.end()) return;
    is_source[it->second] = true;
    p_gen_max[it->second] += std::max(0.0, p_mw);
  };
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    const double cap = eg.s_sc_max_mva > 0.0 ? eg.s_sc_max_mva : 1.0e4;
    mark_source(eg.bus, cap);
    auto it = ac_bus_pos.find(eg.bus);
    if (it != ac_bus_pos.end()) q_gen_max[it->second] += cap;
  }
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    mark_source(g.bus, g.pmax_mw > 0.0 ? g.pmax_mw : g.pg_mw);
    auto it = ac_bus_pos.find(g.bus);
    if (it != ac_bus_pos.end())
      q_gen_max[it->second] += std::max(0.0, g.qmax_mvar > 0.0 ? g.qmax_mvar : std::abs(g.qg_mvar));
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    const double cap = hacdcpf::model::effective_capacity_mw(sg);
    mark_source(sg.bus, cap);
    auto it = ac_bus_pos.find(sg.bus);
    if (it != ac_bus_pos.end()) q_gen_max[it->second] += cap * 0.5;
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    const double cap = rg.p_rated_mw > 0.0 ? rg.p_rated_mw * rg.capacity_factor : rg.p_mw;
    mark_source(rg.bus, cap);
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    mark_source(pv.bus, pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw);
  }
  for (const auto& st : sys.ac.storage) {
    const double cap = hacdcpf::model::effective_capacity_mw(st);
    if (cap > 1e-9) mark_source(st.bus, cap);
  }

  // ── Build per-bus demand vectors (MW and Mvar) for AC loads ────────────
  // p_d[i]: net active demand at AC bus i (MW)
  // q_d[i]: net reactive demand (Mvar); approximated as p_d * tan(arccos(0.9))
  // Demand is sourced only from c.loads. build_native_case() already expands
  // both ACLoad entries and ACBus::pd_mw into explicit LoadPoint records, so
  // adding sys.ac.buses[i].pd_mw here would double-count bus-level demand.
  std::vector<double> p_d(static_cast<size_t>(n_bus), 0.0);
  std::vector<double> q_d(static_cast<size_t>(n_bus), 0.0);
  // Map from load-point index (in c.loads) → AC bus position (-1 if DC-only)
  std::vector<int> load_ac_bus(static_cast<size_t>(nd_total), -1);

  for (int li = 0; li < nd_total; ++li) {
    const int bus = c.loads[li].bus;
    auto it = ac_bus_pos.find(bus);
    if (it == ac_bus_pos.end()) continue;  // DC-only load — handled below
    load_ac_bus[li] = it->second;
    const double p_mw = std::max(0.0, c.loads[li].p_kw) / 1000.0;
    p_d[it->second] += p_mw;
    q_d[it->second] += p_mw * 0.4843;  // tan(arccos(0.9)) ≈ 0.4843
  }
  // Number of AC load points (those with AC bus)
  // (DC-only loads get connectivity fallback below)

  // ── Variable index helpers ────────────────────────────────────────────────
  // Layout: [p^sh_i] [y_i] [z_ij] [P_ij] [Q_ij] [v_i] [f_ij] [p_g_i] [q_g_i]
  const int off_shed  = 0;
  const int off_y      = off_shed + n_bus;
  const int off_z      = off_y     + n_bus;
  const int off_P     = off_z    + n_br;
  const int off_Q     = off_P   + n_br;
  const int off_v     = off_Q   + n_br;
  const int off_f     = off_v   + n_bus;
  const int off_pg    = off_f   + n_br;
  const int off_qg    = off_pg  + n_bus;
  const int n_vars    = off_qg  + n_bus;

  engine::MIPModel mip;
  auto& lp = mip.linear_part;
  lp.sense = engine::Sense::Minimize;
  lp.vars.resize(static_cast<size_t>(n_vars));
  lp.c = Eigen::VectorXd::Zero(n_vars);

  // p^sh_i — continuous load shed (MW) at AC bus i
  // Objective: min Σ_i p^sh_i  (weight w_i = 1 per MW shed, eq. 3)
  for (int i = 0; i < n_bus; ++i) {
    lp.vars[off_shed + i] = {engine::VarType::Continuous,
                              0.0, p_d[i],
                              "psh_" + std::to_string(sys.ac.buses[i].index)};
    lp.c[off_shed + i] = 1.0;  // minimise total MW shed (eq. 3)
  }

  const int n_sources = static_cast<int>(std::count(is_source.begin(), is_source.end(), true));
  int n_source_components = 0;
  if (n_sources > 0) {
    DSU source_dsu(n_bus);
    for (int b = 0; b < n_br; ++b) {
      const auto& br = ac_branches[b];
      if (br.normally_open) continue;
      if (br.failed && stage < 3) continue;
      source_dsu.unite(br.from_pos, br.to_pos);
    }
    std::unordered_set<int> source_components;
    for (int i = 0; i < n_bus; ++i) {
      if (is_source[i]) source_components.insert(source_dsu.find(i));
    }
    n_source_components = std::max(1, static_cast<int>(source_components.size()));
  }

  // y_i — energized/served bus indicator.  Source buses are energized roots.
  // If no source exists, all y_i are fixed to zero and the model sheds all load.
  for (int i = 0; i < n_bus; ++i) {
    double y_lb = 0.0, y_ub = 1.0;
    engine::VarType y_type = engine::VarType::Binary;
    if (is_source[i]) {
      y_lb = 1.0; y_ub = 1.0; y_type = engine::VarType::Continuous;
    } else if (n_sources == 0) {
      y_lb = 0.0; y_ub = 0.0; y_type = engine::VarType::Continuous;
    }
    lp.vars[off_y + i] = {y_type, y_lb, y_ub,
                          "y_" + std::to_string(sys.ac.buses[i].index)};
    if (y_type == engine::VarType::Binary) mip.binary_idx.push_back(off_y + i);
  }

  // z_ij — binary branch status (1 = closed, 0 = open)
  const int n_no_switch = [&]() {
    int cnt = 0;
    for (const auto& br : ac_branches) if (br.normally_open) ++cnt;
    return cnt;
  }();
  // Big-M commodity capacity = |B| − 1 (number of buses minus 1) so that the
  // single-commodity flow can route one unit to every non-source bus (C6).
  const double commodity_cap = std::max(1.0, static_cast<double>(n_bus - 1));

  for (int b = 0; b < n_br; ++b) {
    const auto& br = ac_branches[b];
    double z_lb = 0.0, z_ub = 1.0;

    // C7: Forced-open failed branch (eq. 12)
    if (br.failed && stage < 3) {
      z_lb = 0.0; z_ub = 0.0;
    }
    // C8: z denotes energized branch use, not the mechanical switch handle.
    // Healthy normally-closed edges may de-energize when their endpoint bus is
    // shed; the radial/commodity constraints choose the energized forest.
    // Normally-open ties stay open in Stage 1/3 and are candidate switches in Stage 2.
    else if (!br.normally_open) {
      z_lb = 0.0; z_ub = 1.0;
    } else if (stage == 1 || stage == 3) {
      // Stage 1: no switching — normally-open switches stay open
      // Stage 3: repair restores everything to nominal (normally-open stays open)
      z_lb = 0.0; z_ub = 0.0;
    }
    // Stage 2: normally-open switch — free binary variable, subject to C9 (count constraint)

    // Fixed z (lb==ub) → Continuous: pure-LP path, avoids B&C postsolve issues.
    // Free z (lb < ub) → Binary: only for truly free NO switches in stage 2.
    const engine::VarType z_type = (z_lb < z_ub) ? engine::VarType::Binary
                                                   : engine::VarType::Continuous;
    lp.vars[off_z + b] = {z_type, z_lb, z_ub, "z_" + std::to_string(b)};
    if (z_lb < z_ub) mip.binary_idx.push_back(off_z + b);
  }

  // p_g_i / q_g_i — finite source dispatch at every AC bus.
  for (int i = 0; i < n_bus; ++i) {
    lp.vars[off_pg + i] = {engine::VarType::Continuous, 0.0, p_gen_max[i],
                           "pg_" + std::to_string(sys.ac.buses[i].index)};
    lp.vars[off_qg + i] = {engine::VarType::Continuous, 0.0, q_gen_max[i],
                           "qg_" + std::to_string(sys.ac.buses[i].index)};
  }

  // P_ij — active power flow (MW, signed: positive = from_bus → to_bus)
  for (int b = 0; b < n_br; ++b) {
    const double s_max = ac_branches[b].s_max_mw;
    lp.vars[off_P + b] = {engine::VarType::Continuous, -s_max, s_max,
                           "P_" + std::to_string(b)};
  }
  // Q_ij — reactive power flow (Mvar, signed)
  for (int b = 0; b < n_br; ++b) {
    const double s_max = ac_branches[b].s_max_mw;
    lp.vars[off_Q + b] = {engine::VarType::Continuous, -s_max, s_max,
                           "Q_" + std::to_string(b)};
  }
  // v_i — squared voltage magnitude (pu²) at AC bus i
  // Source (slack) buses are fixed at 1.0 pu² (nominal substation voltage).
  // Load buses are bounded by [vmin², vmax²] per their bus data.
  for (int i = 0; i < n_bus; ++i) {
    double vmin2, vmax2;
    if (is_source[i]) {
      vmin2 = 1.0; vmax2 = 1.0;  // source bus: fixed at nominal 1.0 pu²
    } else {
      const double vmin = sys.ac.buses[i].vmin_pu;
      const double vmax = sys.ac.buses[i].vmax_pu;
      vmin2 = vmin * vmin;
      vmax2 = vmax * vmax;
    }
    lp.vars[off_v + i] = {engine::VarType::Continuous, vmin2, vmax2,
                           "v_" + std::to_string(sys.ac.buses[i].index)};
  }
  // f_ij — single-commodity flow auxiliary for radiality (C6)
  // Signed: f_ij < 0 means commodity flows in reverse (to_bus → from_bus),
  // which is needed when the source bus is at the to_bus end of a branch.
  for (int b = 0; b < n_br; ++b) {
    lp.vars[off_f + b] = {engine::VarType::Continuous, -commodity_cap, commodity_cap,
                           "f_" + std::to_string(b)};
  }

  // ── Build constraint triplets (equality and inequality) ───────────────────
  std::vector<Eigen::Triplet<double>> eq_trips, ineq_trips;
  std::vector<double> beq_vals, b_vals;

  auto add_eq = [&](const std::vector<std::pair<int,double>>& terms, double rhs) {
    const int row = static_cast<int>(beq_vals.size());
    for (const auto& [col, val] : terms)
      if (std::abs(val) > 1e-12) eq_trips.emplace_back(row, col, val);
    beq_vals.push_back(rhs);
  };
  auto add_le = [&](const std::vector<std::pair<int,double>>& terms, double rhs) {
    const int row = static_cast<int>(b_vals.size());
    for (const auto& [col, val] : terms)
      if (std::abs(val) > 1e-12) ineq_trips.emplace_back(row, col, val);
    b_vals.push_back(rhs);
  };

  // ── C0 (eq. 3a): energized/load-pickup coupling ─────────────────────────
  // P^d_i - p^sh_i ≤ P^d_i y_i.  If y_i=0 the bus must shed all active load.
  for (int i = 0; i < n_bus; ++i) {
    if (p_d[i] <= 1e-9) continue;
    add_le({{off_shed + i, -1.0}, {off_y + i, -p_d[i]}}, -p_d[i]);
  }

  // ── C1 (eq. 4'): LinDistFlow active power balance at every AC bus i ──────
  // Σ_{j:(j,i)} P_ji - Σ_{j:(i,j)} P_ij + p_g,i + p^sh_i = P^d_i
  // with 0 ≤ p_g,i ≤ P^g,max_i.  Source buses are no longer skipped: their
  // injection is finite and appears explicitly through p_g,i.
  for (int i = 0; i < n_bus; ++i) {
    std::vector<std::pair<int,double>> terms = {{off_shed + i, 1.0},
                                                {off_pg + i, 1.0}};
    // Flow: +P_ji for branches where to_pos = i; -P_ij for branches where from_pos = i
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].to_pos   == i) terms.push_back({off_P + b,  1.0});
      if (ac_branches[b].from_pos == i) terms.push_back({off_P + b, -1.0});
    }
    add_eq(terms, p_d[i]);
  }

  // ── C2 (eq. 5): Reactive power balance at every AC bus i ────────────────
  // Σ Q_ji - Σ Q_ij + q_g,i + (q_d,i/p_d,i) · p^sh_i = Q^d_i
  for (int i = 0; i < n_bus; ++i) {
    std::vector<std::pair<int,double>> terms = {{off_qg + i, 1.0}};
    // Reactive shed proportional to active shed
    const double ratio = (p_d[i] > 1e-9) ? (q_d[i] / p_d[i]) : 0.0;
    if (std::abs(ratio) > 1e-12) terms.push_back({off_shed + i, ratio});
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].to_pos   == i) terms.push_back({off_Q + b,  1.0});
      if (ac_branches[b].from_pos == i) terms.push_back({off_Q + b, -1.0});
    }
    add_eq(terms, q_d[i]);
  }

  // ── C3 (eq. 6'): LinDistFlow voltage drop with Big-M on z_ij ────────────
  // When z_ij = 1 (closed):  v_j = v_i - 2(r_pu/S_base · P_MW + x_pu/S_base · Q_MW)
  // When z_ij = 0 (open):    v_j and v_i are decoupled (Big-M relaxation).
  //
  // Dimensional note: v is in pu², r/x are in pu, P/Q are in MW.
  //   Per-unit equation: v_j = v_i - 2(r_pu · P_pu + x_pu · Q_pu)
  //   Substituting P_pu = P_MW / S_base:  coefficient = 2 · r_pu / S_base
  //
  // Big-M formulation (with correct +M·z coefficient):
  //   |v_j - v_i + 2(r/S·P + x/S·Q)| ≤ M·(1 - z_ij)
  // As two ≤ constraints:
  //   v_j - v_i + 2r/S·P + 2x/S·Q + M·z ≤ M
  //  -v_j + v_i - 2r/S·P - 2x/S·Q + M·z ≤ M
  const double base_mva = std::max(sys.ac.base_mva, 1.0);
  for (int b = 0; b < n_br; ++b) {
    const auto& br = ac_branches[b];
    // Scale r/x by 2/S_base for dimensional consistency (MW → pu)
    const double rc = 2.0 * br.r_pu / base_mva;
    const double xc = 2.0 * br.x_pu / base_mva;
    // Constraint 1: v_j - v_i + rc·P + xc·Q + M·z ≤ M
    add_le({{ off_v + br.to_pos,    1.0},
            { off_v + br.from_pos, -1.0},
            { off_P + b,  rc},
            { off_Q + b,  xc},
            { off_z + b,  kBigMVoltage}},
           kBigMVoltage);
    // Constraint 2: -v_j + v_i - rc·P - xc·Q + M·z ≤ M
    add_le({{ off_v + br.to_pos,   -1.0},
            { off_v + br.from_pos,  1.0},
            { off_P + b, -rc},
            { off_Q + b, -xc},
            { off_z + b,  kBigMVoltage}},
           kBigMVoltage);
  }

  // ── C4 (eq. 7–8): Branch flow limits with Big-M ──────────────────────────
  // -z_ij · S̄ ≤ P_ij ≤ z_ij · S̄
  // P_ij - z_ij · S̄ ≤ 0    →   P_ij - S̄ · z_ij ≤ 0
  // -P_ij - z_ij · S̄ ≤ 0   →  -P_ij - S̄ · z_ij ≤ 0
  for (int b = 0; b < n_br; ++b) {
    const double s = ac_branches[b].s_max_mw;
    add_le({{ off_P + b,  1.0}, { off_z + b, -s}}, 0.0);  // P ≤ z·S̄
    add_le({{ off_P + b, -1.0}, { off_z + b, -s}}, 0.0);  // -P ≤ z·S̄
    add_le({{ off_Q + b,  1.0}, { off_z + b, -s}}, 0.0);  // Q ≤ z·S̄
    add_le({{ off_Q + b, -1.0}, { off_z + b, -s}}, 0.0);  // -Q ≤ z·S̄
  }

  // ── C6 (eq. 10–11): strict energized radial forest ──────────────────────
  // Energized non-source buses consume one commodity unit; source buses inject.
  // z≤y endpoint constraints forbid closed branches touching de-energized buses.
  // The edge-count inequality eliminates redundant energized cycles.
  for (int b = 0; b < n_br; ++b) {
    // |f_ij| ≤ (|B|-1) · z_ij  (bidirectional flow bound)
    add_le({{ off_f + b,  1.0}, { off_z + b, -commodity_cap}}, 0.0);  // f ≤ cap·z
    add_le({{ off_f + b, -1.0}, { off_z + b, -commodity_cap}}, 0.0);  // -f ≤ cap·z
    add_le({{ off_z + b,  1.0}, { off_y + ac_branches[b].from_pos, -1.0}}, 0.0);
    add_le({{ off_z + b,  1.0}, { off_y + ac_branches[b].to_pos,   -1.0}}, 0.0);
    if (!ac_branches[b].normally_open && !(ac_branches[b].failed && stage < 3)) {
      add_le({{ off_y + ac_branches[b].from_pos, 1.0},
              { off_y + ac_branches[b].to_pos,   1.0},
              { off_z + b,                      -1.0}},
             1.0);
    }
  }
  for (int i = 0; i < n_bus; ++i) {
    std::vector<std::pair<int,double>> terms;
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].to_pos   == i) terms.push_back({off_f + b,  1.0});  // f_ji (inflow)
      if (ac_branches[b].from_pos == i) terms.push_back({off_f + b, -1.0});  // f_ij (outflow)
    }
    if (is_source[i]) {
      // Source bus: net outflow ≥ 0  →  -(net outflow) ≤ 0  →  Σ inflow - Σ outflow ≤ 0
      add_le(terms, 0.0);
    } else {
      terms.push_back({off_y + i, -1.0});
      add_eq(terms, 0.0);
    }
  }
  if (n_sources > 0) {
    std::vector<std::pair<int,double>> forest_terms;
    for (int b = 0; b < n_br; ++b) forest_terms.push_back({off_z + b, 1.0});
    for (int i = 0; i < n_bus; ++i) forest_terms.push_back({off_y + i, -1.0});
    add_le(forest_terms, -static_cast<double>(n_source_components));
  }

  // ── C9 (eq. 14): Switch count ≤ K^sw ────────────────────────────────────
  // Σ_{(i,j)∈L^NO} z_ij ≤ K^sw   (Stage 2 only; K^sw = max_sw_ops)
  // In stage 2, normally-open switches have free z_ij ∈ {0,1}.
  if (stage == 2 && n_no_switch > 0) {
    std::vector<std::pair<int,double>> sw_terms;
    for (int b = 0; b < n_br; ++b) {
      if (ac_branches[b].normally_open) sw_terms.push_back({off_z + b, 1.0});
    }
    if (!sw_terms.empty()) {
      const double ksw = (max_sw_ops == INT_MAX)
                           ? static_cast<double>(n_no_switch)  // no limit
                           : static_cast<double>(max_sw_ops);
      add_le(sw_terms, ksw);
    }
  }

  // C5 (eq. 9): Voltage bounds enforced via variable bounds [vmin², vmax²]
  // (already set in lp.vars above — no additional constraint row needed)

  // ── Finalise constraint matrices ─────────────────────────────────────────
  const int n_eq   = static_cast<int>(beq_vals.size());
  const int n_ineq = static_cast<int>(b_vals.size());

  lp.Aeq.resize(n_eq, n_vars);
  lp.beq.resize(n_eq);
  lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  lp.Aeq.makeCompressed();
  for (int r = 0; r < n_eq; ++r) lp.beq[r] = beq_vals[r];

  lp.A.resize(n_ineq, n_vars);
  lp.b.resize(n_ineq);
  lp.A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  lp.A.makeCompressed();
  for (int r = 0; r < n_ineq; ++r) lp.b[r] = b_vals[r];

  // ── Solve ─────────────────────────────────────────────────────────────────
  // When all z variables are fixed (no free binary/integer decisions), bypass
  // the B&C to avoid presolve/postsolve size-mismatch bugs on LP-only problems.
  // Only use B&C when there are genuinely free binary/integer variables (stage 2
  // with normally-open switches).
  Eigen::VectorXd res_x;
  bool res_success = false;
  std::string res_status;
  double res_objective = 0.0;
  double res_mip_gap = 0.0;
  constexpr double kGapTol = 1e-6;

  if (mip.binary_idx.empty() && mip.integer_idx.empty()) {
    // Pure LP — use the native dual simplex directly.
    engine::SimplexOptions simp_opt;
    simp_opt.max_iter        = 10000;
    simp_opt.feasibility_tol = 1e-8;
    simp_opt.optimality_tol  = 1e-8;
    simp_opt.verbose         = false;
    auto sr = engine::solve_lp_with_basis(lp, simp_opt, nullptr);
    res_x        = sr.result.x;
    res_success  = sr.result.stats.success;
    res_status   = sr.result.stats.status;
    res_objective= sr.result.stats.objective;
    res_mip_gap  = 0.0;  // pure LP has no integrality gap
  } else {
    engine::HighsAdapter highs;
    if (highs.available()) {
      auto highs_res = highs.solve_milp(mip);
      res_x        = highs_res.x;
      res_success  = highs_res.stats.success;
      res_status   = highs_res.stats.status;
      res_objective= highs_res.stats.objective;
      res_mip_gap  = highs_res.stats.mip_gap;
    } else {
      engine::BCOptions opt;
      opt.max_nodes        = 2048;
      opt.time_limit_sec   = 30.0;
      opt.gap_tol          = kGapTol;
      opt.verbose          = false;
      opt.use_simplex_lp_nodes = true;
      auto bc_res  = engine::solve_milp_bc(mip, opt);
      res_x        = bc_res.x;
      res_success  = bc_res.stats.success;
      res_status   = bc_res.stats.status;
      res_objective= bc_res.stats.objective;
      res_mip_gap  = bc_res.stats.mip_gap;
    }
  }

  if (!res_success || res_x.size() != static_cast<size_t>(n_vars)) {
    out.status = res_status.empty() ? "failed" : res_status;
    // Conservative fallback: shed all AC loads
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] >= 0) out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
    }
    out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
    return out;
  }

  const bool proven_optimal = (res_mip_gap <= kGapTol + 1e-9);
  out.status = proven_optimal ? "success" : "success (approximate)";
  out.mip_gap = res_mip_gap;
  out.proven_optimal = proven_optimal;
  out.objective = res_objective;

  const double bound_tol = 1e-6;
  const double integer_tol = 1e-4;
  auto fail_postsolve = [&]() {
    out.status = "failed (constraint violation)";
    out.proven_optimal = false;
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] >= 0) out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
    }
    out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
  };
  for (int var = 0; var < n_vars; ++var) {
    const double value = res_x[var];
    const auto& bounds = lp.vars[static_cast<size_t>(var)];
    if (value < bounds.lb - bound_tol || value > bounds.ub + bound_tol) {
      spdlog::warn("[三阶段可靠性] LinDistFlow MILP bound violation: var[{}]={:.8f} bounds=[{:.8f},{:.8f}]",
                   var, value, bounds.lb, bounds.ub);
      fail_postsolve();
      return out;
    }
  }
  auto check_integer = [&](int var) -> bool {
    if (var < 0 || var >= n_vars) return true;
    const double value = res_x[var];
    if (std::abs(value - std::round(value)) <= integer_tol) return true;
    spdlog::warn("[三阶段可靠性] LinDistFlow MILP integrality violation: var[{}]={:.8f}",
                 var, value);
    return false;
  };
  for (int var : mip.binary_idx) {
    if (!check_integer(var)) { fail_postsolve(); return out; }
  }
  for (int var : mip.integer_idx) {
    if (!check_integer(var)) { fail_postsolve(); return out; }
  }
  const double recomputed_objective = lp.c.dot(res_x);
  const double objective_tol = 1e-5 * std::max(1.0, std::abs(recomputed_objective));
  if (std::isfinite(res_objective) &&
      std::abs(res_objective - recomputed_objective) > objective_tol) {
    spdlog::warn("[三阶段可靠性] LinDistFlow MILP objective mismatch: solver={:.8f} recomputed={:.8f}",
                 res_objective, recomputed_objective);
    fail_postsolve();
    return out;
  }
  out.objective = recomputed_objective;

  // Post-solve: verify equality and inequality residuals
  if (n_eq > 0) {
    const Eigen::VectorXd eq_res = (lp.Aeq * res_x - lp.beq).cwiseAbs();
    if (eq_res.maxCoeff() > 1e-4) {
      spdlog::warn("[三阶段可靠性] LinDistFlow MILP equality violation={:.2e} — fallback",
                   eq_res.maxCoeff());
      out.status = "failed (constraint violation)";
      for (int li = 0; li < nd_total; ++li)
        if (load_ac_bus[li] >= 0) out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
      out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
      return out;
    }
  }
  if (n_ineq > 0) {
    const double ineq_viol = (lp.A * res_x - lp.b).cwiseMax(0.0).maxCoeff();
    if (ineq_viol > 1e-4) {
      spdlog::warn("[三阶段可靠性] LinDistFlow MILP inequality violation={:.2e} — fallback",
                   ineq_viol);
      out.status = "failed (constraint violation)";
      for (int li = 0; li < nd_total; ++li)
        if (load_ac_bus[li] >= 0) out.shed_by_load[li] = std::max(0.0, c.loads[li].p_kw);
      out.shed_kw = std::accumulate(out.shed_by_load.begin(), out.shed_by_load.end(), 0.0);
      return out;
    }
  }

  // ── Extract p^sh_i (per AC bus) → map back to load points ───────────────
  // p^sh_i is defined per AC bus.  Each load point (c.loads[li]) that maps
  // to AC bus i gets a shed proportional to its share of p_d[i].
  for (int i = 0; i < n_bus; ++i) {
    const double psh_bus_mw = std::max(0.0, std::min(res_x[off_shed + i], p_d[i]));
    if (p_d[i] < 1e-9) continue;
    // Distribute shed proportionally among load points at this bus
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] != i) continue;
      const double ld_mw = std::max(0.0, c.loads[li].p_kw) / 1000.0;
      const double share = ld_mw / p_d[i];
      out.shed_by_load[li] = psh_bus_mw * share * 1000.0;  // back to kW
    }
  }

  // ── DC-only loads: connectivity/capacity fallback ─────────────────────────
  // Loads whose bus is not in ac_bus_pos (i.e., DC buses not connected to any
  // AC bus in this model) are handled by the original capacity-balance approach.
  {
    // Build DSU over DC buses for the current stage
    auto dc_pos = bus_position_map(c.buses);
    DSU dsu_dc(static_cast<int>(c.buses.size()));
    auto connect_dc = [&](int a, int b_bus) {
      auto ia = dc_pos.find(a), ib = dc_pos.find(b_bus);
      if (ia != dc_pos.end() && ib != dc_pos.end()) dsu_dc.unite(ia->second, ib->second);
    };
    for (int b = 0; b < static_cast<int>(sys.ac.branches.size()); ++b) {
      const auto& br = sys.ac.branches[b];
      bool on = br.in_service;
      if (fault.ac && b == fault.index && stage < 3) on = false;
      if (stage >= 3 && br.in_service) on = true;
      if (on) connect_dc(br.from_bus, br.to_bus);
    }
    for (int b = 0; b < static_cast<int>(sys.dc.branches.size()); ++b) {
      const auto& br = sys.dc.branches[b];
      bool on = br.in_service;
      if (!fault.ac && b == fault.index && stage < 3) on = false;
      if (stage >= 3 && br.in_service) on = true;
      if (on) connect_dc(br.from_bus + kDCBusOffset, br.to_bus + kDCBusOffset);
    }
    for (const auto& vsc : sys.vsc_converters) {
      if (vsc.in_service) connect_dc(vsc.bus_ac, vsc.bus_dc + kDCBusOffset);
    }
    for (const auto& dc : sys.dc.dcdc_converters) {
      if (dc.in_service) connect_dc(dc.bus_in + kDCBusOffset, dc.bus_out + kDCBusOffset);
    }

    std::unordered_map<int, int> dc_comp;
    for (int i = 0; i < static_cast<int>(c.buses.size()); ++i)
      dc_comp[c.buses[i]] = dsu_dc.find(i);

    std::unordered_map<int, double> comp_ac_source_kw;
    std::unordered_map<int, double> comp_dc_source_kw;
    for (const auto& s : c.sources) {
      auto it = dc_comp.find(s.bus);
      if (it == dc_comp.end()) continue;
      if (s.bus >= kDCBusOffset) comp_dc_source_kw[it->second] += s.p_kw;
      else comp_ac_source_kw[it->second] += s.p_kw;
    }
    std::unordered_map<int, double> comp_ac_served_kw;
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] < 0) continue;
      auto it = dc_comp.find(c.loads[li].bus);
      if (it == dc_comp.end()) continue;
      comp_ac_served_kw[it->second] +=
          std::max(0.0, c.loads[li].p_kw - out.shed_by_load[li]);
    }
    std::unordered_map<int, double> comp_vsc_transfer_kw;
    for (const auto& vsc : sys.vsc_converters) {
      if (!vsc.in_service) continue;
      auto ac_it = dc_comp.find(vsc.bus_ac);
      auto dc_it = dc_comp.find(vsc.bus_dc + kDCBusOffset);
      if (ac_it == dc_comp.end() || dc_it == dc_comp.end()) continue;
      if (ac_it->second == dc_it->second) {
        comp_vsc_transfer_kw[ac_it->second] += vsc_transfer_capacity_kw(vsc);
      }
    }
    // Proper per-component capacity check for DC loads
    std::unordered_map<int, double> dc_comp_demand;
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] >= 0) continue;
      auto it = dc_comp.find(c.loads[li].bus);
      if (it != dc_comp.end()) dc_comp_demand[it->second] += std::max(0.0, c.loads[li].p_kw);
    }
    for (int li = 0; li < nd_total; ++li) {
      if (load_ac_bus[li] >= 0) continue;
      const double li_kw = std::max(0.0, c.loads[li].p_kw);
      auto it = dc_comp.find(c.loads[li].bus);
      if (it == dc_comp.end()) { out.shed_by_load[li] = li_kw; continue; }
      const int cid = it->second;
      const double dc_cap = comp_dc_source_kw.count(cid) ? comp_dc_source_kw.at(cid) : 0.0;
      const double ac_cap = comp_ac_source_kw.count(cid) ? comp_ac_source_kw.at(cid) : 0.0;
      const double ac_served = comp_ac_served_kw.count(cid) ? comp_ac_served_kw.at(cid) : 0.0;
      const double ac_surplus = std::max(0.0, ac_cap - ac_served);
      const double transfer_cap = comp_vsc_transfer_kw.count(cid) ? comp_vsc_transfer_kw.at(cid) : 0.0;
      const double cap = dc_cap + std::min(ac_surplus, transfer_cap);
      const double demand = dc_comp_demand.count(cid) ? dc_comp_demand.at(cid) : 0.0;
      if (cap >= demand - 1e-6) {
        out.shed_by_load[li] = 0.0;
      } else {
        // Proportional shed: each DC load gets shed proportional to its share
        const double ratio = demand > 1e-9 ? (demand - cap) / demand : 1.0;
        out.shed_by_load[li] = li_kw * std::min(1.0, std::max(0.0, ratio));
      }
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

void run_native_case(const NativeCase& c, ThreeStageReliabilityResult& r,
                     int max_sw_ops = INT_MAX) {
  // Clamp negative values: negative has no sensible meaning; treat as 0
  // (no switching) rather than propagating a misleading negative count string.
  if (max_sw_ops < 0) max_sw_ops = 0;
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
    auto s2 = solve_stage_milp(c, fault, 2, max_sw_ops);  // switch-count limit applies only to Stage 2
    auto s3 = solve_stage_milp(c, fault, 3);

    ThreeStageFaultDetail d;
    d.line_id = fault.id;
    d.stage1_status = s1.status;
    d.stage2_status = s2.status;
    d.stage3_status = s3.status;
    d.stage1_mip_gap = s1.mip_gap;
    d.stage2_mip_gap = s2.mip_gap;
    d.stage3_mip_gap = s3.mip_gap;
    auto is_success_like = [](const std::string& s) { return s.rfind("success", 0) == 0; };
    const bool all_success = is_success_like(s1.status) &&
                 is_success_like(s2.status) &&
                 is_success_like(s3.status);
    const bool any_approx = s1.status == "success (approximate)" ||
                s2.status == "success (approximate)" ||
                s3.status == "success (approximate)";
    d.status = !all_success ? "failed" : (any_approx ? "success (approximate)" : "success");
    d.pls_stage1 = s1.shed_kw;
    d.pls_stage2 = s2.shed_kw;
    d.pls_stage3 = s3.shed_kw;
    d.pls_total = d.pls_stage1 + d.pls_stage2 + d.pls_stage3;
    // P1c: kTauSwitchHr/kTauTrippingHr/kTauRepairHr are boundary times
    // (τ_SW, τ_TP, τ_RP).  Stage durations are τ_SW, τ_TP-τ_SW, τ_RP-τ_TP.
    d.objective = d.pls_stage1 * kTauSwitchHr
                + d.pls_stage2 * (kTauTrippingHr - kTauSwitchHr)
                + d.pls_stage3 * (kTauRepairHr   - kTauTrippingHr);
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
           s2.shed_by_load[i] * (kTauTrippingHr - kTauSwitchHr) +
           s3.shed_by_load[i] * (kTauRepairHr   - kTauTrippingHr));
      r.nodal_eens_kwh_yr[i] += ens;
      r.eens_kwh_yr += ens;
      const bool interrupted = s1.shed_by_load[i] > 1e-6 || s2.shed_by_load[i] > 1e-6 || s3.shed_by_load[i] > 1e-6;
      if (interrupted) {
        const double cust = std::max(1.0, c.loads[i].customers);
        const double duration_min = ((s1.shed_by_load[i] > 1e-6 ? kTauSwitchHr                      : 0.0) +
                                     (s2.shed_by_load[i] > 1e-6 ? (kTauTrippingHr - kTauSwitchHr)  : 0.0) +
                                     (s3.shed_by_load[i] > 1e-6 ? (kTauRepairHr   - kTauTrippingHr) : 0.0)) * 60.0;
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

  // Populate model limitations so callers can surface the approximations.
  const bool has_dc_or_vsc = !c.sys.dc.buses.empty() || !c.sys.dc.branches.empty() ||
                             !c.sys.dc.loads.empty() || !c.sys.vsc_converters.empty() ||
                             !c.sys.dc.dcdc_converters.empty();
  r.model_limitations =
      "AC network: finite-source LinDistFlow restoration MILP with energized-bus variables, "
      "explicit p_g/q_g source capacity bounds, strict commodity-flow radial forest, "
      "branch active/reactive flow limits, voltage bounds [vmin², vmax²], "
      "continuous load shed variables p^sh_i ∈ [0, p_d,i], "
      "and switch-count constraint"
      + (max_sw_ops == INT_MAX
           ? std::string(" (no limit)")
           : " (≤ " + std::to_string(max_sw_ops) + " operations per fault)")
      + ". "
       "DC sub-network: connectivity/capacity fallback with DC source capacity and "
       "AC-source surplus transferable through VSC capacity limits; no DC power-flow constraints. "
       "DCDC devices are still treated as lossless connectivity edges. "
      "SOP setpoints (psop) are zero (VSC dispatch not co-optimised in this model). "
      "Reactive power modelled as p_d * tan(arccos(0.9)); loss terms dropped (LinDistFlow).";

      // Validity flags describe whole-result physical coverage.  They are true for
      // pure-AC cases and intentionally false for hybrid cases because DC/VSC/SOP
      // physics are not part of the MILP.
      r.model_scope = has_dc_or_vsc ? "ac-lindistflow-milp+dc-connectivity-fallback"
                    : "ac-lindistflow-milp";
  r.validity = ThreeStageReliabilityResult::ValidityFlags{
        .branch_flow_enforced        = !has_dc_or_vsc,
        .voltage_constraints_enforced = !has_dc_or_vsc,
        .radial_topology_enforced    = !has_dc_or_vsc,
      .sop_dispatch_optimised      = false,  // VSC setpoints still zero
      .dc_power_flow_enforced      = false,  // DC sub-network is connectivity-only
        .restoration_milp_solved     = !has_dc_or_vsc,
  };

  // r.ok is true only when every fault stage solved to a verified optimum and
  // the submitted system is inside the evaluator's full physical scope.  Hybrid
  // DC/VSC/SOP cases return metrics, but they use the documented connectivity
  // fallback and therefore are not exact full-system MILP results.
  // Any stage that returned "failed*" used conservative full-shed estimates;
  // those values are still accumulated into EENS but the result is flagged
  // so that callers know the metrics are upper-bound estimates, not exact.
  // M3: "success (approximate)" stages also clear r.ok — the gap was not
  // proven, so the shed values are feasible but potentially non-optimal.
  bool any_failed = false;
  for (const auto& fd : r.faults) {
    if (fd.status.rfind("failed", 0) == 0 ||
        fd.status == "success (approximate)") {
      any_failed = true;
      break;
    }
  }
  r.ok = !any_failed && !has_dc_or_vsc;
  if (!r.ok && r.error.empty()) {
    if (has_dc_or_vsc) {
      r.error = "hybrid three-stage reliability uses DC/VSC/SOP connectivity fallback; "
                "full physical restoration MILP coverage is not available";
    } else if (any_failed) {
      r.error = "one or more fault stages failed or returned an approximate solution";
    }
  }
}

}  // anonymous namespace

ThreeStageReliabilityResult run_three_stage_reliability(
    const fs::path& case_json,
    const ThreeStageReliabilityOptions& options) {
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
    run_native_case(c, result, options.max_switch_operations);
  } catch (const std::exception& e) {
    result.error = std::string("failed to evaluate native three-stage reliability: ") + e.what();
  }
  return result;
}

ThreeStageReliabilityResult run_three_stage_reliability_from_string(
    const std::string& case_json_text,
    const ThreeStageReliabilityOptions& options) {
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
    run_native_case(c, result, options.max_switch_operations);
  } catch (const std::exception& e) {
    result.error = std::string("failed to evaluate native three-stage reliability: ") + e.what();
  }
  return result;
}

}  // namespace hacdcpf::analysis
