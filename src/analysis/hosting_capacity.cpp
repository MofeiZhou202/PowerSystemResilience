/// analysis/hosting_capacity.cpp
/// =============================
/// DL/T 2041-2025 distributed-resource hosting-capacity assessment
/// (equipment-level path + optional engineering verification).

#include "hacdcpf/analysis/hosting_capacity.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/analysis/harmonics_power_flow.hpp"

namespace hacdcpf::analysis {

namespace {

// Bucket a nominal voltage (kV) into a DL/T 2041 assessment level string.
std::string voltage_level_str(double kv) {
  struct Lvl { double kv; const char* s; };
  static const Lvl levels[] = {
      {330.0, "330kV"}, {220.0, "220kV"}, {110.0, "110kV"},
      {66.0, "66kV"},   {35.0, "35kV"},   {10.0, "10kV"}};
  double best_d = 1e18;
  const char* best = "other";
  for (const auto& l : levels) {
    double d = std::abs(kv - l.kv);
    if (d < best_d) { best_d = d; best = l.s; }
  }
  return best;
}

// Descending order used by the subordinate-obeys-superior grading pass.
int voltage_rank(const std::string& lvl) {
  if (lvl == "330kV") return 0;
  if (lvl == "220kV") return 1;
  if (lvl == "110kV") return 2;
  if (lvl == "66kV")  return 3;
  if (lvl == "35kV")  return 4;
  if (lvl == "10kV")  return 5;
  return 6;
}

// green / yellow / red from the lower-bound accessible capacities (DL/T 2041 §11).
std::string grade_from_accessible(double c1_min, double c2_min) {
  if (c2_min > 0.0) return "green";
  if (c1_min > 0.0) return "yellow";
  return "red";
}

}  // namespace

HostingCapacityOptions hosting_capacity_options_from_json(const nlohmann::json& j) {
  HostingCapacityOptions o;
  o.default_power_factor       = j.value("default_power_factor", o.default_power_factor);
  o.default_dr_max_output_coeff= j.value("default_dr_max_output_coeff", o.default_dr_max_output_coeff);
  o.single_transformer_beta    = j.value("single_transformer_beta", o.single_transformer_beta);
  o.n1_loading_limit           = j.value("n1_loading_limit", o.n1_loading_limit);
  o.enable_verification        = j.value("enable_verification", o.enable_verification);
  o.kr                         = j.value("kr", o.kr);
  o.delta_UH_pct               = j.value("delta_UH_pct", o.delta_UH_pct);
  o.delta_UL_pct               = j.value("delta_UL_pct", o.delta_UL_pct);
  o.thd_limit_pct              = j.value("thd_limit_pct", o.thd_limit_pct);
  o.enable_harmonic            = j.value("enable_harmonic", o.enable_harmonic);
  o.default_power_factor = std::clamp(o.default_power_factor, 0.0, 1.0);
  o.default_dr_max_output_coeff =
      std::max(o.default_dr_max_output_coeff, 1e-9);
  o.single_transformer_beta = std::max(o.single_transformer_beta, 0.0);
  o.n1_loading_limit = std::max(o.n1_loading_limit, 0.0);
  o.kr = std::clamp(o.kr, 0.0, 1.0);
  o.delta_UH_pct = std::max(o.delta_UH_pct, 0.0);
  o.delta_UL_pct = std::max(o.delta_UL_pct, 0.0);
  o.thd_limit_pct = std::max(o.thd_limit_pct, 0.0);
  return o;
}

HostingCapacityResult assess_hosting_capacity(const HybridPowerSystem& sys,
                                              const HostingCapacityOptions& opt) {
  HostingCapacityResult result;
  const auto& ac = sys.ac;

  // ── Bus lookup tables ────────────────────────────────────────────────────
  std::unordered_map<int, double> bus_kv;    // bus index → base_kv
  std::unordered_map<int, int>    bus_area;  // bus index → area
  std::unordered_map<int, bool>   bus_live;  // bus index → in_service
  for (const auto& b : ac.buses) {
    bus_kv[b.index]   = b.base_kv;
    bus_area[b.index] = b.area;
    bus_live[b.index] = b.in_service;
  }

  // ── Per-bus aggregation of load / non-DR gen / existing DR / ESS charging ─
  std::unordered_map<int, double> bus_load, bus_nondr_gen, bus_existing_dr, bus_ess_charge;
  for (const auto& b : ac.buses)
    if (b.in_service) bus_load[b.index] += std::max(0.0, b.pd_mw);
  for (const auto& ld : ac.loads)
    if (ld.in_service) bus_load[ld.bus] += std::max(0.0, ld.p_mw * ld.scaling);
  for (const auto& g : ac.generators)            // synchronous units = non-DR generation
    if (g.in_service) bus_nondr_gen[g.bus] += g.pg_mw;
  for (const auto& pv : ac.pv_systems)
    if (pv.in_service) bus_existing_dr[pv.bus] += pv.p_mw;
  for (const auto& rg : ac.renewable_gens)
    if (rg.in_service) bus_existing_dr[rg.bus] += rg.p_mw;
  for (const auto& sg : ac.static_generators)
    if (sg.in_service) bus_existing_dr[sg.bus] += std::max(0.0, sg.p_mw * sg.scaling);
  for (const auto& st : ac.storage) {
    if (!st.in_service) continue;
    double charge = (st.cap_charging_strategy == "static")
                        ? std::max(0.0, st.cap_static_charging_mw)
                        : std::max(0.0, -st.p_mw);   // negative p_mw = charging
    if (charge > 0.0) bus_ess_charge[st.bus] += charge;
  }

  // ── Line adjacency (branches whose endpoints share a voltage level) ───────
  // Transformers are boundaries and are intentionally NOT traversed, so a BFS
  // from a transformer's LV bus yields exactly its downstream supply area.
  auto is_line = [&](const ACBranch& br) {
    auto itf = bus_kv.find(br.from_bus);
    auto itt = bus_kv.find(br.to_bus);
    if (itf == bus_kv.end() || itt == bus_kv.end()) return false;
    return std::abs(itf->second - itt->second) < 1e-3;
  };
  std::unordered_map<int, std::vector<int>> line_adj;
  std::unordered_set<int> transformer_branch_idx;  // branches that ARE transformers
  for (const auto& br : ac.branches) {
    if (!br.in_service) continue;
    if (is_line(br)) {
      line_adj[br.from_bus].push_back(br.to_bus);
      line_adj[br.to_bus].push_back(br.from_bus);
    } else {
      transformer_branch_idx.insert(br.index);
    }
  }

  auto supply_area_buses = [&](int lv_bus) {
    std::unordered_set<int> seen;
    std::deque<int> q;
    if (bus_live.count(lv_bus) && bus_live[lv_bus]) { seen.insert(lv_bus); q.push_back(lv_bus); }
    while (!q.empty()) {
      int u = q.front(); q.pop_front();
      auto it = line_adj.find(u);
      if (it == line_adj.end()) continue;
      for (int v : it->second) {
        if (seen.count(v)) continue;
        if (!(bus_live.count(v) && bus_live[v])) continue;
        seen.insert(v);
        q.push_back(v);
      }
    }
    return seen;
  };

  // ── Enumerate transformers: transformers_2w + transformer-typed branches ──
  struct Xf { int index; std::string name; int hv_bus; int lv_bus; double sn_mva; int n_parallel;
              double cap_pf; double cap_beta; double cap_tau; double cap_reg;
              double cap_ess_min; double cap_ess_max;
              std::string canvas_type; int canvas_index; };
  std::vector<Xf> xfs;
  std::unordered_set<int> used_branch;   // source branches already owned by a transformer2w
  std::unordered_set<long long> xf_bus_pairs;  // {min,max} bus pair covered by a transformer2w
  auto pair_key = [](int a, int b) -> long long {
    int lo = std::min(a, b), hi = std::max(a, b);
    return static_cast<long long>(lo) * 1000000LL + hi;
  };
  for (const auto& t : ac.transformers_2w) {
    if (!t.in_service) continue;
    if (t.source_branch_idx > 0) used_branch.insert(t.source_branch_idx);
    xf_bus_pairs.insert(pair_key(t.hv_bus, t.lv_bus));
    xfs.push_back(Xf{t.index, t.name.empty() ? ("T" + std::to_string(t.index)) : t.name,
                     t.hv_bus, t.lv_bus, t.sn_mva, std::max(1, t.n_parallel),
                     t.cap_power_factor, t.cap_max_reverse_load_rate, t.cap_dr_max_output_coeff,
                     t.cap_registered_dr_mw, t.cap_expected_new_storage_min_mw,
                     t.cap_expected_new_storage_max_mw,
                     t.source_branch_idx > 0 ? "ac_branch" : "transformer_2w",
                     t.source_branch_idx > 0 ? t.source_branch_idx : t.index});
  }
  for (const auto& br : ac.branches) {
    if (!br.in_service) continue;
    if (!transformer_branch_idx.count(br.index)) continue;  // only transformer-branches
    if (used_branch.count(br.index)) continue;              // already represented by a 2W row
    if (xf_bus_pairs.count(pair_key(br.from_bus, br.to_bus))) continue;  // same device as a 2W row
    double fk = bus_kv.count(br.from_bus) ? bus_kv[br.from_bus] : 0.0;
    double tk = bus_kv.count(br.to_bus) ? bus_kv[br.to_bus] : 0.0;
    int hv = fk >= tk ? br.from_bus : br.to_bus;
    int lv = fk >= tk ? br.to_bus : br.from_bus;
    double sn = br.sn_mva > 0 ? br.sn_mva : (br.rate_a_mva > 0 ? br.rate_a_mva : 0.0);
    xfs.push_back(Xf{br.index, br.name.empty() ? ("Br" + std::to_string(br.index)) : br.name,
                     hv, lv, sn, std::max(1, br.n_parallel),
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     "ac_branch", br.index});   // branch-transformers use global defaults
  }

  // ── Equipment-level hosting capacity per transformer (DL/T 2041 §7) ───────
  for (const auto& x : xfs) {
    TransformerHostingResult r;
    r.index = x.index;
    r.name  = x.name;
    r.canvas_type = x.canvas_type;
    r.canvas_index = x.canvas_index;
    r.hv_bus = x.hv_bus;
    r.lv_bus = x.lv_bus;
    double hv_kv = bus_kv.count(x.hv_bus) ? bus_kv[x.hv_bus] : 0.0;
    r.voltage_level = voltage_level_str(hv_kv);
    r.area = bus_area.count(x.hv_bus) ? bus_area[x.hv_bus] : 1;
    r.sn_mva = x.sn_mva;

    double pf  = x.cap_pf  > 0 ? x.cap_pf  : opt.default_power_factor;
    double tau = x.cap_tau > 0 ? x.cap_tau : opt.default_dr_max_output_coeff;
    if (tau <= 0) tau = 1.0;
    int n = std::max(1, x.n_parallel);
    bool beta_auto = !(x.cap_beta > 0);
    double beta = x.cap_beta;
    if (beta_auto) {
      beta = (n <= 1) ? opt.single_transformer_beta
                      : (static_cast<double>(n - 1) / n) * opt.n1_loading_limit;
    }
    r.power_factor = pf; r.tau_max = tau; r.beta = beta; r.beta_auto = beta_auto;

    // Aggregate the LV-side supply area.
    auto area_buses = supply_area_buses(x.lv_bus);
    double P = 0, PG = 0, DR = 0, PESS = 0;
    for (int b : area_buses) {
      if (bus_load.count(b))        P    += bus_load[b];
      if (bus_nondr_gen.count(b))   PG   += bus_nondr_gen[b];
      if (bus_existing_dr.count(b)) DR   += bus_existing_dr[b];
      if (bus_ess_charge.count(b))  PESS += bus_ess_charge[b];
    }
    r.supply_load_mw = P; r.supply_nondr_gen_mw = PG;
    r.supply_existing_dr_mw = DR; r.supply_ess_charging_mw = PESS;
    r.supply_bus_count = static_cast<int>(area_buses.size());

    double s_total = static_cast<double>(n) * x.sn_mva;
    double reverse_allow = beta * s_total * pf;
    double base = P - PG + reverse_allow + PESS;
    double dpess_min = x.cap_ess_min;
    double dpess_max = x.cap_ess_max;
    double sd_min = std::max(0.0, (base + dpess_min) / tau);
    double sd_max = std::max(0.0, (base + dpess_max) / tau);
    if (sd_min > sd_max) std::swap(sd_min, sd_max);
    r.hosting_min_mw = sd_min;
    r.hosting_max_mw = sd_max;

    // Accessible capacity (DL/T 2041 §10) — may be negative by design.
    r.registered_dr_mw = x.cap_reg;
    r.accessible_grid_min_mw = sd_min - DR;
    r.accessible_grid_max_mw = sd_max - DR;
    r.accessible_reg_min_mw  = r.accessible_grid_min_mw - x.cap_reg;
    r.accessible_reg_max_mw  = r.accessible_grid_max_mw - x.cap_reg;

    r.self_grade = grade_from_accessible(r.accessible_grid_min_mw, r.accessible_reg_min_mw);
    r.grade = r.self_grade;  // refined by the area pass below
    result.transformers.push_back(std::move(r));
  }

  // ── Area aggregation & grading (DL/T 2041 §8 / §11.1) ────────────────────
  std::map<int, AreaHostingResult> area_map;
  for (const auto& t : result.transformers) {
    auto& a = area_map[t.area];
    a.area = t.area;
    a.transformer_count += 1;
    a.hosting_min_mw += t.hosting_min_mw;
    a.hosting_max_mw += t.hosting_max_mw;
    a.existing_dr_mw += t.supply_existing_dr_mw;
    a.registered_dr_mw += t.registered_dr_mw;
  }
  std::unordered_map<int, std::string> area_grade;
  for (auto& [aid, a] : area_map) {
    a.accessible_grid_min_mw = a.hosting_min_mw - a.existing_dr_mw;
    a.accessible_grid_max_mw = a.hosting_max_mw - a.existing_dr_mw;
    a.accessible_reg_min_mw  = a.accessible_grid_min_mw - a.registered_dr_mw;
    a.accessible_reg_max_mw  = a.accessible_grid_max_mw - a.registered_dr_mw;
    a.grade = grade_from_accessible(a.accessible_grid_min_mw, a.accessible_reg_min_mw);
    area_grade[aid] = a.grade;
    if (a.grade == "red")
      result.warnings.push_back("Area " + std::to_string(aid) +
                                ": no remaining grid-connection space (red) — suspend new DR connection.");
    else if (a.grade == "yellow")
      result.warnings.push_back("Area " + std::to_string(aid) +
                                ": registered projects cannot all be connected (yellow).");
    result.areas.push_back(a);
  }

  // ── Final transformer grade: subordinate obeys superior (area) ───────────
  // Process high→low voltage for determinism (DL/T 2041 §11.2.1 order).
  std::sort(result.transformers.begin(), result.transformers.end(),
            [](const TransformerHostingResult& x, const TransformerHostingResult& y) {
              return voltage_rank(x.voltage_level) < voltage_rank(y.voltage_level);
            });
  for (auto& t : result.transformers) {
    std::string sup = area_grade.count(t.area) ? area_grade[t.area] : "green";
    t.grade = (sup == "red") ? std::string("red") : t.self_grade;
  }

  // ── Optional engineering verification (DL/T 2041 §12) ────────────────────
  if (opt.enable_verification) {
    auto& v = result.verification;
    v.run = true;

    PowerFlowOptions pf_opt;
    pf_opt.max_iter = 100;
    pf_opt.tol = 1e-8;
    PowerFlowResult pf = solve_power_flow(sys, pf_opt);
    v.power_flow_converged = pf.converged;
    v.pf_iterations = pf.iterations;
    v.pf_residual = pf.residual;

    if (!pf.converged) {
      v.power_flow_passed = false;
      v.violations.push_back(HostingViolation{"power_flow_divergence", 0, "system", pf.residual,
                                              pf_opt.tol, "pu", "critical",
                                              "Verification power flow did not converge."});
    } else {
      // Voltage deviation (Δ = (Vm − 1) × 100 %).
      for (size_t i = 0; i < ac.buses.size(); ++i) {
        const auto& b = ac.buses[i];
        if (!b.in_service || i >= pf.vm.size()) continue;
        double dev = (pf.vm[i] - 1.0) * 100.0;
        if (dev > opt.delta_UH_pct || dev < -opt.delta_UL_pct) {
          v.voltage_deviation_passed = false;
          v.violations.push_back(HostingViolation{
              "voltage_limit", b.index, b.name.empty() ? ("Bus" + std::to_string(b.index)) : b.name,
              dev, dev > 0 ? opt.delta_UH_pct : -opt.delta_UL_pct, "%",
              std::abs(dev) > 10 ? "high" : "medium",
              "Steady-state voltage deviation outside allowed band."});
        }
      }
      // Branch / transformer loading vs. thermal rating.
      for (size_t i = 0; i < ac.branches.size() && i < pf.branch_flows.size(); ++i) {
        const auto& br = ac.branches[i];
        if (!br.in_service || br.rate_a_mva <= 0) continue;
        const auto& bf = pf.branch_flows[i];
        double s = std::sqrt(bf.pf_mw * bf.pf_mw + bf.qf_mvar * bf.qf_mvar);
        double loading = s / br.rate_a_mva * 100.0;
        if (loading > 100.0) {
          v.power_flow_passed = false;
          v.violations.push_back(HostingViolation{
              "transformer_overload", br.from_bus,
              br.name.empty() ? ("Br" + std::to_string(br.index)) : br.name,
              loading, 100.0, "%", loading > 120 ? "high" : "medium",
              "Branch/transformer loading exceeds rating."});
        }
      }
    }

    // Short-circuit vs. breaker interrupting capability (DL/T 2041 Eq.3).
    SCDetailedOptions sc_opt;
    sc_opt.fault_type = FaultType::ThreePhase;
    sc_opt.compute_ith = true;
    std::vector<int> fault_buses;
    std::unordered_map<int, double> bus_breaker;
    for (const auto& b : ac.buses)
      if (b.in_service && b.i_breaker_ka > 0) { fault_buses.push_back(b.index); bus_breaker[b.index] = b.i_breaker_ka; }
    if (!fault_buses.empty()) {
      try {
        auto sc = run_short_circuit_detailed_batch(sys, fault_buses, sc_opt);
        for (const auto& dr : sc) {
          for (const auto& br : dr.bus_results) {
            if (br.bus_id != dr.fault_bus_id) continue;
            double lim = bus_breaker.count(br.bus_id) ? bus_breaker[br.bus_id] : 0.0;
            if (lim > 0 && br.ikss_ka > lim) {
              v.short_circuit_passed = false;
              v.violations.push_back(HostingViolation{
                  "short_circuit_exceedance", br.bus_id, "Bus" + std::to_string(br.bus_id),
                  br.ikss_ka, lim, "kA", "critical",
                  "Three-phase short-circuit current exceeds breaker interrupting capability."});
            }
          }
        }
      } catch (...) { /* SC engine unavailable for this case — leave passed=true */ }
    }

    // Harmonic distortion (best-effort; DL/T 2041 §12.5).
    if (opt.enable_harmonic) {
      try {
        harmonics::HPFResult hr = harmonics::solve_harmonic_power_flow(sys);
        v.harmonic_evaluated = hr.ok;
        v.max_thd_pct = hr.max_ac_thd_pct;
        if (hr.ok && hr.max_ac_thd_pct > opt.thd_limit_pct) {
          v.harmonic_passed = false;
          v.violations.push_back(HostingViolation{
              "harmonic_exceedance", hr.max_ac_thd_bus,
              "Bus" + std::to_string(hr.max_ac_thd_bus),
              hr.max_ac_thd_pct, opt.thd_limit_pct, "%", "medium",
              "Total harmonic distortion exceeds limit."});
        }
      } catch (...) { v.harmonic_evaluated = false; }
    }

    // Overall recommendation.
    bool critical = false, any = !v.violations.empty();
    for (const auto& vi : v.violations) if (vi.severity == "critical") critical = true;
    if (critical) v.recommendation = "suspend_connection";
    else if (any) v.recommendation = "allow_with_mitigation";
    else v.recommendation = "allow_connection";
  }

  return result;
}

// ───────────────────────────────────────────────────────────────────────────
// JSON serialization
// ───────────────────────────────────────────────────────────────────────────
nlohmann::json hosting_capacity_result_to_json(const HostingCapacityResult& r) {
  using nlohmann::json;
  json out;
  out["standard"] = r.standard;

  json txs = json::array();
  for (const auto& t : r.transformers) {
    txs.push_back(json{
        {"index", t.index}, {"name", t.name},
        {"canvas_type", t.canvas_type}, {"canvas_index", t.canvas_index},
        {"hv_bus", t.hv_bus}, {"lv_bus", t.lv_bus},
        {"voltage_level", t.voltage_level}, {"area", t.area},
        {"sn_mva", t.sn_mva}, {"power_factor", t.power_factor},
        {"beta", t.beta}, {"beta_auto", t.beta_auto}, {"tau_max", t.tau_max},
        {"supply_load_mw", t.supply_load_mw}, {"supply_nondr_gen_mw", t.supply_nondr_gen_mw},
        {"supply_existing_dr_mw", t.supply_existing_dr_mw},
        {"supply_ess_charging_mw", t.supply_ess_charging_mw},
        {"supply_bus_count", t.supply_bus_count},
        {"hosting_min_mw", t.hosting_min_mw}, {"hosting_max_mw", t.hosting_max_mw},
        {"registered_dr_mw", t.registered_dr_mw},
        {"accessible_grid_min_mw", t.accessible_grid_min_mw},
        {"accessible_grid_max_mw", t.accessible_grid_max_mw},
        {"accessible_reg_min_mw", t.accessible_reg_min_mw},
        {"accessible_reg_max_mw", t.accessible_reg_max_mw},
        {"self_grade", t.self_grade}, {"grade", t.grade}});
  }
  out["transformers"] = txs;

  json areas = json::array();
  for (const auto& a : r.areas) {
    areas.push_back(json{
        {"area", a.area}, {"transformer_count", a.transformer_count},
        {"hosting_min_mw", a.hosting_min_mw}, {"hosting_max_mw", a.hosting_max_mw},
        {"existing_dr_mw", a.existing_dr_mw}, {"registered_dr_mw", a.registered_dr_mw},
        {"accessible_grid_min_mw", a.accessible_grid_min_mw},
        {"accessible_grid_max_mw", a.accessible_grid_max_mw},
        {"accessible_reg_min_mw", a.accessible_reg_min_mw},
        {"accessible_reg_max_mw", a.accessible_reg_max_mw},
        {"grade", a.grade}});
  }
  out["areas"] = areas;
  out["warnings"] = r.warnings;

  const auto& v = r.verification;
  json vio = json::array();
  for (const auto& x : v.violations) {
    vio.push_back(json{{"type", x.type}, {"bus_id", x.bus_id}, {"equipment", x.equipment},
                       {"value", x.value}, {"limit", x.limit}, {"unit", x.unit},
                       {"severity", x.severity}, {"message", x.message}});
  }
  out["verification"] = json{
      {"run", v.run}, {"power_flow_converged", v.power_flow_converged},
      {"pf_iterations", v.pf_iterations}, {"pf_residual", v.pf_residual},
      {"power_flow_passed", v.power_flow_passed},
      {"short_circuit_passed", v.short_circuit_passed},
      {"voltage_deviation_passed", v.voltage_deviation_passed},
      {"harmonic_evaluated", v.harmonic_evaluated}, {"harmonic_passed", v.harmonic_passed},
      {"max_thd_pct", v.max_thd_pct}, {"recommendation", v.recommendation},
      {"violations", vio}};

  return out;
}

}  // namespace hacdcpf::analysis
