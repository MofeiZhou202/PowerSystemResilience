#include "hacdcpf/reliability/reliability_assessment.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <spdlog/spdlog.h>

#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"

namespace hacdcpf::analysis {

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// IEEE RTS-24 Load Profile Data
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
namespace {

// Compute unavailability from MTTF and MTTR
double compute_unavailability(double mttf_hr, double mttr_hr) {
  return mttr_hr / (mttf_hr + mttr_hr);
}

// Compute unavailability from lambda and repair duration
double compute_unavailability_lambda(double lambda_per_yr, double repair_hr) {
  // mu = repair rate = 8760 / repair_hr (repairs per year)
  // U = lambda / (lambda + mu)
  double mu = 8760.0 / repair_hr;
  return lambda_per_yr / (lambda_per_yr + mu);
}

// Hash function for state vectors (for state deduplication)
// Uses a position-sensitive FNV-style contribution so that components at
// positions i and i+64 produce distinct hash values (avoids BUG-1 collision).
size_t hash_state(const std::vector<bool>& state) {
  size_t h = 0;
  for (size_t i = 0; i < state.size(); ++i) {
    if (state[i]) {
      // Mix the position index into the contribution so each slot is unique.
      size_t val = (i + 1) * 2654435761ULL ^ 0x9e3779b97f4a7c15ULL;
      h ^= val + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
  }
  return h;
}

// Returns total in-service DC load (MW) for the given system.
// Used to populate model_limitations in exported result structs.
static double total_inservice_dc_load_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& ld : sys.dc.loads)
    if (ld.in_service) total += ld.p_mw;
  // Also count direct DCBus::pd_mw loads (not backed by a DCLoad element).
  for (const auto& b : sys.dc.buses)
    if (b.in_service) total += b.pd_mw;
  return total;
}

// State evaluation result
struct StateEvalResult {
  double curtailment_mw{0.0};
  std::vector<double> nodal_curtailment_mw;
  bool is_loss_state{false};
};

// Compute full component count for extended state vector:
// [generators | AC branches | static gens | renewable gens | storage |
//  VSC converters | DC branches | transformers_2w | transformers_3w]
struct ComponentOffsets {
  size_t ng, nl, nsg, nrg, nst, nvsc, ndb, nt2, nt3;
  size_t off_gen, off_br, off_sg, off_rg, off_st, off_vsc, off_db, off_t2, off_t3;
  size_t total;
  
  ComponentOffsets(const HybridPowerSystem& sys) {
    ng  = sys.ac.generators.size();
    nl  = sys.ac.branches.size();
    nsg = sys.ac.static_generators.size();
    nrg = sys.ac.renewable_gens.size();
    nst = sys.ac.storage.size();
    nvsc = sys.vsc_converters.size();
    ndb = sys.dc.branches.size();
    nt2 = sys.ac.transformers_2w.size();
    nt3 = sys.ac.transformers_3w.size();
    
    off_gen = 0;
    off_br  = ng;
    off_sg  = off_br + nl;
    off_rg  = off_sg + nsg;
    off_st  = off_rg + nrg;
    off_vsc = off_st + nst;
    off_db  = off_vsc + nvsc;
    off_t2  = off_db + ndb;
    off_t3  = off_t2 + nt2;
    total   = off_t3 + nt3;
  }
};

// Evaluate a single system state using DC-OPF
// The state vector layout follows ComponentOffsets.
//
// ── HYBRID AC/DC PHYSICS LIMITATION ──────────────────────────────────────────
// Despite accepting VSC and DC branch failures (via component_failures),
// the underlying power-balance evaluation is an AC-only DC OPF (solve_dc_opf).
// DC bus loads, DC bus generation, and VSC/DC branch flow constraints are NOT
// included in the OPF model.  The effect of a failed VSC or DC branch is
// captured only indirectly: the AC-side island pre-screening may detect that
// the AC network has become disconnected, and any AC-side load stranded in a
// dead island is shed directly.  However:
//   • DC loads are not shed through the OPF; their curtailment is zero.
//   • VSC power injection into the AC network is treated as zero when the VSC
//     is out of service, but DC-side reserves are NOT re-dispatched.
//   • EENS/LOLE computed from this function will underestimate true mixed-
//     system curtailment for systems with significant DC load or DC generation.
// See also apply_fmea_support_sources() which approximates grid-forming VSC
// converters as AC emergency generators — a conservative approximation that
// can overestimate available support in post-fault AC-island scenarios.
// ─────────────────────────────────────────────────────────────────────────────
StateEvalResult evaluate_state(
    HybridPowerSystem sys,  // copy intentional
    const std::vector<bool>& component_failures,
    const opf::DCOPFOptions& opf_opt,
    double load_scale = 1.0) {
  
  StateEvalResult result;
  ComponentOffsets co(sys);
  
  // Apply generator failures
  for (size_t i = 0; i < co.ng; ++i) {
    if (component_failures[co.off_gen + i]) {
      sys.ac.generators[i].in_service = false;
    }
  }
  
  // Apply branch failures
  for (size_t i = 0; i < co.nl; ++i) {
    if (component_failures[co.off_br + i]) {
      sys.ac.branches[i].in_service = false;
    }
  }
  
  // Apply static generator failures
  for (size_t i = 0; i < co.nsg; ++i) {
    if (component_failures[co.off_sg + i]) {
      sys.ac.static_generators[i].in_service = false;
    }
  }
  
  // Apply renewable generator failures
  for (size_t i = 0; i < co.nrg; ++i) {
    if (component_failures[co.off_rg + i]) {
      sys.ac.renewable_gens[i].in_service = false;
    }
  }
  
  // Apply storage failures
  for (size_t i = 0; i < co.nst; ++i) {
    if (component_failures[co.off_st + i]) {
      sys.ac.storage[i].in_service = false;
    }
  }
  
  // Apply VSC converter failures
  for (size_t i = 0; i < co.nvsc; ++i) {
    if (component_failures[co.off_vsc + i]) {
      sys.vsc_converters[i].in_service = false;
    }
  }
  
  // Apply DC branch failures
  for (size_t i = 0; i < co.ndb; ++i) {
    if (component_failures[co.off_db + i]) {
      sys.dc.branches[i].in_service = false;
    }
  }
  
  // Apply transformer 2W failures
  for (size_t i = 0; i < co.nt2; ++i) {
    if (component_failures[co.off_t2 + i]) {
      sys.ac.transformers_2w[i].in_service = false;
    }
  }
  
  // Apply transformer 3W failures
  for (size_t i = 0; i < co.nt3; ++i) {
    if (component_failures[co.off_t3 + i]) {
      sys.ac.transformers_3w[i].in_service = false;
    }
  }
  
  // Apply load scaling if needed
  if (std::abs(load_scale - 1.0) > 1e-9) {
    for (auto& bus : sys.ac.buses) {
      bus.pd_mw *= load_scale;
      bus.qd_mvar *= load_scale;
    }
    for (auto& ld : sys.ac.loads) {
      ld.p_mw *= load_scale;
      ld.q_mvar *= load_scale;
    }
  }

  // ── Island pre-screening ──────────────────────────────────────────────────
  // Identify islands with no voltage reference (IsolatedLoad / NoSlack).
  // Their loads are shed directly without running OPF, and the bus-level
  // pd_mw is zeroed to keep the DC-OPF B-matrix well-posed.
  double direct_shed_mw = 0.0;
  std::vector<double> nodal_direct_shed(sys.ac.buses.size(), 0.0);
  {
    namespace gr = hacdcpf::graph;
    auto g    = gr::build_power_system_graph(sys);
    auto topo = gr::analyze_topology(g);

    // Build bus_id → array-position map
    std::unordered_map<int, int> bus_pos;
    bus_pos.reserve(sys.ac.buses.size());
    for (int i = 0; i < (int)sys.ac.buses.size(); ++i)
      bus_pos[sys.ac.buses[i].index] = i;

    // Collect all buses belonging to dead islands
    std::unordered_set<int> dead_buses;
    for (const auto& isl : topo.islands) {
      if (isl.status == gr::IslandStatus::IsolatedLoad ||
          isl.status == gr::IslandStatus::NoSlack) {
        for (int bid : isl.ac_bus_ids) dead_buses.insert(bid);
      }
    }

    // Accumulate curtailment.  Convention (matching ONR and DC-OPF):
    // if sys.ac.loads is non-empty it is the authoritative load source;
    // bus.pd_mw is used only when the Load table is empty.
    // NOTE: if dc_opf_solver ever changes to treat them as additive, this
    // logic must be updated to sum both to avoid under-counting curtailment.
    if (!sys.ac.loads.empty()) {
      for (auto& ld : sys.ac.loads) {
        if (!ld.in_service || !dead_buses.count(ld.bus)) continue;
        double extra = std::max(0.0, ld.p_mw * ld.scaling);
        direct_shed_mw += extra;
        if (auto it = bus_pos.find(ld.bus); it != bus_pos.end())
          nodal_direct_shed[it->second] += extra;
        ld.in_service = false;  // prevent OPF from seeing this load
      }
    } else {
      for (int bid : dead_buses) {
        auto it = bus_pos.find(bid);
        if (it == bus_pos.end()) continue;
        int pos = it->second;
        double load = std::max(0.0, sys.ac.buses[pos].pd_mw);
        direct_shed_mw += load;
        nodal_direct_shed[pos] += load;
      }
    }

    // Always zero bus-level loads for dead buses to prevent a singular
    // B-matrix in the DC-OPF (isolated bus ⇒ zero row in B).
    for (int bid : dead_buses) {
      auto it = bus_pos.find(bid);
      if (it == bus_pos.end()) continue;
      sys.ac.buses[it->second].pd_mw   = 0.0;
      sys.ac.buses[it->second].qd_mvar = 0.0;
    }

    // Fast path: skip OPF if no valid island exists
    bool has_valid = std::any_of(topo.islands.begin(), topo.islands.end(),
        [](const gr::IslandInfo& x) { return x.status == gr::IslandStatus::Valid; });
    if (!has_valid) {
      result.curtailment_mw       = direct_shed_mw;
      result.nodal_curtailment_mw = std::move(nodal_direct_shed);
      result.is_loss_state        = (result.curtailment_mw > 0.01);
      return result;
    }
  }

  // Run DC-OPF with load shedding enabled.
  // Guard: warn if the system has non-trivial DC loads that this AC-only OPF
  // cannot model.  The resulting EENS/LOLE will be optimistic for such systems.
  {
    double total_dc_load_mw = 0.0;
    for (const auto& ld : sys.dc.loads)
      if (ld.in_service) total_dc_load_mw += ld.p_mw;
    if (total_dc_load_mw > 0.01) {
      spdlog::warn("[FMEA] 系统含 DC 负荷 {:.3f} MW，但当前可靠性评估使用 AC-only DC-OPF。"
                   "DC 负荷中断不计入 OPF，EENS/LOLE 可能偏乐观。",
                   total_dc_load_mw);
    }
  }
  auto opf_result = opf::solve_dc_opf(sys, opf_opt);

  // Combine direct shed (dead islands) + OPF shed (surviving islands)
  result.curtailment_mw       = direct_shed_mw + opf_result.total_load_shedding_mw;
  result.nodal_curtailment_mw = std::move(nodal_direct_shed);
  if (!opf_result.load_shedding_mw.empty()) {
    result.nodal_curtailment_mw.resize(
        std::max(result.nodal_curtailment_mw.size(),
                 opf_result.load_shedding_mw.size()), 0.0);
    for (size_t i = 0; i < opf_result.load_shedding_mw.size(); ++i)
      result.nodal_curtailment_mw[i] += opf_result.load_shedding_mw[i];
  }
  result.is_loss_state = (result.curtailment_mw > 0.01);

  return result;
}

// Sample component states (Bernoulli trials)
std::vector<bool> sample_state(
    const std::vector<double>& unavailabilities,
    std::mt19937& rng) {
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  std::vector<bool> state(unavailabilities.size(), false);
  for (size_t i = 0; i < unavailabilities.size(); ++i) {
    state[i] = (dist(rng) < unavailabilities[i]);
  }
  return state;
}

// Count number of loss events (transitions from 0 to 1) in a time series
int count_loss_events(const std::vector<bool>& loss_flags) {
  if (loss_flags.empty()) return 0;
  int count = loss_flags[0] ? 1 : 0;
  for (size_t i = 1; i < loss_flags.size(); ++i) {
    if (loss_flags[i] && !loss_flags[i - 1]) ++count;
  }
  return count;
}

}  // namespace

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Public Functions
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

ReliabilityResult run_nonsequential_mc(
    const HybridPowerSystem& sys,
    const ReliabilityOptions& options) {
  
  spdlog::info("Non-Sequential MC: Starting reliability assessment");
  
  ReliabilityResult result;
  {
    double dc_mw = total_inservice_dc_load_mw(sys);
    if (dc_mw > 0.01)
      result.model_limitations = "DC loads not modelled in OPF (" +
          std::to_string(dc_mw) + " MW unaccounted)";
  }
  ComponentOffsets co(sys);
  const size_t nc = co.total;
  const size_t nb = sys.ac.buses.size();
  
  // Compute unavailabilities for all components
  std::vector<double> unavailabilities(nc);
  
  // --- Generators ---
  for (size_t i = 0; i < co.ng; ++i) {
    const auto& gen = sys.ac.generators[i];
    unavailabilities[co.off_gen + i] = gen.forced_outage_rate;
    if (unavailabilities[co.off_gen + i] <= 0) {
      if (gen.mttr_hr > 0) {
        double mttf = 2000.0;
        unavailabilities[co.off_gen + i] = compute_unavailability(mttf, gen.mttr_hr);
      } else {
        unavailabilities[co.off_gen + i] = 0.02;
      }
    }
  }
  
  // --- AC Branches ---
  for (size_t i = 0; i < co.nl; ++i) {
    const auto& br = sys.ac.branches[i];
    if (br.failure_rate > 0 && br.mttr_hr > 0) {
      unavailabilities[co.off_br + i] = compute_unavailability_lambda(br.failure_rate, br.mttr_hr);
    } else {
      unavailabilities[co.off_br + i] = 0.01;
    }
  }
  
  // --- Static Generators ---
  for (size_t i = 0; i < co.nsg; ++i) {
    const auto& sg = sys.ac.static_generators[i];
    if (sg.mtbf_hours > 0 && sg.mttr_hours > 0) {
      unavailabilities[co.off_sg + i] = compute_unavailability(sg.mtbf_hours, sg.mttr_hours);
    } else {
      unavailabilities[co.off_sg + i] = 0.03;
    }
  }
  
  // --- Renewable Generators ---
  for (size_t i = 0; i < co.nrg; ++i) {
    const auto& rg = sys.ac.renewable_gens[i];
    if (rg.mtbf_hours > 0 && rg.mttr_hours > 0) {
      unavailabilities[co.off_rg + i] = compute_unavailability(rg.mtbf_hours, rg.mttr_hours);
    } else {
      unavailabilities[co.off_rg + i] = 0.03;
    }
  }
  
  // --- Storage ---
  for (size_t i = 0; i < co.nst; ++i) {
    const auto& st = sys.ac.storage[i];
    double u = st.forced_outage_rate;
    if (u <= 0 && st.mttr_hr > 0) {
      double mttf = 5000.0;
      u = compute_unavailability(mttf, st.mttr_hr);
    }
    unavailabilities[co.off_st + i] = (u > 0) ? u : 0.02;
  }
  
  // --- VSC Converters ---
  for (size_t i = 0; i < co.nvsc; ++i) {
    const auto& v = sys.vsc_converters[i];
    double u = v.forced_outage_rate;
    if (u <= 0 && v.mttr_hr > 0) {
      double mttf = 10000.0;  // default MTTF for VSC
      u = compute_unavailability(mttf, v.mttr_hr);
    }
    unavailabilities[co.off_vsc + i] = (u > 0) ? u : 0.01;
  }
  
  // --- DC Branches ---
  for (size_t i = 0; i < co.ndb; ++i) {
    const auto& db = sys.dc.branches[i];
    if (db.mtbf_hours > 0 && db.mttr_hours > 0) {
      unavailabilities[co.off_db + i] = compute_unavailability(db.mtbf_hours, db.mttr_hours);
    } else {
      unavailabilities[co.off_db + i] = 0.005;
    }
  }
  
  // --- Transformers 2W ---
  for (size_t i = 0; i < co.nt2; ++i) {
    const auto& t = sys.ac.transformers_2w[i];
    if (t.mtbf_hours > 0 && t.mttr_hours > 0) {
      unavailabilities[co.off_t2 + i] = compute_unavailability(t.mtbf_hours, t.mttr_hours);
    } else {
      unavailabilities[co.off_t2 + i] = 0.005;
    }
  }
  
  // --- Transformers 3W ---
  for (size_t i = 0; i < co.nt3; ++i) {
    const auto& t = sys.ac.transformers_3w[i];
    if (t.mtbf_hours > 0 && t.mttr_hours > 0) {
      unavailabilities[co.off_t3 + i] = compute_unavailability(t.mtbf_hours, t.mttr_hours);
    } else {
      unavailabilities[co.off_t3 + i] = 0.005;
    }
  }
  
  spdlog::info("NSQ MC: {} total components (gen={}, br={}, sg={}, rg={}, st={}, vsc={}, db={}, t2={}, t3={})",
               nc, co.ng, co.nl, co.nsg, co.nrg, co.nst, co.nvsc, co.ndb, co.nt2, co.nt3);
  // Initialize random number generator
  std::mt19937 rng;
  if (options.seed != 0) {
    rng.seed(options.seed);
  } else {
    std::random_device rd;
    rng.seed(rd());
  }
  
  // DC-OPF options
  opf::DCOPFOptions opf_opt = options.opf_options;
  opf_opt.load_shedding = true;
  opf_opt.verbose = false;
  opf_opt.compute_lmp = false;  // batch path — LMPs not needed, skip supporting LP
  
  // State database for deduplication
  std::unordered_map<size_t, StateEvalResult> state_db;
  int n0_count = 0;  // Count of N-0 states
  int contingency_count = 0;  // Count of states with failures
  
  // Accumulators
  double sum_dns = 0.0;
  double sum_dns_sq = 0.0;
  int loss_hours = 0;
  std::vector<double> nodal_dns_sum(nb, 0.0);
  std::vector<double> comp_fail_count(nc, 0.0);
  int total_loss_samples = 0;
  
  // Compute total system load
  double total_load_mw = 0.0;
  for (const auto& bus : sys.ac.buses) {
    total_load_mw += bus.pd_mw;
  }
  for (const auto& ld : sys.ac.loads) {
    if (ld.in_service) total_load_mw += ld.p_mw * ld.scaling;
  }
  
  spdlog::info("NSQ MC: total load = {:.1f} MW (scale={:.2f})",
               total_load_mw * options.load_scale_factor, options.load_scale_factor);
  
  // Main simulation loop
  for (int iter = 1; iter <= options.max_iterations; ++iter) {
    // Sample component state
    auto state = sample_state(unavailabilities, rng);
    
    // Check if N-0 (all up)
    bool is_n0 = std::none_of(state.begin(), state.end(), [](bool b) { return b; });
    
    StateEvalResult eval_result;
    if (is_n0) {
      ++n0_count;
      if (options.load_scale_factor > 1.0) {
        eval_result = evaluate_state(sys, state, opf_opt, options.load_scale_factor);
      } else {
        eval_result.curtailment_mw = 0.0;
        eval_result.nodal_curtailment_mw.resize(nb, 0.0);
        eval_result.is_loss_state = false;
      }
    } else {
      ++contingency_count;
      // Count failures
      int num_fail = 0;
      for (size_t i = 0; i < nc; ++i) if (state[i]) ++num_fail;
      
      // Check state database
      size_t state_hash = hash_state(state);
      auto it = state_db.find(state_hash);
      if (it != state_db.end()) {
        eval_result = it->second;
      } else {
        // Evaluate new state
        eval_result = evaluate_state(sys, state, opf_opt, options.load_scale_factor);
        state_db[state_hash] = eval_result;
        
        if (state_db.size() <= 5) {
          spdlog::info("NSQ MC: State #{} - {} failures -> shed={:.2f} MW",
                       state_db.size(), num_fail, eval_result.curtailment_mw);
        }
      }
    }
    
    // Accumulate results
    double dns = eval_result.curtailment_mw;
    sum_dns += dns;
    sum_dns_sq += dns * dns;

    // Record per-sample EENS and LOLE for tail-risk computation
    if (options.compute_tail_risk) {
      result.annual_eens.push_back(dns * 8760.0);
      result.annual_lole.push_back(eval_result.is_loss_state ? 8760.0 : 0.0);
    }
    
    if (eval_result.is_loss_state) {
      ++loss_hours;
      ++total_loss_samples;
      
      // Accumulate nodal curtailment
      for (size_t b = 0; b < nb && b < eval_result.nodal_curtailment_mw.size(); ++b) {
        nodal_dns_sum[b] += eval_result.nodal_curtailment_mw[b];
      }
      
      // Track component failures during loss
      for (size_t c = 0; c < nc; ++c) {
        if (state[c]) comp_fail_count[c] += 1.0;
      }
    }
    
    // Update expected indices
    double edns = sum_dns / iter;
    double eens = edns * 8760.0;  // MWh/yr
    double lole = (double)loss_hours / iter * 8760.0;  // hr/yr
    (void)lole;  // Used in verbose output
    
    // Calculate CoV
    double cov = 0.0;
    if (iter > 1 && edns > 0) {
      double var = (sum_dns_sq / iter) - (edns * edns);
      if (var > 0) {
        double std_dev = std::sqrt(var);
        cov = std_dev / (edns * std::sqrt((double)iter));
      }
    }
    
    // Record history
    result.eens_history.push_back(eens);
    result.cov_history.push_back(cov);
    
    // Progress callback
    if (options.progress_callback) {
      if (!options.progress_callback(iter, eens, cov)) {
        spdlog::info("NSQ MC: Cancelled by user at iteration {}", iter);
        break;
      }
    }
    
    // Verbose output
    if (options.verbose && iter % 1000 == 0) {
      spdlog::info("NSQ MC: Iter {} | EENS={:.2f} MWh/yr | LOLE={:.2f} hr/yr | CoV={:.4f}",
                   iter, eens, lole, cov);
    }
    
    // Check convergence
    if (iter > 100 && cov > 0 && cov < options.cov_threshold) {
      spdlog::info("NSQ MC: Converged at iteration {} (CoV={:.4f})", iter, cov);
      result.converged = true;
      result.iterations_used = iter;
      break;
    }
    
    result.iterations_used = iter;
  }
  
  // Log summary of sampling
  spdlog::info("NSQ MC: Sampling summary - N-0 states: {}, Contingency states: {}, Unique states evaluated: {}",
               n0_count, contingency_count, state_db.size());
  spdlog::info("NSQ MC: Loss states: {}, Total loss samples: {}", loss_hours, total_loss_samples);
  
  // Final results
  int n = result.iterations_used;
  result.edns_mw = sum_dns / n;
  result.eens_mwh_yr = result.edns_mw * 8760.0;
  result.lole_hr_yr = (double)loss_hours / n * 8760.0;
  result.plc = (double)loss_hours / n;
  result.final_cov = result.cov_history.empty() ? 0.0 : result.cov_history.back();
  
  // Nodal EENS
  result.nodal_eens_mwh_yr.resize(nb);
  for (size_t b = 0; b < nb; ++b) {
    result.nodal_eens_mwh_yr[b] = (nodal_dns_sum[b] / n) * 8760.0;
  }
  
  // Critical components
  if (total_loss_samples > 0) {
    for (size_t c = 0; c < nc; ++c) {
      double importance = comp_fail_count[c] / total_loss_samples;
      if (importance > 0.01) {
        ReliabilityResult::ComponentImportance ci;
        // Compute within-type index based on the component's section in the
        // flat state vector (BUG-2 fix: was always using c - co.off_br).
        if      (c < co.off_br)  { ci.index = static_cast<int>(c - co.off_gen); }
        else if (c < co.off_sg)  { ci.index = static_cast<int>(c - co.off_br);  }
        else if (c < co.off_rg)  { ci.index = static_cast<int>(c - co.off_sg);  }
        else if (c < co.off_st)  { ci.index = static_cast<int>(c - co.off_rg);  }
        else if (c < co.off_vsc) { ci.index = static_cast<int>(c - co.off_st);  }
        else if (c < co.off_db)  { ci.index = static_cast<int>(c - co.off_vsc); }
        else if (c < co.off_t2)  { ci.index = static_cast<int>(c - co.off_db);  }
        else if (c < co.off_t3)  { ci.index = static_cast<int>(c - co.off_t2);  }
        else                     { ci.index = static_cast<int>(c - co.off_t3);  }
        ci.is_generator = (c < co.off_br);
        ci.importance = importance;
        result.critical_components.push_back(ci);
      }
    }
    // Sort by importance descending
    std::sort(result.critical_components.begin(), result.critical_components.end(),
              [](const auto& a, const auto& b) { return a.importance > b.importance; });
  }
  
  spdlog::info("NSQ MC: Complete. EENS={:.2f} MWh/yr, LOLE={:.2f} hr/yr, PLC={:.4f}",
               result.eens_mwh_yr, result.lole_hr_yr, result.plc);
  
  // 鈹€鈹€鈹€ Compute Tail Risk Metrics (if enabled and we have annual samples) 鈹€鈹€鈹€
  if (options.compute_tail_risk && !result.annual_eens.empty()) {
    result.tail_risk = compute_tail_risk(
        result.annual_eens, result.annual_lole, options.var_confidence);
  }
  
  return result;
}

ReliabilityResult run_sequential_mc(
    const HybridPowerSystem& sys,
    const LoadProfile& load_profile,
    const ReliabilityOptions& options) {
  
  spdlog::info("Sequential MC: Starting reliability assessment");
  
  ReliabilityResult result;
  {
    double dc_mw = total_inservice_dc_load_mw(sys);
    if (dc_mw > 0.01)
      result.model_limitations = "DC loads not modelled in OPF (" +
          std::to_string(dc_mw) + " MW unaccounted)";
  }
  ComponentOffsets co(sys);
  const size_t nc = co.total;
  const size_t nb = sys.ac.buses.size();
  const int hours_per_year = options.hours_per_year;
  
  // Compute MTTF and MTTR for all components (in hours)
  std::vector<double> mttf(nc);
  std::vector<double> mttr_v(nc);
  
  // --- Generators ---
  for (size_t i = 0; i < co.ng; ++i) {
    const auto& gen = sys.ac.generators[i];
    mttr_v[co.off_gen + i] = gen.mttr_hr > 0 ? gen.mttr_hr : 50.0;
    if (gen.forced_outage_rate > 0 && gen.forced_outage_rate < 1) {
      mttf[co.off_gen + i] = mttr_v[co.off_gen + i] * (1.0 - gen.forced_outage_rate) / gen.forced_outage_rate;
    } else {
      mttf[co.off_gen + i] = 2000.0;
    }
  }
  // --- AC Branches ---
  for (size_t i = 0; i < co.nl; ++i) {
    const auto& br = sys.ac.branches[i];
    mttr_v[co.off_br + i] = br.mttr_hr > 0 ? br.mttr_hr : 10.0;
    if (br.failure_rate > 0) {
      mttf[co.off_br + i] = 8760.0 / br.failure_rate;
    } else {
      mttf[co.off_br + i] = 25000.0;
    }
  }
  // --- Static Generators ---
  for (size_t i = 0; i < co.nsg; ++i) {
    const auto& sg = sys.ac.static_generators[i];
    mttr_v[co.off_sg + i] = sg.mttr_hours > 0 ? sg.mttr_hours : 24.0;
    mttf[co.off_sg + i] = sg.mtbf_hours > 0 ? sg.mtbf_hours : 5000.0;
  }
  // --- Renewable Generators ---
  for (size_t i = 0; i < co.nrg; ++i) {
    const auto& rg = sys.ac.renewable_gens[i];
    mttr_v[co.off_rg + i] = rg.mttr_hours > 0 ? rg.mttr_hours : 48.0;
    mttf[co.off_rg + i] = rg.mtbf_hours > 0 ? rg.mtbf_hours : 4000.0;
  }
  // --- Storage ---
  for (size_t i = 0; i < co.nst; ++i) {
    const auto& st = sys.ac.storage[i];
    mttr_v[co.off_st + i] = st.mttr_hr > 0 ? st.mttr_hr : 24.0;
    if (st.forced_outage_rate > 0 && st.forced_outage_rate < 1) {
      mttf[co.off_st + i] = mttr_v[co.off_st + i] * (1.0 - st.forced_outage_rate) / st.forced_outage_rate;
    } else {
      mttf[co.off_st + i] = 5000.0;
    }
  }
  // --- VSC Converters ---
  for (size_t i = 0; i < co.nvsc; ++i) {
    const auto& v = sys.vsc_converters[i];
    mttr_v[co.off_vsc + i] = v.mttr_hr > 0 ? v.mttr_hr : 48.0;
    if (v.forced_outage_rate > 0 && v.forced_outage_rate < 1) {
      mttf[co.off_vsc + i] = mttr_v[co.off_vsc + i] * (1.0 - v.forced_outage_rate) / v.forced_outage_rate;
    } else {
      mttf[co.off_vsc + i] = 10000.0;
    }
  }
  // --- DC Branches ---
  for (size_t i = 0; i < co.ndb; ++i) {
    const auto& db = sys.dc.branches[i];
    mttr_v[co.off_db + i] = db.mttr_hours > 0 ? db.mttr_hours : 24.0;
    mttf[co.off_db + i] = db.mtbf_hours > 0 ? db.mtbf_hours : 50000.0;
  }
  // --- Transformers 2W ---
  for (size_t i = 0; i < co.nt2; ++i) {
    const auto& t = sys.ac.transformers_2w[i];
    mttr_v[co.off_t2 + i] = t.mttr_hours > 0 ? t.mttr_hours : 200.0;
    mttf[co.off_t2 + i] = t.mtbf_hours > 0 ? t.mtbf_hours : 300000.0;
  }
  // --- Transformers 3W ---
  for (size_t i = 0; i < co.nt3; ++i) {
    const auto& t = sys.ac.transformers_3w[i];
    mttr_v[co.off_t3 + i] = t.mttr_hours > 0 ? t.mttr_hours : 200.0;
    mttf[co.off_t3 + i] = t.mtbf_hours > 0 ? t.mtbf_hours : 300000.0;
  }
  
  // Initialize random number generator
  std::mt19937 rng;
  if (options.seed != 0) {
    rng.seed(options.seed);
  } else {
    std::random_device rd;
    rng.seed(rd());
  }
  
  // DC-OPF options
  opf::DCOPFOptions opf_opt = options.opf_options;
  opf_opt.load_shedding = true;
  opf_opt.verbose = false;
  opf_opt.compute_lmp = false;  // batch path — LMPs not needed, skip supporting LP
  
  // Accumulators
  std::vector<double> nodal_eens_accum(nb, 0.0);
  std::vector<double> comp_fail_during_loss(nc, 0.0);
  int total_loss_hours = 0;
  
  spdlog::info("SEQ MC: {} total components, {} hours/year", nc, hours_per_year);
  
  // Main simulation loop (years)
  for (int year = 1; year <= options.max_iterations; ++year) {
    // Generate chronological state for each component
    // state_matrix[comp][hour] = true if component is DOWN
    std::vector<std::vector<bool>> state_matrix(nc, std::vector<bool>(hours_per_year, false));
    
    for (size_t c = 0; c < nc; ++c) {
      double current_time = 0.0;
      bool is_up = true;
      std::uniform_real_distribution<double> dist(0.0, 1.0);
      
      while (current_time < hours_per_year) {
        // Sample duration from exponential distribution
        double u = dist(rng);
        if (u <= 0) u = 1e-10;
        
        double duration;
        if (is_up) {
          // Time to failure
          duration = -mttf[c] * std::log(u);
          int dur_int = std::max(1, (int)std::round(duration));
          current_time += dur_int;
        } else {
          // Time to repair
          duration = -mttr_v[c] * std::log(u);
          int dur_int = std::max(1, (int)std::ceil(duration));
          
          int start_hour = (int)current_time;
          int end_hour = std::min(start_hour + dur_int, hours_per_year);
          
          for (int h = start_hour; h < end_hour; ++h) {
            state_matrix[c][h] = true;
          }
          current_time += dur_int;
        }
        is_up = !is_up;
      }
    }
    
    // Identify contingency hours (any component down)
    std::vector<int> contingency_hours;
    for (int h = 0; h < hours_per_year; ++h) {
      bool any_down = false;
      for (size_t c = 0; c < nc; ++c) {
        if (state_matrix[c][h]) {
          any_down = true;
          break;
        }
      }
      if (any_down) contingency_hours.push_back(h);
    }
    
    if (options.verbose && year % 100 == 0) {
      spdlog::info("SEQ MC: Year {} has {} contingency hours",
                   year, contingency_hours.size());
    }
    
    // Evaluate contingency hours
    double year_eens = 0.0;
    int year_loss_hours = 0;
    std::vector<bool> year_loss_flags(hours_per_year, false);
    
    for (int h : contingency_hours) {
      // Build component state for this hour
      std::vector<bool> state(nc);
      for (size_t c = 0; c < nc; ++c) {
        state[c] = state_matrix[c][h];
      }
      
      // Get load scale factor (profile 脳 overall scale)
      double load_scale = options.load_scale_factor;
      if (h < (int)load_profile.factors.size()) {
        load_scale *= load_profile.factors[h];
      }
      
      // Evaluate state
      auto eval_result = evaluate_state(sys, state, opf_opt, load_scale);
      
      if (eval_result.is_loss_state) {
        year_eens += eval_result.curtailment_mw;
        ++year_loss_hours;
        year_loss_flags[h] = true;
        
        // Accumulate nodal EENS
        for (size_t b = 0; b < nb && b < eval_result.nodal_curtailment_mw.size(); ++b) {
          nodal_eens_accum[b] += eval_result.nodal_curtailment_mw[b];
        }
        
        // Track component failures during loss
        for (size_t c = 0; c < nc; ++c) {
          if (state[c]) comp_fail_during_loss[c] += 1.0;
        }
        total_loss_hours++;
      }
    }
    
    // Count loss events (frequency)
    int year_lolf = count_loss_events(year_loss_flags);
    
    // Record annual results
    result.annual_eens.push_back(year_eens);
    result.annual_lole.push_back((double)year_loss_hours);
    result.annual_lolf.push_back((double)year_lolf);
    
    // Update cumulative indices
    double eens_avg = std::accumulate(result.annual_eens.begin(),
                                      result.annual_eens.end(), 0.0) / year;
    
    // Calculate CoV
    double cov = 0.0;
    if (year > 1 && eens_avg > 0) {
      double sum_sq = 0.0;
      for (double e : result.annual_eens) sum_sq += e * e;
      double var = (sum_sq / year) - (eens_avg * eens_avg);
      if (var > 0) {
        double std_dev = std::sqrt(var);
        cov = std_dev / (eens_avg * std::sqrt((double)year));
      }
    }
    
    result.eens_history.push_back(eens_avg);
    result.cov_history.push_back(cov);
    
    // Progress callback
    if (options.progress_callback) {
      if (!options.progress_callback(year, eens_avg, cov)) {
        spdlog::info("SEQ MC: Cancelled by user at year {}", year);
        break;
      }
    }
    
    // Verbose output
    if (options.verbose && year % 100 == 0) {
      double lole_avg = std::accumulate(result.annual_lole.begin(),
                                        result.annual_lole.end(), 0.0) / year;
      spdlog::info("SEQ MC: Year {} | EENS={:.2f} MWh/yr | LOLE={:.2f} hr/yr | CoV={:.4f}",
                   year, eens_avg, lole_avg, cov);
    }
    
    // Check convergence
    if (year > 100 && cov > 0 && cov < options.cov_threshold) {
      spdlog::info("SEQ MC: Converged at year {} (CoV={:.4f})", year, cov);
      result.converged = true;
      result.iterations_used = year;
      break;
    }
    
    result.iterations_used = year;
  }
  
  // Final results
  int n_years = result.iterations_used;
  result.eens_mwh_yr = std::accumulate(result.annual_eens.begin(),
                                        result.annual_eens.end(), 0.0) / n_years;
  result.lole_hr_yr = std::accumulate(result.annual_lole.begin(),
                                       result.annual_lole.end(), 0.0) / n_years;
  result.lolf_occ_yr = std::accumulate(result.annual_lolf.begin(),
                                        result.annual_lolf.end(), 0.0) / n_years;
  result.edns_mw = result.eens_mwh_yr / 8760.0;
  result.plc = result.lole_hr_yr / 8760.0;
  result.final_cov = result.cov_history.empty() ? 0.0 : result.cov_history.back();
  
  // Nodal EENS
  result.nodal_eens_mwh_yr.resize(nb);
  for (size_t b = 0; b < nb; ++b) {
    result.nodal_eens_mwh_yr[b] = nodal_eens_accum[b] / n_years;
  }
  
  // Critical components
  if (total_loss_hours > 0) {
    for (size_t c = 0; c < nc; ++c) {
      double importance = comp_fail_during_loss[c] / total_loss_hours;
      if (importance > 0.01) {
        ReliabilityResult::ComponentImportance ci;
        // Compute within-type index based on the component's section in the
        // flat state vector (BUG-2 fix: was always using c - co.off_br).
        if      (c < co.off_br)  { ci.index = static_cast<int>(c - co.off_gen); }
        else if (c < co.off_sg)  { ci.index = static_cast<int>(c - co.off_br);  }
        else if (c < co.off_rg)  { ci.index = static_cast<int>(c - co.off_sg);  }
        else if (c < co.off_st)  { ci.index = static_cast<int>(c - co.off_rg);  }
        else if (c < co.off_vsc) { ci.index = static_cast<int>(c - co.off_st);  }
        else if (c < co.off_db)  { ci.index = static_cast<int>(c - co.off_vsc); }
        else if (c < co.off_t2)  { ci.index = static_cast<int>(c - co.off_db);  }
        else if (c < co.off_t3)  { ci.index = static_cast<int>(c - co.off_t2);  }
        else                     { ci.index = static_cast<int>(c - co.off_t3);  }
        ci.is_generator = (c < co.off_br);
        ci.importance = importance;
        result.critical_components.push_back(ci);
      }
    }
    std::sort(result.critical_components.begin(), result.critical_components.end(),
              [](const auto& a, const auto& b) { return a.importance > b.importance; });
  }
  
  spdlog::info("SEQ MC: Complete. EENS={:.2f} MWh/yr, LOLE={:.2f} hr/yr, LOLF={:.2f} occ/yr",
               result.eens_mwh_yr, result.lole_hr_yr, result.lolf_occ_yr);
  
  // 鈹€鈹€鈹€ Compute Tail Risk Metrics (if enabled) 鈹€鈹€鈹€
  if (options.compute_tail_risk && !result.annual_eens.empty()) {
    result.tail_risk = compute_tail_risk(
        result.annual_eens, result.annual_lole, options.var_confidence);
  }
  
  return result;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Tail Risk Metrics (VaR / CVaR)
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

TailRiskMetrics compute_tail_risk(
    const std::vector<double>& eens_samples,
    const std::vector<double>& lole_samples,
    double confidence) {
  
  TailRiskMetrics metrics;
  if (eens_samples.empty()) return metrics;
  
  const size_t n = eens_samples.size();
  
  // Copy and sort for percentile calculations
  std::vector<double> sorted_eens = eens_samples;
  std::vector<double> sorted_lole = lole_samples.empty() ? 
      std::vector<double>(n, 0.0) : lole_samples;
  
  std::sort(sorted_eens.begin(), sorted_eens.end());
  std::sort(sorted_lole.begin(), sorted_lole.end());
  
  // VaR at confidence level (e.g., 95% 鈫?index at 95th percentile)
  size_t var_idx = static_cast<size_t>(std::ceil(confidence * n)) - 1;
  var_idx = std::min(var_idx, n - 1);
  
  metrics.eens_var = sorted_eens[var_idx];
  metrics.lole_var = sorted_lole[var_idx];
  
  // CVaR (Expected Shortfall): mean of values above VaR
  double eens_sum_above = 0.0;
  double lole_sum_above = 0.0;
  int count_above = 0;
  
  for (size_t i = var_idx; i < n; ++i) {
    eens_sum_above += sorted_eens[i];
    lole_sum_above += sorted_lole[i];
    ++count_above;
  }
  
  if (count_above > 0) {
    metrics.eens_cvar = eens_sum_above / count_above;
    metrics.lole_cvar = lole_sum_above / count_above;
  }
  
  // Store full distribution
  metrics.eens_distribution = eens_samples;
  metrics.lole_distribution = lole_samples;
  
  // Compute percentiles (5th, 25th, 50th, 75th, 95th)
  const std::vector<double> pct_levels = {0.05, 0.25, 0.50, 0.75, 0.95};
  for (double p : pct_levels) {
    size_t idx = static_cast<size_t>(p * n);
    idx = std::min(idx, n - 1);
    metrics.eens_percentiles.push_back(sorted_eens[idx]);
    metrics.lole_percentiles.push_back(sorted_lole[idx]);
  }
  
  return metrics;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Frequency & Duration Method (Analytical)
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

FrequencyDurationResult run_frequency_duration_analysis(
    const HybridPowerSystem& sys,
    double peak_load_mw) {
  
  FrequencyDurationResult result;
  
  // Calculate peak load if not provided
  if (peak_load_mw <= 0.0) {
    for (const auto& bus : sys.ac.buses) {
      peak_load_mw += bus.pd_mw;
    }
    for (const auto& ld : sys.ac.loads) {
      if (ld.in_service) peak_load_mw += ld.p_mw * ld.scaling;
    }
  }
  
  spdlog::info("F&D Analysis: Starting with peak load = {:.1f} MW", peak_load_mw);
  
  // Build list of generators with F&D parameters
  struct GenFD {
    double capacity;
    double lambda;  // failure rate (per hour)
    double mu;      // repair rate (per hour)
    double p;       // availability
    double q;       // unavailability (FOR)
  };
  
  std::vector<GenFD> gens;
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    
    GenFD gfd;
    gfd.capacity = g.pmax_mw;
    
    // Convert FOR and MTTR to lambda and mu
    // FOR = q = MTTR / (MTTF + MTTR) = lambda / (lambda + mu)
    // lambda = FOR / MTTR (per hour), mu = 1 / MTTR
    double for_rate = g.forced_outage_rate;
    double mttr_hr = g.mttr_hr > 0 ? g.mttr_hr : 50.0;  // default 50 hrs
    
    if (for_rate > 0 && for_rate < 1.0) {
      gfd.mu = 1.0 / mttr_hr;
      gfd.q = for_rate;
      gfd.p = 1.0 - for_rate;
      // lambda = q * mu / (1 - q) = FOR / ((1 - FOR) * MTTR)
      gfd.lambda = for_rate / ((1.0 - for_rate) * mttr_hr);
    } else {
      // Perfect generator
      gfd.lambda = 0.0;
      gfd.mu = 1.0;
      gfd.p = 1.0;
      gfd.q = 0.0;
    }
    
    gens.push_back(gfd);
  }
  
  if (gens.empty()) {
    spdlog::warn("F&D Analysis: No generators found");
    return result;
  }
  
  // Build COPT recursively using convolution
  // Start with "no outage" state: P(0) = 1, F(0) = 0
  double step_size = 10.0;  // MW resolution
  double total_capacity = 0.0;
  for (const auto& g : gens) total_capacity += g.capacity;
  
  int n_levels = static_cast<int>(total_capacity / step_size) + 1;
  std::vector<double> outage_levels(n_levels);
  std::vector<double> cum_prob(n_levels, 0.0);
  std::vector<double> cum_freq(n_levels, 0.0);
  
  for (int i = 0; i < n_levels; ++i) {
    outage_levels[i] = i * step_size;
  }
  cum_prob[0] = 1.0;  // P(outage >= 0) = 1
  
  // Add each generator using recursive formula
  for (const auto& g : gens) {
    int c_steps = static_cast<int>(g.capacity / step_size);
    if (c_steps <= 0) continue;
    
    std::vector<double> new_prob(n_levels, 0.0);
    std::vector<double> new_freq(n_levels, 0.0);
    
    for (int x = 0; x < n_levels; ++x) {
      // Get old values at x and x - C
      double p_old_x = cum_prob[x];
      double f_old_x = cum_freq[x];
      double p_old_xc = (x >= c_steps) ? cum_prob[x - c_steps] : 1.0;
      double f_old_xc = (x >= c_steps) ? cum_freq[x - c_steps] : 0.0;
      
      // Probability recursion: P_new(X) = p * P_old(X) + q * P_old(X - C)
      new_prob[x] = g.p * p_old_x + g.q * p_old_xc;
      
      // Frequency recursion: F_new(X) = p * F_old(X) + q * F_old(X-C) 
      //                               + lambda * p * [P_old(X-C) - P_old(X)]
      double term1 = g.p * f_old_x;
      double term2 = g.q * f_old_xc;
      double term3 = g.lambda * g.p * (p_old_xc - p_old_x);
      new_freq[x] = term1 + term2 + term3;
    }
    
    cum_prob = new_prob;
    cum_freq = new_freq;
  }
  
  // Store COPT
  result.capacity_outage_levels = outage_levels;
  result.cumulative_probability = cum_prob;
  result.cumulative_frequency = cum_freq;
  
  // Calculate reliability indices at reserve margin
  double reserve = total_capacity - peak_load_mw;
  int reserve_idx = static_cast<int>(reserve / step_size);
  reserve_idx = std::max(0, std::min(reserve_idx, n_levels - 1));
  
  // LOLP = P(outage > reserve) = P(outage >= reserve + step)
  int lolp_idx = reserve_idx + 1;
  result.lolp = (lolp_idx < n_levels) ? cum_prob[lolp_idx] : 0.0;
  
  // LOLF = F(outage > reserve)
  result.lolf_fd = (lolp_idx < n_levels) ? cum_freq[lolp_idx] * 8760.0 : 0.0;  // per year
  
  // LOLE = LOLP * 8760 (simplified, assumes constant load)
  result.lole_fd = result.lolp * 8760.0;
  
  // LOLD = LOLE / LOLF (average duration of loss event)
  result.lold = (result.lolf_fd > 0) ? result.lole_fd / result.lolf_fd : 0.0;
  
  spdlog::info("F&D Analysis: Complete. LOLP={:.6f}, LOLE={:.2f} hr/yr, LOLF={:.2f} occ/yr, LOLD={:.2f} hr/occ",
               result.lolp, result.lole_fd, result.lolf_fd, result.lold);
  
  return result;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Distribution System Reliability Indices
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

DistributionIndices compute_distribution_indices(
    const HybridPowerSystem& sys,
    const std::vector<double>& nodal_cif,
    const std::vector<double>& nodal_cid,
    int hours_per_year) {
  
  DistributionIndices idx;
  
  // Get customer counts from loads
  // Note: In our model, we use load.p_mw as a proxy for customers if num_customers not available
  std::vector<double> customers_per_bus(sys.ac.buses.size(), 0.0);
  double total_customers = 0.0;
  
  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service || ld.bus < 1) continue;
    size_t bus_idx = static_cast<size_t>(ld.bus - 1);
    if (bus_idx < customers_per_bus.size()) {
      // Use n_customers if available, otherwise estimate from load
      double nc = ld.n_customers > 0 ? ld.n_customers : std::max(1.0, ld.p_mw * 10.0);
      customers_per_bus[bus_idx] += nc;
      total_customers += nc;
    }
  }
  
  // For buses without explicit loads, add customers based on pd_mw
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (customers_per_bus[i] == 0.0 && sys.ac.buses[i].pd_mw > 0) {
      double nc = std::max(1.0, sys.ac.buses[i].pd_mw * 10.0);  // ~10 customers per MW
      customers_per_bus[i] = nc;
      total_customers += nc;
    }
  }
  
  if (total_customers < 1.0) {
    spdlog::warn("Distribution indices: No customers found");
    return idx;
  }
  
  // Copy nodal indices
  idx.nodal_cif = nodal_cif;
  idx.nodal_cid = nodal_cid;
  
  // Calculate system indices
  // SAIFI = 危(CIF_i * N_i) / 危(N_i)
  // SAIDI = 危(CID_i * N_i) / 危(N_i)
  double sum_cif_n = 0.0;
  double sum_cid_n = 0.0;
  
  size_t n_buses = std::min({nodal_cif.size(), nodal_cid.size(), customers_per_bus.size()});
  for (size_t i = 0; i < n_buses; ++i) {
    sum_cif_n += nodal_cif[i] * customers_per_bus[i];
    sum_cid_n += nodal_cid[i] * customers_per_bus[i];
  }
  
  idx.saifi = sum_cif_n / total_customers;  // interruptions per customer per year
  idx.saidi = sum_cid_n / total_customers;  // hours per customer per year
  
  // CAIDI = SAIDI / SAIFI (average duration per interruption)
  idx.caidi = (idx.saifi > 0) ? idx.saidi / idx.saifi : 0.0;
  
  // ASAI = 1 - SAIDI / (hours_per_year)
  idx.asai = 1.0 - (idx.saidi / hours_per_year);
  idx.asui = 1.0 - idx.asai;
  
  spdlog::info("Distribution indices: SAIFI={:.4f} int/cust/yr, SAIDI={:.4f} hr/cust/yr, ASAI={:.6f}",
               idx.saifi, idx.saidi, idx.asai);
  
  return idx;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// FMEA: Deterministic N-1 Failure-Mode Enumeration
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

namespace {

// Represent a single component eligible for N-1 contingency.
struct FMEAComponent {
  enum Type { Generator, ACBranch, DCBranch, VSCConverter,
              StaticGen, RenewableGen, Storage, Transformer2W, Transformer3W,
              DCDCConv, DCCB, DCStorage, DCPVArray, DCStaticGen,
              ACSwitch, ACCB };
  Type type = Generator;
  int idx;              // Index into corresponding vector
  std::string name;     // Human-readable label
  double lambda;        // Failure rate (occ/yr)
  double tau_sw_hr;     // Switching/isolation duration (hours)
  double tau_rep_hr;    // Repair duration (hours)
};

std::string fmea_component_type_name(FMEAComponent::Type type) {
  switch (type) {
    case FMEAComponent::Generator:     return "generator";
    case FMEAComponent::ACBranch:      return "ac_branch";
    case FMEAComponent::DCBranch:      return "dc_branch";
    case FMEAComponent::VSCConverter:  return "vsc_converter";
    case FMEAComponent::StaticGen:     return "static_generator";
    case FMEAComponent::RenewableGen:  return "renewable_gen";
    case FMEAComponent::Storage:       return "storage";
    case FMEAComponent::Transformer2W: return "transformer_2w";
    case FMEAComponent::Transformer3W: return "transformer_3w";
    case FMEAComponent::DCDCConv:      return "dcdc_converter";
    case FMEAComponent::DCCB:          return "dc_circuit_breaker";
    case FMEAComponent::DCStorage:     return "dc_storage";
    case FMEAComponent::DCPVArray:     return "dc_pv_array";
    case FMEAComponent::DCStaticGen:   return "dc_static_generator";
    case FMEAComponent::ACSwitch:      return "ac_switch";
    case FMEAComponent::ACCB:          return "ac_circuit_breaker";
  }
  return "unknown";
}

std::string switch_type_name(SwitchType type) {
  switch (type) {
    case SwitchType::CircuitBreaker: return "circuit_breaker";
    case SwitchType::Disconnector: return "disconnector";
    case SwitchType::LoadBreakSwitch: return "load_break_switch";
    case SwitchType::Fuse: return "fuse";
    case SwitchType::Recloser: return "recloser";
    case SwitchType::Sectionalizer: return "sectionalizer";
  }
  return "switch";
}

// Build the component catalog from the system.
std::vector<FMEAComponent> build_fmea_catalog(
    const HybridPowerSystem& sys,
    double default_sw_hr) {
  
  std::vector<FMEAComponent> catalog;

  // ---- AC Generators ----
  for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
    const auto& g = sys.ac.generators[i];
    if (!g.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::Generator;
    c.idx = static_cast<int>(i);
    c.name = g.name.empty() ? "Gen_" + std::to_string(g.index) : g.name;
    
    // Derive lambda from FOR and MTTR
    double mttr = g.mttr_hr > 0 ? g.mttr_hr : 50.0;
    if (g.forced_outage_rate > 0 && g.forced_outage_rate < 1.0) {
      c.lambda = g.forced_outage_rate / ((1.0 - g.forced_outage_rate) * mttr) * 8760.0;
    } else {
      c.lambda = 8760.0 / 2000.0;   // default ~4.38 occ/yr
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- AC Branches ----
  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    const auto& b = sys.ac.branches[i];
    if (!b.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::ACBranch;
    c.idx = static_cast<int>(i);
    c.name = b.name.empty() ?
        "ACBr_" + std::to_string(b.from_bus) + "-" + std::to_string(b.to_bus) : b.name;
    
    c.lambda = b.failure_rate > 0 ? b.failure_rate : 0.35;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = b.mttr_hr > 0 ? b.mttr_hr : 10.0;
    catalog.push_back(c);
  }

  // ---- DC Branches ----
  for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
    const auto& b = sys.dc.branches[i];
    if (!b.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::DCBranch;
    c.idx = static_cast<int>(i);
    c.name = b.name.empty() ?
        "DCBr_" + std::to_string(b.from_bus) + "-" + std::to_string(b.to_bus) : b.name;
    
    double mttr = b.mttr_hours > 0 ? b.mttr_hours : 24.0;
    if (b.mtbf_hours > 0) {
      c.lambda = 8760.0 / b.mtbf_hours;
    } else {
      c.lambda = 0.20;
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- VSC Converters ----
  for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
    const auto& v = sys.vsc_converters[i];
    if (!v.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::VSCConverter;
    c.idx = static_cast<int>(i);
    c.name = v.name.empty() ? "VSC_" + std::to_string(v.index) : v.name;
    
    double mttr = v.mttr_hr > 0 ? v.mttr_hr : 48.0;
    if (v.forced_outage_rate > 0 && v.forced_outage_rate < 1.0) {
      c.lambda = v.forced_outage_rate / ((1.0 - v.forced_outage_rate) * mttr) * 8760.0;
    } else {
      c.lambda = 0.10;
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- Static Generators ----
  for (size_t i = 0; i < sys.ac.static_generators.size(); ++i) {
    const auto& sg = sys.ac.static_generators[i];
    if (!sg.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::StaticGen;
    c.idx = static_cast<int>(i);
    c.name = sg.name.empty() ? "SGen_" + std::to_string(sg.index) : sg.name;
    
    double mttr = sg.mttr_hours > 0 ? sg.mttr_hours : 24.0;
    if (sg.mtbf_hours > 0) {
      c.lambda = 8760.0 / sg.mtbf_hours;
    } else {
      c.lambda = 1.5;  // typical DG ~1.5 occ/yr
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- Renewable Generators ----
  for (size_t i = 0; i < sys.ac.renewable_gens.size(); ++i) {
    const auto& rg = sys.ac.renewable_gens[i];
    if (!rg.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::RenewableGen;
    c.idx = static_cast<int>(i);
    c.name = rg.name.empty() ? "RGen_" + std::to_string(rg.index) : rg.name;
    
    double mttr = rg.mttr_hours > 0 ? rg.mttr_hours : 48.0;
    if (rg.mtbf_hours > 0) {
      c.lambda = 8760.0 / rg.mtbf_hours;
    } else {
      c.lambda = 2.0;  // typical wind ~2 occ/yr
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- Storage Units ----
  for (size_t i = 0; i < sys.ac.storage.size(); ++i) {
    const auto& st = sys.ac.storage[i];
    if (!st.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::Storage;
    c.idx = static_cast<int>(i);
    c.name = st.name.empty() ? "BESS_" + std::to_string(st.index) : st.name;
    
    double mttr = st.mttr_hr > 0 ? st.mttr_hr : 24.0;
    if (st.forced_outage_rate > 0 && st.forced_outage_rate < 1.0) {
      c.lambda = st.forced_outage_rate / ((1.0 - st.forced_outage_rate) * mttr) * 8760.0;
    } else {
      c.lambda = 1.0;  // typical BESS ~1 occ/yr
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- Transformers 2W ----
  for (size_t i = 0; i < sys.ac.transformers_2w.size(); ++i) {
    const auto& t = sys.ac.transformers_2w[i];
    if (!t.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::Transformer2W;
    c.idx = static_cast<int>(i);
    c.name = t.name.empty() ? "Trafo2W_" + std::to_string(t.index) : t.name;
    
    double mttr = t.mttr_hours > 0 ? t.mttr_hours : 200.0;
    if (t.mtbf_hours > 0) {
      c.lambda = 8760.0 / t.mtbf_hours;
    } else {
      c.lambda = 0.03;  // typical power transformer ~0.03 occ/yr
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- Transformers 3W ----
  for (size_t i = 0; i < sys.ac.transformers_3w.size(); ++i) {
    const auto& t = sys.ac.transformers_3w[i];
    if (!t.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::Transformer3W;
    c.idx = static_cast<int>(i);
    c.name = t.name.empty() ? "Trafo3W_" + std::to_string(t.index) : t.name;
    
    double mttr = t.mttr_hours > 0 ? t.mttr_hours : 200.0;
    if (t.mtbf_hours > 0) {
      c.lambda = 8760.0 / t.mtbf_hours;
    } else {
      c.lambda = 0.04;  // slightly higher than 2W
    }
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- DC-DC Converters ----
  for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
    const auto& d = sys.dc.dcdc_converters[i];
    if (!d.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::DCDCConv;
    c.idx  = static_cast<int>(i);
    c.name = d.name.empty() ? "DCDC_" + std::to_string(d.index) : d.name;
    double mttr = d.mttr_hours > 0 ? d.mttr_hours : 48.0;
    c.lambda = d.mtbf_hours > 0 ? 8760.0 / d.mtbf_hours : 0.20;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- DC Circuit Breakers ----
  for (size_t i = 0; i < sys.dc.dc_circuit_breakers.size(); ++i) {
    const auto& cb = sys.dc.dc_circuit_breakers[i];
    if (!cb.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::DCCB;
    c.idx  = static_cast<int>(i);
    c.name = cb.name.empty() ? "DCCB_" + std::to_string(cb.index) : cb.name;
    // DCCircuitBreaker has no mtbf/mttr fields; use typical CB defaults.
    c.lambda     = 0.10;   // ~0.1 failures/yr
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = 8.0;    // 8-hr typical repair
    catalog.push_back(c);
  }

  // ---- DC Storage ----
  for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
    const auto& st = sys.dc.storage[i];
    if (!st.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::DCStorage;
    c.idx  = static_cast<int>(i);
    c.name = st.name.empty() ? "DCStorage_" + std::to_string(st.index) : st.name;
    double mttr = st.mttr_hr > 0 ? st.mttr_hr : 24.0;
    if (st.forced_outage_rate > 0 && st.forced_outage_rate < 1.0)
      c.lambda = st.forced_outage_rate / ((1.0 - st.forced_outage_rate) * mttr) * 8760.0;
    else
      c.lambda = 1.0;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- DC PV Arrays ----
  for (size_t i = 0; i < sys.dc.pv_arrays.size(); ++i) {
    const auto& pv = sys.dc.pv_arrays[i];
    if (!pv.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::DCPVArray;
    c.idx  = static_cast<int>(i);
    c.name = pv.name.empty() ? "DCPV_" + std::to_string(pv.index) : pv.name;
    double mttr = pv.mttr_hours > 0 ? pv.mttr_hours : 24.0;
    c.lambda = pv.mtbf_hours > 0 ? 8760.0 / pv.mtbf_hours : 1.5;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- DC Static Generators ----
  for (size_t i = 0; i < sys.dc.dc_static_generators.size(); ++i) {
    const auto& sg = sys.dc.dc_static_generators[i];
    if (!sg.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::DCStaticGen;
    c.idx  = static_cast<int>(i);
    c.name = sg.name.empty() ? "DCSGen_" + std::to_string(sg.index) : sg.name;
    double mttr = sg.mttr_hours > 0 ? sg.mttr_hours : 24.0;
    c.lambda = sg.mtbf_hours > 0 ? 8760.0 / sg.mtbf_hours : 1.5;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- AC Switches (protection device failure → section de-energised) ----
  for (size_t i = 0; i < sys.ac.switches.size(); ++i) {
    const auto& sw = sys.ac.switches[i];
    if (!sw.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::ACSwitch;
    c.idx  = static_cast<int>(i);
    c.name = sw.name.empty() ? "SW_" + std::to_string(sw.index) : sw.name;
    double mttr = sw.mttr_hours > 0 ? sw.mttr_hours : 4.0;
    // p_sw_fail as annual failure probability if set, else use MTBF.
    if (sw.mtbf_hours > 0)
      c.lambda = 8760.0 / sw.mtbf_hours;
    else if (sw.p_sw_fail > 0 && sw.p_sw_fail < 1.0)
      c.lambda = sw.p_sw_fail;   // already in occ/yr convention
    else
      c.lambda = 0.05;  // typical distribution switch ~0.05 failures/yr
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = mttr;
    catalog.push_back(c);
  }

  // ---- AC Circuit Breakers ----
  for (size_t i = 0; i < sys.ac.circuit_breakers.size(); ++i) {
    const auto& cb = sys.ac.circuit_breakers[i];
    if (!cb.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::ACCB;
    c.idx  = static_cast<int>(i);
    c.name = cb.name.empty() ? "CB_" + std::to_string(cb.index) : cb.name;
    // CircuitBreaker has no mtbf/mttr; use utility-typical CB defaults.
    c.lambda     = 0.05;   // ~0.05 failures/yr
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = 8.0;
    catalog.push_back(c);
  }

  return catalog;
}

struct FMEAStageEval {
  StateEvalResult eval;
  std::vector<FMEAContingencyDetail::SwitchActionDetail> switch_actions;
  bool search_truncated{false};  ///< true if OPF call budget was exhausted
};

void apply_fmea_component_outage(HybridPowerSystem& sys, const FMEAComponent& comp) {
  switch (comp.type) {
    case FMEAComponent::Generator:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.generators.size()))
        sys.ac.generators[comp.idx].in_service = false;
      break;
    case FMEAComponent::ACBranch:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.branches.size()))
        sys.ac.branches[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCBranch:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.dc.branches.size()))
        sys.dc.branches[comp.idx].in_service = false;
      break;
    case FMEAComponent::VSCConverter:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.vsc_converters.size()))
        sys.vsc_converters[comp.idx].in_service = false;
      break;
    case FMEAComponent::StaticGen:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.static_generators.size()))
        sys.ac.static_generators[comp.idx].in_service = false;
      break;
    case FMEAComponent::RenewableGen:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.renewable_gens.size()))
        sys.ac.renewable_gens[comp.idx].in_service = false;
      break;
    case FMEAComponent::Storage:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.storage.size()))
        sys.ac.storage[comp.idx].in_service = false;
      break;
    case FMEAComponent::Transformer2W:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.transformers_2w.size()))
        sys.ac.transformers_2w[comp.idx].in_service = false;
      break;
    case FMEAComponent::Transformer3W:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.transformers_3w.size()))
        sys.ac.transformers_3w[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCDCConv:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.dc.dcdc_converters.size()))
        sys.dc.dcdc_converters[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCCB:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.dc.dc_circuit_breakers.size()))
        sys.dc.dc_circuit_breakers[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCStorage:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.dc.storage.size()))
        sys.dc.storage[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCPVArray:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.dc.pv_arrays.size()))
        sys.dc.pv_arrays[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCStaticGen:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.dc.dc_static_generators.size()))
        sys.dc.dc_static_generators[comp.idx].in_service = false;
      break;
    case FMEAComponent::ACSwitch:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.switches.size()))
        sys.ac.switches[comp.idx].in_service = false;
      break;
    case FMEAComponent::ACCB:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.circuit_breakers.size()))
        sys.ac.circuit_breakers[comp.idx].in_service = false;
      break;
  }
}

void scale_fmea_loads(HybridPowerSystem& sys, double load_scale) {
  if (std::abs(load_scale - 1.0) <= 1e-9) return;
  for (auto& bus : sys.ac.buses) {
    bus.pd_mw *= load_scale;
    bus.qd_mvar *= load_scale;
  }
  for (auto& ld : sys.ac.loads) {
    ld.p_mw *= load_scale;
    ld.q_mvar *= load_scale;
  }
  for (auto& ld : sys.dc.loads) {
    ld.p_mw *= load_scale;
  }
}

double storage_available_mw(const Storage& st, double stage_duration_hr) {
  const double p_rating = st.pmax_mw > 0.0 ? st.pmax_mw : st.p_rated_mw;
  const double e_now = st.e_mwh > 0.0 ? st.e_mwh : st.e_rated_mwh * st.soc_init;
  const double e_min = st.e_rated_mwh > 0.0 ? st.e_rated_mwh * st.soc_min : 0.0;
  const double usable = std::max(0.0, e_now - e_min) * std::max(1e-6, st.eta_discharge);
  const double energy_limited = usable / std::max(stage_duration_hr, 1e-6);
  return std::max(0.0, std::min(p_rating, energy_limited));
}

int next_generator_index(const HybridPowerSystem& sys) {
  int next = 1;
  for (const auto& g : sys.ac.generators) next = std::max(next, g.index + 1);
  return next;
}

bool bus_in_microgrid(const HybridPowerSystem& sys, int bus) {
  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service || !mg.islanding_capability) continue;
    if (mg.pcc_bus == bus) return true;
    if (std::find(mg.internal_buses.begin(), mg.internal_buses.end(), bus) !=
        mg.internal_buses.end()) {
      return true;
    }
  }
  return false;
}

void add_emergency_generator(
    HybridPowerSystem& sys,
    int bus,
    double pmax_mw,
    const std::string& name,
    int& next_index,
    double marginal_cost) {
  if (bus <= 0 || pmax_mw <= 1e-9) return;
  Generator g;
  g.index = next_index++;
  g.bus = bus;
  g.in_service = true;
  g.name = name;
  g.pg_mw = 0.0;
  g.pmin_mw = 0.0;
  g.pmax_mw = pmax_mw;
  g.qmin_mvar = -pmax_mw;
  g.qmax_mvar = pmax_mw;
  g.cost_c1 = marginal_cost;
  g.cost_c2 = 0.0;
  g.is_slack = false;
  sys.ac.generators.push_back(std::move(g));
}

void mark_island_anchor(HybridPowerSystem& sys, int bus, int index, const std::string& name) {
  if (bus <= 0) return;
  for (auto& b : sys.ac.buses) {
    if (b.index == bus) {
      b.bus_type = BusType::SLACK;
      break;
    }
  }
  ExternalGrid eg;
  eg.index = index;
  eg.bus = bus;
  eg.in_service = true;
  eg.name = name;
  eg.vm_pu = 1.0;
  eg.va_deg = 0.0;
  sys.ac.external_grids.push_back(std::move(eg));
}

int next_external_grid_index(const HybridPowerSystem& sys) {
  int next = 1;
  for (const auto& eg : sys.ac.external_grids) next = std::max(next, eg.index + 1);
  return next;
}

double total_positive_ac_load_mw(const HybridPowerSystem& sys) {
  double demand = 0.0;
  for (const auto& b : sys.ac.buses) demand += std::max(0.0, b.pd_mw);
  for (const auto& ld : sys.ac.loads) {
    if (ld.in_service) demand += std::max(0.0, ld.p_mw * ld.scaling);
  }
  return demand;
}

void add_external_grid_dispatch_sources(HybridPowerSystem& sys, double marginal_cost) {
  int next_gen = next_generator_index(sys);
  const double demand = total_positive_ac_load_mw(sys);
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service || eg.bus <= 0) continue;
    double pmax = eg.s_sc_max_mva > 0.0 ? eg.s_sc_max_mva : std::max(1000.0, demand * 2.0);
    if (pmax <= 1e-9) pmax = 1000.0;
    Generator g;
    g.index = next_gen++;
    g.bus = eg.bus;
    g.in_service = true;
    g.name = eg.name.empty() ? "FMEA_external_grid_" + std::to_string(eg.index)
                             : "FMEA_" + eg.name;
    g.pg_mw = 0.0;
    g.pmin_mw = 0.0;
    g.pmax_mw = pmax;
    g.qmin_mvar = -pmax;
    g.qmax_mvar = pmax;
    g.cost_c1 = marginal_cost;
    g.is_slack = true;
    sys.ac.generators.push_back(std::move(g));
  }
}

void apply_fmea_support_sources(
    HybridPowerSystem& sys,
    const FMEAOptions& options,
    double stage_duration_hr,
    bool repair_stage) {
  int next_gen = next_generator_index(sys);
  int next_eg = next_external_grid_index(sys);
  const double support_cost = std::max(1.0, options.voll * 0.01);

  if (options.enable_storage_dispatch) {
    for (auto& st : sys.ac.storage) {
      if (!st.in_service || !st.controllable) continue;
      const double p = storage_available_mw(st, stage_duration_hr);
      if (p <= 1e-9) continue;
      st.p_mw = std::max(st.p_mw, p);
      const bool black_start_ok = options.enable_black_start_storage &&
                                  st.grid_forming &&
                                  bus_in_microgrid(sys, st.bus);
      if (black_start_ok) {
        add_emergency_generator(
            sys, st.bus, p, "FMEA_black_start_storage_" + std::to_string(st.index),
            next_gen, support_cost);
        mark_island_anchor(
            sys, st.bus, next_eg++,
            "FMEA_black_start_storage_grid_" + std::to_string(st.index));
      }
    }
  }

  if (repair_stage && options.enable_repair_reconfiguration) {
    for (auto& sg : sys.ac.static_generators) {
      if (!sg.in_service || !sg.controllable) continue;
      double p = sg.pmax_mw > 0.0 ? sg.pmax_mw : sg.p_rated_mw;
      if (p <= 1e-9) continue;
      sg.p_mw = 0.0;
      add_emergency_generator(
          sys, sg.bus, p, "FMEA_controllable_sgen_" + std::to_string(sg.index),
          next_gen, support_cost);
    }
  }

  if (options.enable_grid_forming_vsc_support) {
    for (const auto& vsc : sys.vsc_converters) {
      if (!vsc.in_service || !vsc.grid_forming) continue;
      double p = vsc.pmax_mw > 0.0 ? vsc.pmax_mw : vsc.p_rated_mw;
      if (p <= 1e-9 && std::abs(vsc.p_set_mw) > 1e-9) p = std::abs(vsc.p_set_mw);
      add_emergency_generator(
          sys, vsc.bus_ac, p, "FMEA_grid_forming_vsc_" + std::to_string(vsc.index),
          next_gen, support_cost);
      mark_island_anchor(
          sys, vsc.bus_ac, next_eg++,
          "FMEA_grid_forming_vsc_grid_" + std::to_string(vsc.index));
    }
  }

  if (options.enable_microgrid_islanding) {
    for (const auto& mg : sys.microgrids) {
      if (!mg.in_service || !mg.islanding_capability) continue;
      double p = mg.capacity_mw;
      if (p <= 1e-9) p = mg.total_generation_mw;
      if (p <= 1e-9) p = mg.total_dg_capacity_mw + mg.total_diesel_capacity_mw;
      if (p <= 1e-9) p = mg.p_exchange_max_mw;
      add_emergency_generator(
          sys, mg.pcc_bus, p, "FMEA_microgrid_" + std::to_string(mg.index),
          next_gen, support_cost);
      mark_island_anchor(
          sys, mg.pcc_bus, next_eg++,
          "FMEA_microgrid_grid_" + std::to_string(mg.index));
    }
  }
}

std::vector<int> repair_switch_candidates(const HybridPowerSystem& sys) {
  std::vector<int> candidates;
  for (int i = 0; i < static_cast<int>(sys.ac.switches.size()); ++i) {
    const auto& sw = sys.ac.switches[i];
    if (!sw.in_service || sw.closed) continue;
    if (!sw.is_automated && sw.t_operation_s <= 0.0) continue;
    candidates.push_back(i);
  }
  return candidates;
}

std::vector<int> repair_branch_candidates(const HybridPowerSystem& sys) {
  std::vector<int> candidates;
  for (int i = 0; i < static_cast<int>(sys.ac.branches.size()); ++i) {
    const auto& br = sys.ac.branches[i];
    if (br.in_service) continue;
    candidates.push_back(i);
  }
  return candidates;
}

void close_repair_actions(HybridPowerSystem& sys,
                          const std::vector<int>& switch_indices,
                          const std::vector<int>& branch_indices) {
  for (int i : switch_indices) {
    if (i >= 0 && i < static_cast<int>(sys.ac.switches.size())) {
      sys.ac.switches[i].closed = true;
      sys.ac.switches[i].in_service = true;
    }
  }
  for (int i : branch_indices) {
    if (i >= 0 && i < static_cast<int>(sys.ac.branches.size())) {
      sys.ac.branches[i].in_service = true;
    }
  }
}

std::vector<FMEAContingencyDetail::SwitchActionDetail> describe_repair_actions(
    const HybridPowerSystem& sys,
    const std::vector<int>& switch_indices,
    const std::vector<int>& branch_indices) {
  std::vector<FMEAContingencyDetail::SwitchActionDetail> out;
  out.reserve(switch_indices.size() + branch_indices.size());
  for (int i : switch_indices) {
    if (i < 0 || i >= static_cast<int>(sys.ac.switches.size())) continue;
    const auto& sw = sys.ac.switches[i];
    FMEAContingencyDetail::SwitchActionDetail a;
    a.switch_index = i;
    a.switch_name = sw.name.empty() ? "switch_" + std::to_string(sw.index) : sw.name;
    a.switch_type = switch_type_name(sw.switch_type);
    a.action = "close";
    a.bus_from = sw.bus_from;
    a.bus_to = sw.bus_to;
    out.push_back(std::move(a));
  }
  for (int i : branch_indices) {
    if (i < 0 || i >= static_cast<int>(sys.ac.branches.size())) continue;
    const auto& br = sys.ac.branches[i];
    FMEAContingencyDetail::SwitchActionDetail a;
    a.switch_index = i;
    a.switch_name = br.name.empty() ? "branch_" + std::to_string(br.index) : br.name;
    a.switch_type = "normally_open_branch";
    a.action = "close";
    a.bus_from = br.from_bus;
    a.bus_to = br.to_bus;
    out.push_back(std::move(a));
  }
  return out;
}

StateEvalResult evaluate_prepared_fmea_system(
    const HybridPowerSystem& sys,
    const opf::DCOPFOptions& opf_opt) {
  std::vector<bool> no_failures(ComponentOffsets(sys).total, false);
  return evaluate_state(sys, no_failures, opf_opt, 1.0);
}

FMEAStageEval evaluate_contingency_stage(
    const HybridPowerSystem& sys,
    const FMEAComponent& comp,
    const FMEAOptions& options,
    const opf::DCOPFOptions& opf_opt,
    double stage_duration_hr,
    bool repair_stage) {
  
  HybridPowerSystem sys_copy = sys;
  scale_fmea_loads(sys_copy, options.load_scale_factor);
  apply_fmea_component_outage(sys_copy, comp);
  add_external_grid_dispatch_sources(sys_copy, 1.0);
  apply_fmea_support_sources(sys_copy, options, stage_duration_hr, repair_stage);

  FMEAStageEval best;
  best.eval = evaluate_prepared_fmea_system(sys_copy, opf_opt);

  if (!repair_stage || !options.enable_switch_reconfiguration ||
      options.max_repair_switch_actions <= 0) {
    return best;
  }

  std::vector<int> switch_candidates = repair_switch_candidates(sys_copy);
  std::vector<int> branch_candidates = repair_branch_candidates(sys_copy);
  if (comp.type == FMEAComponent::ACBranch) {
    branch_candidates.erase(
        std::remove(branch_candidates.begin(), branch_candidates.end(), comp.idx),
        branch_candidates.end());
  }
  const int max_actions = std::max(0, options.max_repair_switch_actions);
  // Budget cap: 0 means unlimited.
  const int opf_budget = std::max(0, options.max_repair_opf_calls);
  int opf_calls = 1;  // baseline evaluation already counted

  auto budget_exhausted = [&]() {
    return opf_budget > 0 && opf_calls >= opf_budget;
  };

  auto try_candidate = [&](const std::vector<int>& switches,
                           const std::vector<int>& branches) {
    if (budget_exhausted()) return;
    HybridPowerSystem cand = sys_copy;
    close_repair_actions(cand, switches, branches);
    StateEvalResult ev = evaluate_prepared_fmea_system(cand, opf_opt);
    ++opf_calls;
    if (ev.curtailment_mw + 1e-9 < best.eval.curtailment_mw) {
      best.eval = std::move(ev);
      best.switch_actions = describe_repair_actions(sys_copy, switches, branches);
    }
  };

  for (int s : switch_candidates) {
    if (budget_exhausted()) break;
    try_candidate({s}, {});
    // Early exit: curtailment already zeroed, no need to continue.
    if (best.eval.curtailment_mw <= 1e-9) { return best; }
  }
  for (int b : branch_candidates) {
    if (budget_exhausted()) break;
    try_candidate({}, {b});
    if (best.eval.curtailment_mw <= 1e-9) { return best; }
  }

  if (max_actions >= 2) {
    for (size_t i = 0; i < switch_candidates.size() && !budget_exhausted(); ++i) {
      for (size_t j = i + 1; j < switch_candidates.size() && !budget_exhausted(); ++j) {
        try_candidate({switch_candidates[i], switch_candidates[j]}, {});
        if (best.eval.curtailment_mw <= 1e-9) { return best; }
      }
    }
    for (int s : switch_candidates) {
      if (budget_exhausted()) break;
      for (int b : branch_candidates) {
        if (budget_exhausted()) break;
        try_candidate({s}, {b});
        if (best.eval.curtailment_mw <= 1e-9) { return best; }
      }
    }
    for (size_t i = 0; i < branch_candidates.size() && !budget_exhausted(); ++i) {
      for (size_t j = i + 1; j < branch_candidates.size() && !budget_exhausted(); ++j) {
        try_candidate({}, {branch_candidates[i], branch_candidates[j]});
        if (best.eval.curtailment_mw <= 1e-9) { return best; }
      }
    }
  }

  best.search_truncated = budget_exhausted();
  return best;
}

}  // anonymous namespace

FMEAResult run_distribution_fmea(
    const HybridPowerSystem& sys,
    const FMEAOptions& options) {
  
  spdlog::info("FMEA: Starting N-1 failure-mode enumeration");
  
  FMEAResult result;
  {
    double dc_mw = total_inservice_dc_load_mw(sys);
    if (dc_mw > 0.01)
      result.model_limitations = "DC loads not modelled in OPF (" +
          std::to_string(dc_mw) + " MW unaccounted)";
  }
  const size_t nb = sys.ac.buses.size();
  result.nodal_eens_mwh_yr.resize(nb, 0.0);

  // Build component catalog
  auto catalog = build_fmea_catalog(sys, options.switching_time_hr);
  result.n_contingencies = static_cast<int>(catalog.size());
  
  spdlog::info("FMEA: {} components in catalog (generators={}, ac_branches={}, dc_branches={}, vsc={})",
               catalog.size(),
               sys.ac.generators.size(), sys.ac.branches.size(),
               sys.dc.branches.size(), sys.vsc_converters.size());

  // DC-OPF options
  opf::DCOPFOptions opf_opt = options.opf_options;
  opf_opt.load_shedding = true;
  opf_opt.verbose = false;
  opf_opt.compute_lmp = false;  // batch path — LMPs not needed, skip supporting LP

  // Accumulators for SAIFI/SAIDI
  std::vector<double> nodal_cif(nb, 0.0);  // per-bus interruption frequency
  std::vector<double> nodal_cid(nb, 0.0);  // per-bus interruption duration

  // Enumerate N-1 contingencies
  // Suppress a bogus GCC -Wmaybe-uninitialized warning when tracking
  // `comp.type` from the catalog vector through this loop at -O2.
#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
  for (const auto& comp : catalog) {
    FMEAContingencyDetail detail;
    detail.component_index = comp.idx;
    detail.component_name = comp.name;
    detail.failure_rate = comp.lambda;
    detail.tau_sw_hr = comp.tau_sw_hr;
    detail.tau_rep_hr = comp.tau_rep_hr;
    detail.component_type = fmea_component_type_name(comp.type);

    // 鈹€鈹€ Switching stage 鈹€鈹€
    auto eval_sw = evaluate_contingency_stage(
        sys, comp, options, opf_opt, comp.tau_sw_hr, false);
    detail.shed_sw_mw = eval_sw.eval.curtailment_mw;
    detail.ens_sw_mwh = eval_sw.eval.curtailment_mw * comp.tau_sw_hr;
    detail.causes_loss_sw = eval_sw.eval.is_loss_state;
    detail.nodal_shed_sw_mw = eval_sw.eval.nodal_curtailment_mw;

    // 鈹€鈹€ Repair stage 鈹€鈹€
    auto eval_rep = evaluate_contingency_stage(
        sys, comp, options, opf_opt, comp.tau_rep_hr, true);
    detail.shed_rep_mw = eval_rep.eval.curtailment_mw;
    detail.ens_rep_mwh = eval_rep.eval.curtailment_mw * comp.tau_rep_hr;
    detail.causes_loss_rep = eval_rep.eval.is_loss_state;
    detail.nodal_shed_rep_mw = eval_rep.eval.nodal_curtailment_mw;
    detail.repair_switch_actions = std::move(eval_rep.switch_actions);
    detail.repair_search_truncated = eval_rep.search_truncated;

    // 鈹€鈹€ Frequency-weighted contributions 鈹€鈹€
    double total_ens = detail.ens_sw_mwh + detail.ens_rep_mwh;
    detail.eens_contribution = comp.lambda * total_ens;

    double loss_duration = 0.0;
    if (detail.causes_loss_sw) loss_duration += comp.tau_sw_hr;
    if (detail.causes_loss_rep) loss_duration += comp.tau_rep_hr;
    detail.lole_contribution = comp.lambda * loss_duration;

    // 鈹€鈹€ Aggregate system indices 鈹€鈹€
    result.eens_mwh_yr += detail.eens_contribution;
    result.lole_hr_yr += detail.lole_contribution;
    if (detail.causes_loss_sw || detail.causes_loss_rep) {
      result.lolf_occ_yr += comp.lambda;
      result.n_loss_contingencies++;
    }

    // 鈹€鈹€ Nodal EENS accumulation 鈹€鈹€
    for (size_t b = 0; b < nb; ++b) {
      double shed_sw = b < detail.nodal_shed_sw_mw.size()
                           ? detail.nodal_shed_sw_mw[b]
                           : 0.0;
      double shed_rep = b < detail.nodal_shed_rep_mw.size()
                            ? detail.nodal_shed_rep_mw[b]
                            : 0.0;
      double nodal_ens = shed_sw * comp.tau_sw_hr + shed_rep * comp.tau_rep_hr;
      result.nodal_eens_mwh_yr[b] += comp.lambda * nodal_ens;
    }

    // 鈹€鈹€ Nodal CIF / CID for SAIFI/SAIDI 鈹€鈹€
    for (size_t b = 0; b < nb; ++b) {
      bool bus_loss = false;
      double duration = 0.0;
      if (b < detail.nodal_shed_sw_mw.size() && detail.nodal_shed_sw_mw[b] > 0.01) {
        bus_loss = true;
        duration += comp.tau_sw_hr;
      }
      if (b < detail.nodal_shed_rep_mw.size() && detail.nodal_shed_rep_mw[b] > 0.01) {
        bus_loss = true;
        duration += comp.tau_rep_hr;
      }
      if (bus_loss) {
        nodal_cif[b] += comp.lambda;
        nodal_cid[b] += comp.lambda * duration;
      }
    }

    result.contingencies.push_back(std::move(detail));
    
    if (options.verbose) {
      spdlog::info("FMEA: {} -> shed={:.2f} MW, EENS_contrib={:.2f} MWh/yr",
                   comp.name, detail.shed_rep_mw, detail.eens_contribution);
    }
  }
#if defined(__GNUC__) && !defined(__clang__)
#  pragma GCC diagnostic pop
#endif

  result.edns_mw = result.eens_mwh_yr / 8760.0;

  // Sort contingencies by EENS contribution descending
  std::sort(result.contingencies.begin(), result.contingencies.end(),
            [](const FMEAContingencyDetail& a, const FMEAContingencyDetail& b) {
              return a.eens_contribution > b.eens_contribution;
            });

  // Compute distribution indices from nodal CIF/CID
  result.distribution_idx = compute_distribution_indices(sys, nodal_cif, nodal_cid);

  spdlog::info("FMEA: Complete. {} contingencies ({} with loss). "
               "EENS={:.2f} MWh/yr, LOLE={:.2f} hr/yr, LOLF={:.2f} occ/yr",
               result.n_contingencies, result.n_loss_contingencies,
               result.eens_mwh_yr, result.lole_hr_yr, result.lolf_occ_yr);
  spdlog::info("FMEA: SAIFI={:.4f}, SAIDI={:.4f}, ASAI={:.6f}",
               result.distribution_idx.saifi,
               result.distribution_idx.saidi,
               result.distribution_idx.asai);

  return result;
}

}  // namespace hacdcpf::analysis
