#include "hacdcpf/reliability/reliability_assessment.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
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
size_t hash_state(const std::vector<bool>& state) {
  size_t h = 0;
  for (size_t i = 0; i < state.size(); ++i) {
    if (state[i]) h ^= (size_t(1) << (i % 64)) + 0x9e3779b9 + (h << 6) + (h >> 2);
  }
  return h;
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
        for (int bid : isl.bus_ids) dead_buses.insert(bid);
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

  // Run DC-OPF with load shedding enabled
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
        ci.index = static_cast<int>(c < co.ng ? c : c - co.off_br);
        ci.is_generator = (c < co.ng);
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
        ci.index = static_cast<int>(c < co.ng ? c : c - co.off_br);
        ci.is_generator = (c < co.ng);
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
              StaticGen, RenewableGen, Storage, Transformer2W, Transformer3W };
  Type type = Generator;
  int idx;              // Index into corresponding vector
  std::string name;     // Human-readable label
  double lambda;        // Failure rate (occ/yr)
  double tau_sw_hr;     // Switching/isolation duration (hours)
  double tau_rep_hr;    // Repair duration (hours)
};

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

  return catalog;
}

// Evaluate a single contingency (one component out) for a given stage
// by running DC-OPF with load shedding.
StateEvalResult evaluate_contingency_stage(
    const HybridPowerSystem& sys,
    const FMEAComponent& comp,
    const opf::DCOPFOptions& opf_opt,
    double load_scale) {
  
  HybridPowerSystem sys_copy = sys;
  
  // Apply load scaling
  if (std::abs(load_scale - 1.0) > 1e-9) {
    for (auto& bus : sys_copy.ac.buses) {
      bus.pd_mw *= load_scale;
      bus.qd_mvar *= load_scale;
    }
    for (auto& ld : sys_copy.ac.loads) {
      ld.p_mw *= load_scale;
      ld.q_mvar *= load_scale;
    }
  }
  
  // Take the failed component out of service
  switch (comp.type) {
    case FMEAComponent::Generator:
      sys_copy.ac.generators[comp.idx].in_service = false;
      break;
    case FMEAComponent::ACBranch:
      sys_copy.ac.branches[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCBranch:
      sys_copy.dc.branches[comp.idx].in_service = false;
      break;
    case FMEAComponent::VSCConverter:
      sys_copy.vsc_converters[comp.idx].in_service = false;
      break;
    case FMEAComponent::StaticGen:
      sys_copy.ac.static_generators[comp.idx].in_service = false;
      break;
    case FMEAComponent::RenewableGen:
      sys_copy.ac.renewable_gens[comp.idx].in_service = false;
      break;
    case FMEAComponent::Storage:
      sys_copy.ac.storage[comp.idx].in_service = false;
      break;
    case FMEAComponent::Transformer2W:
      sys_copy.ac.transformers_2w[comp.idx].in_service = false;
      break;
    case FMEAComponent::Transformer3W:
      sys_copy.ac.transformers_3w[comp.idx].in_service = false;
      break;
  }
  
  // Solve DC-OPF with load shedding
  auto opf_result = opf::solve_dc_opf(sys_copy, opf_opt);
  
  StateEvalResult result;
  result.curtailment_mw = opf_result.total_load_shedding_mw;
  result.nodal_curtailment_mw = opf_result.load_shedding_mw;
  result.is_loss_state = (result.curtailment_mw > 0.01);
  return result;
}

}  // anonymous namespace

FMEAResult run_distribution_fmea(
    const HybridPowerSystem& sys,
    const FMEAOptions& options) {
  
  spdlog::info("FMEA: Starting N-1 failure-mode enumeration");
  
  FMEAResult result;
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

  // TODO(unimplemented): options.enable_microgrid_islanding — islands not formed
  // TODO(unimplemented): options.enable_switch_reconfiguration — no network switching
  // TODO(unimplemented): options.enable_repair_reconfiguration — repair stage reuses switching OPF result
  // TODO(unimplemented): options.enable_storage_dispatch — storage not dispatched during contingency
  // TODO(unimplemented): options.enable_grid_forming_vsc_support — VSC grid-forming not modelled
  // TODO(unimplemented): options.enable_black_start_storage — black-start capability not modelled
  // Currently the repair stage is identical to the switching stage (same topology, same OPF result).

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

    switch (comp.type) {
      case FMEAComponent::Generator:    detail.component_type = "generator"; break;
      case FMEAComponent::ACBranch:     detail.component_type = "ac_branch"; break;
      case FMEAComponent::DCBranch:     detail.component_type = "dc_branch"; break;
      case FMEAComponent::VSCConverter: detail.component_type = "vsc_converter"; break;
      case FMEAComponent::StaticGen:    detail.component_type = "static_generator"; break;
      case FMEAComponent::RenewableGen: detail.component_type = "renewable_gen"; break;
      case FMEAComponent::Storage:      detail.component_type = "storage"; break;
      case FMEAComponent::Transformer2W: detail.component_type = "transformer_2w"; break;
      case FMEAComponent::Transformer3W: detail.component_type = "transformer_3w"; break;
    }

    // 鈹€鈹€ Switching stage 鈹€鈹€
    auto eval_sw = evaluate_contingency_stage(
        sys, comp, opf_opt, options.load_scale_factor);
    detail.shed_sw_mw = eval_sw.curtailment_mw;
    detail.ens_sw_mwh = eval_sw.curtailment_mw * comp.tau_sw_hr;
    detail.causes_loss_sw = eval_sw.is_loss_state;
    detail.nodal_shed_sw_mw = eval_sw.nodal_curtailment_mw;

    // 鈹€鈹€ Repair stage 鈹€鈹€
    // Same topology (component still out), same evaluation
    // We reuse the same OPF result since the topology is unchanged in both
    // stages; the only difference is the duration multiplied to get ENS.
    detail.shed_rep_mw = eval_sw.curtailment_mw;
    detail.ens_rep_mwh = eval_sw.curtailment_mw * comp.tau_rep_hr;
    detail.causes_loss_rep = eval_sw.is_loss_state;
    detail.nodal_shed_rep_mw = eval_sw.nodal_curtailment_mw;

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
    for (size_t b = 0; b < nb && b < eval_sw.nodal_curtailment_mw.size(); ++b) {
      double nodal_ens = eval_sw.nodal_curtailment_mw[b] *
                          (comp.tau_sw_hr + comp.tau_rep_hr);
      result.nodal_eens_mwh_yr[b] += comp.lambda * nodal_ens;
    }

    // 鈹€鈹€ Nodal CIF / CID for SAIFI/SAIDI 鈹€鈹€
    for (size_t b = 0; b < nb && b < eval_sw.nodal_curtailment_mw.size(); ++b) {
      if (eval_sw.nodal_curtailment_mw[b] > 0.01) {
        nodal_cif[b] += comp.lambda;
        nodal_cid[b] += comp.lambda * (comp.tau_sw_hr + comp.tau_rep_hr);
      }
    }

    result.contingencies.push_back(std::move(detail));
    
    if (options.verbose) {
      spdlog::info("FMEA: {} -> shed={:.2f} MW, EENS_contrib={:.2f} MWh/yr",
                   comp.name, eval_sw.curtailment_mw, detail.eens_contribution);
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
