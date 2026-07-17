#include "hacdcpf/reliability/reliability_assessment.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <spdlog/spdlog.h>

#include <Eigen/Sparse>

#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/model/effective_capacity.hpp"
#include "hacdcpf/util/parallel_execution.hpp"
#include "hacdcpf/util/thread_pool.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// Unified reliability parameter resolver  (code-review Finding 1 & 2)
// ═══════════════════════════════════════════════════════════════════════
// Converts the heterogeneous component reliability fields into one canonical
// {lambda, repair_hr, unavailability, mttf_hr} set, applying the exact rules
// from the review's "Reliability Parameter Semantics" section.  The same
// function is used by FMEA, the Monte-Carlo paths, and unit tests so that
// identical components yield identical parameters across all methods.
ReliabilityParams resolve_reliability_params(
    const ReliabilityRawFields& raw,
    const ReliabilityDataPolicy& policy,
    double default_lambda_per_year,
    double default_repair_hr) {
  ReliabilityParams p;

  const double kHoursPerYear =
      (std::isfinite(policy.hours_per_year) && policy.hours_per_year > 0.0)
          ? policy.hours_per_year
          : 8760.0;

  // Domain validity: non-finite or non-positive inputs are treated as "not
  // provided" (frozen mathematical spec — all conversions require domain checks).
  auto valid = [](double v) { return std::isfinite(v) && v > 0.0; };

  const bool has_lambda     = valid(raw.failure_rate_per_year);
  const bool has_mttr_hr    = valid(raw.mttr_hr);
  const bool has_mtbf       = valid(raw.mtbf_hours);
  const bool has_mttr_hours = valid(raw.mttr_hours);
  const bool has_mttf       = valid(raw.mttf_hours);
  const bool has_for        = std::isfinite(raw.forced_outage_rate) &&
                              raw.forced_outage_rate > 0.0 && raw.forced_outage_rate < 1.0;
  const bool has_pdemand    = std::isfinite(raw.probability_per_demand) &&
                              raw.probability_per_demand > 0.0 &&
                              raw.probability_per_demand <= 1.0;
  const bool has_demandfreq = valid(raw.demand_frequency_per_year);
  const bool has_cyber      = valid(raw.cyber_recovery_hr);

  // Carry active-mode descriptors through regardless of the resolution outcome.
  p.is_active = raw.is_active;
  p.probability_per_demand = raw.probability_per_demand;
  p.demand_frequency_per_year = raw.demand_frequency_per_year;
  p.cyber_recovery_hr = raw.cyber_recovery_hr;

  auto finalize_from_lambda_repair = [&](double lambda, double repair) {
    p.lambda_per_year = lambda;
    p.repair_hr = repair;
    if (repair > 0.0) {
      const double mu = kHoursPerYear / repair;  // repairs per year
      p.unavailability = (lambda + mu) > 0.0 ? lambda / (lambda + mu) : 0.0;
    }
    p.mttf_hr = lambda > 0.0 ? kHoursPerYear / lambda : 0.0;
  };

  // Repair/recovery preference: cyber recovery time overrides physical repair
  // when explicitly provided (cyber/control modes need no physical repair).
  const double active_repair =
      has_cyber ? raw.cyber_recovery_hr
                : (has_mttr_hr ? raw.mttr_hr : (has_mttr_hours ? raw.mttr_hours : 0.0));

  // ── Case-data resolution (priority by which fields the mode provides) ──
  // Template active params (catalog defaults, not case/model data) only resolve
  // when defaulting is permitted; under StrictCaseDataOnly they are "missing".
  const bool active_case = raw.is_active && has_pdemand && has_demandfreq;
  const bool may_default_now =
      policy.default_policy != ReliabilityDefaultPolicy::StrictCaseDataOnly;
  const bool active_usable =
      active_case && (!raw.active_params_are_template || may_default_now);
  bool template_active_resolved = false;
  if (active_usable) {
    // Active-on-demand: lambda_active = nu_demand * p_demand.
    const double lambda_active =
        raw.demand_frequency_per_year * raw.probability_per_demand;
    p.lambda_active_per_year = lambda_active;
    finalize_from_lambda_repair(lambda_active, active_repair);
    if (raw.active_params_are_template) {
      p.used_default = true;
      p.data_source = "default";  // template default — not case data (honest provenance)
      template_active_resolved = true;
    } else {
      p.has_data = true;
      p.data_source = "case";
    }
    if (active_repair <= 0.0) {
      p.warnings.push_back(
          "Active-on-demand mode has no repair/recovery time; unavailability is "
          "undetermined (equivalent annual frequency only).");
    }
  } else if (has_lambda && has_mttr_hr) {
    // ACBranch-style: failures/year + repair hours.
    finalize_from_lambda_repair(raw.failure_rate_per_year, raw.mttr_hr);
    p.has_data = true;
    p.data_source = "case";
  } else if (has_mttf) {
    // Explicit MTTF (preferred over the ambiguous legacy MTBF).
    const double lambda = kHoursPerYear / raw.mttf_hours;
    const double repair = has_mttr_hours ? raw.mttr_hours
                                         : (has_mttr_hr ? raw.mttr_hr : 0.0);
    if (repair > 0.0) {
      finalize_from_lambda_repair(lambda, repair);
      p.unavailability = repair / (raw.mttf_hours + repair);
    } else {
      p.lambda_per_year = lambda;
      p.mttf_hr = raw.mttf_hours;
    }
    p.has_data = true;
    p.data_source = "case";
  } else if (has_mtbf) {
    // Legacy MTBF with declared convention (records which was applied).
    double mttf = raw.mtbf_hours;
    if (policy.mtbf_convention == MtbfConvention::MtbfAsCycleTime && has_mttr_hours) {
      mttf = std::max(raw.mtbf_hours - raw.mttr_hours, 0.0);
    }
    p.mtbf_convention_applied =
        (policy.mtbf_convention == MtbfConvention::Unspecified)
            ? MtbfConvention::MtbfAsMttf
            : policy.mtbf_convention;
    if (policy.mtbf_convention == MtbfConvention::Unspecified) {
      p.warnings.push_back(
          "mtbf_hours convention not declared; assumed MTBF-as-MTTF (provenance).");
    }
    const double lambda = mttf > 0.0 ? kHoursPerYear / mttf : 0.0;
    if (has_mttr_hours) {
      finalize_from_lambda_repair(lambda, raw.mttr_hours);
      // Exact MTBF/MTTR steady-state unavailability form.
      p.unavailability = raw.mttr_hours / (mttf + raw.mttr_hours);
    } else {
      // MTBF only: frequency known, repair time will be defaulted downstream.
      p.lambda_per_year = lambda;
      p.mttf_hr = mttf;  // MTTF ≈ MTBF when MTTR ≪ MTBF
    }
    p.has_data = true;
    p.data_source = "case";
  } else if (has_for && (has_mttr_hr || has_mttr_hours)) {
    // Generator / VSC / Storage-style: FOR + repair hours.
    const double mttr = has_mttr_hr ? raw.mttr_hr : raw.mttr_hours;
    const double f = raw.forced_outage_rate;
    p.unavailability = f;
    p.repair_hr = mttr;
    p.lambda_per_year = f / ((1.0 - f) * mttr) * kHoursPerYear;
    p.mttf_hr = mttr * (1.0 - f) / f;
    p.has_data = true;
    p.data_source = "case";
  } else if (has_for) {
    // FOR only: unavailability is known but frequency/duration are not.
    p.unavailability = raw.forced_outage_rate;
    p.has_data = true;
    p.data_source = "case";
    p.warnings.push_back(
        "Only forced_outage_rate provided; failure frequency and repair time "
        "are undetermined (no MTTR).");
  } else if (has_lambda) {
    // Lambda only: frequency known, repair time unknown.
    p.lambda_per_year = raw.failure_rate_per_year;
    p.mttf_hr = kHoursPerYear / raw.failure_rate_per_year;
    p.has_data = true;
    p.data_source = "case";
    p.warnings.push_back(
        "Only failure_rate provided; repair time and unavailability are "
        "undetermined (no MTTR).");
  }

  // ── Missing-data handling per policy ──
  const bool resolved = p.has_data || template_active_resolved;
  const bool incomplete = !resolved || p.repair_hr <= 0.0 || p.lambda_per_year <= 0.0;
  if (incomplete) {
    const bool may_default =
        policy.default_policy != ReliabilityDefaultPolicy::StrictCaseDataOnly;
    if (may_default && default_lambda_per_year > 0.0 && default_repair_hr > 0.0) {
      // Fill only the gaps from the per-kind template defaults.
      const double lambda =
          (p.lambda_per_year > 0.0) ? p.lambda_per_year : default_lambda_per_year;
      const double repair = (p.repair_hr > 0.0) ? p.repair_hr : default_repair_hr;
      finalize_from_lambda_repair(lambda, repair);
      p.used_default = true;
      p.data_source = p.has_data ? "case" : "default";
      if (!resolved) {
        p.warnings.push_back("No case reliability data; using per-kind default "
                             "(lambda=" + std::to_string(lambda) +
                             " occ/yr, repair=" + std::to_string(repair) + " hr).");
      }
      p.has_data = p.has_data;  // keep "had real data" semantics distinct
    } else if (!resolved) {
      // Strict mode (or no template available): report missing, invent nothing.
      p.data_source = "missing";
      p.warnings.push_back("No reliability data available under the active "
                           "data policy (StrictCaseDataOnly).");
    }
  }

  return p;
}

// IEEE RTS-24 Load Profile Data

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

// ── Reliability objective: minimise LOAD SHEDDING, not generation cost ───────
// Every Monte-Carlo / FMEA method reuses the economic-dispatch DC-OPF (or the
// hybrid AC/DC network LP) to evaluate a *failed* network state.  In a
// reliability study we do not care about generation economics — only whether
// the surviving network can serve load.  The objective must therefore be
// dominated by load shedding so the solver never sheds load it could otherwise
// supply; generation/source cost is kept only as a negligible tie-breaker among
// minimum-shed dispatches (lexicographic "minimum load shedding", matching the
// three-stage restoration MILP which already minimises pure MW shed).
//
// Enforced by choosing a Value-of-Lost-Load that strongly dominates the worst
// generator/source marginal cost.  The result is scaled, floored, and capped so
// the objective coefficient stays well-conditioned for the native simplex LP.
// Reliability code reads only the shed quantities (total_load_shedding_mw /
// nodal load_shedding_mw), never the objective value, so inflating VOLL changes
// *which* min-shed dispatch is chosen but not the reported curtailment.
double reliability_shedding_voll(const HybridPowerSystem& sys,
                                 double user_voll,
                                 double extra_max_marginal = 0.0) {
  double max_marginal = std::max(0.0, extra_max_marginal);
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    const double mc = g.cost_c1 + 2.0 * g.cost_c2 * std::max(0.0, g.pmax_mw);
    max_marginal = std::max(max_marginal, mc);
  }
  // 1000x the worst marginal cost guarantees min-shed dominance; the [1e5, 1e7]
  // window keeps the objective coefficient well-conditioned for the simplex.
  double dominant = 1000.0 * max_marginal;
  dominant = std::max(dominant, 1.0e5);
  dominant = std::min(dominant, 1.0e7);
  // Honour an explicit (larger) user VOLL but never fall below the dominant
  // floor that makes the evaluation a true minimum-load-shedding study.
  if (user_voll > 0.0) dominant = std::max(dominant, user_voll);
  return dominant;
}

using StateKey = std::vector<std::uint64_t>;

StateKey pack_state_key(const std::vector<bool>& state) {
  StateKey key((state.size() + 63U) / 64U, 0U);
  for (size_t i = 0; i < state.size(); ++i) {
    if (state[i]) {
      key[i / 64U] |= (std::uint64_t{1} << (i % 64U));
    }
  }
  return key;
}

struct StateKeyHash {
  size_t operator()(const StateKey& key) const noexcept {
    size_t h = 1469598103934665603ULL;
    for (std::uint64_t word : key) {
      h ^= static_cast<size_t>(word);
      h *= 1099511628211ULL;
    }
    return h;
  }
};

static int resolve_reliability_worker_count(int requested_threads,
                                            int work_items) {
  return util::resolve_worker_count(requested_threads, work_items);
}

// Returns total in-service DC load (MW) for the given system.
// Used to populate model_limitations in exported result structs.
static double total_inservice_dc_load_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  // Honour `scaling` so a unit scheduled out (scaling=0) is not reported as
  // hidden DC demand; matches the OPF / time-series convention.
  for (const auto& ld : sys.dc.loads)
    total += hacdcpf::model::effective_load_p_mw(ld);
  // Also count direct DCBus::pd_mw loads (not backed by a DCLoad element).
  for (const auto& b : sys.dc.buses)
    if (b.in_service) total += b.pd_mw;
  return total;
}

// Returns true when the system contains any DC/VSC component (buses, branches,
// loads, VSC converters, DC/DC converters, DC CBs, DC storage, DC PV).
// Used as the trigger for model_limitations warnings — AC-only OPF cannot
// faithfully model such systems regardless of whether DC load is currently > 0.
static bool has_dc_rich_components(const HybridPowerSystem& sys) {
  return !sys.dc.buses.empty()               ||
         !sys.dc.branches.empty()            ||
         !sys.dc.loads.empty()               ||
         !sys.vsc_converters.empty()         ||
         !sys.dc.dcdc_converters.empty()     ||
         !sys.dc.dc_circuit_breakers.empty() ||
         !sys.dc.storage.empty()             ||
         !sys.dc.pv_arrays.empty();
}

static bool has_hybrid_fmea_components(const HybridPowerSystem& sys) {
  return !sys.dc.buses.empty()               ||
         !sys.dc.branches.empty()            ||
         !sys.dc.loads.empty()               ||
         !sys.vsc_converters.empty()         ||
         !sys.dc.dcdc_converters.empty()     ||
         !sys.dc.dc_circuit_breakers.empty() ||
         !sys.dc.storage.empty()             ||
         !sys.dc.pv_arrays.empty()           ||
         !sys.dc.static_generators.empty()   ||
         !sys.dc.dc_static_generators.empty();
}

// State evaluation result
struct StateEvalResult {
  double curtailment_mw{0.0};
  std::vector<double> nodal_curtailment_mw;
  bool is_loss_state{false};
};

// Compute full component count for extended state vector:
// [generators | AC branches | static gens | renewable gens | AC storage |
//  VSC converters | DC branches | transformers_2w | transformers_3w |
//  DC/DC converters | DC circuit breakers | DC storage | DC PV arrays |
//  AC switches | AC circuit breakers | AC PV | DC static generation |
//  microgrids]
struct ComponentOffsets {
  size_t ng, nl, nsg, nrg, nst, nvsc, ndb, nt2, nt3;
  size_t ndcdc, ndccb, ndcst, ndcpv, nsw, ncb;
  size_t npvs;    ///< ac.pv_systems (PVSystem)
  size_t ndcsg;   ///< dc.static_generators (StaticGenerator AC-type in DC container)
  size_t ndcsgx;  ///< dc.dc_static_generators (StaticGeneratorDC)
  size_t nmg;     ///< system-level microgrids
  size_t off_gen, off_br, off_sg, off_rg, off_st, off_vsc, off_db, off_t2, off_t3;
  size_t off_dcdc, off_dccb, off_dcst, off_dcpv, off_sw, off_cb;
  size_t off_pvs, off_dcsg, off_dcsgx, off_mg;
  size_t total;

  ComponentOffsets(const HybridPowerSystem& sys) {
    ng    = sys.ac.generators.size();
    nl    = sys.ac.branches.size();
    nsg   = sys.ac.static_generators.size();
    nrg   = sys.ac.renewable_gens.size();
    nst   = sys.ac.storage.size();
    nvsc  = sys.vsc_converters.size();
    ndb   = sys.dc.branches.size();
    nt2   = sys.ac.transformers_2w.size();
    nt3   = sys.ac.transformers_3w.size();
    ndcdc = sys.dc.dcdc_converters.size();
    ndccb = sys.dc.dc_circuit_breakers.size();
    ndcst = sys.dc.storage.size();
    ndcpv = sys.dc.pv_arrays.size();
    nsw   = sys.ac.switches.size();
    ncb   = sys.ac.circuit_breakers.size();
    npvs  = sys.ac.pv_systems.size();
    ndcsg = sys.dc.static_generators.size();
    ndcsgx= sys.dc.dc_static_generators.size();
    nmg   = sys.microgrids.size();

    off_gen   = 0;
    off_br    = ng;
    off_sg    = off_br   + nl;
    off_rg    = off_sg   + nsg;
    off_st    = off_rg   + nrg;
    off_vsc   = off_st   + nst;
    off_db    = off_vsc  + nvsc;
    off_t2    = off_db   + ndb;
    off_t3    = off_t2   + nt2;
    off_dcdc  = off_t3   + nt3;
    off_dccb  = off_dcdc + ndcdc;
    off_dcst  = off_dccb + ndccb;
    off_dcpv  = off_dcst + ndcst;
    off_sw    = off_dcpv + ndcpv;
    off_cb    = off_sw   + nsw;
    off_pvs   = off_cb   + ncb;
    off_dcsg  = off_pvs  + npvs;
    off_dcsgx = off_dcsg + ndcsg;
    off_mg    = off_dcsgx + ndcsgx;
    total     = off_mg + nmg;
  }
};

// Decode a flat state-vector index c into {within-type index, type name}.
// Must be called after ComponentOffsets is constructed.
static std::pair<int, std::string>
decode_component(size_t c, const ComponentOffsets& co) {
  if      (c < co.off_br)    return {int(c - co.off_gen),   "Generator"};
  else if (c < co.off_sg)    return {int(c - co.off_br),    "ACBranch"};
  else if (c < co.off_rg)    return {int(c - co.off_sg),    "StaticGen"};
  else if (c < co.off_st)    return {int(c - co.off_rg),    "RenewableGen"};
  else if (c < co.off_vsc)   return {int(c - co.off_st),    "ACStorage"};
  else if (c < co.off_db)    return {int(c - co.off_vsc),   "VSCConverter"};
  else if (c < co.off_t2)    return {int(c - co.off_db),    "DCBranch"};
  else if (c < co.off_t3)    return {int(c - co.off_t2),    "Transformer2W"};
  else if (c < co.off_dcdc)  return {int(c - co.off_t3),    "Transformer3W"};
  else if (c < co.off_dccb)  return {int(c - co.off_dcdc),  "DCDCConverter"};
  else if (c < co.off_dcst)  return {int(c - co.off_dccb),  "DCCircuitBreaker"};
  else if (c < co.off_dcpv)  return {int(c - co.off_dcst),  "DCStorage"};
  else if (c < co.off_sw)    return {int(c - co.off_dcpv),  "DCPVArray"};
  else if (c < co.off_cb)    return {int(c - co.off_sw),    "ACSwitch"};
  else if (c < co.off_pvs)   return {int(c - co.off_cb),    "ACCircuitBreaker"};
  else if (c < co.off_dcsg)  return {int(c - co.off_pvs),   "ACPVSystem"};
  else if (c < co.off_dcsgx) return {int(c - co.off_dcsg),  "DCStaticGenAC"};
  else if (c < co.off_mg)    return {int(c - co.off_dcsgx), "DCStaticGen"};
  else                       return {int(c - co.off_mg), "Microgrid"};
}

// Per-state-vector-index flag: does the component have usable CASE reliability
// data?  Index layout matches ComponentOffsets.  Used by the Monte-Carlo paths
// to honour StrictCaseDataOnly — components without case data are not assigned
// invented failure rates (code-review Finding 1 & 2).
static std::vector<bool> mc_component_has_case_data(
    const HybridPowerSystem& sys, const ComponentOffsets& co,
    const ReliabilityDataPolicy& policy) {
  std::vector<bool> hd(co.total, false);
  auto has = [&](const ReliabilityRawFields& raw) {
    return resolve_reliability_params(raw, policy, 0.0, 0.0).has_data;
  };
  auto for_mttr = [](double f, double m) {
    ReliabilityRawFields r; r.forced_outage_rate = f; r.mttr_hr = m; return r;
  };
  auto lam_mttr = [](double l, double m) {
    ReliabilityRawFields r; r.failure_rate_per_year = l; r.mttr_hr = m; return r;
  };
  auto mtbf_mttr = [](double mb, double mt) {
    ReliabilityRawFields r; r.mtbf_hours = mb; r.mttr_hours = mt; return r;
  };
  for (size_t i = 0; i < co.ng; ++i) { const auto& x = sys.ac.generators[i];        hd[co.off_gen + i] = has(for_mttr(x.forced_outage_rate, x.mttr_hr)); }
  for (size_t i = 0; i < co.nl; ++i) { const auto& x = sys.ac.branches[i];          hd[co.off_br + i]  = has(lam_mttr(x.failure_rate, x.mttr_hr)); }
  for (size_t i = 0; i < co.nsg; ++i){ const auto& x = sys.ac.static_generators[i]; hd[co.off_sg + i]  = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.nrg; ++i){ const auto& x = sys.ac.renewable_gens[i];    hd[co.off_rg + i]  = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.nst; ++i){ const auto& x = sys.ac.storage[i];           hd[co.off_st + i]  = has(for_mttr(x.forced_outage_rate, x.mttr_hr)); }
  for (size_t i = 0; i < co.nvsc; ++i){ const auto& x = sys.vsc_converters[i];      hd[co.off_vsc + i] = has(for_mttr(x.forced_outage_rate, x.mttr_hr)); }
  for (size_t i = 0; i < co.ndb; ++i){ const auto& x = sys.dc.branches[i];          hd[co.off_db + i]  = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.nt2; ++i){ const auto& x = sys.ac.transformers_2w[i];   hd[co.off_t2 + i]  = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.nt3; ++i){ const auto& x = sys.ac.transformers_3w[i];   hd[co.off_t3 + i]  = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.ndcdc; ++i){ const auto& x = sys.dc.dcdc_converters[i]; hd[co.off_dcdc + i] = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  // DC circuit breakers: no reliability fields → always missing.
  for (size_t i = 0; i < co.ndcst; ++i){ const auto& x = sys.dc.storage[i];         hd[co.off_dcst + i] = has(for_mttr(x.forced_outage_rate, x.mttr_hr)); }
  for (size_t i = 0; i < co.ndcpv; ++i){ const auto& x = sys.dc.pv_arrays[i];       hd[co.off_dcpv + i] = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.nsw; ++i){
    const auto& x = sys.ac.switches[i];
    ReliabilityRawFields r = mtbf_mttr(x.mtbf_hours, x.mttr_hours);
    if (x.mtbf_hours <= 0 && x.p_sw_fail > 0 && x.p_sw_fail < 1.0) {
      r.failure_rate_per_year = x.p_sw_fail;
      r.mttr_hr = x.mttr_hours > 0 ? x.mttr_hours : 4.0;
    }
    hd[co.off_sw + i] = has(r);
  }
  // AC circuit breakers: no reliability fields → always missing.
  for (size_t i = 0; i < co.npvs; ++i){ const auto& x = sys.ac.pv_systems[i];          hd[co.off_pvs + i]   = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.ndcsg; ++i){ const auto& x = sys.dc.static_generators[i];  hd[co.off_dcsg + i]  = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.ndcsgx; ++i){ const auto& x = sys.dc.dc_static_generators[i]; hd[co.off_dcsgx + i] = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  for (size_t i = 0; i < co.nmg; ++i){ const auto& x = sys.microgrids[i]; hd[co.off_mg + i] = has(mtbf_mttr(x.mtbf_hours, x.mttr_hours)); }
  return hd;
}

// ── F10: unified Monte-Carlo reliability parameters ──────────────────────────
// Produce per-state-index {unavailability, MTTF(hr), MTTR(hr)} via the SAME
// resolve_reliability_params resolver and the SAME per-kind template defaults
// that build_fmea_catalog uses, so NSQ / SEQ / FMEA agree on both present-data
// and missing-data parameters for identical components (previously the MC paths
// used their own inline fallback constants, which diverged from FMEA).
struct MCReliability {
  std::vector<double> U;      // steady-state unavailability (NSQ Bernoulli prob)
  std::vector<double> mttf;   // mean time to failure, hr  (SEQ up-time)
  std::vector<double> mttr;   // mean time to repair, hr   (SEQ down-time)
};

static MCReliability compute_mc_reliability(
    const HybridPowerSystem& sys, const ComponentOffsets& co,
    const ReliabilityDataPolicy& policy) {
  MCReliability r;
  r.U.assign(co.total, 0.0);
  r.mttf.assign(co.total, 1e15);  // default: never fails within the horizon
  r.mttr.assign(co.total, 0.0);

  // Resolve one component and store {U, MTTF, MTTR} at state index `idx`.
  // `for_based` marks kinds whose case datum is a forced-outage rate (FOR): FOR
  // is the steady-state unavailability by definition, so it is honoured directly
  // (and used to back out a consistent MTTF) rather than being re-derived from a
  // template default when the repair time is absent.
  auto set = [&](size_t idx, const ReliabilityRawFields& raw,
                 double def_lambda, double def_repair, bool for_based) {
    const ReliabilityParams pr =
        resolve_reliability_params(raw, policy, def_lambda, def_repair);
    const bool valid_for = for_based && std::isfinite(raw.forced_outage_rate) &&
                           raw.forced_outage_rate > 0.0 &&
                           raw.forced_outage_rate < 1.0;
    double U, mttf, mttr;
    if (valid_for) {
      U = raw.forced_outage_rate;
      mttr = raw.mttr_hr > 0.0 ? raw.mttr_hr
                               : (pr.repair_hr > 0.0 ? pr.repair_hr : def_repair);
      mttf = mttr * (1.0 - U) / U;
    } else {
      U = pr.unavailability;
      mttf = pr.mttf_hr > 0.0 ? pr.mttf_hr : 1e15;  // unresolved -> never fails
      mttr = pr.repair_hr > 0.0 ? pr.repair_hr : def_repair;
    }
    r.U[idx] = U;
    r.mttf[idx] = mttf;
    r.mttr[idx] = (mttr > 0.0 ? mttr : def_repair);
  };
  auto raw_for  = [](double f, double m){ ReliabilityRawFields x; x.forced_outage_rate=f; x.mttr_hr=m; return x; };
  auto raw_lam  = [](double l, double m){ ReliabilityRawFields x; x.failure_rate_per_year=l; x.mttr_hr=m; return x; };
  auto raw_mtbf = [](double mb, double mt){ ReliabilityRawFields x; x.mtbf_hours=mb; x.mttr_hours=mt; return x; };

  for (size_t i=0;i<co.ng;   ++i){ const auto& x=sys.ac.generators[i];         set(co.off_gen+i,  raw_for (x.forced_outage_rate,x.mttr_hr), 8760.0/2000.0, 50.0, true); }
  for (size_t i=0;i<co.nl;   ++i){ const auto& x=sys.ac.branches[i];           set(co.off_br+i,   raw_lam (x.failure_rate,x.mttr_hr),       0.35, 10.0, false); }
  for (size_t i=0;i<co.nsg;  ++i){ const auto& x=sys.ac.static_generators[i];  set(co.off_sg+i,   raw_mtbf(x.mtbf_hours,x.mttr_hours),     1.5, 24.0, false); }
  for (size_t i=0;i<co.nrg;  ++i){ const auto& x=sys.ac.renewable_gens[i];     set(co.off_rg+i,   raw_mtbf(x.mtbf_hours,x.mttr_hours),     2.0, 48.0, false); }
  for (size_t i=0;i<co.nst;  ++i){ const auto& x=sys.ac.storage[i];            set(co.off_st+i,   raw_for (x.forced_outage_rate,x.mttr_hr), 1.0, 24.0, true); }
  for (size_t i=0;i<co.nvsc; ++i){ const auto& x=sys.vsc_converters[i];        set(co.off_vsc+i,  raw_for (x.forced_outage_rate,x.mttr_hr), 0.10, 48.0, true); }
  for (size_t i=0;i<co.ndb;  ++i){ const auto& x=sys.dc.branches[i];           set(co.off_db+i,   raw_mtbf(x.mtbf_hours,x.mttr_hours),     0.20, 24.0, false); }
  for (size_t i=0;i<co.nt2;  ++i){ const auto& x=sys.ac.transformers_2w[i];    set(co.off_t2+i,   raw_mtbf(x.mtbf_hours,x.mttr_hours),     0.03, 200.0, false); }
  for (size_t i=0;i<co.nt3;  ++i){ const auto& x=sys.ac.transformers_3w[i];    set(co.off_t3+i,   raw_mtbf(x.mtbf_hours,x.mttr_hours),     0.04, 200.0, false); }
  for (size_t i=0;i<co.ndcdc;++i){ const auto& x=sys.dc.dcdc_converters[i];    set(co.off_dcdc+i, raw_mtbf(x.mtbf_hours,x.mttr_hours),     0.20, 48.0, false); }
  for (size_t i=0;i<co.ndccb;++i){                                             set(co.off_dccb+i, ReliabilityRawFields{},                   0.10, 8.0, false); }
  for (size_t i=0;i<co.ndcst;++i){ const auto& x=sys.dc.storage[i];            set(co.off_dcst+i, raw_for (x.forced_outage_rate,x.mttr_hr), 1.0, 24.0, true); }
  for (size_t i=0;i<co.ndcpv;++i){ const auto& x=sys.dc.pv_arrays[i];          set(co.off_dcpv+i, raw_mtbf(x.mtbf_hours,x.mttr_hours),     1.5, 24.0, false); }
  for (size_t i=0;i<co.nsw;  ++i){
    const auto& x=sys.ac.switches[i];
    ReliabilityRawFields raw = raw_mtbf(x.mtbf_hours, x.mttr_hours);
    if (x.mtbf_hours <= 0 && x.p_sw_fail > 0 && x.p_sw_fail < 1.0) {
      raw.failure_rate_per_year = x.p_sw_fail;
      raw.mttr_hr = x.mttr_hours > 0 ? x.mttr_hours : 4.0;
    }
    set(co.off_sw+i, raw, 0.05, 4.0, false);
  }
  for (size_t i=0;i<co.ncb;  ++i){                                             set(co.off_cb+i,   ReliabilityRawFields{},                   0.05, 8.0, false); }
  for (size_t i=0;i<co.npvs; ++i){ const auto& x=sys.ac.pv_systems[i];         set(co.off_pvs+i,  raw_mtbf(x.mtbf_hours,x.mttr_hours),     1.5, 24.0, false); }
  for (size_t i=0;i<co.ndcsg;++i){ const auto& x=sys.dc.static_generators[i];  set(co.off_dcsg+i, raw_mtbf(x.mtbf_hours,x.mttr_hours),     1.5, 24.0, false); }
  for (size_t i=0;i<co.ndcsgx;++i){const auto& x=sys.dc.dc_static_generators[i];set(co.off_dcsgx+i,raw_mtbf(x.mtbf_hours,x.mttr_hours),     1.5, 24.0, false); }
  for (size_t i=0;i<co.nmg;   ++i){const auto& x=sys.microgrids[i];             set(co.off_mg+i,   raw_mtbf(x.mtbf_hours,x.mttr_hours),     8760.0/20000.0, 8.0, false); }
  return r;
}

// Build the display name for a component from the actual system data.
// Uses component.index (original element ID) and component.name (business
// label) instead of the 0-based vector position, so that non-contiguous
// numbering and named components are represented faithfully.
// Format: "TypeName[index]" or "TypeName[name(index)]" when name is present.
static std::string resolve_component_name(
    int vec_idx, const std::string& type_name,
    const HybridPowerSystem& sys)
{
  auto fmt = [&](int id, const std::string& nm) -> std::string {
    return type_name + "[" +
           (nm.empty() ? std::to_string(id) : nm + "(" + std::to_string(id) + ")") +
           "]";
  };
  if (type_name == "Generator"       && vec_idx < (int)sys.ac.generators.size())
    { const auto& x = sys.ac.generators[vec_idx];      return fmt(x.index, x.name); }
  if (type_name == "ACBranch"        && vec_idx < (int)sys.ac.branches.size())
    { const auto& x = sys.ac.branches[vec_idx];        return fmt(x.index, x.name); }
  if (type_name == "StaticGen"       && vec_idx < (int)sys.ac.static_generators.size())
    { const auto& x = sys.ac.static_generators[vec_idx]; return fmt(x.index, x.name); }
  if (type_name == "RenewableGen"    && vec_idx < (int)sys.ac.renewable_gens.size())
    { const auto& x = sys.ac.renewable_gens[vec_idx];  return fmt(x.index, x.name); }
  if (type_name == "ACStorage"       && vec_idx < (int)sys.ac.storage.size())
    { const auto& x = sys.ac.storage[vec_idx];         return fmt(x.index, x.name); }
  if (type_name == "VSCConverter"    && vec_idx < (int)sys.vsc_converters.size())
    { const auto& x = sys.vsc_converters[vec_idx];     return fmt(x.index, x.name); }
  if (type_name == "DCBranch"        && vec_idx < (int)sys.dc.branches.size())
    { const auto& x = sys.dc.branches[vec_idx];        return fmt(x.index, x.name); }
  if (type_name == "Transformer2W"   && vec_idx < (int)sys.ac.transformers_2w.size())
    { const auto& x = sys.ac.transformers_2w[vec_idx]; return fmt(x.index, x.name); }
  if (type_name == "Transformer3W"   && vec_idx < (int)sys.ac.transformers_3w.size())
    { const auto& x = sys.ac.transformers_3w[vec_idx]; return fmt(x.index, x.name); }
  if (type_name == "DCDCConverter"   && vec_idx < (int)sys.dc.dcdc_converters.size())
    { const auto& x = sys.dc.dcdc_converters[vec_idx]; return fmt(x.index, x.name); }
  if (type_name == "DCCircuitBreaker"&& vec_idx < (int)sys.dc.dc_circuit_breakers.size())
    { const auto& x = sys.dc.dc_circuit_breakers[vec_idx]; return fmt(x.index, x.name); }
  if (type_name == "DCStorage"       && vec_idx < (int)sys.dc.storage.size())
    { const auto& x = sys.dc.storage[vec_idx];         return fmt(x.index, x.name); }
  if (type_name == "DCPVArray"       && vec_idx < (int)sys.dc.pv_arrays.size())
    { const auto& x = sys.dc.pv_arrays[vec_idx];       return fmt(x.index, x.name); }
  if (type_name == "ACSwitch"        && vec_idx < (int)sys.ac.switches.size())
    { const auto& x = sys.ac.switches[vec_idx];        return fmt(x.index, x.name); }
  if (type_name == "ACCircuitBreaker"&& vec_idx < (int)sys.ac.circuit_breakers.size())
    { const auto& x = sys.ac.circuit_breakers[vec_idx]; return fmt(x.index, x.name); }
  if (type_name == "ACPVSystem"       && vec_idx < (int)sys.ac.pv_systems.size())
    { const auto& x = sys.ac.pv_systems[vec_idx];           return fmt(x.index, x.name); }
  if (type_name == "DCStaticGenAC"    && vec_idx < (int)sys.dc.static_generators.size())
    { const auto& x = sys.dc.static_generators[vec_idx];    return fmt(x.index, x.name); }
  if (type_name == "DCStaticGen"      && vec_idx < (int)sys.dc.dc_static_generators.size())
    { const auto& x = sys.dc.dc_static_generators[vec_idx]; return fmt(x.index, x.name); }
  if (type_name == "Microgrid"        && vec_idx < (int)sys.microgrids.size())
    { const auto& x = sys.microgrids[vec_idx];              return fmt(x.index, x.name); }
  // Fallback: vector position (unchanged from old behaviour)
  return type_name + "[" + std::to_string(vec_idx) + "]";
}

static std::vector<bool> build_active_component_mask(
    const HybridPowerSystem& sys,
    const ComponentOffsets& co) {
  std::vector<bool> active(co.total, true);
  for (size_t i = 0; i < co.ng;    ++i) active[co.off_gen  + i] = sys.ac.generators[i].in_service;
  for (size_t i = 0; i < co.nl;    ++i) active[co.off_br   + i] = sys.ac.branches[i].in_service;
  for (size_t i = 0; i < co.nsg;   ++i) active[co.off_sg   + i] = sys.ac.static_generators[i].in_service;
  for (size_t i = 0; i < co.nrg;   ++i) active[co.off_rg   + i] = sys.ac.renewable_gens[i].in_service;
  for (size_t i = 0; i < co.nst;   ++i) active[co.off_st   + i] = sys.ac.storage[i].in_service;
  for (size_t i = 0; i < co.nvsc;  ++i) active[co.off_vsc  + i] = sys.vsc_converters[i].in_service;
  for (size_t i = 0; i < co.ndb;   ++i) active[co.off_db   + i] = sys.dc.branches[i].in_service;
  for (size_t i = 0; i < co.nt2;   ++i) active[co.off_t2   + i] = sys.ac.transformers_2w[i].in_service;
  for (size_t i = 0; i < co.nt3;   ++i) active[co.off_t3   + i] = sys.ac.transformers_3w[i].in_service;
  for (size_t i = 0; i < co.ndcdc; ++i) active[co.off_dcdc + i] = sys.dc.dcdc_converters[i].in_service;
  for (size_t i = 0; i < co.ndccb; ++i) active[co.off_dccb + i] = sys.dc.dc_circuit_breakers[i].in_service;
  for (size_t i = 0; i < co.ndcst; ++i) active[co.off_dcst + i] = sys.dc.storage[i].in_service;
  for (size_t i = 0; i < co.ndcpv; ++i) active[co.off_dcpv + i] = sys.dc.pv_arrays[i].in_service;
  for (size_t i = 0; i < co.nsw;   ++i) active[co.off_sw   + i] = sys.ac.switches[i].in_service;
  for (size_t i = 0; i < co.ncb;   ++i) active[co.off_cb   + i] = sys.ac.circuit_breakers[i].in_service;
  for (size_t i = 0; i < co.npvs;  ++i) active[co.off_pvs  + i] = sys.ac.pv_systems[i].in_service;
  for (size_t i = 0; i < co.ndcsg; ++i) active[co.off_dcsg + i] = sys.dc.static_generators[i].in_service;
  for (size_t i = 0; i < co.ndcsgx;++i) active[co.off_dcsgx+ i] = sys.dc.dc_static_generators[i].in_service;
  for (size_t i = 0; i < co.nmg;   ++i) active[co.off_mg   + i] = sys.microgrids[i].in_service;
  return active;
}

static long long load_scale_cache_key(double load_scale) {
  return static_cast<long long>(std::llround(load_scale * 1.0e9));
}

static double hourly_load_scale(
    const LoadProfile& load_profile,
    const ReliabilityOptions& options,
    int hour) {
  double load_scale = options.load_scale_factor;
  if (hour >= 0 && hour < static_cast<int>(load_profile.factors.size())) {
    load_scale *= load_profile.factors[static_cast<size_t>(hour)];
  }
  return load_scale;
}

static HybridPowerSystem apply_load_profile_spatial_factors(
    const HybridPowerSystem& source,
    const LoadProfile& load_profile) {
  HybridPowerSystem sys = source;
  auto factor_at = [](const std::vector<double>& factors, size_t i) {
    const double factor = i < factors.size() ? factors[i] : 1.0;
    if (!std::isfinite(factor) || factor < 0.0) {
      throw std::invalid_argument(
          "Sequential load-profile spatial factors must be finite and non-negative");
    }
    return factor;
  };
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const double factor = factor_at(load_profile.ac_bus_factors, i);
    sys.ac.buses[i].pd_mw *= factor;
    sys.ac.buses[i].qd_mvar *= factor;
  }
  for (size_t i = 0; i < sys.ac.loads.size(); ++i) {
    const double factor = factor_at(load_profile.ac_load_factors, i);
    sys.ac.loads[i].p_mw *= factor;
    sys.ac.loads[i].q_mvar *= factor;
  }
  for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
    sys.dc.buses[i].pd_mw *= factor_at(load_profile.dc_bus_factors, i);
  }
  for (size_t i = 0; i < sys.dc.loads.size(); ++i) {
    sys.dc.loads[i].p_mw *= factor_at(load_profile.dc_load_factors, i);
  }
  return sys;
}

bool reliability_microgrid_contains_bus(const Microgrid& microgrid, int bus) {
  return microgrid.pcc_bus == bus ||
         std::find(microgrid.internal_buses.begin(),
                   microgrid.internal_buses.end(), bus) !=
             microgrid.internal_buses.end();
}

double reliability_microgrid_capacity_mw(const Microgrid& microgrid) {
  return std::max({0.0, microgrid.capacity_mw,
                   microgrid.total_generation_mw,
                   microgrid.total_dg_capacity_mw +
                       microgrid.total_diesel_capacity_mw,
                   microgrid.p_exchange_max_mw});
}

// Materialize only the aggregate capacity not already represented by explicit
// devices. An islanding-capable microgrid also supplies the voltage-reference
// role needed by the topology precheck. If its sampled MC state is failed,
// neither the residual source nor that island anchor is created.
void materialize_mc_microgrid_support(HybridPowerSystem& sys) {
  int next_generator = 1;
  for (const auto& generator : sys.ac.generators)
    next_generator = std::max(next_generator, generator.index + 1);

  for (const auto& microgrid : sys.microgrids) {
    if (!microgrid.in_service || microgrid.pcc_bus <= 0) continue;

    const auto inside = [&](int bus) {
      return reliability_microgrid_contains_bus(microgrid, bus);
    };
    double explicit_capacity = 0.0;
    for (const auto& generator : sys.ac.generators)
      if (generator.in_service && inside(generator.bus))
        explicit_capacity += hacdcpf::model::effective_capacity_mw(generator);
    for (const auto& source : sys.ac.static_generators)
      if (source.in_service && inside(source.bus))
        explicit_capacity += hacdcpf::model::effective_capacity_mw(source);
    for (const auto& source : sys.ac.renewable_gens)
      if (source.in_service && inside(source.bus))
        explicit_capacity += source.p_rated_mw > 0.0
                                 ? source.p_rated_mw * source.capacity_factor
                                 : std::max(0.0, source.p_mw);
    for (const auto& source : sys.ac.pv_systems)
      if (source.in_service && inside(source.bus))
        explicit_capacity += source.pmax_mw > 0.0
                                 ? source.pmax_mw
                                 : std::max(0.0, source.p_mw);
    for (const auto& storage : sys.ac.storage)
      if (storage.in_service && inside(storage.bus))
        explicit_capacity += hacdcpf::model::effective_capacity_mw(storage);

    const double residual = std::max(
        0.0, reliability_microgrid_capacity_mw(microgrid) - explicit_capacity);
    if (residual > 1e-9) {
      Generator aggregate;
      aggregate.index = next_generator++;
      aggregate.name = "Reliability_microgrid_" +
                       std::to_string(microgrid.index);
      aggregate.bus = microgrid.pcc_bus;
      aggregate.in_service = true;
      aggregate.pmin_mw = 0.0;
      aggregate.pmax_mw = residual;
      aggregate.qmin_mvar = -residual;
      aggregate.qmax_mvar = residual;
      aggregate.cost_c1 = 1.0;
      sys.ac.generators.push_back(std::move(aggregate));
    }

    if (microgrid.islanding_capability) {
      for (auto& bus : sys.ac.buses) {
        if (bus.index == microgrid.pcc_bus) {
          bus.bus_type = BusType::SLACK;
          break;
        }
      }
    }
  }
}

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
    double load_scale = 1.0,
    double curtail_threshold_mw = 0.01) {
  
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

  // Apply DC/DC converter failures
  for (size_t i = 0; i < co.ndcdc; ++i) {
    if (component_failures[co.off_dcdc + i]) {
      sys.dc.dcdc_converters[i].in_service = false;
    }
  }

  // Apply DC circuit breaker failures
  for (size_t i = 0; i < co.ndccb; ++i) {
    if (component_failures[co.off_dccb + i]) {
      sys.dc.dc_circuit_breakers[i].in_service = false;
    }
  }

  // Apply DC storage failures
  for (size_t i = 0; i < co.ndcst; ++i) {
    if (component_failures[co.off_dcst + i]) {
      sys.dc.storage[i].in_service = false;
    }
  }

  // Apply DC PV array failures
  for (size_t i = 0; i < co.ndcpv; ++i) {
    if (component_failures[co.off_dcpv + i]) {
      sys.dc.pv_arrays[i].in_service = false;
    }
  }

  // Apply AC switch failures
  for (size_t i = 0; i < co.nsw; ++i) {
    if (component_failures[co.off_sw + i]) {
      sys.ac.switches[i].in_service = false;
    }
  }

  // Apply AC circuit breaker failures
  for (size_t i = 0; i < co.ncb; ++i) {
    if (component_failures[co.off_cb + i]) {
      sys.ac.circuit_breakers[i].in_service = false;
    }
  }

  // Apply AC PV system failures
  for (size_t i = 0; i < co.npvs; ++i) {
    if (component_failures[co.off_pvs + i]) {
      sys.ac.pv_systems[i].in_service = false;
    }
  }

  // Apply DC static generator failures (AC StaticGenerator type in DC container)
  for (size_t i = 0; i < co.ndcsg; ++i) {
    if (component_failures[co.off_dcsg + i]) {
      sys.dc.static_generators[i].in_service = false;
    }
  }

  // Apply DC static generator failures (StaticGeneratorDC type)
  for (size_t i = 0; i < co.ndcsgx; ++i) {
    if (component_failures[co.off_dcsgx + i]) {
      sys.dc.dc_static_generators[i].in_service = false;
    }
  }

  // Apply microgrid supervisory/islanding-function failures before aggregate
  // capacity and island anchors are materialized for this sampled state.
  for (size_t i = 0; i < co.nmg; ++i) {
    if (component_failures[co.off_mg + i]) {
      sys.microgrids[i].in_service = false;
    }
  }

  materialize_mc_microgrid_support(sys);

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
    // Scale DC demand too so the hybrid LP (below) sees a consistent stress level.
    for (auto& ld : sys.dc.loads) ld.p_mw *= load_scale;
    for (auto& bus : sys.dc.buses) bus.pd_mw *= load_scale;
  }

  // ── Hybrid AC/DC routing ────────────────────────────────────────────────
  // For hybrid systems, evaluate the failed state with the hybrid network LP so
  // DC load curtailment, DC sources, and VSC/DC-DC transfers are included in the
  // shed (DC load now contributes to EENS/LOLE).  evaluate_failed_network_state
  // prepares supply sources (external grid / VPP / mobile storage / microgrid /
  // energy router) and routes hybrid systems to evaluate_hybrid_fmea_network_lp;
  // it only calls back into evaluate_state for NON-hybrid systems, so there is
  // no recursion here.
  if (has_hybrid_fmea_components(sys)) {
    FMEAOptions fo;
    fo.opf_options = opf_opt;
    fo.voll = opf_opt.voll;
    fo.load_scale_factor = 1.0;  // load scaling already applied to `sys` above
    NetworkShedResult ns = evaluate_failed_network_state(sys, fo);
    result.curtailment_mw = ns.total_shed_mw;
    result.nodal_curtailment_mw = std::move(ns.nodal_shed_mw);
    result.is_loss_state = ns.is_loss;
    return result;
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

    // Accumulate curtailment for dead islands.
    // P1b: DC-OPF adds bus.pd_mw and ac.loads as additive demand sources
    // (both are summed in the B-matrix); direct-shed must use the same
    // convention to avoid under-counting when a bus has both components.
    for (int bid : dead_buses) {
      auto it = bus_pos.find(bid);
      if (it == bus_pos.end()) continue;
      int pos = it->second;
      double load = std::max(0.0, sys.ac.buses[pos].pd_mw);
      direct_shed_mw += load;
      nodal_direct_shed[pos] += load;
    }
    for (auto& ld : sys.ac.loads) {
      if (!ld.in_service || !dead_buses.count(ld.bus)) continue;
      double extra = std::max(0.0, ld.p_mw * ld.scaling);
      direct_shed_mw += extra;
      if (auto it = bus_pos.find(ld.bus); it != bus_pos.end())
        nodal_direct_shed[it->second] += extra;
      ld.in_service = false;  // prevent OPF from seeing this load
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
      result.is_loss_state        = (result.curtailment_mw > curtail_threshold_mw);
      return result;
    }
  }

  // Run DC-OPF with load shedding enabled.
  // Guard: warn if the system has non-trivial DC loads (including DCBus::pd_mw)
  // that this AC-only OPF cannot model.
  {
    double total_dc_load_mw = total_inservice_dc_load_mw(sys);
    if (total_dc_load_mw > 0.01) {
      spdlog::warn("[FMEA] 系统含 DC 负荷 {:.3f} MW，但当前可靠性评估使用 AC-only DC-OPF。"
                   "DC 负荷中断不计入 OPF，EENS/LOLE 可能偏乐观。",
                   total_dc_load_mw);
    }
  }
  auto opf_result = opf::solve_dc_opf(sys, opf_opt);

  // P1a: OPF infeasibility — treat all remaining surviving-bus load as shed.
  // Covers: no generator on island, singular B-matrix, LP solver failure.
  // Silently returning zero curtailment for infeasible states inflates EENS.
  if (!opf_result.converged) {
    spdlog::warn("evaluate_state: DC-OPF failed to converge; "
                 "treating all surviving-island load as curtailed (conservative)");
    result.curtailment_mw = direct_shed_mw;
    result.nodal_curtailment_mw = std::move(nodal_direct_shed);
    result.nodal_curtailment_mw.resize(sys.ac.buses.size(), 0.0);
    // bus pd_mw was zeroed for dead-island buses above; remainder is live.
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      double load = std::max(0.0, sys.ac.buses[i].pd_mw);
      result.curtailment_mw += load;
      result.nodal_curtailment_mw[i] += load;
    }
    for (const auto& ld : sys.ac.loads) {
      if (!ld.in_service) continue;  // dead-island loads already disabled
      double load = std::max(0.0, ld.p_mw * ld.scaling);
      result.curtailment_mw += load;
      for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
        if (sys.ac.buses[i].index == ld.bus) {
          result.nodal_curtailment_mw[i] += load;
          break;
        }
      }
    }
    result.is_loss_state = (result.curtailment_mw > curtail_threshold_mw);
    return result;
  }

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
  result.is_loss_state = (result.curtailment_mw > curtail_threshold_mw);

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

  if (options.max_iterations <= 0) {
    spdlog::warn("NSQ MC: max_iterations={} <= 0; returning empty result",
                 options.max_iterations);
    return {};
  }

  ReliabilityResult result;
  // Hard-code structured capability declarations.  Flags stay false until the
  // state evaluator is upgraded to a true hybrid AC/DC OPF.  All call sites
  // that emit EENS/LOLE must set these consistently.
  result.model_scope = "ac-only-dcopf";
  result.validity = ReliabilityResult::ValidityFlags{};
  if (has_dc_rich_components(sys))
    result.model_limitations =
        "System contains DC/VSC components; AC-only OPF is used — "
        "DC power-flow constraints are not enforced. "
        "DC load curtailment is not captured in EENS/LOLE. "
        "DC components (DC/DC, DCCB, DC storage, DC PV, DC static gens, AC switch, AC CB) "
        "and AC PV systems are sampled in the state vector but failures affect only network "
        "topology \u2014 no DC power-flow re-dispatch is performed.";
  // Data-quality summary for the resolved reliability parameters (Finding 4).
  result.data_quality = summarize_reliability_data_quality(sys, options.data_policy);
  ComponentOffsets co(sys);
  const size_t nc = co.total;
  // Hybrid systems use the hybrid network LP per sampled state (DC load in EENS).
  const bool hybrid_mc = has_hybrid_fmea_components(sys);
  const size_t nb = sys.ac.buses.size() + (hybrid_mc ? sys.dc.buses.size() : 0);
  if (hybrid_mc) {
    result.model_scope = "hybrid-acdc-network-lp";
    result.validity = ReliabilityResult::ValidityFlags{};
    result.validity.dc_load_curtailment_included = true;
    result.validity.vsc_dc_power_flow_modelled = true;
    result.validity.ac_opf_curtailment = true;
    result.model_limitations =
        "Monte Carlo with the hybrid AC/DC network LP: AC/DC branch transfer "
        "limits, DC load shedding, DC sources, and DC/DC + VSC active-power "
        "transfers are included for each sampled state. Nonlinear AC "
        "voltage/reactive limits are outside this evaluator.";
  }

  // F10: unified per-component reliability using the SAME resolver and per-kind
  // template defaults as build_fmea_catalog, so NSQ/SEQ/FMEA agree on both
  // present-data and missing-data parameters (replaces the old inline fallback
  // constants that diverged across methods).
  const MCReliability mcr = compute_mc_reliability(sys, co, options.data_policy);
  std::vector<double> unavailabilities = mcr.U;

  spdlog::info("NSQ MC: {} total components "
               "(gen={}, br={}, sg={}, rg={}, st={}, vsc={}, db={}, t2={}, t3={}, "
               "dcdc={}, dccb={}, dcst={}, dcpv={}, sw={}, cb={}, pvs={}, dcsg={}, dcsgx={}, mg={})",
               nc, co.ng, co.nl, co.nsg, co.nrg, co.nst, co.nvsc, co.ndb, co.nt2, co.nt3,
               co.ndcdc, co.ndccb, co.ndcst, co.ndcpv, co.nsw, co.ncb,
               co.npvs, co.ndcsg, co.ndcsgx, co.nmg);

  // P2b: Zero unavailability for base-case out-of-service components.
  // Such components are already in their 'failed' state in the system baseline;
  // randomly sampling them would create meaningless failure events and pollute
  // critical-component statistics.  The FMEA catalog already skips !in_service;
  // MC now uses the same semantic: unavailability = 0 means 'baseline outage'.
  const auto active_component = build_active_component_mask(sys, co);
  for (size_t c = 0; c < nc; ++c) {
    if (!active_component[c]) unavailabilities[c] = 0.0;
  }

  // StrictCaseDataOnly: do not invent failure data.  Components without usable
  // case reliability data are treated as non-failing (unavailability = 0) so
  // EENS/LOLE reflect only documented component risk (Finding 1 & 2).
  if (options.data_policy.default_policy ==
      ReliabilityDefaultPolicy::StrictCaseDataOnly) {
    const auto has_data = mc_component_has_case_data(sys, co, options.data_policy);
    for (size_t c = 0; c < nc && c < has_data.size(); ++c) {
      if (!has_data[c]) unavailabilities[c] = 0.0;
    }
  }

  // Initialize random number generator
  std::mt19937 rng;
  if (options.seed != 0) {
    rng.seed(options.seed);
  } else {
    std::random_device rd;
    rng.seed(rd());
  }
  
  // DC-OPF options.  Reliability state evaluation minimises LOAD SHEDDING, so
  // VOLL is set to strongly dominate generation cost (lexicographic min-shed).
  opf::DCOPFOptions opf_opt = options.opf_options;
  opf_opt.load_shedding = true;
  opf_opt.voll = reliability_shedding_voll(sys, opf_opt.voll);
  opf_opt.verbose = false;
  opf_opt.compute_lmp = false;  // batch path — LMPs not needed, skip supporting LP
  
  // State database for deduplication.  Store the full packed bit pattern as the
  // key, not just its hash, so a hash collision cannot alias two outage states.
  std::unordered_map<StateKey, StateEvalResult, StateKeyHash> state_db;
  int n0_count = 0;  // Count of N-0 states
  int contingency_count = 0;  // Count of states with failures
  
  // Accumulators
  double sum_dns = 0.0;
  double sum_dns_sq = 0.0;
  int loss_hours = 0;
	  std::vector<double> nodal_dns_sum(nb, 0.0);
	  std::vector<double> comp_fail_count(nc, 0.0);
	  std::vector<double> comp_loss_weighted_sum(nc, 0.0);
	  double total_loss_weighted_sum = 0.0;
	  int total_loss_samples = 0;

  // F8: collect the per-sampled-state DNS and loss flag so tail risk can be
  // computed on bootstrap-aggregated SYNTHETIC YEARS (a statistically valid
  // annual-EENS distribution) rather than on the per-state dns*8760, which is
  // the distribution of one random hour annualized — not an annual quantity.
  std::vector<double> hourly_dns_samples;
  std::vector<std::uint8_t> hourly_loss_samples;
  if (options.compute_tail_risk) {
    hourly_dns_samples.reserve(static_cast<size_t>(options.max_iterations));
    hourly_loss_samples.reserve(static_cast<size_t>(options.max_iterations));
  }

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

  // P2a: Pre-compute the N-0 (all components healthy) baseline once.
  // We cannot assume zero curtailment even when load_scale_factor <= 1.0:
  // the base case may have islands, no-slack buses, or insufficient generation.
  // The cached result is reused for every sampled N-0 state (no redundant OPF).
  const StateEvalResult n0_result = evaluate_state(
      sys, std::vector<bool>(nc, false), opf_opt, options.load_scale_factor,
      options.curtail_threshold_mw);
  if (n0_result.curtailment_mw > 1e-6)
    spdlog::warn("NSQ MC: N-0 baseline has {:.3f} MW curtailment — "
                 "base-case infeasibility will affect all samples' EENS/LOLE",
                 n0_result.curtailment_mw);

  auto consume_nsq_sample = [&](int iter,
                                const std::vector<bool>& state,
                                const StateEvalResult& eval_result) {
    // Accumulate results
    double dns = eval_result.curtailment_mw;
    sum_dns += dns;
    sum_dns_sq += dns * dns;

    // F8: record the raw per-state DNS and loss flag; annual samples are built by
    // bootstrap aggregation after the loop (a per-state dns*8760 is NOT annual).
    if (options.compute_tail_risk) {
      hourly_dns_samples.push_back(dns);
      hourly_loss_samples.push_back(eval_result.is_loss_state ? 1u : 0u);
    }
    
	    if (eval_result.is_loss_state) {
	      ++loss_hours;
	      ++total_loss_samples;
	      total_loss_weighted_sum += dns;
	      
	      // Accumulate nodal curtailment
	      for (size_t b = 0; b < nb && b < eval_result.nodal_curtailment_mw.size(); ++b) {
	        nodal_dns_sum[b] += eval_result.nodal_curtailment_mw[b];
	      }
	      
	      // Track component failures during loss.  The conditional count is a
	      // diagnostic, while the MW-weighted sum is the risk ranking aligned
	      // with FMEA EENS contribution.
	      for (size_t c = 0; c < nc; ++c) {
	        if (state[c]) {
	          comp_fail_count[c] += 1.0;
	          comp_loss_weighted_sum[c] += dns;
	        }
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
        result.iterations_used = iter;  // P2a: ensure denominator is valid
        return false;
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
      return false;
    }
    
    result.iterations_used = iter;
    return true;
  };

  const int nsq_worker_items =
      std::min(options.max_iterations, std::max(1, options.parallel_threads > 0
                                                     ? options.parallel_threads * 4
                                                     : 64));
  const int nsq_workers = options.enable_parallel
      ? resolve_reliability_worker_count(options.parallel_threads, nsq_worker_items)
      : 1;
  result.parallel_workers = nsq_workers;
  result.parallel_effective = options.enable_parallel && nsq_workers > 1 &&
                              options.max_iterations > 1;
  result.parallel_mode = result.parallel_effective
      ? "parallel-nsq-state-batches"
      : (options.enable_parallel ? "serial/insufficient-work" : "serial/disabled");
  result.parallel_execution = util::make_parallel_execution_info(
      options.enable_parallel, options.parallel_threads, nsq_worker_items,
      "parallel-nsq-state-batches", "state-cache");
  result.parallel_execution.effective = result.parallel_effective;
  result.parallel_execution.resolved_workers = nsq_workers;
  result.parallel_execution.mode = result.parallel_mode;
  if (!result.parallel_effective && options.enable_parallel) {
    result.parallel_execution.guard_reason =
        util::insufficient_work_reason(result.parallel_execution);
  }
  std::unique_ptr<util::ThreadPool> nsq_pool;
  if (result.parallel_effective) {
    nsq_pool = std::make_unique<util::ThreadPool>(nsq_workers);
    spdlog::info("NSQ MC: evaluating cache-miss states in parallel with {} workers",
                 nsq_workers);
  }

  struct NSQSampleWork {
    std::vector<bool> state;
    StateKey key;
    StateEvalResult eval;
    bool is_n0{false};
    bool cache_miss{false};
    int num_failures{0};
  };

  const int nsq_batch_size = result.parallel_effective
      ? std::max(1, nsq_workers * 4)
      : 1;
  result.parallel_execution.batch_size = nsq_batch_size;
  long long nsq_cache_hits = 0;
  long long nsq_cache_misses = 0;
  long long nsq_parallel_evals = 0;
  long long nsq_serial_evals = 0;

  // Main simulation loop.  Sampling and reduction stay in iteration order;
  // only expensive cache-miss state evaluations are dispatched to workers.
  for (int batch_start = 1; batch_start <= options.max_iterations; ) {
    const int batch_end = std::min(options.max_iterations,
                                   batch_start + nsq_batch_size - 1);
    std::vector<NSQSampleWork> batch(
        static_cast<size_t>(batch_end - batch_start + 1));
    std::vector<size_t> miss_indices;

    for (size_t bi = 0; bi < batch.size(); ++bi) {
      auto& item = batch[bi];
      item.state = sample_state(unavailabilities, rng);
      item.is_n0 = std::none_of(item.state.begin(), item.state.end(),
                                [](bool b) { return b; });
      if (item.is_n0) {
        ++n0_count;
        item.eval = n0_result;
        continue;
      }

      ++contingency_count;
      for (size_t c = 0; c < nc; ++c) {
        if (item.state[c]) ++item.num_failures;
      }
      item.key = pack_state_key(item.state);
      auto it = state_db.find(item.key);
      if (it != state_db.end()) {
        item.eval = it->second;
        ++nsq_cache_hits;
      } else {
        item.cache_miss = true;
        miss_indices.push_back(bi);
      }
    }

    if (!miss_indices.empty()) {
      nsq_cache_misses += static_cast<long long>(miss_indices.size());
      auto eval_missing = [&](size_t begin, size_t end) {
        for (size_t pos = begin; pos < end; ++pos) {
          auto& item = batch[miss_indices[pos]];
          item.eval = evaluate_state(sys, item.state, opf_opt,
                                     options.load_scale_factor,
                                     options.curtail_threshold_mw);
        }
      };
      if (result.parallel_effective && nsq_pool && miss_indices.size() > 1U) {
        nsq_parallel_evals += static_cast<long long>(miss_indices.size());
        nsq_pool->parallel_for_dynamic(
            miss_indices.size(),
            [&](size_t pos) {
              auto& item = batch[miss_indices[pos]];
              item.eval = evaluate_state(sys, item.state, opf_opt,
                                         options.load_scale_factor,
                                         options.curtail_threshold_mw);
            },
            nsq_workers);
      } else {
        nsq_serial_evals += static_cast<long long>(miss_indices.size());
        eval_missing(0, miss_indices.size());
      }

      for (size_t bi : miss_indices) {
        auto& item = batch[bi];
        auto [it, inserted] = state_db.emplace(item.key, item.eval);
        if (!inserted) item.eval = it->second;
        if (inserted && state_db.size() <= 5) {
          spdlog::info("NSQ MC: State #{} - {} failures -> shed={:.2f} MW",
                       state_db.size(), item.num_failures,
                       item.eval.curtailment_mw);
        }
      }
    }

    bool stop = false;
    for (size_t bi = 0; bi < batch.size(); ++bi) {
      const int iter = batch_start + static_cast<int>(bi);
      if (!consume_nsq_sample(iter, batch[bi].state, batch[bi].eval)) {
        stop = true;
        break;
      }
    }
    if (stop) break;
    batch_start = batch_end + 1;
  }
  
  // Log summary of sampling
  spdlog::info("NSQ MC: Sampling summary - N-0 states: {}, Contingency states: {}, Unique states evaluated: {}",
               n0_count, contingency_count, state_db.size());
  spdlog::info("NSQ MC: Loss states: {}, Total loss samples: {}", loss_hours, total_loss_samples);
  result.parallel_execution.cache_hits = nsq_cache_hits;
  result.parallel_execution.cache_misses = nsq_cache_misses;
  result.parallel_execution.n0_evaluations = n0_count;
  result.parallel_execution.actual_parallel_evaluations = nsq_parallel_evals;
  result.parallel_execution.serial_evaluations = nsq_serial_evals;
  
  // Final results
  int n = result.iterations_used;
  if (n <= 0) {
    spdlog::warn("NSQ MC: No iterations completed; metrics are undefined");
    return result;
  }
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
	      const double conditional = comp_fail_count[c] / total_loss_samples;
	      const double loss_weighted =
	          total_loss_weighted_sum > 0.0
	              ? comp_loss_weighted_sum[c] / total_loss_weighted_sum
	              : 0.0;
	      if (loss_weighted > 0.01 || conditional > 0.01) {
	        ReliabilityResult::ComponentImportance ci;
	        auto [idx, type_name] = decode_component(c, co);
	        ci.index               = idx;
	        ci.global_state_index  = c;
	        ci.component_type      = type_name;
	        ci.component_name      = resolve_component_name(idx, type_name, sys);
	        ci.is_generator        = (c < co.off_br);
	        ci.importance          = loss_weighted;
	        ci.loss_weighted_risk  = loss_weighted;
	        ci.conditional_down_given_loss = conditional;
	        ci.associated_eens_mwh_yr = (comp_loss_weighted_sum[c] / n) * 8760.0;
	        result.critical_components.push_back(ci);
	      }
	    }
	    // Sort by the risk metric, not by P(component down | loss).  This aligns
	    // Monte Carlo "critical components" with FMEA's EENS-contribution ranking.
	    std::sort(result.critical_components.begin(), result.critical_components.end(),
	              [](const auto& a, const auto& b) {
	                if (a.loss_weighted_risk != b.loss_weighted_risk) {
	                  return a.loss_weighted_risk > b.loss_weighted_risk;
	                }
	                return a.conditional_down_given_loss > b.conditional_down_given_loss;
	              });
	  }
  
  spdlog::info("NSQ MC: Complete. EENS={:.2f} MWh/yr, LOLE={:.2f} hr/yr, PLC={:.4f}",
               result.eens_mwh_yr, result.lole_hr_yr, result.plc);
  
  // 鈹€鈹€鈹€ Compute Tail Risk Metrics (F8: bootstrap synthetic years) 鈹€鈹€鈹€
  // Non-sequential samples are i.i.d. system states, each standing for one random
  // hour.  A synthetic year's EENS is the sum of 8760 such hourly DNS draws; its
  // distribution (tight, by the CLT) is the correct object for VaR/CVaR.  We
  // resample the observed hourly DNS/loss with replacement to form K synthetic
  // years, then take tail metrics over them.  Mean(annual_eens) == eens_mwh_yr and
  // mean(annual_lole) == lole_hr_yr by construction, so the summary is unchanged.
  if (options.compute_tail_risk && !hourly_dns_samples.empty()) {
    constexpr int kSynthYearHours = 8760;
    const int n_years = std::min(std::max(200, result.iterations_used), 2000);
    std::uniform_int_distribution<size_t> pick(0, hourly_dns_samples.size() - 1);
    result.annual_eens.clear();
    result.annual_lole.clear();
    result.annual_eens.reserve(static_cast<size_t>(n_years));
    result.annual_lole.reserve(static_cast<size_t>(n_years));
    for (int y = 0; y < n_years; ++y) {
      double eens_y = 0.0;
      double loss_hours_y = 0.0;
      for (int h = 0; h < kSynthYearHours; ++h) {
        const size_t idx = pick(rng);
        eens_y += hourly_dns_samples[idx];          // MW * 1 hr -> MWh
        loss_hours_y += hourly_loss_samples[idx];   // loss-hour indicator
      }
      result.annual_eens.push_back(eens_y);
      result.annual_lole.push_back(loss_hours_y);
    }
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

  if (options.max_iterations <= 0) {
    spdlog::warn("SEQ MC: max_iterations={} <= 0; returning empty result",
                 options.max_iterations);
    return {};
  }

  if (options.hours_per_year <= 0) {
    spdlog::warn("SEQ MC: hours_per_year={} <= 0; returning empty result",
                 options.hours_per_year);
    return {};
  }

  const HybridPowerSystem spatial_sys =
      apply_load_profile_spatial_factors(sys, load_profile);

  ReliabilityResult result;
  result.model_scope = "ac-only-dcopf";
  result.validity = ReliabilityResult::ValidityFlags{};
  if (has_dc_rich_components(sys))
    result.model_limitations =
        "System contains DC/VSC components; AC-only OPF is used — "
        "DC power-flow constraints are not enforced. "
        "DC load curtailment is not captured in EENS/LOLE. "
        "DC components (DC/DC, DCCB, DC storage, DC PV, DC static gens, AC switch, AC CB) "
        "and AC PV systems are sampled in the state vector but failures affect only network "
        "topology \u2014 no DC power-flow re-dispatch is performed.";
  // Data-quality summary for the resolved reliability parameters (Finding 4).
  result.data_quality = summarize_reliability_data_quality(sys, options.data_policy);
  ComponentOffsets co(sys);
  const size_t nc = co.total;
  // Hybrid systems use the hybrid network LP per sampled state (DC load in EENS).
  const bool hybrid_mc = has_hybrid_fmea_components(sys);
  const size_t nb = sys.ac.buses.size() + (hybrid_mc ? sys.dc.buses.size() : 0);
  if (hybrid_mc) {
    result.model_scope = "hybrid-acdc-network-lp";
    result.validity = ReliabilityResult::ValidityFlags{};
    result.validity.dc_load_curtailment_included = true;
    result.validity.vsc_dc_power_flow_modelled = true;
    result.validity.ac_opf_curtailment = true;
    result.model_limitations =
        "Sequential Monte Carlo with the hybrid AC/DC network LP: AC/DC branch "
        "transfer limits, DC load shedding, DC sources, and DC/DC + VSC "
        "active-power transfers are included for each chronological state.";
  }
  const int hours_per_year = options.hours_per_year;
  
  // F10: unified per-component MTTF/MTTR via the shared resolver (same per-kind
  // template defaults as build_fmea_catalog), replacing the old inline fallback
  // constants so SEQ agrees with NSQ and FMEA on identical components.
  const MCReliability mcr = compute_mc_reliability(sys, co, options.data_policy);
  std::vector<double> mttf = mcr.mttf;
  std::vector<double> mttr_v = mcr.mttr;

  const auto active_component = build_active_component_mask(sys, co);

  // StrictCaseDataOnly: components without usable case reliability data are
  // assigned an effectively infinite MTTF so they never fail in the simulated
  // horizon (no invented failure data — Finding 1 & 2).
  if (options.data_policy.default_policy ==
      ReliabilityDefaultPolicy::StrictCaseDataOnly) {
    const auto has_data = mc_component_has_case_data(sys, co, options.data_policy);
    for (size_t c = 0; c < nc && c < has_data.size(); ++c) {
      if (!has_data[c]) mttf[c] = 1e15;
    }
  }

  std::mt19937 rng;
  if (options.seed != 0) {
    rng.seed(options.seed);
  } else {
    std::random_device rd;
    rng.seed(rd());
  }
  
  // DC-OPF options.  Reliability state evaluation minimises LOAD SHEDDING, so
  // VOLL is set to strongly dominate generation cost (lexicographic min-shed).
  opf::DCOPFOptions opf_opt = options.opf_options;
  opf_opt.load_shedding = true;
  opf_opt.voll = reliability_shedding_voll(sys, opf_opt.voll);
  opf_opt.verbose = false;
  opf_opt.compute_lmp = false;  // batch path — LMPs not needed, skip supporting LP
  
  // Accumulators
	  std::vector<double> nodal_eens_accum(nb, 0.0);
	  std::vector<double> comp_fail_during_loss(nc, 0.0);
	  std::vector<double> comp_loss_during_loss(nc, 0.0);
	  double total_loss_weighted_mwh = 0.0;
	  int total_loss_hours = 0;
  
  spdlog::info("SEQ MC: {} total components, {} hours/year", nc, hours_per_year);

  const int seq_workers = options.enable_parallel
      ? resolve_reliability_worker_count(options.parallel_threads, hours_per_year)
      : 1;
  result.parallel_workers = seq_workers;
  result.parallel_effective = options.enable_parallel && seq_workers > 1 &&
                              hours_per_year > 1;
  result.parallel_mode = result.parallel_effective
      ? "parallel-seq-hourly-states"
      : (options.enable_parallel ? "serial/insufficient-work" : "serial/disabled");
  result.parallel_execution = util::make_parallel_execution_info(
      options.enable_parallel, options.parallel_threads, hours_per_year,
      "parallel-seq-hourly-states", "hourly-states");
  result.parallel_execution.effective = result.parallel_effective;
  result.parallel_execution.resolved_workers = seq_workers;
  result.parallel_execution.mode = result.parallel_mode;
  if (!result.parallel_effective && options.enable_parallel) {
    result.parallel_execution.guard_reason =
        util::insufficient_work_reason(result.parallel_execution);
  }
  std::unique_ptr<util::ThreadPool> seq_pool;
  if (result.parallel_effective) {
    seq_pool = std::make_unique<util::ThreadPool>(seq_workers);
    spdlog::info("SEQ MC: evaluating hourly states in parallel with {} workers",
                 seq_workers);
  }
  long long seq_parallel_evals = 0;
  long long seq_serial_evals = 0;
  long long seq_n0_hours = 0;

  std::unordered_map<long long, StateEvalResult> n0_cache;
  auto n0_for_hour = [&](int h) -> const StateEvalResult& {
    const double load_scale = hourly_load_scale(load_profile, options, h);
    const long long key = load_scale_cache_key(load_scale);
    auto [it, inserted] = n0_cache.emplace(key, StateEvalResult{});
    if (inserted) {
      it->second = evaluate_state(spatial_sys, std::vector<bool>(nc, false), opf_opt, load_scale,
                                   options.curtail_threshold_mw);
      if (it->second.curtailment_mw > 1e-6) {
        spdlog::warn("SEQ MC: N-0 baseline at load_scale={:.6f} has {:.3f} MW curtailment",
                     load_scale, it->second.curtailment_mw);
      }
    }
    return it->second;
  };
  
  // Main simulation loop (years)
  for (int year = 1; year <= options.max_iterations; ++year) {
    // Generate chronological state for each component
    // state_matrix[comp][hour] = true if component is DOWN
    std::vector<std::vector<bool>> state_matrix(nc, std::vector<bool>(hours_per_year, false));
    
    for (size_t c = 0; c < nc; ++c) {
      if (!active_component[c]) continue;
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
    
    if (options.verbose && year % 100 == 0) {
      int contingency_hours = 0;
      for (int h = 0; h < hours_per_year; ++h) {
        for (size_t c = 0; c < nc; ++c) {
          if (state_matrix[c][h]) {
            ++contingency_hours;
            break;
          }
        }
      }
      spdlog::info("SEQ MC: Year {} has {} contingency hours",
                   year, contingency_hours);
    }
    
    // Evaluate every hour.  N-0 hours are not assumed to have zero shed: the base
    // system can already be islanded, capacity-short, or stressed by load profile.
    double year_eens = 0.0;
    int year_loss_hours = 0;
    std::vector<bool> year_loss_flags(hours_per_year, false);

    struct SEQHourWork {
      std::vector<bool> state;
      StateEvalResult eval;
      bool any_down{false};
      double load_scale{1.0};
    };
    std::vector<SEQHourWork> hour_work(static_cast<size_t>(hours_per_year));
    std::vector<size_t> down_hours;
    for (int h = 0; h < hours_per_year; ++h) {
      auto& hw = hour_work[static_cast<size_t>(h)];
      hw.state.assign(nc, false);
      for (size_t c = 0; c < nc; ++c) {
        hw.state[c] = state_matrix[c][h];
        hw.any_down = hw.any_down || hw.state[c];
      }
      hw.load_scale = hourly_load_scale(load_profile, options, h);
      if (hw.any_down) {
        down_hours.push_back(static_cast<size_t>(h));
      } else {
        hw.eval = n0_for_hour(h);
      }
    }

    auto eval_down_hours = [&](size_t begin, size_t end) {
      for (size_t pos = begin; pos < end; ++pos) {
        auto& hw = hour_work[down_hours[pos]];
        hw.eval = evaluate_state(spatial_sys, hw.state, opf_opt, hw.load_scale,
                                 options.curtail_threshold_mw);
      }
    };
    if (result.parallel_effective && seq_pool && down_hours.size() > 1U) {
      seq_parallel_evals += static_cast<long long>(down_hours.size());
      seq_pool->parallel_for_dynamic(
          down_hours.size(),
          [&](size_t pos) {
            auto& hw = hour_work[down_hours[pos]];
            hw.eval = evaluate_state(spatial_sys, hw.state, opf_opt, hw.load_scale,
                                     options.curtail_threshold_mw);
          },
          seq_workers);
    } else if (!down_hours.empty()) {
      seq_serial_evals += static_cast<long long>(down_hours.size());
      eval_down_hours(0, down_hours.size());
    }
    seq_n0_hours += static_cast<long long>(hours_per_year) -
                    static_cast<long long>(down_hours.size());

    for (int h = 0; h < hours_per_year; ++h) {
      const auto& state = hour_work[static_cast<size_t>(h)].state;
      const auto& eval_result = hour_work[static_cast<size_t>(h)].eval;
      
	      if (eval_result.is_loss_state) {
	        year_eens += eval_result.curtailment_mw;
	        ++year_loss_hours;
	        year_loss_flags[h] = true;
	        total_loss_weighted_mwh += eval_result.curtailment_mw;
	        
	        // Accumulate nodal EENS
	        for (size_t b = 0; b < nb && b < eval_result.nodal_curtailment_mw.size(); ++b) {
	          nodal_eens_accum[b] += eval_result.nodal_curtailment_mw[b];
	        }
	        
	        // Track component failures during loss.  Keep both the conditional
	        // probability diagnostic and the loss-weighted risk ranking.
	        for (size_t c = 0; c < nc; ++c) {
	          if (state[c]) {
	            comp_fail_during_loss[c] += 1.0;
	            comp_loss_during_loss[c] += eval_result.curtailment_mw;
	          }
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
        result.iterations_used = year;  // P2a: ensure denominator is valid
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
  if (n_years <= 0) {
    spdlog::warn("SEQ MC: No years completed; metrics are undefined");
    return result;
  }
  result.eens_mwh_yr = std::accumulate(result.annual_eens.begin(),
                                        result.annual_eens.end(), 0.0) / n_years;
  result.lole_hr_yr = std::accumulate(result.annual_lole.begin(),
                                       result.annual_lole.end(), 0.0) / n_years;
  result.lolf_occ_yr = std::accumulate(result.annual_lolf.begin(),
                                        result.annual_lolf.end(), 0.0) / n_years;
  result.edns_mw = result.eens_mwh_yr / static_cast<double>(options.hours_per_year);
  result.plc = result.lole_hr_yr / static_cast<double>(options.hours_per_year);
  result.final_cov = result.cov_history.empty() ? 0.0 : result.cov_history.back();
  result.parallel_execution.actual_parallel_evaluations = seq_parallel_evals;
  result.parallel_execution.serial_evaluations = seq_serial_evals;
  result.parallel_execution.n0_evaluations = seq_n0_hours;
  
  // Nodal EENS
  result.nodal_eens_mwh_yr.resize(nb);
  for (size_t b = 0; b < nb; ++b) {
    result.nodal_eens_mwh_yr[b] = nodal_eens_accum[b] / n_years;
  }
  
	  // Critical components
	  if (total_loss_hours > 0) {
	    for (size_t c = 0; c < nc; ++c) {
	      const double conditional = comp_fail_during_loss[c] / total_loss_hours;
	      const double loss_weighted =
	          total_loss_weighted_mwh > 0.0
	              ? comp_loss_during_loss[c] / total_loss_weighted_mwh
	              : 0.0;
	      if (loss_weighted > 0.01 || conditional > 0.01) {
	        ReliabilityResult::ComponentImportance ci;
	        auto [idx, type_name] = decode_component(c, co);
	        ci.index               = idx;
	        ci.global_state_index  = c;
	        ci.component_type      = type_name;
	        ci.component_name      = resolve_component_name(idx, type_name, sys);
	        ci.is_generator        = (c < co.off_br);
	        ci.importance          = loss_weighted;
	        ci.loss_weighted_risk  = loss_weighted;
	        ci.conditional_down_given_loss = conditional;
	        ci.associated_eens_mwh_yr = comp_loss_during_loss[c] / n_years;
	        result.critical_components.push_back(ci);
	      }
	    }
	    std::sort(result.critical_components.begin(), result.critical_components.end(),
	              [](const auto& a, const auto& b) {
	                if (a.loss_weighted_risk != b.loss_weighted_risk) {
	                  return a.loss_weighted_risk > b.loss_weighted_risk;
	                }
	                return a.conditional_down_given_loss > b.conditional_down_given_loss;
	              });
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

  // M4: guard against divide-by-zero in ASAI calculation (idx.saidi / hours_per_year).
  if (hours_per_year <= 0) {
    spdlog::warn("compute_distribution_indices: hours_per_year={} <= 0; defaulting to 8760",
                 hours_per_year);
    hours_per_year = 8760;
  }
  
  // Get customer counts from loads
  // Note: In our model, we use load.p_mw as a proxy for customers if num_customers not available.
  //
  // Hybrid layout (code-review Finding 3): the nodal CIF/CID vectors produced by
  // the FMEA evaluator are laid out as [AC buses | DC buses].  When the caller
  // passes DC nodes (vector longer than the AC bus count) we must weight DC bus
  // customers too, otherwise DC load interruptions never reach SAIFI/SAIDI/ASAI.
  const bool include_dc =
      std::max(nodal_cif.size(), nodal_cid.size()) > sys.ac.buses.size();
  const size_t n_dc = include_dc ? sys.dc.buses.size() : 0U;

  std::vector<double> customers_per_bus(sys.ac.buses.size() + n_dc, 0.0);
  double total_customers = 0.0;

  // P2a: bus IDs may be non-contiguous; build a position map so that
  // ld.bus is looked up correctly rather than using the unsafe ld.bus-1 offset.
  std::unordered_map<int, size_t> bp_map;
  bp_map.reserve(sys.ac.buses.size());
  for (size_t i = 0; i < sys.ac.buses.size(); ++i)
    bp_map[sys.ac.buses[i].index] = i;

  for (const auto& ld : sys.ac.loads) {
    if (!ld.in_service || ld.bus < 1) continue;
    const auto it = bp_map.find(ld.bus);
    if (it == bp_map.end() || it->second >= sys.ac.buses.size()) continue;
    // Use n_customers if available, otherwise estimate from load
    double nc = ld.n_customers > 0 ? ld.n_customers : std::max(1.0, ld.p_mw * 10.0);
    customers_per_bus[it->second] += nc;
    total_customers += nc;
  }
  
  // For AC buses without explicit loads, add customers based on pd_mw
  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    if (customers_per_bus[i] == 0.0 && sys.ac.buses[i].pd_mw > 0) {
      double nc = std::max(1.0, sys.ac.buses[i].pd_mw * 10.0);  // ~10 customers per MW
      customers_per_bus[i] = nc;
      total_customers += nc;
    }
  }

  // DC customer weights, appended after the AC block to match the nodal layout.
  if (include_dc) {
    const size_t off = sys.ac.buses.size();
    std::unordered_map<int, size_t> dbp_map;
    dbp_map.reserve(sys.dc.buses.size());
    for (size_t j = 0; j < sys.dc.buses.size(); ++j)
      dbp_map[sys.dc.buses[j].index] = j;

    // Aggregate DC load MW and explicit DC load customers per DC bus.
    std::vector<double> dc_load_mw(sys.dc.buses.size(), 0.0);
    std::vector<double> dc_load_cust(sys.dc.buses.size(), 0.0);
    for (const auto& ld : sys.dc.loads) {
      if (!ld.in_service) continue;
      const auto it = dbp_map.find(ld.bus);
      if (it == dbp_map.end()) continue;
      double scaling = ld.scaling != 0.0 ? ld.scaling : 1.0;
      dc_load_mw[it->second] += std::abs(ld.p_mw) * scaling;
      if (ld.n_customers > 0)
        dc_load_cust[it->second] += static_cast<double>(ld.n_customers);
    }
    for (size_t j = 0; j < sys.dc.buses.size(); ++j) {
      const auto& b = sys.dc.buses[j];
      double nc = 0.0;
      if (b.n_customers > 0) {
        nc = static_cast<double>(b.n_customers);          // DC bus-level count
      } else if (dc_load_cust[j] > 0.0) {
        nc = dc_load_cust[j];                              // explicit DC load count
      } else {
        double mw = dc_load_mw[j] + std::max(0.0, b.pd_mw);
        if (mw > 0.0) nc = std::max(1.0, mw * 10.0);       // ~10 customers per MW
      }
      customers_per_bus[off + j] = nc;
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

// ═══════════════════════════════════════════════════════════════════════
// Reliability data-quality summary (code-review Finding 4)
// ═══════════════════════════════════════════════════════════════════════
// Scans every component that carries reliability fields and reports how many
// have usable case data, how many would be defaulted, and which are missing,
// using the same resolver as the assessment methods.  Surfaced to the API/GUI
// pre-run diagnostics and result panels.
ReliabilityDataQuality summarize_reliability_data_quality(
    const HybridPowerSystem& sys,
    const ReliabilityDataPolicy& policy) {
  ReliabilityDataQuality dq;

  auto tally = [&](const ReliabilityRawFields& raw, const std::string& name) {
    // def_lambda/def_repair = 0 → the resolver reports "missing" for no-data
    // components instead of silently filling, giving an honest coverage signal.
    ReliabilityParams pr = resolve_reliability_params(raw, policy, 0.0, 0.0);
    dq.components_total++;
    if (pr.data_source == "missing") {
      dq.missing_required_data.push_back(name);
    } else if (pr.used_default) {
      dq.components_defaulted++;
    } else {
      dq.components_with_reliability_data++;
    }
  };
  auto nm = [](const std::string& n, const char* p, int idx) {
    return n.empty() ? (std::string(p) + std::to_string(idx)) : n;
  };

  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    ReliabilityRawFields r;
    r.forced_outage_rate = g.forced_outage_rate;
    r.mttr_hr = g.mttr_hr;
    tally(r, nm(g.name, "Gen_", g.index));
  }
  for (const auto& b : sys.ac.branches) {
    if (!b.in_service) continue;
    ReliabilityRawFields r;
    r.failure_rate_per_year = b.failure_rate;
    r.mttr_hr = b.mttr_hr;
    tally(r, nm(b.name, "ACBr_", b.index));
  }
  for (const auto& t : sys.ac.transformers_2w) {
    if (!t.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = t.mtbf_hours;
    r.mttr_hours = t.mttr_hours;
    tally(r, nm(t.name, "Trafo2W_", t.index));
  }
  for (const auto& t : sys.ac.transformers_3w) {
    if (!t.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = t.mtbf_hours;
    r.mttr_hours = t.mttr_hours;
    tally(r, nm(t.name, "Trafo3W_", t.index));
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = sg.mtbf_hours;
    r.mttr_hours = sg.mttr_hours;
    tally(r, nm(sg.name, "SGen_", sg.index));
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = rg.mtbf_hours;
    r.mttr_hours = rg.mttr_hours;
    tally(r, nm(rg.name, "RGen_", rg.index));
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    ReliabilityRawFields r;
    r.forced_outage_rate = st.forced_outage_rate;
    r.mttr_hr = st.mttr_hr;
    tally(r, nm(st.name, "BESS_", st.index));
  }
  for (const auto& v : sys.vsc_converters) {
    if (!v.in_service) continue;
    ReliabilityRawFields r;
    r.forced_outage_rate = v.forced_outage_rate;
    r.mttr_hr = v.mttr_hr;
    tally(r, nm(v.name, "VSC_", v.index));
  }
  for (const auto& b : sys.dc.branches) {
    if (!b.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = b.mtbf_hours;
    r.mttr_hours = b.mttr_hours;
    tally(r, nm(b.name, "DCBr_", b.index));
  }
  for (const auto& d : sys.dc.dcdc_converters) {
    if (!d.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = d.mtbf_hours;
    r.mttr_hours = d.mttr_hours;
    tally(r, nm(d.name, "DCDC_", d.index));
  }
  for (const auto& st : sys.dc.storage) {
    if (!st.in_service) continue;
    ReliabilityRawFields r;
    r.forced_outage_rate = st.forced_outage_rate;
    r.mttr_hr = st.mttr_hr;
    tally(r, nm(st.name, "DCStorage_", st.index));
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = pv.mtbf_hours;
    r.mttr_hours = pv.mttr_hours;
    tally(r, nm(pv.name, "DCPV_", pv.index));
  }
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = sg.mtbf_hours;
    r.mttr_hours = sg.mttr_hours;
    tally(r, nm(sg.name, "DCSGen_", sg.index));
  }
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = sg.mtbf_hours;
    r.mttr_hours = sg.mttr_hours;
    tally(r, nm(sg.name, "DCSGen2_", sg.index));
  }
  for (const auto& sw : sys.ac.switches) {
    if (!sw.in_service) continue;
    ReliabilityRawFields r;
    r.mtbf_hours = sw.mtbf_hours;
    r.mttr_hours = sw.mttr_hours;
    if (sw.mtbf_hours <= 0 && sw.p_sw_fail > 0 && sw.p_sw_fail < 1.0) {
      r.failure_rate_per_year = sw.p_sw_fail;
      r.mttr_hr = sw.mttr_hours > 0 ? sw.mttr_hours : 4.0;
    }
    tally(r, nm(sw.name, "SW_", sw.index));
  }

  return dq;
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
              ACSwitch, ACCB,
              ACPVSystem,    ///< ac.pv_systems (PVSystem)
              DCStaticGenAC  ///< dc.static_generators (StaticGenerator type in DC container)
  };
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
    case FMEAComponent::ACPVSystem:    return "ac_pv_system";
    case FMEAComponent::DCStaticGenAC: return "dc_static_generator_ac";
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
    case SwitchType::Unknown: return "unknown";
  }
  return "switch";
}

// Build the component catalog from the system.
//
// All per-component failure frequency / repair time values are now produced by
// the single `resolve_reliability_params` resolver (code-review Finding 1) so
// that FMEA, Monte-Carlo, and three-stage paths agree for identical components.
// `policy` controls missing-data handling and `dq` accumulates the data-quality
// summary surfaced to the API/GUI.
std::vector<FMEAComponent> build_fmea_catalog(
    const HybridPowerSystem& sys,
    double default_sw_hr,
    const ReliabilityDataPolicy& policy,
    ReliabilityDataQuality& dq) {

  std::vector<FMEAComponent> catalog;

  // Resolve one component's reliability params, update the data-quality
  // tally, and return {lambda_per_year, repair_hr}.  `def_lambda`/`def_repair`
  // are the per-kind template fallbacks used only when the policy permits
  // defaulting and case data is incomplete.
  auto resolve_cat = [&](const ReliabilityRawFields& raw,
                         double def_lambda, double def_repair,
                         const std::string& name) -> std::pair<double, double> {
    ReliabilityParams pr =
        resolve_reliability_params(raw, policy, def_lambda, def_repair);
    dq.components_total++;
    if (pr.data_source == "missing") {
      dq.missing_required_data.push_back(name);
    } else if (pr.used_default) {
      dq.components_defaulted++;
    } else {
      dq.components_with_reliability_data++;
    }
    double repair = pr.repair_hr > 0.0 ? pr.repair_hr : def_repair;
    return {pr.lambda_per_year, repair};
  };

  // ---- AC Generators ----
  for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
    const auto& g = sys.ac.generators[i];
    if (!g.in_service) continue;
    
    FMEAComponent c;
    c.type = FMEAComponent::Generator;
    c.idx = static_cast<int>(i);
    c.name = g.name.empty() ? "Gen_" + std::to_string(g.index) : g.name;
    
    // Resolve FOR + MTTR -> {lambda, repair} via the unified resolver.
    ReliabilityRawFields raw;
    raw.forced_outage_rate = g.forced_outage_rate;
    raw.mttr_hr = g.mttr_hr;
    auto pr = resolve_cat(raw, 8760.0 / 2000.0, 50.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.failure_rate_per_year = b.failure_rate;
    raw.mttr_hr = b.mttr_hr;
    auto pr = resolve_cat(raw, 0.35, 10.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.mtbf_hours = b.mtbf_hours;
    raw.mttr_hours = b.mttr_hours;
    auto pr = resolve_cat(raw, 0.20, 24.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.forced_outage_rate = v.forced_outage_rate;
    raw.mttr_hr = v.mttr_hr;
    auto pr = resolve_cat(raw, 0.10, 48.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.mtbf_hours = sg.mtbf_hours;
    raw.mttr_hours = sg.mttr_hours;
    auto pr = resolve_cat(raw, 1.5, 24.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.mtbf_hours = rg.mtbf_hours;
    raw.mttr_hours = rg.mttr_hours;
    auto pr = resolve_cat(raw, 2.0, 48.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.forced_outage_rate = st.forced_outage_rate;
    raw.mttr_hr = st.mttr_hr;
    auto pr = resolve_cat(raw, 1.0, 24.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.mtbf_hours = t.mtbf_hours;
    raw.mttr_hours = t.mttr_hours;
    auto pr = resolve_cat(raw, 0.03, 200.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    
    ReliabilityRawFields raw;
    raw.mtbf_hours = t.mtbf_hours;
    raw.mttr_hours = t.mttr_hours;
    auto pr = resolve_cat(raw, 0.04, 200.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    ReliabilityRawFields raw;
    raw.mtbf_hours = d.mtbf_hours;
    raw.mttr_hours = d.mttr_hours;
    auto pr = resolve_cat(raw, 0.20, 48.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    // DCCircuitBreaker has no mtbf/mttr fields; resolver applies CB defaults.
    auto pr = resolve_cat(ReliabilityRawFields{}, 0.10, 8.0, c.name);
    c.lambda     = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    ReliabilityRawFields raw;
    raw.forced_outage_rate = st.forced_outage_rate;
    raw.mttr_hr = st.mttr_hr;
    auto pr = resolve_cat(raw, 1.0, 24.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    ReliabilityRawFields raw;
    raw.mtbf_hours = pv.mtbf_hours;
    raw.mttr_hours = pv.mttr_hours;
    auto pr = resolve_cat(raw, 1.5, 24.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    ReliabilityRawFields raw;
    raw.mtbf_hours = sg.mtbf_hours;
    raw.mttr_hours = sg.mttr_hours;
    auto pr = resolve_cat(raw, 1.5, 24.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    // Prefer MTBF/MTTR; fall back to p_sw_fail (occ/yr) through the resolver.
    ReliabilityRawFields raw;
    raw.mtbf_hours = sw.mtbf_hours;
    raw.mttr_hours = sw.mttr_hours;
    if (sw.mtbf_hours <= 0 && sw.p_sw_fail > 0 && sw.p_sw_fail < 1.0) {
      raw.failure_rate_per_year = sw.p_sw_fail;
      raw.mttr_hr = sw.mttr_hours > 0 ? sw.mttr_hours : 4.0;
    }
    auto pr = resolve_cat(raw, 0.05, 4.0, c.name);
    c.lambda = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
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
    // CircuitBreaker has no mtbf/mttr; resolver applies utility CB defaults.
    auto pr = resolve_cat(ReliabilityRawFields{}, 0.05, 8.0, c.name);
    c.lambda     = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
    catalog.push_back(c);
  }

  // ---- AC PV Systems ----
  for (size_t i = 0; i < sys.ac.pv_systems.size(); ++i) {
    const auto& pv = sys.ac.pv_systems[i];
    if (!pv.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::ACPVSystem;
    c.idx  = static_cast<int>(i);
    c.name = pv.name.empty() ? "PV_" + std::to_string(pv.index) : pv.name;
    ReliabilityRawFields raw;
    raw.mtbf_hours = pv.mtbf_hours;
    raw.mttr_hours = pv.mttr_hours;
    auto pr = resolve_cat(raw, 1.5, 24.0, c.name);
    c.lambda     = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
    catalog.push_back(c);
  }

  // ---- DC Static Generators (AC StaticGenerator type in DC container) ----
  for (size_t i = 0; i < sys.dc.static_generators.size(); ++i) {
    const auto& sg = sys.dc.static_generators[i];
    if (!sg.in_service) continue;
    FMEAComponent c;
    c.type = FMEAComponent::DCStaticGenAC;
    c.idx  = static_cast<int>(i);
    c.name = sg.name.empty() ? "DCSGen2_" + std::to_string(sg.index) : sg.name;
    ReliabilityRawFields raw;
    raw.mtbf_hours = sg.mtbf_hours;
    raw.mttr_hours = sg.mttr_hours;
    auto pr = resolve_cat(raw, 1.5, 24.0, c.name);
    c.lambda     = pr.first;
    c.tau_sw_hr  = default_sw_hr;
    c.tau_rep_hr = pr.second;
    catalog.push_back(c);
  }

  return catalog;
}

struct FMEAStageEval {
  StateEvalResult eval;
  std::vector<FMEAContingencyDetail::SwitchActionDetail> switch_actions;
  bool switch_sequence_valid{true};
  std::string switch_sequence_message{"no switching required"};
  bool fault_isolation_explicit{false};
  std::string fault_isolation_message{
      "fault isolation represented by forced-open failed component"};
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
    case FMEAComponent::ACPVSystem:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.ac.pv_systems.size()))
        sys.ac.pv_systems[comp.idx].in_service = false;
      break;
    case FMEAComponent::DCStaticGenAC:
      if (comp.idx >= 0 && comp.idx < static_cast<int>(sys.dc.static_generators.size()))
        sys.dc.static_generators[comp.idx].in_service = false;
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
  for (auto& bus : sys.dc.buses) {
    bus.pd_mw *= load_scale;
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

void mark_island_anchor(HybridPowerSystem& sys, int bus, int index,
                        const std::string& name, double rating_mva = 0.0) {
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
  // F12: cap the island anchor's injection at the forming device's rating so it
  // is not an unbounded 2x-demand slack.  add_external_grid_dispatch_sources
  // uses s_sc_max_mva as the anchor generator's pmax when it is > 0.
  eg.s_sc_max_mva = std::max(0.0, rating_mva);
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

// Materialize the remaining aggregated / transfer reliability sources so that a
// failure mode which sets one out of service produces real load shedding:
//   * VirtualPowerPlant / MobileStorage / Microgrid -> dispatchable AC injection
//     at the PCC/connection bus (mirrors add_external_grid_dispatch_sources).
//   * EnergyRouter -> an AC<->DC transfer modelled as a VSC link between its
//     first in-service AC port and first in-service DC port (dominant routing
//     function; multi-port detail is a later refinement).
// Out-of-service components are skipped, so apply_consequence_patch setting one
// out removes its contribution and the load it supplied is shed.
void materialize_aggregated_reliability_sources(HybridPowerSystem& sys,
                                                double marginal_cost) {
  int next_gen = next_generator_index(sys);
  for (const auto& vpp : sys.vpps) {
    if (!vpp.in_service || vpp.pcc_bus <= 0) continue;
    double p = vpp.pmax_mw > 0.0 ? vpp.pmax_mw : vpp.p_output_mw;
    if (p <= 1e-9) p = vpp.p_generation_sum_mw;
    add_emergency_generator(sys, vpp.pcc_bus, std::max(0.0, p),
        "FMEA_vpp_" + std::to_string(vpp.index), next_gen, marginal_cost);
  }
  for (const auto& ms : sys.mobile_storage) {
    if (!ms.in_service || ms.bus <= 0 || !ms.controllable) continue;
    double p = ms.pmax_mw > 0.0 ? ms.pmax_mw : ms.p_rated_mw;
    if (p <= 1e-9) p = std::max(0.0, ms.p_mw);
    add_emergency_generator(sys, ms.bus, std::max(0.0, p),
        "FMEA_mobile_storage_" + std::to_string(ms.index), next_gen, marginal_cost);
  }
  for (const auto& mg : sys.microgrids) {
    if (!mg.in_service || mg.pcc_bus <= 0) continue;
    double p = mg.capacity_mw;
    if (p <= 1e-9) p = mg.total_generation_mw;
    if (p <= 1e-9) p = mg.total_dg_capacity_mw + mg.total_diesel_capacity_mw;
    if (p <= 1e-9) p = mg.p_exchange_max_mw;
    add_emergency_generator(sys, mg.pcc_bus, std::max(0.0, p),
        "FMEA_microgrid_" + std::to_string(mg.index), next_gen, marginal_cost);
  }
  int next_vsc = 1;
  for (const auto& v : sys.vsc_converters) next_vsc = std::max(next_vsc, v.index + 1);
  // An AC<->DC router transfer only matters when the system has a DC side.
  if (!sys.dc.buses.empty()) {
    for (const auto& er : sys.energy_routers) {
      if (!er.in_service) continue;
      int ac_bus = -1, dc_bus = -1;
      for (const auto& port : er.ports) {
        if (!port.in_service) continue;
        if (port.port_type == ERPortType::AC && ac_bus < 0) ac_bus = port.bus;
        if (port.port_type == ERPortType::DC && dc_bus < 0) dc_bus = port.bus;
      }
      if (ac_bus <= 0 || dc_bus <= 0) continue;
      double p = er.p_rated_mw > 0.0 ? er.p_rated_mw : er.pmax_mw;
      if (p <= 1e-9) continue;
      VSCConverter v;
      v.index = next_vsc++;
      v.bus_ac = ac_bus;
      v.bus_dc = dc_bus;
      v.in_service = true;
      v.controllable = true;
      v.pmax_mw = p;
      v.pmin_mw = -p;
      v.p_rated_mw = p;
      v.p_set_mw = 0.0;
      v.name = "FMEA_energy_router_" + std::to_string(er.index);
      sys.vsc_converters.push_back(std::move(v));
    }
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
            "FMEA_black_start_storage_grid_" + std::to_string(st.index), p);
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
      // F12: cap the island anchor at the converter's own MVA rating.
      const double vsc_cap =
          std::max({vsc.pmax_mw, std::abs(vsc.pmin_mw), vsc.p_rated_mw});
      mark_island_anchor(
          sys, vsc.bus_ac, next_eg++,
          "FMEA_grid_forming_vsc_grid_" + std::to_string(vsc.index), vsc_cap);
    }
  }

  if (options.enable_microgrid_islanding) {
    for (const auto& mg : sys.microgrids) {
      if (!mg.in_service || !mg.islanding_capability) continue;
      double p = mg.capacity_mw;
      if (p <= 1e-9) p = mg.total_generation_mw;
      if (p <= 1e-9) p = mg.total_dg_capacity_mw + mg.total_diesel_capacity_mw;
      if (p <= 1e-9) p = mg.p_exchange_max_mw;
      if (p <= 1e-9) continue;  // no rated island capacity -> no credited support
      // F12: one capacity-capped anchor provides both the island voltage
      // reference and the bounded injection (no separate unbounded emergency gen).
      mark_island_anchor(
          sys, mg.pcc_bus, next_eg++,
          "FMEA_microgrid_" + std::to_string(mg.index), p);
    }
  }
}

struct FMEANetworkSource {
  bool ac_side{true};
  int bus_pos{-1};
  double pmin_mw{0.0};
  double pmax_mw{0.0};
  double cost_mwh{1.0};
  std::string name;
};

struct FMEANetworkEdge {
  bool ac_side{true};
  int from_pos{-1};
  int to_pos{-1};
  double capacity_mw{0.0};
  std::string name;
  // F9: per-unit susceptance B = 1/x for AC branches (DC power-flow / Kirchhoff
  // voltage law).  0 = equipotential edge (zero-impedance switch/breaker/
  // transformer) whose endpoints are angle-tied but flow is free (a bus merge).
  double susceptance{0.0};
};

struct FMEANetworkTransfer {
  int ac_pos{-1};
  int dc_pos{-1};
  double pmin_mw{0.0};
  double pmax_mw{0.0};
  std::string name;
};

struct FMEANetworkDCDC {
  int from_pos{-1};
  int to_pos{-1};
  double pmin_mw{0.0};
  double pmax_mw{0.0};
  std::string name;
};

double positive_or_zero(double value) {
  return std::isfinite(value) ? std::max(0.0, value) : 0.0;
}

double default_network_capacity_mw(
    const std::vector<double>& ac_load_mw,
    const std::vector<double>& dc_load_mw) {
  const double ac_total = std::accumulate(ac_load_mw.begin(), ac_load_mw.end(), 0.0);
  const double dc_total = std::accumulate(dc_load_mw.begin(), dc_load_mw.end(), 0.0);
  return std::max(1000.0, 4.0 * (ac_total + dc_total + 1.0));
}

void add_fmea_source(std::vector<FMEANetworkSource>& sources,
                     bool ac_side,
                     int bus_pos,
                     double pmin_mw,
                     double pmax_mw,
                     double cost_mwh,
                     std::string name) {
  if (bus_pos < 0 || !std::isfinite(pmax_mw)) return;
  if (pmax_mw <= 1e-9) return;
  if (!std::isfinite(pmin_mw)) pmin_mw = 0.0;
  pmin_mw = std::clamp(pmin_mw, 0.0, pmax_mw);
  sources.push_back({ac_side, bus_pos, pmin_mw, pmax_mw,
                     std::max(0.0, cost_mwh), std::move(name)});
}

double vsc_capacity_mw(const VSCConverter& vsc) {
  double capacity = std::max({std::abs(vsc.pmax_mw),
                              std::abs(vsc.pmin_mw),
                              std::abs(vsc.p_set_mw),
                              vsc.p_rated_mw});
  return positive_or_zero(capacity);
}

std::pair<double, double> vsc_bounds_mw(const VSCConverter& vsc) {
  const double capacity = vsc_capacity_mw(vsc);
  if (capacity <= 1e-9) return {0.0, 0.0};
  if (!vsc.controllable) return {vsc.p_set_mw, vsc.p_set_mw};
  double pmin = vsc.pmin_mw;
  double pmax = vsc.pmax_mw;
  if (std::abs(pmin) <= 1e-12 && std::abs(pmax) <= 1e-12) {
    pmin = -capacity;
    pmax = capacity;
  } else {
    if (pmax <= 1e-12) pmax = capacity;
    if (pmin >= -1e-12 && vsc.p_set_mw < 0.0) pmin = -capacity;
  }
  if (pmin > pmax) std::swap(pmin, pmax);
  return {pmin, pmax};
}

std::pair<double, double> dcdc_bounds_mw(const DCDCConverter& converter) {
  double capacity = std::max({std::abs(converter.pmax_mw),
                              std::abs(converter.pmin_mw),
                              std::abs(converter.p_ref_mw),
                              converter.sn_mva});
  capacity = positive_or_zero(capacity);
  if (capacity <= 1e-9) return {0.0, 0.0};
  if (!converter.controllable) return {converter.p_ref_mw, converter.p_ref_mw};
  double pmin = converter.pmin_mw;
  double pmax = converter.pmax_mw;
  if (std::abs(pmin) <= 1e-12 && std::abs(pmax) <= 1e-12) {
    pmin = -capacity;
    pmax = capacity;
  } else {
    if (pmax <= 1e-12) pmax = capacity;
    if (pmin >= -1e-12 && converter.p_ref_mw < 0.0) pmin = -capacity;
  }
  if (pmin > pmax) std::swap(pmin, pmax);
  return {pmin, pmax};
}

StateEvalResult conservative_hybrid_shed(
    const HybridPowerSystem& sys,
    const std::vector<double>& ac_load_mw,
    const std::vector<double>& dc_load_mw,
    double curtail_threshold_mw) {
  StateEvalResult result;
  result.nodal_curtailment_mw.assign(
      sys.ac.buses.size() + sys.dc.buses.size(), 0.0);
  for (size_t bus_pos = 0; bus_pos < ac_load_mw.size(); ++bus_pos) {
    result.nodal_curtailment_mw[bus_pos] = positive_or_zero(ac_load_mw[bus_pos]);
    result.curtailment_mw += result.nodal_curtailment_mw[bus_pos];
  }
  for (size_t bus_pos = 0; bus_pos < dc_load_mw.size(); ++bus_pos) {
    const size_t out_pos = sys.ac.buses.size() + bus_pos;
    result.nodal_curtailment_mw[out_pos] = positive_or_zero(dc_load_mw[bus_pos]);
    result.curtailment_mw += result.nodal_curtailment_mw[out_pos];
  }
  result.is_loss_state = result.curtailment_mw > curtail_threshold_mw;
  return result;
}

StateEvalResult evaluate_hybrid_fmea_network_lp(
    const HybridPowerSystem& sys,
    const FMEAOptions& options,
    double stage_duration_hr,
    double curtail_threshold_mw = 0.01) {
  const size_t ac_bus_count = sys.ac.buses.size();
  const size_t dc_bus_count = sys.dc.buses.size();
  std::unordered_map<int, int> ac_bus_pos;
  std::unordered_map<int, int> dc_bus_pos;
  ac_bus_pos.reserve(ac_bus_count);
  dc_bus_pos.reserve(dc_bus_count);
  for (size_t bus_pos = 0; bus_pos < ac_bus_count; ++bus_pos) {
    if (sys.ac.buses[bus_pos].in_service) {
      ac_bus_pos[sys.ac.buses[bus_pos].index] = static_cast<int>(bus_pos);
    }
  }
  for (size_t bus_pos = 0; bus_pos < dc_bus_count; ++bus_pos) {
    if (sys.dc.buses[bus_pos].in_service) {
      dc_bus_pos[sys.dc.buses[bus_pos].index] = static_cast<int>(bus_pos);
    }
  }

  std::vector<double> ac_load_mw(ac_bus_count, 0.0);
  std::vector<double> dc_load_mw(dc_bus_count, 0.0);
  for (size_t bus_pos = 0; bus_pos < ac_bus_count; ++bus_pos) {
    if (!sys.ac.buses[bus_pos].in_service) continue;
    ac_load_mw[bus_pos] += positive_or_zero(sys.ac.buses[bus_pos].pd_mw);
  }
  for (const auto& load : sys.ac.loads) {
    if (!load.in_service) continue;
    auto bus_it = ac_bus_pos.find(load.bus);
    if (bus_it == ac_bus_pos.end()) continue;
    ac_load_mw[static_cast<size_t>(bus_it->second)] +=
        positive_or_zero(load.p_mw * load.scaling);
  }
  for (size_t bus_pos = 0; bus_pos < dc_bus_count; ++bus_pos) {
    if (!sys.dc.buses[bus_pos].in_service) continue;
    dc_load_mw[bus_pos] += positive_or_zero(sys.dc.buses[bus_pos].pd_mw);
  }
  for (const auto& load : sys.dc.loads) {
    if (!load.in_service) continue;
    auto bus_it = dc_bus_pos.find(load.bus);
    if (bus_it == dc_bus_pos.end()) continue;
    dc_load_mw[static_cast<size_t>(bus_it->second)] +=
        positive_or_zero(hacdcpf::model::effective_load_p_mw(load));
  }

  const double reserve_capacity_mw = default_network_capacity_mw(ac_load_mw, dc_load_mw);
  std::vector<FMEANetworkSource> sources;
  std::vector<FMEANetworkEdge> ac_edges;
  std::vector<FMEANetworkEdge> dc_edges;
  std::vector<FMEANetworkTransfer> vsc_transfers;
  std::vector<FMEANetworkDCDC> dcdc_transfers;

  for (const auto& generator : sys.ac.generators) {
    if (!generator.in_service) continue;
    auto bus_it = ac_bus_pos.find(generator.bus);
    if (bus_it == ac_bus_pos.end()) continue;
    double pmax = generator.pmax_mw > 0.0 ? generator.pmax_mw : positive_or_zero(generator.pg_mw);
    add_fmea_source(sources, true, bus_it->second,
                    positive_or_zero(generator.pmin_mw), pmax,
                    generator.cost_c1,
                    generator.name.empty() ? "gen_" + std::to_string(generator.index)
                                           : generator.name);
  }
  for (const auto& source : sys.ac.static_generators) {
    if (!source.in_service) continue;
    auto bus_it = ac_bus_pos.find(source.bus);
    if (bus_it == ac_bus_pos.end()) continue;
    double pmax = source.pmax_mw > 0.0 ? source.pmax_mw : source.p_rated_mw;
    if (pmax <= 1e-9) pmax = positive_or_zero(source.p_mw * source.scaling);
    add_fmea_source(sources, true, bus_it->second, 0.0, pmax, 1.0,
                    source.name.empty() ? "sgen_" + std::to_string(source.index)
                                        : source.name);
  }
  for (const auto& renewable : sys.ac.renewable_gens) {
    if (!renewable.in_service) continue;
    auto bus_it = ac_bus_pos.find(renewable.bus);
    if (bus_it == ac_bus_pos.end()) continue;
    double pmax = renewable.p_mw > 0.0 ? renewable.p_mw : renewable.p_rated_mw;
    add_fmea_source(sources, true, bus_it->second, 0.0, pmax, 1.0,
                    renewable.name.empty() ? "ren_" + std::to_string(renewable.index)
                                           : renewable.name);
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    auto bus_it = ac_bus_pos.find(pv.bus);
    if (bus_it == ac_bus_pos.end()) continue;
    double pmax = pv.pmax_mw > 0.0 ? pv.pmax_mw : pv.p_mw;
    add_fmea_source(sources, true, bus_it->second, 0.0, pmax, 1.0,
                    pv.name.empty() ? "pv_" + std::to_string(pv.index) : pv.name);
  }
  for (const auto& storage : sys.ac.storage) {
    if (!storage.in_service || !storage.controllable) continue;
    auto bus_it = ac_bus_pos.find(storage.bus);
    if (bus_it == ac_bus_pos.end()) continue;
    add_fmea_source(sources, true, bus_it->second, 0.0,
                    storage_available_mw(storage, stage_duration_hr), 1.0,
                    storage.name.empty() ? "storage_" + std::to_string(storage.index)
                                         : storage.name);
  }

  for (const auto& source : sys.dc.static_generators) {
    if (!source.in_service) continue;
    auto bus_it = dc_bus_pos.find(source.bus);
    if (bus_it == dc_bus_pos.end()) continue;
    double pmax = source.pmax_mw > 0.0 ? source.pmax_mw : source.p_rated_mw;
    if (pmax <= 1e-9) pmax = positive_or_zero(source.p_mw * source.scaling);
    add_fmea_source(sources, false, bus_it->second, 0.0, pmax, 1.0,
                    source.name.empty() ? "dc_sgen_ac_" + std::to_string(source.index)
                                        : source.name);
  }
  for (const auto& source : sys.dc.dc_static_generators) {
    if (!source.in_service) continue;
    auto bus_it = dc_bus_pos.find(source.bus);
    if (bus_it == dc_bus_pos.end()) continue;
    double pmax = source.pmax_mw > 0.0 ? source.pmax_mw : positive_or_zero(source.p_set_mw * source.scaling);
    add_fmea_source(sources, false, bus_it->second, 0.0, pmax, 1.0,
                    source.name.empty() ? "dc_sgen_" + std::to_string(source.index)
                                        : source.name);
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service) continue;
    auto bus_it = dc_bus_pos.find(pv.bus);
    if (bus_it == dc_bus_pos.end()) continue;
    add_fmea_source(sources, false, bus_it->second, 0.0,
                    positive_or_zero(pv.p_set_mw), 1.0,
                    pv.name.empty() ? "dc_pv_" + std::to_string(pv.index) : pv.name);
  }
  for (const auto& storage : sys.dc.storage) {
    if (!storage.in_service || !storage.controllable) continue;
    auto bus_it = dc_bus_pos.find(storage.bus);
    if (bus_it == dc_bus_pos.end()) continue;
    add_fmea_source(sources, false, bus_it->second, 0.0,
                    storage_available_mw(storage, stage_duration_hr), 1.0,
                    storage.name.empty() ? "dc_storage_" + std::to_string(storage.index)
                                         : storage.name);
  }

  auto add_edge = [&](std::vector<FMEANetworkEdge>& edges,
                      bool ac_side,
                      int from_pos,
                      int to_pos,
                      double capacity_mw,
                      std::string name,
                      double susceptance = 0.0) {
    if (from_pos < 0 || to_pos < 0 || from_pos == to_pos) return;
    if (!std::isfinite(capacity_mw) || capacity_mw <= 1e-9) {
      capacity_mw = reserve_capacity_mw;
    }
    edges.push_back({ac_side, from_pos, to_pos, capacity_mw, std::move(name),
                     susceptance});
  };

  for (const auto& branch : sys.ac.branches) {
    if (!branch.in_service) continue;
    auto from_it = ac_bus_pos.find(branch.from_bus);
    auto to_it = ac_bus_pos.find(branch.to_bus);
    if (from_it == ac_bus_pos.end() || to_it == ac_bus_pos.end()) continue;
    double capacity = branch.rate_a_mva > 0.0 ? branch.rate_a_mva : branch.sn_mva;
    // F9: DC power-flow susceptance B = 1/x (p.u.) so the branch flow obeys
    // Kirchhoff's voltage law (Pf = B*(theta_from - theta_to)).
    const double b_pu = 1.0 / std::max(std::abs(branch.x_pu), 1e-4);
    add_edge(ac_edges, true, from_it->second, to_it->second, capacity,
             branch.name.empty() ? "ac_branch_" + std::to_string(branch.index)
                                 : branch.name,
             b_pu);
  }
  for (const auto& transformer : sys.ac.transformers_2w) {
    if (!transformer.in_service) continue;
    auto from_it = ac_bus_pos.find(transformer.hv_bus);
    auto to_it = ac_bus_pos.find(transformer.lv_bus);
    if (from_it == ac_bus_pos.end() || to_it == ac_bus_pos.end()) continue;
    add_edge(ac_edges, true, from_it->second, to_it->second, transformer.sn_mva,
             transformer.name.empty() ? "trafo2w_" + std::to_string(transformer.index)
                                      : transformer.name);
  }
  for (const auto& transformer : sys.ac.transformers_3w) {
    if (!transformer.in_service) continue;
    auto hv_it = ac_bus_pos.find(transformer.hv_bus);
    auto mv_it = ac_bus_pos.find(transformer.mv_bus);
    auto lv_it = ac_bus_pos.find(transformer.lv_bus);
    if (hv_it == ac_bus_pos.end()) continue;
    const double capacity = std::max({transformer.sn_hv_mva,
                                      transformer.sn_mv_mva,
                                      transformer.sn_lv_mva});
    if (mv_it != ac_bus_pos.end()) {
      add_edge(ac_edges, true, hv_it->second, mv_it->second, capacity,
               transformer.name.empty() ? "trafo3w_hm_" + std::to_string(transformer.index)
                                        : transformer.name + "_hm");
    }
    if (lv_it != ac_bus_pos.end()) {
      add_edge(ac_edges, true, hv_it->second, lv_it->second, capacity,
               transformer.name.empty() ? "trafo3w_hl_" + std::to_string(transformer.index)
                                        : transformer.name + "_hl");
    }
  }
  for (const auto& sw : sys.ac.switches) {
    if (!sw.in_service || !sw.closed) continue;
    auto from_it = ac_bus_pos.find(sw.bus_from);
    auto to_it = ac_bus_pos.find(sw.bus_to);
    if (from_it == ac_bus_pos.end() || to_it == ac_bus_pos.end()) continue;
    add_edge(ac_edges, true, from_it->second, to_it->second, reserve_capacity_mw,
             sw.name.empty() ? "switch_" + std::to_string(sw.index) : sw.name);
  }
  for (const auto& breaker : sys.ac.circuit_breakers) {
    if (!breaker.in_service || !breaker.closed) continue;
    auto from_it = ac_bus_pos.find(breaker.bus_from);
    auto to_it = ac_bus_pos.find(breaker.bus_to);
    if (from_it == ac_bus_pos.end() || to_it == ac_bus_pos.end()) continue;
    add_edge(ac_edges, true, from_it->second, to_it->second, reserve_capacity_mw,
             breaker.name.empty() ? "ac_cb_" + std::to_string(breaker.index)
                                  : breaker.name);
  }

  for (const auto& branch : sys.dc.branches) {
    if (!branch.in_service) continue;
    auto from_it = dc_bus_pos.find(branch.from_bus);
    auto to_it = dc_bus_pos.find(branch.to_bus);
    if (from_it == dc_bus_pos.end() || to_it == dc_bus_pos.end()) continue;
    double capacity = branch.rate_a_mva > 0.0 ? branch.rate_a_mva : branch.s_max_mva;
    add_edge(dc_edges, false, from_it->second, to_it->second, capacity,
             branch.name.empty() ? "dc_branch_" + std::to_string(branch.index)
                                 : branch.name);
  }
  for (const auto& breaker : sys.dc.dc_circuit_breakers) {
    if (!breaker.in_service || !breaker.closed) continue;
    auto from_it = dc_bus_pos.find(breaker.bus_from);
    auto to_it = dc_bus_pos.find(breaker.bus_to);
    if (from_it == dc_bus_pos.end() || to_it == dc_bus_pos.end()) continue;
    add_edge(dc_edges, false, from_it->second, to_it->second, reserve_capacity_mw,
             breaker.name.empty() ? "dc_cb_" + std::to_string(breaker.index)
                                  : breaker.name);
  }

  for (const auto& converter : sys.vsc_converters) {
    if (!converter.in_service) continue;
    auto ac_it = ac_bus_pos.find(converter.bus_ac);
    auto dc_it = dc_bus_pos.find(converter.bus_dc);
    if (ac_it == ac_bus_pos.end() || dc_it == dc_bus_pos.end()) continue;
    auto [pmin, pmax] = vsc_bounds_mw(converter);
    if (std::abs(pmax - pmin) <= 1e-9 && std::abs(pmax) <= 1e-9) continue;
    vsc_transfers.push_back({ac_it->second, dc_it->second, pmin, pmax,
                             converter.name.empty() ? "vsc_" + std::to_string(converter.index)
                                                    : converter.name});
  }
  for (const auto& converter : sys.dc.dcdc_converters) {
    if (!converter.in_service) continue;
    auto from_it = dc_bus_pos.find(converter.bus_in);
    auto to_it = dc_bus_pos.find(converter.bus_out);
    if (from_it == dc_bus_pos.end() || to_it == dc_bus_pos.end()) continue;
    auto [pmin, pmax] = dcdc_bounds_mw(converter);
    if (std::abs(pmax - pmin) <= 1e-9 && std::abs(pmax) <= 1e-9) continue;
    dcdc_transfers.push_back({from_it->second, to_it->second, pmin, pmax,
                              converter.name.empty() ? "dcdc_" + std::to_string(converter.index)
                                                     : converter.name});
  }

  const int source_offset = 0;
  const int ac_edge_offset = source_offset + static_cast<int>(sources.size());
  const int dc_edge_offset = ac_edge_offset + static_cast<int>(ac_edges.size());
  const int vsc_offset = dc_edge_offset + static_cast<int>(dc_edges.size());
  const int dcdc_offset = vsc_offset + static_cast<int>(vsc_transfers.size());
  const int ac_shed_offset = dcdc_offset + static_cast<int>(dcdc_transfers.size());
  const int dc_shed_offset = ac_shed_offset + static_cast<int>(ac_bus_count);
  // F9: AC bus voltage-angle (theta) variables for the DC power-flow constraints.
  const int theta_offset = dc_shed_offset + static_cast<int>(dc_bus_count);
  const int variable_count = theta_offset + static_cast<int>(ac_bus_count);
  // Rows: [AC bus balance | DC bus balance | one flow-definition row per AC edge].
  const int flow_row_offset = static_cast<int>(ac_bus_count + dc_bus_count);
  const int row_count = flow_row_offset + static_cast<int>(ac_edges.size());
  if (variable_count <= 0 || row_count <= 0) {
    return conservative_hybrid_shed(sys, ac_load_mw, dc_load_mw, curtail_threshold_mw);
  }

  const double base_mva = std::max({sys.base_mva, sys.ac.base_mva, sys.dc.base_mva, 1.0});
  // Reliability evaluation: minimise load shedding, not dispatch cost.  Use a
  // VOLL that strongly dominates every source's marginal cost so the LP serves
  // all load it physically can before shedding (source cost stays only a
  // tie-breaker among minimum-shed dispatches).
  double max_src_cost = 0.0;
  for (const auto& s : sources) max_src_cost = std::max(max_src_cost, s.cost_mwh);
  const double voll = reliability_shedding_voll(sys, options.voll, max_src_cost);
  engine::LPModel lp;
  lp.sense = engine::Sense::Minimize;
  lp.vars.resize(variable_count);
  lp.c = Eigen::VectorXd::Zero(variable_count);
  std::vector<Eigen::Triplet<double>> equality_triplets;
  equality_triplets.reserve(static_cast<size_t>(variable_count) * 2U);
  lp.beq = Eigen::VectorXd::Zero(row_count);
  for (size_t bus_pos = 0; bus_pos < ac_bus_count; ++bus_pos) {
    lp.beq[static_cast<int>(bus_pos)] = ac_load_mw[bus_pos] / base_mva;
  }
  for (size_t bus_pos = 0; bus_pos < dc_bus_count; ++bus_pos) {
    lp.beq[static_cast<int>(ac_bus_count + bus_pos)] = dc_load_mw[bus_pos] / base_mva;
  }

  for (size_t source_pos = 0; source_pos < sources.size(); ++source_pos) {
    const auto& source = sources[source_pos];
    const int var = source_offset + static_cast<int>(source_pos);
    lp.vars[var] = {engine::VarType::Continuous,
                    source.pmin_mw / base_mva,
                    source.pmax_mw / base_mva,
                    source.name};
    lp.c[var] = std::max(1.0, source.cost_mwh) * base_mva;
    const int row = source.ac_side ? source.bus_pos
                                   : static_cast<int>(ac_bus_count) + source.bus_pos;
    equality_triplets.emplace_back(row, var, 1.0);
  }
  for (size_t edge_pos = 0; edge_pos < ac_edges.size(); ++edge_pos) {
    const auto& edge = ac_edges[edge_pos];
    const int var = ac_edge_offset + static_cast<int>(edge_pos);
    lp.vars[var] = {engine::VarType::Continuous,
                    -edge.capacity_mw / base_mva,
                    edge.capacity_mw / base_mva,
                    edge.name};
    equality_triplets.emplace_back(edge.from_pos, var, -1.0);
    equality_triplets.emplace_back(edge.to_pos, var, 1.0);
  }
  for (size_t edge_pos = 0; edge_pos < dc_edges.size(); ++edge_pos) {
    const auto& edge = dc_edges[edge_pos];
    const int var = dc_edge_offset + static_cast<int>(edge_pos);
    lp.vars[var] = {engine::VarType::Continuous,
                    -edge.capacity_mw / base_mva,
                    edge.capacity_mw / base_mva,
                    edge.name};
    const int from_row = static_cast<int>(ac_bus_count) + edge.from_pos;
    const int to_row = static_cast<int>(ac_bus_count) + edge.to_pos;
    equality_triplets.emplace_back(from_row, var, -1.0);
    equality_triplets.emplace_back(to_row, var, 1.0);
  }
  for (size_t transfer_pos = 0; transfer_pos < vsc_transfers.size(); ++transfer_pos) {
    const auto& transfer = vsc_transfers[transfer_pos];
    const int var = vsc_offset + static_cast<int>(transfer_pos);
    lp.vars[var] = {engine::VarType::Continuous,
                    transfer.pmin_mw / base_mva,
                    transfer.pmax_mw / base_mva,
                    transfer.name};
    equality_triplets.emplace_back(transfer.ac_pos, var, 1.0);
    equality_triplets.emplace_back(static_cast<int>(ac_bus_count) + transfer.dc_pos, var, -1.0);
  }
  for (size_t transfer_pos = 0; transfer_pos < dcdc_transfers.size(); ++transfer_pos) {
    const auto& transfer = dcdc_transfers[transfer_pos];
    const int var = dcdc_offset + static_cast<int>(transfer_pos);
    lp.vars[var] = {engine::VarType::Continuous,
                    transfer.pmin_mw / base_mva,
                    transfer.pmax_mw / base_mva,
                    transfer.name};
    equality_triplets.emplace_back(static_cast<int>(ac_bus_count) + transfer.from_pos, var, -1.0);
    equality_triplets.emplace_back(static_cast<int>(ac_bus_count) + transfer.to_pos, var, 1.0);
  }
  for (size_t bus_pos = 0; bus_pos < ac_bus_count; ++bus_pos) {
    const int var = ac_shed_offset + static_cast<int>(bus_pos);
    lp.vars[var] = {engine::VarType::Continuous,
                    0.0,
                    positive_or_zero(ac_load_mw[bus_pos]) / base_mva,
                    "shed_ac_" + std::to_string(sys.ac.buses[bus_pos].index)};
    lp.c[var] = voll * base_mva;
    equality_triplets.emplace_back(static_cast<int>(bus_pos), var, 1.0);
  }
  for (size_t bus_pos = 0; bus_pos < dc_bus_count; ++bus_pos) {
    const int var = dc_shed_offset + static_cast<int>(bus_pos);
    lp.vars[var] = {engine::VarType::Continuous,
                    0.0,
                    positive_or_zero(dc_load_mw[bus_pos]) / base_mva,
                    "shed_dc_" + std::to_string(sys.dc.buses[bus_pos].index)};
    lp.c[var] = voll * base_mva;
    equality_triplets.emplace_back(static_cast<int>(ac_bus_count + bus_pos), var, 1.0);
  }

  // F9: AC bus angle variables + DC power-flow (Kirchhoff) flow-definition rows.
  // theta[0] is the angle reference (fixed at 0); the others are free.  Each AC
  // edge adds one row: branches enforce Pf = B*(theta_from - theta_to) (DC power
  // flow), while zero-impedance edges (switch/breaker/transformer, B=0) enforce
  // theta_from = theta_to (equipotential bus merge) with their flow left free.
  // For radial networks this reproduces the transport solution; for meshed
  // networks it constrains loop flows that the pure transport LP left free.
  for (size_t bus_pos = 0; bus_pos < ac_bus_count; ++bus_pos) {
    const int var = theta_offset + static_cast<int>(bus_pos);
    const double bound = (bus_pos == 0) ? 0.0 : 1.0e3;
    lp.vars[var] = {engine::VarType::Continuous, -bound, bound,
                    "theta_ac_" + std::to_string(sys.ac.buses[bus_pos].index)};
  }
  for (size_t edge_pos = 0; edge_pos < ac_edges.size(); ++edge_pos) {
    const auto& edge = ac_edges[edge_pos];
    const int row = flow_row_offset + static_cast<int>(edge_pos);
    const int theta_from = theta_offset + edge.from_pos;
    const int theta_to = theta_offset + edge.to_pos;
    if (edge.susceptance > 0.0) {
      // Pf - B*(theta_from - theta_to) = 0
      const int flow_var = ac_edge_offset + static_cast<int>(edge_pos);
      equality_triplets.emplace_back(row, flow_var, 1.0);
      equality_triplets.emplace_back(row, theta_from, -edge.susceptance);
      equality_triplets.emplace_back(row, theta_to, edge.susceptance);
    } else {
      // Equipotential (zero-impedance): theta_from - theta_to = 0
      equality_triplets.emplace_back(row, theta_from, 1.0);
      equality_triplets.emplace_back(row, theta_to, -1.0);
    }
  }

  lp.Aeq.resize(row_count, variable_count);
  lp.Aeq.setFromTriplets(equality_triplets.begin(), equality_triplets.end());
  lp.Aeq.makeCompressed();
  lp.A.resize(0, variable_count);
  lp.b.resize(0);

  engine::SimplexOptions simplex_options;
  simplex_options.max_iter = std::max(1000, options.opf_options.max_iterations);
  simplex_options.feasibility_tol = std::max(1e-9, options.opf_options.feasibility_tol);
  simplex_options.optimality_tol = std::max(1e-9, options.opf_options.optimality_tol);
  auto solve_certificate = engine::solve_lp_with_basis(lp, simplex_options, nullptr);
  const auto& solve_result = solve_certificate.result;
  if (!solve_result.stats.success || solve_result.x.size() < variable_count) {
    spdlog::warn("FMEA hybrid AC/DC LP failed (status='{}'); applying conservative full-load shed",
                 solve_result.stats.status);
    return conservative_hybrid_shed(sys, ac_load_mw, dc_load_mw, curtail_threshold_mw);
  }

  StateEvalResult result;
  result.nodal_curtailment_mw.assign(ac_bus_count + dc_bus_count, 0.0);
  for (size_t bus_pos = 0; bus_pos < ac_bus_count; ++bus_pos) {
    const double shed = std::max(0.0, solve_result.x[ac_shed_offset + static_cast<int>(bus_pos)] * base_mva);
    result.nodal_curtailment_mw[bus_pos] = shed;
    result.curtailment_mw += shed;
  }
  for (size_t bus_pos = 0; bus_pos < dc_bus_count; ++bus_pos) {
    const double shed = std::max(0.0, solve_result.x[dc_shed_offset + static_cast<int>(bus_pos)] * base_mva);
    result.nodal_curtailment_mw[ac_bus_count + bus_pos] = shed;
    result.curtailment_mw += shed;
  }
  result.is_loss_state = result.curtailment_mw > curtail_threshold_mw;
  return result;
}

std::vector<int> repair_switch_candidates(const HybridPowerSystem& sys) {
  std::vector<int> candidates;
  for (int i = 0; i < static_cast<int>(sys.ac.switches.size()); ++i) {
    const auto& sw = sys.ac.switches[i];
    if (!sw.in_service || sw.closed) continue;
    if (sw.locked_open) continue;
    const auto caps = effective_switch_capabilities(sw);
    if (!caps.can_close_for_restoration) continue;
    if (sw.switch_type == SwitchType::Fuse) continue;
    // Explicit equipment data must identify a restoration tie.  Unspecified
    // legacy cases retain their previous type-derived behaviour.
    if (sw.capabilities_explicit && sw.role != SwitchRole::Tie) continue;
    if (!sw.capabilities_explicit && !sw.is_automated &&
        sw.t_operation_s <= 0.0) continue;
    candidates.push_back(i);
  }
  return candidates;
}

struct IsolationPlan {
  std::vector<int> switch_positions;
  std::vector<FMEAContingencyDetail::SwitchActionDetail> actions;
  bool explicit_plan{false};
  bool valid{true};
  std::string message{"fault isolation represented by forced-open failed component"};
};

IsolationPlan build_fmea_isolation_plan(const HybridPowerSystem& sys,
                                        const FMEAComponent& comp) {
  IsolationPlan plan;
  std::string controlled_type;
  int controlled_index = -1;
  if (comp.type == FMEAComponent::ACBranch && comp.idx >= 0 &&
      comp.idx < static_cast<int>(sys.ac.branches.size())) {
    controlled_type = "ac_branch";
    controlled_index = sys.ac.branches[comp.idx].index;
  } else if (comp.type == FMEAComponent::Transformer2W && comp.idx >= 0 &&
             comp.idx < static_cast<int>(sys.ac.transformers_2w.size())) {
    controlled_type = "transformer_2w";
    controlled_index = sys.ac.transformers_2w[comp.idx].index;
  } else {
    return plan;
  }
  std::vector<int> protection;
  std::vector<int> boundaries;
  std::unordered_map<int, int> switch_position_by_index;
  std::unordered_map<int, int> boundary_upstream_position;
  for (int i = 0; i < static_cast<int>(sys.ac.switches.size()); ++i)
    switch_position_by_index[sys.ac.switches[i].index] = i;
  const auto add_protection = [&](int pos) {
    if (std::find(protection.begin(), protection.end(), pos) == protection.end())
      protection.push_back(pos);
  };
  for (int i = 0; i < static_cast<int>(sys.ac.switches.size()); ++i) {
    const auto& sw = sys.ac.switches[i];
    const bool generic_match = sw.controlled_element_type == controlled_type &&
                               sw.controlled_element_index == controlled_index;
    const bool legacy_branch_match = controlled_type == "ac_branch" &&
                                     sw.controlled_branch_index == controlled_index;
    if (!sw.in_service || !sw.closed ||
        (!generic_match && !legacy_branch_match))
      continue;
    if (sw.role == SwitchRole::Protection) add_protection(i);
    else if (sw.role == SwitchRole::Sectionalizing ||
             sw.role == SwitchRole::Isolation)
      boundaries.push_back(i);
  }
  for (int pos : boundaries) {
    const auto& boundary = sys.ac.switches[pos];
    if (!effective_switch_capabilities(boundary).requires_deenergized_operation)
      continue;
    int upstream_index = boundary.upstream_protective_switch_index;
    if (upstream_index < 0)
      upstream_index = boundary.sectionalizer_protection.upstream_switch_index;
    if (upstream_index < 0) continue;
    const auto upstream = switch_position_by_index.find(upstream_index);
    boundary_upstream_position[pos] =
        upstream == switch_position_by_index.end() ? -1 : upstream->second;
    if (upstream != switch_position_by_index.end()) {
      const auto& protector = sys.ac.switches[upstream->second];
      if (protector.in_service && protector.closed)
        add_protection(upstream->second);
    }
  }
  std::unordered_set<int> cleared_by_protection;
  const auto append = [&](int pos, bool protection_action,
                          bool upstream_cleared) {
    const auto& sw = sys.ac.switches[pos];
    const auto caps = effective_switch_capabilities(sw);
    FMEAContingencyDetail::SwitchActionDetail action;
    action.switch_index = sw.index;
    action.switch_name = sw.name;
    action.switch_type = switch_type_name(sw.switch_type);
    action.action = protection_action ? "trip" : "open";
    action.bus_from = sw.bus_from;
    action.bus_to = sw.bus_to;
    action.sequence_order = static_cast<int>(plan.actions.size()) + 1;
    action.purpose = protection_action ? "fault_clearance" : "fault_section_isolation";
    action.operation_time_s = sw.t_open_s > 0.0 ? sw.t_open_s : sw.t_operation_s;
    action.validated = !sw.locked_closed &&
        (protection_action ? (sw.role == SwitchRole::Protection &&
                              caps.can_interrupt_fault_current)
                           : (caps.can_interrupt_load_current ||
                              (caps.requires_deenergized_operation &&
                               upstream_cleared)));
    action.validation_message = action.validated
        ? (protection_action
               ? "protection device is rated to interrupt fault current"
               : "boundary device opens after upstream fault clearance")
        : (caps.requires_deenergized_operation && !upstream_cleared
               ? "de-energized operation requires a validated upstream protective trip"
               : "device capability or lock state does not permit this isolation action");
    plan.valid = plan.valid && action.validated;
    if (action.validated) {
      plan.switch_positions.push_back(pos);
      if (protection_action) cleared_by_protection.insert(pos);
    }
    plan.actions.push_back(std::move(action));
  };
  for (int pos : protection) append(pos, true, false);
  for (int pos : boundaries) {
    const auto dependency = boundary_upstream_position.find(pos);
    const bool upstream_cleared = dependency == boundary_upstream_position.end()
        ? !cleared_by_protection.empty()
        : dependency->second >= 0 &&
              cleared_by_protection.count(dependency->second) > 0;
    append(pos, false, upstream_cleared);
  }
  plan.explicit_plan = !plan.actions.empty();
  if (plan.explicit_plan) {
    plan.message = plan.valid
        ? "protection and isolation actions validated from controlled-equipment bindings"
        : "controlled-equipment isolation plan contains an invalid switch action";
  }
  return plan;
}

void apply_fmea_isolation_plan(HybridPowerSystem& sys,
                               const IsolationPlan& plan) {
  for (int pos : plan.switch_positions) {
    if (pos >= 0 && pos < static_cast<int>(sys.ac.switches.size()))
      sys.ac.switches[pos].closed = false;
  }
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
    a.switch_index = sw.index;
    a.switch_name = sw.name.empty() ? "switch_" + std::to_string(sw.index) : sw.name;
    a.switch_type = switch_type_name(sw.switch_type);
    a.action = "close";
    a.bus_from = sw.bus_from;
    a.bus_to = sw.bus_to;
    a.sequence_order = static_cast<int>(out.size()) + 1;
    a.purpose = "service_restoration_tie_close";
    a.operation_time_s = sw.t_close_s > 0.0 ? sw.t_close_s : sw.t_operation_s;
    const auto caps = effective_switch_capabilities(sw);
    a.validated = !sw.locked_open && caps.can_close_for_restoration &&
                  sw.switch_type != SwitchType::Fuse &&
                  (!sw.capabilities_explicit || sw.role == SwitchRole::Tie);
    a.validation_message = a.validated
        ? "eligible restoration tie; final topology accepted by contingency OPF"
        : "switch is not eligible for restoration closing";
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
    a.sequence_order = static_cast<int>(out.size()) + 1;
    a.purpose = "legacy_normally_open_branch_close";
    a.validated = false;
    a.validation_message =
        "legacy out-of-service branch has no controlling switch capability metadata";
    out.push_back(std::move(a));
  }
  return out;
}

StateEvalResult evaluate_prepared_fmea_system(
    const HybridPowerSystem& sys,
    const FMEAOptions& options,
    const opf::DCOPFOptions& opf_opt,
    double stage_duration_hr) {
  if (has_hybrid_fmea_components(sys)) {
    return evaluate_hybrid_fmea_network_lp(sys, options, stage_duration_hr);
  }
  std::vector<bool> no_failures(ComponentOffsets(sys).total, false);
  return evaluate_state(sys, no_failures, opf_opt, 1.0);
}

void apply_fmea_cyber_control_freeze(HybridPowerSystem& sys) {
  for (auto& source : sys.ac.static_generators) {
    if (!source.in_service || !source.controllable) continue;
    const double scheduled = std::max(0.0, source.p_mw * source.scaling);
    source.controllable = false;
    source.pmax_mw = scheduled;
    source.p_rated_mw = scheduled;
  }
  for (auto& source : sys.ac.renewable_gens) {
    if (!source.in_service || !source.curtailable) continue;
    source.curtailable = false;
    source.p_rated_mw = std::max(0.0, source.p_mw);
  }
  for (auto& source : sys.ac.pv_systems) {
    if (!source.in_service || !source.controllable) continue;
    source.controllable = false;
    source.pmax_mw = std::max(0.0, source.p_mw);
  }
  for (auto& storage : sys.ac.storage) storage.controllable = false;
  for (auto& storage : sys.dc.storage) storage.controllable = false;
  for (auto& storage : sys.dc.dc_storage) storage.controllable = false;
  for (auto& source : sys.dc.static_generators) {
    if (!source.in_service || !source.controllable) continue;
    const double scheduled = std::max(0.0, source.p_mw * source.scaling);
    source.controllable = false;
    source.pmax_mw = scheduled;
    source.p_rated_mw = scheduled;
  }
  for (auto& source : sys.dc.dc_static_generators) {
    if (!source.in_service || !source.controllable) continue;
    source.controllable = false;
    source.pmax_mw = std::max(0.0, source.p_set_mw * source.scaling);
  }
  for (auto& converter : sys.vsc_converters) converter.controllable = false;
  for (auto& converter : sys.dc.dcdc_converters) converter.controllable = false;
}

FMEAStageEval evaluate_contingency_stage(
    const HybridPowerSystem& sys,
    const FMEAComponent& comp,
    const FMEAOptions& options,
    const opf::DCOPFOptions& opf_opt,
    double stage_duration_hr,
    bool repair_stage,
    bool cyber_control_frozen = false) {
  
  HybridPowerSystem sys_copy = sys;
  scale_fmea_loads(sys_copy, options.load_scale_factor);
  apply_fmea_component_outage(sys_copy, comp);
  const IsolationPlan isolation = build_fmea_isolation_plan(sys, comp);
  apply_fmea_isolation_plan(sys_copy, isolation);
  if (cyber_control_frozen) apply_fmea_cyber_control_freeze(sys_copy);
  add_external_grid_dispatch_sources(sys_copy, 1.0);
  apply_fmea_support_sources(sys_copy, options, stage_duration_hr, repair_stage);

  FMEAStageEval best;
  if (repair_stage) {
    best.switch_actions = isolation.actions;
    best.switch_sequence_valid = isolation.valid;
    best.switch_sequence_message = isolation.message;
    best.fault_isolation_explicit = isolation.explicit_plan;
    best.fault_isolation_message = isolation.message;
  }
  best.eval = evaluate_prepared_fmea_system(
      sys_copy, options, opf_opt, stage_duration_hr);

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
    StateEvalResult ev = evaluate_prepared_fmea_system(
      cand, options, opf_opt, stage_duration_hr);
    ++opf_calls;
    if (ev.curtailment_mw + 1e-9 < best.eval.curtailment_mw) {
      best.eval = std::move(ev);
      best.switch_actions = isolation.actions;
      auto close_actions = describe_repair_actions(sys_copy, switches, branches);
      for (auto& action : close_actions) {
        action.sequence_order = static_cast<int>(best.switch_actions.size()) + 1;
        best.switch_actions.push_back(std::move(action));
      }
      best.switch_sequence_valid = std::all_of(
          best.switch_actions.begin(), best.switch_actions.end(),
          [](const auto& action) { return action.validated; });
      best.switch_sequence_message = best.switch_sequence_valid
          ? "ordered isolation and restoration sequence validated; final topology accepted by contingency OPF"
          : "sequence contains an invalid or metadata-free switching action";
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

NetworkShedResult evaluate_failed_network_state(
    const HybridPowerSystem& sys,
    const FMEAOptions& options) {
  NetworkShedResult r;
  // Materialize supply / aggregated / transfer sources (external grid, VPP,
  // mobile storage, microgrid, energy router) as dispatchable injections so a
  // failure mode that sets one out of service produces real load shedding.
  // Only this failure-mode-FMEA evaluator is affected; run_distribution_fmea
  // prepares its own sources upstream and does not call this function.
  HybridPowerSystem prepared = sys;
  add_external_grid_dispatch_sources(prepared, /*marginal_cost=*/1.0);
  materialize_aggregated_reliability_sources(prepared, /*marginal_cost=*/1.0);
  if (has_hybrid_fmea_components(prepared)) {
    // Hybrid AC/DC: reuse the FMEA network LP (load scale applied internally).
    StateEvalResult e = evaluate_hybrid_fmea_network_lp(
        prepared, options, /*stage_duration_hr=*/1.0, /*curtail_threshold_mw=*/0.01);
    r.total_shed_mw = e.curtailment_mw;
    r.nodal_shed_mw = std::move(e.nodal_curtailment_mw);
    r.is_loss = e.is_loss_state;
    r.model_scope = "hybrid-acdc-network-lp";
  } else {
    // AC-only: reuse the DC-OPF state evaluator with no extra sampled failures.
    // Minimise load shedding (not dispatch cost) for the failed state.
    opf::DCOPFOptions opf_opt = options.opf_options;
    opf_opt.load_shedding = true;
    opf_opt.voll = reliability_shedding_voll(prepared, std::max(opf_opt.voll, options.voll));
    opf_opt.verbose = false;
    opf_opt.compute_lmp = false;
    ComponentOffsets co(prepared);
    StateEvalResult e = evaluate_state(
        prepared, std::vector<bool>(co.total, false), opf_opt,
        options.load_scale_factor, 0.01);
    r.total_shed_mw = e.curtailment_mw;
    r.nodal_shed_mw = std::move(e.nodal_curtailment_mw);
    r.is_loss = e.is_loss_state;
    r.model_scope = "ac-only-dcopf";
  }
  return r;
}

FMEAResult run_distribution_fmea(
    const HybridPowerSystem& sys,
    const FMEAOptions& options) {
  
  spdlog::info("FMEA: Starting N-1 failure-mode enumeration");
  
  FMEAResult result;
  const bool hybrid_fmea = has_hybrid_fmea_components(sys);
  if (hybrid_fmea) {
    result.model_scope = "hybrid-acdc-network-lp";
    result.validity.dc_load_curtailment_included = true;
    result.validity.vsc_dc_power_flow_modelled = true;
    result.validity.ac_opf_curtailment = true;
    result.validity.repair_ac_switch_reconfiguration_modelled =
      options.enable_repair_reconfiguration && options.enable_switch_reconfiguration;
    result.validity.repair_dc_side_reconfiguration_modelled = false;
    result.model_limitations =
        "FMEA uses a linear hybrid AC/DC restoration LP: AC and DC branch transfer "
        "limits, DC load shedding, DC sources, DC/DC converters, and VSC active-power "
        "transfers are optimized for each stage. Nonlinear AC voltage/reactive limits "
      "are outside this reliability evaluator. Repair-stage explicit switch search "
      "enumerates AC switches/branches only; DC breakers, DC branches, DCDC devices, "
      "and VSC topology actions are not repair reconfiguration candidates. Ordered "
      "restoration closures are capability-checked and reported, while fault isolation "
      "is represented by forcing the failed component out because protection-zone-to-"
      "switch bindings are not available in the base network schema.";
  } else {
    result.model_scope = "ac-only-dcopf";
    result.validity = FMEAResult::ValidityFlags{};
    result.validity.ac_opf_curtailment = true;
    result.validity.repair_ac_switch_reconfiguration_modelled =
      options.enable_repair_reconfiguration && options.enable_switch_reconfiguration;
    result.model_limitations =
        "FMEA physical evaluation uses AC-only DC-OPF for AC distribution contingencies. "
        "Ordered restoration closures are capability-checked and reported; fault "
        "isolation is represented by the failed component outage until explicit "
        "protection-zone-to-switch bindings are supplied.";
  }
  if (options.cyber_physical.enabled) {
    auto& cyber = result.cyber_physical;
    cyber.enabled = true;
    cyber.level = 1;
    cyber.model_scope = "level1-scalar-interface-matrix";
    cyber.automation_availability =
        std::clamp(options.cyber_physical.automation_availability, 0.0, 1.0);
    cyber.automatic_switching_time_hr =
        std::max(0.0, options.cyber_physical.automatic_switching_time_hr);
    cyber.manual_switching_time_hr =
        std::max(0.0, options.cyber_physical.manual_switching_time_hr);
    result.validity.restoration_duration_cyber_conditioned = true;
    result.validity.cyber_control_consequence_modelled =
        options.cyber_physical.freeze_der_on_automation_loss;
    result.model_limitations +=
        " Cyber-physical conditioning is Level 1: scalar automation availability "
        "with class-conditioned restoration duration and optional DER/control "
        "freezing. The automation-unavailable class still credits crew-based "
        "switch reconfiguration during the repair stage (manual restoration), "
        "but DER/storage dispatch, grid-forming support, and microgrid islanding "
        "are frozen. The class switching times replace the global "
        "switching_time_hr for every contingency. Communication topology, shared "
        "cyber cut sets, cyber-node power supply, weather common cause, and "
        "adversarial attacks are not modelled; "
        "delta_protection_misoperation_mwh_yr is a reserved placeholder (always "
        "zero at Level 1 — protection misoperation lives in the failure-mode "
        "FMEA).";
  }
  const size_t nb = sys.ac.buses.size() + (hybrid_fmea ? sys.dc.buses.size() : 0U);
  result.nodal_eens_mwh_yr.resize(nb, 0.0);

  // Build component catalog (resolves reliability params + data quality).
  auto catalog = build_fmea_catalog(sys, options.switching_time_hr,
                                    options.data_policy, result.data_quality);
  result.n_contingencies = static_cast<int>(catalog.size());
  
  spdlog::info("FMEA: {} components in catalog (generators={}, ac_branches={}, dc_branches={}, vsc={})",
               catalog.size(),
               sys.ac.generators.size(), sys.ac.branches.size(),
               sys.dc.branches.size(), sys.vsc_converters.size());

  // DC-OPF options.  FMEA contingency evaluation minimises LOAD SHEDDING, so
  // VOLL is set to strongly dominate generation cost (lexicographic min-shed).
  opf::DCOPFOptions opf_opt = options.opf_options;
  opf_opt.load_shedding = true;
  opf_opt.voll = reliability_shedding_voll(sys, std::max(opf_opt.voll, options.voll));
  opf_opt.verbose = false;
  opf_opt.compute_lmp = false;  // batch path — LMPs not needed, skip supporting LP

  // Accumulators for SAIFI/SAIDI
  std::vector<double> nodal_cif(nb, 0.0);  // per-bus interruption frequency
  std::vector<double> nodal_cid(nb, 0.0);  // per-bus interruption duration
  std::vector<double> nodal_cid_perfect_cyber(nb, 0.0);
  std::vector<double> nodal_cid_no_automation(nb, 0.0);

  struct FMEAContingencyWorkResult {
    FMEAContingencyDetail detail;
    std::vector<double> nodal_eens;
    std::vector<double> nodal_cif;
    std::vector<double> nodal_cid;
    std::vector<double> nodal_cid_perfect_cyber;
    std::vector<double> nodal_cid_no_automation;
    bool causes_loss{false};
    double loss_frequency{0.0};
    double eens_perfect_cyber{0.0};
    double eens_no_automation{0.0};
    double eens_duration_increment{0.0};
    double eens_control_increment{0.0};
  };

  auto evaluate_fmea_contingency = [&](const FMEAComponent& comp) {
    FMEAContingencyWorkResult work;
    FMEAContingencyDetail detail;
    detail.component_index = comp.idx;
    detail.component_name = comp.name;
    detail.failure_rate = comp.lambda;
    detail.component_type = fmea_component_type_name(comp.type);

    double automation_availability = 1.0;
    if (options.cyber_physical.enabled) {
      automation_availability = options.cyber_physical.automation_availability;
      for (const auto& override : options.cyber_physical.availability_overrides) {
        if (override.component_type == detail.component_type &&
            override.component_index == comp.idx) {
          automation_availability = override.availability;
          break;
        }
      }
    }
    automation_availability = std::clamp(automation_availability, 0.0, 1.0);
    const double down_probability = 1.0 - automation_availability;
    const double automatic_tau_sw = std::clamp(
        options.cyber_physical.enabled
            ? options.cyber_physical.automatic_switching_time_hr
            : comp.tau_sw_hr,
        0.0, std::max(0.0, comp.tau_rep_hr));
    // Manual restoration cannot beat automatic restoration: a manual switching
    // time below the automatic one would produce a negative duration increment.
    const double manual_tau_sw = std::clamp(
        options.cyber_physical.enabled
            ? std::max(options.cyber_physical.manual_switching_time_hr,
                       options.cyber_physical.automatic_switching_time_hr)
            : comp.tau_sw_hr,
        0.0, std::max(0.0, comp.tau_rep_hr));

    FMEAOptions automatic_options = options;
    automatic_options.cyber_physical.enabled = false;
    FMEAOptions manual_options = automatic_options;
    if (options.cyber_physical.enabled &&
        options.cyber_physical.freeze_der_on_automation_loss) {
      // Class c2 = manual restoration: the crew still performs switch/tie
      // reconfiguration during the repair stage (enable_switch_reconfiguration
      // is deliberately kept), but everything that requires a live control
      // channel — DER re-dispatch, storage, grid-forming support, black start,
      // microgrid islanding — is unavailable.
      manual_options.enable_repair_reconfiguration = false;
      manual_options.enable_storage_dispatch = false;
      manual_options.enable_grid_forming_vsc_support = false;
      manual_options.enable_black_start_storage = false;
      manual_options.enable_microgrid_islanding = false;
    }

    struct ClassEvaluation {
      double tau_sw_hr{0.0};
      double tau_rep_hr{0.0};
      FMEAStageEval switching;
      FMEAStageEval repair;
      double ens_mwh{0.0};
      double loss_duration_hr{0.0};
      bool causes_loss{false};
    };
    auto evaluate_class = [&](const FMEAOptions& class_options,
                              double tau_sw_hr,
                              bool cyber_control_frozen,
                              const FMEAStageEval* reuse_repair = nullptr) {
      ClassEvaluation class_eval;
      class_eval.tau_sw_hr = tau_sw_hr;
      // F11: the event lasts MTTR in total. Switching and repair are disjoint
      // windows, so a slower cyber class shortens the remaining repair window.
      class_eval.tau_rep_hr =
          std::max(0.0, comp.tau_rep_hr - class_eval.tau_sw_hr);
      class_eval.switching = evaluate_contingency_stage(
          sys, comp, class_options, opf_opt, class_eval.tau_sw_hr, false,
          cyber_control_frozen);
      // The physical repair horizon is retained for storage-energy feasibility;
      // only the frequency-weighted stage duration uses tau_rep_hr.  The repair
      // evaluation depends on (options, horizon, freeze) but NOT on tau_sw, so
      // a caller that already solved the identical repair problem passes it in.
      class_eval.repair = reuse_repair
          ? *reuse_repair
          : evaluate_contingency_stage(
                sys, comp, class_options, opf_opt, comp.tau_rep_hr, true,
                cyber_control_frozen);
      class_eval.ens_mwh =
          class_eval.switching.eval.curtailment_mw * class_eval.tau_sw_hr +
          class_eval.repair.eval.curtailment_mw * class_eval.tau_rep_hr;
      if (class_eval.switching.eval.is_loss_state)
        class_eval.loss_duration_hr += class_eval.tau_sw_hr;
      if (class_eval.repair.eval.is_loss_state)
        class_eval.loss_duration_hr += class_eval.tau_rep_hr;
      class_eval.causes_loss = class_eval.switching.eval.is_loss_state ||
                               class_eval.repair.eval.is_loss_state;
      return class_eval;
    };

    const ClassEvaluation automatic =
        evaluate_class(automatic_options, automatic_tau_sw, false);
    // duration_only differs from automatic only in the switching-stage
    // duration; its repair problem is identical, so reuse the solved stage.
    const ClassEvaluation duration_only = options.cyber_physical.enabled
        ? evaluate_class(automatic_options, manual_tau_sw, false,
                         &automatic.repair)
        : automatic;
    const ClassEvaluation manual = options.cyber_physical.enabled &&
                                           options.cyber_physical.freeze_der_on_automation_loss
        ? evaluate_class(manual_options, manual_tau_sw,
                         options.cyber_physical.freeze_der_on_automation_loss)
        : duration_only;

    detail.automation_availability = automation_availability;
    detail.tau_sw_automatic_hr = automatic.tau_sw_hr;
    detail.tau_sw_manual_hr = manual.tau_sw_hr;
    detail.tau_sw_hr = automation_availability * automatic.tau_sw_hr +
                       down_probability * manual.tau_sw_hr;
    detail.tau_rep_hr = automation_availability * automatic.tau_rep_hr +
                        down_probability * manual.tau_rep_hr;
    detail.shed_sw_automatic_mw = automatic.switching.eval.curtailment_mw;
    detail.shed_sw_manual_mw = manual.switching.eval.curtailment_mw;
    detail.shed_rep_automatic_mw = automatic.repair.eval.curtailment_mw;
    detail.shed_rep_manual_mw = manual.repair.eval.curtailment_mw;
    detail.shed_sw_mw =
        automation_availability * detail.shed_sw_automatic_mw +
        down_probability * detail.shed_sw_manual_mw;
    detail.shed_rep_mw =
        automation_availability * detail.shed_rep_automatic_mw +
        down_probability * detail.shed_rep_manual_mw;
    detail.ens_sw_mwh =
        automation_availability * automatic.switching.eval.curtailment_mw *
            automatic.tau_sw_hr +
        down_probability * manual.switching.eval.curtailment_mw * manual.tau_sw_hr;
    detail.ens_rep_mwh =
        automation_availability * automatic.repair.eval.curtailment_mw *
            automatic.tau_rep_hr +
        down_probability * manual.repair.eval.curtailment_mw * manual.tau_rep_hr;
    detail.causes_loss_sw =
        (automation_availability > 0.0 && automatic.switching.eval.is_loss_state) ||
        (down_probability > 0.0 && manual.switching.eval.is_loss_state);
    detail.causes_loss_rep =
        (automation_availability > 0.0 && automatic.repair.eval.is_loss_state) ||
        (down_probability > 0.0 && manual.repair.eval.is_loss_state);
    detail.repair_switch_actions = automation_availability > 0.0
        ? automatic.repair.switch_actions
        : manual.repair.switch_actions;
    detail.repair_switch_sequence_valid = automation_availability > 0.0
        ? automatic.repair.switch_sequence_valid
        : manual.repair.switch_sequence_valid;
    detail.repair_switch_sequence_message = automation_availability > 0.0
        ? automatic.repair.switch_sequence_message
        : manual.repair.switch_sequence_message;
    detail.fault_isolation_explicit = automation_availability > 0.0
        ? automatic.repair.fault_isolation_explicit
        : manual.repair.fault_isolation_explicit;
    detail.fault_isolation_message = automation_availability > 0.0
        ? automatic.repair.fault_isolation_message
        : manual.repair.fault_isolation_message;
    detail.repair_search_truncated = automatic.repair.search_truncated ||
                                     manual.repair.search_truncated;

    if (options.cyber_physical.enabled) {
      detail.eens_perfect_cyber_contribution = comp.lambda * automatic.ens_mwh;
      detail.eens_no_automation_contribution = comp.lambda * manual.ens_mwh;
      detail.eens_cyber_duration_increment =
          comp.lambda * down_probability *
          (duration_only.ens_mwh - automatic.ens_mwh);
      detail.eens_cyber_control_increment =
          comp.lambda * down_probability *
          (manual.ens_mwh - duration_only.ens_mwh);
      detail.eens_contribution =
          detail.eens_perfect_cyber_contribution +
          detail.eens_cyber_duration_increment +
          detail.eens_cyber_control_increment;
    } else {
      detail.eens_contribution = comp.lambda * automatic.ens_mwh;
    }
    detail.lole_contribution = comp.lambda *
        (automation_availability * automatic.loss_duration_hr +
         down_probability * manual.loss_duration_hr);
    work.eens_perfect_cyber = detail.eens_perfect_cyber_contribution;
    work.eens_no_automation = detail.eens_no_automation_contribution;
    work.eens_duration_increment = detail.eens_cyber_duration_increment;
    work.eens_control_increment = detail.eens_cyber_control_increment;
    work.loss_frequency = comp.lambda *
        (automation_availability * (automatic.causes_loss ? 1.0 : 0.0) +
         down_probability * (manual.causes_loss ? 1.0 : 0.0));

    detail.nodal_shed_sw_mw.assign(nb, 0.0);
    detail.nodal_shed_rep_mw.assign(nb, 0.0);
    auto nodal_value = [](const std::vector<double>& values, size_t index) {
      return index < values.size() ? values[index] : 0.0;
    };

    // 鈹€鈹€ Nodal EENS accumulation 鈹€鈹€
    work.nodal_eens.assign(nb, 0.0);
    work.nodal_cif.assign(nb, 0.0);
    work.nodal_cid.assign(nb, 0.0);
    work.nodal_cid_perfect_cyber.assign(nb, 0.0);
    work.nodal_cid_no_automation.assign(nb, 0.0);
    for (size_t b = 0; b < nb; ++b) {
      const double auto_sw = nodal_value(
          automatic.switching.eval.nodal_curtailment_mw, b);
      const double auto_rep = nodal_value(
          automatic.repair.eval.nodal_curtailment_mw, b);
      const double manual_sw = nodal_value(
          manual.switching.eval.nodal_curtailment_mw, b);
      const double manual_rep = nodal_value(
          manual.repair.eval.nodal_curtailment_mw, b);
      detail.nodal_shed_sw_mw[b] =
          automation_availability * auto_sw + down_probability * manual_sw;
      detail.nodal_shed_rep_mw[b] =
          automation_availability * auto_rep + down_probability * manual_rep;
      const double auto_nodal_ens = auto_sw * automatic.tau_sw_hr +
                                    auto_rep * automatic.tau_rep_hr;
      const double manual_nodal_ens = manual_sw * manual.tau_sw_hr +
                                      manual_rep * manual.tau_rep_hr;
      work.nodal_eens[b] = comp.lambda *
          (automation_availability * auto_nodal_ens +
           down_probability * manual_nodal_ens);

      const bool auto_loss = auto_sw > 0.01 || auto_rep > 0.01;
      const bool manual_loss = manual_sw > 0.01 || manual_rep > 0.01;
      const double auto_duration =
          (auto_sw > 0.01 ? automatic.tau_sw_hr : 0.0) +
          (auto_rep > 0.01 ? automatic.tau_rep_hr : 0.0);
      const double manual_duration =
          (manual_sw > 0.01 ? manual.tau_sw_hr : 0.0) +
          (manual_rep > 0.01 ? manual.tau_rep_hr : 0.0);
      work.nodal_cif[b] = comp.lambda *
          (automation_availability * (auto_loss ? 1.0 : 0.0) +
           down_probability * (manual_loss ? 1.0 : 0.0));
      work.nodal_cid[b] = comp.lambda *
          (automation_availability * auto_duration +
           down_probability * manual_duration);
      work.nodal_cid_perfect_cyber[b] = comp.lambda * auto_duration;
      work.nodal_cid_no_automation[b] = comp.lambda * manual_duration;
    }

    if (options.verbose) {
      spdlog::info("FMEA: {} -> shed={:.2f} MW, EENS_contrib={:.2f} MWh/yr",
                   comp.name, detail.shed_rep_mw, detail.eens_contribution);
    }

    // A class that can never occur (probability 0) must not mark the
    // contingency as loss-causing; mirrors the causes_loss_sw/rep gating.
    work.causes_loss =
        (automation_availability > 0.0 && automatic.causes_loss) ||
        (down_probability > 0.0 && manual.causes_loss);
    work.detail = std::move(detail);
    return work;
  };

  std::vector<FMEAContingencyWorkResult> work_results(catalog.size());
  // Cyber-conditioned evaluations parallelize exactly like the physical path:
  // each worker owns its contingency's system copies and OPF instances, so the
  // extra per-class stage solves introduce no shared mutable state.
  const int fmea_workers = options.enable_parallel
      ? resolve_reliability_worker_count(options.parallel_threads,
                                         static_cast<int>(catalog.size()))
      : 1;
  result.parallel_workers = fmea_workers;
  result.parallel_effective = options.enable_parallel && fmea_workers > 1 &&
                              catalog.size() > 1U;
  result.parallel_mode = result.parallel_effective
      ? "parallel-fmea-contingencies"
      : (options.enable_parallel ? "serial/insufficient-work"
                                 : "serial/disabled");
  result.parallel_execution = util::make_parallel_execution_info(
      options.enable_parallel, options.parallel_threads,
      static_cast<int>(catalog.size()), "parallel-fmea-contingencies",
      "contingencies");
  result.parallel_execution.effective = result.parallel_effective;
  result.parallel_execution.resolved_workers = fmea_workers;
  result.parallel_execution.mode = result.parallel_mode;
  if (!result.parallel_effective && options.enable_parallel) {
    result.parallel_execution.guard_reason =
        util::insufficient_work_reason(result.parallel_execution);
  }

  if (result.parallel_effective) {
    spdlog::info("FMEA: evaluating contingencies in parallel with {} workers",
                 fmea_workers);
    util::ThreadPool pool(fmea_workers);
    pool.parallel_for_dynamic(
        catalog.size(),
        [&](size_t i) {
          work_results[i] = evaluate_fmea_contingency(catalog[i]);
        },
        fmea_workers);
    result.parallel_execution.actual_parallel_evaluations =
        static_cast<long long>(catalog.size());
  } else {
    for (size_t i = 0; i < catalog.size(); ++i) {
      work_results[i] = evaluate_fmea_contingency(catalog[i]);
    }
    result.parallel_execution.serial_evaluations =
        static_cast<long long>(catalog.size());
  }

  for (auto& work : work_results) {
    const auto& detail = work.detail;
    result.eens_mwh_yr += detail.eens_contribution;
    result.lole_hr_yr += detail.lole_contribution;
    result.cyber_physical.eens_perfect_cyber_mwh_yr += work.eens_perfect_cyber;
    result.cyber_physical.eens_no_automation_mwh_yr += work.eens_no_automation;
    result.cyber_physical.delta_cyber_duration_mwh_yr +=
        work.eens_duration_increment;
    result.cyber_physical.delta_cyber_control_mwh_yr +=
        work.eens_control_increment;
    if (work.causes_loss) {
      result.lolf_occ_yr += work.loss_frequency;
      result.n_loss_contingencies++;
    }
    for (size_t b = 0; b < nb; ++b) {
      if (b < work.nodal_eens.size()) result.nodal_eens_mwh_yr[b] += work.nodal_eens[b];
      if (b < work.nodal_cif.size()) nodal_cif[b] += work.nodal_cif[b];
      if (b < work.nodal_cid.size()) nodal_cid[b] += work.nodal_cid[b];
      if (b < work.nodal_cid_perfect_cyber.size())
        nodal_cid_perfect_cyber[b] += work.nodal_cid_perfect_cyber[b];
      if (b < work.nodal_cid_no_automation.size())
        nodal_cid_no_automation[b] += work.nodal_cid_no_automation[b];
    }
    result.contingencies.push_back(std::move(work.detail));
  }

  result.edns_mw = result.eens_mwh_yr / 8760.0;

  // Sort contingencies by EENS contribution descending
  std::sort(result.contingencies.begin(), result.contingencies.end(),
            [](const FMEAContingencyDetail& a, const FMEAContingencyDetail& b) {
              return a.eens_contribution > b.eens_contribution;
            });

  // Compute distribution indices from nodal CIF/CID
  result.distribution_idx = compute_distribution_indices(sys, nodal_cif, nodal_cid);
  if (result.cyber_physical.enabled) {
    auto& cyber = result.cyber_physical;
    cyber.eens_adjusted_mwh_yr = result.eens_mwh_yr;
    const double attainable = cyber.eens_no_automation_mwh_yr -
                              cyber.eens_perfect_cyber_mwh_yr;
    if (std::abs(attainable) > 1e-12) {
      cyber.automation_efficacy = std::clamp(
          (cyber.eens_no_automation_mwh_yr - cyber.eens_adjusted_mwh_yr) /
              attainable,
          0.0, 1.0);
    } else {
      cyber.automation_efficacy = 1.0;
    }
    const std::vector<double> zero_cif(nb, 0.0);
    cyber.saidi_perfect_cyber_hr_cust_yr =
        compute_distribution_indices(sys, zero_cif, nodal_cid_perfect_cyber).saidi;
    cyber.saidi_no_automation_hr_cust_yr =
        compute_distribution_indices(sys, zero_cif, nodal_cid_no_automation).saidi;
    if (result.distribution_idx.saidi > 1e-12) {
      cyber.cyber_caused_saidi_share = std::clamp(
          (result.distribution_idx.saidi -
           cyber.saidi_perfect_cyber_hr_cust_yr) /
              result.distribution_idx.saidi,
          0.0, 1.0);
    }
  }

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
