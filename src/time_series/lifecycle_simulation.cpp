#include "hacdcpf/time_series/lifecycle_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>

namespace hacdcpf::analysis {

namespace {

// ─── Tier 2 Stratified Sampling ───────────────────────────────────────

struct StratumDef {
  std::string name;
  std::vector<int> hour_indices;  // indices into step_results
};

// Classify each hour into an operating stratum based on Tier 1 dispatch
std::vector<StratumDef> classify_strata(
    const std::vector<AnnualStepResult>& steps) {
  // 7 strata: peak load, low RE, high curtailment, heavy charge,
  // heavy discharge, normal, low load
  std::vector<StratumDef> strata(7);
  strata[0].name = "Peak Load";
  strata[1].name = "Low Renewable";
  strata[2].name = "High Curtailment";
  strata[3].name = "Heavy Charging";
  strata[4].name = "Heavy Discharging";
  strata[5].name = "Normal";
  strata[6].name = "Low Load";

  if (steps.empty()) return strata;

  // Compute thresholds
  double max_load = 0.0, max_ren = 0.0, max_curt = 0.0;
  double max_ess_charge = 0.0, max_ess_discharge = 0.0;
  for (const auto& s : steps) {
    max_load = std::max(max_load, s.total_load_mw);
    max_ren = std::max(max_ren, s.total_renewable_mw);
    max_curt = std::max(max_curt, s.total_curtailment_mw);
    if (s.total_ess_mw < 0) max_ess_charge = std::max(max_ess_charge, -s.total_ess_mw);
    if (s.total_ess_mw > 0) max_ess_discharge = std::max(max_ess_discharge, s.total_ess_mw);
  }

  const double peak_thresh = 0.85 * max_load;
  const double low_load_thresh = 0.3 * max_load;
  const double low_ren_thresh = 0.1 * (max_ren > 0 ? max_ren : 1.0);
  const double high_curt_thresh = 0.3 * (max_curt > 0 ? max_curt : 1.0);
  const double heavy_charge_thresh = 0.5 * (max_ess_charge > 0 ? max_ess_charge : 1.0);
  const double heavy_discharge_thresh = 0.5 * (max_ess_discharge > 0 ? max_ess_discharge : 1.0);

  for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
    const auto& s = steps[static_cast<size_t>(i)];
    if (s.total_load_mw >= peak_thresh) {
      strata[0].hour_indices.push_back(i);
    } else if (max_ren > 0 && s.total_renewable_mw <= low_ren_thresh) {
      strata[1].hour_indices.push_back(i);
    } else if (max_curt > 0 && s.total_curtailment_mw >= high_curt_thresh) {
      strata[2].hour_indices.push_back(i);
    } else if (s.total_ess_mw < -heavy_charge_thresh) {
      strata[3].hour_indices.push_back(i);
    } else if (s.total_ess_mw > heavy_discharge_thresh) {
      strata[4].hour_indices.push_back(i);
    } else if (s.total_load_mw <= low_load_thresh) {
      strata[6].hour_indices.push_back(i);
    } else {
      strata[5].hour_indices.push_back(i);
    }
  }
  return strata;
}

// Select representative hours from each stratum (evenly spaced)
std::vector<int> select_sample_hours(
    const std::vector<StratumDef>& strata, int samples_per_stratum) {
  std::vector<int> sampled;
  for (const auto& st : strata) {
    if (st.hour_indices.empty()) continue;
    const int n = std::min(samples_per_stratum,
                           static_cast<int>(st.hour_indices.size()));
    const int stride = std::max(1, static_cast<int>(st.hour_indices.size()) / n);
    for (int j = 0; j < n; ++j) {
      sampled.push_back(st.hour_indices[static_cast<size_t>(j * stride)]);
    }
  }
  // Deduplicate and sort
  std::sort(sampled.begin(), sampled.end());
  sampled.erase(std::unique(sampled.begin(), sampled.end()), sampled.end());
  return sampled;
}

// ─── Error Bound Computation ──────────────────────────────────────────

DispatchErrorBound compute_dispatch_bound(
    const std::vector<AnnualStepResult>& steps,
    double step_hr,
    double loss_proxy_fraction,
    double max_emission_factor) {
  DispatchErrorBound b;
  b.max_loss_fraction = loss_proxy_fraction;
  b.max_emission_factor = max_emission_factor;
  double sum_load = 0.0;
  for (const auto& s : steps) {
    sum_load += s.total_load_mw * step_hr;
  }
  b.bound_tco2 = max_emission_factor * loss_proxy_fraction * sum_load;
  return b;
}

SamplingErrorBound compute_sampling_bound(
    const std::vector<StratumDef>& strata,
    const std::vector<AnnualStepResult>& steps,
    int samples_per_stratum,
    double confidence_level,
    double step_hr,
    double max_emission_factor,
    double loss_proxy_fraction) {
  SamplingErrorBound b;
  b.confidence_level = confidence_level;
  // z-score for common confidence levels
  if (confidence_level >= 0.99) b.z_score = 2.576;
  else if (confidence_level >= 0.95) b.z_score = 1.96;
  else if (confidence_level >= 0.90) b.z_score = 1.645;
  else b.z_score = 1.28;

  b.num_strata = static_cast<int>(strata.size());

  // ── Phase 1: Compute within-stratum variance of carbon proxy ──
  // Since Tier 1 dispatch provides generation for ALL hours, the carbon
  // total is already available from the full sum. Tier 2 only adds PF
  // corrections. The sampling uncertainty is about the PF correction
  // residual, not the absolute carbon. The correction variance is
  // scaled by loss_proxy_fraction² (PF correction magnitude relative
  // to dispatch proxy).
  //
  // ξ_t = gen_mw_t × emission_factor × step_hr   (carbon proxy tCO2)
  // δ_t = ξ_t^PF - ξ_t^proxy  (correction from detailed PF)
  // Var(δ_t) ≈ loss_proxy² × Var(ξ_t)  within each stratum

  const double ef = std::max(max_emission_factor, 0.01);
  const double correction_scale = loss_proxy_fraction * ef;
  // correction_scale² converts gen-MWh variance to correction-tCO2 variance

  // Compute raw carbon proxy variance per stratum and Neyman weights
  struct StratumInfo {
    double sigma_correction{0.0}; // std dev of correction in tCO2
    int N{0};
  };
  std::vector<StratumInfo> sinfo(strata.size());

  double neyman_denom = 0.0;
  for (size_t m = 0; m < strata.size(); ++m) {
    const auto& st = strata[m];
    int Nm = static_cast<int>(st.hour_indices.size());
    sinfo[m].N = Nm;
    if (Nm <= 1) continue;

    // Compute within-stratum variance of carbon proxy
    double mean = 0.0;
    for (int idx : st.hour_indices) {
      mean += steps[static_cast<size_t>(idx)].total_gen_mw * step_hr;
    }
    mean /= Nm;

    double var_gen = 0.0;
    for (int idx : st.hour_indices) {
      double v = steps[static_cast<size_t>(idx)].total_gen_mw * step_hr - mean;
      var_gen += v * v;
    }
    var_gen /= (Nm - 1);

    // Correction variance: loss_proxy² × emission_factor² × gen variance
    double var_corr = correction_scale * correction_scale * var_gen;
    sinfo[m].sigma_correction = std::sqrt(var_corr);
    neyman_denom += Nm * sinfo[m].sigma_correction;
  }

  // ── Phase 2: Neyman allocation of total sample budget ──
  int total_budget = samples_per_stratum * static_cast<int>(strata.size());
  total_budget = std::max(total_budget, static_cast<int>(strata.size()));

  double total_var = 0.0;
  for (size_t m = 0; m < strata.size(); ++m) {
    const auto& st = strata[m];
    SamplingErrorBound::Stratum sd;
    sd.name = st.name;
    sd.population_size = sinfo[m].N;

    if (sd.population_size <= 1) {
      sd.sample_size = sd.population_size;
      sd.variance = 0.0;
      b.strata.push_back(sd);
      b.total_sampled_hours += sd.sample_size;
      continue;
    }

    // Neyman-allocated sample size
    int nm;
    if (neyman_denom > 0.0) {
      double nf = total_budget * (sinfo[m].N * sinfo[m].sigma_correction) / neyman_denom;
      nm = std::max(1, static_cast<int>(std::round(nf)));
    } else {
      nm = samples_per_stratum;
    }
    nm = std::min(nm, sd.population_size);
    sd.sample_size = nm;
    b.total_sampled_hours += nm;

    double var_corr = sinfo[m].sigma_correction * sinfo[m].sigma_correction;
    sd.variance = var_corr;

    // Stratified variance contribution: N_m² / n_m × σ_corr² × (1 - n_m/N_m)
    double fpc = 1.0 - static_cast<double>(nm) / sd.population_size;
    double contrib = static_cast<double>(sd.population_size) *
                     static_cast<double>(sd.population_size) /
                     nm * var_corr * fpc;
    total_var += contrib;
    b.strata.push_back(sd);
  }

  b.variance_estimate = total_var;
  b.bound_tco2 = b.z_score * std::sqrt(total_var);
  return b;
}

StorageCarbonErrorBound compute_storage_carbon_bound(
    const std::vector<AnnualStepResult>& steps,
    const std::vector<int>& sampled_hours,
    double step_hr,
    int num_storage) {
  StorageCarbonErrorBound b;

  // Estimate Lipschitz constant K_w from generation variation
  double max_gen_change = 0.0;
  for (size_t i = 1; i < steps.size(); ++i) {
    double change = std::abs(steps[i].total_gen_mw - steps[i - 1].total_gen_mw);
    max_gen_change = std::max(max_gen_change, change);
  }
  // K_w ≈ max hourly change in emission intensity proxy
  b.lipschitz_constant = max_gen_change * 0.001;  // scale to tCO2/MWh units

  // Maximum gap between sampled hours
  if (sampled_hours.size() >= 2) {
    double max_gap = 0.0;
    for (size_t i = 1; i < sampled_hours.size(); ++i) {
      double gap = (sampled_hours[i] - sampled_hours[i - 1]) * step_hr;
      max_gap = std::max(max_gap, gap);
    }
    b.max_gap_hours = max_gap;
  } else {
    b.max_gap_hours = steps.size() * step_hr;
  }

  b.bound_tco2_per_mwh = b.lipschitz_constant * b.max_gap_hours / 2.0;

  // Total bound proportional to storage throughput
  double total_ess_energy = 0.0;
  for (const auto& s : steps) {
    total_ess_energy += std::abs(s.total_ess_mw) * step_hr;
  }
  b.bound_tco2 = b.bound_tco2_per_mwh * total_ess_energy *
                  std::max(1, num_storage);
  return b;
}

// ─── Carbon estimation from sampled hours ─────────────────────────────

/// Compute average emission factor from system generators
static double compute_avg_emission_factor(const HybridPowerSystem& sys) {
  double avg_ef = 0.0;
  int ef_count = 0;
  for (const auto& g : sys.ac.generators) {
    if (g.emission_factor_tco2_mwh > 0) {
      avg_ef += g.emission_factor_tco2_mwh;
      ++ef_count;
    }
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (sg.co2_emission_rate > 0) {
      avg_ef += sg.co2_emission_rate;
      ++ef_count;
    }
  }
  if (ef_count > 0) avg_ef /= ef_count;
  else avg_ef = 0.5;
  return avg_ef;
}

/// Dense carbon estimate: sum gen × emission_factor × step_hr over ALL hours
double compute_dense_carbon(
    const HybridPowerSystem& sys,
    const AnnualProductionSimResult& ann_result) {
  if (ann_result.step_results.empty()) return 0.0;
  double avg_ef = compute_avg_emission_factor(sys);
  double total = 0.0;
  for (const auto& s : ann_result.step_results) {
    total += s.total_gen_mw * avg_ef * ann_result.step_duration_hr;
  }
  return total;
}

double estimate_annual_carbon(
    const HybridPowerSystem& sys,
    const AnnualProductionSimResult& ann_result,
    const std::vector<StratumDef>& strata,
    const std::vector<int>& sampled_hours) {
  if (ann_result.step_results.empty()) return 0.0;

  double avg_ef = compute_avg_emission_factor(sys);

  // Weighted extrapolation: each sampled hour represents its stratum
  double total_carbon = 0.0;
  for (const auto& st : strata) {
    if (st.hour_indices.empty()) continue;
    // Find sampled hours in this stratum
    double stratum_sample_carbon = 0.0;
    int sample_count = 0;
    for (int sh : sampled_hours) {
      for (int hi : st.hour_indices) {
        if (hi == sh) {
          const auto& s = ann_result.step_results[static_cast<size_t>(sh)];
          stratum_sample_carbon += s.total_gen_mw * avg_ef * ann_result.step_duration_hr;
          ++sample_count;
          break;
        }
      }
    }
    if (sample_count > 0) {
      // Extrapolate stratum total
      double stratum_total = stratum_sample_carbon / sample_count *
                             static_cast<double>(st.hour_indices.size());
      total_carbon += stratum_total;
    }
  }
  return total_carbon;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// Public API implementation
// ═══════════════════════════════════════════════════════════════════════

LifecycleSimResult run_lifecycle_simulation(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const LifecycleSimOptions& opts) {
  LifecycleSimResult result;
  result.num_years = opts.num_years;

  // Working copy of the system
  HybridPowerSystem work_sys = sys;

  // Synthesize ac.loads from bus pd_mw if none exist (MATPOWER-parsed cases)
  if (work_sys.ac.loads.empty()) {
    int load_idx = 1;
    for (const auto& b : work_sys.ac.buses) {
      if (b.pd_mw > 1e-6 || b.qd_mvar > 1e-6) {
        Load ld;
        ld.index = load_idx++;
        ld.bus = b.index;
        ld.in_service = true;
        ld.name = "Bus" + std::to_string(b.index) + "-Load";
        ld.p_mw = b.pd_mw;
        ld.q_mvar = b.qd_mvar;
        ld.scaling = 1.0;
        ld.profile_id = 0;  // default load profile
        work_sys.ac.loads.push_back(ld);
      }
    }
  }

  // Track original capacities for derating
  struct OrigPV { int index; double pmax_mw; };
  std::vector<OrigPV> orig_pvs;
  for (const auto& pv : work_sys.ac.pv_systems) {
    orig_pvs.push_back({pv.index, pv.pmax_mw});
  }

  // Save original loads (after synthesis) for load growth re-derivation
  struct OrigLoad { int index; double p_mw; double q_mvar; };
  std::vector<OrigLoad> orig_loads;
  for (const auto& ld : work_sys.ac.loads) {
    orig_loads.push_back({ld.index, ld.p_mw, ld.q_mvar});
  }

  // Save original bus loads for bus-level growth
  struct OrigBusLoad { int index; double pd_mw; double qd_mvar; };
  std::vector<OrigBusLoad> orig_bus_loads;
  for (const auto& b : work_sys.ac.buses) {
    orig_bus_loads.push_back({b.index, b.pd_mw, b.qd_mvar});
  }

  struct OrigStorage { int index; double e_rated_mwh; };
  std::vector<OrigStorage> orig_storages;
  for (const auto& st : work_sys.ac.storage) {
    orig_storages.push_back({st.index, st.e_rated_mwh});
  }

  // Compute max emission factor for bounds
  double max_ef = 0.0;
  for (const auto& g : sys.ac.generators) {
    max_ef = std::max(max_ef, g.emission_factor_tco2_mwh);
  }
  for (const auto& sg : sys.ac.static_generators) {
    max_ef = std::max(max_ef, sg.co2_emission_rate);
  }
  if (max_ef <= 0.0) max_ef = 0.5;

  double cumulative_cost = 0.0;
  double cumulative_carbon = 0.0;
  double cumulative_replacement = 0.0;

  for (int year = 1; year <= opts.num_years; ++year) {
    YearResult yr;
    yr.year = year;

    // ─── Lifecycle state update ─────────────────────────────────────

    // Load growth
    yr.load_growth_factor = std::pow(1.0 + opts.load_growth_rate, year - 1);
    for (auto& ld : work_sys.ac.loads) {
      for (const auto& orig_ld : orig_loads) {
        if (orig_ld.index == ld.index) {
          ld.p_mw = orig_ld.p_mw * yr.load_growth_factor;
          ld.q_mvar = orig_ld.q_mvar * yr.load_growth_factor;
          break;
        }
      }
    }
    // Also scale bus loads
    for (auto& b : work_sys.ac.buses) {
      for (const auto& orig_b : orig_bus_loads) {
        if (orig_b.index == b.index) {
          b.pd_mw = orig_b.pd_mw * yr.load_growth_factor;
          b.qd_mvar = orig_b.qd_mvar * yr.load_growth_factor;
          break;
        }
      }
    }

    // PV derating
    for (auto& pv : work_sys.ac.pv_systems) {
      double derating = std::pow(1.0 - opts.pv_annual_derating, year - 1);
      for (const auto& op : orig_pvs) {
        if (op.index == pv.index) {
          pv.pmax_mw = op.pmax_mw * derating;
          pv.p_mw = std::min(pv.p_mw, pv.pmax_mw);
          RenewableYearState rs;
          rs.ren_index = pv.index;
          rs.name = pv.name;
          rs.original_capacity_mw = op.pmax_mw;
          rs.derated_capacity_mw = pv.pmax_mw;
          rs.derating_factor = derating;
          yr.renewable_states.push_back(rs);
          break;
        }
      }
    }

    // Battery degradation
    for (auto& st : work_sys.ac.storage) {
      for (const auto& os : orig_storages) {
        if (os.index == st.index) {
          // Calendar degradation
          st.soh_calendar = std::max(0.0,
              1.0 - opts.calendar_degradation_per_year * (year - 1));
          // Cycle degradation
          st.soh_cycle = std::max(0.0,
              1.0 - opts.cycle_degradation_per_cycle * st.current_cycles);
          // Combined SOH
          st.soh = std::min(st.soh_calendar, st.soh_cycle);

          // Check for replacement
          const double eol_fraction =
              st.eol_percent > 1.0 ? st.eol_percent / 100.0
                                   : st.eol_percent;
          if (st.soh <= std::clamp(eol_fraction, 0.0, 1.0)) {
            ReplacementEvent re;
            re.year = year;
            re.storage_index = st.index;
            re.storage_name = st.name;
            re.old_soh = st.soh;
            re.replacement_cost_usd = st.replacement_cost * os.e_rated_mwh * 1000.0;
            yr.replacements.push_back(re);
            result.all_replacements.push_back(re);
            result.total_replacements++;
            cumulative_replacement += re.replacement_cost_usd;

            // Reset after replacement
            st.soh = 1.0;
            st.soh_cycle = 1.0;
            st.soh_calendar = 1.0;
            st.current_cycles = 0;
          }

          // Update effective capacity
          st.e_rated_mwh = os.e_rated_mwh * st.soh;
          st.e_mwh = st.e_rated_mwh * st.soc_init;

          StorageYearState ss;
          ss.storage_index = st.index;
          ss.name = st.name;
          ss.soh = st.soh;
          ss.soh_cycle = st.soh_cycle;
          ss.soh_calendar = st.soh_calendar;
          ss.effective_capacity_mwh = st.e_rated_mwh;
          ss.cumulative_cycles = st.current_cycles;
          ss.replaced = !yr.replacements.empty() &&
              yr.replacements.back().storage_index == st.index;
          yr.storage_states.push_back(ss);
          break;
        }
      }
    }

    // ─── Tier 1: Annual chronological dispatch ──────────────────────

    AnnualProductionSimOptions ann_opts;
    ann_opts.skip_replay = true;  // schedule only for speed
    ann_opts.enforce_cyclic_soc = true;
    ann_opts.verbose = false;
    ann_opts.ts_pf_options.run_opf = false;

    auto ann_result = solve_annual_production_simulation(work_sys, ts_data, ann_opts);

    yr.annual_cost = ann_result.total_cost;
    yr.annual_gen_mwh = ann_result.total_gen_mwh;
    yr.annual_load_mwh = ann_result.total_load_mwh;
    yr.annual_renewable_mwh = ann_result.total_renewable_mwh;
    yr.annual_curtailment_mwh = ann_result.total_curtailment_mwh;
    yr.annual_ens_mwh = ann_result.total_ens_mwh;
    yr.annual_loss_mwh = ann_result.total_loss_mwh;
    yr.feasible = ann_result.feasible;

    // Update storage cycle counts from annual results
    for (const auto& sa : ann_result.storage_stats) {
      for (auto& st : work_sys.ac.storage) {
        if (st.name == sa.name) {
          st.current_cycles += static_cast<int>(sa.cycles);
          // Update year state with cycles info
          for (auto& ss : yr.storage_states) {
            if (ss.storage_index == st.index) {
              ss.cycles_this_year = sa.cycles;
              ss.cumulative_cycles = st.current_cycles;
              break;
            }
          }
          break;
        }
      }
    }

    // ─── Tier 2: Stratified sampling + carbon estimation ────────────
    // Tier 1 provides dispatch for ALL hours, so the primary carbon
    // estimate uses the dense (all-hours) summation. The stratified
    // sampling selects hours for PF validation (Tier 2 corrections).
    // The sampling error bound covers only the PF correction residual.

    auto strata = classify_strata(ann_result.step_results);
    auto sampled_hours = select_sample_hours(strata, opts.tier2_samples_per_stratum);

    double dense_carbon = compute_dense_carbon(work_sys, ann_result);
    double sampled_carbon = estimate_annual_carbon(
        work_sys, ann_result, strata, sampled_hours);

    // Use dense estimate as primary (Tier 1 gives us all hours)
    yr.annual_carbon_tco2 = dense_carbon;
    yr.avg_carbon_intensity = (yr.annual_load_mwh > 0)
        ? yr.annual_carbon_tco2 / yr.annual_load_mwh : 0.0;

    // ─── Theoretical error bounds ───────────────────────────────────

    yr.bounds.dispatch = compute_dispatch_bound(
        ann_result.step_results, ann_result.step_duration_hr,
        opts.loss_proxy_fraction, max_ef);

    yr.bounds.sampling = compute_sampling_bound(
        strata, ann_result.step_results, opts.tier2_samples_per_stratum,
        opts.confidence_level, ann_result.step_duration_hr,
        max_ef, opts.loss_proxy_fraction);

    yr.bounds.storage_carbon = compute_storage_carbon_bound(
        ann_result.step_results, sampled_hours, ann_result.step_duration_hr,
        static_cast<int>(work_sys.ac.storage.size()));

    yr.bounds.total_bound_tco2 = yr.bounds.dispatch.bound_tco2 +
                                  yr.bounds.sampling.bound_tco2 +
                                  yr.bounds.storage_carbon.bound_tco2;

    // ─── Cross-validation: dense vs. sampled carbon ─────────────────

    auto& cv = yr.cross_validation;
    cv.dense_carbon_tco2 = dense_carbon;
    cv.sampled_carbon_tco2 = sampled_carbon;
    cv.sampling_gap_tco2 =
        std::abs(cv.dense_carbon_tco2 - cv.sampled_carbon_tco2);
    cv.sampling_gap_pct =
        (cv.dense_carbon_tco2 > 1e-12)
            ? cv.sampling_gap_tco2 / cv.dense_carbon_tco2 * 100.0 : 0.0;
    cv.total_bound_pct =
        (cv.dense_carbon_tco2 > 1e-12)
            ? yr.bounds.total_bound_tco2 / cv.dense_carbon_tco2 * 100.0
            : 0.0;
    cv.dispatch_bound_pct =
        (cv.dense_carbon_tco2 > 1e-12)
            ? yr.bounds.dispatch.bound_tco2 / cv.dense_carbon_tco2 * 100.0
            : 0.0;
    cv.sampling_bound_pct =
        (cv.dense_carbon_tco2 > 1e-12)
            ? yr.bounds.sampling.bound_tco2 / cv.dense_carbon_tco2 * 100.0
            : 0.0;
    cv.storage_bound_pct =
        (cv.dense_carbon_tco2 > 1e-12)
            ? yr.bounds.storage_carbon.bound_tco2 / cv.dense_carbon_tco2 * 100.0
            : 0.0;

    // ─── NPV accumulation ───────────────────────────────────────────

    double discount = std::pow(1.0 + opts.discount_rate, -(year - 1));
    cumulative_cost += yr.annual_cost * discount;
    for (const auto& re : yr.replacements) {
      cumulative_cost += re.replacement_cost_usd * discount;
    }
    cumulative_carbon += yr.annual_carbon_tco2;

    result.year_results.push_back(yr);
  }

  result.npv_total_cost = cumulative_cost;
  result.total_carbon_tco2 = cumulative_carbon;
  result.total_replacement_cost = cumulative_replacement;
  result.feasible = std::all_of(result.year_results.begin(),
      result.year_results.end(), [](const YearResult& y) { return y.feasible; });

  // Generate summary
  {
    std::ostringstream oss;
    oss << "Lifecycle simulation: " << opts.num_years << " years, "
        << "NPV cost: $" << result.npv_total_cost
        << ", total carbon: " << result.total_carbon_tco2 << " tCO2"
        << ", replacements: " << result.total_replacements
        << " ($" << result.total_replacement_cost << ")";
    result.summary_text = oss.str();
  }

  return result;
}

// ═══════════════════════════════════════════════════════════════════════
// Capacity Scaling
// ═══════════════════════════════════════════════════════════════════════

void apply_capacity_scaling(HybridPowerSystem& sys, const CapacityScaling& s) {
  // Scale PV systems
  for (auto& pv : sys.ac.pv_systems) {
    pv.pmax_mw *= s.pv_scale;
    pv.pmin_mw *= s.pv_scale;
    pv.sn_mva *= s.pv_scale;
    if (pv.p_mw > pv.pmax_mw) pv.p_mw = pv.pmax_mw;
  }

  // Scale wind renewable generators
  for (auto& ren : sys.ac.renewable_gens) {
    if (ren.type == RenewableType::Wind) {
      ren.p_rated_mw *= s.wind_scale;
      if (ren.p_mw > ren.p_rated_mw) ren.p_mw = ren.p_rated_mw;
    }
  }

  // Scale battery storage
  for (auto& st : sys.ac.storage) {
    st.p_rated_mw *= s.bess_power_scale;
    st.pmax_mw *= s.bess_power_scale;
    st.pmin_mw *= s.bess_power_scale;  // negative, scaling preserves sign
    st.e_rated_mwh *= s.bess_energy_scale;
    st.e_mwh = st.e_rated_mwh * 0.5;  // reset to 50% SOC
  }

  // Scale diesel/gas static generators
  for (auto& sg : sys.ac.static_generators) {
    sg.p_rated_mw *= s.diesel_scale;
    sg.pmax_mw *= s.diesel_scale;
    sg.pmin_mw *= s.diesel_scale;
    if (sg.p_mw > sg.pmax_mw) sg.p_mw = sg.pmax_mw;
  }
}

// Helper: compute installed capacities after scaling
static ScenarioResult make_scenario_summary(
    const HybridPowerSystem& sys,
    const CapacityScaling& scaling,
    const std::string& label,
    const LifecycleSimResult& result) {
  ScenarioResult sr;
  sr.scaling = scaling;
  sr.label = label;
  sr.npv_total_cost = result.npv_total_cost;
  sr.total_carbon_tco2 = result.total_carbon_tco2;
  sr.total_replacement_cost = result.total_replacement_cost;
  sr.total_replacements = result.total_replacements;
  sr.feasible = result.feasible;

  for (const auto& yr : result.year_results) {
    sr.yearly_carbon.push_back(yr.annual_carbon_tco2);
    sr.yearly_cost.push_back(yr.annual_cost);
  }

  // Installed capacities
  for (const auto& pv : sys.ac.pv_systems) sr.total_pv_mw += pv.pmax_mw;
  for (const auto& ren : sys.ac.renewable_gens) {
    if (ren.type == RenewableType::Wind)
      sr.total_wind_mw += ren.p_rated_mw;
  }
  for (const auto& st : sys.ac.storage) {
    sr.total_bess_mw += st.p_rated_mw;
    sr.total_bess_mwh += st.e_rated_mwh;
  }
  for (const auto& sg : sys.ac.static_generators) sr.total_diesel_mw += sg.p_rated_mw;

  return sr;
}

LifecycleCompareResult run_lifecycle_comparison(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const LifecycleSimOptions& opts,
    const std::string& sweep_param,
    const std::vector<double>& scale_values,
    const CapacityScaling& base_scaling) {

  LifecycleCompareResult cmp;
  cmp.sweep_parameter = sweep_param;
  cmp.num_scenarios = static_cast<int>(scale_values.size());

  for (double sv : scale_values) {
    CapacityScaling scaling = base_scaling;
    std::string label;

    if (sweep_param == "pv") {
      scaling.pv_scale = sv;
      label = "PV x" + std::to_string(sv).substr(0, 4);
    } else if (sweep_param == "wind") {
      scaling.wind_scale = sv;
      label = "Wind x" + std::to_string(sv).substr(0, 4);
    } else if (sweep_param == "bess_power") {
      scaling.bess_power_scale = sv;
      label = "BESS-P x" + std::to_string(sv).substr(0, 4);
    } else if (sweep_param == "bess_energy") {
      scaling.bess_energy_scale = sv;
      label = "BESS-E x" + std::to_string(sv).substr(0, 4);
    } else if (sweep_param == "diesel") {
      scaling.diesel_scale = sv;
      label = "Diesel x" + std::to_string(sv).substr(0, 4);
    } else {
      label = "Scenario";
    }

    // Copy system, apply scaling, run lifecycle
    HybridPowerSystem scaled_sys = sys;
    apply_capacity_scaling(scaled_sys, scaling);
    auto result = run_lifecycle_simulation(scaled_sys, ts_data, opts);
    cmp.scenarios.push_back(
        make_scenario_summary(scaled_sys, scaling, label, result));
  }

  return cmp;
}

}  // namespace hacdcpf::analysis
