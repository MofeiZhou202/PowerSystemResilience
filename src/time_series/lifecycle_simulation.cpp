#include "hacdcpf/time_series/lifecycle_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

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

// Compute one Neyman allocation used by both the estimator and its bound.  The
// previous implementation computed an allocation for the bound but sampled a
// different fixed count, so the reported interval did not describe the
// estimator that was actually run.
std::vector<int> compute_sample_allocation(
    const std::vector<StratumDef>& strata,
    const std::vector<AnnualStepResult>& steps,
    int samples_per_stratum,
    double step_hr,
    double max_emission_factor,
    double loss_proxy_fraction) {
  std::vector<int> allocation(strata.size(), 0);
  const int budget = std::max(1, samples_per_stratum) *
                     static_cast<int>(strata.size());
  std::vector<double> weights(strata.size(), 0.0);
  double denominator = 0.0;
  for (size_t m = 0; m < strata.size(); ++m) {
    const auto& st = strata[m];
    if (st.hour_indices.empty()) continue;
    double mean = 0.0;
    for (int idx : st.hour_indices)
      mean += steps[static_cast<size_t>(idx)].total_gen_mw * step_hr;
    mean /= static_cast<double>(st.hour_indices.size());
    double variance = 0.0;
    if (st.hour_indices.size() > 1) {
      for (int idx : st.hour_indices) {
        const double d = steps[static_cast<size_t>(idx)].total_gen_mw * step_hr - mean;
        variance += d * d;
      }
      variance /= static_cast<double>(st.hour_indices.size() - 1);
    }
    const double sigma = std::sqrt(variance) *
        std::max(0.01, max_emission_factor) *
        std::max(0.0, loss_proxy_fraction);
    weights[m] = static_cast<double>(st.hour_indices.size()) * sigma;
    denominator += weights[m];
  }
  for (size_t m = 0; m < strata.size(); ++m) {
    const int population = static_cast<int>(strata[m].hour_indices.size());
    if (population == 0) continue;
    int n = denominator > 0.0
        ? static_cast<int>(std::round(budget * weights[m] / denominator))
        : std::max(1, samples_per_stratum);
    allocation[m] = std::clamp(n, 1, population);
  }
  return allocation;
}

// Select representative hours from each stratum (evenly spaced) using the
// same per-stratum allocation consumed by compute_sampling_bound().
std::vector<int> select_sample_hours(
    const std::vector<StratumDef>& strata,
    const std::vector<int>& allocation) {
  std::vector<int> sampled;
  for (size_t m = 0; m < strata.size(); ++m) {
    const auto& st = strata[m];
    if (st.hour_indices.empty()) continue;
    const int n = std::min(allocation[m],
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
    const std::vector<int>& allocation,
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

  // ── Phase 1: Compute within-stratum variance of the carbon proxy ──
  // Tier 1 already provides all dispatch hours.  Tier 2 is therefore a
  // stratified reconstruction check of the same proxy; no unexecuted PF
  // correction is claimed here.  The legacy loss-proxy factor remains an
  // explicit conservative scale in the bound and is not hidden as measured
  // network carbon.
  //
  // ξ_t = gen_mw_t × emission_factor × step_hr   (carbon proxy tCO2)
  // δ_t = ξ_t^PF - ξ_t^proxy  (correction from detailed PF)
  // Var(δ_t) ≈ loss_proxy² × Var(ξ_t)  within each stratum

  const double ef = std::max(max_emission_factor, 0.01);
  const double correction_scale = loss_proxy_fraction * ef;
  // correction_scale² converts gen-MWh variance to correction-tCO2 variance

  double total_var = 0.0;
  for (size_t m = 0; m < strata.size(); ++m) {
    const auto& st = strata[m];
    SamplingErrorBound::Stratum sd;
    sd.name = st.name;
    sd.population_size = static_cast<int>(st.hour_indices.size());

    if (sd.population_size <= 1) {
      sd.sample_size = sd.population_size;
      sd.variance = 0.0;
      b.strata.push_back(sd);
      b.total_sampled_hours += sd.sample_size;
      continue;
    }

    const int nm = std::clamp(
        m < allocation.size() ? allocation[m] : 1, 1, sd.population_size);
    sd.sample_size = nm;
    b.total_sampled_hours += nm;

    double mean = 0.0;
    for (int idx : st.hour_indices)
      mean += steps[static_cast<size_t>(idx)].total_gen_mw * step_hr;
    mean /= static_cast<double>(sd.population_size);
    double var_gen = 0.0;
    for (int idx : st.hour_indices) {
      const double d = steps[static_cast<size_t>(idx)].total_gen_mw * step_hr - mean;
      var_gen += d * d;
    }
    var_gen /= static_cast<double>(std::max(1, sd.population_size - 1));
    double var_corr = correction_scale * correction_scale * var_gen;
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

static bool has_authored_emission_factor(const HybridPowerSystem& sys) {
  for (const auto& g : sys.ac.generators)
    if (g.in_service && g.emission_factor_tco2_mwh > 0.0) return true;
  for (const auto& sg : sys.ac.static_generators)
    if (sg.in_service && sg.co2_emission_rate > 0.0) return true;
  for (const auto& sg : sys.dc.dc_static_generators)
    if (sg.in_service && sg.emission_factor_tco2_mwh > 0.0) return true;
  return false;
}

static const UCSchedule* schedule_at_step(
    const AnnualProductionSimResult& ann_result, int global_step,
    int& local_step) {
  for (const auto& ws : ann_result.weekly_schedules) {
    if (global_step < ws.start_step || global_step >= ws.start_step + ws.num_steps)
      continue;
    local_step = global_step - ws.start_step;
    return &ws.uc;
  }
  local_step = -1;
  return nullptr;
}

static bool carbon_schedule_complete(const AnnualProductionSimResult& ann_result) {
  if (ann_result.step_results.empty() || ann_result.weekly_schedules.empty())
    return false;
  for (int t = 0; t < static_cast<int>(ann_result.step_results.size()); ++t) {
    int local = -1;
    if (!schedule_at_step(ann_result, t, local)) return false;
  }
  return true;
}

static double profile_value_at(const TimeSeriesData& ts, int id, int t,
                              double fallback) {
  for (const auto& p : ts.profiles) {
    if (p.id != id) continue;
    return t >= 0 && t < static_cast<int>(p.values.size())
        ? p.values[static_cast<size_t>(t)] : fallback;
  }
  return fallback;
}

static double external_grid_factor_at(const HybridPowerSystem& sys,
                                      const TimeSeriesData& ts, int t,
                                      bool& known) {
  double weighted = 0.0;
  double weight = 0.0;
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    double factor = eg.emission_factor_tco2_mwh;
    if (eg.emission_factor_profile_id >= 0)
      factor = profile_value_at(ts, eg.emission_factor_profile_id, t, factor);
    if (factor > 0.0 && std::isfinite(factor)) {
      weighted += factor;
      weight += 1.0;
    }
  }
  known = weight > 0.0;
  return known ? weighted / weight : 0.0;
}

static UCSchedule slice_schedule_for_step(const UCSchedule& src, int t) {
  UCSchedule out = src;
  auto d = [t](const std::vector<std::vector<double>>& rows) {
    std::vector<std::vector<double>> result;
    result.reserve(rows.size());
    for (const auto& row : rows) {
      result.push_back({t >= 0 && t < static_cast<int>(row.size())
                            ? row[static_cast<size_t>(t)] : 0.0});
    }
    return result;
  };
  auto i = [t](const std::vector<std::vector<int>>& rows) {
    std::vector<std::vector<int>> result;
    result.reserve(rows.size());
    for (const auto& row : rows) {
      result.push_back({t >= 0 && t < static_cast<int>(row.size())
                            ? row[static_cast<size_t>(t)] : 0});
    }
    return result;
  };
  out.gen_dispatch = d(src.gen_dispatch);
  out.gen_commit = i(src.gen_commit);
  out.ess_dispatch = d(src.ess_dispatch);
  out.ess_soc = d(src.ess_soc);
  out.renewable_dispatch = d(src.renewable_dispatch);
  out.ac_pv_dispatch = d(src.ac_pv_dispatch);
  out.ac_sgen_dispatch = d(src.ac_sgen_dispatch);
  out.external_grid_dispatch = d(src.external_grid_dispatch);
  out.flexible_load_up = d(src.flexible_load_up);
  out.flexible_load_down = d(src.flexible_load_down);
  out.dc_pv_dispatch = d(src.dc_pv_dispatch);
  out.dc_ess_dispatch = d(src.dc_ess_dispatch);
  out.dc_ess_soc = d(src.dc_ess_soc);
  out.dc_sgen_dispatch = d(src.dc_sgen_dispatch);
  out.dc_load_demand = d(src.dc_load_demand);
  out.vsc_dispatch = d(src.vsc_dispatch);
  out.dcdc_dispatch = d(src.dcdc_dispatch);
  out.vsc_direction_ac_to_dc = i(src.vsc_direction_ac_to_dc);
  out.dcdc_direction_forward = i(src.dcdc_direction_forward);
  out.market_dc_storage_dispatch_mw = d(src.market_dc_storage_dispatch_mw);
  out.market_dc_storage_soc_mwh = d(src.market_dc_storage_soc_mwh);
  out.market_dc_storage_direction_charging = i(src.market_dc_storage_direction_charging);
  return out;
}

struct CarbonStep {
  double authored_assets{0.0};
  double external_grid{0.0};
  double storage_inventory{0.0};
  double dc_assets{0.0};
  bool authored_known{false};
  bool external_known{false};
};

/// Asset-resolved source carbon for one step. External-grid imports use the
/// authored static factor or the time-varying profile when available. If no
/// source factor exists, the caller applies the explicit legacy fallback.
static CarbonStep source_carbon_components_at_step(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const AnnualProductionSimResult& ann_result,
    int global_step) {
  CarbonStep out;
  int local_t = -1;
  const UCSchedule* schedule = schedule_at_step(ann_result, global_step, local_t);
  if (!schedule) return out;
  const double dt = ann_result.step_duration_hr;
  int gen_pos = 0;
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    double p = 0.0;
    if (schedule && gen_pos < static_cast<int>(schedule->gen_dispatch.size()) &&
        local_t >= 0 && local_t < static_cast<int>(
            schedule->gen_dispatch[static_cast<size_t>(gen_pos)].size())) {
      p = schedule->gen_dispatch[static_cast<size_t>(gen_pos)]
          [static_cast<size_t>(local_t)];
    }
    if (g.emission_factor_tco2_mwh > 0.0) {
      out.authored_assets += std::max(0.0, p) * g.emission_factor_tco2_mwh * dt;
      out.authored_known = true;
    }
    ++gen_pos;
  }
  int static_pos = 0;
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    double p = std::max(0.0, sg.p_mw * sg.scaling);
    if (schedule && static_pos < static_cast<int>(schedule->ac_sgen_dispatch.size()) &&
        local_t >= 0 && local_t < static_cast<int>(
            schedule->ac_sgen_dispatch[static_cast<size_t>(static_pos)].size())) {
      p = std::max(0.0, schedule->ac_sgen_dispatch[static_cast<size_t>(static_pos)]
          [static_cast<size_t>(local_t)]);
    }
    if (sg.co2_emission_rate > 0.0) {
      out.authored_assets += p * sg.co2_emission_rate * dt;
      out.authored_known = true;
    }
    ++static_pos;
  }
  // DC static generators and PV-backed static assets are lifecycle assets,
  // even though they are not represented in the AC generator table.
  int dc_sgen_pos = 0;
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    double p = sg.p_set_mw * sg.scaling *
        profile_value_at(ts_data, sg.profile_id, global_step, 1.0);
    if (dc_sgen_pos < static_cast<int>(schedule->dc_sgen_dispatch.size()) &&
        local_t < static_cast<int>(schedule->dc_sgen_dispatch[static_cast<size_t>(dc_sgen_pos)].size()))
      p = schedule->dc_sgen_dispatch[static_cast<size_t>(dc_sgen_pos)][static_cast<size_t>(local_t)];
    if (sg.emission_factor_tco2_mwh > 0.0) {
      out.dc_assets += std::max(0.0, p) * sg.emission_factor_tco2_mwh * dt;
      out.authored_known = true;
    }
    ++dc_sgen_pos;
  }
  bool ext_known = false;
  (void)external_grid_factor_at(sys, ts_data, global_step, ext_known);
  out.external_known = ext_known;
  int ext_pos = 0;
  if (ext_known) {
    for (const auto& eg : sys.ac.external_grids) {
      if (!eg.in_service) continue;
      double factor = eg.emission_factor_tco2_mwh;
      if (eg.emission_factor_profile_id >= 0)
        factor = profile_value_at(ts_data, eg.emission_factor_profile_id,
                                  global_step, factor);
      if (ext_pos < static_cast<int>(schedule->external_grid_dispatch.size()) &&
          local_t < static_cast<int>(schedule->external_grid_dispatch[static_cast<size_t>(ext_pos)].size()) &&
          factor > 0.0 && std::isfinite(factor)) {
        out.external_grid += std::max(0.0, schedule->external_grid_dispatch[static_cast<size_t>(ext_pos)][static_cast<size_t>(local_t)]) * factor * dt;
      }
      ++ext_pos;
    }
  }
  return out;
}

struct StorageCarbonLedgerState {
  double energy_mwh{0.0};
  double carbon_tco2{0.0};
};

/// Chronological storage-carbon ledger. A storage unit carries its existing
/// inventory into the year; charging adds the contemporaneous source
/// intensity and discharging removes carbon at the pre-dispatch intensity.
static std::vector<CarbonStep> build_carbon_ledger(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const AnnualProductionSimResult& ann_result) {
  const int T = static_cast<int>(ann_result.step_results.size());
  std::vector<CarbonStep> ledger(static_cast<size_t>(std::max(0, T)));
  if (T == 0) return ledger;

  std::vector<StorageCarbonLedgerState> ac_state;
  std::vector<StorageCarbonLedgerState> dc_state;
  auto init = [](const auto& stores, auto& dst) {
    for (const auto& st : stores) {
      if (!st.in_service || st.cap_charging_strategy == "static") continue;
      const double energy = st.e_mwh > 0.0 ? st.e_mwh
          : std::max(0.0, st.e_rated_mwh * st.soc_init);
      dst.push_back({energy, energy * std::max(0.0,
          st.soc_carbon_intensity_tco2_mwh)});
    }
  };
  init(sys.ac.storage, ac_state);
  init(sys.dc.storage, dc_state);
  init(sys.dc.dc_storage, dc_state);

  for (int t = 0; t < T; ++t) {
    int local_t = -1;
    const UCSchedule* schedule = schedule_at_step(ann_result, t, local_t);
    if (!schedule) continue;
    CarbonStep out = source_carbon_components_at_step(sys, ts_data, ann_result, t);
    const double dt = ann_result.step_duration_hr;
    const double source_mwh = std::max(0.0,
        ann_result.step_results[static_cast<size_t>(t)].total_gen_mw) * dt;
    double source_intensity = (out.authored_assets + out.external_grid +
        out.dc_assets) / std::max(source_mwh, 1e-12);
    if (!std::isfinite(source_intensity) || source_intensity < 0.0)
      source_intensity = 0.0;
    if (source_intensity == 0.0 && !has_authored_emission_factor(sys) &&
        !out.external_known) {
      source_intensity = 0.5;
    }

    auto apply = [&](const auto& stores, const auto& rows, auto& states,
                     size_t state_offset) {
      size_t row = 0;
      size_t state = state_offset;
      for (const auto& st : stores) {
        if (!st.in_service || st.cap_charging_strategy == "static") continue;
        if (state >= states.size()) break;
        const double p = row < rows.size() && local_t >= 0 &&
                local_t < static_cast<int>(rows[row].size())
            ? rows[row][static_cast<size_t>(local_t)] : 0.0;
        auto& inv = states[state++];
        const double before = inv.energy_mwh > 1e-12
            ? inv.carbon_tco2 / inv.energy_mwh : 0.0;
        if (p > 0.0) {
          const double discharged = p * dt;
          out.storage_inventory += discharged * before;
          inv.energy_mwh = std::max(0.0, inv.energy_mwh - discharged);
          inv.carbon_tco2 = std::max(0.0, inv.carbon_tco2 - discharged * before);
        } else if (p < 0.0) {
          const double charged = -p * dt;
          const double eta = std::clamp(st.eta_charge, 0.0, 1.0);
          inv.energy_mwh += charged * eta;
          inv.carbon_tco2 += charged * source_intensity;
        }
        ++row;
      }
    };
    apply(sys.ac.storage, schedule->ess_dispatch, ac_state, 0);
    size_t dc_row = 0;
    size_t dc_state_offset = 0;
    auto apply_dc_group = [&](const auto& stores) {
      std::vector<std::vector<double>> rows;
      rows.reserve(schedule->dc_ess_dispatch.size() -
                   std::min(dc_row, schedule->dc_ess_dispatch.size()));
      for (size_t i = dc_row; i < schedule->dc_ess_dispatch.size(); ++i)
        rows.push_back(schedule->dc_ess_dispatch[i]);
      apply(stores, rows, dc_state, dc_state_offset);
      for (const auto& st : stores)
        if (st.in_service && st.cap_charging_strategy != "static") ++dc_row;
    };
    auto apply_dc_group_with_offset = [&](const auto& stores) {
      apply_dc_group(stores);
      for (const auto& st : stores)
        if (st.in_service && st.cap_charging_strategy != "static") ++dc_state_offset;
    };
    apply_dc_group_with_offset(sys.dc.storage);
    apply_dc_group_with_offset(sys.dc.dc_storage);
    ledger[static_cast<size_t>(t)] = out;
  }
  return ledger;
}

/// Dense carbon estimate: sum gen × emission_factor × step_hr over ALL hours
double compute_dense_carbon(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const AnnualProductionSimResult& ann_result) {
  if (ann_result.step_results.empty()) return 0.0;
  const auto ledger = build_carbon_ledger(sys, ts_data, ann_result);
  double total = 0.0;
  for (int t = 0; t < static_cast<int>(ann_result.step_results.size()); ++t) {
    const auto& c = ledger[static_cast<size_t>(t)];
    double value = c.authored_assets + c.external_grid + c.storage_inventory + c.dc_assets;
    if (value == 0.0 && !has_authored_emission_factor(sys))
      value = ann_result.step_results[static_cast<size_t>(t)].total_gen_mw * 0.5 *
          ann_result.step_duration_hr;
    total += value;
  }
  return total;
}

double estimate_annual_carbon(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const AnnualProductionSimResult& ann_result,
    const std::vector<StratumDef>& strata,
    const std::vector<int>& sampled_hours) {
  if (ann_result.step_results.empty()) return 0.0;
  const auto ledger = build_carbon_ledger(sys, ts_data, ann_result);

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
          const auto& c = ledger[static_cast<size_t>(sh)];
          double value = c.authored_assets + c.external_grid + c.storage_inventory + c.dc_assets;
          if (value == 0.0 && !has_authored_emission_factor(sys))
            value = ann_result.step_results[static_cast<size_t>(sh)].total_gen_mw * 0.5 *
                ann_result.step_duration_hr;
          stratum_sample_carbon += value;
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
  if (opts.num_years < 0) {
    throw std::invalid_argument("LifecycleSimOptions.num_years must be non-negative");
  }
  if (!std::isfinite(opts.discount_rate) || opts.discount_rate <= -1.0 ||
      !std::isfinite(opts.load_growth_rate) || opts.load_growth_rate <= -1.0 ||
      !std::isfinite(opts.pv_annual_derating) || opts.pv_annual_derating < 0.0 ||
      opts.pv_annual_derating > 1.0 ||
      !std::isfinite(opts.calendar_degradation_per_year) ||
      opts.calendar_degradation_per_year < 0.0 ||
      !std::isfinite(opts.cycle_degradation_per_cycle) ||
      opts.cycle_degradation_per_cycle < 0.0 ||
      !std::isfinite(opts.loss_proxy_fraction) || opts.loss_proxy_fraction < 0.0 ||
      opts.tier2_samples_per_stratum < 1 ||
      !std::isfinite(opts.confidence_level) || opts.confidence_level <= 0.0 ||
      opts.confidence_level >= 1.0) {
    throw std::invalid_argument(
        "LifecycleSimOptions contains a non-finite or out-of-range rate, "
        "sample count, confidence level, or loss proxy");
  }
  if (ts_data.num_steps < 0 ||
      (ts_data.num_steps > 0 &&
       (!std::isfinite(ts_data.step_duration_hr) || ts_data.step_duration_hr <= 0.0))) {
    throw std::invalid_argument("TimeSeriesData cadence must be positive and finite");
  }
  if (opts.step_duration_hr > 0.0 && ts_data.num_steps > 0 &&
      std::abs(opts.step_duration_hr - ts_data.step_duration_hr) > 1e-12) {
    throw std::invalid_argument(
        "LifecycleSimOptions.step_duration_hr disagrees with TimeSeriesData.step_duration_hr");
  }
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
  std::unordered_map<int, double> cumulative_cycles;
  std::unordered_map<int, double> age_since_replacement_years;
  for (const auto& st : work_sys.ac.storage) {
    cumulative_cycles[st.index] = std::max(0, st.current_cycles);
    age_since_replacement_years[st.index] = 0.0;
  }
  struct OrigDCStorage { int index; double e_rated_mwh; };
  std::vector<OrigDCStorage> orig_dc_storages;
  std::unordered_map<int, double> dc_cumulative_cycles;
  std::unordered_map<int, double> dc_age_since_replacement_years;
  for (const auto& st : work_sys.dc.storage) {
    orig_dc_storages.push_back({st.index, st.e_rated_mwh});
    dc_cumulative_cycles[st.index] = std::max(0, st.current_cycles);
    dc_age_since_replacement_years[st.index] = 0.0;
  }
  for (const auto& st : work_sys.dc.dc_storage) {
    orig_dc_storages.push_back({st.index, st.e_rated_mwh});
    dc_cumulative_cycles[st.index] = std::max(0, st.current_cycles);
    dc_age_since_replacement_years[st.index] = 0.0;
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
          const double age = age_since_replacement_years[st.index];
          const double cycles = cumulative_cycles[st.index];
          // Calendar and cycle degradation are local to the current battery
          // installation.  A replacement resets both clocks; project-year
          // indexing would otherwise continue degrading a new unit.
          st.soh_calendar = std::max(
              0.0, 1.0 - opts.calendar_degradation_per_year * age);
          st.soh_cycle = std::max(
              0.0, 1.0 - opts.cycle_degradation_per_cycle * cycles);
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
            cumulative_cycles[st.index] = 0.0;
            age_since_replacement_years[st.index] = 0.0;
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
          ss.cumulative_cycles = cumulative_cycles[st.index];
          ss.age_since_replacement_years = age_since_replacement_years[st.index];
          ss.replaced = std::any_of(
              yr.replacements.begin(), yr.replacements.end(),
              [&](const ReplacementEvent& re) {
                return re.storage_index == st.index;
              });
          yr.storage_states.push_back(ss);
          break;
        }
      }
    }

    // DC and expanded DC-storage assets follow the same calendar/cycle model
    // as AC storage. Their state is kept by stable component index and is
    // reported with is_dc=true so domain collisions cannot be mistaken for one
    // another in lifecycle attribution.
    auto update_dc_storage_group = [&](auto& group) {
      for (auto& st : group) {
        for (const auto& os : orig_dc_storages) {
          if (os.index != st.index) continue;
          const double age = dc_age_since_replacement_years[st.index];
          const double cycles = dc_cumulative_cycles[st.index];
          const double soh_calendar = std::max(0.0, 1.0 - opts.calendar_degradation_per_year * age);
          const double soh_cycle = std::max(0.0, 1.0 - opts.cycle_degradation_per_cycle * cycles);
          st.soh = std::min(soh_calendar, soh_cycle);
          const double eol_fraction = st.eol_percent > 1.0 ? st.eol_percent / 100.0 : st.eol_percent;
          if (st.soh <= std::clamp(eol_fraction, 0.0, 1.0)) {
            ReplacementEvent re;
            re.year = year; re.storage_index = st.index; re.storage_name = st.name;
            re.old_soh = st.soh;
            re.replacement_cost_usd = st.replacement_cost * os.e_rated_mwh * 1000.0;
            yr.replacements.push_back(re); result.all_replacements.push_back(re);
            result.total_replacements++; cumulative_replacement += re.replacement_cost_usd;
            st.soh = 1.0;
            st.current_cycles = 0; dc_cumulative_cycles[st.index] = 0.0;
            dc_age_since_replacement_years[st.index] = 0.0;
          }
          st.e_rated_mwh = os.e_rated_mwh * st.soh;
          st.e_mwh = st.e_rated_mwh * st.soc_init;
          StorageYearState ss;
          ss.storage_index = st.index; ss.name = st.name; ss.is_dc = true;
          ss.soh = st.soh; ss.soh_cycle = soh_cycle; ss.soh_calendar = soh_calendar;
          ss.effective_capacity_mwh = st.e_rated_mwh;
          ss.cumulative_cycles = dc_cumulative_cycles[st.index];
          ss.age_since_replacement_years = dc_age_since_replacement_years[st.index];
          ss.replaced = std::any_of(yr.replacements.begin(), yr.replacements.end(),
              [&](const ReplacementEvent& re) { return re.storage_index == st.index; });
          yr.storage_states.push_back(std::move(ss));
          break;
        }
      }
    };
    update_dc_storage_group(work_sys.dc.storage);
    update_dc_storage_group(work_sys.dc.dc_storage);

    // ─── Tier 1: Annual chronological dispatch ──────────────────────

    AnnualProductionSimOptions ann_opts;
    ann_opts.skip_replay = !opts.run_physical_replay;
    ann_opts.enforce_cyclic_soc = true;
    ann_opts.verbose = false;
    ann_opts.ts_pf_options.run_opf = opts.run_physical_replay;
    ann_opts.ts_pf_options.keep_system_snapshots = false;
    ann_opts.curtailment_penalty = 50.0;
    ann_opts.ens_penalty = 10000.0;

    auto ann_result = solve_annual_production_simulation(work_sys, ts_data, ann_opts);

    yr.annual_cost = ann_result.total_cost;
    yr.annual_gen_mwh = ann_result.total_gen_mwh;
    yr.annual_load_mwh = ann_result.total_load_mwh;
    yr.annual_renewable_mwh = ann_result.total_renewable_mwh;
    yr.annual_curtailment_mwh = ann_result.total_curtailment_mwh;
    yr.annual_ens_mwh = ann_result.total_ens_mwh;
    yr.annual_loss_mwh = ann_result.total_loss_mwh;
    yr.feasible = ann_result.feasible;
    yr.schedule_only = ann_result.schedule_only;
    yr.physical_replay_complete = ann_result.physical_replay_complete;

    // Update storage cycle counts from annual results
    for (const auto& sa : ann_result.storage_stats) {
      if (sa.is_dc) {
        auto it = dc_cumulative_cycles.find(sa.storage_index);
        if (it != dc_cumulative_cycles.end()) {
          it->second += sa.cycles;
          for (auto& ss : yr.storage_states)
            if (ss.is_dc && ss.storage_index == sa.storage_index) {
              ss.cycles_this_year = sa.cycles;
              ss.cumulative_cycles = it->second;
            }
        }
        continue;
      }
      for (auto& st : work_sys.ac.storage) {
        if (st.index == sa.storage_index) {
          cumulative_cycles[st.index] += sa.cycles;
          // Preserve the public integer field for legacy callers, while the
          // lifecycle state retains fractional equivalent full cycles.
          st.current_cycles = static_cast<int>(std::floor(cumulative_cycles[st.index]));
          // Update year state with cycles info
          for (auto& ss : yr.storage_states) {
            if (ss.storage_index == st.index) {
              ss.cycles_this_year = sa.cycles;
              ss.cumulative_cycles = cumulative_cycles[st.index];
              break;
            }
          }
          break;
        }
      }
    }
    for (const auto& st : work_sys.ac.storage) {
      const bool replaced_this_year = std::any_of(
          yr.replacements.begin(), yr.replacements.end(),
          [&](const ReplacementEvent& re) { return re.storage_index == st.index; });
      if (!replaced_this_year) age_since_replacement_years[st.index] += 1.0;
      for (auto& ss : yr.storage_states) {
        if (ss.storage_index == st.index) {
          ss.age_since_replacement_years = age_since_replacement_years[st.index];
          ss.cumulative_cycles = cumulative_cycles[st.index];
          break;
        }
      }
    }
    auto age_dc_group = [&](const auto& group) {
      for (const auto& st : group) {
        const bool replaced_this_year = std::any_of(yr.replacements.begin(), yr.replacements.end(),
            [&](const ReplacementEvent& re) { return re.storage_index == st.index; });
        if (!replaced_this_year) dc_age_since_replacement_years[st.index] += 1.0;
        for (auto& ss : yr.storage_states)
          if (ss.is_dc && ss.storage_index == st.index) {
            ss.age_since_replacement_years = dc_age_since_replacement_years[st.index];
            ss.cumulative_cycles = dc_cumulative_cycles[st.index];
          }
      }
    };
    age_dc_group(work_sys.dc.storage);
    age_dc_group(work_sys.dc.dc_storage);

    // ─── Tier 2: Stratified sampling + carbon estimation ────────────
    // Tier 1 provides dispatch for ALL hours, so the primary carbon
    // estimate uses the dense (all-hours) summation. The stratified
    // sampling selects hours for PF validation (Tier 2 corrections).
    // The sampling error bound covers only the PF correction residual.

    auto strata = classify_strata(ann_result.step_results);
    auto allocation = compute_sample_allocation(
        strata, ann_result.step_results, opts.tier2_samples_per_stratum,
        ann_result.step_duration_hr, max_ef, opts.loss_proxy_fraction);
    auto sampled_hours = select_sample_hours(strata, allocation);

    const auto carbon_ledger = build_carbon_ledger(work_sys, ts_data, ann_result);
    double dense_carbon = compute_dense_carbon(work_sys, ts_data, ann_result);
    double sampled_carbon = estimate_annual_carbon(
        work_sys, ts_data, ann_result, strata, sampled_hours);

    // Execute an independent one-step UC-schedule replay for every selected
    // sample. This is the measured PF/OPF correction; the previous estimator
    // only rescaled a schedule proxy and therefore was not a physical check.
    double corrected_carbon = dense_carbon;
    int sampled_converged = 0;
    double correction_total = 0.0;
    std::vector<double> correction_by_stratum(strata.size(), 0.0);
    std::vector<int> correction_count_by_stratum(strata.size(), 0);
    if (opts.run_sampled_pf_correction && !sampled_hours.empty()) {
      for (int sh : sampled_hours) {
        int local = -1;
        const UCSchedule* source = schedule_at_step(ann_result, sh, local);
        if (!source) continue;
        const UCSchedule one = slice_schedule_for_step(*source, local);
        TimeSeriesData one_ts;
        one_ts.num_steps = 1;
        one_ts.step_duration_hr = ts_data.step_duration_hr;
        for (const auto& p : ts_data.profiles) {
          TimeSeriesProfile q = p;
          q.values = {sh >= 0 && sh < static_cast<int>(p.values.size())
                          ? p.values[static_cast<size_t>(sh)] : 1.0};
          one_ts.profiles.push_back(std::move(q));
        }
        TimeSeriesPFOptions pf_opts = ann_opts.ts_pf_options;
        pf_opts.skip_uc = false;
        pf_opts.run_opf = true;
        pf_opts.parallel_daily = false;
        pf_opts.precomputed_uc_schedule = &one;
        auto physical = solve_time_series_pf(work_sys, one_ts, pf_opts);
        if (physical.num_steps != 1 || physical.opf_results.empty() ||
            !physical.opf_results.front().converged) continue;
        ++sampled_converged;
        const auto& opf = physical.opf_results.front();
        double physical_carbon = 0.0;
        int gp = 0;
        for (const auto& g : work_sys.ac.generators) {
          if (!g.in_service) continue;
          if (g.emission_factor_tco2_mwh > 0.0 && gp < static_cast<int>(opf.pg_mw.size()))
            physical_carbon += std::max(0.0, opf.pg_mw[static_cast<size_t>(gp)]) *
                g.emission_factor_tco2_mwh * ts_data.step_duration_hr;
          ++gp;
        }
        size_t xi = 0;
        for (const auto& eg : work_sys.ac.external_grids) {
          if (!eg.in_service) continue;
          double factor = eg.emission_factor_tco2_mwh;
          if (eg.emission_factor_profile_id >= 0)
            factor = profile_value_at(ts_data, eg.emission_factor_profile_id,
                                      sh, factor);
          if (xi < opf.external_grid_p_mw.size() && factor > 0.0 &&
              std::isfinite(factor)) {
            physical_carbon += std::max(0.0, opf.external_grid_p_mw[xi]) *
                factor * ts_data.step_duration_hr;
          }
          ++xi;
        }
        const auto& proxy = carbon_ledger[static_cast<size_t>(sh)];
        const double correction = physical_carbon -
            (proxy.authored_assets + proxy.external_grid + proxy.storage_inventory + proxy.dc_assets);
        correction_total += correction;
        for (size_t m = 0; m < strata.size(); ++m) {
          if (std::find(strata[m].hour_indices.begin(), strata[m].hour_indices.end(), sh) !=
              strata[m].hour_indices.end()) {
            correction_by_stratum[m] += correction;
            ++correction_count_by_stratum[m];
            break;
          }
        }
      }
      for (size_t m = 0; m < strata.size(); ++m) {
        if (correction_count_by_stratum[m] > 0) {
          corrected_carbon += correction_by_stratum[m] /
              static_cast<double>(correction_count_by_stratum[m]) *
              static_cast<double>(strata[m].hour_indices.size());
        }
      }
    }
    yr.sampled_pf_requested = static_cast<int>(sampled_hours.size());
    yr.sampled_pf_converged = sampled_converged;
    yr.sampled_pf_correction_complete =
        !opts.run_sampled_pf_correction || sampled_converged == yr.sampled_pf_requested;
    yr.sampled_pf_correction_tco2 = correction_total;

    // Use dense estimate as primary (Tier 1 gives us all hours)
    yr.annual_carbon_tco2 = corrected_carbon;
    for (int t = 0; t < static_cast<int>(ann_result.step_results.size()); ++t) {
      const auto& c = carbon_ledger[static_cast<size_t>(t)];
      yr.carbon_external_grid_tco2 += c.external_grid;
      yr.carbon_storage_inventory_tco2 += c.storage_inventory;
      yr.carbon_dc_assets_tco2 += c.dc_assets;
    }
    yr.carbon_expanded_assets_tco2 = yr.carbon_dc_assets_tco2;
    yr.avg_carbon_intensity = (yr.annual_load_mwh > 0)
        ? yr.annual_carbon_tco2 / yr.annual_load_mwh : 0.0;

    // ─── Theoretical error bounds ───────────────────────────────────

    yr.bounds.dispatch = compute_dispatch_bound(
        ann_result.step_results, ann_result.step_duration_hr,
        opts.loss_proxy_fraction, max_ef);

    yr.bounds.sampling = compute_sampling_bound(
        strata, ann_result.step_results, allocation,
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
    cv.bound_passed = std::isfinite(cv.sampling_gap_tco2) &&
        cv.sampling_gap_tco2 <= yr.bounds.total_bound_tco2 + 1e-12;
    bool any_external_factor = false;
    for (const auto& eg : work_sys.ac.external_grids)
      any_external_factor = any_external_factor || eg.emission_factor_tco2_mwh > 0.0 || eg.emission_factor_profile_id >= 0;
    yr.carbon_factor_source = has_authored_emission_factor(work_sys) || any_external_factor
        ? "asset_factors+external_grid_profile"
        : "explicit_0.5_tco2_per_mwh_fallback";
    yr.carbon_known = carbon_schedule_complete(ann_result) &&
        (!opts.run_physical_replay || ann_result.physical_replay_complete) &&
        yr.sampled_pf_correction_complete;
    yr.feasible = yr.feasible &&
        (!opts.run_physical_replay || yr.physical_replay_complete) &&
        yr.sampled_pf_correction_complete;
    if (!yr.carbon_known) {
      yr.carbon_factor_source = "unknown_dispatch_schedule";
    }

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
