#include "hacdcpf/time_series/annual_production_sim.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <future>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "hacdcpf/util/parallel_execution.hpp"
#include "hacdcpf/util/thread_pool.hpp"

namespace hacdcpf::analysis {

// ═══════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════

static double checked_step_duration_hr(const TimeSeriesData& ts_data) {
  if (ts_data.num_steps < 0) {
    throw std::invalid_argument("TimeSeriesData.num_steps must be non-negative");
  }
  if (ts_data.num_steps > 0 &&
      (!std::isfinite(ts_data.step_duration_hr) ||
       ts_data.step_duration_hr <= 0.0)) {
    throw std::invalid_argument(
        "TimeSeriesData.step_duration_hr must be positive and finite");
  }
  return ts_data.step_duration_hr;
}

static int steps_for_duration(double duration_hr, double step_hr) {
  if (!std::isfinite(duration_hr) || duration_hr <= 0.0) {
    throw std::invalid_argument("duration window must be positive and finite");
  }
  return std::max(1, static_cast<int>(std::round(duration_hr / step_hr)));
}

/// Build block boundaries for the year (monthly or weekly).
static std::vector<std::pair<int, int>> build_block_ranges(
    int total_steps, double step_hr, AnnualBlockType btype) {
  std::vector<std::pair<int, int>> blocks;

  if (btype == AnnualBlockType::Weekly) {
    // 52 weekly blocks (last block absorbs remainder)
    const int steps_per_week = steps_for_duration(168.0, step_hr);
    for (int w = 0; w < 52; ++w) {
      int s = w * steps_per_week;
      int e = (w == 51) ? total_steps
                        : std::min((w + 1) * steps_per_week, total_steps);
      if (s >= total_steps) break;
      blocks.emplace_back(s, e);
    }
  } else {
    // Monthly blocks: approximate days-per-month
    static constexpr int days_per_month[] = {31, 28, 31, 30, 31, 30,
                                              31, 31, 30, 31, 30, 31};
    const int steps_per_day = steps_for_duration(24.0, step_hr);
    int cursor = 0;
    for (int m = 0; m < 12; ++m) {
      int s = cursor;
      int e = cursor + days_per_month[m] * steps_per_day;
      if (m == 11) e = total_steps;  // absorb rounding into last month
      e = std::min(e, total_steps);
      if (s >= total_steps) break;
      blocks.emplace_back(s, e);
      cursor = e;
    }
  }
  return blocks;
}

/// Create a TimeSeriesData slice for a sub-horizon [t_start, t_end).
static TimeSeriesData slice_ts_data(const TimeSeriesData& full,
                                     int t_start, int t_end) {
  TimeSeriesData sub;
  sub.num_steps = t_end - t_start;
  sub.step_duration_hr = full.step_duration_hr;

  sub.profiles.reserve(full.profiles.size());
  for (const auto& p : full.profiles) {
    TimeSeriesProfile sp;
    sp.id = p.id;
    sp.name = p.name;
    const int n = static_cast<int>(p.values.size());
    sp.values.reserve(static_cast<size_t>(sub.num_steps));
    for (int t = t_start; t < t_end; ++t) {
      sp.values.push_back(t < n ? p.values[static_cast<size_t>(t)] : 1.0);
    }
    sub.profiles.push_back(std::move(sp));
  }
  return sub;
}

static constexpr double kDefaultReportedEnergyCostPerMWh = 20.0;
static constexpr double kCostEpsilon = 1e-9;

static double profile_scale(
    const std::unordered_map<int, const TimeSeriesProfile*>& pmap,
    int profile_id,
    int t) {
  auto it = pmap.find(profile_id);
  if (it == pmap.end()) return 1.0;
  if (t < 0 || t >= static_cast<int>(it->second->values.size())) return 1.0;
  return it->second->values[static_cast<size_t>(t)];
}

static bool scheduled_value(const std::vector<std::vector<double>>& rows,
                            size_t row,
                            int t,
                            double& value) {
  if (row >= rows.size()) return false;
  if (t < 0 || t >= static_cast<int>(rows[row].size())) return false;
  value = rows[row][static_cast<size_t>(t)];
  return true;
}

static bool is_renewable_sgen_type(SgenType type) {
  return type == SgenType::PV || type == SgenType::Wind;
}

static bool looks_renewable_type(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char ch) {
                   return static_cast<char>(std::tolower(ch));
                 });
  return text.find("pv") != std::string::npos ||
         text.find("solar") != std::string::npos ||
         text.find("wind") != std::string::npos;
}

static double static_generator_report_price(const StaticGenerator& sg) {
  if (std::abs(sg.cost_c1) > kCostEpsilon) return sg.cost_c1;
  return is_renewable_sgen_type(sg.sgen_type)
             ? 0.0
             : kDefaultReportedEnergyCostPerMWh;
}

static double dc_static_generator_report_price(const StaticGeneratorDC& sg) {
  if (std::abs(sg.cost_c1) > kCostEpsilon) return sg.cost_c1;
  return looks_renewable_type(sg.type) ? 0.0
                                       : kDefaultReportedEnergyCostPerMWh;
}

struct ReportedProfileDispatch {
  double dispatchable_generation_mw{0.0};
  double renewable_generation_mw{0.0};
  double dc_load_mw{0.0};
  double cost_rate_per_hr{0.0};
};

static double storage_bid_cost_rate(const HybridPowerSystem& sys,
                                    const UCSchedule& schedule,
                                    int t) {
  double cost = 0.0;
  size_t row = 0;
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service) continue;
    double p = 0.0;
    if (scheduled_value(schedule.ess_dispatch, row++, t, p)) {
      cost += p >= 0.0 ? p * std::max(0.0, st.discharge_bid_price)
                       : -p * std::max(0.0, st.charge_bid_price);
    }
  }
  row = 0;
  auto add_dc = [&](const auto& st) {
    if (!st.in_service) return;
    double p = 0.0;
    if (scheduled_value(schedule.dc_ess_dispatch, row++, t, p)) {
      cost += p >= 0.0 ? p * std::max(0.0, st.discharge_bid_price)
                       : -p * std::max(0.0, st.charge_bid_price);
    }
  };
  for (const auto& st : sys.dc.storage) add_dc(st);
  for (const auto& st : sys.dc.dc_storage) add_dc(st);
  return cost;
}

struct ScheduledStoragePower {
  double net_mw{0.0};
  double discharge_mw{0.0};
  double charge_mw{0.0};
};

static ScheduledStoragePower scheduled_storage_power(
    const UCSchedule& schedule, int t) {
  ScheduledStoragePower total;
  const auto add_rows = [&](const std::vector<std::vector<double>>& rows) {
    for (const auto& row : rows) {
      if (t >= 0 && t < static_cast<int>(row.size())) {
        const double p = row[static_cast<size_t>(t)];
        total.net_mw += p;
        total.discharge_mw += std::max(p, 0.0);
        total.charge_mw += std::max(-p, 0.0);
      }
    }
  };
  add_rows(schedule.ess_dispatch);
  add_rows(schedule.dc_ess_dispatch);
  return total;
}

static void finalize_step_power_accounting(
    AnnualStepResult& step, double external_grid_net_mw) {
  step.external_grid_net_mw = external_grid_net_mw;
  step.total_supply_mw =
      step.total_gen_mw + step.total_renewable_mw +
      step.storage_discharge_mw;
  step.total_demand_mw =
      step.total_load_mw + std::max(0.0, step.total_loss_mw) +
      step.storage_charge_mw +
      std::max(0.0, -external_grid_net_mw);
  step.power_balance_error_mw =
      step.total_supply_mw - step.total_demand_mw;
}

static ReportedProfileDispatch reported_profile_dispatch(
    const HybridPowerSystem& sys,
    const UCSchedule& schedule,
    const std::unordered_map<int, const TimeSeriesProfile*>& pmap,
    int schedule_t,
    int profile_t) {
  ReportedProfileDispatch out;

  for (size_t k = 0; k < sys.ac.static_generators.size(); ++k) {
    const auto& sg = sys.ac.static_generators[k];
    if (!sg.in_service) continue;
    double p = 0.0;
    if (!scheduled_value(schedule.ac_sgen_dispatch, k, schedule_t, p)) {
      p = sg.p_mw * sg.scaling;
    }
    p = std::max(0.0, p);
    if (p <= 0.0) continue;
    if (is_renewable_sgen_type(sg.sgen_type)) {
      out.renewable_generation_mw += p;
    } else {
      out.dispatchable_generation_mw += p;
    }
    out.cost_rate_per_hr += p * static_generator_report_price(sg);
  }

  for (size_t k = 0; k < sys.ac.pv_systems.size(); ++k) {
    const auto& pv = sys.ac.pv_systems[k];
    if (!pv.in_service) continue;
    double p = 0.0;
    if (!scheduled_value(schedule.ac_pv_dispatch, k, schedule_t, p)) {
      p = pv.p_mw * profile_scale(pmap, pv.profile_id, profile_t);
    }
    p = std::max(0.0, p);
    out.renewable_generation_mw += p;
    out.cost_rate_per_hr += p * pv.cost_c1;
  }

  size_t renewable_row = 0;
  for (const auto& ren : sys.ac.renewable_gens) {
    if (!ren.in_service) continue;
    double p = 0.0;
    if (!scheduled_value(schedule.renewable_dispatch, renewable_row++, schedule_t, p)) {
      const double base = ren.p_rated_mw > 0.0 ? ren.p_rated_mw : ren.p_mw;
      p = base * profile_scale(pmap, ren.profile_id, profile_t);
    }
    out.cost_rate_per_hr += std::max(0.0, p) * ren.cost_c1;
  }

  for (size_t k = 0; k < sys.dc.pv_arrays.size(); ++k) {
    const auto& pv = sys.dc.pv_arrays[k];
    if (!pv.in_service) continue;
    double p = 0.0;
    if (!scheduled_value(schedule.dc_pv_dispatch, k, schedule_t, p)) {
      p = pv.p_set_mw * profile_scale(pmap, pv.profile_id, profile_t);
    }
    p = std::max(0.0, p);
    out.renewable_generation_mw += p;
    out.cost_rate_per_hr += p * pv.cost_c1;
  }

  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    const double p = std::max(0.0, sg.p_mw * sg.scaling);
    if (p <= 0.0) continue;
    if (is_renewable_sgen_type(sg.sgen_type)) {
      out.renewable_generation_mw += p;
    } else {
      out.dispatchable_generation_mw += p;
    }
    out.cost_rate_per_hr += p * static_generator_report_price(sg);
  }

  for (size_t k = 0; k < sys.dc.dc_static_generators.size(); ++k) {
    const auto& sg = sys.dc.dc_static_generators[k];
    if (!sg.in_service) continue;
    double p = 0.0;
    if (!scheduled_value(schedule.dc_sgen_dispatch, k, schedule_t, p)) {
      p = sg.p_set_mw * sg.scaling *
          profile_scale(pmap, sg.profile_id, profile_t);
    }
    p = std::max(0.0, p);
    if (p <= 0.0) continue;
    if (looks_renewable_type(sg.type)) {
      out.renewable_generation_mw += p;
    } else {
      out.dispatchable_generation_mw += p;
    }
    out.cost_rate_per_hr += p * dc_static_generator_report_price(sg);
  }

  for (size_t k = 0; k < sys.dc.loads.size(); ++k) {
    const auto& ld = sys.dc.loads[k];
    if (!ld.in_service) continue;
    double p = 0.0;
    if (!scheduled_value(schedule.dc_load_demand, k, schedule_t, p)) {
      p = ld.p_mw * ld.scaling *
          profile_scale(pmap, ld.profile_id, profile_t);
    }
    out.dc_load_mw += std::max(0.0, p);
  }

  return out;
}

static double reported_ac_renewable_mw(
    const HybridPowerSystem& sys,
    const UCSchedule& schedule,
    const std::unordered_map<int, const TimeSeriesProfile*>& pmap,
    int schedule_t,
    int profile_t) {
  double total = 0.0;
  if (!schedule.renewable_dispatch.empty()) {
    for (const auto& row : schedule.renewable_dispatch) {
      if (schedule_t >= 0 && schedule_t < static_cast<int>(row.size())) {
        total += row[static_cast<size_t>(schedule_t)];
      }
    }
    return total;
  }

  for (const auto& ren : sys.ac.renewable_gens) {
    if (!ren.in_service) continue;
    const double base = ren.p_rated_mw > 0.0 ? ren.p_rated_mw : ren.p_mw;
    total += std::max(0.0, base * profile_scale(pmap, ren.profile_id, profile_t));
  }
  return total;
}

static double reported_external_grid_price(
    const HybridPowerSystem& sys,
    const std::unordered_map<int, const TimeSeriesProfile*>& pmap,
    int profile_t) {
  double sum = 0.0;
  int count = 0;
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    double price = eg.cost_c1;
    if (eg.price_profile_id >= 0) {
      const double prof = profile_scale(pmap, eg.price_profile_id, profile_t);
      price = std::abs(eg.cost_c1) > kCostEpsilon ? eg.cost_c1 * prof : prof;
    }
    if (!std::isfinite(price) || price <= kCostEpsilon) {
      price = kDefaultReportedEnergyCostPerMWh;
    }
    sum += price;
    ++count;
  }
  return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

struct ReportedExternalGridDispatch {
  bool scheduled{false};
  double net_import_mw{0.0};
  double cost_rate_per_hr{0.0};
};

static ReportedExternalGridDispatch reported_external_grid_dispatch(
    const HybridPowerSystem& sys,
    const UCSchedule& schedule,
    const std::unordered_map<int, const TimeSeriesProfile*>& pmap,
    int schedule_t,
    int profile_t) {
  ReportedExternalGridDispatch out;
  for (size_t k = 0; k < sys.ac.external_grids.size(); ++k) {
    const auto& grid = sys.ac.external_grids[k];
    if (!grid.in_service) continue;
    double p = 0.0;
    if (!scheduled_value(schedule.external_grid_dispatch, k, schedule_t, p)) continue;
    out.scheduled = true;
    out.net_import_mw += p;
    out.cost_rate_per_hr +=
        p * grid.cost_c1 * profile_scale(pmap, grid.price_profile_id, profile_t);
  }
  return out;
}

/// Conventional generator operating cost rate ($/h) from dispatch in MW.
/// This is the reporting basis used by annual/monthly cost summaries regardless
/// of whether the optimizer itself minimized cost, carbon, curtailment, or loss.
static double generator_operating_cost_rate(
    const HybridPowerSystem& sys,
    const std::vector<double>& dispatch_mw,
    const std::vector<int>* commitment = nullptr,
    int t = 0) {
  double cost = 0.0;
  int gpos = 0;
  for (const auto& gen : sys.ac.generators) {
    if (!gen.in_service) continue;
    if (gpos < static_cast<int>(dispatch_mw.size())) {
      const double p = dispatch_mw[static_cast<size_t>(gpos)];
      double u = std::abs(p) > 1e-8 ? 1.0 : 0.0;
      if (commitment && gpos < static_cast<int>(commitment->size())) {
        u = (*commitment)[static_cast<size_t>(gpos)] ? 1.0 : 0.0;
      }
      cost += std::max(0.0, gen.cost_c0) * u + gen.cost_c1 * p +
              gen.cost_c2 * p * p;
    }
    ++gpos;
  }
  (void)t;
  return cost;
}

static double generator_operating_cost_rate_from_uc(
    const HybridPowerSystem& sys,
    const UCSchedule& uc,
    int t) {
  std::vector<double> dispatch;
  dispatch.reserve(uc.gen_dispatch.size());
  for (const auto& row : uc.gen_dispatch) {
    dispatch.push_back(
        (t >= 0 && t < static_cast<int>(row.size()))
            ? row[static_cast<size_t>(t)]
            : 0.0);
  }
  std::vector<int> commitment;
  commitment.reserve(uc.gen_commit.size());
  for (const auto& row : uc.gen_commit) {
    commitment.push_back(
        (t >= 0 && t < static_cast<int>(row.size()))
            ? row[static_cast<size_t>(t)]
            : 0);
  }
  return generator_operating_cost_rate(
      sys, dispatch, commitment.empty() ? nullptr : &commitment, t);
}

static double generator_operating_cost_rate_from_opf(
    const HybridPowerSystem& sys,
    const std::vector<double>& pg_mw) {
  std::vector<double> active_dispatch;
  active_dispatch.reserve(pg_mw.size());
  const bool original_indexed = pg_mw.size() == sys.ac.generators.size();
  int active_pos = 0;
  for (size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
    const auto& gen = sys.ac.generators[gi];
    if (!gen.in_service) continue;
    const size_t pos = original_indexed ? gi : static_cast<size_t>(active_pos);
    active_dispatch.push_back(pos < pg_mw.size() ? pg_mw[pos] : 0.0);
    ++active_pos;
  }
  return generator_operating_cost_rate(sys, active_dispatch);
}

/// Extract per-step metrics from a TimeSeriesPFResult into
/// the corresponding range of AnnualStepResult entries.
static void fill_step_results(std::vector<AnnualStepResult>& steps,
                               int global_offset,
                               const TimeSeriesPFResult& sub_result,
                               const HybridPowerSystem& sys,
                               const TimeSeriesData& sub_ts,
                               int sub_T) {
  // Build profile map for load calculations
  std::unordered_map<int, const TimeSeriesProfile*> pmap;
  for (const auto& p : sub_ts.profiles) pmap[p.id] = &p;
  auto get_scale = [&](int pid, int t) -> double {
    auto it = pmap.find(pid);
    if (it == pmap.end()) return 1.0;
    if (t < 0 || t >= static_cast<int>(it->second->values.size())) return 1.0;
    return it->second->values[static_cast<size_t>(t)];
  };

  for (int t = 0; t < sub_T; ++t) {
    const int g_idx = global_offset + t;
    if (g_idx >= static_cast<int>(steps.size())) break;
    auto& sr = steps[static_cast<size_t>(g_idx)];
    double base_cost_rate = 0.0;

    // PF convergence
    if (t < static_cast<int>(sub_result.pf_results.size())) {
      sr.pf_converged = sub_result.pf_results[static_cast<size_t>(t)].converged;
    }
    // OPF convergence
    if (t < static_cast<int>(sub_result.opf_results.size())) {
      const auto& opf = sub_result.opf_results[static_cast<size_t>(t)];
      sr.opf_converged = opf.converged;
      base_cost_rate =
          (opf.converged && !opf.pg_mw.empty())
              ? generator_operating_cost_rate_from_opf(sys, opf.pg_mw)
              : generator_operating_cost_rate_from_uc(
                    sys, sub_result.uc_schedule, t);
    } else {
      // No OPF for this step (e.g. dynamic SCED / UC-only): derive the step cost
      // from the UC generator dispatch using each unit's cost curve so the
      // production-cost objective is still populated.
      base_cost_rate = generator_operating_cost_rate_from_uc(
          sys, sub_result.uc_schedule, t);
      sr.opf_converged = sub_result.uc_schedule.feasible;
    }

    const ReportedProfileDispatch reported =
        reported_profile_dispatch(sys, sub_result.uc_schedule, pmap, t, t);
    ReportedExternalGridDispatch grid = reported_external_grid_dispatch(
        sys, sub_result.uc_schedule, pmap, t, t);
    if (t < static_cast<int>(sub_result.opf_results.size()) &&
        sub_result.opf_results[static_cast<size_t>(t)].converged) {
      const auto& opf = sub_result.opf_results[static_cast<size_t>(t)];
      grid = {};
      grid.scheduled = !opf.external_grid_p_mw.empty();
      for (size_t k = 0; k < sys.ac.external_grids.size() &&
                         k < opf.external_grid_p_mw.size(); ++k) {
        const auto& external = sys.ac.external_grids[k];
        if (!external.in_service) continue;
        const double p = opf.external_grid_p_mw[k];
        grid.net_import_mw += p;
        grid.cost_rate_per_hr +=
            p * external.cost_c1 *
            profile_scale(pmap, external.price_profile_id, t);
      }
    }

    // Aggregate generation from UC schedule
    double gen = 0.0;
    for (size_t g = 0; g < sub_result.uc_schedule.gen_dispatch.size(); ++g) {
      const auto& disp = sub_result.uc_schedule.gen_dispatch[g];
      if (t < static_cast<int>(disp.size())) gen += disp[static_cast<size_t>(t)];
    }
    sr.total_gen_mw = gen + reported.dispatchable_generation_mw +
                      std::max(0.0, grid.net_import_mw);

    // Aggregate load from profiles (ac.loads or bus pd_mw fallback)
    double load = 0.0;
    if (!sys.ac.loads.empty()) {
      for (const auto& ld : sys.ac.loads) {
        if (!ld.in_service) continue;
        double scale = get_scale(ld.profile_id, t);
        load += ld.p_mw * scale * ld.scaling;
      }
    } else {
      for (const auto& bus : sys.ac.buses) {
        if (bus.pd_mw > 0.0) load += bus.pd_mw * get_scale(0, t);
      }
    }
    sr.total_load_mw = load + reported.dc_load_mw;

    // Renewable dispatch
    double ren = reported_ac_renewable_mw(sys, sub_result.uc_schedule, pmap, t, t);
    sr.total_renewable_mw = ren + reported.renewable_generation_mw;

    // ESS net dispatch
    const auto ess_power = scheduled_storage_power(sub_result.uc_schedule, t);
    const double ess = ess_power.net_mw;
    sr.total_ess_mw = ess;
    sr.storage_discharge_mw = ess_power.discharge_mw;
    sr.storage_charge_mw = ess_power.charge_mw;

    // Loss from PF result
    if (t < static_cast<int>(sub_result.pf_results.size()) &&
        sub_result.pf_results[static_cast<size_t>(t)].converged) {
      const auto& pfr = sub_result.pf_results[static_cast<size_t>(t)];
      double loss = 0.0;
      for (const auto& bf : pfr.branch_flows)
        loss += bf.pf_mw + bf.pt_mw;  // net power = loss
      for (const auto& vt : pfr.vsc_transfers)
        loss += vt.loss_mw;
      for (const auto& dt_ : pfr.dcdc_transfers)
        loss += dt_.loss_mw;
      sr.total_loss_mw = loss;

      // Energy reporting must use the exchange realized by the converged PF,
      // not the OPF/UC scheduled grid value paired with PF losses. The latter
      // mixes two operating points and creates a visible annual balance gap.
      const double non_grid_supply_mw =
          gen + ren + reported.dispatchable_generation_mw +
          reported.renewable_generation_mw + std::max(0.0, ess);
      const double non_grid_demand_mw =
          sr.total_load_mw + std::max(0.0, sr.total_loss_mw) +
          std::max(0.0, -ess);
      grid.net_import_mw = non_grid_demand_mw - non_grid_supply_mw;
      grid.scheduled = true;
      sr.total_gen_mw = gen + reported.dispatchable_generation_mw +
                        std::max(0.0, grid.net_import_mw);
    }

    const double supply_mw =
        gen + ren + reported.dispatchable_generation_mw +
        reported.renewable_generation_mw + std::max(0.0, ess) +
        std::max(0.0, grid.net_import_mw);
    const double need_mw =
        sr.total_load_mw + std::max(0.0, sr.total_loss_mw) +
        std::max(0.0, -ess) + std::max(0.0, -grid.net_import_mw);
    finalize_step_power_accounting(sr, grid.net_import_mw);
    const double import_mw = grid.scheduled ? 0.0 : std::max(0.0, need_mw - supply_mw);
    sr.opf_cost =
        base_cost_rate + reported.cost_rate_per_hr +
        storage_bid_cost_rate(sys, sub_result.uc_schedule, t) +
        grid.cost_rate_per_hr +
        import_mw * reported_external_grid_price(sys, pmap, t);
  }
}

/// Aggregate step results into a block summary.
static BlockSummary aggregate_block(int block_id, int start, int end,
                                     const std::vector<AnnualStepResult>& steps,
                                     double dt) {
  BlockSummary bs;
  bs.block_id = block_id;
  bs.start_step = start;
  bs.end_step = end;
  bs.num_steps = end - start;
  for (int t = start; t < end && t < static_cast<int>(steps.size()); ++t) {
    const auto& s = steps[static_cast<size_t>(t)];
    bs.total_gen_mwh += s.total_gen_mw * dt;
    bs.total_load_mwh += s.total_load_mw * dt;
    bs.total_renewable_mwh += s.total_renewable_mw * dt;
    bs.total_curtailment_mwh += s.total_curtailment_mw * dt;
    bs.storage_discharge_mwh += s.storage_discharge_mw * dt;
    bs.storage_charge_mwh += s.storage_charge_mw * dt;
    bs.external_grid_import_mwh += std::max(0.0, s.external_grid_net_mw) * dt;
    bs.external_grid_export_mwh += std::max(0.0, -s.external_grid_net_mw) * dt;
    bs.total_supply_mwh += s.total_supply_mw * dt;
    bs.total_demand_mwh += s.total_demand_mw * dt;
    bs.power_balance_error_mwh += s.power_balance_error_mw * dt;
    bs.total_loss_mwh += s.total_loss_mw * dt;
    bs.total_ens_mwh += s.load_shed_mw * dt;
    bs.total_cost += s.opf_cost * dt;
    if (s.pf_converged) ++bs.num_pf_converged;
    if (s.opf_converged) ++bs.num_opf_converged;
  }
  return bs;
}

/// Compute generator annual statistics from step results and UC schedules.
static std::vector<GenAnnualStats> compute_gen_stats(
    const HybridPowerSystem& sys,
    const std::vector<WeeklySchedule>& weekly,
    double dt) {
  const auto& gens = sys.ac.generators;
  std::vector<GenAnnualStats> stats(gens.size());
  for (size_t g = 0; g < gens.size(); ++g) {
    stats[g].name = gens[g].name;
    stats[g].gen_index = static_cast<int>(g);
  }

  // Map in-service generator indices to UC vector positions
  std::vector<int> gen_uc_pos(gens.size(), -1);
  {
    int pos = 0;
    for (size_t g = 0; g < gens.size(); ++g) {
      if (!gens[g].in_service) continue;
      gen_uc_pos[g] = pos++;
    }
  }

  for (const auto& ws : weekly) {
    const auto& uc = ws.uc;
    for (size_t g = 0; g < gens.size(); ++g) {
      int pos = gen_uc_pos[g];
      if (pos < 0) continue;
      if (static_cast<size_t>(pos) >= uc.gen_dispatch.size()) continue;

      const auto& disp = uc.gen_dispatch[static_cast<size_t>(pos)];
      const auto& commit =
          (static_cast<size_t>(pos) < uc.gen_commit.size())
              ? uc.gen_commit[static_cast<size_t>(pos)]
              : std::vector<int>{};

      int prev_commit = -1;
      for (int t = 0; t < static_cast<int>(disp.size()) && t < ws.num_steps;
           ++t) {
        stats[g].total_energy_mwh += disp[static_cast<size_t>(t)] * dt;
        int c = (t < static_cast<int>(commit.size()))
                    ? commit[static_cast<size_t>(t)]
                    : 1;
        if (c) stats[g].total_hours_online += dt;
        if (prev_commit == 0 && c == 1) ++stats[g].total_startups;
        if (prev_commit == 1 && c == 0) ++stats[g].total_shutdowns;
        prev_commit = c;
      }
    }
  }

  for (size_t g = 0; g < gens.size(); ++g) {
    double pmax = gens[g].pmax_mw;
    double total_hours =
        static_cast<double>(weekly.empty() ? 0 : weekly.back().start_step +
                                                      weekly.back().num_steps) *
        dt;
    if (pmax > 0.0 && total_hours > 0.0)
      stats[g].capacity_factor = stats[g].total_energy_mwh / (pmax * total_hours);
  }
  return stats;
}

/// Compute storage annual statistics.
static std::vector<StorageAnnualStats> compute_storage_stats(
    const HybridPowerSystem& sys,
    const std::vector<WeeklySchedule>& weekly,
    double dt) {
  std::vector<StorageAnnualStats> stats;
  stats.reserve(sys.ac.storage.size() + sys.dc.storage.size() +
                sys.dc.dc_storage.size());

  const auto append_group = [&](const auto& storage, bool is_dc,
                                int& active_pos) {
    std::vector<int> schedule_pos(storage.size(), -1);
    for (size_t s = 0; s < storage.size(); ++s) {
      if (storage[s].in_service) schedule_pos[s] = active_pos++;
    }

    for (size_t s = 0; s < storage.size(); ++s) {
      StorageAnnualStats stat;
      stat.name = storage[s].name;
      stat.storage_index = storage[s].index;
      stat.is_dc = is_dc;
      const int pos = schedule_pos[s];
      if (pos >= 0) {
        for (const auto& ws : weekly) {
          const auto& rows = is_dc ? ws.uc.dc_ess_dispatch
                                   : ws.uc.ess_dispatch;
          if (static_cast<size_t>(pos) >= rows.size()) continue;
          const auto& dispatch = rows[static_cast<size_t>(pos)];
          for (int t = 0; t < static_cast<int>(dispatch.size()) &&
                          t < ws.num_steps; ++t) {
            const double p = dispatch[static_cast<size_t>(t)];
            stat.total_discharge_mwh += std::max(p, 0.0) * dt;
            stat.total_charge_mwh += std::max(-p, 0.0) * dt;
          }
        }
      }
      if (storage[s].e_rated_mwh > 0.0) {
        stat.cycles = stat.total_discharge_mwh / storage[s].e_rated_mwh;
      }
      stats.push_back(std::move(stat));
    }
  };

  int ac_active_pos = 0;
  append_group(sys.ac.storage, false, ac_active_pos);
  int dc_active_pos = 0;
  append_group(sys.dc.storage, true, dc_active_pos);
  append_group(sys.dc.dc_storage, true, dc_active_pos);
  return stats;
}

/// Compute renewable annual statistics.
static std::vector<RenewableAnnualStats> compute_renewable_stats(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const std::vector<WeeklySchedule>& weekly,
    double dt) {
  const auto& rens = sys.ac.renewable_gens;
  std::vector<RenewableAnnualStats> stats(rens.size());

  std::vector<int> ren_uc_pos(rens.size(), -1);
  {
    int pos = 0;
    for (size_t r = 0; r < rens.size(); ++r) {
      if (!rens[r].in_service) continue;
      ren_uc_pos[r] = pos++;
    }
  }

  // Build profile map for availability computation
  std::unordered_map<int, const TimeSeriesProfile*> pmap;
  for (const auto& p : ts_data.profiles) pmap[p.id] = &p;

  for (size_t r = 0; r < rens.size(); ++r) {
    stats[r].name = rens[r].name;
    stats[r].ren_index = static_cast<int>(r);
  }

  for (const auto& ws : weekly) {
    const auto& uc = ws.uc;
    for (size_t r = 0; r < rens.size(); ++r) {
      int pos = ren_uc_pos[r];
      if (pos < 0) continue;
      if (static_cast<size_t>(pos) >= uc.renewable_dispatch.size()) continue;
      const auto& disp = uc.renewable_dispatch[static_cast<size_t>(pos)];
      for (int t = 0; t < static_cast<int>(disp.size()) && t < ws.num_steps;
           ++t) {
        stats[r].total_energy_mwh += disp[static_cast<size_t>(t)] * dt;

        // Compute availability at global step for curtailment
        int global_t = ws.start_step + t;
        double avail = rens[r].p_rated_mw;
        auto pit = pmap.find(rens[r].profile_id);
        if (pit != pmap.end() &&
            global_t < static_cast<int>(pit->second->values.size())) {
          avail *= pit->second->values[static_cast<size_t>(global_t)];
        }
        double curtailed = avail - disp[static_cast<size_t>(t)];
        if (curtailed > 0.0) stats[r].total_curtailed_mwh += curtailed * dt;
      }
    }
  }

  double total_year_hours = ts_data.num_steps * dt;
  for (size_t r = 0; r < rens.size(); ++r) {
    double pmax = rens[r].p_rated_mw;
    if (pmax > 0.0 && total_year_hours > 0.0)
      stats[r].capacity_factor = stats[r].total_energy_mwh / (pmax * total_year_hours);
    double total_available = stats[r].total_energy_mwh + stats[r].total_curtailed_mwh;
    if (total_available > 0.0)
      stats[r].curtailment_rate = stats[r].total_curtailed_mwh / total_available;
  }
  return stats;
}

// ═══════════════════════════════════════════════════════════════════════
// L0: Annual Planning (coarse energy + maintenance schedule)
// ═══════════════════════════════════════════════════════════════════════

static AnnualPlanResult solve_annual_plan(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const std::vector<std::pair<int, int>>& block_ranges,
    const AnnualProductionSimOptions& /*opts*/) {
  AnnualPlanResult plan;
  plan.feasible = true;
  plan.solver_name = "annual_plan_default";

  const auto& gens = sys.ac.generators;
  const auto& storage = sys.ac.storage;
  const int n_blocks = static_cast<int>(block_ranges.size());

  // Build profile map for load/renewable estimation
  std::unordered_map<int, const TimeSeriesProfile*> pmap;
  for (const auto& p : ts_data.profiles) pmap[p.id] = &p;
  auto get_scale = [&](int pid, int t) -> double {
    auto it = pmap.find(pid);
    if (it == pmap.end()) return 1.0;
    if (t < 0 || t >= static_cast<int>(it->second->values.size())) return 1.0;
    return it->second->values[static_cast<size_t>(t)];
  };

  plan.blocks.resize(static_cast<size_t>(n_blocks));
  double dt = ts_data.step_duration_hr;

  for (int b = 0; b < n_blocks; ++b) {
    auto& blk = plan.blocks[static_cast<size_t>(b)];
    blk.block_id = b;
    blk.start_step = block_ranges[static_cast<size_t>(b)].first;
    blk.end_step = block_ranges[static_cast<size_t>(b)].second;
    int steps = blk.end_step - blk.start_step;

    // Default: no maintenance
    blk.gen_maintenance.assign(gens.size(), 0);

    // Energy budget: pmax * steps * dt per generator (unconstrained default)
    blk.gen_energy_budget_mwh.resize(gens.size());
    for (size_t g = 0; g < gens.size(); ++g) {
      blk.gen_energy_budget_mwh[g] = gens[g].pmax_mw * steps * dt;
    }

    blk.fuel_budget = 1e30;  // no fuel constraint by default

    // Storage SOC: propagate linearly
    blk.storage_init_soc.resize(storage.size());
    blk.storage_terminal_soc.resize(storage.size());
    for (size_t s = 0; s < storage.size(); ++s) {
      blk.storage_init_soc[s] = storage[s].soc_init;
      blk.storage_terminal_soc[s] = storage[s].soc_init;
    }

    // Estimate block cost (load-weighted average generation cost)
    double block_load_mwh = 0.0;
    for (int t = blk.start_step; t < blk.end_step; ++t) {
      for (const auto& ld : sys.ac.loads) {
        if (!ld.in_service) continue;
        block_load_mwh += ld.p_mw * get_scale(ld.profile_id, t) * ld.scaling * dt;
      }
    }
    // Simple cost estimate using average generator marginal cost
    double avg_mc = 0.0;
    int n_gen = 0;
    for (const auto& g : gens) {
      if (!g.in_service) continue;
      avg_mc += g.cost_c1;
      ++n_gen;
    }
    if (n_gen > 0) avg_mc /= n_gen;
    blk.block_cost = block_load_mwh * avg_mc;
    plan.total_plan_cost += blk.block_cost;
  }

  return plan;
}

static HybridPowerSystem apply_annual_block_controls(
    const HybridPowerSystem& sys,
    const AnnualPlanBlock& block) {
  HybridPowerSystem out = sys;
  for (size_t g = 0; g < out.ac.generators.size() &&
                      g < block.gen_maintenance.size();
       ++g) {
    if (block.gen_maintenance[g]) {
      out.ac.generators[g].in_service = false;
    }
  }
  for (size_t s = 0; s < out.ac.storage.size() &&
                      s < block.storage_init_soc.size();
       ++s) {
    out.ac.storage[s].soc_init = block.storage_init_soc[s];
  }
  return out;
}

// ═══════════════════════════════════════════════════════════════════════
// L2: Weekly Rolling UC
// ═══════════════════════════════════════════════════════════════════════

static WeeklySchedule solve_weekly_uc(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    int week_id, int start_step, int bind_steps, int la_steps,
    const AnnualPlanBlock& block,
    const AnnualProductionSimOptions& opts) {
  WeeklySchedule ws;
  ws.week_id = week_id;
  ws.start_step = start_step;
  ws.num_steps = bind_steps;
  ws.lookahead_steps = la_steps;

  int total_window = bind_steps + la_steps;
  int t_end = std::min(start_step + total_window, ts_data.num_steps);
  total_window = t_end - start_step;

  // Slice profiles for the sub-horizon
  auto sub_ts = slice_ts_data(ts_data, start_step, t_end);

  HybridPowerSystem sub_sys = apply_annual_block_controls(sys, block);

  // Solve UC for the sub-horizon
  ws.uc = solve_unit_commitment(sub_sys, sub_ts, opts.ts_pf_options);

  return ws;
}

// ═══════════════════════════════════════════════════════════════════════
// L3: Daily OPF/PF Replay
// ═══════════════════════════════════════════════════════════════════════

static TimeSeriesPFResult solve_daily_replay(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    int start_step, int num_steps,
    const UCSchedule& weekly_uc,
    int uc_offset,  // offset within the weekly UC arrays
    const AnnualPlanBlock& block,
    const AnnualProductionSimOptions& opts) {
  // Create day-sized slice
  int t_end = std::min(start_step + num_steps, ts_data.num_steps);
  auto sub_ts = slice_ts_data(ts_data, start_step, t_end);
  int sub_T = t_end - start_step;

  // Build a pre-committed UC schedule for the day by slicing the weekly UC
  UCSchedule day_uc;
  day_uc.feasible = weekly_uc.feasible;
  day_uc.solver_name = weekly_uc.solver_name;

  auto slice_2d_double = [&](const std::vector<std::vector<double>>& src) {
    std::vector<std::vector<double>> dst;
    dst.reserve(src.size());
    for (const auto& row : src) {
      std::vector<double> sub;
      sub.reserve(static_cast<size_t>(sub_T));
      for (int t = uc_offset; t < uc_offset + sub_T; ++t) {
        sub.push_back(
            t < static_cast<int>(row.size()) ? row[static_cast<size_t>(t)] : 0.0);
      }
      dst.push_back(std::move(sub));
    }
    return dst;
  };

  auto slice_2d_int = [&](const std::vector<std::vector<int>>& src) {
    std::vector<std::vector<int>> dst;
    dst.reserve(src.size());
    for (const auto& row : src) {
      std::vector<int> sub;
      sub.reserve(static_cast<size_t>(sub_T));
      for (int t = uc_offset; t < uc_offset + sub_T; ++t) {
        sub.push_back(
            t < static_cast<int>(row.size()) ? row[static_cast<size_t>(t)] : 0);
      }
      dst.push_back(std::move(sub));
    }
    return dst;
  };

  day_uc.gen_dispatch = slice_2d_double(weekly_uc.gen_dispatch);
  day_uc.gen_commit = slice_2d_int(weekly_uc.gen_commit);
  day_uc.ess_dispatch = slice_2d_double(weekly_uc.ess_dispatch);
  day_uc.ess_soc = slice_2d_double(weekly_uc.ess_soc);
  day_uc.renewable_dispatch = slice_2d_double(weekly_uc.renewable_dispatch);
  day_uc.ac_pv_dispatch = slice_2d_double(weekly_uc.ac_pv_dispatch);
  day_uc.ac_sgen_dispatch = slice_2d_double(weekly_uc.ac_sgen_dispatch);
  day_uc.external_grid_dispatch =
      slice_2d_double(weekly_uc.external_grid_dispatch);
  day_uc.dc_pv_dispatch = slice_2d_double(weekly_uc.dc_pv_dispatch);
  day_uc.dc_ess_dispatch = slice_2d_double(weekly_uc.dc_ess_dispatch);
  day_uc.dc_ess_soc = slice_2d_double(weekly_uc.dc_ess_soc);
  day_uc.dc_sgen_dispatch = slice_2d_double(weekly_uc.dc_sgen_dispatch);
  day_uc.dc_load_demand = slice_2d_double(weekly_uc.dc_load_demand);
  day_uc.vsc_dispatch = slice_2d_double(weekly_uc.vsc_dispatch);
  day_uc.dcdc_dispatch = slice_2d_double(weekly_uc.dcdc_dispatch);

  HybridPowerSystem sub_sys = apply_annual_block_controls(sys, block);

  // Run the full UC->OPF->PF pipeline using the already solved weekly UC
  // dispatch.  The time-series solver still does profile replay, OPF/PF, and
  // schedule fallback completion, but skips the expensive UC MILP.
  TimeSeriesPFOptions pf_opts = opts.ts_pf_options;
  pf_opts.skip_uc = false;
  pf_opts.parallel_daily = false;
  pf_opts.precomputed_uc_schedule = &day_uc;

  return solve_time_series_pf(sub_sys, sub_ts, pf_opts);
}

// ═══════════════════════════════════════════════════════════════════════
// Parallel daily decomposition
// ═══════════════════════════════════════════════════════════════════════
// The year is partitioned into independent calendar days.  Each day is an
// energy-neutral horizon (per-day cyclic SOC, e_{s,T-1}=soc_init), so the days
// are decoupled and can be solved concurrently.  Inside each day the chosen
// DailySimMode selects SCUC / dynamic SCED / dynamic OPF.

/// Translate the per-day simulation mode into a TimeSeriesPFOptions config.
static TimeSeriesPFOptions make_daily_pf_options(
    const AnnualProductionSimOptions& opts) {
  TimeSeriesPFOptions p = opts.ts_pf_options;
  p.keep_system_snapshots = false;
  p.parallel_daily = false;
  p.precomputed_uc_schedule = nullptr;
  // Per-day cyclic SOC makes the horizons independent (the key to parallelism).
  p.enforce_terminal_soc_cyclic = opts.enforce_daily_cyclic_soc;
  switch (opts.daily_mode) {
    case DailySimMode::SCUC:
      p.skip_uc = false; p.run_opf = true;  p.fix_commitment = false;
      break;
    case DailySimMode::DynamicSCED:
      // Commitment fixed ON → multi-period security-constrained economic dispatch.
      p.skip_uc = false; p.run_opf = false; p.fix_commitment = true;
      break;
    case DailySimMode::DynamicOPF:
      // Per-step AC-OPF, no unit commitment.
      p.skip_uc = true;  p.run_opf = true;  p.fix_commitment = false;
      break;
  }
  // Thread-safety: in parallel mode force the native parity IPM so the
  // non-thread-safe Ipopt/MUMPS OPF fallback is never invoked concurrently.
  if (opts.enable_parallel_daily) {
    p.opf_options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  }
  return p;
}

/// Solve the annual production simulation by independent, parallel calendar days.
static AnnualProductionSimResult solve_parallel_daily(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const AnnualProductionSimOptions& opts) {
  AnnualProductionSimResult result;
  const int T_yr = ts_data.num_steps;
  const double dt = checked_step_duration_hr(ts_data);
  result.num_steps = T_yr;
  result.step_duration_hr = dt;
  if (T_yr <= 0) return result;
  result.step_results.resize(static_cast<size_t>(T_yr));

  const int steps_per_day =
      steps_for_duration(static_cast<double>(opts.daily_window_hours), dt);
  const int num_days = (T_yr + steps_per_day - 1) / steps_per_day;
  TimeSeriesPFOptions day_opts = make_daily_pf_options(opts);
  day_opts.uc_solver =
      hacdcpf::resolve_parallel_daily_uc_solver(day_opts.uc_solver);
  day_opts.uc_solver_threads =
      day_opts.uc_solver == UCSolverChoice::Native ? 1 : 0;

  // Dynamic-SCED commitment: optionally solve a representative (peak-load) day
  // as SCUC up front and reuse its commitment across every SCED day, instead of
  // forcing all units ON.  repr_commit must outlive the parallel region.
  std::vector<std::vector<int>> repr_commit;
  TimeSeriesPFOptions eff_day_opts = day_opts;
  if (opts.daily_mode == DailySimMode::DynamicSCED &&
      opts.sced_reuse_scuc_commitment && num_days > 0) {
    int peak_day = 0;
    double peak_val = -1.0;
    const std::vector<double>* lp = nullptr;
    for (const auto& p : ts_data.profiles) {
      if (p.id == 0) { lp = &p.values; break; }
    }
    if (lp) {
      for (int d = 0; d < num_days; ++d) {
        const int g0 = d * steps_per_day;
        const int g1 = std::min(g0 + steps_per_day, T_yr);
        double mx = 0.0;
        for (int t = g0; t < g1 && t < static_cast<int>(lp->size()); ++t) {
          mx = std::max(mx, (*lp)[static_cast<size_t>(t)]);
        }
        if (mx > peak_val) { peak_val = mx; peak_day = d; }
      }
    }
    const int g0 = peak_day * steps_per_day;
    const int g1 = std::min(g0 + steps_per_day, T_yr);
    TimeSeriesData repr_ts = slice_ts_data(ts_data, g0, g1);
    TimeSeriesPFOptions scuc_opts = opts.ts_pf_options;
    scuc_opts.skip_uc = false;
    scuc_opts.run_opf = false;
    scuc_opts.fix_commitment = false;
    scuc_opts.fixed_commitment_schedule = nullptr;
    scuc_opts.enforce_terminal_soc_cyclic = opts.enforce_daily_cyclic_soc;
    scuc_opts.uc_solver =
        hacdcpf::resolve_parallel_daily_uc_solver(scuc_opts.uc_solver);
    UCSchedule repr = solve_unit_commitment(sys, repr_ts, scuc_opts);
    if (repr.feasible && !repr.gen_commit.empty()) {
      repr_commit = repr.gen_commit;
      eff_day_opts.fix_commitment = false;
      eff_day_opts.fixed_commitment_schedule = &repr_commit;
    }
    // Otherwise eff_day_opts keeps fix_commitment=true (all-on fallback).
  }

  // Concurrent solves are safe only when every solver used owns its worker
  // state. HiGHS/Gurobi share process-global scheduling state, so explicit
  // selections are guarded to serial day execution. DynamicOPF uses no MILP.
  //   * OPF: forced to the native parity IPM above (no Ipopt/MUMPS global state).
  //   * PF: the Newton kernel keeps all state per-call.
  const bool uses_milp = (opts.daily_mode != DailySimMode::DynamicOPF);
  const bool parallel_safe = !uses_milp ||
      hacdcpf::uc_solver_allows_parallel_daily(day_opts.uc_solver);
  const std::string parallel_backend =
      !uses_milp ? "no-milp"
      : (day_opts.uc_solver == UCSolverChoice::SCIP) ? "scip"
      : (day_opts.uc_solver == UCSolverChoice::Native) ? "native-bc"
      : "guarded";
  const std::string guard_reason = parallel_safe ? std::string()
      : "explicit UC solver backend is guarded for concurrent daily solves";
  auto parallel_info = util::make_parallel_execution_info(
      true, opts.parallel_threads, num_days,
      parallel_safe ? "parallel-daily/" + parallel_backend
                    : "parallel-daily/serial-guarded",
      parallel_backend, guard_reason);
  const int workers = parallel_info.resolved_workers;

  // Per-day outputs are index-addressed so worker threads never write the same
  // slot (lock-free); they are stitched together after the parallel region.
  std::vector<WeeklySchedule> day_scheds(static_cast<size_t>(num_days));
  std::vector<std::vector<PFSnapshot>> day_snaps(static_cast<size_t>(num_days));
  std::atomic<int> feasible_days{0};

  auto solve_one_day = [&](int d) {
    const int g0 = d * steps_per_day;
    const int g1 = std::min(g0 + steps_per_day, T_yr);
    const int day_steps = g1 - g0;
    if (day_steps <= 0) return;

    TimeSeriesData sub_ts = slice_ts_data(ts_data, g0, g1);
    TimeSeriesPFResult day_res = solve_time_series_pf(sys, sub_ts, eff_day_opts);

    fill_step_results(result.step_results, g0, day_res, sys, sub_ts, day_steps);

    WeeklySchedule ws;
    ws.week_id = d;
    ws.start_step = g0;
    ws.num_steps = day_steps;
    ws.uc = day_res.uc_schedule;
    if (day_res.uc_schedule.feasible) feasible_days.fetch_add(1);
    day_scheds[static_cast<size_t>(d)] = std::move(ws);

    if (opts.pf_snapshot_interval > 0) {
      for (int t = 0; t < day_steps; ++t) {
        const int g_step = g0 + t;
        if (g_step % opts.pf_snapshot_interval != 0) continue;
        if (t >= static_cast<int>(day_res.pf_results.size())) continue;
        const auto& pfr = day_res.pf_results[static_cast<size_t>(t)];
        if (!pfr.converged) continue;
        PFSnapshot snap;
        snap.global_step = g_step;
        snap.vm = pfr.vm;
        snap.vdc = pfr.vdc;
        snap.branch_flows = pfr.branch_flows;
        snap.vsc_transfers = pfr.vsc_transfers;
        snap.dcdc_transfers = pfr.dcdc_transfers;
        snap.converged = true;
        day_snaps[static_cast<size_t>(d)].push_back(std::move(snap));
      }
    }
  };

  if (parallel_info.effective) {
    util::ThreadPool pool(workers);
    std::vector<std::future<void>> futs;
    futs.reserve(static_cast<size_t>(num_days));
    for (int d = 0; d < num_days; ++d) {
      futs.push_back(pool.submit([&solve_one_day, d]() { solve_one_day(d); }));
    }
    for (auto& f : futs) f.get();
  } else {
    for (int d = 0; d < num_days; ++d) solve_one_day(d);
  }

  // Stitch ordered outputs.
  result.weekly_schedules = std::move(day_scheds);
  for (auto& v : day_snaps) {
    for (auto& s : v) result.pf_snapshots.push_back(std::move(s));
  }

  // Monthly summaries + component statistics (reuse the sequential helpers).
  auto block_ranges = build_block_ranges(T_yr, dt, AnnualBlockType::Monthly);
  result.monthly_summaries.reserve(block_ranges.size());
  for (size_t b = 0; b < block_ranges.size(); ++b) {
    result.monthly_summaries.push_back(aggregate_block(
        static_cast<int>(b), block_ranges[b].first, block_ranges[b].second,
        result.step_results, dt));
  }
  result.gen_stats = compute_gen_stats(sys, result.weekly_schedules, dt);
  result.storage_stats = compute_storage_stats(sys, result.weekly_schedules, dt);
  result.renewable_stats =
      compute_renewable_stats(sys, ts_data, result.weekly_schedules, dt);

  for (const auto& ms : result.monthly_summaries) {
    result.total_gen_mwh += ms.total_gen_mwh;
    result.total_load_mwh += ms.total_load_mwh;
    result.total_renewable_mwh += ms.total_renewable_mwh;
    result.total_curtailment_mwh += ms.total_curtailment_mwh;
    result.storage_discharge_mwh += ms.storage_discharge_mwh;
    result.storage_charge_mwh += ms.storage_charge_mwh;
    result.external_grid_import_mwh += ms.external_grid_import_mwh;
    result.external_grid_export_mwh += ms.external_grid_export_mwh;
    result.total_supply_mwh += ms.total_supply_mwh;
    result.total_demand_mwh += ms.total_demand_mwh;
    result.power_balance_error_mwh += ms.power_balance_error_mwh;
    result.total_ens_mwh += ms.total_ens_mwh;
    result.total_loss_mwh += ms.total_loss_mwh;
    result.total_cost += ms.total_cost;
    result.num_pf_converged += ms.num_pf_converged;
    result.num_opf_converged += ms.num_opf_converged;
  }

  result.feasible = (feasible_days.load() == num_days);
  const char* mode_tag = (opts.daily_mode == DailySimMode::SCUC) ? "SCUC"
                         : (opts.daily_mode == DailySimMode::DynamicSCED)
                             ? "dyn-SCED"
                             : "dyn-OPF";
  result.solver_name = std::string("parallel-daily/") + mode_tag +
                       (parallel_safe ? "/parallel" : "/serial");
  parallel_info.actual_parallel_evaluations =
      parallel_info.effective ? num_days : 0;
  parallel_info.serial_evaluations =
      parallel_info.effective ? 0 : num_days;
  if (!parallel_info.effective && parallel_info.guard_reason.empty()) {
    parallel_info.guard_reason = util::insufficient_work_reason(parallel_info);
  }
  result.parallel_execution = parallel_info;
  result.parallel_daily_effective = parallel_info.effective;
  result.parallel_workers = workers;
  result.parallel_mode = parallel_info.mode;
  return result;
}

// ═══════════════════════════════════════════════════════════════════════
// Main Orchestrator
// ═══════════════════════════════════════════════════════════════════════

AnnualProductionSimResult solve_annual_production_simulation(
    const HybridPowerSystem& sys,
    const TimeSeriesData& ts_data,
    const AnnualProductionSimOptions& opts) {
  AnnualProductionSimResult result;

  const int T_yr = ts_data.num_steps;
  const double dt = checked_step_duration_hr(ts_data);
  result.num_steps = T_yr;
  result.step_duration_hr = dt;
  result.parallel_workers = 1;
  result.parallel_mode = opts.enable_parallel_daily ? "parallel-daily" : "hierarchical";
  result.parallel_execution = util::make_parallel_execution_info(
      opts.enable_parallel_daily, opts.parallel_threads, 0,
      result.parallel_mode);

  if (T_yr <= 0) {
    return result;
  }

  // Parallel daily decomposition path: independent, energy-neutral calendar days
  // solved concurrently (bypasses the sequential L0→L3 hierarchy).
  if (opts.enable_parallel_daily) {
    return solve_parallel_daily(sys, ts_data, opts);
  }

  result.step_results.resize(static_cast<size_t>(T_yr));

  // ─── L0: Annual Planning ───
  auto block_ranges = build_block_ranges(T_yr, dt, opts.block_type);
  result.annual_plan = solve_annual_plan(sys, ts_data, block_ranges, opts);

  // ─── L1/L2: Monthly → Weekly Rolling UC ───
  const int steps_per_week = steps_for_duration(168.0, dt);
  const int steps_per_day =
      steps_for_duration(static_cast<double>(opts.daily_window_hours), dt);
  const int la_steps =
      opts.weekly_lookahead_hours <= 0
          ? 0
          : steps_for_duration(static_cast<double>(opts.weekly_lookahead_hours),
                               dt);

  int week_counter = 0;

  for (size_t b = 0; b < result.annual_plan.blocks.size(); ++b) {
    const auto& block = result.annual_plan.blocks[b];
    int block_start = block.start_step;
    int block_end = block.end_step;
    const HybridPowerSystem block_sys = apply_annual_block_controls(sys, block);

    // Partition block into weekly windows
    int cursor = block_start;
    while (cursor < block_end) {
      int bind_end = std::min(cursor + steps_per_week, block_end);
      int bind_steps = bind_end - cursor;

      // L2: Solve weekly UC
      WeeklySchedule ws = solve_weekly_uc(
          sys, ts_data, week_counter, cursor, bind_steps, la_steps,
          block, opts);

      // ─── L3: Daily OPF/PF Replay ───
      if (!opts.skip_replay) {
        int day_cursor = 0;
        while (day_cursor < bind_steps) {
          int day_steps = std::min(steps_per_day, bind_steps - day_cursor);
          int global_day_start = cursor + day_cursor;

          TimeSeriesPFResult day_result = solve_daily_replay(
              sys, ts_data, global_day_start, day_steps,
              ws.uc, day_cursor, block, opts);

          // Fill annual step results
          auto sub_ts_slice = slice_ts_data(ts_data, global_day_start,
                                             global_day_start + day_steps);
          fill_step_results(result.step_results, global_day_start,
                            day_result, block_sys, sub_ts_slice, day_steps);

          // Store PF snapshots at configured interval
          if (opts.pf_snapshot_interval > 0) {
            for (int t = 0; t < day_steps; ++t) {
              int g_step = global_day_start + t;
              if (g_step % opts.pf_snapshot_interval != 0) continue;
              if (t >= static_cast<int>(day_result.pf_results.size())) continue;
              const auto& pfr = day_result.pf_results[static_cast<size_t>(t)];
              if (!pfr.converged) continue;
              PFSnapshot snap;
              snap.global_step = g_step;
              snap.vm = pfr.vm;
              snap.vdc = pfr.vdc;
              snap.branch_flows = pfr.branch_flows;
              snap.vsc_transfers = pfr.vsc_transfers;
              snap.dcdc_transfers = pfr.dcdc_transfers;
              snap.converged = true;
              result.pf_snapshots.push_back(std::move(snap));
            }
          }

          day_cursor += day_steps;
        }
      }

      result.weekly_schedules.push_back(std::move(ws));
      cursor = bind_end;
      ++week_counter;
    }
  }

  // ─── Populate step_results from UC schedules when replay was skipped ───
  if (opts.skip_replay) {
    // Build profile map for load calculations
    std::unordered_map<int, const TimeSeriesProfile*> pmap;
    for (const auto& p : ts_data.profiles) pmap[p.id] = &p;
    auto get_scale = [&](int pid, int t) -> double {
      auto it = pmap.find(pid);
      if (it == pmap.end()) return 1.0;
      if (t < 0 || t >= static_cast<int>(it->second->values.size())) return 1.0;
      return it->second->values[static_cast<size_t>(t)];
    };

    for (const auto& ws : result.weekly_schedules) {
      for (int t = 0; t < ws.num_steps; ++t) {
        int g_idx = ws.start_step + t;
        if (g_idx < 0 || g_idx >= static_cast<int>(result.step_results.size())) continue;
        auto& sr = result.step_results[static_cast<size_t>(g_idx)];
        const ReportedProfileDispatch reported =
            reported_profile_dispatch(sys, ws.uc, pmap, t, g_idx);
        const ReportedExternalGridDispatch grid =
            reported_external_grid_dispatch(sys, ws.uc, pmap, t, g_idx);

        // Generation from UC schedule
        double gen = 0.0;
        for (size_t g = 0; g < ws.uc.gen_dispatch.size(); ++g) {
          const auto& disp = ws.uc.gen_dispatch[g];
          if (t < static_cast<int>(disp.size())) gen += disp[static_cast<size_t>(t)];
        }
        sr.total_gen_mw = gen + reported.dispatchable_generation_mw +
                          std::max(0.0, grid.net_import_mw);

        // Load from profiles (ac.loads + bus pd_mw fallback)
        double load = 0.0;
        if (!sys.ac.loads.empty()) {
          for (const auto& ld : sys.ac.loads) {
            if (!ld.in_service) continue;
            load += ld.p_mw * get_scale(ld.profile_id, g_idx) * ld.scaling;
          }
        } else {
          for (const auto& b : sys.ac.buses) {
            if (b.pd_mw > 0.0) load += b.pd_mw * get_scale(0, g_idx);
          }
        }
        sr.total_load_mw = load + reported.dc_load_mw;

        // Renewable dispatch
        double ren = reported_ac_renewable_mw(sys, ws.uc, pmap, t, g_idx);
        sr.total_renewable_mw = ren + reported.renewable_generation_mw;

        // ESS net dispatch
        const auto ess_power = scheduled_storage_power(ws.uc, t);
        const double ess = ess_power.net_mw;
        sr.total_ess_mw = ess;
        sr.storage_discharge_mw = ess_power.discharge_mw;
        sr.storage_charge_mw = ess_power.charge_mw;

        const double supply_mw =
            gen + ren + reported.dispatchable_generation_mw +
            reported.renewable_generation_mw + std::max(0.0, ess) +
            std::max(0.0, grid.net_import_mw);
        const double need_mw =
            sr.total_load_mw + std::max(0.0, sr.total_loss_mw) +
            std::max(0.0, -ess) + std::max(0.0, -grid.net_import_mw);
        finalize_step_power_accounting(sr, grid.net_import_mw);
        const double import_mw =
            grid.scheduled ? 0.0 : std::max(0.0, need_mw - supply_mw);
        sr.opf_cost =
            generator_operating_cost_rate_from_uc(sys, ws.uc, t) +
            reported.cost_rate_per_hr +
            storage_bid_cost_rate(sys, ws.uc, t) +
            grid.cost_rate_per_hr +
            import_mw * reported_external_grid_price(sys, pmap, g_idx);
        sr.opf_converged = true;
        sr.pf_converged = true;
      }
    }
  }

  // ─── Aggregate monthly summaries ───
  result.monthly_summaries.reserve(block_ranges.size());
  for (size_t b = 0; b < block_ranges.size(); ++b) {
    result.monthly_summaries.push_back(aggregate_block(
        static_cast<int>(b),
        block_ranges[b].first,
        block_ranges[b].second,
        result.step_results, dt));
  }

  // ─── Component-level statistics ───
  result.gen_stats = compute_gen_stats(sys, result.weekly_schedules, dt);
  result.storage_stats = compute_storage_stats(sys, result.weekly_schedules, dt);
  result.renewable_stats =
      compute_renewable_stats(sys, ts_data, result.weekly_schedules, dt);

  // ─── Scalar annual metrics ───
  for (const auto& ms : result.monthly_summaries) {
    result.total_gen_mwh += ms.total_gen_mwh;
    result.total_load_mwh += ms.total_load_mwh;
    result.total_renewable_mwh += ms.total_renewable_mwh;
    result.total_curtailment_mwh += ms.total_curtailment_mwh;
    result.storage_discharge_mwh += ms.storage_discharge_mwh;
    result.storage_charge_mwh += ms.storage_charge_mwh;
    result.external_grid_import_mwh += ms.external_grid_import_mwh;
    result.external_grid_export_mwh += ms.external_grid_export_mwh;
    result.total_supply_mwh += ms.total_supply_mwh;
    result.total_demand_mwh += ms.total_demand_mwh;
    result.power_balance_error_mwh += ms.power_balance_error_mwh;
    result.total_ens_mwh += ms.total_ens_mwh;
    result.total_loss_mwh += ms.total_loss_mwh;
    result.total_cost += ms.total_cost;
    result.num_pf_converged += ms.num_pf_converged;
    result.num_opf_converged += ms.num_opf_converged;
  }

  result.feasible = result.annual_plan.feasible;
  result.solver_name = result.annual_plan.solver_name;

  return result;
}

// ═══════════════════════════════════════════════════════════════════════
// Text summary
// ═══════════════════════════════════════════════════════════════════════

std::string AnnualProductionSimResult::summary() const {
  std::ostringstream ss;
  ss << "=== Annual Production Simulation Summary ===\n";
  ss << "Steps:        " << num_steps << " (dt=" << step_duration_hr << " h)\n";
  ss << "Feasible:     " << (feasible ? "yes" : "no") << "\n";
  ss << "Solver:       " << solver_name << "\n";
  ss << "Total cost:   " << total_cost << " $/yr\n";
  ss << "Generation:   " << total_gen_mwh << " MWh\n";
  ss << "Load:         " << total_load_mwh << " MWh\n";
  ss << "Renewable:    " << total_renewable_mwh << " MWh\n";
  ss << "Curtailment:  " << total_curtailment_mwh << " MWh\n";
  ss << "ENS:          " << total_ens_mwh << " MWh\n";
  ss << "Losses:       " << total_loss_mwh << " MWh\n";
  ss << "PF converged: " << num_pf_converged << " / " << num_steps << "\n";
  ss << "OPF converged:" << num_opf_converged << " / " << num_steps << "\n";
  ss << "\n--- Monthly Summaries ---\n";
  static const char* month_names[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                       "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  for (size_t m = 0; m < monthly_summaries.size(); ++m) {
    const auto& ms = monthly_summaries[m];
    const char* mname = (m < 12) ? month_names[m] : "Blk";
    ss << "  " << mname << ": gen=" << ms.total_gen_mwh
       << " load=" << ms.total_load_mwh
       << " ren=" << ms.total_renewable_mwh
       << " loss=" << ms.total_loss_mwh
       << " cost=" << ms.total_cost << "\n";
  }
  ss << "\n--- Generator Stats ---\n";
  for (const auto& gs : gen_stats) {
    if (gs.total_energy_mwh < 1e-6) continue;
    ss << "  " << gs.name << ": E=" << gs.total_energy_mwh
       << " MWh, CF=" << (gs.capacity_factor * 100.0) << "%"
       << ", SU=" << gs.total_startups
       << ", hours=" << gs.total_hours_online << "\n";
  }
  ss << "\n--- Renewable Stats ---\n";
  for (const auto& rs : renewable_stats) {
    ss << "  " << rs.name << ": E=" << rs.total_energy_mwh
       << " MWh, curt=" << rs.total_curtailed_mwh
       << " MWh (" << (rs.curtailment_rate * 100.0) << "%)\n";
  }
  return ss.str();
}

}  // namespace hacdcpf::analysis
