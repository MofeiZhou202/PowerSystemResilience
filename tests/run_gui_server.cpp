#define _USE_MATH_DEFINES  // M_PI on strict-conformance toolchains (MSYS2 UCRT, MSVC)
#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/short_circuit.hpp"
#include "hacdcpf/analysis/harmonics_power_flow.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/power_flow/assembly/branch_flow.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/etap_io.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/power_flow/three_phase.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"
#include "hacdcpf/network_reconfiguration/topology_analysis.hpp"
#include "hacdcpf/network_reconfiguration/topology_reconfiguration.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"
#include "hacdcpf/time_series/annual_production_sim.hpp"
#include "hacdcpf/time_series/lifecycle_simulation.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"
#include "hacdcpf/analysis/typhoon_resilience.hpp"
#include "hacdcpf/analysis/scenario_generation.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/graph/graph.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace {

void apply_replay_zip_and_solver_flags(hacdcpf::powerflow::SolverData& data,
                                       const hacdcpf::PowerFlowOptions& opt) {
  for (int k = 0; k < 3; ++k) {
    data.zip_pw[k] = opt.zip_pw[k];
    data.zip_qw[k] = opt.zip_qw[k];
  }
  data.enable_coupled_jacobian = opt.enable_coupled_jacobian;
  data.enable_augmented_equations = opt.enable_augmented_equations;
  data.enable_semi_smooth_newton = opt.enable_semi_smooth_newton;
}

void populate_projected_replay_results(const hacdcpf::powerflow::SolverData& data,
                                       hacdcpf::PowerFlowResult& result,
                                       hacdcpf::LossModelType loss_model) {
  result.branch_flows.clear();
  result.vsc_transfers.clear();
  result.dcdc_transfers.clear();
  result.er_port_transfers.clear();
  if (!result.converged) return;

  result.branch_flows =
      hacdcpf::powerflow::compute_branch_flows(data, result.vm, result.va);

  Eigen::VectorXd vm =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(result.vm.size()));
  Eigen::VectorXd va =
      Eigen::VectorXd::Zero(static_cast<Eigen::Index>(result.va.size()));
  Eigen::VectorXd vdc =
      Eigen::VectorXd::Ones(static_cast<Eigen::Index>(result.vdc.size()));
  for (Eigen::Index i = 0; i < vm.size(); ++i) vm[i] = result.vm[static_cast<size_t>(i)];
  for (Eigen::Index i = 0; i < va.size(); ++i) va[i] = result.va[static_cast<size_t>(i)];
  for (Eigen::Index i = 0; i < vdc.size(); ++i) vdc[i] = result.vdc[static_cast<size_t>(i)];

  std::unordered_map<int, int> dc_pos_by_bus;
  dc_pos_by_bus.reserve(data.dc_buses.size());
  for (int i = 0; i < static_cast<int>(data.dc_buses.size()); ++i) {
    dc_pos_by_bus[data.dc_buses[static_cast<size_t>(i)].index] = i;
  }
  auto dc_pos = [&](int bus) {
    const auto it = dc_pos_by_bus.find(bus);
    return it != dc_pos_by_bus.end() ? it->second : -1;
  };

  const std::vector<hacdcpf::VSCConverter>& eff_converters =
      !result.diagnostics.effective_converters.empty()
          ? result.diagnostics.effective_converters
          : data.converters;
  result.vsc_transfers.reserve(eff_converters.size());
  for (const auto& conv : eff_converters) {
    if (!conv.in_service) continue;
    const auto [p_ac_pu, q_ac_pu] =
        hacdcpf::powerflow::converter_ac_injection(
            conv, vm, va, vdc, data.base_mva, loss_model);
    const double p_dc_pu =
        hacdcpf::powerflow::converter_dc_injection(
            conv, vm, va, vdc, data.base_mva, loss_model);
    hacdcpf::VSCTransfer tr;
    tr.index = conv.index;
    tr.bus_ac = conv.bus_ac;
    tr.bus_dc = conv.bus_dc;
    tr.p_ac_mw = p_ac_pu * data.base_mva;
    tr.q_ac_mvar = q_ac_pu * data.base_mva;
    tr.p_dc_mw = p_dc_pu * data.base_mva;
    tr.loss_mw = -(tr.p_ac_mw + tr.p_dc_mw);
    result.vsc_transfers.push_back(tr);
  }

  result.dcdc_transfers.reserve(data.dcdc_converters.size());
  for (const auto& c : data.dcdc_converters) {
    if (!c.in_service) continue;
    const auto xfer =
        hacdcpf::powerflow::dcdc_power_transfer(c, vdc, data.base_mva);
    hacdcpf::DCDCTransfer tr;
    tr.index = c.index;
    tr.bus_in = c.bus_in;
    tr.bus_out = c.bus_out;
    tr.p_in_mw = xfer.p_in_pu * data.base_mva;
    tr.p_out_mw = xfer.p_out_pu * data.base_mva;
    tr.loss_mw = tr.p_in_mw - tr.p_out_mw;
    const int bi = dc_pos(c.bus_in);
    const int bo = dc_pos(c.bus_out);
    const double v_in =
        (bi >= 0 && bi < static_cast<int>(vdc.size()))
            ? vdc[static_cast<Eigen::Index>(bi)]
            : 1.0;
    const double v_out =
        (bo >= 0 && bo < static_cast<int>(vdc.size()))
            ? vdc[static_cast<Eigen::Index>(bo)]
            : 1.0;
    const auto duty = hacdcpf::powerflow::dcdc_duty_ratio(c, v_in, v_out);
    tr.duty = duty.duty;
    tr.voltage_ratio = duty.voltage_ratio;
    tr.duty_defined = duty.defined;
    tr.duty_feasible = duty.feasible;
    result.dcdc_transfers.push_back(tr);
  }

  for (const auto& er : data.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service) continue;
      hacdcpf::ERPortTransfer tr;
      tr.router_index = er.index;
      tr.port_index = p.index;
      tr.bus = p.bus;
      tr.is_ac = (p.port_type == hacdcpf::ERPortType::AC);
      tr.p_mw = p.p_mw;
      tr.q_mvar = p.q_mvar;
      if (tr.is_ac) {
        const auto it = std::find_if(
            data.ac_buses.begin(), data.ac_buses.end(),
            [&](const hacdcpf::ACBus& b) { return b.index == p.bus; });
        const size_t pos = static_cast<size_t>(std::distance(data.ac_buses.begin(), it));
        tr.v_pu = (it != data.ac_buses.end() && pos < result.vm.size())
                      ? result.vm[pos]
                      : 1.0;
      } else {
        const int b = dc_pos(p.bus);
        tr.v_pu = (b >= 0 && b < static_cast<int>(result.vdc.size()))
                      ? result.vdc[static_cast<size_t>(b)]
                      : 1.0;
      }
      result.er_port_transfers.push_back(tr);
    }
  }

  auto& sc = result.converter_model_scope;
  sc.model_scope = "steady-state-newton:vsc-3mode+dcdc-power-transfer";
  sc.validity.vsc_loss_modelled = true;
  sc.validity.vsc_ac_conduction_loss_modelled = true;
  sc.validity.vsc_vdc_control_modelled = true;
  sc.validity.dcdc_loss_modelled = true;
  sc.validity.dc_multisource_coordination_modelled = true;
  sc.validity.equation_closure_checked =
      result.diagnostics.equation_closure_checked;
}

void unproject_replay_branch_flows(hacdcpf::PowerFlowResult& result,
                                   const hacdcpf::BusMergeMap& map) {
  if (map.n_original_branches <= 0 || map.branch_orig_to_proj.empty()) return;

  std::vector<hacdcpf::BranchFlow> out(static_cast<size_t>(map.n_original_branches));
  for (const auto& [orig_pos, proj_pos] : map.branch_orig_to_proj) {
    if (orig_pos < 0 || orig_pos >= map.n_original_branches) continue;
    if (proj_pos < 0 || proj_pos >= static_cast<int>(result.branch_flows.size())) continue;
    out[static_cast<size_t>(orig_pos)] = result.branch_flows[static_cast<size_t>(proj_pos)];
  }
  result.branch_flows = std::move(out);
}

void unproject_replay_result(hacdcpf::PowerFlowResult& result,
                             const hacdcpf::BusMergeMap& map) {
  if (!map.ext_to_int.empty() && map.n_original > 0) {
    result.vm = hacdcpf::unproject_bus_vector(result.vm, map);
    result.va = hacdcpf::unproject_bus_vector(result.va, map);
  }
  unproject_replay_branch_flows(result, map);
}

hacdcpf::PowerFlowResult solve_projected_replay_power_flow(
    hacdcpf::HybridPowerSystem replay_sys,
    const hacdcpf::PowerFlowOptions& opt = {},
    hacdcpf::PowerFlowResult* projected_result = nullptr) {
  auto data = hacdcpf::powerflow::make_solver_data_projected(
      std::move(replay_sys), opt.loss_model);
  apply_replay_zip_and_solver_flags(data, opt);
  hacdcpf::powerflow::NewtonSolver solver;
  const hacdcpf::InitialState* init_ptr =
      opt.initial_state ? &*opt.initial_state : nullptr;
  hacdcpf::PowerFlowResult result = solver.solve(data, opt, init_ptr);
  populate_projected_replay_results(data, result, opt.loss_model);
  if (projected_result != nullptr) *projected_result = result;
  if (data.bus_merge_map) unproject_replay_result(result, *data.bus_merge_map);
  if (result.converged) {
    for (const auto& tr : result.dcdc_transfers) {
      if (tr.duty_defined && !tr.duty_feasible) {
        result.diagnostics.warnings.push_back(
            "[DCDC-PHYS-01] DC/DC converter " + std::to_string(tr.index) +
            " duty ratio " + std::to_string(tr.duty) +
            " is outside its feasible window (Vout/Vin=" +
            std::to_string(tr.voltage_ratio) + ").");
      }
    }
  }
  return result;
}

std::vector<double> project_replay_ac_bus_vector(
    const std::vector<double>& original_space,
    const hacdcpf::HybridPowerSystem& projected_sys) {
  if (!projected_sys.bus_merge_map || original_space.empty()) return original_space;
  const auto& map = *projected_sys.bus_merge_map;
  std::vector<double> projected(static_cast<size_t>(map.n_merged), 0.0);
  std::vector<char> assigned(static_cast<size_t>(map.n_merged), 0);
  for (const auto& [ext_bus, int_pos] : map.ext_to_int) {
    const auto op = map.ext_to_orig_pos.find(ext_bus);
    if (op == map.ext_to_orig_pos.end()) continue;
    if (int_pos < 0 || int_pos >= static_cast<int>(projected.size())) continue;
    const int orig_pos = op->second;
    if (orig_pos < 0 || orig_pos >= static_cast<int>(original_space.size())) continue;
    if (!assigned[static_cast<size_t>(int_pos)]) {
      projected[static_cast<size_t>(int_pos)] = original_space[static_cast<size_t>(orig_pos)];
      assigned[static_cast<size_t>(int_pos)] = 1;
    }
  }
  for (size_t i = 0; i < projected.size() && i < original_space.size(); ++i) {
    if (!assigned[i]) projected[i] = original_space[i];
  }
  return projected;
}

std::optional<double> replay_ac_value_for_bus(
    const std::vector<double>& original_space,
    const hacdcpf::HybridPowerSystem& projected_sys,
    int projected_bus) {
  if (original_space.empty()) return std::nullopt;
  if (projected_sys.bus_merge_map) {
    const auto& map = *projected_sys.bus_merge_map;
    const int int_pos = projected_bus - 1;
    if (int_pos >= 0 && int_pos < static_cast<int>(map.int_to_ext.size())) {
      const int ext_bus = map.int_to_ext[static_cast<size_t>(int_pos)];
      const auto op = map.ext_to_orig_pos.find(ext_bus);
      if (op != map.ext_to_orig_pos.end() && op->second >= 0 &&
          op->second < static_cast<int>(original_space.size())) {
        return original_space[static_cast<size_t>(op->second)];
      }
    }
  }
  const int pos = projected_bus - 1;
  if (pos >= 0 && pos < static_cast<int>(original_space.size())) {
    return original_space[static_cast<size_t>(pos)];
  }
  return std::nullopt;
}

std::optional<size_t> replay_ac_original_pos_for_bus(
    const hacdcpf::HybridPowerSystem& projected_sys,
    int projected_bus) {
  if (projected_sys.bus_merge_map) {
    const auto& map = *projected_sys.bus_merge_map;
    const int int_pos = projected_bus - 1;
    if (int_pos >= 0 && int_pos < static_cast<int>(map.int_to_ext.size())) {
      const int ext_bus = map.int_to_ext[static_cast<size_t>(int_pos)];
      const auto op = map.ext_to_orig_pos.find(ext_bus);
      if (op != map.ext_to_orig_pos.end() && op->second >= 0) {
        return static_cast<size_t>(op->second);
      }
    }
  }
  const int pos = projected_bus - 1;
  return pos >= 0 ? std::optional<size_t>(static_cast<size_t>(pos)) : std::nullopt;
}

unsigned int fresh_typhoon_seed() {
  static std::random_device rd;
  static std::mt19937 rng(rd());
  static std::mutex mu;
  std::lock_guard<std::mutex> lk(mu);
  return rng();
}

// ───────────────────────────────────────────────────────────────────────────
// Harmonic power-flow REST helpers (shared by every /api/session/harmonics* route)
// ───────────────────────────────────────────────────────────────────────────
namespace hpf_api {
namespace H = hacdcpf::harmonics;

// Parse the common HPFOptions block (used by all harmonic endpoints).
inline void parse_options(const json& o, H::HPFOptions& opt) {
  if (o.contains("ac_orders") && o["ac_orders"].is_array())
    opt.ac_orders = o["ac_orders"].get<std::vector<int>>();
  if (o.contains("dc_orders") && o["dc_orders"].is_array())
    opt.dc_orders = o["dc_orders"].get<std::vector<int>>();
  if (o.contains("include_load_impedance"))
    opt.include_load_impedance = o["include_load_impedance"].get<bool>();
  if (o.contains("run_base_power_flow"))
    opt.run_base_power_flow = o["run_base_power_flow"].get<bool>();
  if (o.contains("default_source_xpp_pu"))
    opt.default_source_xpp_pu = o["default_source_xpp_pu"].get<double>();
  if (o.contains("dc_source_impedance_pu"))
    opt.dc_source_impedance_pu = o["dc_source_impedance_pu"].get<double>();
  if (o.contains("auto_nic_from_vscs"))
    opt.auto_nic_from_vscs = o["auto_nic_from_vscs"].get<bool>();
  if (o.contains("compute_branch_flows"))
    opt.compute_branch_flows = o["compute_branch_flows"].get<bool>();
  if (o.contains("skin_effect")) {
    const std::string s = o["skin_effect"].get<std::string>();
    opt.skin_effect = (s == "sqrt" || s == "SqrtOrder") ? H::SkinEffectModel::SqrtOrder
                    : (s == "prop" || s == "ProportionalSqrt")
                          ? H::SkinEffectModel::ProportionalSqrt
                          : H::SkinEffectModel::None;
  }
  if (o.contains("skin_coefficient"))
    opt.skin_coefficient = o["skin_coefficient"].get<double>();
  // Optional frequency-dependent DC ripple network refinement.
  if (o.contains("dc_ripple_branch_x") && o["dc_ripple_branch_x"].is_array())
    for (const auto& e : o["dc_ripple_branch_x"])
      opt.dc_ripple_model.branch_x_pu[e.value("index", 0)] = e.value("x_pu", 0.0);
  if (o.contains("dc_ripple_bus_b") && o["dc_ripple_bus_b"].is_array())
    for (const auto& e : o["dc_ripple_bus_b"])
      opt.dc_ripple_model.bus_b_pu[e.value("index", 0)] = e.value("b_pu", 0.0);
}

inline H::HarmonicSpectrum parse_spectrum(const json& arr) {
  H::HarmonicSpectrum spec;
  if (arr.is_array())
    for (const auto& l : arr)
      spec.push_back({l.value("order", 0), l.value("mag_percent", 0.0),
                      l.value("phase_deg", 0.0)});
  return spec;
}

// Parse user harmonic current sources and explicit NIC overrides into `inputs`.
inline void parse_inputs(const json& j, H::HarmonicStudyInputs& inputs) {
  if (j.contains("sources") && j["sources"].is_array()) {
    for (const auto& s : j["sources"]) {
      H::HarmonicCurrentSource src;
      src.bus = s.value("bus", 0);
      src.is_dc = s.value("is_dc", false);
      src.i_base_pu = s.value("i_base_pu", 0.0);
      src.i_base_phase_deg = s.value("i_base_phase_deg", 0.0);
      src.name = s.value("name", std::string());
      if (s.contains("spectrum")) src.spectrum = parse_spectrum(s["spectrum"]);
      inputs.sources.push_back(std::move(src));
    }
  }
  if (j.contains("nics") && j["nics"].is_array()) {
    for (const auto& n : j["nics"]) {
      H::HarmonicNIC nic;
      nic.vsc_index = n.value("vsc_index", -1);
      nic.bus_ac = n.value("bus_ac", 0);
      nic.bus_dc = n.value("bus_dc", 0);
      nic.name = n.value("name", std::string());
      nic.s_ac_p_mw = n.value("s_ac_p_mw", 0.0);
      nic.s_ac_q_mvar = n.value("s_ac_q_mvar", 0.0);
      nic.p_dc_mw = n.value("p_dc_mw", 0.0);
      if (n.contains("ac_spectrum")) nic.ac_spectrum = parse_spectrum(n["ac_spectrum"]);
      if (n.contains("dc_spectrum")) nic.dc_spectrum = parse_spectrum(n["dc_spectrum"]);
      if (n.contains("y_out_ac_g") || n.contains("y_out_ac_b"))
        nic.y_out_ac = H::Complex(n.value("y_out_ac_g", 0.0), n.value("y_out_ac_b", 0.0));
      inputs.nics.push_back(std::move(nic));
    }
  }
}

// Serialize a single-phase / DC bus harmonic result (incl. the per-order spectrum).
inline json bus_json(const H::HarmonicBusResult& b) {
  json jb;
  jb["bus"] = b.bus;
  jb["is_dc"] = b.is_dc;
  jb["v_fund_pu"] = b.v_fund_pu;
  jb["thd_pct"] = b.thd_pct;
  json spec = json::array();
  for (const auto& [ord, v] : b.v_by_order)
    spec.push_back(json{{"order", ord}, {"mag_pu", std::abs(v)},
                        {"phase_deg", std::arg(v) * 180.0 / M_PI}});
  jb["harmonics"] = spec;
  return jb;
}

// Parse a {order: [re, im]} or {order: {re, im}} map of complex per-order values.
inline std::map<int, H::Complex> parse_order_complex(const json& m) {
  std::map<int, H::Complex> out;
  if (!m.is_array()) return out;
  for (const auto& e : m) {
    int ord = e.value("order", 0);
    out[ord] = H::Complex(e.value("re", 0.0), e.value("im", 0.0));
  }
  return out;
}

}  // namespace hpf_api

constexpr std::array<hacdcpf::analysis::TyphoonIntensityCategory, 6> kTyphoonCategories{
    hacdcpf::analysis::TyphoonIntensityCategory::TD,
    hacdcpf::analysis::TyphoonIntensityCategory::TS,
    hacdcpf::analysis::TyphoonIntensityCategory::STS,
    hacdcpf::analysis::TyphoonIntensityCategory::TY,
    hacdcpf::analysis::TyphoonIntensityCategory::STY,
    hacdcpf::analysis::TyphoonIntensityCategory::SuperTY,
};

int typhoon_category_ordinal(hacdcpf::analysis::TyphoonIntensityCategory category) {
  for (std::size_t i = 0; i < kTyphoonCategories.size(); ++i) {
    if (kTyphoonCategories[i] == category) return static_cast<int>(i);
  }
  return -1;
}

const hacdcpf::analysis::TyphoonTrackSample* sample_typhoon_category_with_fallback(
    const hacdcpf::analysis::TyphoonCatalog& catalog,
    hacdcpf::analysis::TyphoonIntensityCategory requested,
    unsigned int selection_seed,
    hacdcpf::analysis::TyphoonIntensityCategory& selected_category,
    bool& used_fallback) {
  selected_category = requested;
  used_fallback = false;
  if (const auto* sample = hacdcpf::analysis::sample_typhoon_catalog(catalog, requested, selection_seed)) return sample;
  const int ord = typhoon_category_ordinal(requested);
  if (ord >= 0) {
    for (int distance = 1; distance < static_cast<int>(kTyphoonCategories.size()); ++distance) {
      for (const int sign : {-1, 1}) {
        const int candidate_ord = ord + sign * distance;
        if (candidate_ord < 0 || candidate_ord >= static_cast<int>(kTyphoonCategories.size())) continue;
        const auto candidate = kTyphoonCategories[static_cast<std::size_t>(candidate_ord)];
        if (const auto* sample = hacdcpf::analysis::sample_typhoon_catalog(catalog, candidate, selection_seed + static_cast<unsigned int>(distance))) {
          selected_category = candidate;
          used_fallback = true;
          return sample;
        }
      }
    }
  }
  if (catalog.samples.empty()) return nullptr;
  std::mt19937 rng(selection_seed);
  std::uniform_int_distribution<std::size_t> pick(0, catalog.samples.size() - 1);
  const auto* sample = &catalog.samples[pick(rng)];
  selected_category = sample->category;
  used_fallback = true;
  return sample;
}

json typhoon_catalog_counts_json(const hacdcpf::analysis::TyphoonCatalog& catalog) {
  json counts = json::object();
  for (const auto category : kTyphoonCategories) {
    auto it = catalog.category_counts.find(category);
    counts[hacdcpf::analysis::to_string(category)] = it == catalog.category_counts.end() ? 0 : it->second;
  }
  return counts;
}

bool configure_typhoon_catalog_sample(const json& j,
                                      hacdcpf::analysis::TyphoonScenarioOptions& typhoon_opts,
                                      json* response_metadata = nullptr) {
  if (!j.contains("intensity_category")) return false;
  const auto requested = hacdcpf::analysis::typhoon_intensity_category_from_string(
      j.value("intensity_category", std::string{"TY"}));
  if (requested == hacdcpf::analysis::TyphoonIntensityCategory::Unknown) {
    throw std::runtime_error("Unknown typhoon intensity_category: " + j.value("intensity_category", std::string{}));
  }

  hacdcpf::analysis::TyphoonCatalogOptions catalog_opts;
  catalog_opts.samples_per_month = j.value("catalog_samples_per_month", catalog_opts.samples_per_month);
  catalog_opts.first_month = j.value("catalog_first_month", catalog_opts.first_month);
  catalog_opts.last_month = j.value("catalog_last_month", catalog_opts.last_month);
  catalog_opts.base_seed = j.value("catalog_base_seed", catalog_opts.base_seed);
  catalog_opts.horizon_hours = typhoon_opts.horizon_hours;
  catalog_opts.time_step_hr = typhoon_opts.time_step_hr;
  catalog_opts.stochastic = true;
  catalog_opts.use_month_defaults = true;
  catalog_opts.use_sst_resource = true;
  catalog_opts.sst_resource_path = typhoon_opts.sst_resource_path;
  catalog_opts.catalog_path = j.value("catalog_path", catalog_opts.catalog_path);
  const auto& catalog = hacdcpf::analysis::get_or_build_typhoon_catalog(catalog_opts);

  hacdcpf::analysis::TyphoonIntensityCategory selected = requested;
  bool used_fallback = false;
  const auto* sample = sample_typhoon_category_with_fallback(catalog, requested, typhoon_opts.seed, selected, used_fallback);
  if (!sample) throw std::runtime_error("Typhoon catalog is empty; cannot sample intensity category");

  typhoon_opts.use_precomputed_track = true;
  typhoon_opts.precomputed_track = sample->track;
  typhoon_opts.month = sample->month;
  typhoon_opts.seed = sample->seed;
  typhoon_opts.stochastic = sample->stochastic;
  typhoon_opts.requested_category = requested;
  typhoon_opts.selected_category = selected;
  typhoon_opts.selected_track_max_vmax_ms = sample->max_vmax_ms;
  typhoon_opts.selected_sample_id = sample->sample_id;
  typhoon_opts.used_category_fallback = used_fallback;
  typhoon_opts.sst_resource_path = sample->sst_resource_path;

  if (response_metadata) {
    (*response_metadata)["requested_intensity_category"] = hacdcpf::analysis::to_string(requested);
    (*response_metadata)["requested_intensity_category_zh"] = hacdcpf::analysis::to_zh_name(requested);
    (*response_metadata)["selected_intensity_category"] = hacdcpf::analysis::to_string(selected);
    (*response_metadata)["selected_intensity_category_zh"] = hacdcpf::analysis::to_zh_name(selected);
    (*response_metadata)["selected_track_max_vmax_ms"] = sample->max_vmax_ms;
    (*response_metadata)["selected_sample_id"] = sample->sample_id;
    (*response_metadata)["used_catalog_sample"] = true;
    (*response_metadata)["used_category_fallback"] = used_fallback;
    (*response_metadata)["catalog_counts"] = typhoon_catalog_counts_json(catalog);
    (*response_metadata)["catalog_sample_count"] = catalog.samples.size();
    (*response_metadata)["catalog_path"] = catalog.source_path;
    (*response_metadata)["catalog_loaded_from_disk"] = catalog.loaded_from_disk;
  }
  return true;
}

void append_typhoon_result_metadata(json& target,
                                    const hacdcpf::analysis::TyphoonFaultSequenceResult& result) {
  target["requested_intensity_category"] = hacdcpf::analysis::to_string(result.requested_category);
  target["requested_intensity_category_zh"] = hacdcpf::analysis::to_zh_name(result.requested_category);
  target["selected_intensity_category"] = hacdcpf::analysis::to_string(result.selected_category);
  target["selected_intensity_category_zh"] = hacdcpf::analysis::to_zh_name(result.selected_category);
  target["selected_track_max_vmax_ms"] = result.selected_track_max_vmax_ms;
  target["selected_sample_id"] = result.selected_sample_id;
  target["used_catalog_sample"] = result.used_catalog_sample;
  target["used_category_fallback"] = result.used_category_fallback;
}

struct Args {
  std::string host{"127.0.0.1"};
  int port{8088};
  std::string data_dir{"../data"};
  std::string matpower_dir{"../external_data/matpower"};
};

void usage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " [--host <host>] [--port <port>] [--data-dir <path>] [--matpower-dir <path>]\n";
}

bool parse_args(int argc, char** argv, Args& args) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k == "--host") {
      if (i + 1 >= argc) return false;
      args.host = argv[++i];
    } else if (k == "--port") {
      if (i + 1 >= argc) return false;
      args.port = std::atoi(argv[++i]);
      if (args.port <= 0) return false;
    } else if (k == "--data-dir") {
      if (i + 1 >= argc) return false;
      args.data_dir = argv[++i];
    } else if (k == "--matpower-dir") {
      if (i + 1 >= argc) return false;
      args.matpower_dir = argv[++i];
    } else if (k == "--help" || k == "-h") {
      usage(argv[0]);
      return false;
    } else {
      std::cerr << "Unknown arg: " << k << "\n";
      return false;
    }
  }
  return true;
}

// ---- global session state ----
struct Session {
  std::mutex mu;
  std::optional<hacdcpf::HybridPowerSystem> current_system;
  std::string current_name{"(none)"};
  hacdcpf::TimeSeriesData ts_data;     // time-series profile store
  std::atomic<bool> busy{false};       // true while a heavy computation runs
  std::atomic<bool> cancel{false};     // set by cancel endpoint
  // Last power flow result for carbon analysis reuse
  std::optional<hacdcpf::PowerFlowResult> last_pf_result;
  std::string last_pf_method;
  std::optional<hacdcpf::TimeSeriesPFResult> last_tspf_result;
  hacdcpf::TimeSeriesData last_tspf_data;
  bool last_tspf_skip_uc{true};
  bool last_tspf_run_opf{false};
  int last_tspf_num_steps{0};
  // Persistent time-series binding spec — re-applied at every run_ts_pf call.
  // Survives system replacement (e.g. canvas resync via load_json_string)
  // so per-load profile mappings are not lost between set_ts_config and run.
  struct TsBindingSpec {
    bool valid = false;
    std::vector<std::pair<int,int>> map_by_load_index; // (load_index, profile_id)
    std::vector<std::pair<int,int>> map_by_bus;        // (bus, profile_id)
    int assign_all_loads_to = -1;
    int assign_all_pv_to = -1;
  };
  TsBindingSpec ts_binding;
  struct ExternalGridCarbonProfile {
    int index{0};
    int bus{0};
    std::string name;
    std::vector<double> values_tco2_mwh;
  };
  std::vector<ExternalGridCarbonProfile> external_grid_carbon_profiles;
};
Session g_session;

void clear_cached_analysis(Session& s) {
  s.last_pf_result.reset();
  s.last_pf_method.clear();
  s.last_tspf_result.reset();
  s.last_tspf_data = hacdcpf::TimeSeriesData{};
  s.last_tspf_skip_uc = true;
  s.last_tspf_run_opf = false;
  s.last_tspf_num_steps = 0;
}

void apply_current_carbon_factors_to_snapshot(
    const hacdcpf::HybridPowerSystem& current,
    hacdcpf::HybridPowerSystem& snapshot) {
  auto match = [](int index, int bus, const std::string& name,
                  int ref_index, int ref_bus, const std::string& ref_name) {
    return index == ref_index ||
           bus == ref_bus ||
           (!name.empty() && name == ref_name);
  };

  for (auto& g : snapshot.ac.generators) {
    for (const auto& src : current.ac.generators) {
      if (!match(g.index, g.bus, g.name, src.index, src.bus, src.name)) continue;
      g.emission_factor_tco2_mwh = src.emission_factor_tco2_mwh;
      break;
    }
  }
  for (auto& sg : snapshot.ac.static_generators) {
    for (const auto& src : current.ac.static_generators) {
      if (!match(sg.index, sg.bus, sg.name, src.index, src.bus, src.name)) continue;
      sg.co2_emission_rate = src.co2_emission_rate;
      break;
    }
  }
  for (auto& eg : snapshot.ac.external_grids) {
    for (const auto& src : current.ac.external_grids) {
      if (!match(eg.index, eg.bus, eg.name, src.index, src.bus, src.name)) continue;
      eg.emission_factor_tco2_mwh = src.emission_factor_tco2_mwh;
      break;
    }
  }
}

void apply_current_carbon_factors_to_tspf(
    const hacdcpf::HybridPowerSystem& current,
    hacdcpf::TimeSeriesPFResult& ts_result) {
  for (auto& snapshot : ts_result.pf_system_snapshots) {
    apply_current_carbon_factors_to_snapshot(current, snapshot);
  }
}

double carbon_factor_scale_from_unit(const std::string& unit) {
  std::string u = unit;
  std::transform(u.begin(), u.end(), u.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (u.find("kg") != std::string::npos && u.find("mwh") != std::string::npos) return 0.001;
  if (u.find("kg") != std::string::npos && u.find("kwh") != std::string::npos) return 1.0;
  return 1.0;
}

double carbon_factor_scale_from_json(const json& root, const json& row) {
  const json* unit = nullptr;
  if (row.contains("unit") && row["unit"].is_string()) unit = &row["unit"];
  else if (row.contains("units") && row["units"].is_string()) unit = &row["units"];
  else if (root.contains("unit") && root["unit"].is_string()) unit = &root["unit"];
  else if (root.contains("units") && root["units"].is_string()) unit = &root["units"];
  return unit ? carbon_factor_scale_from_unit(unit->get<std::string>()) : 1.0;
}

double json_emission_factor_value(const json& row, double scale = 1.0,
                                  double default_value = 0.0) {
  auto read = [default_value](const json& value) {
    if (value.is_number()) return value.get<double>();
    if (value.is_string()) {
      try { return std::stod(value.get<std::string>()); } catch (...) {}
    }
    return default_value;
  };
  if (row.contains("emission_factor_tco2_mwh")) return read(row["emission_factor_tco2_mwh"]);
  if (row.contains("co2_emission_rate")) return read(row["co2_emission_rate"]) * scale;
  if (row.contains("emission_factor")) return read(row["emission_factor"]) * scale;
  return default_value;
}

std::optional<std::vector<double>> json_emission_factor_profile(const json& row,
                                                                double scale = 1.0) {
  const json* arr = nullptr;
  bool already_tco2_mwh = false;
  if (row.contains("emission_factor_profile_tco2_mwh")) {
    arr = &row["emission_factor_profile_tco2_mwh"];
    already_tco2_mwh = true;
  } else if (row.contains("emission_factor_tco2_mwh_profile")) {
    arr = &row["emission_factor_tco2_mwh_profile"];
    already_tco2_mwh = true;
  } else if (row.contains("co2_emission_rate_profile")) {
    arr = &row["co2_emission_rate_profile"];
  } else if (row.contains("emission_factor_profile")) {
    arr = &row["emission_factor_profile"];
  }
  if (arr == nullptr || !arr->is_array()) return std::nullopt;

  std::vector<double> values;
  values.reserve(arr->size());
  for (const auto& v : *arr) {
    try {
      double raw = 0.0;
      if (v.is_number()) raw = v.get<double>();
      else if (v.is_string()) raw = std::stod(v.get<std::string>());
      else continue;
      values.push_back(raw * (already_tco2_mwh ? 1.0 : scale));
    } catch (...) {
    }
  }
  if (values.empty()) return std::nullopt;
  return values;
}

json emissions_summary_to_json(const hacdcpf::analysis::EmissionsSummary& s) {
  return json{{"total_generation_emissions_tco2", s.total_generation_emissions_tco2},
              {"total_load_emissions_tco2", s.total_load_emissions_tco2},
              {"total_loss_emissions_tco2", s.total_loss_emissions_tco2},
              {"balance_error_tco2", s.balance_error_tco2},
              {"balance_error_pct", s.balance_error_pct}};
}

json carbon_analysis_to_json(const hacdcpf::HybridPowerSystem& sys,
                             const hacdcpf::analysis::CarbonAnalysisResult& carbon) {
  auto storage_canvas_type = [&](const hacdcpf::analysis::StorageCarbonResult& s) {
    return s.is_dc ? "dc_storage" : "storage";
  };
  auto storage_source_type = [&](const hacdcpf::analysis::StorageCarbonResult& s) {
    if (!s.is_dc) return "storage";
    for (const auto& st : sys.dc.dc_storage) {
      if (st.index == s.storage_index) return "dc_storage";
    }
    return "storage";
  };
  auto load_display_label = [](const hacdcpf::analysis::LoadCarbonResult& l,
                               bool is_dc) {
    if (l.load_index >= 0) return std::to_string(l.load_index);
    const char* prefix = is_dc ? "DC Bus Load @" : "Bus Load @";
    return std::string(prefix) + std::to_string(l.bus);
  };

  auto ac_bus_sink_power = [&](int bus) {
    double p = 0.0;
    for (const auto& l : sys.ac.loads) {
      if (l.in_service && l.bus == bus)
        p += std::max(0.0, l.p_mw * l.scaling);
    }
    for (const auto& l : sys.ac.flexible_loads) {
      if (l.in_service && l.bus == bus)
        p += std::max(0.0, l.p_mw);
    }
    for (const auto& l : sys.ac.asymmetric_loads) {
      if (l.in_service && l.bus == bus)
        p += std::max(0.0, (l.pa_mw + l.pb_mw + l.pc_mw) * l.scaling);
    }
    for (const auto& l : sys.ac.charging_stations) {
      if (l.in_service && l.bus == bus)
        p += std::max(0.0, l.p_total_kw / 1000.0);
    }
    for (const auto& st : sys.ac.storage) {
      if (st.in_service && st.bus == bus && st.p_mw < 0.0)
        p += -st.p_mw;
    }
    return p;
  };
  auto dc_bus_sink_power = [&](int bus) {
    double p = 0.0;
    for (const auto& l : sys.dc.loads) {
      if (l.in_service && l.bus == bus)
        p += std::max(0.0, l.p_mw * l.scaling);
    }
    for (const auto& st : sys.dc.storage) {
      if (st.in_service && st.bus == bus && st.p_mw < 0.0)
        p += -st.p_mw;
    }
    return p;
  };

  json out;
  out["matrix_solved"] = carbon.matrix_solved;
  out["tracing_verified"] = carbon.tracing_verified;
  out["matrix_residual"] = carbon.matrix_residual;
  out["tracing_summary"] = emissions_summary_to_json(carbon.tracing_summary);
  out["matrix_summary"] = emissions_summary_to_json(carbon.matrix_summary);
  out["total_storage_charge_emissions_tco2"] =
      carbon.total_storage_charge_emissions_tco2;
  out["total_storage_discharge_emissions_tco2"] =
      carbon.total_storage_discharge_emissions_tco2;

  json bc = json::array();
  for (const auto& b : carbon.bus_carbon) {
    const double sink_power_mw = ac_bus_sink_power(b.bus_index);
    const bool has_sink = sink_power_mw > 1e-9;
    const bool sane_potential =
        std::isfinite(b.carbon_intensity_tco2_mwh) &&
        b.carbon_intensity_tco2_mwh >= 0.0 &&
        b.carbon_intensity_tco2_mwh <= 10.0;
    bc.push_back({{"bus_index", b.bus_index},
                  {"carbon_intensity_tco2_mwh", b.carbon_intensity_tco2_mwh},
                  {"sink_power_mw", sink_power_mw},
                  {"has_carbon_sink", has_sink},
                  {"carbon_potential_valid", sane_potential},
                  {"carbon_potential_outlier", !sane_potential}});
  }
  out["bus_carbon"] = bc;

  json dbc = json::array();
  for (const auto& b : carbon.dc_bus_carbon) {
    const double sink_power_mw = dc_bus_sink_power(b.bus_index);
    const bool has_sink = sink_power_mw > 1e-9;
    const bool sane_potential =
        std::isfinite(b.carbon_intensity_tco2_mwh) &&
        b.carbon_intensity_tco2_mwh >= 0.0 &&
        b.carbon_intensity_tco2_mwh <= 10.0;
    dbc.push_back({{"bus_index", b.bus_index},
                   {"carbon_intensity_tco2_mwh", b.carbon_intensity_tco2_mwh},
                   {"sink_power_mw", sink_power_mw},
                   {"has_carbon_sink", has_sink},
                   {"carbon_potential_valid", sane_potential},
                   {"carbon_potential_outlier", !sane_potential}});
  }
  out["dc_bus_carbon"] = dbc;

  json lc = json::array();
  for (const auto& l : carbon.load_carbon)
    lc.push_back({{"load_index", l.load_index},
                  {"display_load", load_display_label(l, false)},
                  {"bus", l.bus},
                  {"demand_mw", l.demand_mw},
                  {"display_demand_mw", std::abs(l.demand_mw)},
                  {"carbon_intensity_tco2_mwh", l.carbon_intensity_tco2_mwh},
                  {"total_emissions_tco2", l.total_emissions_tco2}});
  out["load_carbon"] = lc;

  json dlc = json::array();
  for (const auto& l : carbon.dc_load_carbon)
    dlc.push_back({{"load_index", l.load_index},
                   {"display_load", load_display_label(l, true)},
                   {"bus", l.bus},
                   {"demand_mw", l.demand_mw},
                   {"display_demand_mw", std::abs(l.demand_mw)},
                   {"carbon_intensity_tco2_mwh", l.carbon_intensity_tco2_mwh},
                   {"total_emissions_tco2", l.total_emissions_tco2}});
  out["dc_load_carbon"] = dlc;

  json brcc = json::array();
  for (const auto& b : carbon.branch_carbon)
    brcc.push_back({{"branch_index", b.branch_index},
                    {"from_bus", b.from_bus},
                    {"to_bus", b.to_bus},
                    {"loss_mw", b.loss_mw},
                    {"carbon_intensity_tco2_mwh", b.carbon_intensity_tco2_mwh},
                    {"total_emissions_tco2", b.total_emissions_tco2}});
  out["branch_carbon"] = brcc;

  json dbrcc = json::array();
  for (const auto& b : carbon.dc_branch_carbon)
    dbrcc.push_back({{"branch_index", b.branch_index},
                     {"from_bus", b.from_bus},
                     {"to_bus", b.to_bus},
                     {"loss_mw", b.loss_mw},
                     {"carbon_intensity_tco2_mwh", b.carbon_intensity_tco2_mwh},
                     {"total_emissions_tco2", b.total_emissions_tco2}});
  out["dc_branch_carbon"] = dbrcc;

  json vcc = json::array();
  for (const auto& v : carbon.vsc_carbon)
    vcc.push_back({{"converter_index", v.converter_index},
                   {"bus_ac", v.bus_ac},
                   {"bus_dc", v.bus_dc},
                   {"loss_mw", v.loss_mw},
                   {"total_emissions_tco2", v.total_emissions_tco2}});
  out["vsc_carbon"] = vcc;

  json dcc = json::array();
  for (const auto& d : carbon.dcdc_carbon)
    dcc.push_back({{"converter_index", d.converter_index},
                   {"bus_in", d.bus_in},
                   {"bus_out", d.bus_out},
                   {"loss_mw", d.loss_mw},
                   {"total_emissions_tco2", d.total_emissions_tco2}});
  out["dcdc_carbon"] = dcc;

  json sc = json::array();
  for (const auto& s : carbon.storage_carbon)
    sc.push_back({{"storage_index", s.storage_index},
                  {"bus", s.bus},
                  {"is_dc", s.is_dc},
                  {"canvas_type", storage_canvas_type(s)},
                  {"canvas_index", s.storage_index},
                  {"source_type", storage_source_type(s)},
                  {"p_mw", s.p_mw},
                  {"soc", s.soc},
                  {"stored_energy_mwh", s.stored_energy_mwh},
                  {"soc_carbon_intensity_tco2_mwh", s.soc_carbon_intensity_tco2_mwh},
                  {"carbon_intensity_tco2_mwh", s.carbon_intensity_tco2_mwh},
                  {"total_emissions_tco2", s.total_emissions_tco2}});
  out["storage_carbon"] = sc;

  json sk_labels = json::array(), sk_src = json::array(), sk_tgt = json::array(), sk_val = json::array();
  int src_idx = 0;
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    sk_labels.push_back(g.name.empty() ? "Gen" + std::to_string(g.index) : g.name);
    src_idx++;
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    sk_labels.push_back(eg.name.empty() ? "Grid" + std::to_string(eg.index) : eg.name);
    src_idx++;
  }
  for (const auto& sg : sys.ac.static_generators) {
    if (!sg.in_service) continue;
    sk_labels.push_back(sg.name.empty() ? "Sgen" + std::to_string(sg.index) : sg.name);
    src_idx++;
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    if (!rg.in_service || rg.p_mw <= 1e-6) continue;
    sk_labels.push_back(rg.name.empty() ? "Ren" + std::to_string(rg.index) : rg.name);
    src_idx++;
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service || pv.p_mw <= 1e-6) continue;
    sk_labels.push_back(pv.name.empty() ? "PV" + std::to_string(pv.index) : pv.name);
    src_idx++;
  }
  for (const auto& sg : sys.dc.static_generators) {
    if (!sg.in_service) continue;
    sk_labels.push_back(sg.name.empty() ? "DCSgen" + std::to_string(sg.index) : sg.name);
    src_idx++;
  }
  for (const auto& sg : sys.dc.dc_static_generators) {
    if (!sg.in_service) continue;
    sk_labels.push_back(sg.name.empty() ? "DCGen" + std::to_string(sg.index) : sg.name);
    src_idx++;
  }
  for (const auto& pv : sys.dc.pv_arrays) {
    if (!pv.in_service || pv.p_set_mw <= 1e-6) continue;
    sk_labels.push_back(pv.name.empty() ? "DCPV" + std::to_string(pv.index) : pv.name);
    src_idx++;
  }
  for (const auto& st : sys.ac.storage) {
    if (!st.in_service || st.p_mw <= 1e-6) continue;
    sk_labels.push_back(st.name.empty() ? "BESS" + std::to_string(st.index) : st.name);
    src_idx++;
  }
  for (const auto& st : sys.dc.storage) {
    if (!st.in_service || st.p_mw <= 1e-6) continue;
    sk_labels.push_back(st.name.empty() ? "DCBESS" + std::to_string(st.index) : st.name);
    src_idx++;
  }

  const int n_sources = src_idx;
  for (const auto& l : carbon.load_carbon) sk_labels.push_back("Load@Bus" + std::to_string(l.bus));
  const int n_ac_loads = static_cast<int>(carbon.load_carbon.size());
  for (const auto& l : carbon.dc_load_carbon) sk_labels.push_back("DCLoad@Bus" + std::to_string(l.bus));
  for (int li = 0; li < static_cast<int>(carbon.load_carbon.size()); ++li) {
    for (const auto& [gi, smw] : carbon.load_carbon.at(static_cast<size_t>(li)).generator_supply_mw) {
      if (smw < 1e-3 || gi < 0 || gi >= n_sources) continue;
      sk_src.push_back(gi); sk_tgt.push_back(n_sources + li); sk_val.push_back(smw);
    }
  }
  for (int li = 0; li < static_cast<int>(carbon.dc_load_carbon.size()); ++li) {
    for (const auto& [gi, smw] : carbon.dc_load_carbon.at(static_cast<size_t>(li)).generator_supply_mw) {
      if (smw < 1e-3 || gi < 0 || gi >= n_sources) continue;
      sk_src.push_back(gi); sk_tgt.push_back(n_sources + n_ac_loads + li); sk_val.push_back(smw);
    }
  }
  out["sankey_labels"] = sk_labels;
  out["sankey_sources"] = sk_src;
  out["sankey_targets"] = sk_tgt;
  out["sankey_values"] = sk_val;

  return out;
}

json annual_carbon_to_json(const hacdcpf::analysis::AnnualCarbonAnalysisResult& annual) {
  json out;
  out["num_steps"] = annual.num_steps;
  out["step_duration_hr"] = annual.step_duration_hr;
  out["num_pf_converged"] = annual.num_pf_converged;
  out["total_generation_emissions_tco2"] = annual.total_generation_emissions_tco2;
  out["total_load_emissions_tco2"] = annual.total_load_emissions_tco2;
  out["total_loss_emissions_tco2"] = annual.total_loss_emissions_tco2;
  out["total_storage_charge_emissions_tco2"] =
      annual.total_storage_charge_emissions_tco2;
  out["total_storage_discharge_emissions_tco2"] =
      annual.total_storage_discharge_emissions_tco2;
  const double balance_basic =
      annual.total_generation_emissions_tco2 -
      annual.total_load_emissions_tco2 -
      annual.total_loss_emissions_tco2;
  const double balance_storage_adjusted =
      balance_basic -
      annual.total_storage_charge_emissions_tco2 +
      annual.total_storage_discharge_emissions_tco2;
  const double storage_inventory_delta =
      annual.total_storage_charge_emissions_tco2 -
      annual.total_storage_discharge_emissions_tco2;
  out["balance_error_tco2"] = balance_basic;
  out["balance_error_storage_adjusted_tco2"] = balance_storage_adjusted;
  out["storage_carbon_inventory_delta_tco2"] = storage_inventory_delta;
  out["balance_error_pct"] =
      std::abs(balance_basic) /
      std::max(std::abs(annual.total_generation_emissions_tco2), 1e-12) * 100.0;
  out["balance_error_storage_adjusted_pct"] =
      std::abs(balance_storage_adjusted) /
      std::max(std::abs(annual.total_generation_emissions_tco2), 1e-12) * 100.0;

  json steps = json::array();
  for (size_t i = 0; i < annual.step_results.size(); ++i) {
    const auto& s = annual.step_results[i];
    const double step_basic =
        s.total_generation_emissions_tco2 -
        s.total_load_emissions_tco2 -
        s.total_loss_emissions_tco2;
    const double step_storage_adjusted =
        step_basic -
        s.total_storage_charge_emissions_tco2 +
        s.total_storage_discharge_emissions_tco2;
    const double step_storage_inventory_delta =
        s.total_storage_charge_emissions_tco2 -
        s.total_storage_discharge_emissions_tco2;
    steps.push_back({{"step", static_cast<int>(i)},
                     {"pf_converged", s.pf_converged},
                     {"total_generation_emissions_tco2", s.total_generation_emissions_tco2},
                     {"total_load_emissions_tco2", s.total_load_emissions_tco2},
                     {"total_loss_emissions_tco2", s.total_loss_emissions_tco2},
                     {"total_storage_charge_emissions_tco2",
                      s.total_storage_charge_emissions_tco2},
                     {"total_storage_discharge_emissions_tco2",
                      s.total_storage_discharge_emissions_tco2},
                     {"balance_error_tco2", step_basic},
                     {"balance_error_storage_adjusted_tco2", step_storage_adjusted},
                     {"storage_carbon_inventory_delta_tco2", step_storage_inventory_delta}});
  }
  out["step_results"] = steps;

  json buses = json::array();
  for (const auto& b : annual.bus_stats)
    buses.push_back({{"bus_index", b.bus_index},
                     {"is_dc", b.is_dc},
                     {"energy_mwh", b.energy_mwh},
                     {"emissions_tco2", b.emissions_tco2},
                     {"average_intensity_tco2_mwh", b.average_intensity_tco2_mwh},
                     {"min_intensity_tco2_mwh", b.min_intensity_tco2_mwh},
                     {"max_intensity_tco2_mwh", b.max_intensity_tco2_mwh}});
  out["bus_stats"] = buses;

  json loads = json::array();
  for (const auto& l : annual.load_stats)
    loads.push_back({{"load_index", l.load_index},
                     {"bus", l.bus},
                     {"is_dc", l.is_dc},
                     {"energy_mwh", l.energy_mwh},
                     {"emissions_tco2", l.emissions_tco2},
                     {"average_intensity_tco2_mwh", l.average_intensity_tco2_mwh}});
  out["load_stats"] = loads;

  json storage = json::array();
  for (const auto& st : annual.terminal_storage_states)
    storage.push_back({{"storage_index", st.storage_index},
                       {"bus", st.bus},
                       {"is_dc", st.is_dc},
                       {"soc", st.soc},
                       {"stored_energy_mwh", st.stored_energy_mwh},
                       {"soc_carbon_intensity_tco2_mwh", st.soc_carbon_intensity_tco2_mwh}});
  out["terminal_storage_states"] = storage;

  out["hourly_bus_intensity_tco2_mwh"] = annual.hourly_bus_intensity_tco2_mwh;
  out["hourly_load_emissions_tco2"] = annual.hourly_load_emissions_tco2;
  out["hourly_load_energy_mwh"] = annual.hourly_load_energy_mwh;
  return out;
}

struct FMEAComponentPresentation {
  std::string display_name;
  std::string display_type;
  std::string canvas_type;
  int canvas_index{-1};
  int primary_bus{0};
  int secondary_bus{0};
  bool mappable{false};
};

std::string fmea_type_label(const std::string& type) {
  static const std::unordered_map<std::string, std::string> labels{
      {"generator", "发电机"},
      {"ac_branch", "AC线路"},
      {"dc_branch", "DC线路"},
      {"vsc_converter", "VSC换流器"},
      {"static_generator", "静态电源"},
      {"renewable_gen", "新能源电源"},
      {"storage", "储能"},
      {"transformer_2w", "双绕组变压器"},
      {"transformer_3w", "三绕组变压器"},
      {"dcdc_converter", "DC/DC变换器"},
      {"dc_circuit_breaker", "DC断路器"},
      {"dc_storage", "DC储能"},
      {"dc_pv_array", "DC光伏"},
      {"dc_static_generator", "DC静态电源"},
      {"ac_switch", "AC开关"},
      {"ac_circuit_breaker", "AC断路器"},
      {"ac_pv_system", "AC光伏"},
      {"dc_static_generator_ac", "DC分布式电源"},
  };
  const auto it = labels.find(type);
  return it == labels.end() ? type : it->second;
}

std::string fmea_default_name(const std::string& display_type, int index) {
  return display_type + " #" + std::to_string(index);
}

std::string fmea_branch_name(const std::string& display_type, int index, int from_bus, int to_bus) {
  return display_type + " #" + std::to_string(index) + " (" +
         std::to_string(from_bus) + " -> " + std::to_string(to_bus) + ")";
}

template <typename T>
std::string explicit_or_default_name(const T& item, const std::string& display_type) {
  return item.name.empty() ? fmea_default_name(display_type, item.index) : item.name;
}

template <typename T>
FMEAComponentPresentation fmea_one_bus_component(const std::vector<T>& items,
                                                 int position,
                                                 const std::string& display_type,
                                                 const std::string& canvas_type) {
  FMEAComponentPresentation p;
  p.display_type = display_type;
  p.canvas_type = canvas_type;
  if (position < 0 || position >= static_cast<int>(items.size())) {
    p.display_name = fmea_default_name(display_type, position);
    return p;
  }
  const auto& item = items[static_cast<size_t>(position)];
  p.canvas_index = item.index;
  p.primary_bus = item.bus;
  p.display_name = explicit_or_default_name(item, display_type);
  p.mappable = !canvas_type.empty();
  return p;
}

FMEAComponentPresentation describe_fmea_component(const hacdcpf::HybridPowerSystem& sys,
                                                  const std::string& type,
                                                  int position) {
  const std::string label = fmea_type_label(type);
  FMEAComponentPresentation p;
  p.display_type = label;
  p.display_name = fmea_default_name(label, position);

  if (type == "generator")
    return fmea_one_bus_component(sys.ac.generators, position, label, "gen");
  if (type == "static_generator")
    return fmea_one_bus_component(sys.ac.static_generators, position, label, "sgen");
  if (type == "renewable_gen")
    return fmea_one_bus_component(sys.ac.renewable_gens, position, label, "renGen");
  if (type == "storage")
    return fmea_one_bus_component(sys.ac.storage, position, label, "storage");
  if (type == "ac_pv_system")
    return fmea_one_bus_component(sys.ac.pv_systems, position, label, "pv");
  if (type == "dc_storage")
    return fmea_one_bus_component(sys.dc.storage, position, label, "dcStorage");
  if (type == "dc_pv_array")
    return fmea_one_bus_component(sys.dc.pv_arrays, position, label, "dcPv");
  if (type == "dc_static_generator_ac")
    return fmea_one_bus_component(sys.dc.static_generators, position, label, "dcSgen");
  if (type == "dc_static_generator")
    return fmea_one_bus_component(sys.dc.dc_static_generators, position, label, "");

  auto missing = [&] {
    p.canvas_index = -1;
    p.mappable = false;
    return p;
  };

  if (type == "ac_branch") {
    if (position < 0 || position >= static_cast<int>(sys.ac.branches.size())) return missing();
    const auto& br = sys.ac.branches[static_cast<size_t>(position)];
    p.canvas_type = "branch";
    p.canvas_index = br.index;
    p.primary_bus = br.from_bus;
    p.secondary_bus = br.to_bus;
    p.display_name = br.name.empty() ? fmea_branch_name(label, br.index, br.from_bus, br.to_bus) : br.name;
    p.mappable = true;
    return p;
  }
  if (type == "dc_branch") {
    if (position < 0 || position >= static_cast<int>(sys.dc.branches.size())) return missing();
    const auto& br = sys.dc.branches[static_cast<size_t>(position)];
    p.canvas_type = "dcBranch";
    p.canvas_index = br.index;
    p.primary_bus = br.from_bus;
    p.secondary_bus = br.to_bus;
    p.display_name = br.name.empty() ? fmea_branch_name(label, br.index, br.from_bus, br.to_bus) : br.name;
    p.mappable = true;
    return p;
  }
  if (type == "transformer_2w") {
    if (position < 0 || position >= static_cast<int>(sys.ac.transformers_2w.size())) return missing();
    const auto& tr = sys.ac.transformers_2w[static_cast<size_t>(position)];
    p.canvas_type = "trafo";
    p.canvas_index = tr.index;
    p.primary_bus = tr.hv_bus;
    p.secondary_bus = tr.lv_bus;
    p.display_name = tr.name.empty() ? fmea_branch_name(label, tr.index, tr.hv_bus, tr.lv_bus) : tr.name;
    p.mappable = true;
    return p;
  }
  if (type == "transformer_3w") {
    if (position < 0 || position >= static_cast<int>(sys.ac.transformers_3w.size())) return missing();
    const auto& tr = sys.ac.transformers_3w[static_cast<size_t>(position)];
    p.canvas_type = "trafo3w";
    p.canvas_index = tr.index;
    p.primary_bus = tr.hv_bus;
    p.secondary_bus = tr.lv_bus;
    p.display_name = tr.name.empty()
        ? fmea_default_name(label, tr.index) + " (" + std::to_string(tr.hv_bus) + " / " +
              std::to_string(tr.mv_bus) + " / " + std::to_string(tr.lv_bus) + ")"
        : tr.name;
    p.mappable = true;
    return p;
  }
  if (type == "vsc_converter") {
    if (position < 0 || position >= static_cast<int>(sys.vsc_converters.size())) return missing();
    const auto& v = sys.vsc_converters[static_cast<size_t>(position)];
    p.canvas_type = "vsc";
    p.canvas_index = v.index;
    p.primary_bus = v.bus_ac;
    p.secondary_bus = v.bus_dc;
    p.display_name = v.name.empty()
        ? label + " #" + std::to_string(v.index) + " (AC " + std::to_string(v.bus_ac) +
              " -> DC " + std::to_string(v.bus_dc) + ")"
        : v.name;
    p.mappable = true;
    return p;
  }
  if (type == "dcdc_converter") {
    if (position < 0 || position >= static_cast<int>(sys.dc.dcdc_converters.size())) return missing();
    const auto& dc = sys.dc.dcdc_converters[static_cast<size_t>(position)];
    p.canvas_type = "dcdcConverter";
    p.canvas_index = dc.index;
    p.primary_bus = dc.bus_in;
    p.secondary_bus = dc.bus_out;
    p.display_name = dc.name.empty()
        ? label + " #" + std::to_string(dc.index) + " (DC " + std::to_string(dc.bus_in) +
              " -> DC " + std::to_string(dc.bus_out) + ")"
        : dc.name;
    p.mappable = true;
    return p;
  }
  if (type == "ac_switch") {
    if (position < 0 || position >= static_cast<int>(sys.ac.switches.size())) return missing();
    const auto& sw = sys.ac.switches[static_cast<size_t>(position)];
    p.canvas_type = "sw";
    p.canvas_index = sw.index;
    p.primary_bus = sw.bus_from;
    p.secondary_bus = sw.bus_to;
    p.display_name = sw.name.empty() ? fmea_branch_name(label, sw.index, sw.bus_from, sw.bus_to) : sw.name;
    p.mappable = true;
    return p;
  }
  if (type == "ac_circuit_breaker") {
    if (position < 0 || position >= static_cast<int>(sys.ac.circuit_breakers.size())) return missing();
    const auto& cb = sys.ac.circuit_breakers[static_cast<size_t>(position)];
    p.canvas_type = "cb";
    p.canvas_index = cb.index;
    p.primary_bus = cb.bus_from;
    p.secondary_bus = cb.bus_to;
    p.display_name = cb.name.empty() ? fmea_branch_name(label, cb.index, cb.bus_from, cb.bus_to) : cb.name;
    p.mappable = true;
    return p;
  }
  if (type == "dc_circuit_breaker") {
    if (position < 0 || position >= static_cast<int>(sys.dc.dc_circuit_breakers.size())) return missing();
    const auto& cb = sys.dc.dc_circuit_breakers[static_cast<size_t>(position)];
    p.canvas_type = "dcCb";
    p.canvas_index = cb.index;
    p.primary_bus = cb.bus_from;
    p.secondary_bus = cb.bus_to;
    p.display_name = cb.name.empty() ? fmea_branch_name(label, cb.index, cb.bus_from, cb.bus_to) : cb.name;
    p.mappable = true;
    return p;
  }

  return p;
}

std::string opf_default_name(const std::string& display_type, int index) {
  return display_type + " #" + std::to_string(index);
}

std::string opf_branch_name(const std::string& display_type, int index, int from_bus, int to_bus) {
  return display_type + " #" + std::to_string(index) + " (" +
         std::to_string(from_bus) + " -> " + std::to_string(to_bus) + ")";
}

template <typename T>
std::string opf_component_name(const T& item, const std::string& display_type) {
  return item.name.empty() ? opf_default_name(display_type, item.index) : item.name;
}

std::vector<size_t> opf_active_vsc_positions(const hacdcpf::HybridPowerSystem& sys) {
  std::vector<size_t> positions;
  positions.reserve(sys.vsc_converters.size());
  std::unordered_set<int> ac_bus_ids, dc_bus_ids;
  ac_bus_ids.reserve(sys.ac.buses.size());
  dc_bus_ids.reserve(sys.dc.buses.size());
  for (const auto& b : sys.ac.buses) ac_bus_ids.insert(b.index);
  for (const auto& b : sys.dc.buses) dc_bus_ids.insert(b.index);
  for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
    const auto& c = sys.vsc_converters[i];
    if (!c.in_service) continue;
    if (!ac_bus_ids.count(c.bus_ac) || !dc_bus_ids.count(c.bus_dc)) continue;
    positions.push_back(i);
  }
  return positions;
}

std::vector<size_t> opf_active_dcdc_positions(const hacdcpf::HybridPowerSystem& sys) {
  std::vector<size_t> positions;
  positions.reserve(sys.dc.dcdc_converters.size());
  std::unordered_set<int> dc_bus_ids;
  dc_bus_ids.reserve(sys.dc.buses.size());
  for (const auto& b : sys.dc.buses) dc_bus_ids.insert(b.index);
  for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
    const auto& c = sys.dc.dcdc_converters[i];
    if (!c.in_service) continue;
    if (!dc_bus_ids.count(c.bus_in) || !dc_bus_ids.count(c.bus_out)) continue;
    positions.push_back(i);
  }
  return positions;
}

std::optional<int> opf_energy_router_index_from_name(const std::vector<hacdcpf::EnergyRouter>& routers,
                                                     const std::string& canonical_name) {
  if (canonical_name.empty()) return std::nullopt;
  std::optional<int> best;
  size_t best_len = 0;
  for (const auto& er : routers) {
    if (er.name.empty()) continue;
    if (canonical_name.rfind(er.name + "_", 0) == 0 && er.name.size() > best_len) {
      best = er.index;
      best_len = er.name.size();
    }
  }
  return best;
}

const hacdcpf::EnergyRouter* opf_find_energy_router(const hacdcpf::HybridPowerSystem& sys, int index) {
  for (const auto& er : sys.energy_routers) {
    if (er.index == index) return &er;
  }
  return nullptr;
}

template <typename T>
const T* opf_find_by_index(const std::vector<T>& items, int index) {
  for (const auto& item : items) {
    if (item.index == index) return &item;
  }
  return nullptr;
}

std::optional<int> opf_energy_router_port_bus_from_vsc_name(const hacdcpf::EnergyRouter& er,
                                                            const std::string& canonical_name) {
  const std::string prefix_a = er.name + "_VSC_A";
  const std::string prefix_b = er.name + "_VSC_B";
  std::string suffix;
  if (canonical_name.rfind(prefix_a, 0) == 0) {
    suffix = canonical_name.substr(prefix_a.size());
  } else if (canonical_name.rfind(prefix_b, 0) == 0) {
    suffix = canonical_name.substr(prefix_b.size());
  } else {
    return std::nullopt;
  }
  try {
    const int port_index = std::stoi(suffix);
    for (const auto& port : er.ports) {
      if (port.index == port_index) return port.bus;
    }
  } catch (const std::exception&) {
  }
  return std::nullopt;
}

void opf_apply_canonical_source(json& row,
                                const hacdcpf::HybridPowerSystem& original_sys,
                                const std::string& source_type,
                                int source_index) {
  if (source_type == "energy_router") {
    if (const auto* er = opf_find_energy_router(original_sys, source_index)) {
      row["source_type"] = source_type;
      row["source_index"] = source_index;
      row["source_name"] = er->name.empty() ? opf_default_name("能量路由器", er->index) : er->name;
      row["index"] = er->index;
      row["canvas_type"] = "energyRouter";
      row["canvas_index"] = er->index;
      row["name"] = row["source_name"];
    }
  }
}

json opf_ac_bus_results_json(const hacdcpf::HybridPowerSystem& sys,
                             const std::vector<double>& vm,
                             const std::vector<double>& va,
                             const std::vector<double>& lmp_p = {},
                             const std::vector<double>& lmp_q = {}) {
  json rows = json::array();
  const size_t n = std::max({sys.ac.buses.size(), vm.size(), va.size(), lmp_p.size(), lmp_q.size()});
  for (size_t i = 0; i < n; ++i) {
    json row{{"position", i}, {"canvas_type", "ac"}};
    if (i < sys.ac.buses.size()) {
      const auto& b = sys.ac.buses[i];
      row["index"] = b.index;
      row["name"] = b.name.empty() ? opf_default_name("AC Bus", b.index) : b.name;
      row["canvas_index"] = b.index;
    } else {
      row["index"] = nullptr;
      row["name"] = "AC Bus pos " + std::to_string(i);
      row["canvas_index"] = nullptr;
    }
    if (i < vm.size()) row["vm_pu"] = vm[i];
    if (i < va.size()) row["va_rad"] = va[i];
    if (i < lmp_p.size()) row["lmp_p"] = lmp_p[i];
    if (i < lmp_q.size()) row["lmp_q"] = lmp_q[i];
    rows.push_back(std::move(row));
  }
  return rows;
}

json opf_dc_bus_results_json(const hacdcpf::HybridPowerSystem& sys,
                             const std::vector<double>& vdc) {
  json rows = json::array();
  const size_t n = std::max(sys.dc.buses.size(), vdc.size());
  for (size_t i = 0; i < n; ++i) {
    json row{{"position", i}, {"canvas_type", "dc"}};
    if (i < sys.dc.buses.size()) {
      const auto& b = sys.dc.buses[i];
      row["index"] = b.index;
      row["name"] = b.name.empty() ? opf_default_name("DC Bus", b.index) : b.name;
      row["canvas_index"] = b.index;
    } else {
      row["index"] = nullptr;
      row["name"] = "DC Bus pos " + std::to_string(i);
      row["canvas_index"] = nullptr;
    }
    if (i < vdc.size()) row["vdc_pu"] = vdc[i];
    rows.push_back(std::move(row));
  }
  return rows;
}

json opf_generator_dispatch_json(const hacdcpf::HybridPowerSystem& sys,
                                 const std::vector<double>& pg,
                                 const std::vector<double>& qg = {}) {
  json rows = json::array();
  const size_t n = std::max(sys.ac.generators.size(), pg.size());
  for (size_t i = 0; i < n; ++i) {
    json row{{"position", i}, {"display_type", "发电机"}, {"canvas_type", "gen"}};
    if (i < sys.ac.generators.size()) {
      const auto& g = sys.ac.generators[i];
      row["index"] = g.index;
      row["name"] = opf_component_name(g, "发电机");
      row["bus"] = g.bus;
      row["canvas_index"] = g.index;
    } else {
      row["index"] = nullptr;
      row["name"] = "发电机 pos " + std::to_string(i);
      row["canvas_index"] = nullptr;
    }
    if (i < pg.size()) row["pg_mw"] = pg[i];
    if (i < qg.size()) row["qg_mvar"] = qg[i];
    rows.push_back(std::move(row));
  }
  return rows;
}

json opf_vsc_dispatch_json(const hacdcpf::HybridPowerSystem& sys,
                           const hacdcpf::HybridPowerSystem& original_sys,
                           const std::vector<double>& pac,
                           const std::vector<double>& qac) {
  json rows = json::array();
  const auto positions = opf_active_vsc_positions(sys);
  const size_t n = std::max(pac.size(), std::max(qac.size(), positions.size()));
  for (size_t k = 0; k < n; ++k) {
    json row{{"position", k}, {"display_type", "VSC换流器"}, {"canvas_type", "vsc"}};
    const bool has_pos = k < positions.size() && positions[k] < sys.vsc_converters.size();
    if (has_pos) {
      const size_t pos = positions[k];
      const auto& c = sys.vsc_converters[pos];
      const auto* original_vsc = opf_find_by_index(original_sys.vsc_converters, c.index);
      row["source_position"] = pos;
      row["index"] = c.index;
      row["name"] = (original_vsc && !original_vsc->name.empty())
          ? original_vsc->name
          : (c.name.empty()
                ? "VSC换流器 #" + std::to_string(c.index) + " (AC " + std::to_string(c.bus_ac) +
                      " -> DC " + std::to_string(c.bus_dc) + ")"
                : c.name);
      row["bus_ac"] = original_vsc ? original_vsc->bus_ac : c.bus_ac;
      row["bus_dc"] = original_vsc ? original_vsc->bus_dc : c.bus_dc;
      row["canvas_index"] = c.index;
      if (const auto er_idx = opf_energy_router_index_from_name(original_sys.energy_routers, c.name)) {
        opf_apply_canonical_source(row, original_sys, "energy_router", *er_idx);
        if (const auto* er = opf_find_energy_router(original_sys, *er_idx)) {
          if (const auto port_bus = opf_energy_router_port_bus_from_vsc_name(*er, c.name))
            row["bus_ac"] = *port_bus;
        }
        row["bus_dc"] = nullptr;
      }
    } else {
      row["index"] = nullptr;
      row["name"] = "VSC换流器 pos " + std::to_string(k);
      row["canvas_index"] = nullptr;
    }
    if (k < pac.size()) row["pac_mw"] = pac[k];
    if (k < qac.size()) row["qac_mvar"] = qac[k];
    rows.push_back(std::move(row));
  }
  return rows;
}

json opf_dcdc_dispatch_json(const hacdcpf::HybridPowerSystem& sys,
                            const hacdcpf::HybridPowerSystem& original_sys,
                            const hacdcpf::opf::ACOPFResult& r) {
  json rows = json::array();
  const auto fallback_positions = opf_active_dcdc_positions(sys);
  for (size_t k = 0; k < r.pdcdc_mw.size(); ++k) {
    size_t pos = k;
    if (k < r.dcdc_map.size() && r.dcdc_map[k].original_index >= 0) {
      pos = static_cast<size_t>(r.dcdc_map[k].original_index);
    } else if (k < fallback_positions.size()) {
      pos = fallback_positions[k];
    }
    json row{{"position", k}, {"display_type", "DC/DC变换器"}, {"canvas_type", "dcdcConverter"},
             {"pdcdc_mw", r.pdcdc_mw[k]}};
    if (pos < sys.dc.dcdc_converters.size()) {
      const auto& c = sys.dc.dcdc_converters[pos];
      const auto* original_dcdc = opf_find_by_index(original_sys.dc.dcdc_converters, c.index);
      row["source_position"] = pos;
      row["index"] = c.index;
      row["name"] = (original_dcdc && !original_dcdc->name.empty())
          ? original_dcdc->name
          : (c.name.empty()
                ? "DC/DC变换器 #" + std::to_string(c.index) + " (DC " + std::to_string(c.bus_in) +
                      " -> DC " + std::to_string(c.bus_out) + ")"
                : c.name);
      row["bus_in"] = original_dcdc ? original_dcdc->bus_in : c.bus_in;
      row["bus_out"] = original_dcdc ? original_dcdc->bus_out : c.bus_out;
      row["canvas_index"] = c.index;
      if (const auto er_idx = opf_energy_router_index_from_name(original_sys.energy_routers, c.name)) {
        opf_apply_canonical_source(row, original_sys, "energy_router", *er_idx);
        row["bus_in"] = nullptr;
        row["bus_out"] = nullptr;
      }
    } else {
      row["index"] = nullptr;
      row["name"] = "DC/DC变换器 pos " + std::to_string(k);
      row["canvas_index"] = nullptr;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

json opf_er_port_dispatch_json(const hacdcpf::HybridPowerSystem& original_sys,
                               const hacdcpf::opf::ACOPFResult& r) {
  json rows = json::array();
  for (size_t k = 0; k < r.er_port_p_mw.size(); ++k) {
    json row{{"position", k}, {"display_type", "能量路由器端口"},
             {"canvas_type", "energyRouter"}};
    if (k < r.er_port_map.size()) {
      const int er_index = r.er_port_map[k].original_index;
      const int port_index = r.er_port_map[k].source_type;
      row["router_index"] = er_index;
      row["port_index"] = port_index;
      row["canvas_index"] = er_index;
      if (const auto* er = opf_find_energy_router(original_sys, er_index)) {
        row["router_name"] =
            er->name.empty() ? opf_default_name("能量路由器", er->index) : er->name;
        if (const auto* port = opf_find_by_index(er->ports, port_index)) {
          row["port_name"] =
              port->name.empty() ? ("Port " + std::to_string(port->index)) : port->name;
          row["bus"] = port->bus;
          row["side"] = port->side;
          row["is_ac"] = port->port_type == hacdcpf::ERPortType::AC;
        }
      }
    }
    row["p_mw"] = r.er_port_p_mw[k];
    if (k < r.er_port_q_mvar.size()) row["q_mvar"] = r.er_port_q_mvar[k];
    rows.push_back(std::move(row));
  }
  return rows;
}

struct EnergyRouterExpansionRefs {
  std::unordered_map<int, int> port_vsc_index;
  int dcdc_index{-1};
};

std::unordered_map<int, EnergyRouterExpansionRefs> build_energy_router_expansion_refs(
    const hacdcpf::HybridPowerSystem& sys,
    const std::vector<hacdcpf::EnergyRouter>& er_snapshot) {
  std::unordered_map<int, EnergyRouterExpansionRefs> refs;
  if (er_snapshot.empty()) return refs;
  try {
    const hacdcpf::HybridPowerSystem projected =
        hacdcpf::project_to_canonical_models(sys);
    std::unordered_map<std::string, int> vsc_by_name;
    vsc_by_name.reserve(projected.vsc_converters.size());
    for (const auto& vsc : projected.vsc_converters) {
      vsc_by_name[vsc.name] = vsc.index;
    }
    std::unordered_map<std::string, int> dcdc_by_name;
    dcdc_by_name.reserve(projected.dc.dcdc_converters.size());
    for (const auto& dcdc : projected.dc.dcdc_converters) {
      dcdc_by_name[dcdc.name] = dcdc.index;
    }
    for (const auto& er : er_snapshot) {
      auto& r = refs[er.index];
      if (const auto dit = dcdc_by_name.find(er.name + "_DCDC");
          dit != dcdc_by_name.end()) {
        r.dcdc_index = dit->second;
      }
      for (const auto& p : er.ports) {
        const char side = (p.side == 0) ? 'A' : 'B';
        const std::string vsc_name =
            er.name + "_VSC_" + side + std::to_string(p.index);
        if (const auto vit = vsc_by_name.find(vsc_name);
            vit != vsc_by_name.end()) {
          r.port_vsc_index[p.index] = vit->second;
        }
      }
    }
  } catch (...) {
    // Presentation JSON should still be returned if rich ER attribution is not
    // recoverable; the caller falls back to scheduled port values.
  }
  return refs;
}

json power_flow_geo_energy_router_json(
    const hacdcpf::HybridPowerSystem& sys,
    const std::vector<hacdcpf::EnergyRouter>& er_snapshot,
    const hacdcpf::PowerFlowResult& pf) {
  json geo_er = json::array();
  const auto er_refs = build_energy_router_expansion_refs(sys, er_snapshot);
  std::unordered_map<int, const hacdcpf::VSCTransfer*> vsc_tr_by_index;
  for (const auto& vt : pf.vsc_transfers) vsc_tr_by_index[vt.index] = &vt;
  std::unordered_map<int, const hacdcpf::DCDCTransfer*> dcdc_tr_by_index;
  for (const auto& dt : pf.dcdc_transfers) dcdc_tr_by_index[dt.index] = &dt;

  for (const auto& er : er_snapshot) {
    if (!er.in_service) continue;
    const auto ref_it = er_refs.find(er.index);
    const EnergyRouterExpansionRefs* refs =
        ref_it != er_refs.end() ? &ref_it->second : nullptr;
    json port_arr = json::array();
    for (const auto& p : er.ports) {
      if (!p.in_service || p.bus == 0) continue;
      double p_mw = 0.0, q_mvar = 0.0, v_pu = 1.0;
      const bool is_ac = (p.port_type == hacdcpf::ERPortType::AC);
      int matched_vsc_index = -1;

      if (refs) {
        const auto pit = refs->port_vsc_index.find(p.index);
        if (pit != refs->port_vsc_index.end()) matched_vsc_index = pit->second;
      }
      if (const auto tit = vsc_tr_by_index.find(matched_vsc_index);
          tit != vsc_tr_by_index.end()) {
        p_mw = tit->second->p_ac_mw;
        q_mvar = tit->second->q_ac_mvar;
      } else {
        p_mw = p.p_set_mw;
        q_mvar = p.q_set_mvar;
      }
      if (is_ac) {
        for (size_t bi = 0; bi < sys.ac.buses.size(); ++bi) {
          if (sys.ac.buses[bi].index == p.bus && bi < pf.vm.size()) {
            v_pu = pf.vm[bi];
            break;
          }
        }
      } else {
        for (size_t bi = 0; bi < sys.dc.buses.size(); ++bi) {
          if (sys.dc.buses[bi].index == p.bus && bi < pf.vdc.size()) {
            v_pu = pf.vdc[bi];
            break;
          }
        }
      }
      port_arr.push_back(json{
        {"port_index", p.index}, {"name", p.name}, {"bus", p.bus},
        {"side", p.side}, {"control_mode", hacdcpf::er_control_str(p.control_mode)},
        {"matched_vsc_index", matched_vsc_index},
        {"is_ac", is_ac},
        {"p_mw", p_mw}, {"q_mvar", q_mvar}, {"v_pu", v_pu}
      });
    }
    const int internal_dcdc_index = refs ? refs->dcdc_index : -1;
    const hacdcpf::DCDCTransfer* internal_dcdc = nullptr;
    if (const auto dit = dcdc_tr_by_index.find(internal_dcdc_index);
        dit != dcdc_tr_by_index.end()) {
      internal_dcdc = dit->second;
    }
    double sum_p = 0.0;
    for (const auto& pp : port_arr) sum_p += pp["p_mw"].get<double>();
    double vsc_loss = 0.0;
    for (const auto& pp : port_arr) {
      const int vidx = pp.value("matched_vsc_index", -1);
      if (const auto vit = vsc_tr_by_index.find(vidx); vit != vsc_tr_by_index.end()) {
        vsc_loss += vit->second->loss_mw;
      }
    }
    const double dcdc_loss = internal_dcdc ? internal_dcdc->loss_mw : 0.0;
    const double total_loss = vsc_loss + dcdc_loss;
    geo_er.push_back(json{
      {"router_index", er.index}, {"ports", port_arr},
      {"p_port_sum_mw", sum_p},
      {"internal_dcdc_index", internal_dcdc_index},
      {"internal_dcdc_pin_mw", internal_dcdc ? internal_dcdc->p_in_mw : 0.0},
      {"internal_dcdc_pout_mw", internal_dcdc ? internal_dcdc->p_out_mw : 0.0},
      {"internal_dcdc_loss_mw", dcdc_loss},
      {"vsc_loss_mw", vsc_loss},
      {"loss_mw", total_loss}, {"p_rated_mw", er.p_rated_mw}
    });
  }
  return geo_er;
}

json opf_dc_branch_dispatch_json(const hacdcpf::HybridPowerSystem& sys,
                                 const std::vector<double>& pf) {
  json rows = json::array();
  const size_t n = std::max(sys.ac.branches.size(), pf.size());
  for (size_t i = 0; i < n; ++i) {
    json row{{"position", i}, {"display_type", "AC线路"}, {"canvas_type", "branch"}};
    if (i < sys.ac.branches.size()) {
      const auto& br = sys.ac.branches[i];
      row["index"] = br.index;
      row["name"] = br.name.empty() ? opf_branch_name("AC线路", br.index, br.from_bus, br.to_bus) : br.name;
      row["from_bus"] = br.from_bus;
      row["to_bus"] = br.to_bus;
      row["canvas_index"] = br.index;
      row["rate_mva"] = br.rate_a_mva;
    } else {
      row["index"] = nullptr;
      row["name"] = "AC线路 pos " + std::to_string(i);
      row["canvas_index"] = nullptr;
    }
    if (i < pf.size()) row["pf_mw"] = pf[i];
    rows.push_back(std::move(row));
  }
  return rows;
}

// Build default 24-h (or N-step) scaling profiles
hacdcpf::TimeSeriesData make_default_ts_data(int steps = 24) {
  hacdcpf::TimeSeriesData ts;
  ts.num_steps = steps;
  ts.step_duration_hr = 1.0;
  // Profile 0: daily load curve
  hacdcpf::TimeSeriesProfile lp; lp.id = 0; lp.name = "daily_load";
  if (steps == 24) {
    lp.values = {0.50,0.45,0.42,0.40,0.42,0.50,0.60,0.72,0.80,0.85,0.88,0.90,
                 0.88,0.85,0.82,0.85,0.90,1.00,1.10,1.05,0.95,0.85,0.72,0.60};
  } else { lp.values.assign(steps, 0.80); }
  ts.profiles.push_back(lp);
  // Profile 1: wind
  hacdcpf::TimeSeriesProfile wp; wp.id = 1; wp.name = "wind_daily";
  if (steps == 24) {
    wp.values = {0.65,0.70,0.75,0.80,0.72,0.55,0.35,0.20,0.15,0.10,0.18,0.25,
                 0.30,0.40,0.55,0.60,0.50,0.35,0.25,0.30,0.45,0.55,0.60,0.65};
  } else { wp.values.assign(steps, 0.40); }
  ts.profiles.push_back(wp);
  // Profile 2: solar
  hacdcpf::TimeSeriesProfile sp; sp.id = 2; sp.name = "solar_daily";
  if (steps == 24) {
    sp.values = {0.00,0.00,0.00,0.00,0.00,0.02,0.10,0.30,0.55,0.80,0.92,1.00,
                 0.98,0.90,0.75,0.55,0.30,0.10,0.02,0.00,0.00,0.00,0.00,0.00};
  } else { sp.values.assign(steps, 0.30); }
  ts.profiles.push_back(sp);
  return ts;
}

// Build annual time-series data with seasonal variation
hacdcpf::TimeSeriesData make_annual_ts_data(double step_hr = 6.0) {
  const int steps_per_day = static_cast<int>(24.0 / step_hr);
  const int total_steps = 365 * steps_per_day;
  hacdcpf::TimeSeriesData ts;
  ts.num_steps = total_steps;
  ts.step_duration_hr = step_hr;
  // Monthly seasonal factors: J,F,M,A,M,J,J,A,S,O,N,D
  const double load_season[] = {1.05,1.02,0.95,0.88,0.85,0.92,1.10,1.12,1.00,0.90,0.95,1.08};
  const double wind_season[] = {1.10,1.05,0.95,0.80,0.65,0.50,0.45,0.50,0.70,0.90,1.00,1.10};
  const double sol_season[]  = {0.65,0.75,0.90,1.05,1.15,1.20,1.18,1.10,0.95,0.80,0.65,0.55};
  // 24-h daily templates
  const double load_daily[] = {0.50,0.45,0.42,0.40,0.42,0.50,0.60,0.72,0.80,0.85,0.88,0.90,
                               0.88,0.85,0.82,0.85,0.90,1.00,1.10,1.05,0.95,0.85,0.72,0.60};
  const double wind_daily[] = {0.65,0.70,0.75,0.80,0.72,0.55,0.35,0.20,0.15,0.10,0.18,0.25,
                               0.30,0.40,0.55,0.60,0.50,0.35,0.25,0.30,0.45,0.55,0.60,0.65};
  const double sol_daily[]  = {0.00,0.00,0.00,0.00,0.00,0.02,0.10,0.30,0.55,0.80,0.92,1.00,
                               0.98,0.90,0.75,0.55,0.30,0.10,0.02,0.00,0.00,0.00,0.00,0.00};
  // Month day counts
  const int mdays[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  auto make_profile = [&](int id, const char* name, const double* daily,
                          const double* season) {
    hacdcpf::TimeSeriesProfile p; p.id = id; p.name = name;
    p.values.resize(static_cast<size_t>(total_steps));
    int idx = 0;
    for (int m = 0; m < 12; ++m) {
      for (int d = 0; d < mdays[m]; ++d) {
        for (int s = 0; s < steps_per_day; ++s) {
          int hour = static_cast<int>(s * step_hr);
          if (hour >= 24) hour = 23;
          double v = daily[hour] * season[m];
          if (v < 0.0) v = 0.0;
          if (v > 1.5) v = 1.5;
          p.values[static_cast<size_t>(idx++)] = v;
        }
      }
    }
    return p;
  };
  ts.profiles.push_back(make_profile(0, "annual_load", load_daily, load_season));
  ts.profiles.push_back(make_profile(1, "annual_wind", wind_daily, wind_season));
  ts.profiles.push_back(make_profile(2, "annual_solar", sol_daily, sol_season));
  return ts;
}

std::vector<std::string> case_names() {
  return {
      "ieee14_acdc",
      "ieee24_3area_acdc",
      "ieee24_3area_acdc_expanded",
      "ieee118_acdc",
      "case33bw_acdc",
      "case33mg_acdc",
      "case69_acdc",
      "case300_acdc",
      "case2000_acdc",
      "demo_multizone_acdc",
      "dist33_microgrid_der",
      "comprehensive_hybrid_acdc",
      "market_3bus_toy",
      "market_5bus_acdc_toy",
  };
}

hacdcpf::HybridPowerSystem build_case(const std::string& name) {
  using namespace hacdcpf::io;
  if (name == "ieee14_acdc") return build_ieee14_acdc();
  if (name == "ieee24_3area_acdc") return build_ieee24_3area_acdc();
  if (name == "ieee24_3area_acdc_expanded") return build_ieee24_3area_acdc_expanded();
  if (name == "ieee118_acdc") return build_ieee118_acdc();
  if (name == "case33bw_acdc") return build_case33bw_acdc();
  if (name == "case33mg_acdc") return build_case33mg_acdc();
  if (name == "case69_acdc") return build_case69_acdc();
  if (name == "case300_acdc") return build_case300_acdc();
  if (name == "case2000_acdc") return build_case2000_acdc();
  if (name == "demo_multizone_acdc") return build_demo_multizone_acdc();
  if (name == "dist33_microgrid_der") return build_dist33_microgrid_der();
  if (name == "comprehensive_hybrid_acdc") return build_comprehensive_hybrid_acdc();
  if (name == "market_3bus_toy") return build_market_3bus_toy();
  if (name == "market_5bus_acdc_toy") return build_market_5bus_acdc_toy();
  throw std::runtime_error("Unsupported case: " + name);
}

hacdcpf::HybridPowerSystem system_from_request(const json& req) {
  if (req.contains("system_json") && req["system_json"].is_string()) {
    return hacdcpf::io::from_json(req["system_json"].get<std::string>());
  }
  std::string name = "ieee24_3area_acdc_expanded";
  if (req.contains("case") && req["case"].is_string()) {
    name = req["case"].get<std::string>();
  }
  return build_case(name);
}

// List MATPOWER .m files in a given directory
std::vector<std::string> list_matpower_files(const std::string& dir) {
  std::vector<std::string> result;
  if (!fs::exists(dir) || !fs::is_directory(dir)) return result;
  for (const auto& entry : fs::directory_iterator(dir)) {
    if (entry.is_regular_file() && entry.path().extension() == ".m") {
      result.push_back(entry.path().filename().string());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

// Build a component-level summary of a HybridPowerSystem
json system_summary(const hacdcpf::HybridPowerSystem& sys) {
  const json root = json::parse(hacdcpf::io::to_json(sys, 2));
  json s;
  s["name"] = sys.name;
  s["base_mva"] = sys.base_mva;

  auto get_nested_array = [&root](const std::vector<std::string>& path) -> json {
    const json* cur = &root;
    for (const auto& k : path) {
      if (!cur->is_object() || !cur->contains(k)) return json::array();
      cur = &((*cur)[k]);
    }
    return cur->is_array() ? *cur : json::array();
  };

  const std::vector<std::pair<std::string, std::vector<std::string>>> comp_map = {
      {"ac_buses", {"ac", "buses"}},
      {"ac_branches", {"ac", "branches"}},
      {"generators", {"ac", "generators"}},
      {"static_generators", {"ac", "static_generators"}},
      {"loads", {"ac", "loads"}},
      {"flexible_loads", {"ac", "flexible_loads"}},
      {"asymmetric_loads", {"ac", "asymmetric_loads"}},
      {"shunts", {"ac", "shunts"}},
      {"storage", {"ac", "storage"}},
      {"renewable_gens", {"ac", "renewable_gens"}},
      {"pv_systems", {"ac", "pv_systems"}},
      {"external_grids", {"ac", "external_grids"}},
      {"transformers_2w", {"ac", "transformers_2w"}},
      {"transformers_3w", {"ac", "transformers_3w"}},
      {"switches", {"ac", "switches"}},
      {"circuit_breakers", {"ac", "circuit_breakers"}},
      {"charging_stations", {"ac", "charging_stations"}},
      {"chargers", {"ac", "chargers"}},
      {"motors", {"ac", "motors"}},
      {"dc_buses", {"dc", "buses"}},
      {"dc_branches", {"dc", "branches"}},
      {"dc_loads", {"dc", "loads"}},
      {"dc_storage", {"dc", "dc_storage"}},
      {"dc_static_generators", {"dc", "static_generators"}},
      {"dc_native_static_generators", {"dc", "dc_static_generators"}},
      {"pv_arrays", {"dc", "pv_arrays"}},
      {"dc_circuit_breakers", {"dc", "dc_circuit_breakers"}},
      {"vsc_converters", {"vsc_converters"}},
      {"dcdc_converters", {"dcdc_converters"}},
      {"energy_routers", {"energy_routers"}},
      {"mobile_storage", {"mobile_storage"}},
      {"vpps", {"vpps"}},
      {"microgrids", {"microgrids"}},
      {"tp_buses", {"three_phase_ac", "buses"}},
      {"tp_lines", {"three_phase_ac", "lines"}},
      {"tp_transformers", {"three_phase_ac", "transformers"}},
      {"tp_loads", {"three_phase_ac", "loads"}},
      {"tp_generators", {"three_phase_ac", "generators"}},
      {"tp_external_grids", {"three_phase_ac", "external_grids"}},
  };

  json counts = json::object();
  for (const auto& [k, path] : comp_map) {
    s[k] = get_nested_array(path);
    counts[k] = s[k].size();
  }
  if (s["dc_storage"].empty()) {
    s["dc_storage"] = get_nested_array({"dc", "storage"});
    counts["dc_storage"] = s["dc_storage"].size();
  }
  if (s["dcdc_converters"].empty()) {
    s["dcdc_converters"] = get_nested_array({"dc", "dcdc_converters"});
    counts["dcdc_converters"] = s["dcdc_converters"].size();
  }
  s["counts"] = counts;
  return s;
}

#if 0
std::string make_index_html() {
  return R"html(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1.0"/>
<title>Hybrid AC/DC Planning Studio</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;500;700&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet">
<link href="https://fonts.googleapis.com/css2?family=Material+Symbols+Outlined:opsz,wght,FILL,GRAD@20..48,100..700,0..1,-50..200" rel="stylesheet">
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
:root {
  --bg: #f3efe7; --paper: #fffdf8; --ink: #17252a;
  --accent: #0b6e4f; --accent-2: #2c8c99; --muted: #5a666a;
  --warn: #b5651d; --line: #d6d0c4; --danger: #c34d4d;
}
*{box-sizing:border-box;}
body{margin:0;font-family:"Space Grotesk",sans-serif;color:var(--ink);
  background:radial-gradient(circle at 10% 10%,rgba(44,140,153,0.14),transparent 35%),
    radial-gradient(circle at 90% 80%,rgba(11,110,79,0.16),transparent 30%),
    linear-gradient(165deg,#f8f3eb 0%,#ece7de 100%);min-height:100vh;}
header{padding:16px 28px;border-bottom:1px solid var(--line);
  backdrop-filter:blur(4px);background:rgba(255,253,248,0.85);
  position:sticky;top:0;z-index:10;display:flex;align-items:center;gap:18px;}
header h1{margin:0;font-size:clamp(1rem,2vw,1.5rem);letter-spacing:.03em;}
header .subtitle{color:var(--muted);font-size:.85rem;}
header .spacer{flex:1;}
.badge{display:inline-block;padding:3px 10px;border-radius:6px;font-size:.75rem;
  font-weight:600;letter-spacing:.04em;}
.badge-green{background:#d9f2e6;color:#0b6e4f;}
.badge-orange{background:#fde8d0;color:#b5651d;}
.container{width:min(1440px,97vw);margin:14px auto 28px;}
.top-bar{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px;align-items:center;}
.top-bar select,.top-bar input[type=file]{border-radius:10px;border:1px solid #cfc7ba;
  padding:7px 10px;font:inherit;background:#fff;min-width:160px;}
.top-bar .sep{width:1px;height:28px;background:var(--line);}
.grid{display:grid;grid-template-columns:1fr;gap:14px;}
.panel{background:var(--paper);border:1px solid var(--line);border-radius:14px;
  box-shadow:0 10px 28px rgba(23,37,42,.08);}
.tabs{display:flex;gap:6px;padding:8px 10px;border-bottom:1px solid var(--line);
  overflow-x:auto;flex-wrap:nowrap;}
.tab{padding:8px 14px;border-radius:8px;border:1px solid var(--line);
  cursor:pointer;background:#fdf9f1;font-size:.85rem;white-space:nowrap;
  transition:all .15s ease;}
.tab.active{background:var(--accent-2);color:#fff;border-color:var(--accent-2);}
.tab:hover:not(.active){background:#f0ede5;}
.tabpane{display:none;padding:14px;animation:fadeIn .25s ease;}
.tabpane.active{display:block;}
@keyframes fadeIn{from{opacity:0;transform:translateY(4px);}to{opacity:1;transform:none;}}
.counts{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:10px;}
.count-chip{background:#fff;border:1px solid var(--line);border-radius:8px;
  padding:5px 12px;font-size:.8rem;display:flex;align-items:center;gap:6px;}
.count-chip .n{font-weight:700;color:var(--accent);font-size:.95rem;}
.kpis{display:grid;grid-template-columns:repeat(auto-fill,minmax(130px,1fr));
  gap:8px;margin-bottom:10px;}
.kpi{border:1px solid var(--line);border-radius:10px;background:#fff;padding:8px 10px;}
.kpi .v{font-size:1.1rem;font-weight:700;color:#15353a;}
.kpi .l{font-size:.72rem;color:var(--muted);text-transform:uppercase;letter-spacing:.05em;}
.stg{padding:4px 10px;border-radius:12px;font-size:.78rem;font-weight:600;background:#e9ecef;color:#6c757d;}
.stg.done{background:#d4edda;color:#155724;} .stg.active{background:#cce5ff;color:#004085;animation:pulse 1s infinite;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.6}}
.dtable-wrap{max-height:420px;overflow:auto;border:1px solid var(--line);
  border-radius:10px;margin-bottom:10px;}
table.dtable{width:100%;border-collapse:collapse;font-size:.8rem;}
table.dtable th{position:sticky;top:0;background:#f4f1ea;border-bottom:2px solid var(--line);
  padding:7px 8px;text-align:left;font-weight:600;white-space:nowrap;z-index:2;}
table.dtable td{padding:5px 8px;border-bottom:1px solid #eae6de;white-space:nowrap;}
table.dtable tr:hover{background:#f9f6ef;}
table.dtable input,table.dtable select{border:1px solid transparent;background:transparent;
  font:inherit;padding:2px 4px;width:100%;border-radius:4px;min-width:60px;}
table.dtable input:focus,table.dtable select:focus{border-color:var(--accent-2);
  background:#fff;outline:none;}
table.dtable .row-del{cursor:pointer;color:var(--danger);font-size:.9rem;
  background:none;border:none;width:auto;min-width:auto;padding:2px;}
.chart{height:360px;border:1px solid var(--line);border-radius:10px;
  background:#fff;margin-top:8px;}
button,.btn{border-radius:10px;border:none;padding:8px 14px;font:inherit;
  cursor:pointer;font-weight:600;font-size:.85rem;transition:all .15s ease;}
.btn-primary{background:var(--accent);color:#fff;}
.btn-primary:hover{box-shadow:0 6px 14px rgba(11,110,79,.25);transform:translateY(-1px);}
.btn-secondary{background:#fff;color:var(--accent);border:1px solid var(--accent);}
.btn-secondary:hover{background:#f0fdf6;}
.btn-accent{background:var(--accent-2);color:#fff;}
.btn-accent:hover{box-shadow:0 6px 14px rgba(44,140,153,.25);}
.btn-danger{background:var(--danger);color:#fff;}
.btn-sm{padding:5px 10px;font-size:.78rem;}
.btn-group{display:flex;flex-wrap:wrap;gap:6px;margin:8px 0;}
.status{margin-top:8px;border:1px dashed #bcc5bf;border-radius:9px;padding:8px 10px;
  font-size:.85rem;color:#2d3f43;background:#fbfaf6;}
.mono{font-family:"IBM Plex Mono",monospace;font-size:.78rem;}
label{display:block;font-size:.82rem;margin:6px 0 4px;color:#294045;}
select,input[type=number]{width:100%;border-radius:8px;border:1px solid #cfc7ba;
  padding:7px 8px;font:inherit;background:#fff;}
textarea{width:100%;border-radius:8px;border:1px solid #cfc7ba;padding:7px 8px;
  font-family:"IBM Plex Mono",monospace;font-size:.78rem;
  min-height:100px;resize:vertical;background:#fff;}
.ctrl-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:6px;}
</style>
</head>
<body>

<header>
  <h1>Hybrid AC/DC Planning Studio</h1>
  <span class="subtitle">Power Flow &bull; OPF &bull; UC &bull; Carbon &bull; Reconfig &bull; Annual Sim &bull; Dashboard</span>
  <span class="spacer"></span>
  <span id="sysLabel" class="badge badge-orange">No system loaded</span>
</header>

<div class="container">
  <!-- Top bar: case loading & file I/O -->
  <div class="top-bar">
    <select id="builtinSelect" title="Built-in case"></select>
    <button class="btn btn-primary btn-sm" id="loadBuiltinBtn">Load Built-in</button>
    <span class="sep"></span>
    <select id="matpowerSelect" title="MATPOWER case file"></select>
    <button class="btn btn-accent btn-sm" id="loadMatpowerBtn">Load MATPOWER</button>
    <span class="sep"></span>
    <input type="file" id="jsonFileInput" accept=".json" title="Upload JSON system file"/>
    <button class="btn btn-secondary btn-sm" id="uploadJsonBtn">Upload JSON</button>
    <span class="sep"></span>
    <input type="file" id="etapXmlInput" accept=".xml" title="Import a native ETAP project XML (Feeder.xml)"/>
    <button class="btn btn-secondary btn-sm" id="loadEtapXmlBtn">Import ETAP XML</button>
    <span class="sep"></span>
    <button class="btn btn-secondary btn-sm" id="exportJsonBtn">Export JSON</button>
    <button class="btn btn-secondary btn-sm" id="exportEtapBtn">Export ETAP</button>
    <button class="btn btn-secondary btn-sm" id="newCaseBtn">New Empty Case</button>
  </div>

  <!-- Component counts -->
  <div id="countsBar" class="counts"></div>

  <!-- Main panel -->
  <div class="grid">
    <div class="panel">
      <div class="tabs">
        <div class="tab active" data-tab="builderPane">Case Builder</div>
        <div class="tab" data-tab="pfPane">Power Flow</div>
        <div class="tab" data-tab="opfPane">OPF</div>
        <div class="tab" data-tab="scPane">Short Circuit</div>
        <div class="tab" data-tab="jsonPane">JSON Editor</div>
        <div class="tab" data-tab="tsPane">Time-Series PF</div>
        <div class="tab" data-tab="ucPane">Unit Commitment</div>
        <div class="tab" data-tab="marketPane">Market Simulation</div>
        <div class="tab" data-tab="carbonPane">Carbon Flow</div>
        <div class="tab" data-tab="reliabilityPane">Reliability</div>
        <div class="tab" data-tab="resiliencePane">Resilience</div>
        <div class="tab" data-tab="rpoPane">RPO (MINLP)</div>
        <div class="tab" data-tab="reconfigPane">Net. Reconfig</div>
        <div class="tab" data-tab="annualPane">Annual Sim</div>
        <div class="tab" data-tab="lifecyclePane">Lifecycle Sim</div>
        <div class="tab" data-tab="dashPane">Dashboard</div>
      </div>

      <!-- ======================== CASE BUILDER TAB ======================== -->
      <section id="builderPane" class="tabpane active">
        <div class="tabs" id="compTabs" style="border:none;padding:0 0 8px 0;"></div>
        <div class="btn-group">
          <button class="btn btn-primary btn-sm" id="addRowBtn">+ Add Row</button>
          <button class="btn btn-accent btn-sm" id="commitBtn">Commit Changes to System</button>
        </div>
        <div id="compTableArea"></div>
      </section>

      <!-- ======================== POWER FLOW TAB ========================= -->
      <section id="pfPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Method</label>
            <select id="pfMethod">
              <option value="ac_newton">AC Newton-Raphson</option>
              <option value="dc">DC Power Flow</option>
              <option value="hybrid_linearized">Hybrid AC/DC Linearized</option>
              <option value="fdpf">Fast-Decoupled PF</option>
              <option value="adaptive">Adaptive PF</option>
              <option value="islanded">Islanded PF</option>
              <option value="distributed_slack">Distributed Slack PF</option>
              <option value="three_phase">Three-Phase PF</option>
            </select>
          </div>
          <div><label>Max iterations</label><input id="pfMaxIter" type="number" min="5" max="500" value="80"/></div>
          <div><label>Tolerance</label><input id="pfTol" type="number" step="1e-8" value="1e-8"/></div>
          <div><label>FDPF Max Iter</label><input id="pfFdpfMaxIter" type="number" min="10" max="5000" value="1000"/></div>
        </div>
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label><input type="checkbox" id="pfPvPq" checked style="width:auto;min-width:auto;"/> PV-PQ Conversion</label></div>
          <div><label><input type="checkbox" id="pfAutoSwing" checked style="width:auto;min-width:auto;"/> Auto Swing Selection</label></div>
          <div><label><input type="checkbox" id="pfConvSwitch" checked style="width:auto;min-width:auto;"/> Converter Mode Switching</label></div>
          <div><label><input type="checkbox" id="pfCoordCheck" checked disabled style="width:auto;min-width:auto;"/> Converter Coordination Check</label></div>
          <div><label><input type="checkbox" id="pfVerbose" style="width:auto;min-width:auto;"/> Verbose</label></div>
          <div><label>PV Q Hysteresis (pu)</label><input id="pfPvQHyst" type="number" step="0.001" value="0.01" style="width:90px;"/></div>
          <div><label>Max ΔVa (rad)</label><input id="pfMaxDVa" type="number" step="0.1" value="1.5" style="width:80px;"/></div>
          <div><label>Max ΔVm (pu)</label><input id="pfMaxDVm" type="number" step="0.1" value="0.5" style="width:80px;"/></div>
          <div><label>Loss Model</label>
            <select id="pfLossModel">
              <option value="linear">Linear</option>
              <option value="current_based">Current-Based</option>
            </select>
          </div>
          <div><label>Display Unit</label>
            <select id="pfDisplayUnit">
              <option value="MW" selected>MW</option>
              <option value="kW">kW</option>
              <option value="W">W</option>
            </select>
          </div>
        </div>
        <div class="btn-group">
          <button class="btn btn-primary" id="runPfBtn">Run Selected Method</button>
          <button class="btn btn-accent" id="runPfCompareBtn">Compare All Methods</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="pfConv" class="v">-</div><div class="l">Converged</div></div>
          <div class="kpi"><div id="pfIter" class="v">-</div><div class="l">Iterations</div></div>
          <div class="kpi"><div id="pfRes" class="v">-</div><div class="l">Residual</div></div>
          <div class="kpi"><div id="pfNBus" class="v">-</div><div class="l">AC Buses</div></div>
        </div>
        <div id="pfVoltChart" class="chart"></div>
        <div id="pfDcVoltChart" class="chart"></div>
        <div id="pfConverterChart" class="chart"></div>
        <div id="pfBranchChart" class="chart"></div>
        <div id="pfCompareChart" class="chart"></div>
        <div id="pfCompareTable" class="status mono"></div>
        <h3 style="margin:1.5rem 0 0.5rem;color:var(--accent);">GIS Network Map</h3>
        <div id="pfGeoMap" class="chart" style="height:500px;"></div>
      </section>

      <!-- ========================== OPF TAB ============================== -->
      <section id="opfPane" class="tabpane">
        <div class="btn-group">
          <label style="align-self:center;font-size:13px;color:#555;">AC Solver</label>
          <select id="opfSolver" title="AC OPF solver backend">
            <option value="auto" selected>Auto (Parity IPM &rarr; Ipopt)</option>
            <option value="parity">Parity IPM (native)</option>
            <option value="ipopt">Ipopt (filter line-search)</option>
            <option value="dispatch">Economic Dispatch (fast)</option>
          </select>
          <button class="btn btn-primary" id="runAcOpfBtn">Run AC OPF</button>
          <button class="btn btn-accent" id="runDcOpfBtn">Run DC OPF</button>
          <button class="btn" id="runParityOpfBtn" style="background:#6a0dad;color:#fff;">Run Parity OPF</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="opfType" class="v">-</div><div class="l">Solver</div></div>
          <div class="kpi"><div id="opfConv" class="v">-</div><div class="l">Converged</div></div>
          <div class="kpi"><div id="opfObj" class="v">-</div><div class="l">Objective</div></div>
          <div class="kpi"><div id="opfIter" class="v">-</div><div class="l">Iterations</div></div>
        </div>
        <div id="opfDispatchChart" class="chart"></div>
        <div id="opfAuxChart" class="chart"></div>
        <div id="opfLmpChart" class="chart"></div>
        <div id="opfDcVoltChart" class="chart"></div>
        <div id="opfConverterChart" class="chart"></div>
      </section>

      <!-- ========================== SC TAB =============================== -->
      <section id="scPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Fault type</label>
            <select id="scFaultType">
              <option value="ThreePhase">Three-Phase</option>
              <option value="SinglePhaseGround">Single-Phase-Ground</option>
              <option value="TwoPhase">Two-Phase</option>
              <option value="TwoPhaseGround">Two-Phase-Ground</option>
            </select>
          </div>
          <div><label>c-factor</label><input id="scCFactor" type="number" step=".01" value="1.10"/></div>
          <div><label>Fault Bus (blank=all)</label><input id="scFaultBus" type="text" placeholder="e.g. 0,3,5 or blank" style="width:130px;"/></div>
        </div>
        <div class="btn-group">
          <button class="btn btn-primary" id="runScBtn">Run Short Circuit (All Buses)</button>
          <button class="btn btn-accent" id="runScDetailedBtn">Run Detailed SC (Selected Buses)</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="scFault" class="v">-</div><div class="l">Fault Type</div></div>
          <div class="kpi"><div id="scBuses" class="v">-</div><div class="l">Buses</div></div>
          <div class="kpi"><div id="scMaxIk" class="v">-</div><div class="l">Max Ik" kA</div></div>
          <div class="kpi"><div id="scMinIk" class="v">-</div><div class="l">Min Ik" kA</div></div>
        </div>
        <div id="scIkChart" class="chart"></div>
        <div id="scSkChart" class="chart"></div>
        <div id="scDetailedChart" class="chart"></div>
        <div id="scContribChart" class="chart"></div>
        <div id="scVremainChart" class="chart"></div>
      </section>

      <!-- ======================== JSON EDITOR TAB ======================== -->
      <section id="jsonPane" class="tabpane">
        <div class="btn-group">
          <button class="btn btn-primary btn-sm" id="applyJsonBtn">Apply JSON to System</button>
          <button class="btn btn-secondary btn-sm" id="refreshJsonBtn">Refresh from System</button>
        </div>
        <textarea id="jsonEditor" style="min-height:500px;" placeholder="Load a case first..."></textarea>
      </section>

      <!-- =================== RPO (MINLP) TAB ========================= -->
      <section id="rpoPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Objective</label>
            <select id="rpoObjective">
              <option value="voltage" selected>Minimize Voltage Deviation</option>
              <option value="loss">Minimize Active Power Loss</option>
              <option value="combined">Combined (Loss + V-dev)</option>
            </select>
          </div>
          <div><label>MIP Gap (%)</label><input id="rpoMipGap" type="number" step="0.001" value="0.01"></div>
          <div><label>Time Limit (s)</label><input id="rpoTimeLimit" type="number" min="10" max="600" value="120"></div>
          <div><label>V-dev weight</label><input id="rpoVdevWeight" type="number" step="0.1" value="1.0"></div>
        </div>
        <div class="btn-group">
          <button class="btn btn-primary" id="runRPOBtn">Run RPO (Branch &amp; Bound + IPM)</button>
          <button class="btn btn-secondary" id="runRPORelaxBtn">Run Continuous Relaxation Only</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="rpoConv" class="v">-</div><div class="l">Converged</div></div>
          <div class="kpi"><div id="rpoObj" class="v">-</div><div class="l">Objective</div></div>
          <div class="kpi"><div id="rpoGap" class="v">-</div><div class="l">MIP Gap %</div></div>
          <div class="kpi"><div id="rpoNodes" class="v">-</div><div class="l">B&amp;B Nodes</div></div>
          <div class="kpi"><div id="rpoLPSolves" class="v">-</div><div class="l">NLP Solves</div></div>
          <div class="kpi"><div id="rpoTime" class="v">-</div><div class="l">Runtime (s)</div></div>
          <div class="kpi"><div id="rpoLossBefore" class="v">-</div><div class="l">Loss Before MW</div></div>
          <div class="kpi"><div id="rpoLossAfter" class="v">-</div><div class="l">Loss After MW</div></div>
          <div class="kpi"><div id="rpoVdevBefore" class="v">-</div><div class="l">Max |V-1| Before</div></div>
          <div class="kpi"><div id="rpoVdevAfter" class="v">-</div><div class="l">Max |V-1| After</div></div>
          <div class="kpi"><div id="rpoStatus" class="v" style="font-size:11px;">-</div><div class="l">Status</div></div>
        </div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;">
          <div id="rpoVoltChart" class="chart"></div>
          <div id="rpoQgChart" class="chart"></div>
          <div id="rpoPgChart" class="chart"></div>
          <div id="rpoVaChart" class="chart"></div>
          <div id="rpoTapChart" class="chart"></div>
          <div id="rpoShuntChart" class="chart"></div>
        </div>
        <div id="rpoDetailsChart" class="chart" style="height:300px;margin-top:10px;"></div>
        <div id="rpoResultTable" class="status mono" style="margin-top:8px;white-space:pre;overflow-x:auto;"></div>
      </section>

      <!-- =================== TIME-SERIES PF TAB ======================= -->
      <section id="tsPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Time Steps</label>
            <select id="tsNumSteps">
              <option value="4">4 steps (quick test)</option>
              <option value="24" selected>24 hours (1 day)</option>
              <option value="48">48 hours (2 days)</option>
            </select>
          </div>
          <div><label>Pipeline options</label>
            <label style="font-size:.82rem;margin:2px 0;"><input type="checkbox" id="tsSkipUC" style="width:auto;min-width:auto;"> Skip UC (PF only)</label>
            <label style="font-size:.82rem;margin:2px 0;"><input type="checkbox" id="tsRunOPF" style="width:auto;min-width:auto;"> Include OPF validation</label>
          </div>
        </div>
        <div class="btn-group">
          <button class="btn btn-primary" id="runTsPfBtn">Run Time-Series PF</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="tsSteps" class="v">-</div><div class="l">Steps</div></div>
          <div class="kpi"><div id="tsConv" class="v">-</div><div class="l">PF Converged</div></div>
          <div class="kpi"><div id="tsOPFConv" class="v">-</div><div class="l">OPF Converged</div></div>
          <div class="kpi"><div id="tsCost" class="v">-</div><div class="l">Total Gen Cost $</div></div>
        </div>
        <div id="tsGenDispatchChart" class="chart"></div>
        <div id="tsVoltChart" class="chart"></div>
        <div id="tsESSChart" class="chart"></div>
      </section>

      <!-- =================== UNIT COMMITMENT TAB ====================== -->
      <section id="ucPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Time Steps</label>
            <select id="ucNumSteps">
              <option value="4">4 steps (quick test)</option>
              <option value="24" selected>24 hours (1 day)</option>
              <option value="48">48 hours (2 days)</option>
            </select>
          </div>
          <div><label>MILP Solver</label>
            <select id="ucSolverChoice">
              <option value="auto" selected>Auto (Gurobi &gt; HiGHS &gt; Native)</option>
              <option value="native">Native Branch &amp; Cut</option>
              <option value="highs">HiGHS</option>
              <option value="gurobi">Gurobi</option>
            </select>
          </div>
        </div>
        <div class="btn-group">
          <button class="btn btn-primary" id="runUCBtn">Run Unit Commitment</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="ucFeas" class="v">-</div><div class="l">Feasible</div></div>
          <div class="kpi"><div id="ucCost" class="v">-</div><div class="l">Total Cost $</div></div>
          <div class="kpi"><div id="ucNGen" class="v">-</div><div class="l">Generators</div></div>
          <div class="kpi"><div id="ucNESS" class="v">-</div><div class="l">Storage Units</div></div>
        </div>
        <div id="ucDispatchChart" class="chart"></div>
        <div id="ucCommitChart" class="chart"></div>
        <div id="ucESSSOCChart" class="chart"></div>
      </section>

      <!-- =================== MARKET SIMULATION TAB ==================== -->
      <section id="marketPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Time Steps</label>
            <select id="mktNumSteps">
              <option value="4">4 steps (quick test)</option>
              <option value="24" selected>24 hours (1 day)</option>
              <option value="48">48 hours (2 days)</option>
            </select>
          </div>
          <div><label>MILP Solver</label>
            <select id="milpSolverChoice">
              <option value="auto" selected>Auto (Gurobi &gt; HiGHS &gt; Native)</option>
              <option value="native">Native Branch &amp; Cut</option>
              <option value="highs">HiGHS</option>
              <option value="gurobi">Gurobi</option>
            </select>
          </div>
          <div><label><input type="checkbox" id="scenarioToggle"> Enable Scenario Generation</label></div>
        </div>
        <div id="scenarioConfigPanel" style="display:none;margin-bottom:10px;padding:8px;background:#f0f4f8;border-radius:6px;">
          <div class="ctrl-row">
            <div><label>Periods/Day</label><input type="number" id="scenPeriodsPerDay" value="96" style="width:70px;"></div>
            <div><label>Period (min)</label><input type="number" id="scenPeriodLen" value="15" step="1" style="width:70px;"></div>
            <div><label>Load Noise</label><input type="number" id="scenLoadNoise" value="0.05" step="0.01" style="width:70px;"></div>
            <div><label>Wind Err</label><input type="number" id="scenWindErr" value="0.20" step="0.01" style="width:70px;"></div>
            <div><label>Solar Err</label><input type="number" id="scenSolarErr" value="0.15" step="0.01" style="width:70px;"></div>
            <div><label>Seed (0=rand)</label><input type="number" id="scenSeed" value="42" style="width:70px;"></div>
            <div><label><input type="checkbox" id="scenWorkday" checked> Workday</label></div>
          </div>
        </div>
        <div class="btn-group">
          <button class="btn btn-accent" id="runMarketBtn">Run Market Clearing (§2 Pipeline)</button>
        </div>
        <!-- §2 Parameters (expandable) -->
        <details style="margin:8px 0;padding:8px;background:#f0f4f8;border-radius:6px;">
          <summary style="cursor:pointer;font-weight:600;font-size:13px;">§2 Market Parameters (click to expand)</summary>
          <div class="ctrl-row" style="margin-top:8px;">
            <div><label>§2.2 Bid Segments</label><input type="number" id="s22BidSegs" value="3" min="1" max="10" style="width:55px;"></div>
            <div><label>§2.2 Price Cap (¥/MWh)</label><input type="number" id="s22PriceCap" value="1500" style="width:75px;"></div>
            <div><label>§2.2 Price Floor (¥/MWh)</label><input type="number" id="s22PriceFloor" value="0" style="width:70px;"></div>
          </div>
          <div class="ctrl-row">
            <div><label>§2.4 PTDF Threshold</label><input type="number" id="s24PtdfThresh" value="0.05" step="0.01" style="width:65px;"></div>
            <div><label>§2.6 MIP Gap</label><input type="number" id="s26MipGap" value="0.0001" step="0.0001" style="width:75px;"></div>
            <div><label>§2.6 Time Limit (s)</label><input type="number" id="s26TimeLimit" value="300" style="width:70px;"></div>
          </div>
          <div class="ctrl-row">
            <div><label>§2.6 LMP Delta</label><input type="number" id="s26LmpDelta" value="0.10" step="0.01" style="width:65px;"></div>
            <div><label>§2.8 N-1 Contingencies</label><input type="number" id="s28MaxN1" value="50" style="width:65px;"></div>
            <div><label><input type="checkbox" id="s28EnableN1" checked> §2.8 N-1 Check</label></div>
          </div>
          <div class="ctrl-row">
            <div><label>§2.9 VOLL ($/MWh)</label><input type="number" id="s29VOLL" value="10000" style="width:80px;"></div>
            <div><label>Reserve Req (%)</label><input type="number" id="s29SpinReq" value="5" step="1" style="width:55px;"></div>
          </div>
        </details>
        <!-- §2 Pipeline Progress (9 stages per market_rules.pdf) -->
        <div id="pipelineStages" style="display:none;margin:10px 0;padding:8px;background:#f8f9fa;border-radius:6px;overflow-x:auto;">
          <div style="display:flex;gap:3px;align-items:center;flex-wrap:wrap;font-size:11px;">
            <span id="stg1" class="stg">§2.2 Bids</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg2" class="stg">§2.3-4 Bounds</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg3" class="stg">§2.6.1 SCUC</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg4" class="stg">§2.6.2 SCED</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg5" class="stg">§2.6.3 LMP</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg6" class="stg">§2.7 ACPF</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg7" class="stg">§2.8 Security</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg8" class="stg">§2.9 Settlement</span>
            <span style="color:#ccc;">&rarr;</span>
            <span id="stg9" class="stg">§2.10 Results</span>
          </div>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="mktFeas" class="v">-</div><div class="l">Feasible</div></div>
          <div class="kpi"><div id="mktCost" class="v">-</div><div class="l">Total Cost $</div></div>
          <div class="kpi"><div id="mktNGen" class="v">-</div><div class="l">Generators</div></div>
          <div class="kpi"><div id="mktSolver" class="v">-</div><div class="l">Solver</div></div>
          <div class="kpi"><div id="mktValidation" class="v">-</div><div class="l">Validation</div></div>
          <div class="kpi"><div id="mktAvgLmp" class="v">-</div><div class="l">Avg LMP $/MWh</div></div>
        </div>
        <div id="scenarioProfileChart" class="chart"></div>
        <div id="mktDispatchChart" class="chart"></div>
        <div id="marketBidChart" class="chart"></div>
        <div id="marketLmpHeatmap" class="chart"></div>
        <div id="marketGenCoProfit" class="chart"></div>
        <div id="mktGenCoIncomeTable" style="margin:10px 0;"></div>
        <div id="marketPfResidualChart" class="chart"></div>
        <div id="mktVoltageProfileChart" class="chart"></div>
        <div id="mktReactivePowerChart" class="chart"></div>
        <div id="mktPtdfHeatmap" class="chart"></div>
        <div id="mktPtdfVerifyChart" class="chart"></div>
        <div id="mktViolationPanel" style="margin:10px 0;"></div>
        <div id="mktSecurityPanel" style="margin:10px 0;"></div>
        <div id="marketValidationPanel" style="margin:10px 0;"></div>
        <div class="dtable-wrap"><table class="dtable" id="marketLmpDecompTable"></table></div>
        <h3 style="margin:1.5rem 0 0.5rem;color:var(--accent);">GIS Network Map (LMP)</h3>
        <div id="mktGeoMap" class="chart" style="height:550px;"></div>
      </section>

      <!-- =================== CARBON FLOW TAB ========================== -->
      <section id="carbonPane" class="tabpane">
        <div class="btn-group">
          <button class="btn btn-primary" id="runCarbonBtn">Run Carbon Flow Analysis</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="carbonGenEmit" class="v">-</div><div class="l">Gen Emissions tCO&#x2082;</div></div>
          <div class="kpi"><div id="carbonLoadEmit" class="v">-</div><div class="l">Load Emissions tCO&#x2082;</div></div>
          <div class="kpi"><div id="carbonLossEmit" class="v">-</div><div class="l">Loss Emissions tCO&#x2082;</div></div>
          <div class="kpi"><div id="carbonMatrix" class="v">-</div><div class="l">Matrix Solved</div></div>
        </div>
        <div id="carbonBusChart" class="chart"></div>
        <div id="carbonDcBusChart" class="chart"></div>
        <div id="carbonLoadChart" class="chart" style="height:260px;"></div>
        <div id="carbonBranchChart" class="chart" style="height:260px;"></div>
        <div id="carbonSankeyChart" class="chart" style="height:380px;"></div>
      </section>

      <!-- =================== NET. RECONFIGURATION TAB ================= -->
      <section id="reconfigPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Time Steps</label>
            <select id="rcNumSteps">
              <option value="4" selected>4 steps (quick test)</option>
              <option value="24">24 hours (1 day)</option>
            </select>
          </div>
          <div><label>Voltage Limits</label>
            <div style="display:flex;gap:6px;">
              <input id="rcVmin" type="number" step=".01" value="0.95" placeholder="Vmin" style="width:80px;">
              <input id="rcVmax" type="number" step=".01" value="1.05" placeholder="Vmax" style="width:80px;">
            </div>
          </div>
          <div><label>MIP Gap</label><input id="rcMipGap" type="number" step=".005" value="0.01"></div>
          <div><label>Max time (s)</label><input id="rcMaxTime" type="number" min="10" max="600" value="60"></div>
        </div>
        <div class="btn-group">
          <button class="btn btn-primary" id="runReconfigBtn">Run Network Reconfiguration</button>
        </div>
        <div class="kpis">
          <div class="kpi"><div id="rcFeas" class="v">-</div><div class="l">Feasible</div></div>
          <div class="kpi"><div id="rcObj" class="v">-</div><div class="l">Objective</div></div>
          <div class="kpi"><div id="rcACLoss" class="v">-</div><div class="l">Curtailment MW</div></div>
          <div class="kpi"><div id="rcShed" class="v">-</div><div class="l">Branches</div></div>
          <div class="kpi"><div id="rcSwActions" class="v">-</div><div class="l">Switch Actions</div></div>
        </div>
        <div id="rcLossChart" class="chart"></div>
        <div id="rcSwitchChart" class="chart"></div>
        <div id="rcESSChart" class="chart"></div>
      </section>

      <!-- =================== ANNUAL PRODUCTION SIM TAB ================ -->
      <section id="annualPane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Resolution</label>
            <select id="annualResolution">
              <option value="6h" selected>6-hour (1460 steps)</option>
              <option value="1h">Hourly (8760 steps)</option>
            </select>
          </div>
          <div><label>Block Type</label>
            <select id="annualBlockType">
              <option value="monthly" selected>Monthly (12 blocks)</option>
              <option value="weekly">Weekly (52 blocks)</option>
            </select>
          </div>
          <div><label>PF Snapshot Interval</label><input id="annualSnapshotInterval" type="number" min="0" max="168" value="24"/></div>
          <div><label><input type="checkbox" id="annualRunOPF" checked style="width:auto;min-width:auto;"/> Include OPF</label></div>
          <div><label><input type="checkbox" id="annualCyclicSOC" checked style="width:auto;min-width:auto;"/> Cyclic SOC</label></div>
          <div><label><input type="checkbox" id="annualSkipReplay" style="width:auto;min-width:auto;"/> Schedule Only (skip replay)</label></div>
        </div>
        <div class="btn-group">
          <button class="btn btn-primary" id="runAnnualBtn">Run Annual Production Simulation</button>
        </div>
        <!-- Animation Playback Controls -->
        <div id="annAnimControls" style="display:none;margin:12px 0;padding:10px;background:var(--bg);border:1px solid var(--border);border-radius:6px;">
          <div style="display:flex;align-items:center;gap:10px;flex-wrap:wrap;">
            <button class="btn" id="annAnimPlayPause" style="width:80px;">&#9658; Play</button>
            <button class="btn" id="annAnimReset" style="width:60px;">&#8634;</button>
            <input type="range" id="annAnimSlider" min="0" max="100" value="0" style="flex:1;min-width:200px;"/>
            <span id="annAnimTimeLabel" style="min-width:80px;font-weight:600;">Hour 0</span>
            <select id="annAnimSpeed" style="width:100px;">
              <option value="200">Fast</option>
              <option value="500" selected>Medium</option>
              <option value="1000">Slow</option>
            </select>
            <label style="display:flex;align-items:center;gap:4px;"><input type="checkbox" id="annAnimGeo" checked style="width:auto;min-width:auto;"/> GIS Map</label>
          </div>
        </div>
        <div id="annGeoMap" class="chart" style="height:450px;display:none;"></div>
        <div class="kpis">
          <div class="kpi"><div id="annFeas" class="v">-</div><div class="l">Feasible</div></div>
          <div class="kpi"><div id="annCost" class="v">-</div><div class="l">Annual Cost $</div></div>
          <div class="kpi"><div id="annGenMWh" class="v">-</div><div class="l">Generation MWh</div></div>
          <div class="kpi"><div id="annRenMWh" class="v">-</div><div class="l">Renewable MWh</div></div>
          <div class="kpi"><div id="annCurtMWh" class="v">-</div><div class="l">Curtailed MWh</div></div>
          <div class="kpi"><div id="annENSMWh" class="v">-</div><div class="l">ENS MWh</div></div>
          <div class="kpi"><div id="annLossMWh" class="v">-</div><div class="l">Losses MWh</div></div>
          <div class="kpi"><div id="annPFConv" class="v">-</div><div class="l">PF Converged</div></div>
        </div>
        <div id="annTimelineChart" class="chart"></div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;">
          <div id="annMonthlyChart" class="chart" style="height:320px;"></div>
          <div id="annMonthlyCostChart" class="chart" style="height:320px;"></div>
          <div id="annGenStatsChart" class="chart" style="height:320px;"></div>
          <div id="annRenStatsChart" class="chart" style="height:320px;"></div>
          <div id="annStorageStatsChart" class="chart" style="height:320px;"></div>
          <div id="annVoltHeatmap" class="chart" style="height:320px;"></div>
        </div>
      </section>

      <!-- =================== LIFECYCLE SIM TAB ======================== -->
      <section id="lifecyclePane" class="tabpane">
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Years</label><input id="lcNumYears" type="number" min="1" max="50" value="20"/></div>
          <div><label>Discount Rate</label><input id="lcDiscountRate" type="number" min="0" max="0.2" step="0.01" value="0.05"/></div>
          <div><label>Load Growth %/yr</label><input id="lcLoadGrowth" type="number" min="0" max="0.1" step="0.005" value="0.02"/></div>
          <div><label>PV Derating %/yr</label><input id="lcPVDerating" type="number" min="0" max="0.05" step="0.001" value="0.005"/></div>
          <div><label>Cal. Degrad. %/yr</label><input id="lcCalDegrad" type="number" min="0" max="0.1" step="0.005" value="0.02"/></div>
          <div><label>Resolution</label>
            <select id="lcResolution">
              <option value="6h" selected>6-hour (1460 steps)</option>
              <option value="1h">Hourly (8760 steps)</option>
            </select>
          </div>
        </div>
        <details style="margin-bottom:10px;border:1px solid var(--border);border-radius:6px;padding:8px 12px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);">&#9881; Capacity Scaling (Planning Parameters)</summary>
          <div class="ctrl-row" style="margin-top:8px;">
            <div><label>PV Scale</label><input id="lcPVScale" type="number" min="0" max="10" step="0.1" value="1.0"/></div>
            <div><label>Wind Scale</label><input id="lcWindScale" type="number" min="0" max="10" step="0.1" value="1.0"/></div>
            <div><label>BESS Power Scale</label><input id="lcBESSPowerScale" type="number" min="0" max="10" step="0.1" value="1.0"/></div>
            <div><label>BESS Energy Scale</label><input id="lcBESSEnergyScale" type="number" min="0" max="10" step="0.1" value="1.0"/></div>
            <div><label>Diesel/Gas Scale</label><input id="lcDieselScale" type="number" min="0" max="10" step="0.1" value="1.0"/></div>
          </div>
        </details>
        <div class="btn-group" style="margin-bottom:10px;">
          <button class="btn btn-primary" id="runLifecycleBtn">Run Lifecycle Simulation</button>
        </div>
        <details open style="margin-bottom:10px;border:1px solid var(--border);border-radius:6px;padding:8px 12px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);">&#128202; Capacity Comparison Sweep</summary>
          <div class="ctrl-row" style="margin-top:8px;">
            <div><label>Sweep Parameter</label>
              <select id="lcSweepParam">
                <option value="pv">PV Capacity</option>
                <option value="wind">Wind Capacity</option>
                <option value="bess_power">BESS Power (MW)</option>
                <option value="bess_energy" selected>BESS Energy (MWh)</option>
                <option value="diesel">Diesel/Gas</option>
              </select>
            </div>
            <div><label>Min Scale</label><input id="lcSweepMin" type="number" min="0" max="10" step="0.25" value="0.5"/></div>
            <div><label>Max Scale</label><input id="lcSweepMax" type="number" min="0.5" max="10" step="0.25" value="3.0"/></div>
            <div><label>Steps</label><input id="lcSweepSteps" type="number" min="3" max="20" value="6"/></div>
          </div>
          <div class="btn-group" style="margin-top:6px;">
            <button class="btn" id="runCapCompareBtn" style="background:#8e44ad;color:#fff;">&#9654; Run Capacity Comparison</button>
          </div>
        </details>
        <div class="kpis" id="lcKPIs">
          <div class="kpi"><div id="lcFeas" class="v">-</div><div class="l">Feasible</div></div>
          <div class="kpi"><div id="lcNPV" class="v">-</div><div class="l">NPV Cost $</div></div>
          <div class="kpi"><div id="lcTotalCarbon" class="v">-</div><div class="l">Total CO&#8322;</div></div>
          <div class="kpi"><div id="lcReplacements" class="v">-</div><div class="l">Replacements</div></div>
          <div class="kpi"><div id="lcReplCost" class="v">-</div><div class="l">Replacement $</div></div>
          <div class="kpi"><div id="lcYears" class="v">-</div><div class="l">Years</div></div>
        </div>
        <div id="lcCostChart" class="chart" style="height:360px;"></div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;">
          <div id="lcCarbonChart" class="chart" style="height:320px;"></div>
          <div id="lcCarbonIntensityChart" class="chart" style="height:320px;"></div>
          <div id="lcEnergyChart" class="chart" style="height:320px;"></div>
          <div id="lcSOHChart" class="chart" style="height:320px;"></div>
          <div id="lcPVChart" class="chart" style="height:320px;"></div>
          <div id="lcBoundsChart" class="chart" style="height:320px;"></div>
          <div id="lcBoundsBreakdownChart" class="chart" style="height:320px;"></div>
          <div id="lcCrossValChart" class="chart" style="height:320px;"></div>
          <div id="lcTightnessChart" class="chart" style="height:320px;"></div>
        </div>
        <h3 style="margin-top:18px;color:var(--accent);">Capacity Comparison Results</h3>
        <div id="lcCompareStatus" class="status mono" style="margin-bottom:6px;">Run a capacity comparison sweep above.</div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;">
          <div id="lcCompNPVChart" class="chart" style="height:340px;"></div>
          <div id="lcCompCarbonChart" class="chart" style="height:340px;"></div>
          <div id="lcCompParetoChart" class="chart" style="height:340px;"></div>
          <div id="lcCompCarbonTrajChart" class="chart" style="height:340px;"></div>
        </div>
      </section>

      <!-- =================== RELIABILITY ASSESSMENT TAB ================ -->
      <section id="reliabilityPane" class="tabpane">
        <h3 style="color:var(--accent);margin-bottom:12px;">Monte Carlo Reliability Assessment</h3>
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Load Scale Factor</label><input id="relLoadScale" type="number" min="0.5" max="3.0" step="0.05" value="1.0" title="Scale load (>1 reduces reserve margin)"/></div>
          <div><label>CoV Threshold</label><input id="relCovThresh" type="number" min="0.01" max="0.5" step="0.01" value="0.05"/></div>
          <div><label>Random Seed</label><input id="relSeed" type="number" min="0" max="999999" value="0" title="0 = non-deterministic"/></div>
          <div style="display:flex;align-items:center;gap:4px;">
            <input type="checkbox" id="relApplyIEEE24" checked/>
            <label for="relApplyIEEE24" style="margin:0;">Apply IEEE-24 Reliability Data</label>
          </div>
        </div>
        <!-- Advanced Options: Tail Risk -->
        <div class="ctrl-row" style="margin-bottom:10px;background:#f5f7f5;padding:8px;border-radius:5px;">
          <div style="display:flex;align-items:center;gap:4px;">
            <input type="checkbox" id="relTailRisk"/>
            <label for="relTailRisk" style="margin:0;">Compute Tail Risk (VaR/CVaR)</label>
          </div>
          <div><label>VaR Confidence</label><input id="relVarConf" type="number" min="0.80" max="0.99" step="0.01" value="0.95" title="Confidence level for VaR/CVaR"/></div>
        </div>
        <details open style="border:1px solid var(--border);border-radius:6px;padding:10px 14px;margin-bottom:14px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);">Non-Sequential Monte Carlo (State Sampling)</summary>
          <div class="ctrl-row" style="margin-top:10px;">
            <div><label>Max Samples</label><input id="relNsqMaxIter" type="number" min="100" max="100000" value="5000"/></div>
          </div>
          <div class="btn-group" style="margin-top:8px;">
            <button class="btn btn-primary" id="runNsqBtn">&#9654; Run NSQ Monte Carlo</button>
          </div>
        </details>
        <details style="border:1px solid var(--border);border-radius:6px;padding:10px 14px;margin-bottom:14px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);">Sequential Monte Carlo (Chronological Simulation)</summary>
          <div class="ctrl-row" style="margin-top:10px;">
            <div><label>Max Years</label><input id="relSeqMaxYears" type="number" min="10" max="5000" value="500"/></div>
            <div><label>Hours/Year</label>
              <select id="relSeqHours">
                <option value="8736">8736 (52 weeks)</option>
                <option value="8760">8760 (full year)</option>
              </select>
            </div>
          </div>
          <div class="btn-group" style="margin-top:8px;">
            <button class="btn" style="background:#8e44ad;color:#fff;" id="runSeqBtn">&#9654; Run SEQ Monte Carlo</button>
          </div>
        </details>
        <details style="border:1px solid var(--border);border-radius:6px;padding:10px 14px;margin-bottom:14px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);">Frequency &amp; Duration (Analytical)</summary>
          <p style="color:var(--muted);font-size:0.9em;margin:6px 0 10px 0;">Build COPT analytically (no simulation). Fast calculation of LOLP, LOLE, LOLF, LOLD.</p>
          <div class="btn-group">
            <button class="btn" style="background:#b57e2b;color:#fff;" id="runFDBtn">&#9654; Run F&amp;D Analysis</button>
          </div>
        </details>
        <details style="border:1px solid var(--border);border-radius:6px;padding:10px 14px;margin-bottom:14px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);">&#128270; FMEA (Failure Modes &amp; Effects Analysis)</summary>
          <p style="color:var(--muted);font-size:0.9em;margin:6px 0 10px 0;">N-1 contingency enumeration for distribution system reliability. Evaluates each component failure individually.</p>
          <div class="ctrl-row" style="margin-top:10px;">
            <div style="display:flex;align-items:center;gap:4px;">
              <input type="checkbox" id="relFmeaApplyComprehensive" checked/>
              <label for="relFmeaApplyComprehensive" style="margin:0;">Apply Comprehensive Reliability Data</label>
            </div>
          </div>
          <div class="btn-group" style="margin-top:8px;">
            <button class="btn" style="background:#c0392b;color:#fff;" id="runFmeaBtn">&#9654; Run FMEA</button>
          </div>
        </details>
        <details open style="border:1px solid var(--border);border-radius:6px;padding:10px 14px;margin-bottom:14px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);">&#128200; Cross-Validation (NSQ vs SEQ)</summary>
          <p style="color:var(--muted);font-size:0.9em;margin:6px 0 10px 0;">Run both methods with the same settings and compare results.</p>
          <div class="btn-group">
            <button class="btn" style="background:#2c8c99;color:#fff;" id="runCrossValBtn">&#9654; Run Cross-Validation</button>
          </div>
        </details>
        <div class="kpis" id="relKPIs">
          <div class="kpi"><div id="relMethod" class="v">-</div><div class="l">Method</div></div>
          <div class="kpi"><div id="relEENS" class="v">-</div><div class="l">EENS (MWh/yr)</div></div>
          <div class="kpi"><div id="relLOLE" class="v">-</div><div class="l">LOLE (hr/yr)</div></div>
          <div class="kpi"><div id="relLOLF" class="v">-</div><div class="l">LOLF (occ/yr)</div></div>
          <div class="kpi"><div id="relPLC" class="v">-</div><div class="l">PLC (%)</div></div>
          <div class="kpi"><div id="relCoV" class="v">-</div><div class="l">Final CoV</div></div>
          <div class="kpi"><div id="relIters" class="v">-</div><div class="l">Iterations</div></div>
        </div>
        <!-- Tail Risk KPIs -->
        <div class="kpis" id="relTailKPIs" style="display:none;margin-top:8px;background:#f5f7f5;border-radius:6px;padding:6px;">
          <div class="kpi"><div id="relEENSVar" class="v">-</div><div class="l">EENS VaR</div></div>
          <div class="kpi"><div id="relEENSCVar" class="v">-</div><div class="l">EENS CVaR</div></div>
          <div class="kpi"><div id="relLOLEVar" class="v">-</div><div class="l">LOLE VaR</div></div>
          <div class="kpi"><div id="relLOLECVar" class="v">-</div><div class="l">LOLE CVaR</div></div>
        </div>
        <div id="relStatus" class="status mono" style="margin-bottom:10px;">Set parameters above and run an analysis.</div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;">
          <div id="relConvergenceChart" class="chart" style="height:320px;"></div>
          <div id="relNodalEENSChart" class="chart" style="height:320px;"></div>
          <div id="relCriticalChart" class="chart" style="height:320px;"></div>
          <div id="relCrossValChart" class="chart" style="height:320px;"></div>
        </div>
        <!-- FMEA Results Section -->
        <div id="fmeaResultsSection" style="display:none;margin-top:14px;">
          <h4 style="color:var(--accent);margin-bottom:8px;">FMEA Results</h4>
          <div class="kpis" id="fmeaKPIs">
            <div class="kpi"><div id="fmeaEENS" class="v">-</div><div class="l">EENS (MWh/yr)</div></div>
            <div class="kpi"><div id="fmeaLOLE" class="v">-</div><div class="l">LOLE (hr/yr)</div></div>
            <div class="kpi"><div id="fmeaLOLF" class="v">-</div><div class="l">LOLF (occ/yr)</div></div>
            <div class="kpi"><div id="fmeaSAIFI" class="v">-</div><div class="l">SAIFI</div></div>
            <div class="kpi"><div id="fmeaSAIDI" class="v">-</div><div class="l">SAIDI</div></div>
            <div class="kpi"><div id="fmeaASAI" class="v">-</div><div class="l">ASAI</div></div>
            <div class="kpi"><div id="fmeaContingencies" class="v">-</div><div class="l">Contingencies</div></div>
          </div>
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px;">
            <div id="fmeaContChart" class="chart" style="height:320px;"></div>
            <div id="fmeaTypeChart" class="chart" style="height:320px;"></div>
          </div>
        </div>
      </section>

      <!-- =================== RESILIENCE TAB ============================ -->
      <section id="resiliencePane" class="tabpane">
        <h3 style="color:var(--accent);margin-bottom:12px;">Distribution Resilience Assessment (MESS)</h3>
        <p style="color:var(--muted);font-size:0.9em;margin:0 0 14px 0;">Multi-hour restoration study with time-varying load/RES profiles, feeder outages, repair windows, tie-switch reconfiguration, and mobile energy storage dispatch &amp; routing.</p>
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div><label>Horizon (hr)</label><input id="resHorizonHours" type="number" min="1" max="168" value="24"/></div>
          <div><label>Repair Time (hr)</label><input id="resRepairHours" type="number" min="1" max="72" value="6"/></div>
          <div><label>Default Fault Count</label><input id="resFaultCount" type="number" min="1" max="10" value="1"/></div>
          <div><label>MESS Speed (km/h)</label><input id="resMessSpeed" type="number" min="5" max="120" value="40"/></div>
          <div><label>Load Scale Factor</label><input id="resLoadScale" type="number" min="0.5" max="3.0" step="0.05" value="1.0"/></div>
        </div>
        <div class="ctrl-row" style="margin-bottom:10px;">
          <div style="min-width:260px;flex:1;"><label>Fault Branch IDs</label><input id="resFaultBranches" type="text" placeholder="e.g. 25,37 (blank = auto-select)"/></div>
          <div><label>Fault Stagger (hr)</label><input id="resFaultStagger" type="number" min="0" max="24" step="0.5" value="0"/></div>
          <div><label>First Fault at (hr)</label><input id="resFaultStartHr" type="number" min="0" max="48" step="0.5" value="0"/></div>
          <div style="display:flex;align-items:center;gap:4px;"><input type="checkbox" id="resApplyDemoData" checked/><label for="resApplyDemoData" style="margin:0;">Apply Demo Data</label></div>
          <div style="display:flex;align-items:center;gap:4px;"><input type="checkbox" id="resAllowReconfig"/><label for="resAllowReconfig" style="margin:0;">Allow Reconfiguration</label></div>
          <div style="display:flex;align-items:center;gap:4px;"><input type="checkbox" id="resAllowMess" checked/><label for="resAllowMess" style="margin:0;">Allow MESS Dispatch</label></div>
          <div style="display:flex;align-items:center;gap:4px;"><input type="checkbox" id="resRunPF"/><label for="resRunPF" style="margin:0;">Run Power Flow</label></div>
        </div>
        <details style="margin-bottom:10px;border:1px solid var(--border);border-radius:8px;padding:6px 12px;">
          <summary style="cursor:pointer;font-weight:600;color:var(--accent);font-size:0.9em;">&#9881; Manual Fault Sequence Editor</summary>
          <p style="color:var(--muted);font-size:0.82em;margin:4px 0 8px 0;">Define individual faults with branch ID, start time and repair duration. Leave empty to use auto-generation above.</p>
          <table id="resFaultTable" style="width:100%;border-collapse:collapse;font-size:0.85em;">
            <thead><tr style="background:var(--bg2);"><th style="padding:4px 8px;">Branch ID</th><th style="padding:4px 8px;">Start (hr)</th><th style="padding:4px 8px;">Repair (hr)</th><th style="padding:4px 8px;">Label</th><th style="width:40px;"></th></tr></thead>
            <tbody id="resFaultTableBody"></tbody>
          </table>
          <button class="btn" style="margin-top:6px;font-size:0.82em;padding:4px 12px;" onclick="addFaultRow()">+ Add Fault</button>
        </details>
        <div class="btn-group" style="margin-bottom:10px;">
          <button class="btn" style="background:#0f766e;color:#fff;" id="runResilienceBtn">&#9654; Run Resilience Assessment</button>
        </div>
        <div id="resStatus" class="status mono" style="margin-bottom:10px;">Set parameters and click Run.</div>
        <div id="resilienceResultsSection" style="display:none;">
          <div class="kpis" id="resilienceKPIs">
            <div class="kpi"><div id="resilienceRI" class="v">-</div><div class="l">Resilience Index</div></div>
            <div class="kpi"><div id="resilienceDemand" class="v">-</div><div class="l">Total Demand (MWh)</div></div>
            <div class="kpi"><div id="resilienceShed" class="v">-</div><div class="l">Total Shed (MWh)</div></div>
            <div class="kpi"><div id="resilienceFinalRestore" class="v">-</div><div class="l">Final Restoration (%)</div></div>
            <div class="kpi"><div id="resilienceAvgRestore" class="v">-</div><div class="l">Avg Restoration (%)</div></div>
            <div class="kpi"><div id="resiliencePeakShed" class="v">-</div><div class="l">Peak Shed (MW)</div></div>
            <div class="kpi"><div id="resilienceMessEnergy" class="v">-</div><div class="l">MESS Energy (MWh)</div></div>
            <div class="kpi"><div id="resilienceMessTravel" class="v">-</div><div class="l">MESS Travel (km)</div></div>
            <div class="kpi"><div id="resilienceSwitches" class="v">-</div><div class="l">Switch Actions</div></div>
            <div class="kpi"><div id="resilienceRepairs" class="v">-</div><div class="l">Repaired Faults</div></div>
          </div>
          <div id="resilienceCompareKPIs" style="display:none;margin-top:6px;padding:8px 14px;background:linear-gradient(135deg,#f0fdf4,#ecfdf5);border:1px solid #86efac;border-radius:8px;">
            <span style="font-weight:600;color:#166534;font-size:0.85em;">&#9650; MESS Improvement:</span>
            <span id="resCompRI" style="margin-left:10px;font-weight:600;color:#166534;"></span>
            <span id="resCompShed" style="margin-left:10px;font-weight:600;color:#166534;"></span>
            <span id="resCompServed" style="margin-left:10px;font-weight:600;color:#166534;"></span>
          </div>
          <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px;">
            <div id="resilienceRestorationChart" class="chart" style="height:340px;"></div>
            <div id="resilienceMessChart" class="chart" style="height:340px;"></div>
            <div id="resilienceEnergyChart" class="chart" style="height:340px;"></div>
            <div id="resilienceIslandChart" class="chart" style="height:340px;"></div>
            <div id="resilienceProfileChart" class="chart" style="height:340px;"></div>
            <div id="resilienceTrajectoryChart" class="chart" style="height:340px;"></div>
            <div id="resiliencePriorityChart" class="chart" style="height:340px;"></div>
            <div id="resilienceMessSOCChart" class="chart" style="height:340px;"></div>
            <div id="resilienceVoltageChart" class="chart" style="height:340px;display:none;"></div>
            <div id="resilienceBranchFlowChart" class="chart" style="height:340px;display:none;"></div>
          </div>
        </div>
      </section>

      <!-- =================== DASHBOARD TAB ============================ -->
      <section id="dashPane" class="tabpane">
        <div class="btn-group">
          <button class="btn btn-primary" id="runDashBtn">&#9654; Run Full Analysis (TS-PF + Carbon)</button>
        </div>
        <div id="dashStatus" class="status mono" style="margin-bottom:8px;">Not run yet. Load a case and click Run.</div>
        <div id="dashKPIs" class="kpis"></div>
        <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:8px;">
          <div id="dashGenChart" class="chart" style="height:300px;"></div>
          <div id="dashCarbonChart" class="chart" style="height:300px;"></div>
          <div id="dashVoltChart" class="chart" style="height:300px;"></div>
          <div id="dashLossChart" class="chart" style="height:300px;"></div>
          <div id="dashESSChart" class="chart" style="height:300px;"></div>
          <div id="dashLoadChart" class="chart" style="height:300px;"></div>
        </div>
      </section>

    </div>
  </div>

  <div id="statusBox" class="status" style="display:flex;align-items:center;gap:10px;">
    <span id="statusText">Ready. Load a case to begin.</span>
    <button id="cancelBtn" style="display:none;padding:2px 12px;background:#c34d4d;color:#fff;border:none;border-radius:4px;cursor:pointer;font-size:12px;" onclick="cancelAnalysis()">Cancel</button>
  </div>
</div>

<script>
let SYS=null, activeComp='ac_buses', lastPfBody=null, lastPfCompareRows=[];
const statusBox=document.getElementById('statusBox');
const statusText=document.getElementById('statusText');
const cancelBtn=document.getElementById('cancelBtn');
function setStatus(m,e=false){statusText.textContent=m;statusBox.style.borderColor=e?'#c34d4d':'#bcc5bf';statusBox.style.background=e?'#fff1f1':'#fbfaf6';cancelBtn.style.display=m.includes('Running')?'inline-block':'none';}
async function cancelAnalysis(){try{await fetch('/api/session/cancel',{method:'POST'});setStatus('Cancellation requested...');}catch(e){console.error(e);}}
async function api(p,d,method='POST'){const o={method,headers:{'Content-Type':'application/json'}};if(d!==undefined)o.body=JSON.stringify(d);const r=await fetch(p,o);const t=await r.text();let b={};try{b=JSON.parse(t);}catch{b={error:t};}if(!r.ok||b.error)throw new Error(b.error||'HTTP '+r.status);return b;}
function fmt(v,d=4){if(v==null||v===undefined||Number.isNaN(Number(v)))return'-';return Number(v).toFixed(d);}

/* Power unit conversion helpers */
function getPowerUnit(){return document.getElementById('pfDisplayUnit').value||'MW';}
function pScale(){const u=getPowerUnit();if(u==='kW')return 1e3;if(u==='W')return 1e6;return 1;}
function pUnit(){return getPowerUnit();}
function pConv(mw){return mw*pScale();}
function pFmt(mw,d=2){return (mw*pScale()).toFixed(d);}
function qUnit(){const u=getPowerUnit();if(u==='kW')return 'kVar';if(u==='W')return 'Var';return 'MVar';}

document.querySelectorAll('.panel > .tabs > .tab').forEach(tab=>{
  tab.addEventListener('click',()=>{
    document.querySelectorAll('.panel > .tabs > .tab').forEach(t=>t.classList.remove('active'));
    document.querySelectorAll('.panel > .tabpane').forEach(p=>p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById(tab.dataset.tab).classList.add('active');
  });
});

/* Case lists */
async function loadCaseLists(){
  const [cases,mfiles]=await Promise.all([fetch('/api/cases').then(r=>r.json()),fetch('/api/matpower_files').then(r=>r.json())]);
  document.getElementById('builtinSelect').innerHTML=cases.cases.map(c=>`<option value="${c}"${c===cases.default_case?' selected':''}>${c}</option>`).join('');
  document.getElementById('matpowerSelect').innerHTML=mfiles.files.map(f=>`<option value="${f}">${f}</option>`).join('');
}

async function loadSystem(ep,payload){
  setStatus('Loading system...');
  try{const d=await api(ep,payload);SYS=d;updateSystemUI();setStatus('Loaded: '+SYS.name+' ('+SYS.counts.ac_buses+' AC buses, '+SYS.counts.generators+' gens)');}
  catch(e){setStatus(e.message,true);}
}

function updateSystemUI(){
  if(!SYS)return;
  document.getElementById('sysLabel').textContent=SYS.name;
  document.getElementById('sysLabel').className='badge badge-green';
  const cb=document.getElementById('countsBar');
  const c=SYS.counts;
  cb.innerHTML=Object.entries(c).map(([k,v])=>'<div class="count-chip"><span class="n">'+v+'</span>'+k.replace(/_/g,' ')+'</div>').join('');
  renderCompTable(activeComp);
  document.getElementById('jsonEditor').value=SYS._raw_json||'(use Refresh to load JSON)';
}

document.getElementById('loadBuiltinBtn').onclick=()=>loadSystem('/api/session/load_builtin',{case:document.getElementById('builtinSelect').value});
document.getElementById('loadMatpowerBtn').onclick=()=>loadSystem('/api/session/load_matpower',{filename:document.getElementById('matpowerSelect').value});
document.getElementById('uploadJsonBtn').onclick=()=>{
  const fi=document.getElementById('jsonFileInput');
  if(!fi.files.length){setStatus('Select a JSON file first.',true);return;}
  const reader=new FileReader();
  reader.onload=()=>loadSystem('/api/session/load_json_string',{json_string:reader.result});
  reader.readAsText(fi.files[0]);
};
document.getElementById('loadEtapXmlBtn').onclick=()=>{
  const fi=document.getElementById('etapXmlInput');
  if(!fi.files.length){setStatus('Select an ETAP project .xml file first.',true);return;}
  const reader=new FileReader();
  reader.onload=()=>loadSystem('/api/session/load_etap_xml',{xml_string:reader.result});
  reader.readAsText(fi.files[0]);
};
document.getElementById('exportJsonBtn').onclick=async()=>{
  try{const d=await api('/api/session/export_json',{},'POST');
  const blob=new Blob([d.json_string],{type:'application/json'});
  const a=document.createElement('a');a.href=URL.createObjectURL(blob);
  a.download=(SYS?SYS.name.replace(/[^a-zA-Z0-9_-]/g,'_'):'system')+'.json';a.click();
  setStatus('JSON exported.');}catch(e){setStatus(e.message,true);}
};
document.getElementById('exportEtapBtn').onclick=async()=>{
  try{setStatus('Exporting ETAP workbook...');
  const resp=await fetch('/api/session/export_etap',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  if(!resp.ok){let m='HTTP '+resp.status;try{const j=await resp.json();if(j&&j.error)m=j.error;}catch(e){}throw new Error(m);}
  const blob=await resp.blob();let fn='system.xlsx';const cd=resp.headers.get('Content-Disposition');
  if(cd){const mm=/filename="?([^"]+)"?/.exec(cd);if(mm&&mm[1])fn=mm[1];}
  const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=fn;a.click();URL.revokeObjectURL(a.href);
  setStatus('ETAP workbook exported: '+fn);}catch(e){setStatus(e.message,true);}
};
document.getElementById('newCaseBtn').onclick=()=>loadSystem('/api/session/new_empty',{});

/* Case Builder tables – grouped by domain */
const COMP_GROUPS=[
  {label:'AC Network',items:[
    ['ac_buses','Buses'],['ac_branches','Branches'],['transformers_2w','Trafo 2W'],['transformers_3w','Trafo 3W'],['switches','Switches'],['circuit_breakers','Breakers'],
  ]},
  {label:'AC Devices',items:[
    ['generators','Generators'],['loads','Loads'],['shunts','Shunts'],['storage','Storage'],
    ['static_generators','Static Gens'],['flexible_loads','Flex Loads'],['asymmetric_loads','Asym Loads'],
    ['renewable_gens','Renewables'],['pv_systems','PV Systems'],['external_grids','Ext Grids'],
    ['charging_stations','EV Stations'],['chargers','Chargers'],['motors','Motors'],
  ]},
  {label:'DC Network',items:[
    ['dc_buses','Buses'],['dc_branches','Branches'],['vsc_converters','VSC Conv'],['dcdc_converters','DC/DC Conv'],['dc_circuit_breakers','Breakers'],
  ]},
  {label:'DC Devices',items:[
    ['dc_loads','Loads'],['dc_storage','Storage'],['dc_static_generators','Static Gens'],
    ['dc_native_static_generators','Native SGs'],['pv_arrays','PV Arrays'],
    ['energy_routers','E-Routers'],['mobile_storage','Mobile Stor'],['vpps','VPPs'],['microgrids','Microgrids'],
  ]},
  {label:'Three-Phase',items:[
    ['tp_buses','Buses'],['tp_lines','Lines'],['tp_transformers','Trafos'],
    ['tp_loads','Loads'],['tp_generators','Generators'],['tp_external_grids','Ext Grids'],
  ]},
];
/* flat list for legacy compat */
const COMP_META=COMP_GROUPS.flatMap(g=>g.items);

const COMP_COLS={
  ac_buses:[{key:'index',label:'#',type:'int'},{key:'name',label:'Name',type:'str'},{key:'bus_type',label:'Type',type:'select',options:['PQ','PV','SLACK','ISOLATED']},{key:'base_kv',label:'Base kV',type:'num'},{key:'vm_pu',label:'Vm pu',type:'num'},{key:'va_deg',label:'Va deg',type:'num'},{key:'pd_mw',label:'Pd MW',type:'num'},{key:'qd_mvar',label:'Qd MVAr',type:'num'},{key:'vmax_pu',label:'Vmax',type:'num'},{key:'vmin_pu',label:'Vmin',type:'num'},{key:'i_breaker_ka',label:'Ib kA',type:'num'},{key:'area',label:'Area',type:'int'},{key:'zone',label:'Zone',type:'int'},{key:'in_service',label:'In Svc',type:'bool'}],
  dc_buses:[{key:'index',label:'#',type:'int'},{key:'name',label:'Name',type:'str'},{key:'bus_type',label:'Type',type:'select',options:['DC_P','DC_V']},{key:'vm_pu',label:'Vm pu',type:'num'},{key:'pd_mw',label:'Pd MW',type:'num'},{key:'base_kv',label:'Base kV',type:'num'},{key:'in_service',label:'In Svc',type:'bool'}],
};

const COMP_DEFAULTS={
  ac_buses:{index:0,name:'',bus_type:'PQ',base_kv:110,vm_pu:1.0,va_deg:0,pd_mw:0,qd_mvar:0,vmax_pu:1.05,vmin_pu:0.95,i_breaker_ka:0,area:1,zone:1,in_service:true},
  ac_branches:{index:0,name:'',from_bus:1,to_bus:2,r_pu:0.01,x_pu:0.1,b_pu:0,tap:1.0,shift_deg:0,rate_a_mva:100,in_service:true},
  generators:{index:0,name:'',bus:1,is_slack:false,pg_mw:0,qg_mvar:0,vg_pu:1.0,pmax_mw:100,pmin_mw:0,qmax_mvar:50,qmin_mvar:-50,cost_c2:0,cost_c1:20,cost_c0:0,in_service:true},
  loads:{index:0,name:'',bus:1,p_mw:10,q_mvar:5,in_service:true},
  storage:{index:0,name:'',bus:1,p_mw:0,p_rated_mw:10,e_rated_mwh:40,soc_init:0.5,eta_charge:0.95,eta_discharge:0.95,in_service:true},
  dc_buses:{index:0,name:'',bus_type:'DC_P',vm_pu:1.0,pd_mw:0,base_kv:320,in_service:true},
  dc_branches:{index:0,name:'',from_bus:1,to_bus:2,r_pu:0.01,rate_a_mva:100,in_service:true},
  vsc_converters:{index:0,name:'',bus_ac:1,bus_dc:1,p_set_mw:0,q_set_mvar:0,v_dc_set_pu:1.0,eta:0.98,pmax_mw:100,p_rated_mw:100,in_service:true},
  dcdc_converters:{index:0,name:'',bus_in:1,bus_out:2,p_ref_mw:0,v_ref_pu:1.0,eta:0.98,in_service:true},
  energy_routers:{index:0,name:'',router_type:'SST',num_ports:0,ports:[],in_service:true},
  mobile_storage:{index:0,name:'',bus:1,p_mw:0,e_rated_mwh:20,soc_init:0.5,in_service:true},
  vpps:{index:0,name:'',pcc_bus:1,p_output_mw:0,q_output_mvar:0,in_service:true,aggregated_gen_ids:[],aggregated_storage_ids:[],aggregated_load_ids:[]},
  microgrids:{index:0,name:'',pcc_bus:1,p_exchange_mw:0,in_service:true},
  circuit_breakers:{index:0,name:'',bus_from:1,bus_to:2,breaker_type:'CB',closed:true,z_ohm:0,rated_voltage_kv:110,i_rated_ka:2.0,i_breaking_ka:40,element_type:'l',element_id:0,in_service:true},
  dc_circuit_breakers:{index:0,name:'',bus_from:1,bus_to:2,breaker_type:'CB',closed:true,r_ohm:0,rated_voltage_kv:320,i_rated_ka:2.0,i_breaking_ka:20,element_type:'l',element_id:0,in_service:true},
  transformers_2w:{index:0,name:'',std_type:'',hv_bus:1,lv_bus:2,in_service:true,sn_mva:25,vn_hv_kv:110,vn_lv_kv:20,vk_percent:10,vkr_percent:0.3,pk_kw:30,pfe_kw:20,i0_percent:0.5,tap_side:'hv',tap_pos:0,tap_min:-10,tap_max:10,tap_neutral:0,tap_step_percent:1.5,shift_deg:0,vector_group:'Dyn'},
  transformers_3w:{index:0,name:'',std_type:'',hv_bus:1,mv_bus:2,lv_bus:3,in_service:true,sn_hv_mva:40,sn_mv_mva:20,sn_lv_mva:10,vn_hv_kv:110,vn_mv_kv:20,vn_lv_kv:10,vk_hv_mv_percent:10,vk_hv_lv_percent:10,vk_mv_lv_percent:10,vkr_hv_mv_percent:0.3,vkr_hv_lv_percent:0.3,vkr_mv_lv_percent:0.3,pfe_kw:30,i0_percent:0.5,tap_side:'hv',tap_pos:0,tap_step_percent:1.5,shift_mv_deg:0,shift_lv_deg:0},
  switches:{index:0,name:'',bus_from:1,bus_to:2,in_service:true,switch_type:'CB',closed:true,r_contact_ohm:0,z_ohm:0,i_rated_ka:2.0,i_breaking_ka:40,element_type:'l',element_id:0},
  shunts:{index:0,name:'',bus:1,in_service:true,gs_mw:0,bs_mvar:0,switchable:false,n_steps:1,current_step:1,bs_per_step:0},
  static_generators:{index:0,name:'',bus:1,in_service:true,sgen_type:'PQ',p_mw:0,q_mvar:0,p_rated_mw:10,sn_mva:10,pmax_mw:10,pmin_mw:0,qmax_mvar:5,qmin_mvar:-5,scaling:1.0,controllable:false},
  flexible_loads:{index:0,name:'',bus:1,in_service:true,p_mw:10,q_mvar:5,flex_up_mw:2,flex_down_mw:2,flex_duration_h:1,response_time_s:60,ramp_rate_mw_min:1,availability_pct:95,controllable:true,priority:1,control_area:''},
  asymmetric_loads:{index:0,name:'',bus:1,in_service:true,connection:'wye',grounded:true,pa_mw:0,qa_mvar:0,pb_mw:0,qb_mvar:0,pc_mw:0,qc_mvar:0,scaling:1.0,controllable:false},
  renewable_gens:{index:0,name:'',bus:1,in_service:true,type:'wind',p_mw:0,q_mvar:0,p_rated_mw:10,qmax_mvar:5,qmin_mvar:-5,curtailable:true,cost_curtail_mwh:0,capacity_factor:0.3,profile_id:''},
  pv_systems:{index:0,name:'',bus:1,in_service:true,p_mw:0,q_mvar:0,sn_mva:10,pmax_mw:10,pmin_mw:0,qmax_mvar:5,qmin_mvar:-5,control_mode:'PQ',controllable:false},
  external_grids:{index:0,name:'',bus:1,in_service:true,vm_pu:1.0,va_deg:0,s_sc_max_mva:1000,s_sc_min_mva:500,rx_max:0.1,rx_min:0.1},
  charging_stations:{index:0,name:'',bus:1,location:'',in_service:true,n_fast:2,n_slow:4,num_chargers:6,p_fast_max_kw:150,p_slow_max_kw:22,max_power_kw:500,simultaneity_factor:0.8,power_factor:0.95,utilization_rate:0.3,p_total_kw:0,q_total_kvar:0},
  chargers:{index:0,name:'',station_id:0,charger_type:'fast',in_service:true,p_rated_kw:150,p_ch_max_kw:150,p_ch_min_kw:0,eta:0.95,v2g_capable:false,p_dis_max_kw:0},
  motors:{index:0,name:'',bus:1,in_service:true,vn_kv:0.4,sn_mva:0.1,r_pu:0.02,x_pu:0.15,x_r:7.5,lrc:6.0,poles:4,cos_phi:0.85,efficiency:0.9},
  dc_loads:{index:0,name:'',bus:1,in_service:true,p_mw:10,controllable:false,p_min_mw:0,cost_mw:0,profile_id:''},
  dc_storage:{index:0,name:'',bus:1,in_service:true,p_mw:0,p_rated_mw:10,e_rated_mwh:40,soc_init:0.5,eta_charge:0.95,eta_discharge:0.95},
  dc_static_generators:{index:0,name:'',bus:1,in_service:true,sgen_type:'PQ',p_mw:0,q_mvar:0,p_rated_mw:10,sn_mva:10,pmax_mw:10,pmin_mw:0,qmax_mvar:5,qmin_mvar:-5,scaling:1.0,controllable:false},
  dc_native_static_generators:{index:0,name:'',bus:1,in_service:true,type:'PQ',p_set_mw:0,scaling:1.0,profile_id:'',pmax_mw:10,pmin_mw:0,controllable:false},
  pv_arrays:{index:0,name:'',bus:1,in_service:true,p_set_mw:0,profile_id:'',num_series:10,num_parallel:5,vmpp:30,impp:8,voc:37,isc:9,irradiance:1000,temperature:25},
  tp_buses:{index:0,name:'',bus_type:'PQ',base_kv:20,in_service:true,vm_a_pu:1.0,va_a_deg:0,vm_b_pu:1.0,va_b_deg:-120,vm_c_pu:1.0,va_c_deg:120,vmin_pu:0.95,vmax_pu:1.05,area:1,zone:1},
  tp_lines:{index:0,name:'',from_bus:1,to_bus:2,in_service:true,length_km:1,parallel:1,r1_ohm_per_km:0.1,x1_ohm_per_km:0.1,c1_nf_per_km:10,r0_ohm_per_km:0.3,x0_ohm_per_km:0.3,c0_nf_per_km:5,max_i_ka:0.5,rate_a_mva:20},
  tp_transformers:{index:0,name:'',hv_bus:1,lv_bus:2,in_service:true,sn_mva:25,vn_hv_kv:110,vn_lv_kv:20,vk_percent:10,vkr_percent:0.3,pfe_kw:20,i0_percent:0.5,vector_group:'Dyn',tap_side:'hv',tap_pos:0,tap_min:-10,tap_max:10,tap_neutral:0,tap_step_percent:1.5,shift_deg:0},
  tp_loads:{index:0,name:'',bus:1,in_service:true,connection:'wye',grounded:true,p_a_mw:0,q_a_mvar:0,p_b_mw:0,q_b_mvar:0,p_c_mw:0,q_c_mvar:0},
  tp_generators:{index:0,name:'',bus:1,in_service:true,is_slack:false,p_mw:0,q_mvar:0,vm_pu:1.0,pmax_mw:100,pmin_mw:0,qmax_mvar:50,qmin_mvar:-50,mbase_mva:100},
  tp_external_grids:{index:0,name:'',bus:1,in_service:true,vm_pu:1.0,va_deg:0,s_sc_max_mva:1000,s_sc_min_mva:500,rx_max:0.1,rx_min:0.1},
};

function prettyLabel(k){return k.replace(/_/g,' ').replace(/\b\w/g,m=>m.toUpperCase());}
function inferType(v){if(typeof v==='boolean')return'bool';if(typeof v==='number')return'num';if(v===null||v===undefined)return'num';if(typeof v==='object')return'json';return'str';}
function inferCols(comp,rows){
  if(COMP_COLS[comp])return COMP_COLS[comp];
  const keys=new Set(Object.keys(COMP_DEFAULTS[comp]||{}));
  rows.forEach(r=>Object.keys(r||{}).forEach(k=>keys.add(k)));
  return [...keys].map(k=>{
    let sample=null;
    for(const r of rows){if(r&&r[k]!==undefined&&r[k]!==null){sample=r[k];break;}}
    if(sample===null&&COMP_DEFAULTS[comp]&&COMP_DEFAULTS[comp][k]!==undefined)sample=COMP_DEFAULTS[comp][k];
    return {key:k,label:(k==='index'?'#':prettyLabel(k)),type:inferType(sample)};
  });
}

function initCompTabs(){
  const ct=document.getElementById('compTabs');
  let html='',first=true;
  COMP_GROUPS.forEach(g=>{
    html+='<div style="display:flex;flex-wrap:wrap;align-items:center;margin-bottom:2px;">';
    html+='<span style="margin:2px 6px 2px 0;padding:2px 6px;font-size:11px;font-weight:700;color:var(--muted);text-transform:uppercase;border-left:3px solid var(--accent);user-select:none;white-space:nowrap;">'+g.label+'</span>';
    g.items.forEach(([k,l])=>{
      html+='<div class="tab'+(first?' active':'')+'" data-comp="'+k+'">'+l+'</div>';
      first=false;
    });
    html+='</div>';
  });
  ct.innerHTML=html;
  ct.querySelectorAll('.tab').forEach(tab=>tab.addEventListener('click',()=>{
    ct.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
    tab.classList.add('active');
    activeComp=tab.dataset.comp;
    renderCompTable(activeComp);
  }));
}
initCompTabs();

function renderCompTable(comp){
  const area=document.getElementById('compTableArea');
  if(!SYS){area.innerHTML='<div style="padding:20px;color:var(--muted);">Load a system first.</div>';return;}
  if(!SYS[comp]||!Array.isArray(SYS[comp]))SYS[comp]=[];
  const rows=SYS[comp];
  const cols=inferCols(comp,rows);
  let h='<div class="dtable-wrap"><table class="dtable"><thead><tr>';
  cols.forEach(c=>h+='<th>'+c.label+'</th>');h+='<th></th></tr></thead><tbody>';
  rows.forEach((row,ri)=>{
    h+='<tr data-ri="'+ri+'">';
    cols.forEach(c=>{
      const v=row[c.key]??'';
      if(c.type==='bool')h+='<td><input type="checkbox" data-key="'+c.key+'" '+(v?'checked':'')+' style="width:auto;min-width:auto;"/></td>';
      else if(c.type==='select'){h+='<td><select data-key="'+c.key+'">';c.options.forEach(o=>h+='<option'+(o===String(v)?' selected':'')+'>'+o+'</option>');h+='</select></td>';}
      else if(c.type==='json')h+='<td><input type="text" data-key="'+c.key+'" value="'+String(JSON.stringify(v)).replace(/"/g,'&quot;')+'"/></td>';
      else h+='<td><input type="'+(c.type==='str'?'text':'number')+'" data-key="'+c.key+'" value="'+v+'" '+(c.type==='num'?'step="any"':'')+'/></td>';
    });
    h+='<td><button class="row-del" data-ri="'+ri+'" title="Delete row">&times;</button></td></tr>';
  });
  h+='</tbody></table></div>';area.innerHTML=h;
  area.querySelectorAll('.row-del').forEach(btn=>btn.addEventListener('click',()=>{SYS[comp].splice(parseInt(btn.dataset.ri),1);renderCompTable(comp);}));
  area.querySelectorAll('input,select').forEach(el=>el.addEventListener('change',()=>{
    const ri=parseInt(el.closest('tr').dataset.ri),key=el.dataset.key,col=cols.find(c=>c.key===key);
    if(col.type==='bool')SYS[comp][ri][key]=el.checked;
    else if(col.type==='int')SYS[comp][ri][key]=parseInt(el.value)||0;
    else if(col.type==='num')SYS[comp][ri][key]=parseFloat(el.value)||0;
    else if(col.type==='json'){try{SYS[comp][ri][key]=JSON.parse(el.value);}catch{SYS[comp][ri][key]=el.value;}}
    else SYS[comp][ri][key]=el.value;
  }));
}

document.getElementById('addRowBtn').onclick=()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  if(!SYS[activeComp])SYS[activeComp]=[];
  const def={index:0,name:'',in_service:true,...(COMP_DEFAULTS[activeComp]||{})};
  def.index=SYS[activeComp].reduce((mx,r)=>Math.max(mx,r.index||0),0)+1;
  SYS[activeComp].push(def);renderCompTable(activeComp);
  const w=document.querySelector('.dtable-wrap');if(w)w.scrollTop=w.scrollHeight;
};

document.getElementById('commitBtn').onclick=async()=>{
  if(!SYS){setStatus('No system to commit.',true);return;}
  try{setStatus('Committing changes...');const d=await api('/api/session/update_components',SYS);SYS=d;updateSystemUI();setStatus('Changes committed successfully.');}
  catch(e){setStatus(e.message,true);}
};

/* JSON editor */
document.getElementById('refreshJsonBtn').onclick=async()=>{try{const d=await api('/api/session/export_json',{},'POST');document.getElementById('jsonEditor').value=d.json_string;setStatus('JSON refreshed.');}catch(e){setStatus(e.message,true);}};
document.getElementById('applyJsonBtn').onclick=async()=>{const j=document.getElementById('jsonEditor').value.trim();if(!j){setStatus('JSON editor is empty.',true);return;}loadSystem('/api/session/load_json_string',{json_string:j});};

/* Power Flow */
const PF_METHOD_LABEL={
  ac_newton:'AC Newton-Raphson',dc:'DC PF',hybrid_linearized:'Hybrid AC/DC Linearized',fdpf:'Fast-Decoupled PF',
  adaptive:'Adaptive PF',islanded:'Islanded PF',distributed_slack:'Distributed Slack PF',three_phase:'Three-Phase PF',
};

async function runPfMethod(method){
  const body=await api('/api/session/pf',{method,options:{
    max_iter:Number(document.getElementById('pfMaxIter').value||80),
    tol:Number(document.getElementById('pfTol').value||1e-8),
    fdpf_max_iter:Number(document.getElementById('pfFdpfMaxIter').value||1000),
    enable_pv_pq_conversion:document.getElementById('pfPvPq').checked,
    enable_auto_swing_selection:document.getElementById('pfAutoSwing').checked,
    enable_converter_mode_switching:document.getElementById('pfConvSwitch').checked,
    enable_converter_coordination_check:true,
    verbose:document.getElementById('pfVerbose').checked,
    pv_q_hysteresis_pu:Number(document.getElementById('pfPvQHyst').value||0.01),
    max_delta_va_rad:Number(document.getElementById('pfMaxDVa').value||1.5),
    max_delta_vm_pu:Number(document.getElementById('pfMaxDVm').value||0.5),
    loss_model:document.getElementById('pfLossModel').value,
  }});
  body.method=method;
  return body;
}

function renderPf(body){
  document.getElementById('pfConv').textContent=body.converged?'Yes':'No';
  document.getElementById('pfIter').textContent=body.iterations;
  document.getElementById('pfRes').textContent=fmt(body.residual,3);
  document.getElementById('pfNBus').textContent=(body.vm||[]).length;
  const coord=body.converter_coordination;
  if(coord&&coord.enabled){
    const blocking=(coord.blocking_count!=null)?coord.blocking_count:((coord.fatal_count||0)+(coord.error_count||0));
    const lines=[`Converter coordination: ${coord.feasible?'feasible':'infeasible'}; blocking=${blocking||0}; fatal=${coord.fatal_count||0}; error=${coord.error_count||0}; warnings=${coord.warning_count||0}`];
    (coord.issues||[]).forEach(i=>lines.push(`${i.severity||''} ${i.rule_id||''} ${i.component_type||''}${i.component_index>=0?'#'+i.component_index:''}: ${i.message||''}`));
    document.getElementById('pfCompareTable').textContent=lines.join('\n');
  }

  const u=pUnit(), sc=pScale(), qu=qUnit();
  const vm=body.vm||[];
  const branch=body.branch_abs||[];
  const geoBuses=body.geo_buses||[];
  const acBusIds=geoBuses.filter(b=>b.type==='AC').map(b=>b.id);
  const dcBusIds=geoBuses.filter(b=>b.type==='DC').map(b=>b.id);
  const acVoltX=vm.map((_,i)=>acBusIds[i]??`pos ${i}`);
  const dcVoltX=(body.vdc||[]).map((_,i)=>dcBusIds[i]??`pos ${i}`);
  const branchX=(body.geo_ac_branches||[]).length
    ? (body.geo_ac_branches||[]).map(b=>b.index??`${b.from}->${b.to}`)
    : branch.map((_,i)=>`pos ${i}`);
  Plotly.newPlot('pfVoltChart',[{x:acVoltX,y:vm,mode:'lines+markers',line:{color:'#0b6e4f',width:2},marker:{size:5},name:'Vm'}],
    {title:`AC Voltage Magnitude (${PF_METHOD_LABEL[body.method]||body.method})`,xaxis:{title:'AC Bus'},yaxis:{title:'p.u.'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});

  /* DC Bus Voltages */
  const vdc=body.vdc||[];
  if(vdc.length>0){
    Plotly.newPlot('pfDcVoltChart',[{x:dcVoltX,y:vdc,mode:'lines+markers',line:{color:'#6a0dad',width:2},marker:{size:5,color:'#6a0dad'},name:'Vdc'}],
      {title:'DC Bus Voltage',xaxis:{title:'DC Bus'},yaxis:{title:'p.u.'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  } else { Plotly.purge('pfDcVoltChart'); }

  /* VSC Converter Transfers */
  const vsc=body.vsc_transfers||[];
  if(vsc.length>0){
    const labels=vsc.map(v=>'VSC'+v.index+' (AC'+v.bus_ac+'↔DC'+v.bus_dc+')');
    Plotly.newPlot('pfConverterChart',[
      {x:labels,y:vsc.map(v=>pConv(v.p_ac_mw)),type:'bar',name:`P_ac (${u})`,marker:{color:'#0b6e4f'}},
      {x:labels,y:vsc.map(v=>pConv(v.p_dc_mw)),type:'bar',name:`P_dc (${u})`,marker:{color:'#6a0dad'}},
      {x:labels,y:vsc.map(v=>pConv(v.loss_mw)),type:'bar',name:`Loss (${u})`,marker:{color:'#c34d4d'}}
    ],{title:'VSC Converter Power Transfers',barmode:'group',xaxis:{title:'Converter'},yaxis:{title:u},margin:{l:55,r:15,t:45,b:80}},{responsive:true});
  } else { Plotly.purge('pfConverterChart'); }

  Plotly.newPlot('pfBranchChart',[{x:branchX,y:branch.map(v=>pConv(v)),type:'bar',marker:{color:'#2c8c99'}}],
    {title:`Branch |P|`,xaxis:{title:'Branch'},yaxis:{title:u},margin:{l:55,r:15,t:45,b:45}},{responsive:true});

  // GIS Map
  renderGeoMap(body);
}

function renderPfCompare(rows){
  if(!rows.length){document.getElementById('pfCompareTable').textContent='No comparison results yet.';return;}
  const methods=rows.map(r=>PF_METHOD_LABEL[r.method]||r.method);
  const iters=rows.map(r=>r.iterations||0);
  const residuals=rows.map(r=>Number(r.residual||0));
  Plotly.newPlot('pfCompareChart',[
    {x:methods,y:iters,type:'bar',name:'Iterations',marker:{color:'#0b6e4f'}},
    {x:methods,y:residuals,type:'scatter',mode:'lines+markers',name:'Residual',yaxis:'y2',line:{color:'#b5651d',width:2}},
  ],{title:'PF Method Comparison',yaxis:{title:'Iterations'},yaxis2:{title:'Residual',overlaying:'y',side:'right'},margin:{l:55,r:55,t:45,b:80}}, {responsive:true});
  const lines=['Method | Conv | Iter | Residual | |Vm| count'];
  rows.forEach(r=>lines.push(`${PF_METHOD_LABEL[r.method]||r.method} | ${r.converged?'Y':'N'} | ${r.iterations} | ${fmt(r.residual,4)} | ${(r.vm||[]).length}`));
  document.getElementById('pfCompareTable').textContent=lines.join('\n');
}

function renderGeoMap(body){
  const buses=body.geo_buses||[];
  const acBranches=body.geo_ac_branches||[];
  const dcBranches=body.geo_dc_branches||[];
  const vsc=body.geo_vsc||[];
  const dcdc=body.geo_dcdc||[];
  const u=pUnit(), qu=qUnit();
  
  if(!buses.length){Plotly.purge('pfGeoMap');return;}
  
  // Check if we have valid geo coordinates (not all zeros)
  const hasGeo=buses.some(b=>Math.abs(b.lat)>0.001||Math.abs(b.lon)>0.001);
  
  const traces=[];
  
  // AC Branches as lines
  for(const br of acBranches){
    traces.push({
      type:'scattergeo',
      mode:'lines',
      lon:[br.from_lon,br.to_lon],
      lat:[br.from_lat,br.to_lat],
      line:{width:Math.max(1,Math.min(6,(br.loading_pct||0)/20)),color:'#0b6e4f'},
      hoverinfo:'text',
      text:`AC ${br.from}-${br.to}: Pf=${pFmt(br.pf_mw||0)} Pt=${pFmt(br.pt_mw||0)} Loss=${pFmt(br.loss_mw||0,3)} ${u} (${(br.loading_pct||0).toFixed(1)}%)`,
      showlegend:false
    });
  }
  
  // DC Branches as lines
  for(const br of dcBranches){
    traces.push({
      type:'scattergeo',
      mode:'lines',
      lon:[br.from_lon,br.to_lon],
      lat:[br.from_lat,br.to_lat],
      line:{width:2,color:'#6a0dad',dash:'dash'},
      hoverinfo:'text',
      text:`DC ${br.from}-${br.to}: ${pFmt(br.pf_mw||0)} ${u}`,
      showlegend:false
    });
  }
  
  // VSC connections
  for(const v of vsc){
    traces.push({
      type:'scattergeo',
      mode:'lines',
      lon:[v.ac_lon,v.dc_lon],
      lat:[v.ac_lat,v.dc_lat],
      line:{width:3,color:'#b5651d'},
      hoverinfo:'text',
      text:`VSC: AC${v.bus_ac}↔DC${v.bus_dc} P=${pFmt(v.p_ac_mw||0)}${u}`,
      showlegend:false
    });
  }
  
  // DC-DC converter connections
  for(const d of dcdc){
    traces.push({
      type:'scattergeo',
      mode:'lines',
      lon:[d.in_lon,d.out_lon],
      lat:[d.in_lat,d.out_lat],
      line:{width:3,color:'#c34d4d',dash:'dot'},
      hoverinfo:'text',
      text:`DCDC: DC${d.bus_in}↔DC${d.bus_out} P=${pFmt(d.p_in_mw||0)}${u}`,
      showlegend:false
    });
  }
  
  // AC buses with dynamic voltage color range
  const acBuses=buses.filter(b=>b.type==='AC');
  if(acBuses.length){
    const vmVals=acBuses.map(b=>b.vm_pu||1.0);
    const vmMin=Math.min(...vmVals);
    const vmMax=Math.max(...vmVals);
    const vmRange=Math.max(vmMax-vmMin,0.005);
    const cmin=Math.max(0.9, vmMin-vmRange*0.3);
    const cmax=Math.min(1.1, vmMax+vmRange*0.3);
    traces.push({
      type:'scattergeo',
      mode:'markers+text',
      lon:acBuses.map(b=>b.lon),
      lat:acBuses.map(b=>b.lat),
      marker:{size:acBuses.map(b=>b.bus_type==='SLACK'?14:10),color:vmVals,colorscale:'RdYlGn',cmin:cmin,cmax:cmax,colorbar:{title:'Vm (pu)',x:1.02,thickness:12,len:0.7}},
      text:acBuses.map(b=>b.name||'AC'+b.id),
      textposition:'top center',
      textfont:{size:9},
      hoverinfo:'text',
      hovertext:acBuses.map(b=>`AC${b.id} ${b.name||''}<br>Vm=${(b.vm_pu||1.0).toFixed(4)} Va=${((b.va_rad||0)*180/Math.PI).toFixed(2)}°<br>P=${pFmt(b.pd_mw||0)}${u} Q=${pFmt(b.qd_mvar||0)}${qu}`),
      name:'AC Buses',
      showlegend:true
    });
  }
  
  // DC buses with voltage-based coloring
  const dcBuses=buses.filter(b=>b.type==='DC');
  if(dcBuses.length){
    const vdcVals=dcBuses.map(b=>b.vm_pu||1.0);
    const vdcMin=Math.min(...vdcVals);
    const vdcMax=Math.max(...vdcVals);
    const vdcRange=Math.max(vdcMax-vdcMin,0.005);
    traces.push({
      type:'scattergeo',
      mode:'markers+text',
      lon:dcBuses.map(b=>b.lon),
      lat:dcBuses.map(b=>b.lat),
      marker:{size:12,color:vdcVals,colorscale:'Purples',cmin:Math.max(0.9,vdcMin-vdcRange*0.3),cmax:Math.min(1.1,vdcMax+vdcRange*0.3),symbol:'square'},
      text:dcBuses.map(b=>b.name||'DC'+b.id),
      textposition:'top center',
      textfont:{size:9},
      hoverinfo:'text',
      hovertext:dcBuses.map(b=>`DC${b.id} ${b.name||''}<br>Vdc=${(b.vm_pu||1.0).toFixed(4)}<br>P=${pFmt(b.pd_mw||0)}${u}`),
      name:'DC Buses',
      showlegend:true
    });
  }
  
  const layout={
    title:'Network GIS Visualization',
    geo:{
      scope:hasGeo?undefined:'usa',
      projection:{type:hasGeo?'mercator':'albers usa'},
      showland:true,landcolor:'#f5f5dc',
      showlakes:true,lakecolor:'#a0d2db',
      showcountries:true,countrycolor:'#888',
      showsubunits:true,subunitcolor:'#aaa',
      resolution:hasGeo?50:110,
      lonaxis:hasGeo?(()=>{const lons=buses.map(b=>b.lon);const minL=Math.min(...lons),maxL=Math.max(...lons);const span=maxL-minL;const pad=Math.max(0.002,span*0.08);return{range:[minL-pad,maxL+pad]};})():undefined,
      lataxis:hasGeo?(()=>{const lats=buses.map(b=>b.lat);const minL=Math.min(...lats),maxL=Math.max(...lats);const span=maxL-minL;const pad=Math.max(0.002,span*0.08);return{range:[minL-pad,maxL+pad]};})():undefined
    },
    margin:{l:0,r:0,t:40,b:0},
    legend:{x:0,y:1,bgcolor:'rgba(255,255,255,0.7)'}
  };
  
  Plotly.newPlot('pfGeoMap',traces,layout,{responsive:true});
}

// --- Market GIS Map: buses coloured by average LMP, branches/DC links overlaid ---
function renderMktGeoMap(body, lmp, sced){
  const el=document.getElementById('mktGeoMap');
  if(!el)return;
  if(!SYS||!SYS.ac_buses||!SYS.ac_buses.length){el.innerHTML='<p style=\"color:#999\">No system loaded.</p>';return;}

  // Build bus coord map from the loaded system
  const busList=SYS.ac_buses||[];
  const brList=SYS.ac_branches||[];
  const dcBusList=SYS.dc_buses||[];
  const dcBrList=SYS.dc_branches||[];
  const vscList=SYS.vsc_converters||[];

  const hasGeo=busList.some(b=>(Math.abs(b.latitude||0)>0.001||Math.abs(b.longitude||0)>0.001));
  if(!hasGeo){el.innerHTML='<p style=\"color:#999\">No geographic coordinates available for this case.</p>';return;}

  // Average LMP per bus across all periods
  const nodalLmp=lmp.nodal_lmp||[];
  const T=nodalLmp.length>0?(nodalLmp[0]||[]).length:0;
  const avgLmp=nodalLmp.map(arr=>{
    if(!arr||!arr.length)return 0;
    return arr.reduce((s,v)=>s+Number(v||0),0)/arr.length;
  });
  const maxLmp=Math.max(1,...avgLmp);
  const minLmp=Math.min(0,...avgLmp);

  // Province colour palette
  const areaColors=['#666','#2c8c99','#ff9800','#6a0dad','#27ae60','#d32f2f'];

  const traces=[];

  // AC Branches
  for(const br of brList){
    const fb=busList.find(b=>b.index===br.from_bus);
    const tb=busList.find(b=>b.index===br.to_bus);
    if(!fb||!tb)continue;
    const fromArea=fb.area||0, toArea=tb.area||0;
    const isInterArea=fromArea!==toArea;
    traces.push({
      type:'scattergeo',mode:'lines',
      lon:[fb.longitude,tb.longitude],lat:[fb.latitude,tb.latitude],
      line:{width:isInterArea?2.5:1.5,color:isInterArea?'#333':'#aaa',dash:isInterArea?'solid':'solid'},
      hoverinfo:'text',
      text:'AC '+(br.name||br.index)+' ('+fmt(br.rate_a_mva||0,0)+' MVA)',
      showlegend:false
    });
  }

  // DC Branches (dashed purple)
  for(const dbr of dcBrList){
    const fb=dcBusList.find(b=>b.index===dbr.from_bus);
    const tb=dcBusList.find(b=>b.index===dbr.to_bus);
    if(!fb||!tb)continue;
    traces.push({
      type:'scattergeo',mode:'lines',
      lon:[fb.longitude,tb.longitude],lat:[fb.latitude,tb.latitude],
      line:{width:3,color:'#6a0dad',dash:'dash'},
      hoverinfo:'text',
      text:'HVDC '+(dbr.name||dbr.index)+' ('+fmt(dbr.rate_a_mva||dbr.s_max_mva||0,0)+' MW)',
      showlegend:false
    });
  }

  // AC buses coloured by average LMP
  if(busList.length){
    traces.push({
      type:'scattergeo',mode:'markers+text',
      lon:busList.map(b=>b.longitude||0),
      lat:busList.map(b=>b.latitude||0),
      marker:{
        size:busList.map((b,i)=>{const bt=b.bus_type||'PQ';return(bt==='SLACK'||bt===3||bt==='3')?14:10;}),
        color:avgLmp.length>=busList.length?avgLmp.slice(0,busList.length):busList.map(()=>0),
        colorscale:[[0,'#2166ac'],[0.25,'#67a9cf'],[0.5,'#f7f7f7'],[0.75,'#ef8a62'],[1,'#b2182b']],
        cmin:minLmp,cmax:maxLmp,
        colorbar:{title:'Avg LMP ($/MWh)',x:1.02,thickness:12,len:0.7},
        line:{width:1,color:'#333'}
      },
      text:busList.map(b=>b.name||('Bus '+b.index)),
      textposition:'top center',
      textfont:{size:8},
      hoverinfo:'text',
      hovertext:busList.map((b,i)=>{
        const al=avgLmp[i]||0;
        return (b.name||'Bus '+b.index)+'<br>Area '+b.area+', Zone '+b.zone+
          '<br>Avg LMP: $'+fmt(al,2)+'/MWh'+
          '<br>Load: '+fmt(b.pd_mw||0,1)+' MW';
      }),
      name:'AC Buses (LMP)',
      showlegend:true
    });
  }

  // DC buses (squares)
  if(dcBusList.length&&dcBusList.some(b=>Math.abs(b.latitude||0)>0.001)){
    traces.push({
      type:'scattergeo',mode:'markers+text',
      lon:dcBusList.map(b=>b.longitude||0),
      lat:dcBusList.map(b=>b.latitude||0),
      marker:{size:10,color:'#6a0dad',symbol:'square'},
      text:dcBusList.map(b=>b.name||('DC '+b.index)),
      textposition:'top center',
      textfont:{size:8},
      hoverinfo:'text',
      hovertext:dcBusList.map(b=>(b.name||'DC '+b.index)),
      name:'DC Buses',
      showlegend:true
    });
  }

  const lons=busList.map(b=>b.longitude||0).concat(dcBusList.map(b=>b.longitude||0));
  const lats=busList.map(b=>b.latitude||0).concat(dcBusList.map(b=>b.latitude||0));
  const lonMin=Math.min(...lons),lonMax=Math.max(...lons);
  const latMin=Math.min(...lats),latMax=Math.max(...lats);
  const lonPad=Math.max(0.5,(lonMax-lonMin)*0.08);
  const latPad=Math.max(0.3,(latMax-latMin)*0.08);

  Plotly.newPlot('mktGeoMap',traces,{
    title:'GIS Network Map — Average LMP by Bus',
    geo:{
      projection:{type:'mercator'},
      showland:true,landcolor:'#f5f5dc',
      showlakes:true,lakecolor:'#a0d2db',
      showcountries:true,countrycolor:'#888',
      showsubunits:true,subunitcolor:'#aaa',
      resolution:50,
      lonaxis:{range:[lonMin-lonPad,lonMax+lonPad]},
      lataxis:{range:[latMin-latPad,latMax+latPad]}
    },
    margin:{l:0,r:0,t:40,b:0},
    legend:{x:0,y:1,bgcolor:'rgba(255,255,255,0.7)'}
  },{responsive:true});
}

document.getElementById('runPfBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  const method=document.getElementById('pfMethod').value;
  try{
    setStatus('Running '+(PF_METHOD_LABEL[method]||method)+'...');
    const body=await runPfMethod(method);
    lastPfBody=body; lastPfCompareRows=[body];
    renderPf(body);
    renderPfCompare([body]);
    setStatus('Power flow completed: '+(PF_METHOD_LABEL[method]||method));
  }catch(e){setStatus(e.message,true);}
};

document.getElementById('runPfCompareBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  const methods=['ac_newton','dc','hybrid_linearized','fdpf','adaptive','islanded','distributed_slack','three_phase'];
  setStatus('Running comparison across PF methods...');
  const rows=[];
  for(const m of methods){
    try{rows.push(await runPfMethod(m));}
    catch(e){rows.push({method:m,converged:false,iterations:0,residual:NaN,vm:[],_error:e.message});}
  }
  const selected=rows.find(r=>r.method===document.getElementById('pfMethod').value)||rows[0];
  if(selected){lastPfBody=selected;renderPf(selected);}
  lastPfCompareRows=rows;
  renderPfCompare(rows);
  const ok=rows.filter(r=>r.converged).length;
  setStatus(`PF comparison complete: ${ok}/${rows.length} methods converged.`);
};

/* Re-render on display unit change */
document.getElementById('pfDisplayUnit').onchange=()=>{
  if(lastPfBody)renderPf(lastPfBody);
  if(lastPfCompareRows.length)renderPfCompare(lastPfCompareRows);
};

/* OPF */
function renderOpfDcResults(body){
  const acBusX=(n)=>Array.from({length:n},(_,i)=>(SYS&&SYS.ac_buses&&SYS.ac_buses[i]&&SYS.ac_buses[i].index!=null)?SYS.ac_buses[i].index:`pos ${i}`);
  const dcBusX=(n)=>Array.from({length:n},(_,i)=>(SYS&&SYS.dc_buses&&SYS.dc_buses[i]&&SYS.dc_buses[i].index!=null)?SYS.dc_buses[i].index:`pos ${i}`);
  const vdc=body.vdc||[];
  if(vdc.length>0){
    Plotly.newPlot('opfDcVoltChart',[{x:dcBusX(vdc.length),y:vdc,mode:'lines+markers',line:{color:'#6a0dad',width:2},marker:{size:5,color:'#6a0dad'},name:'Vdc'}],
      {title:'DC Bus Voltage',xaxis:{title:'DC Bus'},yaxis:{title:'p.u.'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  } else { Plotly.purge('opfDcVoltChart'); }
  const pac=body.pac_mw||[], qac=body.qac_mvar||[];
  if(pac.length>0){
    const u=pUnit(), qu=qUnit();
    Plotly.newPlot('opfConverterChart',[
      {x:pac.map((_,i)=>'VSC'+(i+1)),y:pac.map(v=>pConv(v)),type:'bar',name:`P_ac (${u})`,marker:{color:'#0b6e4f'}},
      {x:qac.map((_,i)=>'VSC'+(i+1)),y:qac.map(v=>pConv(v)),type:'bar',name:`Q_ac (${qu})`,marker:{color:'#2c8c99'}}
    ],{title:'Converter Operating Points',barmode:'group',xaxis:{title:'Converter'},yaxis:{title:`${u} / ${qu}`},margin:{l:55,r:15,t:45,b:65}},{responsive:true});
  } else { Plotly.purge('opfConverterChart'); }
  // LMP chart
  const lp=body.lmp_p||[], lq=body.lmp_q||[], lmp=body.lmp||[];
  if(lp.length>0){
    const busX=acBusX(lp.length);
    const traces=[{x:busX,y:lp,type:'bar',name:'Active LMP ($/MWh)',marker:{color:'#0b6e4f'}}];
    if(lq.length>0) traces.push({x:busX,y:lq,type:'bar',name:'Reactive LMP ($/MVArh)',marker:{color:'#2c8c99'}});
    Plotly.newPlot('opfLmpChart',traces,{title:'Locational Marginal Prices',barmode:'group',xaxis:{title:'Bus'},yaxis:{title:'$/MWh'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.15}},{responsive:true});
  } else if(lmp.length>0){
    Plotly.newPlot('opfLmpChart',[{x:dcBusX(lmp.length),y:lmp,mode:'lines+markers',line:{color:'#b5651d',width:2},marker:{size:5},name:'LMP ($/MWh)'}],
      {title:'Locational Marginal Prices (DC OPF)',xaxis:{title:'Bus'},yaxis:{title:'$/MWh'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  } else { Plotly.purge('opfLmpChart'); }
}
document.getElementById('runAcOpfBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  const solver=document.getElementById('opfSolver').value;
  try{setStatus('Running AC OPF ('+solver+')...');const body=await api('/api/session/opf_ac',{solver});
  document.getElementById('opfType').textContent=body.solver_backend?body.solver_backend:('AC ('+solver+')');
  document.getElementById('opfConv').textContent=body.converged?'Yes':'No';
  document.getElementById('opfObj').textContent=fmt(body.objective,2);
  document.getElementById('opfIter').textContent=body.iterations;
  Plotly.newPlot('opfDispatchChart',[{x:body.pg_mw.map((_,i)=>i+1),y:body.pg_mw.map(v=>pConv(v)),type:'bar',marker:{color:'#0b6e4f'}}],{title:'AC OPF Dispatch',xaxis:{title:'Gen'},yaxis:{title:pUnit()},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  if(body.vm)Plotly.newPlot('opfAuxChart',[{x:Array.from({length:body.vm.length},(_,i)=>(SYS&&SYS.ac_buses&&SYS.ac_buses[i]&&SYS.ac_buses[i].index!=null)?SYS.ac_buses[i].index:`pos ${i}`),y:body.vm,mode:'lines+markers',line:{color:'#b5651d',width:2},marker:{size:5}}],{title:'AC OPF Voltage',xaxis:{title:'Bus'},yaxis:{title:'p.u.'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  renderOpfDcResults(body);
  setStatus(body.converged?('AC OPF completed ('+(body.solver_backend||solver)+').'):('AC OPF did not converge: '+(body.status||'')) ,!body.converged);}catch(e){setStatus(e.message,true);}
};

document.getElementById('runDcOpfBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{setStatus('Running DC OPF...');const body=await api('/api/session/opf_dc',{});
  document.getElementById('opfType').textContent='DC';
  document.getElementById('opfConv').textContent=body.converged?'Yes':'No';
  document.getElementById('opfObj').textContent=fmt(body.objective,2);
  document.getElementById('opfIter').textContent=body.iterations;
  Plotly.newPlot('opfDispatchChart',[{x:body.pg_mw.map((_,i)=>i+1),y:body.pg_mw.map(v=>pConv(v)),type:'bar',marker:{color:'#2c8c99'}}],{title:'DC OPF Dispatch',xaxis:{title:'Gen'},yaxis:{title:pUnit()},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  Plotly.purge('opfAuxChart');
  renderOpfDcResults(body);
  setStatus('DC OPF completed.');}catch(e){setStatus(e.message,true);}
};

document.getElementById('runParityOpfBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{setStatus('Running Parity OPF (full-space IPM)...');const body=await api('/api/session/opf_parity',{});
  document.getElementById('opfType').textContent='Parity IPM';
  document.getElementById('opfConv').textContent=body.converged?'Yes':'No';
  document.getElementById('opfObj').textContent=fmt(body.objective,2);
  document.getElementById('opfIter').textContent=body.iterations;
  Plotly.newPlot('opfDispatchChart',[{x:body.pg_mw.map((_,i)=>i+1),y:body.pg_mw.map(v=>pConv(v)),type:'bar',marker:{color:'#6a0dad'}}],{title:'Parity OPF Dispatch',xaxis:{title:'Gen'},yaxis:{title:pUnit()},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  if(body.vm)Plotly.newPlot('opfAuxChart',[{x:Array.from({length:body.vm.length},(_,i)=>(SYS&&SYS.ac_buses&&SYS.ac_buses[i]&&SYS.ac_buses[i].index!=null)?SYS.ac_buses[i].index:`pos ${i}`),y:body.vm,mode:'lines+markers',line:{color:'#b5651d',width:2},marker:{size:5}}],{title:'Parity OPF AC Voltage',xaxis:{title:'Bus'},yaxis:{title:'p.u.'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  renderOpfDcResults(body);
  setStatus('Parity OPF completed.');}catch(e){setStatus(e.message,true);}
};

/* Short Circuit */
document.getElementById('runScBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{setStatus('Running short circuit (all buses)...');
  const body=await api('/api/session/sc',{options:{fault_type:document.getElementById('scFaultType').value,c_factor:Number(document.getElementById('scCFactor').value||1.1)}});
  document.getElementById('scFault').textContent=body.fault_type;
  document.getElementById('scBuses').textContent=body.bus_results.length;
  const ik=body.bus_results.map(r=>r.ikpp_ka);
  document.getElementById('scMaxIk').textContent=fmt(Math.max(...ik),3);
  document.getElementById('scMinIk').textContent=fmt(Math.min(...ik),3);
  const busIds=body.bus_results.map(r=>'Bus '+r.bus_id);
  Plotly.newPlot('scIkChart',[
    {x:busIds,y:body.bus_results.map(r=>r.ikpp_ka),type:'bar',name:"Ik'' (initial)",marker:{color:'#0b6e4f'}},
    {x:busIds,y:body.bus_results.map(r=>r.ip_ka||0),type:'bar',name:'ip (peak)',marker:{color:'#e74c3c'}},
    {x:busIds,y:body.bus_results.map(r=>r.ib_ka||0),type:'bar',name:'Ib (breaking)',marker:{color:'#2980b9'}},
    {x:busIds,y:body.bus_results.map(r=>r.ik_ka||0),type:'bar',name:'Ik (steady)',marker:{color:'#f39c12'}},
    {x:busIds,y:body.bus_results.map(r=>r.ith_ka||0),type:'bar',name:'Ith (thermal)',marker:{color:'#8e44ad'}},
  ],{title:'Short Circuit Currents by Bus (kA)',barmode:'group',xaxis:{title:'Bus'},yaxis:{title:'kA'},margin:{l:55,r:15,t:45,b:60},legend:{orientation:'h',y:-0.2}},{responsive:true});
  Plotly.newPlot('scSkChart',[{x:body.bus_results.map(r=>r.bus_id),y:body.bus_results.map(r=>r.sk_mva),mode:'lines+markers',line:{color:'#2c8c99',width:2}}],{title:'Sk (SC Power) by Bus',xaxis:{title:'Bus'},yaxis:{title:'MVA'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
  document.getElementById('scDetailedChart').innerHTML='';
  document.getElementById('scContribChart').innerHTML='';
  document.getElementById('scVremainChart').innerHTML='';
  setStatus('Short circuit completed. '+body.bus_results.length+' buses analyzed.');}catch(e){setStatus(e.message,true);}
};

/* Detailed Short Circuit at Selected Buses */
document.getElementById('runScDetailedBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  const busInput=document.getElementById('scFaultBus').value.trim();
  if(!busInput){setStatus('Enter bus IDs (e.g. 0,3,5) for detailed SC analysis.',true);return;}
  const busIds=busInput.split(/[,\s]+/).map(Number).filter(n=>!isNaN(n));
  if(!busIds.length){setStatus('Invalid bus IDs.',true);return;}
  try{
    setStatus('Running detailed SC at bus(es): '+busIds.join(', ')+'...');
    const body=await api('/api/session/sc_detailed',{
      fault_bus_ids:busIds,
      fault_type:document.getElementById('scFaultType').value,
      c_factor:Number(document.getElementById('scCFactor').value||1.1),
    });
    document.getElementById('scFault').textContent=body.fault_type;
    document.getElementById('scBuses').textContent=body.results.length+' (detailed)';
    // Grouped bar: ikss, ip, ib, ik, ith for each fault bus
    const faultBuses=body.results.map(r=>'Fault@Bus '+r.fault_bus_id);
    const fb=body.results.map(r=>{const b=r.bus_results.find(x=>x.bus_id===r.fault_bus_id)||{};return b;});
    Plotly.newPlot('scDetailedChart',[
      {x:faultBuses,y:fb.map(b=>b.ikss_ka||0),type:'bar',name:'Ik\" (initial)',marker:{color:'#0b6e4f'}},
      {x:faultBuses,y:fb.map(b=>b.ip_ka||0),type:'bar',name:'ip (peak)',marker:{color:'#e74c3c'}},
      {x:faultBuses,y:fb.map(b=>b.ib_ka||0),type:'bar',name:'Ib (breaking)',marker:{color:'#2980b9'}},
      {x:faultBuses,y:fb.map(b=>b.ik_ka||0),type:'bar',name:'Ik (steady)',marker:{color:'#f39c12'}},
      {x:faultBuses,y:fb.map(b=>b.ith_ka||0),type:'bar',name:'Ith (thermal)',marker:{color:'#8e44ad'}},
    ],{title:'Detailed SC Currents at Fault Buses (kA)',barmode:'group',
       xaxis:{title:''},yaxis:{title:'kA'},margin:{l:55,r:15,t:45,b:60},
       legend:{orientation:'h',y:-0.2}},{responsive:true});
    document.getElementById('scMaxIk').textContent=fmt(Math.max(...fb.map(b=>b.ikss_ka||0)),3);
    document.getElementById('scMinIk').textContent=fmt(Math.min(...fb.map(b=>b.ikss_ka||0)),3);
    // Source contributions stacked bar
    Plotly.newPlot('scContribChart',[
      {x:faultBuses,y:fb.map(b=>b.ikss_gen_contrib_ka||0),type:'bar',name:'Generators',marker:{color:'#0b6e4f'}},
      {x:faultBuses,y:fb.map(b=>b.ikss_motor_contrib_ka||0),type:'bar',name:'Motors',marker:{color:'#2c8c99'}},
      {x:faultBuses,y:fb.map(b=>b.ikss_extgrid_contrib_ka||0),type:'bar',name:'External Grid',marker:{color:'#b5651d'}},
      {x:faultBuses,y:fb.map(b=>b.ikss_converter_contrib_ka||0),type:'bar',name:'Converters',marker:{color:'#8e44ad'}},
      {x:faultBuses,y:fb.map(b=>b.ikss_sgen_contrib_ka||0),type:'bar',name:'Static Gens',marker:{color:'#27ae60'}},
      {x:faultBuses,y:fb.map(b=>b.ikss_load_contrib_ka||0),type:'bar',name:'Loads',marker:{color:'#f39c12'}},
    ],{title:'SC Current Source Contributions (kA)',barmode:'stack',
       xaxis:{title:''},yaxis:{title:'kA'},margin:{l:55,r:15,t:45,b:60},
       legend:{orientation:'h',y:-0.2}},{responsive:true});
    // Remaining voltage profile for the first fault
    if(body.results.length>0){
      const r0=body.results[0];
      const allBus=r0.bus_results.map(b=>b.bus_id);
      const vr=r0.bus_results.map(b=>b.v_remaining_pu);
      Plotly.newPlot('scVremainChart',[{x:allBus,y:vr,type:'bar',
        marker:{color:vr.map(v=>v<0.8?'#e74c3c':v<0.9?'#f39c12':'#27ae60')}}],
        {title:'Remaining Voltage During Fault at Bus '+r0.fault_bus_id+' (p.u.)',
         xaxis:{title:'Bus'},yaxis:{title:'V (pu)',range:[0,1.2]},margin:{l:55,r:15,t:45,b:45},
         shapes:[{type:'line',y0:0.8,y1:0.8,x0:-0.5,x1:allBus.length-0.5,line:{color:'#e74c3c',dash:'dash',width:1}}]
        },{responsive:true});
    }
    setStatus('Detailed SC complete for bus(es): '+busIds.join(', ')+'.');
  }catch(e){setStatus(e.message,true);}
};

/* Init */
(async function(){await loadCaseLists();setStatus('Ready. Load a built-in case, MATPOWER file, or upload a JSON file.');})();

/* Time-Series PF */
document.getElementById('runTsPfBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running Time-Series PF pipeline (UC \u2192 PF)...');
    const numSteps=parseInt(document.getElementById('tsNumSteps').value)||24;
    const skipUC=document.getElementById('tsSkipUC').checked;
    const runOPF=document.getElementById('tsRunOPF').checked;
    const body=await api('/api/session/run_ts_pf',{num_steps:numSteps,skip_uc:skipUC,run_opf:runOPF});
    document.getElementById('tsSteps').textContent=body.num_steps;
    document.getElementById('tsConv').textContent=body.num_converged+'/'+body.num_steps;
    document.getElementById('tsOPFConv').textContent=body.num_opf_converged+'/'+body.num_steps;
    document.getElementById('tsCost').textContent='$'+fmt(body.total_generation_cost,0);
    const hrs=Array.from({length:body.num_steps},(_,i)=>i);
    const pal=['#0b6e4f','#2c8c99','#b5651d','#8e44ad','#2980b9','#e74c3c','#27ae60','#f39c12'];
    // Generation dispatch stacked bar
    const dtraces=(body.gen_dispatch||[]).map((d,gi)=>({x:hrs,y:d,type:'bar',name:body.gen_names[gi]||'Gen '+gi,marker:{color:pal[gi%pal.length]}}));
    (body.renewable_dispatch||[]).forEach((rd,ri)=>dtraces.push({x:hrs,y:rd,type:'bar',name:body.ren_names[ri]||'Ren '+ri,marker:{color:'#27ae60'}}));
    Plotly.newPlot('tsGenDispatchChart',dtraces,{title:'Generation Dispatch (MW)',barmode:'stack',xaxis:{title:'Hour'},yaxis:{title:'MW'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
    // Voltage mean time series (with min/max band)
    if(body.vm_mean&&body.vm_mean.length){
      const vtraces=[];
      if(body.vm_max&&body.vm_min){
        vtraces.push({x:hrs,y:body.vm_max,mode:'lines',line:{width:0,color:'rgba(11,110,79,0.3)'},name:'Vm max',showlegend:false});
        vtraces.push({x:hrs,y:body.vm_min,mode:'lines',line:{width:0,color:'rgba(11,110,79,0.3)'},fill:'tonexty',fillcolor:'rgba(11,110,79,0.1)',name:'Vm range'});
      }
      vtraces.push({x:hrs,y:body.vm_mean,mode:'lines+markers',line:{color:'#0b6e4f',width:2},name:'Mean Vm'});
      const allV=[...body.vm_mean,...(body.vm_min||[]),...(body.vm_max||[])].filter(v=>v>0);
      const vLo=Math.min(...allV),vHi=Math.max(...allV),vP=Math.max((vHi-vLo)*0.15,0.005);
      Plotly.newPlot('tsVoltChart',vtraces,{title:'Bus Voltage p.u. per Hour',xaxis:{title:'Hour'},yaxis:{title:'p.u.',range:[vLo-vP,vHi+vP]},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
    }
    // ESS SOC
    if(body.ess_soc&&body.ess_soc.length)
      Plotly.newPlot('tsESSChart',(body.ess_soc||[]).map((soc,si)=>({x:hrs,y:soc,mode:'lines+markers',name:body.ess_names[si]||'ESS '+si,line:{width:2}})),{title:'ESS State of Charge',xaxis:{title:'Hour'},yaxis:{title:'SOC',range:[0,1]},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
    setStatus('Time-Series PF complete: '+body.num_converged+'/'+body.num_steps+' steps converged. Cost: $'+fmt(body.total_generation_cost,0)+'.');
  }catch(e){setStatus(e.message,true);}
};

/* Unit Commitment */
document.getElementById('runUCBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running Unit Commitment MILP...');
    const numSteps=parseInt(document.getElementById('ucNumSteps').value)||24;
    const body=await api('/api/session/run_uc',{num_steps:numSteps});
    document.getElementById('ucFeas').textContent=body.feasible?'Yes':'No';
    document.getElementById('ucCost').textContent='$'+fmt(body.total_cost,0);
    document.getElementById('ucNGen').textContent=(body.gen_names||[]).length;
    document.getElementById('ucNESS').textContent=(body.ess_names||[]).length;
    const hrs=Array.from({length:(body.gen_dispatch[0]||[]).length},(_,i)=>i);
    const pal=['#0b6e4f','#2c8c99','#b5651d','#8e44ad','#2980b9','#e74c3c','#27ae60','#f39c12'];
    const dtraces=(body.gen_dispatch||[]).map((d,gi)=>({x:hrs,y:d,type:'bar',name:body.gen_names[gi]||'Gen '+gi,marker:{color:pal[gi%pal.length]}}));
    (body.renewable_dispatch||[]).forEach((rd,ri)=>dtraces.push({x:hrs,y:rd,type:'bar',name:body.ren_names[ri]||'Ren '+ri,marker:{color:'#27ae60'}}));
    Plotly.newPlot('ucDispatchChart',dtraces,{title:'UC Generator Dispatch (MW)',barmode:'stack',xaxis:{title:'Hour'},yaxis:{title:'MW'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
    if(body.gen_commit&&body.gen_commit.length)
      Plotly.newPlot('ucCommitChart',[{z:body.gen_commit,x:hrs,y:body.gen_names,type:'heatmap',colorscale:[[0,'#f5f5f5'],[1,'#0b6e4f']],showscale:true,colorbar:{title:'Commit',tickvals:[0,1],ticktext:['Off','On']}}],{title:'Commitment Schedule',xaxis:{title:'Hour'},margin:{l:110,r:15,t:45,b:45}},{responsive:true});
    if(body.ess_soc&&body.ess_soc.length)
      Plotly.newPlot('ucESSSOCChart',(body.ess_soc||[]).map((soc,si)=>({x:hrs,y:soc,mode:'lines+markers',name:body.ess_names[si]||'ESS '+si})),{title:'ESS State of Charge',xaxis:{title:'Hour'},yaxis:{title:'SOC',range:[0,1]},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
    setStatus('Unit Commitment complete. Feasible: '+body.feasible+'. Total cost: $'+fmt(body.total_cost,0));
  }catch(e){setStatus(e.message,true);}
};

/* Market Clearing (SCUC -> SCED -> ACPF -> LMP -> Settlement) */
document.getElementById('scenarioToggle').onchange=function(){
  document.getElementById('scenarioConfigPanel').style.display=this.checked?'block':'none';
};
document.getElementById('runMarketBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    const useScenario=document.getElementById('scenarioToggle').checked;
    setStatus('Running market clearing pipeline'+(useScenario?' with scenario generation':'')+'...');
    const numSteps=parseInt(document.getElementById('mktNumSteps').value)||24;
    const solverSel=document.getElementById('milpSolverChoice');
    const milpSolver=solverSel?solverSel.value:'auto';
    const payload={num_steps:numSteps,period_length_hr:1.0,milp_solver:milpSolver,
      n_segments:parseInt(document.getElementById('s22BidSegs').value)||3,
      price_cap:parseFloat(document.getElementById('s22PriceCap').value)||1500,
      price_floor:parseFloat(document.getElementById('s22PriceFloor').value)||0,
      mip_gap:parseFloat(document.getElementById('s26MipGap').value)||0.0001,
      time_limit_sec:parseFloat(document.getElementById('s26TimeLimit').value)||300,
      lmp_delta:parseFloat(document.getElementById('s26LmpDelta').value)||0.10,
      enable_n1:document.getElementById('s28EnableN1').checked,
      max_n1_contingencies:parseInt(document.getElementById('s28MaxN1').value)||50,
      voll:parseFloat(document.getElementById('s29VOLL').value)||10000,
      spinning_reserve_req:parseFloat(document.getElementById('s29SpinReq').value)/100.0||0.05,
    };
    if(useScenario){
      payload.scenario={
        periods_per_day:parseInt(document.getElementById('scenPeriodsPerDay').value)||96,
        period_length_min:parseFloat(document.getElementById('scenPeriodLen').value)||15,
        is_workday:document.getElementById('scenWorkday').checked,
        load_variation_std:parseFloat(document.getElementById('scenLoadNoise').value)||0.05,
        wind_forecast_error:parseFloat(document.getElementById('scenWindErr').value)||0.20,
        solar_forecast_error:parseFloat(document.getElementById('scenSolarErr').value)||0.15,
        seed:parseInt(document.getElementById('scenSeed').value)||0,
      };
    }
    // Show pipeline stages
    const stgPanel=document.getElementById('pipelineStages');
    stgPanel.style.display='flex';
    for(let i=1;i<=9;i++){const el=document.getElementById('stg'+i);el.className='stg';if(i===1)el.className='stg active';}

    const body=await api('/api/session/run_market_clearing',payload);

    // Update pipeline stages based on stages_completed
    const sc=(body.summary||{}).stages_completed||0;
    for(let i=1;i<=9;i++){
      const el=document.getElementById('stg'+i);
      el.className=i<=sc?'stg done':'stg';
    }

    const sced=body.sced||{};
    const lmp=body.lmp||{};
    const settlements=body.genco_settlements||[];
    const bids=body.bids||[];

    const scuc=body.scuc||{};
    document.getElementById('mktFeas').textContent=
      (scuc.converged?'\u2705':'\u274c')+'SCUC '+
      (sced.converged?'\u2705':'\u274c')+'SCED '+
      (lmp.converged?'\u2705':'\u274c')+'LMP';
    document.getElementById('mktFeas').style.fontSize='11px';
    document.getElementById('mktCost').textContent='$'+fmt((body.summary||{}).total_system_cost||0,0);
    document.getElementById('mktNGen').textContent=(settlements||[]).length;
    const solverEl=document.getElementById('mktSolver');
    if(solverEl) solverEl.textContent=(body.summary||{}).solver_name||'N/A';
    const valEl=document.getElementById('mktValidation');
    const val=body.validation||{};
    if(valEl) valEl.textContent=val.all_checks_passed?'\u2705 PASS':'\u274c FAIL';
    document.getElementById('mktAvgLmp').textContent=fmt(lmp.avg_lmp||0,2);

    // SCED dispatch stacked‐bar chart (generators × time steps)
    const dsp=sced.dispatch||[];
    if(dsp.length){
      const T2=dsp[0].length;
      const hrs2=Array.from({length:T2},(_,i)=>i);
      const dspTraces=dsp.map((row,gi)=>({
        x:hrs2,y:row,type:'bar',name:settlements[gi]?settlements[gi].name:('Gen '+gi)
      }));
      Plotly.newPlot('mktDispatchChart',dspTraces,{title:'SCED Generator Dispatch (MW)',barmode:'stack',xaxis:{title:'Hour'},yaxis:{title:'MW'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.25}},{responsive:true});
    } else { document.getElementById('mktDispatchChart').innerHTML=''; }

    /* --- Validation panel --- */
    const vp=document.getElementById('marketValidationPanel');
    if(vp && val.sced_power_balance){
      let html='<h3 style="margin:0 0 8px">Market Validation Results</h3>';
      // Power Balance (use SCED result)
      const pb=val.sced_power_balance||{};
      html+='<h4>Power Balance Check '+(pb.balanced?'\u2705':'\u274c')+'</h4>';
      html+='<table class="tbl"><thead><tr><th>Hour</th><th>Gen (MW)</th><th>Load (MW)</th><th>Mismatch (MW)</th></tr></thead><tbody>';
      const pmm=pb.mismatch||[];
      const pGen=pb.gen_total||[];
      const pLoad=pb.load_total||[];
      for(let t=0;t<pmm.length;t++){
        const mm=pmm[t], gn=pGen[t]||0, ld=pLoad[t]||0;
        const cls=Math.abs(mm)>1e-3?'style="color:red"':'';
        html+=`<tr><td>${t}</td><td>${fmt(gn,2)}</td><td>${fmt(ld,2)}</td><td ${cls}>${fmt(mm,4)}</td></tr>`;
      }
      html+='</tbody></table>';
      html+='<p>Max mismatch: '+fmt(pb.max_abs_mismatch||0,4)+' MW</p>';
      // Budget Balance
      const bb=val.lmp_budget||{};
      html+='<h4>LMP Budget Balance '+(bb.balanced?'\u2705':'\u274c')+'</h4>';
      html+='<table class="tbl"><thead><tr><th>Metric</th><th>Value ($)</th></tr></thead><tbody>';
      html+='<tr><td>Total Load Payment</td><td>'+fmt(bb.total_load_payment||0,2)+'</td></tr>';
      html+='<tr><td>Total Gen Payment</td><td>'+fmt(bb.total_gen_payment||0,2)+'</td></tr>';
      html+='<tr><td>Total Congestion Rent</td><td>'+fmt(bb.total_congestion_rent||0,2)+'</td></tr>';
      html+='<tr><td>Imbalance</td><td>'+fmt(bb.max_abs_imbalance||0,4)+'</td></tr>';
      html+='</tbody></table>';
      html+='<p>Balance: Load Payment \u2248 Gen Payment + Congestion Rent</p>';
      // Generator Income
      const gens=val.gen_income||[];
      const allProfitable=gens.every(g=>g.profitable);
      html+='<h4>Generator Income '+(allProfitable?'\u2705':'\u274c')+'</h4>';
      html+='<table class="tbl"><thead><tr><th>Generator</th><th>LMP Revenue ($)</th><th>Bid Cost ($)</th><th>Startup ($)</th><th>No-Load ($)</th><th>Profit ($)</th><th>Profitable</th></tr></thead><tbody>';
      gens.forEach(g=>{
        html+=`<tr><td>${g.name||''}</td><td>${fmt(g.lmp_revenue||0,2)}</td><td>${fmt(g.bid_cost||0,2)}</td><td>${fmt(g.startup_cost||0,2)}</td><td>${fmt(g.no_load_cost||0,2)}</td><td>${fmt(g.profit||0,2)}</td><td>${g.profitable?'\u2705':'\u274c'}</td></tr>`;
      });
      html+='</tbody></table>';
      // ACPF Adjustment Summary
      const adj=body.acpf_adjustment||{};
      if(adj.enabled){
        html+='<h4>AC Power Flow Post-Check</h4>';
        html+='<table class="tbl"><thead><tr><th>Metric</th><th>Value</th></tr></thead><tbody>';
        html+='<tr><td>Converged Before Adjustment</td><td>'+adj.num_converged_before+' / '+(adj.periods||[]).length+'</td></tr>';
        html+='<tr><td>Converged After Adjustment</td><td>'+adj.num_converged_after+' / '+(adj.periods||[]).length+'</td></tr>';
        html+='<tr><td>Periods Adjusted</td><td>'+adj.num_adjusted+'</td></tr>';
        html+='<tr><td>Max Adjustment</td><td>'+fmt(adj.max_adjustment_mw||0,4)+' MW</td></tr>';
        html+='</tbody></table>';
      }
      vp.innerHTML=html;
      vp.style.display='block';
    } else if(vp){
      vp.style.display='none';
    }

    // Bid curves by GenCo
    const bidTraces=[];
    (bids||[]).forEach((b,gi)=>{
      let x=[0], y=[0], cum=0;
      (b.segments||[]).forEach(seg=>{
        const q=Number(seg.quantity||0), p=Number(seg.price||0);
        x.push(cum); y.push(p);
        cum+=q;
        x.push(cum); y.push(p);
      });
      bidTraces.push({x,y,mode:'lines',name:b.generator_name||('Gen '+gi),line:{shape:'hv',width:2}});
    });
    Plotly.newPlot('marketBidChart',bidTraces,{title:'GenCo Bidding Curves (Piecewise Linear)',xaxis:{title:'MW'},yaxis:{title:'$/MWh'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.25}},{responsive:true});

    // Scenario profile charts (if scenario generation was used)
    const scen=body.scenario||{};
    if(scen.num_periods>0){
      const nT=scen.num_periods;
      const dt=scen.period_length_hr||0.25;
      const hrs=Array.from({length:nT},(_,i)=>(i*dt).toFixed(2));
      const traces=[];
      if(scen.aggregate_load_profile) traces.push({x:hrs,y:scen.aggregate_load_profile,mode:'lines',name:'Agg Load (multiplier)',line:{color:'#e74c3c',width:2}});
      // Show all wind source profiles (wind_profile_0, wind_profile_1, ...)
      for(let w=0;w<20;w++){
        const key='wind_profile_'+w;
        if(scen[key]) traces.push({x:hrs,y:scen[key],mode:'lines',name:'Wind #'+w+' CF',line:{color:'hsl('+(200+w*30)+',70%,50%)',width:1.5,dash:w>0?'dot':'solid'}});
        else break;
      }
      // Show all solar source profiles
      for(let s=0;s<20;s++){
        const key='solar_profile_'+s;
        if(scen[key]) traces.push({x:hrs,y:scen[key],mode:'lines',name:'Solar #'+s+' CF',line:{color:'hsl('+(30+s*20)+',80%,50%)',width:1.5,dash:s>0?'dot':'solid'}});
        else break;
      }
      if(traces.length) Plotly.newPlot('scenarioProfileChart',traces,{title:'Scenario-Generated Profiles ('+nT+' periods, '+dt+'h each)'+(scen.bus_load_types?' — Load types: '+scen.bus_load_types.join(', '):''),xaxis:{title:'Hour'},yaxis:{title:'Profile Value'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
      else document.getElementById('scenarioProfileChart').innerHTML='';
    } else {
      document.getElementById('scenarioProfileChart').innerHTML='';
    }

    // LMP heatmap: bus x hour
    if(lmp.nodal_lmp && lmp.nodal_lmp.length){
      const nb=lmp.nodal_lmp.length;
      const T=(lmp.nodal_lmp[0]||[]).length;
      const hrs=Array.from({length:T},(_,i)=>i);
      const buses=Array.from({length:nb},(_,i)=>'Bus '+i);
      Plotly.newPlot('marketLmpHeatmap',[{z:lmp.nodal_lmp,x:hrs,y:buses,type:'heatmap',colorscale:'YlGnBu',colorbar:{title:'$/MWh'}}],{title:'LMP Heatmap',xaxis:{title:'Hour'},margin:{l:80,r:35,t:45,b:45}},{responsive:true});

      const eLmp=lmp.energy_lmp||[];
      const cLmp=lmp.congestion_lmp||[];
      let table='<thead><tr><th>Bus</th><th>Hour</th><th>Energy LMP</th><th>Congestion LMP</th><th>Total LMP</th></tr></thead><tbody>';
      for(let b=0;b<nb;b++){
        for(let t=0;t<T;t++){
          const e=Number((eLmp[b]||[])[t]||0);
          const c=Number((cLmp[b]||[])[t]||0);
          const total=Number((lmp.nodal_lmp[b]||[])[t]||0);
          table += `<tr><td>${b}</td><td>${t}</td><td>${fmt(e,3)}</td><td>${fmt(c,3)}</td><td>${fmt(total,3)}</td></tr>`;
        }
      }
      table += '</tbody>';
      document.getElementById('marketLmpDecompTable').innerHTML=table;
    } else {
      document.getElementById('marketLmpHeatmap').innerHTML='';
      document.getElementById('marketLmpDecompTable').innerHTML='';
    }

    // GenCo profit bars (§2.9 settlement with uplift)
    Plotly.newPlot('marketGenCoProfit',[
      {x:settlements.map(s=>s.name||('Gen '+s.generator_index)),y:settlements.map(s=>s.energy_revenue||0),type:'bar',name:'Energy Rev.',marker:{color:'#2c8c99'}},
      {x:settlements.map(s=>s.name||('Gen '+s.generator_index)),y:settlements.map(s=>(s.uplift_payment||0)),type:'bar',name:'Uplift (§2.9)',marker:{color:'#27ae60'}},
      {x:settlements.map(s=>s.name||('Gen '+s.generator_index)),y:settlements.map(s=>s.total_cost||0),type:'bar',name:'Total Cost',marker:{color:'#b5651d'}},
      {x:settlements.map(s=>s.name||('Gen '+s.generator_index)),y:settlements.map(s=>s.market_profit||0),type:'scatter',mode:'markers+lines',name:'Market Profit',marker:{color:'#d32f2f',size:9},line:{color:'#d32f2f',dash:'dot'}},
      {x:settlements.map(s=>s.name||('Gen '+s.generator_index)),y:settlements.map(s=>s.profit||0),type:'scatter',mode:'markers+lines',name:'Net Profit (after uplift)',marker:{color:'#0b6e4f',size:9},line:{color:'#0b6e4f'}}
    ],{title:'§2.9 GenCo Settlement (Revenue / Uplift / Cost / Market Profit / Net Profit)',barmode:'group',xaxis:{tickangle:-35},margin:{l:55,r:15,t:45,b:70}},{responsive:true});

    // GenCo Income Derivation Table — direct per-generator income breakdown
    if(settlements.length){
      let ht='<h3 style="margin:0 0 8px">§2.9 GenCo Income Derivation</h3>';
      ht+='<table class="tbl"><thead><tr><th>Generator</th><th>Energy Revenue ($)</th><th>Reserve Revenue ($)</th>';
      ht+='<th>Market Income ($)</th><th>Energy Cost ($)</th><th>Startup ($)</th><th>No-Load ($)</th>';
      ht+='<th>Total Cost ($)</th><th>Market Profit ($)</th><th>Uplift ($)</th><th>Total Revenue ($)</th><th>Net Profit ($)</th></tr></thead><tbody>';
      let totIncome=0,totCost=0,totMktProfit=0,totUplift=0,totProfit=0;
      settlements.forEach(s=>{
        const mi=s.market_income||((s.energy_revenue||0)+(s.reserve_revenue||0));
        const tc=s.total_cost||0;
        const mp=s.market_profit||(mi-tc);
        totIncome+=mi; totCost+=tc; totMktProfit+=mp; totUplift+=(s.uplift_payment||0); totProfit+=(s.profit||0);
        ht+=`<tr><td>${s.name||('Gen '+s.generator_index)}</td><td>${fmt(s.energy_revenue||0,2)}</td><td>${fmt(s.reserve_revenue||0,2)}</td>`;
        ht+=`<td style="font-weight:bold">${fmt(mi,2)}</td><td>${fmt(s.energy_cost||0,2)}</td><td>${fmt(s.startup_cost||0,2)}</td><td>${fmt(s.no_load_cost||0,2)}</td>`;
        ht+=`<td>${fmt(tc,2)}</td><td style="color:${mp>=0?'green':'red'};font-weight:bold">${fmt(mp,2)}</td><td>${fmt(s.uplift_payment||0,2)}</td><td>${fmt(s.total_revenue||0,2)}</td>`;
        ht+=`<td style="color:${(s.profit||0)>=0?'green':'red'}">${fmt(s.profit||0,2)}</td></tr>`;
      });
      ht+=`<tr style="font-weight:bold;border-top:2px solid #333"><td>Total</td><td></td><td></td>`;
      ht+=`<td>${fmt(totIncome,2)}</td><td></td><td></td><td></td>`;
      ht+=`<td>${fmt(totCost,2)}</td><td style="color:${totMktProfit>=0?'green':'red'}">${fmt(totMktProfit,2)}</td><td>${fmt(totUplift,2)}</td><td>${fmt(totIncome+totUplift,2)}</td>`;
      ht+=`<td style="color:${totProfit>=0?'green':'red'}">${fmt(totProfit,2)}</td></tr>`;
      ht+='</tbody></table>';
      ht+='<p style="font-size:11px;color:#666">Market Profit = Market Income \u2212 Total Cost (pre-uplift, can be negative). Uplift = max(0, \u2212Market Profit). Net Profit = Market Profit + Uplift (always \u2265 0 by §2.9).</p>';
      document.getElementById('mktGenCoIncomeTable').innerHTML=ht;
    } else { document.getElementById('mktGenCoIncomeTable').innerHTML=''; }

    const pfPost=body.pf_post_check||{};
    const pfResiduals=pfPost.residuals||[];
    const pfConv=pfPost.converged||[];
    if(pfResiduals.length){
      const hrs=Array.from({length:pfResiduals.length},(_,i)=>i);
      Plotly.newPlot('marketPfResidualChart',[
        {x:hrs,y:pfResiduals,type:'scatter',mode:'lines+markers',name:'Residual',line:{color:'#b5651d',width:2}},
        {x:hrs,y:pfConv.map(v=>v?1:0),type:'bar',name:'Converged',yaxis:'y2',marker:{color:'#0b6e4f',opacity:0.35}}
      ],{title:'Nonlinear PF Post-Check by Hour',xaxis:{title:'Hour'},yaxis:{title:'Residual'},yaxis2:{title:'Converged (0/1)',overlaying:'y',side:'right',range:[-0.05,1.05]},margin:{l:60,r:60,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
    } else {
      document.getElementById('marketPfResidualChart').innerHTML='';
    }

    // --- Voltage Profile Chart (bus voltage magnitudes across converged periods) ---
    const adjPeriods=(body.acpf_adjustment||{}).periods||[];
    const convPeriods=adjPeriods.filter(p=>p.converged_after&&p.vm_pu&&p.vm_pu.length>0);
    if(convPeriods.length>0){
      // Heatmap: buses × periods
      const nb2=convPeriods[0].vm_pu.length;
      const vmMatrix=convPeriods.map(p=>p.vm_pu);
      const periods=convPeriods.map(p=>p.period);
      const buses=Array.from({length:nb2},(_,i)=>'Bus '+i);
      Plotly.newPlot('mktVoltageProfileChart',[{
        z:vmMatrix,x:buses,y:periods,type:'heatmap',
        colorscale:[[0,'#d32f2f'],[0.25,'#ff9800'],[0.5,'#4caf50'],[0.75,'#ff9800'],[1,'#d32f2f']],
        zmin:0.90,zmax:1.10,
        colorbar:{title:'V (pu)',len:0.7}
      }],{title:'Bus Voltage Profile (pu) — AC PF',
          xaxis:{title:'Bus'},yaxis:{title:'Period'},
          margin:{l:55,r:80,t:45,b:55}},{responsive:true});

      // Reactive power chart: bus Q injection for last converged period
      const lastP=convPeriods[convPeriods.length-1];
      if(lastP.bus_q_mvar&&lastP.bus_q_mvar.length>0){
        const busLabels=Array.from({length:lastP.bus_q_mvar.length},(_,i)=>'Bus '+i);
        Plotly.newPlot('mktReactivePowerChart',[{
          x:busLabels,y:lastP.bus_q_mvar,type:'bar',
          marker:{color:lastP.bus_q_mvar.map(v=>v>=0?'#2c8c99':'#b5651d')}
        }],{title:'Bus Reactive Power Injection (Mvar) — Last Converged Period',
            xaxis:{title:'Bus',tickangle:-45},yaxis:{title:'Mvar'},
            margin:{l:55,r:15,t:45,b:55}},{responsive:true});
      } else { document.getElementById('mktReactivePowerChart').innerHTML=''; }
    } else {
      document.getElementById('mktVoltageProfileChart').innerHTML='';
      document.getElementById('mktReactivePowerChart').innerHTML='';
    }

    // --- Voltage & Thermal Violation Summary ---
    const violPanel=document.getElementById('mktViolationPanel');
    const adjInfo=body.acpf_adjustment||{};
    const totalVV=adjInfo.total_voltage_violations||0;
    const totalTV=adjInfo.total_thermal_violations||0;
    if(totalVV>0||totalTV>0){
      let vhtml='<h3 style="margin:0 0 8px">AC PF Violation Summary</h3>';
      vhtml+='<p>Total voltage violations: <b>'+totalVV+'</b>, Total thermal violations: <b>'+totalTV+'</b></p>';
      // Collect violations across periods
      let vRows=[];let tRows=[];
      (adjInfo.periods||[]).forEach(p=>{
        (p.voltage_violations||[]).forEach(v=>{
          vRows.push({period:p.period,bus:v.bus,vm:v.vm_pu,limit:v.limit,type:v.is_low?'Under-voltage':'Over-voltage'});
        });
        (p.thermal_violations||[]).forEach(v=>{
          tRows.push({period:p.period,branch:v.branch,flow:v.flow_mva,rating:v.rating_mva});
        });
      });
      if(vRows.length){
        vhtml+='<h4>Voltage Violations \u26a0\ufe0f</h4>';
        vhtml+='<table class="tbl"><thead><tr><th>Period</th><th>Bus</th><th>V (pu)</th><th>Limit (pu)</th><th>Type</th></tr></thead><tbody>';
        vRows.slice(0,50).forEach(r=>{vhtml+=`<tr><td>${r.period}</td><td>${r.bus}</td><td style="color:red">${fmt(r.vm,4)}</td><td>${fmt(r.limit,3)}</td><td>${r.type}</td></tr>`;});
        if(vRows.length>50)vhtml+='<tr><td colspan="5">... '+(vRows.length-50)+' more</td></tr>';
        vhtml+='</tbody></table>';
      }
      if(tRows.length){
        vhtml+='<h4>Thermal Violations \u26a0\ufe0f</h4>';
        vhtml+='<table class="tbl"><thead><tr><th>Period</th><th>Branch</th><th>Flow (MVA)</th><th>Rating (MVA)</th><th>Overload %</th></tr></thead><tbody>';
        tRows.slice(0,50).forEach(r=>{const pct=r.rating>0?100*(r.flow/r.rating-1):0;vhtml+=`<tr><td>${r.period}</td><td>${r.branch}</td><td style="color:red">${fmt(r.flow,2)}</td><td>${fmt(r.rating,2)}</td><td>${fmt(pct,1)}%</td></tr>`;});
        if(tRows.length>50)vhtml+='<tr><td colspan="5">... '+(tRows.length-50)+' more</td></tr>';
        vhtml+='</tbody></table>';
      }
      violPanel.innerHTML=vhtml;violPanel.style.display='block';
    } else { violPanel.innerHTML='';violPanel.style.display='none'; }

    // --- §2.8 Security Check Panel ---
    const secPanel=document.getElementById('mktSecurityPanel');
    const secData=body.security_check||{};
    if(secPanel && secData.enabled){
      let shtml='<h3 style="margin:0 0 8px">§2.8 Security Check Results '+(secData.all_secure?'\u2705':'\u274c')+'</h3>';
      shtml+='<table class="tbl"><thead><tr><th>Check</th><th>Result</th><th>Detail</th></tr></thead><tbody>';
      shtml+='<tr><td>Power Balance Adequacy</td><td>'+(secData.power_balance_adequate?'\u2705 Pass':'\u274c Fail')+'</td>';
      shtml+='<td>Reserve margin: '+fmt(secData.reserve_margin_mw||0,1)+' MW (required: '+fmt(secData.reserve_requirement_mw||0,1)+' MW)</td></tr>';
      shtml+='<tr><td>Base Case AC PF</td><td>'+(secData.base_case_secure?'\u2705 Secure':'\u274c Insecure')+'</td>';
      shtml+='<td>Voltage violations: '+(secData.base_voltage_violations||0)+', Thermal violations: '+(secData.base_thermal_violations||0)+'</td></tr>';
      shtml+='<tr><td>N-1 Contingency</td><td>'+((secData.n1_contingencies_failed||0)===0?'\u2705 Secure':'\u274c '+(secData.n1_contingencies_failed)+' failures')+'</td>';
      shtml+='<td>Checked: '+(secData.n1_contingencies_checked||0)+', Failed: '+(secData.n1_contingencies_failed||0)+'</td></tr>';
      shtml+='<tr><td>Curtailment Fairness</td><td>'+(secData.curtailment_fair?'\u2705 Fair':'\u274c Unfair')+'</td>';
      shtml+='<td>Fairness index: '+fmt(secData.curtailment_fairness_index||0,3)+' (max ratio: '+fmt(secData.max_curtailment_ratio||0,3)+', min: '+fmt(secData.min_curtailment_ratio||0,3)+')</td></tr>';
      shtml+='</tbody></table>';
      // N-1 detail table for failed contingencies
      const n1r=(secData.n1_results||[]).filter(r=>!r.secure);
      if(n1r.length){
        shtml+='<h4>N-1 Contingency Failures</h4>';
        shtml+='<table class="tbl"><thead><tr><th>Branch</th><th>Name</th><th>Converged</th><th>Max Loading %</th><th>Max V Dev (pu)</th><th>V Viol</th><th>Thermal Viol</th></tr></thead><tbody>';
        n1r.slice(0,30).forEach(r=>{
          shtml+=`<tr><td>${r.branch}</td><td>${r.branch_name||''}</td><td>${r.converged?'Yes':'No'}</td>`;
          shtml+=`<td style="color:${r.max_loading_pct>100?'red':'inherit'}">${fmt(r.max_loading_pct,1)}</td>`;
          shtml+=`<td>${fmt(r.max_v_deviation_pu,4)}</td><td>${r.v_violations}</td><td>${r.thermal_violations}</td></tr>`;
        });
        if(n1r.length>30)shtml+='<tr><td colspan="7">... '+(n1r.length-30)+' more</td></tr>';
        shtml+='</tbody></table>';
      }
      secPanel.innerHTML=shtml;secPanel.style.display='block';
    } else if(secPanel){ secPanel.innerHTML='';secPanel.style.display='none'; }

    // --- PTDF Heatmap ---
    const ptdfV=body.ptdf_verification||{};
    if(ptdfV.computed&&ptdfV.ptdf_matrix&&ptdfV.ptdf_matrix.length){
      const bLabels=Array.from({length:ptdfV.num_buses},(_,i)=>'Bus '+i);
      const lLabels=ptdfV.branch_labels||Array.from({length:ptdfV.num_branches},(_,i)=>'Br '+i);
      Plotly.newPlot('mktPtdfHeatmap',[{
        z:ptdfV.ptdf_matrix,x:bLabels,y:lLabels,type:'heatmap',
        colorscale:'RdBu',zmid:0,
        colorbar:{title:'PTDF',len:0.7}
      }],{title:'Power Transfer Distribution Factors (PTDF)',
          xaxis:{title:'Bus'},yaxis:{title:'Branch'},
          margin:{l:90,r:60,t:45,b:55}},{responsive:true});

      // PTDF vs ACPF verification bar chart
      if(ptdfV.max_error_mw&&ptdfV.max_error_mw.length){
        Plotly.newPlot('mktPtdfVerifyChart',[{
          x:lLabels,y:ptdfV.max_error_mw,type:'bar',
          marker:{color:ptdfV.max_error_mw.map(e=>e>5?'#d32f2f':e>1?'#ff9800':'#4caf50')}
        }],{title:'PTDF vs AC PF Verification — Max Error per Branch (MW)<br>Overall Max: '+fmt(ptdfV.overall_max_error_mw,3)+' MW ('+fmt(ptdfV.overall_max_error_pct,1)+'% of rating), Mean: '+fmt(ptdfV.overall_mean_error_mw,3)+' MW',
            xaxis:{title:'Branch',tickangle:-45},yaxis:{title:'Max Error (MW)'},
            margin:{l:55,r:15,t:65,b:70}},{responsive:true});
      } else { document.getElementById('mktPtdfVerifyChart').innerHTML=''; }
    } else {
      document.getElementById('mktPtdfHeatmap').innerHTML='';
      document.getElementById('mktPtdfVerifyChart').innerHTML='';
    }

    // --- GIS Network Map with LMP colouring ---
    renderMktGeoMap(body, lmp, sced);

    setStatus('Market clearing complete. Avg LMP: $'+fmt(lmp.avg_lmp||0,2)+', renewable penetration: '+fmt((body.summary||{}).renewable_penetration||0,1)+'%, PF max residual: '+fmt(pfPost.max_residual||0,4)+
      (totalVV>0?', \u26a0 '+totalVV+' voltage violations':'')+
      (totalTV>0?', \u26a0 '+totalTV+' thermal violations':'')+
      (secData.enabled?(secData.all_secure?', \u2705 Security check passed':' \u274c Security check failed'):'')+'.');
  }catch(e){setStatus(e.message,true);}
};

/* Carbon Flow Analysis */
document.getElementById('runCarbonBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running Carbon Flow Analysis (proportional tracing + matrix method)...');
    const body=await api('/api/session/run_carbon',{});
    const ts=body.tracing_summary||{};
    document.getElementById('carbonGenEmit').textContent=fmt(ts.total_generation_emissions_tco2,2);
    document.getElementById('carbonLoadEmit').textContent=fmt(ts.total_load_emissions_tco2,2);
    document.getElementById('carbonLossEmit').textContent=fmt(ts.total_loss_emissions_tco2,3);
    document.getElementById('carbonMatrix').textContent=body.matrix_solved?'Yes':'No';
    // Carbon intensity by bus
    if(body.bus_carbon&&body.bus_carbon.length){
      const ci=body.bus_carbon.map(b=>b.carbon_intensity_tco2_mwh);
      const mx=Math.max(...ci)||1;
      Plotly.newPlot('carbonBusChart',[{x:body.bus_carbon.map(b=>'Bus '+b.bus_index),y:ci,type:'bar',
        marker:{color:ci,colorscale:'RdYlGn',reversescale:true,cmin:0,cmax:mx,showscale:true,colorbar:{title:'tCO\u2082/MWh',len:0.7}},
        text:ci.map(v=>v.toFixed(4)),textposition:'auto'}],
        {title:'AC Bus Carbon Intensity (tCO\u2082/MWh)',xaxis:{title:'Bus',tickangle:-45},yaxis:{title:'tCO\u2082/MWh'},margin:{l:55,r:80,t:45,b:60}},{responsive:true});
    }
    // DC bus carbon intensity
    if(body.dc_bus_carbon&&body.dc_bus_carbon.length){
      const dci=body.dc_bus_carbon.map(b=>b.carbon_intensity_tco2_mwh);
      const dmx=Math.max(...dci)||1;
      Plotly.newPlot('carbonDcBusChart',[{x:body.dc_bus_carbon.map(b=>'DC Bus '+b.bus_index),y:dci,type:'bar',
        marker:{color:dci,colorscale:'RdYlGn',reversescale:true,cmin:0,cmax:dmx,showscale:true,colorbar:{title:'tCO\u2082/MWh',len:0.7}},
        text:dci.map(v=>v.toFixed(4)),textposition:'auto'}],
        {title:'DC Bus Carbon Intensity (tCO\u2082/MWh)',xaxis:{title:'DC Bus',tickangle:-45},yaxis:{title:'tCO\u2082/MWh'},margin:{l:55,r:80,t:45,b:60}},{responsive:true});
    } else { document.getElementById('carbonDcBusChart').innerHTML=''; }
    // Per-load total emissions bar
    if(body.load_carbon&&body.load_carbon.length){
      Plotly.newPlot('carbonLoadChart',[{x:body.load_carbon.map(l=>'Load '+l.load_index+' B'+l.bus),
        y:body.load_carbon.map(l=>l.total_emissions_tco2),type:'bar',
        marker:{color:'#c34d4d'}}],
        {title:'Load Carbon Emissions (tCO\u2082)',xaxis:{title:'',tickangle:-35},yaxis:{title:'tCO\u2082'},margin:{l:55,r:15,t:45,b:70}},{responsive:true});
    }
    // Branch carbon emissions
    if(body.branch_carbon&&body.branch_carbon.length){
      const bc=body.branch_carbon.filter(b=>b.total_emissions_tco2>1e-6);
      if(bc.length) Plotly.newPlot('carbonBranchChart',[
        {x:bc.map(b=>b.from_bus+'\u2192'+b.to_bus),y:bc.map(b=>b.loss_mw),type:'bar',name:'Loss MW',marker:{color:'#b5651d'}},
        {x:bc.map(b=>b.from_bus+'\u2192'+b.to_bus),y:bc.map(b=>b.total_emissions_tco2),type:'bar',name:'Emissions tCO\u2082',marker:{color:'#8e44ad'},yaxis:'y2'},
      ],{title:'Branch Loss & Carbon Emissions',barmode:'group',xaxis:{title:'Branch',tickangle:-45},
         yaxis:{title:'Loss MW',side:'left'},yaxis2:{title:'tCO\u2082',overlaying:'y',side:'right'},
         margin:{l:55,r:55,t:45,b:70},legend:{orientation:'h',y:-0.3}},{responsive:true});
      else document.getElementById('carbonBranchChart').innerHTML='';
    } else { document.getElementById('carbonBranchChart').innerHTML=''; }
    // Sankey: generators -> loads
    if(body.sankey_sources&&body.sankey_sources.length){
      Plotly.newPlot('carbonSankeyChart',[{type:'sankey',orientation:'h',
        node:{pad:12,thickness:18,line:{color:'#ccc',width:0.5},label:body.sankey_labels,
          color:body.sankey_labels.map((_,i)=>i<(SYS.generators||[]).length?'#0b6e4f':'#2c8c99')},
        link:{source:body.sankey_sources,target:body.sankey_targets,value:body.sankey_values,
          color:'rgba(44,140,153,0.3)'}}],
        {title:'Carbon Flow Sankey: Generators \u2192 Loads (MW)',margin:{l:20,r:20,t:45,b:20}},{responsive:true});
    } else {
      document.getElementById('carbonSankeyChart').innerHTML='<div style="padding:20px;color:var(--muted);">No generator-load supply data for Sankey (run AC power flow first or check case data).</div>';
    }
    setStatus('Carbon analysis complete. Gen: '+fmt(ts.total_generation_emissions_tco2,2)+' tCO\u2082, Load: '+fmt(ts.total_load_emissions_tco2,2)+' tCO\u2082. Balance err: '+fmt(ts.balance_error_pct,4)+'%.');
  }catch(e){setStatus(e.message,true);}
};

/* Reliability Assessment (Monte Carlo) */
let relLastNSQ=null, relLastSEQ=null, relLastFD=null, relLastResilience=null;

function getRelOpts(){
  return {
    load_scale_factor: parseFloat(document.getElementById('relLoadScale').value)||1.0,
    cov_threshold: parseFloat(document.getElementById('relCovThresh').value)||0.05,
    seed: parseInt(document.getElementById('relSeed').value)||0,
    apply_ieee24_data: document.getElementById('relApplyIEEE24').checked,
    compute_tail_risk: document.getElementById('relTailRisk').checked,
    var_confidence: parseFloat(document.getElementById('relVarConf').value)||0.95,
  };
}

function updateRelKPIs(body, method){
  document.getElementById('relMethod').textContent=method;
  document.getElementById('relEENS').textContent=fmt(body.eens_mwh_yr,1);
  document.getElementById('relLOLE').textContent=fmt(body.lole_hr_yr,2);
  document.getElementById('relLOLF').textContent=body.lolf_occ_yr!==undefined?fmt(body.lolf_occ_yr,2):'N/A';
  document.getElementById('relPLC').textContent=fmt(body.plc*100,2);
  document.getElementById('relCoV').textContent=fmt(body.final_cov,4);
  document.getElementById('relIters').textContent=body.iterations_used;
  
  // Update tail risk KPIs if available
  const tailKPIs=document.getElementById('relTailKPIs');
  if(body.tail_risk&&(body.tail_risk.eens_var>0||body.tail_risk.eens_cvar>0)){
    tailKPIs.style.display='flex';
    document.getElementById('relEENSVar').textContent=fmt(body.tail_risk.eens_var,1);
    document.getElementById('relEENSCVar').textContent=fmt(body.tail_risk.eens_cvar,1);
    document.getElementById('relLOLEVar').textContent=fmt(body.tail_risk.lole_var,2);
    document.getElementById('relLOLECVar').textContent=fmt(body.tail_risk.lole_cvar,2);
  } else {
    tailKPIs.style.display='none';
  }
}

function updateFDKPIs(body){
  document.getElementById('relMethod').textContent='F&D';
  document.getElementById('relEENS').textContent='N/A';  // F&D doesn't compute EENS
  document.getElementById('relLOLE').textContent=fmt(body.lole_fd,2);
  document.getElementById('relLOLF').textContent=fmt(body.lolf_fd,2);
  document.getElementById('relPLC').textContent=fmt(body.lolp*100,4);
  document.getElementById('relCoV').textContent='N/A';
  document.getElementById('relIters').textContent='Analytical';
  document.getElementById('relTailKPIs').style.display='none';
}

function plotRelConvergence(body, title){
  if(body.eens_history&&body.eens_history.length){
    const iters=body.eens_history.map((_,i)=>i+1);
    Plotly.newPlot('relConvergenceChart',[
      {x:iters,y:body.eens_history,mode:'lines',name:'EENS (MWh/yr)',line:{color:'#0b6e4f'}},
      {x:iters,y:body.cov_history,mode:'lines',name:'CoV',yaxis:'y2',line:{color:'#c34d4d',dash:'dot'}}
    ],{title:title+' Convergence',xaxis:{title:'Iteration'},yaxis:{title:'EENS (MWh/yr)',side:'left'},
       yaxis2:{title:'CoV',overlaying:'y',side:'right',rangemode:'tozero'},
       margin:{l:60,r:60,t:45,b:50},legend:{orientation:'h',y:-0.15}},{responsive:true});
  }
}

function plotNodalEENS(body){
  if(body.nodal_eens_mwh_yr&&body.nodal_eens_mwh_yr.length){
    const nz=body.nodal_eens_mwh_yr.map((v,i)=>({bus:i+1,eens:v})).filter(x=>x.eens>0.01);
    if(nz.length){
      Plotly.newPlot('relNodalEENSChart',[{x:nz.map(x=>'Bus '+x.bus),y:nz.map(x=>x.eens),type:'bar',
        marker:{color:'#c34d4d'}}],{title:'Nodal EENS (MWh/yr)',xaxis:{title:'Bus',tickangle:-45},
        yaxis:{title:'MWh/yr'},margin:{l:60,r:15,t:45,b:60}},{responsive:true});
    } else {
      document.getElementById('relNodalEENSChart').innerHTML='<div style="padding:40px;color:var(--muted);text-align:center;">No nodal EENS (all buses have zero curtailment)</div>';
    }
  }
}

function plotCriticalComponents(body){
  if(body.critical_components&&body.critical_components.length){
    const top10=body.critical_components.slice(0,10);
    Plotly.newPlot('relCriticalChart',[{
      x:top10.map(c=>c.importance*100),
      y:top10.map(c=>(c.is_generator?'Gen ':'Br ')+c.index),
      type:'bar',orientation:'h',
      marker:{color:top10.map(c=>c.is_generator?'#0b6e4f':'#2c8c99')}
    }],{title:'Top Critical Components (% importance)',xaxis:{title:'Importance (%)'},
       yaxis:{autorange:'reversed'},margin:{l:80,r:15,t:45,b:50}},{responsive:true});
  } else {
    document.getElementById('relCriticalChart').innerHTML='<div style="padding:40px;color:var(--muted);text-align:center;">No critical component data (no loss events detected)</div>';
  }
}

document.getElementById('runNsqBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    const opts=getRelOpts();
    opts.max_iterations=parseInt(document.getElementById('relNsqMaxIter').value)||5000;
    setStatus('Running Non-Sequential Monte Carlo ('+opts.max_iterations+' samples)...');
    document.getElementById('relStatus').textContent='Running NSQ MC...';
    const body=await api('/api/session/run_reliability_nsq',opts);
    relLastNSQ=body;
    updateRelKPIs(body,'NSQ');
    plotRelConvergence(body,'NSQ MC');
    plotNodalEENS(body);
    plotCriticalComponents(body);
    document.getElementById('relCrossValChart').innerHTML='';
    const conv=body.converged?'Converged':'Max iterations';
    document.getElementById('relStatus').textContent='NSQ MC complete ('+conv+'). EENS='+fmt(body.eens_mwh_yr,1)+' MWh/yr, LOLE='+fmt(body.lole_hr_yr,2)+' hr/yr';
    setStatus('NSQ MC complete. EENS='+fmt(body.eens_mwh_yr,1)+' MWh/yr, LOLE='+fmt(body.lole_hr_yr,2)+' hr/yr, PLC='+fmt(body.plc*100,2)+'%');
  }catch(e){setStatus(e.message,true);document.getElementById('relStatus').textContent='Error: '+e.message;}
};

document.getElementById('runSeqBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    const opts=getRelOpts();
    opts.max_years=parseInt(document.getElementById('relSeqMaxYears').value)||500;
    opts.hours_per_year=parseInt(document.getElementById('relSeqHours').value)||8736;
    setStatus('Running Sequential Monte Carlo ('+opts.max_years+' years, '+opts.hours_per_year+' hr/yr)...');
    document.getElementById('relStatus').textContent='Running SEQ MC (this may take a while)...';
    const body=await api('/api/session/run_reliability_seq',opts);
    relLastSEQ=body;
    updateRelKPIs(body,'SEQ');
    plotRelConvergence(body,'SEQ MC');
    plotNodalEENS(body);
    plotCriticalComponents(body);
    document.getElementById('relCrossValChart').innerHTML='';
    const conv=body.converged?'Converged':'Max years';
    document.getElementById('relStatus').textContent='SEQ MC complete ('+conv+'). EENS='+fmt(body.eens_mwh_yr,1)+' MWh/yr, LOLF='+fmt(body.lolf_occ_yr,2)+' occ/yr';
    setStatus('SEQ MC complete. EENS='+fmt(body.eens_mwh_yr,1)+' MWh/yr, LOLE='+fmt(body.lole_hr_yr,2)+' hr/yr, LOLF='+fmt(body.lolf_occ_yr,2)+' occ/yr');
  }catch(e){setStatus(e.message,true);document.getElementById('relStatus').textContent='Error: '+e.message;}
};

document.getElementById('runCrossValBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    const opts=getRelOpts();
    opts.max_iterations=parseInt(document.getElementById('relNsqMaxIter').value)||5000;
    opts.max_years=parseInt(document.getElementById('relSeqMaxYears').value)||500;
    opts.hours_per_year=parseInt(document.getElementById('relSeqHours').value)||8736;
    
    // Run NSQ
    setStatus('Cross-validation: Running NSQ MC...');
    document.getElementById('relStatus').textContent='Running NSQ MC for cross-validation...';
    const nsq=await api('/api/session/run_reliability_nsq',opts);
    relLastNSQ=nsq;
    
    // Run SEQ
    setStatus('Cross-validation: Running SEQ MC...');
    document.getElementById('relStatus').textContent='Running SEQ MC for cross-validation...';
    const seq=await api('/api/session/run_reliability_seq',opts);
    relLastSEQ=seq;
    
    // Display comparison
    updateRelKPIs(nsq,'NSQ vs SEQ');
    plotRelConvergence(nsq,'NSQ MC');
    plotNodalEENS(nsq);
    plotCriticalComponents(nsq);
    
    // Cross-validation comparison chart
    const indices=['EENS (MWh/yr)','LOLE (hr/yr)','PLC (%)'];
    const nsqVals=[nsq.eens_mwh_yr, nsq.lole_hr_yr, nsq.plc*100];
    const seqVals=[seq.eens_mwh_yr, seq.lole_hr_yr, seq.plc*100];
    Plotly.newPlot('relCrossValChart',[
      {x:indices,y:nsqVals,type:'bar',name:'NSQ MC',marker:{color:'#0b6e4f'}},
      {x:indices,y:seqVals,type:'bar',name:'SEQ MC',marker:{color:'#8e44ad'}}
    ],{title:'Cross-Validation: NSQ vs SEQ',barmode:'group',yaxis:{title:'Value'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.15}},{responsive:true});
    
    // Calculate differences
    const eensDiff=Math.abs(nsq.eens_mwh_yr-seq.eens_mwh_yr);
    const eensPct=seq.eens_mwh_yr>0?(eensDiff/seq.eens_mwh_yr*100):0;
    const loleDiff=Math.abs(nsq.lole_hr_yr-seq.lole_hr_yr);
    const lolePct=seq.lole_hr_yr>0?(loleDiff/seq.lole_hr_yr*100):0;
    
    document.getElementById('relStatus').textContent='Cross-validation complete. EENS diff: '+fmt(eensDiff,1)+' MWh/yr ('+fmt(eensPct,1)+'%), LOLE diff: '+fmt(loleDiff,2)+' hr/yr ('+fmt(lolePct,1)+'%)';
    setStatus('Cross-validation complete. NSQ: EENS='+fmt(nsq.eens_mwh_yr,0)+', SEQ: EENS='+fmt(seq.eens_mwh_yr,0)+'. Diff: '+fmt(eensPct,1)+'%');
  }catch(e){setStatus(e.message,true);document.getElementById('relStatus').textContent='Error: '+e.message;}
};

document.getElementById('runFDBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    const opts=getRelOpts();
    setStatus('Running Frequency & Duration analysis (analytical)...');
    document.getElementById('relStatus').textContent='Running F&D analysis...';
    const body=await api('/api/session/run_reliability_fd',opts);
    relLastFD=body;
    updateFDKPIs(body);
    
    // Plot COPT
    if(body.capacity_outage_levels&&body.cumulative_probability){
      Plotly.newPlot('relConvergenceChart',[
        {x:body.capacity_outage_levels,y:body.cumulative_probability,mode:'lines',name:'Cum. Probability',line:{color:'#0b6e4f'}},
        {x:body.capacity_outage_levels,y:body.cumulative_frequency,mode:'lines',name:'Cum. Frequency',yaxis:'y2',line:{color:'#c34d4d',dash:'dot'}}
      ],{title:'Capacity Outage Probability Table (COPT)',xaxis:{title:'Outage Level (MW)'},
         yaxis:{title:'Cumulative Probability',side:'left'},yaxis2:{title:'Cumulative Frequency (occ/hr)',overlaying:'y',side:'right'},
         margin:{l:60,r:60,t:45,b:50},legend:{orientation:'h',y:-0.15}},{responsive:true});
    }
    
    document.getElementById('relNodalEENSChart').innerHTML='<div style="padding:40px;color:var(--muted);text-align:center;">F&D is a system-level method (no nodal breakdown)</div>';
    document.getElementById('relCriticalChart').innerHTML='<div style="padding:40px;color:var(--muted);text-align:center;">F&D is an analytical method (no component breakdown)</div>';
    document.getElementById('relCrossValChart').innerHTML='';
    
    document.getElementById('relStatus').textContent='F&D complete. LOLP='+fmt(body.lolp,6)+', LOLE='+fmt(body.lole_fd,2)+' hr/yr, LOLF='+fmt(body.lolf_fd,2)+' occ/yr, LOLD='+fmt(body.lold,2)+' hr/occ';
    setStatus('F&D complete. LOLP='+fmt(body.lolp*100,4)+'%, LOLE='+fmt(body.lole_fd,2)+' hr/yr, LOLF='+fmt(body.lolf_fd,2)+' occ/yr');
  }catch(e){setStatus(e.message,true);document.getElementById('relStatus').textContent='Error: '+e.message;}
};

/* FMEA (Failure Modes & Effects Analysis) */
let relLastFMEA=null;
document.getElementById('runFmeaBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    const opts={
      load_scale_factor: parseFloat(document.getElementById('relLoadScale').value)||1.0,
      apply_comprehensive_data: document.getElementById('relFmeaApplyComprehensive').checked,
      verbose: false,
    };
    setStatus('Running FMEA (N-1 contingency enumeration)...');
    document.getElementById('relStatus').textContent='Running FMEA analysis...';
    const body=await api('/api/session/run_reliability_fmea',opts);
    relLastFMEA=body;
    
    // Update FMEA KPIs
    document.getElementById('fmeaResultsSection').style.display='block';
    document.getElementById('fmeaEENS').textContent=fmt(body.eens_mwh_yr,1);
    document.getElementById('fmeaLOLE').textContent=fmt(body.lole_hr_yr,2);
    document.getElementById('fmeaLOLF').textContent=fmt(body.lolf_occ_yr,2);
    document.getElementById('fmeaSAIFI').textContent=fmt(body.saifi,4);
    document.getElementById('fmeaSAIDI').textContent=fmt(body.saidi,4);
    document.getElementById('fmeaASAI').textContent=fmt(body.asai,6);
    document.getElementById('fmeaContingencies').textContent=body.n_contingencies+' ('+body.n_with_loss+' with loss)';
    
    // Plot top contingencies (bar chart)
    if(body.contingencies&&body.contingencies.length>0){
      const top=body.contingencies.slice(0,15);
      Plotly.newPlot('fmeaContChart',[{
        x:top.map(c=>c.display_name||c.component_name),
        y:top.map(c=>c.eens_contribution),
        type:'bar',
        marker:{color:top.map(c=>c.shed_mw>0?'#c0392b':'#95a5a6')},
        text:top.map(c=>c.display_type||c.component_type),
        hovertemplate:'%{x}<br>Type: %{text}<br>EENS: %{y:.2f} MWh/yr<extra></extra>'
      }],{title:'Top Contingencies by EENS Contribution',xaxis:{tickangle:-45},
          yaxis:{title:'EENS (MWh/yr)'},margin:{l:60,r:15,t:45,b:120}},{responsive:true});
    }
    
    // Plot EENS by component type (pie chart)
    const eensByType=body.eens_by_display_type||body.eens_by_type;
    if(eensByType){
      const types=Object.keys(eensByType).filter(k=>eensByType[k]>0);
      const vals=types.map(k=>eensByType[k]);
      Plotly.newPlot('fmeaTypeChart',[{
        labels:types,values:vals,type:'pie',
        marker:{colors:['#c0392b','#e67e22','#2ecc71','#3498db','#9b59b6','#1abc9c','#34495e','#e74c3c','#f39c12']},
        textinfo:'label+percent',hovertemplate:'%{label}<br>EENS: %{value:.2f} MWh/yr<extra></extra>'
      }],{title:'EENS Breakdown by Component Type',margin:{l:20,r:20,t:45,b:20}},{responsive:true});
    }
    
    document.getElementById('relStatus').textContent='FMEA complete. EENS='+fmt(body.eens_mwh_yr,1)+' MWh/yr, SAIFI='+fmt(body.saifi,4)+', SAIDI='+fmt(body.saidi,4)+', ASAI='+fmt(body.asai,6);
    setStatus('FMEA: '+body.n_contingencies+' contingencies, EENS='+fmt(body.eens_mwh_yr,1)+' MWh/yr, SAIFI='+fmt(body.saifi,4));
  }catch(e){setStatus(e.message,true);document.getElementById('relStatus').textContent='Error: '+e.message;}
};

/* Distribution Resilience (MESS) */
function addFaultRow(bid,shr,rhr,lbl){
  const tbody=document.getElementById('resFaultTableBody');
  const tr=document.createElement('tr');
  tr.innerHTML='<td><input type="number" min="1" value="'+(bid||'')+'" style="width:70px;"/></td>'
    +'<td><input type="number" min="0" step="0.5" value="'+(shr||0)+'" style="width:70px;"/></td>'
    +'<td><input type="number" min="1" step="0.5" value="'+(rhr||6)+'" style="width:70px;"/></td>'
    +'<td><input type="text" value="'+(lbl||'')+'" style="width:120px;"/></td>'
    +'<td><button onclick="this.closest(\'tr\').remove()" style="cursor:pointer;border:none;background:none;color:#ef4444;font-size:1.1em;">&#10005;</button></td>';
  tbody.appendChild(tr);
}
function collectManualFaults(){
  const rows=document.querySelectorAll('#resFaultTableBody tr');
  const arr=[];
  rows.forEach(r=>{
    const cells=r.querySelectorAll('input');
    const bid=parseInt(cells[0].value,10);
    if(!Number.isFinite(bid)||bid<=0) return;
    arr.push({branch_id:bid,start_hr:parseFloat(cells[1].value)||0,repair_hr:parseFloat(cells[2].value)||6,label:cells[3].value||''});
  });
  return arr;
}
async function runResilienceSingle(overrides={}){
  const faultIds=(document.getElementById('resFaultBranches').value||'')
    .split(',').map(s=>parseInt(s.trim(),10)).filter(v=>Number.isFinite(v));
  const manualFaults=collectManualFaults();
  const params=Object.assign({
    horizon_hours: parseInt(document.getElementById('resHorizonHours').value)||8,
    time_step_hr: 1.0,
    load_scale_factor: parseFloat(document.getElementById('resLoadScale').value)||1.0,
    repair_time_hr: parseFloat(document.getElementById('resRepairHours').value)||6.0,
    default_fault_count: parseInt(document.getElementById('resFaultCount').value)||1,
    fault_branch_ids: faultIds,
    manual_faults: manualFaults,
    auto_fault_stagger_hr: parseFloat(document.getElementById('resFaultStagger').value)||0,
    auto_fault_start_hr: parseFloat(document.getElementById('resFaultStartHr').value)||0,
    mess_travel_speed_kmph: parseFloat(document.getElementById('resMessSpeed').value)||40.0,
    allow_reconfiguration: document.getElementById('resAllowReconfig').checked,
    allow_mess_dispatch: document.getElementById('resAllowMess').checked,
    apply_demo_data: document.getElementById('resApplyDemoData').checked,
    run_power_flow: document.getElementById('resRunPF').checked,
  },overrides);
  return await api('/api/session/run_distribution_resilience',params);
}

function plotResilienceResults(body,baseBody){
  document.getElementById('resilienceResultsSection').style.display='block';
  document.getElementById('resilienceRI').textContent=fmt(body.resilience_index,4);
  document.getElementById('resilienceDemand').textContent=fmt(body.total_demand_mwh,2);
  document.getElementById('resilienceShed').textContent=fmt(body.total_shed_mwh,2);
  document.getElementById('resilienceFinalRestore').textContent=fmt(body.final_restoration_ratio*100,2);
  document.getElementById('resilienceAvgRestore').textContent=fmt(body.avg_restoration_ratio*100,2);
  document.getElementById('resiliencePeakShed').textContent=fmt(body.peak_shed_mw,2);
  document.getElementById('resilienceMessEnergy').textContent=fmt(body.mess_energy_delivered_mwh,2);
  document.getElementById('resilienceMessTravel').textContent=fmt(body.mess_travel_distance_km,1);
  document.getElementById('resilienceSwitches').textContent=body.total_switch_actions;
  document.getElementById('resilienceRepairs').textContent=body.total_repaired_faults;

  // Populate fault sequence table from response (so user can see & edit for re-run).
  if(body.fault_sequence&&body.fault_sequence.length>0){
    const tbody=document.getElementById('resFaultTableBody');
    tbody.innerHTML='';
    body.fault_sequence.forEach(f=>{addFaultRow(f.branch_index,f.start_hr,f.repair_hr,f.name);});
  }

  // Comparison improvement banner
  const compDiv=document.getElementById('resilienceCompareKPIs');
  if(baseBody&&baseBody.feasible){
    compDiv.style.display='block';
    const dRI=body.resilience_index-baseBody.resilience_index;
    const dShed=baseBody.total_shed_mwh-body.total_shed_mwh;
    const dServed=body.total_served_mwh-baseBody.total_served_mwh;
    document.getElementById('resCompRI').textContent='RI: '+(dRI>=0?'+':'')+fmt(dRI,4);
    document.getElementById('resCompShed').textContent='Shed: '+(dShed>=0?'\u2212':'+' )+fmt(Math.abs(dShed),2)+' MWh';
    document.getElementById('resCompServed').textContent='Served: '+(dServed>=0?'+':'')+fmt(dServed,2)+' MWh';
  }else{compDiv.style.display='none';}

  // Chart 1: Restoration Timeline with optional baseline overlay
  const restoTraces=[
    {x:body.hours,y:body.demand_mw,mode:'lines+markers',name:'Demand',line:{color:'#475569',width:2}},
    {x:body.hours,y:body.served_mw,mode:'lines+markers',name:'Served (MESS)',line:{color:'#0b6e4f',width:2},fill:'tozeroy',fillcolor:'rgba(11,110,79,0.08)'},
    {x:body.hours,y:body.shed_mw,mode:'lines+markers',name:'Shed (MESS)',line:{color:'#c34d4d',width:2}},
    {x:body.hours,y:body.restoration_ratio.map(v=>v*100),mode:'lines',name:'Restoration %',yaxis:'y2',line:{color:'#0f766e',dash:'dot',width:2}}
  ];
  if(baseBody&&baseBody.feasible){
    restoTraces.push({x:baseBody.hours,y:baseBody.served_mw,mode:'lines',name:'Served (baseline)',line:{color:'#94a3b8',dash:'dash',width:1.5}});
    restoTraces.push({x:baseBody.hours,y:baseBody.shed_mw,mode:'lines',name:'Shed (baseline)',line:{color:'#f87171',dash:'dash',width:1.5}});
    restoTraces.push({x:baseBody.hours,y:baseBody.restoration_ratio.map(v=>v*100),mode:'lines',name:'Restoration % (base)',yaxis:'y2',line:{color:'#a7f3d0',dash:'dashdot',width:1}});
  }
  Plotly.newPlot('resilienceRestorationChart',restoTraces,{title:'Restoration Timeline'+(baseBody?' (MESS vs Baseline)':''),
    xaxis:{title:'Hour'},yaxis:{title:'MW'},yaxis2:{title:'Restoration (%)',overlaying:'y',side:'right',range:[0,105]},
    margin:{l:60,r:60,t:45,b:55},legend:{orientation:'h',y:-0.22}},{responsive:true});

  // Chart 2: MESS Dispatch + Faults/Switching
  const messTraces=[];
  (body.mess_traces||[]).forEach((tr)=>{
    messTraces.push({x:body.hours,y:tr.dispatch_mw,mode:'lines+markers',name:tr.name||('MESS '+tr.storage_index),line:{width:2}});
  });
  messTraces.push({x:body.hours,y:body.active_faults,type:'bar',name:'Active Faults',yaxis:'y2',marker:{color:'#f59e0b',opacity:0.35}});
  messTraces.push({x:body.hours,y:body.switch_actions,mode:'lines+markers',name:'Switch Actions',yaxis:'y2',line:{color:'#7c3aed',dash:'dot'}});
  Plotly.newPlot('resilienceMessChart',messTraces,{title:'MESS Dispatch & Restoration Actions',xaxis:{title:'Hour'},
    yaxis:{title:'MESS Dispatch (MW)'},yaxis2:{title:'Faults / Switching',overlaying:'y',side:'right',rangemode:'tozero'},
    margin:{l:60,r:60,t:45,b:55},legend:{orientation:'h',y:-0.25}},{responsive:true});

  // Chart 3: MESS Energy / SOC timeline
  const energyTraces=[];
  (body.mess_traces||[]).forEach((tr)=>{
    energyTraces.push({x:body.hours,y:tr.energy_mwh,mode:'lines+markers',name:(tr.name||'MESS')+' Energy',line:{width:2}});
  });
  if(energyTraces.length>0){
    Plotly.newPlot('resilienceEnergyChart',energyTraces,{title:'MESS Stored Energy Over Time',
      xaxis:{title:'Hour'},yaxis:{title:'Energy (MWh)',rangemode:'tozero'},
      margin:{l:60,r:30,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
  }else{
    document.getElementById('resilienceEnergyChart').innerHTML='<div style="padding:60px;color:var(--muted);text-align:center;">No MESS units active</div>';
  }

  // Chart 4: Island count + fault/repair status over time
  const islandCounts=(body.island_counts||body.hours.map(()=>0));
  const repairCounts=(body.repaired_faults_arr||body.hours.map(()=>0));
  Plotly.newPlot('resilienceIslandChart',[
    {x:body.hours,y:body.active_faults,type:'bar',name:'Active Faults',marker:{color:'#fbbf24',opacity:0.5}},
    {x:body.hours,y:repairCounts,type:'bar',name:'Repaired',marker:{color:'#34d399',opacity:0.5}},
    {x:body.hours,y:islandCounts,mode:'lines+markers',name:'Islands',yaxis:'y2',line:{color:'#6366f1',width:2}}
  ],{title:'Network Topology Over Time',xaxis:{title:'Hour'},
    yaxis:{title:'Fault Count',rangemode:'tozero'},yaxis2:{title:'Island Count',overlaying:'y',side:'right',rangemode:'tozero'},
    barmode:'stack',margin:{l:60,r:60,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});

  // Chart 5: Load & RES multiplier profiles
  const loadMults=body.load_multipliers||[];
  const resMults=body.res_multipliers||[];
  const resMwArr=body.res_mw||[];
  if(loadMults.length>0 || resMults.length>0){
    const profTraces=[];
    if(loadMults.length>0) profTraces.push({x:body.hours,y:loadMults,mode:'lines+markers',name:'Load Multiplier',line:{color:'#0369a1',width:2}});
    if(resMults.length>0)  profTraces.push({x:body.hours,y:resMults,mode:'lines+markers',name:'RES Multiplier',line:{color:'#f59e0b',width:2}});
    if(resMwArr.length>0)  profTraces.push({x:body.hours,y:resMwArr,mode:'lines+markers',name:'RES Output (MW)',yaxis:'y2',line:{color:'#22c55e',width:2,dash:'dot'}});
    const profLayout={title:'Load & Renewable Profiles',xaxis:{title:'Hour'},
      yaxis:{title:'Multiplier',rangemode:'tozero'},margin:{l:60,r:60,t:45,b:55},legend:{orientation:'h',y:-0.22}};
    if(resMwArr.length>0) Object.assign(profLayout,{yaxis2:{title:'RES MW',overlaying:'y',side:'right',rangemode:'tozero'}});
    Plotly.newPlot('resilienceProfileChart',profTraces,profLayout,{responsive:true});
  }else{
    document.getElementById('resilienceProfileChart').innerHTML='<div style="padding:60px;color:var(--muted);text-align:center;">No profile data</div>';
  }

  // Chart 6: MESS spatial trajectory (bus location over time)
  const trajTraces=[];
  (body.mess_traces||[]).forEach((tr,i)=>{
    const buses=tr.target_bus||tr.bus||[];
    if(buses.length>0){
      trajTraces.push({x:body.hours.slice(0,buses.length),y:buses,mode:'lines+markers',name:(tr.name||'MESS '+tr.storage_index)+' Bus',
        line:{width:2,shape:'hv'},marker:{size:7}});
    }
  });
  if(trajTraces.length>0){
    Plotly.newPlot('resilienceTrajectoryChart',trajTraces,{title:'MESS Spatial Trajectory (Bus Location)',
      xaxis:{title:'Hour'},yaxis:{title:'Bus ID',dtick:1},
      margin:{l:60,r:30,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
  }else{
    document.getElementById('resilienceTrajectoryChart').innerHTML='<div style="padding:60px;color:var(--muted);text-align:center;">No MESS trajectory data</div>';
  }

  // Chart 7: Load Shed by Priority Tier (stacked bar)
  const sCrit=body.shed_critical||[];
  const sHigh=body.shed_high||[];
  const sMed=body.shed_medium||[];
  const sLow=body.shed_low||[];
  if(sCrit.length>0){
    Plotly.newPlot('resiliencePriorityChart',[
      {x:body.hours,y:sCrit,type:'bar',name:'Critical',marker:{color:'#dc2626'}},
      {x:body.hours,y:sHigh,type:'bar',name:'High',marker:{color:'#f59e0b'}},
      {x:body.hours,y:sMed,type:'bar',name:'Medium',marker:{color:'#3b82f6'}},
      {x:body.hours,y:sLow,type:'bar',name:'Low',marker:{color:'#94a3b8'}},
    ],{title:'Load Shed by Priority Tier',barmode:'stack',xaxis:{title:'Hour'},
      yaxis:{title:'Shed (MW)',rangemode:'tozero'},
      margin:{l:60,r:30,t:45,b:55},legend:{orientation:'h',y:-0.22}},{responsive:true});
  }else{
    document.getElementById('resiliencePriorityChart').innerHTML='<div style="padding:60px;color:var(--muted);text-align:center;">No shed data</div>';
  }

  // Chart 8: MESS Dispatch vs SOC (dual-axis)
  const messSOCTraces=[];
  (body.mess_traces||[]).forEach((tr)=>{
    const nm=tr.name||('MESS '+tr.storage_index);
    messSOCTraces.push({x:body.hours,y:tr.dispatch_mw,mode:'lines+markers',name:nm+' Dispatch',line:{width:2}});
    messSOCTraces.push({x:body.hours,y:(tr.soc||[]).map(v=>v*100),mode:'lines',name:nm+' SOC%',yaxis:'y2',line:{width:2,dash:'dot'}});
  });
  if(messSOCTraces.length>0){
    Plotly.newPlot('resilienceMessSOCChart',messSOCTraces,{title:'MESS Dispatch vs SOC',
      xaxis:{title:'Hour'},yaxis:{title:'Dispatch (MW)',rangemode:'tozero'},
      yaxis2:{title:'SOC (%)',overlaying:'y',side:'right',range:[0,105]},
      margin:{l:60,r:60,t:45,b:55},legend:{orientation:'h',y:-0.25}},{responsive:true});
  }else{
    document.getElementById('resilienceMessSOCChart').innerHTML='<div style="padding:60px;color:var(--muted);text-align:center;">No MESS units active</div>';
  }

  // Chart 9: Bus Voltage Profile from Power Flow
  const bvTraces=body.bus_voltage_traces||[];
  const vChartDiv=document.getElementById('resilienceVoltageChart');
  if(bvTraces.length>0){
    vChartDiv.style.display='';
    const vTraces=[];
    bvTraces.forEach(bv=>{
      vTraces.push({x:body.hours.slice(0,bv.vm_pu.length),y:bv.vm_pu,mode:'lines',name:'Bus '+bv.bus_index,line:{width:1.5}});
    });
    // Upper and lower voltage limits
    vTraces.push({x:body.hours,y:body.hours.map(()=>1.05),mode:'lines',name:'V_max',line:{color:'#c34d4d',dash:'dash',width:1},showlegend:true});
    vTraces.push({x:body.hours,y:body.hours.map(()=>0.95),mode:'lines',name:'V_min',line:{color:'#c34d4d',dash:'dash',width:1},showlegend:false});
    Plotly.newPlot('resilienceVoltageChart',vTraces,{title:'Bus Voltage Magnitude (from Power Flow)',
      xaxis:{title:'Hour'},yaxis:{title:'Vm (p.u.)',range:[0.85,1.10]},
      margin:{l:60,r:30,t:45,b:55},legend:{orientation:'h',y:-0.25}},{responsive:true});
  }else{
    vChartDiv.style.display='none';
  }

  // Chart 10: Branch Power Flow (MW) from Power Flow
  const bfTraces=body.branch_flow_traces||[];
  const fChartDiv=document.getElementById('resilienceBranchFlowChart');
  if(bfTraces.length>0){
    fChartDiv.style.display='';
    const flowTraces=[];
    bfTraces.forEach(bf=>{
      flowTraces.push({x:body.hours.slice(0,bf.pf_mw.length),y:bf.pf_mw,mode:'lines',
        name:'Br '+bf.branch_index+' ('+bf.from_bus+'\u2192'+bf.to_bus+')',line:{width:1.5}});
    });
    Plotly.newPlot('resilienceBranchFlowChart',flowTraces,{title:'Branch Active Power Flow (from Power Flow)',
      xaxis:{title:'Hour'},yaxis:{title:'Pf (MW)'},
      margin:{l:60,r:30,t:45,b:55},legend:{orientation:'h',y:-0.25}},{responsive:true});
  }else{
    fChartDiv.style.display='none';
  }
}

document.getElementById('runResilienceBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running resilience assessment...');
    document.getElementById('resStatus').textContent='Running resilience assessment...';
    const useMess=document.getElementById('resAllowMess').checked;
    let baseBody=null;
    // Always run a baseline (no MESS) first for comparison if MESS is enabled
    if(useMess){
      setStatus('Running baseline (no MESS) for comparison...');
      baseBody=await runResilienceSingle({allow_mess_dispatch:false});
    }
    setStatus('Running resilience with current settings...');
    const body=await runResilienceSingle();
    relLastResilience=body;
    plotResilienceResults(body,baseBody);
    document.getElementById('resStatus').textContent='Resilience complete. RI='+fmt(body.resilience_index,4)+', shed='+fmt(body.total_shed_mwh,2)+' MWh, MESS='+fmt(body.mess_energy_delivered_mwh,2)+' MWh';
    setStatus('Resilience complete. RI='+fmt(body.resilience_index,4)+', shed='+fmt(body.total_shed_mwh,2)+' MWh, final='+fmt(body.final_restoration_ratio*100,1)+'%');
  }catch(e){setStatus(e.message,true);document.getElementById('resStatus').textContent='Error: '+e.message;}
};

/* Reactive Power Optimization (RPO) */
async function runRPO(relaxOnly){
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running RPO '+(relaxOnly?'(continuous relaxation)':'(MINLP B&B + IPM)')+'...');
    const body=await api('/api/session/run_rpo',{
      objective:document.getElementById('rpoObjective').value,
      mip_gap:parseFloat(document.getElementById('rpoMipGap').value)||0.01,
      time_limit_s:parseInt(document.getElementById('rpoTimeLimit').value)||120,
      vdev_weight:parseFloat(document.getElementById('rpoVdevWeight').value)||1.0,
      relax_only:!!relaxOnly,
    });
    /* KPI cards */
    document.getElementById('rpoConv').textContent=body.converged?'Yes':'No';
    document.getElementById('rpoObj').textContent=fmt(body.objective,4);
    document.getElementById('rpoGap').textContent=fmt(body.gap*100,2);
    document.getElementById('rpoNodes').textContent=body.nodes_explored;
    document.getElementById('rpoLPSolves').textContent=body.nlp_solves;
    document.getElementById('rpoTime').textContent=fmt(body.runtime_sec,2);
    document.getElementById('rpoLossBefore').textContent=fmt(body.total_loss_before,2);
    document.getElementById('rpoLossAfter').textContent=fmt(body.total_loss_after,2);
    document.getElementById('rpoVdevBefore').textContent=fmt(body.max_vdev_before,4);
    document.getElementById('rpoVdevAfter').textContent=fmt(body.max_vdev_after,4);
    document.getElementById('rpoStatus').textContent=body.status||'-';
    const busIdx=(body.vm_before||[]).map((_,i)=>i+1);
    /* 1. Bus Voltage Profile */
    Plotly.newPlot('rpoVoltChart',[
      {x:busIdx,y:body.vm_before||[],mode:'lines+markers',name:'Vm Before',line:{color:'#b5651d',width:2,dash:'dot'},marker:{size:4}},
      {x:busIdx,y:body.vm_after||[],mode:'lines+markers',name:'Vm After',line:{color:'#0b6e4f',width:2},marker:{size:4}},
      {x:busIdx,y:busIdx.map(()=>body.v_min||0.95),mode:'lines',name:'Vmin',line:{color:'#c34d4d',width:1,dash:'dash'},showlegend:true},
      {x:busIdx,y:busIdx.map(()=>body.v_max||1.05),mode:'lines',name:'Vmax',line:{color:'#c34d4d',width:1,dash:'dash'},showlegend:false},
    ],{title:'Bus Voltage Magnitude (p.u.)',xaxis:{title:'Bus'},yaxis:{title:'p.u.'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
    /* 2. Generator Reactive Power */
    const genNames=body.gen_names||(body.qg_before||[]).map((_,i)=>'Gen '+i);
    Plotly.newPlot('rpoQgChart',[
      {x:genNames,y:body.qg_before||[],type:'bar',name:'Qg Before',marker:{color:'#b5651d'}},
      {x:genNames,y:body.qg_after||[],type:'bar',name:'Qg After',marker:{color:'#0b6e4f'}},
    ],{title:'Generator Reactive Power Qg (MVAr)',barmode:'group',xaxis:{title:''},yaxis:{title:'MVAr'},margin:{l:55,r:15,t:45,b:50},legend:{orientation:'h',y:-0.25}},{responsive:true});
    /* 3. Generator Active Power */
    Plotly.newPlot('rpoPgChart',[
      {x:genNames,y:body.pg_before||[],type:'bar',name:'Pg Before',marker:{color:'#8e6f3e'}},
      {x:genNames,y:body.pg_after||[],type:'bar',name:'Pg After',marker:{color:'#2c8c99'}},
    ],{title:'Generator Active Power Pg (MW)',barmode:'group',xaxis:{title:''},yaxis:{title:'MW'},margin:{l:55,r:15,t:45,b:50},legend:{orientation:'h',y:-0.25}},{responsive:true});
    /* 4. Bus Voltage Angle */
    Plotly.newPlot('rpoVaChart',[
      {x:busIdx,y:(body.va_before||[]).map(a=>a*180/Math.PI),mode:'lines+markers',name:'Va Before (deg)',line:{color:'#b5651d',width:2,dash:'dot'},marker:{size:3}},
      {x:busIdx,y:(body.va_after||[]).map(a=>a*180/Math.PI),mode:'lines+markers',name:'Va After (deg)',line:{color:'#0b6e4f',width:2},marker:{size:3}},
    ],{title:'Bus Voltage Angle (degrees)',xaxis:{title:'Bus'},yaxis:{title:'deg'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
    /* 5. Tap Changer Positions */
    if(body.tap_names&&body.tap_names.length){
      Plotly.newPlot('rpoTapChart',[
        {x:body.tap_names,y:body.tap_before||[],type:'bar',name:'Ratio Before',marker:{color:'#8e44ad'}},
        {x:body.tap_names,y:body.tap_after||[],type:'bar',name:'Ratio After',marker:{color:'#2c8c99'}},
      ],{title:'OLTC Tap Ratio (p.u.)',barmode:'group',xaxis:{title:''},yaxis:{title:'Tap Ratio'},margin:{l:55,r:15,t:45,b:50},legend:{orientation:'h',y:-0.25}},{responsive:true});
    } else {
      document.getElementById('rpoTapChart').innerHTML='<div style="padding:20px;color:var(--muted);">No OLTC transformers detected.</div>';
    }
    /* 6. Switchable Shunt Steps */
    if(body.shunt_names&&body.shunt_names.length){
      Plotly.newPlot('rpoShuntChart',[
        {x:body.shunt_names,y:body.shunt_mvar_before||body.shunt_before||[],type:'bar',name:'Before (MVAr)',marker:{color:'#e67e22'}},
        {x:body.shunt_names,y:body.shunt_mvar_after||body.shunt_after||[],type:'bar',name:'After (MVAr)',marker:{color:'#27ae60'}},
      ],{title:'Switchable Shunt Output (MVAr)',barmode:'group',xaxis:{title:''},yaxis:{title:'MVAr'},margin:{l:55,r:15,t:45,b:50},legend:{orientation:'h',y:-0.25}},{responsive:true});
    } else {
      document.getElementById('rpoShuntChart').innerHTML='<div style="padding:20px;color:var(--muted);">No switchable shunts detected.</div>';
    }
    /* 7. Voltage Deviation |V-1| per Bus */
    const vdev_before=(body.vm_before||[]).map(v=>Math.abs(v-1.0));
    const vdev_after=(body.vm_after||[]).map(v=>Math.abs(v-1.0));
    Plotly.newPlot('rpoDetailsChart',[
      {x:busIdx,y:vdev_before,type:'bar',name:'|V-1| Before',marker:{color:'rgba(181,101,29,0.6)'}},
      {x:busIdx,y:vdev_after,type:'bar',name:'|V-1| After',marker:{color:'rgba(11,110,79,0.6)'}},
    ],{title:'Voltage Deviation |V - 1.0| per Bus',barmode:'group',xaxis:{title:'Bus'},yaxis:{title:'p.u.'},margin:{l:55,r:15,t:45,b:45},legend:{orientation:'h',y:-0.2}},{responsive:true});
    /* Debug summary table */
    const L=s=>s;  // line helper
    const lines=[];
    lines.push('╔══════════════════════════════════════════════════════════════════╗');
    lines.push('║  RPO (MINLP) Debug Summary                                     ║');
    lines.push('╠══════════════════════════════════════════════════════════════════╣');
    lines.push('  Solver:     Branch & Bound + Parity IPM');
    lines.push('  Status:     '+(body.status||'N/A'));
    lines.push('  Objective:  '+body.objective_type+' = '+fmt(body.objective,6));
    lines.push('  Converged:  '+body.converged+'  |  Gap: '+fmt(body.gap*100,4)+'%');
    lines.push('  B&B Nodes:  '+body.nodes_explored+'  |  NLP Solves: '+body.nlp_solves+'  |  Time: '+fmt(body.runtime_sec,2)+'s');
    lines.push('');
    lines.push('── Loss ─────────────────────────────────────────────────');
    const dLoss=body.total_loss_after-body.total_loss_before;
    const pLoss=body.total_loss_before?((dLoss/body.total_loss_before)*100):0;
    lines.push('  Before: '+fmt(body.total_loss_before,3)+' MW  |  After: '+fmt(body.total_loss_after,3)+' MW  |  Δ: '+(dLoss>=0?'+':'')+fmt(dLoss,3)+' MW ('+fmt(pLoss,2)+'%)');
    lines.push('');
    lines.push('── Voltage Deviation ────────────────────────────────────');
    const svb=vdev_before.reduce((a,b)=>a+b*b,0), sva=vdev_after.reduce((a,b)=>a+b*b,0);
    lines.push('  Max |V-1|  Before: '+fmt(Math.max(...vdev_before),4)+' p.u.  |  After: '+fmt(Math.max(...vdev_after),4)+' p.u.');
    lines.push('  Σ(V-1)²   Before: '+fmt(svb,6)+'  |  After: '+fmt(sva,6)+'  |  Δ: '+fmt(sva-svb,6)+' ('+(svb?fmt((sva-svb)/svb*100,2):'N/A')+'%)');
    lines.push('');
    if(body.tap_names&&body.tap_names.length){
      lines.push('── OLTC Tap Changers ────────────────────────────────────');
      lines.push('  Name              Pos Before  Pos After  Ratio Before  Ratio After');
      body.tap_names.forEach((n,i)=>{
        const pb=(body.tap_pos_before||[])[i], pa=(body.tap_pos_after||[])[i];
        lines.push('  '+n.padEnd(18)+(pb!=null?String(pb).padStart(6):' ??   ')+'       '+(pa!=null?String(pa).padStart(6):' ??   ')+'       '+fmt(body.tap_before[i],4).padStart(10)+'    '+fmt(body.tap_after[i],4).padStart(10));
      });
      lines.push('');
    }
    if(body.shunt_names&&body.shunt_names.length){
      lines.push('── Switchable Shunts ────────────────────────────────────');
      lines.push('  Name                Step Before  Step After   MVAr Before   MVAr After');
      body.shunt_names.forEach((n,i)=>{
        const mb=(body.shunt_mvar_before||[])[i], ma=(body.shunt_mvar_after||[])[i];
        lines.push('  '+n.padEnd(20)+String(body.shunt_before[i]).padStart(8)+'       '+String(body.shunt_after[i]).padStart(8)+'       '+(mb!=null?fmt(mb,2).padStart(10):' ??       ')+'    '+(ma!=null?fmt(ma,2).padStart(10):' ??       '));
      });
      lines.push('');
    }
    lines.push('── Generator Dispatch ───────────────────────────────────');
    lines.push('  Name                 Pg Before    Pg After   ΔPg (MW)   Qg Before    Qg After   ΔQg (MVAr)');
    (body.gen_names||[]).forEach((n,i)=>{
      const pgb=(body.pg_before||[])[i]||0, pga=(body.pg_after||[])[i]||0;
      const qgb=(body.qg_before||[])[i]||0, qga=(body.qg_after||[])[i]||0;
      lines.push('  '+n.padEnd(20)+fmt(pgb,2).padStart(9)+'    '+fmt(pga,2).padStart(9)+'   '+(pga-pgb>=0?'+':'')+fmt(pga-pgb,2).padStart(8)+'   '+fmt(qgb,2).padStart(9)+'    '+fmt(qga,2).padStart(9)+'   '+(qga-qgb>=0?'+':'')+fmt(qga-qgb,2).padStart(8));
    });
    lines.push('');
    lines.push('── Bus Voltages ─────────────────────────────────────────');
    lines.push('  Bus   Vm Before   Vm After   ΔVm        Va Before°   Va After°   ΔVa°');
    busIdx.forEach((b,i)=>{
      const vmb=(body.vm_before||[])[i]||0, vma=(body.vm_after||[])[i]||0;
      const vab=((body.va_before||[])[i]||0)*180/Math.PI, vaa=((body.va_after||[])[i]||0)*180/Math.PI;
      lines.push('  '+String(b).padStart(3)+'    '+fmt(vmb,4).padStart(8)+'   '+fmt(vma,4).padStart(8)+'  '+(vma-vmb>=0?'+':'')+fmt(vma-vmb,4).padStart(7)+'     '+fmt(vab,2).padStart(8)+'    '+fmt(vaa,2).padStart(8)+'  '+(vaa-vab>=0?'+':'')+fmt(vaa-vab,2).padStart(7));
    });
    lines.push('╚══════════════════════════════════════════════════════════════════╝');
    document.getElementById('rpoResultTable').textContent=lines.join('\n');
    setStatus('RPO complete. Obj='+fmt(body.objective,4)+' ('+body.objective_type+'), Status: '+(body.status||'OK')+', Nodes='+body.nodes_explored+', Time='+fmt(body.runtime_sec,2)+'s');
  }catch(e){setStatus(e.message,true);}
}
document.getElementById('runRPOBtn').onclick=()=>runRPO(false);
document.getElementById('runRPORelaxBtn').onclick=()=>runRPO(true);

/* Network Reconfiguration */
document.getElementById('runReconfigBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running Network Reconfiguration (TS-TR MILP)...');
    const numSteps=parseInt(document.getElementById('rcNumSteps').value)||4;
    const body=await api('/api/session/run_reconfig',{
      num_steps:numSteps,
      v_min:parseFloat(document.getElementById('rcVmin').value)||0.95,
      v_max:parseFloat(document.getElementById('rcVmax').value)||1.05,
      mip_gap:parseFloat(document.getElementById('rcMipGap').value)||0.01,
      max_time_s:parseInt(document.getElementById('rcMaxTime').value)||60,
    });
    document.getElementById('rcFeas').textContent=body.feasible?'Yes':'No';
    document.getElementById('rcObj').textContent=fmt(body.total_objective,2);
    const totalCurt=(body.curt_per_step||[]).reduce((a,b)=>a+b,0);
    const totalSwOn=(body.sw_on_per_step||[]).reduce((a,b)=>a+b,0);
    const totalSwOff=(body.sw_off_per_step||[]).reduce((a,b)=>a+b,0);
    document.getElementById('rcACLoss').textContent=fmt(totalCurt,2);
    document.getElementById('rcShed').textContent=body.num_branches;
    document.getElementById('rcSwActions').textContent=totalSwOn+totalSwOff;
    const T=body.num_steps||numSteps;
    const hrs=Array.from({length:T},(_,i)=>i);
    Plotly.newPlot('rcLossChart',[
      {x:hrs,y:body.curt_per_step||[],mode:'lines+markers',name:'Curtailment MW',line:{color:'#b5651d',width:2}},
      {x:hrs,y:body.sw_on_per_step||[],mode:'lines+markers',name:'Switch-On',line:{color:'#27ae60',width:2,dash:'dot'}},
      {x:hrs,y:body.sw_off_per_step||[],mode:'lines+markers',name:'Switch-Off',line:{color:'#e74c3c',width:2,dash:'dash'}},
    ],{title:'Curtailment & Switching per Step',xaxis:{title:'Step'},yaxis:{title:'Count / MW'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
    Plotly.newPlot('rcSwitchChart',[
      {x:hrs,y:body.sw_on_per_step||[],type:'bar',name:'Close (AC)',marker:{color:'#27ae60'}},
      {x:hrs,y:body.sw_off_per_step||[],type:'bar',name:'Open (AC)',marker:{color:'#e74c3c'}},
    ],{title:'AC Switching Actions per Step',barmode:'group',xaxis:{title:'Step'},yaxis:{title:'Count'},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
    const essSoc=body.ess_soc_by_step||[];
    const nESS=body.num_ess||0;
    const essSocData=nESS>0
      ? Array.from({length:nESS},(_,si)=>({x:hrs,y:essSoc.map(row=>(row&&row[si])||0),mode:'lines+markers',name:'ESS '+(si+1)}))
      : [];
    if(essSocData.length)
      Plotly.newPlot('rcESSChart',essSocData,{title:'ESS SOC during Reconfiguration',xaxis:{title:'Step'},yaxis:{title:'SOC',range:[0,1]},margin:{l:55,r:15,t:45,b:45}},{responsive:true});
    else
      document.getElementById('rcESSChart').innerHTML='<div style="padding:20px;color:var(--muted);">No storage units in this case.</div>';
    setStatus('Reconfiguration done. Feasible: '+body.feasible+'. Obj: '+fmt(body.total_objective,2)+'. Switch actions: '+(totalSwOn+totalSwOff)+'. Solver: '+(body.solver_name||'native')+'.');
  }catch(e){setStatus(e.message,true);}
};

/* Annual Production Simulation */
document.getElementById('runAnnualBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running Annual Production Simulation (this may take a while)...');
    const body=await api('/api/session/run_annual_sim',{
      resolution:document.getElementById('annualResolution').value,
      block_type:document.getElementById('annualBlockType').value,
      snapshot_interval:parseInt(document.getElementById('annualSnapshotInterval').value)||24,
      run_opf:document.getElementById('annualRunOPF').checked,
      cyclic_soc:document.getElementById('annualCyclicSOC').checked,
      skip_replay:document.getElementById('annualSkipReplay').checked,
    });
    document.getElementById('annFeas').textContent=body.feasible?'Yes':'No';
    document.getElementById('annCost').textContent='$'+fmt(body.total_cost,0);
    document.getElementById('annGenMWh').textContent=fmt(body.total_gen_mwh,0);
    document.getElementById('annRenMWh').textContent=fmt(body.total_renewable_mwh,0);
    document.getElementById('annCurtMWh').textContent=fmt(body.total_curtailment_mwh,0);
    document.getElementById('annENSMWh').textContent=fmt(body.total_ens_mwh,0);
    document.getElementById('annLossMWh').textContent=fmt(body.total_loss_mwh,0);
    document.getElementById('annPFConv').textContent=body.num_pf_converged+'/'+body.num_steps;
    const pal=['#0b6e4f','#2c8c99','#b5651d','#8e44ad','#2980b9','#e74c3c','#27ae60','#f39c12'];
    // Timeline chart
    const hrs=body.timeline_hours||[];
    Plotly.newPlot('annTimelineChart',[
      {x:hrs,y:body.timeline_gen,mode:'lines',name:'Generation',line:{color:pal[0],width:1.5}},
      {x:hrs,y:body.timeline_load,mode:'lines',name:'Load',line:{color:pal[1],width:1.5}},
      {x:hrs,y:body.timeline_ren,mode:'lines',name:'Renewable',line:{color:pal[6],width:1.5}},
      {x:hrs,y:body.timeline_curt,mode:'lines',name:'Curtailment',line:{color:pal[5],width:1}},
      {x:hrs,y:body.timeline_ess,mode:'lines',name:'ESS (net)',line:{color:pal[3],width:1,dash:'dot'}},
    ],{title:'Annual Generation & Load Timeline (MW)',xaxis:{title:'Hour of Year'},yaxis:{title:'MW'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.15}},{responsive:true});
    // Monthly energy breakdown
    const mo=body.monthly_summaries||[];
    const mLabels=mo.map(m=>'Month '+(m.block_id+1));
    Plotly.newPlot('annMonthlyChart',[
      {x:mLabels,y:mo.map(m=>m.total_gen_mwh),type:'bar',name:'Generation',marker:{color:pal[0]}},
      {x:mLabels,y:mo.map(m=>m.total_renewable_mwh),type:'bar',name:'Renewable',marker:{color:pal[6]}},
      {x:mLabels,y:mo.map(m=>m.total_curtailment_mwh),type:'bar',name:'Curtailment',marker:{color:pal[5]}},
      {x:mLabels,y:mo.map(m=>m.total_loss_mwh),type:'bar',name:'Losses',marker:{color:pal[2]}},
    ],{title:'Monthly Energy Breakdown (MWh)',barmode:'group',xaxis:{title:''},yaxis:{title:'MWh'},
       margin:{l:60,r:15,t:45,b:60},legend:{orientation:'h',y:-0.25}},{responsive:true});
    // Monthly cost
    Plotly.newPlot('annMonthlyCostChart',[
      {x:mLabels,y:mo.map(m=>m.total_cost),type:'bar',name:'Cost',marker:{color:pal[3]}},
    ],{title:'Monthly Cost ($)',xaxis:{title:''},yaxis:{title:'$'},
       margin:{l:60,r:15,t:45,b:60}},{responsive:true});
    // Generator stats
    const gs=body.gen_stats||[];
    if(gs.length) Plotly.newPlot('annGenStatsChart',[
      {x:gs.map(g=>g.name||'Gen'),y:gs.map(g=>g.capacity_factor*100),type:'bar',name:'Capacity Factor %',marker:{color:pal[0]}},
    ],{title:'Generator Capacity Factors (%)',xaxis:{title:''},yaxis:{title:'%'},
       margin:{l:55,r:15,t:45,b:80}},{responsive:true});
    // Renewable stats
    const rns=body.renewable_stats||[];
    if(rns.length) Plotly.newPlot('annRenStatsChart',[
      {x:rns.map(r=>r.name||'Ren'),y:rns.map(r=>r.total_energy_mwh),type:'bar',name:'Energy MWh',marker:{color:pal[6]}},
      {x:rns.map(r=>r.name||'Ren'),y:rns.map(r=>r.total_curtailed_mwh),type:'bar',name:'Curtailed MWh',marker:{color:pal[5]}},
    ],{title:'Renewable Energy vs Curtailment (MWh)',barmode:'group',xaxis:{title:''},yaxis:{title:'MWh'},
       margin:{l:60,r:15,t:45,b:80},legend:{orientation:'h',y:-0.25}},{responsive:true});
    // Storage stats
    const stg=body.storage_stats||[];
    if(stg.length) Plotly.newPlot('annStorageStatsChart',[
      {x:stg.map(s=>s.name||'ESS'),y:stg.map(s=>s.total_charge_mwh),type:'bar',name:'Charge MWh',marker:{color:pal[1]}},
      {x:stg.map(s=>s.name||'ESS'),y:stg.map(s=>s.total_discharge_mwh),type:'bar',name:'Discharge MWh',marker:{color:pal[2]}},
    ],{title:'Storage Charge/Discharge (MWh) — Cycles: '+stg.map(s=>fmt(s.cycles,1)).join(', '),
       barmode:'group',xaxis:{title:''},yaxis:{title:'MWh'},
       margin:{l:60,r:15,t:45,b:80},legend:{orientation:'h',y:-0.25}},{responsive:true});
    // Voltage heatmap from PF snapshots
    const snapHrs=body.snapshot_hours||[];
    const snapVm=body.snapshot_vm||[];
    if(snapHrs.length&&snapVm.length){
      const nBus=snapVm[0].length;
      const busLabels=Array.from({length:nBus},(_,i)=>'Bus '+i);
      Plotly.newPlot('annVoltHeatmap',[{z:snapVm,x:busLabels,y:snapHrs,type:'heatmap',
        colorscale:'RdYlGn',zmin:0.9,zmax:1.1,colorbar:{title:'Vm (pu)',len:0.7}}],
        {title:'Bus Voltage Heatmap over Year',xaxis:{title:'Bus'},yaxis:{title:'Hour'},
         margin:{l:60,r:80,t:45,b:50}},{responsive:true});
    }
    setStatus('Annual simulation complete. Feasible: '+body.feasible+'. Annual cost: $'+fmt(body.total_cost,0)+'. Gen: '+fmt(body.total_gen_mwh,0)+' MWh.');
    // Setup animation data
    setupAnnualAnimation(body);
  }catch(e){setStatus(e.message,true);}
};

/* Annual Animation Controls */
let annAnimData=null;
let annAnimPlaying=false;
let annAnimFrame=0;
let annAnimTimer=null;

function setupAnnualAnimation(body){
  // Store time-series data for animation
  annAnimData={
    hours:body.timeline_hours||[],
    gen:body.timeline_gen||[],
    load:body.timeline_load||[],
    ren:body.timeline_ren||[],
    curt:body.timeline_curt||[],
    ess:body.timeline_ess||[],
    snapshot_hours:body.snapshot_hours||[],
    snapshot_vm:body.snapshot_vm||[],
    geo_buses:body.geo_buses||[],
    geo_ac_branches:body.geo_ac_branches||[],
  };
  annAnimFrame=0;
  annAnimPlaying=false;
  if(annAnimTimer)clearInterval(annAnimTimer);
  annAnimTimer=null;
  
  const ctrl=document.getElementById('annAnimControls');
  const slider=document.getElementById('annAnimSlider');
  const geoMap=document.getElementById('annGeoMap');
  
  if(annAnimData.hours.length>0){
    ctrl.style.display='block';
    slider.max=annAnimData.hours.length-1;
    slider.value=0;
    document.getElementById('annAnimTimeLabel').textContent='Hour '+annAnimData.hours[0];
    document.getElementById('annAnimPlayPause').innerHTML='&#9658; Play';
    geoMap.style.display=document.getElementById('annAnimGeo').checked?'block':'none';
    renderAnnualGeoFrame(0);
  } else {
    ctrl.style.display='none';
    geoMap.style.display='none';
  }
}

function renderAnnualGeoFrame(idx){
  if(!annAnimData||!annAnimData.geo_buses.length)return;
  
  const buses=annAnimData.geo_buses;
  const branches=annAnimData.geo_ac_branches||[];
  const hasGeo=buses.some(b=>Math.abs(b.lat)>0.001||Math.abs(b.lon)>0.001);
  
  // Interpolate voltage from snapshots if available
  let vmVals=null;
  if(annAnimData.snapshot_vm.length>0&&annAnimData.snapshot_hours.length>0){
    const hr=annAnimData.hours[idx];
    // Find closest snapshot
    let closest=0;
    let minDiff=Math.abs(annAnimData.snapshot_hours[0]-hr);
    for(let i=1;i<annAnimData.snapshot_hours.length;i++){
      const d=Math.abs(annAnimData.snapshot_hours[i]-hr);
      if(d<minDiff){minDiff=d;closest=i;}
    }
    vmVals=annAnimData.snapshot_vm[closest];
  }
  
  const traces=[];
  
  // Branch traces
  for(const br of branches){
    traces.push({
      type:'scattergeo',
      mode:'lines',
      lon:[br.from_lon,br.to_lon],
      lat:[br.from_lat,br.to_lat],
      line:{width:2,color:'#0b6e4f'},
      hoverinfo:'text',
      text:`${br.from_bus}-${br.to_bus}`,
      showlegend:false
    });
  }
  
  // Bus markers with dynamic voltage coloring
  const acBuses=buses.filter(b=>b.type==='AC');
  if(acBuses.length){
    const colors=vmVals?acBuses.map(b=>{const v=vmVals[b.id-1];return v!==undefined?v:1.0;}):acBuses.map(b=>1.0);
    // Compute dynamic color range based on actual voltage spread
    const vmMin=Math.min(...colors);
    const vmMax=Math.max(...colors);
    const vmRange=Math.max(vmMax-vmMin,0.01);  // At least 0.01 spread
    const cmin=Math.max(0.9, vmMin-vmRange*0.2);
    const cmax=Math.min(1.1, vmMax+vmRange*0.2);
    traces.push({
      type:'scattergeo',
      mode:'markers',
      lon:acBuses.map(b=>b.lon),
      lat:acBuses.map(b=>b.lat),
      marker:{size:12,color:colors,colorscale:'RdYlGn',cmin:cmin,cmax:cmax,colorbar:{title:'Vm (pu)',x:1.02,thickness:12,len:0.7}},
      hoverinfo:'text',
      hovertext:acBuses.map((b,i)=>`AC${b.id} ${b.name||''}<br>Vm=${colors[i].toFixed(4)} pu`),
      name:'AC Buses',
      showlegend:true
    });
  }
  
  const dcBuses=buses.filter(b=>b.type==='DC');
  if(dcBuses.length){
    // DC buses also get voltage-based coloring from vdc data
    const dcColors=annAnimData.snapshot_vdc&&annAnimData.snapshot_vdc[Math.floor(idx*annAnimData.snapshot_vdc.length/Math.max(1,annAnimData.hours.length))]||dcBuses.map(()=>1.0);
    const dcMin=Math.min(...dcColors);
    const dcMax=Math.max(...dcColors);
    const dcRange=Math.max(dcMax-dcMin,0.01);
    traces.push({
      type:'scattergeo',
      mode:'markers',
      lon:dcBuses.map(b=>b.lon),
      lat:dcBuses.map(b=>b.lat),
      marker:{size:12,color:dcColors.slice(0,dcBuses.length),colorscale:'Purples',cmin:Math.max(0.9,dcMin-dcRange*0.2),cmax:Math.min(1.1,dcMax+dcRange*0.2),symbol:'square'},
      hoverinfo:'text',
      hovertext:dcBuses.map((b,i)=>`DC${b.id} ${b.name||''}<br>Vdc=${(dcColors[i]||1.0).toFixed(4)} pu`),
      name:'DC Buses',
      showlegend:true
    });
  }
  
  // Summary text annotation
  const genVal=annAnimData.gen[idx]||0;
  const loadVal=annAnimData.load[idx]||0;
  const renVal=annAnimData.ren[idx]||0;
  
  const layout={
    title:`Network State @ Hour ${annAnimData.hours[idx]} | Gen: ${genVal.toFixed(0)} MW | Load: ${loadVal.toFixed(0)} MW | Ren: ${renVal.toFixed(0)} MW`,
    geo:{
      scope:hasGeo?undefined:'usa',
      projection:{type:hasGeo?'mercator':'albers usa'},
      showland:true,landcolor:'#f5f5dc',
      showlakes:true,lakecolor:'#a0d2db',
      showcountries:true,countrycolor:'#888',
      resolution:hasGeo?50:110,
      lonaxis:hasGeo?(()=>{const lons=buses.map(b=>b.lon);const minL=Math.min(...lons),maxL=Math.max(...lons);const span=maxL-minL;const pad=Math.max(0.002,span*0.08);return{range:[minL-pad,maxL+pad]};})():undefined,
      lataxis:hasGeo?(()=>{const lats=buses.map(b=>b.lat);const minL=Math.min(...lats),maxL=Math.max(...lats);const span=maxL-minL;const pad=Math.max(0.002,span*0.08);return{range:[minL-pad,maxL+pad]};})():undefined
    },
    margin:{l:0,r:0,t:50,b:0},
    legend:{x:0,y:1,bgcolor:'rgba(255,255,255,0.7)'}
  };
  
  Plotly.react('annGeoMap',traces,layout);
  
  // Update timeline chart marker
  Plotly.relayout('annTimelineChart',{
    shapes:[{type:'line',x0:annAnimData.hours[idx],x1:annAnimData.hours[idx],y0:0,y1:1,yref:'paper',
             line:{color:'red',width:2,dash:'dot'}}]
  });
}

function stepAnnualAnimation(){
  if(!annAnimData||annAnimData.hours.length===0)return;
  annAnimFrame++;
  if(annAnimFrame>=annAnimData.hours.length){
    annAnimFrame=0;
  }
  document.getElementById('annAnimSlider').value=annAnimFrame;
  document.getElementById('annAnimTimeLabel').textContent='Hour '+annAnimData.hours[annAnimFrame];
  if(document.getElementById('annAnimGeo').checked){
    renderAnnualGeoFrame(annAnimFrame);
  }
}

document.getElementById('annAnimPlayPause').onclick=function(){
  if(!annAnimData||annAnimData.hours.length===0)return;
  annAnimPlaying=!annAnimPlaying;
  this.innerHTML=annAnimPlaying?'&#10074;&#10074; Pause':'&#9658; Play';
  if(annAnimPlaying){
    const speed=parseInt(document.getElementById('annAnimSpeed').value)||500;
    annAnimTimer=setInterval(stepAnnualAnimation,speed);
  } else {
    if(annAnimTimer)clearInterval(annAnimTimer);
    annAnimTimer=null;
  }
};

document.getElementById('annAnimReset').onclick=function(){
  annAnimPlaying=false;
  if(annAnimTimer)clearInterval(annAnimTimer);
  annAnimTimer=null;
  annAnimFrame=0;
  document.getElementById('annAnimPlayPause').innerHTML='&#9658; Play';
  if(annAnimData&&annAnimData.hours.length){
    document.getElementById('annAnimSlider').value=0;
    document.getElementById('annAnimTimeLabel').textContent='Hour '+annAnimData.hours[0];
    renderAnnualGeoFrame(0);
  }
};

document.getElementById('annAnimSlider').oninput=function(){
  if(!annAnimData||annAnimData.hours.length===0)return;
  annAnimFrame=parseInt(this.value);
  document.getElementById('annAnimTimeLabel').textContent='Hour '+annAnimData.hours[annAnimFrame];
  if(document.getElementById('annAnimGeo').checked){
    renderAnnualGeoFrame(annAnimFrame);
  }
};

document.getElementById('annAnimSpeed').onchange=function(){
  if(annAnimPlaying&&annAnimTimer){
    clearInterval(annAnimTimer);
    const speed=parseInt(this.value)||500;
    annAnimTimer=setInterval(stepAnnualAnimation,speed);
  }
};

document.getElementById('annAnimGeo').onchange=function(){
  const geoMap=document.getElementById('annGeoMap');
  geoMap.style.display=this.checked?'block':'none';
  if(this.checked&&annAnimData)renderAnnualGeoFrame(annAnimFrame);
};

/* Lifecycle Simulation */
document.getElementById('runLifecycleBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    setStatus('Running Lifecycle Simulation (multi-year, may take a while)...');
    const body=await api('/api/session/run_lifecycle_sim',{
      num_years:parseInt(document.getElementById('lcNumYears').value)||20,
      discount_rate:parseFloat(document.getElementById('lcDiscountRate').value)||0.05,
      load_growth_rate:parseFloat(document.getElementById('lcLoadGrowth').value)||0.02,
      pv_annual_derating:parseFloat(document.getElementById('lcPVDerating').value)||0.005,
      calendar_degradation:parseFloat(document.getElementById('lcCalDegrad').value)||0.02,
      resolution:document.getElementById('lcResolution').value,
      pv_scale:parseFloat(document.getElementById('lcPVScale').value)||1.0,
      wind_scale:parseFloat(document.getElementById('lcWindScale').value)||1.0,
      bess_power_scale:parseFloat(document.getElementById('lcBESSPowerScale').value)||1.0,
      bess_energy_scale:parseFloat(document.getElementById('lcBESSEnergyScale').value)||1.0,
      diesel_scale:parseFloat(document.getElementById('lcDieselScale').value)||1.0,
    });
    document.getElementById('lcFeas').textContent=body.feasible?'Yes':'No';
    document.getElementById('lcNPV').textContent='$'+fmt(body.npv_total_cost,0);
    document.getElementById('lcTotalCarbon').textContent=fmt(body.total_carbon_tco2,0)+' tCO\u2082';
    document.getElementById('lcReplacements').textContent=body.total_replacements;
    document.getElementById('lcReplCost').textContent='$'+fmt(body.total_replacement_cost,0);
    document.getElementById('lcYears').textContent=body.num_years;
    const pal=['#0b6e4f','#2c8c99','#b5651d','#8e44ad','#2980b9','#e74c3c','#27ae60','#f39c12'];
    const yrs=(body.years||[]).map(y=>y.year);
    // Cost trajectory with replacement markers
    const costs=(body.years||[]).map(y=>y.annual_cost);
    const replYears=(body.replacements||[]).map(r=>r.year);
    const replCosts=(body.replacements||[]).map(r=>r.replacement_cost_usd);
    Plotly.newPlot('lcCostChart',[
      {x:yrs,y:costs,mode:'lines+markers',name:'Annual Cost',line:{color:pal[0],width:2},marker:{size:5}},
      {x:replYears,y:replCosts,mode:'markers',name:'Replacement Cost',marker:{color:pal[5],size:12,symbol:'diamond'}},
    ],{title:'Annual Cost Trajectory with Replacements',xaxis:{title:'Year',dtick:Math.max(1,Math.floor(yrs.length/10))},yaxis:{title:'$ Cost'},
       margin:{l:70,r:15,t:45,b:50},legend:{orientation:'h',y:-0.15}},{responsive:true});
    // Carbon trajectory with bounds
    const carbon=(body.years||[]).map(y=>y.annual_carbon_tco2);
    const boundUpper=(body.years||[]).map(y=>y.annual_carbon_tco2+y.bounds.total_bound_tco2);
    const boundLower=(body.years||[]).map(y=>Math.max(0,y.annual_carbon_tco2-y.bounds.total_bound_tco2));
    Plotly.newPlot('lcCarbonChart',[
      {x:yrs,y:boundUpper,mode:'lines',line:{color:'rgba(231,76,60,0.2)',width:0},showlegend:false},
      {x:yrs,y:boundLower,mode:'lines',fill:'tonexty',fillcolor:'rgba(231,76,60,0.12)',line:{color:'rgba(231,76,60,0.2)',width:0},name:'Error Bound'},
      {x:yrs,y:carbon,mode:'lines+markers',name:'Annual CO\u2082',line:{color:pal[5],width:2},marker:{size:5}},
    ],{title:'Annual Carbon Emissions with Theoretical Bounds',xaxis:{title:'Year'},yaxis:{title:'tCO\u2082'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    // Carbon intensity trajectory
    const carbonIntensity=(body.years||[]).map(y=>y.avg_carbon_intensity);
    const cumCarbon=(body.years||[]).reduce((acc,y)=>{const prev=acc.length>0?acc[acc.length-1]:0;acc.push(prev+y.annual_carbon_tco2);return acc;},[]);
    Plotly.newPlot('lcCarbonIntensityChart',[
      {x:yrs,y:carbonIntensity,mode:'lines+markers',name:'Avg Intensity (tCO\u2082/MWh)',line:{color:pal[2],width:2},marker:{size:5},yaxis:'y'},
      {x:yrs,y:cumCarbon,mode:'lines',name:'Cumulative CO\u2082 (tCO\u2082)',line:{color:pal[5],width:2,dash:'dot'},yaxis:'y2'},
    ],{title:'Carbon Intensity & Cumulative Emissions',
       xaxis:{title:'Year',dtick:Math.max(1,Math.floor(yrs.length/10))},
       yaxis:{title:'tCO\u2082/MWh',rangemode:'tozero',side:'left'},
       yaxis2:{title:'Cumulative tCO\u2082',overlaying:'y',side:'right',rangemode:'tozero'},
       margin:{l:60,r:60,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    // Energy breakdown
    const gen=(body.years||[]).map(y=>y.annual_gen_mwh);
    const ren=(body.years||[]).map(y=>y.annual_renewable_mwh);
    const load=(body.years||[]).map(y=>y.annual_load_mwh);
    const curt=(body.years||[]).map(y=>y.annual_curtailment_mwh);
    Plotly.newPlot('lcEnergyChart',[
      {x:yrs,y:gen,mode:'lines',name:'Generation',line:{color:pal[0],width:2}},
      {x:yrs,y:load,mode:'lines',name:'Load',line:{color:pal[1],width:2}},
      {x:yrs,y:ren,mode:'lines',name:'Renewable',line:{color:pal[6],width:2}},
      {x:yrs,y:curt,mode:'lines',name:'Curtailment',line:{color:pal[5],width:1,dash:'dot'}},
    ],{title:'Energy Trajectories (MWh)',xaxis:{title:'Year'},yaxis:{title:'MWh'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    // Battery SOH curves
    const storNames=[...new Set((body.years||[]).flatMap(y=>(y.storage_states||[]).map(s=>s.name)))];
    const sohTraces=storNames.map((nm,si)=>{
      const sohVals=yrs.map(yr=>{const y=(body.years||[]).find(yy=>yy.year===yr);if(!y)return null;const s=(y.storage_states||[]).find(ss=>ss.name===nm);return s?s.soh:null;});
      return {x:yrs,y:sohVals,mode:'lines+markers',name:nm,line:{color:pal[si%pal.length],width:2},marker:{size:4}};
    });
    // Add EOL threshold line
    sohTraces.push({x:[yrs[0],yrs[yrs.length-1]],y:[0.8,0.8],mode:'lines',name:'EOL Threshold',line:{color:'#c34d4d',width:1,dash:'dash'}});
    Plotly.newPlot('lcSOHChart',sohTraces,{title:'Battery State of Health',xaxis:{title:'Year'},yaxis:{title:'SOH',range:[0,1.05]},
       margin:{l:55,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    // PV derating curves
    const pvNames=[...new Set((body.years||[]).flatMap(y=>(y.renewable_states||[]).map(r=>r.name)))];
    const pvTraces=pvNames.map((nm,ri)=>{
      const capVals=yrs.map(yr=>{const y=(body.years||[]).find(yy=>yy.year===yr);if(!y)return null;const r=(y.renewable_states||[]).find(rr=>rr.name===nm);return r?r.derated_capacity_mw:null;});
      return {x:yrs,y:capVals,mode:'lines+markers',name:nm,line:{color:pal[(ri+2)%pal.length],width:2},marker:{size:4}};
    });
    Plotly.newPlot('lcPVChart',pvTraces,{title:'PV Capacity Derating (MW)',xaxis:{title:'Year'},yaxis:{title:'MW'},
       margin:{l:55,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    // Theoretical bounds breakdown
    const bDispatch=(body.years||[]).map(y=>y.bounds.dispatch_bound_tco2);
    const bSampling=(body.years||[]).map(y=>y.bounds.sampling_bound_tco2);
    const bStorage=(body.years||[]).map(y=>y.bounds.storage_carbon_bound_tco2);
    Plotly.newPlot('lcBoundsChart',[
      {x:yrs,y:bDispatch,type:'bar',name:'Dispatch Approx.',marker:{color:pal[2]}},
      {x:yrs,y:bSampling,type:'bar',name:'Sampling Error',marker:{color:pal[3]}},
      {x:yrs,y:bStorage,type:'bar',name:'Storage Carbon',marker:{color:pal[4]}},
    ],{title:'Theoretical Error Bounds Breakdown (tCO\u2082)',barmode:'stack',xaxis:{title:'Year'},yaxis:{title:'tCO\u2082'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    // Cross-validation: sampling gap (dense vs sampled)
    const cvDense=(body.years||[]).map(y=>(y.cross_validation||{}).dense_carbon_tco2||0);
    const cvSampled=(body.years||[]).map(y=>(y.cross_validation||{}).sampled_carbon_tco2||0);
    const cvGapPct=(body.years||[]).map(y=>(y.cross_validation||{}).sampling_gap_pct||0);
    const cvBoundPct=(body.years||[]).map(y=>(y.cross_validation||{}).total_bound_pct||0);
    const cvDispPct=(body.years||[]).map(y=>(y.cross_validation||{}).dispatch_bound_pct||0);
    const cvSampPct=(body.years||[]).map(y=>(y.cross_validation||{}).sampling_bound_pct||0);
    const cvStorPct=(body.years||[]).map(y=>(y.cross_validation||{}).storage_bound_pct||0);
    Plotly.newPlot('lcBoundsBreakdownChart',[
      {x:yrs,y:cvDispPct,type:'bar',name:'Dispatch Bound %',marker:{color:pal[2]}},
      {x:yrs,y:cvSampPct,type:'bar',name:'Sampling Bound %',marker:{color:pal[3]}},
      {x:yrs,y:cvStorPct,type:'bar',name:'Storage Bound %',marker:{color:pal[4]}},
    ],{title:'Theoretical Error Bounds as % of Annual Carbon',barmode:'stack',
       xaxis:{title:'Year'},yaxis:{title:'%',rangemode:'tozero'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    // Dense vs sampled carbon comparison
    Plotly.newPlot('lcCrossValChart',[
      {x:yrs,y:cvDense,mode:'lines+markers',name:'Dense (All Hours)',line:{color:pal[1],width:2},marker:{size:5}},
      {x:yrs,y:cvSampled,mode:'lines+markers',name:'Stratified Sample',line:{color:pal[3],width:2,dash:'dash'},marker:{size:5}},
    ],{title:'Carbon Cross-Validation: Dense vs Sampled Estimate (tCO\u2082)',
       xaxis:{title:'Year'},yaxis:{title:'tCO\u2082'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}
    },{responsive:true});
    // Sampling gap % vs total bound %
    Plotly.newPlot('lcTightnessChart',[
      {x:yrs,y:cvBoundPct,mode:'lines+markers',name:'Total Bound / Carbon %',line:{color:pal[5],width:2,dash:'dash'},marker:{size:5},fill:'tozeroy',fillcolor:'rgba(231,76,60,0.08)'},
      {x:yrs,y:cvGapPct,mode:'lines+markers',name:'Sampling Gap / Carbon %',line:{color:pal[0],width:2},marker:{size:5},fill:'tozeroy',fillcolor:'rgba(11,110,79,0.08)'},
    ],{title:'Validation: Theoretical Bound vs Sampling Gap (% of Carbon)',
       xaxis:{title:'Year'},yaxis:{title:'%',rangemode:'tozero'},
       margin:{l:55,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}
    },{responsive:true});
    setStatus('Lifecycle simulation complete. '+body.num_years+' years. NPV: $'+fmt(body.npv_total_cost,0)+'. Carbon: '+fmt(body.total_carbon_tco2,0)+' tCO\u2082.');
  }catch(e){setStatus(e.message,true);}
};

/* Capacity Comparison Sweep */
document.getElementById('runCapCompareBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  try{
    const sweepParam=document.getElementById('lcSweepParam').value;
    const minS=parseFloat(document.getElementById('lcSweepMin').value)||0.5;
    const maxS=parseFloat(document.getElementById('lcSweepMax').value)||3.0;
    const steps=parseInt(document.getElementById('lcSweepSteps').value)||6;
    document.getElementById('lcCompareStatus').textContent='Running '+steps+' scenarios for '+sweepParam+'...';
    setStatus('Capacity comparison: sweeping '+sweepParam+' ('+steps+' scenarios)...');
    const body=await api('/api/session/run_lifecycle_compare',{
      num_years:parseInt(document.getElementById('lcNumYears').value)||20,
      discount_rate:parseFloat(document.getElementById('lcDiscountRate').value)||0.05,
      load_growth_rate:parseFloat(document.getElementById('lcLoadGrowth').value)||0.02,
      pv_annual_derating:parseFloat(document.getElementById('lcPVDerating').value)||0.005,
      calendar_degradation:parseFloat(document.getElementById('lcCalDegrad').value)||0.02,
      resolution:document.getElementById('lcResolution').value,
      pv_scale:parseFloat(document.getElementById('lcPVScale').value)||1.0,
      wind_scale:parseFloat(document.getElementById('lcWindScale').value)||1.0,
      bess_power_scale:parseFloat(document.getElementById('lcBESSPowerScale').value)||1.0,
      bess_energy_scale:parseFloat(document.getElementById('lcBESSEnergyScale').value)||1.0,
      diesel_scale:parseFloat(document.getElementById('lcDieselScale').value)||1.0,
      sweep_param:sweepParam,
      sweep_min:minS,
      sweep_max:maxS,
      sweep_steps:steps,
    });
    const scens=body.scenarios||[];
    const labels=scens.map(s=>s.label);
    const pal=['#0b6e4f','#2c8c99','#b5651d','#8e44ad','#2980b9','#e74c3c','#27ae60','#f39c12','#1abc9c','#d35400'];
    const paramLabel={pv:'PV (MW)',wind:'Wind (MW)',bess_power:'BESS Power (MW)',bess_energy:'BESS Energy (MWh)',diesel:'Diesel (MW)'}[sweepParam]||sweepParam;
    const capVals=scens.map(s=>{
      if(sweepParam==='pv') return s.total_pv_mw;
      if(sweepParam==='wind') return s.total_wind_mw;
      if(sweepParam==='bess_power') return s.total_bess_mw;
      if(sweepParam==='bess_energy') return s.total_bess_mwh;
      if(sweepParam==='diesel') return s.total_diesel_mw;
      return 0;
    });
    // NPV vs capacity
    Plotly.newPlot('lcCompNPVChart',[
      {x:capVals,y:scens.map(s=>s.npv_total_cost),mode:'lines+markers',name:'NPV Cost',line:{color:pal[0],width:2},marker:{size:8}},
    ],{title:'NPV Total Cost vs '+paramLabel,xaxis:{title:paramLabel},yaxis:{title:'$ NPV Cost'},
       margin:{l:70,r:15,t:45,b:50}},{responsive:true});
    // Total carbon vs capacity
    Plotly.newPlot('lcCompCarbonChart',[
      {x:capVals,y:scens.map(s=>s.total_carbon_tco2),mode:'lines+markers',name:'Total CO\u2082',line:{color:pal[5],width:2},marker:{size:8}},
    ],{title:'Total Carbon vs '+paramLabel,xaxis:{title:paramLabel},yaxis:{title:'tCO\u2082'},
       margin:{l:60,r:15,t:45,b:50}},{responsive:true});
    // Pareto: NPV vs Carbon
    Plotly.newPlot('lcCompParetoChart',[
      {x:scens.map(s=>s.total_carbon_tco2),y:scens.map(s=>s.npv_total_cost),
       mode:'markers+text',text:labels,textposition:'top center',textfont:{size:9},
       marker:{size:12,color:capVals,colorscale:'Viridis',showscale:true,colorbar:{title:paramLabel,len:0.7}},
       name:'Scenarios'},
    ],{title:'Pareto Front: NPV Cost vs Total Carbon',xaxis:{title:'Total tCO\u2082'},yaxis:{title:'$ NPV Cost'},
       margin:{l:70,r:70,t:45,b:50}},{responsive:true});
    // Carbon trajectory comparison (each scenario as a line)
    const trajTraces=scens.map((s,si)=>{
      const yrs=s.yearly_carbon.map((_,i)=>i+1);
      return {x:yrs,y:s.yearly_carbon,mode:'lines',name:s.label,line:{color:pal[si%pal.length],width:1.5}};
    });
    Plotly.newPlot('lcCompCarbonTrajChart',trajTraces,{title:'Annual Carbon Trajectories by Scenario',
       xaxis:{title:'Year'},yaxis:{title:'tCO\u2082'},
       margin:{l:60,r:15,t:45,b:50},legend:{orientation:'h',y:-0.18}},{responsive:true});
    document.getElementById('lcCompareStatus').textContent='\u2705 Comparison complete: '+scens.length+' scenarios for '+paramLabel;
    setStatus('Capacity comparison complete. '+scens.length+' scenarios.');
  }catch(e){setStatus(e.message,true);document.getElementById('lcCompareStatus').textContent='Error: '+e.message;}
};

/* Dashboard */
document.getElementById('runDashBtn').onclick=async()=>{
  if(!SYS){setStatus('Load a system first.',true);return;}
  const ds=document.getElementById('dashStatus');
  ds.textContent='Running Time-Series PF...'; setStatus('Dashboard: running full analysis...');
  const numSteps=24; const hrs=Array.from({length:numSteps},(_,i)=>i);
  const results={};
  try{results.tspf=await api('/api/session/run_ts_pf',{num_steps:numSteps,skip_uc:false,run_opf:false});}
  catch(e){results.tspf=null;ds.textContent='TS-PF failed: '+e.message;}
  ds.textContent='Running Carbon Analysis...';
  try{results.carbon=await api('/api/session/run_carbon',{});}
  catch(e){results.carbon=null;}
  // KPI row
  const kd=[];
  if(results.tspf){
    kd.push({v:results.tspf.num_converged+'/'+results.tspf.num_steps,l:'TS-PF Conv.'});
    kd.push({v:'$'+fmt(results.tspf.total_generation_cost,0),l:'Gen Cost'});
    const avgLoss=results.tspf.losses_mw?(results.tspf.losses_mw.reduce((a,b)=>a+b,0)/numSteps).toFixed(2):'-';
    kd.push({v:avgLoss+' MW',l:'Avg Losses'});
  }
  if(results.carbon){
    const ts2=results.carbon.tracing_summary||{};
    kd.push({v:fmt(ts2.total_generation_emissions_tco2,1)+' tCO\u2082',l:'Gen Emissions'});
    kd.push({v:fmt(ts2.total_load_emissions_tco2,1)+' tCO\u2082',l:'Load Emissions'});
    if(results.carbon.bus_carbon&&results.carbon.bus_carbon.length){
      const avgCI=results.carbon.bus_carbon.reduce((a,b)=>a+b.carbon_intensity_tco2_mwh,0)/results.carbon.bus_carbon.length;
      kd.push({v:fmt(avgCI,3)+' tCO\u2082/MWh',l:'Avg Carbon Int.'});
    }
  }
  document.getElementById('dashKPIs').innerHTML=kd.map(k=>'<div class="kpi"><div class="v">'+k.v+'</div><div class="l">'+k.l+'</div></div>').join('');
  const pal=['#0b6e4f','#2c8c99','#b5651d','#8e44ad','#2980b9','#e74c3c','#27ae60','#f39c12'];
  const M={l:50,r:15,t:40,b:45};
  if(results.tspf){
    const dtraces=(results.tspf.gen_dispatch||[]).map((d,gi)=>({x:hrs,y:d,type:'bar',name:results.tspf.gen_names[gi]||'Gen '+gi,marker:{color:pal[gi%pal.length]}}));
    (results.tspf.renewable_dispatch||[]).forEach((rd,ri)=>dtraces.push({x:hrs,y:rd,type:'bar',name:results.tspf.ren_names[ri]||'Ren '+ri,marker:{color:'#27ae60'}}));
    Plotly.newPlot('dashGenChart',dtraces,{title:'Generation Dispatch',barmode:'stack',xaxis:{title:'Hour'},yaxis:{title:'MW'},margin:M,showlegend:false,height:300},{responsive:true});
    if(results.tspf.vm_mean){
      const vt=[];const d2=results.tspf;
      if(d2.vm_max&&d2.vm_min){vt.push({x:hrs,y:d2.vm_max,mode:'lines',line:{width:0},showlegend:false});vt.push({x:hrs,y:d2.vm_min,mode:'lines',line:{width:0},fill:'tonexty',fillcolor:'rgba(11,110,79,0.12)',name:'Range'});}
      vt.push({x:hrs,y:d2.vm_mean,mode:'lines',line:{color:'#0b6e4f',width:2},name:'Mean'});
      const allV2=[...d2.vm_mean,...(d2.vm_min||[]),...(d2.vm_max||[])].filter(v=>v>0);
      const vL=Math.min(...allV2),vH=Math.max(...allV2),vP2=Math.max((vH-vL)*0.15,0.005);
      Plotly.newPlot('dashVoltChart',vt,{title:'Bus Voltage (p.u.)',xaxis:{title:'Hour'},yaxis:{title:'p.u.',range:[vL-vP2,vH+vP2]},margin:M,height:300},{responsive:true});
    }
    if(results.tspf.losses_mw)
      Plotly.newPlot('dashLossChart',[{x:hrs,y:results.tspf.losses_mw,type:'bar',marker:{color:'#b5651d'}}],{title:'System Losses per Hour (MW)',xaxis:{title:'Hour'},yaxis:{title:'MW'},margin:M,height:300},{responsive:true});
    if(results.tspf.ess_soc&&results.tspf.ess_soc.length)
      Plotly.newPlot('dashESSChart',(results.tspf.ess_soc||[]).map((s,si)=>({x:hrs,y:s,mode:'lines',name:results.tspf.ess_names[si]||'ESS '+si})),{title:'ESS State of Charge',xaxis:{title:'Hour'},yaxis:{title:'SOC',range:[0,1]},margin:M,height:300},{responsive:true});
    else
      document.getElementById('dashESSChart').innerHTML='<div style="padding:20px;color:var(--muted);">No storage data.</div>';
    // Load demand profile (from load profile 0 scalings × total load)
    const loadFromLoads=(SYS.loads||[]).reduce((s,l)=>s+(l.p_mw||0),0);
    const loadFromBuses=(SYS.ac_buses||[]).reduce((s,b)=>s+(b.pd_mw||0),0);
    const totalLoad=loadFromLoads>0?loadFromLoads:loadFromBuses;
    const lpVals=[0.50,0.45,0.42,0.40,0.42,0.50,0.60,0.72,0.80,0.85,0.88,0.90,0.88,0.85,0.82,0.85,0.90,1.00,1.10,1.05,0.95,0.85,0.72,0.60];
    Plotly.newPlot('dashLoadChart',[{x:hrs,y:lpVals.map(s=>s*totalLoad),mode:'lines',fill:'tozeroy',fillcolor:'rgba(44,140,153,0.1)',line:{color:'#2c8c99',width:2},name:'Load'}],{title:'Total Load Demand (MW)',xaxis:{title:'Hour'},yaxis:{title:'MW'},margin:M,height:300},{responsive:true});
  }
  if(results.carbon&&results.carbon.bus_carbon&&results.carbon.bus_carbon.length){
    const ci=results.carbon.bus_carbon.map(b=>b.carbon_intensity_tco2_mwh);
    const mx=Math.max(...ci)||1;
    Plotly.newPlot('dashCarbonChart',[{x:results.carbon.bus_carbon.map(b=>'B'+b.bus_index),y:ci,type:'bar',
      marker:{color:ci,colorscale:'RdYlGn',reversescale:true,cmin:0,cmax:mx,showscale:true,colorbar:{len:0.8}}}],
      {title:'Carbon Intensity by Bus (tCO\u2082/MWh)',xaxis:{title:''},yaxis:{title:'tCO\u2082/MWh'},margin:{l:50,r:70,t:40,b:50},height:300},{responsive:true});
  } else {
    document.getElementById('dashCarbonChart').innerHTML='<div style="padding:20px;color:var(--muted);">Carbon data unavailable.</div>';
  }
  ds.textContent='\u2705 Full analysis complete! TS-PF: '+(results.tspf?results.tspf.num_converged+'/'+results.tspf.num_steps+' conv.':'failed')+'. Carbon: '+(results.carbon?(fmt(results.carbon.tracing_summary.total_generation_emissions_tco2,1)+' tCO\u2082'):'failed')+'.'
  setStatus('Dashboard analysis complete.');
};
</script>
</body>
</html>)html";
}
#endif

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  httplib::Server svr;
  svr.new_task_queue = [] { return new httplib::ThreadPool(8); };
  // Request *reading* timeout — bounds how long a worker thread waits for the
  // request bytes (NOT the handler/computation, which runs unbounded after the
  // request is read).  Kept modest so a slow or body-less POST (e.g. a POST
  // with no Content-Length) cannot tie up a pool thread for many minutes.
  svr.set_read_timeout(120, 0);   // 2 min to read the request
  svr.set_write_timeout(600, 0);  // 10 min to write response (large exports)
  svr.set_keep_alive_max_count(100);
  svr.set_idle_interval(0, 500000); // 0.5 sec idle check

  // Global exception handler — catch anything that escapes per-route handlers
  svr.set_exception_handler([](const httplib::Request&, httplib::Response& res, std::exception_ptr ep) {
    g_session.busy.store(false);
    try {
      if (ep) std::rethrow_exception(ep);
    } catch (const std::exception& e) {
      res.status = 500;
      res.set_content(json{{"error", std::string("Server error: ") + e.what()}}.dump(), "application/json");
      return;
    } catch (...) {}
    res.status = 500;
    res.set_content(json{{"error", "Unknown server error"}}.dump(), "application/json");
  });

  const std::string data_dir = [&]() {
    fs::path p(args.data_dir);
    // If absolute and exists, use it directly
    if (p.is_absolute() && fs::exists(p)) return p.string();
    // Try relative to cwd
    if (p.is_relative()) {
      auto c1 = fs::current_path() / p;
      if (fs::exists(c1)) return c1.string();
    }
    // Search common locations relative to cwd and the compile-time source root
    // so the server works regardless of cwd (repo root, build/, build/tests/).
    std::vector<fs::path> candidates = {
        fs::current_path() / ".." / "data",
        fs::current_path() / "data",
        fs::current_path() / ".." / ".." / "data",
        fs::current_path() / ".." / "matpower" / "data",
        fs::current_path() / "matpower" / "data",
    };
#ifdef HACDCPF_PROJECT_ROOT
    candidates.push_back(fs::path(HACDCPF_PROJECT_ROOT) / "data");
#endif
    for (const auto& c : candidates) {
      std::error_code ec;
      if (fs::exists(c, ec) && !ec) return fs::canonical(c).string();
    }
    return p.string();
  }();
  std::cout << "Data directory: " << data_dir << "\n";

  // Resolve MATPOWER directory (independent of data_dir).
  // Default: ../external_data/matpower, with auto-detection fallbacks so the
  // server works regardless of cwd (build/, repo root, etc.).
  const std::string matpower_dir = [&]() {
    fs::path p(args.matpower_dir);
    if (p.is_absolute() && fs::exists(p)) return p.string();
    if (p.is_relative()) {
      auto c1 = fs::current_path() / p;
      if (fs::exists(c1)) return fs::canonical(c1).string();
    }
    std::vector<fs::path> candidates = {
        fs::current_path() / "external_data" / "matpower",
        fs::current_path() / ".." / "external_data" / "matpower",
        fs::current_path() / ".." / ".." / "external_data" / "matpower",
    };
#ifdef HACDCPF_PROJECT_ROOT
    candidates.push_back(fs::path(HACDCPF_PROJECT_ROOT) / "external_data" / "matpower");
#endif
    candidates.push_back(fs::path(data_dir));  // legacy fallback: matpower files in data dir
    for (const auto& c : candidates) {
      std::error_code ec;
      if (fs::exists(c, ec) && !ec) return fs::canonical(c).string();
    }
    return p.string();
  }();
  std::cout << "MATPOWER directory: " << matpower_dir << "\n";

  svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
    res.set_redirect("/xjtu/");
  });

  svr.Get("/api/cases", [](const httplib::Request&, httplib::Response& res) {
    json out;
    out["cases"] = case_names();
    out["default_case"] = "ieee24_3area_acdc_expanded";
    res.set_content(out.dump(), "application/json");
  });

  svr.Get("/api/matpower_files", [&matpower_dir](const httplib::Request&, httplib::Response& res) {
    json out;
    out["files"] = list_matpower_files(matpower_dir);
    out["data_dir"] = matpower_dir;
    res.set_content(out.dump(), "application/json");
  });

  // ---- Session: load system ----
  svr.Post("/api/session/load_builtin",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      std::string name = j.value("case", "ieee24_3area_acdc_expanded");
      auto sys = build_case(name);
      std::lock_guard<std::mutex> lk(g_session.mu);
      g_session.current_system = std::move(sys);
      g_session.current_name = name;
      g_session.external_grid_carbon_profiles.clear();
      clear_cached_analysis(g_session);
      auto summary = system_summary(*g_session.current_system);
      summary["_raw_json"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      res.set_content(summary.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/load_matpower",
           [&matpower_dir](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      std::string filename = j.value("filename", "");
      if (filename.empty()) throw std::runtime_error("No filename provided");
      if (filename.find('/') != std::string::npos ||
          filename.find('\\') != std::string::npos ||
          filename.find("..") != std::string::npos) {
        throw std::runtime_error("Invalid filename");
      }
      fs::path fpath = fs::path(matpower_dir) / filename;
      if (!fs::exists(fpath)) throw std::runtime_error("File not found: " + filename);
      auto sys = hacdcpf::io::parse_matpower(fpath.string());
      sys.name = filename;
      std::lock_guard<std::mutex> lk(g_session.mu);
      g_session.current_system = std::move(sys);
      g_session.current_name = filename;
      g_session.external_grid_carbon_profiles.clear();
      clear_cached_analysis(g_session);
      auto summary = system_summary(*g_session.current_system);
      summary["_raw_json"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      res.set_content(summary.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/load_json_string",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      std::string js = j.value("json_string", "");
      if (js.empty()) throw std::runtime_error("Empty JSON string");
      auto sys = hacdcpf::io::from_json(js);

      std::lock_guard<std::mutex> lk(g_session.mu);
      g_session.current_system = std::move(sys);
      g_session.current_name = g_session.current_system->name;
      g_session.external_grid_carbon_profiles.clear();
      clear_cached_analysis(g_session);
      auto summary = system_summary(*g_session.current_system);
      summary["_raw_json"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      res.set_content(summary.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/new_empty",
           [](const httplib::Request&, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      sys.name = "New System";
      sys.base_mva = 100.0;
      sys.ac.base_mva = 100.0;
      sys.dc.base_mva = 100.0;
      std::lock_guard<std::mutex> lk(g_session.mu);
      g_session.current_system = std::move(sys);
      g_session.current_name = "New System";
      clear_cached_analysis(g_session);
      auto summary = system_summary(*g_session.current_system);
      summary["_raw_json"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      res.set_content(summary.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/export_json",
           [](const httplib::Request&, httplib::Response& res) {
    try {
      std::lock_guard<std::mutex> lk(g_session.mu);
      if (!g_session.current_system) throw std::runtime_error("No system loaded");
      json out;
      out["json_string"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      out["name"] = g_session.current_name;
      res.set_content(out.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Session: export the current system as an ETAP-schema .xlsx workbook ----
  // Returns the raw binary workbook (one sheet per ETAP element class) so the
  // browser can download it directly.  save_etap() throws if ETAP support is not
  // compiled in (HACDCPF_ENABLE_ETAP off), which surfaces here as a 400 error.
  svr.Post("/api/session/export_etap",
           [](const httplib::Request&, httplib::Response& res) {
    std::string tmp;
    try {
      std::string name;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        name = g_session.current_name;
        // Write to a unique temp workbook, then read the bytes back out.
        static std::atomic<int> export_counter{0};
        tmp = (std::filesystem::temp_directory_path() /
               ("hacdcpf_gui_export_" +
                std::to_string(export_counter.fetch_add(1)) + ".xlsx"))
                  .string();
        hacdcpf::io::EtapIoReport rep;
        hacdcpf::io::save_etap(*g_session.current_system, tmp, rep);
      }
      std::ifstream ifs(tmp, std::ios::binary);
      if (!ifs) throw std::runtime_error("Failed to read generated ETAP workbook");
      std::string bytes((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
      ifs.close();
      std::error_code ec;
      std::filesystem::remove(tmp, ec);

      // Sanitize the system name into a safe download filename.
      std::string safe;
      for (char ch : name) {
        safe += (std::isalnum(static_cast<unsigned char>(ch)) || ch == '-' || ch == '_')
                    ? ch : '_';
      }
      if (safe.empty()) safe = "system";
      res.set_header("Content-Disposition",
                     "attachment; filename=\"" + safe + ".xlsx\"");
      res.set_content(
          bytes,
          "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet");
    } catch (const std::exception& e) {
      if (!tmp.empty()) { std::error_code ec; std::filesystem::remove(tmp, ec); }
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Session: export the current system to native ETAP XML (PDE) ----
  svr.Post("/api/session/export_etap_xml",
           [](const httplib::Request&, httplib::Response& res) {
    std::string tmp;
    try {
      std::string name;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        name = g_session.current_name;
        static std::atomic<int> xml_export_counter{0};
        tmp = (std::filesystem::temp_directory_path() /
               ("hacdcpf_gui_export_" +
                std::to_string(xml_export_counter.fetch_add(1)) + ".xml"))
                  .string();
        hacdcpf::io::EtapIoReport rep;
        hacdcpf::io::save_etap_xml(*g_session.current_system, tmp, rep);
      }
      std::ifstream ifs(tmp, std::ios::binary);
      if (!ifs) throw std::runtime_error("Failed to read generated ETAP XML");
      std::string bytes((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
      ifs.close();
      std::error_code ec;
      std::filesystem::remove(tmp, ec);

      std::string safe;
      for (char ch : name) {
        safe += (std::isalnum(static_cast<unsigned char>(ch)) || ch == '-' || ch == '_')
                    ? ch : '_';
      }
      if (safe.empty()) safe = "system";
      res.set_content(json{{"xml_string", bytes}, {"name", safe}}.dump(),
                      "application/json");
    } catch (const std::exception& e) {
      if (!tmp.empty()) { std::error_code ec; std::filesystem::remove(tmp, ec); }
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Session: import a native ETAP project XML (e.g. Feeder.xml) ----
  svr.Post("/api/session/load_etap_xml",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      const std::string xml = j.value("xml_string", "");
      if (xml.empty()) throw std::runtime_error("Empty ETAP XML string");
      const std::string tmp =
          (std::filesystem::temp_directory_path() / "hacdcpf_gui_etap.xml").string();
      { std::ofstream ofs(tmp, std::ios::binary); ofs << xml; }
      hacdcpf::io::EtapIoReport rep;
      auto sys = hacdcpf::io::load_etap_xml(
          tmp, hacdcpf::io::EtapImportMode::Permissive, rep);
      std::error_code ec;
      std::filesystem::remove(tmp, ec);

      std::lock_guard<std::mutex> lk(g_session.mu);
      g_session.current_system = std::move(sys);
      g_session.current_name = g_session.current_system->name;
      clear_cached_analysis(g_session);
      auto summary = system_summary(*g_session.current_system);
      summary["_raw_json"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      summary["_etap_warnings"] = rep.warnings;
      res.set_content(summary.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Session: import an ETAP-schema .xlsx workbook (binary upload) ----
  // The raw workbook bytes are sent as the request body (OpenXLSX needs a real
  // file, so they are written to a temp file and load_etap() reads it back).
  svr.Post("/api/session/load_etap_xlsx",
           [](const httplib::Request& req, httplib::Response& res) {
    std::string tmp;
    try {
      if (req.body.empty()) throw std::runtime_error("Empty ETAP workbook upload");
      static std::atomic<int> import_counter{0};
      tmp = (std::filesystem::temp_directory_path() /
             ("hacdcpf_gui_import_" +
              std::to_string(import_counter.fetch_add(1)) + ".xlsx"))
                .string();
      { std::ofstream ofs(tmp, std::ios::binary); ofs.write(req.body.data(),
            static_cast<std::streamsize>(req.body.size())); }
      hacdcpf::io::EtapIoReport rep;
      auto sys = hacdcpf::io::load_etap(
          tmp, hacdcpf::io::EtapImportMode::Permissive, rep);
      std::error_code ec;
      std::filesystem::remove(tmp, ec);

      std::lock_guard<std::mutex> lk(g_session.mu);
      g_session.current_system = std::move(sys);
      g_session.current_name = g_session.current_system->name.empty()
                                   ? "ETAP workbook"
                                   : g_session.current_system->name;
      clear_cached_analysis(g_session);
      auto summary = system_summary(*g_session.current_system);
      summary["_raw_json"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      summary["_etap_warnings"] = rep.warnings;
      res.set_content(summary.dump(), "application/json");
    } catch (const std::exception& e) {
      if (!tmp.empty()) { std::error_code ec; std::filesystem::remove(tmp, ec); }
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Session: update components from Case Builder ----
  svr.Post("/api/session/update_components",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      std::lock_guard<std::mutex> lk(g_session.mu);
      if (!g_session.current_system) throw std::runtime_error("No system loaded");

      json root = json::parse(hacdcpf::io::to_json(*g_session.current_system, 2));

      auto put_array = [&](const std::string& top,
                           const std::string& key,
                           const std::string& src) {
        if (j.contains(src) && j[src].is_array()) root[top][key] = j[src];
      };
      auto put_root_array = [&](const std::string& key) {
        if (j.contains(key) && j[key].is_array()) root[key] = j[key];
      };

      if (j.contains("name") && j["name"].is_string()) root["name"] = j["name"];
      if (j.contains("base_mva") && j["base_mva"].is_number()) root["base_mva"] = j["base_mva"];

      put_array("ac", "buses", "ac_buses");
      put_array("ac", "branches", "ac_branches");
      put_array("ac", "generators", "generators");
      put_array("ac", "static_generators", "static_generators");
      put_array("ac", "loads", "loads");
      put_array("ac", "flexible_loads", "flexible_loads");
      put_array("ac", "asymmetric_loads", "asymmetric_loads");
      put_array("ac", "shunts", "shunts");
      put_array("ac", "storage", "storage");
      put_array("ac", "renewable_gens", "renewable_gens");
      put_array("ac", "pv_systems", "pv_systems");
      put_array("ac", "external_grids", "external_grids");
      put_array("ac", "transformers_2w", "transformers_2w");
      put_array("ac", "transformers_3w", "transformers_3w");
      put_array("ac", "switches", "switches");
      put_array("ac", "circuit_breakers", "circuit_breakers");
      put_array("ac", "charging_stations", "charging_stations");
      put_array("ac", "chargers", "chargers");
      put_array("ac", "motors", "motors");

      put_array("dc", "buses", "dc_buses");
      put_array("dc", "branches", "dc_branches");
      put_array("dc", "loads", "dc_loads");
      if (j.contains("dc_storage") && j["dc_storage"].is_array()) {
        root["dc"]["dc_storage"] = j["dc_storage"];
        root["dc"]["storage"] = json::array();
      }
      put_array("dc", "static_generators", "dc_static_generators");
      put_array("dc", "dc_static_generators", "dc_native_static_generators");
      put_array("dc", "pv_arrays", "pv_arrays");
      put_array("dc", "dc_circuit_breakers", "dc_circuit_breakers");

      put_root_array("vsc_converters");
      put_root_array("dcdc_converters");
      put_root_array("energy_routers");
      put_root_array("mobile_storage");
      put_root_array("vpps");
      put_root_array("microgrids");

      const bool has_tp = j.contains("tp_buses") || j.contains("tp_lines") ||
                          j.contains("tp_transformers") || j.contains("tp_loads") ||
                          j.contains("tp_generators") || j.contains("tp_external_grids");
      if (has_tp) {
        if (!root.contains("three_phase_ac") || !root["three_phase_ac"].is_object()) {
          root["three_phase_ac"] = json::object();
        }
        if (j.contains("tp_buses") && j["tp_buses"].is_array()) root["three_phase_ac"]["buses"] = j["tp_buses"];
        if (j.contains("tp_lines") && j["tp_lines"].is_array()) root["three_phase_ac"]["lines"] = j["tp_lines"];
        if (j.contains("tp_transformers") && j["tp_transformers"].is_array()) root["three_phase_ac"]["transformers"] = j["tp_transformers"];
        if (j.contains("tp_loads") && j["tp_loads"].is_array()) root["three_phase_ac"]["loads"] = j["tp_loads"];
        if (j.contains("tp_generators") && j["tp_generators"].is_array()) root["three_phase_ac"]["generators"] = j["tp_generators"];
        if (j.contains("tp_external_grids") && j["tp_external_grids"].is_array()) root["three_phase_ac"]["external_grids"] = j["tp_external_grids"];
      }

      g_session.current_system = hacdcpf::io::from_json(root.dump());
      g_session.current_name = g_session.current_system->name;
      clear_cached_analysis(g_session);
      auto summary = system_summary(*g_session.current_system);
      summary["_raw_json"] = hacdcpf::io::to_json(*g_session.current_system, 2);
      res.set_content(summary.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/update_carbon_factors",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      std::lock_guard<std::mutex> lk(g_session.mu);
      if (!g_session.current_system) throw std::runtime_error("No system loaded");
      auto& sys = *g_session.current_system;

      auto matches = [](const json& row, int index, int bus, const std::string& name) {
        if (row.contains("index") && row["index"].is_number_integer() &&
            row["index"].get<int>() == index)
          return true;
        if (row.contains("name") && row["name"].is_string() &&
            !name.empty() && row["name"].get<std::string>() == name)
          return true;
        if (row.contains("bus") && row["bus"].is_number_integer() &&
            row["bus"].get<int>() == bus)
          return true;
        return false;
      };

      int gen_updated = 0, sgen_updated = 0, grid_updated = 0, grid_profiles_updated = 0;
      if (j.contains("generators") && j["generators"].is_array()) {
        for (const auto& row : j["generators"]) {
          if (!row.is_object()) continue;
          const double scale = carbon_factor_scale_from_json(j, row);
          const double ef = json_emission_factor_value(row, scale);
          for (auto& g : sys.ac.generators) {
            if (!matches(row, g.index, g.bus, g.name)) continue;
            g.emission_factor_tco2_mwh = ef;
            ++gen_updated;
          }
        }
      }
      if (j.contains("static_generators") && j["static_generators"].is_array()) {
        for (const auto& row : j["static_generators"]) {
          if (!row.is_object()) continue;
          const double scale = carbon_factor_scale_from_json(j, row);
          const double ef = json_emission_factor_value(row, scale);
          for (auto& sg : sys.ac.static_generators) {
            if (!matches(row, sg.index, sg.bus, sg.name)) continue;
            sg.co2_emission_rate = ef;
            ++sgen_updated;
          }
        }
      }
      if (j.contains("external_grids") && j["external_grids"].is_array()) {
        for (const auto& row : j["external_grids"]) {
          if (!row.is_object()) continue;
          const double scale = carbon_factor_scale_from_json(j, row);
          const double ef = json_emission_factor_value(row, scale);
          for (auto& eg : sys.ac.external_grids) {
            if (!matches(row, eg.index, eg.bus, eg.name)) continue;
            eg.emission_factor_tco2_mwh = ef;
            auto it = std::find_if(
                g_session.external_grid_carbon_profiles.begin(),
                g_session.external_grid_carbon_profiles.end(),
                [&](const Session::ExternalGridCarbonProfile& item) {
                  return item.index == eg.index && item.bus == eg.bus &&
                         item.name == eg.name;
                });
            if (const auto profile = json_emission_factor_profile(row, scale)) {
              if (it == g_session.external_grid_carbon_profiles.end()) {
                Session::ExternalGridCarbonProfile item;
                item.index = eg.index;
                item.bus = eg.bus;
                item.name = eg.name;
                item.values_tco2_mwh = *profile;
                g_session.external_grid_carbon_profiles.push_back(std::move(item));
              } else {
                it->values_tco2_mwh = *profile;
              }
              ++grid_profiles_updated;
            } else if (it != g_session.external_grid_carbon_profiles.end()) {
              g_session.external_grid_carbon_profiles.erase(it);
            }
            ++grid_updated;
          }
        }
      }

      json out;
      out["updated_generators"] = gen_updated;
      out["updated_static_generators"] = sgen_updated;
      out["updated_external_grids"] = grid_updated;
      out["updated_external_grid_profiles"] = grid_profiles_updated;
      out["total_updated"] = gen_updated + sgen_updated + grid_updated;
      res.set_content(out.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Session: cancel / status ----
  svr.Post("/api/session/cancel",
           [](const httplib::Request&, httplib::Response& res) {
    g_session.cancel.store(true);
    res.set_content(json{{"cancelled", true}}.dump(), "application/json");
  });

  svr.Get("/api/session/status",
          [](const httplib::Request&, httplib::Response& res) {
    json out;
    out["busy"] = g_session.busy.load();
    out["cancel"] = g_session.cancel.load();
    res.set_content(out.dump(), "application/json");
  });

  // ---- Session: run analyses ----
  svr.Post("/api/session/pf",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      // Copy system under short lock, then release
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);

      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      const std::string method = j.value("method", std::string("ac_newton"));

      hacdcpf::PowerFlowOptions opt;
      if (j.contains("options")) {
        const auto& o = j["options"];
        if (o.contains("max_iter")) opt.max_iter = o["max_iter"].get<int>();
        if (o.contains("tol")) opt.tol = o["tol"].get<double>();
        if (o.contains("fdpf_max_iter")) opt.fdpf_max_iter = o["fdpf_max_iter"].get<int>();
        if (o.contains("enable_pv_pq_conversion")) opt.enable_pv_pq_conversion = o["enable_pv_pq_conversion"].get<bool>();
        if (o.contains("enable_auto_swing_selection")) opt.enable_auto_swing_selection = o["enable_auto_swing_selection"].get<bool>();
        if (o.contains("enable_converter_mode_switching")) opt.enable_converter_mode_switching = o["enable_converter_mode_switching"].get<bool>();
        if (o.contains("enable_converter_coordination_check")) opt.enable_converter_coordination_check = o["enable_converter_coordination_check"].get<bool>();
        if (o.contains("verbose")) opt.verbose = o["verbose"].get<bool>();
        if (o.contains("pv_q_hysteresis_pu")) opt.pv_q_hysteresis_pu = o["pv_q_hysteresis_pu"].get<double>();
        if (o.contains("pv_recover_vm_tol_pu")) opt.pv_recover_vm_tol_pu = o["pv_recover_vm_tol_pu"].get<double>();
        if (o.contains("max_delta_va_rad")) opt.max_delta_va_rad = o["max_delta_va_rad"].get<double>();
        if (o.contains("max_delta_vm_pu")) opt.max_delta_vm_pu = o["max_delta_vm_pu"].get<double>();
        if (o.contains("max_delta_vdc_pu")) opt.max_delta_vdc_pu = o["max_delta_vdc_pu"].get<double>();
        if (o.contains("loss_model")) {
          const std::string lm = o["loss_model"].get<std::string>();
          if (lm == "current_based") opt.loss_model = hacdcpf::LossModelType::CurrentBased;
          else opt.loss_model = hacdcpf::LossModelType::Linear;
        }
      }
      // Honor the request's coordination-check flag (the GUI checkbox). Default
      // to enabled for hybrid AC/DC safety when the request omits it.
      if (!j.contains("options") || !j["options"].contains("enable_converter_coordination_check")) {
        opt.enable_converter_coordination_check = true;
      }
      // The GUI schema exposes ac_grid_forming as an explicit device role.  The
      // Newton solver consumes that role through AC_GRID_FORMING control mode, so
      // translate it on this request-local copy before solving.  This keeps
      // user-authored JSON fields and solver semantics aligned without mutating
      // the stored session system.
	      for (auto& conv : sys.vsc_converters) {
	        if (!conv.in_service || !conv.ac_grid_forming) continue;
	        conv.control_mode = hacdcpf::ConverterMode::AC_GRID_FORMING;
	        conv.p_is_hard_constraint = false;
	      }
      json out;
      out["method"] = method;
      out["vm"] = json::array();
      out["va"] = json::array();
      out["vdc"] = json::array();
      out["branch_abs"] = json::array();
      out["dc_branch_flows"] = json::array();
      out["vsc_transfers"] = json::array();
      out["dcdc_transfers"] = json::array();
      out["ac_switch_flows"] = json::array();
      out["ac_circuit_breaker_flows"] = json::array();
      out["dc_circuit_breaker_flows"] = json::array();
      out["notes"] = "";

      // Snapshot original energy routers BEFORE canonical projection clears them.
      // After solve_power_flow → project_to_canonical_models → expand_energy_routers,
      // sys.energy_routers is empty.  We reconstruct ER port flows from the
      // expanded VSC/DCDC transfers by matching the canonical expansion names
      // back to solved transfer indices.
      const auto er_snapshot = sys.energy_routers;
      // Helper to serialize VSC/DCDC transfers from a PowerFlowResult
      auto add_transfers = [&](const hacdcpf::PowerFlowResult& pf) {
        for (const auto& v : pf.vsc_transfers)
          out["vsc_transfers"].push_back(json{{"index",v.index},{"bus_ac",v.bus_ac},{"bus_dc",v.bus_dc},
            {"p_ac_mw",v.p_ac_mw},{"q_ac_mvar",v.q_ac_mvar},{"p_dc_mw",v.p_dc_mw},{"loss_mw",v.loss_mw}});
        for (const auto& d : pf.dcdc_transfers)
          out["dcdc_transfers"].push_back(json{{"index",d.index},{"bus_in",d.bus_in},{"bus_out",d.bus_out},
            {"p_in_mw",d.p_in_mw},{"p_out_mw",d.p_out_mw},{"loss_mw",d.loss_mw}});
      };

      auto add_converter_coordination = [&](const hacdcpf::PowerFlowResult& pf) {
        const auto& rep = pf.diagnostics.converter_coordination;
        json coord;
        coord["enabled"] = rep.enabled;
        coord["feasible"] = rep.feasible;
        coord["blocking_count"] = rep.blocking_count();
        coord["fatal_count"] = rep.fatal_count();
        coord["error_count"] = rep.error_count();
        coord["warning_count"] = rep.warning_count();
        coord["issues"] = json::array();
        for (const auto& issue : rep.issues) {
          coord["issues"].push_back(json{
            {"severity", hacdcpf::powerflow::coordination_severity_str(issue.severity)},
            {"rule_id", issue.rule_id},
            {"component_type", issue.component_type},
            {"component_index", issue.component_index},
            {"island_index", issue.island_index},
            {"message", issue.message}
          });
        }
        coord["dc_islands"] = json::array();
        for (const auto& isle : rep.dc_islands) {
          coord["dc_islands"].push_back(json{
            {"island_index", isle.island_index},
            {"dc_buses", isle.dc_buses},
            {"declared_v_buses", isle.declared_v_buses},
            {"hard_vdc_sources", isle.hard_vdc_sources},
            {"droop_sources", isle.droop_sources},
            {"fixed_power_devices", isle.fixed_power_devices},
            {"fixed_power_mw", isle.fixed_power_mw},
            {"flexible_up_mw", isle.flexible_up_mw},
            {"flexible_down_mw", isle.flexible_down_mw}
          });
          auto& island_json = coord["dc_islands"].back();
          island_json["voltage_sources"] = json::array();
          for (const auto& source : isle.voltage_sources) {
            island_json["voltage_sources"].push_back(json{
              {"component_type", source.component_type},
              {"component_index", source.component_index},
              {"bus", source.bus},
              {"v_set_pu", source.v_set_pu},
              {"has_v_set", source.has_v_set},
              {"droop", source.droop}
            });
          }
        }
        out["converter_coordination"] = coord;
      };

      auto add_converter_coordination_report =
          [&](const hacdcpf::powerflow::ConverterCoordinationReport& rep) {
        json coord;
        coord["enabled"] = rep.enabled;
        coord["feasible"] = rep.feasible;
        coord["blocking_count"] = rep.blocking_count();
        coord["fatal_count"] = rep.fatal_count();
        coord["error_count"] = rep.error_count();
        coord["warning_count"] = rep.warning_count();
        coord["issues"] = json::array();
        for (const auto& issue : rep.issues) {
          coord["issues"].push_back(json{
            {"severity", hacdcpf::powerflow::coordination_severity_str(issue.severity)},
            {"rule_id", issue.rule_id},
            {"component_type", issue.component_type},
            {"component_index", issue.component_index},
            {"island_index", issue.island_index},
            {"message", issue.message}
          });
        }
        coord["dc_islands"] = json::array();
        for (const auto& isle : rep.dc_islands) {
          coord["dc_islands"].push_back(json{
            {"island_index", isle.island_index},
            {"dc_buses", isle.dc_buses},
            {"declared_v_buses", isle.declared_v_buses},
            {"hard_vdc_sources", isle.hard_vdc_sources},
            {"droop_sources", isle.droop_sources},
            {"fixed_power_devices", isle.fixed_power_devices},
            {"fixed_power_mw", isle.fixed_power_mw},
            {"flexible_up_mw", isle.flexible_up_mw},
            {"flexible_down_mw", isle.flexible_down_mw}
          });
          auto& island_json = coord["dc_islands"].back();
          island_json["voltage_sources"] = json::array();
          for (const auto& source : isle.voltage_sources) {
            island_json["voltage_sources"].push_back(json{
              {"component_type", source.component_type},
              {"component_index", source.component_index},
              {"bus", source.bus},
              {"v_set_pu", source.v_set_pu},
              {"has_v_set", source.has_v_set},
              {"droop", source.droop}
            });
          }
        }
        out["converter_coordination"] = coord;
      };

      auto store_last_pf = [&](const hacdcpf::PowerFlowResult* pf) {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (pf != nullptr) {
          g_session.last_pf_result = *pf;
          g_session.last_pf_method = method;
        } else {
          g_session.last_pf_result.reset();
          g_session.last_pf_method.clear();
        }
      };
      store_last_pf(nullptr);

      auto add_solver_diagnostics = [&](const hacdcpf::SolverDiagnostics& diag) {
        json warns = json::array();
        for (const auto& w : diag.warnings) warns.push_back(w);
        out["warnings"] = warns;
        out["termination_reason"] = diag.termination_reason;
        out["promoted_vsc_indices"] = diag.promoted_vsc_indices;
      };

      // Helper to compute DC branch flows from DC bus voltages
      auto add_dc_branch_flows = [&]() {
        if (out.contains("geo_dc_branches") && out["geo_dc_branches"].is_array() &&
            !out["geo_dc_branches"].empty()) {
          json dc_br = json::array();
          for (const auto& br : out["geo_dc_branches"]) {
            dc_br.push_back(json{
              {"index", br.value("index", 0)},
              {"from_bus", br.value("from", br.value("from_bus", 0))},
              {"to_bus", br.value("to", br.value("to_bus", 0))},
              {"pf_mw", br.value("pf_mw", 0.0)},
              {"pt_mw", br.value("pt_mw", 0.0)},
              {"loss_mw", br.value("loss_mw", br.value("pf_mw", 0.0) + br.value("pt_mw", 0.0))},
              {"loading_pct", br.value("loading_pct", 0.0)},
              {"rate_mva", br.value("rate_mva", 0.0)}
            });
          }
          out["dc_branch_flows"] = dc_br;
          return;
        }
        if (!out.contains("vdc") || !out["vdc"].is_array() || out["vdc"].empty() || sys.dc.branches.empty()) return;
        // Build dc_bus_index → position map
        std::unordered_map<int, size_t> dc_idx;
        for (size_t i = 0; i < sys.dc.buses.size(); ++i) dc_idx[sys.dc.buses[i].index] = i;
        const auto& vdc_arr = out["vdc"];
        json dc_br = json::array();
        for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
          const auto& br = sys.dc.branches[i];
          if (!br.in_service) { dc_br.push_back(json{{"from_bus", br.from_bus}, {"to_bus", br.to_bus}, {"pf_mw", 0.0}, {"pt_mw", 0.0}}); continue; }
          auto fi = dc_idx.find(br.from_bus), ti = dc_idx.find(br.to_bus);
          double pf_mw = 0.0, pt_mw = 0.0;
          if (fi != dc_idx.end() && ti != dc_idx.end() && fi->second < vdc_arr.size() && ti->second < vdc_arr.size()) {
            double vf = vdc_arr[fi->second].get<double>(), vt = vdc_arr[ti->second].get<double>();
            double r_pu = br.r_pu;
            if (r_pu > 1e-12) {
              double i_pu = (vf - vt) / r_pu;
              pf_mw = vf * i_pu * sys.base_mva;
              pt_mw = -vt * i_pu * sys.base_mva;
            }
          }
          dc_br.push_back(json{{"from_bus", br.from_bus}, {"to_bus", br.to_bus}, {"pf_mw", pf_mw}, {"pt_mw", pt_mw}});
        }
        out["dc_branch_flows"] = dc_br;
      };

      // Helper to add GIS/geographic data for map visualization
      auto add_geo_data = [&](const hacdcpf::PowerFlowResult& pf) {
        json geo_buses = json::array();
        json geo_ac_branches = json::array();
        json geo_dc_branches = json::array();
        json geo_vsc = json::array();
        json geo_dcdc = json::array();

        // Aggregate load demand and generation per bus for display.
        std::unordered_map<int, double> bus_pd, bus_qd, bus_pg, bus_qg;
        for (const auto& ld : sys.ac.loads) {
          if (!ld.in_service) continue;
          bus_pd[ld.bus] += ld.p_mw * ld.scaling;
          bus_qd[ld.bus] += ld.q_mvar * ld.scaling;
        }
        for (const auto& g : sys.ac.generators) {
          if (!g.in_service) continue;
          bus_pg[g.bus] += g.pg_mw;
          bus_qg[g.bus] += g.qg_mvar;
        }

        struct ThreeWindingTerminalFlow {
          int index{0};
          int hv_bus{0};
          int mv_bus{0};
          int lv_bus{0};
          double p_hv_mw{0.0};
          double q_hv_mvar{0.0};
          double p_mv_mw{0.0};
          double q_mv_mvar{0.0};
          double p_lv_mw{0.0};
          double q_lv_mvar{0.0};
          double loss_mw{0.0};
          double loading_pct{0.0};
          double rate_mva{0.0};
          bool valid{false};
        };
        std::vector<ThreeWindingTerminalFlow> trafo3w_flows(sys.ac.transformers_3w.size());
        std::vector<bool> trafo3w_supplied_by_pf(sys.ac.transformers_3w.size(), false);
        std::unordered_map<int, size_t> trafo3w_pos_by_index;
        for (size_t ti = 0; ti < sys.ac.transformers_3w.size(); ++ti) {
          const auto& tr = sys.ac.transformers_3w[ti];
          trafo3w_pos_by_index[tr.index] = ti;
          auto& tf = trafo3w_flows[ti];
          tf.index = tr.index;
          tf.hv_bus = tr.hv_bus;
          tf.mv_bus = tr.mv_bus;
          tf.lv_bus = tr.lv_bus;
          tf.rate_mva = std::max({tr.sn_hv_mva, tr.sn_mv_mva, tr.sn_lv_mva});
        }
        for (const auto& src : pf.trafo3w_flows) {
          auto it = trafo3w_pos_by_index.find(src.index);
          if (it == trafo3w_pos_by_index.end()) continue;
          auto& tf = trafo3w_flows[it->second];
          tf.index = src.index;
          tf.hv_bus = src.hv_bus;
          tf.mv_bus = src.mv_bus;
          tf.lv_bus = src.lv_bus;
          tf.p_hv_mw = src.p_hv_mw;
          tf.q_hv_mvar = src.q_hv_mvar;
          tf.p_mv_mw = src.p_mv_mw;
          tf.q_mv_mvar = src.q_mv_mvar;
          tf.p_lv_mw = src.p_lv_mw;
          tf.q_lv_mvar = src.q_lv_mvar;
          tf.loss_mw = src.loss_mw;
          tf.loading_pct = src.loading_pct;
          tf.rate_mva = src.rate_mva;
          tf.valid = true;
          trafo3w_supplied_by_pf[it->second] = true;
        }
        if (!sys.ac.transformers_3w.empty()) {
          try {
            const hacdcpf::HybridPowerSystem projected =
                hacdcpf::project_to_canonical_models(sys);
            if (projected.branch_expand_map.has_value()) {
              std::unordered_map<int, size_t> flow_pos_by_branch_index;
              for (size_t bi = 0; bi < sys.ac.branches.size(); ++bi) {
                flow_pos_by_branch_index[sys.ac.branches[bi].index] = bi;
              }
              size_t next_expanded_pos = sys.ac.branches.size();
              for (const auto& entry : projected.branch_expand_map->entries) {
                if (!flow_pos_by_branch_index.count(entry.branch_index)) {
                  flow_pos_by_branch_index[entry.branch_index] =
                      next_expanded_pos++;
                }
              }
              for (const auto& entry : projected.branch_expand_map->entries) {
                if (entry.origin_type != hacdcpf::BranchOriginType::Transformer3W) {
                  continue;
                }
                auto tit = trafo3w_pos_by_index.find(entry.origin_index);
                if (tit == trafo3w_pos_by_index.end()) continue;
                auto fit = flow_pos_by_branch_index.find(entry.branch_index);
                if (fit == flow_pos_by_branch_index.end() ||
                    fit->second >= pf.branch_flows.size()) {
                  continue;
                }
                auto& tf = trafo3w_flows[tit->second];
                if (trafo3w_supplied_by_pf[tit->second]) continue;
                const auto& br_flow = pf.branch_flows[fit->second];
                const auto& tr = sys.ac.transformers_3w[tit->second];
                double pair_rate = 0.0;
                if (entry.pair_number == 0) {
                  tf.p_hv_mw += br_flow.pf_mw;
                  tf.q_hv_mvar += br_flow.qf_mvar;
                  tf.p_mv_mw += br_flow.pt_mw;
                  tf.q_mv_mvar += br_flow.qt_mvar;
                  pair_rate = std::min(tr.sn_hv_mva, tr.sn_mv_mva);
                } else if (entry.pair_number == 1) {
                  tf.p_hv_mw += br_flow.pf_mw;
                  tf.q_hv_mvar += br_flow.qf_mvar;
                  tf.p_lv_mw += br_flow.pt_mw;
                  tf.q_lv_mvar += br_flow.qt_mvar;
                  pair_rate = std::min(tr.sn_hv_mva, tr.sn_lv_mva);
                } else if (entry.pair_number == 2) {
                  tf.p_mv_mw += br_flow.pf_mw;
                  tf.q_mv_mvar += br_flow.qf_mvar;
                  tf.p_lv_mw += br_flow.pt_mw;
                  tf.q_lv_mvar += br_flow.qt_mvar;
                  pair_rate = std::min(tr.sn_mv_mva, tr.sn_lv_mva);
                }
                tf.valid = true;
                const double pair_s = std::max(
                    std::hypot(br_flow.pf_mw, br_flow.qf_mvar),
                    std::hypot(br_flow.pt_mw, br_flow.qt_mvar));
                if (pair_rate > 1e-9) {
                  tf.loading_pct =
                      std::max(tf.loading_pct, 100.0 * pair_s / pair_rate);
                }
              }
              for (auto& tf : trafo3w_flows) {
                if (!tf.valid) continue;
                tf.loss_mw = tf.p_hv_mw + tf.p_mv_mw + tf.p_lv_mw;
              }
            }
          } catch (const std::exception&) {
            // Result attribution is best-effort; PF convergence/result payload
            // should not fail if projection diagnostics cannot be reconstructed.
          }
        }

        // ── Compute solved per-generator P/Q from bus power balance ──
        // For non-slack generators, scheduled pg_mw is enforced by the solver.
        // For slack/PV generators, we back-calculate from branch flows.
        std::unordered_map<int, double> bus_p_out, bus_q_out;
        for (size_t i = 0; i < sys.ac.branches.size() && i < pf.branch_flows.size(); ++i) {
          const auto& br = sys.ac.branches[i];
          bus_p_out[br.from_bus] += pf.branch_flows[i].pf_mw;
          bus_p_out[br.to_bus]   += pf.branch_flows[i].pt_mw;
          bus_q_out[br.from_bus] += pf.branch_flows[i].qf_mvar;
          bus_q_out[br.to_bus]   += pf.branch_flows[i].qt_mvar;
        }
        for (const auto& tf : trafo3w_flows) {
          if (!tf.valid) continue;
          bus_p_out[tf.hv_bus] += tf.p_hv_mw;
          bus_p_out[tf.mv_bus] += tf.p_mv_mw;
          bus_p_out[tf.lv_bus] += tf.p_lv_mw;
          bus_q_out[tf.hv_bus] += tf.q_hv_mvar;
          bus_q_out[tf.mv_bus] += tf.q_mv_mvar;
          bus_q_out[tf.lv_bus] += tf.q_lv_mvar;
        }
        for (const auto& v : pf.vsc_transfers) {
          bus_p_out[v.bus_ac] -= v.p_ac_mw;
          bus_q_out[v.bus_ac] -= v.q_ac_mvar;
        }

        struct TwoWindingTerminalFlow {
          double p_hv_mw{0.0};
          double q_hv_mvar{0.0};
          double p_lv_mw{0.0};
          double q_lv_mvar{0.0};
          double loss_mw{0.0};
          bool valid{false};
        };
        std::vector<TwoWindingTerminalFlow> trafo2w_flows(sys.ac.transformers_2w.size());
        std::unordered_map<int, double> ac_tf_vm_by_bus, ac_tf_va_by_bus;
        for (size_t bi = 0; bi < sys.ac.buses.size(); ++bi) {
          const auto& bus = sys.ac.buses[bi];
          ac_tf_vm_by_bus[bus.index] = (bi < pf.vm.size()) ? pf.vm[bi] : bus.vm_pu;
          ac_tf_va_by_bus[bus.index] =
              (bi < pf.va.size()) ? pf.va[bi] : bus.va_deg * M_PI / 180.0;
        }
        auto ac_voltage = [&](int bus) {
          const double vm = ac_tf_vm_by_bus.count(bus) ? ac_tf_vm_by_bus[bus] : 1.0;
          const double va = ac_tf_va_by_bus.count(bus) ? ac_tf_va_by_bus[bus] : 0.0;
          return std::polar(vm, va);
        };
        for (size_t ti = 0; ti < sys.ac.transformers_2w.size(); ++ti) {
          const auto& tr = sys.ac.transformers_2w[ti];
          if (!tr.in_service || tr.source_branch_idx > 0 ||
              tr.sn_mva <= 1e-9 || sys.base_mva <= 1e-9) {
            continue;
          }
          const auto vh = ac_voltage(tr.hv_bus);
          const auto vl = ac_voltage(tr.lv_bus);
          const double scale = sys.base_mva / tr.sn_mva;
          const double z_mag = std::max(0.0, tr.vk_percent / 100.0) * scale;
          double r_pu = std::max(0.0, tr.vkr_percent / 100.0) * scale;
          double x_pu = std::sqrt(std::max(0.0, z_mag * z_mag - r_pu * r_pu));
          if (r_pu == 0.0 && x_pu == 0.0) x_pu = 1e-4;
          const double raw_tap = std::max(
              1e-6, 1.0 + (static_cast<double>(tr.tap_pos) -
                            static_cast<double>(tr.tap_neutral)) *
                               tr.tap_step_percent / 100.0);
          if (tr.tap_side == 1) {
            r_pu *= raw_tap * raw_tap;
            x_pu *= raw_tap * raw_tap;
          }
          const std::complex<double> ys = 1.0 / std::complex<double>(r_pu, x_pu);
          const double tap_mag = tr.tap_side == 1 ? 1.0 / raw_tap : raw_tap;
          const double shift = tr.shift_deg * M_PI / 180.0;
          const std::complex<double> tap = std::polar(tap_mag, shift);
          const double tap_abs2 = std::norm(tap);
          if (tap_abs2 <= 0.0) continue;
          const std::complex<double> yff = ys / tap_abs2;
          const std::complex<double> yft = -ys / std::conj(tap);
          const std::complex<double> ytf = -ys / tap;
          const std::complex<double> ih = yff * vh + yft * vl;
          const std::complex<double> il = ytf * vh + ys * vl;
          const auto sh = vh * std::conj(ih) * sys.base_mva;
          const auto sl = vl * std::conj(il) * sys.base_mva;
          auto& tf = trafo2w_flows[ti];
          tf.p_hv_mw = sh.real();
          tf.q_hv_mvar = sh.imag();
          tf.p_lv_mw = sl.real();
          tf.q_lv_mvar = sl.imag();
          tf.loss_mw = tf.p_hv_mw + tf.p_lv_mw;
          tf.valid = true;
          bus_p_out[tr.hv_bus] += tf.p_hv_mw;
          bus_p_out[tr.lv_bus] += tf.p_lv_mw;
          bus_q_out[tr.hv_bus] += tf.q_hv_mvar;
          bus_q_out[tr.lv_bus] += tf.q_lv_mvar;
        }

        std::vector<double> gen_pg_solved(sys.ac.generators.size());
        std::vector<double> gen_qg_solved(sys.ac.generators.size());
        for (size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
          gen_pg_solved[gi] = sys.ac.generators[gi].pg_mw;
          gen_qg_solved[gi] = sys.ac.generators[gi].qg_mvar;
        }
        std::unordered_map<int, std::vector<size_t>> gens_at_bus;
        for (size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
          if (!sys.ac.generators[gi].in_service) continue;
          gens_at_bus[sys.ac.generators[gi].bus].push_back(gi);
        }
        for (const auto& b : sys.ac.buses) {
          auto git = gens_at_bus.find(b.index);
          if (git == gens_at_bus.end()) continue;
          const auto& gen_list = git->second;
          double total_pd = b.pd_mw + (bus_pd.count(b.index) ? bus_pd[b.index] : 0.0);
          double total_qd = b.qd_mvar + (bus_qd.count(b.index) ? bus_qd[b.index] : 0.0);
          double total_pg_solved = bus_p_out[b.index] + total_pd;
          double total_qg_solved = bus_q_out[b.index] + total_qd;
          if (b.bus_type == hacdcpf::BusType::SLACK) {
            double non_slack_pg = 0.0, non_slack_qg = 0.0;
            int slack_gi = -1;
            for (size_t gi : gen_list) {
              if (sys.ac.generators[gi].is_slack) {
                slack_gi = static_cast<int>(gi);
              } else {
                non_slack_pg += sys.ac.generators[gi].pg_mw;
                non_slack_qg += sys.ac.generators[gi].qg_mvar;
              }
            }
            if (slack_gi >= 0) {
              // One generator is the designated slack unit: it absorbs the bus
              // balance, the rest stay at their scheduled setpoints.
              gen_pg_solved[slack_gi] = total_pg_solved - non_slack_pg;
              gen_qg_solved[slack_gi] = total_qg_solved - non_slack_qg;
            } else if (!gen_list.empty()) {
              // No generator flagged is_slack: distribute the full bus generation
              // across all units on the bus, by rated capacity (even split if the
              // rating sum is ~0). Without this, every unit shows its 0 setpoint.
              double pmax_sum = 0.0, qrange_sum = 0.0;
              for (size_t gi : gen_list) {
                pmax_sum += std::max(sys.ac.generators[gi].pmax_mw, 0.0);
                qrange_sum += std::max(sys.ac.generators[gi].qmax_mvar -
                                       sys.ac.generators[gi].qmin_mvar, 0.0);
              }
              const double n = static_cast<double>(gen_list.size());
              for (size_t gi : gen_list) {
                const double p_w = pmax_sum > 1e-9
                    ? std::max(sys.ac.generators[gi].pmax_mw, 0.0) / pmax_sum
                    : 1.0 / n;
                const double q_w = qrange_sum > 1e-9
                    ? std::max(sys.ac.generators[gi].qmax_mvar -
                               sys.ac.generators[gi].qmin_mvar, 0.0) / qrange_sum
                    : 1.0 / n;
                gen_pg_solved[gi] = total_pg_solved * p_w;
                gen_qg_solved[gi] = total_qg_solved * q_w;
              }
            }
          } else if (b.bus_type == hacdcpf::BusType::PV) {
            if (gen_list.size() == 1) {
              gen_qg_solved[gen_list[0]] = total_qg_solved;
            } else {
              double q_range_sum = 0.0;
              for (size_t gi : gen_list)
                q_range_sum += std::max(sys.ac.generators[gi].qmax_mvar -
                                        sys.ac.generators[gi].qmin_mvar, 0.01);
              for (size_t gi : gen_list) {
                double range = std::max(sys.ac.generators[gi].qmax_mvar -
                                        sys.ac.generators[gi].qmin_mvar, 0.01);
                gen_qg_solved[gi] = total_qg_solved * range / q_range_sum;
              }
            }
          }
        }

        // Rebuild bus_pg/bus_qg with solved values for geo_buses display
        bus_pg.clear(); bus_qg.clear();
        for (size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
          if (!sys.ac.generators[gi].in_service) continue;
          bus_pg[sys.ac.generators[gi].bus] += gen_pg_solved[gi];
          bus_qg[sys.ac.generators[gi].bus] += gen_qg_solved[gi];
        }
        for (const auto& sg : sys.ac.static_generators) {
          if (!sg.in_service) continue;
          bus_pg[sg.bus] += sg.p_mw * sg.scaling;
          bus_qg[sg.bus] += sg.q_mvar * sg.scaling;
        }

        // Build AC bus coordinate lookup and geo_buses array
        std::map<int, std::pair<double, double>> ac_bus_coords;
        for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
          const auto& b = sys.ac.buses[i];
          ac_bus_coords[b.index] = {b.latitude, b.longitude};
          double vm_val = (i < pf.vm.size()) ? pf.vm[i] : b.vm_pu;
          double va_val = (i < pf.va.size()) ? pf.va[i] : b.va_deg * M_PI / 180.0;
          // A dead-island bus has Vm ≈ 0 — zero out all injections.
          const bool is_dead = (vm_val < 1e-6);
          double pd = b.pd_mw + (bus_pd.count(b.index) ? bus_pd[b.index] : 0.0);
          double qd = b.qd_mvar + (bus_qd.count(b.index) ? bus_qd[b.index] : 0.0);
          double pg = (bus_pg.count(b.index) ? bus_pg[b.index] : 0.0);
          double qg = (bus_qg.count(b.index) ? bus_qg[b.index] : 0.0);
          if (is_dead) { pd = 0.0; qd = 0.0; pg = 0.0; qg = 0.0; }
          std::string bus_type_str = "PQ";
          switch(b.bus_type) {
            case hacdcpf::BusType::PV: bus_type_str = "PV"; break;
            case hacdcpf::BusType::SLACK: bus_type_str = "SLACK"; break;
            case hacdcpf::BusType::ISOLATED: bus_type_str = "ISOLATED"; break;
            default: break;
          }
          for (const auto& conv : sys.vsc_converters) {
            if (conv.in_service && conv.bus_ac == b.index &&
                conv.control_mode == hacdcpf::ConverterMode::AC_GRID_FORMING) {
              bus_type_str = "SLACK";
              break;
            }
          }
          geo_buses.push_back(json{
            {"id", b.index}, {"name", b.name.empty() ? ("Bus" + std::to_string(b.index)) : b.name},
            {"type", "AC"}, {"bus_type", is_dead ? "DEAD" : bus_type_str},
            {"lat", b.latitude}, {"lon", b.longitude},
            {"base_kv", b.base_kv}, {"area", b.area}, {"zone", b.zone},
            {"vm_pu", vm_val}, {"va_rad", va_val},
            {"pd_mw", pd}, {"qd_mvar", qd}, {"pg_mw", pg}, {"qg_mvar", qg}
          });
        }

        // Build DC bus coordinate lookup and add to geo_buses
        std::map<int, std::pair<double, double>> dc_bus_coords;
        for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
          const auto& b = sys.dc.buses[i];
          dc_bus_coords[b.index] = {b.latitude, b.longitude};
          double vdc_val = (i < pf.vdc.size()) ? pf.vdc[i] : b.vm_pu;
          std::string bus_type_str = (b.bus_type == hacdcpf::DCBusType::DC_V) ? "DC_V" : "DC_P";
          geo_buses.push_back(json{
            {"id", b.index}, {"name", b.name.empty() ? ("DCBus" + std::to_string(b.index)) : b.name},
            {"type", "DC"}, {"bus_type", bus_type_str},
            {"lat", b.latitude}, {"lon", b.longitude},
            {"base_kv", b.base_kv}, {"area", b.area}, {"zone", b.zone},
            {"vm_pu", vdc_val}, {"pd_mw", b.pd_mw}
          });
        }

        // AC branches with coordinates and power flow
        for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
          const auto& br = sys.ac.branches[i];
          auto from_it = ac_bus_coords.find(br.from_bus);
          auto to_it = ac_bus_coords.find(br.to_bus);
          if (from_it == ac_bus_coords.end() || to_it == ac_bus_coords.end()) continue;
          double pf_mw = 0.0, pt_mw = 0.0, qf_mvar = 0.0, qt_mvar = 0.0, loss_mw = 0.0, loading_pct = 0.0;
          if (i < pf.branch_flows.size()) {
            pf_mw = pf.branch_flows[i].pf_mw;
            pt_mw = pf.branch_flows[i].pt_mw;
            qf_mvar = pf.branch_flows[i].qf_mvar;
            qt_mvar = pf.branch_flows[i].qt_mvar;
            loss_mw = pf_mw + pt_mw;  // sum of from and to active power (loss = pf + pt if both signed into the branch)
            if (br.rate_a_mva > 0) {
              double smax = std::max(std::hypot(pf_mw, qf_mvar), std::hypot(pt_mw, qt_mvar));
              loading_pct = 100.0 * smax / br.rate_a_mva;
            }
          }
          geo_ac_branches.push_back(json{
            {"index", br.index},
            {"name", br.name.empty() ? ("Line" + std::to_string(i)) : br.name},
            {"from", br.from_bus}, {"to", br.to_bus},
            {"from_lat", from_it->second.first}, {"from_lon", from_it->second.second},
            {"to_lat", to_it->second.first}, {"to_lon", to_it->second.second},
            {"pf_mw", pf_mw}, {"pt_mw", pt_mw}, {"qf_mvar", qf_mvar}, {"qt_mvar", qt_mvar},
            {"loss_mw", loss_mw}, {"loading_pct", loading_pct}, {"rate_mva", br.rate_a_mva}
          });
        }

        // DC branches with coordinates – compute power flow from DC bus voltages
        {
          // Build dc_bus_index → position map for voltage lookup
          std::unordered_map<int, size_t> dc_idx_map;
          for (size_t k = 0; k < sys.dc.buses.size(); ++k) dc_idx_map[sys.dc.buses[k].index] = k;
          const bool has_vdc = out.contains("vdc") && out["vdc"].is_array() && !out["vdc"].empty();
          for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
            const auto& br = sys.dc.branches[i];
            auto from_it = dc_bus_coords.find(br.from_bus);
            auto to_it = dc_bus_coords.find(br.to_bus);
            if (from_it == dc_bus_coords.end() || to_it == dc_bus_coords.end()) continue;
            double pf_mw = 0.0, pt_mw = 0.0, loss_mw = 0.0, loading_pct = 0.0;
            if (has_vdc && br.in_service) {
              auto fi = dc_idx_map.find(br.from_bus), ti = dc_idx_map.find(br.to_bus);
              if (fi != dc_idx_map.end() && ti != dc_idx_map.end() &&
                  fi->second < out["vdc"].size() && ti->second < out["vdc"].size()) {
                double vf = out["vdc"][fi->second].get<double>();
                double vt = out["vdc"][ti->second].get<double>();
                double r_pu = br.r_pu;
                if (r_pu > 1e-12) {
                  double i_pu = (vf - vt) / r_pu;
                  pf_mw = vf * i_pu * sys.base_mva;    // power leaving from-bus (positive = from→to)
                  pt_mw = -vt * i_pu * sys.base_mva;    // power arriving at to-bus (negative = into to-bus)
                  loss_mw = pf_mw + pt_mw;               // branch loss
                }
              }
              if (br.rate_a_mva > 0) loading_pct = 100.0 * std::abs(pf_mw) / br.rate_a_mva;
            }
            geo_dc_branches.push_back(json{
              {"index", br.index},
              {"name", br.name.empty() ? ("DCLine" + std::to_string(i)) : br.name},
              {"from", br.from_bus}, {"to", br.to_bus},
              {"from_lat", from_it->second.first}, {"from_lon", from_it->second.second},
              {"to_lat", to_it->second.first}, {"to_lon", to_it->second.second},
              {"pf_mw", pf_mw}, {"pt_mw", pt_mw}, {"loss_mw", loss_mw},
              {"loading_pct", loading_pct}, {"rate_mva", br.rate_a_mva}
            });
          }
        }

        // VSC converters connecting AC and DC buses
        for (const auto& v : pf.vsc_transfers) {
          auto ac_it = ac_bus_coords.find(v.bus_ac);
          auto dc_it = dc_bus_coords.find(v.bus_dc);
          if (ac_it == ac_bus_coords.end() || dc_it == dc_bus_coords.end()) continue;
          geo_vsc.push_back(json{
            {"index", v.index},
            {"bus_ac", v.bus_ac}, {"bus_dc", v.bus_dc},
            {"ac_lat", ac_it->second.first}, {"ac_lon", ac_it->second.second},
            {"dc_lat", dc_it->second.first}, {"dc_lon", dc_it->second.second},
            {"p_ac_mw", v.p_ac_mw}, {"q_ac_mvar", v.q_ac_mvar}, {"p_dc_mw", v.p_dc_mw}, {"loss_mw", v.loss_mw}
          });
        }

        // DCDC converters
        for (const auto& d : pf.dcdc_transfers) {
          auto in_it = dc_bus_coords.find(d.bus_in);
          auto out_it = dc_bus_coords.find(d.bus_out);
          if (in_it == dc_bus_coords.end() || out_it == dc_bus_coords.end()) continue;
          geo_dcdc.push_back(json{
            {"index", d.index},
            {"bus_in", d.bus_in}, {"bus_out", d.bus_out},
            {"in_lat", in_it->second.first}, {"in_lon", in_it->second.second},
            {"out_lat", out_it->second.first}, {"out_lon", out_it->second.second},
            {"p_in_mw", d.p_in_mw}, {"p_out_mw", d.p_out_mw}, {"loss_mw", d.loss_mw}
          });
        }

        // 3W transformers with per-winding power flow
        json geo_trafo3w = json::array();
        for (const auto& tf : trafo3w_flows) {
          if (!tf.valid) continue;
          auto hv_it = ac_bus_coords.find(tf.hv_bus);
          auto mv_it = ac_bus_coords.find(tf.mv_bus);
          auto lv_it = ac_bus_coords.find(tf.lv_bus);
          if (hv_it == ac_bus_coords.end() || mv_it == ac_bus_coords.end() ||
              lv_it == ac_bus_coords.end()) continue;
          geo_trafo3w.push_back(json{
            {"index", tf.index},
            {"hv_bus", tf.hv_bus}, {"mv_bus", tf.mv_bus}, {"lv_bus", tf.lv_bus},
            {"hv_lat", hv_it->second.first}, {"hv_lon", hv_it->second.second},
            {"mv_lat", mv_it->second.first}, {"mv_lon", mv_it->second.second},
            {"lv_lat", lv_it->second.first}, {"lv_lon", lv_it->second.second},
            {"p_hv_mw", tf.p_hv_mw}, {"q_hv_mvar", tf.q_hv_mvar},
            {"p_mv_mw", tf.p_mv_mw}, {"q_mv_mvar", tf.q_mv_mvar},
            {"p_lv_mw", tf.p_lv_mw}, {"q_lv_mvar", tf.q_lv_mvar},
            {"loss_mw", tf.loss_mw}, {"loading_pct", tf.loading_pct},
            {"rate_mva", tf.rate_mva}
          });
        }

        // Individual generator results for visualization
        json geo_gen = json::array();
        for (size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
          const auto& g = sys.ac.generators[gi];
          if (!g.in_service) continue;
          geo_gen.push_back(json{
            {"index", g.index}, {"bus", g.bus},
            {"pg_mw", gen_pg_solved[gi]}, {"qg_mvar", gen_qg_solved[gi]},
            {"vg_pu", g.vg_pu}, {"is_slack", g.is_slack},
            {"pmax_mw", g.pmax_mw}, {"pmin_mw", g.pmin_mw},
            {"name", g.name}
          });
        }

        // Energy router port transfers – reconstructed from expanded VSC/DCDC
        // results. During canonical projection each ER port becomes a VSC named
        // <router>_VSC_A/B<port_index>, and each router gets an internal DCDC
        // named <router>_DCDC. Matching by expanded index avoids bus collisions
        // when an ER port shares a bus with a normal VSC or another ER port.
        json geo_er = power_flow_geo_energy_router_json(sys, er_snapshot, pf);

        out["geo_buses"] = geo_buses;
        out["geo_ac_branches"] = geo_ac_branches;
        out["geo_dc_branches"] = geo_dc_branches;
        out["geo_vsc"] = geo_vsc;
        out["geo_dcdc"] = geo_dcdc;
        out["geo_trafo3w"] = geo_trafo3w;
        out["geo_gen"] = geo_gen;
        out["geo_er"] = geo_er;

        // Full post-power-flow component result list for the /xjtu/ canvas GUI.
        // The frontend uses canvas_type + domain + index/position only to match a
        // row back to the drawn component; the values below are produced after the
        // PF run from pf + system data consumed by the solver.
        json component_results = json::array();

        std::unordered_map<int, double> ac_vm_by_bus, ac_va_by_bus, dc_vm_by_bus;
        for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
          const auto& b = sys.ac.buses[i];
          ac_vm_by_bus[b.index] = (i < pf.vm.size()) ? pf.vm[i] : b.vm_pu;
          ac_va_by_bus[b.index] = (i < pf.va.size()) ? pf.va[i] : b.va_deg * M_PI / 180.0;
        }
        for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
          const auto& b = sys.dc.buses[i];
          dc_vm_by_bus[b.index] = (i < pf.vdc.size()) ? pf.vdc[i] : b.vm_pu;
        }

        auto is_solved = [&](const bool in_service) {
          return in_service ? (pf.converged ? std::string("已求解") : std::string("有潮流返回/未收敛"))
                            : std::string("停运");
        };
        auto add_note = [](json& row, const std::string& text) {
          if (!text.empty()) row["notes"].push_back(text);
        };
        auto add_metric = [](json& row, const std::string& label, double value,
                             const std::string& unit, const std::string& quantity,
                             int precision) {
          row["metrics"].push_back(json{{"label", label}, {"value", value},
                                        {"unit", unit}, {"quantity", quantity},
                                        {"precision", precision}});
        };
        auto add_text_metric = [](json& row, const std::string& label, const std::string& value) {
          row["metrics"].push_back(json{{"label", label}, {"value", value},
                                        {"quantity", "text"}});
        };
        auto base_row = [&](const std::string& canvas_type,
                            const std::string& domain,
                            int index,
                            int position,
                            const std::string& type_label,
                            const std::string& name,
                            const std::string& status,
                            const std::string& connection) {
          json row;
          row["canvas_type"] = canvas_type;
          row["domain"] = domain;
          row["index"] = index;
          row["position"] = position;
          row["type_label"] = type_label;
          row["name"] = name;
          row["status"] = status;
          row["connection"] = connection;
          row["metrics"] = json::array();
          row["notes"] = json::array();
          return row;
        };
        auto ac_conn = [](int bus) { return "AC Bus " + std::to_string(bus); };
        auto dc_conn = [](int bus) { return "DC Bus " + std::to_string(bus); };
        auto solved_ac_load = [&](const hacdcpf::Load& ld) {
          const double vm = ac_vm_by_bus.count(ld.bus) ? ac_vm_by_bus[ld.bus] : 1.0;
          const double scale = ld.scaling;
          const double p0 = ld.p_mw * scale;
          const double q0 = ld.q_mvar * scale;
          if (ld.model != hacdcpf::LoadModel::ZIP) {
            return std::pair<double, double>{p0, q0};
          }
          const double p = p0 * (ld.p_percent_p / 100.0 +
                                 ld.i_percent_p / 100.0 * vm +
                                 ld.z_percent_p / 100.0 * vm * vm);
          const double q = q0 * (ld.p_percent_q / 100.0 +
                                 ld.i_percent_q / 100.0 * vm +
                                 ld.z_percent_q / 100.0 * vm * vm);
          return std::pair<double, double>{p, q};
        };

        // Reconstruct per-bus AC flow using the same signed conventions used by
        // the geo_* branch/converter results.  External-grid P/Q is the residual
        // injection needed at its bus after all non-grid devices are accounted.
        std::unordered_map<int, double> ac_flow_p, ac_flow_q;
        auto add_ac_flow = [&](int bus, double p, double q) {
          ac_flow_p[bus] += p;
          ac_flow_q[bus] += q;
        };
        for (size_t i = 0; i < sys.ac.branches.size() && i < pf.branch_flows.size(); ++i) {
          const auto& br = sys.ac.branches[i];
          const auto& fl = pf.branch_flows[i];
          add_ac_flow(br.from_bus, fl.pf_mw, fl.qf_mvar);
          add_ac_flow(br.to_bus, fl.pt_mw, fl.qt_mvar);
        }
        for (size_t ti = 0; ti < trafo2w_flows.size(); ++ti) {
          const auto& tf = trafo2w_flows[ti];
          if (!tf.valid || ti >= sys.ac.transformers_2w.size()) continue;
          const auto& tr = sys.ac.transformers_2w[ti];
          add_ac_flow(tr.hv_bus, tf.p_hv_mw, tf.q_hv_mvar);
          add_ac_flow(tr.lv_bus, tf.p_lv_mw, tf.q_lv_mvar);
        }
        for (const auto& tf : trafo3w_flows) {
          if (!tf.valid) continue;
          add_ac_flow(tf.hv_bus, tf.p_hv_mw, tf.q_hv_mvar);
          add_ac_flow(tf.mv_bus, tf.p_mv_mw, tf.q_mv_mvar);
          add_ac_flow(tf.lv_bus, tf.p_lv_mw, tf.q_lv_mvar);
        }
        for (const auto& v : pf.vsc_transfers) {
          add_ac_flow(v.bus_ac, -v.p_ac_mw, -v.q_ac_mvar);
        }

        std::unordered_map<int, double> ac_non_grid_p, ac_non_grid_q;
        auto add_ac_injection = [&](int bus, double p, double q) {
          ac_non_grid_p[bus] += p;
          ac_non_grid_q[bus] += q;
        };
        for (const auto& b : sys.ac.buses) {
          if (!b.in_service) continue;
          add_ac_injection(b.index, -b.pd_mw, -b.qd_mvar);
          const double vm = ac_vm_by_bus.count(b.index) ? ac_vm_by_bus[b.index] : b.vm_pu;
          const double v2 = vm * vm;
          add_ac_injection(b.index, -b.gs_mw * v2, b.bs_mvar * v2);
        }
        for (size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
          const auto& g = sys.ac.generators[gi];
          if (g.in_service) add_ac_injection(g.bus, gen_pg_solved[gi], gen_qg_solved[gi]);
        }
        for (const auto& sg : sys.ac.static_generators)
          if (sg.in_service) add_ac_injection(sg.bus, sg.p_mw * sg.scaling, sg.q_mvar * sg.scaling);
        for (const auto& rg : sys.ac.renewable_gens)
          if (rg.in_service) add_ac_injection(rg.bus, rg.p_mw, rg.q_mvar);
        for (const auto& pv : sys.ac.pv_systems)
          if (pv.in_service) add_ac_injection(pv.bus, pv.p_mw, pv.q_mvar);
        for (const auto& st : sys.ac.storage)
          if (st.in_service) add_ac_injection(st.bus, st.p_mw, st.q_mvar);
        for (const auto& ld : sys.ac.loads) {
          if (!ld.in_service) continue;
          const auto [p_load, q_load] = solved_ac_load(ld);
          add_ac_injection(ld.bus, -p_load, -q_load);
        }
        for (const auto& fl : sys.ac.flexible_loads)
          if (fl.in_service) add_ac_injection(fl.bus, -fl.p_mw, -fl.q_mvar);
        for (const auto& al : sys.ac.asymmetric_loads) {
          if (!al.in_service) continue;
          add_ac_injection(al.bus,
                           -(al.pa_mw + al.pb_mw + al.pc_mw) * al.scaling,
                           -(al.qa_mvar + al.qb_mvar + al.qc_mvar) * al.scaling);
        }
        for (const auto& sh : sys.ac.shunts)
          if (sh.in_service) {
            const double vm = ac_vm_by_bus.count(sh.bus) ? ac_vm_by_bus[sh.bus] : 1.0;
            const double v2 = vm * vm;
            const double bs = (sh.switchable && sh.n_steps > 0)
                ? sh.bs_per_step * sh.current_step
                : sh.bs_mvar;
            add_ac_injection(sh.bus, -sh.gs_mw * v2, bs * v2);
          }
        for (const auto& cs : sys.ac.charging_stations) {
          if (!cs.in_service) continue;
          const double p_mw = cs.p_total_kw > 0.0
              ? cs.p_total_kw / 1000.0
              : cs.max_power_kw * cs.utilization_rate * cs.simultaneity_factor / 1000.0;
          const double q_mvar = cs.q_total_kvar / 1000.0;
          add_ac_injection(cs.bus, -p_mw, -q_mvar);
        }
        for (const auto& m : sys.ac.motors) {
          if (!m.in_service) continue;
          const double p_mw = m.efficiency > 1e-9 ? m.sn_mva * m.cos_phi / m.efficiency : m.sn_mva * m.cos_phi;
          const double q_mvar = std::sqrt(std::max(0.0, m.sn_mva * m.sn_mva - (m.sn_mva * m.cos_phi) * (m.sn_mva * m.cos_phi)));
          add_ac_injection(m.bus, -p_mw, -q_mvar);
        }
        for (const auto& ms : sys.mobile_storage)
          if (ms.in_service) add_ac_injection(ms.bus, ms.p_mw, ms.q_mvar);
        for (const auto& vpp : sys.vpps)
          if (vpp.in_service) add_ac_injection(vpp.pcc_bus, vpp.p_output_mw, vpp.q_output_mvar);
        for (const auto& mg : sys.microgrids)
          if (mg.in_service) add_ac_injection(mg.pcc_bus, -mg.p_exchange_mw, 0.0);

        // Net non-converter DC device injection per bus (bus-injection positive:
        // PV / static-gen / discharging-storage +, DC load / bus demand -).  The
        // old per-DC-bus "balance" summed only branch + converter port flows and
        // omitted every DC source/load, so it never reflected the bus's real net
        // power.  Combined with the converter port injections below this gives
        // the true per-bus net injection.
        std::unordered_map<int, double> dc_dev_inj_p;
        auto add_dc_dev_inj = [&](int bus, double p) { dc_dev_inj_p[bus] += p; };
        for (const auto& b : sys.dc.buses)
          if (b.in_service) add_dc_dev_inj(b.index, -b.pd_mw);
        for (const auto& ld : sys.dc.loads)
          if (ld.in_service) add_dc_dev_inj(ld.bus, -ld.p_mw * ld.scaling);
        for (const auto& pv : sys.dc.pv_arrays)
          if (pv.in_service) add_dc_dev_inj(pv.bus, pv.p_set_mw);
        for (const auto& sg : sys.dc.static_generators)
          if (sg.in_service) add_dc_dev_inj(sg.bus, sg.p_mw * sg.scaling);
        for (const auto& sg : sys.dc.dc_static_generators)
          if (sg.in_service) add_dc_dev_inj(sg.bus, sg.p_set_mw * sg.scaling);
        for (const auto& st : sys.dc.dc_storage)
          if (st.in_service) add_dc_dev_inj(st.bus, st.p_mw);
        for (const auto& st : sys.dc.storage)
          if (st.in_service) add_dc_dev_inj(st.bus, st.p_mw);

        std::unordered_map<int, int> ext_count_by_bus;
        for (const auto& eg : sys.ac.external_grids)
          if (eg.in_service) ++ext_count_by_bus[eg.bus];

        // Solved external-grid injection per bus (the same reverse-calculated
        // residual used for the external-grid rows), so the AC bus net-injection
        // total includes the grid contribution.
        std::unordered_map<int, double> ac_grid_inj_p, ac_grid_inj_q;
        for (const auto& kv : ext_count_by_bus) {
          const int bus = kv.first;
          ac_grid_inj_p[bus] = ac_flow_p[bus] - ac_non_grid_p[bus];
          ac_grid_inj_q[bus] = ac_flow_q[bus] - ac_non_grid_q[bus];
        }

        // Converter port injections per bus (bus-injection positive).  A VSC
        // injects p_ac at its AC bus and p_dc at its DC bus; a DC/DC withdraws
        // p_in at bus_in (negative injection) and injects p_out at bus_out.
        // These complete the per-bus net injection so it equals the net power
        // the bus exports to the rest of the network (= Σ branch outflow by KCL).
        std::unordered_map<int, double> vsc_inj_ac_p, vsc_inj_ac_q, conv_inj_dc_p;
        for (const auto& v : pf.vsc_transfers) {
          vsc_inj_ac_p[v.bus_ac] += v.p_ac_mw;
          vsc_inj_ac_q[v.bus_ac] += v.q_ac_mvar;
          conv_inj_dc_p[v.bus_dc] += v.p_dc_mw;
        }
        for (const auto& d : pf.dcdc_transfers) {
          conv_inj_dc_p[d.bus_in] += -d.p_in_mw;
          conv_inj_dc_p[d.bus_out] += d.p_out_mw;
        }

        // AC/DC bus rows.
        struct TwoTerminalFlow {
          double pf_mw{0.0};
          double pt_mw{0.0};
          double qf_mvar{0.0};
          double qt_mvar{0.0};
          double loading_pct{0.0};
          double rate_mva{0.0};
          std::string source{"open"};
        };

        std::unordered_map<int, double> ac_net_export_p, ac_net_export_q;
        for (const auto& b : sys.ac.buses) {
          ac_net_export_p[b.index] =
              ac_non_grid_p[b.index] + ac_grid_inj_p[b.index] + vsc_inj_ac_p[b.index];
          ac_net_export_q[b.index] =
              ac_non_grid_q[b.index] + ac_grid_inj_q[b.index] + vsc_inj_ac_q[b.index];
        }
        std::unordered_map<int, double> ac_visible_branch_p, ac_visible_branch_q;
        for (const auto& br : geo_ac_branches) {
          const int from = br.value("from", 0);
          const int to = br.value("to", 0);
          ac_visible_branch_p[from] += br.value("pf_mw", 0.0);
          ac_visible_branch_q[from] += br.value("qf_mvar", 0.0);
          ac_visible_branch_p[to] += br.value("pt_mw", 0.0);
          ac_visible_branch_q[to] += br.value("qt_mvar", 0.0);
        }
        for (size_t ti = 0; ti < trafo2w_flows.size(); ++ti) {
          if (ti >= sys.ac.transformers_2w.size()) continue;
          const auto& tf = trafo2w_flows[ti];
          if (!tf.valid) continue;
          const auto& tr = sys.ac.transformers_2w[ti];
          ac_visible_branch_p[tr.hv_bus] += tf.p_hv_mw;
          ac_visible_branch_q[tr.hv_bus] += tf.q_hv_mvar;
          ac_visible_branch_p[tr.lv_bus] += tf.p_lv_mw;
          ac_visible_branch_q[tr.lv_bus] += tf.q_lv_mvar;
        }
        for (const auto& tf : trafo3w_flows) {
          if (!tf.valid) continue;
          ac_visible_branch_p[tf.hv_bus] += tf.p_hv_mw;
          ac_visible_branch_q[tf.hv_bus] += tf.q_hv_mvar;
          ac_visible_branch_p[tf.mv_bus] += tf.p_mv_mw;
          ac_visible_branch_q[tf.mv_bus] += tf.q_mv_mvar;
          ac_visible_branch_p[tf.lv_bus] += tf.p_lv_mw;
          ac_visible_branch_q[tf.lv_bus] += tf.q_lv_mvar;
        }
        std::unordered_map<int, double> dc_net_export_p;
        for (const auto& b : sys.dc.buses) {
          dc_net_export_p[b.index] = dc_dev_inj_p[b.index] + conv_inj_dc_p[b.index];
        }

        auto is_dc_reference_bus = [&](int bus) {
          for (const auto& b : sys.dc.buses) {
            if (b.index == bus) return b.bus_type == hacdcpf::DCBusType::DC_V;
          }
          return false;
        };
        auto dc_branch_loss_estimate = [&](const hacdcpf::DCBranch& br, int bus, double p_mw) {
          if (br.r_pu <= 0.0 || sys.base_mva <= 1e-12) return 0.0;
          double v = 1.0;
          auto vit = dc_vm_by_bus.find(bus);
          if (vit != dc_vm_by_bus.end() && std::abs(vit->second) > 1e-6) v = std::abs(vit->second);
          const double p_pu = std::abs(p_mw) / sys.base_mva;
          const double i_pu = p_pu / std::max(v, 1e-3);
          return br.r_pu * i_pu * i_pu * sys.base_mva;
        };
        auto set_dc_branch_flow_from_endpoint = [&](size_t branch_pos,
                                                    int exact_bus,
                                                    double exact_outflow_mw,
                                                    const std::string& source) {
          if (branch_pos >= sys.dc.branches.size() || branch_pos >= geo_dc_branches.size()) return;
          const auto& br = sys.dc.branches[branch_pos];
          const double loss = dc_branch_loss_estimate(br, exact_bus, exact_outflow_mw);
          double pf = 0.0;
          double pt = 0.0;
          if (exact_bus == br.from_bus) {
            pf = exact_outflow_mw;
            pt = loss - pf;
          } else if (exact_bus == br.to_bus) {
            pt = exact_outflow_mw;
            pf = loss - pt;
          } else {
            return;
          }
          double loading = 0.0;
          if (br.rate_a_mva > 0.0) {
            loading = 100.0 * std::max(std::abs(pf), std::abs(pt)) / br.rate_a_mva;
          }
          geo_dc_branches[branch_pos]["pf_mw"] = pf;
          geo_dc_branches[branch_pos]["pt_mw"] = pt;
          geo_dc_branches[branch_pos]["loss_mw"] = pf + pt;
          geo_dc_branches[branch_pos]["loading_pct"] = loading;
          geo_dc_branches[branch_pos]["source"] = source;
        };
        auto repair_dc_branch_flows_from_kcl = [&]() {
          if (sys.dc.branches.empty() || sys.dc.buses.empty() || geo_dc_branches.empty()) return;

          std::unordered_map<int, double> current_outflow;
          for (const auto& br : geo_dc_branches) {
            const int from = br.value("from", br.value("from_bus", 0));
            const int to = br.value("to", br.value("to_bus", 0));
            current_outflow[from] += br.value("pf_mw", 0.0);
            current_outflow[to] += br.value("pt_mw", 0.0);
          }
          double max_non_ref_mismatch = 0.0;
          for (const auto& b : sys.dc.buses) {
            if (!b.in_service || is_dc_reference_bus(b.index)) continue;
            max_non_ref_mismatch =
                std::max(max_non_ref_mismatch,
                         std::abs(dc_net_export_p[b.index] - current_outflow[b.index]));
          }
          if (max_non_ref_mismatch < 1e-5) return;

          std::unordered_map<int, std::vector<std::pair<int, size_t>>> adj;
          std::unordered_set<int> bus_ids;
          for (const auto& b : sys.dc.buses) {
            if (b.in_service && b.bus_type != hacdcpf::DCBusType::DC_ISOLATED) {
              bus_ids.insert(b.index);
            }
          }
          size_t active_edges = 0;
          for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
            const auto& br = sys.dc.branches[i];
            if (!br.in_service || !bus_ids.count(br.from_bus) || !bus_ids.count(br.to_bus)) continue;
            adj[br.from_bus].push_back({br.to_bus, i});
            adj[br.to_bus].push_back({br.from_bus, i});
            ++active_edges;
          }
          if (active_edges == 0) return;

          std::unordered_set<int> visited;
          std::unordered_set<size_t> repaired_edges;
          for (int start : bus_ids) {
            if (visited.count(start)) continue;
            std::vector<int> component;
            std::vector<int> roots;
            std::vector<int> stack{start};
            visited.insert(start);
            size_t comp_edges_twice = 0;
            while (!stack.empty()) {
              const int u = stack.back();
              stack.pop_back();
              component.push_back(u);
              if (is_dc_reference_bus(u)) roots.push_back(u);
              comp_edges_twice += adj[u].size();
              for (const auto& [v, edge_pos] : adj[u]) {
                (void)edge_pos;
                if (!visited.count(v)) {
                  visited.insert(v);
                  stack.push_back(v);
                }
              }
            }
            if (roots.size() != 1) continue;
            const size_t comp_edges = comp_edges_twice / 2;
            if (comp_edges + 1 != component.size()) continue;  // KCL tree repair only.

            const int root = roots.front();
            std::unordered_set<int> comp_set(component.begin(), component.end());
            std::function<double(int, int)> dfs = [&](int u, int parent) -> double {
              double subtree_export = dc_net_export_p[u];
              for (const auto& [v, edge_pos] : adj[u]) {
                if (v == parent || !comp_set.count(v)) continue;
                const double child_export = dfs(v, u);
                set_dc_branch_flow_from_endpoint(edge_pos, v, child_export, "dc_kcl_tree");
                repaired_edges.insert(edge_pos);
                subtree_export += child_export;
              }
              return subtree_export;
            };
            (void)dfs(root, -1);
          }

          // Meshed or multi-reference fragments cannot be oriented as a tree.
          // Still recover terminal/leaf branches from their non-reference bus
          // balance so a single visible feeder does not remain at stale zero.
          for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
            if (repaired_edges.count(i)) continue;
            const auto& br = sys.dc.branches[i];
            if (!br.in_service) continue;
            const int f = br.from_bus;
            const int t = br.to_bus;
            if (!is_dc_reference_bus(f) && adj[f].size() == 1) {
              set_dc_branch_flow_from_endpoint(i, f, dc_net_export_p[f], "dc_kcl_leaf");
            } else if (!is_dc_reference_bus(t) && adj[t].size() == 1) {
              set_dc_branch_flow_from_endpoint(i, t, dc_net_export_p[t], "dc_kcl_leaf");
            }
          }
        };
        repair_dc_branch_flows_from_kcl();
        out["geo_dc_branches"] = geo_dc_branches;
        json repaired_dc_flows = json::array();
        for (const auto& br : geo_dc_branches) {
          repaired_dc_flows.push_back(json{
            {"index", br.value("index", 0)},
            {"from_bus", br.value("from", br.value("from_bus", 0))},
            {"to_bus", br.value("to", br.value("to_bus", 0))},
            {"pf_mw", br.value("pf_mw", 0.0)},
            {"pt_mw", br.value("pt_mw", 0.0)},
            {"loss_mw", br.value("loss_mw", br.value("pf_mw", 0.0) + br.value("pt_mw", 0.0))},
            {"loading_pct", br.value("loading_pct", 0.0)},
            {"rate_mva", br.value("rate_mva", 0.0)},
            {"source", br.value("source", std::string{})}
          });
        }
        out["dc_branch_flows"] = std::move(repaired_dc_flows);

        std::unordered_map<int, double> dc_visible_branch_p;
        for (const auto& br : geo_dc_branches) {
          const int from = br.value("from", br.value("from_bus", 0));
          const int to = br.value("to", br.value("to_bus", 0));
          dc_visible_branch_p[from] += br.value("pf_mw", 0.0);
          dc_visible_branch_p[to] += br.value("pt_mw", 0.0);
        }

        auto ac_bus_base_kv = [&](int bus) {
          for (const auto& b : sys.ac.buses) {
            if (b.index == bus) return b.base_kv;
          }
          return 0.0;
        };
        auto dc_bus_base_kv = [&](int bus) {
          for (const auto& b : sys.dc.buses) {
            if (b.index == bus) return b.base_kv;
          }
          return 0.0;
        };
        auto signed_ac_endpoint_flow = [&](int from, int to, double& p, double& q) {
          for (const auto& br : geo_ac_branches) {
            const int br_from = br.value("from", 0);
            const int br_to = br.value("to", 0);
            if (br_from == from && br_to == to) {
              p = br.value("pf_mw", 0.0);
              q = br.value("qf_mvar", 0.0);
              return true;
            }
            if (br_from == to && br_to == from) {
              p = br.value("pt_mw", 0.0);
              q = br.value("qt_mvar", 0.0);
              return true;
            }
          }
          return false;
        };
        auto has_nontrivial_ac_match = [&](int from, int to) {
          double p = 0.0, q = 0.0;
          if (!signed_ac_endpoint_flow(from, to, p, q)) return false;
          return std::hypot(p, q) > 1e-7;
        };
        auto signed_dc_endpoint_flow = [&](int from, int to, double& p) {
          for (const auto& br : geo_dc_branches) {
            const int br_from = br.value("from", br.value("from_bus", 0));
            const int br_to = br.value("to", br.value("to_bus", 0));
            if (br_from == from && br_to == to) {
              p = br.value("pf_mw", 0.0);
              return true;
            }
            if (br_from == to && br_to == from) {
              p = br.value("pt_mw", 0.0);
              return true;
            }
          }
          return false;
        };
        auto derive_ac_terminal_flow = [&](int from, int to, bool closed,
                                           double rated_current_ka,
                                           double rated_voltage_kv) {
          TwoTerminalFlow fl;
          const double kv = rated_voltage_kv > 0.0 ? rated_voltage_kv : ac_bus_base_kv(from);
          fl.rate_mva = rated_current_ka > 0.0 && kv > 0.0
              ? std::sqrt(3.0) * kv * rated_current_ka
              : 0.0;
          if (!closed || from == 0 || to == 0 || from == to) return fl;

          if (signed_ac_endpoint_flow(from, to, fl.pf_mw, fl.qf_mvar) &&
              std::hypot(fl.pf_mw, fl.qf_mvar) > 1e-7) {
            fl.pt_mw = -fl.pf_mw;
            fl.qt_mvar = -fl.qf_mvar;
            fl.source = "matched_ac_branch";
          } else {
            // Closed zero-impedance switching devices are contracted out of the
            // public branch-flow list.  Recover their display flow from the
            // solved KCL at one endpoint: choose the side with the smaller hidden
            // exchange magnitude, which is usually the isolated device side
            // (e.g. a converter bus behind a breaker).
            const double p_from = ac_net_export_p[from] - ac_visible_branch_p[from];
            const double q_from = ac_net_export_q[from] - ac_visible_branch_q[from];
            const double p_to = ac_net_export_p[to] - ac_visible_branch_p[to];
            const double q_to = ac_net_export_q[to] - ac_visible_branch_q[to];
            const double mag_from = std::hypot(p_from, q_from);
            const double mag_to = std::hypot(p_to, q_to);
            if (mag_from <= mag_to || mag_to < 1e-12) {
              fl.pf_mw = p_from;
              fl.qf_mvar = q_from;
              fl.pt_mw = -p_from;
              fl.qt_mvar = -q_from;
              fl.source = "endpoint_kcl_from";
            } else {
              fl.pf_mw = -p_to;
              fl.qf_mvar = -q_to;
              fl.pt_mw = p_to;
              fl.qt_mvar = q_to;
              fl.source = "endpoint_kcl_to";
            }
          }
          if (fl.rate_mva > 0.0) {
            fl.loading_pct = 100.0 *
                std::max(std::hypot(fl.pf_mw, fl.qf_mvar),
                         std::hypot(fl.pt_mw, fl.qt_mvar)) / fl.rate_mva;
          }
          return fl;
        };
        auto derive_dc_terminal_flow = [&](int from, int to, bool closed,
                                           double rated_current_ka,
                                           double rated_voltage_kv) {
          TwoTerminalFlow fl;
          const double kv = rated_voltage_kv > 0.0 ? rated_voltage_kv : dc_bus_base_kv(from);
          fl.rate_mva = rated_current_ka > 0.0 && kv > 0.0 ? kv * rated_current_ka : 0.0;
          if (!closed || from == 0 || to == 0 || from == to) return fl;

          double p = 0.0;
          if (signed_dc_endpoint_flow(from, to, p)) {
            fl.pf_mw = p;
            fl.pt_mw = -p;
            fl.source = "matched_dc_branch";
          } else {
            const double p_from = dc_net_export_p[from] - dc_visible_branch_p[from];
            const double p_to = dc_net_export_p[to] - dc_visible_branch_p[to];
            const double mag_from = std::abs(p_from);
            const double mag_to = std::abs(p_to);
            if (mag_from <= mag_to || mag_to < 1e-12) {
              fl.pf_mw = p_from;
              fl.pt_mw = -p_from;
              fl.source = "endpoint_kcl_from";
            } else {
              fl.pf_mw = -p_to;
              fl.pt_mw = p_to;
              fl.source = "endpoint_kcl_to";
            }
          }
          if (fl.rate_mva > 0.0) {
            fl.loading_pct = 100.0 * std::max(std::abs(fl.pf_mw), std::abs(fl.pt_mw)) / fl.rate_mva;
          }
          return fl;
        };
        auto flow_json = [](int index, size_t position, int from, int to,
                            const TwoTerminalFlow& fl, const std::string& source_type) {
          return json{
            {"index", index},
            {"position", static_cast<int>(position)},
            {"from", from},
            {"to", to},
            {"from_bus", from},
            {"to_bus", to},
            {"pf_mw", fl.pf_mw},
            {"pt_mw", fl.pt_mw},
            {"qf_mvar", fl.qf_mvar},
            {"qt_mvar", fl.qt_mvar},
            {"loss_mw", fl.pf_mw + fl.pt_mw},
            {"loading_pct", fl.loading_pct},
            {"rate_mva", fl.rate_mva},
            {"source", fl.source},
            {"source_type", source_type}
          };
        };

        json ac_switch_flows = json::array();
        for (size_t i = 0; i < sys.ac.switches.size(); ++i) {
          const auto& sw = sys.ac.switches[i];
          const bool closed = sw.in_service && sw.closed;
          const bool contracted = closed && !has_nontrivial_ac_match(sw.bus_from, sw.bus_to);
          const auto fl = derive_ac_terminal_flow(sw.bus_from, sw.bus_to,
                                                  closed,
                                                  sw.i_rated_ka, 0.0);
          auto row = flow_json(sw.index, i, sw.bus_from, sw.bus_to, fl, "ac_switch");
          if (contracted) {
            row["source"] = fl.source == "open" ? "open" : fl.source + "_contracted";
            ac_visible_branch_p[sw.bus_from] += fl.pf_mw;
            ac_visible_branch_q[sw.bus_from] += fl.qf_mvar;
            ac_visible_branch_p[sw.bus_to] += fl.pt_mw;
            ac_visible_branch_q[sw.bus_to] += fl.qt_mvar;
          }
          ac_switch_flows.push_back(std::move(row));
        }
        json ac_cb_flows = json::array();
        for (size_t i = 0; i < sys.ac.circuit_breakers.size(); ++i) {
          const auto& cb = sys.ac.circuit_breakers[i];
          const bool closed = cb.in_service && cb.closed;
          const bool contracted = closed && !has_nontrivial_ac_match(cb.bus_from, cb.bus_to);
          const auto fl = derive_ac_terminal_flow(cb.bus_from, cb.bus_to,
                                                  closed,
                                                  cb.i_rated_ka, cb.rated_voltage_kv);
          auto row = flow_json(cb.index, i, cb.bus_from, cb.bus_to, fl, "ac_circuit_breaker");
          if (contracted) {
            row["source"] = fl.source == "open" ? "open" : fl.source + "_contracted";
            ac_visible_branch_p[cb.bus_from] += fl.pf_mw;
            ac_visible_branch_q[cb.bus_from] += fl.qf_mvar;
            ac_visible_branch_p[cb.bus_to] += fl.pt_mw;
            ac_visible_branch_q[cb.bus_to] += fl.qt_mvar;
          }
          ac_cb_flows.push_back(std::move(row));
        }
        json dc_cb_flows = json::array();
        for (size_t i = 0; i < sys.dc.dc_circuit_breakers.size(); ++i) {
          const auto& cb = sys.dc.dc_circuit_breakers[i];
          const auto fl = derive_dc_terminal_flow(cb.bus_from, cb.bus_to,
                                                  cb.in_service && cb.closed,
                                                  cb.i_rated_ka, cb.rated_voltage_kv);
          dc_cb_flows.push_back(flow_json(cb.index, i, cb.bus_from, cb.bus_to, fl, "dc_circuit_breaker"));
        }
        out["ac_switch_flows"] = ac_switch_flows;
        out["ac_circuit_breaker_flows"] = ac_cb_flows;
        out["dc_circuit_breaker_flows"] = dc_cb_flows;

        std::unordered_map<int, double> dc_terminal_export_p = dc_visible_branch_p;
        for (const auto& fl : dc_cb_flows) {
          if (fl.value("source", std::string{}) == "matched_dc_branch") continue;
          dc_terminal_export_p[fl.value("from_bus", fl.value("from", 0))] += fl.value("pf_mw", 0.0);
          dc_terminal_export_p[fl.value("to_bus", fl.value("to", 0))] += fl.value("pt_mw", 0.0);
        }
        std::unordered_map<int, double> dc_balance_source_p;
        for (const auto& b : sys.dc.buses) {
          if (b.bus_type != hacdcpf::DCBusType::DC_V) continue;
          const double source_p = dc_terminal_export_p[b.index] - dc_net_export_p[b.index];
          if (std::abs(source_p) > 1e-5) dc_balance_source_p[b.index] = source_p;
        }
        std::unordered_map<int, int> dc_swing_storage_count;
        auto storage_is_dc_swing = [&](int bus, bool in_service, bool controllable) {
          return in_service && controllable && dc_balance_source_p.count(bus) > 0;
        };
        for (const auto& st : sys.dc.storage) {
          if (storage_is_dc_swing(st.bus, st.in_service, st.controllable)) {
            ++dc_swing_storage_count[st.bus];
          }
        }
        for (const auto& st : sys.dc.dc_storage) {
          if (storage_is_dc_swing(st.bus, st.in_service, st.controllable)) {
            ++dc_swing_storage_count[st.bus];
          }
        }
        auto solved_dc_storage_p = [&](int bus, double scheduled_p, bool in_service,
                                       bool controllable) {
          if (!storage_is_dc_swing(bus, in_service, controllable)) return scheduled_p;
          const int n = std::max(1, dc_swing_storage_count[bus]);
          return scheduled_p + dc_balance_source_p[bus] / static_cast<double>(n);
        };

        auto metric_detail = [](const std::string& label, double mw) {
          return json{{"label", label}, {"mw", mw}};
        };
        auto ac_is_slack_bus = [&](int bus) {
          for (const auto& b : sys.ac.buses) {
            if (b.index == bus && b.bus_type == hacdcpf::BusType::SLACK) return true;
          }
          for (const auto& eg : sys.ac.external_grids) {
            if (eg.in_service && eg.bus == bus) return true;
          }
          for (const auto& conv : sys.vsc_converters) {
            if (!conv.in_service) continue;
            if (conv.bus_ac == bus &&
                (conv.ac_grid_forming ||
                 conv.control_mode == hacdcpf::ConverterMode::AC_GRID_FORMING)) {
              return true;
            }
          }
          return false;
        };
        auto ac_source_power = [&](int bus) {
          double p = ac_grid_inj_p[bus];
          for (size_t gi = 0; gi < sys.ac.generators.size(); ++gi) {
            const auto& g = sys.ac.generators[gi];
            if (!g.in_service || g.bus != bus) continue;
            if (g.is_slack || ac_is_slack_bus(bus)) p += gen_pg_solved[gi];
          }
          for (const auto& v : pf.vsc_transfers) {
            if (v.bus_ac == bus) {
              auto cit = std::find_if(sys.vsc_converters.begin(), sys.vsc_converters.end(),
                  [&](const hacdcpf::VSCConverter& c) { return c.index == v.index; });
              const bool forming = cit != sys.vsc_converters.end() &&
                  (cit->ac_grid_forming ||
                   cit->control_mode == hacdcpf::ConverterMode::AC_GRID_FORMING);
              if (forming) p += v.p_ac_mw;
            }
          }
          return p;
        };
        auto dc_reference_sets = [&]() {
          struct RefSets {
            std::unordered_set<int> explicit_refs;
            std::unordered_set<int> implicit_refs;
          } refs;
          std::unordered_set<int> bus_ids;
          std::unordered_map<int, const hacdcpf::DCBus*> by_bus;
          std::unordered_map<int, std::vector<int>> adj;
          for (const auto& b : sys.dc.buses) {
            if (!b.in_service) continue;
            bus_ids.insert(b.index);
            by_bus[b.index] = &b;
            adj[b.index];
            if (b.bus_type == hacdcpf::DCBusType::DC_V) refs.explicit_refs.insert(b.index);
          }
          auto link = [&](int a, int b) {
            if (!bus_ids.count(a) || !bus_ids.count(b)) return;
            adj[a].push_back(b);
            adj[b].push_back(a);
          };
          for (const auto& br : sys.dc.branches) {
            if (br.in_service) link(br.from_bus, br.to_bus);
          }
          for (const auto& d : sys.dc.dcdc_converters) {
            if (d.in_service) link(d.bus_in, d.bus_out);
          }
          std::unordered_set<int> visited;
          for (int start : bus_ids) {
            if (visited.count(start)) continue;
            std::vector<int> comp;
            std::vector<int> stack{start};
            visited.insert(start);
            while (!stack.empty()) {
              const int u = stack.back();
              stack.pop_back();
              comp.push_back(u);
              for (int v : adj[u]) {
                if (!visited.count(v)) {
                  visited.insert(v);
                  stack.push_back(v);
                }
              }
            }
            const bool has_explicit = std::any_of(comp.begin(), comp.end(), [&](int b) {
              return refs.explicit_refs.count(b) > 0;
            });
            if (has_explicit || comp.empty()) continue;
            auto pick = std::find_if(comp.begin(), comp.end(), [&](int b) {
              auto it = by_bus.find(b);
              return it != by_bus.end() &&
                     it->second->bus_type != hacdcpf::DCBusType::DC_ISOLATED;
            });
            if (pick == comp.end()) pick = comp.begin();
            refs.implicit_refs.insert(*pick);
          }
          return refs;
        }();
        auto ac_balance_groups = [&]() {
          std::unordered_map<int, int> parent;
          for (const auto& b : sys.ac.buses) {
            if (b.in_service) parent[b.index] = b.index;
          }
          std::function<int(int)> find = [&](int x) {
            auto it = parent.find(x);
            if (it == parent.end()) return x;
            if (it->second == x) return x;
            it->second = find(it->second);
            return it->second;
          };
          auto unite = [&](int a, int b) {
            if (!parent.count(a) || !parent.count(b)) return;
            int ra = find(a), rb = find(b);
            if (ra == rb) return;
            if (rb < ra) std::swap(ra, rb);
            parent[rb] = ra;
          };
          for (const auto& sw : sys.ac.switches) {
            if (sw.in_service && sw.closed) unite(sw.bus_from, sw.bus_to);
          }
          for (const auto& cb : sys.ac.circuit_breakers) {
            if (cb.in_service && cb.closed) unite(cb.bus_from, cb.bus_to);
          }
          std::unordered_map<int, std::vector<int>> groups;
          for (const auto& kv : parent) groups[find(kv.first)].push_back(kv.first);
          for (auto& kv : groups) std::sort(kv.second.begin(), kv.second.end());
          return groups;
        }();
        auto dc_is_reference = [&](int bus) {
          return dc_reference_sets.explicit_refs.count(bus) > 0 ||
                 dc_reference_sets.implicit_refs.count(bus) > 0;
        };

        json balance_diag;
        balance_diag["basis"] =
            "canonical_projection_with_rich_terminal_attribution";
        balance_diag["tolerance_kw"] = 1.0;
        balance_diag["ordinary"] = json::array();
        balance_diag["sources"] = json::array();
        balance_diag["ac_terminal_export_p"] = json::object();
        balance_diag["dc_terminal_export_p"] = json::object();
        for (const auto& [bus, p] : ac_visible_branch_p)
          balance_diag["ac_terminal_export_p"][std::to_string(bus)] = p;
        for (const auto& [bus, p] : dc_terminal_export_p)
          balance_diag["dc_terminal_export_p"][std::to_string(bus)] = p;

        auto push_balance_row = [&](const std::string& domain, int bus,
                                    const std::string& type,
                                    const std::string& name,
                                    double residual_mw,
                                    double net_mw,
                                    double terminal_mw,
                                    bool is_implicit) {
          json details = json::array();
          details.push_back(metric_detail("端口外送合计", terminal_mw));
          details.push_back(metric_detail("求解净注入", -net_mw));
          balance_diag["ordinary"].push_back(json{
              {"kind", domain},
              {"domain", domain},
              {"id", bus},
              {"bus", bus},
              {"type", type},
              {"bus_type", type},
              {"name", name},
              {"mw", residual_mw},
              {"kw", residual_mw * 1000.0},
              {"residual_mw", residual_mw},
              {"residual_kw", residual_mw * 1000.0},
              {"net_injection_mw", net_mw},
              {"terminal_export_mw", terminal_mw},
              {"isImplicit", is_implicit},
              {"is_implicit", is_implicit},
              {"details", details}});
        };
        auto push_source_row = [&](const std::string& domain, int bus,
                                   const std::string& type,
                                   const std::string& name,
                                   double source_mw,
                                   bool is_implicit,
                                   const std::string& label) {
          if (std::abs(source_mw) * 1000.0 <= 1.0) return;
          json details = json::array();
          details.push_back(metric_detail(label, source_mw));
          balance_diag["sources"].push_back(json{
              {"kind", domain},
              {"domain", domain},
              {"id", bus},
              {"bus", bus},
              {"type", type},
              {"bus_type", type},
              {"name", name},
              {"mw", source_mw},
              {"kw", source_mw * 1000.0},
              {"source_mw", source_mw},
              {"source_kw", source_mw * 1000.0},
              {"isImplicit", is_implicit},
              {"is_implicit", is_implicit},
              {"details", details}});
        };
        int ordinary_bad_count = 0;
        auto ac_bus_by_id = [&](int bus) -> const hacdcpf::ACBus* {
          for (const auto& b : sys.ac.buses)
            if (b.index == bus) return &b;
          return nullptr;
        };
        for (const auto& [root, members] : ac_balance_groups) {
          double net = 0.0;
          double terminal = 0.0;
          bool is_slack = false;
          std::string type = "PQ";
          std::string name;
          int row_bus = root;
          for (int bus : members) {
            const auto* b = ac_bus_by_id(bus);
            if (!b || !b->in_service) continue;
            net += ac_net_export_p[bus];
            terminal += ac_visible_branch_p[bus];
            if (ac_is_slack_bus(bus)) {
              is_slack = true;
              row_bus = bus;
              type = bus_type_str(b->bus_type);
              name = b->name.empty() ? ("Bus " + std::to_string(bus)) : b->name;
            }
          }
          if (name.empty()) {
            const auto* b = ac_bus_by_id(row_bus);
            if (b) {
              type = bus_type_str(b->bus_type);
              name = b->name.empty() ? ("Bus " + std::to_string(row_bus)) : b->name;
            }
          }
          const double residual = terminal - net;
          if (!is_slack && std::abs(residual) * 1000.0 > 1.0) {
            ++ordinary_bad_count;
            push_balance_row("AC", row_bus, type, name,
                             residual, net, terminal, false);
            if (members.size() > 1) {
              auto& row = balance_diag["ordinary"].back();
              row["merged_buses"] = members;
              row["basis_note"] =
                  "closed AC switches/circuit breakers are checked as one canonical supernode";
            }
          }
          if (is_slack) {
            double source = 0.0;
            for (int bus : members) source += ac_source_power(bus);
            push_source_row("AC", row_bus, type, name, source, false, "AC平衡源");
          }
        }
        for (const auto& b : sys.dc.buses) {
          if (!b.in_service) continue;
          const double net = dc_net_export_p[b.index];
          const double terminal = dc_terminal_export_p[b.index];
          const double residual = terminal - net;
          const bool is_ref = dc_is_reference(b.index);
          const bool is_implicit = dc_reference_sets.implicit_refs.count(b.index) > 0;
          const std::string name =
              b.name.empty() ? ("DC Bus " + std::to_string(b.index)) : b.name;
          if (!is_ref && std::abs(residual) * 1000.0 > 1.0) {
            ++ordinary_bad_count;
            push_balance_row("DC", b.index, dc_bus_type_str(b.bus_type), name,
                             residual, net, terminal, is_implicit);
          }
          if (is_ref) {
            double source = terminal - net;
            auto sit = dc_balance_source_p.find(b.index);
            if (sit != dc_balance_source_p.end()) source = sit->second;
            push_source_row("DC", b.index, dc_bus_type_str(b.bus_type), name,
                            source, is_implicit,
                            is_implicit ? "隐式DC平衡源" : "DC_V平衡源");
          }
        }
        balance_diag["ordinary_bad_count"] = ordinary_bad_count;
        balance_diag["source_count"] = balance_diag["sources"].size();
        out["power_balance_diagnostics"] = balance_diag;

        for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
          const auto& b = sys.ac.buses[i];
          json row = base_row("ac_bus", "AC", b.index, static_cast<int>(i),
                              "交流母线", b.name.empty() ? ("Bus " + std::to_string(b.index)) : b.name,
                              is_solved(b.in_service), ac_conn(b.index));
          bool converter_slack = false;
          for (const auto& conv : sys.vsc_converters) {
            if (conv.in_service && conv.bus_ac == b.index &&
                conv.control_mode == hacdcpf::ConverterMode::AC_GRID_FORMING) {
              converter_slack = true;
              break;
            }
          }
          add_text_metric(row, "类型", converter_slack ? "SLACK" : bus_type_str(b.bus_type));
          add_metric(row, "Vm", ac_vm_by_bus[b.index], "pu", "scalar", 6);
          add_metric(row, "Va", ac_va_by_bus[b.index] * 180.0 / M_PI, "deg", "scalar", 4);
          // Net nodal injection = Σ(all device injections at the bus: generation/
          // grid/converter +, load −).  Equals the net power the bus exports to
          // the network (Σ branch outflow) by KCL.  (The old value summed only
          // branch/converter port flows and ignored local loads/sources.)
          const double ac_net_p =
              ac_non_grid_p[b.index] + ac_grid_inj_p[b.index] + vsc_inj_ac_p[b.index];
          const double ac_net_q =
              ac_non_grid_q[b.index] + ac_grid_inj_q[b.index] + vsc_inj_ac_q[b.index];
          add_metric(row, "P净注入", ac_net_p, "MW", "p", 4);
          add_metric(row, "Q净注入", ac_net_q, "MVar", "q", 4);
          add_note(row, "净注入=该母线所有设备净注入(电源/外网/变换器为正，负荷为负)，等于经支路外送的净有功/无功");
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.dc.buses.size(); ++i) {
          const auto& b = sys.dc.buses[i];
          json row = base_row("dc_bus", "DC", b.index, static_cast<int>(i),
                              "直流母线", b.name.empty() ? ("DC Bus " + std::to_string(b.index)) : b.name,
                              is_solved(b.in_service), dc_conn(b.index));
          add_text_metric(row, "类型", dc_bus_type_str(b.bus_type));
          add_metric(row, "Vdc", dc_vm_by_bus[b.index], "pu", "scalar", 6);
          // Net nodal injection = Σ(DC source/load/converter injections).
          const double dc_net_p = dc_net_export_p[b.index];
          add_metric(row, "P净注入", dc_net_p, "MW", "p", 4);
          const auto source_it = dc_balance_source_p.find(b.index);
          if (source_it != dc_balance_source_p.end()) {
            add_metric(row, "DC平衡源", source_it->second, "MW", "p", 4);
          }
          add_note(row, "净注入=该母线普通设备净注入(直流源为正、负荷为负、变换器按端口方向)；DC_V母线还可显示平衡源功率，使总注入等于经支路外送的净有功");
          component_results.push_back(std::move(row));
        }

        // AC branch and transformer rows.
        for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
          const auto& br = sys.ac.branches[i];
          json row = base_row("ac_branch", "AC", br.index, static_cast<int>(i),
                              "交流线路", br.name.empty() ? ("Line " + std::to_string(br.index)) : br.name,
                              is_solved(br.in_service),
                              ac_conn(br.from_bus) + " -> " + ac_conn(br.to_bus));
          if (i < pf.branch_flows.size()) {
            const auto& fl = pf.branch_flows[i];
            add_metric(row, "Pf", fl.pf_mw, "MW", "p", 4);
            add_metric(row, "Pt", fl.pt_mw, "MW", "p", 4);
            add_metric(row, "Qf", fl.qf_mvar, "MVar", "q", 4);
            add_metric(row, "Qt", fl.qt_mvar, "MVar", "q", 4);
            add_metric(row, "Loss", fl.pf_mw + fl.pt_mw, "MW", "p", 4);
            if (br.rate_a_mva > 0.0) {
              const double loading = 100.0 * std::max(std::hypot(fl.pf_mw, fl.qf_mvar),
                                                       std::hypot(fl.pt_mw, fl.qt_mvar)) / br.rate_a_mva;
              add_metric(row, "Loading", loading, "%", "scalar", 1);
            }
          }
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.transformers_2w.size(); ++i) {
          const auto& tr = sys.ac.transformers_2w[i];
          json row = base_row("transformer_2w", "AC", tr.index, static_cast<int>(i),
                              "双绕组变压器", tr.name.empty() ? ("Trafo " + std::to_string(tr.index)) : tr.name,
                              is_solved(tr.in_service),
                              ac_conn(tr.hv_bus) + " -> " + ac_conn(tr.lv_bus));
          if (i < trafo2w_flows.size() && trafo2w_flows[i].valid) {
            const auto& fl = trafo2w_flows[i];
            add_metric(row, "P_HV", fl.p_hv_mw, "MW", "p", 4);
            add_metric(row, "P_LV", fl.p_lv_mw, "MW", "p", 4);
            add_metric(row, "Q_HV", fl.q_hv_mvar, "MVar", "q", 4);
            add_metric(row, "Q_LV", fl.q_lv_mvar, "MVar", "q", 4);
            add_metric(row, "Loss", fl.loss_mw, "MW", "p", 4);
            add_note(row, "双绕组变压器端口潮流由潮流后两端电压和变压器等值参数反算");
          } else {
            add_metric(row, "HV Vm", ac_vm_by_bus[tr.hv_bus], "pu", "scalar", 6);
            add_metric(row, "LV Vm", ac_vm_by_bus[tr.lv_bus], "pu", "scalar", 6);
            add_metric(row, "Sn", tr.sn_mva, "MVA", "scalar", 3);
            add_metric(row, "Tap", static_cast<double>(tr.tap_pos), "", "scalar", 0);
            add_note(row, "该元件已参与潮流；未能反算独立双绕组变压器端口潮流");
          }
          component_results.push_back(std::move(row));
        }
        for (const auto& tf : trafo3w_flows) {
          if (!tf.valid) continue;
          json row = base_row("transformer_3w", "AC", tf.index, tf.index,
                              "三绕组变压器", "Trafo3W " + std::to_string(tf.index),
                              pf.converged ? "已求解" : "有潮流返回/未收敛",
                              ac_conn(tf.hv_bus) + "; " + ac_conn(tf.mv_bus) + "; " + ac_conn(tf.lv_bus));
          add_metric(row, "P_HV", tf.p_hv_mw, "MW", "p", 4);
          add_metric(row, "P_MV", tf.p_mv_mw, "MW", "p", 4);
          add_metric(row, "P_LV", tf.p_lv_mw, "MW", "p", 4);
          add_metric(row, "Loss", tf.loss_mw, "MW", "p", 4);
          add_metric(row, "Loading", tf.loading_pct, "%", "scalar", 1);
          component_results.push_back(std::move(row));
        }

        // Source, load, storage, shunt and aggregation rows.
        for (size_t i = 0; i < sys.ac.generators.size(); ++i) {
          const auto& g = sys.ac.generators[i];
          json row = base_row("generator", "AC", g.index, static_cast<int>(i),
                              "发电机", g.name.empty() ? ("Gen " + std::to_string(g.index)) : g.name,
                              is_solved(g.in_service), ac_conn(g.bus));
          add_metric(row, "Pg", gen_pg_solved[i], "MW", "p", 4);
          add_metric(row, "Qg", gen_qg_solved[i], "MVar", "q", 4);
          add_metric(row, "Vg", g.vg_pu, "pu", "scalar", 4);
          add_text_metric(row, "Slack", g.is_slack ? "是" : "否");
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.external_grids.size(); ++i) {
          const auto& eg = sys.ac.external_grids[i];
          const int n_ext = std::max(1, ext_count_by_bus[eg.bus]);
          const double p = (ac_flow_p[eg.bus] - ac_non_grid_p[eg.bus]) / static_cast<double>(n_ext);
          const double q = (ac_flow_q[eg.bus] - ac_non_grid_q[eg.bus]) / static_cast<double>(n_ext);
          json row = base_row("external_grid", "AC", eg.index, static_cast<int>(i),
                              "外部电网", eg.name.empty() ? ("Grid " + std::to_string(eg.index)) : eg.name,
                              is_solved(eg.in_service), ac_conn(eg.bus));
          add_metric(row, "P平衡", eg.in_service ? p : 0.0, "MW", "p", 4);
          add_metric(row, "Q平衡", eg.in_service ? q : 0.0, "MVar", "q", 4);
          add_metric(row, "Vm", ac_vm_by_bus[eg.bus], "pu", "scalar", 6);
          add_metric(row, "Va", ac_va_by_bus[eg.bus] * 180.0 / M_PI, "deg", "scalar", 4);
          add_note(row, "P/Q由潮流后母线功率平衡反算");
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.loads.size(); ++i) {
          const auto& ld = sys.ac.loads[i];
          const auto [p_load, q_load] = ld.in_service
              ? solved_ac_load(ld)
              : std::pair<double, double>{0.0, 0.0};
          json row = base_row("load", "AC", ld.index, static_cast<int>(i),
                              "负荷", ld.name.empty() ? ("Load " + std::to_string(ld.index)) : ld.name,
                              is_solved(ld.in_service), ac_conn(ld.bus));
          add_metric(row, "P消耗", p_load, "MW", "p", 4);
          add_metric(row, "Q消耗", q_load, "MVar", "q", 4);
          if (ld.in_service && ld.model == hacdcpf::LoadModel::ZIP) {
            add_note(row, "ZIP负荷按求解后母线电压折算显示");
          }
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.flexible_loads.size(); ++i) {
          const auto& fl = sys.ac.flexible_loads[i];
          json row = base_row("flexible_load", "AC", fl.index, static_cast<int>(i),
                              "柔性负荷", fl.name.empty() ? ("FlexLoad " + std::to_string(fl.index)) : fl.name,
                              is_solved(fl.in_service), ac_conn(fl.bus));
          add_metric(row, "P消耗", fl.in_service ? fl.p_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Q消耗", fl.in_service ? fl.q_mvar : 0.0, "MVar", "q", 4);
          add_metric(row, "可上调", fl.flex_up_mw, "MW", "p", 4);
          add_metric(row, "可下调", fl.flex_down_mw, "MW", "p", 4);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.asymmetric_loads.size(); ++i) {
          const auto& al = sys.ac.asymmetric_loads[i];
          json row = base_row("asymmetric_load", "AC", al.index, static_cast<int>(i),
                              "不平衡负荷", al.name.empty() ? ("AsymLoad " + std::to_string(al.index)) : al.name,
                              is_solved(al.in_service), ac_conn(al.bus));
          add_metric(row, "Pa", al.pa_mw * al.scaling, "MW", "p", 4);
          add_metric(row, "Pb", al.pb_mw * al.scaling, "MW", "p", 4);
          add_metric(row, "Pc", al.pc_mw * al.scaling, "MW", "p", 4);
          add_metric(row, "Psum", (al.pa_mw + al.pb_mw + al.pc_mw) * al.scaling, "MW", "p", 4);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.storage.size(); ++i) {
          const auto& st = sys.ac.storage[i];
          json row = base_row("storage", "AC", st.index, static_cast<int>(i),
                              "储能", st.name.empty() ? ("ESS " + std::to_string(st.index)) : st.name,
                              is_solved(st.in_service), ac_conn(st.bus));
          add_metric(row, "P计算", st.in_service ? st.p_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Q计算", st.in_service ? st.q_mvar : 0.0, "MVar", "q", 4);
          add_metric(row, "SOC", st.soc_init, "", "scalar", 4);
          add_metric(row, "P额定", st.p_rated_mw, "MW", "p", 4);
          add_note(row, "正值为放电注入，负值为充电吸收");
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.static_generators.size(); ++i) {
          const auto& sg = sys.ac.static_generators[i];
          json row = base_row("static_generator", "AC", sg.index, static_cast<int>(i),
                              "分布式电源", sg.name.empty() ? ("SGen " + std::to_string(sg.index)) : sg.name,
                              is_solved(sg.in_service), ac_conn(sg.bus));
          add_metric(row, "P注入", sg.in_service ? sg.p_mw * sg.scaling : 0.0, "MW", "p", 4);
          add_metric(row, "Q注入", sg.in_service ? sg.q_mvar * sg.scaling : 0.0, "MVar", "q", 4);
          add_text_metric(row, "类型", sgen_type_str(sg.sgen_type));
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.renewable_gens.size(); ++i) {
          const auto& rg = sys.ac.renewable_gens[i];
          json row = base_row("renewable_gen", "AC", rg.index, static_cast<int>(i),
                              "可再生电源", rg.name.empty() ? ("RenGen " + std::to_string(rg.index)) : rg.name,
                              is_solved(rg.in_service), ac_conn(rg.bus));
          add_metric(row, "P注入", rg.in_service ? rg.p_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Q注入", rg.in_service ? rg.q_mvar : 0.0, "MVar", "q", 4);
          add_text_metric(row, "类型", renewable_type_str(rg.type));
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.pv_systems.size(); ++i) {
          const auto& pv = sys.ac.pv_systems[i];
          json row = base_row("pv_system", "AC", pv.index, static_cast<int>(i),
                              "光伏系统", pv.name.empty() ? ("PV " + std::to_string(pv.index)) : pv.name,
                              is_solved(pv.in_service), ac_conn(pv.bus));
          add_metric(row, "P注入", pv.in_service ? pv.p_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Q注入", pv.in_service ? pv.q_mvar : 0.0, "MVar", "q", 4);
          add_text_metric(row, "控制", pv_control_str(pv.control_mode));
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.shunts.size(); ++i) {
          const auto& sh = sys.ac.shunts[i];
          json row = base_row("shunt", "AC", sh.index, static_cast<int>(i),
                              "并联补偿", sh.name.empty() ? ("Shunt " + std::to_string(sh.index)) : sh.name,
                              is_solved(sh.in_service), ac_conn(sh.bus));
          add_metric(row, "Gs", sh.in_service ? sh.gs_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Bs", sh.in_service ? sh.bs_mvar : 0.0, "MVar", "q", 4);
          add_text_metric(row, "投切", sh.switchable ? "可投切" : "固定");
          component_results.push_back(std::move(row));
        }

        // Switching/protection and special AC demand rows.
        for (size_t i = 0; i < sys.ac.switches.size(); ++i) {
          const auto& sw = sys.ac.switches[i];
          json row = base_row("switch_comp", "AC", sw.index, static_cast<int>(i),
                              "开关", sw.name.empty() ? ("Switch " + std::to_string(sw.index)) : sw.name,
                              sw.in_service ? (sw.closed ? "合闸" : "分闸") : "停运",
                              ac_conn(sw.bus_from) + " -> " + ac_conn(sw.bus_to));
          const auto& fl = ac_switch_flows[static_cast<int>(i)];
          add_text_metric(row, "状态", sw.closed ? "合闸" : "分闸");
          add_text_metric(row, "类型", switch_type_str(sw.switch_type));
          add_metric(row, "Pf", fl.value("pf_mw", 0.0), "MW", "p", 4);
          add_metric(row, "Pt", fl.value("pt_mw", 0.0), "MW", "p", 4);
          add_metric(row, "Qf", fl.value("qf_mvar", 0.0), "MVar", "q", 4);
          add_metric(row, "Qt", fl.value("qt_mvar", 0.0), "MVar", "q", 4);
          if (fl.value("rate_mva", 0.0) > 0.0) {
            add_metric(row, "Loading", fl.value("loading_pct", 0.0), "%", "scalar", 1);
          }
          add_note(row, "开关潮流由求解后支路匹配或端点KCL反算，用于显示零阻抗合闸开关的真实通过功率");
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.circuit_breakers.size(); ++i) {
          const auto& cb = sys.ac.circuit_breakers[i];
          json row = base_row("circuit_breaker", "AC", cb.index, static_cast<int>(i),
                              "断路器", cb.name.empty() ? ("CB " + std::to_string(cb.index)) : cb.name,
                              cb.in_service ? (cb.closed ? "合闸" : "分闸") : "停运",
                              ac_conn(cb.bus_from) + " -> " + ac_conn(cb.bus_to));
          const auto& fl = ac_cb_flows[static_cast<int>(i)];
          add_text_metric(row, "状态", cb.closed ? "合闸" : "分闸");
          add_metric(row, "额定电流", cb.i_rated_ka, "kA", "scalar", 3);
          add_metric(row, "Pf", fl.value("pf_mw", 0.0), "MW", "p", 4);
          add_metric(row, "Pt", fl.value("pt_mw", 0.0), "MW", "p", 4);
          add_metric(row, "Qf", fl.value("qf_mvar", 0.0), "MVar", "q", 4);
          add_metric(row, "Qt", fl.value("qt_mvar", 0.0), "MVar", "q", 4);
          if (fl.value("rate_mva", 0.0) > 0.0) {
            add_metric(row, "Loading", fl.value("loading_pct", 0.0), "%", "scalar", 1);
          }
          add_note(row, "断路器潮流由求解后支路匹配或端点KCL反算，用于显示零阻抗合闸断路器的真实通过功率");
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.motors.size(); ++i) {
          const auto& m = sys.ac.motors[i];
          json row = base_row("motor", "AC", m.index, static_cast<int>(i),
                              "电动机", m.name.empty() ? ("Motor " + std::to_string(m.index)) : m.name,
                              is_solved(m.in_service), ac_conn(m.bus));
          add_metric(row, "Sn", m.sn_mva, "MVA", "scalar", 3);
          add_metric(row, "cosPhi", m.cos_phi, "", "scalar", 3);
          add_metric(row, "效率", m.efficiency, "", "scalar", 4);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.chargers.size(); ++i) {
          const auto& ch = sys.ac.chargers[i];
          json row = base_row("charger", "AC", ch.index, static_cast<int>(i),
                              "充电桩", ch.name.empty() ? ("Charger " + std::to_string(ch.index)) : ch.name,
                              is_solved(ch.in_service),
                              "Station " + std::to_string(ch.station_id));
          add_metric(row, "额定功率", ch.p_rated_kw, "kW", "scalar", 3);
          add_metric(row, "最大充电", ch.p_ch_max_kw, "kW", "scalar", 3);
          add_text_metric(row, "V2G", ch.v2g_capable ? "是" : "否");
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.ac.charging_stations.size(); ++i) {
          const auto& cs = sys.ac.charging_stations[i];
          const double p_mw = cs.p_total_kw > 0.0
              ? cs.p_total_kw / 1000.0
              : cs.max_power_kw * cs.utilization_rate * cs.simultaneity_factor / 1000.0;
          json row = base_row("charging_station", "AC", cs.index, static_cast<int>(i),
                              "充电站", cs.name.empty() ? ("EVStation " + std::to_string(cs.index)) : cs.name,
                              is_solved(cs.in_service), ac_conn(cs.bus));
          add_metric(row, "P消耗", cs.in_service ? p_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Q消耗", cs.in_service ? cs.q_total_kvar / 1000.0 : 0.0, "MVar", "q", 4);
          component_results.push_back(std::move(row));
        }

        // DC network/device rows.
        for (size_t i = 0; i < sys.dc.branches.size(); ++i) {
          const auto& br = sys.dc.branches[i];
          double pf_mw = 0.0, pt_mw = 0.0, loss_mw = 0.0, loading = 0.0;
          if (i < geo_dc_branches.size()) {
            pf_mw = geo_dc_branches[i].value("pf_mw", 0.0);
            pt_mw = geo_dc_branches[i].value("pt_mw", 0.0);
            loss_mw = geo_dc_branches[i].value("loss_mw", pf_mw + pt_mw);
            loading = geo_dc_branches[i].value("loading_pct", 0.0);
          }
          json row = base_row("dc_branch", "DC", br.index, static_cast<int>(i),
                              "直流线路", br.name.empty() ? ("DC Line " + std::to_string(br.index)) : br.name,
                              is_solved(br.in_service),
                              dc_conn(br.from_bus) + " -> " + dc_conn(br.to_bus));
          add_metric(row, "Pf", pf_mw, "MW", "p", 4);
          add_metric(row, "Pt", pt_mw, "MW", "p", 4);
          add_metric(row, "Loss", loss_mw, "MW", "p", 4);
          add_metric(row, "Loading", loading, "%", "scalar", 1);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.dc.loads.size(); ++i) {
          const auto& ld = sys.dc.loads[i];
          json row = base_row("dc_load", "DC", ld.index, static_cast<int>(i),
                              "直流负荷", ld.name.empty() ? ("DC Load " + std::to_string(ld.index)) : ld.name,
                              is_solved(ld.in_service), dc_conn(ld.bus));
          add_metric(row, "P消耗", ld.in_service ? ld.p_mw * ld.scaling : 0.0, "MW", "p", 4);
          component_results.push_back(std::move(row));
        }
        json dc_storage_results = json::array();
        for (size_t i = 0; i < sys.dc.storage.size(); ++i) {
          const auto& st = sys.dc.storage[i];
          const double p_solved = st.in_service
              ? solved_dc_storage_p(st.bus, st.p_mw, st.in_service, st.controllable)
              : 0.0;
          const double p_balance_share = p_solved - (st.in_service ? st.p_mw : 0.0);
          json row = base_row("storage", "DC", st.index, static_cast<int>(i),
                              "直流储能", st.name.empty() ? ("DC ESS " + std::to_string(st.index)) : st.name,
                              is_solved(st.in_service), dc_conn(st.bus));
          add_metric(row, "P计算", p_solved, "MW", "p", 4);
          if (std::abs(p_balance_share) > 1e-5) {
            add_metric(row, "平衡分摊", p_balance_share, "MW", "p", 4);
          }
          add_metric(row, "SOC", st.soc_init, "", "scalar", 4);
          add_note(row, std::abs(p_balance_share) > 1e-5
                            ? "正值为放电注入，负值为充电吸收；DC_V平衡功率已投影到该储能"
                            : "正值为放电注入，负值为充电吸收");
          component_results.push_back(std::move(row));
          dc_storage_results.push_back(json{
            {"index", st.index},
            {"position", static_cast<int>(i)},
            {"bus", st.bus},
            {"p_mw", p_solved},
            {"p_scheduled_mw", st.in_service ? st.p_mw : 0.0},
            {"p_balance_share_mw", p_balance_share},
            {"p_rated_mw", st.p_rated_mw},
            {"e_rated_mwh", st.e_rated_mwh},
            {"soc", st.soc_init},
            {"in_service", st.in_service},
            {"controllable", st.controllable},
            {"name", st.name},
            {"source_type", "storage"}
          });
        }
        for (size_t i = 0; i < sys.dc.dc_storage.size(); ++i) {
          const auto& st = sys.dc.dc_storage[i];
          const double p_solved = st.in_service
              ? solved_dc_storage_p(st.bus, st.p_mw, st.in_service, st.controllable)
              : 0.0;
          const double p_balance_share = p_solved - (st.in_service ? st.p_mw : 0.0);
          json row = base_row("dc_storage", "DC", st.index, static_cast<int>(i),
                              "直流储能", st.name.empty() ? ("DC ESS " + std::to_string(st.index)) : st.name,
                              is_solved(st.in_service), dc_conn(st.bus));
          add_metric(row, "P计算", p_solved, "MW", "p", 4);
          if (std::abs(p_balance_share) > 1e-5) {
            add_metric(row, "平衡分摊", p_balance_share, "MW", "p", 4);
          }
          add_metric(row, "SOC", st.soc_init, "", "scalar", 4);
          add_note(row, std::abs(p_balance_share) > 1e-5
                            ? "正值为放电注入，负值为充电吸收；DC_V平衡功率已投影到该储能"
                            : "正值为放电注入，负值为充电吸收");
          component_results.push_back(std::move(row));
          dc_storage_results.push_back(json{
            {"index", st.index},
            {"position", static_cast<int>(i)},
            {"bus", st.bus},
            {"p_mw", p_solved},
            {"p_scheduled_mw", st.in_service ? st.p_mw : 0.0},
            {"p_balance_share_mw", p_balance_share},
            {"p_rated_mw", st.p_rated_mw},
            {"e_rated_mwh", st.e_rated_mwh},
            {"soc", st.soc_init},
            {"in_service", st.in_service},
            {"controllable", st.controllable},
            {"name", st.name},
            {"source_type", "dc_storage"}
          });
        }
        out["dc_storage_results"] = dc_storage_results;
        for (size_t i = 0; i < sys.dc.static_generators.size(); ++i) {
          const auto& sg = sys.dc.static_generators[i];
          json row = base_row("static_generator", "DC", sg.index, static_cast<int>(i),
                              "直流分布式电源", sg.name.empty() ? ("DC SGen " + std::to_string(sg.index)) : sg.name,
                              is_solved(sg.in_service), dc_conn(sg.bus));
          add_metric(row, "P注入", sg.in_service ? sg.p_mw * sg.scaling : 0.0, "MW", "p", 4);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.dc.dc_static_generators.size(); ++i) {
          const auto& sg = sys.dc.dc_static_generators[i];
          json row = base_row("static_generator", "DC", sg.index, static_cast<int>(i),
                              "直流静态电源", sg.name.empty() ? ("DC SGen " + std::to_string(sg.index)) : sg.name,
                              is_solved(sg.in_service), dc_conn(sg.bus));
          add_metric(row, "P注入", sg.in_service ? sg.p_set_mw * sg.scaling : 0.0, "MW", "p", 4);
          add_text_metric(row, "类型", sg.type);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.dc.pv_arrays.size(); ++i) {
          const auto& pv = sys.dc.pv_arrays[i];
          json row = base_row("dc_pv_array", "DC", pv.index, static_cast<int>(i),
                              "直流光伏", pv.name.empty() ? ("DC PV " + std::to_string(pv.index)) : pv.name,
                              is_solved(pv.in_service), dc_conn(pv.bus));
          add_metric(row, "P注入", pv.in_service ? pv.p_set_mw : 0.0, "MW", "p", 4);
          add_metric(row, "辐照度", pv.irradiance, "W/m2", "scalar", 1);
          add_metric(row, "温度", pv.temperature, "degC", "scalar", 1);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.dc.dc_circuit_breakers.size(); ++i) {
          const auto& cb = sys.dc.dc_circuit_breakers[i];
          json row = base_row("circuit_breaker", "DC", cb.index, static_cast<int>(i),
                              "直流断路器", cb.name.empty() ? ("DC CB " + std::to_string(cb.index)) : cb.name,
                              cb.in_service ? (cb.closed ? "合闸" : "分闸") : "停运",
                              dc_conn(cb.bus_from) + " -> " + dc_conn(cb.bus_to));
          const auto& fl = dc_cb_flows[static_cast<int>(i)];
          add_text_metric(row, "状态", cb.closed ? "合闸" : "分闸");
          add_metric(row, "额定电流", cb.i_rated_ka, "kA", "scalar", 3);
          add_metric(row, "Pf", fl.value("pf_mw", 0.0), "MW", "p", 4);
          add_metric(row, "Pt", fl.value("pt_mw", 0.0), "MW", "p", 4);
          if (fl.value("rate_mva", 0.0) > 0.0) {
            add_metric(row, "Loading", fl.value("loading_pct", 0.0), "%", "scalar", 1);
          }
          add_note(row, "直流断路器潮流由求解后DC支路匹配或端点KCL反算");
          component_results.push_back(std::move(row));
        }

        // Converter and higher-level coupling rows.
        // A PQ converter the solver auto-promoted to form its DC island's Vdc
        // reference reports its transfers in VDC_Q mode; surface that so the row
        // mode matches the computed powers instead of the stored PQ setpoint.
        auto vsc_auto_promoted = [&](int conv_index) {
          for (int ci : pf.diagnostics.promoted_vsc_indices) {
            if (ci >= 0 && ci < static_cast<int>(pf.diagnostics.effective_converters.size()) &&
                pf.diagnostics.effective_converters[static_cast<size_t>(ci)].index == conv_index) {
              return true;
            }
          }
          return false;
        };
        for (size_t i = 0; i < sys.vsc_converters.size(); ++i) {
          const auto& c = sys.vsc_converters[i];
          const auto vit = std::find_if(pf.vsc_transfers.begin(), pf.vsc_transfers.end(),
              [&](const hacdcpf::VSCTransfer& v) { return v.index == c.index; });
          const hacdcpf::VSCTransfer* v = vit != pf.vsc_transfers.end() ? &(*vit) : nullptr;
          json row = base_row("vsc_converter", "ACDC", c.index, static_cast<int>(i),
                              "AC/DC变换器", c.name.empty() ? ("VSC " + std::to_string(c.index)) : c.name,
                              c.in_service ? (v ? "已求解" : (pf.converged ? "已参与计算" : "未收敛")) : "停运",
                              ac_conn(c.bus_ac) + " <-> " + dc_conn(c.bus_dc));
          const bool promoted = c.in_service && vsc_auto_promoted(c.index);
          add_text_metric(row, "模式",
                          promoted ? (std::string(converter_mode_str(c.control_mode)) + "→VDC_Q(自动构网)")
                                   : std::string(converter_mode_str(c.control_mode)));
          // Active-power setpoint semantics (multi-converter model §16.3): make
          // explicit whether P is a hard constraint or only a dispatch schedule /
          // initial guess that the DC voltage control releases.
          {
            const bool forms_vdc = promoted ||
                ((c.control_mode == hacdcpf::ConverterMode::VDC_Q ||
                  c.control_mode == hacdcpf::ConverterMode::VDC_VAC) &&
                 std::abs(c.k_vdc) > 1e-9) || c.grid_forming;
            const double p_sched = std::abs(c.p_schedule_mw) > 1e-12 ? c.p_schedule_mw : c.p_set_mw;
            const double p_init = std::abs(c.p_initial_mw) > 1e-12 ? c.p_initial_mw : c.p_set_mw;
            if (c.p_is_hard_constraint && !forms_vdc) {
              add_text_metric(row, "有功语义", "硬约束 P_set");
              add_metric(row, "P设定", c.p_set_mw, "MW", "p", 4);
            } else if (forms_vdc) {
              add_text_metric(row, "有功语义", "构网释放(P自由)");
              add_metric(row, "P调度(非约束)", p_sched, "MW", "p", 4);
              add_metric(row, "P初值", p_init, "MW", "p", 4);
            }
          }
          add_metric(row, "Pac", v ? v->p_ac_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Qac", v ? v->q_ac_mvar : 0.0, "MVar", "q", 4);
          add_metric(row, "Pdc", v ? v->p_dc_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Loss", v ? v->loss_mw : 0.0, "MW", "p", 4);
          if (promoted) {
            add_note(row, "所在直流岛无电压参考，求解器已将其自动升压为VDC_Q并以大增益构造Vdc(释放有功设定，仅作调度/初值)");
          }
          if (c.control_mode == hacdcpf::ConverterMode::VDC_Q ||
              c.control_mode == hacdcpf::ConverterMode::VDC_VAC || promoted) {
            add_note(row, "VDC类模式下P调度不是AC侧有功等式约束；潮流AC侧采用求解得到的Pac，DC侧采用求解得到的Pdc并计入损耗");
          }
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.dc.dcdc_converters.size(); ++i) {
          const auto& c = sys.dc.dcdc_converters[i];
          const auto dit = std::find_if(pf.dcdc_transfers.begin(), pf.dcdc_transfers.end(),
              [&](const hacdcpf::DCDCTransfer& d) { return d.index == c.index; });
          const hacdcpf::DCDCTransfer* d = dit != pf.dcdc_transfers.end() ? &(*dit) : nullptr;
          json row = base_row("dcdc_converter", "DC", c.index, static_cast<int>(i),
                              "DC/DC变换器", c.name.empty() ? ("DCDC " + std::to_string(c.index)) : c.name,
                              c.in_service ? (d ? "已求解" : (pf.converged ? "已参与计算" : "未收敛")) : "停运",
                              dc_conn(c.bus_in) + " -> " + dc_conn(c.bus_out));
          add_text_metric(row, "模式", dcdc_control_str(c.control_mode));
          add_metric(row, "Pin", d ? d->p_in_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Pout", d ? d->p_out_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Loss", d ? d->loss_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Vref", c.v_ref_pu, "pu", "scalar", 4);
          // Power-stage topology + duty-ratio feasibility (multi-converter §3.2).
          if (c.topology != hacdcpf::DCDCTopology::Generic) {
            add_text_metric(row, "拓扑", dcdc_topology_str(c.topology));
            if (d && d->duty_defined) {
              add_metric(row, c.topology == hacdcpf::DCDCTopology::Isolated ? "调制增益M" : "占空比D",
                         d->duty, "", "scalar", 4);
              add_metric(row, "电压比", d->voltage_ratio, "", "scalar", 4);
              if (!d->duty_feasible) {
                add_note(row, "占空比/增益超出[" + std::to_string(c.d_min) + ", " +
                                  std::to_string(c.d_max) + "]可行域，该电压变换不可行(DCDC-PHYS-01)");
              }
            }
          }
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < er_snapshot.size(); ++i) {
          const auto& er = er_snapshot[i];
          const auto eit = std::find_if(geo_er.begin(), geo_er.end(),
              [&](const json& item) { return item.value("router_index", -1) == er.index; });
          json row = base_row("energy_router", "ACDC", er.index, static_cast<int>(i),
                              "能量路由器", er.name.empty() ? ("ER " + std::to_string(er.index)) : er.name,
                              is_solved(er.in_service), "multi-port");
          if (eit != geo_er.end() && eit->contains("ports")) {
            for (const auto& pt : (*eit)["ports"]) {
              const std::string base_label =
                  "Port " + std::to_string(pt.value("port_index", 0));
              add_metric(row, base_label + " P", pt.value("p_mw", 0.0), "MW", "p", 4);
              add_metric(row, base_label + " Q", pt.value("q_mvar", 0.0), "MVar", "q", 4);
              add_metric(row, base_label + " V", pt.value("v_pu", 1.0), "pu", "scalar", 4);
            }
            add_metric(row, "Port Psum", eit->value("p_port_sum_mw", 0.0), "MW", "p", 4);
            add_metric(row, "DCDC Pin", eit->value("internal_dcdc_pin_mw", 0.0), "MW", "p", 4);
            add_metric(row, "DCDC Pout", eit->value("internal_dcdc_pout_mw", 0.0), "MW", "p", 4);
            add_metric(row, "DCDC Loss", eit->value("internal_dcdc_loss_mw", 0.0), "MW", "p", 4);
            add_metric(row, "VSC Loss", eit->value("vsc_loss_mw", 0.0), "MW", "p", 4);
            add_metric(row, "Loss", eit->value("loss_mw", 0.0), "MW", "p", 4);
            add_note(row, "能量路由器结果由展开后的端口VSC与内部DC/DC潮流按索引回填");
          }
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.mobile_storage.size(); ++i) {
          const auto& ms = sys.mobile_storage[i];
          json row = base_row("mobile_storage", "AC", ms.index, static_cast<int>(i),
                              "移动储能", ms.name.empty() ? ("Mobile ESS " + std::to_string(ms.index)) : ms.name,
                              is_solved(ms.in_service), ac_conn(ms.bus));
          add_metric(row, "P计算", ms.in_service ? ms.p_mw : 0.0, "MW", "p", 4);
          add_metric(row, "Q计算", ms.in_service ? ms.q_mvar : 0.0, "MVar", "q", 4);
          add_metric(row, "SOC", ms.soc_init, "", "scalar", 4);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.vpps.size(); ++i) {
          const auto& vpp = sys.vpps[i];
          json row = base_row("vpp", "AC", vpp.index, static_cast<int>(i),
                              "虚拟电厂", vpp.name.empty() ? ("VPP " + std::to_string(vpp.index)) : vpp.name,
                              is_solved(vpp.in_service), ac_conn(vpp.pcc_bus));
          add_metric(row, "P输出", vpp.p_output_mw, "MW", "p", 4);
          add_metric(row, "Q输出", vpp.q_output_mvar, "MVar", "q", 4);
          component_results.push_back(std::move(row));
        }
        for (size_t i = 0; i < sys.microgrids.size(); ++i) {
          const auto& mg = sys.microgrids[i];
          json row = base_row("microgrid", "AC", mg.index, static_cast<int>(i),
                              "微电网", mg.name.empty() ? ("MicroGrid " + std::to_string(mg.index)) : mg.name,
                              is_solved(mg.in_service), ac_conn(mg.pcc_bus));
          add_text_metric(row, "模式", microgrid_mode_str(mg.operating_mode));
          add_metric(row, "交换P", mg.p_exchange_mw, "MW", "p", 4);
          add_metric(row, "容量", mg.capacity_mw, "MW", "p", 4);
          component_results.push_back(std::move(row));
        }

        out["component_results"] = component_results;
      };

      if (method == "ac_newton") {
        // Island detection: dead islands (no generation source) are
        // automatically stripped during canonical projection
        // (strip_dead_islands in project_in_place), so the monolithic
        // solver always receives a system free of dead nodes.
        // We only need the adaptive solver when multiple *solvable*
        // islands exist (each needing its own slack bus).
        auto islands = hacdcpf::powerflow::detect_islands(sys);
        int solvable_islands = 0;
        for (const auto& isle : islands) {
          if (isle.has_generators) ++solvable_islands;
        }
        out["islands_detected"] = static_cast<int>(islands.size());
        out["solvable_islands"] = solvable_islands;

        if (solvable_islands > 1) {
          // Multiple solvable islands: use adaptive solver which
          // extracts and solves each island independently.
          auto pf = hacdcpf::solve_power_flow_adaptive(sys, opt);
          out["converged"] = pf.converged;
          out["iterations"] = pf.iterations;
          out["residual"] = pf.residual;
          out["vm"] = pf.vm;
          out["va"] = pf.va;
          out["vdc"] = pf.vdc;
          for (const auto& bf : pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
          out["method_actual"] = "adaptive (auto island)";
          auto ac_pf = hacdcpf::solve_power_flow(sys, opt);
          if (ac_pf.converged) {
            add_transfers(ac_pf);
            add_geo_data(ac_pf);
            std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = method;
          }
        } else {
          auto pf = hacdcpf::solve_power_flow(sys, opt);
          out["converged"] = pf.converged;
          out["iterations"] = pf.iterations;
          out["residual"] = pf.residual;
          out["vm"] = pf.vm;
          out["va"] = pf.va;
          out["vdc"] = pf.vdc;
          for (const auto& bf : pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
          // Set descriptive method name: unified AC/DC when DC components exist
          const bool has_dc = !sys.dc.buses.empty() || !sys.vsc_converters.empty() || !sys.dc.dcdc_converters.empty();
          out["method_actual"] = has_dc ? "hybrid_ac_dc_newton" : "ac_newton";
          if (pf.converged) {
            add_transfers(pf);
            add_geo_data(pf);
            store_last_pf(&pf);
          } else {
            add_solver_diagnostics(pf.diagnostics);
          }
          add_converter_coordination(pf);
        }
      } else if (method == "pure_ac") {
        // Pure AC Newton-Raphson: strip DC buses, DC branches, converters
        // so the Jacobian and mismatch only contain AC equations.
        hacdcpf::HybridPowerSystem ac_sys = sys;
        ac_sys.dc.buses.clear();
        ac_sys.dc.branches.clear();
        ac_sys.dc.loads.clear();
        ac_sys.dc.storage.clear();
        ac_sys.dc.static_generators.clear();
        ac_sys.dc.dc_static_generators.clear();
        ac_sys.dc.pv_arrays.clear();
        ac_sys.dc.dc_circuit_breakers.clear();
        ac_sys.vsc_converters.clear();
        ac_sys.dc.dcdc_converters.clear();
        auto pf = hacdcpf::solve_power_flow(ac_sys, opt);
        out["converged"] = pf.converged;
        out["iterations"] = pf.iterations;
        out["residual"] = pf.residual;
        out["vm"] = pf.vm;
        out["va"] = pf.va;
        out["vdc"] = json::array();
        for (const auto& bf : pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
        add_geo_data(pf);
        { std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = pf; g_session.last_pf_method = method; }
      } else if (method == "dc") {
        auto pf = hacdcpf::solve_dc_power_flow(sys, opt);
        out["converged"] = pf.converged;
        out["iterations"] = pf.iterations;
        out["residual"] = pf.residual;
        out["vdc"] = pf.vdc;
        // Run AC PF to get branch flows and converter transfers for carbon analysis
        auto ac_pf = hacdcpf::solve_power_flow(sys);
        if (ac_pf.converged) {
          add_transfers(ac_pf);
          add_geo_data(ac_pf);
          for (const auto& bf : ac_pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
          out["vm"] = ac_pf.vm;
          out["va"] = ac_pf.va;
          std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = method;
        }
      } else if (method == "hybrid_linearized") {
        auto pf = hacdcpf::solve_ac_dc_power_flow(sys, opt);
        out["converged"] = pf.success;
        out["iterations"] = 1;
        out["residual"] = 0.0;
        out["va"] = pf.va;
        for (const auto& p : pf.pf_mw) out["branch_abs"].push_back(std::abs(p));
        // Run AC PF to get converter transfers for carbon analysis
        auto ac_pf = hacdcpf::solve_power_flow(sys);
        if (ac_pf.converged) {
          add_transfers(ac_pf);
          add_geo_data(ac_pf);
          out["vm"] = ac_pf.vm;
          out["vdc"] = ac_pf.vdc;
          std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = method;
        }
      } else if (method == "fdpf") {
        auto pf = hacdcpf::solve_power_flow_fdpf(sys, opt);
        out["converged"] = pf.converged;
        out["iterations"] = pf.iterations;
        out["residual"] = pf.residual;
        out["vm"] = pf.vm;
        out["va"] = pf.va;
        out["vdc"] = pf.vdc;
        for (const auto& bf : pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
        add_transfers(pf);
        add_geo_data(pf);
        { std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = pf; g_session.last_pf_method = method; }
      } else if (method == "adaptive") {
        auto pf = hacdcpf::solve_power_flow_adaptive(sys, opt);
        out["converged"] = pf.converged;
        out["iterations"] = pf.iterations;
        out["residual"] = pf.residual;
        out["vm"] = pf.vm;
        out["va"] = pf.va;
        out["vdc"] = pf.vdc;
        for (const auto& bf : pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
        // Run AC PF to get converter transfers
        if (pf.converged) {
          auto ac_pf = hacdcpf::solve_power_flow(sys, opt);
          if (ac_pf.converged) {
            add_transfers(ac_pf);
            add_geo_data(ac_pf);
            std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = method;
          } else {
            hacdcpf::PowerFlowResult pfr; pfr.vm = pf.vm; pfr.va = pf.va; pfr.vdc = pf.vdc;
            pfr.converged = pf.converged; pfr.iterations = pf.iterations; pfr.residual = pf.residual;
            pfr.branch_flows = pf.branch_flows;
            add_geo_data(pfr);
            std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = pfr; g_session.last_pf_method = method;
          }
        }
      } else if (method == "islanded") {
        auto pf = hacdcpf::solve_power_flow_islanded(sys, opt);
        out["converged"] = pf.converged;
        out["iterations"] = pf.iterations;
        out["residual"] = pf.residual;
        out["vm"] = pf.vm;
        out["va"] = pf.va;
        out["vdc"] = pf.vdc;
        // Run AC PF to get branch flows and converter transfers
        if (pf.converged) {
          auto ac_pf = hacdcpf::solve_power_flow(sys, opt);
          if (ac_pf.converged) {
            add_transfers(ac_pf);
            add_geo_data(ac_pf);
            for (const auto& bf : ac_pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
            std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = method;
          } else {
            hacdcpf::PowerFlowResult pfr; pfr.vm = pf.vm; pfr.va = pf.va; pfr.vdc = pf.vdc;
            pfr.converged = pf.converged; pfr.iterations = pf.iterations; pfr.residual = pf.residual;
            add_geo_data(pfr);
            std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = pfr; g_session.last_pf_method = method;
          }
        }
      } else if (method == "distributed_slack") {
        hacdcpf::DistributedSlack slack;
        std::map<int, bool> seen;
        for (const auto& g : sys.ac.generators) {
          if (!g.in_service) continue;
          if (!seen[g.bus]) {
            slack.participating_buses.push_back(g.bus);
            seen[g.bus] = true;
          }
        }
        if (slack.participating_buses.empty() && !sys.ac.buses.empty()) {
          slack.participating_buses.push_back(sys.ac.buses.front().index);
        }
        if (slack.participating_buses.empty()) {
          throw std::runtime_error("No AC buses available for distributed slack");
        }
        slack.reference_bus = slack.participating_buses.front();
        const double w = 1.0 / static_cast<double>(slack.participating_buses.size());
        slack.participation_factors.assign(slack.participating_buses.size(), w);

        auto pf = hacdcpf::solve_power_flow_distributed_slack_full(sys, slack, opt);
        out["converged"] = pf.converged;
        out["iterations"] = pf.iterations;
        out["residual"] = pf.residual;
        out["vm"] = pf.vm;
        out["va"] = pf.va;
        out["vdc"] = pf.vdc;
        add_converter_coordination_report(pf.diagnostics.converter_coordination);
        // Run AC PF to get converter transfers for carbon analysis
        if (pf.converged) {
          auto ac_pf = hacdcpf::solve_power_flow(sys, opt);
          if (ac_pf.converged) {
            add_transfers(ac_pf);
            add_geo_data(ac_pf);
            for (const auto& bf : ac_pf.branch_flows) out["branch_abs"].push_back(std::abs(bf.pf_mw));
            store_last_pf(&ac_pf);
          } else {
            store_last_pf(nullptr);
          }
        } else {
          add_solver_diagnostics(pf.diagnostics);
        }
      } else if (method == "three_phase") {
        if (!sys.three_phase_ac.has_value()) {
          throw std::runtime_error("Three-phase subsystem not available in this case");
        }
        hacdcpf::powerflow::ThreePhaseFlowOptions tp_opt;
        tp_opt.max_iter = opt.max_iter;
        tp_opt.tol = opt.tol;
        auto pf = hacdcpf::powerflow::solve_three_phase(*sys.three_phase_ac, tp_opt);
        out["converged"] = pf.converged;
        out["iterations"] = pf.iterations;
        out["residual"] = pf.residual;
        json vm = json::array();
        json vm_a = json::array();
        json vm_b = json::array();
        json vm_c = json::array();
        for (const auto& br : pf.bus_results) {
          vm_a.push_back(br.vm_a_pu);
          vm_b.push_back(br.vm_b_pu);
          vm_c.push_back(br.vm_c_pu);
          vm.push_back((br.vm_a_pu + br.vm_b_pu + br.vm_c_pu) / 3.0);
        }
        out["vm"] = vm;
        out["vm_a"] = vm_a;
        out["vm_b"] = vm_b;
        out["vm_c"] = vm_c;
      } else {
        throw std::runtime_error("Unsupported power flow method: " + method);
      }

      add_dc_branch_flows();
      // Surface solver diagnostics (e.g. auto-promoted converters, Vdc-limit
      // warnings) so the GUI can show why a DC island balanced the way it did.
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (g_session.last_pf_result) {
          json warns = json::array();
          for (const auto& w : g_session.last_pf_result->diagnostics.warnings) warns.push_back(w);
          out["warnings"] = warns;
          out["promoted_vsc_indices"] = g_session.last_pf_result->diagnostics.promoted_vsc_indices;
          if (!out.contains("converter_coordination")) {
            const auto& rep = g_session.last_pf_result->diagnostics.converter_coordination;
            json coord;
            coord["enabled"] = rep.enabled;
            coord["feasible"] = rep.feasible;
            coord["blocking_count"] = rep.blocking_count();
            coord["fatal_count"] = rep.fatal_count();
            coord["error_count"] = rep.error_count();
            coord["warning_count"] = rep.warning_count();
            coord["issues"] = json::array();
            for (const auto& issue : rep.issues) {
              coord["issues"].push_back(json{
                {"severity", hacdcpf::powerflow::coordination_severity_str(issue.severity)},
                {"rule_id", issue.rule_id},
                {"component_type", issue.component_type},
                {"component_index", issue.component_index},
                {"island_index", issue.island_index},
                {"message", issue.message}
              });
            }
            coord["dc_islands"] = json::array();
            for (const auto& isle : rep.dc_islands) {
              coord["dc_islands"].push_back(json{
                {"island_index", isle.island_index},
                {"dc_buses", isle.dc_buses},
                {"declared_v_buses", isle.declared_v_buses},
                {"hard_vdc_sources", isle.hard_vdc_sources},
                {"droop_sources", isle.droop_sources},
                {"fixed_power_devices", isle.fixed_power_devices},
                {"fixed_power_mw", isle.fixed_power_mw},
                {"flexible_up_mw", isle.flexible_up_mw},
                {"flexible_down_mw", isle.flexible_down_mw}
              });
              auto& island_json = coord["dc_islands"].back();
              island_json["voltage_sources"] = json::array();
              for (const auto& source : isle.voltage_sources) {
                island_json["voltage_sources"].push_back(json{
                  {"component_type", source.component_type},
                  {"component_index", source.component_index},
                  {"bus", source.bus},
                  {"v_set_pu", source.v_set_pu},
                  {"has_v_set", source.has_v_set},
                  {"droop", source.droop}
                });
              }
            }
            out["converter_coordination"] = coord;
          }
        }
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Session: topology analysis (graph structure) ----
  // Read-only structural analysis of the network graph: island / connectivity
  // detection, radiality, bridges (cut-edges), articulation points (cut-vertices)
  // and per-island validity diagnostics.  Backed by hacdcpf::graph.
  svr.Post("/api/session/topology",
           [](const httplib::Request&, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);

      namespace gr = hacdcpf::graph;
      const gr::PowerSystemGraph graph = gr::build_power_system_graph(sys);
      const gr::TopologyReport report = gr::analyze_topology(graph);

      // ── Enum → string helpers ──────────────────────────────────────
      auto island_status_str = [](gr::IslandStatus s) -> const char* {
        switch (s) {
          case gr::IslandStatus::Valid:          return "Valid";
          case gr::IslandStatus::NoSlack:        return "NoSlack";
          case gr::IslandStatus::NoDCVoltageRef: return "NoDCVoltageRef";
          case gr::IslandStatus::IsolatedLoad:   return "IsolatedLoad";
          case gr::IslandStatus::Empty:          return "Empty";
        }
        return "Unknown";
      };
      auto edge_cat_str = [](gr::EdgeCategory c) -> const char* {
        switch (c) {
          case gr::EdgeCategory::AC_Line:        return "AC_Line";
          case gr::EdgeCategory::AC_Transformer: return "AC_Transformer";
          case gr::EdgeCategory::Switch:         return "Switch";
          case gr::EdgeCategory::Breaker:        return "Breaker";
          case gr::EdgeCategory::DC_Line:        return "DC_Line";
          case gr::EdgeCategory::DC_Switch:      return "DC_Switch";
          case gr::EdgeCategory::VSC_Coupling:   return "VSC_Coupling";
          case gr::EdgeCategory::DCDC_Coupling:  return "DCDC_Coupling";
        }
        return "Unknown";
      };

      // ── model bus_id → canvas position index (matches frontend ordering:
      //    AC buses in sys.ac.buses order, DC buses in sys.dc.buses order) ──
      std::unordered_map<int,int> ac_id_to_pos, dc_id_to_pos;
      for (size_t i = 0; i < sys.ac.buses.size(); ++i) ac_id_to_pos[sys.ac.buses[i].index] = static_cast<int>(i);
      for (size_t i = 0; i < sys.dc.buses.size(); ++i) dc_id_to_pos[sys.dc.buses[i].index] = static_cast<int>(i);

      json out;
      out["is_connected"]      = report.is_connected;
      out["is_radial"]         = report.is_radial;
      out["cycle_count"]       = report.cycle_count;
      out["n_ac_islands"]      = report.n_ac_islands;
      out["n_dc_islands"]      = report.n_dc_islands;
      out["all_islands_valid"] = report.all_islands_valid;
      out["n_buses"]           = graph.node_count();
      out["n_branches"]        = graph.edge_count();

      // ── Islands (+ per-bus island id arrays keyed by canvas position) ──
      json islands = json::array();
      std::vector<int> ac_bus_island(sys.ac.buses.size(), -1);
      std::vector<int> dc_bus_island(sys.dc.buses.size(), -1);
      for (const auto& isl : report.islands) {
        json ji;
        ji["island_id"]          = isl.island_id;
        ji["domain"]             = (isl.domain == gr::NodeDomain::DC) ? "DC" : "AC";
        ji["status"]             = island_status_str(isl.status);
        ji["has_ac_slack"]       = isl.has_ac_slack;
        ji["has_dc_voltage_ref"] = isl.has_dc_voltage_ref;
        ji["n_buses"]            = static_cast<int>(isl.ac_bus_ids.size() + isl.dc_bus_ids.size());
        ji["ac_bus_ids"]         = isl.ac_bus_ids;
        ji["dc_bus_ids"]         = isl.dc_bus_ids;
        islands.push_back(ji);
        for (int bid : isl.ac_bus_ids) {
          auto it = ac_id_to_pos.find(bid);
          if (it != ac_id_to_pos.end()) ac_bus_island[it->second] = isl.island_id;
        }
        for (int bid : isl.dc_bus_ids) {
          auto it = dc_id_to_pos.find(bid);
          if (it != dc_id_to_pos.end()) dc_bus_island[it->second] = isl.island_id;
        }
      }
      out["islands"]       = islands;
      out["ac_bus_island"] = ac_bus_island;
      out["dc_bus_island"] = dc_bus_island;

      // ── Cut vertices (articulation points) ──────────────────────────
      out["cut_vertex_bus_ids"] = report.cut_vertex_bus_ids;
      std::vector<bool> ac_cut(sys.ac.buses.size(), false);
      std::vector<bool> dc_cut(sys.dc.buses.size(), false);
      for (int bid : report.cut_vertex_bus_ids) {
        auto a = ac_id_to_pos.find(bid);
        if (a != ac_id_to_pos.end()) ac_cut[a->second] = true;
        auto d = dc_id_to_pos.find(bid);
        if (d != dc_id_to_pos.end()) dc_cut[d->second] = true;
      }
      out["ac_cut_vertex"] = ac_cut;
      out["dc_cut_vertex"] = dc_cut;

      // ── Bridges (cut-edges) ─────────────────────────────────────────
      json bridges = json::array();
      for (int eid : report.bridge_edge_ids) {
        if (eid < 0 || eid >= graph.edge_count()) continue;
        const auto& e = graph.edges[eid];
        const bool dc_edge = (e.category == gr::EdgeCategory::DC_Line ||
                              e.category == gr::EdgeCategory::DC_Switch);
        json jb;
        jb["from_bus"] = e.from_bus_id;
        jb["to_bus"]   = e.to_bus_id;
        jb["category"] = edge_cat_str(e.category);
        jb["domain"]   = dc_edge ? "DC" : "AC";
        // Canvas positions (-1 when the endpoint is not in the matching domain map)
        auto& fmap = dc_edge ? dc_id_to_pos : ac_id_to_pos;
        auto& tmap = dc_edge ? dc_id_to_pos : ac_id_to_pos;
        auto fit = fmap.find(e.from_bus_id);
        auto tit = tmap.find(e.to_bus_id);
        jb["from_pos"] = (fit != fmap.end()) ? fit->second : -1;
        jb["to_pos"]   = (tit != tmap.end()) ? tit->second : -1;
        bridges.push_back(jb);
      }
      out["bridges"] = bridges;

      // ── Diagnostics ─────────────────────────────────────────────────
      json diags = json::array();
      for (const auto& d : report.diagnostics) {
        diags.push_back(json{
          {"code", static_cast<int>(d.code)},
          {"message", d.message},
          {"related_buses", d.related_buses},
          {"related_branches", d.related_branches},
        });
      }
      out["diagnostics"] = diags;

      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ──────────────────────────────────────────────────────────────────
  // Network reduction (graph reduction pipeline — before/after view).
  // Runs switch contraction → series reduction → (optional) pendant folding
  // and reports, for every original bus, what it collapses into.  Kron-
  // eligible passive nodes are identified (count only — Kron operates on the
  // Y-bus, not the system model, so it is not collapsed in this view).
  // ──────────────────────────────────────────────────────────────────
  svr.Post("/api/session/network_reduction",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      json body = req.body.empty() ? json::object() : json::parse(req.body);
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);

      namespace gr = hacdcpf::graph;
      const bool en_switch  = body.value("enable_switch_contraction", true);
      const bool en_series  = body.value("enable_series_reduction",   true);
      const bool en_pendant = body.value("enable_pendant_reduction",  false);
      const bool en_kron    = body.value("enable_kron_reduction",     false);
      // Whether the switch-contraction stage should also merge zero-impedance
      // LINES (not just closed switches/breakers).  Default OFF: a plain line
      // with ~zero impedance — e.g. a DC line the user left at r=0 — must NOT
      // silently collapse its two buses when the system contains no switch or
      // breaker.  Opt in via the "合并零阻抗线路" checkbox to recover the old
      // behaviour (needed only to de-singularise genuinely zero-impedance ties).
      const bool en_zeroz_lines = body.value("contract_zero_impedance_lines", false);

      auto dom_str = [](gr::NodeDomain d) -> const char* {
        return d == gr::NodeDomain::DC ? "DC" : "AC";
      };
      // Reducers mark eliminated nodes/edges as in_service=false rather than
      // erasing them, so count live (in-service) elements for honest before/after.
      auto count_live = [](const gr::PowerSystemGraph& g, int& nb, int& ne) {
        nb = 0; ne = 0;
        for (const auto& n : g.nodes) if (n.in_service) ++nb;
        for (const auto& e : g.edges) if (e.in_service) ++ne;
      };

      gr::GraphReductionOptions opts;
      opts.enable_switch_contraction = en_switch;
      opts.enable_series_reduction   = en_series;
      opts.enable_pendant_reduction  = en_pendant;
      opts.enable_kron_reduction     = en_kron;

      // Original graph (BEFORE).
      const gr::PowerSystemGraph graph0 = gr::build_power_system_graph(sys);
      int before_buses = 0, before_branches = 0;
      count_live(graph0, before_buses, before_branches);

      // Composition state in ORIGINAL-bus-id space, domain-qualified.
      // status: 0 retained, 1 merged (contraction), 2 series, 3 pendant.
      std::unordered_map<int,int> ac_rep, dc_rep, ac_status, dc_status;
      for (const auto& b : sys.ac.buses) { ac_rep[b.index] = b.index; ac_status[b.index] = 0; }
      for (const auto& b : sys.dc.buses) { dc_rep[b.index] = b.index; dc_status[b.index] = 0; }

      gr::PowerSystemGraph  work_graph = graph0;
      hacdcpf::HybridPowerSystem work_sys = sys;

      std::vector<gr::Diagnostic> all_diags;
      json switch_groups = json::array();
      int  n_buses_merged = 0;

      // ── Stage 1: switch contraction (zero-impedance / closed switches) ──
      if (en_switch) {
        gr::ContractionOptions copt;
        copt.zero_impedance_threshold = opts.zero_impedance_threshold;
        copt.voltage_base_tolerance   = opts.voltage_base_tolerance;
        // Closed switches/breakers always contract (that is what this stage is
        // named for); zero-impedance LINES only when the caller opts in.
        copt.contract_zero_impedance_lines = en_zeroz_lines;
        gr::ContractionResult cr = gr::contract_zero_impedance_edges(work_graph, work_sys, copt);
        auto record_groups = [&](const std::unordered_map<int,std::vector<int>>& super_to_buses,
                                 gr::NodeDomain dom,
                                 std::unordered_map<int,int>& rep,
                                 std::unordered_map<int,int>& status) {
          for (const auto& [super, buses] : super_to_buses) {
            if (buses.size() <= 1) continue;
            switch_groups.push_back(json{
              {"super_bus_id", super}, {"domain", dom_str(dom)}, {"bus_ids", buses}});
            for (int b : buses) {
              rep[b] = super;
              if (b != super) { status[b] = 1; ++n_buses_merged; }
            }
          }
        };
        record_groups(cr.ac_super_to_buses, gr::NodeDomain::AC, ac_rep, ac_status);
        record_groups(cr.dc_super_to_buses, gr::NodeDomain::DC, dc_rep, dc_status);
        for (const auto& d : cr.diagnostics) all_diags.push_back(d);
        work_graph = cr.contracted_graph;
        work_sys   = cr.contracted_system;
      }

      // ── Stage 2: series reduction (passive degree-2 elimination) ──
      json series_records = json::array();
      if (en_series) {
        gr::ReductionCandidates cand = gr::classify_reduction_candidates(work_graph, work_sys, opts);
        gr::ReductionPlan       plan = gr::make_reduction_plan(work_graph, cand, opts);
        gr::SeriesReductionResult sr = gr::apply_series_reduction(work_graph, work_sys, plan, opts);
        for (const auto& rec : sr.mapping.series_records) {
          series_records.push_back(json{
            {"eliminated_bus_id", rec.eliminated_bus_id},
            {"from_bus_id", rec.from_bus_id}, {"to_bus_id", rec.to_bus_id},
            {"domain", dom_str(rec.domain)}, {"r_eq", rec.r_eq}, {"x_eq", rec.x_eq},
            {"method", "series_impedance_sum"}, {"fidelity", "exact_for_passive_zero_injection_no_shunt"}});
          auto& rep    = (rec.domain == gr::NodeDomain::DC) ? dc_rep : ac_rep;
          auto& status = (rec.domain == gr::NodeDomain::DC) ? dc_status : ac_status;
          for (auto& [b, r] : rep)
            if (r == rec.eliminated_bus_id) { status[b] = 2; r = rec.from_bus_id; }
        }
        for (const auto& d : sr.diagnostics) all_diags.push_back(d);
        work_graph = sr.reduced_graph;
        work_sys   = sr.reduced_system;
      }

      // ── Stage 3: pendant folding (leaf load nodes; approximate) ──
      json pendant_records = json::array();
      if (en_pendant) {
        gr::ReductionCandidates cand = gr::classify_reduction_candidates(work_graph, work_sys, opts);
        gr::ReductionPlan       plan = gr::make_reduction_plan(work_graph, cand, opts);
        gr::PendantReductionResult pr = gr::apply_pendant_reduction(work_graph, work_sys, plan, opts);
        for (const auto& rec : pr.mapping.pendant_records) {
          pendant_records.push_back(json{
            {"eliminated_bus_id", rec.eliminated_bus_id},
            {"parent_bus_id", rec.parent_bus_id}, {"domain", dom_str(rec.domain)},
            {"method", "flat_voltage_load_fold"}, {"fidelity", "approximate"}});
          auto& rep    = (rec.domain == gr::NodeDomain::DC) ? dc_rep : ac_rep;
          auto& status = (rec.domain == gr::NodeDomain::DC) ? dc_status : ac_status;
          for (auto& [b, r] : rep)
            if (r == rec.eliminated_bus_id) { status[b] = 3; r = rec.parent_bus_id; }
        }
        for (const auto& d : pr.diagnostics) all_diags.push_back(d);
        work_graph = pr.reduced_graph;
        work_sys   = pr.reduced_system;
      }

      // Resolve representative chains (a series 'from' bus may be folded later).
      auto resolve = [](std::unordered_map<int,int>& rep) {
        for (auto& [b, r] : rep) {
          int cur = r, guard = 0;
          while (rep.count(cur) && rep[cur] != cur && guard++ < 100000) cur = rep[cur];
          r = cur;
        }
      };
      resolve(ac_rep);
      resolve(dc_rep);

      // ── Kron-eligible passive interior nodes (identify only) ──
      int n_kron_candidates = 0;
      {
        gr::ReductionCandidates cand = gr::classify_reduction_candidates(work_graph, work_sys, opts);
        for (const auto& c : cand.candidates)
          if (c.type == gr::CandidateType::ZeroInjectionPassiveInterior ||
              c.type == gr::CandidateType::KronPassiveNode) ++n_kron_candidates;
      }

      int after_buses = 0, after_branches = 0;
      count_live(work_graph, after_buses, after_branches);

      // bus_id → canvas position (AC buses then DC buses, matching frontend).
      std::unordered_map<int,int> ac_id_to_pos, dc_id_to_pos;
      for (size_t i = 0; i < sys.ac.buses.size(); ++i) ac_id_to_pos[sys.ac.buses[i].index] = static_cast<int>(i);
      for (size_t i = 0; i < sys.dc.buses.size(); ++i) dc_id_to_pos[sys.dc.buses[i].index] = static_cast<int>(i);

      auto status_str = [](int s) -> const char* {
        switch (s) {
          case 1: return "merged";
          case 2: return "series_eliminated";
          case 3: return "pendant_eliminated";
          default: return "retained";
        }
      };
      auto bus_reduction = [&](size_t n, bool dc_domain) {
        auto& rep       = dc_domain ? dc_rep : ac_rep;
        auto& status    = dc_domain ? dc_status : ac_status;
        auto& id_to_pos = dc_domain ? dc_id_to_pos : ac_id_to_pos;
        json arr = json::array();
        for (size_t i = 0; i < n; ++i) {
          int bid = dc_domain ? sys.dc.buses[i].index : sys.ac.buses[i].index;
          int st  = status.count(bid) ? status[bid] : 0;
          int rp  = rep.count(bid) ? rep[bid] : bid;
          auto pit = id_to_pos.find(rp);
          arr.push_back(json{
            {"bus_id", bid}, {"pos", static_cast<int>(i)},
            {"status", status_str(st)}, {"rep_bus_id", rp},
            {"rep_pos", pit != id_to_pos.end() ? pit->second : -1}});
        }
        return arr;
      };

      json out;
      out["before"] = json{{"n_buses", before_buses}, {"n_branches", before_branches}};
      out["after"]  = json{{"n_buses", after_buses},  {"n_branches", after_branches}};
      out["n_buses_eliminated"]    = before_buses - after_buses;
      out["n_branches_eliminated"] = before_branches - after_branches;
      out["reduction_pct_buses"]   = before_buses > 0
        ? 100.0 * (before_buses - after_buses) / before_buses : 0.0;
      out["stages"] = json{
        {"switch_contraction", json{{"enabled", en_switch}, {"n_groups", switch_groups.size()},
                                    {"n_buses_merged", n_buses_merged},
                                    {"fidelity", "exact_voltage_equality"}}},
        {"series_reduction",   json{{"enabled", en_series},  {"n_eliminated", series_records.size()},
                                    {"fidelity", "exact_for_passive_zero_injection_no_shunt"}}},
        {"pendant_reduction",  json{{"enabled", en_pendant}, {"n_eliminated", pendant_records.size()},
                                    {"fidelity", "approximate_flat_voltage_loss_fold"}}},
        {"kron_identify",      json{{"enabled", en_kron},    {"n_candidates", n_kron_candidates},
                                    {"fidelity", "identified_only_not_applied"}}},
      };
      out["formulation_notes"] = json{
        "switch_contraction uses V_i = V_super for closed-switch/zero-impedance supernodes",
        "series reduction is applied only to passive degree-2 line nodes and uses Z_eq = Z_ij + Z_jk",
        "pendant folding is approximate and uses |V_parent| = 1.0 pu for folded load-loss accounting",
        "Kron candidates are reported only; the GUI endpoint does not collapse them into the exported network view"
      };
      out["switch_groups"]   = switch_groups;
      out["series_records"]  = series_records;
      out["pendant_records"] = pendant_records;
      out["ac_bus_reduction"] = bus_reduction(sys.ac.buses.size(), false);
      out["dc_bus_reduction"] = bus_reduction(sys.dc.buses.size(), true);

      json diags = json::array();
      for (const auto& d : all_diags)
        diags.push_back(json{{"code", static_cast<int>(d.code)}, {"message", d.message},
                             {"related_buses", d.related_buses}, {"related_branches", d.related_branches}});
      out["diagnostics"] = diags;

      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // Unified OPF endpoint: choose solver (ac/parity/dc) and which constraint
  // families to enforce (branch / converter capacity / current / modulation).
  // Used by the XJTU GUI OPF module.
  svr.Post("/api/session/opf",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      const std::string solver = j.value("solver", std::string("parity"));
      // Optional post-OPF power-flow consistency audit (OPF ↔ PF model/parameters).
      const bool want_consistency = j.value("check_consistency", false);
      const json cons = j.contains("constraints") ? j["constraints"] : json::object();
      const bool en_branch = cons.value("branch_limits", true);
      const bool en_cap    = cons.value("converter_capacity", true);
      const bool en_iac    = cons.value("converter_current", true);
      const bool en_mod    = cons.value("converter_modulation", true);

	      for (auto& conv : sys.vsc_converters) {
	        if (!conv.in_service || !conv.ac_grid_forming) continue;
	        conv.control_mode = hacdcpf::ConverterMode::AC_GRID_FORMING;
	        conv.p_is_hard_constraint = false;
	      }
		      const hacdcpf::HybridPowerSystem original_sys = sys;
		      const auto er_snapshot = original_sys.energy_routers;
		      hacdcpf::HybridPowerSystem presentation_sys =
		          hacdcpf::project_to_canonical_models(original_sys);
	      for (auto& conv : presentation_sys.vsc_converters) {
	        if (!conv.in_service || !conv.ac_grid_forming) continue;
	        conv.control_mode = hacdcpf::ConverterMode::AC_GRID_FORMING;
	        conv.p_is_hard_constraint = false;
	      }

	      json out;
      out["solver"] = solver;
      out["constraints"] = {{"branch_limits", en_branch},
                            {"converter_capacity", en_cap},
                            {"converter_current", en_iac},
                            {"converter_modulation", en_mod}};

	      if (solver == "dc") {
	        hacdcpf::opf::DCOPFOptions opt;
	        opt.include_branch_limits = en_branch;
	        auto r = hacdcpf::solve_dc_opf(sys, opt);
	        out["converged"]=r.converged; out["iterations"]=r.iterations;
	        out["objective"]=r.objective; out["status"]=r.status;
	        out["pg_mw"]=r.pg_mw; out["pf_mw"]=r.pf_mw; out["lmp"]=r.lmp;
	        out["generator_dispatch"] = opf_generator_dispatch_json(sys, r.pg_mw);
	        out["ac_bus_results"] = opf_ac_bus_results_json(sys, {}, r.va, r.lmp);
	        out["dc_opf_branch_dispatch"] = opf_dc_branch_dispatch_json(sys, r.pf_mw);
	        out["total_load_shedding_mw"]=r.total_load_shedding_mw;
	      } else {
        hacdcpf::opf::ACOPFOptions opt;
        // Map the GUI solver selector onto the modern backend enum.  The parity
        // full-space formulation is shared by the native IPM and Ipopt; only the
        // inner Newton engine differs.  EconomicDispatch is the fast merit-order
        // + single AC PF path (pure-AC).  Unknown / legacy values default to the
        // native parity IPM.
        using SB = hacdcpf::opf::ACOPFSolverBackend;
        if (solver == "ipopt") {
          opt.ac_solver_backend = SB::Ipopt;
        } else if (solver == "dispatch") {
          opt.ac_solver_backend = SB::EconomicDispatch;
        } else if (solver == "auto") {
          // Auto = parity-first with an Ipopt fallback.  Force a real OPF so a
          // pure-AC system does not silently degrade to economic dispatch.
          opt.ac_solver_backend = SB::Auto;
          opt.enable_primal_dual = true;
          opt.use_parity_ipm = true;
          opt.allow_fallback = true;
        } else {  // "parity" (default)
          opt.ac_solver_backend = SB::ParityIPM;
        }
        const bool nonlinear = (opt.ac_solver_backend != SB::EconomicDispatch);
        opt.max_inner_iterations = nonlinear ? 400 : 120;
        opt.enforce_branch_limits = en_branch;
        opt.enforce_converter_capacity = en_cap;
        opt.enforce_converter_current_limits = en_iac;
        opt.enforce_converter_modulation_limits = en_mod;
        auto r = hacdcpf::solve_ac_opf(sys, opt);
        out["converged"]=r.converged; out["iterations"]=r.iterations;
        out["objective"]=r.objective; out["status"]=r.status;
        // Report the engine that actually ran (e.g. parity_ipm:sparse_umfpack
        // or ipopt_filter_linesearch) so the GUI can show the realized backend.
        out["solver_backend"]=r.profiling.linear_solver_backend;
        out["solver_path"]=(r.solver_path==hacdcpf::opf::OPFSolverPath::ParityIPM)?"parity":
                           (r.solver_path==hacdcpf::opf::OPFSolverPath::NativeAC)?"native_ac":"other";
	        out["vm"]=r.vm; out["va"]=r.va; out["pg_mw"]=r.pg_mw; out["qg_mvar"]=r.qg_mvar;
	        out["vdc"]=r.vdc; out["pac_mw"]=r.pac_mw; out["qac_mvar"]=r.qac_mvar;
	        out["dpd_mw"]=r.dpd_mw; out["pren_mw"]=r.pren_mw; out["pstor_mw"]=r.pstor_mw;
	        out["pdcdc_mw"]=r.pdcdc_mw; out["pflex_mw"]=r.pflex_mw;
	        out["er_port_p_mw"]=r.er_port_p_mw; out["er_port_q_mvar"]=r.er_port_q_mvar;
	        out["ac_bus_results"] = opf_ac_bus_results_json(original_sys, r.vm, r.va, r.lmp_p, r.lmp_q);
	        out["dc_bus_results"] = opf_dc_bus_results_json(original_sys, r.vdc);
	        out["generator_dispatch"] = opf_generator_dispatch_json(presentation_sys, r.pg_mw, r.qg_mvar);
	        out["vsc_dispatch"] = opf_vsc_dispatch_json(presentation_sys, original_sys, r.pac_mw, r.qac_mvar);
	        out["dcdc_dispatch"] = opf_dcdc_dispatch_json(presentation_sys, original_sys, r);
	        out["er_port_dispatch"] = opf_er_port_dispatch_json(original_sys, r);
	        if (!r.lmp_p.empty()) out["lmp_p"]=r.lmp_p;
	        if (!r.lmp_q.empty()) out["lmp_q"]=r.lmp_q;
        const auto& v = r.converter_model_scope.validity;
        out["scope"] = {{"model_scope", r.converter_model_scope.model_scope},
                        {"capacity", v.vsc_capacity_circle_enforced},
                        {"current", v.vsc_current_limits_enforced},
                        {"modulation", v.vsc_modulation_limits_enforced},
                        {"dcdc_duty", v.dcdc_duty_ratio_enforced},
                        {"vdc_control", v.vsc_vdc_control_modelled}};
        if (r.converged) {
          // Replay against the same canonical model that the OPF formulation
          // sees.  This preserves projected external grids and energy-router
          // internal VSC/DC-DC devices for the OPF -> PF consistency audit.
          hacdcpf::HybridPowerSystem replay_sys = presentation_sys;
          const auto projected_opf_vm = project_replay_ac_bus_vector(r.vm, replay_sys);
          const auto projected_opf_va = project_replay_ac_bus_vector(r.va, replay_sys);
          for (size_t i = 0; i < r.pg_mw.size() && i < replay_sys.ac.generators.size(); ++i) {
            replay_sys.ac.generators[i].pg_mw = r.pg_mw[i];
            if (i < r.qg_mvar.size()) replay_sys.ac.generators[i].qg_mvar = r.qg_mvar[i];
          }
          if (!projected_opf_vm.empty()) {
            for (size_t i = 0; i < projected_opf_vm.size() && i < replay_sys.ac.buses.size(); ++i)
              replay_sys.ac.buses[i].vm_pu = projected_opf_vm[i];
          }
          // Build the post-OPF PF view from the OPF operating point, even when the
          // optional consistency audit is disabled.  Otherwise the GUI overlay can
          // show stale converter setpoints while the OPF tables show the optimized
          // Pac/Qac/Pdcdc vectors.
          constexpr double kRad2Deg = 57.295779513082320876798154814105;
          for (size_t i = 0; i < projected_opf_va.size() && i < replay_sys.ac.buses.size(); ++i)
            replay_sys.ac.buses[i].va_deg = projected_opf_va[i] * kRad2Deg;
	          for (size_t i = 0; i < r.vdc.size() && i < replay_sys.dc.buses.size(); ++i) {
	            replay_sys.dc.buses[i].vm_pu = r.vdc[i];
	            replay_sys.dc.buses[i].bus_type = hacdcpf::DCBusType::DC_V;
	          }
          for (auto& eg : replay_sys.ac.external_grids) {
            if (const auto vm = replay_ac_value_for_bus(r.vm, replay_sys, eg.bus))
              eg.vm_pu = *vm;
            if (const auto va = replay_ac_value_for_bus(r.va, replay_sys, eg.bus))
              eg.va_deg = *va * kRad2Deg;
          }
          for (auto& gen : replay_sys.ac.generators) {
            if (const auto vm = replay_ac_value_for_bus(r.vm, replay_sys, gen.bus))
              gen.vg_pu = *vm;
          }

          bool dc_has_voltage_anchor = false;
          for (const auto& db : replay_sys.dc.buses)
            if (db.in_service && db.bus_type == hacdcpf::DCBusType::DC_V) {
              dc_has_voltage_anchor = true; break;
            }
          std::unordered_map<int, size_t> ac_result_pos_by_bus, dc_result_pos_by_bus;
          for (const auto& bus : replay_sys.ac.buses) {
            if (const auto pos = replay_ac_original_pos_for_bus(replay_sys, bus.index);
                pos && *pos < r.vm.size()) {
              ac_result_pos_by_bus[bus.index] = *pos;
            }
          }
          for (size_t bi = 0; bi < replay_sys.dc.buses.size() && bi < r.vdc.size(); ++bi)
            dc_result_pos_by_bus[replay_sys.dc.buses[bi].index] = bi;
          auto ac_result_pos = [&](int bus) -> std::optional<size_t> {
            const auto it = ac_result_pos_by_bus.find(bus);
            if (it == ac_result_pos_by_bus.end()) return std::nullopt;
            return it->second;
          };
          auto dc_result_pos = [&](int bus) -> std::optional<size_t> {
            const auto it = dc_result_pos_by_bus.find(bus);
            if (it == dc_result_pos_by_bus.end()) return std::nullopt;
            return it->second;
          };
	          const auto opf_vsc_positions = opf_active_vsc_positions(replay_sys);
	          std::unordered_map<size_t, size_t> opf_vsc_result_pos_by_component;
	          opf_vsc_result_pos_by_component.reserve(opf_vsc_positions.size());
	          for (size_t k = 0; k < opf_vsc_positions.size(); ++k)
	            opf_vsc_result_pos_by_component[opf_vsc_positions[k]] = k;
	          for (size_t i = 0; i < replay_sys.vsc_converters.size(); ++i) {
	            auto& c = replay_sys.vsc_converters[i];
	            const auto rit = opf_vsc_result_pos_by_component.find(i);
	            const std::optional<size_t> result_pos =
	                rit == opf_vsc_result_pos_by_component.end()
	                    ? std::nullopt
	                    : std::optional<size_t>(rit->second);
	            if (result_pos && *result_pos < r.qac_mvar.size()) c.q_set_mvar = r.qac_mvar[*result_pos];
	            if (result_pos && *result_pos < r.pac_mw.size()) { c.p_schedule_mw = r.pac_mw[*result_pos];
	                                                               c.p_initial_mw  = r.pac_mw[*result_pos]; }
	            // Replay the OPF point, not the converter controller response:
	            // OPF optimized Pac/Qac/Pdc/Vdc directly, while PF VDC modes
	            // would otherwise recompute Pac from Vdc and move the state.
	            if (result_pos && *result_pos < r.pac_mw.size()) { c.p_set_mw = r.pac_mw[*result_pos];
	                                                               c.p_is_hard_constraint = true; }
	            c.control_mode = hacdcpf::ConverterMode::PQ_MODE;
	            if (const auto pos = ac_result_pos(c.bus_ac))
	              c.v_ac_set_pu = r.vm[*pos];
	            if (const auto pos = dc_result_pos(c.bus_dc))
	              c.v_dc_set_pu = r.vdc[*pos];
	          }
		          const auto er_expansion_refs =
		              build_energy_router_expansion_refs(original_sys, er_snapshot);
		          std::unordered_map<int, size_t> replay_vsc_pos_by_index;
		          replay_vsc_pos_by_index.reserve(replay_sys.vsc_converters.size());
		          for (size_t vi = 0; vi < replay_sys.vsc_converters.size(); ++vi)
		            replay_vsc_pos_by_index[replay_sys.vsc_converters[vi].index] = vi;
		          for (size_t k = 0; k < r.er_port_p_mw.size() && k < r.er_port_map.size(); ++k) {
		            const int er_index = r.er_port_map[k].original_index;
		            const int port_index = r.er_port_map[k].source_type;
		            if (const auto ref_it = er_expansion_refs.find(er_index);
		                ref_it != er_expansion_refs.end()) {
		              if (const auto vit = ref_it->second.port_vsc_index.find(port_index);
		                  vit != ref_it->second.port_vsc_index.end()) {
		                if (const auto pit = replay_vsc_pos_by_index.find(vit->second);
		                    pit != replay_vsc_pos_by_index.end()) {
		                  auto& c = replay_sys.vsc_converters[pit->second];
		                  c.p_set_mw = r.er_port_p_mw[k];
		                  c.p_schedule_mw = r.er_port_p_mw[k];
		                  c.p_initial_mw = r.er_port_p_mw[k];
		                  c.p_is_hard_constraint = true;
		                  if (k < r.er_port_q_mvar.size()) c.q_set_mvar = r.er_port_q_mvar[k];
		                  c.control_mode = hacdcpf::ConverterMode::PQ_MODE;
		                }
		              }
		            }
		            for (auto& er : replay_sys.energy_routers) {
		              if (er.index != er_index) continue;
		              for (auto& port : er.ports) {
	                if (port.index != port_index) continue;
	                port.p_mw = r.er_port_p_mw[k];
	                port.p_set_mw = r.er_port_p_mw[k];
	                if (k < r.er_port_q_mvar.size()) {
	                  port.q_mvar = r.er_port_q_mvar[k];
	                  port.q_set_mvar = r.er_port_q_mvar[k];
	                }
	                if (const auto vm = replay_ac_value_for_bus(r.vm, replay_sys, port.bus))
	                  port.v_pu = *vm;
	              }
	            }
	          }
	          const auto opf_dcdc_positions = opf_active_dcdc_positions(replay_sys);
	          for (size_t k = 0; k < r.pdcdc_mw.size(); ++k) {
	            size_t i = k;
	            if (k < r.dcdc_map.size() && r.dcdc_map[k].original_index >= 0) {
	              i = static_cast<size_t>(r.dcdc_map[k].original_index);
	            } else if (k < opf_dcdc_positions.size()) {
	              i = opf_dcdc_positions[k];
	            }
	            if (i >= replay_sys.dc.dcdc_converters.size()) continue;
	            auto& dcdc = replay_sys.dc.dcdc_converters[i];
	            dcdc.p_ref_mw = hacdcpf::powerflow::dcdc_output_power_from_input_ref_mw(
	                dcdc, r.pdcdc_mw[k]);
	            if (dc_has_voltage_anchor) {
	              dcdc.control_mode = hacdcpf::DCDCControlMode::Power;
	            } else if (const auto pos = dc_result_pos(replay_sys.dc.dcdc_converters[i].bus_out)) {
              dcdc.v_ref_pu = r.vdc[*pos];
            }
          }
          hacdcpf::PowerFlowOptions replay_opt;
          replay_opt.max_iter = 80;
          replay_opt.tol = 1e-8;
          hacdcpf::PowerFlowResult projected_ac_pf;
          auto ac_pf = solve_projected_replay_power_flow(
              replay_sys, replay_opt, &projected_ac_pf);
          if (ac_pf.converged) {
            { std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = "opf"; }
            // Post-OPF power-flow view: the actual branch flows + converter
            // transfers at the OPF dispatch, so the GUI can show 潮流 (and drive a
            // branch-flow heat-map) for the optimized operating point.
            json post_pf;
            post_pf["converged"] = true;
            json brf = json::array();
            for (size_t i = 0; i < replay_sys.ac.branches.size() && i < ac_pf.branch_flows.size(); ++i) {
              const auto& br = replay_sys.ac.branches[i];
              const auto& f = ac_pf.branch_flows[i];
              const double rate = br.rate_a_mva;
              const double s_from = std::hypot(f.pf_mw, f.qf_mvar);
              const double s_to = std::hypot(f.pt_mw, f.qt_mvar);
              const double loading = (rate > 1e-9) ? (std::max(s_from, s_to) / rate * 100.0) : 0.0;
	              brf.push_back(json{{"index", br.index}, {"name", br.name}, {"canvas_type", "branch"},
	                {"canvas_index", br.index}, {"from_bus", br.from_bus}, {"to_bus", br.to_bus},
	                {"pf_mw", f.pf_mw}, {"qf_mvar", f.qf_mvar}, {"pt_mw", f.pt_mw}, {"qt_mvar", f.qt_mvar},
	                {"loss_mw", f.pf_mw + f.pt_mw}, {"loading_pct", loading}, {"rate_mva", rate}});
            }
            post_pf["branch_flows"] = brf;
            json vtr = json::array();
            for (const auto& v : ac_pf.vsc_transfers)
              vtr.push_back(json{{"index",v.index},{"bus_ac",v.bus_ac},{"bus_dc",v.bus_dc},
                {"p_ac_mw",v.p_ac_mw},{"q_ac_mvar",v.q_ac_mvar},{"p_dc_mw",v.p_dc_mw},{"loss_mw",v.loss_mw}});
            post_pf["vsc_transfers"] = vtr;
            json dtr = json::array();
            for (const auto& d : ac_pf.dcdc_transfers)
              dtr.push_back(json{{"index",d.index},{"bus_in",d.bus_in},{"bus_out",d.bus_out},
                {"p_in_mw",d.p_in_mw},{"p_out_mw",d.p_out_mw},{"loss_mw",d.loss_mw}});
            post_pf["dcdc_transfers"] = dtr;
            json dcbr = json::array();
            std::unordered_map<int, size_t> dc_pos_by_bus;
            for (size_t bi = 0; bi < replay_sys.dc.buses.size(); ++bi)
              dc_pos_by_bus[replay_sys.dc.buses[bi].index] = bi;
            for (size_t i = 0; i < replay_sys.dc.branches.size(); ++i) {
              const auto& br = replay_sys.dc.branches[i];
              double pf_mw = 0.0, pt_mw = 0.0, loss_mw = 0.0, loading = 0.0;
              const auto fi = dc_pos_by_bus.find(br.from_bus);
              const auto ti = dc_pos_by_bus.find(br.to_bus);
              if (br.in_service && br.r_pu > 1e-12 &&
                  fi != dc_pos_by_bus.end() && ti != dc_pos_by_bus.end() &&
                  fi->second < ac_pf.vdc.size() && ti->second < ac_pf.vdc.size()) {
                const double vf = ac_pf.vdc[fi->second];
                const double vt = ac_pf.vdc[ti->second];
                const double i_pu = (vf - vt) / br.r_pu;
                pf_mw = vf * i_pu * sys.base_mva;
                pt_mw = -vt * i_pu * sys.base_mva;
                loss_mw = pf_mw + pt_mw;
                if (br.rate_a_mva > 0.0)
                  loading = 100.0 * std::max(std::abs(pf_mw), std::abs(pt_mw)) / br.rate_a_mva;
              }
              dcbr.push_back(json{{"index",br.index},{"from_bus",br.from_bus},{"to_bus",br.to_bus},
                {"pf_mw",pf_mw},{"pt_mw",pt_mw},{"loss_mw",loss_mw},{"loading_pct",loading},
                {"rate_mva",br.rate_a_mva}});
            }
	            post_pf["dc_branch_flows"] = dcbr;
	            post_pf["geo_er"] =
	                power_flow_geo_energy_router_json(original_sys, er_snapshot, ac_pf);
	            post_pf["vm"] = ac_pf.vm; post_pf["va"] = ac_pf.va; post_pf["vdc"] = ac_pf.vdc;
	            out["post_pf"] = post_pf;
            // Post-OPF carbon flow: static carbon-emission-flow analysis on the OPF
            // dispatch (same engine as /api/session/run_carbon, fed the OPF PF).
            try {
              hacdcpf::analysis::CarbonAnalysisOptions ca_opt; ca_opt.verbose = false;
              auto carbon = hacdcpf::analysis::compute_carbon_analysis(replay_sys, ac_pf, ca_opt);
              out["post_carbon"] = carbon_analysis_to_json(sys, carbon);
            } catch (const std::exception&) { /* carbon view is best-effort */ }
          }
          // ── OPF ↔ PF consistency audit ──────────────────────────────────
          // Re-solve a plain power flow at the OPF dispatch (generation + voltage
          // setpoints fixed) and measure how far the independent PF state drifts
          // from the OPF state.  When the OPF and PF share the same network model
          // and parameters, the OPF optimum is a PF fixed point, so the voltage /
          // angle deviations are ~0.  A visible deviation localizes a modelling
          // or parameter inconsistency between the two formulations.
          if (want_consistency) {
            json cc;
            cc["ran"] = true;
            cc["pf_converged"] = ac_pf.converged;
            cc["pf_iterations"] = ac_pf.iterations;
            cc["pf_residual"] = ac_pf.residual;
            if (ac_pf.converged) {
              auto ac_bus_id_at = [&](int pos) -> int {
                return pos >= 0 && static_cast<size_t>(pos) < replay_sys.ac.buses.size()
                           ? replay_sys.ac.buses[static_cast<size_t>(pos)].index
                           : -1;
              };
              auto dc_bus_id_at = [&](int pos) -> int {
                return pos >= 0 && static_cast<size_t>(pos) < replay_sys.dc.buses.size()
                           ? replay_sys.dc.buses[static_cast<size_t>(pos)].index
                           : -1;
              };
              auto ac_branch_id_at = [&](int pos) -> int {
                if (pos < 0 || static_cast<size_t>(pos) >= replay_sys.ac.branches.size()) return -1;
                const auto& br = replay_sys.ac.branches[static_cast<size_t>(pos)];
                return br.index;
              };
              double max_dvm = 0.0, sum_dvm = 0.0; int worst_vm = -1; size_t nvm = 0;
              for (size_t i = 0; i < r.vm.size() && i < ac_pf.vm.size(); ++i) {
                const double d = std::abs(r.vm[i] - ac_pf.vm[i]);
                sum_dvm += d; ++nvm;
                if (d > max_dvm) { max_dvm = d; worst_vm = static_cast<int>(i); }
              }
              double max_dva = 0.0, sum_dva = 0.0; int worst_va = -1; size_t nva = 0;
              const double va_ref =
                  (!r.va.empty() && !ac_pf.va.empty()) ? (r.va[0] - ac_pf.va[0]) : 0.0;
              for (size_t i = 0; i < r.va.size() && i < ac_pf.va.size(); ++i) {
                const double d = std::abs((r.va[i] - ac_pf.va[i]) - va_ref);
                sum_dva += d; ++nva;
                if (d > max_dva) { max_dva = d; worst_va = static_cast<int>(i); }
              }
              double max_dvdc = 0.0; int worst_vdc = -1;
              for (size_t i = 0; i < r.vdc.size() && i < ac_pf.vdc.size(); ++i) {
                const double d = std::abs(r.vdc[i] - ac_pf.vdc[i]);
                if (d > max_dvdc) { max_dvdc = d; worst_vdc = static_cast<int>(i); }
              }
              constexpr double kRad2Deg = 57.295779513082320876798154814105;
              const double max_dva_deg = max_dva * kRad2Deg;
              const double tol_vm = 1e-3, tol_va_deg = 0.1, tol_vdc = 1e-3;
              cc["max_dvm_pu"]  = max_dvm;
              cc["mean_dvm_pu"] = nvm ? sum_dvm / static_cast<double>(nvm) : 0.0;
              cc["max_dvm_bus"] = ac_bus_id_at(worst_vm);
              cc["max_dva_deg"]  = max_dva_deg;
              cc["mean_dva_deg"] = nva ? (sum_dva / static_cast<double>(nva)) * kRad2Deg : 0.0;
              cc["max_dva_bus"]  = ac_bus_id_at(worst_va);
              cc["max_dvdc_pu"]  = max_dvdc;
              cc["max_dvdc_bus"] = dc_bus_id_at(worst_vdc);
              cc["tol_vm_pu"] = tol_vm; cc["tol_va_deg"] = tol_va_deg; cc["tol_vdc_pu"] = tol_vdc;
              cc["consistent"] = (max_dvm <= tol_vm) && (max_dva_deg <= tol_va_deg) &&
                                 (r.vdc.empty() || max_dvdc <= tol_vdc);

              // Branch-flow + power-balance (loss) consistency: AC branch flows
              // at the OPF voltage state, assembled with the same network the PF
              // used, compared against the PF flows.  Because the load is the
              // same in both, the branch-loss difference equals the extra slack
              // generation the PF needs — i.e. the generation/loss consistency.
              try {
                auto opf_data = hacdcpf::powerflow::make_solver_data_projected(
                    hacdcpf::HybridPowerSystem(replay_sys), hacdcpf::LossModelType::Linear);
                if (projected_opf_vm.size() == opf_data.ac_buses.size() &&
                    projected_opf_va.size() == opf_data.ac_buses.size()) {
                  auto opf_flows =
                      hacdcpf::powerflow::compute_branch_flows(
                          opf_data, projected_opf_vm, projected_opf_va);
                  double max_dpf = 0.0, max_dqf = 0.0; int worst_br = -1;
                  double loss_opf = 0.0, loss_pf = 0.0;
                  const size_t nbr =
                      std::min(opf_flows.size(), projected_ac_pf.branch_flows.size());
                  for (size_t i = 0; i < nbr; ++i) {
                    const double dpf =
                        std::abs(opf_flows[i].pf_mw -
                                 projected_ac_pf.branch_flows[i].pf_mw);
                    const double dqf =
                        std::abs(opf_flows[i].qf_mvar -
                                 projected_ac_pf.branch_flows[i].qf_mvar);
                    if (dpf > max_dpf) { max_dpf = dpf; worst_br = static_cast<int>(i); }
                    if (dqf > max_dqf) max_dqf = dqf;
                    loss_opf += opf_flows[i].pf_mw + opf_flows[i].pt_mw;
                    loss_pf += projected_ac_pf.branch_flows[i].pf_mw +
                               projected_ac_pf.branch_flows[i].pt_mw;
                  }
                  double conv_loss = 0.0;
                  for (const auto& vt : ac_pf.vsc_transfers) conv_loss += vt.loss_mw;
                  cc["max_dpf_mw"]     = max_dpf;
                  cc["max_dqf_mvar"]   = max_dqf;
                  cc["max_dpf_branch"] = ac_branch_id_at(worst_br);
                  cc["branch_loss_opf_mw"]      = loss_opf;
                  cc["branch_loss_pf_mw"]       = loss_pf;
                  cc["branch_loss_mismatch_mw"] = std::abs(loss_opf - loss_pf);
                  cc["converter_loss_pf_mw"]    = conv_loss;
                }
              } catch (const std::exception&) { /* flow/loss audit is best-effort */ }
            } else {
              cc["consistent"] = false;
              cc["note"] = "post-OPF 潮流未收敛，无法完成一致性校验";
            }
            out["consistency"] = cc;
          }
        }
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/opf_ac",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      hacdcpf::opf::ACOPFOptions opt;
      opt.allow_fallback = true; opt.max_inner_iterations = 120;
      // Solver backend selection from the GUI dropdown.  "auto"/"parity"/"ipopt"
      // all run a genuine nonlinear OPF (fixing the historical behaviour where
      // "Run AC OPF" silently fell back to economic dispatch on pure-AC cases);
      // "dispatch" keeps the fast merit-order + AC PF path.
      std::string solver = "auto";
      try {
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        if (j.contains("solver") && j["solver"].is_string()) solver = j["solver"].get<std::string>();
      } catch (...) {}
      if (solver == "parity") {
        opt.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
      } else if (solver == "ipopt") {
        opt.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::Ipopt;
      } else if (solver == "dispatch") {
        opt.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::EconomicDispatch;
      } else {
        opt.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::Auto;
        opt.enable_primal_dual = true; opt.use_parity_ipm = true;
      }
      auto r = hacdcpf::solve_ac_opf(sys, opt);
      json out;
      out["converged"]=r.converged; out["iterations"]=r.iterations;
      out["objective"]=r.objective; out["status"]=r.status;
      out["solver_backend"]=r.profiling.linear_solver_backend;
      out["vm"]=r.vm; out["va"]=r.va; out["pg_mw"]=r.pg_mw; out["qg_mvar"]=r.qg_mvar;
      out["vdc"]=r.vdc; out["pac_mw"]=r.pac_mw; out["qac_mvar"]=r.qac_mvar;
      if (!r.lmp_p.empty()) out["lmp_p"]=r.lmp_p;
      if (!r.lmp_q.empty()) out["lmp_q"]=r.lmp_q;
      // Apply OPF dispatch and run AC PF for carbon analysis
      if (r.converged) {
        for (size_t i = 0; i < r.pg_mw.size() && i < sys.ac.generators.size(); ++i) {
          sys.ac.generators[i].pg_mw = r.pg_mw[i];
          if (i < r.qg_mvar.size()) sys.ac.generators[i].qg_mvar = r.qg_mvar[i];
        }
        if (!r.vm.empty()) { for (size_t i = 0; i < r.vm.size() && i < sys.ac.buses.size(); ++i) sys.ac.buses[i].vm_pu = r.vm[i]; }
        auto ac_pf = hacdcpf::solve_power_flow(sys);
        if (ac_pf.converged) { std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = "opf_ac"; }
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/opf_parity",
           [](const httplib::Request&, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      hacdcpf::opf::ACOPFOptions opt;
      opt.use_parity_ipm = true;
      opt.allow_fallback = false;
      opt.max_inner_iterations = 400;
      auto r = hacdcpf::solve_ac_opf(sys, opt);
      json out;
      out["converged"]=r.converged; out["iterations"]=r.iterations;
      out["objective"]=r.objective; out["status"]=r.status;
      out["vm"]=r.vm; out["va"]=r.va; out["pg_mw"]=r.pg_mw; out["qg_mvar"]=r.qg_mvar;
      out["vdc"]=r.vdc; out["pac_mw"]=r.pac_mw; out["qac_mvar"]=r.qac_mvar;
      out["dpd_mw"]=r.dpd_mw; out["dqd_mvar"]=r.dqd_mvar;
      out["pren_mw"]=r.pren_mw; out["pstor_mw"]=r.pstor_mw;
      out["pdcdc_mw"]=r.pdcdc_mw; out["pflex_mw"]=r.pflex_mw;
      if (!r.lmp_p.empty()) out["lmp_p"]=r.lmp_p;
      if (!r.lmp_q.empty()) out["lmp_q"]=r.lmp_q;
      // Apply OPF dispatch and run AC PF for carbon analysis
      if (r.converged) {
        for (size_t i = 0; i < r.pg_mw.size() && i < sys.ac.generators.size(); ++i) {
          sys.ac.generators[i].pg_mw = r.pg_mw[i];
          if (i < r.qg_mvar.size()) sys.ac.generators[i].qg_mvar = r.qg_mvar[i];
        }
        if (!r.vm.empty()) { for (size_t i = 0; i < r.vm.size() && i < sys.ac.buses.size(); ++i) sys.ac.buses[i].vm_pu = r.vm[i]; }
        auto ac_pf = hacdcpf::solve_power_flow(sys);
        if (ac_pf.converged) { std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = "opf_parity"; }
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/opf_dc",
           [](const httplib::Request&, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      hacdcpf::opf::DCOPFOptions opt;
      auto r = hacdcpf::solve_dc_opf(sys, opt);
      json out;
      out["converged"]=r.converged; out["iterations"]=r.iterations;
      out["objective"]=r.objective; out["status"]=r.status;
      out["pg_mw"]=r.pg_mw; out["pf_mw"]=r.pf_mw; out["lmp"]=r.lmp;
      out["total_load_shedding_mw"]=r.total_load_shedding_mw;
      // Apply DC OPF dispatch and run AC PF for carbon analysis
      if (r.converged) {
        for (size_t i = 0; i < r.pg_mw.size() && i < sys.ac.generators.size(); ++i)
          sys.ac.generators[i].pg_mw = r.pg_mw[i];
        auto ac_pf = hacdcpf::solve_power_flow(sys);
        if (ac_pf.converged) { std::lock_guard<std::mutex> lk(g_session.mu); g_session.last_pf_result = ac_pf; g_session.last_pf_method = "opf_dc"; }
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/session/sc",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      hacdcpf::analysis::SCDetailedOptions dopt;
      if (j.contains("options")) {
        const auto& o = j["options"];
        std::string ft = o.value("fault_type", std::string("ThreePhase"));
        if (ft=="SinglePhaseGround") dopt.fault_type=hacdcpf::analysis::FaultType::SinglePhaseGround;
        else if (ft=="TwoPhase") dopt.fault_type=hacdcpf::analysis::FaultType::TwoPhase;
        else if (ft=="TwoPhaseGround") dopt.fault_type=hacdcpf::analysis::FaultType::TwoPhaseGround;
        else dopt.fault_type=hacdcpf::analysis::FaultType::ThreePhase;
      }
      dopt.compute_branch_flows = false;
      dopt.compute_voltage_drops = false;
      dopt.compute_ith = true;
      // Fault at every AC bus
      std::vector<int> all_bus_ids;
      std::unordered_map<int, double> bus_kv;
      for (const auto& b : sys.ac.buses) {
        if (b.in_service) {
          all_bus_ids.push_back(b.index);
          bus_kv[b.index] = b.base_kv;
        }
      }
      auto detailed = hacdcpf::analysis::run_short_circuit_detailed_batch(sys, all_bus_ids, dopt);
      json out;
      out["fault_type"] = [&](){
        switch(dopt.fault_type){
          case hacdcpf::analysis::FaultType::ThreePhase: return "ThreePhase";
          case hacdcpf::analysis::FaultType::SinglePhaseGround: return "SinglePhaseGround";
          case hacdcpf::analysis::FaultType::TwoPhase: return "TwoPhase";
          case hacdcpf::analysis::FaultType::TwoPhaseGround: return "TwoPhaseGround";
        } return "ThreePhase";
      }();
      out["bus_results"] = json::array();
      for (const auto& dr : detailed) {
        // Find the fault bus's own result
        const auto it = std::find_if(dr.bus_results.begin(), dr.bus_results.end(),
            [&](const hacdcpf::analysis::SCDetailedBusResult& br){ return br.bus_id == dr.fault_bus_id; });
        if (it != dr.bus_results.end()) {
          const double un = bus_kv.count(dr.fault_bus_id) ? bus_kv.at(dr.fault_bus_id) : 110.0;
          const double sk = std::sqrt(3.0) * un * it->ikss_ka;
          out["bus_results"].push_back(json{
            {"bus_id", dr.fault_bus_id},
            {"ikpp_ka", it->ikss_ka}, {"sk_mva", sk},
            {"ip_ka", it->ip_ka}, {"ib_ka", it->ib_ka},
            {"ik_ka", it->ik_ka}, {"ith_ka", it->ith_ka}
          });
        }
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Harmonic Power Flow (hybrid AC/DC, frequency-domain penetration) ----
  svr.Post("/api/session/harmonics",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);

      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      hacdcpf::harmonics::HPFOptions opt;
      hacdcpf::harmonics::HarmonicStudyInputs inputs;
      if (j.contains("options")) hpf_api::parse_options(j["options"], opt);
      // User harmonic current sources (nonlinear loads / CIDERs) + explicit NICs.
      hpf_api::parse_inputs(j, inputs);

      auto r = hacdcpf::harmonics::solve_harmonic_power_flow(sys, inputs, opt);

      json out;
      out["ok"] = r.ok;
      out["message"] = r.message;
      out["base_pf_converged"] = r.base_pf_converged;
      out["ac_orders"] = r.ac_orders;
      out["dc_orders"] = r.dc_orders;
      out["max_ac_thd_pct"] = r.max_ac_thd_pct;
      out["max_ac_thd_bus"] = r.max_ac_thd_bus;
      out["max_dc_thd_pct"] = r.max_dc_thd_pct;
      out["max_dc_thd_bus"] = r.max_dc_thd_bus;

      auto bus_json = [](const hacdcpf::harmonics::HarmonicBusResult& b) {
        json jb;
        jb["bus"] = b.bus;
        jb["is_dc"] = b.is_dc;
        jb["v_fund_pu"] = b.v_fund_pu;
        jb["thd_pct"] = b.thd_pct;
        json spec = json::array();
        for (const auto& [ord, v] : b.v_by_order)
          spec.push_back(json{{"order", ord}, {"mag_pu", std::abs(v)},
                              {"phase_deg", std::arg(v) * 180.0 / M_PI}});
        jb["harmonics"] = spec;
        return jb;
      };
      out["ac_bus_results"] = json::array();
      for (const auto& b : r.ac_bus_results) out["ac_bus_results"].push_back(bus_json(b));
      out["dc_bus_results"] = json::array();
      for (const auto& b : r.dc_bus_results) out["dc_bus_results"].push_back(bus_json(b));

      out["ac_branch_flows"] = json::array();
      for (const auto& bf : r.ac_branch_flows) {
        json spec = json::array();
        for (const auto& [ord, m] : bf.i_by_order)
          spec.push_back(json{{"order", ord}, {"i_pu", m}});
        out["ac_branch_flows"].push_back(json{{"from_bus", bf.from_bus},
            {"to_bus", bf.to_bus}, {"thd_i_pct", bf.thd_i_pct}, {"harmonics", spec}});
      }
      out["dc_branch_flows"] = json::array();
      for (const auto& bf : r.dc_branch_flows) {
        json spec = json::array();
        for (const auto& [ord, m] : bf.i_by_order)
          spec.push_back(json{{"order", ord}, {"i_pu", m}});
        out["dc_branch_flows"].push_back(json{{"from_bus", bf.from_bus},
            {"to_bus", bf.to_bus}, {"thd_i_pct", bf.thd_i_pct}, {"harmonics", spec}});
      }

      // Optional harmonic-distortion limit compliance (IEEE 519 / GB-T 14549).
      std::string std_name;
      if (j.contains("options")) std_name = j["options"].value("standard", std::string(""));
      if (!std_name.empty() && r.ok) {
        hacdcpf::harmonics::HarmonicStandard hs =
            (std_name == "GBT14549" || std_name == "GBT14549_1993")
                ? hacdcpf::harmonics::HarmonicStandard::GBT14549_1993
                : hacdcpf::harmonics::HarmonicStandard::IEEE519_2014;
        auto rep = hacdcpf::harmonics::check_harmonic_limits(r, sys, hs);
        json comp;
        comp["standard"] = (hs == hacdcpf::harmonics::HarmonicStandard::GBT14549_1993)
                               ? "GB/T 14549-1993" : "IEEE 519-2014";
        comp["all_compliant"] = rep.all_compliant;
        comp["n_violations"] = rep.n_violations;
        comp["worst_bus"] = rep.worst_bus;
        comp["worst_ratio"] = rep.worst_ratio;
        comp["checks"] = json::array();
        for (const auto& c : rep.checks) {
          comp["checks"].push_back(json{
            {"bus", c.bus}, {"base_kv", c.base_kv}, {"thd_pct", c.thd_pct},
            {"thd_limit_pct", c.thd_limit_pct}, {"thd_ok", c.thd_ok},
            {"worst_ihd_order", c.worst_ihd_order}, {"worst_ihd_pct", c.worst_ihd_pct},
            {"ihd_limit_pct", c.ihd_limit_pct}, {"ihd_ok", c.ihd_ok},
            {"compliant", c.compliant}});
        }
        out["compliance"] = comp;
      }

      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Harmonic frequency scan / resonance analysis ----
  svr.Post("/api/session/harmonics_freqscan",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      hacdcpf::harmonics::HPFOptions opt;
      if (j.contains("options")) hpf_api::parse_options(j["options"], opt);

      hacdcpf::harmonics::FrequencyScanOptions sopt;
      const auto& o = j.contains("scan") ? j["scan"] : j;
      sopt.f_start = o.value("f_start", 1.0);
      sopt.f_end = o.value("f_end", 25.0);
      sopt.f_step = o.value("f_step", 0.1);
      sopt.resonance_min_pu = o.value("resonance_min_pu", 0.0);
      sopt.detect_resonances = o.value("detect_resonances", true);
      if (o.contains("buses") && o["buses"].is_array())
        sopt.buses = o["buses"].get<std::vector<int>>();

      json out;
      const bool sequence = o.value("sequence", false);
      if (sequence) {
        if (!sys.three_phase_ac)
          throw std::runtime_error("Per-sequence scan requires a three-phase model");
        int bus = o.value("bus", sopt.buses.empty() ? 0 : sopt.buses.front());
        auto r = hacdcpf::harmonics::sequence_frequency_scan(*sys.three_phase_ac, bus, sopt, opt);
        out["ok"] = r.ok; out["message"] = r.message; out["sequence"] = true;
        out["bus"] = r.bus; out["freqs"] = r.freqs;
        out["z1_mag"] = r.z1_mag; out["z2_mag"] = r.z2_mag; out["z0_mag"] = r.z0_mag;
        out["resonances"] = json::array();
        for (const auto& rz : r.resonances)
          out["resonances"].push_back(json{{"bus", rz.bus}, {"freq_order", rz.freq_order},
              {"z_mag", rz.z_mag}, {"parallel", rz.parallel}, {"sequence", rz.sequence}});
      } else {
        auto r = hacdcpf::harmonics::frequency_scan(sys, sopt, opt);
        out["ok"] = r.ok; out["message"] = r.message; out["sequence"] = false;
        out["freqs"] = r.freqs;
        out["buses"] = json::array();
        for (const auto& [bus, zm] : r.z_mag) {
          json jb; jb["bus"] = bus; jb["z_mag"] = zm;
          auto ait = r.z_ang_deg.find(bus);
          if (ait != r.z_ang_deg.end()) jb["z_ang_deg"] = ait->second;
          out["buses"].push_back(jb);
        }
        out["resonances"] = json::array();
        for (const auto& rz : r.resonances)
          out["resonances"].push_back(json{{"bus", rz.bus}, {"freq_order", rz.freq_order},
              {"z_mag", rz.z_mag}, {"parallel", rz.parallel}});
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Three-phase (abc-domain) harmonic power flow ----
  svr.Post("/api/session/harmonics_3ph",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (!sys.three_phase_ac) {
        res.status = 400;
        res.set_content(json{{"error","This case has no three-phase (abc) model; "
            "three-phase harmonic power flow is unavailable."}}.dump(), "application/json");
        return;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      hacdcpf::harmonics::HPFOptions opt;
      if (j.contains("options")) hpf_api::parse_options(j["options"], opt);

      hacdcpf::harmonics::ThreePhaseHarmonicInputs in;
      if (j.contains("sources") && j["sources"].is_array()) {
        for (const auto& s : j["sources"]) {
          hacdcpf::harmonics::ThreePhaseHarmonicSource src;
          src.bus = s.value("bus", 0);
          src.balanced = s.value("balanced", true);
          src.i_base_pu = s.value("i_base_pu", 0.0);
          src.i_base_phase_deg = s.value("i_base_phase_deg", 0.0);
          src.i_base_pu_a = s.value("i_base_pu_a", 0.0);
          src.i_base_pu_b = s.value("i_base_pu_b", 0.0);
          src.i_base_pu_c = s.value("i_base_pu_c", 0.0);
          if (s.contains("spectrum")) src.spectrum = hpf_api::parse_spectrum(s["spectrum"]);
          in.sources.push_back(std::move(src));
        }
      }
      auto r = hacdcpf::harmonics::solve_harmonic_power_flow_3ph(*sys.three_phase_ac, in, opt);

      json out;
      out["ok"] = r.ok; out["message"] = r.message;
      out["base_pf_converged"] = r.base_pf_converged;
      out["ac_orders"] = r.ac_orders;
      out["max_thd_pct"] = r.max_thd_pct; out["max_thd_bus"] = r.max_thd_bus;
      auto phase_spec = [](const std::map<int, hacdcpf::harmonics::Complex>& m) {
        json s = json::array();
        for (const auto& [ord, v] : m)
          s.push_back(json{{"order", ord}, {"mag_pu", std::abs(v)},
                           {"phase_deg", std::arg(v) * 180.0 / M_PI}});
        return s;
      };
      out["bus_results"] = json::array();
      for (const auto& b : r.bus_results) {
        out["bus_results"].push_back(json{
          {"bus", b.bus},
          {"v_fund_pu_a", b.v_fund_pu_a}, {"v_fund_pu_b", b.v_fund_pu_b},
          {"v_fund_pu_c", b.v_fund_pu_c},
          {"thd_a_pct", b.thd_a_pct}, {"thd_b_pct", b.thd_b_pct}, {"thd_c_pct", b.thd_c_pct},
          {"harmonics_a", phase_spec(b.v_by_order_a)},
          {"harmonics_b", phase_spec(b.v_by_order_b)},
          {"harmonics_c", phase_spec(b.v_by_order_c)}});
      }
      // Optional per-phase compliance.
      std::string std_name = j.contains("options")
          ? j["options"].value("standard", std::string("")) : std::string();
      if (!std_name.empty() && r.ok) {
        auto hs = (std_name == "GBT14549" || std_name == "GBT14549_1993")
                      ? hacdcpf::harmonics::HarmonicStandard::GBT14549_1993
                      : hacdcpf::harmonics::HarmonicStandard::IEEE519_2014;
        auto rep = hacdcpf::harmonics::check_harmonic_limits(r, *sys.three_phase_ac, hs);
        json comp;
        comp["standard"] = (hs == hacdcpf::harmonics::HarmonicStandard::GBT14549_1993)
                               ? "GB/T 14549-1993" : "IEEE 519-2014";
        comp["all_compliant"] = rep.all_compliant;
        comp["n_violations"] = rep.n_violations;
        comp["checks"] = json::array();
        for (const auto& c : rep.checks)
          comp["checks"].push_back(json{{"bus", c.bus}, {"phase", c.phase},
            {"base_kv", c.base_kv}, {"thd_pct", c.thd_pct},
            {"thd_limit_pct", c.thd_limit_pct}, {"compliant", c.compliant}});
        out["compliance"] = comp;
      }
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Harmonic metrics: losses, K-factor, current THD / TDD ----
  svr.Post("/api/session/harmonics_metrics",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      hacdcpf::harmonics::HPFOptions opt;
      hacdcpf::harmonics::HarmonicStudyInputs inputs;
      if (j.contains("options")) hpf_api::parse_options(j["options"], opt);
      hpf_api::parse_inputs(j, inputs);
      opt.compute_branch_flows = true;

      auto r = hacdcpf::harmonics::solve_harmonic_power_flow(sys, inputs, opt);
      hacdcpf::harmonics::HarmonicMetricsOptions mopt;
      mopt.i_demand_pu = j.value("i_demand_pu", 0.0);
      auto m = hacdcpf::harmonics::harmonic_metrics(sys, r, mopt, opt);

      json out;
      out["ok"] = r.ok; out["message"] = r.message;
      out["total_loss_pu"] = m.total_loss_pu;
      out["total_harmonic_loss_pu"] = m.total_harmonic_loss_pu;
      out["harmonic_loss_fraction"] = m.harmonic_loss_fraction;
      out["max_k_factor"] = m.max_k_factor;
      out["max_k_factor_branch"] = m.max_k_factor_branch;
      out["max_thd_i_pct"] = m.max_thd_i_pct;
      out["max_tdd_pct"] = m.max_tdd_pct;
      out["branches"] = json::array();
      for (const auto& b : m.ac_branches)
        out["branches"].push_back(json{
          {"from_bus", b.from_bus}, {"to_bus", b.to_bus},
          {"i_fund_pu", b.i_fund_pu}, {"i_rms_pu", b.i_rms_pu},
          {"thd_i_pct", b.thd_i_pct}, {"tdd_pct", b.tdd_pct},
          {"k_factor", b.k_factor}, {"p_loss_pu", b.p_loss_pu},
          {"p_loss_harmonic_pu", b.p_loss_harmonic_pu}});
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- Newton-Raphson harmonic power flow (nonlinear resources) ----
  svr.Post("/api/session/harmonics_newton",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      hacdcpf::harmonics::HPFOptions opt;
      hacdcpf::harmonics::HarmonicStudyInputs inputs;
      if (j.contains("options")) hpf_api::parse_options(j["options"], opt);
      hpf_api::parse_inputs(j, inputs);
      opt.newton_max_iter = j.value("max_iter", opt.newton_max_iter);
      opt.newton_tol = j.value("tol", opt.newton_tol);

      const std::string mode = j.value("mode", std::string("constant_power"));
      hacdcpf::harmonics::HPFNewtonResult r;
      if (mode == "holomorphic") {
        std::vector<hacdcpf::harmonics::HarmonicNonlinearSource> nl;
        if (j.contains("resources") && j["resources"].is_array())
          for (const auto& s : j["resources"]) {
            hacdcpf::harmonics::HarmonicNonlinearSource n;
            n.bus = s.value("bus", 0); n.name = s.value("name", std::string());
            if (s.contains("i_src")) n.i_src = hpf_api::parse_order_complex(s["i_src"]);
            if (s.contains("y_out")) n.y_out = hpf_api::parse_order_complex(s["y_out"]);
            if (s.contains("g2")) n.g2 = hpf_api::parse_order_complex(s["g2"]);
            nl.push_back(std::move(n));
          }
        r = hacdcpf::harmonics::solve_harmonic_power_flow_newton(sys, nl, inputs, opt);
      } else {  // constant_power (non-holomorphic, real/imag 2N Newton)
        std::vector<hacdcpf::harmonics::ConstantPowerHarmonicLoad> cp;
        if (j.contains("resources") && j["resources"].is_array())
          for (const auto& s : j["resources"]) {
            hacdcpf::harmonics::ConstantPowerHarmonicLoad c;
            c.bus = s.value("bus", 0); c.name = s.value("name", std::string());
            if (s.contains("s_set")) c.s_set = hpf_api::parse_order_complex(s["s_set"]);
            cp.push_back(std::move(c));
          }
        r = hacdcpf::harmonics::solve_harmonic_power_flow_newton_real(sys, cp, inputs, opt);
      }

      json out;
      out["ok"] = r.ok; out["message"] = r.message; out["mode"] = mode;
      out["converged"] = r.converged;
      out["max_iterations_used"] = r.max_iterations_used;
      out["max_ac_thd_pct"] = r.max_ac_thd_pct; out["max_ac_thd_bus"] = r.max_ac_thd_bus;
      out["ac_orders"] = r.ac_orders;
      out["iterations"] = json::object();
      for (const auto& [ord, it] : r.iterations) out["iterations"][std::to_string(ord)] = it;
      out["final_residual"] = json::object();
      for (const auto& [ord, rr] : r.final_residual) out["final_residual"][std::to_string(ord)] = rr;
      out["ac_bus_results"] = json::array();
      for (const auto& b : r.ac_bus_results) out["ac_bus_results"].push_back(hpf_api::bus_json(b));
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // ---- DG Bearing Capability Assessment (DL/T 2041-2019) ----
  svr.Post("/api/session/run_bearing_capacity",
           [](const httplib::Request& req, httplib::Response& res) {
    try {
      hacdcpf::HybridPowerSystem sys;
      {
        std::lock_guard<std::mutex> lk(g_session.mu);
        if (!g_session.current_system) throw std::runtime_error("No system loaded");
        sys = *g_session.current_system;
      }
      if (g_session.busy.exchange(true)) {
        res.status = 409;
        res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
        return;
      }
      g_session.cancel.store(false);

      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      const double kr = j.value("kr", 0.8);   // equipment margin factor k_r
      // Voltage deviation limits (GB/T 12325 defaults for 35-220kV)
      const double delta_UH = j.value("delta_UH_pct", 7.0);  // max positive deviation %
      const double delta_UL = j.value("delta_UL_pct", 7.0);  // max negative deviation %
      // Harmonic limits (GB/T 14549 defaults — total harmonic distortion, %)
      const double thd_limit = j.value("thd_limit_pct", 5.0);

      // ══════════════════════════════════════════════════════════
      // Step 1: Run Power Flow — obtain branch flows and bus voltages
      // ══════════════════════════════════════════════════════════
      hacdcpf::PowerFlowOptions pf_opt;
      pf_opt.max_iter = 100;
      pf_opt.tol = 1e-8;
      auto pf = hacdcpf::solve_power_flow(sys, pf_opt);
      if (!pf.converged) {
        g_session.busy.store(false);
        res.set_content(json{{"error","Power flow did not converge — cannot assess bearing capacity"},
                             {"converged", false}}.dump(), "application/json");
        return;
      }

      // ══════════════════════════════════════════════════════════
      // Step 2: Run Short Circuit (three-phase) at all buses
      // ══════════════════════════════════════════════════════════
      hacdcpf::analysis::SCDetailedOptions sc_opt;
      sc_opt.fault_type = hacdcpf::analysis::FaultType::ThreePhase;
      sc_opt.compute_ith = true;
      std::vector<int> bus_ids;
      std::unordered_map<int, size_t> bus_idx_map; // bus_id → index in buses vector
      std::unordered_map<int, double> bus_kv_map;
      for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
        const auto& b = sys.ac.buses[i];
        if (b.in_service) {
          bus_ids.push_back(b.index);
          bus_idx_map[b.index] = i;
          bus_kv_map[b.index] = b.base_kv;
        }
      }
      auto sc_results = hacdcpf::analysis::run_short_circuit_detailed_batch(sys, bus_ids, sc_opt);
      // Build bus_id → ikss_ka map
      std::unordered_map<int, double> bus_ikss;
      for (const auto& dr : sc_results) {
        for (const auto& br : dr.bus_results) {
          if (br.bus_id == dr.fault_bus_id) {
            bus_ikss[br.bus_id] = br.ikss_ka;
          }
        }
      }

      // ══════════════════════════════════════════════════════════
      // Step 3: Aggregate DG injection and load per bus
      //   P_D = DG output (pv_system, renewable_gen, static_generator)
      //   P_L = conventional load
      // ══════════════════════════════════════════════════════════
      std::unordered_map<int, double> bus_dg_mw;   // total DG P at bus
      std::unordered_map<int, double> bus_dg_qmax;  // max DG Q at bus
      std::unordered_map<int, double> bus_dg_qmin;  // min DG Q at bus
      for (const auto& pv : sys.ac.pv_systems) {
        if (!pv.in_service) continue;
        bus_dg_mw[pv.bus] += pv.p_mw;
        double s = pv.sn_mva > 0 ? pv.sn_mva : std::abs(pv.p_mw) * 1.1;
        double q = std::sqrt(std::max(0.0, s*s - pv.p_mw*pv.p_mw));
        bus_dg_qmax[pv.bus] += q;
        bus_dg_qmin[pv.bus] -= q;
      }
      for (const auto& rg : sys.ac.renewable_gens) {
        if (!rg.in_service) continue;
        bus_dg_mw[rg.bus] += rg.p_mw;
        double s = rg.p_rated_mw > 0 ? rg.p_rated_mw * 1.1 : std::abs(rg.p_mw) * 1.1;
        double q = std::sqrt(std::max(0.0, s*s - rg.p_mw*rg.p_mw));
        bus_dg_qmax[rg.bus] += q;
        bus_dg_qmin[rg.bus] -= q;
      }
      for (const auto& sg : sys.ac.static_generators) {
        if (!sg.in_service) continue;
        bus_dg_mw[sg.bus] += sg.p_mw;
        double s = std::abs(sg.p_mw) * 1.1;
        double q = std::sqrt(std::max(0.0, s*s - sg.p_mw*sg.p_mw));
        bus_dg_qmax[sg.bus] += q;
        bus_dg_qmin[sg.bus] -= q;
      }
      // Conventional loads
      std::unordered_map<int, double> bus_load_mw;
      for (const auto& ld : sys.ac.loads) {
        if (ld.in_service) bus_load_mw[ld.bus] += ld.p_mw;
      }

      // ══════════════════════════════════════════════════════════
      // Step 4: Branch/Transformer Thermal Stability Assessment
      //   DL/T 2041 Eq.(1): λ = (P_D − P_L) / S_e × 100%
      //   DL/T 2041 Eq.(2): P_m = (1 − λ_max) × S_e × k_r
      //
      //   For each branch (line or transformer), the reverse flow
      //   P_reverse = P_D − P_L for the downstream area is the
      //   absolute power flow when direction is from low-kV to high-kV.
      //   We include ALL branches (even those without ratings) so the
      //   user sees a complete thermal stability table.
      // ══════════════════════════════════════════════════════════
      json branch_results = json::array();
      bool any_reverse_220 = false;
      for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
        const auto& br = sys.ac.branches[i];
        if (!br.in_service) continue;

        // Determine equipment type (line vs transformer)
        bool is_trafo = (br.sn_mva > 0.0 || std::abs(br.tap - 1.0) > 1e-6);
        std::string equip_type = is_trafo ? "transformer" : "line";

        // S_e: rated capacity (MVA). Use rate_a_mva first, fall back to sn_mva
        double se = br.rate_a_mva;
        if (se <= 0 && br.sn_mva > 0) se = br.sn_mva;
        bool has_rating = (se > 0);

        // Actual power flow on this branch
        double pf_mw = 0.0, pt_mw = 0.0;
        if (i < pf.branch_flows.size()) {
          pf_mw = pf.branch_flows[i].pf_mw;   // from-bus injection
          pt_mw = pf.branch_flows[i].pt_mw;    // to-bus injection
        }

        // Determine voltage levels to identify flow direction
        double from_kv = bus_kv_map.count(br.from_bus) ? bus_kv_map[br.from_bus] : 0;
        double to_kv = bus_kv_map.count(br.to_bus) ? bus_kv_map[br.to_bus] : 0;

        // Reverse flow: power flowing from low-kV side toward high-kV side
        // This represents P_D − P_L > 0 (DG exceeds load, surplus feeds upstream)
        bool reverse = false;
        double p_reverse_mw = 0.0;
        if (std::abs(from_kv - to_kv) > 0.01) {
          // Different voltage levels (transformer or inter-level line)
          // Normal: high→low. Reverse: low→high.
          if (from_kv > to_kv && pf_mw < 0) {
            reverse = true; p_reverse_mw = std::abs(pf_mw);
          } else if (to_kv > from_kv && pf_mw > 0) {
            reverse = true; p_reverse_mw = std::abs(pf_mw);
          }
        } else {
          // Same voltage level: reverse determined by net DG surplus
          double dg_from = bus_dg_mw.count(br.from_bus) ? bus_dg_mw[br.from_bus] : 0.0;
          double dg_to = bus_dg_mw.count(br.to_bus) ? bus_dg_mw[br.to_bus] : 0.0;
          double ld_from = bus_load_mw.count(br.from_bus) ? bus_load_mw[br.from_bus] : 0.0;
          double ld_to = bus_load_mw.count(br.to_bus) ? bus_load_mw[br.to_bus] : 0.0;
          double net_from = dg_from - ld_from;
          double net_to = dg_to - ld_to;
          if (net_from > net_to && pf_mw > 0) {
            reverse = true; p_reverse_mw = std::abs(pf_mw);
          } else if (net_to > net_from && pf_mw < 0) {
            reverse = true; p_reverse_mw = std::abs(pf_mw);
          }
        }

        // DL/T 2041 Eq.(1): λ = P_reverse / S_e × 100%
        // Only non-zero when flow is reverse
        double lambda_pct = 0.0;
        if (reverse && has_rating) {
          lambda_pct = p_reverse_mw / se * 100.0;
        }

        // Check 220kV+ reverse flow → automatic Red
        if (reverse && (std::max(from_kv, to_kv) >= 220.0)) any_reverse_220 = true;

        // DL/T 2041 Eq.(2): P_m = (1 − λ/100) × S_e × k_r
        double pm = 0.0;
        if (has_rating) {
          pm = std::max(0.0, (1.0 - lambda_pct / 100.0) * se * kr);
        }

        // Branch thermal grade
        std::string thermal_grade = "green";
        if (!has_rating) thermal_grade = "no_rating";
        else if (lambda_pct > 80.0) thermal_grade = "red";
        else if (lambda_pct > 0.0) thermal_grade = "yellow";

        // Loading percentage (absolute flow / rating)
        double loading_pct = (has_rating && se > 0) ?
            std::max(std::abs(pf_mw), std::abs(pt_mw)) / se * 100.0 : 0.0;

        std::string name = br.name.empty() ?
            (equip_type == "transformer" ?
                ("Trafo " + std::to_string(br.index)) :
                ("Branch " + std::to_string(br.index))) :
            br.name;

        branch_results.push_back(json{
          {"name", name}, {"index", br.index}, {"branch_pos", (int)i},
          {"from_bus", br.from_bus}, {"to_bus", br.to_bus},
          {"from_kv", from_kv}, {"to_kv", to_kv},
          {"equip_type", equip_type},
          {"se_mva", se}, {"has_rating", has_rating},
          {"pf_mw", pf_mw}, {"loading_pct", loading_pct},
          {"p_reverse_mw", p_reverse_mw},
          {"lambda_pct", lambda_pct}, {"pm_mw", pm},
          {"reverse_flow", reverse},
          {"thermal_grade", thermal_grade}
        });
      }

      // Also evaluate standalone 2W transformers that were NOT projected to branches
      // (source_branch_idx > 0 means already represented in branches)
      for (size_t ti = 0; ti < sys.ac.transformers_2w.size(); ++ti) {
        const auto& tr = sys.ac.transformers_2w[ti];
        if (!tr.in_service) continue;
        if (tr.source_branch_idx > 0) continue;  // already in branches

        double se = tr.sn_mva;
        bool has_rating = (se > 0);
        double hv_kv = bus_kv_map.count(tr.hv_bus) ? bus_kv_map[tr.hv_bus] : tr.vn_hv_kv;
        double lv_kv = bus_kv_map.count(tr.lv_bus) ? bus_kv_map[tr.lv_bus] : tr.vn_lv_kv;

        // No direct PF result for standalone transformers — estimate from bus DG/load
        double dg_lv = bus_dg_mw.count(tr.lv_bus) ? bus_dg_mw[tr.lv_bus] : 0.0;
        double ld_lv = bus_load_mw.count(tr.lv_bus) ? bus_load_mw[tr.lv_bus] : 0.0;
        double p_net = dg_lv - ld_lv;  // positive = reverse
        bool reverse = (p_net > 0);
        double p_reverse_mw = reverse ? p_net : 0.0;

        double lambda_pct = (reverse && has_rating) ? p_reverse_mw / se * 100.0 : 0.0;
        if (reverse && (std::max(hv_kv, lv_kv) >= 220.0)) any_reverse_220 = true;
        double pm = has_rating ? std::max(0.0, (1.0 - lambda_pct / 100.0) * se * kr) : 0.0;

        std::string thermal_grade = "green";
        if (!has_rating) thermal_grade = "no_rating";
        else if (lambda_pct > 80.0) thermal_grade = "red";
        else if (lambda_pct > 0.0) thermal_grade = "yellow";

        std::string name = tr.name.empty() ?
            ("Trafo2W " + std::to_string(tr.index)) : tr.name;
        branch_results.push_back(json{
          {"name", name}, {"index", tr.index}, {"branch_pos", -1},
          {"from_bus", tr.hv_bus}, {"to_bus", tr.lv_bus},
          {"from_kv", hv_kv}, {"to_kv", lv_kv},
          {"equip_type", "transformer"},
          {"se_mva", se}, {"has_rating", has_rating},
          {"pf_mw", p_net}, {"loading_pct", 0.0},
          {"p_reverse_mw", p_reverse_mw},
          {"lambda_pct", lambda_pct}, {"pm_mw", pm},
          {"reverse_flow", reverse},
          {"thermal_grade", thermal_grade}
        });
      }

      // ══════════════════════════════════════════════════════════
      // Step 5: Bus-level Assessment
      //   - Short circuit check:  I_k'' < I_m  (DL/T 2041 Eq.3)
      //   - Voltage deviation:    δU = (R·P + X·Q)/U² (Eq.4-5)
      //   - Harmonic check:       placeholder (no simulation data)
      // ══════════════════════════════════════════════════════════
      json bus_results = json::array();
      for (const auto& b : sys.ac.buses) {
        if (!b.in_service) continue;
        double un = b.base_kv;
        // PF voltage
        auto idx_it = bus_idx_map.find(b.index);
        size_t bidx = (idx_it != bus_idx_map.end()) ? idx_it->second : 0;
        double vm = (bidx < pf.vm.size()) ? pf.vm[bidx] : 1.0;
        double vm_deviation_pct = (vm - 1.0) * 100.0;

        // ── Short circuit check: DL/T 2041 Eq.(3) ──
        double ikss = bus_ikss.count(b.index) ? bus_ikss[b.index] : 0.0;
        double i_limit = b.i_breaker_ka;
        bool sc_pass = (i_limit <= 0) || (ikss < i_limit);

        // ── Voltage deviation: DL/T 2041 Eq.(4-5) ──
        double dg_p = bus_dg_mw.count(b.index) ? bus_dg_mw[b.index] : 0.0;
        double dg_qmax = bus_dg_qmax.count(b.index) ? bus_dg_qmax[b.index] : 0.0;
        double dg_qmin = bus_dg_qmin.count(b.index) ? bus_dg_qmin[b.index] : 0.0;
        double delta_u_h_pct = 0.0, delta_u_l_pct = 0.0;
        if (ikss > 0 && un > 0) {
          // Thevenin impedance from SC: Z_th = U_N / (√3 · Ik'')
          double z_th = un / (std::sqrt(3.0) * ikss); // ohm
          double rx = 0.1; // typical R/X ratio for HV grid
          double x_th = z_th / std::sqrt(1.0 + rx * rx);
          double r_th = rx * x_th;
          // Eq.(4): δU_H = (R_th·P_DG + X_th·Q_max) / U_N² × 100%
          delta_u_h_pct = (r_th * dg_p + x_th * dg_qmax) / (un * un) * 100.0;
          // Eq.(4): δU_L = |(R_th·P_DG + X_th·Q_min)| / U_N² × 100%
          delta_u_l_pct = std::abs((r_th * dg_p + x_th * dg_qmin) / (un * un) * 100.0);
        }
        bool voltage_pass = (delta_u_h_pct < delta_UH) && (delta_u_l_pct < delta_UL);

        // ── Harmonic check: DL/T 2041 Eq.(6) ──
        // Requires measured harmonic data (I_h per bus). Since the simulator
        // does not compute harmonics, we report "no_data" and default to pass
        // if no harmonic data is provided by the user.
        bool harmonic_pass = true;
        std::string harmonic_status = "no_data"; // "pass", "fail", "no_data"

        // ── Per-bus grade ──
        std::string grade = "green";
        if (!sc_pass || !voltage_pass || !harmonic_pass) grade = "red";

        bus_results.push_back(json{
          {"bus_id", b.index}, {"name", b.name}, {"base_kv", un}, {"area", b.area},
          {"vm_pu", vm}, {"vm_deviation_pct", vm_deviation_pct},
          {"ikss_ka", ikss}, {"i_breaker_ka", i_limit}, {"sc_pass", sc_pass},
          {"dg_mw", dg_p},
          {"delta_u_h_pct", delta_u_h_pct}, {"delta_u_l_pct", delta_u_l_pct},
          {"delta_UH_limit", delta_UH}, {"delta_UL_limit", delta_UL},
          {"voltage_pass", voltage_pass},
          {"harmonic_pass", harmonic_pass}, {"harmonic_status", harmonic_status},
          {"thd_limit_pct", thd_limit},
          {"grade", grade}
        });
      }

      // ══════════════════════════════════════════════════════════
      // Step 6: Area/Zone Grading (DL/T 2041 Table 1)
      //   Green:  λ ≤ 0 for all, SC/voltage/harmonic pass
      //   Yellow: 0 < λ ≤ 80%, SC/voltage/harmonic pass
      //   Red:    λ > 80%, or any check fails, or reverse to 220kV+
      //   Lower-level inherits worst upper-level grade.
      // ══════════════════════════════════════════════════════════
      std::map<int, std::string> area_grade;
      // Bus-level → area
      for (const auto& bj : bus_results) {
        int area = bj.value("area", 1);
        std::string bg = bj["grade"].get<std::string>();
        auto& ag = area_grade[area];
        if (ag.empty()) ag = bg;
        else if (bg == "red") ag = "red";
        else if (bg == "yellow" && ag != "red") ag = "yellow";
      }
      // Branch thermal → area
      for (const auto& bj : branch_results) {
        std::string tg = bj.value("thermal_grade", std::string("green"));
        if (tg == "no_rating") continue;
        int from = bj["from_bus"].get<int>();
        int area_from = 1;
        for (const auto& b : sys.ac.buses) {
          if (b.index == from) { area_from = b.area; break; }
        }
        auto& ag = area_grade[area_from];
        if (tg == "red") ag = "red";
        else if (tg == "yellow" && ag == "green") ag = "yellow";
      }
      // 220kV+ reverse → all areas red
      if (any_reverse_220) {
        for (auto& [a, g] : area_grade) g = "red";
      }

      json area_results = json::array();
      for (const auto& [a, g] : area_grade) {
        area_results.push_back(json{{"area", a}, {"grade", g}});
      }

      // ══════════════════════════════════════════════════════════
      // Step 7: Comprehensive Summary Table (DL/T 2041 Appendix C)
      // Per-element: thermal λ, SC check, voltage check, harmonic, grade, P_m
      // ══════════════════════════════════════════════════════════
      json summary_table = json::array();
      // Add branch entries
      for (const auto& bj : branch_results) {
        // Find worst bus-level checks for the buses connected to this branch
        int from_id = bj["from_bus"].get<int>();
        int to_id = bj["to_bus"].get<int>();
        bool br_sc_pass = true, br_volt_pass = true, br_harm_pass = true;
        std::string br_harm_status = "no_data";
        for (const auto& bres : bus_results) {
          int bid = bres["bus_id"].get<int>();
          if (bid == from_id || bid == to_id) {
            if (!bres["sc_pass"].get<bool>()) br_sc_pass = false;
            if (!bres["voltage_pass"].get<bool>()) br_volt_pass = false;
            if (!bres["harmonic_pass"].get<bool>()) br_harm_pass = false;
            if (bres["harmonic_status"].get<std::string>() != "no_data")
              br_harm_status = bres["harmonic_status"].get<std::string>();
          }
        }
        std::string tg = bj.value("thermal_grade", std::string("green"));
        // Composite grade
        std::string grade = tg;
        if (!br_sc_pass || !br_volt_pass || !br_harm_pass) grade = "red";
        else if (tg == "no_rating") grade = "green"; // no thermal constraint
        if (any_reverse_220 && std::max(bj["from_kv"].get<double>(),
                                         bj["to_kv"].get<double>()) >= 220.0)
          grade = "red";

        summary_table.push_back(json{
          {"name", bj["name"]},
          {"type", bj["equip_type"]},
          {"lambda_pct", bj["lambda_pct"]},
          {"has_rating", bj["has_rating"]},
          {"sc_pass", br_sc_pass},
          {"voltage_pass", br_volt_pass},
          {"harmonic_pass", br_harm_pass},
          {"harmonic_status", br_harm_status},
          {"grade", grade},
          {"pm_mw", bj["pm_mw"]}
        });
      }
      // Add bus entries
      for (const auto& bj : bus_results) {
        double bus_lam = 0.0;
        // Find worst λ among branches connected to this bus
        int bid = bj["bus_id"].get<int>();
        for (const auto& brj : branch_results) {
          if (brj["from_bus"].get<int>() == bid || brj["to_bus"].get<int>() == bid) {
            bus_lam = std::max(bus_lam, brj["lambda_pct"].get<double>());
          }
        }
        std::string grade = bj["grade"].get<std::string>();
        // Incorporate thermal grade
        if (bus_lam > 80.0) grade = "red";
        else if (bus_lam > 0.0 && grade == "green") grade = "yellow";

        summary_table.push_back(json{
          {"name", bj.value("name", std::string("")) + " (Bus " +
                   std::to_string(bj["bus_id"].get<int>()) + ")"},
          {"type", "bus"},
          {"lambda_pct", bus_lam},
          {"has_rating", true},
          {"sc_pass", bj["sc_pass"]},
          {"voltage_pass", bj["voltage_pass"]},
          {"harmonic_pass", bj["harmonic_pass"]},
          {"harmonic_status", bj["harmonic_status"]},
          {"grade", grade},
          {"pm_mw", 0.0}
        });
      }

      json out;
      out["converged"] = true;
      out["pf_iterations"] = pf.iterations;
      out["pf_residual"] = pf.residual;
      out["kr"] = kr;
      out["delta_UH_limit"] = delta_UH;
      out["delta_UL_limit"] = delta_UL;
      out["thd_limit_pct"] = thd_limit;
      out["any_reverse_220kv"] = any_reverse_220;
      out["branch_results"] = branch_results;
      out["bus_results"] = bus_results;
      out["area_results"] = area_results;
      out["summary_table"] = summary_table;
      res.set_content(out.dump(), "application/json");
      g_session.busy.store(false);
    } catch (const std::exception& e) {
      g_session.busy.store(false);
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

    // ---- Detailed Short Circuit at Selected Buses ----
    svr.Post("/api/session/sc_detailed",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        std::vector<int> fault_bus_ids;
        if (j.contains("fault_bus_ids") && j["fault_bus_ids"].is_array())
          fault_bus_ids = j["fault_bus_ids"].get<std::vector<int>>();
        if (fault_bus_ids.empty()) throw std::runtime_error("No fault_bus_ids specified");
        hacdcpf::analysis::SCDetailedOptions dopt;
        const std::string ft = j.value("fault_type", std::string("ThreePhase"));
        if (ft=="SinglePhaseGround") dopt.fault_type=hacdcpf::analysis::FaultType::SinglePhaseGround;
        else if (ft=="TwoPhase") dopt.fault_type=hacdcpf::analysis::FaultType::TwoPhase;
        else if (ft=="TwoPhaseGround") dopt.fault_type=hacdcpf::analysis::FaultType::TwoPhaseGround;
        else dopt.fault_type=hacdcpf::analysis::FaultType::ThreePhase;
        dopt.compute_branch_flows = true;
        dopt.compute_voltage_drops = true;
        dopt.compute_ith = true;
        auto results = hacdcpf::analysis::run_short_circuit_detailed_batch(sys, fault_bus_ids, dopt);
        json out;
        out["fault_type"] = ft;
        json res_arr = json::array();
        for (const auto& dr : results) {
          json rj;
          rj["fault_bus_id"] = dr.fault_bus_id;
          rj["solved"] = dr.solved;
          json bus_arr = json::array();
          for (const auto& br : dr.bus_results) {
            bus_arr.push_back({
              {"bus_id", br.bus_id}, {"ikss_ka", br.ikss_ka},
              {"ip_ka", br.ip_ka}, {"ib_ka", br.ib_ka},
              {"ik_ka", br.ik_ka}, {"ith_ka", br.ith_ka},
              {"v_remaining_pu", br.v_remaining_pu},
              {"ikss_gen_contrib_ka", br.ikss_gen_contrib_ka},
              {"ikss_motor_contrib_ka", br.ikss_motor_contrib_ka},
              {"ikss_load_contrib_ka", br.ikss_load_contrib_ka},
              {"ikss_sgen_contrib_ka", br.ikss_sgen_contrib_ka},
              {"ikss_extgrid_contrib_ka", br.ikss_extgrid_contrib_ka},
              {"ikss_converter_contrib_ka", br.ikss_converter_contrib_ka},
            });
          }
          rj["bus_results"] = bus_arr;
          res_arr.push_back(rj);
        }
        out["results"] = res_arr;
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Advanced analysis endpoints ----
    // ---- Time-series configuration ----

    // Helper: materialize loads from bus.pd_mw/qd_mvar (idempotent — only
    // when ac.loads is empty, which is the case for any fresh MATPOWER
    // parse) and re-apply a TsBindingSpec to the given system. Used by
    // BOTH set_ts_config (mutates g_session.current_system) and run_ts_pf
    // (mutates the per-call sys_ts copy, since canvas re-sync between
    // set_ts_config and run_ts_pf would otherwise wipe profile_ids).
    auto materialize_loads_and_apply_binding =
        [](hacdcpf::HybridPowerSystem& sys,
           const Session::TsBindingSpec& spec) -> int {
      int n_materialized = 0;
      if (sys.ac.loads.empty()) {
        int li = 0;
        for (const auto& b : sys.ac.buses) {
          if (!b.in_service) continue;
          if (std::fabs(b.pd_mw) < 1e-12 && std::fabs(b.qd_mvar) < 1e-12) continue;
          hacdcpf::Load ld;
          ld.index = li;
          ld.bus = b.index;
          ld.in_service = true;
          ld.name = "load_bus" + std::to_string(b.index);
          ld.p_mw = b.pd_mw;
          ld.q_mvar = b.qd_mvar;
          ld.scaling = 1.0;
          sys.ac.loads.push_back(std::move(ld));
          ++li;
        }
        for (auto& b : sys.ac.buses) { b.pd_mw = 0.0; b.qd_mvar = 0.0; }
        n_materialized = li;
      }
      if (!spec.valid) return n_materialized;
      auto& loads = sys.ac.loads;
      for (const auto& [li, pid] : spec.map_by_load_index) {
        if (li >= 0 && li < (int)loads.size()) loads[li].profile_id = pid;
      }
      for (const auto& [bus, pid] : spec.map_by_bus) {
        for (auto& ld : loads)
          if (ld.bus == bus) { ld.profile_id = pid; break; }
      }
      if (spec.assign_all_loads_to >= 0)
        for (auto& ld : loads) ld.profile_id = spec.assign_all_loads_to;
      if (spec.assign_all_pv_to >= 0) {
        for (auto& pv : sys.ac.pv_systems) pv.profile_id = spec.assign_all_pv_to;
        for (auto& pv : sys.dc.pv_arrays) pv.profile_id = spec.assign_all_pv_to;
      }
      return n_materialized;
    };

    svr.Post("/api/session/set_ts_config",
             [materialize_loads_and_apply_binding]
             (const httplib::Request& req, httplib::Response& res) {
      try {
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        std::lock_guard<std::mutex> lk(g_session.mu);
        int num_steps = j.value("num_steps", 24);
        double step_hr = j.value("step_duration_hr", 1.0);
        if (j.contains("profiles") && j["profiles"].is_array()) {
          hacdcpf::TimeSeriesData ts;
          ts.num_steps = num_steps;
          ts.step_duration_hr = step_hr;
          for (const auto& p : j["profiles"]) {
            hacdcpf::TimeSeriesProfile prof;
            prof.id = p.value("id", 0);
            prof.name = p.value("name", std::string{});
            if (p.contains("values") && p["values"].is_array())
              prof.values = p["values"].get<std::vector<double>>();
            ts.profiles.push_back(prof);
          }
          g_session.ts_data = std::move(ts);
        } else {
          g_session.ts_data = make_default_ts_data(num_steps);
        }
        // Optional per-load mapping: bind each load to a specific profile id.
        // Accepts either {load_index, profile_id} or {bus, profile_id}.
        int n_load_mapped = 0;
        int n_loads_materialized = 0;

        // Build / refresh persistent binding spec from this request so that
        // a later canvas re-sync (load_json_string) before run_ts_pf can be
        // re-applied transparently.
        Session::TsBindingSpec spec;
        spec.valid = true;
        if (j.contains("load_profile_map") && j["load_profile_map"].is_array()) {
          for (const auto& m : j["load_profile_map"]) {
            int pid = m.value("profile_id", -1);
            if (pid < 0) continue;
            if (m.contains("load_index")) {
              int li = m.value("load_index", -1);
              if (li >= 0) spec.map_by_load_index.emplace_back(li, pid);
            } else if (m.contains("bus")) {
              int bus = m.value("bus", -1);
              if (bus >= 0) spec.map_by_bus.emplace_back(bus, pid);
            }
          }
        }
        if (j.contains("assign_all_loads_to"))
          spec.assign_all_loads_to = j.value("assign_all_loads_to", -1);
        if (j.contains("assign_all_pv_to"))
          spec.assign_all_pv_to = j.value("assign_all_pv_to", -1);
        g_session.ts_binding = spec;

        if (g_session.current_system) {
          n_loads_materialized = materialize_loads_and_apply_binding(
              *g_session.current_system, spec);
          // Count actual mapped loads (those whose profile_id matches the spec).
          for (const auto& ld : g_session.current_system->ac.loads)
            if (ld.profile_id >= 0) ++n_load_mapped;
        }
        json out;
        out["num_steps"] = g_session.ts_data.num_steps;
        out["step_duration_hr"] = g_session.ts_data.step_duration_hr;
        out["num_profiles"] = (int)g_session.ts_data.profiles.size();
        out["num_loads_mapped"] = n_load_mapped;
        out["num_loads_materialized"] = n_loads_materialized;
        json pnames = json::array();
        for (const auto& p : g_session.ts_data.profiles) pnames.push_back(p.name);
        out["profile_names"] = pnames;
        res.set_content(out.dump(), "application/json");
      } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Time-series PF pipeline (UC → PF) ----
    svr.Post("/api/session/run_ts_pf",
             [materialize_loads_and_apply_binding]
             (const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys_ts;
        hacdcpf::TimeSeriesData ts_data;
        Session::TsBindingSpec spec;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          const auto j2 = json::parse(req.body.empty() ? "{}" : req.body);
          const int ns = j2.value("num_steps", 24);
          if (g_session.ts_data.num_steps != ns || g_session.ts_data.profiles.empty())
            g_session.ts_data = make_default_ts_data(ns);
          sys_ts = *g_session.current_system;
          ts_data = g_session.ts_data;
          spec = g_session.ts_binding;
        }
        // Re-apply persistent binding to the per-call sys copy. This
        // protects against the common GUI flow where a canvas re-sync
        // (load_json_string) replaces current_system between set_ts_config
        // and run_ts_pf — which would otherwise wipe out the materialized
        // loads + profile_id assignments and silently fall back to a
        // constant-load solve.
        int n_remat = materialize_loads_and_apply_binding(sys_ts, spec);
        // Fold DC-side storage into the engine's Storage-typed dc.storage path
        // so it participates in the time-series solve AND appears in the
        // dc_ess_* name/dispatch result arrays built below. solve_time_series_pf
        // also materializes internally; this is idempotent.
        hacdcpf::materialize_dc_storage(sys_ts);
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const int num_steps = j.value("num_steps", 24);
        const bool skip_uc = j.value("skip_uc", false);
        const bool run_opf = j.value("run_opf", false);
        // Auto-assign profiles to components without explicit ids
        for (auto& ld : sys_ts.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        for (auto& ren : sys_ts.ac.renewable_gens) {
          if (ren.profile_id < 0)
            ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                              ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
        }
        // Auto-assign solar profile to PV systems and DC PV arrays
        for (auto& pv : sys_ts.ac.pv_systems) if (pv.profile_id < 0) pv.profile_id = 2;
        for (auto& pv : sys_ts.dc.pv_arrays) if (pv.profile_id < 0) pv.profile_id = 2;
        // Auto-assign load profile to DC loads
        for (auto& ld : sys_ts.dc.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        hacdcpf::TimeSeriesPFOptions opts;
        opts.skip_uc = skip_uc;
        opts.run_opf = run_opf;
        opts.keep_system_snapshots = true;
        opts.verbose = false;
        auto result = hacdcpf::solve_time_series_pf(sys_ts, ts_data, opts);
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          g_session.last_tspf_result = result;
          g_session.last_tspf_data = ts_data;
          g_session.last_tspf_skip_uc = skip_uc;
          g_session.last_tspf_run_opf = run_opf;
          g_session.last_tspf_num_steps = result.num_steps;
        }
        json out;
        out["num_steps"] = result.num_steps;
        out["num_converged"] = result.num_converged;
        out["num_opf_converged"] = result.num_opf_converged;
        out["total_generation_cost"] = result.total_generation_cost;
        out["uc_feasible"] = result.uc_schedule.feasible;
        out["uc_solver_name"] = result.uc_schedule.solver_name;
        // Diagnostic: how many loads were materialized at PF time and how
        // many ended up bound to a profile id. Helps catch silent mapping
        // failures from the GUI side.
        {
          int n_bound = 0;
          for (const auto& ld : sys_ts.ac.loads) if (ld.profile_id >= 0) ++n_bound;
          out["num_loads"] = (int)sys_ts.ac.loads.size();
          out["num_loads_materialized"] = n_remat;
          out["num_loads_bound_to_profile"] = n_bound;
          out["ts_binding_active"] = spec.valid;
        }
        // Per-step metrics
        json vm_mean = json::array(), vm_min_arr = json::array(), vm_max_arr = json::array(), losses_mw = json::array();
        for (const auto& pf : result.pf_results) {
          double vmm = 0.0, vmin = 1e30, vmax = -1e30;
          if (!pf.vm.empty()) {
            for (double v : pf.vm) { vmm += v; vmin = std::min(vmin, v); vmax = std::max(vmax, v); }
            vmm /= (double)pf.vm.size();
          } else { vmin = 0.0; vmax = 0.0; }
          vm_mean.push_back(vmm);
          vm_min_arr.push_back(vmin);
          vm_max_arr.push_back(vmax);
          // Total system losses = AC branch losses + VSC converter losses + DC-DC losses
          double loss = 0.0;
          for (const auto& bf : pf.branch_flows) loss += (bf.pf_mw + bf.pt_mw);
          for (const auto& vt : pf.vsc_transfers) loss += vt.loss_mw;
          for (const auto& dt2 : pf.dcdc_transfers) loss += dt2.loss_mw;
          losses_mw.push_back(std::max(loss, 0.0));
        }
        while ((int)vm_mean.size() < result.num_steps) { vm_mean.push_back(0.0); vm_min_arr.push_back(0.0); vm_max_arr.push_back(0.0); losses_mw.push_back(0.0); }
        out["vm_mean"] = vm_mean;
        out["vm_min"] = vm_min_arr;
        out["vm_max"] = vm_max_arr;
        out["losses_mw"] = losses_mw;
        // UC schedule
        const auto& uc = result.uc_schedule;
        out["gen_dispatch"] = uc.gen_dispatch;
        out["gen_commit"] = uc.gen_commit;
        out["ess_dispatch"] = uc.ess_dispatch;
        out["ess_soc"] = uc.ess_soc;
        json renewable_dispatch = uc.renewable_dispatch;
        int n_ren_active = 0;
        for (const auto& ren : sys_ts.ac.renewable_gens)
          if (ren.in_service) ++n_ren_active;
        if (!renewable_dispatch.is_array() || static_cast<int>(renewable_dispatch.size()) != n_ren_active) {
          std::unordered_map<int, const hacdcpf::TimeSeriesProfile*> pmap;
          for (const auto& p : ts_data.profiles) pmap[p.id] = &p;
          renewable_dispatch = json::array();
          for (const auto& ren : sys_ts.ac.renewable_gens) {
            if (!ren.in_service) continue;
            double cap = ren.p_rated_mw > 1e-9 ? ren.p_rated_mw : ren.p_mw;
            json row = json::array();
            const auto it = pmap.find(ren.profile_id);
            for (int t = 0; t < result.num_steps; ++t) {
              double scale = 1.0;
              if (it != pmap.end() && t < static_cast<int>(it->second->values.size()))
                scale = it->second->values[static_cast<size_t>(t)];
              row.push_back(std::max(0.0, cap * scale));
            }
            renewable_dispatch.push_back(row);
          }
        }
        out["renewable_dispatch"] = renewable_dispatch;
        // DC-side dispatch
        out["dc_pv_dispatch"] = uc.dc_pv_dispatch;
        out["dc_ess_dispatch"] = uc.dc_ess_dispatch;
        out["dc_sgen_dispatch"] = uc.dc_sgen_dispatch;
        out["dc_load_demand"] = uc.dc_load_demand;
        // AC PV dispatch — extract from OPF results when available,
        // otherwise compute from solar profile for each timestep
        {
          std::unordered_map<int, const hacdcpf::TimeSeriesProfile*> pmap;
          for (const auto& p : ts_data.profiles) pmap[p.id] = &p;

          // Build an in-service PV index list (parallels the JSON output rows)
          std::vector<size_t> pv_service_idx;
          for (size_t pi = 0; pi < sys_ts.ac.pv_systems.size(); ++pi)
            if (sys_ts.ac.pv_systems[pi].in_service)
              pv_service_idx.push_back(pi);

          json ac_pv_dispatch = json::array();
          for (size_t si = 0; si < pv_service_idx.size(); ++si) {
            const size_t pi = pv_service_idx[si];
            const auto& pvsys = sys_ts.ac.pv_systems[pi];
            json row = json::array();
            for (int t = 0; t < result.num_steps; ++t) {
              // 1) Compute the profile-scaled (MPPT) baseline
              double scale = 1.0;
              if (pvsys.profile_id >= 0) {
                auto it2 = pmap.find(pvsys.profile_id);
                if (it2 != pmap.end() && t < (int)it2->second->values.size())
                  scale = it2->second->values[t];
              }
              double pv_mw;
              if (pvsys.voc > 0 && pvsys.isc > 0 && pvsys.vmpp > 0) {
                hacdcpf::PVSystem pv_copy = pvsys;
                pv_copy.irradiance = scale * 1000.0;
                pv_mw = hacdcpf::powerflow::compute_pv_power_mw(pv_copy);
              } else {
                pv_mw = pvsys.p_mw * scale;
              }

              // 2) If OPF ran and converged, override with OPF dispatch
              if (run_opf && t < (int)result.opf_results.size() &&
                  result.opf_results[t].converged) {
                const auto& opf_res = result.opf_results[t];
                for (size_t k = 0; k < opf_res.pren_mw.size() &&
                                    k < opf_res.ren_map.size(); ++k) {
                  if (opf_res.ren_map[k].source_type == 1 &&
                      opf_res.ren_map[k].original_index == (int)pi) {
                    pv_mw = opf_res.pren_mw[k];
                    break;
                  }
                }
              }
              row.push_back(pv_mw);
            }
            ac_pv_dispatch.push_back(row);
          }
          out["ac_pv_dispatch"] = ac_pv_dispatch;
        }
        // Component names — only in-service components to match UC dispatch indices
        json gn = json::array(), rn = json::array(), en = json::array();
        for (size_t i = 0; i < sys_ts.ac.generators.size(); ++i)
          if (sys_ts.ac.generators[i].in_service)
            gn.push_back(sys_ts.ac.generators[i].name.empty() ? "Gen"+std::to_string(i) : sys_ts.ac.generators[i].name);
        for (size_t i = 0; i < sys_ts.ac.renewable_gens.size(); ++i)
          if (sys_ts.ac.renewable_gens[i].in_service)
            rn.push_back(sys_ts.ac.renewable_gens[i].name.empty() ? "Ren"+std::to_string(i) : sys_ts.ac.renewable_gens[i].name);
        for (size_t i = 0; i < sys_ts.ac.storage.size(); ++i)
          if (sys_ts.ac.storage[i].in_service)
            en.push_back(sys_ts.ac.storage[i].name.empty() ? "ESS"+std::to_string(i) : sys_ts.ac.storage[i].name);
        out["gen_names"] = gn; out["ren_names"] = rn; out["ess_names"] = en;
        // PV system names
        json pvn = json::array();
        for (size_t i = 0; i < sys_ts.ac.pv_systems.size(); ++i)
          pvn.push_back(sys_ts.ac.pv_systems[i].name.empty() ? "PV"+std::to_string(i) : sys_ts.ac.pv_systems[i].name);
        out["pv_names"] = pvn;
        // DC component names
        json dcpv = json::array(), dcess = json::array();
        for (size_t i = 0; i < sys_ts.dc.pv_arrays.size(); ++i)
          dcpv.push_back(sys_ts.dc.pv_arrays[i].name.empty() ? "DC_PV"+std::to_string(i) : sys_ts.dc.pv_arrays[i].name);
        for (size_t i = 0; i < sys_ts.dc.storage.size(); ++i)
          dcess.push_back(sys_ts.dc.storage[i].name.empty() ? "DC_ESS"+std::to_string(i) : sys_ts.dc.storage[i].name);
        out["dc_pv_names"] = dcpv; out["dc_ess_names"] = dcess;
        // Per-step per-bus voltage matrix (vm) and angle matrix (va, radians)
        {
          json vm_matrix = json::array();
          json va_matrix = json::array();
          json bus_labels = json::array();
          for (size_t b = 0; b < sys_ts.ac.buses.size(); ++b) {
            bus_labels.push_back(sys_ts.ac.buses[b].name.empty()
              ? ("Bus" + std::to_string(sys_ts.ac.buses[b].index))
              : sys_ts.ac.buses[b].name);
            json vm_row = json::array();
            json va_row = json::array();
            for (int t = 0; t < result.num_steps; ++t) {
              double vm = 1.0, va = 0.0;
              if (t < (int)result.pf_results.size()) {
                const auto& pf = result.pf_results[t];
                if (b < pf.vm.size()) vm = pf.vm[b];
                if (b < pf.va.size()) va = pf.va[b];
              }
              vm_row.push_back(vm);
              va_row.push_back(va);
            }
            vm_matrix.push_back(vm_row);
            va_matrix.push_back(va_row);
          }
          out["vm_matrix"] = vm_matrix;
          out["va_matrix"] = va_matrix;
          out["bus_labels"] = bus_labels;
        }
        // Per-step per-branch flows (Pf, Pt, Qf, Qt in MW / MVAr) — ordered
        // identically to sys.ac.branches so labels can be zipped client-side.
        {
          json pf_mat = json::array(), pt_mat = json::array();
          json qf_mat = json::array(), qt_mat = json::array();
          json branch_labels = json::array();
          json branch_from = json::array(), branch_to = json::array();
          const size_t nb = sys_ts.ac.branches.size();
          for (size_t i = 0; i < nb; ++i) {
            const auto& br = sys_ts.ac.branches[i];
            branch_labels.push_back(br.name.empty()
              ? ("Br" + std::to_string(br.index)) : br.name);
            branch_from.push_back(br.from_bus);
            branch_to.push_back(br.to_bus);
            json pfr = json::array(), ptr = json::array();
            json qfr = json::array(), qtr = json::array();
            for (int t = 0; t < result.num_steps; ++t) {
              double pfv = 0.0, ptv = 0.0, qfv = 0.0, qtv = 0.0;
              if (t < (int)result.pf_results.size() &&
                  i < result.pf_results[t].branch_flows.size()) {
                const auto& bf = result.pf_results[t].branch_flows[i];
                pfv = bf.pf_mw; ptv = bf.pt_mw;
                qfv = bf.qf_mvar; qtv = bf.qt_mvar;
              }
              pfr.push_back(pfv); ptr.push_back(ptv);
              qfr.push_back(qfv); qtr.push_back(qtv);
            }
            pf_mat.push_back(pfr); pt_mat.push_back(ptr);
            qf_mat.push_back(qfr); qt_mat.push_back(qtr);
          }
          out["branch_pf_mw"] = pf_mat;
          out["branch_pt_mw"] = pt_mat;
          out["branch_qf_mvar"] = qf_mat;
          out["branch_qt_mvar"] = qt_mat;
          out["branch_labels"] = branch_labels;
          out["branch_from_bus"] = branch_from;
          out["branch_to_bus"]   = branch_to;
        }
        // Total load per timestep (from profile scaling)
        {
          std::unordered_map<int, const hacdcpf::TimeSeriesProfile*> pmap;
          for (const auto& p : ts_data.profiles) pmap[p.id] = &p;
          auto get_scale = [&](int pid, int t) -> double {
            auto it = pmap.find(pid);
            if (it == pmap.end()) return 1.0;
            if (t < 0 || t >= (int)it->second->values.size()) return 1.0;
            return it->second->values[t];
          };
          json total_load = json::array();
          json load_demand = json::array();
          json load_names = json::array();
          json load_bus_indices = json::array();
          if (!sys_ts.ac.loads.empty()) {
            for (size_t li = 0; li < sys_ts.ac.loads.size(); ++li) {
              const auto& ld = sys_ts.ac.loads[li];
              json row = json::array();
              for (int t = 0; t < result.num_steps; ++t) {
                row.push_back(ld.in_service ? ld.p_mw * ld.scaling * get_scale(ld.profile_id, t) : 0.0);
              }
              load_demand.push_back(row);
              load_names.push_back(ld.name.empty() ? ("Load" + std::to_string(ld.index ? ld.index : (int)li)) : ld.name);
              load_bus_indices.push_back(ld.bus);
            }
          } else {
            for (size_t bi = 0; bi < sys_ts.ac.buses.size(); ++bi) {
              const auto& b = sys_ts.ac.buses[bi];
              if (!b.in_service || std::max(0.0, b.pd_mw) <= 1e-12) continue;
              json row = json::array();
              for (int t = 0; t < result.num_steps; ++t) row.push_back(std::max(0.0, b.pd_mw) * get_scale(0, t));
              load_demand.push_back(row);
              load_names.push_back(b.name.empty() ? ("BusLoad" + std::to_string(b.index)) : (b.name + " Load"));
              load_bus_indices.push_back(b.index);
            }
          }
          for (int t = 0; t < result.num_steps; ++t) {
            double tl = 0.0;
            if (!sys_ts.ac.loads.empty()) {
              for (const auto& ld : sys_ts.ac.loads) {
                if (!ld.in_service) continue;
                tl += ld.p_mw * ld.scaling * get_scale(ld.profile_id, t);
              }
            } else {
              double s0 = get_scale(0, t);
              for (const auto& b : sys_ts.ac.buses) {
                if (!b.in_service) continue;
                tl += std::max(0.0, b.pd_mw) * s0;
              }
            }
            for (const auto& dl : sys_ts.dc.loads) {
              if (!dl.in_service) continue;
              tl += dl.p_mw * dl.scaling * get_scale(dl.profile_id, t);
            }
            total_load.push_back(tl);
          }
          out["load_demand"] = load_demand;
          out["load_names"] = load_names;
          out["load_bus_indices"] = load_bus_indices;
          out["total_load"] = total_load;
        }
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      } catch (...) {
        g_session.busy.store(false);
        res.status = 500;
        res.set_content(json{{"error", "Unknown internal error in time-series PF"}}.dump(), "application/json");
      }
    });

    // ---- Unit Commitment only ----
    svr.Post("/api/session/run_uc",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys_ts;
        hacdcpf::TimeSeriesData ts_data;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          const auto j2 = json::parse(req.body.empty() ? "{}" : req.body);
          const int ns = j2.value("num_steps", 24);
          if (g_session.ts_data.num_steps != ns || g_session.ts_data.profiles.empty())
            g_session.ts_data = make_default_ts_data(ns);
          sys_ts = *g_session.current_system;
          ts_data = g_session.ts_data;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const int num_steps = j.value("num_steps", 24);
        for (auto& ld : sys_ts.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        for (auto& ren : sys_ts.ac.renewable_gens) {
          if (ren.profile_id < 0)
            ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                              ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
        }
        for (auto& pv : sys_ts.ac.pv_systems) if (pv.profile_id < 0) pv.profile_id = 2;
        for (auto& pv : sys_ts.dc.pv_arrays) if (pv.profile_id < 0) pv.profile_id = 2;
        for (auto& ld : sys_ts.dc.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        hacdcpf::TimeSeriesPFOptions opts; opts.verbose = false;
        auto uc = hacdcpf::solve_unit_commitment(sys_ts, ts_data, opts);
        json out;
        out["feasible"] = uc.feasible; out["total_cost"] = uc.total_cost;
        out["gen_dispatch"] = uc.gen_dispatch; out["gen_commit"] = uc.gen_commit;
        out["ess_dispatch"] = uc.ess_dispatch; out["ess_soc"] = uc.ess_soc;
        out["renewable_dispatch"] = uc.renewable_dispatch;
        json gn = json::array(), rn = json::array(), en = json::array();
        for (size_t i = 0; i < sys_ts.ac.generators.size(); ++i)
          if (sys_ts.ac.generators[i].in_service)
            gn.push_back(sys_ts.ac.generators[i].name.empty() ? "Gen"+std::to_string(i) : sys_ts.ac.generators[i].name);
        for (size_t i = 0; i < sys_ts.ac.renewable_gens.size(); ++i)
          if (sys_ts.ac.renewable_gens[i].in_service)
            rn.push_back(sys_ts.ac.renewable_gens[i].name.empty() ? "Ren"+std::to_string(i) : sys_ts.ac.renewable_gens[i].name);
        for (size_t i = 0; i < sys_ts.ac.storage.size(); ++i)
          if (sys_ts.ac.storage[i].in_service)
            en.push_back(sys_ts.ac.storage[i].name.empty() ? "ESS"+std::to_string(i) : sys_ts.ac.storage[i].name);
        out["gen_names"] = gn; out["ren_names"] = rn; out["ess_names"] = en;
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      } catch (...) {
        g_session.busy.store(false);
        res.status = 500;
        res.set_content(json{{"error", "Unknown internal error in unit commitment"}}.dump(), "application/json");
      }
    });

    // ---- Market Clearing (SCUC -> SCED -> LMP -> Settlement) ----
    svr.Post("/api/session/run_market_clearing",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }

        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }

        // NOTE (simulation build): the full market-clearing pipeline
        // (SCED + LMP + settlement) from HybridACDCPowerSystemsPlanning is a
        // planning-only module and is not part of this distribution-simulation
        // backend. We honour the request by running the SCUC unit-commitment
        // stage (solve_unit_commitment) so a meaningful dispatch schedule is
        // still returned. Full LMP/settlement fields are intentionally omitted.
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const int num_periods = std::max(1, j.value("num_steps", 24));

        hacdcpf::TimeSeriesData ts_data;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (g_session.ts_data.num_steps != num_periods || g_session.ts_data.profiles.empty())
            g_session.ts_data = make_default_ts_data(num_periods);
          ts_data = g_session.ts_data;
        }
        for (auto& ld : sys.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        for (auto& ren : sys.ac.renewable_gens) {
          if (ren.profile_id < 0)
            ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                              ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
        }
        for (auto& pv : sys.ac.pv_systems) if (pv.profile_id < 0) pv.profile_id = 2;

        hacdcpf::TimeSeriesPFOptions opts; opts.verbose = false;
        auto uc = hacdcpf::solve_unit_commitment(sys, ts_data, opts);

        json payload;
        payload["mode"] = "scuc";
        payload["note"] = "Market settlement (SCED/LMP) not available in simulation build; "
                          "returning SCUC unit-commitment dispatch.";
        payload["num_periods"] = num_periods;
        payload["feasible"] = uc.feasible;
        payload["total_cost"] = uc.total_cost;
        payload["solver_name"] = uc.solver_name;
        payload["gen_dispatch"] = uc.gen_dispatch;
        payload["gen_commit"] = uc.gen_commit;
        payload["ess_dispatch"] = uc.ess_dispatch;
        payload["ess_soc"] = uc.ess_soc;
        payload["renewable_dispatch"] = uc.renewable_dispatch;

        res.set_content(payload.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Carbon Flow Analysis ----
    svr.Post("/api/session/run_carbon",
             [](const httplib::Request&, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        hacdcpf::analysis::CarbonAnalysisOptions ca_opt; ca_opt.verbose = false;
        hacdcpf::analysis::CarbonAnalysisResult carbon;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.last_pf_result || !g_session.last_pf_result->converged) {
            throw std::runtime_error(
                "请先运行潮流计算，并确保潮流收敛后再进行静态碳流分析");
          }
          carbon = hacdcpf::analysis::compute_carbon_analysis(
              sys, *g_session.last_pf_result, ca_opt);
        }
        json out = carbon_analysis_to_json(sys, carbon);
        { std::lock_guard<std::mutex> lk(g_session.mu);
          out["pf_source"] = g_session.last_pf_method;
          out["requires_pf"] = true;
        }
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Dynamic Carbon Flow Analysis (TS-PF snapshots + storage carbon recursion) ----
    svr.Post("/api/session/run_dynamic_carbon",
             [materialize_loads_and_apply_binding]
             (const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::TimeSeriesData ts_data;
        hacdcpf::TimeSeriesPFResult ts_result;
        hacdcpf::HybridPowerSystem current_sys;
        bool skip_uc = true;
        bool run_opf = false;
        int n_remat = 0;
        bool ts_binding_active = false;
        std::vector<Session::ExternalGridCarbonProfile> grid_carbon_profiles;
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const bool use_last_tspf = j.value("use_last_tspf", true);
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          current_sys = *g_session.current_system;
          if (use_last_tspf) {
            if (!g_session.last_tspf_result) {
              throw std::runtime_error(
                  "请先在“时序潮流”模块运行时序潮流或时序OPF，再进行动态碳流分析");
            }
            ts_result = *g_session.last_tspf_result;
            ts_data = g_session.last_tspf_data;
            skip_uc = g_session.last_tspf_skip_uc;
            run_opf = g_session.last_tspf_run_opf;
            n_remat = -1;
            ts_binding_active = g_session.ts_binding.valid;
          }
          grid_carbon_profiles = g_session.external_grid_carbon_profiles;
        }

        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);

        if (!use_last_tspf) {
          hacdcpf::HybridPowerSystem sys_ts;
          Session::TsBindingSpec spec;
          {
            std::lock_guard<std::mutex> lk(g_session.mu);
            if (!g_session.current_system) throw std::runtime_error("No system loaded");
            const int ns = j.value("num_steps", 24);
            if (g_session.ts_data.num_steps != ns || g_session.ts_data.profiles.empty())
              g_session.ts_data = make_default_ts_data(ns);
            sys_ts = *g_session.current_system;
            ts_data = g_session.ts_data;
            spec = g_session.ts_binding;
          }
          n_remat = materialize_loads_and_apply_binding(sys_ts, spec);
          ts_binding_active = spec.valid;
          skip_uc = j.value("skip_uc", false);
          run_opf = j.value("run_opf", false);

          for (auto& ld : sys_ts.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
          for (auto& ren : sys_ts.ac.renewable_gens) {
            if (ren.profile_id < 0)
              ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                                ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
          }
          for (auto& pv : sys_ts.ac.pv_systems) if (pv.profile_id < 0) pv.profile_id = 2;
          for (auto& pv : sys_ts.dc.pv_arrays) if (pv.profile_id < 0) pv.profile_id = 2;
          for (auto& ld : sys_ts.dc.loads) if (ld.profile_id < 0) ld.profile_id = 0;

          hacdcpf::TimeSeriesPFOptions ts_opts;
          ts_opts.skip_uc = skip_uc;
          ts_opts.run_opf = run_opf;
          ts_opts.keep_system_snapshots = true;
          ts_opts.verbose = false;
          ts_result = hacdcpf::solve_time_series_pf(sys_ts, ts_data, ts_opts);
        }
        apply_current_carbon_factors_to_tspf(current_sys, ts_result);
        int applied_grid_carbon_profiles = 0;
        for (size_t t = 0; t < ts_result.pf_system_snapshots.size(); ++t) {
          auto& snapshot = ts_result.pf_system_snapshots[t];
          for (auto& eg : snapshot.ac.external_grids) {
            for (const auto& profile : grid_carbon_profiles) {
              const bool matches =
                  (profile.index == eg.index) ||
                  (profile.bus == eg.bus) ||
                  (!profile.name.empty() && profile.name == eg.name);
              if (!matches || profile.values_tco2_mwh.empty()) continue;
              const size_t pos = std::min(t, profile.values_tco2_mwh.size() - 1);
              eg.emission_factor_tco2_mwh = profile.values_tco2_mwh[pos];
              ++applied_grid_carbon_profiles;
              break;
            }
          }
        }

        hacdcpf::analysis::AnnualCarbonAnalysisOptions ca_opts;
        ca_opts.keep_hourly_bus_intensity = true;
        ca_opts.keep_hourly_load_emissions = true;
        ca_opts.keep_hourly_load_energy = true;
        auto annual = hacdcpf::analysis::compute_annual_carbon_analysis(
            ts_result, ts_data.step_duration_hr, ca_opts);

        json out = annual_carbon_to_json(annual);
        if (out.contains("step_results") && out["step_results"].is_array()) {
          for (size_t i = 0; i < out["step_results"].size(); ++i) {
            const bool has_opf =
                run_opf &&
                i < ts_result.opf_results.size();
            out["step_results"][i]["opf_converged"] =
                has_opf ? json(ts_result.opf_results[i].converged) : json(nullptr);
          }
        }
        out["num_loads_materialized"] = n_remat;
        out["ts_binding_active"] = ts_binding_active;
        out["result_source"] = use_last_tspf ? "last_tspf" : "recomputed_tspf";
        out["skip_uc"] = skip_uc;
        out["run_opf"] = run_opf;
        out["storage_dispatch_source"] =
            run_opf ? "opf" :
            ((!skip_uc && ts_result.uc_schedule.feasible) ? "uc" : "fixed_or_fallback");
        out["num_converged"] = ts_result.num_converged;
        out["num_pf_converged"] = ts_result.num_converged;
        out["num_opf_converged"] = ts_result.num_opf_converged;
        out["uc_feasible"] = ts_result.uc_schedule.feasible;
        out["uc_solver_name"] = ts_result.uc_schedule.solver_name;
        out["dynamic_storage_enabled"] =
            ts_result.pf_system_snapshots.size() == ts_result.pf_results.size();
        out["external_grid_carbon_profiles"] =
            static_cast<int>(grid_carbon_profiles.size());
        out["external_grid_carbon_profile_applications"] =
            applied_grid_carbon_profiles;

        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      } catch (...) {
        g_session.busy.store(false);
        res.status = 500;
        res.set_content(json{{"error", "Unknown internal error in dynamic carbon flow"}}.dump(), "application/json");
      }
    });

    // ---- Reactive Power Optimization (MINLP: B&B + IPM) ----
    svr.Post("/api/session/run_rpo",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const std::string obj_type = j.value("objective", std::string("voltage"));

        hacdcpf::opf::RPOOptions rpo_opt;
        if (obj_type == "loss")
          rpo_opt.objective = hacdcpf::opf::RPOObjective::MinActiveLoss;
        else if (obj_type == "combined")
          rpo_opt.objective = hacdcpf::opf::RPOObjective::Combined;
        else
          rpo_opt.objective = hacdcpf::opf::RPOObjective::MinVoltageDeviation;
        rpo_opt.vdev_weight    = j.value("vdev_weight", 1.0);
        rpo_opt.gap_tol        = j.value("mip_gap", 0.01);
        rpo_opt.time_limit_sec = j.value("time_limit_s", 120.0);

        if (j.value("relax_only", false)) {
          rpo_opt.max_nodes = 1;   // root relaxation only
        }

        auto result = hacdcpf::solve_rpo(sys, rpo_opt);

        // Build JSON response
        json out;
        out["converged"]       = result.converged;
        out["objective"]       = result.objective;
        out["objective_type"]  = obj_type;
        out["gap"]             = result.gap;
        out["nodes_explored"]  = result.nodes_explored;
        out["nlp_solves"]      = result.nlp_solves;
        out["runtime_sec"]     = result.runtime_sec;
        out["vm_before"]       = result.vm_before;
        out["vm_after"]        = result.vm_after;
        out["va_before"]       = result.va_before;
        out["va_after"]        = result.va_after;
        out["pg_before"]       = result.pg_mw_before;
        out["pg_after"]        = result.pg_mw_after;
        out["qg_before"]       = result.qg_mvar_before;
        out["qg_after"]        = result.qg_mvar_after;
        out["total_loss_before"] = result.total_loss_before_mw;
        out["total_loss_after"]  = result.total_loss_after_mw;
        out["max_vdev_before"]   = result.max_vdev_before;
        out["max_vdev_after"]    = result.max_vdev_after;
        out["status"]            = result.status;
        out["v_min"] = 0.95;
        out["v_max"] = 1.05;

        json tap_names = json::array(), tap_ratio_before = json::array(), tap_ratio_after = json::array();
        json tap_pos_before = json::array(), tap_pos_after = json::array();
        for (const auto& t : result.taps) {
          tap_names.push_back(t.name);
          tap_ratio_before.push_back(t.ratio_before);
          tap_ratio_after.push_back(t.ratio_after);
          tap_pos_before.push_back(t.tap_before);
          tap_pos_after.push_back(t.tap_after);
        }
        out["tap_names"]        = tap_names;
        out["tap_before"]       = tap_ratio_before;
        out["tap_after"]        = tap_ratio_after;
        out["tap_pos_before"]   = tap_pos_before;
        out["tap_pos_after"]    = tap_pos_after;

        json sh_names = json::array(), sh_before = json::array(), sh_after = json::array();
        json sh_mvar_before = json::array(), sh_mvar_after = json::array();
        for (const auto& s : result.shunts) {
          sh_names.push_back(s.name);
          sh_before.push_back(s.step_before);
          sh_after.push_back(s.step_after);
          sh_mvar_before.push_back(s.bs_mvar_before);
          sh_mvar_after.push_back(s.bs_mvar_after);
        }
        out["shunt_names"]       = sh_names;
        out["shunt_before"]      = sh_before;
        out["shunt_after"]       = sh_after;
        out["shunt_mvar_before"] = sh_mvar_before;
        out["shunt_mvar_after"]  = sh_mvar_after;
        out["n_taps"]   = (int)result.taps.size();
        out["n_shunts"] = (int)result.shunts.size();

        json gn = json::array();
        for (size_t i = 0; i < sys.ac.generators.size(); ++i)
          gn.push_back(sys.ac.generators[i].name.empty()
                         ? "Gen" + std::to_string(i)
                         : sys.ac.generators[i].name);
        out["gen_names"] = gn;

        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Network Reconfiguration (TS-TR MILP) ----
    svr.Post("/api/session/run_reconfig",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys_tr;
        hacdcpf::TimeSeriesData ts_data;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          const auto j2 = json::parse(req.body.empty() ? "{}" : req.body);
          const int ns = j2.value("num_steps", 4);
          if (g_session.ts_data.num_steps != ns || g_session.ts_data.profiles.empty())
            g_session.ts_data = make_default_ts_data(ns);
          sys_tr = *g_session.current_system;
          ts_data = g_session.ts_data;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        int num_steps = j.value("num_steps", 4);
        for (auto& ld : sys_tr.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        for (auto& ren : sys_tr.ac.renewable_gens) {
          if (ren.profile_id < 0)
            ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                              ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
        }

        // ========== Step 1: Baseline loss (original topology) ==========
        double base_loss_mw = 0.0;
        bool base_pf_converged = false;
        int base_pf_iterations = 0;
        bool base_is_radial = hacdcpf::analysis::is_radial(sys_tr.ac);
        bool base_is_connected = hacdcpf::analysis::is_connected(sys_tr.ac);
        int base_islands = hacdcpf::analysis::count_islands(sys_tr.ac);
        {
          hacdcpf::PowerFlowOptions pf_opt;
          pf_opt.max_iter = 200; pf_opt.tol = 1e-6;
          auto pf_base = hacdcpf::solve_power_flow(sys_tr, pf_opt);
          base_pf_converged = pf_base.converged;
          base_pf_iterations = pf_base.iterations;
          if (pf_base.converged) {
            // Sum branch losses: loss = pf_from + pt_to for each branch
            for (const auto& bf : pf_base.branch_flows) {
              double loss = bf.pf_mw + bf.pt_mw;  // sending + receiving end
              if (loss > 0.0) base_loss_mw += loss;
            }
          }
        }

        // ========== Step 2: Run topology reconfiguration (single snapshot) ==========
        // The SIM backend exposes a single-snapshot MILP reconfiguration
        // (run_topology_reconfiguration) rather than the time-series schedule
        // used in HybridACDCPowerSystemsPlanning. Run it once and present the
        // result below as a one-step schedule (num_steps == 1).
        hacdcpf::analysis::TopoReconfOptions tr_opts;
        const auto& opts = j.contains("options") ? j["options"] : j;
        tr_opts.v_min_pu  = opts.value("v_min_pu", opts.value("v_min", 0.95));
        tr_opts.v_max_pu  = opts.value("v_max_pu", opts.value("v_max", 1.05));
        tr_opts.mip_gap   = opts.value("mip_gap", 0.01);
        tr_opts.max_time_s = opts.value("max_time_s", 60);
        tr_opts.enable_pf = true;
        tr_opts.verbose   = false;
        auto recon = hacdcpf::analysis::run_topology_reconfiguration(sys_tr, tr_opts);

        // Per-branch closed flag (1 = in service) derived from the open-branch set,
        // then exposed as a single-step [branch][1] matrix for the response code below.
        std::vector<std::vector<int>> ac_branch_closed_ts(
            sys_tr.ac.branches.size(), std::vector<int>(1, 1));
        {
          std::set<int> open_ac;
          for (const auto& br : recon.open_branches)
            if (br.category == hacdcpf::graph::EdgeCategory::AC_Line) open_ac.insert(br.index);
          for (int id : recon.open_branch_ids) open_ac.insert(id);
          for (size_t b = 0; b < sys_tr.ac.branches.size(); ++b)
            ac_branch_closed_ts[b][0] = open_ac.count(sys_tr.ac.branches[b].index) ? 0 : 1;
        }
        // Single-step schedule view consumed by the response builder below.
        struct ReconSchedView {
          bool feasible;
          double total_objective;
          double total_shed_mw;
          std::string solver_name;
          std::vector<std::vector<int>>& ac_branch_closed;
          std::vector<std::vector<int>> ac_switch_on;
          std::vector<std::vector<int>> ac_switch_off;
          std::vector<std::vector<double>> storage_ac_soc_mwh;
          std::vector<std::vector<double>> ren_ac_curtail_mw;
        } sched{recon.feasible, recon.milp_objective, 0.0,
                recon.solver_backend.empty() ? recon.solver_status : recon.solver_backend,
                ac_branch_closed_ts, {}, {}, {}, {}};
        // Single-step switch counts.
        sched.ac_switch_on.assign(1, std::vector<int>(1, recon.n_switch_on));
        sched.ac_switch_off.assign(1, std::vector<int>(1, recon.n_switch_off));
        num_steps = 1;

        // ========== Step 3: Reconfigured loss + radiality check ==========
        double reconfig_loss_mw = 0.0;
        bool reconfig_pf_converged = false;
        int reconfig_pf_iterations = 0;
        double reconfig_pf_residual = 0.0;
        bool reconfig_is_radial = false;
        bool reconfig_is_connected = false;
        int reconfig_islands = 0;
        int reconfig_closed_count = 0;
        int reconfig_open_count = 0;
        if (sched.feasible) {
          // Apply final-step topology to a copy
          auto sys_reconfig = sys_tr;
          const int last_t = std::max(0, num_steps - 1);
          for (size_t b = 0; b < sys_reconfig.ac.branches.size(); ++b) {
            bool is_closed = true;
            if (b < sched.ac_branch_closed.size() && last_t < (int)sched.ac_branch_closed[b].size())
              is_closed = (sched.ac_branch_closed[b][(size_t)last_t] != 0);
            sys_reconfig.ac.branches[b].in_service = is_closed;
            if (is_closed) ++reconfig_closed_count;
            else ++reconfig_open_count;
          }
          reconfig_is_radial = hacdcpf::analysis::is_radial(sys_reconfig.ac);
          reconfig_is_connected = hacdcpf::analysis::is_connected(sys_reconfig.ac);
          reconfig_islands = hacdcpf::analysis::count_islands(sys_reconfig.ac);

          // Run power flow on reconfigured topology
          hacdcpf::PowerFlowOptions pf_opt;
          pf_opt.max_iter = 200; pf_opt.tol = 1e-6;
          auto pf_reconfig = hacdcpf::solve_power_flow(sys_reconfig, pf_opt);
          reconfig_pf_converged = pf_reconfig.converged;
          reconfig_pf_iterations = pf_reconfig.iterations;
          reconfig_pf_residual = pf_reconfig.residual;
          if (pf_reconfig.converged) {
            for (const auto& bf : pf_reconfig.branch_flows) {
              double loss = bf.pf_mw + bf.pt_mw;
              if (loss > 0.0) reconfig_loss_mw += loss;
            }
          }
        }

        // ========== Build JSON response ==========
        json out;
        out["feasible"] = sched.feasible;
        out["total_objective"] = sched.total_objective;
        out["total_shed_mw"] = sched.total_shed_mw;
        out["milp_objective"] = sched.total_objective;
        out["estimated_loss_mw"] = sched.total_objective * sys_tr.base_mva;
        out["solver_name"] = sched.solver_name;
        out["num_steps"] = num_steps;

        // Loss comparison (before vs after reconfiguration)
        out["base_loss_mw"] = base_loss_mw;
        out["base_pf_converged"] = base_pf_converged;
        out["base_pf_iterations"] = base_pf_iterations;
        out["reconfig_loss_mw"] = reconfig_loss_mw;
        out["reconfig_pf_converged"] = reconfig_pf_converged;
        out["reconfig_pf_iterations"] = reconfig_pf_iterations;
        out["reconfig_pf_residual"] = reconfig_pf_residual;
        double loss_reduction_mw = base_loss_mw - reconfig_loss_mw;
        double loss_reduction_pct = (base_loss_mw > 1e-9) ? (loss_reduction_mw / base_loss_mw * 100.0) : 0.0;
        out["loss_reduction_mw"] = loss_reduction_mw;
        out["loss_reduction_pct"] = loss_reduction_pct;

        // Topology verification
        out["base_is_radial"] = base_is_radial;
        out["base_is_connected"] = base_is_connected;
        out["base_islands"] = base_islands;
        out["reconfig_is_radial"] = reconfig_is_radial;
        out["reconfig_is_connected"] = reconfig_is_connected;
        out["reconfig_islands"] = reconfig_islands;
        out["reconfig_closed_count"] = reconfig_closed_count;
        out["reconfig_open_count"] = reconfig_open_count;
        out["num_buses"] = (int)sys_tr.ac.buses.size();

        // Verification PF
        json vf;
        vf["converged"] = reconfig_pf_converged;
        vf["iterations"] = reconfig_pf_iterations;
        vf["residual"] = reconfig_pf_residual;
        out["verification_pf"] = vf;

        json sw_on = json::array(), sw_off = json::array();
        for (int t = 0; t < num_steps; ++t) {
          int on_cnt = 0, off_cnt = 0;
          for (const auto& bv : sched.ac_switch_on) if (t < (int)bv.size()) on_cnt += bv.at((size_t)t);
          for (const auto& bv : sched.ac_switch_off) if (t < (int)bv.size()) off_cnt += bv.at((size_t)t);
          sw_on.push_back(on_cnt); sw_off.push_back(off_cnt);
        }
        out["sw_on_per_step"] = sw_on; out["sw_off_per_step"] = sw_off;
        out["ac_branch_closed"] = sched.ac_branch_closed;
        // Derive open/closed branch ID lists from last timestep for frontend display
        {
          json open_ids = json::array(), closed_ids = json::array();
          json branch_details = json::array();
          const int last_t = std::max(0, num_steps - 1);
          for (size_t b = 0; b < sys_tr.ac.branches.size(); ++b) {
            const auto& br = sys_tr.ac.branches[b];
            const int br_id = br.index;
            int closed = 0;
            if (b < sched.ac_branch_closed.size() && last_t < (int)sched.ac_branch_closed[b].size())
              closed = sched.ac_branch_closed[b][(size_t)last_t];
            if (closed) closed_ids.push_back(br_id);
            else        open_ids.push_back(br_id);
            // Detailed branch info for the table
            json bd;
            bd["id"] = br_id;
            bd["from_bus"] = br.from_bus;
            bd["to_bus"] = br.to_bus;
            bd["closed"] = closed != 0;
            bd["name"] = br.name;
            branch_details.push_back(bd);
          }
          out["open_branch_ids"] = open_ids;
          out["closed_branch_ids"] = closed_ids;
          out["branch_details"] = branch_details;
        }
        // Per-step per-branch topology for detailed display
        {
          json steps_arr = json::array();
          for (int t = 0; t < num_steps; ++t) {
            json step_open = json::array(), step_closed = json::array();
            for (size_t b = 0; b < sys_tr.ac.branches.size(); ++b) {
              const int br_id = sys_tr.ac.branches[b].index;
              int closed = 0;
              if (b < sched.ac_branch_closed.size() && t < (int)sched.ac_branch_closed[b].size())
                closed = sched.ac_branch_closed[b][(size_t)t];
              if (closed) step_closed.push_back(br_id);
              else        step_open.push_back(br_id);
            }
            steps_arr.push_back(json{{"open", step_open}, {"closed", step_closed}});
          }
          out["steps_topology"] = steps_arr;
        }
        const int n_ess = (int)sched.storage_ac_soc_mwh.size();
        out["num_branches"] = (int)sched.ac_branch_closed.size();
        out["num_ess"] = n_ess;
        json soc_j = json::array();
        for (int t = 0; t < num_steps; ++t) {
          json row = json::array();
          for (int s = 0; s < n_ess; ++s) {
            double soc = 0.0;
            const auto& sv = sched.storage_ac_soc_mwh.at((size_t)s);
            if (t < (int)sv.size()) {
              double cap = (s < (int)sys_tr.ac.storage.size())
                             ? sys_tr.ac.storage.at((size_t)s).e_rated_mwh : 1.0;
              soc = (cap > 0.0) ? sv.at((size_t)t) / cap : 0.0;
            }
            row.push_back(soc);
          }
          soc_j.push_back(row);
        }
        out["ess_soc_by_step"] = soc_j;
        json curt_j = json::array();
        for (int t = 0; t < num_steps; ++t) {
          double curt = 0.0;
          for (const auto& rv : sched.ren_ac_curtail_mw)
            if (t < (int)rv.size()) curt += rv.at((size_t)t);
          curt_j.push_back(curt);
        }
        out["curt_per_step"] = curt_j;
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      } catch (...) {
        g_session.busy.store(false);
        res.status = 500;
        res.set_content(json{{"error", "Unknown internal error in topology reconfiguration"}}.dump(), "application/json");
      }
    });

    // ---- Annual Production Simulation ----
    svr.Post("/api/session/run_annual_sim",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys_ann;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys_ann = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const double step_hr = (j.value("resolution", std::string("6h")) == "1h") ? 1.0 : 6.0;
        auto ts_data = make_annual_ts_data(step_hr);
        // Auto-assign profiles
        for (auto& ld : sys_ann.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        for (auto& ren : sys_ann.ac.renewable_gens) {
          if (ren.profile_id < 0)
            ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                              ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
        }
        hacdcpf::analysis::AnnualProductionSimOptions opts;
        opts.block_type = (j.value("block_type", std::string("monthly")) == "weekly")
            ? hacdcpf::analysis::AnnualBlockType::Weekly
            : hacdcpf::analysis::AnnualBlockType::Monthly;
        opts.ts_pf_options.run_opf = j.value("run_opf", true);
        opts.ts_pf_options.verbose = false;
        opts.skip_replay = j.value("skip_replay", false);
        opts.enforce_cyclic_soc = j.value("cyclic_soc", true);
        opts.pf_snapshot_interval = j.value("snapshot_interval", 24);
        opts.verbose = false;
        auto result = hacdcpf::analysis::solve_annual_production_simulation(sys_ann, ts_data, opts);
        json out;
        out["feasible"] = result.feasible;
        out["num_steps"] = result.num_steps;
        out["step_duration_hr"] = result.step_duration_hr;
        out["total_cost"] = result.total_cost;
        out["total_gen_mwh"] = result.total_gen_mwh;
        out["total_load_mwh"] = result.total_load_mwh;
        out["total_renewable_mwh"] = result.total_renewable_mwh;
        out["total_curtailment_mwh"] = result.total_curtailment_mwh;
        out["total_ens_mwh"] = result.total_ens_mwh;
        out["total_loss_mwh"] = result.total_loss_mwh;
        out["num_pf_converged"] = result.num_pf_converged;
        out["num_opf_converged"] = result.num_opf_converged;
        // Per-step timeline (subsample for large data)
        const int max_timeline = 1000;
        const int stride = std::max(1, (int)result.step_results.size() / max_timeline);
        json tl_gen = json::array(), tl_load = json::array(), tl_ren = json::array(),
             tl_curt = json::array(), tl_ess = json::array(), tl_loss = json::array(),
             tl_hours = json::array();
        for (size_t i = 0; i < result.step_results.size(); i += static_cast<size_t>(stride)) {
          const auto& s = result.step_results[i];
          tl_hours.push_back(static_cast<double>(i) * result.step_duration_hr);
          tl_gen.push_back(s.total_gen_mw);
          tl_load.push_back(s.total_load_mw);
          tl_ren.push_back(s.total_renewable_mw);
          tl_curt.push_back(s.total_curtailment_mw);
          tl_ess.push_back(s.total_ess_mw);
          tl_loss.push_back(s.total_loss_mw);
        }
        out["timeline_hours"] = tl_hours;
        out["timeline_gen"] = tl_gen;
        out["timeline_load"] = tl_load;
        out["timeline_ren"] = tl_ren;
        out["timeline_curt"] = tl_curt;
        out["timeline_ess"] = tl_ess;
        out["timeline_loss"] = tl_loss;
        // Monthly summaries
        json ms = json::array();
        for (const auto& m : result.monthly_summaries) {
          ms.push_back({{"block_id", m.block_id}, {"total_gen_mwh", m.total_gen_mwh},
                        {"total_load_mwh", m.total_load_mwh},
                        {"total_renewable_mwh", m.total_renewable_mwh},
                        {"total_curtailment_mwh", m.total_curtailment_mwh},
                        {"total_loss_mwh", m.total_loss_mwh},
                        {"total_ens_mwh", m.total_ens_mwh},
                        {"total_cost", m.total_cost},
                        {"num_pf_converged", m.num_pf_converged},
                        {"num_steps", m.num_steps}});
        }
        out["monthly_summaries"] = ms;
        // Generator stats
        json gs = json::array();
        for (const auto& g : result.gen_stats) {
          gs.push_back({{"name", g.name}, {"total_energy_mwh", g.total_energy_mwh},
                        {"capacity_factor", g.capacity_factor},
                        {"total_startups", g.total_startups},
                        {"total_hours_online", g.total_hours_online}});
        }
        out["gen_stats"] = gs;
        // Storage stats
        json ss = json::array();
        for (const auto& s : result.storage_stats) {
          ss.push_back({{"name", s.name}, {"total_charge_mwh", s.total_charge_mwh},
                        {"total_discharge_mwh", s.total_discharge_mwh},
                        {"cycles", s.cycles}});
        }
        out["storage_stats"] = ss;
        // Renewable stats
        json rs = json::array();
        for (const auto& r : result.renewable_stats) {
          rs.push_back({{"name", r.name}, {"total_energy_mwh", r.total_energy_mwh},
                        {"total_curtailed_mwh", r.total_curtailed_mwh},
                        {"capacity_factor", r.capacity_factor},
                        {"curtailment_rate", r.curtailment_rate}});
        }
        out["renewable_stats"] = rs;
        // PF voltage snapshots (subsampled vm for heatmap)
        json snap_hours = json::array(), snap_vm = json::array();
        for (const auto& snap : result.pf_snapshots) {
          if (!snap.converged || snap.vm.empty()) continue;
          snap_hours.push_back(static_cast<double>(snap.global_step) * result.step_duration_hr);
          snap_vm.push_back(snap.vm);
        }
        out["snapshot_hours"] = snap_hours;
        out["snapshot_vm"] = snap_vm;
        // Geo data for animation map
        json geo_buses = json::array();
        for (const auto& bus : sys_ann.ac.buses) {
          geo_buses.push_back({{"id", bus.index}, {"name", bus.name}, {"type", "ac"},
                               {"lat", bus.latitude}, {"lon", bus.longitude}});
        }
        for (const auto& bus : sys_ann.dc.buses) {
          geo_buses.push_back({{"id", bus.index}, {"name", bus.name}, {"type", "dc"},
                               {"lat", bus.latitude}, {"lon", bus.longitude}});
        }
        out["geo_buses"] = geo_buses;
        json geo_ac_branches = json::array();
        for (const auto& br : sys_ann.ac.branches) {
          auto it_from = std::find_if(sys_ann.ac.buses.begin(), sys_ann.ac.buses.end(),
                                      [&](const auto& b){ return b.index == br.from_bus; });
          auto it_to = std::find_if(sys_ann.ac.buses.begin(), sys_ann.ac.buses.end(),
                                    [&](const auto& b){ return b.index == br.to_bus; });
          double from_lat = 0, from_lon = 0, to_lat = 0, to_lon = 0;
          if (it_from != sys_ann.ac.buses.end()) { from_lat = it_from->latitude; from_lon = it_from->longitude; }
          if (it_to != sys_ann.ac.buses.end()) { to_lat = it_to->latitude; to_lon = it_to->longitude; }
          geo_ac_branches.push_back({{"from_bus", br.from_bus}, {"to_bus", br.to_bus},
                                     {"from_lat", from_lat}, {"from_lon", from_lon},
                                     {"to_lat", to_lat}, {"to_lon", to_lon}});
        }
        out["geo_ac_branches"] = geo_ac_branches;
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Lifecycle Simulation ----
    svr.Post("/api/session/run_lifecycle_sim",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys_lc;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys_lc = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const double step_hr = (j.value("resolution", std::string("6h")) == "1h") ? 1.0 : 6.0;
        auto ts_data = make_annual_ts_data(step_hr);
        // Auto-assign profiles
        for (auto& ld : sys_lc.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        for (auto& ren : sys_lc.ac.renewable_gens) {
          if (ren.profile_id < 0)
            ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                              ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
        }

        hacdcpf::analysis::LifecycleSimOptions opts;
        opts.num_years = j.value("num_years", 20);
        opts.discount_rate = j.value("discount_rate", 0.05);
        opts.load_growth_rate = j.value("load_growth_rate", 0.02);
        opts.pv_annual_derating = j.value("pv_annual_derating", 0.005);
        opts.calendar_degradation_per_year = j.value("calendar_degradation", 0.02);
        opts.step_duration_hr = step_hr;
        opts.verbose = false;

        // Apply capacity scaling
        hacdcpf::analysis::CapacityScaling cap_scaling;
        cap_scaling.pv_scale = j.value("pv_scale", 1.0);
        cap_scaling.wind_scale = j.value("wind_scale", 1.0);
        cap_scaling.bess_power_scale = j.value("bess_power_scale", 1.0);
        cap_scaling.bess_energy_scale = j.value("bess_energy_scale", 1.0);
        cap_scaling.diesel_scale = j.value("diesel_scale", 1.0);
        hacdcpf::analysis::apply_capacity_scaling(sys_lc, cap_scaling);

        auto result = hacdcpf::analysis::run_lifecycle_simulation(sys_lc, ts_data, opts);

        json out;
        out["feasible"] = result.feasible;
        out["num_years"] = result.num_years;
        out["npv_total_cost"] = result.npv_total_cost;
        out["total_carbon_tco2"] = result.total_carbon_tco2;
        out["total_replacement_cost"] = result.total_replacement_cost;
        out["total_replacements"] = result.total_replacements;

        // Per-year data
        json years_arr = json::array();
        for (const auto& yr : result.year_results) {
          json yj;
          yj["year"] = yr.year;
          yj["load_growth_factor"] = yr.load_growth_factor;
          yj["annual_cost"] = yr.annual_cost;
          yj["annual_gen_mwh"] = yr.annual_gen_mwh;
          yj["annual_load_mwh"] = yr.annual_load_mwh;
          yj["annual_renewable_mwh"] = yr.annual_renewable_mwh;
          yj["annual_curtailment_mwh"] = yr.annual_curtailment_mwh;
          yj["annual_ens_mwh"] = yr.annual_ens_mwh;
          yj["annual_carbon_tco2"] = yr.annual_carbon_tco2;
          yj["avg_carbon_intensity"] = yr.avg_carbon_intensity;
          yj["feasible"] = yr.feasible;

          // Bounds
          json bj;
          bj["total_bound_tco2"] = yr.bounds.total_bound_tco2;
          bj["dispatch_bound_tco2"] = yr.bounds.dispatch.bound_tco2;
          bj["sampling_bound_tco2"] = yr.bounds.sampling.bound_tco2;
          bj["storage_carbon_bound_tco2"] = yr.bounds.storage_carbon.bound_tco2;
          yj["bounds"] = bj;

          // Cross-validation
          json cvj;
          cvj["dense_carbon_tco2"] = yr.cross_validation.dense_carbon_tco2;
          cvj["sampled_carbon_tco2"] = yr.cross_validation.sampled_carbon_tco2;
          cvj["sampling_gap_tco2"] = yr.cross_validation.sampling_gap_tco2;
          cvj["sampling_gap_pct"] = yr.cross_validation.sampling_gap_pct;
          cvj["total_bound_pct"] = yr.cross_validation.total_bound_pct;
          cvj["dispatch_bound_pct"] = yr.cross_validation.dispatch_bound_pct;
          cvj["sampling_bound_pct"] = yr.cross_validation.sampling_bound_pct;
          cvj["storage_bound_pct"] = yr.cross_validation.storage_bound_pct;
          yj["cross_validation"] = cvj;

          // Storage states
          json ss_arr = json::array();
          for (const auto& ss : yr.storage_states) {
            ss_arr.push_back({
              {"storage_index", ss.storage_index},
              {"name", ss.name},
              {"soh", ss.soh},
              {"soh_cycle", ss.soh_cycle},
              {"soh_calendar", ss.soh_calendar},
              {"effective_capacity_mwh", ss.effective_capacity_mwh},
              {"cycles_this_year", ss.cycles_this_year},
              {"cumulative_cycles", ss.cumulative_cycles},
              {"replaced", ss.replaced},
            });
          }
          yj["storage_states"] = ss_arr;

          // Renewable states
          json rs_arr = json::array();
          for (const auto& rs : yr.renewable_states) {
            rs_arr.push_back({
              {"ren_index", rs.ren_index},
              {"name", rs.name},
              {"derated_capacity_mw", rs.derated_capacity_mw},
              {"original_capacity_mw", rs.original_capacity_mw},
              {"derating_factor", rs.derating_factor},
            });
          }
          yj["renewable_states"] = rs_arr;

          years_arr.push_back(yj);
        }
        out["years"] = years_arr;

        // Replacement events
        json repl_arr = json::array();
        for (const auto& re : result.all_replacements) {
          repl_arr.push_back({
            {"year", re.year},
            {"storage_index", re.storage_index},
            {"storage_name", re.storage_name},
            {"old_soh", re.old_soh},
            {"replacement_cost_usd", re.replacement_cost_usd},
          });
        }
        out["replacements"] = repl_arr;

        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Lifecycle Capacity Comparison ----
    svr.Post("/api/session/run_lifecycle_compare",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys_base;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys_base = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        const double step_hr = (j.value("resolution", std::string("6h")) == "1h") ? 1.0 : 6.0;
        auto ts_data = make_annual_ts_data(step_hr);
        // Auto-assign profiles
        for (auto& ld : sys_base.ac.loads) if (ld.profile_id < 0) ld.profile_id = 0;
        for (auto& ren : sys_base.ac.renewable_gens) {
          if (ren.profile_id < 0)
            ren.profile_id = (ren.type == hacdcpf::RenewableType::SolarPV ||
                              ren.type == hacdcpf::RenewableType::SolarCSP) ? 2 : 1;
        }

        hacdcpf::analysis::LifecycleSimOptions opts;
        opts.num_years = j.value("num_years", 20);
        opts.discount_rate = j.value("discount_rate", 0.05);
        opts.load_growth_rate = j.value("load_growth_rate", 0.02);
        opts.pv_annual_derating = j.value("pv_annual_derating", 0.005);
        opts.calendar_degradation_per_year = j.value("calendar_degradation", 0.02);
        opts.step_duration_hr = step_hr;
        opts.verbose = false;

        // Base capacity scaling
        hacdcpf::analysis::CapacityScaling base_scaling;
        base_scaling.pv_scale = j.value("pv_scale", 1.0);
        base_scaling.wind_scale = j.value("wind_scale", 1.0);
        base_scaling.bess_power_scale = j.value("bess_power_scale", 1.0);
        base_scaling.bess_energy_scale = j.value("bess_energy_scale", 1.0);
        base_scaling.diesel_scale = j.value("diesel_scale", 1.0);

        // Sweep configuration
        const std::string sweep_param = j.value("sweep_param", std::string("bess_energy"));
        const double sweep_min = j.value("sweep_min", 0.5);
        const double sweep_max = j.value("sweep_max", 3.0);
        const int sweep_steps = std::min(j.value("sweep_steps", 6), 20);

        std::vector<double> scale_values;
        for (int i = 0; i < sweep_steps; ++i) {
          double sv = (sweep_steps <= 1) ? sweep_min :
              sweep_min + (sweep_max - sweep_min) * i / (sweep_steps - 1);
          scale_values.push_back(sv);
        }

        auto cmp = hacdcpf::analysis::run_lifecycle_comparison(
            sys_base, ts_data, opts, sweep_param, scale_values, base_scaling);

        json out;
        out["sweep_parameter"] = cmp.sweep_parameter;
        out["num_scenarios"] = cmp.num_scenarios;
        json scens_arr = json::array();
        for (const auto& sc : cmp.scenarios) {
          json sj;
          sj["label"] = sc.label;
          sj["npv_total_cost"] = sc.npv_total_cost;
          sj["total_carbon_tco2"] = sc.total_carbon_tco2;
          sj["total_replacement_cost"] = sc.total_replacement_cost;
          sj["total_replacements"] = sc.total_replacements;
          sj["feasible"] = sc.feasible;
          sj["yearly_carbon"] = sc.yearly_carbon;
          sj["yearly_cost"] = sc.yearly_cost;
          sj["total_pv_mw"] = sc.total_pv_mw;
          sj["total_wind_mw"] = sc.total_wind_mw;
          sj["total_bess_mw"] = sc.total_bess_mw;
          sj["total_bess_mwh"] = sc.total_bess_mwh;
          sj["total_diesel_mw"] = sc.total_diesel_mw;
          scens_arr.push_back(sj);
        }
        out["scenarios"] = scens_arr;

        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Reliability Assessment (Non-Sequential Monte Carlo) ----
    svr.Post("/api/session/run_reliability_nsq",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        
        // Apply IEEE-24 reliability data if requested
        if (j.value("apply_ieee24_data", true)) {
          hacdcpf::analysis::apply_ieee24_reliability_data(sys);
        }
        
        hacdcpf::analysis::ReliabilityOptions opts;
        opts.max_iterations = j.value("max_iterations", 5000);
        opts.cov_threshold = j.value("cov_threshold", 0.05);
        opts.load_scale_factor = j.value("load_scale_factor", 1.0);
        opts.seed = j.value("seed", 0);
        opts.verbose = j.value("verbose", false);
        opts.compute_tail_risk = j.value("compute_tail_risk", false);
        opts.var_confidence = j.value("var_confidence", 0.95);
        opts.opf_options.verbose = false;
        
        // Progress callback with cancellation support
        opts.progress_callback = [](int iter, double eens, double cov) {
          return !g_session.cancel.load();
        };
        
        auto result = hacdcpf::analysis::run_nonsequential_mc(sys, opts);
        
        json out;
        out["converged"] = result.converged;
        out["iterations_used"] = result.iterations_used;
        out["final_cov"] = result.final_cov;
        out["eens_mwh_yr"] = result.eens_mwh_yr;
        out["edns_mw"] = result.edns_mw;
        out["lole_hr_yr"] = result.lole_hr_yr;
        out["plc"] = result.plc;
        out["nodal_eens_mwh_yr"] = result.nodal_eens_mwh_yr;
        out["eens_history"] = result.eens_history;
        out["cov_history"] = result.cov_history;
        
        // Tail risk metrics
        if (opts.compute_tail_risk) {
          out["tail_risk"]["eens_var"] = result.tail_risk.eens_var;
          out["tail_risk"]["eens_cvar"] = result.tail_risk.eens_cvar;
          out["tail_risk"]["lole_var"] = result.tail_risk.lole_var;
          out["tail_risk"]["lole_cvar"] = result.tail_risk.lole_cvar;
        }
        
        // Critical components
        json crit_arr = json::array();
        for (const auto& ci : result.critical_components) {
          crit_arr.push_back({
            {"index", ci.index},
            {"is_generator", ci.is_generator},
            {"importance", ci.importance}
          });
        }
        out["critical_components"] = crit_arr;
        
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Reliability Assessment (Sequential Monte Carlo) ----
    svr.Post("/api/session/run_reliability_seq",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        
        // Apply IEEE-24 reliability data if requested
        if (j.value("apply_ieee24_data", true)) {
          hacdcpf::analysis::apply_ieee24_reliability_data(sys);
        }
        
        hacdcpf::analysis::ReliabilityOptions opts;
        opts.max_iterations = j.value("max_years", 500);
        opts.cov_threshold = j.value("cov_threshold", 0.05);
        opts.load_scale_factor = j.value("load_scale_factor", 1.0);
        opts.hours_per_year = j.value("hours_per_year", 8736);
        opts.seed = j.value("seed", 0);
        opts.verbose = j.value("verbose", false);
        opts.compute_tail_risk = j.value("compute_tail_risk", false);
        opts.var_confidence = j.value("var_confidence", 0.95);
        opts.opf_options.verbose = false;
        
        // Progress callback with cancellation support
        opts.progress_callback = [](int year, double eens, double cov) {
          return !g_session.cancel.load();
        };
        
        // Build load profile
        auto load_profile = hacdcpf::analysis::build_ieee_rts24_load_profile(opts.hours_per_year);
        
        auto result = hacdcpf::analysis::run_sequential_mc(sys, load_profile, opts);
        
        json out;
        out["converged"] = result.converged;
        out["iterations_used"] = result.iterations_used;
        out["final_cov"] = result.final_cov;
        out["eens_mwh_yr"] = result.eens_mwh_yr;
        out["edns_mw"] = result.edns_mw;
        out["lole_hr_yr"] = result.lole_hr_yr;
        out["lolf_occ_yr"] = result.lolf_occ_yr;
        out["plc"] = result.plc;
        out["nodal_eens_mwh_yr"] = result.nodal_eens_mwh_yr;
        out["eens_history"] = result.eens_history;
        out["cov_history"] = result.cov_history;
        out["annual_eens"] = result.annual_eens;
        out["annual_lole"] = result.annual_lole;
        out["annual_lolf"] = result.annual_lolf;
        
        // Tail risk metrics
        if (opts.compute_tail_risk) {
          out["tail_risk"]["eens_var"] = result.tail_risk.eens_var;
          out["tail_risk"]["eens_cvar"] = result.tail_risk.eens_cvar;
          out["tail_risk"]["lole_var"] = result.tail_risk.lole_var;
          out["tail_risk"]["lole_cvar"] = result.tail_risk.lole_cvar;
        }
        
        // Critical components
        json crit_arr = json::array();
        for (const auto& ci : result.critical_components) {
          crit_arr.push_back({
            {"index", ci.index},
            {"is_generator", ci.is_generator},
            {"importance", ci.importance}
          });
        }
        out["critical_components"] = crit_arr;
        
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Reliability Assessment (FMEA - Distribution System) ----
    svr.Post("/api/session/run_reliability_fmea",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        
        // Apply comprehensive reliability data if requested
        if (j.value("apply_comprehensive_data", true)) {
          hacdcpf::analysis::apply_comprehensive_reliability_data(sys);
        } else if (j.value("apply_ieee24_data", false)) {
          hacdcpf::analysis::apply_ieee24_reliability_data(sys);
        }
        
        hacdcpf::analysis::FMEAOptions fmea_opts;
        fmea_opts.load_scale_factor = j.value("load_scale_factor", 1.0);
        fmea_opts.verbose = j.value("verbose", false);
        
        auto result = hacdcpf::analysis::run_distribution_fmea(sys, fmea_opts);
        
        json out;
        out["eens_mwh_yr"] = result.eens_mwh_yr;
        out["edns_mw"] = result.edns_mw;
        out["lole_hr_yr"] = result.lole_hr_yr;
        out["lolf_occ_yr"] = result.lolf_occ_yr;
        out["saifi"] = result.distribution_idx.saifi;
        out["saidi"] = result.distribution_idx.saidi;
        out["caidi"] = result.distribution_idx.caidi;
        out["asai"] = result.distribution_idx.asai;
        out["n_contingencies"] = result.contingencies.size();
        
        // Count contingencies with loss
        int n_with_loss = 0;
        for (const auto& c : result.contingencies) {
          if (c.shed_rep_mw > 0.01) n_with_loss++;
        }
        out["n_with_loss"] = n_with_loss;
        
        // Contingencies sorted by EENS contribution (descending)
        auto sorted = result.contingencies;
        std::sort(sorted.begin(), sorted.end(),
          [](const auto& a, const auto& b) { return a.eens_contribution > b.eens_contribution; });
        
        json cont_arr = json::array();
        for (const auto& c : sorted) {
          const auto meta = describe_fmea_component(sys, c.component_type, c.component_index);
          cont_arr.push_back({
            {"component_name", meta.display_name},
            {"component_type", c.component_type},
            {"display_name", meta.display_name},
            {"display_type", meta.display_type},
            {"component_index", c.component_index},
            {"canvas_type", meta.canvas_type},
            {"canvas_index", meta.canvas_index},
            {"primary_bus", meta.primary_bus},
            {"secondary_bus", meta.secondary_bus},
            {"mappable", meta.mappable},
            {"shed_mw", c.shed_rep_mw},
            {"eens_contribution", c.eens_contribution},
            {"failure_rate", c.failure_rate},
            {"repair_time", c.tau_rep_hr}
          });
        }
        out["contingencies"] = cont_arr;
        
        // Aggregate EENS by component type
        json eens_by_type;
        json eens_by_display_type;
        for (const auto& c : result.contingencies) {
          std::string tp = c.component_type;
          if (eens_by_type.contains(tp)) {
            eens_by_type[tp] = eens_by_type[tp].get<double>() + c.eens_contribution;
          } else {
            eens_by_type[tp] = c.eens_contribution;
          }
          const std::string display_tp = fmea_type_label(tp);
          if (eens_by_display_type.contains(display_tp)) {
            eens_by_display_type[display_tp] =
                eens_by_display_type[display_tp].get<double>() + c.eens_contribution;
          } else {
            eens_by_display_type[display_tp] = c.eens_contribution;
          }
        }
        out["eens_by_type"] = eens_by_type;
        out["eens_by_display_type"] = eens_by_display_type;
        
        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Distribution Resilience Assessment (MESS) ----
    svr.Post("/api/session/generate_scenarios",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        auto options = hacdcpf::analysis::scenario_generation_options_from_json(j);

        options.regular.num_steps = 8760;
        options.reliability.num_steps = 1;
        options.resilience.num_steps = 48;
        options.regular.candidate_count = std::clamp(options.regular.candidate_count, 1, 500);
        options.regular.cluster_count = std::clamp(options.regular.cluster_count, 1, options.regular.candidate_count);
        options.reliability.candidates_per_contingency = std::clamp(options.reliability.candidates_per_contingency, 1, 200);
        options.reliability.cluster_count_per_contingency = std::clamp(options.reliability.cluster_count_per_contingency, 1, options.reliability.candidates_per_contingency);
        options.resilience.candidates_per_intensity = std::clamp(options.resilience.candidates_per_intensity, 1, 200);
        options.resilience.default_cluster_count = std::clamp(options.resilience.default_cluster_count, 1, options.resilience.candidates_per_intensity);
        for (auto& [category, count] : options.resilience.cluster_count_by_intensity) {
          count = std::clamp(count, 1, options.resilience.candidates_per_intensity);
        }

        const auto result = hacdcpf::analysis::generate_scenarios(sys, options);
        res.set_content(hacdcpf::analysis::scenario_generation_result_to_json(result).dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Typhoon scenario preview: generate faults, let frontend fill manual fields. ----
    svr.Post("/api/session/generate_typhoon_faults",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        hacdcpf::analysis::TyphoonScenarioOptions typhoon_opts;
        typhoon_opts.horizon_hours = j.value("horizon_hours", 48);
        typhoon_opts.time_step_hr = j.value("time_step_hr", 1.0);
        typhoon_opts.seed = j.value("seed", fresh_typhoon_seed());
        typhoon_opts.stochastic = j.value("stochastic", true);
        typhoon_opts.month = j.value("month", 8);
        typhoon_opts.use_month_defaults = true;
        typhoon_opts.use_sst_resource = true;
        typhoon_opts.sst_resource_path = j.value("sst_resource_path", typhoon_opts.sst_resource_path);
        typhoon_opts.target_segment_length_km = j.value("target_segment_length_km", typhoon_opts.target_segment_length_km);
        typhoon_opts.min_fault_probability = j.value("min_fault_probability", typhoon_opts.min_fault_probability);
        typhoon_opts.staged_post_disaster_repair = j.value("staged_post_disaster_repair", typhoon_opts.staged_post_disaster_repair);
        typhoon_opts.post_disaster_repair_delay_hr = j.value("post_disaster_repair_delay_hr", typhoon_opts.post_disaster_repair_delay_hr);
        typhoon_opts.repair_crew_time_per_branch_hr = j.value("repair_crew_time_per_branch_hr", typhoon_opts.repair_crew_time_per_branch_hr);
        typhoon_opts.apply_pv_wind_derating = j.value("apply_pv_wind_derating", typhoon_opts.apply_pv_wind_derating);
        json catalog_metadata = json::object();
        configure_typhoon_catalog_sample(j, typhoon_opts, &catalog_metadata);
        const auto generated = hacdcpf::analysis::generate_typhoon_fault_sequence(sys, typhoon_opts);

        json out;
        out.update(catalog_metadata);
        out["scenario_source"] = "typhoon";
        append_typhoon_result_metadata(out, generated);
        out["status"] = generated.status;
        out["month"] = typhoon_opts.month;
        out["seed"] = generated.seed;
        out["stochastic"] = typhoon_opts.stochastic;
        out["monthly_sst_c"] = generated.monthly_sst_c;
        out["sst_resource_path"] = generated.sst_resource_path;
        out["used_fallback_coordinates"] = generated.used_fallback_coordinates;
        out["used_synthetic_segments"] = generated.used_synthetic_segments;
        out["segment_count"] = generated.generated_segments.size();
        out["fault_count"] = generated.faults.size();
        json locations = json::array();
        json typed_locations = json::array();
        json starts = json::array();
        json repairs = json::array();
        json manual = json::array();
        for (const auto& f : generated.faults) {
          if (f.branch_kind == hacdcpf::analysis::ResilienceBranchKind::AC) {
            locations.push_back(f.branch_index);
          }
          typed_locations.push_back(json{{"branch_type", hacdcpf::analysis::to_string(f.branch_kind)},
                                         {"branch_index", f.branch_index}});
          starts.push_back(f.outage_start_hr);
          repairs.push_back(f.repair_duration_hr);
          manual.push_back(json{{"branch_type", hacdcpf::analysis::to_string(f.branch_kind)},
                                {"branch_id", f.branch_index},
                                {"start_hr", f.outage_start_hr},
                                {"repair_hr", f.repair_duration_hr},
                                {"label", f.name}});
        }
        out["fault_locations"] = locations;
        out["fault_locations_typed"] = typed_locations;
        out["fault_start_hours"] = starts;
        out["repair_durations"] = repairs;
        out["manual_faults"] = manual;
        json track = json::array();
        for (const auto& p : generated.track) {
          track.push_back(json{{"hour", p.hour}, {"lat", p.latitude}, {"lon", p.longitude},
                               {"delta_p_hpa", p.delta_p_hpa}, {"rmw_km", p.rmw_km},
                               {"vmax_ms", p.vmax_ms}, {"sst_c", p.sst_c}});
        }
        out["track"] = track;
        json peaks = json::array();
        for (const auto& risk : generated.branch_risks) {
          peaks.push_back(json{{"branch_type", hacdcpf::analysis::to_string(risk.branch_kind)},
                               {"branch_index", risk.branch_index},
                               {"peak_wind_ms", risk.peak_wind_ms},
                               {"peak_rain_mm_hr", risk.peak_rain_mm_hr},
                               {"peak_failure_probability", risk.peak_failure_probability}});
        }
        out["branch_peak_wind"] = peaks;
        res.set_content(out.dump(), "application/json");
      } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Distribution Resilience Assessment (MESS) ----
    svr.Post("/api/session/run_distribution_resilience",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        if (g_session.busy.exchange(true)) {
          res.status = 409;
          res.set_content(json{{"error","Another analysis is already running"}}.dump(), "application/json");
          return;
        }
        g_session.cancel.store(false);
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);

        if (j.value("apply_demo_data", true)) {
          hacdcpf::analysis::apply_distribution_resilience_demo_data(sys);
        }

        hacdcpf::analysis::DistributionResilienceOptions opts;
        opts.horizon_hours = j.value("horizon_hours", 8);
        opts.time_step_hr = j.value("time_step_hr", 1.0);
        opts.load_scale_factor = j.value("load_scale_factor", 1.0);
        opts.allow_reconfiguration = j.value("allow_reconfiguration", false);
        opts.allow_mess_dispatch = j.value("allow_mess_dispatch", true);
        opts.mess_travel_speed_kmph = j.value("mess_travel_speed_kmph", 40.0);
        opts.default_fault_count = j.value("default_fault_count", 1);
        opts.default_repair_time_hr = j.value("repair_time_hr", 6.0);
        opts.auto_fault_stagger_hr = j.value("auto_fault_stagger_hr", 0.0);
        opts.auto_fault_start_hr = j.value("auto_fault_start_hr", 0.0);
        opts.run_power_flow = j.value("run_power_flow", false);
        auto parse_profile = [&](const char* key) {
          std::vector<double> profile;
          if (!j.contains(key) || !j[key].is_array()) return profile;
          for (const auto& v : j[key]) {
            if (!v.is_number()) continue;
            const double x = v.get<double>();
            if (std::isfinite(x)) profile.push_back(std::max(0.0, x));
          }
          return profile;
        };
        if (auto profile = parse_profile("load_profile"); !profile.empty()) {
          opts.load_profile = std::move(profile);
        }
        if (auto profile = parse_profile("renewable_profile"); !profile.empty()) {
          opts.renewable_profile = std::move(profile);
        }
        if (auto profile = parse_profile("pv_profile"); !profile.empty()) {
          opts.pv_profile = std::move(profile);
        }
        if (auto profile = parse_profile("wind_profile"); !profile.empty()) {
          opts.wind_profile = std::move(profile);
        }
        std::map<int, std::vector<double>> scenario_profiles_by_id;
        if (j.contains("scenario_profiles") && j["scenario_profiles"].is_array()) {
          for (const auto& p : j["scenario_profiles"]) {
            if (!p.is_object() || !p.contains("values") || !p["values"].is_array()) continue;
            const int id = p.value("id", -1);
            if (id < 0) continue;
            std::vector<double> values;
            for (const auto& v : p["values"]) {
              if (!v.is_number()) continue;
              const double x = v.get<double>();
              if (std::isfinite(x)) values.push_back(std::max(0.0, x));
            }
            if (!values.empty()) scenario_profiles_by_id[id] = std::move(values);
          }
        }
        if (j.contains("load_profile_map") && j["load_profile_map"].is_array()) {
          for (const auto& row : j["load_profile_map"]) {
            if (!row.is_object()) continue;
            const int profile_id = row.value("profile_id", -1);
            const auto profile_it = scenario_profiles_by_id.find(profile_id);
            if (profile_it == scenario_profiles_by_id.end()) continue;
            const auto& values = profile_it->second;
            std::string kind = row.value("kind", std::string{"AC_LOAD"});
            std::transform(kind.begin(), kind.end(), kind.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            const int position = row.value("load_position", row.value("position", -1));
            const int load_index = row.value("load_index", -1);
            const int bus = row.value("bus", -1);
            if (kind == "DC_LOAD") {
              if (position >= 0) opts.dc_load_profiles_by_position[position] = values;
              if (load_index >= 0) opts.dc_load_profiles_by_index[load_index] = values;
              if (bus >= 0) opts.dc_load_profiles_by_bus[bus] = values;
            } else if (kind == "AC_BUS") {
              if (bus >= 0) opts.ac_bus_load_profiles_by_bus[bus] = values;
            } else if (kind == "DC_BUS") {
              if (bus >= 0) opts.dc_bus_load_profiles_by_bus[bus] = values;
            } else {
              if (position >= 0) opts.ac_load_profiles_by_position[position] = values;
              if (load_index >= 0) opts.ac_load_profiles_by_index[load_index] = values;
              if (bus >= 0) opts.ac_load_profiles_by_bus[bus] = values;
            }
          }
        }

        const bool consider_switches = j.value("consider_switches", true);
        opts.enable_disaster_stages = j.value("enable_disaster_stages", true);
        opts.use_ra_style_stage_milp = j.value("use_ra_style_stage_milp", true);
        opts.post_fault_reconfig_window_hr = j.value("post_fault_reconfig_window_hr", opts.post_fault_reconfig_window_hr);
        opts.disaster_post_fault_reconfig_window_hr = j.value(
            "disaster_post_fault_reconfig_window_hr", opts.post_fault_reconfig_window_hr);
        opts.use_switch_based_fault_isolation = j.value("use_switch_based_fault_isolation", true);
        opts.allow_stage1_open_switches = j.value("allow_stage1_open_switches", true);
        opts.allow_stage2_close_ties = j.value("allow_stage2_close_ties", true);
        opts.require_switch_for_nonfault_branch_operation =
            j.value("require_switch_for_nonfault_branch_operation", consider_switches);
        opts.allow_branch_operation_without_switch =
            j.value("allow_branch_operation_without_switch", !consider_switches);
        opts.use_remote_switch_only = j.value("use_remote_switch_only", false);
        opts.model = hacdcpf::analysis::DistributionResilienceModel::RAStyleStageMILP;
        opts.enable_disaster_stages = true;
        opts.use_ra_style_stage_milp = true;
        opts.use_switch_based_fault_isolation = true;
        opts.allow_stage1_open_switches = true;
        opts.allow_stage2_close_ties = true;
        opts.require_switch_for_nonfault_branch_operation = true;
        opts.allow_branch_operation_without_switch = false;
        (void)consider_switches;

        auto value_int_any = [](const json& obj, std::initializer_list<const char*> keys, int dflt = -1) {
          for (const auto* key : keys) if (obj.contains(key) && !obj[key].is_null()) return obj[key].get<int>();
          return dflt;
        };
        auto value_double_any = [](const json& obj, std::initializer_list<const char*> keys, double dflt = 0.0) {
          for (const auto* key : keys) if (obj.contains(key) && !obj[key].is_null()) return obj[key].get<double>();
          return dflt;
        };
        auto value_string_any = [](const json& obj, std::initializer_list<const char*> keys, std::string dflt = {}) {
          for (const auto* key : keys) if (obj.contains(key) && obj[key].is_string()) return obj[key].get<std::string>();
          return dflt;
        };
        auto add_fault = [&](hacdcpf::analysis::ResilienceBranchKind kind, int bid,
                             double start_hr, double repair_hr, std::string label) {
          if (bid < 0) return;
          hacdcpf::analysis::DistributionResilienceFault f;
          f.branch_kind = kind;
          f.branch_index = bid;
          f.ac_branch_index = kind == hacdcpf::analysis::ResilienceBranchKind::AC ? bid : 0;
          f.outage_start_hr = start_hr;
          f.repair_duration_hr = repair_hr;
          f.name = std::move(label);
          opts.faults.push_back(std::move(f));
        };

        // Manual faults with per-fault timing and branch type take priority.
        if (j.contains("manual_faults") && j["manual_faults"].is_array() && !j["manual_faults"].empty()) {
          for (const auto& mf : j["manual_faults"]) {
            const int bid = value_int_any(mf, {"branch_id", "branch_index", "branch"}, -1);
            const auto kind = hacdcpf::analysis::resilience_branch_kind_from_string(
                value_string_any(mf, {"branch_type", "branch_kind", "type"}, "AC"));
            add_fault(kind, bid,
                      value_double_any(mf, {"start_hr", "outage_start_hr"}, 0.0),
                      value_double_any(mf, {"repair_hr", "repair_duration_hr", "repair_time_hr"}, opts.default_repair_time_hr),
                      value_string_any(mf, {"label", "name"}, std::string{}));
          }
        } else {
          const double ac_start = j.value("ac_fault_start_hr", j.value("auto_fault_start_hr", 0.0));
          const double dc_start = j.value("dc_fault_start_hr", j.value("auto_fault_start_hr", 0.0));
          const double ac_repair = j.value("ac_repair_time_hr", opts.default_repair_time_hr);
          const double dc_repair = j.value("dc_repair_time_hr", opts.default_repair_time_hr);
          for (const auto& id : j.value("ac_fault_branch_ids", std::vector<int>{})) {
            add_fault(hacdcpf::analysis::ResilienceBranchKind::AC, id, ac_start, ac_repair, std::string{});
          }
          for (const auto& id : j.value("dc_fault_branch_ids", std::vector<int>{})) {
            add_fault(hacdcpf::analysis::ResilienceBranchKind::DC, id, dc_start, dc_repair, std::string{});
          }
          // Legacy API: untyped fault_branch_ids are AC branch IDs.
          if (opts.faults.empty()) {
            for (const auto& id : j.value("fault_branch_ids", std::vector<int>{})) {
              add_fault(hacdcpf::analysis::ResilienceBranchKind::AC, id, 0.0,
                        opts.default_repair_time_hr, std::string{});
            }
          }
        }
        opts.progress_callback = [](int step, double ratio, double shed) {
          return !g_session.cancel.load();
        };

        auto result = hacdcpf::analysis::run_distribution_resilience_assessment(sys, opts);

        json out;
        out["feasible"] = result.feasible;
        out["status"] = result.status;
        out["total_demand_mwh"] = result.total_demand_mwh;
        out["total_served_mwh"] = result.total_served_mwh;
        out["total_shed_mwh"] = result.total_shed_mwh;
        out["weighted_unserved_mwh"] = result.weighted_unserved_mwh;
        out["resilience_index"] = result.resilience_index;
        out["avg_restoration_ratio"] = result.avg_restoration_ratio;
        out["final_restoration_ratio"] = result.final_restoration_ratio;
        out["peak_shed_mw"] = result.peak_shed_mw;
        out["mess_energy_delivered_mwh"] = result.mess_energy_delivered_mwh;
        out["mess_travel_distance_km"] = result.mess_travel_distance_km;
        out["total_switch_actions"] = result.total_switch_actions;
        out["total_repaired_faults"] = result.total_repaired_faults;

        // Fault sequence used (auto-generated or user-specified).
        {
          json fs_arr = json::array();
          for (const auto& f : result.fault_sequence) {
            fs_arr.push_back(json{{"branch_type", hacdcpf::analysis::to_string(f.branch_kind)},
                                  {"branch_kind", hacdcpf::analysis::to_string(f.branch_kind)},
                                  {"branch_index", f.branch_index},
                                  {"branch_id", f.branch_index},
                                  {"start_hr", f.start_hr},
                                  {"repair_hr", f.repair_hr},
                                  {"name", f.name}});
          }
          out["fault_sequence"] = fs_arr;
        }

        json hours = json::array();
        json demand = json::array();
        json served = json::array();
        json shed = json::array();
        json ratio = json::array();
        json active_faults = json::array();
        json switch_actions = json::array();
        json island_counts = json::array();
        json repaired_faults_arr = json::array();
        json load_multipliers = json::array();
        json res_multipliers = json::array();
        json pv_multipliers = json::array();
        json wind_multipliers = json::array();
        json res_mw_arr = json::array();
        json disaster_stages = json::array();
        json open_ac_branch_ids = json::array();
        json open_dc_branch_ids = json::array();
        json closed_tie_branch_ids = json::array();
        json closed_dc_tie_branch_ids = json::array();
        json open_switch_ids = json::array();
        json closed_switch_ids = json::array();
        json switched_open_ids = json::array();
        json switched_closed_ids = json::array();
        json hourly = json::array();
        json shed_critical = json::array();
        json shed_high = json::array();
        json shed_medium = json::array();
        json shed_low = json::array();
        json bus_supply_kind = json::array();
        json bus_supply_index = json::array();
        json bus_supply_demand_mw = json::array();
        json bus_supply_served_mw = json::array();
        json bus_supply_shed_mw = json::array();
        json bus_supply_priority_tier = json::array();
        json bus_supply_importance = json::array();
        std::map<int, json> mess_traces;
        std::map<int, std::string> mess_names;
        // PF time-series accumulators (only populated when run_power_flow is on).
        json pf_converged_arr = json::array();
        // Per-bus: bus_index → {vm_pu: [...], va_deg: [...]}
        std::map<int, json> bus_voltage_traces;
        // Per-branch: branch_index → {pf_mw: [...], loading_pct: [...]}
        std::map<int, json> branch_flow_traces;

        for (const auto& step : result.steps) {
          hours.push_back(step.hour);
          demand.push_back(step.total_demand_mw);
          served.push_back(step.served_mw);
          shed.push_back(step.shed_mw);
          ratio.push_back(step.restoration_ratio);
          active_faults.push_back(step.active_faults);
          switch_actions.push_back(step.switch_actions);
          island_counts.push_back(step.island_count);
          repaired_faults_arr.push_back(step.repaired_faults);
          load_multipliers.push_back(step.load_multiplier);
          res_multipliers.push_back(step.res_multiplier);
          pv_multipliers.push_back(step.pv_multiplier);
          wind_multipliers.push_back(step.wind_multiplier);
          res_mw_arr.push_back(step.total_res_mw);
          disaster_stages.push_back(step.disaster_stage);
          open_ac_branch_ids.push_back(step.open_ac_branch_ids);
          open_dc_branch_ids.push_back(step.open_dc_branch_ids);
          closed_tie_branch_ids.push_back(step.closed_tie_branch_ids);
          closed_dc_tie_branch_ids.push_back(step.closed_dc_tie_branch_ids);
          open_switch_ids.push_back(step.open_switch_ids);
          closed_switch_ids.push_back(step.closed_switch_ids);
          switched_open_ids.push_back(step.switched_open_ids);
          switched_closed_ids.push_back(step.switched_closed_ids);
          hourly.push_back(json{{"step_index", step.step_index},
                                {"hour", step.hour},
                                {"disaster_stage", step.disaster_stage},
                                {"demand_mw", step.total_demand_mw},
                                {"served_mw", step.served_mw},
                                {"shed_mw", step.shed_mw},
                                {"restoration_ratio", step.restoration_ratio},
                                {"load_multiplier", step.load_multiplier},
                                {"res_multiplier", step.res_multiplier},
                                {"pv_multiplier", step.pv_multiplier},
                                {"wind_multiplier", step.wind_multiplier},
                                {"active_faults", step.active_faults},
                                {"repaired_faults", step.repaired_faults},
                                {"switch_actions", step.switch_actions},
                                {"island_count", step.island_count},
                                {"open_ac_branch_ids", step.open_ac_branch_ids},
                                {"open_dc_branch_ids", step.open_dc_branch_ids},
                                {"closed_tie_branch_ids", step.closed_tie_branch_ids},
                                {"closed_dc_tie_branch_ids", step.closed_dc_tie_branch_ids},
                                {"open_switch_ids", step.open_switch_ids},
                                {"closed_switch_ids", step.closed_switch_ids},
                                {"switched_open_ids", step.switched_open_ids},
                                {"switched_closed_ids", step.switched_closed_ids},
                                {"isolation_switch_actions", step.isolation_switch_actions},
                                {"reconfiguration_switch_actions", step.reconfiguration_switch_actions},
                                {"fault_zone_ac_bus_ids", step.fault_zone_ac_bus_ids},
                                {"fault_zone_dc_bus_ids", step.fault_zone_dc_bus_ids},
                                {"shed_by_priority", step.shed_by_priority}});
          shed_critical.push_back(step.shed_by_priority.size() > 0 ? step.shed_by_priority[0] : 0.0);
          shed_high.push_back(step.shed_by_priority.size() > 1 ? step.shed_by_priority[1] : 0.0);
          shed_medium.push_back(step.shed_by_priority.size() > 2 ? step.shed_by_priority[2] : 0.0);
          shed_low.push_back(step.shed_by_priority.size() > 3 ? step.shed_by_priority[3] : 0.0);
          bus_supply_kind.push_back(step.bus_supply_kind);
          bus_supply_index.push_back(step.bus_supply_index);
          bus_supply_demand_mw.push_back(step.bus_supply_demand_mw);
          bus_supply_served_mw.push_back(step.bus_supply_served_mw);
          bus_supply_shed_mw.push_back(step.bus_supply_shed_mw);
          bus_supply_priority_tier.push_back(step.bus_supply_priority_tier);
          bus_supply_importance.push_back(step.bus_supply_importance);
          for (const auto& ms : step.mess_states) {
            if (!mess_traces.count(ms.storage_index)) {
              mess_traces[ms.storage_index] = json{{"storage_index", ms.storage_index},
                                                   {"dispatch_mw", json::array()},
                                                   {"energy_mwh", json::array()},
                                                   {"soc", json::array()},
                                                   {"bus", json::array()},
                                                   {"target_bus", json::array()},
                                                   {"remaining_travel_hr", json::array()},
                                                   {"status", json::array()}};
              mess_names[ms.storage_index] = "MESS " + std::to_string(ms.storage_index);
            }
            mess_traces[ms.storage_index]["dispatch_mw"].push_back(ms.dispatch_mw);
            mess_traces[ms.storage_index]["energy_mwh"].push_back(ms.energy_mwh);
            mess_traces[ms.storage_index]["soc"].push_back(ms.soc);
            mess_traces[ms.storage_index]["bus"].push_back(ms.bus);
            mess_traces[ms.storage_index]["target_bus"].push_back(ms.target_bus);
            mess_traces[ms.storage_index]["remaining_travel_hr"].push_back(ms.remaining_travel_hr);
            mess_traces[ms.storage_index]["status"].push_back(ms.status);
          }
          // Collect PF voltage & flow data for this step.
          pf_converged_arr.push_back(step.pf_converged);
          for (const auto& bv : step.bus_voltages) {
            if (!bus_voltage_traces.count(bv.bus_index)) {
              bus_voltage_traces[bv.bus_index] = json{{"bus_index", bv.bus_index},
                                                      {"vm_pu", json::array()},
                                                      {"va_deg", json::array()}};
            }
            bus_voltage_traces[bv.bus_index]["vm_pu"].push_back(bv.vm_pu);
            bus_voltage_traces[bv.bus_index]["va_deg"].push_back(bv.va_deg);
          }
          for (const auto& bf : step.branch_flows) {
            if (!branch_flow_traces.count(bf.branch_index)) {
              branch_flow_traces[bf.branch_index] = json{{"branch_index", bf.branch_index},
                                                          {"from_bus", bf.from_bus},
                                                          {"to_bus", bf.to_bus},
                                                          {"pf_mw", json::array()},
                                                          {"qf_mvar", json::array()},
                                                          {"loading_pct", json::array()}};
            }
            branch_flow_traces[bf.branch_index]["pf_mw"].push_back(bf.pf_mw);
            branch_flow_traces[bf.branch_index]["qf_mvar"].push_back(bf.qf_mvar);
            branch_flow_traces[bf.branch_index]["loading_pct"].push_back(bf.loading_percent);
          }
        }
        out["hours"] = hours;
        out["demand_mw"] = demand;
        out["served_mw"] = served;
        out["shed_mw"] = shed;
        out["restoration_ratio"] = ratio;
        out["active_faults"] = active_faults;
        out["switch_actions"] = switch_actions;
        out["island_counts"] = island_counts;
        out["repaired_faults_arr"] = repaired_faults_arr;
        out["load_multipliers"] = load_multipliers;
        out["res_multipliers"] = res_multipliers;
        out["pv_multipliers"] = pv_multipliers;
        out["wind_multipliers"] = wind_multipliers;
        out["res_mw"] = res_mw_arr;
        out["disaster_stages"] = disaster_stages;
        out["open_ac_branch_ids"] = open_ac_branch_ids;
        out["open_dc_branch_ids"] = open_dc_branch_ids;
        out["closed_tie_branch_ids"] = closed_tie_branch_ids;
        out["closed_dc_tie_branch_ids"] = closed_dc_tie_branch_ids;
        out["open_switch_ids"] = open_switch_ids;
        out["closed_switch_ids"] = closed_switch_ids;
        out["switched_open_ids"] = switched_open_ids;
        out["switched_closed_ids"] = switched_closed_ids;
        out["hourly"] = hourly;
        out["model"] = result.model == hacdcpf::analysis::DistributionResilienceModel::RAStyleStageMILP
                           ? "RAStyleStageMILP"
                           : (result.model == hacdcpf::analysis::DistributionResilienceModel::MultiPeriodMIPLinDistFlow
                                  ? "MultiPeriodMIPLinDistFlow"
                                  : "HeuristicSequential");
        out["model_stats"] = json{{"solver_name", result.model_stats.solver_name},
                                   {"solver_status", result.model_stats.solver_status},
                                   {"model_scope", result.model_stats.model_scope},
                                   {"model_built", result.model_stats.model_built},
                                   {"model_solved", result.model_stats.model_solved},
                                   {"num_variables", result.model_stats.num_variables},
                                   {"num_binary_variables", result.model_stats.num_binary_variables},
                                   {"num_eq_constraints", result.model_stats.num_eq_constraints},
                                   {"num_ineq_constraints", result.model_stats.num_ineq_constraints},
                                   {"objective_value", result.model_stats.objective_value},
                                   {"mip_gap", result.model_stats.mip_gap},
                                   {"runtime_sec", result.model_stats.runtime_sec},
                                   {"formulation_notes", result.model_stats.formulation_notes}};
        out["shed_critical"] = shed_critical;
        out["shed_high"] = shed_high;
        out["shed_medium"] = shed_medium;
        out["shed_low"] = shed_low;
        out["bus_supply_kind"] = bus_supply_kind;
        out["bus_supply_index"] = bus_supply_index;
        out["bus_supply_demand_mw"] = bus_supply_demand_mw;
        out["bus_supply_served_mw"] = bus_supply_served_mw;
        out["bus_supply_shed_mw"] = bus_supply_shed_mw;
        out["bus_supply_priority_tier"] = bus_supply_priority_tier;
        out["bus_supply_importance"] = bus_supply_importance;
        json mess_arr = json::array();
        for (auto& [storage_index, trace] : mess_traces) {
          trace["name"] = mess_names[storage_index];
          mess_arr.push_back(trace);
        }
        out["mess_traces"] = mess_arr;

        // PF results
        out["pf_converged"] = pf_converged_arr;
        json bv_arr = json::array();
        for (auto& [bus_idx, trace] : bus_voltage_traces) bv_arr.push_back(trace);
        out["bus_voltage_traces"] = bv_arr;
        json bf_arr = json::array();
        for (auto& [br_idx, trace] : branch_flow_traces) bf_arr.push_back(trace);
        out["branch_flow_traces"] = bf_arr;

        res.set_content(out.dump(), "application/json");
        g_session.busy.store(false);
      } catch (const std::exception& e) {
        g_session.busy.store(false);
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

    // ---- Reliability Assessment (Frequency & Duration - Analytical) ----
    svr.Post("/api/session/run_reliability_fd",
             [](const httplib::Request& req, httplib::Response& res) {
      try {
        hacdcpf::HybridPowerSystem sys;
        {
          std::lock_guard<std::mutex> lk(g_session.mu);
          if (!g_session.current_system) throw std::runtime_error("No system loaded");
          sys = *g_session.current_system;
        }
        const auto j = json::parse(req.body.empty() ? "{}" : req.body);
        
        // Apply IEEE-24 reliability data if requested
        if (j.value("apply_ieee24_data", true)) {
          hacdcpf::analysis::apply_ieee24_reliability_data(sys);
        }
        
        // Calculate total load (with optional scaling)
        double load_scale = j.value("load_scale_factor", 1.0);
        double total_load = 0.0;
        for (const auto& bus : sys.ac.buses) {
          total_load += bus.pd_mw;
        }
        for (const auto& ld : sys.ac.loads) {
          if (ld.in_service) total_load += ld.p_mw * ld.scaling;
        }
        total_load *= load_scale;
        
        auto result = hacdcpf::analysis::run_frequency_duration_analysis(sys, total_load);
        
        json out;
        out["lolp"] = result.lolp;
        out["lole_fd"] = result.lole_fd;
        out["lolf_fd"] = result.lolf_fd;
        out["lold"] = result.lold;
        out["capacity_outage_levels"] = result.capacity_outage_levels;
        out["cumulative_probability"] = result.cumulative_probability;
        out["cumulative_frequency"] = result.cumulative_frequency;
        
        res.set_content(out.dump(), "application/json");
      } catch (const std::exception& e) {
        res.status = 400;
        res.set_content(json{{"error", e.what()}}.dump(), "application/json");
      }
    });

  // ---- Legacy endpoints ----
  svr.Post("/api/load_case", [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      auto sys = system_from_request(j);
      json out;
      out["case_name"] = j.value("case", "ieee24_3area_acdc_expanded");
      out["system_json"] = hacdcpf::io::to_json(sys, 2);
      res.set_content(out.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/pf", [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      auto sys = system_from_request(j);
      hacdcpf::PowerFlowOptions opt;
      opt.enable_converter_coordination_check = true;  // default for hybrid safety
      if (j.contains("options")) { auto& o=j["options"];
        if(o.contains("max_iter"))opt.max_iter=o["max_iter"].get<int>();
        if(o.contains("tol"))opt.tol=o["tol"].get<double>();
        if(o.contains("enable_converter_coordination_check"))opt.enable_converter_coordination_check=o["enable_converter_coordination_check"].get<bool>();
      }
      auto pf = hacdcpf::solve_power_flow(sys, opt);
      json out;
      out["converged"]=pf.converged; out["iterations"]=pf.iterations;
      out["residual"]=pf.residual; out["vm"]=pf.vm; out["va"]=pf.va; out["vdc"]=pf.vdc;
      json ba=json::array(); for(const auto& bf:pf.branch_flows)ba.push_back(std::abs(bf.pf_mw));
      out["branch_abs"]=ba;
      out["warnings"]=pf.diagnostics.warnings;
      const auto& rep = pf.diagnostics.converter_coordination;
      json coord;
      coord["enabled"] = rep.enabled;
      coord["feasible"] = rep.feasible;
      coord["blocking_count"] = rep.blocking_count();
      coord["fatal_count"] = rep.fatal_count();
      coord["error_count"] = rep.error_count();
      coord["warning_count"] = rep.warning_count();
      coord["issues"] = json::array();
      for (const auto& issue : rep.issues) {
        coord["issues"].push_back(json{
          {"severity", hacdcpf::powerflow::coordination_severity_str(issue.severity)},
          {"rule_id", issue.rule_id},
          {"component_type", issue.component_type},
          {"component_index", issue.component_index},
          {"island_index", issue.island_index},
          {"message", issue.message}
        });
      }
      coord["dc_islands"] = json::array();
      for (const auto& isle : rep.dc_islands) {
        coord["dc_islands"].push_back(json{
          {"island_index", isle.island_index},
          {"dc_buses", isle.dc_buses},
          {"declared_v_buses", isle.declared_v_buses},
          {"hard_vdc_sources", isle.hard_vdc_sources},
          {"droop_sources", isle.droop_sources},
          {"fixed_power_devices", isle.fixed_power_devices},
          {"fixed_power_mw", isle.fixed_power_mw},
          {"flexible_up_mw", isle.flexible_up_mw},
          {"flexible_down_mw", isle.flexible_down_mw}
        });
        auto& island_json = coord["dc_islands"].back();
        island_json["voltage_sources"] = json::array();
        for (const auto& source : isle.voltage_sources) {
          island_json["voltage_sources"].push_back(json{
            {"component_type", source.component_type},
            {"component_index", source.component_index},
            {"bus", source.bus},
            {"v_set_pu", source.v_set_pu},
            {"has_v_set", source.has_v_set},
            {"droop", source.droop}
          });
        }
      }
      out["converter_coordination"] = coord;
      res.set_content(out.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/opf/ac", [](const httplib::Request& req, httplib::Response& res) {
    try {
      auto sys = system_from_request(json::parse(req.body.empty() ? "{}" : req.body));
      hacdcpf::opf::ACOPFOptions opt; opt.allow_fallback=true; opt.max_inner_iterations=120;
      auto r = hacdcpf::solve_ac_opf(sys, opt);
      json out;
      out["converged"]=r.converged; out["iterations"]=r.iterations;
      out["objective"]=r.objective; out["status"]=r.status;
      out["vm"]=r.vm; out["va"]=r.va; out["pg_mw"]=r.pg_mw; out["qg_mvar"]=r.qg_mvar;
      res.set_content(out.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/opf/dc", [](const httplib::Request& req, httplib::Response& res) {
    try {
      auto sys = system_from_request(json::parse(req.body.empty() ? "{}" : req.body));
      hacdcpf::opf::DCOPFOptions opt;
      auto r = hacdcpf::solve_dc_opf(sys, opt);
      json out;
      out["converged"]=r.converged; out["iterations"]=r.iterations;
      out["objective"]=r.objective; out["status"]=r.status;
      out["pg_mw"]=r.pg_mw; out["pf_mw"]=r.pf_mw;
      out["lmp"]=r.lmp; out["total_load_shedding_mw"]=r.total_load_shedding_mw;
      res.set_content(out.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  svr.Post("/api/sc", [](const httplib::Request& req, httplib::Response& res) {
    try {
      const auto j = json::parse(req.body.empty() ? "{}" : req.body);
      auto sys = system_from_request(j);
      hacdcpf::analysis::SCOptions opt;
      if (j.contains("options")) { auto& o=j["options"];
        std::string ft=o.value("fault_type",std::string("ThreePhase"));
        if(ft=="SinglePhaseGround")opt.fault_type=hacdcpf::analysis::FaultType::SinglePhaseGround;
        else if(ft=="TwoPhase")opt.fault_type=hacdcpf::analysis::FaultType::TwoPhase;
        else if(ft=="TwoPhaseGround")opt.fault_type=hacdcpf::analysis::FaultType::TwoPhaseGround;
        if(o.contains("c_factor"))opt.c_factor=o["c_factor"].get<double>();
      }
      auto sc = hacdcpf::analysis::compute_short_circuit(sys, opt);
      json out;
      out["fault_type"]=[&](){switch(opt.fault_type){
        case hacdcpf::analysis::FaultType::ThreePhase:return"ThreePhase";
        case hacdcpf::analysis::FaultType::SinglePhaseGround:return"SinglePhaseGround";
        case hacdcpf::analysis::FaultType::TwoPhase:return"TwoPhase";
        case hacdcpf::analysis::FaultType::TwoPhaseGround:return"TwoPhaseGround";
      }return"ThreePhase";}();
      out["bus_results"]=json::array();
      for(const auto& br:sc.bus_results)
        out["bus_results"].push_back(json{{"bus_id",br.bus_id},{"sk_mva",br.sk_mva},{"ikpp_ka",br.ikpp_ka}});
      res.set_content(out.dump(), "application/json");
    } catch (const std::exception& e) {
      res.status = 400;
      res.set_content(json{{"error", e.what()}}.dump(), "application/json");
    }
  });

  // Serve static files for the ETAP-style frontend from the web/ directory.
  // The directory is located robustly so the canvas UI loads regardless of the
  // working directory the binary is launched from (repo root, build/,
  // build/tests/, an installed prefix, ...).  Previously only cwd/web and
  // cwd/../web were tried, so launching the binary from its build location
  // (build/tests/) silently skipped the mount and every /xjtu/ asset 404'd —
  // i.e. the toolbars rendered but no JS loaded, so buttons and the canvas were
  // dead.  HACDCPF_PROJECT_ROOT (a compile-time define for this target) is the
  // authoritative fallback for an in-tree build.
  const auto web_dir = [&]() -> std::string {
    std::vector<fs::path> candidates = {
        fs::current_path() / "web",
        fs::current_path() / ".." / "web",
        fs::current_path() / ".." / ".." / "web",
        fs::current_path() / ".." / ".." / ".." / "web",
    };
#ifdef HACDCPF_PROJECT_ROOT
    candidates.push_back(fs::path(HACDCPF_PROJECT_ROOT) / "web");
#endif
    for (const auto& c : candidates) {
      std::error_code ec;
      if (fs::exists(c, ec) && !ec) return fs::canonical(c).string();
    }
    return (fs::current_path() / "web").string();
  }();
  if (fs::exists(web_dir)) {
    svr.set_mount_point("/xjtu", web_dir);
    std::cout << "XJTU frontend: http://" << args.host << ":" << args.port << "/xjtu/\n";
    std::cout << "  (serving from " << web_dir << ")\n";
  } else {
    std::cerr << "WARNING: web/ frontend directory not found.\n"
              << "         Looked relative to cwd (" << fs::current_path().string() << ")"
#ifdef HACDCPF_PROJECT_ROOT
              << " and " << HACDCPF_PROJECT_ROOT
#endif
              << ".\n         The /xjtu/ canvas UI will be unavailable (404). "
                 "Launch from the repo root or pass a correct working directory.\n";
  }

  // CORS headers for cross-origin access
  svr.set_default_headers({
    {"Access-Control-Allow-Origin", "*"},
    {"Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS"},
    {"Access-Control-Allow-Headers", "Content-Type, Authorization"}
  });

  // Handle OPTIONS preflight requests
  svr.Options("/(.*)", [](const httplib::Request&, httplib::Response& res) {
    res.status = 204;
  });

  std::cout << "Hybrid AC/DC Planning Studio running at http://"
            << args.host << ":" << args.port << "\n";
  std::cout << "Press Ctrl+C to stop.\n";
  if (!svr.listen(args.host, args.port)) {
    std::cerr << "Failed to start server on " << args.host << ":" << args.port << "\n";
    return 2;
  }
  return 0;
}
