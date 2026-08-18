#include "hacdcpf/sppt/fault_campaign.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/sppt/certificate.hpp"
#include "hacdcpf/sppt/guard.hpp"
#include "hacdcpf/validation/validate_system.hpp"

namespace hacdcpf::sppt {
namespace {

using Clock = std::chrono::steady_clock;

template <typename Function>
double timed_ms(Function&& function) {
  const auto start = Clock::now();
  function();
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

double max_abs_difference(const std::vector<double>& first,
                          const std::vector<double>& second) {
  if (first.size() != second.size()) return std::numeric_limits<double>::infinity();
  double maximum = 0.0;
  for (std::size_t i = 0; i < first.size(); ++i)
    maximum = std::max(maximum, std::abs(first[i] - second[i]));
  return maximum;
}

double max_branch_difference(const std::vector<BranchFlow>& first,
                             const std::vector<BranchFlow>& second) {
  if (first.size() != second.size()) return std::numeric_limits<double>::infinity();
  double maximum = 0.0;
  for (std::size_t i = 0; i < first.size(); ++i) {
    maximum = std::max(maximum, std::abs(first[i].pf_mw - second[i].pf_mw));
    maximum = std::max(maximum, std::abs(first[i].pt_mw - second[i].pt_mw));
  }
  return maximum;
}

double max_converter_difference(const std::vector<VSCTransfer>& first,
                                const std::vector<VSCTransfer>& second) {
  std::unordered_map<int, const VSCTransfer*> reference;
  for (const auto& transfer : first) reference.emplace(transfer.index, &transfer);
  double maximum = 0.0;
  for (const auto& transfer : second) {
    const auto it = reference.find(transfer.index);
    if (it == reference.end()) return std::numeric_limits<double>::infinity();
    maximum = std::max(maximum,
                       std::abs(transfer.p_ac_mw - it->second->p_ac_mw));
    maximum = std::max(maximum,
                       std::abs(transfer.p_dc_mw - it->second->p_dc_mw));
  }
  return second.size() == first.size() ? maximum
                                       : std::numeric_limits<double>::infinity();
}

std::pair<double, double> wilson_interval(int detected, int samples) {
  if (samples <= 0) return {0.0, 0.0};
  // Wilson (1927), 95% score interval with z=1.959963984540054.
  constexpr double z = 1.959963984540054;
  const double n = static_cast<double>(samples);
  const double p = static_cast<double>(detected) / n;
  const double denominator = 1.0 + z * z / n;
  const double centre = (p + z * z / (2.0 * n)) / denominator;
  const double radius = z * std::sqrt((p * (1.0 - p) + z * z / (4.0 * n)) / n) /
                        denominator;
  return {std::max(0.0, centre - radius), std::min(1.0, centre + radius)};
}

bool is_structural_fault(CampaignFaultClass fault) {
  return fault != CampaignFaultClass::ValidControl &&
         fault != CampaignFaultClass::PlausibleLoadError &&
         fault != CampaignFaultClass::PlausibleConverterSetpointError;
}

bool issue_localizes(const validation::ValidationReport& report,
                     const std::string& component,
                     const std::string& id,
                     const std::string& field) {
  return std::any_of(report.issues.begin(), report.issues.end(),
                     [&](const validation::ValidationIssue& issue) {
    const bool component_match = component.empty() || issue.component_type == component;
    const bool id_match = id.empty() || issue.component_id == id;
    const bool field_match = field.empty() || issue.field == field;
    return issue.severity == validation::Severity::Error && component_match &&
           id_match && field_match;
  });
}

struct InjectedModel {
  HybridPowerSystem system;
  std::string target;
  double magnitude{0.0};
  std::string component;
  std::string component_id;
  std::string field;
};

template <typename Generator>
std::size_t choose_index(std::size_t size, Generator& generator) {
  std::uniform_int_distribution<std::size_t> distribution(0, size - 1);
  return distribution(generator);
}

InjectedModel inject_fault(const HybridPowerSystem& base,
                           CampaignFaultClass fault,
                           std::mt19937_64& generator) {
  InjectedModel injected{base};
  if (fault == CampaignFaultClass::ValidControl) {
    std::shuffle(injected.system.ac.buses.begin(), injected.system.ac.buses.end(),
                 generator);
    std::shuffle(injected.system.ac.branches.begin(),
                 injected.system.ac.branches.end(), generator);
    std::shuffle(injected.system.dc.buses.begin(), injected.system.dc.buses.end(),
                 generator);
    std::shuffle(injected.system.dc.branches.begin(),
                 injected.system.dc.branches.end(), generator);
    std::shuffle(injected.system.vsc_converters.begin(),
                 injected.system.vsc_converters.end(), generator);
    injected.target = "none";
    return injected;
  }

  if (fault == CampaignFaultClass::PlausibleConverterSetpointError) {
    std::vector<std::size_t> candidates;
    for (std::size_t i = 0; i < injected.system.vsc_converters.size(); ++i) {
      const auto& converter = injected.system.vsc_converters[i];
      if (converter.in_service &&
          converter.control_mode == ConverterMode::PQ_MODE)
        candidates.push_back(i);
    }
    if (candidates.empty()) {
      injected.target = "unsupported:no-in-service-pq-vsc";
      return injected;
    }
    auto& converter = injected.system.vsc_converters[
        candidates[choose_index(candidates.size(), generator)]];
    const double rating_scale =
        std::max(1.0, std::min(injected.system.base_mva,
                              std::max(std::abs(converter.pmin_mw),
                                       std::abs(converter.pmax_mw))));
    const double setpoint_scale =
        std::max(std::abs(converter.p_set_mw), 0.10 * rating_scale);
    double delta = std::uniform_real_distribution<double>(0.05, 0.30)(generator) *
                   setpoint_scale;
    if ((generator() & 1U) == 0U) delta = -delta;
    converter.p_set_mw += delta;
    converter.p_schedule_mw += delta;
    converter.p_initial_mw += delta;
    injected.magnitude = delta;
    injected.target = "VSCConverter/" + std::to_string(converter.index) +
                      "/p_set_mw";
    return injected;
  }

  if (fault == CampaignFaultClass::MissingVscAcTerminal ||
      fault == CampaignFaultClass::MissingVscDcTerminal ||
      fault == CampaignFaultClass::InvalidConverterEfficiency) {
    if (injected.system.vsc_converters.empty()) {
      injected.target = "unsupported:no-vsc";
      return injected;
    }
    auto& converter = injected.system.vsc_converters[
        choose_index(injected.system.vsc_converters.size(), generator)];
    injected.component = "VSCConverter";
    injected.component_id = std::to_string(converter.index);
    if (fault == CampaignFaultClass::MissingVscAcTerminal) {
      converter.bus_ac = 100000000 + static_cast<int>(generator() % 1000000);
      injected.field = "bus_ac";
      injected.target = "VSCConverter/" + injected.component_id + "/bus_ac";
    } else if (fault == CampaignFaultClass::MissingVscDcTerminal) {
      converter.bus_dc = 200000000 + static_cast<int>(generator() % 1000000);
      injected.field = "bus_dc";
      injected.target = "VSCConverter/" + injected.component_id + "/bus_dc";
    } else if (fault == CampaignFaultClass::InvalidConverterEfficiency) {
      converter.eta = 1.01 + std::uniform_real_distribution<double>(0.0, 0.19)(generator);
      injected.magnitude = converter.eta;
      injected.field = "eta";
      injected.target = "VSCConverter/" + injected.component_id + "/eta";
    }
    return injected;
  }

  if (fault == CampaignFaultClass::MissingAcReference) {
    for (auto& bus : injected.system.ac.buses)
      if (bus.bus_type == BusType::SLACK) bus.bus_type = BusType::PQ;
    for (auto& generator_device : injected.system.ac.generators)
      generator_device.is_slack = false;
    for (auto& grid : injected.system.ac.external_grids) grid.in_service = false;
    injected.component = "ACSystem";
    injected.target = "ACSystem/reference";
    return injected;
  }

  if (fault == CampaignFaultClass::MissingDcSupport) {
    for (auto& converter : injected.system.vsc_converters) converter.in_service = false;
    for (auto& storage : injected.system.dc.storage) storage.in_service = false;
    for (auto& storage : injected.system.dc.dc_storage) storage.in_service = false;
    for (auto& converter : injected.system.dc.dcdc_converters)
      converter.in_service = false;
    injected.component = "dc_island";
    injected.target = "dc_island/voltage_support";
    return injected;
  }

  if (fault == CampaignFaultClass::DuplicateAcBusIdentity) {
    if (injected.system.ac.buses.size() < 2) {
      injected.target = "unsupported:fewer-than-two-ac-buses";
      return injected;
    }
    const std::size_t first = choose_index(injected.system.ac.buses.size(), generator);
    std::size_t second = choose_index(injected.system.ac.buses.size() - 1, generator);
    if (second >= first) ++second;
    injected.system.ac.buses[second].index = injected.system.ac.buses[first].index;
    injected.component = "ACBus";
    injected.component_id = std::to_string(injected.system.ac.buses[first].index);
    injected.field = "index";
    injected.target = "ACBus/" + injected.component_id + "/index";
    return injected;
  }

  if (fault == CampaignFaultClass::PlausibleLoadError) {
    if (!injected.system.ac.loads.empty()) {
      std::vector<std::size_t> candidates;
      for (std::size_t i = 0; i < injected.system.ac.loads.size(); ++i) {
        const auto& load = injected.system.ac.loads[i];
        if (load.in_service && std::abs(load.p_mw) + std::abs(load.q_mvar) > 1e-9)
          candidates.push_back(i);
      }
      if (!candidates.empty()) {
        auto& load = injected.system.ac.loads[
            candidates[choose_index(candidates.size(), generator)]];
        const double factor =
            std::uniform_real_distribution<double>(0.70, 1.30)(generator);
        load.p_mw *= factor;
        load.q_mvar *= factor;
        injected.magnitude = factor;
        injected.target = "Load/" + std::to_string(load.index) + "/load";
        return injected;
      }
    }
    std::vector<std::size_t> candidates;
    for (std::size_t i = 0; i < injected.system.ac.buses.size(); ++i) {
      const auto& bus = injected.system.ac.buses[i];
      if (std::abs(bus.pd_mw) + std::abs(bus.qd_mvar) > 1e-9)
        candidates.push_back(i);
    }
    if (candidates.empty()) {
      injected.target = "unsupported:no-authored-bus-load";
      return injected;
    }
    auto& bus = injected.system.ac.buses[
        candidates[choose_index(candidates.size(), generator)]];
    const double factor = std::uniform_real_distribution<double>(0.70, 1.30)(generator);
    bus.pd_mw *= factor;
    bus.qd_mvar *= factor;
    injected.magnitude = factor;
    injected.target = "ACBus/" + std::to_string(bus.index) + "/load";
  }
  return injected;
}

std::string csv_escape(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) return value;
  std::string escaped = "\"";
  for (char character : value) {
    if (character == '\"') escaped += '\"';
    escaped += character;
  }
  return escaped + "\"";
}

std::string fixed(double value, int precision = 6) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string latex_escape(const std::string& value) {
  std::string escaped;
  for (char character : value) {
    if (character == '_' || character == '%' || character == '&' ||
        character == '#')
      escaped.push_back('\\');
    escaped.push_back(character);
  }
  return escaped;
}

void append_summary(FaultCampaign& campaign, CampaignFaultClass fault,
                    const char* detector,
                    const std::function<bool(const FaultCampaignRow&)>& eligible,
                    const std::function<bool(const FaultCampaignRow&)>& rejected) {
  FaultCampaignSummaryRow summary;
  summary.detector = detector;
  summary.fault = fault;
  for (const auto& row : campaign.rows) {
    if (row.fault != fault || row.target.rfind("unsupported:", 0) == 0 ||
        !eligible(row))
      continue;
    ++summary.samples;
    if (rejected(row)) ++summary.detected;
    if (rejected(row) && row.localized) ++summary.localized;
  }
  summary.detection_rate = summary.samples > 0
                               ? static_cast<double>(summary.detected) / summary.samples
                               : 0.0;
  const auto interval = wilson_interval(summary.detected, summary.samples);
  summary.wilson_low = interval.first;
  summary.wilson_high = interval.second;
  campaign.summaries.push_back(std::move(summary));
}

}  // namespace

const char* to_string(CampaignFaultClass fault) noexcept {
  switch (fault) {
    case CampaignFaultClass::ValidControl: return "valid_control";
    case CampaignFaultClass::MissingVscAcTerminal: return "missing_vsc_ac_terminal";
    case CampaignFaultClass::MissingVscDcTerminal: return "missing_vsc_dc_terminal";
    case CampaignFaultClass::MissingAcReference: return "missing_ac_reference";
    case CampaignFaultClass::MissingDcSupport: return "missing_dc_support";
    case CampaignFaultClass::InvalidConverterEfficiency: return "invalid_converter_efficiency";
    case CampaignFaultClass::DuplicateAcBusIdentity: return "duplicate_ac_bus_identity";
    case CampaignFaultClass::PlausibleLoadError: return "plausible_load_error";
    case CampaignFaultClass::PlausibleConverterSetpointError:
      return "plausible_converter_setpoint_error";
  }
  return "unknown";
}

FaultCampaign run_fault_campaign(
    const std::vector<std::pair<std::string, HybridPowerSystem>>& systems,
    const FaultCampaignOptions& options) {
  FaultCampaign campaign;
  campaign.options = options;
  std::mt19937_64 generator(options.seed);
  const std::vector<CampaignFaultClass> faults = {
      CampaignFaultClass::ValidControl,
      CampaignFaultClass::MissingVscAcTerminal,
      CampaignFaultClass::MissingVscDcTerminal,
      CampaignFaultClass::MissingAcReference,
      CampaignFaultClass::MissingDcSupport,
      CampaignFaultClass::InvalidConverterEfficiency,
      CampaignFaultClass::DuplicateAcBusIdentity,
      CampaignFaultClass::PlausibleLoadError,
      CampaignFaultClass::PlausibleConverterSetpointError};

  for (const auto& [case_name, base] : systems) {
    PowerFlowResult baseline;
    bool baseline_available = false;
    if (static_cast<int>(base.ac.buses.size()) <= options.max_impact_solver_ac_buses) {
      try {
        baseline = solve_power_flow(base);
        baseline_available = baseline.converged;
      } catch (const std::exception&) {
        baseline_available = false;
      }
    }

    for (CampaignFaultClass fault : faults) {
      for (int repetition = 0; repetition < std::max(1, options.repetitions);
           ++repetition) {
        InjectedModel injected = inject_fault(base, fault, generator);
        FaultCampaignRow row;
        row.case_name = case_name;
        row.fault = fault;
        row.repetition = repetition;
        row.target = injected.target;
        row.magnitude = injected.magnitude;
        row.expected_admissible = !is_structural_fault(fault);
        if (row.target.rfind("unsupported:", 0) == 0) {
          row.note = row.target;
          campaign.rows.push_back(std::move(row));
          continue;
        }

        validation::ValidationReport validation_report;
        row.validation_ms = timed_ms([&] {
          validation_report = validation::validate(
              injected.system, validation::ValidationLevel::SolverReady);
        });
        row.validation_rejects = !validation_report.is_valid();
        if (row.validation_rejects)
          row.note = "validation: " + validation_report.summary();
        row.localized = issue_localizes(validation_report, injected.component,
                                        injected.component_id, injected.field);

        try {
          row.projection_ms = timed_ms([&] {
            const auto projected = project_to_canonical_models(injected.system);
            if (projected.ac.buses.empty() && !injected.system.ac.buses.empty())
              row.note = "projection unexpectedly removed every AC bus";
          });
        } catch (const std::exception& error) {
          row.note = std::string("projection: ") + error.what();
        }

        GuardVerdict guard;
        row.guard_ms = timed_ms([&] { guard = guard_system(injected.system); });
        row.sppt_rejects = !guard.accepted;
        if (!row.localized && row.sppt_rejects &&
            (fault == CampaignFaultClass::MissingAcReference ||
             fault == CampaignFaultClass::MissingDcSupport))
          row.localized = guard.reason == "validation" ||
                          guard.reason == "ill-posed reference";

        PowerFlowResult solved;
        const bool within_structural_solver_scope =
            static_cast<int>(injected.system.ac.buses.size()) <=
            options.max_structural_solver_ac_buses;
        const bool within_impact_solver_scope =
            static_cast<int>(injected.system.ac.buses.size()) <=
            options.max_impact_solver_ac_buses;
        if ((!is_structural_fault(fault) && within_impact_solver_scope) ||
            (options.solve_structural_faults && within_structural_solver_scope)) {
          row.solver_attempted = true;
          try {
            row.solve_ms = timed_ms([&] { solved = solve_power_flow(injected.system); });
            row.solver_converged = solved.converged;
            row.solver_residual = solved.residual;
            row.solver_rejects = !solved.converged || !std::isfinite(solved.residual) ||
                                 solved.residual > options.residual_tolerance;
          } catch (const std::exception& error) {
            row.solver_rejects = true;
            row.note += (row.note.empty() ? "" : "; ") +
                        std::string("solve: ") + error.what();
          }
        }
        if (is_structural_fault(fault) && options.solve_structural_faults &&
            !within_structural_solver_scope)
          row.note += (row.note.empty() ? "" : "; ") +
                      std::string("solver-only baseline excluded above structural-fault AC-bus budget") ;
        if (!is_structural_fault(fault) && !within_impact_solver_scope)
          row.note += (row.note.empty() ? "" : "; ") +
                      std::string("physical-impact solve excluded above AC-bus budget");
        row.validation_solver_rejects = row.validation_rejects || row.solver_rejects;

        row.sppt_independent_rejects = row.sppt_rejects;
        if (!row.sppt_rejects && row.solver_converged) {
          const IndependentResidualCertificate certificate =
              certify_independent_hybrid_residual(injected.system, solved,
                                                   options.residual_tolerance);
          row.independent_supported = certificate.supported;
          row.independent_residual = certificate.total_residual;
          row.sppt_independent_rejects =
              certificate.supported &&
              (!certificate.converged ||
               !std::isfinite(certificate.total_residual) ||
               certificate.total_residual > options.residual_tolerance);
        }

        if (!is_structural_fault(fault) && fault != CampaignFaultClass::ValidControl &&
            baseline_available && row.solver_converged) {
          row.max_ac_voltage_error_pu = max_abs_difference(baseline.vm, solved.vm);
          row.max_dc_voltage_error_pu = max_abs_difference(baseline.vdc, solved.vdc);
          row.max_branch_active_error_mw =
              max_branch_difference(baseline.branch_flows, solved.branch_flows);
          row.max_converter_active_error_mw =
              max_converter_difference(baseline.vsc_transfers, solved.vsc_transfers);
        }
        campaign.rows.push_back(std::move(row));
      }
    }
  }

  for (CampaignFaultClass fault : faults) {
    const auto all = [](const FaultCampaignRow&) { return true; };
    const auto solver_attempted = [](const FaultCampaignRow& row) {
      return row.solver_attempted;
    };
    append_summary(campaign, fault, "validation", all,
                   [](const FaultCampaignRow& row) { return row.validation_rejects; });
    append_summary(campaign, fault, "solver_only", solver_attempted,
                   [](const FaultCampaignRow& row) { return row.solver_rejects; });
    append_summary(campaign, fault, "validation_plus_solver", all,
                   [](const FaultCampaignRow& row) {
                     return row.validation_solver_rejects;
                   });
    append_summary(campaign, fault, "full_sppt", all,
                   [](const FaultCampaignRow& row) { return row.sppt_rejects; });
    append_summary(campaign, fault, "sppt_plus_independent", all,
                   [](const FaultCampaignRow& row) {
                     return row.sppt_independent_rejects;
                   });
  }
  return campaign;
}

std::string FaultCampaign::samples_csv() const {
  std::ostringstream stream;
  stream << "case,fault,repetition,target,magnitude,expected_admissible,"
            "validation_rejects,solver_rejects,validation_solver_rejects,"
            "sppt_rejects,sppt_independent_rejects,localized,solver_attempted,"
            "solver_converged,solver_residual,independent_supported,"
            "independent_residual,validation_ms,projection_ms,guard_ms,solve_ms,"
            "max_ac_voltage_error_pu,max_dc_voltage_error_pu,"
            "max_branch_active_error_mw,max_converter_active_error_mw,note\n";
  for (const auto& row : rows) {
    stream << csv_escape(row.case_name) << ',' << to_string(row.fault) << ','
           << row.repetition << ',' << csv_escape(row.target) << ','
           << fixed(row.magnitude) << ',' << row.expected_admissible << ','
           << row.validation_rejects << ',' << row.solver_rejects << ','
           << row.validation_solver_rejects << ',' << row.sppt_rejects << ','
           << row.sppt_independent_rejects << ',' << row.localized << ','
           << row.solver_attempted << ',' << row.solver_converged << ','
           << fixed(row.solver_residual, 12) << ',' << row.independent_supported << ','
           << fixed(row.independent_residual, 12) << ',' << fixed(row.validation_ms)
           << ',' << fixed(row.projection_ms) << ',' << fixed(row.guard_ms) << ','
           << fixed(row.solve_ms) << ',' << fixed(row.max_ac_voltage_error_pu, 9)
           << ',' << fixed(row.max_dc_voltage_error_pu, 9) << ','
           << fixed(row.max_branch_active_error_mw, 6) << ','
           << fixed(row.max_converter_active_error_mw, 6) << ','
           << csv_escape(row.note) << '\n';
  }
  return stream.str();
}

std::string FaultCampaign::summary_csv() const {
  std::ostringstream stream;
  stream << "detector,fault,samples,detected,localized,detection_rate,"
            "wilson_95_low,wilson_95_high\n";
  for (const auto& row : summaries)
    stream << row.detector << ',' << to_string(row.fault) << ',' << row.samples << ','
           << row.detected << ',' << row.localized << ','
           << fixed(row.detection_rate) << ',' << fixed(row.wilson_low) << ','
           << fixed(row.wilson_high) << '\n';
  return stream.str();
}

std::string FaultCampaign::to_latex() const {
  std::ostringstream stream;
  stream << "% Auto-generated by hacdcpf::sppt::FaultCampaign::to_latex()\n"
         << "\\begin{tabular}{@{}llrrrr@{}}\n\\toprule\n"
         << "Detector & Fault class & $N$ & detected & localized & rate [95\\% CI] \\\\\n"
         << "\\midrule\n";
  for (const auto& row : summaries) {
    if (row.fault == CampaignFaultClass::PlausibleLoadError ||
        row.fault == CampaignFaultClass::PlausibleConverterSetpointError)
      continue;
    stream << "\\texttt{" << latex_escape(row.detector) << "} & \\texttt{"
           << latex_escape(to_string(row.fault)) << "} & " << row.samples
           << " & " << row.detected << " & " << row.localized << " & "
           << fixed(row.detection_rate, 3) << " [" << fixed(row.wilson_low, 3)
           << ", " << fixed(row.wilson_high, 3) << "] \\\\\n";
  }
  stream << "\\bottomrule\n\\end{tabular}\n";
  return stream.str();
}

}  // namespace hacdcpf::sppt
