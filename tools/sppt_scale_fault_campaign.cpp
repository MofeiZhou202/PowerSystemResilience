#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/io/external_grid_io.hpp"
#include "hacdcpf/sppt/agent.hpp"
#include "hacdcpf/sppt/fault_campaign.hpp"

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "."
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;

namespace {

struct PublicStudySystem {
  std::string label;
  std::string ac_source;
  std::string source_reference;
  std::size_t expected_ac_buses;
  std::size_t import_warnings;
  std::size_t import_skipped;
  HybridPowerSystem system;
};

HybridPowerSystem attach_public_two_terminal_overlay(HybridPowerSystem system,
                                                     const std::string& label) {
  if (system.ac.buses.empty())
    throw std::runtime_error(label + " imported no AC buses");

  int slack_bus = 0;
  for (const auto& bus : system.ac.buses) {
    if (bus.in_service && bus.bus_type == BusType::SLACK) {
      slack_bus = bus.index;
      break;
    }
  }
  if (slack_bus == 0)
    throw std::runtime_error(label + " imported no in-service slack bus");

  int load_bus = 0;
  double largest_load_mw = -1.0;
  double total_load_mw = 0.0;
  for (const auto& load : system.ac.loads) {
    if (!load.in_service) continue;
    total_load_mw += std::max(0.0, load.p_mw);
    if (load.p_mw > largest_load_mw ||
        (load.p_mw == largest_load_mw && load.bus < load_bus)) {
      largest_load_mw = load.p_mw;
      load_bus = load.bus;
    }
  }
  if (load_bus == 0 || load_bus == slack_bus) {
    for (const auto& bus : system.ac.buses) {
      if (bus.in_service && bus.index != slack_bus) {
        load_bus = bus.index;
        break;
      }
    }
  }
  if (load_bus == 0)
    throw std::runtime_error(label + " has no distinct VSC terminal");

  const double transfer_mw = std::clamp(
      0.02 * total_load_mw, 0.001 * system.base_mva, 0.05 * system.base_mva);
  system.name = label;
  system.dc.base_mva = system.base_mva;
  system.dc.name = label + " deterministic DC overlay";
  system.dc.buses = {
      DCBus{.index = 1, .bus_type = DCBusType::DC_P, .vm_pu = 1.0,
            .in_service = true, .name = "DC-P"},
      DCBus{.index = 2, .bus_type = DCBusType::DC_V, .vm_pu = 1.02,
            .in_service = true, .name = "DC-V"}};
  system.dc.branches = {
      DCBranch{.index = 1, .from_bus = 1, .to_bus = 2, .r_pu = 0.02,
               .in_service = true, .name = "Public two-terminal DC line"}};
  system.vsc_converters = {
      VSCConverter{.index = 1, .bus_ac = load_bus, .bus_dc = 1,
                   .in_service = true, .control_mode = ConverterMode::PQ_MODE,
                   .p_set_mw = transfer_mw, .q_set_mvar = 0.0,
                   .v_dc_set_pu = 1.0, .v_ac_set_pu = 1.0, .eta = 0.99,
                   .k_vdc = 0.1, .pmax_mw = 0.10 * system.base_mva,
                   .pmin_mw = -0.10 * system.base_mva,
                   .qmax_mvar = 0.10 * system.base_mva,
                   .qmin_mvar = -0.10 * system.base_mva,
                   .name = "PQ VSC at largest imported load"},
      VSCConverter{.index = 2, .bus_ac = slack_bus, .bus_dc = 2,
                   .in_service = true, .control_mode = ConverterMode::VDC_Q,
                   .p_set_mw = 0.0, .q_set_mvar = 0.0,
                   .v_dc_set_pu = 1.02, .v_ac_set_pu = 1.0, .eta = 0.99,
                   .k_vdc = 0.1, .pmax_mw = 0.10 * system.base_mva,
                   .pmin_mw = -0.10 * system.base_mva,
                   .qmax_mvar = 0.10 * system.base_mva,
                   .qmin_mvar = -0.10 * system.base_mva,
                   .name = "VDC-Q VSC at imported source"}};
  return system;
}

PublicStudySystem load_public_gridlabd_variant(
    const fs::path& root, const std::string& filename,
    const std::string& source_reference, std::size_t expected_ac_buses) {
  const fs::path path = root / "data" / "test_cases" /
      "gridlab-d-code-r5643-Taxonomy_Feeders" / filename;
  io::GridLABDImportOptions options;
  options.mode = io::ImportMode::Permissive;
  options.default_base_mva = 10.0;
  const auto report = io::load_gridlabd_with_report(path, options);
  const std::string label = filename.substr(0, filename.size() - 4) + "-ACDC";
  return {label, "GridLAB-D r5643 taxonomy feeder " + filename,
          source_reference, expected_ac_buses, report.warnings.size(),
          report.skipped.size(),
          attach_public_two_terminal_overlay(report.system, label)};
}

std::string converter_mode_name(ConverterMode mode) {
  switch (mode) {
    case ConverterMode::PQ_MODE: return "PQ";
    case ConverterMode::VDC_Q: return "VDC-Q";
    case ConverterMode::VDC_VAC: return "VDC-VAC";
    case ConverterMode::AC_PV: return "AC-PV";
    case ConverterMode::AC_GRID_FORMING: return "AC-GFM";
    case ConverterMode::DC_V_DROOP_AC_V: return "VDC-droop-VAC";
  }
  return "unknown";
}

std::string public_manifest_csv(const std::vector<PublicStudySystem>& systems) {
  std::ostringstream out;
  out << "system,ac_source,source_reference,ac_buses,ac_branches,dc_buses,dc_branches,"
         "import_warnings,import_skipped,vsc_index,ac_terminal,dc_terminal,mode,"
         "p_set_mw,q_set_mvar,vdc_set_pu,eta,loss_percent,loss_mw,dc_branch_list\n";
  out << std::setprecision(12);
  for (const auto& item : systems) {
    std::ostringstream dc_branches;
    for (std::size_t i = 0; i < item.system.dc.branches.size(); ++i) {
      const auto& branch = item.system.dc.branches[i];
      if (i != 0) dc_branches << ';';
      dc_branches << branch.from_bus << '-' << branch.to_bus << ':' << branch.r_pu;
    }
    for (const auto& converter : item.system.vsc_converters) {
      out << '"' << item.label << "\",\"" << item.ac_source << "\",\""
          << item.source_reference << "\"," << item.system.ac.buses.size() << ','
          << item.system.ac.branches.size() << ',' << item.system.dc.buses.size() << ','
          << item.system.dc.branches.size() << ',' << item.import_warnings << ','
          << item.import_skipped << ',' << converter.index << ','
          << converter.bus_ac << ',' << converter.bus_dc << ','
          << converter_mode_name(converter.control_mode) << ',' << converter.p_set_mw
          << ',' << converter.q_set_mvar << ',' << converter.v_dc_set_pu << ','
          << converter.eta << ',' << converter.loss_percent << ',' << converter.loss_mw
          << ",\"" << dc_branches.str() << "\"\n";
    }
  }
  return out.str();
}

std::string public_manifest_latex(const std::vector<PublicStudySystem>& systems) {
  std::ostringstream out;
  out << "\\begin{tabularx}{\\linewidth}{@{}lXrrrr@{}}\n"
         "\\toprule\n"
         "Public variant & AC source and deterministic hybridization & AC buses & DC buses & DC lines & VSCs \\\\\n"
         "\\midrule\n";
  for (const auto& item : systems) {
    out << item.label << " & " << item.ac_source
        << "; deterministic two-terminal overlay & "
        << item.system.ac.buses.size() << " & " << item.system.dc.buses.size() << " & "
        << item.system.dc.branches.size() << " & " << item.system.vsc_converters.size()
        << " \\\\\n";
  }
  out << "\\bottomrule\n\\end{tabularx}\n";
  return out.str();
}

std::string public_agent_campaign_csv(
    const std::vector<PublicStudySystem>& systems) {
  const std::vector<std::string> actions = {
      "scale_loads_10pct", "no_op", "hallucinate_vsc_ac_terminal",
      "hallucinate_vsc_dc_terminal", "remove_dc_voltage_support",
      "invalid_vsc_efficiency"};
  std::ostringstream out;
  out << "system,ac_buses,action,expected_admissible,accepted,committed,"
         "analysis_ran,analysis_converged,attributed_buses,recovery_accepted,"
         "recovery_converged,trajectory_sound,reason\n";
  for (const auto& item : systems) {
    for (const auto& action : actions) {
      const auto edit = sppt::agent_edit_from_action_id(action);
      const bool solve_in_scope = item.system.ac.buses.size() <= 823;
      const auto trajectory = sppt::run_agent_loop(
          item.system, {edit, sppt::agent_edit_from_action_id("no_op")},
          solve_in_scope);
      if (trajectory.steps.size() != 2)
        throw std::runtime_error("agent campaign returned an incomplete trajectory");
      const auto& step = trajectory.steps.front();
      const auto& recovery = trajectory.steps.back();
      out << item.label << ',' << item.system.ac.buses.size() << ',' << action
          << ',' << edit.expected_admissible << ',' << step.accepted << ','
          << step.applied << ',' << step.analysis_ran << ','
          << step.analysis_converged << ',' << step.attributed_buses << ','
          << recovery.accepted << ',' << recovery.analysis_converged << ','
          << trajectory.sound << ",\"" << step.reason << "\"\n";
    }
  }
  return out.str();
}

}  // namespace

int main(int argc, char** argv) {
  const fs::path root = HACDCPF_PROJECT_ROOT;
  const fs::path output = argc > 1 ? fs::path(argv[1]) : root / "docs" / "latex";
  sppt::FaultCampaignOptions options;
  if (argc > 2) options.repetitions = std::stoi(argv[2]);
  if (argc > 3) options.seed = std::stoull(argv[3]);

  const std::vector<PublicStudySystem> public_systems = {
      load_public_gridlabd_variant(root, "GC-12.47-1.glm", "PNNL GridLAB-D taxonomy feeder corpus", 32),
      load_public_gridlabd_variant(root, "R1-12.47-3.glm", "PNNL GridLAB-D taxonomy feeder corpus", 78),
      load_public_gridlabd_variant(root, "R3-12.47-2.glm", "PNNL GridLAB-D taxonomy feeder corpus", 330),
      load_public_gridlabd_variant(root, "R4-12.47-2.glm", "PNNL GridLAB-D taxonomy feeder corpus", 823),
      load_public_gridlabd_variant(root, "R1-12.47-2.glm", "PNNL GridLAB-D taxonomy feeder corpus", 1105),
      load_public_gridlabd_variant(root, "R2-35.00-1.glm", "PNNL GridLAB-D taxonomy feeder corpus", 1976),
      load_public_gridlabd_variant(root, "R3-12.47-3.glm", "PNNL GridLAB-D taxonomy feeder corpus", 6986),
  };

  std::vector<std::pair<std::string, HybridPowerSystem>> systems;
  systems.reserve(public_systems.size());
  for (const auto& item : public_systems) {
    if (item.system.ac.buses.size() != item.expected_ac_buses) {
      std::cerr << item.label << " expected " << item.expected_ac_buses
                << " public AC buses but loaded " << item.system.ac.buses.size()
                << "; refusing mislabeled campaign input\n";
      return 3;
    }
    systems.emplace_back(item.label, item.system);
  }

  const sppt::FaultCampaign campaign = sppt::run_fault_campaign(systems, options);
  std::error_code error;
  fs::create_directories(output, error);
  if (error) {
    std::cerr << "Cannot create output directory: " << error.message() << '\n';
    return 2;
  }
  std::ofstream(output / "sppt_fault_campaign_samples.csv")
      << campaign.samples_csv();
  std::ofstream(output / "sppt_fault_campaign_summary.csv")
      << campaign.summary_csv();
  std::ofstream(output / "sppt_fault_campaign.tex") << campaign.to_latex();
  std::ofstream(output / "sppt_public_benchmark_manifest.csv")
      << public_manifest_csv(public_systems);
  std::ofstream(output / "sppt_public_benchmark_manifest.tex")
      << public_manifest_latex(public_systems);
  std::ofstream(output / "sppt_agent_public_campaign.csv")
      << public_agent_campaign_csv(public_systems);

  std::cout << "SPPT campaign: " << campaign.rows.size() << " samples, seed "
            << options.seed << ", repetitions " << options.repetitions << '\n'
            << "Outputs: " << output << '\n';
  return 0;
}
