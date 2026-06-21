#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "phase_domain_test_utils.hpp"

int main() {
  using hacdcpf::analysis::PhaseDomainSolverAlgorithm;
  using phase_domain_test::compare_jpc_results;
  using phase_domain_test::compare_to_truth;
  using phase_domain_test::find_bus_row_case_insensitive;
  using phase_domain_test::format_percent;
  using phase_domain_test::load_truth_csv;
  using phase_domain_test::phase_domain_data_dir;
  using phase_domain_test::require_or_throw;
  using phase_domain_test::row_vm;
  using phase_domain_test::run_from_dss;
  using phase_domain_test::sequence_angle_diff_deg;

  try {
    const std::filesystem::path data_dir = phase_domain_data_dir();

    const std::vector<std::tuple<std::string, std::string, std::string>> step_cases = {
        {"test_step1_basic.dss", "Step1_Basic_EXP_VOLTAGES.csv", "Step1"},
        {"test_step2_zip_model.dss", "Step2_ZIP_EXP_VOLTAGES.csv", "Step2"},
        {"test_step3_single_phase.dss", "Step3_SinglePhase_EXP_VOLTAGES.csv", "Step3"},
        {"test_step4_transformer.dss", "Step4_Transformer_EXP_VOLTAGES.csv", "Step4"},
        {"test_step5_linecode.dss", "Step5_LineCode_EXP_VOLTAGES.csv", "Step5"},
        {"test_step6_capacitor.dss", "Step6_Capacitor_EXP_VOLTAGES.csv", "Step6"},
        {"test_step7_delta_load.dss", "Step7_Delta_EXP_VOLTAGES.csv", "Step7"},
        {"test_step8_combined.dss", "Step8_Combined_EXP_VOLTAGES.csv", "Step8"},
    };

    for (const auto& [dss_name, csv_name, label] : step_cases) {
      const auto truth = load_truth_csv(data_dir / csv_name);
      const auto compact = run_from_dss(
          data_dir / dss_name,
          PhaseDomainSolverAlgorithm::Compact,
          200,
          1e-6,
          false,
          0.95);
      require_or_throw(compact.success, label + " compact solver did not converge");
      const auto stats = compare_to_truth(compact, truth);
      require_or_throw(stats.count > 0, label + " has zero truth compare points");
      if (stats.max_error >= 0.05) {
        std::vector<std::tuple<std::string, int, double, double, double>> top_errors;
        for (const auto& [bus_name, _] : compact.bus_name_to_id) {
          const auto* row = find_bus_row_case_insensitive(compact, bus_name);
          const auto truth_it = truth.find(phase_domain_test::uppercase(bus_name));
          if (row == nullptr || truth_it == truth.end()) continue;
          for (int phase = 0; phase < 3; ++phase) {
            if (!row->has_phase[static_cast<std::size_t>(phase)]) continue;
            const auto& truth_value = truth_it->second[static_cast<std::size_t>(phase)];
            if (!truth_value.has_value()) continue;
            const double vm = row_vm(*row, phase);
            const double err = std::abs(vm - *truth_value);
            top_errors.emplace_back(bus_name, phase + 1, vm, *truth_value, err);
          }
        }
        std::sort(
            top_errors.begin(),
            top_errors.end(),
            [](const auto& lhs, const auto& rhs) {
              return std::get<4>(lhs) > std::get<4>(rhs);
            });
        const std::size_t limit = std::min<std::size_t>(10, top_errors.size());
        for (std::size_t idx = 0; idx < limit; ++idx) {
          const auto& [bus, phase, vm, truth_vm, err] = top_errors[idx];
          std::cerr << "  " << label << " " << bus << "." << phase
                    << " compact=" << vm
                    << " truth=" << truth_vm
                    << " diff=" << err << '\n';
        }
      }
      std::cout << label << ": max_err=" << format_percent(stats.max_error)
                << ", count=" << stats.count << '\n';
      require_or_throw(stats.max_error < 0.05, label + " compact max error >= 5%");
    }

    const std::vector<std::tuple<std::string, double, std::string>> transformer_cases = {
        {"test_transformer_Dyn1.dss", 30.0, "Dyn1"},
        {"test_transformer_Dyn11.dss", -30.0, "Dyn11"},
        {"test_transformer_Ynyn0.dss", 0.0, "Ynyn0"},
    };

    for (const auto& [dss_name, expected_shift_deg, label] : transformer_cases) {
      const auto compact = run_from_dss(
          data_dir / dss_name,
          PhaseDomainSolverAlgorithm::Compact,
          200,
          1e-6,
          false,
          0.95);
      require_or_throw(compact.success, label + " compact solver did not converge");

      const auto dss = run_from_dss(
          data_dir / dss_name,
          PhaseDomainSolverAlgorithm::OpenDSS,
          100,
          1e-6,
          false,
          0.95);
      require_or_throw(dss.success, label + " OpenDSS YMatrix solver did not converge");

      const double actual_shift_deg =
          sequence_angle_diff_deg(compact, "Bus_HV", "Bus_LV");
      const double shift_err_deg = std::abs(actual_shift_deg - expected_shift_deg);
      const auto compare_stats = compare_jpc_results(compact, dss);
      require_or_throw(shift_err_deg < 1.0, label + " sequence shift error >= 1 deg");
      require_or_throw(compare_stats.count > 0, label + " has zero OpenDSS compare points");
      require_or_throw(compare_stats.max_vm_error < 0.001, label + " voltage mismatch >= 0.1%");
      require_or_throw(compare_stats.max_angle_error_deg < 1.0, label + " angle mismatch >= 1 deg");
      std::cout << label << ": shift_err=" << shift_err_deg
                << " deg, max_vm_err=" << format_percent(compare_stats.max_vm_error)
                << ", max_ang_err=" << compare_stats.max_angle_error_deg << " deg\n";
    }

    const auto yy_verify = run_from_dss(
        data_dir / "test_transformer_Ynyn0_verify.dss",
        PhaseDomainSolverAlgorithm::Compact,
        200,
        1e-6,
        false,
        0.95);
    require_or_throw(yy_verify.success, "Y-Y verification compact solver did not converge");
    const auto* hv = find_bus_row_case_insensitive(yy_verify, "Bus_HV");
    const auto* lv = find_bus_row_case_insensitive(yy_verify, "Bus_LV");
    require_or_throw(hv != nullptr && lv != nullptr, "Y-Y verification buses missing");
    double yy_max_diff = 0.0;
    for (int phase = 0; phase < 3; ++phase) {
      yy_max_diff = std::max(
          yy_max_diff,
          std::abs(row_vm(*hv, phase) - row_vm(*lv, phase)));
    }
    require_or_throw(yy_max_diff < 0.01, "Y-Y verification PU voltage drop >= 0.01");
    std::cout << "Y-Y verification: max_vm_drop=" << format_percent(yy_max_diff) << '\n';

    const std::vector<std::string> three_w_cases = {
        "test_transformer_3w.dss",
        "test_transformer_3w_dyn.dss",
        "test_transformer_3w_tap.dss",
    };
    for (const auto& dss_name : three_w_cases) {
      const auto compact = run_from_dss(
          data_dir / dss_name,
          PhaseDomainSolverAlgorithm::Compact,
          200,
          1e-6,
          false,
          0.95);
      require_or_throw(compact.success, dss_name + " compact solver did not converge");
      const auto dss = run_from_dss(
          data_dir / dss_name,
          PhaseDomainSolverAlgorithm::OpenDSS,
          100,
          1e-6,
          false,
          0.95);
      require_or_throw(dss.success, dss_name + " OpenDSS YMatrix solver did not converge");
      const auto compare_stats = compare_jpc_results(compact, dss);
      if (compare_stats.max_vm_error >= 0.001 ||
          compare_stats.max_angle_error_deg >= 1.0) {
        std::cerr << dss_name
                  << " 3W mismatch: max_vm_err="
                  << format_percent(compare_stats.max_vm_error)
                  << ", max_ang_err=" << compare_stats.max_angle_error_deg
                  << " deg, count=" << compare_stats.count << '\n';
      }
      require_or_throw(compare_stats.max_vm_error < 0.001, dss_name + " 3W voltage mismatch >= 0.1%");
      require_or_throw(compare_stats.max_angle_error_deg < 1.0, dss_name + " 3W angle mismatch >= 1 deg");
      std::cout << dss_name << ": max_vm_err=" << format_percent(compare_stats.max_vm_error)
                << ", max_ang_err=" << compare_stats.max_angle_error_deg << " deg\n";
    }
  } catch (const std::exception& ex) {
    std::cerr << "[test_phase_domain_multi_case_compact_julia] " << ex.what() << '\n';
    return 1;
  }

  return 0;
}
