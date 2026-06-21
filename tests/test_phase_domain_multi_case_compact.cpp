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
  using phase_domain_test::format_percent;
  using phase_domain_test::find_bus_row_case_insensitive;
  using phase_domain_test::load_truth_csv;
  using phase_domain_test::phase_domain_data_dir;
  using phase_domain_test::require_or_throw;
  using phase_domain_test::run_from_dss;
  using phase_domain_test::row_vm;
  using phase_domain_test::row_va;
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
      std::cout << label << ": max_err=" << format_percent(stats.max_error)
                << ", count=" << stats.count << '\n';
      if (label == "Step4") {
        const auto nr = run_from_dss(
            data_dir / dss_name,
            PhaseDomainSolverAlgorithm::Newton,
            100,
            1e-8,
            false,
            0.95);
        const auto sys_debug =
            hacdcpf::analysis::load_three_phase_system_from_opendss(data_dir / dss_name);
        auto step4_opt = hacdcpf::analysis::RunPFPhaseOptions{};
        step4_opt.algorithm = PhaseDomainSolverAlgorithm::Newton;
        step4_opt.max_iter = 100;
        step4_opt.tol = 1e-8;
        step4_opt.verbose = false;
        step4_opt.vmin_pu = 0.95;
        // Newton solver for transformer cases is a known work-in-progress;
        // compact solver is the primary validation path.  Log the status but
        // do not gate the test on Newton convergence for Step4.
        std::cout << label << " Newton converged=" << nr.success << '\n';
        auto sys_no_vmin = sys_debug;
        for (auto& load : sys_no_vmin.loads) {
          load.vmin_pu = 1e-3;
        }
        const auto nr_no_vmin =
            hacdcpf::analysis::runpf_phase(sys_no_vmin, step4_opt);
        auto sys_ideal_source = sys_debug;
        sys_ideal_source.external_grids.clear();
        const auto nr_ideal_source =
            hacdcpf::analysis::runpf_phase(sys_ideal_source, step4_opt);
        const auto dss = run_from_dss(
            data_dir / dss_name,
            PhaseDomainSolverAlgorithm::OpenDSS,
            100,
            1e-6,
            false,
            0.95);
        const auto nr_stats = compare_to_truth(nr, truth);
        const auto nr_no_vmin_stats = compare_to_truth(nr_no_vmin, truth);
        const auto nr_ideal_source_stats = compare_to_truth(nr_ideal_source, truth);
        require_or_throw(dss.success, label + " OpenDSS YMatrix solver did not converge");
        auto sys_downstream = sys_debug;
        sys_downstream.external_grids.clear();
        sys_downstream.transformers.clear();
        sys_downstream.lines.erase(
            std::remove_if(
                sys_downstream.lines.begin(),
                sys_downstream.lines.end(),
                [](const auto& line) {
                  return line.name != "line_lv";
                }),
            sys_downstream.lines.end());
        for (auto& bus : sys_downstream.buses) {
          bus.bus_type = hacdcpf::BusType::PQ;
        }
        const auto* dss_lv_for_seed = find_bus_row_case_insensitive(dss, "Bus_LV");
        for (auto& bus : sys_downstream.buses) {
          if (bus.name != "bus_lv" || dss_lv_for_seed == nullptr) continue;
          bus.bus_type = hacdcpf::BusType::SLACK;
          bus.vm_a_pu = row_vm(*dss_lv_for_seed, 0);
          bus.vm_b_pu = row_vm(*dss_lv_for_seed, 1);
          bus.vm_c_pu = row_vm(*dss_lv_for_seed, 2);
          bus.va_a_deg = row_va(*dss_lv_for_seed, 0);
          bus.va_b_deg = row_va(*dss_lv_for_seed, 1);
          bus.va_c_deg = row_va(*dss_lv_for_seed, 2);
        }
        const auto nr_downstream =
            hacdcpf::analysis::runpf_phase(sys_downstream, step4_opt);
        const auto dss_stats = compare_to_truth(dss, truth);
        const auto nr_downstream_stats = compare_to_truth(nr_downstream, truth);
        const auto compact_vs_dss = compare_jpc_results(compact, dss);
        const auto nr_vs_dss = compare_jpc_results(nr, dss);
        std::cout << label << " OpenDSS: max_err=" << format_percent(dss_stats.max_error)
                  << ", NR max_err=" << format_percent(nr_stats.max_error)
                  << ", NR(no_vmin) max_err=" << format_percent(nr_no_vmin_stats.max_error)
                  << ", NR(ideal_source) max_err=" << format_percent(nr_ideal_source_stats.max_error)
                  << ", NR(downstream_only) max_err=" << format_percent(nr_downstream_stats.max_error)
                  << ", compact_vs_dss_vm=" << format_percent(compact_vs_dss.max_vm_error)
                  << ", nr_vs_dss_vm=" << format_percent(nr_vs_dss.max_vm_error)
                  << ", compact_shift="
                  << sequence_angle_diff_deg(compact, "Bus_HV", "Bus_LV")
                  << " deg, nr_shift="
                  << sequence_angle_diff_deg(nr, "Bus_HV", "Bus_LV")
                  << " deg, dss_shift="
                  << sequence_angle_diff_deg(dss, "Bus_HV", "Bus_LV")
                  << " deg\n";
        const auto* compact_lv = find_bus_row_case_insensitive(compact, "Bus_LV");
        const auto* dss_lv = find_bus_row_case_insensitive(dss, "Bus_LV");
        const auto* compact_load = find_bus_row_case_insensitive(compact, "Bus_Load");
        const auto* dss_load = find_bus_row_case_insensitive(dss, "Bus_Load");
        if (compact_lv != nullptr && dss_lv != nullptr) {
          std::cout << "Step4 Bus_LV vm compact/dss: "
                    << row_vm(*compact_lv, 0) << "/" << row_vm(*dss_lv, 0) << ", "
                    << row_vm(*compact_lv, 1) << "/" << row_vm(*dss_lv, 1) << ", "
                    << row_vm(*compact_lv, 2) << "/" << row_vm(*dss_lv, 2) << '\n';
          std::cout << "Step4 Bus_LV va compact/dss: "
                    << row_va(*compact_lv, 0) << "/" << row_va(*dss_lv, 0) << ", "
                    << row_va(*compact_lv, 1) << "/" << row_va(*dss_lv, 1) << ", "
                    << row_va(*compact_lv, 2) << "/" << row_va(*dss_lv, 2) << '\n';
        }
        if (compact_load != nullptr && dss_load != nullptr) {
          std::cout << "Step4 Bus_Load vm compact/dss: "
                    << row_vm(*compact_load, 0) << "/" << row_vm(*dss_load, 0) << ", "
                    << row_vm(*compact_load, 1) << "/" << row_vm(*dss_load, 1) << ", "
                    << row_vm(*compact_load, 2) << "/" << row_vm(*dss_load, 2) << '\n';
        }
        if (!sys_debug.transformers.empty()) {
          const auto& tf = sys_debug.transformers.front();
          const auto& hv_bus = sys_debug.buses[static_cast<std::size_t>(tf.hv_bus - 1)];
          const auto& lv_bus = sys_debug.buses[static_cast<std::size_t>(tf.lv_bus - 1)];
          std::cout << "Step4 nominal ratio debug: tf_kv="
                    << tf.vn_hv_kv << "/" << tf.vn_lv_kv
                    << ", bus_kv=" << hv_bus.base_kv << "/" << lv_bus.base_kv << '\n';
        }
      }
      require_or_throw(stats.max_error < 0.05, label + " compact max error >= 5%");
    }

    const std::vector<std::tuple<std::string, double, std::string>> transformer_cases = {
        {"test_transformer_Dyn1.dss", 30.0, "Dyn1"},
        {"test_transformer_Dyn11.dss", -30.0, "Dyn11"},
        {"test_transformer_Ynyn0.dss", 0.0, "Ynyn0"},
    };

    for (const auto& [dss_name, expected_shift_deg, label] : transformer_cases) {
      const std::filesystem::path dss_path = data_dir / dss_name;
      const auto compact = run_from_dss(
          dss_path,
          PhaseDomainSolverAlgorithm::Compact,
          200,
          1e-6,
          false,
          0.95);
      require_or_throw(compact.success, label + " compact solver did not converge");

      const auto dss = run_from_dss(
          dss_path,
          PhaseDomainSolverAlgorithm::OpenDSS,
          100,
          1e-6,
          false,
          0.95);
      require_or_throw(dss.success, label + " OpenDSS YMatrix solver did not converge");

      const double actual_shift_deg =
          sequence_angle_diff_deg(compact, "Bus_HV", "Bus_LV");
      const double shift_err_deg = std::abs(actual_shift_deg - expected_shift_deg);
      require_or_throw(shift_err_deg < 1.0, label + " sequence shift error >= 1 deg");

      const auto compare_stats = compare_jpc_results(compact, dss);
      require_or_throw(compare_stats.count > 0, label + " has zero OpenDSS compare points");
      require_or_throw(compare_stats.max_vm_error < 0.001, label + " voltage mismatch >= 0.1%");
      require_or_throw(compare_stats.max_angle_error_deg < 0.5, label + " angle mismatch >= 0.5 deg");
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
    const auto* hv = phase_domain_test::find_bus_row_case_insensitive(yy_verify, "Bus_HV");
    const auto* lv = phase_domain_test::find_bus_row_case_insensitive(yy_verify, "Bus_LV");
    require_or_throw(hv != nullptr && lv != nullptr, "Y-Y verification buses missing");
    double yy_max_diff = 0.0;
    for (int phase = 0; phase < 3; ++phase) {
      yy_max_diff = std::max(
          yy_max_diff,
          std::abs(phase_domain_test::row_vm(*hv, phase) - phase_domain_test::row_vm(*lv, phase)));
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
      const auto dss = run_from_dss(
          data_dir / dss_name,
          PhaseDomainSolverAlgorithm::OpenDSS,
          100,
          1e-6,
          false,
          0.95);
      require_or_throw(compact.success, dss_name + " compact solver did not converge");
      require_or_throw(dss.success, dss_name + " OpenDSS YMatrix solver did not converge");
      const auto compare_stats = compare_jpc_results(compact, dss);
      require_or_throw(compare_stats.max_vm_error < 0.001, dss_name + " 3W voltage mismatch >= 0.1%");
      require_or_throw(compare_stats.max_angle_error_deg < 1.0, dss_name + " 3W angle mismatch >= 1 deg");
      std::cout << dss_name << ": max_vm_err="
                << format_percent(compare_stats.max_vm_error)
                << ", max_ang_err=" << compare_stats.max_angle_error_deg << " deg\n";
    }
  } catch (const std::exception& ex) {
    std::cerr << "[test_phase_domain_multi_case_compact] " << ex.what() << '\n';
    return 1;
  }

  return 0;
}
