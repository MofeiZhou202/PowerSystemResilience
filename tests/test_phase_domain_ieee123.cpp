#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "hacdcpf/analysis/distribution_power_flow.hpp"
#include "phase_domain_test_utils.hpp"

int main() {
  using hacdcpf::analysis::PhaseDomainSolverAlgorithm;
  using phase_domain_test::compare_jpc_results;
  using phase_domain_test::compare_jpc_results_detailed;
  using phase_domain_test::find_bus_row_case_insensitive;
  using phase_domain_test::format_percent;
  using phase_domain_test::phase_domain_data_dir;
  using phase_domain_test::require_or_throw;
  using phase_domain_test::run_from_dss;
  using phase_domain_test::run_snapshot_from_dss;

  try {
    const std::filesystem::path ieee123_dss =
        phase_domain_data_dir() / "IEEE123_Fixed" / "IEEE123Master_Fixed.dss";
    const auto compact = run_from_dss(
        ieee123_dss,
        PhaseDomainSolverAlgorithm::Compact,
        200,
        1e-6,
        false,
        0.95);
    require_or_throw(compact.success, "IEEE123 compact solver did not converge");
    const auto dss_snapshot = run_snapshot_from_dss(ieee123_dss);
    require_or_throw(
        dss_snapshot.success,
        "IEEE123 OpenDSS official snapshot did not converge");
    const auto compact_vs_dss = compare_jpc_results_detailed(compact, dss_snapshot);
    require_or_throw(
        compact_vs_dss.max_vm_error < 0.001,
        "IEEE123 compact max voltage error vs OpenDSS >= 0.1%");
    require_or_throw(
        compact_vs_dss.max_angle_error_deg < 0.1,
        "IEEE123 compact max angle error vs OpenDSS >= 0.1 deg");
    require_or_throw(
        compact_vs_dss.count >= 270,
        "IEEE123 compact OpenDSS compare point count < 270");

    const auto ieee123_sys =
        hacdcpf::analysis::load_three_phase_system_from_opendss(ieee123_dss);
    hacdcpf::analysis::ThreePhaseJPCPhase ybus_shell =
        hacdcpf::analysis::case2jpc_phase(ieee123_sys);
    hacdcpf::analysis::makeYbus_phase(ybus_shell, ieee123_sys, true);
    require_or_throw(
        ybus_shell.ybus_abc_dim == static_cast<int>(ieee123_sys.buses.size()) * 3,
        "IEEE123 full Ybus dimension mismatch");
    require_or_throw(
        !ybus_shell.ybus_abc_entries.empty(),
        "IEEE123 full Ybus entries are empty");

    const std::filesystem::path step4_dss =
        phase_domain_data_dir() / "test_step4_transformer.dss";
    const auto step4_sys =
        hacdcpf::analysis::load_three_phase_system_from_opendss(step4_dss, 1.0);
    const auto step4_jpc = hacdcpf::analysis::case2jpc_phase(step4_sys);
    const auto* bus_hv = find_bus_row_case_insensitive(step4_jpc, "Bus_HV");
    const auto* bus_lv = find_bus_row_case_insensitive(step4_jpc, "Bus_LV");
    const auto* bus_load = find_bus_row_case_insensitive(step4_jpc, "Bus_Load");
    require_or_throw(bus_hv != nullptr, "Step4 Bus_HV row missing from case2jpc_phase");
    require_or_throw(bus_lv != nullptr, "Step4 Bus_LV row missing from case2jpc_phase");
    require_or_throw(bus_load != nullptr, "Step4 Bus_Load row missing from case2jpc_phase");
    require_or_throw(
        std::abs(bus_hv->vm_pu[0] - 1.0) < 1e-12 &&
            std::abs(bus_hv->vm_pu[1] - 1.0) < 1e-12 &&
            std::abs(bus_hv->vm_pu[2] - 1.0) < 1e-12,
        "Step4 Bus_HV should stay on Julia flat-start shell");
    require_or_throw(
        std::abs(bus_lv->vm_pu[0] - 1.0) < 1e-12 &&
            std::abs(bus_lv->vm_pu[1] - 1.0) < 1e-12 &&
            std::abs(bus_lv->vm_pu[2] - 1.0) < 1e-12,
        "Step4 Bus_LV should stay on Julia flat-start shell");
    require_or_throw(
        std::abs(bus_load->vm_pu[0] - 1.0) < 1e-12 &&
            std::abs(bus_load->vm_pu[1] - 1.0) < 1e-12 &&
            std::abs(bus_load->vm_pu[2] - 1.0) < 1e-12,
        "Step4 Bus_Load should stay on Julia flat-start shell");
    bool saw_step4_transformer = false;
    for (const auto& branch : step4_jpc.branch_abc) {
      if (branch.vector_group != "DYN1") continue;
      saw_step4_transformer = true;
      require_or_throw(
          std::abs(branch.shift_deg[0] + 30.0) < 1e-12 &&
              std::abs(branch.shift_deg[1] + 30.0) < 1e-12 &&
              std::abs(branch.shift_deg[2] + 30.0) < 1e-12,
          "Step4 transformer shift should follow Julia vector_group clock semantics");
    }
    require_or_throw(
        saw_step4_transformer,
        "Step4 transformer branch missing from case2jpc_phase shell");

    const std::filesystem::path step8_dss =
        phase_domain_data_dir() / "test_step8_combined.dss";
    const auto compact_step8 = run_from_dss(
        step8_dss,
        PhaseDomainSolverAlgorithm::Compact,
        200,
        1e-6,
        false,
        0.95);
    require_or_throw(compact_step8.success, "Step8 compact solver did not converge");

    const auto fixed_step8 = run_from_dss(
        step8_dss,
        PhaseDomainSolverAlgorithm::FixedPoint,
        200,
        1e-6,
        false,
        0.95);
    require_or_throw(fixed_step8.success, "Step8 full fixed-point solver did not converge");
    const auto fixed_vs_compact = compare_jpc_results(compact_step8, fixed_step8);
    require_or_throw(
        fixed_vs_compact.max_vm_error < 0.02,
        "Step8 fixed-point vs compact max voltage error >= 2%");

    const auto dss_step8 = run_from_dss(
        step8_dss,
        PhaseDomainSolverAlgorithm::OpenDSS,
        100,
        1e-6,
        false,
        0.95);
    require_or_throw(dss_step8.success, "Step8 OpenDSS YMatrix solver did not converge");
    const auto dss_vs_compact = compare_jpc_results(compact_step8, dss_step8);
    require_or_throw(
        dss_vs_compact.max_vm_error < 0.02,
        "Step8 OpenDSS YMatrix vs compact max voltage error >= 2%");

    std::cout
        << "IEEE123 compact vs OpenDSS official: max_vm_err="
        << format_percent(compact_vs_dss.max_vm_error)
        << ", avg_vm_err=" << format_percent(compact_vs_dss.avg_vm_error)
        << ", max_angle_err=" << compact_vs_dss.max_angle_error_deg
        << " deg, count=" << compact_vs_dss.count << '\n'
        << "Step8 fixed-point vs compact: max_vm_err="
        << format_percent(fixed_vs_compact.max_vm_error)
        << ", compare_points=" << fixed_vs_compact.count << '\n'
        << "Step8 OpenDSS YMatrix vs compact: max_vm_err="
        << format_percent(dss_vs_compact.max_vm_error)
        << ", compare_points=" << dss_vs_compact.count << '\n';
  } catch (const std::exception& ex) {
    std::cerr << "[test_phase_domain_ieee123] " << ex.what() << '\n';
    return 1;
  }

  return 0;
}
