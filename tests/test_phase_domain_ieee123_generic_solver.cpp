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
  using phase_domain_test::compare_jpc_results_detailed;
  using phase_domain_test::find_bus_row_case_insensitive;
  using phase_domain_test::format_percent;
  using phase_domain_test::phase_domain_data_dir;
  using phase_domain_test::require_or_throw;
  using phase_domain_test::row_vm;
  using phase_domain_test::row_va;
  using phase_domain_test::run_from_dss;
  using phase_domain_test::run_snapshot_from_dss;
  using phase_domain_test::wrap_angle_diff_deg;

  try {
    const std::filesystem::path dss_file =
        phase_domain_data_dir() / "IEEE123_Fixed" / "IEEE123Master_Fixed.dss";
    const auto compact = run_from_dss(
        dss_file,
        PhaseDomainSolverAlgorithm::Compact,
        200,
        1e-6,
        false,
        0.95);
    require_or_throw(compact.success, "IEEE123 compact solver did not converge");
    const auto dss = run_snapshot_from_dss(dss_file);
    require_or_throw(dss.success, "IEEE123 OpenDSS official snapshot did not converge");

    const auto stats = compare_jpc_results_detailed(compact, dss);
    if (stats.max_vm_error >= 0.001 || stats.max_angle_error_deg >= 0.1 ||
        stats.count < 270) {
      std::vector<std::tuple<std::string, int, double, double, double, double, double, double>>
          top_errors;
      for (const auto& [bus_name, _] : compact.bus_name_to_id) {
        const auto* compact_row = find_bus_row_case_insensitive(compact, bus_name);
        const auto* dss_row = find_bus_row_case_insensitive(dss, bus_name);
        if (compact_row == nullptr || dss_row == nullptr) continue;
        for (int phase = 0; phase < 3; ++phase) {
          if (!compact_row->has_phase[static_cast<std::size_t>(phase)] ||
              !dss_row->has_phase[static_cast<std::size_t>(phase)]) {
            continue;
          }
          const double compact_vm = row_vm(*compact_row, phase);
          const double dss_vm = row_vm(*dss_row, phase);
          const double compact_va = row_va(*compact_row, phase);
          const double dss_va = row_va(*dss_row, phase);
          const double vm_err = std::abs(compact_vm - dss_vm);
          const double angle_err = std::abs(wrap_angle_diff_deg(compact_va, dss_va));
          top_errors.emplace_back(
              bus_name,
              phase + 1,
              compact_vm,
              dss_vm,
              compact_va,
              dss_va,
              vm_err,
              angle_err);
        }
      }
      std::sort(
          top_errors.begin(),
          top_errors.end(),
          [](const auto& lhs, const auto& rhs) {
            return std::get<6>(lhs) > std::get<6>(rhs);
          });
      std::cout << "IEEE123 compact vs OpenDSS failed: max_vm_err="
                << format_percent(stats.max_vm_error)
                << ", avg_vm_err=" << format_percent(stats.avg_vm_error)
                << ", max_angle_err=" << stats.max_angle_error_deg << " deg"
                << ", count=" << stats.count << '\n';
      const std::size_t limit = std::min<std::size_t>(20, top_errors.size());
      for (std::size_t idx = 0; idx < limit; ++idx) {
        const auto& [bus, phase, compact_vm, dss_vm, compact_va, dss_va, vm_err, angle_err] =
            top_errors[idx];
        std::cout << "  " << bus << "." << phase
                  << " compact_vm=" << compact_vm
                  << " dss_vm=" << dss_vm
                  << " compact_va=" << compact_va
                  << " dss_va=" << dss_va
                  << " vm_diff=" << vm_err
                  << " angle_diff_deg=" << angle_err << '\n';
      }
    }

    require_or_throw(stats.max_vm_error < 0.001, "IEEE123 compact max voltage error >= 0.1%");
    require_or_throw(
        stats.max_angle_error_deg < 0.1,
        "IEEE123 compact max angle error >= 0.1 deg");
    require_or_throw(
        stats.count >= 270,
        "IEEE123 compact OpenDSS compare point count < 270");

    std::cout << "IEEE123 compact vs OpenDSS official: max_vm_err="
              << format_percent(stats.max_vm_error)
              << ", avg_vm_err=" << format_percent(stats.avg_vm_error)
              << ", max_angle_err=" << stats.max_angle_error_deg << " deg"
              << ", count=" << stats.count << '\n';
  } catch (const std::exception& ex) {
    std::cerr << "[test_phase_domain_ieee123_generic_solver] " << ex.what() << '\n';
    return 1;
  }

  return 0;
}
