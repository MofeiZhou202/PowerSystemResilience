#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "phase_domain_test_utils.hpp"

int main() {
  using hacdcpf::analysis::PhaseDomainSolverAlgorithm;
  using phase_domain_test::compare_jpc_results_detailed;
  using phase_domain_test::compare_jpc_points;
  using phase_domain_test::format_percent;
  using phase_domain_test::project_root;
  using phase_domain_test::require_or_throw;
  using phase_domain_test::run_from_dss;
  using phase_domain_test::run_snapshot_from_dss;

  try {
    const std::filesystem::path ieee13_dss =
        project_root() / "external_data" / "opendss_ieee_pes" /
        "opendss_reference" / "13_node" / "official_full" /
        "IEEE13Nodeckt.dss";
    require_or_throw(
        std::filesystem::exists(ieee13_dss),
        "IEEE13 OpenDSS fixture is missing");

    const auto sys =
        hacdcpf::analysis::load_three_phase_system_from_opendss(ieee13_dss);
    require_or_throw(sys.buses.size() >= 10, "IEEE13 phase loader returned too few buses");
    require_or_throw(!sys.lines.empty(), "IEEE13 phase loader returned no lines");
    require_or_throw(!sys.loads.empty(), "IEEE13 phase loader returned no loads");
    require_or_throw(
        !sys.transformers.empty(),
        "IEEE13 phase loader returned no transformers/regulators");

    const auto compact = run_from_dss(
        ieee13_dss,
        PhaseDomainSolverAlgorithm::Compact,
        200,
        1e-6,
        false,
        0.95);
    require_or_throw(compact.success, "IEEE13 compact solver did not converge");

    const auto dss_snapshot = run_snapshot_from_dss(ieee13_dss);
    require_or_throw(
        dss_snapshot.success,
        "IEEE13 OpenDSS official snapshot did not converge");

	    const auto stats = compare_jpc_results_detailed(compact, dss_snapshot);
	    if (stats.max_vm_error >= 0.002 || stats.max_angle_error_deg >= 0.2) {
	      std::cerr << "IEEE13 worst compact/OpenDSS mismatch: bus="
	                << stats.worst_bus << "." << stats.worst_phase
	                << " vm_ours=" << stats.worst_vm_lhs
	                << " vm_dss=" << stats.worst_vm_rhs
	                << " va_ours=" << stats.worst_angle_lhs_deg
	                << " va_dss=" << stats.worst_angle_rhs_deg
	                << " max_vm_err=" << stats.max_vm_error
	                << " max_angle_err_deg=" << stats.max_angle_error_deg
	                << " avg_vm_err=" << stats.avg_vm_error
	                << " count=" << stats.count << '\n';
	      const auto points = compare_jpc_points(compact, dss_snapshot);
	      const std::size_t shown = std::min<std::size_t>(10, points.size());
	      for (std::size_t idx = 0; idx < shown; ++idx) {
	        const auto& point = points[idx];
	        std::cerr << "  #" << (idx + 1)
	                  << " " << point.bus << "." << point.phase
	                  << " vm_ours=" << point.vm_lhs
	                  << " vm_dss=" << point.vm_rhs
	                  << " vm_err=" << point.vm_error
	                  << " va_ours=" << point.va_lhs_deg
	                  << " va_dss=" << point.va_rhs_deg
	                  << " va_err_deg=" << point.angle_error_deg << '\n';
	      }
	    }
	    require_or_throw(stats.count >= 30, "IEEE13 compare point count < 30");
    require_or_throw(
        stats.max_vm_error < 0.002,
        "IEEE13 compact max voltage error vs OpenDSS >= 0.2%");
    require_or_throw(
        stats.max_angle_error_deg < 0.2,
        "IEEE13 compact max angle error vs OpenDSS >= 0.2 deg");

    std::cout << "IEEE13 compact vs OpenDSS official: max_vm_err="
              << format_percent(stats.max_vm_error)
              << ", avg_vm_err=" << format_percent(stats.avg_vm_error)
              << ", max_angle_err=" << stats.max_angle_error_deg
              << " deg, count=" << stats.count << '\n';
  } catch (const std::exception& ex) {
    std::cerr << "[test_phase_domain_ieee13] " << ex.what() << '\n';
    return 1;
  }

  return 0;
}
