#include "hacdcpf/dynamics/DynamicResults.hpp"

#include <cmath>
#include <sstream>

namespace hacdcpf::dynamics {

namespace {

void append_cell(std::ostringstream& os, char delimiter, bool& first, double value) {
  if (!first) os << delimiter;
  first = false;
  if (std::isfinite(value)) {
    os << value;
  }
}

}  // namespace

std::string to_csv(const DynamicResults& results,
                   const DynamicResultExportOptions& options) {
  std::ostringstream os;
  const char d = options.delimiter == '\0' ? ',' : options.delimiter;

  bool first = true;
  os << "time_s";
  if (options.include_ac_voltage_magnitudes) {
    const Eigen::Index n_ac =
        results.snapshots.empty() ? 0 : results.snapshots.front().vac_abc.size();
    for (Eigen::Index i = 0; i < n_ac; ++i) {
      os << d << "vac_" << i << "_mag_pu";
    }
  }
  if (options.include_dc_voltages) {
    const Eigen::Index n_dc =
        results.snapshots.empty() ? 0 : results.snapshots.front().vdc.size();
    for (Eigen::Index i = 0; i < n_dc; ++i) {
      os << d << "vdc_" << i << "_pu";
    }
  }
  if (options.include_device_outputs && !results.snapshots.empty()) {
    for (const auto& dev : results.snapshots.front().device_outputs) {
      for (const auto& [metric, _] : dev.values) {
        os << d << dev.type << "_" << dev.component_index << "_" << metric;
      }
    }
  }
  os << '\n';

  for (const auto& snap : results.snapshots) {
    first = true;
    append_cell(os, d, first, snap.time_s);
    if (options.include_ac_voltage_magnitudes) {
      for (Eigen::Index i = 0; i < snap.vac_abc.size(); ++i) {
        append_cell(os, d, first, std::abs(snap.vac_abc[i]));
      }
    }
    if (options.include_dc_voltages) {
      for (Eigen::Index i = 0; i < snap.vdc.size(); ++i) {
        append_cell(os, d, first, snap.vdc[i]);
      }
    }
    if (options.include_device_outputs && !results.snapshots.empty()) {
      for (const auto& dev : snap.device_outputs) {
        for (const auto& [_, value] : dev.values) {
          append_cell(os, d, first, value);
        }
      }
    }
    os << '\n';
  }
  return os.str();
}

}  // namespace hacdcpf::dynamics

