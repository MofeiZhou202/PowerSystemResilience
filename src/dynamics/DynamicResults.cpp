#include "hacdcpf/dynamics/DynamicResults.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
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

std::string fixed(double value, int precision) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(precision) << value;
  return os.str();
}

}  // namespace

std::string to_csv(const DynamicResults& results,
                   const DynamicResultExportOptions& options) {
  std::ostringstream os;
  const char d = options.delimiter == '\0' ? ',' : options.delimiter;

  // Small-signal (modal) screen as a leading '#'-commented block (design doc
  // §18). Comment-prefixed so the time-series table below stays plain CSV.
  if (options.include_modal && results.modal.computed) {
    const auto& m = results.modal;
    os << "# small_signal computed=" << (m.computed ? 1 : 0)
       << " success=" << (m.success ? 1 : 0)
       << " stable=" << (m.stable ? 1 : 0)
       << " n_differential=" << m.n_differential
       << " n_algebraic=" << m.n_algebraic
       << " min_damping_ratio=" << m.min_damping_ratio << '\n';
    os << "# mode,eigen_real_1_s,eigen_imag_rad_s,frequency_hz,damping_ratio,"
          "oscillatory,dominant_state,participation\n";
    int idx = 1;
    for (const auto& mode : m.modes) {
      os << "# " << idx++ << ',' << mode.eigen_real << ',' << mode.eigen_imag << ','
         << mode.frequency_hz << ',' << mode.damping_ratio << ','
         << (mode.oscillatory ? 1 : 0) << ',' << mode.dominant_state << ',';
      bool pf = true;
      for (const auto& p : mode.participation) {
        if (!pf) os << '|';
        pf = false;
        os << p.state_label << '=' << p.factor;
      }
      os << '\n';
    }
    os << '\n';  // blank line separates the modal block from the time series
  }

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

std::string to_modal_report(const DynamicModalSummary& modal,
                            int max_modes,
                            int max_participation_modes) {
  if (!modal.computed) return {};
  std::ostringstream os;
  os << "### Small-signal (modal) screen\n\n";
  if (!modal.success) {
    os << "> Analysis did not complete: " << modal.message << "\n";
    return os.str();
  }

  int rhp = 0;
  for (const auto& m : modal.modes)
    if (m.eigen_real > 1e-6) ++rhp;

  if (modal.stable) {
    os << "**Stable** — all eigenvalues lie in the left half-plane.\n\n";
  } else {
    os << "**UNSTABLE** — " << rhp
       << (rhp == 1 ? " eigenvalue" : " eigenvalues")
       << " in the right half-plane.\n\n";
  }
  os << "- Differential states: " << modal.n_differential << "\n";
  os << "- Algebraic variables: " << modal.n_algebraic << "\n";
  os << "- Minimum damping ratio: " << fixed(modal.min_damping_ratio, 4) << "\n\n";

  if (modal.modes.empty()) {
    os << "_No differential states to analyze._\n";
    return os.str();
  }

  const int n = std::min<int>(max_modes, static_cast<int>(modal.modes.size()));
  os << "| # | Re(λ) [1/s] | Im(λ) [rad/s] | Freq [Hz] | Damping ζ | Osc | Dominant state |\n";
  os << "| --- | --- | --- | --- | --- | --- | --- |\n";
  for (int i = 0; i < n; ++i) {
    const auto& m = modal.modes[static_cast<std::size_t>(i)];
    os << "| " << (i + 1) << " | " << fixed(m.eigen_real, 4) << " | "
       << fixed(m.eigen_imag, 4) << " | " << fixed(m.frequency_hz, 4) << " | "
       << fixed(m.damping_ratio, 4) << " | " << (m.oscillatory ? "yes" : "—")
       << " | `" << m.dominant_state << "` |\n";
  }
  if (static_cast<int>(modal.modes.size()) > n) {
    os << "\n_… " << (static_cast<int>(modal.modes.size()) - n)
       << " further modes omitted._\n";
  }

  const int pn = std::min<int>(max_participation_modes, n);
  for (int i = 0; i < pn; ++i) {
    const auto& m = modal.modes[static_cast<std::size_t>(i)];
    if (m.participation.empty()) continue;
    os << "\n#### Mode " << (i + 1) << " participation (ζ="
       << fixed(m.damping_ratio, 3);
    if (m.oscillatory) os << ", " << fixed(m.frequency_hz, 3) << " Hz";
    os << ")\n";
    for (const auto& p : m.participation) {
      os << "- `" << p.state_label << "` — " << fixed(100.0 * p.factor, 1) << "%\n";
    }
  }
  return os.str();
}

}  // namespace hacdcpf::dynamics

