/// model/unit_conversion.cpp
/// =========================
/// Implementation of convert_actual_to_per_unit (see header for the contract).

#include "hacdcpf/model/unit_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace hacdcpf {

namespace {

constexpr double kZeroTol = 1e-15;

inline bool is_zero(double v) { return std::abs(v) < kZeroTol; }

}  // namespace

int convert_actual_to_per_unit(HybridPowerSystem& sys) {
  const double base_mva = sys.base_mva > 1e-9 ? sys.base_mva : 100.0;
  int converted = 0;

  // ── AC branches ──────────────────────────────────────────────────────
  if (!sys.ac.buses.empty() && !sys.ac.branches.empty()) {
    const double ac_base_mva = sys.ac.base_mva > 1e-9 ? sys.ac.base_mva : base_mva;

    std::unordered_map<int, double> base_kv;
    base_kv.reserve(sys.ac.buses.size());
    for (const auto& b : sys.ac.buses) base_kv[b.index] = b.base_kv;

    for (auto& br : sys.ac.branches) {
      const bool has_pu     = !is_zero(br.r_pu) || !is_zero(br.x_pu);
      const bool has_actual = !is_zero(br.r_ohm_per_km) || !is_zero(br.x_ohm_per_km);
      const bool has_charging_actual =
          !is_zero(br.b_us_per_km) || !is_zero(br.c_nf_per_km);
      if (br.length_km <= 0.0 ||
          ((has_pu || !has_actual) &&
           (!is_zero(br.b_pu) || !has_charging_actual)))
        continue;

      const auto it = base_kv.find(br.from_bus);
      if (it == base_kv.end() || it->second <= 0.0) continue;
      const double kv = it->second;
      const double z_base = (kv * kv) / ac_base_mva;
      if (z_base <= 0.0) continue;

      const int n = std::max(1, br.n_parallel);
      bool branch_converted = false;
      if (!has_pu && has_actual) {
        const double r_ohm =
            (br.r_ohm_per_km * br.length_km) / static_cast<double>(n);
        const double x_ohm =
            (br.x_ohm_per_km * br.length_km) / static_cast<double>(n);
        br.r_pu = r_ohm / z_base;
        br.x_pu = x_ohm / z_base;
        branch_converted = true;
      }

      // Line charging: b_us_per_km is the total shunt susceptance per km in
      // microsiemens.  Per-unit susceptance is B[S] * Z_base.
      if (is_zero(br.b_pu) && has_charging_actual) {
        double b_siemens = 0.0;
        if (!is_zero(br.b_us_per_km)) {
          b_siemens = br.b_us_per_km * 1.0e-6 * br.length_km *
                      static_cast<double>(n);
        } else {
          const double capacitance_f =
              br.c_nf_per_km * 1.0e-9 * br.length_km *
              static_cast<double>(n);
          b_siemens = 2.0 * 3.14159265358979323846 *
                      std::max(sys.ac.freq_hz, 0.0) * capacitance_f;
        }
        br.b_pu = b_siemens * z_base;
        branch_converted = true;
      }
      if (branch_converted) ++converted;
    }
  }

  // ── DC branches ──────────────────────────────────────────────────────
  if (!sys.dc.buses.empty() && !sys.dc.branches.empty()) {
    const double dc_base_mva = sys.dc.base_mva > 1e-9 ? sys.dc.base_mva : base_mva;

    std::unordered_map<int, double> base_kv;
    base_kv.reserve(sys.dc.buses.size());
    for (const auto& b : sys.dc.buses) base_kv[b.index] = b.base_kv;

    for (auto& br : sys.dc.branches) {
      if (!is_zero(br.r_pu) || is_zero(br.r_ohm_per_km) || br.length_km <= 0.0) {
        continue;
      }
      double kv = 0.0;
      const auto it = base_kv.find(br.from_bus);
      if (it != base_kv.end() && it->second > 0.0) {
        kv = it->second;
      } else if (br.base_kv > 0.0) {
        kv = br.base_kv;
      }
      if (kv <= 0.0) continue;
      const double z_base = (kv * kv) / dc_base_mva;
      if (z_base <= 0.0) continue;

      const int n = std::max(1, br.n_parallel);
      const double r_ohm = (br.r_ohm_per_km * br.length_km) / static_cast<double>(n);
      br.r_pu = r_ohm / z_base;
      ++converted;
    }
  }

  return converted;
}

}  // namespace hacdcpf
