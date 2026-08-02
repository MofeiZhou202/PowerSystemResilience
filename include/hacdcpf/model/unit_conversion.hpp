#pragma once

/// model/unit_conversion.hpp
/// =========================
/// Bridge from engineering "actual" values (ohms, kilovolts) to the per-unit
/// quantities the power-flow / OPF solvers consume.
///
/// Historically every branch had to be supplied with per-unit impedance
/// (`r_pu`, `x_pu`, ...).  Importers that follow the ETAP / OpenDSS convention
/// instead carry real physical data — resistance per kilometre, line length and
/// a base voltage per bus.  `convert_actual_to_per_unit` lets such a system be
/// solved directly: it computes the per-unit impedance from the actual values
/// using the standard base impedance \f$Z_\text{base}=V_\text{base}^2/S_\text{base}\f$.

#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf {

/// Fill in per-unit branch impedances from actual per-length values.
///
/// The conversion is **opt-in and non-destructive**:
///   * An AC branch is converted only when its per-unit impedance is still zero
///     (`r_pu == 0 && x_pu == 0`) AND it carries actual per-length data
///     (`r_ohm_per_km` / `x_ohm_per_km`) with a positive `length_km` and a
///     positive from-bus `base_kv`.
///   * A DC branch is converted only when `r_pu == 0` AND it carries
///     `r_ohm_per_km` with a positive `length_km` and a positive base voltage.
///
/// Line charging (`b_us_per_km`, microsiemens/km, preferred when present; or
/// `c_nf_per_km`, nanofarads/km at `ac.freq_hz`) fills `b_pu` independently
/// when it is still zero. Parallel circuits divide the series impedance and
/// scale the shunt by `n_parallel`.
///
/// Branches that already provide per-unit data are left untouched, so existing
/// per-unit cases are completely unaffected.  The call is idempotent.
///
/// @return the number of branches whose per-unit values were filled in.
int convert_actual_to_per_unit(HybridPowerSystem& sys);

}  // namespace hacdcpf
