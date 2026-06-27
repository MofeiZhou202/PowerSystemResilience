#include "hacdcpf/power_flow/jacobian_builder.hpp"

#include <cstdint>
#include <unordered_set>
#include <vector>

#include <Eigen/Sparse>

namespace hacdcpf::powerflow {

namespace {

std::uint64_t rc_key(int row, int col) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 32U) |
         static_cast<std::uint32_t>(col);
}

void add_pattern_position(int row,
                          int col,
                          std::vector<Eigen::Triplet<double>>& triplets,
                          std::unordered_set<std::uint64_t>& seen) {
  if (row < 0 || col < 0) {
    return;
  }
  const std::uint64_t key = rc_key(row, col);
  if (seen.insert(key).second) {
    triplets.emplace_back(row, col, 0.0);
  }
}

void build_entry_to_nz(JacobianPattern& pattern) {
  pattern.entry_to_nz.clear();
  pattern.entry_to_nz.reserve(static_cast<size_t>(pattern.matrix.nonZeros()) * 2U + 1U);

  const int* outer = pattern.matrix.outerIndexPtr();
  const int* inner = pattern.matrix.innerIndexPtr();
  for (int col = 0; col < pattern.matrix.outerSize(); ++col) {
    for (int nz = outer[col]; nz < outer[col + 1]; ++nz) {
      pattern.entry_to_nz.emplace(rc_key(inner[nz], col), nz);
    }
  }
}

int lookup_nz(const JacobianPattern& pattern, int row, int col) {
  const auto it = pattern.entry_to_nz.find(rc_key(row, col));
  if (it == pattern.entry_to_nz.end()) {
    return -1;
  }
  return it->second;
}

}  // namespace

JacobianPattern build_jacobian_pattern(const SolverData& data, const JacobianContext& ctx) {
  JacobianPattern pattern;
  pattern.matrix.resize(ctx.nvar, ctx.nvar);
  if (ctx.nvar == 0) {
    return pattern;
  }

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(data.ybus.nonZeros()) * 4U +
                   static_cast<size_t>(data.gdc.nonZeros()) + static_cast<size_t>(ctx.nvar));
  std::unordered_set<std::uint64_t> seen;
  seen.reserve(triplets.capacity() * 2U + 1U);

  for (int col = 0; col < data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(data.ybus, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());

      const int p_row = ctx.p_row[static_cast<size_t>(i)];
      if (p_row >= 0) {
        add_pattern_position(p_row, ctx.va_col[static_cast<size_t>(j)], triplets, seen);
        add_pattern_position(p_row, ctx.vm_col[static_cast<size_t>(j)], triplets, seen);
      }

      const int q_row = ctx.q_row[static_cast<size_t>(i)];
      if (q_row >= 0) {
        add_pattern_position(q_row, ctx.va_col[static_cast<size_t>(j)], triplets, seen);
        add_pattern_position(q_row, ctx.vm_col[static_cast<size_t>(j)], triplets, seen);
      }
    }
  }

  // Ensure diagonals exist even for edge cases with sparse/degenerate rows.
  for (int i = 0; i < ctx.n; ++i) {
    const int p_row = ctx.p_row[static_cast<size_t>(i)];
    if (p_row >= 0) {
      add_pattern_position(p_row, ctx.va_col[static_cast<size_t>(i)], triplets, seen);
      add_pattern_position(p_row, ctx.vm_col[static_cast<size_t>(i)], triplets, seen);
    }
    const int q_row = ctx.q_row[static_cast<size_t>(i)];
    if (q_row >= 0) {
      add_pattern_position(q_row, ctx.va_col[static_cast<size_t>(i)], triplets, seen);
      add_pattern_position(q_row, ctx.vm_col[static_cast<size_t>(i)], triplets, seen);
    }
  }

  for (int col = 0; col < data.gdc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(data.gdc, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      add_pattern_position(ctx.dc_row[static_cast<size_t>(i)],
                           ctx.vdc_col[static_cast<size_t>(j)],
                           triplets,
                           seen);
    }
  }

  for (int i = 0; i < ctx.ndc; ++i) {
    add_pattern_position(ctx.dc_row[static_cast<size_t>(i)],
                         ctx.vdc_col[static_cast<size_t>(i)],
                         triplets,
                         seen);
  }

  // Cross-coupling sparsity for exact coupled Jacobian (Direction 1).
  if (data.enable_coupled_jacobian) {
    for (const auto& conv : data.converters) {
      if (!conv.in_service) continue;
      const int ac_bus = conv.bus_ac - 1;
      const int dc_bus = conv.bus_dc - 1;
      if (ac_bus < 0 || ac_bus >= ctx.n || dc_bus < 0 || dc_bus >= ctx.ndc) continue;
      const int p_row = ctx.p_row[static_cast<size_t>(ac_bus)];
      const int q_row = ctx.q_row[static_cast<size_t>(ac_bus)];
      const int dc_row = ctx.dc_row[static_cast<size_t>(dc_bus)];
      const int vdc_col = ctx.vdc_col[static_cast<size_t>(dc_bus)];
      // P-row(ac_bus) → Vdc-col(dc_bus)
      add_pattern_position(p_row, vdc_col, triplets, seen);
      // Q-row(ac_bus) → Vdc-col(dc_bus)
      add_pattern_position(q_row, vdc_col, triplets, seen);
      // DC-row(dc_bus) → Vdc-col(dc_bus) (already present from Gdc, but ensure)
      add_pattern_position(dc_row, vdc_col, triplets, seen);
      // DC-row(dc_bus) → Vm-col(ac_bus)  [AC–AC cross-coupling; always reserve
      // the structural slot so the sparsity pattern is fixed across iterations.]
      const int vm_col_ac = ctx.vm_col[static_cast<size_t>(ac_bus)];
      add_pattern_position(dc_row, vm_col_ac, triplets, seen);
    }
    for (const auto& dcdc : data.dcdc_converters) {
      if (!dcdc.in_service || dcdc.r_eq_pu <= 0.0) continue;
      const int bin = dcdc.bus_in - 1;
      const int bout = dcdc.bus_out - 1;
      if (bin < 0 || bin >= ctx.ndc || bout < 0 || bout >= ctx.ndc) continue;
      // DC-row(bout) → Vdc-col(bin)  [cross-bus I²R coupling]
      add_pattern_position(ctx.dc_row[static_cast<size_t>(bout)],
                           ctx.vdc_col[static_cast<size_t>(bin)],
                           triplets, seen);
    }
  }

  // Always reserve pattern slots for DCDC voltage-forming (Droop or Voltage):
  // diagonal + input-bus cross-column. Needed regardless of enable_coupled_jacobian
  // because the former modifies the DC spec (pdc_spec[bout] and pdc_spec[bin] both
  // depend on Vdc_out via the droop / voltage-forming law).
  for (const auto& dcdc : data.dcdc_converters) {
    if (!dcdc.in_service) continue;
    const bool forms =
        dcdc.control_mode == DCDCControlMode::Voltage ||
        (dcdc.control_mode == DCDCControlMode::Droop && dcdc.k_droop != 0.0);
    if (!forms) continue;
    const int bin = dcdc.bus_in - 1;
    const int bout = dcdc.bus_out - 1;
    if (bin < 0 || bin >= ctx.ndc || bout < 0 || bout >= ctx.ndc) continue;
    // DC-row(bout) → Vdc-col(bout)  [droop diagonal: already in Gdc, but ensure]
    add_pattern_position(ctx.dc_row[static_cast<size_t>(bout)],
                         ctx.vdc_col[static_cast<size_t>(bout)],
                         triplets, seen);
    // DC-row(bin) → Vdc-col(bout)  [droop cross: input row depends on output voltage]
    add_pattern_position(ctx.dc_row[static_cast<size_t>(bin)],
                         ctx.vdc_col[static_cast<size_t>(bout)],
                         triplets, seen);
  }

  pattern.matrix.setFromTriplets(triplets.begin(), triplets.end());
  pattern.matrix.makeCompressed();
  build_entry_to_nz(pattern);

  pattern.p_va_diag_nz.assign(static_cast<size_t>(ctx.n), -1);
  pattern.p_vm_diag_nz.assign(static_cast<size_t>(ctx.n), -1);
  pattern.q_va_diag_nz.assign(static_cast<size_t>(ctx.n), -1);
  pattern.q_vm_diag_nz.assign(static_cast<size_t>(ctx.n), -1);
  pattern.g_diag.assign(static_cast<size_t>(ctx.n), 0.0);
  pattern.b_diag.assign(static_cast<size_t>(ctx.n), 0.0);
  for (int i = 0; i < ctx.n; ++i) {
    const int p_row = ctx.p_row[static_cast<size_t>(i)];
    const int q_row = ctx.q_row[static_cast<size_t>(i)];
    const int va_col = ctx.va_col[static_cast<size_t>(i)];
    const int vm_col = ctx.vm_col[static_cast<size_t>(i)];
    if (p_row >= 0 && va_col >= 0) {
      pattern.p_va_diag_nz[static_cast<size_t>(i)] = lookup_nz(pattern, p_row, va_col);
    }
    if (p_row >= 0 && vm_col >= 0) {
      pattern.p_vm_diag_nz[static_cast<size_t>(i)] = lookup_nz(pattern, p_row, vm_col);
    }
    if (q_row >= 0 && va_col >= 0) {
      pattern.q_va_diag_nz[static_cast<size_t>(i)] = lookup_nz(pattern, q_row, va_col);
    }
    if (q_row >= 0 && vm_col >= 0) {
      pattern.q_vm_diag_nz[static_cast<size_t>(i)] = lookup_nz(pattern, q_row, vm_col);
    }
  }

  // Always-filled: DC diagonal nz for each DC bus (used by always-on VSC/droop Jacobian).
  pattern.dc_vdc_diag_nz.assign(static_cast<size_t>(ctx.ndc), -1);
  for (int i = 0; i < ctx.ndc; ++i) {
    const int dc_row = ctx.dc_row[static_cast<size_t>(i)];
    const int vdc_col = ctx.vdc_col[static_cast<size_t>(i)];
    if (dc_row >= 0 && vdc_col >= 0) {
      pattern.dc_vdc_diag_nz[static_cast<size_t>(i)] = lookup_nz(pattern, dc_row, vdc_col);
    }
  }

  // Always-built: DCDC droop entries (not gated by enable_coupled_jacobian).
  pattern.dcdc_droop_entries.clear();
  for (size_t di = 0; di < data.dcdc_converters.size(); ++di) {
    const auto& dcdc = data.dcdc_converters[di];
    if (!dcdc.in_service) continue;
    const bool forms =
        dcdc.control_mode == DCDCControlMode::Voltage ||
        (dcdc.control_mode == DCDCControlMode::Droop && dcdc.k_droop != 0.0);
    if (!forms) continue;
    const int bin = dcdc.bus_in - 1;
    const int bout = dcdc.bus_out - 1;
    if (bin < 0 || bin >= ctx.ndc || bout < 0 || bout >= ctx.ndc) continue;
    JacobianPattern::DCDroopEntry de;
    de.dcdc_index = static_cast<int>(di);
    de.bus_in     = bin;
    de.bus_out    = bout;
    de.k_droop    = dcdc.k_droop;
    de.eta        = (dcdc.eta > 1e-9) ? dcdc.eta : 1.0;
    const int dc_out_row  = ctx.dc_row[static_cast<size_t>(bout)];
    const int vdc_out_col = ctx.vdc_col[static_cast<size_t>(bout)];
    const int dc_in_row   = ctx.dc_row[static_cast<size_t>(bin)];
    if (dc_out_row >= 0 && vdc_out_col >= 0) {
      de.dc_out_vdc_out_nz = lookup_nz(pattern, dc_out_row, vdc_out_col);
    }
    if (dc_in_row >= 0 && vdc_out_col >= 0) {
      de.dc_in_vdc_out_nz = lookup_nz(pattern, dc_in_row, vdc_out_col);
    }
    pattern.dcdc_droop_entries.push_back(de);
  }

  pattern.ac_entries.clear();
  pattern.ac_entries.reserve(static_cast<size_t>(data.ybus.nonZeros()));
  for (int col = 0; col < data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(data.ybus, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      const int p_row = ctx.p_row[static_cast<size_t>(i)];
      const int q_row = ctx.q_row[static_cast<size_t>(i)];
      const int va_col = ctx.va_col[static_cast<size_t>(j)];
      const int vm_col = ctx.vm_col[static_cast<size_t>(j)];

      JacobianPattern::ACEntry entry;
      entry.i = i;
      entry.j = j;
      entry.g = it.value().real();
      entry.b = it.value().imag();

      if (p_row >= 0 && va_col >= 0) {
        entry.p_va_nz = lookup_nz(pattern, p_row, va_col);
      }
      if (p_row >= 0 && vm_col >= 0) {
        entry.p_vm_nz = lookup_nz(pattern, p_row, vm_col);
      }
      if (q_row >= 0 && va_col >= 0) {
        entry.q_va_nz = lookup_nz(pattern, q_row, va_col);
      }
      if (q_row >= 0 && vm_col >= 0) {
        entry.q_vm_nz = lookup_nz(pattern, q_row, vm_col);
      }

      if (i == j) {
        pattern.g_diag[static_cast<size_t>(i)] = entry.g;
        pattern.b_diag[static_cast<size_t>(i)] = entry.b;
      }

      if (p_row >= 0 || q_row >= 0) {
        pattern.ac_entries.push_back(entry);
      }
    }
  }

  pattern.dc_entries.clear();
  pattern.dc_entries.reserve(static_cast<size_t>(data.gdc.nonZeros()));
  for (int col = 0; col < data.gdc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(data.gdc, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      const int row = ctx.dc_row[static_cast<size_t>(i)];
      const int col_vdc = ctx.vdc_col[static_cast<size_t>(j)];
      if (row < 0 || col_vdc < 0) {
        continue;
      }
      const int nz = lookup_nz(pattern, row, col_vdc);
      if (nz < 0) {
        continue;
      }

      JacobianPattern::DCEntry entry;
      entry.i = i;
      entry.j = j;
      entry.nz = nz;
      entry.g = it.value();
      entry.diagonal = (i == j);
      pattern.dc_entries.push_back(entry);
    }
  }

  // Build cross-coupling nz entries for exact coupled Jacobian.
  pattern.coupling_entries.clear();
  pattern.dcdc_coupling_entries.clear();
  if (data.enable_coupled_jacobian) {
    for (size_t ci = 0; ci < data.converters.size(); ++ci) {
      const auto& conv = data.converters[ci];
      if (!conv.in_service) continue;
      const int ac_bus = conv.bus_ac - 1;
      const int dc_bus = conv.bus_dc - 1;
      if (ac_bus < 0 || ac_bus >= ctx.n || dc_bus < 0 || dc_bus >= ctx.ndc) continue;

      JacobianPattern::CouplingEntry ce;
      ce.conv_index = static_cast<int>(ci);
      ce.ac_bus = ac_bus;
      ce.dc_bus = dc_bus;
      const int vdc_col = ctx.vdc_col[static_cast<size_t>(dc_bus)];
      if (vdc_col >= 0) {
        const int p_row = ctx.p_row[static_cast<size_t>(ac_bus)];
        const int q_row = ctx.q_row[static_cast<size_t>(ac_bus)];
        const int dc_row = ctx.dc_row[static_cast<size_t>(dc_bus)];
        if (p_row >= 0) ce.p_vdc_nz = lookup_nz(pattern, p_row, vdc_col);
        if (q_row >= 0) ce.q_vdc_nz = lookup_nz(pattern, q_row, vdc_col);
        if (dc_row >= 0) ce.dc_vdc_nz = lookup_nz(pattern, dc_row, vdc_col);
      }
      // DC-row(dc_bus) → Vm-col(ac_bus): AC-resistance cross-block coupling.
      {
        const int dc_row = ctx.dc_row[static_cast<size_t>(dc_bus)];
        const int vm_col_ac = ctx.vm_col[static_cast<size_t>(ac_bus)];
        if (dc_row >= 0 && vm_col_ac >= 0) {
          ce.dc_vm_nz = lookup_nz(pattern, dc_row, vm_col_ac);
        }
      }
      pattern.coupling_entries.push_back(ce);
    }
    for (size_t di = 0; di < data.dcdc_converters.size(); ++di) {
      const auto& dcdc = data.dcdc_converters[di];
      if (!dcdc.in_service || dcdc.r_eq_pu <= 0.0) continue;
      const int bin = dcdc.bus_in - 1;
      const int bout = dcdc.bus_out - 1;
      if (bin < 0 || bin >= ctx.ndc || bout < 0 || bout >= ctx.ndc) continue;

      JacobianPattern::DCDCCouplingEntry de;
      de.dcdc_index = static_cast<int>(di);
      de.bus_in = bin;
      de.bus_out = bout;
      const int dc_row_out = ctx.dc_row[static_cast<size_t>(bout)];
      const int vdc_col_in = ctx.vdc_col[static_cast<size_t>(bin)];
      if (dc_row_out >= 0 && vdc_col_in >= 0) {
        de.dc_out_vdc_in_nz = lookup_nz(pattern, dc_row_out, vdc_col_in);
      }
      pattern.dcdc_coupling_entries.push_back(de);
    }
  }

  // The sparse index map is only needed during pattern construction.
  pattern.entry_to_nz.clear();
  pattern.entry_to_nz.rehash(0);

  return pattern;
}

}  // namespace hacdcpf::powerflow
