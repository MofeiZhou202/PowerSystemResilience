#include "hacdcpf/power_flow/jacobian_builder.hpp"

#include "hacdcpf/power_flow/ac_kernel_impl.hpp"

#include <cmath>
#include <vector>

#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/ncp_functions.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"

namespace hacdcpf::powerflow {

namespace {

void reset_or_resize(Eigen::VectorXd& v, int n) {
  if (v.size() != n) {
    v = Eigen::VectorXd::Zero(n);
  } else {
    v.setZero();
  }
}

void build_power_spec(const SolverData& data,
                      const std::vector<ACBus>& ac_buses,
                      const std::vector<VSCConverter>& converters,
                      const Eigen::VectorXd& pg,
                      const Eigen::VectorXd& qg,
                      const Eigen::VectorXd& vm,
                      const Eigen::VectorXd& va,
                      const Eigen::VectorXd& vdc,
                      const Eigen::VectorXd& pcalc,
                      Eigen::VectorXd& p_spec,
                      Eigen::VectorXd& q_spec,
                      Eigen::VectorXd& pdc_spec) {
  const int n = static_cast<int>(ac_buses.size());
  const int ndc = static_cast<int>(data.dc_buses.size());

  for (int i = 0; i < n; ++i) {
    const double v = vm[i];
    double pd, qd, pw0, pw1, pw2, qw0, qw1, qw2;
    if (data.has_component_loads) {
      pd = data.pd_pu[i];
      qd = data.qd_pu[i];
      pw0 = data.bus_zip_pp[i];
      pw1 = data.bus_zip_ip[i];
      pw2 = data.bus_zip_zp[i];
      qw0 = data.bus_zip_pq[i];
      qw1 = data.bus_zip_iq[i];
      qw2 = data.bus_zip_zq[i];
    } else {
      pd = ac_buses[static_cast<size_t>(i)].pd_mw / data.base_mva;
      qd = ac_buses[static_cast<size_t>(i)].qd_mvar / data.base_mva;
      pw0 = data.zip_pw[0]; pw1 = data.zip_pw[1]; pw2 = data.zip_pw[2];
      qw0 = data.zip_qw[0]; qw1 = data.zip_qw[1]; qw2 = data.zip_qw[2];
    }
    p_spec[i] = pg[i] - (pd * pw0 + pd * pw1 * v + pd * pw2 * v * v);
    q_spec[i] = qg[i] - (qd * qw0 + qd * qw1 * v + qd * qw2 * v * v);
  }
  // pdc_spec is the net DC injection (generation positive, load negative),
  // mirroring the AC convention p_spec = pg - pd above. Bus-level DC demand is
  // therefore a negative injection.
  for (int i = 0; i < ndc; ++i) {
    pdc_spec[i] = -data.dc_buses[static_cast<size_t>(i)].pd_mw / data.base_mva;
  }
  // DC-side storage: positive p_mw = discharge = generation.
  for (const auto& st : data.dc_storage) {
    if (!st.in_service) continue;
    const int dc_bus = st.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] += st.p_mw / data.base_mva;
    }
  }
  // DC-side static generators.
  for (const auto& sg : data.dc_static_generators) {
    if (!sg.in_service) continue;
    const int dc_bus = sg.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] += sg.p_mw * sg.scaling / data.base_mva;
    }
  }
  // DC-side component loads: consumption is a negative net injection.
  for (const auto& ld : data.dc_loads) {
    if (!ld.in_service) continue;
    const int dc_bus = ld.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] -= ld.p_mw / data.base_mva;
    }
  }
  // DC-side PV arrays: generation is a positive net injection.
  for (const auto& pv : data.dc_pv_arrays) {
    if (!pv.in_service) continue;
    const int dc_bus = pv.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      pdc_spec[dc_bus] += pv.p_set_mw / data.base_mva;
    }
  }
  // DC-DC converters: draw from input bus, inject into output bus.
  // Include I²R loss: ploss = r_eq * p_in² / Vdc_in² (subtracted from output).
  for (const auto& dcdc : data.dcdc_converters) {
    if (!dcdc.in_service) continue;
    const int bin  = dcdc.bus_in  - 1;
    const int bout = dcdc.bus_out - 1;
    if (bin < 0 || bin >= ndc || bout < 0 || bout >= ndc) continue;
    const DCDCPowerTransfer transfer = dcdc_power_transfer(dcdc, vdc, data.base_mva);
    pdc_spec[bin]  -= transfer.p_in_pu;
    pdc_spec[bout] += transfer.p_out_pu;
  }
  // Energy-router AC ports: p_mw > 0 injects into bus, p_mw < 0 consumes.
  for (const auto& er : data.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service || p.port_type != ERPortType::AC) continue;
      const int ac_bus = p.bus - 1;
      if (ac_bus >= 0 && ac_bus < n) {
        p_spec[ac_bus] += p.p_mw / data.base_mva;
        q_spec[ac_bus] += p.q_mvar / data.base_mva;
      }
    }
  }
  // Energy-router DC ports.
  for (const auto& er : data.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service || p.port_type != ERPortType::DC) continue;
      const int dc_bus = p.bus - 1;
      if (dc_bus >= 0 && dc_bus < ndc) {
        pdc_spec[dc_bus] += p.p_mw / data.base_mva;
      }
    }
  }

  for (const auto& conv : converters) {
    if (!conv.in_service) {
      continue;
    }
    const int ac_bus = conv.bus_ac - 1;
    const int dc_bus = conv.bus_dc - 1;
    if (ac_bus >= 0 && ac_bus < n) {
      const auto [pac, qac] =
          converter_ac_injection(conv, vm, va, vdc, data.base_mva, data.loss_model);
      p_spec[ac_bus] += pac;
      q_spec[ac_bus] += qac;
    }
    if (dc_bus >= 0 && dc_bus < ndc) {
      if (conv.control_mode == ConverterMode::AC_GRID_FORMING && ac_bus >= 0 &&
          ac_bus < n && pcalc.size() == n) {
        // Energy-conduit coupling (multi-converter model r1 §4.2, F_loss): an AC
        // grid-forming converter's AC bus is a slack whose active power is free,
        // so to conserve energy the converter must draw exactly that AC power
        // (plus losses) from the DC side.  Its AC output is the slack injection
        //   P_ac = pcalc[ac] − p_spec[ac]
        // (network injection minus the co-located generation/load already in
        // p_spec; the converter itself contributes 0 to p_spec in this mode), and
        // its DC injection tracks it: P_dc = −(P_ac + loss(P_ac)).  This makes the
        // DC-bus balance reflect the true AC power instead of the scheduled p_set,
        // so the converter is energy-consistent regardless of the schedule.
        const double p_ac = pcalc[ac_bus] - p_spec[ac_bus];
        const double vdc_bus =
            (dc_bus >= 0 && dc_bus < ndc) ? vdc[dc_bus] : 1.0;
        const double ploss =
            converter_loss(conv, p_ac, vdc_bus, data.base_mva, data.loss_model);
        double pdc = -(p_ac + ploss);
        if (conv.r_conv_ac_pu > 0.0) {
          const double vm_ac = std::max(vm[ac_bus], 0.1);
          pdc -= conv.r_conv_ac_pu * p_ac * p_ac / (vm_ac * vm_ac);
        }
        pdc_spec[dc_bus] += pdc;
      } else {
        pdc_spec[dc_bus] +=
            converter_dc_injection(conv, vm, va, vdc, data.base_mva, data.loss_model);
      }
    }
  }
}

double evaluate_residual_impl(const SolverData& data,
                              const JacobianContext& ctx,
                              const std::vector<ACBus>& ac_buses,
                              const std::vector<VSCConverter>& converters,
                              const Eigen::VectorXd& pg,
                              const Eigen::VectorXd& qg,
                              const Eigen::VectorXd& vm,
                              const Eigen::VectorXd& va,
                              const Eigen::VectorXd& vdc,
                              Eigen::VectorXd& pcalc,
                              Eigen::VectorXd& qcalc,
                              Eigen::VectorXd& p_spec,
                              Eigen::VectorXd& q_spec,
                              Eigen::VectorXd& pdc_linear,
                              Eigen::VectorXd& pdc_calc,
                              Eigen::VectorXd& pdc_spec,
                              Eigen::VectorXd& mismatch,
                              JacobianPattern& pattern,
                              int ac_eval_threads,
                              bool build_jacobian) {
  const int n = ctx.n;
  const int ndc = ctx.ndc;
  const int nvar = ctx.nvar;

  reset_or_resize(pcalc, n);
  reset_or_resize(qcalc, n);
  reset_or_resize(p_spec, n);
  reset_or_resize(q_spec, n);
  reset_or_resize(pdc_linear, ndc);
  reset_or_resize(pdc_calc, ndc);
  reset_or_resize(pdc_spec, ndc);
  reset_or_resize(mismatch, nvar);

  // Derive the values pointer directly from pattern.matrix inside this function
  // so the pointer cannot be stale — pattern.matrix is non-const here and cannot
  // be reallocated by any caller code between obtaining the pointer and its use.
  double* values = (build_jacobian && pattern.matrix.nonZeros() > 0)
                       ? pattern.matrix.valuePtr()
                       : nullptr;
  if (values != nullptr) {
    std::fill(values, values + pattern.matrix.nonZeros(), 0.0);
  }

  if (ctx.n > 0 && !pattern.ac_entries.empty()) {
    if (build_jacobian && values != nullptr) {
      evaluate_ac_kernel_parallel(pattern, vm, va, ac_eval_threads, true, values, pcalc, qcalc);
    } else {
      evaluate_ac_kernel_parallel(pattern, vm, va, ac_eval_threads, false, nullptr, pcalc, qcalc);
    }
  }

  if (build_jacobian && values != nullptr) {
    for (int i = 0; i < n; ++i) {
      const double vi = vm[i];
      const double vi_safe = std::max(std::abs(vi), ctx.min_vm_pu);
      const double gii = pattern.g_diag[static_cast<size_t>(i)];
      const double bii = pattern.b_diag[static_cast<size_t>(i)];

      const int p_va_nz = pattern.p_va_diag_nz[static_cast<size_t>(i)];
      const int p_vm_nz = pattern.p_vm_diag_nz[static_cast<size_t>(i)];
      const int q_va_nz = pattern.q_va_diag_nz[static_cast<size_t>(i)];
      const int q_vm_nz = pattern.q_vm_diag_nz[static_cast<size_t>(i)];

      if (p_va_nz >= 0) {
        values[p_va_nz] += -qcalc[i] - bii * vi * vi;
      }
      if (q_va_nz >= 0) {
        values[q_va_nz] += pcalc[i] - gii * vi * vi;
      }
      if (p_vm_nz >= 0) {
        values[p_vm_nz] += pcalc[i] / vi_safe + gii * vi;
        // ZIP load derivative: -d(Pd_load)/dVm
        double pd, pw1, pw2;
        if (data.has_component_loads) {
          pd = data.pd_pu[i];
          pw1 = data.bus_zip_ip[i];
          pw2 = data.bus_zip_zp[i];
        } else {
          pd = ac_buses[static_cast<size_t>(i)].pd_mw / data.base_mva;
          pw1 = data.zip_pw[1];
          pw2 = data.zip_pw[2];
        }
        values[p_vm_nz] -= pd * pw1 + 2.0 * vi * pd * pw2;
      }
      if (q_vm_nz >= 0) {
        values[q_vm_nz] += qcalc[i] / vi_safe - bii * vi;
        // ZIP load derivative: -d(Qd_load)/dVm
        double qd, qw1, qw2;
        if (data.has_component_loads) {
          qd = data.qd_pu[i];
          qw1 = data.bus_zip_iq[i];
          qw2 = data.bus_zip_zq[i];
        } else {
          qd = ac_buses[static_cast<size_t>(i)].qd_mvar / data.base_mva;
          qw1 = data.zip_qw[1];
          qw2 = data.zip_qw[2];
        }
        values[q_vm_nz] -= qd * qw1 + 2.0 * vi * qd * qw2;
      }
    }
  }

  build_power_spec(data, ac_buses, converters, pg, qg, vm, va, vdc, pcalc, p_spec, q_spec, pdc_spec);

  pdc_linear = data.gdc * vdc;
  pdc_calc = vdc.array() * pdc_linear.array();

  if (build_jacobian && values != nullptr) {
    for (const auto& entry : pattern.dc_entries) {
      const double dcalc = entry.diagonal ? (pdc_linear[entry.i] + entry.g * vdc[entry.i])
                                          : (entry.g * vdc[entry.i]);
      values[entry.nz] += dcalc;
    }

    // Cross-coupling Jacobian terms (Direction 1).
    if (data.enable_coupled_jacobian) {
      for (const auto& ce : pattern.coupling_entries) {
        const auto& conv = data.converters[static_cast<size_t>(ce.conv_index)];
        const auto ac_jac =
            converter_ac_jacobian_vdc(conv, vdc, data.base_mva, data.loss_model);
        // AC rows ↔ DC column: P/Q-row(ac_bus) → Vdc-col(dc_bus).
        if (ce.p_vdc_nz >= 0) values[ce.p_vdc_nz] -= ac_jac.dpac_dvdc;
        if (ce.q_vdc_nz >= 0) values[ce.q_vdc_nz] -= ac_jac.dqac_dvdc;
        // DC row ↔ AC-Vm column: DC-row(dc_bus) → Vm-col(ac_bus).
        // Arises from the AC-conduction-loss coupling (r_conv_ac_pu): the DC bus
        // supplies ploss_AC = r·pac²/Vm_AC², so ∂(pdc_calc−pdc_spec)/∂Vm_AC = −dpdc/dVm.
        if (ce.dc_vm_nz >= 0 && conv.r_conv_ac_pu > 0.0) {
          const auto [pac0, qac0] =
              converter_ac_injection(conv, vm, va, vdc, data.base_mva, data.loss_model);
          (void)qac0;
          const auto dcvm_jac =
              converter_dc_jacobian_vm_ac(conv, vm, pac0, data.base_mva);
          values[ce.dc_vm_nz] -= dcvm_jac.dpdc_dvm_ac;
        }
        // NOTE: dc_vdc_nz (within-DC-block diagonal) is intentionally NOT applied here.
        // It is now handled unconditionally in the "always-on DC self-consistency" block
        // below, so that the DC Newton equations have exact Jacobians regardless of
        // enable_coupled_jacobian.
      }
    }

    // Always-on: DC self-consistency Jacobian.
    // ∂pdc_spec/∂vdc for VDC-mode VSC converters and DCDC droop mode.
    // These within-DC-block derivatives are required for the DC Newton equations to have
    // exact Jacobians, regardless of whether AC↔DC cross-coupling is enabled.
    //
    // Sign convention: J += -(∂spec/∂x)  (mismatch = spec - calc, so J = ∂(spec-calc)/∂x).
    for (const auto& conv : converters) {
      if (!conv.in_service) continue;
      const int dc_bus = conv.bus_dc - 1;
      if (dc_bus < 0 || dc_bus >= ctx.ndc) continue;
      const int nz = pattern.dc_vdc_diag_nz[static_cast<size_t>(dc_bus)];
      if (nz < 0) continue;
      const auto dc_jac = converter_dc_jacobian_vdc(conv, vdc, data.base_mva, data.loss_model);
      if (dc_jac.dpdc_dvdc != 0.0) {
        values[nz] -= dc_jac.dpdc_dvdc;
      }
    }
    for (const auto& de : pattern.dcdc_coupling_entries) {
      const auto& dcdc = data.dcdc_converters[static_cast<size_t>(de.dcdc_index)];
      const auto jac = dcdc_jacobian_vdc(dcdc, vdc, data.base_mva);
      if (de.dc_out_vdc_in_nz >= 0) values[de.dc_out_vdc_in_nz] -= jac.dpout_dvdc_in;
    }
    for (const auto& de : pattern.dcdc_droop_entries) {
      const auto& dcdc = data.dcdc_converters[static_cast<size_t>(de.dcdc_index)];
      const DCDCPowerTransfer transfer = dcdc_power_transfer(dcdc, vdc, data.base_mva);
      if (de.dc_out_vdc_out_nz >= 0) {
        values[de.dc_out_vdc_out_nz] -= transfer.dpout_dvdc_out;
      }
      if (de.dc_in_vdc_out_nz >= 0) {
        values[de.dc_in_vdc_out_nz] += transfer.dpin_dvdc_out;
      }
    }
  }

  for (int row = 0; row < ctx.np; ++row) {
    const int i = ctx.non_slack[static_cast<size_t>(row)];
    mismatch[row] = p_spec[i] - pcalc[i];
  }
  for (int row = 0; row < ctx.nq; ++row) {
    const int i = ctx.pq[static_cast<size_t>(row)];
    mismatch[ctx.np + row] = q_spec[i] - qcalc[i];
  }
  for (int row = 0; row < ctx.ndc_eq; ++row) {
    const int i = ctx.dc_non_slack[static_cast<size_t>(row)];
    mismatch[ctx.np + ctx.nq + row] = pdc_spec[i] - pdc_calc[i];
  }

  // Augmented equations (Direction 2): override selected Q/DC rows.
  if (data.enable_augmented_equations) {
    for (size_t k = 0; k < ctx.augmented_q_buses.size(); ++k) {
      const int bus = ctx.augmented_q_buses[k];
      const int q_row = ctx.q_row[static_cast<size_t>(bus)];
      if (q_row < 0) continue;
      // Solver convention is mismatch = spec - calc. For a setpoint equation
      // f(Vm) = Vm - Vset = 0, the Newton RHS must be -(f) = Vset - Vm.
      mismatch[q_row] = ctx.augmented_q_targets[k] - vm[bus];
      // Zero out Jacobian row and set diagonal to 1.
      if (build_jacobian && values != nullptr) {
        const int nnz = static_cast<int>(pattern.matrix.nonZeros());
        const int* outer = pattern.matrix.outerIndexPtr();
        const int* inner = pattern.matrix.innerIndexPtr();
        // Zero all entries in this row.
        for (int col = 0; col < pattern.matrix.outerSize(); ++col) {
          for (int nz = outer[col]; nz < outer[col + 1]; ++nz) {
            if (inner[nz] == q_row) {
              values[nz] = 0.0;
            }
          }
        }
        // Set diagonal: q_row → vm_col[bus]
        const int vm_col = ctx.vm_col[static_cast<size_t>(bus)];
        if (vm_col >= 0) {
          const int diag_nz = pattern.q_vm_diag_nz[static_cast<size_t>(bus)];
          if (diag_nz >= 0) {
            values[diag_nz] = 1.0;
          }
        }
        (void)nnz;
      }
    }
    for (size_t k = 0; k < ctx.augmented_dc_buses.size(); ++k) {
      const int bus = ctx.augmented_dc_buses[k];
      const int dc_row = ctx.dc_row[static_cast<size_t>(bus)];
      if (dc_row < 0) continue;
      // Same sign convention as the AC augmented setpoint rows.
      mismatch[dc_row] = ctx.augmented_dc_targets[k] - vdc[bus];
      // Zero out Jacobian row and set diagonal to 1.
      if (build_jacobian && values != nullptr) {
        const int* outer = pattern.matrix.outerIndexPtr();
        const int* inner = pattern.matrix.innerIndexPtr();
        for (int col = 0; col < pattern.matrix.outerSize(); ++col) {
          for (int nz = outer[col]; nz < outer[col + 1]; ++nz) {
            if (inner[nz] == dc_row) {
              values[nz] = 0.0;
            }
          }
        }
        const int vdc_col = ctx.vdc_col[static_cast<size_t>(bus)];
        if (vdc_col >= 0) {
          // Find the nz for (dc_row, vdc_col) — it's on the diagonal of DC block.
          for (int nz = outer[vdc_col]; nz < outer[vdc_col + 1]; ++nz) {
            if (inner[nz] == dc_row) {
              values[nz] = 1.0;
              break;
            }
          }
        }
      }
    }
  }

  // Semi-smooth Newton NCP equations (Direction 3).
  if (data.enable_semi_smooth_newton && !ctx.ncp_buses.empty()) {
    for (const auto& nd : ctx.ncp_buses) {
      const int bus = nd.bus;
      const int q_row = ctx.q_row[static_cast<size_t>(bus)];
      if (q_row < 0) continue;

      // Qg = Qcalc + Qload - Qconv (implied reactive generation).
      const double qload_pu = data.has_component_loads
                                   ? data.qd_pu[bus]
                                   : (data.ac_buses[static_cast<size_t>(bus)].qd_mvar / data.base_mva);
      const double qg_implied = qcalc[bus] + qload_pu;

      // NCP residual replaces Q mismatch.
      const double ncp_mu = data.ncp_mu;
      if (ncp_mu > 0.0) {
        mismatch[q_row] =
            pv_pq_smooth_ncp(qg_implied, nd.qmin, nd.qmax, vm[bus], nd.vm_set, ncp_mu);
      } else {
        mismatch[q_row] = pv_pq_ncp(qg_implied, nd.qmin, nd.qmax, vm[bus], nd.vm_set);
      }

      if (build_jacobian && values != nullptr) {
        const auto [dF_dQg, dF_dVm] =
            (ncp_mu > 0.0)
                ? pv_pq_smooth_ncp_jacobian(qg_implied, nd.qmin, nd.qmax, vm[bus], nd.vm_set, ncp_mu)
                : pv_pq_ncp_jacobian(qg_implied, nd.qmin, nd.qmax, vm[bus], nd.vm_set);

        // The standard Q-row stores dQcalc/d(var) and the Newton system is
        // J·Δx = mismatch, with J_row = dQcalc/d(var) and mismatch = Qspec - Qcalc.
        // For NCP, mismatch = F_ncp.  The "Jacobian" must be -dF_ncp/d(var) so that
        // Δx = J^{-1} * F_ncp = [-dF/d(var)]^{-1} * F_ncp converges.
        //
        // dF_ncp/d(var) = dF_dQg * dQg/d(var) + dF_dVm * dVm/d(var)
        //               = dF_dQg * dQcalc/d(var) + dF_dVm * dVm/d(var)
        //
        // So J_ncp_row = -dF_dQg * dQcalc/d(var)  (scale existing entries by -dF_dQg)
        //              + -dF_dVm on the Vm diagonal.
        const int* outer = pattern.matrix.outerIndexPtr();
        const int* inner = pattern.matrix.innerIndexPtr();
        for (int col = 0; col < pattern.matrix.outerSize(); ++col) {
          for (int nz = outer[col]; nz < outer[col + 1]; ++nz) {
            if (inner[nz] == q_row) {
              values[nz] *= -dF_dQg;
            }
          }
        }

        // Add -dF/dVm contribution on the Vm diagonal.
        const int vm_col = ctx.vm_col[static_cast<size_t>(bus)];
        if (vm_col >= 0) {
          const int diag_nz = pattern.q_vm_diag_nz[static_cast<size_t>(bus)];
          if (diag_nz >= 0) {
            values[diag_nz] += -dF_dVm;
          }
        }
      }
    }
  }

  return inf_norm(mismatch);
}

}  // namespace

double evaluate_residual_and_jacobian(const SolverData& data,
                                      const JacobianContext& ctx,
                                      const std::vector<ACBus>& ac_buses,
                                      const std::vector<VSCConverter>& converters,
                                      const Eigen::VectorXd& pg,
                                      const Eigen::VectorXd& qg,
                                      const Eigen::VectorXd& vm,
                                      const Eigen::VectorXd& va,
                                      const Eigen::VectorXd& vdc,
                                      Eigen::VectorXd& pcalc,
                                      Eigen::VectorXd& qcalc,
                                      Eigen::VectorXd& p_spec,
                                      Eigen::VectorXd& q_spec,
                                      Eigen::VectorXd& pdc_linear,
                                      Eigen::VectorXd& pdc_calc,
                                      Eigen::VectorXd& pdc_spec,
                                      Eigen::VectorXd& mismatch,
                                      JacobianPattern& pattern,
                                      int ac_eval_threads) {
  return evaluate_residual_impl(data,
                                ctx,
                                ac_buses,
                                converters,
                                pg,
                                qg,
                                vm,
                                va,
                                vdc,
                                pcalc,
                                qcalc,
                                p_spec,
                                q_spec,
                                pdc_linear,
                                pdc_calc,
                                pdc_spec,
                                mismatch,
                                pattern,
                                ac_eval_threads,
                                true);
}

double evaluate_residual_only(const SolverData& data,
                              const JacobianContext& ctx,
                              const std::vector<ACBus>& ac_buses,
                              const std::vector<VSCConverter>& converters,
                              const Eigen::VectorXd& pg,
                              const Eigen::VectorXd& qg,
                              const Eigen::VectorXd& vm,
                              const Eigen::VectorXd& va,
                              const Eigen::VectorXd& vdc,
                              Eigen::VectorXd& pcalc,
                              Eigen::VectorXd& qcalc,
                              Eigen::VectorXd& p_spec,
                              Eigen::VectorXd& q_spec,
                              Eigen::VectorXd& pdc_linear,
                              Eigen::VectorXd& pdc_calc,
                              Eigen::VectorXd& pdc_spec,
                              Eigen::VectorXd& mismatch,
                              const JacobianPattern& pattern,
                              int ac_eval_threads) {
  // build_jacobian=false: the impl will never call valuePtr() or write through
  // the values pointer, so casting away const is safe here.
  return evaluate_residual_impl(data,
                                ctx,
                                ac_buses,
                                converters,
                                pg,
                                qg,
                                vm,
                                va,
                                vdc,
                                pcalc,
                                qcalc,
                                p_spec,
                                q_spec,
                                pdc_linear,
                                pdc_calc,
                                pdc_spec,
                                mismatch,
                                const_cast<JacobianPattern&>(pattern),
                                ac_eval_threads,
                                false);
}

}  // namespace hacdcpf::powerflow
