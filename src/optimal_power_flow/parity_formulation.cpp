#include "hacdcpf/optimal_power_flow/formulation.hpp"
#include "hacdcpf/detail/core_compat.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SparseLU>

#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"

namespace hacdcpf::opf::parity {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

double clamp_interior(double x, double lo, double hi) {
  const double w = hi - lo;
  const double eps = std::max(1e-9, 0.01 * std::max(w, 1e-9));  // 1% margin (Julia :clamp_1pct)
  return std::clamp(x, lo + eps, hi - eps);
}

void ensure_size(Eigen::VectorXd& v, int n) {
  if (v.size() != n) {
    v = Eigen::VectorXd::Zero(n);
  } else {
    v.setZero();
  }
}

double resolve_voll_auto(const Problem& prob) {
  if (prob.options.voll > 0.0) {
    return prob.options.voll;
  }
  if (prob.gen_var_to_data.empty()) {
    return 100.0;
  }
  double max_marginal = 0.0;
  for (size_t k = 0; k < prob.gen_var_to_data.size(); ++k) {
    const auto& gen = prob.data.generators[static_cast<size_t>(prob.gen_var_to_data[k])];
    const double slope_hi = gen.cost_c1 + 2.0 * gen.cost_c2 * std::max(gen.pmax_mw, gen.pmin_mw);
    max_marginal = std::max(max_marginal, std::abs(slope_hi));
  }
  return std::max(100.0, 1.1 * max_marginal);
}

double converter_smax_pu(const Problem& prob, int conv_data_idx) {
  const auto& conv = prob.data.converters[static_cast<size_t>(conv_data_idx)];
  double smax = conv.p_rated_mw;
  if (!(smax > 0.0) || !std::isfinite(smax)) {
    smax = std::max({std::abs(conv.pmax_mw),
                     std::abs(conv.pmin_mw),
                     std::abs(conv.qmax_mvar),
                     std::abs(conv.qmin_mvar),
                     1e-6});
  }
  return smax / prob.data.base_mva;
}

}  // namespace

Problem build_problem(const HybridPowerSystem& sys, const ParityOptions& opt) {
  Problem prob;
  prob.data = core::make_solver_data(sys, LossModelType::Linear);
  prob.options = opt;

  const int nb = static_cast<int>(prob.data.ac_buses.size());
  const int ndc = static_cast<int>(prob.data.dc_buses.size());

  // Pre-compute per-bus demand in p.u. from the load table (pd_pu) when it
  // is available.  project_to_canonical_models() may move bus.pd_mw into Load
  // entries and zero the bus field, so reading bus.pd_mw directly can give
  // zero demand.  data.pd_pu is populated by aggregate_load_demand() from
  // the Load table and is always correct.
  prob.pd_demand_pu = Eigen::VectorXd::Zero(nb);
  prob.qd_demand_pu = Eigen::VectorXd::Zero(nb);
  if (prob.data.has_component_loads) {
    prob.pd_demand_pu = prob.data.pd_pu;
    prob.qd_demand_pu = prob.data.qd_pu;
  } else {
    for (int i = 0; i < nb; ++i) {
      prob.pd_demand_pu[i] = prob.data.ac_buses[static_cast<size_t>(i)].pd_mw / prob.data.base_mva;
      prob.qd_demand_pu[i] = prob.data.ac_buses[static_cast<size_t>(i)].qd_mvar / prob.data.base_mva;
    }
  }

  // Populate per-bus ZIP load coefficients from SolverData.
  // When component loads exist, SolverData carries per-bus weighted ZIP;
  // otherwise fall back to the global ZIP weights (default: constant-power).
  prob.zip_pp = Eigen::VectorXd::Ones(nb);
  prob.zip_ip = Eigen::VectorXd::Zero(nb);
  prob.zip_zp = Eigen::VectorXd::Zero(nb);
  prob.zip_pq = Eigen::VectorXd::Ones(nb);
  prob.zip_iq = Eigen::VectorXd::Zero(nb);
  prob.zip_zq = Eigen::VectorXd::Zero(nb);
  if (prob.data.has_component_loads && prob.data.bus_zip_pp.size() == nb) {
    for (int i = 0; i < nb; ++i) {
      prob.zip_pp[i] = prob.data.bus_zip_pp[i];
      prob.zip_ip[i] = prob.data.bus_zip_ip[i];
      prob.zip_zp[i] = prob.data.bus_zip_zp[i];
      prob.zip_pq[i] = prob.data.bus_zip_pq[i];
      prob.zip_iq[i] = prob.data.bus_zip_iq[i];
      prob.zip_zq[i] = prob.data.bus_zip_zq[i];
    }
  } else if (!prob.data.has_component_loads) {
    for (int i = 0; i < nb; ++i) {
      prob.zip_pp[i] = prob.data.zip_pw[0];
      prob.zip_ip[i] = prob.data.zip_pw[1];
      prob.zip_zp[i] = prob.data.zip_pw[2];
      prob.zip_pq[i] = prob.data.zip_qw[0];
      prob.zip_iq[i] = prob.data.zip_qw[1];
      prob.zip_zq[i] = prob.data.zip_qw[2];
    }
  }

  prob.gen_var_to_data.clear();
  prob.gen_bus.clear();
  prob.gen_var_to_data.reserve(prob.data.generators.size());
  prob.gen_bus.reserve(prob.data.generators.size());
  for (size_t gi = 0; gi < prob.data.generators.size(); ++gi) {
    const int bus = prob.data.generators[gi].bus - 1;
    if (bus < 0 || bus >= nb) {
      continue;
    }
    prob.gen_var_to_data.push_back(static_cast<int>(gi));
    prob.gen_bus.push_back(bus);
  }

  prob.conv_var_to_data.clear();
  prob.conv_ac_bus.clear();
  prob.conv_dc_bus.clear();
  prob.conv_var_to_data.reserve(prob.data.converters.size());
  prob.conv_ac_bus.reserve(prob.data.converters.size());
  prob.conv_dc_bus.reserve(prob.data.converters.size());
  for (size_t ci = 0; ci < prob.data.converters.size(); ++ci) {
    const auto& conv = prob.data.converters[ci];
    if (!conv.in_service) {
      continue;
    }
    const int ac = conv.bus_ac - 1;
    const int dc = conv.bus_dc - 1;
    if (ac < 0 || ac >= nb || dc < 0 || dc >= ndc) {
      continue;
    }
    prob.conv_var_to_data.push_back(static_cast<int>(ci));
    prob.conv_ac_bus.push_back(ac);
    prob.conv_dc_bus.push_back(dc);
  }

  prob.branch_limited.clear();
  prob.branch_limited.reserve(prob.data.ac_branches.size());
  for (size_t bi = 0; bi < prob.data.ac_branches.size(); ++bi) {
    const auto& br = prob.data.ac_branches[bi];
    if (br.in_service && br.rate_a_mva > 0.0) {
      prob.branch_limited.push_back(static_cast<int>(bi));
    }
  }

  prob.dc_branch_limited.clear();
  prob.dc_branch_limited.reserve(prob.data.dc_branches.size());
  for (size_t bi = 0; bi < prob.data.dc_branches.size(); ++bi) {
    const auto& br = prob.data.dc_branches[bi];
    if (br.in_service && br.rate_a_mva > 0.0) {
      prob.dc_branch_limited.push_back(static_cast<int>(bi));
    }
  }

  // --- Enhanced component: curtailable renewables ---
  prob.ren_var_to_data.clear();
  prob.ren_source.clear();
  prob.ren_bus.clear();
  for (size_t ri = 0; ri < prob.data.renewable_gens.size(); ++ri) {
    const auto& rg = prob.data.renewable_gens[ri];
    if (!rg.in_service || !rg.curtailable) continue;
    const int bus = rg.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.ren_var_to_data.push_back(static_cast<int>(ri));
    prob.ren_source.push_back(0);  // 0 = RenewableGen
    prob.ren_bus.push_back(bus);
  }
  for (size_t pi = 0; pi < prob.data.pv_systems.size(); ++pi) {
    const auto& pv = prob.data.pv_systems[pi];
    if (!pv.in_service || !pv.controllable) continue;
    const int bus = pv.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.ren_var_to_data.push_back(static_cast<int>(pi));
    prob.ren_source.push_back(1);  // 1 = PVSystem
    prob.ren_bus.push_back(bus);
  }

  // --- Enhanced component: storage ---
  prob.stor_var_to_data.clear();
  prob.stor_bus.clear();
  for (size_t si = 0; si < prob.data.storage_units.size(); ++si) {
    const auto& st = prob.data.storage_units[si];
    if (!st.in_service) continue;
    const int bus = st.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.stor_var_to_data.push_back(static_cast<int>(si));
    prob.stor_bus.push_back(bus);
  }

  // --- Enhanced component: DC storage ---
  prob.stor_dc_var_to_data.clear();
  prob.stor_dc_bus.clear();
  for (size_t si = 0; si < prob.data.dc_storage.size(); ++si) {
    const auto& st = prob.data.dc_storage[si];
    if (!st.in_service) continue;
    const int bus = st.bus - 1;
    if (bus < 0 || bus >= ndc) continue;
    prob.stor_dc_var_to_data.push_back(static_cast<int>(si));
    prob.stor_dc_bus.push_back(bus);
  }

  // --- Enhanced component: DCDC converters ---
  prob.dcdc_var_to_data.clear();
  prob.dcdc_bus_in.clear();
  prob.dcdc_bus_out.clear();
  for (size_t di = 0; di < prob.data.dcdc_converters.size(); ++di) {
    const auto& dc = prob.data.dcdc_converters[di];
    if (!dc.in_service) continue;
    const int bin = dc.bus_in - 1;
    const int bout = dc.bus_out - 1;
    if (bin < 0 || bin >= ndc || bout < 0 || bout >= ndc) continue;
    prob.dcdc_var_to_data.push_back(static_cast<int>(di));
    prob.dcdc_bus_in.push_back(bin);
    prob.dcdc_bus_out.push_back(bout);
  }

  // --- Enhanced component: flexible loads ---
  prob.flex_var_to_data.clear();
  prob.flex_bus.clear();
  for (size_t fi = 0; fi < prob.data.flexible_loads.size(); ++fi) {
    const auto& fl = prob.data.flexible_loads[fi];
    if (!fl.in_service || !fl.controllable) continue;
    const int bus = fl.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.flex_var_to_data.push_back(static_cast<int>(fi));
    prob.flex_bus.push_back(bus);
  }

  VarIndex vidx;
  vidx.n_va = nb;
  vidx.n_vm = nb;
  vidx.n_pg = static_cast<int>(prob.gen_var_to_data.size());
  vidx.n_qg = vidx.n_pg;
  vidx.n_vdc = ndc;
  vidx.n_pac = static_cast<int>(prob.conv_var_to_data.size());
  vidx.n_qac = vidx.n_pac;
  vidx.n_pdc = vidx.n_pac;
  vidx.n_dpd = opt.load_shedding ? nb : 0;
  vidx.n_dqd = opt.load_shedding ? nb : 0;
  vidx.n_pren = static_cast<int>(prob.ren_var_to_data.size());
  vidx.n_qren = vidx.n_pren;
  vidx.n_pstor = static_cast<int>(prob.stor_var_to_data.size());
  vidx.n_qstor = vidx.n_pstor;
  vidx.n_pstordc = static_cast<int>(prob.stor_dc_var_to_data.size());
  vidx.n_pdcdc = static_cast<int>(prob.dcdc_var_to_data.size());
  vidx.n_pflex = static_cast<int>(prob.flex_var_to_data.size());

  int off = 0;
  vidx.i_va = off;
  off += vidx.n_va;
  vidx.i_vm = off;
  off += vidx.n_vm;
  vidx.i_pg = off;
  off += vidx.n_pg;
  vidx.i_qg = off;
  off += vidx.n_qg;
  vidx.i_vdc = off;
  off += vidx.n_vdc;
  vidx.i_pac = off;
  off += vidx.n_pac;
  vidx.i_qac = off;
  off += vidx.n_qac;
  vidx.i_pdc = off;
  off += vidx.n_pdc;
  vidx.i_dpd = off;
  off += vidx.n_dpd;
  vidx.i_dqd = off;
  off += vidx.n_dqd;
  vidx.i_pren = off;
  off += vidx.n_pren;
  vidx.i_qren = off;
  off += vidx.n_qren;
  vidx.i_pstor = off;
  off += vidx.n_pstor;
  vidx.i_qstor = off;
  off += vidx.n_qstor;
  vidx.i_pstordc = off;
  off += vidx.n_pstordc;
  vidx.i_pdcdc = off;
  off += vidx.n_pdcdc;
  vidx.i_pflex = off;
  off += vidx.n_pflex;
  vidx.n_total = off;
  prob.vidx = vidx;

  ConstraintIndex cidx;
  cidx.n_pbal_ac = nb;
  cidx.n_qbal_ac = nb;
  cidx.n_pbal_dc = ndc;
  cidx.n_conv_bal = vidx.n_pac;
  cidx.n_dcdc_bal = vidx.n_pdcdc;
  cidx.i_pbal_ac = 0;
  cidx.i_qbal_ac = cidx.i_pbal_ac + cidx.n_pbal_ac;
  cidx.i_pbal_dc = cidx.i_qbal_ac + cidx.n_qbal_ac;
  cidx.i_conv_bal = cidx.i_pbal_dc + cidx.n_pbal_dc;
  cidx.i_dcdc_bal = cidx.i_conv_bal + cidx.n_conv_bal;
  cidx.n_eq_total = cidx.i_dcdc_bal + cidx.n_dcdc_bal;
  cidx.n_sf = static_cast<int>(prob.branch_limited.size());
  cidx.n_st = cidx.n_sf;
  cidx.n_sconv = vidx.n_pac;
  cidx.n_sdc = static_cast<int>(prob.dc_branch_limited.size());
  cidx.n_ineq_nonlin = cidx.n_sf + cidx.n_st + cidx.n_sconv + cidx.n_sdc;
  prob.cidx = cidx;

  double max_pd = 1.0;
  double max_qd = 1.0;
  for (int i = 0; i < nb; ++i) {
    max_pd = std::max(max_pd, std::abs(prob.pd_demand_pu[i]));
    max_qd = std::max(max_qd, std::abs(prob.qd_demand_pu[i]));
  }
  prob.scale_p = 1.0 / max_pd;
  prob.scale_q = 1.0 / max_qd;
  prob.voll_effective = resolve_voll_auto(prob);

  // Extract Ybus diagonal for fast access
  prob.g_diag = Eigen::VectorXd::Zero(nb);
  prob.b_diag = Eigen::VectorXd::Zero(nb);
  for (int i = 0; i < nb; ++i) {
    const auto yii = prob.data.ybus.coeff(i, i);
    prob.g_diag[i] = yii.real();
    prob.b_diag[i] = yii.imag();
  }

  prob.gdc_dense = Eigen::MatrixXd(prob.data.gdc);

  // --- Compute fixed DER injections (non-variable components) ---
  prob.p_fixed_inj = Eigen::VectorXd::Zero(nb);
  prob.q_fixed_inj = Eigen::VectorXd::Zero(nb);

  // StaticGenerator (always fixed — no OPF variable)
  for (const auto& sg : prob.data.static_generators) {
    if (!sg.in_service) continue;
    const int bus = sg.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.p_fixed_inj[bus] += sg.p_mw * sg.scaling / prob.data.base_mva;
    prob.q_fixed_inj[bus] += sg.q_mvar * sg.scaling / prob.data.base_mva;
  }

  // Non-curtailable RenewableGen (fixed)
  for (size_t ri = 0; ri < prob.data.renewable_gens.size(); ++ri) {
    const auto& rg = prob.data.renewable_gens[ri];
    if (!rg.in_service) continue;
    if (rg.curtailable) continue;  // handled as OPF variable
    const int bus = rg.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.p_fixed_inj[bus] += rg.p_mw / prob.data.base_mva;
    prob.q_fixed_inj[bus] += rg.q_mvar / prob.data.base_mva;
  }

  // Non-controllable PVSystem (fixed)
  for (size_t pi = 0; pi < prob.data.pv_systems.size(); ++pi) {
    const auto& pv = prob.data.pv_systems[pi];
    if (!pv.in_service) continue;
    if (pv.controllable) continue;  // handled as OPF variable
    const int bus = pv.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    double p_mw = powerflow::compute_pv_power_mw(pv);
    prob.p_fixed_inj[bus] += p_mw / prob.data.base_mva;
    prob.q_fixed_inj[bus] += pv.q_mvar / prob.data.base_mva;
  }

  // VPP (fixed)
  for (const auto& vpp : prob.data.vpps) {
    if (!vpp.in_service) continue;
    const int bus = vpp.pcc_bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.p_fixed_inj[bus] += vpp.p_output_mw / prob.data.base_mva;
    prob.q_fixed_inj[bus] += vpp.q_output_mvar / prob.data.base_mva;
  }

  // Microgrid (fixed, grid-connected exchange only)
  for (const auto& mg : prob.data.microgrids) {
    if (!mg.in_service) continue;
    if (mg.operating_mode != MicrogridMode::GridConnected) continue;
    const int bus = mg.pcc_bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.p_fixed_inj[bus] += mg.p_exchange_mw / prob.data.base_mva;
  }

  // MobileStorage (fixed)
  for (const auto& ms : prob.data.mobile_storage) {
    if (!ms.in_service) continue;
    if (ms.status == MobileStorageStatus::InTransit) continue;
    const int bus = ms.bus - 1;
    if (bus < 0 || bus >= nb) continue;
    prob.p_fixed_inj[bus] += ms.p_mw / prob.data.base_mva;
    prob.q_fixed_inj[bus] += ms.q_mvar / prob.data.base_mva;
  }

  return prob;
}

void build_variable_bounds(const Problem& prob, Eigen::VectorXd& xmin, Eigen::VectorXd& xmax) {
  const auto& idx = prob.vidx;
  xmin = Eigen::VectorXd::Constant(idx.n_total, -std::numeric_limits<double>::infinity());
  xmax = Eigen::VectorXd::Constant(idx.n_total, std::numeric_limits<double>::infinity());

  for (int i = 0; i < idx.n_va; ++i) {
    xmin[idx.i_va + i] = -kPi;
    xmax[idx.i_va + i] = kPi;
  }
  for (int i = 0; i < idx.n_vm; ++i) {
    const auto& b = prob.data.ac_buses[static_cast<size_t>(i)];
    xmin[idx.i_vm + i] = b.vmin_pu;
    xmax[idx.i_vm + i] = b.vmax_pu;
  }

  for (int k = 0; k < idx.n_pg; ++k) {
    const auto& gen = prob.data.generators[static_cast<size_t>(prob.gen_var_to_data[static_cast<size_t>(k)])];
    const int pcol = idx.i_pg + k;
    const int qcol = idx.i_qg + k;
    xmin[pcol] = gen.pmin_mw / prob.data.base_mva;
    xmax[pcol] = gen.pmax_mw / prob.data.base_mva;
    xmin[qcol] = gen.qmin_mvar / prob.data.base_mva;
    xmax[qcol] = gen.qmax_mvar / prob.data.base_mva;
    if (!gen.in_service) {
      const double p0 = gen.pg_mw / prob.data.base_mva;
      const double q0 = gen.qg_mvar / prob.data.base_mva;
      xmin[pcol] = p0 - 1e-8;
      xmax[pcol] = p0 + 1e-8;
      xmin[qcol] = q0 - 1e-8;
      xmax[qcol] = q0 + 1e-8;
    }
  }

  for (int k = 0; k < idx.n_vdc; ++k) {
    const auto& dcb = prob.data.dc_buses[static_cast<size_t>(k)];
    double lo = dcb.vmin_pu;
    double hi = dcb.vmax_pu;
    if (!(lo < hi)) {
      lo = 0.8;
      hi = 1.2;
    }
    xmin[idx.i_vdc + k] = lo;
    xmax[idx.i_vdc + k] = hi;
  }

  for (int k = 0; k < idx.n_pac; ++k) {
    const auto& conv = prob.data.converters[static_cast<size_t>(prob.conv_var_to_data[static_cast<size_t>(k)])];
    xmin[idx.i_pac + k] = conv.pmin_mw / prob.data.base_mva;
    xmax[idx.i_pac + k] = conv.pmax_mw / prob.data.base_mva;
    xmin[idx.i_qac + k] = conv.qmin_mvar / prob.data.base_mva;
    xmax[idx.i_qac + k] = conv.qmax_mvar / prob.data.base_mva;
    const double smax_pu = converter_smax_pu(prob, prob.conv_var_to_data[static_cast<size_t>(k)]);
    xmin[idx.i_pdc + k] = -smax_pu;
    xmax[idx.i_pdc + k] = smax_pu;
  }

  if (idx.n_dpd > 0) {
    for (int i = 0; i < idx.n_dpd; ++i) {
      xmin[idx.i_dpd + i] = 0.0;
      xmax[idx.i_dpd + i] = std::max(prob.pd_demand_pu[i], 0.01 / prob.data.base_mva);
      xmin[idx.i_dqd + i] = 0.0;
      xmax[idx.i_dqd + i] = std::max(std::abs(prob.qd_demand_pu[i]), 0.01 / prob.data.base_mva);
    }
  }

  // --- Enhanced component bounds ---
  // Curtailable renewables
  for (int k = 0; k < idx.n_pren; ++k) {
    const int src = prob.ren_source[static_cast<size_t>(k)];
    const int di = prob.ren_var_to_data[static_cast<size_t>(k)];
    if (src == 0) {
      const auto& rg = prob.data.renewable_gens[static_cast<size_t>(di)];
      xmin[idx.i_pren + k] = 0.0;
      // Cap at the available (scheduled) power — the resource-available output.
      // p_mw is set by apply_schedule_step / apply_time_series_profiles to the
      // actual available generation; dispatching above it is physically impossible.
      xmax[idx.i_pren + k] = std::min(rg.p_rated_mw, std::max(rg.p_mw, 0.0)) / prob.data.base_mva;
      xmin[idx.i_qren + k] = rg.qmin_mvar / prob.data.base_mva;
      xmax[idx.i_qren + k] = rg.qmax_mvar / prob.data.base_mva;
    } else {
      const auto& pv = prob.data.pv_systems[static_cast<size_t>(di)];
      xmin[idx.i_pren + k] = 0.0;
      // Cap PV output at the available (scheduled) power.
      const double pv_rated = std::max(pv.pmax_mw, pv.sn_mva);
      xmax[idx.i_pren + k] = std::min(pv_rated, std::max(pv.p_mw, 0.0)) / prob.data.base_mva;
      xmin[idx.i_qren + k] = pv.qmin_mvar / prob.data.base_mva;
      xmax[idx.i_qren + k] = pv.qmax_mvar / prob.data.base_mva;
    }
  }

  // Storage — constrain to the scheduled dispatch set by apply_schedule_step.
  // The per-period OPF has no inter-temporal SOC model, so unconstrained
  // storage would discharge at max every period, ignoring the UC schedule.
  // Fix to the scheduled p_mw (net discharge) with a small tolerance band
  // so the IPM can converge.
  for (int k = 0; k < idx.n_pstor; ++k) {
    const auto& st = prob.data.storage_units[static_cast<size_t>(prob.stor_var_to_data[static_cast<size_t>(k)])];
    const double p_sched = st.p_mw / prob.data.base_mva;
    const double eps = 1e-4 / prob.data.base_mva;  // small tolerance
    xmin[idx.i_pstor + k] = std::max(st.pmin_mw / prob.data.base_mva, p_sched - eps);
    xmax[idx.i_pstor + k] = std::min(st.pmax_mw / prob.data.base_mva, p_sched + eps);
    xmin[idx.i_qstor + k] = st.qmin_mvar / prob.data.base_mva;
    xmax[idx.i_qstor + k] = st.qmax_mvar / prob.data.base_mva;
  }

  // DC storage (active power only)
  for (int k = 0; k < idx.n_pstordc; ++k) {
    const auto& st = prob.data.dc_storage[static_cast<size_t>(
        prob.stor_dc_var_to_data[static_cast<size_t>(k)])];
    xmin[idx.i_pstordc + k] = st.pmin_mw / prob.data.base_mva;
    xmax[idx.i_pstordc + k] = st.pmax_mw / prob.data.base_mva;
  }

  // DCDC converters
  for (int k = 0; k < idx.n_pdcdc; ++k) {
    const auto& dc = prob.data.dcdc_converters[static_cast<size_t>(prob.dcdc_var_to_data[static_cast<size_t>(k)])];
    double pmax = (dc.pmax_mw > 1e-9) ? dc.pmax_mw : dc.sn_mva;
    double pmin = dc.pmin_mw;
    if (pmin >= pmax) pmin = -pmax;
    xmin[idx.i_pdcdc + k] = pmin / prob.data.base_mva;
    xmax[idx.i_pdcdc + k] = pmax / prob.data.base_mva;
  }

  // Flexible loads
  for (int k = 0; k < idx.n_pflex; ++k) {
    const auto& fl = prob.data.flexible_loads[static_cast<size_t>(prob.flex_var_to_data[static_cast<size_t>(k)])];
    xmin[idx.i_pflex + k] = (fl.p_mw - fl.flex_down_mw) / prob.data.base_mva;
    xmax[idx.i_pflex + k] = (fl.p_mw + fl.flex_up_mw) / prob.data.base_mva;
  }
}

// DC warm start: solve B_red · θ = -P_inj for voltage angles (matching Julia)
void dc_warm_start(const Problem& prob, Eigen::VectorXd& x0) {
  const auto& idx = prob.vidx;
  const int nb = idx.n_va;
  if (nb <= 1 || idx.n_pg == 0) {
    return;
  }

  // Find slack bus (bus_type == SLACK = 3)
  int slack_idx = -1;
  for (int i = 0; i < nb; ++i) {
    if (prob.data.ac_buses[static_cast<size_t>(i)].bus_type == BusType::SLACK) {
      slack_idx = i;
      break;
    }
  }
  if (slack_idx < 0) {
    return;
  }

  // Compute net power injection: P_inj = P_gen - P_load (per-unit)
  // Use ZIP-adjusted demand at current Vm (flat start ≈ 1.0, but consistent)
  Eigen::VectorXd p_inj = Eigen::VectorXd::Zero(nb);
  for (int k = 0; k < idx.n_pg; ++k) {
    const int bus = prob.gen_bus[static_cast<size_t>(k)];
    p_inj[bus] += x0[idx.i_pg + k];
  }
  for (int i = 0; i < nb; ++i) {
    const double vi = x0[idx.i_vm + i];
    p_inj[i] -= prob.pd_demand_pu[i] * (prob.zip_pp[i] + prob.zip_ip[i] * vi + prob.zip_zp[i] * vi * vi);
  }
  // Include converter AC-side injection
  for (int k = 0; k < idx.n_pac; ++k) {
    const int bus = prob.conv_ac_bus[static_cast<size_t>(k)];
    p_inj[bus] += x0[idx.i_pac + k];
  }

  // Build reduced B matrix (remove slack row/col)
  const int n_red = nb - 1;
  std::vector<int> non_slack;
  non_slack.reserve(static_cast<size_t>(n_red));
  for (int i = 0; i < nb; ++i) {
    if (i != slack_idx) {
      non_slack.push_back(i);
    }
  }

  Eigen::MatrixXd B_red = Eigen::MatrixXd::Zero(n_red, n_red);
  Eigen::VectorXd P_red(n_red);
  // Map from bus index to reduced index (-1 for slack)
  std::vector<int> bus_to_red(static_cast<size_t>(nb), -1);
  for (int ri = 0; ri < n_red; ++ri) {
    bus_to_red[static_cast<size_t>(non_slack[static_cast<size_t>(ri)])] = ri;
    P_red[ri] = -p_inj[non_slack[static_cast<size_t>(ri)]];
  }
  // Fill B_red from sparse Ybus imaginary part
  for (int col = 0; col < prob.data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(prob.data.ybus, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      const int ri = bus_to_red[static_cast<size_t>(i)];
      const int rj = bus_to_red[static_cast<size_t>(j)];
      if (ri >= 0 && rj >= 0) {
        B_red(ri, rj) += it.value().imag();
      }
    }
  }

  // Solve B_red * θ_red = P_red
  Eigen::VectorXd theta_red = B_red.partialPivLu().solve(P_red);
  if (!theta_red.allFinite()) {
    return;
  }

  // Assemble full θ vector
  Eigen::VectorXd va = Eigen::VectorXd::Zero(nb);
  for (int ri = 0; ri < n_red; ++ri) {
    va[non_slack[static_cast<size_t>(ri)]] = theta_red[ri];
  }
  va.array() -= va[slack_idx];  // reference to slack
  va = va.cwiseMax(-kPi).cwiseMin(kPi);  // clamp to [-π, π]

  for (int i = 0; i < nb; ++i) {
    x0[idx.i_va + i] = va[i];
  }
}

// Reactive power warm start: compute Qg to approximately satisfy Q-balance
// given Va from DC warm start and Vm from bus setpoints.
void qg_warm_start(const Problem& prob,
                   const Eigen::VectorXd& xmin,
                   const Eigen::VectorXd& xmax,
                   Eigen::VectorXd& x0) {
  const auto& idx = prob.vidx;
  const int nb = idx.n_va;
  if (nb == 0 || idx.n_qg == 0) {
    return;
  }

  // Compute Q_calc(Va, Vm) for each bus using sparse Ybus
  Eigen::VectorXd q_calc = Eigen::VectorXd::Zero(nb);
  for (int i = 0; i < nb; ++i) {
    const double vi = x0[idx.i_vm + i];
    q_calc[i] = -vi * vi * prob.b_diag[i];
  }
  for (int col = 0; col < prob.data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(prob.data.ybus, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      if (i == j) {
        continue;
      }
      const double gij = it.value().real();
      const double bij = it.value().imag();
      const double vi = x0[idx.i_vm + i];
      const double vj = x0[idx.i_vm + j];
      const double theta = x0[idx.i_va + i] - x0[idx.i_va + j];
      q_calc[i] += vi * vj * (gij * std::sin(theta) - bij * std::cos(theta));
    }
  }

  // For Q-balance: Q_calc - Q_gen + Q_load = 0  →  Q_gen = Q_calc + Q_load
  // Build per-bus Q target using ZIP-adjusted Q demand at current Vm
  Eigen::VectorXd q_target = Eigen::VectorXd::Zero(nb);
  for (int i = 0; i < nb; ++i) {
    const double vi = x0[idx.i_vm + i];
    q_target[i] = q_calc[i] + prob.qd_demand_pu[i] * (prob.zip_pq[i] + prob.zip_iq[i] * vi + prob.zip_zq[i] * vi * vi);
  }

  // Count generators per bus for splitting
  std::vector<int> n_gen_at_bus(static_cast<size_t>(nb), 0);
  for (int k = 0; k < idx.n_qg; ++k) {
    const int bus = prob.gen_bus[static_cast<size_t>(k)];
    n_gen_at_bus[static_cast<size_t>(bus)]++;
  }

  // Set Qg = Q_target / n_gen_at_bus (split evenly), then clamp to bounds
  for (int k = 0; k < idx.n_qg; ++k) {
    const int bus = prob.gen_bus[static_cast<size_t>(k)];
    const int ng = n_gen_at_bus[static_cast<size_t>(bus)];
    double qg_est = q_target[bus] / std::max(ng, 1);
    const double qmin = xmin[idx.i_qg + k];
    const double qmax = xmax[idx.i_qg + k];
    x0[idx.i_qg + k] = std::clamp(qg_est, qmin, qmax);
  }
}

void build_initial_point(const Problem& prob,
                         const Eigen::VectorXd& xmin,
                         const Eigen::VectorXd& xmax,
                         Eigen::VectorXd& x0) {
  const auto& idx = prob.vidx;
  x0 = Eigen::VectorXd::Zero(idx.n_total);
  for (int i = 0; i < idx.n_va; ++i) {
    x0[idx.i_va + i] = prob.data.ac_buses[static_cast<size_t>(i)].va_deg * kDegToRad;
  }
  for (int i = 0; i < idx.n_vm; ++i) {
    x0[idx.i_vm + i] = prob.data.ac_buses[static_cast<size_t>(i)].vm_pu;
  }
  for (int k = 0; k < idx.n_pg; ++k) {
    const auto& gen = prob.data.generators[static_cast<size_t>(prob.gen_var_to_data[static_cast<size_t>(k)])];
    x0[idx.i_pg + k] = gen.pg_mw / prob.data.base_mva;
    x0[idx.i_qg + k] = gen.qg_mvar / prob.data.base_mva;
  }

  // Smart dispatch balancing: when many generators have Pg=0 (common in
  // MATPOWER CDF-converted cases), set them to a fraction of their capacity
  // so the DC warm start gets a realistic distributed dispatch.
  if (idx.n_pg > 1) {
    int n_zero = 0;
    double pmax_zero_sum = 0.0;
    double pg_nonzero_sum = 0.0;
    for (int k = 0; k < idx.n_pg; ++k) {
      if (std::abs(x0[idx.i_pg + k]) < 1e-6) {
        ++n_zero;
        pmax_zero_sum += std::max(xmax[idx.i_pg + k], 0.0);
      } else {
        pg_nonzero_sum += x0[idx.i_pg + k];
      }
    }
    double pd_sum = prob.pd_demand_pu.sum();
    // Distribute the remaining (Pd - Pg_nonzero) among zero-Pg generators
    // proportional to their Pmax
    double deficit = pd_sum - pg_nonzero_sum;
    if (n_zero > 0 && pmax_zero_sum > 1e-6 && deficit > 0.1) {
      for (int k = 0; k < idx.n_pg; ++k) {
        if (std::abs(x0[idx.i_pg + k]) < 1e-6) {
          const double pmn = xmin[idx.i_pg + k];
          const double pmx = xmax[idx.i_pg + k];
          double pg_prop = deficit * std::max(pmx, 0.0) / pmax_zero_sum;
          x0[idx.i_pg + k] = std::clamp(pg_prop, pmn, pmx);
        }
      }
    }
  }

  for (int k = 0; k < idx.n_vdc; ++k) {
    x0[idx.i_vdc + k] = prob.data.dc_buses[static_cast<size_t>(k)].vm_pu;
  }
  for (int k = 0; k < idx.n_pac; ++k) {
    const auto& conv = prob.data.converters[static_cast<size_t>(prob.conv_var_to_data[static_cast<size_t>(k)])];
    x0[idx.i_pac + k] = conv.p_set_mw / prob.data.base_mva;
    x0[idx.i_qac + k] = conv.q_set_mvar / prob.data.base_mva;
    // Injection-positive convention on both sides:
    // Pdc ≈ -(Pac + Ploss). Use -Pac as a warm-start approximation.
    x0[idx.i_pdc + k] = -conv.p_set_mw / prob.data.base_mva;
  }

  // Enhanced component warm start — set to current operating point.
  for (int k = 0; k < idx.n_pren; ++k) {
    const int src = prob.ren_source[static_cast<size_t>(k)];
    const int di = prob.ren_var_to_data[static_cast<size_t>(k)];
    if (src == 0) {
      const auto& rg = prob.data.renewable_gens[static_cast<size_t>(di)];
      x0[idx.i_pren + k] = rg.p_mw / prob.data.base_mva;
      x0[idx.i_qren + k] = rg.q_mvar / prob.data.base_mva;
    } else {
      const auto& pv = prob.data.pv_systems[static_cast<size_t>(di)];
      x0[idx.i_pren + k] = pv.p_mw / prob.data.base_mva;
      x0[idx.i_qren + k] = pv.q_mvar / prob.data.base_mva;
    }
  }
  for (int k = 0; k < idx.n_pstor; ++k) {
    const auto& st = prob.data.storage_units[static_cast<size_t>(prob.stor_var_to_data[static_cast<size_t>(k)])];
    x0[idx.i_pstor + k] = st.p_mw / prob.data.base_mva;
    x0[idx.i_qstor + k] = st.q_mvar / prob.data.base_mva;
  }
  for (int k = 0; k < idx.n_pstordc; ++k) {
    const auto& st = prob.data.dc_storage[static_cast<size_t>(
        prob.stor_dc_var_to_data[static_cast<size_t>(k)])];
    x0[idx.i_pstordc + k] = st.p_mw / prob.data.base_mva;
  }
  for (int k = 0; k < idx.n_pdcdc; ++k) {
    const auto& dc = prob.data.dcdc_converters[static_cast<size_t>(prob.dcdc_var_to_data[static_cast<size_t>(k)])];
    // Warm start: p_ref_mw is OUTPUT-referenced (PF convention),
    // but the OPF variable is INPUT-referenced (p_in), so convert via η.
    const double p_out = dc.p_ref_mw / prob.data.base_mva;
    const double eta = std::clamp(dc.eta, 0.01, 1.0);
    const double p_in = (p_out >= 0.0) ? (p_out / eta) : (p_out * eta);
    x0[idx.i_pdcdc + k] = p_in;
  }
  for (int k = 0; k < idx.n_pflex; ++k) {
    const auto& fl = prob.data.flexible_loads[static_cast<size_t>(prob.flex_var_to_data[static_cast<size_t>(k)])];
    x0[idx.i_pflex + k] = fl.p_mw / prob.data.base_mva;
  }

  // DC warm start: compute voltage angles consistent with P dispatch
  dc_warm_start(prob, x0);

  // Reactive power warm start: set Qg to approximately satisfy Q-balance
  qg_warm_start(prob, xmin, xmax, x0);

  // Load-shedding warm start: set dPd, dQd to absorb per-bus residual
  if (idx.n_dpd > 0) {
    const int nb = idx.n_va;
    // Compute P_calc, Q_calc at each bus using sparse Ybus
    Eigen::VectorXd p_bus = Eigen::VectorXd::Zero(nb);
    Eigen::VectorXd q_bus = Eigen::VectorXd::Zero(nb);
    for (int i = 0; i < nb; ++i) {
      const double vi = x0[idx.i_vm + i];
      p_bus[i] = vi * vi * prob.g_diag[i];
      q_bus[i] = -vi * vi * prob.b_diag[i];
    }
    for (int col = 0; col < prob.data.ybus.outerSize(); ++col) {
      for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(prob.data.ybus, col); it; ++it) {
        const int i = static_cast<int>(it.row());
        const int j = static_cast<int>(it.col());
        if (i == j) {
          continue;
        }
        const double gij = it.value().real();
        const double bij = it.value().imag();
        const double vi = x0[idx.i_vm + i];
        const double vj = x0[idx.i_vm + j];
        const double theta = x0[idx.i_va + i] - x0[idx.i_va + j];
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        p_bus[i] += vi * vj * (gij * c + bij * s);
        q_bus[i] += vi * vj * (gij * s - bij * c);
      }
    }
    for (int i = 0; i < nb; ++i) {
      // Accumulate generation at this bus
      double pg_bus = 0.0;
      double qg_bus = 0.0;
      for (int k = 0; k < idx.n_pg; ++k) {
        if (prob.gen_bus[static_cast<size_t>(k)] == i) {
          pg_bus += x0[idx.i_pg + k];
          qg_bus += x0[idx.i_qg + k];
        }
      }
      // Converter injection at this bus
      double p_conv = 0.0;
      double q_conv = 0.0;
      for (int k = 0; k < idx.n_pac; ++k) {
        if (prob.conv_ac_bus[static_cast<size_t>(k)] == i) {
          p_conv += x0[idx.i_pac + k];
          q_conv += x0[idx.i_qac + k];
        }
      }
      const double vi = x0[idx.i_vm + i];
      const double pd = prob.pd_demand_pu[i] * (prob.zip_pp[i] + prob.zip_ip[i] * vi + prob.zip_zp[i] * vi * vi);
      const double qd = prob.qd_demand_pu[i] * (prob.zip_pq[i] + prob.zip_iq[i] * vi + prob.zip_zq[i] * vi * vi);
      // g_P = P_calc - Pg + pd - dPd - P_conv; set dPd so g_P = 0
      const double dpd_target = p_bus[i] - pg_bus + pd - p_conv;
      x0[idx.i_dpd + i] = std::clamp(dpd_target, xmin[idx.i_dpd + i], xmax[idx.i_dpd + i]);
      // g_Q = Q_calc - Qg + qd - dQd - Q_conv; set dQd so g_Q = 0
      const double dqd_target = q_bus[i] - qg_bus + qd - q_conv;
      x0[idx.i_dqd + i] = std::clamp(dqd_target, xmin[idx.i_dqd + i], xmax[idx.i_dqd + i]);
    }
  }

  // Least-squares feasibility correction: solve Jg'·(Jg·Jg')^{-1}·(-g)
  // to minimally adjust x0 so equality constraints are nearly satisfied.
  {
    EvalWorkspace ws_init;
    Eigen::VectorXd g_init;
    Eigen::SparseMatrix<double> jg_init;
    Eigen::VectorXd x_tmp = x0;
    // Clamp before evaluating (needed for valid Jacobian)
    for (int i = 0; i < idx.n_total; ++i) {
      x_tmp[i] = clamp_interior(x_tmp[i], xmin[i], xmax[i]);
    }
    equality_constraints(prob, x_tmp, ws_init, g_init);
    equality_jacobian(prob, x_tmp, ws_init, jg_init);
    // Sparse normal equations: (Jg·Jg')·v = -g, then dx = Jg'·v
    const Eigen::SparseMatrix<double> JJt_sparse = jg_init * jg_init.transpose();
    Eigen::SparseLU<Eigen::SparseMatrix<double>> splu;
    splu.compute(JJt_sparse);
    if (splu.info() == Eigen::Success) {
      const Eigen::VectorXd v = splu.solve(-g_init);
      if (v.allFinite()) {
        const Eigen::VectorXd dx = jg_init.transpose() * v;
        constexpr double kDamp = 0.8;
        x0 = x_tmp + kDamp * dx;
      } else {
        x0 = x_tmp;
      }
    } else {
      x0 = x_tmp;
    }
  }

  // Clamp to interior of bounds (1% margin matching Julia :clamp_1pct)
  for (int i = 0; i < idx.n_total; ++i) {
    x0[i] = clamp_interior(x0[i], xmin[i], xmax[i]);
  }
}

double objective(const Problem& prob, const Eigen::VectorXd& x) {
  const auto& idx = prob.vidx;
  double f = 0.0;
  for (int k = 0; k < idx.n_pg; ++k) {
    const auto& gen = prob.data.generators[static_cast<size_t>(prob.gen_var_to_data[static_cast<size_t>(k)])];
    const double pg_mw = x[idx.i_pg + k] * prob.data.base_mva;
    f += gen.cost_c2 * pg_mw * pg_mw + gen.cost_c1 * pg_mw + gen.cost_c0;
  }
  if (idx.n_dpd > 0) {
    for (int i = 0; i < idx.n_dpd; ++i) {
      f += prob.voll_effective * x[idx.i_dpd + i] * prob.data.base_mva;
    }
    for (int i = 0; i < idx.n_dqd; ++i) {
      f += prob.voll_effective * x[idx.i_dqd + i] * prob.data.base_mva;
    }
  }
  // Curtailment cost: negative linear term (incentive to produce)
  for (int k = 0; k < idx.n_pren; ++k) {
    if (prob.ren_source[static_cast<size_t>(k)] == 0) {
      const auto& rg = prob.data.renewable_gens[static_cast<size_t>(prob.ren_var_to_data[static_cast<size_t>(k)])];
      const double p_curtailed_mw = rg.p_rated_mw - x[idx.i_pren + k] * prob.data.base_mva;
      if (rg.cost_curtail_mwh > 0.0 && p_curtailed_mw > 0.0)
        f += rg.cost_curtail_mwh * p_curtailed_mw;
    }
  }
  return f;
}

void objective_gradient_hessian_diag(const Problem& prob,
                                     const Eigen::VectorXd& x,
                                     Eigen::VectorXd& grad,
                                     Eigen::VectorXd& hdiag) {
  const auto& idx = prob.vidx;
  grad = Eigen::VectorXd::Zero(idx.n_total);
  hdiag = Eigen::VectorXd::Zero(idx.n_total);
  for (int k = 0; k < idx.n_pg; ++k) {
    const auto& gen = prob.data.generators[static_cast<size_t>(prob.gen_var_to_data[static_cast<size_t>(k)])];
    const int col = idx.i_pg + k;
    const double quad = 2.0 * gen.cost_c2 * prob.data.base_mva * prob.data.base_mva;
    const double lin = gen.cost_c1 * prob.data.base_mva;
    grad[col] = quad * x[col] + lin;
    hdiag[col] = quad;
  }
  if (idx.n_dpd > 0) {
    const double grad_shed = prob.voll_effective * prob.data.base_mva;
    for (int i = 0; i < idx.n_dpd; ++i) {
      grad[idx.i_dpd + i] = grad_shed;
    }
    for (int i = 0; i < idx.n_dqd; ++i) {
      grad[idx.i_dqd + i] = grad_shed;
    }
  }
  // Curtailment cost gradient: ∂f/∂pren = -cost_curtail_mwh * baseMVA
  for (int k = 0; k < idx.n_pren; ++k) {
    if (prob.ren_source[static_cast<size_t>(k)] == 0) {
      const auto& rg = prob.data.renewable_gens[static_cast<size_t>(prob.ren_var_to_data[static_cast<size_t>(k)])];
      if (rg.cost_curtail_mwh > 0.0)
        grad[idx.i_pren + k] = -rg.cost_curtail_mwh * prob.data.base_mva;
    }
  }
}

void equality_constraints(const Problem& prob,
                          const Eigen::VectorXd& x,
                          EvalWorkspace& ws,
                          Eigen::VectorXd& g) {
  const auto& idx = prob.vidx;
  const auto& cidx = prob.cidx;
  const int nb = idx.n_va;
  const int ndc = idx.n_vdc;
  ensure_size(ws.p_calc, nb);
  ensure_size(ws.q_calc, nb);
  ensure_size(ws.p_gen, nb);
  ensure_size(ws.q_gen, nb);
  ensure_size(ws.p_conv_ac, nb);
  ensure_size(ws.q_conv_ac, nb);
  ensure_size(ws.p_conv_dc, ndc);
  ensure_size(ws.p_dc_flow, ndc);
  ensure_size(ws.p_ren, nb);
  ensure_size(ws.q_ren, nb);
  ensure_size(ws.p_stor, nb);
  ensure_size(ws.q_stor, nb);
  ensure_size(ws.p_stor_dc, ndc);
  ensure_size(ws.p_flex, nb);

  const Eigen::Map<const Eigen::VectorXd> va(x.data() + idx.i_va, idx.n_va);
  const Eigen::Map<const Eigen::VectorXd> vm(x.data() + idx.i_vm, idx.n_vm);
  const Eigen::Map<const Eigen::VectorXd> pg(x.data() + idx.i_pg, idx.n_pg);
  const Eigen::Map<const Eigen::VectorXd> qg(x.data() + idx.i_qg, idx.n_qg);
  const Eigen::Map<const Eigen::VectorXd> vdc(x.data() + idx.i_vdc, idx.n_vdc);
  const Eigen::Map<const Eigen::VectorXd> pac(x.data() + idx.i_pac, idx.n_pac);
  const Eigen::Map<const Eigen::VectorXd> qac(x.data() + idx.i_qac, idx.n_qac);
  const Eigen::Map<const Eigen::VectorXd> pdc(x.data() + idx.i_pdc, idx.n_pdc);

  // Initialize P_calc, Q_calc with self-admittance (diagonal) terms
  for (int i = 0; i < nb; ++i) {
    const double vi = vm[i];
    ws.p_calc[i] = vi * vi * prob.g_diag[i];
    ws.q_calc[i] = -vi * vi * prob.b_diag[i];
  }
  // Add off-diagonal contributions by iterating over all nonzero Ybus entries
  for (int col = 0; col < prob.data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(prob.data.ybus, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      if (i == j) {
        continue;
      }
      const double gij = it.value().real();
      const double bij = it.value().imag();
      const double theta = va[i] - va[j];
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      ws.p_calc[i] += vm[i] * vm[j] * (gij * c + bij * s);
      ws.q_calc[i] += vm[i] * vm[j] * (gij * s - bij * c);
    }
  }

  for (int k = 0; k < idx.n_pg; ++k) {
    const int bus = prob.gen_bus[static_cast<size_t>(k)];
    ws.p_gen[bus] += pg[k];
    ws.q_gen[bus] += qg[k];
  }
  for (int k = 0; k < idx.n_pac; ++k) {
    const int ac = prob.conv_ac_bus[static_cast<size_t>(k)];
    const int dc = prob.conv_dc_bus[static_cast<size_t>(k)];
    ws.p_conv_ac[ac] += pac[k];
    ws.q_conv_ac[ac] += qac[k];
    ws.p_conv_dc[dc] += pdc[k];
  }

  // Accumulate enhanced component per-bus injections
  for (int k = 0; k < idx.n_pren; ++k) {
    const int bus = prob.ren_bus[static_cast<size_t>(k)];
    ws.p_ren[bus] += x[idx.i_pren + k];
    ws.q_ren[bus] += x[idx.i_qren + k];
  }
  for (int k = 0; k < idx.n_pstor; ++k) {
    const int bus = prob.stor_bus[static_cast<size_t>(k)];
    ws.p_stor[bus] += x[idx.i_pstor + k];
    ws.q_stor[bus] += x[idx.i_qstor + k];
  }
  for (int k = 0; k < idx.n_pflex; ++k) {
    const int bus = prob.flex_bus[static_cast<size_t>(k)];
    ws.p_flex[bus] += x[idx.i_pflex + k];  // actual demand (positive = consuming)
  }
  for (int k = 0; k < idx.n_pstordc; ++k) {
    const int bus = prob.stor_dc_bus[static_cast<size_t>(k)];
    ws.p_stor_dc[bus] += x[idx.i_pstordc + k];
  }

  g = Eigen::VectorXd::Zero(cidx.n_eq_total);

  for (int i = 0; i < nb; ++i) {
    const double dpd = (idx.n_dpd > 0) ? x[idx.i_dpd + i] : 0.0;
    const double dqd = (idx.n_dqd > 0) ? x[idx.i_dqd + i] : 0.0;
    const double vi = vm[i];
    // Voltage-dependent demand (ZIP model, consistent with PF solver)
    const double pd = prob.pd_demand_pu[i] * (prob.zip_pp[i] + prob.zip_ip[i] * vi + prob.zip_zp[i] * vi * vi);
    const double qd = prob.qd_demand_pu[i] * (prob.zip_pq[i] + prob.zip_iq[i] * vi + prob.zip_zq[i] * vi * vi);
    g[cidx.i_pbal_ac + i] = ws.p_calc[i] - ws.p_gen[i] - prob.p_fixed_inj[i]
                            - ws.p_ren[i] - ws.p_stor[i]
                            + pd - dpd + ws.p_flex[i] - ws.p_conv_ac[i];
    g[cidx.i_qbal_ac + i] = ws.q_calc[i] - ws.q_gen[i] - prob.q_fixed_inj[i]
                            - ws.q_ren[i] - ws.q_stor[i]
                            + qd - dqd - ws.q_conv_ac[i];
    g[cidx.i_pbal_ac + i] *= prob.scale_p;
    g[cidx.i_qbal_ac + i] *= prob.scale_q;
  }

  for (int k = 0; k < ndc; ++k) {
    double pflow = 0.0;
    for (int m = 0; m < ndc; ++m) {
      if (m == k) {
        continue;
      }
      const double gkm = prob.gdc_dense(k, m);
      if (std::abs(gkm) <= 0.0) {
        continue;
      }
      pflow += gkm * (vdc[k] * vdc[k] - vdc[k] * vdc[m]);
    }
    g[cidx.i_pbal_dc + k] = prob.data.dc_buses[static_cast<size_t>(k)].pd_mw / prob.data.base_mva + pflow -
                            ws.p_conv_dc[k];
  }

  // Add DC-side component injections to DC power balance
  // (mirrors residual_evaluator.cpp approach)
  if (!prob.data.dc_loads.empty()) {
    for (const auto& ld : prob.data.dc_loads) {
      if (!ld.in_service) continue;
      const int dc_bus = ld.bus - 1;
      if (dc_bus >= 0 && dc_bus < ndc) {
        g[cidx.i_pbal_dc + dc_bus] += ld.p_mw / prob.data.base_mva;
      }
    }
  }
  for (const auto& sg : prob.data.dc_static_generators) {
    if (!sg.in_service) continue;
    const int dc_bus = sg.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      g[cidx.i_pbal_dc + dc_bus] += sg.p_mw * sg.scaling / prob.data.base_mva;
    }
  }
  for (const auto& pv : prob.data.dc_pv_arrays) {
    if (!pv.in_service) continue;
    const int dc_bus = pv.bus - 1;
    if (dc_bus >= 0 && dc_bus < ndc) {
      g[cidx.i_pbal_dc + dc_bus] -= pv.p_set_mw / prob.data.base_mva;
    }
  }
  for (int k = 0; k < ndc; ++k) {
    g[cidx.i_pbal_dc + k] += ws.p_stor_dc[k];
  }

  // DC-DC converter contributions to DC power balance.
  // Sign convention (正方向): positive p_in = forward transfer (bus_in → bus_out).
  // The OPF variable p_in is INPUT-referenced: the power drawn from bus_in.
  // Relationship to PF variable: p_in = p_ref_mw / η (forward), p_ref_mw * η (reverse).
  // Bidirectional efficiency: forward (p_in >= 0) → p_out = p_in * η
  //                           reverse (p_in <  0) → p_out = p_in / η
  // When r_eq_pu > 0, add I²R conduction loss: P_loss_r = r_eq * p_in² / vdc_in²
  for (int k = 0; k < idx.n_pdcdc; ++k) {
    const auto& dc = prob.data.dcdc_converters[static_cast<size_t>(prob.dcdc_var_to_data[static_cast<size_t>(k)])];
    const double p_in = x[idx.i_pdcdc + k];
    const double eta = std::clamp(dc.eta, 0.01, 1.0);
    const int bin = prob.dcdc_bus_in[static_cast<size_t>(k)];
    const int bout = prob.dcdc_bus_out[static_cast<size_t>(k)];
    // Input bus: power withdrawn (positive p_in = power drawn from input bus)
    g[cidx.i_pbal_dc + bin] += p_in;
    // Output bus: power injected (with direction-dependent efficiency)
    double p_out = (p_in >= 0.0) ? (p_in * eta) : (p_in / eta);
    // Conduction loss from equivalent series resistance (I²R)
    if (dc.r_eq_pu > 0.0) {
      const double vdc_in = std::max(vdc[bin], 0.1);
      const double ploss_r = dc.r_eq_pu * p_in * p_in / (vdc_in * vdc_in);
      p_out -= ploss_r;
    }
    g[cidx.i_pbal_dc + bout] -= p_out;
  }

  const double eps_iac = std::max(prob.options.eps_iac, 1e-12);
  for (int k = 0; k < idx.n_pac; ++k) {
    const auto& conv = prob.data.converters[static_cast<size_t>(prob.conv_var_to_data[static_cast<size_t>(k)])];
    const int ac = prob.conv_ac_bus[static_cast<size_t>(k)];
    const double vm_c = std::max(vm[ac], 1e-8);
    const double p = pac[k];
    const double q = qac[k];
    const double sac = std::sqrt(p * p + q * q + eps_iac);
    const double iac = sac / vm_c;
    const double a = conv.loss_mw / prob.data.base_mva;
    const double b = conv.loss_percent / 100.0;
    const double c = 1.0 - conv.eta;
    const double ploss = a + b * iac + c * iac * iac;
    g[cidx.i_conv_bal + k] = p + pdc[k] + ploss;
  }

  // DC-DC converter power balance (implicit — no separate constraint row).
  // Unlike the VSC which has pac + pdc + ploss = 0 (positive = converter→port),
  // the DC-DC has only 1 variable (p_in).  The output is a deterministic
  // function of it:  p_out = p_in * η − r_eq * p_in² / vdc²,
  // and both are folded into the DC bus power-balance rows above:
  //   bus_in:  += p_in   (power withdrawn from bus_in)
  //   bus_out: -= p_out  (power injected into bus_out)
}

void equality_jacobian(const Problem& prob,
                       const Eigen::VectorXd& x,
                       EvalWorkspace& ws,
                       Eigen::SparseMatrix<double>& jg) {
  const auto& idx = prob.vidx;
  const auto& cidx = prob.cidx;
  const int nb = idx.n_va;
  const int ndc = idx.n_vdc;

  // NOTE: ws.p_calc and ws.q_calc must already be populated by a prior call
  // to equality_constraints(). We no longer re-call it here.

  const Eigen::Map<const Eigen::VectorXd> va(x.data() + idx.i_va, idx.n_va);
  const Eigen::Map<const Eigen::VectorXd> vm(x.data() + idx.i_vm, idx.n_vm);
  const Eigen::Map<const Eigen::VectorXd> vdc(x.data() + idx.i_vdc, idx.n_vdc);
  const Eigen::Map<const Eigen::VectorXd> pac(x.data() + idx.i_pac, idx.n_pac);
  const Eigen::Map<const Eigen::VectorXd> qac(x.data() + idx.i_qac, idx.n_qac);

  std::vector<Eigen::Triplet<double>> t;
  t.reserve(static_cast<size_t>(12 * nb + 24 * prob.data.ybus.nonZeros() + 20 * idx.n_pac +
                                std::max(1, ndc * std::max(1, ndc))));

  for (int i = 0; i < nb; ++i) {
    const int rp = cidx.i_pbal_ac + i;
    const int rq = cidx.i_qbal_ac + i;
    const double bii = prob.b_diag[i];
    const double gii = prob.g_diag[i];
    const double vi = vm[i];
    const double vi_safe = std::max(vi, 1e-8);
    const double dpp_dvai = (-ws.q_calc[i] - bii * vi * vi) * prob.scale_p;
    const double dqq_dvai = (ws.p_calc[i] - gii * vi * vi) * prob.scale_q;
    // ZIP demand derivative: d/dVm [pd*(w0 + w1*V + w2*V^2)] = pd*(w1 + 2*w2*V)
    const double dpd_dvmi = prob.pd_demand_pu[i] * (prob.zip_ip[i] + 2.0 * prob.zip_zp[i] * vi);
    const double dqd_dvmi = prob.qd_demand_pu[i] * (prob.zip_iq[i] + 2.0 * prob.zip_zq[i] * vi);
    const double dpp_dvmi = (ws.p_calc[i] / vi_safe + gii * vi) * prob.scale_p + dpd_dvmi * prob.scale_p;
    const double dqq_dvmi = (ws.q_calc[i] / vi_safe - bii * vi) * prob.scale_q + dqd_dvmi * prob.scale_q;
    t.emplace_back(rp, idx.i_va + i, dpp_dvai);
    t.emplace_back(rq, idx.i_va + i, dqq_dvai);
    t.emplace_back(rp, idx.i_vm + i, dpp_dvmi);
    t.emplace_back(rq, idx.i_vm + i, dqq_dvmi);
  }

  for (int col = 0; col < prob.data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(prob.data.ybus, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      if (i == j) {
        continue;
      }
      const double gij = it.value().real();
      const double bij = it.value().imag();
      const double theta = va[i] - va[j];
      const double c = std::cos(theta);
      const double s = std::sin(theta);

      const double dP_dVa = vm[i] * vm[j] * (gij * s - bij * c);
      const double dQ_dVa = -vm[i] * vm[j] * (gij * c + bij * s);
      const double dP_dVm = vm[i] * (gij * c + bij * s);
      const double dQ_dVm = vm[i] * (gij * s - bij * c);

      const int rp = cidx.i_pbal_ac + i;
      const int rq = cidx.i_qbal_ac + i;
      t.emplace_back(rp, idx.i_va + j, dP_dVa * prob.scale_p);
      t.emplace_back(rq, idx.i_va + j, dQ_dVa * prob.scale_q);
      t.emplace_back(rp, idx.i_vm + j, dP_dVm * prob.scale_p);
      t.emplace_back(rq, idx.i_vm + j, dQ_dVm * prob.scale_q);
    }
  }

  for (int k = 0; k < idx.n_pg; ++k) {
    const int bus = prob.gen_bus[static_cast<size_t>(k)];
    t.emplace_back(cidx.i_pbal_ac + bus, idx.i_pg + k, -prob.scale_p);
    t.emplace_back(cidx.i_qbal_ac + bus, idx.i_qg + k, -prob.scale_q);
  }

  for (int k = 0; k < idx.n_pac; ++k) {
    const int ac = prob.conv_ac_bus[static_cast<size_t>(k)];
    t.emplace_back(cidx.i_pbal_ac + ac, idx.i_pac + k, -prob.scale_p);
    t.emplace_back(cidx.i_qbal_ac + ac, idx.i_qac + k, -prob.scale_q);
  }

  if (idx.n_dpd > 0) {
    for (int i = 0; i < nb; ++i) {
      t.emplace_back(cidx.i_pbal_ac + i, idx.i_dpd + i, -prob.scale_p);
      t.emplace_back(cidx.i_qbal_ac + i, idx.i_dqd + i, -prob.scale_q);
    }
  }

  for (int k = 0; k < ndc; ++k) {
    const int row = cidx.i_pbal_dc + k;
    for (int m = 0; m < ndc; ++m) {
      double v = 0.0;
      if (m == k) {
        for (int mm = 0; mm < ndc; ++mm) {
          if (mm == k) {
            continue;
          }
          const double gkmm = prob.gdc_dense(k, mm);
          if (std::abs(gkmm) <= 0.0) {
            continue;
          }
          v += gkmm * (2.0 * vdc[k] - vdc[mm]);
        }
      } else {
        const double gkm = prob.gdc_dense(k, m);
        if (std::abs(gkm) <= 0.0) {
          continue;
        }
        v = -gkm * vdc[k];
      }
      if (v != 0.0) {
        t.emplace_back(row, idx.i_vdc + m, v);
      }
    }
  }

  for (int k = 0; k < idx.n_pac; ++k) {
    const int row = cidx.i_pbal_dc + prob.conv_dc_bus[static_cast<size_t>(k)];
    t.emplace_back(row, idx.i_pdc + k, -1.0);
  }

  const double eps_iac = std::max(prob.options.eps_iac, 1e-12);
  for (int k = 0; k < idx.n_pac; ++k) {
    const auto& conv = prob.data.converters[static_cast<size_t>(prob.conv_var_to_data[static_cast<size_t>(k)])];
    const int row = cidx.i_conv_bal + k;
    const int ac = prob.conv_ac_bus[static_cast<size_t>(k)];
    const double vm_c = std::max(vm[ac], 1e-8);
    const double p = x[idx.i_pac + k];
    const double q = x[idx.i_qac + k];
    const double sac = std::sqrt(p * p + q * q + eps_iac);
    const double iac = sac / vm_c;
    const double b = conv.loss_percent / 100.0;
    const double c = 1.0 - conv.eta;
    const double dI_dP = p / (sac * vm_c);
    const double dI_dQ = q / (sac * vm_c);
    const double dI_dVm = -sac / (vm_c * vm_c);
    const double dPloss_dP = b * dI_dP + 2.0 * c * iac * dI_dP;
    const double dPloss_dQ = b * dI_dQ + 2.0 * c * iac * dI_dQ;
    const double dPloss_dVm = b * dI_dVm + 2.0 * c * iac * dI_dVm;
    t.emplace_back(row, idx.i_pac + k, 1.0 + dPloss_dP);
    t.emplace_back(row, idx.i_qac + k, dPloss_dQ);
    t.emplace_back(row, idx.i_vm + ac, dPloss_dVm);
    t.emplace_back(row, idx.i_pdc + k, 1.0);
  }

  // Jacobian entries for enhanced DER variables
  for (int k = 0; k < idx.n_pren; ++k) {
    const int bus = prob.ren_bus[static_cast<size_t>(k)];
    t.emplace_back(cidx.i_pbal_ac + bus, idx.i_pren + k, -prob.scale_p);
  }
  for (int k = 0; k < idx.n_qren; ++k) {
    const int bus = prob.ren_bus[static_cast<size_t>(k)];
    t.emplace_back(cidx.i_qbal_ac + bus, idx.i_qren + k, -prob.scale_q);
  }
  for (int k = 0; k < idx.n_pstor; ++k) {
    const int bus = prob.stor_bus[static_cast<size_t>(k)];
    t.emplace_back(cidx.i_pbal_ac + bus, idx.i_pstor + k, -prob.scale_p);
  }
  for (int k = 0; k < idx.n_qstor; ++k) {
    const int bus = prob.stor_bus[static_cast<size_t>(k)];
    t.emplace_back(cidx.i_qbal_ac + bus, idx.i_qstor + k, -prob.scale_q);
  }
  for (int k = 0; k < idx.n_pstordc; ++k) {
    const int bus = prob.stor_dc_bus[static_cast<size_t>(k)];
    t.emplace_back(cidx.i_pbal_dc + bus, idx.i_pstordc + k, 1.0);
  }
  for (int k = 0; k < idx.n_pdcdc; ++k) {
    const auto& dc = prob.data.dcdc_converters[static_cast<size_t>(prob.dcdc_var_to_data[static_cast<size_t>(k)])];
    const double eta = std::clamp(dc.eta, 0.01, 1.0);
    const int bin = prob.dcdc_bus_in[static_cast<size_t>(k)];
    const int bout = prob.dcdc_bus_out[static_cast<size_t>(k)];
    // dg_DC_in/dp_dcdc = +1
    // dg_DC_out/dp_dcdc = -(eta - 2*r_eq*p_in/vdc²) (forward) or -(1/eta - ...) (reverse)
    const double p_in = x[idx.i_pdcdc + k];
    double d_out = (p_in >= 0.0) ? eta : (1.0 / eta);
    t.emplace_back(cidx.i_pbal_dc + bin, idx.i_pdcdc + k, 1.0);
    if (dc.r_eq_pu > 0.0) {
      const double vdc_in = std::max(vdc[bin], 0.1);
      const double vdc2 = vdc_in * vdc_in;
      // d(p_out)/dp_in = d_out - 2*r_eq*p_in/vdc²
      d_out -= 2.0 * dc.r_eq_pu * p_in / vdc2;
      t.emplace_back(cidx.i_pbal_dc + bout, idx.i_pdcdc + k, -d_out);
      // g_DC_out includes +r_eq*p_in²/vdc_in², so increasing input voltage
      // reduces the mismatch contribution from I2R loss.
      const double dg_dvdc = -2.0 * dc.r_eq_pu * p_in * p_in / (vdc2 * vdc_in);
      t.emplace_back(cidx.i_pbal_dc + bout, idx.i_vdc + bin, dg_dvdc);
    } else {
      t.emplace_back(cidx.i_pbal_dc + bout, idx.i_pdcdc + k, -d_out);
    }
  }
  for (int k = 0; k < idx.n_pflex; ++k) {
    const int bus = prob.flex_bus[static_cast<size_t>(k)];
    // Flex load in power balance: +pflex (adds to demand side)
    t.emplace_back(cidx.i_pbal_ac + bus, idx.i_pflex + k, prob.scale_p);
  }

  jg.resize(cidx.n_eq_total, idx.n_total);
  jg.setFromTriplets(t.begin(), t.end());
  jg.makeCompressed();
}

void nonlinear_inequality_constraints(const Problem& prob,
                                      const Eigen::VectorXd& x,
                                      Eigen::VectorXd& h) {
  const auto& idx = prob.vidx;
  const auto& cidx = prob.cidx;
  const Eigen::Map<const Eigen::VectorXd> va(x.data() + idx.i_va, idx.n_va);
  const Eigen::Map<const Eigen::VectorXd> vm(x.data() + idx.i_vm, idx.n_vm);
  const Eigen::Map<const Eigen::VectorXd> pac(x.data() + idx.i_pac, idx.n_pac);
  const Eigen::Map<const Eigen::VectorXd> qac(x.data() + idx.i_qac, idx.n_qac);

  h = Eigen::VectorXd::Zero(cidx.n_ineq_nonlin);
  int off = 0;
  for (int br_idx : prob.branch_limited) {
    const auto& br = prob.data.ac_branches[static_cast<size_t>(br_idx)];
    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    const double tap = (std::abs(br.tap) < 1e-12) ? 1.0 : br.tap;
    const double shift = br.shift_deg * kDegToRad;
    const std::complex<double> ys = 1.0 / std::complex<double>(br.r_pu, br.x_pu);
    const double gs = ys.real();
    const double bs = ys.imag();
    const double bc = br.b_pu * 0.5;
    const double theta = va[i] - va[j] - shift;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double vi = vm[i];
    const double vj = vm[j];
    const double smax = br.rate_a_mva / prob.data.base_mva;
    const double smax2 = smax * smax;

    const double pf = (gs / (tap * tap)) * vi * vi - (gs * c + bs * s) / tap * vi * vj;
    const double qf = -(bs / (tap * tap) + bc) * vi * vi - (gs * s - bs * c) / tap * vi * vj;
    h[off++] = pf * pf + qf * qf - smax2;

    const double pt = gs * vj * vj - (gs * c - bs * s) / tap * vi * vj;
    const double qt = -(bs + bc) * vj * vj + (gs * s + bs * c) / tap * vi * vj;
    h[off++] = pt * pt + qt * qt - smax2;
  }
  for (int k = 0; k < idx.n_pac; ++k) {
    const double smax = converter_smax_pu(prob, prob.conv_var_to_data[static_cast<size_t>(k)]);
    const double p = pac[k];
    const double q = qac[k];
    h[off++] = p * p + q * q - smax * smax;
  }

  // DC branch power flow limits: P_km² ≤ P_max²
  const Eigen::Map<const Eigen::VectorXd> vdc(x.data() + idx.i_vdc, idx.n_vdc);
  for (int br_idx : prob.dc_branch_limited) {
    const auto& br = prob.data.dc_branches[static_cast<size_t>(br_idx)];
    const int k = br.from_bus - 1;
    const int m = br.to_bus - 1;
    const double gkm = (std::abs(br.r_pu) > 1e-14) ? (1.0 / br.r_pu) : 0.0;
    const double pmax = br.rate_a_mva / prob.data.base_mva;
    const double pflow = gkm * vdc[k] * (vdc[k] - vdc[m]);
    h[off++] = pflow * pflow - pmax * pmax;
  }
}

void nonlinear_inequality_jacobian(const Problem& prob,
                                   const Eigen::VectorXd& x,
                                   Eigen::SparseMatrix<double>& jh) {
  const auto& idx = prob.vidx;
  const auto& cidx = prob.cidx;
  const Eigen::Map<const Eigen::VectorXd> va(x.data() + idx.i_va, idx.n_va);
  const Eigen::Map<const Eigen::VectorXd> vm(x.data() + idx.i_vm, idx.n_vm);
  const Eigen::Map<const Eigen::VectorXd> pac(x.data() + idx.i_pac, idx.n_pac);
  const Eigen::Map<const Eigen::VectorXd> qac(x.data() + idx.i_qac, idx.n_qac);

  std::vector<Eigen::Triplet<double>> t;
  t.reserve(static_cast<size_t>(16 * cidx.n_sf + 2 * cidx.n_sconv + 2 * cidx.n_sdc));
  int off = 0;
  for (int br_idx : prob.branch_limited) {
    const auto& br = prob.data.ac_branches[static_cast<size_t>(br_idx)];
    const int i = br.from_bus - 1;
    const int j = br.to_bus - 1;
    const double tap = (std::abs(br.tap) < 1e-12) ? 1.0 : br.tap;
    const double shift = br.shift_deg * kDegToRad;
    const std::complex<double> ys = 1.0 / std::complex<double>(br.r_pu, br.x_pu);
    const double gs = ys.real();
    const double bs = ys.imag();
    const double bc = br.b_pu * 0.5;
    const double theta = va[i] - va[j] - shift;
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double vi = vm[i];
    const double vj = vm[j];
    const double tap2 = tap * tap;

    const double pf = (gs / tap2) * vi * vi - (gs * c + bs * s) / tap * vi * vj;
    const double qf = -(bs / tap2 + bc) * vi * vi - (gs * s - bs * c) / tap * vi * vj;
    const double dpf_dti = (gs * s - bs * c) / tap * vi * vj;
    const double dpf_dtj = -dpf_dti;
    const double dpf_dvi = 2.0 * gs / tap2 * vi - (gs * c + bs * s) / tap * vj;
    const double dpf_dvj = -(gs * c + bs * s) / tap * vi;
    const double dqf_dti = -(gs * c + bs * s) / tap * vi * vj;
    const double dqf_dtj = -dqf_dti;
    const double dqf_dvi = -2.0 * (bs / tap2 + bc) * vi - (gs * s - bs * c) / tap * vj;
    const double dqf_dvj = -(gs * s - bs * c) / tap * vi;

    t.emplace_back(off, idx.i_va + i, 2.0 * pf * dpf_dti + 2.0 * qf * dqf_dti);
    t.emplace_back(off, idx.i_va + j, 2.0 * pf * dpf_dtj + 2.0 * qf * dqf_dtj);
    t.emplace_back(off, idx.i_vm + i, 2.0 * pf * dpf_dvi + 2.0 * qf * dqf_dvi);
    t.emplace_back(off, idx.i_vm + j, 2.0 * pf * dpf_dvj + 2.0 * qf * dqf_dvj);
    ++off;

    const double pt = gs * vj * vj - (gs * c - bs * s) / tap * vi * vj;
    const double qt = -(bs + bc) * vj * vj + (gs * s + bs * c) / tap * vi * vj;
    const double dpt_dti = (gs * s + bs * c) / tap * vi * vj;
    const double dpt_dtj = -dpt_dti;
    const double dpt_dvi = -(gs * c - bs * s) / tap * vj;
    const double dpt_dvj = 2.0 * gs * vj - (gs * c - bs * s) / tap * vi;
    const double dqt_dti = (gs * c - bs * s) / tap * vi * vj;
    const double dqt_dtj = -dqt_dti;
    const double dqt_dvi = (gs * s + bs * c) / tap * vj;
    const double dqt_dvj = -2.0 * (bs + bc) * vj + (gs * s + bs * c) / tap * vi;

    t.emplace_back(off, idx.i_va + i, 2.0 * pt * dpt_dti + 2.0 * qt * dqt_dti);
    t.emplace_back(off, idx.i_va + j, 2.0 * pt * dpt_dtj + 2.0 * qt * dqt_dtj);
    t.emplace_back(off, idx.i_vm + i, 2.0 * pt * dpt_dvi + 2.0 * qt * dqt_dvi);
    t.emplace_back(off, idx.i_vm + j, 2.0 * pt * dpt_dvj + 2.0 * qt * dqt_dvj);
    ++off;
  }

  for (int k = 0; k < idx.n_pac; ++k) {
    t.emplace_back(off, idx.i_pac + k, 2.0 * pac[k]);
    t.emplace_back(off, idx.i_qac + k, 2.0 * qac[k]);
    ++off;
  }

  // DC branch power flow limit Jacobian
  const Eigen::Map<const Eigen::VectorXd> vdc(x.data() + idx.i_vdc, idx.n_vdc);
  for (int br_idx : prob.dc_branch_limited) {
    const auto& br = prob.data.dc_branches[static_cast<size_t>(br_idx)];
    const int k = br.from_bus - 1;
    const int m = br.to_bus - 1;
    const double gkm = (std::abs(br.r_pu) > 1e-14) ? (1.0 / br.r_pu) : 0.0;
    const double pflow = gkm * vdc[k] * (vdc[k] - vdc[m]);
    // dP/dVk = g(2Vk - Vm),  dP/dVm = -g*Vk
    const double dp_dvk = gkm * (2.0 * vdc[k] - vdc[m]);
    const double dp_dvm = -gkm * vdc[k];
    // dh/dVk = 2*P*dP/dVk,  dh/dVm = 2*P*dP/dVm
    t.emplace_back(off, idx.i_vdc + k, 2.0 * pflow * dp_dvk);
    t.emplace_back(off, idx.i_vdc + m, 2.0 * pflow * dp_dvm);
    ++off;
  }

  jh.resize(cidx.n_ineq_nonlin, idx.n_total);
  jh.setFromTriplets(t.begin(), t.end());
  jh.makeCompressed();
}

void lagrangian_hessian_dense(const Problem& prob,
                              const Eigen::VectorXd& x,
                              const Eigen::VectorXd& lambda_eq,
                              const Eigen::VectorXd* nu_ineq,
                              Eigen::MatrixXd& hess) {
  // Delegate to sparse, then convert
  Eigen::SparseMatrix<double> hsp;
  lagrangian_hessian(prob, x, lambda_eq, nu_ineq, hsp, 0.0);
  hess = Eigen::MatrixXd(hsp);
}

void lagrangian_hessian(const Problem& prob,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& lambda_eq,
                        const Eigen::VectorXd* nu_ineq,
                        Eigen::SparseMatrix<double>& hess,
                        double /*drop_tol*/) {
  const auto& idx = prob.vidx;
  const auto& cidx = prob.cidx;
  const int n = idx.n_total;
  const int nb = idx.n_va;
  const int ndc = idx.n_vdc;
  if (x.size() != n) {
    throw std::runtime_error("lagrangian_hessian: x size mismatch.");
  }
  if (lambda_eq.size() != cidx.n_eq_total) {
    throw std::runtime_error("lagrangian_hessian: lambda_eq size mismatch.");
  }
  if (nu_ineq != nullptr && nu_ineq->size() > 0 && nu_ineq->size() != cidx.n_ineq_nonlin) {
    throw std::runtime_error("lagrangian_hessian: nu_ineq size mismatch.");
  }

  // We build a symmetric matrix using only lower-triangular triplets,
  // then symmetrize at the end via selfAdjointView.
  std::vector<Eigen::Triplet<double>> trips;
  const int nnz_ybus = static_cast<int>(prob.data.ybus.nonZeros());
  trips.reserve(static_cast<size_t>(
      idx.n_pg + 4 * nnz_ybus * 4 + ndc * ndc + 6 * idx.n_pac +
      16 * static_cast<int>(prob.branch_limited.size()) + 2 * idx.n_pac));

  // Helper: add value to a (row, col) accumulator stored in triplets.
  // We store both (r,c) and (c,r) to accumulate a full symmetric matrix.
  auto add = [&trips](int r, int c, double v) {
    trips.emplace_back(r, c, v);
  };
  auto add_sym_trip = [&trips](int r, int c, double v) {
    trips.emplace_back(r, c, v);
    if (r != c) {
      trips.emplace_back(c, r, v);
    }
  };

  // ── Objective Hessian (diagonal: Pg cost) ──
  for (int k = 0; k < idx.n_pg; ++k) {
    const auto& gen = prob.data.generators[static_cast<size_t>(prob.gen_var_to_data[static_cast<size_t>(k)])];
    const int col = idx.i_pg + k;
    add(col, col, 2.0 * gen.cost_c2 * prob.data.base_mva * prob.data.base_mva);
  }

  const Eigen::Map<const Eigen::VectorXd> va(x.data() + idx.i_va, idx.n_va);
  const Eigen::Map<const Eigen::VectorXd> vm(x.data() + idx.i_vm, idx.n_vm);
  const Eigen::Map<const Eigen::VectorXd> vdc(x.data() + idx.i_vdc, idx.n_vdc);
  const Eigen::Map<const Eigen::VectorXd> pac(x.data() + idx.i_pac, idx.n_pac);
  const Eigen::Map<const Eigen::VectorXd> qac(x.data() + idx.i_qac, idx.n_qac);

  // ── AC Power Balance Hessian (sparse Ybus iteration) ──
  // For each bus i, we need:
  //   d2P/dθi² = -Σ_{j≠i} vi·vj·t2(i,j)  where t2(i,j)=gij·cos(θij)+bij·sin(θij)
  //   d2Q/dθi² = -Σ_{j≠i} vi·vj·t1(i,j)  where t1(i,j)=gij·sin(θij)-bij·cos(θij)
  // Strategy: iterate over Ybus nonzeros, accumulate diagonal sums, and emit off-diagonal terms.

  // Pre-compute diagonal accumulation vectors
  Eigen::VectorXd sum_vivj_t2(nb);
  Eigen::VectorXd sum_vivj_t1(nb);
  Eigen::VectorXd sum_vj_t1(nb);
  Eigen::VectorXd sum_vj_t2(nb);
  sum_vivj_t2.setZero();
  sum_vivj_t1.setZero();
  sum_vj_t1.setZero();
  sum_vj_t2.setZero();

  // First pass: iterate over off-diagonal Ybus entries to compute all contributions
  for (int col = 0; col < prob.data.ybus.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(prob.data.ybus, col); it; ++it) {
      const int i = static_cast<int>(it.row());
      const int j = static_cast<int>(it.col());
      if (i == j) {
        continue;
      }
      const double gij = it.value().real();
      const double bij = it.value().imag();
      const double theta = va[i] - va[j];
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      const double t2ij = gij * c + bij * s;
      const double t1ij = gij * s - bij * c;
      const double vi = vm[i];
      const double vj = vm[j];
      const double vivj = vi * vj;

      // Accumulate for diagonal terms
      sum_vivj_t2[i] += vivj * t2ij;
      sum_vivj_t1[i] += vivj * t1ij;
      sum_vj_t1[i] += vj * t1ij;
      sum_vj_t2[i] += vj * t2ij;

      const double lambda_p = lambda_eq[cidx.i_pbal_ac + i] * prob.scale_p;
      const double lambda_q = lambda_eq[cidx.i_qbal_ac + i] * prob.scale_q;
      if (std::abs(lambda_p) < 1e-14 && std::abs(lambda_q) < 1e-14) {
        continue;
      }

      const int theta_i = idx.i_va + i;
      const int theta_j = idx.i_va + j;
      const int vm_i = idx.i_vm + i;
      const int vm_j = idx.i_vm + j;

      // d2P/dθj² = -vivj * t2ij (from bus i's constraint)
      add(theta_j, theta_j, lambda_p * (-vivj * t2ij) + lambda_q * (-vivj * t1ij));

      // θi-θj cross term
      add_sym_trip(theta_i, theta_j, lambda_p * (vivj * t2ij) + lambda_q * (vivj * t1ij));

      // Vm_i-Vm_j cross term
      add_sym_trip(vm_i, vm_j, lambda_p * t2ij + lambda_q * t1ij);

      // θj-Vm_i cross
      add_sym_trip(theta_j, vm_i, lambda_p * (vj * t1ij) + lambda_q * (-vj * t2ij));

      // θj-Vm_j cross
      add_sym_trip(theta_j, vm_j, lambda_p * (vi * t1ij) + lambda_q * (-vi * t2ij));

      // θi-Vm_j cross
      add_sym_trip(theta_i, vm_j, lambda_p * (-vi * t1ij) + lambda_q * (vi * t2ij));
    }
  }

  // Emit diagonal terms that depend on accumulated sums
  for (int i = 0; i < nb; ++i) {
    const double lambda_p = lambda_eq[cidx.i_pbal_ac + i] * prob.scale_p;
    const double lambda_q = lambda_eq[cidx.i_qbal_ac + i] * prob.scale_q;
    if (std::abs(lambda_p) < 1e-14 && std::abs(lambda_q) < 1e-14) {
      continue;
    }

    const int theta_i = idx.i_va + i;
    const int vm_i = idx.i_vm + i;

    // d2P/dθi² = -Σ vivj·t2ij, d2Q/dθi² = -Σ vivj·t1ij
    add(theta_i, theta_i,
        lambda_p * (-sum_vivj_t2[i]) + lambda_q * (-sum_vivj_t1[i]));

    // d2P/dVm_i² = 2·gii, d2Q/dVm_i² = -2·bii
    // ZIP demand: d²[pd*(w0+w1*V+w2*V²)]/dV² = pd*2*w2
    const double d2pd_dvm2 = prob.pd_demand_pu[i] * 2.0 * prob.zip_zp[i];
    const double d2qd_dvm2 = prob.qd_demand_pu[i] * 2.0 * prob.zip_zq[i];
    add(vm_i, vm_i,
        lambda_p * (2.0 * prob.g_diag[i] + d2pd_dvm2) +
        lambda_q * (-2.0 * prob.b_diag[i] + d2qd_dvm2));

    // θi-Vm_i mixed
    add_sym_trip(theta_i, vm_i,
                 lambda_p * (-sum_vj_t1[i]) + lambda_q * (sum_vj_t2[i]));
  }

  // ── DC Bus Hessian ──
  for (int k = 0; k < ndc; ++k) {
    const double lambda_k = lambda_eq[cidx.i_pbal_dc + k];
    if (std::abs(lambda_k) < 1e-14) {
      continue;
    }
    const int row = idx.i_vdc + k;
    double d2 = 0.0;
    for (int m = 0; m < ndc; ++m) {
      if (m == k) {
        continue;
      }
      const double gkm = prob.gdc_dense(k, m);
      if (gkm == 0.0) {
        continue;
      }
      d2 += 2.0 * gkm;
      add_sym_trip(row, idx.i_vdc + m, lambda_k * (-gkm));
    }
    add(row, row, lambda_k * d2);
  }

  // ── Converter Loss Hessian ──
  {
    const double eps_iac = std::max(prob.options.eps_iac, 1e-12);
    for (int k = 0; k < idx.n_pac; ++k) {
      const double lambda_c = lambda_eq[cidx.i_conv_bal + k];
      if (std::abs(lambda_c) < 1e-14) {
        continue;
      }
      const auto& conv = prob.data.converters[static_cast<size_t>(prob.conv_var_to_data[static_cast<size_t>(k)])];
      const int ac = prob.conv_ac_bus[static_cast<size_t>(k)];
      const double vm_c = std::max(vm[ac], 1e-8);
      const double p = pac[k];
      const double q = qac[k];
      const double b = conv.loss_percent / 100.0;
      const double c = 1.0 - conv.eta;
      const double sac2 = p * p + q * q + eps_iac;
      const double sac = std::sqrt(sac2);
      const double iac = sac / vm_c;
      const double sac3 = sac * sac2;

      const double dI_dP = p / (sac * vm_c);
      const double dI_dQ = q / (sac * vm_c);
      const double dI_dVm = -sac / (vm_c * vm_c);

      const double d2I_dP2 = (q * q + eps_iac) / (sac3 * vm_c);
      const double d2I_dQ2 = (p * p + eps_iac) / (sac3 * vm_c);
      const double d2I_dPdQ = -p * q / (sac3 * vm_c);
      const double d2I_dPdVm = -p / (sac * vm_c * vm_c);
      const double d2I_dQdVm = -q / (sac * vm_c * vm_c);
      const double d2I_dVm2 = 2.0 * sac / (vm_c * vm_c * vm_c);

      const double coef = b + 2.0 * c * iac;

      const double d2Ploss_dP2 = 2.0 * c * dI_dP * dI_dP + coef * d2I_dP2;
      const double d2Ploss_dQ2 = 2.0 * c * dI_dQ * dI_dQ + coef * d2I_dQ2;
      const double d2Ploss_dPdQ = 2.0 * c * dI_dP * dI_dQ + coef * d2I_dPdQ;
      const double d2Ploss_dPdVm = 2.0 * c * dI_dP * dI_dVm + coef * d2I_dPdVm;
      const double d2Ploss_dQdVm = 2.0 * c * dI_dQ * dI_dVm + coef * d2I_dQdVm;
      const double d2Ploss_dVm2 = 2.0 * c * dI_dVm * dI_dVm + coef * d2I_dVm2;

      const int i_pac = idx.i_pac + k;
      const int i_qac = idx.i_qac + k;
      const int i_vm = idx.i_vm + ac;

      add(i_pac, i_pac, lambda_c * d2Ploss_dP2);
      add(i_qac, i_qac, lambda_c * d2Ploss_dQ2);
      add(i_vm, i_vm, lambda_c * d2Ploss_dVm2);
      add_sym_trip(i_pac, i_qac, lambda_c * d2Ploss_dPdQ);
      add_sym_trip(i_pac, i_vm, lambda_c * d2Ploss_dPdVm);
      add_sym_trip(i_qac, i_vm, lambda_c * d2Ploss_dQdVm);
    }
  }

  // ── DCDC Converter I²R Loss Hessian ──
  // Only non-zero when r_eq_pu > 0.
  // The I²R term in the DC-out power balance is:
  //   g_out -= -(r_eq * p_in² / vdc²)
  // so ∂²g_out/∂p_in² = 2*r_eq/vdc²  (positive because g_out has a -(- ploss_r) = + ploss_r)
  //    ∂²g_out/(∂p_in ∂vdc) = -4*r_eq*p_in/vdc³
  //    ∂²g_out/∂vdc² = 6*r_eq*p_in²/vdc⁴
  for (int k = 0; k < idx.n_pdcdc; ++k) {
    const auto& dc = prob.data.dcdc_converters[static_cast<size_t>(prob.dcdc_var_to_data[static_cast<size_t>(k)])];
    if (dc.r_eq_pu <= 0.0) continue;
    const int bin = prob.dcdc_bus_in[static_cast<size_t>(k)];
    const int bout = prob.dcdc_bus_out[static_cast<size_t>(k)];
    const double lambda_out = lambda_eq[cidx.i_pbal_dc + bout];
    if (std::abs(lambda_out) < 1e-14) continue;

    const double p_in = x[idx.i_pdcdc + k];
    const double vdc_in = std::max(vdc[bin], 0.1);
    const double vdc2 = vdc_in * vdc_in;
    const double vdc3 = vdc2 * vdc_in;
    const double vdc4 = vdc2 * vdc2;
    const double r = dc.r_eq_pu;

    // g_bout -= p_out, where p_out includes -r*p_in²/vdc²
    // So g_bout += r*p_in²/vdc²  (the loss adds to the mismatch)
    // d²(r*p_in²/vdc²)/dp_in² = 2r/vdc²
    // d²(r*p_in²/vdc²)/(dp_in dvdc) = -4r*p_in/vdc³
    // d²(r*p_in²/vdc²)/dvdc² = 6r*p_in²/vdc⁴
    const int col_p = idx.i_pdcdc + k;
    const int col_v = idx.i_vdc + bin;
    add(col_p, col_p, lambda_out * 2.0 * r / vdc2);
    add_sym_trip(col_p, col_v, lambda_out * (-4.0 * r * p_in / vdc3));
    add(col_v, col_v, lambda_out * 6.0 * r * p_in * p_in / vdc4);
  }

  // ── Inequality Hessian (branch flow limits + converter MVA limits) ──
  if (nu_ineq != nullptr && nu_ineq->size() > 0 && cidx.n_ineq_nonlin > 0) {
    const Eigen::VectorXd& nu = *nu_ineq;
    int off = 0;
    for (int br_idx : prob.branch_limited) {
      const auto& br = prob.data.ac_branches[static_cast<size_t>(br_idx)];
      const int i = br.from_bus - 1;
      const int j = br.to_bus - 1;
      const double tap = (std::abs(br.tap) < 1e-12) ? 1.0 : br.tap;
      const double tap2 = tap * tap;
      const double shift = br.shift_deg * kDegToRad;
      const std::complex<double> ys = 1.0 / std::complex<double>(br.r_pu, br.x_pu);
      const double gs = ys.real();
      const double bs = ys.imag();
      const double bc = br.b_pu * 0.5;
      const double theta = va[i] - va[j] - shift;
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      const double vi = vm[i];
      const double vj = vm[j];
      const double vivj = vi * vj;

      const int i_theta_i = idx.i_va + i;
      const int i_theta_j = idx.i_va + j;
      const int i_vi = idx.i_vm + i;
      const int i_vj = idx.i_vm + j;

      const double nu_f = nu[off++];
      if (std::abs(nu_f) > 1e-14) {
        const double pf = (gs / tap2) * vi * vi - (gs * c + bs * s) / tap * vivj;
        const double qf = -(bs / tap2 + bc) * vi * vi - (gs * s - bs * c) / tap * vivj;

        const double dpf_dti = (gs * s - bs * c) / tap * vivj;
        const double dpf_dtj = -dpf_dti;
        const double dpf_dvi = 2.0 * gs / tap2 * vi - (gs * c + bs * s) / tap * vj;
        const double dpf_dvj = -(gs * c + bs * s) / tap * vi;

        const double dqf_dti = -(gs * c + bs * s) / tap * vivj;
        const double dqf_dtj = -dqf_dti;
        const double dqf_dvi = -2.0 * (bs / tap2 + bc) * vi - (gs * s - bs * c) / tap * vj;
        const double dqf_dvj = -(gs * s - bs * c) / tap * vi;

        const double d2pf_dti2 = (gs * c + bs * s) / tap * vivj;
        const double d2pf_dtidtj = -d2pf_dti2;
        const double d2pf_dtj2 = d2pf_dti2;
        const double d2pf_dtidvi = (gs * s - bs * c) / tap * vj;
        const double d2pf_dtidvj = (gs * s - bs * c) / tap * vi;
        const double d2pf_dtjdvi = -d2pf_dtidvi;
        const double d2pf_dtjdvj = -d2pf_dtidvj;
        const double d2pf_dvi2 = 2.0 * gs / tap2;
        const double d2pf_dvidvj = -(gs * c + bs * s) / tap;
        const double d2pf_dvj2 = 0.0;

        const double d2qf_dti2 = (gs * s - bs * c) / tap * vivj;
        const double d2qf_dtidtj = -d2qf_dti2;
        const double d2qf_dtj2 = d2qf_dti2;
        const double d2qf_dtidvi = -(gs * c + bs * s) / tap * vj;
        const double d2qf_dtidvj = -(gs * c + bs * s) / tap * vi;
        const double d2qf_dtjdvi = -d2qf_dtidvi;
        const double d2qf_dtjdvj = -d2qf_dtidvj;
        const double d2qf_dvi2 = -2.0 * (bs / tap2 + bc);
        const double d2qf_dvidvj = -(gs * s - bs * c) / tap;
        const double d2qf_dvj2 = 0.0;

        add(i_theta_i, i_theta_i,
            2.0 * nu_f * (dpf_dti * dpf_dti + pf * d2pf_dti2 + dqf_dti * dqf_dti + qf * d2qf_dti2));
        add(i_theta_j, i_theta_j,
            2.0 * nu_f * (dpf_dtj * dpf_dtj + pf * d2pf_dtj2 + dqf_dtj * dqf_dtj + qf * d2qf_dtj2));
        add(i_vi, i_vi,
            2.0 * nu_f * (dpf_dvi * dpf_dvi + pf * d2pf_dvi2 + dqf_dvi * dqf_dvi + qf * d2qf_dvi2));
        add(i_vj, i_vj,
            2.0 * nu_f * (dpf_dvj * dpf_dvj + pf * d2pf_dvj2 + dqf_dvj * dqf_dvj + qf * d2qf_dvj2));

        add_sym_trip(i_theta_i, i_theta_j,
            2.0 * nu_f * (dpf_dti * dpf_dtj + pf * d2pf_dtidtj + dqf_dti * dqf_dtj + qf * d2qf_dtidtj));
        add_sym_trip(i_theta_i, i_vi,
            2.0 * nu_f * (dpf_dti * dpf_dvi + pf * d2pf_dtidvi + dqf_dti * dqf_dvi + qf * d2qf_dtidvi));
        add_sym_trip(i_theta_i, i_vj,
            2.0 * nu_f * (dpf_dti * dpf_dvj + pf * d2pf_dtidvj + dqf_dti * dqf_dvj + qf * d2qf_dtidvj));
        add_sym_trip(i_theta_j, i_vi,
            2.0 * nu_f * (dpf_dtj * dpf_dvi + pf * d2pf_dtjdvi + dqf_dtj * dqf_dvi + qf * d2qf_dtjdvi));
        add_sym_trip(i_theta_j, i_vj,
            2.0 * nu_f * (dpf_dtj * dpf_dvj + pf * d2pf_dtjdvj + dqf_dtj * dqf_dvj + qf * d2qf_dtjdvj));
        add_sym_trip(i_vi, i_vj,
            2.0 * nu_f * (dpf_dvi * dpf_dvj + pf * d2pf_dvidvj + dqf_dvi * dqf_dvj + qf * d2qf_dvidvj));
      }

      const double nu_t = nu[off++];
      if (std::abs(nu_t) > 1e-14) {
        const double pt = gs * vj * vj - (gs * c - bs * s) / tap * vivj;
        const double qt = -(bs + bc) * vj * vj + (gs * s + bs * c) / tap * vivj;

        const double dpt_dti = (gs * s + bs * c) / tap * vivj;
        const double dpt_dtj = -dpt_dti;
        const double dpt_dvi = -(gs * c - bs * s) / tap * vj;
        const double dpt_dvj = 2.0 * gs * vj - (gs * c - bs * s) / tap * vi;

        const double dqt_dti = (gs * c - bs * s) / tap * vivj;
        const double dqt_dtj = -dqt_dti;
        const double dqt_dvi = (gs * s + bs * c) / tap * vj;
        const double dqt_dvj = -2.0 * (bs + bc) * vj + (gs * s + bs * c) / tap * vi;

        const double d2pt_dti2 = (gs * c - bs * s) / tap * vivj;
        const double d2pt_dtidtj = -d2pt_dti2;
        const double d2pt_dtj2 = d2pt_dti2;
        const double d2pt_dtidvi = (gs * s + bs * c) / tap * vj;
        const double d2pt_dtidvj = (gs * s + bs * c) / tap * vi;
        const double d2pt_dtjdvi = -d2pt_dtidvi;
        const double d2pt_dtjdvj = -d2pt_dtidvj;
        const double d2pt_dvi2 = 0.0;
        const double d2pt_dvidvj = -(gs * c - bs * s) / tap;
        const double d2pt_dvj2 = 2.0 * gs;

        const double d2qt_dti2 = -(gs * s + bs * c) / tap * vivj;
        const double d2qt_dtidtj = -d2qt_dti2;
        const double d2qt_dtj2 = d2qt_dti2;
        const double d2qt_dtidvi = (gs * c - bs * s) / tap * vj;
        const double d2qt_dtidvj = (gs * c - bs * s) / tap * vi;
        const double d2qt_dtjdvi = -d2qt_dtidvi;
        const double d2qt_dtjdvj = -d2qt_dtidvj;
        const double d2qt_dvi2 = 0.0;
        const double d2qt_dvidvj = (gs * s + bs * c) / tap;
        const double d2qt_dvj2 = -2.0 * (bs + bc);

        add(i_theta_i, i_theta_i,
            2.0 * nu_t * (dpt_dti * dpt_dti + pt * d2pt_dti2 + dqt_dti * dqt_dti + qt * d2qt_dti2));
        add(i_theta_j, i_theta_j,
            2.0 * nu_t * (dpt_dtj * dpt_dtj + pt * d2pt_dtj2 + dqt_dtj * dqt_dtj + qt * d2qt_dtj2));
        add(i_vi, i_vi,
            2.0 * nu_t * (dpt_dvi * dpt_dvi + pt * d2pt_dvi2 + dqt_dvi * dqt_dvi + qt * d2qt_dvi2));
        add(i_vj, i_vj,
            2.0 * nu_t * (dpt_dvj * dpt_dvj + pt * d2pt_dvj2 + dqt_dvj * dqt_dvj + qt * d2qt_dvj2));

        add_sym_trip(i_theta_i, i_theta_j,
            2.0 * nu_t * (dpt_dti * dpt_dtj + pt * d2pt_dtidtj + dqt_dti * dqt_dtj + qt * d2qt_dtidtj));
        add_sym_trip(i_theta_i, i_vi,
            2.0 * nu_t * (dpt_dti * dpt_dvi + pt * d2pt_dtidvi + dqt_dti * dqt_dvi + qt * d2qt_dtidvi));
        add_sym_trip(i_theta_i, i_vj,
            2.0 * nu_t * (dpt_dti * dpt_dvj + pt * d2pt_dtidvj + dqt_dti * dqt_dvj + qt * d2qt_dtidvj));
        add_sym_trip(i_theta_j, i_vi,
            2.0 * nu_t * (dpt_dtj * dpt_dvi + pt * d2pt_dtjdvi + dqt_dtj * dqt_dvi + qt * d2qt_dtjdvi));
        add_sym_trip(i_theta_j, i_vj,
            2.0 * nu_t * (dpt_dtj * dpt_dvj + pt * d2pt_dtjdvj + dqt_dtj * dqt_dvj + qt * d2qt_dtjdvj));
        add_sym_trip(i_vi, i_vj,
            2.0 * nu_t * (dpt_dvi * dpt_dvj + pt * d2pt_dvidvj + dqt_dvi * dqt_dvj + qt * d2qt_dvidvj));
      }
    }

    for (int k = 0; k < idx.n_pac; ++k) {
      const double nu_c = nu[off++];
      if (std::abs(nu_c) < 1e-14) {
        continue;
      }
      add(idx.i_pac + k, idx.i_pac + k, 2.0 * nu_c);
      add(idx.i_qac + k, idx.i_qac + k, 2.0 * nu_c);
    }

    // DC branch power flow limit Hessian
    for (int br_idx : prob.dc_branch_limited) {
      const double nu_dc = nu[off++];
      if (std::abs(nu_dc) < 1e-14) {
        continue;
      }
      const auto& br = prob.data.dc_branches[static_cast<size_t>(br_idx)];
      const int k = br.from_bus - 1;
      const int m = br.to_bus - 1;
      const double gkm = (std::abs(br.r_pu) > 1e-14) ? (1.0 / br.r_pu) : 0.0;
      const double pflow = gkm * vdc[k] * (vdc[k] - vdc[m]);
      const double dp_dvk = gkm * (2.0 * vdc[k] - vdc[m]);
      const double dp_dvm = -gkm * vdc[k];
      // h = P² - Pmax²
      // d²h/dVk² = 2*(dP/dVk)² + 2*P*d²P/dVk² = 2*g²(2Vk-Vm)² + 4*g*P
      add(idx.i_vdc + k, idx.i_vdc + k,
          2.0 * nu_dc * (dp_dvk * dp_dvk + pflow * 2.0 * gkm));
      // d²h/dVm² = 2*(dP/dVm)² = 2*g²*Vk²
      add(idx.i_vdc + m, idx.i_vdc + m,
          2.0 * nu_dc * dp_dvm * dp_dvm);
      // d²h/dVkdVm = 2*dP/dVk*dP/dVm + 2*P*d²P/dVkdVm = 2*g(2Vk-Vm)*(-gVk) + 2P*(-g)
      add_sym_trip(idx.i_vdc + k, idx.i_vdc + m,
          2.0 * nu_dc * (dp_dvk * dp_dvm + pflow * (-gkm)));
    }
  }

  // Assemble sparse matrix from triplets (duplicate entries are summed)
  hess.resize(n, n);
  hess.setFromTriplets(trips.begin(), trips.end());
  hess.makeCompressed();
}

}  // namespace hacdcpf::opf::parity
