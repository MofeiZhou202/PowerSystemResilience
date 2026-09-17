#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <unordered_map>

#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/power_flow/ac_linearized_pf.hpp"
#include "hacdcpf/power_flow/branch_flow.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"
#include "hacdcpf/power_flow/dc_solver.hpp"
#include "hacdcpf/power_flow/fdpf_solver.hpp"
#include "hacdcpf/power_flow/helm_solver.hpp"
#include "hacdcpf/power_flow/homotopy_continuation.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/pf_injection_assembly.hpp"
#include "hacdcpf/power_flow/vsc_limit_ncp.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/power_flow/adaptive_solver.hpp"
#include "hacdcpf/power_flow/distributed_slack_solver.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/validation/validate_system.hpp"

namespace hacdcpf {

struct SolverHandle {
  powerflow::SolverData data;
  powerflow::NewtonSolver newton_solver;
  powerflow::DCSolver dc_solver;
  LossModelType configured_loss_model{LossModelType::Linear};
  std::unordered_map<int, int> original_vsc_bus_ac;
  std::unordered_map<int, int> original_vsc_bus_dc;
  std::unordered_map<int, int> original_lcc_bus_ac;
  std::unordered_map<int, int> original_lcc_bus_dc;
  std::unordered_map<int, std::pair<int, int>> original_dcdc_buses;
  std::vector<ACBranch> authored_ac_branches;
  double authored_base_mva{100.0};
  size_t authored_ac_branch_count{0};
  std::uint64_t system_signature{0};
  double pending_projection_ms{0.0};
  double pending_assembly_ms{0.0};
  int prepared_rebuilds{0};
  int prepared_reuses{0};
  int prepared_numeric_refreshes{0};
  bool report_build_cost{false};
};

namespace {

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t v) {
  return seed ^ (v + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U));
}

std::uint64_t hash_double(double d) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(d), "Unexpected double size.");
  std::memcpy(&bits, &d, sizeof(bits));
  return bits;
}

std::uint64_t hash_system_signature(const HybridPowerSystem& sys, LossModelType loss_model) {
  std::uint64_t h = 0xcbf29ce484222325ULL;
  h = hash_combine(h, static_cast<std::uint64_t>(loss_model));
  h = hash_combine(h, hash_double(sys.base_mva));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.buses.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.branches.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.generators.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.buses.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.branches.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.loads.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.vsc_converters.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.lcc_converters.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.dcdc_converters.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.energy_routers.size()));

  for (const auto& b : sys.ac.buses) {
    h = hash_combine(h, static_cast<std::uint64_t>(b.index));
    h = hash_combine(h, static_cast<std::uint64_t>(b.bus_type));
    h = hash_combine(h, hash_double(b.pd_mw));
    h = hash_combine(h, hash_double(b.qd_mvar));
    h = hash_combine(h, hash_double(b.vm_pu));
    h = hash_combine(h, hash_double(b.va_deg));
    h = hash_combine(h, static_cast<std::uint64_t>(b.in_service));
  }
  for (const auto& br : sys.ac.branches) {
    h = hash_combine(h, static_cast<std::uint64_t>(br.from_bus));
    h = hash_combine(h, static_cast<std::uint64_t>(br.to_bus));
    h = hash_combine(h, hash_double(br.r_pu));
    h = hash_combine(h, hash_double(br.x_pu));
    h = hash_combine(h, hash_double(br.b_pu));
    h = hash_combine(h, hash_double(br.tap));
    h = hash_combine(h, hash_double(br.shift_deg));
    h = hash_combine(h, static_cast<std::uint64_t>(br.in_service));
  }
  for (const auto& g : sys.ac.generators) {
    h = hash_combine(h, static_cast<std::uint64_t>(g.bus));
    h = hash_combine(h, static_cast<std::uint64_t>(g.is_slack));
    h = hash_combine(h, static_cast<std::uint64_t>(g.in_service));
    h = hash_combine(h, hash_double(g.pg_mw));
    h = hash_combine(h, hash_double(g.qg_mvar));
    h = hash_combine(h, hash_double(g.vg_pu));
  }
  for (const auto& b : sys.dc.buses) {
    h = hash_combine(h, static_cast<std::uint64_t>(b.index));
    h = hash_combine(h, static_cast<std::uint64_t>(b.bus_type));
    h = hash_combine(h, hash_double(b.vm_pu));
    h = hash_combine(h, hash_double(b.vmin_pu));
    h = hash_combine(h, hash_double(b.vmax_pu));
    h = hash_combine(h, hash_double(b.pd_mw));
    h = hash_combine(h, static_cast<std::uint64_t>(b.in_service));
  }
  for (const auto& br : sys.dc.branches) {
    h = hash_combine(h, static_cast<std::uint64_t>(br.from_bus));
    h = hash_combine(h, static_cast<std::uint64_t>(br.to_bus));
    h = hash_combine(h, hash_double(br.r_pu));
    h = hash_combine(h, static_cast<std::uint64_t>(br.in_service));
  }
  for (const auto& ld : sys.dc.loads) {
    h = hash_combine(h, static_cast<std::uint64_t>(ld.bus));
    h = hash_combine(h, static_cast<std::uint64_t>(ld.in_service));
    h = hash_combine(h, hash_double(ld.p_mw));
    h = hash_combine(h, hash_double(ld.scaling));
  }
  for (const auto& c : sys.vsc_converters) {
    h = hash_combine(h, static_cast<std::uint64_t>(c.bus_ac));
    h = hash_combine(h, static_cast<std::uint64_t>(c.bus_dc));
    h = hash_combine(h, static_cast<std::uint64_t>(c.control_mode));
    h = hash_combine(h, static_cast<std::uint64_t>(c.in_service));
    h = hash_combine(h, hash_double(c.p_set_mw));
    h = hash_combine(h, hash_double(c.q_set_mvar));
    h = hash_combine(h, hash_double(c.v_dc_set_pu));
    h = hash_combine(h, hash_double(c.v_ac_set_pu));
    h = hash_combine(h, hash_double(c.eta));
    h = hash_combine(h, hash_double(c.loss_percent));
    h = hash_combine(h, hash_double(c.loss_mw));
    h = hash_combine(h, hash_double(c.k_vdc));
    h = hash_combine(h, hash_double(c.pmax_mw));
    h = hash_combine(h, hash_double(c.pmin_mw));
    h = hash_combine(h, hash_double(c.qmax_mvar));
    h = hash_combine(h, hash_double(c.qmin_mvar));
    h = hash_combine(h, hash_double(c.p_rated_mw));
    // Steady-state coupling / feasibility fields (multi-converter model): these
    // affect the DC injection (r_conv_ac_pu) or post-solve feasibility checks,
    // so they must invalidate the cached SolverData when changed.
    h = hash_combine(h, hash_double(c.r_conv_ac_pu));
    h = hash_combine(h, hash_double(c.i_ac_max_pu));
    h = hash_combine(h, static_cast<std::uint64_t>(c.enable_limit_ncp));
    h = hash_combine(
        h, static_cast<std::uint64_t>(c.current_limit_priority));
    h = hash_combine(h, hash_double(c.droop_p_min_mw));
    h = hash_combine(h, hash_double(c.droop_p_max_mw));
    h = hash_combine(h, hash_double(c.gfm_internal_voltage_set_pu));
    h = hash_combine(h, hash_double(c.gfm_internal_angle_set_deg));
    h = hash_combine(h, hash_double(c.gfm_virtual_r_pu));
    h = hash_combine(h, hash_double(c.gfm_virtual_x_pu));
    h = hash_combine(h, hash_double(c.i_dc_max_pu));
    h = hash_combine(h, hash_double(c.k_m_modulation));
    h = hash_combine(h, hash_double(c.m_min));
    h = hash_combine(h, hash_double(c.m_max));
  }
  // LCC quasi-steady stations: every field that enters the power-flow
  // injections must invalidate the cached SolverData when changed.
  for (const auto& c : sys.lcc_converters) {
    h = hash_combine(h, static_cast<std::uint64_t>(c.ac_bus));
    h = hash_combine(h, static_cast<std::uint64_t>(c.dc_bus));
    h = hash_combine(h, static_cast<std::uint64_t>(c.station_role));
    h = hash_combine(h, static_cast<std::uint64_t>(c.control_mode));
    h = hash_combine(h, static_cast<std::uint64_t>(c.in_service));
    h = hash_combine(h, static_cast<std::uint64_t>(c.n_bridges));
    h = hash_combine(h, hash_double(c.v_drop_v));
    h = hash_combine(h, hash_double(c.rated_current_a));
    h = hash_combine(h, hash_double(c.x_comm_ohm));
    h = hash_combine(h, hash_double(c.rated_dc_kv));
    h = hash_combine(h, hash_double(c.vn_ac_kv));
    h = hash_combine(h, hash_double(c.p_set_mw));
    h = hash_combine(h, hash_double(c.i_set_ka));
    h = hash_combine(h, hash_double(c.alpha_set_deg));
    h = hash_combine(h, hash_double(c.gamma_set_deg));
    h = hash_combine(h, hash_double(c.v_dc_set_kv));
    h = hash_combine(h, static_cast<std::uint64_t>(c.tap_control_modelled));
    h = hash_combine(
        h, static_cast<std::uint64_t>(c.converter_transformer_branch));
    h = hash_combine(h, hash_double(c.transformer_tap_min_pu));
    h = hash_combine(h, hash_double(c.transformer_tap_max_pu));
    h = hash_combine(h,
                     static_cast<std::uint64_t>(c.transformer_tap_steps));
    h = hash_combine(h,
                     static_cast<std::uint64_t>(c.transformer_tap_winding));
  }
  for (const auto& c : sys.dc.dcdc_converters) {
    h = hash_combine(h, static_cast<std::uint64_t>(c.bus_in));
    h = hash_combine(h, static_cast<std::uint64_t>(c.bus_out));
    h = hash_combine(h, static_cast<std::uint64_t>(c.control_mode));
    h = hash_combine(h, static_cast<std::uint64_t>(c.in_service));
    h = hash_combine(h, hash_double(c.p_ref_mw));
    h = hash_combine(h, hash_double(c.v_ref_pu));
    h = hash_combine(h, hash_double(c.eta));
    h = hash_combine(h, hash_double(c.k_droop));
  }
  for (const auto& er : sys.energy_routers) {
    h = hash_combine(h, static_cast<std::uint64_t>(er.index));
    h = hash_combine(h, static_cast<std::uint64_t>(er.in_service));
    h = hash_combine(h, hash_double(er.loss_percent));
    h = hash_combine(h, static_cast<std::uint64_t>(er.ports.size()));
    for (const auto& p : er.ports) {
      h = hash_combine(h, static_cast<std::uint64_t>(p.bus));
      h = hash_combine(h, static_cast<std::uint64_t>(p.port_type));
      h = hash_combine(h, static_cast<std::uint64_t>(p.control_mode));
      h = hash_combine(h, static_cast<std::uint64_t>(p.in_service));
      h = hash_combine(h, hash_double(p.p_mw));
      h = hash_combine(h, hash_double(p.q_mvar));
      h = hash_combine(h, hash_double(p.p_set_mw));
      h = hash_combine(h, hash_double(p.q_set_mvar));
      h = hash_combine(h, hash_double(p.v_set_pu));
    }
  }
  // Hash component tables that affect power flow results.
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.loads.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.flexible_loads.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.asymmetric_loads.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.static_generators.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.renewable_gens.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.pv_systems.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.storage.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.shunts.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.transformers_2w.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.transformers_3w.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.switches.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.circuit_breakers.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.charging_stations.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.chargers.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.external_grids.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.motors.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.storage.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.static_generators.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.dc_static_generators.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.pv_arrays.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.dc_circuit_breakers.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.three_phase_ac.has_value()));
  for (const auto& ld : sys.ac.loads) {
    h = hash_combine(h, static_cast<std::uint64_t>(ld.bus));
    h = hash_combine(h, hash_double(ld.p_mw));
    h = hash_combine(h, hash_double(ld.q_mvar));
    h = hash_combine(h, static_cast<std::uint64_t>(ld.in_service));
  }
  for (const auto& sg : sys.ac.static_generators) {
    h = hash_combine(h, static_cast<std::uint64_t>(sg.bus));
    h = hash_combine(h, hash_double(sg.p_mw));
    h = hash_combine(h, hash_double(sg.scaling));
    h = hash_combine(h, static_cast<std::uint64_t>(sg.in_service));
  }
  for (const auto& st : sys.ac.storage) {
    h = hash_combine(h, static_cast<std::uint64_t>(st.bus));
    h = hash_combine(h, hash_double(st.p_mw));
    h = hash_combine(h, static_cast<std::uint64_t>(st.in_service));
  }
  for (const auto& sh : sys.ac.shunts) {
    h = hash_combine(h, static_cast<std::uint64_t>(sh.bus));
    h = hash_combine(h, hash_double(sh.gs_mw));
    h = hash_combine(h, hash_double(sh.bs_mvar));
    h = hash_combine(h, static_cast<std::uint64_t>(sh.in_service));
    h = hash_combine(h, static_cast<std::uint64_t>(sh.current_step));
  }
  for (const auto& cs : sys.ac.charging_stations) {
    h = hash_combine(h, static_cast<std::uint64_t>(cs.bus));
    h = hash_combine(h, hash_double(cs.p_total_kw));
    h = hash_combine(h, hash_double(cs.q_total_kvar));
    h = hash_combine(h, static_cast<std::uint64_t>(cs.in_service));
  }
  for (const auto& eg : sys.ac.external_grids) {
    h = hash_combine(h, static_cast<std::uint64_t>(eg.bus));
    h = hash_combine(h, hash_double(eg.vm_pu));
    h = hash_combine(h, hash_double(eg.va_deg));
    h = hash_combine(h, static_cast<std::uint64_t>(eg.in_service));
  }
  for (const auto& st : sys.dc.storage) {
    h = hash_combine(h, static_cast<std::uint64_t>(st.bus));
    h = hash_combine(h, hash_double(st.p_mw));
    h = hash_combine(h, static_cast<std::uint64_t>(st.in_service));
  }
  for (const auto& sg : sys.dc.static_generators) {
    h = hash_combine(h, static_cast<std::uint64_t>(sg.bus));
    h = hash_combine(h, hash_double(sg.p_mw));
    h = hash_combine(h, hash_double(sg.scaling));
    h = hash_combine(h, static_cast<std::uint64_t>(sg.in_service));
  }
  for (const auto& pv : sys.ac.pv_systems) {
    h = hash_combine(h, static_cast<std::uint64_t>(pv.bus));
    h = hash_combine(h, hash_double(pv.p_mw));
    h = hash_combine(h, hash_double(pv.q_mvar));
    h = hash_combine(h, static_cast<std::uint64_t>(pv.in_service));
  }
  for (const auto& rg : sys.ac.renewable_gens) {
    h = hash_combine(h, static_cast<std::uint64_t>(rg.bus));
    h = hash_combine(h, hash_double(rg.p_mw));
    h = hash_combine(h, static_cast<std::uint64_t>(rg.in_service));
  }
  // VPP / Microgrid / MobileStorage
  h = hash_combine(h, static_cast<std::uint64_t>(sys.vpps.size()));
  for (const auto& vpp : sys.vpps) {
    h = hash_combine(h, static_cast<std::uint64_t>(vpp.pcc_bus));
    h = hash_combine(h, hash_double(vpp.p_output_mw));
    h = hash_combine(h, hash_double(vpp.q_output_mvar));
    h = hash_combine(h, static_cast<std::uint64_t>(vpp.in_service));
  }
  h = hash_combine(h, static_cast<std::uint64_t>(sys.microgrids.size()));
  for (const auto& mg : sys.microgrids) {
    h = hash_combine(h, static_cast<std::uint64_t>(mg.pcc_bus));
    h = hash_combine(h, hash_double(mg.p_exchange_mw));
    h = hash_combine(h, static_cast<std::uint64_t>(mg.operating_mode));
    h = hash_combine(h, static_cast<std::uint64_t>(mg.in_service));
  }
  h = hash_combine(h, static_cast<std::uint64_t>(sys.mobile_storage.size()));
  for (const auto& ms : sys.mobile_storage) {
    h = hash_combine(h, static_cast<std::uint64_t>(ms.bus));
    h = hash_combine(h, hash_double(ms.p_mw));
    h = hash_combine(h, hash_double(ms.q_mvar));
    h = hash_combine(h, static_cast<std::uint64_t>(ms.status));
    h = hash_combine(h, static_cast<std::uint64_t>(ms.in_service));
  }
  for (const auto& fl : sys.ac.flexible_loads) {
    h = hash_combine(h, static_cast<std::uint64_t>(fl.bus));
    h = hash_combine(h, hash_double(fl.p_mw));
    h = hash_combine(h, hash_double(fl.q_mvar));
    h = hash_combine(h, hash_double(fl.flex_up_mw));
    h = hash_combine(h, hash_double(fl.flex_down_mw));
    h = hash_combine(h, static_cast<std::uint64_t>(fl.in_service));
  }
  for (const auto& al : sys.ac.asymmetric_loads) {
    h = hash_combine(h, static_cast<std::uint64_t>(al.bus));
    h = hash_combine(h, hash_double(al.pa_mw));
    h = hash_combine(h, hash_double(al.pb_mw));
    h = hash_combine(h, hash_double(al.pc_mw));
    h = hash_combine(h, hash_double(al.qa_mvar));
    h = hash_combine(h, hash_double(al.qb_mvar));
    h = hash_combine(h, hash_double(al.qc_mvar));
    h = hash_combine(h, hash_double(al.scaling));
    h = hash_combine(h, static_cast<std::uint64_t>(al.in_service));
  }
  for (const auto& tr : sys.ac.transformers_2w) {
    h = hash_combine(h, static_cast<std::uint64_t>(tr.hv_bus));
    h = hash_combine(h, static_cast<std::uint64_t>(tr.lv_bus));
    h = hash_combine(h, hash_double(tr.sn_mva));
    h = hash_combine(h, hash_double(tr.vk_percent));
    h = hash_combine(h, hash_double(tr.vkr_percent));
    h = hash_combine(h, hash_double(tr.tap_step_percent));
    h = hash_combine(h, static_cast<std::uint64_t>(tr.tap_pos));
    h = hash_combine(h, static_cast<std::uint64_t>(tr.tap_neutral));
    h = hash_combine(h, static_cast<std::uint64_t>(tr.in_service));
  }
  for (const auto& tr : sys.ac.transformers_3w) {
    h = hash_combine(h, static_cast<std::uint64_t>(tr.hv_bus));
    h = hash_combine(h, static_cast<std::uint64_t>(tr.mv_bus));
    h = hash_combine(h, static_cast<std::uint64_t>(tr.lv_bus));
    h = hash_combine(h, hash_double(tr.vk_hv_mv_percent));
    h = hash_combine(h, hash_double(tr.vk_hv_lv_percent));
    h = hash_combine(h, hash_double(tr.vk_mv_lv_percent));
    h = hash_combine(h, hash_double(tr.vkr_hv_mv_percent));
    h = hash_combine(h, hash_double(tr.vkr_hv_lv_percent));
    h = hash_combine(h, hash_double(tr.vkr_mv_lv_percent));
    h = hash_combine(h, static_cast<std::uint64_t>(tr.in_service));
  }
  for (const auto& sw : sys.ac.switches) {
    h = hash_combine(h, static_cast<std::uint64_t>(sw.bus_from));
    h = hash_combine(h, static_cast<std::uint64_t>(sw.bus_to));
    h = hash_combine(h, hash_double(sw.r_contact_ohm));
    h = hash_combine(h, hash_double(sw.z_ohm));
    h = hash_combine(h, static_cast<std::uint64_t>(sw.closed));
    h = hash_combine(h, static_cast<std::uint64_t>(sw.in_service));
  }
  for (const auto& cb : sys.ac.circuit_breakers) {
    h = hash_combine(h, static_cast<std::uint64_t>(cb.bus_from));
    h = hash_combine(h, static_cast<std::uint64_t>(cb.bus_to));
    h = hash_combine(h, hash_double(cb.z_ohm));
    h = hash_combine(h, hash_double(cb.i_rated_ka));
    h = hash_combine(h, static_cast<std::uint64_t>(cb.closed));
    h = hash_combine(h, static_cast<std::uint64_t>(cb.in_service));
  }
  for (const auto& ch : sys.ac.chargers) {
    h = hash_combine(h, static_cast<std::uint64_t>(ch.station_id));
    h = hash_combine(h, hash_double(ch.p_ch_max_kw));
    h = hash_combine(h, hash_double(ch.p_dis_max_kw));
    h = hash_combine(h, hash_double(ch.eta));
    h = hash_combine(h, static_cast<std::uint64_t>(ch.in_service));
  }
  for (const auto& m : sys.ac.motors) {
    h = hash_combine(h, static_cast<std::uint64_t>(m.bus));
    h = hash_combine(h, hash_double(m.sn_mva));
    h = hash_combine(h, static_cast<std::uint64_t>(m.in_service));
  }
  for (const auto& sgdc : sys.dc.dc_static_generators) {
    h = hash_combine(h, static_cast<std::uint64_t>(sgdc.bus));
    h = hash_combine(h, hash_double(sgdc.p_set_mw));
    h = hash_combine(h, hash_double(sgdc.scaling));
    h = hash_combine(h, static_cast<std::uint64_t>(sgdc.in_service));
  }
  for (const auto& pva : sys.dc.pv_arrays) {
    h = hash_combine(h, static_cast<std::uint64_t>(pva.bus));
    h = hash_combine(h, hash_double(pva.p_set_mw));
    h = hash_combine(h, static_cast<std::uint64_t>(pva.in_service));
  }
  for (const auto& cb : sys.dc.dc_circuit_breakers) {
    h = hash_combine(h, static_cast<std::uint64_t>(cb.bus_from));
    h = hash_combine(h, static_cast<std::uint64_t>(cb.bus_to));
    h = hash_combine(h, hash_double(cb.r_ohm));
    h = hash_combine(h, hash_double(cb.i_rated_ka));
    h = hash_combine(h, static_cast<std::uint64_t>(cb.closed));
    h = hash_combine(h, static_cast<std::uint64_t>(cb.in_service));
  }
  if (sys.three_phase_ac.has_value()) {
    const auto& tp = sys.three_phase_ac.value();
    h = hash_combine(h, static_cast<std::uint64_t>(tp.buses.size()));
    h = hash_combine(h, static_cast<std::uint64_t>(tp.lines.size()));
    h = hash_combine(h, static_cast<std::uint64_t>(tp.transformers.size()));
    h = hash_combine(h, static_cast<std::uint64_t>(tp.loads.size()));
    h = hash_combine(h, static_cast<std::uint64_t>(tp.generators.size()));
    h = hash_combine(h, static_cast<std::uint64_t>(tp.external_grids.size()));
  }
  return h;
}

struct SolverDataCache {
  struct StorageView {
    const ACBus* ac_buses{nullptr};
    size_t ac_buses_size{0};
    const ACBranch* ac_branches{nullptr};
    size_t ac_branches_size{0};
    const Generator* ac_generators{nullptr};
    size_t ac_generators_size{0};
    const DCBus* dc_buses{nullptr};
    size_t dc_buses_size{0};
    const DCBranch* dc_branches{nullptr};
    size_t dc_branches_size{0};
    const VSCConverter* vsc_converters{nullptr};
    size_t vsc_converters_size{0};
  };

  bool valid{false};
  const HybridPowerSystem* system_ptr{nullptr};
  LossModelType loss_model{LossModelType::Linear};
  std::uint64_t signature{0};
  StorageView storage;
  powerflow::SolverData data;
};

SolverDataCache::StorageView capture_storage_view(const HybridPowerSystem& sys) {
  SolverDataCache::StorageView s;
  s.ac_buses = sys.ac.buses.data();
  s.ac_buses_size = sys.ac.buses.size();
  s.ac_branches = sys.ac.branches.data();
  s.ac_branches_size = sys.ac.branches.size();
  s.ac_generators = sys.ac.generators.data();
  s.ac_generators_size = sys.ac.generators.size();
  s.dc_buses = sys.dc.buses.data();
  s.dc_buses_size = sys.dc.buses.size();
  s.dc_branches = sys.dc.branches.data();
  s.dc_branches_size = sys.dc.branches.size();
  s.vsc_converters = sys.vsc_converters.data();
  s.vsc_converters_size = sys.vsc_converters.size();
  return s;
}

bool same_storage_view(const SolverDataCache::StorageView& cached, const HybridPowerSystem& sys) {
  return cached.ac_buses == sys.ac.buses.data() && cached.ac_buses_size == sys.ac.buses.size() &&
         cached.ac_branches == sys.ac.branches.data() &&
         cached.ac_branches_size == sys.ac.branches.size() &&
         cached.ac_generators == sys.ac.generators.data() &&
         cached.ac_generators_size == sys.ac.generators.size() &&
         cached.dc_buses == sys.dc.buses.data() && cached.dc_buses_size == sys.dc.buses.size() &&
         cached.dc_branches == sys.dc.branches.data() &&
         cached.dc_branches_size == sys.dc.branches.size() &&
         cached.vsc_converters == sys.vsc_converters.data() &&
         cached.vsc_converters_size == sys.vsc_converters.size();
}

powerflow::SolverData& get_cached_solver_data(const HybridPowerSystem& sys,
                                               LossModelType loss_model,
                                               double* projection_ms = nullptr,
                                               double* assembly_ms = nullptr) {
  thread_local SolverDataCache cache;
  // The facade accepts a mutable rich model by const reference. A handwritten
  // field hash cannot prove that every assembly-relevant field is unchanged,
  // so implicit reuse risks solving stale Ybus/injection data. Performance
  // callers use the explicit SolverHandle lifecycle instead.
  const auto projection_start = std::chrono::steady_clock::now();
  HybridPowerSystem projected =
      projection::RichToCanonicalOperator::apply(sys).canonical;
  const auto projection_end = std::chrono::steady_clock::now();
  cache.data =
      powerflow::make_solver_data_projected(std::move(projected), loss_model);
  const auto assembly_end = std::chrono::steady_clock::now();
  if (projection_ms != nullptr) {
    *projection_ms = std::chrono::duration<double, std::milli>(
                         projection_end - projection_start)
                         .count();
  }
  if (assembly_ms != nullptr) {
    *assembly_ms = std::chrono::duration<double, std::milli>(
                       assembly_end - projection_end)
                       .count();
  }
  cache.signature = hash_system_signature(sys, loss_model);
  cache.loss_model = loss_model;
  cache.valid = true;
  cache.system_ptr = &sys;
  cache.storage = capture_storage_view(sys);
  return cache.data;
}

std::unordered_map<int, int> build_original_vsc_ac_bus_map(
    const HybridPowerSystem& sys);
std::unordered_map<int, int> build_original_lcc_ac_bus_map(
    const HybridPowerSystem& sys);

void rebuild_handle_data(SolverHandle& handle,
                         const HybridPowerSystem& sys,
                         LossModelType loss_model) {
  handle.original_vsc_bus_ac = build_original_vsc_ac_bus_map(sys);
  handle.original_vsc_bus_dc.clear();
  handle.original_lcc_bus_ac = build_original_lcc_ac_bus_map(sys);
  handle.original_lcc_bus_dc.clear();
  handle.original_dcdc_buses.clear();
  for (const auto& converter : sys.vsc_converters) {
    handle.original_vsc_bus_dc[converter.index] = converter.bus_dc;
  }
  for (const auto& converter : sys.lcc_converters) {
    handle.original_lcc_bus_dc[converter.index] = converter.dc_bus;
  }
  for (const auto& converter : sys.dc.dcdc_converters) {
    handle.original_dcdc_buses[converter.index] =
        {converter.bus_in, converter.bus_out};
  }
  handle.authored_ac_branch_count = sys.ac.branches.size();
  handle.authored_ac_branches = sys.ac.branches;
  handle.authored_base_mva =
      sys.base_mva > 0.0
          ? sys.base_mva
          : (sys.ac.base_mva > 0.0 ? sys.ac.base_mva : 100.0);
  const auto projection_start = std::chrono::steady_clock::now();
  HybridPowerSystem projected =
      projection::RichToCanonicalOperator::apply(sys).canonical;
  const auto projection_end = std::chrono::steady_clock::now();
  handle.data =
      powerflow::make_solver_data_projected(std::move(projected), loss_model);
  const auto assembly_end = std::chrono::steady_clock::now();
  handle.pending_projection_ms =
      std::chrono::duration<double, std::milli>(projection_end - projection_start)
          .count();
  handle.pending_assembly_ms =
      std::chrono::duration<double, std::milli>(assembly_end - projection_end)
          .count();
  handle.report_build_cost = true;
  handle.system_signature = hash_system_signature(sys, loss_model);
  handle.prepared_rebuilds += 1;
  handle.configured_loss_model = loss_model;
}

std::unordered_map<int, int> build_original_vsc_ac_bus_map(
    const HybridPowerSystem& sys) {
  std::unordered_map<int, int> bus_by_index;
  bus_by_index.reserve(sys.vsc_converters.size() + sys.energy_routers.size() * 4);
  for (const auto& conv : sys.vsc_converters) {
    bus_by_index[conv.index] = conv.bus_ac;
  }
  if (sys.energy_routers.empty()) return bus_by_index;

  try {
    const HybridPowerSystem projected =
        projection::RichToCanonicalOperator::apply(sys).canonical;
    std::unordered_map<std::string, int> vsc_index_by_name;
    vsc_index_by_name.reserve(projected.vsc_converters.size());
    for (const auto& conv : projected.vsc_converters) {
      vsc_index_by_name[conv.name] = conv.index;
    }
    for (const auto& er : sys.energy_routers) {
      if (!er.in_service) continue;
      for (const auto& port : er.ports) {
        if (!port.in_service || port.bus == 0 ||
            port.port_type != ERPortType::AC) {
          continue;
        }
        const char side = (port.side == 0) ? 'A' : 'B';
        const std::string vsc_name =
            er.name + "_VSC_" + side + std::to_string(port.index);
        const auto it = vsc_index_by_name.find(vsc_name);
        if (it != vsc_index_by_name.end()) {
          bus_by_index[it->second] = port.bus;
        }
      }
    }
  } catch (...) {
    // Result attribution should not make a converged PF fail. Existing VSC
    // converters are still restored by index; expanded ER ports fall back to
    // the canonical bus if their reconstruction metadata is unavailable.
  }
  return bus_by_index;
}

std::unordered_map<int, int> build_original_lcc_ac_bus_map(
    const HybridPowerSystem& sys) {
  std::unordered_map<int, int> bus_by_index;
  bus_by_index.reserve(sys.lcc_converters.size());
  for (const auto& conv : sys.lcc_converters) {
    bus_by_index[conv.index] = conv.ac_bus;
  }
  return bus_by_index;
}

void apply_zip_weights(powerflow::SolverData& data, const PowerFlowOptions& opt) {
  for (int k = 0; k < 3; ++k) {
    data.zip_pw[k] = opt.zip_pw[k];
    data.zip_qw[k] = opt.zip_qw[k];
  }
  // Propagate exact fully-coupled Newton flags.
  data.enable_coupled_jacobian = opt.enable_coupled_jacobian;
  data.enable_augmented_equations = opt.enable_augmented_equations;
  data.enable_semi_smooth_newton = opt.enable_semi_smooth_newton;
}

void merge_solver_profiling(SolverProfiling& total,
                            const SolverProfiling& attempt) {
  if (!attempt.linear_solver_backend.empty()) {
    total.linear_solver_backend = attempt.linear_solver_backend;
  }
  total.ac_eval_threads = std::max(total.ac_eval_threads,
                                   attempt.ac_eval_threads);
  total.jacobian_pattern_rebuilds += attempt.jacobian_pattern_rebuilds;
  total.jacobian_analyze_calls += attempt.jacobian_analyze_calls;
  total.factorization_calls += attempt.factorization_calls;
  total.numeric_refactor_attempts += attempt.numeric_refactor_attempts;
  total.numeric_refactor_accepted += attempt.numeric_refactor_accepted;
  total.numeric_refactor_fallbacks += attempt.numeric_refactor_fallbacks;
  total.max_refactor_backward_error = std::max(
      total.max_refactor_backward_error,
      attempt.max_refactor_backward_error);
  total.vsc_schur_attempts += attempt.vsc_schur_attempts;
  total.vsc_schur_accepted += attempt.vsc_schur_accepted;
  total.vsc_schur_fallbacks += attempt.vsc_schur_fallbacks;
  total.vsc_schur_local_factorizations +=
      attempt.vsc_schur_local_factorizations;
  total.vsc_schur_sparse_factorizations +=
      attempt.vsc_schur_sparse_factorizations;
  total.vsc_schur_numeric_refactor_attempts +=
      attempt.vsc_schur_numeric_refactor_attempts;
  total.vsc_schur_numeric_refactor_accepted +=
      attempt.vsc_schur_numeric_refactor_accepted;
  total.vsc_schur_local_regular_rejections +=
      attempt.vsc_schur_local_regular_rejections;
  total.vsc_schur_reduced_solve_rejections +=
      attempt.vsc_schur_reduced_solve_rejections;
  total.vsc_schur_full_backward_error_rejections +=
      attempt.vsc_schur_full_backward_error_rejections;
  total.vsc_schur_local_blocks = std::max(
      total.vsc_schur_local_blocks, attempt.vsc_schur_local_blocks);
  total.vsc_schur_full_dimension = std::max(
      total.vsc_schur_full_dimension, attempt.vsc_schur_full_dimension);
  total.vsc_schur_reduced_dimension = std::max(
      total.vsc_schur_reduced_dimension, attempt.vsc_schur_reduced_dimension);
  total.vsc_schur_full_structural_nnz = std::max(
      total.vsc_schur_full_structural_nnz,
      attempt.vsc_schur_full_structural_nnz);
  total.vsc_schur_reduced_structural_nnz = std::max(
      total.vsc_schur_reduced_structural_nnz,
      attempt.vsc_schur_reduced_structural_nnz);
  if (attempt.vsc_schur_reduced_factor_nonzeros >= 0) {
    total.vsc_schur_reduced_factor_nonzeros =
        attempt.vsc_schur_reduced_factor_nonzeros;
  }
  if (attempt.vsc_schur_reduced_factor_work >= 0) {
    total.vsc_schur_reduced_factor_work =
        attempt.vsc_schur_reduced_factor_work;
  }
  if (attempt.full_lu_factor_nonzeros >= 0) {
    total.full_lu_factor_nonzeros = attempt.full_lu_factor_nonzeros;
  }
  if (attempt.full_lu_factor_work >= 0) {
    total.full_lu_factor_work = attempt.full_lu_factor_work;
  }
  if (attempt.vsc_schur_minimum_local_rcond > 0.0) {
    total.vsc_schur_minimum_local_rcond =
        total.vsc_schur_minimum_local_rcond > 0.0
            ? std::min(total.vsc_schur_minimum_local_rcond,
                       attempt.vsc_schur_minimum_local_rcond)
            : attempt.vsc_schur_minimum_local_rcond;
  }
  if (attempt.vsc_schur_minimum_accepted_local_rcond > 0.0) {
    total.vsc_schur_minimum_accepted_local_rcond =
        total.vsc_schur_minimum_accepted_local_rcond > 0.0
            ? std::min(total.vsc_schur_minimum_accepted_local_rcond,
                       attempt.vsc_schur_minimum_accepted_local_rcond)
            : attempt.vsc_schur_minimum_accepted_local_rcond;
  }
  total.max_vsc_schur_reduced_backward_error = std::max(
      total.max_vsc_schur_reduced_backward_error,
      attempt.max_vsc_schur_reduced_backward_error);
  total.max_vsc_schur_full_backward_error = std::max(
      total.max_vsc_schur_full_backward_error,
      attempt.max_vsc_schur_full_backward_error);
  total.vsc_schur_assembly_factor_ms_total +=
      attempt.vsc_schur_assembly_factor_ms_total;
  total.semismooth_rate_samples += attempt.semismooth_rate_samples;
  if (attempt.semismooth_rate_samples > 0) {
    total.semismooth_last_residual_ratio =
        attempt.semismooth_last_residual_ratio;
    total.semismooth_last_quadratic_ratio =
        attempt.semismooth_last_quadratic_ratio;
  }
  if (attempt.vsc_schur_status != "not_attempted") {
    total.vsc_schur_status = attempt.vsc_schur_status;
  }
  total.linear_solve_calls += attempt.linear_solve_calls;
  total.regularization_attempts += attempt.regularization_attempts;
  total.line_search_evaluations += attempt.line_search_evaluations;
  total.rejected_steps += attempt.rejected_steps;
  total.pv_to_pq_switches += attempt.pv_to_pq_switches;
  total.pq_to_pv_switches += attempt.pq_to_pv_switches;
  total.pv_pq_outer_iterations += attempt.pv_pq_outer_iterations;
  total.pv_pq_repeated_active_sets += attempt.pv_pq_repeated_active_sets;
  total.smooth_ncp_continuation_updates +=
      attempt.smooth_ncp_continuation_updates;
  total.smooth_ncp_final_mu = attempt.smooth_ncp_final_mu;
  total.converter_mode_switches += attempt.converter_mode_switches;
  total.residual_by_iter.insert(total.residual_by_iter.end(),
                                attempt.residual_by_iter.begin(),
                                attempt.residual_by_iter.end());
  total.line_search_evals_by_iter.insert(
      total.line_search_evals_by_iter.end(),
      attempt.line_search_evals_by_iter.begin(),
      attempt.line_search_evals_by_iter.end());
  total.eval_jacobian_ms_by_iter.insert(
      total.eval_jacobian_ms_by_iter.end(),
      attempt.eval_jacobian_ms_by_iter.begin(),
      attempt.eval_jacobian_ms_by_iter.end());
  total.linear_solve_ms_by_iter.insert(
      total.linear_solve_ms_by_iter.end(),
      attempt.linear_solve_ms_by_iter.begin(),
      attempt.linear_solve_ms_by_iter.end());
  total.line_search_ms_by_iter.insert(
      total.line_search_ms_by_iter.end(),
      attempt.line_search_ms_by_iter.begin(),
      attempt.line_search_ms_by_iter.end());
  total.eval_jacobian_ms_total += attempt.eval_jacobian_ms_total;
  total.linear_solve_ms_total += attempt.linear_solve_ms_total;
  total.line_search_ms_total += attempt.line_search_ms_total;
  total.residual_evaluation_ms_total +=
      attempt.residual_evaluation_ms_total;
  total.scaling_ms_total += attempt.scaling_ms_total;
  total.active_set_scan_ms_total += attempt.active_set_scan_ms_total;
  total.projection_ms_total += attempt.projection_ms_total;
  total.assembly_ms_total += attempt.assembly_ms_total;
  total.result_derivation_ms_total += attempt.result_derivation_ms_total;
  total.solver_core_ms_total += attempt.solver_core_ms_total;
  total.unclassified_core_ms_total += attempt.unclassified_core_ms_total;
  total.facade_ms_total += attempt.facade_ms_total;
  total.unclassified_facade_ms_total += attempt.unclassified_facade_ms_total;
  total.prepared_session_rebuilds += attempt.prepared_session_rebuilds;
  total.prepared_session_reuses += attempt.prepared_session_reuses;
  total.prepared_session_numeric_refreshes +=
      attempt.prepared_session_numeric_refreshes;
  total.raw_residual_norm = attempt.raw_residual_norm;
  total.scaled_residual_norm = attempt.scaled_residual_norm;
  total.condition_estimate = attempt.condition_estimate;
  total.regularization_count += attempt.regularization_count;
  total.linear_solver_status = attempt.linear_solver_status;
  if (attempt.stagnation_detected && !total.stagnation_detected) {
    total.stagnation_exit_iteration = attempt.stagnation_exit_iteration;
  }
  total.stagnation_detected =
      total.stagnation_detected || attempt.stagnation_detected;
  total.homotopy_fallback_attempted =
      total.homotopy_fallback_attempted ||
      attempt.homotopy_fallback_attempted;
  total.homotopy_fallback_succeeded =
      total.homotopy_fallback_succeeded ||
      attempt.homotopy_fallback_succeeded;
  total.nonlinear_escalation_attempts +=
      attempt.nonlinear_escalation_attempts;
  total.ncp_fallback_attempted =
      total.ncp_fallback_attempted || attempt.ncp_fallback_attempted;
  total.dc_angle_seed_attempted =
      total.dc_angle_seed_attempted || attempt.dc_angle_seed_attempted;
  if (!attempt.successful_fallback_stage.empty()) {
    total.successful_fallback_stage = attempt.successful_fallback_stage;
  }
}

bool eligible_for_nonlinear_escalation(const PowerFlowResult& result) {
  return !result.converged && result.iterations > 0 &&
         (!result.diagnostics.equation_closure_checked ||
          result.diagnostics.equation_closure_ok);
}

InitialState make_dc_angle_initial_state(
    const powerflow::SolverData& data,
    const powerflow::ACLinearizedDCResult& dc_seed) {
  InitialState initial;
  initial.va = dc_seed.va;
  initial.vm.reserve(data.ac_buses.size());
  for (const auto& bus : data.ac_buses) {
    initial.vm.push_back(bus.vm_pu);
  }
  initial.vdc.reserve(data.dc_buses.size());
  for (const auto& bus : data.dc_buses) {
    initial.vdc.push_back(bus.vm_pu);
  }
  return initial;
}

PowerFlowResult solve_isolated_newton_attempt(
    const powerflow::SolverData& base_data,
    PowerFlowOptions attempt_options,
    const InitialState* initial_state,
    bool enable_ncp) {
  powerflow::SolverData attempt_data = base_data;
  attempt_options.enable_semi_smooth_newton = enable_ncp;
  attempt_options.robust_nonlinear.enable_fixed_pv_pq_layout = !enable_ncp;
  attempt_options.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
  attempt_data.enable_semi_smooth_newton = enable_ncp;
  powerflow::NewtonSolver attempt_solver;
  return attempt_solver.solve(attempt_data, attempt_options, initial_state);
}

struct LCCTapControlStatus {
  bool active{false};
  bool converged{false};
  bool at_limit{false};
  bool outer_solve_limit_reached{false};
  bool update_blocked{false};
  int accepted_changes{0};
  double target_angle_deg{0.0};
  double final_angle_deg{0.0};
  double target_dc_voltage_kv{0.0};
  double final_dc_voltage_kv{0.0};
  double final_tap{1.0};
};

using LCCTapControlStatusMap =
    std::unordered_map<int, LCCTapControlStatus>;

ACBranch* find_ac_branch_by_index(powerflow::SolverData& data, int index) {
  const auto it = std::find_if(
      data.ac_branches.begin(), data.ac_branches.end(),
      [index](const ACBranch& branch) { return branch.index == index; });
  return it == data.ac_branches.end() ? nullptr : &*it;
}

const ACBranch* find_ac_branch_by_index(const powerflow::SolverData& data,
                                        int index) {
  const auto it = std::find_if(
      data.ac_branches.begin(), data.ac_branches.end(),
      [index](const ACBranch& branch) { return branch.index == index; });
  return it == data.ac_branches.end() ? nullptr : &*it;
}

bool solver_data_requests_lcc_tap_control(
    const powerflow::SolverData& data) {
  return std::any_of(
      data.lcc_converters.begin(), data.lcc_converters.end(),
      [](const LCCConverter& lcc) {
        return lcc.in_service && lcc.tap_control_modelled;
      });
}

double lcc_tap_target_angle_deg(const LCCConverter& lcc) {
  return lcc.station_role == LCCStationRole::Rectifier
             ? lcc.alpha_set_deg
             : lcc.gamma_set_deg;
}

double next_lcc_transformer_tap(const LCCConverter& lcc,
                                const ACBranch& branch,
                                double actual_commutation_kv,
                                double required_commutation_kv) {
  if (!(branch.tap > 0.0) || !(actual_commutation_kv > 0.0) ||
      !(required_commutation_kv > 0.0)) {
    return branch.tap;
  }

  // ACBranch places its ideal off-nominal tap at the from terminal:
  // V_from/tap ~= V_to. This local response update is corrected after every
  // full Newton solve, so transformer leakage and AC-network feedback remain
  // in the closed loop instead of being discarded by an import-time guess.
  double requested = branch.tap;
  if (branch.to_bus == lcc.ac_bus) {
    requested *= actual_commutation_kv / required_commutation_kv;
  } else if (branch.from_bus == lcc.ac_bus) {
    requested *= required_commutation_kv / actual_commutation_kv;
  } else {
    return branch.tap;
  }
  requested = std::clamp(requested, lcc.transformer_tap_min_pu,
                         lcc.transformer_tap_max_pu);

  if (lcc.transformer_tap_steps > 1) {
    const double step =
        (lcc.transformer_tap_max_pu - lcc.transformer_tap_min_pu) /
        static_cast<double>(lcc.transformer_tap_steps - 1);
    if (step > 0.0) {
      const int last = lcc.transformer_tap_steps - 1;
      const int current_pos = std::clamp(
          static_cast<int>(std::lround(
              (branch.tap - lcc.transformer_tap_min_pu) / step)),
          0, last);
      const int target_pos = std::clamp(
          static_cast<int>(std::lround(
              (requested - lcc.transformer_tap_min_pu) / step)),
          0, last);
      const int next_pos = current_pos +
                           (target_pos > current_pos ? 1
                            : target_pos < current_pos ? -1
                                                       : 0);
      return lcc.transformer_tap_min_pu +
             static_cast<double>(next_pos) * step;
    }
  }

  constexpr double kTapDamping = 0.65;
  return std::clamp(
      branch.tap + kTapDamping * (requested - branch.tap),
      lcc.transformer_tap_min_pu, lcc.transformer_tap_max_pu);
}

double normalize_lcc_transformer_tap(const LCCConverter& lcc,
                                     double tap) {
  const double positive_tap = std::isfinite(tap) && tap > 0.0 ? tap : 1.0;
  double normalized = std::clamp(positive_tap,
                                 lcc.transformer_tap_min_pu,
                                 lcc.transformer_tap_max_pu);
  if (lcc.transformer_tap_steps > 1) {
    const double step =
        (lcc.transformer_tap_max_pu - lcc.transformer_tap_min_pu) /
        static_cast<double>(lcc.transformer_tap_steps - 1);
    if (step > 0.0) {
      const int last = lcc.transformer_tap_steps - 1;
      const int position = std::clamp(
          static_cast<int>(std::lround(
              (normalized - lcc.transformer_tap_min_pu) / step)),
          0, last);
      normalized = lcc.transformer_tap_min_pu +
                   static_cast<double>(position) * step;
    }
  }
  return normalized;
}

PowerFlowResult solve_newton_with_lcc_tap_control(
    powerflow::SolverData& data,
    powerflow::NewtonSolver& solver,
    const PowerFlowOptions& opt,
    const InitialState* initial_state,
    LCCTapControlStatusMap& statuses) {
  constexpr int kMaxOuterSolves = 30;
  constexpr double kAngleToleranceDeg = 0.005;
  constexpr double kDcVoltageToleranceKv = 0.05;
  constexpr double kTapTolerance = 1e-10;

  std::vector<std::string> setup_warnings;
  std::unordered_map<int, int> branch_owner;
  bool has_active_control = false;
  bool initial_tap_changed = false;
  for (const auto& lcc : data.lcc_converters) {
    if (!lcc.in_service || !lcc.tap_control_modelled) continue;

    LCCTapControlStatus status;
    status.target_angle_deg = lcc_tap_target_angle_deg(lcc);
    if (lcc.station_role == LCCStationRole::Inverter &&
        lcc.control_mode == LCCControlMode::ConstantGamma) {
      status.target_dc_voltage_kv = lcc.v_dc_set_kv;
    }
    ACBranch* branch =
        find_ac_branch_by_index(data, lcc.converter_transformer_branch);
    if (branch != nullptr && std::isfinite(branch->tap)) {
      status.final_tap = branch->tap;
    }
    const bool branch_touches_valve =
        branch != nullptr &&
        (branch->from_bus == lcc.ac_bus || branch->to_bus == lcc.ac_bus);
    const bool valid_branch = branch != nullptr && branch->in_service &&
                              std::isfinite(branch->tap);
    const bool valid_range =
        std::isfinite(lcc.transformer_tap_min_pu) &&
        std::isfinite(lcc.transformer_tap_max_pu) &&
        lcc.transformer_tap_min_pu > 0.0 &&
                             lcc.transformer_tap_max_pu >=
                                 lcc.transformer_tap_min_pu;
    const bool compound_control_closed =
        lcc.station_role != LCCStationRole::Inverter ||
        lcc.control_mode != LCCControlMode::ConstantGamma ||
        status.target_dc_voltage_kv > 0.0;
    if (!valid_branch || !branch_touches_valve || !valid_range ||
        !(status.target_angle_deg > 0.0) || !compound_control_closed) {
      setup_warnings.push_back(
          "[LCC-TAP-01] LCC station " + std::to_string(lcc.index) +
          " declares R-card tap control, but its in-service transformer "
          "branch/tap, valve terminal, finite tap range, angle target, or "
          "companion inverter DC-voltage target is invalid; the T-card tap "
          "remains fixed.");
      statuses.emplace(lcc.index, status);
      continue;
    }
    if (const auto owner = branch_owner.find(branch->index);
        owner != branch_owner.end()) {
      setup_warnings.push_back(
          "[LCC-TAP-01] LCC stations " + std::to_string(owner->second) +
          " and " + std::to_string(lcc.index) +
          " request the same converter-transformer tap; only the first "
          "controller is active.");
      statuses.emplace(lcc.index, status);
      continue;
    }
    branch_owner.emplace(branch->index, lcc.index);
    const double normalized_tap =
        normalize_lcc_transformer_tap(lcc, branch->tap);
    if (std::abs(normalized_tap - branch->tap) > kTapTolerance) {
      branch->tap = normalized_tap;
      status.final_tap = normalized_tap;
      ++status.accepted_changes;
      initial_tap_changed = true;
    }
    status.active = true;
    status.final_tap = branch->tap;
    statuses.emplace(lcc.index, status);
    has_active_control = true;
  }

  if (initial_tap_changed) {
    powerflow::rebuild_matrices(data);
  }

  if (!has_active_control) {
    PowerFlowResult result = solver.solve(data, opt, initial_state);
    result.diagnostics.warnings.insert(result.diagnostics.warnings.end(),
                                       setup_warnings.begin(),
                                       setup_warnings.end());
    return result;
  }

  InitialState warm_start;
  const InitialState* solve_initial = initial_state;
  PowerFlowResult result;
  for (int outer_solve = 0; outer_solve < kMaxOuterSolves; ++outer_solve) {
    result = solver.solve(data, opt, solve_initial);
    if (!result.converged) break;

    bool all_converged = true;
    bool any_tap_changed = false;
    for (const auto& lcc : data.lcc_converters) {
      const auto status_it = statuses.find(lcc.index);
      if (status_it == statuses.end() || !status_it->second.active) continue;
      auto& status = status_it->second;
      status.update_blocked = false;
      ACBranch* branch =
          find_ac_branch_by_index(data, lcc.converter_transformer_branch);
      if (branch == nullptr) continue;

      const int ac_pos = lcc.ac_bus - 1;
      const int dc_pos = lcc.dc_bus - 1;
      if (ac_pos < 0 || dc_pos < 0 ||
          ac_pos >= static_cast<int>(result.vm.size()) ||
          dc_pos >= static_cast<int>(result.vdc.size())) {
        all_converged = false;
        status.update_blocked = true;
        continue;
      }
      const double actual_valve_kv =
          result.vm[static_cast<size_t>(ac_pos)] * lcc.vn_ac_kv;
      const Eigen::Map<const Eigen::VectorXd> solved_vm(
          result.vm.data(), static_cast<Eigen::Index>(result.vm.size()));
      const double actual_commutation_kv =
          powerflow::lcc_commutation_voltage_kv(data, lcc, solved_vm);
      const double ud_kv =
          result.vdc[static_cast<size_t>(dc_pos)] *
          powerflow::lcc_dc_base_kv(data, lcc);
      const auto operating = powerflow::lcc_operating_point(
          lcc, actual_valve_kv, actual_commutation_kv, ud_kv);
      if (!operating.valid) {
        all_converged = false;
        status.update_blocked = true;
        continue;
      }

      status.final_angle_deg =
          lcc.station_role == LCCStationRole::Rectifier
              ? operating.alpha_deg
              : operating.gamma_deg;
      status.final_dc_voltage_kv = operating.ud_kv;
      status.final_tap = branch->tap;
      const double tap_scale = std::max(1.0, std::abs(branch->tap));
      status.at_limit =
          std::abs(branch->tap - lcc.transformer_tap_min_pu) <=
              1e-8 * tap_scale ||
          std::abs(branch->tap - lcc.transformer_tap_max_pu) <=
              1e-8 * tap_scale;
      const bool angle_converged =
          std::abs(status.final_angle_deg - status.target_angle_deg) <=
          kAngleToleranceDeg;
      const bool dc_voltage_converged =
          !(status.target_dc_voltage_kv > 0.0) ||
          std::abs(status.final_dc_voltage_kv -
                   status.target_dc_voltage_kv) <=
              kDcVoltageToleranceKv;
      status.converged = angle_converged && dc_voltage_converged;
      if (status.converged) continue;
      all_converged = false;

      if (outer_solve + 1 >= kMaxOuterSolves) {
        status.outer_solve_limit_reached = true;
        continue;
      }
      const double controlled_ud_kv =
          status.target_dc_voltage_kv > 0.0
              ? status.target_dc_voltage_kv
              : operating.ud_kv;
      const double required_commutation_kv =
          powerflow::lcc_required_valve_voltage_kv(
              lcc, controlled_ud_kv, operating.id_ka,
              status.target_angle_deg);
      const double next_tap = next_lcc_transformer_tap(
          lcc, *branch, actual_commutation_kv,
          required_commutation_kv);
      if (!std::isfinite(next_tap) ||
          std::abs(next_tap - branch->tap) <= kTapTolerance) {
        status.update_blocked = true;
        continue;
      }
      branch->tap = next_tap;
      status.final_tap = next_tap;
      ++status.accepted_changes;
      any_tap_changed = true;
    }

    if (all_converged || !any_tap_changed) break;
    powerflow::rebuild_matrices(data);
    warm_start.vm = result.vm;
    warm_start.va = result.va;
    warm_start.vdc = result.vdc;
    solve_initial = &warm_start;
  }

  result.diagnostics.warnings.insert(result.diagnostics.warnings.end(),
                                     setup_warnings.begin(),
                                     setup_warnings.end());
  if (result.converged) {
    for (const auto& [index, status] : statuses) {
      if (!status.active || status.converged) continue;
      std::string detail =
          " final angle=" + std::to_string(status.final_angle_deg) +
          " deg, tap=" + std::to_string(status.final_tap);
      if (status.target_dc_voltage_kv > 0.0) {
        detail += ", final Udc=" +
                  std::to_string(status.final_dc_voltage_kv) +
                  " kV (target " +
                  std::to_string(status.target_dc_voltage_kv) + " kV)";
      }
      result.diagnostics.warnings.push_back(
          "[LCC-TAP-02] LCC station " + std::to_string(index) +
          " could not hold its R/LD angle target " +
          std::to_string(status.target_angle_deg) + " deg;" + detail +
          (status.at_limit
               ? " (R-card limit reached)."
               : status.outer_solve_limit_reached
                     ? " (tap-control outer iteration limit reached)."
                     : " (no further admissible tap update)."));
    }
  } else {
    result.diagnostics.warnings.push_back(
        "[LCC-TAP-03] Newton power flow failed during the R-card tap-control "
        "outer loop; no converged controlled operating point is available.");
  }
  return result;
}

void populate_derived_results(const powerflow::SolverData& data,
                              PowerFlowResult& result,
                              LossModelType loss_model,
                              const LCCTapControlStatusMap* tap_statuses =
                                  nullptr) {
  result.branch_flows.clear();
  result.vsc_transfers.clear();
  result.lcc_transfers.clear();
  result.dcdc_transfers.clear();
  result.er_port_transfers.clear();
  if (!result.converged) {
    return;
  }

  result.branch_flows = powerflow::compute_branch_flows(data, result.vm, result.va);
  result.canonical_branch_flows = result.branch_flows;

  Eigen::VectorXd vm = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(result.vm.size()));
  Eigen::VectorXd va = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(result.va.size()));
  Eigen::VectorXd vdc = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(result.vdc.size()));
  for (Eigen::Index i = 0; i < vm.size(); ++i) vm[i] = result.vm[static_cast<size_t>(i)];
  for (Eigen::Index i = 0; i < va.size(); ++i) va[i] = result.va[static_cast<size_t>(i)];
  for (Eigen::Index i = 0; i < vdc.size(); ++i) vdc[i] = result.vdc[static_cast<size_t>(i)];

  std::unordered_map<int, int> ac_pos_by_bus;
  ac_pos_by_bus.reserve(data.ac_buses.size());
  for (int i = 0; i < static_cast<int>(data.ac_buses.size()); ++i) {
    ac_pos_by_bus[data.ac_buses[static_cast<size_t>(i)].index] = i;
  }
  std::unordered_map<int, int> dc_pos_by_bus;
  dc_pos_by_bus.reserve(data.dc_buses.size());
  for (int i = 0; i < static_cast<int>(data.dc_buses.size()); ++i) {
    dc_pos_by_bus[data.dc_buses[static_cast<size_t>(i)].index] = i;
  }
  auto ac_pos = [&](int bus) {
    const auto it = ac_pos_by_bus.find(bus);
    return it != ac_pos_by_bus.end() ? it->second : -1;
  };
  auto dc_pos = [&](int bus) {
    const auto it = dc_pos_by_bus.find(bus);
    return it != dc_pos_by_bus.end() ? it->second : -1;
  };

  // Converters the solver auto-promoted from PQ to VDC_Q (because their DC island
  // had no voltage reference) must report transfers using the regulating mode,
  // not their stored PQ setpoint — otherwise the reported AC/DC powers are wrong.
  // The solver also stiffens the promoted converter's Vdc gain and may switch
  // modes mid-iteration, so prefer its final `effective_converters` list (which
  // already reflects all of that) and only fall back to the input converters +
  // promotion-index flip when it is unavailable.
  const std::vector<VSCConverter>& eff_converters =
      !result.diagnostics.effective_converters.empty()
          ? result.diagnostics.effective_converters
          : data.converters;
  const auto& promoted = result.diagnostics.promoted_vsc_indices;
  auto is_promoted = [&promoted](int ci) {
    return std::find(promoted.begin(), promoted.end(), ci) != promoted.end();
  };
  std::unordered_map<int, const VSCLimitStateResult*> limit_state_by_index;
  limit_state_by_index.reserve(result.vsc_limit_states.size());
  for (const auto& state : result.vsc_limit_states) {
    const auto [iterator, inserted] =
        limit_state_by_index.emplace(state.index, &state);
    if (!inserted) {
      throw std::runtime_error(
          "populate_derived_results: duplicate stable VSC limit-state index " +
          std::to_string(state.index));
    }
    (void)iterator;
  }

  // AC bus-attribution converters: an AC_PV converter's AC bus is voltage-
  // controlled (PV) and an AC_GRID_FORMING converter's AC bus is a slack, so in
  // both cases the converter's released power equals the bus's net injection.
  // Compute the network current I = Ybus·V once when any such converter is
  // present.  This credits the bus's balance to the converter; with several
  // devices on one bus the attribution is shared (multi-converter model r1 §1.3)
  // and this single-source convention over-credits one converter.
  const bool has_bus_attribution = std::any_of(
      eff_converters.begin(), eff_converters.end(), [](const VSCConverter& c) {
        return c.in_service && (c.control_mode == ConverterMode::AC_PV ||
                                c.control_mode == ConverterMode::AC_GRID_FORMING);
      });
  Eigen::VectorXcd ybus_current;
  Eigen::VectorXd fixed_p_spec;
  Eigen::VectorXd fixed_q_spec;
  std::vector<int> free_p_count(static_cast<size_t>(vm.size()), 0);
  std::vector<int> free_q_count(static_cast<size_t>(vm.size()), 0);
  if (has_bus_attribution && data.ybus.rows() == vm.size() &&
      data.ybus.cols() == vm.size()) {
    Eigen::VectorXcd vbus(vm.size());
    for (Eigen::Index i = 0; i < vm.size(); ++i) vbus[i] = std::polar(vm[i], va[i]);
    ybus_current = data.ybus * vbus;

    powerflow::SolverData attribution_data = data;
    attribution_data.converters = eff_converters;
    powerflow::assemble_ac_injections(
        attribution_data, vm, va, vdc, fixed_p_spec, fixed_q_spec);
    for (const auto& converter : eff_converters) {
      if (!converter.in_service) continue;
      const int aci = ac_pos(converter.bus_ac);
      if (aci < 0 || aci >= static_cast<int>(vm.size())) continue;
      const auto [pac, qac] = powerflow::converter_ac_injection(
          converter, vm, va, vdc, data.base_mva, loss_model);
      if (converter.control_mode == ConverterMode::AC_GRID_FORMING) {
        fixed_p_spec[aci] -= pac;
        fixed_q_spec[aci] -= qac;
        free_p_count[static_cast<size_t>(aci)] += 1;
        free_q_count[static_cast<size_t>(aci)] += 1;
      } else if (converter.control_mode == ConverterMode::AC_PV) {
        fixed_q_spec[aci] -= qac;
        free_q_count[static_cast<size_t>(aci)] += 1;
      }
    }
    for (Eigen::Index aci = 0; aci < vm.size(); ++aci) {
      if (free_p_count[static_cast<size_t>(aci)] > 1 ||
          free_q_count[static_cast<size_t>(aci)] > 1) {
        result.diagnostics.warnings.push_back(
            "[PF-ATTR-01] Multiple free-power converters share AC bus " +
            std::to_string(data.ac_buses[static_cast<size_t>(aci)].index) +
            "; the unassigned bus injection was attributed equally.");
      }
    }
  }

  result.vsc_transfers.reserve(eff_converters.size());
  for (int ci = 0; ci < static_cast<int>(eff_converters.size()); ++ci) {
    VSCConverter conv = eff_converters[static_cast<size_t>(ci)];
    if (!conv.in_service) continue;
    if (conv.control_mode == ConverterMode::PQ_MODE && is_promoted(ci)) {
      conv.control_mode = ConverterMode::VDC_Q;
    }
    const auto [p_ac_pu, q_ac_pu] =
        powerflow::converter_ac_injection(conv, vm, va, vdc, data.base_mva, loss_model);
    const double p_dc_pu =
        powerflow::converter_dc_injection(conv, vm, va, vdc, data.base_mva, loss_model);

    // AC_PV releases reactive power: report the bus's reactive generation (the
    // free balancing injection) instead of the placeholder q_set_mvar.
    double p_ac_pu_out = p_ac_pu;
    double q_ac_pu_out = q_ac_pu;
    double p_dc_pu_out = p_dc_pu;
    const auto limit_state_iterator = limit_state_by_index.find(conv.index);
    const VSCLimitStateResult* limit_state =
        limit_state_iterator != limit_state_by_index.end()
            ? limit_state_iterator->second
            : nullptr;
    if (limit_state != nullptr) {
      p_ac_pu_out = limit_state->p_ac_pu;
      q_ac_pu_out = limit_state->q_ac_pu;
      p_dc_pu_out = limit_state->p_dc_pu;
    } else if (conv.control_mode == ConverterMode::AC_PV &&
        ybus_current.size() == vm.size()) {
      const int aci = ac_pos(conv.bus_ac);
      if (aci >= 0 && aci < static_cast<int>(vm.size())) {
        const std::complex<double> v_i = std::polar(vm[aci], va[aci]);
        const double q_net_inj =
            (v_i * std::conj(ybus_current[static_cast<Eigen::Index>(aci)])).imag();
        const int count = free_q_count[static_cast<size_t>(aci)];
        if (count > 0 && fixed_q_spec.size() == vm.size()) {
          q_ac_pu_out =
              (q_net_inj - fixed_q_spec[aci]) / static_cast<double>(count);
        }
      }
    } else if (conv.control_mode == ConverterMode::AC_GRID_FORMING &&
               ybus_current.size() == vm.size()) {
      // AC_GRID_FORMING releases BOTH active and reactive power (its AC bus is a
      // slack). Report the bus's net active/reactive generation, and report the
      // DC injection as the energy-conduit value −(P_ac + loss) so the converter's
      // reported powers are energy-consistent (matching the solved DC balance).
      const int aci = ac_pos(conv.bus_ac);
      if (aci >= 0 && aci < static_cast<int>(vm.size())) {
        const std::complex<double> v_i = std::polar(vm[aci], va[aci]);
        const std::complex<double> s_net =
            v_i * std::conj(ybus_current[static_cast<Eigen::Index>(aci)]);
        const int p_count = free_p_count[static_cast<size_t>(aci)];
        const int q_count = free_q_count[static_cast<size_t>(aci)];
        if (p_count > 0 && fixed_p_spec.size() == vm.size()) {
          p_ac_pu_out =
              (s_net.real() - fixed_p_spec[aci]) /
              static_cast<double>(p_count);
        }
        if (q_count > 0 && fixed_q_spec.size() == vm.size()) {
          q_ac_pu_out =
              (s_net.imag() - fixed_q_spec[aci]) /
              static_cast<double>(q_count);
        }
        const int dci = dc_pos(conv.bus_dc);
        const double vdc_b = (dci >= 0 && dci < static_cast<int>(vdc.size()))
                                 ? vdc[static_cast<size_t>(dci)]
                                 : 1.0;
        const double ploss = powerflow::converter_loss(
            conv, p_ac_pu_out, vdc_b, data.base_mva, loss_model);
        p_dc_pu_out = -(p_ac_pu_out + ploss);
      }
    }

    VSCTransfer tr;
    tr.index = conv.index;
    tr.bus_ac = conv.bus_ac;
    tr.bus_dc = conv.bus_dc;
    tr.p_ac_mw = p_ac_pu_out * data.base_mva;
    tr.q_ac_mvar = q_ac_pu_out * data.base_mva;
    tr.p_dc_mw = p_dc_pu_out * data.base_mva;
    tr.loss_mw = -(tr.p_ac_mw + tr.p_dc_mw);
    tr.limit_ncp_enabled = limit_state != nullptr;
    tr.current_limit_active =
        limit_state != nullptr && limit_state->current_limit_active;
    tr.droop_saturated =
        limit_state != nullptr && limit_state->droop_saturated;
    tr.current_limit_priority =
        limit_state != nullptr ? limit_state->current_limit_priority : "none";
    tr.effective_mode = converter_mode_str(conv.control_mode);
    tr.ac_current_pu =
        limit_state != nullptr ? limit_state->current_pu : 0.0;
    tr.current_margin_pu =
        limit_state != nullptr ? limit_state->current_margin_pu : 0.0;
    tr.complementarity_residual =
        limit_state != nullptr ? limit_state->complementarity_residual : 0.0;
    if (limit_state != nullptr) {
      tr.internal_voltage_real_pu = limit_state->internal_voltage_real_pu;
      tr.internal_voltage_imag_pu = limit_state->internal_voltage_imag_pu;
      tr.terminal_current_real_pu = limit_state->terminal_current_real_pu;
      tr.terminal_current_imag_pu = limit_state->terminal_current_imag_pu;
      tr.gfm_norton_model = limit_state->gfm_norton_model;
    }
    result.vsc_transfers.push_back(tr);

    // ── Post-solve converter physical-limit feasibility (multi-converter model
    // §3.1.4–3.1.7).  Each check is opt-in — it only fires when the relevant
    // limit field is set (>0), so default systems are unaffected.  These are
    // diagnostics (warnings), not hard constraints in this determined solve.
    const int ac_bus_pos = ac_pos(conv.bus_ac);
    const int dc_bus_pos = dc_pos(conv.bus_dc);
    const double vm_ac =
        (ac_bus_pos >= 0 && ac_bus_pos < static_cast<int>(vm.size()))
            ? std::max(vm[ac_bus_pos], 1e-3)
            : 1.0;
    const double vdc_b =
        (dc_bus_pos >= 0 && dc_bus_pos < static_cast<int>(vdc.size()))
            ? std::max(vdc[dc_bus_pos], 1e-3)
            : 1.0;
    const double s_ac_pu = std::hypot(p_ac_pu_out, q_ac_pu_out);
    const double s_rated_pu = conv.p_rated_mw / data.base_mva;
    if (s_rated_pu > 0.0 && s_ac_pu > s_rated_pu * (1.0 + 1e-6)) {
      result.diagnostics.warnings.push_back(
          "[ACDC-PHYS-04] VSC converter " + std::to_string(conv.index) +
          " apparent power " + std::to_string(s_ac_pu * data.base_mva) +
          " MVA exceeds its rating " + std::to_string(conv.p_rated_mw) + " MVA.");
    }
    if (conv.i_ac_max_pu > 0.0) {
      const double i_ac_pu = s_ac_pu / vm_ac;
      if (i_ac_pu > conv.i_ac_max_pu * (1.0 + 1e-6)) {
        result.diagnostics.warnings.push_back(
            "[ACDC-PHYS-02] VSC converter " + std::to_string(conv.index) +
            " AC current " + std::to_string(i_ac_pu) + " pu exceeds limit " +
            std::to_string(conv.i_ac_max_pu) + " pu.");
      }
    }
    if (conv.i_dc_max_pu > 0.0) {
      const double i_dc_pu = std::abs(p_dc_pu_out) / vdc_b;
      if (i_dc_pu > conv.i_dc_max_pu * (1.0 + 1e-6)) {
        result.diagnostics.warnings.push_back(
            "[ACDC-PHYS-03] VSC converter " + std::to_string(conv.index) +
            " DC current " + std::to_string(i_dc_pu) + " pu exceeds limit " +
            std::to_string(conv.i_dc_max_pu) + " pu.");
      }
    }
    if (conv.k_m_modulation > 0.0 && conv.m_max > 0.0 && conv.vn_ac_kv > 0.0 &&
        conv.vn_dc_kv > 0.0) {
      const double v_ac_model = vm_ac * conv.vn_ac_kv;  // kV
      const double v_dc_model = vdc_b * conv.vn_dc_kv;   // kV
      const double m = v_ac_model / (conv.k_m_modulation * v_dc_model);
      if (m > conv.m_max * (1.0 + 1e-6)) {
        result.diagnostics.warnings.push_back(
            "[ACDC-PHYS-05] VSC converter " + std::to_string(conv.index) +
            " requires modulation index " + std::to_string(m) + " > m_max " +
            std::to_string(conv.m_max) +
            "; DC voltage is insufficient for the requested AC voltage.");
      } else if (conv.m_min > 0.0 && m < conv.m_min * (1.0 - 1e-6)) {
        result.diagnostics.warnings.push_back(
            "[ACDC-PHYS-01] VSC converter " + std::to_string(conv.index) +
            " modulation index " + std::to_string(m) + " is below m_min " +
            std::to_string(conv.m_min) + ".");
      }
    }
  }

  result.dcdc_transfers.reserve(data.dcdc_converters.size());
  for (const auto& c : data.dcdc_converters) {
    if (!c.in_service) continue;
    // Report the SOLVED transfer evaluated at the converged DC voltages, not the
    // schedule. In Voltage/Droop modes the delivered power is set by the island
    // power balance, so c.p_ref_mw is NOT the actual transfer; Power mode yields
    // the same numbers as before (minus the now-accounted I2R loss).
    const auto xfer = powerflow::dcdc_power_transfer(c, vdc, data.base_mva);
    DCDCTransfer tr;
    tr.index = c.index;
    tr.bus_in = c.bus_in;
    tr.bus_out = c.bus_out;
    tr.p_in_mw = xfer.p_in_pu * data.base_mva;
    tr.p_out_mw = xfer.p_out_pu * data.base_mva;
    tr.loss_mw = tr.p_in_mw - tr.p_out_mw;
    // Duty-ratio feasibility from the solved port voltages (multi-converter §3.2).
    const int bi = dc_pos(c.bus_in);
    const int bo = dc_pos(c.bus_out);
    const double v_in = (bi >= 0 && bi < vdc.size()) ? vdc[bi] : 1.0;
    const double v_out = (bo >= 0 && bo < vdc.size()) ? vdc[bo] : 1.0;
    const auto duty = powerflow::dcdc_duty_ratio(c, v_in, v_out);
    tr.duty = duty.duty;
    tr.voltage_ratio = duty.voltage_ratio;
    tr.duty_defined = duty.defined;
    tr.duty_feasible = duty.feasible;
    result.dcdc_transfers.push_back(tr);
  }

  for (const auto& er : data.energy_routers) {
    if (!er.in_service) continue;
    for (const auto& p : er.ports) {
      if (!p.in_service) continue;
      ERPortTransfer tr;
      tr.router_index = er.index;
      tr.port_index = p.index;
      tr.bus = p.bus;
      tr.is_ac = (p.port_type == ERPortType::AC);
      tr.p_mw = p.p_mw;
      tr.q_mvar = p.q_mvar;
      if (tr.is_ac) {
        const int b = ac_pos(p.bus);
        tr.v_pu = (b >= 0 && b < vm.size()) ? vm[b] : 1.0;
      } else {
        const int b = dc_pos(p.bus);
        tr.v_pu = (b >= 0 && b < vdc.size()) ? vdc[b] : 1.0;
      }
      result.er_port_transfers.push_back(tr);
    }
  }

  // ── LCC stations (quasi-steady, dat manual ch.4) ─────────────────────────
  // Re-evaluate each station at the converged state and report the full
  // operating point, including the effective converter-transformer tap and
  // the outcome of any R-card outer-loop control.
  result.lcc_transfers.reserve(data.lcc_converters.size());
  for (const auto& lcc : data.lcc_converters) {
    if (!lcc.in_service) continue;
    const int aci = ac_pos(lcc.ac_bus);
    const int dci = dc_pos(lcc.dc_bus);
    if (aci < 0 || dci < 0) continue;
    const double e_kv = vm[aci] * lcc.vn_ac_kv;
    const double commutation_e_kv =
        powerflow::lcc_commutation_voltage_kv(data, lcc, vm);
    const double ud_kv = vdc[dci] * powerflow::lcc_dc_base_kv(data, lcc);
    const auto op = powerflow::lcc_operating_point(
        lcc, e_kv, commutation_e_kv, ud_kv);
    if (!op.valid) continue;

    LCCTransfer tr;
    tr.index = lcc.index;
    tr.bus_ac = lcc.ac_bus;
    tr.bus_dc = lcc.dc_bus;
    tr.station_role = static_cast<int>(lcc.station_role);
    tr.control_mode = static_cast<int>(lcc.control_mode);
    tr.alpha_deg = op.alpha_deg;
    tr.gamma_deg = op.gamma_deg;
    tr.ud0_kv = op.ud0_kv;
    tr.ud_kv = op.ud_kv;
    tr.id_ka = op.id_ka;
    tr.p_ac_mw = op.p_ac_mw;
    tr.q_ac_mvar = op.q_ac_mvar;
    tr.p_dc_mw = op.p_dc_mw;
    tr.tap_target_angle_deg = lcc_tap_target_angle_deg(lcc);
    if (const ACBranch* branch =
            find_ac_branch_by_index(data,
                                    lcc.converter_transformer_branch);
        branch != nullptr) {
      tr.transformer_tap = branch->tap;
    }
    if (tap_statuses != nullptr) {
      const auto status = tap_statuses->find(lcc.index);
      if (status != tap_statuses->end()) {
        tr.tap_control_active = status->second.active;
        tr.tap_control_converged = status->second.converged;
        tr.tap_control_iterations = status->second.accepted_changes;
        tr.tap_at_limit = status->second.at_limit;
        tr.tap_target_angle_deg = status->second.target_angle_deg;
      }
    }
    tr.id_at_limit = op.id_at_limit;
    tr.alpha_within_limits =
        op.alpha_deg >= lcc.alpha_min_deg - 1e-6 &&
        op.alpha_deg <= lcc.alpha_stop_deg + 1e-6;
    tr.gamma_within_limits =
        lcc.gamma_min_deg <= 0.0 || op.gamma_deg >= lcc.gamma_min_deg - 1e-6;
    result.lcc_transfers.push_back(tr);

    // Honest post-solve angle-limit diagnostics (no silent wrong solution).
    if (!tr.alpha_within_limits &&
        lcc.station_role == LCCStationRole::Rectifier) {
      result.diagnostics.warnings.push_back(
          "[LCC-PHYS-01] LCC rectifier " + std::to_string(lcc.index) +
          " back-calculated alpha=" + std::to_string(op.alpha_deg) +
          " deg is outside [alpha_min=" + std::to_string(lcc.alpha_min_deg) +
          ", alpha_stop=" + std::to_string(lcc.alpha_stop_deg) +
          "] deg; with fixed converter-transformer taps the model cannot "
          "restore alpha — treat the reported operating point as the "
          "no-tap-action solution.");
    }
    if (!tr.gamma_within_limits &&
        lcc.station_role == LCCStationRole::Inverter) {
      result.diagnostics.warnings.push_back(
          "[LCC-PHYS-02] LCC inverter " + std::to_string(lcc.index) +
          " extinction angle gamma=" + std::to_string(op.gamma_deg) +
          " deg violates gamma_min=" + std::to_string(lcc.gamma_min_deg) +
          " deg (commutation-failure risk); no tap/control remedial action "
          "is modelled.");
    }
    if (op.id_at_limit && op.id_ka > 0.0 &&
        powerflow::lcc_forms_dc_voltage(lcc) &&
        (!tr.tap_control_active || !tr.tap_control_converged)) {
      // Honest control-mode note: the rated-current clamp (current order)
      // binds, so the declared CEA / constant-alpha setpoint is not held;
      // the reported gamma/alpha is the back-calculated physical value
      // (>= the minimum, so commutation margin is preserved).
      result.diagnostics.warnings.push_back(
          "[LCC-PHYS-03] LCC station " + std::to_string(lcc.index) +
          " runs at its rated current (" + std::to_string(op.id_ka) +
          " kA): the current limit binds and the CEA/constant-alpha setpoint "
          "is not held with fixed converter-transformer taps; back-calculated "
          "gamma/alpha is reported instead.");
    }
  }
}

void populate_newton_converter_scope(const powerflow::SolverData& data,
                                     PowerFlowResult& result,
                                     const PowerFlowOptions& options,
                                     bool lcc_tap_control_active = false) {
  auto& scope = result.converter_model_scope;
  const bool has_active_lcc = std::any_of(
      data.lcc_converters.begin(), data.lcc_converters.end(),
      [](const LCCConverter& converter) { return converter.in_service; });
  const bool has_vsc_limit_ncp = !result.vsc_limit_states.empty();
  scope.model_scope = !has_active_lcc
                          ? "steady-state-newton:vsc-3mode+dcdc-power-transfer"
                          : "steady-state-newton:vsc-3mode+dcdc-power-transfer+lcc-quasi-steady";
  if (lcc_tap_control_active) {
    scope.model_scope += "+r-card-transformer-tap-control";
  }
  if (has_vsc_limit_ncp) {
    scope.model_scope += "+vsc-current-limit-ncp";
  }
  scope.validity.vsc_loss_modelled = true;
  scope.validity.vsc_ac_conduction_loss_modelled = true;
  scope.validity.vsc_vdc_control_modelled = true;
  scope.validity.dcdc_loss_modelled = true;
  scope.validity.vsc_capacity_circle_enforced =
      options.enforce_converter_physical_limits;
  scope.validity.vsc_current_limits_enforced =
      options.enforce_converter_physical_limits || has_vsc_limit_ncp;
  scope.validity.vsc_gfm_norton_modelled = std::any_of(
      result.vsc_limit_states.begin(), result.vsc_limit_states.end(),
      [](const VSCLimitStateResult& state) { return state.gfm_norton_model; });
  scope.validity.vsc_gfm_priority_limit_enforced =
      scope.validity.vsc_gfm_norton_modelled && has_vsc_limit_ncp;
  scope.validity.vsc_gfm_island_reference_modelled =
      !result.diagnostics.gfm_island_reference_vsc_indices.empty();
  scope.validity.vsc_modulation_limits_enforced =
      options.enforce_converter_physical_limits;
  scope.validity.lcc_quasi_steady_modelled = has_active_lcc;
  scope.validity.lcc_transformer_tap_control_modelled =
      lcc_tap_control_active;
  scope.validity.dc_multisource_coordination_modelled = true;
  scope.validity.dcdc_duty_ratio_enforced =
      options.enforce_converter_physical_limits;
  scope.validity.equation_closure_checked =
      result.diagnostics.equation_closure_checked;
}

void restore_original_vsc_bus_ac(PowerFlowResult& result,
                                 const std::unordered_map<int, int>& bus_by_index) {
  if (bus_by_index.empty()) return;
  auto original_bus = [&](int index, int fallback) {
    const auto it = bus_by_index.find(index);
    return it != bus_by_index.end() ? it->second : fallback;
  };

  for (auto& tr : result.vsc_transfers) {
    tr.bus_ac = original_bus(tr.index, tr.bus_ac);
  }
  for (auto& conv : result.diagnostics.effective_converters) {
    conv.bus_ac = original_bus(conv.index, conv.bus_ac);
  }
}

void restore_original_vsc_bus_ac(PowerFlowResult& result, const HybridPowerSystem& sys) {
  restore_original_vsc_bus_ac(result, build_original_vsc_ac_bus_map(sys));
}

// LCC valve-side AC buses go through the same canonical reindexing as VSC
// bus_ac; restore the AUTHORED component ids for reporting (three-ID rule:
// results are attributed back to stable component ids).
void restore_original_lcc_bus_ac(
    PowerFlowResult& result,
    const std::unordered_map<int, int>& bus_by_index) {
  if (bus_by_index.empty() || result.lcc_transfers.empty()) return;
  for (auto& tr : result.lcc_transfers) {
    const auto it = bus_by_index.find(tr.index);
    if (it != bus_by_index.end()) tr.bus_ac = it->second;
  }
}

void restore_original_lcc_bus_ac(PowerFlowResult& result,
                                 const HybridPowerSystem& sys) {
  restore_original_lcc_bus_ac(result, build_original_lcc_ac_bus_map(sys));
}

void restore_original_dc_bus_ids(
    PowerFlowResult& result,
    const std::unordered_map<int, int>& vsc_bus_dc,
    const std::unordered_map<int, int>& lcc_bus_dc,
    const std::unordered_map<int, std::pair<int, int>>& dcdc_buses) {
  for (auto& transfer : result.vsc_transfers) {
    const auto it = vsc_bus_dc.find(transfer.index);
    if (it != vsc_bus_dc.end()) transfer.bus_dc = it->second;
  }
  for (auto& converter : result.diagnostics.effective_converters) {
    const auto it = vsc_bus_dc.find(converter.index);
    if (it != vsc_bus_dc.end()) converter.bus_dc = it->second;
  }
  for (auto& transfer : result.lcc_transfers) {
    const auto it = lcc_bus_dc.find(transfer.index);
    if (it != lcc_bus_dc.end()) transfer.bus_dc = it->second;
  }
  for (auto& transfer : result.dcdc_transfers) {
    const auto it = dcdc_buses.find(transfer.index);
    if (it != dcdc_buses.end()) {
      transfer.bus_in = it->second.first;
      transfer.bus_out = it->second.second;
    }
  }
}

void restore_original_dc_bus_ids(PowerFlowResult& result,
                                 const HybridPowerSystem& sys) {
  std::unordered_map<int, int> vsc_bus_dc;
  std::unordered_map<int, int> lcc_bus_dc;
  std::unordered_map<int, std::pair<int, int>> dcdc_buses;
  for (const auto& converter : sys.vsc_converters) {
    vsc_bus_dc[converter.index] = converter.bus_dc;
  }
  for (const auto& converter : sys.lcc_converters) {
    lcc_bus_dc[converter.index] = converter.dc_bus;
  }
  for (const auto& converter : sys.dc.dcdc_converters) {
    dcdc_buses[converter.index] = {converter.bus_in, converter.bus_out};
  }
  restore_original_dc_bus_ids(result, vsc_bus_dc, lcc_bus_dc, dcdc_buses);
}

void unproject_branch_flows(PowerFlowResult& result, const BusMergeMap& map,
                            size_t authored_branch_count) {
  if (map.n_original_branches <= 0 || map.branch_orig_to_proj.empty()) return;

  std::vector<BranchFlow> out(static_cast<size_t>(map.n_original_branches));
  for (const auto& [orig_pos, proj_pos] : map.branch_orig_to_proj) {
    if (orig_pos < 0 || orig_pos >= map.n_original_branches) continue;
    if (proj_pos < 0 || proj_pos >= static_cast<int>(result.branch_flows.size())) continue;
    out[static_cast<size_t>(orig_pos)] = result.branch_flows[static_cast<size_t>(proj_pos)];
  }
  result.branch_flows = std::move(out);
  if (result.branch_flows.size() > authored_branch_count) {
    result.branch_flows.resize(authored_branch_count);
  }
}

void restore_collapsed_branch_charging(
    PowerFlowResult& result, const BusMergeMap& map,
    const std::vector<ACBranch>& authored_branches, double base_mva,
    const std::vector<double>& authored_vm) {
  for (const auto& [orig_pos, proj_pos] : map.branch_orig_to_proj) {
    if (proj_pos >= 0 || orig_pos < 0 ||
        orig_pos >= static_cast<int>(authored_branches.size()) ||
        orig_pos >= static_cast<int>(result.branch_flows.size())) {
      continue;
    }
    const auto& branch = authored_branches[static_cast<size_t>(orig_pos)];
    const auto from = map.ext_to_int.find(branch.from_bus);
    const auto to = map.ext_to_int.find(branch.to_bus);
    if (!branch.in_service || !std::isfinite(branch.b_pu) ||
        from == map.ext_to_int.end() || to == map.ext_to_int.end() ||
        from->second != to->second) {
      continue;
    }
    const auto from_pos = map.ext_to_orig_pos.find(branch.from_bus);
    const auto to_pos = map.ext_to_orig_pos.find(branch.to_bus);
    if (from_pos == map.ext_to_orig_pos.end() ||
        to_pos == map.ext_to_orig_pos.end() || from_pos->second < 0 ||
        to_pos->second < 0 ||
        from_pos->second >= static_cast<int>(authored_vm.size()) ||
        to_pos->second >= static_cast<int>(authored_vm.size())) {
      continue;
    }
    const double vf = authored_vm[static_cast<size_t>(from_pos->second)];
    const double vt = authored_vm[static_cast<size_t>(to_pos->second)];
    auto& flow = result.branch_flows[static_cast<size_t>(orig_pos)];
    flow.qf_mvar = -0.5 * branch.b_pu * base_mva * vf * vf;
    flow.qt_mvar = -0.5 * branch.b_pu * base_mva * vt * vt;
  }
}

double authored_ac_base_mva(const HybridPowerSystem& authored) {
  return authored.base_mva > 0.0
             ? authored.base_mva
             : (authored.ac.base_mva > 0.0 ? authored.ac.base_mva : 100.0);
}

// Expand merged PF result vectors (vm, va) back to the original bus count
// so that callers can index by original bus position.
void unproject_pf_result(PowerFlowResult& result, const BusMergeMap& map,
                         const std::vector<ACBranch>& authored_branches,
                         double base_mva) {
  if (!map.ext_to_int.empty() && map.n_original > 0) {
    result.vm = unproject_bus_vector(result.vm, map, BusVectorSemantics::Intensive);
    result.va = unproject_bus_vector(result.va, map, BusVectorSemantics::Intensive);
  }
  unproject_branch_flows(result, map, authored_branches.size());
  restore_collapsed_branch_charging(result, map, authored_branches, base_mva,
                                    result.vm);
}

// Recover DC bus voltages after a possible dead-island strip, then drop
// energy-router-internal DC buses, leaving the vector in authored DC bus order.
// A no-op resize when no DC strip occurred (identity unproject).
void recover_dc_bus_voltages(
    std::vector<double>& vdc,
    const std::optional<ProjectionCertificate>& certificate) {
  if (!certificate || certificate->n_authored_dc_buses < 0) return;
  if (certificate->has_dc_strip())
    vdc = unproject_dc_bus_vector(vdc, *certificate);
  const size_t authored =
      static_cast<size_t>(certificate->n_authored_dc_buses);
  if (vdc.size() > authored) vdc.resize(authored);
}

void trim_internal_dc_bus_results(
    PowerFlowResult& result,
    const std::optional<ProjectionCertificate>& certificate) {
  recover_dc_bus_voltages(result.vdc, certificate);
  // Surface stripped DC dead islands on the honest diagnostics.warnings channel
  // (serialized by the runtime API, rendered by the GUI) so consumers can flag
  // the affected authored DC bus positions.
  if (certificate && certificate->has_dc_strip()) {
    const auto& prestrip = certificate->dc_prestrip_to_survivor;
    const int authored = certificate->n_authored_dc_buses;
    std::string msg = "DC dead-island strip: authored DC bus position(s)";
    bool any = false;
    for (int p = 0; p < static_cast<int>(prestrip.size()) && p < authored; ++p) {
      if (prestrip[static_cast<size_t>(p)] < 0) {
        msg += " " + std::to_string(p + 1);
        any = true;
      }
    }
    if (any) {
      msg += " removed as unsourced islands; reported as 0 pu.";
      result.diagnostics.warnings.push_back(std::move(msg));
    }
  }
}

std::vector<double> project_intensive_bus_vector(
    const std::vector<double>& authored, const BusMergeMap& map) {
  if (static_cast<int>(authored.size()) != map.n_original) {
    throw std::invalid_argument(
        "project_intensive_bus_vector: authored vector size does not match "
        "BusMergeMap.n_original");
  }
  std::vector<double> canonical(static_cast<size_t>(map.n_merged), 0.0);
  std::vector<char> assigned(static_cast<size_t>(map.n_merged), 0);
  for (const auto& [ext_bus, int_pos] : map.ext_to_int) {
    const auto orig = map.ext_to_orig_pos.find(ext_bus);
    if (orig == map.ext_to_orig_pos.end() || int_pos < 0 ||
        int_pos >= map.n_merged || orig->second < 0 ||
        orig->second >= map.n_original) {
      continue;
    }
    if (assigned[static_cast<size_t>(int_pos)] == 0) {
      canonical[static_cast<size_t>(int_pos)] =
          authored[static_cast<size_t>(orig->second)];
      assigned[static_cast<size_t>(int_pos)] = 1;
    }
  }
  return canonical;
}

}  // namespace

// ── Validation wrapper ────────────────────────────────────────────────────────

validation::ValidationReport validate_full(const HybridPowerSystem& sys) {
  return validation::validate(sys);
}

// ── Exception-free solve ──────────────────────────────────────────────────────

Result<PowerFlowResult> safe_solve_power_flow(
    const HybridPowerSystem& sys,
    const PowerFlowOptions& opt,
    bool validate_input)
{
  if (validate_input) {
    auto report = validate_full(sys);
    if (report.has_errors())
      return Error::validation_failed(std::move(report));
  }
  try {
    return solve_power_flow(sys, opt);
  } catch (const std::exception& e) {
    return Error{ErrorCode::NumericalFailure, e.what(), {}};
  }
}

PowerFlowResult solve_power_flow(const HybridPowerSystem& sys, const PowerFlowOptions& opt) {
  const auto facade_start = std::chrono::steady_clock::now();
  const auto reference_validation =
      validation::validate_reference_bus_eligibility(sys);
  if (reference_validation.has_errors()) {
    PowerFlowResult result;
    result.converged = false;
    result.diagnostics.termination_reason =
        "Reference-bus eligibility check failed";
    for (const auto& issue : reference_validation.issues)
      result.diagnostics.warnings.push_back(issue.message);
    return result;
  }
  const powerflow::ConverterCoordinationReport coordination =
      powerflow::evaluate_converter_coordination(sys, opt.enable_converter_coordination_check);
  if (coordination.enabled && coordination.has_blocking_issue()) {
    PowerFlowResult result;
    result.converged = false;
    result.diagnostics.converter_coordination = coordination;
    result.diagnostics.termination_reason = "Converter coordination feasibility check failed";
    for (const auto& issue : coordination.issues) {
      result.diagnostics.warnings.push_back("[" + issue.rule_id + "] " + issue.message);
    }
    return result;
  }

  // ── LCC station readiness pre-check (honest failure, no silent wrong
  // solution): every in-service LCC station must resolve its valve-side AC
  // bus and DC bus, carry a valve-side voltage base, use a supported
  // role/control-mode combination, and have a commutation reactance for the
  // characteristic (CEA / constant-alpha) control modes.
  if (!sys.lcc_converters.empty()) {
    std::unordered_map<int, int> ac_bus_ids, dc_bus_ids;
    for (const auto& b : sys.ac.buses) ac_bus_ids.emplace(b.index, 1);
    for (const auto& b : sys.dc.buses) dc_bus_ids.emplace(b.index, 1);
    std::vector<std::string> lcc_errors;
    for (const auto& lcc : sys.lcc_converters) {
      if (!lcc.in_service) continue;
      if (ac_bus_ids.find(lcc.ac_bus) == ac_bus_ids.end()) {
        lcc_errors.push_back("LCC station " + std::to_string(lcc.index) +
                             " references missing valve-side AC bus " +
                             std::to_string(lcc.ac_bus) + ".");
      }
      if (dc_bus_ids.find(lcc.dc_bus) == dc_bus_ids.end()) {
        lcc_errors.push_back("LCC station " + std::to_string(lcc.index) +
                             " references missing DC bus " +
                             std::to_string(lcc.dc_bus) + ".");
      }
      if (lcc.vn_ac_kv <= 0.0) {
        lcc_errors.push_back("LCC station " + std::to_string(lcc.index) +
                             " has no valve-side AC base voltage (vn_ac_kv); "
                             "U_d0 cannot be evaluated.");
      }
      if (!powerflow::lcc_control_supported(lcc)) {
        lcc_errors.push_back(
            "LCC station " + std::to_string(lcc.index) +
            ": control mode " + std::to_string(static_cast<int>(lcc.control_mode)) +
            " is not supported for station role " +
            std::to_string(static_cast<int>(lcc.station_role)) +
            " (rectifier: ConstantPower/ConstantCurrent/ConstantAlpha; "
            "inverter: ConstantGamma/ConstantPower/ConstantCurrent).");
      }
      if (powerflow::lcc_forms_dc_voltage(lcc) && lcc.x_comm_ohm <= 0.0) {
        lcc_errors.push_back(
            "LCC station " + std::to_string(lcc.index) +
            " runs a characteristic control mode (CEA/constant-alpha) but has "
            "no commutation reactance (converter transformer not identified); "
            "its DC voltage characteristic is undefined.");
      }
      if (lcc.tap_control_modelled) {
        const auto transformer = std::find_if(
            sys.ac.branches.begin(), sys.ac.branches.end(),
            [&](const ACBranch& branch) {
              return branch.index == lcc.converter_transformer_branch;
            });
        const bool valid_transformer =
            transformer != sys.ac.branches.end() && transformer->in_service &&
            (transformer->from_bus == lcc.ac_bus ||
             transformer->to_bus == lcc.ac_bus);
        if (!valid_transformer) {
          lcc_errors.push_back(
              "[LCC-TAP-01] LCC station " + std::to_string(lcc.index) +
              " declares R-card tap control, but its bound T-card branch is "
              "missing, out of service, or does not touch the valve-side AC "
              "bus.");
        }
      }
    }
    if (!lcc_errors.empty()) {
      PowerFlowResult result;
      result.converged = false;
      result.diagnostics.termination_reason = "LCC station readiness check failed";
      for (auto& e : lcc_errors) {
        result.diagnostics.warnings.push_back(std::move(e));
      }
      return result;
    }
  }

  // ── Graph topology pre-check ──────────────────────────────────────────────
  // A GFM Norton internal phasor is an AC angle anchor even though its terminal
  // bus intentionally remains PQ. Preserve the legacy fast admission when any
  // graph island is valid; otherwise admit only when every AC NoSlack island
  // contains an enabled GFM-NCP port. Canonical Newton planning performs the
  // authoritative per-island check after projection.
  if (!sys.ac.buses.empty()) {
    namespace gr = hacdcpf::graph;
    const auto g    = gr::build_power_system_graph(sys);
    const auto topo = gr::analyze_topology(g);
    const bool has_valid = std::any_of(
        topo.islands.begin(), topo.islands.end(),
        [](const gr::IslandInfo& i) { return i.status == gr::IslandStatus::Valid; });
    const bool all_no_slack_ac_islands_have_gfm = std::all_of(
        topo.islands.begin(), topo.islands.end(), [&](const gr::IslandInfo& island) {
          if (island.ac_bus_ids.empty() ||
              island.status != gr::IslandStatus::NoSlack) {
            return true;
          }
          const std::unordered_set<int> ac_bus_ids(
              island.ac_bus_ids.begin(), island.ac_bus_ids.end());
          return std::any_of(
              sys.vsc_converters.begin(), sys.vsc_converters.end(),
              [&](const VSCConverter& converter) {
                return converter.in_service && converter.enable_limit_ncp &&
                       converter.i_ac_max_pu > 0.0 &&
                       converter.control_mode == ConverterMode::AC_GRID_FORMING &&
                       ac_bus_ids.count(converter.bus_ac) != 0;
              });
        });
    if (!has_valid && !all_no_slack_ac_islands_have_gfm) {
      PowerFlowResult result;
      result.converged = false;
      result.diagnostics.termination_reason =
          "No AC island with terminal slack or GFM Norton internal reference "
          "(graph pre-check)";
      for (const auto& diag : topo.diagnostics)
        result.diagnostics.warnings.push_back(diag.message);
      return result;
    }
  }
  double projection_ms = 0.0;
  double assembly_ms = 0.0;
  powerflow::SolverData& cached_data =
      get_cached_solver_data(sys, opt.loss_model, &projection_ms, &assembly_ms);
  const bool requests_lcc_tap_control =
      solver_data_requests_lcc_tap_control(cached_data);
  std::optional<powerflow::SolverData> controlled_data;
  if (requests_lcc_tap_control) {
    // Tap control rebuilds Ybus repeatedly. Work on a private snapshot so the
    // thread-local cache and the authored HybridPowerSystem remain immutable.
    controlled_data.emplace(cached_data);
  }
  powerflow::SolverData& data =
      controlled_data ? *controlled_data : cached_data;
  apply_zip_weights(data, opt);
  const InitialState* init_ptr = opt.initial_state ? &*opt.initial_state : nullptr;
  LCCTapControlStatusMap lcc_tap_statuses;
  PowerFlowResult result;
  if (requests_lcc_tap_control) {
    // A controlled solve uses a private SolverData snapshot whose stack address
    // can be reused by a later call. Keep its Newton pattern cache local to the
    // invocation so no stale mode/layout state can cross that address reuse.
    powerflow::NewtonSolver controlled_solver;
    result = solve_newton_with_lcc_tap_control(
        data, controlled_solver, opt, init_ptr, lcc_tap_statuses);
  } else {
    static thread_local powerflow::NewtonSolver solver;
    result = solver.solve(data, opt, init_ptr);
  }

  // Allgower & Georg (1990), sec. 2.1, and Facchinei & Pang (2003),
  // sec. 9.1: escalate a structurally closed but numerically failed root solve
  // through formulations with different basins before invoking continuation.
  // The DC seed changes only the initial AC angles; voltage magnitudes retain
  // the authored canonical profile. Every direct retry owns SolverData and a
  // NewtonSolver so active-set/NCP state cannot pollute the facade cache.
  SolverProfiling escalation_profile;
  merge_solver_profiling(escalation_profile, result.profiling);
  const PowerFlowResult direct_failure = result;
  const bool escalation_enabled =
      opt.robust_nonlinear.enable_homotopy_fallback_on_failure;
  if (escalation_enabled && !requests_lcc_tap_control &&
      eligible_for_nonlinear_escalation(result)) {
    int escalation_attempts = 0;
    const bool ncp_allowed =
        opt.enable_pv_pq_conversion || opt.enable_semi_smooth_newton;
    const auto accept_direct_retry = [&](PowerFlowResult attempt,
                                         const char* stage,
                                         const char* description) {
      merge_solver_profiling(escalation_profile, attempt.profiling);
      escalation_attempts += 1;
      escalation_profile.nonlinear_escalation_attempts = escalation_attempts;
      escalation_profile.ncp_fallback_attempted =
          escalation_profile.ncp_fallback_attempted ||
          std::string(stage).find("ncp") != std::string::npos;
      escalation_profile.dc_angle_seed_attempted =
          escalation_profile.dc_angle_seed_attempted ||
          std::string(stage).find("dc_angle") != std::string::npos;
      if (attempt.converged) {
        escalation_profile.successful_fallback_stage = stage;
        attempt.profiling = escalation_profile;
        attempt.diagnostics.warnings.push_back(
            "[PF-ESCALATION-01] Direct Newton did not converge (residual " +
            std::to_string(direct_failure.residual) + " after " +
            std::to_string(direct_failure.iterations) + " iterations" +
            (direct_failure.profiling.stagnation_detected
                 ? ", stagnation detected"
                 : "") +
            "); " + description + " converged to the requested full-system "
            "root.");
      }
      return attempt;
    };

    if (ncp_allowed && !opt.enable_semi_smooth_newton) {
      PowerFlowResult ncp_attempt = solve_isolated_newton_attempt(
          data, opt, init_ptr, /*enable_ncp=*/true);
      ncp_attempt = accept_direct_retry(
          std::move(ncp_attempt), "semi_smooth_ncp",
          "the semi-smooth NCP retry");
      if (ncp_attempt.converged) {
        result = std::move(ncp_attempt);
      }
    }

    std::optional<InitialState> dc_initial;
    if (!result.converged) {
      const powerflow::ACLinearizedDCResult dc_seed =
          powerflow::solve_ac_linearized_dc(data);
      if (dc_seed.success && dc_seed.va.size() == data.ac_buses.size()) {
        dc_initial = make_dc_angle_initial_state(data, dc_seed);
        PowerFlowResult fixed_attempt = solve_isolated_newton_attempt(
            data, opt, &*dc_initial, /*enable_ncp=*/false);
        fixed_attempt = accept_direct_retry(
            std::move(fixed_attempt), "dc_angle_fixed_active_set",
            "the linearized-DC angle seed plus fixed-layout active-set retry");
        if (fixed_attempt.converged) {
          result = std::move(fixed_attempt);
        }
      }
    }

    if (!result.converged && dc_initial && ncp_allowed) {
      PowerFlowResult dc_ncp_attempt = solve_isolated_newton_attempt(
          data, opt, &*dc_initial, /*enable_ncp=*/true);
      dc_ncp_attempt = accept_direct_retry(
          std::move(dc_ncp_attempt), "dc_angle_semi_smooth_ncp",
          "the linearized-DC angle seed plus semi-smooth NCP retry");
      if (dc_ncp_attempt.converged) {
        result = std::move(dc_ncp_attempt);
      }
    }
  }

  // Final escalation stage: ramp injections from a zero-injection base point
  // to lambda=1. Each nested solve disables this outer ladder, preventing
  // recursive NCP/DC/homotopy dispatch.
  if (escalation_enabled && eligible_for_nonlinear_escalation(result)) {
    PowerFlowOptions fallback_opt = opt;
    fallback_opt.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
    const powerflow::HomotopyContinuationSolver homotopy_solver;
    powerflow::HomotopyState homotopy_state;
    PowerFlowResult escalated =
        homotopy_solver.solve(sys, fallback_opt, homotopy_state);
    merge_solver_profiling(escalation_profile, escalated.profiling);
    escalation_profile.nonlinear_escalation_attempts += 1;
    escalation_profile.homotopy_fallback_attempted = true;
    if (escalated.converged) {
      escalation_profile.homotopy_fallback_succeeded = true;
      escalation_profile.successful_fallback_stage = "homotopy";
      escalated.profiling = escalation_profile;
      escalated.diagnostics.converter_coordination = coordination;
      escalated.diagnostics.warnings.push_back(
          "[PF-ESCALATION-02] The direct Newton/NCP/DC-seed attempts did not "
          "converge "
          "(residual " + std::to_string(result.residual) + " after " +
          std::to_string(result.iterations) + " iterations" +
          (result.profiling.stagnation_detected ? ", stagnation detected"
                                                : "") +
          "); homotopy continuation reached lambda=1 in " +
          std::to_string(homotopy_state.accepted_steps) + " accepted / " +
          std::to_string(homotopy_state.rejected_steps) +
          " rejected steps. The returned solution is the full-system "
          "(lambda=1) root.");
      return escalated;
    }
    result = direct_failure;
    result.profiling = escalation_profile;
    result.diagnostics.warnings.push_back(
        "[PF-ESCALATION-03] NCP, DC-angle seed, and homotopy fallbacks failed; "
        "homotopy stopped at "
        "lambda=" + std::to_string(homotopy_state.lambda) +
        " (accepted=" + std::to_string(homotopy_state.accepted_steps) +
        ", rejected=" + std::to_string(homotopy_state.rejected_steps) +
        "); the operating point is likely infeasible at full loading or "
        "requires a problem-specific initial state.");
  } else if (!result.converged && escalation_enabled) {
    result = direct_failure;
    result.profiling = escalation_profile;
  }

  result.diagnostics.converter_coordination = coordination;
  if (coordination.enabled) {
    for (const auto& issue : coordination.issues) {
      if (issue.severity == powerflow::CoordinationSeverity::Warning ||
          issue.severity == powerflow::CoordinationSeverity::Info) {
        result.diagnostics.warnings.push_back("[" + issue.rule_id + "] " + issue.message);
      }
    }
  }
  const auto derivation_start = std::chrono::steady_clock::now();
  populate_derived_results(data, result, opt.loss_model,
                           &lcc_tap_statuses);
  // Post-solve DC/DC duty-ratio feasibility (multi-converter §3.2): a converged
  // solution can still demand an infeasible voltage conversion for the declared
  // power-stage topology.
  if (result.converged) {
    for (const auto& tr : result.dcdc_transfers) {
      if (tr.duty_defined && !tr.duty_feasible) {
        result.diagnostics.warnings.push_back(
            "[DCDC-PHYS-01] DC/DC converter " + std::to_string(tr.index) +
            " duty ratio " + std::to_string(tr.duty) +
            " is outside its feasible window (Vout/Vin=" + std::to_string(tr.voltage_ratio) +
            "); the requested voltage conversion is infeasible for the declared topology.");
      }
    }
  }

  // A load-flow root and a physically feasible converter operating point are
  // different contracts. In strict mode, enforce the declared inequality set
  // as an acceptance guard. Redispatch is deliberately not attempted here:
  // changing PQ setpoints would turn this determined PF into an optimization.
  if (result.converged && opt.enforce_converter_physical_limits) {
    const bool vsc_violation = std::any_of(
        result.diagnostics.warnings.begin(), result.diagnostics.warnings.end(),
        [](const std::string& warning) {
          return warning.rfind("[ACDC-PHYS-", 0) == 0;
        });
    const bool dcdc_violation = std::any_of(
        result.dcdc_transfers.begin(), result.dcdc_transfers.end(),
        [](const DCDCTransfer& transfer) {
          return transfer.duty_defined && !transfer.duty_feasible;
        });
    if (vsc_violation || dcdc_violation) {
      result.converged = false;
      result.diagnostics.converged = false;
      result.diagnostics.termination_reason =
          "Converter physical-limit feasibility check failed";
      result.diagnostics.warnings.push_back(
          "[CONVERTER-PHYS-HARD] Newton equations converged, but the operating "
          "point was rejected because a declared converter inequality was violated; "
          "use OPF or revise converter controls/setpoints to obtain a feasible point.");
    }
  }
  restore_original_vsc_bus_ac(result, sys);
  restore_original_lcc_bus_ac(result, sys);
  restore_original_dc_bus_ids(result, sys);
  if (data.bus_merge_map) {
    unproject_pf_result(result, *data.bus_merge_map, sys.ac.branches,
                        authored_ac_base_mva(sys));
  } else if (result.branch_flows.size() > sys.ac.branches.size()) {
    result.branch_flows.resize(sys.ac.branches.size());
  }
  trim_internal_dc_bus_results(result, data.projection_certificate);

  // Terminal recovery consumes authored-space branch flows only. Running this
  // after bus/branch reprojection keeps meshed cuts in one declared index space.
  if (result.converged &&
      (!sys.ac.switches.empty() || !sys.ac.circuit_breakers.empty())) {
    if (result.branch_flows.size() != sys.ac.branches.size()) {
      throw std::runtime_error(
          "solve_power_flow: authored branch-flow recovery size mismatch");
    }
    auto df = compute_device_terminal_flows(sys, result.branch_flows);
    result.ac_switch_flows = std::move(df.ac_switches);
    result.ac_circuit_breaker_flows = std::move(df.ac_circuit_breakers);
  }

  const bool has_lcc_tap_control = std::any_of(
      lcc_tap_statuses.begin(), lcc_tap_statuses.end(),
      [](const auto& item) { return item.second.active; });
  populate_newton_converter_scope(
      data, result, opt, has_lcc_tap_control);
  result.profiling.projection_ms_total += projection_ms;
  result.profiling.assembly_ms_total += assembly_ms;
  result.profiling.result_derivation_ms_total +=
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - derivation_start)
          .count();
  result.profiling.facade_ms_total +=
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - facade_start)
          .count();
  const double classified_facade = result.profiling.solver_core_ms_total +
                                   result.profiling.projection_ms_total +
                                   result.profiling.assembly_ms_total +
                                   result.profiling.result_derivation_ms_total;
  result.profiling.unclassified_facade_ms_total =
      std::max(0.0, result.profiling.facade_ms_total - classified_facade);
  return result;
}

DCPowerFlowResult solve_dc_power_flow(const HybridPowerSystem& sys,
                                      const PowerFlowOptions& opt) {
  const auto reference_validation =
      validation::validate_reference_bus_eligibility(sys);
  if (reference_validation.has_errors()) return {};
  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  static thread_local powerflow::DCSolver solver;
  auto result = solver.solve(data, opt, nullptr);
  recover_dc_bus_voltages(result.vdc, data.projection_certificate);
  return result;
}

AdaptiveSolveResult solve_power_flow_adaptive(const HybridPowerSystem& sys,
                                              const PowerFlowOptions& opt) {
  powerflow::AdaptiveSolver solver;
  AdaptiveSolveResult result = solver.solve(sys, opt, {});
  if (result.converged) {
    powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
    apply_zip_weights(data, opt);
    std::vector<double> canonical_vm = result.vm;
    std::vector<double> canonical_va = result.va;
    if (data.bus_merge_map) {
      canonical_vm =
          project_intensive_bus_vector(result.vm, *data.bus_merge_map);
      canonical_va =
          project_intensive_bus_vector(result.va, *data.bus_merge_map);
    }
    PowerFlowResult physical;
    physical.branch_flows =
        powerflow::compute_branch_flows(data, canonical_vm, canonical_va);
    if (data.bus_merge_map) {
      unproject_branch_flows(physical, *data.bus_merge_map,
                             sys.ac.branches.size());
      restore_collapsed_branch_charging(
          physical, *data.bus_merge_map, sys.ac.branches,
          authored_ac_base_mva(sys), result.vm);
    } else if (physical.branch_flows.size() > sys.ac.branches.size()) {
      physical.branch_flows.resize(sys.ac.branches.size());
    }
    result.branch_flows = std::move(physical.branch_flows);
  }
  recover_dc_bus_voltages(
      result.vdc,
      get_cached_solver_data(sys, opt.loss_model).projection_certificate);
  return result;
}

IslandedSolveResult solve_power_flow_islanded(const HybridPowerSystem& sys,
                                              const PowerFlowOptions& opt) {
  IslandedSolveResult result;
  powerflow::AdaptiveSolver solver;
  const AdaptiveSolveResult adaptive = solver.solve(sys, opt, {});
  result.vm = adaptive.vm;
  result.va = adaptive.va;
  result.vdc = adaptive.vdc;
  recover_dc_bus_voltages(
      result.vdc,
      get_cached_solver_data(sys, opt.loss_model).projection_certificate);
  result.converged = adaptive.converged;
  result.iterations = adaptive.iterations;
  result.residual = adaptive.residual;
  result.islands = adaptive.islands;
  return result;
}

DistributedSlackResult solve_power_flow_distributed_slack(const HybridPowerSystem& sys,
                                                          const DistributedSlack& slack_cfg,
                                                          const PowerFlowOptions& opt) {
  powerflow::DistributedSlackSolver solver;
  auto result = solver.solve_simplified(sys, slack_cfg, opt);
  recover_dc_bus_voltages(
      result.vdc,
      get_cached_solver_data(sys, opt.loss_model).projection_certificate);
  return result;
}

DistributedSlackResult solve_power_flow_distributed_slack_full(const HybridPowerSystem& sys,
                                                               const DistributedSlack& slack_cfg,
                                                               const PowerFlowOptions& opt) {
  powerflow::DistributedSlackSolver solver;
  auto result = solver.solve_full_jacobian(sys, slack_cfg, opt);
  recover_dc_bus_voltages(
      result.vdc,
      get_cached_solver_data(sys, opt.loss_model).projection_certificate);
  return result;
}

SolverHandle* create_solver_handle(const HybridPowerSystem& sys, LossModelType loss_model) {
  auto handle = std::make_unique<SolverHandle>();
  rebuild_handle_data(*handle, sys, loss_model);
  return handle.release();
}

void reset_solver_handle(SolverHandle* handle,
                         const HybridPowerSystem& sys,
                         LossModelType loss_model) {
  if (handle == nullptr) {
    throw std::invalid_argument("reset_solver_handle: handle is null.");
  }
  rebuild_handle_data(*handle, sys, loss_model);
}

PowerFlowResult solve_handle(SolverHandle* handle, const PowerFlowOptions& opt) {
  if (handle == nullptr) {
    throw std::invalid_argument("solve_handle: handle is null.");
  }
  const auto facade_start = std::chrono::steady_clock::now();
  const auto finish_profiling = [&](PowerFlowResult& result,
                                    double derivation_ms) {
    result.profiling.result_derivation_ms_total += derivation_ms;
    if (handle->report_build_cost) {
      result.profiling.projection_ms_total += handle->pending_projection_ms;
      result.profiling.assembly_ms_total += handle->pending_assembly_ms;
      handle->report_build_cost = false;
    }
    result.profiling.prepared_session_rebuilds = handle->prepared_rebuilds;
    result.profiling.prepared_session_reuses = handle->prepared_reuses;
    result.profiling.prepared_session_numeric_refreshes =
        handle->prepared_numeric_refreshes;
    result.profiling.facade_ms_total +=
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - facade_start)
            .count();
    const double classified = result.profiling.solver_core_ms_total +
                              result.profiling.projection_ms_total +
                              result.profiling.assembly_ms_total +
                              result.profiling.result_derivation_ms_total;
    result.profiling.unclassified_facade_ms_total =
        std::max(0.0, result.profiling.facade_ms_total - classified);
  };
  handle->configured_loss_model = opt.loss_model;
  const InitialState* init_ptr = opt.initial_state ? &*opt.initial_state : nullptr;

  if (solver_data_requests_lcc_tap_control(handle->data)) {
    // The handle owns the authored fixed-tap snapshot. R-card control is a
    // per-solve operating action, so keep every accepted tap and rebuilt Ybus
    // in a private copy and leave repeated handle calls deterministic.
    powerflow::SolverData controlled_data = handle->data;
    controlled_data.loss_model = opt.loss_model;
    apply_zip_weights(controlled_data, opt);
    powerflow::NewtonSolver controlled_solver;
    LCCTapControlStatusMap tap_statuses;
    PowerFlowResult result = solve_newton_with_lcc_tap_control(
        controlled_data, controlled_solver, opt, init_ptr, tap_statuses);
    const auto derivation_start = std::chrono::steady_clock::now();
    populate_derived_results(controlled_data, result, opt.loss_model,
                             &tap_statuses);
    restore_original_vsc_bus_ac(result, handle->original_vsc_bus_ac);
    restore_original_lcc_bus_ac(result, handle->original_lcc_bus_ac);
    restore_original_dc_bus_ids(result, handle->original_vsc_bus_dc,
                                handle->original_lcc_bus_dc,
                                handle->original_dcdc_buses);
    if (controlled_data.bus_merge_map) {
      unproject_pf_result(result, *controlled_data.bus_merge_map,
                          handle->authored_ac_branches,
                          handle->authored_base_mva);
    } else if (result.branch_flows.size() >
               handle->authored_ac_branch_count) {
      result.branch_flows.resize(handle->authored_ac_branch_count);
    }
    trim_internal_dc_bus_results(result,
                                 controlled_data.projection_certificate);
    const bool has_lcc_tap_control = std::any_of(
        tap_statuses.begin(), tap_statuses.end(),
        [](const auto& item) { return item.second.active; });
    populate_newton_converter_scope(
        controlled_data, result, opt, has_lcc_tap_control);
    finish_profiling(
        result,
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - derivation_start)
            .count());
    return result;
  }

  handle->data.loss_model = opt.loss_model;
  apply_zip_weights(handle->data, opt);
  PowerFlowResult result =
      handle->newton_solver.solve(handle->data, opt, init_ptr);
  const auto derivation_start = std::chrono::steady_clock::now();
  populate_derived_results(handle->data, result, opt.loss_model);
  restore_original_vsc_bus_ac(result, handle->original_vsc_bus_ac);
  restore_original_lcc_bus_ac(result, handle->original_lcc_bus_ac);
  restore_original_dc_bus_ids(result, handle->original_vsc_bus_dc,
                              handle->original_lcc_bus_dc,
                              handle->original_dcdc_buses);
  if (handle->data.bus_merge_map) {
    unproject_pf_result(result, *handle->data.bus_merge_map,
                        handle->authored_ac_branches,
                        handle->authored_base_mva);
  } else if (result.branch_flows.size() > handle->authored_ac_branch_count) {
    result.branch_flows.resize(handle->authored_ac_branch_count);
  }
  trim_internal_dc_bus_results(result, handle->data.projection_certificate);
  populate_newton_converter_scope(handle->data, result, opt);
  finish_profiling(
      result,
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - derivation_start)
          .count());
  return result;
}

DCPowerFlowResult solve_dc_handle(SolverHandle* handle, const PowerFlowOptions& opt) {
  if (handle == nullptr) {
    throw std::invalid_argument("solve_dc_handle: handle is null.");
  }
  handle->data.loss_model = opt.loss_model;
  handle->configured_loss_model = opt.loss_model;
  auto result = handle->dc_solver.solve(handle->data, opt, nullptr);
  if (handle->data.projection_certificate) {
    const size_t authored = static_cast<size_t>(
        handle->data.projection_certificate->n_authored_dc_buses);
    if (result.vdc.size() > authored) result.vdc.resize(authored);
  }
  return result;
}

void destroy_solver_handle(SolverHandle* handle) {
  delete handle;
}

struct PreparedPowerFlowSession::Impl {
  explicit Impl(PowerFlowOptions options_in)
      : options(std::move(options_in)) {}

  PowerFlowOptions options;
  std::unique_ptr<SolverHandle> handle;
};

PreparedPowerFlowSession::PreparedPowerFlowSession(PowerFlowOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
PreparedPowerFlowSession::~PreparedPowerFlowSession() = default;
PreparedPowerFlowSession::PreparedPowerFlowSession(
    PreparedPowerFlowSession&&) noexcept = default;
PreparedPowerFlowSession& PreparedPowerFlowSession::operator=(
    PreparedPowerFlowSession&&) noexcept = default;

PowerFlowResult PreparedPowerFlowSession::solve(
    const HybridPowerSystem& sys) {
  const std::uint64_t signature =
      hash_system_signature(sys, impl_->options.loss_model);
  if (!impl_->handle) {
    impl_->handle.reset(create_solver_handle(sys, impl_->options.loss_model));
  } else if (impl_->handle->system_signature == signature) {
    impl_->handle->prepared_reuses += 1;
  } else if (powerflow::refresh_solver_data_values(impl_->handle->data, sys)) {
    impl_->handle->system_signature = signature;
    impl_->handle->authored_ac_branches = sys.ac.branches;
    impl_->handle->authored_ac_branch_count = sys.ac.branches.size();
    impl_->handle->prepared_reuses += 1;
    impl_->handle->prepared_numeric_refreshes += 1;
  } else {
    rebuild_handle_data(*impl_->handle, sys, impl_->options.loss_model);
  }
  return solve_handle(impl_->handle.get(), impl_->options);
}

void PreparedPowerFlowSession::reset() {
  PowerFlowOptions options = impl_->options;
  impl_ = std::make_unique<Impl>(std::move(options));
}

PowerFlowResult solve_power_flow_fdpf(const HybridPowerSystem& sys,
                                       const PowerFlowOptions& opt) {
  const bool has_hybrid_assets =
      !sys.dc.buses.empty() || !sys.dc.branches.empty() ||
      !sys.vsc_converters.empty() || !sys.lcc_converters.empty() ||
      !sys.dc.dcdc_converters.empty() || !sys.energy_routers.empty();
  if (has_hybrid_assets) {
    PowerFlowResult result;
    result.diagnostics.termination_reason =
        "FDPF facade supports the balanced AC subsystem only";
    result.diagnostics.warnings.push_back(
        "FDPF was not run because the model contains DC buses or converter "
        "assets; use unified Newton or Newton-Krylov.");
    result.converter_model_scope.model_scope =
        "ac-only-fdpf:not-applicable-to-hybrid";
    return result;
  }
  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  apply_zip_weights(data, opt);
  powerflow::FDPFSolver solver;
  PowerFlowResult result = solver.solve(data, opt);
  populate_derived_results(data, result, opt.loss_model);
  restore_original_vsc_bus_ac(result, sys);
  restore_original_lcc_bus_ac(result, sys);
  restore_original_dc_bus_ids(result, sys);
  if (data.bus_merge_map) {
    unproject_pf_result(result, *data.bus_merge_map, sys.ac.branches,
                        authored_ac_base_mva(sys));
  } else if (result.branch_flows.size() > sys.ac.branches.size()) {
    result.branch_flows.resize(sys.ac.branches.size());
  }
  trim_internal_dc_bus_results(result, data.projection_certificate);
  return result;
}

PowerFlowResult solve_power_flow_helm(
    const HybridPowerSystem& sys,
    const PowerFlowOptions& opt,
    const powerflow::HelmOptions& helm_opt) {
  const bool has_hybrid_assets =
      !sys.dc.buses.empty() || !sys.dc.branches.empty() ||
      !sys.vsc_converters.empty() || !sys.lcc_converters.empty() ||
      !sys.dc.dcdc_converters.empty() || !sys.energy_routers.empty();
  if (has_hybrid_assets) {
    PowerFlowResult result;
    result.diagnostics.termination_reason =
        "HELM facade supports the balanced AC subsystem only";
    result.diagnostics.warnings.push_back(
        "HELM was not run because the model contains DC buses or AC/DC/DC-DC "
        "converter assets; use unified Newton, homotopy, or Newton-Krylov.");
    result.converter_model_scope.model_scope = "ac-only-helm:not-applicable-to-hybrid";
    return result;
  }

  const auto nonconstant_zip = [](const Load& load) {
    return load.in_service &&
           (std::abs(load.z_percent_p) > 1e-12 ||
            std::abs(load.i_percent_p) > 1e-12 ||
            std::abs(load.z_percent_q) > 1e-12 ||
            std::abs(load.i_percent_q) > 1e-12);
  };
  const bool option_zip =
      std::abs(opt.zip_pw[1]) > 1e-12 || std::abs(opt.zip_pw[2]) > 1e-12 ||
      std::abs(opt.zip_qw[1]) > 1e-12 || std::abs(opt.zip_qw[2]) > 1e-12;
  if (option_zip || std::any_of(sys.ac.loads.begin(), sys.ac.loads.end(),
                                nonconstant_zip)) {
    PowerFlowResult result;
    result.diagnostics.termination_reason =
        "HELM facade requires constant-power load models";
    result.diagnostics.warnings.push_back(
        "Voltage-dependent ZIP load terms are not embedded by this HELM "
        "implementation; the solve was rejected instead of silently treating "
        "them as constant power.");
    result.converter_model_scope.model_scope = "ac-only-helm:constant-power-loads";
    return result;
  }

  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  apply_zip_weights(data, opt);
  powerflow::HelmSolver solver;
  solver.helm_opts = helm_opt;
  PowerFlowResult result = solver.solve(data, opt);
  populate_derived_results(data, result, opt.loss_model);
  if (data.bus_merge_map) {
    unproject_pf_result(result, *data.bus_merge_map, sys.ac.branches,
                        authored_ac_base_mva(sys));
  } else if (result.branch_flows.size() > sys.ac.branches.size()) {
    result.branch_flows.resize(sys.ac.branches.size());
  }
  result.converter_model_scope.model_scope =
      "ac-only-helm:constant-power-pq-pv-slack";
  return result;
}

PowerFlowResult solve_power_flow_homotopy(
    const HybridPowerSystem& sys,
    const PowerFlowOptions& opt) {
  powerflow::HomotopyContinuationSolver solver;
  powerflow::HomotopyState state;
  PowerFlowResult result = solver.solve(sys, opt, state);
  result.diagnostics.warnings.push_back(
      "Explicit homotopy path: lambda=" + std::to_string(state.lambda) +
      ", accepted=" + std::to_string(state.accepted_steps) +
      ", rejected=" + std::to_string(state.rejected_steps) + ".");
  if (state.failed) {
    result.converged = false;
    result.diagnostics.converged = false;
    result.diagnostics.termination_reason =
        "Explicit homotopy path did not reach full loading";
  }
  return result;
}

PowerFlowResult solve_power_flow_newton_krylov(
    const HybridPowerSystem& sys,
    const PowerFlowOptions& opt) {
  PowerFlowOptions krylov_opt = opt;
  krylov_opt.robust_nonlinear.enable_condition_monitor = true;
  krylov_opt.robust_nonlinear.enable_newton_krylov_fallback = true;
  krylov_opt.robust_nonlinear.nk_condition_trigger = 0.0;
  return solve_power_flow(sys, krylov_opt);
}

powerflow::ACLinearizedDCResult solve_ac_dc_power_flow(const HybridPowerSystem& sys,
                                                        const PowerFlowOptions& opt) {
  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  auto result = powerflow::solve_ac_linearized_dc(data);
  if (data.bus_merge_map) {
    const auto& map = *data.bus_merge_map;
    result.va = unproject_bus_vector(
        result.va, map, BusVectorSemantics::Intensive);
    std::vector<double> authored(sys.ac.branches.size(), 0.0);
    for (const auto& [orig_pos, projected_pos] : map.branch_orig_to_proj) {
      if (orig_pos < 0 ||
          orig_pos >= static_cast<int>(authored.size()) ||
          projected_pos < 0 ||
          projected_pos >= static_cast<int>(result.pf_mw.size())) {
        continue;
      }
      authored[static_cast<size_t>(orig_pos)] =
          result.pf_mw[static_cast<size_t>(projected_pos)];
    }
    result.pf_mw = std::move(authored);
  } else if (result.pf_mw.size() > sys.ac.branches.size()) {
    result.pf_mw.resize(sys.ac.branches.size());
  }
  return result;
}

dynamics::DynamicResults run_transient_simulation(
    const HybridPowerSystem& sys,
    const dynamics::DynamicSolverOptions& opt) {
  return dynamics::run_transient_simulation(sys, opt);
}

opf::ACOPFResult solve_ac_opf(const HybridPowerSystem& sys, const opf::ACOPFOptions& opt) {
  opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);
  // Surface switch / circuit-breaker terminal flows at the OPF dispatch point.
  if (r.converged && (!sys.ac.switches.empty() || !sys.ac.circuit_breakers.empty())) {
    auto data = powerflow::make_solver_data(sys, LossModelType::Linear);
    std::vector<double> canonical_vm = r.vm;
    std::vector<double> canonical_va = r.va;
    if (data.bus_merge_map) {
      canonical_vm = project_intensive_bus_vector(r.vm, *data.bus_merge_map);
      canonical_va = project_intensive_bus_vector(r.va, *data.bus_merge_map);
    }
    PowerFlowResult physical;
    physical.branch_flows =
        powerflow::compute_branch_flows(data, canonical_vm, canonical_va);
    if (data.bus_merge_map) {
      unproject_branch_flows(physical, *data.bus_merge_map,
                             sys.ac.branches.size());
      restore_collapsed_branch_charging(
          physical, *data.bus_merge_map, sys.ac.branches,
          authored_ac_base_mva(sys), r.vm);
    } else if (physical.branch_flows.size() > sys.ac.branches.size()) {
      physical.branch_flows.resize(sys.ac.branches.size());
    }
    auto df = compute_device_terminal_flows(sys, physical.branch_flows);
    r.ac_switch_flows = std::move(df.ac_switches);
    r.ac_circuit_breaker_flows = std::move(df.ac_circuit_breakers);
  }
  return r;
}

opf::DCOPFResult solve_dc_opf(const HybridPowerSystem& sys, const opf::DCOPFOptions& opt) {
  return opf::solve_dc_opf(sys, opt);
}

opf::RPOResult solve_rpo(const HybridPowerSystem& sys, const opf::RPOOptions& opt) {
  return opf::solve_rpo(sys, opt);
}

analysis::DistributionResilienceResult run_distribution_resilience_assessment(
    const HybridPowerSystem& sys,
    const analysis::DistributionResilienceOptions& opt) {
  return analysis::run_distribution_resilience_assessment(sys, opt);
}

// ── PowerFlowOptions::from_parts ─────────────────────────────────────────────

PowerFlowOptions PowerFlowOptions::from_parts(
    const ConvergenceOptions&      conv,
    const LinearSolverOptions&     linear,
    const GlobalizationOptions&    glob,
    const PVPQSwitchingOptions&    switching,
    const ConverterModeOptions&    converter,
    const ZipLoadOptions&          zip,
    const RuntimeOptions&          runtime)
{
  PowerFlowOptions o;

  // Convergence
  o.max_iter      = conv.max_iter;
  o.tol           = conv.tol;
  o.fdpf_max_iter = conv.fdpf_max_iter;

  // Linear solver
  o.max_line_search_steps      = linear.max_line_search_steps;
  o.max_regularization_steps   = linear.max_regularization_steps;
  o.regularization_lambda0     = linear.regularization_lambda0;
  o.regularization_growth      = linear.regularization_growth;
  o.max_delta_va_rad            = linear.max_delta_va_rad;
  o.max_delta_vm_pu             = linear.max_delta_vm_pu;
  o.max_delta_vdc_pu            = linear.max_delta_vdc_pu;
  o.enable_coupled_jacobian     = linear.enable_coupled_jacobian;
  o.enable_augmented_equations  = linear.enable_augmented_equations;
  o.enable_semi_smooth_newton   = linear.enable_semi_smooth_newton;

  // Globalization
  using GS = PowerFlowOptions::GlobalizationStrategy;
  using GIn = GlobalizationOptions::Strategy;
  switch (glob.strategy) {
    case GIn::TrustRegion:    o.globalization = GS::TrustRegion;    break;
    case GIn::PseudoTransient: o.globalization = GS::PseudoTransient; break;
    default:                  o.globalization = GS::LineSearch;     break;
  }
  o.trust_region_radius0 = glob.trust_region_radius0;
  o.trust_region_max     = glob.trust_region_max;
  o.ptc_delta0           = glob.ptc_delta0;
  o.ptc_growth           = glob.ptc_growth;

  // PV/PQ switching
  o.pv_q_hysteresis_pu        = switching.pv_q_hysteresis_pu;
  o.pv_recover_vm_tol_pu      = switching.pv_recover_vm_tol_pu;
  o.pv_pq_max_outer_iterations = switching.pv_pq_max_outer_iterations;
  o.enable_pv_pq_conversion   = switching.enable_pv_pq_conversion;
  o.enable_auto_swing_selection = switching.enable_auto_swing_selection;

  // Converter mode
  o.converter_vdc_switch_high_pu    = converter.converter_vdc_switch_high_pu;
  o.converter_vdc_switch_low_pu     = converter.converter_vdc_switch_low_pu;
  o.mode_hysteresis_iters           = converter.mode_hysteresis_iters;
  o.enable_converter_mode_switching = converter.enable_converter_mode_switching;
  o.enable_converter_coordination_check = converter.enable_converter_coordination_check;
  o.loss_model                      = converter.loss_model;

  // ZIP
  for (int i = 0; i < 3; ++i) {
    o.zip_pw[i] = zip.pw[i];
    o.zip_qw[i] = zip.qw[i];
  }

  // Runtime
  o.ac_eval_threads         = runtime.ac_eval_threads;
  o.enable_solver_profiling = runtime.enable_solver_profiling;
  o.enable_iteration_log    = runtime.enable_iteration_log;
  o.verbose                 = runtime.verbose;

  return o;
}

// ── Solver capabilities ───────────────────────────────────────────────────────

SolverCapabilities get_solver_capabilities() noexcept {
  SolverCapabilities caps;
  // These are always compiled in:
  caps.has_sparse_lu      = true;
  caps.has_native_ipm     = true;
  caps.supports_ac_opf    = true;
  caps.supports_dc_opf    = true;
  caps.supports_three_phase = true;
  caps.supports_quadratic_objective = true;

  // Optional backends detected at compile time via HACDCPF_HAVE_* defines
  // propagated from the MIPSolvers sub-project.
#ifdef HACDCPF_HAVE_HIGHS_LIB
  caps.has_highs = true;
  caps.supports_integer_variables = true;
#endif
#ifdef HACDCPF_HAVE_GUROBI
  caps.has_gurobi = true;
  caps.supports_integer_variables = true;
#endif
#ifdef HACDCPF_HAVE_CPLEX
  caps.has_cplex = true;
  caps.supports_integer_variables = true;
#endif
#ifdef HACDCPF_HAVE_IPOPT
  caps.has_ipopt = true;
#endif
#ifdef HACDCPF_HAVE_SCIP_LIB
  caps.has_scip = true;
  caps.supports_integer_variables = true;
#endif
#ifdef HACDCPF_HAVE_PAPILO
  caps.has_papilo = true;
#endif
#ifdef HACDCPF_HAVE_KLU
  caps.has_klu = true;
#endif
#ifdef HACDCPF_HAVE_UMFPACK
  caps.has_umfpack = true;
#endif
#ifdef HACDCPF_HAVE_ACCELERATE
  caps.has_accelerate = true;
#endif
#ifdef HACDCPF_ENABLE_ETAP
  caps.has_excel = true;
#endif
#ifndef HACDCPF_NO_OPENDSS
  caps.has_opendss = true;
#endif

  return caps;
}

// ── OPF feasibility audit ─────────────────────────────────────────────────────

void verify_opf_result(const HybridPowerSystem& sys, opf::ACOPFResult& result) {
  opf::OpfAudit audit;
  audit.audited = true;
  audit.objective_reported = result.objective;

  const double base_mva = sys.base_mva > 1e-12 ? sys.base_mva : 100.0;

  // ── Check voltage limits ──────────────────────────────────────────────────
  for (int i = 0; i < static_cast<int>(sys.ac.buses.size()); ++i) {
    if (i >= static_cast<int>(result.vm.size())) break;
    const auto& bus = sys.ac.buses[i];
    const double vm  = result.vm[i];
    const double vio = std::max(0.0, std::max(bus.vmin_pu - vm, vm - bus.vmax_pu));
    if (vio > audit.max_voltage_limit_violation_pu) {
      audit.max_voltage_limit_violation_pu = vio;
      if (vio > 1e-4) {
        audit.violations.push_back(
            "Voltage at bus " + std::to_string(bus.index) +
            " = " + std::to_string(vm) + " pu, limits [" +
            std::to_string(bus.vmin_pu) + ", " + std::to_string(bus.vmax_pu) + "]");
      }
    }
  }

  // ── Check generator limits ────────────────────────────────────────────────
  for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
    if (i >= static_cast<int>(result.pg_mw.size())) break;
    const auto& gen = sys.ac.generators[i];
    if (!gen.in_service) continue;
    const double pg  = result.pg_mw[i];
    const double vio = std::max(0.0, std::max(gen.pmin_mw - pg, pg - gen.pmax_mw));
    if (vio > audit.max_gen_limit_violation_mw) {
      audit.max_gen_limit_violation_mw = vio;
      if (vio > 1e-3) {
        audit.violations.push_back(
            "Generator at bus " + std::to_string(gen.bus) +
            " pg=" + std::to_string(pg) + " MW outside [" +
            std::to_string(gen.pmin_mw) + ", " + std::to_string(gen.pmax_mw) + "]");
      }
    }
  }

  // ── Recompute objective from local generators and external-grid sources ───
  double obj_recomputed = 0.0;
  for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
    if (i >= static_cast<int>(result.pg_mw.size())) break;
    const auto& gen = sys.ac.generators[i];
    if (!gen.in_service) continue;
    const double pg = result.pg_mw[i];
    obj_recomputed += gen.cost_c2 * pg * pg + gen.cost_c1 * pg + gen.cost_c0;
  }
  for (int i = 0; i < static_cast<int>(sys.ac.external_grids.size()); ++i) {
    if (i >= static_cast<int>(result.external_grid_p_mw.size())) break;
    const auto& grid = sys.ac.external_grids[i];
    if (!grid.in_service) continue;
    const double pg = result.external_grid_p_mw[i];
    obj_recomputed +=
        grid.cost_c2 * pg * pg + grid.cost_c1 * pg + grid.cost_c0;
  }
  audit.objective_recomputed = obj_recomputed;
  if (std::fabs(audit.objective_reported) > 1e-10) {
    audit.objective_discrepancy_pct =
        std::fabs(obj_recomputed - audit.objective_reported) /
        std::fabs(audit.objective_reported) * 100.0;
  }

  // ── Infeasibility hints when not converged ────────────────────────────────
  if (!result.converged && result.infeasibility_hints.empty()) {
    // Check for load without source
    for (const auto& bus : sys.ac.buses) {
      bool has_source = false;
      for (const auto& gen : sys.ac.generators)
        if (gen.bus == bus.index && gen.in_service) { has_source = true; break; }
      if (!has_source) {
        for (const auto& eg : sys.ac.external_grids)
          if (eg.bus == bus.index && eg.in_service) { has_source = true; break; }
      }
      double load_p = 0.0;
      for (const auto& ld : sys.ac.loads)
        if (ld.bus == bus.index && ld.in_service) load_p += ld.p_mw;
      if (load_p > 1e-3 && !has_source)
        result.infeasibility_hints.push_back(
            "Bus " + std::to_string(bus.index) +
            " has load (" + std::to_string(load_p) + " MW) but no connected source");
    }
    // Check for conflicting limits
    for (const auto& gen : sys.ac.generators) {
      if (gen.pmin_mw > gen.pmax_mw + 1e-6)
        result.infeasibility_hints.push_back(
            "Generator at bus " + std::to_string(gen.bus) +
            ": pmin_mw (" + std::to_string(gen.pmin_mw) +
            ") > pmax_mw (" + std::to_string(gen.pmax_mw) + ")");
    }
    for (const auto& bus : sys.ac.buses) {
      if (bus.vmax_pu < bus.vmin_pu + 1e-6)
        result.infeasibility_hints.push_back(
            "Bus " + std::to_string(bus.index) +
            ": vmax_pu (" + std::to_string(bus.vmax_pu) +
            ") <= vmin_pu (" + std::to_string(bus.vmin_pu) + ")");
    }
  }

  result.audit = std::move(audit);
}

}  // namespace hacdcpf
