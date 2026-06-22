#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>

#include "hacdcpf/graph/graph.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"
#include "hacdcpf/power_flow/ac_linearized_pf.hpp"
#include "hacdcpf/power_flow/branch_flow.hpp"
#include "hacdcpf/power_flow/converter_coordination.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/dc_solver.hpp"
#include "hacdcpf/power_flow/fdpf_solver.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
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
    h = hash_combine(h, hash_double(c.i_dc_max_pu));
    h = hash_combine(h, hash_double(c.k_m_modulation));
    h = hash_combine(h, hash_double(c.m_min));
    h = hash_combine(h, hash_double(c.m_max));
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
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.charging_stations.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.chargers.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.external_grids.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.ac.motors.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.storage.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.static_generators.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.dc_static_generators.size()));
  h = hash_combine(h, static_cast<std::uint64_t>(sys.dc.pv_arrays.size()));
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

powerflow::SolverData& get_cached_solver_data(const HybridPowerSystem& sys, LossModelType loss_model) {
  thread_local SolverDataCache cache;

  // Hot-path reuse for repeated solves on an unchanged system object.
  // Requires pointer match, storage view match, AND hash consistency to
  // guard against dangling-pointer aliasing (e.g. a stack-local copy that
  // was destroyed and reallocated at the same address with different data).
  if (cache.valid && cache.system_ptr == &sys && cache.loss_model == loss_model &&
      same_storage_view(cache.storage, sys)) {
    const std::uint64_t sig = hash_system_signature(sys, loss_model);
    if (cache.signature == sig) {
      return cache.data;
    }
  }

  const std::uint64_t sig = hash_system_signature(sys, loss_model);
  if (!cache.valid || cache.signature != sig || cache.loss_model != loss_model) {
    cache.data = powerflow::make_solver_data(sys, loss_model);
    cache.signature = sig;
    cache.loss_model = loss_model;
    cache.valid = true;
  }
  cache.system_ptr = &sys;
  cache.storage = capture_storage_view(sys);
  return cache.data;
}

void rebuild_handle_data(SolverHandle& handle,
                         const HybridPowerSystem& sys,
                         LossModelType loss_model) {
  handle.data = powerflow::make_solver_data(sys, loss_model);
  handle.configured_loss_model = loss_model;
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

void populate_derived_results(const powerflow::SolverData& data,
                              PowerFlowResult& result,
                              LossModelType loss_model) {
  result.branch_flows.clear();
  result.vsc_transfers.clear();
  result.dcdc_transfers.clear();
  result.er_port_transfers.clear();
  if (!result.converged) {
    return;
  }

  result.branch_flows = powerflow::compute_branch_flows(data, result.vm, result.va);

  Eigen::VectorXd vm = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(result.vm.size()));
  Eigen::VectorXd va = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(result.va.size()));
  Eigen::VectorXd vdc = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(result.vdc.size()));
  for (Eigen::Index i = 0; i < vm.size(); ++i) vm[i] = result.vm[static_cast<size_t>(i)];
  for (Eigen::Index i = 0; i < va.size(); ++i) va[i] = result.va[static_cast<size_t>(i)];
  for (Eigen::Index i = 0; i < vdc.size(); ++i) vdc[i] = result.vdc[static_cast<size_t>(i)];

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

  // AC_PV reactive attribution: an AC_PV converter's AC bus is voltage-controlled
  // (PV), so the converter supplies the bus's reactive generation = network
  // reactive injection + local reactive demand.  Compute the network current
  // I = Ybus·V once, only when an AC_PV converter is present.  This credits the
  // bus's reactive balance to the converter; with several reactive devices on
  // one bus the attribution is shared (multi-converter model r1 §1.3) and this
  // single-source convention over-credits one converter.
  const bool has_ac_pv = std::any_of(
      eff_converters.begin(), eff_converters.end(), [](const VSCConverter& c) {
        return c.in_service && c.control_mode == ConverterMode::AC_PV;
      });
  Eigen::VectorXcd ybus_current;
  if (has_ac_pv && data.ybus.rows() == vm.size() && data.ybus.cols() == vm.size()) {
    Eigen::VectorXcd vbus(vm.size());
    for (Eigen::Index i = 0; i < vm.size(); ++i) vbus[i] = std::polar(vm[i], va[i]);
    ybus_current = data.ybus * vbus;
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
    double q_ac_pu_out = q_ac_pu;
    if (conv.control_mode == ConverterMode::AC_PV &&
        ybus_current.size() == vm.size()) {
      const int aci = conv.bus_ac - 1;
      if (aci >= 0 && aci < static_cast<int>(vm.size())) {
        const std::complex<double> v_i = std::polar(vm[aci], va[aci]);
        const double q_net_inj =
            (v_i * std::conj(ybus_current[static_cast<Eigen::Index>(aci)])).imag();
        const double qd_i = (aci < static_cast<int>(data.qd_pu.size()))
                                ? data.qd_pu[static_cast<size_t>(aci)]
                                : 0.0;
        q_ac_pu_out = q_net_inj + qd_i;
      }
    }

    VSCTransfer tr;
    tr.index = conv.index;
    tr.bus_ac = conv.bus_ac;
    tr.bus_dc = conv.bus_dc;
    tr.p_ac_mw = p_ac_pu * data.base_mva;
    tr.q_ac_mvar = q_ac_pu_out * data.base_mva;
    tr.p_dc_mw = p_dc_pu * data.base_mva;
    tr.loss_mw = -(tr.p_ac_mw + tr.p_dc_mw);
    result.vsc_transfers.push_back(tr);

    // ── Post-solve converter physical-limit feasibility (multi-converter model
    // §3.1.4–3.1.7).  Each check is opt-in — it only fires when the relevant
    // limit field is set (>0), so default systems are unaffected.  These are
    // diagnostics (warnings), not hard constraints in this determined solve.
    const int ac_pos = conv.bus_ac - 1;
    const int dc_pos = conv.bus_dc - 1;
    const double vm_ac =
        (ac_pos >= 0 && ac_pos < static_cast<int>(vm.size())) ? std::max(vm[ac_pos], 1e-3) : 1.0;
    const double vdc_b =
        (dc_pos >= 0 && dc_pos < static_cast<int>(vdc.size())) ? std::max(vdc[dc_pos], 1e-3) : 1.0;
    const double s_ac_pu = std::hypot(p_ac_pu, q_ac_pu);
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
      const double i_dc_pu = std::abs(p_dc_pu) / vdc_b;
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
    const double eta = (c.eta > 1e-9) ? c.eta : 1.0;
    const double p_out_mw = c.p_ref_mw;
    const double p_in_mw = (p_out_mw >= 0.0) ? (p_out_mw / eta) : (p_out_mw * eta);
    DCDCTransfer tr;
    tr.index = c.index;
    tr.bus_in = c.bus_in;
    tr.bus_out = c.bus_out;
    tr.p_in_mw = p_in_mw;
    tr.p_out_mw = p_out_mw;
    tr.loss_mw = p_in_mw - p_out_mw;
    // Duty-ratio feasibility from the solved port voltages (multi-converter §3.2).
    const int bi = c.bus_in - 1;
    const int bo = c.bus_out - 1;
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
        const int b = p.bus - 1;
        tr.v_pu = (b >= 0 && b < vm.size()) ? vm[b] : 1.0;
      } else {
        const int b = p.bus - 1;
        tr.v_pu = (b >= 0 && b < vdc.size()) ? vdc[b] : 1.0;
      }
      result.er_port_transfers.push_back(tr);
    }
  }
}

// Expand merged PF result vectors (vm, va) back to the original bus count
// so that callers can index by original bus position.
void unproject_pf_result(PowerFlowResult& result, const BusMergeMap& map) {
  if (!map.has_merges()) return;
  result.vm = unproject_bus_vector(result.vm, map);
  result.va = unproject_bus_vector(result.va, map);
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

  // ── Graph topology pre-check ──────────────────────────────────────────────
  // Return immediately (before Y-bus assembly) if no island has a slack bus.
  if (!sys.ac.buses.empty()) {
    namespace gr = hacdcpf::graph;
    const auto g    = gr::build_power_system_graph(sys);
    const auto topo = gr::analyze_topology(g);
    const bool has_valid = std::any_of(
        topo.islands.begin(), topo.islands.end(),
        [](const gr::IslandInfo& i) { return i.status == gr::IslandStatus::Valid; });
    if (!has_valid) {
      PowerFlowResult result;
      result.converged = false;
      result.diagnostics.termination_reason = "No AC island with slack bus (graph pre-check)";
      for (const auto& diag : topo.diagnostics)
        result.diagnostics.warnings.push_back(diag.message);
      return result;
    }
  }
  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  apply_zip_weights(data, opt);
  static thread_local powerflow::NewtonSolver solver;
  const InitialState* init_ptr = opt.initial_state ? &*opt.initial_state : nullptr;
  PowerFlowResult result = solver.solve(data, opt, init_ptr);
  result.diagnostics.converter_coordination = coordination;
  if (coordination.enabled) {
    for (const auto& issue : coordination.issues) {
      if (issue.severity == powerflow::CoordinationSeverity::Warning ||
          issue.severity == powerflow::CoordinationSeverity::Info) {
        result.diagnostics.warnings.push_back("[" + issue.rule_id + "] " + issue.message);
      }
    }
  }
  populate_derived_results(data, result, opt.loss_model);
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
  if (data.bus_merge_map) unproject_pf_result(result, *data.bus_merge_map);

  // Declare which parts of the unified converter model this snapshot Newton
  // solve honored (multi-converter model, docs/multiple_converter.md).
  {
    auto& sc = result.converter_model_scope;
    sc.model_scope = "steady-state-newton:vsc-3mode+dcdc-power-transfer";
    sc.validity.vsc_loss_modelled = true;
    sc.validity.vsc_ac_conduction_loss_modelled = true;  // r_conv_ac_pu coupling (opt-in)
    sc.validity.vsc_vdc_control_modelled = true;          // VDC_Q/VDC_VAC + stiff droop forming
    sc.validity.dcdc_loss_modelled = true;
    // Not yet enforced inside the Newton solve (post-hoc checks only):
    sc.validity.vsc_capacity_circle_enforced = false;
    sc.validity.vsc_current_limits_enforced = false;
    sc.validity.vsc_modulation_limits_enforced = false;
    sc.validity.dc_multisource_coordination_modelled = true;  // is_master + participation in solve
    sc.validity.dcdc_duty_ratio_enforced = false;  // computed post-solve, reported as warning
    sc.validity.equation_closure_checked = result.diagnostics.equation_closure_checked;
  }
  return result;
}

DCPowerFlowResult solve_dc_power_flow(const HybridPowerSystem& sys,
                                      const PowerFlowOptions& opt) {
  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  static thread_local powerflow::DCSolver solver;
  return solver.solve(data, opt, nullptr);
}

AdaptiveSolveResult solve_power_flow_adaptive(const HybridPowerSystem& sys,
                                              const PowerFlowOptions& opt) {
  powerflow::AdaptiveSolver solver;
  AdaptiveSolveResult result = solver.solve(sys, opt, {});
  if (result.converged) {
    powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
    apply_zip_weights(data, opt);
    result.branch_flows = powerflow::compute_branch_flows(data, result.vm, result.va);
  }
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
  return solver.solve_simplified(sys, slack_cfg, opt);
}

DistributedSlackResult solve_power_flow_distributed_slack_full(const HybridPowerSystem& sys,
                                                               const DistributedSlack& slack_cfg,
                                                               const PowerFlowOptions& opt) {
  powerflow::DistributedSlackSolver solver;
  return solver.solve_full_jacobian(sys, slack_cfg, opt);
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
  handle->data.loss_model = opt.loss_model;
  handle->configured_loss_model = opt.loss_model;
  apply_zip_weights(handle->data, opt);
  PowerFlowResult result = handle->newton_solver.solve(handle->data, opt, nullptr);
  populate_derived_results(handle->data, result, opt.loss_model);
  if (handle->data.bus_merge_map) unproject_pf_result(result, *handle->data.bus_merge_map);
  return result;
}

DCPowerFlowResult solve_dc_handle(SolverHandle* handle, const PowerFlowOptions& opt) {
  if (handle == nullptr) {
    throw std::invalid_argument("solve_dc_handle: handle is null.");
  }
  handle->data.loss_model = opt.loss_model;
  handle->configured_loss_model = opt.loss_model;
  return handle->dc_solver.solve(handle->data, opt, nullptr);
}

void destroy_solver_handle(SolverHandle* handle) {
  delete handle;
}

PowerFlowResult solve_power_flow_fdpf(const HybridPowerSystem& sys,
                                       const PowerFlowOptions& opt) {
  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  apply_zip_weights(data, opt);
  powerflow::FDPFSolver solver;
  PowerFlowResult result = solver.solve(data, opt);
  populate_derived_results(data, result, opt.loss_model);
  if (data.bus_merge_map) unproject_pf_result(result, *data.bus_merge_map);
  return result;
}

powerflow::ACLinearizedDCResult solve_ac_dc_power_flow(const HybridPowerSystem& sys,
                                                        const PowerFlowOptions& opt) {
  powerflow::SolverData& data = get_cached_solver_data(sys, opt.loss_model);
  return powerflow::solve_ac_linearized_dc(data);
}

opf::ACOPFResult solve_ac_opf(const HybridPowerSystem& sys, const opf::ACOPFOptions& opt) {
  return opf::solve_ac_opf(sys, opt);
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

  // ── Recompute objective (linear generation cost) ──────────────────────────
  double obj_recomputed = 0.0;
  for (int i = 0; i < static_cast<int>(sys.ac.generators.size()); ++i) {
    if (i >= static_cast<int>(result.pg_mw.size())) break;
    const auto& gen = sys.ac.generators[i];
    if (!gen.in_service) continue;
    // Linear cost term c1 * pg (MW)
    obj_recomputed += gen.cost_c1 * result.pg_mw[i];
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
