#include "hacdcpf/sppt/certificate.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <unordered_map>
#include <vector>

#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"

namespace hacdcpf::sppt {
namespace {

using Complex = std::complex<double>;

template <typename Container>
const typename Container::value_type* find_by_index(const Container& values, int index) {
  for (const auto& value : values)
    if (value.index == index) return &value;
  return nullptr;
}

bool primitive_scope_supported(const HybridPowerSystem& sys, std::string& note) {
  if (!sys.ac.transformers_2w.empty() || !sys.ac.transformers_3w.empty() ||
      !sys.ac.switches.empty() || !sys.ac.circuit_breakers.empty() ||
      !sys.ac.flexible_loads.empty() || !sys.ac.asymmetric_loads.empty() ||
      !sys.ac.motors.empty() || !sys.energy_routers.empty() || !sys.vpps.empty() ||
      !sys.microgrids.empty() || !sys.mobile_storage.empty() || sys.three_phase_ac) {
    note = "independent evaluator supports primitive AC/DC/VSC/DCDC models; rich macro expansion requires a separate terminal-equation oracle";
    return false;
  }
  return true;
}

}  // namespace

IndependentResidualCertificate certify_independent_hybrid_residual(
    const HybridPowerSystem& authored,
    const PowerFlowResult& result,
    double tolerance) {
  IndependentResidualCertificate cert;
  cert.scope = "authored-primitive-hybrid-acdc-v1";
  cert.converged = result.converged;
  if (!primitive_scope_supported(authored, cert.note)) return cert;
  if (!result.converged) {
    cert.note = "production solve did not converge";
    return cert;
  }
  if (result.vm.size() != authored.ac.buses.size() ||
      result.va.size() != authored.ac.buses.size() ||
      result.vdc.size() != authored.dc.buses.size()) {
    cert.note = "result vectors are not in authored bus-position space";
    return cert;
  }

  const double base = authored.base_mva > 0.0 ? authored.base_mva : 100.0;
  const int nac = static_cast<int>(authored.ac.buses.size());
  const int ndc = static_cast<int>(authored.dc.buses.size());
  std::unordered_map<int, int> ac_pos;
  std::unordered_map<int, int> dc_pos;
  for (int i = 0; i < nac; ++i) ac_pos[authored.ac.buses[static_cast<std::size_t>(i)].index] = i;
  for (int i = 0; i < ndc; ++i) dc_pos[authored.dc.buses[static_cast<std::size_t>(i)].index] = i;

  std::vector<std::vector<Complex>> ybus(
      static_cast<std::size_t>(nac),
      std::vector<Complex>(static_cast<std::size_t>(nac), Complex{}));
  for (const auto& branch : authored.ac.branches) {
    if (!branch.in_service) continue;
    const auto fi = ac_pos.find(branch.from_bus);
    const auto ti = ac_pos.find(branch.to_bus);
    const Complex z(branch.r_pu, branch.x_pu);
    if (fi == ac_pos.end() || ti == ac_pos.end() || std::abs(z) == 0.0) continue;
    const Complex ys = 1.0 / z;
    const Complex ytt = ys + Complex(0.0, branch.b_pu / 2.0);
    const double tap_magnitude = std::abs(branch.tap) < 1e-12 ? 1.0 : branch.tap;
    const double shift = branch.shift_deg * std::acos(-1.0) / 180.0;
    const Complex tap = std::polar(tap_magnitude, shift);
    const int i = fi->second;
    const int j = ti->second;
    ybus[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] +=
        ytt / std::norm(tap);
    ybus[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] += ytt;
    ybus[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -=
        ys / std::conj(tap);
    ybus[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] -= ys / tap;
  }
  for (int i = 0; i < nac; ++i) {
    const auto& bus = authored.ac.buses[static_cast<std::size_t>(i)];
    ybus[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] +=
        Complex(bus.gs_mw, bus.bs_mvar) / base;
  }
  for (const auto& shunt : authored.ac.shunts) {
    if (!shunt.in_service) continue;
    const auto position = ac_pos.find(shunt.bus);
    if (position == ac_pos.end()) continue;
    const double bs = shunt.switchable && shunt.n_steps > 0
                          ? shunt.bs_per_step * shunt.current_step
                          : shunt.bs_mvar;
    ybus[static_cast<std::size_t>(position->second)]
        [static_cast<std::size_t>(position->second)] +=
        Complex(shunt.gs_mw, bs) / base;
  }

  std::vector<Complex> voltage(static_cast<std::size_t>(nac));
  std::vector<Complex> network_power(static_cast<std::size_t>(nac));
  for (int i = 0; i < nac; ++i)
    voltage[static_cast<std::size_t>(i)] =
        std::polar(result.vm[static_cast<std::size_t>(i)],
                   result.va[static_cast<std::size_t>(i)]);
  for (int i = 0; i < nac; ++i) {
    Complex current{};
    for (int j = 0; j < nac; ++j)
      current += ybus[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                 voltage[static_cast<std::size_t>(j)];
    network_power[static_cast<std::size_t>(i)] =
        voltage[static_cast<std::size_t>(i)] * std::conj(current);
  }

  std::vector<double> p_spec(static_cast<std::size_t>(nac), 0.0);
  std::vector<double> q_spec(static_cast<std::size_t>(nac), 0.0);
  for (int i = 0; i < nac; ++i) {
    p_spec[static_cast<std::size_t>(i)] -= authored.ac.buses[static_cast<std::size_t>(i)].pd_mw;
    q_spec[static_cast<std::size_t>(i)] -= authored.ac.buses[static_cast<std::size_t>(i)].qd_mvar;
  }
  for (const auto& load : authored.ac.loads) {
    if (!load.in_service) continue;
    const auto position = ac_pos.find(load.bus);
    if (position == ac_pos.end()) continue;
    const double v = result.vm[static_cast<std::size_t>(position->second)];
    const double p_factor = (load.z_percent_p * v * v + load.i_percent_p * v +
                             load.p_percent_p) /
                            100.0;
    const double q_factor = (load.z_percent_q * v * v + load.i_percent_q * v +
                             load.p_percent_q) /
                            100.0;
    p_spec[static_cast<std::size_t>(position->second)] -=
        load.p_mw * load.scaling * p_factor;
    q_spec[static_cast<std::size_t>(position->second)] -=
        load.q_mvar * load.scaling * q_factor;
  }
  auto add_ac = [&](int bus, double p, double q) {
    const auto position = ac_pos.find(bus);
    if (position == ac_pos.end()) return;
    p_spec[static_cast<std::size_t>(position->second)] += p;
    q_spec[static_cast<std::size_t>(position->second)] += q;
  };
  for (const auto& gen : authored.ac.generators)
    if (gen.in_service) add_ac(gen.bus, gen.pg_mw, gen.qg_mvar);
  for (const auto& gen : authored.ac.static_generators)
    if (gen.in_service) add_ac(gen.bus, gen.p_mw * gen.scaling, gen.q_mvar * gen.scaling);
  for (const auto& gen : authored.ac.renewable_gens)
    if (gen.in_service) add_ac(gen.bus, gen.p_mw, gen.q_mvar);
  for (const auto& pv : authored.ac.pv_systems)
    if (pv.in_service) add_ac(pv.bus, powerflow::compute_pv_power_mw(pv), pv.q_mvar);
  for (const auto& storage : authored.ac.storage)
    if (storage.in_service) add_ac(storage.bus, storage.p_mw, storage.q_mvar);
  for (const auto& station : authored.ac.charging_stations)
    if (station.in_service)
      add_ac(station.bus, -station.p_total_kw / 1000.0,
             -station.q_total_kvar / 1000.0);
  for (const auto& transfer : result.vsc_transfers)
    add_ac(transfer.bus_ac, transfer.p_ac_mw, transfer.q_ac_mvar);

  std::vector<bool> slack(static_cast<std::size_t>(nac), false);
  std::vector<bool> voltage_controlled(static_cast<std::size_t>(nac), false);
  for (int i = 0; i < nac; ++i) {
    slack[static_cast<std::size_t>(i)] =
        authored.ac.buses[static_cast<std::size_t>(i)].bus_type == BusType::SLACK;
    voltage_controlled[static_cast<std::size_t>(i)] =
        authored.ac.buses[static_cast<std::size_t>(i)].bus_type == BusType::PV;
  }
  for (const auto& grid : authored.ac.external_grids) {
    const auto position = ac_pos.find(grid.bus);
    if (grid.in_service && position != ac_pos.end())
      slack[static_cast<std::size_t>(position->second)] = true;
  }
  for (const auto& converter : authored.vsc_converters) {
    if (!converter.in_service) continue;
    const auto position = ac_pos.find(converter.bus_ac);
    if (position == ac_pos.end()) continue;
    const DeviceControlRole role = resolve_device_control_role(converter);
    if (role.is_ac_grid_forming)
      slack[static_cast<std::size_t>(position->second)] = true;
    else if (role.controls_ac_v)
      voltage_controlled[static_cast<std::size_t>(position->second)] = true;
  }

  for (int i = 0; i < nac; ++i) {
    if (slack[static_cast<std::size_t>(i)]) continue;
    cert.ac_residual = std::max(
        cert.ac_residual,
        std::abs(p_spec[static_cast<std::size_t>(i)] / base -
                 network_power[static_cast<std::size_t>(i)].real()));
    if (!voltage_controlled[static_cast<std::size_t>(i)]) {
      cert.ac_residual = std::max(
          cert.ac_residual,
          std::abs(q_spec[static_cast<std::size_t>(i)] / base -
                   network_power[static_cast<std::size_t>(i)].imag()));
    }
  }

  std::vector<std::vector<double>> gdc(
      static_cast<std::size_t>(ndc),
      std::vector<double>(static_cast<std::size_t>(ndc), 0.0));
  for (const auto& branch : authored.dc.branches) {
    if (!branch.in_service || branch.r_pu == 0.0) continue;
    const auto fi = dc_pos.find(branch.from_bus);
    const auto ti = dc_pos.find(branch.to_bus);
    if (fi == dc_pos.end() || ti == dc_pos.end()) continue;
    const double g = 1.0 / branch.r_pu;
    const int i = fi->second;
    const int j = ti->second;
    gdc[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += g;
    gdc[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] += g;
    gdc[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -= g;
    gdc[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] -= g;
  }
  std::vector<double> pdc_spec(static_cast<std::size_t>(ndc), 0.0);
  if (authored.dc.loads.empty()) {
    for (int i = 0; i < ndc; ++i)
      pdc_spec[static_cast<std::size_t>(i)] -=
          authored.dc.buses[static_cast<std::size_t>(i)].pd_mw;
  } else {
    for (const auto& load : authored.dc.loads) {
      const auto position = dc_pos.find(load.bus);
      if (load.in_service && position != dc_pos.end())
        pdc_spec[static_cast<std::size_t>(position->second)] -= load.p_mw;
    }
  }
  auto add_dc = [&](int bus, double p) {
    const auto position = dc_pos.find(bus);
    if (position != dc_pos.end())
      pdc_spec[static_cast<std::size_t>(position->second)] += p;
  };
  for (const auto& gen : authored.dc.static_generators)
    if (gen.in_service) add_dc(gen.bus, gen.p_mw * gen.scaling);
  for (const auto& gen : authored.dc.dc_static_generators)
    if (gen.in_service) add_dc(gen.bus, gen.p_set_mw * gen.scaling);
  for (const auto& pv : authored.dc.pv_arrays)
    if (pv.in_service) add_dc(pv.bus, pv.p_set_mw);
  for (const auto& storage : authored.dc.storage)
    if (storage.in_service) add_dc(storage.bus, storage.p_mw);
  for (const auto& storage : authored.dc.dc_storage)
    if (storage.in_service) add_dc(storage.bus, storage.p_mw);
  for (const auto& transfer : result.vsc_transfers) add_dc(transfer.bus_dc, transfer.p_dc_mw);
  for (const auto& transfer : result.dcdc_transfers) {
    add_dc(transfer.bus_in, -transfer.p_in_mw);
    add_dc(transfer.bus_out, transfer.p_out_mw);
  }

  int dc_reference = -1;
  for (int i = 0; i < ndc; ++i) {
    if (authored.dc.buses[static_cast<std::size_t>(i)].bus_type == DCBusType::DC_V) {
      dc_reference = i;
      break;
    }
  }
  for (int i = 0; i < ndc; ++i) {
    if (i == dc_reference ||
        authored.dc.buses[static_cast<std::size_t>(i)].bus_type ==
            DCBusType::DC_ISOLATED)
      continue;
    double current = 0.0;
    for (int j = 0; j < ndc; ++j)
      current += gdc[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                 result.vdc[static_cast<std::size_t>(j)];
    const double calculated = result.vdc[static_cast<std::size_t>(i)] * current;
    cert.dc_residual = std::max(
        cert.dc_residual,
        std::abs(pdc_spec[static_cast<std::size_t>(i)] / base - calculated));
  }

  for (const auto& transfer : result.vsc_transfers) {
    const auto* converter = find_by_index(authored.vsc_converters, transfer.index);
    if (converter == nullptr) continue;
    const auto ac_position = ac_pos.find(transfer.bus_ac);
    const double vm = ac_position != ac_pos.end()
                          ? std::max(result.vm[static_cast<std::size_t>(ac_position->second)], 0.1)
                          : 1.0;
    double expected_loss =
        (1.0 - converter->eta) *
        std::abs(converter->control_mode == ConverterMode::PQ_MODE ||
                         converter->control_mode == ConverterMode::AC_PV
                     ? converter->p_set_mw
                     : -transfer.p_dc_mw);
    expected_loss += base * converter->r_conv_ac_pu *
                     std::pow(transfer.p_ac_mw / base, 2) / (vm * vm);
    cert.converter_balance_residual = std::max(
        cert.converter_balance_residual,
        std::abs(transfer.p_ac_mw + transfer.p_dc_mw + expected_loss) / base);
  }

  cert.total_residual =
      std::max({cert.ac_residual, cert.dc_residual,
                cert.converter_balance_residual});
  cert.supported = true;
  if (cert.total_residual > tolerance)
    cert.note = "independent authored-equation residual exceeds tolerance";
  return cert;
}

}  // namespace hacdcpf::sppt
