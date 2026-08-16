#include "hacdcpf/power_flow/vsc_limit_ncp.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <complex>
#include <numbers>

#include "hacdcpf/model/gfm_norton_contract.hpp"
#include "hacdcpf/power_flow/converter_model.hpp"
#include "hacdcpf/power_flow/ncp_functions.hpp"

namespace hacdcpf::powerflow {
namespace {

constexpr double kTieTolerance = 1e-12;
constexpr double kMinimumGFMImpedance = 1e-8;
using Complex = std::complex<double>;

struct MedianGradient {
  double da{0.0};
  double db{0.0};
  double dc{0.0};
};

double median_ncp(double value,
                  double reference,
                  double lower,
                  double upper,
                  double mu = 0.0) {
  if (mu > 0.0) {
    // Chen-Harker-Kanzow-Smale smoothing of max(min(a,c),b), with mu as
    // the smoothing radius; docs/vsc_limit_ncp_power_flow_contract.md.
    const double a = value - reference;
    const double b = value - upper;
    const double c = value - lower;
    const double inner_delta = a - c;
    const double inner = 0.5 *
        (a + c - std::sqrt(inner_delta * inner_delta + 4.0 * mu * mu));
    const double outer_delta = inner - b;
    return 0.5 *
        (inner + b +
         std::sqrt(outer_delta * outer_delta + 4.0 * mu * mu));
  }
  return std::max(std::min(value - reference, value - lower), value - upper);
}

MedianGradient median_ncp_gradient(double value,
                                   double reference,
                                   double lower,
                                   double upper,
                                   double mu = 0.0) {
  const double a = value - reference;
  const double b = value - upper;
  const double c = value - lower;
  if (mu > 0.0) {
    const double inner_delta = a - c;
    const double inner_radius =
        std::sqrt(inner_delta * inner_delta + 4.0 * mu * mu);
    const double inner = 0.5 * (a + c - inner_radius);
    const double inner_a = 0.5 * (1.0 - inner_delta / inner_radius);
    const double inner_c = 1.0 - inner_a;
    const double outer_delta = inner - b;
    const double outer_radius =
        std::sqrt(outer_delta * outer_delta + 4.0 * mu * mu);
    const double inner_weight =
        0.5 * (1.0 + outer_delta / outer_radius);
    return {inner_weight * inner_a,
            1.0 - inner_weight,
            inner_weight * inner_c};
  }
  const double inner = std::min(a, c);

  double inner_a = 0.0;
  double inner_c = 0.0;
  if (a < c - kTieTolerance) {
    inner_a = 1.0;
  } else if (c < a - kTieTolerance) {
    inner_c = 1.0;
  } else {
    inner_a = 0.5;
    inner_c = 0.5;
  }

  if (inner > b + kTieTolerance) return {inner_a, 0.0, inner_c};
  if (b > inner + kTieTolerance) return {0.0, 1.0, 0.0};
  return {0.5 * inner_a, 0.5, 0.5 * inner_c};
}

double effective_lower_mw(const VSCConverter& converter) {
  if (converter.droop_p_min_mw != 0.0 || converter.droop_p_max_mw != 0.0) {
    return converter.droop_p_min_mw;
  }
  return converter.pmin_mw;
}

double effective_upper_mw(const VSCConverter& converter) {
  if (converter.droop_p_min_mw != 0.0 || converter.droop_p_max_mw != 0.0) {
    return converter.droop_p_max_mw;
  }
  return converter.pmax_mw;
}

Complex gfm_impedance(const VSCConverter& converter) {
  return model::resolve_gfm_norton_parameters(converter).virtual_impedance();
}

Complex gfm_internal_reference(const VSCConverter& converter) {
  return model::resolve_gfm_norton_parameters(converter).internal_voltage();
}

struct GFMPort {
  Complex voltage;
  Complex internal;
  Complex current;
  Complex power;
  Complex impedance;
};

GFMPort gfm_port(const VSCConverter& converter,
                 double vm_pu,
                 double va_rad,
                 const Eigen::Matrix<double, kVSCLimitStateSize, 1>& state) {
  GFMPort port;
  port.voltage = std::polar(vm_pu, va_rad);
  port.internal = {state[3], state[4]};
  port.impedance = gfm_impedance(converter);
  port.current = (port.internal - port.voltage) / port.impedance;
  port.power = port.voltage * std::conj(port.current);
  return port;
}

double command_derivative_vdc(const VSCConverter& converter,
                               double vdc_pu,
                               double base_mva) {
  if (converter.control_mode != ConverterMode::VDC_Q) return 0.0;
  const double raw = converter.k_vdc *
      (vdc_pu * vdc_pu -
       converter.v_dc_set_pu * converter.v_dc_set_pu);
  const double lower = effective_lower_mw(converter) / base_mva;
  const double upper = effective_upper_mw(converter) / base_mva;
  if (upper > lower && (raw <= lower || raw >= upper)) return 0.0;
  return 2.0 * converter.k_vdc * vdc_pu;
}

}  // namespace

bool supports_vsc_limit_ncp(const VSCConverter& converter) {
  if (!converter.in_service || !converter.enable_limit_ncp ||
      !(converter.i_ac_max_pu > 0.0)) {
    return false;
  }
  if (converter.control_mode == ConverterMode::PQ_MODE ||
      converter.control_mode == ConverterMode::VDC_Q) return true;
  if (converter.control_mode != ConverterMode::AC_GRID_FORMING) return false;
  return std::abs(gfm_impedance(converter)) > kMinimumGFMImpedance;
}

bool uses_gfm_limit_ncp(const VSCConverter& converter) {
  return supports_vsc_limit_ncp(converter) &&
         converter.control_mode == ConverterMode::AC_GRID_FORMING;
}

const char* vsc_current_limit_priority_str(VSCCurrentLimitPriority priority) {
  switch (priority) {
    case VSCCurrentLimitPriority::Magnitude: return "magnitude";
    case VSCCurrentLimitPriority::ActivePower: return "active_power";
    case VSCCurrentLimitPriority::ReactivePower: return "reactive_power";
  }
  return "magnitude";
}

VSCCurrentLimitPriority vsc_current_limit_priority_from_str(
    const std::string& value) {
  if (value == "magnitude" || value == "vector_scaling") {
    return VSCCurrentLimitPriority::Magnitude;
  }
  if (value == "active_power" || value == "p_first") {
    return VSCCurrentLimitPriority::ActivePower;
  }
  if (value == "reactive_power" || value == "q_first") {
    return VSCCurrentLimitPriority::ReactivePower;
  }
  throw std::invalid_argument("Unknown VSC current-limit priority: " + value);
}

VSCLimitCommand vsc_limit_command(const VSCConverter& converter,
                                   double vdc_pu,
                                   double base_mva) {
  VSCLimitCommand command;
  command.q_ref_pu = converter.q_set_mvar / base_mva;
  if (converter.control_mode == ConverterMode::PQ_MODE) {
    command.p_ref_pu = converter.p_set_mw / base_mva;
    return command;
  }
  if (converter.control_mode != ConverterMode::VDC_Q) {
    throw std::invalid_argument(
        "VSC limit NCP supports PQ_MODE and VDC_Q in the balanced production path");
  }

  const double raw = converter.k_vdc *
      (vdc_pu * vdc_pu - converter.v_dc_set_pu * converter.v_dc_set_pu);
  const double lower = effective_lower_mw(converter) / base_mva;
  const double upper = effective_upper_mw(converter) / base_mva;
  if (upper > lower) {
    command.p_ref_pu = std::clamp(raw, lower, upper);
    command.droop_saturated = command.p_ref_pu != raw;
  } else {
    command.p_ref_pu = raw;
  }
  return command;
}

Eigen::Matrix<double, kVSCLimitStateSize, 1> initialize_vsc_limit_state(
    const VSCConverter& converter,
    double vm_pu,
    double va_rad,
    double vdc_pu,
    double base_mva,
    LossModelType loss_model) {
  if (uses_gfm_limit_ncp(converter)) {
    const Complex voltage = std::polar(vm_pu, va_rad);
    const Complex impedance = gfm_impedance(converter);
    const Complex reference = gfm_internal_reference(converter);
    Complex current = (reference - voltage) / impedance;
    const double current_norm = std::abs(current);
    if (current_norm > converter.i_ac_max_pu && current_norm > 0.0) {
      const Complex voltage_frame = std::polar(1.0, -va_rad);
      Complex rotating_current = current * voltage_frame;
      if (converter.current_limit_priority == VSCCurrentLimitPriority::Magnitude) {
        rotating_current *= converter.i_ac_max_pu / current_norm;
      } else if (converter.current_limit_priority == VSCCurrentLimitPriority::ActivePower) {
        rotating_current.real(std::clamp(rotating_current.real(),
                                         -converter.i_ac_max_pu,
                                         converter.i_ac_max_pu));
        const double q_cap = std::sqrt(std::max(
            0.0, converter.i_ac_max_pu * converter.i_ac_max_pu -
                     rotating_current.real() * rotating_current.real()));
        rotating_current.imag(std::clamp(rotating_current.imag(), -q_cap, q_cap));
      } else {
        rotating_current.imag(std::clamp(rotating_current.imag(),
                                         -converter.i_ac_max_pu,
                                         converter.i_ac_max_pu));
        const double p_cap = std::sqrt(std::max(
            0.0, converter.i_ac_max_pu * converter.i_ac_max_pu -
                     rotating_current.imag() * rotating_current.imag()));
        rotating_current.real(std::clamp(rotating_current.real(), -p_cap, p_cap));
      }
      current = rotating_current / voltage_frame;
    }
    const Complex internal = voltage + impedance * current;
    const Complex power = voltage * std::conj(current);
    const double loss = converter_loss(
        converter, power.real(), vdc_pu, base_mva, loss_model);
    Eigen::Matrix<double, kVSCLimitStateSize, 1> state;
    const double multiplier =
        converter.current_limit_priority == VSCCurrentLimitPriority::Magnitude &&
                converter.i_ac_max_pu > 0.0
            ? std::max(0.0, current_norm / converter.i_ac_max_pu - 1.0)
            : 0.0;
    state << power.real(), power.imag(), -(power.real() + loss),
        internal.real(), internal.imag(), multiplier;
    return state;
  }

  const VSCLimitCommand command =
      vsc_limit_command(converter, vdc_pu, base_mva);
  const double radius = std::max(vm_pu, 1e-8) * converter.i_ac_max_pu;
  double p = command.p_ref_pu;
  double q = command.q_ref_pu;
  double multiplier = 0.0;

  if (converter.current_limit_priority == VSCCurrentLimitPriority::Magnitude) {
    const double norm = std::hypot(p, q);
    if (norm > radius && norm > 0.0) {
      const double scale = radius / norm;
      p *= scale;
      q *= scale;
      multiplier = norm / radius - 1.0;
    }
  } else if (converter.current_limit_priority ==
             VSCCurrentLimitPriority::ActivePower) {
    p = std::clamp(p, -radius, radius);
    const double q_radius = std::sqrt(std::max(0.0, radius * radius - p * p));
    q = std::clamp(q, -q_radius, q_radius);
  } else {
    q = std::clamp(q, -radius, radius);
    const double p_radius = std::sqrt(std::max(0.0, radius * radius - q * q));
    p = std::clamp(p, -p_radius, p_radius);
  }

  const double loss = converter_loss(converter, p, vdc_pu, base_mva, loss_model);
  Eigen::Matrix<double, kVSCLimitStateSize, 1> state;
  state << p, q, -(p + loss), 0.0, 0.0, multiplier;
  return state;
}

VSCLimitEvaluation evaluate_vsc_limit_ncp(
    const VSCConverter& converter,
    double vm_pu,
    double va_rad,
    double vdc_pu,
    double base_mva,
    LossModelType loss_model,
    const Eigen::Matrix<double, kVSCLimitStateSize, 1>& state,
    double ncp_mu) {
  if (!supports_vsc_limit_ncp(converter)) {
    throw std::invalid_argument("VSC does not have a supported active limit-NCP contract");
  }

  VSCLimitEvaluation out;
  const double p = state[0];
  const double q = state[1];
  const double pdc = state[2];
  const double multiplier = state[5];
  const double mu = std::max(0.0, ncp_mu);
  const double vm = std::max(vm_pu, 1e-8);
  const double radius = vm * converter.i_ac_max_pu;
  const double margin = radius * radius - p * p - q * q;
  if (uses_gfm_limit_ncp(converter)) {
    // Kundur (1994), ch. 12; internal contract
    // docs/vsc_limit_ncp_power_flow_contract.md: S=V*conj((E-V)/Zv).
    const GFMPort port = gfm_port(converter, vm_pu, va_rad, state);
    const Complex y = 1.0 / port.impedance;
    const Complex d_v_dvm = std::polar(1.0, va_rad);
    const Complex d_v_dva = Complex{0.0, 1.0} * port.voltage;
    const auto power_derivative = [&](Complex dv, Complex de) {
      const Complex di = (de - dv) * y;
      return dv * std::conj(port.current) + port.voltage * std::conj(di);
    };
    const Complex ds_de_re = power_derivative({}, {1.0, 0.0});
    const Complex ds_de_im = power_derivative({}, {0.0, 1.0});
    const Complex ds_dvm = power_derivative(d_v_dvm, {});
    const Complex ds_dva = power_derivative(d_v_dva, {});
    out.equation[0] = p - port.power.real();
    out.equation[1] = q - port.power.imag();
    out.jacobian(0, 0) = 1.0;
    out.jacobian(1, 1) = 1.0;
    out.jacobian(0, 3) = -ds_de_re.real();
    out.jacobian(0, 4) = -ds_de_im.real();
    out.jacobian(1, 3) = -ds_de_re.imag();
    out.jacobian(1, 4) = -ds_de_im.imag();
    out.derivative_vm[0] = -ds_dvm.real();
    out.derivative_vm[1] = -ds_dvm.imag();
    out.derivative_va[0] = -ds_dva.real();
    out.derivative_va[1] = -ds_dva.imag();

    const auto [dloss_dp, dloss_dvdc] = converter_loss_jacobian(
        converter, p, vdc_pu, base_mva, loss_model);
    out.equation[2] = pdc + p +
        converter_loss(converter, p, vdc_pu, base_mva, loss_model);
    out.jacobian(2, 0) = 1.0 + dloss_dp;
    out.jacobian(2, 2) = 1.0;
    out.derivative_vdc[2] = dloss_dvdc;

    const Complex reference = gfm_internal_reference(converter);
    const Complex voltage_drop = port.internal - port.voltage;
    const Complex reference_drop = reference - port.voltage;
    if (converter.current_limit_priority == VSCCurrentLimitPriority::Magnitude) {
      // Facchinei & Pang (2003), vol. I, sec. 9.1. The scalar NCP multiplier
      // increases virtual impedance: (1+lambda)(E-V)=E0-V.
      const Complex control = (1.0 + multiplier) * voltage_drop - reference_drop;
      out.equation[3] = control.real();
      out.equation[4] = control.imag();
      out.jacobian(3, 3) = 1.0 + multiplier;
      out.jacobian(4, 4) = 1.0 + multiplier;
      out.jacobian(3, 5) = voltage_drop.real();
      out.jacobian(4, 5) = voltage_drop.imag();
      out.derivative_vm[3] = -multiplier * d_v_dvm.real();
      out.derivative_vm[4] = -multiplier * d_v_dvm.imag();
      out.derivative_va[3] = -multiplier * d_v_dva.real();
      out.derivative_va[4] = -multiplier * d_v_dva.imag();
      const double margin = converter.i_ac_max_pu * converter.i_ac_max_pu -
                            std::norm(port.current);
      out.equation[5] = mu > 0.0
          ? smooth_fb(multiplier, margin, mu)
          : fischer_burmeister(multiplier, margin);
      const auto [dphi_dl, dphi_dm] =
          mu > 0.0
              ? smooth_fb_derivatives(multiplier, margin, mu)
              : fischer_burmeister_jacobian(multiplier, margin);
      const auto margin_derivative = [&](Complex di) {
        return -2.0 * std::real(std::conj(port.current) * di);
      };
      out.jacobian(5, 3) = dphi_dm * margin_derivative(y);
      out.jacobian(5, 4) = dphi_dm * margin_derivative(Complex{0.0, 1.0} * y);
      out.jacobian(5, 5) = dphi_dl;
      out.derivative_vm[5] =
          dphi_dm * margin_derivative(-d_v_dvm * y);
      out.derivative_va[5] =
          dphi_dm * margin_derivative(-d_v_dva * y);
    } else {
      const Complex rotation = std::polar(1.0, -va_rad);
      const Complex current_dq = port.current * rotation;
      const Complex reference_current = (reference - port.voltage) * y * rotation;
      const auto component_equation = [&](double value, double target,
                                          double lower, double upper) {
        return median_ncp(value, target, lower, upper, mu);
      };
      if (converter.current_limit_priority == VSCCurrentLimitPriority::ActivePower) {
        out.equation[3] = component_equation(current_dq.real(),
            reference_current.real(), -converter.i_ac_max_pu, converter.i_ac_max_pu);
        const double cap = std::sqrt(std::max(0.0,
            converter.i_ac_max_pu * converter.i_ac_max_pu -
            current_dq.real() * current_dq.real()));
        out.equation[4] = component_equation(current_dq.imag(),
            reference_current.imag(), -cap, cap);
      } else {
        out.equation[4] = component_equation(current_dq.imag(),
            reference_current.imag(), -converter.i_ac_max_pu, converter.i_ac_max_pu);
        const double cap = std::sqrt(std::max(0.0,
            converter.i_ac_max_pu * converter.i_ac_max_pu -
            current_dq.imag() * current_dq.imag()));
        out.equation[3] = component_equation(current_dq.real(),
            reference_current.real(), -cap, cap);
      }
      // Facchinei & Pang (2003), vol. I, sec. 9.1: chain one selected
      // generalized derivative of the median-NCP through the terminal-voltage
      // synchronous frame. Coordinate order is Ere, Eim, Vm, Va.
      std::array<Complex, 4> d_actual{
          y * rotation,
          Complex{0.0, 1.0} * y * rotation,
          -y,
          -Complex{0.0, 1.0} * (vm_pu * y + current_dq)};
      std::array<Complex, 4> d_target{
          Complex{}, Complex{}, -y,
          -Complex{0.0, 1.0} * (vm_pu * y + reference_current)};
      std::array<double, 4> primary_derivative{};
      const auto stamp_primary = [&](int row, bool real_component) {
        const double value = real_component ? current_dq.real() : current_dq.imag();
        const double target = real_component
            ? reference_current.real() : reference_current.imag();
        const MedianGradient gradient = median_ncp_gradient(
            value, target, -converter.i_ac_max_pu,
            converter.i_ac_max_pu, mu);
        for (int coordinate = 0; coordinate < 4; ++coordinate) {
          const double dvalue = real_component
              ? d_actual[coordinate].real() : d_actual[coordinate].imag();
          const double dtarget = real_component
              ? d_target[coordinate].real() : d_target[coordinate].imag();
          primary_derivative[coordinate] = dvalue;
          const double derivative =
              (gradient.da + gradient.db + gradient.dc) * dvalue -
              gradient.da * dtarget;
          if (coordinate < 2) out.jacobian(row, 3 + coordinate) = derivative;
          else if (coordinate == 2) out.derivative_vm[row] = derivative;
          else out.derivative_va[row] = derivative;
        }
      };
      const auto stamp_secondary = [&](int row, bool real_component,
                                       double primary_value) {
        const double cap = std::sqrt(std::max(
            0.0, converter.i_ac_max_pu * converter.i_ac_max_pu -
                     primary_value * primary_value));
        const double value = real_component ? current_dq.real() : current_dq.imag();
        const double target = real_component
            ? reference_current.real() : reference_current.imag();
        const MedianGradient gradient =
            median_ncp_gradient(value, target, -cap, cap, mu);
        for (int coordinate = 0; coordinate < 4; ++coordinate) {
          const double dvalue = real_component
              ? d_actual[coordinate].real() : d_actual[coordinate].imag();
          const double dtarget = real_component
              ? d_target[coordinate].real() : d_target[coordinate].imag();
          const double dcap = cap > kTieTolerance
              ? -primary_value * primary_derivative[coordinate] / cap : 0.0;
          const double derivative =
              (gradient.da + gradient.db + gradient.dc) * dvalue -
              gradient.da * dtarget + (gradient.dc - gradient.db) * dcap;
          if (coordinate < 2) out.jacobian(row, 3 + coordinate) = derivative;
          else if (coordinate == 2) out.derivative_vm[row] = derivative;
          else out.derivative_va[row] = derivative;
        }
      };
      if (converter.current_limit_priority == VSCCurrentLimitPriority::ActivePower) {
        stamp_primary(3, true);
        stamp_secondary(4, false, current_dq.real());
      } else {
        stamp_primary(4, false);
        stamp_secondary(3, true, current_dq.imag());
      }
      out.equation[5] = multiplier;
      out.jacobian(5, 5) = 1.0;
    }
    out.current_pu = std::abs(port.current);
    out.current_margin_pu = converter.i_ac_max_pu - out.current_pu;
    out.complementarity_residual = out.equation.cwiseAbs().maxCoeff();
    out.current_limit_active = out.current_margin_pu <= 1e-8 || multiplier > 1e-8;
    return out;
  }

  const VSCLimitCommand command =
      vsc_limit_command(converter, vdc_pu, base_mva);
  const double dpref_dvdc =
      command_derivative_vdc(converter, vdc_pu, base_mva);

  if (converter.current_limit_priority == VSCCurrentLimitPriority::Magnitude) {
    out.equation[0] = p * (1.0 + multiplier) - command.p_ref_pu;
    out.equation[1] = q * (1.0 + multiplier) - command.q_ref_pu;
    out.jacobian(0, 0) = 1.0 + multiplier;
    out.jacobian(0, 5) = p;
    out.jacobian(1, 1) = 1.0 + multiplier;
    out.jacobian(1, 5) = q;
    out.derivative_vdc[0] = -dpref_dvdc;
  } else if (converter.current_limit_priority ==
             VSCCurrentLimitPriority::ActivePower) {
    out.equation[0] = median_ncp(
        p, command.p_ref_pu, -radius, radius, mu);
    const MedianGradient gradient =
        median_ncp_gradient(p, command.p_ref_pu, -radius, radius, mu);
    out.jacobian(0, 0) = gradient.da + gradient.db + gradient.dc;
    out.derivative_vm[0] = converter.i_ac_max_pu *
        (gradient.dc - gradient.db);
    out.derivative_vdc[0] = -gradient.da * dpref_dvdc;

    const double q_radius =
        std::sqrt(std::max(0.0, radius * radius - p * p));
    out.equation[1] = median_ncp(
        q, command.q_ref_pu, -q_radius, q_radius, mu);
    const MedianGradient q_gradient = median_ncp_gradient(
        q, command.q_ref_pu, -q_radius, q_radius, mu);
    out.jacobian(1, 1) = q_gradient.da + q_gradient.db + q_gradient.dc;
    if (q_radius > kTieTolerance) {
      const double dqcap_dp = -p / q_radius;
      const double dqcap_dvm =
          radius * converter.i_ac_max_pu / q_radius;
      out.jacobian(1, 0) = (q_gradient.dc - q_gradient.db) * dqcap_dp;
      out.derivative_vm[1] =
          (q_gradient.dc - q_gradient.db) * dqcap_dvm;
    }
    out.jacobian(5, 5) = 1.0;
  } else {
    out.equation[1] = median_ncp(
        q, command.q_ref_pu, -radius, radius, mu);
    const MedianGradient gradient =
        median_ncp_gradient(q, command.q_ref_pu, -radius, radius, mu);
    out.jacobian(1, 1) = gradient.da + gradient.db + gradient.dc;
    out.derivative_vm[1] = converter.i_ac_max_pu *
        (gradient.dc - gradient.db);

    const double p_radius =
        std::sqrt(std::max(0.0, radius * radius - q * q));
    out.equation[0] = median_ncp(
        p, command.p_ref_pu, -p_radius, p_radius, mu);
    const MedianGradient p_gradient = median_ncp_gradient(
        p, command.p_ref_pu, -p_radius, p_radius, mu);
    out.jacobian(0, 0) = p_gradient.da + p_gradient.db + p_gradient.dc;
    out.derivative_vdc[0] = -p_gradient.da * dpref_dvdc;
    if (p_radius > kTieTolerance) {
      const double dpcap_dq = -q / p_radius;
      const double dpcap_dvm =
          radius * converter.i_ac_max_pu / p_radius;
      out.jacobian(0, 1) = (p_gradient.dc - p_gradient.db) * dpcap_dq;
      out.derivative_vm[0] =
          (p_gradient.dc - p_gradient.db) * dpcap_dvm;
    }
    out.jacobian(5, 5) = 1.0;
  }

  const auto [dloss_dp, dloss_dvdc] = converter_loss_jacobian(
      converter, p, vdc_pu, base_mva, loss_model);
  out.equation[2] = pdc + p +
      converter_loss(converter, p, vdc_pu, base_mva, loss_model);
  out.jacobian(2, 0) = 1.0 + dloss_dp;
  out.jacobian(2, 2) = 1.0;
  out.derivative_vdc[2] = dloss_dvdc;

  if (converter.current_limit_priority == VSCCurrentLimitPriority::Magnitude) {
    out.equation[5] = mu > 0.0
        ? smooth_fb(multiplier, margin, mu)
        : fischer_burmeister(multiplier, margin);
    const auto [dphi_dlambda, dphi_dmargin] =
        mu > 0.0
            ? smooth_fb_derivatives(multiplier, margin, mu)
            : fischer_burmeister_jacobian(multiplier, margin);
    out.jacobian(5, 0) = -2.0 * p * dphi_dmargin;
    out.jacobian(5, 1) = -2.0 * q * dphi_dmargin;
    out.jacobian(5, 5) = dphi_dlambda;
    out.derivative_vm[5] =
        2.0 * vm * converter.i_ac_max_pu *
        converter.i_ac_max_pu * dphi_dmargin;
  } else {
    out.equation[5] = multiplier;
  }

  out.equation[3] = state[3];
  out.equation[4] = state[4];
  out.jacobian(3, 3) = 1.0;
  out.jacobian(4, 4) = 1.0;

  out.current_pu = std::hypot(p, q) / vm;
  out.current_margin_pu = converter.i_ac_max_pu - out.current_pu;
  out.complementarity_residual = out.equation.cwiseAbs().maxCoeff();
  out.current_limit_active =
      out.current_margin_pu <= 1e-8 || multiplier > 1e-8;
  out.droop_saturated = command.droop_saturated;
  return out;
}

}  // namespace hacdcpf::powerflow
