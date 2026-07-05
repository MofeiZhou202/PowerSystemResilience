#include "hacdcpf/dynamics/DynamicDaeDiagnostics.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/dynamics/DynamicStamp.hpp"

namespace hacdcpf::dynamics {
namespace {

using Complex = std::complex<double>;

constexpr double kPi = 3.141592653589793238462643383279502884;

struct SelectedState {
  int source_index{-1};
  std::string label;
  std::string device_type;
  int component_index{0};
};

Eigen::Vector3cd bus_voltage(const NetworkState& y, int bus_pos) {
  Eigen::Vector3cd v = Eigen::Vector3cd::Zero();
  const int base = 3 * bus_pos;
  if (bus_pos < 0 || base + 2 >= y.Vac_abc.size()) return v;
  v[0] = y.Vac_abc[base + 0];
  v[1] = y.Vac_abc[base + 1];
  v[2] = y.Vac_abc[base + 2];
  return v;
}

Complex positive_sequence_voltage(const Eigen::Vector3cd& v) {
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  return (v[0] + a * v[1] + a * a * v[2]) / 3.0;
}

void set_balanced_voltage(NetworkState& y, int bus_pos, Complex v) {
  const int base = 3 * bus_pos;
  if (bus_pos < 0 || base + 2 >= y.Vac_abc.size()) return;
  y.Vac_abc[base + 0] = v;
  y.Vac_abc[base + 1] = v * std::polar(1.0, -2.0 * kPi / 3.0);
  y.Vac_abc[base + 2] = v * std::polar(1.0, 2.0 * kPi / 3.0);
}

double inf_norm(const Eigen::VectorXd& v) {
  return v.size() == 0 ? 0.0 : v.lpNorm<Eigen::Infinity>();
}

double matrix_inf_norm(const Eigen::MatrixXd& m) {
  double value = 0.0;
  for (Eigen::Index i = 0; i < m.rows(); ++i) {
    value = std::max(value, m.row(i).cwiseAbs().sum());
  }
  return value;
}

std::string state_label(const DynamicDevice& device, const std::string& state) {
  return device.type() + "#" + std::to_string(device.componentIndex()) + ":" + state;
}

void add_state(std::vector<SelectedState>& states,
               const DynamicDevice& device,
               int source_index,
               const std::string& local_label) {
  states.push_back({source_index,
                    state_label(device, local_label),
                    device.type(),
                    device.componentIndex()});
}

std::vector<SelectedState> select_states(DynamicSystem& system,
                                         const DynamicDaeDiagnosticOptions& options) {
  std::vector<SelectedState> selected;
  int offset = 0;
  for (const auto& device : system.devices) {
    const int before = offset;
    device->assignStateIndices(offset);
    const int count = offset - before;
    if (count <= 0) continue;

    if (options.psd_simple_marconato_state_order &&
        device->type() == "SynchronousMachine" &&
        device->modelName() == "SimpleMarconatoMachine" &&
        count >= 8) {
      add_state(selected, *device, before + 2, "eq_p");
      add_state(selected, *device, before + 3, "ed_p");
      add_state(selected, *device, before + 4, "eq_pp");
      add_state(selected, *device, before + 5, "ed_pp");
      add_state(selected, *device, before + 0, "delta");
      add_state(selected, *device, before + 1, "omega");
      add_state(selected, *device, before + 7, "Vf");
      continue;
    }

    if (options.psd_simple_marconato_state_order &&
        device->type() == "Exciter" &&
        device->modelName() == "AVRTypeI" &&
        count == 3) {
      add_state(selected, *device, before + 0, "Vr1");
      add_state(selected, *device, before + 1, "Vr2");
      add_state(selected, *device, before + 2, "Vm");
      continue;
    }

    if (options.psd_simple_marconato_state_order) {
      continue;
    }

    for (int k = 0; k < count; ++k) {
      add_state(selected, *device, before + k, "s" + std::to_string(k));
    }
  }
  return selected;
}

Eigen::VectorXd pack_positive_sequence_state(const DynamicSystem& system,
                                             const std::vector<SelectedState>& selected) {
  const int n_bus = static_cast<int>(system.network.ac_bus_ids.size());
  Eigen::VectorXd z(2 * n_bus + static_cast<int>(selected.size()));
  for (int i = 0; i < n_bus; ++i) {
    const Complex v = positive_sequence_voltage(bus_voltage(system.y, i));
    z[i] = v.real();
    z[n_bus + i] = v.imag();
  }
  for (std::size_t k = 0; k < selected.size(); ++k) {
    const int idx = selected[k].source_index;
    z[2 * n_bus + static_cast<int>(k)] =
        (idx >= 0 && idx < system.x.x.size()) ? system.x.x[idx] : 0.0;
  }
  return z;
}

bool eval_positive_sequence_F(DynamicSystem& system,
                              const Eigen::VectorXd& x_save,
                              const NetworkState& y_save,
                              const std::vector<SelectedState>& selected,
                              const Eigen::VectorXd& z,
                              Eigen::VectorXd& F,
                              std::string& error) {
  const int n_bus = static_cast<int>(system.network.ac_bus_ids.size());
  const int n_diff = static_cast<int>(selected.size());
  if (z.size() != 2 * n_bus + n_diff) {
    error = "diagnostic state vector has unexpected size";
    return false;
  }

  system.x.x = x_save;
  system.y = y_save;

  for (int i = 0; i < n_bus; ++i) {
    set_balanced_voltage(system.y, i, Complex(z[i], z[n_bus + i]));
  }
  for (int k = 0; k < n_diff; ++k) {
    const int idx = selected[static_cast<std::size_t>(k)].source_index;
    if (idx >= 0 && idx < system.x.x.size()) {
      system.x.x[idx] = z[2 * n_bus + k];
    }
  }

  DynamicStamp stamp(system.network.acPhaseNodeCount(), system.network.dcBusCount());
  for (const auto& device : system.devices) {
    device->stamp(system.x.time_s, system.x, system.y, stamp);
  }
  Eigen::SparseMatrix<Complex> Yac;
  Eigen::VectorXcd Iac;
  Eigen::SparseMatrix<double> Gdc;
  Eigen::VectorXd Idc;
  system.network.assembleEffectiveMatrices(stamp,
                                           system.options.singular_regularization_pu,
                                           Yac,
                                           Iac,
                                           Gdc,
                                           Idc);

  Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(system.x.x.size());
  for (const auto& device : system.devices) {
    device->computeDerivatives(system.x.time_s, system.x, system.y, dxdt);
  }

  F = Eigen::VectorXd::Zero(z.size());
  if (n_bus > 0) {
    Eigen::VectorXcd Vac(system.network.acPhaseNodeCount());
    for (Eigen::Index i = 0; i < system.y.Vac_abc.size(); ++i) {
      Vac[i] = system.y.Vac_abc[i];
    }
    const Eigen::VectorXcd g = Iac - Yac * Vac;
    const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
    for (int bus = 0; bus < n_bus; ++bus) {
      const int base = 3 * bus;
      const Complex g_pos =
          (g[base + 0] + a * g[base + 1] + a * a * g[base + 2]) / 3.0;
      F[bus] = g_pos.real();
      F[n_bus + bus] = g_pos.imag();
    }
  }
  for (int k = 0; k < n_diff; ++k) {
    const int idx = selected[static_cast<std::size_t>(k)].source_index;
    F[2 * n_bus + k] =
        (idx >= 0 && idx < dxdt.size()) ? dxdt[idx] : 0.0;
  }
  if (!F.allFinite()) {
    error = "diagnostic residual produced a non-finite value";
    return false;
  }
  return true;
}

bool build_central_jacobian(DynamicSystem& system,
                            const Eigen::VectorXd& x_save,
                            const NetworkState& y_save,
                            const std::vector<SelectedState>& selected,
                            const Eigen::VectorXd& z0,
                            double fd_step,
                            Eigen::MatrixXd& jacobian,
                            std::string& error) {
  const int n = static_cast<int>(z0.size());
  jacobian = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd fp;
  Eigen::VectorXd fm;
  const double step = std::max(1e-8, std::abs(fd_step));
  for (int j = 0; j < n; ++j) {
    const double h = step * std::max(1.0, std::abs(z0[j]));
    Eigen::VectorXd zp = z0;
    Eigen::VectorXd zm = z0;
    zp[j] += h;
    zm[j] -= h;
    if (!eval_positive_sequence_F(system, x_save, y_save, selected, zp, fp, error) ||
        !eval_positive_sequence_F(system, x_save, y_save, selected, zm, fm, error)) {
      return false;
    }
    jacobian.col(j) = (fp - fm) / (2.0 * h);
  }
  return true;
}

bool reduce_jacobian(const Eigen::MatrixXd& J,
                     int n_algebraic,
                     int n_differential,
                     Eigen::MatrixXd& reduced,
                     std::string& error) {
  if (n_differential == 0) {
    reduced.resize(0, 0);
    return true;
  }
  const Eigen::MatrixXd f_x =
      J.bottomRightCorner(n_differential, n_differential);
  if (n_algebraic == 0) {
    reduced = f_x;
    return true;
  }
  const Eigen::MatrixXd g_y = J.topLeftCorner(n_algebraic, n_algebraic);
  const Eigen::MatrixXd g_x = J.topRightCorner(n_algebraic, n_differential);
  const Eigen::MatrixXd f_y = J.bottomLeftCorner(n_differential, n_algebraic);
  Eigen::SparseMatrix<double> gy_sparse = g_y.sparseView();
  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
  lu.compute(gy_sparse);
  if (lu.info() != Eigen::Success) {
    error = "diagnostic algebraic Jacobian block is singular";
    return false;
  }
  const Eigen::MatrixXd gy_solve = lu.solve(g_x);
  if (lu.info() != Eigen::Success || !gy_solve.allFinite()) {
    error = "diagnostic algebraic Jacobian solve failed";
    return false;
  }
  reduced = f_x - f_y * gy_solve;
  return reduced.allFinite();
}

}  // namespace

DynamicDaeDiagnostics dynamic_dae_diagnostics(
    DynamicSystem& system,
    const DynamicDaeDiagnosticOptions& options) {
  DynamicDaeDiagnostics result;
  result.coordinate_system =
      options.positive_sequence_projection ? "positive_sequence_psd_order"
                                           : "native";
  if (!options.positive_sequence_projection) {
    result.success = false;
    result.message = "native DAE diagnostics are not exposed by this API yet";
    return result;
  }

  const Eigen::VectorXd x_save = system.x.x;
  const NetworkState y_save = system.y;
  const auto selected = select_states(system, options);
  const int n_bus = static_cast<int>(system.network.ac_bus_ids.size());
  result.n_algebraic = 2 * n_bus;
  result.n_differential = static_cast<int>(selected.size());
  result.n_variables = result.n_algebraic + result.n_differential;

  result.states.reserve(static_cast<std::size_t>(result.n_variables));
  for (int i = 0; i < n_bus; ++i) {
    DynamicDaeStateInfo s;
    s.index = i;
    s.kind = "algebraic";
    s.label = "bus#" + std::to_string(system.network.ac_bus_ids[static_cast<std::size_t>(i)]) +
              ":Vr";
    result.states.push_back(std::move(s));
  }
  for (int i = 0; i < n_bus; ++i) {
    DynamicDaeStateInfo s;
    s.index = n_bus + i;
    s.kind = "algebraic";
    s.label = "bus#" + std::to_string(system.network.ac_bus_ids[static_cast<std::size_t>(i)]) +
              ":Vi";
    result.states.push_back(std::move(s));
  }
  for (int k = 0; k < result.n_differential; ++k) {
    const auto& selected_state = selected[static_cast<std::size_t>(k)];
    DynamicDaeStateInfo s;
    s.index = result.n_algebraic + k;
    s.source_state_index = selected_state.source_index;
    s.kind = "differential";
    s.label = selected_state.label;
    s.device_type = selected_state.device_type;
    s.component_index = selected_state.component_index;
    result.states.push_back(std::move(s));
  }

  result.state = pack_positive_sequence_state(system, selected);
  std::string error;
  if (!eval_positive_sequence_F(system,
                                x_save,
                                y_save,
                                selected,
                                result.state,
                                result.residual,
                                error)) {
    system.x.x = x_save;
    system.y = y_save;
    result.success = false;
    result.message = error;
    return result;
  }
  result.residual_inf_norm = inf_norm(result.residual);

  result.mass_diag = Eigen::VectorXd::Zero(result.n_variables);
  for (int k = 0; k < result.n_differential; ++k) {
    result.mass_diag[result.n_algebraic + k] = 1.0;
  }

  if (options.build_jacobian) {
    if (!build_central_jacobian(system,
                                x_save,
                                y_save,
                                selected,
                                result.state,
                                options.finite_difference_step,
                                result.jacobian,
                                error)) {
      system.x.x = x_save;
      system.y = y_save;
      result.success = false;
      result.message = error;
      return result;
    }
    result.jacobian_inf_norm = matrix_inf_norm(result.jacobian);
    if (!reduce_jacobian(result.jacobian,
                         result.n_algebraic,
                         result.n_differential,
                         result.reduced_jacobian,
                         error)) {
      system.x.x = x_save;
      system.y = y_save;
      result.success = false;
      result.message = error;
      return result;
    }
    result.reduced_jacobian_inf_norm = matrix_inf_norm(result.reduced_jacobian);
    if (result.n_differential > 0) {
      Eigen::EigenSolver<Eigen::MatrixXd> es(result.reduced_jacobian,
                                             /*computeEigenvectors=*/false);
      if (es.info() != Eigen::Success) {
        system.x.x = x_save;
        system.y = y_save;
        result.success = false;
        result.message = "diagnostic eigen-decomposition failed";
        return result;
      }
      result.eigenvalues = es.eigenvalues();
    }
  }

  system.x.x = x_save;
  system.y = y_save;
  result.success = true;
  result.message = "Dynamic DAE diagnostics completed";
  return result;
}

}  // namespace hacdcpf::dynamics
