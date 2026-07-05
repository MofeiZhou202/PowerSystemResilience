#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

struct DynamicSystem;

// One eigenvalue (mode) of the reduced state matrix.
struct SmallSignalMode {
  double eigen_real{0.0};       // Re(lambda), 1/s
  double eigen_imag{0.0};       // Im(lambda), rad/s
  double frequency_hz{0.0};     // |Im(lambda)| / (2*pi)
  double damping_ratio{0.0};    // -Re(lambda) / |lambda|
  bool oscillatory{false};      // |Im(lambda)| above a small threshold
  int dominant_state_index{-1}; // differential state with the largest participation
  std::string dominant_state;   // label of that state
};

// Attribution of one differential state to its owning device.
struct SmallSignalStateInfo {
  int index{0};             // global differential-state index (0..n_differential-1)
  std::string device_name;
  std::string device_type;
  int component_index{0};
  int local_index{0};       // index within the owning device
  std::string label;        // "type#component:sLocal"
};

// Result of a small-signal (linearized) analysis about the current operating
// point of a dynamic system. Modes are sorted by damping ratio (most critical
// first). `participation(i, k)` is the normalized participation of state k in
// mode i (rows sum to 1).
struct SmallSignalResult {
  bool success{false};
  std::string message;
  int n_differential{0};
  int n_algebraic{0};
  bool stable{false};                // all Re(lambda) < stability_margin
  double stability_margin{1e-6};
  std::vector<SmallSignalMode> modes;
  std::vector<SmallSignalStateInfo> states;
  Eigen::MatrixXd reduced_jacobian;  // A = f_x - f_y * g_y^-1 * g_x  (n_diff x n_diff)
  Eigen::MatrixXd participation;     // n_diff x n_diff, |.|, normalized per mode (row)
};

// Compute the small-signal analysis about the system's current state (which
// should be the initialized/trimmed equilibrium). Reuses the same device stamp /
// derivative model as the mass-matrix DAE core: the algebraic network variables
// (bus voltages) are eliminated by a Schur complement.
SmallSignalResult small_signal_analysis(DynamicSystem& system);

}  // namespace hacdcpf::dynamics
