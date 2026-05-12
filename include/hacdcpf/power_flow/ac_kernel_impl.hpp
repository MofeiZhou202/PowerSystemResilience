#pragma once
// Internal header: declares the AC power-flow evaluation kernel so that
// ac_kernel.cpp (definition) and jacobian_builder.cpp (usage) share a
// single declaration without duplicating it.

#include <Eigen/Core>

#include "hacdcpf/power_flow/jacobian_builder.hpp"

namespace hacdcpf::powerflow {

/// Evaluate the AC power-flow contributions in a single thread.
/// Accumulates into pcalc/qcalc and, when build_jacobian is true, into values[].
void evaluate_ac_kernel_serial(const JacobianPattern& pattern,
                               const Eigen::VectorXd& vm,
                               const Eigen::VectorXd& va,
                               bool build_jacobian,
                               double* values,
                               Eigen::VectorXd& pcalc,
                               Eigen::VectorXd& qcalc);

/// Multi-threaded wrapper around evaluate_ac_kernel_serial.
/// Falls back to serial when nthreads <= 1 or the entry count is below threshold.
void evaluate_ac_kernel_parallel(const JacobianPattern& pattern,
                                 const Eigen::VectorXd& vm,
                                 const Eigen::VectorXd& va,
                                 int ac_eval_threads,
                                 bool build_jacobian,
                                 double* values,
                                 Eigen::VectorXd& pcalc,
                                 Eigen::VectorXd& qcalc);

}  // namespace hacdcpf::powerflow
