#include "hacdcpf/power_flow/ac_kernel_impl.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "hacdcpf/util/thread_pool.hpp"

namespace hacdcpf::powerflow {

namespace {

constexpr size_t kParallelEntryThreshold = 1024;

int effective_ac_threads(int requested, size_t entries) {
  if (requested == 1 || entries < kParallelEntryThreshold) {
    return 1;
  }
  int hw = static_cast<int>(std::thread::hardware_concurrency());
  if (hw <= 0) {
    hw = 1;
  }
  int t = requested;
  if (t <= 0) {
    t = hw;
  }
  t = std::max(1, std::min(t, hw));
  t = std::min(t, static_cast<int>(entries));
  return t;
}

}  // namespace

void evaluate_ac_kernel_serial(const JacobianPattern& pattern,
                               const Eigen::VectorXd& vm,
                               const Eigen::VectorXd& va,
                               bool build_jacobian,
                               double* values,
                               Eigen::VectorXd& pcalc,
                               Eigen::VectorXd& qcalc) {
  for (const auto& entry : pattern.ac_entries) {
    const int i = entry.i;
    const int j = entry.j;
    const double vi = vm[i];
    const double vj = vm[j];
    const double theta = va[i] - va[j];
    const double g = entry.g;
    const double b = entry.b;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    pcalc[i] += vi * vj * (g * c + b * s);
    qcalc[i] += vi * vj * (g * s - b * c);

    if (!build_jacobian || i == j) {
      continue;
    }
    if (entry.p_va_nz >= 0) {
      values[entry.p_va_nz] += vi * vj * (g * s - b * c);
    }
    if (entry.q_va_nz >= 0) {
      values[entry.q_va_nz] += -vi * vj * (g * c + b * s);
    }
    if (entry.p_vm_nz >= 0) {
      values[entry.p_vm_nz] += vi * (g * c + b * s);
    }
    if (entry.q_vm_nz >= 0) {
      values[entry.q_vm_nz] += vi * (g * s - b * c);
    }
  }
}

void evaluate_ac_kernel_parallel(const JacobianPattern& pattern,
                                 const Eigen::VectorXd& vm,
                                 const Eigen::VectorXd& va,
                                 int ac_eval_threads,
                                 bool build_jacobian,
                                 double* values,
                                 Eigen::VectorXd& pcalc,
                                 Eigen::VectorXd& qcalc) {
  const size_t n_entries = pattern.ac_entries.size();
  const int nthreads = effective_ac_threads(ac_eval_threads, n_entries);
  if (n_entries < kParallelEntryThreshold) {
    evaluate_ac_kernel_serial(pattern, vm, va, build_jacobian, values, pcalc, qcalc);
    return;
  }

  struct EntryContribution {
    double p{0.0};
    double q{0.0};
    double p_va{0.0};
    double q_va{0.0};
    double p_vm{0.0};
    double q_vm{0.0};
  };

  // Each entry is independent. Parallelize that work, then reduce in the
  // authored entry order so floating-point addition order is independent of
  // thread count.
  std::vector<EntryContribution> contributions(n_entries);

  auto& pool = util::ThreadPool::global();
  pool.parallel_for(static_cast<size_t>(nthreads),
    [&](size_t t_begin, size_t t_end) {
      for (size_t t = t_begin; t < t_end; ++t) {
        const size_t begin = (n_entries * t) / static_cast<size_t>(nthreads);
        const size_t end = (n_entries * (t + 1)) / static_cast<size_t>(nthreads);
        for (size_t idx = begin; idx < end; ++idx) {
          const auto& entry = pattern.ac_entries[idx];
          EntryContribution& contribution = contributions[idx];
          const int i = entry.i;
          const int j = entry.j;
          const double vi = vm[i];
          const double vj = vm[j];
          const double theta = va[i] - va[j];
          const double g = entry.g;
          const double b = entry.b;
          const double c = std::cos(theta);
          const double s = std::sin(theta);

          contribution.p = vi * vj * (g * c + b * s);
          contribution.q = vi * vj * (g * s - b * c);

          if (!build_jacobian || i == j) {
            continue;
          }
          if (entry.p_va_nz >= 0) {
            contribution.p_va = vi * vj * (g * s - b * c);
          }
          if (entry.q_va_nz >= 0) {
            contribution.q_va = -vi * vj * (g * c + b * s);
          }
          if (entry.p_vm_nz >= 0) {
            contribution.p_vm = vi * (g * c + b * s);
          }
          if (entry.q_vm_nz >= 0) {
            contribution.q_vm = vi * (g * s - b * c);
          }
        }
      }
    }, nthreads);

  for (size_t idx = 0; idx < n_entries; ++idx) {
    const auto& entry = pattern.ac_entries[idx];
    const auto& contribution = contributions[idx];
    pcalc[entry.i] += contribution.p;
    qcalc[entry.i] += contribution.q;
    if (!build_jacobian || entry.i == entry.j) continue;
    if (entry.p_va_nz >= 0) values[entry.p_va_nz] += contribution.p_va;
    if (entry.q_va_nz >= 0) values[entry.q_va_nz] += contribution.q_va;
    if (entry.p_vm_nz >= 0) values[entry.p_vm_nz] += contribution.p_vm;
    if (entry.q_vm_nz >= 0) values[entry.q_vm_nz] += contribution.q_vm;
  }
}

}  // namespace hacdcpf::powerflow
