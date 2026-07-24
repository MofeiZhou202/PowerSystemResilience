#include "hacdcpf/optimal_power_flow/three_phase_hybrid_relaxation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <iterator>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>

#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/engine/solver/external/adapters.hpp"

namespace hacdcpf::opf::phase_hybrid {
namespace {

using Complex = std::complex<double>;
using Triplet = Eigen::Triplet<double>;

constexpr double kInfinity = 1e20;
constexpr double kPi = 3.141592653589793238462643383279502884;

struct LinearRow {
  std::vector<std::pair<int, double>> terms;
  double rhs{0.0};
};

struct LiftedPair {
  int first{-1};
  int second{-1};
  int real_var{-1};
  int imag_var{-1};
};

struct DCLiftedPair {
  int first{-1};
  int second{-1};
  int var{-1};
};

struct Layout {
  std::vector<int> w_diag;
  std::vector<LiftedPair> ac_pairs;
  std::unordered_map<std::uint64_t, int> ac_pair_position;
  std::vector<std::vector<int>> ac_psd_cliques;
  std::vector<int> x_diag;
  std::vector<DCLiftedPair> dc_pairs;
  std::unordered_map<std::uint64_t, int> dc_pair_position;
  std::vector<int> pg;
  std::vector<int> qg;
  std::vector<std::vector<int>> pac;
  std::vector<std::vector<int>> qac;
  std::vector<int> pdc;
  std::vector<int> cost_epigraph;
  int nvar{0};
};

struct RuntimeRecorder {
  ThreePhaseHybridRelaxationResult& result;
  std::chrono::steady_clock::time_point start;

  ~RuntimeRecorder() {
    result.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
  }
};

std::uint64_t pair_key(int first, int second) {
  const auto lo = static_cast<std::uint32_t>(std::min(first, second));
  const auto hi = static_cast<std::uint32_t>(std::max(first, second));
  return (static_cast<std::uint64_t>(lo) << 32U) |
         static_cast<std::uint64_t>(hi);
}

void add_term(LinearRow& row, int variable, double coefficient) {
  if (variable >= 0 && std::abs(coefficient) > 1e-14) {
    row.terms.emplace_back(variable, coefficient);
  }
}

void require_dimensions(const ThreePhaseHybridOPFCase& c) {
  const int n = static_cast<int>(c.y_ac.rows());
  const int ndc = static_cast<int>(c.g_dc.rows());
  if (n <= 0 || c.y_ac.cols() != n || c.i_ac_fixed.size() != n ||
      c.p_load_pu.size() != n || c.q_load_pu.size() != n ||
      c.v_min_pu.size() != n || c.v_max_pu.size() != n ||
      c.g_dc.cols() != ndc || c.p_dc_load_pu.size() != ndc ||
      c.v_dc_min_pu.size() != ndc || c.v_dc_max_pu.size() != ndc) {
    throw std::invalid_argument(
        "three-phase hybrid relaxation case dimension mismatch");
  }
  if (c.reference_nodes.size() !=
          static_cast<std::size_t>(c.reference_voltage.size()) ||
      c.dc_reference_terminals.size() !=
          static_cast<std::size_t>(c.dc_reference_voltage_pu.size())) {
    throw std::invalid_argument(
        "three-phase hybrid relaxation reference dimension mismatch");
  }
  for (const auto& converter : c.converters) {
    if (converter.phase_nodes.empty() || converter.s_max_pu < 0.0 ||
        converter.dc_terminal < 0 || converter.dc_terminal >= ndc) {
      throw std::invalid_argument(
          "invalid converter in three-phase hybrid relaxation case");
    }
    for (int node : converter.phase_nodes) {
      if (node < 0 || node >= n) {
        throw std::invalid_argument(
            "converter phase node outside three-phase relaxation case");
      }
    }
  }
}

int add_variable(std::vector<engine::VariableMeta>& variables,
                 double lower,
                 double upper,
                 std::string name) {
  engine::VariableMeta variable;
  variable.type = engine::VarType::Continuous;
  variable.lb = lower;
  variable.ub = upper;
  variable.name = std::move(name);
  variables.push_back(std::move(variable));
  return static_cast<int>(variables.size()) - 1;
}

void ensure_ac_pair(std::vector<std::pair<int, int>>& pairs,
                    std::unordered_map<std::uint64_t, bool>& present,
                    int first,
                    int second) {
  if (first == second) return;
  const int lo = std::min(first, second);
  const int hi = std::max(first, second);
  const auto key = pair_key(lo, hi);
  if (present.emplace(key, true).second) pairs.emplace_back(lo, hi);
}

void ensure_dc_pair(std::vector<std::pair<int, int>>& pairs,
                    std::unordered_map<std::uint64_t, bool>& present,
                    int first,
                    int second) {
  ensure_ac_pair(pairs, present, first, second);
}

const LiftedPair& ac_pair(const Layout& layout, int first, int second) {
  return layout.ac_pairs.at(static_cast<std::size_t>(
      layout.ac_pair_position.at(pair_key(first, second))));
}

const DCLiftedPair& dc_pair(const Layout& layout, int first, int second) {
  return layout.dc_pairs.at(static_cast<std::size_t>(
      layout.dc_pair_position.at(pair_key(first, second))));
}

engine::LPModel build_lp(const Layout& layout,
                         const std::vector<engine::VariableMeta>& variables,
                         const Eigen::VectorXd& objective,
                         const std::vector<LinearRow>& inequalities,
                         const std::vector<LinearRow>& equalities) {
  engine::LPModel lp;
  lp.sense = engine::Sense::Minimize;
  lp.c = objective;
  lp.vars = variables;
  std::vector<Triplet> inequality_triplets;
  std::vector<Triplet> equality_triplets;
  for (int row = 0; row < static_cast<int>(inequalities.size()); ++row) {
    for (const auto& [variable, coefficient] :
         inequalities[static_cast<std::size_t>(row)].terms) {
      inequality_triplets.emplace_back(row, variable, coefficient);
    }
  }
  for (int row = 0; row < static_cast<int>(equalities.size()); ++row) {
    for (const auto& [variable, coefficient] :
         equalities[static_cast<std::size_t>(row)].terms) {
      equality_triplets.emplace_back(row, variable, coefficient);
    }
  }
  lp.A.resize(static_cast<int>(inequalities.size()), layout.nvar);
  lp.A.setFromTriplets(inequality_triplets.begin(),
                       inequality_triplets.end());
  lp.b.resize(static_cast<int>(inequalities.size()));
  for (int row = 0; row < static_cast<int>(inequalities.size()); ++row) {
    lp.b[row] = inequalities[static_cast<std::size_t>(row)].rhs;
  }
  lp.Aeq.resize(static_cast<int>(equalities.size()), layout.nvar);
  lp.Aeq.setFromTriplets(equality_triplets.begin(), equality_triplets.end());
  lp.beq.resize(static_cast<int>(equalities.size()));
  for (int row = 0; row < static_cast<int>(equalities.size()); ++row) {
    lp.beq[row] = equalities[static_cast<std::size_t>(row)].rhs;
  }
  return lp;
}

struct ConeCuts {
  std::vector<LinearRow> rows;
  double max_violation{0.0};
  double max_ac_lift_violation{0.0};
  double max_ac_psd_violation{0.0};
  double max_dc_lift_violation{0.0};
  double max_converter_apparent_power_violation{0.0};
  double max_converter_current_violation{0.0};
};

void append_ac_psd_cuts(const Layout& layout,
                        const Eigen::VectorXd& x,
                        double tolerance,
                        ConeCuts& cuts) {
  for (const auto& clique : layout.ac_psd_cliques) {
    const int size = static_cast<int>(clique.size());
    Eigen::MatrixXcd lifted = Eigen::MatrixXcd::Zero(size, size);
    for (int i = 0; i < size; ++i) {
      lifted(i, i) = x[layout.w_diag[static_cast<std::size_t>(clique[i])]];
      for (int j = i + 1; j < size; ++j) {
        const auto& pair = ac_pair(layout, clique[i], clique[j]);
        const Complex value{x[pair.real_var], x[pair.imag_var]};
        lifted(i, j) = value;
        lifted(j, i) = std::conj(value);
      }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcd> eigen(lifted);
    if (eigen.info() != Eigen::Success) continue;
    const double minimum = eigen.eigenvalues()[0];
    const double violation = std::max(0.0, -minimum);
    cuts.max_ac_psd_violation =
        std::max(cuts.max_ac_psd_violation, violation);
    if (violation <= tolerance) continue;

    const Eigen::VectorXcd direction = eigen.eigenvectors().col(0);
    LinearRow row;
    for (int i = 0; i < size; ++i) {
      add_term(row, layout.w_diag[static_cast<std::size_t>(clique[i])],
               -std::norm(direction[i]));
      for (int j = i + 1; j < size; ++j) {
        const Complex coefficient = std::conj(direction[i]) * direction[j];
        const auto& pair = ac_pair(layout, clique[i], clique[j]);
        add_term(row, pair.real_var, -2.0 * std::real(coefficient));
        add_term(row, pair.imag_var, 2.0 * std::imag(coefficient));
      }
    }
    cuts.rows.push_back(std::move(row));
  }
}

struct DualBound {
  bool available{false};
  double value{-std::numeric_limits<double>::infinity()};
};

DualBound evaluate_lp_dual_bound(const engine::LPModel& lp,
                                 const engine::SolveResult& solved) {
  const int inequalities = static_cast<int>(lp.A.rows());
  const int equalities = static_cast<int>(lp.Aeq.rows());
  if (solved.constraint_duals.size() != inequalities + equalities) return {};

  // HiGHS row duals for upper-bounded rows A*x <= b are nonpositive.
  // Clipping them to that cone and minimizing the resulting Lagrangian over
  // each variable box gives a valid dual-function lower bound even when the
  // returned dual vector has a small numerical residual.
  Eigen::VectorXd inequality_duals =
      solved.constraint_duals.head(inequalities).cwiseMin(0.0);
  const Eigen::VectorXd equality_duals =
      solved.constraint_duals.tail(equalities);
  Eigen::VectorXd reduced = lp.c;
  if (inequalities > 0) reduced.noalias() -= lp.A.transpose() * inequality_duals;
  if (equalities > 0) reduced.noalias() -= lp.Aeq.transpose() * equality_duals;

  double value = 0.0;
  if (inequalities > 0) value += lp.b.dot(inequality_duals);
  if (equalities > 0) value += lp.beq.dot(equality_duals);
  for (int variable = 0; variable < reduced.size(); ++variable) {
    const double coefficient = reduced[variable];
    const auto& meta = lp.vars[static_cast<std::size_t>(variable)];
    const double bound = coefficient >= 0.0 ? meta.lb : meta.ub;
    if (!std::isfinite(coefficient) || !std::isfinite(bound)) return {};
    value += coefficient * bound;
  }
  if (!std::isfinite(value)) return {};
  return {true, value};
}

bool ac_admittance_is_passive(const graph::SparseComplexMatrix& admittance,
                              double tolerance = 1e-8) {
  const Eigen::MatrixXcd dense(admittance);
  const Eigen::MatrixXcd hermitian = 0.5 * (dense + dense.adjoint());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXcd> eigen(
      hermitian, Eigen::EigenvaluesOnly);
  if (eigen.info() != Eigen::Success) return false;
  const double scale = std::max(1.0, hermitian.norm());
  return eigen.eigenvalues().minCoeff() >= -tolerance * scale;
}

bool dc_conductance_is_passive(const Eigen::SparseMatrix<double>& conductance,
                               double tolerance = 1e-8) {
  if (conductance.rows() == 0) return true;
  const Eigen::MatrixXd dense(conductance);
  const Eigen::MatrixXd symmetric = 0.5 * (dense + dense.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(
      symmetric, Eigen::EigenvaluesOnly);
  if (eigen.info() != Eigen::Success) return false;
  const double scale = std::max(1.0, symmetric.norm());
  return eigen.eigenvalues().minCoeff() >= -tolerance * scale;
}

ConeCuts violated_cone_cuts(const ThreePhaseHybridOPFCase& problem,
                            const Layout& layout,
                            const Eigen::VectorXd& x,
                            double tolerance) {
  ConeCuts cuts;
  const auto append_if_violated = [&](const std::vector<double>& z,
                                      double t,
                                      double& category_maximum,
                                      auto make_row) {
    double norm = 0.0;
    for (double value : z) norm += value * value;
    norm = std::sqrt(norm);
    const double violation = norm - t;
    cuts.max_violation = std::max(cuts.max_violation, violation);
    category_maximum = std::max(category_maximum, violation);
    if (violation <= tolerance || norm <= 1e-14) return;
    std::vector<double> direction(z.size());
    for (std::size_t i = 0; i < z.size(); ++i) {
      direction[i] = z[i] / norm;
    }
    cuts.rows.push_back(make_row(direction));
  };

  for (const auto& pair : layout.ac_pairs) {
    const double wi = x[layout.w_diag[static_cast<std::size_t>(pair.first)]];
    const double wj = x[layout.w_diag[static_cast<std::size_t>(pair.second)]];
    const double wr = x[pair.real_var];
    const double wq = x[pair.imag_var];
    append_if_violated({2.0 * wr, 2.0 * wq, wi - wj}, wi + wj,
                       cuts.max_ac_lift_violation,
                       [&](const std::vector<double>& u) {
      LinearRow row;
      add_term(row, pair.real_var, 2.0 * u[0]);
      add_term(row, pair.imag_var, 2.0 * u[1]);
      add_term(row, layout.w_diag[static_cast<std::size_t>(pair.first)],
               u[2] - 1.0);
      add_term(row, layout.w_diag[static_cast<std::size_t>(pair.second)],
               -u[2] - 1.0);
      return row;
    });
  }

  append_ac_psd_cuts(layout, x, tolerance, cuts);

  for (const auto& pair : layout.dc_pairs) {
    const double xi = x[layout.x_diag[static_cast<std::size_t>(pair.first)]];
    const double xj = x[layout.x_diag[static_cast<std::size_t>(pair.second)]];
    const double xij = x[pair.var];
    append_if_violated({2.0 * xij, xi - xj}, xi + xj,
                       cuts.max_dc_lift_violation,
                       [&](const std::vector<double>& u) {
      LinearRow row;
      add_term(row, pair.var, 2.0 * u[0]);
      add_term(row, layout.x_diag[static_cast<std::size_t>(pair.first)],
               u[1] - 1.0);
      add_term(row, layout.x_diag[static_cast<std::size_t>(pair.second)],
               -u[1] - 1.0);
      return row;
    });
  }

  for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
    const auto& converter = problem.converters[static_cast<std::size_t>(ci)];
    std::vector<double> apparent;
    for (int local = 0; local < static_cast<int>(layout.pac[ci].size()); ++local) {
      apparent.push_back(x[layout.pac[ci][static_cast<std::size_t>(local)]]);
      apparent.push_back(x[layout.qac[ci][static_cast<std::size_t>(local)]]);
    }
    append_if_violated(apparent, converter.s_max_pu,
                       cuts.max_converter_apparent_power_violation,
                       [&](const std::vector<double>& u) {
      LinearRow row;
      row.rhs = converter.s_max_pu;
      for (int local = 0; local < static_cast<int>(layout.pac[ci].size());
           ++local) {
        add_term(row, layout.pac[ci][static_cast<std::size_t>(local)],
                 u[static_cast<std::size_t>(2 * local)]);
        add_term(row, layout.qac[ci][static_cast<std::size_t>(local)],
                 u[static_cast<std::size_t>(2 * local + 1)]);
      }
      return row;
    });

    const double phase_current_max = converter.phase_current_max_pu > 0.0
        ? converter.phase_current_max_pu
        : converter.s_max_pu /
              std::sqrt(static_cast<double>(converter.phase_nodes.size()));
    const double current_squared = phase_current_max * phase_current_max;
    for (int local = 0; local < static_cast<int>(converter.phase_nodes.size());
         ++local) {
      const int node = converter.phase_nodes[static_cast<std::size_t>(local)];
      const int p = layout.pac[ci][static_cast<std::size_t>(local)];
      const int q = layout.qac[ci][static_cast<std::size_t>(local)];
      const int w = layout.w_diag[static_cast<std::size_t>(node)];
      append_if_violated(
          {2.0 * x[p], 2.0 * x[q], x[w] - current_squared},
          x[w] + current_squared,
          cuts.max_converter_current_violation,
          [&](const std::vector<double>& u) {
        LinearRow row;
        add_term(row, p, 2.0 * u[0]);
        add_term(row, q, 2.0 * u[1]);
        add_term(row, w, u[2] - 1.0);
        row.rhs = current_squared * (u[2] + 1.0);
        return row;
      });
    }
  }
  return cuts;
}

}  // namespace

ThreePhaseHybridRelaxationResult solve_three_phase_hybrid_opf_relaxation(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridRelaxationOptions& options) {
  ThreePhaseHybridRelaxationResult result;
  const auto start = std::chrono::steady_clock::now();
  const RuntimeRecorder runtime_recorder{result, start};
  try {
    require_dimensions(problem);
    if (options.max_outer_approximation_rounds < 0 ||
        options.cost_tangent_points < 2 || options.cone_tolerance <= 0.0 ||
        options.solver_tolerance <= 0.0) {
      throw std::invalid_argument("invalid three-phase relaxation options");
    }
    if (problem.i_ac_fixed.cwiseAbs().maxCoeff() > 1e-12) {
      result.status =
          "fixed-current AC injections require an augmented voltage lift";
      result.model_limitations.push_back(result.status);
      return result;
    }
    for (const auto& generator : problem.generators) {
      if (generator.cost_c2 < -1e-12) {
        result.status = "negative quadratic generation cost is not convex";
        return result;
      }
    }

    const int n = static_cast<int>(problem.y_ac.rows());
    const int ndc = static_cast<int>(problem.g_dc.rows());
    Layout layout;
    std::vector<engine::VariableMeta> variables;
    std::vector<std::pair<int, int>> ac_pair_nodes;
    std::unordered_map<std::uint64_t, bool> ac_pair_present;
    for (int col = 0; col < problem.y_ac.outerSize(); ++col) {
      for (graph::SparseComplexMatrix::InnerIterator it(problem.y_ac, col); it;
           ++it) {
        ensure_ac_pair(ac_pair_nodes, ac_pair_present, it.row(), it.col());
      }
    }
    for (const auto& nodes : problem.three_phase_bus_nodes) {
      for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(nodes.size()); ++j) {
          ensure_ac_pair(ac_pair_nodes, ac_pair_present, nodes[i], nodes[j]);
        }
      }
    }
    if (options.max_outer_approximation_rounds > 0) {
      std::set<std::vector<int>> candidate_cliques;
      for (int col = 0; col < problem.y_ac.outerSize(); ++col) {
        std::vector<int> clique{col};
        for (graph::SparseComplexMatrix::InnerIterator it(problem.y_ac, col);
             it; ++it) {
          if (std::abs(it.value()) > 1e-14) clique.push_back(it.row());
        }
        std::sort(clique.begin(), clique.end());
        clique.erase(std::unique(clique.begin(), clique.end()), clique.end());
        if (clique.size() > 2) candidate_cliques.insert(std::move(clique));
      }
      for (auto nodes : problem.three_phase_bus_nodes) {
        std::sort(nodes.begin(), nodes.end());
        nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
        if (nodes.size() > 2) candidate_cliques.insert(std::move(nodes));
      }
      for (const auto& clique : candidate_cliques) {
        bool contained = false;
        for (const auto& other : candidate_cliques) {
          if (clique.size() >= other.size()) continue;
          if (std::includes(other.begin(), other.end(), clique.begin(),
                            clique.end())) {
            contained = true;
            break;
          }
        }
        if (!contained) layout.ac_psd_cliques.push_back(clique);
      }
      for (const auto& clique : layout.ac_psd_cliques) {
        for (int i = 0; i < static_cast<int>(clique.size()); ++i) {
          for (int j = i + 1; j < static_cast<int>(clique.size()); ++j) {
            ensure_ac_pair(ac_pair_nodes, ac_pair_present, clique[i], clique[j]);
          }
        }
      }
    }
    for (int i = 0; i < static_cast<int>(problem.reference_nodes.size()); ++i) {
      for (int j = i + 1;
           j < static_cast<int>(problem.reference_nodes.size()); ++j) {
        ensure_ac_pair(ac_pair_nodes, ac_pair_present,
                       problem.reference_nodes[static_cast<std::size_t>(i)],
                       problem.reference_nodes[static_cast<std::size_t>(j)]);
      }
    }
    std::sort(ac_pair_nodes.begin(), ac_pair_nodes.end());

    std::unordered_map<int, Complex> reference_voltage;
    for (int i = 0; i < static_cast<int>(problem.reference_nodes.size()); ++i) {
      reference_voltage[problem.reference_nodes[static_cast<std::size_t>(i)]] =
          problem.reference_voltage[i];
    }
    layout.w_diag.resize(static_cast<std::size_t>(n));
    for (int node = 0; node < n; ++node) {
      double lower = problem.v_min_pu[node] * problem.v_min_pu[node];
      double upper = problem.v_max_pu[node] * problem.v_max_pu[node];
      if (reference_voltage.count(node)) {
        lower = upper = std::norm(reference_voltage.at(node));
      }
      layout.w_diag[static_cast<std::size_t>(node)] = add_variable(
          variables, lower, upper, "Wdiag_" + std::to_string(node));
    }
    for (const auto& [first, second] : ac_pair_nodes) {
      const double bound = problem.v_max_pu[first] * problem.v_max_pu[second];
      double real_lower = -bound;
      double real_upper = bound;
      double imag_lower = -bound;
      double imag_upper = bound;
      if (reference_voltage.count(first) && reference_voltage.count(second)) {
        const Complex fixed =
            reference_voltage.at(first) * std::conj(reference_voltage.at(second));
        real_lower = real_upper = std::real(fixed);
        imag_lower = imag_upper = std::imag(fixed);
      }
      LiftedPair pair;
      pair.first = first;
      pair.second = second;
      pair.real_var = add_variable(
          variables, real_lower, real_upper,
          "Wre_" + std::to_string(first) + "_" + std::to_string(second));
      pair.imag_var = add_variable(
          variables, imag_lower, imag_upper,
          "Wim_" + std::to_string(first) + "_" + std::to_string(second));
      layout.ac_pair_position[pair_key(first, second)] =
          static_cast<int>(layout.ac_pairs.size());
      layout.ac_pairs.push_back(pair);
    }

    std::vector<std::pair<int, int>> dc_pair_nodes;
    std::unordered_map<std::uint64_t, bool> dc_pair_present;
    for (int col = 0; col < problem.g_dc.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(problem.g_dc, col); it;
           ++it) {
        ensure_dc_pair(dc_pair_nodes, dc_pair_present, it.row(), it.col());
      }
    }
    std::sort(dc_pair_nodes.begin(), dc_pair_nodes.end());
    std::unordered_map<int, double> dc_reference_voltage;
    for (int i = 0;
         i < static_cast<int>(problem.dc_reference_terminals.size()); ++i) {
      dc_reference_voltage[
          problem.dc_reference_terminals[static_cast<std::size_t>(i)]] =
          problem.dc_reference_voltage_pu[i];
    }
    layout.x_diag.resize(static_cast<std::size_t>(ndc));
    for (int node = 0; node < ndc; ++node) {
      double lower = problem.v_dc_min_pu[node] * problem.v_dc_min_pu[node];
      double upper = problem.v_dc_max_pu[node] * problem.v_dc_max_pu[node];
      if (dc_reference_voltage.count(node)) {
        lower = upper = dc_reference_voltage.at(node) *
                          dc_reference_voltage.at(node);
      }
      layout.x_diag[static_cast<std::size_t>(node)] = add_variable(
          variables, lower, upper, "Xdiag_" + std::to_string(node));
    }
    for (const auto& [first, second] : dc_pair_nodes) {
      DCLiftedPair pair;
      pair.first = first;
      pair.second = second;
      const double lower = problem.v_dc_min_pu[first] *
                           problem.v_dc_min_pu[second];
      const double upper = problem.v_dc_max_pu[first] *
                           problem.v_dc_max_pu[second];
      pair.var = add_variable(
          variables, lower, upper,
          "X_" + std::to_string(first) + "_" + std::to_string(second));
      layout.dc_pair_position[pair_key(first, second)] =
          static_cast<int>(layout.dc_pairs.size());
      layout.dc_pairs.push_back(pair);
    }

    layout.pg.resize(problem.generators.size());
    layout.qg.resize(problem.generators.size());
    layout.cost_epigraph.assign(problem.generators.size(), -1);
    for (int gi = 0; gi < static_cast<int>(problem.generators.size()); ++gi) {
      const auto& generator = problem.generators[static_cast<std::size_t>(gi)];
      layout.pg[static_cast<std::size_t>(gi)] = add_variable(
          variables, generator.p_min_pu, generator.p_max_pu,
          "Pg_" + std::to_string(gi));
      layout.qg[static_cast<std::size_t>(gi)] = add_variable(
          variables, generator.q_min_pu, generator.q_max_pu,
          "Qg_" + std::to_string(gi));
      if (generator.cost_c2 > 1e-14) {
        layout.cost_epigraph[static_cast<std::size_t>(gi)] = add_variable(
            variables, 0.0, kInfinity, "Cquad_" + std::to_string(gi));
      }
    }

    layout.pac.resize(problem.converters.size());
    layout.qac.resize(problem.converters.size());
    layout.pdc.resize(problem.converters.size());
    for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
      const auto& converter = problem.converters[static_cast<std::size_t>(ci)];
      layout.pac[static_cast<std::size_t>(ci)].resize(
          converter.phase_nodes.size());
      layout.qac[static_cast<std::size_t>(ci)].resize(
          converter.phase_nodes.size());
      for (int local = 0; local < static_cast<int>(converter.phase_nodes.size());
           ++local) {
        layout.pac[static_cast<std::size_t>(ci)]
                   [static_cast<std::size_t>(local)] = add_variable(
            variables, -converter.s_max_pu, converter.s_max_pu,
            "Pac_" + std::to_string(ci) + "_" + std::to_string(local));
        const double q_lower = converter.fixed_unity_power_factor
                                   ? 0.0 : -converter.s_max_pu;
        const double q_upper = converter.fixed_unity_power_factor
                                   ? 0.0 : converter.s_max_pu;
        layout.qac[static_cast<std::size_t>(ci)]
                   [static_cast<std::size_t>(local)] = add_variable(
            variables, q_lower, q_upper,
            "Qac_" + std::to_string(ci) + "_" + std::to_string(local));
      }
      layout.pdc[static_cast<std::size_t>(ci)] = add_variable(
          variables, -converter.s_max_pu, converter.s_max_pu,
          "Pdc_" + std::to_string(ci));
      if (converter.control_mode != PhaseVSCControlMode::EqualPhasePower) {
        result.model_limitations.push_back(
            "Nonconvex sequence-aware VSC control equalities are omitted from "
            "the lower-bound relaxation");
      }
    }
    layout.nvar = static_cast<int>(variables.size());

    std::vector<LinearRow> equalities(
        static_cast<std::size_t>(2 * n + ndc));
    for (int node = 0; node < n; ++node) {
      equalities[static_cast<std::size_t>(node)].rhs = -problem.p_load_pu[node];
      equalities[static_cast<std::size_t>(n + node)].rhs =
          -problem.q_load_pu[node];
    }
    for (int col = 0; col < problem.y_ac.outerSize(); ++col) {
      for (graph::SparseComplexMatrix::InnerIterator it(problem.y_ac, col); it;
           ++it) {
        const int i = it.row();
        const int j = it.col();
        const double conductance = std::real(it.value());
        const double susceptance = std::imag(it.value());
        if (i == j) {
          add_term(equalities[static_cast<std::size_t>(i)],
                   layout.w_diag[static_cast<std::size_t>(i)], conductance);
          add_term(equalities[static_cast<std::size_t>(n + i)],
                   layout.w_diag[static_cast<std::size_t>(i)], -susceptance);
        } else {
          const auto& pair = ac_pair(layout, i, j);
          const double orientation = i < j ? 1.0 : -1.0;
          add_term(equalities[static_cast<std::size_t>(i)], pair.real_var,
                   conductance);
          add_term(equalities[static_cast<std::size_t>(i)], pair.imag_var,
                   orientation * susceptance);
          add_term(equalities[static_cast<std::size_t>(n + i)], pair.real_var,
                   -susceptance);
          add_term(equalities[static_cast<std::size_t>(n + i)], pair.imag_var,
                   orientation * conductance);
        }
      }
    }
    for (int gi = 0; gi < static_cast<int>(problem.generators.size()); ++gi) {
      const int node = problem.generators[static_cast<std::size_t>(gi)].phase_node;
      add_term(equalities[static_cast<std::size_t>(node)],
               layout.pg[static_cast<std::size_t>(gi)], -1.0);
      add_term(equalities[static_cast<std::size_t>(n + node)],
               layout.qg[static_cast<std::size_t>(gi)], -1.0);
    }
    for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
      const auto& converter = problem.converters[static_cast<std::size_t>(ci)];
      for (int local = 0; local < static_cast<int>(converter.phase_nodes.size());
           ++local) {
        const int node = converter.phase_nodes[static_cast<std::size_t>(local)];
        add_term(equalities[static_cast<std::size_t>(node)],
                 layout.pac[static_cast<std::size_t>(ci)]
                            [static_cast<std::size_t>(local)], -1.0);
        add_term(equalities[static_cast<std::size_t>(n + node)],
                 layout.qac[static_cast<std::size_t>(ci)]
                            [static_cast<std::size_t>(local)], -1.0);
      }
    }

    for (int node = 0; node < ndc; ++node) {
      auto& row = equalities[static_cast<std::size_t>(2 * n + node)];
      row.rhs = -problem.p_dc_load_pu[node];
      for (int col = 0; col < problem.g_dc.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(problem.g_dc, col);
             it; ++it) {
          if (it.row() != node) continue;
          if (it.row() == it.col()) {
            add_term(row, layout.x_diag[static_cast<std::size_t>(node)],
                     it.value());
          } else {
            add_term(row, dc_pair(layout, it.row(), it.col()).var, it.value());
          }
        }
      }
      for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
        if (problem.converters[static_cast<std::size_t>(ci)].dc_terminal == node) {
          add_term(row, layout.pdc[static_cast<std::size_t>(ci)], -1.0);
        }
      }
    }

    for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
      const auto& converter = problem.converters[static_cast<std::size_t>(ci)];
      LinearRow coupling;
      add_term(coupling, layout.pdc[static_cast<std::size_t>(ci)], 1.0);
      for (int variable : layout.pac[static_cast<std::size_t>(ci)]) {
        add_term(coupling, variable,
                 std::clamp(converter.efficiency, 0.01, 1.0));
      }
      equalities.push_back(std::move(coupling));
      if (converter.control_mode == PhaseVSCControlMode::EqualPhasePower &&
          !layout.pac[static_cast<std::size_t>(ci)].empty()) {
        const int first_p = layout.pac[static_cast<std::size_t>(ci)].front();
        const int first_q = layout.qac[static_cast<std::size_t>(ci)].front();
        for (int local = 1;
             local < static_cast<int>(converter.phase_nodes.size()); ++local) {
          LinearRow active;
          add_term(active,
                   layout.pac[static_cast<std::size_t>(ci)]
                              [static_cast<std::size_t>(local)], 1.0);
          add_term(active, first_p, -1.0);
          equalities.push_back(std::move(active));
          LinearRow reactive;
          add_term(reactive,
                   layout.qac[static_cast<std::size_t>(ci)]
                              [static_cast<std::size_t>(local)], 1.0);
          add_term(reactive, first_q, -1.0);
          equalities.push_back(std::move(reactive));
        }
      }
    }

    std::vector<LinearRow> inequalities;
    if (ac_admittance_is_passive(problem.y_ac)) {
      LinearRow nonnegative_ac_loss;
      nonnegative_ac_loss.rhs = -problem.p_load_pu.sum();
      for (int variable : layout.pg) add_term(nonnegative_ac_loss, variable, -1.0);
      for (const auto& converter : layout.pac) {
        for (int variable : converter) {
          add_term(nonnegative_ac_loss, variable, -1.0);
        }
      }
      inequalities.push_back(std::move(nonnegative_ac_loss));
      result.ac_passivity_cut_applied = true;
    } else {
      result.model_limitations.push_back(
          "The AC admittance failed the passivity check; the aggregate "
          "nonnegative-loss cut was not applied");
    }
    if (dc_conductance_is_passive(problem.g_dc)) {
      LinearRow nonnegative_dc_loss;
      nonnegative_dc_loss.rhs = -problem.p_dc_load_pu.sum();
      for (int variable : layout.pdc) {
        add_term(nonnegative_dc_loss, variable, -1.0);
      }
      inequalities.push_back(std::move(nonnegative_dc_loss));
      result.dc_passivity_cut_applied = true;
    } else {
      result.model_limitations.push_back(
          "The DC conductance matrix failed the passivity check; the aggregate "
          "nonnegative-loss cut was not applied");
    }
    const Complex phase_rotation = std::polar(1.0, 2.0 * kPi / 3.0);
    const Complex phase_rotation_2 = phase_rotation * phase_rotation;
    const Complex positive[3] = {
        Complex{1.0 / 3.0, 0.0}, phase_rotation / 3.0,
        phase_rotation_2 / 3.0};
    const Complex negative[3] = {
        Complex{1.0 / 3.0, 0.0}, phase_rotation_2 / 3.0,
        phase_rotation / 3.0};
    for (const auto& nodes : problem.three_phase_bus_nodes) {
      if (nodes.size() != 3) continue;
      LinearRow row;
      for (int i = 0; i < 3; ++i) {
        const Complex diagonal = negative[i] * std::conj(negative[i]) -
            problem.vuf_max * problem.vuf_max *
                positive[i] * std::conj(positive[i]);
        add_term(row, layout.w_diag[static_cast<std::size_t>(nodes[i])],
                 std::real(diagonal));
        for (int j = i + 1; j < 3; ++j) {
          const Complex coefficient =
              negative[i] * std::conj(negative[j]) -
              problem.vuf_max * problem.vuf_max *
                  positive[i] * std::conj(positive[j]);
          const auto& pair = ac_pair(layout, nodes[i], nodes[j]);
          const double orientation = nodes[i] < nodes[j] ? 1.0 : -1.0;
          add_term(row, pair.real_var, 2.0 * std::real(coefficient));
          add_term(row, pair.imag_var,
                   -2.0 * orientation * std::imag(coefficient));
        }
      }
      inequalities.push_back(std::move(row));
    }

    Eigen::VectorXd objective = Eigen::VectorXd::Zero(layout.nvar);
    for (int gi = 0; gi < static_cast<int>(problem.generators.size()); ++gi) {
      const auto& generator = problem.generators[static_cast<std::size_t>(gi)];
      const int p = layout.pg[static_cast<std::size_t>(gi)];
      objective[p] += generator.cost_c1 * problem.base_mva;
      const int epigraph = layout.cost_epigraph[static_cast<std::size_t>(gi)];
      if (epigraph < 0) continue;
      objective[epigraph] = 1.0;
      const double quadratic = generator.cost_c2 * problem.base_mva *
                               problem.base_mva;
      for (int point = 0; point < options.cost_tangent_points; ++point) {
        const double fraction = static_cast<double>(point) /
            static_cast<double>(options.cost_tangent_points - 1);
        const double p0 = generator.p_min_pu +
            fraction * (generator.p_max_pu - generator.p_min_pu);
        LinearRow tangent;
        add_term(tangent, p, 2.0 * quadratic * p0);
        add_term(tangent, epigraph, -1.0);
        tangent.rhs = quadratic * p0 * p0;
        inequalities.push_back(std::move(tangent));
      }
    }
    result.model_limitations.push_back(
        "Positive reactive-power and converter regularization terms are "
        "omitted from the lower-bound objective");
    result.model_limitations.push_back(
        "Convex quadratic generation costs use supporting-tangent "
        "underestimators");

    engine::HighsAdapter highs;
    if (!highs.available()) {
      result.status = "HiGHS LP backend is unavailable";
      return result;
    }
    std::vector<LinearRow> cone_cuts;
    engine::SolveResult solved;
    double best_dual_bound = -std::numeric_limits<double>::infinity();
    for (int round = 0;
         round <= options.max_outer_approximation_rounds; ++round) {
      std::vector<LinearRow> all_inequalities = inequalities;
      all_inequalities.insert(all_inequalities.end(), cone_cuts.begin(),
                              cone_cuts.end());
      const auto lp = build_lp(layout, variables, objective, all_inequalities,
                               equalities);
      solved = highs.solve_lp(lp);
      result.rounds = round + 1;
      result.variables = layout.nvar;
      result.equalities = static_cast<int>(equalities.size());
      result.inequalities = static_cast<int>(all_inequalities.size());
      if (!solved.stats.success || solved.x.size() != layout.nvar) {
        result.status = solved.stats.status;
        result.solver = solved.stats.solver_name;
        return result;
      }
      const auto dual_bound = evaluate_lp_dual_bound(lp, solved);
      if (dual_bound.available) {
        best_dual_bound = std::max(best_dual_bound, dual_bound.value);
        result.round_lower_bounds.push_back(best_dual_bound);
      } else {
        result.round_lower_bounds.push_back(
            -std::numeric_limits<double>::infinity());
      }
      auto violated = violated_cone_cuts(
          problem, layout, solved.x, options.cone_tolerance);
      result.max_soc_violation = violated.max_violation;
      result.max_ac_lift_violation = violated.max_ac_lift_violation;
      result.max_ac_psd_violation = violated.max_ac_psd_violation;
      result.max_dc_lift_violation = violated.max_dc_lift_violation;
      result.max_converter_apparent_power_violation =
          violated.max_converter_apparent_power_violation;
      result.max_converter_current_violation =
          violated.max_converter_current_violation;
      if (violated.rows.empty()) {
        result.soc_outer_approximation_converged = true;
        break;
      }
      if (round == options.max_outer_approximation_rounds) break;
      result.cuts_added += static_cast<int>(violated.rows.size());
      cone_cuts.insert(cone_cuts.end(),
                       std::make_move_iterator(violated.rows.begin()),
                       std::make_move_iterator(violated.rows.end()));
    }

    result.solved = true;
    result.outer_relaxation_valid = true;
    result.status = solved.stats.status;
    result.solver = solved.stats.solver_name;
    result.lp_primal_objective = solved.stats.objective;
    result.dual_certificate_available = std::isfinite(best_dual_bound);
    result.objective_lower_bound = result.dual_certificate_available
        ? best_dual_bound : -std::numeric_limits<double>::infinity();
    result.lp_primal_dual_gap = result.dual_certificate_available
        ? std::max(0.0, result.lp_primal_objective - best_dual_bound)
        : std::numeric_limits<double>::infinity();
    result.primal_residual = solved.stats.primal_feas;
    result.dual_residual = solved.stats.dual_feas;
    result.primal = solved.x;
    result.generator_active_power_pu.resize(problem.generators.size());
    result.generator_reactive_power_pu.resize(problem.generators.size());
    for (int gi = 0; gi < static_cast<int>(problem.generators.size()); ++gi) {
      result.generator_active_power_pu[static_cast<std::size_t>(gi)] =
          solved.x[layout.pg[static_cast<std::size_t>(gi)]];
      result.generator_reactive_power_pu[static_cast<std::size_t>(gi)] =
          solved.x[layout.qg[static_cast<std::size_t>(gi)]];
    }
  } catch (const std::exception& error) {
    result.status = error.what();
  }
  return result;
}

std::vector<ThreePhaseHybridRelaxationResult>
solve_three_phase_hybrid_opf_relaxation_sequence(
    const std::vector<ThreePhaseHybridOPFCase>& problems,
    const ThreePhaseHybridRelaxationOptions& options) {
  std::vector<ThreePhaseHybridRelaxationResult> results;
  results.reserve(problems.size());
  for (const auto& problem : problems) {
    results.push_back(
        solve_three_phase_hybrid_opf_relaxation(problem, options));
  }
  return results;
}

}  // namespace hacdcpf::opf::phase_hybrid
