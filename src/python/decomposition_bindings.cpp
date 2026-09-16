// ─────────────────────────────────────────────────────────────────────────────
// decomposition_bindings.cpp — pybind11 bindings for the two-stage
// decomposition module, exposed as a submodule of the `mipsolvers` extension.
//
//   import mipsolvers
//   dec = mipsolvers.decomposition
//   res = dec.solve_stochastic(first, scenarios, method="benders")
//
// Compiled into the same mipsolvers_py extension; bind_decomposition() is called
// from mipsolvers_py.cpp at the end of PYBIND11_MODULE (mirrors bind_aml).
// ─────────────────────────────────────────────────────────────────────────────

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "mipsolvers/engine/decomposition/two_stage.hpp"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace mipsolvers::engine;
using namespace mipsolvers::engine::decomposition;

namespace {

Eigen::VectorXd vec_from_numpy(const py::array_t<double>& arr) {
  py::buffer_info info = arr.request();
  if (info.ndim != 1) throw std::invalid_argument("expected a 1-D float64 array");
  Eigen::VectorXd v(static_cast<Eigen::Index>(info.shape[0]));
  const double* ptr = static_cast<const double*>(info.ptr);
  const Eigen::Index stride = static_cast<Eigen::Index>(info.strides[0] / sizeof(double));
  for (Eigen::Index i = 0; i < v.size(); ++i) v[i] = ptr[i * stride];
  return v;
}

py::array_t<double> vec_to_numpy(const Eigen::VectorXd& v) {
  py::array_t<double> arr(static_cast<py::ssize_t>(v.size()));
  double* ptr = arr.mutable_data();
  for (Eigen::Index i = 0; i < v.size(); ++i) ptr[i] = v[i];
  return arr;
}

Eigen::SparseMatrix<double> sparse_from_dense(const py::array_t<double>& arr) {
  py::buffer_info info = arr.request();
  if (info.ndim != 2) throw std::invalid_argument("expected a 2-D float64 matrix");
  const Eigen::Index rows = static_cast<Eigen::Index>(info.shape[0]);
  const Eigen::Index cols = static_cast<Eigen::Index>(info.shape[1]);
  const double* ptr = static_cast<const double*>(info.ptr);
  const Eigen::Index rs = static_cast<Eigen::Index>(info.strides[0] / sizeof(double));
  const Eigen::Index cs = static_cast<Eigen::Index>(info.strides[1] / sizeof(double));
  std::vector<Eigen::Triplet<double>> t;
  for (Eigen::Index r = 0; r < rows; ++r)
    for (Eigen::Index c = 0; c < cols; ++c) {
      const double v = ptr[r * rs + c * cs];
      if (v != 0.0) t.emplace_back(r, c, v);
    }
  Eigen::SparseMatrix<double> sp(rows, cols);
  sp.setFromTriplets(t.begin(), t.end());
  sp.makeCompressed();
  return sp;
}

py::object get(const py::dict& d, const char* k) {
  return d.contains(k) ? d[k].cast<py::object>() : py::none();
}

py::array_t<double> req_arr(const py::dict& d, const char* k) {
  py::object o = get(d, k);
  if (o.is_none())
    throw std::invalid_argument(std::string("missing required key '") + k + "'");
  return o.cast<py::array_t<double>>();
}

std::vector<VariableMeta> vars_from(int n, const py::object& lb,
                                    const py::object& ub, const py::object& vtype) {
  std::vector<VariableMeta> vars(static_cast<std::size_t>(n));
  Eigen::VectorXd lbv, ubv;
  if (!lb.is_none()) lbv = vec_from_numpy(lb.cast<py::array_t<double>>());
  if (!ub.is_none()) ubv = vec_from_numpy(ub.cast<py::array_t<double>>());
  std::string vt = vtype.is_none() ? std::string() : vtype.cast<std::string>();
  for (int i = 0; i < n; ++i) {
    VariableMeta& v = vars[static_cast<std::size_t>(i)];
    v.lb = lbv.size() == n ? lbv[i] : -1e20;
    v.ub = ubv.size() == n ? ubv[i] : 1e20;
    const char ch = static_cast<int>(vt.size()) > i
                        ? static_cast<char>(std::toupper(static_cast<unsigned char>(vt[static_cast<std::size_t>(i)])))
                        : 'C';
    if (ch == 'B') {
      v.type = VarType::Binary;
      v.lb = 0.0;
      v.ub = 1.0;
    } else if (ch == 'I') {
      v.type = VarType::Integer;
    } else {
      v.type = VarType::Continuous;
    }
  }
  return vars;
}

FirstStage build_first_stage(const py::dict& d) {
  FirstStage f;
  f.c = vec_from_numpy(req_arr(d, "c"));
  const int n = static_cast<int>(f.c.size());
  py::object A = get(d, "A"), b = get(d, "b"), Aeq = get(d, "Aeq"), beq = get(d, "beq");
  if (!A.is_none()) {
    f.A = sparse_from_dense(A.cast<py::array_t<double>>());
    f.b = vec_from_numpy(b.cast<py::array_t<double>>());
  } else {
    f.A.resize(0, n);
    f.b.resize(0);
  }
  if (!Aeq.is_none()) {
    f.Aeq = sparse_from_dense(Aeq.cast<py::array_t<double>>());
    f.beq = vec_from_numpy(beq.cast<py::array_t<double>>());
  } else {
    f.Aeq.resize(0, n);
    f.beq.resize(0);
  }
  f.vars = vars_from(n, get(d, "lb"), get(d, "ub"), get(d, "vtype"));
  return f;
}

Recourse build_recourse(const py::dict& d) {
  Recourse r;
  r.d = vec_from_numpy(req_arr(d, "d"));
  r.T = sparse_from_dense(req_arr(d, "T"));
  r.W = sparse_from_dense(req_arr(d, "W"));
  r.h = vec_from_numpy(req_arr(d, "h"));
  r.vars = vars_from(static_cast<int>(r.d.size()), get(d, "lb"), get(d, "ub"),
                     get(d, "vtype"));
  py::object prob = get(d, "prob");
  r.probability = prob.is_none() ? 1.0 : prob.cast<double>();
  return r;
}

RobustRecourse build_robust_recourse(const py::dict& d) {
  RobustRecourse r;
  r.d = vec_from_numpy(req_arr(d, "d"));
  r.T = sparse_from_dense(req_arr(d, "T"));
  r.W = sparse_from_dense(req_arr(d, "W"));
  r.h0 = vec_from_numpy(req_arr(d, "h0"));
  r.P = sparse_from_dense(req_arr(d, "P"));
  r.vars = vars_from(static_cast<int>(r.d.size()), get(d, "lb"), get(d, "ub"),
                     get(d, "vtype"));
  return r;
}

UncertaintySet build_uncertainty(const py::dict& d) {
  UncertaintySet u;
  u.u_lb = vec_from_numpy(req_arr(d, "u_lb"));
  u.u_ub = vec_from_numpy(req_arr(d, "u_ub"));
  const int nu = static_cast<int>(u.u_lb.size());
  py::object G = get(d, "G"), g = get(d, "g");
  if (!G.is_none()) {
    u.G = sparse_from_dense(G.cast<py::array_t<double>>());
    u.g = vec_from_numpy(g.cast<py::array_t<double>>());
  } else {
    u.G.resize(0, nu);
    u.g.resize(0);
  }
  return u;
}

py::dict result_to_dict(const DecompositionResult& r) {
  py::dict o;
  o["success"] = r.success;
  o["status"] = r.status;
  o["objective"] = r.objective;
  o["x"] = vec_to_numpy(r.x);
  o["lower_bound"] = r.lower_bound;
  o["upper_bound"] = r.upper_bound;
  o["relative_gap"] = r.relative_gap;
  o["iterations"] = r.iterations;
  o["cuts_added"] = r.cuts_added;
  o["scenarios_generated"] = r.scenarios_generated;
  py::list ys;
  for (const auto& y : r.y) ys.append(vec_to_numpy(y));
  o["y"] = ys;
  return o;
}

py::dict solve_stochastic(const py::dict& first, const py::list& scenarios,
                          const std::string& method, const std::string& cut_mode,
                          int threads, int max_iterations, double gap,
                          double stabilization_alpha) {
  TwoStageModel m;
  m.first = build_first_stage(first);
  for (const auto& s : scenarios)
    m.scenarios.push_back(build_recourse(s.cast<py::dict>()));
  if (method == "extensive") return result_to_dict(solve_extensive_form_stochastic(m));
  BendersOptions opt;
  opt.threads = threads;
  opt.max_iterations = max_iterations;
  opt.gap_tolerance = gap;
  opt.stabilization_alpha = stabilization_alpha;
  if (cut_mode == "single")
    opt.cut_mode = BendersCutMode::SingleCut;
  else if (cut_mode == "integer")
    opt.cut_mode = BendersCutMode::IntegerLShaped;
  else if (cut_mode == "lagrangian")
    opt.cut_mode = BendersCutMode::Lagrangian;
  else
    opt.cut_mode = BendersCutMode::MultiCut;
  return result_to_dict(solve_benders_stochastic(m, opt));
}

py::dict solve_robust(const py::dict& first, const py::list& scenarios,
                      const std::string& method, int threads, int max_iterations,
                      double gap) {
  TwoStageModel m;
  m.first = build_first_stage(first);
  for (const auto& s : scenarios)
    m.scenarios.push_back(build_recourse(s.cast<py::dict>()));
  if (method == "extensive") return result_to_dict(solve_extensive_form_robust(m));
  CCGOptions opt;
  opt.threads = threads;
  opt.max_iterations = max_iterations;
  opt.gap_tolerance = gap;
  return result_to_dict(solve_ccg_robust(m, opt));
}

py::dict solve_robust_polyhedral(const py::dict& first, const py::dict& recourse,
                                 const py::dict& uncertainty, double big_m,
                                 int max_iterations, double gap) {
  PolyhedralRobustModel m;
  m.first = build_first_stage(first);
  m.recourse = build_robust_recourse(recourse);
  m.uncertainty = build_uncertainty(uncertainty);
  PolyhedralCCGOptions opt;
  opt.big_m = big_m;
  opt.max_iterations = max_iterations;
  opt.gap_tolerance = gap;
  return result_to_dict(solve_ccg_polyhedral_robust(m, opt));
}

}  // namespace

void bind_decomposition(py::module_& parent) {
  py::module_ m = parent.def_submodule(
      "decomposition",
      "Two-stage stochastic (L-shaped/Benders) and robust (CCG) decomposition.");

  m.def("solve_stochastic", &solve_stochastic, "first"_a, "scenarios"_a,
        "method"_a = "benders", "cut_mode"_a = "multi", "threads"_a = 1,
        "max_iterations"_a = 500, "gap"_a = 1e-6, "stabilization_alpha"_a = 0.0,
        R"doc(Solve a two-stage stochastic program.

first: dict{c, A?, b?, Aeq?, beq?, lb?, ub?, vtype?} (numpy arrays; vtype str of C/I/B).
scenarios: list of dict{d, T, W, h, lb?, ub?, vtype?, prob?} with W y >= h - T x.
method: "benders" or "extensive".
cut_mode: "multi" or "single" (continuous recourse); "integer" (Laporte-Louveaux)
          or "lagrangian" (SDDiP-style) for a binary first stage with integer recourse.
threads: worker threads for the per-scenario subproblems (>=1).
stabilization_alpha: in-out stabilization in [0,1) for a continuous first stage.
Returns a dict with success, status, objective, x, lower_bound, upper_bound,
relative_gap, iterations, cuts_added, y.)doc");

  m.def("solve_robust", &solve_robust, "first"_a, "scenarios"_a,
        "method"_a = "ccg", "threads"_a = 1, "max_iterations"_a = 200,
        "gap"_a = 1e-6,
        R"doc(Solve a two-stage robust program over a finite uncertainty set
(column-and-constraint generation). Accepts continuous or integer recourse.
method: "ccg" or "extensive".)doc");

  m.def("solve_robust_polyhedral", &solve_robust_polyhedral, "first"_a,
        "recourse"_a, "uncertainty"_a, "big_m"_a = 1e6, "max_iterations"_a = 200,
        "gap"_a = 1e-6,
        R"doc(Solve a two-stage robust program over a continuous polyhedral
uncertainty set (CCG with a KKT max-min oracle; continuous recourse).

recourse: dict{d, T, W, h0, P, lb?, ub?} with W y >= h0 + P u - T x.
uncertainty: dict{u_lb, u_ub, G?, g?} with U = {u_lb <= u <= u_ub, G u <= g}.)doc");
}
