// ─────────────────────────────────────────────────────────────────────────────
// aml_bindings.cpp — pybind11 bindings for the AML (Algebraic Modeling
// Library) exposed as a submodule of the `mipsolvers` Python extension.
//
// Accessed as:
//   import mipsolvers
//   aml = mipsolvers.aml
//   m   = aml.Model("my_model")
//
// This file is compiled into the same mipsolvers_py.so as the SCUC/engine
// bindings. The top-level registration function bind_aml(py::module_& parent)
// is called from mipsolvers_py.cpp at the end of PYBIND11_MODULE.
// ─────────────────────────────────────────────────────────────────────────────

#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "mipsolvers/aml/aml.hpp"

namespace py = pybind11;
using namespace mipsolvers::aml;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: convert a Python key argument to aml::Key
//   - str       → Key::scalar(str)
//   - tuple/list→ Key::make({...})
//   - Key       → identity
// ─────────────────────────────────────────────────────────────────────────────
static Key py_to_key(const py::object& obj) {
  if (py::isinstance<Key>(obj)) {
    return obj.cast<Key>();
  }
  if (py::isinstance<py::str>(obj)) {
    return Key::scalar(obj.cast<std::string>());
  }
  if (py::isinstance<py::tuple>(obj) || py::isinstance<py::list>(obj)) {
    std::vector<Atom> atoms;
    for (auto item : obj) atoms.push_back(item.cast<std::string>());
    return Key{atoms};
  }
  throw py::type_error(
      "Key argument must be str, tuple[str,...], list[str] or aml.Key");
}

// ─────────────────────────────────────────────────────────────────────────────
// Public entry point — called from mipsolvers_py.cpp
// ─────────────────────────────────────────────────────────────────────────────
void bind_aml(py::module_& parent) {
  py::module_ m = parent.def_submodule("aml",
      "Algebraic Modeling Library (AML): set-indexed variable/parameter "
      "construction, linear/quadratic/nonlinear objective and constraint "
      "building, and solver dispatch.");

  // ──────────────────────────────────────────────────────────────────────────
  // Key
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<Key>(m, "Key",
      "A multi-dimensional index key (sequence of string atoms).")
    .def(py::init([](py::object arg) { return py_to_key(arg); }),
         py::arg("atoms"),
         "Construct from str, tuple[str,...] or list[str].")
    .def_static("scalar", &Key::scalar, py::arg("atom"),
         "Create a 1-D key from a single atom string.")
    .def_static("pair",   &Key::pair,   py::arg("a"), py::arg("b"),
         "Create a 2-D key from two atom strings.")
    .def_static("make",
         [](const std::vector<std::string>& atoms) {
           return Key{atoms};
         }, py::arg("atoms"),
         "Create a key from a list of atom strings.")
    .def_property_readonly("values",
         [](const Key& k) { return k.values; },
         "Underlying atom list.")
    .def_property_readonly("dimension",
         [](const Key& k) { return k.dimension(); })
    .def("__repr__",
         [](const Key& k) {
           std::string s = "Key(";
           for (std::size_t i = 0; i < k.values.size(); ++i) {
             if (i) s += ", ";
             s += "'" + k.values[i] + "'";
           }
           return s + ")";
         })
    .def("__eq__",  &Key::operator==)
    .def("__lt__",  &Key::operator<)
    .def("__hash__",
         [](const Key& k) { return KeyHash{}(k); });

  // ──────────────────────────────────────────────────────────────────────────
  // Enums
  // ──────────────────────────────────────────────────────────────────────────
  py::enum_<VarType>(m, "VarType")
    .value("Continuous", VarType::Continuous)
    .value("Integer",    VarType::Integer)
    .value("Binary",     VarType::Binary)
    .export_values();

  py::enum_<MissingPolicy>(m, "MissingPolicy")
    .value("Error",      MissingPolicy::Error)
    .value("ReturnZero", MissingPolicy::ReturnZero)
    .export_values();

  py::enum_<TerminationStatus>(m, "TerminationStatus")
    .value("Optimal",                TerminationStatus::Optimal)
    .value("Infeasible",             TerminationStatus::Infeasible)
    .value("Unbounded",              TerminationStatus::Unbounded)
    .value("InfeasibleOrUnbounded",  TerminationStatus::InfeasibleOrUnbounded)
    .value("TimeLimit",              TerminationStatus::TimeLimit)
    .value("IterationLimit",         TerminationStatus::IterationLimit)
    .value("NodeLimit",              TerminationStatus::NodeLimit)
    .value("ObjectiveLimit",         TerminationStatus::ObjectiveLimit)
    .value("NumericalError",         TerminationStatus::NumericalError)
    .value("UserInterrupt",          TerminationStatus::UserInterrupt)
    .value("SolverError",            TerminationStatus::SolverError)
    .value("Unknown",                TerminationStatus::Unknown)
    .export_values();

  py::enum_<PrimalStatus>(m, "PrimalStatus")
    .value("Optimal",    PrimalStatus::Optimal)
    .value("Feasible",   PrimalStatus::Feasible)
    .value("Infeasible", PrimalStatus::Infeasible)
    .value("NoSolution", PrimalStatus::NoSolution)
    .value("Unknown",    PrimalStatus::Unknown)
    .export_values();

  py::enum_<DualStatus>(m, "DualStatus")
    .value("Optimal",    DualStatus::Optimal)
    .value("Feasible",   DualStatus::Feasible)
    .value("Infeasible", DualStatus::Infeasible)
    .value("NoSolution", DualStatus::NoSolution)
    .value("Unknown",    DualStatus::Unknown)
    .export_values();

  // ──────────────────────────────────────────────────────────────────────────
  // VarRef
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<VarRef>(m, "VarRef",
      "Reference to a single scalar decision variable.")
    .def_property_readonly("id", &VarRef::id,
         "Integer column index in the compiled matrix.")
    .def("__repr__",
         [](const VarRef& v) {
           return "VarRef(id=" + std::to_string(v.id()) + ")";
         })
    .def("__mul__",
         [](const VarRef& v, double c) -> LinearExpr {
           return static_cast<LinearExpr>(v) * c;
         })
    .def("__rmul__",
         [](const VarRef& v, double c) -> LinearExpr {
           return c * static_cast<LinearExpr>(v);
         })
    .def("__add__",
         [](const VarRef& v, const VarRef& w) -> LinearExpr {
           return static_cast<LinearExpr>(v) + static_cast<LinearExpr>(w);
         })
    .def("__add__",
         [](const VarRef& v, const LinearExpr& e) -> LinearExpr {
           return static_cast<LinearExpr>(v) + e;
         })
    .def("__radd__",
         [](const VarRef& v, const LinearExpr& e) -> LinearExpr {
           return e + static_cast<LinearExpr>(v);
         })
    .def("__neg__",
         [](const VarRef& v) -> LinearExpr {
           return LinearExpr::from_var(v.id(), -1.0);
         })
    .def("__sub__",
         [](const VarRef& v, const VarRef& w) -> LinearExpr {
           LinearExpr r = LinearExpr::from_var(v.id());
           r -= LinearExpr::from_var(w.id());
           return r;
         })
    .def("__sub__",
         [](const VarRef& v, const LinearExpr& e) -> LinearExpr {
           return static_cast<LinearExpr>(v) - e;
         })
    .def("__le__",
         [](const VarRef& v, double rhs) -> TempConstr { return v <= rhs; })
    .def("__ge__",
         [](const VarRef& v, double rhs) -> TempConstr { return v >= rhs; })
    .def("__eq__",
         [](const VarRef& v, double rhs) -> TempConstr { return v == rhs; },
         py::is_operator());

  // ──────────────────────────────────────────────────────────────────────────
  // LinearExpr
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<LinearExpr>(m, "LinearExpr",
      "Sparse affine expression: c0 + sum_i coef_i * x_i.")
    .def(py::init<>())
    .def_static("from_var",
         [](VarId vid, double coef) { return LinearExpr::from_var(vid, coef); },
         py::arg("var_id"), py::arg("coef") = 1.0)
    .def_static("const_expr", &LinearExpr::const_expr, py::arg("c"),
         "Create a constant expression.")
    .def_readwrite("constant", &LinearExpr::constant)
    .def("__iadd__",
         [](LinearExpr& self, const LinearExpr& o) -> LinearExpr& {
           self += o; return self;
         }, py::is_operator())
    .def("__iadd__",
         [](LinearExpr& self, double c) -> LinearExpr& {
           self += c; return self;
         }, py::is_operator())
    .def("__add__",
         [](const LinearExpr& a, const LinearExpr& b) { return a + b; })
    .def("__add__",
         [](const LinearExpr& a, double c) {
           LinearExpr r = a; r += c; return r;
         })
    .def("__radd__",
         [](const LinearExpr& a, double c) {
           LinearExpr r = a; r += c; return r;
         })
    .def("__sub__",
         [](const LinearExpr& a, const LinearExpr& b) { return a - b; })
    .def("__neg__",
         [](const LinearExpr& a) { return -a; })
    .def("__mul__",
         [](const LinearExpr& a, double s) { return a * s; })
    .def("__rmul__",
         [](const LinearExpr& a, double s) { return s * a; })
    .def("__le__",
         [](const LinearExpr& lhs, double rhs) -> TempConstr { return lhs <= rhs; })
    .def("__ge__",
         [](const LinearExpr& lhs, double rhs) -> TempConstr { return lhs >= rhs; })
    .def("__eq__",
         [](const LinearExpr& lhs, double rhs) -> TempConstr { return lhs == rhs; },
         py::is_operator())
    .def("__le__",
         [](const LinearExpr& lhs, const LinearExpr& rhs) -> TempConstr {
           return lhs <= rhs;
         })
    .def("__ge__",
         [](const LinearExpr& lhs, const LinearExpr& rhs) -> TempConstr {
           return lhs >= rhs;
         })
    .def("__repr__",
         [](const LinearExpr& e) {
           return "<LinearExpr with " + std::to_string(e.terms.size()) +
                  " terms + constant=" + std::to_string(e.constant) + ">";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // QuadExpr
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<QuadExpr>(m, "QuadExpr",
      "Quadratic expression: linear part + quadratic terms.")
    .def(py::init<>())
    .def_static("sq",
         [](VarId vid, double coef) { return QuadExpr::sq(vid, coef); },
         py::arg("var_id"), py::arg("coef") = 1.0,
         "Create coef * x_i^2.")
    .def_static("bilinear",
         [](VarId i, VarId j, double coef) {
           return QuadExpr::bilinear(i, j, coef);
         },
         py::arg("var_i"), py::arg("var_j"), py::arg("coef") = 1.0,
         "Create coef * x_i * x_j.")
    .def_static("from_linear", &QuadExpr::from_linear, py::arg("lin"),
         "Wrap a LinearExpr in a QuadExpr (zero quadratic terms).")
    .def("__iadd__",
         [](QuadExpr& self, const QuadExpr& o) -> QuadExpr& {
           self += o; return self;
         }, py::is_operator())
    .def("__add__",
         [](const QuadExpr& a, const QuadExpr& b) { return a + b; })
    .def("__sub__",
         [](const QuadExpr& a, const QuadExpr& b) { return a - b; })
    .def("__mul__",
         [](const QuadExpr& a, double s) { return a * s; })
    .def("__rmul__",
         [](const QuadExpr& a, double s) { return s * a; })
    .def("__repr__",
         [](const QuadExpr& e) {
           return "<QuadExpr: " +
                  std::to_string(e.quad_terms.size()) + " quad terms, " +
                  std::to_string(e.linear_part.terms.size()) + " linear terms>";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // NonlinearExpr  (opaque arena handle)
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<NonlinearExpr>(m, "NonlinearExpr",
      "Opaque handle into the model's nonlinear expression arena. "
      "Create via Model.nl_var(), nl_const(), nl_add(), nl_cos(), etc.")
    .def_property_readonly("id", &NonlinearExpr::id)
    .def("is_null", &NonlinearExpr::is_null,
         "True if this handle points to no node (default-constructed).")
    .def("__repr__",
         [](const NonlinearExpr& e) {
           if (e.is_null()) return std::string("NonlinearExpr(<null>)");
           return "NonlinearExpr(id=" + std::to_string(e.id()) + ")";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // TempConstr
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<TempConstr>(m, "TempConstr",
      "Temporary constraint handle produced by comparison operators.")
    .def("__repr__",
         [](const TempConstr& c) {
           const char* op = (c.sense == CompareOp::LessEq)    ? "<="
                          : (c.sense == CompareOp::GreaterEq) ? ">="
                                                               : "==";
           return std::string("<TempConstr [lhs] ") + op + " [rhs]>";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // ConstraintRef
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<ConstraintRef>(m, "ConstraintRef",
      "Reference to a single named constraint row.")
    .def_property_readonly("id",   &ConstraintRef::id)
    .def_property_readonly("name", &ConstraintRef::name)
    .def("__repr__",
         [](const ConstraintRef& c) {
           return "ConstraintRef(id=" + std::to_string(c.id()) +
                  ", name='" + c.name() + "')";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // ConstraintArray
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<ConstraintArray>(m, "ConstraintArray",
      "Indexed family of constraints, accessible by Key.")
    .def_property_readonly("name",        &ConstraintArray::name)
    .def_property_readonly("total_count", &ConstraintArray::total_count)
    .def("__call__",
         [](const ConstraintArray& a, py::object key) -> ConstraintRef {
           return a(py_to_key(key));
         }, py::arg("key"))
    .def("__getitem__",
         [](const ConstraintArray& a, py::object key) -> ConstraintRef {
           return a(py_to_key(key));
         }, py::arg("key"),
         "Access by Key, str or tuple[str,...].")
    .def("__repr__",
         [](const ConstraintArray& a) {
           return "ConstraintArray(name='" + a.name() +
                  "', size=" + std::to_string(a.total_count()) + ")";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // VarArray
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<VarArray>(m, "VarArray",
      "Indexed family of decision variables over a domain set.")
    .def_property_readonly("name",        &VarArray::name)
    .def_property_readonly("type",        &VarArray::type)
    .def_property_readonly("total_count", &VarArray::total_count)
    .def("__call__",
         [](const VarArray& a, py::object key) -> VarRef {
           return a(py_to_key(key));
         }, py::arg("key"))
    .def("__getitem__",
         [](const VarArray& a, py::object key) -> VarRef {
           return a(py_to_key(key));
         }, py::arg("key"),
         "Access variable by Key, str or tuple[str, str].")
    .def("set_lb",
         [](VarArray& a, py::object key, double lb) {
           a.set_lb(py_to_key(key), lb);
         }, py::arg("key"), py::arg("lb"))
    .def("set_ub",
         [](VarArray& a, py::object key, double ub) {
           a.set_ub(py_to_key(key), ub);
         }, py::arg("key"), py::arg("ub"))
    .def("fix",
         [](VarArray& a, py::object key, double value) {
           a.fix(py_to_key(key), value);
         }, py::arg("key"), py::arg("value"))
    .def("lb",
         [](const VarArray& a, py::object key) {
           return a.lb(py_to_key(key));
         }, py::arg("key"))
    .def("ub",
         [](const VarArray& a, py::object key) {
           return a.ub(py_to_key(key));
         }, py::arg("key"))
    .def("__repr__",
         [](const VarArray& a) {
           return "VarArray(name='" + a.name() +
                  "', size=" + std::to_string(a.total_count()) + ")";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // Parameter
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<Parameter>(m, "Parameter",
      "Named table of double-precision parameter values indexed by Key.")
    .def_property_readonly("name",      &Parameter::name)
    .def_property_readonly("dimension", &Parameter::dimension)
    .def_property_readonly("unit",      &Parameter::unit)
    .def("set",
         [](Parameter& p, py::object key, double v) {
           p.set(py_to_key(key), v);
         }, py::arg("key"), py::arg("value"))
    .def("set_scalar", &Parameter::set_scalar, py::arg("value"),
         "Set value for a scalar (dim-0) parameter.")
    .def("get",
         [](const Parameter& p, py::object key) {
           return p.get(py_to_key(key));
         }, py::arg("key"))
    .def("get_scalar", &Parameter::get_scalar,
         "Get value of a scalar (dim-0) parameter.")
    .def("get_or",
         [](const Parameter& p, py::object key, double default_val) {
           return p.get_or(py_to_key(key), default_val);
         }, py::arg("key"), py::arg("default") = 0.0)
    .def("contains",
         [](const Parameter& p, py::object key) {
           return p.contains(py_to_key(key));
         }, py::arg("key"))
    .def("load",
         [](Parameter& p, const py::dict& d) {
           for (auto item : d) {
             Key k = py_to_key(py::reinterpret_borrow<py::object>(item.first));
             p.set(k, item.second.cast<double>());
           }
         }, py::arg("data"),
         "Bulk-load from a dict mapping key→float. "
         "Keys may be str or tuple[str,...].")
    .def("__setitem__",
         [](Parameter& p, py::object key, double v) {
           p.set(py_to_key(key), v);
         })
    .def("__getitem__",
         [](const Parameter& p, py::object key) {
           return p.get(py_to_key(key));
         })
    .def("to_dict",
         [](const Parameter& p) {
           py::dict d;
           for (const auto& [k, v] : p.to_map()) {
             if (k.dimension() == 1)
               d[py::str(k.values[0])] = v;
             else {
               py::tuple t(k.values.size());
               for (std::size_t i = 0; i < k.values.size(); ++i)
                 t[i] = py::str(k.values[i]);
               d[t] = v;
             }
           }
           return d;
         },
         "Return all values as a dict {key: float}.")
    .def("__repr__",
         [](const Parameter& p) {
           return "Parameter('" + p.name() + "', unit='" + p.unit() + "')";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // Set / ExplicitSet / OrderedSet
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<Set>(m, "Set",
      "Abstract base: a finite, indexable collection of Keys.")
    .def_property_readonly("name",        &Set::name)
    .def_property_readonly("dimension",   &Set::dimension)
    .def_property_readonly("cardinality", &Set::cardinality)
    .def("contains",
         [](const Set& s, py::object key) { return s.contains(py_to_key(key)); },
         py::arg("key"))
    .def_property_readonly("elements", &Set::elements,
         "Ordered list of all Keys in this set.")
    .def("add_element",
         [](Set& s, py::object key) { return s.add_element(py_to_key(key)); },
         py::arg("key"), "Add a Key to the set. Returns False if already present.")
    .def("__contains__",
         [](const Set& s, py::object key) { return s.contains(py_to_key(key)); })
    .def("__len__", &Set::cardinality)
    .def("__iter__",
         [](const Set& s) {
           return py::make_iterator(s.elements().begin(), s.elements().end());
         }, py::keep_alive<0, 1>())
    .def("__repr__",
         [](const Set& s) {
           return "Set('" + s.name() + "', size=" +
                  std::to_string(s.cardinality()) + ")";
         });

  py::class_<ExplicitSet, Set>(m, "ExplicitSet",
      "User-managed, insertion-ordered set.")
    .def("__repr__",
         [](const ExplicitSet& s) {
           return "ExplicitSet('" + s.name() + "', size=" +
                  std::to_string(s.cardinality()) + ")";
         });

  py::class_<OrderedSet, ExplicitSet>(m, "OrderedSet",
      "ExplicitSet with prev/next navigation (for time-period indexing).")
    .def("at",       &OrderedSet::at,       py::arg("pos"),
         "0-based positional access.")
    .def("position", &OrderedSet::position, py::arg("key"),
         "0-based position of key; throws if absent.")
    .def("prev",
         [](const OrderedSet& s, py::object key) -> py::object {
           auto r = s.prev(py_to_key(key));
           return r ? py::cast(*r) : py::none();
         }, py::arg("key"), "Previous key, or None if this is the first.")
    .def("next",
         [](const OrderedSet& s, py::object key) -> py::object {
           auto r = s.next(py_to_key(key));
           return r ? py::cast(*r) : py::none();
         }, py::arg("key"), "Next key, or None if this is the last.")
    .def("__repr__",
         [](const OrderedSet& s) {
           return "OrderedSet('" + s.name() + "', size=" +
                  std::to_string(s.cardinality()) + ")";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // SolveOptions
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<SolveOptions>(m, "SolveOptions",
      "Options forwarded to the solver engine.")
    .def(py::init<>())
    .def_readwrite("solver_name",    &SolveOptions::solver_name,
         "Solver name ('', 'highs', 'native', …). Empty = auto-select.")
    .def_readwrite("time_limit_sec", &SolveOptions::time_limit_sec)
    .def_readwrite("mip_gap_tol",    &SolveOptions::mip_gap_tol)
    .def_readwrite("verbosity",      &SolveOptions::verbosity,
         "0 = silent, 1 = summary, 2 = verbose.")
    .def("__repr__",
         [](const SolveOptions& o) {
           return "SolveOptions(solver='" + o.solver_name + "', "
                  "time_limit=" + std::to_string(o.time_limit_sec) + ")";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // SolveResult
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<SolveResult>(m, "SolveResult",
      "Result returned by Model.solve().")
    .def_readonly("termination_status", &SolveResult::termination_status)
    .def_readonly("primal_status",      &SolveResult::primal_status)
    .def_readonly("dual_status",        &SolveResult::dual_status)
    .def_property_readonly("has_primal", &SolveResult::has_primal)
    .def_property_readonly("is_optimal", &SolveResult::is_optimal)
    .def_property_readonly("has_duals",  &SolveResult::has_duals)
    .def_readonly("objective_value",     &SolveResult::objective_value)
    .def_readonly("objective_bound",     &SolveResult::objective_bound)
    .def_readonly("optimality_gap",      &SolveResult::optimality_gap)
    .def_readonly("solve_time_sec",      &SolveResult::solve_time_sec)
    .def_readonly("simplex_iterations",  &SolveResult::simplex_iterations)
    .def_readonly("branch_and_cut_nodes",&SolveResult::branch_and_cut_nodes)
    .def_readonly("solver_used",         &SolveResult::solver_used)
    .def("var_value",
         [](const SolveResult& sr, const VarRef& v) {
           return sr.var_value(v);
         }, py::arg("var"),
         "Get primal value of a variable (by VarRef).")
    .def("var_value_by_id",
         [](const SolveResult& sr, VarId vid) {
           return sr.var_value(vid);
         }, py::arg("var_id"),
         "Get primal value of a variable (by integer VarId).")
    .def("array_values",
         [](const SolveResult& sr, const VarArray& a) -> py::dict {
           py::dict d;
           for (const auto& [k, vid] : a.key_to_id()) {
             py::object py_key;
             if (k.dimension() == 1)
               py_key = py::str(k.values[0]);
             else {
               py::tuple t(k.values.size());
               for (std::size_t i = 0; i < k.values.size(); ++i)
                 t[i] = py::str(k.values[i]);
               py_key = t;
             }
             d[py_key] = sr.var_value(vid);
           }
           return d;
         }, py::arg("var_array"),
         "Return all values of a VarArray as dict {key: float}.")
    .def("dual",
         [](const SolveResult& sr, const ConstraintRef& c) -> py::object {
           auto v = sr.dual(c);
           return v ? py::cast(*v) : py::none();
         }, py::arg("constraint"))
    .def("dual_array",
         [](const SolveResult& sr, const ConstraintArray& a) -> py::dict {
           py::dict d;
           for (const auto& [k, cid] : a.key_to_id()) {
             auto v = sr.dual(cid);
             if (!v) continue;
             py::object py_key;
             if (k.dimension() == 1)
               py_key = py::str(k.values[0]);
             else {
               py::tuple t(k.values.size());
               for (std::size_t i = 0; i < k.values.size(); ++i)
                 t[i] = py::str(k.values[i]);
               py_key = t;
             }
             d[py_key] = *v;
           }
           return d;
         }, py::arg("constraint_array"),
         "Return dual values of a ConstraintArray as dict {key: float}.")
    .def("reduced_cost",
         [](const SolveResult& sr, const VarRef& v) -> py::object {
           auto r = sr.reduced_cost(v);
           return r ? py::cast(*r) : py::none();
         }, py::arg("var"))
    .def("__repr__",
         [](const SolveResult& sr) {
           return "<SolveResult: obj=" + std::to_string(sr.objective_value) +
                  ", " +
                  (sr.is_optimal() ? "OPTIMAL" :
                   sr.has_primal() ? "FEASIBLE" : "NO_SOLUTION") +
                  ">";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // Model
  // ──────────────────────────────────────────────────────────────────────────
  py::class_<Model>(m, "Model",
      "Top-level AML model container. Build sets → params → vars → "
      "constraints → objective, then call solve().")
    .def(py::init<const std::string&>(), py::arg("name") = "",
         "Create a new empty model.")
    // ---- Set construction -----------------------------------------------
    .def("add_set",
         [](Model& mdl, const std::string& name,
            const std::vector<std::string>& atoms) -> ExplicitSet& {
           return mdl.add_set(name, atoms);
         },
         py::arg("name"), py::arg("atoms") = std::vector<std::string>{},
         "Add a 1-D explicit set.",
         py::return_value_policy::reference_internal)
    .def("add_ordered_set",
         [](Model& mdl, const std::string& name,
            const std::vector<std::string>& atoms) -> OrderedSet& {
           return mdl.add_ordered_set(name, atoms);
         },
         py::arg("name"), py::arg("atoms") = std::vector<std::string>{},
         "Add a 1-D ordered set (for time-period indexing).",
         py::return_value_policy::reference_internal)
    // ---- Parameter construction -----------------------------------------
    .def("add_param",
         [](Model& mdl, const std::string& name, int dim,
            const std::string& unit) -> Parameter& {
           return mdl.add_param(name, dim, unit);
         },
         py::arg("name"), py::arg("dim") = 1, py::arg("unit") = "",
         "Add an N-dimensional parameter.",
         py::return_value_policy::reference_internal)
    .def("add_param_scalar",
         [](Model& mdl, const std::string& name,
            const std::string& unit) -> Parameter& {
           return mdl.add_param_scalar(name, unit);
         },
         py::arg("name"), py::arg("unit") = "",
         "Add a scalar (dim-0) parameter.",
         py::return_value_policy::reference_internal)
    // ---- Variable construction ------------------------------------------
    .def("add_var",
         [](Model& mdl, const std::string& name, Set& domain,
            VarType vtype, double lb, double ub) -> VarArray& {
           return mdl.add_var(name, domain, vtype, lb, ub);
         },
         py::arg("name"), py::arg("domain"), py::arg("type"),
         py::arg("lb") = -1e20, py::arg("ub") = 1e20,
         "Add a 1-D variable array over a set.",
         py::return_value_policy::reference_internal)
    .def("add_var2",
         [](Model& mdl, const std::string& name,
            Set& domain_a, Set& domain_b,
            VarType vtype, double lb, double ub) -> VarArray& {
           return mdl.add_var(name, domain_a, domain_b, vtype, lb, ub);
         },
         py::arg("name"), py::arg("domain_a"), py::arg("domain_b"),
         py::arg("type"), py::arg("lb") = -1e20, py::arg("ub") = 1e20,
         "Add a 2-D variable array (Cartesian product of two sets).",
         py::return_value_policy::reference_internal)
    // ---- Objective ------------------------------------------------------
    .def("minimize",
         [](Model& mdl, const LinearExpr& obj) { mdl.minimize(obj); },
         py::arg("obj"), "Set minimization objective (linear).")
    .def("maximize",
         [](Model& mdl, const LinearExpr& obj) { mdl.maximize(obj); },
         py::arg("obj"), "Set maximization objective (linear).")
    .def("minimize_quad",
         [](Model& mdl, const QuadExpr& obj) { mdl.minimize(obj); },
         py::arg("obj"), "Set minimization objective (quadratic).")
    .def("maximize_quad",
         [](Model& mdl, const QuadExpr& obj) { mdl.maximize(obj); },
         py::arg("obj"), "Set maximization objective (quadratic).")
    // ---- Nonlinear objective  -------------------------------------------
    .def("minimize_nl",
         [](Model& mdl, const NonlinearExpr& obj) { mdl.minimize(obj); },
         py::arg("obj"), "Set minimization objective (nonlinear).")
    .def("maximize_nl",
         [](Model& mdl, const NonlinearExpr& obj) { mdl.maximize(obj); },
         py::arg("obj"), "Set maximization objective (nonlinear).")
    // ---- NL expression builders ----------------------------------------
    .def("nl_var",   &Model::nl_var,   py::arg("v"),
         "Wrap a VarRef as a NonlinearExpr leaf.")
    .def("nl_const", &Model::nl_const, py::arg("c"),
         "Wrap a constant double as a NonlinearExpr leaf.")
    .def("nl_neg",  &Model::nl_neg,  py::arg("a"))
    .def("nl_add",  &Model::nl_add,  py::arg("a"), py::arg("b"))
    .def("nl_sub",  &Model::nl_sub,  py::arg("a"), py::arg("b"))
    .def("nl_mul",  &Model::nl_mul,  py::arg("a"), py::arg("b"))
    .def("nl_div",  &Model::nl_div,  py::arg("a"), py::arg("b"))
    .def("nl_sq",   &Model::nl_sq,   py::arg("a"))
    .def("nl_pow",  &Model::nl_pow,  py::arg("a"), py::arg("n"))
    .def("nl_sqrt", &Model::nl_sqrt, py::arg("a"))
    .def("nl_exp",  &Model::nl_exp,  py::arg("a"))
    .def("nl_log",  &Model::nl_log,  py::arg("a"))
    .def("nl_sin",  &Model::nl_sin,  py::arg("a"))
    .def("nl_cos",  &Model::nl_cos,  py::arg("a"))
    .def("nl_tan",  &Model::nl_tan,  py::arg("a"))
    .def("nl_abs",  &Model::nl_abs,  py::arg("a"))
    .def("nl_max",  &Model::nl_max,  py::arg("a"), py::arg("b"))
    .def("nl_min",  &Model::nl_min,  py::arg("a"), py::arg("b"))
    // ---- Nonlinear constraint addition ---------------------------------
    .def("add_nl_constraint",
         [](Model& mdl, const std::string& name,
            const NonlinearExpr& expr,
            const std::string& sense_str,
            double rhs) -> ConstraintRef {
           CompareOp sense = CompareOp::Equal;
           if      (sense_str == "<=" || sense_str == "le") sense = CompareOp::LessEq;
           else if (sense_str == ">=" || sense_str == "ge") sense = CompareOp::GreaterEq;
           else if (sense_str == "==" || sense_str == "eq") sense = CompareOp::Equal;
           else throw py::value_error("sense must be '<=', '>=', or '=='");
           return mdl.add_nl_constraint(name, expr, sense, rhs);
         },
         py::arg("name"), py::arg("expr"),
         py::arg("sense") = "==", py::arg("rhs") = 0.0,
         "Add a nonlinear constraint: expr [sense] rhs. sense = '<=','>=','=='.")
    // ---- NLP warm start ------------------------------------------------
    .def("set_nlp_x0",
         [](Model& mdl, const std::vector<double>& x0) {
           mdl.set_nlp_x0(x0);
         },
         py::arg("x0"), "Provide an initial point for the NLP solver.")
    // ---- Constraint addition --------------------------------------------
    .def("add_constraint",
         [](Model& mdl, const TempConstr& c,
            const std::string& name) -> ConstraintRef {
           return mdl.add_constraint(c, name);
         },
         py::arg("constraint"), py::arg("name") = "",
         "Add a single named constraint.")
    .def("add_constraints",
         [](Model& mdl, const std::string& family,
            Set& domain,
            const py::function& fn) -> ConstraintArray& {
           return mdl.add_constraints(family, domain,
               [&fn](const Key& k) -> TempConstr {
                 return fn(k).cast<TempConstr>();
               });
         },
         py::arg("family"), py::arg("domain"), py::arg("fn"),
         "Add an indexed family of constraints. fn(Key) -> TempConstr.",
         py::return_value_policy::reference_internal)
    // ---- Inspect --------------------------------------------------------
    .def_property_readonly("num_vars",        &Model::num_vars)
    .def_property_readonly("num_constraints", &Model::num_constraints)
    .def_property_readonly("name",            &Model::name)
    .def("print_summary",        &Model::print_summary)
    .def("check_bounds",         &Model::check_bounds)
    .def("check_missing_params", &Model::check_missing_params)
    // ---- Solve ----------------------------------------------------------
    .def("solve",
         [](Model& mdl, const SolveOptions& opts) -> SolveResult {
           return mdl.solve(opts);
         },
         py::arg("options") = SolveOptions{},
         "Compile and solve the model. Returns a SolveResult.")
    // ---- Export ---------------------------------------------------------
    .def("write_lp",
         [](const Model& mdl, const std::string& path) { mdl.write_lp(path); },
         py::arg("path"), "Write LP format file.")
    .def("write_mps",
         [](const Model& mdl, const std::string& path) { mdl.write_mps(path); },
         py::arg("path"), "Write MPS format file.")
    .def("write_json",
         [](const Model& mdl, const std::string& path) { mdl.write_json(path); },
         py::arg("path"), "Write JSON model export (Beta).")
    .def("__repr__",
         [](const Model& mdl) {
           return "Model('" + mdl.name() + "', vars=" +
                  std::to_string(mdl.num_vars()) + ", constrs=" +
                  std::to_string(mdl.num_constraints()) + ")";
         });

  // ──────────────────────────────────────────────────────────────────────────
  // sum_over helpers
  // ──────────────────────────────────────────────────────────────────────────
  m.def("sum_over",
        [](const Set& s, const py::function& fn) -> LinearExpr {
          LinearExpr result;
          for (const auto& k : s.elements()) {
            result += fn(k).cast<LinearExpr>();
          }
          return result;
        },
        py::arg("set"), py::arg("fn"),
        "Sum LinearExpr values over all keys in a set. fn(Key) -> LinearExpr.");

  m.def("sum_over_quad",
        [](const Set& s, const py::function& fn) -> QuadExpr {
          QuadExpr result;
          for (const auto& k : s.elements()) {
            result += fn(k).cast<QuadExpr>();
          }
          return result;
        },
        py::arg("set"), py::arg("fn"),
        "Sum QuadExpr values over all keys in a set. fn(Key) -> QuadExpr.");
}
