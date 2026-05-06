/**
 * @file mipsolvers_py.cpp
 * @brief pybind11 Python bindings for MIPSolvers: SCUC market dispatch module
 *        and SolverEngine LP/MILP interface.
 *
 * Python 3.8+ compatible. Build with CMake option -DMIPSOLVERS_BUILD_PYTHON=ON.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// SCUC public API
#include "mipsolvers/scuc/scuc.hpp"

// Engine public API
#include "mipsolvers/engine/engine.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/problem_types.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

// ─────────────────────────────────────────────────────────────────────────────
// Helpers: numpy <-> Eigen
// ─────────────────────────────────────────────────────────────────────────────
namespace {

[[noreturn]] void rethrow_scuc_exception(const std::string& context,
                                         const std::exception& ex) {
  throw std::runtime_error(context + ": " + ex.what());
}

mipsolvers::scuc::SCUCInput scuc_from_json_checked(const std::string& json_str) {
  try {
    return mipsolvers::scuc::scuc_from_json(json_str);
  } catch (const nlohmann::json::parse_error& ex) {
    rethrow_scuc_exception("SCUC JSON parse error", ex);
  } catch (const nlohmann::json::type_error& ex) {
    rethrow_scuc_exception("SCUC JSON type error", ex);
  } catch (const nlohmann::json::out_of_range& ex) {
    rethrow_scuc_exception("SCUC JSON field error", ex);
  } catch (const nlohmann::json::exception& ex) {
    rethrow_scuc_exception("SCUC JSON error", ex);
  }
}

std::string scuc_output_to_json_checked(const mipsolvers::scuc::SCUCOutput& out,
                                        const mipsolvers::scuc::SCUCInput& inp,
                                        int indent) {
  try {
    return mipsolvers::scuc::scuc_output_to_json(out, inp, indent);
  } catch (const nlohmann::json::exception& ex) {
    rethrow_scuc_exception("SCUC JSON serialization error", ex);
  }
}

void require_vector_size(const Eigen::VectorXd& vec,
                         int expected,
                         const std::string& name) {
  if (vec.size() != expected) {
    throw std::invalid_argument(name + " length must equal c.size");
  }
}

void require_matrix_cols(const Eigen::SparseMatrix<double>& matrix,
                         int expected,
                         const std::string& name) {
  if (matrix.cols() != expected) {
    throw std::invalid_argument(name + ".shape[1] must equal c.size");
  }
}

/// 1-D numpy float64 array -> Eigen::VectorXd
Eigen::VectorXd vec_from_numpy(const py::array_t<double>& arr) {
  py::buffer_info info = arr.request();
  if (info.ndim != 1)
    throw std::invalid_argument("Expected 1-D float64 array");
  Eigen::VectorXd v(static_cast<Eigen::Index>(info.shape[0]));
  const double* ptr = static_cast<const double*>(info.ptr);
  for (Eigen::Index i = 0; i < v.size(); ++i) v[i] = ptr[i];
  return v;
}

/// Eigen::VectorXd -> 1-D numpy float64 array
py::array_t<double> vec_to_numpy(const Eigen::VectorXd& v) {
  py::array_t<double> arr(static_cast<py::ssize_t>(v.size()));
  double* ptr = arr.mutable_data();
  for (Eigen::Index i = 0; i < v.size(); ++i) ptr[i] = v[i];
  return arr;
}

/// Dense 2-D numpy float64 [rows, cols] -> Eigen::SparseMatrix<double> (row-major scan)
Eigen::SparseMatrix<double> sparse_from_dense(const py::array_t<double>& arr) {
  py::buffer_info info = arr.request();
  if (info.ndim != 2)
    throw std::invalid_argument("Expected 2-D float64 array for matrix A");
  const Eigen::Index rows = static_cast<Eigen::Index>(info.shape[0]);
  const Eigen::Index cols = static_cast<Eigen::Index>(info.shape[1]);
  const double* ptr = static_cast<const double*>(info.ptr);
  const Eigen::Index row_stride = static_cast<Eigen::Index>(info.strides[0] / sizeof(double));
  const Eigen::Index col_stride = static_cast<Eigen::Index>(info.strides[1] / sizeof(double));

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(rows * cols / 4));  // rough estimate
  for (Eigen::Index r = 0; r < rows; ++r)
    for (Eigen::Index c = 0; c < cols; ++c) {
      const double v = ptr[r * row_stride + c * col_stride];
      if (v != 0.0) triplets.emplace_back(r, c, v);
    }
  Eigen::SparseMatrix<double> sp(rows, cols);
  sp.setFromTriplets(triplets.begin(), triplets.end());
  return sp;
}

/// COO triplets (rows[], cols[], data[]) + shape -> Eigen::SparseMatrix<double>
/// (reserved for future scipy.sparse interop)
[[maybe_unused]]
Eigen::SparseMatrix<double> sparse_from_coo(
    const py::array_t<int>& row_arr,
    const py::array_t<int>& col_arr,
    const py::array_t<double>& data_arr,
    int n_rows, int n_cols) {
  py::buffer_info ri = row_arr.request();
  py::buffer_info ci = col_arr.request();
  py::buffer_info di = data_arr.request();
  if (ri.ndim != 1 || ci.ndim != 1 || di.ndim != 1)
    throw std::invalid_argument("COO arrays must be 1-D");
  const py::ssize_t nnz = ri.shape[0];
  if (ci.shape[0] != nnz || di.shape[0] != nnz)
    throw std::invalid_argument("COO arrays must have equal length");

  const int* rp = static_cast<const int*>(ri.ptr);
  const int* cp = static_cast<const int*>(ci.ptr);
  const double* dp = static_cast<const double*>(di.ptr);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(nnz));
  for (py::ssize_t k = 0; k < nnz; ++k)
    triplets.emplace_back(rp[k], cp[k], dp[k]);

  Eigen::SparseMatrix<double> sp(n_rows, n_cols);
  sp.setFromTriplets(triplets.begin(), triplets.end());
  return sp;
}

/// Build engine::LPModel from numpy arrays.
/// A, Aeq may be None (passed as py::none), or 2-D numpy arrays.
/// lb / ub may be None (defaults to -inf / +inf per variable).
mipsolvers::engine::LPModel build_lp_model(
    const py::array_t<double>& c,
    const py::object& A,
    const py::object& b,
    const py::object& Aeq,
    const py::object& beq,
    const py::object& lb,
    const py::object& ub,
    bool maximize) {
  using namespace mipsolvers::engine;
  LPModel lp;
  lp.sense = maximize ? Sense::Maximize : Sense::Minimize;
  lp.c = vec_from_numpy(c);
  const int n = static_cast<int>(lp.c.size());

  if (n <= 0) {
    throw std::invalid_argument("c must be a non-empty 1-D float64 array");
  }

  if (A.is_none() != b.is_none()) {
    throw std::invalid_argument("A and b must be provided together");
  }
  if (Aeq.is_none() != beq.is_none()) {
    throw std::invalid_argument("Aeq and beq must be provided together");
  }

  if (!A.is_none() && !b.is_none()) {
    lp.A   = sparse_from_dense(A.cast<py::array_t<double>>());
    lp.b   = vec_from_numpy(b.cast<py::array_t<double>>());
    require_matrix_cols(lp.A, n, "A");
    if (lp.b.size() != lp.A.rows()) {
      throw std::invalid_argument("b length must equal A.shape[0]");
    }
  }
  if (!Aeq.is_none() && !beq.is_none()) {
    lp.Aeq  = sparse_from_dense(Aeq.cast<py::array_t<double>>());
    lp.beq  = vec_from_numpy(beq.cast<py::array_t<double>>());
    require_matrix_cols(lp.Aeq, n, "Aeq");
    if (lp.beq.size() != lp.Aeq.rows()) {
      throw std::invalid_argument("beq length must equal Aeq.shape[0]");
    }
  }

  lp.vars.resize(static_cast<size_t>(n));
  if (!lb.is_none()) {
    auto lbv = vec_from_numpy(lb.cast<py::array_t<double>>());
    require_vector_size(lbv, n, "lb");
    for (int i = 0; i < n; ++i) lp.vars[static_cast<size_t>(i)].lb = lbv[i];
  }
  if (!ub.is_none()) {
    auto ubv = vec_from_numpy(ub.cast<py::array_t<double>>());
    require_vector_size(ubv, n, "ub");
    for (int i = 0; i < n; ++i) lp.vars[static_cast<size_t>(i)].ub = ubv[i];
  }
  return lp;
}

/// Build engine::MIPModel from numpy arrays.
/// vartypes: list of "C"/"I"/"B" strings (or single char), length n.
mipsolvers::engine::MIPModel build_mip_model(
    const py::array_t<double>& c,
    const py::object& A,
    const py::object& b,
    const py::object& Aeq,
    const py::object& beq,
    const py::object& lb,
    const py::object& ub,
    const py::list& vartypes,
    bool maximize) {
  using namespace mipsolvers::engine;
  MIPModel mip;
  mip.linear_part = build_lp_model(c, A, b, Aeq, beq, lb, ub, maximize);
  const int n = static_cast<int>(mip.linear_part.c.size());

  if (!vartypes.empty() && static_cast<int>(vartypes.size()) != n) {
    throw std::invalid_argument("vartypes length must equal c.size");
  }

  for (int i = 0; i < n; ++i) {
    const std::string vt = vartypes.empty()
                               ? "C"
                               : py::str(vartypes[i]).cast<std::string>();
    const char ch = vt.empty() ? 'C' : static_cast<char>(std::toupper(static_cast<unsigned char>(vt[0])));
    auto& meta = mip.linear_part.vars[static_cast<size_t>(i)];
    if (ch == 'B') {
      mip.binary_idx.push_back(i);
      meta.type = VarType::Binary;
      meta.lb = 0.0;
      meta.ub = 1.0;
    } else if (ch == 'I') {
      mip.integer_idx.push_back(i);
      meta.type = VarType::Integer;
    } else {
      meta.type = VarType::Continuous;
    }
  }
  return mip;
}

}  // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// Module definition
// ─────────────────────────────────────────────────────────────────────────────
PYBIND11_MODULE(mipsolvers, m) {
  m.doc() = R"doc(
MIPSolvers Python bindings.

Submodules
----------
scuc   : Short-term Unit Commitment (SCUC/SCED/LMP) market dispatch module.
engine : Low-level LP/MILP/NLP solver engine.
)doc";

  // ═══════════════════════════════════════════════════════════════════════════
  // SCUC submodule
  // ═══════════════════════════════════════════════════════════════════════════
  py::module_ m_scuc = m.def_submodule("scuc", R"doc(
Short-Term Unit Commitment (SCUC) market dispatch module.

Quick start (JSON API — recommended)
--------------------------------------
>>> import mipsolvers
>>> result_json = mipsolvers.scuc.solve_json(input_json_str)

Quick start (struct API)
------------------------
>>> inp = mipsolvers.scuc.SCUCInput()
>>> inp.config.num_periods = 24
>>> g = mipsolvers.scuc.Generator()
>>> g.name = "G1"; g.bus = 0; g.pmin = 100.0; g.pmax = 400.0
>>> g.bid_segments = [mipsolvers.scuc.BidSegment(20.0, 300.0)]
>>> inp.generators.append(g)
>>> out = mipsolvers.scuc.solve(inp)
>>> print(out.scuc.converged, out.scuc.total_cost)
)doc");

  // ── BidSegment ─────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::BidSegment>(m_scuc, "BidSegment", R"doc(
A single price–quantity segment of a generator's offer curve.

Attributes
----------
price    : float  Offer price ($/MWh). **First field.**
quantity : float  Incremental capacity above Pmin (MW). **Second field.**
)doc")
    .def(py::init<>())
    .def(py::init([](double price, double quantity) {
      mipsolvers::scuc::BidSegment s;
      s.price = price;
      s.quantity = quantity;
      return s;
    }), "price"_a, "quantity"_a)
    .def_readwrite("price",    &mipsolvers::scuc::BidSegment::price)
    .def_readwrite("quantity", &mipsolvers::scuc::BidSegment::quantity)
    .def("__repr__", [](const mipsolvers::scuc::BidSegment& s) {
      return "BidSegment(price=" + std::to_string(s.price) +
             ", quantity=" + std::to_string(s.quantity) + ")";
    });

  // ── Generator ──────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Generator>(m_scuc, "Generator", R"doc(
Thermal generator parameters.
)doc")
    .def(py::init<>())
    .def_readwrite("name",                   &mipsolvers::scuc::Generator::name)
    .def_readwrite("bus",                    &mipsolvers::scuc::Generator::bus)
    .def_readwrite("pmin",                   &mipsolvers::scuc::Generator::pmin)
    .def_readwrite("pmax",                   &mipsolvers::scuc::Generator::pmax)
    .def_readwrite("ramp_up_mw_min",         &mipsolvers::scuc::Generator::ramp_up_mw_min)
    .def_readwrite("ramp_dn_mw_min",         &mipsolvers::scuc::Generator::ramp_dn_mw_min)
    .def_readwrite("min_up_time_hr",         &mipsolvers::scuc::Generator::min_up_time_hr)
    .def_readwrite("min_dn_time_hr",         &mipsolvers::scuc::Generator::min_dn_time_hr)
    .def_readwrite("must_run",               &mipsolvers::scuc::Generator::must_run)
    .def_readwrite("max_startups",           &mipsolvers::scuc::Generator::max_startups)
    .def_readwrite("max_shutdowns",          &mipsolvers::scuc::Generator::max_shutdowns)
    .def_readwrite("startup_cost",           &mipsolvers::scuc::Generator::startup_cost)
    .def_readwrite("startup_cost_warm",      &mipsolvers::scuc::Generator::startup_cost_warm)
    .def_readwrite("startup_cost_cold",      &mipsolvers::scuc::Generator::startup_cost_cold)
    .def_readwrite("hot_start_threshold_hr", &mipsolvers::scuc::Generator::hot_start_threshold_hr)
    .def_readwrite("warm_start_threshold_hr",&mipsolvers::scuc::Generator::warm_start_threshold_hr)
    .def_readwrite("ud_periods",             &mipsolvers::scuc::Generator::ud_periods)
    .def_readwrite("dd_periods",             &mipsolvers::scuc::Generator::dd_periods)
    .def_readwrite("no_load_cost",           &mipsolvers::scuc::Generator::no_load_cost)
    .def_readwrite("spinning_reserve_price", &mipsolvers::scuc::Generator::spinning_reserve_price)
    .def_readwrite("regulation_up_price",    &mipsolvers::scuc::Generator::regulation_up_price)
    .def_readwrite("regulation_down_price",  &mipsolvers::scuc::Generator::regulation_down_price)
    .def_readwrite("pfr_alpha",              &mipsolvers::scuc::Generator::pfr_alpha)
    .def_readwrite("group_id",               &mipsolvers::scuc::Generator::group_id)
    .def_readwrite("bid_segments",           &mipsolvers::scuc::Generator::bid_segments);

  // ── Branch ─────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Branch>(m_scuc, "Branch", R"doc(
AC transmission branch (line or transformer).
)doc")
    .def(py::init<>())
    .def_readwrite("from_bus",   &mipsolvers::scuc::Branch::from)
    .def_readwrite("to_bus",     &mipsolvers::scuc::Branch::to)
    .def_readwrite("reactance",  &mipsolvers::scuc::Branch::reactance)
    .def_readwrite("rating_mw",  &mipsolvers::scuc::Branch::rating_mw)
    .def_readwrite("in_service", &mipsolvers::scuc::Branch::in_service);

  // ── Load ───────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Load>(m_scuc, "Load")
    .def(py::init<>())
    .def_readwrite("bus",  &mipsolvers::scuc::Load::bus)
    .def_readwrite("p_mw", &mipsolvers::scuc::Load::p_mw);

  // ── WindUnit / SolarUnit ───────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::WindUnit>(m_scuc, "WindUnit")
    .def(py::init<>())
    .def_readwrite("bus",  &mipsolvers::scuc::WindUnit::bus)
    .def_readwrite("pmax", &mipsolvers::scuc::WindUnit::pmax);

  py::class_<mipsolvers::scuc::SolarUnit>(m_scuc, "SolarUnit")
    .def(py::init<>())
    .def_readwrite("bus",  &mipsolvers::scuc::SolarUnit::bus)
    .def_readwrite("pmax", &mipsolvers::scuc::SolarUnit::pmax);

  // ── StorageUnit ────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::StorageUnit>(m_scuc, "StorageUnit", R"doc(
Battery / pumped-hydro storage unit.
)doc")
    .def(py::init<>())
    .def_readwrite("name",                  &mipsolvers::scuc::StorageUnit::name)
    .def_readwrite("bus",                   &mipsolvers::scuc::StorageUnit::bus)
    .def_readwrite("pmax_charge",           &mipsolvers::scuc::StorageUnit::pmax_charge)
    .def_readwrite("pmax_discharge",        &mipsolvers::scuc::StorageUnit::pmax_discharge)
    .def_readwrite("pmin_charge",           &mipsolvers::scuc::StorageUnit::pmin_charge)
    .def_readwrite("pmin_discharge",        &mipsolvers::scuc::StorageUnit::pmin_discharge)
    .def_readwrite("energy_capacity_mwh",   &mipsolvers::scuc::StorageUnit::energy_capacity_mwh)
    .def_readwrite("efficiency",            &mipsolvers::scuc::StorageUnit::efficiency)
    .def_readwrite("eta_charge",            &mipsolvers::scuc::StorageUnit::eta_charge)
    .def_readwrite("eta_discharge",         &mipsolvers::scuc::StorageUnit::eta_discharge)
    .def_readwrite("soc_init",              &mipsolvers::scuc::StorageUnit::soc_init)
    .def_readwrite("soc_min",               &mipsolvers::scuc::StorageUnit::soc_min)
    .def_readwrite("soc_final",             &mipsolvers::scuc::StorageUnit::soc_final)
    .def_readwrite("charge_bid_price",      &mipsolvers::scuc::StorageUnit::charge_bid_price)
    .def_readwrite("discharge_bid_price",   &mipsolvers::scuc::StorageUnit::discharge_bid_price)
    .def_readwrite("cycle_limit",           &mipsolvers::scuc::StorageUnit::cycle_limit)
    .def_readwrite("use_binary_indicators", &mipsolvers::scuc::StorageUnit::use_binary_indicators);

  // ── DCLine ─────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::DCLine>(m_scuc, "DCLine")
    .def(py::init<>())
    .def_readwrite("from_bus", &mipsolvers::scuc::DCLine::from)
    .def_readwrite("to_bus",   &mipsolvers::scuc::DCLine::to)
    .def_readwrite("pmin",     &mipsolvers::scuc::DCLine::pmin)
    .def_readwrite("pmax",     &mipsolvers::scuc::DCLine::pmax)
    .def_readwrite("ramp_up",  &mipsolvers::scuc::DCLine::ramp_up)
    .def_readwrite("ramp_dn",  &mipsolvers::scuc::DCLine::ramp_dn);

  // ── GeneratorGroup ─────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::GeneratorGroup>(m_scuc, "GeneratorGroup")
    .def(py::init<>())
    .def_readwrite("id",          &mipsolvers::scuc::GeneratorGroup::id)
    .def_readwrite("name",        &mipsolvers::scuc::GeneratorGroup::name)
    .def_readwrite("gen_indices", &mipsolvers::scuc::GeneratorGroup::gen_indices)
    .def_readwrite("pmin_t",      &mipsolvers::scuc::GeneratorGroup::pmin_t)
    .def_readwrite("pmax_t",      &mipsolvers::scuc::GeneratorGroup::pmax_t)
    .def_readwrite("emin",        &mipsolvers::scuc::GeneratorGroup::emin)
    .def_readwrite("emax",        &mipsolvers::scuc::GeneratorGroup::emax);

  // ── Section ────────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::Section>(m_scuc, "Section", R"doc(
Monitored network section (aggregate transmission corridor).
line_weights: list of (branch_index, weight) tuples.
)doc")
    .def(py::init<>())
    .def_readwrite("name",           &mipsolvers::scuc::Section::name)
    .def_readwrite("line_weights",   &mipsolvers::scuc::Section::line_weights)
    .def_readwrite("rating_fwd_mw",  &mipsolvers::scuc::Section::rating_fwd_mw)
    .def_readwrite("rating_rev_mw",  &mipsolvers::scuc::Section::rating_rev_mw);

  // ── SCUCConfig ─────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCConfig>(m_scuc, "SCUCConfig", R"doc(
Solver and model configuration.

Key attributes
--------------
solver          : str   "Auto" | "Gurobi" | "HiGHS" | "NativeBranchAndCut"
num_periods     : int   Number of dispatch periods T (default 24)
period_length_hr: float Hours per period (default 1.0)
n_segments      : int   Offer curve segments per generator (default 3)
mip_gap         : float Relative MIP optimality gap (default 0.001)
time_limit_sec  : float Solver time limit in seconds (default 300.0)
voll            : float Value of lost load $/MWh (default 10000.0)
solve_sced      : bool  Run SCED LP after SCUC (default True)
solve_lmp       : bool  Compute nodal LMP after SCED (default True)
)doc")
    .def(py::init<>())
    .def_readwrite("solver",                      &mipsolvers::scuc::SCUCConfig::solver)
    .def_readwrite("allow_fallback",              &mipsolvers::scuc::SCUCConfig::allow_fallback)
    .def_readwrite("num_periods",                 &mipsolvers::scuc::SCUCConfig::num_periods)
    .def_readwrite("period_length_hr",            &mipsolvers::scuc::SCUCConfig::period_length_hr)
    .def_readwrite("n_segments",                  &mipsolvers::scuc::SCUCConfig::n_segments)
    .def_readwrite("mip_gap",                     &mipsolvers::scuc::SCUCConfig::mip_gap)
    .def_readwrite("time_limit_sec",              &mipsolvers::scuc::SCUCConfig::time_limit_sec)
    .def_readwrite("verbose",                     &mipsolvers::scuc::SCUCConfig::verbose)
    .def_readwrite("spinning_reserve_req",        &mipsolvers::scuc::SCUCConfig::spinning_reserve_req)
    .def_readwrite("regulation_up_req",           &mipsolvers::scuc::SCUCConfig::regulation_up_req)
    .def_readwrite("regulation_down_req",         &mipsolvers::scuc::SCUCConfig::regulation_down_req)
    .def_readwrite("neg_reserve_req",             &mipsolvers::scuc::SCUCConfig::neg_reserve_req)
    .def_readwrite("pfr_reserve_req_mw",          &mipsolvers::scuc::SCUCConfig::pfr_reserve_req_mw)
    .def_readwrite("voll",                        &mipsolvers::scuc::SCUCConfig::voll)
    .def_readwrite("vocc",                        &mipsolvers::scuc::SCUCConfig::vocc)
    .def_readwrite("renewable_min_output_coeff",  &mipsolvers::scuc::SCUCConfig::renewable_min_output_coeff)
    .def_readwrite("M2_renewable_curtail_penalty",&mipsolvers::scuc::SCUCConfig::M2_renewable_curtail_penalty)
    .def_readwrite("M1_line_slack_penalty",       &mipsolvers::scuc::SCUCConfig::M1_line_slack_penalty)
    .def_readwrite("wheeling_fee_per_mwh",        &mipsolvers::scuc::SCUCConfig::wheeling_fee_per_mwh)
    .def_readwrite("enable_market_cuts",          &mipsolvers::scuc::SCUCConfig::enable_market_cuts)
    .def_readwrite("solve_sced",                  &mipsolvers::scuc::SCUCConfig::solve_sced)
    .def_readwrite("solve_lmp",                   &mipsolvers::scuc::SCUCConfig::solve_lmp)
    .def_readwrite("lmp_delta",                   &mipsolvers::scuc::SCUCConfig::lmp_delta);

  // ── SCUCProfiles ───────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCProfiles>(m_scuc, "SCUCProfiles", R"doc(
Time-series profiles.

Attributes
----------
load  : list[list[float]]  load[d][t] = per-unit multiplier for Load[d] at period t
wind  : list[list[float]]  wind[w][t] = forecast MW for WindUnit w at period t
solar : list[list[float]]  solar[s][t] = forecast MW for SolarUnit s at period t
)doc")
    .def(py::init<>())
    .def_readwrite("load",  &mipsolvers::scuc::SCUCProfiles::load)
    .def_readwrite("wind",  &mipsolvers::scuc::SCUCProfiles::wind)
    .def_readwrite("solar", &mipsolvers::scuc::SCUCProfiles::solar);

  // ── SCUCInitialStatus ──────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCInitialStatus>(m_scuc, "SCUCInitialStatus", R"doc(
Generator and storage state at the start of the planning horizon.
)doc")
    .def(py::init<>())
    .def_readwrite("commitment",    &mipsolvers::scuc::SCUCInitialStatus::commitment)
    .def_readwrite("dispatch",      &mipsolvers::scuc::SCUCInitialStatus::dispatch)
    .def_readwrite("storage_soc",   &mipsolvers::scuc::SCUCInitialStatus::storage_soc)
    .def_readwrite("time_in_state", &mipsolvers::scuc::SCUCInitialStatus::time_in_state);

  // ── SCUCInput ──────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCInput>(m_scuc, "SCUCInput", R"doc(
Full SCUC problem input.  All fields have sensible defaults; populate only
what you need.

Example
-------
>>> inp = mipsolvers.scuc.SCUCInput()
>>> inp.config.num_periods = 24
>>> inp.config.solver = "HiGHS"
)doc")
    .def(py::init<>())
    .def_readwrite("config",            &mipsolvers::scuc::SCUCInput::config)
    .def_readwrite("num_buses",         &mipsolvers::scuc::SCUCInput::num_buses)
    .def_readwrite("generators",        &mipsolvers::scuc::SCUCInput::generators)
    .def_readwrite("branches",          &mipsolvers::scuc::SCUCInput::branches)
    .def_readwrite("loads",             &mipsolvers::scuc::SCUCInput::loads)
    .def_readwrite("wind",              &mipsolvers::scuc::SCUCInput::wind)
    .def_readwrite("solar",             &mipsolvers::scuc::SCUCInput::solar)
    .def_readwrite("storage",           &mipsolvers::scuc::SCUCInput::storage)
    .def_readwrite("dc_lines",          &mipsolvers::scuc::SCUCInput::dc_lines)
    .def_readwrite("generator_groups",  &mipsolvers::scuc::SCUCInput::generator_groups)
    .def_readwrite("sections",          &mipsolvers::scuc::SCUCInput::sections)
    .def_readwrite("profiles",          &mipsolvers::scuc::SCUCInput::profiles)
    .def_readwrite("initial_status",    &mipsolvers::scuc::SCUCInput::initial_status);

  // ── SCUCSolveResult ────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCSolveResult>(m_scuc, "SCUCSolveResult", R"doc(
SCUC or SCED solve result.

Key attributes
--------------
converged       : bool
objective       : float  Optimal objective value ($)
solver_name     : str
solve_time_sec  : float
mip_gap         : float
commitment      : list[list[float]]  [ng][T_commit]
dispatch        : list[list[float]]  [ng][T]
line_flows      : list[list[float]]  [nl][T]
nodal_lmp       : (in SCUCLMPResult) [nb][T]
load_shedding   : list[float]        [T]
total_cost      : float
energy_cost     : float
startup_cost    : float
no_load_cost    : float
reserve_cost    : float
penalty_cost    : float
)doc")
    .def(py::init<>())
    .def_readonly("converged",         &mipsolvers::scuc::SCUCSolveResult::converged)
    .def_readonly("objective",         &mipsolvers::scuc::SCUCSolveResult::objective)
    .def_readonly("solver_name",       &mipsolvers::scuc::SCUCSolveResult::solver_name)
    .def_readonly("solve_time_sec",    &mipsolvers::scuc::SCUCSolveResult::solve_time_sec)
    .def_readonly("mip_gap",           &mipsolvers::scuc::SCUCSolveResult::mip_gap)
    .def_readonly("n_cuts_added",      &mipsolvers::scuc::SCUCSolveResult::n_cuts_added)
    .def_readonly("commitment",        &mipsolvers::scuc::SCUCSolveResult::commitment)
    .def_readonly("startup",           &mipsolvers::scuc::SCUCSolveResult::startup)
    .def_readonly("shutdown",          &mipsolvers::scuc::SCUCSolveResult::shutdown)
    .def_readonly("dispatch",          &mipsolvers::scuc::SCUCSolveResult::dispatch)
    .def_readonly("spinning_reserve",  &mipsolvers::scuc::SCUCSolveResult::spinning_reserve)
    .def_readonly("regulation_up",     &mipsolvers::scuc::SCUCSolveResult::regulation_up)
    .def_readonly("regulation_down",   &mipsolvers::scuc::SCUCSolveResult::regulation_down)
    .def_readonly("segment_dispatch",  &mipsolvers::scuc::SCUCSolveResult::segment_dispatch)
    .def_readonly("wind_generation",   &mipsolvers::scuc::SCUCSolveResult::wind_generation)
    .def_readonly("solar_generation",  &mipsolvers::scuc::SCUCSolveResult::solar_generation)
    .def_readonly("wind_curtailment",  &mipsolvers::scuc::SCUCSolveResult::wind_curtailment)
    .def_readonly("solar_curtailment", &mipsolvers::scuc::SCUCSolveResult::solar_curtailment)
    .def_readonly("storage_charging",     &mipsolvers::scuc::SCUCSolveResult::storage_charging)
    .def_readonly("storage_discharging",  &mipsolvers::scuc::SCUCSolveResult::storage_discharging)
    .def_readonly("storage_soc",          &mipsolvers::scuc::SCUCSolveResult::storage_soc)
    .def_readonly("line_flows",           &mipsolvers::scuc::SCUCSolveResult::line_flows)
    .def_readonly("section_flows",        &mipsolvers::scuc::SCUCSolveResult::section_flows)
    .def_readonly("load_shedding",        &mipsolvers::scuc::SCUCSolveResult::load_shedding)
    .def_readonly("gen_curtailment",      &mipsolvers::scuc::SCUCSolveResult::gen_curtailment)
    .def_readonly("energy_cost",          &mipsolvers::scuc::SCUCSolveResult::energy_cost)
    .def_readonly("startup_cost",         &mipsolvers::scuc::SCUCSolveResult::startup_cost)
    .def_readonly("no_load_cost",         &mipsolvers::scuc::SCUCSolveResult::no_load_cost)
    .def_readonly("reserve_cost",         &mipsolvers::scuc::SCUCSolveResult::reserve_cost)
    .def_readonly("penalty_cost",         &mipsolvers::scuc::SCUCSolveResult::penalty_cost)
    .def_readonly("total_cost",           &mipsolvers::scuc::SCUCSolveResult::total_cost);

  // ── SCUCLMPResult ──────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCLMPResult>(m_scuc, "SCUCLMPResult", R"doc(
Locational Marginal Price (LMP) result.

Attributes
----------
converged       : bool
solve_time_sec  : float
nodal_lmp       : list[list[float]]  [nb][T]  $/MWh
energy_lmp      : list[list[float]]  [nb][T]
congestion_lmp  : list[list[float]]  [nb][T]
avg_lmp         : float
max_lmp         : float
min_lmp         : float
)doc")
    .def(py::init<>())
    .def_readonly("converged",       &mipsolvers::scuc::SCUCLMPResult::converged)
    .def_readonly("solve_time_sec",  &mipsolvers::scuc::SCUCLMPResult::solve_time_sec)
    .def_readonly("nodal_lmp",       &mipsolvers::scuc::SCUCLMPResult::nodal_lmp)
    .def_readonly("energy_lmp",      &mipsolvers::scuc::SCUCLMPResult::energy_lmp)
    .def_readonly("congestion_lmp",  &mipsolvers::scuc::SCUCLMPResult::congestion_lmp)
    .def_readonly("avg_lmp",         &mipsolvers::scuc::SCUCLMPResult::avg_lmp)
    .def_readonly("max_lmp",         &mipsolvers::scuc::SCUCLMPResult::max_lmp)
    .def_readonly("min_lmp",         &mipsolvers::scuc::SCUCLMPResult::min_lmp);

  // ── SCUCOutput ─────────────────────────────────────────────────────────────
  py::class_<mipsolvers::scuc::SCUCOutput>(m_scuc, "SCUCOutput", R"doc(
Aggregated output of the full SCUC pipeline (SCUC → SCED → LMP).

Attributes
----------
scuc : SCUCSolveResult   Day-ahead unit commitment (MILP)
sced : SCUCSolveResult   Economic dispatch LP (only if config.solve_sced=True)
lmp  : SCUCLMPResult     Nodal prices (only if config.solve_lmp=True)
)doc")
    .def(py::init<>())
    .def_readonly("scuc", &mipsolvers::scuc::SCUCOutput::scuc)
    .def_readonly("sced", &mipsolvers::scuc::SCUCOutput::sced)
    .def_readonly("lmp",  &mipsolvers::scuc::SCUCOutput::lmp);

  // ── Free functions ─────────────────────────────────────────────────────────
  m_scuc.def("from_json", &scuc_from_json_checked,
    py::arg("json_str"),
    R"doc(
Deserialize an SCUCInput from a JSON string.

Parameters
----------
json_str : str  JSON representation of the input (see data_format_spec.md).

Returns
-------
SCUCInput

Raises
------
RuntimeError  If the JSON is malformed or a field type is wrong.
)doc");

  m_scuc.def("solve",
    [](const mipsolvers::scuc::SCUCInput& inp) {
      try {
        return mipsolvers::scuc::scuc_solve(inp);
      } catch (const std::invalid_argument&) {
        throw;
      } catch (const std::exception& ex) {
        rethrow_scuc_exception("SCUC solve failed", ex);
      }
    },
    py::arg("input"),
    py::call_guard<py::gil_scoped_release>(),
    R"doc(
Run the full SCUC pipeline (SCUC MILP → SCED LP → LMP).

Parameters
----------
input : SCUCInput

Returns
-------
SCUCOutput

Notes
-----
The GIL is released during the solve; other Python threads can run.
)doc");

  m_scuc.def("output_to_json",
    [](const mipsolvers::scuc::SCUCOutput& out,
       const mipsolvers::scuc::SCUCInput& inp,
       int indent) {
      return scuc_output_to_json_checked(out, inp, indent);
    },
    py::arg("output"),
    py::arg("input"),
    py::arg("indent") = 2,
    R"doc(
Serialize an SCUCOutput to a JSON string.

Parameters
----------
output : SCUCOutput
input  : SCUCInput   (used for metadata echo)
indent : int         JSON indentation (default 2; use -1 for compact)

Returns
-------
str  JSON string
)doc");

  // ── Convenience: JSON-in → JSON-out ────────────────────────────────────────
  m_scuc.def("solve_json",
    [](const std::string& json_str, int indent) -> std::string {
      const auto inp = scuc_from_json_checked(json_str);
      mipsolvers::scuc::SCUCOutput out;
      {
        py::gil_scoped_release rel;
        try {
          out = mipsolvers::scuc::scuc_solve(inp);
        } catch (const std::invalid_argument&) {
          throw;
        } catch (const std::exception& ex) {
          rethrow_scuc_exception("SCUC solve failed", ex);
        }
      }
      return scuc_output_to_json_checked(out, inp, indent);
    },
    py::arg("json_str"),
    py::arg("indent") = 2,
    R"doc(
One-shot JSON API: parse input, run SCUC pipeline, return JSON result.

This is the simplest way to call SCUC from Python.

Parameters
----------
json_str : str  Input JSON string (see data_format_spec.md for schema).
indent   : int  Output JSON indentation (default 2; use -1 for compact).

Returns
-------
str  JSON result string containing "scuc", "sced", "lmp", "meta" keys.

Example
-------
>>> import json, mipsolvers
>>> inp = {"config": {"num_periods": 3}, "generators": [...], "loads": [...]}
>>> result = json.loads(mipsolvers.scuc.solve_json(json.dumps(inp)))
>>> print(result["scuc"]["converged"], result["scuc"]["cost"]["total"])
)doc");

  // ═══════════════════════════════════════════════════════════════════════════
  // Engine submodule
  // ═══════════════════════════════════════════════════════════════════════════
  py::module_ m_eng = m.def_submodule("engine", R"doc(
Low-level LP and MILP solver engine.

Quick start
-----------
>>> import numpy as np, mipsolvers
>>> # Minimize  c'x  subject to  A*x <= b,  lb <= x <= ub
>>> res = mipsolvers.engine.solve_lp(c, A, b, lb=lb, ub=ub, solver="Auto")
>>> print(res["success"], res["objective"], res["x"])

>>> # MILP: same as LP but pass vartypes list
>>> res = mipsolvers.engine.solve_milp(c, A, b, lb, ub, vartypes=["C","I","B",...])
)doc");

  // ── solve_lp ───────────────────────────────────────────────────────────────
  m_eng.def("solve_lp",
    [](const py::array_t<double>& c,
       const py::object& A,
       const py::object& b,
       const py::object& Aeq,
       const py::object& beq,
       const py::object& lb,
       const py::object& ub,
       const std::string& solver,
       bool maximize) -> py::dict {

      auto lp = build_lp_model(c, A, b, Aeq, beq, lb, ub, maximize);
      mipsolvers::engine::SolverEngine eng;
      mipsolvers::engine::SolveOptions opts;
      opts.preferred_solver = solver;
      opts.allow_fallback = true;

      py::gil_scoped_release rel;
      auto res = eng.solve_lp(lp, opts);
      py::gil_scoped_acquire acq;

      py::dict d;
      d["success"]    = res.stats.success;
      d["objective"]  = res.stats.objective;
      d["status"]     = res.stats.status;
      d["solver"]     = res.stats.solver_name;
      d["iterations"] = res.stats.iterations;
      d["runtime_sec"]= res.stats.runtime_sec;
      d["x"]          = vec_to_numpy(res.x);
      if (res.constraint_duals.size() > 0)
        d["duals"]    = vec_to_numpy(res.constraint_duals);
      return d;
    },
    py::arg("c"),
    py::arg("A")    = py::none(),
    py::arg("b")    = py::none(),
    py::arg("Aeq")  = py::none(),
    py::arg("beq")  = py::none(),
    py::arg("lb")   = py::none(),
    py::arg("ub")   = py::none(),
    py::arg("solver")   = "Auto",
    py::arg("maximize") = false,
    R"doc(
Solve a Linear Program.

  minimize    c @ x
  subject to  A  @ x <= b       (inequality, optional)
              Aeq @ x  = beq    (equality, optional)
              lb <= x <= ub     (box bounds, optional)

Parameters
----------
c        : np.ndarray  (n,)   Objective vector
A        : np.ndarray  (m,n)  Inequality constraint matrix (optional)
b        : np.ndarray  (m,)   Inequality RHS (optional)
Aeq      : np.ndarray  (p,n)  Equality constraint matrix (optional)
beq      : np.ndarray  (p,)   Equality RHS (optional)
lb       : np.ndarray  (n,)   Variable lower bounds (default -inf)
ub       : np.ndarray  (n,)   Variable upper bounds (default +inf)
solver   : str                "Auto" | "Gurobi" | "HiGHS" | "NativeIPMLP" |
                               "NativePDLP" | "NativeLCQP"
maximize : bool               If True, maximize c@x (default False)

Returns
-------
dict with keys:
  success    : bool
  objective  : float
  x          : np.ndarray  (n,)  Optimal solution
  duals      : np.ndarray  Constraint duals (if available)
  status     : str
  solver     : str
  iterations : int
  runtime_sec: float
)doc");

  // ── solve_milp ─────────────────────────────────────────────────────────────
  m_eng.def("solve_milp",
    [](const py::array_t<double>& c,
       const py::object& A,
       const py::object& b,
       const py::object& Aeq,
       const py::object& beq,
       const py::object& lb,
       const py::object& ub,
       const py::list& vartypes,
       const std::string& solver,
       double /*mip_gap*/,
       double /*time_limit_sec*/,
       bool maximize) -> py::dict {

      auto mip = build_mip_model(c, A, b, Aeq, beq, lb, ub, vartypes, maximize);
      mipsolvers::engine::SolverEngine eng;
      mipsolvers::engine::SolveOptions opts;
      opts.preferred_solver = solver;
      opts.allow_fallback = true;

      py::gil_scoped_release rel;
      auto res = eng.solve_milp(mip, opts);
      py::gil_scoped_acquire acq;

      py::dict d;
      d["success"]    = res.stats.success;
      d["objective"]  = res.stats.objective;
      d["status"]     = res.stats.status;
      d["solver"]     = res.stats.solver_name;
      d["mip_gap"]    = res.stats.mip_gap;
      d["runtime_sec"]= res.stats.runtime_sec;
      d["x"]          = vec_to_numpy(res.x);
      return d;
    },
    py::arg("c"),
    py::arg("A")    = py::none(),
    py::arg("b")    = py::none(),
    py::arg("Aeq")  = py::none(),
    py::arg("beq")  = py::none(),
    py::arg("lb")   = py::none(),
    py::arg("ub")   = py::none(),
    py::arg("vartypes")      = py::list(),
    py::arg("solver")        = "Auto",
    py::arg("mip_gap")       = 1e-4,
    py::arg("time_limit_sec")= 300.0,
    py::arg("maximize")      = false,
    R"doc(
Solve a Mixed-Integer Linear Program.

  minimize    c @ x
  subject to  A  @ x <= b       (inequality, optional)
              Aeq @ x  = beq    (equality, optional)
              lb <= x <= ub
              x[i] ∈ {0,1}  for i in binary_idx
              x[i] ∈ ℤ       for i in integer_idx

Parameters
----------
c        : np.ndarray  (n,)
A, b, Aeq, beq, lb, ub : same as solve_lp
vartypes : list[str]   Per-variable type: "C" continuous, "I" integer, "B" binary.
                       Default: all continuous. Length must equal n if provided.
solver   : str         "Auto" | "Gurobi" | "HiGHS" | "NativeBranchAndCut"
mip_gap       : float  Relative MIP gap tolerance (default 1e-4)
time_limit_sec: float  Time limit (default 300.0)
maximize      : bool

Returns
-------
dict with keys:
  success    : bool
  objective  : float
  x          : np.ndarray  (n,)
  mip_gap    : float
  status     : str
  solver     : str
  runtime_sec: float
)doc");

  // ── list_solvers ───────────────────────────────────────────────────────────
  m_eng.def("list_solvers",
    [](const std::string& problem_class) -> std::vector<std::string> {
      mipsolvers::engine::SolverEngine eng;
      using PC = mipsolvers::engine::ProblemClass;
      PC cls = PC::MILP;
      if (problem_class == "LP")    cls = PC::LP;
      else if (problem_class == "MILP")  cls = PC::MILP;
      else if (problem_class == "QP")    cls = PC::QP;
      else if (problem_class == "NLP")   cls = PC::NLP;
      else if (problem_class == "MINLP") cls = PC::MINLP;
      else if (problem_class == "LE")    cls = PC::LE;
      else if (problem_class == "NLE")   cls = PC::NLE;
      return eng.list_solvers(cls);
    },
    py::arg("problem_class") = "MILP",
    R"doc(
List registered solver adapters for a given problem class.

Parameters
----------
problem_class : str  "LP" | "MILP" | "QP" | "NLP" | "MINLP" | "LE" | "NLE"

Returns
-------
list[str]  Available solver names (in priority order for "Auto")
)doc");
}
