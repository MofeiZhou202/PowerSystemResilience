#include "mipsolvers/engine/util/problem_validation.hpp"

#include <algorithm>
#include <cmath>

namespace mipsolvers::engine {
namespace {

constexpr double kBoundEps = 1e-12;

void add_error(ValidationReport& r, const std::string& msg) {
  r.valid = false;
  r.errors.push_back(msg);
}

void validate_variable_meta(const std::vector<VariableMeta>& vars, ValidationReport& out) {
  for (size_t i = 0; i < vars.size(); ++i) {
    if (!std::isfinite(vars[i].lb) || !std::isfinite(vars[i].ub)) {
      add_error(out, "Variable bounds must be finite at index " + std::to_string(i));
      continue;
    }
    if (vars[i].lb > vars[i].ub + kBoundEps) {
      add_error(out, "Variable lower bound exceeds upper bound at index " +
                         std::to_string(i));
    }
    if (vars[i].type == VarType::Binary &&
        (vars[i].lb < -kBoundEps || vars[i].ub > 1.0 + kBoundEps)) {
      out.warnings.push_back("Binary variable bounds should typically be within [0,1] at "
                             "index " +
                             std::to_string(i));
    }
  }
}

void validate_index_set(const std::vector<int>& idx,
                        int nvars,
                        const std::string& label,
                        ValidationReport& out) {
  std::vector<int> sorted = idx;
  std::sort(sorted.begin(), sorted.end());
  for (size_t i = 0; i < sorted.size(); ++i) {
    if (sorted[i] < 0 || sorted[i] >= nvars) {
      add_error(out, label + " index out of range: " + std::to_string(sorted[i]));
    }
    if (i > 0 && sorted[i] == sorted[i - 1]) {
      add_error(out, label + " has duplicate index: " + std::to_string(sorted[i]));
    }
  }
}

bool validate_symbolic_expr(const std::shared_ptr<SymExpr>& e,
                           int nvars,
                           const std::string& label,
                           ValidationReport& out) {
  if (!e) {
    add_error(out, label + " is null");
    return false;
  }

  switch (e->op) {
    case SymOp::Constant:
      return true;
    case SymOp::Variable:
      if (e->var_index < 0 || e->var_index >= nvars) {
        add_error(out, label + " has variable index out of range: " +
                           std::to_string(e->var_index));
        return false;
      }
      return true;
    case SymOp::Add:
    case SymOp::Sub:
    case SymOp::Mul:
      return validate_symbolic_expr(e->lhs, nvars, label + ".lhs", out) &&
             validate_symbolic_expr(e->rhs, nvars, label + ".rhs", out);
    case SymOp::Neg:
    case SymOp::Pow2:
    case SymOp::PowN:
      return validate_symbolic_expr(e->lhs, nvars, label + ".arg", out);
  }
  return false;
}

void validate_symbolic_constraints(const std::vector<SymbolicConstraint>& cons,
                                   int nvars,
                                   ValidationReport& out) {
  for (size_t i = 0; i < cons.size(); ++i) {
    validate_symbolic_expr(cons[i].expr, nvars, "symbolic_constraints[" + std::to_string(i) + "]", out);
  }
}

}  // namespace

ValidationReport validate(const SparseLinSys& model) {
  ValidationReport out;
  if (model.A.rows() <= 0 || model.A.cols() <= 0) {
    add_error(out, "A must be non-empty");
    return out;
  }
  if (model.A.rows() != model.b.size()) {
    add_error(out, "Dimension mismatch: A.rows must equal b.size");
  }
  return out;
}

ValidationReport validate(const NonlinearSystem& model) {
  ValidationReport out;
  if (model.n <= 0) {
    add_error(out, "n must be positive");
  }
  if (!model.residual) {
    add_error(out, "Residual callback is required");
  }
  if (!model.jacobian) {
    add_error(out, "Jacobian callback is required");
  }
  if (model.x0.size() != model.n) {
    add_error(out, "x0 size must equal n");
  }
  return out;
}

ValidationReport validate(const LPModel& model) {
  ValidationReport out;
  const int n = static_cast<int>(model.c.size());
  if (n <= 0) {
    add_error(out, "Objective vector c must be non-empty");
    return out;
  }

  if (model.A.rows() > 0 && model.A.cols() != n) {
    add_error(out, "A.cols must equal c.size");
  }
  if (model.A.rows() != model.b.size()) {
    add_error(out, "A.rows must equal b.size");
  }
  if (model.row_lhs.size() != 0 && model.row_lhs.size() != model.A.rows()) {
    add_error(out, "row_lhs must be empty or have A.rows entries");
  }
  if (lp_has_row_lhs(model)) {
    for (int i = 0; i < model.A.rows(); ++i) {
      if (std::isfinite(model.row_lhs[i]) && std::isfinite(model.b[i]) &&
          model.row_lhs[i] > model.b[i] + kBoundEps) {
        add_error(out, "row_lhs exceeds row upper bound at row " +
                           std::to_string(i));
      }
    }
  }

  if (model.Aeq.rows() > 0 && model.Aeq.cols() != n) {
    add_error(out, "Aeq.cols must equal c.size");
  }
  if (model.Aeq.rows() != model.beq.size()) {
    add_error(out, "Aeq.rows must equal beq.size");
  }

  if (static_cast<int>(model.vars.size()) != n) {
    add_error(out, "vars size must equal c.size");
  } else {
    validate_variable_meta(model.vars, out);
  }

  return out;
}

ValidationReport validate(const QPModel& model) {
  ValidationReport out;
  const int n = static_cast<int>(model.c.size());
  if (n <= 0) {
    add_error(out, "Objective vector c must be non-empty");
    return out;
  }

  const bool has_empty_hessian = model.Q.rows() == 0 && model.Q.cols() == 0;
  if (!has_empty_hessian && (model.Q.rows() != n || model.Q.cols() != n)) {
    add_error(out, "Q must be empty or have shape c.size by c.size");
  }

  if (model.A.rows() > 0 && model.A.cols() != n) {
    add_error(out, "A.cols must equal c.size");
  }
  if (model.A.rows() != model.b.size()) {
    add_error(out, "A.rows must equal b.size");
  }

  if (model.Aeq.rows() > 0 && model.Aeq.cols() != n) {
    add_error(out, "Aeq.cols must equal c.size");
  }
  if (model.Aeq.rows() != model.beq.size()) {
    add_error(out, "Aeq.rows must equal beq.size");
  }

  if (static_cast<int>(model.vars.size()) != n) {
    add_error(out, "vars size must equal c.size");
  } else {
    validate_variable_meta(model.vars, out);
  }

  return out;
}

ValidationReport validate(const NLPModel& model) {
  ValidationReport out;
  const int n = static_cast<int>(model.vars.size());
  if (n <= 0) {
    add_error(out, "vars must be non-empty");
    return out;
  }

  validate_variable_meta(model.vars, out);

  const bool has_symbolic_objective = static_cast<bool>(model.symbolic_objective);
  const bool has_callback_objective = static_cast<bool>(model.f) && static_cast<bool>(model.grad);

  if (!has_symbolic_objective && !has_callback_objective) {
    add_error(out, "Either symbolic_objective or (f and grad) callbacks are required");
  }

  if (has_symbolic_objective) {
    validate_symbolic_expr(model.symbolic_objective, n, "symbolic_objective", out);
  }

  if (!has_callback_objective) {
    out.warnings.push_back("Objective callbacks f/grad are not set; symbolic objective assumed");
  }

  if (!model.hess) {
    out.warnings.push_back("Hessian callback hess is not set");
  }

  const bool has_symbolic_constraints = !model.symbolic_constraints.empty();
  if (has_symbolic_constraints) {
    validate_symbolic_constraints(model.symbolic_constraints, n, out);
  } else {
    if (!model.g) {
      out.warnings.push_back("Equality callback g is not set");
    }
    if (model.g && !model.jac_g) {
      add_error(out, "jac_g is required when g is set");
    }
    if (!model.h) {
      out.warnings.push_back("Inequality callback h is not set");
    }
    if (model.h && !model.jac_h) {
      add_error(out, "jac_h is required when h is set");
    }
  }

  if (model.x0.size() != n) {
    add_error(out, "x0 size must equal number of variables");
  }

  return out;
}

ValidationReport validate(const MIPModel& model) {
  ValidationReport out = validate(model.linear_part);
  const int n = static_cast<int>(model.linear_part.c.size());

  validate_index_set(model.integer_idx, n, "integer_idx", out);
  validate_index_set(model.binary_idx, n, "binary_idx", out);

  std::vector<int> both = model.integer_idx;
  both.insert(both.end(), model.binary_idx.begin(), model.binary_idx.end());
  std::sort(both.begin(), both.end());
  for (size_t i = 1; i < both.size(); ++i) {
    if (both[i] == both[i - 1]) {
      add_error(out, "Index appears in both integer_idx and binary_idx: " +
                         std::to_string(both[i]));
    }
  }

  return out;
}

ValidationReport validate(const MINLPModel& model) {
  ValidationReport out = validate(model.nonlinear_part);
  const int n = static_cast<int>(model.nonlinear_part.vars.size());

  validate_index_set(model.integer_idx, n, "integer_idx", out);
  validate_index_set(model.binary_idx, n, "binary_idx", out);

  std::vector<int> both = model.integer_idx;
  both.insert(both.end(), model.binary_idx.begin(), model.binary_idx.end());
  std::sort(both.begin(), both.end());
  for (size_t i = 1; i < both.size(); ++i) {
    if (both[i] == both[i - 1]) {
      add_error(out, "Index appears in both integer_idx and binary_idx: " +
                         std::to_string(both[i]));
    }
  }

  return out;
}

}  // namespace mipsolvers::engine
