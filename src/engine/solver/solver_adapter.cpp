#include "hacdcpf/engine/solver/solver_adapter.hpp"

namespace hacdcpf::engine {
namespace {

const char* class_name(ProblemClass cls) {
  switch (cls) {
    case ProblemClass::LE:
      return "LE";
    case ProblemClass::NLE:
      return "NLE";
    case ProblemClass::LP:
      return "LP";
    case ProblemClass::QP:
      return "QP";
    case ProblemClass::NLP:
      return "NLP";
    case ProblemClass::MILP:
      return "MILP";
    case ProblemClass::MINLP:
      return "MINLP";
  }
  return "Unknown";
}

}  // namespace

SolveResult SolverAdapter::unsupported_result(ProblemClass cls) const {
  SolveResult out;
  out.stats.success = false;
  out.stats.status = std::string("Unsupported problem class for adapter ") + name() +
                     ": " + class_name(cls);
  out.stats.solver_name = name();
  return out;
}

SolveResult SolverAdapter::solve_le(const SparseLinSys&) const {
  return unsupported_result(ProblemClass::LE);
}

SolveResult SolverAdapter::solve_nle(const NonlinearSystem&) const {
  return unsupported_result(ProblemClass::NLE);
}

SolveResult SolverAdapter::solve_lp(const LPModel&) const {
  return unsupported_result(ProblemClass::LP);
}

SolveResult SolverAdapter::solve_qp(const QPModel&) const {
  return unsupported_result(ProblemClass::QP);
}

SolveResult SolverAdapter::solve_nlp(const NLPModel&) const {
  return unsupported_result(ProblemClass::NLP);
}

SolveResult SolverAdapter::solve_milp(const MIPModel&) const {
  return unsupported_result(ProblemClass::MILP);
}

SolveResult SolverAdapter::solve_minlp(const MINLPModel&) const {
  return unsupported_result(ProblemClass::MINLP);
}

}  // namespace hacdcpf::engine
