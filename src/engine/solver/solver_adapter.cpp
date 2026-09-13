#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {
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
    case ProblemClass::CONIC:
      return "CONIC";
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

SolveResult SolverAdapter::solve_conic(const ConicModel&) const {
  return unsupported_result(ProblemClass::CONIC);
}

SolveResult SolverAdapter::solve_le(const SparseLinSys& prob,
                                    const SolveContext&) const {
  return solve_le(prob);
}

SolveResult SolverAdapter::solve_nle(const NonlinearSystem& prob,
                                     const SolveContext&) const {
  return solve_nle(prob);
}

SolveResult SolverAdapter::solve_lp(const LPModel& prob,
                                    const SolveContext&) const {
  return solve_lp(prob);
}

SolveResult SolverAdapter::solve_qp(const QPModel& prob,
                                    const SolveContext&) const {
  return solve_qp(prob);
}

SolveResult SolverAdapter::solve_nlp(const NLPModel& prob,
                                     const SolveContext&) const {
  return solve_nlp(prob);
}

SolveResult SolverAdapter::solve_milp(const MIPModel& prob,
                                      const SolveContext&) const {
  return solve_milp(prob);
}

SolveResult SolverAdapter::solve_minlp(const MINLPModel& prob,
                                       const SolveContext&) const {
  return solve_minlp(prob);
}

SolveResult SolverAdapter::solve_conic(const ConicModel& prob,
                                       const SolveContext&) const {
  return solve_conic(prob);
}

}  // namespace mipsolvers::engine
