/// @file bc_status.hpp
/// @brief Shared status strings for the legacy branch-and-cut core.

#pragma once

namespace mipsolvers::engine::detail::bc_status {

inline constexpr char kNotStarted[] = "Not started";
inline constexpr char kEmptyMilpModel[] = "Empty MILP model";
inline constexpr char kEmptyMinlpModel[] = "Empty MINLP model";
inline constexpr char kInvalidIntegralityIndex[] = "Invalid MILP integrality index";
inline constexpr char kConflictingIntegralityDeclaration[] =
    "Conflicting MILP integrality declaration";
inline constexpr char kUnsupportedFreeIntegerVariable[] =
    "Unsupported free integer variable in native B&C";
inline constexpr char kInfeasiblePapiloPresolve[] = "Infeasible (PaPILO presolve)";
inline constexpr char kInfeasibleHighsPresolve[] = "Infeasible (HiGHS presolve)";
inline constexpr char kOptimalHighsPresolve[] = "Optimal (HiGHS presolve)";
inline constexpr char kInfeasibleNativePresolve[] = "Infeasible (native presolve)";
inline constexpr char kInvalidReducedIncumbent[] = "Invalid reduced-space incumbent";
inline constexpr char kInvalidPapiloPostsolveIncumbent[] = "Invalid incumbent after PaPILO postsolve";
inline constexpr char kInvalidHighsPostsolveIncumbent[] = "Invalid incumbent after HiGHS postsolve";
inline constexpr char kInfeasibleVariableBounds[] = "Infeasible variable bounds";
inline constexpr char kHighsPresolvedWorkingLpUnavailable[] =
    "HiGHS presolved working LP unavailable";
inline constexpr char kHighsPresolvedWorkingLpMismatch[] =
    "HiGHS presolved working LP mismatch";
inline constexpr char kRootRelaxationFailed[] = "Root relaxation failed";
inline constexpr char kRootRelaxationNanObjective[] = "Root relaxation returned NaN objective";
inline constexpr char kRootNlpRelaxationFailed[] = "Root NLP relaxation failed";
inline constexpr char kOptimalRootGapClosed[] = "Optimal (root gap closed)";
inline constexpr char kRootSeparationOnly[] = "Stopped after root separation";
inline constexpr char kTimeLimitReached[] = "Time limit reached";
inline constexpr char kNodeLimitReached[] = "Node limit reached";
inline constexpr char kSearchQueueExhausted[] = "Search queue exhausted";
inline constexpr char kOptimalityGapReached[] = "Optimality gap reached";
inline constexpr char kGapStagnation[] = "Gap stagnation";
inline constexpr char kOptimalTreeExhausted[] = "Optimal (tree exhausted)";
inline constexpr char kFeasibleIncumbentFound[] = "Feasible incumbent found";
inline constexpr char kNoFeasibleIntegerSolutionFound[] = "No feasible integer solution found";

}  // namespace mipsolvers::engine::detail::bc_status
