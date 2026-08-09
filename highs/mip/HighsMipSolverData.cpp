/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "mip/HighsMipSolverData.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <cstdlib>
#include <vector>

#include "../extern/pdqsort/pdqsort.h"
#include "lp_data/HighsModelUtils.h"
#include "mip/HighsPseudocost.h"
#include "mip/HighsRedcostFixing.h"
#include "mip/MipTimer.h"
#include "parallel/HighsParallel.h"
#include "presolve/HPresolve.h"
#include "util/HighsIntegers.h"

namespace {

bool hacdcpfLocalCutAlreadyPresent(
    const std::vector<HacdcpfLocalCut>& cuts, std::uint64_t row_hash,
    std::uint64_t row_sig) {
  if (row_hash == 0 && row_sig == 0) return false;
  for (const HacdcpfLocalCut& cut : cuts) {
    if (cut.row_hash == row_hash && cut.row_sig == row_sig) return true;
  }
  return false;
}

bool hacdcpfCutSetAlreadyHasLocalRow(const HighsCutSet& cutset,
                                     std::uint64_t row_hash,
                                     std::uint64_t row_sig) {
  if (row_hash == 0 && row_sig == 0) return false;
  for (HighsInt row = 0; row != cutset.numCuts(); ++row) {
    if (hacdcpfCutSetRowHash(cutset, row) == row_hash &&
        hacdcpfCutSetRowSig(cutset, row) == row_sig) {
      return true;
    }
  }
  return false;
}

bool hacdcpfCutSetAlreadyHasGlobalRow(const HighsCutSet& cutset,
                                      std::uint64_t row_hash,
                                      std::uint64_t row_sig) {
  if (row_hash == 0 && row_sig == 0) return false;
  for (HighsInt row = 0; row != cutset.numCuts(); ++row) {
    if (hacdcpfCutSetRowHash(cutset, row) == row_hash &&
        hacdcpfCutSetRowSig(cutset, row) == row_sig) {
      return true;
    }
  }
  return false;
}

}  // namespace

static bool hacdcpfHighsConfEnabled() {
  const char* env = std::getenv("HACDCPF_HIGHS_CONF");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool hacdcpfHighsTimelineEnabled() {
  const char* env = std::getenv("HACDCPF_HIGHS_TIMELINE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool hacdcpfHighsFrontierConformEnabled() {
  const char* env = std::getenv("HACDCPF_HIGHS_FRONTIER_CONFORM");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static HighsInt hacdcpfHighsFrontierTraceTerms() {
  const char* env = std::getenv("HACDCPF_HIGHS_FRONTIER_CONFORM_TERMS");
  if (env == nullptr || env[0] == '\0')
    env = std::getenv("HACDCPF_FRONTIER_CONFORM_TERMS");
  if (env == nullptr || env[0] == '\0') return 32;
  char* end = nullptr;
  const long parsed = std::strtol(env, &end, 10);
  if (end == env || parsed <= 0) return 32;
  return std::min<HighsInt>(static_cast<HighsInt>(parsed), 256);
}

static bool hacdcpfHighsRootRoundTraceEnabled() {
  const char* env = std::getenv("HACDCPF_HIGHS_ROOT_ROUND");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

struct HacdcpfRootLedgerScopeGuard {
  HighsLpRelaxation& lp;
  HighsCutPool& cutpool;
  bool old_lp_scope;
  bool old_cutpool_scope;
  bool active;

  HacdcpfRootLedgerScopeGuard(HighsLpRelaxation& lp_, HighsCutPool& cutpool_,
                              bool active_)
      : lp(lp_),
        cutpool(cutpool_),
        old_lp_scope(lp_.getHacdcpfRootLedgerScope()),
        old_cutpool_scope(cutpool_.getHacdcpfRootLedgerScope()),
        active(active_) {
    if (!active) return;
    lp.setHacdcpfRootLedgerScope(true);
    cutpool.setHacdcpfRootLedgerScope(true);
  }

  ~HacdcpfRootLedgerScopeGuard() {
    if (!active) return;
    lp.setHacdcpfRootLedgerScope(old_lp_scope);
    cutpool.setHacdcpfRootLedgerScope(old_cutpool_scope);
  }
};

static const char* hacdcpfHighsVarTypeName(HighsVarType type,
                                           bool binary_like) {
  switch (type) {
    case HighsVarType::kContinuous:
      return "cont";
    case HighsVarType::kImplicitInteger:
      return "impl";
    case HighsVarType::kInteger:
      return binary_like ? "bin" : "int";
    case HighsVarType::kSemiContinuous:
      return "semicont";
    case HighsVarType::kSemiInteger:
      return "semiint";
  }
  return "var";
}

static char hacdcpfHighsBasisStatusChar(HighsBasisStatus status) {
  switch (status) {
    case HighsBasisStatus::kBasic:
      return 'B';
    case HighsBasisStatus::kLower:
      return 'L';
    case HighsBasisStatus::kUpper:
      return 'U';
    case HighsBasisStatus::kZero:
      return 'Z';
    case HighsBasisStatus::kNonbasic:
      return 'N';
  }
  return '?';
}

static std::uint64_t hacdcpfHighsTraceHashMix(std::uint64_t seed,
                                              std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

static std::uint64_t hacdcpfHighsTraceHashDouble(double value) {
  if (std::isinf(value))
    return value > 0.0 ? 0x7ff0000000000000ULL : 0xfff0000000000000ULL;
  if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
  return static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::llround(value * 1e9)));
}

static void hacdcpfLogHighsFrontierConformance(
    HighsMipSolverData& data, const HighsLpRelaxation& lprelax,
    const char* phase) {
  if (!hacdcpfHighsFrontierConformEnabled()) return;

  const HighsInt n = data.mipsolver.numCol();
  const HighsLp& lp = lprelax.getLp();
  const HighsSolution& sol = lprelax.getLpSolver().getSolution();
  const HighsBasis& basis = lprelax.getLpSolver().getBasis();
  if (static_cast<HighsInt>(sol.col_value.size()) < n ||
      lp.num_col_ < n) {
    highsLogUser(data.mipsolver.options_mip_->log_options,
                 HighsLogType::kInfo,
                 "[HIGHS-FRONTIER] phase=%s invalid_sizes n=%lld x=%lld "
                 "lpcols=%lld\n",
                 phase != nullptr ? phase : "unknown",
                 static_cast<long long>(n),
                 static_cast<long long>(sol.col_value.size()),
                 static_cast<long long>(lp.num_col_));
    return;
  }

  HighsInt basic_cols = 0;
  HighsInt nonbasic_lower = 0;
  HighsInt nonbasic_upper = 0;
  HighsInt nonbasic_zero = 0;
  HighsInt nonbasic_unknown = 0;
  HighsInt fixed_cols = 0;
  HighsInt integer_like_total = 0;
  HighsInt fractional_total = 0;
  HighsInt fractional_binary = 0;
  HighsInt fractional_integer = 0;
  HighsInt fractional_implied = 0;

  struct FrontierTerm {
    HighsInt col;
    double value;
    double rounded;
    double frac;
    char status;
    double rc;
    const char* type;
  };
  struct SideTerm {
    HighsInt col;
    double value;
    double lb;
    double ub;
    char status;
    double rc;
    const char* type;
  };

  std::vector<FrontierTerm> frontier;
  std::vector<SideTerm> side_terms;
  frontier.reserve(256);
  side_terms.reserve(256);
  const double int_tol = data.feastol;
  std::uint64_t status_hash = 0x4849474853535441ULL;

  for (HighsInt j = 0; j < n; ++j) {
    const HighsBasisStatus st =
        basis.valid && j < static_cast<HighsInt>(basis.col_status.size())
            ? basis.col_status[j]
            : HighsBasisStatus::kNonbasic;
    const char st_char = hacdcpfHighsBasisStatusChar(st);
    status_hash =
        hacdcpfHighsTraceHashMix(status_hash, static_cast<std::uint64_t>(j));
    status_hash = hacdcpfHighsTraceHashMix(
        status_hash, static_cast<std::uint64_t>(st_char));
    status_hash = hacdcpfHighsTraceHashMix(
        status_hash, hacdcpfHighsTraceHashDouble(sol.col_value[j]));
    switch (st) {
      case HighsBasisStatus::kBasic:
        ++basic_cols;
        break;
      case HighsBasisStatus::kUpper:
        ++nonbasic_upper;
        break;
      case HighsBasisStatus::kZero:
        ++nonbasic_zero;
        break;
      case HighsBasisStatus::kLower:
        ++nonbasic_lower;
        break;
      case HighsBasisStatus::kNonbasic:
        ++nonbasic_unknown;
        break;
    }

    const double lb = lp.col_lower_[j];
    const double ub = lp.col_upper_[j];
    const bool is_fixed =
        std::isfinite(lb) && std::isfinite(ub) &&
        ub - lb <= std::max(1e-9, int_tol);
    if (is_fixed) ++fixed_cols;

    const HighsVarType vt = data.mipsolver.variableType(j);
    const bool binary_like =
        vt == HighsVarType::kInteger &&
        data.mipsolver.model_->col_lower_[j] == 0.0 &&
        data.mipsolver.model_->col_upper_[j] == 1.0;
    const bool implied = vt == HighsVarType::kImplicitInteger;
    const bool integer_like = vt != HighsVarType::kContinuous;
    if (!integer_like) continue;
    ++integer_like_total;
    const double rc =
        j < static_cast<HighsInt>(sol.col_dual.size())
            ? sol.col_dual[j]
            : std::numeric_limits<double>::quiet_NaN();
    if (st != HighsBasisStatus::kBasic) {
      side_terms.push_back(SideTerm{
          j, sol.col_value[j], lb, ub, st_char, rc,
          hacdcpfHighsVarTypeName(vt, binary_like)});
    }

    const double value = sol.col_value[j];
    const double rounded =
        std::min(ub, std::max(lb, std::round(value)));
    const double frac = std::abs(value - rounded);
    if (!(frac > int_tol)) continue;
    ++fractional_total;
    if (binary_like)
      ++fractional_binary;
    else if (vt == HighsVarType::kInteger)
      ++fractional_integer;
    else if (implied)
      ++fractional_implied;

    frontier.push_back(FrontierTerm{
        j, value, rounded, frac, st_char, rc,
        hacdcpfHighsVarTypeName(vt, binary_like)});
  }

  std::sort(frontier.begin(), frontier.end(),
            [](const FrontierTerm& a, const FrontierTerm& b) {
              return a.col < b.col;
            });
  std::sort(side_terms.begin(), side_terms.end(),
            [](const SideTerm& a, const SideTerm& b) {
              return a.col < b.col;
            });

  std::uint64_t hash = 0x4849474853465241ULL;
  for (const FrontierTerm& t : frontier) {
    hash = hacdcpfHighsTraceHashMix(hash,
                                    static_cast<std::uint64_t>(t.col));
    hash = hacdcpfHighsTraceHashMix(hash,
                                    hacdcpfHighsTraceHashDouble(t.value));
    hash = hacdcpfHighsTraceHashMix(hash,
                                    static_cast<std::uint64_t>(t.status));
  }

  std::ostringstream sample;
  sample << std::setprecision(12);
  const HighsInt max_terms = hacdcpfHighsFrontierTraceTerms();
  for (HighsInt k = 0; k < static_cast<HighsInt>(frontier.size()) &&
                       k < max_terms;
       ++k) {
    const FrontierTerm& t = frontier[k];
    if (k > 0) sample << ";";
    sample << t.col << ":" << t.value << ":r" << t.rounded << ":f"
           << std::setprecision(6) << t.frac << ":" << t.status << ":rc"
           << t.rc << ":" << t.type << ":"
           << (std::string(t.type) == "impl" ? "impl" : "decl")
           << std::setprecision(12);
  }
  std::ostringstream side_sample;
  side_sample << std::setprecision(12);
  for (HighsInt k = 0; k < static_cast<HighsInt>(side_terms.size()) &&
                       k < max_terms;
       ++k) {
    const SideTerm& t = side_terms[k];
    if (k > 0) side_sample << ";";
    side_sample << t.col << ":" << t.value << ":lb" << t.lb << ":ub"
                << t.ub << ":" << t.status << ":rc" << std::setprecision(6)
                << t.rc << ":" << t.type << ":"
                << (std::string(t.type) == "impl" ? "impl" : "decl")
                << std::setprecision(12);
  }

  std::uint64_t fraclist_hash = 0x4849474853464c53ULL;
  std::ostringstream fraclist_sample;
  fraclist_sample << std::setprecision(12);
  const auto& fraclist = lprelax.getFractionalIntegers();
  for (HighsInt k = 0; k < static_cast<HighsInt>(fraclist.size()); ++k) {
    const HighsInt col = fraclist[k].first;
    fraclist_hash = hacdcpfHighsTraceHashMix(
        fraclist_hash, static_cast<std::uint64_t>(col));
    fraclist_hash = hacdcpfHighsTraceHashMix(
        fraclist_hash, hacdcpfHighsTraceHashDouble(fraclist[k].second));
    if (k >= max_terms) continue;
    const HighsVarType vt = data.mipsolver.variableType(col);
    const bool binary_like =
        vt == HighsVarType::kInteger &&
        data.mipsolver.model_->col_lower_[col] == 0.0 &&
        data.mipsolver.model_->col_upper_[col] == 1.0;
    if (k > 0) fraclist_sample << ";";
    fraclist_sample << col << ":" << fraclist[k].second << ":"
                    << hacdcpfHighsVarTypeName(vt, binary_like);
  }

  highsLogUser(
      data.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-FRONTIER] phase=%s obj=%.12g cols=%lld rows=%lld "
      "basis=basic%lld:lower%lld:upper%lld:zero%lld:nonbasic%lld:fixed%lld "
      "integer_like=%lld frac=%lld bin=%lld int=%lld impl=%lld hash=%016llx "
      "status_hash=%016llx "
      "basis_valid=%d basis_cols=%lld lp_fraclist=%lld fraclist_hash=%016llx "
      "sample=[%s] fraclist_sample=[%s]\n",
      phase != nullptr ? phase : "unknown", lprelax.getObjective(),
      static_cast<long long>(n), static_cast<long long>(lp.num_row_),
      static_cast<long long>(basic_cols),
      static_cast<long long>(nonbasic_lower),
      static_cast<long long>(nonbasic_upper),
      static_cast<long long>(nonbasic_zero),
      static_cast<long long>(nonbasic_unknown),
      static_cast<long long>(fixed_cols),
      static_cast<long long>(integer_like_total),
      static_cast<long long>(fractional_total),
      static_cast<long long>(fractional_binary),
      static_cast<long long>(fractional_integer),
      static_cast<long long>(fractional_implied),
      static_cast<unsigned long long>(hash),
      static_cast<unsigned long long>(status_hash), basis.valid ? 1 : 0,
      static_cast<long long>(basis.col_status.size()),
      static_cast<long long>(fraclist.size()),
      static_cast<unsigned long long>(fraclist_hash),
      sample.str().c_str(), fraclist_sample.str().c_str());
  highsLogUser(
      data.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-LPSTATE] phase=%s status_hash=%016llx "
      "nonbasic_integer_like=%lld side_sample=[%s]\n",
      phase != nullptr ? phase : "unknown",
      static_cast<unsigned long long>(status_hash),
      static_cast<long long>(side_terms.size()), side_sample.str().c_str());
}

static void hacdcpfLogHighsTimeline(
    HighsMipSolverData& data, const char* phase,
    double previous_lower_bound, double previous_upper_bound,
    double pruned_treeweight_delta,
    const int solution_source = kSolutionSourceNone) {
  if (!hacdcpfHighsTimelineEnabled()) return;
  double dual_bound = 0.0;
  double primal_bound = 0.0;
  const double gap =
      data.limitsToGap(data.lower_bound, data.upper_bound, dual_bound,
                       primal_bound);
  const double queue_lb = data.nodequeue.empty()
                              ? kHighsInf
                              : data.nodequeue.getBestLowerBound();
  highsLogUser(
      data.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-TL] phase=%s time_ms=%.0f node=%lld q=%lld lb=%.10g "
      "ub=%.10g upper_limit=%.10g opt_limit=%.10g gap=%.6g "
      "queue_lb=%.10g prev_lb=%.10g prev_ub=%.10g pruned_tw=%.6g "
      "sols=%" HIGHSINT_FORMAT " src=%s\n",
      phase != nullptr ? phase : "state", 1000.0 * data.mipsolver.timer_.read(),
      static_cast<long long>(data.num_nodes),
      static_cast<long long>(data.nodequeue.numActiveNodes()), data.lower_bound,
      data.upper_bound, data.upper_limit, data.optimality_limit, gap, queue_lb,
      previous_lower_bound, previous_upper_bound, pruned_treeweight_delta,
      data.numImprovingSols,
      data.solutionSourceToString(solution_source, false).c_str());
}

static void hacdcpfLogHighsConformanceState(HighsMipSolverData& data,
                                            const char* phase) {
  if (!hacdcpfHighsConfEnabled()) return;

  HighsInt num_binary = 0;
  HighsInt num_general_integer = 0;
  HighsInt num_implied_integer = 0;
  HighsInt num_continuous = 0;
  HighsInt num_domain_fixed = 0;
  std::uint64_t implied_hash = 0x4849474853494948ULL;
  std::ostringstream implied_sample;
  implied_sample << std::setprecision(12);
  HighsInt implied_emit = 0;
  const HighsInt max_terms = hacdcpfHighsFrontierTraceTerms();
  for (HighsInt i = 0; i != data.mipsolver.numCol(); ++i) {
    if (data.domain.isFixed(i)) {
      ++num_domain_fixed;
      continue;
    }
    switch (data.mipsolver.variableType(i)) {
      case HighsVarType::kContinuous:
        ++num_continuous;
        break;
      case HighsVarType::kImplicitInteger:
        ++num_implied_integer;
        implied_hash = hacdcpfHighsTraceHashMix(
            implied_hash, static_cast<std::uint64_t>(i));
        implied_hash = hacdcpfHighsTraceHashMix(
            implied_hash,
            hacdcpfHighsTraceHashDouble(data.mipsolver.model_->col_lower_[i]));
        implied_hash = hacdcpfHighsTraceHashMix(
            implied_hash,
            hacdcpfHighsTraceHashDouble(data.mipsolver.model_->col_upper_[i]));
        if (implied_emit < max_terms) {
          if (implied_emit++ > 0) implied_sample << ";";
          implied_sample << i << ":lb" << data.mipsolver.model_->col_lower_[i]
                         << ":ub" << data.mipsolver.model_->col_upper_[i]
                         << ":impl";
        }
        break;
      case HighsVarType::kInteger:
        if (data.mipsolver.model_->col_lower_[i] == 0.0 &&
            data.mipsolver.model_->col_upper_[i] == 1.0)
          ++num_binary;
        else
          ++num_general_integer;
        break;
      case HighsVarType::kSemiContinuous:
      case HighsVarType::kSemiInteger:
        break;
    }
  }

  const HighsLp& model = *data.mipsolver.model_;
  HighsSparseMatrix rowwise_matrix = model.a_matrix_;
  rowwise_matrix.ensureRowwise();
  std::uint64_t row_hash = 0x4849474853524f57ULL;
  HighsInt ranged_rows = 0;
  std::ostringstream row_sample;
  row_sample << std::setprecision(12);
  HighsInt row_emit = 0;
  for (HighsInt r = 0; r < model.num_row_; ++r) {
    const double lhs = model.row_lower_[r];
    const double rhs = model.row_upper_[r];
    if (lhs > -0.5 * kHighsInf && rhs < 0.5 * kHighsInf &&
        std::abs(lhs - rhs) > data.feastol) {
      ++ranged_rows;
    }
    row_hash =
        hacdcpfHighsTraceHashMix(row_hash, static_cast<std::uint64_t>(r));
    row_hash = hacdcpfHighsTraceHashMix(row_hash,
                                        hacdcpfHighsTraceHashDouble(lhs));
    row_hash = hacdcpfHighsTraceHashMix(row_hash,
                                        hacdcpfHighsTraceHashDouble(rhs));
    HighsInt nnz = 0;
    if (static_cast<HighsInt>(rowwise_matrix.start_.size()) > r + 1) {
      nnz = rowwise_matrix.start_[r + 1] - rowwise_matrix.start_[r];
    }
    row_hash =
        hacdcpfHighsTraceHashMix(row_hash, static_cast<std::uint64_t>(nnz));
    if (row_emit < max_terms) {
      if (row_emit++ > 0) row_sample << ";";
      row_sample << r << ":lhs" << lhs << ":rhs" << rhs << ":nnz" << nnz;
    }
  }

  const auto& clq = data.cliquetable;
  const auto& impl = data.implications;
  const auto& cs = clq.getSourceConformanceStats();
  const auto& is = impl.getSourceConformanceStats();
  highsLogUser(
      data.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-CONF] phase=%s model rows=%" HIGHSINT_FORMAT
      " cols=%" HIGHSINT_FORMAT " nnz=%" HIGHSINT_FORMAT
      " bin=%" HIGHSINT_FORMAT " int=%" HIGHSINT_FORMAT
      " implint=%" HIGHSINT_FORMAT " cont=%" HIGHSINT_FORMAT
      " fixed=%" HIGHSINT_FORMAT " obj_bin=%" HIGHSINT_FORMAT
      " obj_part=%" HIGHSINT_FORMAT "\n",
      phase, data.mipsolver.numRow(), data.mipsolver.numCol(),
      data.mipsolver.numNonzero(), num_binary, num_general_integer,
      num_implied_integer, num_continuous, num_domain_fixed,
      data.objectiveFunction.getNumBinariesInObjective(),
      data.objectiveFunction.getNumCliquePartitions());
  highsLogUser(
      data.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-CONF] phase=%s table entries=%" HIGHSINT_FORMAT
      " cliques=%" HIGHSINT_FORMAT " size2=%" HIGHSINT_FORMAT
      " fix=%" HIGHSINT_FORMAT " clq_subst=%lld impl_subst=%lld "
      "implications=%" HIGHSINT_FORMAT " varbounds=%" HIGHSINT_FORMAT "\n",
      phase, clq.getNumEntries(), clq.numCliques(),
      clq.getNumSizeTwoCliques(), clq.getNumFixings(),
      static_cast<long long>(clq.getSubstitutions().size()),
      static_cast<long long>(impl.substitutions.size()), impl.getNumImplications(),
      impl.getNumVarBounds());
  highsLogUser(
      data.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-CONF] phase=%s source add=%lld/%lld size2=%lld edge=%lld "
      "cover=%lld/%lld foundCover=%lld/%lld subst=%lld vertexInf=%lld "
      "cutRows=%lld mixed=%lld cutVUB=%lld cutVLB=%lld cutClq=%lld/%lld "
      "rowScan=%lld sides=%lld setppc=%lld objCalls=%lld objLit=%lld "
      "objClq=%lld probe=%lld probeConf=%lld probeRed=%lld probeSubst=%lld "
      "binImplClq=%lld vub=%lld/%lld/%lld vlb=%lld/%lld/%lld\n",
      phase, static_cast<long long>(cs.add_clique_actual),
      static_cast<long long>(cs.add_clique_calls),
      static_cast<long long>(cs.add_clique_size2_actual),
      static_cast<long long>(cs.add_clique_edge_checks),
      static_cast<long long>(cs.process_new_edge_cover_hits),
      static_cast<long long>(cs.process_new_edge_cover_tests),
      static_cast<long long>(cs.found_cover_hits),
      static_cast<long long>(cs.found_cover_calls),
      static_cast<long long>(cs.process_new_edge_substitutions),
      static_cast<long long>(cs.vertex_infeasible),
      static_cast<long long>(cs.extract_cut_calls),
      static_cast<long long>(cs.extract_cut_mixed_rows),
      static_cast<long long>(cs.extract_cut_vub_candidates),
      static_cast<long long>(cs.extract_cut_vlb_candidates),
      static_cast<long long>(cs.extract_cut_cliques_added),
      static_cast<long long>(cs.extract_cut_clique_rows),
      static_cast<long long>(cs.extract_rows_scanned),
      static_cast<long long>(cs.extract_rows_transformed_sides),
      static_cast<long long>(cs.extract_rows_direct_setppc),
      static_cast<long long>(cs.extract_obj_calls),
      static_cast<long long>(cs.extract_obj_candidate_literals),
      static_cast<long long>(cs.extract_obj_cliques_added),
      static_cast<long long>(is.probing_calls),
      static_cast<long long>(is.probing_conflicts),
      static_cast<long long>(is.probing_reductions),
      static_cast<long long>(is.probing_substitutions),
      static_cast<long long>(is.binary_implication_cliques),
      static_cast<long long>(is.vub_accepted),
      static_cast<long long>(is.vub_attempts),
      static_cast<long long>(is.vub_replaced),
      static_cast<long long>(is.vlb_accepted),
      static_cast<long long>(is.vlb_attempts),
      static_cast<long long>(is.vlb_replaced));
  highsLogUser(
      data.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-PRESOLVE-STATE] phase=%s rows=%" HIGHSINT_FORMAT
      " cols=%" HIGHSINT_FORMAT " nnz=%" HIGHSINT_FORMAT
      " ranged=%" HIGHSINT_FORMAT " bin=%" HIGHSINT_FORMAT
      " int=%" HIGHSINT_FORMAT " impl=%" HIGHSINT_FORMAT
      " cont=%" HIGHSINT_FORMAT " fixed=%" HIGHSINT_FORMAT
      " implied_hash=%016llx row_hash=%016llx "
      "implied_sample=[%s] row_sample=[%s]\n",
      phase, data.mipsolver.numRow(), data.mipsolver.numCol(),
      data.mipsolver.numNonzero(), ranged_rows, num_binary,
      num_general_integer, num_implied_integer, num_continuous,
      num_domain_fixed, static_cast<unsigned long long>(implied_hash),
      static_cast<unsigned long long>(row_hash), implied_sample.str().c_str(),
      row_sample.str().c_str());
  impl.logConformanceVarBounds(phase, max_terms);
}

std::string HighsMipSolverData::solutionSourceToString(
    const int solution_source, const bool code) const {
  if (solution_source == kSolutionSourceNone) {
    if (code) return " ";
    return "None";
    //  } else if (solution_source == kSolutionSourceInitial) {
    //    if (code) return "0";
    //    return "Initial";
  } else if (solution_source == kSolutionSourceBranching) {
    if (code) return "B";
    return "Branching";
  } else if (solution_source == kSolutionSourceCentralRounding) {
    if (code) return "C";
    return "Central rounding";
  } else if (solution_source == kSolutionSourceFeasibilityPump) {
    if (code) return "F";
    return "Feasibility pump";
  } else if (solution_source == kSolutionSourceHeuristic) {
    if (code) return "H";
    return "Heuristic";
  } else if (solution_source == kSolutionSourceShifting) {
    if (code) return "I";
    return "Shifting";
  } else if (solution_source == kSolutionSourceFeasibilityJump) {
    if (code) return "J";
    return "Feasibility jump";
  } else if (solution_source == kSolutionSourceSubMip) {
    if (code) return "L";
    return "Sub-MIP";
  } else if (solution_source == kSolutionSourceEmptyMip) {
    if (code) return "P";
    return "Empty MIP";
  } else if (solution_source == kSolutionSourceRandomizedRounding) {
    if (code) return "R";
    return "Randomized rounding";
  } else if (solution_source == kSolutionSourceSolveLp) {
    if (code) return "S";
    return "Solve LP";
  } else if (solution_source == kSolutionSourceEvaluateNode) {
    if (code) return "T";
    return "Evaluate node";
  } else if (solution_source == kSolutionSourceUnbounded) {
    if (code) return "U";
    return "Unbounded";
  } else if (solution_source == kSolutionSourceUserSolution) {
    if (code) return "X";
    return "User solution";
  } else if (solution_source == kSolutionSourceHighsSolution) {
    if (code) return "Y";
    return "HiGHS solution";
  } else if (solution_source == kSolutionSourceZiRound) {
    if (code) return "Z";
    return "ZI Round";
  } else if (solution_source == kSolutionSourceTrivialZ) {
    if (code) return "z";
    return "Trivial zero";
  } else if (solution_source == kSolutionSourceTrivialL) {
    if (code) return "l";
    return "Trivial lower";
  } else if (solution_source == kSolutionSourceTrivialU) {
    if (code) return "u";
    return "Trivial upper";
  } else if (solution_source == kSolutionSourceTrivialP) {
    if (code) return "p";
    return "Trivial point";
  } else if (solution_source == kSolutionSourceCleanup) {
    if (code) return " ";
    return "";
  } else {
    printf("HighsMipSolverData::solutionSourceToString: Unknown source = %d\n",
           solution_source);
    assert(0 == 111);
    if (code) return "*";
    return "None";
  }
}

bool HighsMipSolverData::checkSolution(
    const std::vector<double>& solution) const {
  for (HighsInt i = 0; i != mipsolver.numCol(); ++i) {
    if (solution[i] < mipsolver.model_->col_lower_[i] - feastol) return false;
    if (solution[i] > mipsolver.model_->col_upper_[i] + feastol) return false;
    if (mipsolver.isColInteger(i) && fractionality(solution[i]) > feastol)
      return false;
  }

  for (HighsInt i = 0; i != mipsolver.numRow(); ++i) {
    double rowactivity = 0.0;

    HighsInt start = ARstart_[i];
    HighsInt end = ARstart_[i + 1];

    for (HighsInt j = start; j != end; ++j)
      rowactivity += solution[ARindex_[j]] * ARvalue_[j];

    if (rowactivity > mipsolver.rowUpper(i) + feastol) return false;
    if (rowactivity < mipsolver.rowLower(i) - feastol) return false;
  }

  return true;
}

std::vector<std::tuple<HighsInt, HighsInt, double>>
HighsMipSolverData::getInfeasibleRows(
    const std::vector<double>& solution) const {
  std::vector<std::tuple<HighsInt, HighsInt, double>> infeasibleRows;
  for (HighsInt i = 0; i != mipsolver.numRow(); ++i) {
    HighsInt start = ARstart_[i];
    HighsInt end = ARstart_[i + 1];

    HighsCDouble row_activity_quad = 0.0;
    for (HighsInt j = start; j != end; ++j)
      row_activity_quad +=
          static_cast<HighsCDouble>(solution[ARindex_[j]]) * ARvalue_[j];

    double row_activity = static_cast<double>(row_activity_quad);
    if (row_activity > mipsolver.rowUpper(i) + feastol) {
      double difference = std::abs(row_activity - mipsolver.rowUpper(i));
      infeasibleRows.push_back({i, +1, difference});
    }
    if (row_activity < mipsolver.rowLower(i) - feastol) {
      double difference = std::abs(mipsolver.rowLower(i) - row_activity);
      infeasibleRows.push_back({i, -1, difference});
    }
  }
  return infeasibleRows;
}

bool HighsMipSolverData::trySolution(const std::vector<double>& solution,
                                     const int solution_source) {
  if (int(solution.size()) != mipsolver.numCol()) return false;

  HighsCDouble obj = 0;

  for (HighsInt i = 0; i != mipsolver.numCol(); ++i) {
    if (solution[i] < mipsolver.model_->col_lower_[i] - feastol) return false;
    if (solution[i] > mipsolver.model_->col_upper_[i] + feastol) return false;
    if (mipsolver.isColInteger(i) && fractionality(solution[i]) > feastol)
      return false;

    obj += mipsolver.colCost(i) * solution[i];
  }

  for (HighsInt i = 0; i != mipsolver.numRow(); ++i) {
    double rowactivity = 0.0;

    HighsInt start = ARstart_[i];
    HighsInt end = ARstart_[i + 1];

    for (HighsInt j = start; j != end; ++j)
      rowactivity += solution[ARindex_[j]] * ARvalue_[j];

    if (rowactivity > mipsolver.rowUpper(i) + feastol) return false;
    if (rowactivity < mipsolver.rowLower(i) - feastol) return false;
  }

  return addIncumbent(solution, double(obj), solution_source);
}

bool HighsMipSolverData::solutionRowFeasible(
    const std::vector<double>& solution) const {
  for (HighsInt i = 0; i != mipsolver.numRow(); ++i) {
    HighsCDouble c_double_rowactivity = HighsCDouble(0.0);

    HighsInt start = ARstart_[i];
    HighsInt end = ARstart_[i + 1];

    for (HighsInt j = start; j != end; ++j)
      c_double_rowactivity += HighsCDouble(solution[ARindex_[j]] * ARvalue_[j]);

    double rowactivity = double(c_double_rowactivity);
    if (rowactivity > mipsolver.rowUpper(i) + feastol) return false;
    if (rowactivity < mipsolver.rowLower(i) - feastol) return false;
  }
  return true;
}

HighsModelStatus HighsMipSolverData::trivialHeuristics() {
  //  printf("\nHighsMipSolverData::trivialHeuristics() Number of continuous
  //  columns is %d\n",
  //	 int(continuous_cols.size()));
  if (continuous_cols.size() > 0) return HighsModelStatus::kNotset;
  const HighsInt num_try_heuristic = 4;
  const std::vector<int> heuristic_source = {
      kSolutionSourceTrivialZ, kSolutionSourceTrivialL, kSolutionSourceTrivialU,
      kSolutionSourceTrivialP};

  std::vector<double> col_lower = mipsolver.model_->col_lower_;
  std::vector<double> col_upper = mipsolver.model_->col_upper_;
  const std::vector<double>& row_lower = mipsolver.model_->row_lower_;
  const std::vector<double>& row_upper = mipsolver.model_->row_upper_;
  const HighsSparseMatrix& matrix = mipsolver.model_->a_matrix_;
  // Determine the following properties, according to which some
  // trivial heuristics are duplicated or fail immediately
  bool all_integer_lower_non_positive = true;
  bool all_integer_lower_zero = true;
  bool all_integer_lower_finite = true;
  bool all_integer_upper_finite = true;
  for (HighsInt integer_col = 0; integer_col < numintegercols; integer_col++) {
    HighsInt iCol = integer_cols[integer_col];
    // Round bounds in to nearest integer
    col_lower[iCol] = std::ceil(col_lower[iCol]);
    col_upper[iCol] = std::floor(col_upper[iCol]);
    const bool legal_bounds =
        col_lower[iCol] <= col_upper[iCol] && col_lower[iCol] < kHighsInf &&
        col_upper[iCol] > -kHighsInf && !std::isnan(col_lower[iCol]) &&
        !std::isnan(col_upper[iCol]);
    if (!legal_bounds) {
      assert(legal_bounds);
      highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                   "HighsMipSolverData::trivialHeuristics() has detected "
                   "infeasible/illegal bounds [%g, %g] for column %d: MIP is "
                   "infeasible\n",
                   col_lower[iCol], col_upper[iCol], int(iCol));
      return HighsModelStatus::kInfeasible;
    }
    // If bounds are inconsistent then MIP is infeasible
    if (col_lower[iCol] > col_upper[iCol]) return HighsModelStatus::kInfeasible;

    if (col_lower[iCol] > 0) all_integer_lower_non_positive = false;
    if (col_lower[iCol]) all_integer_lower_zero = false;
    if (col_lower[iCol] <= -kHighsInf) all_integer_lower_finite = false;
    if (col_upper[iCol] >= kHighsInf) all_integer_upper_finite = false;
    // Only continue if one of the properties still holds
    if (!(all_integer_lower_non_positive || all_integer_lower_zero ||
          all_integer_upper_finite))
      break;
  }
  const bool all_integer_boxed =
      all_integer_lower_finite && all_integer_upper_finite;
  //  printf(
  //      "Trying trivial heuristics\n"
  //      "   all_integer_lower_non_positive = %d\n"
  //      "   all_integer_lower_zero = %d\n"
  //      "   all_integer_upper_finite = %d\n"
  //      "   all_integer_boxed = %d\n",
  //      all_integer_lower_non_positive, all_integer_lower_zero,
  //      all_integer_upper_finite, all_integer_boxed);
  const double feasibility_tolerance =
      mipsolver.options_mip_->mip_feasibility_tolerance;
  // Loop through the trivial heuristics
  std::vector<double> solution(mipsolver.numCol());
  for (HighsInt try_heuristic = 0; try_heuristic < num_try_heuristic;
       try_heuristic++) {
    if (try_heuristic == 0) {
      // First heuristic is to see whether all-zero for integer
      // variables is feasible
      //
      // If there is a positive lower bound then the heuristic fails
      if (!all_integer_lower_non_positive) continue;
      // Determine whether a zero row activity is feasible
      bool heuristic_failed = false;
      for (HighsInt iRow = 0; iRow < mipsolver.numRow(); iRow++) {
        if (row_lower[iRow] > feasibility_tolerance ||
            row_upper[iRow] < -feasibility_tolerance) {
          heuristic_failed = true;
          break;
        }
      }
      if (heuristic_failed) continue;
      solution.assign(mipsolver.numCol(), 0);
    } else if (try_heuristic == 1) {
      // Second heuristic is to see whether all-lower for integer
      // variables (if distinct from all-zero) is feasible
      if (all_integer_lower_zero) continue;
      // Trivially feasible for columns
      if (!solutionRowFeasible(col_lower)) continue;
      solution = col_lower;
    } else if (try_heuristic == 2) {
      // Third heuristic is to see whether all-upper for integer
      // variables is feasible
      //
      // If there is an infinite upper bound then the heuristic fails
      if (!all_integer_upper_finite) continue;
      // Trivially feasible for columns
      if (!solutionRowFeasible(col_upper)) continue;
      solution = col_upper;
    } else if (try_heuristic == 3) {
      // Fourth heuristic is to see whether the "lock point" is feasible
      if (!all_integer_boxed) continue;
      for (HighsInt integer_col = 0; integer_col < numintegercols;
           integer_col++) {
        HighsInt iCol = integer_cols[integer_col];
        HighsInt num_positive_values = 0;
        HighsInt num_negative_values = 0;
        for (HighsInt iEl = matrix.start_[iCol]; iEl < matrix.start_[iCol + 1];
             iEl++) {
          if (matrix.value_[iEl] > 0)
            num_positive_values++;
          else
            num_negative_values++;
        }
        solution[iCol] = num_positive_values > num_negative_values
                             ? col_lower[iCol]
                             : col_upper[iCol];
      }
      // Trivially feasible for columns
      if (!solutionRowFeasible(solution)) continue;
    }

    HighsCDouble cdouble_obj = 0.0;
    for (HighsInt iCol = 0; iCol < mipsolver.numCol(); iCol++)
      cdouble_obj += mipsolver.colCost(iCol) * solution[iCol];
    double obj = double(cdouble_obj);
    const double save_upper_bound = upper_bound;
    const bool new_incumbent =
        addIncumbent(solution, obj, heuristic_source[try_heuristic]);
    const bool lc_report = false;
    if (lc_report) {
      printf("Trivial heuristic %d has succeeded: objective = %g",
             int(try_heuristic), obj);
      if (new_incumbent) {
        printf("; upper bound from %g to %g\n", save_upper_bound, upper_bound);
      } else {
        printf("\n");
      }
    }
  }
  return HighsModelStatus::kNotset;
}

void HighsMipSolverData::startAnalyticCenterComputation(
    const highs::parallel::TaskGroup& taskGroup) {
  taskGroup.spawn([&]() {
    // first check if the analytic centre computation should be cancelled, e.g.
    // due to early return in the root node evaluation
    Highs ipm;
    ipm.setOptionValue("output_flag", false);
    const std::vector<double>& sol = ipm.getSolution().col_value;
    // Don't use presolve - because this can lead to postsolve putting
    // integer variables onto bounds. This is not just a "less good"
    // AC. It can have implications leading to erroneous fixing of
    // variables and a suboptimal solution declared as optimal.
    ipm.setOptionValue("presolve", kHighsOffString);
    // Determine the solver
    const std::string mip_ipm_solver = mipsolver.options_mip_->mip_ipm_solver;
    // Currently use IPX by default and take action on failure here if
    // using HiPO.
    bool use_hipo =
        /*
  #ifdef HIPO
        // Later use HiPO by default
        mip_ipm_solver == kHighsChooseString ||
  #endif
        */
        mip_ipm_solver == kHipoString;
    // Later still, pass mip_ipm_solver and take action on failure in
    // solveLp
#ifndef HIPO
    // Shouldn't be possible to choose HiPO if it's not in the build
    assert(!use_hipo);
    use_hipo = false;
#endif
    const std::string ipm_solver = use_hipo ? kHipoString : kIpxString;
    ipm.setOptionValue("solver", ipm_solver);
    ipm.setOptionValue("ipm_iteration_limit", 200);
    ipm.setOptionValue("run_crossover", kHighsOffString);
    ipm.setOptionValue("run_centring", true);
    HighsLp lpmodel(*mipsolver.model_);
    lpmodel.col_cost_.assign(lpmodel.num_col_, 0.0);
    lpmodel.integrality_.clear();
    ipm.passModel(std::move(lpmodel));
    const bool dump_ipm_lp = false;
    if (dump_ipm_lp && !mipsolver.submip) {
      const std::string file_name = mipsolver.model_->model_name_ + "_ac.mps";
      printf(
          "HighsMipSolverData::startAnalyticCenterComputation: Calling "
          "ipm.writeModel(%s)\n",
          file_name.c_str());
      ipm.writeModel(file_name);
      fflush(stdout);
      exit(1);
    }
    const bool ipm_logging = false;
    if (ipm_logging) {
      bool output_flag;
      ipm.getOptionValue("output_flag", output_flag);
      assert(output_flag == false);
      (void)output_flag;
      ipm.setOptionValue("output_flag", !mipsolver.submip);
    }
    ipm.optimizeLp();
    if (ipm_logging) ipm.setOptionValue("output_flag", false);
    if (use_hipo && mip_ipm_solver == kHighsChooseString &&
        HighsInt(sol.size()) != mipsolver.numCol()) {
      printf(
          "In HighsMipSolverData::startAnalyticCenterComputation HiPO has "
          "failed to get a solution: status = %s Try IPX\n",
          ipm.modelStatusToString(ipm.getModelStatus()).c_str());
      // HiPO has failed to get a solution, so try IPX
      ipm.setOptionValue("solver", kIpxString);
      ipm.optimizeLp();
    }
    if (!mipsolver.submip) {
      const HighsSubSolverCallTime& sub_solver_call_time =
          ipm.getSubSolverCallTime();
      const bool analytic_centre = true;
      mipsolver.analysis_.addSubSolverCallTime(sub_solver_call_time,
                                               analytic_centre);
      // Go through sub_solver_call_time to update any MIP clocks
      const bool valid_basis = false;
      const bool use_presolve = false;
      mipsolver.analysis_.mipTimerUpdate(sub_solver_call_time, valid_basis,
                                         use_presolve, analytic_centre);
    }
    if (HighsInt(sol.size()) != mipsolver.numCol()) return;
    analyticCenterStatus = ipm.getModelStatus();
    analyticCenter = sol;
  });
}

void HighsMipSolverData::finishAnalyticCenterComputation(
    const highs::parallel::TaskGroup& taskGroup) {
  if (mipsolver.analysis_.analyse_mip_time) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "MIP-Timing: %11.2g - starting  analytic centre synch\n",
                 mipsolver.analysis_.mipTimerRead());
    fflush(stdout);
  }
  taskGroup.sync();
  if (mipsolver.analysis_.analyse_mip_time) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "MIP-Timing: %11.2g - completed analytic centre synch\n",
                 mipsolver.analysis_.mipTimerRead());
    fflush(stdout);
  }
  analyticCenterComputed = true;
  if (analyticCenterStatus == HighsModelStatus::kOptimal) {
    HighsInt nfixed = 0;
    HighsInt nintfixed = 0;
    for (HighsInt i = 0; i != mipsolver.numCol(); ++i) {
      double boundRange = mipsolver.mipdata_->domain.col_upper_[i] -
                          mipsolver.mipdata_->domain.col_lower_[i];
      if (boundRange == 0.0) continue;

      double tolerance =
          mipsolver.mipdata_->feastol * std::min(boundRange, 1.0);

      if (analyticCenter[i] <= mipsolver.model_->col_lower_[i] + tolerance) {
        mipsolver.mipdata_->domain.changeBound(
            HighsBoundType::kUpper, i, mipsolver.model_->col_lower_[i],
            HighsDomain::Reason::unspecified());
        if (mipsolver.mipdata_->domain.infeasible()) return;
        ++nfixed;
        if (mipsolver.isColInteger(i)) ++nintfixed;
      } else if (analyticCenter[i] >=
                 mipsolver.model_->col_upper_[i] - tolerance) {
        mipsolver.mipdata_->domain.changeBound(
            HighsBoundType::kLower, i, mipsolver.model_->col_upper_[i],
            HighsDomain::Reason::unspecified());
        if (mipsolver.mipdata_->domain.infeasible()) return;
        ++nfixed;
        if (mipsolver.isColInteger(i)) ++nintfixed;
      }
    }
    if (nfixed > 0)
      highsLogDev(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                  "Fixing %d columns (%d integers) sitting at bound at "
                  "analytic center\n",
                  int(nfixed), int(nintfixed));
    mipsolver.mipdata_->domain.propagate();
    if (mipsolver.mipdata_->domain.infeasible()) return;
  }
}

void HighsMipSolverData::startSymmetryDetection(
    const highs::parallel::TaskGroup& taskGroup,
    std::unique_ptr<SymmetryDetectionData>& symData) {
  symData = std::unique_ptr<SymmetryDetectionData>(new SymmetryDetectionData());
  symData->symDetection.loadModelAsGraph(
      mipsolver.mipdata_->presolvedModel,
      mipsolver.options_mip_->small_matrix_value);
  detectSymmetries = symData->symDetection.initializeDetection();

  if (detectSymmetries) {
    taskGroup.spawn([&]() {
      double startTime = mipsolver.timer_.getWallTime();
      symData->symDetection.run(symData->symmetries);
      symData->detectionTime = mipsolver.timer_.getWallTime() - startTime;
    });
  } else
    symData.reset();
}

void HighsMipSolverData::finishSymmetryDetection(
    const highs::parallel::TaskGroup& taskGroup,
    std::unique_ptr<SymmetryDetectionData>& symData) {
  taskGroup.sync();

  symmetries = std::move(symData->symmetries);
  std::string symmetry_time =
      mipsolver.options_mip_->timeless_log
          ? ""
          : highsFormatToString(" %.1fs", symData->detectionTime);
  highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
               "\nSymmetry detection completed in%s\n", symmetry_time.c_str());

  if (symmetries.numGenerators == 0) {
    detectSymmetries = false;
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "No symmetry present\n\n");
  } else if (symmetries.orbitopes.size() == 0) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "Found %d generator(s)\n\n", int(symmetries.numGenerators));

  } else {
    if (symmetries.numPerms != 0) {
      highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                   "Found %d generator(s) and %d full orbitope(s) acting on %d "
                   "columns\n\n",
                   int(symmetries.numPerms), int(symmetries.orbitopes.size()),
                   int(symmetries.columnToOrbitope.size()));
    } else {
      highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                   "Found %d full orbitope(s) acting on %d columns\n\n",
                   int(symmetries.orbitopes.size()),
                   int(symmetries.columnToOrbitope.size()));
    }
  }
  symData.reset();

  for (HighsOrbitopeMatrix& orbitope : symmetries.orbitopes)
    orbitope.determineOrbitopeType(cliquetable);

  if (symmetries.numPerms != 0)
    globalOrbits = symmetries.computeStabilizerOrbits(domain);
}

double HighsMipSolverData::limitsToGap(const double use_lower_bound,
                                       const double use_upper_bound, double& lb,
                                       double& ub) const {
  double offset = mipsolver.model_->offset_;
  lb = use_lower_bound + offset;
  if (std::abs(lb) <= epsilon) lb = 0;
  ub = kHighsInf;
  double gap = kHighsInf;
  if (use_upper_bound != kHighsInf) {
    ub = use_upper_bound + offset;
    if (std::fabs(ub) <= epsilon) ub = 0;
    lb = std::min(ub, lb);
    if (ub == 0.0)
      gap = lb == 0.0 ? 0.0 : kHighsInf;
    else
      gap = (ub - lb) / fabs(ub);
  }
  return gap;
}

double HighsMipSolverData::computeNewUpperLimit(double ub, double mip_abs_gap,
                                                double mip_rel_gap) const {
  double new_upper_limit;
  if (objectiveFunction.isIntegral()) {
    new_upper_limit =
        (std::floor(objectiveFunction.integralScale() * ub - 0.5) /
         objectiveFunction.integralScale());

    if (mip_rel_gap != 0.0)
      new_upper_limit = std::min(
          new_upper_limit,
          ub - std::ceil(mip_rel_gap * fabs(ub + mipsolver.model_->offset_) *
                             objectiveFunction.integralScale() -
                         mipsolver.mipdata_->epsilon) /
                   objectiveFunction.integralScale());

    if (mip_abs_gap != 0.0)
      new_upper_limit = std::min(
          new_upper_limit,
          ub - std::ceil(mip_abs_gap * objectiveFunction.integralScale() -
                         mipsolver.mipdata_->epsilon) /
                   objectiveFunction.integralScale());

    // add feasibility tolerance so that the next best integer feasible solution
    // is definitely included in the remaining search
    new_upper_limit += feastol;
  } else {
    new_upper_limit = std::min(ub - feastol, std::nextafter(ub, -kHighsInf));

    if (mip_rel_gap != 0.0)
      new_upper_limit =
          std::min(new_upper_limit,
                   ub - mip_rel_gap * fabs(ub + mipsolver.model_->offset_));

    if (mip_abs_gap != 0.0)
      new_upper_limit = std::min(new_upper_limit, ub - mip_abs_gap);
  }

  return new_upper_limit;
}

bool HighsMipSolverData::moreHeuristicsAllowed() const {
  // in the beginning of the search and in sub-MIP heuristics we only allow
  // what is proportionally for the currently spent effort plus an initial
  // offset. This is because in a sub-MIP we usually do a truncated search and
  // therefore should not extrapolate the time we spent for heuristics as in
  // the other case. Moreover, since we estimate the total effort for
  // exploring the tree based on the weight of the already pruned nodes, the
  // estimated effort the is not expected to be a good prediction in the
  // beginning.
  if (mipsolver.submip) {
    return heuristic_lp_iterations < total_lp_iterations * heuristic_effort;
  } else if (pruned_treeweight < 1e-3 &&
             num_leaves - num_leaves_before_run < 10 &&
             num_nodes - num_nodes_before_run < 1000) {
    // in the main MIP solver allow an initial offset of 10000 heuristic LP
    // iterations
    if (heuristic_lp_iterations <
        total_lp_iterations * heuristic_effort + 10000)
      return true;
  } else if (heuristic_lp_iterations <
             100000 + ((total_lp_iterations - heuristic_lp_iterations -
                        sb_lp_iterations) >>
                       1)) {
    // compute the node LP iterations in the current run as only those should be
    // used when estimating the total required LP iterations to complete the
    // search
    int64_t heur_iters_curr_run =
        heuristic_lp_iterations - heuristic_lp_iterations_before_run;
    int64_t sb_iters_curr_run = sb_lp_iterations - sb_lp_iterations_before_run;
    int64_t node_iters_curr_run = total_lp_iterations -
                                  total_lp_iterations_before_run -
                                  heur_iters_curr_run - sb_iters_curr_run;
    // now estimate the total fraction of LP iterations that we have spent on
    // heuristics by assuming the node iterations of the current run will
    // grow proportional to the pruned weight of the current tree and the
    // iterations spent for anything else are just added as an offset
    double total_heuristic_effort_estim =
        heuristic_lp_iterations /
        ((total_lp_iterations - node_iters_curr_run) +
         node_iters_curr_run / std::max(0.01, double(pruned_treeweight)));
    // since heuristics help most in the beginning of the search, we want to
    // spent the time we have for heuristics in the first 80% of the tree
    // exploration. Additionally we want to spent the proportional effort
    // of heuristics that is allowed in the first 30% of tree exploration as
    // fast as possible, which is why we have the max(0.3/0.8,...).
    // Hence, in the first 30% of the tree exploration we allow to spent all
    // effort available for heuristics in that part of the search as early as
    // possible, whereas after that we allow the part that is proportionally
    // adequate when we want to spent all available time in the first 80%.
    if (total_heuristic_effort_estim <
        std::max(0.3 / 0.8, std::min(double(pruned_treeweight), 0.8) / 0.8) *
            heuristic_effort) {
      // printf(
      //     "heuristic lp iterations: %ld, total_lp_iterations: %ld, "
      //     "total_heur_effort_estim = %.3f%%\n",
      //     heuristic_lp_iterations, total_lp_iterations,
      //     total_heuristic_effort_estim);
      return true;
    }
  }

  return false;
}

void HighsMipSolverData::removeFixedIndices() {
  integral_cols.erase(
      std::remove_if(integral_cols.begin(), integral_cols.end(),
                     [&](HighsInt col) { return domain.isFixed(col); }),
      integral_cols.end());
  integer_cols.erase(
      std::remove_if(integer_cols.begin(), integer_cols.end(),
                     [&](HighsInt col) { return domain.isFixed(col); }),
      integer_cols.end());
  implint_cols.erase(
      std::remove_if(implint_cols.begin(), implint_cols.end(),
                     [&](HighsInt col) { return domain.isFixed(col); }),
      implint_cols.end());
  continuous_cols.erase(
      std::remove_if(continuous_cols.begin(), continuous_cols.end(),
                     [&](HighsInt col) { return domain.isFixed(col); }),
      continuous_cols.end());
}

void HighsMipSolverData::init() {
  postSolveStack.initializeIndexMaps(mipsolver.numRow(), mipsolver.numCol());
  mipsolver.orig_model_ = mipsolver.model_;
  feastol = mipsolver.options_mip_->mip_feasibility_tolerance;
  epsilon = mipsolver.options_mip_->small_matrix_value;
  if (mipsolver.clqtableinit)
    cliquetable.buildFrom(mipsolver.orig_model_, *mipsolver.clqtableinit);
  cliquetable.setMinEntriesForParallelism(
      highs::parallel::num_threads() > 1
          ? mipsolver.options_mip_->mip_min_cliquetable_entries_for_parallelism
          : kHighsIInf);
  if (mipsolver.implicinit) implications.buildFrom(*mipsolver.implicinit);
  heuristic_effort = mipsolver.options_mip_->mip_heuristic_effort;
  detectSymmetries = mipsolver.options_mip_->mip_detect_symmetry;

  firstlpsolobj = -kHighsInf;
  rootlpsolobj = -kHighsInf;
  analyticCenterComputed = false;
  analyticCenterStatus = HighsModelStatus::kNotset;
  maxTreeSizeLog2 = 0;
  numRestarts = 0;
  numRestartsRoot = 0;
  numImprovingSols = 0;
  pruned_treeweight = 0;
  avgrootlpiters = 0;
  num_nodes = 0;
  num_nodes_before_run = 0;
  num_leaves = 0;
  num_leaves_before_run = 0;
  total_repair_lp = 0;
  total_repair_lp_feasible = 0;
  total_repair_lp_iterations = 0;
  total_lp_iterations = 0;
  heuristic_lp_iterations = 0;
  sepa_lp_iterations = 0;
  sb_lp_iterations = 0;
  total_lp_iterations_before_run = 0;
  heuristic_lp_iterations_before_run = 0;
  sepa_lp_iterations_before_run = 0;
  sb_lp_iterations_before_run = 0;
  num_disp_lines = 0;
  numCliqueEntriesAfterPresolve = 0;
  numCliqueEntriesAfterFirstPresolve = 0;
  cliquesExtracted = false;
  rowMatrixSet = false;
  lower_bound = -kHighsInf;
  upper_bound = kHighsInf;
  upper_limit = mipsolver.options_mip_->objective_bound;
  optimality_limit = mipsolver.options_mip_->objective_bound;
  primal_dual_integral.initialise();

  if (mipsolver.options_mip_->mip_report_level == 0)
    dispfreq = 0;
  else if (mipsolver.options_mip_->mip_report_level == 1)
    dispfreq = 2000;
  else
    dispfreq = 100;
}

void HighsMipSolverData::runMipPresolve(
    const HighsInt presolve_reduction_limit) {
  mipsolver.timer_.start(mipsolver.timer_.presolve_clock);
  presolve::HPresolve presolve;
  if (!presolve.okSetInput(mipsolver, presolve_reduction_limit)) {
    mipsolver.modelstatus_ = HighsModelStatus::kMemoryLimit;
    presolve_status = HighsPresolveStatus::kOutOfMemory;
  } else {
    mipsolver.modelstatus_ = presolve.run(postSolveStack);
    presolve_status = presolve.getPresolveStatus();
  }
  mipsolver.timer_.stop(mipsolver.timer_.presolve_clock);

  // Report the final presolve reductions unless this is a restart
  if (mipsolver.options_mip_->presolve != kHighsOffString && numRestarts == 0)
    reportPresolveReductions(mipsolver.options_mip_->log_options,
                             presolve_status, *mipsolver.orig_model_,
                             *mipsolver.model_);

  hacdcpfLogHighsConformanceState(*this, "presolve_complete");
}

HighsPresolveSideState HighsMipSolverData::getPresolveSideStateSnapshot(
    HighsInt max_var_bound_records) const {
  HighsPresolveSideState state;
  const HighsLp& lp = presolvedModel;
  state.available = true;
  state.rows = lp.num_row_;
  state.cols = lp.num_col_;
  state.nnz = lp.a_matrix_.numNz();
  state.presolved_row_lower = lp.row_lower_;
  state.presolved_row_upper = lp.row_upper_;
  state.presolved_col_lower = lp.col_lower_;
  state.presolved_col_upper = lp.col_upper_;
  state.presolved_col_orig.reserve(static_cast<std::size_t>(lp.num_col_));
  state.presolved_col_type.reserve(static_cast<std::size_t>(lp.num_col_));
  state.presolved_col_scale.reserve(static_cast<std::size_t>(lp.num_col_));
  state.presolved_col_constant.reserve(static_cast<std::size_t>(lp.num_col_));
  state.presolved_col_linearly_transformable.reserve(
      static_cast<std::size_t>(lp.num_col_));
  for (HighsInt r = 0; r < lp.num_row_; ++r) {
    const double lhs = lp.row_lower_[r];
    const double rhs = lp.row_upper_[r];
    if (std::isfinite(lhs) && std::isfinite(rhs) && std::abs(lhs - rhs) > 1e-9)
      ++state.ranged_rows;
  }
  for (HighsInt c = 0; c < lp.num_col_; ++c) {
    const double lb = lp.col_lower_[c];
    const double ub = lp.col_upper_[c];
    if (std::isfinite(lb) && std::isfinite(ub) && std::abs(lb - ub) <= 1e-9)
      ++state.fixed_cols;
    HighsVarType type = HighsVarType::kContinuous;
    if (c < static_cast<HighsInt>(lp.integrality_.size()))
      type = lp.integrality_[c];
    state.presolved_col_orig.push_back(postSolveStack.getOrigColIndex(c));
    state.presolved_col_type.push_back(type);
    state.presolved_col_scale.push_back(
        postSolveStack.getColLinearTransformScale(c));
    state.presolved_col_constant.push_back(
        postSolveStack.getColLinearTransformConstant(c));
    state.presolved_col_linearly_transformable.push_back(
        postSolveStack.isColLinearlyTransformable(c) ? 1 : 0);
    switch (type) {
      case HighsVarType::kContinuous:
      case HighsVarType::kSemiContinuous:
        ++state.continuous_cols;
        break;
      case HighsVarType::kImplicitInteger:
        ++state.implied_integer_cols;
        break;
      case HighsVarType::kInteger:
      case HighsVarType::kSemiInteger:
        if (std::isfinite(lb) && std::isfinite(ub) && std::abs(lb) <= 1e-9 &&
            std::abs(ub - 1.0) <= 1e-9)
          ++state.binary_cols;
        else
          ++state.integer_cols;
        break;
    }
  }
  HighsSparseMatrix row_matrix = lp.a_matrix_;
  row_matrix.ensureRowwise();
  state.presolved_a_start = row_matrix.start_;
  state.presolved_a_index = row_matrix.index_;
  state.presolved_a_value = row_matrix.value_;
  implications.appendPresolveVarBoundSnapshot(state, max_var_bound_records);
  return state;
}

void HighsMipSolverData::runSetup() {
  const HighsLp& model = *mipsolver.model_;

  // Indicate that the first LP has not been solved
  this->lp.setSolvedFirstLp(false);

  last_disptime = -kHighsInf;
  disptime = 0;

  // Transform the reference of the objective limit and lower/upper
  // bounds from the original model to the current model, undoing the
  // transformation done before restart so that the offset change due
  // to presolve is incorporated. Bound changes are transitory, so no
  // real gap change, and no update to P-D integral is necessary
  upper_limit -= mipsolver.model_->offset_;
  optimality_limit -= mipsolver.model_->offset_;

  lower_bound -= mipsolver.model_->offset_;
  upper_bound -= mipsolver.model_->offset_;

  if (mipsolver.solution_objective_ != kHighsInf) {
    // Assigning new incumbent
    incumbent = postSolveStack.getReducedPrimalSolution(mipsolver.solution_);
    // return the objective value in the transformed space
    double solobj =
        mipsolver.solution_objective_ * (int)mipsolver.orig_model_->sense_ -
        mipsolver.model_->offset_;
    bool feasible = mipsolver.bound_violation_ <=
                        mipsolver.options_mip_->mip_feasibility_tolerance &&
                    mipsolver.integrality_violation_ <=
                        mipsolver.options_mip_->mip_feasibility_tolerance &&
                    mipsolver.row_violation_ <=
                        mipsolver.options_mip_->mip_feasibility_tolerance;
    if (numRestarts == 0) {
      highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                   "\nMIP start solution is %s, objective value is %.12g\n",
                   feasible ? "feasible" : "infeasible",
                   mipsolver.solution_objective_);
    }
    if (feasible && solobj < upper_bound) {
      double prev_upper_bound = upper_bound;

      upper_bound = solobj;

      bool bound_change = upper_bound != prev_upper_bound;
      if (!mipsolver.submip && bound_change)
        updatePrimalDualIntegral(lower_bound, lower_bound, prev_upper_bound,
                                 upper_bound);

      double new_upper_limit = computeNewUpperLimit(solobj, 0.0, 0.0);

      saveReportMipSolution(new_upper_limit);
      if (new_upper_limit < upper_limit) {
        upper_limit = new_upper_limit;
        optimality_limit =
            computeNewUpperLimit(solobj, mipsolver.options_mip_->mip_abs_gap,
                                 mipsolver.options_mip_->mip_rel_gap);
        nodequeue.setOptimalityLimit(optimality_limit);
      }
    }
    if (!mipsolver.submip && feasible && mipsolver.callback_->user_callback &&
        mipsolver.callback_->active[kCallbackMipSolution]) {
      assert(!mipsolver.submip);
      mipsolver.callback_->clearHighsCallbackOutput();
      mipsolver.callback_->data_out.mip_solution = mipsolver.solution_;
      const bool interrupt = interruptFromCallbackWithData(
          kCallbackMipSolution, mipsolver.solution_objective_,
          "Feasible solution");
      assert(!interrupt);
    }
  }

  if (mipsolver.numCol() == 0)
    addIncumbent(std::vector<double>(), 0, kSolutionSourceEmptyMip);

  redcostfixing = HighsRedcostFixing();
  pseudocost = HighsPseudocost(mipsolver);
  nodequeue.setNumCol(mipsolver.numCol());
  nodequeue.setOptimalityLimit(optimality_limit);

  continuous_cols.clear();
  integer_cols.clear();
  implint_cols.clear();
  integral_cols.clear();

  rowMatrixSet = false;
  if (!rowMatrixSet) {
    rowMatrixSet = true;
    highsSparseTranspose(model.num_row_, model.num_col_, model.a_matrix_.start_,
                         model.a_matrix_.index_, model.a_matrix_.value_,
                         ARstart_, ARindex_, ARvalue_);
    // (re-)initialize number of uplocks and downlocks
    uplocks.assign(model.num_col_, 0);
    downlocks.assign(model.num_col_, 0);
    for (HighsInt i = 0; i != model.num_col_; ++i) {
      HighsInt start = model.a_matrix_.start_[i];
      HighsInt end = model.a_matrix_.start_[i + 1];
      for (HighsInt j = start; j != end; ++j) {
        HighsInt row = model.a_matrix_.index_[j];

        if (model.row_lower_[row] != -kHighsInf) {
          if (model.a_matrix_.value_[j] < 0)
            ++uplocks[i];
          else
            ++downlocks[i];
        }
        if (model.row_upper_[row] != kHighsInf) {
          if (model.a_matrix_.value_[j] < 0)
            ++downlocks[i];
          else
            ++uplocks[i];
        }
      }
    }
  }

  rowintegral.resize(mipsolver.numRow());

  // compute the maximal absolute coefficients to filter propagation
  maxAbsRowCoef.resize(mipsolver.numRow());
  for (HighsInt i = 0; i != mipsolver.numRow(); ++i) {
    double maxabsval = 0.0;

    HighsInt start = ARstart_[i];
    HighsInt end = ARstart_[i + 1];
    bool integral = true;
    for (HighsInt j = start; j != end; ++j) {
      integral = integral && mipsolver.isColIntegral(ARindex_[j]) &&
                 fractionality(ARvalue_[j]) <= epsilon;

      maxabsval = std::max(maxabsval, std::abs(ARvalue_[j]));
    }

    if (integral) {
      if (presolvedModel.row_lower_[i] != -kHighsInf)
        presolvedModel.row_lower_[i] =
            std::ceil(presolvedModel.row_lower_[i] - feastol);

      if (presolvedModel.row_upper_[i] != kHighsInf)
        presolvedModel.row_upper_[i] =
            std::floor(presolvedModel.row_upper_[i] + feastol);
    }

    rowintegral[i] = integral;
    maxAbsRowCoef[i] = maxabsval;
  }

  // compute row activities and propagate all rows once
  objectiveFunction.setupCliquePartition(domain, cliquetable);
  domain.setupObjectivePropagation();
  domain.computeRowActivities();
  domain.propagate();
  if (domain.infeasible()) {
    mipsolver.modelstatus_ = HighsModelStatus::kInfeasible;

    updateLowerBound(kHighsInf);

    pruned_treeweight = 1.0;
    return;
  }

  if (model.num_col_ == 0) {
    mipsolver.modelstatus_ = HighsModelStatus::kOptimal;
    return;
  }

  if (checkLimits()) return;
  // extract cliques if they have not been extracted before

  for (HighsInt col : domain.getChangedCols())
    implications.cleanupVarbounds(col);
  domain.clearChangedCols();

  lp.getLpSolver().setOptionValue("presolve", kHighsOffString);
  // lp.getLpSolver().setOptionValue("dual_simplex_cost_perturbation_multiplier",
  // 0.0); lp.getLpSolver().setOptionValue("parallel", kHighsOnString);
  lp.getLpSolver().setOptionValue("simplex_initial_condition_check", false);

  checkObjIntegrality();
  rootlpsol.clear();
  firstlpsol.clear();
  HighsInt num_binary = 0;
  HighsInt num_domain_fixed = 0;
  maxTreeSizeLog2 = 0;
  for (HighsInt i = 0; i != mipsolver.numCol(); ++i) {
    switch (mipsolver.variableType(i)) {
      case HighsVarType::kContinuous:
        if (domain.isFixed(i)) {
          num_domain_fixed++;
          continue;
        }
        continuous_cols.push_back(i);
        break;
      case HighsVarType::kImplicitInteger:
        if (domain.isFixed(i)) {
          num_domain_fixed++;
          continue;
        }
        implint_cols.push_back(i);
        integral_cols.push_back(i);
        break;
      case HighsVarType::kInteger:
        if (domain.isFixed(i)) {
          num_domain_fixed++;
          if (fractionality(domain.col_lower_[i]) > feastol) {
            // integer variable is fixed to a fractional value -> infeasible
            mipsolver.modelstatus_ = HighsModelStatus::kInfeasible;

            updateLowerBound(kHighsInf);

            pruned_treeweight = 1.0;
            return;
          }
          continue;
        }
        integer_cols.push_back(i);
        integral_cols.push_back(i);
        maxTreeSizeLog2 += (HighsInt)std::ceil(
            std::log2(std::min(1024.0, 1.0 + mipsolver.model_->col_upper_[i] -
                                           mipsolver.model_->col_lower_[i])));
        // NB Since this is for counting the number of times the
        // condition is true using the bitwise operator avoids having
        // any conditional branch whereas using the logical operator
        // would require a branch due to short circuit
        // evaluation. Semantically both is equivalent and correct. If
        // there was any code to be executed for the condition being
        // true then there would be a conditional branch in any case
        // and I would have used the logical to begin with.
        //
        // Hence any compiler warning can be ignored safely
        num_binary +=
            (static_cast<HighsInt>(mipsolver.model_->col_lower_[i] == 0.0) &
             static_cast<HighsInt>(mipsolver.model_->col_upper_[i] == 1.0));
        break;
      case HighsVarType::kSemiContinuous:
      case HighsVarType::kSemiInteger:
        highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kError,
                     "Semicontinuous or semiinteger variables should have been "
                     "reformulated away before HighsMipSolverData::runSetup() "
                     "is called.");
        throw std::logic_error("Unexpected variable type");
    }
  }

  basisTransfer();

  // [HACDCP improvement (7)] Build presolved col_branch_priority_ by mapping
  // original-space priorities through the postSolveStack.
  if (!mipsolver.col_branch_priority.empty()) {
    const HighsInt num_orig = static_cast<HighsInt>(mipsolver.col_branch_priority.size());
    const HighsInt num_presolved = mipsolver.numCol();
    col_branch_priority_.assign(static_cast<size_t>(num_presolved), 0);
    for (HighsInt c = 0; c < num_presolved; ++c) {
      const HighsInt orig_c = postSolveStack.getOrigColIndex(c);
      if (orig_c >= 0 && orig_c < num_orig)
        col_branch_priority_[static_cast<size_t>(c)] =
            mipsolver.col_branch_priority[static_cast<size_t>(orig_c)];
    }
  } else {
    col_branch_priority_.clear();
  }

  numintegercols = integer_cols.size();
  detectSymmetries = detectSymmetries && num_binary > 0;
  numCliqueEntriesAfterPresolve = cliquetable.getNumEntries();
  HighsInt num_col = mipsolver.numCol();
  HighsInt num_general_integer = numintegercols - num_binary;
  HighsInt num_implied_integer = implint_cols.size();
  HighsInt num_continuous = continuous_cols.size();
  assert(num_col == num_continuous + num_binary + num_general_integer +
                        num_implied_integer + num_domain_fixed);
  if (numRestarts == 0) {
    numCliqueEntriesAfterFirstPresolve = cliquetable.getNumEntries();
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 // clang-format off
		 "\nSolving MIP model with:\n"
		 "   %" HIGHSINT_FORMAT " row%s\n"
		 "   %" HIGHSINT_FORMAT " col%s ("
		 "%" HIGHSINT_FORMAT" binary, "
		 "%" HIGHSINT_FORMAT " integer, "
		 "%" HIGHSINT_FORMAT" implied int., "
		 "%" HIGHSINT_FORMAT " continuous, "
		 "%" HIGHSINT_FORMAT " domain fixed)\n"
		 "   %" HIGHSINT_FORMAT " nonzero%s\n",
                 // clang-format on
                 mipsolver.numRow(), mipsolver.numRow() == 1 ? "" : "s",
                 num_col, num_col == 1 ? "" : "s", num_binary,
                 num_general_integer, num_implied_integer, num_continuous,
                 num_domain_fixed, mipsolver.numNonzero(),
                 mipsolver.numNonzero() == 1 ? "" : "s");
  } else {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "Model after restart has "
                 // clang-format off
		 "%" HIGHSINT_FORMAT " row%s, "
		 "%" HIGHSINT_FORMAT " col%s ("
		 "%" HIGHSINT_FORMAT " bin., "
		 "%" HIGHSINT_FORMAT " int., "
		 "%" HIGHSINT_FORMAT " impl., "
		 "%" HIGHSINT_FORMAT " cont., "
		 "%" HIGHSINT_FORMAT " dom.fix.), and "
		 "%" HIGHSINT_FORMAT " nonzero%s\n",
                 // clang-format on
                 mipsolver.numRow(), mipsolver.numRow() == 1 ? "" : "s",
                 num_col, num_col == 1 ? "" : "s", num_binary,
                 num_general_integer, num_implied_integer, num_continuous,
                 num_domain_fixed, mipsolver.numNonzero(),
                 mipsolver.numNonzero() == 1 ? "" : "s");
  }

  hacdcpfLogHighsConformanceState(*this, "setup_after_propagation");

  heuristics.setupIntCols();

#ifdef HIGHS_DEBUGSOL
  if (debugSolution.debugSolActive) {
    debugSolution.debugSolution.clear();
    debugSolution.debugSolution = postSolveStack.getReducedPrimalSolution(
        debugSolution.debugOrigSolution);
    debugSolution.debugSolObjective = 0;
    HighsCDouble debugsolobj = 0.0;
    for (HighsInt i = 0; i != mipsolver.numCol(); ++i)
      debugsolobj +=
          mipsolver.colCost(i) * HighsCDouble(debugSolution.debugSolution[i]);
    debugSolution.debugSolObjective = static_cast<double>(debugsolobj);
    debugSolution.registerDomain(domain);
    assert(checkSolution(debugSolution.debugSolution));
  }
#endif

  if (upper_limit == kHighsInf) analyticCenterComputed = false;
  analyticCenterStatus = HighsModelStatus::kNotset;
  analyticCenter.clear();

  symmetries.clear();

  if (numRestarts != 0)
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "\n");
}

double HighsMipSolverData::transformNewIntegerFeasibleSolution(
    const std::vector<double>& sol,
    const bool possibly_store_as_new_incumbent) {
  HighsSolution solution;
  solution.col_value = sol;
  solution.value_valid = true;
  // Perform primal postsolve to get the original column values
  postSolveStack.undoPrimal(*mipsolver.options_mip_, solution);
  // Determine the row values, as they aren't computed in primal
  // postsolve
  HighsStatus return_status =
      calculateRowValuesQuad(*mipsolver.orig_model_, solution);
  if (kAllowDeveloperAssert) assert(return_status == HighsStatus::kOk);
  bool allow_try_again = true;
try_again:

  // compute the objective value in the original space
  double bound_violation_ = 0;
  double row_violation_ = 0;
  double integrality_violation_ = 0;
  HighsCDouble mipsolver_quad_objective_value = 0;
  bool feasible = mipsolver.solutionFeasible(
      mipsolver.orig_model_, solution.col_value, &solution.row_value,
      bound_violation_, row_violation_, integrality_violation_,
      mipsolver_quad_objective_value);
  double mipsolver_objective_value = double(mipsolver_quad_objective_value);
  if (!feasible && allow_try_again) {
    // printf(
    //     "trying to repair sol that is violated by %.12g bounds, %.12g "
    //     "integrality, %.12g rows\n",
    //     bound_violation_, integrality_violation_, row_violation_);
    HighsLp fixedModel = *mipsolver.orig_model_;
    fixedModel.integrality_.clear();
    for (HighsInt i = 0; i != mipsolver.orig_model_->num_col_; ++i) {
      if (mipsolver.orig_model_->integrality_[i] == HighsVarType::kInteger) {
        double solval = std::round(solution.col_value[i]);
        fixedModel.col_lower_[i] = std::max(fixedModel.col_lower_[i], solval);
        fixedModel.col_upper_[i] = std::min(fixedModel.col_upper_[i], solval);
      }
    }
    this->total_repair_lp++;
    double time_available = std::max(
        mipsolver.options_mip_->time_limit - mipsolver.timer_.read(), 0.1);
    Highs tmpSolver;
    const bool debug_report = false;
    if (debug_report) {
      tmpSolver.setOptionValue("log_dev_level", 2);
      tmpSolver.setOptionValue("highs_analysis_level", 4);
    } else {
      tmpSolver.setOptionValue("output_flag", false);
    }
    // tmpSolver.setOptionValue("simplex_scale_strategy", 0);
    // tmpSolver.setOptionValue("presolve", kHighsOffString);
    tmpSolver.setOptionValue("time_limit", time_available);
    // Set primal feasibility tolerance for LP solves according to
    // mip_feasibility_tolerance. Interestingly, dual feasibility
    // tolerance not set to smaller tolerance as in
    // HighsLpRelaxationconstructor.
    double mip_primal_feasibility_tolerance =
        mipsolver.options_mip_->mip_feasibility_tolerance;
    tmpSolver.setOptionValue("primal_feasibility_tolerance",
                             mip_primal_feasibility_tolerance);
    // check if only root presolve is allowed
    const bool use_presolve = !mipsolver.options_mip_->mip_root_presolve_only;
    const std::string presolve =
        use_presolve ? kHighsChooseString : kHighsOffString;
    tmpSolver.setOptionValue("presolve", presolve);
    tmpSolver.passModel(std::move(fixedModel));
    // Until a good decision can be made on whether to use simplex,
    // HiPO or IPX to solve an LP without a basis, use simplex
    tmpSolver.setOptionValue("solver", kSimplexString);
    tmpSolver.optimizeLp();
    if (!mipsolver.submip) {
      const HighsSubSolverCallTime& sub_solver_call_time =
          tmpSolver.getSubSolverCallTime();
      const bool analytic_centre = false;
      mipsolver.analysis_.addSubSolverCallTime(sub_solver_call_time,
                                               analytic_centre);
      // Go through sub_solver_call_time to update any MIP clocks
      const bool valid_basis = false;
      mipsolver.analysis_.mipTimerUpdate(sub_solver_call_time, valid_basis,
                                         use_presolve, analytic_centre);
    }
    this->total_repair_lp_iterations =
        tmpSolver.getInfo().simplex_iteration_count;
    if (tmpSolver.getInfo().primal_solution_status == kSolutionStatusFeasible) {
      this->total_repair_lp_feasible++;
      solution = tmpSolver.getSolution();
      allow_try_again = false;
      goto try_again;
    }
  }

  const double transformed_solobj =
      static_cast<double>(static_cast<HighsInt>(mipsolver.orig_model_->sense_) *
                              mipsolver_quad_objective_value -
                          mipsolver.model_->offset_);

  // Possible MIP solution callback
  if (!mipsolver.submip && feasible && mipsolver.callback_->user_callback &&
      mipsolver.callback_->active[kCallbackMipSolution]) {
    mipsolver.callback_->clearHighsCallbackOutput();
    mipsolver.callback_->data_out.mip_solution = solution.col_value;
    const bool interrupt = interruptFromCallbackWithData(
        kCallbackMipSolution, mipsolver_objective_value, "Feasible solution");
    assert(!interrupt);
  }

  // Catch the case where the repaired solution now has worse objective
  // than the current stored solution
  if (transformed_solobj >= upper_bound && !sol.empty()) {
    return transformed_solobj;
  }

  if (possibly_store_as_new_incumbent) {
    // Store the solution as incumbent in the original space if there
    // is no solution or if it is feasible
    if (feasible) {
      // if (!allow_try_again)
      //   printf("repaired solution with value %g\n",
      //   mipsolver_objective_value);
      // store
      mipsolver.row_violation_ = row_violation_;
      mipsolver.bound_violation_ = bound_violation_;
      mipsolver.integrality_violation_ = integrality_violation_;
      mipsolver.solution_ = std::move(solution.col_value);
      mipsolver.solution_objective_ = mipsolver_objective_value;
    } else {
      bool currentFeasible =
          mipsolver.solution_objective_ != kHighsInf &&
          mipsolver.bound_violation_ <=
              mipsolver.options_mip_->mip_feasibility_tolerance &&
          mipsolver.integrality_violation_ <=
              mipsolver.options_mip_->mip_feasibility_tolerance &&
          mipsolver.row_violation_ <=
              mipsolver.options_mip_->mip_feasibility_tolerance;
      highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kWarning,
                   "Solution with objective %g has untransformed violations: "
                   "bound = %.4g; integrality = %.4g; row = %.4g\n",
                   mipsolver_objective_value, bound_violation_,
                   integrality_violation_, row_violation_);
      if (!currentFeasible) {
        // if the current incumbent is non existent or also not feasible we
        // still store the new one
        mipsolver.row_violation_ = row_violation_;
        mipsolver.bound_violation_ = bound_violation_;
        mipsolver.integrality_violation_ = integrality_violation_;
        mipsolver.solution_ = std::move(solution.col_value);
        mipsolver.solution_objective_ = mipsolver_objective_value;
      }

      // return infinity so that it is not used for bounding
      return kHighsInf;
    }
  }

  // return the objective value in the transformed space
  return transformed_solobj;
}

double HighsMipSolverData::percentageInactiveIntegers() const {
  return 100.0 *
         (1.0 - static_cast<double>(integer_cols.size() -
                                    cliquetable.getSubstitutions().size()) /
                    numintegercols);
}

void HighsMipSolverData::performRestart() {
  HighsBasis root_basis;
  HighsPseudocostInitialization pscostinit(
      pseudocost, mipsolver.options_mip_->mip_pscost_minreliable,
      postSolveStack);

  mipsolver.pscostinit = &pscostinit;
  ++numRestarts;
  num_leaves_before_run = num_leaves;
  num_nodes_before_run = num_nodes;
  num_nodes_before_run = num_nodes;
  total_lp_iterations_before_run = total_lp_iterations;
  heuristic_lp_iterations_before_run = heuristic_lp_iterations;
  sepa_lp_iterations_before_run = sepa_lp_iterations;
  sb_lp_iterations_before_run = sb_lp_iterations;
  HighsInt numLpRows = lp.getLp().num_row_;
  HighsInt numModelRows = mipsolver.numRow();
  HighsInt numCuts = numLpRows - numModelRows;
  if (numCuts > 0) postSolveStack.appendCutsToModel(numCuts);
  auto integrality = std::move(presolvedModel.integrality_);
  double offset = presolvedModel.offset_;
  presolvedModel = lp.getLp();
  presolvedModel.offset_ = offset;
  presolvedModel.integrality_ = std::move(integrality);
#ifdef HIGHS_DEBUGSOL
  bool debugSolActive = false;
  std::swap(debugSolution.debugSolActive, debugSolActive);
#endif

  const HighsBasis& basis = firstrootbasis;
  if (basis.valid) {
    // if we have a basis after solving the root LP, we expand it to the
    // original space so that it can be used for constructing a starting basis
    // for the presolved model after the restart
    root_basis.col_status.resize(postSolveStack.getOrigNumCol());
    root_basis.row_status.resize(postSolveStack.getOrigNumRow(),
                                 HighsBasisStatus::kBasic);
    root_basis.valid = true;
    root_basis.useful = true;

    for (HighsInt i = 0; i < mipsolver.numCol(); ++i)
      root_basis.col_status[postSolveStack.getOrigColIndex(i)] =
          basis.col_status[i];

    HighsInt numRow = basis.row_status.size();
    for (HighsInt i = 0; i < numRow; ++i)
      root_basis.row_status[postSolveStack.getOrigRowIndex(i)] =
          basis.row_status[i];

    mipsolver.rootbasis = &root_basis;
  }

  // Transform the reference of the objective limit and lower/upper
  // bounds to the original model, since offset will generally change
  // in presolve. Bound changes are transitory, so no real gap change,
  // and no update to P-D integral is necessary
  upper_limit += mipsolver.model_->offset_;
  optimality_limit += mipsolver.model_->offset_;

  upper_bound += mipsolver.model_->offset_;
  lower_bound += mipsolver.model_->offset_;

  // remove the current incumbent. Any incumbent is already transformed into the
  // original space and kept there
  incumbent.clear();
  pruned_treeweight = 0;
  nodequeue.clear();
  globalOrbits.reset();

  // Need to be able to set presolve reduction limit separately when
  // restarting - so that bugs in presolve restart can be investigated
  // independently (see #1553)
  //
  // However, when restarting, presolve is (naturally) applied to the
  // presolved problem, so have to control the number of _further_
  // presolve reductions
  //
  // The number of further presolve reductions must be positive,
  // otherwise the MIP solver cycles, hence
  // restart_presolve_reduction_limit cannot be zero
  //
  // Although postSolveStack.numReductions() is size_t, it makes no
  // sense to use presolve_reduction_limit when the number of
  // reductions is vast
  HighsInt num_reductions = HighsInt(postSolveStack.numReductions());
  HighsInt restart_presolve_reduction_limit =
      mipsolver.options_mip_->restart_presolve_reduction_limit;
  assert(restart_presolve_reduction_limit);
  HighsInt further_presolve_reduction_limit =
      restart_presolve_reduction_limit >= 0
          ? num_reductions + restart_presolve_reduction_limit
          : -1;
  runMipPresolve(further_presolve_reduction_limit);

  if (mipsolver.modelstatus_ != HighsModelStatus::kNotset) {
    // transform the objective limit to the current model
    upper_limit -= mipsolver.model_->offset_;
    optimality_limit -= mipsolver.model_->offset_;

    if (mipsolver.modelstatus_ == HighsModelStatus::kOptimal) {
      mipsolver.mipdata_->upper_bound = 0;
      mipsolver.mipdata_->transformNewIntegerFeasibleSolution(
          std::vector<double>());
    } else {
      upper_bound -= mipsolver.model_->offset_;
    }

    // lower_bound still relates to the original model, and the offset
    // is never applied, since MIP solving is complete, and
    // lower_bound is set to upper_bound, so apply the offset now, so
    // that housekeeping in updatePrimalDualIntegral is correct
    lower_bound -= mipsolver.model_->offset_;

    // There must be a gap change, since it's now zero, so always call
    // updatePrimalDualIntegral (unless solving a sub-MIP)
    //
    // Surely there must be a lower bound change
    updateLowerBound(upper_bound);

    if (mipsolver.solution_objective_ != kHighsInf &&
        mipsolver.modelstatus_ == HighsModelStatus::kInfeasible)
      mipsolver.modelstatus_ = HighsModelStatus::kOptimal;
    return;
  }
  // Bounds are currently in the original space since presolve will have
  // changed offset_
#ifdef HIGHS_DEBUGSOL
  debugSolution.debugSolActive = debugSolActive;
#endif
  runSetup();
  if (mipsolver.terminate()) return;

  postSolveStack.removeCutsFromModel(numCuts);

  // HighsNodeQueue oldNodeQueue;
  // std::swap(nodequeue, oldNodeQueue);

  // remove the pointer into the stack-space of this function
  if (mipsolver.rootbasis == &root_basis) mipsolver.rootbasis = nullptr;
  mipsolver.pscostinit = nullptr;
}

void HighsMipSolverData::basisTransfer() {
  // if a root basis is given, construct a basis for the root LP from
  // in the reduced problem space after presolving
  if (mipsolver.rootbasis) {
    const HighsInt numRow = mipsolver.numRow();
    const HighsInt numCol = mipsolver.numCol();
    firstrootbasis.col_status.assign(numCol, HighsBasisStatus::kNonbasic);
    firstrootbasis.row_status.assign(numRow, HighsBasisStatus::kNonbasic);
    firstrootbasis.valid = true;
    firstrootbasis.alien = true;
    firstrootbasis.useful = true;

    for (HighsInt i = 0; i < numRow; ++i) {
      HighsBasisStatus status =
          mipsolver.rootbasis->row_status[postSolveStack.getOrigRowIndex(i)];
      firstrootbasis.row_status[i] = status;
    }

    for (HighsInt i = 0; i < numCol; ++i) {
      HighsBasisStatus status =
          mipsolver.rootbasis->col_status[postSolveStack.getOrigColIndex(i)];
      firstrootbasis.col_status[i] = status;
    }
  }
}

const std::vector<double>& HighsMipSolverData::getSolution() const {
  return incumbent;
}

bool HighsMipSolverData::addIncumbent(const std::vector<double>& sol,
                                      double solobj, const int solution_source,
                                      const bool print_display_line,
                                      const bool is_user_solution) {
  const bool execute_mip_solution_callback =
      !is_user_solution && !mipsolver.submip &&
      (mipsolver.callback_->user_callback
           ? mipsolver.callback_->active[kCallbackMipSolution]
           : false);
  // Determine whether the potential new incumbent should be
  // transformed
  //
  // Happens if solobj improves on the upper bound or the MIP solution
  // callback is active
  const bool possibly_store_as_new_incumbent = solobj < upper_bound;
  const bool get_transformed_solution =
      possibly_store_as_new_incumbent || execute_mip_solution_callback;
  // Get the transformed objective and solution if required
  const double transformed_solobj =
      get_transformed_solution ? transformNewIntegerFeasibleSolution(
                                     sol, possibly_store_as_new_incumbent)
                               : 0;
  const bool highs_solution_report = false;
  if (solution_source == kSolutionSourceHighsSolution && highs_solution_report
      //&& possibly_store_as_new_incumbent
  ) {
    std::stringstream ss;
    ss.str(std::string());
    ss << highsFormatToString(
        "HighsMipSolverData::addIncumbent HiGHS solution Obj "
        "= %15.8g; UB = %15.8g; Obj-UB = %11.4g; PossAdd = %s",
        solobj, upper_bound, solobj - upper_bound,
        possibly_store_as_new_incumbent ? "T" : "F");
    if (possibly_store_as_new_incumbent)
      ss << highsFormatToString(
          "; TransObj = %15.8g; TransObj-UB = %11.4g; TransSolobj < UB %s",
          transformed_solobj, transformed_solobj - upper_bound,
          transformed_solobj < upper_bound ? "T" : "F");
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "%s\n", ss.str().c_str());
    fflush(stdout);
  }
  if (possibly_store_as_new_incumbent) {
    solobj = transformed_solobj;
    if (solobj >= upper_bound) return false;

    double prev_upper_bound = upper_bound;

    upper_bound = solobj;

    bool bound_change = upper_bound != prev_upper_bound;
    if (!mipsolver.submip && bound_change)
      updatePrimalDualIntegral(lower_bound, lower_bound, prev_upper_bound,
                               upper_bound);

    // Assigning new incumbent
    incumbent = sol;
    double new_upper_limit = computeNewUpperLimit(solobj, 0.0, 0.0);

    if (!is_user_solution && !mipsolver.submip)
      saveReportMipSolution(new_upper_limit);
    if (new_upper_limit < upper_limit) {
      ++numImprovingSols;
      upper_limit = new_upper_limit;
      optimality_limit =
          computeNewUpperLimit(solobj, mipsolver.options_mip_->mip_abs_gap,
                               mipsolver.options_mip_->mip_rel_gap);
      nodequeue.setOptimalityLimit(optimality_limit);
      hacdcpfLogHighsTimeline(*this, "incumbent_accept", lower_bound,
                               prev_upper_bound, 0.0, solution_source);
      debugSolution.newIncumbentFound();
      domain.propagate();
      hacdcpfLogHighsTimeline(*this, "incumbent_after_domain", lower_bound,
                               prev_upper_bound, 0.0, solution_source);
      if (!domain.infeasible()) redcostfixing.propagateRootRedcost(mipsolver);
      hacdcpfLogHighsTimeline(*this, "incumbent_after_redcost", lower_bound,
                               prev_upper_bound, 0.0, solution_source);

      // Two calls to printDisplayLine added for completeness,
      // ensuring that when the root node has an integer solution, a
      // logging line is issued

      if (domain.infeasible()) {
        pruned_treeweight = 1.0;
        nodequeue.clear();
        if (print_display_line)
          printDisplayLine(solution_source);  // Added for completeness
        return true;
      }
      cliquetable.extractObjCliques(mipsolver);
      hacdcpfLogHighsConformanceState(*this, "incumbent_after_obj_cliques");
      hacdcpfLogHighsTimeline(*this, "incumbent_after_obj_cliques",
                               lower_bound, prev_upper_bound, 0.0,
                               solution_source);
      if (domain.infeasible()) {
        pruned_treeweight = 1.0;
        nodequeue.clear();
        if (print_display_line)
          printDisplayLine(solution_source);  // Added for completeness
        return true;
      }
      const double bound_pruned = nodequeue.performBounding(upper_limit);
      pruned_treeweight += bound_pruned;
      hacdcpfLogHighsTimeline(*this, "incumbent_after_queue_bound",
                               lower_bound, prev_upper_bound, bound_pruned,
                               solution_source);
      printDisplayLine(solution_source);
    }
  } else if (incumbent.empty())
    // Assigning new incumbent
    incumbent = sol;

  return true;
}

static std::array<char, 22> convertToPrintString(int64_t val) {
  decltype(convertToPrintString(std::declval<int64_t>())) printString = {};
  double l = std::log10(std::max(1.0, double(val)));
  switch (int(l)) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      std::snprintf(printString.data(), printString.size(), "%" PRId64, val);
      break;
    case 6:
    case 7:
    case 8:
      std::snprintf(printString.data(), printString.size(), "%" PRId64 "k",
                    val / 1000);
      break;
    default:
      std::snprintf(printString.data(), printString.size(), "%" PRId64 "m",
                    val / 1000000);
  }

  return printString;
}

static std::array<char, 22> convertToPrintString(double val,
                                                 const char* trailingStr = "") {
  decltype(convertToPrintString(std::declval<double>(),
                                std::declval<char*>())) printString = {};
  double l = std::abs(val) == kHighsInf
                 ? 0.0
                 : std::log10(std::max(1e-6, std::abs(val)));
  switch (int(l)) {
    case 0:
    case 1:
    case 2:
    case 3:
      std::snprintf(printString.data(), printString.size(), "%.10g%s", val,
                    trailingStr);
      break;
    case 4:
      std::snprintf(printString.data(), printString.size(), "%.11g%s", val,
                    trailingStr);
      break;
    case 5:
      std::snprintf(printString.data(), printString.size(), "%.12g%s", val,
                    trailingStr);
      break;
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
      std::snprintf(printString.data(), printString.size(), "%.13g%s", val,
                    trailingStr);
      break;
    default:
      std::snprintf(printString.data(), printString.size(), "%.9g%s", val,
                    trailingStr);
  }

  return printString;
}

void HighsMipSolverData::printSolutionSourceKey() const {
  std::stringstream ss;
  // Last MipSolutionSource enum is kSolutionSourceCleanup - which is
  // not a solution source, but used to force the last logging line to
  // be printed
  const int last_enum = kSolutionSourceCount - 1;
  // Set the index of the last solution source to be printed in each
  // line of the key. Four or five can be printed, depending on the
  // lengths of the solution source strings in that line
  std::vector<int> limits = {4, 9, 14, last_enum};
  assert(last_enum > limits[limits.size() - 2]);

  ss.str(std::string());
  for (int k = 0; k < limits[0]; k++) {
    if (k == 0) {
      ss << "\nSrc: ";
    } else {
      ss << "; ";
    }
    ss << solutionSourceToString(k) << " => "
       << solutionSourceToString(k, false);
  }
  highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
               "%s;\n", ss.str().c_str());
  int to_line = limits.size() - 1;
  for (int line = 0; line < to_line; line++) {
    ss.str(std::string());
    for (int k = limits[line]; k < limits[line + 1]; k++) {
      if (k == limits[line]) {
        ss << "     ";
      } else {
        ss << "; ";
      }
      ss << solutionSourceToString(k) << " => "
         << solutionSourceToString(k, false);
    }
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "%s%s\n", ss.str().c_str(), line < to_line - 1 ? ";" : "");
  }
}

void HighsMipSolverData::printDisplayLine(const int solution_source) {
  // MIP logging method
  //
  // Note that if the original problem is a maximization, the cost
  // coefficients are negated so that the MIP solver only solves a
  // minimization. Hence, in preparing to print the display line, the
  // dual bound (lb) is always less than the primal bound (ub). When
  // printed, the sense of the optimization is applied so that the
  // values printed correspond to the original objective.

  // No point in computing all the logging values if logging is off
  bool output_flag = *mipsolver.options_mip_->log_options.output_flag;
  if (!output_flag) return;

  bool timeless_log = mipsolver.options_mip_->timeless_log;
  disptime = timeless_log ? disptime + 1 : mipsolver.timer_.read();
  if (solution_source == kSolutionSourceNone &&
      disptime - last_disptime <
          mipsolver.options_mip_->mip_min_logging_interval)
    return;
  last_disptime = disptime;
  std::string time_string =
      timeless_log ? "" : highsFormatToString(" %7.1fs", disptime);

  if (num_disp_lines % 20 == 0) {
    if (num_disp_lines == 0) printSolutionSourceKey();
    std::string work_string0 = timeless_log ? "   Work" : "      Work      ";
    std::string work_string1 = timeless_log ? "LpIters" : "LpIters     Time";
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 // clang-format off
	"\n        Nodes      |    B&B Tree     |            Objective Bounds              |  Dynamic Constraints | %s\n"
	  "Src  Proc. InQueue |  Leaves   Expl. | BestBound       BestSol              Gap |   Cuts   InLp Confl. | %s\n\n",
                 // clang-format on
                 work_string0.c_str(), work_string1.c_str());

    //"   %7s | %10s | %10s | %10s | %10s | %-15s | %-15s | %7s | %7s "
    //"| %8s | %8s\n",
    //"time", "open nodes", "nodes", "leaves", "lpiters", "dual bound",
    //"primal bound", "cutpool", "confl.", "gap", "explored");
  }

  ++num_disp_lines;

  auto print_nodes = convertToPrintString(num_nodes);
  auto queue_nodes = convertToPrintString(nodequeue.numActiveNodes());
  auto print_leaves = convertToPrintString(num_leaves - num_leaves_before_run);

  double explored = 100 * double(pruned_treeweight);

  double lb;
  double ub;
  double gap = limitsToGap(lower_bound, upper_bound, lb, ub);
  gap *= 1e2;
  if (mipsolver.options_mip_->objective_bound < ub)
    ub = mipsolver.options_mip_->objective_bound;

  auto print_lp_iters = convertToPrintString(total_lp_iterations);
  HighsInt dynamic_constraints_in_lp =
      lp.numRows() > 0 ? lp.numRows() - lp.getNumModelRows() : 0;
  if (upper_bound != kHighsInf) {
    std::array<char, 22> gap_string = {};
    if (gap >= 9999.)
      std::strcpy(gap_string.data(), "Large");
    else
      std::snprintf(gap_string.data(), gap_string.size(), "%.2f%%", gap);

    std::array<char, 22> ub_string;
    if (mipsolver.options_mip_->objective_bound < ub) {
      ub_string =
          convertToPrintString((int)mipsolver.orig_model_->sense_ * ub, "*");
    } else
      ub_string = convertToPrintString((int)mipsolver.orig_model_->sense_ * ub);

    auto lb_string =
        convertToPrintString((int)mipsolver.orig_model_->sense_ * lb);

    highsLogUser(
        mipsolver.options_mip_->log_options, HighsLogType::kInfo,
        // clang-format off
                 " %s %7s %7s   %7s %6.2f%%   %-15s %-15s %8s   %6" HIGHSINT_FORMAT " %6" HIGHSINT_FORMAT " %6" HIGHSINT_FORMAT "   %7s%s\n",
        // clang-format on
        solutionSourceToString(solution_source).c_str(), print_nodes.data(),
        queue_nodes.data(), print_leaves.data(), explored, lb_string.data(),
        ub_string.data(), gap_string.data(), cutpool.getNumCuts(),
        dynamic_constraints_in_lp, conflictPool.getNumConflicts(),
        print_lp_iters.data(), time_string.c_str());
  } else {
    std::array<char, 22> ub_string;
    if (mipsolver.options_mip_->objective_bound < ub) {
      ub_string =
          convertToPrintString((int)mipsolver.orig_model_->sense_ * ub, "*");
    } else
      ub_string = convertToPrintString((int)mipsolver.orig_model_->sense_ * ub);

    auto lb_string =
        convertToPrintString((int)mipsolver.orig_model_->sense_ * lb);

    highsLogUser(
        mipsolver.options_mip_->log_options, HighsLogType::kInfo,
        // clang-format off
        " %s %7s %7s   %7s %6.2f%%   %-15s %-15s %8.2f   %6" HIGHSINT_FORMAT " %6" HIGHSINT_FORMAT " %6" HIGHSINT_FORMAT "   %7s%s\n",
        // clang-format on
        solutionSourceToString(solution_source).c_str(), print_nodes.data(),
        queue_nodes.data(), print_leaves.data(), explored, lb_string.data(),
        ub_string.data(), gap, cutpool.getNumCuts(), dynamic_constraints_in_lp,
        conflictPool.getNumConflicts(), print_lp_iters.data(),
        time_string.c_str());
  }
  // Check that limitsToBounds yields the same values for the
  // dual_bound, primal_bound (modulo optimization sense) and
  // mip_rel_gap
  double dual_bound;
  double primal_bound;
  double mip_rel_gap;
  limitsToBounds(dual_bound, primal_bound, mip_rel_gap);
  mip_rel_gap *= 1e2;
  assert(dual_bound == (int)mipsolver.orig_model_->sense_ * lb);
  assert(primal_bound == (int)mipsolver.orig_model_->sense_ * ub);
  assert(gap == mip_rel_gap);

  // Possibly interrupt from MIP logging callback
  mipsolver.callback_->clearHighsCallbackOutput();
  const bool interrupt = interruptFromCallbackWithData(
      kCallbackMipLogging, mipsolver.solution_objective_, "MIP logging");
  assert(!interrupt);
}

bool HighsMipSolverData::rootSeparationRound(
    HighsSeparation& sepa, HighsInt& ncuts, HighsLpRelaxation::Status& status) {
  int64_t tmpLpIters = -lp.getNumLpIterations();
  ncuts = sepa.separationRound(domain, status);
  tmpLpIters += lp.getNumLpIterations();
  avgrootlpiters = lp.getAvgSolveIters();
  total_lp_iterations += tmpLpIters;
  sepa_lp_iterations += tmpLpIters;

  status = evaluateRootLp();
  if (status == HighsLpRelaxation::Status::kInfeasible) return true;

  const std::vector<double>& solvals = lp.getLpSolver().getSolution().col_value;

  if (!mipsolver.hacdcpf_skip_primal_heuristics &&
      (mipsolver.submip || incumbent.empty())) {
    heuristics.randomizedRounding(solvals);
    if (mipsolver.options_mip_->mip_heuristic_run_shifting)
      heuristics.shifting(solvals);
    heuristics.flushStatistics();
    status = evaluateRootLp();
    if (status == HighsLpRelaxation::Status::kInfeasible) return true;
  }

  return false;
}

HighsLpRelaxation::Status HighsMipSolverData::evaluateRootLp() {
  do {
    domain.propagate();

    if (globalOrbits && !domain.infeasible())
      globalOrbits->orbitalFixing(domain);

    if (domain.infeasible()) {
      updateLowerBound(std::min(kHighsInf, upper_bound));
      pruned_treeweight = 1.0;
      num_nodes += 1;
      num_leaves += 1;
      return HighsLpRelaxation::Status::kInfeasible;
    }

    bool lpBoundsChanged = false;
    if (!domain.getChangedCols().empty()) {
      lpBoundsChanged = true;
      removeFixedIndices();
      lp.flushDomain(domain);
    }

    bool lpWasSolved = false;
    HighsLpRelaxation::Status status;
    if (lpBoundsChanged ||
        lp.getLpSolver().getModelStatus() == HighsModelStatus::kNotset) {
      int64_t lpIters = -lp.getNumLpIterations();
      status = lp.resolveLp(&domain);
      lpIters += lp.getNumLpIterations();
      total_lp_iterations += lpIters;
      avgrootlpiters = lp.getAvgSolveIters();
      lpWasSolved = true;

      if (status == HighsLpRelaxation::Status::kUnbounded) {
        if (mipsolver.solution_.empty())
          mipsolver.modelstatus_ = HighsModelStatus::kUnboundedOrInfeasible;
        else
          mipsolver.modelstatus_ = HighsModelStatus::kUnbounded;

        pruned_treeweight = 1.0;
        num_nodes += 1;
        num_leaves += 1;
        return status;
      }

      if (status == HighsLpRelaxation::Status::kOptimal &&
          lp.getFractionalIntegers().empty() &&
          addIncumbent(lp.getLpSolver().getSolution().col_value,
                       lp.getObjective(), kSolutionSourceEvaluateNode)) {
        mipsolver.modelstatus_ = HighsModelStatus::kOptimal;
        updateLowerBound(upper_bound);
        pruned_treeweight = 1.0;
        num_nodes += 1;
        num_leaves += 1;
        return HighsLpRelaxation::Status::kInfeasible;
      }

        if (!mipsolver.hacdcpf_skip_primal_heuristics &&
          status == HighsLpRelaxation::Status::kOptimal &&
          mipsolver.options_mip_->mip_heuristic_run_zi_round)
        heuristics.ziRound(lp.getLpSolver().getSolution().col_value);

    } else
      status = lp.getStatus();

    if (status == HighsLpRelaxation::Status::kInfeasible) {
      updateLowerBound(std::min(kHighsInf, upper_bound));
      pruned_treeweight = 1.0;
      num_nodes += 1;
      num_leaves += 1;
      return status;
    }

    if (lp.unscaledDualFeasible(lp.getStatus())) {
      updateLowerBound(std::max(lp.getObjective(), lower_bound));

      if (lpWasSolved) {
        redcostfixing.addRootRedcost(mipsolver,
                                     lp.getLpSolver().getSolution().col_dual,
                                     lp.getObjective());
        if (upper_limit != kHighsInf)
          redcostfixing.propagateRootRedcost(mipsolver);
      }
    }

    if (lower_bound > optimality_limit) {
      pruned_treeweight = 1.0;
      num_nodes += 1;
      num_leaves += 1;
      return HighsLpRelaxation::Status::kInfeasible;
    }

    if (domain.getChangedCols().empty()) return status;
  } while (true);
}

static void clockOff(HighsMipAnalysis& analysis) {
  if (!analysis.analyse_mip_time) return;
  // Make sure that exactly one of the following clocks is running
  const int clock0_running =
      analysis.mipTimerRunning(kMipClockEvaluateRootNode0) ? 1 : 0;
  const int clock1_running =
      analysis.mipTimerRunning(kMipClockEvaluateRootNode1) ? 1 : 0;
  const int clock2_running =
      analysis.mipTimerRunning(kMipClockEvaluateRootNode2) ? 1 : 0;
  const bool one_running = clock0_running + clock1_running + clock2_running;
  if (!one_running)
    printf("HighsMipSolverData::clockOff Clocks running are (%d; %d; %d)\n",
           clock0_running, clock1_running, clock2_running);
  assert(one_running);
  if (clock0_running) analysis.mipTimerStop(kMipClockEvaluateRootNode0);
  if (clock1_running) analysis.mipTimerStop(kMipClockEvaluateRootNode1);
  if (clock2_running) analysis.mipTimerStop(kMipClockEvaluateRootNode2);
}

void HighsMipSolverData::evaluateRootNode() {
  const bool run_primal_heuristics = !mipsolver.hacdcpf_skip_primal_heuristics;
  const bool compute_analytic_centre =
      run_primal_heuristics && !mipsolver.hacdcpf_skip_analytic_center;
  if (!compute_analytic_centre) printf("NOT COMPUTING ANALYTIC CENTRE!\n");
  HighsInt maxSepaRounds = mipsolver.submip ? 5 : kHighsIInf;
  if (numRestarts == 0)
    maxSepaRounds =
        std::min(HighsInt(2 * std::sqrt(maxTreeSizeLog2)), maxSepaRounds);
  // [K6] Cap with user-provided limit (hacdcpf_max_root_sepa_rounds).
  if (mipsolver.options_mip_->hacdcpf_max_root_sepa_rounds < kHighsIInf)
    maxSepaRounds = std::min(maxSepaRounds,
                             mipsolver.options_mip_->hacdcpf_max_root_sepa_rounds);
  std::unique_ptr<SymmetryDetectionData> symData;
  highs::parallel::TaskGroup tg;
  HighsMipAnalysis& analysis = mipsolver.analysis_;
restart:
  analysis.mipTimerStart(kMipClockEvaluateRootNode0);

  if (detectSymmetries) {
    analysis.mipTimerStart(kMipClockStartSymmetryDetection);
    startSymmetryDetection(tg, symData);
    analysis.mipTimerStop(kMipClockStartSymmetryDetection);
  }
  if (compute_analytic_centre && !analyticCenterComputed) {
    if (analysis.analyse_mip_time)
      highsLogUser(
          mipsolver.options_mip_->log_options, HighsLogType::kInfo,
          "MIP-Timing: %11.2g - starting analytic centre calculation\n",
          mipsolver.timer_.read());
    analysis.mipTimerStart(kMipClockStartAnalyticCentreComputation);
    startAnalyticCenterComputation(tg);
    analysis.mipTimerStop(kMipClockStartAnalyticCentreComputation);
  }

  // lp.getLpSolver().setOptionValue(
  //     "dual_simplex_cost_perturbation_multiplier", 10.0);
  lp.setIterationLimit();
  lp.loadModel();
  domain.clearChangedCols();
  lp.setObjectiveLimit(upper_limit);

  updateLowerBound(std::max(lower_bound, domain.getObjectiveLowerBound()));

  printDisplayLine();

  // Possibly query existence of an external solution
  if (!mipsolver.submip)
    mipsolver.mipdata_->queryExternalSolution(
        mipsolver.solution_objective_,
        kExternalMipSolutionQueryOriginEvaluateRootNode0);

  // check if only root presolve is allowed
  if (firstrootbasis.valid)
    lp.getLpSolver().setBasis(firstrootbasis,
                              "HighsMipSolverData::evaluateRootNode");
  else if (mipsolver.options_mip_->mip_root_presolve_only)
    lp.getLpSolver().setOptionValue("presolve", kHighsOffString);
  else
    lp.getLpSolver().setOptionValue("presolve", kHighsOnString);
  if (mipsolver.options_mip_->highs_debug_level)
    lp.getLpSolver().setOptionValue("output_flag",
                                    mipsolver.options_mip_->output_flag);
  //  lp.getLpSolver().setOptionValue("log_dev_level", kHighsLogDevLevelInfo);
  //  lp.getLpSolver().setOptionValue("log_file",
  //  mipsolver.options_mip_->log_file);

  analysis.mipTimerStart(kMipClockEvaluateRootLp);
  HighsLpRelaxation::Status status = evaluateRootLp();
  analysis.mipTimerStop(kMipClockEvaluateRootLp);
  if (numRestarts == 0) firstrootlpiters = total_lp_iterations;

  lp.getLpSolver().setOptionValue("output_flag", false);
  lp.getLpSolver().setOptionValue("presolve", kHighsOffString);
  lp.getLpSolver().setOptionValue("parallel", kHighsOffString);

  auto apply_hacdcpf_root_user_cuts = [&](const char* event) -> HighsInt {
    if (mipsolver.callback_ == nullptr ||
        !mipsolver.callback_->hacdcpf_node_cut_callback ||
        !lp.scaledOptimal(status) || !lp.unscaledDualFeasible(status)) {
      return 0;
    }

    HighsInt total_accepted = 0;
    HighsInt callback_round = 0;
    while (true) {
      HighsCutSet usercuts;
      mipsolver.callback_->hacdcpf_node_cut_callback(
          event, &lp, &domain, &postSolveStack, &usercuts, num_nodes, 0,
          mipsolver.callback_->hacdcpf_node_cut_callback_data);
      if (usercuts.numCuts() == 0) break;
      if (usercuts.ARstart_.size() !=
              static_cast<std::size_t>(usercuts.numCuts() + 1) ||
          usercuts.lower_.size() !=
              static_cast<std::size_t>(usercuts.numCuts()) ||
          usercuts.upper_.size() !=
              static_cast<std::size_t>(usercuts.numCuts()) ||
          (!usercuts.hacdcpf_integral_.empty() &&
           usercuts.hacdcpf_integral_.size() !=
               static_cast<std::size_t>(usercuts.numCuts())) ||
          (!usercuts.hacdcpf_propagate_.empty() &&
           usercuts.hacdcpf_propagate_.size() !=
               static_cast<std::size_t>(usercuts.numCuts())) ||
          (!usercuts.hacdcpf_validity_scope_.empty() &&
           usercuts.hacdcpf_validity_scope_.size() !=
               static_cast<std::size_t>(usercuts.numCuts())) ||
          (!usercuts.hacdcpf_proof_family_.empty() &&
           usercuts.hacdcpf_proof_family_.size() !=
               static_cast<std::size_t>(usercuts.numCuts())) ||
          (!usercuts.hacdcpf_proof_key_.empty() &&
           usercuts.hacdcpf_proof_key_.size() !=
               static_cast<std::size_t>(usercuts.numCuts()))) {
        highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kError,
                     "HACDCPF root cut callback returned malformed cutset\n");
        break;
      }
      const HacdcpfUserCutAdmissionLimits admission_limits =
          hacdcpfGetUserCutAdmissionLimits(/*root_event=*/true);
      std::vector<HacdcpfUserCutAdmission> global_admissions;
      std::vector<HacdcpfUserCutAdmission> local_admissions;
      global_admissions.reserve(static_cast<std::size_t>(usercuts.numCuts()));
      local_admissions.reserve(static_cast<std::size_t>(usercuts.numCuts()));
      HighsCutSet localcuts;
      localcuts.ARstart_.push_back(0);
      for (HighsInt i = 0; i != usercuts.numCuts(); ++i) {
        if (usercuts.cutindices[static_cast<std::size_t>(i)] != -2) continue;
        HacdcpfUserCutAdmission admission = hacdcpfAdmitUserCutRow(
            usercuts, i, lp.getSolution().col_value, lp.numCols(), feastol,
            admission_limits);
        hacdcpfLogUserCutRow("ADD", "root", event, callback_round, num_nodes,
                             0, usercuts, i, i, -1, -1, admission.efficacy,
                             admission.violation, admission.len,
                             &lp.getSolution().col_value, admission.reason);
        if (!admission.admitted) continue;
        if (admission.scope == HighsCutSet::kHacdcpfLocalNodeCut) {
          if (hacdcpfLocalCutAlreadyPresent(lp.getLocalCuts(),
                                            admission.row_hash,
                                            admission.row_sig)) {
            hacdcpfLogUserCutRow("ADD", "root", event, callback_round,
                                 num_nodes, 0, usercuts, i, i, -2, -1,
                                 admission.efficacy, admission.violation,
                                 admission.len, &lp.getSolution().col_value,
                                 "duplicate_existing_root_local");
            continue;
          }
          local_admissions.push_back(admission);
        } else {
          global_admissions.push_back(admission);
        }
      }
      hacdcpfSortUserCutAdmissions(global_admissions);
      hacdcpfSortUserCutAdmissions(local_admissions);
      HighsCutSet global_added_rows;
      global_added_rows.ARstart_.push_back(0);
      HighsInt accepted_global = 0;
      for (const HacdcpfUserCutAdmission& admission : global_admissions) {
        if (admission_limits.max_global_cuts >= 0 &&
            accepted_global >= admission_limits.max_global_cuts) {
          hacdcpfLogUserCutRow("ADD", "root", event, callback_round,
                               num_nodes, 0, usercuts, admission.row,
                               admission.row, -1, -1, admission.efficacy,
                               admission.violation, admission.len,
                               &lp.getSolution().col_value,
                               "selection_budget_global");
          continue;
        }
        if (hacdcpfCutSetAlreadyHasGlobalRow(global_added_rows,
                                             admission.row_hash,
                                             admission.row_sig)) {
          hacdcpfLogUserCutRow("ADD", "root", event, callback_round,
                               num_nodes, 0, usercuts, admission.row,
                               admission.row, -1, -1, admission.efficacy,
                               admission.violation, admission.len,
                               &lp.getSolution().col_value,
                               "duplicate_selected_global");
          continue;
        }
        hacdcpfCopyUserCutRow(global_added_rows, usercuts, admission,
                              HighsCutSet::kHacdcpfGlobalCut);
        const HighsInt cutindex = cutpool.addCut(
            mipsolver, usercuts.ARindex_.data() + admission.start,
            usercuts.ARvalue_.data() + admission.start, admission.len,
            usercuts.upper_[static_cast<std::size_t>(admission.row)],
            /*integral=*/
            !usercuts.hacdcpf_integral_.empty() &&
                usercuts.hacdcpf_integral_
                        [static_cast<std::size_t>(admission.row)] != 0,
            /*propagate=*/
            usercuts.hacdcpf_propagate_.empty() ||
                usercuts.hacdcpf_propagate_
                        [static_cast<std::size_t>(admission.row)] != 0,
            /*extractCliques=*/false);
        hacdcpfLogUserCutRow("ADD", "root", event, callback_round, num_nodes,
                             0, usercuts, admission.row, admission.row,
                             cutindex, -1, admission.efficacy,
                             admission.violation, admission.len,
                             &lp.getSolution().col_value,
                             cutindex < 0 ? "duplicate_cutpool"
                                          : "accepted_global");
        if (cutindex >= 0) ++accepted_global;
      }
      HighsInt accepted_local = 0;
      for (const HacdcpfUserCutAdmission& admission : local_admissions) {
        if (admission_limits.max_local_cuts >= 0 &&
            accepted_local >= admission_limits.max_local_cuts) {
          hacdcpfLogUserCutRow("ADD", "root", event, callback_round,
                               num_nodes, 0, usercuts, admission.row,
                               admission.row, -2, -1, admission.efficacy,
                               admission.violation, admission.len,
                               &lp.getSolution().col_value,
                               "selection_budget_local");
          continue;
        }
        if (hacdcpfCutSetAlreadyHasLocalRow(localcuts, admission.row_hash,
                                            admission.row_sig)) {
          hacdcpfLogUserCutRow("ADD", "root", event, callback_round,
                               num_nodes, 0, usercuts, admission.row,
                               admission.row, -2, -1, admission.efficacy,
                               admission.violation, admission.len,
                               &lp.getSolution().col_value,
                               "duplicate_selected_local");
          continue;
        }
        if (hacdcpfCopyUserCutRow(localcuts, usercuts, admission,
                                  HighsCutSet::kHacdcpfLocalNodeCut)) {
          hacdcpfLogUserCutRow("ADD", "root", event, callback_round,
                               num_nodes, 0, usercuts, admission.row,
                               admission.row, -2, -1, admission.efficacy,
                               admission.violation, admission.len,
                               &lp.getSolution().col_value,
                               "accepted_root_local_node");
          ++accepted_local;
        }
      }
      HighsCutSet selectedcuts;
      cutpool.separate(lp.getSolution().col_value, domain, selectedcuts,
                       feastol, &mipsolver);
      if (localcuts.numCuts() == 0 && selectedcuts.numCuts() == 0) break;
      if (localcuts.numCuts() != 0) {
        total_accepted += localcuts.numCuts();
        const HighsInt pre_append_rows = lp.numRows();
        const HighsCutSet append_ledger_cuts = localcuts;
        lp.addLocalCuts(localcuts);
        for (HighsInt i = 0; i != append_ledger_cuts.numCuts(); ++i) {
          hacdcpfLogUserCutRow(
              "APPEND", "root", event, callback_round, num_nodes, 0,
              append_ledger_cuts, i, i, -2, pre_append_rows + i, 0.0, 0.0,
              0, &lp.getSolution().col_value,
              "appended_root_local_node_cut");
        }
      }
      if (selectedcuts.numCuts() != 0) {
        total_accepted += selectedcuts.numCuts();
        const HighsInt pre_append_rows = lp.numRows();
        const HighsCutSet append_ledger_cuts = selectedcuts;
        lp.addCuts(selectedcuts);
        for (HighsInt i = 0; i != append_ledger_cuts.numCuts(); ++i) {
          hacdcpfLogUserCutRow(
              "APPEND", "root", event, callback_round, num_nodes, 0,
              append_ledger_cuts, i, i, append_ledger_cuts.cutindices[i],
              pre_append_rows + i, 0.0, 0.0, 0, &lp.getSolution().col_value,
              "appended_by_highs_cutpool");
        }
      }
      analysis.mipTimerStart(kMipClockEvaluateRootLp);
      const auto resolve_t0 = std::chrono::steady_clock::now();
      const int64_t iter_before = total_lp_iterations;
      status = evaluateRootLp();
      const double resolve_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - resolve_t0)
              .count();
      analysis.mipTimerStop(kMipClockEvaluateRootLp);
      lp.logHacdcpfUserCutResolveLedger(
          "root", event, callback_round, num_nodes, 0, status,
          total_lp_iterations - iter_before, total_lp_iterations, resolve_ms);
      if (status == HighsLpRelaxation::Status::kInfeasible) return -1;
      if (!lp.scaledOptimal(status)) break;
      if (++callback_round >= 100) {
        highsLogUser(mipsolver.options_mip_->log_options,
                     HighsLogType::kWarning,
                     "HACDCPF root cut callback reached round limit\n");
        break;
      }
    }
    return total_accepted;
  };

  if (status == HighsLpRelaxation::Status::kInfeasible ||
      status == HighsLpRelaxation::Status::kUnbounded)
    return clockOff(analysis);
  if (apply_hacdcpf_root_user_cuts("root_lp_optimal") < 0)
    return clockOff(analysis);

  firstlpsol = lp.getSolution().col_value;
  firstlpsolobj = lp.getObjective();
  rootlpsolobj = firstlpsolobj;

  if (lp.getLpSolver().getBasis().valid && lp.numRows() == mipsolver.numRow())
    firstrootbasis = lp.getLpSolver().getBasis();
  else {
    // the root basis is later expected to be consistent for the model without
    // cuts so set it to the slack basis if the current basis already includes
    // cuts, e.g. due to a restart
    firstrootbasis.col_status.assign(mipsolver.numCol(),
                                     HighsBasisStatus::kNonbasic);
    firstrootbasis.row_status.assign(mipsolver.numRow(),
                                     HighsBasisStatus::kBasic);
    firstrootbasis.valid = true;
    firstrootbasis.useful = true;
  }
  hacdcpfLogHighsFrontierConformance(*this, lp, "root_lp_initial");

  if (numRestarts != 0 && cutpool.getNumCuts() != 0) {
    HighsCutSet cutset;
    analysis.mipTimerStart(kMipClockSeparateLpCuts);
    cutpool.separateLpCutsAfterRestart(cutset, &mipsolver);
    analysis.mipTimerStop(kMipClockSeparateLpCuts);
#ifdef HIGHS_DEBUGSOL
    for (HighsInt i = 0; i < cutset.numCuts(); ++i) {
      debugSolution.checkCut(cutset.ARindex_.data() + cutset.ARstart_[i],
                             cutset.ARvalue_.data() + cutset.ARstart_[i],
                             cutset.ARstart_[i + 1] - cutset.ARstart_[i],
                             cutset.upper_[i]);
    }
#endif
    lp.addCuts(cutset);
    analysis.mipTimerStart(kMipClockEvaluateRootLp);
    status = evaluateRootLp();
    analysis.mipTimerStop(kMipClockEvaluateRootLp);
    lp.removeObsoleteRows();
    if (status == HighsLpRelaxation::Status::kInfeasible)
      return clockOff(analysis);
  }
  hacdcpfLogHighsFrontierConformance(*this, lp, "after_restart_cuts");

  lp.setIterationLimit(std::max(10000, int(10 * avgrootlpiters)));

  // make sure first line after solving root LP is printed
  last_disptime = -kHighsInf;
  disptime = 0;

  if (run_primal_heuristics) {
    if (mipsolver.options_mip_->mip_heuristic_run_zi_round)
      heuristics.ziRound(firstlpsol);
    analysis.mipTimerStart(kMipClockRandomizedRounding);
    heuristics.randomizedRounding(firstlpsol);
    analysis.mipTimerStop(kMipClockRandomizedRounding);
    if (mipsolver.options_mip_->mip_heuristic_run_shifting)
      heuristics.shifting(firstlpsol);

    heuristics.flushStatistics();

    analysis.mipTimerStart(kMipClockEvaluateRootLp);
    status = evaluateRootLp();
    analysis.mipTimerStop(kMipClockEvaluateRootLp);
    if (status == HighsLpRelaxation::Status::kInfeasible)
      return clockOff(analysis);
    hacdcpfLogHighsFrontierConformance(*this, lp, "after_initial_heuristics");
  }

  rootlpsolobj = firstlpsolobj;
  removeFixedIndices();
  if (mipsolver.options_mip_->mip_allow_restart &&
      mipsolver.options_mip_->presolve != kHighsOffString) {
    double fixingRate = percentageInactiveIntegers();
    if (fixingRate >= 10.0) {
      tg.cancel();
      highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                   "\n%.1f%% inactive integer columns, restarting\n",
                   fixingRate);
      tg.taskWait();
      analysis.mipTimerStart(kMipClockPerformRestart);
      performRestart();
      analysis.mipTimerStop(kMipClockPerformRestart);
      ++numRestartsRoot;
      if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
        clockOff(analysis);
        goto restart;
      }

      return clockOff(analysis);
    }
  }

  // begin separation
  if (analysis.analyse_mip_time) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "MIP-Timing: %11.2g - starting  separation\n",
                 analysis.mip_clocks.timer_pointer_->read(0));
    fflush(stdout);
  }
  analysis.mipTimerStart(kMipClockRootSeparation);
  std::vector<double> avgdirection;
  std::vector<double> curdirection;
  avgdirection.resize(mipsolver.numCol());
  curdirection.resize(mipsolver.numCol());

  HighsInt stall = 0;
  double smoothprogress = 0.0;
  HighsInt nseparounds = 0;
  HighsSeparation sepa(mipsolver);
  sepa.setLpRelaxation(&lp);
  const bool hacdcpf_trace_root_round = hacdcpfHighsRootRoundTraceEnabled();

  while (lp.scaledOptimal(status) && !lp.getFractionalIntegers().empty() &&
         stall < 3) {
    const HighsInt frac_before =
        static_cast<HighsInt>(lp.getFractionalIntegers().size());
    const double round_obj_before = lp.getObjective();
    printDisplayLine();

    if (checkLimits()) {
      analysis.mipTimerStop(kMipClockRootSeparation);
      return clockOff(analysis);
    }

    if (nseparounds == maxSepaRounds) break;

    removeFixedIndices();

    if (!mipsolver.submip &&
        mipsolver.options_mip_->presolve != kHighsOffString) {
      double fixingRate = percentageInactiveIntegers();
      if (fixingRate >= 10.0) {
        stall = -1;
        break;
      }
    }

    ++nseparounds;

    HighsInt ncuts;

    analysis.mipTimerStart(kMipClockRootSeparationRound);
    bool root_separation_round_result = false;
    {
      HacdcpfRootLedgerScopeGuard hacdcpf_root_ledger_scope(
          lp, cutpool, !mipsolver.submip);
      root_separation_round_result = rootSeparationRound(sepa, ncuts, status);
    }
    analysis.mipTimerStop(kMipClockRootSeparationRound);
    if (root_separation_round_result) {
      analysis.mipTimerStop(kMipClockRootSeparation);
      return clockOff(analysis);
    }
    hacdcpfLogHighsFrontierConformance(*this, lp, "after_root_round");
    if (nseparounds >= 5 && !mipsolver.submip && !analyticCenterComputed &&
        compute_analytic_centre) {
      if (checkLimits()) {
        analysis.mipTimerStop(kMipClockRootSeparation);
        return clockOff(analysis);
      }
      analysis.mipTimerStart(
          kMipClockRootSeparationFinishAnalyticCentreComputation);
      finishAnalyticCenterComputation(tg);
      analysis.mipTimerStop(
          kMipClockRootSeparationFinishAnalyticCentreComputation);

      analysis.mipTimerStart(kMipClockRootSeparationCentralRounding);
      heuristics.centralRounding();
      analysis.mipTimerStop(kMipClockRootSeparationCentralRounding);

      heuristics.flushStatistics();

      if (checkLimits()) {
        analysis.mipTimerStop(kMipClockRootSeparation);
        return clockOff(analysis);
      }
      analysis.mipTimerStart(kMipClockRootSeparationEvaluateRootLp);
      status = evaluateRootLp();
      analysis.mipTimerStop(kMipClockRootSeparationEvaluateRootLp);
      if (status == HighsLpRelaxation::Status::kInfeasible) {
        analysis.mipTimerStop(kMipClockRootSeparation);
        return clockOff(analysis);
      }
    }

    HighsCDouble sqrnorm = 0.0;
    const auto& solvals = lp.getSolution().col_value;

    for (HighsInt i = 0; i != mipsolver.numCol(); ++i) {
      curdirection[i] = firstlpsol[i] - solvals[i];

      // if (mip.integrality_[i] == 2 && lp.getObjective() > firstobj &&
      //    std::abs(curdirection[i]) > 1e-6)
      //  pseudocost.addObservation(i, -curdirection[i],
      //                            lp.getObjective() - firstobj);

      sqrnorm += curdirection[i] * curdirection[i];
    }
#if 1
    double scale = double(1.0 / sqrt(sqrnorm));
    sqrnorm = 0.0;
    HighsCDouble dotproduct = 0.0;
    for (HighsInt i = 0; i != mipsolver.numCol(); ++i) {
      avgdirection[i] =
          (scale * curdirection[i] - avgdirection[i]) / nseparounds;
      sqrnorm += avgdirection[i] * avgdirection[i];
      dotproduct += avgdirection[i] * curdirection[i];
    }
#endif

    double progress = double(dotproduct / sqrt(sqrnorm));
    double nextprogress = smoothprogress;
    bool stall_hit_this_round = false;

    if (nseparounds == 1) {
      smoothprogress = progress;
      nextprogress = smoothprogress;
    } else {
      double alpha = 1.0 / 3.0;
      nextprogress = (1.0 - alpha) * smoothprogress + alpha * progress;

      if (nextprogress < smoothprogress * 1.01 &&
          (lp.getObjective() - firstlpsolobj) <=
              (rootlpsolobj - firstlpsolobj) * 1.001) {
        ++stall;
        stall_hit_this_round = true;
      } else {
        stall = 0;
      }
      smoothprogress = nextprogress;
    }

    const HighsInt frac_after =
        static_cast<HighsInt>(lp.getFractionalIntegers().size());
    if (hacdcpf_trace_root_round) {
      highsLogUser(
          mipsolver.options_mip_->log_options, HighsLogType::kInfo,
          "[HIGHS-ROOT-ROUND] round=%lld max=%lld frac=%lld->%lld "
          "ncuts=%lld obj=%.17g->%.17g first=%.17g prevRoot=%.17g "
          "progress=%.17g smooth=%.17g next=%.17g stall_hit=%d stall=%lld "
          "stop_ncuts=%d stop_frac=%d stop_stall=%d\n",
          static_cast<long long>(nseparounds),
          static_cast<long long>(maxSepaRounds),
          static_cast<long long>(frac_before),
          static_cast<long long>(frac_after), static_cast<long long>(ncuts),
          round_obj_before, lp.getObjective(), firstlpsolobj, rootlpsolobj,
          progress, smoothprogress, nextprogress, stall_hit_this_round ? 1 : 0,
          static_cast<long long>(stall), ncuts == 0 ? 1 : 0,
          frac_after == 0 ? 1 : 0, stall >= 3 ? 1 : 0);
    }
    if (!mipsolver.submip &&
        mipsolver.options_mip_->hacdcpf_root_oracle_stop_after_root_round) {
      analysis.mipTimerStop(kMipClockRootSeparation);
      return clockOff(analysis);
    }
    rootlpsolobj = lp.getObjective();
    lp.setIterationLimit(std::max(10000, int(10 * avgrootlpiters)));
    if (ncuts == 0) break;

    // Possibly query existence of an external solution
    if (!mipsolver.submip)
      mipsolver.mipdata_->queryExternalSolution(
          mipsolver.solution_objective_,
          kExternalMipSolutionQueryOriginEvaluateRootNode1);
  }
  analysis.mipTimerStop(kMipClockRootSeparation);
  if (analysis.analyse_mip_time) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "MIP-Timing: %11.2g - completed separation\n",
                 analysis.mip_clocks.timer_pointer_->read(0));
    fflush(stdout);
  }

  lp.setIterationLimit();
  analysis.mipTimerStart(kMipClockEvaluateRootLp);
  status = evaluateRootLp();
  analysis.mipTimerStop(kMipClockEvaluateRootLp);
  if (status == HighsLpRelaxation::Status::kInfeasible)
    return clockOff(analysis);

  rootlpsol = lp.getLpSolver().getSolution().col_value;
  rootlpsolobj = lp.getObjective();
  lp.setIterationLimit(std::max(10000, int(10 * avgrootlpiters)));
  hacdcpfLogHighsFrontierConformance(*this, lp, "after_root_separation");

  if (run_primal_heuristics &&
      mipsolver.options_mip_->mip_heuristic_run_zi_round) {
    heuristics.ziRound(firstlpsol);
    heuristics.flushStatistics();
  }
  if (run_primal_heuristics &&
      mipsolver.options_mip_->mip_heuristic_run_shifting) {
    heuristics.shifting(rootlpsol);
    heuristics.flushStatistics();
  }

  if (!analyticCenterComputed && compute_analytic_centre) {
    if (checkLimits()) return clockOff(analysis);

    analysis.mipTimerStart(kMipClockFinishAnalyticCentreComputation);
    finishAnalyticCenterComputation(tg);
    analysis.mipTimerStop(kMipClockFinishAnalyticCentreComputation);

    analysis.mipTimerStart(kMipClockRootCentralRounding);
    heuristics.centralRounding();
    analysis.mipTimerStop(kMipClockRootCentralRounding);

    heuristics.flushStatistics();

    // if there are new global bound changes we re-evaluate the LP and do one
    // more separation round
    if (checkLimits()) return clockOff(analysis);
    bool separate = !domain.getChangedCols().empty();
    analysis.mipTimerStart(kMipClockEvaluateRootLp);
    status = evaluateRootLp();
    analysis.mipTimerStop(kMipClockEvaluateRootLp);
    if (status == HighsLpRelaxation::Status::kInfeasible)
      return clockOff(analysis);
    hacdcpfLogHighsFrontierConformance(*this, lp, "after_root_reduced_cost");
    if (separate && lp.scaledOptimal(status)) {
      HighsInt ncuts;
      analysis.mipTimerStart(kMipClockRootSeparationRound0);
      bool root_separation_round_result = false;
      {
        HacdcpfRootLedgerScopeGuard hacdcpf_root_ledger_scope(
          lp, cutpool, !mipsolver.submip);
        root_separation_round_result = rootSeparationRound(sepa, ncuts, status);
      }
      analysis.mipTimerStop(kMipClockRootSeparationRound0);
      if (root_separation_round_result) return clockOff(analysis);
      ++nseparounds;
      printDisplayLine();
    }
  }

  printDisplayLine();
  // Possibly query existence of an external solution
  if (!mipsolver.submip)
    mipsolver.mipdata_->queryExternalSolution(
        mipsolver.solution_objective_,
        kExternalMipSolutionQueryOriginEvaluateRootNode2);

  // Possible cut extraction callback
  if (!mipsolver.submip && mipsolver.callback_->user_callback &&
      mipsolver.callback_->callbackActive(kCallbackMipGetCutPool))
    mipsolver.callbackGetCutPool();
  if (checkLimits()) return clockOff(analysis);

  analysis.mipTimerStop(kMipClockEvaluateRootNode0);
  analysis.mipTimerStart(kMipClockEvaluateRootNode1);
  do {
    if (!run_primal_heuristics) break;
    if (rootlpsol.empty()) break;
    if (upper_limit != kHighsInf && !moreHeuristicsAllowed()) break;

    if (mipsolver.options_mip_->mip_heuristic_run_root_reduced_cost) {
      analysis.mipTimerStart(kMipClockRootHeuristicsReducedCost);
      heuristics.rootReducedCost();
      analysis.mipTimerStop(kMipClockRootHeuristicsReducedCost);
      heuristics.flushStatistics();
    }

    if (checkLimits()) return clockOff(analysis);

    // if there are new global bound changes we re-evaluate the LP and do one
    // more separation round
    bool separate = !domain.getChangedCols().empty();
    analysis.mipTimerStart(kMipClockEvaluateRootLp);
    status = evaluateRootLp();
    analysis.mipTimerStop(kMipClockEvaluateRootLp);
    if (status == HighsLpRelaxation::Status::kInfeasible)
      return clockOff(analysis);
    hacdcpfLogHighsFrontierConformance(*this, lp, "after_root_rens");
    if (separate && lp.scaledOptimal(status)) {
      HighsInt ncuts;
      analysis.mipTimerStart(kMipClockRootSeparationRound1);
      bool root_separation_round_result = false;
      {
        HacdcpfRootLedgerScopeGuard hacdcpf_root_ledger_scope(
          lp, cutpool, !mipsolver.submip);
        root_separation_round_result = rootSeparationRound(sepa, ncuts, status);
      }
      analysis.mipTimerStop(kMipClockRootSeparationRound1);
      if (root_separation_round_result) return clockOff(analysis);
      ++nseparounds;
      printDisplayLine();
    }

    if (upper_limit != kHighsInf && !moreHeuristicsAllowed()) break;

    if (checkLimits()) return clockOff(analysis);
    if (mipsolver.options_mip_->mip_heuristic_run_rens) {
      analysis.mipTimerStart(kMipClockRootHeuristicsRens);
      heuristics.RENS(rootlpsol);
      analysis.mipTimerStop(kMipClockRootHeuristicsRens);
      heuristics.flushStatistics();
    }

    if (checkLimits()) return clockOff(analysis);
    // if there are new global bound changes we re-evaluate the LP and do one
    // more separation round
    separate = !domain.getChangedCols().empty();
    analysis.mipTimerStart(kMipClockEvaluateRootLp);
    status = evaluateRootLp();
    analysis.mipTimerStop(kMipClockEvaluateRootLp);
    if (status == HighsLpRelaxation::Status::kInfeasible)
      return clockOff(analysis);
    if (separate && lp.scaledOptimal(status)) {
      HighsInt ncuts;
      analysis.mipTimerStart(kMipClockRootSeparationRound2);
      bool root_separation_round_result = false;
      {
        HacdcpfRootLedgerScopeGuard hacdcpf_root_ledger_scope(
          lp, cutpool, !mipsolver.submip);
        root_separation_round_result = rootSeparationRound(sepa, ncuts, status);
      }
      analysis.mipTimerStop(kMipClockRootSeparationRound2);
      if (root_separation_round_result) return clockOff(analysis);
      ++nseparounds;

      printDisplayLine();
      // Possibly query existence of an external solution
      if (!mipsolver.submip)
        mipsolver.mipdata_->queryExternalSolution(
            mipsolver.solution_objective_,
            kExternalMipSolutionQueryOriginEvaluateRootNode3);
    }

    if (upper_limit != kHighsInf || mipsolver.submip) break;

    if (checkLimits()) return clockOff(analysis);
    analysis.mipTimerStart(kMipClockRootFeasibilityPump);
    heuristics.feasibilityPump();
    analysis.mipTimerStop(kMipClockRootFeasibilityPump);
    heuristics.flushStatistics();

    if (checkLimits()) return clockOff(analysis);
    analysis.mipTimerStart(kMipClockEvaluateRootLp);
    status = evaluateRootLp();
    analysis.mipTimerStop(kMipClockEvaluateRootLp);
    if (status == HighsLpRelaxation::Status::kInfeasible)
      return clockOff(analysis);
  } while (false);

  analysis.mipTimerStop(kMipClockEvaluateRootNode1);
  analysis.mipTimerStart(kMipClockEvaluateRootNode2);
  if (lower_bound > upper_limit) {
    mipsolver.modelstatus_ = HighsModelStatus::kOptimal;
    pruned_treeweight = 1.0;
    num_nodes += 1;
    num_leaves += 1;
    return clockOff(analysis);
  }

  // if there are new global bound changes we re-evaluate the LP and do one
  // more separation round
  bool separate = !domain.getChangedCols().empty();
  analysis.mipTimerStart(kMipClockEvaluateRootLp);
  {
    HacdcpfRootLedgerScopeGuard hacdcpf_root_ledger_scope(
        lp, cutpool, !mipsolver.submip);
    status = evaluateRootLp();
  }
  analysis.mipTimerStop(kMipClockEvaluateRootLp);
  if (status == HighsLpRelaxation::Status::kInfeasible)
    return clockOff(analysis);
  hacdcpfLogHighsFrontierConformance(*this, lp, "root_lp_final");
  if (separate && lp.scaledOptimal(status)) {
    HighsInt ncuts;
    analysis.mipTimerStart(kMipClockRootSeparationRound3);
    bool root_separation_round_result = false;
    {
      HacdcpfRootLedgerScopeGuard hacdcpf_root_ledger_scope(
          lp, cutpool, !mipsolver.submip);
      root_separation_round_result = rootSeparationRound(sepa, ncuts, status);
    }
    analysis.mipTimerStop(kMipClockRootSeparationRound3);
    if (root_separation_round_result) return clockOff(analysis);
    ++nseparounds;
    printDisplayLine();
  }

  // Possibly query existence of an external solution
  if (!mipsolver.submip)
    mipsolver.mipdata_->queryExternalSolution(
        mipsolver.solution_objective_,
        kExternalMipSolutionQueryOriginEvaluateRootNode4);

  removeFixedIndices();
  if (lp.getLpSolver().getBasis().valid) lp.removeObsoleteRows();
  hacdcpf_root_lp_basis.clear();
  if (!mipsolver.submip && lp.getLpSolver().getBasis().valid) {
    hacdcpf_root_lp_basis = lp.getLpSolver().getBasis();
  }
  rootlpsolobj = lp.getObjective();

  printDisplayLine();

  if (lower_bound <= upper_limit) {
    if (!mipsolver.submip && mipsolver.options_mip_->mip_allow_restart &&
        mipsolver.options_mip_->presolve != kHighsOffString) {
      if (!analyticCenterComputed && compute_analytic_centre) {
        analysis.mipTimerStart(kMipClockFinishAnalyticCentreComputation);
        finishAnalyticCenterComputation(tg);
        analysis.mipTimerStop(kMipClockFinishAnalyticCentreComputation);
      }
      double fixingRate = percentageInactiveIntegers();
      if (fixingRate >= 2.5 + 7.5 * mipsolver.submip ||
          (!mipsolver.submip && fixingRate > 0 && numRestarts == 0)) {
        tg.cancel();
        highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                     "\n%.1f%% inactive integer columns, restarting\n",
                     fixingRate);
        if (stall != -1) maxSepaRounds = std::min(maxSepaRounds, nseparounds);
        tg.taskWait();
        analysis.mipTimerStart(kMipClockPerformRestart);
        performRestart();
        analysis.mipTimerStop(kMipClockPerformRestart);
        if (mipsolver.terminate()) return;
        ++numRestartsRoot;
        if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
          clockOff(analysis);
          goto restart;
        }
        return clockOff(analysis);
      }
    }

    if (detectSymmetries) {
      finishSymmetryDetection(tg, symData);
      analysis.mipTimerStart(kMipClockEvaluateRootLp);
      status = evaluateRootLp();
      analysis.mipTimerStop(kMipClockEvaluateRootLp);
      if (status == HighsLpRelaxation::Status::kInfeasible)
        return clockOff(analysis);
    }

    // add the root node to the nodequeue to initialize the search
    std::vector<HacdcpfLocalCut> root_local_cuts = lp.getLocalCuts();
    nodequeue.emplaceNode(std::vector<HighsDomainChange>(),
                          std::vector<HighsInt>(), lower_bound,
                          lp.computeBestEstimate(pseudocost), 1,
                          std::move(root_local_cuts));
  }
  // End of HighsMipSolverData::evaluateRootNode()
  clockOff(analysis);
}

bool HighsMipSolverData::checkLimits(int64_t nodeOffset) const {
  const HighsOptions& options = *mipsolver.options_mip_;

  // This MIP instance may have been terminated
  if (terminatorActive())
    if (this->terminatorTerminated()) return true;

  // Possible user interrupt
  if (!mipsolver.submip && mipsolver.callback_->user_callback) {
    mipsolver.callback_->clearHighsCallbackOutput();
    if (interruptFromCallbackWithData(kCallbackMipInterrupt,
                                      mipsolver.solution_objective_,
                                      "MIP check limits")) {
      if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
        highsLogDev(options.log_options, HighsLogType::kInfo,
                    "User interrupt\n");
        mipsolver.modelstatus_ = HighsModelStatus::kInterrupt;
      }
      return true;
    }
  }
  // Possible termination due to objective being at least as good as
  // the target value
  if (!mipsolver.submip && mipsolver.solution_objective_ < kHighsInf &&
      options.objective_target > -kHighsInf) {
    // Note:
    //
    // Whether the sense is ObjSense::kMinimize or
    // ObjSense::kMaximize, the undefined value of
    // mipsolver.solution_objective_ is kHighsInf, and the default
    // target value is -kHighsInf, so had to rule out these cases in
    // the conditional statement above.
    //
    // mipsolver.solution_objective_ is the actual objective of the
    // MIP - including the offset, and independent of objective sense
    //
    // The target is reached if the objective is below (above) the
    // target value when minimizing (maximizing).
    const int int_sense = int(this->mipsolver.orig_model_->sense_);
    const bool reached_objective_target =
        int_sense * mipsolver.solution_objective_ <
        int_sense * options.objective_target;
    if (reached_objective_target) {
      if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
        highsLogDev(options.log_options, HighsLogType::kInfo,
                    "Reached objective target\n");
        mipsolver.modelstatus_ = HighsModelStatus::kObjectiveTarget;
      }
      return true;
    }
  }

  if (options.mip_max_nodes != kHighsIInf &&
      num_nodes + nodeOffset >= options.mip_max_nodes) {
    if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
      highsLogDev(options.log_options, HighsLogType::kInfo,
                  "Reached node limit\n");
      mipsolver.modelstatus_ = HighsModelStatus::kSolutionLimit;
    }
    return true;
  }

  if (options.mip_max_leaves != kHighsIInf &&
      num_leaves >= options.mip_max_leaves) {
    if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
      highsLogDev(options.log_options, HighsLogType::kInfo,
                  "Reached leaf node limit\n");
      mipsolver.modelstatus_ = HighsModelStatus::kSolutionLimit;
    }
    return true;
  }

  if (options.mip_max_improving_sols != kHighsIInf &&
      numImprovingSols >= options.mip_max_improving_sols) {
    if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
      highsLogDev(options.log_options, HighsLogType::kInfo,
                  "Reached improving solution limit\n");
      mipsolver.modelstatus_ = HighsModelStatus::kSolutionLimit;
    }
    return true;
  }

  //  const double time = mipsolver.timer_.read();
  //  printf("checkLimits: time = %g\n", time);
  if (options.time_limit < kHighsInf &&
      mipsolver.timer_.read() >= options.time_limit) {
    if (mipsolver.modelstatus_ == HighsModelStatus::kNotset) {
      highsLogDev(options.log_options, HighsLogType::kInfo,
                  "Reached time limit\n");
      mipsolver.modelstatus_ = HighsModelStatus::kTimeLimit;
    }
    return true;
  }

  return false;
}

void HighsMipSolverData::checkObjIntegrality() {
  objectiveFunction.checkIntegrality(epsilon);
  if (objectiveFunction.isIntegral() && numRestarts == 0) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "Objective function is integral with scale %g\n",
                 objectiveFunction.integralScale());
  }
}

void HighsMipSolverData::setupDomainPropagation() {
  const HighsLp& model = *mipsolver.model_;
  highsSparseTranspose(model.num_row_, model.num_col_, model.a_matrix_.start_,
                       model.a_matrix_.index_, model.a_matrix_.value_, ARstart_,
                       ARindex_, ARvalue_);

  pseudocost = HighsPseudocost(mipsolver);

  // compute the maximal absolute coefficients to filter propagation
  maxAbsRowCoef.resize(mipsolver.numRow());
  for (HighsInt i = 0; i != mipsolver.numRow(); ++i) {
    double maxabsval = 0.0;

    HighsInt start = ARstart_[i];
    HighsInt end = ARstart_[i + 1];
    for (HighsInt j = start; j != end; ++j)
      maxabsval = std::max(maxabsval, std::abs(ARvalue_[j]));

    maxAbsRowCoef[i] = maxabsval;
  }

  domain = HighsDomain(mipsolver);
  domain.computeRowActivities();
}

void HighsMipSolverData::saveReportMipSolution(const double new_upper_limit) {
  const bool non_improving = new_upper_limit >= upper_limit;
  if (mipsolver.submip) return;
  if (non_improving) return;

  if (mipsolver.callback_->user_callback) {
    if (mipsolver.callback_->active[kCallbackMipImprovingSolution]) {
      mipsolver.callback_->clearHighsCallbackOutput();
      mipsolver.callback_->data_out.mip_solution = mipsolver.solution_;
      const bool interrupt = interruptFromCallbackWithData(
          kCallbackMipImprovingSolution, mipsolver.solution_objective_,
          "Improving solution");
      assert(!interrupt);
    }
  }

  if (mipsolver.options_mip_->mip_improving_solution_save) {
    HighsObjectiveSolution record;
    record.objective = mipsolver.solution_objective_;
    record.col_value = mipsolver.solution_;
    mipsolver.saved_objective_and_solution_.push_back(record);
  }
  FILE* file = mipsolver.improving_solution_file_;
  if (file) {
    writeLpObjective(file, mipsolver.options_mip_->log_options,
                     *(mipsolver.orig_model_), mipsolver.solution_);
    writePrimalSolution(
        file, mipsolver.options_mip_->log_options, *(mipsolver.orig_model_),
        mipsolver.solution_,
        mipsolver.options_mip_->mip_improving_solution_report_sparse);
  }
}

void HighsMipSolverData::limitsToBounds(double& dual_bound,
                                        double& primal_bound,
                                        double& mip_rel_gap) const {
  mip_rel_gap = limitsToGap(lower_bound, upper_bound, dual_bound, primal_bound);
  primal_bound =
      std::min(mipsolver.options_mip_->objective_bound, primal_bound);
  // Adjust objective sense in case of maximization problem
  if (this->mipsolver.orig_model_->sense_ == ObjSense::kMaximize) {
    dual_bound = -dual_bound;
    primal_bound = -primal_bound;
  }
}

void HighsMipSolverData::updateLowerBound(double new_lower_bound) {
  // Update lower bound
  double prev_lower_bound = lower_bound;
  lower_bound = new_lower_bound;
  if (!mipsolver.submip && lower_bound != prev_lower_bound) {
    updatePrimalDualIntegral(prev_lower_bound, lower_bound, upper_bound,
                             upper_bound);
    hacdcpfLogHighsTimeline(*this, "lower_bound_update", prev_lower_bound,
                             upper_bound, 0.0);
  }
}

// Interface to callbackAction, with mipsolver_objective_value since
// incumbent value (mipsolver.solution_objective_) is not right for
// callback_type = kCallbackMipSolution

void HighsMipSolverData::setCallbackDataOut(
    const double mipsolver_objective_value) const {
  double dual_bound;
  double primal_bound;
  double mip_rel_gap;
  limitsToBounds(dual_bound, primal_bound, mip_rel_gap);
  mipsolver.callback_->data_out.running_time = mipsolver.timer_.read();
  mipsolver.callback_->data_out.objective_function_value =
      mipsolver_objective_value;
  mipsolver.callback_->data_out.mip_node_count = mipsolver.mipdata_->num_nodes;
  mipsolver.callback_->data_out.mip_total_lp_iterations =
      mipsolver.mipdata_->total_lp_iterations;
  mipsolver.callback_->data_out.mip_primal_bound = primal_bound;
  mipsolver.callback_->data_out.mip_dual_bound = dual_bound;
  mipsolver.callback_->data_out.mip_gap = mip_rel_gap;
}

bool HighsMipSolverData::interruptFromCallbackWithData(
    const int callback_type, const double mipsolver_objective_value,
    const std::string message) const {
  if (!mipsolver.callback_->callbackActive(callback_type)) return false;
  assert(!mipsolver.submip);
  setCallbackDataOut(mipsolver_objective_value);
  return mipsolver.callback_->callbackAction(callback_type, message);
}

void HighsMipSolverData::queryExternalSolution(
    const double mipsolver_objective_value,
    const ExternalMipSolutionQueryOrigin external_solution_query_origin) {
  assert(!mipsolver.submip);
  HighsCallback* callback = mipsolver.callback_;
  const bool use_callback =
      callback->user_callback && callback->active[kCallbackMipUserSolution];
  if (use_callback) {
    setCallbackDataOut(mipsolver_objective_value);
    callback->data_out.external_solution_query_origin =
        external_solution_query_origin;
    callback->clearHighsCallbackInput();

    const bool interrupt =
        callback->callbackAction(kCallbackMipUserSolution, "MIP User solution");
    assert(!interrupt);
    if (callback->data_in.user_has_solution) {
      // Objective is assumed to be original_offset +
      // (original_c)^T(original_x), but MIP solver bounds are based on the
      // reduced objective (reduced_c)^T(reduced_x)
      //
      // Now, original_sense*[reduced_offset + (reduced_c)^T(reduced_x)] is an
      // objective in the original space, so
      //
      // f0 + c0^Tx0 = s*(f1 + c1^Tx1)
      //
      // where 0 => original; 1 => reduced
      //
      // This allows the reduced objective value to be deduced as
      //
      // c1^Tx1 = s*(f0 + c0^Tx0) - f1
      //
      // (reduced_c)^T(reduced_x) = original_sense*[original_offset +
      // (original_c)^T(original_x) - reduced_offset]
      const auto& user_solution = callback->data_in.user_solution;
      double bound_violation_ = 0;
      double row_violation_ = 0;
      double integrality_violation_ = 0;
      HighsCDouble user_solution_quad_objective_value = 0;
      const bool feasible = mipsolver.solutionFeasible(
          mipsolver.orig_model_, user_solution, nullptr, bound_violation_,
          row_violation_, integrality_violation_,
          user_solution_quad_objective_value);
      double user_solution_objective_value =
          double(user_solution_quad_objective_value);
      if (!feasible) {
        highsLogUser(
            mipsolver.options_mip_->log_options, HighsLogType::kWarning,
            "User-supplied solution has with objective %g has violations: "
            "bound = %.4g; integrality = %.4g; row = %.4g\n",
            user_solution_objective_value, bound_violation_,
            integrality_violation_, row_violation_);
        return;
      }
      std::vector<double> reduced_user_solution;
      reduced_user_solution =
          postSolveStack.getReducedPrimalSolution(user_solution);
      const bool print_display_line = true;
      const bool is_user_solution = true;
      addIncumbent(reduced_user_solution, user_solution_objective_value,
                   kSolutionSourceUserSolution, print_display_line,
                   is_user_solution);
    }
  }
}

HighsInt HighsMipSolverData::terminatorConcurrency() const {
  return mipsolver.terminator_.num_instance;
}

HighsInt HighsMipSolverData::terminatorMyInstance() const {
  return mipsolver.terminator_.my_instance;
}

void HighsMipSolverData::terminatorTerminate() {
  assert(terminatorActive());
  mipsolver.terminator_.terminate();
}

bool HighsMipSolverData::terminatorTerminated() const {
  if (this->terminatorActive())
    mipsolver.termination_status_ = mipsolver.terminator_.terminationStatus();
  return mipsolver.termination_status_ != HighsModelStatus::kNotset;
}

void HighsMipSolverData::terminatorReport() const {
  if (this->terminatorActive())
    mipsolver.terminator_.report(mipsolver.options_mip_->log_options);
}

static double possInfRelDiff(const double v0, const double v1,
                             const double den) {
  double rel_diff;
  if (std::fabs(v0) == kHighsInf) {
    if (std::fabs(v1) == kHighsInf) {
      rel_diff = 0;
    } else {
      rel_diff = kHighsInf;
    }
  } else {
    if (std::fabs(v1) == kHighsInf) {
      rel_diff = kHighsInf;
    } else {
      rel_diff = std::fabs(v1 - v0) / std::max(1.0, std::fabs(den));
    }
  }
  return rel_diff;
}

void HighsMipSolverData::updatePrimalDualIntegral(const double from_lower_bound,
                                                  const double to_lower_bound,
                                                  const double from_upper_bound,
                                                  const double to_upper_bound,
                                                  const bool check_bound_change,
                                                  const bool check_prev_data) {
  // Parameters to updatePrimalDualIntegral are lower and upper bounds
  // before/after a change
  //
  // updatePrimalDualIntegral should only be called when there is a
  // change in one of the bounds, except when the final update is
  // made, in which case the bounds must NOT have changed. By default,
  // a check for some bound change is made, unless check_bound_change
  // is false, in which case there is a check for unchanged bounds.
  //
  HighsPrimaDualIntegral& pdi = this->primal_dual_integral;
  // HighsPrimaDualIntegral struct contains the following data
  //
  // * value: Current value of the P-D integral
  //
  // * prev_lb: Value of lb that was computed from to_lower_bound in
  //   the previous call. Used as a check that the value of lb
  //   computed from from_lower_bound in this call is equal - to
  //   within bound_change_tolerance. If not true, then a change in lb
  //   has been missed. Only for checking/debugging
  //
  // * prev_ub: Ditto for upper_bound. Only for checking/debugging
  //
  // * prev_gap: Ditto for gap. Only for checking/debugging
  //
  // * prev_time: Used to determine the time spent at the previous gap

  double from_lb;
  double from_ub;
  const double from_gap =
      this->limitsToGap(from_lower_bound, from_upper_bound, from_lb, from_ub);
  double to_lb;
  double to_ub;
  const double to_gap =
      this->limitsToGap(to_lower_bound, to_upper_bound, to_lb, to_ub);

  const double lb_difference = possInfRelDiff(from_lb, to_lb, to_lb);
  const double ub_difference = possInfRelDiff(from_ub, to_ub, to_ub);
  const double bound_change_tolerance = 0;
  const bool bound_change = lb_difference > bound_change_tolerance ||
                            ub_difference > bound_change_tolerance;

  if (check_bound_change) {
    if (!bound_change) {
      if (from_lower_bound == to_lower_bound &&
          from_upper_bound == to_upper_bound) {
        const double lower_bound_difference =
            possInfRelDiff(from_lower_bound, to_lower_bound, to_lower_bound);
        const double upper_bound_difference =
            possInfRelDiff(from_upper_bound, to_upper_bound, to_upper_bound);
        assert(bound_change);
      }
    }
  } else {
    if (bound_change) {
      if (from_lower_bound != to_lower_bound ||
          from_upper_bound != to_upper_bound) {
        const double lower_bound_difference =
            possInfRelDiff(from_lower_bound, to_lower_bound, to_lower_bound);
        const double upper_bound_difference =
            possInfRelDiff(from_upper_bound, to_upper_bound, to_upper_bound);
        assert(!bound_change);
      }
    }
  }
  if (pdi.value > -kHighsInf) {
    // updatePrimalDualIntegral has been called previously, so can
    // usually test housekeeping, even if gap is still inf
    //
    // The one case where the checking can't be done comes after restart, where
    // the
    //
    if (check_prev_data) {
      // These housekeeping tests check that the previous saved
      // lower/upper bounds and gap are very close to the "from"
      // lower/upper bounds and corresponding gap. They are usually
      // identical, but rounding error can occur when passing through
      // reset, when the old/new offsets are added/subtracted from the
      // bounds due to changes in offset during presolve.
      const double lb_inconsistency =
          possInfRelDiff(from_lb, pdi.prev_lb, pdi.prev_lb);
      const bool lb_consistent = lb_inconsistency < 1e-12;
      const double ub_inconsistency =
          possInfRelDiff(from_ub, pdi.prev_ub, pdi.prev_ub);
      const bool ub_consistent = ub_inconsistency < 1e-12;
      const double gap_inconsistency =
          possInfRelDiff(from_gap, pdi.prev_gap, 1.0);
      const bool gap_consistent = gap_inconsistency < 1e-12;
      assert(lb_consistent);
      assert(ub_consistent);
      assert(gap_consistent);
    }
    if (to_gap < kHighsInf) {
      double time = mipsolver.timer_.read();
      if (from_gap < kHighsInf) {
        // Need to update the P-D integral
        double time_diff = time - pdi.prev_time;
        assert(time_diff >= 0);
        pdi.value += time_diff * pdi.prev_gap;
      }
      pdi.prev_time = time;
    }
  } else {
    pdi.value = 0;
  }
  pdi.prev_lb = to_lb;
  pdi.prev_ub = to_ub;
  pdi.prev_gap = to_gap;
}

void HighsPrimaDualIntegral::initialise() { this->value = -kHighsInf; }

void HighsTerminator::clear() {
  this->num_instance = 0;
  this->my_instance = kNoThreadInstance;
  this->record = nullptr;
}

void HighsTerminator::initialise(HighsInt num_instance_, HighsInt my_instance_,
                                 HighsModelStatus* record_) {
  this->clear();
  this->num_instance = num_instance_;
  this->my_instance = my_instance_;
  this->record = record_;
}

HighsInt HighsTerminator::concurrency() const { return this->num_instance; }

void HighsTerminator::terminate() {
  assert(this->record);
  assert(this->my_instance < this->num_instance);
  this->record[this->my_instance] = HighsModelStatus::kHighsInterrupt;
}

HighsModelStatus HighsTerminator::terminationStatus() const {
  assert(this->record);
  for (HighsInt instance = 0; instance < this->num_instance; instance++) {
    if (this->record[instance] != HighsModelStatus::kNotset)
      return this->record[instance];
  }
  return HighsModelStatus::kNotset;
}

void HighsTerminator::report(const HighsLogOptions log_options) const {
  highsLogUser(log_options, HighsLogType::kInfo, "\nTerminator:        ");
  for (HighsInt instance = 0; instance < this->num_instance; instance++)
    highsLogUser(log_options, HighsLogType::kInfo, " %20d",
                 int(this->record[instance]));
  highsLogUser(log_options, HighsLogType::kInfo, "\n");
}
