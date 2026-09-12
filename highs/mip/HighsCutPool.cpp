/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "mip/HighsCutPool.h"

#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>

#include "../extern/pdqsort/pdqsort.h"
#include "lp_data/HighsCallback.h"
#include "mip/HighsDomain.h"
#include "mip/HighsLpRelaxation.h"
#include "mip/HighsMipSolverData.h"
#include "util/HighsCDouble.h"
#include "util/HighsHash.h"

static uint64_t compute_cut_hash(const HighsInt* Rindex, const double* Rvalue,
                                 double maxabscoef, const HighsInt Rlen) {
  if (Rlen <= 0 || !(maxabscoef > 0.0)) return uint64_t{0};
  std::vector<uint32_t> valueHashCodes(Rlen);

  double scale = 1.0 / maxabscoef;
  for (HighsInt i = 0; i < Rlen; ++i)
    valueHashCodes[i] = HighsHashHelpers::double_hash_code(scale * Rvalue[i]);

  return HighsHashHelpers::vector_hash(Rindex, Rlen) ^
         (HighsHashHelpers::vector_hash(valueHashCodes.data(), Rlen) >> 32);
}

static uint64_t hacdcpfTraceHashMix(uint64_t h, uint64_t v) {
  v ^= v >> 33;
  v *= uint64_t{0xff51afd7ed558ccd};
  v ^= v >> 33;
  v *= uint64_t{0xc4ceb9fe1a85ec53};
  v ^= v >> 33;
  return h ^ (v + uint64_t{0x9e3779b97f4a7c15} + (h << 6) + (h >> 2));
}

uint64_t hacdcpfCutRowHash(const HighsInt* Rindex, const double* Rvalue,
                           HighsInt Rlen) {
  double maxabscoef = 0.0;
  for (HighsInt i = 0; i != Rlen; ++i)
    maxabscoef = std::max(maxabscoef, std::abs(Rvalue[i]));
  return compute_cut_hash(Rindex, Rvalue, maxabscoef, Rlen);
}

uint64_t hacdcpfCutSetRowHash(const HighsCutSet& cutset, HighsInt row) {
  if (row < 0 || row + 1 >= static_cast<HighsInt>(cutset.ARstart_.size()))
    return uint64_t{0};
  const HighsInt start = cutset.ARstart_[static_cast<std::size_t>(row)];
  const HighsInt end = cutset.ARstart_[static_cast<std::size_t>(row + 1)];
  if (start < 0 || end < start ||
      end > static_cast<HighsInt>(cutset.ARindex_.size()) ||
      end > static_cast<HighsInt>(cutset.ARvalue_.size())) {
    return uint64_t{0};
  }
  return hacdcpfCutRowHash(cutset.ARindex_.data() + start,
                           cutset.ARvalue_.data() + start, end - start);
}

uint64_t hacdcpfCutSetRowSig(const HighsCutSet& cutset, HighsInt row) {
  const uint64_t row_hash = hacdcpfCutSetRowHash(cutset, row);
  uint64_t row_sig = row_hash;
  if (row >= 0 && row < static_cast<HighsInt>(cutset.upper_.size())) {
    row_sig = hacdcpfTraceHashMix(
        row_sig, static_cast<uint64_t>(
                     HighsHashHelpers::double_hash_code(cutset.upper_[row])));
  }
  if (row >= 0 && row + 1 < static_cast<HighsInt>(cutset.ARstart_.size())) {
    row_sig = hacdcpfTraceHashMix(
        row_sig,
        static_cast<uint64_t>(cutset.ARstart_[row + 1] - cutset.ARstart_[row]));
  }
  return row_sig;
}

bool hacdcpfUserCutLedgerEnabled() {
  const char* env = std::getenv("HACDCPF_HIGHS_USERCUT_LEDGER");
  if (env == nullptr || env[0] == '\0')
    env = std::getenv("HACDCPF_USERCUT_LEDGER");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static HighsInt hacdcpfEnvHighsInt(const char* name, HighsInt fallback,
                                   HighsInt min_value,
                                   HighsInt max_value) {
  const char* env = std::getenv(name);
  if (env == nullptr || env[0] == '\0') return fallback;
  char* end = nullptr;
  const long parsed = std::strtol(env, &end, 10);
  if (end == env) return fallback;
  return static_cast<HighsInt>(
      std::min<long>(std::max<long>(parsed, min_value), max_value));
}

static double hacdcpfEnvDouble(const char* name, double fallback,
                               double min_value, double max_value) {
  const char* env = std::getenv(name);
  if (env == nullptr || env[0] == '\0') return fallback;
  char* end = nullptr;
  const double parsed = std::strtod(env, &end);
  if (end == env || !std::isfinite(parsed)) return fallback;
  return std::min(std::max(parsed, min_value), max_value);
}

static bool hacdcpfEnvFlagDefaultTrue(const char* name) {
  const char* env = std::getenv(name);
  return env == nullptr || env[0] == '\0' || env[0] != '0';
}

HacdcpfUserCutAdmissionLimits hacdcpfGetUserCutAdmissionLimits(
    bool root_event) {
  HacdcpfUserCutAdmissionLimits limits;
  limits.max_global_cuts = hacdcpfEnvHighsInt(
      root_event ? "HACDCPF_HIGHS_ROOT_USERCUT_MAX_GLOBAL"
                 : "HACDCPF_HIGHS_NODE_USERCUT_MAX_GLOBAL",
      root_event ? HighsInt{16} : HighsInt{4}, 0, 1000000);
  limits.max_local_cuts = hacdcpfEnvHighsInt(
      root_event ? "HACDCPF_HIGHS_ROOT_USERCUT_MAX_LOCAL"
                 : "HACDCPF_HIGHS_NODE_USERCUT_MAX_LOCAL",
      root_event ? HighsInt{8} : HighsInt{2}, 0, 1000000);
  limits.max_row_len = hacdcpfEnvHighsInt(
      "HACDCPF_HIGHS_USERCUT_MAX_ROW_LEN", HighsInt{1000}, 1, 1000000000);
  limits.min_violation = hacdcpfEnvDouble(
      "HACDCPF_HIGHS_USERCUT_MIN_VIOLATION", 1e-7, 0.0, kHighsInf);
  limits.min_efficacy = hacdcpfEnvDouble(
      "HACDCPF_HIGHS_USERCUT_MIN_EFFICACY", 1e-9, 0.0, kHighsInf);
  limits.max_coeff_ratio = hacdcpfEnvDouble(
      "HACDCPF_HIGHS_USERCUT_MAX_COEFF_RATIO", 1e9, 1.0, kHighsInf);
  limits.require_proof_metadata =
      hacdcpfEnvFlagDefaultTrue("HACDCPF_HIGHS_USERCUT_REQUIRE_PROOF");
  return limits;
}

HacdcpfUserCutAdmission hacdcpfAdmitUserCutRow(
    const HighsCutSet& cutset, HighsInt row, const std::vector<double>& sol,
    HighsInt num_col, double feastol,
    const HacdcpfUserCutAdmissionLimits& limits) {
  HacdcpfUserCutAdmission admission;
  admission.row = row;
  if (row < 0 || row >= cutset.numCuts()) {
    admission.reason = "row_out_of_range";
    return admission;
  }
  if (row + 1 >= static_cast<HighsInt>(cutset.ARstart_.size())) {
    admission.reason = "malformed_row_start";
    return admission;
  }
  if (row >= static_cast<HighsInt>(cutset.lower_.size()) ||
      row >= static_cast<HighsInt>(cutset.upper_.size())) {
    admission.reason = "malformed_bounds";
    return admission;
  }
  if (row < static_cast<HighsInt>(cutset.hacdcpf_validity_scope_.size())) {
    admission.scope = cutset.hacdcpf_validity_scope_[row];
  }
  if (admission.scope != HighsCutSet::kHacdcpfGlobalCut &&
      admission.scope != HighsCutSet::kHacdcpfLocalNodeCut &&
      admission.scope != HighsCutSet::kHacdcpfLazyConstraint) {
    admission.reason = "unknown_scope";
    return admission;
  }
  if (admission.scope == HighsCutSet::kHacdcpfLazyConstraint) {
    admission.reason = "skip_lazy_scope";
    return admission;
  }
  if (limits.require_proof_metadata) {
    const bool has_family =
        row < static_cast<HighsInt>(cutset.hacdcpf_proof_family_.size()) &&
        !cutset.hacdcpf_proof_family_[row].empty();
    const bool has_key =
        row < static_cast<HighsInt>(cutset.hacdcpf_proof_key_.size()) &&
        !cutset.hacdcpf_proof_key_[row].empty();
    if (!has_family || !has_key) {
      admission.reason = "missing_proof_metadata";
      return admission;
    }
  }
  const HighsInt start = cutset.ARstart_[static_cast<std::size_t>(row)];
  const HighsInt end = cutset.ARstart_[static_cast<std::size_t>(row + 1)];
  admission.start = start;
  admission.end = end;
  if (start < 0 || end < start ||
      end > static_cast<HighsInt>(cutset.ARindex_.size()) ||
      end > static_cast<HighsInt>(cutset.ARvalue_.size())) {
    admission.reason = "malformed_row_range";
    return admission;
  }
  admission.len = end - start;
  if (admission.len <= 0) {
    admission.reason = "empty_row";
    return admission;
  }
  if (limits.max_row_len > 0 && admission.len > limits.max_row_len) {
    admission.reason = "row_too_long";
    return admission;
  }
  const double rhs = cutset.upper_[static_cast<std::size_t>(row)];
  if (!std::isfinite(rhs) || rhs >= kHighsInf) {
    admission.reason = "missing_finite_upper";
    return admission;
  }
  if (std::isfinite(cutset.lower_[static_cast<std::size_t>(row)])) {
    admission.reason = "finite_lower_not_upper_row";
    return admission;
  }

  double activity = 0.0;
  double norm_sq = 0.0;
  double max_abs = 0.0;
  double min_abs = kHighsInf;
  for (HighsInt k = start; k != end; ++k) {
    const HighsInt col = cutset.ARindex_[static_cast<std::size_t>(k)];
    const double val = cutset.ARvalue_[static_cast<std::size_t>(k)];
    if (col < 0 || col >= num_col ||
        col >= static_cast<HighsInt>(sol.size())) {
      admission.reason = "column_out_of_range";
      return admission;
    }
    if (!std::isfinite(val) ||
        !std::isfinite(sol[static_cast<std::size_t>(col)])) {
      admission.reason = "nonfinite_row_or_solution";
      return admission;
    }
    if (val == 0.0) {
      admission.reason = "explicit_zero_coefficient";
      return admission;
    }
    const double abs_val = std::abs(val);
    max_abs = std::max(max_abs, abs_val);
    min_abs = std::min(min_abs, abs_val);
    norm_sq += val * val;
    activity += val * sol[static_cast<std::size_t>(col)];
  }
  if (!(norm_sq > 0.0) || !(max_abs > 0.0) || !(min_abs > 0.0)) {
    admission.reason = "zero_norm";
    return admission;
  }
  admission.activity = activity;
  admission.violation = activity - rhs;
  admission.norm = std::sqrt(norm_sq);
  admission.efficacy = admission.violation / admission.norm;
  admission.max_abs = max_abs;
  admission.min_abs = min_abs;
  admission.coeff_ratio = max_abs / min_abs;
  if (!std::isfinite(admission.activity) ||
      !std::isfinite(admission.violation) ||
      !std::isfinite(admission.efficacy) ||
      !std::isfinite(admission.coeff_ratio)) {
    admission.reason = "nonfinite_metrics";
    return admission;
  }
  if (admission.violation <= std::max(limits.min_violation, feastol)) {
    admission.reason = "not_violated";
    return admission;
  }
  if (admission.efficacy <= limits.min_efficacy) {
    admission.reason = "low_efficacy";
    return admission;
  }
  if (admission.coeff_ratio > limits.max_coeff_ratio) {
    admission.reason = "bad_coefficient_ratio";
    return admission;
  }
  admission.row_hash = hacdcpfCutSetRowHash(cutset, row);
  admission.row_sig = hacdcpfCutSetRowSig(cutset, row);
  admission.admitted = true;
  admission.reason = "admissible";
  return admission;
}

void hacdcpfSortUserCutAdmissions(
    std::vector<HacdcpfUserCutAdmission>& admissions) {
  pdqsort(admissions.begin(), admissions.end(),
          [](const HacdcpfUserCutAdmission& a,
             const HacdcpfUserCutAdmission& b) {
            if (a.efficacy != b.efficacy) return a.efficacy > b.efficacy;
            if (a.violation != b.violation) return a.violation > b.violation;
            if (a.len != b.len) return a.len < b.len;
            return a.row < b.row;
          });
}

bool hacdcpfCopyUserCutRow(HighsCutSet& target, const HighsCutSet& source,
                           const HacdcpfUserCutAdmission& admission,
                           unsigned char scope) {
  if (!admission.admitted) return false;
  if (target.ARstart_.empty()) target.ARstart_.push_back(0);
  if (admission.start < 0 || admission.end < admission.start ||
      admission.end > static_cast<HighsInt>(source.ARindex_.size()) ||
      admission.end > static_cast<HighsInt>(source.ARvalue_.size())) {
    return false;
  }
  const HighsInt row = admission.row;
  target.cutindices.push_back(-2);
  target.lower_.push_back(source.lower_[static_cast<std::size_t>(row)]);
  target.upper_.push_back(source.upper_[static_cast<std::size_t>(row)]);
  target.hacdcpf_integral_.push_back(
      row < static_cast<HighsInt>(source.hacdcpf_integral_.size())
          ? source.hacdcpf_integral_[static_cast<std::size_t>(row)]
          : 0);
  target.hacdcpf_propagate_.push_back(
      row < static_cast<HighsInt>(source.hacdcpf_propagate_.size())
          ? source.hacdcpf_propagate_[static_cast<std::size_t>(row)]
          : 1);
  target.hacdcpf_validity_scope_.push_back(scope);
  target.hacdcpf_proof_family_.push_back(
      row < static_cast<HighsInt>(source.hacdcpf_proof_family_.size())
          ? source.hacdcpf_proof_family_[static_cast<std::size_t>(row)]
          : std::string{});
  target.hacdcpf_proof_key_.push_back(
      row < static_cast<HighsInt>(source.hacdcpf_proof_key_.size())
          ? source.hacdcpf_proof_key_[static_cast<std::size_t>(row)]
          : std::string{});
  for (HighsInt k = admission.start; k != admission.end; ++k) {
    target.ARindex_.push_back(source.ARindex_[static_cast<std::size_t>(k)]);
    target.ARvalue_.push_back(source.ARvalue_[static_cast<std::size_t>(k)]);
  }
  target.ARstart_.push_back(static_cast<HighsInt>(target.ARindex_.size()));
  return true;
}

static HighsInt hacdcpfUserCutLedgerTermLimit() {
  const char* env = std::getenv("HACDCPF_HIGHS_USERCUT_LEDGER_TERMS");
  if (env == nullptr || env[0] == '\0')
    env = std::getenv("HACDCPF_USERCUT_LEDGER_TERMS");
  if (env == nullptr || env[0] == '\0') return 0;
  const std::string value(env);
  if (value == "all" || value == "full")
    return std::numeric_limits<HighsInt>::max();
  char* end = nullptr;
  const long parsed = std::strtol(env, &end, 10);
  if (end == env || parsed <= 0) return 0;
  return static_cast<HighsInt>(std::min<long>(parsed, 1000000));
}

void hacdcpfLogUserCutRow(const char* phase, const char* scope,
                          const char* event, HighsInt round,
                          int64_t node_count, HighsInt depth,
                          const HighsCutSet& cutset, HighsInt row,
                          HighsInt ord, HighsInt cutindex, HighsInt lp_row,
                          double score, double viol, HighsInt active,
                          const std::vector<double>* sol,
                          const char* reason) {
  if (!hacdcpfUserCutLedgerEnabled()) return;
  if (row < 0 || row + 1 >= static_cast<HighsInt>(cutset.ARstart_.size()))
    return;
  const HighsInt start = cutset.ARstart_[static_cast<std::size_t>(row)];
  const HighsInt end = cutset.ARstart_[static_cast<std::size_t>(row + 1)];
  if (start < 0 || end < start ||
      end > static_cast<HighsInt>(cutset.ARindex_.size()) ||
      end > static_cast<HighsInt>(cutset.ARvalue_.size())) {
    return;
  }
  const HighsInt len = end - start;
  const uint64_t row_hash = hacdcpfCutSetRowHash(cutset, row);
  const uint64_t row_sig = hacdcpfCutSetRowSig(cutset, row);
  const double lower =
      row < static_cast<HighsInt>(cutset.lower_.size()) ? cutset.lower_[row]
                                                        : -kHighsInf;
  const double upper =
      row < static_cast<HighsInt>(cutset.upper_.size()) ? cutset.upper_[row]
                                                        : kHighsInf;
  const int integral =
      row < static_cast<HighsInt>(cutset.hacdcpf_integral_.size()) &&
              cutset.hacdcpf_integral_[row] != 0
          ? 1
          : 0;
  const int propagate =
      cutset.hacdcpf_propagate_.empty() ||
              (row < static_cast<HighsInt>(cutset.hacdcpf_propagate_.size()) &&
               cutset.hacdcpf_propagate_[row] != 0)
          ? 1
          : 0;
  const HighsInt term_limit = hacdcpfUserCutLedgerTermLimit();
  std::ostringstream terms;
  terms << std::setprecision(17);
  if (term_limit > 0) {
    for (HighsInt k = start; k != end && k - start < term_limit; ++k) {
      if (k != start) terms << ",";
      const HighsInt col = cutset.ARindex_[static_cast<std::size_t>(k)];
      terms << col << ":" << cutset.ARvalue_[static_cast<std::size_t>(k)];
      if (sol != nullptr && col >= 0 &&
          col < static_cast<HighsInt>(sol->size())) {
        terms << ":" << (*sol)[static_cast<std::size_t>(col)];
      }
    }
  }
  std::fprintf(
      stderr,
      "[HIGHS-USERCUT-%s-ROW] scope=%s event=%s round=%lld node=%lld "
      "depth=%lld ord=%lld inputRow=%lld cutidx=%lld lpRow=%lld len=%lld "
      "lower=%.17g upper=%.17g score=%.17g viol=%.17g active=%lld "
      "integral=%d propagate=%d rowHash=%016llx rowSig=%016llx reason=%s "
      "terms=[%s]\n",
      phase != nullptr ? phase : "UNKNOWN",
      scope != nullptr ? scope : "unknown",
      event != nullptr ? event : "unknown", static_cast<long long>(round),
      static_cast<long long>(node_count), static_cast<long long>(depth),
      static_cast<long long>(ord), static_cast<long long>(row),
      static_cast<long long>(cutindex), static_cast<long long>(lp_row),
      static_cast<long long>(len), lower, upper, score, viol,
      static_cast<long long>(active), integral, propagate,
      static_cast<unsigned long long>(row_hash),
      static_cast<unsigned long long>(row_sig),
      reason != nullptr ? reason : "ok", terms.str().c_str());
  std::fflush(stderr);
}

static HighsInt hacdcpfCutpoolSelectTraceLimit() {
  const char* root_only = std::getenv("HACDCPF_XPOOL_ROOT_LEDGER_ONLY");
  if (root_only != nullptr && root_only[0] != '\0' && root_only[0] != '0')
    return 0;
  const char* env = std::getenv("HACDCPF_HIGHS_CUTPOOL_SELECT_TRACE");
  if (env == nullptr || env[0] == '\0') {
    env = std::getenv("HACDCPF_HIGHS_CUTPOOL_TRACE");
    if (env == nullptr || env[0] == '\0') return 0;
    const std::string trace_value(env);
    if (trace_value != "all" && trace_value != "rows") return 0;
  }
  if (env == nullptr || env[0] == '\0') return 0;
  if (std::string(env) == "all") return 1000000000;
  if (std::string(env) == "rows") return 16;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return 16;
  return value > 0 ? static_cast<HighsInt>(std::min<long>(value, 1000000)) : 0;
}

static bool hacdcpfCutpoolRootLedgerOnly() {
  const char* env = std::getenv("HACDCPF_XPOOL_ROOT_LEDGER_ONLY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

static bool hacdcpfRootLpCallbackEnabled(const HighsMipSolver& mipsolver,
                                         bool rootLedgerScope) {
  if (mipsolver.callback_ == nullptr ||
      !mipsolver.callback_->hacdcpf_root_lp_callback) {
    return false;
  }
  return !hacdcpfCutpoolRootLedgerOnly() || rootLedgerScope;
}

static bool hacdcpfCutpoolAddTraceEnabled(bool rootLedgerScope) {
  if (hacdcpfCutpoolRootLedgerOnly() && !rootLedgerScope) return false;
  if (std::getenv("HACDCPF_HIGHS_CUTPOOL_ADD_TRACE") != nullptr) return true;
  const char* env = std::getenv("HACDCPF_HIGHS_CUTPOOL_TRACE");
  if (env == nullptr || env[0] == '\0') return false;
  const std::string value(env);
  return value == "all" || value == "add" || value == "rows";
}

static HighsInt hacdcpfCutpoolAddTraceTermLimit() {
  const char* env = std::getenv("HACDCPF_HIGHS_CUTPOOL_ADD_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') {
    env = std::getenv("HACDCPF_XPOOL_FULL_CUTPOOL_ADD_REPLAY");
    if (env != nullptr && env[0] != '\0' && env[0] != '0')
      return std::numeric_limits<HighsInt>::max();
    return 0;
  }
  const std::string value_str(env);
  if (value_str == "all" || value_str == "full")
    return std::numeric_limits<HighsInt>::max();
  char* end = nullptr;
  const long numeric_value = std::strtol(env, &end, 10);
  if (end == env || numeric_value <= 0) return 0;
  return static_cast<HighsInt>(std::min<long>(numeric_value, 1000000000));
}

#if 0
static void printCut(const HighsInt* Rindex, const double* Rvalue, HighsInt Rlen,
                     double rhs) {
  for (HighsInt i = 0; i != Rlen; ++i) {
    if (Rvalue[i] > 0)
      printf("+%g<x%" HIGHSINT_FORMAT "> ", Rvalue[i], Rindex[i]);
    else
      printf("-%g<x%" HIGHSINT_FORMAT "> ", -Rvalue[i], Rindex[i]);
  }

  printf("<= %g\n", rhs);
}
#endif

bool HighsCutPool::isDuplicate(size_t hash, double norm, const HighsInt* Rindex,
                               const double* Rvalue, HighsInt Rlen,
                               double rhs) {
  auto range = hashToCutMap.equal_range(hash);
  const double* ARvalue = matrix_.getARvalue();
  const HighsInt* ARindex = matrix_.getARindex();

  for (auto it = range.first; it != range.second; ++it) {
    HighsInt rowindex = it->second;
    HighsInt start = matrix_.getRowStart(rowindex);
    HighsInt end = matrix_.getRowEnd(rowindex);

    if (end - start != Rlen) continue;
    if (std::equal(Rindex, Rindex + Rlen, &ARindex[start])) {
      double dotprod = 0.0;

      for (HighsInt i = 0; i != Rlen; ++i)
        dotprod += Rvalue[i] * ARvalue[start + i];

      double parallelism = double(dotprod) * rownormalization_[rowindex] * norm;

      // printf("\n\ncuts with same support and parallelism %g:\n",
      // parallelism); printf("CUT1: "); printCut(Rindex, Rvalue, Rlen, rhs);
      // printf("CUT2: ");
      // printCut(Rindex, ARvalue + start, Rlen, rhs_[rowindex]);
      // printf("\n");

      if (parallelism >= 1 - 1e-6) return true;

      //{
      //  if (ages_[rowindex] >= 0) {
      //    matrix_.replaceRowValues(rowindex, Rvalue);
      //    return rowindex;
      //  } else
      //    return -2;
      //}
    }
  }

  return false;
}

double HighsCutPool::getParallelism(HighsInt row1, HighsInt row2) const {
  HighsInt i1 = matrix_.getRowStart(row1);
  const HighsInt end1 = matrix_.getRowEnd(row1);

  HighsInt i2 = matrix_.getRowStart(row2);
  const HighsInt end2 = matrix_.getRowEnd(row2);

  const HighsInt* ARindex = matrix_.getARindex();
  const double* ARvalue = matrix_.getARvalue();

  double dotprod = 0.0;
  while (i1 != end1 && i2 != end2) {
    HighsInt col1 = ARindex[i1];
    HighsInt col2 = ARindex[i2];

    if (col1 < col2)
      ++i1;
    else if (col2 < col1)
      ++i2;
    else {
      dotprod += ARvalue[i1] * ARvalue[i2];
      ++i1;
      ++i2;
    }
  }

  return dotprod * rownormalization_[row1] * rownormalization_[row2];
}

double HighsCutPool::getParallelism(HighsInt row1, HighsInt row2,
                                    const HighsCutPool& pool2) const {
  HighsInt i1 = matrix_.getRowStart(row1);
  const HighsInt end1 = matrix_.getRowEnd(row1);

  HighsInt i2 = pool2.matrix_.getRowStart(row2);
  const HighsInt end2 = pool2.matrix_.getRowEnd(row2);

  const HighsInt* ARindex1 = matrix_.getARindex();
  const double* ARvalue1 = matrix_.getARvalue();
  const HighsInt* ARindex2 = pool2.matrix_.getARindex();
  const double* ARvalue2 = pool2.matrix_.getARvalue();

  double dotprod = 0.0;
  while (i1 != end1 && i2 != end2) {
    HighsInt col1 = ARindex1[i1];
    HighsInt col2 = ARindex2[i2];

    if (col1 < col2)
      ++i1;
    else if (col2 < col1)
      ++i2;
    else {
      dotprod += ARvalue1[i1] * ARvalue2[i2];
      ++i1;
      ++i2;
    }
  }

  return dotprod * rownormalization_[row1] * pool2.rownormalization_[row2];
}

void HighsCutPool::lpCutRemoved(HighsInt cut, bool thread_safe) {
  const HighsInt n = numLps_[cut].fetch_add(-1, std::memory_order_relaxed);
  if (thread_safe || n > 1) return;
  if (matrix_.columnsLinked(cut)) {
    propRows.erase(std::make_pair(ages_[cut], cut));
    propRows.emplace(1, cut);
  }
  ages_[cut] = 1;
  --numLpCuts;
  ++ageDistribution[1];
}

void HighsCutPool::performAging() {
  HighsInt cutIndexEnd = matrix_.getNumRows();

  HighsInt agelim = agelim_;
  HighsInt numAvailableCuts = getNumAvailableCuts();
  while (agelim > 5 && numAvailableCuts > softlimit_) {
    numAvailableCuts -= ageDistribution[agelim];
    --agelim;
  }

  for (HighsInt i = 0; i != cutIndexEnd; ++i) {
    // Catch buffered changes (should only occur in parallel case)
    if (numLps_[i] > 0 && ages_[i] >= 0) {
      // Cut has been added to the LP, but age changes haven't been made
      --ageDistribution[ages_[i]];
      if (matrix_.columnsLinked(i)) {
        propRows.erase(std::make_pair(ages_[i], i));
        propRows.emplace(-1, i);
      }
      ages_[i] = -1;
      ++numLpCuts;
      ageResetWhileLocked_[i].store(0, std::memory_order_relaxed);
    } else if (numLps_[i] == 0 && ages_[i] == -1 && rhs_[i] != kHighsInf) {
      // Cut was removed from the LP, but age changes haven't been made
      if (matrix_.columnsLinked(i)) {
        propRows.erase(std::make_pair(ages_[i], i));
        propRows.emplace(1, i);
      }
      ages_[i] = 1;
      --numLpCuts;
      ++ageDistribution[1];
      ageResetWhileLocked_[i].store(0, std::memory_order_relaxed);
      continue;
    } else if (ageResetWhileLocked_[i].load(std::memory_order_relaxed) == 1) {
      resetAge(i);
    }
    ageResetWhileLocked_[i].store(0, std::memory_order_relaxed);
    if (ages_[i] < 0) continue;

    bool isPropagated = matrix_.columnsLinked(i);
    if (isPropagated) propRows.erase(std::make_pair(ages_[i], i));
    ageDistribution[ages_[i]] -= 1;
    ages_[i] += 1;

    if (ages_[i] > agelim) {
      for (HighsDomain::CutpoolPropagation* propagationdomain :
           propagationDomains)
        propagationdomain->cutDeleted(i);

      if (isPropagated) {
        --numPropRows;
        numPropNzs -= getRowLength(i);
      }

      matrix_.removeRow(i);
      ages_[i] = -1;
      rhs_[i] = kHighsInf;
      hasSynced_[i] = false;
    } else {
      if (isPropagated) propRows.emplace(ages_[i], i);
      ageDistribution[ages_[i]] += 1;
    }
  }

  assert((HighsInt)propRows.size() == numPropRows);
}

void HighsCutPool::separate(const std::vector<double>& sol,
                            const HighsDomain& domain, HighsCutSet& cutset,
                            double feastol,
                            const std::deque<HighsCutPool>& cutpools,
                            bool thread_safe,
                            const HighsMipSolver* mipsolver) {
  HighsInt nrows = matrix_.getNumRows();
  const HighsInt* ARindex = matrix_.getARindex();
  const double* ARvalue = matrix_.getARvalue();

  std::vector<std::pair<double, HighsInt>> efficacious_cuts;
  const bool traceCutpool =
      std::getenv("HACDCPF_HIGHS_CUTPOOL_TRACE") != nullptr &&
      (!hacdcpfCutpoolRootLedgerOnly() || hacdcpfRootLedgerScope);
  HighsInt traceAvailable = 0;
  HighsInt traceViolated = 0;
  HighsInt traceActive = 0;
  HighsInt traceParallelDiscarded = 0;
  HighsInt traceSelectedNnz = 0;
  uint64_t traceSelectIndexHash = 0;
  uint64_t traceSelectContentHash = 0;
  double traceMaxViolation = 0.0;

  HighsInt agelim = agelim_;

  auto traceSeparate = [&](HighsInt numEfficacious) {
    if (!traceCutpool) return;
    std::fprintf(stderr,
                 "[HIGHS-CUTPOOL] pool=%lld avail=%lld violated=%lld "
                 "active=%lld efficacious=%lld selected=%lld parallel=%lld "
                 "bestScore=%.12g minScoreFactor=%.12g maxViol=%.12g "
                 "agelim=%lld numLpCuts=%lld selNnz=%lld selIdxHash=%016llx "
                 "selContentHash=%016llx\n",
                 static_cast<long long>(getNumCuts()),
                 static_cast<long long>(traceAvailable),
                 static_cast<long long>(traceViolated),
                 static_cast<long long>(traceActive),
                 static_cast<long long>(numEfficacious),
                 static_cast<long long>(cutset.numCuts()),
                 static_cast<long long>(traceParallelDiscarded),
                 bestObservedScore, minScoreFactor, traceMaxViolation,
                 static_cast<long long>(agelim),
                 static_cast<long long>(numLpCuts),
                 static_cast<long long>(traceSelectedNnz),
                 static_cast<unsigned long long>(traceSelectIndexHash),
                 static_cast<unsigned long long>(traceSelectContentHash));
  };

  HighsInt numCuts = getNumCuts() - numLpCuts;
  while (agelim > 1 && numCuts > softlimit_) {
    numCuts -= ageDistribution[agelim];
    --agelim;
  }

  for (HighsInt i = 0; i < nrows; ++i) {
    // cuts with an age of -1 are already in the LP and are therefore skipped
    // Warning: Parallel case tries to add cuts already in current LP.
    // Inefficient. Not sure what happens if added twice.
    // The cut shouldn't have enough violation to be added though.
    if (ages_[i] < 0) continue;
    ++traceAvailable;

    HighsInt start = matrix_.getRowStart(i);
    HighsInt end = matrix_.getRowEnd(i);

    double viol(-rhs_[i]);

    for (HighsInt j = start; j != end; ++j) {
      HighsInt col = ARindex[j];
      double solval = sol[col];

      viol += ARvalue[j] * solval;
    }
    traceMaxViolation = std::max(traceMaxViolation, double(viol));

    // if the cut is not violated more than feasibility tolerance
    // we skip it and increase its age, otherwise we reset its age
    bool isPropagated = matrix_.columnsLinked(i);
    if (!thread_safe) {
      ageDistribution[ages_[i]] -= 1;
      if (isPropagated) {
        propRows.erase(std::make_pair(ages_[i], i));
      }
    }
    if (double(viol) <= feastol) {
      if (thread_safe) continue;
      ++ages_[i];
      if (ages_[i] >= agelim) {
        uint64_t h = compute_cut_hash(&ARindex[start], &ARvalue[start],
                                      maxabscoef_[i], end - start);

        for (HighsDomain::CutpoolPropagation* propagationdomain :
             propagationDomains)
          propagationdomain->cutDeleted(i);

        if (isPropagated) {
          --numPropRows;
          numPropNzs -= getRowLength(i);
        }

        matrix_.removeRow(i);
        ages_[i] = -1;
        rhs_[i] = kHighsInf;
        ageResetWhileLocked_[i].store(0, std::memory_order_relaxed);
        hasSynced_[i] = false;
        auto range = hashToCutMap.equal_range(h);

        for (auto it = range.first; it != range.second; ++it) {
          if (it->second == i) {
            hashToCutMap.erase(it);
            break;
          }
        }
      } else {
        if (isPropagated) propRows.emplace(ages_[i], i);
        ageDistribution[ages_[i]] += 1;
      }
      continue;
    }
    ++traceViolated;

    // compute the norm only for those entries that do not sit at their minimal
    // activity in the current solution this avoids the phenomenon that the
    // traditional efficacy gets weaker for stronger cuts E.g. when considering
    // a clique cut which has additional entries whose value in the current
    // solution is 0 then the efficacy gets lower for each such entry even
    // though the cut dominates the clique cut where all those entries are
    // relaxed out.
    HighsCDouble rownorm = 0.0;
    HighsInt numActiveNzs = 0;
    for (HighsInt j = start; j != end; ++j) {
      HighsInt col = ARindex[j];
      double solval = sol[col];
      if (ARvalue[j] > 0) {
        if (solval > domain.col_lower_[col] + feastol) {
          rownorm += ARvalue[j] * ARvalue[j];
          numActiveNzs += 1;
        }
      } else {
        if (solval < domain.col_upper_[col] - feastol) {
          rownorm += ARvalue[j] * ARvalue[j];
          numActiveNzs += 1;
        }
      }
    }
    if (numActiveNzs > 0) ++traceActive;

    if (!thread_safe) {
      ages_[i] = 0;
      ++ageDistribution[0];
      if (isPropagated) propRows.emplace(ages_[i], i);
    }
    double score = viol / (numActiveNzs * sqrt(double(rownorm)));

    efficacious_cuts.emplace_back(score, i);
  }
  assert((HighsInt)propRows.size() == numPropRows);
  if (efficacious_cuts.empty()) {
    traceSeparate(0);
    return;
  }

  pdqsort(efficacious_cuts.begin(), efficacious_cuts.end(),
          [&efficacious_cuts](const std::pair<double, HighsInt>& a,
                              const std::pair<double, HighsInt>& b) {
            if (a.first > b.first) return true;
            if (a.first < b.first) return false;
            return std::make_pair(
                       HighsHashHelpers::hash((uint64_t(a.second) << 32) +
                                              efficacious_cuts.size()),
                       a.second) >
                   std::make_pair(
                       HighsHashHelpers::hash((uint64_t(b.second) << 32) +
                                              efficacious_cuts.size()),
                       b.second);
          });

  double bestObservedScoreCopy = bestObservedScore;
  double& bestObservedScore_ =
      thread_safe ? bestObservedScoreCopy : bestObservedScore;
  bestObservedScore_ = std::max(efficacious_cuts[0].first, bestObservedScore);
  double minScoreFactorCopy = minScoreFactor;
  double& minScoreFactor_ = thread_safe ? minScoreFactorCopy : minScoreFactor;
  double minScore = minScoreFactor_ * bestObservedScore_;

  HighsInt numefficacious =
      std::upper_bound(efficacious_cuts.begin(), efficacious_cuts.end(),
                       minScore,
                       [](double mscore, std::pair<double, HighsInt> const& c) {
                         return mscore > c.first;
                       }) -
      efficacious_cuts.begin();

  HighsInt lowerThreshold = efficacious_cuts.size() / 20;
  HighsInt upperThreshold = efficacious_cuts.size() - 1;

  if (numefficacious <= lowerThreshold) {
    numefficacious = std::max(efficacious_cuts.size() / 2, size_t{1});
    minScoreFactor_ =
        efficacious_cuts[numefficacious - 1].first / bestObservedScore_;
  } else if (numefficacious > upperThreshold) {
    minScoreFactor_ =
        efficacious_cuts[upperThreshold].first / bestObservedScore_;
  }

  efficacious_cuts.resize(numefficacious);

  HighsInt orignumcuts = cutset.numCuts();
  HighsInt origselectednnz = cutset.ARindex_.size();
  HighsInt selectednnz = origselectednnz;

  if (const char* scoreTraceEnv =
          std::getenv("HACDCPF_HIGHS_CUTPOOL_SCORE_TRACE")) {
    HighsInt traceLimit = 32;
    HighsInt traceTerms = 0;
    if (const char* termsEnv =
            std::getenv("HACDCPF_HIGHS_CUTPOOL_SCORE_TRACE_TERMS")) {
      char* end = nullptr;
      const long parsed = std::strtol(termsEnv, &end, 10);
      if (end != termsEnv && parsed > 0)
        traceTerms = static_cast<HighsInt>(std::min<long>(parsed, 200));
    }
    if (std::string(scoreTraceEnv) == "all") {
      traceLimit = std::numeric_limits<HighsInt>::max();
    } else if (scoreTraceEnv[0] != '\0') {
      char* end = nullptr;
      const long parsed = std::strtol(scoreTraceEnv, &end, 10);
      if (end != scoreTraceEnv) {
        traceLimit = parsed > 0
                         ? static_cast<HighsInt>(std::min<long>(parsed, 1000000))
                         : 0;
      }
    }
    for (HighsInt ord = 0;
         ord < static_cast<HighsInt>(efficacious_cuts.size()) &&
         ord < traceLimit;
         ++ord) {
      const HighsInt cut = efficacious_cuts[ord].second;
      const HighsInt start = matrix_.getRowStart(cut);
      const HighsInt end = matrix_.getRowEnd(cut);
      const HighsInt len = end - start;
      const uint64_t rowHash = compute_cut_hash(&ARindex[start], &ARvalue[start],
                                                maxabscoef_[cut], len);
      double viol = -rhs_[cut];
      HighsCDouble rownorm = 0.0;
      HighsInt numActiveNzs = 0;
      std::ostringstream terms;
      terms << std::setprecision(17);
      HighsInt emittedTerms = 0;
      for (HighsInt j = start; j != end; ++j) {
        const HighsInt col = ARindex[j];
        const double value = ARvalue[j];
        const double solval = sol[col];
        viol += value * solval;
        if (value > 0) {
          if (solval > domain.col_lower_[col] + feastol) {
            rownorm += value * value;
            numActiveNzs += 1;
          }
        } else {
          if (solval < domain.col_upper_[col] - feastol) {
            rownorm += value * value;
            numActiveNzs += 1;
          }
        }
        if (emittedTerms < traceTerms) {
          if (emittedTerms > 0) terms << ",";
          terms << col << ":" << value << ":" << solval;
          ++emittedTerms;
        }
      }
      std::fprintf(stderr,
                   "[HIGHS-CUTPOOL-SCORED] ord=%lld idx=%lld len=%lld "
                   "rhs=%.17g score=%.17g viol=%.17g active=%lld "
                   "norm=%.17g rowHash=%016llx terms=[%s]\n",
                   static_cast<long long>(ord), static_cast<long long>(cut),
                   static_cast<long long>(len), rhs_[cut],
                   efficacious_cuts[ord].first,
                   viol, static_cast<long long>(numActiveNzs), double(rownorm),
                   static_cast<unsigned long long>(rowHash),
                   terms.str().c_str());
    }
  }

  std::vector<double> selectedScore;

  for (const std::pair<double, HighsInt>& p : efficacious_cuts) {
    bool discard = false;
    double maxpar = 0.1;
    for (HighsInt i = 0; i != static_cast<HighsInt>(cutset.cutindices.size());
         ++i) {
      if (cutset.cutpools[i] == index_) {
        if (getParallelism(cutset.cutindices[i], p.second) > maxpar) {
          discard = true;
          break;
        }
      } else {
        // Warning: This assumes the cuts in the pool are not changing during,
        // this query, i.e., the worker's pool and the global pool.
        // Currently safe, but doesn't generalise to all designs.
        if (getParallelism(p.second, cutset.cutindices[i],
                           cutpools[cutset.cutpools[i]]) > maxpar) {
          discard = true;
          break;
        }
      }
    }

    if (discard) {
      ++traceParallelDiscarded;
      continue;
    }

    numLps_[p.second].fetch_add(1, std::memory_order_relaxed);
    if (!thread_safe) {
      --ageDistribution[ages_[p.second]];
      ++numLpCuts;
      if (matrix_.columnsLinked(p.second)) {
        propRows.erase(std::make_pair(ages_[p.second], p.second));
        propRows.emplace(-1, p.second);
      }
      ages_[p.second] = -1;
    }
    cutset.cutindices.push_back(p.second);
    cutset.cutpools.push_back(index_);
    selectedScore.push_back(p.first);
    selectednnz += matrix_.getRowEnd(p.second) - matrix_.getRowStart(p.second);
  }

  cutset.resize(selectednnz);

  assert(int(cutset.ARvalue_.size()) == selectednnz);
  assert(int(cutset.ARindex_.size()) == selectednnz);

  HighsInt offset = origselectednnz;
  for (HighsInt i = orignumcuts; i != cutset.numCuts(); ++i) {
    cutset.ARstart_[i] = offset;
    HighsInt cut = cutset.cutindices[i];
    HighsInt start = matrix_.getRowStart(cut);
    HighsInt end = matrix_.getRowEnd(cut);
    cutset.upper_[i] = rhs_[cut];

    for (HighsInt j = start; j != end; ++j) {
      assert(offset < selectednnz);
      cutset.ARvalue_[offset] = ARvalue[j];
      cutset.ARindex_[offset] = ARindex[j];
      ++offset;
    }
  }

  assert((HighsInt)propRows.size() == numPropRows);
  cutset.ARstart_[cutset.numCuts()] = offset;
  HighsInt selectTraceLimit = hacdcpfCutpoolSelectTraceLimit();
  if (hacdcpfCutpoolRootLedgerOnly() && hacdcpfRootLedgerScope) {
    const char* env = std::getenv("HACDCPF_HIGHS_CUTPOOL_SELECT_TRACE");
    if (env == nullptr || env[0] == '\0')
      env = std::getenv("HACDCPF_HIGHS_CUTPOOL_TRACE");
    if (env != nullptr && std::string(env) == "all")
      selectTraceLimit = std::numeric_limits<HighsInt>::max();
    else if (env != nullptr && std::string(env) == "rows")
      selectTraceLimit = 16;
    else if (env != nullptr && env[0] != '\0') {
      char* end = nullptr;
      const long value = std::strtol(env, &end, 10);
      selectTraceLimit =
          end != env && value > 0
              ? static_cast<HighsInt>(std::min<long>(value, 1000000))
              : 16;
    }
  }
  traceSelectIndexHash = uint64_t{0x4849474853435349};
  traceSelectContentHash = uint64_t{0x4849474853435343};
  traceSelectedNnz = selectednnz - origselectednnz;
  for (HighsInt i = 0; i != cutset.numCuts(); ++i) {
    const HighsInt cut = cutset.cutindices[i];
    const HighsInt start = matrix_.getRowStart(cut);
    const HighsInt end = matrix_.getRowEnd(cut);
    const HighsInt len = end - start;
    const uint64_t rowHash = compute_cut_hash(&ARindex[start], &ARvalue[start],
                                              maxabscoef_[cut], len);
    uint64_t rowSig = hacdcpfTraceHashMix(
        rowHash,
        static_cast<uint64_t>(HighsHashHelpers::double_hash_code(rhs_[cut])));
    rowSig = hacdcpfTraceHashMix(rowSig, static_cast<uint64_t>(len));
    traceSelectIndexHash =
        hacdcpfTraceHashMix(traceSelectIndexHash, static_cast<uint64_t>(cut));
    traceSelectContentHash =
        hacdcpfTraceHashMix(traceSelectContentHash, rowSig);
    if (selectTraceLimit > 0 && i < selectTraceLimit) {
      std::fprintf(stderr,
                   "[HIGHS-CUTPOOL-SELECT-ROW] ord=%lld idx=%lld len=%lld "
                   "rhs=%.17g score=%.17g rowHash=%016llx rowSig=%016llx\n",
                   static_cast<long long>(i), static_cast<long long>(cut),
                   static_cast<long long>(len), rhs_[cut],
                   i >= orignumcuts &&
                           (i - orignumcuts) <
                               static_cast<HighsInt>(selectedScore.size())
                       ? selectedScore[i - orignumcuts]
                       : 0.0,
                   static_cast<unsigned long long>(rowHash),
                   static_cast<unsigned long long>(rowSig));
    }
  }
  if (selectTraceLimit > 0) {
    std::fprintf(stderr,
                 "[HIGHS-CUTPOOL-SELECT] selected=%lld indexHash=%016llx "
                 "contentHash=%016llx nnz=%lld traceRows=%lld\n",
                 static_cast<long long>(cutset.numCuts()),
                 static_cast<unsigned long long>(traceSelectIndexHash),
                 static_cast<unsigned long long>(traceSelectContentHash),
                 static_cast<long long>(traceSelectedNnz),
                 static_cast<long long>(std::min(selectTraceLimit,
                                                 cutset.numCuts())));
  }
  if (mipsolver != nullptr &&
      hacdcpfRootLpCallbackEnabled(*mipsolver, hacdcpfRootLedgerScope)) {
    mipsolver->callback_->hacdcpf_root_lp_callback(
        "select", nullptr, &cutset,
        mipsolver->callback_->hacdcpf_root_lp_callback_data);
  }
  traceSeparate(numefficacious);
}

void HighsCutPool::separateLpCutsAfterRestart(HighsCutSet& cutset,
                                              const HighsMipSolver* mipsolver) {
  // should only be called after a restart with a fresh row matrix right now
  assert(matrix_.getNumDelRows() == 0);
  HighsInt numcuts = matrix_.getNumRows();

  cutset.cutindices.resize(numcuts);
  cutset.cutpools.resize(numcuts, index_);
  std::iota(cutset.cutindices.begin(), cutset.cutindices.end(), 0);
  cutset.resize(matrix_.nonzeroCapacity());

  HighsInt offset = 0;
  const HighsInt* ARindex = matrix_.getARindex();
  const double* ARvalue = matrix_.getARvalue();
  for (HighsInt i = 0; i != cutset.numCuts(); ++i) {
    --ageDistribution[ages_[i]];
    ++numLpCuts;
    if (matrix_.columnsLinked(i)) {
      propRows.erase(std::make_pair(ages_[i], i));
      propRows.emplace(-1, i);
    }
    numLps_[i] = 1;
    ages_[i] = -1;
    cutset.ARstart_[i] = offset;
    HighsInt cut = cutset.cutindices[i];
    HighsInt start = matrix_.getRowStart(cut);
    HighsInt end = matrix_.getRowEnd(cut);
    cutset.upper_[i] = rhs_[cut];

    for (HighsInt j = start; j != end; ++j) {
      assert(offset < (HighsInt)matrix_.nonzeroCapacity());
      cutset.ARvalue_[offset] = ARvalue[j];
      cutset.ARindex_[offset] = ARindex[j];
      ++offset;
    }
  }

  cutset.ARstart_[cutset.numCuts()] = offset;

  assert((HighsInt)propRows.size() == numPropRows);
  if (mipsolver != nullptr &&
      hacdcpfRootLpCallbackEnabled(*mipsolver, hacdcpfRootLedgerScope)) {
    mipsolver->callback_->hacdcpf_root_lp_callback(
        "select_restart", nullptr, &cutset,
        mipsolver->callback_->hacdcpf_root_lp_callback_data);
  }
}

HighsInt HighsCutPool::addCut(const HighsMipSolver& mipsolver, HighsInt* Rindex,
                              double* Rvalue, HighsInt Rlen, double rhs,
                              bool integral, bool propagate,
                              bool extractCliques, bool isConflict) {
  mipsolver.mipdata_->debugSolution.checkCut(Rindex, Rvalue, Rlen, rhs);

  sortBuffer.resize(Rlen);

  // compute 1/||a|| for the cut
  // as it is only computed once
  double norm = 0.0;
  double maxabscoef = 0.0;
  for (HighsInt i = 0; i != Rlen; ++i) {
    norm += Rvalue[i] * Rvalue[i];
    maxabscoef = std::max(maxabscoef, std::abs(Rvalue[i]));
    sortBuffer[i].first = Rindex[i];
    sortBuffer[i].second = Rvalue[i];
  }
  pdqsort_branchless(
      sortBuffer.begin(), sortBuffer.end(),
      [](const std::pair<HighsInt, double>& a,
         const std::pair<HighsInt, double>& b) { return a.first < b.first; });
  for (HighsInt i = 0; i != Rlen; ++i) {
    Rindex[i] = sortBuffer[i].first;
    Rvalue[i] = sortBuffer[i].second;
  }
  uint64_t h = compute_cut_hash(Rindex, Rvalue, maxabscoef, Rlen);
  double normalization = 1.0 / double(sqrt(norm));

  // Warning: This global duplicate check assumes the global pool doesn't
  // have cuts added or deleted during time when local pools can add a cut.
  if (this != &mipsolver.mipdata_->getCutPool()) {
    if (mipsolver.mipdata_->getCutPool().isDuplicate(h, normalization, Rindex,
                                                     Rvalue, Rlen, rhs)) {
      return -1;
    }
  }
  if (isDuplicate(h, normalization, Rindex, Rvalue, Rlen, rhs)) return -1;

  // if (Rlen > 0.15 * matrix_.numCols())
  //   printf("cut with len %d not propagated\n", Rlen);
  if (propagate) {
    HighsInt newPropNzs = numPropNzs + Rlen;

    double avgModelNzs = mipsolver.numNonzero() / (double)mipsolver.numRow();

    double newAvgPropNzs = newPropNzs / (double)(numPropRows + 1);

    constexpr double alpha = 2.0;
    if (isConflict) {
      // for conflicts we allow an increased average propagation density
      if (newAvgPropNzs > std::max(alpha * avgModelNzs, minDensityLim)) {
        propagate = false;
      } else {
        ++numPropRows;
        numPropNzs = newPropNzs;
      }
    } else {
      // for cuts we do not want to accept any dense cuts and don't use the
      // average but its actual length
      if (Rlen >= std::max(alpha * avgModelNzs, minDensityLim)) {
        propagate = false;
      } else {
        ++numPropRows;
        numPropNzs = newPropNzs;
      }
    }
  }

  // if we have more than twice the number of nonzeros of the model in use for
  // propagation we stop propagating the rows with the highest age
  HighsInt propRowExcessNzs = numPropNzs - 2 * mipsolver.numNonzero();
  if (propRowExcessNzs > 0) {
    auto it = propRows.rbegin();

    while (propRowExcessNzs > 0 && it != propRows.rend()) {
      HighsInt len = getRowLength(it->second);
      propRowExcessNzs -= len;
      numPropNzs -= len;
      --numPropRows;
      ++it;
    }

    for (auto i = propRows.rbegin(); i != it; ++i) {
      HighsInt row = i->second;
      matrix_.unlinkColumns(row);
      for (HighsDomain::CutpoolPropagation* propagationdomain :
           propagationDomains)
        propagationdomain->cutDeleted(row, true);
    }

    propRows.erase(it.base(), propRows.end());
  }

  // if no such cut exists we append the new cut
  HighsInt rowindex = matrix_.addRow(Rindex, Rvalue, Rlen, propagate);
  hashToCutMap.emplace(h, rowindex);

  if (rowindex == int(rhs_.size())) {
    rhs_.resize(rowindex + 1);
    ages_.resize(rowindex + 1);
    numLps_.resize(rowindex + 1);
    ageResetWhileLocked_.resize(rowindex + 1);
    hasSynced_.resize(rowindex + 1);
    rownormalization_.resize(rowindex + 1);
    maxabscoef_.resize(rowindex + 1);
    rowintegral.resize(rowindex + 1);
  }

  // set the right hand side and reset the age
  rhs_[rowindex] = rhs;
  ages_[rowindex] = std::max(HighsInt{0}, agelim_ - 5);
  ++ageDistribution[ages_[rowindex]];
  rowintegral[rowindex] = integral;
  numLps_[rowindex] = 0;
  ageResetWhileLocked_[rowindex].store(0, std::memory_order_relaxed);
  hasSynced_[rowindex] = false;
  if (propagate) propRows.emplace(ages_[rowindex], rowindex);
  assert((HighsInt)propRows.size() == numPropRows);

  rownormalization_[rowindex] = normalization;
  maxabscoef_[rowindex] = maxabscoef;

  if (hacdcpfCutpoolAddTraceEnabled(hacdcpfRootLedgerScope)) {
    uint64_t rowSig = hacdcpfTraceHashMix(
        h,
        static_cast<uint64_t>(HighsHashHelpers::double_hash_code(rhs)));
    rowSig = hacdcpfTraceHashMix(rowSig, static_cast<uint64_t>(Rlen));
    const HighsInt traceTerms = hacdcpfCutpoolAddTraceTermLimit();
    if (traceTerms > 0) {
      std::ostringstream terms;
      terms << std::setprecision(17);
      for (HighsInt i = 0; i < Rlen && i < traceTerms; ++i) {
        if (i > 0) terms << ",";
        terms << Rindex[i] << ":" << Rvalue[i];
      }
      std::fprintf(stderr,
                   "[HIGHS-CUTPOOL-ADD] idx=%lld len=%lld rhs=%.17g "
                   "integral=%d propagate=%d rowHash=%016llx rowSig=%016llx "
                   "terms=[%s]\n",
                   static_cast<long long>(rowindex),
                   static_cast<long long>(Rlen), rhs, integral ? 1 : 0,
                   propagate ? 1 : 0, static_cast<unsigned long long>(h),
                   static_cast<unsigned long long>(rowSig),
                   terms.str().c_str());
    } else {
      std::fprintf(stderr,
                   "[HIGHS-CUTPOOL-ADD] idx=%lld len=%lld rhs=%.17g "
                   "integral=%d propagate=%d rowHash=%016llx rowSig=%016llx\n",
                   static_cast<long long>(rowindex),
                   static_cast<long long>(Rlen), rhs, integral ? 1 : 0,
                   propagate ? 1 : 0, static_cast<unsigned long long>(h),
                   static_cast<unsigned long long>(rowSig));
    }
  }

  // printf("density: %.2f%%\n", 100.0 * Rlen / (double)matrix_.numCols());
  for (HighsDomain::CutpoolPropagation* propagationdomain : propagationDomains)
    propagationdomain->cutAdded(rowindex, propagate);

  if (extractCliques && this == &mipsolver.mipdata_->getCutPool()) {
    // if this is the global cutpool extract cliques from the cut
    if (Rlen <= 100)
      mipsolver.mipdata_->cliquetable.extractCliquesFromCut(mipsolver, Rindex,
                                                            Rvalue, Rlen, rhs);
  }

  return rowindex;
}

void HighsCutPool::syncCutPool(const HighsMipSolver& mipsolver,
                               HighsCutPool& syncpool) {
  HighsInt cutIndexEnd = matrix_.getNumRows();
  std::vector<HighsInt> idxs;
  std::vector<double> vals;

  for (HighsInt i = 0; i != cutIndexEnd; ++i) {
    // Only sync cuts in the LP that are not already synced
    if ((numLps_[i] > 0 ||
         ageResetWhileLocked_[i].load(std::memory_order_relaxed) == 1) &&
        !hasSynced_[i]) {
      HighsInt Rlen;
      const HighsInt* Rindex;
      const double* Rvalue;
      getCut(i, Rlen, Rindex, Rvalue);
      // copy cut into something mutable (addCut reorders so can't take const)
      idxs.assign(Rindex, Rindex + Rlen);
      vals.assign(Rvalue, Rvalue + Rlen);
      syncpool.addCut(mipsolver, idxs.data(), vals.data(), Rlen, rhs_[i],
                      rowintegral[i]);
      hasSynced_[i] = true;
    }
  }

  assert((HighsInt)propRows.size() == numPropRows);
}
