/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "mip/HighsLpRelaxation.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include "lp_data/HighsSolve.h"  // For useIpm()
#include "mip/HighsCutPool.h"
#include "mip/HighsDomain.h"
#include "mip/HighsMipSolver.h"
#include "mip/HighsMipSolverData.h"
#include "mip/HighsPseudocost.h"
#include "mip/MipTimer.h"
#include "util/HighsCDouble.h"
#include "util/HighsHash.h"

namespace {

bool hacdcpfXpoolLedgerEnabled() {
  const char* env = std::getenv("HACDCPF_XPOOL_LEDGER");
  if (env != nullptr && env[0] != '\0' && env[0] != '0') return true;
  env = std::getenv("HACDCPF_XTAB_DIAG");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool hacdcpfXpoolRootLedgerOnly() {
  const char* env = std::getenv("HACDCPF_XPOOL_ROOT_LEDGER_ONLY");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

bool hacdcpfXpoolLedgerEnabledForLp(const HighsLpRelaxation& lp) {
  return hacdcpfXpoolLedgerEnabled() &&
         (!hacdcpfXpoolRootLedgerOnly() || lp.getHacdcpfRootLedgerScope());
}

bool hacdcpfRootLpCallbackEnabled(const HighsMipSolver& mipsolver,
                                  const HighsLpRelaxation& lp) {
  if (mipsolver.callback_ == nullptr ||
      !mipsolver.callback_->hacdcpf_root_lp_callback) {
    return false;
  }
  return !hacdcpfXpoolRootLedgerOnly() || lp.getHacdcpfRootLedgerScope();
}


uint64_t hacdcpfXpoolLedgerMix(uint64_t h, uint64_t v) {
  v ^= v >> 33;
  v *= uint64_t{0xff51afd7ed558ccd};
  v ^= v >> 33;
  v *= uint64_t{0xc4ceb9fe1a85ec53};
  v ^= v >> 33;
  return h ^ (v + uint64_t{0x9e3779b97f4a7c15} + (h << 6) + (h >> 2));
}

uint64_t hacdcpfXpoolLedgerDouble(double value) {
  return static_cast<uint64_t>(HighsHashHelpers::double_hash_code(value));
}

bool hacdcpfXpoolStateTraceEnabled() {
  const char* env = std::getenv("HACDCPF_XPOOL_STATE_TRACE");
  return env != nullptr && env[0] != '\0' && env[0] != '0';
}

int hacdcpfXpoolStateTraceTerms() {
  const char* env = std::getenv("HACDCPF_XPOOL_STATE_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 24;
  char* end = nullptr;
  const long parsed = std::strtol(env, &end, 10);
  if (end == env || parsed < 0) return 24;
  return static_cast<int>(std::min<long>(parsed, 512));
}

uint64_t hacdcpfXpoolStateBits(double value) {
  uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "double bit ledger size mismatch");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

char hacdcpfXpoolBasisStatusChar(HighsBasisStatus status) {
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

uint64_t hacdcpfXpoolCutsetPayloadHash(const HighsCutSet& cutset,
                                       HighsInt& nnz) {
  uint64_t hash = uint64_t{0x4849474858504150};
  nnz = 0;
  const HighsInt numcuts = cutset.numCuts();
  for (HighsInt i = 0; i != numcuts; ++i) {
    const HighsInt start = cutset.ARstart_[i];
    const HighsInt end = cutset.ARstart_[i + 1];
    const HighsInt len = end - start;
    nnz += len;
    uint64_t row_hash = uint64_t{0x4849474858505257};
    row_hash = hacdcpfXpoolLedgerMix(row_hash, static_cast<uint64_t>(i));
    row_hash = hacdcpfXpoolLedgerMix(row_hash, static_cast<uint64_t>(len));
    row_hash =
        hacdcpfXpoolLedgerMix(row_hash, hacdcpfXpoolLedgerDouble(cutset.lower_[i]));
    row_hash =
        hacdcpfXpoolLedgerMix(row_hash, hacdcpfXpoolLedgerDouble(cutset.upper_[i]));
    for (HighsInt k = start; k != end; ++k) {
      row_hash = hacdcpfXpoolLedgerMix(
          row_hash, static_cast<uint64_t>(cutset.ARindex_[k]));
      row_hash = hacdcpfXpoolLedgerMix(
          row_hash, hacdcpfXpoolLedgerDouble(cutset.ARvalue_[k]));
    }
    hash = hacdcpfXpoolLedgerMix(hash, row_hash);
  }
  return hash;
}

uint64_t hacdcpfXpoolLpRowPayloadHash(const HighsLp& lp, HighsInt first_row,
                                      HighsInt num_rows, HighsInt& nnz) {
  uint64_t hash = uint64_t{0x4849474858504150};
  nnz = 0;
  if (num_rows <= 0 || first_row < 0 || first_row + num_rows > lp.num_row_)
    return hash;
  std::vector<HighsInt> index(static_cast<std::size_t>(std::max<HighsInt>(0, lp.num_col_)));
  std::vector<double> value(static_cast<std::size_t>(std::max<HighsInt>(0, lp.num_col_)));
  for (HighsInt ord = 0; ord != num_rows; ++ord) {
    HighsInt len = 0;
    lp.a_matrix_.getRow(first_row + ord, len, index.data(), value.data());
    nnz += len;
    uint64_t row_hash = uint64_t{0x4849474858505257};
    row_hash = hacdcpfXpoolLedgerMix(row_hash, static_cast<uint64_t>(ord));
    row_hash = hacdcpfXpoolLedgerMix(row_hash, static_cast<uint64_t>(len));
    row_hash = hacdcpfXpoolLedgerMix(
        row_hash, hacdcpfXpoolLedgerDouble(lp.row_lower_[first_row + ord]));
    row_hash = hacdcpfXpoolLedgerMix(
        row_hash, hacdcpfXpoolLedgerDouble(lp.row_upper_[first_row + ord]));
    for (HighsInt k = 0; k != len; ++k) {
      row_hash =
          hacdcpfXpoolLedgerMix(row_hash, static_cast<uint64_t>(index[k]));
      row_hash =
          hacdcpfXpoolLedgerMix(row_hash, hacdcpfXpoolLedgerDouble(value[k]));
    }
    hash = hacdcpfXpoolLedgerMix(hash, row_hash);
  }
  return hash;
}

const char* hacdcpfXpoolResolveStatusName(HighsLpRelaxation::Status status) {
  switch (status) {
    case HighsLpRelaxation::Status::kNotSet:
      return "not_set";
    case HighsLpRelaxation::Status::kOptimal:
      return "optimal";
    case HighsLpRelaxation::Status::kInfeasible:
      return "infeasible";
    case HighsLpRelaxation::Status::kUnscaledDualFeasible:
      return "unscaled_dual_feasible";
    case HighsLpRelaxation::Status::kUnscaledPrimalFeasible:
      return "unscaled_primal_feasible";
    case HighsLpRelaxation::Status::kUnscaledInfeasible:
      return "unscaled_infeasible";
    case HighsLpRelaxation::Status::kUnbounded:
      return "unbounded";
    case HighsLpRelaxation::Status::kError:
      return "error";
  }
  return "unknown";
}

void hacdcpfLogXpoolResolveLedger(const HighsMipSolver& mipsolver,
                                  const HighsLpRelaxation& lprelax,
                                  HighsLpRelaxation::Status status,
                                  int64_t iter_delta,
                                  int64_t iter_total,
                                  double resolve_ms) {
  if (!hacdcpfXpoolLedgerEnabledForLp(lprelax)) return;
  const HighsInt n = mipsolver.numCol();
  const HighsLp& lp = lprelax.getLp();
  const HighsSolution& sol = lprelax.getSolution();
  const HighsBasis& basis = lprelax.getLpSolver().getBasis();
  if (lp.num_col_ < n || static_cast<HighsInt>(sol.col_value.size()) < n) return;

  HighsInt basic_cols = 0;
  HighsInt lower_cols = 0;
  HighsInt upper_cols = 0;
  HighsInt zero_cols = 0;
  HighsInt other_cols = 0;
  HighsInt fractional = 0;
  uint64_t frontier_hash = uint64_t{0x4849474853465241};
  uint64_t status_hash = uint64_t{0x4849474853535441};
  uint64_t basis_hash = uint64_t{0x4849474858424153};
  std::ostringstream sample;
  sample.precision(12);
  int emitted = 0;
  const double int_tol = mipsolver.mipdata_->feastol;

  for (HighsInt j = 0; j != n; ++j) {
    const HighsBasisStatus st =
        basis.valid && j < static_cast<HighsInt>(basis.col_status.size())
            ? basis.col_status[j]
            : HighsBasisStatus::kNonbasic;
    const char st_char = hacdcpfXpoolBasisStatusChar(st);
    status_hash = hacdcpfXpoolLedgerMix(status_hash, static_cast<uint64_t>(j));
    status_hash =
        hacdcpfXpoolLedgerMix(status_hash, static_cast<uint64_t>(st_char));
    status_hash =
        hacdcpfXpoolLedgerMix(status_hash, hacdcpfXpoolLedgerDouble(sol.col_value[j]));
    basis_hash = hacdcpfXpoolLedgerMix(basis_hash, static_cast<uint64_t>(j));
    basis_hash =
        hacdcpfXpoolLedgerMix(basis_hash, static_cast<uint64_t>(st_char));
    switch (st) {
      case HighsBasisStatus::kBasic:
        ++basic_cols;
        break;
      case HighsBasisStatus::kLower:
        ++lower_cols;
        break;
      case HighsBasisStatus::kUpper:
        ++upper_cols;
        break;
      case HighsBasisStatus::kZero:
        ++zero_cols;
        break;
      case HighsBasisStatus::kNonbasic:
        ++other_cols;
        break;
    }

    if (mipsolver.variableType(j) == HighsVarType::kContinuous) continue;
    const double value = sol.col_value[j];
    const double rounded =
        std::min(lp.col_upper_[j], std::max(lp.col_lower_[j], std::round(value)));
    const double frac = std::abs(value - rounded);
    if (!(frac > int_tol)) continue;
    ++fractional;
    frontier_hash = hacdcpfXpoolLedgerMix(frontier_hash, static_cast<uint64_t>(j));
    frontier_hash =
        hacdcpfXpoolLedgerMix(frontier_hash, hacdcpfXpoolLedgerDouble(value));
    frontier_hash =
        hacdcpfXpoolLedgerMix(frontier_hash, static_cast<uint64_t>(st_char));
    if (emitted < 16) {
      if (emitted++ > 0) sample << ";";
      sample << j << ":" << value << ":r" << rounded << ":f" << frac << ":"
             << st_char;
    }
  }

  std::fprintf(
      stderr,
      "[HIGHS-XPOOL-RESOLVE] status=%s obj=%.17g rows=%lld cols=%lld "
      "nnz=%lld iterDelta=%lld iterTotal=%lld frac=%lld "
      "frontierHash=%016llx statusHash=%016llx basisHash=%016llx "
      "basis=basic%lld:lower%lld:upper%lld:zero%lld:other%lld "
      "sample=[%s] resolveMs=%.6f\n",
      hacdcpfXpoolResolveStatusName(status), lprelax.getObjective(),
      static_cast<long long>(lp.num_row_), static_cast<long long>(lp.num_col_),
      static_cast<long long>(lp.a_matrix_.numNz()),
      static_cast<long long>(iter_delta), static_cast<long long>(iter_total),
      static_cast<long long>(fractional),
      static_cast<unsigned long long>(frontier_hash),
      static_cast<unsigned long long>(status_hash),
      static_cast<unsigned long long>(basis_hash),
      static_cast<long long>(basic_cols), static_cast<long long>(lower_cols),
      static_cast<long long>(upper_cols), static_cast<long long>(zero_cols),
      static_cast<long long>(other_cols), sample.str().c_str(), resolve_ms);
  if (hacdcpfXpoolStateTraceEnabled()) {
    const HighsInt block_size = 64;
    for (HighsInt start = 0; start < n; start += block_size) {
      const HighsInt end = std::min(n, start + block_size);
      uint64_t hash = uint64_t{0x4849474853544154};
      for (HighsInt j = start; j != end; ++j) {
        const HighsBasisStatus st =
            basis.valid && j < static_cast<HighsInt>(basis.col_status.size())
                ? basis.col_status[j]
                : HighsBasisStatus::kNonbasic;
        const char st_char = hacdcpfXpoolBasisStatusChar(st);
        hash = hacdcpfXpoolLedgerMix(hash, static_cast<uint64_t>(j));
        hash = hacdcpfXpoolLedgerMix(hash, static_cast<uint64_t>(st_char));
        hash = hacdcpfXpoolLedgerMix(hash,
                                     hacdcpfXpoolLedgerDouble(sol.col_value[j]));
      }
      std::fprintf(stderr,
                   "[HIGHS-XPOOL-STATE-BLOCK] rows=%lld start=%lld end=%lld "
                   "hash=%016llx\n",
                   static_cast<long long>(lp.num_row_),
                   static_cast<long long>(start), static_cast<long long>(end),
                   static_cast<unsigned long long>(hash));
    }
    const int limit = hacdcpfXpoolStateTraceTerms();
    int emitted_terms = 0;
    for (HighsInt j = 0; j != n && emitted_terms < limit; ++j) {
      const HighsBasisStatus st =
          basis.valid && j < static_cast<HighsInt>(basis.col_status.size())
              ? basis.col_status[j]
              : HighsBasisStatus::kNonbasic;
      const char st_char = hacdcpfXpoolBasisStatusChar(st);
      if (st_char != 'B' && sol.col_value[j] == 0.0) continue;
      std::fprintf(stderr,
                   "[HIGHS-XPOOL-STATE-COL] rows=%lld col=%lld status=%c "
                   "value=%.17g bits=%016llx dh=%08x\n",
                   static_cast<long long>(lp.num_row_),
                   static_cast<long long>(j), st_char, sol.col_value[j],
                   static_cast<unsigned long long>(
                       hacdcpfXpoolStateBits(sol.col_value[j])),
                   HighsHashHelpers::double_hash_code(sol.col_value[j]));
      ++emitted_terms;
    }
  }
  const char* full_state_env = std::getenv("HACDCPF_XPOOL_FULL_STATE_REPLAY");
  if (full_state_env != nullptr && full_state_env[0] != '\0' &&
      full_state_env[0] != '0') {
    for (HighsInt j = 0; j != n; ++j) {
      const HighsBasisStatus st =
          basis.valid && j < static_cast<HighsInt>(basis.col_status.size())
              ? basis.col_status[j]
              : HighsBasisStatus::kNonbasic;
      const char st_char = hacdcpfXpoolBasisStatusChar(st);
      std::fprintf(stderr,
                   "[HIGHS-XPOOL-BASIS-COL] rows=%lld col=%lld status=%c "
                   "value=%.17g bits=%016llx dh=%08x\n",
                   static_cast<long long>(lp.num_row_),
                   static_cast<long long>(j), st_char, sol.col_value[j],
                   static_cast<unsigned long long>(
                       hacdcpfXpoolStateBits(sol.col_value[j])),
                   HighsHashHelpers::double_hash_code(sol.col_value[j]));
    }
    for (HighsInt row = 0; row != lp.num_row_; ++row) {
      const HighsBasisStatus st =
          basis.valid && row < static_cast<HighsInt>(basis.row_status.size())
              ? basis.row_status[row]
              : HighsBasisStatus::kNonbasic;
      const char st_char = hacdcpfXpoolBasisStatusChar(st);
      const double value =
          row < static_cast<HighsInt>(sol.row_value.size())
              ? sol.row_value[row]
              : std::numeric_limits<double>::quiet_NaN();
      std::fprintf(stderr,
                   "[HIGHS-XPOOL-BASIS-ROW] rows=%lld row=%lld status=%c "
                   "value=%.17g bits=%016llx dh=%08x\n",
                   static_cast<long long>(lp.num_row_),
                   static_cast<long long>(row), st_char, value,
                   static_cast<unsigned long long>(hacdcpfXpoolStateBits(value)),
                   HighsHashHelpers::double_hash_code(value));
    }
  }
  std::fflush(stderr);
}

}  // namespace

void HighsLpRelaxation::logHacdcpfUserCutResolveLedger(
    const char* scope, const char* event, HighsInt round, int64_t node_count,
    HighsInt depth, Status resolve_status, int64_t iter_delta,
    int64_t iter_total, double resolve_ms) const {
  if (!hacdcpfUserCutLedgerEnabled()) return;
  const HighsInt n = mipsolver.numCol();
  const HighsLp& lp = getLp();
  const HighsSolution& sol = getSolution();
  const HighsBasis& basis = getLpSolver().getBasis();
  if (lp.num_col_ < n || static_cast<HighsInt>(sol.col_value.size()) < n) {
    return;
  }

  HighsInt basic_cols = 0;
  HighsInt lower_cols = 0;
  HighsInt upper_cols = 0;
  HighsInt zero_cols = 0;
  HighsInt other_cols = 0;
  HighsInt fractional = 0;
  uint64_t frontier_hash = uint64_t{0x4849474855434652};
  uint64_t status_hash = uint64_t{0x4849474855435354};
  uint64_t basis_hash = uint64_t{0x4849474855434241};
  const double int_tol = mipsolver.mipdata_->feastol;

  for (HighsInt j = 0; j != n; ++j) {
    const HighsBasisStatus st =
        basis.valid && j < static_cast<HighsInt>(basis.col_status.size())
            ? basis.col_status[j]
            : HighsBasisStatus::kNonbasic;
    const char st_char = hacdcpfXpoolBasisStatusChar(st);
    status_hash = hacdcpfXpoolLedgerMix(status_hash, static_cast<uint64_t>(j));
    status_hash =
        hacdcpfXpoolLedgerMix(status_hash, static_cast<uint64_t>(st_char));
    status_hash =
        hacdcpfXpoolLedgerMix(status_hash,
                              hacdcpfXpoolLedgerDouble(sol.col_value[j]));
    basis_hash = hacdcpfXpoolLedgerMix(basis_hash, static_cast<uint64_t>(j));
    basis_hash =
        hacdcpfXpoolLedgerMix(basis_hash, static_cast<uint64_t>(st_char));
    switch (st) {
      case HighsBasisStatus::kBasic:
        ++basic_cols;
        break;
      case HighsBasisStatus::kLower:
        ++lower_cols;
        break;
      case HighsBasisStatus::kUpper:
        ++upper_cols;
        break;
      case HighsBasisStatus::kZero:
        ++zero_cols;
        break;
      case HighsBasisStatus::kNonbasic:
        ++other_cols;
        break;
    }

    if (mipsolver.variableType(j) == HighsVarType::kContinuous) continue;
    const double value = sol.col_value[j];
    const double rounded =
        std::min(lp.col_upper_[j], std::max(lp.col_lower_[j], std::round(value)));
    const double frac = std::abs(value - rounded);
    if (!(frac > int_tol)) continue;
    ++fractional;
    frontier_hash = hacdcpfXpoolLedgerMix(frontier_hash, static_cast<uint64_t>(j));
    frontier_hash =
        hacdcpfXpoolLedgerMix(frontier_hash, hacdcpfXpoolLedgerDouble(value));
    frontier_hash =
        hacdcpfXpoolLedgerMix(frontier_hash, static_cast<uint64_t>(st_char));
  }

  std::fprintf(
      stderr,
      "[HIGHS-USERCUT-RESOLVE] scope=%s event=%s round=%lld node=%lld "
      "depth=%lld status=%s obj=%.17g rows=%lld cols=%lld nnz=%lld "
      "iterDelta=%lld iterTotal=%lld frac=%lld frontierHash=%016llx "
      "statusHash=%016llx basisHash=%016llx "
      "basis=basic%lld:lower%lld:upper%lld:zero%lld:other%lld "
      "resolveMs=%.6f\n",
      scope != nullptr ? scope : "unknown",
      event != nullptr ? event : "unknown", static_cast<long long>(round),
      static_cast<long long>(node_count), static_cast<long long>(depth),
      hacdcpfXpoolResolveStatusName(resolve_status), getObjective(),
      static_cast<long long>(lp.num_row_), static_cast<long long>(lp.num_col_),
      static_cast<long long>(lp.a_matrix_.numNz()),
      static_cast<long long>(iter_delta), static_cast<long long>(iter_total),
      static_cast<long long>(fractional),
      static_cast<unsigned long long>(frontier_hash),
      static_cast<unsigned long long>(status_hash),
      static_cast<unsigned long long>(basis_hash),
      static_cast<long long>(basic_cols), static_cast<long long>(lower_cols),
      static_cast<long long>(upper_cols), static_cast<long long>(zero_cols),
      static_cast<long long>(other_cols), resolve_ms);
  std::fflush(stderr);
}

void HighsLpRelaxation::getCutPool(HighsInt& num_col, HighsInt& num_cut,
                                   std::vector<double>& cut_lower,
                                   std::vector<double>& cut_upper,
                                   HighsSparseMatrix& cut_matrix) const {
  const HighsLp& lp = lpsolver.getLp();
  num_col = lp.num_col_;
  HighsInt num_lp_row = lp.num_row_;
  num_cut = 0;
  for (HighsInt iRow = 0; iRow < num_lp_row; iRow++) {
    if (lprows[iRow].origin == LpRow::Origin::kCutPool) ++num_cut;
  }
  cut_lower.resize(num_cut);
  cut_upper.resize(num_cut);
  // Get a map from row index to cut row index
  std::vector<HighsInt> cut_row_index;
  cut_row_index.assign(num_lp_row, -1);
  HighsInt cut_num = 0;
  for (HighsInt iRow = 0; iRow < lp.num_row_; iRow++) {
    if (lprows[iRow].origin != LpRow::Origin::kCutPool) continue;
    cut_row_index[iRow] = cut_num;
    cut_lower[cut_num] = lp.row_lower_[iRow];
    cut_upper[cut_num] = lp.row_upper_[iRow];
    cut_num++;
  }
  assert(cut_num == num_cut);

  cut_matrix.num_col_ = lp.num_col_;
  cut_matrix.num_row_ = num_cut;
  cut_matrix.format_ = MatrixFormat::kRowwise;

  std::vector<HighsInt> cut_matrix_length;
  cut_matrix_length.assign(num_cut, 0);
  for (HighsInt iCol = 0; iCol < lp.num_col_; iCol++) {
    for (HighsInt iEl = lp.a_matrix_.start_[iCol];
         iEl < lp.a_matrix_.start_[iCol + 1]; iEl++) {
      HighsInt iCut = cut_row_index[lp.a_matrix_.index_[iEl]];
      if (iCut >= 0) cut_matrix_length[iCut]++;
    }
  }
  cut_matrix.start_.resize(num_cut + 1);
  cut_matrix.start_[0] = 0;
  HighsInt num_cut_nz = 0;
  for (HighsInt iCut = 0; iCut < num_cut; iCut++) {
    HighsInt length = cut_matrix_length[iCut];
    cut_matrix_length[iCut] = cut_matrix.start_[iCut];
    num_cut_nz += length;
    cut_matrix.start_[iCut + 1] = num_cut_nz;
  }
  cut_matrix.index_.resize(num_cut_nz);
  cut_matrix.value_.resize(num_cut_nz);
  for (HighsInt iCol = 0; iCol < lp.num_col_; iCol++) {
    for (HighsInt iEl = lp.a_matrix_.start_[iCol];
         iEl < lp.a_matrix_.start_[iCol + 1]; iEl++) {
      HighsInt iCut = cut_row_index[lp.a_matrix_.index_[iEl]];
      if (iCut >= 0) {
        cut_matrix.index_[cut_matrix_length[iCut]] = iCol;
        cut_matrix.value_[cut_matrix_length[iCut]] = lp.a_matrix_.value_[iEl];
        cut_matrix_length[iCut]++;
      }
    }
  }
}

void HighsLpRelaxation::LpRow::get(const HighsMipSolver& mipsolver,
                                   const std::vector<HacdcpfLocalCut>& localCuts,
                                   HighsInt& len, const HighsInt*& inds,
                                   const double*& vals) const {
  switch (origin) {
    case kCutPool:
      mipsolver.mipdata_->cutpool.getCut(index, len, inds, vals);
      break;
    case kModel:
      mipsolver.mipdata_->getRow(index, len, inds, vals);
      break;
    case kLocalCut:
      assert(index >= 0 && index < static_cast<HighsInt>(localCuts.size()));
      len = localCuts[static_cast<std::size_t>(index)].size();
      inds = localCuts[static_cast<std::size_t>(index)].indices.data();
      vals = localCuts[static_cast<std::size_t>(index)].values.data();
      break;
  };
}

HighsInt HighsLpRelaxation::LpRow::getRowLen(
    const HighsMipSolver& mipsolver,
    const std::vector<HacdcpfLocalCut>& localCuts) const {
  switch (origin) {
    case kCutPool:
      return mipsolver.mipdata_->cutpool.getRowLength(index);
    case kModel:
      return mipsolver.mipdata_->ARstart_[index + 1] -
             mipsolver.mipdata_->ARstart_[index];
    case kLocalCut:
      assert(index >= 0 && index < static_cast<HighsInt>(localCuts.size()));
      return localCuts[static_cast<std::size_t>(index)].size();
  };

  assert(false);
  return -1;
}

bool HighsLpRelaxation::LpRow::isIntegral(
    const HighsMipSolver& mipsolver,
    const std::vector<HacdcpfLocalCut>& localCuts) const {
  switch (origin) {
    case kCutPool:
      return mipsolver.mipdata_->cutpool.cutIsIntegral(index);
    case kModel:
      return (mipsolver.mipdata_->rowintegral[index] != 0);
    case kLocalCut:
      assert(index >= 0 && index < static_cast<HighsInt>(localCuts.size()));
      return localCuts[static_cast<std::size_t>(index)].integral != 0;
  };

  assert(false);
  return false;
}

double HighsLpRelaxation::LpRow::getMaxAbsVal(
    const HighsMipSolver& mipsolver,
    const std::vector<HacdcpfLocalCut>& localCuts) const {
  switch (origin) {
    case kCutPool:
      return mipsolver.mipdata_->cutpool.getMaxAbsCutCoef(index);
    case kModel:
      return mipsolver.mipdata_->maxAbsRowCoef[index];
    case kLocalCut:
      assert(index >= 0 && index < static_cast<HighsInt>(localCuts.size()));
      return localCuts[static_cast<std::size_t>(index)].maxAbsVal();
  };

  assert(false);
  return 0.0;
}

double HighsLpRelaxation::slackLower(HighsInt row) const {
  switch (lprows[row].origin) {
    case LpRow::kCutPool:
      return mipsolver.mipdata_->domain.getMinCutActivity(
          mipsolver.mipdata_->cutpool, lprows[row].index);
    case LpRow::kLocalCut:
      return rowLower(row);
    case LpRow::kModel:
      double rowlower = rowLower(row);
      if (rowlower != -kHighsInf) return rowlower;
      return mipsolver.mipdata_->domain.getMinActivity(lprows[row].index);
  };

  assert(false);
  return -kHighsInf;
}

double HighsLpRelaxation::slackUpper(HighsInt row) const {
  double rowupper = rowUpper(row);
  switch (lprows[row].origin) {
    case LpRow::kCutPool:
      return rowupper;
    case LpRow::kLocalCut:
      return rowupper;
    case LpRow::kModel:
      if (rowupper != kHighsInf) return rowupper;
      return mipsolver.mipdata_->domain.getMaxActivity(lprows[row].index);
  };

  assert(false);
  return kHighsInf;
}

// Used when creating the instance of HighsMipSolverData in
// HighsMipSolver::run()
//
// Parameter creating_mip_solver_data is false by default, and is only
// set true when the HighsLpRelaxation instance is created as part of
// a new HighsMipSolverData instance
HighsLpRelaxation::HighsLpRelaxation(const HighsMipSolver& mipsolver)
    : mipsolver(mipsolver) {
  lpsolver.setOptionValue("output_flag", false);
  lpsolver.setOptionValue("random_seed", mipsolver.options_mip_->random_seed);
  // Set primal feasibility tolerance for LP solves according to
  // mip_feasibility_tolerance, and smaller tolerance for dual
  // feasibility
  double mip_primal_feasibility_tolerance =
      mipsolver.options_mip_->mip_feasibility_tolerance;
  double mip_dual_feasibility_tolerance =
      mipsolver.options_mip_->mip_feasibility_tolerance * 0.1;
  lpsolver.setOptionValue("primal_feasibility_tolerance",
                          mip_primal_feasibility_tolerance);
  lpsolver.setOptionValue("dual_feasibility_tolerance",
                          mip_dual_feasibility_tolerance);
  status = Status::kNotSet;
  numlpiters = 0;
  avgSolveIters = 0;
  numSolved = 0;
  epochs = 0;
  maxNumFractional = 0;
  lastAgeCall = 0;
  objective = -kHighsInf;
  currentbasisstored = false;
  adjustSymBranchingCol = true;
  solved_first_lp = true;
  row_ep.size = 0;
}

// Used in LP-based primal heuristics (RINS, RENS, tryRoundedPoint,
// randomizedRounding, shifting, feasibilityPump) to create a local LP
// relaxation using the MIP solver's LP relaxation
HighsLpRelaxation::HighsLpRelaxation(const HighsLpRelaxation& other)
    : mipsolver(other.mipsolver),
      lprows(other.lprows),
      fractionalints(other.fractionalints),
      objective(other.objective),
      basischeckpoint(other.basischeckpoint),
      currentbasisstored(other.currentbasisstored),
      adjustSymBranchingCol(other.adjustSymBranchingCol) {
  lpsolver.setOptionValue("output_flag", false);
  lpsolver.passOptions(other.lpsolver.getOptions());
  lpsolver.passModel(other.lpsolver.getLp());
  lpsolver.setBasis(other.lpsolver.getBasis());
  colLbBuffer.resize(mipsolver.numCol());
  colUbBuffer.resize(mipsolver.numCol());
  status = Status::kNotSet;
  numlpiters = 0;
  avgSolveIters = 0;
  numSolved = 0;
  epochs = 0;
  maxNumFractional = 0;
  lastAgeCall = 0;
  objective = -kHighsInf;
  solved_first_lp = true;
  row_ep.size = 0;
}

void HighsLpRelaxation::loadModel() {
  HighsLp lpmodel = *mipsolver.model_;
  lpmodel.col_lower_ = mipsolver.mipdata_->domain.col_lower_;
  lpmodel.col_upper_ = mipsolver.mipdata_->domain.col_upper_;
  lpmodel.offset_ = 0;
  lprows.clear();
  lprows.reserve(lpmodel.num_row_);
  for (HighsInt i = 0; i != lpmodel.num_row_; ++i)
    lprows.push_back(LpRow::model(i));
  lpmodel.integrality_.clear();
  HighsInt num_col = lpmodel.num_col_;
  lpsolver.clearSolver();
  lpsolver.clearModel();
  lpsolver.passModel(std::move(lpmodel));
  colLbBuffer.resize(num_col);
  colUbBuffer.resize(num_col);
}

void HighsLpRelaxation::resetToGlobalDomain() {
  lpsolver.changeColsBounds(0, mipsolver.numCol() - 1,
                            mipsolver.mipdata_->domain.col_lower_.data(),
                            mipsolver.mipdata_->domain.col_upper_.data());
}

void HighsLpRelaxation::computeBasicDegenerateDuals(double threshold,
                                                    HighsDomain* localdom) {
  if (!lpsolver.hasInvert()) return;

  HighsInt k = 0;
  const HighsLp& lp = lpsolver.getLp();
  const HighsBasis& basis = lpsolver.getBasis();
  HighsSolution& solution = const_cast<HighsSolution&>(lpsolver.getSolution());
  for (HighsInt col : mipsolver.mipdata_->integral_cols) {
    if (basis.col_status[col] != HighsBasisStatus::kBasic) continue;
    const double lb = lp.col_lower_[col];
    const double ub = lp.col_upper_[col];
    if (ub - lb < mipsolver.mipdata_->feastol) continue;

    if (solution.col_value[col] - lb < ub - solution.col_value[col]) {
      if (solution.col_value[col] > lb + mipsolver.mipdata_->feastol) continue;

      solution.col_dual[col] = 1.0;
      ++k;
    } else {
      if (solution.col_value[col] < ub - mipsolver.mipdata_->feastol) continue;

      solution.col_dual[col] = -1.0;
      ++k;
    }
  }

  if (k == 0) return;

  const HighsInt num_row = lp.num_row_;
  const HighsInt num_col = lp.num_col_;

  if (row_ep.size < num_row) {
    row_ep.setup(num_row);

    if ((HighsInt)row_ap.values.size() < num_col) {
      row_ap.setDimension(num_col);
      dualproofvals.reserve(num_col);
      dualproofinds.reserve(num_col);
    }
  }

  const HighsInt* basicIndex = lpsolver.getBasicVariablesArray();

  for (HighsInt row = 0; k > 0; row++) {
    HighsInt var = basicIndex[row];
    if (var >= num_col) continue;
    if (std::fabs(solution.col_dual[var]) != 1.0) continue;

    --k;
    lpsolver.getBasisInverseRowSparse(row, row_ep);

    double sign = solution.col_dual[var];
    solution.col_dual[var] = 0.0;
    double degenerateColDual = kHighsInf;
    for (HighsInt i = 0; i < row_ep.count; ++i) {
      HighsInt iRow = row_ep.index[i];
      const double lb = lp.row_lower_[iRow];
      const double ub = lp.row_upper_[iRow];
      if (lb == ub) continue;
      const double dual = solution.row_dual[iRow];

      double val = -sign * row_ep.array[iRow];
      if (val > 0) {
        if (solution.row_value[iRow] - lb > mipsolver.mipdata_->feastol) {
          degenerateColDual = std::min(degenerateColDual, -dual / val);
          if (degenerateColDual < threshold) break;
        }
      } else {
        if (ub - solution.row_value[iRow] > mipsolver.mipdata_->feastol) {
          degenerateColDual = std::min(degenerateColDual, -dual / val);
          if (degenerateColDual < threshold) break;
        }
      }
    }

    if (degenerateColDual < threshold) continue;

    row_ap.clear();
    for (HighsInt i = 0; i < row_ep.count; ++i) {
      HighsInt iRow = row_ep.index[i];

      HighsInt len;
      const HighsInt* inds;
      const double* vals;
      getRow(iRow, len, inds, vals);

      for (HighsInt j = 0; j < len; ++j)
        row_ap.add(inds[j], row_ep.array[iRow] * vals[j]);
    }

    double eps = mipsolver.mipdata_->epsilon;
    row_ap.cleanup(
        [eps](HighsInt, double val) { return std::fabs(val) <= eps; });

    for (HighsInt iCol : row_ap.nonzeroinds) {
      if (iCol == var) continue;

      const double lb = lp.col_lower_[iCol];
      const double ub = lp.col_upper_[iCol];
      if (lb == ub) continue;
      const double dual = solution.col_dual[iCol];

      double val = sign * row_ap.getValue(iCol);
      if (val > mipsolver.mipdata_->epsilon) {
        if (solution.col_value[iCol] - lb > mipsolver.mipdata_->feastol) {
          degenerateColDual = std::min(degenerateColDual, -dual / val);
          if (degenerateColDual < threshold) break;
        }
      } else if (val < -mipsolver.mipdata_->epsilon) {
        if (ub - solution.col_value[iCol] > mipsolver.mipdata_->feastol) {
          degenerateColDual = std::min(degenerateColDual, -dual / val);
          if (degenerateColDual < threshold) break;
        }
      }
    }

    if (degenerateColDual < threshold) continue;

    if (degenerateColDual == kHighsInf && localdom) {
      HighsCDouble rhs = 0;
      for (HighsInt i = 0; i < row_ep.count; ++i) {
        HighsInt iRow = row_ep.index[i];
        const double lb = lp.row_lower_[iRow];
        const double ub = lp.row_upper_[iRow];

        double val = sign * row_ep.array[iRow];
        if (ub == lb || val > mipsolver.mipdata_->epsilon) {
          rhs += val * ub;
        } else if (val < -mipsolver.mipdata_->epsilon) {
          rhs += val * lb;
        } else {
          rhs += val * solution.row_value[iRow];
        }
      }

      dualproofvals.resize(row_ap.nonzeroinds.size());
      for (HighsInt i = 0; i < (HighsInt)row_ap.nonzeroinds.size(); ++i)
        dualproofvals[i] = sign * row_ap.getValue(row_ap.nonzeroinds[i]);

      HighsDomainChange domchg;
      domchg.column = var;
      if (sign == 1.0) {
        domchg.boundtype = HighsBoundType::kUpper;
        domchg.boundval = lp.col_lower_[var];
      } else {
        domchg.boundtype = HighsBoundType::kLower;
        domchg.boundval = lp.col_upper_[var];
      }

      localdom->conflictAnalyzeReconvergence(
          domchg, row_ap.nonzeroinds.data(), dualproofvals.data(),
          row_ap.nonzeroinds.size(), double(rhs),
          mipsolver.mipdata_->conflictPool);

      continue;
    }

    solution.col_dual[var] = sign * double(degenerateColDual);
  }
}

double HighsLpRelaxation::computeBestEstimate(const HighsPseudocost& ps) const {
  HighsCDouble estimate = objective;

  if (!fractionalints.empty()) {
    // because the pseudocost may be zero, we add an offset to the pseudocost so
    // that we always have some part of the estimate depending on the
    // fractionality.

    HighsCDouble increase = 0.0;
    double offset =
        mipsolver.mipdata_->feastol * std::max(std::abs(objective), 1.0) /
        static_cast<double>(mipsolver.mipdata_->integral_cols.size());

    for (const std::pair<HighsInt, double>& f : fractionalints) {
      increase += std::min(ps.getPseudocostUp(f.first, f.second, offset),
                           ps.getPseudocostDown(f.first, f.second, offset));
    }

    estimate += double(increase);
  }

  return double(estimate);
}

double HighsLpRelaxation::computeLPDegneracy(
    const HighsDomain& localdomain) const {
  if (!lpsolver.getSolution().dual_valid || !lpsolver.getBasis().valid) {
    return 1.0;
  }

  double dualFeasTol = lpsolver.getInfo().max_dual_infeasibility;

  const HighsBasis& basis = lpsolver.getBasis();
  const HighsSolution& sol = lpsolver.getSolution();

  HighsInt numFixedRows = 0;
  HighsInt numInequalities = 0;
  HighsInt numBasicEqualities = 0;

  for (HighsInt i = 0; i < numRows(); ++i) {
    if (lpsolver.getLp().row_lower_[i] != lpsolver.getLp().row_upper_[i]) {
      ++numInequalities;

      if (basis.row_status[i] != HighsBasisStatus::kBasic) {
        if (std::abs(sol.row_dual[i]) > dualFeasTol) ++numFixedRows;
      }
    } else
      numBasicEqualities += basis.row_status[i] == HighsBasisStatus::kBasic;
  }

  HighsInt numAlreadyFixedCols = 0;
  HighsInt numFixedCols = 0;
  for (HighsInt i = 0; i < numCols(); ++i) {
    if (basis.col_status[i] != HighsBasisStatus::kBasic) {
      if (std::abs(sol.col_dual[i]) > dualFeasTol)
        ++numFixedCols;
      else if (localdomain.col_lower_[i] == localdomain.col_upper_[i])
        ++numAlreadyFixedCols;
    }
  }

  HighsInt base = numCols() - numAlreadyFixedCols + numInequalities +
                  numBasicEqualities - numRows();

  double degenerateColumnShare =
      base > 0 ? 1.0 - double(numFixedCols + numFixedRows) / base : 0.0;

  double varConsRatio =
      numRows() > 0
          ? double(numCols() + numInequalities + numBasicEqualities -
                   numFixedCols - numFixedRows - numAlreadyFixedCols) /
                numRows()
          : 1.0;
  double fac1 = degenerateColumnShare < 0.8
                    ? 1.0
                    : std::pow(10.0, 10 * (degenerateColumnShare - 0.7));
  double fac2 = varConsRatio < 2.0 ? 1.0 : 10.0 * varConsRatio;

  return fac1 * fac2;
}

void HighsLpRelaxation::addCuts(HighsCutSet& cutset) {
  HighsInt numcuts = cutset.numCuts();
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
  assert(lpsolver.getLp().num_row_ == (HighsInt)lprows.size());
  if (numcuts > 0) {
    const auto append_t0 = std::chrono::steady_clock::now();
    HighsInt selectednnz = 0;
    const uint64_t selected_payload_hash =
        hacdcpfXpoolCutsetPayloadHash(cutset, selectednnz);
    const HighsLp& lp_before = lpsolver.getLp();
    const HighsInt rows_before = lp_before.num_row_;
    const HighsInt nnz_before = lp_before.a_matrix_.numNz();
    status = Status::kNotSet;
    currentbasisstored = false;
    basischeckpoint.reset();

    lprows.reserve(lprows.size() + numcuts);
    for (HighsInt i = 0; i != numcuts; ++i)
      lprows.push_back(LpRow::cut(cutset.cutindices[i]));

    bool success =
        lpsolver.addRows(numcuts, cutset.lower_.data(), cutset.upper_.data(),
                         cutset.ARvalue_.size(), cutset.ARstart_.data(),
                         cutset.ARindex_.data(),
                         cutset.ARvalue_.data()) == HighsStatus::kOk;
    assert(success);
    (void)success;
    assert(lpsolver.getLp().num_row_ ==
           (HighsInt)lpsolver.getLp().row_lower_.size());
    if (hacdcpfXpoolLedgerEnabledForLp(*this)) {
      const double append_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - append_t0)
              .count();
      const HighsLp& lp_after = lpsolver.getLp();
      HighsInt appendednnz = 0;
      const uint64_t appended_payload_hash = hacdcpfXpoolLpRowPayloadHash(
          lp_after, rows_before, numcuts, appendednnz);
      std::fprintf(
          stderr,
          "[HIGHS-XPOOL-APPEND] selected=%lld preRows=%lld postRows=%lld "
          "preNnz=%lld postNnz=%lld selNnz=%lld appNnz=%lld "
          "selPayloadHash=%016llx appPayloadHash=%016llx success=%d "
          "appendMs=%.6f\n",
          static_cast<long long>(numcuts), static_cast<long long>(rows_before),
          static_cast<long long>(lp_after.num_row_),
          static_cast<long long>(nnz_before),
          static_cast<long long>(lp_after.a_matrix_.numNz()),
          static_cast<long long>(selectednnz),
          static_cast<long long>(appendednnz),
          static_cast<unsigned long long>(selected_payload_hash),
          static_cast<unsigned long long>(appended_payload_hash),
          success ? 1 : 0, append_ms);
      std::fflush(stderr);
    }
    if (hacdcpfRootLpCallbackEnabled(mipsolver, *this)) {
      mipsolver.callback_->hacdcpf_root_lp_callback(
          "append", this, &cutset,
          mipsolver.callback_->hacdcpf_root_lp_callback_data);
    }
    cutset.clear();
  }
}

void HighsLpRelaxation::addLocalCuts(HighsCutSet& cutset) {
  HighsInt numcuts = cutset.numCuts();
  if (numcuts == 0) return;
  std::vector<HacdcpfLocalCut> cuts;
  cuts.reserve(static_cast<std::size_t>(numcuts));
  for (HighsInt i = 0; i != numcuts; ++i) {
    const std::size_t start =
        static_cast<std::size_t>(cutset.ARstart_[static_cast<std::size_t>(i)]);
    const std::size_t end = static_cast<std::size_t>(
        cutset.ARstart_[static_cast<std::size_t>(i + 1)]);
    HacdcpfLocalCut cut;
    cut.lower = cutset.lower_[static_cast<std::size_t>(i)];
    cut.upper = cutset.upper_[static_cast<std::size_t>(i)];
    cut.integral =
        !cutset.hacdcpf_integral_.empty()
            ? cutset.hacdcpf_integral_[static_cast<std::size_t>(i)]
            : 0;
    cut.propagate =
        !cutset.hacdcpf_propagate_.empty()
            ? cutset.hacdcpf_propagate_[static_cast<std::size_t>(i)]
            : 1;
    cut.indices.insert(cut.indices.end(), cutset.ARindex_.begin() + start,
                       cutset.ARindex_.begin() + end);
    cut.values.insert(cut.values.end(), cutset.ARvalue_.begin() + start,
                      cutset.ARvalue_.begin() + end);
    cut.row_hash = hacdcpfCutSetRowHash(cutset, i);
    cut.row_sig = hacdcpfCutSetRowSig(cutset, i);
    cut.proof_family =
        i < static_cast<HighsInt>(cutset.hacdcpf_proof_family_.size())
            ? cutset.hacdcpf_proof_family_[static_cast<std::size_t>(i)]
            : std::string{};
    cut.proof_key =
        i < static_cast<HighsInt>(cutset.hacdcpf_proof_key_.size())
            ? cutset.hacdcpf_proof_key_[static_cast<std::size_t>(i)]
            : std::string{};
    cuts.push_back(std::move(cut));
  }
  addLocalCuts(cuts);
  cutset.clear();
}

void HighsLpRelaxation::addLocalCuts(const std::vector<HacdcpfLocalCut>& cuts) {
  HighsInt numcuts = static_cast<HighsInt>(cuts.size());
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
  assert(lpsolver.getLp().num_row_ == (HighsInt)lprows.size());
  if (numcuts <= 0) return;

  status = Status::kNotSet;
  currentbasisstored = false;
  basischeckpoint.reset();

  std::vector<double> lower(static_cast<std::size_t>(numcuts));
  std::vector<double> upper(static_cast<std::size_t>(numcuts));
  std::vector<HighsInt> ARstart(static_cast<std::size_t>(numcuts + 1));
  std::vector<HighsInt> ARindex;
  std::vector<double> ARvalue;
  HighsInt nnz = 0;
  for (HighsInt i = 0; i != numcuts; ++i) {
    const HacdcpfLocalCut& cut = cuts[static_cast<std::size_t>(i)];
    lower[static_cast<std::size_t>(i)] = cut.lower;
    upper[static_cast<std::size_t>(i)] = cut.upper;
    ARstart[static_cast<std::size_t>(i)] = nnz;
    nnz += cut.size();
  }
  ARstart[static_cast<std::size_t>(numcuts)] = nnz;
  ARindex.reserve(static_cast<std::size_t>(nnz));
  ARvalue.reserve(static_cast<std::size_t>(nnz));
  for (const HacdcpfLocalCut& cut : cuts) {
    ARindex.insert(ARindex.end(), cut.indices.begin(), cut.indices.end());
    ARvalue.insert(ARvalue.end(), cut.values.begin(), cut.values.end());
  }

  const HighsInt local_index_base =
      static_cast<HighsInt>(localCuts.size());
  localCuts.insert(localCuts.end(), cuts.begin(), cuts.end());
  lprows.reserve(lprows.size() + numcuts);
  for (HighsInt i = 0; i != numcuts; ++i)
    lprows.push_back(LpRow::localCut(local_index_base + i));

  bool success =
      lpsolver.addRows(numcuts, lower.data(), upper.data(), ARvalue.size(),
                       ARstart.data(), ARindex.data(), ARvalue.data()) ==
      HighsStatus::kOk;
  assert(success);
  (void)success;
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
}

void HighsLpRelaxation::setLocalCuts(
    const std::vector<HacdcpfLocalCut>& cuts) {
  removeLocalCuts();
  addLocalCuts(cuts);
}

void HighsLpRelaxation::removeObsoleteRows(bool notifyPool) {
  HighsInt nlprows = numRows();
  HighsInt nummodelrows = getNumModelRows();
  std::vector<HighsInt> deletemask;

  HighsInt ndelcuts = 0;
  for (HighsInt i = nummodelrows; i != nlprows; ++i) {
    assert(lprows[i].origin == LpRow::Origin::kCutPool ||
           lprows[i].origin == LpRow::Origin::kLocalCut);
    if (lprows[i].origin == LpRow::Origin::kLocalCut) continue;
    if (lpsolver.getBasis().row_status[i] == HighsBasisStatus::kBasic) {
      if (ndelcuts == 0) deletemask.resize(nlprows);
      ++ndelcuts;
      deletemask[i] = 1;
      if (notifyPool && lprows[i].origin == LpRow::Origin::kCutPool)
        mipsolver.mipdata_->cutpool.lpCutRemoved(lprows[i].index);
    }
  }

  removeCuts(ndelcuts, deletemask);
}

void HighsLpRelaxation::removeCuts(HighsInt ndelcuts,
                                   std::vector<HighsInt>& deletemask) {
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
  if (ndelcuts > 0) {
    const bool hacdcpf_delete_ledger = hacdcpfXpoolLedgerEnabledForLp(*this);
    const HighsLp& lp_before = lpsolver.getLp();
    const HighsInt rows_before = lp_before.num_row_;
    const HighsInt nnz_before = lp_before.a_matrix_.numNz();
    std::vector<HighsInt> hacdcpf_deleted_rows;
    std::vector<HighsInt> hacdcpf_deleted_cut_indices;
    if (hacdcpf_delete_ledger) {
      hacdcpf_deleted_rows.reserve(ndelcuts);
      hacdcpf_deleted_cut_indices.reserve(ndelcuts);
      for (HighsInt i = 0; i != rows_before; ++i) {
        if (i < (HighsInt)deletemask.size() && deletemask[i] == 1) {
          hacdcpf_deleted_rows.push_back(i);
          hacdcpf_deleted_cut_indices.push_back(
              i < (HighsInt)lprows.size() ? lprows[i].index : HighsInt{-1});
        }
      }
    }
    HighsBasis basis = lpsolver.getBasis();
    HighsInt nlprows = lpsolver.getNumRow();
    lpsolver.deleteRows(deletemask.data());
    if (hacdcpf_delete_ledger) {
      const HighsLp& lp_after_delete = lpsolver.getLp();
      std::fprintf(stderr,
                   "[HIGHS-XPOOL-DELETE] preRows=%lld postRows=%lld "
                   "preNnz=%lld postNnz=%lld deleted=%lld rows=[",
                   static_cast<long long>(rows_before),
                   static_cast<long long>(lp_after_delete.num_row_),
                   static_cast<long long>(nnz_before),
                   static_cast<long long>(lp_after_delete.a_matrix_.numNz()),
                   static_cast<long long>(hacdcpf_deleted_rows.size()));
      for (HighsInt k = 0; k < (HighsInt)hacdcpf_deleted_rows.size(); ++k) {
        if (k) std::fprintf(stderr, ",");
        std::fprintf(stderr, "%lld",
                     static_cast<long long>(hacdcpf_deleted_rows[k]));
      }
      std::fprintf(stderr, "] cutIdx=[");
      for (HighsInt k = 0; k < (HighsInt)hacdcpf_deleted_cut_indices.size();
           ++k) {
        if (k) std::fprintf(stderr, ",");
        std::fprintf(stderr, "%lld",
                     static_cast<long long>(hacdcpf_deleted_cut_indices[k]));
      }
      std::fprintf(stderr, "]\n");
      std::fflush(stderr);
    }
    for (HighsInt i = mipsolver.numRow(); i != nlprows; ++i) {
      if (deletemask[i] >= 0) {
        lprows[deletemask[i]] = lprows[i];
        basis.row_status[deletemask[i]] = basis.row_status[i];
      }
    }

    assert(lpsolver.getLp().num_row_ ==
           (HighsInt)lpsolver.getLp().row_lower_.size());

    basis.row_status.resize(basis.row_status.size() - ndelcuts);
    lprows.resize(lprows.size() - ndelcuts);

    assert(lpsolver.getLp().num_row_ == (HighsInt)lprows.size());
    basis.debug_origin_name = "HighsLpRelaxation::removeCuts";
    lpsolver.setBasis(basis);
    lpsolver.optimizeLp();
    if (!mipsolver.submip) {
      const HighsSubSolverCallTime& sub_solver_call_time =
          lpsolver.getSubSolverCallTime();
      mipsolver.analysis_.addSubSolverCallTime(sub_solver_call_time);
      // Go through sub_solver_call_time to update any MIP clocks
      const bool valid_basis = true;
      const bool use_presolve = false;
      mipsolver.analysis_.mipTimerUpdate(sub_solver_call_time, valid_basis,
                                         use_presolve);
    }
  }
}

void HighsLpRelaxation::removeCuts() {
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
  HighsInt nlprows = lpsolver.getNumRow();
  HighsInt modelrows = mipsolver.numRow();

  lpsolver.deleteRows(modelrows, nlprows - 1);
  for (HighsInt i = modelrows; i != nlprows; ++i) {
    if (lprows[i].origin == LpRow::Origin::kCutPool)
      mipsolver.mipdata_->cutpool.lpCutRemoved(lprows[i].index);
  }
  lprows.resize(modelrows);
  localCuts.clear();
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
}

void HighsLpRelaxation::removeLocalCuts() {
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
  const HighsInt nlprows = lpsolver.getNumRow();
  const HighsInt modelrows = mipsolver.numRow();
  HighsInt ndelcuts = 0;
  std::vector<HighsInt> deletemask;
  for (HighsInt i = modelrows; i != nlprows; ++i) {
    if (lprows[i].origin == LpRow::Origin::kLocalCut) {
      if (ndelcuts == 0) deletemask.resize(nlprows);
      ++ndelcuts;
      deletemask[i] = 1;
    }
  }
  removeCuts(ndelcuts, deletemask);
  localCuts.clear();
}

void HighsLpRelaxation::performAging(bool deleteRows) {
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());
  HighsInt agelimit;

  if (lpsolver.getInfo().basis_validity == kBasisValidityInvalid ||
      lpsolver.getInfo().max_dual_infeasibility > mipsolver.mipdata_->feastol ||
      !lpsolver.getSolution().dual_valid)
    return;

  if (deleteRows) {
    agelimit = mipsolver.options_mip_->mip_lp_age_limit;

    ++epochs;
    if (epochs % std::max(agelimit >> 1, HighsInt{2}) != 0)
      agelimit = kHighsIInf;
    else if ((HighsInt)epochs < agelimit)
      agelimit = epochs;
  } else {
    if (lastAgeCall == numlpiters) return;
    agelimit = kHighsIInf;
  }

  lastAgeCall = numlpiters;

  HighsInt nlprows = numRows();
  HighsInt nummodelrows = getNumModelRows();
  std::vector<HighsInt> deletemask;

  HighsInt ndelcuts = 0;
  for (HighsInt i = nummodelrows; i != nlprows; ++i) {
    assert(lprows[i].origin == LpRow::Origin::kCutPool ||
           lprows[i].origin == LpRow::Origin::kLocalCut);
    if (lprows[i].origin == LpRow::Origin::kLocalCut) continue;
    if (lpsolver.getBasis().row_status[i] == HighsBasisStatus::kBasic) {
      lprows[i].age += (deleteRows || lprows[i].age != 0);
      if (lprows[i].age > agelimit) {
        if (ndelcuts == 0) deletemask.resize(nlprows);
        ++ndelcuts;
        deletemask[i] = 1;
        if (lprows[i].origin == LpRow::Origin::kCutPool)
          mipsolver.mipdata_->cutpool.lpCutRemoved(lprows[i].index);
      }
    } else if (std::abs(lpsolver.getSolution().row_dual[i]) >
               lpsolver.getOptions().dual_feasibility_tolerance) {
      lprows[i].age = 0;
    }
  }

  removeCuts(ndelcuts, deletemask);
}

void HighsLpRelaxation::resetAges() {
  assert(lpsolver.getLp().num_row_ ==
         (HighsInt)lpsolver.getLp().row_lower_.size());

  if (lpsolver.getInfo().basis_validity == kBasisValidityInvalid ||
      lpsolver.getInfo().max_dual_infeasibility > mipsolver.mipdata_->feastol ||
      !lpsolver.getSolution().dual_valid)
    return;

  HighsInt nlprows = numRows();
  HighsInt nummodelrows = getNumModelRows();

  for (HighsInt i = nummodelrows; i != nlprows; ++i) {
    assert(lprows[i].origin == LpRow::Origin::kCutPool ||
           lprows[i].origin == LpRow::Origin::kLocalCut);
    if (lpsolver.getBasis().row_status[i] != HighsBasisStatus::kBasic &&
        std::abs(lpsolver.getSolution().row_dual[i]) >
            lpsolver.getOptions().dual_feasibility_tolerance)
      lprows[i].age = 0;
  }
}

void HighsLpRelaxation::flushDomain(HighsDomain& domain, bool continuous) {
  if (!domain.getChangedCols().empty()) {
    if (&domain == &mipsolver.mipdata_->domain) continuous = true;
    currentbasisstored = false;
    if (!continuous) domain.removeContinuousChangedCols();
    HighsInt numChgCols = domain.getChangedCols().size();
    if (numChgCols == 0) return;
    const HighsInt* chgCols = domain.getChangedCols().data();
    for (HighsInt i = 0; i < numChgCols; ++i) {
      HighsInt col = chgCols[i];
      colLbBuffer[i] = domain.col_lower_[col];
      colUbBuffer[i] = domain.col_upper_[col];
    }

    lpsolver.changeColsBounds(numChgCols, domain.getChangedCols().data(),
                              colLbBuffer.data(), colUbBuffer.data());

    domain.clearChangedCols();
  }
}

bool HighsLpRelaxation::computeDualProof(const HighsDomain& globaldomain,
                                         double upperbound,
                                         std::vector<HighsInt>& inds,
                                         std::vector<double>& vals, double& rhs,
                                         bool extractCliques) const {
#if 0
  const HighsBasis& basis = lpsolver.getBasis();
  const HighsSolution& sol = lpsolver.getSolution();
  HighsCDouble proofRhs = upperbound;
  assert(lpsolver.getInfo().max_dual_infeasibility <=
         mipsolver.mipdata_->feastol);

  proofRhs -= lpsolver.getInfo().objective_function_value;
  inds.clear();
  vals.clear();
  double maxVal = 0.0;
  double maxValGlb = 0.0;
  double sumLocal = 0.0;
  HighsInt numLocalCols = 0;
  const HighsInt numCol = lpsolver.getNumCol();
  for (HighsInt i : mipsolver.mipdata_->integral_cols) {
    if (basis.col_status[i] == HighsBasisStatus::kBasic) continue;

    if (sol.col_dual[i] > kHighsTiny) {
      if (sol.col_value[i] != globaldomain.col_lower_[i]) {
        maxVal = std::max(sol.col_dual[i], maxVal);
        sumLocal += sol.col_dual[i];
        ++numLocalCols;
      }
    } else if (sol.col_dual[i] < -kHighsTiny) {
      if (sol.col_value[i] != globaldomain.col_upper_[i]) {
        maxVal = std::max(-sol.col_dual[i], maxVal);
        sumLocal += -sol.col_dual[i];
        ++numLocalCols;
      }
    } else
      continue;

    proofRhs += sol.col_value[i] * sol.col_dual[i];

    vals.push_back(sol.col_dual[i]);
    inds.push_back(i);
  }

  int expShift;
  std::frexp(maxVal - mipsolver.mipdata_->epsilon, &expShift);
  expShift = -expShift;

  HighsInt len = vals.size();

  double minGlbVal = numLocalCols == 0
                         ? 0.0
                         : std::ldexp(sumLocal / numLocalCols, expShift) -
                               mipsolver.mipdata_->feastol;

  for (HighsInt i = len - 1; i >= 0; --i) {
    HighsInt iCol = inds[i];
    double val = std::ldexp(vals[i], expShift);

    bool remove = false;

    double absVal = std::fabs(val);
    if (absVal <= mipsolver.mipdata_->feastol || globaldomain.isFixed(iCol)) {
      if (vals[i] > 0) {
        if (globaldomain.col_lower_[iCol] == -kHighsInf) return false;
        proofRhs -= vals[i] * globaldomain.col_lower_[iCol];
      } else {
        if (globaldomain.col_upper_[iCol] == kHighsInf) return false;
        proofRhs -= vals[i] * globaldomain.col_upper_[iCol];
      }

      remove = true;
    } else if (absVal < minGlbVal) {
      if (vals[i] > 0)
        remove = sol.col_value[iCol] == globaldomain.col_lower_[iCol];
      else
        remove = sol.col_value[iCol] == globaldomain.col_upper_[iCol];

      if (remove) proofRhs -= vals[i] * sol.col_value[iCol];
    }

    if (remove) {
      --len;
      vals[i] = vals[len];
      inds[i] = inds[len];
    } else {
      vals[i] = val;
    }
  }

  vals.resize(len);
  inds.resize(len);

  rhs = std::ldexp(double(proofRhs), expShift);

  globaldomain.tightenCoefficients(inds.data(), vals.data(), inds.size(), rhs);

  mipsolver.mipdata_->debugSolution.checkCut(inds.data(), vals.data(),
                                             inds.size(), rhs);
  if (extractCliques)
    mipsolver.mipdata_->cliquetable.extractCliquesFromCut(
        mipsolver, inds.data(), vals.data(), inds.size(), rhs);

  return true;
#else
  std::vector<double> row_dual = lpsolver.getSolution().row_dual;

  const HighsLp& lp = lpsolver.getLp();

  assert(std::isfinite(upperbound));
  HighsCDouble upper = upperbound;

  for (HighsInt i = 0; i != lp.num_row_; ++i) {
    // @FlipRowDual row_dual[i] < 0 became row_dual[i] > 0
    if (row_dual[i] > 0) {
      if (lp.row_lower_[i] != -kHighsInf)
        // @FlipRowDual += became -=
        upper -= row_dual[i] * lp.row_lower_[i];
      else
        row_dual[i] = 0;
      // @FlipRowDual row_dual[i] > 0 became row_dual[i] < 0
    } else if (row_dual[i] < 0) {
      if (lp.row_upper_[i] != kHighsInf)
        // @FlipRowDual += became -=
        upper -= row_dual[i] * lp.row_upper_[i];
      else
        row_dual[i] = 0;
    }
  }

  inds.clear();
  vals.clear();
  inds.reserve(lp.num_col_);
  vals.reserve(lp.num_col_);
  for (HighsInt i = 0; i != lp.num_col_; ++i) {
    HighsInt start = lp.a_matrix_.start_[i];
    HighsInt end = lp.a_matrix_.start_[i + 1];

    HighsCDouble sum = lp.col_cost_[i];

    for (HighsInt j = start; j != end; ++j) {
      if (row_dual[lp.a_matrix_.index_[j]] == 0) continue;
      // @FlipRowDual += became -=
      sum -= lp.a_matrix_.value_[j] * row_dual[lp.a_matrix_.index_[j]];
    }

    double val = double(sum);

    if (std::fabs(val) <= mipsolver.options_mip_->small_matrix_value) continue;

    bool removeValue = std::fabs(val) <= mipsolver.mipdata_->feastol;

    if (!removeValue &&
        (globaldomain.col_lower_[i] == globaldomain.col_upper_[i] ||
         mipsolver.isColContinuous(i))) {
      if (val > 0)
        removeValue =
            lpsolver.getSolution().col_value[i] - globaldomain.col_lower_[i] <=
            mipsolver.mipdata_->feastol;
      else
        removeValue =
            globaldomain.col_upper_[i] - lpsolver.getSolution().col_value[i] <=
            mipsolver.mipdata_->feastol;
    }

    if (removeValue) {
      if (val < 0) {
        if (globaldomain.col_upper_[i] == kHighsInf) return false;
        upper -= val * globaldomain.col_upper_[i];
      } else {
        if (globaldomain.col_lower_[i] == -kHighsInf) return false;

        upper -= val * globaldomain.col_lower_[i];
      }

      continue;
    }

    vals.push_back(val);
    inds.push_back(i);
  }

  rhs = double(upper);
  assert(std::isfinite(rhs));
  globaldomain.tightenCoefficients(inds.data(), vals.data(), inds.size(), rhs);

  mipsolver.mipdata_->debugSolution.checkCut(inds.data(), vals.data(),
                                             inds.size(), rhs);
  if (extractCliques)
    mipsolver.mipdata_->cliquetable.extractCliquesFromCut(
        mipsolver, inds.data(), vals.data(), inds.size(), rhs);

  return true;
#endif
}

void HighsLpRelaxation::storeDualInfProof() {
  assert(lpsolver.getModelStatus() == HighsModelStatus::kInfeasible);
  hasdualproof = false;
  if (lpsolver.getInfo().basis_validity == kBasisValidityInvalid) return;
  HighsInt num_row = lpsolver.getNumRow();
  HighsInt num_col = lpsolver.getNumCol();

  if (row_ep.size < num_row) {
    row_ep.setup(num_row);

    if ((HighsInt)row_ap.values.size() < num_col) {
      row_ap.setDimension(num_col);
      dualproofvals.reserve(num_col);
      dualproofinds.reserve(num_col);
    }
  }

  lpsolver.getDualRaySparse(hasdualproof, row_ep);

  if (!hasdualproof) {
    highsLogDev(mipsolver.options_mip_->log_options, HighsLogType::kVerbose,
                "no dual ray stored\n");
    return;
  }

  dualproofinds.clear();
  dualproofvals.clear();
  dualproofrhs = kHighsInf;
  const HighsLp& lp = lpsolver.getLp();

  assert(hasdualproof);

  HighsCDouble upper = 0.0;
  row_ap.clear();

  for (HighsInt i = 0; i < row_ep.count; ++i) {
    HighsInt iRow = row_ep.index[i];
    const double weight = -row_ep.array[iRow];
    if (std::fabs(weight) * getMaxAbsRowVal(iRow) <=
        mipsolver.mipdata_->epsilon)
      continue;
    else if (weight > 0) {
      if (lp.row_upper_[iRow] == kHighsInf) continue;
      upper += weight * lp.row_upper_[iRow];
    } else {
      if (lp.row_lower_[iRow] == -kHighsInf) continue;
      upper += weight * lp.row_lower_[iRow];
    }

    HighsInt len;
    const HighsInt* inds;
    const double* vals;
    getRow(iRow, len, inds, vals);

    for (HighsInt j = 0; j < len; ++j) row_ap.add(inds[j], weight * vals[j]);
  }

  const HighsDomain& globaldomain = mipsolver.mipdata_->domain;

  for (HighsInt i : row_ap.getNonzeros()) {
    double val = row_ap.getValue(i);

    if (std::fabs(val) <= mipsolver.mipdata_->epsilon) continue;

    bool removeValue = std::abs(val) <= mipsolver.mipdata_->feastol;

    if (!removeValue &&
        (globaldomain.col_lower_[i] == globaldomain.col_upper_[i] ||
         mipsolver.isColContinuous(i))) {
      // remove continuous entries and globally fixed entries whenever the
      // local LP's bound is not tighter than the global bound
      if (val > 0)
        removeValue = lp.col_lower_[i] - globaldomain.col_lower_[i] <=
                      mipsolver.mipdata_->feastol;
      else
        removeValue = globaldomain.col_upper_[i] - lp.col_upper_[i] <=
                      mipsolver.mipdata_->feastol;
    }

    if (removeValue) {
      if (val < 0) {
        if (globaldomain.col_upper_[i] == kHighsInf) {
          hasdualproof = false;
          return;
        }
        upper -= val * globaldomain.col_upper_[i];
      } else {
        if (globaldomain.col_lower_[i] == -kHighsInf) {
          hasdualproof = false;
          return;
        }
        upper -= val * globaldomain.col_lower_[i];
      }

      continue;
    }

    dualproofvals.push_back(val);
    dualproofinds.push_back(i);
  }

  dualproofrhs = double(upper);
  mipsolver.mipdata_->domain.tightenCoefficients(
      dualproofinds.data(), dualproofvals.data(), dualproofinds.size(),
      dualproofrhs);

  mipsolver.mipdata_->debugSolution.checkCut(
      dualproofinds.data(), dualproofvals.data(), dualproofinds.size(),
      dualproofrhs);

  mipsolver.mipdata_->cliquetable.extractCliquesFromCut(
      mipsolver, dualproofinds.data(), dualproofvals.data(),
      dualproofinds.size(), dualproofrhs);
}

void HighsLpRelaxation::storeDualUBProof() {
  assert(lpsolver.getModelStatus() == HighsModelStatus::kObjectiveBound);

  dualproofinds.clear();
  dualproofvals.clear();

  if (lpsolver.getSolution().dual_valid)
    hasdualproof = computeDualProof(mipsolver.mipdata_->domain,
                                    mipsolver.mipdata_->upper_limit,
                                    dualproofinds, dualproofvals, dualproofrhs);
  else
    hasdualproof = false;

  if (!hasdualproof) dualproofrhs = kHighsInf;
}

bool HighsLpRelaxation::checkDualProof() const {
  if (!hasdualproof) return true;
  if (dualproofrhs == kHighsInf) return false;

  HighsInt len = dualproofinds.size();

  HighsCDouble viol = -dualproofrhs;

  const HighsLp& lp = lpsolver.getLp();

  for (HighsInt i = 0; i != len; ++i) {
    HighsInt col = dualproofinds[i];
    if (dualproofvals[i] > 0) {
      if (lp.col_lower_[col] == -kHighsInf) return false;
      viol += dualproofvals[i] * lp.col_lower_[col];
    } else {
      assert(dualproofvals[i] < 0);
      if (lp.col_upper_[col] == kHighsInf) return false;
      viol += dualproofvals[i] * lp.col_upper_[col];
    }
  }

  return viol > mipsolver.mipdata_->feastol;
}

bool HighsLpRelaxation::computeDualInfProof(const HighsDomain& globaldomain,
                                            std::vector<HighsInt>& inds,
                                            std::vector<double>& vals,
                                            double& rhs) const {
  if (!hasdualproof) return false;

  assert(std::isfinite(dualproofrhs));

  inds = dualproofinds;
  vals = dualproofvals;
  rhs = dualproofrhs;
  return true;
}

void HighsLpRelaxation::recoverBasis() {
  if (basischeckpoint) {
    lpsolver.setBasis(*basischeckpoint, "HighsLpRelaxation::recoverBasis");
    currentbasisstored = true;
  }
}

void HighsLpRelaxation::setObjectiveLimit(double objlim) {
  double offset;
  if (mipsolver.mipdata_->objectiveFunction.isIntegral())
    offset = 0.5 / mipsolver.mipdata_->objectiveFunction.integralScale();
  else
    offset = std::max(1000.0 * mipsolver.mipdata_->feastol,
                      std::abs(objlim) * kHighsTiny);

  lpsolver.setOptionValue("objective_bound", objlim + offset);
}

HighsLpRelaxation::Status HighsLpRelaxation::run(bool resolve_on_error) {
  const HighsInfo& info = lpsolver.getInfo();
  const double this_time_limit =
      std::max(lpsolver.getRunTime() + mipsolver.options_mip_->time_limit -
                   mipsolver.timer_.read(),
               0.0);
  lpsolver.setOptionValue("time_limit", this_time_limit);
  // lpsolver.setOptionValue("output_flag", true);
  const bool valid_basis = lpsolver.getBasis().valid;

  if (mipsolver.analysis_.analyse_mip_time && !mipsolver.submip &&
      !this->solved_first_lp) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "MIP-Timing: %11.2g - start first LP solve (with%s basis)\n",
                 mipsolver.timer_.read(), valid_basis ? "" : "out");
  }
  // Determine the solver
  //
  // Currently use simplex by default, unless IPM is requested
  // explicitly and there is no basis. Later pass mip_lp_solver and
  // take action on failure in SolveLp
  std::string solver;
  lpsolver.getOptionValue("solver", solver);
  std::string use_solver;
  if (valid_basis) {
    use_solver = kSimplexString;
  } else {
    const std::string mip_lp_solver = mipsolver.options_mip_->mip_lp_solver;
    if (useIpm(mip_lp_solver)) {
      bool use_hipo = mip_lp_solver == kHipoString;
      // Later still, pass mip_lp_solver and take action on failure in
      // solveLp
#ifndef HIPO
      // Shouldn't be possible to choose HiPO if it's not in the build
      assert(!use_hipo);
      use_hipo = false;
#endif
      use_solver = use_hipo ? kHipoString : kIpxString;
    } else {
      use_solver = kSimplexString;
    }
  }
  HighsStatus callstatus;
  // Now allowing the use of IPM at the root node
  lpsolver.setOptionValue("solver", use_solver);
  bool use_ipm = useIpm(use_solver);
  bool use_simplex = !use_ipm;
  if (use_ipm) {
    assert(!valid_basis);
    const bool ipm_logging = false;
    if (ipm_logging) {
      std::string presolve;
      lpsolver.getOptionValue("presolve", presolve);
      printf(
          "HighsLpRelaxation::run Solving the root node with IPM, using "
          "presolve = %s\n",
          presolve.c_str());
      bool output_flag;
      lpsolver.getOptionValue("output_flag", output_flag);
      assert(output_flag == false);
      (void)output_flag;
      lpsolver.setOptionValue("output_flag", !mipsolver.submip);
    }
    const bool dump_ipm_lp = false;
    if (dump_ipm_lp && !mipsolver.submip) {
      const std::string file_name = mipsolver.model_->model_name_ + "_root.mps";
      printf("HighsMipSolverData::run Calling lpsolver.writeModel(%s)\n",
             file_name.c_str());
      lpsolver.writeModel(file_name);
      fflush(stdout);
      exit(1);
    }
    callstatus = lpsolver.optimizeLp();
    if (ipm_logging) lpsolver.setOptionValue("output_flag", false);
    if (callstatus == HighsStatus::kError) {
      highsLogDev(
          mipsolver.options_mip_->log_options, HighsLogType::kInfo,
          "HighsLpRelaxation::run HiPO has failed : status = %s Try IPX\n",
          lpsolver.modelStatusToString(lpsolver.getModelStatus()).c_str());
      lpsolver.setOptionValue("solver", kSimplexString);
      use_simplex = true;
    }
  }
  if (use_simplex) {
    callstatus = lpsolver.optimizeLp();
  }
  // Revert the value of lpsolver.options_.solver
  lpsolver.setOptionValue("solver", solver);
  if (!mipsolver.submip) {
    const HighsSubSolverCallTime& sub_solver_call_time =
        lpsolver.getSubSolverCallTime();
    mipsolver.analysis_.addSubSolverCallTime(sub_solver_call_time);
    // Go through sub_solver_call_time to update any MIP clocks
    std::string presolve;
    lpsolver.getOptionValue("presolve", presolve);
    const bool use_presolve = presolve != kHighsOffString;
    mipsolver.analysis_.mipTimerUpdate(sub_solver_call_time, valid_basis,
                                       use_presolve);
  }
  if (mipsolver.analysis_.analyse_mip_time && !mipsolver.submip &&
      !this->solved_first_lp) {
    highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                 "MIP-Timing: %11.2g - finish first LP solve\n",
                 mipsolver.timer_.read());
  }
  this->solved_first_lp = true;
  HighsInt itercount = -1;
  if (use_simplex) {
    itercount = std::max(HighsInt{0}, info.simplex_iteration_count);
    numlpiters += itercount;
  }

  if (callstatus == HighsStatus::kError) {
    lpsolver.clearSolver();
#if 0
    // first try to use the primal simplex solver starting from the last basis
    if (lpsolver.getOptions().simplex_strategy == kSimplexStrategyDual) {
      lpsolver.setOptionValue("simplex_strategy", kSimplexStrategyPrimal);
      recoverBasis();
      auto retval = run(resolve_on_error);
      lpsolver.setOptionValue("simplex_strategy", kSimplexStrategyDual);

      return retval;
    }
#endif

    if (resolve_on_error) {
      // still an error: now try to solve with presolve from scratch
      lpsolver.setOptionValue("simplex_strategy", kSimplexStrategyDual);
      lpsolver.setOptionValue("presolve", kHighsOnString);
      auto retval = run(false);
      lpsolver.setOptionValue("presolve", kHighsOffString);

      return retval;
    }

    recoverBasis();

    return Status::kError;
  }

  HighsModelStatus model_status = lpsolver.getModelStatus();
  switch (model_status) {
    case HighsModelStatus::kObjectiveBound:
      ++numSolved;
      if (itercount >= 0)
        avgSolveIters +=
            (itercount - avgSolveIters) / static_cast<double>(numSolved);

      storeDualUBProof();
      return Status::kInfeasible;
    case HighsModelStatus::kInfeasible: {
      ++numSolved;
      if (itercount >= 0)
        avgSolveIters +=
            (itercount - avgSolveIters) / static_cast<double>(numSolved);

      storeDualInfProof();
      if (true || checkDualProof()) return Status::kInfeasible;
      // /printf("infeasibility proof not valid\n");
      hasdualproof = false;

      // HighsInt scalestrategy = lpsolver.getOptions().simplex_scale_strategy;
      // if (scalestrategy != kSimplexScaleStrategyOff) {
      //   lpsolver.setOptionValue("simplex_scale_strategy",
      //                           kSimplexScaleStrategyOff);
      //   HighsBasis basis = lpsolver.getBasis();
      //   lpsolver.clearSolver();
      //   lpsolver.setBasis(basis);
      //   auto tmp = run(resolve_on_error);
      //   lpsolver.setOptionValue("simplex_scale_strategy", scalestrategy);
      //   if (!scaledOptimal(tmp)) {
      //     lpsolver.clearSolver();
      //     lpsolver.setBasis(basis);
      //   }
      //   return tmp;
      // }
      //
      // // trust the primal simplex result without scaling
      // if (lpsolver.getModelStatus() == HighsModelStatus::kInfeasible)
      //   return Status::kInfeasible;

      // highsLogUser(mipsolver.options_mip_->log_options,
      //                 HighsLogType::kWarning,
      //                 "LP failed to reliably determine infeasibility\n");

      // printf("error: unreliable infeasibilities, modelstatus = %"
      // HIGHSINT_FORMAT " (scaled
      // %" HIGHSINT_FORMAT ")\n",
      //        (HighsInt)lpsolver.getModelStatus(),
      //        (HighsInt)lpsolver.getModelStatus(true));
      return Status::kError;
    }
    case HighsModelStatus::kUnbounded:
      // If unboundedness is detected in the presolved LP, then
      // postsolve cannot be run, so there is no basis. Returning
      // Status::kError as a result yielded #1962, where the root node
      // is unbounded.
      if (info.basis_validity == kBasisValidityInvalid)
        highsLogUser(mipsolver.options_mip_->log_options,
                     HighsLogType::kWarning,
                     "HighsLpRelaxation::run LP is unbounded with no basis, "
                     "but not returning Status::kError\n");
      if (info.primal_solution_status == kSolutionStatusFeasible)
        mipsolver.mipdata_->trySolution(lpsolver.getSolution().col_value,
                                        kSolutionSourceUnbounded);

      return Status::kUnbounded;
    case HighsModelStatus::kUnknown:
      if (info.basis_validity == kBasisValidityInvalid) return Status::kError;
      // fall through
    case HighsModelStatus::kOptimal:
      assert(info.max_primal_infeasibility >= 0);
      assert(info.max_dual_infeasibility >= 0);
      ++numSolved;
      if (itercount >= 0)
        avgSolveIters +=
            (itercount - avgSolveIters) / static_cast<double>(numSolved);
      if (info.max_primal_infeasibility <= mipsolver.mipdata_->feastol &&
          info.max_dual_infeasibility <= mipsolver.mipdata_->feastol)
        return Status::kOptimal;

      if (info.max_primal_infeasibility <= mipsolver.mipdata_->feastol)
        return Status::kUnscaledPrimalFeasible;

      if (info.max_dual_infeasibility <= mipsolver.mipdata_->feastol)
        return Status::kUnscaledDualFeasible;

      if (model_status == HighsModelStatus::kOptimal)
        return Status::kUnscaledInfeasible;

      return Status::kError;
    case HighsModelStatus::kIterationLimit: {
      if (!mipsolver.submip && resolve_on_error) {
        // printf(
        //     "error: lpsolver reached iteration limit, resolving with basis "
        //     "from ipm\n");
        Highs ipm;
        ipm.setOptionValue("output_flag", false);
        // check if only root presolve is allowed
        const bool use_presolve =
            !mipsolver.options_mip_->mip_root_presolve_only;
        const std::string presolve =
            use_presolve ? kHighsChooseString : kHighsOffString;
        ipm.setOptionValue("presolve", presolve);
        // Determine the solver
        const std::string mip_ipm_solver =
            mipsolver.options_mip_->mip_ipm_solver;
        // Currently use HiPO by default and take action on failure
        // here. Later pass mip_ipm_solver and take action on failure in
        // solveLp
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
        ipm.passModel(lpsolver.getLp());
        // todo @ Julian : If you remove this you can see the looping on
        // istanbul-no-cutoff
        ipm.setOptionValue("simplex_iteration_limit",
                           info.simplex_iteration_count);
        const bool ipm_logging = false;
        if (ipm_logging) {
          std::string presolve;
          ipm.getOptionValue("presolve", presolve);
          printf(
              "HighsLpRelaxation::run After lpsolver reached iteration limit, "
              "solving with IPM, using presolve = %s\n",
              presolve.c_str());
          bool output_flag;
          ipm.getOptionValue("output_flag", output_flag);
          assert(output_flag == false);
          (void)output_flag;
          ipm.setOptionValue("output_flag", !mipsolver.submip);
        }
        ipm.optimizeLp();
        if (ipm_logging) ipm.setOptionValue("output_flag", false);
        if (use_hipo && !ipm.getBasis().valid) {
          // HiPO has failed to get a solution, so try IPX
          highsLogDev(mipsolver.options_mip_->log_options, HighsLogType::kInfo,
                      "HighsLpRelaxation::run HiPO has failed to get a valid "
                      "basis: status = %s Try IPX\n",
                      ipm.modelStatusToString(ipm.getModelStatus()).c_str());
          ipm.setOptionValue("solver", kIpxString);
          ipm.optimizeLp();
        }
        const HighsSubSolverCallTime& sub_solver_call_time =
            ipm.getSubSolverCallTime();
        mipsolver.analysis_.addSubSolverCallTime(sub_solver_call_time);
        // Go through sub_solver_call_time to update any MIP clocks
        const bool valid_basis = false;
        mipsolver.analysis_.mipTimerUpdate(sub_solver_call_time, valid_basis,
                                           use_presolve);

        lpsolver.setBasis(ipm.getBasis(), "HighsLpRelaxation::run IPM basis");
        return run(false);
      }

      // printf("error: lpsolver reached iteration limit\n");
      return Status::kError;
    }
    case HighsModelStatus::kTimeLimit:
      return Status::kError;
    default:
      // printf("error: lpsolver stopped with unexpected status %"
      // HIGHSINT_FORMAT "\n",
      //        (HighsInt)model_status);
      highsLogUser(mipsolver.options_mip_->log_options, HighsLogType::kWarning,
                   "LP solved to unexpected status: %s\n",
                   lpsolver.modelStatusToString(model_status).c_str());
      return Status::kError;
  }
}

HighsLpRelaxation::Status HighsLpRelaxation::resolveLp(HighsDomain* domain) {
  fractionalints.clear();

  bool solveagain;
  const bool xpool_ledger = hacdcpfXpoolLedgerEnabledForLp(*this);
  const auto resolve_t0 = std::chrono::steady_clock::now();
  const int64_t iter_before = numlpiters;
  do {
    solveagain = false;
    if (domain) flushDomain(*domain);
    status = run();

    switch (status) {
      case Status::kUnscaledInfeasible:
      case Status::kUnscaledDualFeasible:
      case Status::kUnscaledPrimalFeasible:
      case Status::kOptimal: {
        HighsHashTable<HighsInt, std::pair<double, int>> fracints(
            maxNumFractional);
        const HighsSolution& sol = lpsolver.getSolution();

        HighsCDouble objsum = 0;
        bool roundable = true;

        for (HighsInt i : mipsolver.mipdata_->integral_cols) {
          // for the fractionality we assume that LP bounds are not violated
          // bounds that are violated by the unscaled LP are indicated by the
          // return status already
          double val = std::max(
              std::min(sol.col_value[i], lpsolver.getLp().col_upper_[i]),
              lpsolver.getLp().col_lower_[i]);

          if (fractionality(val) > mipsolver.mipdata_->feastol) {
            HighsInt col = i;
            roundable = roundable && (mipsolver.mipdata_->uplocks[col] == 0 ||
                                      mipsolver.mipdata_->downlocks[col] == 0);

            const HighsCliqueTable::Substitution* subst =
                mipsolver.mipdata_->cliquetable.getSubstitution(col);
            while (subst != nullptr) {
              if (lpsolver.getLp().col_lower_[subst->replace.col] ==
                  lpsolver.getLp().col_upper_[subst->replace.col]) {
                if (domain) {
                  domain->fixCol(
                      col, subst->replace.weight(lpsolver.getLp().col_lower_));
                  if (domain->infeasible()) {
                    objective = kHighsInf;
                    status = Status::kInfeasible;
                    return status;
                  }
                } else
                  break;
              }

              // Spot cases where a global domain change during the search has
              // led to a clique and the local domain has branched into a state
              // where the clique subst. would be invalid for the local domain.
              // TODO: Turn into assert when search becomes parallel
              if (domain) {
                double replace_val = subst->replace.val == 0 ? 1.0 - val : val;
                double replace_lb = domain->col_lower_[subst->replace.col];
                double replace_ub = domain->col_upper_[subst->replace.col];
                double feastol = mipsolver.mipdata_->feastol;
                if ((replace_val < replace_lb - feastol) ||
                    (replace_val > replace_ub + feastol)) {
                  break;
                }
              }

              col = subst->replace.col;
              if (subst->replace.val == 0) val = 1.0 - val;

              subst = mipsolver.mipdata_->cliquetable.getSubstitution(col);
            }

            if (adjustSymBranchingCol)
              col = mipsolver.mipdata_->symmetries.getBranchingColumn(
                  getLp().col_lower_, getLp().col_upper_, col);

            auto& pair = fracints[col];
            pair.first += val;
            pair.second += 1;
          } else {
            if (lpsolver.getBasis().col_status[i] == HighsBasisStatus::kBasic)
              continue;

            const double glb = mipsolver.mipdata_->domain.col_lower_[i];
            const double gub = mipsolver.mipdata_->domain.col_upper_[i];

            if (std::min(gub - sol.col_value[i], sol.col_value[i] - glb) <=
                mipsolver.mipdata_->feastol)
              continue;

            const auto& matrix = lpsolver.getLp().a_matrix_;
            const HighsInt colStart =
                matrix.start_[i] + (mipsolver.model_->a_matrix_.start_[i + 1] -
                                    mipsolver.model_->a_matrix_.start_[i]);
            const HighsInt colEnd = matrix.start_[i + 1];

            // skip further checks if the column has no entry in any cut
            if (colStart == colEnd) continue;

            for (HighsInt j = colStart; j < colEnd; ++j) {
              HighsInt row = matrix.index_[j];
              assert(row >= mipsolver.numRow());

              // age is already zero, so irrelevant whether we reset it
              if (lprows[row].age == 0) continue;

              // check that row has no slack in the current solution
              if (sol.row_value[row] <
                  getLp().row_upper_[row] - mipsolver.mipdata_->feastol)
                continue;

              lprows[row].age = 0;
            }
          }
        }

        maxNumFractional =
            std::max((HighsInt)fracints.size(), maxNumFractional);

        if (domain && !domain->getChangedCols().empty()) {
          // printf("resolving due to fixings of substituted columns\n");
          solveagain = true;
          continue;
        }

        for (const auto& it : fracints) {
          fractionalints.emplace_back(
              it.key(), it.value().first / (double)it.value().second);
        }

        if (roundable && !fractionalints.empty()) {
          std::vector<double> roundsol = sol.col_value;

          for (const std::pair<HighsInt, double>& fracint : fractionalints) {
            // get column index
            HighsInt col = fracint.first;
            // round based on locks and sign of objective coefficient
            if (mipsolver.mipdata_->uplocks[col] == 0 &&
                (mipsolver.colCost(col) < 0 ||
                 mipsolver.mipdata_->downlocks[col] != 0)) {
              // round up
              roundsol[col] = std::min(
                  std::ceil(fracint.second - mipsolver.mipdata_->feastol),
                  lpsolver.getLp().col_upper_[col] == kHighsInf
                      ? kHighsInf
                      : std::floor(lpsolver.getLp().col_upper_[col] +
                                   mipsolver.mipdata_->feastol));
            } else {
              // round down
              roundsol[col] = std::max(
                  std::floor(fracint.second + mipsolver.mipdata_->feastol),
                  lpsolver.getLp().col_lower_[col] == -kHighsInf
                      ? -kHighsInf
                      : std::ceil(lpsolver.getLp().col_lower_[col] -
                                  mipsolver.mipdata_->feastol));
            }
          }

          const auto& cliquesubst =
              mipsolver.mipdata_->cliquetable.getSubstitutions();
          for (HighsInt k = cliquesubst.size() - 1; k >= 0; --k) {
            if (cliquesubst[k].replace.val == 0)
              roundsol[cliquesubst[k].substcol] =
                  1 - roundsol[cliquesubst[k].replace.col];
            else
              roundsol[cliquesubst[k].substcol] =
                  roundsol[cliquesubst[k].replace.col];
          }

          for (HighsInt i = 0; i != mipsolver.numCol(); ++i)
            objsum += roundsol[i] * mipsolver.colCost(i);

          mipsolver.mipdata_->addIncumbent(roundsol, double(objsum),
                                           kSolutionSourceSolveLp);
          objsum = 0;
        }

        for (HighsInt i = 0; i != mipsolver.numCol(); ++i)
          objsum += sol.col_value[i] * mipsolver.colCost(i);

        if (fractionalints.empty() && !unscaledPrimalFeasible(status)) {
          std::vector<double> fixSol = sol.col_value;
          for (HighsInt i = 0; i < mipsolver.numCol(); ++i) {
            if (fixSol[i] < lpsolver.getLp().col_lower_[i])
              fixSol[i] = lpsolver.getLp().col_lower_[i];
            else if (fixSol[i] > lpsolver.getLp().col_upper_[i])
              fixSol[i] = lpsolver.getLp().col_upper_[i];
            else if (mipsolver.isColIntegral(i))
              fixSol[i] = std::round(fixSol[i]);
          }

          if (mipsolver.mipdata_->checkSolution(fixSol)) {
            const_cast<std::vector<double>&>(sol.col_value) = std::move(fixSol);
            if (unscaledDualFeasible(status))
              status = Status::kOptimal;
            else
              status = Status::kUnscaledPrimalFeasible;
          }
        }

        objective = double(objsum);
        break;
      }
      case Status::kInfeasible:
        objective = kHighsInf;
        break;
      default:
        break;
    }
  } while (solveagain);

  if (xpool_ledger) {
    const double resolve_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - resolve_t0)
            .count();
    hacdcpfLogXpoolResolveLedger(mipsolver, *this, status,
                                 numlpiters - iter_before, numlpiters,
                                 resolve_ms);
    if (hacdcpfRootLpCallbackEnabled(mipsolver, *this)) {
      mipsolver.callback_->hacdcpf_root_lp_callback(
          "resolve", this, nullptr,
          mipsolver.callback_->hacdcpf_root_lp_callback_data);
    }
  }
  return status;
}
