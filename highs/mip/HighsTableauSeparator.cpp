/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/**@file mip/HighsTableauSeparator.cpp
 */

#include "mip/HighsTableauSeparator.h"

#include <algorithm>

#include "../extern/pdqsort/pdqsort.h"
#include "mip/HighsCutGeneration.h"
#include "mip/HighsLpAggregator.h"
#include "mip/HighsLpRelaxation.h"
#include "mip/HighsMipSolverData.h"
#include "mip/HighsTransformedLp.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

struct FractionalInteger {
  double fractionality;
  double row_ep_norm2;
  double score;
  HighsInt basisIndex;
  std::vector<std::pair<HighsInt, double>> row_ep;

  bool operator<(const FractionalInteger& other) const {
    return score > other.score;
  }

  FractionalInteger() = default;

  FractionalInteger(HighsInt basisIndex, double fractionality)
      : fractionality(fractionality), score(-1.0), basisIndex(basisIndex) {}
};

namespace {

const char* highsTableauTraceEnv(const char* highsName,
                                 const char* fallbackName) {
  const char* env = std::getenv(highsName);
  if (env != nullptr) return env;
  return std::getenv(fallbackName);
}

bool highsTableauTraceBasisEnabled() {
  const char* env = highsTableauTraceEnv("HIGHS_XROW_TRACE_BASIS",
                                         "HACDCPF_XTAB_ROW_TRACE_BASIS");
  return env != nullptr && env[0] != '\0' && std::string(env) != "0";
}

int highsTableauTraceTerms() {
  const char* env = highsTableauTraceEnv("HIGHS_XROW_TRACE_TERMS",
                                         "HACDCPF_XTAB_ROW_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 12;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 12;
  return static_cast<int>(std::min<long>(value, 200));
}

bool highsLpBasisTraceEnabled() {
  const char* env = highsTableauTraceEnv("HIGHS_LP_BASIS_TRACE",
                                         "HACDCPF_LP_BASIS_TRACE");
  return (env != nullptr && env[0] != '\0' && std::string(env) != "0") ||
         highsTableauTraceBasisEnabled();
}

int highsLpBasisTraceTerms() {
  const char* env = highsTableauTraceEnv("HIGHS_LP_BASIS_TRACE_TERMS",
                                         "HACDCPF_LP_BASIS_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 24;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 24;
  return static_cast<int>(std::min<long>(value, 200));
}

std::uint64_t highsLpBasisHashCombine(std::uint64_t seed,
                                      std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

std::uint64_t highsLpBasisHashDouble(double value) {
  if (std::isinf(value)) return value > 0.0 ? 0x7ff0000000000000ULL
                                            : 0xfff0000000000000ULL;
  if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
  const auto q = static_cast<std::int64_t>(std::llround(value * 1e9));
  return static_cast<std::uint64_t>(q) ^ 0x517cc1b727220a95ULL;
}

const char* highsBasisStatusName(HighsBasisStatus status) {
  switch (status) {
    case HighsBasisStatus::kLower:
      return "lower";
    case HighsBasisStatus::kBasic:
      return "basic";
    case HighsBasisStatus::kUpper:
      return "upper";
    case HighsBasisStatus::kZero:
      return "zero";
    case HighsBasisStatus::kNonbasic:
      return "nonbasic";
  }
  return "unknown";
}

std::uint64_t highsLpBasisSemanticRowHash(
    const HighsLpRelaxation& lpRelaxation, HighsInt row) {
  const double lower = lpRelaxation.slackLower(row);
  const double upper = lpRelaxation.slackUpper(row);
  HighsInt len = 0;
  const HighsInt* rowinds = nullptr;
  const double* rowvals = nullptr;
  lpRelaxation.getRow(row, len, rowinds, rowvals);
  double first = 0.0;
  for (HighsInt k = 0; k < len; ++k) {
    if (std::abs(rowvals[k]) <= 1e-12) continue;
    first = rowvals[k];
    break;
  }
  const bool negate = first < 0.0;
  double canonLower = lower;
  double canonUpper = upper;
  if (negate) {
    canonLower = -upper;
    canonUpper = -lower;
  }
  std::uint64_t h = 0x4842474849533131ULL;
  h = highsLpBasisHashCombine(h, highsLpBasisHashDouble(canonLower));
  h = highsLpBasisHashCombine(h, highsLpBasisHashDouble(canonUpper));
  h = highsLpBasisHashCombine(h, static_cast<std::uint64_t>(len));
  for (HighsInt k = 0; k < len; ++k) {
    if (std::abs(rowvals[k]) <= 1e-12) continue;
    h = highsLpBasisHashCombine(h, static_cast<std::uint64_t>(rowinds[k]));
    h = highsLpBasisHashCombine(
        h, highsLpBasisHashDouble(negate ? -rowvals[k] : rowvals[k]));
  }
  return h;
}

void highsTraceLpBasisState(HighsLpRelaxation& lpRelaxation,
                            const HighsInt* basisinds,
                            const char* stage) {
  if (!highsLpBasisTraceEnabled()) return;
  Highs& lpSolver = lpRelaxation.getLpSolver();
  const auto& basis = lpSolver.getBasis();
  const HighsSolution& solution = lpRelaxation.getSolution();
  const HighsInt numRow = lpRelaxation.numRows();
  const HighsInt numCol = lpRelaxation.numCols();
  const int maxTerms = highsLpBasisTraceTerms();

  int colLower = 0, colBasic = 0, colUpper = 0, colZero = 0, colNonbasic = 0;
  for (HighsBasisStatus status : basis.col_status) {
    if (status == HighsBasisStatus::kLower) ++colLower;
    else if (status == HighsBasisStatus::kBasic) ++colBasic;
    else if (status == HighsBasisStatus::kUpper) ++colUpper;
    else if (status == HighsBasisStatus::kZero) ++colZero;
    else if (status == HighsBasisStatus::kNonbasic) ++colNonbasic;
  }
  int rowLower = 0, rowBasic = 0, rowUpper = 0, rowZero = 0, rowNonbasic = 0;
  for (HighsBasisStatus status : basis.row_status) {
    if (status == HighsBasisStatus::kLower) ++rowLower;
    else if (status == HighsBasisStatus::kBasic) ++rowBasic;
    else if (status == HighsBasisStatus::kUpper) ++rowUpper;
    else if (status == HighsBasisStatus::kZero) ++rowZero;
    else if (status == HighsBasisStatus::kNonbasic) ++rowNonbasic;
  }

  int basicCol = 0, basicRow = 0, invalid = 0, degenerate = 0, fracBasic = 0;
  std::vector<unsigned char> seen(static_cast<std::size_t>(numCol + numRow),
                                  0);
  int duplicateBasic = 0;
  for (HighsInt i = 0; i < numRow; ++i) {
    const HighsInt basicVar = basisinds[i];
    if (basicVar < 0 || basicVar >= numCol + numRow) {
      ++invalid;
      continue;
    }
    if (seen[static_cast<std::size_t>(basicVar)] != 0) ++duplicateBasic;
    seen[static_cast<std::size_t>(basicVar)] = 1;
    double value = 0.0;
    if (basicVar < numCol) {
      ++basicCol;
      value = solution.col_value[basicVar];
    } else {
      ++basicRow;
      value = solution.row_value[basicVar - numCol];
    }
    if (std::abs(value) <= 1e-8) ++degenerate;
    const double frac = std::abs(value - std::round(value));
    if (std::isfinite(frac) && frac > 1e-8 && frac < 0.5 + 1e-8) {
      ++fracBasic;
    }
  }

  struct DuplicateRows {
    int total{0};
    std::vector<HighsInt> rows;
    std::vector<HighsInt> basisRows;
    std::vector<HighsInt> basicVars;
  };
  std::unordered_map<std::uint64_t, DuplicateRows> rowClasses;
  rowClasses.reserve(static_cast<std::size_t>(2 * numRow + 1));
  for (HighsInt row = 0; row < numRow; ++row) {
    auto& bucket =
        rowClasses[highsLpBasisSemanticRowHash(lpRelaxation, row)];
    ++bucket.total;
    if (static_cast<int>(bucket.rows.size()) < maxTerms) {
      bucket.rows.push_back(row);
    }
  }
  for (HighsInt i = 0; i < numRow; ++i) {
    const HighsInt basicVar = basisinds[i];
    if (basicVar < numCol || basicVar >= numCol + numRow) continue;
    const HighsInt row = basicVar - numCol;
    auto it = rowClasses.find(highsLpBasisSemanticRowHash(lpRelaxation, row));
    if (it == rowClasses.end()) continue;
    if (static_cast<int>(it->second.basisRows.size()) < maxTerms) {
      it->second.basisRows.push_back(i);
      it->second.basicVars.push_back(basicVar);
    }
  }

  int duplicateClasses = 0;
  int duplicateRows = 0;
  int duplicateBasicClasses = 0;
  int duplicateBasicRows = 0;
  std::ostringstream dupSample;
  int dupEmitted = 0;
  for (const auto& entry : rowClasses) {
    const DuplicateRows& bucket = entry.second;
    if (bucket.total <= 1) continue;
    ++duplicateClasses;
    duplicateRows += bucket.total;
    if (!bucket.basisRows.empty()) {
      ++duplicateBasicClasses;
      duplicateBasicRows += static_cast<int>(bucket.basisRows.size());
    }
    if (dupEmitted < maxTerms && !bucket.basisRows.empty()) {
      if (dupEmitted > 0) dupSample << ";";
      dupSample << std::hex << entry.first << std::dec << ":total"
                << bucket.total << ":rows[";
      for (std::size_t k = 0; k < bucket.rows.size(); ++k) {
        if (k > 0) dupSample << ",";
        dupSample << bucket.rows[k];
      }
      dupSample << "]:basic[";
      for (std::size_t k = 0; k < bucket.basisRows.size(); ++k) {
        if (k > 0) dupSample << ",";
        dupSample << bucket.basisRows[k] << "->" << bucket.basicVars[k];
      }
      dupSample << "]";
      ++dupEmitted;
    }
  }

  std::ostringstream sample;
  sample << std::setprecision(12);
  int emitted = 0;
  for (HighsInt i = 0; i < numRow && emitted < maxTerms; ++i) {
    const HighsInt basicVar = basisinds[i];
    if (emitted > 0) sample << ";";
    if (basicVar < 0 || basicVar >= numCol + numRow) {
      sample << i << ":" << basicVar << ":invalid";
      ++emitted;
      continue;
    }
    if (basicVar < numCol) {
      const HighsBasisStatus status =
          basicVar < static_cast<HighsInt>(basis.col_status.size())
              ? basis.col_status[basicVar]
              : HighsBasisStatus::kNonbasic;
      const double value = solution.col_value[basicVar];
      sample << i << ":" << basicVar << ":col:status"
             << highsBasisStatusName(status) << ":val" << value << ":frac"
             << std::abs(value - std::round(value));
    } else {
      const HighsInt row = basicVar - numCol;
      const HighsBasisStatus status =
          row < static_cast<HighsInt>(basis.row_status.size())
              ? basis.row_status[row]
              : HighsBasisStatus::kNonbasic;
      const double value = solution.row_value[row];
      sample << i << ":" << basicVar << ":row" << row << ":status"
             << highsBasisStatusName(status) << ":val" << value << ":lb"
             << lpRelaxation.slackLower(row) << ":ub"
             << lpRelaxation.slackUpper(row) << ":frac"
             << std::abs(value - std::round(value)) << ":rh" << std::hex
             << highsLpBasisSemanticRowHash(lpRelaxation, row) << std::dec;
    }
    ++emitted;
  }

  std::fprintf(
      stderr,
      "[HIGHS-LPBASIS] stage=%s valid=%d alien=%d useful=%d origin=%s "
      "debugId=%lld update=%lld m=%lld n=%lld "
      "statusCol=lower%d:basic%d:upper%d:zero%d:nonbasic%d "
      "statusRow=lower%d:basic%d:upper%d:zero%d:nonbasic%d "
      "basic=col%d:row%d invalid=%d dupBasic=%d deg=%d frac=%d "
      "dupLogical=classes%d:rows%d:basicClasses%d:basicRows%d "
      "dupSample=[%s] sample=[%s]\n",
      stage == nullptr ? "unknown" : stage, basis.valid ? 1 : 0,
      basis.alien ? 1 : 0, basis.useful ? 1 : 0,
      basis.debug_origin_name.c_str(), static_cast<long long>(basis.debug_id),
      static_cast<long long>(basis.debug_update_count),
      static_cast<long long>(numRow), static_cast<long long>(numCol),
      colLower, colBasic, colUpper, colZero, colNonbasic, rowLower, rowBasic,
      rowUpper, rowZero, rowNonbasic, basicCol, basicRow, invalid,
      duplicateBasic, degenerate, fracBasic, duplicateClasses, duplicateRows,
      duplicateBasicClasses, duplicateBasicRows, dupSample.str().c_str(),
      sample.str().c_str());
  std::fflush(stderr);
}

void highsTableauTraceFractionalRows(
    const char* stage, const std::vector<FractionalInteger>& rows,
    const char* extra, const HighsInt* basisinds) {
  if (!highsTableauTraceBasisEnabled()) return;
  const int maxTerms = highsTableauTraceTerms();
  std::fprintf(stderr, "[HIGHS-XROW] id=0 family=tableau stage=%s %s count=%zu "
                       "sample=[",
               stage, extra == nullptr ? "" : extra, rows.size());
  int emitted = 0;
  for (const FractionalInteger& row : rows) {
    if (emitted >= maxTerms) break;
    if (emitted > 0) std::fprintf(stderr, ";");
    std::fprintf(stderr, "%lld:%lld:%.17g:%.17g:%zu",
                 static_cast<long long>(row.basisIndex),
                 static_cast<long long>(basisinds[row.basisIndex]),
                 row.fractionality, row.score, row.row_ep.size());
    ++emitted;
  }
  if (static_cast<int>(rows.size()) > emitted) std::fprintf(stderr, ";...");
  std::fprintf(stderr, "]\n");
}

void highsTableauTraceRowEp(const FractionalInteger& fracvar,
                            const HighsInt* basisinds, HighsInt rawCount,
                            HighsInt keptCount, double minWeight,
                            double maxWeight, double norm2,
                            const char* reject) {
  if (!highsTableauTraceBasisEnabled()) return;
  std::fprintf(stderr,
               "[HIGHS-XROW] id=0 family=tableau stage=rowep "
               "basisRow=%lld basicVar=%lld raw=%lld kept=%lld "
               "min=%.12g max=%.12g ratio=%.12g norm2=%.12g "
               "frac=%.12g score=%.12g reject=%s\n",
               static_cast<long long>(fracvar.basisIndex),
               static_cast<long long>(basisinds[fracvar.basisIndex]),
               static_cast<long long>(rawCount),
               static_cast<long long>(keptCount), minWeight, maxWeight,
               minWeight > 0.0 ? maxWeight / minWeight : kHighsInf, norm2,
               fracvar.fractionality, fracvar.score, reject);
}

std::string highsTableauRowSignature(const HighsLpRelaxation& lpRelaxation,
                                     HighsInt row, int maxTerms) {
  std::ostringstream out;
  out << std::setprecision(12);
  HighsInt len = 0;
  const HighsInt* rowinds = nullptr;
  const double* rowvals = nullptr;
  lpRelaxation.getRow(row, len, rowinds, rowvals);
  HighsInt emitted = 0;
  for (HighsInt k = 0; k < len && emitted < maxTerms; ++k) {
    if (std::abs(rowvals[k]) <= 1e-12) continue;
    if (emitted > 0) out << ",";
    out << rowinds[k] << ":" << rowvals[k];
    ++emitted;
  }
  if (len > emitted) out << ",...";
  return out.str();
}

std::string highsTableauBasisTraceInfo(
    const HighsLpRelaxation& lpRelaxation, const HighsInt* basisinds,
    const FractionalInteger& fracvar, bool negatedAggregation) {
  if (!highsTableauTraceBasisEnabled()) return {};
  const HighsInt numCol = lpRelaxation.numCols();
  const HighsInt basicVar = basisinds[fracvar.basisIndex];
  const bool basicRow = basicVar >= numCol;
  const HighsInt basicLogicalRow = basicRow ? basicVar - numCol : -1;
  const int maxTerms = highsTableauTraceTerms();

  std::ostringstream out;
  out << std::setprecision(12);
  out << "basisRow=" << fracvar.basisIndex << " basicVar=" << basicVar
      << " basicKind=" << (basicRow ? "row" : "col")
      << " basicLogicalRow=" << basicLogicalRow
      << " sign=" << (negatedAggregation ? -1 : 1)
      << " frac=" << fracvar.fractionality << " score=" << fracvar.score
      << " rowEp=[";
  int emitted = 0;
  for (const std::pair<HighsInt, double>& rowWeight : fracvar.row_ep) {
    if (emitted >= maxTerms) break;
    if (emitted > 0) out << ";";
    const HighsInt row = rowWeight.first;
    out << "row" << row << ":w" << rowWeight.second << ":lb"
        << lpRelaxation.slackLower(row) << ":ub"
        << lpRelaxation.slackUpper(row) << ":sig["
        << highsTableauRowSignature(lpRelaxation, row, maxTerms) << "]";
    ++emitted;
  }
  if (static_cast<int>(fracvar.row_ep.size()) > emitted) out << ";...";
  out << "]";
  return out.str();
}

}  // namespace

void HighsTableauSeparator::separateLpSolution(HighsLpRelaxation& lpRelaxation,
                                               HighsLpAggregator& lpAggregator,
                                               HighsTransformedLp& transLp,
                                               HighsCutPool& cutpool) {
  Highs& lpSolver = lpRelaxation.getLpSolver();
  if (!lpSolver.hasInvert()) return;

  const HighsMipSolver& mip = lpRelaxation.getMipSolver();
  if (cutpool.getNumAvailableCuts() > mip.options_mip_->mip_pool_soft_limit)
    return;

  const HighsInt* basisinds =
      lpRelaxation.getLpSolver().getBasicVariablesArray();
  HighsInt numRow = lpRelaxation.numRows();
  HighsInt numCol = lpRelaxation.numCols();
  highsTraceLpBasisState(lpRelaxation, basisinds, "tableau_entry");

  HighsCutGeneration cutGen(lpRelaxation, cutpool);

  std::vector<HighsInt> baseRowInds;
  std::vector<double> baseRowVals;

  const HighsSolution& lpSolution = lpRelaxation.getSolution();

  std::vector<FractionalInteger> fractionalBasisvars;
  fractionalBasisvars.reserve(numRow);
  for (HighsInt i = 0; i < numRow; ++i) {
    double my_fractionality;
    if (basisinds[i] >= numCol) {
      HighsInt row = basisinds[i] - numCol;

      if (!lpRelaxation.isRowIntegral(row)) continue;

      my_fractionality = fractionality(lpSolution.row_value[row]);
    } else {
      HighsInt col = basisinds[i];
      if (mip.isColContinuous(col)) continue;

      my_fractionality = fractionality(lpSolution.col_value[col]);
    }

    if (my_fractionality < 1000 * mip.mipdata_->feastol) continue;

    fractionalBasisvars.emplace_back(i, my_fractionality);
  }

  highsTraceLpBasisState(lpRelaxation, basisinds, "frac_scan_ready");
  highsTableauTraceFractionalRows("frac_scan", fractionalBasisvars,
                                  "key=basis_order", basisinds);
  if (fractionalBasisvars.empty()) return;
  int64_t maxTries = 5000 + getNumCalls() * 50 +
                     (mip.mipdata_->total_lp_iterations -
                      mip.mipdata_->heuristic_lp_iterations) /
                         10;
  if (numTries >= maxTries) return;

  maxTries -= numTries;

  maxTries = std::min(
      {maxTries,
       200 + int64_t(0.1 *
                     std::min(numRow,
                              (HighsInt)mip.mipdata_->integral_cols.size()))});

  const bool traceTruncated =
      static_cast<HighsInt>(fractionalBasisvars.size()) > maxTries;
  if (traceTruncated) {
    const double* edgeWt = lpRelaxation.getLpSolver().getDualEdgeWeights();
    if (edgeWt) {
      // printf("choosing %ld/%zu with DSE weights\n", maxTries,
      // fractionalBasisvars.size());
      pdqsort(
          fractionalBasisvars.begin(), fractionalBasisvars.end(),
          [&](const FractionalInteger& fracint1,
              const FractionalInteger& fracint2) {
            double score1 = fracint1.fractionality *
                            (1.0 - fracint1.fractionality) /
                            edgeWt[fracint1.basisIndex];
            double score2 = fracint2.fractionality *
                            (1.0 - fracint2.fractionality) /
                            edgeWt[fracint2.basisIndex];
            return std::make_pair(score1, HighsHashHelpers::hash(
                                              numTries + fracint1.basisIndex)) >
                   std::make_pair(score2, HighsHashHelpers::hash(
                                              numTries + fracint2.basisIndex));
          });
    } else {
      // printf("choosing %ld/%zu without DSE weights\n", maxTries,
      // fractionalBasisvars.size());
      pdqsort(
          fractionalBasisvars.begin(), fractionalBasisvars.end(),
          [&](const FractionalInteger& fracint1,
              const FractionalInteger& fracint2) {
            return std::make_pair(
                       fracint1.fractionality,
                       HighsHashHelpers::hash(numTries + fracint1.basisIndex)) >
                   std::make_pair(
                       fracint2.fractionality,
                       HighsHashHelpers::hash(numTries + fracint2.basisIndex));
          });
    }

    fractionalBasisvars.resize(maxTries);
  }
  {
    std::string extra =
        "maxTries=" + std::to_string(maxTries) +
        " triesBefore=" + std::to_string(numTries) +
        " truncated=" + (traceTruncated ? "1" : "0") +
        " key=" +
        (traceTruncated
             ? (lpRelaxation.getLpSolver().getDualEdgeWeights()
                    ? "dse_hash"
                    : "fractionality_hash")
             : "basis_order");
    highsTableauTraceFractionalRows("pre_rowep_order", fractionalBasisvars,
                                    extra.c_str(), basisinds);
  }

  HVector rowEpBuffer;
  rowEpBuffer.setup(numRow);

  numTries += fractionalBasisvars.size();

  for (auto& fracvar : fractionalBasisvars) {
    if (lpSolver.getBasisInverseRowSparse(fracvar.basisIndex, rowEpBuffer) !=
        HighsStatus::kOk)
      continue;

    // handled by other separator
    if (rowEpBuffer.count == 1) {
      highsTableauTraceRowEp(fracvar, basisinds, rowEpBuffer.count, 1,
                             kHighsInf, 0.0, 0.0, "count");
      continue;
    }

    fracvar.row_ep_norm2 = 0.0;
    double minWeight = kHighsInf;
    double maxWeight = 0.0;
    fracvar.row_ep.reserve(rowEpBuffer.count);
    for (HighsInt j = 0; j < rowEpBuffer.count; ++j) {
      HighsInt row = rowEpBuffer.index[j];
      double weight = rowEpBuffer.array[row];
      double maxAbsRowVal = lpRelaxation.getMaxAbsRowVal(row);

      double scaledWeight = maxAbsRowVal * std::abs(weight);
      if (scaledWeight <= mip.mipdata_->feastol) continue;

      minWeight = std::min(minWeight, scaledWeight);
      maxWeight = std::max(maxWeight, scaledWeight);
      fracvar.row_ep_norm2 += scaledWeight * scaledWeight;
      fracvar.row_ep.emplace_back(row, weight);
    }

    if (fracvar.row_ep.size() <= 1) {
      highsTableauTraceRowEp(fracvar, basisinds, rowEpBuffer.count,
                             fracvar.row_ep.size(), minWeight, maxWeight,
                             fracvar.row_ep_norm2, "count");
      continue;
    }

    if (maxWeight / minWeight <= 1e4) {
      fracvar.score = fracvar.fractionality * (1.0 - fracvar.fractionality) /
                      fracvar.row_ep_norm2;
      highsTableauTraceRowEp(fracvar, basisinds, rowEpBuffer.count,
                             fracvar.row_ep.size(), minWeight, maxWeight,
                             fracvar.row_ep_norm2, "none");
    } else {
      highsTableauTraceRowEp(fracvar, basisinds, rowEpBuffer.count,
                             fracvar.row_ep.size(), minWeight, maxWeight,
                             fracvar.row_ep_norm2, "ratio");
    }
  }

  fractionalBasisvars.erase(
      std::remove_if(fractionalBasisvars.begin(), fractionalBasisvars.end(),
                     [&](const FractionalInteger& fracInteger) {
                       return fracInteger.score <= mip.mipdata_->feastol;
                     }),
      fractionalBasisvars.end());

  if (fractionalBasisvars.empty()) return;

  pdqsort_branchless(fractionalBasisvars.begin(), fractionalBasisvars.end());
  highsTableauTraceFractionalRows("post_rowep_order", fractionalBasisvars,
                                  "key=score", basisinds);
  double bestScore = -1.0;

  HighsInt numCuts = cutpool.getNumCuts();
  const double bestScoreFac[] = {0.0025, 0.01};

  for (const auto& fracvar : fractionalBasisvars) {
    if (cutpool.getNumCuts() - numCuts >= 1000) break;

    if (fracvar.score <
        bestScoreFac[cutpool.getNumCuts() - numCuts >= 50] * bestScore)
      break;

    assert(lpAggregator.isEmpty());
    for (std::pair<HighsInt, double> rowWeight : fracvar.row_ep)
      lpAggregator.addRow(rowWeight.first, rowWeight.second);

    lpAggregator.getCurrentAggregation(baseRowInds, baseRowVals, false);

    if (10 * (baseRowInds.size() - fracvar.row_ep.size()) >
        10000 + static_cast<size_t>(mip.numCol())) {
      lpAggregator.clear();
      continue;
    }

    HighsInt len = baseRowInds.size();
    if (len > (HighsInt)fracvar.row_ep.size()) {
      double maxAbsVal = 0.0;
      double minAbsVal = kHighsInf;
      for (HighsInt i = 0; i < len; ++i) {
        if (baseRowInds[i] < mip.numCol()) {
          maxAbsVal = std::max(std::abs(baseRowVals[i]), maxAbsVal);
          minAbsVal = std::min(std::abs(baseRowVals[i]), minAbsVal);
        }
      }
      if (maxAbsVal / minAbsVal > 1e6) {
        lpAggregator.clear();
        continue;
      }
    }

    mip.mipdata_->debugSolution.checkRowAggregation(
        lpSolver.getLp(), baseRowInds.data(), baseRowVals.data(),
        baseRowInds.size());

    double rhs = 0;
    const std::string basisTraceInfo =
        highsTableauBasisTraceInfo(lpRelaxation, basisinds, fracvar,
                                   /*negatedAggregation=*/false);
    cutGen.generateCut(transLp, baseRowInds, baseRowVals, rhs, false,
                       "tableau",
                       basisTraceInfo.empty() ? nullptr
                                              : basisTraceInfo.c_str());
    if (mip.mipdata_->domain.infeasible()) break;

    lpAggregator.getCurrentAggregation(baseRowInds, baseRowVals, true);
    rhs = 0;
    const std::string negatedBasisTraceInfo =
        highsTableauBasisTraceInfo(lpRelaxation, basisinds, fracvar,
                                   /*negatedAggregation=*/true);
    cutGen.generateCut(transLp, baseRowInds, baseRowVals, rhs, false,
                       "tableau",
                       negatedBasisTraceInfo.empty()
                           ? nullptr
                           : negatedBasisTraceInfo.c_str());
    if (mip.mipdata_->domain.infeasible()) break;

    lpAggregator.clear();
    if (bestScore == -1.0 && cutpool.getNumCuts() != numCuts)
      bestScore = fracvar.score;
  }
}
