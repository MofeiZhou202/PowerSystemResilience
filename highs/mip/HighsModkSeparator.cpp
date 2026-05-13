/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/**@file mip/HighsModKSeparator.cpp
 */

#include "mip/HighsModkSeparator.h"

#include <unordered_set>
#include <cstdlib>
#include <string>

#include "../extern/pdqsort/pdqsort.h"
#include "mip/HighsCutGeneration.h"
#include "mip/HighsGFkSolve.h"
#include "mip/HighsLpAggregator.h"
#include "mip/HighsLpRelaxation.h"
#include "mip/HighsMipSolverData.h"
#include "mip/HighsTransformedLp.h"
#include "io/HighsIO.h"
#include "util/HighsHash.h"
#include "util/HighsIntegers.h"

static uint64_t hacdcpfModkHashMix(uint64_t h, uint64_t v) {
  v ^= v >> 33;
  v *= uint64_t{0xff51afd7ed558ccd};
  v ^= v >> 33;
  v *= uint64_t{0xc4ceb9fe1a85ec53};
  v ^= v >> 33;
  return h ^ (v + uint64_t{0x9e3779b97f4a7c15} + (h << 6) + (h >> 2));
}

static uint64_t hacdcpfModkSystemHash(
    const std::vector<int64_t>& intSystemValue,
    const std::vector<HighsInt>& intSystemIndex,
    const std::vector<HighsInt>& intSystemStart) {
  uint64_t h = uint64_t{0x48494748534d4f44};
  h = hacdcpfModkHashMix(h, static_cast<uint64_t>(intSystemValue.size()));
  h = hacdcpfModkHashMix(h, static_cast<uint64_t>(intSystemIndex.size()));
  h = hacdcpfModkHashMix(h, static_cast<uint64_t>(intSystemStart.size()));
  for (HighsInt start : intSystemStart)
    h = hacdcpfModkHashMix(h, static_cast<uint64_t>(start));
  for (HighsInt index : intSystemIndex)
    h = hacdcpfModkHashMix(h, static_cast<uint64_t>(index));
  for (int64_t value : intSystemValue)
    h = hacdcpfModkHashMix(h, static_cast<uint64_t>(value));
  return h;
}

static int hacdcpfModkTraceLimit() {
  const char* env = std::getenv("HACDCPF_XMODK_SYSTEM_TRACE");
  if (env == nullptr || env[0] == '\0') return 0;
  if (std::string(env) == "all") return 1000000000;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return 40;
  return value > 0 ? static_cast<int>(std::min<long>(value, 1000000)) : 0;
}

static int hacdcpfModkTraceTerms() {
  const char* env = std::getenv("HACDCPF_XMODK_SYSTEM_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 16;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 16;
  return static_cast<int>(std::min<long>(value, 200));
}

static HighsInt hacdcpfModkTransformTraceRow() {
  const char* env = std::getenv("HACDCPF_XMODK_TRANSFORM_ROW");
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<HighsInt>(value);
}

template <HighsInt k, typename FoundModKCut>
static bool separateModKCuts(const std::vector<int64_t>& intSystemValue,
                             const std::vector<HighsInt>& intSystemIndex,
                             const std::vector<HighsInt>& intSystemStart,
                             const HighsCutPool& cutpool, HighsInt numCol,
                             FoundModKCut&& foundModKCut) {
  HighsGFkSolve GFkSolve;

  HighsInt numCuts = cutpool.getNumCuts();

  GFkSolve.fromCSC<k>(intSystemValue, intSystemIndex, intSystemStart,
                      numCol + 1);
  GFkSolve.setRhs<k>(numCol, 1);
  GFkSolve.solve<k>(foundModKCut);

  return cutpool.getNumCuts() != numCuts;
}

void HighsModkSeparator::separateLpSolution(HighsLpRelaxation& lpRelaxation,
                                            HighsLpAggregator& lpAggregator,
                                            HighsTransformedLp& transLp,
                                            HighsCutPool& cutpool) {
  const HighsMipSolver& mipsolver = lpRelaxation.getMipSolver();
  const HighsLp& lp = lpRelaxation.getLp();
  const bool traceLedger = std::getenv("HACDCPF_XPATH_LEDGER") != nullptr ||
                           std::getenv("HACDCPF_XTAB_DIAG") != nullptr;
  const int systemTraceLimit = hacdcpfModkTraceLimit();
  const int systemTraceTerms = hacdcpfModkTraceTerms();
  struct XModkLedger {
    HighsInt skippedContinuous{0};
    HighsInt activeLe{0};
    HighsInt activeGe{0};
    HighsInt inactive{0};
    HighsInt transformOk{0};
    HighsInt transformFail{0};
    HighsInt integralRows{0};
    HighsInt scaledRows{0};
    HighsInt longSkip{0};
    HighsInt zeroSkip{0};
    HighsInt systemNnz{0};
    HighsInt nonzeroRhs{0};
    HighsInt gf2Calls{0};
    HighsInt gf3Calls{0};
    HighsInt gf5Calls{0};
    HighsInt gf7Calls{0};
    HighsInt solutions{0};
    HighsInt cutgenCalls{0};
    HighsInt cutgenSuccess{0};
    uint64_t systemHash{0};
  } ledger;
  const HighsInt cutpoolStart = cutpool.getNumCuts();
  auto printLedger = [&](const char* stage) {
    if (!traceLedger) return;
    highsLogUser(
        mipsolver.options_mip_->log_options, HighsLogType::kInfo,
        "[HIGHS-XMODK-LEDGER] stage=%s mode=cutpool srcRows=%lld srcDim=%lld "
        "skipCont=%lld active=le%lld:ge%lld inactive=%lld transform=%lld/%lld "
        "rows=int%lld:scaled%lld longSkip=%lld zeroSkip=%lld systemNnz=%lld "
        "rhsNz=%lld gf=2:%lld:3:%lld:5:%lld:7:%lld sol=%lld "
        "cutgen=%lld/%lld poolAccepted=%lld sysHash=%016llx\n",
        stage, static_cast<long long>(lp.num_row_),
        static_cast<long long>(lp.num_row_ + lp.num_col_),
        static_cast<long long>(ledger.skippedContinuous),
        static_cast<long long>(ledger.activeLe),
        static_cast<long long>(ledger.activeGe),
        static_cast<long long>(ledger.inactive),
        static_cast<long long>(ledger.transformOk),
        static_cast<long long>(ledger.transformFail),
        static_cast<long long>(ledger.integralRows),
        static_cast<long long>(ledger.scaledRows),
        static_cast<long long>(ledger.longSkip),
        static_cast<long long>(ledger.zeroSkip),
        static_cast<long long>(ledger.systemNnz),
        static_cast<long long>(ledger.nonzeroRhs),
        static_cast<long long>(ledger.gf2Calls),
        static_cast<long long>(ledger.gf3Calls),
        static_cast<long long>(ledger.gf5Calls),
        static_cast<long long>(ledger.gf7Calls),
        static_cast<long long>(ledger.solutions),
        static_cast<long long>(ledger.cutgenSuccess),
        static_cast<long long>(ledger.cutgenCalls),
        static_cast<long long>(cutpool.getNumCuts() - cutpoolStart),
        static_cast<unsigned long long>(ledger.systemHash));
  };

  std::vector<uint8_t> skipRow(lp.num_row_);

  // mark all rows that have continuous variables with a nonzero solution value
  // in the transformed LP to be skipped
  for (HighsInt col : mipsolver.mipdata_->continuous_cols) {
    if (transLp.boundDistance(col) == 0) continue;

    const HighsInt start = lp.a_matrix_.start_[col];
    const HighsInt end = lp.a_matrix_.start_[col + 1];

    for (HighsInt i = start; i != end; ++i) {
      if (!skipRow[lp.a_matrix_.index_[i]]) ++ledger.skippedContinuous;
      skipRow[lp.a_matrix_.index_[i]] = true;
    }
  }

  HighsCutGeneration cutGen(lpRelaxation, cutpool);

  std::vector<std::pair<HighsInt, double>> integralScales;
  std::vector<int64_t> intSystemValue;
  std::vector<HighsInt> intSystemIndex;
  std::vector<HighsInt> intSystemStart;

  intSystemValue.reserve(lp.a_matrix_.value_.size() + lp.num_row_);
  intSystemIndex.reserve(intSystemValue.size());
  intSystemStart.reserve(lp.num_row_ + 1);
  intSystemStart.push_back(0);

  std::vector<HighsInt> inds;
  std::vector<double> vals;
  std::vector<double> scaleVals;

  inds.reserve(lp.num_col_);
  vals.reserve(lp.num_col_);
  scaleVals.reserve(lp.num_col_);

  std::vector<double> upper;
  std::vector<double> solval;
  double rhs;

  const HighsSolution& lpSolution = lpRelaxation.getSolution();
  HighsInt numNonzeroRhs = 0;
  HighsInt maxIntRowLen = 1000 + 0.1 * lp.num_col_;

  for (HighsInt row = 0; row != lp.num_row_; ++row) {
    if (skipRow[row]) continue;

    bool leqRow;

    if (lp.row_upper_[row] - lpSolution.row_value[row] <=
        mipsolver.mipdata_->feastol) {
      leqRow = true;
      ++ledger.activeLe;
    } else if (lpSolution.row_value[row] - lp.row_lower_[row] <=
               mipsolver.mipdata_->feastol) {
      leqRow = false;
      ++ledger.activeGe;
    } else {
      ++ledger.inactive;
      continue;
    }

    HighsInt rowlen;
    const HighsInt* rowinds;
    const double* rowvals;

    lpRelaxation.getRow(row, rowlen, rowinds, rowvals);
    const HighsInt sourceRowLen = rowlen;

    if (leqRow) {
      rhs = lp.row_upper_[row];
      inds.assign(rowinds, rowinds + rowlen);
      vals.assign(rowvals, rowvals + rowlen);
    } else {
      assert(lpSolution.row_value[row] - lp.row_lower_[row] <=
             mipsolver.mipdata_->feastol);

      rhs = -lp.row_lower_[row];
      inds.assign(rowinds, rowinds + rowlen);
      vals.resize(rowlen);
      std::transform(rowvals, rowvals + rowlen, vals.begin(),
                     [](double x) { return -x; });
    }
    std::vector<HighsInt> rawInds;
    std::vector<double> rawVals;
    if (row == hacdcpfModkTransformTraceRow()) {
      rawInds = inds;
      rawVals = vals;
    }

    bool integralPositive = false;
    if (!transLp.transform(vals, upper, solval, inds, rhs, integralPositive,
                           true)) {
      ++ledger.transformFail;
      continue;
    }
    ++ledger.transformOk;
    if (row == hacdcpfModkTransformTraceRow()) {
      std::string raw;
      const HighsInt rawEmit =
          std::min<HighsInt>(rawInds.size(), static_cast<HighsInt>(200));
      for (HighsInt j = 0; j != rawEmit; ++j) {
        if (!raw.empty()) raw += ",";
        raw += std::to_string(static_cast<long long>(rawInds[j]));
        raw += ":";
        raw += std::to_string(rawVals[j]);
      }
      if (static_cast<HighsInt>(rawInds.size()) > rawEmit) raw += ",...";

      std::string trans;
      const HighsInt transEmit =
          std::min<HighsInt>(inds.size(), static_cast<HighsInt>(200));
      for (HighsInt j = 0; j != transEmit; ++j) {
        if (!trans.empty()) trans += ",";
        trans += std::to_string(static_cast<long long>(inds[j]));
        trans += ":";
        trans += std::to_string(vals[j]);
        trans += ":";
        trans += std::to_string(solval[j]);
        trans += ":";
        trans += mipsolver.isColContinuous(inds[j]) ? "0" : "1";
      }
      if (static_cast<HighsInt>(inds.size()) > transEmit) trans += ",...";
      highsLogUser(
          mipsolver.options_mip_->log_options, HighsLogType::kInfo,
          "[HIGHS-XMODK-TRANSFORM] row=%lld side=%s rawLen=%lld rhs=%.17g "
          "raw=[%s] transLen=%lld transRhs=%.17g trans=[%s]\n",
          static_cast<long long>(row), leqRow ? "le" : "ge",
          static_cast<long long>(sourceRowLen), leqRow ? lp.row_upper_[row]
                                                       : -lp.row_lower_[row],
          raw.c_str(), static_cast<long long>(inds.size()), rhs,
          trans.c_str());
    }

    rowlen = inds.size();
    if (rowlen > maxIntRowLen) {
      HighsInt intRowLen = 0;
      for (HighsInt i = 0; i < rowlen; ++i) {
        if (solval[i] <= mipsolver.mipdata_->feastol) continue;
        if (mipsolver.isColContinuous(inds[i])) continue;
        ++intRowLen;
      }

      // skip row if either too long or 0 = 0 row
      if (intRowLen > maxIntRowLen ||
          (intRowLen == 0 && fabs(rhs) <= mipsolver.mipdata_->epsilon)) {
        ++ledger.longSkip;
        continue;
      }
    }

    double intscale;
    int64_t intrhs;
    const HighsInt systemRowStart = intSystemStart.back();
    const bool rowIntegral = lpRelaxation.isRowIntegral(row);

    if (!rowIntegral) {
      scaleVals.clear();
      for (HighsInt i = 0; i != rowlen; ++i) {
        if (mipsolver.isColContinuous(inds[i])) continue;
        if (solval[i] > mipsolver.mipdata_->feastol) {
          scaleVals.push_back(vals[i]);
        }
      }

      if (fabs(rhs) > mipsolver.mipdata_->epsilon) scaleVals.push_back(-rhs);
      if (scaleVals.empty()) {
        ++ledger.zeroSkip;
        continue;
      }

      intscale = HighsIntegers::integralScale(
          scaleVals, mipsolver.mipdata_->feastol, mipsolver.mipdata_->epsilon);
      if (intscale == 0.0 || intscale > 1e6) {
        ++ledger.zeroSkip;
        continue;
      }

      intrhs = HighsIntegers::nearestInteger(intscale * rhs);

      for (HighsInt i = 0; i != rowlen; ++i) {
        if (mipsolver.isColContinuous(inds[i])) continue;
        if (solval[i] > mipsolver.mipdata_->feastol) {
          intSystemIndex.push_back(inds[i]);
          intSystemValue.push_back(
              HighsIntegers::nearestInteger(intscale * vals[i]));
        }
      }
      ++ledger.scaledRows;
    } else {
      intscale = 1.0;
      intrhs = HighsIntegers::nearestInteger(rhs);

      for (HighsInt i = 0; i != rowlen; ++i) {
        if (solval[i] > mipsolver.mipdata_->feastol) {
          intSystemIndex.push_back(inds[i]);
          intSystemValue.push_back(HighsIntegers::nearestInteger(vals[i]));
        }
      }
      ++ledger.integralRows;
    }

    numNonzeroRhs += (intrhs != 0);

    intSystemIndex.push_back(lp.num_col_);
    intSystemValue.push_back(intrhs);
    intSystemStart.push_back(intSystemValue.size());
    if (systemTraceLimit > 0 &&
        static_cast<HighsInt>(integralScales.size()) < systemTraceLimit) {
      std::string terms;
      const HighsInt systemRowEnd = intSystemValue.size();
      const HighsInt emitEnd =
          std::min<HighsInt>(systemRowEnd, systemRowStart + systemTraceTerms);
      for (HighsInt p = systemRowStart; p != emitEnd; ++p) {
        if (!terms.empty()) terms += ",";
        terms += std::to_string(static_cast<long long>(intSystemIndex[p]));
        terms += ":";
        terms += std::to_string(static_cast<long long>(intSystemValue[p]));
      }
      if (systemRowEnd > emitEnd) terms += ",...";
      highsLogUser(
          mipsolver.options_mip_->log_options, HighsLogType::kInfo,
          "[HIGHS-XMODK-SYS] seq=%lld row=%lld side=%s rowIntegral=%d "
          "intscale=%.17g intrhs=%lld rawLen=%lld transLen=%lld start=%lld "
          "end=%lld terms=[%s]\n",
          static_cast<long long>(integralScales.size()),
          static_cast<long long>(row), leqRow ? "le" : "ge",
          rowIntegral ? 1 : 0, intscale, static_cast<long long>(intrhs),
          static_cast<long long>(sourceRowLen),
          static_cast<long long>(inds.size()),
          static_cast<long long>(systemRowStart),
          static_cast<long long>(systemRowEnd), terms.c_str());
    }
    integralScales.emplace_back(row, intscale);
  }

  ledger.systemNnz = intSystemValue.size();
  ledger.nonzeroRhs = numNonzeroRhs;
  ledger.systemHash =
      hacdcpfModkSystemHash(intSystemValue, intSystemIndex, intSystemStart);
  if (integralScales.empty() || numNonzeroRhs == 0) {
    printLedger("empty_system");
    return;
  }

  std::vector<HighsInt> tmpinds;
  std::vector<double> tmpvals;

  HighsHashTable<std::vector<HighsGFkSolve::SolutionEntry>> usedWeights;
  // std::unordered_set<std::vector<HighsGFkSolve::SolutionEntry>,
  //                   HighsVectorHasher, HighsVectorEqual>
  //    usedWeights;
  HighsInt k;
  auto foundCut = [&](std::vector<HighsGFkSolve::SolutionEntry>& weights,
                      int rhsIndex) {
    // cuts which come from a single row can already be found with the
    // aggregation heuristic
    if (weights.empty()) return;

    pdqsort(weights.begin(), weights.end());
    if (!usedWeights.insert(weights)) return;
    ++ledger.solutions;

    assert(lpAggregator.isEmpty());
    for (const auto& w : weights) {
      double weight = integralScales[w.index].second *
                      (double((w.weight * (k - 1)) % k) / k);
      HighsInt row = integralScales[w.index].first;
      lpAggregator.addRow(row, weight);
    }

    lpAggregator.getCurrentAggregation(inds, vals, false);

    rhs = 0.0;
    ++ledger.cutgenCalls;
    if (cutGen.generateCut(transLp, inds, vals, rhs, true, "modk"))
      ++ledger.cutgenSuccess;

    if (k != 2) {
      lpAggregator.clear();
      for (const auto& w : weights) {
        double weight = integralScales[w.index].second * (double(w.weight) / k);
        HighsInt row = integralScales[w.index].first;
        lpAggregator.addRow(row, weight);
      }
    }

    lpAggregator.getCurrentAggregation(inds, vals, true);

    rhs = 0.0;
    ++ledger.cutgenCalls;
    if (cutGen.generateCut(transLp, inds, vals, rhs, true, "modk"))
      ++ledger.cutgenSuccess;

    lpAggregator.clear();
  };

  k = 2;
  ledger.gf2Calls = 1;
  if (separateModKCuts<2>(intSystemValue, intSystemIndex, intSystemStart,
                          cutpool, lp.num_col_, foundCut)) {
    printLedger("done");
    return;
  }

  usedWeights.clear();
  k = 3;
  ledger.gf3Calls = 1;
  if (separateModKCuts<3>(intSystemValue, intSystemIndex, intSystemStart,
                          cutpool, lp.num_col_, foundCut)) {
    printLedger("done");
    return;
  }

  usedWeights.clear();
  k = 5;
  ledger.gf5Calls = 1;
  if (separateModKCuts<5>(intSystemValue, intSystemIndex, intSystemStart,
                          cutpool, lp.num_col_, foundCut)) {
    printLedger("done");
    return;
  }

  usedWeights.clear();
  k = 7;
  ledger.gf7Calls = 1;
  if (separateModKCuts<7>(intSystemValue, intSystemIndex, intSystemStart,
                          cutpool, lp.num_col_, foundCut)) {
    printLedger("done");
    return;
  }
  printLedger("done");
}
