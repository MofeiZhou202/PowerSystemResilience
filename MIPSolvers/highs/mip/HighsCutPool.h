/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef HIGHS_CUTPOOL_H_
#define HIGHS_CUTPOOL_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lp_data/HConst.h"
#include "mip/HighsDomain.h"
#include "mip/HighsDynamicRowMatrix.h"

class HighsLpRelaxation;

struct HighsCutSet {
  enum HacdcpfValidityScope : unsigned char {
    kHacdcpfGlobalCut = 0,
    kHacdcpfLocalNodeCut = 1,
    kHacdcpfLazyConstraint = 2,
  };

  std::vector<HighsInt> cutindices;
  std::vector<HighsInt> cutpools;
  std::vector<HighsInt> ARstart_;
  std::vector<HighsInt> ARindex_;
  std::vector<double> ARvalue_;
  std::vector<double> lower_;  // Currently only ever contains -kHighsInf
  std::vector<double> upper_;
  std::vector<unsigned char> hacdcpf_integral_;
  std::vector<unsigned char> hacdcpf_propagate_;
  std::vector<unsigned char> hacdcpf_validity_scope_;
  std::vector<std::string> hacdcpf_proof_family_;
  std::vector<std::string> hacdcpf_proof_key_;

  HighsInt numCuts() const { return cutindices.size(); }

  void resize(HighsInt nnz) {
    HighsInt ncuts = numCuts();
    lower_.resize(ncuts, -kHighsInf);
    upper_.resize(ncuts);
    hacdcpf_integral_.resize(ncuts, 0);
    hacdcpf_propagate_.resize(ncuts, 1);
    hacdcpf_validity_scope_.resize(ncuts, kHacdcpfGlobalCut);
    hacdcpf_proof_family_.resize(ncuts);
    hacdcpf_proof_key_.resize(ncuts);
    ARstart_.resize(ncuts + 1);
    ARindex_.resize(nnz);
    ARvalue_.resize(nnz);
  }

  void clear() {
    cutindices.clear();
    cutpools.clear();
    lower_.clear();
    upper_.clear();
    hacdcpf_integral_.clear();
    hacdcpf_propagate_.clear();
    hacdcpf_validity_scope_.clear();
    hacdcpf_proof_family_.clear();
    hacdcpf_proof_key_.clear();
    ARstart_.clear();
    ARindex_.clear();
    ARvalue_.clear();
  }

  bool empty() const { return cutindices.empty(); }
};

struct HacdcpfUserCutAdmissionLimits {
  HighsInt max_global_cuts{0};
  HighsInt max_local_cuts{0};
  HighsInt max_row_len{0};
  double min_violation{0.0};
  double min_efficacy{0.0};
  double max_coeff_ratio{0.0};
  bool require_proof_metadata{true};
};

struct HacdcpfUserCutAdmission {
  bool admitted{false};
  const char* reason{"not_checked"};
  unsigned char scope{HighsCutSet::kHacdcpfGlobalCut};
  HighsInt row{0};
  HighsInt start{0};
  HighsInt end{0};
  HighsInt len{0};
  double activity{0.0};
  double violation{0.0};
  double norm{0.0};
  double efficacy{0.0};
  double max_abs{0.0};
  double min_abs{0.0};
  double coeff_ratio{0.0};
  uint64_t row_hash{0};
  uint64_t row_sig{0};
};

HacdcpfUserCutAdmissionLimits hacdcpfGetUserCutAdmissionLimits(
    bool root_event);

HacdcpfUserCutAdmission hacdcpfAdmitUserCutRow(
    const HighsCutSet& cutset, HighsInt row, const std::vector<double>& sol,
    HighsInt num_col, double feastol,
    const HacdcpfUserCutAdmissionLimits& limits);

void hacdcpfSortUserCutAdmissions(
    std::vector<HacdcpfUserCutAdmission>& admissions);

bool hacdcpfCopyUserCutRow(HighsCutSet& target, const HighsCutSet& source,
                           const HacdcpfUserCutAdmission& admission,
                           unsigned char scope);

uint64_t hacdcpfCutRowHash(const HighsInt* Rindex, const double* Rvalue,
                           HighsInt Rlen);

uint64_t hacdcpfCutSetRowHash(const HighsCutSet& cutset, HighsInt row);

uint64_t hacdcpfCutSetRowSig(const HighsCutSet& cutset, HighsInt row);

bool hacdcpfUserCutLedgerEnabled();

void hacdcpfLogUserCutRow(const char* phase, const char* scope,
                          const char* event, HighsInt round,
                          int64_t node_count, HighsInt depth,
                          const HighsCutSet& cutset, HighsInt row,
                          HighsInt ord, HighsInt cutindex, HighsInt lp_row,
                          double score, double viol, HighsInt active,
                          const std::vector<double>* sol,
                          const char* reason);

class HighsCutPool {
 private:
  HighsDynamicRowMatrix matrix_;
  std::vector<double> rhs_;
  std::vector<int16_t> ages_;
  std::deque<std::atomic<int16_t>> numLps_;
  std::deque<std::atomic<uint8_t>>
      ageResetWhileLocked_;      // Was the cut propagated?
  std::vector<bool> hasSynced_;  // Has the cut been globally synced?
  std::vector<double> rownormalization_;
  std::vector<double> maxabscoef_;
  std::vector<uint8_t> rowintegral;
  std::unordered_multimap<uint64_t, HighsInt> hashToCutMap;
  std::vector<HighsDomain::CutpoolPropagation*> propagationDomains;
  std::set<std::pair<HighsInt, HighsInt>> propRows;

  double bestObservedScore;
  double minScoreFactor;
  double minDensityLim;

  HighsInt agelim_;
  HighsInt softlimit_;
  HighsInt numLpCuts;
  HighsInt numPropNzs;
  HighsInt numPropRows;
  std::vector<HighsInt> ageDistribution;
  std::vector<std::pair<HighsInt, double>> sortBuffer;
  bool hacdcpfRootLedgerScope = false;

 public:
  HighsInt index_;

  HighsCutPool(HighsInt ncols, HighsInt agelim, HighsInt softlimit,
               HighsInt index)
      : matrix_(ncols),
        agelim_(agelim),
        softlimit_(softlimit),
        numLpCuts(0),
        numPropNzs(0),
        numPropRows(0),
        index_(index) {
    ageDistribution.resize(agelim_ + 1);
    minScoreFactor = 0.9;
    bestObservedScore = 0.0;
    minDensityLim = 0.1 * ncols;
  }
  const HighsDynamicRowMatrix& getMatrix() const { return matrix_; }

  const std::vector<double>& getRhs() const { return rhs_; }

  bool isDuplicate(size_t hash, double norm, const HighsInt* Rindex,
                   const double* Rvalue, HighsInt Rlen, double rhs);

  void resetAge(HighsInt cut, bool thread_safe = false) {
    if (ages_[cut] > 0) {
      if (thread_safe) {
        ageResetWhileLocked_[cut].store(1, std::memory_order_relaxed);
        return;
      }
      if (matrix_.columnsLinked(cut)) {
        propRows.erase(std::make_pair(ages_[cut], cut));
        propRows.emplace(0, cut);
      }
      ageDistribution[ages_[cut]] -= 1;
      ageDistribution[0] += 1;
      ages_[cut] = 0;
      ageResetWhileLocked_[cut].store(0, std::memory_order_relaxed);
    }
  }

  double getParallelism(HighsInt row1, HighsInt row2) const;

  double getParallelism(HighsInt row1, HighsInt row2,
                        const HighsCutPool& pool2) const;

  void performAging();

  void lpCutRemoved(HighsInt cut, bool thread_safe = false);

  void addPropagationDomain(HighsDomain::CutpoolPropagation* domain) {
    propagationDomains.push_back(domain);
  }

  void removePropagationDomain(HighsDomain::CutpoolPropagation* domain) {
    for (HighsInt k = propagationDomains.size() - 1; k >= 0; --k) {
      if (propagationDomains[k] == domain) {
        propagationDomains.erase(propagationDomains.begin() + k);
        return;
      }
    }
  }

  void setAgeLimit(HighsInt agelim) {
    agelim_ = agelim;
    ageDistribution.resize(agelim_ + 1);
  }

  void increaseNumLps(HighsInt cut, HighsInt n) {
    assert(numLps_[cut] >= 1);
    numLps_[cut].fetch_add(n, std::memory_order_relaxed);
  };

  void setHacdcpfRootLedgerScope(bool active) {
    hacdcpfRootLedgerScope = active;
  }

  bool getHacdcpfRootLedgerScope() const { return hacdcpfRootLedgerScope; }

  void separate(const std::vector<double>& sol, const HighsDomain& domprop,
                HighsCutSet& cutset, double feastol,
                const std::deque<HighsCutPool>& cutpools,
                bool thread_safe = false,
                const HighsMipSolver* mipsolver = nullptr);

  void separateLpCutsAfterRestart(HighsCutSet& cutset,
                                  const HighsMipSolver* mipsolver = nullptr);

  bool cutIsIntegral(HighsInt cut) const { return (rowintegral[cut] != 0); }

  HighsInt getNumCuts() const {
    return matrix_.getNumRows() - matrix_.getNumDelRows();
  }

  HighsInt getNumAvailableCuts() const { return getNumCuts() - numLpCuts; }

  double getMaxAbsCutCoef(HighsInt cut) const { return maxabscoef_[cut]; }

  double getRowNormalization(HighsInt cut) const {
    return rownormalization_[cut];
  }

  HighsInt addCut(const HighsMipSolver& mipsolver, HighsInt* Rindex,
                  double* Rvalue, HighsInt Rlen, double rhs,
                  bool integral = false, bool propagate = true,
                  bool extractCliques = true, bool isConflict = false);

  HighsInt getRowLength(HighsInt row) const {
    return matrix_.getRowEnd(row) - matrix_.getRowStart(row);
  }

  void getCut(HighsInt cut, HighsInt& cutlen, const HighsInt*& cutinds,
              const double*& cutvals) const {
    HighsInt start = matrix_.getRowStart(cut);
    cutlen = matrix_.getRowEnd(cut) - start;
    cutinds = matrix_.getARindex() + start;
    cutvals = matrix_.getARvalue() + start;
  }

  void syncCutPool(const HighsMipSolver& mipsolver, HighsCutPool& syncpool);
};

#endif
