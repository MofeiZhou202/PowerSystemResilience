/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "mip/HighsSeparation.h"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <queue>

#include "mip/HighsCliqueTable.h"
#include "mip/HighsDomain.h"
#include "mip/HighsImplications.h"
#include "mip/HighsLpAggregator.h"
#include "mip/HighsLpRelaxation.h"
#include "mip/HighsMipSolverData.h"
#include "mip/HighsModkSeparator.h"
#include "mip/HighsPathSeparator.h"
#include "mip/HighsTableauSeparator.h"
#include "mip/HighsTransformedLp.h"

namespace {

void hacdcpfLogHighsSeparationSource(const HighsMipSolverData& mipdata,
                                      const char* phase) {
  const auto& clq = mipdata.cliquetable;
  const auto& impl = mipdata.implications;
  const auto& cs = clq.getSourceConformanceStats();
  const auto& is = impl.getSourceConformanceStats();
  highsLogUser(
      mipdata.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-SEP] phase=%s cutpool=%" HIGHSINT_FORMAT
      " implications=%" HIGHSINT_FORMAT " varbounds=%" HIGHSINT_FORMAT
      " cutRows=%lld mixed=%lld cutVUB=%lld cutVLB=%lld cutClq=%lld/%lld"
      " probe=%lld probeConf=%lld probeRed=%lld vub=%lld/%lld/%lld"
      " vlb=%lld/%lld/%lld\n",
      phase, mipdata.getCutPool().getNumCuts(), impl.getNumImplications(),
      impl.getNumVarBounds(), static_cast<long long>(cs.extract_cut_calls),
      static_cast<long long>(cs.extract_cut_mixed_rows),
      static_cast<long long>(cs.extract_cut_vub_candidates),
      static_cast<long long>(cs.extract_cut_vlb_candidates),
      static_cast<long long>(cs.extract_cut_cliques_added),
      static_cast<long long>(cs.extract_cut_clique_rows),
      static_cast<long long>(is.probing_calls),
      static_cast<long long>(is.probing_conflicts),
      static_cast<long long>(is.probing_reductions),
      static_cast<long long>(is.vub_accepted),
      static_cast<long long>(is.vub_attempts),
      static_cast<long long>(is.vub_replaced),
      static_cast<long long>(is.vlb_accepted),
      static_cast<long long>(is.vlb_attempts),
      static_cast<long long>(is.vlb_replaced));
}

}  // namespace

HighsSeparation::HighsSeparation(HighsMipWorker& mipworker)
    : mipworker_(mipworker) {
  /*
  if (mipworker.mipsolver_.profiling_->mip_) {
    implBoundClock =
        mipworker.mipsolver_.profiling_->getSepaClockIndex(kImplboundSepaString);
    cliqueClock =
        mipworker.mipsolver_.profiling_->getSepaClockIndex(kCliqueSepaString);
  }
  */
  implBoundClock = 990;
  cliqueClock = 991;
  const HighsMipSolver& mipsolver = mipworker.getMipSolver();
  separators.emplace_back(new HighsTableauSeparator(mipsolver));
  separators.emplace_back(new HighsPathSeparator(mipsolver));
  separators.emplace_back(new HighsModkSeparator(mipsolver));
}

HighsInt HighsSeparation::separationRound(HighsDomain& propdomain,
                                          HighsLpRelaxation::Status& status) {
  const HighsSolution& sol = lp->getLpSolver().getSolution();

  HighsMipSolverData& mipdata = *lp->getMipSolver().mipdata_;

  auto propagateAndResolve = [&]() {
    if (propdomain.infeasible() || mipworker_.getGlobalDomain().infeasible()) {
      status = HighsLpRelaxation::Status::kInfeasible;
      propdomain.clearChangedCols();
      return -1;
    }

    propdomain.propagate();
    if (propdomain.infeasible()) {
      status = HighsLpRelaxation::Status::kInfeasible;
      propdomain.clearChangedCols();
      return -1;
    }

    // only modify cliquetable for master worker.
    if (&propdomain == &mipdata.getDomain())
      mipdata.cliquetable.cleanupFixed(mipdata.getDomain());

    if (mipworker_.getGlobalDomain().infeasible()) {
      status = HighsLpRelaxation::Status::kInfeasible;
      propdomain.clearChangedCols();
      return -1;
    }

    int numBoundChgs = (int)propdomain.getChangedCols().size();

    while (!propdomain.getChangedCols().empty()) {
      lp->setObjectiveLimit(mipworker_.upper_limit);
      status = lp->resolveLp(&propdomain);
      if (!lp->scaledOptimal(status)) return -1;

      if (&propdomain == &mipdata.getDomain() &&
          lp->unscaledDualFeasible(status)) {
        mipdata.redcostfixing.addRootRedcost(
            mipdata.mipsolver, lp->getSolution().col_dual, lp->getObjective());
        if (mipworker_.upper_limit != kHighsInf)
          mipdata.redcostfixing.propagateRootRedcost(mipdata.mipsolver);
      }
    }

    return numBoundChgs;
  };

  if (!mipdata.parallelLockActive())
    lp->getMipSolver().profiling_->start(implBoundClock);
  mipdata.implications.separateImpliedBounds(
      *lp, lp->getSolution().col_value, mipworker_.getCutPool(),
      mipdata.feastol, mipworker_.getGlobalDomain(),
      mipdata.parallelLockActive());
  if (!mipdata.parallelLockActive())
    lp->getMipSolver().profiling_->stop(implBoundClock);
  hacdcpfLogHighsSeparationSource(mipdata, "after_implied_bounds");

  HighsInt ncuts = 0;
  HighsInt numboundchgs = propagateAndResolve();
  hacdcpfLogHighsSeparationSource(mipdata, "after_implied_bounds_propagate");
  if (numboundchgs == -1)
    return 0;
  else
    ncuts += numboundchgs;

  if (!mipdata.parallelLockActive())
    lp->getMipSolver().profiling_->start(cliqueClock);
  mipdata.cliquetable.separateCliques(
      lp->getMipSolver(), sol.col_value, mipworker_.getCutPool(),
      mipdata.feastol,
      mipdata.parallelLockActive() ? mipworker_.randgen
                                   : mipdata.cliquetable.getRandgen(),
      mipdata.parallelLockActive()
          ? mipworker_.getNumNeighbourhoodQueries()
          : mipdata.cliquetable.getNumNeighbourhoodQueries());
  if (!mipdata.parallelLockActive())
    lp->getMipSolver().profiling_->stop(cliqueClock);
  hacdcpfLogHighsSeparationSource(mipdata, "after_clique_separator");

  numboundchgs = propagateAndResolve();
  hacdcpfLogHighsSeparationSource(mipdata, "after_clique_propagate");
  if (numboundchgs == -1)
    return 0;
  else
    ncuts += numboundchgs;

  if (&propdomain != &mipworker_.getGlobalDomain())
    lp->computeBasicDegenerateDuals(
        mipdata.feastol, propdomain, mipworker_.getGlobalDomain(),
        mipworker_.getConflictPool(), mipworker_.getPseudocost(), true);

  HighsTransformedLp transLp(*lp, mipdata.implications,
                             mipworker_.getGlobalDomain());
  if (mipworker_.getGlobalDomain().infeasible()) {
    status = HighsLpRelaxation::Status::kInfeasible;
    return 0;
  }
  HighsLpAggregator lpAggregator(*lp);

  HighsInt separatorIndex = 0;
  for (const std::unique_ptr<HighsSeparator>& separator : separators) {
    separator->run(*lp, lpAggregator, transLp, mipworker_.getCutPool());
    if (separatorIndex == 0)
      hacdcpfLogHighsSeparationSource(mipdata, "after_tableau_separator");
    else if (separatorIndex == 1)
      hacdcpfLogHighsSeparationSource(mipdata, "after_path_separator");
    else
      hacdcpfLogHighsSeparationSource(mipdata, "after_modk_separator");
    ++separatorIndex;
    if (mipworker_.getGlobalDomain().infeasible()) {
      status = HighsLpRelaxation::Status::kInfeasible;
      return 0;
    }
  }

  numboundchgs = propagateAndResolve();
  hacdcpfLogHighsSeparationSource(mipdata, "after_separator_propagate");
  if (numboundchgs == -1)
    return 0;
  else
    ncuts += numboundchgs;

  const HighsInt hacdcpfCutpoolBefore = mipworker_.getCutPool().getNumCuts();
  mipworker_.getCutPool().separate(sol.col_value, propdomain, cutset,
                                   mipdata.feastol, mipdata.cutpools, false,
                                   &mipdata.mipsolver);
  // Also separate the global cut pool
  if (&mipworker_.getCutPool() != &mipdata.getCutPool()) {
    mipdata.getCutPool().separate(sol.col_value, propdomain, cutset,
                                  mipdata.feastol, mipdata.cutpools, true);
  }
  if (std::getenv("HACDCPF_HIGHS_CUTPOOL_TRACE") != nullptr) {
    highsLogUser(
        mipdata.mipsolver.options_mip_->log_options, HighsLogType::kInfo,
        "[HIGHS-CUTPOOL-ROUND] pool_before=%lld pool_after=%lld "
        "selected=%lld ncuts_before=%lld\n",
        static_cast<long long>(hacdcpfCutpoolBefore),
        static_cast<long long>(mipworker_.getCutPool().getNumCuts()),
        static_cast<long long>(cutset.numCuts()), static_cast<long long>(ncuts));
  }
  hacdcpfLogHighsSeparationSource(mipdata, "after_cutpool_separate");

  if (cutset.numCuts() > 0) {
    ncuts += cutset.numCuts();
    lp->addCuts(cutset);
    status = lp->resolveLp(&propdomain);
    lp->performAging(true);

    // only for the master domain.
    if (&propdomain == &mipdata.getDomain() &&
        lp->unscaledDualFeasible(status)) {
      mipdata.redcostfixing.addRootRedcost(
          mipdata.mipsolver, lp->getSolution().col_dual, lp->getObjective());
      if (mipdata.upper_limit != kHighsInf)
        mipdata.redcostfixing.propagateRootRedcost(mipdata.mipsolver);
    }
    hacdcpfLogHighsSeparationSource(mipdata, "after_cutpool_resolve");
  }

  return ncuts;
}

void HighsSeparation::separate(HighsDomain& propdomain) {
  HighsLpRelaxation::Status status = lp->getStatus();
  const HighsMipSolver& mipsolver = lp->getMipSolver();

  if (lp->scaledOptimal(status) && !lp->getFractionalIntegers().empty()) {
    // double firstobj = lp->getObjective();
    double firstobj = mipsolver.mipdata_->rootlpsolobj;

    while (lp->getObjective() < mipworker_.optimality_limit) {
      double lastobj = lp->getObjective();

      int64_t nlpiters = -lp->getNumLpIterations();
      HighsInt ncuts = separationRound(propdomain, status);
      nlpiters += lp->getNumLpIterations();

      if (mipsolver.mipdata_->parallelLockActive()) {
        mipworker_.getSepaLpIterations() += nlpiters;
      } else {
        mipsolver.mipdata_->sepa_lp_iterations += nlpiters;
        mipsolver.mipdata_->total_lp_iterations += nlpiters;
      }

      // printf("separated %" HIGHSINT_FORMAT " cuts\n", ncuts);

      // printf(
      //     "separation round %" HIGHSINT_FORMAT " at node %" HIGHSINT_FORMAT "
      //     added %" HIGHSINT_FORMAT " cuts objective changed " "from %g to %g,
      //     first obj is %g\n", nrounds, (HighsInt)nnodes, ncuts, lastobj,
      //     lp->getObjective(), firstobj);
      if (ncuts == 0 || !lp->scaledOptimal(status) ||
          lp->getFractionalIntegers().empty())
        break;

      // if the objective improved considerably we continue
      if ((lp->getObjective() - firstobj) <=
          std::max((lastobj - firstobj), mipsolver.mipdata_->feastol) * 1.01)
        break;
    }

    // printf("done separating\n");
  } else {
    // printf("no separation, just aging. status: %" HIGHSINT_FORMAT "\n",
    //        (HighsInt)status);
    lp->performAging(true);

    mipworker_.getCutPool().performAging();
  }
}
