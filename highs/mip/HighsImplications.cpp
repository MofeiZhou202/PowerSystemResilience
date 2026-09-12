/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "mip/HighsImplications.h"

#include "../extern/pdqsort/pdqsort.h"
#include "mip/HighsCliqueTable.h"
#include "mip/HighsMipSolverData.h"
#include "mip/MipTimer.h"

#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace {

std::uint64_t hacdcpfImpTraceHashMix(std::uint64_t seed,
                                     std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

std::uint64_t hacdcpfImpTraceHashDouble(double value) {
  if (std::isinf(value))
    return value > 0.0 ? 0x7ff0000000000000ULL : 0xfff0000000000000ULL;
  if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
  return static_cast<std::uint64_t>(
      static_cast<std::int64_t>(std::llround(value * 1e9)));
}

int hacdcpfImpTraceInt(const char* name) {
  const char* env = std::getenv(name);
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<int>(value);
}

}  // namespace

bool HighsImplications::computeImplications(HighsInt col, bool val) {
  const int trace_col = hacdcpfImpTraceInt("HIGHS_IMPLICATION_TRACE_COL");
  const int trace_target = hacdcpfImpTraceInt("HIGHS_IMPLICATION_TRACE_TARGET");
  if (col == trace_col) {
    std::fprintf(stderr,
                 "[HIGHS-IMPL-COMPUTE] col=%lld val=%d start\n",
                 static_cast<long long>(col), val ? 1 : 0);
  }
  ++sourceConformanceStats.compute_calls;
  HighsDomain& globaldomain = mipsolver.mipdata_->getDomain();
  HighsCliqueTable& cliquetable = mipsolver.mipdata_->cliquetable;
  globaldomain.propagate();
  if (globaldomain.infeasible() || globaldomain.isFixed(col)) return true;

  // record redundant rows for lifting
  assert(globaldomain.getRedundantRows().size() == 0);
  if (storeLiftingOpportunity != nullptr)
    globaldomain.setRecordRedundantRows(true);

  const auto& domchgstack = globaldomain.getDomainChangeStack();
  const auto& domchgreason = globaldomain.getDomainChangeReason();
  size_t changedend = globaldomain.getChangedCols().size();

  HighsInt stackimplicstart = domchgstack.size() + 1;
  HighsInt numImplications = -stackimplicstart;
  if (val)
    globaldomain.changeBound(HighsBoundType::kLower, col, 1);
  else
    globaldomain.changeBound(HighsBoundType::kUpper, col, 0);

  auto storeLiftingOpportunities = [&](HighsInt col, bool val) {
    // use callback to store lifting opportunities
    if (storeLiftingOpportunity != nullptr) {
      for (const auto& elm : globaldomain.getRedundantRows())
        storeLiftingOpportunity(
            elm.key(), col, val ? 1 : 0,
            (val ? -1 : 1) * globaldomain.getRedundantRowValue(elm.key()));
      globaldomain.clearRedundantRows();
      globaldomain.setRecordRedundantRows(false);
    }
  };

  auto doBacktrack = [&](size_t changedend) {
    globaldomain.backtrack();
    globaldomain.clearChangedCols(changedend);
  };

  auto isInfeasible = [&](HighsInt col, bool val) {
    if (!globaldomain.infeasible()) return false;
    ++sourceConformanceStats.computed_infeasible;
    storeLiftingOpportunities(col, val);
    doBacktrack(changedend);
    cliquetable.vertexInfeasible(globaldomain, col, val);
    return true;
  };

  if (isInfeasible(col, val)) return true;

  globaldomain.propagate();

  if (isInfeasible(col, val)) return true;

  HighsInt stackimplicend = domchgstack.size();
  numImplications += stackimplicend;
  mipsolver.mipdata_->getPseudoCost().addInferenceObservation(
      col, numImplications, val);

  std::vector<HighsDomainChange> implics;
  implics.reserve(numImplications);

  HighsInt numEntries = mipsolver.mipdata_->cliquetable.getNumEntries();
  HighsInt maxEntries = 100000 + mipsolver.numNonzero();

  for (HighsInt i = stackimplicstart; i < stackimplicend; ++i) {
    if (domchgreason[i].type == HighsDomain::Reason::kCliqueTable &&
        ((domchgreason[i].index >> 1) == col || numEntries >= maxEntries))
      continue;

    implics.push_back(domchgstack[i]);
  }

  // inform caller about lifting opportunities
  storeLiftingOpportunities(col, val);

  // backtrack
  doBacktrack(changedend);

  // add the implications of binary variables to the clique table
  auto binstart = std::partition(implics.begin(), implics.end(),
                                 [&](const HighsDomainChange& a) {
                                   return !globaldomain.isBinary(a.column);
                                 });

  pdqsort(implics.begin(), binstart);

  std::array<HighsCliqueTable::CliqueVar, 2> clique;
  clique[0] = HighsCliqueTable::CliqueVar(col, val);

  for (auto i = binstart; i != implics.end(); ++i) {
    if (i->boundtype == HighsBoundType::kLower)
      clique[1] = HighsCliqueTable::CliqueVar(i->column, 0);
    else
      clique[1] = HighsCliqueTable::CliqueVar(i->column, 1);

    cliquetable.addClique(mipsolver, clique.data(), 2);
    ++sourceConformanceStats.binary_implication_cliques;
    if (globaldomain.infeasible() || globaldomain.isFixed(col)) return true;
  }

  // store variable bounds derived from implications
  for (auto i = implics.begin(); i != binstart; ++i) {
    if ((col == trace_col || i->column == trace_target) &&
        (trace_col >= 0 || trace_target >= 0)) {
      std::fprintf(stderr,
                   "[HIGHS-IMPL-BOUND] trigger=%lld val=%d target=%lld "
                   "type=%s value=%.17g lb=%.17g ub=%.17g\n",
                   static_cast<long long>(col), val ? 1 : 0,
                   static_cast<long long>(i->column),
                   i->boundtype == HighsBoundType::kLower ? "lb" : "ub",
                   i->boundval, globaldomain.col_lower_[i->column],
                   globaldomain.col_upper_[i->column]);
    }
    if (i->boundtype == HighsBoundType::kLower) {
      if (val == 1) {
        if (globaldomain.col_lower_[i->column] != -kHighsInf)
          addVLB(i->column, col,
                 i->boundval - globaldomain.col_lower_[i->column],
                 globaldomain.col_lower_[i->column]);
      } else
        addVLB(i->column,
               col,  // in case the lower bound is infinite the varbound can
                     // still be tightened as soon as a finite upper bound is
                     // known because the offset is finite
               globaldomain.col_lower_[i->column] - i->boundval, i->boundval);
    } else {
      if (val == 1) {
        if (globaldomain.col_upper_[i->column] != kHighsInf)
          addVUB(i->column, col,
                 i->boundval - globaldomain.col_upper_[i->column],
                 globaldomain.col_upper_[i->column]);
      } else
        addVUB(i->column,
               col,  // in case the upper bound is infinite the varbound can
                     // still be tightened as soon as a finite upper bound is
                     // known because the offset is finite
               globaldomain.col_upper_[i->column] - i->boundval, i->boundval);
    }
  }

  HighsInt loc = 2 * col + val;
  implications[loc].computed = true;
  implics.erase(binstart, implics.end());
  if (!implics.empty()) {
    implications[loc].implics = std::move(implics);
    this->numImplications += implications[loc].implics.size();
  }

  if (col == trace_col) {
    std::fprintf(stderr,
                 "[HIGHS-IMPL-COMPUTE] col=%lld val=%d done kept=%lld\n",
                 static_cast<long long>(col), val ? 1 : 0,
                 static_cast<long long>(implications[loc].implics.size()));
  }

  return false;
}

static constexpr bool kSkipBadVbds = true;
static constexpr bool kUseDualsForBreakingTies = true;

std::pair<HighsInt, HighsImplications::VarBound> HighsImplications::getBestVub(
    HighsInt col, const HighsSolution& lpSolution, double& bestUb,
    const HighsDomain& globaldom) const {
  const int trace_col = hacdcpfImpTraceInt("HIGHS_VB_TRACE_COL");
  const int trace_trigger = hacdcpfImpTraceInt("HIGHS_VB_TRACE_TRIGGER");
  std::pair<HighsInt, VarBound> bestVub =
      std::make_pair(-1, VarBound{0.0, kHighsInf});
  double minbestUb = bestUb;
  double bestUbDist = kHighsInf;
  int64_t bestvubnodes = 0;

  auto isVubBetter = [&](double ubDist, int64_t vubNodes, double minVubVal,
                         HighsInt vubCol, const VarBound& vub) {
    if (ubDist < bestUbDist - mipsolver.mipdata_->feastol) return true;
    if (vubNodes > bestvubnodes) return true;
    if (vubNodes < bestvubnodes) return false;
    if (minVubVal < minbestUb - mipsolver.mipdata_->feastol) return true;
    if (kUseDualsForBreakingTies) {
      if (minVubVal > minbestUb + mipsolver.mipdata_->feastol) return false;
      if (lpSolution.col_dual[vubCol] / vub.coef -
              lpSolution.col_dual[bestVub.first] / bestVub.second.coef >
          mipsolver.mipdata_->feastol)
        return true;
    }

    return false;
  };

  double scale = globaldom.col_upper_[col] - globaldom.col_lower_[col];
  if (scale == kHighsInf)
    scale = 1.0;
  else
    scale = 1.0 / scale;

  vubs[col].for_each([&](HighsInt vubCol, const VarBound& vub) {
    if (vub.coef == kHighsInf) return;
    if (globaldom.isFixed(vubCol)) return;
    assert(globaldom.isBinary(vubCol));
    double vubval = lpSolution.col_value[vubCol] * vub.coef + vub.constant;
    double ubDist = std::max(0.0, vubval - lpSolution.col_value[col]);

    double yDist = mipsolver.mipdata_->feastol +
                   (vub.coef > 0 ? 1 - lpSolution.col_value[vubCol]
                                 : lpSolution.col_value[vubCol]);
    // skip variable bound if the distance towards the variable bound constraint
    // is larger than the distance to the point where the binary column is
    // relaxed to it's weakest bound, i.e. 1 if its coefficient is positive.
    // The variable bound constraint has the form x <= ay + b with y binary and
    // hence the norm is sqrt(1 + a^2) and the distance of the variable bound
    // constraint is ay + b - x evaluated at the solution values of x and y
    // divided by the norm.
    double norm2 = 1.0 + vub.coef * vub.coef;
    if (kSkipBadVbds && ubDist * ubDist > yDist * yDist * norm2) return;

    if (static_cast<int>(col) == trace_col &&
        (trace_trigger < 0 || static_cast<int>(vubCol) == trace_trigger)) {
      std::fprintf(stderr,
                   "[HIGHS-VB-BEST-CAND] target=%lld trigger=%lld coef=%.17g "
                   "constant=%.17g vubval=%.17g ubDist=%.17g yDist=%.17g "
                   "norm2=%.17g min=%.17g solTarget=%.17g solTrigger=%.17g\n",
                   static_cast<long long>(col), static_cast<long long>(vubCol),
                   vub.coef, vub.constant, vubval, ubDist, yDist, norm2,
                   vub.minValue(), lpSolution.col_value[col],
                   lpSolution.col_value[vubCol]);
    }

    assert(vubCol >= 0 && vubCol < mipsolver.numCol());
    ubDist *= scale;
    if (ubDist <= bestUbDist + mipsolver.mipdata_->feastol) {
      double minvubval = vub.minValue();
      int64_t vubnodes =
          vub.coef > 0 ? mipsolver.mipdata_->nodequeue.numNodesDown(vubCol)
                       : mipsolver.mipdata_->nodequeue.numNodesUp(vubCol);

      if (isVubBetter(ubDist, vubnodes, minvubval, vubCol, vub)) {
        bestUb = vubval;
        minbestUb = minvubval;
        bestVub = std::make_pair(vubCol, vub);
        bestvubnodes = vubnodes;
        bestUbDist = ubDist;
      }
    }
  });

  if (static_cast<int>(col) == trace_col) {
    std::fprintf(stderr,
                 "[HIGHS-VB-BEST-FINAL] target=%lld trigger=%lld coef=%.17g "
                 "constant=%.17g bestUb=%.17g\n",
                 static_cast<long long>(col),
                 static_cast<long long>(bestVub.first), bestVub.second.coef,
                 bestVub.second.constant, bestUb);
  }

  return bestVub;
}

std::pair<HighsInt, HighsImplications::VarBound> HighsImplications::getBestVlb(
    HighsInt col, const HighsSolution& lpSolution, double& bestLb,
    const HighsDomain& globaldom) const {
  std::pair<HighsInt, VarBound> bestVlb =
      std::make_pair(-1, VarBound{0.0, -kHighsInf});
  double maxbestlb = bestLb;
  double bestLbDist = kHighsInf;
  int64_t bestvlbnodes = 0;

  auto isVlbBetter = [&](double lbDist, int64_t vlbNodes, double maxVlbVal,
                         HighsInt vlbCol, const VarBound& vlb) {
    if (lbDist < bestLbDist - mipsolver.mipdata_->feastol) return true;
    if (vlbNodes > bestvlbnodes) return true;
    if (vlbNodes < bestvlbnodes) return false;
    if (maxVlbVal > maxbestlb + mipsolver.mipdata_->feastol) return true;
    if (kUseDualsForBreakingTies) {
      if (maxVlbVal < maxbestlb - mipsolver.mipdata_->feastol) return false;
      if (lpSolution.col_dual[vlbCol] / vlb.coef -
              lpSolution.col_dual[bestVlb.first] / bestVlb.second.coef <
          -mipsolver.mipdata_->feastol)
        return true;
    }

    return false;
  };

  double scale = globaldom.col_upper_[col] - globaldom.col_lower_[col];
  if (scale == kHighsInf)
    scale = 1.0;
  else
    scale = 1.0 / scale;

  vlbs[col].for_each([&](HighsInt vlbCol, const VarBound& vlb) {
    if (vlb.coef == -kHighsInf) return;
    if (globaldom.isFixed(vlbCol)) return;
    assert(globaldom.isBinary(vlbCol));
    assert(vlbCol >= 0 && vlbCol < mipsolver.numCol());
    double vlbval = lpSolution.col_value[vlbCol] * vlb.coef + vlb.constant;
    double lbDist = std::max(0.0, lpSolution.col_value[col] - vlbval);

    double yDist = mipsolver.mipdata_->feastol +
                   (vlb.coef > 0 ? lpSolution.col_value[vlbCol]
                                 : 1 - lpSolution.col_value[vlbCol]);

    double norm2 = 1.0 + vlb.coef * vlb.coef;
    if (kSkipBadVbds && lbDist * lbDist > yDist * yDist * norm2) return;

    // scale the distance as if the bounded column was scaled to have ub-lb=1
    lbDist *= scale;
    if (lbDist <= bestLbDist + mipsolver.mipdata_->feastol) {
      double maxvlbval = vlb.maxValue();
      int64_t vlbnodes =
          vlb.coef > 0 ? mipsolver.mipdata_->nodequeue.numNodesUp(vlbCol)
                       : mipsolver.mipdata_->nodequeue.numNodesDown(vlbCol);

      if (isVlbBetter(lbDist, vlbnodes, maxvlbval, vlbCol, vlb)) {
        bestLb = vlbval;
        maxbestlb = maxvlbval;
        bestVlb = std::make_pair(vlbCol, vlb);
        bestvlbnodes = vlbnodes;
        bestLbDist = lbDist;
      }
    }
  });

  return bestVlb;
}

bool HighsImplications::runProbing(HighsInt col, HighsInt& numReductions) {
  HighsDomain& globaldomain = mipsolver.mipdata_->getDomain();
  if (globaldomain.isBinary(col) && !implicationsCached(col, 1) &&
      !implicationsCached(col, 0) &&
      mipsolver.mipdata_->cliquetable.getSubstitution(col) == nullptr) {
    ++sourceConformanceStats.probing_calls;
    const HighsInt reductionsStart = numReductions;
    const size_t substitutionsStart = substitutions.size();
    bool infeasible = computeImplications(col, 1);
    if (globaldomain.infeasible()) return true;
    if (infeasible) {
      ++sourceConformanceStats.probing_conflicts;
      return true;
    }
    if (mipsolver.mipdata_->cliquetable.getSubstitution(col) != nullptr)
      return true;

    infeasible = computeImplications(col, 0);
    if (globaldomain.infeasible()) return true;
    if (infeasible) {
      ++sourceConformanceStats.probing_conflicts;
      return true;
    }
    if (mipsolver.mipdata_->cliquetable.getSubstitution(col) != nullptr)
      return true;

    // analyze implications
    const std::vector<HighsDomainChange>& implicsdown =
        getImplications(col, 0, infeasible);
    const std::vector<HighsDomainChange>& implicsup =
        getImplications(col, 1, infeasible);
    HighsInt nimplicsdown = implicsdown.size();
    HighsInt nimplicsup = implicsup.size();
    HighsInt u = 0;
    HighsInt d = 0;

    while (u < nimplicsup && d < nimplicsdown) {
      if (implicsup[u].column < implicsdown[d].column)
        ++u;
      else if (implicsdown[d].column < implicsup[u].column)
        ++d;
      else {
        assert(implicsup[u].column == implicsdown[d].column);
        HighsInt implcol = implicsup[u].column;
        double lbDown = globaldomain.col_lower_[implcol];
        double ubDown = globaldomain.col_upper_[implcol];
        double lbUp = lbDown;
        double ubUp = ubDown;

        do {
          if (implicsdown[d].boundtype == HighsBoundType::kLower)
            lbDown = std::max(lbDown, implicsdown[d].boundval);
          else
            ubDown = std::min(ubDown, implicsdown[d].boundval);
          ++d;
        } while (d < nimplicsdown && implicsdown[d].column == implcol);

        do {
          if (implicsup[u].boundtype == HighsBoundType::kLower)
            lbUp = std::max(lbUp, implicsup[u].boundval);
          else
            ubUp = std::min(ubUp, implicsup[u].boundval);
          ++u;
        } while (u < nimplicsup && implicsup[u].column == implcol);

        if (colsubstituted[implcol] || globaldomain.isFixed(implcol)) continue;

        if (lbDown == ubDown && lbUp == ubUp &&
            std::abs(lbDown - lbUp) > mipsolver.mipdata_->feastol) {
          HighsSubstitution substitution;
          substitution.substcol = implcol;
          substitution.staycol = col;
          substitution.offset = lbDown;
          substitution.scale = lbUp - lbDown;
          substitutions.push_back(substitution);
          colsubstituted[implcol] = true;
          ++numReductions;
        } else if (!mipsolver.mipdata_->parallelLockActive()) {
          double lb = std::min(lbDown, lbUp);
          double ub = std::max(ubDown, ubUp);

          if (lb > globaldomain.col_lower_[implcol]) {
            globaldomain.changeBound(HighsBoundType::kLower, implcol, lb,
                                     HighsDomain::Reason::unspecified());
            ++numReductions;
          }

          if (ub < globaldomain.col_upper_[implcol]) {
            globaldomain.changeBound(HighsBoundType::kUpper, implcol, ub,
                                     HighsDomain::Reason::unspecified());
            ++numReductions;
          }
        }
      }
    }

    sourceConformanceStats.probing_reductions +=
        numReductions - reductionsStart;
    sourceConformanceStats.probing_substitutions +=
        static_cast<int64_t>(substitutions.size() - substitutionsStart);
    return true;
  }

  return false;
}

void HighsImplications::logConformanceVarBounds(const char* phase,
                                                HighsInt max_terms) const {
  if (max_terms <= 0) max_terms = 32;
  std::uint64_t vub_hash = 0x4849474853564255ULL;
  std::uint64_t vlb_hash = 0x484947485356424cULL;
  int64_t vub_count = 0;
  int64_t vlb_count = 0;
  std::ostringstream vub_sample;
  std::ostringstream vlb_sample;
  vub_sample << std::setprecision(12);
  vlb_sample << std::setprecision(12);
  HighsInt vub_emit = 0;
  HighsInt vlb_emit = 0;

  for (HighsInt col = 0; col < mipsolver.numCol(); ++col) {
    vubs[col].for_each([&](HighsInt vubCol, const VarBound& vub) {
      ++vub_count;
      vub_hash =
          hacdcpfImpTraceHashMix(vub_hash, static_cast<std::uint64_t>(col));
      vub_hash = hacdcpfImpTraceHashMix(
          vub_hash, static_cast<std::uint64_t>(vubCol));
      vub_hash =
          hacdcpfImpTraceHashMix(vub_hash, hacdcpfImpTraceHashDouble(vub.coef));
      vub_hash = hacdcpfImpTraceHashMix(
          vub_hash, hacdcpfImpTraceHashDouble(vub.constant));
      if (vub_emit < max_terms) {
        if (vub_emit++ > 0) vub_sample << ";";
        vub_sample << col << "<= " << vub.coef << "*" << vubCol << " + "
                   << vub.constant << ":min" << vub.minValue();
      }
    });
    vlbs[col].for_each([&](HighsInt vlbCol, const VarBound& vlb) {
      ++vlb_count;
      vlb_hash =
          hacdcpfImpTraceHashMix(vlb_hash, static_cast<std::uint64_t>(col));
      vlb_hash = hacdcpfImpTraceHashMix(
          vlb_hash, static_cast<std::uint64_t>(vlbCol));
      vlb_hash =
          hacdcpfImpTraceHashMix(vlb_hash, hacdcpfImpTraceHashDouble(vlb.coef));
      vlb_hash = hacdcpfImpTraceHashMix(
          vlb_hash, hacdcpfImpTraceHashDouble(vlb.constant));
      if (vlb_emit < max_terms) {
        if (vlb_emit++ > 0) vlb_sample << ";";
        vlb_sample << col << ">= " << vlb.coef << "*" << vlbCol << " + "
                   << vlb.constant << ":max" << vlb.maxValue();
      }
    });
  }

  highsLogUser(
      mipsolver.options_mip_->log_options, HighsLogType::kInfo,
      "[HIGHS-VBSTATE] phase=%s vub=%lld:%016llx vlb=%lld:%016llx "
      "total=%lld stats=vub%lld/%lld/%lld:vlb%lld/%lld/%lld "
      "sample_vub=[%s] sample_vlb=[%s]\n",
      phase != nullptr ? phase : "varbounds",
      static_cast<long long>(vub_count),
      static_cast<unsigned long long>(vub_hash),
      static_cast<long long>(vlb_count),
      static_cast<unsigned long long>(vlb_hash),
      static_cast<long long>(numVarBounds),
      static_cast<long long>(sourceConformanceStats.vub_accepted),
      static_cast<long long>(sourceConformanceStats.vub_attempts),
      static_cast<long long>(sourceConformanceStats.vub_replaced),
      static_cast<long long>(sourceConformanceStats.vlb_accepted),
      static_cast<long long>(sourceConformanceStats.vlb_attempts),
      static_cast<long long>(sourceConformanceStats.vlb_replaced),
      vub_sample.str().c_str(), vlb_sample.str().c_str());
}

void HighsImplications::appendPresolveVarBoundSnapshot(
    HighsPresolveSideState& state, HighsInt max_records) const {
  if (max_records < 0) max_records = 0;
  state.vub_attempts = sourceConformanceStats.vub_attempts;
  state.vub_accepted = sourceConformanceStats.vub_accepted;
  state.vub_replaced = sourceConformanceStats.vub_replaced;
  state.vlb_attempts = sourceConformanceStats.vlb_attempts;
  state.vlb_accepted = sourceConformanceStats.vlb_accepted;
  state.vlb_replaced = sourceConformanceStats.vlb_replaced;
  state.probing_calls = sourceConformanceStats.probing_calls;
  state.probing_conflicts = sourceConformanceStats.probing_conflicts;
  state.probing_reductions = sourceConformanceStats.probing_reductions;
  state.probing_substitutions = sourceConformanceStats.probing_substitutions;
  state.vub_hash = 0x4849474853564255ULL;
  state.vlb_hash = 0x484947485356424cULL;
  state.vub_count = 0;
  state.vlb_count = 0;
  state.var_bounds.clear();
  if (max_records > 0) {
    const HighsInt reserve_count =
        std::min<HighsInt>(max_records, static_cast<HighsInt>(numVarBounds));
    state.var_bounds.reserve(static_cast<std::size_t>(reserve_count));
  }

  auto maybeAppend = [&](HighsInt target, HighsInt trigger,
                         const VarBound& bound, bool upper) {
    if (static_cast<HighsInt>(state.var_bounds.size()) >= max_records) return;
    HighsPresolveVarBoundRecord rec;
    rec.target_col = target;
    rec.trigger_col = trigger;
    rec.target_orig_col = mipsolver.mipdata_->postSolveStack.getOrigColIndex(target);
    rec.trigger_orig_col =
        mipsolver.mipdata_->postSolveStack.getOrigColIndex(trigger);
    rec.coef = bound.coef;
    rec.constant = bound.constant;
    rec.target_scale =
        mipsolver.mipdata_->postSolveStack.getColLinearTransformScale(target);
    rec.target_constant =
        mipsolver.mipdata_->postSolveStack.getColLinearTransformConstant(target);
    rec.trigger_scale =
        mipsolver.mipdata_->postSolveStack.getColLinearTransformScale(trigger);
    rec.trigger_constant =
        mipsolver.mipdata_->postSolveStack.getColLinearTransformConstant(trigger);
    rec.upper = upper;
    rec.target_linearly_transformable =
        mipsolver.mipdata_->postSolveStack.isColLinearlyTransformable(target);
    rec.trigger_linearly_transformable =
        mipsolver.mipdata_->postSolveStack.isColLinearlyTransformable(trigger);
    state.var_bounds.push_back(rec);
  };

  for (HighsInt col = 0; col < mipsolver.numCol(); ++col) {
    vubs[col].for_each([&](HighsInt vubCol, const VarBound& vub) {
      ++state.vub_count;
      state.vub_hash =
          hacdcpfImpTraceHashMix(state.vub_hash, static_cast<std::uint64_t>(col));
      state.vub_hash = hacdcpfImpTraceHashMix(
          state.vub_hash, static_cast<std::uint64_t>(vubCol));
      state.vub_hash = hacdcpfImpTraceHashMix(
          state.vub_hash, hacdcpfImpTraceHashDouble(vub.coef));
      state.vub_hash = hacdcpfImpTraceHashMix(
          state.vub_hash, hacdcpfImpTraceHashDouble(vub.constant));
      maybeAppend(col, vubCol, vub, true);
    });
    vlbs[col].for_each([&](HighsInt vlbCol, const VarBound& vlb) {
      ++state.vlb_count;
      state.vlb_hash =
          hacdcpfImpTraceHashMix(state.vlb_hash, static_cast<std::uint64_t>(col));
      state.vlb_hash = hacdcpfImpTraceHashMix(
          state.vlb_hash, static_cast<std::uint64_t>(vlbCol));
      state.vlb_hash = hacdcpfImpTraceHashMix(
          state.vlb_hash, hacdcpfImpTraceHashDouble(vlb.coef));
      state.vlb_hash = hacdcpfImpTraceHashMix(
          state.vlb_hash, hacdcpfImpTraceHashDouble(vlb.constant));
      maybeAppend(col, vlbCol, vlb, false);
    });
  }
}

void HighsImplications::strengthenVarBound(VarBound& vbnd,
                                           HighsInt multiplier) const {
  // try to strengthen variable bound constraint x + a * y <= b by computing
  // MIR cut. since x is a general-integer variable (with integral bounds) and
  // its coefficient is 1.0, it is not shifted (or complemented). similarly,
  // since y is a binary variable (i.e., its lower bound is 0.0), it does not
  // need to be shifted, and we also do not try to complement it.
  if (std::abs(vbnd.coef) == kHighsInf || std::abs(vbnd.constant) == kHighsInf)
    return;
  constexpr double f0min = 0.005;
  constexpr double f0max = 0.995;
  double downrhs = std::floor(multiplier * vbnd.constant);
  double f0 = multiplier * vbnd.constant - downrhs;
  if (f0 < f0min || f0 > f0max) return;
  double downaj = std::floor(-multiplier * vbnd.coef + kHighsTiny);
  double fj = -multiplier * vbnd.coef - downaj;
  vbnd.constant = multiplier * downrhs;
  vbnd.coef = -multiplier * (downaj + std::max(fj - f0, 0.0) / (1.0 - f0));
};

void HighsImplications::addVUB(HighsInt col, HighsInt vubcol, double vubcoef,
                               double vubconstant) {
  addVUB(col, vubcol, vubcoef, vubconstant,
         mipsolver.mipdata_->getDomain().col_upper_[col],
         mipsolver.isColIntegral(col));
}

void HighsImplications::addVUB(HighsInt col, HighsInt vubcol, double vubcoef,
                               double vubconstant, double colupperbound,
                               bool colisintegral) {
  const int trace_col = hacdcpfImpTraceInt("HIGHS_VB_TRACE_COL");
  const int trace_trigger = hacdcpfImpTraceInt("HIGHS_VB_TRACE_TRIGGER");
  const bool trace = static_cast<int>(col) == trace_col &&
                     (trace_trigger < 0 || static_cast<int>(vubcol) == trace_trigger);
  if (trace) {
    std::fprintf(stderr,
                 "[HIGHS-VB-ADD] target=%lld trigger=%lld phase=attempt "
                 "coef=%.17g constant=%.17g colUb=%.17g integral=%d\n",
                 static_cast<long long>(col), static_cast<long long>(vubcol),
                 vubcoef, vubconstant, colupperbound, colisintegral ? 1 : 0);
  }
  ++sourceConformanceStats.vub_attempts;
  // assume that VUBs do not have infinite coefficients and infinite constant
  // terms since such VUBs effectively evaluate to NaN.
  assert(std::abs(vubcoef) != kHighsInf || std::abs(vubconstant) != kHighsInf);
  if (tooManyVarBounds()) return;

  VarBound vub{vubcoef, vubconstant};

  if (colisintegral) {
    // try to strengthen VUB
    strengthenVarBound(vub, HighsInt{1});
    if (vub.coef == 0.0) {
      if (trace) {
        std::fprintf(stderr,
                     "[HIGHS-VB-ADD] target=%lld trigger=%lld phase=reject "
                     "reason=strengthened_zero\n",
                     static_cast<long long>(col),
                     static_cast<long long>(vubcol));
      }
      return;
    }
  }

  mipsolver.mipdata_->debugSolution.checkVub(col, vubcol, vubcoef, vubconstant);

  double minBound = vub.minValue();
  if (minBound >= colupperbound - mipsolver.mipdata_->feastol) {
    if (trace) {
      std::fprintf(stderr,
                   "[HIGHS-VB-ADD] target=%lld trigger=%lld phase=reject "
                   "reason=redundant min=%.17g colUb=%.17g\n",
                   static_cast<long long>(col), static_cast<long long>(vubcol),
                   minBound, colupperbound);
    }
    return;
  }

  auto insertresult = vubs[col].insert_or_get(vubcol, vub);

  if (!insertresult.second) {
    VarBound& currentvub = *insertresult.first;
    double currentMinBound = currentvub.minValue();
    if (minBound < currentMinBound - mipsolver.mipdata_->feastol) {
      currentvub.coef = vub.coef;
      currentvub.constant = vub.constant;
      ++sourceConformanceStats.vub_replaced;
      if (trace) {
        std::fprintf(stderr,
                     "[HIGHS-VB-ADD] target=%lld trigger=%lld phase=replace "
                     "coef=%.17g constant=%.17g min=%.17g oldMin=%.17g\n",
                     static_cast<long long>(col),
                     static_cast<long long>(vubcol), vub.coef, vub.constant,
                     minBound, currentMinBound);
      }
    }
  } else {
    ++sourceConformanceStats.vub_accepted;
    numVarBounds++;
    if (trace) {
      std::fprintf(stderr,
                   "[HIGHS-VB-ADD] target=%lld trigger=%lld phase=accept "
                   "coef=%.17g constant=%.17g min=%.17g\n",
                   static_cast<long long>(col), static_cast<long long>(vubcol),
                   vubcoef, vubconstant, minBound);
    }
  }
}

void HighsImplications::addVLB(HighsInt col, HighsInt vlbcol, double vlbcoef,
                               double vlbconstant) {
  addVLB(col, vlbcol, vlbcoef, vlbconstant,
         mipsolver.mipdata_->getDomain().col_lower_[col],
         mipsolver.isColIntegral(col));
}

void HighsImplications::addVLB(HighsInt col, HighsInt vlbcol, double vlbcoef,
                               double vlbconstant, double colllowerbound,
                               bool colisintegral) {
  ++sourceConformanceStats.vlb_attempts;
  // assume that VLBs do not have infinite coefficients and infinite constant
  // terms since such VLBs effectively evaluate to NaN.
  assert(std::abs(vlbcoef) != kHighsInf || std::abs(vlbconstant) != kHighsInf);
  if (tooManyVarBounds()) return;

  VarBound vlb{vlbcoef, vlbconstant};

  if (colisintegral) {
    // try to strengthen VLB
    strengthenVarBound(vlb, HighsInt{-1});
    if (vlb.coef == 0.0) return;
  }

  mipsolver.mipdata_->debugSolution.checkVlb(col, vlbcol, vlbcoef, vlbconstant);

  double maxBound = vlb.maxValue();
  if (maxBound <= colllowerbound + mipsolver.mipdata_->feastol) return;

  auto insertresult = vlbs[col].insert_or_get(vlbcol, vlb);

  if (!insertresult.second) {
    VarBound& currentvlb = *insertresult.first;

    double currentMaxBound = currentvlb.maxValue();
    if (maxBound > currentMaxBound + mipsolver.mipdata_->feastol) {
      currentvlb.coef = vlb.coef;
      currentvlb.constant = vlb.constant;
      ++sourceConformanceStats.vlb_replaced;
    }
  } else {
    ++sourceConformanceStats.vlb_accepted;
    numVarBounds++;
  }
}

void HighsImplications::rebuild(HighsInt ncols,
                                const std::vector<HighsInt>& orig2reducedcol,
                                const std::vector<HighsInt>& orig2reducedrow) {
  std::vector<HighsHashTree<HighsInt, VarBound>> oldvubs;
  std::vector<HighsHashTree<HighsInt, VarBound>> oldvlbs;

  oldvlbs.swap(vlbs);
  oldvubs.swap(vubs);

  colsubstituted.clear();
  colsubstituted.shrink_to_fit();
  implications.clear();
  implications.shrink_to_fit();

  implications.resize(2 * ncols);
  colsubstituted.resize(ncols);
  substitutions.clear();
  vubs.clear();
  vubs.shrink_to_fit();
  vubs.resize(ncols);
  vlbs.clear();
  vlbs.shrink_to_fit();
  vlbs.resize(ncols);
  numImplications = 0;
  numVarBounds = 0;
  HighsInt oldncols = oldvubs.size();

  nextCleanupCall = mipsolver.numNonzero();

  for (HighsInt i = 0; i != oldncols; ++i) {
    HighsInt newi = orig2reducedcol[i];

    if (newi == -1 ||
        !mipsolver.mipdata_->postSolveStack.isColLinearlyTransformable(newi))
      continue;

    oldvubs[i].for_each([&](HighsInt vubCol, VarBound vub) {
      HighsInt newVubCol = orig2reducedcol[vubCol];
      if (newVubCol == -1) return;

      if (!mipsolver.mipdata_->getDomain().isBinary(newVubCol) ||
          !mipsolver.mipdata_->postSolveStack.isColLinearlyTransformable(
              newVubCol))
        return;

      const int trace_col = hacdcpfImpTraceInt("HIGHS_VB_TRACE_COL");
      const int trace_trigger = hacdcpfImpTraceInt("HIGHS_VB_TRACE_TRIGGER");
      if (static_cast<int>(newi) == trace_col &&
          (trace_trigger < 0 || static_cast<int>(newVubCol) == trace_trigger)) {
        std::fprintf(stderr,
                     "[HIGHS-VB-XFORM] oldTarget=%lld oldTrigger=%lld "
                     "newTarget=%lld newTrigger=%lld coef=%.17g constant=%.17g\n",
                     static_cast<long long>(i), static_cast<long long>(vubCol),
                     static_cast<long long>(newi),
                     static_cast<long long>(newVubCol), vub.coef,
                     vub.constant);
      }

      addVUB(newi, newVubCol, vub.coef, vub.constant);
    });

    oldvlbs[i].for_each([&](HighsInt vlbCol, VarBound vlb) {
      HighsInt newVlbCol = orig2reducedcol[vlbCol];
      if (newVlbCol == -1) return;

      if (!mipsolver.mipdata_->getDomain().isBinary(newVlbCol) ||
          !mipsolver.mipdata_->postSolveStack.isColLinearlyTransformable(
              newVlbCol))
        return;

      addVLB(newi, newVlbCol, vlb.coef, vlb.constant);
    });

    // todo also add old implications once implications can be added
    // incrementally for now we discard the old implications as they might be
    // weaker then newly computed ones and adding them would block computation
    // of new implications
  }
}

void HighsImplications::buildFrom(const HighsImplications& init) {
  // todo check if this should be done
  HighsInt numcol = mipsolver.numCol();

  for (HighsInt i = 0; i != numcol; ++i) {
    init.vubs[i].for_each([&](HighsInt vubCol, VarBound vub) {
      if (!mipsolver.mipdata_->getDomain().isBinary(vubCol)) return;
      const int trace_col = hacdcpfImpTraceInt("HIGHS_VB_TRACE_COL");
      const int trace_trigger = hacdcpfImpTraceInt("HIGHS_VB_TRACE_TRIGGER");
      if (static_cast<int>(i) == trace_col &&
          (trace_trigger < 0 || static_cast<int>(vubCol) == trace_trigger)) {
        std::fprintf(stderr,
                     "[HIGHS-VB-BUILDFROM] target=%lld trigger=%lld "
                     "coef=%.17g constant=%.17g\n",
                     static_cast<long long>(i), static_cast<long long>(vubCol),
                     vub.coef, vub.constant);
      }
      addVUB(i, vubCol, vub.coef, vub.constant);
    });

    init.vlbs[i].for_each([&](HighsInt vlbCol, VarBound vlb) {
      if (!mipsolver.mipdata_->getDomain().isBinary(vlbCol)) return;
      addVLB(i, vlbCol, vlb.coef, vlb.constant);
    });

    // todo also add old implications once implications can be added
    // incrementally for now we discard the old implications as they might be
    // weaker then newly computed ones and adding them would block computation
    // of new implications
  }
}

void HighsImplications::separateImpliedBounds(
    const HighsLpRelaxation& lpRelaxation, const std::vector<double>& sol,
    HighsCutPool& cutpool, double feastol, HighsDomain& globaldom,
    bool thread_safe) {
  std::array<HighsInt, 2> inds;
  std::array<double, 2> vals;
  double rhs;

  HighsInt numboundchgs = 0;

  // first do probing on all candidates that have not been probed yet
  if (!mipsolver.mipdata_->cliquetable.isFull() && !thread_safe) {
    auto oldNumQueries =
        mipsolver.mipdata_->cliquetable.numNeighbourhoodQueries;
    HighsInt oldNumEntries = mipsolver.mipdata_->cliquetable.getNumEntries();

    for (std::pair<HighsInt, double> fracint :
         lpRelaxation.getFractionalIntegers()) {
      HighsInt col = fracint.first;
      if (globaldom.col_lower_[col] != 0.0 ||
          globaldom.col_upper_[col] != 1.0 ||
          (implicationsCached(col, 0) && implicationsCached(col, 1)))
        continue;

      mipsolver.profiling_->start(kMipClockProbingImplications);
      const bool probing_result = runProbing(col, numboundchgs);
      mipsolver.profiling_->stop(kMipClockProbingImplications);
      if (probing_result) {
        if (globaldom.infeasible()) return;
      }

      if (mipsolver.mipdata_->cliquetable.isFull()) break;
    }

    // if (!mipsolver.submip)
    //   printf("numEntries: %d, beforeProbing: %d\n",
    //          mipsolver.mipdata_->cliquetable.getNumEntries(), oldNumEntries);
    HighsInt numNewEntries =
        mipsolver.mipdata_->cliquetable.getNumEntries() - oldNumEntries;

    nextCleanupCall -= std::max(HighsInt{0}, numNewEntries);

    if (nextCleanupCall < 0) {
      // HighsInt oldNumEntries =
      // mipsolver.mipdata_->cliquetable.getNumEntries();
      if (!mipsolver.mipdata_->parallelLockActive())
        mipsolver.mipdata_->cliquetable.runCliqueMerging(globaldom);

      // printf("numEntries: %d, beforeMerging: %d\n",
      //        mipsolver.mipdata_->cliquetable.getNumEntries(), oldNumEntries);
      nextCleanupCall =
          std::min(mipsolver.mipdata_->numCliqueEntriesAfterFirstPresolve,
                   mipsolver.mipdata_->cliquetable.getNumEntries());
      // printf("nextCleanupCall: %d\n", nextCleanupCall);
    }

    if (!mipsolver.mipdata_->parallelLockActive())
      mipsolver.mipdata_->cliquetable.numNeighbourhoodQueries = oldNumQueries;
  }

  for (std::pair<HighsInt, double> fracint :
       lpRelaxation.getFractionalIntegers()) {
    HighsInt col = fracint.first;
    // skip non binary variables
    if (globaldom.col_lower_[col] != 0.0 || globaldom.col_upper_[col] != 1.0)
      continue;

    bool infeas;
    if (implicationsCached(col, 1)) {
      const std::vector<HighsDomainChange>& implics =
          getImplications(col, 1, infeas);
      if (globaldom.infeasible()) return;

      if (infeas) {
        vals[0] = 1.0;
        inds[0] = col;
        cutpool.addCut(mipsolver, inds.data(), vals.data(), 1, 0.0, false, true,
                       false);
        continue;
      }

      HighsInt nimplics = implics.size();
      for (HighsInt i = 0; i < nimplics; ++i) {
        if (implics[i].boundtype == HighsBoundType::kUpper) {
          if (implics[i].boundval + feastol >=
              globaldom.col_upper_[implics[i].column])
            continue;

          vals[0] = 1.0;
          inds[0] = implics[i].column;
          vals[1] =
              globaldom.col_upper_[implics[i].column] - implics[i].boundval;
          inds[1] = col;
          rhs = globaldom.col_upper_[implics[i].column];

        } else {
          if (implics[i].boundval - feastol <=
              globaldom.col_lower_[implics[i].column])
            continue;

          vals[0] = -1.0;
          inds[0] = implics[i].column;
          vals[1] =
              implics[i].boundval - globaldom.col_lower_[implics[i].column];
          inds[1] = col;
          rhs = -globaldom.col_lower_[implics[i].column];
        }

        double viol = sol[inds[0]] * vals[0] + sol[inds[1]] * vals[1] - rhs;

        if (viol > feastol) {
          // printf("added implied bound cut to pool\n");
          cutpool.addCut(mipsolver, inds.data(), vals.data(), 2, rhs,
                         !mipsolver.isColContinuous(implics[i].column), false,
                         false, false);
        }
      }
    }

    if (implicationsCached(col, 0)) {
      const std::vector<HighsDomainChange>& implics =
          getImplications(col, 0, infeas);
      if (globaldom.infeasible()) return;

      if (infeas) {
        vals[0] = -1.0;
        inds[0] = col;
        cutpool.addCut(mipsolver, inds.data(), vals.data(), 1, -1.0, false,
                       true, false);
        continue;
      }

      HighsInt nimplics = implics.size();
      for (HighsInt i = 0; i < nimplics; ++i) {
        if (implics[i].boundtype == HighsBoundType::kUpper) {
          if (implics[i].boundval + feastol >=
              globaldom.col_upper_[implics[i].column])
            continue;

          vals[0] = 1.0;
          inds[0] = implics[i].column;
          vals[1] =
              implics[i].boundval - globaldom.col_upper_[implics[i].column];
          inds[1] = col;
          rhs = implics[i].boundval;
        } else {
          if (implics[i].boundval - feastol <=
              globaldom.col_lower_[implics[i].column])
            continue;

          vals[0] = -1.0;
          inds[0] = implics[i].column;
          vals[1] =
              globaldom.col_lower_[implics[i].column] - implics[i].boundval;
          inds[1] = col;
          rhs = -implics[i].boundval;
        }

        double viol = sol[inds[0]] * vals[0] + sol[inds[1]] * vals[1] - rhs;

        if (viol > feastol) {
          // printf("added implied bound cut to pool\n");
          cutpool.addCut(mipsolver, inds.data(), vals.data(), 2, rhs,
                         !mipsolver.isColContinuous(implics[i].column), false,
                         false, false);
        }
      }
    }
  }
}

void HighsImplications::cleanupVarbounds(HighsInt col) {
  double ub = mipsolver.mipdata_->getDomain().col_upper_[col];
  double lb = mipsolver.mipdata_->getDomain().col_lower_[col];

  if (ub == lb) {
    HighsInt numVubs = 0;
    vubs[col].for_each([&](HighsInt vubCol, VarBound& vub) { numVubs++; });
    HighsInt numVlbs = 0;
    vlbs[col].for_each([&](HighsInt vlbCol, VarBound& vlb) { numVlbs++; });
    numVarBounds -= numVubs + numVlbs;
    vlbs[col].clear();
    vubs[col].clear();
    return;
  }

  std::vector<HighsInt> delVbds;

  vubs[col].for_each([&](HighsInt vubCol, VarBound& vub) {
    bool redundant = false;
    bool infeasible = false;
    cleanupVub(col, vubCol, vub, ub, redundant, infeasible);
    if (redundant) delVbds.push_back(vubCol);
    if (infeasible) return;
  });

  for (HighsInt vubCol : delVbds) vubs[col].erase(vubCol);
  numVarBounds -= delVbds.size();
  delVbds.clear();

  vlbs[col].for_each([&](HighsInt vlbCol, VarBound& vlb) {
    bool redundant = false;
    bool infeasible = false;
    cleanupVlb(col, vlbCol, vlb, lb, redundant, infeasible);
    if (redundant) delVbds.push_back(vlbCol);
    if (infeasible) return;
  });

  for (HighsInt vlbCol : delVbds) vlbs[col].erase(vlbCol);
  numVarBounds -= delVbds.size();
}

void HighsImplications::cleanupVlb(HighsInt col, HighsInt vlbCol,
                                   HighsImplications::VarBound& vlb, double lb,
                                   bool& redundant, bool& infeasible,
                                   bool allowBoundChanges) const {
  // initialize
  redundant = false;
  infeasible = false;

  // return if there is no variable bound
  if (vlbCol == -1) return;

  // check variable lower bound
  mipsolver.mipdata_->debugSolution.checkVlb(col, vlbCol, vlb.coef,
                                             vlb.constant);

  HighsCDouble maxlb = vlb.maxValue();
  HighsCDouble minlb = vlb.minValue();

  if (maxlb <= lb + mipsolver.mipdata_->feastol) {
    // variable bound is redundant
    redundant = true;
  } else if (minlb < lb - mipsolver.mipdata_->epsilon) {
    // coefficient can be tightened
    double newcoef = static_cast<double>(lb - maxlb);
    if (vlb.coef < 0) {
      vlb.coef = newcoef;
    } else {
      vlb.constant = lb;
      vlb.coef = -newcoef;
    }
    // check tightened variable lower bound
    mipsolver.mipdata_->debugSolution.checkVlb(col, vlbCol, vlb.coef,
                                               vlb.constant);
  } else if (allowBoundChanges && minlb > lb + mipsolver.mipdata_->epsilon) {
    mipsolver.mipdata_->getDomain().changeBound(
        HighsBoundType::kLower, col, static_cast<double>(minlb),
        HighsDomain::Reason::unspecified());
    infeasible = mipsolver.mipdata_->getDomain().infeasible();
  }
}

void HighsImplications::cleanupVub(HighsInt col, HighsInt vubCol,
                                   HighsImplications::VarBound& vub, double ub,
                                   bool& redundant, bool& infeasible,
                                   bool allowBoundChanges) const {
  // initialize
  redundant = false;
  infeasible = false;

  // return if there is no variable bound
  if (vubCol == -1) return;

  // check variable upper bound
  mipsolver.mipdata_->debugSolution.checkVub(col, vubCol, vub.coef,
                                             vub.constant);

  HighsCDouble maxub = vub.maxValue();
  HighsCDouble minub = vub.minValue();

  if (minub >= ub - mipsolver.mipdata_->feastol) {
    // variable bound is redundant
    redundant = true;
  } else if (maxub > ub + mipsolver.mipdata_->epsilon) {
    // coefficient can be tightened
    double newcoef = static_cast<double>(ub - minub);
    if (vub.coef > 0) {
      vub.coef = newcoef;
    } else {
      vub.constant = ub;
      vub.coef = -newcoef;
    }
    // check tightened variable upper bound
    mipsolver.mipdata_->debugSolution.checkVub(col, vubCol, vub.coef,
                                               vub.constant);
  } else if (allowBoundChanges && maxub < ub - mipsolver.mipdata_->epsilon) {
    mipsolver.mipdata_->getDomain().changeBound(
        HighsBoundType::kUpper, col, static_cast<double>(maxub),
        HighsDomain::Reason::unspecified());
    infeasible = mipsolver.mipdata_->getDomain().infeasible();
  }
}

void HighsImplications::applyImplications(HighsDomain& domain,
                                          const HighsInt col,
                                          const HighsInt val) {
  assert(domain.isFixed(col));

  auto checkImplication = [&](const HighsDomainChange& domchg) -> bool {
    assert(!domain.infeasible());
    if (domain.isFixed(domchg.column)) return false;
    const bool isint =
        domain.variableType(domchg.column) != HighsVarType::kContinuous;
    // Directly change bounds on all integer columns. Only change continuous
    // columns that fix the column, as changing their domains risks
    // suppressing further bound changes found in propagation, e.g.,
    // change [0, 100] -> [0, 50], propagation could tighten to [0, 48], but
    // such a tightening would not be applied due to min boundRange improvement.
    if (domchg.boundtype == HighsBoundType::kLower) {
      if ((!isint && domchg.boundval >
                         domain.col_upper_[domchg.column] - domain.feastol()) ||
          (isint && domchg.boundval >
                        domain.col_lower_[domchg.column] + domain.feastol())) {
        domain.changeBound(domchg, HighsDomain::Reason::cliqueTable(col, val));
      }
    } else {
      if ((!isint && domchg.boundval <
                         domain.col_lower_[domchg.column] + domain.feastol()) ||
          (isint && domchg.boundval <
                        domain.col_upper_[domchg.column] - domain.feastol())) {
        domain.changeBound(domchg, HighsDomain::Reason::cliqueTable(col, val));
      }
    }
    return domain.infeasible();
  };

  HighsInt loc = 2 * col + val;
  if (implications[loc].computed) {
    for (HighsDomainChange& domchg : implications[loc].implics) {
      if (checkImplication(domchg)) break;
    }
  }
}
