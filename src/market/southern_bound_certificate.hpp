#pragma once

#include "hacdcpf/engine/engine.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace hacdcpf::market::detail {
inline bool same_lp_with_tighter_box(const engine::LPModel& old,const engine::LPModel& next) {
  const auto same_vector=[](const auto& a,const auto& b){return a.size()==b.size()&&(a.array()==b.array()).all();};
  const auto same_matrix=[](const auto& a,const auto& b){return a.isCompressed()&&b.isCompressed()&&
    a.rows()==b.rows()&&a.cols()==b.cols()&&a.nonZeros()==b.nonZeros()&&
    std::equal(a.outerIndexPtr(),a.outerIndexPtr()+a.outerSize()+1,b.outerIndexPtr())&&
    std::equal(a.innerIndexPtr(),a.innerIndexPtr()+a.nonZeros(),b.innerIndexPtr())&&
    std::equal(a.valuePtr(),a.valuePtr()+a.nonZeros(),b.valuePtr());};
  if(old.sense!=next.sense||old.vars.size()!=next.vars.size()||!same_vector(old.c,next.c)||
      !same_vector(old.b,next.b)||!same_vector(old.beq,next.beq)||!same_vector(old.row_lhs,next.row_lhs)||
      !same_matrix(old.A,next.A)||!same_matrix(old.Aeq,next.Aeq))return false;
  for(size_t c=0;c<old.vars.size();++c)if(old.vars[c].name!=next.vars[c].name||
      !(next.vars[c].lb>=old.vars[c].lb)||!(next.vars[c].ub<=old.vars[c].ub))return false;
  return true;
}

// Lagrangian box support bound with outward rounding; performance.md,
// 2000-node goal. The caller must justify any bounds tighter than the LP box.
inline std::optional<double> box_dual_lower_bound(const engine::LPModel& lp,
    const Eigen::VectorXd& dual, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) {
  const auto n=lp.c.size(),m=lp.A.rows(),q=lp.Aeq.rows();
  if(lp.sense!=engine::Sense::Minimize||lp.A.cols()!=n||lp.Aeq.cols()!=n||
      lp.b.size()!=m||lp.beq.size()!=q||!lp.b.allFinite()||!lp.beq.allFinite()||
      dual.size()!=m+q||lower.size()!=n||upper.size()!=n||
      !dual.allFinite()||!lower.allFinite()||!upper.allFinite()||!lp.c.allFinite()||
      !(lower.array()<=upper.array()).all())return std::nullopt;
  const double infinity=std::numeric_limits<double>::infinity();
  const auto down=[&](double v){return std::nextafter(v,-infinity);};
  const auto up=[&](double v){return std::nextafter(v,infinity);};
  const auto product_down=[&](double a,double b){return a==0||b==0?0:down(a*b);};
  const auto product_up=[&](double a,double b){return a==0||b==0?0:up(a*b);};
  Eigen::VectorXd y=dual;
  for(Eigen::Index i=0;i<m;++i)y[i]=std::min(0.0,y[i]);
  double bound=0;
  for(Eigen::Index i=0;i<m+q;++i) {
    const double term=product_down(y[i],i<m?lp.b[i]:lp.beq[i-m]);
    if(term!=0)bound=down(bound+term);
  }
  for(Eigen::Index j=0;j<n;++j) {
    double lo=lp.c[j],hi=lp.c[j];
    const auto subtract=[&](const auto& matrix,Eigen::Index offset){
      for(Eigen::SparseMatrix<double>::InnerIterator it(matrix,j);it;++it) {
        if(!std::isfinite(it.value())){lo=hi=infinity;return;}
        const double value=y[offset+it.row()];if(value==0||it.value()==0)continue;
        lo=down(lo-product_up(it.value(),value));hi=up(hi-product_down(it.value(),value));
      }
    };
    subtract(lp.A,0);subtract(lp.Aeq,m);
    const double term=std::min({product_down(lo,lower[j]),product_down(lo,upper[j]),
      product_down(hi,lower[j]),product_down(hi,upper[j])});
    if(term!=0)bound=down(bound+term);
    if(!std::isfinite(bound)||!std::isfinite(lo)||!std::isfinite(hi))return std::nullopt;
  }
  return std::isfinite(bound)?std::optional<double>(bound):std::nullopt;
}
} // namespace hacdcpf::market::detail
