/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                       */
/*    This file is part of the HiGHS linear optimization suite           */
/*                                                                       */
/*    Available as open-source under the MIT License                     */
/*                                                                       */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef HIGHS_HACDCPF_LOCAL_CUTS_H_
#define HIGHS_HACDCPF_LOCAL_CUTS_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "lp_data/HConst.h"

struct HacdcpfLocalCut {
  std::vector<HighsInt> indices;
  std::vector<double> values;
  double lower{-kHighsInf};
  double upper{kHighsInf};
  unsigned char integral{0};
  unsigned char propagate{1};
  uint64_t row_hash{0};
  uint64_t row_sig{0};
  std::string proof_family;
  std::string proof_key;

  HighsInt size() const { return static_cast<HighsInt>(indices.size()); }

  double maxAbsVal() const {
    double max_abs = 0.0;
    for (double value : values) max_abs = std::max(max_abs, std::abs(value));
    return max_abs;
  }
};

#endif
