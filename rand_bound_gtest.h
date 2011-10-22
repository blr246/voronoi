#ifndef _HPS_AMBULANCE_RAND_BOUND_GTEST_H_
#define _HPS_AMBULANCE_RAND_BOUND_GTEST_H_
#include "rand_bound.h"
#include "gtest/gtest.h"

namespace _hps_ambulance_rand_bound_gtest_h_
{
using namespace hps;

TEST(RatioOfUniforms, RandBound)
{
  // reissb -- 20111017 -- This is a visual GTest. Inspect the output
  //   histogram to verify that it has the proper normal distribution shape.
  enum { Mean = 20, };
  enum { Var = 5, };
  enum { HistVar = Var * 3, };
  std::vector<int> hist(2 * (HistVar) + 1, 0);
  for (int i = 0; i < 300; ++i)
  {
    const double val = RatioOfUniforms(20, 4);
    const int bin = static_cast<int>((val - Mean)) + HistVar;
    const int binClamp = std::min(std::max(0, bin), static_cast<int>(hist.size()) - 1);
    ++hist[binClamp];
  }
  for (int i = Mean - HistVar, binIdx = 0; i <= Mean + HistVar; ++i, ++binIdx)
  {
    std::cout << std::setw(2) << std::setfill(' ') << std::right << i << ":"
              << std::setw(hist[binIdx]) << std::setfill('x') << " " << std::endl;
  }
}

}

#endif //_HPS_AMBULANCE_RAND_BOUND_GTEST_H_
