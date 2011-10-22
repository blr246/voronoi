#include "rand_bound_gtest.h"
#include "geometry_gtest.h"
#include "voronoi_core_gtest.h"
#include "player_gtest.h"
#include "gtest/gtest.h"
#ifdef WIN32
#include <time.h>
#elif __APPLE__
#include <time.h>
#else
#include <sys/time.h>
#endif

int main(int argc, char** argv)
{
  srand(static_cast<unsigned int>(time(NULL)));
  testing::InitGoogleTest(&argc, argv);
  testing::FLAGS_gtest_catch_exceptions = false;
  return RUN_ALL_TESTS();
}
