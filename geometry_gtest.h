#ifndef _HPS_GEOMETRY_GEOMETRY_GTEST_H_
#define _HPS_GEOMETRY_GEOMETRY_GTEST_H_
#include "geometry.h"
#include "gtest/gtest.h"

namespace _hps_geometry_geometry_gtest_h_
{
using namespace hps;

TEST(GeometryObjects, geometry)
{
  // Point.
  {
    Point<int> p(1, 2);
    EXPECT_EQ(1, p.x);
    EXPECT_EQ(2, p.y);
  }
  // Vector2.
  {
    {
      Vector2<int> v;
      v.v[0] = 1;
      v.v[1] = 2;
      EXPECT_EQ(1, v.x);
      EXPECT_EQ(2, v.y);
    }
    {
      Vector2<int> v;
      v.x = 1;
      v.y = 2;
      EXPECT_EQ(1, v.v[0]);
      EXPECT_EQ(2, v.v[1]);
    }
  }
  // Segment.
  {
    Segment<int> s(Point<int>(5, 4), Point<int>(8, 12));
    EXPECT_EQ(5, s.p0.x);
    EXPECT_EQ(4, s.p0.y);
    EXPECT_EQ(8, s.p1.x);
    EXPECT_EQ(12, s.p1.y);
  }
  // DirectedSegment.
  {
    DirectedSegment<int> s(Point<int>(5, 4), Point<int>(8, 12));
    EXPECT_EQ(5, s.p0.x);
    EXPECT_EQ(4, s.p0.y);
    EXPECT_EQ(8, s.p1.x);
    EXPECT_EQ(12, s.p1.y);
  }
  // Line.
  {
    Line<int> l(Point<int>(1, 2), Vector2<int>(3, 4));
    EXPECT_EQ(1, l.p0.x);
    EXPECT_EQ(2, l.p0.y);
    EXPECT_EQ(3, l.dir.x);
    EXPECT_EQ(4, l.dir.y);
  }
  // AxisAlignedBox.
  {
    AxisAlignedBox<int> box(Vector2<int>(0, 0), Vector2<int>(100, 100));
    EXPECT_EQ(0, box.mins.x);
    EXPECT_EQ(0, box.mins.y);
    EXPECT_EQ(100, box.maxs.x);
    EXPECT_EQ(100, box.maxs.y);
  }
}

}

#endif //_HPS_GEOMETRY_GEOMETRY_GTEST_H_
