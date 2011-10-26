#ifndef _HPS_GEOMETRY_GEOMETRY_GTEST_H_
#define _HPS_GEOMETRY_GEOMETRY_GTEST_H_
#include "geometry.h"
#include "rand_bound.h"
#include "gtest/gtest.h"

template <typename NumericType>
inline std::ostream& operator<<(std::ostream& stream, const hps::Vector2<NumericType>& v)
{
  stream << "(" << v.x << ", " << v.y << ")";
  return stream;
}

namespace _hps_geometry_geometry_gtest_h_
{
using namespace hps;

TEST(GeometryObjects, geometry)
{
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
  // Segment2.
  {
    Segment2<int> s(Vector2<int>(5, 4), Vector2<int>(8, 12));
    EXPECT_EQ(5, s.p0.x);
    EXPECT_EQ(4, s.p0.y);
    EXPECT_EQ(8, s.p1.x);
    EXPECT_EQ(12, s.p1.y);
  }
  // DirectedSegment.
  {
    DirectedSegment<int> s(Vector2<int>(5, 4), Vector2<int>(8, 12));
    EXPECT_EQ(5, s.p0.x);
    EXPECT_EQ(4, s.p0.y);
    EXPECT_EQ(8, s.p1.x);
    EXPECT_EQ(12, s.p1.y);
  }
  // Line.
  {
    Line<int> l(Vector2<int>(1, 2), Vector2<int>(3, 4));
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

template <typename TestType>
void TestVector2()
{
  // Equality.
  {
    EXPECT_TRUE(Vector2<TestType>(1, 2) == Vector2<TestType>(1, 2));
    EXPECT_FALSE(Vector2<TestType>(1, 2) == Vector2<TestType>(1, 3));
    EXPECT_FALSE(Vector2<TestType>(2, 2) == Vector2<TestType>(1, 2));
    EXPECT_TRUE(Vector2<TestType>(1, 2) != Vector2<TestType>(1, 3));
    EXPECT_TRUE(Vector2<TestType>(2, 2) != Vector2<TestType>(1, 2));
    EXPECT_FALSE(Vector2<TestType>(1, 2) != Vector2<TestType>(1, 2));
  }
  // Addition.
  {
    const Vector2<TestType> v = Vector2<TestType>(1, 2);
    EXPECT_EQ(Vector2<TestType>(v.x * 2, v.y * 2), v + v);
    EXPECT_EQ(Vector2<TestType>(v.x * 3, v.y * 3), v + v + v);
    const Vector2<TestType> vx = Vector2<TestType>(v.x, 0);
    EXPECT_EQ(Vector2<TestType>(v.x * 2, v.y), v + vx);
    const Vector2<TestType> vy = Vector2<TestType>(0, v.y);
    EXPECT_EQ(Vector2<TestType>(v.x, v.y * 2), v + vy);
  }
  // Subtraction.
  {
    const Vector2<TestType> v = Vector2<TestType>(1, 2);
    EXPECT_EQ(Vector2<TestType>(0, 0), v - v);
    EXPECT_EQ(Vector2<TestType>(-v.x, -v.y), v - v - v);
    const Vector2<TestType> vx = Vector2<TestType>(v.x, 0);
    EXPECT_EQ(Vector2<TestType>(0, v.y), v - vx);
    const Vector2<TestType> vy = Vector2<TestType>(0, v.y);
    EXPECT_EQ(Vector2<TestType>(v.x, 0), v - vy);
  }
  // Unary minus.
  {
    const Vector2<TestType> v = Vector2<TestType>(1, 2);
    EXPECT_EQ(Vector2<TestType>(-v.x, -v.y), -v);
  }
  // Multiplication.
  {
    const Vector2<TestType> v = Vector2<TestType>(1, 2);
    EXPECT_EQ(Vector2<TestType>(v + v), 2 * v);
    EXPECT_EQ(Vector2<TestType>(v + v), v * 2);
    EXPECT_EQ(Vector2<TestType>(-v - v), -2 * v);
    EXPECT_EQ(Vector2<TestType>(-v - v), v * -2);
  }
  // Length squared.
  {
    {
      const Vector2<TestType> v = Vector2<TestType>(8, 0);
      EXPECT_EQ(v.x * v.x, Vector2LengthSq(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(-8, 0);
      EXPECT_EQ(v.x * v.x, Vector2LengthSq(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(0, 2);
      EXPECT_EQ(v.y * v.y, Vector2LengthSq(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(0, -2);
      EXPECT_EQ(v.y * v.y, Vector2LengthSq(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(6, -7);
      EXPECT_EQ((v.x * v.x) + (v.y * v.y), Vector2LengthSq(v));
    }
  }
  // Length.
  {
    {
      const Vector2<TestType> v = Vector2<TestType>(8, 0);
      EXPECT_EQ(v.x, Vector2Length(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(-8, 0);
      EXPECT_EQ(abs(v.x), Vector2Length(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(0, 2);
      EXPECT_EQ(v.y, Vector2Length(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(0, -2);
      EXPECT_EQ(abs(v.y), Vector2Length(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(6, 7);
      const float l = sqrt(static_cast<float>((v.x * v.x) + (v.y * v.y)));
      EXPECT_EQ(l, Vector2Length(v));
    }
  }
  // Vector2Rotate90.
  {
    {
      const Vector2<TestType> v = Vector2<TestType>(1, 0);
      EXPECT_EQ(Vector2<TestType>(0, 1), Vector2Rotate90(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(0, -2);
      EXPECT_EQ(Vector2<TestType>(2, 0), Vector2Rotate90(v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(-6, 3);
      EXPECT_EQ(Vector2<TestType>(-3, -6), Vector2Rotate90(v));
    }
  }
  // Vector2Dot.
  {
    {
      const Vector2<TestType> v = Vector2<TestType>(5, 6);
      EXPECT_EQ(Vector2LengthSq(v), Vector2Dot(v, v));
    }
    {
      const Vector2<TestType> v = Vector2<TestType>(5, 6);
      EXPECT_EQ(0, Vector2Dot(v, Vector2Rotate90(v)));
    }
  }
}
TEST(Vector2, geometry)
{
  {
    SCOPED_TRACE("Vector2<int>");
    TestVector2<int>();
  }
  {
    SCOPED_TRACE("Vector2<float>");
    TestVector2<float>();
  }
}

template <int Iterations>
struct LineIntersectLineTest
{
  inline static Vector2<float> Vector2RandUniform()
  {
    return Vector2<float>(RandUniform<float>(), RandUniform<float>());
  }

  static void Run()
  {
    // Positive test.
    {
      SCOPED_TRACE("LineIntersectLineTest(): Positive test.");
      enum { MaxCoord = 100, };
      const float kParallelDotBound = 1.0e-2f;
      const float kUnitVecLenSqBound = 1.0e-6f;
      const float kErrSqBound = 1.0e-5f;
      enum { MaxOffset = MaxCoord / 2, };
      for (int iteration = 0; iteration < Iterations; ++iteration)
      {
        // Get a random intersection point.
        const Vector2<float> intersectPt = (float) MaxCoord * Vector2RandUniform();
        // Get two random directions.
        const Vector2<float> dir_a = Vector2RandUniform();
        const Vector2<float> dir_a_norm = Vector2Normalize(dir_a);
        // Select dir_b so that it is not nearly parallel.
        Vector2<float> dir_b = Vector2RandUniform();
        Vector2<float> dir_b_norm = Vector2Normalize(dir_b);
        while (fabs(Vector2Dot(dir_a_norm, dir_b_norm)) > (1.0f - kParallelDotBound))
        {
          dir_b = Vector2RandUniform();
          dir_b_norm = Vector2Normalize(dir_b);
        }
        assert((1.0f - Vector2LengthSq(dir_a_norm)) < kUnitVecLenSqBound);
        assert((1.0f - Vector2LengthSq(dir_b_norm)) < kUnitVecLenSqBound);
        // Create two lines.
        const Line<float> a(intersectPt + (dir_a_norm * (1 + RandBound(MaxOffset))), dir_a_norm);
        const Line<float> b(intersectPt + (dir_b_norm * (1 + RandBound(MaxOffset))), dir_b_norm);
        // Intersect lines.
        Vector2<float> testIntersectPt;
        EXPECT_TRUE(LineIntersectLineUnique(a, b, &testIntersectPt));
        const float errSq = Vector2LengthSq(intersectPt - testIntersectPt);
        EXPECT_LT(errSq, kErrSqBound);
      }
    }
    // Negative test.
    {
      SCOPED_TRACE("LineIntersectLineTest(): Negative test.");
      enum { MaxCoord = 100, };
      const float kParallelDotBound = 1.0e-5f;
      const float kPerturbScale = 1.0e-4f;
      enum { MaxOffset = MaxCoord / 2, };
      for (int iteration = 0; iteration < Iterations; ++iteration)
      {
        // Get a random intersection point.
        const Vector2<float> intersectPt = (float) MaxCoord * Vector2RandUniform();
        // Get two random directions that are nearly parallel;
        const Vector2<float> dir_a = Vector2RandUniform();
        const Vector2<float> dir_a_norm = Vector2Normalize(dir_a);
        // Select dir_b so that it is nearly parallel.
        Vector2<float> dir_b = dir_a + (kPerturbScale * Vector2RandUniform());
        Vector2<float> dir_b_norm = Vector2Normalize(dir_b);
        while (fabs(Vector2Dot(dir_a_norm, dir_b_norm)) < (1.0f - kParallelDotBound))
        {
          dir_b = dir_a + (kPerturbScale * Vector2RandUniform());
          dir_b_norm = Vector2Normalize(dir_b);
        }
        // Create two lines.
        const Line<float> a(intersectPt + dir_a_norm * (1 + RandBound(MaxOffset)), dir_a_norm);
        const Line<float> b(intersectPt + dir_b_norm * (1 + RandBound(MaxOffset)), dir_b_norm);
        // Intersect lines.
        Vector2<float> testIntersectPt;
        EXPECT_FALSE(LineIntersectLineUnique(a, b, &testIntersectPt));
      }
    }
  }
};
TEST(LineIntersectLine, geometry)
{
#if NDEBUG
  LineIntersectLineTest<100000>::Run();
#else
  LineIntersectLineTest<10000>::Run();
#endif
}

}

#endif //_HPS_GEOMETRY_GEOMETRY_GTEST_H_
