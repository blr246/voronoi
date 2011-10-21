#ifndef _HPS_GEOMETRY_GEOMETRY_H_
#define _HPS_GEOMETRY_GEOMETRY_H_

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4201) // nameless struct/union
#endif

namespace hps
{
namespace geometry
{

template <typename NumericType>
struct Point
{
  Point() : x(0), y(0) {}
  Point(const NumericType x_, const NumericType y_) : x(x_), y(y_) {}
  NumericType x;
  NumericType y;
};

template <typename NumericType>
struct Vector2
{
  enum { Dim = 2, };
  Vector2() : x(0), y(0) {}
  Vector2(const NumericType x_, const NumericType y_) : x(x_), y(y_) {}
  union
  {
    struct
    {
      NumericType x;
      NumericType y;
    };
    NumericType v[2];
  };
};

template <typename NumericType>
struct Segment
{
  Segment() : p0(), p1() {}
  Segment(const Point<NumericType>& p0_, const Point<NumericType>& p1_)
    : p0(p0_),
      p1(p1_)
  {}
  Point<NumericType> p0;
  Point<NumericType> p1;
};

template <typename NumericType>
struct DirectedSegment
{
  DirectedSegment() : p0(), p1() {}
  DirectedSegment(const Point<NumericType>& p0_, const Point<NumericType>& p1_)
    : p0(p0_),
      p1(p1_)
  {}
  Point<NumericType> p0;
  Point<NumericType> p1;
};

template <typename NumericType>
struct Line
{
  Line() : p0(), dir() {}
  Line(const Point<NumericType>& p0_, const Vector2<NumericType>& dir_)
    : p0(p0_),
      dir(dir_)
  {}
  Point<NumericType> p0;
  Vector2<NumericType> dir;
};

template <typename NumericType>
struct AxisAlignedBox
{
  AxisAlignedBox() : mins(), maxs() {}
  AxisAlignedBox(const Vector2<NumericType>& mins_, const Vector2<NumericType>& maxs_)
    : mins(mins_),
      maxs(maxs_)
  {}
  Vector2<NumericType> mins;
  Vector2<NumericType> maxs;
};

//template <typename NumericType>
//struct HitTestBox
//{
//  enum { BoxSides = 4, };
//  union
//  {
//    struct
//    {
//      Line<NumericType> left;
//      Line<NumericType> top;
//      Line<NumericType> right;
//      Line<NumericType> bottom;
//    };
//    Line<NumericType> lines[BoxSides];
//  };
//};

}
using namespace geometry;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif //_HPS_GEOMETRY_GEOMETRY_H_
