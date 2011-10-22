#include "voronoi_core.h"
#include <algorithm>
#include <limits>
#include <assert.h>

namespace hps
{
namespace voronoi
{

namespace detail
{
/// <summary> Comparison function for two unit vectors. </summary>
/// <remarks>
///   <para> Sort unit vectors by angle measured from the x-axis. </summary>
/// </remarks>
struct UnitVectorFastAngleSort
{
  inline bool operator()(const Vector2<float>& v0, const Vector2<float>& v1) const
  {
#if !NDEBUG
    const float kUnitVecLenSqBound = 1.0e-6f;
    assert((1.0f - Vector2LengthSq(v0)) < kUnitVecLenSqBound);
    assert((1.0f - Vector2LengthSq(v1)) < kUnitVecLenSqBound);
#endif
    // Use the regions quadrant(I + II), quadrant(III, IV) to separate vectors.
    if ((v0.y * v1.y) < 0) // Different regions?
    {
      return v0.y > v1.y;
    }
    else
    {
      // Region defines reference vector. If y is positive, then it is the
      // positive x-axis. Otherwise it is the negative x-axis.
      const bool positiveX = (v0.y >= 0) && (v1.y >= 0);
      const Vector2<float> ref(Vector2<float>(positiveX ? 1.0f : -1.0f, 0.0f));
      const float cosTheta_v0 = Vector2Dot(v0, ref);
      const float cosTheta_v1 = Vector2Dot(v1, ref);
      // Smaller angle comes first (cos(0) = 0).
      return (cosTheta_v0 > cosTheta_v1) ? true : false;
    }
  }
};

/// <summary> Comparison function for lines with unit direction. </summary>
/// <remarks>
///   <para> Sort lines by angle measured from the x-axis. </summary>
/// </remarks>
struct LineFastAngleSort
{
  inline bool operator()(const Line<float>& a, const Line<float>& b) const
  {
    static UnitVectorFastAngleSort s_fastAngleSort;
    return s_fastAngleSort(a.dir, b.dir);
  }
};
}

void Voronoi::Scores(ScoreList* scores) const
{
  // reissb -- 20111021 -- Scoring algorithm:
  //   For each stone s_i:
  //     Lines <= empty
  //     Boundaries <= empty
  //     PolygonPoints <= empty
  //     For each stone s_j such that s_j != s_i:
  //       Push line(point = average(s_i, s_j), dir = perp(s_i - s_j)) to Lines
  //     For each stone s_j such that s_j != s_i:
  //       Find closest point p by intersecting ray (s_j - s_i) with all Lines
  //       If p = average(s_i, s_j), then stone s_j defines a region boundary
  //         for s_i so push line to BoundaryLines
  //     Sort BoundaryLines in angle order using the line direction.
  //     For each consecutive pair of boundaries in sorted list
  //       Push point p = intersect(b_n, b_n+1) to PolygonPoints
  //     Push point p = intersect(b_0, b_last) to PolygonPoints
  //     Compute the polygon area using polygon points as in
  //       http://www.mathopenref.com/coordpolygonarea.html

  // Initialize scores.
  scores->assign(m_players, 0);

  // Compute score for all stones.
  typedef std::vector<StoneNormalized>::const_iterator StoneIterator;
  for (StoneIterator s_i = m_stonesPlayedNorm.begin();
       s_i != m_stonesPlayedNorm.end();
       ++s_i)
  {
    // Clear score data.
    m_scoreData.Reset();
    std::vector<Line<float> >& lines = m_scoreData.lines;
    std::vector<Line<float> >& orthoLines = m_scoreData.orthoLines;
    std::vector<Line<float> >& boundaries = m_scoreData.boundaries;
    std::vector<Vector2<float> >& vertices = m_scoreData.vertices;
    // Find lines.
    for (StoneIterator s_j = m_stonesPlayedNorm.begin();
         s_j != m_stonesPlayedNorm.end();
         ++s_j)
    {
      if (s_i == s_j)
      {
        continue;
      }
      const Vector2<float> p0 = 0.5f * (s_i->pos + s_j->pos);
      const Vector2<float> dir(s_i->pos - s_j->pos);
      lines.push_back(Line<float>(p0, dir));
      orthoLines.push_back(Line<float>(p0, Vector2Rotate90(dir)));
    }
    // Find closest point to stone on lines. If it matches a line point,
    // then the line is a boundary.
    typedef std::vector<Line<float> >::const_iterator LineIterator;
    for (LineIterator orthoLine = orthoLines.begin();
         orthoLine != orthoLines.end();
         ++orthoLine)
    {
      // Intersect with all other lines to find closest.
      const Line<float>* closestLine = NULL;
      float closestLineDist = std::numeric_limits<float>::infinity();
      for (LineIterator line = lines.begin();
           line != lines.end();
           ++line)
      {
        Vector2<float> isect;
#if NDEBUG
        LineIntersectLineUnique(*line, *orthoLine, &isect);
#else
        const bool isectRes = LineIntersectLineUnique(*line, *orthoLine, &isect);
        assert(isectRes);
#endif
        Vector2<float> stoneToLine(isect - s_i->pos);
        const float lineDist = Vector2Length(stoneToLine);
        if (lineDist < closestLineDist)
        {
          closestLineDist = lineDist;
          closestLine = &*line;
        }
      }
      // See if we found a boundary for stone s_i.
      if (orthoLine->p0 == closestLine->p0)
      {
        boundaries.push_back(*closestLine);
      }
    }
    assert(boundaries.size() > 2);
    // reissb -- 20111022 -- The shape is always convex, so we don't need to
    //   worry about the sorting producing out-of-order edges.
    // Sort the boundaries.
    std::sort(boundaries.begin(), boundaries.end(), detail::LineFastAngleSort());
    // Find intersection of all consecutive boundaries to get polygon points.
    {
      LineIterator l0 = boundaries.begin();
      LineIterator l1 = ++l0;
      for (; l1 != boundaries.end(); ++l0, ++l1)
      {
        Vector2<float> vertex;
#if NDEBUG
        LineIntersectLineUnique(*l0, *l1, &vertex);
#else
        const bool isectRes = LineIntersectLineUnique(*l0, *l1, &vertex);
        assert(isectRes);
#endif
        vertices.push_back(vertex);
      }
    }
    // Compute polygon area.
    float area = 0.0f;
    {
      typedef std::vector<Vector2<float> >::const_iterator VertexIterator;
      VertexIterator p0 = vertices.begin();
      VertexIterator p1 = ++p0;
      for (; p1 != vertices.end(); ++p0, ++p1)
      {
        area += (p0->x * p1->y) - (p0->y * p1->x);
      }
      // Integrate last point and divide by two.
      area += (vertices.front().x * p0->y) - (vertices.front().y * p0->x);
      area *= 0.5f;
    }
    // Add area to the player's score.
    assert((s_i->color >= 0) && (s_i->color < static_cast<int>(scores->size())));
    scores->at(s_i->color) += area;
  }
}

}
}
