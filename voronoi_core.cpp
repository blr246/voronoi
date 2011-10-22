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
struct StonePositionsEqual
{
  StonePositionsEqual(const Stone& stone_) : stone(&stone_) {}
  inline bool operator()(const Stone& testStone) const
  {
    return (testStone.pos == stone->pos);
  }
  const Stone* stone;
};
}

bool Voronoi::Play(const Stone& stone)
{
  // The move is valid only if it was not made already.
  if (std::find_if(m_stonesPlayed.begin(),
                   m_stonesPlayed.end(),
                   detail::StonePositionsEqual(stone)) != m_stonesPlayed.end())
  {
    return false;
  }
  // Stone within bounds.
  assert((stone.pos.x >= m_board.mins.x) && (stone.pos.y >= m_board.mins.y));
  assert((stone.pos.x <= m_board.maxs.x) && (stone.pos.y <= m_board.maxs.y));
  // Push the stone and the normalized stone.
  m_stonesPlayed.push_back(stone);
  const StoneNormalized::Position posNorm(stone.pos.x / static_cast<float>(m_board.maxs.x),
                                          stone.pos.y / static_cast<float>(m_board.maxs.y));
  m_stonesPlayedNorm.push_back(StoneNormalized(stone.player, posNorm));
  assert(m_stonesPlayed.size() == m_stonesPlayedNorm.size());
  return true;
}

void Voronoi::Undo()
{
  if (!m_stonesPlayed.empty())
  {
    m_stonesPlayed.pop_back();
    m_stonesPlayedNorm.pop_back();
  }
  assert(m_stonesPlayed.size() == m_stonesPlayedNorm.size());
}

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
    if (((v0.y < 0) && (v1.y > 0)) || ((v0.y > 0) && (v1.y < 0)))
    {
      return v0.y > v1.y;
    }
    // If at least one on the x-axis.
    else if (0 == (v0.y * v1.y))
    {
      int onX[2] = {0};
      // First on x-axis?
      if (0 == v0.y)
      {
        ++onX[0];
        // This is the reference axis so it comes first.
        // Arbirary if both reference.
        if (v0.x > 0)
        {
          return true;
        }
      }
      // Second on x-axis?
      if (0 == v1.y)
      {
        ++onX[1];
        // This is the reference axis or both are not reference.
        // Arbitrary when both are not reference.
        if ((v1.x > 0) || (1 == onX[0]))
        {
          return false;
        }
      }
      // First not the reference on x-axis?
      if (1 == onX[0])
      {
        return (v1.y < 0);
      }
      // Second not the reference on x-axis?
      else
      {
        return (v0.y > 0);
      }
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
  // Initialize scores.
  scores->assign(m_players, 0);
  // One stone is a special case. No combinations.
  const float boardTotalArea = static_cast<float>(AxisAlignedBoxArea(m_board));
  if (1 == m_stonesPlayed.size())
  {
    scores->at(0) = boardTotalArea;
    return;
  }

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

  // Compute score for all stones.
  typedef std::vector<StoneNormalized>::const_iterator StoneIterator;
  for (StoneIterator s_i = m_stonesPlayedNorm.begin();
       s_i != m_stonesPlayedNorm.end();
       ++s_i)
  {
    // Clear score data.
    m_scoreData.Reset();
    std::vector<Line<float> >& candidateEdges = m_scoreData.candidateEdges;
    std::vector<Line<float> >& candidateEdgeNormals = m_scoreData.candidateEdgeNormals;
    std::vector<Line<float> >& edges = m_scoreData.edges;
    std::vector<Vector2<float> >& vertices = m_scoreData.vertices;
    // Find candidate edges.
    for (StoneIterator s_j = m_stonesPlayedNorm.begin();
         s_j != m_stonesPlayedNorm.end();
         ++s_j)
    {
      if (s_i == s_j)
      {
        continue;
      }
      const Vector2<float> p0 = 0.5f * (s_i->pos + s_j->pos);
      const Vector2<float> dir(s_j->pos - s_i->pos);
      candidateEdgeNormals.push_back(Line<float>(p0, dir));
      candidateEdges.push_back(Line<float>(p0, Vector2Rotate90(dir)));
    }
    // Board boundaries are just virtual stones.
    {
      // Bottom.
      {
        const Vector2<float> boundaryPt(s_i->pos.x, 0.0f);
        const Vector2<float> dir(0.0f, -1.0f);
        candidateEdgeNormals.push_back(Line<float>(boundaryPt, dir));
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
      // Right.
      {
        const Vector2<float> boundaryPt(1.0f, s_i->pos.y);
        const Vector2<float> dir(1.0f, 0.0f);
        candidateEdgeNormals.push_back(Line<float>(boundaryPt, dir));
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
      // Top.
      {
        const Vector2<float> boundaryPt(s_i->pos.x, 1.0f);
        const Vector2<float> dir(0.0f, 1.0f);
        candidateEdgeNormals.push_back(Line<float>(boundaryPt, dir));
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
      // Left.
      {
        const Vector2<float> boundaryPt(0.0f, s_i->pos.y);
        const Vector2<float> dir(-1.0f, 0.0f);
        candidateEdgeNormals.push_back(Line<float>(boundaryPt, dir));
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
    }
    // reissb -- 20111022 -- The shape is always convex, so we don't need to
    //   worry about the sorting producing out-of-order edges.
    // Sort the boundaries.
    std::sort(candidateEdges.begin(), candidateEdges.end(), detail::LineFastAngleSort());
    // Find intersection of all consecutive boundaries to get polygon points.
    {
      typedef std::vector<Line<float> >::const_iterator LineIterator;
      LineIterator l0 = candidateEdges.begin();
      LineIterator l1 = candidateEdges.begin() + 1;
      for (; l1 != candidateEdges.end(); ++l0, ++l1)
      {
        Vector2<float> vertex;
        const bool isectRes = LineIntersectLineUnique(*l0, *l1, &vertex);
        if (isectRes)
        {
          vertices.push_back(vertex);
        }
      }
      // First and last.
      {
        Vector2<float> vertex;
        const bool isectRes = LineIntersectLineUnique(*l0, candidateEdges.front(), &vertex);
        if (isectRes)
        {
          vertices.push_back(vertex);
        }
      }
    }
    // Remove duplicate vertices.
    vertices.erase(std::unique(vertices.begin(), vertices.end()), vertices.end());
    if (vertices.front() == vertices.back())
    {
      vertices.pop_back();
    }
    // Compute polygon normalize area.
    float normArea = 0.0f;
    {
      typedef std::vector<Vector2<float> >::const_iterator VertexIterator;
      VertexIterator p0 = vertices.begin();
      VertexIterator p1 = vertices.begin() + 1;
      for (; p1 != vertices.end(); ++p0, ++p1)
      {
        normArea += (p0->x * p1->y) - (p0->y * p1->x);
      }
      // Integrate last point and divide by two.
      normArea += (p0->x * vertices.front().y) - (p0->y * vertices.front().x);
      normArea *= 0.5f;
    }
    // Add normalized area to the player's score.
    assert((s_i->player >= 0) && (s_i->player < static_cast<int>(scores->size())));
    scores->at(s_i->player) += normArea * boardTotalArea;
  }
}

//    // Find closest point to stone on edges. If it matches a the edge point,
//    // then the line is a boundary.
//    typedef std::vector<Line<float> >::const_iterator LineIterator;
//    for (LineIterator edgeNormal = candidateEdgeNormals.begin();
//         edgeNormal != candidateEdgeNormals.end();
//         ++edgeNormal)
//    {
//      // Intersect with all other lines to find closest.
//      const Line<float>* closestEdge = NULL;
//      float closestEdgeDist = std::numeric_limits<float>::infinity();
//      for (LineIterator edge = candidateEdges.begin();
//           edge != candidateEdges.end();
//           ++edge)
//      {
//        Vector2<float> isect;
//#if NDEBUG
//        LineIntersectLineUnique(*edge, *edgeNormal, &isect);
//#else
//        const bool isectRes = LineIntersectLineUnique(*edge, *edgeNormal, &isect);
//        assert(isectRes);
//#endif
//        Vector2<float> stoneToLine(isect - s_i->pos);
//        const float edgeDist = Vector2Length(stoneToLine);
//        if (edgeDist < closestEdgeDist)
//        {
//          closestEdgeDist = edgeDist;
//          closestEdge = &*edge;
//        }
//      }
//      // See if we found an edge for stone s_i.
//      if (edgeNormal->p0 == closestEdge->p0)
//      {
//        edges.push_back(*closestEdge);
//      }
//    }

}
}
