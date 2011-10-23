#include "voronoi_core.h"
#include <algorithm>
#include <limits>
#include <assert.h>

// For Python debugging.
#include <sstream>
#include <string>
#include <iostream>

namespace hps
{
namespace voronoi
{

namespace detail
{
/// <summary> Test that a stone has the same position as a base stone. </summary>
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
///   <para> This is a strict weak ordering. </summary>
/// </remarks>
struct UnitVectorFastAngleSort
{
  inline bool operator()(const Vector2<float>& v0, const Vector2<float>& v1) const
  {
#if !NDEBUG
    const float kUnitVecLenSqBound = 1.0e-6f;
    assert(fabs(1.0f - Vector2LengthSq(v0)) < kUnitVecLenSqBound);
    assert(fabs(1.0f - Vector2LengthSq(v1)) < kUnitVecLenSqBound);
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
        if (v0.x > 0)
        {
          // Is the other reference?
          if (0 == v1.y)
          {
            // Use x-then-y keys to enforce strict weak ordering.
            return (v0.x > v1.x) || (v0.y > v1.y);
          }
          else
          {
            return true;
          }
        }
      }
      // Second on x-axis?
      if (0 == v1.y)
      {
        ++onX[1];
        // This is the reference axis.
        if (v1.x > 0)
        {
          return false;
        }
        // Both are not reference.
        else if (1 == onX[0])
        {
          // Use x-then-y keys to enforce strict weak ordering.
          return (v0.x > v1.x) || (v0.y > v1.y);
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

/// <summary> Test that two vectors are nearly equal. </summary>
struct Vector2NearlyEqual
{
  Vector2NearlyEqual(const float errSq_) : errSq(errSq_) {}
  inline bool operator()(const Vector2<float>& a, const Vector2<float>& b) const
  {
    const float testErrSq = Vector2LengthSq(a - b);
    return testErrSq <= errSq;
  }
  float errSq;
};

/// <summary> Helper to find lines with the same direction. </summary>
struct LineDirNearlyEqual
{
  LineDirNearlyEqual(const float errSq) : vector2NearlyEqual(errSq) {}
  inline bool operator()(const Line<float>& a, const Line<float>& b) const
  {
    return vector2NearlyEqual(a.dir, b.dir);
  }
  Vector2NearlyEqual vector2NearlyEqual;
};

void PrettyPrintPyPlot(std::ostream& stream,
                       const std::vector<Voronoi::StoneNormalized>& stones,
                       const std::vector<Line<float> >& edges,
                       const std::vector<Vector2<float> >& vertices)
{
  std::stringstream ss;

  // Print script header.
  stream
    << "import matplotlib.pyplot as plt" << std::endl
    << std::endl
    << "# Plot the board." << std::endl
    << "boardPts = [(0.,0.), (1.,0.), (1.,1.), (0.,1.), (0., 0.)]" << std::endl
    << "plt.plot([pt[0] for pt in boardPts], [pt[1] for pt in boardPts], 'k--', lw=2)" << std::endl;

  // Print stone locations.
  const char colors[] = "bgrcmyk";
  enum { NumColors = (sizeof(colors) / sizeof(colors[0])), };
  ss.str(std::string());
  ss.clear();
  ss << "# Print stones." << std::endl;
  for (int stoneIdx = 0; stoneIdx < static_cast<int>(stones.size()); ++stoneIdx)
  {
    const Voronoi::StoneNormalized& stone = stones[stoneIdx];
    const char color = colors[stone.player];
    ss << "plt.plot([" << stone.pos.x << "], [" << stone.pos.y << "], "
       << "'" << color << ".')" << std::endl;
  }
  stream << ss.str() << std::endl << std::endl;

  // Print edges.
  ss.str(std::string());
  ss.clear();
  ss << "# Print edges." << std::endl;
  for (int edgeIdx = 0; edgeIdx < static_cast<int>(edges.size()); ++edgeIdx)
  {
    const Line<float>& edge = edges[edgeIdx]; 
//    const Vector2<float> edgeStart = edge.p0 - edge.dir;
//    const Vector2<float> edgeEnd = edge.p0 + edge.dir;
//    ss << "plt.plot([" << edgeStart.x << ", " << edgeEnd.x << "], "
//       << "[" << edgeStart.y << ", " << edgeEnd.y << "], 'k-')" << std::endl;
    ss << "plt.arrow(" << edge.p0.x << ", " << edge.p0.y << ", "
       << 0.375f * edge.dir.x << ", "
       << 0.375f * edge.dir.y << ", "
       << "head_width=.025)" << std::endl;
  }
  stream << ss.str() << std::endl << std::endl;

  // Print vertices.
  ss.str(std::string());
  ss.clear();
  ss << "# Print vertices." << std::endl;
  for (int vertexIdx = 0; vertexIdx < static_cast<int>(vertices.size()); ++vertexIdx)
  {
    const Vector2<float>& vertex = vertices[vertexIdx]; 
    ss << "plt.plot([" << vertex.x << "], " << "[" << vertex.y << "], 'ko')" << std::endl;
  }
  stream << ss.str() << std::endl << std::endl;

  // Print script footer.
  stream
    << "# Set axis limits and plot." << std::endl
    << "VIEW_OFFSET = 0.0625" << std::endl
    << "plt.gca().set_xlim([-VIEW_OFFSET, 1. + VIEW_OFFSET])" << std::endl
    << "plt.gca().set_ylim([-VIEW_OFFSET, 1. + VIEW_OFFSET])" << std::endl
    << "plt.show()" << std::endl
    << std::endl;
}
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
      const Vector2<float> dirDenorm(s_j->pos - s_i->pos);
      const Vector2<float> dir = (1.0f / Vector2Length(dirDenorm)) * dirDenorm;
      candidateEdges.push_back(Line<float>(p0, Vector2Rotate90(dir)));
    }
    // Board boundaries are just virtual stones.
    {
      // Bottom.
      {
        const Vector2<float> boundaryPt(s_i->pos.x, 0);
        const Vector2<float> dir(0.0f, -1.0f);
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
      // Right.
      {
        const Vector2<float> boundaryPt(1.0f, s_i->pos.y);
        const Vector2<float> dir(1.0f, 0.0f);
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
      // Top.
      {
        const Vector2<float> boundaryPt(s_i->pos.x, 1.0f);
        const Vector2<float> dir(0.0f, 1.0f);
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
      // Left.
      {
        const Vector2<float> boundaryPt(0.0f, s_i->pos.y);
        const Vector2<float> dir(-1.0f, 0.0f);
        candidateEdges.push_back(Line<float>(boundaryPt, Vector2Rotate90(dir)));
      }
    }
    // reissb -- 20111022 -- The shape is always convex, so we don't need to
    //   worry about the sorting producing out-of-order edges.
    // Sort the boundaries.
    std::sort(candidateEdges.begin(), candidateEdges.end(), detail::LineFastAngleSort());
    // Eliminate edges that have the same direction as existing edges. Keep only
    // the closest edge.
    {
      typedef std::vector<Line<float> >::const_iterator EdgeIterator;
      for (;;)
      {
        // Find next edge with sime direction.
        EdgeIterator edgeSameDir = std::adjacent_find(candidateEdges.begin(),
                                                      candidateEdges.end(),
                                                      detail::LineDirNearlyEqual(1.0e-6f));
        if (edgeSameDir == candidateEdges.end())
        {
          break;
        }
        else
        {
          // Retain edges and advance pointer.
          const EdgeIterator edgeA = edgeSameDir;
          const EdgeIterator edgeB = ++edgeSameDir;
          // Keep the closer one.
          const float distA = Vector2LengthSq(edgeA->p0 - s_i->pos);
          const float distB = Vector2LengthSq(edgeB->p0 - s_i->pos);
          if (distA < distB)
          {
            candidateEdges.erase(edgeB);
          }
          else
          {
            candidateEdges.erase(edgeA);
          }
        }
      }
    }
    // Find start index for vertex creation. Want an edge that is closest to the stone.
    int edgeStartIdx = 0;
    {
      typedef std::vector<Line<float> >::const_iterator EdgeIterator;
      for (; edgeStartIdx < static_cast<int>(candidateEdges.size()); ++edgeStartIdx)
      {
        const Line<float>& edgeCheck = candidateEdges[edgeStartIdx];
        const Vector2<float>& ecDir = edgeCheck.dir;
        const Vector2<float>& ecP0 = edgeCheck.p0;
        // Cannot be a board edge.
        {
          const bool lt = ((0.0f == ecP0.x) && (Vector2<float>( 0.0f, -1.0f) == ecDir));
          const bool rt = ((1.0f == ecP0.x) && (Vector2<float>( 0.0f,  1.0f) == ecDir));
          const bool tp = ((1.0f == ecP0.y) && (Vector2<float>(-1.0f,  0.0f) == ecDir));
          const bool bt = ((0.0f == ecP0.y) && (Vector2<float>( 1.0f,  0.0f) == ecDir));
          if (lt || rt || tp || bt)
          {
            continue;
          }
        }
        const Vector2<float> dirDenorm = ecP0 - s_i->pos;
        if (detail::Vector2NearlyEqual(1.0e-6f)(dirDenorm, Vector2<float>(0.0f, 0.0f)))
        {
          break;
        }
        const Vector2<float> dir = (1.0f / Vector2Length(dirDenorm)) * dirDenorm;
        const Line<float> edgeNormal(ecP0, dir);
        // Intersect with all other lines to find closest.
        const Line<float>* closestEdge = NULL;
        float closestEdgeDist = std::numeric_limits<float>::infinity();
        for (EdgeIterator edge = candidateEdges.begin();
             edge != candidateEdges.end();
             ++edge)
        {
          // Cannot be a board edge.
          {
            const bool lt = ((0.0f == edge->p0.x) && (Vector2<float>( 0.0f, -1.0f) == edge->dir));
            const bool rt = ((1.0f == edge->p0.x) && (Vector2<float>( 0.0f,  1.0f) == edge->dir));
            const bool tp = ((1.0f == edge->p0.y) && (Vector2<float>(-1.0f,  0.0f) == edge->dir));
            const bool bt = ((0.0f == edge->p0.y) && (Vector2<float>( 1.0f,  0.0f) == edge->dir));
            if (lt || rt || tp || bt)
            {
              continue;
            }
          }
          Vector2<float> isect;
          const bool isectRes = LineIntersectLineUnique(*edge, edgeNormal, &isect);
          if (isectRes)
          {
            Vector2<float> stoneToLine(isect - s_i->pos);
            const float edgeDist = Vector2Length(stoneToLine);
            if (edgeDist < closestEdgeDist)
            {
              closestEdgeDist = edgeDist;
              closestEdge = &*edge;
            }
          }
        }
        // See if we found an edge for stone s_i.
        if (edgeNormal.p0 == closestEdge->p0)
        {
          break;
        }
      }
    }
    assert(edgeStartIdx < static_cast<int>(candidateEdges.size()));
    // Find intersection of all consecutive boundaries to get polygon points.
    {
      static const AxisAlignedBox<float> s_normBoard(Vector2<float>(-1e-3f, -1e-3f),
                                                     Vector2<float>(1.0f + 1e-3f, 1.0f + 1e-3f));
      typedef std::vector<Line<float> >::const_iterator EdgeIterator;
      int edgeCount = static_cast<int>(candidateEdges.size());
      int edgeIdxA = edgeStartIdx;
      int edgeIdxB = (edgeStartIdx + 1) % edgeCount;
      for (;; edgeIdxA = (edgeIdxA + 1) % edgeCount, edgeIdxB = (edgeIdxB + 1) % edgeCount)
      {
        assert(edgeIdxA != edgeIdxB);
        assert(((edgeIdxA + 1) % edgeCount) == edgeIdxB);
        if (edgeIdxB == edgeStartIdx)
        {
          break;
        }
        const Line<float>& a = candidateEdges[edgeIdxA];
        const Line<float>& b = candidateEdges[edgeIdxB];
        Vector2<float> vertex;
        const bool isectRes = LineIntersectLineUnique(a, b, &vertex);
        // Is the vertex within the board region?
        if (isectRes && AxisAlignedBoxContains(s_normBoard, vertex))
        {
          vertices.push_back(vertex);
        }
        // Vertex is bad so remove the unused edge and back up.
        else
        {
          candidateEdges.erase(candidateEdges.begin() + edgeIdxB);
          --edgeCount;
          if (edgeCount == edgeIdxA)
          {
            --edgeIdxA;
          }
          --edgeIdxA;
          if (edgeIdxA < 0)
          {
            edgeIdxA = edgeCount - 1;
          }
          if (edgeIdxB < edgeStartIdx)
          {
            --edgeStartIdx;
          }
          edgeIdxB = (edgeIdxA + 1) % edgeCount;
          continue;
        }
      }
      // Final edge. This must close the shape.
      bool shapeClosed = false;
      do
      {
        assert(vertices.size() == (candidateEdges.size() - 1));
        assert(edgeIdxA != edgeIdxB);
        assert(((edgeIdxA + 1) % edgeCount) == edgeIdxB);
        const Line<float>& a = candidateEdges[edgeIdxA];
        const Line<float>& b = candidateEdges[edgeIdxB];
        Vector2<float> vertex;
        const bool isectRes = LineIntersectLineUnique(a, b, &vertex);
        if (isectRes && AxisAlignedBoxContains(s_normBoard, vertex))
        {
          // Make sure that the edge defined by this vertex is the same as the first edge.
          Vector2<float> finalEdge = vertices[0] - vertex;
          Vector2<float> finalEdgeDir = (1.0f / Vector2Length(finalEdge)) * finalEdge;
          if (detail::Vector2NearlyEqual(1.0e-6f)(finalEdgeDir, candidateEdges[edgeStartIdx].dir))
          {
            vertices.push_back(vertex);
            shapeClosed = true;
          }
        }
        if (!shapeClosed)
        {
          // Vertex is bad so remove the unused edge and back up.
          candidateEdges.erase(candidateEdges.begin() + edgeIdxA);
          --edgeCount;
          if (edgeIdxA < edgeStartIdx)
          {
            --edgeStartIdx;
            if (edgeStartIdx < 0)
            {
              edgeStartIdx = 0;
            }
          }
          assert(edgeStartIdx >= 0);
          edgeIdxB = edgeStartIdx;
          edgeIdxA = edgeIdxB - 1;
          if (edgeIdxA < 0)
          {
            edgeIdxA = edgeCount - 1;
          }
          vertices.pop_back();
        }
      } while (!shapeClosed);
    }
    // Remove duplicate vertices.
    vertices.erase(std::unique(vertices.begin(), vertices.end(),
                               detail::Vector2NearlyEqual(1.0e-6f)), vertices.end());
    if (vertices.front() == vertices.back())
    {
      vertices.pop_back();
    }
    // reissb -- 20111023 -- Export as python script.
    //detail::PrettyPrintPyPlot(std::cout, m_stonesPlayedNorm, candidateEdges, vertices);
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
