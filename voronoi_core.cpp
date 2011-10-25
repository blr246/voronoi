#include "voronoi_core.h"
#include <algorithm>
#include <limits>
#include <numeric>
#include <assert.h>

// For Python debugging.
#include <sstream>
#include <string>
#include <iostream>

namespace hps
{
namespace voronoi
{

typedef Voronoi::FloatType FloatType;

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

void InitBoardBoundaries(std::vector<Line<FloatType> >* edges)
{
  assert(edges);
  // Bottom.
  {
    const Vector2<FloatType> boundaryPt(0.0f, 0.0f);
    const Vector2<FloatType> dir(1.0f, 0.0f);
    edges->push_back(Line<FloatType>(boundaryPt, dir));
  }
  // Right.
  {
    const Vector2<FloatType> boundaryPt(1.0f, 0.0f);
    const Vector2<FloatType> dir(0.0f, 1.0f);
    edges->push_back(Line<FloatType>(boundaryPt, dir));
  }
  // Top.
  {
    const Vector2<FloatType> boundaryPt(1.0f, 1.0f);
    const Vector2<FloatType> dir(-1.0f, 0.0f);
    edges->push_back(Line<FloatType>(boundaryPt, dir));
  }
  // Left.
  {
    const Vector2<FloatType> boundaryPt(0.0f, 1.0f);
    const Vector2<FloatType> dir(0.0f, -1.0f);
    edges->push_back(Line<FloatType>(boundaryPt, dir));
  }
}
}

Voronoi::Voronoi(const int players, const int stonesPerPlayer,
                 const BoardSize& boardSize)
: m_players(players),
  m_stonesPerPlayer(stonesPerPlayer),
  m_stonesPlayed(),
  m_stonesPlayedNorm(),
  m_board(Vector2<int>(0, 0), Vector2<int>(boardSize.x, boardSize.y)),
  m_scoreData(stonesPerPlayer)
{}

void Voronoi::InitBoard(std::string buffer, std::string start, std::string end)
{
  //std::cout << "Initializing Board..." << std::endl;
  m_stonesPlayed.clear();
  size_t prev = 0;
  size_t next = buffer.find_first_of("\n");
  buffer = buffer.substr(buffer.find(start)+start.length());
  while(next!=std::string::npos)
  {
    //std::cout << "reading coordinates..." <<std::endl; 
    std::stringstream ss;
    int player;
    int x;
    int y;
    std::string sep;
    std::string buf = buffer.substr(prev,next-prev);
    if(buf.length()>0)
    {
      ss << buf;
      ss >> player >> sep >> x >> y;
      Vector2<int> pos(x,y); 
      Stone stone(player,pos);
      //std::cout << "player: " << player << ", x: " << x << ", y: " << y <<std::endl;
      Play(stone);
    }
    prev = next+1;
    next = buffer.find_first_of("\n",next+1);
  }
  //std::cout << "done initializing board..." <<std::endl;
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
  const StoneNormalized::Position posNorm(stone.pos.x / static_cast<FloatType>(m_board.maxs.x),
                                          stone.pos.y / static_cast<FloatType>(m_board.maxs.y));
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
  inline bool operator()(const Vector2<FloatType>& v0, const Vector2<FloatType>& v1) const
  {
#if !NDEBUG
    const FloatType kUnitVecLenSqBound = 1.0e-6f;
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
      const Vector2<FloatType> ref(Vector2<FloatType>(positiveX ? 1.0f : -1.0f, 0.0f));
      const FloatType cosTheta_v0 = Vector2Dot(v0, ref);
      const FloatType cosTheta_v1 = Vector2Dot(v1, ref);
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
  inline bool operator()(const Line<FloatType>& a, const Line<FloatType>& b) const
  {
    static UnitVectorFastAngleSort s_fastAngleSort;
    return s_fastAngleSort(a.dir, b.dir);
  }
};

/// <summary> Test that two vectors are nearly equal. </summary>
struct Vector2NearlyEqual
{
  Vector2NearlyEqual(const FloatType errSq_) : errSq(errSq_) {}
  inline bool operator()(const Vector2<FloatType>& a, const Vector2<FloatType>& b) const
  {
    const FloatType testErrSq = Vector2LengthSq(a - b);
    return testErrSq <= errSq;
  }
  FloatType errSq;
};

/// <summary> Helper to find lines with the same direction. </summary>
struct LineDirNearlyEqual
{
  LineDirNearlyEqual(const FloatType errSq) : vector2NearlyEqual(errSq) {}
  inline bool operator()(const Line<FloatType>& a, const Line<FloatType>& b) const
  {
    return vector2NearlyEqual(a.dir, b.dir);
  }
  Vector2NearlyEqual vector2NearlyEqual;
};

void PrettyPrintPyPlot(std::ostream& stream,
                       const std::vector<Voronoi::StoneNormalized>& stones,
                       const std::vector<Line<FloatType> >& edges,
                       const std::vector<Vector2<FloatType> >& vertices)
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
    const Line<FloatType>& edge = edges[edgeIdx]; 
//    const Vector2<FloatType> edgeStart = edge.p0 - edge.dir;
//    const Vector2<FloatType> edgeEnd = edge.p0 + edge.dir;
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
    const Vector2<FloatType>& vertex = vertices[vertexIdx]; 
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
  const FloatType boardTotalArea = static_cast<FloatType>(AxisAlignedBoxArea(m_board));
  // Return on base cases.
  if (m_stonesPlayed.size() <= 1)
  {
    // One stone is a special case. No combinations.
    if (1 == m_stonesPlayed.size())
    {
      scores->at(0) = boardTotalArea;
    }
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
  std::vector<Line<FloatType> >& candidateEdges = m_scoreData.candidateEdges;
  const std::vector<Line<FloatType> >& boardEdges = m_scoreData.boardEdges;
  std::vector<Vector2<FloatType> >& vertices = m_scoreData.vertices;
  typedef std::vector<StoneNormalized>::const_iterator StoneIterator;
  for (StoneIterator s_i = m_stonesPlayedNorm.begin();
//       s_i != m_stonesPlayedNorm.end() - 1; // reissb -- 20111023 -- Cannot use this optimization
//                                            //   until the scoring is verified correct
       s_i != m_stonesPlayedNorm.end();
       ++s_i)
  {
    // Clear score data.
    m_scoreData.Reset();
    // Find candidate edges.
    for (StoneIterator s_j = m_stonesPlayedNorm.begin();
         s_j != m_stonesPlayedNorm.end();
         ++s_j)
    {
      if (s_i == s_j)
      {
        continue;
      }
      const Vector2<FloatType> p0 = 0.5f * (s_i->pos + s_j->pos);
      const Vector2<FloatType> dirDenorm(s_j->pos - s_i->pos);
      const Vector2<FloatType> dir = (1.0f / Vector2Length(dirDenorm)) * dirDenorm;
      candidateEdges.push_back(Line<FloatType>(p0, Vector2Rotate90(dir)));
    }
    // reissb -- 20111022 -- The shape is always convex, so we don't need to
    //   worry about the sorting producing out-of-order edges.
    // Sort the boundaries.
    std::sort(candidateEdges.begin(), candidateEdges.end(), detail::LineFastAngleSort());
    // Eliminate edges that have the same direction as existing edges. Keep only
    // the closest edge.
    {
      typedef std::vector<Line<FloatType> >::iterator EdgeIterator;
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
          const FloatType distA = Vector2LengthSq(edgeA->p0 - s_i->pos);
          const FloatType distB = Vector2LengthSq(edgeB->p0 - s_i->pos);
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
    {
      int edgeStartIdx = 0;
      typedef std::vector<Line<FloatType> >::const_iterator EdgeIterator;
      for (; edgeStartIdx < static_cast<int>(candidateEdges.size()); ++edgeStartIdx)
      {
        const Vector2<FloatType>& ecP0 = candidateEdges[edgeStartIdx].p0;
        const Vector2<FloatType> dirDenorm = ecP0 - s_i->pos;
        if (detail::Vector2NearlyEqual(1.0e-6f)(dirDenorm, Vector2<FloatType>(0.0f, 0.0f)))
        {
          break;
        }
        const Vector2<FloatType> dir = (1.0f / Vector2Length(dirDenorm)) * dirDenorm;
        const Line<FloatType> edgeNormal(ecP0, dir);
        // Intersect with all other lines to find closest.
        const Line<FloatType>* closestEdge = NULL;
        FloatType closestEdgeDist = std::numeric_limits<FloatType>::infinity();
        for (EdgeIterator edge = candidateEdges.begin();
             edge != candidateEdges.end();
             ++edge)
        {
          Vector2<FloatType> isect;
          const bool isectRes = LineIntersectLineUnique(*edge, edgeNormal, &isect);
          if (isectRes)
          {
            Vector2<FloatType> stoneToLine(isect - s_i->pos);
            const FloatType edgeDist = Vector2Length(stoneToLine);
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
      // Transform the start to the front of the vector.
      assert(edgeStartIdx < static_cast<int>(candidateEdges.size()));
      if (edgeStartIdx > 0)
      {
        std::vector<Line<FloatType> > newEdgeList(candidateEdges.size());
        std::copy(candidateEdges.begin() + edgeStartIdx, candidateEdges.end(), newEdgeList.begin());
        std::copy(candidateEdges.begin(), candidateEdges.begin() + edgeStartIdx,
                  newEdgeList.end() - edgeStartIdx);
        candidateEdges = newEdgeList;
      }
    }
    // Find intersection of all consecutive boundaries to get polygon points.
    {
      static const AxisAlignedBox<FloatType> s_normBoard(Vector2<FloatType>(-1e-6f, -1e-6f),
                                                     Vector2<FloatType>(1.0f + 1e-6f, 1.0f + 1e-6f));
      typedef std::vector<Line<FloatType> >::const_iterator EdgeIterator;
      int edgeCount = static_cast<int>(candidateEdges.size());
      int maxEdgeIdx = edgeCount - 1;
      int edgeFrontier = 0;
      for (;; ++edgeFrontier)
      {
        assert(edgeCount == static_cast<int>(candidateEdges.size()));
        assert(maxEdgeIdx == edgeCount - 1);
        assert(static_cast<int>(vertices.size()) == edgeFrontier);
        const Line<FloatType>& a = candidateEdges[edgeFrontier];
        Vector2<FloatType> vertex;
        bool tryWall = false;
        if (edgeFrontier < maxEdgeIdx)
        {
          int edgeIdxNext = (edgeFrontier + 1) % edgeCount;
          const Line<FloatType>& b = candidateEdges[edgeIdxNext];
          const bool isectRes = LineIntersectLineUnique(a, b, &vertex);
          // Is the vertex within the board region?
          if (isectRes)
          {
            if (AxisAlignedBoxContains(s_normBoard, vertex))
            {
              bool acceptVertex = true;
              bool tossLastVertex = false;
              // See if the last edge is on the intersection.
              const Vector2<FloatType> edgeDirDenorm = vertex - a.p0;
              if (detail::Vector2NearlyEqual(1.0e-6f)(edgeDirDenorm, Vector2<FloatType>(0.0f, 0.0f)))
              {
                tossLastVertex = true;
              }
              else
              {
                // See if in line with edge.
                const Vector2<FloatType> edgeDir = (1.0f / Vector2Length(edgeDirDenorm)) * edgeDirDenorm;
                acceptVertex = detail::Vector2NearlyEqual(1.0e-6f)(edgeDir, a.dir);
                if (acceptVertex && vertices.size() > 0)
                {
                  // See if duplicate.
                  if (detail::Vector2NearlyEqual(1.0e-6f)(vertex, vertices.back()))
                  {
                    tossLastVertex = true;
                  }
                  // See if proper direction with last vertex.
                  else
                  {
                    const Vector2<FloatType> polyEdgeDenorm = vertex - vertices.back();
                    const Vector2<FloatType> polyEdge = (1.0f / Vector2Length(polyEdgeDenorm)) *
                                                    polyEdgeDenorm;
                    acceptVertex = detail::Vector2NearlyEqual(1.0e-6f)(polyEdge, a.dir);
                  }
                }
              }
              if (tossLastVertex && (vertices.size() > 0))
              {
                vertices.pop_back();
                --maxEdgeIdx;
                --edgeCount;
                candidateEdges.erase(candidateEdges.begin() + edgeFrontier);
                --edgeFrontier;
              }
              if (acceptVertex)
              {
                vertices.push_back(vertex);
              }
              else
              {
                tryWall = true;
              }
            }
            // Not on board.
            else
            {
              if (Vector2Dot(a.dir, b.dir) > 0)
              {
                const FloatType distSqA = Vector2LengthSq(a.p0 - s_i->pos);
                const FloatType distSqB = Vector2LengthSq(b.p0 - s_i->pos);
                if (distSqA < distSqB)
                {
                  candidateEdges.erase(candidateEdges.begin() + edgeIdxNext);
                }
                else
                {
                  candidateEdges.erase(candidateEdges.begin() + edgeFrontier);
                  if (vertices.size() > 0)
                  {
                    --edgeFrontier;
                    vertices.pop_back();
                  }
                }
                --maxEdgeIdx;
                --edgeCount;
              }
              tryWall = true;
            }
          }
          // Need to try a wall.
          else
          {
            if (Vector2Dot(a.dir, b.dir) > 0.99f)
            {
              const FloatType distSqA = Vector2LengthSq(a.p0 - s_i->pos);
              const FloatType distSqB = Vector2LengthSq(b.p0 - s_i->pos);
              if (distSqA < distSqB)
              {
                candidateEdges.erase(candidateEdges.begin() + edgeIdxNext);
              }
              else
              {
                candidateEdges.erase(candidateEdges.begin() + edgeFrontier);
                if (vertices.size() > 0)
                {
                  --edgeFrontier;
                  vertices.pop_back();
                }
              }
              --maxEdgeIdx;
              --edgeCount;
            }
            tryWall = true;
          }
        }
        // When at the end of the list, find a wall to intersect.
        if ((edgeFrontier == maxEdgeIdx) || tryWall)
        {
          // Locate a board edge. Find edge just after this one in order.
          int boardEdgeIdx = 0;
          {
            for (; boardEdgeIdx < static_cast<int>(boardEdges.size()); ++boardEdgeIdx)
            {
              if (detail::LineFastAngleSort()(a, boardEdges[boardEdgeIdx]))
              {
                break;
              }
            }
          }
          // Intersect with the board edge.
          for (;; ++boardEdgeIdx)
          {
            boardEdgeIdx %= static_cast<int>(boardEdges.size());
            const Line<FloatType>& boardEdge = boardEdges[boardEdgeIdx];
            const bool isectRes = LineIntersectLineUnique(a, boardEdge, &vertex);
            // Is the vertex within the board region?
            if (isectRes && AxisAlignedBoxContains(s_normBoard, vertex))
            {
              // Make sure that the vertex is in line with the edge. If it is on the
              // edge point then we'll take it unless it is a duplicate.
              bool acceptVertex = true;
              const Vector2<FloatType> edgeDirDenorm = vertex - a.p0;
              if (!detail::Vector2NearlyEqual(1.0e-6f)(edgeDirDenorm, Vector2<FloatType>(0.0f, 0.0f)))
              {
                const Vector2<FloatType> edgeDir = (1.0f / Vector2Length(edgeDirDenorm)) * edgeDirDenorm;
                acceptVertex = detail::Vector2NearlyEqual(1.0e-6f)(edgeDir, a.dir);
              }
              if (acceptVertex && (vertices.size() > 0))
              {
                // See if this is a dup.
                if (detail::Vector2NearlyEqual(1.0e-6f)(vertex, vertices.back()))
                {
                  vertices.pop_back();
                  --maxEdgeIdx;
                  --edgeCount;
                  candidateEdges.erase(candidateEdges.begin() + edgeFrontier);
                  --edgeFrontier;
                }
              }
              if (acceptVertex)
              {
                candidateEdges.insert(candidateEdges.begin() + edgeFrontier + 1, boardEdge);
                ++edgeCount;
                ++maxEdgeIdx;
                vertices.push_back(vertex);
                break;
              }
            }
          }
        }
        // See if shape can be closed.
        if ((vertices.size() == candidateEdges.size() - 1) && (candidateEdges.size() > 2))
        {
          bool shapeClosed = false;
          Vector2<FloatType> lastVertex;
          const bool isectRes = LineIntersectLineUnique(candidateEdges.back(),
                                                        candidateEdges.front(), &lastVertex);
          // Is the vertex within the board region?
          if (isectRes && AxisAlignedBoxContains(s_normBoard, lastVertex))
          {
            // Does the final edge match the first edge?
            const Vector2<FloatType> edgeDirDenorm = vertices.front() - lastVertex;
            const Vector2<FloatType> edgeDir = (1.0f / Vector2Length(edgeDirDenorm)) * edgeDirDenorm;
            shapeClosed = detail::Vector2NearlyEqual(1.0e-6f)(edgeDir, candidateEdges.front().dir);
          }
          if (shapeClosed)
          {
            // If this vertex is a duplicate, then eliminate the last edge.
            if (detail::Vector2NearlyEqual(1.0e-6f)(lastVertex, vertices.back()))
            {
              candidateEdges.pop_back();
              vertices.pop_back();
            }
            vertices.push_back(lastVertex);
            assert(vertices.size() == candidateEdges.size());
            break;
          }
        }
      }
    }
    // reissb -- 20111023 -- Export as python script.
    //detail::PrettyPrintPyPlot(std::cout, m_stonesPlayedNorm, candidateEdges, vertices);
    // Compute polygon normalize area.
    FloatType normArea = 0.0f;
    {
      typedef std::vector<Vector2<FloatType> >::const_iterator VertexIterator;
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
//  // reissb -- 20111023 -- This optimization cannot be used until the scoring is verified correct.
//  // The remainder of the score goes to the last player to play a stone.
//  const FloatType scoreTotal = std::accumulate(scores->begin(), scores->end(),
//                                               static_cast<FloatType>(0));
//  assert((scoreTotal > 0) && (scoreTotal < boardTotalArea));
//  assert((m_stonesPlayed.back().player >= 0) &&
//         (m_stonesPlayed.back().player < static_cast<int>(scores->size())));
//  scores->at(m_stonesPlayed.back().player) += boardTotalArea - scoreTotal;
}

Voronoi::ScoreData::ScoreData(const int players)
: candidateEdges(players),
  boardEdges(),
  vertices(players)
{
  // Initialize board edges.
  detail::InitBoardBoundaries(&boardEdges);
}

}
}
