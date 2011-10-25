#include "voronoi_core.h"
#include "scoring.h"
#include "voronoi_diagram_generator.h"
#include <deque>
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
}

Voronoi::Voronoi(const int players, const int stonesPerPlayer,
                 const BoardSize& boardSize)
: m_players(players),
  m_stonesPerPlayer(stonesPerPlayer),
  m_stonesPlayed(),
  m_stonesPlayedNorm(),
  m_board(Vector2<int>(0, 0), Vector2<int>(boardSize.x, boardSize.y))
{}

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
  const int dx = m_board.maxs.x - m_board.mins.x;
  const int dy = m_board.maxs.y - m_board.mins.y;
  m_stonesPlayed.push_back(stone);
  const StoneNormalized::Position posNorm(stone.pos.x / static_cast<FloatType>(dx),
                                          stone.pos.y / static_cast<FloatType>(dy));
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
/// <summary> Test that a segment's length is more than the min length. </summary>
template <typename NumericType>
struct SegmentMinLengthSqFilter
{
  SegmentMinLengthSqFilter(const NumericType minLengthSq_) : minLengthSq(minLengthSq_) {}
  inline bool operator()(const Segment<NumericType>& s) const
  {
    const NumericType lengthSq = Vector2LengthSq(s.p1 - s.p0);
    return lengthSq > minLengthSq;
  }
  NumericType minLengthSq;
};

/// <summary> Helper to find board edges in Voronoi polygons. </summary>
template <typename NumericType>
struct BoardEdgeHelper
{
  enum { NumBoardEdges = 4, };

  BoardEdgeHelper(const AxisAlignedBox<NumericType>& board)
  {
    typedef Vector2<NumericType> Vector;
    boardEdges[0] = Line<NumericType>(Vector(board.mins.x, board.mins.y), Vector( 1,  0));
    boardEdges[1] = Line<NumericType>(Vector(board.maxs.x, board.mins.y), Vector( 0,  1));
    boardEdges[2] = Line<NumericType>(Vector(board.maxs.x, board.maxs.y), Vector(-1,  0));
    boardEdges[3] = Line<NumericType>(Vector(board.mins.x, board.maxs.y), Vector( 0, -1));
  }

  /// <summary> Test if the given points falls on the edge with the given index. </summary>
  inline bool EdgeContainsPoint(const int edgeIdx, const Vector2<NumericType>& p)
  {
    // Get vector from edge base to point.
    const Line<NumericType>* edge = boardEdges + edgeIdx;
    const Vector2<NumericType> edgePtToP = p - edge->p0;
    // See if the point is along the edge.
    const NumericType dotAlongEdgeTest = Vector2Dot(edgePtToP, Vector2Rotate90(edge->dir));
    if (fabs(dotAlongEdgeTest) < static_cast<NumericType>(1.0e-6))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /// <summary> Find the edge most counter clockwise that includes the given point. </summary>
  /// <remarks>
  ///   <para> Returns the edge index. </para>
  /// </remarks>
  int FindEdgeCCW(const Vector2<NumericType>& p)
  {
    int retEdgeIdx = -1;
    for (int edgeIdx = 0; edgeIdx < NumBoardEdges; ++edgeIdx)
    {
      if (EdgeContainsPoint(edgeIdx, p))
      {
        retEdgeIdx = edgeIdx;
      }
    }
    return retEdgeIdx;
  }

  /// <summary> Get the next edge index. </summary>
  inline const int NextEdge(const int edgeIdx)
  {
    return (edgeIdx + 1) % NumBoardEdges;
  }

  /// <summary> Return the board edge endpoint traversing in the CCW direction. </summary>
  inline const Vector2<NumericType>& EdgePtCCW(const int edgeIdx)
  {
    assert((edgeIdx >= 0) && (edgeIdx < NumBoardEdges));
    int nextEdgeIdx = NextEdge(edgeIdx);
    const Line<NumericType>& nextEdge = boardEdges[nextEdgeIdx];
    return nextEdge.p0;
  }

  Line<NumericType> boardEdges[NumBoardEdges];
};

template <typename EqType>
struct PtrDerefEquals
  : public std::binary_function<EqType*, EqType*, bool>
{
  inline bool operator()(const EqType* lhs, const EqType* rhs) const
  {
    return *lhs == *rhs;
  }
};
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

  AxisAlignedBox<FloatType> boardNorm(Vector2<FloatType>(0, 0), Vector2<FloatType>(1, 1));
  detail::BoardEdgeHelper<FloatType> boardEdgeHelper(boardNorm);
  const int dx = m_board.maxs.x - m_board.mins.x;
  const int dy = m_board.maxs.y - m_board.mins.y;
  const Vector2<FloatType> halfMinEdge(1 / static_cast<FloatType>(2 * dx),
                                       1 / static_cast<FloatType>(2 * dy));
  const FloatType halfMinEdgeLenSq = Vector2Length(halfMinEdge);
  const detail::SegmentMinLengthSqFilter<FloatType> minLengthSqFilter(halfMinEdgeLenSq);
  const FloatType halfMinEdgeLen = sqrt(halfMinEdgeLenSq);
  const Vector2<FloatType> halfMinEdgeLenVec(halfMinEdgeLen, halfMinEdgeLen);
  AxisAlignedBox<FloatType> boardNormNoEdge(boardNorm.mins + halfMinEdgeLenVec,
                                            boardNorm.maxs - halfMinEdgeLenVec);

  // Run the legacy code to generate the Voronoi diagram.
  VoronoiDiagramGenerator diagram;
  const int numStones = static_cast<int>(m_stonesPlayedNorm.size());
  std::vector<float> xCoords(numStones);
  std::vector<float> yCoords(numStones);
  for (int stoneIdx = 0; stoneIdx < numStones; ++stoneIdx)
  {
    xCoords[stoneIdx] = m_stonesPlayedNorm[stoneIdx].pos.x;
    yCoords[stoneIdx] = m_stonesPlayedNorm[stoneIdx].pos.y;
  }
  const bool res = diagram.generateVoronoi(&xCoords.front(), &yCoords.front(),
                                           numStones, 0.0f, 1.0f, 0.0f, 1.0f);
  assert(res);
  // Extract the segments from the diagram assigned to each stone.
  diagram.resetIterator();
  std::vector<Segment<FloatType> > edges;
  edges.reserve(numStones * numStones);
  Vector2<FloatType> p0;
  Vector2<FloatType> p1;
  typedef std::deque<const Segment<FloatType>*> SegmentQueue;
  std::vector<SegmentQueue> stoneEdges(numStones);
  std::for_each(stoneEdges.begin(), stoneEdges.end(),
                std::mem_fun_ref(&SegmentQueue::clear));
  int stoneIndices[2];
  while (diagram.getNext(p0.x, p0.y, p1.x, p1.y, stoneIndices))
  {
    std::cout << "Edge: "
              << "(" << p0.x << ", " << p0.y << ") -> "
              << "(" << p1.x << ", " << p1.y << ") "
              << " : Stones {" << stoneIndices[0] << " "
              << stoneIndices[1] << "}" << std::endl;
    const Segment<FloatType> segment(p0, p1);
    // Filter min edge length squared based on board size.
    if (minLengthSqFilter(segment))
    {
//      bool onEdge = false;
//      // Don't take segments on the board edge. I'll handle those later.
//      if (!AxisAlignedBoxContains(boardNormNoEdge, segment.p0) &&
//          !AxisAlignedBoxContains(boardNormNoEdge, segment.p1))
//      {
//        const Vector2<FloatType> segDirDenorm = segment.p1 - segment.p0;
//        const Vector2<FloatType> segDir = (1.0f / Vector2Length(segDirDenorm)) * segDirDenorm;
//        // Do deep check.
//        for (int edgeIdx = 0; edgeIdx < boardEdgeHelper.NumBoardEdges; ++edgeIdx)
//        {
//          if (fabs(Vector2Dot(segDir, boardEdgeHelper.boardEdges[0].dir)) < 1.0e-6f)
//          {
//            onEdge |= boardEdgeHelper.EdgeContainsPoint(0, segment.p0);
//          }
//        }
//      }
//      if (!onEdge)
      {
        edges.push_back(segment);
        stoneEdges[stoneIndices[0]].push_back(&edges.back());
        stoneEdges[stoneIndices[1]].push_back(&edges.back());
      }
    }
  }
  // Link the extracted segments to create closed polygons around each stone.
  typedef std::vector<Vector2<FloatType> > VertexList;
  std::vector<VertexList> stoneVertices(numStones);
  typedef std::deque<Segment<FloatType> > LinkedSegmentsQueue;
  LinkedSegmentsQueue linkedSegments;
  for (int stoneIdx = 0; stoneIdx < numStones; ++stoneIdx)
  {
    linkedSegments.clear();
    SegmentQueue* segments = &stoneEdges[stoneIdx];
    assert(segments->size() > 0);
    VertexList& vertices = stoneVertices[stoneIdx];
    // For each segment, find its match in the linked segments.
    //   { (x0, y0) -> (x1, y1) } ==> { (x1, y1) -> (x2, y2) }
    // Want to go counter-clockwise. Need to find an unambiguous edge.
    const int numSegments = static_cast<int>(segments->size());
    for (int segmentIdx = 0; segmentIdx < numSegments; ++segmentIdx)
    {
      const Segment<FloatType>* seg = segments->front();
      segments->pop_front();
      // Find the inverse transformation of the vector along the segment and
      // it to the vector from the start of the segment to the stone.
      const Vector2<FloatType> alongSegDenorm = seg->p1 - seg->p0;
      const Vector2<FloatType> alongSeg = (1.0f / Vector2Length(alongSegDenorm)) * alongSegDenorm;
      const Vector2<FloatType> xAxis(1.0f, 0.0f);
#ifndef NDEBUG
      const Vector2<FloatType> alongSegTForm(
        (alongSeg.x * alongSeg.x) + (alongSeg.y * alongSeg.y),
        (alongSeg.x * alongSeg.y) - (alongSeg.y * alongSeg.x));
      const FloatType errSq = Vector2LengthSq(xAxis - alongSegTForm);
      assert(errSq < 1.0e-6f);
#endif
      const Vector2<FloatType> ptToStone = m_stonesPlayedNorm[stoneIdx].pos - seg->p0;
      const Vector2<FloatType> ptToStoneTform(
        (ptToStone.x * alongSeg.x) + (ptToStone.y * alongSeg.y),
        (ptToStone.x * alongSeg.y) - (ptToStone.y * alongSeg.x));
      // Get the Z-component of the in-plane cross product of the vector from
      // the stone to the segment point transformed by the inverse rotation of
      // the segment and the X-axis. This is simply the Y-component.
      //   (a cross b).z = (a_x*b_y - a_y*b_x)
      // The sign of the Z-component gives the handedness. When using the
      // Nearly parallel?
      if (fabs(ptToStoneTform.y) < 1.0e-6f)
      {
        // Rotate queue to the next segment.
        segments->push_back(seg);
        continue;
      }
      else
      {
        linkedSegments.push_front(*seg);
        if (ptToStoneTform.y > 0)
        {
          std::swap(linkedSegments.front().p0, linkedSegments.front().p1);
        }
        break;
      }
    }
    assert(1 == linkedSegments.size());
    typedef SegmentQueue::const_iterator EdgeIterator;
    for (;;)
    {
      // Link another segment.
      bool linkExtended = false;
      for (EdgeIterator seg = segments->begin(); seg != segments->end(); ++seg)
      {
        Segment<FloatType> segIns = **seg;
        // Check start link (segments have no direction).
        {
          const Segment<FloatType>& front = linkedSegments.front();
          const bool insert     = (segIns.p1 == front.p0);
          const bool insertFlip = (segIns.p0 == front.p0);
          if (insert || insertFlip)
          {
            if (segIns != front)
            {
              if (insertFlip)
              {
                std::swap(segIns.p0, segIns.p1);
              }
              linkedSegments.push_front(segIns);
            }
            segments->erase(seg);
            linkExtended = true;
            break;
          }
        }
        // Check end link (segments have no direction).
        {
          const Segment<FloatType>& back = linkedSegments.back();
          const bool insert     = (back.p1 == segIns.p0);
          const bool insertFlip = (back.p1 == segIns.p1);
          if (insert || insertFlip)
          {
            if (segIns != back)
            {
              if (insertFlip)
              {
                std::swap(segIns.p0, segIns.p1);
              }
              linkedSegments.push_back(segIns);
            }
            segments->erase(seg);
            linkExtended = true;
            break;
          }
        }
      }
      if (!linkExtended)
      {
#ifndef NDEBUG
        // Remaining edges must be dups.
        for (EdgeIterator seg = segments->begin(); seg != segments->end(); ++seg)
        {
          assert((std::find_if(linkedSegments.begin(),
                               linkedSegments.end(),
                               std::bind1st(std::equal_to<Segment<FloatType>>(), **seg))
                  != linkedSegments.end()) && "Undiscovered link.");
        }
#endif
        break;
      }
    }
    // Create polygons. First test if the shape is closed.
    {
      const Vector2<FloatType>* firstPt = &linkedSegments.front().p0;
      const Vector2<FloatType>* lastPt = &linkedSegments.back().p1;
      const bool shapeClosed = (*firstPt == *lastPt);
      if (!shapeClosed)
      {
        // Close the shape using the board edges.
        int lastSegEdge = boardEdgeHelper.FindEdgeCCW(*lastPt);
        assert(lastSegEdge >= 0);
        for (;;)
        {
          // Are we done?
          const bool foundCloser = boardEdgeHelper.EdgeContainsPoint(lastSegEdge, *firstPt);
          if (foundCloser)
          {
            linkedSegments.push_back(Segment<FloatType>(*lastPt, *firstPt));
            break;
          }
          else
          {
            // Add the segment to the end of the board edge.
            const Vector2<FloatType>& nextPt = boardEdgeHelper.EdgePtCCW(lastSegEdge);
            linkedSegments.push_back(Segment<FloatType>(*lastPt, nextPt));
            lastPt = &nextPt;
            lastSegEdge = boardEdgeHelper.NextEdge(lastSegEdge);
          }
        }
      }
    }
    assert(linkedSegments.back().p1 == linkedSegments.front().p0);
    // Gather the vertices and compute the area.
    FloatType normArea = 0;
    {
      const Segment<FloatType>& segFirst = linkedSegments.front();
      normArea += (segFirst.p0.x * segFirst.p1.y) - (segFirst.p0.y * segFirst.p1.x);
      vertices.push_back(segFirst.p0);
    }
    typedef LinkedSegmentsQueue::const_iterator LinkedSegIter;
    for (LinkedSegIter linkedSeg = linkedSegments.begin() + 1;
         linkedSeg != linkedSegments.end();
         ++linkedSeg)
    {
      vertices.push_back(linkedSeg->p0);
      vertices.push_back(linkedSeg->p1);
      normArea += (linkedSeg->p0.x * linkedSeg->p1.y) - (linkedSeg->p0.y * linkedSeg->p1.x);
    }
    normArea *= static_cast<FloatType>(0.5);
    // Add this polygon to the player score.
    scores->at(stoneIdx % m_players) += normArea * boardTotalArea;
  }
}

}
}
