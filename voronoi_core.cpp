#include "voronoi_core.h"
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
  inline bool operator()(const Segment2<NumericType>& s) const
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
  : public std::binary_function<const EqType*, const EqType*, bool>
{
  inline bool operator()(const EqType* lhs, const EqType* rhs) const
  {
    return *lhs == *rhs;
  }
};

/// <summary> Comparison function for two unit vectors. </summary>
/// <remarks>
///   <para> Sort unit vectors by angle measured from the x-axis. </summary>
///   <para> This is a strict weak ordering. </summary>
/// </remarks>
template <typename NumericType>
struct UnitVectorFastAngleSort
{
  inline bool operator()(const Vector2<NumericType>& v0, const Vector2<NumericType>& v1) const
  {
#if !NDEBUG
    const NumericType kUnitVecLenSqBound = 1.0e-6f;
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
      const Vector2<NumericType> ref(Vector2<NumericType>(positiveX ? 1.0f : -1.0f, 0.0f));
      const NumericType cosTheta_v0 = Vector2Dot(v0, ref);
      const NumericType cosTheta_v1 = Vector2Dot(v1, ref);
      // Smaller angle comes first (cos(0) = 0).
      return (cosTheta_v0 > cosTheta_v1) ? true : false;
    }
  }
};

/// <summary> Sort segments by converting to unit vectors. This is computationally
///   inefficient due to lots of square rooting.
/// </summary>
template <typename NumericType>
struct SegmentAngleSort
{
  inline bool operator()(const Segment2<NumericType>& lhs, const Segment2<NumericType>& rhs) const
  {
    static UnitVectorFastAngleSort<NumericType> s_unitVecSort;
    return s_unitVecSort(Vector2Normalize(lhs.p1 - lhs.p0), Vector2Normalize(rhs.p1 - rhs.p0));
  }
};
}

const FloatType kVecEqSqBound = static_cast<FloatType>(1.0e-5);


bool Voronoi::FortuneScores(ScoreList* scores) const
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
    return true;;
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
  typedef std::vector<Segment2<FloatType> > EdgeList;
  EdgeList edges;
  edges.reserve(numStones * numStones);
  Vector2<FloatType> p0;
  Vector2<FloatType> p1;
  typedef std::deque<const Segment2<FloatType>*> SegmentQueue;
  std::vector<SegmentQueue> stoneEdges(numStones);
  std::for_each(stoneEdges.begin(), stoneEdges.end(),
                std::mem_fun_ref(&SegmentQueue::clear));
  int stoneIndices[2];
  while (diagram.getNext(p0.x, p0.y, p1.x, p1.y, stoneIndices))
  {
//    std::cout << "Edge: "
//              << "(" << p0.x << ", " << p0.y << ") -> "
//              << "(" << p1.x << ", " << p1.y << ") "
//              << " : Stones {" << stoneIndices[0] << " "
//              << stoneIndices[1] << "}" << std::endl;
    const Segment2<FloatType> segment(p0, p1);
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
        // See if dup.
        EdgeList::const_iterator seg = std::find_if(edges.begin(), edges.end(),
                                                    std::bind1st(std::equal_to<Segment2<FloatType> >(),
                                                                 segment));
        if (edges.end() == seg)
        {
          edges.push_back(segment);
          seg = edges.end() - 1;
        }
        // See if added already.
        {
          SegmentQueue& thisStoneEdges = stoneEdges[stoneIndices[0]];
          SegmentQueue::const_iterator added =
            std::find_if(thisStoneEdges.begin(), thisStoneEdges.end(),
                         std::bind1st(detail::PtrDerefEquals<Segment2<FloatType> >(), &*seg));
          if (thisStoneEdges.end() == added)
          {
            thisStoneEdges.push_back(&*seg);
          }
        }
        {
          SegmentQueue& thisStoneEdges = stoneEdges[stoneIndices[1]];
          SegmentQueue::const_iterator added =
            std::find_if(thisStoneEdges.begin(), thisStoneEdges.end(),
                         std::bind1st(detail::PtrDerefEquals<Segment2<FloatType> >(), &*seg));
          if (thisStoneEdges.end() == added)
          {
            thisStoneEdges.push_back(&*seg);
          }
        }
      }
    }
  }
  // Link the extracted segments to create closed polygons around each stone.
  typedef std::vector<Vector2<FloatType> > VertexList;
  std::vector<VertexList> stoneVertices(numStones);
  typedef std::deque<Segment2<FloatType> > LinkedSegmentsQueue;
  LinkedSegmentsQueue linkedSegments;
  for (int stoneIdx = 0; stoneIdx < numStones; ++stoneIdx)
  {
    linkedSegments.clear();
    SegmentQueue* segments = &stoneEdges[stoneIdx];
    assert(segments->size() > 0);
    VertexList& vertices = stoneVertices[stoneIdx];
    while (!segments->empty())
    {
      const int linkedSegmentCountBefore = linkedSegments.size();
      // For each segment, find its match in the linked segments.
      //   { (x0, y0) -> (x1, y1) } ==> { (x1, y1) -> (x2, y2) }
      // Want to go counter-clockwise. Need to find an unambiguous edge.
      const int numSegments = static_cast<int>(segments->size());
      for (int segmentIdx = 0; segmentIdx < numSegments; ++segmentIdx)
      {
        const Segment2<FloatType>* seg = segments->front();
        segments->pop_front();
        // Find the inverse transformation of the vector along the segment and
        // it to the vector from the start of the segment to the stone.
        const Vector2<FloatType> alongSegDenorm = seg->p1 - seg->p0;
        const Vector2<FloatType> alongSeg = Vector2Normalize(alongSegDenorm);
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
      // TODO(reissb) -- 20111025 -- Just split the score evenly if I fail?
      assert((linkedSegmentCountBefore + 1) == static_cast<int>(linkedSegments.size()));
      typedef LinkedSegmentsQueue::const_iterator LinkedSegmentIter;
      const Segment2<FloatType> frontLink = linkedSegments.front();
      typedef SegmentQueue::const_iterator EdgeIterator;
      bool linkExtended;
      do
      {
        linkExtended = false;
        // Link another segment.
        for (EdgeIterator seg = segments->begin(); seg != segments->end(); ++seg)
        {
          Segment2<FloatType> segIns = **seg;
          // Check start link (segments have no direction).
          {
            const bool insert     = Vector2NearlyEqual(segIns.p1, frontLink.p0, kVecEqSqBound);
            const bool insertFlip = Vector2NearlyEqual(segIns.p0, frontLink.p0, kVecEqSqBound);
            if (insert || insertFlip)
            {
              if (!Segment2NearlyEqual(segIns, frontLink, kVecEqSqBound))
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
            const Segment2<FloatType>& back = linkedSegments.back();
            const bool insert     = Vector2NearlyEqual(back.p1, segIns.p0, kVecEqSqBound);
            const bool insertFlip = Vector2NearlyEqual(back.p1, segIns.p1, kVecEqSqBound);
            if (insert || insertFlip)
            {
              if (!Segment2NearlyEqual(segIns, back, kVecEqSqBound))
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
      } while (linkExtended);
    }
    // Sort the segments.
    std::sort(linkedSegments.begin(), linkedSegments.end(), detail::SegmentAngleSort<FloatType>());
    // Create closed polygons.
    {
      typedef LinkedSegmentsQueue::const_iterator LinkedSegIter;
      const Vector2<FloatType>* firstPt = &linkedSegments.front().p0;
      LinkedSegIter curSeg = linkedSegments.begin();
      // Until the shape is closed...
      int wallsAdded = 0;
      while (!Vector2NearlyEqual(curSeg->p1, *firstPt, kVecEqSqBound))
      {
        // Do we need to use walls?
        const Vector2<FloatType>* lastPt = &curSeg->p1;
        LinkedSegIter nextSeg = curSeg + 1;
        const bool outOfSegments = (linkedSegments.end() == nextSeg);
        bool runAlongWalls = outOfSegments || (!outOfSegments &&
                                               !Vector2NearlyEqual(*lastPt, nextSeg->p0,
                                                                   kVecEqSqBound));
        const Vector2<FloatType> runAlongWallsToPt = (runAlongWalls && !outOfSegments) ?
                                                     nextSeg->p0 : *firstPt;
        if (runAlongWalls)
        {
          // Close the shape using the board edges.
          int lastSegEdge = boardEdgeHelper.FindEdgeCCW(*lastPt);
          if(lastSegEdge < 0)
          {
            // reissb -- 20111025 -- Don't know what do here...
            return false;
          }
          for (;;)
          {
            // Are we done?
            const bool foundCloser = boardEdgeHelper.EdgeContainsPoint(lastSegEdge,
                                                                       runAlongWallsToPt);
            if (foundCloser)
            {
              nextSeg = linkedSegments.insert(nextSeg, Segment2<FloatType>(*lastPt,
                                                                           runAlongWallsToPt)) + 1;
              if (++wallsAdded > 4)
              {
                return false;
              }
              break;
            }
            else
            {
              // Add the segment to the end of the board edge.
              const Vector2<FloatType>& nextPt = boardEdgeHelper.EdgePtCCW(lastSegEdge);
              nextSeg = linkedSegments.insert(nextSeg, Segment2<FloatType>(*lastPt, nextPt)) + 1;
              lastPt = &nextPt;
              lastSegEdge = boardEdgeHelper.NextEdge(lastSegEdge);
              if (++wallsAdded > 4)
              {
                return false;
              }
            }
          }
          curSeg = linkedSegments.end() - 1;
        }
        else
        {
          ++curSeg;
        }
      }
    }
//    if (!Vector2NearlyEqual(linkedSegments.back().p1, linkedSegments.front().p0, kVecEqSqBound))
//    {
//      return false;
//    }
    // Gather the vertices and compute the area.
    FloatType normArea = 0;
    {
      const Segment2<FloatType>& segFirst = linkedSegments.front();
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
  return true;
}

bool Voronoi::Scores(ScoreList* scores) const
{
  bool FortuneScoringSucceeded = FortuneScores(scores);
  if(FortuneScoringSucceeded)
    return true
  else
    NaiveScore(this, scores);
}

}
}
