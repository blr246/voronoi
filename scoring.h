#ifndef _HPS_VORONOI_SCORING_H_
#define _HPS_VORONOI_SCORING_H_
/* This code is taken from http://www.cs.hmc.edu/~mbrubeck/voronoi.html. It has
   no license there. It was written by Matt Brubeck in 2002 for a Computational
   Geometry class taught by Greg Levin at Harvey Mudd College.
*/
#include "voronoi_core.h"
#include "geometry.h"
#include <vector>

namespace hps
{
namespace voronoi
{

/// <summary> Adapted code to extract a Voronoi diagram. </summary>
class VoronoiScoring
{
public:
  typedef Voronoi::StoneNormalized StoneType;
  typedef StoneType::Position Point;

  // Forward declarations.
  struct Arc;
  struct Seg;
  /// <summary> Fortune's algorithm events. </summary>
  struct Event
  {
    Event(const double& xx, const Point& pp, Arc* aa) : x(xx), p(pp), a(aa), valid(true) {}
    double x;
    Point p;
    Arc *a;
    bool valid;
  };
  /// <summary> Arc. </summary>
  struct Arc
  {
    Arc(const Point& pp) : p(pp), prev(NULL), next(NULL), e(0), s0(0), s1(0) {}
    Arc(const Point& pp, Arc *a) : p(pp), prev(a), next(NULL), e(0), s0(0), s1(0) {}
    Arc(const Point& pp, Arc *a, Arc *b) : p(pp), prev(a), next(b), e(0), s0(0), s1(0) {}
    Point p;
    Arc *prev, *next;
    Event *e;
    Seg *s0, *s1;
  };
  /// <summary> A line in the final Voronoi diagram. </summary>
  struct Seg
  {
    Seg(const Point& p) : start(p), end(0,0), done(false) {}
    Point start, end;
    bool done;
    /// <summary> Set the end point and mark as "done." </summary>
    inline void Finish(const Point& p)
    {
      if (done)
      {
        return;
      }
      else
      {
        end = p;
        done = true;
      }
    }
  };
  /// <summary> "Greater than" comparison, for reverse sorting in priority queue. </summary>
  struct PointGreater
  {
    inline bool operator()(const Point& a, const Point& b) const
    {
      return (a.x == b.x) ? (a.y > b.y) : (a.x > b.x);
    }
  };
  /// <summary> Priority sorting for stones. </summary>
  struct StonePriorityRec
  {
    StonePriorityRec(const StoneType& stone_) : stone(&stone_) {}
    struct Sort
    {
      inline bool operator()(const StonePriorityRec& lhs, const StonePriorityRec& rhs) const
      {
        static PointGreater s_pointCmp;
        return s_pointCmp(lhs.stone->pos, rhs.stone->pos);
      }
    };
    const StoneType* stone;
  };
  /// <summary> Priority sorting for events. </summary>
  struct EventPriorityRec
  {
    EventPriorityRec(Event* event_) : event(event_) {}
    struct Sort
    {
      inline bool operator()(const EventPriorityRec& lhs, const EventPriorityRec& rhs) const
      {
        static PointGreater s_pointCmp;
        return s_pointCmp(lhs.event->p, rhs.event->p);
      }
    };
    Event* event;
  };
  /// <summary> Data required to score the Voronoi diagram. </summary>
struct ScoringProcessParams
{
    ScoringProcessParams();
    /// <summary> Array of output segments. </summary>
    std::vector<Seg> output;
    /// <summary> First item in the parabolic front linked list. </summary>
    Arc* root;
    /// <summary> Site events queue. </summary>
    std::vector<StonePriorityRec> stonesQueue;
    /// <summary> Circle events queue. </summary>
    std::vector<EventPriorityRec> eventsQueue;
    /// <summary> Circle events. </summary>
    std::vector<Event> events;
  };

  /// <summary> Score a Voronoi diagram. </summary>
  static void Scores(const Voronoi::StoneNormalizedList& stones,
                     ScoringProcessParams* params, Voronoi::ScoreList* scores);
};

}
}

#endif //_HPS_VORONOI_SCORING_H_
