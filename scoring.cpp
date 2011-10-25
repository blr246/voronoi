/* This code is taken from http://www.cs.hmc.edu/~mbrubeck/voronoi.html. It has
   no license there. It was written by Matt Brubeck in 2002 for a Computational
   Geometry class taught by Greg Levin at Harvey Mudd College.
*/
#include "scoring.h"
#include <math.h>
#include <iostream>
#include <algorithm>

namespace hps
{
namespace voronoi
{

VoronoiScoring::ScoringProcessParams::ScoringProcessParams()
 : output(),
   root(NULL),
   stonesQueue(),
   eventsQueue(),
   events()
{}

namespace detail
{
typedef Voronoi::FloatType FloatType;
typedef VoronoiScoring::StoneType StoneType;
typedef StoneType::Position Point;
typedef VoronoiScoring::ScoringProcessParams ScoringProcessParams;
typedef VoronoiScoring::Arc Arc;
typedef VoronoiScoring::Event Event;
typedef VoronoiScoring::EventPriorityRec EventPriorityRec;
typedef VoronoiScoring::StonePriorityRec StonePriorityRec;
typedef VoronoiScoring::Seg Seg;

static const AxisAlignedBox<FloatType>& BoundingBox()
{
  // Bounding box plus 20% margins.
//  static AxisAlignedBox<FloatType>
//    s_bounds(Vector2<FloatType>(static_cast<FloatType>(-0.2), static_cast<FloatType>(-0.2)),
//             Vector2<FloatType>(static_cast<FloatType>(1.2), static_cast<FloatType>(1.2)));
//  static AxisAlignedBox<Voronoi::FloatType>
//    s_bounds(Vector2<FloatType>(static_cast<FloatType>(-0.501f), static_cast<FloatType>(-0.501f)),
//             Vector2<FloatType>(static_cast<FloatType>( 0.501f), static_cast<FloatType>( 0.501f)));
  static AxisAlignedBox<Voronoi::FloatType>
    s_bounds(Vector2<FloatType>(static_cast<FloatType>(0), static_cast<FloatType>(0)),
             Vector2<FloatType>(static_cast<FloatType>(1), static_cast<FloatType>(1)));
  return s_bounds;
}

void ProcessPoint(ScoringProcessParams* params);
void ProcessEvent(ScoringProcessParams* params);
void FrontInsert(const StoneType& stone, ScoringProcessParams* params);

inline bool Circle(const Point& a, const Point& b, const Point& c, double *x, Point *o);
void CheckCircleEvent(const double& x0, Arc *i, ScoringProcessParams* params);

inline bool Intersect(const Point& p, const Arc* i, Point *result);
const Point Intersection(const Point& p0, const Point& p1, const double& l);

void FinishEdges(ScoringProcessParams* params);
void PrintOutput(ScoringProcessParams* params);
}

void VoronoiScoring::Scores(const Voronoi::StoneNormalizedList& stones,
                            ScoringProcessParams* params, Voronoi::ScoreList* scores)
{
  assert(params);
  assert(scores);

  params->stonesQueue.clear();
  params->eventsQueue.clear();
  params->events.clear();
  params->output.clear();
  delete params->root;
  params->root = NULL;

  const int numStones = static_cast<int>(stones.size());
  params->events.reserve(numStones * numStones);
  params->output.reserve(numStones * numStones * numStones);

  std::vector<StonePriorityRec>& stonesQueue = params->stonesQueue;
  std::vector<EventPriorityRec>& eventsQueue = params->eventsQueue;

  // Load stones into queue.
  typedef Voronoi::StoneNormalizedList::const_iterator StoneIterator;
  for (StoneIterator stone = stones.begin(); stone != stones.end(); ++stone)
  {
    stonesQueue.push_back(StonePriorityRec(*stone));
  }
  std::make_heap(stonesQueue.begin(), stonesQueue.end(), StonePriorityRec::Sort());

  // Process the queues; select the top element with smaller x coordinate.
  while (!stonesQueue.empty())
  {
    if (!eventsQueue.empty() &&
        (eventsQueue.front().event->p.x <= stonesQueue.front().stone->pos.x))
    {
      detail::ProcessEvent(params);
    }
    else
    {
      detail::ProcessPoint(params);
    }
  }

  // After all points are processed, do the remaining circle events.
  while (!eventsQueue.empty())
  {
     detail::ProcessEvent(params);
  }

  detail::FinishEdges(params); // Clean up dangling edges.
  detail::PrintOutput(params); // Output the voronoi diagram.
}

namespace detail
{

void ProcessPoint(ScoringProcessParams* params)
{
  std::vector<VoronoiScoring::StonePriorityRec>& stonesQueue = params->stonesQueue;

  // Get the next point from the queue.
  const StoneType* stone = stonesQueue.front().stone;
  std::pop_heap(stonesQueue.begin(), stonesQueue.end(), StonePriorityRec::Sort());
  stonesQueue.pop_back();

  // Add a new arc to the parabolic front.
  FrontInsert(*stone, params);
}

void ProcessEvent(ScoringProcessParams* params)
{
  std::vector<EventPriorityRec>& eventsQueue = params->eventsQueue;
  std::vector<Seg>& output = params->output;

  // Get the next event from the queue.
  Event *e = eventsQueue.front().event;
  std::pop_heap(eventsQueue.begin(), eventsQueue.end(), EventPriorityRec::Sort());
  eventsQueue.pop_back();

  if (e->valid)
  {
    // Start a new edge.
    output.push_back(Seg(e->p));
    Seg *s = &output.back();

    // Remove the associated arc from the front.
    Arc *a = e->a;
    if (a->prev)
    {
      a->prev->next = a->next;
      a->prev->s1 = s;
    }
    if (a->next)
    {
      a->next->prev = a->prev;
      a->next->s0 = s;
    }

    // Finish the edges before and after a.
    if (a->s0)
    {
      a->s0->Finish(e->p);
    }
    if (a->s1)
    {
      a->s1->Finish(e->p);
    }

    // Recheck circle events on either side of p:
    if (a->prev)
    {
      CheckCircleEvent(e->x, a->prev, params);
    }
    if (a->next)
    {
      CheckCircleEvent(e->x, a->next, params);
    }
  }
}

void FrontInsert(const StoneType& stone, ScoringProcessParams* params)
{
  VoronoiScoring::Arc *& root = params->root;
  const Point& p = stone.pos;
  if (NULL == root)
  {
    root = new Arc(p);
    return;
  }

  std::vector<Seg>& output = params->output;
  // Find the current arc(s) at height p.y (if there are any).
  for (Arc *i = root; NULL != i; i = i->next)
  {
    Point z, zz;
    if (Intersect(p, i, &z))
    {
      // New parabola intersects arc i.  If necessary, duplicate i.
      if (i->next && !Intersect(p, i->next, &zz))
      {
        i->next->prev = new Arc(i->p, i, i->next);
        i->next = i->next->prev;
      }
      else
      {
        i->next = new Arc(i->p, i);
      }
      i->next->s1 = i->s1;

      // Add p between i and i->next.
      i->next->prev = new Arc(p, i, i->next);
      i->next = i->next->prev;

      i = i->next; // Now i points to the new arc.

      // Add new half-edges connected to i's endpoints.
      output.push_back(Seg(z));
      i->prev->s1 = i->s0 = &output.back();
      output.push_back(Seg(z));
      i->next->s0 = i->s1 = &output.back();

      // Check for new circle events around the new arc:
      CheckCircleEvent(p.x, i, params);
      CheckCircleEvent(p.x, i->prev, params);
      CheckCircleEvent(p.x, i->next, params);

      return;
    }
  }

  // Special case: If p never intersects an arc, append it to the list.
  Arc *i;
  for (i = root; i->next; i=i->next) ; // Find the last node.

  i->next = new Arc(p,i);  
  // Insert segment between p and i
  Point start;
  start.x = BoundingBox().mins.x;
  start.y = (i->next->p.y + i->p.y) / 2;
  i->s1 = i->next->s0 = new Seg(start);
}

// Look for a new circle event for arc i.
void CheckCircleEvent(const double& x0, Arc *i, ScoringProcessParams* params)
{
  // Invalidate any old event.
  if (i->e && (i->e->x != x0))
  {
    i->e->valid = false;
  }
  i->e = NULL;

  if (!i->prev || !i->next)
  {
    return;
  }

  double x;
  Point o;
  if (Circle(i->prev->p, i->p, i->next->p, &x,&o) && (x > x0))
  {
    // Create new event.
    std::vector<VoronoiScoring::Event>& events = params->events;
    std::vector<VoronoiScoring::EventPriorityRec>& eventsQueue = params->eventsQueue;
    events.push_back(Event(x, o, i));
    i->e = &events.back();
    eventsQueue.push_back(EventPriorityRec(&events.back()));
    std::make_heap(eventsQueue.begin(), eventsQueue.end(), EventPriorityRec::Sort());
  }
}

// Find the rightmost point on the circle through a,b,c.
inline bool Circle(const Point& a, const Point& b, const Point& c, double *x, Point *o)
{
   // Check that bc is a "right turn" from ab.
   if ((b.x-a.x)*(c.y-a.y) - (c.x-a.x)*(b.y-a.y) > 0)
   {
      return false;
   }

   // Algorithm from O'Rourke 2ed p. 189.
   const double A = b.x - a.x,  B = b.y - a.y;
   const double C = c.x - a.x,  D = c.y - a.y;
   const double E = A*(a.x+b.x) + B*(a.y+b.y);
   const double F = C*(a.x+c.x) + D*(a.y+c.y);
   const double G = 2*(A*(c.y-b.y) - B*(c.x-b.x));

   // Points are co-linear?
   if (G == 0)
   {
     return false;
   }

   // Point o is the center of the circle.
   o->x = (D*E-B*F)/G;
   o->y = (A*F-C*E)/G;

   // o.x plus radius equals max x coordinate.
   *x = o->x + sqrt( pow(a.x - o->x, 2) + pow(a.y - o->y, 2) );
   return true;
}

// Will a new parabola at point p intersect with arc i?
inline bool Intersect(const Point& p, const Arc* i, Point *result)
{
   if (i->p.x == p.x)
   {
     return false;
   }

   double a = 0.0, b = 0.0;
   if (i->prev) // Get the intersection of i->prev, i.
   {
      a = Intersection(i->prev->p, i->p, p.x).y;
   }
   if (i->next) // Get the intersection of i->next, i.
   {
      b = Intersection(i->p, i->next->p, p.x).y;
   }

   if ((!i->prev || (a <= p.y)) && (!i->next || (p.y <= b)))
   {
      result->y = p.y;
      result->x = (i->p.x*i->p.x + (i->p.y-result->y)*(i->p.y-result->y) - p.x*p.x) /
                  (2*i->p.x - 2*p.x);

      return true;
   }
   return false;
}

// Where do two parabolas intersect?
const Point Intersection(const Point& p0, const Point& p1, const double& l)
{
  Point res, p = p0;

  const double z0 = 2*(p0.x - l);
  const double z1 = 2*(p1.x - l);

  if (p0.x == p1.x)
  {
    res.y = (p0.y + p1.y) / 2;
  }
  else if (1 == p1.x)
  {
    res.y = p1.y;
  }
  else if (1 == p0.x)
  {
    res.y = p0.y;
    p = p1;
  }
  else
  {
    // Use the quadratic formula.
    const double a = (1/z0) - (1/z1);
    const double b = -2*((p0.y/z0) - (p1.y/z1));
    const double c = (((p0.y*p0.y) + (p0.x*p0.x) - (l*l)) / z0) -
                     (((p1.y*p1.y) + (p1.x*p1.x) - (l*l)) / z1);

    res.y = ( -b - sqrt(b*b - 4*a*c) ) / (2*a);
  }
  // Plug back into one of the parabola equations.
  res.x = (p.x*p.x + (p.y-res.y)*(p.y-res.y) - l*l)/(2*p.x-2*l);
  return res;
}

void FinishEdges(ScoringProcessParams* params)
{
  // Advance the sweep line so no parabolas can cross the bounding box.
  const AxisAlignedBox<FloatType>& boundingBox = BoundingBox();
  const double l = boundingBox.maxs.x + (boundingBox.maxs.x-boundingBox.mins.x) +
                                        (boundingBox.maxs.y-boundingBox.mins.y);

  // Extend each remaining segment to the new parabola intersections.
  Arc*& root = params->root;
  for (Arc *i = root; NULL != i->next; i = i->next)
  {
    if (i->s1)
    {
      i->s1->Finish(Intersection(i->p, i->next->p, l*2));
    }
  }
}

void PrintOutput(ScoringProcessParams* params)
{
  // Bounding box coordinates.
  const AxisAlignedBox<FloatType>& boundingBox = BoundingBox();
  std::cout << boundingBox.mins.x << " " << boundingBox.maxs.x << " "
            << boundingBox.mins.y << " " << boundingBox.maxs.y << std::endl;

  // Each output segment in four-column format.
  std::vector<Seg>::const_iterator i = params->output.begin();
  for (; i != params->output.end(); ++i)
  {
    const Point& p0 = i->start;
    const Point& p1 = i->end;
    std::cout << p0.x << " " << p0.y << " " << p1.x << " " << p1.y << std::endl;
  }
}

} // end ns detail

}
}
