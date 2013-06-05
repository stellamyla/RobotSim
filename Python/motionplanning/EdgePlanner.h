/*
This file is part of LMPL.

    LMPL is free software: you can redistribute it and/or modify
    it under the terms of the Lesser GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LMPL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Lesser
    GNU General Public License for more details.

    You should have received a copy of the Lesser GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef ROBOTICS_EDGE_PLANNER_H
#define ROBOTICS_EDGE_PLANNER_H

#include "CSpace.h"
#include <list>
#include <queue>
#include <stdlib.h>
using namespace std;

/** @ingroup MotionPlanning
 * @brief Abstract base class for an edge planner.
 *
 * NOTE: the only thing allowed to be referenced is the workspace!
 *
 * IsVisible performs the planning process
 * Eval returns the path p(u) with u from 0->1.  p(0)=a, p(1)=b
 * Start returns p(0)
 * Goal returns p(1)
 * Space() returns the workspace in which this lives
 * Copy returns a copy
 * ReverseCopy returns the reverse edge planner
 * 
 * For incremental planners:
 * Priority returns some priority measure (such as distance btwn configs)
 * Plan performs one step of the planning process, returns true
 *   to continue, false on failure
 * Done returns true if the planning's done
 */
class EdgePlanner
{
public:
  EdgePlanner() {}
  virtual ~EdgePlanner() {}
  virtual bool IsVisible() =0;
  virtual void Eval(double u,Config& x) const=0;
  virtual const Config& Start() const=0;
  virtual const Config& Goal() const=0;
  virtual CSpace* Space() const=0;
  virtual EdgePlanner* Copy() const=0;
  virtual EdgePlanner* ReverseCopy() const=0;

  //for incremental planners
  virtual double Priority() const { abort(); return 0; }
  virtual bool Plan() { abort(); return false; }
  virtual bool Done() const { abort(); return false; }
  virtual bool Failed() const { abort(); return false; }
};

/// Point-to-self edge planner
class DummyEdgePlanner : public EdgePlanner
{
public:
  DummyEdgePlanner(CSpace* space,const Config& x);
  virtual bool IsVisible() { return true; }
  virtual void Eval(double u,Config& x) const { x=a; }
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return a; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const { return new DummyEdgePlanner(space,a); }
  virtual EdgePlanner* ReverseCopy() const { return Copy(); }

  Config a;
  CSpace* space;
};

/** @ingroup MotionPlanning
 * @brief Straight-line edge planner that divides the segment until 
 * epsilon is reached.
 */
class StraightLineEpsilonPlanner : public EdgePlanner
{
public:
  StraightLineEpsilonPlanner(const Config& a,const Config& b,CSpace* space,double epsilon);
  virtual bool IsVisible();
  virtual void Eval(double u,Config& x) const;
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;
  virtual double Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;

  Config a,b;
  CSpace* space;
  double epsilon;

private:
  double dist;
  int depth;
  int segs;
  Config m;
};

/** @ingroup MotionPlanning
 * @brief Straight-line edge planner that divides the segment until 
 * the segment distance is below CSpace.ObstacleDistance()
 */
class StraightLineObstacleDistancePlanner : public EdgePlanner
{
public:
  StraightLineObstacleDistancePlanner(const Config& a,const Config& b,CSpace* space);
  virtual bool IsVisible();
  virtual void Eval(double u,Config& x) const;
  virtual const Config& Start() const { return a; }
  virtual const Config& Goal() const { return b; }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;

  bool CheckVisibility(const Config& a,const Config& b,double da,double db);

  Config a,b;
  CSpace* space;
};

/** @ingroup MotionPlanning
 * The same as StraightLineEpsilonPlanner, but keeps the bisected configs,
 * and recalculates distances for every subdivision.
 *
 * Used in constrained configuration spaces where CSpace.Midpoint() doesn't
 * necessarily return a configuration whose distance is half of the endpoints.
 */
class BisectionEpsilonEdgePlanner : public EdgePlanner
{
public:
  BisectionEpsilonEdgePlanner(const Config& a,const Config& b,CSpace* space,double epsilon);
  virtual ~BisectionEpsilonEdgePlanner() {}
  virtual bool IsVisible();
  virtual void Eval(double u,Config& x) const;
  virtual const Config& Start() const { return path.front(); }
  virtual const Config& Goal() const { return path.back(); }
  virtual CSpace* Space() const { return space; }
  virtual EdgePlanner* Copy() const;
  virtual EdgePlanner* ReverseCopy() const;
  virtual double Priority() const;
  virtual bool Plan();
  virtual bool Done() const;
  virtual bool Failed() const;
  //on failure, returns the segment last checked
  bool Plan(Config*& pre,Config*& post);

  const std::list<Config>& GetPath() const { return path; }
  const Config& InfeasibleConfig() const { return x; }

protected:
  BisectionEpsilonEdgePlanner(CSpace* space,double epsilon);

  std::list<Config> path;
  CSpace* space;
  double epsilon;

  struct Segment
  {
    inline bool operator < (const Segment& s) const { return length<s.length; }

    std::list<Config>::iterator prev;
    double length;
  };

  std::priority_queue<Segment,std::vector<Segment> > q;
  Config x;
};

///helper, returns non-NULL if the edge is visible
inline EdgePlanner* IsVisible(CSpace* w,const Config& a,const Config& b)
{
  EdgePlanner* e=w->LocalPlanner(a,b);
  if(e->IsVisible()) return e;
  delete e;
  return NULL;
}

#endif
