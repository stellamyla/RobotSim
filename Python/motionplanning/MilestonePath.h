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
#ifndef MILESTONE_PATH_H
#define MILESTONE_PATH_H

#include "CSpace.h"
#include "EdgePlanner.h"
#include "SmartPointer.h"
using namespace std;

/** @ingroup MotionPlanning
 * @brief A sequence of locally planned paths between milestones
 *
 * If n is the number of edges, the milestones are indexed
 * M0...Mn+1, such that edge k goes from Mk to Mk+1.
 */
class MilestonePath
{
public:
  MilestonePath();
  ~MilestonePath();

  const Config& GetMilestone(int milestone) const;
  void SetMilestone(int milestone,const Config& x);
  const Config& Start() const { return edges.front()->Start(); }
  const Config& End() const { return edges.back()->Goal(); }
  /// Sets the milestone to x only if x and the paths to adjoining 
  /// milestones are feasible 
  bool CheckSetMilestone(int milestone,const Config& x);
  inline CSpace* Space(int i=0) const { assert(!edges.empty()); return edges[i]->Space(); }
  inline int NumMilestones() const { return (int)edges.size()+1; }
  inline int NumEdges() const { return (int)edges.size(); }
  bool IsValid();
  /// Returns the sum of the distances between milestones
  double Length() const;
  /// Adds the path onto the end of this one
  void Concat(const MilestonePath& path);
  /// Create the path that connects the milestones in the given workspace
  void CreateEdgesFromMilestones(CSpace* space,const vector<Config>& milestones);
  /// Checks the feasibility of all edges, returns true if they all succeed
  bool InitializeEdgePlans();
  /// Checks the feasibility of all milestones and edges, returns true if so
  bool IsFeasible();
  /// Supposing all milestones have equal time spacings, evaluates the
  /// point on the path at time t in [0,1].
  /// Returns the edge of time t.
  int Eval(double t, Config& c) const;
  /// Tries to shorten the path by connecting subsequent milestones.
  /// Returns # of shortcuts made.
  int Shortcut();
  /// Tries to shorten the path by connecting random points
  /// with a shortcut, for numIters iterations.  Returns # of shortcuts
  int Reduce(int numIters);
  /// Replaces the section of the path between milestones
  /// start and goal with a new path.  If the index is negative,
  /// erases the corresponding start/goal milestones too.
  void Splice(int start,int goal,const MilestonePath& path); 
  /// Discretizes the path such that each edge is no longer than h.
  /// Assumes straight-line path segments.
  void Discretize(double h);
  /// Discretizes only the given edge.  Returns the number of new segments.
  int DiscretizeEdge(int e,double h);
  /// Discretizes the given edge with the specified interpolation.
  void DiscretizeEdge(int e,const vector<double>& u);
  /// Loads the intermediate milestones, and creates the edges from the given space
  bool Load(istream& in,CSpace* space);
  /// Saves the intermediate milestones
  bool Save(ostream& out);

  vector<SmartPointer<EdgePlanner> > edges;
};

#endif
