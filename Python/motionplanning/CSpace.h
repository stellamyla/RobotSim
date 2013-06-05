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
#ifndef ROBOTICS_CSPACE_H
#define ROBOTICS_CSPACE_H

#include "Vector.h"
#include "Miscellany.h"

typedef Vector Config;

class EdgePlanner;

/** @ingroup MotionPlanning
 * @brief Motion planning configuration space base class.
 *
 * An abstract base class defining the C-space interfaces needed for
 * motion planning.  The methods represent a configuration as a
 * vector<double>, but the space may be noneuclidean or not even the
 * same dimensionality as the # of entries in the configuration.
 *
 * The Sample, IsFeasible, and LocalPlanner methods must be overridden
 * by the subclass.  They implicitly define the C-space and free space.
 * Sample MUST have a nonzero chance of succeeding for any of the 
 * motion planning algorithms to work.
 * LocalPlanner will return a new instance of an EdgePlanner
 * (see EdgePlanner.h) depending on the desired type of local planner.
 * 
 * The Distance, Interpolate, and SampleNeighborhood methods
 * are optionally overrideable; the default implementations assume a
 * Euclidean space.  They must be overridden to implement a C-space with
 * a noneuclidean topology.
 *
 * The ObstacleDistance method can be overridden to use the
 * StraightLineObstacleDistancePlanner local planner (note that this has
 * not been thoroughly tested).
 */
class CSpace
{
public:
  virtual ~CSpace() {}
  virtual void Sample(Config& x)=0;
  virtual void SampleNeighborhood(const Config& c,double r,Config& x);
  virtual bool IsFeasible(const Config&)=0;
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) =0;

  ///optionally overrideable (default uses euclidean space)
  virtual double Distance(const Config& x, const Config& y) { return VectorOperation::euclideanDistance(x,y); }
  virtual void Interpolate(const Config& x,const Config& y,double u,Config& out);

  ///for local planners using obstacle distance
  virtual double ObstacleDistance(const Config& a) { return ConstantHelper::Inf; };
};

#endif
