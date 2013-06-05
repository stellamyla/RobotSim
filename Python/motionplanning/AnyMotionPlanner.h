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
#ifndef ANY_MOTION_PLANNER_H
#define ANY_MOTION_PLANNER_H

#include "MotionPlanner.h"

/** @ingroup MotionPlanning
 * @brief A base class for a generic motion planner, which can be used
 * to compare multiple planning algorithms under a common interface.
 */
class MotionPlannerInterface
{
 public:
  MotionPlannerInterface() {}
  virtual ~MotionPlannerInterface() {}
  virtual int PlanMore()=0;
  virtual void PlanMore(int numIters) { for(int i=0;i<numIters;i++) PlanMore(); }
  virtual int NumIterations() const=0;
  virtual int NumMilestones() const=0;
  virtual int NumComponents() const=0;
  virtual bool CanAddMilestone() const { return false; }
  virtual int AddMilestone(const Config& q)=0;
  virtual void GetMilestone(int,Config& q)=0;
  virtual void ConnectHint(int m) { }
  virtual bool ConnectHint(int ma,int mb) { return false; }
  virtual bool IsConnected(int ma,int mb) const=0;
  virtual bool IsLazy() const { return false; }
  virtual bool CheckPath(int ma,int mb) { return false; }
  virtual bool IsLazyConnected(int ma,int mb) const { return IsConnected(ma,mb); }
  virtual void GetPath(int ma,int mb,MilestonePath& path)=0;
  virtual void GetRoadmap(RoadmapPlanner& roadmap) {}
};

/** @ingroup MotionPlanning
 * @brief A factory that assists with comparing the motion planners 
 * implemented in LMPL. 
 *
 * So far, only PRM, SBL, and SBLPRT are functional.  More will come
 * over time.
 *
 * Planner settings can be tweaked by changing the members of this
 * class from their default settings.
 */
class MotionPlannerFactory
{
 public:
  enum Type { PRM,LazyPRM,PerturbationTree,EST,RRT,SBL,SBLPRT};

  MotionPlannerFactory();
  virtual ~MotionPlannerFactory() {}
  virtual MotionPlannerInterface* Create(CSpace* space);

  Type type;
  int knn;                 //for PRM
  double connectionThreshold;//for PRM,SBL
  double perturbationRadius; //for Perturbation,EST,RRT,SBL,SBLPRT
  int perturbationIters;   //for SBL
  bool bidirectional;      //for RRT
  bool useGrid;            //for SBL
  double gridResolution;     //for SBL
  int randomizeFrequency;  //for SBL
};


#endif
