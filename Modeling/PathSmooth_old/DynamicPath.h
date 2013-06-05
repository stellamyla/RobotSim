#ifndef DYNAMIC_PATH_H
#define DYNAMIC_PATH_H

#include "Robot.h"
#include "Ramp.h"
#include <Robotics/CSpace.h>
#include "Planning/RobotCSpace.h"
#include <limits.h>

bool CheckRampExact(const ParabolicRampND& ramp,SingleRobotCSpace* space,int maxiters);
bool CheckRamp(const ParabolicRampND& ramp,CSpace* space,Real tol);

class DynamicPath
{
 public:
  DynamicPath();
  DynamicPath(const Robot& robot);
  void Init(const Robot& robot);
  void Init(const Vector& velMax,const Vector& accMax);
  inline void Clear() { ramps.clear(); }
  inline bool Empty() const { return ramps.empty(); }
  Real GetTotalTime() const;
  void Evaluate(Real t,Vector& x) const;
  void Derivative(Real t,Vector& dx) const;
  void SetMilestones(const vector<Vector>& x);
  void SetMilestones(const vector<Vector>& x,const vector<Vector>& dx);
  void GetMilestones(vector<Vector>& x,vector<Vector>& dx) const;
  void Append(const Vector& x);
  void Append(const Vector& x,const Vector& dx);
  void Concat(const DynamicPath& suffix);
  void Split(Real t,DynamicPath& before,DynamicPath& after) const;
  bool TryShortcut(Real t1,Real t2,CSpace* space,Real tol);
  int Shortcut(CSpace* space,Real tol,Real timeLimit=Inf,int numIters=INT_MAX);
  int DynamicShortcut(Real leadTime,Real padTime,CSpace* space,Real tol);
  int ShortCircuit(CSpace* space,Real tol);
  bool IsValid() const;

  Vector velMax,accMax;
  vector<ParabolicRampND> ramps;
};

#endif
