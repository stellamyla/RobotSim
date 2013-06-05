#ifndef CONTACT_TIME_SCALING_H
#define CONTACT_TIME_SCALING_H

#include "TimeScaling.h"
#include "Modeling/MultiPath.h"
#include "Modeling/Robot.h"
#include "RobotCSpace.h"

class ContactTimeScaling
{
 public:
  ContactTimeScaling(Robot& robot);
  bool SetParams(const MultiPath& path,const vector<Real>& paramDivs,int numFCEdges = 4);
  bool Optimize();

  ///double-check whether the solution is actually feasible
  bool Check(const MultiPath& path);

  RobotCSpace cspace;
  RobotGeodesicManifold manifold;
  Real torqueRobustness;   ///< from 0 to 1, indicates the amount of increased robustness in torque limits
  Real frictionRobustness;  ///< from 0 to 1, indicates the amount of increased robustness in friction cones
  Real forceRobustness;   ///< >= 0, indicates the absolute margin for forces to be contained within the friction cone
  TimeScaledBezierCurve traj;
  vector<Real> paramDivs;
  vector<int> paramSections;
  //colocation points
  vector<Vector> xs,dxs,ddxs;
  //Constraint planes a^T (ds^2,dds) <= b in the
  //(squared-rate, acceleration) plane
  vector<vector<Vector2> > ds2ddsConstraintNormals;
  vector<vector<Real> > ds2ddsConstraintOffsets;
};

#endif
