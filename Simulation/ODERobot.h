#ifndef ODE_INTERFACE_ROBOT_H
#define ODE_INTERFACE_ROBOT_H

#include "Modeling/Robot.h"
#include "ODETriMesh.h"
#include "ODESurface.h"
#include <ode/common.h>
#include <myfile.h>
using namespace Math;
using namespace std;

/** @ingroup Simulation
 * @brief A robot simulated in an ODE "world"
 *
 * Collision detection
 * geom is a list of ODETriMesh objects.  The ODE "geom" of geom[i] contains
 *    user data = (void*)body_index.  This can be used for collision
 *    identification.
 * spaceID is a pointer to an ODE "space" containing all the robot's geometry.
 *    it contains user data = (void*)this.
 */
class ODERobot
{
 public:
  static double defaultPadding;
  static ODESurfaceProperties defaultSurface;

  ODERobot(Robot& robot);
  ~ODERobot();
  void Create(dWorldID worldID);
  void Clear();
  void SetConfig(const Config& q);
  void GetConfig(Config& q) const;
  void SetVelocities(const Config& dq);
  void GetVelocities(Config& dq) const;
  void AddTorques(const Vector& t);
  Real GetJointAngle(int joint) const;
  Real GetJointVelocity(int joint) const;
  void AddJointTorque(int joint,Real t);
  void SetJointDryFriction(int joint,Real coeff);
  void SetJointFixedVelocity(int joint,Real vel,Real tmax);
  Real GetLinkAngle(int link) const;
  Real GetLinkVelocity(int link) const;
  void AddLinkTorque(int link,Real t);
  void SetLinkDryFriction(int link,Real coeff);
  void SetLinkFixedVelocity(int link,Real vel,Real tmax);
  Real GetDriverValue(int driver) const;
  Real GetDriverVelocity(int driver) const;
  void AddDriverTorques(const Vector& t);
  void AddDriverTorque(int driver,Real t);
  void SetDriverFixedVelocity(int driver,Real vel,Real tmax);

  void SetLinkTransform(int link,const RigidTransform& T);
  void GetLinkTransform(int link,RigidTransform& T) const;
  void SetLinkVelocity(int link,const Vector3& w,const Vector3& v);
  void GetLinkVelocity(int link,Vector3& w,Vector3& v) const;
  bool ReadState(File& f);
  bool WriteState(File& f) const;

  inline dSpaceID space() const { return spaceID; }
  inline dBodyID body(int link) const { return bodyID[link]; }
  inline dGeomID geom(int link) const { return geometry[link]->geom(); }
  inline dJointID joint(int link) const { return jointID[link]; }
  inline ODETriMesh* triMesh(int link) const { return geometry[link]; }
  dBodyID baseBody(int link) const;  //for attached links, returns the base body for the link
  inline dJointFeedback feedback(int link) const { return jointFeedback[link]; }

  Robot& robot;

 private:
  vector<RigidTransform> T_bodyToLink;
  vector<dBodyID> bodyID;
  vector<ODETriMesh*> geometry;
  vector<dJointID> jointID;
  vector<dJointFeedback> jointFeedback;
  dJointGroupID jointGroupID;
  dSpaceID spaceID;
};

#endif
