#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "Modeling/World.h"
#include "Modeling/DynamicPath.h"
#include <camera/viewport.h>
#include "Planning/RobotCSpace.h"
#include "Planning/PlannerSettings.h"
#include "Val3/SafeTrajClient.h"

class WorldSimulation;
class Val3Interface;
class RealTimePlannerBase;

/** @brief An abstract base class for some interface that uses a robot/world
 * model to control either a simulated or real robot.
 *
 * world, viewport, settings, and either sim or val3 must be initialized
 * before using the class.
 */
struct CommandInterface
{
  //settings
  RobotWorld* world;
  WorldSimulation* sim;
  Val3Interface* val3;
  Camera::Viewport* viewport;
  WorldPlannerSettings* settings;
  double minimumLatency,resetLatencyMultiplier;
  typedef SafeTrajClient::MotionResult MotionResult;

  CommandInterface();
  virtual ~CommandInterface() { }
  Robot* GetRobot() const { return world->robots[0].robot; }
  void GetClickRay(int mx,int my,Ray3D& ray) const;

  //interfaces to simulated or actual world
  Real GetCurTime();
  void GetCurConfig(Config& x);
  void GetCurVelocity(Config& dx);
  Real GetEndTime();
  void GetEndConfig(Config& x);
  void GetEndVelocity(Config& dx);
  MotionResult SendMilestone(const Config& x);
  MotionResult SendMilestoneImmediate(const Config& x);
  MotionResult SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path);

  //Subclasses overload these functions
  //
  virtual string Name() const { return "Unnamed"; }
  virtual string Description() const { return "Unnamed"; }
  virtual string Instructions() const { return ""; }
  virtual void DrawGL() {}
  //
  //The following callbacks are called respectively upon
  //activation/deactivation;
  //mouse input;
  //spaceball input;
  //keyboard input;
  //and idle update (here is where motions should be sent).
  //The return value is a string that gets logged to disk.
  virtual string ActivateEvent(bool enable) { return ""; }
  virtual string MouseInputEvent(int mx,int my,bool drag) { return ""; }
  virtual string SpaceballEvent(const RigidTransform& T) { return ""; }
  virtual string KeypressEvent(unsigned char c,int mx,int my) { return ""; }
  virtual string UpdateEvent() { return ""; }
};

/** @brief An interface that allows the user to pose individual joints
 * using mouse dragging.
 */
struct JointCommandInterface : public CommandInterface
{
  virtual string Name() const { return "JointPoser"; }
  virtual string Description() const { return "Joint poser"; }
  virtual string Instructions() const { return "Click and drag to pose individual joints"; }
  virtual string ActivateEvent(bool enabled);
  virtual string MouseInputEvent(int mx,int my,bool drag);
  virtual string UpdateEvent();

  int currentLink; 
  Config command;
  bool sendCommand;
};

/** @brief An interface that allows the user to pose points on the robot by
 * pointing and dragging
 */
struct IKCommandInterface : public CommandInterface
{
  virtual string Name() const { return "PointPoser"; }
  virtual string Description() const { return "Point poser"; }
  virtual string Instructions() const { return "Click and drag to pose points"; }
  virtual string ActivateEvent(bool enabled);
  virtual void DrawGL();
  virtual string MouseInputEvent(int mx,int my,bool drag);
  virtual string SpaceballEvent(const RigidTransform& T);
  virtual string UpdateEvent();

  int currentLink; 
  Vector3 currentPoint;
  Vector3 currentDestination;
  Config command;
  bool sendCommand;
};




struct PlannerCommandInterface : public CommandInterface
{
  PlannerCommandInterface();
  virtual ~PlannerCommandInterface();
  virtual string Name() const { return "UnnamedPlanner"; }
  virtual string Description() const { return "Unnamed planner interface"; }
  virtual string Instructions() const { return "Click and drag to set planner goal"; }

  virtual string ActivateEvent(bool enabled);
  virtual void DrawGL();
  virtual string MouseInputEvent(int mx,int my,bool drag);
  virtual string SpaceballEvent(const RigidTransform& T);
  virtual string UpdateEvent();

  bool dragging;
  int mousex,mousey;
  int currentLink; 
  Vector3 currentPoint;
  Vector3 currentDestination;
  RealTimePlannerBase* planner;
  double lastPlanTime;
  double nextPlanTime;
};

struct IKPlannerCommandInterface : public PlannerCommandInterface
{
  virtual string Name() const { return "IKPointPoser"; }
  virtual string Description() const { return "Smart point poser"; }
  virtual string Instructions() const { return "Click and drag to pose points"; }

  virtual string ActivateEvent(bool enabled);

  SmartPointer<SingleRobotCSpace> cspace;
};

struct RRTCommandInterface : public PlannerCommandInterface
{
  virtual string Name() const { return "RRTPointPoser"; }
  virtual string Description() const { return "Goal-based point poser"; }
  virtual string Instructions() const { return "Click and drag to set the goal for a point"; }
  virtual string ActivateEvent(bool enabled);

  //TODO: draw the plan feedback?
  //GLuint planDisplayList;

  SmartPointer<SingleRobotCSpace> cspace;
};

#endif
