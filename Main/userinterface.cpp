#include "userinterface.h"
#include "Planning/RealTimePlanner.h"
#include "Simulation/WorldSimulation.h"
#include "Val3/Val3Interface.h"
#include "Simulation/FeedforwardController.h"
#include "Simulation/LoggingController.h"
#include "Simulation/PathController.h"
#include <Robotics/IKFunctions.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawExtra.h>
#include <sstream>

typedef MilestonePathController MyController;
inline MyController* GetController(RobotController* rc)
{
  return dynamic_cast<MilestonePathController*>(dynamic_cast<LoggingController<FeedforwardController>* >(rc)->base);
}

void PrintConfig(ostream& out,const vector<double>& q)
{
  for(size_t i=0;i<q.size();i++) out<<q[i]<<" ";
}

double MaxAbsError(const vector<double>& a,const vector<double>& b)
{
  assert(a.size()==b.size());
  double e=0;
  for(size_t i=0;i<a.size();i++)
    e = Max(e,Abs(a[i]-b[i]));
  return e;
}

CommandInterface::CommandInterface()
:world(NULL),sim(NULL),val3(NULL),viewport(NULL),
 minimumLatency(0.05),resetLatencyMultiplier(2.0)
{}

void CommandInterface::GetClickRay(int mx,int my,Ray3D& ray) const
{
  viewport->getClickSource(mx,viewport->h-my,ray.source);
  viewport->getClickVector(mx,viewport->h-my,ray.direction);
}

  //interfaces to simulated or actual world
Real CommandInterface::GetCurTime()
{
  if(sim) return sim->time;
  else if(val3) return val3->GetCurTime();
  return 0;
}

void CommandInterface::GetCurConfig(Config& x)
{
  if(sim) sim->controlSimulators[0].GetCommandedConfig(x);
  else if(val3) val3->GetConfig(x);
}

void CommandInterface::GetCurVelocity(Config& dx)
{
  if(sim) sim->controlSimulators[0].GetCommandedVelocity(dx);
  else if(val3) val3->GetVelocity(dx);
}

Real CommandInterface::GetEndTime()
{
  if(sim) return sim->time+GetController(sim->robotControllers[0])->TimeRemaining();
  else if(val3) return val3->client.GetTrajEndTime();
  return 0;
}

void CommandInterface::GetEndConfig(Config& x)
{
  if(sim) x=GetController(sim->robotControllers[0])->Endpoint();
  else if(val3) val3->GetEndConfig(x);
}

void CommandInterface::GetEndVelocity(Config& dx)
{
  if(sim) dx=GetController(sim->robotControllers[0])->EndpointVelocity();
  else if(val3) val3->GetEndVelocity(dx);
}

SafeTrajClient::MotionResult CommandInterface::SendMilestone(const Config& x)
{
  if(sim) GetController(sim->robotControllers[0])->AddMilestone(x);
  else if(val3) {
    return val3->MoveTo(x);
  }
  return SafeTrajClient::Success;
}

SafeTrajClient::MotionResult CommandInterface::SendMilestoneImmediate(const Config& x)
{
  if(sim) GetController(sim->robotControllers[0])->SetMilestone(x);
  else if(val3) {
    double delay = Max(minimumLatency,val3->client.GetAverageLatency()*resetLatencyMultiplier);
    return val3->MoveToImmediate(delay,x);
  }
  return SafeTrajClient::Success;
}

SafeTrajClient::MotionResult CommandInterface::SendPathImmediate(Real tbreak,const ParabolicRamp::DynamicPath& path)
{
  if(sim) {
    MyController* c=GetController(sim->robotControllers[0]);
    if(tbreak > sim->time) {
      ParabolicRamp::DynamicPath curPath,temp,temp2;
      c->GetPath(curPath);
      curPath.Split(tbreak-sim->time,temp,temp2);
      temp.Concat(path);
      c->SetPath(temp);
    }
    else 
      c->SetPath(path);
    return SafeTrajClient::Success;
  }
  else if(val3) {
    double delay = Max(minimumLatency,val3->client.GetAverageLatency()*resetLatencyMultiplier);
    if(tbreak < GetCurTime()+delay)
      printf("SendPathImmediate: Warning, break time %g < time %g + estimated delay %g\n",tbreak,GetCurTime(),delay);

    if(path.Empty()) {
      SafeTrajClient::MotionResult res=val3->client.ResetTrajectoryAbs(tbreak);
      return res;
    }
    else {
      //Config q;
      //val3->GetConfig(q);
      //cout<<"Current config: "<<q<<endl;
      //val3->GetEndConfig(q);
      //cout<<"Current end config: "<<q<<endl;
      //val3->GetEndVelocity(q);
      //cout<<"Current end velocity: "<<q<<endl;

      //cout<<"New trajectory start config: "<<path.ramps.front().x0<<endl;
      //cout<<"New trajectory start velocity: "<<path.ramps.front().dx0<<endl;
      for(size_t i=0;i<path.ramps.size();i++) {
	//assert();
	//assert(GetRobot()->InJointLimits(path.ramps[i].x1));
	//Vector bmin,bmax;
	//path.ramps[i].Bound(0,path.ramps[i].endTime,bmin,bmax);
	//assert(GetRobot()->InJointLimits(bmin));
	//assert(GetRobot()->InJointLimits(bmax));
	if(i > 0) {
	  assert(path.ramps[i].x0 == path.ramps[i-1].x1);
	  assert(path.ramps[i].dx0 == path.ramps[i-1].dx1);
	}
      }
      assert(path.IsValid());
      SafeTrajClient::MotionResult res=val3->MoveRampsImmediate(tbreak,path.ramps);
      if(res != SafeTrajClient::Success) fprintf(stderr,"MoveRampsImmediate failed, code %d\n",res);

      /*
      cout<<"New trajectory goal config: "<<path.ramps.back().x1<<endl;
      cout<<"New trajectory goal velocity: "<<path.ramps.back().dx1<<endl;
      val3->GetEndConfig(q);
      cout<<"New end config: "<<q<<endl;
      val3->GetEndVelocity(q);
      cout<<"New end velocity: "<<q<<endl;
      */
      return res;
    }
  }
  return SafeTrajClient::Success;
}





string JointCommandInterface::ActivateEvent(bool enabled)
{
  currentLink=-1;
  sendCommand = false;
  return "";
}
 
string JointCommandInterface::MouseInputEvent(int mx,int my,bool drag) 
{ 
  if(drag) {
    if(currentLink >= 0) {
      //alter current desired configuration
      Config q;
      GetEndConfig(q);
      Robot* robot=GetRobot();
      robot->UpdateConfig(q);
      for(size_t i=0;i<robot->drivers.size();i++) {
	if(robot->DoesDriverAffect(i,currentLink)) {
	  Real val = robot->GetDriverValue(i);
	  val = Clamp(val+my*0.02,robot->drivers[i].qmin,robot->drivers[i].qmax);
	  robot->SetDriverValue(i,val);
	}
      }
      command = robot->q;
      sendCommand = true;
    }
    stringstream ss;
    ss<<"Drag "<<currentLink<<" "<<mx<<" "<<my<<endl;
    return ss.str();
  }
  else {
    Ray3D ray;
    GetClickRay(mx,my,ray);

    Config q;
    GetCurConfig(q);
    Robot* robot=GetRobot();
    robot->UpdateConfig(q);
    int link;
    Vector3 localPos;
    RobotInfo* rob=world->ClickRobot(ray,link,localPos);

    if(rob) {
      currentLink = link;
      world->robots[0].view.SetGrey();
      world->robots[0].view.colors[currentLink].set(1,1,0);
    }
    else {
      world->robots[0].view.SetGrey();
      currentLink = -1;
    }
    return "";
  }
}

string JointCommandInterface::UpdateEvent()
{
  if(sendCommand) {
    SendMilestoneImmediate(command);
    sendCommand = false;
  }
  return "";
}


string IKCommandInterface::ActivateEvent(bool enabled)
{
  currentLink=-1;
  sendCommand = false;
  return "";
}

void IKCommandInterface::DrawGL()
{
  if(currentLink >= 0) {
    glPointSize(5.0);
    glEnable(GL_POINT_SMOOTH);
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    glColor3f(0,1,1);
    glVertex3v(GetRobot()->links[currentLink].T_World*currentPoint);
    glColor3f(0,1,0.5);
    glVertex3v(currentDestination);
    glEnd();
  }
}

string IKCommandInterface::MouseInputEvent(int mx,int my,bool drag)
{ 
  if(drag) {
    if(currentLink >= 0) {
      //alter current desired configuration
      Config q;
      GetEndConfig(q);
      Robot* robot=GetRobot();

      robot->UpdateConfig(q);
      Vector3 wp = currentDestination;
      Vector3 ofs;
      Vector3 vv;
      viewport->getViewVector(vv);
      Real d = (wp-viewport->position()).dot(vv);
      viewport->getMovementVectorAtDistance(mx,-my,d,ofs);
      currentDestination = wp+ofs;

      IKGoal goal;
      goal.link = currentLink;
      goal.localPosition = currentPoint;
      goal.SetFixedPosition(currentDestination);

      vector<IKGoal> problem(1,goal);
      int iters=100;
      bool res=SolveIK(*robot,problem,1e-3,iters,0);

      command = robot->q;
      sendCommand = true;
    }
    stringstream ss;
    ss<<"Drag "<<currentLink<<" "<<currentPoint<<" "<<mx<<" "<<my<<endl;
    return ss.str();
  }
  else {
    Ray3D ray;
    GetClickRay(mx,my,ray);

    Config q;
    GetCurConfig(q);
    Robot* robot=GetRobot();
    robot->UpdateConfig(q);

    int link;
    Vector3 localPos;
    RobotInfo* rob=world->ClickRobot(ray,link,localPos);
    if(rob) {
      currentLink = link;
      currentPoint = localPos;
      currentDestination = robot->links[link].T_World*localPos;
      world->robots[0].view.SetGrey();
      world->robots[0].view.colors[currentLink].set(1,1,0);
    }
    else {
      world->robots[0].view.SetGrey();
      currentLink = -1;
    }
    return "";
  }
}

string IKCommandInterface::SpaceballEvent(const RigidTransform& T)
{
  if(currentLink >= 0) {
    RigidTransform Tlink;
    Config q;
    GetEndConfig(q);
    Robot* robot=GetRobot();
    robot->UpdateConfig(q);
    Tlink = robot->links[currentLink].T_World;

    IKGoal goal;
    goal.link = currentLink;
    goal.localPosition.setZero();
    RigidTransform Tgoal = Tlink*T;
    goal.SetFixedRotation(Tgoal.R);
    goal.SetFixedPosition(Tgoal.t);

    vector<IKGoal> problem(1,goal);
    int iters=100;
    bool res=SolveIK(*robot,problem,1e-3,iters,0);

    command = robot->q;
    sendCommand = true;
  }
  return "";
}

string IKCommandInterface::UpdateEvent()
{
  if(sendCommand) {
    SendMilestoneImmediate(command);
    sendCommand = false;
  }
  return "";
}



PlannerCommandInterface::PlannerCommandInterface()
  :planner(NULL),lastPlanTime(0),nextPlanTime(0)
{}

PlannerCommandInterface::~PlannerCommandInterface()
{
  SafeDelete(planner);
}

string PlannerCommandInterface::ActivateEvent(bool enabled)
{
  dragging = false;
  currentLink=-1;
  planner->currentPath.ramps.resize(0);
  lastPlanTime = nextPlanTime = GetCurTime();
  return "";
}

void PlannerCommandInterface::DrawGL()
{
  if(planner && planner->goal) {
    CartesianObjective* ikgoal=(CartesianObjective*)(planner->goal);
    const IKGoal& goal=ikgoal->ikGoal;
    glPointSize(5.0);
    glEnable(GL_POINT_SMOOTH);
    glDisable(GL_LIGHTING);
    glBegin(GL_POINTS);
    glColor3f(0,1,1);
    glVertex3v(GetRobot()->links[goal.link].T_World*goal.localPosition);
    glColor3f(0,1,0.5);
    glVertex3v(goal.endPosition);
    glEnd();
  }
}

string PlannerCommandInterface::MouseInputEvent(int mx,int my,bool drag) 
{ 
  if(drag) {
    printf("Dragging event, dragging status %d\n",(int)dragging);
    if(!dragging) {
      //first click
      Ray3D ray;
      GetClickRay(mousex,mousey,ray);
      
      Config q;
      GetCurConfig(q);
      Robot* robot=GetRobot();
      robot->UpdateConfig(q);
      int link;
      Vector3 localPos;
      RobotInfo* rob=world->ClickRobot(ray,link,localPos);
      if(rob) {
	currentLink = link;
	currentPoint = localPos;
	currentDestination = robot->links[link].T_World*localPos;
	world->robots[0].view.SetGrey();
	world->robots[0].view.colors[currentLink].set(1,1,0);
      }
      else {
	world->robots[0].view.SetGrey();
	currentLink = -1;
      }
      dragging = true;
    }
    if(currentLink >= 0) {
      //alter current desired configuration
      Config q;
      GetEndConfig(q);

      Robot* robot=GetRobot();
      robot->UpdateConfig(q);
      Vector3 wp = currentDestination;
      Vector3 ofs;
      Vector3 vv;
      viewport->getViewVector(vv);
      Real d = (wp-viewport->position()).dot(vv);
      viewport->getMovementVectorAtDistance(mx,-my,d,ofs);
      currentDestination = wp + ofs;

      IKGoal goal;
      goal.link = currentLink;
      goal.localPosition = currentPoint;
      goal.SetFixedPosition(currentDestination);

      if(planner) {
	CartesianObjective* obj=new CartesianObjective(robot);
	obj->ikGoal = goal;
	planner->Reset(obj);
      }
    }
    stringstream ss;
    ss<<"Drag "<<currentLink<<" "<<currentPoint<<" "<<mx<<" "<<my<<endl;
    return ss.str();
  }
  else {
    printf("Stop dragging event, dragging status %d\n",(int)dragging);
    dragging = false;
    mousex=mx;
    mousey=my;
    return "";
  }
}


string PlannerCommandInterface::SpaceballEvent(const RigidTransform& T)
{
  if(currentLink >= 0) {
    RigidTransform Tlink;
    Config q;
    GetEndConfig(q);
    Robot* robot=GetRobot();
    robot->UpdateConfig(q);
    Tlink = robot->links[currentLink].T_World;

    IKGoal goal;
    goal.link = currentLink;
    goal.localPosition.setZero();
    goal.SetFixedPosition(Tlink.t+T.t);
    goal.SetFixedRotation(Tlink.R*T.R);

    if(planner) {
      CartesianObjective* obj=new CartesianObjective(robot);
      obj->ikGoal = goal;
      if(planner) planner->Reset(obj);
    }
  }
  return "";
}

string PlannerCommandInterface::UpdateEvent()
{
  if(!planner) return "";
  if(planner->currentPath.ramps.empty()) {
    if(GetEndTime() <= GetCurTime()) { //done moving
      printf("Planner initialized\n");
      Config q;
      GetCurConfig(q);
      planner->currentPath.Init(GetRobot()->velMax,GetRobot()->accMax);
      planner->currentPath.ramps.resize(1);
      planner->currentPath.ramps[0].SetConstant(q);
    }
    else
      //wait until done moving
      return "";
  }

  //wait until next planning step, otherwise try planning
  if(GetCurTime()  < nextPlanTime) return "";

  stringstream ss;

  //advance modeled trajectory
  double t = GetCurTime();
  double startPlanTime = t;
  ParabolicRamp::DynamicPath before,updatedCurrent;
  planner->currentPath.Split(startPlanTime-lastPlanTime,before,updatedCurrent);
  assert(updatedCurrent.IsValid());
  planner->currentPath = updatedCurrent;
  //cout<<"Path advance "<<startPlanTime-lastPlanTime<<endl;
  /*
  //TODO: debug with mirror config at time startPlanTime, not the queried
  //config from the server
  Config qcur;
  GetCurConfig(qcur);
  //double check the assumptions of the planner
  if(MaxAbsError(planner->currentPath.ramps.front().x0,qcur)>1e-2) {
    cout<<"Major discrepancy between predicted and actual path"<<endl;
    cout<<"\tCur predicted config "; PrintConfig(cout,planner->currentPath.ramps.front().x0); cout<<endl;
    cout<<"\tCur actual config "; PrintConfig(cout,qcur); cout<<endl;
    getchar();
    
    GetCurConfig(qcur);
    if(GetEndTime() <= GetCurTime()) {
      planner->currentPath.ramps.resize(1);
      planner->currentPath.ramps[0].SetConstant(qcur);
      updatedCurrent = planner->currentPath;
      startPlanTime = GetCurTime();
    }
  }
  */
  //cout<<"End time "<<GetEndTime()<<", predicted end time "<<t+updatedCurrent.GetTotalTime()<<endl;    
  
  Timer timer;
  Real splitTime,planTime;
  bool res=false;
  if(currentLink >= 0) {
    res=planner->PlanUpdate(splitTime,planTime);
    printf("Plan update: time %g, time elapsed %g, result %d\n",t,planTime,(int)res);
    
    ss<<"Plan "<<planTime<<", padding "<<planner->currentPadding<<", split "<<splitTime<<", res "<<res<<endl;
    assert(planner->currentPath.IsValid());
  }
  lastPlanTime = startPlanTime;
  
  //some time elapsed
  t = GetCurTime();
  //getchar();
  if(res){
    nextPlanTime = t + splitTime;

    //impose hard real time constraint
    if(t < startPlanTime + splitTime) {
      printf("Plan time elapsed = %g, split time ... %g\n",t-startPlanTime,splitTime);
      ParabolicRamp::DynamicPath temp2;
      planner->currentPath.Split(splitTime,before,temp2);
      assert(planner->currentPath.IsValid());
      assert(temp2.IsValid());
      SafeTrajClient::MotionResult res=SendPathImmediate(startPlanTime+splitTime,temp2);
      if(res == SafeTrajClient::TransmitError) {
	//revert path
	printf("Warning, unable to send path to controller\n");
	planner->currentPath = updatedCurrent;
	assert(updatedCurrent.IsValid());
	//getchar();
	ss<<", failed to send path";
	planner->MarkLastFailure();
      }
      else if(res != SafeTrajClient::Success) {
	printf("Hmmm.. failed path check.  Should debug!\n");
	//revert path
	planner->currentPath = updatedCurrent;
	assert(updatedCurrent.IsValid());
      }
      
      //cout<<"End time "<<GetEndTime()<<", predicted end time "<<lastPlanTime+splitTime+updatedCurrent.GetTotalTime()<<endl; 
    }
    else {
      printf("Hard real-time constraint was violated by %gs\n",t-(startPlanTime+splitTime));
      //revert path
      planner->currentPath = updatedCurrent;
      assert(updatedCurrent.IsValid());
      planner->MarkLastFailure();
      //getchar();
      ss<<", violated real-time constraint "<<t-(startPlanTime+splitTime);
    }
  }
  else
    nextPlanTime = t;
  return ss.str();
}


string IKPlannerCommandInterface::ActivateEvent(bool enabled)
{
  if(!planner) {
    printf("IK planner activated\n");
    cspace = new SingleRobotCSpace(*world,0,settings);
    
    planner = new RealTimeIKPlanner;
    planner->protocol = RealTimePlannerBase::Constant;
    planner->currentSplitTime=0.1;
    planner->currentPadding=0.05;
    planner->robot = GetRobot();
    planner->settings = settings;
    planner->cspace = cspace;
  }
  return "";
}

string RRTCommandInterface::ActivateEvent(bool enabled)
{
  if(!planner) {
    cspace = new SingleRobotCSpace(*world,0,settings);
    
    RealTimeRRTPlanner* p=new RealTimeRRTPlanner;
    p->delta = 0.5;
    planner = p;
    //planner->protocol = RealTimePlannerBase::Constant;
    planner->protocol = RealTimePlannerBase::ExponentialBackoff;
    planner->currentSplitTime=0.1;
    planner->currentPadding=0.05;
    //test insufficient padding
    //planner->currentPadding=0.01;
    planner->robot = GetRobot();
    planner->settings = settings;
    planner->cspace = cspace;
  }
  return "";
}
