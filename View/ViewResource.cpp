#include "ViewResource.h"
#include "Planning/RobotCSpace.h"
#include "Planning/RobotTimeScaling.h"
#include "Modeling/Interpolate.h"
#include <GLdraw/drawgeometry.h>
#include <robotics/IKFunctions.h>
#include <sstream>

void drawgeom(const GeometricPrimitive3D& geom);

ViewResource::ViewResource(Robot* robot)
{
  SetRobot(robot);
  pathTime = 0;

  pathIKResolution = 0.005;
}

void ViewResource::SetRobot(Robot* robot)
{
  configViewer.robot = robot;
  configsViewer.robot = robot;
  pathViewer.robot = robot;
  configViewer.SetGrey();
  configsViewer.SetGrey();
  pathViewer.SetGrey();

}
void ViewResource::SetAnimTime(Real time)
{
  pathTime = time;
}

void ViewResource::DrawGL(const ResourcePtr& r)
{
  if(typeid(*r)==typeid(ConfigResource)) {
    if(configViewer.robot==NULL) return;
    const ConfigResource* rc=dynamic_cast<const ConfigResource*>((const ResourceBase*)r);
    Config oldq = configViewer.robot->q;
    if(rc->data.n != configViewer.robot->q.n) {
      fprintf(stderr,"Incorrect robot configuration size: %d vs %d\n",rc->data.n,configViewer.robot->q.n);
    }
    else {
      configViewer.robot->UpdateConfig(rc->data);
      configViewer.Draw();
    }
    configViewer.robot->UpdateConfig(oldq);
  }
  else if(typeid(*r)==typeid(ConfigsResource)) {
    if(configsViewer.robot==NULL) return;
    const ConfigsResource* rc=dynamic_cast<const ConfigsResource*>((const ResourceBase*)r);
    Config oldq = configsViewer.robot->q;
    int skip = 1;
    if(rc->configs.size() > 50)
      skip = rc->configs.size()/50;
    for(size_t i=0;i<rc->configs.size();i+=skip) {
      if(rc->configs[i].n != configViewer.robot->q.n) {
	fprintf(stderr,"Incorrect robot configuration size: %d vs %d\n",rc->configs[i].n,configViewer.robot->q.n);
      }
      else {
	configsViewer.robot->UpdateConfig(rc->configs[i]);
	configsViewer.Draw();
      }
    }
    configsViewer.robot->UpdateConfig(oldq);
  }
  else if(typeid(*r)==typeid(LinearPathResource)) {
    const LinearPathResource* rc=dynamic_cast<const LinearPathResource*>((const ResourceBase*)r);
    if(!rc) return;
    RenderLinearPath(rc,pathTime);
  }
  else if(typeid(*r)==typeid(MultiPathResource)) {
    const MultiPathResource* rc=dynamic_cast<const MultiPathResource*>((const ResourceBase*)r);
    if(!rc) return;
    RenderMultiPath(rc,pathTime);
  }
  else if(typeid(*r)==typeid(StanceResource)) {
    const StanceResource* rc=dynamic_cast<const StanceResource*>((const ResourceBase*)r);
    stanceViewer.DrawHolds(rc->stance);
  }
  else if(typeid(*r)==typeid(GraspResource)) {
    const GraspResource* rc=dynamic_cast<const GraspResource*>((const ResourceBase*)r);
    graspViewer.viewRobot = &configViewer;
    graspViewer.Draw(rc->grasp);
  }
  else if(typeid(*r)==typeid(HoldResource)) {
    const HoldResource* rc=dynamic_cast<const HoldResource*>((const ResourceBase*)r);
    holdViewer.Draw(rc->data);
  }
  else if(typeid(*r)==typeid(GeometricPrimitive3DResource)) {
    const GeometricPrimitive3DResource* rc=dynamic_cast<const GeometricPrimitive3DResource*>((const ResourceBase*)r);
    drawgeom(rc->data);
  }
}


void ViewResource::RenderLinearPath(const LinearPathResource* rc,Real pathTime)
{
  if(pathViewer.robot==NULL) {
    printf("ViewResource: Robot is NULL\n");
    return;
  }
  Config oldq = pathViewer.robot->q;
  if(rc->times.empty()) {
  }
  else if(rc->times.front() == rc->times.back()) {
    pathViewer.robot->UpdateConfig(rc->milestones[0]);
    pathViewer.Draw();
  }
  else {
    //TODO: faster tracking using upper_bound?
    Assert(rc->times.size()==rc->milestones.size());
    Assert(rc->times.back() > rc->times.front());
    Real normalizedPathTime = pathTime;
    //looping behavior
    /*
      while(normalizedPathTime < rc->times.front()) 
      normalizedPathTime += rc->times.back()-rc->times.front();
      while(normalizedPathTime > rc->times.back()) 
      normalizedPathTime -= rc->times.back()-rc->times.front();
      pathTime = normalizedPathTime;
    */
    //bouncing behavior
    double cnt = (pathTime-rc->times.front())/(rc->times.back()-rc->times.front());
    int n = (int)Floor(cnt);
    if(n%2==0)
      normalizedPathTime = (cnt-n)*(rc->times.back()-rc->times.front());
    else
      normalizedPathTime = rc->times.back()-(cnt-n)*(rc->times.back()-rc->times.front());
    bool drawn=false;
    for(size_t i=0;i+1<rc->times.size();i++) {
      Assert(rc->milestones[i].n == oldq.n);
      Assert(rc->milestones[i+1].n == oldq.n);
      if(rc->times[i] <= normalizedPathTime && normalizedPathTime <= rc->times[i+1]) {
	Real u=(normalizedPathTime-rc->times[i])/(rc->times[i+1]-rc->times[i]);
	Vector q;
	Interpolate(*pathViewer.robot,rc->milestones[i],rc->milestones[i+1],u,q);
	pathViewer.robot->UpdateConfig(q);
	pathViewer.Draw();
	drawn=true;
	break;
      }
    }
  }
  pathViewer.robot->UpdateConfig(oldq);
}

void ViewResource::RenderMultiPath(const MultiPathResource* rc,Real pathTime)
{
  if(pathViewer.robot==NULL) {
    printf("ViewResource: Robot is NULL\n");
    return;
  }
  Config oldq = pathViewer.robot->q;

  Real minTime = 0, maxTime = 1;
  if(rc->path.HasTiming()) {
    minTime = rc->path.sections.front().times.front();
    maxTime = rc->path.sections.back().times.back();
  }
  else
    pathTime /= rc->path.sections.size();
  //do bouncing behavior
  double cnt = (pathTime-minTime)/(maxTime-minTime);
  int n = (int)Floor(cnt);
  Real normalizedPathTime;
  if(n%2==0)
    normalizedPathTime = (cnt-n)*(maxTime-minTime)+minTime;
  else
    normalizedPathTime = maxTime-(cnt-n)*(maxTime-minTime);

  Config q;
  EvaluateMultiPath(*pathViewer.robot,rc->path,normalizedPathTime,q,pathIKResolution);
  pathViewer.robot->UpdateConfig(q);
  pathViewer.Draw();
  pathViewer.robot->UpdateConfig(oldq);

  int seg=rc->path.TimeToSection(normalizedPathTime);
  Assert(seg >= 0 && seg < (int)rc->path.sections.size());
  Stance s;
  rc->path.GetStance(s,seg);
  stanceViewer.DrawHolds(s);
}
