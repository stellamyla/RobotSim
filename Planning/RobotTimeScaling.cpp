#include "RobotTimeScaling.h"
#include "RobotConstrainedInterpolator.h"
#include "Modeling/SplineInterpolate.h"
#include "Modeling/Interpolate.h"
#include "TimeScaling.h"
#include "ConstrainedInterpolator.h"
#include <robotics/IKFunctions.h>
#include <Timer.h>
#include <sstream>
#include <fstream>

#define DO_CHECK_BOUNDS 0
#define DO_SAVE_LIMITS 0
#define DO_TEST_TRIANGULAR 1

bool CheckBounds(Robot& robot,const TimeScaledBezierCurve& traj,const vector<Real>& times)
{
  Vector v,a,maxv,maxa;
  Vector oldv,diffa,maxdiffa;
  bool res=true;
  for(size_t i=0;i<times.size();i++) {
    //double-checking velocity and acceleration bounds
    traj.Deriv(times[i],v);
    traj.Accel(times[i],a);
    if(i==0) { maxv=v; maxa=a; }
    else {
      for(int j=0;j<v.n;j++) {
	maxv[j] = Max(maxv[j],Abs(v[j]));
	maxa[j] = Max(maxa[j],Abs(a[j]));
      }
    }
    for(int j=0;j<v.n;j++) {
      if(Abs(v[j]) > robot.velMax[j]+1e-3) {
	printf("Exceeded vel max %s=%g at time %g\n",robot.LinkName(j).c_str(),v(j),times[i]);
	res = false;
      }
      if(Abs(a[j]) > robot.accMax[j]+1e-3) {
	printf("Exceeded accel max %s=%g at time %g\n",robot.LinkName(j).c_str(),a(j),times[i]);
	res = false; 
      }
    }
    if(!oldv.empty()) {
      diffa = (v-oldv)/(times[i]-times[i-1]);
      for(int j=0;j<v.n;j++) {
	if(Abs(diffa[j]) > robot.accMax[j]+1e-3) {
	  printf("Diff accel max %s=%g at time %g\n",robot.LinkName(j).c_str(),diffa(j),times[i]);
	  res = false;
	}
      }	    
    }
    oldv = v;
  }
  cout<<"Max vel "<<maxv<<endl;
  cout<<"Max accel "<<maxa<<endl;
  cout<<"End vel "<<v<<endl;
  cout<<"End accel "<<a<<endl;
  return res;
}


bool CheckBounds(Robot& robot,const TimeScaledBezierCurve& traj,Real dt)
{
  Real T=traj.EndTime();
  int numdivs = (int)Ceil(T/dt);
  vector<Real> times(numdivs);
  for(int i=0;i<numdivs;i++) 
    times[i] = T*Real(i)/Real(numdivs-1);  
  return CheckBounds(robot,traj,times);
}


void SaveLimits(Robot& robot,const TimeScaledBezierCurve& traj,Real dt,const char* fn)
{
  Real T=traj.EndTime();
  int numdivs = (int)Ceil(T/dt);
  printf("Saving time-scaled values to %s\n",fn);
  ofstream out(fn,ios::out);
  out<<"t";
  for(size_t i=0;i<robot.links.size();i++)
    out<<",q["<<robot.linkNames[i]<<"]";
  for(size_t i=0;i<robot.links.size();i++)
    out<<",v["<<robot.linkNames[i]<<"]";
  for(size_t i=0;i<robot.links.size();i++)
    out<<",a["<<robot.linkNames[i]<<"]";
  out<<",activeLimit,saturation";
  out<<endl;

  Vector q,v,a,maxv,maxa;
  for(int i=0;i<=numdivs;i++) {
    //double-checking velocity and acceleration bounds
    Real t = Real(i)/Real(numdivs)*T;
    traj.Eval(t,q);
    traj.Deriv(t,v);
    traj.Accel(t,a);
    out<<t;
    for(int j=0;j<q.n;j++)
      out<<","<<q(j);
    for(int j=0;j<v.n;j++)
      out<<","<<v(j);
    for(int j=0;j<a.n;j++)
      out<<","<<a(j);
    Real maxSat=0;
    int maxSatInd=0;
    bool maxSatA=false;
    for(int j=0;j<v.n;j++)
      if(Abs(v(j))/robot.velMax(j) > maxSat) {
	maxSat = Abs(v(j))/robot.velMax(j);
	maxSatInd = j;
      }
    for(int j=0;j<a.n;j++)
      if(Abs(a(j))/robot.accMax(j) > maxSat) {
	maxSat = Abs(a(j))/robot.accMax(j);
	maxSatInd = j;
	maxSatA=true;
      }
    if(maxSatA)
      out<<",a["<<robot.linkNames[maxSatInd]<<"]";
    else
      out<<",v["<<robot.linkNames[maxSatInd]<<"]";
    out<<","<<maxSat<<endl;
  }
  out<<endl;
  for(size_t i=0;i<traj.timeScaling.times.size();i++) {
    //double-checking velocity and acceleration bounds
    Real t = traj.timeScaling.times[i];
    traj.Eval(t,q);
    traj.Deriv(t,v);
    traj.Accel(t,a);
    out<<t;
    for(int j=0;j<q.n;j++)
      out<<","<<q(j);
    for(int j=0;j<v.n;j++)
      out<<","<<v(j);
    for(int j=0;j<a.n;j++)
      out<<","<<a(j);
    Real maxSat=0;
    int maxSatInd=0;
    bool maxSatA=false;
    for(int j=0;j<v.n;j++)
      if(Abs(v(j))/robot.velMax(j) > maxSat) {
	maxSat = Abs(v(j))/robot.velMax(j);
	maxSatInd = j;
      }
    for(int j=0;j<a.n;j++)
      if(Abs(a(j))/robot.accMax(j) > maxSat) {
	maxSat = Abs(a(j))/robot.accMax(j);
	maxSatInd = j;
	maxSatA=true;
      }
    if(maxSatA)
      out<<",a["<<robot.linkNames[maxSatInd]<<"]";
    else
      out<<",v["<<robot.linkNames[maxSatInd]<<"]";
    out<<","<<maxSat<<endl;
  }
}


bool TimeOptimizePath(Robot& robot,const vector<Real>& oldtimes,const vector<Config>& oldconfigs,Real dt,vector<Real>& newtimes,vector<Config>& newconfigs)
{
  //make a smooth interpolator
  Vector dx0,dx1,temp;
  RobotCSpace cspace(robot);
  RobotGeodesicManifold manifold(robot);
  TimeScaledBezierCurve traj;
  traj.path.durations.resize(oldconfigs.size()-1);
  for(size_t i=0;i+1<oldconfigs.size();i++) {
    traj.path.durations[i] = oldtimes[i+1]-oldtimes[i];
    Assert(traj.path.durations[i] > 0);
  }
  traj.path.segments.resize(oldconfigs.size()-1);
  for(size_t i=0;i+1<oldconfigs.size();i++) {
    traj.path.segments[i].space = &cspace;
    traj.path.segments[i].manifold = &manifold;
    traj.path.segments[i].x0 = oldconfigs[i];
    traj.path.segments[i].x3 = oldconfigs[i+1];
    if(i > 0) {
      InterpolateDerivative(robot,oldconfigs[i],oldconfigs[i+1],dx0);
      InterpolateDerivative(robot,oldconfigs[i],oldconfigs[i-1],temp);
      dx0 -= temp*(traj.path.durations[i]/traj.path.durations[i-1]);
      dx0 *= 0.5;
    }
    else dx0.resize(0);
    if(i+2 < oldconfigs.size()) {
      InterpolateDerivative(robot,oldconfigs[i+1],oldconfigs[i+2],dx1);
      InterpolateDerivative(robot,oldconfigs[i+1],oldconfigs[i],temp);
      dx1 *= traj.path.durations[i]/traj.path.durations[i+1];
      dx1 -= temp;
      dx1 *= 0.5;
    }
    else dx1.resize(0);
    traj.path.segments[i].SetNaturalTangents(dx0,dx1);
  }
  Timer timer;
  bool res=traj.OptimizeTimeScaling(robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
  if(!res) {
    printf("Failed to optimize time scaling\n");
    return false;
  }
  else {
    printf("Optimized into a path with duration %g, (took %gs)\n",traj.EndTime(),timer.ElapsedTime());
  }
  double T = traj.EndTime();
  int numdivs = (int)Ceil(T/dt);

  printf("Discretizing at time resolution %g\n",T/numdivs);
  numdivs++;
  newtimes.resize(numdivs);
  newconfigs.resize(numdivs);

  for(int i=0;i<numdivs;i++) {
    newtimes[i] = T*Real(i)/Real(numdivs-1);
    traj.Eval(newtimes[i],newconfigs[i]);
  }
#if DO_CHECK_BOUNDS
  CheckBounds(robot,traj,newtimes);
#endif  //DO_CHECKBOUNDS
  return true;
}

bool InterpolateConstrainedPath(Robot& robot,const Config& a,const Config& b,const vector<IKGoal>& ikGoals,vector<Config>& milestones,Real xtol)
{
  RobotConstrainedInterpolator interp(robot,ikGoals);
  interp.ftol = xtol*1e-2;
  interp.xtol = xtol;
  if(!interp.Make(a,b,milestones)) return false;
  return true;
  /*
  RobotIKFunction f(robot);
  for(size_t i=0;i<ikGoals.size();i++)
    f.UseIK(ikGoals[i]);
  GetDefaultIKDofs(robot,ikGoals,f.activeDofs);
  ActiveRobotCSpace space(robot,f.activeDofs);
  ActiveRobotGeodesicManifold manifold(robot);
  Config activeA(f.activeDofs.Size()),activeB(f.activeDofs.Size());
  f.activeDofs.InvMap(a,activeA);
  f.activeDofs.InvMap(b,activeB);
  
  ConstrainedInterpolator interp(&space,&f);
  interp.ftol = xtol*1e-2;
  interp.xtol = xtol;
  if(!interp.Make(activeA,activeB,milestones)) {
    return false;
  }

  //lift milestones back into original space
  Vector q;
  for(size_t i=0;i<milestones.size();i++) {
    Real time = Real(i)/Real(milestones.size()-1); 
    Interpolate(robot,a,b,time,q);
    f.activeDofs.Map(milestones[i],q);
    milestones[i] = q;
  }
  return true;
  */
}

bool InterpolateConstrainedPath(Robot& robot,const vector<Config>& milestones,const vector<IKGoal>& ikGoals,vector<Config>& path,Real xtol)
{
  RobotSmoothConstrainedInterpolator interp(robot,ikGoals);
  interp.ftol = xtol*1e-2;
  interp.xtol = xtol;
  GeneralizedCubicBezierSpline spline;
  if(!MultiSmoothInterpolate(interp,milestones,spline)) return false;
  path.resize(spline.segments.size()+1);
  path[0] = spline.segments[0].x0;
  for(size_t i=0;i<spline.segments.size();i++)
    path[i+1] = spline.segments[i].x3;
  return true;
}

void SmoothDiscretizePath(Robot& robot,const vector<Config>& oldconfigs,int n,vector<Real>& times,vector<Config>& configs)
{
  times.resize(n);
  configs.resize(n);
  RobotCSpace space(robot);
  RobotGeodesicManifold geodesic(robot);
  vector<GeneralizedCubicBezierCurve> curves;
  //SplineInterpolate(oldconfigs,curves,&space,&geodesic);
  MonotonicInterpolate(oldconfigs,curves,&space,&geodesic);

  times[0] = 0;
  for(int i=1;i<n;i++) 
    times[i] = Real(i)/Real(n-1);
  configs[0] = oldconfigs[0];
  configs.back() = oldconfigs.back();
  Vector temp;
  for(int i=1;i+1<n;i++) {
    Real u0 = Floor(times[i]*curves.size());
    int segind = (int)u0;
    Real u=times[i]*curves.size()-u0;
    curves[segind].Eval(u,configs[i]);
  }
}


bool InterpolateConstrainedMultiPath(Robot& robot,const MultiPath& path,vector<GeneralizedCubicBezierSpline>& paths,Real xtol)
{
  //sanity check -- make sure it's a continuous path
  if(!path.IsContinuous()) {
    fprintf(stderr,"InterpolateConstrainedMultiPath: path is discontinuous\n");
    return false;
  }
  if(path.sections.empty()) {
    fprintf(stderr,"InterpolateConstrainedMultiPath: path is empty\n");
    return false;
  }

  if(path.settings.count("resolution") != 0) {
    //see if the resolution is high enough to just interpolate directly
    stringstream ss(path.settings.find("resolution")->second);
    Real res;
    ss >> res;
    if(res <= xtol) {
      printf("Direct interpolating trajectory with res %g\n",res);
      //just interpolate directly
      RobotCSpace space(robot);
      RobotGeodesicManifold manifold(robot);
      paths.resize(path.sections.size());
      for(size_t i=0;i<path.sections.size();i++) {
	/** TEMP */
	if(path.sections[i].times.empty()) {
	  //MonotonicInterpolate(path.sections[i].milestones,paths[i].segments,
	  SplineInterpolate(path.sections[i].milestones,paths[i].segments,
			       &space,&manifold);
	  //uniform timing
	  paths[i].durations.resize(paths[i].segments.size());
	  Real dt=1.0/Real(paths[i].segments.size());
	  for(size_t j=0;j<paths[i].segments.size();j++)
	    paths[i].durations[j] = dt;
	}
	else {
	  //MonotonicInterpolate(path.sections[i].milestones,path.sections[i].times,paths[i].segments,
	  SplineInterpolate(path.sections[i].milestones,path.sections[i].times,paths[i].segments,
			       &space,&manifold);
	  //get timing from path
	  paths[i].durations.resize(paths[i].segments.size());
	  for(size_t j=0;j<paths[i].segments.size();j++)
	    paths[i].durations[j] = path.sections[i].times[j+1]-path.sections[i].times[j];
	}
      }
      return true;
    }
  }
  printf("Discretizing constrained trajectory at res %g\n",xtol);

  RobotCSpace cspace(robot);
  RobotGeodesicManifold manifold(robot);

  //create transition constraints and derivatives
  vector<vector<IKGoal> > stanceConstraints(path.sections.size());
  vector<vector<IKGoal> > transitionConstraints(path.sections.size()-1);
  vector<Config> transitionDerivs(path.sections.size()-1);
  for(size_t i=0;i<path.sections.size();i++) 
    path.GetIKProblem(stanceConstraints[i],i);
  for(size_t i=0;i+1<path.sections.size();i++) {
    //put all nonredundant constraints into transitionConstraints[i]
    transitionConstraints[i]=stanceConstraints[i];
    for(size_t j=0;j<stanceConstraints[i+1].size();j++) {
      bool res=AddGoalNonredundant(stanceConstraints[i+1][j],transitionConstraints[i]);
      if(!res) {
	fprintf(stderr,"Conflict between goal %d of stance %d and stance %d\n",j,i+1,i);
	fprintf(stderr,"  Link %d\n",stanceConstraints[i+1][j].link);
	return false;
      }
    }

    const Config& prev=path.sections[i].milestones[path.sections[i].milestones.size()-2];
    const Config& next=path.sections[i+1].milestones[1];
    manifold.InterpolateDeriv(prev,next,0.5,transitionDerivs[i]);
    transitionDerivs[i] *= 0.5;

    //check for overshoots a la MonotonicInterpolate
    Vector inslope,outslope;
    manifold.InterpolateDeriv(prev,path.sections[i].milestones.back(),1.0,inslope);
    manifold.InterpolateDeriv(path.sections[i].milestones.back(),next,0.0,outslope);
    for(int j=0;j<transitionDerivs[i].n;j++) {
      if(Sign(transitionDerivs[i][j]) != Sign(inslope[j]) || Sign(transitionDerivs[i][j]) != Sign(outslope[j])) transitionDerivs[i][j] = 0;
      else {
	if(transitionDerivs[i][j] > 0) {
	  if(transitionDerivs[i][j] > 3.0*outslope[j])
	    transitionDerivs[i][j] = 3.0*outslope[j];
	  if(transitionDerivs[i][j] > 3.0*inslope[j])
	    transitionDerivs[i][j] = 3.0*inslope[j];
	}
	else {
	  if(transitionDerivs[i][j] < 3.0*outslope[j])
	    transitionDerivs[i][j] = 3.0*outslope[j];
	  if(transitionDerivs[i][j] < 3.0*inslope[j])
	    transitionDerivs[i][j] = 3.0*inslope[j];
	}
      }
    }

    //project "natural" derivative onto transition manifold
    RobotIKFunction f(robot);
    f.UseIK(transitionConstraints[i]);
    GetDefaultIKDofs(robot,transitionConstraints[i],f.activeDofs);
    Vector temp(f.activeDofs.Size()),dtemp(f.activeDofs.Size()),dtemp2;
    f.activeDofs.InvMap(path.sections[i].milestones.back(),temp);
    f.activeDofs.InvMap(transitionDerivs[i],dtemp);
    Matrix J;
    f.PreEval(temp);
    f.Jacobian(temp,J);
    RobustSVD<Real> svd;
    if(!svd.set(J)) {
      fprintf(stderr,"Unable to set SVD of transition constraints %d\n",i);
      return false;
    }
    svd.nullspaceComponent(dtemp,dtemp2);
    dtemp -= dtemp2;
    f.activeDofs.Map(dtemp,transitionDerivs[i]);
  }

  //start constructing path
  paths.resize(path.sections.size());   
  for(size_t i=0;i<path.sections.size();i++) {
    paths[i].segments.resize(0);
    paths[i].durations.resize(0);

    RobotSmoothConstrainedInterpolator interp(robot,stanceConstraints[i]);
    interp.xtol = xtol;
    Vector dxprev,dxnext;
    if(i>0) 
      dxprev.setRef(transitionDerivs[i-1]); 
    if(i<transitionDerivs.size()) 
      dxnext.setRef(transitionDerivs[i]); 
    if(!MultiSmoothInterpolate(interp,path.sections[i].milestones,dxprev,dxnext,paths[i])) {
      fprintf(stderr,"Unable to interpolate section %d\n",i);
      return false;
    }
    //set the time scale if the input path is timed
    if(!path.sections[i].times.empty())
      paths[i].TimeScale(path.sections[i].times.back()-path.sections[i].times.front());
  }
  return true;
}


bool DiscretizeConstrainedMultiPath(Robot& robot,const MultiPath& path,MultiPath& out,Real xtol)
{
  if(path.settings.count("resolution") != 0) {
    //see if the resolution is high enough to just interpolate directly
    stringstream ss(path.settings.find("resolution")->second);
    Real res;
    ss >> res;
    if(res <= xtol) {
      out = path;
      return true;
    }
  }

  vector<GeneralizedCubicBezierSpline> paths;
  if(!InterpolateConstrainedMultiPath(robot,path,paths,xtol))
    return false;

  out = path;
  {
    stringstream ss;
    ss<<xtol;
    out.settings["resolution"]=ss.str();
    out.settings["program"]="DiscretizeConstrainedMultiPath";
  }
  Real tofs = 0;
  for(size_t i=0;i<out.sections.size();i++) {
    out.sections[i].velocities.resize(0);
    paths[i].GetPiecewiseLinear(out.sections[i].times,out.sections[i].milestones);
    //shift section timing
    for(size_t j=0;j<out.sections[i].times.size();j++)
      out.sections[i].times[j] += tofs;
    tofs = out.sections[i].times.back();
  }
  return true;
}

Real OptimalTriangularTimeScaling(const GeneralizedCubicBezierSpline& path,const Vector& vmin,const Vector& vmax,const Vector& amin,const Vector& amax)
{
  Real pathlen = path.TotalTime();
  Real invpathlen = 1.0/pathlen;
  Real vmaxrel = 0, amaxrel = 0;
  Real u=0;
  for(size_t i=0;i<path.segments.size();i++) {
    Vector vimin,vimax,aimin,aimax;
    path.segments[i].GetDerivBounds(vimin,vimax,aimin,aimax);
    Real du = path.durations[i];
    //get the height of velocity profile, and speed at this point
    Real h;
    if(u < 0.5*pathlen && u+du > 0.5*pathlen) {
      h = 1.0;
    }
    else if(u > 0.5*pathlen) {
      h  = 2.0*(1.0 - u*invpathlen);
    }
    else {
      h = 2.0*(u+du)*invpathlen;
    }
    for(int j=0;j<vmin.n;j++) {
      if(vimin[j]*h < vmaxrel*(vmin[j]*du)) {
	vmaxrel = vimin[j]*h/(vmin[j]*du);
      }
      if(vimax[j]*h > vmaxrel*(vmax[j]*du)) {
	vmaxrel = vimax[j]*h/(vmax[j]*du);
      }
    }
    if(u <= 0.5*pathlen) {
      //positive acceleration
      for(int j=0;j<vmin.n;j++) {
	if((aimin[j]*Sqr(h) + vimin[j]*du*invpathlen) < amaxrel*amin[j]*Sqr(du)) 
	  amaxrel = (aimin[j]*Sqr(h) + vimin[j]*du*invpathlen)/(amin[j]*Sqr(du));
	if((aimax[j]*Sqr(h) + vimax[j]*du*invpathlen) > amaxrel*amax[j]*Sqr(du)) 
	  amaxrel = (aimax[j]*Sqr(h) + vimax[j]*du*invpathlen)/(amax[j]*Sqr(du));
      }
    }
    if(u+du >= 0.5*pathlen) {
      //negative acceleration
      for(int j=0;j<vmin.n;j++) {
	if((aimin[j]*Sqr(h) - vimin[j]*du*invpathlen) < amaxrel*amin[j]*Sqr(du)) 
	  amaxrel = (aimin[j]*Sqr(h) - vimin[j]*du*invpathlen)/(amin[j]*Sqr(du));
	if((aimax[j]*Sqr(h) - vimax[j]*du*invpathlen) > amaxrel*amax[j]*Sqr(du)) 
	  amaxrel = (aimax[j]*Sqr(h) - vimax[j]*du*invpathlen)/(amax[j]*Sqr(du));

      }
    }
    u += du;
  }
  printf("Max relative velocity %g, acceleration %g\n",vmaxrel,amaxrel);
  //can set rate to Max(1.0/vmaxrel,1.0/Sqrt(amaxrel))
  //duration of ramp up is 2*pathlen/2 / rate, ramp down is the same
  return 2.0*pathlen*Max(vmaxrel,Sqrt(amaxrel));
}

bool GenerateAndTimeOptimizeMultiPath(Robot& robot,MultiPath& multipath,Real xtol,Real dt)
{
  Timer timer;
  vector<GeneralizedCubicBezierSpline > paths;
  if(!InterpolateConstrainedMultiPath(robot,multipath,paths,xtol))
    return false;
  printf("Generated interpolating path in time %gs\n",timer.ElapsedTime());
  //TEMP: save paths
  /*
  printf("Saving sections to section_x_bezier.csv\n");
  for(size_t i=0;i<paths.size();i++) {
    stringstream ss;
    ss<<"section_"<<i<<"_bezier.csv";
    ofstream out(ss.str().c_str(),ios::out);
    out<<"duration,x0,x1,x2,x3"<<endl;
    for(size_t j=0;j<paths[i].segments.size();j++) {
      out<<paths[i].durations[j]<<","<<paths[i].segments[j].x0<<","<<paths[i].segments[j].x1<<","<<paths[i].segments[j].x2<<","<<paths[i].segments[j].x3<<endl;
    }
    out.close();
  }
  */

  RobotCSpace cspace(robot);
  RobotGeodesicManifold manifold(robot);
  TimeScaledBezierCurve traj;
  vector<int> edgeToSection,sectionEdges(1,0);
  for(size_t i=0;i<multipath.sections.size();i++) {
    for(size_t j=0;j<paths[i].segments.size();j++) {
      paths[i].segments[j].space = &cspace;
      paths[i].segments[j].manifold = &manifold;
    }
    traj.path.Concat(paths[i]);
    for(size_t j=0;j<paths[i].segments.size();j++)
      edgeToSection.push_back((int)i);
    sectionEdges.push_back(sectionEdges.back()+(int)paths[i].segments.size());
  }

  timer.Reset();
  bool res=traj.OptimizeTimeScaling(robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
  if(!res) {
    printf("Failed to optimize time scaling\n");
    return false;
  }
  else {
    printf("Optimized into a path with duration %g, (took %gs)\n",traj.EndTime(),timer.ElapsedTime());
  }
  double T = traj.EndTime();
  int numdivs = (int)Ceil(T/dt);
  printf("Discretizing at time resolution %g\n",T/numdivs);
  numdivs++;

  Vector x,v;
  int sCur = -1;
  for(int i=0;i<numdivs;i++) {
    Real t=T*Real(i)/Real(numdivs-1);
    int trajEdge = traj.timeScaling.TimeToSegment(t);
    if(trajEdge == (int)edgeToSection.size()) trajEdge--; //end of path
    Assert(trajEdge < (int)edgeToSection.size());
    int s=edgeToSection[trajEdge];
    if(s < sCur) {
      fprintf(stderr,"Strange: edge index is going backward? %d -> %d\n",sCur,s);
      fprintf(stderr,"  time %g, division %d, traj segment %d\n",t,i,trajEdge);
    }
    Assert(s - sCur >=0);
    while(sCur < s) {
      //close off the current section and add a new one
      Real switchTime=traj.timeScaling.times[sectionEdges[sCur+1]];
      Assert(switchTime <= t);
      traj.Eval(switchTime,x);
      traj.Deriv(switchTime,v);
      if(sCur >= 0) {
	multipath.sections[sCur].times.push_back(switchTime);
	multipath.sections[sCur].milestones.push_back(x);
	multipath.sections[sCur].velocities.push_back(v);
      }
      multipath.sections[sCur+1].milestones.resize(0);
      multipath.sections[sCur+1].velocities.resize(0);
      multipath.sections[sCur+1].times.resize(0);
      multipath.sections[sCur+1].milestones.push_back(x);
      multipath.sections[sCur+1].velocities.push_back(v);
      multipath.sections[sCur+1].times.push_back(switchTime);
      sCur++;
    }
    if(t == multipath.sections[s].times.back()) continue;
    traj.Eval(t,x);
    traj.Deriv(t,v);
    multipath.sections[s].times.push_back(t);
    multipath.sections[s].milestones.push_back(x);
    multipath.sections[s].velocities.push_back(v);
  }

#if DO_TEST_TRIANGULAR
  timer.Reset();
  Real Ttrap = 0;
  printf("%d paths?\n",paths.size());
  for(size_t i=0;i<paths.size();i++)
    Ttrap += OptimalTriangularTimeScaling(paths[i],robot.velMin,robot.velMax,-1.0*robot.accMax,robot.accMax);
  printf("Optimal trapezoidal time scaling duration %g, calculated in %gs\n",Ttrap,timer.ElapsedTime());
#endif // DO_TEST_TRAPEZOIDAL


#if DO_CHECK_BOUNDS  
  CheckBounds(robot,traj,dt);
#endif // DO_CHECK_BOUNDS

#if DO_SAVE_LIMITS
  SaveLimits(robot,traj,dt,"opt_multipath_limits.csv");
#endif // DO_SAVE_LIMITS

  {
    stringstream ss;
    ss<<xtol;
    multipath.settings["resolution"] = ss.str();
    multipath.settings["program"] = "GenerateAndTimeOptimizeMultiPath";
  }
  return true;
}


void EvaluateMultiPath(Robot& robot,const MultiPath& path,Real t,Config& q,Real xtol,Real contactol,int numIKIters)
{
  RobotCSpace space(robot);;
  RobotGeodesicManifold manifold(robot);
  GeneralizedCubicBezierCurve curve(&space,&manifold);
  Real duration,param;
  int seg=path.Evaluate(t,curve,duration,param,MultiPath::InterpLinear);
  curve.Eval(param,q);

  //solve for constraints
  bool solveIK = false;
  if(path.settings.count("resolution")==0)
    solveIK = true;
  else {
    stringstream ss(path.settings.find("resolution")->second);
    Real res;
    ss>>res;
    if(res > xtol) solveIK=true;
  }
  if(solveIK) {
    vector<IKGoal> ik;
    path.GetIKProblem(ik,seg);
    if(!ik.empty()) {
      swap(q,robot.q);
      robot.UpdateFrames();

      int iters=numIKIters;
      bool res=SolveIK(robot,ik,contactol,iters,0);
      if(!res) printf("Warning, couldn't solve IK problem at sec %d, time %g\n",seg,t);
      swap(q,robot.q);
    }
  }
}
