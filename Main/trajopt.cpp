#include "Planning/ConstrainedInterpolator.h"
#include "Planning/DynamicConstrainedInterpolator.h"
#include <math3d/primitives.h>
#include <math/random.h>
#include <math/differentiation.h>
#include "Planning/TimeScaling.h"
#include "Planning/RobotConstrainedInterpolator.h"
#include "Planning/ContactTimeScaling.h"
#include "Contact/Utils.h"
#include "Modeling/MultiPath.h"
#include <Timer.h>
#include <fstream>
using namespace std;
using namespace Math3D;



void RunConstrainedInterpolateTest(const Config& a,const Config& b,CSpace* space,VectorFieldFunction* constraint)
{
  ConstrainedInterpolator interp(space,constraint);
  interp.ftol = 1e-8;
  interp.xtol = 1e-2;
  interp.maxNewtonIters = 100;
  SmoothConstrainedInterpolator interp2(space,constraint);
  interp2.ftol = interp.ftol;
  interp2.xtol = interp.xtol;
  interp2.maxNewtonIters = interp.maxNewtonIters;

  //run tests
  cout<<"Interpolating "<<a<<" -> "<<b<<endl;
  double xtols[10] = {1e-3,2.5e-3,5e-3,1e-2,2.5e-2,5e-2,1e-1,2.5e-1,5e-1,1};
  const static int N=10;

  /*
  cout<<"Descent interpolation"<<endl;
  cout<<"xtol,success,time,edges,vertex error,edge error,smoothed error"<<endl;
  for(int iter = 0; iter < N; iter++) {
    interp.xtol = xtols[iter];

    cout<<xtols[iter]<<",";
    int numTrials = 10;
    bool res;
    vector<Config> path;
    Timer timer;
    for(int j=0;j<numTrials;j++) {
      path.resize(0);
      res=true;
      path.push_back(a);
      Vector temp,dir;
      while(true) {
	dir=b-path.back();
	if(dir.norm() < interp.xtol) {
	  path.push_back(b);
	  break;
	}
	interp2.ProjectVelocity(path.back(),dir);
	Real n = dir.norm();
	if(n < 1e-7) {
	  res = false;
	  break;
	}
	dir *= interp.xtol / n;
	temp = path.back() + dir;
	interp2.Project(temp);
	path.push_back(temp);
      }
      if(!res) { numTrials = j+1; break; }
    }
    if(res) {
      cout<<"1,"<<timer.ElapsedTime()/numTrials<<","<<path.size()<<",";
      Real maxerr = 0.0;
      Real maxerrmid = 0.0;
      Real maxerrsmooth = 0.0;
      Vector val;
      Vector x;
      for(size_t i=0;i<path.size();i++) {
	(*interp.constraint)(path[i],val);
	maxerr = Max(maxerr,val.norm());
	if(i+1 < path.size()) {
	  int numdivs = Max(10000/(int)path.size(),2);
	  for(int j=1;j<numdivs;j++) {
	    space->Interpolate(path[i],path[i+1],Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrmid = Max(maxerrmid,val.norm());
	  }

	  //do smoothed
	  GeneralizedCubicBezierCurve curve(space);
	  curve.x0 = path[i];
	  curve.x3 = path[i+1];
	  if(i == 0) curve.x1 = curve.x0 + (path[i+1]-path[i])/3.0;
	  else curve.x1 = curve.x0 + 0.5*(path[i+1]-path[i-1])/3.0;
	  if(i+2 >= path.size()) curve.x2 = curve.x3 - (path[i+1]-path[i])/3.0;
	  else curve.x2 = curve.x3 - 0.5*(path[i+2]-path[i])/3.0;
	  for(int j=1;j<numdivs;j++) {
	    curve.Eval(Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrsmooth = Max(maxerrsmooth,val.norm());
	  }
	}
      }
      cout<<maxerr<<","<<maxerrmid<<","<<maxerrsmooth<<endl;
    }
    else {
      cout<<"0,"<<timer.ElapsedTime()<<endl;
    }
  }
  

  cout<<"Linear interpolation"<<endl;
  cout<<"xtol,success,time,edges,vertex error,edge error,smoothed error"<<endl;
  for(int iter = 0; iter < N; iter++) {
    interp.xtol = xtols[iter];

    cout<<xtols[iter]<<",";
    int numTrials = 10;
    bool res;
    vector<Config> path;
    Timer timer;
    for(int j=0;j<numTrials;j++) {
      path.resize(0);
      res=interp.Make(a,b,path);
      if(!res) { numTrials = j+1; break; }
    }
    if(res) {
      cout<<"1,"<<timer.ElapsedTime()/numTrials<<","<<path.size()<<",";
      Real maxerr = 0.0;
      Real maxerrmid = 0.0;
      Real maxerrsmooth = 0.0;
      Vector val;
      Vector x;
      for(size_t i=0;i<path.size();i++) {
	(*interp.constraint)(path[i],val);
	maxerr = Max(maxerr,val.norm());
	if(i+1 < path.size()) {
	  int numdivs = Max(10000/(int)path.size(),2);
	  for(int j=1;j<numdivs;j++) {
	    space->Interpolate(path[i],path[i+1],Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrmid = Max(maxerrmid,val.norm());
	  }

	  //do smoothed
	  GeneralizedCubicBezierCurve curve(space);
	  curve.x0 = path[i];
	  curve.x3 = path[i+1];
	  if(i == 0) curve.x1 = curve.x0 + (path[i+1]-path[i])/3.0;
	  else curve.x1 = curve.x0 + 0.5*(path[i+1]-path[i-1])/3.0;
	  if(i+2 >= path.size()) curve.x2 = curve.x3 - (path[i+1]-path[i])/3.0;
	  else curve.x2 = curve.x3 - 0.5*(path[i+2]-path[i])/3.0;
	  for(int j=1;j<numdivs;j++) {
	    curve.Eval(Real(j)/Real(numdivs),x);
	    (*interp.constraint)(x,val);
	    maxerrsmooth = Max(maxerrsmooth,val.norm());
	  }
	}
      }
      cout<<maxerr<<","<<maxerrmid<<","<<maxerrsmooth<<endl;
    }
    else {
      cout<<"0,"<<timer.ElapsedTime()<<endl;
    }
  }
  */

  cout<<"Smooth interpolation"<<endl;
  cout<<"xtol,success,time,edges,vertex error,edge error"<<endl;
  for(int iter = 0; iter < N; iter++) {
    interp2.xtol = xtols[iter];
    cout<<xtols[iter]<<",";

    GeneralizedCubicBezierSpline cpath;
    int numTrials = 10;
    bool res;
    Timer timer;
    for(int j=0;j<numTrials;j++) {
      cpath.segments.clear();
      cpath.durations.clear();
      res=interp2.Make(a,b,cpath);
      if(!res) { numTrials = j+1; break; }
    }
    if(res) {
      cout<<"1,"<<timer.ElapsedTime()/numTrials<<","<<cpath.segments.size()<<",";
      Real maxerr = 0.0;
      Real maxerrmid = 0.0;
      for(size_t i=0;i<cpath.segments.size();i++) {
	if(i > 0) Assert(cpath.segments[i].x0 == cpath.segments[i-1].x3);
	
	Vector val;
	(*interp.constraint)(cpath.segments[i].x0,val);
	maxerr = Max(maxerr,val.norm());
	(*interp.constraint)(cpath.segments[i].x3,val);
	maxerr = Max(maxerr,val.norm());
	
	Vector x;
	int numdivs = Max(10000/(int)cpath.segments.size(),2);
	for(int j=1;j<numdivs;j++) {
	  cpath.segments[i].Eval(Real(j)/Real(numdivs),x);
	  (*interp.constraint)(x,val);
	  maxerrmid = Max(maxerrmid,val.norm());
	}
      }
      cout<<maxerr<<","<<maxerrmid<<endl;
    }
    else {
      cout<<"0,"<<timer.ElapsedTime()<<endl;
    }
  }
}


class CartesianCSpace : public CSpace
{
public:
  CartesianCSpace(int _n) : n(_n) {}
  virtual void Sample(Config& q) { q.resize(n,0.0); }
  virtual bool IsFeasible(const Config& q) { return true; }
  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return new TrueEdgePlanner(this,a,b); }
  int n;
};

class SphereConstraint : public VectorFieldFunction
{
public:
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    fx.resize(1);
    fx[0] = x.normSquared()-1;
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    Vector Ji = x;
    Ji *= 2;
    J.copyRow(0,Ji);
  }
};

class MultiSphereConstraint : public VectorFieldFunction
{
public:
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    fx.resize(1);
    Real xmod = Mod(x[0]+1.5,3.0)-1.5;
    fx[0] = x.normSquared()-Sqr(x[0])+Sqr(xmod)-1;
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    Vector Ji = x;
    Ji(0) = Mod(x[0]+1.5,3.0)-1.5;
    Ji *= 2;
    J.copyRow(0,Ji);
  }
};

class SineWaveConstraint : public VectorFieldFunction
{
public:
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    fx.resize(1);
    fx[0] = x[1] - Sin(x[0]);
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    J(0,0) = -Cos(x[0]);
    J(0,1) = 1.0;
  }
};


class TorusConstraint : public VectorFieldFunction
{
public:
  Real primaryRadius,secondaryRadius;
  TorusConstraint(Real _primaryRadius=1.0,Real _secondaryRadius=0.25)
    :primaryRadius(_primaryRadius),secondaryRadius(_secondaryRadius)
  {}
  virtual int NumDimensions() const { return 1; }
  virtual void Eval(const Vector& x,Vector& fx) {
    Assert(x.n >= 3);
    Real len = Sqrt(Sqr(x[1])+Sqr(x[0]));
    Real xc = x[0]*primaryRadius/len;
    Real yc = x[1]*primaryRadius/len;
    if(len == 0) { xc = primaryRadius; yc=0; }
    Real d2 = Sqr(x[0]-xc) + Sqr(x[1]-yc) + Sqr(x[2]);
    fx.resize(1);
    fx[0] = d2 - Sqr(secondaryRadius);
  }
  virtual void Jacobian(const Vector& x,Matrix& J) {
    J.resize(1,x.n);
    Vector Ji(3);
    Real len = Sqrt(Sqr(x[1])+Sqr(x[0]));
    if(FuzzyZero(len)) Ji.setZero();
    else {
      Real xc = x[0]*primaryRadius/len;
      Real yc = x[1]*primaryRadius/len;
      Real dlend0 = x[0]/len;
      Real dlend1 = x[1]/len;
      //derivative of xc w.r.t. x[0], x[1]
      Real dxcd0 = (len*primaryRadius - x[0]*primaryRadius*dlend0)/Sqr(len);
      Real dxcd1 = -x[0]*primaryRadius*dlend1/Sqr(len);
      //derivative of yc w.r.t. x[0], x[1]
      Real dycd0 = -x[1]*primaryRadius*dlend0/Sqr(len);
      Real dycd1 = (len*primaryRadius - x[1]*primaryRadius*dlend1)/Sqr(len);
      Ji[0] = 2.0*(x[0]-xc)*(1-dxcd0) - 2.0*(x[1]-yc)*dycd0;
      Ji[1] = -2.0*(x[0]-xc)*dxcd1 + 2.0*(x[1]-yc)*(1-dycd1);
      Ji[2] = 2*x[2];
    }
    J.copyRow(0,Ji);

    /*
    //TEMP: debugging
    Matrix Jdiff(1,3);
    Vector temp=x;
    JacobianCenteredDifference(*this,temp,0.001,Jdiff);
    if(!Jdiff.isEqual(J,1e-2)) {
      cout<<"Jacobian error at "<<x<<endl;
      cout<<Ji<<endl;
      cout<<Jdiff<<endl;
    }
    Assert(Jdiff.isEqual(J,1e-2));
    */
  }

  void Enumerate(Real resolution,vector<Vector>& grid)
  {
    int udivs = (int)Ceil(secondaryRadius*TwoPi/resolution);
    int thetadivs = (int)Ceil((primaryRadius+secondaryRadius)*TwoPi/resolution);
    Vector res(3);
    for(int i=0;i<thetadivs;i++) {
      Real theta=Real(i)/thetadivs*TwoPi;
      Matrix2 R; R.setRotate(theta);
      for(int j=0;j<udivs;j++) {
	Real u=Real(j)/udivs*TwoPi;
	Real sx=Cos(u)*secondaryRadius+primaryRadius,z=Sin(u)*secondaryRadius;
	res[0] = R(0,0)*sx;
	res[1] = R(1,0)*sx;
	res[2] = z;

	//TEMP: test
	Vector v;
	Eval(res,v);
	Assert(FuzzyZero(v[0]));
	grid.push_back(res);
      }
    }
  }

  void Sample(Vector& res)
  {
    res.resize(3);
    Real u=Rand()*TwoPi;
    Real sx=Cos(u)*secondaryRadius+primaryRadius,z=Sin(u)*secondaryRadius;
    Real theta=Rand()*TwoPi;
    Matrix2 R; R.setRotate(theta);
    res[0] = R(0,0)*sx;
    res[1] = R(1,0)*sx;
    res[2] = z;

    //TEMP: test
    Vector v;
    Eval(res,v);
    Assert(FuzzyZero(v[0]));
  }
};

void ConstrainedInterpolateTest()
{
  {
    /*
    CartesianCSpace space2(2);
    SineWaveConstraint sineWaveConstraint;
    Vector a(2),b(2);
    a(0) = 0; a(1) = 0; 
    b(0) = TwoPi*2; b(1) = 0;

    cout<<endl;
    cout<<"***Sine wave test***"<<endl;
    RunConstrainedInterpolateTest(a,b,&space2,&sineWaveConstraint);
    */

    /*
    ConstrainedInterpolator interp(&space2,&sineWaveConstraint);
    interp.ftol = 1e-8;
    interp.xtol = 4;
    interp.maxNewtonIters = 100;

    SmoothConstrainedInterpolator interp2(&space2,&sineWaveConstraint);
    interp2.ftol = interp.ftol;
    interp2.xtol = interp.xtol;
    interp2.maxNewtonIters = interp.maxNewtonIters;

    vector<Config> path;
    bool res=interp.Make(a,b,path);
    cout<<path.size()<<" milestones"<<endl;
    cout<<"x,y"<<endl;
    for(size_t i=0;i<=path.edges.size();i++)
      cout<<path.GetMilestone(i)[0]<<","<<path.GetMilestone(i)[1]<<endl;
    cout<<endl;
    GeneralizedCubicBezierSpline cpath;
    res=interp2.Make(a,b,cpath);
    cout<<cpath.size()<<" edges"<<endl;
    cout<<"x,y"<<endl;
    double dt=1e-3;
    Vector temp;
    for(size_t i=0;i<cpath.segments.size();i++) {
      double t=0;
      while(t < cpath.durations[i]) {
	cpath.segments[i].Eval(t/cpath.durations[i],temp);
	cout<<temp[0]<<","<<temp[1]<<endl;
	t += dt;
      }
      cpath[i].Eval(1,temp);
      cout<<temp[0]<<","<<temp[1]<<endl;
    }
    cout<<endl;
    return;
    */
  }

  CartesianCSpace space(3);
  SphereConstraint sphereConstraint;
  TorusConstraint torusConstraint;
  Vector a(3),b(3);
  /*
  //interpolate on sphere
  cout<<endl;
  cout<<"***Sphere test***"<<endl;
  a(0) = 1; a(1) = 0; a(2) = 0;
  b(0) = 0; b(1) = 1; b(2) = 0;
  //b(0) = Cos(Pi*9.0/10.0); b(1) = Sin(Pi*9.0/10.0); b(2) = 0;
  RunConstrainedInterpolateTest(a,b,&space,&sphereConstraint);

  //interpolate on torus
  cout<<endl;
  cout<<"***Torus test***"<<endl;
  a(0) = 1.25; a(1) = 0; a(2) = 0;
  b(0) = 0; b(1) = 1; b(2) = 0.25;
  RunConstrainedInterpolateTest(a,b,&space,&torusConstraint);


  //this should fail
  MultiSphereConstraint multiSphereConstraint;
  a(0) = 1; a(1) = 0; a(2) = 0;
  b(0) = 3; b(1) = 1; b(2) = 0;
  //printf("Running fail constrained interpolate test...\n");
  //RunConstrainedInterpolateTest(a,b,&space,&multiSphereConstraint);
  */

  cout<<endl;
  cout<<"***All-torus test***"<<endl;
  //test all points on torus
  vector<Vector> pts;
  //torusConstraint.Enumerate(0.05,pts);
  torusConstraint.Enumerate(0.01,pts);
  //a(0) = 1.25; a(1) = 0; a(2) = 0;
  a(0) = 1.0; a(1) = 0.0; a(2) = 0.25;
  cout<<"Source: "<<a(0)<<","<<a(1)<<","<<a(2)<<endl;
  SmoothConstrainedInterpolator interp(&space,&torusConstraint);
  interp.ftol = 1e-8;
  interp.xtol = 1e-2;
  interp.maxNewtonIters = 100;
  for(size_t i=0;i<pts.size();i++) {
    GeneralizedCubicBezierSpline cpath;
    bool res=interp.Make(a,pts[i],cpath);
    cout<<pts[i][0]<<","<<pts[i][1]<<","<<pts[i][2]<<","<<res<<endl;
  }
}


void TimeOptimizeTest1DLine()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from 0 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    vector<Vector> vs(numSegments+1);
    for(int j=0;j<=numSegments;j++)
      divs[j] = Real(j)/Real(numSegments);

    //straight line path from 0 to 3
    for(int j=0;j<numSegments;j++) {
      vs[j] = Vector(1,3.0);
      vmins[j] = Vector(1,3.0);
      vmaxs[j] = Vector(1,3.0);
      amins[j] = Vector(1,0.0);
      amaxs[j] = Vector(1,0.0);
    }
    vs.back() = Vector(1,3.0);
    Timer timer;
    TimeScaling timeScaling;
    timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    if(res) {
      printf("End time: %g\n",timeScaling.times.back());
      printf("Solution time: %g s\n",time);
      for(size_t j=0;j<timeScaling.ds.size();j++) 
	Assert(vmin[0] <= timeScaling.ds[j] && timeScaling.ds[j] <= vmax[0]);
    }
  }
}

void TimeOptimizeTest1DSine()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from 0 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024,2028};
  for(int i=0;i<11;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    for(int j=0;j<=numSegments;j++)
      divs[j] = Real(j)/Real(numSegments);

    //sine wave
    //p(s) = sin(s*2pi/period)
    //p'(s) = 2pi/period*cos(s*2pi/period)
    //p''(s) = -(2pi/period)^2*sin(s*2pi/period)
    Real period = 1.0;
    for(int j=0;j<numSegments;j++) {
      Real umin = Real(j)/numSegments;
      Real umax = Real(j+1)/numSegments;
      Real xmin = umin*TwoPi/period;
      Real xmax = umax*TwoPi/period;
      Real vmin=Cos(xmin),vmax=Cos(xmax);
      Real amin=-Sin(xmin),amax=-Sin(xmax);
      if(vmin > vmax) Swap(vmin,vmax);
      if(amin > amax) Swap(amin,amax);
      if(xmax > TwoPi && xmin < TwoPi)  vmax=1;
      if(xmax > 2*TwoPi && xmin < 2*TwoPi)  vmax=1;
      if(xmax > Pi && xmin < Pi)  vmin=-1;
      if(xmax > 3*Pi && xmin < 3*Pi)  vmin=-1;
      if(xmax > Pi/2 && xmin < Pi/2)  amin=-1;
      if(xmax > Pi/2+TwoPi && xmin < Pi/2+TwoPi)  amin=-1;
      if(xmax > 3*Pi/2 && xmin < 3*Pi/2)  amax=1;
      if(xmax > 3*Pi/2+TwoPi && xmin < 3*Pi/2+TwoPi)  amax=1;
      Assert(vmax - vmin <= 1.0/numSegments*TwoPi);
      Assert(amax - amin <= 1.0/numSegments*TwoPi/period);
      Assert(vmin >= -1.0 && vmax <= 1.0);
      Assert(amin >= -1.0 && amax <= 1.0);
      Assert(vmin <= vmax);
      Assert(amin <= amax);
      vmax *= TwoPi/period;
      vmin *= TwoPi/period;
      amax *= Sqr(TwoPi/period);
      amin *= Sqr(TwoPi/period);
      vs[j] = Vector(1,TwoPi/period*Cos(xmin));
      if(j+1==numSegments)
	vs[j+1] = Vector(1,TwoPi/period*Cos(xmax));
      vmins[j] = Vector(1,vmin);
      vmaxs[j] = Vector(1,vmax);
      amins[j] = Vector(1,amin);
      amaxs[j] = Vector(1,amax);
    }
    Timer timer;
    TimeScaling timeScaling;
    timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    //bool res=timeScaling.SolveMinTimeArcLength(vmin,vmax,amin,amax,divs,vs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    if(res) {
      printf("End time: %g\n",timeScaling.times.back());
      printf("Solution time: %g s\n",time);
    }

    if(i+1==11) {
      cout<<"time,s,ds,val,vmin,vmax,amin,amax,dy,ddy"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++) {
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<Sin(timeScaling.params[i]*TwoPi/period);
	if(i<vmins.size()) {
	  Real dp=Cos(timeScaling.params[i]*TwoPi/period)*TwoPi/period;
	  Real ddp=-Sin(timeScaling.params[i]*TwoPi/period)*Sqr(TwoPi/period);
	  cout<<","<<vmins[i][0]<<","<<vmaxs[i][0]<<","<<amins[i][0]<<","<<amaxs[i][0];
	  cout<<","<<timeScaling.ds[i]*dp<<","<<timeScaling.TimeToParamAccel(i,timeScaling.times[i])*dp+Sqr(timeScaling.ds[i])*ddp<<endl;
	}
	else cout<<endl;
      }
    }
  }
}


void TimeOptimizeTest1DParabolic()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from -1 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    vector<Vector> vs(numSegments+1);
    for(int j=0;j<=numSegments;j++) {
      divs[j] = Real(j)/Real(numSegments);
      vs[j] = Vector(1,8.0*Abs(divs[j]-0.5));
      if(j > 0) {
	vmins[j-1] = vs[j-1];
	vmaxs[j-1] = vs[j];
	if(divs[j] < 0.5)
	  Swap(vmins[j-1],vmaxs[j-1]);
	amins[j-1] = Vector(1,8.0*Sign(divs[j-1]-0.5));
	amaxs[j-1] = Vector(1,8.0*Sign(divs[j-1]-0.5));
      }
    }
    Timer timer;
    TimeScaling timeScaling;
    //timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTimeArcLength(vmin,vmax,amin,amax,divs,vs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    if(res) {
      printf("End time: %g\n",timeScaling.times.back());
      printf("Solution time: %g s\n",time);
      /*
      for(size_t j=0;j<timeScaling.ds.size();j++) 
	Assert(vmin[0] <= timeScaling.ds[j]*vs[j][0] && timeScaling.ds[j]*vs[j][0] <= vmax[0]);
      */
    }
    if(i+1==10) {
      cout<<"time,s,ds,val,vmin,vmax,amin,amax,dy,ddy"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++) {
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<4.0*Pow(timeScaling.params[i]-0.5,2.0)*Sign(timeScaling.params[i]-0.5);
	if(i<vmins.size()) {
	  Real dp=8.0*Abs(timeScaling.params[i]-0.5);
	  Real ddp=8.0*Sign(timeScaling.params[i]-0.5);
	  cout<<","<<vmins[i][0]<<","<<vmaxs[i][0]<<","<<amins[i][0]<<","<<amaxs[i][0];
	  cout<<","<<timeScaling.ds[i]*dp<<","<<timeScaling.TimeToParamAccel(i,timeScaling.times[i])*dp+Sqr(timeScaling.ds[i])*ddp<<endl;
	}
	else cout<<endl;
      }
    }
  }
}

void TimeOptimizeTest1DCubic()
{
  Vector vmin(1,-1.0),vmax(1,1.0);
  Vector amin(1,-1.0),amax(1,1.0);
  //try the path from -1 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    vector<Vector> vs(numSegments+1);
    for(int j=0;j<=numSegments;j++) {
      divs[j] = Real(j)/Real(numSegments);
      vs[j] = Vector(1,3.0*8.0*(divs[j]-0.5)*(divs[j]-0.5));
      if(j > 0) {
	vmins[j-1] = vs[j-1];
	vmaxs[j-1] = vs[j];
	amins[j-1] = Vector(1,6.0*8.0*(divs[j-1]-0.5));
	amaxs[j-1] = Vector(1,6.0*8.0*(divs[j]-0.5));
      }
    }
    Timer timer;
    TimeScaling timeScaling;
    timeScaling.ConditionMinTime(divs,vs,vmins,vmaxs,amins,amaxs);
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    printf("End time: %g\n",timeScaling.times.back());
    printf("Solution time: %g s\n",time);
    for(size_t j=0;j<timeScaling.ds.size();j++) 
      Assert(vmin[0] <= timeScaling.ds[j]*vs[j][0] && timeScaling.ds[j]*vs[j][0] <= vmax[0]);
    if(i+1==10) {
      cout<<"time,s,ds,val,vmin,vmax,amin,amax,dy,ddy"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++) {
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<8.0*Pow(timeScaling.params[i]-0.5,3.0);
	if(i<vmins.size()) {
	  Real dp=24.0*Sqr(timeScaling.params[i]-0.5);
	  Real ddp=48.0*(timeScaling.params[i]-0.5);
	  cout<<","<<vmins[i][0]<<","<<vmaxs[i][0]<<","<<amins[i][0]<<","<<amaxs[i][0];
	  cout<<","<<timeScaling.ds[i]*dp<<","<<timeScaling.TimeToParamAccel(i,timeScaling.times[i])*dp+Sqr(timeScaling.ds[i])*ddp<<endl;
	}
	else cout<<endl;
      }
    }
  }
}


void TimeOptimizeTest2DCircle()
{
  Vector vmin(2,-1.0),vmax(2,1.0);
  Vector amin(2,-1.0),amax(2,1.0);
  //path param goes from 0 to 1
  int ns [] = {2,4,8,16,32,64,128,256,512,1024};
  for(int i=0;i<10;i++) {
    int numSegments = ns[i];
    vector<Real> divs(numSegments+1);
    vector<Vector> vs(numSegments+1);
    vector<Vector> vmins(numSegments),vmaxs(numSegments);
    vector<Vector> amins(numSegments),amaxs(numSegments);
    for(int j=0;j<=numSegments;j++)
      divs[j] = Real(j)/Real(numSegments);

    //circles
    //p(s) = (sin(s*2pi/period),cos(s*2pi/period)
    //p'(s) = 2pi/period*(cos(s*2pi/period),-sin(s*2pi/period))
    //p''(s) = (2pi/period)^2*(-sin(s*2pi/period),-cos(s*2pi/period))
    Real period = 1.0;
    for(int j=0;j<numSegments;j++) {
      Real umin = Real(j)/numSegments;
      Real umax = Real(j+1)/numSegments;
      Real xmin = umin*TwoPi/period;
      Real xmax = umax*TwoPi/period;
      Vector2 vmin(Cos(xmin),-Sin(xmin)),vmax(Cos(xmax),-Sin(xmax));
      Vector2 amin(-Sin(xmin),-Cos(xmin)),amax(-Sin(xmax),-Cos(xmax));
      if(vmin.x > vmax.x) Swap(vmin.x,vmax.x);
      if(vmin.y > vmax.y) Swap(vmin.y,vmax.y);
      if(amin.x > amax.x) Swap(amin.x,amax.x);
      if(amin.y > amax.y) Swap(amin.y,amax.y);
      if(xmax > TwoPi && xmin < TwoPi) { vmax.x=1; amin.y=-1; }
      if(xmax > 2*TwoPi && xmin < 2*TwoPi) { vmax.x=1; amin.y=-1; }
      if(xmax > Pi && xmin < Pi) { vmin.x=-1; amax.y=1; }
      if(xmax > 3*Pi && xmin < 3*Pi) { vmin.x=-1; amax.y=1; }
      if(xmax > Pi/2 && xmin < Pi/2)  { vmin.y=-1; amin.x=-1; }
      if(xmax > Pi/2+TwoPi && xmin < Pi/2+TwoPi) { vmin.y=-1; amin.x=-1; }
      if(xmax > 3*Pi/2 && xmin < 3*Pi/2) { vmax.y=1; amax.x=1; }
      if(xmax > 3*Pi/2+TwoPi && xmin < 3*Pi/2+TwoPi) { vmax.y=1; amax.x=1; }
      Assert(vmin.x >= -1.0 && vmax.x <= 1.0);
      Assert(vmin.y >= -1.0 && vmax.y <= 1.0);
      Assert(amin.x >= -1.0 && amax.x <= 1.0);
      Assert(amin.y >= -1.0 && amax.y <= 1.0);
      Assert(vmin.x <= vmax.x);
      Assert(vmin.y <= vmax.y);
      Assert(amin.x <= amax.x);
      Assert(amin.y <= amax.y);
      vmax *= TwoPi/period;
      vmin *= TwoPi/period;
      amax *= Sqr(TwoPi/period);
      amin *= Sqr(TwoPi/period);
      vmins[j] = Vector(2,vmin);
      vmaxs[j] = Vector(2,vmax);
      amins[j] = Vector(2,amin);
      amaxs[j] = Vector(2,amax);
      vs[j] = Vector(2,Vector2(TwoPi/period*Cos(xmin),-TwoPi/period*Sin(xmin)));
      if(j+1==numSegments)
	vs[j+1] = Vector(2,Vector2(TwoPi/period*Cos(xmax),-TwoPi/period*Sin(xmax)));
    }
    Timer timer;
    TimeScaling timeScaling;
    bool res=timeScaling.SolveMinTime(vmin,vmax,amin,amax,divs,vs,vmins,vmaxs,amins,amaxs,0.0,0.0);
    double time=timer.ElapsedTime();
    printf("Num segments: %d\n",numSegments);
    printf("Result: %d\n",res);
    printf("End time: %g\n",timeScaling.times.back());
    printf("Solution time: %g s\n",time);

    if(i+1==10) {
      cout<<"t,s,ds,x,y:"<<endl;
      for(size_t i=0;i<timeScaling.times.size();i++)
	cout<<timeScaling.times[i]<<","<<timeScaling.params[i]<<","<<timeScaling.ds[i]<<","<<Sin(timeScaling.params[i]*TwoPi/period)<<","<<Cos(timeScaling.params[i]*TwoPi/period)<<endl;
    }
  }
}

void TimeOptimizeTest2DCircleBezier()
{
  CartesianCSpace space(2);
  SphereConstraint sphereConstraint;
  SmoothConstrainedInterpolator interp(&space,&sphereConstraint);
  interp.ftol = 1e-8;
  interp.xtol = 1e-2;
  interp.maxNewtonIters = 100;
  vector<Vector> pts;
  pts.resize(5);
  pts[0].resize(2);
  pts[1].resize(2);
  pts[2].resize(2);
  pts[3].resize(2);
  pts[4].resize(2);
  pts[0](0)=1;    pts[0](1)=0;
  pts[1](0)=0;    pts[1](1)=1;
  pts[2](0)=-1;    pts[2](1)=0;
  pts[3](0)=0;    pts[3](1)=-1;
  pts[4](0)=1;    pts[4](1)=0;
  Timer timer;
  TimeScaledBezierCurve curve;
  MultiSmoothInterpolate(interp,pts,curve.path);
  for(size_t i=0;i<curve.path.segments.size();i++)
    Assert(curve.path.segments[i].space == &space);

  printf("Num segments: %d\n",curve.path.segments.size());
  Vector vmin(2,-1.0),vmax(2,1.0);
  Vector amin(2,-1.0),amax(2,1.0);
  bool res=curve.OptimizeTimeScaling(vmin,vmax,amin,amax);
  if(!res) {
    printf("Error optimizing path\n");
    return;
  }
  printf("End time: %g\n",curve.EndTime());
  printf("Solution time: %g s\n",timer.ElapsedTime());
  Assert(curve.timeScaling.times.size()==curve.timeScaling.params.size());
  
  cout<<"t,s,ds,x,y,dx,dy:"<<endl;
  for(size_t i=0;i<curve.timeScaling.times.size();i++) {
    Vector x,v;
    curve.Eval(curve.timeScaling.times[i],x);
    curve.Deriv(curve.timeScaling.times[i],v);
    cout<<curve.timeScaling.times[i]<<","<<curve.timeScaling.params[i]<<","<<curve.timeScaling.ds[i]<<","<<x[0]<<","<<x[1]<<","<<v[0]<<","<<v[1]<<endl;
  }
}

void TimeOptimizeTestNLinkPlanar()
{
  const char* fn = "data/planar%d.rob";
  int ns [] = {5,10,15,20,25,30,40,50,60,70,80,90,100};
  Real tolscales [] = {3.95,3.55,2.73,2,2,2,1.8,1.6,1.42,1,1,1,1};
  int num = 13;
  char buf[256];
  for(int i=0;i<num;i++) {
    int n=ns[i];
    printf("Testing optimize %d\n",n);
    sprintf(buf,fn,n);
    Robot robot;
    if(!robot.Load(buf)) {
      fprintf(stderr,"Unable to load robot %s\n",buf);
      return;
    }
    int ee = robot.links.size()-1;
    Vector3 localpt(1,0,0);
    Real len = robot.links.size();
    //make a half circle
    Config a(robot.q.n,Pi/robot.links.size()),b;
    a[0] *= 0.5;
    robot.UpdateConfig(a);
    IKGoal goal,goaltemp;
    goal.link = ee;
    goal.localPosition = localpt;
    goal.SetFixedPosition(robot.links[ee].T_World*localpt);
    //cout<<"Goal position "<<goal.endPosition<<endl;
    goaltemp = goal;
    goaltemp.link = ee/2;
    goaltemp.SetFixedPosition(goal.endPosition*0.5);
    //cout<<"Middle goal position "<<goaltemp.endPosition<<endl;
    vector<IKGoal> onegoal(1),bothgoals(2);
    onegoal[0] = goal;
    bothgoals[0] = goal;
    bothgoals[1] = goaltemp;

    int iters=100;
    bool res=SolveIK(robot,bothgoals,1e-3,iters);
    if(!res) {
      fprintf(stderr,"Couldn't solve for target robot config\n");
      return;
    }
    b = robot.q;    
   
    Timer timer;
    GeneralizedCubicBezierSpline path;
    RobotSmoothConstrainedInterpolator interp(robot,onegoal);
    interp.xtol = 1e-3*tolscales[i];
    if(!interp.Make(a,b,path)) {
      fprintf(stderr,"Couldn't interpolate for target robot config\n");
      return;
    }
    printf("Solved for path with tol %g in time %g\n",interp.xtol,timer.ElapsedTime());

    {
      sprintf(buf,"trajopt_a_%d.config",n);
      ofstream out(buf);
      out<<a<<endl;
    }
    {
      sprintf(buf,"trajopt_b_%d.config",n);
      ofstream out(buf);
      out<<b<<endl;
    }
    {
      sprintf(buf,"trajopt_interp_%d.xml",n);
      vector<Real> times;
      vector<Config> configs;
      path.GetPiecewiseLinear(times,configs);
      MultiPath mpath;
      mpath.SetTimedMilestones(times,configs);
      mpath.SetIKProblem(onegoal);
      mpath.Save(buf);
    }
    {
      //unroll joints
      for(size_t i=0;i<path.segments.size();i++) {
	for(int j=0;j<path.segments[i].x0.n;j++) {
	  if(path.segments[i].x0[j] > Pi)
	    path.segments[i].x0[j] -= TwoPi;
	  if(path.segments[i].x1[j] > Pi)
	    path.segments[i].x1[j] -= TwoPi;
	  if(path.segments[i].x2[j] > Pi)
	    path.segments[i].x2[j] -= TwoPi;
	  if(path.segments[i].x3[j] > Pi)
	    path.segments[i].x3[j] -= TwoPi;
	}
      }
      sprintf(buf,"trajopt_interp_%d.spline",n);
      ofstream out(buf,ios::out);
      out.precision(10);
      path.Save(out);
    }

    TimeScaledBezierCurve curve;
    //curve.path = path;
    {
      sprintf(buf,"trajopt_interp_%d.spline",n);
      ifstream in(buf,ios::in);
      curve.path.Load(in);
      Assert(curve.path.segments.size() == path.durations.size());
      for(size_t i=0;i<curve.path.durations.size();i++) {
	Assert(FuzzyEquals(curve.path.durations[i],path.durations[i]));
	if(!curve.path.segments[i].x0.isEqual(path.segments[i].x0,Epsilon)) {
	  printf("Error on segment %d\n",i);
	  cout<<path.segments[i].x0<<endl;
	  cout<<curve.path.segments[i].x0<<endl;
	  cout<<"Distance: "<<path.segments[i].x0.distance(curve.path.segments[i].x0)<<endl;
	}
	Assert(curve.path.segments[i].x0.isEqual(path.segments[i].x0,Epsilon));
	Assert(curve.path.segments[i].x1.isEqual(path.segments[i].x1,Epsilon));
	Assert(curve.path.segments[i].x2.isEqual(path.segments[i].x2,Epsilon));
	Assert(curve.path.segments[i].x3.isEqual(path.segments[i].x3,Epsilon));
      }
    }
    Vector vmin(robot.q.n,-Pi),vmax(robot.q.n,Pi);
    Vector amin(robot.q.n,-Pi),amax(robot.q.n,Pi);
    timer.Reset();
    if(!curve.OptimizeTimeScaling(vmin,vmax,amin,amax)) {
      fprintf(stderr,"Optimize failed in time %g\n",timer.ElapsedTime());
      return;
    }
    printf("Solved time optimization with %d segments in time %g\n",curve.path.segments.size(),timer.ElapsedTime());
    
    //output optimized path
    {
      Real dt = 0.5;
      vector<Real> times;
      vector<Config> configs;
      /*curve.GetDiscretizedPath(dt,configs);
      times.resize(configs.size());
      for(size_t j=0;j<times.size();j++)
	times[j] = j*dt;
      */
      curve.GetPiecewiseLinear(times,configs);
      MultiPath mpath;
      mpath.SetIKProblem(onegoal);
      mpath.SetTimedMilestones(times,configs);
      sprintf(buf,"trajopt_opt_%d.xml",n);
      mpath.Save(buf);
    }
  }
}

void TimeOptimizeTest()
{
  //TimeOptimizeTest1DLine();
  //TimeOptimizeTest1DSine();
  //TimeOptimizeTest2DCircle();
  //TimeOptimizeTest1DCubic();
  //TimeOptimizeTest1DParabolic();
  //TimeOptimizeTest2DCircleBezier();
  TimeOptimizeTestNLinkPlanar();
}

class BallisticFunction : public ParameterizedVectorFieldFunction
{
public:
  Vector3 gravity;
  BallisticFunction() { gravity.set(0,0,-9.8); }
  virtual int NumParameters() const { return 3; }
  virtual void SetParameters(const Vector& x) {}
  virtual int NumDimensions() const { return 3; }
  virtual void Eval(const Vector& dxddx,Vector& res) {
    Assert(dxddx.n == 6);
    Vector v,a;
    CVSpace::GetState(dxddx,v,a);
    res.resize(3);
    res(0) = a(0)-gravity.x;
    res(1) = a(1)-gravity.y;
    res(2) = a(2)-gravity.z;
  }
  virtual void Jacobian(const Vector& dxddx,Matrix& J) {
    J.resize(3,6);
    J.setZero();
    for(int i=0;i<3;i++)
      J(i,i+3) = 1.0;
  }
};


void DynamicOptimizeTest()
{
  CartesianCSpace space(3);
  BallisticFunction* dynConstraint = new BallisticFunction;
  dynConstraint->gravity.z = -1.0;
  //note: this takes ownership of dynConstraint
  Dynamic2ConstrainedInterpolator interp(&space,NULL,NULL,dynConstraint);
  Vector zero(3,0.0),vstart(3,0.0);
  Vector one(3,0.0),vend(3,0.0);
  one(0) = 1.0;
  /*
  //the following make it a feasible problem
  vstart(0) = 1.0;
  vstart(2) = 0.5;
  vend(0) = 1.0;
  vend(2) = -vstart(2);
  */
  Vector astart(3,dynConstraint->gravity),aend(3,dynConstraint->gravity);
  vector<GeneralizedCubicBezierCurve> path;
  vector<double> durations;
  bool res=interp.Make(zero,vstart,astart,one,vend,aend,path,durations);
  printf("Result: %d\n",res);
  if(res) {
    printf("Path size %d\n",path.size());
  }
}

void ContactOptimizeTest2(const char* robfile,const char* config1,const char* config2)
{
  Robot robot;
  if(!robot.Load(robfile)) {
    printf("Unable to load robot file %s\n",robfile);
    return;
  }

  Vector a,b;
  ifstream ia(config1,ios::in);
  ifstream ib(config2,ios::in);
  ia >> a;
  ib >> b;
  if(!ia || !ib) {
    printf("Unable to load config file(s)\n");
    return;
  }
  ia.close();
  ib.close();

  printf("Automatically detecting contacts...\n");
  robot.UpdateConfig(a);
  ContactFormation formation;
  GetFlatContacts(robot,5e-3,formation);
  printf("Assuming friction 0.5\n");
  for(size_t i=0;i<formation.contacts.size();i++) {
    printf("%d contacts on link %d\n",formation.contacts[i].size(),formation.links[i]);
    for(size_t j=0;j<formation.contacts[i].size();j++)
      formation.contacts[i][j].kFriction = 0.5;
  }
  Stance stance;
  LocalContactsToStance(formation,robot,stance);

  MultiPath path;
  path.sections.resize(1);
  MultiPath::PathSection& section = path.sections[0];
  path.SetStance(stance,0);

  vector<IKGoal> ikGoals;
  path.GetIKProblem(ikGoals);
  RobotConstrainedInterpolator interp(robot,ikGoals);
  //Real xtol = 5e-2;
  Real xtol = 1e-1;
  interp.ftol = 1e-6;
  interp.xtol = xtol;
  if(!interp.Project(a)) {
    printf("Failed to project config a\n");
    return;
  }
  if(!interp.Project(b)) {
    printf("Failed to project config b\n");
    return;
  }
  cout<<"Start: "<<a<<endl;
  cout<<"Goal: "<<b<<endl;
  vector<Vector> milestones,milestones2;
  if(!interp.Make(a,b,milestones)) {
    printf("Failed to interpolate\n");
    return;
  }
  if(!interp.Make(b,a,milestones2)) {
    printf("Failed to interpolate\n");
    return;
  }
  milestones.insert(milestones.end(),++milestones2.begin(),milestones2.end());
  //milestones2 = milestones;
  //milestones.insert(milestones.end(),++milestones2.begin(),milestones2.end());
  {
    cout<<"Saving geometric path to temp.path"<<endl;
    ofstream out("temp.path",ios::out);
    for(size_t i=0;i<milestones.size();i++)
      out<<Real(i)/Real(milestones.size()-1)<<"   "<<milestones[i]<<endl;
    out.close();
  }
    
  section.milestones = milestones;

  vector<Real> divs(101);
  for(size_t i=0;i<divs.size();i++)
    divs[i] = Real(i)/(divs.size()-1);
  Timer timer;
  ContactTimeScaling scaling(robot);
  scaling.frictionRobustness = 0.25;
  scaling.torqueRobustness = 0.25;
  scaling.forceRobustness = 0.05;
  bool res=scaling.SetParams(path,divs);
  if(!res) {
    printf("Unable to set contact scaling, time %g\n",timer.ElapsedTime());
    printf("Saving to scaling_constraints.csv\n");
    ofstream outc("scaling_constraints.csv",ios::out);
    outc<<"collocation point,planeindex,normal x,normal y,offset"<<endl;
    for(size_t i=0;i<scaling.ds2ddsConstraintNormals.size();i++) 
      for(size_t j=0;j<scaling.ds2ddsConstraintNormals[i].size();j++) 
	outc<<i<<","<<j<<","<<scaling.ds2ddsConstraintNormals[i][j].x<<","<<scaling.ds2ddsConstraintNormals[i][j].y<<","<<scaling.ds2ddsConstraintOffsets[i][j]<<endl;
    return;
  }
  printf("Contact scaling init successful, time %g\n",timer.ElapsedTime());
  printf("Saving to scaling_constraints.csv\n");
  ofstream outc("scaling_constraints.csv",ios::out);
  outc<<"collocation point,planeindex,normal x,normal y,offset"<<endl;
  for(size_t i=0;i<scaling.ds2ddsConstraintNormals.size();i++) 
    for(size_t j=0;j<scaling.ds2ddsConstraintNormals[i].size();j++) 
      outc<<i<<","<<j<<","<<scaling.ds2ddsConstraintNormals[i][j].x<<","<<scaling.ds2ddsConstraintNormals[i][j].y<<","<<scaling.ds2ddsConstraintOffsets[i][j]<<endl;


  res=scaling.Optimize();
  if(!res) {
    printf("Time scaling failed in time %g.  Path may be dynamically infeasible.\n",timer.ElapsedTime());
    return;
  }
  printf("Time scaling solved in time %g, execution time %g\n",timer.ElapsedTime(),scaling.traj.timeScaling.times.back());
  scaling.Check(path);
  /*
  for(size_t i=0;i<scaling.traj.ds.size();i++) {
    printf("time %g: rate %g\n",scaling.traj.times[i],scaling.ds[i]);
  }
  */
  {
    cout<<"Saving dynamically optimized path to temp_opt.path"<<endl;
    ofstream out("temp_opt.path",ios::out);
    Real dt = scaling.traj.EndTime()/100;
    Real t=0;
    for(size_t i=0;i<=100;i++) {
      Vector x;
      scaling.traj.Eval(t,x);
      out<<t<<"\t"<<x<<endl;
      t += dt;
    }
    out.close();
  }
}

void ContactOptimizeTest()
{
  Robot robot;
  if(!robot.Load("data/simple_2d_biped.rob")) {
    printf("Unable to load data/simple_2d_biped.rob\n");
    return;
  }

  MultiPath path;
  path.sections.resize(1);
  MultiPath::PathSection& section = path.sections[0];
  section.holds.resize(2);
  section.holds[0].contacts.resize(2);
  section.holds[0].contacts[0].x.set(-0.4,-0.1,0);
  section.holds[0].contacts[1].x.set(-0.4,0.1,0);
  section.holds[0].contacts[0].n.set(0,0,1);
  section.holds[0].contacts[1].n.set(0,0,1);
  section.holds[0].contacts[0].kFriction = 0.5;
  section.holds[0].contacts[1].kFriction = 0.5;
  section.holds[1].contacts.resize(2);
  section.holds[1].contacts[0].x.set(0.4,-0.1,0);
  section.holds[1].contacts[1].x.set(0.4,0.1,0);
  section.holds[1].contacts[0].n.set(0,0,1);
  section.holds[1].contacts[1].n.set(0,0,1);
  section.holds[1].contacts[0].kFriction = 0.5;
  section.holds[1].contacts[1].kFriction = 0.5;
  section.holds[0].link = 7;
  section.holds[1].link = 9;
  section.holds[0].SetupIKConstraint(Vector3(0.5,-0.1,0),Vector3(Zero));
  section.holds[1].SetupIKConstraint(Vector3(0.5,-0.1,0),Vector3(Zero));
  vector<IKGoal> ikGoals;
  path.GetIKProblem(ikGoals);

  RobotConstrainedInterpolator interp(robot,ikGoals);
  Real xtol = 5e-2;
  interp.ftol = 1e-4;
  interp.xtol = xtol;
  Vector a(7),b(7);
  ifstream ia("simple_2d_biped/a.config",ios::in);
  ifstream ib("simple_2d_biped/b.config",ios::in);
  ia >> a;
  ib >> b;
  ia.close();
  ib.close();
  if(!interp.Project(a)) {
    printf("Failed to project config a\n");
    return;
  }
  if(!interp.Project(b)) {
    printf("Failed to project config b\n");
    return;
  }
  cout<<"Start: "<<a<<endl;
  cout<<"Goal: "<<b<<endl;
  vector<Vector> milestones,milestones2;
  if(!interp.Make(a,b,milestones)) {
    printf("Failed to interpolate\n");
    return;
  }
  if(!interp.Make(b,a,milestones2)) {
    printf("Failed to interpolate\n");
    return;
  }
  milestones.insert(milestones.end(),++milestones2.begin(),milestones2.end());
  //milestones2 = milestones;
  //milestones.insert(milestones.end(),++milestones2.begin(),milestones2.end());
  {
    cout<<"Saving geometric path to temp.path"<<endl;
    ofstream out("temp.path",ios::out);
    for(size_t i=0;i<milestones.size();i++)
      out<<Real(i)/Real(milestones.size()-1)<<"   "<<milestones[i]<<endl;
    out.close();
  }
    
  section.milestones = milestones;

  vector<Real> divs(401);
  for(size_t i=0;i<divs.size();i++)
    divs[i] = Real(i)/(divs.size()-1);
  Timer timer;
  ContactTimeScaling scaling(robot);
  scaling.frictionRobustness = 0.25;
  scaling.torqueRobustness = 0.25;
  scaling.forceRobustness = 0.1;
  bool res=scaling.SetParams(path,divs);
  if(!res) {
    printf("Unable to set contact scaling, time %g\n",timer.ElapsedTime());
    return;
  }
  printf("Contact scaling init successful, time %g\n",timer.ElapsedTime());
  ofstream outc("scaling_constraints.csv",ios::out);
  outc<<"collocation point,planeindex,normal x,normal y,offset"<<endl;
  for(size_t i=0;i<scaling.ds2ddsConstraintNormals.size();i++) 
    for(size_t j=0;j<scaling.ds2ddsConstraintNormals[i].size();j++) 
      outc<<i<<","<<j<<","<<scaling.ds2ddsConstraintNormals[i][j].x<<","<<scaling.ds2ddsConstraintNormals[i][j].y<<","<<scaling.ds2ddsConstraintOffsets[i][j]<<endl;
  res = scaling.Optimize();
  if(!res) {
    printf("Time scaling failed in time %g.  Path may be dynamically infeasible.\n",timer.ElapsedTime());
    return;
  }
  printf("Time scaling solved in time %g, execution time %g\n",timer.ElapsedTime(),scaling.traj.timeScaling.times.back());

  printf("\n");
  printf("Checking path for feasibility...\n");
  scaling.Check(path);
  /*
  for(size_t i=0;i<scaling.traj.ds.size();i++) {
    printf("time %g: rate %g\n",scaling.traj.times[i],scaling.ds[i]);
  }
  */
  {
    cout<<"Saving dynamically optimized path to temp_opt.path"<<endl;
    ofstream out("temp_opt.path",ios::out);
    int numdivs = 200;
    Real dt = scaling.traj.EndTime()/numdivs;
    Real t=0;
    for(size_t i=0;i<=numdivs;i++) {
      Vector x;
      scaling.traj.Eval(t,x);
      out<<t<<"\t"<<x<<endl;
      t += dt;
    }
    out.close();
  }
}

int main(int argc, char** argv)
{
  //ConstrainedInterpolateTest();
  TimeOptimizeTest();
  //DynamicOptimizeTest();

  //ContactOptimizeTest();
  return 0;
  if(argc < 4) {
    printf("Usage: TrajOpt robot config1 config 2\n");
    return 1;
  }
  ContactOptimizeTest2(argv[1],argv[2],argv[3]);
  return 0;
}
