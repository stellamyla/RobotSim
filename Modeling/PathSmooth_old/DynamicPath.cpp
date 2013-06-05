#include "DynamicPath.h"
#include <Robotics/EdgePlanner.h>
#include <math/random.h>
#include <structs/Heap.h>
#include <Timer.h>
#include "Planning/RobotCSpace.h"

//maximum iterations for distance-based line segment checker
const static int exactCheckMaxIters = 10000;

inline Real LInfDistance(const Vector& a,const Vector& b)
{
  assert(a.n==b.n);
  Real d=0;
  for(int i=0;i<a.n;i++)
    d = Max(d,Abs(a[i]-b[i]));
  return d;
}


inline bool BBContains(const Vector& x,const Vector& bmin,const Vector& bmax)
{
  assert(x.n==bmin.n);
  assert(x.n==bmax.n);
  for(int i=0;i<x.n;i++)
    if(x[i] < bmin[i] || x[i] > bmax[i]) return false;
  return true;
}

inline Real MinBBLInfDistance(const Vector& x,const Vector& bmin,const Vector& bmax)
{
  assert(x.n==bmin.n);
  assert(x.n==bmax.n);
  Real d=Inf;
  for(int i=0;i<x.n;i++)
    d = Min(d,Min(x[i]-bmin[i],bmax[i]-x[i]));
  return d;
}






struct RampSection
{
  Real ta,tb;
  Vector xa,xb;
  Real da,db;  //workspace radii
};


void BoundingBox(const ParabolicRamp1D& ramp,Real ta,Real tb,Real& bmin,Real& bmax)
{
  if(ta > tb) {  //orient the interval
    return BoundingBox(ramp,tb,ta,bmin,bmax);
  }
  if(ta < 0) ta = 0;
  if(tb <= 0) {
    bmin = bmax = ramp.x0;
    return;
  }
  if(tb > ramp.ttotal) tb=ramp.ttotal;
  if(ta >= ramp.ttotal) {
    bmin = bmax = ramp.x1;
    return;
  }

  bmin = ramp.Evaluate(ta);
  bmax = ramp.Evaluate(tb);
  if(bmin > bmax) Swap(bmin,bmax);

  Real tflip1=0,tflip2=0;
  if(ta < ramp.tswitch1) {
    //x' = a1*t + v0 = 0 => t = -v0/a1
    tflip1 = -ramp.dx0/ramp.a1;
    if(tflip1 > ramp.tswitch1) tflip1 = 0;
  }
  if(tb > ramp.tswitch2) {
    //x' = a2*(T-t) + v1 = 0 => (T-t) = v1/a2
    tflip2 = ramp.ttotal-ramp.dx1/ramp.a2;
    if(tflip2 < ramp.tswitch2) tflip2 = 0;
  }
  if(ta < tflip1 && tb > tflip1) {
    Real xflip = ramp.Evaluate(tflip1);
    if(xflip < bmin) bmin = xflip;
    else if(xflip > bmax) bmax = xflip;
  }
  if(ta < tflip2 && tb > tflip2) {
    Real xflip = ramp.Evaluate(tflip2);
    if(xflip < bmin) bmin = xflip;
    else if(xflip > bmax) bmax = xflip;
  }

  /*
  //double check
  Real t=0;
  while(t < ramp.ttotal) {
    Real x=ramp.Evaluate(t);
    assert(x >= bmin && x < bmax);
    t += 1e-3;
  }
  */
}

void BoundingBox(const ParabolicRampND& ramp,Real ta,Real tb,Vector& bmin,Vector& bmax)
{
  bmin.resize(ramp.ramps.size());
  bmax.resize(ramp.ramps.size());
  for(size_t i=0;i<ramp.ramps.size();i++) {
    BoundingBox(ramp.ramps[i],ta,tb,bmin[i],bmax[i]);
  }
}

void MaxExtremity(const Vector& x,const Vector& bmin,const Vector& bmax,Vector& dx)
{
  dx.resize(x.n);
  for(int i=0;i<x.n;i++) {
    if(Abs(bmax(i)-x(i)) > Abs(x(i)-bmin(i)))
      dx(i) = bmax(i)-x(i);
    else
      dx(i) = bmin(i)-x(i);
  }
}

    

bool CheckRampExact(const ParabolicRampND& ramp,int constraint,SingleRobotCSpace* space,int maxiters)
{
  RampSection section;
  section.ta = 0;
  section.tb = ramp.endTime;
  section.xa = ramp.x0;
  section.xb = ramp.x1;
  section.da = space->FreeWorkspaceBound(ramp.x0,constraint);
  section.db = space->FreeWorkspaceBound(ramp.x1,constraint);
  if(section.da == 0) {
    printf("Failed getting bound on start config\n");
    Assert(!space->CheckFeasible(section.xa,constraint));
    return false;
  }
  if(section.db == 0) {
    printf("Failed getting bound on end config\n");
    Assert(!space->CheckFeasible(section.xb,constraint));
    return false;
  }

  list<RampSection> queue;
  queue.push_back(section);
  int iters=0;
  while(!queue.empty()) {
    section = queue.front();
    queue.erase(queue.begin());
    //printf("Current section %g %g: config distance %g, obstacle distances %g, %g\n",section.ta,section.tb,LInfDistance(section.xa,section.xb),section.da,section.db);
    //getchar();

    //check the bounds around this section
    Vector bmin,bmax,dx;
    BoundingBox(ramp,section.ta,section.tb,bmin,bmax);
    MaxExtremity(section.xa,bmin,bmax,dx);
    Real dmax = space->WorkspaceMovementBound(section.xa,dx,constraint);
    MaxExtremity(section.xb,bmin,bmax,dx);
    dmax = Max(dmax,space->WorkspaceMovementBound(section.xb,dx,constraint));
    if(dmax <= section.da + section.db) {
      //can cull out the section
      continue;
    }

    Real tc = (section.ta+section.tb)*0.5;
    Vector xc;
    ramp.Evaluate(tc,xc);
    if(!space->CheckFeasible(xc,constraint)) {
      printf("Midpoint failure, constraint %s\n",space->ConstraintName(constraint).c_str());
      printf("Current section %g %g: config distance %g, obstacle distances %g, %g\n",section.ta,section.tb,LInfDistance(section.xa,section.xb),section.da,section.db);
      Assert(!space->IsFeasible(xc));
      return false;  //infeasible config
    }

    //subdivide
    RampSection sa,sb;
    sa.ta = section.ta;
    sa.xa = section.xa;
    sa.tb = sb.ta = tc;
    sa.xb = sb.xa = xc;
    sb.tb = section.tb;
    sb.xb = section.xb;
    sa.da = section.da;
    sb.db = section.db;

    sa.db=sb.da = space->FreeWorkspaceBound(sa.xb,constraint);
    if(sa.db == 0) {
      printf("Midpoint bound failure, constraint %s\n",space->ConstraintName(constraint).c_str());
      cout<<"Point "<<sa.xb<<endl;
      Assert(!space->CheckFeasible(sa.xb,constraint));
      Assert(!space->IsFeasible(sa.xb));
      return false;
    }

    //recurse on segments
    queue.push_back(sa);
    queue.push_back(sb);

    if(iters++ >= maxiters) {
      printf("CheckRampExact: timed out after %d iters \n",iters);
      return false;
    }
  }
  return true;
}

bool CheckRampExact(const ParabolicRampND& ramp,SingleRobotCSpace* space,int maxiters)
{
  if(!space->IsFeasible(ramp.x0)) {
    printf("CheckRampExact: start config failed \n");
    return false;
  }
  if(!space->IsFeasible(ramp.x1)) {
    printf("CheckRampExact: end config failed \n");
    return false;
  }

  int n=space->NumConstraints();
  int slowest = 0;
  double maxTime = 0;
  Timer overall;

  Vector bmin,bmax;
  Vector smin,smax;
  space->GetJointLimits(bmin,bmax);
  BoundingBox(ramp,0,ramp.endTime,smin,smax);
  if(!BBContains(smin,bmin,bmax) && BBContains(smax,bmin,bmax)) {
    printf("Ramp exited joint limits\n");
    return false;
  }
  for(int i=(int)space->GetRobot()->links.size();i<n;i++) {
    Timer timer;
    //printf("Checking constraint %s\n",space->ConstraintName(i).c_str());
    if(!CheckRampExact(ramp,i,space,maxiters)) {
      printf("Failed on constraint %s\n",space->ConstraintName(i).c_str());
      return false;
    }
    Real t=timer.ElapsedTime();
    if(t > maxTime) {
      slowest = i;
      maxTime = t;
    }
    /*
    int numdivs = (int)Ceil(ramp.endTime / 0.001);
    for(int d=1;d<numdivs;d++) {
      Vector x;
      ramp.Evaluate(Real(d)/Real(numdivs)*ramp.endTime,x);
      if(!space->CheckFeasible(x,i)) {
	printf("Incorrect on constraint %s\n",space->ConstraintName(i).c_str());
	getchar();
      }
    }
    */
  }
  /*
  int numdivs = (int)Ceil(ramp.endTime / 0.001);
  for(int d=1;d<numdivs;d++) {
    Vector x;
    ramp.Evaluate(Real(d)/Real(numdivs)*ramp.endTime,x);
    if(!space->IsFeasible(x)) {
      printf("Incorrect feasibility check!\n");
      getchar();
    }
  }
  if(!CheckRamp(ramp,space,0.001)) {
    printf("Durr... Incorrect feasibility check!\n");
    getchar();
  }
  */
  //printf("Slowest collision check %s, time %g / %g overall\n",space->ConstraintName(slowest).c_str(),maxTime,overall.ElapsedTime());
  return true;
}


DynamicPath::DynamicPath()
{}

DynamicPath::DynamicPath(const Robot& robot)
{
  Init(robot);
}

void DynamicPath::Init(const Robot& robot)
{
  velMax = robot.velMax;
  accMax = robot.accMax;
}

void DynamicPath::Init(const Vector& _velMax,const Vector& _accMax)
{
  velMax = _velMax;
  accMax = _accMax;
  Assert(velMax.n == accMax.n);
}

Real DynamicPath::GetTotalTime() const
{
  Real t=0;
  for(size_t i=0;i<ramps.size();i++) t+=ramps[i].endTime;
  return t;
}

void DynamicPath::Evaluate(Real t,Vector& x) const
{
  assert(!ramps.empty());
  if(t < 0) {
    x = ramps.front().x0;
  }
  else {
    for(size_t i=0;i<ramps.size();i++) {
      if(t <= ramps[i].endTime) {
	ramps[i].Evaluate(t,x);
	return;
      }
      t -= ramps[i].endTime;
    }
    x = ramps.back().x1;
  }
}

void DynamicPath::Derivative(Real t,Vector& dx) const
{
  assert(!ramps.empty());
  if(t < 0) {
    dx = ramps.front().dx0;
  }
  else {
    for(size_t i=0;i<ramps.size();i++) {
      if(t <= ramps[i].endTime) {
	ramps[i].Derivative(t,dx);
	return;
      }
      t -= ramps[i].endTime;
    }
    dx = ramps.back().dx1;
  }
}

void DynamicPath::SetMilestones(const vector<Vector>& x)
{
  if(x.empty()) {
    ramps.resize(0);
    return;
  }
  else if(x.size()==1) {
    ramps.resize(1);
    ramps[0].SetConstant(x[0]);
  }
  else {
    Vector zero(x[0].n,0.0);
    ramps.resize(x.size()-1);
    for(size_t i=0;i<ramps.size();i++) {
      ramps[i].x0 = x[i];
      ramps[i].x1 = x[i+1];
      ramps[i].dx0 = zero;
      ramps[i].dx1 = zero;
      bool res=ramps[i].SolveMinTimeLinear(accMax,velMax);
      Assert(res);
    }
  }
}

void DynamicPath::SetMilestones(const vector<Vector>& x,const vector<Vector>& dx)
{
  if(x.empty()) {
    ramps.resize(0);
    return;
  }
  else if(x.size()==1) {
    ramps.resize(1);
    ramps[0].SetConstant(x[0]);
  }
  else {
    Vector zero(x[0].n,0.0);
    ramps.resize(x.size()-1);
    for(size_t i=0;i<ramps.size();i++) {
      ramps[i].x0 = x[i];
      ramps[i].x1 = x[i+1];
      ramps[i].dx0 = dx[i];
      ramps[i].dx1 = dx[i+1];
      bool res=ramps[i].SolveMinTime(accMax,velMax);
      Assert(res);
    }
  }
}

void DynamicPath::GetMilestones(vector<Vector>& x,vector<Vector>& dx) const
{
  if(ramps.empty()) {
    x.resize(0);
    dx.resize(0);
    return;
  }
  x.resize(ramps.size()+1);
  dx.resize(ramps.size()+1);
  x[0] = ramps[0].x0;
  dx[0] = ramps[0].dx0;
  for(size_t i=0;i<ramps.size();i++) {
    x[i+1] = ramps[i].x1;
    dx[i+1] = ramps[i].dx1;
  }
}

void DynamicPath::Append(const Vector& x)
{
  size_t n=ramps.size();
  size_t p=n-1;
  ramps.resize(ramps.size()+1);
  if(ramps.size()==1) {
    ramps[0].SetConstant(x);
  }
  else {
    ramps[n].x0 = ramps[p].x1;
    ramps[n].dx0 = ramps[p].dx1;
    ramps[n].x1 = x;
    ramps[n].dx1.resize(x.n);
    ramps[n].dx1.setZero();
    bool res=ramps[n].SolveMinTime(accMax,velMax);
    Assert(res);
  }
}

void DynamicPath::Append(const Vector& x,const Vector& dx)
{
  size_t n=ramps.size();
  size_t p=n-1;
  ramps.resize(ramps.size()+1);
  if(ramps.size()==1) {
    FatalError("Can't append milestone with a nonzero velocity to an empty path ");
  }
  else {
    ramps[n].x0 = ramps[p].x1;
    ramps[n].dx0 = ramps[p].dx1;
    ramps[n].x1 = x;
    ramps[n].dx1 = dx;
    bool res=ramps[n].SolveMinTime(accMax,velMax);
    Assert(res);
  }
}

void DynamicPath::Concat(const DynamicPath& suffix)
{
  assert(&suffix != this);
  if(suffix.ramps.empty()) return;
  if(ramps.empty()) {
    *this=suffix;
    return;
  }
  Assert(ramps.back().x1 == suffix.ramps.front().x0);
  Assert(ramps.back().dx1 == suffix.ramps.front().dx0);
  ramps.insert(ramps.end(),suffix.ramps.begin(),suffix.ramps.end());
}

void DynamicPath::Split(Real t,DynamicPath& before,DynamicPath& after) const
{
  assert(&before != this);
  assert(&after != this);
  if(ramps.empty()) {
    before=*this;
    after=*this;
    return;
  }
  after.velMax = before.velMax = velMax;
  after.accMax = before.accMax = accMax;
  after.ramps.resize(0);
  before.ramps.resize(0);
  if(t < 0) {  //we're before the path starts
    before.velMax = velMax;
    before.accMax = accMax;
    before.ramps.resize(1);
    before.ramps[0].SetConstant(ramps[0].x0);
    //place a constant for time -t on the after path
    after.ramps.resize(1);
    after.ramps[0].SetConstant(ramps[0].x0,-t);
  }
  for(size_t i=0;i<ramps.size();i++) {
    if(t < 0) {
      after.ramps.push_back(ramps[i]);
    }
    else {
      if(t < ramps[i].endTime) {
	//cut current path
	ParabolicRampND temp=ramps[i];
	temp.TrimBack(temp.endTime-t);
	before.ramps.push_back(temp);
	temp=ramps[i];
	temp.TrimFront(t);
	if(!after.ramps.empty()) {
	  printf("DynamicPath::Split: Uh... weird, after is not empty?\n");
	  printf("t = %g, i = %d, endtime = %g\n",t,i,ramps[i].endTime);
	}
	assert(after.ramps.size() == 0);
	after.ramps.push_back(temp);
      }
      else {
	before.ramps.push_back(ramps[i]);
      }
      t -= ramps[i].endTime;
    }
  }

  if(t > 0) {  //dt is longer than path
    ParabolicRampND temp;
    temp.SetConstant(ramps.back().x1,t);
    before.ramps.push_back(temp);
  }
  if(t >= 0) {
    ParabolicRampND temp;
    temp.SetConstant(ramps.back().x1);
    after.ramps.push_back(temp);
  }
  assert(before.IsValid());
  assert(after.IsValid());
}


struct RampBisector
{
  RampBisector() {}
  RampBisector(int _i,int _j) : i(_i),j(_j) {}
  int i,j;
  SmartPointer<EdgePlanner> e;
};

bool CheckRamp(const ParabolicRampND& ramp,CSpace* space,Real tol)
{
  Assert(tol > 0);
  if(!space->IsFeasible(ramp.x0)) return false;
  if(!space->IsFeasible(ramp.x1)) return false;
  //Assert(space->IsFeasible(ramp.x0));
  //Assert(space->IsFeasible(ramp.x1));

  //for a parabola of form f(x) = a x^2 + b x, and the straight line 
  //of form g(X,u) = u*f(X)
  //d^2(g(X,u),p) = |p - <f(X),p>/<f(X),f(X)> f(X)|^2 < tol^2
  //<p,p> - <f(X),p>^2/<f(X),f(X)>  = p^T (I-f(X)f(X)^T/f(X)^T f(X)) p
  //max_x d^2(f(x)) => f(x)^T (I-f(X)f(X)^T/f(X)^T f(X)) f'(x) = 0
  //(x^2 a^T + x b^T) A (2a x + b) = 0
  //(x a^T + b^T) A (2a x + b) = 0
  //2 x^2 a^T A a + 3 x b^T A a + b^T A b = 0

  //the max X for which f(x) deviates from g(X,x) by at most tol is...
  //max_x |g(X,x)-f(x)| = max_x x/X f(X)-f(x)
  //=> f(X)/X - f'(x) = 0
  //=>  X/2 = x 
  //=> max_x |g(X,x)-f(x)| = |(X/2)/X f(X)-f(X/2)|
  //= |1/2 (aX^2+bX) - a(X/2)^2 - b(X/2) + c |
  //= |a| X^2 / 4
  //so... max X st max_x |g(X,x)-f(x)| < tol => X = 2*sqrt(tol/|a|)

  vector<Real> divs;
  vector<Real> cumDist;
  vector<Config> qdiv;
  Real distSum = 0;
  Config q1,q2;
  q2 = ramp.x0;
  Real t=0;
  divs.push_back(t);
  qdiv.push_back(q2);
  cumDist.push_back(0);
  while(t < ramp.endTime) {
    Real tnext=t;
    Real amax = 0;
    Real switchNext=ramp.endTime;
    for(size_t i=0;i<ramp.ramps.size();i++) {
      if(t < ramp.ramps[i].tswitch1) {  //ramp up
	switchNext =  Min(switchNext, ramp.ramps[i].tswitch1);
	amax = Max(amax,ramp.ramps[i].a1);
      }
      else if(t < ramp.ramps[i].tswitch2) {  //constant vel
	switchNext = Min(switchNext, ramp.ramps[i].tswitch2);
      }
      else if(t < ramp.ramps[i].ttotal) {  //ramp down
	amax = Max(amax,ramp.ramps[i].a2);	
      }
    }
    Assert(switchNext > t);
    Real dt = 2.0*Sqrt(tol/amax);
    if(t+dt > switchNext) tnext = switchNext;
    else tnext = t+dt;

    q1 = q2;
    ramp.Evaluate(tnext,q2);
    distSum += space->Distance(q1,q2)+tol;
    cumDist.push_back(distSum);
    t = tnext;
    divs.push_back(tnext);
    qdiv.push_back(q2);
  }
  q1 = q2;
  q2 = ramp.x1;
  distSum += space->Distance(q1,q2)+tol;
  cumDist.push_back(distSum);
  divs.push_back(ramp.endTime);
  qdiv.push_back(q2);
  //printf("Path of time %g, %d divs\n",ramp.endTime,divs.size());

  //do a bisection thingie
  Heap<RampBisector,Real> segs;
  RampBisector temp(0,divs.size()-1);
  segs.push(temp,cumDist.back());
  while(!segs.empty()) {
    temp=segs.top();
    int i=temp.i;
    int j=temp.j;
    segs.pop();
    assert(i < j);
    if(j == i+1) {
      if(!temp.e) {
	//check path from t to tnext
	temp.e = space->LocalPlanner(qdiv[i],qdiv[j]);
      }
      bool res=temp.e->Plan();
      if(!res) { 
	//printf("CheckRamp: Intermediate (%g,%g) failed\n",divs[i],divs[j]);
	return false;
      }
      if(!temp.e->Done())
	segs.push(temp,temp.e->Priority());
    }
    else {
      int k=(i+j)/2;
      if(!space->IsFeasible(qdiv[k])) {
	//printf("CheckRamp: Point %g failed\n",divs[k]);
	return false;
      }
      segs.push(RampBisector(i,k),cumDist[k]-cumDist[i]);
      segs.push(RampBisector(k,j),cumDist[j]-cumDist[k]);
    }
  }
  return true;
}

bool DynamicPath::TryShortcut(Real t1,Real t2,CSpace* space,Real tol)
{
  Assert(IsValid());
  /*
  if(tol < 0) {
    printf("Testing existing ramps, exact checking\n");
    for(size_t i=0;i<ramps.size();i++) {
      printf("Ramp %d...\n",i);
      if(!CheckRampExact(ramps[i],dynamic_cast<SingleRobotCSpace*>(space),exactCheckMaxIters)) {
	printf("Ramp %d failed exact collision check\n",i);
	getchar();
      }
    }
  }
  */

  vector<Real> rampStartTime(ramps.size()); 
  Real endTime=0;
  for(size_t i=0;i<ramps.size();i++) {
    rampStartTime[i] = endTime;
    endTime += ramps[i].endTime;
  }
  if(t1 > t2) Swap(t1,t2);
  Assert(t1 >= 0 && t2 >= 0);
  Assert(t1 <= endTime && t2 <= endTime);

  int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
  int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
  if(i1 < 0 || i1 >= (int)ramps.size()) return false;
  if(i2 < 0 || i2 >= (int)ramps.size()) return false;
  if(i1 == i2) return false; //same ramp
  Assert(i2 > i1);
  Real u1 = t1-rampStartTime[i1];
  Real u2 = t2-rampStartTime[i2];
  Assert(u1 >= 0);
  Assert(u1 <= ramps[i1].endTime+Epsilon);
  Assert(u2 >= 0);
  Assert(u2 <= ramps[i2].endTime+Epsilon);
  u1 = Min(u1,ramps[i1].endTime);
  u2 = Min(u2,ramps[i2].endTime);
  ParabolicRampND test;
  ramps[i1].Evaluate(u1,test.x0);
  ramps[i2].Evaluate(u2,test.x1);
  ramps[i1].Derivative(u1,test.dx0);
  ramps[i2].Derivative(u2,test.dx1);
  if(test.x0.n != test.x1.n) {
    fprintf(stderr,"Weird problem in DynamicPath::TryShortcut... will Abort after enter is pressed\n");
    getchar();
    Abort();
  }
  bool res=test.SolveMinTime(accMax,velMax);
  if(!res) return false;
  Assert(test.endTime >= 0);
  Assert(test.IsValid());
  if(tol < 0) {
    if(!CheckRampExact(test,dynamic_cast<SingleRobotCSpace*>(space),exactCheckMaxIters))   //perform shortcut
      return false;
  }
  else {
    if(!CheckRamp(test,space,tol))   //perform shortcut
      return false;
  }

  //crop i1 and i2
  ramps[i1].TrimBack(ramps[i1].endTime-u1);
  ramps[i1].x1 = test.x0;
  ramps[i1].dx1 = test.dx0;
  ramps[i2].TrimFront(u2);
  ramps[i2].x0 = test.x1;
  ramps[i2].dx0 = test.dx1;
  
  /*
  //test the in-between ramps
  ParabolicRampND temp;
  temp = ramps[i1];
  if(temp.SolveMinTime(accMax,velMax) == false) {
    fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
    fprintf(stderr,"Press enter to continue\n");
    getchar();
  }
  temp = ramps[i2];
  if(temp.SolveMinTime(accMax,velMax) == false) {
    fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
    fprintf(stderr,"Press enter to continue\n");
    getchar();
  }
  */

  //TEMP
  if(tol < 0) {
    if(!CheckRampExact(ramps[i1],dynamic_cast<SingleRobotCSpace*>(space),exactCheckMaxIters)) {
      printf("TryShortcut: Warning, failed when checking lead-up to shortcut!\n");
      //printf("Press enter to continue\n");
      //getchar();
    }
    if(!CheckRampExact(ramps[i2],dynamic_cast<SingleRobotCSpace*>(space),exactCheckMaxIters)) {
      printf("TryShortcut: Warning, failed when checking lead-down from shortcut!\n");
      //printf("Press enter to continue\n");
      //getchar();
    }
  }
  else {
    if(!CheckRamp(ramps[i1],space,tol)) {  
      printf("TryShortcut: Warning, failed when checking lead-up to shortcut!\n");
      //printf("Press enter to continue\n");
      //getchar();
    }
    if(!CheckRamp(ramps[i2],space,tol)) { 
      printf("TryShortcut: Warning, failed when checking lead-down from shortcut!\n");
      //printf("Press enter to continue\n");
      //getchar();
    }
  }
  
  //replace intermediate ramps with test
  for(int i=0;i<i2-i1-1;i++)
    ramps.erase(ramps.begin()+i1+1);
  ramps.insert(ramps.begin()+i1+1,test);
  
  //check for consistency
  /*
  if(!IsValid()) {
    printf("Did a shortcut on edge %d\n",i1+1);
    for(size_t i=0;i<test.ramps.size();i++)
      printf("\t%g %g %g\n",test.ramps[i].a1,test.ramps[i].v,test.ramps[i].a2);
    cout<<"Vel max: "<<velMax<<endl;
    cout<<"Acc max: "<<accMax<<endl;
  }
  */
  Assert(IsValid());
  return true;
}

int DynamicPath::Shortcut(CSpace* space,Real tol,Real timeLimit,int numIters)
{
  if(timeLimit <= 0) return 0;
  Timer timer;
  int shortcuts = 0;
  Real endTime = GetTotalTime();
  for(int iters=0;(iters<numIters) && (timer.ElapsedTime() < timeLimit);iters++) {
    Real t1=Rand()*endTime,t2=Rand()*endTime;
    if(TryShortcut(t1,t2,space,tol)) {
      endTime = GetTotalTime();
      shortcuts++;
    }
  }
  return shortcuts;
}

int DynamicPath::DynamicShortcut(Real leadTime,Real padTime,CSpace* space,Real tol)
{
  Timer timer;
  int shortcuts = 0;
  vector<Real> rampStartTime(ramps.size()); 
  Real endTime=0;
  for(size_t i=0;i<ramps.size();i++) {
    rampStartTime[i] = endTime;
    endTime += ramps[i].endTime;
  }
  while(1) {
    //can only start from here
    Real starttime = timer.ElapsedTime()-leadTime;
    if(starttime+padTime >= endTime) break;
    starttime = Max(0.0,starttime+padTime);

    Real t1=starttime+Sqr(Rand())*(endTime-starttime),t2=Rand(starttime,endTime);
    if(t1 > t2) Swap(t1,t2);
    int i1 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t1)-rampStartTime.begin()-1;
    int i2 = std::upper_bound(rampStartTime.begin(),rampStartTime.end(),t2)-rampStartTime.begin()-1;
    if(i1 == i2) continue; //same ramp
    Real u1 = t1-rampStartTime[i1];
    Real u2 = t2-rampStartTime[i2];
    Assert(u1 >= 0);
    Assert(u1 <= ramps[i1].endTime+Epsilon);
    Assert(u2 >= 0);
    Assert(u2 <= ramps[i2].endTime+Epsilon);
    u1 = Min(u1,ramps[i1].endTime);
    u2 = Min(u2,ramps[i2].endTime);
    ParabolicRampND test;
    ramps[i1].Evaluate(u1,test.x0);
    ramps[i2].Evaluate(u2,test.x1);
    ramps[i1].Derivative(u1,test.dx0);
    ramps[i2].Derivative(u2,test.dx1);
    bool res=test.SolveMinTime(accMax,velMax);
    if(!res) continue;
    Assert(test.endTime >= 0);
    Assert(test.IsValid());
    if(CheckRamp(test,space,tol)) {  //perform shortcut
      //check for time elapse, otherwise can't perform this shortcut
      if(timer.ElapsedTime()-leadTime > t1) continue; 

      shortcuts++;
      //crop i1 and i2
      ramps[i1].TrimBack(ramps[i1].endTime-u1);
      ramps[i1].x1 = test.x0;
      ramps[i1].dx1 = test.dx0;
      ramps[i2].TrimFront(u2);
      ramps[i2].x0 = test.x1;
      ramps[i2].dx0 = test.dx1;
      Assert(ramps[i1].IsValid());
      Assert(ramps[i2].IsValid());

      //test the in-between ramps
      ParabolicRampND temp;
      temp = ramps[i1];
      if(temp.SolveMinTime(accMax,velMax) == false) {
	fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
	fprintf(stderr,"Press enter to continue\n");
	getchar();
      }
      temp = ramps[i2];
      if(temp.SolveMinTime(accMax,velMax) == false) {
	fprintf(stderr,"Warning, truncated ramp can't be re-solved\n");
	fprintf(stderr,"Press enter to continue\n");
	getchar();
      }

      //replace intermediate ramps with test
      for(int i=0;i<i2-i1-1;i++)
	ramps.erase(ramps.begin()+i1+1);
      ramps.insert(ramps.begin()+i1+1,test);

      //check for consistency
      for(size_t i=0;i+1<ramps.size();i++) {
	Assert(ramps[i].x1 == ramps[i+1].x0);
	Assert(ramps[i].dx1 == ramps[i+1].dx0);
      }

      //revise the timing
      rampStartTime.resize(ramps.size());
      endTime=0;
      for(size_t i=0;i<ramps.size();i++) {
	rampStartTime[i] = endTime;
	endTime += ramps[i].endTime;
      }
    }
  }
  return shortcuts;
}

int DynamicPath::ShortCircuit(CSpace* space,Real tol)
{
  int shortcuts=0;
  ParabolicRampND test;
  for(size_t i=0;i+1<ramps.size();i++) {
    test.x0 = ramps[i].x0;
    test.dx0 = ramps[i].dx0;
    test.x1 = ramps[i+1].x1;    
    test.dx1 = ramps[i+1].dx1;    
    bool res=test.SolveMinTime(accMax,velMax);
    if(!res) continue;
    Assert(test.endTime >= 0);
    Assert(test.IsValid());
    if(CheckRamp(test,space,tol)) {  //perform shortcut    
      ramps[i] = test;
      ramps.erase(ramps.begin()+i+1);
      i--;
      shortcuts++;
    }
  }
  return shortcuts;
}


bool DynamicPath::IsValid() const
{
  if(ramps.empty()) {
    fprintf(stderr,"DynamicPath::IsValid: empty path\n");
    return false;
  }
  for(size_t i=0;i<ramps.size();i++) {
    if(!ramps[i].IsValid()) {
      fprintf(stderr,"DynamicPath::IsValid: ramp %d is invalid\n",i);
      return false;
    }
    for(size_t j=0;j<ramps[i].ramps.size();j++) {
      if(Abs(ramps[i].ramps[j].a1) > accMax[j]+Epsilon ||
	 Abs(ramps[i].ramps[j].v) > velMax[j] ||
	 Abs(ramps[i].ramps[j].a2) > accMax[j]+Epsilon) {
	fprintf(stderr,"DynamicPath::IsValid: invalid acceleration or velocity on ramp %d\n",i);
	fprintf(stderr,"\ta1 %g, v %g, a2 %g.  amax %g, vmax %g\n",ramps[i].ramps[j].a1,ramps[i].ramps[j].v,ramps[i].ramps[j].a2,accMax[j],velMax[j]);
	return false;
      }
    }
  }
  for(size_t i=1;i<ramps.size();i++) {
    if(ramps[i].x0 != ramps[i-1].x1) {
      fprintf(stderr,"DynamicPath::IsValid: discontinuity at ramp %d\n",i);
      cerr<<ramps[i].x0<<endl;
      cerr<<ramps[i-1].x1<<endl;
      return false;
    }
    if(ramps[i].dx0 != ramps[i-1].dx1) {
      fprintf(stderr,"DynamicPath::IsValid: derivative discontinuity at ramp %d\n",i);
      cerr<<ramps[i].dx0<<endl;
      cerr<<ramps[i-1].dx1<<endl;
      return false;
    }
  }
  return true;
}
