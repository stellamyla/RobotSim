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
#include "EdgePlanner.h"
#include "Miscellany.h"
#include <assert.h>
#include <math.h>
#include <algorithm>
#include <stdio.h>
using namespace std;


DummyEdgePlanner::DummyEdgePlanner(CSpace* _space,const Config& _a)
  :a(_a),space(_space)
{}

StraightLineEpsilonPlanner::StraightLineEpsilonPlanner(const Config& _a,const Config& _b,CSpace* _space,double _epsilon)
  :a(_a),b(_b),space(_space),epsilon(_epsilon)
{
  dist = space->Distance(a,b);
  depth = 0;
  segs = 1;
}

bool StraightLineEpsilonPlanner::IsVisible()
{
  while(dist > epsilon) {
    depth++;
    segs *= 2;
	dist *= ConstantHelper::Half;
    double du = ConstantHelper::One / (double)segs;
    double u = du;
    for(int k=1;k<segs;k+=2,u+=du+du) {
      space->Interpolate(a,b,u,m);
      if(!space->IsFeasible(m)) return false;
    }
  }
  return true;
}

void StraightLineEpsilonPlanner::Eval(double u,Config& x) const
{
  space->Interpolate(a,b,u,x);
}

EdgePlanner* StraightLineEpsilonPlanner::Copy() const
{
  StraightLineEpsilonPlanner* p=new StraightLineEpsilonPlanner(a,b,space,epsilon);
  p->depth=depth;
  p->segs=segs;
  p->dist=dist;
  return p;
}

EdgePlanner* StraightLineEpsilonPlanner::ReverseCopy() const
{
  StraightLineEpsilonPlanner* p=new StraightLineEpsilonPlanner(b,a,space,epsilon);
  p->depth=depth;
  p->segs=segs;
  p->dist=dist;
  return p;
}

double StraightLineEpsilonPlanner::Priority() const { return dist; }

bool StraightLineEpsilonPlanner::Plan() 
{
  depth++;
  segs *= 2;
  dist *= ConstantHelper::Half;
  double du = ConstantHelper::One / (double)segs;
  double u = du;
  for(int k=1;k<segs;k+=2,u+=du+du) {
    space->Interpolate(a,b,u,m);
    if(!space->IsFeasible(m)) {
      dist = ConstantHelper::Inf;
      return false;
    }
  }
  return true;
}

bool StraightLineEpsilonPlanner::Done() const { return dist <= epsilon; }

bool StraightLineEpsilonPlanner::Failed() const { return ConstantHelper::IsInf(dist); }  






StraightLineObstacleDistancePlanner::StraightLineObstacleDistancePlanner(const Config& _a,const Config& _b,CSpace* _space)
  :a(_a),b(_b),space(_space)
{}

bool StraightLineObstacleDistancePlanner::IsVisible()
{
  return CheckVisibility(a,b,space->ObstacleDistance(a),space->ObstacleDistance(b));
}

bool StraightLineObstacleDistancePlanner::CheckVisibility(const Config& a,const Config& b,double da,double db)
{
	double dmin = da < db ? da : db;
  if(dmin < ConstantHelper::Epsilon) {
    cout<<"Warning, da or db is close to zero"<<endl;
    return false;
  }
  double r = space->Distance(a,b);
  assert(r >= ConstantHelper::Zero);
  if(dmin > r) return true;
  Config m;
  space->Interpolate(a,b,0.5,m);
  if(!space->IsFeasible(m)) return false;
  double ram = space->Distance(a,m);
  double rbm = space->Distance(b,m);
  assert(ram < r*0.9 && ram > r*0.1);
  assert(rbm < r*0.9 && rbm > r*0.1);
  double dm = space->ObstacleDistance(m);
  assert(dm >= ConstantHelper::Zero);
  return CheckVisibility(a,m,da,dm)
    && CheckVisibility(m,b,dm,db);
}

void StraightLineObstacleDistancePlanner::Eval(double u,Config& x) const
{
  space->Interpolate(a,b,u,x);
}

EdgePlanner* StraightLineObstacleDistancePlanner::Copy() const
{
  StraightLineObstacleDistancePlanner* p=new StraightLineObstacleDistancePlanner(a,b,space);
  return p;
}

EdgePlanner* StraightLineObstacleDistancePlanner::ReverseCopy() const
{
  StraightLineObstacleDistancePlanner* p=new StraightLineObstacleDistancePlanner(b,a,space);
  return p;
}





BisectionEpsilonEdgePlanner::BisectionEpsilonEdgePlanner(const Config& a,const Config& b,CSpace* _space,double _epsilon)
  :space(_space),epsilon(_epsilon)
{
  path.push_back(a);
  path.push_back(b);
  Segment s;
  s.prev = path.begin();
  s.length = space->Distance(a,b);
  q.push(s);
}

BisectionEpsilonEdgePlanner::BisectionEpsilonEdgePlanner(CSpace* _space,double _epsilon)
  :space(_space),epsilon(_epsilon)
{}

bool BisectionEpsilonEdgePlanner::IsVisible()
{
  while(!Done()) {
    if(!Plan()) return false;
  }
  return true;
}

void BisectionEpsilonEdgePlanner::Eval(double u,Config& x) const
{
  //if(!Done()) cout<<"Warning, edge planner not done!"<<endl;
  if(ConstantHelper::IsNaN(u) || u < 0 || u > 1) {
    cout<<"Uh... evaluating path outside of [0,1] range"<<endl;
    cout<<"u="<<u<<endl;
    getchar();
  }
  assert(u >= ConstantHelper::Zero && u <= ConstantHelper::One);
  double dt = ConstantHelper::One/(double)(path.size()-1);
  double t=ConstantHelper::Zero;
  list<Config>::const_iterator i=path.begin();
  while(t+dt < u) {
    t+=dt;
    i++;
    if(i == path.end()) { cout<<"End of path, u="<<u<<endl; x=path.back(); return; }
  }
  assert(t<=u);
  if(t==u) { x=*i; }
  else {
    list<Config>::const_iterator n=i; n++;
    if(n != path.end()) {
      space->Interpolate(*i,*n,(u-t)/dt,x);
    }
    else {
      x = *i;
    }
  }
}

EdgePlanner* BisectionEpsilonEdgePlanner::Copy() const
{
  if(path.size() == 2 && q.size()==1) {  //uninitialized
    return new BisectionEpsilonEdgePlanner(path.front(),path.back(),space,epsilon);
  }
  else {
    BisectionEpsilonEdgePlanner* p=new BisectionEpsilonEdgePlanner(space,epsilon);
    p->path = path;
    if(!Done()) {
      cout<<"Warning: making a copy of a bisection edge planner that is not done!"<<endl;
      Segment s;
      s.prev = p->path.begin();
      s.length = space->Distance(path.front(),path.back());
      p->q.push(s);
      assert(!p->Done());
      //cout<<"Press any key to continue..."<<endl;
      //getchar();
    }
    return p;
  }
}

EdgePlanner* BisectionEpsilonEdgePlanner::ReverseCopy() const
{
  BisectionEpsilonEdgePlanner* p=new BisectionEpsilonEdgePlanner(space,epsilon);
  p->path.resize(path.size());
  reverse_copy(path.begin(),path.end(),p->path.begin());
  return p;
}

double BisectionEpsilonEdgePlanner::Priority() const
{
  if(q.empty()) return 0;
  return q.top().length;
}

bool BisectionEpsilonEdgePlanner::Plan()
{
  Segment s=q.top(); q.pop();
  list<Config>::iterator a=s.prev, b=a; b++;
  space->Interpolate(*a,*b,0.5,x);
  if(!space->IsFeasible(x)) { 
    //printf("Midpoint was not feasible\n");
    s.length=ConstantHelper::Inf; q.push(s); return false;
  }
  list<Config>::iterator m=path.insert(b,x);

  if(q.size()%100 == 0 &&
     double(q.size())*epsilon > 4.0*space->Distance(Start(),Goal())) {
    s.length = ConstantHelper::Inf;
    q.push(s);
    cout<<"BisectionEpsilonEdgePlanner: Over 4 times as many iterations as needed, quitting."<<endl;
    cout<<"Original length "<<space->Distance(Start(),Goal())<<", epsilon "<<epsilon<<endl;
    return false;
  }
  //insert the split segments back in the queue
  double l1=space->Distance(*a,x);
  double l2=space->Distance(x,*b);
  if(l1 > 0.9*s.length || l2 > 0.9*s.length) {
    //printf("Midpoint exceeded 0.9 time segment distance\n");
    s.length = ConstantHelper::Inf;
    q.push(s);
    return false;
  }
  s.prev = a;
  s.length = l1;
  if(s.length > epsilon) q.push(s);

  s.prev = m;
  s.length = l2;
  if(s.length > epsilon) q.push(s);
  return true;
}

bool BisectionEpsilonEdgePlanner::Plan(Config*& pre,Config*& post)
{
  Segment s=q.top(); q.pop();
  list<Config>::iterator a=s.prev, b=a; b++;
  space->Interpolate(*a,*b,0.5,x);

  //in case there's a failure...
  pre = &(*a); 
  post = &(*b);

  if(!space->IsFeasible(x))  { s.length=ConstantHelper::Inf; q.push(s); return false; }
  list<Config>::iterator m=path.insert(b,x);

  //insert the split segments back in the queue
  double l1=space->Distance(*a,x);
  double l2=space->Distance(x,*b);
  if(fabs(l1-l2) > 0.8*s.length) {
    s.length = ConstantHelper::Inf;
    q.push(s); 
    return false;
  }
  s.prev = a;
  s.length = l1;
  if(s.length > epsilon) q.push(s);

  s.prev = m;
  s.length = l2;
  if(s.length > epsilon) q.push(s);
  return true;
}

bool BisectionEpsilonEdgePlanner::Done() const
{
  return q.empty() || q.top().length <= epsilon;
}

bool BisectionEpsilonEdgePlanner::Failed() const
{
  if(q.empty()) return false;
  return ConstantHelper::IsInf(q.top().length);
}





