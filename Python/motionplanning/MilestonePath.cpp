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
#include "MilestonePath.h"
#include "Random.h"
#include "Miscellany.h"
#include <assert.h>
#include <math.h>

MilestonePath::MilestonePath()
{}

MilestonePath::~MilestonePath()
{}

void MilestonePath::Concat(const MilestonePath& path)
{
  edges.reserve(edges.size()+path.edges.size());
  for(size_t i=0;i<path.edges.size();i++) {
    edges.push_back(path.edges[i]);
  }
}

bool MilestonePath::InitializeEdgePlans()
{
  bool res=true;
  for(size_t i=0;i<edges.size();i++) {
    if(!edges[i]->IsVisible()) res=false;
  }
  return res;
}

void MilestonePath::CreateEdgesFromMilestones(CSpace* space,const vector<Config>& milestones)
{
  assert(!milestones.empty());
  edges.resize(milestones.size()-1);
  for(size_t i=0;i+1<milestones.size();i++) {
    edges[i] = space->LocalPlanner(milestones[i],milestones[i+1]);
  }
}

bool MilestonePath::IsFeasible()
{
  if(edges.empty()) return true;
  //first check endpoints 
  CSpace* space=Space();
  if(!space->IsFeasible(edges[0]->Start())) return false;
  for(size_t i=0;i<edges.size();i++) {
    if(!space->IsFeasible(edges[i]->Goal())) return false;
  }
  //then check edges
  for(size_t i=0;i<edges.size();i++) 
    if(!edges[i]->IsVisible()) return false;
  return true;
}

int MilestonePath::Eval(double t, Config& c) const
{
	if(t <= ConstantHelper::Zero) { c = edges.front()->Start(); return 0; }
  else if(t >= ConstantHelper::One) { c = edges.back()->Goal(); return edges.size()-1; }
  else {
    double u=t*(double)edges.size();
    double u0=floor(u);
    int index = (int)u0;
    assert(index >= 0 && index < (int)edges.size());
    edges[index]->Eval(u-u0,c);
    return index;
  }
}

int MilestonePath::Shortcut()
{
  int numShortcuts=0;
  size_t i=0;
  while(i+1 < edges.size()) {
    //try to connect milestone i to i+2
    const Config& x1=GetMilestone(i);
    const Config& x2=GetMilestone(i+2);
    EdgePlanner* e= IsVisible(edges[i]->Space(),x1,x2);
    if(e) {
      edges[i]=e;
      edges.erase(edges.begin()+i+1);
      numShortcuts++;
      //don't advance to next milestone
    }
    else //advance to next milestone
      i++; 
  }
  return numShortcuts;
}

int MilestonePath::Reduce(int numIters)
{
  CSpace* space=Space();
  //pick random points on the path, connect them if they're visible
  Config x1,x2;
  int i1,i2;
  int numsplices=0;
  for(int iters=0;iters<numIters;iters++) {
    i1 = rand()%edges.size();
    i2 = rand()%edges.size();
    if(i2 < i1) swap(i1,i2);
    else if(i1 == i2) continue;  //if they're on the same segment, forget it

	double t1=RandHelper::rand();
    double t2=RandHelper::rand();
    edges[i1]->Eval(t1,x1);
    edges[i2]->Eval(t2,x2);
    const Config& a=edges[i1]->Start();
    const Config& b=edges[i2]->Goal();
    EdgePlanner* e_x1x2=space->LocalPlanner(x1,x2);
    if(e_x1x2->IsVisible()) {
      EdgePlanner* e_ax1=space->LocalPlanner(a,x1);
      EdgePlanner* e_x2b=space->LocalPlanner(x2,b);
      if(e_ax1->IsVisible() && e_x2b->IsVisible()) {
	numsplices++;
	cout<<"Visible subsegment "<<i1<<"->"<<i2<<endl;
	//replace edges a->a',...,b'->b with a->x1,x1->x2,x2->b
	edges.erase(edges.begin()+i1,edges.begin()+i2+1);

	edges.insert(edges.begin()+i1,e_ax1);
	edges.insert(edges.begin()+i1+1,e_x1x2);
	edges.insert(edges.begin()+i1+2,e_x2b);
      }
      else {
	delete e_ax1;
	delete e_x2b;
      }
    }
    else {
      delete e_x1x2;
    }
  }
  return numsplices;
}

void MilestonePath::Discretize(double h)
{
  for(size_t i=0;i<edges.size();i++) {
    int n=DiscretizeEdge(i,h);
    i+=n-1;
  }
}

int MilestonePath::DiscretizeEdge(int i,double h)
{
  EdgePlanner* e=edges[i];
  const Config& a=e->Start();
  const Config& b=e->Goal();
  CSpace* space=e->Space();
  int numDivs = (int)ceil(space->Distance(a,b)/h);
  //don't do anything...
  if(numDivs <= 1) return 1;

  //create a bunch of replacement edges
  double du=ConstantHelper::One/numDivs;
  double u=0;
  Config x1,x2;
  MilestonePath replacement;
  for(int k=0;k<numDivs;k++) {
    if(k==0) x1=a;
    else e->Eval(u,x1);
    if(k+1==numDivs) x2=b;
    else e->Eval(u+du,x2);
    EdgePlanner* e2 = space->LocalPlanner(x1,x2);
    if(e2->IsVisible()) 
      replacement.edges.push_back(e2);
    else {
      cerr<<"Warning, reparameterized edge "<<i<<" is infeasible"<<endl;
      replacement.edges.push_back(e2);
    }
    u += du;
  }
  assert(!replacement.edges.empty());
  Splice(i,i+1,replacement);
  return replacement.edges.size();
}

void MilestonePath::DiscretizeEdge(int i,const vector<double>& u)
{
  assert(u.front()==0 && u.back()==1);
  EdgePlanner* e=edges[i];
  CSpace* space=e->Space();
  if(u.size()==2) return;

  Config x1,x2;
  MilestonePath replacement;
  x1 = e->Start();
  for(size_t k=1;k<u.size();k++) {
    e->Eval(u[k],x2);
    EdgePlanner* e2 = space->LocalPlanner(x1,x2);
    if(e2->IsVisible()) 
      replacement.edges.push_back(e2);
    else {
      cerr<<"Warning, reparameterized edge "<<i<<" is infeasible"<<endl;
      replacement.edges.push_back(e2);
    }
    x1 = x2;
  }
  Splice(i,i+1,replacement);
}

bool MilestonePath::IsValid()
{
  if(edges.empty()) return false;
  CSpace* space=Space();
  for(size_t i=0;i<edges.size();i++) {
    if(Space(i) != space) return false;
    if(i!=0) 
      if(edges[i]->Start() != edges[i-1]->Goal()) return false;
  }
  return true;
}

double MilestonePath::Length() const
{
  double len=0;
  for(size_t i=0;i<edges.size();i++)
    len += Space(i)->Distance(edges[i]->Start(),edges[i]->Goal());
  return len;
}

const Config& MilestonePath::GetMilestone(int i) const
{
  assert(i>=0 && i<=(int)edges.size());
  if(i==(int)edges.size())
    return edges.back()->Goal();
  return edges[i]->Start();
}

void MilestonePath::SetMilestone(int i,const Config& x)
{
  if(i == 0) {
    //first milestone
    const Config& b=edges[i]->Goal();
    edges[i] = Space(i)->LocalPlanner(x,b);
  }
  else if(i==(int)edges.size()) {
    assert(!edges.empty());
    //last milestone
    const Config& a=edges[i-1]->Start();
    edges[i-1] = Space(i-1)->LocalPlanner(a,x);
  }
  else {
    const Config& a=edges[i-1]->Start();
    const Config& b=edges[i]->Goal();
    edges[i-1] = Space(i-1)->LocalPlanner(a,x);
    edges[i] = Space(i)->LocalPlanner(x,b);
  }
}

bool MilestonePath::CheckSetMilestone(int i,const Config& x)
{
  if(i == 0) {
    //first milestone
    const Config& b=edges[i]->Goal();
    EdgePlanner* e2=IsVisible(Space(i),x,b);
    if(!e2) return false;
    edges[i] = e2;
    return true;
  }
  else if(i==(int)edges.size()) {
    assert(!edges.empty());
    //last milestone
    const Config& a=edges[i-1]->Start();
    EdgePlanner* e1=IsVisible(Space(i-1),a,x);
    if(!e1) return false;
    edges[i-1] = e1;
    return true;
  }
  else {
    const Config& a=edges[i-1]->Start();
    const Config& b=edges[i]->Goal();
    EdgePlanner* e1=IsVisible(Space(i-1),a,x);
    if(!e1) return false;
    EdgePlanner* e2=IsVisible(Space(i),x,b);
    if(!e2) { delete e1; return false; }
    
    edges[i-1] = e1;
    edges[i] = e2;
    return true;
  }
}

void MilestonePath::Splice(int start,int goal,const MilestonePath& path)
{
  if(start >= 0) assert(start <= (int)edges.size());
  if(goal >= 0) assert(goal <= (int)edges.size());
  if(start < 0) start=0;
  if(goal < 0) goal=(int)edges.size();
  edges.erase(edges.begin()+start,edges.begin()+goal);
  edges.insert(edges.begin()+start,path.edges.begin(),path.edges.end());
}

bool MilestonePath::Load(istream& in,CSpace* space)
{
  assert(space != NULL);
  vector<Config> configs;
  int n;
  in>>n;
  if(in.bad()) return false;
  assert(n > 0);
  configs.reserve(n);
  Config temp;
  for(int i=0;i<n;i++) {
    in>>temp;
    configs.push_back(temp);
  }
  CreateEdgesFromMilestones(space,configs);
  return true;
}

bool MilestonePath::Save(ostream& out)
{
  assert(!edges.empty());
  out<<edges.size()+1<<endl;
  for(size_t i=0;i<edges.size();i++)
    out<<edges[i]->Start()<<endl;
  out<<edges.back()->Goal()<<endl;
  return true;
}
