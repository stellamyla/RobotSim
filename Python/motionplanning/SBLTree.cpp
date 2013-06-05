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
#include "SBLTree.h"
#include "Random.h"
//#include "errors.h"
#include "Miscellany.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <assert.h>
using namespace std;
using namespace Geometry;
using namespace ConstantHelper;


typedef SBLTree::Node Node;
typedef EdgePlanner Edge;


// ClosestMilestoneCallback: finds the closest milestone to x 
struct ClosestMilestoneCallback : public Node::Callback
{
  ClosestMilestoneCallback(CSpace* s,const Config& _x)
    :space(s),closestDistance(Inf),x(_x),closestMilestone(NULL)
  {}
  virtual void Visit(Node* n) {
    double d = space->Distance(x,*n);
    if(d < closestDistance) {
      closestDistance = d;
      closestMilestone = n;
    }
  }
  CSpace* space;
  double closestDistance;
  const Config& x;
  Node* closestMilestone;
};

struct AddPointCallback : public Node::Callback
{
  AddPointCallback(SBLSubdivision& _s) :s(_s) {}
  virtual void Visit(Node* n) { s.AddPoint(n); }
  SBLSubdivision& s;
};

struct PickCallback : public Node::Callback
{
  PickCallback(int _k) :i(0),k(_k),res(NULL) {}
  virtual void Visit(Node* n) { 
    if(i==k) res=n;
    i++;
  }
  virtual bool Stop() { return (res!=NULL); }

  int i;
  int k;
  Node* res;
};

//moves nodes from tree a to b
struct ChangeTreeCallback : public Node::Callback
{
  ChangeTreeCallback(SBLTree* _a,SBLTree* _b)
    :a(_a),b(_b)
  {}
  virtual void Visit(Node* n) { 
    a->RemoveMilestone(n);
    b->AddMilestone(n);
  }

  SBLTree *a, *b;
};









SBLTree::SBLTree(CSpace* s)
  :space(s),root(NULL)
{}

SBLTree::~SBLTree()
{
  Cleanup();
}

void SBLTree::Cleanup()
{
  SafeDelete(root);
}

void SBLTree::Init(const Config& qRoot)
{
  assert(!root);
  assert(space->IsFeasible(qRoot));
  root = AddMilestone(qRoot);
}

Node* SBLTree::Extend(double maxDistance,int maxIters)
{
  Node* n=PickExpand();
  Config x;
  for(int i=1;i<=maxIters;i++) {
    double r = maxDistance/i;
    space->SampleNeighborhood(*n,r,x);
    if(space->IsFeasible(x)) {
      //add as child of n
      return AddChild(n,x);
    }
  }
  return NULL;
}

Node* SBLTree::AddChild(Node* n,const Config& x)
{
  Node* c=AddMilestone(x);
  c->edgeFromParent() = space->LocalPlanner(*n,*c);
  return n->addChild(c);
}

bool SBLTree::HasNode(Node* n) const
{
  return n==root || n->hasAncestor(root);
}




struct LessEdgePriority
{
  typedef SBLTree::EdgeInfo EdgeInfo;
  bool operator() (EdgeInfo& a,EdgeInfo& b) const
  {
    return a.e->Priority() < b.e->Priority();
  }
};

bool SBLTree::CheckPath(SBLTree* s,Node* ns,SBLTree* g,Node* ng,std::list<EdgeInfo>& outputPath)
{
  CSpace* space=s->space;
  assert(s->space == g->space);
  //cout<<"Checking path!!!"<<endl;
  assert(s->HasNode(ns));
  assert(g->HasNode(ng));
  assert(outputPath.empty());
  //start -> ns -> ng -> goal
  SmartPointer<EdgePlanner> bridge = space->LocalPlanner(*ns,*ng);  //edge from ns to ng

  priority_queue<EdgeInfo,vector<EdgeInfo>,LessEdgePriority> q;

  //start->ns
  EdgeInfo temp;
  Node* n=ns;
  while(n->getParent() != NULL) {
    temp.s = n->getParent();
    temp.t = n;
    temp.e = n->edgeFromParent();
    temp.reversed=false;
    outputPath.push_front(temp);
    n=n->getParent();
  }

  //ns->ng
  temp.s = ns;
  temp.t = ng;
  temp.e = bridge;
  temp.reversed=false;
  outputPath.push_back(temp);

  //ng->goal
  n=ng;
  while(n->getParent() != NULL) {
    temp.s = n;
    temp.t = n->getParent();
    temp.e = n->edgeFromParent();
    temp.reversed=true;
    outputPath.push_back(temp);
    n=n->getParent();
  }
  assert(outputPath.front().s == s->root);
  assert(outputPath.back().t == g->root);
  for(list<EdgeInfo>::iterator i=outputPath.begin();i!=outputPath.end();i++) {
    list<EdgeInfo>::iterator n=i; n++;
    if(n != outputPath.end())
      assert(i->t == n->s);
  }

  for(list<EdgeInfo>::iterator i=outputPath.begin();i!=outputPath.end();i++)
    q.push(*i);
  
  //adaptive division of path
  Config x;
  while(!q.empty()) {
    temp=q.top(); q.pop();
    if(temp.e->Done()) continue;
    //double len=temp.e->Priority();
    if(!temp.e->Plan()) {
      //disconnect!
      if(temp.e == bridge) {
	//cout<<"Disconnecting edge between connected nodes"<<endl;
	//no change in graph
	bridge = NULL;
      }
      else if(s->HasNode(temp.s)) {
	//cout<<"Disconnecting edge on start tree"<<endl;
	//disconnect tree from s->t
	temp.s->detachChild(temp.t);
	assert(temp.t == ns || temp.t->hasDescendent(ns));
	ns->reRoot();
	ns->edgeFromParent() = bridge;
	
	//move nodes in subtree from start arrays to goal arrays
	ChangeTreeCallback changeCallback(s,g);
	ns->DFS(changeCallback);
	ng->addChild(ns);
	assert(ns->hasAncestor(g->root));
	assert(s->root->getParent()==NULL);
      }
      else {     //on goal tree
	//cout<<"Disconnecting edge on goal tree"<<endl;
	assert(g->HasNode(temp.t));
	//disconnect tree from s->t
	temp.t->detachChild(temp.s);
	assert(temp.s == ng || temp.s->hasDescendent(ng));
	ng->reRoot();
	ng->edgeFromParent() = bridge;
	
	//move nodes in subtree from goal arrays to start arrays
	ChangeTreeCallback changeCallback(g,s);
	ng->DFS(changeCallback);
	ns->addChild(ng);
	assert(ng->hasAncestor(s->root));
	assert(g->root->getParent()==NULL);
      }
      outputPath.clear();
      return false;
    }
    if(!temp.e->Done()) q.push(temp);
  }

  //done!
  //cout<<"Path checking success"<<endl;

  //check the reversed flags for the output path
  for(list<EdgeInfo>::iterator i=outputPath.begin();i!=outputPath.end();i++) {
    if(i->reversed) {
      if(*i->s == i->e->Start()) i->reversed=false;
    }
    else {
      if(*i->s == i->e->Goal()) i->reversed=true;
    }
  }
  return true;
}

Node* SBLTree::PickExpand()
{
  fprintf(stderr,"PickExpand not implemented by SBLTree subclass\n");
  abort();
  return NULL;
}

Node* SBLTree::FindClosest(const Config& x)
{
  //walk through start tree
  ClosestMilestoneCallback callback(space,x);
  root->DFS(callback);
  return callback.closestMilestone;
}



SBLTreeWithIndex::SBLTreeWithIndex(CSpace* space)
  :SBLTree(space)
{}

void SBLTreeWithIndex::Cleanup()
{
  index.resize(0);
}

void SBLTreeWithIndex::AddMilestone(Node* n)
{
  index.push_back(n);
}

void SBLTreeWithIndex::RemoveMilestone(Node* n)
{
  vector<Node*>::iterator i=find(index.begin(),index.end(),n);
  if(i == index.end()) return;
  *i = index.back();
  index.resize(index.size()-1);
}

Node* SBLTreeWithIndex::PickRandom() const
{
  Graph::CountCallback<Node*> count;
  root->DFS(count);
  PickCallback pick(RandHelper::randInt(count.count));
  root->DFS(pick);
  assert(pick.res != NULL);
  return pick.res;
}


SBLSubdivision::SBLSubdivision(int mappedDims)
  :subdiv(mappedDims,0.1)
{
  subset.resize(mappedDims);
  for(int i=0;i<mappedDims;i++)
    subset[i] = i;
  temp.resize(mappedDims);
}

void SBLSubdivision::Clear()
{
  subdiv.Clear();
}

void SBLSubdivision::RandomizeSubset()
{
  assert(h.size() >= (size_t)subset.size());
  int n=h.size();
  vector<int> p(n); //make a permutation
  for(int i=0;i<n;i++) p[i]=i;
  for(size_t i=0;i<subset.size();i++) {
    p[i] = p[i+rand()%(n-i)];
    subset[i] = p[i];
  }
  subdiv.h.resize(subset.size());
  for(size_t i=0;i<subset.size();i++)
    subdiv.h[i] = h[subset[i]];
  temp.resize(subset.size());
}

void SBLSubdivision::AddPoint(Node* n)
{
  assert(temp.size() == subset.size());
  const Vector& p = *n;
  for(size_t i=0;i<subset.size();i++)
    temp[i] = p[subset[i]];

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  subdiv.Insert(index,(void*)n);
}

void SBLSubdivision::RemovePoint(Node* n)
{
  assert(temp.size() == subset.size());
  const Vector& p = *n;
  for(size_t i=0;i<subset.size();i++)
    temp[i] = p[subset[i]];

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  bool res=subdiv.Erase(index,(void*)n);
  assert(res == true);
}

void* RandomObject(const list<void*>& objs)
{
  //pick a random point from objs
  int n=objs.size();
  assert(n != 0);
  int k=RandHelper::randInt(n);
  list<void*>::const_iterator obj=objs.begin();
  for(int i=0;i<k;i++,obj++);
  return (*obj);
}

Node* SBLSubdivision::PickPoint(const Config& x)
{
  assert(temp.size() == subset.size());
  for(size_t i=0;i<subset.size();i++)
    temp[i] = x[subset[i]];

  GridSubdivision::Index index;
  subdiv.PointToIndex(temp,index);
  list<void*>* objs = subdiv.GetObjectSet(index);
  if(!objs) return NULL;

  return (Node*)RandomObject(*objs);
}

Node* SBLSubdivision::PickRandom()
{
  int n=subdiv.buckets.size();
  assert(n > 0);
  int k = RandHelper::randInt(n);
  GridSubdivision::HashTable::iterator bucket=subdiv.buckets.begin();
  for(int i=0;i<k;i++,bucket++);
  return (Node*)RandomObject(bucket->second);
}




inline int NumDims(CSpace* space)
{
  Vector x;
  space->Sample(x);
  return (int)x.size();
}

SBLTreeWithGrid::SBLTreeWithGrid(CSpace* space)
  :SBLTree(space),A(Min(3,NumDims(space)))
{
}

void SBLTreeWithGrid::Cleanup()
{
  A.Clear();
  SBLTree::Cleanup();
}

void SBLTreeWithGrid::Init(const Config& qStart)
{
  SBLTree::Init(qStart);
  A.Clear();
  A.AddPoint(root);
}

void SBLTreeWithGrid::InitDefaultGrid(int numDims,double h)
{
  A.h.resize(numDims,h);
}

void SBLTreeWithGrid::RandomizeSubset()
{
  A.Clear();
  A.RandomizeSubset();

  if(root) {
    AddPointCallback callback(A);
    root->DFS(callback);
  }
}

void SBLTreeWithGrid::AddMilestone(Node* n)
{
  A.AddPoint(n);
}

void SBLTreeWithGrid::RemoveMilestone(Node* n)
{
  A.RemovePoint(n);
}

Node* SBLTreeWithGrid::PickExpand()
{
  return A.PickRandom();
}

Node* SBLTreeWithGrid::FindNearby(const Config& x)
{
  Node* n=A.PickPoint(x);
  if(n) return n;
  return A.PickRandom();
}
