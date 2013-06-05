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
#ifndef GRAPH_SHORTEST_PATHS_H
#define GRAPH_SHORTEST_PATHS_H

#include "Graph.h"
#include "Miscellany.h"
#include "FixedSizeHeap.h"
#include "Heap.h"
#include <set>

namespace Graph {
	using namespace ConstantHelper;

const static Weight inf = ConstantHelper::Inf;

/** @ingroup Graph
 * @brief Single-source shortest paths (Dijkstra's algorithm)
 *
 * WeightFunc is a callback that returns the weight of an edge.  More
 * specifically, it returns a Weight when given an Edge,source,target
 * as argument.
 *
 * The start node(s) is set with InitializeSource(s)().  The resulting
 * shortest path(s) are calculated using Find[A,All]Path().  The predecessor
 * graph is stored in p (where -1 represents no predecessor) and d is the
 * distance list.
 *
 * FindPath returns when the target node t is reached.
 * The variant FindAPath takes a list of target nodes, and produces the
 * shortest path to one of those nodes.  It returns the node.
 * The variant FindAllPaths solves for all shortest paths.
 *
 * This also allows dynamic updating of shortest paths.  To increase an 
 * edge's weight, call IncreaseUpdate(), and to decrease an edge's weight, 
 * call DecreaseUpdate().  To remove an edge completely, call DeleteUpdate().
 */
template <class Node,class Edge>
class ShortestPathProblem
{
 public:
  ShortestPathProblem(const Graph<Node,Edge>& g);
  virtual ~ShortestPathProblem() {}
  
  void InitializeSource(int s);
  void InitializeSources(const vector<int>& s);

  template <typename WeightFunc,typename Iterator>
  void FindPath(int t,WeightFunc w,Iterator it);
  template <typename WeightFunc,typename Iterator>
  int FindAPath(const vector<int>& t,WeightFunc w,Iterator it);
  template <typename WeightFunc,typename Iterator>
  inline void FindAllPaths(WeightFunc w,Iterator it) { FindPath(-1,w,it); }

  //after all paths are found, the shortest paths can be updated with the following
  //when an edge (u,v)'s weight is increased/decreased, call these methods
  template <typename WeightFunc,typename InIterator,typename OutIterator>
  void IncreaseUpdate(int u,int v,WeightFunc w,InIterator in,OutIterator out);
  template <typename WeightFunc,typename InIterator,typename OutIterator>
  void DecreaseUpdate(int u,int v,WeightFunc w,InIterator in,OutIterator out);
  template <typename WeightFunc,typename InIterator,typename OutIterator>
  void DeleteUpdate(int u,int v,WeightFunc w,InIterator in,OutIterator out);

  template <typename WeightFunc,typename Iterator>
  bool HasShortestPaths(int s,WeightFunc w,Iterator it);

  //template specializations (directed/undirected)

  template <typename WeightFunc>
  inline void FindPath_Directed(int t,WeightFunc w) {
    FindPath(t,w,EdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline int FindAPath_Directed(const vector<int>& t,WeightFunc w) {
    return FindAPath(t,w,EdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline void FindAllPaths_Directed(WeightFunc w) {
    EdgeIterator<Edge> it;
    FindAllPaths(w,it);
  }
  template <typename WeightFunc>
  inline void IncreaseUpdate_Directed(int u,int v,WeightFunc w) {
    CoEdgeIterator<Edge> in; EdgeIterator<Edge> out;
    IncreaseUpdate(u,v,w,in,out);
  }
  template <typename WeightFunc>
  inline void DecreaseUpdate_Directed(int u,int v,WeightFunc w) {
    CoEdgeIterator<Edge> in; EdgeIterator<Edge> out;
    DecreaseUpdate(u,v,w,in,out);
  }
  template <typename WeightFunc>
  inline void DeleteUpdate_Directed(int u,int v,WeightFunc w) {
    CoEdgeIterator<Edge> in; EdgeIterator<Edge> out;
    DeleteUpdate(u,v,w,in,out);
  }
  template <typename WeightFunc>
  inline bool HasShortestPaths_Directed(int s,WeightFunc w) {
    CoEdgeIterator<Edge> it;
    return HasShortestPaths(s,w,it);
  }

  template <typename WeightFunc>
  inline void FindPath_Undirected(int t,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    FindPath(t,w,it);
  }
  template <typename WeightFunc>
  inline int FindAPath_Undirected(const vector<int>& t,WeightFunc w) {
    return FindAPath(t,w,UndirectedEdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline void FindAllPaths_Undirected(WeightFunc w) {
    FindAllPaths(w,UndirectedEdgeIterator<Edge>());
  }
  template <typename WeightFunc>
  inline void IncreaseUpdate_Undirected(int u,int v,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    IncreaseUpdate(u,v,w,it,it);
    IncreaseUpdate(v,u,w,it,it);
  }
  template <typename WeightFunc>
  inline void DecreaseUpdate_Undirected(int u,int v,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    DecreaseUpdate(u,v,w,it,it);
    DecreaseUpdate(v,u,w,it,it);
  }
  template <typename WeightFunc>
  inline void DeleteUpdate_Undirected(int u,int v,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    DeleteUpdate(u,v,w,it,it);
    DeleteUpdate(v,u,w,it,it);
  }
  template <typename WeightFunc>
  inline bool HasShortestPaths_Undirected(int s,WeightFunc w) {
    UndirectedEdgeIterator<Edge> it;
    return HasShortestPaths(s,w,it);
  }

  virtual void OnDistanceUpdate(int n) {}

  const Graph<Node,Edge>& g;

  std::vector<int> p;
  std::vector<Weight> d;
};


//definition of ShortestPaths

template <class Node,class Edge>
ShortestPathProblem<Node,Edge>::
ShortestPathProblem(const Graph<Node,Edge>& _g)
  :g(_g)
{}

template <class Node,class Edge>
void ShortestPathProblem<Node,Edge>::InitializeSource(int s)
{
  //initialize
  int nn = g.NumNodes();
  p.resize(nn);
  d.resize(nn);
  for(int i=0;i<nn;i++) {
    p[i] = -1;
    d[i] = inf;
  }
  d[s] = 0;
}

template <class Node,class Edge>
void ShortestPathProblem<Node,Edge>::InitializeSources(const vector<int>& s)
{
  //initialize
  int nn = g.NumNodes();
  p.resize(nn);
  d.resize(nn);
  for(int i=0;i<nn;i++) {
    p[i] = -1;
    d[i] = inf;
  }
  for(size_t i=0;i<s.size();i++)
    d[s[i]] = 0;
}

template <class Node,class Edge>
template <typename WeightFunc,typename Iterator>
void ShortestPathProblem<Node,Edge>::FindPath(int t,WeightFunc w,Iterator it)
{
  int nn=g.NumNodes();
  FixedSizeHeap<Weight> H(nn);  //O(n) init, worst case log n update
  for(int i=0;i<nn;i++) H.push(i,-d[i]);
  
  while(!H.empty()) {
    //pop the smallest distance element from q
    int u = H.top(); H.pop();
    if(u == t) return;
    
    for(g.Begin(u,it);!it.end();it++) {
      int v=it.target();
      //relax distances
      Weight dvu = d[u] + w(*it,it.source(),it.target());
      if(d[v] > dvu) {
	d[v] = dvu;
	p[v] = u;
	OnDistanceUpdate(v);
	H.adjust(v,-d[v]);
      }
    }
  } 
}

template <class Node,class Edge>
template <typename WeightFunc,typename Iterator>
int ShortestPathProblem<Node,Edge>::FindAPath(const vector<int>& t,WeightFunc w,Iterator it)
{
  int nn=g.NumNodes();
  FixedSizeHeap<Weight> H(nn);  //O(n) init, worst case log n update
  for(int i=0;i<nn;i++) H.push(i,-d[i]);

  set<int> targetSet;
  for(size_t i=0;i<t.size();i++) targetSet.insert(t[i]);

  while(!H.empty()) {
    //pop the smallest distance element from q
    int u = H.top(); H.pop();
    if(targetSet.count(u)!=0) return u;
    
    for(g.Begin(u,it);!it.end();it++) {
      int v=it.target();
      //relax distances
      Weight dvu = d[u] + w(*it,it.source(),it.target());
      if(d[v] > dvu) {
	d[v] = dvu;
	p[v] = u;
	OnDistanceUpdate(v);
	H.adjust(v,-d[v]);
      }
    }
  } 
  return -1;
}


template <class Node,class Edge>
template <typename WeightFunc,typename InIterator,typename OutIterator>
void ShortestPathProblem<Node,Edge>::IncreaseUpdate(int u,int v,WeightFunc w,
						    InIterator in,OutIterator out)
{
  //1) if not in the SP tree, return
  if(p[v] != u) return;

  //2) look for alternative SP, if found, return
  for(g.Begin(v,in);!in.end();in++) {
    int t=in.target();
    if(d[v] == d[t]+w(*in,in.source(),in.target())) {
      p[v]=t;
      return;
    }
  }

  //3) identify affected nodes
  vector<int> Q(1,v);
  for(size_t i=0;i<Q.size();i++) {
    int n=Q[i];
    d[n] = inf;
    OnDistanceUpdate(n);
    for(g.Begin(n,out);!out.end();out++) {
      int t=out.target();
      if(p[t]==n) {
	//identify any alternative routes
	for(g.Begin(t,in);!in.end();in++) {
	  int s=in.target();
	  if(d[t]==d[s]+w(*in,in.source(),in.target())) {
	    p[t]=s;
	    break;
	  }
	}
	if(p[t]==n) Q.push_back(t);
      }
    }
  }
  //4) update affected nodes by finding paths from outside nodes
  Heap<int,Weight> H;  //O(1) init, O(n) adjust
  for(size_t i=0;i<Q.size();i++) {
    int n=Q[i];
    for(g.Begin(n,in);!in.end();in++) {
      int s=in.target();
      double W=w(*in,in.source(),in.target());
      if(d[n]>d[s]+W) {
	d[n]=d[s]+W;
	p[n]=s;
	OnDistanceUpdate(n);
      }
    }
    if(!IsInf(d[n])) H.push(n,-d[n]);
  }
  while(!H.empty()) {
    int n=H.top(); H.pop();
    for(g.Begin(n,out);!out.end();out++) {
      int t=out.target();
      double W=w(*out,out.source(),out.target());
      if(d[t]>d[n]+W) {
	d[t]=d[n]+W;
	p[t]=n;
	OnDistanceUpdate(t);
	H.adjust(t,-d[t]);
      }
    }
  }
}

template <class Node,class Edge>
template <typename WeightFunc,typename InIterator,typename OutIterator>
void ShortestPathProblem<Node,Edge>::DecreaseUpdate(int u,int v,WeightFunc w,
						    InIterator in,OutIterator out)
{
  //NOTE: can't use find edge because it doesn't work for undirected graphs
  for(g.Begin(u,out);!out.end();out++)
    if(out.target()==v) {
      double W=w(*out,out.source(),out.target());
      if(d[v] <= d[u]+W) {
	return;  //no change
      }
      d[v] = d[u]+W;
      p[v] = u;
      OnDistanceUpdate(v);
      break;
    }
  if(out.end()) {
    fprintf(stderr,"ShortestPathProblem::DecreaseUpdate(): Warning, decreasing an edge that doesn't exist in the graph!\n");
    abort();
  }
  Heap<int,Weight> H;   //O(1) init, O(n) adjust
  H.push(v,-d[v]);
  while(!H.empty()) {
    int n=H.top(); H.pop();
    for(g.Begin(n,out);!out.end();out++) {
      int t=out.target();
      double W=w(*out,out.source(),out.target());
      if(d[t]>d[n]+W) {
	d[t]=d[n]+W;
	p[t]=n;
	OnDistanceUpdate(t);
	H.adjust(t,-d[t]);
      }
    }
  }
}

template <class Node,class Edge>
template <typename WeightFunc,typename InIterator,typename OutIterator>
void ShortestPathProblem<Node,Edge>::DeleteUpdate(int u,int v,WeightFunc w,
						  InIterator in,OutIterator out)
{
  if(p[v] == u) {
    d[v] = inf;
    p[v] = -1;
    OnDistanceUpdate(v);
    //find the best alternate parent
    for(g.Begin(v,in);!in.end();++in) {
      int t=in.target();
      if(p[t] == v) continue;
      assert(t != u);
      double W=w(*in,in.source(),in.target());
      if(d[v] > d[t]+W) {
	d[v] = d[t]+W;
	p[v] = t;
	OnDistanceUpdate(v);
      }
    }
    if(p[v] != -1) {
      d[v] = inf;
      DecreaseUpdate(p[v],v,w,in,out);
    }
    else {
      for(g.Begin(v,out);!out.end();++out) {
	int t=in.target();
	IncreaseUpdate(v,t,w,in,out);
      }
    }
  }
}

template <class Node,class Edge>
template <typename WeightFunc,typename Iterator>
bool ShortestPathProblem<Node,Edge>::HasShortestPaths(int s,WeightFunc w,Iterator it) {
  int nn=g.NumNodes();
  FixedSizeHeap<Weight> H(nn);
  for(int i=0;i<nn;i++) H.push(i,-d[i]);
  while(!H.empty()) {
    int n=H.top(); H.pop();
    if(n==s) {
      if(d[n]!=0) {
	printf("The start doesn't have distance 0\n");
	return false;
      }
    }
    for(g.Begin(n,it);!it.end();++it) {
      int t=it.target();
      double W=w(*it,it.source(),it.target());
      if(p[n] == t) {
	if(fabs(d[n]-d[t]-W) > 1e-10) {
	  printf("Inconsistency in node's weight through parent, %f vs %f\n", d[n],d[t]+W);
	  return false;
	}
      }
      if(d[n]-d[t]-W > 1e-10) {
	printf("There exists a shorter path (%d,%d) not (%d,%d)!\n",t,n,p[n],n);
	printf("Weight 1 is %f compared to %f\n", d[t]+W, d[n]);
	return false;
      }
    }
  }
  return true;
}


} //namespace Graph

#endif
