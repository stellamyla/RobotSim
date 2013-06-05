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
#ifndef GRAPH_CONNECTED_COMPONENTS_H
#define GRAPH_CONNECTED_COMPONENTS_H

#include "UndirectedGraph.h"
#include "UnionFind.h"

namespace Graph {

class ConnectedComponents
{
 public:
  template <class Node,class Edge>
  void Compute(const UndirectedGraph<Node,Edge>& G) {
    sets.Initialize(G.nodes.size());
    for(size_t i=0;i<G.nodes.size();i++) {
      for(typename Graph<Node,Edge>::EdgeIterator e=G.edges[i].begin();e!=G.edges[i].end();++e) {
	sets.Union(i,e->first);
      }
    }
  }
  void Resize(int numNodes) { sets.Initialize(numNodes); }
  void Clear() { Resize(0); }
  void AddEdge(int i,int j) { sets.Union(i,j); }
  void AddNode() { sets.AddEntry(); }
  int GetComponent(int i) { return sets.FindSet(i); }
  int GetComponent(int i) const { return sets.FindRoot(i); }
  bool SameComponent(int i,int j) { return sets.FindSet(i)==sets.FindSet(j); }
  bool SameComponent(int i,int j) const { return sets.FindRoot(i)==sets.FindRoot(j); }
  void GetRepresentatives(std::vector<int>& reps) const { sets.GetRoots(reps); }
  size_t NumComponents() const { return sets.CountSets(); }
  void EnumerateComponent(int node,std::vector<int>& items) const { return sets.EnumerateSet(node,items); }

  UnionFind sets;
};

} //namespace Graph

#endif
