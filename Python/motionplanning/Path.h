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
#ifndef GRAPH_PATH_H
#define GRAPH_PATH_H

#include <vector>
#include <list>
#include <map>

/** @file Path.h 
 * @ingroup Graph
 * @brief Functions for creating paths out of ancestor lists.
 */

namespace Graph {

/** @ingroup Graph
 * @brief Calculates a path of nodes from a list of parents.
 *
 * @param p The array of parents such that p[n] is the parent of node n
 * @param n The destination node
 * @param lastAncestor Either the start node or -1
 * @param path The output path from lastAncestor to n
 * @return true if the path reached lastAncestor
 */
inline bool GetAncestorPath(const std::vector<int>& p,
			    int n,int lastAncestor,
			    std::list<int>& path) {
  path.clear();
  path.push_front(n);
  if(n == lastAncestor) return true;
  int i=0;
  while(p[n] != -1) {
    n = p[n];
    path.push_front(n);
    if(n == lastAncestor) return true;
    if(i++ > (int)p.size()) {
      printf("GetAncestorPath(): Iterated more than the number of nodes, aborting\n");
      i=0;
      for(std::list<int>::iterator it=path.begin();it!=path.end()&&i<20;it++,i++) 
	printf("%d ",*it);
      printf("\n");
      abort();
    }
  }
  return (lastAncestor == -1);
}

/** @ingroup Graph
 * @brief Same as above, but with an arbitrary Node type
 */
template <class Node>
inline void GetAncestorPath(const std::map<Node,Node>& p,
			    Node n,std::list<Node>& path) 
{
  path.clear();
  path.push_front(n);
  int i=0;
  while(p.count(n) != 0) {
    n = p[n];
    path.push_front(n);
    if(i++ > p.size()) {
      printf("GetAncestorPath(): Iterated more than the number of nodes, aborting\n");
      abort();
    }
  }
}

}  //namespace Graph;

#endif
