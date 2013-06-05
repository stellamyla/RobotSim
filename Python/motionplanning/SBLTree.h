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
#ifndef ROBOTICS_SBL_TREE_H
#define ROBOTICS_SBL_TREE_H

#include "Tree.h"
#include "SmartPointer.h"
#include "GridSubdivision.h"
#include <list>
#include "CSpace.h"
#include "EdgePlanner.h"

/** @ingroup MotionPlanning
 * @brief A tree of configurations to be used in the SBL motion planner.
 */
class SBLTree
{
public:
  typedef Graph::TreeNode<Config,SmartPointer<EdgePlanner> > Node;
  struct EdgeInfo
  {
    Node *s,*t;
    SmartPointer<EdgePlanner>  e;
    bool reversed;
  };

  SBLTree(CSpace*);
  virtual ~SBLTree();
  virtual void Cleanup();
  virtual void Init(const Config& qStart);
  virtual Node* Extend(double maxDistance,int maxIters);

  virtual void AddMilestone(Node* n) {}
  virtual void RemoveMilestone(Node* n) {}
  virtual Node* PickExpand();

  //helpers
  Node* AddMilestone(const Config& q) { Node* n=new Node(q); AddMilestone(n); return n; }
  bool HasNode(Node* n) const;
  Node* AddChild(Node* n,const Config& x);
  Node* FindClosest(const Config& x);

  //path from ts->ns->ng->tg
  static bool CheckPath(SBLTree* ts, Node* ns,SBLTree* tg,Node* ng,std::list<EdgeInfo>& outputPath);

  CSpace* space;
  Node *root;
};

/** @ingroup MotionPlanning
 * @brief An SBLTree with a node index
 */
class SBLTreeWithIndex : public SBLTree
{
 public:
  SBLTreeWithIndex(CSpace*);
  virtual void Cleanup();  
  virtual void AddMilestone(Node* n);
  virtual void RemoveMilestone(Node* n);
  virtual Node* PickExpand() { return PickRandom(); }
  Node* PickRandom() const;

  std::vector<Node*> index;
};

/** @ingroup MotionPlanning
 * @brief A grid-based subdivision to be used for SBL.
 *
 * The grid operates on certain dimensions of the configuration.
 * Specifically, it picks mappedDims dimensions at random from
 * the full configuration space, and divides the space in those
 * dimensions into cells of width h.  If the mapped dimension is
 * that of index k.
 * 
 * NOTE: h must be initialized to the # of dims in the 
 * configuration space, and containing some cell width value, say 0.1.
 * @todo Do a defualt initialization of h.
 */
class SBLSubdivision
{
public:
  typedef SBLTree::Node Node;

  SBLSubdivision(int mappedDims);
  void Clear();
  void RandomizeSubset();
  void AddPoint(Node*);
  void RemovePoint(Node*);
  Node* PickPoint(const Config& x);
  Node* PickRandom();

  Vector h;
  std::vector<int> subset;
  Geometry::GridSubdivision subdiv;

  //temporary
  Vector temp;
};

/** @ingroup MotionPlanning
 * @brief An SBL motion planner that uses a SBLSubdivision to pick the
 * next node to expand, and nodes to connect.
 */
class SBLTreeWithGrid : public SBLTree
{
public:
  SBLTreeWithGrid(CSpace*);
  virtual void Init(const Config& qStart);
  virtual void Cleanup();
  ///Initializes the grids Astart,Agoal to a configuration space
  ///of numDims dimensions, uniform cell width of h
  void InitDefaultGrid(int numDims,double h);
  ///Randomizes the dimensions of the grid divisions Astart,Agoal
  void RandomizeSubset();

  virtual void AddMilestone(Node* n);
  virtual void RemoveMilestone(Node* n);
  virtual Node* PickExpand();
  
  Node* FindNearby(const Config& x);

  SBLSubdivision A;
};


#endif

