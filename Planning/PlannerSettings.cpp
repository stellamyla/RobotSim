#include "PlannerSettings.h"
using namespace Meshing;
using namespace Geometry;

inline Real Radius(const TriMesh& mesh)
{
  Real r2=0;
  for(size_t i=0;i<mesh.verts.size();i++) {
    r2 = Max(r2,mesh.verts[i].normSquared());
  }
  return Sqrt(r2);
}

WorldPlannerSettings::WorldPlannerSettings()
{}

void WorldPlannerSettings::InitializeDefault(RobotWorld& world)
{
  int n=world.NumIDs();
  collisionEnabled.resize(n,n,true);
  for(int i=0;i<n;i++)
    collisionEnabled(i,i) = false;
  for(size_t i=0;i<world.robots.size();i++) {
    int k=world.RobotID(i);
    collisionEnabled(k,k) = true; //can self collide
    int baseid=world.RobotLinkID(i,0);
    Robot* robot = world.robots[i].robot;
    for(size_t j=0;j<robot->links.size();j++) {  //turn off link-to-robot collision
      collisionEnabled(baseid+j,k) = false;
      collisionEnabled(k,baseid+j) = false;
    }
    for(size_t j=0;j<robot->links.size();j++) 
      for(size_t k=0;k<robot->links.size();k++)
	collisionEnabled(baseid+j,baseid+k) = (robot->selfCollisions(j,k)!=NULL);
    //links that are fixed to the terrain
    for(size_t j=0;j<robot->links.size();j++)
      if(robot->parents[j] == -1) {
	for(size_t k=0;k<world.terrains.size();k++)
	  collisionEnabled(baseid+j,world.TerrainID(k)) = false;
      }
  }

  AABB3D bounds;
  bounds.minimize();
  for(size_t i=0;i<world.rigidObjects.size();i++) {
    AABB3D b,bw;
    world.rigidObjects[i].object->mesh.GetAABB(b.bmin,b.bmax);
    bw.setTransform(b,world.rigidObjects[i].object->T);
    bounds.setUnion(bw);
  }
  for(size_t i=0;i<world.terrains.size();i++) {
    AABB3D b;
    world.terrains[i].terrain->mesh.GetAABB(b.bmin,b.bmax);
    bounds.setUnion(b);
  }
  for(size_t i=0;i<world.robots.size();i++) {
    AABB3D b,bw;
    for(size_t j=0;j<world.robots[i].robot->links.size();j++) {
      world.robots[i].robot->geometry[j].GetAABB(b.bmin,b.bmax);
      bw.setTransform(b,world.robots[i].robot->links[j].T_World);
      bounds.setUnion(bw);
    }
  }

  objectSettings.resize(world.rigidObjects.size());
  for(size_t i=0;i<objectSettings.size();i++) {
    objectSettings[i].worldBounds = bounds;
    objectSettings[i].touchable = true;
    objectSettings[i].collisionMargin = 0.005;
    objectSettings[i].collisionEpsilon = 0.001;
    objectSettings[i].translationWeight = 1.0;
    objectSettings[i].rotationWeight = Radius(world.rigidObjects[i].object->mesh);
  }

  robotSettings.resize(world.robots.size());
  for(size_t i=0;i<robotSettings.size();i++) {
    robotSettings[i].worldBounds = bounds;
    robotSettings[i].collisionEpsilon = 0.005;
    robotSettings[i].collisionMargin.resize(world.robots[i].robot->links.size(),0.002);
    robotSettings[i].distanceWeights.clear();
    robotSettings[i].worldBounds = bounds;
    robotSettings[i].contactEpsilon = 0.001;
    robotSettings[i].contactIKMaxIters = 50;
  }
}

bool CheckCollision(CollisionMesh& m1,CollisionMesh& m2,Real tol)
{
  Assert(tol >= 0);
  CollisionMeshQuery q(m1,m2);
  if(tol == 0) {
    return q.Collide();
  }
  else {
    return q.WithinDistance(tol);
  }
}

Real DistanceLowerBound(CollisionMesh& m1,CollisionMesh& m2,Real epsilon,Real bound=Inf)
{
  Assert(epsilon >= 0);
  CollisionMeshQuery q(m1,m2);
  return q.Distance_Coherent(0.0,epsilon,bound);
}

bool WorldPlannerSettings::CheckCollision(RobotWorld& world,int id1,int id2,Real tol)
{
  if(id2 < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      if(CheckCollision(world,id1,i,tol)) return true;
    }
    return false;
  }
  else {
    if(!collisionEnabled(id1,id2)) return false;
    int index1,index2;
    index1 = world.IsRobot(id1);
    index2 = world.IsRobot(id2);
    //check for robot collision
    if(index1 >= 0) {
      Robot* robot = world.robots[index1].robot;
      if(index2 >= 0) {
	Robot* robot2 = world.robots[index2].robot;
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      Real newtol = tol+robotSettings[index1].collisionMargin[j]+robotSettings[index2].collisionMargin[k];
	      if(::CheckCollision(robot->geometry[j],robot2->geometry[k],newtol)) return true;
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    Real newtol=tol+robotSettings[index1].collisionMargin[j];
	    if(CheckCollision(world,robot->geometry[j],id2,newtol)) return true;
	  }
      }
      return false;
    }
    else if(index2 >= 0) {
      Robot* robot2 = world.robots[index2].robot;
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j)))
	  if(CheckCollision(world,robot2->geometry[j],id1,tol+robotSettings[index2].collisionMargin[j])) return true;
    }
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      return CheckCollision(world,world.terrains[index1].terrain->mesh,id2,tol);
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObject* obj = world.rigidObjects[index1].object;
      obj->mesh.UpdateTransform(obj->T);
      Real newtol=tol+objectSettings[index1].collisionMargin;
      return CheckCollision(world,obj->mesh,id2,newtol);
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      Real newtol=tol+robotSettings[linkid.first].collisionMargin[linkid.second];
      return CheckCollision(world,robot->geometry[linkid.second],id2,newtol);
    }
    return false;
  }
}

bool WorldPlannerSettings::CheckCollision(RobotWorld& world,CollisionMesh& mesh,int id,Real tol)
{
  if(id < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      if(CheckCollision(world,mesh,i,tol)) return true;
    }
    return false;
  }
  else {
    int index;
    index = world.IsTerrain(id);
    if(index >= 0) {
      return ::CheckCollision(mesh,world.terrains[index].terrain->mesh,tol);
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObject* obj = world.rigidObjects[index].object;
      obj->mesh.UpdateTransform(obj->T);
      return ::CheckCollision(mesh,obj->mesh,tol+objectSettings[index].collisionMargin);
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      Robot* robot = world.robots[index].robot;
      for(size_t j=0;j<robot->links.size();j++)
	if(::CheckCollision(mesh,robot->geometry[j],tol+robotSettings[index].collisionMargin[j])) return true;
      return false;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      return ::CheckCollision(mesh,robot->geometry[linkid.second],tol+robotSettings[linkid.first].collisionMargin[linkid.second]);
    }
    return false;
  }
}

Real WorldPlannerSettings::DistanceLowerBound(RobotWorld& world,int id1,int id2,Real eps,Real bound)
{
  Real minDist = bound;
  if(id2 < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      minDist = Min(minDist,DistanceLowerBound(world,id1,i,eps,minDist));
    }
    return minDist;
  }
  else {
    if(!collisionEnabled(id1,id2)) return Inf;
    int index1,index2;
    index1 = world.IsRobot(id1);
    index2 = world.IsRobot(id2);
    //check for robot collision
    if(index1 >= 0) {
      Robot* robot = world.robots[index1].robot;
      if(index2 >= 0) {
	Robot* robot2 = world.robots[index2].robot;
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      Real adj=robotSettings[index1].collisionMargin[j]-robotSettings[index2].collisionMargin[k];
	      minDist = Min(minDist,::DistanceLowerBound(robot->geometry[j],robot2->geometry[k],eps,minDist+adj)-adj);
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    Real adj=robotSettings[index1].collisionMargin[j];
	    minDist = Min(minDist,DistanceLowerBound(world,robot->geometry[j],id2,eps,minDist+adj)-adj);
	  }
      }
      return minDist;
    }
    else if(index2 >= 0) {
      Robot* robot2 = world.robots[index2].robot;
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j))) {
	  Real adj=robotSettings[index2].collisionMargin[j];
	  minDist = Min(minDist,DistanceLowerBound(world,robot2->geometry[j],id1,eps,minDist+adj)-adj);
	}
    }
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      return DistanceLowerBound(world,world.terrains[index1].terrain->mesh,id2,eps,minDist);
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObject* obj = world.rigidObjects[index1].object;
      obj->mesh.UpdateTransform(obj->T);
      Real adj=objectSettings[index1].collisionMargin;
      return DistanceLowerBound(world,obj->mesh,id2,eps,minDist+adj)-adj;
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      assert(linkid.first < (int)world.robots.size());
      Robot* robot = world.robots[linkid.first].robot;
      assert(linkid.second >= 0 && linkid.second < (int)robot->links.size());
      Real adj=robotSettings[linkid.first].collisionMargin[linkid.second];
      Real d=DistanceLowerBound(world,robot->geometry[linkid.second],id2,eps,minDist+adj)-adj;
      //printf("Link %d on robot %d to object %d has distance %g\n",linkid.second,linkid.first,id2,d);
      //printf("Object %s\n",world.GetName(id2).c_str());
      if(d<=0) {
	Assert(CheckCollision(world,robot->geometry[linkid.second],id2,0));
	//printf("It collides!\n");
      }
      return d;
    }
    return Inf;
  }
}

Real WorldPlannerSettings::DistanceLowerBound(RobotWorld& world,CollisionMesh& mesh,int id,Real eps,Real bound)
{
  Real minDist = bound;
  if(id < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      minDist = Min(minDist,DistanceLowerBound(world,mesh,i,eps,minDist));
    }
    return minDist;
  }
  else {
    int index;
    index = world.IsTerrain(id);
    if(index >= 0) {
      return ::DistanceLowerBound(mesh,world.terrains[index].terrain->mesh,eps,minDist);
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObject* obj = world.rigidObjects[index].object;
      obj->mesh.UpdateTransform(obj->T);
      Real adj=objectSettings[index].collisionMargin;
      return ::DistanceLowerBound(mesh,obj->mesh,eps,minDist+adj)-adj;
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      Robot* robot = world.robots[index].robot;
      for(size_t j=0;j<robot->links.size();j++) {
	Real adj = robotSettings[index].collisionMargin[j];
	minDist = Min(minDist,::DistanceLowerBound(mesh,robot->geometry[j],eps,minDist+adj)-adj);
      }
      return minDist;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      assert(linkid.first < (int)world.robots.size());
      Robot* robot = world.robots[linkid.first].robot;
      assert(linkid.second >= 0 && linkid.second < (int)robot->links.size());
      Real adj=robotSettings[linkid.first].collisionMargin[linkid.second];
      return ::DistanceLowerBound(mesh,robot->geometry[linkid.second],eps,minDist+adj)-adj;
    }
    return false;
  }
}

void WorldPlannerSettings::EnumerateCollisionPairs(RobotWorld& world,vector<pair<int,int> >& pairs) const
{
  pairs.resize(0);
  int n=world.NumIDs();
  for(int i=0;i<n;i++) {
    if(world.IsRobot(i)>=0) continue;
    for(int j=0;j<i;j++) {
      if(world.IsRobot(j)>=0) continue;
      if(collisionEnabled(i,j)) {
	pairs.push_back(pair<int,int>(i,j));
      }
    }
  }
}

void WorldPlannerSettings::EnumerateCollisionQueries(RobotWorld& world,int id1,int id2,vector<pair<int,int> >& indices,vector<CollisionMeshQueryEnhanced>& queries)
{
  if(id2 < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      EnumerateCollisionQueries(world,id1,i,indices,queries);
    }
  }
  else {
    if(!collisionEnabled(id1,id2)) return;
    int index1,index2;
    index1 = world.IsRobot(id1);
    index2 = world.IsRobot(id2);
    //check for robot collision
    if(index1 >= 0) {
      Robot* robot = world.robots[index1].robot;
      if(index2 >= 0) {
	Robot* robot2 = world.robots[index2].robot;
	for(size_t j=0;j<robot->links.size();j++)
	  for(size_t k=0;k<robot2->links.size();k++)
	    if(collisionEnabled(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k))) {
	      indices.push_back(pair<int,int>(world.RobotLinkID(index1,j),world.RobotLinkID(index2,k)));
	      queries.push_back(CollisionMeshQueryEnhanced(robot->geometry[j],robot2->geometry[k]));
	      queries.back().margin1=robotSettings[index1].collisionMargin[j];
	      queries.back().margin2=robotSettings[index2].collisionMargin[k];
	    }
      }
      else {
	for(size_t j=0;j<robot->links.size();j++)
	  if(collisionEnabled(world.RobotLinkID(index1,j),id2)) {
	    vector<int> ids;
	    size_t qsize=queries.size();
	    EnumerateCollisionQueries(world,robot->geometry[j],id2,ids,queries);
	    indices.push_back(pair<int,int>(world.RobotLinkID(index1,j),id2));
	    queries.back().margin1=robotSettings[index1].collisionMargin[j];
	  }
      }
      return;
    }
    else if(index2 >= 0) {
      Robot* robot2 = world.robots[index2].robot;
      for(size_t j=0;j<robot2->links.size();j++)
	if(collisionEnabled(id1,world.RobotLinkID(index2,j))) {
	  vector<int> ids;
	  EnumerateCollisionQueries(world,robot2->geometry[j],id1,ids,queries);
	  indices.push_back(pair<int,int>(world.RobotLinkID(index2,j),id1));
	  queries.back().margin1=robotSettings[index2].collisionMargin[j];
	}
      return;
    }
    vector<int> ids;
    //non-robots
    index1 = world.IsTerrain(id1);
    if(index1 >= 0) {
      EnumerateCollisionQueries(world,world.terrains[index1].terrain->mesh,id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
      return;
    }
    index1 = world.IsRigidObject(id1);
    if(index1 >= 0) {
      RigidObject* obj = world.rigidObjects[index1].object;
      obj->mesh.UpdateTransform(obj->T);
      size_t qsize = queries.size();
      EnumerateCollisionQueries(world,obj->mesh,id2,ids,queries);
      for(size_t i=qsize;i<queries.size();i++)
	queries[i].margin1=objectSettings[index1].collisionMargin;
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
      return;
    }
    pair<int,int> linkid = world.IsRobotLink(id1);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      size_t qsize = queries.size();
      EnumerateCollisionQueries(world,robot->geometry[linkid.second],id2,ids,queries);
      for(size_t i=0;i<ids.size();i++)
	indices.push_back(pair<int,int>(id1,ids[i]));
      for(size_t i=qsize;i<queries.size();i++)
	queries.back().margin1=robotSettings[linkid.first].collisionMargin[linkid.second];
    }
    return;
  }
}

void WorldPlannerSettings::EnumerateCollisionQueries(RobotWorld& world,CollisionMesh& mesh,int id,vector<int>& collisionIds,vector<CollisionMeshQueryEnhanced>& queries)
{
  if(id < 0) {  //check all
    for(int i=0;i<collisionEnabled.n;i++) {
      EnumerateCollisionQueries(world,mesh,i,collisionIds,queries);
    }
    return;
  }
  else {
    int index;
    index = world.IsTerrain(id);
    if(index >= 0) {
      queries.push_back(CollisionMeshQueryEnhanced(mesh,world.terrains[index].terrain->mesh));
      collisionIds.push_back(id);
      return;
    }
    index = world.IsRigidObject(id);
    if(index >= 0) {
      RigidObject* obj = world.rigidObjects[index].object;
      obj->mesh.UpdateTransform(obj->T);
      queries.push_back(CollisionMeshQueryEnhanced(mesh,obj->mesh));
      queries.back().margin1=objectSettings[index].collisionMargin;
      collisionIds.push_back(id);
      return;
    }
    index = world.IsRobot(id);
    if(index >= 0) {
      Robot* robot = world.robots[index].robot;
      for(size_t j=0;j<robot->links.size();j++) {
	queries.push_back(CollisionMeshQueryEnhanced(mesh,robot->geometry[j]));
	queries.back().margin2=robotSettings[index].collisionMargin[j];
	collisionIds.push_back(world.RobotLinkID(index,j));
      }
      return;
    }
    pair<int,int> linkid = world.IsRobotLink(id);
    if(linkid.first >= 0) {
      Robot* robot = world.robots[linkid.first].robot;
      queries.push_back(CollisionMeshQueryEnhanced(mesh,robot->geometry[linkid.second]));
      queries.back().margin2=robotSettings[linkid.first].collisionMargin[linkid.second];
      collisionIds.push_back(id);
    }
    return;
  }
}
