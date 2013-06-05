#ifndef ODE_CUSTOM_MESH_H

#include <geometry/CollisionMesh.h>
#include <ode/common.h>
using namespace Geometry;

struct CustomMeshData
{
  CollisionMesh* mesh;
  Real outerMargin;
};

dGeomID dCreateCustomMesh(CollisionMesh* mesh,Real outerMargin=0);
CustomMeshData* dGetCustomMeshData(dGeomID o);
void InitODECustomMesh();

#endif

