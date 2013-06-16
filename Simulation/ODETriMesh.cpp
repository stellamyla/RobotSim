#include "ODETriMesh.h"
#include "ODECommon.h"
#include "ODECustomMesh.h"
#include <ode/ode.h>
#include <ode/common.h>
#include <GLdraw/GL.h>
#include <errors.h>
using namespace Meshing;

#define USING_CUSTOM_MESH 1
#define USING_GIMPACT 0

ODETriMesh::ODETriMesh()
  :triMeshDataID(0),geomID(0),verts(NULL),indices(NULL),normals(NULL),numVerts(0),numTris(0),collisionMesh(0)
{
  surface.kRestitution = 0;
  surface.kFriction = 0;
  surface.kStiffness = Inf;
  surface.kDamping = Inf;

#if USING_GIMPACT
  numVertComponents = 3;
#else
  numVertComponents = 4;
#endif
}

ODETriMesh::~ODETriMesh()
{
  Clear();
}

void ODETriMesh::Create(const TriMesh& mesh,dSpaceID space,Vector3 offset)
{
  Clear();
  if(!USING_CUSTOM_MESH) {
    Assert(numVertComponents == 3 || numVertComponents == 4);
#if USING_GIMPACT
    //GIMPACT needs this
    Assert(numVertComponents == 3);
#endif
    
    numVerts = (int)mesh.verts.size();
    verts = new dReal[mesh.verts.size()*numVertComponents];
    for(size_t i=0;i<mesh.verts.size();i++) {
      if(numVertComponents == 3)
	CopyVector3(&verts[i*numVertComponents],mesh.verts[i]+offset);
      else {
	CopyVector(&verts[i*numVertComponents],mesh.verts[i]+offset);
	verts[i*numVertComponents+3] = 1.0;
      }
    }
    
    numTris = (int)mesh.tris.size();
    indices = new int[mesh.tris.size()*3];
    normals = new dReal[mesh.tris.size()*3];
    for(size_t i=0;i<mesh.tris.size();i++) {
      indices[i*3] = mesh.tris[i].a;
      indices[i*3+1] = mesh.tris[i].b;
      indices[i*3+2] = mesh.tris[i].c;
      CopyVector3(&normals[i*3],mesh.TriangleNormal(i));
    }
    
    triMeshDataID = dGeomTriMeshDataCreate();
    //for some reason, ODE behaves better when it calculates its own normals
#if defined(dDOUBLE)
#if USING_GIMPACT
#error "GIMPACT doesn't work with doubles"
#endif
    //dGeomTriMeshDataBuildDouble1(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3,normals);
    dGeomTriMeshDataBuildDouble(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3);
#else
    //dGeomTriMeshDataBuildSingle1(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3,normals);
    dGeomTriMeshDataBuildSingle(triMeshDataID,verts,sizeof(dReal)*numVertComponents,numVerts,indices,numTris*3,sizeof(int)*3);
#endif
    geomID = dCreateTriMesh(space, triMeshDataID, 0, 0, 0);

  /* Sanity check!
  for(size_t i=0;i<mesh.tris.size();i++) {
    dVector3 v0,v1,v2;
    dVector3 x0,x1,x2;
    dGeomTriMeshGetTriangle(geomID,i,&v0,&v1,&v2);
    Triangle3D tri;
    mesh.GetTriangle(i,tri);
    tri.a += offset;
    tri.b += offset;
    tri.c += offset;
    CopyVector(x0,tri.a);
    CopyVector(x1,tri.b);
    CopyVector(x2,tri.c);
    printf("triangle %d:\n",i);
    printf("  (%g,%g,%g):\n",x0[0],x0[1],x0[2]);
    printf("  (%g,%g,%g):\n",x1[0],x1[1],x1[2]);
    printf("  (%g,%g,%g):\n",x2[0],x2[1],x2[2]);
    printf("ret:\n");
    printf("  (%g,%g,%g):\n",v0[0],v0[1],v0[2]);
    printf("  (%g,%g,%g):\n",v1[0],v1[1],v1[2]);
    printf("  (%g,%g,%g):\n",v2[0],v2[1],v2[2]);
    for(int k=0;k<3;k++) {
      Assert(FuzzyEquals(x0[k],v0[k]));
      Assert(FuzzyEquals(x2[k],v1[k]));
      Assert(FuzzyEquals(x1[k],v2[k]));
    }
  }
  */
  }
  else {
    //add offsets
    collisionMesh = new CollisionMesh;
    collisionMesh->verts.resize(mesh.verts.size());
    for(size_t i=0;i<mesh.verts.size();i++)
      collisionMesh->verts[i] = mesh.verts[i]+offset;
    collisionMesh->tris = mesh.tris;
    collisionMesh->CalcIncidentTris();
    collisionMesh->CalcTriNeighbors();
    collisionMesh->InitCollisions();
    geomID = dCreateCustomMesh(collisionMesh,0.0);
    dSpaceAdd(space,geomID);
  }
}

void ODETriMesh::Clear()
{
  SafeDeleteProc(geomID,dGeomDestroy);
  SafeDeleteProc(triMeshDataID,dGeomTriMeshDataDestroy);
  SafeArrayDelete(verts);
  SafeArrayDelete(indices);
  SafeArrayDelete(normals);
  numTris = numVerts = 0;
  SafeDelete(collisionMesh);
}

void ODETriMesh::DrawGL()
{
  if(!verts) return;

  glColor3f(1,1,0);
  glPointSize(3.0);
  glBegin(GL_POINTS);
  for(int i=0;i<numVerts;i++) {
#if defined(dDOUBLE)
    glVertex3dv(&verts[i*numVertComponents]);
#else
    glVertex3fv(&verts[i*numVertComponents]);
#endif
  }
  glEnd();
  glPointSize(1.0);

  glColor3f(1,0.5,0);
  glBegin(GL_LINES);
  Real len=0.1;
  for(int i=0;i<numTris;i++) {
    int a=indices[i*3];
    int b=indices[i*3+1];
    int c=indices[i*3+2];
    dReal centroid[3]={0,0,0};
    for(int k=0;k<3;k++)
      centroid[k] = (verts[a*numVertComponents+k]+verts[b*numVertComponents+k]+verts[c*numVertComponents+k])/3.0;
#if defined(dDOUBLE)
    glVertex3dv(centroid);
    glVertex3d(centroid[0]+len*normals[i*3],
	       centroid[1]+len*normals[i*3+1],
	       centroid[2]+len*normals[i*3+2]);
#else
    glVertex3fv(centroid);
    glVertex3f(centroid[0]+len*normals[i*3],
	       centroid[1]+len*normals[i*3+1],
	       centroid[2]+len*normals[i*3+2]);
#endif
  }
  glEnd();
}

void ODETriMesh::SetPadding(Real padding)
{
  dGetCustomMeshData(geom())->outerMargin = padding;
}

Real ODETriMesh::GetPadding()
{
  return dGetCustomMeshData(geom())->outerMargin;
}
