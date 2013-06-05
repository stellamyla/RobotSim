#include "ViewRigidObject.h"
#include <errors.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
#include <GLdraw/GLColor.h>
#include <GLdraw/GLError.h>
using namespace Math;
using namespace Meshing;

ViewRigidObject::ViewRigidObject()
{
  faceColor.set(0.4,0.2,0.8);
  edgeColor.set(0,0,0);
  lighting = true;
  edges = false;
  obj = NULL;
}
  
void ViewRigidObject::Draw()
{
  if(!obj) return;
  glDisable(GL_CULL_FACE);
  glPushMatrix();
  glMultMatrix(Matrix4(obj->T));

  if(faceColor.rgba[3] != 1.0) {
    glEnable(GL_BLEND); 
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
  }
  if(lighting) {
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,faceColor.rgba);
  }
  else {
    glDisable(GL_LIGHTING);
    glColor4fv(faceColor.rgba);
  }
  if(edges) {
    glPolygonOffset(1.0,1.0);
    glEnable(GL_POLYGON_OFFSET_FILL);
  }
  
  //draw the mesh
  DrawGLTris(obj->mesh);

  if(edges) {
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_LIGHTING);
    glDepthFunc(GL_LEQUAL);
    glColor4fv(edgeColor.rgba);
    glBegin(GL_LINES);
    if(obj->mesh.triNeighbors.empty())
      obj->mesh.CalcTriNeighbors();
    Vector3 ni;
    const static Real creaseAngle = Pi/6.0;
    Real cosCreaseAngle = Cos(creaseAngle);
    for(size_t i=0;i<obj->mesh.tris.size();i++) {
      ni = obj->mesh.TriangleNormal(i);
      for(int k=0;k<3;k++) {
	int j=obj->mesh.triNeighbors[i][k];
	if(j<0) continue;
	Vector3 nj=obj->mesh.TriangleNormal(j);
	if(dot(ni,nj) < cosCreaseAngle) {
	  int v1,v2;
	  obj->mesh.tris[i].getCompliment(k,v1,v2);
	  Assert(v1 >= 0 && v1 < (int)obj->mesh.verts.size());
	  Assert(v2 >= 0 && v2 < (int)obj->mesh.verts.size());
	  glVertex3v(obj->mesh.verts[v1]);
	  glVertex3v(obj->mesh.verts[v2]);
	}
      }
    }
    glEnd();
    glDepthFunc(GL_LESS);
  }

  if(faceColor.rgba[3] != 1.0) {
    glDisable(GL_BLEND); 
    glEnable(GL_DEPTH_TEST);
  }

  glPopMatrix();
  glEnable(GL_CULL_FACE);
}
