#include "ViewEnvironment.h"
#include "ViewTextures.h"
#include <errors.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/drawMesh.h>
#include <GLdraw/GLColor.h>
#include <GLdraw/GLError.h>
#include <math/random.h>
#include <meshing/LSConformalMapping.h>
using namespace Math;
using namespace Meshing;

vector<Vector2> texcoords;

void SetupTextureCoordinates(TriMeshWithTopology& mesh)
{
  TriMeshChart chart(mesh);
  LSConformalMapping mapping(mesh,chart);
  if(mapping.Calculate()) {
    texcoords = chart.coordinates;
  }
  else {
    cerr<<"Error generating texcoords"<<endl;
  }
}

ViewEnvironment::ViewEnvironment()
{
  faceColor.set(0.4,0.3,0.1);
  edgeColor.set(0,0,0);
  lighting = true;
  edges = false;
  texture=NoiseTexture;
  texture = CheckerTexture;
  texCoords = XYTexCoords;
  texDivs = 1;
}
  
void ViewEnvironment::Draw()
{
  if(faceColor.rgba[3] != 1.0) {
    glEnable(GL_BLEND); 
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
  }
  if(lighting) {
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,faceColor.rgba);
  }
  else {
    glDisable(GL_LIGHTING);
    glColor4fv(faceColor.rgba);
  }
  //set up the texture coordinates
  if(texture != NoTexture) {
    ViewTextures::Initialize();

    GLColor coldouble(faceColor.rgba[0]*2,faceColor.rgba[1]*2,faceColor.rgba[2]*2,faceColor.rgba[3]);
    if(lighting) glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,coldouble.rgba);
    else glColor4fv(coldouble.rgba);
    glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);

    switch(texture) {
    case NoiseTexture:
      glEnable(GL_TEXTURE_2D);
      ViewTextures::noise.setCurrentGL();
      break;
    case CheckerTexture:
      glEnable(GL_TEXTURE_2D);
      ViewTextures::checker.setCurrentGL();
      break;
    case GradientTexture:
      glEnable(GL_TEXTURE_1D);
      ViewTextures::grayscaleGradient.setCurrentGL();
      break;
    case ColorGradientTexture:
      glEnable(GL_TEXTURE_1D);
      ViewTextures::rainbowGradient.setCurrentGL();
      {
	GLColor grey(0.5,0.5,0.5);
	if(lighting) glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,grey.rgba);
	else glColor4fv(grey.rgba);
      }
    }

    switch(texCoords) {
    case ParameterizedTexCoord:
      if(texcoords.empty())
	SetupTextureCoordinates(env->mesh);
      break;
    case XYTexCoords:
      {
	//set up texture coordinate generation
	//TODO: the proper dims according to texdivs
	static const float paramx[4]={1,0,0,0};
	static const float paramy[4]={0,1,0,0};
	glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
	glTexGeni(GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
	glTexGenfv(GL_S,GL_OBJECT_PLANE,paramx);
	glTexGenfv(GL_T,GL_OBJECT_PLANE,paramy);
	glEnable(GL_TEXTURE_GEN_S);
	glEnable(GL_TEXTURE_GEN_T);
	Assert(CheckGLErrors() == false);
      }
      break;
    case ZTexCoord:
      {
	Real minz=Inf,maxz=-Inf;
	for(size_t i=0;i<env->mesh.verts.size();i++) {
	  minz = Min(minz,env->mesh.verts[i].z);
	  maxz = Max(maxz,env->mesh.verts[i].z);
	}
	Real scale,offset;
	if(maxz == minz) {
	  scale=1;
	  offset=-minz+1;
	}
	else {
	  //scale = texDivs/(maxz-minz);
	  scale = (texDivs - 2.0/64.0)/(maxz-minz);
	  offset = -(scale+1.0/64.0)*minz;
	}
	const float paramx[4]={0,0,scale,offset};
	glTexGeni(GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
	glTexGenfv(GL_S,GL_OBJECT_PLANE,paramx);
	glEnable(GL_TEXTURE_GEN_S);
	Assert(CheckGLErrors() == false);
      }
    }   
  }
  if(edges) {
    glPolygonOffset(1.0,1.0);
    glEnable(GL_POLYGON_OFFSET_FILL);
  }
  
  //draw the mesh
  if(texcoords.empty()) {
    DrawGLTris(env->mesh);
  }
  else {
    glBegin(GL_TRIANGLES);
    for(size_t i=0;i<env->mesh.tris.size();i++) {
      const IntTriple&t=env->mesh.tris[i];
      glNormal3v(env->mesh.TriangleNormal(i));
      glTexCoord2v(texcoords[t.a]);
      glVertex3v(env->mesh.verts[t.a]);
      glTexCoord2v(texcoords[t.b]);
      glVertex3v(env->mesh.verts[t.b]);
      glTexCoord2v(texcoords[t.c]);
      glVertex3v(env->mesh.verts[t.c]);
    }
    glEnd();
  }

  if(edges) {
    glDisable(GL_POLYGON_OFFSET_FILL);
  }

  //cleanup, reset GL state
  switch(texture) {
  case NoiseTexture:
    glDisable(GL_TEXTURE_2D);
    break;
  case CheckerTexture:
    glDisable(GL_TEXTURE_2D);
    break;
  case GradientTexture:
    glDisable(GL_TEXTURE_1D);
    break;
  case ColorGradientTexture:
    glDisable(GL_TEXTURE_1D);
    break;
  }
  switch(texCoords) {
  case XYTexCoords:
    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);
    break;
  case ZTexCoord:
    glDisable(GL_TEXTURE_GEN_S);
    break;
  }

  if(edges) {
    glDisable(GL_LIGHTING);
    glDepthFunc(GL_LEQUAL);
    glColor4fv(edgeColor.rgba);
    glBegin(GL_LINES);
    if(env->mesh.triNeighbors.empty())
      env->mesh.CalcTriNeighbors();
    Vector3 ni;
    const static Real creaseAngle = Pi/6.0;
    Real cosCreaseAngle = Cos(creaseAngle);
    for(size_t i=0;i<env->mesh.tris.size();i++) {
      ni = env->mesh.TriangleNormal(i);
      for(int k=0;k<3;k++) {
	int j=env->mesh.triNeighbors[i][k];
	if(j<0) continue;
	Vector3 nj=env->mesh.TriangleNormal(j);
	if(dot(ni,nj) < cosCreaseAngle) {
	  int v1,v2;
	  env->mesh.tris[i].getCompliment(k,v1,v2);
	  Assert(v1 >= 0 && v1 < (int)env->mesh.verts.size());
	  Assert(v2 >= 0 && v2 < (int)env->mesh.verts.size());
	  glVertex3v(env->mesh.verts[v1]);
	  glVertex3v(env->mesh.verts[v2]);
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
}
