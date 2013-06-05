#ifndef MESHING_PRIMITIVES_H
#define MESHING_PRIMITIVES_H

#include "TriMesh.h"

/** @file meshing/MeshPrimitives.h
 * @ingroup Meshing
 * @brief Constructors for basic mesh primitives.
 */

namespace Meshing {

  /** @addtogroup Meshing */
  /*@{*/

///makes a unit square on z=0 with m,n divisions in the x,y directions
void MakeTriPlane(int m,int n,TriMesh& mesh);
///makes a square of size [x,y] with m,n divisions in the x,y directions
void MakeTriPlane(int m,int n,Real x,Real y,TriMesh& mesh);

///makes a unit cube with m,n,p divisions in the x,y,z directions
void MakeTriCube(int m,int n,int p,TriMesh& mesh);
///makes a [x,y,z] sized box with m,n,p divisions in the x,y,z directions
void MakeTriBox(int m,int n,int p,Real x,Real y,Real z,TriMesh& mesh);

///makes a unit cube centered at 0 with m,n,p divisions in the x,y,z directions
void MakeTriCenteredCube(int m,int n,int p,TriMesh& mesh);
///makes a [x,y,z] sized cube centered at 0 with m,n,p divisions in the x,y,z directions
void MakeTriCenteredBox(int m,int n,int p,Real x,Real y,Real z,TriMesh& mesh);

///makes a unit sphere with the given stacks and slices (axis in z direction)
void MakeTriSphere(int numStacks,int numSlices,TriMesh& mesh);
///makes a radius r sphere with the given stacks and slices (axis in z direction)
void MakeTriSphere(int numStacks,int numSlices,Real r,TriMesh& mesh);

///makes a unit height cone with unit base radius (base at origin, tip pointing in z direction)
void MakeTriCone(int numSlices,TriMesh& mesh);
///makes a cone with height h, base radius rbase
void MakeTriCone(int numSlices,Real h,Real rbase,TriMesh& mesh);

///makes a unit height cylinder with unit base radius (centered at origin, extending in z direction)
void MakeTriCylinder(int numSlices,TriMesh& mesh);
///makes a cylinder with height h, base radius rbase
void MakeTriCylinder(int numSlices,Real h,Real rbase,TriMesh& mesh);

  /*@}*/
} //namespace Meshing

#endif
