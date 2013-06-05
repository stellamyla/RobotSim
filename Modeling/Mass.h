#ifndef OBJECT_MASS_H
#define OBJECT_MASS_H

#include <meshing/TriMesh.h>
using namespace Math3D;

///Computes the first moment integrated over the mesh (assuming the mesh
///is a hollow shell)
Vector3 CenterOfMass(const Meshing::TriMesh& mesh);
///Computes the second moment integrated over the mesh (assuming the mesh
///is a hollow shell)
Matrix3 Covariance(const Meshing::TriMesh& mesh,const Vector3& center);
inline Matrix3 Covariance(const Meshing::TriMesh& mesh)
{
  return Covariance(mesh,CenterOfMass(mesh));
}

///Computes the inertia tensor by integrating over the mesh
Matrix3 Inertia(const Meshing::TriMesh& mesh,const Vector3& center,Real mass);
inline Matrix3 Inertia(const Meshing::TriMesh& mesh,Real mass)
{
  return Inertia(mesh,CenterOfMass(mesh),mass);
}

///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Vector3 CenterOfMass_Solid(const Meshing::TriMesh& mesh,Real gridRes);
///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Matrix3 Covariance_Solid(const Meshing::TriMesh& mesh,Real gridRes,const Vector3& center);
inline Matrix3 Covariance_Solid(const Meshing::TriMesh& mesh,Real gridRes)
{
  return Covariance_Solid(mesh,gridRes,CenterOfMass_Solid(mesh,gridRes));
}
///Computes the inertia integrated over the solid inside the mesh
///using a grid approximation
Matrix3 Inertia_Solid(const Meshing::TriMesh& mesh,Real gridRes,const Vector3& center,Real mass);
inline Matrix3 Inertia_Solid(const Meshing::TriMesh& mesh,Real gridRes,Real mass)
{
  return Inertia_Solid(mesh,gridRes,CenterOfMass_Solid(mesh,gridRes),mass);
}

#endif
