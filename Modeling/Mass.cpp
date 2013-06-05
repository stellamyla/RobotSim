#include "Mass.h"
#include <meshing/VolumeGrid.h>
#include <meshing/Voxelize.h>
#include <errors.h>
using namespace Meshing;

const static Real third = 1.0/3.0;

//m = m+xy^t
inline void AddOuterProduct(Matrix3& m,const Vector3& x,const Vector3& y)
{
  //note that the matrix data variable is column-major, not row major
  m.data[0][0]+=x.x*y.x;
  m.data[0][1]+=x.y*y.x;
  m.data[0][2]+=x.z*y.x;

  m.data[1][0]+=x.x*y.y;
  m.data[1][1]+=x.y*y.y;
  m.data[1][2]+=x.z*y.y;

  m.data[2][0]+=x.x*y.z;
  m.data[2][1]+=x.y*y.z;
  m.data[2][2]+=x.z*y.z;
}


//first and second moments integrated over mesh
Vector3 CenterOfMass(const TriMesh& mesh)
{
  Triangle3D tri;
  Vector3 center(Zero);
  if(mesh.tris.empty()) return center;
  Real sumArea=Zero;
  for(size_t i=0;i<mesh.tris.size();i++) {
    mesh.GetTriangle(i,tri);
    Real area=tri.area();
    center.madd(tri.a+tri.b+tri.c,area*third);
    sumArea += area;
  }
  Assert(sumArea != Zero);
  center /= sumArea;
  return center;
}

Matrix3 Covariance(const TriMesh& mesh,const Vector3& center)
{
  Triangle3D tri;
  Real sumArea=Zero;
  Matrix3 A,temp;
  A.setZero();
  if(mesh.tris.empty()) return A;
  for(size_t i=0;i<mesh.tris.size();i++) {
    mesh.GetTriangle(i,tri);
    tri.a-=center;
    tri.b-=center;
    tri.c-=center;
    temp.setZero();
    AddOuterProduct(temp,tri.a,Two*tri.a+tri.b+tri.c);
    AddOuterProduct(temp,tri.b,Two*tri.b+tri.a+tri.c);
    AddOuterProduct(temp,tri.c,Two*tri.c+tri.a+tri.b);
    Real area = tri.area();
    temp *= area;
    A += temp;
    sumArea += area;
  }
  Assert(sumArea != Zero);
  A *= 1.0/sumArea;
  return A;
}

Matrix3 Inertia(const TriMesh& mesh,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance(mesh,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}


///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Vector3 CenterOfMass_Solid(const Meshing::TriMesh& mesh,Real gridRes)
{
  VolumeGrid grid;
  mesh.GetAABB(grid.bb.bmin,grid.bb.bmax);
  //expand slightly
  grid.bb.bmin -= Vector3(0.5*gridRes);
  grid.bb.bmax += Vector3(0.5*gridRes);
  grid.ResizeByResolution(Vector3(gridRes));
  DensityEstimate_CenterShooting(mesh,grid.value,grid.bb);
  Vector3 mean(Zero);
  Real sum=0;
  for(VolumeGridIterator<Real> i=grid.getIterator();!i.isDone();++i) {
    Vector3 c;
    i.getCellCenter(c);
    sum += *i;
    mean += *i*c;
  }
  if(sum == 0) return mean;
  mean /= sum;
  return mean;
}

///Computes the first moment integrated over the solid inside the mesh
///using a grid approximation
Matrix3 Covariance_Solid(const Meshing::TriMesh& mesh,Real gridRes,const Vector3& center)
{
  VolumeGrid grid;
  mesh.GetAABB(grid.bb.bmin,grid.bb.bmax);
  //expand slightly
  grid.bb.bmin -= Vector3(0.5*gridRes);
  grid.bb.bmax += Vector3(0.5*gridRes);
  grid.ResizeByResolution(Vector3(gridRes));
  DensityEstimate_CenterShooting(mesh,grid.value,grid.bb);
  Matrix3 cov(Zero);
  Real sum=0;
  for(VolumeGridIterator<Real> i=grid.getIterator();!i.isDone();++i) {
    Vector3 c;
    i.getCellCenter(c);
    Matrix3 temp; temp.setZero();
    AddOuterProduct(temp,c-center,c-center);
    temp *= *i;
    cov += temp;
    sum += *i;
  }
  if(sum == 0) return cov;
  cov *= 1.0/sum;
  return cov;
}

Matrix3 Inertia_Solid(const TriMesh& mesh,Real gridRes,const Vector3& center,Real mass)
{
  Matrix3 cov=Covariance_Solid(mesh,gridRes,center);
  Matrix3 H;
  H.setNegative(cov);
  H(0,0) = cov(1,1)+cov(2,2);
  H(1,1) = cov(0,0)+cov(2,2);
  H(2,2) = cov(0,0)+cov(1,1);
  H *= mass;
  return H;
}
