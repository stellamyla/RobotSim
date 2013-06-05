#include "AABB.h"

namespace Math {

void AABBLineSearch(const Vector& x0,const Vector& dx,const Vector& bmin,const Vector& bmax,Real& t)
{
  assert(x0.n == dx.n);
  assert(x0.n == bmin.n);
  assert(x0.n == bmax.n);
  for(int i=0;i<bmax.n;i++) {
    assert(x0(i) <= bmax(i));
    assert(x0(i) >= bmin(i));
    if(x0(i) + t*dx(i) > bmax(i)) {
      assert(dx(i) > Zero);
      t = (bmax(i)-x0(i))/(dx(i)+Epsilon);
    }
    if(x0(i) + t*dx(i) < bmin(i)) {
      assert(dx(i) < Zero);
      t = (bmin(i)-x0(i))/(dx(i)-Epsilon);
    }
    assert(x0(i)+t*dx(i) <= bmax(i));
    assert(x0(i)+t*dx(i) >= bmin(i));
  }
}

inline bool ClipLine1D(Real q, Real p, Real& umin, Real& umax)
{
   Real r;
   if(p<0) {			//entering
     r=-q/p;
     if(r > umax) return false;
     if(r > umin) umin = r;
   }
   else if(p>0) {
     r=-q/p;
     if(r < umin) return false;
     if(r < umax) umax = r;
   }
   else {
     if(q>0) return false;
   }
   return true;
}

bool AABBClipLine(const Vector& x0,const Vector& dx,
		  const Vector& bmin,const Vector& bmax,
		  Real& u0,Real& u1)
{
  assert(x0.n == dx.n);
  assert(x0.n == bmin.n);
  assert(x0.n == bmax.n);
  for(int i=0;i<x0.n;i++) {
    //for each face, p is dot(dx, normal), q is signed dist to plane (dot(v,normal)-offset)
    if(!ClipLine1D(bmin(i) - x0(i), -dx(i), u0,u1)) return false;
    if(!ClipLine1D(x0(i) - bmax(i), dx(i), u0,u1)) return false;
  }
  return true;
}

} //namespace Math
