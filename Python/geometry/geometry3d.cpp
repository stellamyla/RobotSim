#include "geometry3d.h"
#include "interpolate.h"
#include <stdlib.h>
#include <cstdio>
using namespace std;

namespace Math3D {




void Box3D::set(const AABB3D& bb)
{
  origin = bb.bmin;
  xbasis.set(1,0,0);
  ybasis.set(0,1,0);
  zbasis.set(0,0,1);
  dims = bb.bmax-bb.bmin;
}

void Box3D::setTransformed(const AABB3D& box,const RigidTransform& T)
{
  T.R.get(xbasis,ybasis,zbasis);
  origin = T.t + T.R*box.bmin;
  dims = box.bmax-box.bmin;
}

void Box3D::setTransformed(const Box3D& box,const RigidTransform& T)
{
  origin = T*box.origin;
  xbasis = T.R*box.xbasis;
  ybasis = T.R*box.ybasis;
  zbasis = T.R*box.zbasis;
  dims = box.dims;
}

bool Box3D::contains(const Point3D& pt) const
{
	Point3D out;
	toLocal(pt,out);
	return 0<=out.x&&out.x<=dims.x &&
		0<=out.y&&out.y<=dims.y &&
		0<=out.z&&out.z<=dims.z;
}

Real Box3D::distance(const Point3D& pt) const
{
  Point3D temp;
  return distance(pt,temp);
}

Real Box3D::distance(const Point3D& pt,Point3D& out) const
{
  return sqrt(distanceSquared(pt,out));
}

Real Box3D::distanceSquared(const Point3D& pt,Point3D& out) const
{
  Point3D loc;
  toLocal(pt, loc);
  if(loc.x < 0) loc.x = 0;
  if(loc.y < 0) loc.y = 0;
  if(loc.z < 0) loc.z = 0;
  if(loc.x > dims.x) loc.x = dims.x;
  if(loc.y > dims.y) loc.y = dims.y;
  if(loc.z > dims.z) loc.z = dims.z;
  Real norm2 = loc.normSquared();
  fromLocal(loc,out);
  return norm2;
}

bool Box3D::intersects(const Box3D& b) const
{
  cout<<"Not quite done... check split planes a's faces, b's faces, and a's edges x b's edges"<<endl;
  abort();
  return false;
}

bool Box3D::intersectsApprox(const Box3D& b) const
{
	Box3D temp;
	AABB3D aabb_temp, aabb_temp2;
	//make temp localized
	temp.dims = b.dims;
	toLocal(b.origin, temp.origin);
	toLocalReorient(b.xbasis, temp.xbasis);
	toLocalReorient(b.ybasis, temp.ybasis);
	toLocalReorient(b.zbasis, temp.zbasis);
	temp.getAABB(aabb_temp);
	aabb_temp2.bmin.setZero();
	aabb_temp2.bmax = dims;
	if(!aabb_temp2.intersects(aabb_temp))
		return false;

	temp.dims = dims;
	b.toLocal(origin, temp.origin);
	b.toLocalReorient(xbasis, temp.xbasis);
	b.toLocalReorient(ybasis, temp.ybasis);
	b.toLocalReorient(zbasis, temp.zbasis);
	temp.getAABB(aabb_temp);
	aabb_temp2.bmax = b.dims;
	if(!aabb_temp2.intersects(aabb_temp))
		return false;
	return true;
}

void Box3D::getAABB(AABB3D& bb) const
{
	Vector3 x(dims.x*xbasis),y(dims.y*ybasis),z(dims.z*zbasis);
	Vector3 c=origin + 0.5*(x+y+z);

	Vector3 d;
	d.setZero();
	for(int i=0; i<3; i++)
	{
	  d[i] = Abs(x[i]) + Abs(y[i]) + Abs(z[i]);
	}

	bb.bmin.sub(c,d*0.5);
	bb.bmax.add(c,d*0.5);
}

bool Box3D::intersects(const Segment3D& s) const
{
  Segment3D sloc;
  toLocal(s,sloc);
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return sloc.intersects(bbloc);
}

bool Box3D::intersects(const Line3D& l) const
{
  Line3D lloc;
  toLocal(l,lloc);
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return lloc.lineIntersects(bbloc);
}

bool Box3D::intersects(const Triangle3D& t) const
{
  Triangle3D tloc;
  toLocal(t.a,tloc.a);
  toLocal(t.b,tloc.b);
  toLocal(t.c,tloc.c);
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return tloc.intersects(bbloc);
}

bool Box3D::intersects(const Sphere3D& s) const
{
  Sphere3D sloc;
  toLocal(s.center,sloc.center);
  sloc.radius = s.radius;
  AABB3D bbloc;
  bbloc.bmin.setZero();
  bbloc.bmax=dims;
  return sloc.intersects(bbloc);
}


Real Sphere3D::distance(const Point3D& v) const
{
	return (center-v).norm() - radius;
}

bool Sphere3D::contains(const Point3D& v) const
{
	return DistanceLEQ(center,v,radius);
}

bool Sphere3D::contains(const Sphere3D& s) const
{
	return DistanceLEQ(center,s.center,radius-s.radius);
}

bool Sphere3D::withinDistance(const Point3D& v, Real dist) const
{
	return DistanceLEQ(center,v, radius + dist);
}

bool Sphere3D::boundaryWithinDistance(const Point3D& v, Real dist) const
{
    return fabs((center-v).norm()-radius) <= dist;
}

bool Sphere3D::intersects(const Line3D& l, Real* t1, Real* t2) const
{
	Vector3 offset=center-l.source;
	Real o_o=dot(offset,offset), o_b=dot(offset,l.direction), b_b=dot(l.direction,l.direction);
	//so we know there's a root to |offset-t*b|==r
	//o.o-2t*b.o+t^2*b.b=r^2
	Real a,b,c;
	a=b_b;
	b=-Two*o_b;
	c=o_o-radius*radius;
	if(b_b == Zero) {
	  if(c < Zero) {
	    if(t1 && t2) { *t1=-Inf; *t2=Inf; }
	    return true;
	  }
	}
	Real x1,x2;
	int res=quadratic(a,b,c,x1,x2);
	if(res<=0) return false;
	if(res==1) {
	  cout<<"Whoa, line just intersects at one point on the sphere"<<endl;
	  cout<<"l= "<<l.source<<"->"<<l.direction<<endl;
	  cout<<"c= "<<center<<", r="<<radius<<endl;
	  cout<<"t="<<x1<<endl;
	  getchar();
	  x2=x1;
	}
	if(x1 > x2) Swap(x1,x2);
	if(t1 && t2) {
		*t1=x1;
		*t2=x2;
	}
	return true;
}

bool Sphere3D::intersects(const Plane3D& p) const
{
  return fabs(p.distance(center)) <= radius;
}

bool Sphere3D::intersects(const Sphere3D& s) const
{
  return ballsIntersect(center,radius,s.center,s.radius);
}

bool Sphere3D::boundaryIntersects(const Sphere3D& s) const
{
  return ballSphereIntersect(s.center,s.radius,center,radius);
}

bool Sphere3D::boundaryIntersectsBoundary(const Sphere3D& s) const
{
  return spheresIntersect(center,radius,s.center,s.radius);
}

bool Sphere3D::ballsIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb)
{
	return DistanceLEQ(ca,cb,ra+rb);
}

bool Sphere3D::ballSphereIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb)
{
  Real r2 = (ca-cb).normSquared();
  if(r2 <= Sqr(ra+rb)) {
    Real r = sqrt(r2);
    if(r + ra < rb) return false;
    return true;
  }
  return false;
}

bool Sphere3D::spheresIntersect(const Point3D& ca,Real ra,const Point3D& cb,Real rb)
{
  Real r2 = (ca-cb).normSquared();
  if(r2 <= Sqr(ra+rb)) {
    Real r = sqrt(r2);
    if(r + ra < rb) return false;
    if(r + rb < ra) return false;
    return true;
  }
  return false;
}

void Sphere3D::getAABB(AABB3D& bb) const
{
  bb.setPoint(center);
  bb.bmin.x-=radius; bb.bmin.y-=radius; bb.bmin.z-=radius;
  bb.bmax.x+=radius; bb.bmax.y+=radius; bb.bmax.z+=radius;
}

bool Sphere3D::intersects(const AABB3D& bb) const
{
  Vector3 temp;
  return bb.distanceSquared(center,temp) < Sqr(radius);
}



bool Ellipsoid3D::contains(const Point3D& pt) const
{
	Point3D out;
	toLocalNormalized(pt,out);
	return NormLEQ(out,One);
}

bool Ellipsoid3D::intersects(const Line3D& l, Real* t1, Real* t2) const
{
	Line3D llocal;
	toLocalNormalized(l,llocal);
	Sphere3D s;
	s.center.setZero();
	s.radius = One;
	return s.intersects(llocal,t1,t2);
}

void Ellipsoid3D::getAABB(AABB3D& bb) const
{
	//get the bases of world space in ellipsoid space
	Vector3 xb,yb,zb;
	xb.x = xbasis.x;
	xb.y = ybasis.x;
	xb.z = zbasis.x;
	yb.x = xbasis.y;
	yb.y = ybasis.y;
	yb.z = zbasis.y;
	zb.x = xbasis.z;
	zb.y = ybasis.z;
	zb.z = zbasis.z;

	normalize(xb,xb);
	normalize(yb,yb);
	normalize(zb,zb);

	//now find the points on the sphere with the correct tangent planes
	Vector3 xt,yt,zt;
	xt = cross(yb,zb);
	yt = cross(zb,xb);
	zt = cross(xb,yb);

	//these are the normals, just normalize them
	xt.inplaceNormalize();
	yt.inplaceNormalize();
	zt.inplaceNormalize();

	xb = xbasis * dims.x;
	yb = ybasis * dims.y;
	zb = zbasis * dims.z;

	//aliases
	Vector3& bmin=bb.bmin, &bmax = bb.bmax;

	//take these points back to world coordinates- these will be the min and max points
	bmax.x = bmin.x = xt.x * xb.x + xt.y * yb.x + xt.z * zb.x;
	bmax.y = bmin.y = yt.x * xb.y + yt.y * yb.y + yt.z * zb.y;
	bmax.z = bmin.z = zt.x * xb.z + zt.y * yb.z + zt.z * zb.z;

	if(bmax.x < 0)
		bmax.x = -bmax.x;
	else
		bmin.x = -bmin.x;

	if(bmax.y < 0)
		bmax.y = -bmax.y;
	else
		bmin.y = -bmin.y;

	if(bmax.z < 0)
		bmax.z = -bmax.z;
	else
		bmin.z = -bmin.z;

	bmax += origin;
	bmin += origin;
}



} // namespace Math3D
