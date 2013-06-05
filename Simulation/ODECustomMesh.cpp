#include "ODECustomMesh.h"
#include "ODECommon.h"
#include <ode/collision.h>
#include <Timer.h>
#include <errors.h>
using namespace std;

//if a normal has this length then it is ignored
const static Real gZeroNormalTolerance = 1e-4;

//if two contact points are closer than this threshold, will try to look
//at the local geometry to derive a contact normal
//const static Real gNormalFromGeometryTolerance = 1e-5;
const static Real gNormalFromGeometryTolerance = 1e-2;

//if a barycentric coordinate is within this tolerance of zero, it will be
//considered a zero
const static Real gBarycentricCoordZeroTolerance = 1e-3;

//if true, takes the ODE tolerance points and performs additional contact
//checking -- useful for flat contacts
const static bool gDoTriangleTriangleCollisionDetection = false;

//doesn't consider unique contact points if they are between this tolerance
const static Real cptol=1e-5;

int gdCustomMeshClass = 0;

dGeomID dCreateCustomMesh(CollisionMesh* mesh,Real outerMargin)
{
  dGeomID geom = dCreateGeom(gdCustomMeshClass);
  CustomMeshData* data = dGetCustomMeshData(geom);
  data->mesh = mesh;
  data->outerMargin = outerMargin;
  dGeomSetCategoryBits(geom,0xffffffff);
  dGeomSetCollideBits(geom,0xffffffff);
  dGeomEnable(geom);
  return geom;
}

CustomMeshData* dGetCustomMeshData(dGeomID o)
{
  return (CustomMeshData*)dGeomGetClassData(o);
}

//1 = pt, 2 = edge, 3 = face
inline int FeatureType(const Vector3& b) 
{
  int type=0;
  if(FuzzyZero(b.x,gBarycentricCoordZeroTolerance)) type++;
  if(FuzzyZero(b.y,gBarycentricCoordZeroTolerance)) type++;
  if(FuzzyZero(b.z,gBarycentricCoordZeroTolerance)) type++;
  return 3-type;
}

int EdgeIndex(const Vector3& b)
{
  if(FuzzyZero(b.x,gBarycentricCoordZeroTolerance)) return 0;
  if(FuzzyZero(b.y,gBarycentricCoordZeroTolerance)) return 1;
  if(FuzzyZero(b.z,gBarycentricCoordZeroTolerance)) return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

int VertexIndex(const Vector3& b)
{
  if(FuzzyEquals(b.x,One,gBarycentricCoordZeroTolerance)) return 0;
  if(FuzzyEquals(b.y,One,gBarycentricCoordZeroTolerance)) return 1;
  if(FuzzyEquals(b.z,One,gBarycentricCoordZeroTolerance)) return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

Vector3 VertexNormal(const CollisionMesh& m,int tri,int vnum)
{
  Assert(!m.incidentTris.empty());
  int v=m.tris[tri][vnum];
  Vector3 n(Zero);
  for(size_t i=0;i<m.incidentTris[v].size();i++)
    n += m.TriangleNormal(m.incidentTris[v][i]);
  n.inplaceNormalize();
  return m.currentTransform.R*n;
}

Vector3 EdgeNormal(const CollisionMesh& m,int tri,int e)
{
  Assert(!m.triNeighbors.empty());
  Vector3 n=m.TriangleNormal(tri);
  if(m.triNeighbors[tri][e] != -1) {
    n += m.TriangleNormal(m.triNeighbors[tri][e]);
    n.inplaceNormalize();
  }
  return m.currentTransform.R*n;
}

//returns the normal needed for m1 to get out of m2
Vector3 ContactNormal(const CollisionMesh& m1,const CollisionMesh& m2,const Vector3& p1,const Vector3& p2,int t1,int t2)
{
  Triangle3D tri1,tri2;
  m1.GetTriangle(t1,tri1);
  m2.GetTriangle(t2,tri2);
  Vector3 b1=tri1.barycentricCoords(p1);
  Vector3 b2=tri2.barycentricCoords(p2);
  int type1=FeatureType(b1),type2=FeatureType(b2);
  switch(type1) {
  case 1:  //pt
    switch(type2) {
    case 1:  //pt
      //get the triangle normals
      {
	//printf("ODECustomMesh: Point-point contact\n");
	Vector3 n1 = VertexNormal(m1,t1,VertexIndex(b1));
	Vector3 n2 = VertexNormal(m2,t2,VertexIndex(b2));
	n2 -= n1;
	n2.inplaceNormalize();
	return n2;
      }
      break;
    case 2:  //edge
      {
	//printf("ODECustomMesh: Point-edge contact\n");
	Vector3 n1 = VertexNormal(m1,t1,VertexIndex(b1));
	int e = EdgeIndex(b2);
	Segment3D s = tri2.edge(e);
	Vector3 ev = m2.currentTransform.R*(s.b-s.a);
	Vector3 n2 = EdgeNormal(m2,t2,e);
	n2-=(n1-ev*ev.dot(n1)/ev.dot(ev)); //project onto normal
	n2.inplaceNormalize();
	return n2;
      }
      break;
    case 3:  //face
      return m2.currentTransform.R*tri2.normal();
    }
    break;
  case 2:  //edge
    switch(type2) {
    case 1:  //pt
      {
	//printf("ODECustomMesh: Edge-point contact\n");
	Vector3 n2 = VertexNormal(m2,t2,VertexIndex(b2));
	int e = EdgeIndex(b1);
	Segment3D s = tri1.edge(e);
	Vector3 ev = m1.currentTransform.R*(s.b-s.a);
	Vector3 n1 = EdgeNormal(m1,t1,e);
	n2 = (n2-ev*ev.dot(n2)/ev.dot(ev))-n1; //project onto normal
	n2.inplaceNormalize();
	return n2;
      }
      break;
    case 2:  //edge
      {
	//printf("ODECustomMesh: Edge-edge contact\n");
	int e = EdgeIndex(b1);
	Segment3D s1 = tri1.edge(e);
	Vector3 ev1 = m1.currentTransform.R*(s1.b-s1.a);
	ev1.inplaceNormalize();
	e = EdgeIndex(b2);
	Segment3D s2 = tri2.edge(e);
	Vector3 ev2 = m2.currentTransform.R*(s2.b-s2.a);
	ev2.inplaceNormalize();
	Vector3 n; 
	n.setCross(ev1,ev2);
	Real len = n.length();
	if(len < gZeroNormalTolerance) {
	  //hmm... edges are parallel?
	}
	n /= len;
	//make sure the normal direction points into m1 and out of m2
	if(n.dot(m1.currentTransform*s1.a) < n.dot(m2.currentTransform*s2.a))
	  n.inplaceNegative();
	/*
	if(n.dot(m1.currentTransform.R*tri1.normal()) > 0.0) {
	  if(n.dot(m2.currentTransform.R*tri2.normal()) > 0.0) {
	    printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
	  }
	  n.inplaceNegative();
	}
	else {
	  if(n.dot(m2.currentTransform.R*tri2.normal()) < 0.0) {
	    printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
	  }
	}
	*/
	//cout<<"Edge vector 1 "<<ev1<<", vector 2" <<ev2<<", normal: "<<n<<endl;
	return n;
      }
      break;
    case 3:  //face
      return m2.currentTransform.R*tri2.normal();
    }
    break;
  case 3:  //face
    if(type2 == 3)
      printf("ODECustomMesh: Warning, face-face contact?\n");
    return m1.currentTransform.R*(-tri1.normal());
  }
  static int warnedCount = 0;
  if(warnedCount % 10000 == 0) 
    printf("ODECustomMesh: Warning, degenerate triangle, types %d %d\n",type1,type2);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

int dCustomMeshCollide (dGeomID o1, dGeomID o2, int flags,
			   dContactGeom *contact, int skip)
{
  int m = (flags&0xffff);
  if(m == 0) m=1;
  //printf("CustomMesh collide\n");
  CustomMeshData* d1 = dGetCustomMeshData(o1);
  CustomMeshData* d2 = dGetCustomMeshData(o2);
  CopyMatrix(d1->mesh->currentTransform.R,dGeomGetRotation(o1));
  CopyVector(d1->mesh->currentTransform.t,dGeomGetPosition(o1));
  //cout<<"Transform 1:"<<endl<<d1->mesh->currentTransform<<endl;
  CopyMatrix(d2->mesh->currentTransform.R,dGeomGetRotation(o2));
  CopyVector(d2->mesh->currentTransform.t,dGeomGetPosition(o2));
  //cout<<"Transform 2:"<<endl<<d2->mesh->currentTransform<<endl;
  CollisionMeshQuery q(*d1->mesh,*d2->mesh);
  bool res=q.WithinDistanceAll(d1->outerMargin+d2->outerMargin);
  if(!res) {
    return 0;
  }

  vector<int> t1,t2;
  vector<Vector3> cp1,cp2;
  q.TolerancePairs(t1,t2);
  q.TolerancePoints(cp1,cp2);
  //printf("%d Collision pairs\n",t1.size());
  const RigidTransform& T1=d1->mesh->currentTransform;
  const RigidTransform& T2=d2->mesh->currentTransform;
  RigidTransform T21; T21.mulInverseA(T1,T2);
  RigidTransform T12; T12.mulInverseA(T2,T1);
  Real tol = d1->outerMargin+d2->outerMargin;
  Real tol2 = Sqr(d1->outerMargin+d2->outerMargin);

  size_t imax=t1.size();
  Triangle3D tri1,tri2,tri1loc,tri2loc;
  if(gDoTriangleTriangleCollisionDetection) {
    //test if more triangle vertices are closer than tolerance
    for(size_t i=0;i<imax;i++) {
      d1->mesh->GetTriangle(t1[i],tri1);
      d2->mesh->GetTriangle(t2[i],tri2);
      
      tri1loc.a = T12*tri1.a;
      tri1loc.b = T12*tri1.b;
      tri1loc.c = T12*tri1.c;
      tri2loc.a = T21*tri2.a;
      tri2loc.b = T21*tri2.b;
      tri2loc.c = T21*tri2.c;
      bool usecpa,usecpb,usecpc,usecpa2,usecpb2,usecpc2;
      Vector3 cpa = tri1.closestPoint(tri2loc.a);
      Vector3 cpb = tri1.closestPoint(tri2loc.b);
      Vector3 cpc = tri1.closestPoint(tri2loc.c);
      Vector3 cpa2 = tri2.closestPoint(tri1loc.a);
      Vector3 cpb2 = tri2.closestPoint(tri1loc.b);
      Vector3 cpc2 = tri2.closestPoint(tri1loc.c);
      usecpa = (cpa.distanceSquared(tri2loc.a) < tol2);
      usecpb = (cpb.distanceSquared(tri2loc.b) < tol2);
      usecpc = (cpc.distanceSquared(tri2loc.c) < tol2);
      usecpa2 = (cpa2.distanceSquared(tri1loc.a) < tol2);
      usecpb2 = (cpb2.distanceSquared(tri1loc.b) < tol2);
      usecpc2 = (cpc2.distanceSquared(tri1loc.c) < tol2);
      //if already existing, disable it
      if(usecpa && cpa.isEqual(cp1[i],cptol)) usecpa=false;
      if(usecpb && cpb.isEqual(cp1[i],cptol)) usecpb=false;
      if(usecpc && cpc.isEqual(cp1[i],cptol)) usecpc=false;
      if(usecpa2 && cpa2.isEqual(cp2[i],cptol)) usecpa2=false;
      if(usecpb2 && cpb2.isEqual(cp2[i],cptol)) usecpb2=false;
      if(usecpc2 && cpc2.isEqual(cp2[i],cptol)) usecpc2=false;
      
      if(usecpa) {
	if(usecpb && cpb.isEqual(cpa,cptol)) usecpb=false;
	if(usecpc && cpc.isEqual(cpa,cptol)) usecpc=false;
      }
      if(usecpb) {
	if(usecpc && cpc.isEqual(cpb,cptol)) usecpc=false;
      }
      if(usecpa2) {
	if(usecpb2 && cpb2.isEqual(cpa2,cptol)) usecpb2=false;
	if(usecpc2 && cpc2.isEqual(cpa2,cptol)) usecpc2=false;
      }
      if(usecpb) {
	if(usecpc2 && cpc.isEqual(cpb2,cptol)) usecpc2=false;
      }
      
      if(usecpa) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(cpa);
	cp2.push_back(tri2.a);
      }
      if(usecpb) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(cpb);
	cp2.push_back(tri2.b);
      }
      if(usecpc) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(cpc);
	cp2.push_back(tri2.c);
      }
      if(usecpa2) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(tri1.a);
	cp2.push_back(cpa2);
      }
      if(usecpb2) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(tri1.b);
	cp2.push_back(cpb2);
      }
      if(usecpc2) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(tri1.c);
	cp2.push_back(cpc2);
      }
    }
    /*
    if(t1.size() != imax)
      printf("ODECustomMesh: Triangle vert checking added %d points\n",t1.size()-imax);
    */
    //getchar();
  }

  imax = t1.size();
  static int warnedCount = 0;
  for(size_t i=0;i<imax;i++) {
    d1->mesh->GetTriangle(t1[i],tri1);
    d2->mesh->GetTriangle(t2[i],tri2);

    tri1loc.a = T12*tri1.a;
    tri1loc.b = T12*tri1.b;
    tri1loc.c = T12*tri1.c;
    if(tri1loc.intersects(tri2)) { 
      if(warnedCount % 1000 == 0) {
	printf("ODECustomMesh: Triangles penetrate margin %g: can't trust contact detector\n",d1->outerMargin+d2->outerMargin);
      }
      warnedCount++;
      /*
      //the two triangles intersect! can't trust results of PQP
      t1[i] = t1.back();
      t2[i] = t2.back();
      cp1[i] = cp1.back();
      cp2[i] = cp2.back();
      i--;
      imax--;
      */
    }
  }
  if(t1.size() != imax) {
    printf("ODECustomMesh: %d candidate points were removed due to collision\n",t1.size()-imax);
    t1.resize(imax);
    t2.resize(imax);
    cp1.resize(imax);
    cp2.resize(imax);
  }
  
  int k=0;  //count the # of contact points added
  for(size_t i=0;i<cp1.size();i++) {
    Vector3 p1 = T1*cp1[i];
    Vector3 p2 = T2*cp2[i];
    Vector3 n=p1-p2;
    Real d = n.norm();
    if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
      n = ContactNormal(*d1->mesh,*d2->mesh,cp1[i],cp2[i],t1[i],t2[i]);
    }
    else if(d > tol) {  //some penetration -- we can't trust the result of PQP
      continue;
    }
    else n /= d;
    //check for invalid normals
    Real len=n.length();
    if(len < gZeroNormalTolerance || !IsFinite(len)) continue;
    //cout<<"Local Points "<<cp1[i]<<", "<<cp2[i]<<endl;
    //cout<<"Points "<<p1<<", "<<p2<<endl;
    CopyVector(contact[k].pos,(p1+p2)*0.5);
    CopyVector(contact[k].normal,n);
    contact[k].depth = tol - d;
    if(contact[k].depth < 0) contact[k].depth = 0;
    //cout<<"Normal "<<n<<", depth "<<contact[i].depth<<endl;
    //getchar();
    contact[k].g1 = o1;
    contact[k].g2 = o2;
    k++;
    if(k == m) break;
  }
  return k;
}

/*
int dCustomMeshTriMeshCollide (dGeomID o1, dGeomID o2, int flags,
				  dContactGeom *contact, int skip)
{
  int n = (flags&0xffff);
  if(n == 0) n=1;  
}
*/

dColliderFn * dCustomMeshGetColliderFn (int num)
{
  if(num == gdCustomMeshClass) return dCustomMeshCollide;
  //else if(num == dTriMeshClass) return dCustomMeshTriMeshCollide;
  else return NULL;
}

void dCustomMeshAABB(dGeomID o,dReal aabb[6])
{
  CustomMeshData* d = dGetCustomMeshData(o);
  Box3D box;
  AABB3D bb;
  CopyMatrix(d->mesh->currentTransform.R,dGeomGetRotation(o));
  CopyVector(d->mesh->currentTransform.t,dGeomGetPosition(o));  
  GetBB(*d->mesh,box);
  box.getAABB(bb);
  CopyVector3(&aabb[0],bb.bmin);
  CopyVector3(&aabb[3],bb.bmax);
}

void dCustomMeshDtor(dGeomID o)
{
}

void InitODECustomMesh()
{
  dGeomClass mmclass;
  mmclass.bytes = sizeof(CustomMeshData);
  mmclass.collider = dCustomMeshGetColliderFn;
  //mmclass.aabb = dCustomMeshAABB;
  mmclass.aabb = dInfiniteAABB;
  mmclass.aabb_test = NULL;
  mmclass.dtor = dCustomMeshDtor;
  gdCustomMeshClass = dCreateGeomClass(&mmclass);
}

