#include "ODERigidObject.h"
#include "ODECommon.h"
#include "ODECustomMesh.h"
#include <ode/ode.h>
#include <errors.h>

double ODERigidObject::defaultPadding = 0.0025;
ODESurfaceProperties ODERigidObject::defaultSurface = {0.1,0.5,Inf,Inf};

ODERigidObject::ODERigidObject(const RigidObject& _obj)
  :obj(_obj),bodyID(0),geometry(0),spaceID(0)
{}

ODERigidObject::~ODERigidObject()
{
  Clear();
}

void ODERigidObject::Create(dWorldID worldID,dSpaceID space)
{
  Clear(); 
  spaceID = space;
  bodyID = dBodyCreate(worldID);

  dMass mass;
  mass.mass = obj.mass;
  //NOTE: in ODE, COM must be zero vector! -- we take care of this by shifting geometry
  //CopyVector3(mass.c,obj.com);
  mass.c[0] = mass.c[1] = mass.c[2] = 0; mass.c[3] = 1.0;
  CopyMatrix(mass.I,obj.inertia);
  int res=dMassCheck(&mass);
  if(res != 1) {
    fprintf(stderr,"Uh... mass is not considered to be valid by ODE?\n");
    std::cerr<<"Inertia: "<<obj.inertia<<std::endl;
  }
  dBodySetMass(bodyID,&mass);
  
  geometry = new ODETriMesh;
  geometry->Create(obj.mesh,spaceID,-obj.com);
  dGeomSetBody(geometry->geom(),bodyID);
  dGeomSetData(geometry->geom(),(void*)-1);
  geometry->SetPadding(defaultPadding);
  geometry->surf().kRestitution = obj.kRestitution;
  geometry->surf().kFriction = obj.kFriction;
  geometry->surf().kStiffness = obj.kStiffness;
  geometry->surf().kDamping = obj.kDamping;

  SetTransform(obj.T);
  UpdateLastTransform();
}

void ODERigidObject::Clear()
{
  SafeDeleteProc(bodyID,dBodyDestroy);
  SafeDelete(geometry);
}

void ODERigidObject::SetTransform(const RigidTransform& T)
{
  Vector3 comPos = T*obj.com;
  dBodySetPosition(bodyID,comPos.x,comPos.y,comPos.z);
  dMatrix3 rot;
  CopyMatrix(rot,T.R);
  dBodySetRotation(bodyID,rot);
}


void ODERigidObject::GetTransform(RigidTransform& T) const
{
  const dReal* pos = dBodyGetPosition(bodyID);
  Vector3 comPos(pos[0],pos[1],pos[2]);
  const dReal* rot = dBodyGetRotation(bodyID);
  CopyMatrix(T.R,rot);
  T.t = comPos - T.R*obj.com;
}

void ODERigidObject::UpdateLastTransform()
{
  if(!bodyID || !geometry) return;
  RigidTransform T;
  const dReal* pos = dBodyGetPosition(bodyID);
  const dReal* rot = dBodyGetRotation(bodyID);
  CopyVector(T.t,pos);
  CopyMatrix(T.R,rot);
  Matrix4 mat(T);
  geometry->SetLastTransform(mat);
}

void ODERigidObject::SetVelocity(const Vector3& w,const Vector3& v)
{
  dBodySetLinearVel(bodyID,v.x,v.y,v.z);
  dBodySetAngularVel(bodyID,w.x,w.y,w.z);
}

void ODERigidObject::GetVelocity(Vector3& w,Vector3& v) const
{
  CopyVector(v,dBodyGetLinearVel(bodyID));
  CopyVector(w,dBodyGetAngularVel(bodyID));
}

bool ODERigidObject::ReadState(File& f)
{
  RigidTransform Told;
  Vector3 w,v;
  dReal pos[3];
  dReal q[4];
  if(!ReadArrayFile(f,pos,3)) return false;
  if(!ReadArrayFile(f,q,4)) return false;
  if(!ReadFile(f,w)) return false;
  if(!ReadFile(f,v)) return false;
  if(!ReadFile(f,Told.R)) return false;
  if(!ReadFile(f,Told.t)) return false;

  dBodySetPosition(bodyID,pos[0],pos[1],pos[2]);
  dBodySetQuaternion(bodyID,q);
  SetVelocity(w,v);
  Matrix4 mat(Told);
  geometry->SetLastTransform(mat);
  return true;
}

bool ODERigidObject::WriteState(File& f) const
{
  //TODO: use body quaternion
  RigidTransform Told;
  Vector3 w,v;
  const dReal* pos=dBodyGetPosition(bodyID);
  const dReal* q=dBodyGetQuaternion(bodyID);
  GetVelocity(w,v);
    
  Matrix4 mat;
  geometry->GetLastTransform(mat);
  Assert(mat(3,0)==0);
  Assert(mat(3,1)==0);
  Assert(mat(3,2)==0);
  Told.set(mat);

  if(!WriteArrayFile(f,pos,3)) return false;
  if(!WriteArrayFile(f,q,4)) return false;
  if(!WriteFile(f,w)) return false;
  if(!WriteFile(f,v)) return false;
  if(!WriteFile(f,Told.R)) return false;
  if(!WriteFile(f,Told.t)) return false;
  return true;
}
