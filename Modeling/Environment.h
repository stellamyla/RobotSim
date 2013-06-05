#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <geometry/CollisionMesh.h>
#include <vector>
using namespace std;
using namespace Math;

/** @ingroup Modeling
 * @brief A triangulated model of a static environment with known friction.
 */
struct Environment
{
  bool Load(const char* fn);
  bool Save(const char* fn);
  inline void SetUniformFriction(Real mu) {
    kFriction.resize(mesh.tris.size());
    fill(kFriction.begin(),kFriction.end(),mu);
  }

  Geometry::CollisionMesh mesh;
  vector<Real> kFriction;       //per triangle friction
};

#endif
