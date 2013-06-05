#include "Planning/RobotDisplacementCSpace.h"
#include "Modeling/Resources.h"
#include <robotics/IKFunctions.h>
#include <fstream>

/** posmeasure.cpp
 * Measures the positions and orientations of all of the robot's links
 * for a given multipath
 */

int main(int argc,char** argv)
{
  if(argc < 3) {
    printf("Usage: posmeasure world path [outfile]\n");
    printf("Spits out link positions/orientations to pos.csv or \n");
    printf("the given file.\n");
    return 0;
  }
  WorldResource world;
  MultiPathResource path;
  world.fileName = argv[1];
  path.fileName = argv[2];
  const char* outfn = "pos.csv";
  if(!world.Load(argv[1])) {
    printf("Failed to load world from %s\n",argv[1]);
    return 1;
  }
  if(!path.Load()) {
    printf("Failed to load path from %s\n",argv[2]);
    return 1;
  }
  if(argc >= 4)
    outfn = argv[3];
  Real discretization = 0.0025/path.path.sections.size(), tmax = 1.0;
  if(path.path.HasTiming())
    tmax = path.path.sections.back().times.back();

  vector<Real> times;
  Real t=0;
  while(t < tmax) {
    times.push_back(t);
    t += discretization;
  }
  times.push_back(tmax);

  Robot* robot = world.world.robots[0].robot;
  RobotCSpace space(*robot);
  RobotGeodesicManifold manifold(*robot);
  GeneralizedCubicBezierCurve curve(&space,&manifold);
  Real duration,param;
  printf("Saving to %s...\n",outfn);
  ofstream out(outfn,ios::out);
  int oldseg=-1;
  for(size_t t=0;t<times.size();t++) {
    int seg=path.path.Evaluate(times[t],curve,duration,param,MultiPath::InterpLinear);
    Config q;
    curve.Eval(param,q);
    robot->UpdateConfig(q);

    //solve for constraints
    vector<IKGoal> ik;
    path.path.GetIKProblem(ik,seg);
    if(!ik.empty()) {
      int iters=100;
      SolveIK(*robot,ik,1e-3,iters,0);
    }
    Stance s;
    path.path.GetStance(s,seg);
    ContactFormation cf;
    ToContactFormation(s,cf);
    /*
      TEMP: override coefficient of friction?
    for(size_t i=0;i<cf.links.size();i++)
      for(size_t j=0;j<cf.contacts[i].size();j++)
	cf.contacts[i][j].kFriction = 0.75;
    */

    if(seg != oldseg) {
      out<<"time";
      for(size_t j=0;j<robot->linkNames.size();j++) {
	out<<","<<robot->linkNames[j];
      }
      for(size_t j=0;j<robot->linkNames.size();j++) {
	out<<",trans["<<robot->linkNames[j]<<"]";
      }
      for(size_t j=0;j<robot->linkNames.size();j++) {
	out<<",rot["<<robot->linkNames[j]<<"]";
      }
      out<<endl;
      oldseg = seg;
    }

    out<<times[t];
    for(int j=0;j<q.n;j++)
      out<<","<<robot->q(j);

    for(int j=0;j<q.n;j++)
      out<<","<<robot->links[j].T_World.t;

    for(int j=0;j<q.n;j++) {
      Real R[9];
      robot->links[j].T_World.R.get(R);
      out<<",";
      for(int k=0;k<9;k++) {
	if(k != 0) out<<" ";
	out<<R[k];
      }
    }
    out<<endl;
  }
  out.close();
  printf("Done.\n");
  return 0;
}
