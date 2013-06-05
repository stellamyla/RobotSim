#include "Control/PathController.h"
#include "Control/FeedforwardController.h"
#include "Control/LoggingController.h"
#include "Simulation/WorldSimulation.h"
#include "Planning/RampCSpace.h"
#include "Planning/RealTimePlanner.h"
#include "WorldViewProgram.h"
#include "Input/InputProcessor.h"
#include "IO/XmlWorld.h"
#include <utils/StatCollector.h>
#include <Robotics/IKFunctions.h>
#include <Robotics/Rotation.h>
#include <Planning/MotionPlanner.h>
#include <math/random.h>
#include <math/SVDecomposition.h>
#include <GLdraw/drawextra.h>
#include <GLdraw/GL.h>
#include <GLdraw/GLLight.h>
#include <GLdraw/GLUTString.h>
#include <glui.h>
#include <fstream>
#include <sstream>
#include <pthread.h>
using namespace Math3D;
using namespace GLDraw;

//show heads up display or not 
#define SHOW_HUD 1
//saves a ppm movie of the performance
#define SAVE_MOVIE 0
//captures stats on workspace position errors (print using 'e')
#define DO_WORKSPACE_ERRORS 0   
//captures timing stats (print using 't')
#define DO_TIMING 0
//commands a straight line path toward the target
#define OVERRIDE_CONTROL 0
//disables ODE simulation (faster)
#define FAKE_SIMULATION 1

enum {
	SIMULATE_BUTTON_ID, RESET_BUTTON_ID, UI_LISTBOX_ID
};

//planner update time step
double dt = 0.01;

typedef MilestonePathController MyController;
inline MyController* GetController(RobotController* rc) {
	return dynamic_cast<MilestonePathController*> (dynamic_cast<LoggingController<
			FeedforwardController>*> (rc)->base);
}

struct CommandInterface {
	//settings
	RobotWorld* world;
	WorldSimulation* sim;
	Camera::Viewport* viewport;
	WorldPlannerSettings* settings;
  StatCollector timingStats;

	CommandInterface() :
		world(NULL), sim(NULL), viewport(NULL) {
	}
	virtual ~CommandInterface() {
	}
	Robot* GetRobot() const {
		return world->robots[0].robot;
	}
	MyController* GetController() const {
		return ::GetController(sim->robotControllers[0]);
	}
	void GetClickRay(int mx, int my, Ray3D& ray) const {
		viewport->getClickSource(mx, viewport->h - my, ray.source);
		viewport->getClickVector(mx, viewport->h - my, ray.direction);
	}

	//Subclasses overload these functions
	//
	virtual string Name() const {
		return "Unnamed";
	}
	virtual string Description() const {
		return "Unnamed";
	}
	virtual string Instructions() const {
		return "";
	}
	virtual void DrawGL() {
	}
	//
	//The following callbacks are called respectively upon
	//activation/deactivation;
	//mouse input;
	//keyboard input;
	//and idle update.
	//The return value is a string that gets logged to disk.
	virtual string ActivateEvent(bool enable) {
		return "";
	}
	virtual string MouseInputEvent(int mx, int my, bool drag) {
		return "";
	}
	virtual string KeypressEvent(unsigned char c, int mx, int my) {
		return "";
	}
	virtual string UpdateEvent() {
		return "";
	}
};


struct JointCommandInterface: public CommandInterface {
	int currentLink;

	virtual string Name() const {
		return "JointPoser";
	}
	virtual string Description() const {
		return "Joint poser";
	}
	virtual string Instructions() const {
		return "Right-click and drag the robot joints to pose individual joints. Drag towards top/bottom to increase/decrease the degree.";
	}
	virtual string ActivateEvent(bool enabled) {
		currentLink = -1;
		return "";
	}
	virtual string MouseInputEvent(int mx, int my, bool drag) {
		if (drag) {
			if (currentLink >= 0) {
#if DO_TIMING
			  Timer timer;
#endif
				//alter current desired configuration
				Config q = GetController()->Endpoint();
				Robot* robot = GetRobot();
				robot->UpdateConfig(q);
				for (size_t i = 0; i < robot->drivers.size(); i++) {
					if (robot->DoesDriverAffect(i, currentLink)) {
						Real val = robot->GetDriverValue(i);
						val = Clamp(val + my * 0.02, robot->drivers[i].qmin,
								robot->drivers[i].qmax);
						robot->SetDriverValue(i, val);
					}
				}
				GetController()->SetMilestone(robot->q);
#if DO_TIMING
			timingStats.collect(timer.ElapsedTime());
#endif

			}
			stringstream ss;
			ss << "Drag " << currentLink << " " << mx << " " << my << endl;
			return ss.str();
		} else {
			Ray3D ray;
			GetClickRay(mx, my, ray);

			sim->UpdateRobot(0);
			int link;
			Vector3 localPos;
			RobotInfo* rob = world->ClickRobot(ray, link, localPos);

			if (rob) {
				currentLink = link;
				world->robots[0].view.SetGrey();
				world->robots[0].view.colors[currentLink].set(1, 1, 0);
			} else {
				world->robots[0].view.SetGrey();
				currentLink = -1;
			}
			return "";
		}
	}
};

struct IKCommandInterface: public CommandInterface {
  StandardInputProcessor * inputProcessor;
	int currentLink;
	Vector3 currentPoint;
	Vector3 currentDestination;

  IKCommandInterface() : inputProcessor(NULL) {}
  virtual ~IKCommandInterface() { SafeDelete(inputProcessor); }
	virtual string Name() const {
		return "PointPoser";
	}
	virtual string Description() const {
		return "Point poser";
	}
	virtual string Instructions() const {
		return "Right-click and drag any point on the robot to the place you want it to be";
	}

	virtual string ActivateEvent(bool enabled) {
	  if(!inputProcessor) {
	    inputProcessor = new StandardInputProcessor;
	    inputProcessor->world = world;
	    inputProcessor->sim = sim;
	    inputProcessor->viewport = viewport;
	  }
	  return "";
	}

	virtual void DrawGL() {
	  if(inputProcessor) inputProcessor->DrawGL();
	}

  void Solve()
  {
    PlannerObjectiveBase* obj = inputProcessor->MakeObjective(GetRobot());
    if(!obj) return;
    CartesianObjective* pobj = dynamic_cast<CartesianObjective*>(obj);
    assert(pobj != NULL);

    vector<IKGoal> problem(1, pobj->ikGoal);

    Config q = GetController()->Endpoint();
    Robot* robot = GetRobot();
    robot->UpdateConfig(q);
    int iters = 100;
    bool res = SolveIK(*robot, problem, 1e-3, iters, 0);
    
    GetController()->SetMilestone(robot->q);

    delete obj;
  }

  virtual string MouseInputEvent(int mx, int my, bool drag)
  {
    if (drag) {
      inputProcessor->Drag(mx,my);
      stringstream ss;
      ss << "Drag " << mx << " " << my << endl;
      return ss.str();
    } else {
      inputProcessor->Hover(mx,my);
      return "";
    }
  }

  virtual string UpdateEvent() {
    Solve();
    return "";
  }
};

struct PFCommandInterface: public IKCommandInterface {
	Config currentStart;
	bool initialedMesh;
	bool firstMove;
	IKGoal goal;
	vector<pair<int, int> > meshPairs;
	vector<pair<int, int> > allMeshPairs;
	vector<Geometry::CollisionMeshQueryEnhanced> meshQueries;
        vector<Real> queryDistances;

	PFCommandInterface() :
	  initialedMesh(false),firstMove(true)  {
	}
	virtual string Name() const {
		return "PFPointPoser";
	}
	virtual string Description() const {
		return "PF Point poser";
	}
	void InitialMesh() {
		settings->EnumerateCollisionPairs(*world, allMeshPairs);
		int robotID = world->GetID(world->robots[0].name);
		for (size_t i = 0; i < allMeshPairs.size(); i++) {
			if ((world->IsRobotLink(allMeshPairs[i].first) != pair<int, int> (
					-1, -1) || world->IsRobotLink(allMeshPairs[i].second)
					!= pair<int, int> (-1, -1))) {
				if ((world->IsRobotLink(allMeshPairs[i].first)).second > 2) {
					settings->EnumerateCollisionQueries(*world,
							allMeshPairs[i].first, allMeshPairs[i].second,
							meshPairs, meshQueries);
					//cout<<"pair<"<<allMeshPairs[i].first<<","<<allMeshPairs[i].second<<">"<<endl;
				}
			}
		}
		initialedMesh = true;
	}

	virtual string MouseInputEvent(int mx, int my, bool drag) {
	  if (!initialedMesh) {
	    InitialMesh();
	  }
	  IKCommandInterface::MouseInputEvent(mx,my,drag);
	  return "";
	}

	virtual string UpdateEvent() {
		int sampleRate = 3;

		//read off the goal
		PlannerObjectiveBase* obj=inputProcessor->MakeObjective(GetRobot());
		if (!obj) return "";
		CartesianObjective* pgoal = dynamic_cast<CartesianObjective*>(obj);
		assert(pgoal != NULL);
		goal = pgoal->ikGoal;
		delete obj;

#if DO_TIMING
		    Timer timer;
#endif
		  bool recomputeClosestPoints=false;
		  if ((int(sim->time * 100.0)) % sampleRate == 0 || queryDistances.empty())
		    recomputeClosestPoints = true;

		  Real timeStep = sampleRate * dt;
		  Robot* robot = GetRobot();
		  Config Q_current = GetController()->xcur;
		  Config dQ_current = GetController()->dxcur;
		  Vector3 P_current, P_goal;
		  P_goal = goal.endPosition;
		  robot->UpdateConfig(Q_current);
		  robot->GetWorldPosition(goal.localPosition, goal.link, P_current);
		  Real D_goal = getGoalDistance(Q_current);
		  Vector3 v_attract;
		  GetAttractVelocity(P_current, P_goal, D_goal, v_attract);
		  Vector3 v_repul;
		  Real D_obst;
		  Matrix Jobst, Jatt, Jcomb, Jp, Jiatt, Jirepu, Jicomb;
		  Config dQ;
		  dQ.resize(Q_current.n);
		  robot->UpdateConfig(Q_current);
		  robot->dq = dQ_current;
		  robot->UpdateGeometry();
		  //getRepulsiveForce(v_repul, D_obst, Jobst,recomputeClosestPoints);
			getRepulsiveForces(dQ,recomputeClosestPoints);
			if(!IsFinite(v_repul.norm())) v_repul.setZero();
			robot->UpdateConfig(Q_current);
			robot->GetPositionJacobian(goal.localPosition, goal.link, Jatt);
			//zero out the first column
			for(int i=0;i<Jatt.m;i++)
			  Jatt(i,0) = 0.0;
			//for(int i=0;i<Jobst.m;i++)
			//Jobst(i,0) = 0.0;

			Vector Vcomb;
			Vcomb.resize(6);

			Vcomb[0] = v_attract.x;
			Vcomb[1] = v_attract.y;
			Vcomb[2] = v_attract.z;
			Vcomb[3] = v_repul.x;
			Vcomb[4] = v_repul.y;
			Vcomb[5] = v_repul.z;
			//cout<<"before: m="<<Jcomb.m<<" n="<<Jcomb.n<<endl;

			Jcomb.resize(Jatt.m * 2, Jatt.n);
			Jcomb.copySubMatrix(0, 0, Jatt);
			//Jcomb.copySubMatrix(Jatt.m, 0, Jobst);

			//project Jcomb on the constraint direction
			Vector3 d_repul = v_repul;
			if(d_repul.norm() >= Epsilon)
			  d_repul.inplaceNormalize();
			else
			  d_repul.setZero();

			//cout<<"after:  m="<<Jcomb.m<<" n="<<Jcomb.n<<endl;
			Vector Vatt, Vrepu;
			Vatt.resize(3);
			Vrepu.resize(3);
			Vatt[0] = v_attract.x;
			Vatt[1] = v_attract.y;
			Vatt[2] = v_attract.z;
			Vrepu[0] = v_repul.x;
			Vrepu[1] = v_repul.y;
			Vrepu[2] = v_repul.z;
			SVDecomposition<Real> svd;
			/*
			svd.set(Jatt);
			svd.getInverse(Jiatt);
			svd.set(Jobst);
			svd.getInverse(Jirepu);
			Jicomb.resize(Jiatt.m * 2, Jiatt.n);
			Jicomb.copySubMatrix(0, 0, Jiatt);
			Jicomb.copySubMatrix(Jiatt.m, 0, Jirepu);
			*/
			//cout<<"Jp:  m="<<Jp.m<<" n="<<Jp.n<<endl;
			//Matrix JJT, JTi;
			//			cout<<"m"<<J.m<<"n"<<J.n<<endl;
			//JJT.mulTransposeB(J,J);
			//cout<<"m"<<JJT.m<<"n"<<JJT.n<<endl;
			//JJT.inplaceInverse();
			//JTi.mul(JJT,J);
			//Jp.mul(Vcomb, dQ);


			/**
			 * well tested working method
			 */
			//Jcomb.mulTranspose(Vcomb, dQ);

			//Jacobian pseudoinverse method for attractive field
			svd.set(Jatt);
			//svd.getInverse(Jiatt);
			svd.getDampedPseudoInverse(Jiatt,0.1);
			Jiatt.madd(Vatt,dQ);

			//Jacobian transpose method for attractive field
			//Jatt.maddTranspose(Vatt,dQ);

			//Jobst.maddTranspose(Vrepu,dQ);

			/*
			Matrix JcombInv;
			svd.set(Jcomb);
			svd.getInverse(JcombInv);
			JcombInv.mul(Vcomb,dQ);
			*/
			//cout<<"Vcomb: "<<Vcomb<<endl;
			//cout<<"dQ: "<<dQ<<endl;

			/**
			 * use attract only and pseudo inverse
			 */
			//Jp.mul(Vcomb,dQ);
			//Jicomb.mul(Vcomb, dQ);
			//cout<<"m"<<JTi.m<<"n"<<JTi.n<<endl;

			//cout<<"before vel limit:"<<dQ.dotSelf()<<endl;
			//dQ -= dQ_current*timeStep*5.0;
			dQ[0] = 0.0;
			Real minScale = 1.0;
			for (int i = 1; i < dQ.n; i++) {  //skip first joint
			  if (dQ[i] < robot->velMin[i]) {
			    minScale = Min(minScale,robot->velMin[i]/dQ[i]);
			    //printf("Vel min %d active, vel %g < %g\n",i,dQ[i],robot->velMin[i]);
			  }
			  else if (dQ[i] > robot->velMax[i]) {
			    //printf("Vel max %d active, vel %g > %g\n",i,dQ[i],robot->velMax[i]);
			    minScale = Min(minScale,robot->velMax[i]/dQ[i]);
			  }
			}
			//printf("Velocity scaled by %g\n",minScale);
			dQ *= minScale;
			//acceleration limits
			dQ = dQ-dQ_current;
			minScale = 1.0;
			for (int i = 1; i < dQ.n; i++) {
			  if (Abs(dQ[i]) > robot->accMax[i]*timeStep) {
			    //printf("Accel max %d active, acc %g > %g\n",i,dQ[i]/timeStep,robot->accMax[i]);
			    minScale=Min(minScale,robot->accMax[i]*timeStep/Abs(dQ[i]));
			  }
			}
			dQ = dQ_current + minScale*dQ;

			//handle joint limits
			for(int i=0;i<dQ.n;i++) {
			  //x + t v - 1/2 t^2 amax = 0
			  //v - t amax = 0  => tmax =  v/amax
			  //x = 1/2 v^2/amax
			  if(robot->accMax(i) == 0) continue;
			  Real dmin = 0.5*Sqr(dQ(i))/robot->accMax(i);
			  Real d = robot->q(i)-robot->qMin(i)-Epsilon;
			  if((d < dmin && dQ(i) < 0) || d < 0) {
			    printf("Joint min limit %d active, distance %g, vel %g, desired new dq ",i,d,dQ(i));
			    dQ(i) = dQ_current(i)+robot->accMax[i]*timeStep;
			    printf("%g\n",dQ(i));
			  }
			  else {
			    d = robot->qMax(i)-Epsilon-robot->q(i);
			    if((d < dmin && dQ(i) > 0) || d < 0) {
			      printf("Joint max limit %d active, distance %g, vel %g, desired new dq ",i,d,dQ(i));
			      dQ(i) = dQ_current(i)-robot->accMax[i]*timeStep;
			      printf("%g\n",dQ(i));
			    }
			  }
			}
			//cout<<"Limited dQ: "<<dQ<<endl;
			/*
			 //cout<<"after vel limit:"<<dQ.dotSelf()<<endl;
			 Config ddQ;
			 ddQ = (dQ - dQ_current) / timeStep;
			 for (int i = 0; i < ddQ.n; i++) {
			 if (ddQ[i] > robot->accMax[i])
			 ddQ[i] = robot->accMax[i];
			 }
			 dQ = dQ_current + ddQ * timeStep;
			 //cout<<"after acc limit:"<<dQ.dotSelf()<<endl;
			 */
			Config nextQ = Q_current + dQ * timeStep;
			for (int i = 1; i < nextQ.n; i++) {
				if (nextQ[i] < robot->qMin[i]+Epsilon)
				  timeStep = Min(timeStep,(robot->qMin[i]+Epsilon-Q_current[i])/dQ[i]);
				else if (nextQ[i] > robot->qMax[i]-Epsilon)
				  timeStep = Min(timeStep,(robot->qMax[i]-Epsilon-Q_current[i])/dQ[i]);
			}
			if(timeStep < 0.01) timeStep = 0.01;
			nextQ = Q_current + dQ * timeStep;
			//printf("Time step: %g\n",timeStep);
			//GetController()->SetMilestone(nextQ);
			GetController()->SetLinearVelocity(dQ,timeStep);
#if DO_TIMING
			timingStats.collect(timer.ElapsedTime());
#endif
			return "";
	}

	void GetAttractVelocity(Vector3& P_current, Vector3& P_goal, Real dist,
			Vector3& v_attract) {
	  Real D_attract = 0.5;
	  Real C_attract = 10;
		v_attract = (P_goal-P_current)*C_attract;
		return;
		/*
		v_attract = (P_goal - P_current) / (P_goal - P_current).norm();
		if (dist > D_attract) {
			v_attract = v_attract * C_attract;
		} else {
			Real temp = C_attract * (dist / D_attract);
			v_attract = v_attract * temp;
		}
		*/
	}

  void computeRepulsiveForce(int queryindex,Real d,Vector3& repul, Matrix& J) {
		Vector3 v1, v2;
		//cout<<"Query index "<<queryindex<<endl;
		meshQueries[queryindex].ClosestPoints(v1, v2);
		//cout<<v1<<", "<<v2<<endl;
		Vector3 worldP1, worldP2;
		worldP1 = meshQueries[queryindex].m1->currentTransform*v1;
		worldP2 = meshQueries[queryindex].m2->currentTransform*v2;
		//cout<<worldP1<<", "<<worldP2<<endl;
		int robotLinkNum = world->IsRobotLink(meshPairs[queryindex].first).second;
		Robot* robot = GetRobot();

		Vector3 vel1;
		robot->GetWorldVelocity(v1,robotLinkNum,robot->dq,vel1);

		Real V_obst = dot(worldP2 - worldP1,vel1)/(worldP2.distance(worldP1));
		//ERKANG'S STRATEGY, SLIGHTLY MODIFIED
		Real V_max = 10;
		Real a_max = 2;
		Real D_min = 0.2;
		//Real V_repul_max = 2*V_max + a_max * dt * 3;
		Real V_repul_max = 1.1 * V_max;
		//cout<<"Vel to obstcale: "<<V_obst<<endl;
		Real D_safe = (V_obst * V_obst) / (2 * a_max);
		D_safe += D_min;
		//if (D_safe < D_min)
		//	D_safe = D_min;
		Real C_repul = 5;
		Real V_repul = C_repul * Sqr(V_obst / d);
		//cout<<"Vel away from obstcale: "<<V_repul<<endl;
		if (V_obst > 0) {
		  if (d > D_safe) {
		    //printf("Turning repulsion %d off, safe distance %g\n",queryindex,D_safe);
		    V_repul = 0;
		  }
		  //else
		  // printf("Repulsion %d = %g, vel %g\n",queryindex,V_repul,V_obst);
		} else {
		  //printf("Turning repulsion %d off, velocity %g\n",queryindex,V_obst);
		  V_repul = 0;
		}

		/*
		//cout<<"V_obst "<<V_obst<<endl;
		Real V_safe = -0.3;
		if(V_obst  < V_safe) {  //moving away sufficiently fast
		  repul.setZero();
		  return;
		}
		//Real V_max = 10;
		Real V_max = 1.5;
		//Real a_max = 5;
		Real a_max = 0.75;
		Real D_min = 0.4;
		//KH: my attempt at making a good repulsive field
		Real C_repul = 1.0;
		Real V_repul = 0.0;
		if(d < D_min)
		  V_repul = C_repul*(V_obst-V_safe)*Sqr(1.0/d - 1.0/D_min);
		*/

		/*
		//Real V_repul_max = 2*V_max + a_max * dt * 3;
		Real V_repul_max = 1.1 * V_max;
		//cout<<"Vel to obstcale: "<<V_obst<<endl;
		Real D_safe = (V_obst * V_obst) / (2 * a_max);
		if (D_safe < D_min)
			D_safe = D_min;
		Real C_repul = 5;
		Real V_repul = C_repul * (V_obst / d) * (V_obst / d);
		//cout<<"Vel away from obstcale: "<<V_repul<<endl;
		if (V_obst > 0) {
			if (d > D_safe) {
				V_repul = V_repul < V_repul_max ? V_repul : V_repul_max;
			} else {
				V_repul = V_repul_max;
			}
		} else {
			if (d <= D_safe)
				V_repul = V_repul_max;
			else
				V_repul = 0;
		}
		*/

		//printf("Closest link %d, position %g %g %g, ",robotLinkNum,worldP1.x,worldP1.y,worldP1.z);
		//printf("World position %g %g %g\n",worldP2.x,worldP2.y,worldP2.z);
		repul = worldP1 - worldP2;
		if(d > 0) {
		  //ERKANG'S STRATEGY
		  repul *= V_repul / d;
		  //repul *= V_repul;
		}
		else {
		  //penetrating?
		  repul *= -V_repul;
		}

		if(V_repul > 0) {
		  robot->GetPositionJacobian(v1,robotLinkNum,J);
		}
		else
		  repul.setZero();
  }

	Real getGoalDistance(Config& q) {
		Vector3 pworld;
		Robot* robot = GetRobot();
		robot->UpdateConfig(q);
		robot->GetWorldPosition(goal.localPosition, goal.link, pworld);
		return goal.endPosition.distance(pworld);
	}

  void getRepulsiveForce(Vector3& repul, Real& distance, Matrix& J, bool recompute=true) {
		Real minD = Inf;
		int closestCollision;
		Timer timer;
		closestCollision = -1;
		if(recompute) {
		  queryDistances.resize(meshQueries.size());
		  for (size_t i = 0; i < meshQueries.size(); i++) {
		    queryDistances[i] = meshQueries[i].Distance_Coherent(1e-3, 1e-2,minD);
		    if (queryDistances[i] < minD) {
		      minD = queryDistances[i];
		      closestCollision = (int)i;
		    }
		  }
		}
		else {
		  for(size_t i=0;i<queryDistances.size();i++) {
		    if (queryDistances[i] < minD) {
		      minD = queryDistances[i];
		      closestCollision = (int)i;
		    }
		  }
		}
		if(closestCollision >= 0) {
		  printf("Distance testing took time %g\n",timer.ElapsedTime());
		  computeRepulsiveForce(closestCollision,minD,repul,J);
		}
		distance = minD;
	}

  void getRepulsiveForces(Vector& dq,bool recompute = true) {
		Timer timer;
		Vector3 repul;
		Matrix J;
		dq.setZero();
		if(recompute) {
		  queryDistances.resize(meshQueries.size());
		  for(size_t i=0;i<meshQueries.size();i++) {
		    //queryDistances[i] = meshQueries[i].Distance_Coherent(1e-3, 1e-2);
		    queryDistances[i] = meshQueries[i].Distance_Coherent(1e-2, 1e-1);
		  }
		}
		for (size_t i = 0; i < meshQueries.size(); i++) {
		  computeRepulsiveForce(i,queryDistances[i],repul,J);
		  if(!repul.isZero()) {
		    //cout<<"repulsion "<<i<<": "<<repul<<endl;
		    for(int j=0;j<dq.n;j++)
		      dq(j) += repul.x*J(0,j)+repul.y*J(1,j)+repul.z*J(2,j);
		    //cout<<"dq now "<<dq<<endl;
		  }
		}
		/*
		//joint limit forces
		Robot* robot = GetRobot();
		Real C_repul = 2.0;
		Real V_safe = 0;
		for(int i=0;i<dq.n;i++) {
		  //x + t vmax - 1/2 t^2 amax = 0
		  //vmax - t amax = 0  => tmax =  vmax/amax
		  //x = 1/2 vmax^2/amax
		  if(robot->accMax(i) == 0) continue;
		  Real dmin = 0.5*Sqr(robot->velMax(i))/robot->accMax(i);
		  Real d = robot->q(i)-robot->qMin(i);
		  if(d < dmin) {
		    if(robot->dq(i) < -V_safe) {
		      printf("Joint min limit %d active, distance %g, vel %g, desired new dq",i,d,robot->dq(i));
		      dq(i) += -C_repul*(robot->dq(i)+V_safe)*Sqr(1.0/d-1.0/dmin);
		      printf("%g\n",dq(i));
		    }
		  }
		  d = robot->qMax(i)-robot->q(i);
		  if(d < dmin) {
		    if(robot->dq(i) > V_safe) {
		      printf("Joint max limit %d active, distance %g, vel %g, desired new dq",i,d,robot->dq(i));
		      dq(i) += -C_repul*(robot->dq(i)-V_safe)*Sqr(1.0/d-1.0/dmin);
		      printf("%g\n",dq(i));
		    }
		  }
		}
		*/
		printf("Distance testing took time %g\n",timer.ElapsedTime());
	}
};



struct PlannerCommandInterface: public CommandInterface
{
  RealTimePlannerBase* planner;
  InputProcessorBase* inputProcessor;
  double nextPlanTime;
  double unsuccessfulPlanTime;

  PlannerCommandInterface()
    : planner(NULL), inputProcessor(NULL), nextPlanTime(0), unsuccessfulPlanTime(0)
  {
  }
  virtual ~PlannerCommandInterface() {
    SafeDelete(planner);
    SafeDelete(inputProcessor);
  }
  virtual string Name() const {
    return "UnnamedPlanner";
  }
  virtual string Description() const {
    return "Unnamed planner interface";
  }
  virtual string Instructions() const {
    return "Right-click and drag any point on the robot to the place you want it to be";
  }
  
  virtual string ActivateEvent(bool enabled) {
    return "";
  }
  
  virtual void DrawGL() {
    if (inputProcessor) {
      inputProcessor->DrawGL();
    }
  }
   
  bool UpdateGoal()
  {
    MyController* c = GetController();
    inputProcessor->SetPredictionTime(planner->currentSplitTime);
    PlannerObjectiveBase* obj = inputProcessor->MakeObjective(GetRobot());
    if(!obj) {
      planner->Reset(NULL);
      return false;
    }
    c->GetPath(planner->currentPath);
    planner->Reset(obj);
    return true;
  }

	virtual string MouseInputEvent(int mx, int my, bool drag) {
		if (drag) {
		  inputProcessor->Drag(mx,my);
		  stringstream ss;
		  ss << "Drag " << mx << " " << my << endl;
		  return ss.str();
		} else {
		  inputProcessor->Hover(mx,my);
		  return "";
		}
	}

	virtual string UpdateEvent() {
		if (!planner)
			return "";

		MyController* c = GetController();

		if (sim->time >= nextPlanTime) {
		  if(!UpdateGoal()) return ""; //no objective
			Real splitTime, planTime;
			bool res = false;
			res = planner->PlanUpdate(splitTime, planTime);
			stringstream ss;
			nextPlanTime = sim->time + splitTime;
			if (res) {
				c->SetPath(planner->currentPath);
			}
			ss << "Plan " << planTime << ", padding "
					<< planner->currentPadding << ", split " << splitTime
					<< ", res " << res << endl;
#if DO_TIMING
			if(res) {
			  timingStats.collect(planTime+unsuccessfulPlanTime);
			  unsuccessfulPlanTime = 0;
			}
			else
			  unsuccessfulPlanTime += planTime;
#endif
			return ss.str();
		}
		return "";
	}
};

struct IKPlannerCommandInterface: public PlannerCommandInterface {
	SmartPointer<SingleRobotCSpace> cspace;

	virtual string Name() const {
		return "IKPointPoser";
	}
	virtual string Description() const {
		return "Smart point poser";
	}
	virtual string Instructions() const {
		return "Right-click and drag any point on the robot to the place you want it to be";
	}

	virtual string ActivateEvent(bool enabled) {
		if (!planner) {
			cspace = new SingleRobotCSpace(*world, 0, settings);

			planner = new RealTimeIKPlanner;
			planner->protocol = RealTimePlannerBase::Constant;
			planner->currentSplitTime = 0.05;
			planner->currentPadding = 0.025;
			planner->robot = GetRobot();
			planner->settings = settings;
			planner->cspace = cspace;
			inputProcessor = new StandardInputProcessor;
			inputProcessor->world = world;
			inputProcessor->sim = sim;
			inputProcessor->viewport = viewport;
		}
		return "";
	}
};

struct RRTCommandInterface: public PlannerCommandInterface {
	//TODO: draw the plan feedback?
	GLuint planDisplayList;

	SmartPointer<SingleRobotCSpace> cspace;

	virtual string Name() const {
		return "RRTPointPoser";
	}
	virtual string Description() const {
		return "Goal-based point poser";
	}
	virtual string Instructions() const {
		return "Right-click and drag any point on the robot to the place you want it to be";
	}

	virtual string ActivateEvent(bool enabled) {
		if (!planner) {
			cspace = new SingleRobotCSpace(*world, 0, settings);

			//RealTimeRRTPlanner* p = new RealTimeRRTPlanner;
			RealTimeTreePlanner* p = new RealTimeTreePlanner;
			p->delta = 0.5;
			planner = p;
			planner->robot = GetRobot();
			planner->settings = settings;
			planner->cspace = cspace;
			inputProcessor = new StandardInputProcessor;
			inputProcessor->world = world;
			inputProcessor->sim = sim;
			inputProcessor->viewport = viewport;
		}
		return "";
	}
};




struct RealTimePlannerData
{
  pthread_mutex_t mutex;
  RealTimePlannerBase* planner;
  InputProcessorBase* inputProcessor;
  bool active;             //set this to false to quite
  int planState;           //(in/out) 0 - waiting command, 1 command available, 2 planning, 3 plan done
  bool overrun;            //(in) whether the last step overran time
  int currentLink;
  Vector3 currentPoint;
  Vector3 currentDestination;
  Real startPlanTime;       //(in) time measured in calling thread
  ParabolicRamp::DynamicPath currentPath;  //(in,out) plan starting from current time
  Timer globalTimer;        //(global) 
  Timer planTimer;          //(out) timer that is reset at plan start
  bool planSuccess;         //(out) whether the plan succeeded or not
  Real splitTime;           //(out) amount of time beyond startPlanTime that is unchanged in currentPath
};

void* planner_thread_func(void * ptr)
{
  RealTimePlannerData* data = reinterpret_cast<RealTimePlannerData*>(ptr);
  assert(data->planState == 0);
  data->planner->currentPath = data->currentPath;
  while (true) {
    //first wait for a lock
    pthread_mutex_lock(&data->mutex);

    //parse the data
    if(!data->active) {
      //quit
      pthread_mutex_unlock(&data->mutex);
      return NULL;
    }
    if(data->overrun) {
      data->overrun = false;
      data->planner->MarkLastFailure();
      printf("Planner overrun, padding increased to %g\n",data->planner->currentExternalPadding);
    }
    bool start = (data->planState==1 && data->planner);
    if(start) {
      data->planState = 2;
      data->planTimer.Reset();
      data->planSuccess = false;
      data->splitTime = 0;
      data->planner->currentPath = data->currentPath;
      data->planner->currentExternalPadding = Max(data->planner->currentExternalPadding,0.01);
      printf("*** split %g, padding %g, external padding %g ***\n",data->planner->currentSplitTime,data->planner->currentPadding,data->planner->currentExternalPadding);
    }
    if(data->inputProcessor->HasUpdate()) {
      PlannerObjectiveBase* obj = data->inputProcessor->MakeObjective(data->planner->robot);
      if(obj) {
	data->planner->Reset(obj);
      }
      else {
	data->planner->Reset(NULL);
	data->planState = 0;
	start = false;
      }
    }
    pthread_mutex_unlock(&data->mutex);

    if(start) {
      //do the planning
      Timer timer;
      Real splitTime, planTime;
      bool res = data->planner->PlanUpdate(splitTime, planTime);

      printf("Planning thread: result %d\n",(int)res);    
      //write out the data
      pthread_mutex_lock(&data->mutex);
      data->planState = 3;
      data->planSuccess = res;
      data->splitTime = splitTime;
      if (res) {
	data->currentPath = data->planner->currentPath;
      }
      pthread_mutex_unlock(&data->mutex);
    }
    usleep(1);
  }
  return NULL;
}


class MTPlannerCommandInterface: public CommandInterface
{
public:
  //need the planner to point into the planningWorld
  //the inputProcessor can point into the regular world
  RobotWorld planningWorld;
  RealTimePlannerBase* planner;
  InputProcessorBase* inputProcessor;
  pthread_t planningThread;
  RealTimePlannerData data;
  double unsuccessfulPlanTime;

  MTPlannerCommandInterface() :
    planner(NULL), inputProcessor(NULL), unsuccessfulPlanTime(0)
  {
    data.mutex = PTHREAD_MUTEX_INITIALIZER;
  }
  virtual ~MTPlannerCommandInterface() {
    SafeDelete(planner);
  }
  virtual string Name() const {
    return "UnnamedPlanner";
  }
  virtual string Description() const {
    return "Unnamed planner interface";
  }
  virtual string Instructions() const {
    return "Right-click and drag any point on the robot to the place you want it to be";
  }
  
  virtual string ActivateEvent(bool enabled) {
    if(enabled) {
      data.planner = planner;
      data.inputProcessor = inputProcessor;
      data.active = true;
      data.overrun = false;
      data.planState = 0;
      data.planSuccess = false;
      GetController()->GetPath(data.currentPath);
      assert((int)data.currentPath.velMax.size()==(int)GetRobot()->q.size());
      pthread_create(&planningThread,NULL,planner_thread_func,&data);
      printf("Creating planning thread\n");
    }
    else {
      pthread_mutex_lock(&data.mutex);
      data.active = false;
      pthread_mutex_unlock(&data.mutex);
      pthread_join(planningThread,NULL);
    }
    return "";
  }

  virtual void DrawGL() {
    if (inputProcessor) {
      sim->UpdateRobot(0);
      inputProcessor->DrawGL();
    }
  }

  virtual string MouseInputEvent(int mx, int my, bool drag) {
    sim->UpdateRobot(0);

    //update the planning world for the input processor
    if (drag) {
      pthread_mutex_lock(&data.mutex);
      data.inputProcessor->Drag(mx,my);
      pthread_mutex_unlock(&data.mutex);

      stringstream ss;
      ss << "Drag " << mx << " " << my << endl;
      return ss.str();
    } 
    else {
      pthread_mutex_lock(&data.mutex);
      data.inputProcessor->Hover(mx,my);
      pthread_mutex_unlock(&data.mutex);
      return "";
    }
  }
  
  virtual string UpdateEvent() {
    if (!planner)
      return "";
    
    MyController* c = GetController();
    
    //otherwise try planning
    pthread_mutex_lock(&data.mutex);
    if(data.planState == 0 || data.planState == 3) {
      if(data.planState == 3 && data.planSuccess) {
	if(data.splitTime+data.startPlanTime < sim->time) {
	  //overrun
	  printf("Sim thread: Planner overrun, split %g, delay %g\n",data.splitTime,sim->time - data.startPlanTime);
	  data.overrun = true;
	}
	else {
	  printf("Sim thread: Planner successful, split %g, delay %g\n",data.splitTime,sim->time - data.startPlanTime);
	  ParabolicRamp::DynamicPath before,after;
	  data.currentPath.Split(sim->time - data.startPlanTime,before,after);
	  c->SetPath(after);
	}
      }
#if DO_TIMING
      if(res) {
	timingStats.collect(planTime+unsuccessfulPlanTime);
	unsuccessfulPlanTime = 0;
      }
      else
	unsuccessfulPlanTime += planTime;
#endif

      printf("Sim thread: starting new plan\n");
      data.planState = 1;
      data.startPlanTime = sim->time;
      c->GetPath(data.currentPath);
    }
    else {
      //printf("Sim thread: waiting for plan to complete\n");
    }
    pthread_mutex_unlock(&data.mutex);
    return "";
  }
};

struct MTIKPlannerCommandInterface: public MTPlannerCommandInterface {
	SmartPointer<SingleRobotCSpace> cspace;

	virtual string Name() const {
		return "IKPointPoser";
	}
	virtual string Description() const {
		return "Smart point poser (MT)";
	}
	virtual string Instructions() const {
		return "Right-click and drag any point on the robot to the place you want it to be";
	}

	virtual string ActivateEvent(bool enabled) {
		if (!planner) {
		  CopyWorld(*world,planningWorld);
		  cspace = new SingleRobotCSpace(planningWorld, 0, settings);
		  
		  planner = new RealTimeIKPlanner;
		  planner->protocol = RealTimePlannerBase::Constant;
		  planner->currentSplitTime = 0.05;
		  planner->currentPadding = 0.025;
		  planner->robot = planningWorld.robots[0].robot;
		  planner->settings = settings;
		  planner->cspace = cspace;
		  inputProcessor = new StandardInputProcessor;
		  inputProcessor->world = world;
		  inputProcessor->sim = sim;
		  inputProcessor->viewport = viewport;
		}
		return MTPlannerCommandInterface::ActivateEvent(enabled);
	}
};

struct MTRRTCommandInterface: public MTPlannerCommandInterface {
	//TODO: draw the plan feedback?
	GLuint planDisplayList;

	SmartPointer<SingleRobotCSpace> cspace;

	virtual string Name() const {
		return "RRTPointPoser";
	}
	virtual string Description() const {
		return "Goal-based point poser (MT)";
	}
	virtual string Instructions() const {
		return "Right-click and drag any point on the robot to the place you want it to be";
	}

	virtual string ActivateEvent(bool enabled) {
		if (!planner) {
		  CopyWorld(*world,planningWorld);
		  cspace = new SingleRobotCSpace(planningWorld, 0, settings);
		  
		  //RealTimeRRTPlanner* p = new RealTimeRRTPlanner;
		  RealTimeTreePlanner* p = new RealTimeTreePlanner;
		  p->delta = 0.5;
		  planner = p;
		  planner->robot = planningWorld.robots[0].robot;
		  planner->settings = settings;
		  planner->cspace = cspace;
		  
		  inputProcessor = new StandardInputProcessor;
		  inputProcessor->world = world;
		  inputProcessor->sim = sim;
		  inputProcessor->viewport = viewport;
		}
		return MTPlannerCommandInterface::ActivateEvent(enabled);
	}
};

struct MT_RRT_FTIECommandInterface: public MTPlannerCommandInterface {
	//TODO: draw the plan feedback?
	GLuint planDisplayList;

	SmartPointer<SingleRobotCSpace> cspace;

	virtual string Name() const {
		return "RRTPointPoser";
	}
	virtual string Description() const {
		return "CMP + FTIE (MT)";
	}
	virtual string Instructions() const {
		return "Right-click and drag any point on the robot to the place you want it to be";
	}

	virtual string ActivateEvent(bool enabled) {
		if (!planner) {
		  CopyWorld(*world,planningWorld);
		  cspace = new SingleRobotCSpace(planningWorld, 0, settings);
		  
		  //RealTimeRRTPlanner* p = new RealTimeRRTPlanner;
		  RealTimeTreePlanner* p = new RealTimeTreePlanner;
		  p->delta = 0.5;
		  planner = p;
		  planner->robot = planningWorld.robots[0].robot;
		  planner->settings = settings;
		  planner->cspace = cspace;
		  
		  TaskPredictorInputProcessor* tproc = new TaskPredictorInputProcessor;
		  inputProcessor = tproc;
		  inputProcessor->world = world;
		  inputProcessor->sim = sim;
		  inputProcessor->viewport = viewport;
		  if(!tproc->InitializeDefault())
		    exit(-1);
		}
		return MTPlannerCommandInterface::ActivateEvent(enabled);
	}
};




class UserTrialProgram: public WorldViewProgram {
public:
	int simulate;
	Timer realClock;
        double accumTime;
	double finishTime;
	double tutorialFinishTime;
	long long int numberCollision;
	Real maxCollisionF;
	bool finished;
	bool collisioned;
	int userID;
	int scenarioNumber;
	int currentStep;

	WorldSimulation sim;
	WorldPlannerSettings settings;
	string initialState;

	string logFile;
	vector<SmartPointer<CommandInterface> > uis;
	int currentUI, oldUI;

	//goals
	Vector3* goals;
	int goalCount, currentGoal;
	Real zDiff, xDiff, yDiff;

	//GUI state
	GLUI* glui;
	GLUI_Listbox* ui_listbox;

	int drawDesired, drawPath, drawUI, drawContacts;

  vector<StatCollector> trackingErrors,numericalErrors;

	UserTrialProgram(RobotWorld* world, Vector3 goalArray[],
			int totalGoalCount, int UI, int user, int scene) :
		WorldViewProgram(world) {
		settings.InitializeDefault(*world);
		logFile = "trial" + int2str(scene) + ".log";
		goals = goalArray;
		goalCount = totalGoalCount;
		currentGoal = 0;
		currentUI = UI;
		userID = user;
		scenarioNumber = scene;
		finished = false;
		numberCollision = 0;
		accumTime = 0.0;
		maxCollisionF = 0.0;
		collisioned = false;
		xDiff = yDiff = zDiff = 0;
		currentStep = 1;
	}

	string int2str(int i) {
		stringstream temp;
		temp << i;
		return temp.str();
	}
	string double2str(double i) {
		stringstream temp;
		temp << i;
		return temp.str();
	}
	string real2str(Real i) {
		stringstream temp;
		temp << i;
		return temp.str();
	}

	void LogBegin(string parameters = "") {
		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "Begin. UserID: " << userID << ", Scenario Number: "
				<< scenarioNumber << ", UI Number:" << currentUI << endl;
		out.close();
	}

	void LogActivate(string result) {
		/*
		 ofstream out(logFile.c_str(), ios::out | ios::app);
		 out << "Activate " << uis[currentUI]->Name() << " " << sim.time << " "
		 << result << endl;
		 out.close();
		 */
	}

	void LogDeactivate(string result) {
		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "Deactivate " << uis[currentUI]->Name() << " " << sim.time
				<< " " << result << endl;
		out.close();
	}

	void LogUpdate(string result) {
		if (!result.empty()) {
			ofstream out(logFile.c_str(), ios::out | ios::app);
			out << "Update " << uis[currentUI]->Name() << " " << sim.time
					<< " " << result << endl;
			out.close();
		}
	}

  //KH: new temporary version for logging distance
	void LogConfig(Config q, Config dq, Real distance) {
		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "Q";
		out << "|ST:" << sim.time;
		out << "|RT:" << realClock.ElapsedTime()+accumTime;
		out << "|" << q << "|" << dq << "|" << distance << endl;
		out.close();
	}
  /*
	void LogConfig(Config q, Config dq) {
		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "Q";
		out << "|ST:" << sim.time;
		out << "|RT:" << realClock.ElapsedTime()+accumTime;
		out << "|" << q << "|" << dq << endl;
		out.close();
	}
  */

	void LogMouseDragInput(string button, int dx, int dy) {

		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "MD";
		out << "|ST:" << sim.time;
		out << "|RT:" << realClock.ElapsedTime()+accumTime;
		out << "|" << button << " Drag " << dx << " " << dy << endl;
		out.close();

	}

	void LogMouseInput(string result) {
		if (!result.empty()) {
			ofstream out(logFile.c_str(), ios::out | ios::app);
			out << "MD";
			out << "|ST:" << sim.time;
			out << "|RT:" << realClock.ElapsedTime()+accumTime;
			out << "|RIGHT_B " << result;
			out.close();
		}
	}

	void LogButtonPress(string key) {
		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "BP";
		out << "|ST:" << sim.time;
		out << "|RT:" << realClock.ElapsedTime()+accumTime;
		out << "|" << key << endl;
		out.close();
	}

	void LogKeypress(string key, int dx, int dy) {
		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "KB";
		out << "|ST:" << sim.time;
		out << "|RT:" << realClock.ElapsedTime()+accumTime;
		out << "|" << key << " " << dx << " " << dy << endl;
		out.close();

	}
	void LogStatus(string status) {
		ofstream out(logFile.c_str(), ios::out | ios::app);
		out << "STATUS:";
		out << "|ST:" << sim.time;
		out << "|RT:" << realClock.ElapsedTime()+accumTime;
		out << "|" << status << endl;
		out.close();
	}

	virtual bool Initialize() {
		drawDesired = 1;
		drawPath = 0;
		drawUI = 1;
		drawContacts = 1;

		sim.fakeSimulation = FAKE_SIMULATION;

		uis.resize(8);
		uis[0] = new JointCommandInterface;
		uis[1] = new IKCommandInterface;
		uis[2] = new PFCommandInterface;
		uis[3] = new IKPlannerCommandInterface;
		uis[4] = new RRTCommandInterface;
		uis[5] = new MTIKPlannerCommandInterface;
		uis[6] = new MTRRTCommandInterface;
		uis[7] = new MT_RRT_FTIECommandInterface;
		for (size_t i = 0; i < uis.size(); i++) {
			uis[i]->world = world;
			uis[i]->sim = &sim;
			uis[i]->viewport = &viewport;
			uis[i]->settings = &settings;
		}

		if(OVERRIDE_CONTROL) {
		  delete dynamic_cast<IKCommandInterface*>((CommandInterface*)uis[1])->inputProcessor;
		  delete dynamic_cast<PFCommandInterface*>((CommandInterface*)uis[2])->inputProcessor;
		  delete dynamic_cast<IKPlannerCommandInterface*>((CommandInterface*)uis[3])->inputProcessor;
		  delete dynamic_cast<RRTCommandInterface*>((CommandInterface*)uis[4])->inputProcessor;
		  StraightLineInputProcessor* processor = new StraightLineInputProcessor;
		  processor->goals = goals;
		  processor->currentGoal = &currentGoal;
		  dynamic_cast<IKCommandInterface*>((CommandInterface*)uis[1])->inputProcessor = processor;
		  processor = new StraightLineInputProcessor;
		  processor->goals = goals;
		  processor->currentGoal = &currentGoal;
		  dynamic_cast<PFCommandInterface*>((CommandInterface*)uis[2])->inputProcessor = processor;
		  processor = new StraightLineInputProcessor;
		  processor->goals = goals;
		  processor->currentGoal = &currentGoal;
		  dynamic_cast<IKPlannerCommandInterface*>((CommandInterface*)uis[3])->inputProcessor = processor;
		  processor = new StraightLineInputProcessor;
		  processor->goals = goals;
		  processor->currentGoal = &currentGoal;
		  dynamic_cast<RRTCommandInterface*>((CommandInterface*)uis[4])->inputProcessor = processor;
		}


		simulate = 0;

		//world-object
		for (size_t i = 0; i < world->rigidObjects.size(); i++)
			sim.EnableContactFeedback(world->RigidObjectID(i),
					world->TerrainID(0));
		//robot-object
		for (size_t i = 0; i < world->rigidObjects.size(); i++)
			sim.EnableContactFeedback(world->RigidObjectID(i),
					world->RobotID(0));

		//robot-world
		for (size_t i = 0; i < world->terrains.size(); i++)
			sim.EnableContactFeedback(world->TerrainID(i), world->RobotID(0));

		printf("Writing initial state\n");
		sim.WriteState(initialState);
		printf("Done.\n");

		if (!WorldViewProgram::Initialize())
			return false;

		glui = GLUI_Master.create_glui_subwindow(main_window,
				GLUI_SUBWINDOW_RIGHT);
		glui->set_main_gfx_window(main_window);
		glui->add_button("Simulate", SIMULATE_BUTTON_ID, ControlFunc);
		glui->add_button("Reset", RESET_BUTTON_ID, ControlFunc);
		glui->add_checkbox("Show destination config", &drawDesired);

		if(currentUI < 0) {
		  currentUI = 0;
		  oldUI = 0;
		  ui_listbox = glui->add_listbox("UI",&currentUI,UI_LISTBOX_ID,ControlFunc);
		  for(size_t i=0;i<uis.size();i++) {
		    char buf[256];
		    strcpy(buf,uis[i]->Description().c_str());
		    ui_listbox->add_item(i,buf);
		  }
		}

		//activate current UI
		string res = uis[currentUI]->ActivateEvent(true);
		LogActivate(res);

		printf("Done initializing...\n");
		return true;
	}

	virtual void RenderWorld() {
		Robot* robot = world->robots[0].robot;
		RobotController* rc = sim.robotControllers[0];
		MyController* c = GetController(rc);

		glEnable( GL_BLEND);
		glEnable( GL_LIGHTING);

		if(drawDesired) {
		  //draw the ODE bodies in the right colors
		  for (size_t i = 0; i < world->robots.size(); i++) {
		    for (size_t j = 0; j < world->robots[i].robot->links.size(); j++) {
		      RigidTransform T;
		      sim.odesim.robot(i)->GetLinkTransform(j, T);
		      glPushMatrix();
		      glMultMatrix(Matrix4(T));
		      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,
				   world->robots[i].view.colors[j].rgba);
		      world->robots[i].view.DrawLink_Local(j);
		      glPopMatrix();
		    }
		  }

		  //draw current commanded configuration -- transparent
		  vector<GLColor> oldColors = world->robots[0].view.colors;
		  GLColor newColor(1, 1, 0, 0.5);
		  world->robots[0].view.SetColors(newColor);
		  const Config& curBest = c->Endpoint();
		  robot->UpdateConfig(curBest);
		  WorldViewProgram::RenderWorld();
		  world->robots[0].view.colors = oldColors;
		}
		else {
		  sim.UpdateModel();
		  WorldViewProgram::RenderWorld();
		}

		//draw goals
		if (currentGoal < goalCount) {
			float goalColor[4] = {0, 255, 0, 1.0};
			glPushMatrix();
			glTranslated(goals[currentGoal].x,goals[currentGoal].y,goals[currentGoal].z);
			glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,goalColor);
			drawSphere(0.05,10,10);
			glPopMatrix();
		}
		/*
		if (drawDesired) {
			const Config& curBest = c->Endpoint();
			robot->UpdateConfig(curBest);
			for (size_t j = 0; j < robot->links.size(); j++) {
				glPushMatrix();
				glMultMatrix(Matrix4(robot->links[j].T_World));
				float color[4] = { 1, 1, 0, 0.5 };
				glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
				world->robots[0].view.DrawLink_Local(j);
				glPopMatrix();
			}
		}
		*/
		if (drawUI) {
			uis[currentUI]->DrawGL();
		}

		//draw desired path
		if (drawPath) {
			RobotController* rc = sim.robotControllers[0];
			for (size_t i = 0; i < robot->drivers.size(); i++) {
				robot->SetDriverValue(i, rc->command->actuators[i].qdes);
			}
			robot->UpdateFrames();
			for (size_t j = 0; j < robot->links.size(); j++) {
				glPushMatrix();
				glMultMatrix(Matrix4(robot->links[j].T_World));
				float color[4] = { 0, 1, 0, 0.5 };
				glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color);
				world->robots[0].view.DrawLink_Local(j);
				glPopMatrix();
			}
		}

		
		//draw collision feedback
		glDisable(GL_LIGHTING);
		glDisable(GL_BLEND);
		glEnable( GL_POINT_SMOOTH);
		if (drawContacts) {
			for (WorldSimulation::ContactFeedbackMap::iterator i = sim.contactFeedback.begin(); i != sim.contactFeedback.end(); i++) {
				ODEContactList* c = sim.odesim.GetContactFeedback(
						i->first.first,
						i->first.second);
				Assert(c != NULL);

				glColor3f(1, 1, 0);
				glPointSize(5.0);
				glBegin(GL_POINTS);
				for (size_t j = 0; j < c->points.size(); j++) {
					glVertex3v(c->points[j].x);
					LogStatus("CL:" + real2str(c->points[j].x.x) + " "
							+ real2str(c->points[j].x.y) + " " + real2str(
							c->points[j].x.z));//+" "+real2str(c->points[j].n.x)+" "+real2str(c->points[j].n.y)+" "+real2str(c->points[j].n.z));
				}
				glEnd();
				Real scale = 0.1;
				glBegin( GL_LINES);
				for (size_t j = 0; j < c->points.size(); j++) {
					glVertex3v(c->points[j].x);
					glVertex3v(c->points[j].x + scale * c->points[j].n);
				}
				glEnd();
				glColor3f(1, 0.5, 0);
				Assert(c->forces.size() <= c->points.size());
				Real fscale = 0.1;
				glBegin(GL_LINES);
				for (size_t j = 0; j < c->forces.size(); j++) {
					glVertex3v(c->points[j].x);
					glVertex3v(c->points[j].x + fscale * c->forces[j]);
					if (maxCollisionF < c->forces[j].norm())
						maxCollisionF = c->forces[j].norm();
				}
				glEnd();
			}

		}
	}

	virtual void RenderScreen() {
		if(SHOW_HUD==0) return;

		void* fontface = GLUT_BITMAP_HELVETICA_18;
		int lineSpacing = 24;
		string line;
		int x, y;

		glDisable( GL_LIGHTING);
		glDisable( GL_DEPTH_TEST);

		x = 20;
		y = 34;

		if (!simulate) {
			glColor3f(1, 1, 0);
			glRasterPos2i(x, y);
			glutBitmapString(fontface, "Press Simulate whenever you are ready\n");
		} else {
			glColor3f(1, 1, 1);
			glRasterPos2i(x, y);
			if(scenarioNumber == 0 && currentStep <=4) {
				glutBitmapString(fontface, uis[currentUI]->Instructions().c_str());
				y += lineSpacing;
				glRasterPos2i(x, y);
				switch(currentStep) {
				case 1:
					line = "First let's learn how to rotate the camera.";
					glutBitmapString(fontface,line.c_str());
					y += lineSpacing;
					glRasterPos2i(x, y);
					line = "Left-click the mouse and drag the screen to rotate the camera.";
					glutBitmapString(fontface,line.c_str());
					break;
				case 2:
					line = "Great! Now let's learn how to move the camera.";
					glutBitmapString(fontface,line.c_str());
					y += lineSpacing;
					glRasterPos2i(x, y);
					line = "Press and hold the 'CTRL' key on the keyboard and then use the mouse";
					glutBitmapString(fontface,line.c_str());
					y += lineSpacing;
					glRasterPos2i(x, y);
					line ="to drag the screen to move the camera.";
					glutBitmapString(fontface,line.c_str());
					break;
				case 3:
					line = "Great! Now let's learn how to zoom in and zoom out.";
					glutBitmapString(fontface,line.c_str());
					y += lineSpacing;
					glRasterPos2i(x, y);
					line = "Press and hold the 'SHIFT' key on the keyboard and then use the mouse";
					glutBitmapString(fontface,line.c_str());
					y += lineSpacing;
					glRasterPos2i(x, y);
					line = "to drag the screen towards the top to ZOOM IN,and drag the screen";
					glutBitmapString(fontface,line.c_str());
					y += lineSpacing;
					glRasterPos2i(x, y);
					line = "towards the bottom to ZOOM out.";
					glutBitmapString(fontface,line.c_str());
					break;
				case 4:
					line = "Great! Now let's try reach the goal by following the above instruction.";
					glutBitmapString(fontface,line.c_str());
					y += lineSpacing;
					glRasterPos2i(x, y);
					line = "The goal is the ball marked in green. This tutorial instruction will be gone soon.";
					glutBitmapString(fontface,line.c_str());					
					if (timer.ElapsedTime()-tutorialFinishTime>5000) {
						currentStep++;
					}
				}
			}else if(!finished) {
				glutBitmapString(fontface, uis[currentUI]->Instructions().c_str());
				y += lineSpacing;
				glRasterPos2i(x, y);
				line = "Time: " + int2str((int)realClock.ElapsedTime()+accumTime)+"s";
				glutBitmapString(fontface,line.c_str());
				y += lineSpacing;
				glRasterPos2i(x, y);
				line = "Number of collisions: "+int2str(numberCollision);
				if(collisioned)
				  line += " (colliding)";
				glutBitmapString(fontface, line.c_str());
				y += lineSpacing;
				glRasterPos2i(x, y);
				line = "Max collision force: " + double2str(maxCollisionF);
				glutBitmapString(fontface, line.c_str());
				y += lineSpacing;
				glRasterPos2i(x, y);
				line = "Number of goals left:" + int2str(goalCount - currentGoal);
				glutBitmapString(fontface, line.c_str());
				y = 450;
				if(SAVE_MOVIE) y=350;
				lineSpacing *= 1.5;
				glRasterPos2i(x, y);
				glColor3f(0, 0, 0);
				glPointSize(5.0);
				glLineWidth(1.5);
				glBegin(GL_LINES);
				glVertex2f(x+250,y);
				glVertex2f(x+400,y);
				glVertex2f(x+250,y);
				glVertex2f(x+250,y-6);
				glVertex2f(x+400,y);
				glVertex2f(x+400,y-6);
				glVertex2f(x+325,y);
				glVertex2f(x+325,y-6);
				y += lineSpacing;
				glVertex2f(x+250,y);
				glVertex2f(x+400,y);
				glVertex2f(x+250,y);
				glVertex2f(x+250,y-6);
				glVertex2f(x+400,y);
				glVertex2f(x+400,y-6);
				glVertex2f(x+325,y);
				glVertex2f(x+325,y-6);
				y += lineSpacing;
				glVertex2f(x+250,y);
				glVertex2f(x+400,y);
				glVertex2f(x+250,y);
				glVertex2f(x+250,y-6);
				glVertex2f(x+400,y);
				glVertex2f(x+400,y-6);
				glVertex2f(x+325,y);
				glVertex2f(x+325,y-6);
				glEnd();
				y = 450;
				if(SAVE_MOVIE) y=350;
				glBegin(GL_POINTS);
				glColor3f(1, 0, 0);
				glVertex2f(x+325+xDiff*25, y);
				y += lineSpacing;
				glVertex2f(x+325+zDiff*25, y);
				y += lineSpacing;
				glVertex2f(x+325+yDiff*25, y);
				glEnd();
				glColor3f(1, 1, 1);
				y = 450;
				if(SAVE_MOVIE) y=350;
				glRasterPos2i(x, y);
				line = "Distance to goal on X-axis: ";
				glutBitmapString(fontface, line.c_str());
				glRasterPos2i(x+240, y-16);
				glutBitmapInt(fontface, -3.0);
				glRasterPos2i(x+320, y-16);
				glutBitmapInt(fontface, 0.0);
				glRasterPos2i(x+390, y-16);
				glutBitmapInt(fontface, 3.0);
				y += lineSpacing;
				glRasterPos2i(x, y);
				line = "Distance to goal on Y-axis: ";
				glutBitmapString(fontface, line.c_str());
				glRasterPos2i(x+240, y-16);
				glutBitmapInt(fontface, -3.0);
				glRasterPos2i(x+320, y-16);
				glutBitmapInt(fontface, 0.0);
				glRasterPos2i(x+390, y-16);
				glutBitmapInt(fontface, 3.0);
				y += lineSpacing;
				glRasterPos2i(x, y);
				line = "Distance to goal on Z-axis: ";
				glutBitmapString(fontface, line.c_str());
				glRasterPos2i(x+240, y-16);
				glutBitmapInt(fontface, -3.0);
				glRasterPos2i(x+320, y-16);
				glutBitmapInt(fontface, 0.0);
				glRasterPos2i(x+390, y-16);
				glutBitmapInt(fontface, 3.0);
			} else {
				glRasterPos2i(x, y);
				line = "Congratulations! You have reached all the goals!";
				glutBitmapString(fontface, line.c_str());
				y += lineSpacing;
				glRasterPos2i(x, y);
				int rem = 5 - int(realClock.ElapsedTime()+accumTime - finishTime);
				line = "This program will terminate in: "+ int2str(rem)+" seconds";
				glutBitmapString(fontface, line.c_str());
				if (rem <= 0)
					exit(0);
			}
		}

		glEnable(GL_DEPTH_TEST);
	}

	virtual void Handle_Control(int id) {
		switch (id) {
		case SIMULATE_BUTTON_ID:
			LogButtonPress("SIMULATE_BT");
			if (simulate) {
				simulate = 0;
				accumTime += realClock.ElapsedTime();
				if(SAVE_MOVIE) StopMovie();
				SleepIdleCallback();
			} else {
				simulate = 1;
				realClock.Reset();
				if(SAVE_MOVIE) StartMovie();
				SleepIdleCallback(0);
			}
			break;
		case RESET_BUTTON_ID:
			LogButtonPress("RESET_BT");
			simulate = 0;
			collisioned = false;
			numberCollision = 0;
			maxCollisionF = 0.0;
			if (!sim.ReadState(initialState)) {
				fprintf(stderr, "Warning, ReadState doesn't work\n");
			}
			break;
		case UI_LISTBOX_ID: {
			string res = uis[oldUI]->ActivateEvent(false);
			LogDeactivate(res);
			res = uis[currentUI]->ActivateEvent(true);
			LogActivate(res);
			oldUI = currentUI;
		}
			break;
		}
	}

	void BeginDrag(int x, int y, int button, int modifiers) {
		if (button == GLUT_RIGHT_BUTTON) {
		} else {
			WorldViewProgram::BeginDrag(x, y, button, modifiers);
		}
	}

	void EndDrag(int x, int y, int button, int modifiers) {
		if (button == GLUT_RIGHT_BUTTON) {
			string res = uis[currentUI]->MouseInputEvent(0, 0, true);
			LogMouseInput(res);
		}
	}

	virtual void DoFreeDrag(int dx, int dy, int button) {
		if (button == GLUT_LEFT_BUTTON) {
			DragRotate(dx, dy);
			LogMouseDragInput("LEFT_B", dx, dy);
			if (currentStep==1 && scenarioNumber == 0) {
				currentStep++;
			}
		} else if (button == GLUT_RIGHT_BUTTON) {
			string res = uis[currentUI]->MouseInputEvent(dx, dy, true);
			LogMouseInput(res);
		}
	}

	virtual void DoCtrlDrag(int dx, int dy, int button) {
		if (button == GLUT_LEFT_BUTTON) {
			DragPan(dx, dy);
			LogKeypress(string("CTRL"), dx, dy);
			if (currentStep==2 && scenarioNumber == 0) {
				currentStep++;
			}
		}
	}

	virtual void DoAltDrag(int dx, int dy, int button) {
		if (button == GLUT_LEFT_BUTTON) {
			DragZoom(dx, dy);
			if (currentStep==3 && scenarioNumber == 0) {
				currentStep++;
				tutorialFinishTime = realClock.ElapsedTime()+accumTime;
			}
			LogKeypress(string("ALT"), dx, dy);
		}
	}

	virtual void DoShiftDrag(int dx, int dy, int button) {
		if (button == GLUT_LEFT_BUTTON) {
			camera.dist *= (1 + 0.01 * Real(dy));
			if (currentStep==3 && scenarioNumber == 0) {
				currentStep++;
				tutorialFinishTime = realClock.ElapsedTime()+accumTime;
			}
			LogKeypress(string("SHIFT"), dx, dy);
		}
	}

	virtual void Handle_Motion(int x, int y) {
		string res = uis[currentUI]->MouseInputEvent(x, y, false);
		LogMouseInput(res);
		Refresh();
	}

	virtual void Handle_Keypress(unsigned char key, int x, int y) {
#if DO_TIMING
	  if (key == 't') {
	    for(size_t i=0;i<uis.size();i++) {
	      cout<<uis[i]->Name()<<": "; uis[i]->timingStats.Print(cout);
	      cout<<endl;
	    }
	    getchar();
	  }
#endif
#if DO_WORKSPACE_ERRORS
	  if (key == 'e') {
	    cout<<"Numerical errors:"<<endl;
	    for(size_t i=0;i<numericalErrors.size();i++) {
	      numericalErrors[i].Print(cout); cout<<endl;
	    }
	    cout<<"Tracking errors:"<<endl;
	    for(size_t i=0;i<trackingErrors.size();i++) {
	      trackingErrors[i].Print(cout); cout<<endl;
	    }
	    getchar();
	  }
#endif
		//string res = uis[currentUI]->KeypressEvent(key, x, y);
		//LogKeypress();
		//cout<<key<<endl;
		//printf("%c",key);
		Refresh();
	}

  virtual void Handle_Reshape(int w,int h) {
    WorldViewProgram::Handle_Reshape(w,h);
    if(SAVE_MOVIE) {
      static bool resize=true;
      if(resize) {
	int totalw = glutGet(GLUT_WINDOW_WIDTH);
	int totalh = glutGet(GLUT_WINDOW_HEIGHT);
	printf("Window %d x %d, viewport %d x %d\n",totalw,totalh,viewport.w,viewport.h); 
	int toolbarw = totalw - viewport.w;
	int toolbarh = totalh - viewport.h;
	glutReshapeWindow(toolbarw+640,toolbarh+480);
	resize=false;
      }
    }
  }

  void HandleGameLogic()
  {
    Robot* robot = world->robots[0].robot;
    RobotController* rc = sim.robotControllers[0];
    MyController* c = GetController(rc);

    //KH
    sim.UpdateRobot(0);

    //test collisions
    bool hasCollision = false;
    //Check joint limits
    if(!robot->InJointLimits(robot->q)) {
      hasCollision = true;
      numberCollision ++;
    }
    for (WorldSimulation::ContactFeedbackMap::iterator i = sim.contactFeedback.begin(); i != sim.contactFeedback.end(); i++) {
      ODEContactList* c = sim.odesim.GetContactFeedback(
							i->first.first,
							i->first.second);
      Assert(c != NULL);
      //count the number of colliding pairs
      if(!c->points.empty()) {
	numberCollision++;
	hasCollision = true;
      }
    }
    if (!collisioned && hasCollision) {
      collisioned = true;
    } else if (!hasCollision) {
      collisioned = false;
    }

    Vector3 local, worldP;
    local.setZero();
    robot->GetWorldPosition(local,
			    world->robots[0].robot->q.n - 1, worldP);
    Real distance = worldP.distance(goals[currentGoal]);
    LogConfig(c->xcur,c->dxcur,distance);
    xDiff = worldP.x - goals[currentGoal].x;
    xDiff = xDiff > 3?3:xDiff;
    yDiff = worldP.y - goals[currentGoal].y;
    yDiff = yDiff > 3?3:yDiff;
    zDiff = worldP.z - goals[currentGoal].z;
    zDiff = zDiff > 3?3:zDiff;
    //KH: added collision free test
    if (distance <= 0.1 && !collisioned) {
      currentGoal++;
      //figure out a way to record the status of the return
      if (currentGoal == goalCount) {
	LogStatus("Finished");
	finished = true;
	finishTime = realClock.ElapsedTime()+accumTime;
      }
    }
  }

	virtual void Handle_Idle() {
		if (simulate) {
			Timer timer;
			string res = uis[currentUI]->UpdateEvent();
			//LogUpdate(res);

			sim.Advance(dt);
			Refresh();

			HandleGameLogic();

			if(DO_WORKSPACE_ERRORS) {
			  Robot* robot=world->robots[0].robot;
			  ODERobot* oderobot=sim.odesim.robot(0);
			  RobotController* rc = sim.robotControllers[0];
			  MyController* c = GetController(rc);

			  vector<RigidTransform> Tode(robot->links.size());
			  vector<RigidTransform> Tactual(robot->links.size());
			  static vector<list<RigidTransform> > Tactual_trace(robot->links.size());
			  vector<RigidTransform> Tdesired(robot->links.size());
			  trackingErrors.resize(robot->links.size());
			  numericalErrors.resize(robot->links.size());

			  for(size_t i=0;i<robot->links.size();i++) 
			    oderobot->GetLinkTransform(i,Tode[i]);
			  Config qode;
			  oderobot->GetConfig(qode);
			  robot->UpdateConfig(qode);
			  for(size_t i=0;i<robot->links.size();i++) 
			    Tactual[i] = robot->links[i].T_World;
			  robot->UpdateConfig(c->xcur); //current desired
			  for(size_t i=0;i<robot->links.size();i++) 
			    Tdesired[i] = robot->links[i].T_World;
			  for(size_t i=0;i<robot->links.size();i++) {
			    Tactual_trace[i].push_back(Tactual[i]);
			    if(Tactual_trace[i].size() > 20)
			      Tactual_trace[i].erase(Tactual_trace[i].begin());
			  }

			  //compute numerical errors
			  for(size_t i=0;i<robot->links.size();i++) {
			    numericalErrors[i].collect(Tactual[i].t.distance(Tode[i].t));
			    /*
			    Matrix3 Rerr;
			    Rerr.mulTransposeB(Tactual[i].R,Tode[i].R);
			    Real d=MatrixAbsoluteAngle(Rerr);

			    //TODO: get geometry bound
			    */
			  }

			  //compute tracking errors
			  for(size_t i=0;i<robot->links.size();i++) {
			    Real mindist = Inf;
			    for(list<RigidTransform>::iterator j=Tactual_trace[i].begin();j!=Tactual_trace[i].end();j++) {
			      list<RigidTransform>::iterator n=j; ++n;
			      if(n != Tactual_trace[i].end()) {
				Segment3D s;
				s.a = j->t;
				s.b = n->t;
				Vector3 cp;
				s.closestPoint(Tdesired[i].t,cp);
				mindist = Min(mindist,Tdesired[i].t.distance(cp));
			      }
			      else
				mindist = Min(mindist,Tdesired[i].t.distance(j->t));
			    }
			    trackingErrors[i].collect(mindist);
			    /*
			    Matrix3 Rerr;
			    Rerr.mulTransposeB(Tactual[i].R,Tdesired[i].R);
			    Real d=MatrixAbsoluteAngle(Rerr);
			    //TODO: get geometry bound
			    */
			  }
			}

			//MovieUpdate(sim.time);
			//real time movie
			if(simulate) {
			  accumTime += realClock.ElapsedTime();
			  MovieUpdate(accumTime);
			  realClock.Reset(); //exclude time saving the movie
			}
			if(!saveMovie)
			  SleepIdleCallback(int(Max(0.0, dt - timer.ElapsedTime()) * 1000.0));
		}
		WorldViewProgram::Handle_Idle();
	}
};

int main(int argc, char** argv) {
	if (argc < 2) {
		printf("USAGE: UserTrials XML_file [UI] [userID] [sceneNumber]\n");
		return 0;
	}
	RobotWorld world;
	int UI=-1, userID=1, sceneNumber=1;
	world.lights.resize(1);
	world.lights[0].setColor(GLColor(1, 1, 1));
	world.lights[0].setDirectionalLight(Vector3(0.2, -0.4, 1));
	world.lights[0].setColor(GLColor(1, 1, 1));

	XmlWorld xmlWorld;
	char* logFile = NULL;

	const char* ext = FileExtension(argv[1]);
	if (0 == strcmp(ext, "xml")) {
		if (!xmlWorld.Load(argv[1])) {
			printf("Error loading world file %s\n", argv[1]);
			return 1;
		}
		if (!xmlWorld.GetWorld(world)) {
			printf("Error loading world from %s\n", argv[1]);
			return 1;
		}
	}
	if(argc > 2)
	  UI = atoi(argv[2]);
	if(argc > 3)
	  userID = atoi(argv[3]);
	if(argc > 4)
	  sceneNumber = atoi(argv[4]);
	UserTrialProgram program(&world, xmlWorld.goals, xmlWorld.goalCount, UI,
			userID, sceneNumber);
	if (logFile)
		program.logFile = logFile;
	program.sim.Init(program.world);
	//setup controllers
	program.sim.robotControllers.resize(world.robots.size());
	for (size_t i = 0; i < program.sim.robotControllers.size(); i++) {
		Robot* robot = world.robots[i].robot;
		MilestonePathController* c = new MilestonePathController(*robot);
		c->modifySpeedByError = false;

		//program.sim.robotControllers[i] = c;
		LoggingController<FeedforwardController>* fc = new LoggingController<
				FeedforwardController> (*robot);
		fc->base = c;
		fc->save = false;

		program.sim.SetController(i, fc);
		program.sim.controlSimulators[i].sensors.hasJointPosition = true;
		program.sim.controlSimulators[i].sensors.hasJointVelocity = true;
	}

	fill(program.settings.robotSettings[0].collisionMargin.begin(),program.settings.robotSettings[0].collisionMargin.end(),0.005);
	/*
	//DEBUG COLLISION TESTS for RSS paper
	char buf[256];
	sprintf(buf,"RSS_logs/collconf%d.txt",sceneNumber);
	ifstream in(buf);
	int numConfigs = 0;
	int numColl = 0;
	int numCollExp = 0;
	int numCollExp2 = 0;
	Config q;
	while(in >> q) {
	  world.robots[0].robot->UpdateConfig(q);
	  world.robots[0].robot->UpdateGeometry();
	  numConfigs++;
	  fill(program.settings.robotSettings[0].collisionMargin.begin(),program.settings.robotSettings[0].collisionMargin.end(),0);
	  if(program.settings.CheckCollision(world,world.RobotID(0)))
	    numColl++;
	  fill(program.settings.robotSettings[0].collisionMargin.begin(),program.settings.robotSettings[0].collisionMargin.end(),0.002);
	  if(program.settings.CheckCollision(world,world.RobotID(0)))
	    numCollExp++;
	  fill(program.settings.robotSettings[0].collisionMargin.begin(),program.settings.robotSettings[0].collisionMargin.end(),0.001);
	  if(program.settings.CheckCollision(world,world.RobotID(0)))
	    numCollExp2++;
	}
	in.close();
	printf("%d/%d colliding, %d exp, %d exp2\n",numColl,numConfigs,numCollExp,numCollExp2);
	return 0;
	*/

	stringstream params;
	for (int i = 1; i < argc; i++)
		params << argv[i] << " ";
	program.LogBegin(params.str());
	return program.Run();
}
