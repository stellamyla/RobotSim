#include "Ramp.h"
#include <iostream>
#include <math/math.h>
#include <math/misc.h>
#include <errors.h>
using namespace std;


//tolerance for time
const static Real EpsilonT = 1e-5;
//tolerance for position
const static Real EpsilonX = 1e-5;
//tolerance for velocity
const static Real EpsilonV = 1e-6;
//tolerance for acceleration
const static Real EpsilonA = 1e-5;

//tolerance for pausing
const static Real gPauseEpsilon = 1e-4;
//a flag used during testing of failed ramps
static bool suppressSavingRamps = false;
//set to true to enable error reporting
static bool gDebugPrint = false;

bool SaveRamp(const char* fn,Real x0,Real dx0,Real x1,Real dx1,
	      Real a,Real v,Real t)
{
  if(suppressSavingRamps) return true;
  FILE* f=fopen(fn,"ab");
  if(!f) {
    f = fopen(fn,"wb");
    if(!f) {
      printf("Unable to open file %s for saving\n",fn);
      return false;
    }
  }
  double vals[7]={x0,dx0,x1,dx1,a,v,t};
  fwrite(vals,sizeof(double),7,f);
  fclose(f);
  return true;
}

bool LoadRamp(FILE* f,Real& x0,Real& dx0,Real& x1,Real& dx1,
	      Real& a,Real& v,Real& t)
{
  double vals[7];
  int size = fread(vals,sizeof(double),7,f);
  if(size != 7) return false;
  x0=vals[0];  dx0=vals[1];
  x1=vals[2];  dx1=vals[3];
  a=vals[4];  v=vals[5];
  t=vals[6]; 
  return true;
}


bool LoadRamp(const char* fn,Real& x0,Real& dx0,Real& x1,Real& dx1,
	      Real& a,Real& v,Real& t)
{
  FILE* f=fopen(fn,"rb");
  if(!f) return false;
  bool res=LoadRamp(f,x0,dx0,x1,dx1,a,v,t);
  fclose(f);
  return res;
}

void TestRamps(const char* fn)
{
  FILE* f=fopen(fn,"rb");
  if(!f) return;

  suppressSavingRamps=true;
  ParabolicRamp1D ramp;
  Real a,v,t;
  int numRamps=0;
  while(LoadRamp(f,ramp.x0,ramp.dx0,ramp.x1,ramp.dx1,a,v,t)) {
    if(t < 0) {
      assert( a >= 0 && v >= 0);
      t = ramp.SolveMinTime(a,v);
      printf("Result: t=%g\n",ramp.ttotal);
    }
    else if(a < 0) {
      assert( t >= 0 && v >= 0);
      a = ramp.SolveMinAccel(t,v);
      printf("Result: a=%g\n",ramp.a1);
    }
    else {
      printf("Ramp fully specified\n");
    }
    numRamps++;
  }
  fclose(f);
  printf("%d ramps loaded from file %s\n",numRamps,fn);
  suppressSavingRamps=false;
}

class ParabolicRamp
{
 public:
  Real Evaluate(Real t) const;
  Real Derivative(Real t) const;
  Real Accel(Real t) const;
  bool Solve(Real amax);
  Real MaxVelocity() const;

  //input
  Real x0,dx0;
  Real x1,dx1;

  //calculated
  Real a;
  Real ttotal;
};


class PPRamp
{
 public:
  Real Evaluate(Real t) const;
  Real Derivative(Real t) const;
  Real Accel(Real t) const;
  bool SolveMinTime(Real amax);
  bool SolveMinAccel(Real endTime);
  Real MaxVelocity() const;

  Real CalcTotalTime(Real a) const;
  Real CalcSwitchTime(Real a) const;
  Real CalcMinAccel(Real endTime,Real sign,Real& switchTime) const;

  //input
  Real x0,dx0;
  Real x1,dx1;

  //calculated
  Real a;
  Real tswitch,ttotal;
};

class PLPRamp
{
 public:
  Real Evaluate(Real t) const;
  Real Derivative(Real t) const;
  Real Accel(Real t) const;
  bool SolveMinTime(Real amax,Real vmax);
  bool SolveMinAccel(Real endTime,Real vmax);

  Real CalcTotalTime(Real a,Real v) const;
  Real CalcSwitchTime1(Real a,Real v) const;
  Real CalcSwitchTime2(Real a,Real v) const;
  Real CalcMinAccel(Real endTime,Real v) const;

  //input
  Real x0,dx0;
  Real x1,dx1;

  //calculated
  Real a,v;
  Real tswitch1,tswitch2,ttotal;
};



Real ParabolicRamp::Evaluate(Real t) const
{
  return x0 + t*dx0 + 0.5*a*Sqr(t);
}

Real ParabolicRamp::Derivative(Real t) const
{
  return dx0 + a*t;
}

Real ParabolicRamp::Accel(Real t) const
{
  return a;
}

bool ParabolicRamp::Solve(Real amax)
{
  if(FuzzyEquals(x0,x1,EpsilonX)) {
    if(FuzzyEquals(dx0,dx1,EpsilonV)) {
      a=0;
      ttotal = 0;
      return true;
    }
    else if(FuzzyEquals(dx0,-dx1,EpsilonV)) { //pure parabola, any acceleration works
      a=amax*Sign(dx1);
      ttotal = (dx1-dx0)/a;
      return true;
    }
    //no parabola will work
    return false;
  }
  a = 0.5*(Sqr(dx0)-Sqr(dx1))/(x0-x1);
  //pick the denominator less likely to result in numerical errors
  if(Abs(a) < Abs(dx0+dx1)) { 
    if(Abs(dx0+dx1) < EpsilonV) {
      //danger of numerical errors
      //dx0 = - dx1
      //need x0 = x1
      return false;
    }
    else {
      ttotal = 2.0*(x1-x0)/(dx0+dx1);
    }
  }
  else {
    ttotal = (dx1-dx0)/a;
  }
  if(ttotal < 0 && ttotal > -EpsilonT) ttotal = 0;
  if(ttotal < 0) {
    ttotal = -1;
    a=0;
    return false;
  }

  //check for numerical errors
  if(Abs(a) > amax && Abs(a) <= amax+EpsilonA) {
    //double check if the capped version works
    a = Sign(a)*amax;
  }
  if(FuzzyEquals(Evaluate(ttotal),x1,EpsilonX) && FuzzyEquals(Derivative(ttotal),dx1,EpsilonV)) {
    return true;
  }
  return false;
}

Real ParabolicRamp::MaxVelocity() const
{
  if(fabs(dx0) > fabs(dx1)) return dx0;
  else return dx1;
}

Real PPRamp::Evaluate(Real t) const
{
  if(t < tswitch) return x0 + 0.5*a*t*t + dx0*t;
  else {
    Real tmT = t - ttotal;
    return x1 - 0.5*a*tmT*tmT + dx1*tmT;
  }
}

Real PPRamp::Derivative(Real t) const
{
  if(t < tswitch) return a*t + dx0;
  else {
    Real tmT = t - ttotal;
    return -a*tmT + dx1;
  }
}

Real PPRamp::Accel(Real t) const
{
  if(t < tswitch) return a;
  else return -a;
}

bool PPRamp::SolveMinTime(Real amax)
{
  Real tpn = CalcTotalTime(amax), tnp = CalcTotalTime(-amax);
  if(gDebugPrint) cout<<"Time for parabola +-: "<<tpn<<", parabola -+: "<<tnp<<endl;
  if(tpn >= 0) {
    if(tnp >= 0 && tnp < tpn) {
      a = -amax;
      ttotal = tnp;
    }
    else {
      a = amax;
      ttotal = tpn;
    }
  }
  else if(tnp >= 0) {
    a = -amax;
    ttotal = tnp;
  }
  else {
    tswitch = -1;
    ttotal = -1;
    a = 0;
    return false;
  }
  tswitch = CalcSwitchTime(a);
  //uncomment for additional debugging
  if(!FuzzyEquals(x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),EpsilonX)) {
    printf("Error computing parabolic-parabolic min-time...\n");
    printf("x0=%g,%g, x1=%g,%g\n",x0,dx0,x1,dx1);
    printf("a = %g, tswitch = %g, ttotal = %g\n",a,tswitch,ttotal);
    printf("Forward %g, backward %g, diff %g\n",x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal), x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch - (x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal)));
    //Real b = 2.0*dx0; //2.0*(dx0-dx1);
    //Real c = (Sqr(dx0)-Sqr(dx1))*0.5/a+x0-x1;
    Real b = 2.0*a*dx0; //2.0*(dx0-dx1);
    Real c = (Sqr(dx0)-Sqr(dx1))*0.5+(x0-x1)*a;
    Real t1,t2;
    int res=quadratic(a*a,b,c,t1,t2);
    printf("Quadratic equation %g x^2 + %g x + %g = 0\n",a*a,b,c);
    printf("%d results, %g %g\n",res,t1,t2);
    if(!FuzzyEquals(x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),gPauseEpsilon))
      getchar();
    printf("Saving to PP_SolveMinTime_failure.dat\n");
    SaveRamp("PP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,Inf,-1);
    return false;
  }
  assert(FuzzyEquals(x0 + 0.5*a*Sqr(tswitch) + dx0*tswitch,x1 - 0.5*a*Sqr(tswitch-ttotal) + dx1*(tswitch-ttotal),EpsilonX));

 
  return true;
}

bool PPRamp::SolveMinAccel(Real endTime)
{
  Real switch1,switch2;
  Real apn = CalcMinAccel(endTime,1.0,switch1);
  Real anp = CalcMinAccel(endTime,-1.0,switch2);
  if(gDebugPrint) cout<<"Accel for parabola +-: "<<apn<<", parabola -+: "<<anp<<endl;
  if(apn >= 0) {
    if(anp >= 0 && anp < apn)  a = -anp;
    else a = apn;
  }
  else if(anp >= 0) a = -anp;
  else {
    a=0;
    tswitch = -1;
    ttotal = -1;
    return false;
  }
  ttotal = endTime;
  if(a == apn) 
    tswitch = switch1;
  else
    tswitch = switch2;

  //debug
  Real t2mT = tswitch-ttotal;
  if(!FuzzyEquals(x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),EpsilonX)) {
    printf("PPRamp: Error solving min-accel!\n");
    printf("Forward ramp: %g, backward %g, diff %g\n",x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch)-(x1+t2mT*dx1-0.5*a*Sqr(t2mT)));
    printf("A+ = %g, A- = %g\n",apn,anp);
    printf("ramp %g,%g -> %g, %g\n",x0,dx0,x1,dx1);
    printf("Switch 1 %g, switch 2 %g, total %g\n",switch1,switch2,ttotal);

    {
      Real sign = 1.0;
      Real a=Sqr(endTime);
      Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
      Real c=-Sqr(dx1-dx0);
      printf("Quadratic %g x^2 + %g x + %g = 0\n",a,b,c);
      Real t1,t2;
      int res = quadratic(a,b,c,t1,t2);
      printf("Solutions: %d, %g and %g\n",res,t1,t2);
    }
    {
      Real sign = -1.0;
      Real a=Sqr(endTime);
      Real b=sign*(2.0*(dx0+dx1)*endTime+4.0*(x0-x1));
      Real c=-Sqr(dx1-dx0);
      printf("Quadratic %g x^2 + %g x + %g = 0\n",a,b,c);
      Real t1,t2;
      int res = quadratic(a,b,c,t1,t2);
      printf("Solutions: %d, %g and %g\n",res,t1,t2);
    }
    printf("Saving to PP_SolveMinAccel_failure.dat\n");
    SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
    if(!FuzzyEquals(x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),gPauseEpsilon)) 
      getchar();
    return false;
  }
  if(!FuzzyEquals(dx0 + a*tswitch,dx1-a*t2mT,EpsilonV)) {
    printf("PPRamp: Error solving min-accel!\n");
    printf("Velocity error %g vs %g, err %g\n",dx0+a*tswitch,dx1-a*t2mT,dx0+a*tswitch-(dx1-a*t2mT));
    printf("Saving to PP_SolveMinAccel_failure.dat\n");
    SaveRamp("PP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,Inf,endTime);
    if(!FuzzyEquals(x0 + tswitch*dx0 + 0.5*a*Sqr(tswitch),x1+t2mT*dx1-0.5*a*Sqr(t2mT),gPauseEpsilon)) 
      getchar();
    return false;
  }
  return true;
}

Real PPRamp::MaxVelocity() const
{
  return tswitch*a+dx0;
}

Real PPRamp::CalcTotalTime(Real a) const
{
  Real tswitch = CalcSwitchTime(a);
  //printf("a = %g, switch time %g\n",a,tswitch);
  if(tswitch < 0) return -1;
  if(tswitch < (dx1-dx0)/a) return -1;
  return tswitch*2.0 - (dx1-dx0)/a;
}

Real PPRamp::CalcSwitchTime(Real a) const
{
  //this may be prone to numerical errors
  //Real b = 2.0*dx0; //2.0*(dx0-dx1);
  //Real c = (Sqr(dx0)-Sqr(dx1))*0.5/a+x0-x1;
  Real b = 2.0*a*dx0; //2.0*(dx0-dx1);
  Real c = (Sqr(dx0)-Sqr(dx1))*0.5+(x0-x1)*a;
  Real t1,t2;
  int res=quadratic(a*a,b,c,t1,t2);
  if(res == 0) {
    return -1;
  }
  else if(res == 2) {
    //if(t1 < 0 && t1 > -EpsilonT) t1=0;
    //if(t2 < 0 && t2 > -EpsilonT) t2=0;
    if(t1 < 0) return t2;
    else if(t2 < 0) return t1;
    else {  //check ttotal
      if(t2*Abs(a) < (dx1-dx0)*Sign(a)) return t1;
      else if(t1*Abs(a) < (dx1-dx0)*Sign(a)) return t2;
      else return Min(t1,t2); //both are ok
    }
  }
  else return t1;
}

//for a PP ramp:
//xs = x0 + ts*dx0 + 0.5*z*ts^2
//xs = x1 - (T-ts)*dx1 - 0.5*z*(T-ts)^2
//xs' = dx0 + ts*z
//xs' = dx1 + (T-ts)*z
//z = sign*a
//(2ts-T)*z = dx1 - dx0
//ts = 1/2* (T+(dx1 - dx0)/z)
//T-ts = 1/2* (T-(dx1 - dx0)/z)
//2 T(dx0+dx1) + 4(x0 - x1) - (dx1 - dx0)^2/z + z*T^2 = 0
//what if z is close to 0?
//suppose dx1 ~= dx0, then the other solution is 
//4 T dx0 + 4(x0 - x1) + z*T^2 = 0
//=>z = - 4 dx0 / T + 4(x1 - x0)/T^2
//
//alt: let y = (dx1 - dx0)/z, z = (dx1 - dx0)/y  (y is a time shift)
//ts = 1/2* (T+y)
//T-ts = 1/2* (T-y)
//x0 + 1/2(T+y)*dx0 + 0.5*z*1/4(T+y)^2 = x1 - 1/2(T-y)*dx1 - 0.5*z*1/4(T-y)^2
//4(x0-x1) + 2(T+y)*dx0 + 0.5*z*(T+y)^2 = - 2(T-y)*dx1 - 0.5*z*(T-y)^2
//[4(x0-x1)/T + 2(dx0+dx1)] y - y^2 (dx1 - dx0)/T + (dx1 - dx0) T = 0
Real PPRamp::CalcMinAccel(Real endTime,Real sign,Real& switchTime) const
{
  Real a = -(dx1 - dx0)/endTime;
  Real b = (2.0*(dx0+dx1)+4.0*(x0-x1)/endTime);
  Real c = (dx1 - dx0)*endTime;
  Real rat1,rat2;
  if(gDebugPrint) printf("x0 %g, x1 %g, dx0 %g, dx1 %g, endtime %g\n",x0,x1,dx0,dx1,endTime);
  if(gDebugPrint) printf("Quadratic %g %g %g\n",a,b,c);
  int res=quadratic(a,b,c,rat1,rat2);
  if(gDebugPrint) printf("results: %d, %g %g\n",res,rat1,rat2);
  Real accel1 = (dx1-dx0)/rat1;
  Real accel2 = (dx1-dx0)/rat2;
  Real switchTime1 = endTime*0.5+0.5*rat1;
  Real switchTime2 = endTime*0.5+0.5*rat2;
  //fix up numerical errors
  if(switchTime1 >  endTime && switchTime1 < endTime+EpsilonT*1e-1)
    switchTime1 = endTime;
  if(switchTime2 >  endTime && switchTime2 < endTime+EpsilonT*1e-1)
    switchTime2 = endTime;
  if(switchTime1 < 0 && switchTime1 > EpsilonT*1e-1)
    switchTime1 = 0;
  if(switchTime2 < 0 && switchTime2 > EpsilonT*1e-1)
    switchTime2 = 0;
  if(res > 0 && FuzzyZero(rat1,EpsilonT)) {
    //consider it as a zero, ts = T/2    
    //z = - 4*(x0-x1)/T^2 - 2 (dx0+dx1)/T
    if(!FuzzyZero(endTime,EpsilonT)) { //no good answer if endtime is small
      accel1=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
    }
  }
  if(res > 1 && FuzzyZero(rat2,EpsilonT)) {
    if(!FuzzyZero(endTime,EpsilonT)) { //no good answer if endtime is small
      accel2=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
    }
  }
  bool firstInfeas = false;
  if(res > 0 && (FuzzyZero(accel1,EpsilonA) || FuzzyZero(endTime/rat1,EpsilonA))) { //infer that accel must be small
    if(!FuzzyZero(dx0-dx1,EpsilonT)) { //no good answer if dx0=dx1
      switchTime1 = endTime*0.5;
    }
    if(gDebugPrint) printf("Rat1: %g, switch %g\n",rat1,switchTime1);
    if(gDebugPrint) printf("Two items: %g, %g\n",x0 + switchTime1*dx0 + 0.5*Sqr(switchTime1)*accel1,x1 - (endTime-switchTime1)*dx1 - 0.5*Sqr(endTime-switchTime1)*accel1);
    if(!FuzzyEquals(x0 + switchTime1*dx0 + 0.5*Sqr(switchTime1)*accel1,x1 - (endTime-switchTime1)*dx1 - 0.5*Sqr(endTime-switchTime1)*accel1,EpsilonX) ||
       !FuzzyEquals(dx0+switchTime1*accel1,dx1+(endTime-switchTime1)*accel1,EpsilonV)) {
      firstInfeas = true;
      if(gDebugPrint) printf("infeasible... \n");
    }
  }
  if(res > 1 && (FuzzyZero(accel2,EpsilonA) || FuzzyZero(endTime/rat2,EpsilonA))) {
    if(!FuzzyZero(dx0-dx1,EpsilonT)) { //no good answer if dx0=dx1
      switchTime2 = endTime*0.5;
    }
    if(gDebugPrint) printf("Rat2: %g, switch %g\n",rat2,switchTime2);
    if(gDebugPrint) printf("Two items: %g, %g\n",x0 + switchTime2*dx0 + 0.5*Sqr(switchTime2)*accel2,x1 - (endTime-switchTime2)*dx1 - 0.5*Sqr(endTime-switchTime2)*accel2);
    if(!FuzzyEquals(x0 + switchTime2*dx0 + 0.5*Sqr(switchTime2)*accel2,x1 - (endTime-switchTime2)*dx1 - 0.5*Sqr(endTime-switchTime2)*accel2,EpsilonX) ||
       !FuzzyEquals(dx0+switchTime2*accel2,dx1+(endTime-switchTime2)*accel2,EpsilonV)) {
      res--;
      if(gDebugPrint) printf("infeasible... \n");
    }
  }
  if(gDebugPrint) printf("accels %g %g\n",accel1,accel2);
  if(gDebugPrint) printf("switch times %g %g < %g\n",switchTime1,switchTime2,endTime);
  if(firstInfeas) {
    accel1 = accel2;
    rat1 = rat2;
    switchTime1 = switchTime2;
    res--;
  }
  if(res==0) return -1;
  else if(res==1) {
    if(switchTime1 >= 0 && switchTime1 <= endTime) { switchTime=switchTime1; return sign*accel1; }
    return -1.0;
  }
  else if(res==2) {
    if(switchTime1 >= 0 && switchTime1 <= endTime) {
      if(switchTime2 >= 0 && switchTime2 <= endTime) {
	if(accel1 < accel2) { switchTime=switchTime1; return sign*accel1; }
	else { switchTime=switchTime2; return sign*accel2; }
      }
      else { switchTime=switchTime1; return sign*accel1; }
    }
    else if(switchTime2 >= 0 && switchTime2 <= endTime) { switchTime=switchTime2; return sign*accel2; }
    return -1.0;
  }
  if(FuzzyZero(a,EpsilonT) && FuzzyZero(b,EpsilonT) && FuzzyZero(c,EpsilonT)) {
    switchTime = 0.5*endTime;
    return 0;
  }
  return -1.0;

  /*
  Real a=endTime;
  Real b=sign*(2.0*(dx0+dx1)+4.0*(x0-x1)/endTime);
  if(FuzzyZero(b,EpsilonX)) {
    //need to double check for numerical instability
    //if sign is +, this means we're switching directly to -
    //if sign is -, this means we're switching directly to +
    //if(sign > 0.0 && x1 > x0+dx0*endTime) return -1;
    //else if(sign < 0.0 && x1 < x0+dx0*endTime) return -1;
    switchTime = 0;
    Real a=(dx1-dx0)/endTime;
    if((sign > 0.0) == (a >= 0.0)) return -1;
    else return Abs(a);
  }
  Real c=-Sqr(dx1-dx0)/endTime;
  if(FuzzyEquals(dx1,dx0,EpsilonV)) {
    //one of the solutions will be very close to zero, use alt solution
    Real a=-2.0*(dx0+dx1)/endTime + 4.0*(x1-x0)/Sqr(endTime);
    printf("only two solutions: 0 and %g\n",a);
    switchTime = 0.5*endTime;
    //try out the zero solution
    printf("diff at 0 solution: %g\n",x0-x1 + switchTime*(dx0+dx1));
    if(FuzzyEquals(x0 + switchTime*dx0,x1 - switchTime*dx1,EpsilonX)) 
      return 0;
    Assert(FuzzyEquals(dx0 + switchTime*a,dx1 + switchTime*a,EpsilonV));
    Assert(FuzzyEquals(x0 + switchTime*dx0 + 0.5*a*Sqr(switchTime),x1 - switchTime*dx1-0.5*a*Sqr(switchTime),EpsilonX));
    if((sign > 0.0) != (a >= 0.0)) return -1;
    return Abs(a);
  }
  if(FuzzyZero(c,EpsilonA)) {
    //need better numerical performance when dx1 ~= dx0
    a = a/Abs(dx1-dx0);
    b = b/Abs(dx1-dx0);
    c = -Abs(dx1-dx0)/endTime;
  }
  Real accel1,accel2;
  int res=quadratic(a,b,c,accel1,accel2);
  if(gDebugPrint) printf("x0 %g, x1 %g, dx0 %g, dx1 %g\n",x0,x1,dx0,dx1);
  if(gDebugPrint) printf("a %g, b %g, c %g\n",a,b,c);
  if(gDebugPrint) printf("%d quadratic results %g %g\n",res,accel1,accel2);
  //remove negative accelerations
  if(res >= 1 && accel1 < 0) {
    accel1 = accel2;
    res--;
  }
  if(res >= 2 && accel2 < 0) {
    res--;
  }

  Real switchTime1 = endTime*0.5+sign*0.5*(dx1-dx0)/accel1;
  Real switchTime2 = endTime*0.5+sign*0.5*(dx1-dx0)/accel2;
  if(gDebugPrint) printf("switch times %g %g < %g\n",switchTime1,switchTime2,endTime);
  //if(accel1 == 0 && x0 == x1) switchTime1 = 0;
  //if(accel2 == 0 && x0 == x1) switchTime2 = 0;
  if(res==0) return -1;
  else if(res==1) {
    if(!IsFinite(accel1)) {
      printf("Error computing accelerations!\n");
      printf("Quadratic %g x^2 + %g x + %g = 0\n",a,b,c);
      printf("x0 %g, dx0 %g, x1 %g, dx1 %g\n",x0,dx0,x1,dx1);
      printf("EndTime %g, sign %g\n",endTime,sign);
      printf("Results %g %g\n",accel1,accel2);
      getchar();
    }
    if(switchTime1 >= 0 && switchTime1 <= endTime) { switchTime=switchTime1; return accel1; }
    return -1.0;
  }
  else if(res==2) {
    if(!IsFinite(accel1) || !IsFinite(accel2)) {
      printf("Error computing accelerations!\n");
      printf("Quadratic %g x^2 + %g x + %g = 0\n",a,b,c);
      printf("x0 %g, dx0 %g, x1 %g, dx1 %g\n",x0,dx0,x1,dx1);
      printf("EndTime %g, sign %g\n",endTime,sign);
      printf("Results %g %g\n",accel1,accel2);
      getchar();
    }
    if(switchTime1 >= 0 && switchTime1 <= endTime) {
      if(switchTime2 >= 0 && switchTime2 <= endTime) {
	if(accel1 < accel2) { switchTime=switchTime1; return accel1; }
	else { switchTime=switchTime2; return accel2; }
      }
      else { switchTime=switchTime1; return accel1; }
    }
    else if(switchTime2 >= 0 && switchTime2 <= endTime) { switchTime=switchTime2; return accel2; }
    return -1.0;
  }
  return -1.0;
*/
}


Real PLPRamp::Evaluate(Real t) const
{
  Real tmT = t - ttotal;
  if(t < tswitch1) return x0 + 0.5*a*Sqr(t) + dx0*t;
  else if(t < tswitch2) {
    Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
    return xswitch + (t-tswitch1)*v;
  }
  else return x1 - 0.5*a*Sqr(tmT) + dx1*tmT;
}

Real PLPRamp::Derivative(Real t) const
{
  Real tmT = t - ttotal;
  if(t < tswitch1) return a*t + dx0;
  else if(t < tswitch2) return v;
  else return -a*tmT + dx1;
}

Real PLPRamp::Accel(Real t) const
{
  if(t < tswitch1) return a;
  else if(t < tswitch2) return 0;
  else return -a;
}

Real PLPRamp::CalcTotalTime(Real a,Real v) const
{
  Real t1 = (v-dx0)/a;
  Real t2mT = (dx1-v)/a;
  Real y1 = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
  Real y2 = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
  Real t2mt1 = (y2-y1)/v;
  //printf("t1 %g, t2mT %g, t2mt %g\n",t1,t2mT,t2mt1);
  //Real t2mt1 = 0.5*(Sqr(dx1) + Sqr(dx0))/(v*a) - v/a + (x1 - x0)/v;
  //Real xswitch = x0 + 0.5*a*Sqr(t1) + dx0*t1;
  if(t1 < 0 || t2mT > 0 || t2mt1 < 0) return -1;
  if(!IsFinite(t1) || !IsFinite(t2mT)) return -1;
  return t1 + t2mt1 - t2mT;
}

Real PLPRamp::CalcSwitchTime1(Real a,Real v) const
{
  Real t1 = (v-dx0)/a;
  if(t1 < 0) return -1;
  return t1;
}

Real PLPRamp::CalcSwitchTime2(Real a,Real v) const
{
  Real t1 = (v-dx0)/a;
  Real y1 = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
  Real y2 = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
  Real t2mt1 = (y2-y1)/v;
  //Real t2mt1 = 0.5*(Sqr(dx1) + Sqr(dx0))/(v*a) - v/a + (x1 - x0)/v;
  //Real xswitch = x0 + 0.5*a*Sqr(t1) + dx0*t1;
  if(t1 < 0 || t2mt1 < 0) return -1;
  return t1 + t2mt1;
}

Real PLPRamp::CalcMinAccel(Real endTime,Real v) const
{
  Real den=endTime*v - (x1-x0);
  //straight line sections have den ~= 0
  if(FuzzyZero(den,EpsilonX)) {
    if(FuzzyEquals(dx0,v,EpsilonV) && FuzzyEquals(dx1,v,EpsilonV)) return 0;
    return Inf;
  }
  
  //Real a = (v - (dx0+dx1) + 0.5/v*(Sqr(dx0)+Sqr(dx1)))/(endTime - (x1-x0)/v);
  Real a = (Sqr(v) - v*(dx0+dx1) + 0.5*(Sqr(dx0)+Sqr(dx1)))/den;
  /*
  Real t1 = (v-dx0)/a;
  Real t2mT = (dx1-v)/a;
  Real y1 = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
  Real y2 = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
  //Real t2mt1 = 0.5*(Sqr(dx1) + Sqr(dx0))/(v*a) - v/a + (x1 - x0)/v;
  Real t2mt1 = (y2-y1)/v;
  Real vold = v;
  //cout<<"EndTime "<<endTime<<", v "<<v<<endl;
  //cout<<"a = "<<a<<", t1="<<t1<<", t2mt1="<<t2mt1<<", t2mT="<<t2mT<<endl;
  if(t1 < 0 || t2mT > 0 || t2mt1 < 0) return Inf;
  if(!IsFinite(t1) || !IsFinite(t2mT)) return Inf;
  */
  if(!(CalcTotalTime(a,v) >= 0)) { return Inf; }
  return a;

  /*
  if(!(CalcTotalTime(a,v) >= 0)) {
    //this is very strange -- does it happen because of a compiler
    //optimization error?
    fprintf(stderr,"PLPRamp::CalcMinAccel: some numerical error prevented computing total time\n");
    fprintf(stderr,"  Ramp %g,%g -> %g,%g\n",x0,dx0,x1,dx1);
    fprintf(stderr,"  endTime %g, accel %g, vel %g, switch times %g %g, total time %g\n",endTime,a,v,CalcSwitchTime1(a,v),CalcSwitchTime2(a,v),CalcTotalTime(a,v)); 
    assert(v == vold);
    printf("y1=%g, y2=%g, t2mt1 = %g\n",y1,y2,t2mt1);
    Real y1_test = 0.5*(Sqr(v) - Sqr(dx0))/a + x0;
    Real y2_test = 0.5*(Sqr(dx1) - Sqr(v))/a + x1;
    Real t2mt1_test = (y2_test-y1_test)/v;
    //Real t2mt1_test = 0.5*(Sqr(dx1) + Sqr(dx0))/(v*a) - v/a + (x1 - x0)/v;
    printf("y1=%g, y2=%g, t2mt1 = %g\n",y1_test,y2_test,t2mt1_test);
    printf("dy1=%g, dy2=%g, dt2mt1 = %g\n",y1-y1_test,y2-y2_test,t2mt1-t2mt1_test);
    getchar();
    return Inf;
    assert(y1 == y1_test);
    assert(y2 == y2_test);
    assert(y2-y1 == y2_test-y1_test);
    assert(t2mt1 == t2mt1_test);
  }
  assert(CalcTotalTime(a,v) >= 0);
  return a;
  */
}


bool PLPRamp::SolveMinTime(Real amax,Real vmax)
{
  Real t1 = CalcTotalTime(amax,vmax);
  Real t2 = CalcTotalTime(-amax,vmax);
  Real t3 = CalcTotalTime(amax,-vmax);
  Real t4 = CalcTotalTime(-amax,-vmax);
  //cout<<"Time for PLP ++-: "<<t1<<", -++: "<<t2<<", +--: "<<t3<<", --+: "<<t4<<endl;
  ttotal = Inf;
  if(t1 >= 0 && t1 < ttotal) {
    a = amax;
    v = vmax;
    ttotal = t1;
  }
  if(t2 >= 0 && t2 < ttotal) {
    a = -amax;
    v = vmax;
    ttotal = t2;
  }
  if(t3 >= 0 && t3 < ttotal) {
    a = amax;
    v = -vmax;
    ttotal = t3;
  }
  if(t4 >= 0 && t4 < ttotal) {
    a = -amax;
    v = -vmax;
    ttotal = t4;
  }
  if(IsInf(ttotal)) {
    a = v = 0;
    tswitch1 = tswitch2 = ttotal = -1;
    return false;
  }
  tswitch1 = CalcSwitchTime1(a,v);
  tswitch2 = CalcSwitchTime2(a,v);

  if(tswitch1 > tswitch2 && FuzzyEquals(tswitch1,tswitch2,EpsilonT)) {
    tswitch1 = tswitch2 = 0.5*(tswitch1+tswitch2);
  }
  if(tswitch2 > ttotal && FuzzyEquals(tswitch2,ttotal,EpsilonT)) {
    tswitch2 = ttotal;
  }

  Real t2mT = tswitch2-ttotal;
  Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
  Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
  if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,EpsilonX)) {
    fprintf(stderr,"PLP Ramp has incorrect switch 2 position: %g vs %g\n",xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT);
    printf("Ramp %g,%g -> %g,%g\n",x0,dx0,x1,dx1);
    printf("Acceleration %g, vel %g, deceleration %g\n",a,v,-a);
    printf("Switch times %g %g %g\n",tswitch1,tswitch2,ttotal);
    printf("Saving to PLP_SolveMinTime_failure.dat\n");
    SaveRamp("PLP_SolveMinTime_failure.dat",x0,dx0,x1,dx1,amax,vmax,-1);
    if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,gPauseEpsilon))
      getchar();
    //return false;
  }
  return true;
}

bool PLPRamp::SolveMinAccel(Real endTime,Real vmax)
{
  Real a1 = CalcMinAccel(endTime,vmax);
  Real a2 = CalcMinAccel(endTime,-vmax);
  a = Inf;
  if(fabs(a1) < a) {
    a = a1;
    v = vmax;
  }
  if(fabs(a2) < a) {
    a = a2;
    v = -vmax;
  }
  if(IsInf(a)) {
    a = 0;
    tswitch1 = tswitch2 = ttotal = -1;
    return false;
  }
  if(fabs(a) == 0) {
    tswitch1 = 0;
    tswitch2 = endTime;
    ttotal = endTime;
  }
  else {
    ttotal = CalcTotalTime(a,v);
    tswitch1 = CalcSwitchTime1(a,v);
    tswitch2 = CalcSwitchTime2(a,v);

    if(tswitch1 > tswitch2 && FuzzyEquals(tswitch1,tswitch2,EpsilonT)) {
      tswitch1 = tswitch2 = 0.5*(tswitch1+tswitch2);
    }
    if(tswitch2 > endTime && FuzzyEquals(tswitch2,endTime,EpsilonT)) {
      tswitch2 = endTime;
    }
    if(ttotal < 0) {  //there was an error computing the total time
      fprintf(stderr,"PLPRamp::SolveMinAccel: some numerical error prevented computing total time\n");
      fprintf(stderr,"  Ramp %g,%g -> %g,%g\n",x0,dx0,x1,dx1);
      fprintf(stderr,"  endTime %g, accel %g, vel %g, switch times %g %g, total time %g\n",endTime,a,v,tswitch1,tswitch2,ttotal); 
      printf("Saving to PLP_SolveMinAccel_failure.dat\n");
      SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
      if(ttotal < -gPauseEpsilon)
	getchar();
      return false;
    }
  }
  if(ttotal > endTime + EpsilonT) {
    fprintf(stderr,"PLPRamp::SolveMinAccel: total time greater than endTime!\n");
    fprintf(stderr,"  endTime %g, accel %g, vel %g, switch times %g %g, total time %g\n",endTime,a,v,tswitch1,tswitch2,ttotal); 
    printf("Saving to PLP_SolveMinAccel_failure.dat\n");
    SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
    if(ttotal > endTime + gPauseEpsilon)
      getchar();
    return false;
  }
  if(fabs(ttotal-endTime) >= EpsilonT) {
    fprintf(stderr,"PLPRamp::SolveMinAccel: total time and endTime are different!\n");
    fprintf(stderr,"  endTime %g, accel %g, vel %g, switch times %g %g, total time %g\n",endTime,a,v,tswitch1,tswitch2,ttotal); 
    printf("Saving to PLP_SolveMinAccel_failure.dat\n");
    SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
    if(fabs(ttotal-endTime) >= gPauseEpsilon)
      getchar();
  }
  assert(fabs(ttotal-endTime) < EpsilonT);
  //fiddle with the numerical errors
  ttotal = endTime;
  if(tswitch2 > ttotal) tswitch2=ttotal;
  if(tswitch1 > ttotal) tswitch1=ttotal;

  Real t2mT = tswitch2-ttotal;
  Real xswitch = x0 + 0.5*a*Sqr(tswitch1) + dx0*tswitch1;
  Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
  if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,EpsilonX)) {
    fprintf(stderr,"PLP Ramp has incorrect switch 2 position: %g vs %g\n",xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT);
    printf("Ramp %g,%g -> %g,%g\n",x0,dx0,x1,dx1);
    printf("Acceleration %g, vel %g, deceleration %g\n",a,v,-a);
    printf("Switch times %g %g %g\n",tswitch1,tswitch2,ttotal);
    printf("Saving to PLP_SolveMinAccel_failure.dat\n");
    SaveRamp("PLP_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
    if(!FuzzyEquals(xswitch2,x1 - 0.5*a*Sqr(t2mT) + dx1*t2mT,gPauseEpsilon)) 
      getchar();
    return false;
  }

  return true;
}







void ParabolicRamp1D::SetConstant(Real x,Real t)
{
  x0 = x1 = x;
  dx0 = dx1 = 0;
  tswitch1=tswitch2=ttotal=t;
  v = a1 = a2 = 0;
}

void ParabolicRamp1D::SetLinear(Real _x0,Real _x1,Real t)
{
  assert(t > 0);
  x0 = _x0;
  x1 = _x1;
  v = dx0 = dx1 = (_x1-_x0)/t;
  a1 = a2 = 0;
  tswitch1 = 0;
  tswitch2 = t;
  ttotal = t;
}

Real ParabolicRamp1D::Evaluate(Real t) const
{
  Real tmT = t - ttotal;
  if(t < tswitch1) return x0 + 0.5*a1*t*t + dx0*t;
  else if(t < tswitch2) {
    Real xswitch = x0 + 0.5*a1*tswitch1*tswitch1 + dx0*tswitch1;
    return xswitch + (t-tswitch1)*v;
  }
  else return x1 + 0.5*a2*tmT*tmT + dx1*tmT;
}

Real ParabolicRamp1D::Derivative(Real t) const
{
  if(t < tswitch1) return a1*t + dx0;
  else if(t < tswitch2) return v;
  else {
    Real tmT = t - ttotal;
    return a2*tmT + dx1;
  }
}

Real ParabolicRamp1D::Accel(Real t) const
{
  if(t < tswitch1) return a1;
  else if(t < tswitch2) return 0;
  else return a2;
}

bool ParabolicRamp1D::SolveMinAccel(Real endTime,Real vmax)
{
  ParabolicRamp p;
  PPRamp pp;
  PLPRamp plp;
  p.x0 = pp.x0 = plp.x0 = x0;
  p.x1 = pp.x1 = plp.x1 = x1;
  p.dx0 = pp.dx0 = plp.dx0 = dx0;
  p.dx1 = pp.dx1 = plp.dx1 = dx1;
  //bool pres = p.Solve();
  bool pres=false;
  bool ppres = pp.SolveMinAccel(endTime);
  bool plpres = false;
  if(!IsInf(vmax))
    plpres = plp.SolveMinAccel(endTime,vmax);
  //cout<<"PP a: "<<pp.a<<", max vel "<<pp.MaxVelocity()<<endl;
  //cout<<"PLP a: "<<plp.a<<", vel "<<plp.v<<endl;
  a1 = Inf;
  if(pres && FuzzyEquals(endTime,p.ttotal,EpsilonT) && Abs(p.MaxVelocity()) <= vmax) {
    a1 = p.a;
    v = 0;
    //tswitch1 = tswitch2 = p.ttotal;
    //ttotal = p.ttotal;
    tswitch1 = tswitch2 = endTime;
    ttotal = endTime;
  }
  if(ppres && Abs(pp.a) < Abs(a1)) {
    if(Abs(pp.MaxVelocity()) <= vmax) {
      a1 = pp.a;
      v = 0;
      tswitch1 = tswitch2 = pp.tswitch;
      ttotal = pp.ttotal;
    }
    else {
      //fprintf(stderr,"ParabolicRamp1D::SolveMinAccel: PP exceeded max velocity\n");
    }
  }
  if(plpres && Abs(plp.a) < Abs(a1)) {
    if(Abs(plp.v) <= vmax) {
      a1 = plp.a;
      v = plp.v;
      tswitch1 = plp.tswitch1;
      tswitch2 = plp.tswitch2;
      ttotal = plp.ttotal;
    }
    else {
      fprintf(stderr,"ParabolicRamp1D::SolveMinAccel: PLP exceeded max velocity\n");
      getchar();
    }
  }
  a2 = -a1;
  
  if(IsInf(a1)) {
    if(vmax == 0) {
      if(FuzzyEquals(x0,x1,EpsilonX) && FuzzyEquals(dx0,dx1,EpsilonV)) {
	a1 = a2 = v = 0;
	tswitch1 = tswitch2 = ttotal = endTime;
	return true;
      }
    }
    if(ppres && Abs(pp.MaxVelocity()) <= vmax + EpsilonV) {
      //some slight numerical error caused velocity to exceed maximum
      a1 = pp.a;
      a2 = -pp.a;
      v = 0;
      tswitch1 = tswitch2 = pp.tswitch;
      ttotal = pp.ttotal;
      if(IsValid()) return true;
    }
    a1 = a2 = v = 0;
    tswitch1 = tswitch2 = ttotal = -1;
    printf("No ramp equation could solve for min-accel!\n");
    printf("x0=%g, x1=%g, dx0=%g, dx1=%g\n",x0,x1,dx0,dx1);
    printf("end time %g, vmax = %g\n",endTime,vmax);

    printf("PP=%d, PLP=%d\n",(int)ppres,(int)plpres);
    printf("pp.a = %g, max vel=%g\n",pp.a,pp.MaxVelocity());
    printf("plp.a = %g, v=%g\n",plp.a,plp.v);

    Real switch1,switch2;
    Real apn = pp.CalcMinAccel(endTime,1.0,switch1);
    Real anp = pp.CalcMinAccel(endTime,-1.0,switch2);
    printf("PP Calcuations: +: %g %g, -: %g %g\n",apn,switch1,anp,switch2);
    printf("Saving to Ramp_SolveMinAccel_failure.dat\n");
    SaveRamp("Ramp_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
    return false;
  }
  assert(ttotal==endTime);
  if(!IsValid()) {
    printf("Invalid min-accel!\n");
    printf("x0=%g, x1=%g, dx0=%g, dx1=%g\n",x0,x1,dx0,dx1);
    printf("end time %g, vmax = %g\n",endTime,vmax);

    printf("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
    printf("p.a = %g, endtime=%g\n",p.a,p.ttotal);
    printf("pp.a = %g, max vel=%g\n",pp.a,pp.MaxVelocity());
    printf("plp.a = %g, v=%g\n",plp.a,plp.v);
    printf("pp.tswitch = %g\n",pp.tswitch);
    printf("plp.tswitch1 = %g, tswitch2=%g\n",plp.tswitch1,plp.tswitch2);
    printf("Saving to Ramp_SolveMinAccel_failure.dat\n");
    SaveRamp("Ramp_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,-1,vmax,endTime);
    getchar();
  }
  return true;
}

bool ParabolicRamp1D::SolveMinTime(Real amax,Real vmax)
{
  ParabolicRamp p;
  PPRamp pp;
  PLPRamp plp;
  p.x0 = pp.x0 = plp.x0 = x0;
  p.x1 = pp.x1 = plp.x1 = x1;
  p.dx0 = pp.dx0 = plp.dx0 = dx0;
  p.dx1 = pp.dx1 = plp.dx1 = dx1;
  bool pres = p.Solve(amax);
  bool ppres = pp.SolveMinTime(amax);
  bool plpres = false;
  if(!IsInf(vmax))
    plpres = plp.SolveMinTime(amax,vmax);
  //cout<<"P time: "<<p.ttotal<<", accel "<<p.a<<endl;
  //cout<<"PP time: "<<pp.ttotal<<", max vel "<<pp.MaxVelocity()<<endl;
  //cout<<"PLP time: "<<plp.ttotal<<", vel "<<plp.v<<endl;
  ttotal = Inf;
  if(pres  && p.ttotal < ttotal) {
    a1 = p.a;
    v = 0;
    tswitch1 = tswitch2 = p.ttotal;
    ttotal = p.ttotal;
  }
  if(ppres && Abs(pp.MaxVelocity()) <= vmax && pp.ttotal < ttotal) {
    a1 = pp.a;
    v = 0;
    tswitch1 = tswitch2 = pp.tswitch;
    ttotal = pp.ttotal;
  }
  if(plpres && plp.ttotal < ttotal) {
    a1 = plp.a;
    v = plp.v;
    tswitch1 = plp.tswitch1;
    tswitch2 = plp.tswitch2;
    ttotal = plp.ttotal;
  }
  if(IsInf(ttotal)) {
    printf("ParabolicRamp1D::SolveMinTime: no ramp equation worked!\n");
    printf("x0=%g, x1=%g, dx0=%g, dx1=%g\n",x0,x1,dx0,dx1);
    printf("vmax = %g, amax = %g\n",vmax,amax);
    printf("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
    printf("Saving to Ramp_SolveMinTime_failure.dat\n");
    SaveRamp("Ramp_SolveMinAccel_failure.dat",x0,dx0,x1,dx1,amax,vmax,-1);
    a1 = a2 = v = 0;
    tswitch1 = tswitch2 = ttotal = -1;
    return false;
  }
  a2 = -a1;
  //cout<<"switch time 1: "<<tswitch1<<", 2: "<<tswitch2<<", total "<<ttotal<<endl;
  if(!IsValid()) {
    printf("ParabolicRamp1D::SolveMinTime: Failure to find valid path!\n");
    printf("x0=%g, x1=%g, dx0=%g, dx1=%g\n",x0,x1,dx0,dx1);
    printf("vmax = %g, amax = %g\n",vmax,amax);
    printf("P=%d, PP=%d, PLP=%d\n",(int)pres,(int)ppres,(int)plpres);
    getchar();
  }
  return true;
}

void ParabolicRamp1D::SolveBraking(Real amax)
{
  tswitch1 = 0;
  tswitch2 = 0;
  a1 = Sign(dx0)*amax;
  v = 0;
  a2 = -Sign(dx0)*amax;
  ttotal = Abs(dx0)/amax;
  x1 = x0 + dx0*ttotal + 0.5*Sqr(ttotal)*a2;
  dx1 = 0;
  assert(IsValid());
}

void ParabolicRamp1D::Dilate(Real timeScale)
{
  tswitch1*=timeScale;
  tswitch2*=timeScale;
  ttotal*=timeScale;
  a1 *= 1.0/Sqr(timeScale);
  a2 *= 1.0/Sqr(timeScale);
  v *= 1.0/timeScale;
}

void ParabolicRamp1D::TrimFront(Real tcut)
{
  if(tcut > ttotal) {
    printf("Hmm... want to trim front of curve at time %g, end time %g\n",tcut,ttotal);
  }
  assert(tcut <= ttotal);
  x0 = Evaluate(tcut);
  dx0 = Derivative(tcut);
  ttotal -= tcut;
  tswitch1 -= tcut;
  tswitch2 -= tcut;
  if(tswitch1 < 0) tswitch1=0;
  if(tswitch2 < 0) tswitch2=0;
  assert(IsValid());

}

void ParabolicRamp1D::TrimBack(Real tcut)
{
  x1 = Evaluate(ttotal-tcut);
  dx1 = Derivative(ttotal-tcut);
  ttotal -= tcut;
  tswitch1 = Min(tswitch1,ttotal);
  tswitch2 = Min(tswitch2,ttotal);
  assert(IsValid());
}

bool ParabolicRamp1D::IsValid() const
{
  if(tswitch1 < 0 || tswitch2 < tswitch1 || ttotal < tswitch2) {
    fprintf(stderr,"Ramp has invalid timing %g %g %g\n",tswitch1,tswitch2,ttotal);
    return false;
  }

  Real t2mT = tswitch2 - ttotal;
  if(tswitch1 != tswitch2) {
    if(!FuzzyEquals(a1*tswitch1 + dx0,v,EpsilonV)) {
      fprintf(stderr,"Ramp has incorrect switch 1 speed: %g vs %g\n",a1*tswitch1 + dx0,v);
      return false;
    }
    if(!FuzzyEquals(a2*t2mT + dx1,v,EpsilonV)) {
      fprintf(stderr,"Ramp has incorrect switch 2 speed: %g vs %g\n",a2*t2mT + dx1,v);
      return false;
    }
  }
  //check switch2
  Real xswitch = x0 + 0.5*a1*Sqr(tswitch1) + dx0*tswitch1;
  Real xswitch2 = xswitch + (tswitch2-tswitch1)*v;
  if(!FuzzyEquals(xswitch2,x1 + 0.5*a2*Sqr(t2mT) + dx1*t2mT,EpsilonX)) {
    fprintf(stderr,"Ramp has incorrect switch 2 position: %g vs %g\n",xswitch2,x1 + 0.5*a2*Sqr(t2mT) + dx1*t2mT);
    printf("Ramp %g,%g -> %g,%g\n",x0,dx0,x1,dx1);
    printf("Acceleration %g, vel %g, deceleration %g\n",a1,v,a2);
    printf("Switch times %g %g %g\n",tswitch1,tswitch2,ttotal);
    return false;
  }
  //check switch velocity
  if(tswitch2 > tswitch1) {
    if(!FuzzyEquals(dx0+tswitch1*a1,v,EpsilonV)) {
      fprintf(stderr,"Ramp has incorrect switch 1 velocity: %g vs %g\n",dx0+tswitch1*a1,v);
      return false;
    }
    if(!FuzzyEquals(dx1+t2mT*a2,v,EpsilonV)) {
      fprintf(stderr,"Ramp has incorrect switch 2 velocity: %g vs %g\n",dx1+t2mT*a2,v);
      return false;
    }
  }
  else {
    if(!FuzzyEquals(dx0+tswitch1*a1,dx1+t2mT*a2,EpsilonV)) {
      fprintf(stderr,"Ramp has incorrect switch velocity: %g vs %g, err=%g\n",dx0+tswitch1*a1,dx1+t2mT*a2,dx0+tswitch1*a1-(dx1+t2mT*a2));
      fprintf(stderr,"Switch times %g, %g, total %g\n",tswitch1,tswitch2,ttotal);
      fprintf(stderr,"Accel1 %g, accel2 %g\n",a1,a2);
      return false;
    }
  }
  return true;
}





void ParabolicRampND::SetConstant(const Vector& x,Real t)
{
  x0 = x1 = x;
  dx0.resize(x.n);
  dx1.resize(x.n);
  fill(dx0.begin(),dx0.end(),0);
  fill(dx1.begin(),dx1.end(),0);
  endTime = t;
  ramps.resize(x.n);
  for(int i=0;i<x.n;i++)
    ramps[i].SetConstant(x[i],t);
}

void ParabolicRampND::SetLinear(const Vector& _x0,const Vector& _x1,Real t)
{
  assert(_x0.n == _x1.n);
  assert(t > 0);
  x0 = _x0;
  x1 = _x1;
  dx0 = dx1 = (_x1-_x0)/t;
  endTime = t;
  ramps.resize(_x0.n);
  for(int i=0;i<_x0.n;i++)
    ramps[i].SetLinear(_x0[i],_x1[i],t);
}

bool ParabolicRampND::SolveMinTimeLinear(const Vector& amax,const Vector& vmax)
{
  assert(x0.n == dx0.n);
  assert(x1.n == dx1.n);
  assert(x0.n == x1.n);
  assert(x0.n == amax.n);
  assert(x0.n == vmax.n);
  endTime = 0;
  ramps.resize(x0.n);
  ParabolicRamp1D sramp;
  sramp.x0 = 0;
  sramp.x1 = 1;
  sramp.dx0 = 0;
  sramp.dx1 = 0;
  Real svmax=Inf,samax=Inf;
  for(size_t i=0;i<ramps.size();i++) {
    ramps[i].x0=x0[i];
    ramps[i].x1=x1[i];
    ramps[i].dx0=dx0[i];
    ramps[i].dx1=dx1[i];
    if(vmax[i]==0 || amax[i]==0) {
      if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
	printf("index %d vmax = %g, amax = %g, X0 != X1 (%g != %g)\n",i,vmax[i],amax[i],x0[i],x1[i]);
	return false;
      }
      if(!FuzzyEquals(dx0[i],dx1[i],EpsilonV)) {
	printf("index %d vmax = %g, amax = %g, DX0 != DX1 (%g != %g)\n",i,vmax[i],amax[i],dx0[i],dx1[i]);
	return false;
      }
      ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
      ramps[i].a1=ramps[i].a1=ramps[i].v=0;
      continue;
    }
    if(vmax[i] < svmax*Abs(x1[i]-x0[i]))
      svmax = vmax[i]/Abs(x1[i]-x0[i]);
    if(amax[i] < samax*Abs(x1[i]-x0[i]))
      samax = amax[i]/Abs(x1[i]-x0[i]);
  }

  if(IsInf(svmax) && IsInf(samax)) {
    //must have equal start/end state
    SetConstant(x0);
    return true;
  }

  bool res=sramp.SolveMinTime(samax,svmax);
  if(!res) {
    fprintf(stderr,"Warning in straight-line parameter solve\n");
    getchar();
    return false;
  }

  endTime = sramp.ttotal;
  for(size_t i=0;i<ramps.size();i++) {
    ramps[i].v = svmax * (x1[i]-x0[i]);
    ramps[i].a1 = samax * (x1[i]-x0[i]);
    ramps[i].a2 = -samax * (x1[i]-x0[i]);
    ramps[i].tswitch1 = sramp.tswitch1;
    ramps[i].tswitch2 = sramp.tswitch2;
    ramps[i].ttotal = endTime;
    if(!ramps[i].IsValid()) {
      fprintf(stderr,"Warning, error in straight-line path formula\n");
      getchar();
      res=false;
    }
  }
  return res;
}

bool ParabolicRampND::SolveMinTime(const Vector& amax,const Vector& vmax)
{
  assert(x0.n == dx0.n);
  assert(x1.n == dx1.n);
  assert(x0.n == x1.n);
  assert(x0.n == amax.n);
  assert(x0.n == vmax.n);
  endTime = 0;
  ramps.resize(x0.n);
  for(size_t i=0;i<ramps.size();i++) {
    ramps[i].x0=x0[i];
    ramps[i].x1=x1[i];
    ramps[i].dx0=dx0[i];
    ramps[i].dx1=dx1[i];
    if(vmax[i]==0 || amax[i]==0) {
      if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
	printf("index %d vmax = %g, amax = %g, X0 != X1 (%g != %g)\n",i,vmax[i],amax[i],x0[i],x1[i]);
	return false;
      }
      if(!FuzzyEquals(dx0[i],dx1[i],EpsilonV)) {
	printf("index %d vmax = %g, amax = %g, DX0 != DX1 (%g != %g)\n",i,vmax[i],amax[i],dx0[i],dx1[i]);
	return false;
      }
      ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
      ramps[i].a1=ramps[i].a2=ramps[i].v=0;
      continue;
    }
    if(!ramps[i].SolveMinTime(amax[i],vmax[i])) return false;
    if(ramps[i].ttotal > endTime) endTime = ramps[i].ttotal;
  }
  for(size_t i=0;i<ramps.size();i++) {
    if(ramps[i].ttotal != endTime) {
      if(vmax[i]==0) {
	ramps[i].ttotal = endTime;
      }
      else if(!ramps[i].SolveMinAccel(endTime,vmax[i])) {
	printf("Failed solving min accel for joint %d\n",i);
	ramps[i].SolveMinTime(amax[i],vmax[i]);
	printf("its min time is %g\n",ramps[i].ttotal);
	if(ramps[i].tswitch1==ramps[i].tswitch2)
	  printf("its type is PP\n");
	else if(Abs(ramps[i].v)==vmax[i])
	  printf("its type is PLP (vmax)\n");
	else 
	  printf("its type is PLP (v=%g %%)\n",ramps[i].v/vmax[i]);
	printf("Saving to ParabolicRampND_SolveMinAccel_failure.dat\n");
	SaveRamp("ParabolicRampND_SolveMinAccel_failure.dat",ramps[i].x0,ramps[i].dx0,ramps[i].x1,ramps[i].dx1,-1,vmax[i],endTime);
	printf("Saving to failed_ramps.txt\n");
	FILE* f=fopen("failed_ramps.txt","w+");
	fprintf(f,"MinAccel T=%g, vmax=%g\n",endTime,vmax[i]);
	fprintf(f,"x0=%g, dx0=%g\n",ramps[i].x0,ramps[i].dx0);
	fprintf(f,"x1=%g, dx1=%g\n",ramps[i].x1,ramps[i].dx1);
	fprintf(f,"MinTime solution v=%g, t1=%g, t2=%g, T=%g\n",ramps[i].v,ramps[i].tswitch1,ramps[i].tswitch2,ramps[i].ttotal);
	fprintf(f,"\n");
	fclose(f);
	return false;
      }
    }
    if(Abs(ramps[i].a1) > amax[i] || Abs(ramps[i].a2) > amax[i] || Abs(ramps[i].v) > vmax[i]) {
      fprintf(stderr,"ParabolicRampND: SolveMinTime: Element %d exceeds maximum accel or velocity\n",i);
      fprintf(stderr,"%g %g -> %g %g\n",ramps[i].x0,ramps[i].dx0,ramps[i].x1,ramps[i].dx1);
      fprintf(stderr,"a1 v a2 %g %g %g, amax %g, vmax %g\n",ramps[i].a1,ramps[i].v,ramps[i].a2,amax[i],vmax[i]);
      ramps[i].SolveMinTime(amax[i],vmax[i]);
      printf("Constraint time %g, element min time is %g\n",endTime,ramps[i].ttotal);
      //this is a new type of scenario that we haven't considered before
      //TODO: address min-time solving with a lower bound
      return false;
    }
    if(Abs(ramps[i].a1) > amax[i]+Epsilon || Abs(ramps[i].a2) > amax[i]+Epsilon || Abs(ramps[i].v) > vmax[i]) {
      fprintf(stderr,"Strange, ramp %d exceeds maximum accel or velocity\n",i);
      fprintf(stderr,"  endtime %g, amax %g, vmax %g\n",endTime,amax[i],vmax[i]);
      fprintf(stderr,"  a1 %g, v %g, a2 %g\n",ramps[i].a1,ramps[i].v,ramps[i].a2);
      fprintf(stderr,"  t1 %g, t2 %g, ttotal %g\n",ramps[i].tswitch1,ramps[i].tswitch2,ramps[i].ttotal);
      ramps[i].SolveMinTime(amax[i],vmax[i]);
      fprintf(stderr,"  min time solution:\n");
      fprintf(stderr,"  a1 %g, v %g, a2 %g\n",ramps[i].a1,ramps[i].v,ramps[i].a2);
      fprintf(stderr,"  t1 %g, t2 %g, ttotal %g\n",ramps[i].tswitch1,ramps[i].tswitch2,ramps[i].ttotal);

      gDebugPrint = true;
      ramps[i].SolveMinAccel(endTime,vmax[i]);
      gDebugPrint = false;
    }
    assert(Abs(ramps[i].a1) <= amax[i]+Epsilon);
    assert(Abs(ramps[i].a2) <= amax[i]+Epsilon);
    assert(Abs(ramps[i].v) <= vmax[i]);
    assert(ramps[i].ttotal==endTime);
  }
  return true;
}

bool ParabolicRampND::SolveMinAccel(const Vector& vmax,Real time)
{
  assert(x0.n == dx0.n);
  assert(x1.n == dx1.n);
  assert(x0.n == x1.n);
  assert(x0.n == vmax.n);
  endTime = time;
  ramps.resize(x0.n);
  for(size_t i=0;i<ramps.size();i++) {
    ramps[i].x0=x0[i];
    ramps[i].x1=x1[i];
    ramps[i].dx0=dx0[i];
    ramps[i].dx1=dx1[i];
    if(vmax[i]==0) {
      assert(FuzzyEquals(x0[i],x1[i],EpsilonX));
      assert(FuzzyEquals(dx0[i],dx1[i],EpsilonV));
      ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
      ramps[i].a1=ramps[i].a2=ramps[i].v=0;
      continue;
    }
    if(!ramps[i].SolveMinAccel(endTime,vmax[i])) {
      return false;
    }
  }
  return true;
}

bool ParabolicRampND::SolveMinAccelLinear(const Vector& vmax,Real time)
{
  assert(x0.n == dx0.n);
  assert(x1.n == dx1.n);
  assert(x0.n == x1.n);
  assert(x0.n == vmax.n);
  endTime = 0;
  ramps.resize(x0.n);
  ParabolicRamp1D sramp;
  sramp.x0 = 0;
  sramp.x1 = 1;
  sramp.dx0 = 0;
  sramp.dx1 = 0;
  Real svmax=Inf;
  for(size_t i=0;i<ramps.size();i++) {
    ramps[i].x0=x0[i];
    ramps[i].x1=x1[i];
    ramps[i].dx0=dx0[i];
    ramps[i].dx1=dx1[i];
    if(vmax[i]==0) {
      if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
	printf("index %d vmax = %g, X0 != X1 (%g != %g)\n",i,vmax[i],x0[i],x1[i]);
	return false;
      }
      if(!FuzzyEquals(dx0[i],dx1[i],EpsilonV)) {
	printf("index %d vmax = %g, DX0 != DX1 (%g != %g)\n",i,vmax[i],dx0[i],dx1[i]);
	return false;
      }
      ramps[i].tswitch1=ramps[i].tswitch2=ramps[i].ttotal=0;
      ramps[i].a1=ramps[i].a1=ramps[i].v=0;
      continue;
    }
    if(vmax[i] < svmax*Abs(x1[i]-x0[i]))
      svmax = vmax[i]/Abs(x1[i]-x0[i]);
  }

  if(IsInf(svmax)) {
    //must have equal start/end state
    SetConstant(x0);
    return true;
  }

  bool res=sramp.SolveMinAccel(svmax,time);
  if(!res) {
    fprintf(stderr,"Warning in straight-line parameter solve\n");
    getchar();
    return false;
  }

  endTime = sramp.ttotal;
  for(size_t i=0;i<ramps.size();i++) {
    ramps[i].v = sramp.v * (x1[i]-x0[i]);
    ramps[i].a1 = sramp.a1 * (x1[i]-x0[i]);
    ramps[i].a2 = sramp.a2 * (x1[i]-x0[i]);
    ramps[i].tswitch1 = sramp.tswitch1;
    ramps[i].tswitch2 = sramp.tswitch2;
    ramps[i].ttotal = endTime;
    if(!ramps[i].IsValid()) {
      fprintf(stderr,"Warning, error in straight-line path formula\n");
      getchar();
      res=false;
    }
  }
  return res;
}

void ParabolicRampND::SolveBraking(const Vector& amax)
{
  assert(x0.n == dx0.n);
  assert(x0.n == amax.n);
  x1.resize(x0.n);
  dx1.resize(x0.n);
  endTime = 0;
  ramps.resize(x0.n);
  for(size_t i=0;i<ramps.size();i++) {
    if(amax[i]==0) {
      if(!FuzzyEquals(dx0[i],0.0,EpsilonV)) {
	printf("index %d amax = %g, DX0 != 0 (%g != 0)\n",i,amax[i],dx0[i]);
	abort();
      }
      ramps[i].SetConstant(0);
      continue;
    }
    ramps[i].x0 = x0[i];
    ramps[i].dx0 = dx0[i];
    ramps[i].SolveBraking(amax[i]);
  }
  for(size_t i=0;i<ramps.size();i++)
    endTime = Max(endTime,ramps[i].ttotal);
  for(size_t i=0;i<ramps.size();i++) {
    if(amax[i] != 0 && ramps[i].ttotal != endTime) {
      //scale ramp acceleration to meet endTimeMax
      ramps[i].ttotal = endTime;
      //y(t) = x0 + t*dx0 + 1/2 t^2 a
      //y'(T) = dx0 + T a = 0
      ramps[i].a2 = -dx0[i] / endTime;
      ramps[i].a1 = -ramps[i].a2;
      ramps[i].x1 = ramps[i].x0 + endTime*ramps[i].dx0 + 0.5*Sqr(endTime)*ramps[i].a2;
    }
    x1[i]=ramps[i].x1;
    dx1[i]=0;
  }
  assert(IsValid());
}


Real ParabolicRampND::CalcMinTime(const Vector& amax,const Vector& vmax) const
{
  assert(x0.n == dx0.n);
  assert(x1.n == dx1.n);
  assert(x0.n == x1.n);
  assert(x0.n == amax.n);
  assert(x0.n == vmax.n);
  Real endTime = 0;
  ParabolicRamp1D temp;
  for(int i=0;i<x0.n;i++) {
    temp.x0=x0[i];
    temp.x1=x1[i];
    temp.dx0=dx0[i];
    temp.dx1=dx1[i];
    if(vmax[i]==0 || amax[i]==0) {
      if(!FuzzyEquals(x0[i],x1[i],EpsilonX)) {
	printf("index %d vmax = %g, amax = %g, X0 != X1 (%g != %g)\n",i,vmax[i],amax[i],x0[i],x1[i]);
	return -1;
      }
      if(!FuzzyEquals(dx0[i],dx1[i],EpsilonV)) {
	printf("index %d vmax = %g, amax = %g, DX0 != DX1 (%g != %g)\n",i,vmax[i],amax[i],dx0[i],dx1[i]);
	return -1;
      }
      continue;
    }
    if(!temp.SolveMinTime(amax[i],vmax[i])) return -1;
    if(temp.ttotal > endTime) endTime = temp.ttotal;
  }
  return endTime;
}

void ParabolicRampND::Evaluate(Real t,Vector& x) const
{
  x.resize(ramps.size());
  for(size_t j=0;j<ramps.size();j++)
    x[j]=ramps[j].Evaluate(t);
}

void ParabolicRampND::Derivative(Real t,Vector& x) const
{
  x.resize(ramps.size());
  for(size_t j=0;j<ramps.size();j++)
    x[j]=ramps[j].Derivative(t);
}

void ParabolicRampND::Accel(Real t,Vector& x) const
{
  x.resize(ramps.size());
  for(size_t j=0;j<ramps.size();j++)
    x[j]=ramps[j].Accel(t);
}

void ParabolicRampND::Output(Real dt,std::vector<Vector>& path) const
{
  assert(!ramps.empty());
  int size = (int)ceil(endTime/dt)+1;
  path.resize(size);
  if(size == 1) {
    path[0].resize(ramps.size());
    for(size_t j=0;j<ramps.size();j++)
      path[0][j] = ramps[j].x0;
    return;
  } 
  for(int i=0;i<size;i++) {
    Real t=endTime*Real(i)/Real(size-1);
    path[i].resize(ramps.size());
    for(size_t j=0;j<ramps.size();j++)
      path[i][j]=ramps[j].Evaluate(t);
  }
  
  /*
  path[0].resize(ramps.size());
  for(size_t j=0;j<ramps.size();j++)
    path[0][j] = ramps[j].x0;
  for(int i=1;i+1<size;i++) {
    Real t=endTime*Real(i)/Real(size-1);
    path[i].resize(ramps.size());
    for(size_t j=0;j<ramps.size();j++)
      path[i][j]=ramps[j].Evaluate(t);
  }
  path[size-1].resize(ramps.size());
  for(size_t j=0;j<ramps.size();j++)
    path[size-1][j] = ramps[j].x1;
  */
}


void ParabolicRampND::Dilate(Real timeScale)
{
  for(size_t i=0;i<ramps.size();i++)
    ramps[i].Dilate(timeScale);
}

void ParabolicRampND::TrimFront(Real tcut)
{
  assert(tcut <= endTime);
  Evaluate(tcut,x0);
  Derivative(tcut,dx0);
  endTime -= tcut;
  for(size_t i=0;i<ramps.size();i++)
    ramps[i].TrimFront(tcut);
  assert(IsValid());
}

void ParabolicRampND::TrimBack(Real tcut)
{
  assert(tcut <= endTime);
  Evaluate(endTime-tcut,x1);
  Derivative(endTime-tcut,dx1);
  endTime -= tcut;
  for(size_t i=0;i<ramps.size();i++)
    ramps[i].TrimBack(tcut);
  assert(IsValid());
}

bool ParabolicRampND::IsValid() const
{
  if(endTime < 0) {
    fprintf(stderr,"ParabolicRampND::IsValid(): endTime is negative\n");
    return false;
  }
  if(x0.n == 0) {
    fprintf(stderr,"ParabolicRampND::IsValid(): empty\n");
    return false;
  }
  if(x1.n != x0.n) {
    fprintf(stderr,"ParabolicRampND::IsValid(): incorrect size of x1\n");
    return false;
  }
  if(dx0.n != x0.n) {
    fprintf(stderr,"ParabolicRampND::IsValid(): incorrect size of dx0\n");
    return false;
  }
  if(dx1.n != x0.n) {
    fprintf(stderr,"ParabolicRampND::IsValid(): incorrect size of dx1\n");
    return false;
  }
  if((int)ramps.size() != x0.n) {
    fprintf(stderr,"ParabolicRampND::IsValid(): incorrect size of ramps, %d vs %d\n",ramps.size(),x0.n);
    return false;
  }
  for(size_t i=0;i<ramps.size();i++) {
    if(!ramps[i].IsValid()) {
      fprintf(stderr,"ParabolicRampND::IsValid(): element %d is invalid\n",i);
      return false;
    }
    if(!FuzzyEquals(ramps[i].ttotal,endTime,EpsilonT)) {
      fprintf(stderr,"ParabolicRampND::IsValid(): element %d has different end time %g != %g\n",i,ramps[i].ttotal,endTime);
      return false;
    }
    if(!FuzzyEquals(ramps[i].x0,x0[i],EpsilonX)) {
      fprintf(stderr,"ParabolicRampND::IsValid(): element %d has different x0 %g != %g\n",i,ramps[i].x0,x0[i]);
      return false;
    }
    if(!FuzzyEquals(ramps[i].x1,x1[i],EpsilonX)) {
      fprintf(stderr,"ParabolicRampND::IsValid(): element %d has different x1 %g != %g\n",i,ramps[i].x1,x1[i]);
      return false;
    }
    if(!FuzzyEquals(ramps[i].dx0,dx0[i],EpsilonV)) {
      fprintf(stderr,"ParabolicRampND::IsValid(): element %d has different dx0 %g != %g\n",i,ramps[i].dx0,dx0[i]);
      return false;
    }
    if(!FuzzyEquals(ramps[i].dx1,dx1[i],EpsilonV)) {
      fprintf(stderr,"ParabolicRampND::IsValid(): element %d has different dx1 %g != %g\n",i,ramps[i].dx1,dx1[i]);
      return false;
    }
  }
  return true;
}
