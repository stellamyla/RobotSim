#include "Newton.h"
#include "infnan.h"
#include "debug.h"
#include "CholeskyDecomposition.h"
#include "AABB.h"
#include "LSQRInterface.h"
#include "sparsefunction.h"
using namespace std;
using namespace Optimization;

const static Real kInequalityAdjustment=1e-4;

namespace Optimization {


Real SurfaceDistance(VectorFieldFunction*C,const Vector& x)
{
  Vector temp(C->NumDimensions());
  (*C)(x,temp);
  return temp.maxAbsElement();
}

bool SatisfiesEquality(VectorFieldFunction*C,const Vector& x,Real tol)
{
  Vector temp(C->NumDimensions());
  (*C)(x,temp);
  return temp.maxAbsElement()<=tol;
}

//returns true if C(x) >= margin
bool SatisfiesInequality(VectorFieldFunction*C,const Vector& x,Real margin)
{
  Vector temp(C->NumDimensions());
  (*C)(x,temp);
  return temp.minElement()>=margin;
}

Real InequalityMargin(VectorFieldFunction* c,const Vector& x,int* index)
{
  c->PreEval(x);
  Vector temp(c->NumDimensions());
  c->Eval(x,temp);
  return temp.minElement(index);
}

} //namespace Optimization


NewtonRoot::NewtonRoot(VectorFieldFunction* _func)
  :func(_func),tolf(1e-4),tolmin(1e-6),tolx(1e-7),stepMax(10),lambda(0.01),
   sparse(false),
   verbose(0),debug(0)
{
}

NewtonRoot::~NewtonRoot()
{}

Real NewtonRoot::Merit()
{
  (*func)(x,fx);
  return Half*fx.normSquared(); 
}

Real NewtonRoot::MaxDistance(const Vector& x)
{
  fx.resize(func->NumDimensions());
  (*func)(x,fx);
  return fx.maxAbsElement();
}

bool NewtonRoot::GlobalSolve(int& iters,ConvergenceResult* r)
{
  if(verbose) { cout<<"NewtonRoot::GlobalSolve(): "; cout.flush(); }
  //Vector xinit;
  //xinit.copy(x);
  Real initDist = MaxDistance(x);
  ConvergenceResult res;
  if(sparse)
    res=Solve_Sparse(iters);
  else
    res=Solve(iters);
  if(r) *r=res;
  Real endDist = MaxDistance(x);

  switch(res) {
  case ConvergenceX:
    if(verbose) cout<<"Reached convergence on x... ";
    if(endDist <= tolf) {
      if(verbose) cout<<"satisfies constraint."<<endl;
      return true;
    }
    else {
      if(verbose) cout<<"does not satisfy tolerance, distance "<<endDist<<"."<<endl;
      return false;
    }
    break;

  case LocalMinimum:
    if(verbose) cout<<"Reached local minimum... ";
    if(endDist <= tolf) {
      if(verbose) cout<<"satisfies constraint."<<endl;
      return true;
    }
    else {
      if(verbose) cout<<"stuck at distance "<<endDist<<"."<<endl;
      return false;
    }

  case ConvergenceF:
    if(verbose) cout<<"Reached convergence on f, new distance "<<endDist<<endl;
    assert(endDist <= tolf);
    return true;

  case MaxItersReached:
    if(endDist < initDist) {
      if(verbose) cout<<"Max iters reached, distance was decreased to "<<endDist<<endl;
    }
    else {
      //if(verbose) cout<<"Max iters reached, looks like divergence.  Reverting to initial."<<endl;
      if(verbose) cout<<"Max iters reached, looks like divergence."<<endl;
      //x.copy(xinit);
    }
    return false;
  default:
    if(verbose) cout<<"Error"<<endl;
    return false;
  }
}

bool NewtonRoot::SolveUnderconstrainedLS(const Matrix& A,const Vector& b,Vector& x)
{
  if(sparse) {
    SparseMatrix sA;
    //Real zeroTol=1e-6*A.maxAbsElement();  //tolerance for zero-sized entries in A
    Real zeroTol=Max(1e-6,1e-7*A.maxAbsElement());  //tolerance for zero-sized entries in A
    sA.set(A,zeroTol);
    return SolveUnderconstrainedLS(sA,b,x);
  }
  else {
    svd.resize(A.m,A.n);
    if(verbose>=1 && A.m*A.n>10000) cout<<"Calculating SVD..."<<endl;
    if(svd.set(A)) {
      if(verbose>=1 && A.m*A.n>10000) cout<<"done"<<endl;
      svd.dampedBackSub(b,lambda,x);
      //svd.epsilon = lambda;
      //svd.backSub(vtemp,p);
      return true;
    }
    else {
      //Try least squares
      //(AtA)x = Atb  =>  (AtA)^-1Atb = x
      CholeskyDecomposition<Real> cholesky;
      Matrix At,AtA;
      //using At is better than just A
      At.setTranspose(A);
      AtA.mulTransposeB(At,At);
      
      Vector Atb;
      At.mul(b,Atb);
      if(!cholesky.set(AtA)) {
	return false;
      }
      else {
	cholesky.backSub(Atb,x);
	return true;
      }
    }
  }
}

bool NewtonRoot::SolveUnderconstrainedLS(const SparseMatrix& A,const Vector& b,Vector& x)
{
  Optimization::LSQRInterface lsqr;
  //A.mulTranspose(b,lsqr.x);
  lsqr.dampValue = lambda;
  lsqr.relError = tolx;
  //lsqr.dampValue=0;
  lsqr.verbose=0;
  if(lsqr.Solve(A,b)) {
    if(!IsFinite(lsqr.x)) {
      cerr<<"NewtonRoot::SolveUnderconstrainedLS: Warning, LSQR returned a non-finite solution"<<endl;
      //cerr<<VectorPrinter(lsqr.x,VectorPrinter::AsciiShade)<<endl;
      getchar();
      return false;
    }
    //cout<<"NewtonRoot::SolveUnderconstrainedLS: LSQR residual is "<<lsqr.residualNorm<<endl;
    x=lsqr.x;
    return true;
  }
  //Hmm.. should we try a non-converged x?
  x=lsqr.x;
  return true;
  return false;
}

ConvergenceResult NewtonRoot::Solve(int& iters)
{
  int m=func->NumDimensions();
  fx.resize(m);
  fJx.resize(m,x.n);

  if(bmin.n!=0) AABBClamp(x,bmin,bmax);

  bool check;
  Real f=Merit(); //fx is also computed by this call. 
  if (fx.maxAbsElement() < tolf) { 
    iters=0;
    Real fxmax=fx.maxAbsElement();
    assert(MaxDistance(x) == fxmax);
    return ConvergenceF;
  } 
  Real stpmax= stepMax*Max(x.norm(),(Real)x.n);
  int maxIters = iters;
  for (iters=0;iters<maxIters;iters++) { 
    func->Jacobian(x,fJx);
    fJx.mulTranspose(fx,g);
    xold.copy(x);
    if(!SolveUnderconstrainedLS(fJx,fx,p)) {
      printf("NewtonRoot::Solve: Unable to compute either pseudoinverse or Cholesky least-squares\n");
      return ConvergenceError;
    }
    p.inplaceNegative();
    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    check = LineMinimization(g,p,&f); //lnsrch returns new x and f. It also calculates fx at the new x when it calls Merit()
    //printf("New value of f after lnsrch: %f\n",f);
    //printf("New value of fx after lnsrch: "); cout<<VectorPrinter(fx)<<endl;
    //printf("New value of x after lnsrch: "); cout<<VectorPrinter(x)<<endl;
    //if(c) { if(!c->Satisfies(x)) cout<<"Warning: line searched x doesnt satisfy contact constraints"<<endl; }
    if (fx.maxAbsElement() < tolf) {
      Real fxmax=fx.maxAbsElement();
      assert(MaxDistance(x) == fxmax);
      return ConvergenceF;
    }
    if (check) { //Check for gradient of f zero, i.e., spurious convergence.
      Real test=Zero;
      Real den=Max(f,Half*x.n);
      for (int j=0;j<x.n;j++) { 
	      Real temp=Abs(g[j])*Max(Abs(x[j]),One)/den;
	      if (temp > test) test=temp;
      }
      if(test < tolmin) return LocalMinimum;
      else {
        //cout<<"Hmm.... check is returned on iter "<<iters<<", but test is not < tolmin"<<endl; 
	/*
	cout<<"Converging on x!"<<endl;
	OutputASCIIShade(cout,g); cout<<endl;
	OutputASCIIShade(cout,p); cout<<endl;
        return ConvergenceX;
	*/
	return ConvergenceX;
      }
    }
    Real test=0.0; //Test for convergence on dx.
    for (int j=0;j<x.n;j++) {
      Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
      if (temp > test) test=temp; 
    }
    if (test < tolx) {
      return ConvergenceX;
    }
  } 
  return MaxItersReached;
}

ConvergenceResult NewtonRoot::Solve_Sparse(int& iters)
{
  int m=func->NumDimensions();
  fx.resize(m);
  SparseVectorFunction* sf;
  try {
    sf=dynamic_cast<SparseVectorFunction*>(func);
  }
  catch(exception& e) {
    FatalError("Could not cast VectorFieldFunctions to sparse, exception %s",e.what());    
  }
  SparseMatrix A(m,x.n);

  if(bmin.n!=0) AABBClamp(x,bmin,bmax);

  bool check;
  Real f=Merit(); //fx is also computed by this call. 
  if (fx.maxAbsElement() < tolf) { 
    iters=0;
    Real fxmax=fx.maxAbsElement();
    assert(MaxDistance(x) == fxmax);
    return ConvergenceF;
  } 
  Real stpmax= stepMax*Max(x.norm(),(Real)x.n);
  int maxIters = iters;
  for (iters=0;iters<maxIters;iters++) { 
    sf->Jacobian_Sparse(x,A);
    A.mulTranspose(fx,g);
    xold.copy(x);
    if(!SolveUnderconstrainedLS(A,fx,p)) {
      printf("NewtonRoot::Solve: Unable to compute either pseudoinverse of sparse matrix\n");
      return ConvergenceError;
    }
    p.inplaceNegative();
    Real sum = p.norm();  //Scale if attempted step is too big
    if (sum > stpmax) p.inplaceMul(stpmax/sum);
    check = LineMinimization(g,p,&f); //lnsrch returns new x and f. It also calculates fx at the new x when it calls Merit()
    //printf("New value of f after lnsrch: %f\n",f);
    //printf("New value of fx after lnsrch: "); cout<<VectorPrinter(fx)<<endl;
    //printf("New value of x after lnsrch: "); cout<<VectorPrinter(x)<<endl;
    //if(c) { if(!c->Satisfies(x)) cout<<"Warning: line searched x doesnt satisfy contact constraints"<<endl; }
    if (fx.maxAbsElement() < tolf) {
      Real fxmax=fx.maxAbsElement();
      assert(MaxDistance(x) == fxmax);
      return ConvergenceF;
    }
    if (check) { //Check for gradient of f zero, i.e., spurious convergence.
      Real test=Zero;
      Real den=Max(f,Half*x.n);
      for (int j=0;j<x.n;j++) { 
	      Real temp=Abs(g[j])*Max(Abs(x[j]),One)/den;
	      if (temp > test) test=temp;
      }
      if(test < tolmin) return LocalMinimum;
      else {
        //cout<<"Hmm.... check is returned on iter "<<iters<<", but test is not < tolmin"<<endl; 
	/*
	cout<<"Converging on x!"<<endl;
	OutputASCIIShade(cout,g); cout<<endl;
	OutputASCIIShade(cout,p); cout<<endl;
        return ConvergenceX;
	*/
	return ConvergenceX;
      }
    }
    Real test=0.0; //Test for convergence on dx.
    for (int j=0;j<x.n;j++) {
      Real temp=(Abs(x[j]-xold[j]))/Max(Abs(x[j]),One);
      if (temp > test) test=temp; 
    }
    if (test < tolx) {
      return ConvergenceX;
    }
  } 
  return MaxItersReached;
}



#define ALF 1.0e-4 //Ensures sufficient decrease in function value. 

/*Given an n-dimensional point x0=x, the value of the
  function and gradient there, f and g, and a direction 
  p, finds a new point x along the direction p from
  x0 where the Merit() function has decreased  sufficiently.  The
  new function value is returned in f. stpmax is an input
  quantity that limits the length of the steps so that you do
  not try to evaluate the function in regions where it is
  undetermined or subject to overflow. p is usually the Newton
  direction. The output return value is false (0) on a
  normal exit. It is true (1) when x is too close to x0.
  In a minimization algorithm, this usually signals
  convergence and can be ignored. However, in a zero-finding
  algorithm the calling program should check whether the
  convergence is spurious. Some  difficult problems may require
  double precision in this routine.
*/
bool NewtonRoot::LineMinimization(const Vector& g, const Vector& p, Real *f) 

{ 
  if(debug && !IsFinite(p)) {
    if(verbose) {
      cerr<<"NewtonRoot::LineMinimization: Error, p is not finite!"<<endl;
      //cerr<<"p="<<VectorPrinter(p,VectorPrinter::AsciiShade)<<endl;
      getchar();
    }
    return false;
  }
  if(debug && !IsFinite(g)) {
    if(verbose) {
      cerr<<"NewtonRoot::LineMinimization: Error, g is not finite!"<<endl;
      //cerr<<"g="<<VectorPrinter(g,VectorPrinter::AsciiShade)<<endl;
      getchar();
    }
    return false;
  }
  Real fold = *f;
  xold.copy(x);
  Real f2,slope,tmplam=1.0;
  slope = g.dot(p);
  if (slope >= 0.0) {
    /*if(slope > 0.001) */{
      if(verbose) printf("NewtonRoot::LineMinimization: Opposing slope and descent directions\n");
      return false;
    }
    //  else slope = Abs(slope);
  }
  Real test=Zero; //Compute lambdamin.
  for (int i=0;i<x.n;i++) {
    Real temp=Abs(p[i])/Max(Abs(xold[i]),One);
    if (temp > test) test=temp; 
  }
  Real alamin=tolx/test;
  Real alam=1.0,alam2;
  for (;;) { //Start of iteration loop.
    x.copy(xold); x.madd(p,alam);
    if(bmin.n!=0) {
      AABBClamp(x,bmin,bmax);
    }
    *f=Merit();
    if (alam < alamin) { //Convergence on  x. For zero finding, the calling program should verify the convergence. 
      //x.copy(xold);
      return true;
    }
    else if (*f <= fold+ALF*alam*slope) {
      return false; //Sufficient function decrease. 
    }
    else if(!IsFinite(*f)) {
      cerr<<"NewtonRoot::LineMinimization: f(x) is infinite or NaN... backtracking"<<endl;
      /*
      cerr<<"x0="<<VectorPrinter(xold,VectorPrinter::AsciiShade)<<endl;
      cerr<<"p="<<VectorPrinter(p,VectorPrinter::AsciiShade)<<endl;
      cerr<<"g="<<VectorPrinter(g,VectorPrinter::AsciiShade)<<endl;
      cerr<<"lambda="<<alam<<endl;
      getchar();
      */
      *f = fold;
      tmplam = 0.5*alam;
    }
    else { //Backtrack. 
      if (alam == 1.0) 
	tmplam = -slope/(2.0*(*f-fold-slope)); //First time. 
      else { //Subsequent backtracks. 
	Real rhs1 = *f-fold-alam*slope; 
	Real rhs2=f2-fold-alam2*slope; 
	Real a=(rhs1/(alam*alam)-rhs2/(alam2*alam2))/(alam-alam2); 
	Real b=(-alam2*rhs1/(alam*alam)+alam*rhs2/(alam2*alam2))/(alam-alam2); 
	if (a == 0.0) tmplam = -slope/(2.0*b);
	else { 
	  Real disc=b*b-3.0*a*slope; 
	  if (disc < 0.0) tmplam=0.5*alam;
	  else if (b <= 0.0) tmplam=(-b+Sqrt(disc))/(3.0*a);
	  else tmplam=-slope/(b+Sqrt(disc));
	}
	if(IsNaN(tmplam)) {
	  cerr<<"NewtonRoot::LineMinimization: templam is NaN??"<<endl;
	  cerr<<"f="<<*f<<endl;
	  cerr<<"fold="<<fold<<endl;
	  cerr<<"a="<<a<<endl;
	  cerr<<"b="<<b<<endl;
	  cerr<<"rhs1="<<rhs1<<endl;
	  cerr<<"hrs2="<<rhs2<<endl;
	  cerr<<"slope="<<slope<<endl;
	  getchar();
	  tmplam = 0.5*alam;
	}
	if (tmplam > 0.5*alam) 
	  tmplam=0.5*alam; 
      } 
    }
    alam2=alam;
    f2 = *f; 
    alam=Max(tmplam,(Real)0.1*alam); 
  } //Try again.
}

