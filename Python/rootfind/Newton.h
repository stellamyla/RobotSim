#ifndef OPTIMIZATION_NEWTON_H
#define OPTIMIZATION_NEWTON_H

#include "function.h"
#include "root.h"
#include "SVDecomposition.h"

namespace Math {
  template <class T> class SparseMatrixTemplate_RM;
  typedef SparseMatrixTemplate_RM<Real> SparseMatrix;
}

namespace Optimization {
using namespace Math;

/** @ingroup Optimization
 * @brief A globally convergent Newton's method for multidimensional root
 * solving.  Solves for func(x)=0.
 */
class NewtonRoot
{
public:
  NewtonRoot(VectorFieldFunction* func);
  virtual ~NewtonRoot();
  bool GlobalSolve(int& iters,ConvergenceResult* res=NULL);
  ConvergenceResult Solve(int& iters);
  ConvergenceResult Solve_Sparse(int& iters);
  bool LineMinimization(const Vector& g, const Vector& p, Real *f);
  Real MaxDistance(const Vector& x);
  virtual bool SolveUnderconstrainedLS(const Matrix& A,const Vector& b,Vector& x);
  virtual bool SolveUnderconstrainedLS(const SparseMatrix& A,const Vector& b,Vector& x);
  virtual Real Merit();  //evaluates the merit function at x

  Vector x;
  VectorFieldFunction* func;
  Real tolf,tolmin,tolx;
  Real stepMax;  ///< maximum distance to step
  Real lambda;   ///< damped-least-squares constant 
  Vector bmin,bmax; ///< optional bound constraints
  bool sparse;   ///< set to true if should use a sparse least-squares solver
  int verbose;
  int debug;

  //temporary
  RobustSVD<Real> svd;
  Vector fx, g, p, xold;
  Matrix fJx;
};

///Returns C(x).maxElement()
Real SurfaceDistance(VectorFieldFunction*C,const Vector& x);
///Returns true if C(x)<=tol, pointwise
bool SatisfiesEquality(VectorFieldFunction*C,const Vector& x,Real tol=Epsilon);
///Returns true if C(x) >= margin
bool SatisfiesInequality(VectorFieldFunction*C,const Vector& x,Real margin=Zero);
///Returns C(x).minElement() (and the index if non=NULL)
Real InequalityMargin(VectorFieldFunction* c,const Vector& x,int* index=NULL);

} //namespace Optimization

#endif
