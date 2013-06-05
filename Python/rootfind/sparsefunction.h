#ifndef MATH_SPARSE_FUNCTION_H
#define MATH_SPARSE_FUNCTION_H

#include "function.h"
#include "sparsematrix.h"
#include "sparsevector.h"

/** @ingroup Math
 * @file math/sparsefunction.h
 * @brief Vector field classes with sparse jacobians
 */

namespace Math {

/** @addtogroup Math */
/*@{*/

///A function g(t) that returns f(x+tn) for a scalar field f(x)
class SparseVectorFunction : public VectorFieldFunction
{
public:
  virtual void Jacobian(const Vector& x,Matrix& J) {
    SparseMatrix sJ(J.m,J.n);
    Jacobian_Sparse(x,sJ);
    sJ.get(J);
  }
  virtual void Jacobian_i(const Vector& x,int i,Vector& Ji) {
    SparseVector sJi(Ji.n);
    Jacobian_i_Sparse(x,i,sJi);
    sJi.get(Ji);
  }
  virtual void Hessian_i(const Vector& x,int i,Matrix& Hi) {
    SparseMatrix sHi(Hi.m,Hi.n);
    Hessian_i_Sparse(x,i,sHi);
    sHi.get(Hi);
  }
  virtual void Jacobian_Sparse(const Vector& x,SparseMatrix& J)=0;
  virtual void Jacobian_i_Sparse(const Vector& x,int i,SparseVector& Ji)=0;
  virtual void Hessian_i_Sparse(const Vector& x,int i,SparseMatrix& Hi) {
    FatalError("Hessian_i_Sparse() not defined in subclass of SparseVectorFunction");
  }
};

} //namespace Math

#endif
