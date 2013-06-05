#ifndef MATH_DEBUG_H
#define MATH_DEBUG_H

#include "VectorTemplate.h"
#include "MatrixTemplate.h"
#include "SparseVectorTemplate.h"
#include "SparseMatrixTemplate.h"

namespace Math {

///returns true if all elements of x are finite
template <class T>
inline bool IsFinite(const VectorTemplate<T>& x)
{
  for(int i=0;i<x.n;i++)
    if(!IsFinite(x(i))) return false;
  return true;
}

///returns true if all elements of A are finite
template <class T>
inline bool IsFinite(const MatrixTemplate<T>& A)
{
  for(int i=0;i<A.m;i++)
    for(int j=0;j<A.n;j++)
      if(!IsFinite(A(i,j))) return false;
  return true;
}

///returns true if all elements of x are finite
template <class T>
inline bool IsFinite(const SparseVectorTemplate<T>& x)
{
  for(typename SparseVectorTemplate<T>::const_iterator i=x.begin();i!=x.end();i++)
    if(!IsFinite(i->second)) return false;
  return true;
}

///returns true if all elements of x are finite
template <class T>
inline bool IsFinite(const SparseMatrixTemplate_RM<T>& x)
{
  for(size_t i=0;i<x.rows.size();i++) {
    typename SparseMatrixTemplate_RM<T>::ConstRowIterator j;
    for(j=x.rows[i].begin();j!=x.rows[i].end();j++)
      if(!IsFinite(j->second)) return false;
  }
  return true;
}

///returns true if any element of x is NaN
template <class T>
inline bool HasNaN(const VectorTemplate<T>& x)
{
  for(int i=0;i<x.n;i++)
    if(IsNaN(x(i))) return true;
  return false;
}

///returns true if any element of A is NaN
template <class T>
inline bool HasNaN(const MatrixTemplate<T>& A)
{
  for(int i=0;i<A.m;i++)
    for(int j=0;j<A.n;j++)
      if(IsNaN(A(i,j))) return true;
  return false;
}

///returns nonzero if any element of x is infinite
template <class T>
inline int HasInf(const VectorTemplate<T>& x)
{
  for(int i=0;i<x.n;i++)
    if(IsInf(x(i))) return IsInf(x(i));
  return 0;
}

///returns nonzero if any element of A is infinite
template <class T>
inline int HasNaN(const MatrixTemplate<T>& A)
{
  for(int i=0;i<A.m;i++)
    for(int j=0;j<A.n;j++)
      if(IsInf(A(i,j))) return IsInf(A(i,j));
  return 0;
}

} //namespace Math

#endif
