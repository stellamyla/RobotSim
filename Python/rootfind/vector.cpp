#if 0

#include "vector.h"
#include "fastvector.h"
#include <utils/stringutils.h>
using namespace std;

namespace Math {

extern const char* VectorError_IncompatibleDimensions;
extern const char* VectorError_ArgIncompatibleDimensions;
extern const char* VectorError_DestIncompatibleDimensions;
extern const char* VectorError_LengthZero;

Vector::Vector()
:n(0), vals(NULL)
{}

Vector::Vector(const Vector& v)
:n(0), vals(NULL)
{
	set(v);
}

Vector::Vector(int _n)
:n(0), vals(NULL)
{
	resize(_n);
}

Vector::Vector(int _n, Real initval)
:n(0), vals(NULL)
{
	resize(_n, initval);
}

Vector::Vector(int _n, const Real* vals)
:n(0), vals(NULL)
{
	resize(_n);
	set(vals);
}

Vector::~Vector()
{
	clear();
}

void Vector::clear()
{
	SafeArrayDelete(vals);
	n = 0;
}

void Vector::resize(int _n)
{
	if(!hasDims(_n))
	{
		SafeArrayDelete(vals);
		n = _n;
		if(n > 0)
			vals = new Real [n];
	}
}

void Vector::resize(int _n, Real initval)
{
	if(_n <= 0) RaiseErrorFmt(WHERE_AM_I,"Resizing vector to size <= 0");
	resize(_n);
	set(initval);
}

const Vector& Vector::operator = (const Vector& v)
{
	set(v);
	return *this;
}

bool Vector::operator == (const Vector& v) const
{
  if(n!=v.n) return false;
  const Real* a=vals;
  const Real* b=v.vals;
  for(int i=0;i<n;i++,a++,b++)
    if(*a!=*b) return false;
  return true;
}

void Vector::operator += (const Vector& v)
{
	if(!hasDims(v.n)) RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);
	vector_acc(vals,v.vals,n);
}

void Vector::operator -= (const Vector& v)
{
	if(!hasDims(v.n)) RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);
	vector_dec(vals,v.vals,n);
}

void Vector::operator *= (Real c)
{
	inplaceScale(c);
}

void Vector::operator /= (Real c)
{
	inplaceScale(Inv(c));
}

void Vector::add(const Vector& a, const Vector& b)
{
	if(!a.hasDims(b.n)) RaiseErrorFmt(WHERE_AM_I,VectorError_ArgIncompatibleDimensions);
	if(isEmpty())
		resize(a.n);
	else if (!hasDims(a.n))
		RaiseErrorFmt(WHERE_AM_I,VectorError_DestIncompatibleDimensions);

	vector_add(vals, a.vals, b.vals, n);
}

void Vector::sub(const Vector& a, const Vector& b)
{
	if(!a.hasDims(b.n)) RaiseErrorFmt(WHERE_AM_I,VectorError_ArgIncompatibleDimensions);
	if(isEmpty())
		resize(a.n);
	else if (!hasDims(a.n))
		RaiseErrorFmt(WHERE_AM_I,VectorError_DestIncompatibleDimensions);

	vector_sub(vals, a.vals, b.vals, n);
}

void Vector::mul(const Vector& a, Real c)
{
	if(isEmpty())
		resize(a.n);
	else if (!hasDims(a.n))
		RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);

	vector_multiply(vals,a.vals,c,n);
}

void Vector::div(const Vector& a, Real c)
{
	mul(a,Inv(c));
}

void Vector::madd(const Vector& a, Real c)
{
	if(isEmpty())
		resize(a.n);
	else if (!hasDims(a.n))
		RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);

	vector_madd(vals,a.vals,c,n);
}

void Vector::axpby(Real a,const Vector& x,Real b,const Vector& y)
{
  if(!x.hasDims(y.n)) 
    RaiseErrorFmt(VectorError_IncompatibleDimensions);
  if(isEmpty())
    resize(x.n);
  else if (!hasDims(x.n))
    RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);
  vector_axpby(vals,a,x.vals,b,y.vals,n);
}

void Vector::set(const Vector& a)
{
  if(this == &a) return;
	if(isEmpty())
		resize(a.n);
	else if (!hasDims(a.n))
		RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);

	vector_equal(vals, a.vals, n);
}

void Vector::set(Real c)
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);

	vector_fill(vals,c,n);
}

void Vector::set(const Real* _vals)
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);

	vector_equal(vals,_vals,n);
}

void Vector::setZero()
{
	set(Zero);
}

void Vector::setNegative(const Vector& a)
{
	if(isEmpty())
		resize(a.n);
	else if (!hasDims(a.n))
		RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);

	vector_negate(vals,a.vals,n);
}

void Vector::setNormalized(const Vector& a)
{
	mul(a, PseudoInv(a.norm()));
}


void Vector::inplaceNegative()
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);

	vector_negate(vals,vals,n);
}

void Vector::inplaceScale(Real c)
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);

	vector_scale(vals,c,n);

}

void Vector::inplaceNormalize()
{
	inplaceScale(PseudoInv(norm()));
}

void Vector::get(Real* _vals) const
{
  vector_equal(_vals,vals,n);
}

bool Vector::isZero(Real eps) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);
  for(int i=0;i<n;i++)
    if(!FuzzyZero(vals[i],eps)) return false;
  return true;
}

bool Vector::isEqual(const Vector& a,Real eps) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);
  if(!hasDims(a.n)) return false;
  for(int i=0;i<n;i++)
    if(!FuzzyEquals(vals[i],a.vals[i],eps)) return false;
  return true;
}

Real Vector::dot(const Vector& a) const
{
	if(!hasDims(a.n)) RaiseErrorFmt(WHERE_AM_I,VectorError_IncompatibleDimensions);
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);

	return vector_dot(vals, a.vals, n);
}

Real Vector::minElement(int* index) const
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);
	Real bmin=vals[0];
	if(index) *index=0;
	for(int i=1;i<n;i++)
	  if(vals[i] < bmin) {
	    bmin=vals[i];
	    if(index) *index=i;
	  }
	return bmin;
}

Real Vector::maxElement(int *index) const
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);
	Real bmax=vals[0];
	if(index) *index=0;
	for(int i=1;i<n;i++)
	  if(vals[i] > bmax) {
	    bmax=vals[i];
	    if(index) *index=i;
	  }
	return bmax;
}

Real Vector::minAbsElement(int* index) const
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);
	Real bmin=Abs(vals[0]);
	if(index) *index=0;
	for(int i=1;i<n;i++)
	  if(Abs(vals[i]) < bmin) {
	    bmin=Abs(vals[i]);
	    if(index) *index=i;
	  }
	return bmin;
}

Real Vector::maxAbsElement(int *index) const
{
	if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,VectorError_LengthZero);
	Real bmax=Abs(vals[0]);
	if(index) *index=0;
	for(int i=1;i<n;i++)
	  if(Abs(vals[i]) > bmax) {
	    bmax=Abs(vals[i]);
	    if(index) *index=i;
	  }
	return bmax;
}

void Vector::print(ostream& out,char delim,char bracket) const
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  for(int i=0; i<n; i++)
    out<<vals[i]<<delim;
  if(bracket) out<<closebracket<<endl;
}

bool Vector::load(File& f)
{
	int _n;
	if(!ReadFile(f, _n)) return false;
	resize(_n);
	if(!ReadArrayFile(f, vals, n)) return false;
	return true;
}

bool Vector::save(File& f)
{
	if(!WriteFile(f, n)) return false;
	if(!WriteArrayFile(f, vals, n)) return false;
	return true;
}

ostream& operator << (ostream& out, const Vector& v)
{
	out << v.n << "\t";
	for(int i=0; i<v.n; i++)
		out << v.vals[i] << " ";
	return out;
}

istream& operator >> (istream& in, Vector& v)
{
	int n;
	in >> n;
	v.resize(n);
	for(int i=0; i<v.n; i++)
		in >> v.vals[i];
	return in;
}

} //namespace Math

#endif
