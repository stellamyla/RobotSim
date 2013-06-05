#if 0

#include "matrix.h"
#include "fastmatrix.h"
#include "utils/stringutils.h"
using namespace std;

namespace Math {

extern const char* MatrixError_IncompatibleDimensions;
extern const char* MatrixError_ArgIncompatibleDimensions;
extern const char* MatrixError_DestIncompatibleDimensions;
extern const char* MatrixError_SizeZero;
extern const char* MatrixError_NotSquare;
extern const char* MatrixError_NotSymmetric;
extern const char* MatrixError_InvalidRow;
extern const char* MatrixError_InvalidCol;

#define CHECKDIMS(am,an) if(!hasDims(am,an)) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,am,an);
#define CHECKARGDIMS(b,am,an) if(!b.hasDims(am,an)) RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
#define CHECKDESTDIMS(am,an) if(!hasDims(am,an)) RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
#define CHECKEMPTY() if(m == 0 || n == 0) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
#define CHECKSQUARE() if(!isSquare()) RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);
#define CHECKROW(i) if(!isValidRow(i)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidRow,i);
#define CHECKCOL(j) if(!isValidCol(j)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidCol,j);
#define CHECKMATROW(mat,i) if(!mat.isValidRow(i)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidRow,i);
#define CHECKMATCOL(mat,j) if(!mat.isValidCol(j)) RaiseErrorFmt(WHERE_AM_I,MatrixError_InvalidCol,j);



Matrix::Matrix()
:m(0), n(0), vals(NULL)
{}

Matrix::Matrix(const Matrix& m)
:m(0), n(0), vals(NULL)
{
	operator = (m);
}

Matrix::Matrix(int _m, int _n)
:m(0), n(0), vals(NULL)
{
	resize(_m,_n);
}

Matrix::Matrix(int _m, int _n, Real initval)
:m(0), n(0), vals(NULL)
{
	resize(_m,_n);
	set(initval);
}

Matrix::Matrix(int _m, int _n, const Real* _vals)
:m(0), n(0), vals(NULL)
{
	resize(_m,_n);
	set(_vals);
}

Matrix::Matrix(int _m, int _n, const Real** _vals)
:m(0), n(0), vals(NULL)
{
	resize(_m,_n);
	set(_vals);
}

Matrix::Matrix(int _m, int _n, const Vector* rows)
:m(0), n(0), vals(NULL)
{
	resize(_m,_n);
	setRows(rows);
}


Matrix::~Matrix()
{
	clear();
}

void Matrix::clear()
{
	SafeDeleteProc(vals, free);
	m = n = 0;
}

void Matrix::resize(int _m, int _n)
{
	if(!hasDims(_m,_n))
	{
		SafeDeleteProc(vals, free);
		m = _m;
		n = _n;

		if(_m > 0 && _n > 0)
		{
			vals = matrix_create(m,n);
		}
		else if(_m != 0 || _n != 0)
		{
			RaiseErrorFmt(WHERE_AM_I,"Invalid matrix size %d x %d",_m,_n);
		}
	}
}

void Matrix::resize(int _m, int _n, Real initval)
{
	resize(_m, _n);
	set(initval);
}

const Matrix& Matrix::operator = (const Matrix& a)
{
	set(a);
	return *this;
}

bool Matrix::operator == (const Matrix& a) const
{
  if(!hasDims(a.m,a.n)) return false;
  const Real *x=vals[0],*y=a.vals[0];
  int mn=m*n;
  for(int i=0;i<mn;i++,x++,y++)
    if(*x!=*y) return false;
  return true;
}

void Matrix::operator += (const Matrix& a)
{
  CHECKDIMS(a.m,a.n);
  vector_acc(vals[0],a.vals[0],m*n);
}

void Matrix::operator -= (const Matrix& a)
{
  CHECKDIMS(a.m,a.n);
  vector_dec(vals[0],a.vals[0],m*n);
}

void Matrix::operator *= (const Matrix& a)
{
	Matrix tmp(*this);
	mul(tmp, a);
}

void Matrix::operator *= (Real c)
{
	inplaceScale(c);
}

void Matrix::operator /= (Real c)
{
	inplaceScale(Inv(c));
}

void Matrix::add(const Matrix& a, const Matrix& b)
{
  CHECKARGDIMS(b,a.m,a.n);
	if(isEmpty())
		resize(a.m,a.n);
	else
	  CHECKDESTDIMS(a.m,a.n);

	matrix_add(vals, a.vals, b.vals, m, n);
}

void Matrix::sub(const Matrix& a, const Matrix& b)
{
  CHECKARGDIMS(b,a.m,a.n);
	if(isEmpty())
		resize(a.m,a.n);
	else 
	  CHECKDESTDIMS(a.m,a.n);

	matrix_sub(vals, a.vals, b.vals, m, n);
}

void Matrix::mul(const Matrix& a, const Matrix& b)
{
	if(b.m != a.n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
	}
	if(isEmpty())
	{
		resize(a.m,b.n);
	}
	else {
	  CHECKDESTDIMS(a.m,b.n);
	}

	matrix_multiply(vals, a.vals, b.vals, m, a.n, n);
}

void Matrix::mulTransposeA(const Matrix& a, const Matrix& b)
{
	if(b.m != a.m)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
	}
	if(isEmpty())
	{
		resize(a.n,b.n);
	}
	else {
	  CHECKDESTDIMS(a.n,b.n);
	}

	matrix_multiply_transposeA(vals, a.vals, b.vals, m, a.m, b.n);
}

void Matrix::mulTransposeB(const Matrix& a, const Matrix& b)
{
	if(b.n != a.n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
	}
	if(isEmpty())
	{
		resize(a.m,b.m);
	}
	else {
	  CHECKDIMS(a.m,b.m);
	}

	matrix_multiply_transposeB(vals, a.vals, b.vals, m, a.n, b.m);
}

void Matrix::mul(const Vector& a, Vector& b) const
{
	if(n != a.n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
	}
	if(b.n == 0)
	{
		b.resize(m);
	}
	else if(b.n != m)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
	}

	matrix_vector_multiply(b.vals, vals, a.vals, m, n);
}

void Matrix::mulTranspose(const Vector& a, Vector& b) const
{
	if(m != a.n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
	}
	if(b.n == 0)
	{
		b.resize(n);
	}
	else if(b.n != n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
	}

	matrix_vector_multiply_transpose(b.vals, vals, a.vals, m, n);
}

void Matrix::mul(const Matrix& a, Real c)
{
	if(isEmpty())
		resize(a.m, a.n);
	else 
	  CHECKDIMS(a.m,a.n);

	vector_multiply(vals[0], a.vals[0], c, m*n);
}

void Matrix::div(const Matrix& a, Real c)
{
	mul(a,Inv(c));
}

void Matrix::madd(const Matrix& a, Real c)
{
  CHECKDIMS(a.m,a.n);
  vector_madd(vals[0], a.vals[0], c, m*n);
}

void Matrix::madd(const Vector& a, Vector& b) const
{
	if(n != a.n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
	}
	if(b.n == 0)
	{
		b.resize(m);
	}
	else if(b.n != m)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
	}

	matrix_vector_madd(b.vals,vals,a.vals,m,n);
}

void Matrix::maddTranspose(const Vector& a, Vector& b) const
{
	if(m != a.n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_ArgIncompatibleDimensions);
	}
	if(b.n == 0)
	{
		b.resize(n);
	}
	else if(b.n != n)
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_DestIncompatibleDimensions);
	}

	matrix_vector_madd_transpose(b.vals,vals,a.vals,m,n);
}

void Matrix::set(const Matrix& a)
{
  if(this == &a) return;
  if(isEmpty())
    resize(a.m, a.n);
  else 
    CHECKDIMS(a.m,a.n);
  matrix_equal(vals, a.vals, m, n);
}

void Matrix::set(Real c)
{
  CHECKEMPTY();
  vector_fill(vals[0],c,m*n);
}

void Matrix::set(const Real* _vals)
{
  CHECKEMPTY();
  vector_equal(vals[0],_vals,m*n);
}

void Matrix::set(const Real** _vals)
{
  CHECKEMPTY();
  matrix_equal(vals,(const matrix_type)_vals,m,n);
}

void Matrix::setRows(const Vector* rows)
{
  CHECKEMPTY();

	for(int i=0; i<m; i++)
	{
		if(rows[i].n != n)
		{
			RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,-1,rows[i].n);
		}
		rows[i].get(vals[i]);
	}
}

void Matrix::setCols(const Vector* cols)
{
  CHECKEMPTY();
	for(int j=0; j<n; j++)
	{
		if(cols[j].n != m)
		{
			RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,cols[j].n,-1);
		}
		//skip n for each entry
		gen_vector_equal(&vals[0][j],n,cols[j],1,m);
	}
}

void Matrix::setZero()
{
  CHECKEMPTY();
  vector_zero(vals[0], m*n);
}

void Matrix::setIdentity()
{
  CHECKEMPTY();
  CHECKSQUARE();

  vector_zero(vals[0],m*n);
  //skip n+1 for each entry
  gen_vector_fill(vals[0],n+1,One,n);
}

void Matrix::setNegative(const Matrix& a)
{
	if(isEmpty())
	{
		resize(a.m,a.n);
	}
	else {
	  CHECKDIMS(a.m,a.n);
	}

	vector_negate(vals[0],a.vals[0],m*n);
}

void Matrix::setTranspose(const Matrix& a)
{
	if(isEmpty())
	{
		resize(a.n,a.m);
	}
	else {
	  CHECKDIMS(a.n,a.m);
	}

	matrix_transpose(vals, a.vals, m, n);
}

void Matrix::setInverse(const Matrix& m)
{
  CHECKSQUARE();
	printf("Inverse not done yet");
	Abort();
}

void Matrix::setSubMatrix(int i, int j, const Matrix& a)
{
  CHECKROW(i);
  CHECKCOL(j);
  CHECKROW(i+a.m-1);
  CHECKCOL(j+a.n-1);

	for(int p=0; p<a.m; p++)
		vector_equal(vals[p+i]+j, a.vals[p], a.n);
}

void Matrix::setDiagonal(const Vector& v)
{
	if(isEmpty())
	{
		resize(v.n,v.n);
	}
	else {
	  CHECKSQUARE();
	  CHECKDIMS(v.n,v.n);
	}
	setZero();
	//skip n+1 entries
	gen_vector_equal(vals[0],n+1,v,1,n);
}

void Matrix::setDiagonal(Real c)
{
  CHECKEMPTY();
  CHECKSQUARE();
  setZero();
  gen_vector_fill(vals[0],n+1,c,n);
}

void Matrix::inplaceNegative()
{
  CHECKEMPTY();
  vector_negate(vals[0],vals[0],m*n);
}

void Matrix::inplaceScale(Real c)
{
  CHECKEMPTY();
	vector_scale(vals[0], c, m*n);
}

void Matrix::inplaceTranspose()
{
  CHECKEMPTY();
  CHECKSQUARE();

	Real tmp;
	for(int i=0; i<m; i++)
	{
		for(int j=0; j<m; j++)
		{
			tmp = vals[i][j];
			vals[i][j] = vals[j][i];
			vals[j][i] = tmp;
		}
	}
}

void Matrix::inplaceInverse()
{
	Matrix tmp(*this);
	setInverse(tmp);
}

void Matrix::getSubMatrix(int i, int j, Matrix& a) const
{
  CHECKROW(i);
  CHECKCOL(j);
  CHECKROW(i+a.m-1);
  CHECKCOL(j+a.n-1);

	for(int p=0; p<a.m; p++)
		vector_equal(a.vals[p], vals[p+i]+j, a.n);
}

void Matrix::getDiagonal(Vector& v) const
{
  CHECKSQUARE();
	if(v.isEmpty())
		v.resize(m);
	else if(!v.hasDims(m))
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,v.n,v.n);
	}
	//skip n+1 entries
	gen_vector_equal(v,1,vals[0],n+1,n);
}


bool Matrix::isIdentity() const
{
  CHECKEMPTY();
	if(!isSquare()) return false;

	int i,j;
	for(i=0; i<m; i++)
		for(j=0; j<n; j++)
			if(!FuzzyEquals(Delta(i,j),vals[i][j]))
				return false;
	return true;
}

bool Matrix::isDiagonal() const
{
  CHECKEMPTY();
	if(!isSquare()) return false;

	int i,j;
	for(i=0; i<m; i++)
		for(j=0; j<n; j++)
			if(i!=j && !FuzzyEquals(vals[i][j],Zero))
				return false;
	return true;
}

bool Matrix::isSymmetric() const
{
  CHECKEMPTY();
	if(!isSquare()) return false;

	int i,j;
	for(i=0; i<m; i++)
		for(j=0; j<i; j++)
			if(!FuzzyEquals(vals[i][j],vals[j][i]))
				return false;
	return true;
}

bool Matrix::isZero() const
{
  CHECKEMPTY();
	int i,j;
	for(i=0; i<m; i++)
		for(j=0; j<n; j++)
			if(!FuzzyEquals(vals[i][j],Zero))
				return false;
	return true;
}


inline bool Matrix::isOrthogonal() const
{
  CHECKEMPTY();

	for(int i=0; i<m; i++)
	{
		if(!FuzzyEquals(vector_dot(vals[i], vals[i], n), One))
			return false;
		for(int j=0; j<i; j++)
			if(!FuzzyEquals(vector_dot(vals[i], vals[j], n), Zero))
				return false;
	}
	return true;
}

bool Matrix::isDiagonallyDominant() const
{
  if(!isSquare()) return false;
  //diagonally dominant
  for(int i=0;i<m;i++) {
    Real sum = Zero;
    for(int j=0;j<n;j++)
      if(j!=i) sum += Abs(vals[i][j]);
    if(sum > Abs(vals[i][i])) return false;
  }
  return true;
}



void Matrix::setRow(int i, Real c)
{
  CHECKROW(i);
  vector_fill(vals[i],c,n);
}

void Matrix::setCol(int j, Real c)
{
  CHECKCOL(j);
  gen_vector_fill(&vals[0][j],n, c, m);
}

void Matrix::setRow(int i, const Vector& v)
{
  CHECKROW(i);
  if(v.n != n) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,-1,v.n);
  v.get(vals[i]);
}

void Matrix::setRow(int i, const Real* v)
{
  CHECKROW(i);
  //for(int j=0;j<n;j++)
  //vals[i][j] = v[j];
  vector_equal(vals[i],v,n); //for some reason this doesn't compile
}

void Matrix::setCol(int j, const Vector& v)
{
  CHECKCOL(j);
  if(v.n != m) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,v.n,-1);
  gen_vector_equal(&vals[0][j],n,v,1,m);
}

void Matrix::setCol(int j, const Real* v)
{
  CHECKCOL(j);
  gen_vector_equal(&vals[0][j],n,v,1,m);
}

void Matrix::getRow(int i, Vector& v) const
{
  CHECKROW(i);
	if(v.isEmpty())
		v.resize(n);
	else if(!v.hasDims(n))
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,-1,v.n);
	}
	v.set(vals[i]);
}

void Matrix::getCol(int j, Vector& v) const
{
  CHECKCOL(j);

	if(v.isEmpty())
		v.resize(m);
	else if(!v.hasDims(m))
	{
		RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,v.n,-1);
	}
	gen_vector_equal(v,1,&vals[0][j],n,m);
}

void Matrix::addRow(int i,const Vector& v) const
{
  CHECKROW(i);
  if(!v.hasDims(n))
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,-1,v.n);
  vector_acc(vals[i],v,n);
}

void Matrix::addCol(int j,const Vector& v) const
{
  CHECKCOL(j);
  if(!v.hasDims(m))
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,v.n,-1);
  gen_vector_acc(&vals[0][j],n,v,1,m);
}

void Matrix::addRow(int i,const Matrix& mat,int im) const
{
  CHECKROW(i);
  CHECKMATROW(mat,im);
  if(n != mat.n)
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,-1,n,-1,mat.n);
  vector_acc(vals[i],mat.vals[i],n);
}

void Matrix::addCol(int j,const Matrix& mat,int jm) const
{
  CHECKCOL(j);
  CHECKMATCOL(mat,jm);
  if(m != mat.m) 
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,-1,mat.m,-1);
  gen_vector_acc(&vals[0][j],n,&mat.vals[0][jm],mat.n,m);
}

void Matrix::scaleRow(int i,Real c) const
{
  CHECKROW(i);
  vector_scale(vals[i],c,n);
}

void Matrix::scaleCol(int j,Real c) const
{
  CHECKCOL(j);
  gen_vector_scale(&vals[0][j],n,c,m);
}

void Matrix::maddRow(int i,const Vector& v,Real c) const
{
  CHECKROW(i);
  if(!v.hasDims(n))
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,-1,v.n);
  vector_madd(vals[i],v,c,n);
}

void Matrix::maddCol(int j,const Vector& v,Real c) const
{
  CHECKCOL(j);
  if(!v.hasDims(m))
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,v.n,-1);
  gen_vector_madd(&vals[0][j],n,v,1,c,m);
}

void Matrix::maddRow(int i,const Matrix& mat,int im,Real c) const
{
  CHECKROW(i);
  CHECKMATROW(mat,im);
  if(n != mat.n)
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,-1,n,-1,mat.n);
  vector_madd(vals[i],mat.vals[i],c,n);
}

void Matrix::maddCol(int j,const Matrix& mat,int jm,Real c) const
{
  CHECKCOL(j);
  CHECKMATCOL(mat,jm);
  if(m != mat.m) 
    RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,-1,mat.m,-1);
  gen_vector_madd(&vals[0][j],n,&mat.vals[0][jm],mat.n,c,m);
}

Real Matrix::dotRow(int i,const Vector& v) const
{
  CHECKROW(i);
  if(v.n != n) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,-1,v.n);
  return vector_dot(vals[i],v.vals,n);
}

Real Matrix::dotCol(int j,const Vector& v) const
{
  CHECKCOL(j);
  if(v.n != m) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,n,v.n,-1);
  return gen_vector_dot(&vals[0][j],n,v,1,m);
}

Real Matrix::dotRow(int i,const Matrix& mat,int im) const
{
  CHECKROW(i);
  CHECKMATROW(mat,im);
  if(mat.n != n) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,-1,n,-1,mat.n);
  return vector_dot(vals[i],mat.vals[im],n);
}

Real Matrix::dotCol(int j,const Matrix& mat,int jm) const
{
  CHECKCOL(j);
  CHECKMATCOL(mat,jm);
  if(mat.m != m) RaiseErrorFmt(WHERE_AM_I,MatrixError_IncompatibleDimensions,m,-1,mat.m,-1);
  return gen_vector_dot(&vals[0][j],n,&mat.vals[0][jm],mat.n,m);
}

Real Matrix::trace() const
{
  if(isEmpty()) return Zero;
  if(!isSquare()) RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);
  Real tr=Zero;
  for(int i=0;i<m;i++) tr += vals[i][i];
  return tr;
}

Real Matrix::determinant() const
{
	if(isEmpty()) return Zero;
	if(!isSquare()) RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);

	printf("Haven't completed the determinant\n");
	Abort();
	/*
	LU_Decomposition(*this, L,U, P);
	return diagonalProduct(L) * diagonalProduct(U) * signature(P);
	*/
	return Zero;
}

Real Matrix::diagonalProduct() const
{
  if(isEmpty()) return One;
  if(!isSquare()) RaiseErrorFmt(WHERE_AM_I,MatrixError_NotSquare);
  Real dp=One;
  for(int i=0;i<m;i++) dp *= vals[i][i];
  return dp;
}

Real Matrix::minElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  Real* v = &vals[0][0];
  Real val=*v;
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++,v++)
      if(*v < val) {
	val = *v;
	if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

Real Matrix::maxElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  Real* v = &vals[0][0];
  Real val=*v;
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++,v++)
      if(*v > val) {
 	val = *v;
	if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

Real Matrix::minAbsElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  Real* v = &vals[0][0];
  Real val=Abs(*v);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++,v++)
      if(Abs(*v) < val) {
	val = Abs(*v);
	if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

Real Matrix::maxAbsElement(int*_i,int*_j) const
{
  if(isEmpty()) RaiseErrorFmt(WHERE_AM_I,MatrixError_SizeZero);
  if(_i) *_i=0;  if(_j) *_j=0;
  Real* v = &vals[0][0];
  Real val=Abs(*v);
  for(int i=0;i<m;i++)
    for(int j=0;j<n;j++,v++)
      if(Abs(*v) > val) {
 	val = Abs(*v);
	if(_i) *_i=i; if(_j) *_j=j;
      }
  return val;
}

void Matrix::print(ostream& out,char delim,char bracket) const
{
  char closebracket = CloseBracket(bracket);
  if(bracket) out<<bracket;
  for(int i=0; i<m; i++) {
    if(bracket) out<<bracket;
    for(int j=0; j<n; j++)
      out<<vals[i][j]<<delim;
    if(bracket) out<<closebracket;
    if(i+1 != m) out<<endl;
  }
  if(bracket) out<<closebracket;
  out<<endl;
}

bool Matrix::load(File&f)
{
	int _m,_n;
	if(!ReadFile(f,_m)) return false;
	if(!ReadFile(f,_n)) return false;
	resize(_m,_n);
	Real* array=(Real*)(*this);
	if(!ReadArrayFile(f,array,m*n)) return false;
	return true;
}

bool Matrix::save(File&f)
{
	if(!WriteFile(f,m)) return false;
	if(!WriteFile(f,n)) return false;
	Real* array=(Real*)(*this);
	if(!WriteArrayFile(f,array,m*n)) return false;
	return true;
}

ostream& operator << (ostream& out, const Matrix& mat)
{
	out << mat.m << " " << mat.n << "\t";
	for(int i=0; i<mat.m; i++) {
		for(int j=0; j<mat.n; j++)
			out << mat.vals[i][j] << " ";
		out << endl;
	}
	return out;
}

istream& operator >> (istream& in, Matrix& mat)
{
	int m,n;
	in >> m >> n;
	mat.resize(m,n);
	for(int i=0; i<mat.m; i++) {
		for(int j=0; j<mat.n; j++)
			in >> mat.vals[i][j];
	}
	return in;
}

} //namespace Math

#endif
