/*
This file is part of LMPL.

    LMPL is free software: you can redistribute it and/or modify
    it under the terms of the Lesser GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LMPL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Lesser
    GNU General Public License for more details.

    You should have received a copy of the Lesser GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef DEP_CONSTANTS_H
#define DEP_CONSTANTS_H
#include <cfloat>
#include <math.h>

typedef double Real;

namespace Math {
	const static double Inf = DBL_MAX;
	const static double Zero = 0;
	const static double One = 1;
	const static double Two = 2;
	const static double Half = 0.5;
	const static double Epsilon = 1e-8;
	const static double Pi = 3.1415926535897932384626433832795;
	const static double TwoPi = 6.283185307;

//math.h aliases
inline double Abs(double x) { return fabs(x); }
inline double Sqrt(double x) { return sqrt(x); }
inline double Exp(double x) { return exp(x); }
inline double Log(double x) { return log(x); }
inline double Pow(double x, double y) { return pow(x,y); }
inline double Sin(double x) { return sin(x); }
inline double Cos(double x) { return cos(x); }
inline double Tan(double x) { return tan(x); }
inline double Sinh(double x) { return sinh(x); }
inline double Cosh(double x) { return cosh(x); }
inline double Tanh(double x) { return tanh(x); }
inline double Asin(double x) { return asin(x); }
inline double Acos(double x) { return acos(x); }
inline double Atan(double x) { return atan(x); }
inline double Atan2(double y, double x) { return atan2(y,x); }
inline double Floor(double x) { return floor(x); }
inline double Ceil(double x) { return ceil(x); }
inline double Mod(double x, double y) { return fmod(x,y); }

	inline static int IsInf(double x) {
	  if (x==DBL_MAX) return 1;
	  else if (x==-DBL_MAX) return -1;
	  return 0;
	}
	inline static bool IsNaN(double x) {
		volatile double d = x;
		return d != d;
	}
	inline static bool IsFinite(double x) {
	  return IsInf(x) && !IsNaN(x);
	}
	/// Returns true if a and b are within +/- eps
	inline bool FuzzyEquals(double a, double b, double eps=Epsilon) { return fabs(a-b) <= eps; }
	/// Returns true if a is zero within +/- eps
	inline bool FuzzyZero(double a, double eps=Epsilon) { return fabs(a) <= eps; }
	/// Returns 1/x if x is nonzero, otherwise 0 
	inline double PseudoInv(double x, double eps=Zero) { return (FuzzyZero(x,eps)?Zero:One/x); }


	inline double Sqr(double x) { return x*x; };
	inline double Delta(double i, double j) { return (i==j? One : Zero); };
	inline double Inv(double x) { return One/x; }
	inline double Sign(double x) { return (x>0 ? 1.0 : (x<0 ? -1.0 : 0.0)); }
	inline double Clamp(double x, double a, double b) { return (x<a? a : (x>b? b : x)); }

	/// Computes up to 2 soln's to quadratic equation a^2 x + b x + c = 0.
	/// Returns the number of solutions (0, 1, or 2)
	int quadratic(double a, double b, double c, double& x1, double& x2);
	/// Computes up to 3 soln's to quadratic equation a^3 x + b^2 x + c x + d = 0.
	/// Returns the number of solutions (0 to 3)
	int cubic(double a, double b, double c, double d, double x[3]);
	/// Returns c where a^2+b^2=c^2
	double pythag(double a, double b);
	/// Returns b where a^2+b^2=c^2
	double pythag_leg(double a,double c);
	/// Computes the sinc function sin(x)/x (stable for small x)
	double Sinc(double x);
	/// Computes the derivative of the sinc function
	double Sinc_Dx(double x);
}

template <class T>
inline const T& Max(const T& a,const T& b)
{
  return (a > b ? a : b);
}

template <class T>
inline const T& Min(const T& a,const T& b)
{
  return (a < b ? a : b);
}

template<class type>
inline void Swap(type& a,type& b) { type temp=a; a=b; b=temp; }

#define SafeDelete(x) { if (x) delete x; x=NULL; }

#endif
