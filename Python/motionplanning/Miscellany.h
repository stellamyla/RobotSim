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

namespace ConstantHelper{
	const static double Inf = DBL_MAX;
	const static double Zero = 0;
	const static double One = 1;
	const static double Two = 2;
	const static double Half = 0.5;
	const static double Epsilon = 1e-8;
	const static double Pi = 3.1415926535897932384626433832795;
	const static double TwoPi = 6.283185307;
	inline static bool IsInf(double x) {return x == DBL_MAX;}
	inline static bool IsNaN(double x){
		volatile double d = x;
		return d != d;
	}
	/// Returns true if a and b are within +/- eps
	inline bool FuzzyEquals(double a, double b, double eps=Epsilon) { return fabs(a-b) <= eps; }
	/// Returns true if a is zero within +/- eps
	inline bool FuzzyZero(double a, double eps=Epsilon) { return fabs(a) <= eps; }
	/// Returns 1/x if x is nonzero, otherwise 0 
	inline double PseudoInv(double x, double eps=Zero) { return (FuzzyZero(x,eps)?Zero:One/x); }
}

namespace Utilities{
	using namespace ConstantHelper;
	inline double square(double x) { return x*x; };
	inline double Delta(double i, double j) { return (i==j? ConstantHelper::One : ConstantHelper::Zero); };
	inline double Inv(double x) { return ConstantHelper::One/x; }
	inline double PseduoInv(double x) { return x == Zero? Zero : One/x; }
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

#define SafeDelete(x) { if (x) delete x; x=NULL; }

#endif
