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
#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <math.h>
#include <iostream>
#include "Miscellany.h"
#include <stdarg.h>
#include <assert.h>

typedef std::vector<double> Vector;

class VectorOperation{
public:
	inline static double euclideanDistance(const Vector&, const Vector&);
	inline static double lInfDistance(const Vector&, const Vector&);
	inline static void multiply(const Vector&, double, Vector&);
	inline static void divide(const Vector&, double, Vector&);
	inline static void add(const Vector&, const Vector&, Vector&);
	inline static void minus(const Vector&, const Vector&, Vector&);
	inline static double dot(const Vector&, const Vector&);
	inline static double magnitude (const Vector&);
	inline static void set(Vector&, double, double);
	inline static void assign(const Vector&, Vector&);
	inline static void retrieveData(const Vector&, double *);
private:
	inline static void _set(Vector&, size_t, ...);
};

typedef VectorOperation V;

std::istream& operator >> (std::istream& in, Vector& v);

std::ostream& operator << (std::ostream& out, const Vector& v);

inline double VectorOperation::lInfDistance(const Vector & x, const Vector & y){
	size_t l = x.size(), i;
	double sum = 0, tmp;
	assert(l == y.size());
	for(i = 0; i < l; i++){
		tmp = fabs(x[i] - y[i]);
		if (tmp > sum) {
			sum = tmp;
		}
	}
	return sum;
}

inline double VectorOperation::euclideanDistance(const Vector & x, const Vector & y){
	size_t l = x.size(), i;
	double sum = 0;
	assert(l == y.size());
	for(i = 0; i < l; i++){
		sum+= Utilities::square(x[i] - y[i]);
	}
	return sqrt(sum);
}

inline void VectorOperation::multiply(const Vector& op1, double v, Vector& result){
	size_t l = op1.size(), i;
	if(l != result.size()){
		result.resize(l);
	};
	for(i = 0; i < l; i++){
		result[i] = op1[i] * v;
	}
};

inline void VectorOperation::divide(const Vector& op1, double v, Vector& result){
	size_t l = op1.size(), i;
	assert(l == result.size());
	for(i = 0; i < l; i++){
		result[i] = op1[i] / v;
	}
};

inline void VectorOperation::add(const Vector& op1, const Vector& op2, Vector& result){
	size_t l = op1.size(), i;
	assert(l == op2.size());
	if (l > result.size()){
		result.resize(l);
	};
	for(i = 0; i < l; i++){
		result[i] = op1[i] + op2[i];
	}
};

inline void VectorOperation::minus(const Vector& op1, const Vector& op2, Vector& result){
	size_t l = op1.size(), i;
	assert(l == op2.size());
	if (l > result.size()){
		result.resize(l);
	};
	for(i = 0; i < l; i++){
		result[i] = op1[i] - op2[i];
	}
};
inline double VectorOperation::dot( const Vector& op1, const Vector& op2)
{
	size_t l = op1.size(), i;
	double result = 0;
	assert(l == op2.size());

	for(i = 0; i < l; i++){
		result += op1[i] * op2[i];
	}
	return result;
}
inline double VectorOperation::magnitude(const Vector & x){
	size_t l = x.size(), i;
	double sum = 0;
	for(i = 0; i < l; i++){
		sum+= Utilities::square(x[i]);
	}
	return sqrt(sum);
}
inline void VectorOperation::_set(Vector& op, size_t length, ...){
	va_list   arg_ptr;
	if (length > op.size()){
		op.resize(length);
	}
	va_start(arg_ptr, length);

	do{
		length--;
		op[length]=va_arg(arg_ptr, double);
	}while(length);

	va_end(arg_ptr);
}


inline void VectorOperation::set(Vector& op, double v1, double v2){
	_set(op, 2, v1, v2);
}
inline void VectorOperation::assign(const Vector& from, Vector& to){
	size_t l = from.size();
	if (l > to.size()){
		to.resize(l);
	}
	to.assign(from.begin(), from.end());
}

inline void VectorOperation::retrieveData(const Vector& vector, double * data){
	size_t l = vector.size(), i;
	for (i = 0; i < l; i++){
		data[i] = vector[i];
	}
}
#endif
