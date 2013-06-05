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
#include "Vector.h"

std::istream& operator >> (std::istream& in, Vector& v)
{
	int n;
	in >> n;
	if(n != v.size())
		v.resize(n);
	for(int i=0; i < n; i++)
		in >> v[i];
	return in;
}

std::ostream& operator << (std::ostream& out, const Vector& v){
	int n = v.size();
	out << n << "\t";
	for(int i=0; i< n; i++)
		out << v[i] << " ";
	return out;
}

