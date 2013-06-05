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
#include "CSpace.h"
#include "Random.h"
#include "Vector.h"
using namespace std;

void CSpace::SampleNeighborhood(const Config& c,double r,Config& x)
{
	int l = c.size();
  x.resize(l);
  for(int i=0;i < l;i++)
	  x[i] = c[i] + RandHelper::randWithRange(-r,r);
}

void CSpace::Interpolate(const Config& x, const Config& y, double u, Config& out)
{
  Config tmp;
  tmp.resize(x.size());

  VectorOperation::multiply(x, ConstantHelper::One - u, out);
  VectorOperation::multiply(y, u, tmp);
  VectorOperation::add(out, tmp, out);
  /*out.mul(x,One-u);
  out.madd(y,u);*/
}


