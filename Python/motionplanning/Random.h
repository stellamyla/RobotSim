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
#ifndef DEP_RANDOM_H
#define DEP_RANDOM_H

#include <stdlib.h>
#include <time.h>

namespace RandHelper
{
  static inline void srand(unsigned seed) {
    ::srand(seed);
  }

  static inline double randWithRange(double a, double b) {
	double t = double(::rand())/double(RAND_MAX);
    return a + t*(b-a);
  }

  static inline double rand() {
	  return double(::rand())/double(RAND_MAX);
  }

  static inline bool randBool() { 
	  return ::rand() < (RAND_MAX/2); 
  }

  static inline bool randBool(double p) { 
	  return rand() < p; 
  }

  
  static inline long int randInt(long int n) { return ::rand()%n; }
};

#endif
