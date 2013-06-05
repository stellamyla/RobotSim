#include "Miscellany.h"
#include "stdio.h"
#include <assert.h>

namespace Math {

static const double dFour = 4.0;
static const double dTwoPi_3 = TwoPi/3.0;
static const double dThird = 1.0/3.0;

//returns the # of real roots found (-1 if infinite)
int quadratic(double a, double b, double c, double& x1, double& x2) {
  //printf("quadratic %f %f %f\n", a, b, c);

  if(a == 0)
  {
	  if(b == 0)
	  {
		if(c == 0)
			return -1;
		return 0;
	  }
	  x1=-c/b;
	  return 1;
  }
  if(c == 0) { //det = b^2
    x1 = 0;
    x2 = -b/a;
    return 2;
  }

  double det = b*b-dFour*a*c;
  if(det < Zero)
    return 0;
  if(det == Zero) {
    x1 = -b/(2.0*a);
    return 1;
  }
  det = sqrt(det);
  //(-b + d) / (2 a)  = (-b + d)(-b - d)/(2a)(-b - d)
  //= 4ac / (2a) (-b - d) = 2c / (-b - d)
  //(-b - d) / (2 a)  =  (-b - d) (-b + d) / (2a)(-b + d)
  //=  2c / (-b + d)
  //can choose
  //x1 = (-b + d)/2a    or    2c / (-b - d)
  //x2 = (-b - d)/2a    or    2c / (-b + d)
  //what if a and (-b-d) are both close to zero?  (happens when b < 0)
  //what if a and (-b+d) are both close to zero?  (happens when b > 0)
  //in first case: x1 is screwed, x2 is fine
  //in second case: x1 is fine, x2 is screwed
  if(fabs(-b - det) < fabs(a))
    x1 = 0.5 * (-b + det)/a;
  else
    x1 = 2.0 * c / (-b-det);
  if(fabs(-b + det) < fabs(a)) 
    x2 = 0.5 * (-b-det) / a;
  else 
    x2 = 2.0 * c / (-b+det);
  return 2;
}

int cubic(double a, double b, double c, double d, double x[3])
{
  if(a==0) 
    return quadratic(b,c,d,x[0],x[1]);
  if(a != 1) {
    b /= a;
    c /= a;
    d /= a;
  }
  double Q = (Sqr(b)-3*c)/9;
  double R = (2*b*b*b - 9*b*c+ 27*d)/54;
  double Q3 = Q*Q*Q; 
  double b_3 = b*dThird;
  if(R*R < Q3) {
    double sqrtQ = sqrt(Q);
    double theta_3 = acos(R/(sqrtQ*Q))*dThird;
    x[0] = -2.0*sqrtQ*cos(theta_3) - b_3;
    x[1] = -2.0*sqrtQ*cos(theta_3+dTwoPi_3) - b_3;
    x[2] = -2.0*sqrtQ*cos(theta_3-dTwoPi_3) - b_3;
    return 3;
  }
  else {
    double A = -Sign(R)*pow(fabs(R)+sqrt(R*R-Q3),dThird);
    double B = (A==0?0:Q/A);
    x[0] = A+B-b_3;
    return 1;
  }
}

double pythag(double a, double b)		//reduce roundoff of large numbers
{
  double absa = fabs(a);
  double absb = fabs(b);
  if(absa > absb)
    return absa*sqrt(One + Sqr(absb/absa));
  else if(absb == 0)
    return Zero;
  else
    return absb*sqrt(One + Sqr(absa/absb));
}

double pythag_leg(double a,double c)
{
  assert(c >= 0);
  assert(c >= fabs(a));
  if(c == 0) return 0;
  return c*sqrt(One-Sqr(a/c));
}

double Sinc(double x)
{
	const double small=1e-7;
	if(fabs(x) < small) {	//taylor expand around 0
		const static double c[5]={1.0,-1.0/6.0,1.0/120.0,-1.0/5040.0,1.0/362880.0};
		double x2=x*x;
		return c[0]+x2*(c[1]+x2*(c[2]+x2*(c[3]+x2*c[4])));
	}
	else return sin(x)/x;
}

double Sinc_Dx(double x)
{
	const double small=1e-4;
	if(fabs(x) < small) {	//taylor expand around 0
		const static double c[4]={-2.0/6.0,4.0/120.0,-6.0/5040.0,8.0/362880.0};
		double x2=x*x;
		return x*(c[0]+x2*(c[1]+x2*(c[2]+x2*c[3])));
	}
	else return cos(x)/x-sin(x)/(x*x);
}


} //namespace Math
