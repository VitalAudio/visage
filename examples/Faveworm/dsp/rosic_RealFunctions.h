#ifndef rosic_RealFunctions_h
#define rosic_RealFunctions_h

// standard library includes:
#include <math.h>
#include <stdlib.h>

// rosic includes:
#include "GlobalFunctions.h"
#include "rosic_NumberManipulations.h"

namespace rosic
{

  /** Inverse hyperbolic sine. */
  INLINE double asinh(double x);

  /** Returns -1.0 if x is below low, 0.0 if x is between low and high and 1.0 if x is above high. */
  INLINE double belowOrAbove(double x, double low, double high);

  /** Clips x into the range min...max. */
  template <class T>
  INLINE T clip(T x, T min, T max);

  /** Evaluates the quartic polynomial y = a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 at x. */
  INLINE double evaluateQuartic(double x, double a0, double a1, double a2, double a3, double a4);

  /** foldover at the specified value */
  INLINE double foldOver(double x, double min, double max);

  /** Computes an integer power of x by successively multiplying x with itself. */
  INLINE double integerPower(double x, int exponent);

  /** Generates a pseudo-random number between min and max. */
  INLINE double random(double min=0.0, double max=1.0);

  /** Generates a 2*pi periodic saw wave. */
  INLINE double sawWave(double x);

  /** Calculates sine and cosine of x - this is more efficient than calling sin(x) and
  cos(x) seperately. */
  INLINE void sinCos(double x, double* sinResult, double* cosResult);

  /** Calculates a parabolic approximation of the sine and cosine of x. */
  INLINE void sinCosApprox(double x, double* sinResult, double* cosResult);

  /** Generates a 2*pi periodic square wave. */
  INLINE double sqrWave(double x);

  /** Rational approximation of the hyperbolic tangent. */
  INLINE double tanhApprox(double x);

  /** Generates a 2*pi periodic triangle wave. */
  INLINE double triWave(double x);

  //===============================================================================================
  // implementation:

  INLINE double asinh(double x)
  {
    return log(x + sqrt(x*x+1) );
  }

  INLINE double belowOrAbove(double x, double low, double high)
  {
    if( x < low )
      return -1.0;
    else if ( x > high )
      return 1.0;
    else
      return 0.0;
  }

  template <class T>
  INLINE T clip(T x, T min, T max)
  {
    if( x > max )
      return max;
    else if ( x < min )
      return min;
    else return x;
  }

  INLINE double evaluateQuartic(double x, double a0, double a1, double a2, double a3, double a4)
  {
    double x2 = x*x;
    return x*(a3*x2+a1) + x2*(a4*x2+a2) + a0;
  }

  INLINE double foldOver(double x, double min, double max)
  {
    if( x > max )
      return max - (x-max);
    else if( x < min )
      return min - (x-min);
    else return x;
  }

  INLINE double integerPower(double x, int exponent)
  {
    double accu = 1.0;
    for(int i=0; i<exponent; i++)
      accu *= x;
    return accu;
  }

  INLINE double random(double min, double max)
  {
    double tmp = (1.0/RAND_MAX) * rand() ;  // between 0...1
    return linToLin(tmp, 0.0, 1.0, min, max);
  }

  INLINE double sawWave(double x)
  {
    double tmp = fmod(x, 2*PI);
    if( tmp < PI )
      return tmp/PI;
    else
      return (tmp/PI)-2.0;
  }

  INLINE void sinCos(double x, double* sinResult, double* cosResult)
  {
    #if defined __GNUC__ || _MSC_VER > 1700 // \todo assembly-version causes compiler errors on gcc and inline _asm deprecated from vs2012 and removed on vs2013
      *sinResult = sin(x);
      *cosResult = cos(x);
    #else
      double s, c;     // do we need these intermediate variables?
      __asm fld x
      __asm fsincos
      __asm fstp c
      __asm fstp s
      *sinResult = s;
      *cosResult = c;
    #endif
  }

  INLINE void sinCosApprox(double x, double* sinResult, double* cosResult)
  {
    static const double c = 0.70710678118654752440;

    // restrict input x to the range 0.0...2*PI:
    while( x > 2.0*PI )
      x -= 2*PI;
    while( x < 0.0 )
      x += 2*PI;

    if( x < PI/2 )
    {
      double tmp1 = x;
      double tmp2 = (2/PI) * tmp1 - 0.5;
      double tmp3 = (2-4*c)*tmp2*tmp2 + c;
      *sinResult  = tmp3 + tmp2;
      *cosResult  = tmp3 - tmp2;
    }
    else if( x < PI )
    {
      double tmp1 = (x-PI/2);
      double tmp2 = 0.5 - (2/PI) * tmp1;
      double tmp3 = (2-4*c)*tmp2*tmp2 + c;
      *sinResult  = tmp2 + tmp3;
      *cosResult  = tmp2 - tmp3;
    }
    else if( x < 1.5*PI )
    {
      double tmp1 = (x-PI);
      double tmp2 = (2/PI) * tmp1 - 0.5;
      double tmp3 = (4*c-2)*tmp2*tmp2 - c;
      *sinResult  = tmp3 - tmp2;
      *cosResult  = tmp3 + tmp2;
    }
    else
    {
      double tmp1 = (x-1.5*PI);
      double tmp2 = (2/PI) * tmp1 - 0.5;
      double tmp3 = (2-4*c)*tmp2*tmp2 + c;
      *sinResult  = tmp2 - tmp3;
      *cosResult  = tmp2 + tmp3;
    }
  }

  INLINE double sqrWave(double x)
  {
    double tmp = fmod(x, 2*PI);
    if( tmp < PI )
      return 1.0;
    else
      return -1.0;
  }

  INLINE double tanhApprox(double x)
  {
    double a = fabs(2*x);
    double b = 24+a*(12+a*(6+a));
    return 2*(x*b)/(a*b+48);
  }

  INLINE double triWave(double x)
  {
    double tmp = fmod(x, 2*PI);
    if( tmp < 0.5*PI )
      return tmp/(0.5*PI);
    else if( tmp < 1.5*PI )
      return 1.0 - ((tmp-0.5*PI)/(0.5*PI));
    else
      return -1.0 + ((tmp-1.5*PI)/(0.5*PI));
  }

} // end namespace rosic

#endif // #ifndef rosic_RealFunctions_h
