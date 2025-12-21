#ifndef rosic_NumberManipulations_h
#define rosic_NumberManipulations_h

// rosic includes:
#include "GlobalDefinitions.h"
#include <math.h>

namespace rosic
{

  /** \todo: check the linux versions....they have been inserted here sloppily in order to make it compile when
  porting. */



  /** Assuming, that the FPU is in 'to nearest even integer' rounding mode (which is the default),
  this function rounds to the nearest integer using upward rounding when the argument is exactly
  halfway between two integers (instead of returning the nearest even integer in this case).
  Argument x must satify (INT_MIN/2)ñ1.0 < x < (INT_MAX/2)+1.0.  */
  INLINE int roundToInt(double x)
  {
#  if defined _MSC_VER && _MSC_VER <= 1700 // inline _asm deprecated from vs2012 and removed on vs2013
    const float round_to_nearest = 0.5f;
    int i;
     __asm
     {
       fld x;
       fadd st, st (0);
       fadd round_to_nearest;
       fistp i;
       sar i, 1;
     }
     return (i);
    /*
#  elif defined __GNUC__  // too many memory references for 'add', too many memory references for 'sar'
    const float round_to_nearest = 0.5f;
    int i;
    asm("fld x");
    asm("fadd st, st(0)");
    asm("fadd round_to_nearest");
    asm("fistp i");
    asm("sar i, 1");
    return (i);
     */
#  else
    double xFloor = floor(x);
    double xFrac  = x-xFloor;
    if( xFrac >= 0.5 )
      return (int) xFloor + 1;
    else
      return (int) xFloor;
#  endif
  }


  INLINE int floorInt(double x)
  {
#  if defined _MSC_VER && _MSC_VER <= 1700 // inline _asm deprecated from vs2012 and removed on vs2013
    const float round_towards_m_i = -0.5f;
    int i;
    __asm
    {
      fld x;
      fadd st, st (0);
      fadd round_towards_m_i;
      fistp i;
      sar i, 1;
    }
    return (i);
    /*
#  elif defined __GNUC__  // under Linux, it says: memory input 2 is not directly addressable
    const float round_towards_m_i = -0.5f;
    int i;
    __asm__ __volatile__
    (
    "fldl     %1             \n\t"
    "fadd     %%st(0), %%st  \n\t"
    "fadds    %2             \n\t"
    "fistpl   %0             \n\t"
    "sarl     %0             \n\t"
    : "=m"(i)
    : "m"(x), "m"(round_towards_m_i)
    : "memory"
    );
    return (i);
#  elif defined __GNUC__  // too many memory references for 'add', too many memory references for 'sar'
    const float round_towards_m_i = -0.5f;
    int i;
    asm("fld x");
    asm("fadd st, st (0)");
    asm("fadd round_towards_m_i");
    asm("fistp i");
    asm("sar i, 1");
    return (i);
     */
#  else
     return (int) floor(x);
#  endif
  }

} // end namespace rosic

#endif // #ifndef rosic_NumberManipulations_h
