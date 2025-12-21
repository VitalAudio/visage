#ifndef GlobalFunctions_h
#define GlobalFunctions_h

#include <math.h>
#include <stdlib.h>
#include "GlobalDefinitions.h"

/** This file contains a bunch of useful macros and functions which are not wrapped into the
rosic namespace to facilitate their global use. */

/** Converts a raw amplitude value/factor to a value in decibels. */
INLINE double amp2dB(double amp);

/** Converts a raw amplitude value/factor to a value in decibels with a check, if the amplitude is
close to zero (to avoid log-of-zero and related errors). */
INLINE double amp2dBWithCheck(double amp, double lowAmplitude = 0.000001);

/** Returns the index of the maximum value in an array of doubles where the array should be of
length numValues. */
template <class T>
INLINE int arrayMaxIndex(T* theArray, int numValues);

/** Returns the index of the minimum value in an array of doubles where the array should be of
length numValues. */
template <class T>
INLINE int arrayMinIndex(T* theArray, int numValues);

/** Converts a time-stamp given in beats into seconds acording to a tempo measured in beats per
minute (bpm). */
INLINE double beatsToSeconds(double beat, double bpm);

/** Converts a value in decibels to a raw amplitude value/factor. */
INLINE double dB2amp(double x);

/** Converts an angle in degrees into radiant. */
INLINE double degreeToRadiant(double degrees);

/** Frees the memory associated with the pointer ans sets the poiter itself to NULL */
//INLINE void deleteAndNullifyPointer(void *pointer);

/** Returns the Euclidean distance between points at coordinates (x1,y1), (x2,y2). */
INLINE double euclideanDistance(double x1, double y1, double x2, double y2);

/** Calculates the exponential function with base 10. */
INLINE double exp10(double x);

/** Calculates the exponential function with base 2. */
INLINE double exp2(double x);

/** Calculates the factorial of some integer n >= 0. */
//INLINE int factorial(int n);

/** Converts a frequency in Hz into a MIDI-note value assuming A4 = 440 Hz. */
INLINE double freqToPitch(double freq);

/** Converts a frequency in Hz into a MIDI-note value for tunings different than the
default 440 Hz. */
INLINE double freqToPitch(double freq, double masterTuneA4);

/** Checks a pointer for nullity and if it is not NULL, it calls delete for the associated object
and then sets the pointer to NULL. */
INLINE void ifNotNullDeleteAndSetNull(void* pointer);

/** Maps an integer index in the range 0...numIndices-1 into a normalized floating point number in
the range 0...1. */
INLINE float indexToNormalizedValue(int index, int numIndices);

/** Checks, if x is close to some target-value within some tolerance. */
INLINE bool isCloseTo(double x, double targetValue, double tolerance);

/** Checks, if x is even. */
INLINE bool isEven(int x);

/** Checks, if x is odd. */
INLINE bool isOdd(int x);

/** Checks, if x is a power of 2. */
INLINE bool isPowerOfTwo(unsigned int x);

/** Calculates the logarithm to base 2. */
#if defined _MSC_VER && _MSC_VER < 1930 // after vs2022 we can get it via native intrinsics support
INLINE double log2(double x);
#endif

/** Calculates logarithm to an arbitrary base b. */
INLINE double logB(double x, double b);

/** Converts a value between inMin and inMax into a value between outMin and outMax where the
mapping is linear for the input and the output. Example: y = linToLin(x, 0.0, 1.0, -96.0, 24.0)
will map the input x assumed to lie inside 0.0...1.0 to the range between -96.0...24.0. This
function is useful to convert between parameter representations between 0.0...1.0 and the
clear-text parameters. */
INLINE double linToLin(double in, double inMin, double inMax, double outMin, double outMax);

/** Converts a value between inMin and inMax into a value between outMin and outMax where the
mapping of the output is exponential. Example: y = linToExp(x, 0.0, 1.0, 20.0, 20000.0) will map
the input x assumed to lie inside 0.0...1.0 to the range between 20.0...20000.0 where equal
differences in the input lead to equal factors in the output. Make sure that the outMin value is
greater than zero! */
INLINE double linToExp(double in, double inMin, double inMax, double outMin, double outMax);

/** Same as linToExp but adds an offset afterwards and compensates for that offset by scaling the
offsetted value so as to hit the outMax correctly. */
INLINE double linToExpWithOffset(double in, double inMin, double inMax, double outMin,
                                 double outMax, double offset = 0.0);

/** Same as linToExp but applies a power curve to the input first, giving more resolution at low
output values. Higher power = more low-end resolution. Useful for decay time knobs where you want
fine control over short times (e.g., 30-100ms) while still reaching long times (e.g., 3000ms). */
INLINE double linPowToExp(double in, double power, double outMin, double outMax);

/** The Inverse of "linToExp" */
INLINE double expToLin(double in, double inMin, double inMax, double outMin, double outMax);

/** The Inverse of "linToExpWithOffset" */
INLINE double expToLinWithOffset(double in, double inMin, double inMax, double outMin,
                                 double outMax, double offset = 0.0);

/** Returns a power of two which is greater than or equal to the input argument. */
template <class T>
INLINE T nextPowerOfTwo(T x);

/** Maps a normalized floating point number in the range 0...1 into an integer index in the range
0...numIndices-1. */
INLINE int normalizedValueToIndex(float normalizedValue, int numIndices);

/** Converts a picth-offset in semitones value into a frequency multiplication factor. */
INLINE double pitchOffsetToFreqFactor(double pitchOffset);

/** Converts a MIDI-note value into a frequency in Hz assuming A4 = 440 Hz. */
INLINE double pitchToFreq(double pitch);

/** Converts a MIDI-note value into a frequency in Hz for arbitrary master-tunings of A4. */
INLINE double pitchToFreq(double pitch, double masterTuneA4);

/** Converts an angle in radiant into degrees. */
INLINE double radiantToDegree(double radiant);

/** Generates a random number that is uniformly distributed between min and max (inclusive). The
underlying integer pseudo random number generator is a linear congruential with period length of
2^32. It is based on Numerical Recipies in C (2nd edition), page 284. You may pass a seed to the
first call to initialize it - otherwise it will use 0 as seed. A negative number (as in the default
argument) will indicate to not initialize the state and just generate a random number based on the
last state (which is the case for a typical call). */
INLINE double randomUniform(double min = 0.0, double max = 1.0, int seed = -1);

/** Returns the nearest integer (as double, without typecast). */
INLINE double round(double x);

/** Converts a time value in seconds into a time value measured in beats. */
INLINE double secondsToBeats(double timeInSeconds, double bpm);

/** Returns the sign of x as double. */
INLINE double sign(double x);

/** Converts a time-stamp given in whole notes into seconds according to a tempo measured in
beats per minute (bpm). */
INLINE double wholeNotesToSeconds(double noteValue, double bpm);

/** Linear interpolation between two values. Returns a when t=0, b when t=1. */
INLINE double lerp(double a, double b, double t);

/** Fixed-point quantization to specified bit depth. Emulates DSP fixed-point accumulators. */
template<int Bits>
INLINE double quantize(double x)
{
  constexpr double scale = static_cast<double>(1ULL << Bits);
  return floor(x * scale) * (1.0 / scale);
}

/** Fixed-point quantization with rounding (more accurate for audio). */
template<int Bits>
INLINE double quantizeRound(double x)
{
  constexpr double scale = static_cast<double>(1ULL << Bits);
  return round(x * scale) * (1.0 / scale);
}

//=================================================================================================
//implementation:

INLINE double amp2dB(double amp)
{
  return 8.6858896380650365530225783783321 * log(amp);
  //return 20*log10(amp); // naive version
}

INLINE double amp2dBWithCheck(double amp, double lowAmplitude)
{
  if( amp >= lowAmplitude )
    return 8.6858896380650365530225783783321 * log(amp);
  else
    return 8.6858896380650365530225783783321 * log(lowAmplitude);
}

template <class T>
INLINE int arrayMaxIndex(T* theArray, int numValues)
{
  int    maxIndex = 0;
  double maxValue = theArray[0];
  for(int i=0; i<numValues; i++)
  {
    if( theArray[i] > maxValue )
    {
      maxValue = theArray[i];
      maxIndex = i;
    }
  }
  return maxIndex;
}

template <class T>
INLINE int arrayMinIndex(T* theArray, int numValues)
{
  int    minIndex = 0;
  double minValue = theArray[0];
  for(int i=0; i<numValues; i++)
  {
    if( theArray[i] < minValue )
    {
      minValue = theArray[i];
      minIndex = i;
    }
  }
  return minIndex;
}

INLINE double beatsToSeconds(double beat, double bpm)
{
  return (60.0/bpm)*beat;
}

INLINE double dB2amp(double dB)
{
  return exp(dB * 0.11512925464970228420089957273422);
  //return pow(10.0, (0.05*dB)); // naive, inefficient version
}

INLINE double degreeToRadiant(double degrees)
{
  return (PI/180.0)*degrees;
}

/*
INLINE void deleteAndNullifyPointer(void *pointer)
{
  delete pointer;
  pointer = NULL;
}
*/

INLINE double euclideanDistance(double x1, double y1, double x2, double y2)
{
  return sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );
}

INLINE double exp10(double x)
{
  return exp(LN10*x);
}

INLINE double exp2(double x)
{
  return exp(LN2*x);
}

INLINE double freqToPitch(double freq)
{
  return 12.0 * log2(freq/440.0) + 69.0;
}

INLINE double freqToPitch(double freq, double masterTuneA4)
{
  return 12.0 * log2(freq/masterTuneA4) + 69.0;
}

/*
INLINE void ifNotNullDeleteAndSetNull(void* pointer)
{
  if( pointer != NULL )
  {
    delete pointer;
    pointer = NULL;
  }
}
*/

INLINE float indexToNormalizedValue(int index, int numIndices)
{
  return (float) (2*index+1) / (float) (2*numIndices);
}

INLINE bool isCloseTo(double x, double targetValue, double tolerance)
{
  if( fabs(x-targetValue) <= tolerance )
    return true;
  else
    return false;
}

INLINE bool isEven(int x)
{
  if( x%2 == 0 )
    return true;
  else
    return false;
}

INLINE bool isOdd(int x)
{
  if( x%2 != 0 )
    return true;
  else
    return false;
}

INLINE bool isPowerOfTwo(unsigned int x)
{
  unsigned int currentPower = 1;
  while( currentPower <= x )
  {
    if( currentPower == x )
      return true;
    currentPower *= 2;
  }
  return false;
}

#if defined _MSC_VER && _MSC_VER < 1930 // after vs2022 we can get it via native intrinsics support
INLINE double log2(double x)
{
  return ONE_OVER_LN2*log(x);
}
#endif

INLINE double logB(double x, double b)
{
  return log(x)/log(b);
}

INLINE double linToLin(double in, double inMin, double inMax, double outMin, double outMax)
{
  // map input to the range 0.0...1.0:
  double tmp = (in-inMin) / (inMax-inMin);

  // map the tmp-value to the range outMin...outMax:
  tmp *= (outMax-outMin);
  tmp += outMin;

  return tmp;
}

INLINE double linToExp(double in, double inMin, double inMax, double outMin, double outMax)
{
  // map input to the range 0.0...1.0:
  double tmp = (in-inMin) / (inMax-inMin);

  // map the tmp-value exponentially to the range outMin...outMax:
  //tmp = outMin * exp( tmp*(log(outMax)-log(outMin)) );
  return outMin * exp( tmp*(log(outMax/outMin)) );
}

INLINE double linToExpWithOffset(double in, double inMin, double inMax, double outMin,
                                 double outMax, double offset)
{
  double tmp = linToExp(in, inMin, inMax, outMin, outMax);
  tmp += offset;
  tmp *= outMax/(outMax+offset);
  return tmp;
}

INLINE double linPowToExp(double in, double power, double outMin, double outMax)
{
  double warped = pow(in, power);
  return outMin * exp(warped * log(outMax / outMin));
}

INLINE double expToLin(double in, double inMin, double inMax, double outMin, double outMax)
{
  double tmp = log(in/inMin) / log(inMax/inMin);
  return outMin + tmp * (outMax-outMin);
}

INLINE double expToLinWithOffset(double in, double inMin, double inMax, double outMin,
                                 double outMax, double offset)
{
  double tmp = in*(inMax+offset)/inMax;
  tmp -= offset;
  return expToLin(tmp, inMin, inMax, outMin, outMax);
  /*
  double tmp = linToExp(in, inMin, inMax, outMin, outMax);
  tmp += offset;
  tmp *= outMax/(outMax+offset);
  return tmp;
  */
}

template <class T>
INLINE T nextPowerOfTwo(T x)
{
  T accu = 1;
  while(accu < x)
    accu *= 2;
  return accu;
}

INLINE int normalizedValueToIndex(float normalizedValue, int numIndices)
{
  return (int) floor(normalizedValue*numIndices);
}

INLINE double pitchOffsetToFreqFactor(double pitchOffset)
{
  return exp(0.057762265046662109118102676788181 * pitchOffset);
  //return pow(2.0, pitchOffset/12.0); // naive, slower but numerically more precise
}

INLINE double pitchToFreq(double pitch)
{
  return 8.1757989156437073336828122976033 * exp(0.057762265046662109118102676788181*pitch);
  //return 440.0*( pow(2.0, (pitch-69.0)/12.0) ); // naive, slower but numerically more precise
}

INLINE double pitchToFreq(double pitch, double masterTuneA4)
{
  return masterTuneA4 * 0.018581361171917516667460937040007
    * exp(0.057762265046662109118102676788181*pitch);
}

INLINE double radiantToDegree(double radiant)
{
  return (180.0/PI)*radiant;
}

INLINE double randomUniform(double min, double max, int seed)
{
  unsigned long state = 0;
  if( seed >= 0 )
    state = seed;                                        // initialization, if desired
  state = 1664525*state + 1013904223;                    // mod implicitely by integer overflow
  return min + (max-min) * ((1.0/4294967296.0) * state); // transform to desired range
}

INLINE double round(double x)
{
  if( x-floor(x) >= 0.5 )
    return ceil(x);
  else
    return floor(x);
}

INLINE double secondsToBeats(double timeInSeconds, double bpm)
{
  return timeInSeconds*(bpm/60.0);
}

INLINE double sign(double x)
{
  if(x<0)
    return -1.0;
  else if(x>0)
    return 1.0;
  else
    return 0;
}

INLINE double wholeNotesToSeconds(double noteValue, double bpm)
{
  return (240.0/bpm)*noteValue;
}

INLINE double lerp(double a, double b, double t)
{
  return a + t * (b - a);
}

#endif // #ifndef GlobalFunctions_h
