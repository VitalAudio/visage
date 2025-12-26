#include "dfl_StateVariableFilter.h"
using namespace dfl;

//-------------------------------------------------------------------------------------------------
// construction/destruction:

StateVariableFilter::StateVariableFilter()
{
  cutoff              =  1000.0;
  driveFactor         =     1.0;
  drive               =     0.0;
  passbandCompensation =    0.0;
  resonance           =     0.0;
  sampleRate          = 44100.0;
  lpBpMix             =     0.0;  // pure LP (0=LP, 0.5=BP, 1=HP)
  notchOffset         =     0.0;  // true notch (balanced LP+HP)
  fourPoleMode        =    true;  // default to 4-pole (24dB/oct)
  notchMode           =   false;  // default to LP/BP/HP morph mode

  // EQ parameters - default to unity gain
  svfType             = SVFType::MultiMode;
  gain                =     1.0;
  gainDb              =     0.0;
  A                   =     1.0;  // sqrt(1.0)
  sqrtA               =     1.0;  // sqrt(sqrt(1.0))

  // Initialize coefficients
  g = 0.0;
  k = 2.0;
  a1 = a2 = a3 = 0.0;

  calculateCoefficients();
  reset();
}

StateVariableFilter::~StateVariableFilter()
{
}

//-------------------------------------------------------------------------------------------------
// parameter settings:

void StateVariableFilter::setSampleRate(double newSampleRate)
{
  if( newSampleRate > 0.0 )
    sampleRate = newSampleRate;
  calculateCoefficients();
}

void StateVariableFilter::setInputDrive(double newDrive)
{
  drive = newDrive;
  driveFactor = dB2amp(newDrive);
}

void StateVariableFilter::setGainDb(double newGainDb)
{
  gainDb = newGainDb;
  gain = dB2amp(newGainDb);
  A = sqrt(gain);
  sqrtA = sqrt(A);
  calculateCoefficients();
}

void StateVariableFilter::setGainLinear(double newGain)
{
  if (newGain <= 0.0)
    newGain = 0.001;  // Prevent log(0)
  gain = newGain;
  gainDb = amp2dB(newGain);
  A = sqrt(gain);
  sqrtA = sqrt(A);
  calculateCoefficients();
}

//-------------------------------------------------------------------------------------------------
// others:

void StateVariableFilter::reset()
{
  // First section
  z1_1 = 0.0;
  z2_1 = 0.0;
  // Second section (4-pole)
  z1_2 = 0.0;
  z2_2 = 0.0;
}
