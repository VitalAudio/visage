#ifndef dfl_StateVariableFilter_h
#define dfl_StateVariableFilter_h

// TPT State Variable Filter (Cytomic/Zavalishin style)
// Supports 2-pole (12dB/oct) and 4-pole (24dB/oct) modes
//
// The SVF topology provides simultaneous LP, BP, HP outputs from the
// same structure. Unlike ladder filters, SVF has independent resonance
// and cutoff control with no interaction between them.
//
// Filter Modes:
// -------------
// - Lowpass, Bandpass, Highpass (with smooth morphing)
// - Notch with offset (ARP 1047 style)
// - Low Shelf, High Shelf, Bell EQ (with gain control)
//
// Nonlinearity Model:
// -------------------
// SVF can be run clean or with saturation. The nonlinearity is applied
// at the input stage and optionally between cascaded sections (4-pole mode).
// This creates a different character than ladder filters - more "clinical"
// when clean, with smooth saturation when driven.
//
// Signal Flow (2-pole):
// ---------------------
//  Input ──> [saturation] ──> v0 ──┬──> HP output
//                                  │
//                           ┌──────┴──────┐
//                           │   g/(1+g)   │ TPT integrator 1
//                           └──────┬──────┘
//                                  │
//                                  v1 ──────> BP output
//                                  │
//                           ┌──────┴──────┐
//                           │   g/(1+g)   │ TPT integrator 2
//                           └──────┬──────┘
//                                  │
//                                  v2 ──────> LP output
//                                  │
//                           ┌──────┴──────┐
//                           │  Resonance  │ Feedback to v0
//                           └─────────────┘
//
// 4-pole mode cascades two 2-pole sections with optional inter-stage saturation.

#include <stdlib.h>
#include <cmath>
#include "dfl_FastMath.h"

#include "rosic_RealFunctions.h"
#include "dfl_FilterBase.h"


namespace dfl
{

  /** SVF filter type for EQ modes */
  enum class SVFType
  {
    MultiMode,    // LP/BP/HP morph or Notch (default synth mode)
    LowShelf,     // Low shelf EQ
    HighShelf,    // High shelf EQ
    Bell          // Parametric bell/peak EQ
  };

  /**
   * TPT State Variable Filter with 2-pole and 4-pole modes.
   *
   * Character:
   * - Clean and precise when not driven
   * - Smooth, symmetrical saturation with drive
   * - Independent cutoff and resonance (no interaction)
   * - True multimode: LP, BP, HP available simultaneously
   */

  class StateVariableFilter : public FilterBase
  {

  public:

    //---------------------------------------------------------------------------------------------
    // construction/destruction:

    /** Constructor. */
    StateVariableFilter();

    /** Destructor. */
    ~StateVariableFilter() override;

    //---------------------------------------------------------------------------------------------
    // parameter settings (FilterBase interface):

    void setSampleRate(double newSampleRate) override;
    INLINE void setCutoff(double newCutoff, bool updateCoefficients = true) override;
    INLINE void setResonance(double newResonance, bool updateCoefficients = true) override;
    void setInputDrive(double newDrive) override;
    void setPassbandCompensation(double newCompensation) override { passbandCompensation = newCompensation; }
    void setLpBpMix(double newMix) override { lpBpMix = newMix; }

    //---------------------------------------------------------------------------------------------
    // inquiry (FilterBase interface):

    double getCutoff() const override { return cutoff; }
    double getResonance() const override { return resonance; }
    double getDrive() const override { return drive; }
    double getPassbandCompensation() const override { return passbandCompensation; }
    double getLpBpMix() const override { return lpBpMix; }

    //---------------------------------------------------------------------------------------------
    // audio processing (FilterBase interface):

    INLINE double getSample(double in) override;

    //---------------------------------------------------------------------------------------------
    // others (FilterBase interface):

    void reset() override;

    //---------------------------------------------------------------------------------------------
    // SVF-specific methods:

    /** Sets 4-pole mode (true = 24dB/oct, false = 12dB/oct). */
    void setFourPoleMode(bool enabled) { fourPoleMode = enabled; }

    /** Returns whether 4-pole mode is enabled. */
    bool isFourPoleMode() const { return fourPoleMode; }

    /** Sets notch mode (true = LP+HP notch blend, false = LP/BP/HP morph). */
    void setNotchMode(bool enabled) { notchMode = enabled; }

    /** Returns whether notch mode is enabled. */
    bool isNotchMode() const { return notchMode; }

    /** Sets the notch offset for notch mode.
        -1 = HP-heavy notch, 0 = true notch (equal LP+HP), +1 = LP-heavy notch.
        This is the "funky" ARP 1047 feature - shifts the notch character. */
    void setNotchOffset(double offset) { notchOffset = (offset < -1.0) ? -1.0 : (offset > 1.0) ? 1.0 : offset; }

    /** Returns the notch offset. */
    double getNotchOffset() const { return notchOffset; }

    /** Causes the filter to re-calculate the coefficients. */
    INLINE void calculateCoefficients();

    /** SVF saturation - applied at input and between stages. */
    INLINE double svfShape(double x);

    //---------------------------------------------------------------------------------------------
    // EQ methods:

    /** Sets the filter type (MultiMode, LowShelf, HighShelf, Bell). */
    void setSVFType(SVFType newType) { svfType = newType; calculateCoefficients(); }

    /** Returns the current filter type. */
    SVFType getSVFType() const { return svfType; }

    /** Sets the EQ gain in dB. Only affects LowShelf, HighShelf, and Bell modes.
        Positive values boost, negative values cut. Range: -24 to +24 dB typical. */
    void setGainDb(double newGainDb);

    /** Returns the EQ gain in dB. */
    double getGainDb() const { return gainDb; }

    /** Sets the EQ gain in linear units. */
    void setGainLinear(double newGain);

    /** Returns the EQ gain in linear units. */
    double getGainLinear() const { return gain; }

    //=============================================================================================

  protected:

    // State variables for 2-pole SVF (first section)
    double z1_1, z2_1;

    // State variables for second 2-pole section (4-pole mode)
    double z1_2, z2_2;

    // Coefficients
    double g;      // TPT integrator gain = tan(pi * fc / fs)
    double k;      // Damping factor = 2 - 2*Q = 2*(1-resonance) for Q in [0.5, inf]
    double a1;     // Coefficient for implicit solution: 1 / (1 + g*(g + k))
    double a2;     // g * a1
    double a3;     // g * a2

    // Filter parameters
    double cutoff;
    double drive;
    double driveFactor;
    double passbandCompensation;
    double resonance;
    double sampleRate;
    double lpBpMix;         // 0.0 = LP, 0.5 = BP, 1.0 = HP (extended range)
    double notchOffset;     // Notch mode: -1 = HP-heavy, 0 = true notch, +1 = LP-heavy
    bool   fourPoleMode;    // true = 24dB/oct (two cascaded sections)
    bool   notchMode;       // true = notch mode (LP+HP blend), false = normal LP/BP/HP morph

    // EQ parameters
    SVFType svfType;        // Filter type (MultiMode, LowShelf, HighShelf, Bell)
    double gain;            // EQ gain in linear units
    double gainDb;          // EQ gain in dB (for reference)
    double A;               // sqrt(gain) - used in shelf/bell calculations
    double sqrtA;           // sqrt(A) = gain^0.25 - used in shelf calculations

    // Saturation parameters
    static constexpr double svfSatSharp = 1.0;  // Saturation sharpness
    static constexpr double bpCompensation = 1.4142135623730951;  // sqrt(2) for BP compensation
  };

  //-----------------------------------------------------------------------------------------------
  // inlined functions:

  INLINE void StateVariableFilter::setCutoff(double newCutoff, bool updateCoefficients)
  {
    if( newCutoff != cutoff )
    {
      if( newCutoff < 200.0 )
        cutoff = 200.0;
      else if( newCutoff > 20000.0 )
        cutoff = 20000.0;
      else
        cutoff = newCutoff;

      if( updateCoefficients == true )
        calculateCoefficients();
    }
  }

  INLINE void StateVariableFilter::setResonance(double newResonance, bool updateCoefficients)
  {
    resonance = newResonance;

    if( updateCoefficients == true )
      calculateCoefficients();
  }

  INLINE void StateVariableFilter::calculateCoefficients()
  {
    // TPT integrator coefficient (Zavalishin's "g")
    // Using tan() for frequency warping - accurate up to Nyquist
    double g0 = tan(PI * cutoff / sampleRate);

    // Damping factor k = 2 - 2*Q  (or 1/Q for EQ modes)
    // When resonance = 0, k = 2 (critically damped)
    // When resonance = 1, k = 0 (self-oscillation)
    // For 4-pole mode, we use slightly less resonance per stage
    double effectiveRes = fourPoleMode ? resonance * 0.8 : resonance;

    // For EQ modes, resonance maps to Q differently
    // Q = 0.5 to 10 typical, resonance 0-1 maps to this range
    double Q = 0.5 + effectiveRes * 9.5;  // Q from 0.5 to 10
    double k0 = 1.0 / Q;

    // Adjust g and k based on filter type (Zavalishin/Cytomic approach)
    switch (svfType)
    {
      case SVFType::Bell:
        // Bell: g unchanged, k divided by A for boost/cut symmetry
        g = g0;
        k = k0 / A;
        break;

      case SVFType::LowShelf:
        // Low shelf: g divided by sqrt(A) for proper shelf shape
        g = g0 / sqrtA;
        k = k0;
        break;

      case SVFType::HighShelf:
        // High shelf: g multiplied by sqrt(A)
        g = g0 * sqrtA;
        k = k0;
        break;

      case SVFType::MultiMode:
      default:
        // Standard synth filter mode - use original resonance mapping
        g = g0;
        k = 2.0 * (1.0 - effectiveRes);
        break;
    }

    // Pre-computed coefficients for the implicit solution
    a1 = 1.0 / (1.0 + g * (g + k));
    a2 = g * a1;
    a3 = g * a2;
  }

  INLINE double StateVariableFilter::svfShape(double x)
  {
    // SVF saturation - symmetric tanh
    // Amount controlled by drive: at 0dB nearly linear, at 12dB fully saturated
    double driveNorm = drive / 12.0;
    if (driveNorm < 0.1) return x;

    double sat = dfl::fastTanh(svfSatSharp * x);
    return x + driveNorm * (sat - x);
  }

  INLINE double StateVariableFilter::getSample(double in)
  {
    // Apply input drive (only for synth modes, not EQ)
    double x = (svfType == SVFType::MultiMode) ? driveFactor * in : in;

    // Apply input saturation (only for synth modes)
    if (svfType == SVFType::MultiMode)
      x = svfShape(x);

    // ========== First 2-pole SVF section ==========
    // Cytomic SVF topology with implicit integration
    // hp = (x - k*bp - lp) / (1 + g*(g+k))
    // bp = g*hp + z1
    // lp = g*bp + z2

    double hp1 = (x - k * z1_1 - z2_1) * a1;
    double bp1 = a2 * hp1 + z1_1;
    double lp1 = a3 * hp1 + a2 * z1_1 + z2_1;

    // Update state (trapezoidal integration)
    z1_1 = 2.0 * bp1 - z1_1;
    z2_1 = 2.0 * lp1 - z2_1;

    double lp, bp, hp;

    if (fourPoleMode && svfType == SVFType::MultiMode)
    {
      // ========== Second 2-pole SVF section (cascade) ==========
      // Only used for synth mode - EQ modes are always 2-pole
      // Feed LP output of first section into second section
      // Apply inter-stage saturation for character
      double x2 = svfShape(lp1);

      double hp2 = (x2 - k * z1_2 - z2_2) * a1;
      double bp2 = a2 * hp2 + z1_2;
      double lp2 = a3 * hp2 + a2 * z1_2 + z2_2;

      // Update state
      z1_2 = 2.0 * bp2 - z1_2;
      z2_2 = 2.0 * lp2 - z2_2;

      // Final outputs (4-pole)
      lp = lp2;
      bp = bp2;
      // 4-pole HP: cascade HP outputs (hp1 feeds into second section as HP)
      // For proper 4-pole HP, we'd need to cascade HP->HP, but this approximation
      // uses the relationship: HP = input - LP - k*BP (from the SVF equations)
      hp = x - lp - k * bp;
    }
    else
    {
      // 2-pole outputs directly
      lp = lp1;
      bp = bp1;
      hp = hp1;
    }

    double out;

    // ========== EQ Modes ==========
    if (svfType == SVFType::LowShelf)
    {
      // Low shelf: LP scaled by A^2, HP unchanged, BP scaled by A
      // y = A^2 * lp + A * k * bp + hp
      out = A * A * lp + A * k * bp + hp;
    }
    else if (svfType == SVFType::HighShelf)
    {
      // High shelf: HP scaled by A^2, LP unchanged, BP scaled by A
      // y = lp + A * k * bp + A^2 * hp
      out = lp + A * k * bp + A * A * hp;
    }
    else if (svfType == SVFType::Bell)
    {
      // Bell/peak: BP scaled by A*k gives boost/cut at center frequency
      // y = lp + A * k * bp + hp
      out = lp + A * k * bp + hp;
    }
    // ========== Synth Modes ==========
    else if (notchMode)
    {
      // ========== Notch mode with offset (ARP 1047 style) ==========
      // notchOffset: -1 = HP-heavy, 0 = true notch, +1 = LP-heavy
      // This creates the "funky" moveable notch character
      double lpMix = notchOffset * 0.5 + 0.5;  // Map [-1,+1] to [0,1]
      double hpMix = 1.0 - lpMix;

      // Apply makeup gain - notch has a dip, so compensate
      // Maximum attenuation at true notch (offset=0), less at extremes
      double makeup = 2.0 - fabs(notchOffset);

      out = makeup * (lpMix * lp + hpMix * hp);
    }
    else
    {
      // ========== Normal LP/BP/HP morph mode ==========
      // lpBpMix: 0 = LP, 0.5 = BP, 1.0 = HP
      // Uses three-way equal-power crossfade with BP compensation

      double lpW, bpW, hpW;

      if (lpBpMix <= 0.5)
      {
        // LP to BP transition (0 to 0.5)
        double t = lpBpMix * 2.0;  // Map [0, 0.5] to [0, 1]
        double angle = t * PI * 0.5;
        lpW = sin(PI * 0.5 - angle);  // cos via sin for consistency
        bpW = sin(angle);
        hpW = 0.0;
      }
      else
      {
        // BP to HP transition (0.5 to 1.0)
        double t = (lpBpMix - 0.5) * 2.0;  // Map [0.5, 1] to [0, 1]
        double angle = t * PI * 0.5;
        lpW = 0.0;
        bpW = sin(PI * 0.5 - angle);  // cos via sin for consistency
        hpW = sin(angle);
      }

      // BP compensation: BP is naturally quieter, boost by sqrt(2) for smooth transition
      bpW *= bpCompensation;

      out = lpW * lp + bpW * bp + hpW * hp;
    }

    // Apply passband gain compensation (synth mode only)
    if (svfType == SVFType::MultiMode)
    {
      // SVF resonance boost is roughly (1 + k), so compensate inversely
      double resBoost = fourPoleMode ? (2.0 - k) * (2.0 - k) : (2.0 - k);
      out *= (1.0 + passbandCompensation * (resBoost - 1.0));
    }

    return out;
  }

} // end namespace dfl

#endif // dfl_StateVariableFilter_h
