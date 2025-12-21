#ifndef dfl_FilterBase_h
#define dfl_FilterBase_h

// Virtual base class for all filter types
// Allows polymorphic filter selection without switch statements

namespace dfl
{

  /** Morph style for multimode filters (SVF-topology filters) */
  enum class MorphStyle
  {
    Bandpass,   // LP -> BP -> HP morph (standard SVF)
    Notch       // LP -> Notch -> HP morph (SEM style)
  };

  /**
   * Abstract base class defining the common interface for all filters.
   *
   * All filters (Diode, Roland, Moog, SVF, Korg, Steiner-Parker, etc.)
   * inherit from this class, enabling runtime filter switching via pointer.
   */

  class FilterBase
  {
  public:
    virtual ~FilterBase() = default;

    //---------------------------------------------------------------------------------------------
    // parameter settings:

    /** Sets the sample-rate for this filter. */
    virtual void setSampleRate(double newSampleRate) = 0;

    /** Sets the cutoff frequency for this filter. */
    virtual void setCutoff(double newCutoff, bool updateCoefficients = true) = 0;

    /** Sets the resonance (0-1 range, pre-skewed). */
    virtual void setResonance(double newResonance, bool updateCoefficients = true) = 0;

    /** Sets the input drive in decibels. */
    virtual void setInputDrive(double newDrive) = 0;

    /** Sets the passband gain compensation amount (0.0 = none, 1.0 = full). */
    virtual void setPassbandCompensation(double newCompensation) = 0;

    /** Sets the LP/BP mix (0.0 = pure LP, 1.0 = pure BP/Notch, depends on morph style). */
    virtual void setLpBpMix(double newMix) = 0;

    /** Sets the morph style for multimode filters (Bandpass or Notch).
        Default implementation does nothing - override in SVF-type filters. */
    virtual void setMorphStyle(MorphStyle /*style*/) {}

    //---------------------------------------------------------------------------------------------
    // inquiry:

    /** Returns the cutoff frequency of this filter. */
    virtual double getCutoff() const = 0;

    /** Returns the resonance parameter of this filter. */
    virtual double getResonance() const = 0;

    /** Returns the drive parameter in decibels. */
    virtual double getDrive() const = 0;

    /** Returns the passband gain compensation amount. */
    virtual double getPassbandCompensation() const = 0;

    /** Returns the current LP/BP mix. */
    virtual double getLpBpMix() const = 0;

    /** Returns the current morph style. Default returns Bandpass. */
    virtual MorphStyle getMorphStyle() const { return MorphStyle::Bandpass; }

    //---------------------------------------------------------------------------------------------
    // audio processing:

    /** Calculates one output sample at a time. */
    virtual double getSample(double in) = 0;

    //---------------------------------------------------------------------------------------------
    // others:

    /** Resets the internal state variables. */
    virtual void reset() = 0;
  };

} // end namespace dfl

#endif // dfl_FilterBase_h
