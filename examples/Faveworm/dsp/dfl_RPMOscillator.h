#ifndef dfl_RPMOscillator_h
#define dfl_RPMOscillator_h

#include "dfl_FastMath.h"
#include "GlobalDefinitions.h"

#include <cmath>

#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif

namespace dfl {

  /**
   * Recursive Phase Modulation Oscillator
   *
   * Based on Yamaha's recursive phase modulation technique where the oscillator's
   * output is fed back into its own phase input through a one-pole "bunting" filter.
   * This creates rich, evolving harmonic content controlled by the beta (feedback amount)
   * and exponent (nonlinearity) parameters.
   *
   * Primary interface: process(phaseIn) - takes a 0-1 phasor input (like rpmb~ in PureData).
   * Convenience: getSample() - uses internal phase accumulator if no external phasor available.
   *
   * Algorithm:
   *   state = 0.5 * (state + pow(lastOut, exponent))  // one-pole averaging filter
   *   output = sin(2π * phaseIn + beta * state)       // phase modulation with feedback
   *
   * Waveform characteristics:
   *   - Low beta: triangle-like (soft harmonics)
   *   - Positive beta + exponent 1: saw-like
   *   - Negative beta + exponent 2: square-like (cleaner edges)
   *   - High |beta|: noise/chaos (use softClip to tame)
   *
   * Use setSawMode() and setSquareMode() for convenient presets.
   */

  class RPMOscillator {
  public:
    //---------------------------------------------------------------------------------------------
    // construction/destruction:

    RPMOscillator() {
      sampleRate = 44100.0;
      sampleRateRec = 1.0 / 44100.0;
      freq = 440.0;
      phase = 0.0;
      increment = 0.0;
      beta = 1.0;  // feedback/modulation amount
      exponent = 1;  // power applied to feedback
      state = 0.0;  // filtered feedback state
      lastOut = 0.0;  // previous output sample
      softClip = false;  // optional tanh limiting on feedback
    }

    ~RPMOscillator() { }

    //---------------------------------------------------------------------------------------------
    // parameter settings:

    /** Sets the sample rate. */
    void setSampleRate(double newSampleRate) {
      if (newSampleRate > 0.0) {
        sampleRate = newSampleRate;
        sampleRateRec = 1.0 / newSampleRate;
        calculateIncrement();
      }
    }

    /** Sets the oscillator frequency in Hz. */
    INLINE void setFrequency(double newFrequency) {
      if ((newFrequency > 0.0) && (newFrequency < 20000.0)) {
        freq = newFrequency;
        calculateIncrement();
      }
    }

    /**
     * Sets the beta parameter (feedback/modulation amount).
     * Higher values create more harmonic complexity and potential instability.
     * Typical range: 0.0 to 4.0, but can go higher for extreme effects.
     */
    void setBeta(double newBeta) { beta = newBeta; }

    /** Returns the current beta value. */
    double getBeta() const { return beta; }

    /**
     * Sets the exponent parameter.
     * Controls the nonlinearity of the feedback path.
     * - 1: linear feedback (pure FM-like, sine to complex)
     * - 2: squared feedback - produces square wave at higher beta values
     * - 3+: increasingly harsh/complex spectra
     */
    void setExponent(int newExponent) {
      if (newExponent >= 1)
        exponent = newExponent;
    }

    /** Returns the current exponent value. */
    int getExponent() const { return exponent; }

    /**
     * Enables/disables soft clipping on the feedback state.
     * When enabled, applies tanh to prevent runaway feedback at high beta values.
     * When disabled, allows chaotic/noise behavior at extreme settings.
     */
    void setSoftClip(bool enabled) { softClip = enabled; }

    /** Returns whether soft clipping is enabled. */
    bool getSoftClip() const { return softClip; }

    //---------------------------------------------------------------------------------------------
    // presets:

    /**
     * Configures for saw-like waveform.
     * Uses positive beta with linear feedback (exponent=1).
     * @param amount Controls harmonic richness (typical range 0.5-2.0)
     */
    void setSawMode(double amount = 1.0) {
      beta = std::abs(amount);
      exponent = 1;
    }

    /**
     * Configures for square-like waveform.
     * Uses negative beta with squared feedback (exponent=2).
     * Negative beta produces cleaner square wave edges.
     * @param amount Controls harmonic richness (typical range 0.5-2.0)
     */
    void setSquareMode(double amount = 1.0) {
      beta = -std::abs(amount);
      exponent = 2;
    }

    //---------------------------------------------------------------------------------------------
    // audio processing:

    /**
     * Process one sample with external phasor input (primary interface).
     * This matches the rpmb~ PureData object design where a phasor~ drives the oscillator.
     *
     * @param phaseIn Normalized phase input from phasor (0.0 to 1.0)
     * @return Output sample (-1.0 to 1.0)
     */
    INLINE double process(double phaseIn) {
      // Update feedback state with one-pole "bunting" filter
      state = 0.5 * (state + fastPow(lastOut, exponent));

      // Optional soft clipping to tame runaway feedback at high beta
      if (softClip)
        state = dfl::fastTanh(state);

      // Generate output: sine of (input phase + feedback modulation)
      // Using LUT: convert modulated radians back to normalized phase
      double modulatedPhase = phaseIn + (beta * state) / TWO_PI;
      lastOut = dfl::fastSin(modulatedPhase);

      // Apply soft clipping to output to tame extreme values
      if (softClip)
        lastOut = dfl::fastTanh(lastOut);

      return lastOut;
    }

    /** Calculates the phase increment based on frequency and sample rate. */
    INLINE void calculateIncrement() { increment = freq * sampleRateRec; }

    /**
     * Generates one output sample using internal phase accumulator.
     * Convenience method when you don't have an external phasor.
     */
    INLINE double getSample() {
      double out = process(phase);

      // Advance internal phase with wraparound
      phase += increment;
      while (phase >= 1.0)
        phase -= 1.0;

      return out;
    }

    /** Resets the oscillator state (phase, feedback state, and last output). */
    void reset() {
      phase = 0.0;
      state = 0.0;
      lastOut = 0.0;
    }

    /** Resets only the phase accumulator to zero. */
    void resetPhase() { phase = 0.0; }

    /** Sets the phase directly (0.0 to 1.0). */
    void setPhase(double newPhase) {
      phase = newPhase;
      while (phase >= 1.0)
        phase -= 1.0;
      while (phase < 0.0)
        phase += 1.0;
    }

    /** Advances (or retards) phase by a number of samples. */
    void advancePhase(double numSamples) {
      phase += increment * numSamples;
      while (phase >= 1.0)
        phase -= 1.0;
      while (phase < 0.0)
        phase += 1.0;
    }

    //=============================================================================================

  protected:
    // Fast power function for integer exponents
    INLINE double fastPow(double base, int exp) {
      if (exp == 1)
        return base;
      if (exp == 2)
        return base * base;
      if (exp == 3)
        return base * base * base;
      if (exp == 4) {
        double sq = base * base;
        return sq * sq;
      }
      // Fallback for higher exponents
      return std::pow(base, exp);
    }

    double sampleRate;
    double sampleRateRec;
    double freq;
    double phase;  // normalized phase (0.0 to 1.0)
    double increment;  // phase increment per sample
    double beta;  // feedback/modulation amount
    int exponent;  // power applied to feedback signal
    double state;  // one-pole filtered feedback state
    double lastOut;  // previous output sample
    bool softClip;  // enable tanh limiting on feedback state
  };

}  // end namespace dfl

#endif  // dfl_RPMOscillator_h
