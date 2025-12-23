#pragma once

#include "dsp/dfl_RPMOscillator.h"

#include <algorithm>
#include <cmath>
#include <random>

// Test signal generator for XY oscilloscope patterns
// Generates two channels (X/Y) with configurable waveforms
class TestSignalGenerator {
public:
  static constexpr double kMinFrequency = 10.0;
  static constexpr double kMaxFrequency = 500.0;
  static constexpr double kMinDetune = 0.9;
  static constexpr double kMaxDetune = 1.1;
  static constexpr double kMinBeta = -10.0;
  static constexpr double kMaxBeta = 50.0;

  TestSignalGenerator() {
    setSampleRate(44100.0);
    setFrequency(80.0);  // Base frequency for good XY visuals
    setDetune(1.003);  // Slight detune for slow rotation
    setBeta(0.0);
    setExponent(1);
    // Enable soft clipping to apply tanh saturation at extreme beta
    osc_x_.setSoftClip(true);
    osc_y_.setSoftClip(true);
  }

  void setSampleRate(double sr) {
    sample_rate_ = sr;
    osc_x_.setSampleRate(sr);
    osc_y_.setSampleRate(sr);
    updateFrequencies();
  }

  void setFrequency(double hz) {
    base_freq_ = std::max(kMinFrequency, std::min(kMaxFrequency, hz));
    updateFrequencies();
  }

  // Detune ratio for Y oscillator (1.0 = unison, 1.01 = 1% sharp)
  void setDetune(double ratio) {
    detune_ = std::clamp(ratio, kMinDetune, kMaxDetune);
    updateFrequencies();
  }

  void setBeta(double b) {
    beta_ = std::clamp(b, kMinBeta, kMaxBeta);
    osc_x_.setBeta(beta_);
    osc_y_.setBeta(beta_);
  }

  void setExponent(int e) {
    exponent_ = e;
    osc_x_.setExponent(e);
    osc_y_.setExponent(e);
  }

  double beta() const { return beta_; }
  int exponent() const { return exponent_; }

  const char* waveformName() const { return "RPM"; }

  // Generate samples into provided buffers
  void generate(float* left, float* right, int num_samples) {
    for (int i = 0; i < num_samples; ++i) {
      left[i] = static_cast<float>(osc_x_.getSample());
      right[i] = static_cast<float>(osc_y_.getSample());
    }
  }

  // Generate a single stereo sample
  void getSample(float& left, float& right) {
    if (paused_) {
      left = last_left_;
      right = last_right_;
    }
    else {
      left = static_cast<float>(osc_x_.getSample());
      right = static_cast<float>(osc_y_.getSample());
      last_left_ = left;
      last_right_ = right;
    }
  }

  void reset() {
    osc_x_.reset();
    osc_y_.reset();
  }

  void setPaused(bool p) { paused_ = p; }
  bool isPaused() const { return paused_; }
  void togglePause() { paused_ = !paused_; }

  void advance(int samples) {
    double s = static_cast<double>(samples);
    osc_x_.advancePhase(s);
    osc_y_.advancePhase(s);
  }

  void step(int num_samples) {
    if (num_samples > 0) {
      for (int i = 0; i < num_samples; ++i) {
        last_left_ = static_cast<float>(osc_x_.getSample());
        last_right_ = static_cast<float>(osc_y_.getSample());
      }
    }
    else {
      // For stateful oscillators, we can't easily go back.
      // We could reset and fast-forward, but for now we'll just ignore negative steps
      // or at least not crash. The time_offset_ in Oscilloscope will still shift
      // the demo patterns and audio player correctly.
    }
  }

private:
  void updateFrequencies() {
    osc_x_.setFrequency(base_freq_);
    osc_y_.setFrequency(base_freq_ * detune_);
  }

  dfl::RPMOscillator osc_x_;
  dfl::RPMOscillator osc_y_;
  double sample_rate_ = 44100.0;
  double base_freq_ = 80.0;
  double detune_ = 1.003;
  double beta_ = 0.0;
  int exponent_ = 1;
  bool paused_ = false;
  float last_left_ = 0.0f;
  float last_right_ = 0.0f;
};
