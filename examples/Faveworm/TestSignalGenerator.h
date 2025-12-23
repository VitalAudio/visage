#pragma once

#include "dsp/dfl_RPMOscillator.h"

#include <algorithm>
#include <cmath>
#include <random>

// Test signal generator for XY oscilloscope patterns
// Generates two channels (X/Y) with configurable waveforms
class TestSignalGenerator {
public:
  enum class Waveform {
    Sine,  // Pure sine - clean ellipses
    Triangle,  // Low beta RPM - soft harmonics
    Saw,  // Positive beta RPM - rich harmonics
    Square,  // Negative beta + exp 2 - hard edges
    Noise  // High beta + soft clip - chaotic
  };

  TestSignalGenerator() {
    setSampleRate(44100.0);
    setFrequency(80.0);  // Base frequency for good XY visuals
    setDetune(1.003);  // Slight detune for slow rotation
    setWaveform(Waveform::Sine);
  }

  void setSampleRate(double sr) {
    sample_rate_ = sr;
    osc_x_.setSampleRate(sr);
    osc_y_.setSampleRate(sr);
    updateFrequencies();
  }

  void setFrequency(double hz) {
    base_freq_ = std::clamp(hz, 10.0, 500.0);
    updateFrequencies();
  }

  // Detune ratio for Y oscillator (1.0 = unison, 1.01 = 1% sharp)
  void setDetune(double ratio) {
    detune_ = std::clamp(ratio, 0.9, 1.1);
    updateFrequencies();
  }

  void setWaveform(Waveform wf) {
    waveform_ = wf;
    // Reset oscillator state when changing waveforms to clear any bad state
    osc_x_.reset();
    osc_y_.reset();
    switch (wf) {
    case Waveform::Sine:
      osc_x_.setBeta(0.0);
      osc_x_.setExponent(1);
      osc_x_.setSoftClip(false);
      osc_y_.setBeta(0.0);
      osc_y_.setExponent(1);
      osc_y_.setSoftClip(false);
      break;
    case Waveform::Triangle:
      osc_x_.setBeta(0.3);
      osc_x_.setExponent(1);
      osc_x_.setSoftClip(false);
      osc_y_.setBeta(0.3);
      osc_y_.setExponent(1);
      osc_y_.setSoftClip(false);
      break;
    case Waveform::Saw:
      osc_x_.setSawMode(1.2);
      osc_x_.setSoftClip(false);
      osc_y_.setSawMode(1.2);
      osc_y_.setSoftClip(false);
      break;
    case Waveform::Square:
      osc_x_.setSquareMode(1.5);
      osc_x_.setSoftClip(false);
      osc_y_.setSquareMode(1.5);
      osc_y_.setSoftClip(false);
      break;
    case Waveform::Noise:
      osc_x_.setBeta(4.0);
      osc_x_.setExponent(1);
      osc_x_.setSoftClip(true);
      osc_y_.setBeta(4.0);
      osc_y_.setExponent(1);
      osc_y_.setSoftClip(true);
      break;
    }
  }

  void cycleWaveform() {
    int next = (static_cast<int>(waveform_) + 1) % 5;
    setWaveform(static_cast<Waveform>(next));
  }

  Waveform waveform() const { return waveform_; }
  double frequency() const { return base_freq_; }
  double detune() const { return detune_; }

  const char* waveformName() const {
    switch (waveform_) {
    case Waveform::Sine: return "Sine";
    case Waveform::Triangle: return "Triangle";
    case Waveform::Saw: return "Saw";
    case Waveform::Square: return "Square";
    case Waveform::Noise: return "Noise";
    }
    return "Unknown";
  }

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
  Waveform waveform_ = Waveform::Sine;
  bool paused_ = false;
  float last_left_ = 0.0f;
  float last_right_ = 0.0f;
};
