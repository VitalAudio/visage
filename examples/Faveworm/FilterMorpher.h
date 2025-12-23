#pragma once

#include <algorithm>
#include <cmath>

// Filter output selection for X/Y routing
enum class FilterOutput {
  LP,
  BP,
  HP,
  BR,
  AP,
  Input
};

inline const char* filterOutputName(FilterOutput f) {
  switch (f) {
  case FilterOutput::LP: return "LP";
  case FilterOutput::BP: return "BP";
  case FilterOutput::HP: return "HP";
  case FilterOutput::BR: return "BR";
  case FilterOutput::AP: return "AP";
  case FilterOutput::Input: return "In";
  }
  return "?";
}

// Simple TPT SVF that exposes LP/BP/HP outputs for morphing
// Allows true self-oscillation with soft-clipping on feedback to prevent blowup
class SimpleSVF {
public:
  static constexpr double kPi = 3.14159265358979323846;

  void setSampleRate(double sr) {
    sample_rate_ = sr;
    updateCoeffs();
  }

  void setCutoff(double fc) {
    cutoff_ = std::clamp(fc, 20.0, sample_rate_ * 0.49);
    updateCoeffs();
  }

  void setResonance(double r) {
    resonance_ = std::clamp(r, 0.0, 1.0);
    updateCoeffs();
  }

  struct Outputs {
    double lp, bp, hp;
  };

  Outputs process(double in) {
    // Attenuate input to operate below saturation knee (preserves resonance)
    constexpr double kInputGain = 0.5;
    constexpr double kOutputGain = 2.0;  // Compensate output level

    in *= kInputGain;

    // Soft-clip the input combined with feedback - this is where energy enters
    // At high resonance the filter self-oscillates; saturation limits amplitude
    double v0 = softClip(in - k_ * z1_ - z2_);
    double hp = v0 * a1_;
    double bp = g_ * hp + z1_;
    double lp = g_ * bp + z2_;

    // Update state (trapezoidal integration)
    // Soft-clip before feedback to prevent runaway at extreme freq/resonance
    z1_ = 2.0 * softClip(bp) - z1_;
    z2_ = 2.0 * softClip(lp) - z2_;

    return { lp * kOutputGain, bp * kOutputGain, hp * kOutputGain };
  }

  void reset() { z1_ = z2_ = 0.0; }

  // Get specific output from last process() call
  static double getOutput(const Outputs& o, FilterOutput type, double input) {
    switch (type) {
    case FilterOutput::LP: return o.lp;
    case FilterOutput::BP: return o.bp;
    case FilterOutput::HP: return o.hp;
    case FilterOutput::BR: return o.lp + o.hp;  // Notch
    case FilterOutput::AP: return o.lp + o.hp - o.bp;  // Allpass approx
    case FilterOutput::Input: return input;
    }
    return input;
  }

private:
  // Soft saturation using tanh - allows self-oscillation to stabilize
  static double softClip(double x) { return std::tanh(x); }

  void updateCoeffs() {
    g_ = std::tan(kPi * cutoff_ / sample_rate_);
    // Allow true self-oscillation: k=0 when resonance=1
    k_ = 2.0 * (1.0 - resonance_);
    a1_ = 1.0 / (1.0 + g_ * (g_ + k_));
  }

  double sample_rate_ = 44100.0;
  double cutoff_ = 150.0;
  double resonance_ = 0.7;
  double g_ = 0.0, k_ = 1.0, a1_ = 1.0;
  double z1_ = 0.0, z2_ = 0.0;
};

// Stereo filter router: mono input -> X/Y outputs via selectable filter modes
// Perfect for Lissajous patterns where phase differences create rotation
class StereoFilterRouter {
public:
  // Preset split modes for interesting XY patterns
  // Each creates different visual characteristics due to phase relationships
  enum class SplitMode {
    LpHp,  // Classic retro: smooth ellipses, analog feel (fc 40-80, Q 0.7)
    BpAp,  // Geometric flowers: swirling shapes, BP + phase rotation (fc 150-250, Q 1.0)
    BrAp,  // Kaleidoscope: notch removes freqs, AP rotates phase (fc 300-800, Q 1.2-1.8)
    LpBp,  // Organic petals: soft loops, classic visualizer look (fc 100-200, Q 0.8)
    ApHp,  // Liquid vector: AP phase shift + HP edge (fc 20-40, Q 0.5-0.7)
    BpBr,  // Complementary: BP emphasizes what notch removes
    InMorph,  // Live control: X=raw input, Y=morphed (use with joystick)
    Custom  // Manual X/Y output selection
  };

  static constexpr int kNumModes = 8;

  void setSplitMode(SplitMode mode) {
    split_mode_ = mode;
    switch (mode) {
    case SplitMode::LpHp:
      x_output_ = FilterOutput::LP;
      y_output_ = FilterOutput::HP;
      break;
    case SplitMode::BpAp:
      x_output_ = FilterOutput::BP;
      y_output_ = FilterOutput::AP;
      break;
    case SplitMode::BrAp:
      x_output_ = FilterOutput::BR;
      y_output_ = FilterOutput::AP;
      break;
    case SplitMode::LpBp:
      x_output_ = FilterOutput::LP;
      y_output_ = FilterOutput::BP;
      break;
    case SplitMode::ApHp:
      x_output_ = FilterOutput::AP;
      y_output_ = FilterOutput::HP;
      break;
    case SplitMode::BpBr:
      x_output_ = FilterOutput::BP;
      y_output_ = FilterOutput::BR;
      break;
    case SplitMode::InMorph:
      x_output_ = FilterOutput::Input;
      y_output_ = FilterOutput::LP;
      break;
    case SplitMode::Custom: break;  // Keep current settings
    }
  }

  void cycleSplitMode() {
    int next = (static_cast<int>(split_mode_) + 1) % kNumModes;
    setSplitMode(static_cast<SplitMode>(next));
  }

  SplitMode splitMode() const { return split_mode_; }

  const char* splitModeName() const {
    switch (split_mode_) {
    case SplitMode::LpHp: return "LP/HP Retro";
    case SplitMode::BpAp: return "BP/AP Flowers";
    case SplitMode::BrAp: return "BR/AP Kaleidoscope";
    case SplitMode::LpBp: return "LP/BP Organic";
    case SplitMode::ApHp: return "AP/HP Liquid";
    case SplitMode::BpBr: return "BP/BR Complement";
    case SplitMode::InMorph: return "In/Morph Live";
    case SplitMode::Custom: return "Custom";
    }
    return "?";
  }

  const char* splitModeDescription() const {
    switch (split_mode_) {
    case SplitMode::LpHp: return "Classic ellipses (fc 40-80, Q 0.7)";
    case SplitMode::BpAp: return "Swirling shapes (fc 150-250, Q 1.0)";
    case SplitMode::BrAp: return "Geometric patterns (fc 300-800, Q 1.2)";
    case SplitMode::LpBp: return "Soft loops (fc 100-200, Q 0.8)";
    case SplitMode::ApHp: return "Phase + edge (fc 20-40, Q 0.5)";
    case SplitMode::BpBr: return "Complementary bands";
    case SplitMode::InMorph: return "X=raw, Y=filtered (use joystick)";
    case SplitMode::Custom: return "Manual X/Y selection";
    }
    return "";
  }

  // Manual output selection
  void setXOutput(FilterOutput f) {
    x_output_ = f;
    split_mode_ = SplitMode::Custom;
  }
  void setYOutput(FilterOutput f) {
    y_output_ = f;
    split_mode_ = SplitMode::Custom;
  }
  FilterOutput xOutput() const { return x_output_; }
  FilterOutput yOutput() const { return y_output_; }

  void setSampleRate(double sr) { svf_.setSampleRate(sr); }
  void setCutoff(double fc) { svf_.setCutoff(fc); }
  void setResonance(double r) { svf_.setResonance(r); }
  void reset() { svf_.reset(); }

  // Process mono input, return X/Y outputs
  struct StereoOut {
    double x, y;
  };

  StereoOut process(double input) {
    auto o = svf_.process(input);
    return { SimpleSVF::getOutput(o, x_output_, input), SimpleSVF::getOutput(o, y_output_, input) };
  }

private:
  SimpleSVF svf_;
  SplitMode split_mode_ = SplitMode::LpHp;
  FilterOutput x_output_ = FilterOutput::LP;
  FilterOutput y_output_ = FilterOutput::HP;
};

// 360-degree Filter Morphing Controller
// Maps polar coordinates to LP/BP/HP/BR with allpass at center
// Extended with split offsets for stereo: Y can be rotated/shifted relative to X
class FilterMorpher {
public:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kTwoPi = 2.0 * kPi;

  void setPosition(float x, float y) {
    // Constrain to unit circle (not square)
    float dist = std::sqrt(x * x + y * y);
    if (dist > 1.0f) {
      x /= dist;
      y /= dist;
    }
    pos_x_ = x;
    pos_y_ = y;
    updateWeights();
  }

  // Split offset controls
  // angle: 0-360 degrees, rotates Y's filter position relative to X
  void setSplitAngle(float degrees) { split_angle_ = degrees * static_cast<float>(kPi) / 180.0f; }
  float splitAngle() const { return split_angle_ * 180.0f / static_cast<float>(kPi); }

  // depth: 0 to +1, adjusts Y radius offset
  // 0 = no change, 1 = Y pinned to center (AP)
  void setSplitDepth(float d) { split_depth_ = std::clamp(d, 0.0f, 1.0f); }
  float splitDepth() const { return split_depth_; }

  bool hasSplit() const { return std::abs(split_angle_) > 0.01f || split_depth_ > 0.01f; }

  float posX() const { return pos_x_; }
  float posY() const { return pos_y_; }
  float radius() const { return radius_; }
  float angle() const { return angle_; }

  // Apply morph to filter outputs for a single channel (X)
  double apply(double lp, double bp, double hp) const {
    return applyAtPosition(lp, bp, hp, radius_, angle_);
  }

  // Apply morph with split offsets, returns both X and Y outputs
  struct StereoOut {
    double x, y;
  };
  StereoOut applyXY(double lp, double bp, double hp) const {
    // X channel: use base position (unaffected by depth in this mode)
    double x_out = applyAtPosition(lp, bp, hp, radius_, angle_);

    // Y channel: apply offset to position
    float y_angle = angle_ + split_angle_;
    // Wrap angle to 0-2π
    while (y_angle >= static_cast<float>(kTwoPi))
      y_angle -= static_cast<float>(kTwoPi);
    while (y_angle < 0)
      y_angle += static_cast<float>(kTwoPi);

    // Apply depth: pushes Y toward center (AP)
    float y_radius = radius_ * (1.0f - split_depth_);  // depth=1 means y_radius=0

    double y_out = applyAtPosition(lp, bp, hp, y_radius, y_angle);
    return { x_out, y_out };
  }

  // Weight accessors for visualization
  float wLP() const { return w_lp_; }
  float wBP() const { return w_bp_; }
  float wHP() const { return w_hp_; }
  float wBR() const { return w_br_; }

private:
  // Compute filter mix at a specific polar position
  double applyAtPosition(double lp, double bp, double hp, float r, float a) const {
    // Notch = LP + HP (cancels BP)
    double notch = lp + hp;
    // Allpass approximation = LP + HP - BP (phase inverted BP)
    double allpass = lp + hp - bp;

    // Compute weights for this angle
    float w_lp, w_bp, w_hp, w_br;
    computeWeights(a, w_lp, w_bp, w_hp, w_br);

    // Mix filter modes by weights
    double filtered = w_lp * lp + w_bp * bp + w_hp * hp + w_br * notch;

    // Blend between filtered and allpass based on radius
    // Center (r=0) = allpass, Edge (r=1) = fully filtered
    return (1.0f - r) * allpass + r * filtered;
  }

  static void computeWeights(float angle, float& w_lp, float& w_bp, float& w_hp, float& w_br) {
    // Map angle to 4 modes evenly spaced:
    // 0 (right) = HP, pi/2 (up) = BP, pi (left) = LP, 3pi/2 (down) = BR
    float a = angle / static_cast<float>(kTwoPi);

    // Calculate weights using smooth cosine crossfades
    auto smoothWeight = [](float center, float a) {
      float dist = std::abs(a - center);
      if (dist > 0.5f)
        dist = 1.0f - dist;
      float w = std::cos(std::min(dist * 4.0f, 1.0f) * static_cast<float>(kPi) * 0.5f);
      return std::max(0.0f, w * w);
    };

    w_hp = smoothWeight(0.0f, a);
    w_bp = smoothWeight(0.25f, a);
    w_lp = smoothWeight(0.5f, a);
    w_br = smoothWeight(0.75f, a);

    // Handle wrap-around for HP
    w_hp = std::max(w_hp, smoothWeight(1.0f, a));

    // Normalize weights
    float sum = w_lp + w_bp + w_hp + w_br + 0.0001f;
    w_lp /= sum;
    w_bp /= sum;
    w_hp /= sum;
    w_br /= sum;
  }

  void updateWeights() {
    radius_ = std::sqrt(pos_x_ * pos_x_ + pos_y_ * pos_y_);
    radius_ = std::min(radius_, 1.0f);

    angle_ = std::atan2(pos_y_, pos_x_);
    if (angle_ < 0)
      angle_ += static_cast<float>(kTwoPi);

    computeWeights(angle_, w_lp_, w_bp_, w_hp_, w_br_);
  }

  float pos_x_ = 0.0f, pos_y_ = 0.0f;
  float radius_ = 0.0f, angle_ = 0.0f;
  float w_lp_ = 0.25f, w_bp_ = 0.25f, w_hp_ = 0.25f, w_br_ = 0.25f;
  float split_angle_ = 0.0f;  // radians
  float split_depth_ = 0.0f;  // -1 to +1
};
