#pragma once

#include <cmath>
#include <algorithm>

// Simple TPT SVF that exposes LP/BP/HP outputs for morphing
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

  struct Outputs { double lp, bp, hp; };

  Outputs process(double in) {
    double hp = (in - k_ * z1_ - z2_) * a1_;
    double bp = g_ * hp + z1_;
    double lp = g_ * bp + z2_;
    z1_ = 2.0 * bp - z1_;
    z2_ = 2.0 * lp - z2_;
    return {lp, bp, hp};
  }

  void reset() { z1_ = z2_ = 0.0; }

private:
  void updateCoeffs() {
    g_ = std::tan(kPi * cutoff_ / sample_rate_);
    k_ = 2.0 * (1.0 - resonance_);
    a1_ = 1.0 / (1.0 + g_ * (g_ + k_));
  }

  double sample_rate_ = 44100.0;
  double cutoff_ = 150.0;
  double resonance_ = 0.7;
  double g_ = 0.0, k_ = 1.0, a1_ = 1.0;
  double z1_ = 0.0, z2_ = 0.0;
};

// 360-degree Filter Morphing Controller
// Maps polar coordinates to LP/BP/HP/BR with allpass at center
class FilterMorpher {
public:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kTwoPi = 2.0 * kPi;

  void setPosition(float x, float y) {
    pos_x_ = std::clamp(x, -1.0f, 1.0f);
    pos_y_ = std::clamp(y, -1.0f, 1.0f);
    updateWeights();
  }

  float posX() const { return pos_x_; }
  float posY() const { return pos_y_; }
  float radius() const { return radius_; }
  float angle() const { return angle_; }

  // Apply morph to filter outputs, returns mixed result
  double apply(double lp, double bp, double hp) const {
    // Notch = LP + HP (cancels BP)
    double notch = lp + hp;
    // Allpass approximation = LP + HP - BP (phase inverted BP)
    double allpass = lp + hp - bp;

    // Mix filter modes by weights
    double filtered = w_lp_ * lp + w_bp_ * bp + w_hp_ * hp + w_br_ * notch;

    // Blend between filtered and allpass based on radius
    // Center (r=0) = allpass, Edge (r=1) = fully filtered
    return (1.0f - radius_) * allpass + radius_ * filtered;
  }

  // Weight accessors for visualization
  float wLP() const { return w_lp_; }
  float wBP() const { return w_bp_; }
  float wHP() const { return w_hp_; }
  float wBR() const { return w_br_; }

private:
  void updateWeights() {
    radius_ = std::sqrt(pos_x_ * pos_x_ + pos_y_ * pos_y_);
    radius_ = std::min(radius_, 1.0f);

    angle_ = std::atan2(pos_y_, pos_x_);
    if (angle_ < 0) angle_ += static_cast<float>(kTwoPi);

    // Map angle to 4 modes evenly spaced:
    // 0 (right) = HP, pi/2 (up) = BP, pi (left) = LP, 3pi/2 (down) = BR
    float a = angle_ / static_cast<float>(kTwoPi);

    // Calculate weights using smooth cosine crossfades
    auto smoothWeight = [](float center, float a) {
      float dist = std::abs(a - center);
      if (dist > 0.5f) dist = 1.0f - dist;
      float w = std::cos(std::min(dist * 4.0f, 1.0f) * static_cast<float>(kPi) * 0.5f);
      return std::max(0.0f, w * w);
    };

    w_hp_ = smoothWeight(0.0f, a);
    w_bp_ = smoothWeight(0.25f, a);
    w_lp_ = smoothWeight(0.5f, a);
    w_br_ = smoothWeight(0.75f, a);

    // Handle wrap-around for HP
    w_hp_ = std::max(w_hp_, smoothWeight(1.0f, a));

    // Normalize weights
    float sum = w_lp_ + w_bp_ + w_hp_ + w_br_ + 0.0001f;
    w_lp_ /= sum;
    w_bp_ /= sum;
    w_hp_ /= sum;
    w_br_ /= sum;
  }

  float pos_x_ = 0.0f, pos_y_ = 0.0f;
  float radius_ = 0.0f, angle_ = 0.0f;
  float w_lp_ = 0.25f, w_bp_ = 0.25f, w_hp_ = 0.25f, w_br_ = 0.25f;
};
