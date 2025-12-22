#pragma once

#include <visage/app.h>
#include <cmath>
#include <algorithm>
#include <functional>
#include <vector>
#include <string>
#include "FilterMorpher.h"
#include "embedded/example_fonts.h"

// Toggle switch (on/off)
class ToggleSwitch : public visage::Frame {
public:
  using Callback = std::function<void(bool)>;

  ToggleSwitch(const char* label = "") : label_(label) {
    setIgnoresMouseEvents(false, false);
  }

  void setValue(bool v) { value_ = v; }
  bool value() const { return value_; }
  void setCallback(Callback cb) { callback_ = cb; }
  void setColor(visage::Color c) { color_ = c; }
  void setEnabled(bool e) { enabled_ = e; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float label_h = 12.0f;
    const float sw = w * 0.5f;  // switch width
    const float sh = (h - label_h) * 0.5f;  // switch height
    const float sx = (w - sw) / 2;
    const float sy = label_h + ((h - label_h) - sh) / 2;
    const float r = sh / 2;
    const float dim = enabled_ ? 1.0f : 0.3f;

    // Label at top
    if (label_ && label_[0]) {
      visage::Font font(10, resources::fonts::DroidSansMono_ttf);
      canvas.setColor(visage::Color(0.7f * dim, 0.6f, 0.65f, 0.7f));
      canvas.text(label_, font, visage::Font::kCenter, 0, 0, w, label_h);
    }

    // Track background
    if (value_) {
      canvas.setColor(visage::Color(color_.alpha() * 0.5f * dim, color_.red(), color_.green(), color_.blue()));
    } else {
      canvas.setColor(visage::Color(0.5f * dim, 0.15f, 0.15f, 0.15f));
    }
    canvas.roundedRectangle(sx, sy, sw, sh, r);

    // Thumb
    float thumb_x = value_ ? (sx + sw - sh) : sx;
    if (value_) {
      canvas.setColor(visage::Color(color_.alpha() * dim, color_.red(), color_.green(), color_.blue()));
    } else {
      canvas.setColor(visage::Color(0.7f * dim, 0.3f, 0.3f, 0.3f));
    }
    canvas.circle(thumb_x + 2, sy + 2, sh - 4);

    redraw();
  }

  void mouseDown(const visage::MouseEvent&) override {
    if (!enabled_) return;
    value_ = !value_;
    if (callback_) callback_(value_);
  }

private:
  const char* label_;
  bool value_ = false;
  Callback callback_;
  visage::Color color_{1.0f, 0.5f, 0.9f, 0.8f};
  bool enabled_ = true;
};

// Mode selector (cycles through options on click)
class ModeSelector : public visage::Frame {
public:
  using Callback = std::function<void(int)>;

  ModeSelector(const char* label = "") : label_(label) {
    setIgnoresMouseEvents(false, false);
  }

  void setLabel(const char* label) { label_ = label; }
  void setOptions(const std::vector<std::string>& opts) { options_ = opts; }
  void setIndex(int i) { index_ = std::clamp(i, 0, static_cast<int>(options_.size()) - 1); }
  int index() const { return index_; }
  void setCallback(Callback cb) { callback_ = cb; }
  void setColor(visage::Color c) { color_ = c; }
  void setEnabled(bool e) { enabled_ = e; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float label_h = (label_ && label_[0]) ? 12.0f : 0.0f;
    const float ctrl_h = h - label_h;
    const float ctrl_y = label_h;
    const float dim = enabled_ ? 1.0f : 0.3f;

    // Label at top
    if (label_ && label_[0]) {
      visage::Font font(10, resources::fonts::DroidSansMono_ttf);
      canvas.setColor(visage::Color(0.7f * dim, 0.6f, 0.65f, 0.7f));
      canvas.text(label_, font, visage::Font::kCenter, 0, 0, w, label_h);
    }

    // Background
    canvas.setColor(visage::Color(0.6f * dim, 0.12f, 0.12f, 0.12f));
    canvas.roundedRectangle(2, ctrl_y + 2, w - 4, ctrl_h - 4, 4);

    // Border highlight
    canvas.setColor(visage::Color(color_.alpha() * dim, color_.red(), color_.green(), color_.blue()));
    canvas.roundedRectangle(0, ctrl_y, w, ctrl_h, 5);
    canvas.setColor(visage::Color(0.95f * dim, 0.08f, 0.08f, 0.08f));
    canvas.roundedRectangle(2, ctrl_y + 2, w - 4, ctrl_h - 4, 4);

    // Draw selected option text
    if (!options_.empty() && index_ >= 0 && index_ < static_cast<int>(options_.size())) {
      visage::Font font(11, resources::fonts::DroidSansMono_ttf);
      canvas.setColor(visage::Color(color_.alpha() * dim, color_.red(), color_.green(), color_.blue()));
      canvas.text(options_[index_].c_str(), font, visage::Font::kCenter,
                  4, ctrl_y, w - 8, ctrl_h);
    }

    redraw();
  }

  void mouseDown(const visage::MouseEvent&) override {
    if (!enabled_ || options_.empty()) return;
    index_ = (index_ + 1) % options_.size();
    if (callback_) callback_(index_);
  }

private:
  const char* label_;
  std::vector<std::string> options_;
  int index_ = 0;
  Callback callback_;
  visage::Color color_{1.0f, 0.5f, 0.9f, 0.8f};
  bool enabled_ = true;
};

// Simple rotary knob for filter parameters
class FilterKnob : public visage::Frame {
public:
  static constexpr float kPi = 3.14159265358979323846f;

  using Callback = std::function<void(float)>;

  FilterKnob(const char* label = "", bool logarithmic = false)
      : label_(label), logarithmic_(logarithmic) {
    setIgnoresMouseEvents(false, false);
  }

  void setValue(float* v) { value_ = v; }
  void setRange(float min, float max) { min_ = min; max_ = max; }
  void setColor(visage::Color c) { color_ = c; }
  void setCallback(Callback cb) { callback_ = cb; }
  void setEnabled(bool e) { enabled_ = e; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float label_h = 12.0f;
    const float knob_h = h - label_h;
    const float cx = w * 0.5f;
    const float cy = knob_h * 0.5f;
    const float r = std::min(w, knob_h) * 0.4f;
    const float dim = enabled_ ? 1.0f : 0.3f;

    // Background
    canvas.setColor(visage::Color(0.6f * dim, 0.08f, 0.08f, 0.08f));
    canvas.circle(cx - r, cy - r, r * 2);

    // Arc showing value range (270 degrees, from 135 to 405)
    const float start_angle = 0.75f * kPi;  // 135 degrees
    const float range = 1.5f * kPi;         // 270 degrees

    // Draw tick marks as small circles
    canvas.setColor(visage::Color(0.5f * dim, 0.3f, 0.3f, 0.3f));
    for (int i = 0; i <= 10; ++i) {
      float t = static_cast<float>(i) / 10.0f;
      float angle = start_angle + t * range;
      float tick_r = r * 0.85f;
      canvas.circle(cx + std::cos(angle) * tick_r - 2, cy + std::sin(angle) * tick_r - 2, 4);
    }

    // Value indicator
    if (value_) {
      float norm = getNormalizedValue();
      float angle = start_angle + norm * range;

      // Arc fill
      canvas.setColor(visage::Color(color_.alpha() * 0.4f * dim, color_.red(), color_.green(), color_.blue()));
      const int segments = static_cast<int>(norm * 30) + 1;
      for (int i = 0; i < segments; ++i) {
        float t0 = static_cast<float>(i) / segments * norm;
        float t1 = static_cast<float>(i + 1) / segments * norm;
        float a0 = start_angle + t0 * range;
        float a1 = start_angle + t1 * range;
        float ri = r * 0.6f;
        float ro = r * 0.85f;
        canvas.triangle(cx + std::cos(a0) * ri, cy + std::sin(a0) * ri,
                       cx + std::cos(a0) * ro, cy + std::sin(a0) * ro,
                       cx + std::cos(a1) * ri, cy + std::sin(a1) * ri);
        canvas.triangle(cx + std::cos(a1) * ri, cy + std::sin(a1) * ri,
                       cx + std::cos(a0) * ro, cy + std::sin(a0) * ro,
                       cx + std::cos(a1) * ro, cy + std::sin(a1) * ro);
      }

      // Pointer as elongated dot
      canvas.setColor(visage::Color(color_.alpha() * dim, color_.red(), color_.green(), color_.blue()));
      float px = cx + std::cos(angle) * r * 0.5f;
      float py = cy + std::sin(angle) * r * 0.5f;
      canvas.circle(px - 5, py - 5, 10);

      // Center dot
      canvas.setColor(visage::Color(0.6f * dim, 0.2f, 0.2f, 0.2f));
      canvas.circle(cx - 6, cy - 6, 12);
    }

    // Label at bottom
    if (label_ && label_[0]) {
      visage::Font font(10, resources::fonts::DroidSansMono_ttf);
      canvas.setColor(visage::Color(0.7f * dim, 0.6f, 0.65f, 0.7f));
      canvas.text(label_, font, visage::Font::kCenter, 0, h - label_h, w, label_h);
    }

    redraw();
  }

  void mouseDown(const visage::MouseEvent& event) override {
    if (!enabled_) return;
    dragging_ = true;
    last_y_ = event.position.y;
  }

  void mouseDrag(const visage::MouseEvent& event) override {
    if (!enabled_ || !dragging_ || !value_) return;

    float delta = (last_y_ - event.position.y) / 100.0f;
    last_y_ = event.position.y;

    if (logarithmic_) {
      // Use exponential scaling: full drag range covers log ratio
      // For 20-2000Hz (100x), delta of 1.0 should cover full range
      float log_range = std::log(max_ / min_);
      float log_delta = delta * log_range;
      *value_ = std::clamp(*value_ * std::exp(log_delta), min_, max_);
    } else {
      *value_ = std::clamp(*value_ + delta * (max_ - min_), min_, max_);
    }
    if (callback_) callback_(*value_);
  }

  void mouseUp(const visage::MouseEvent&) override {
    dragging_ = false;
  }

  bool mouseWheel(const visage::MouseEvent& event) override {
    if (!enabled_ || !value_) return false;

    float wheel_delta = event.wheel_delta_y * 0.05f;
    if (logarithmic_) {
      float log_range = std::log(max_ / min_);
      float log_delta = wheel_delta * log_range;
      *value_ = std::clamp(*value_ * std::exp(log_delta), min_, max_);
    } else {
      *value_ = std::clamp(*value_ + wheel_delta * (max_ - min_), min_, max_);
    }
    if (callback_) callback_(*value_);
    return true;
  }

private:
  float getNormalizedValue() const {
    if (!value_) return 0.0f;
    if (logarithmic_) {
      float log_min = std::log(min_);
      float log_max = std::log(max_);
      float log_val = std::log(*value_);
      return (log_val - log_min) / (log_max - log_min);
    }
    return (*value_ - min_) / (max_ - min_);
  }

  const char* label_;
  bool logarithmic_;
  float* value_ = nullptr;
  float min_ = 0.0f, max_ = 1.0f;
  visage::Color color_{1.0f, 0.5f, 0.8f, 0.8f};
  Callback callback_;
  bool dragging_ = false;
  float last_y_ = 0.0f;
  bool enabled_ = true;
};

// Numeric display with digital font
class NumericDisplay : public visage::Frame {
public:
  NumericDisplay(const char* suffix = "") : suffix_(suffix) {}

  void setValue(float* v) { value_ = v; }
  void setColor(visage::Color c) { color_ = c; }
  void setDecimals(int d) { decimals_ = d; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());

    // Dark background
    canvas.setColor(visage::Color(0.8f, 0.02f, 0.03f, 0.05f));
    canvas.roundedRectangle(0, 0, w, h, 3);

    // Value text
    if (value_) {
      char buf[32];
      if (decimals_ == 0) {
        snprintf(buf, sizeof(buf), "%.0f%s", *value_, suffix_);
      } else if (decimals_ == 1) {
        snprintf(buf, sizeof(buf), "%.1f%s", *value_, suffix_);
      } else if (decimals_ == 2) {
        snprintf(buf, sizeof(buf), "%.2f%s", *value_, suffix_);
      } else {
        snprintf(buf, sizeof(buf), "%.3f%s", *value_, suffix_);
      }

      visage::Font font(14, resources::fonts::DS_DIGIT_ttf);
      canvas.setColor(color_);
      canvas.text(buf, font, visage::Font::kCenter, 2, 0, w - 4, h);
    }

    redraw();
  }

private:
  float* value_ = nullptr;
  const char* suffix_;
  visage::Color color_{1.0f, 0.4f, 1.0f, 0.8f};
  int decimals_ = 0;
};

// Visual Joystick Control for Filter Morphing
class FilterJoystick : public visage::Frame {
public:
  static constexpr float kPi = 3.14159265358979323846f;

  FilterJoystick(const char* label = "Morph") : label_(label) {
    setIgnoresMouseEvents(false, false);
  }

  void setLabel(const char* label) { label_ = label; }
  void setMorpher(FilterMorpher* m) { morpher_ = m; }
  void setCutoff(float* c) { cutoff_ = c; }
  void setResonance(float* r) { resonance_ = r; }
  void setEnabled(bool e) { enabled_ = e; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float label_h = 12.0f;
    const float js_h = h - label_h;
    const float cx = w * 0.5f;
    const float cy = js_h * 0.5f;
    const float r = std::min(w, js_h) * 0.45f;
    const float dim = enabled_ ? 1.0f : 0.3f;

    // Dark background circle
    canvas.setColor(visage::Color(0.7f * dim, 0.06f, 0.08f, 0.12f));
    canvas.circle(cx - r, cy - r, r * 2);

    // Ring gradient for mode zones
    const int segments = 64;
    for (int i = 0; i < segments; ++i) {
      float a0 = kPi * 2.0f * i / segments;
      float a1 = kPi * 2.0f * (i + 1) / segments;
      float am = (a0 + a1) * 0.5f;

      float norm_a = am / (kPi * 2.0f);
      visage::Color color = getModeColor(norm_a);
      color = visage::Color(color.alpha() * 0.3f * dim, color.red(), color.green(), color.blue());

      float x0 = cx + std::cos(a0) * r;
      float y0 = cy + std::sin(a0) * r;
      float x1 = cx + std::cos(a1) * r;
      float y1 = cy + std::sin(a1) * r;
      float x0i = cx + std::cos(a0) * r * 0.85f;
      float y0i = cy + std::sin(a0) * r * 0.85f;
      float x1i = cx + std::cos(a1) * r * 0.85f;
      float y1i = cy + std::sin(a1) * r * 0.85f;

      canvas.setColor(color);
      canvas.triangle(x0, y0, x1, y1, x0i, y0i);
      canvas.triangle(x0i, y0i, x1, y1, x1i, y1i);
    }

    // Mode labels with text
    visage::Font mode_font(8, resources::fonts::DroidSansMono_ttf);
    float labelR = r * 0.65f;
    canvas.setColor(visage::Color(0.7f * dim, 0.5f, 0.6f, 0.6f));
    canvas.text("HP", mode_font, visage::Font::kCenter, cx + labelR - 8, cy - 5, 16, 10);
    canvas.text("BP", mode_font, visage::Font::kCenter, cx - 8, cy - labelR - 5, 16, 10);
    canvas.text("LP", mode_font, visage::Font::kCenter, cx - labelR - 8, cy - 5, 16, 10);
    canvas.text("BR", mode_font, visage::Font::kCenter, cx - 8, cy + labelR - 5, 16, 10);
    canvas.setColor(visage::Color(0.5f * dim, 0.4f, 0.5f, 0.5f));
    canvas.text("AP", mode_font, visage::Font::kCenter, cx - 8, cy - 5, 16, 10);

    // Current position indicator
    if (morpher_) {
      float px = cx + morpher_->posX() * r;
      float py = cy - morpher_->posY() * r;

      // Glow
      canvas.setColor(visage::Color(0.4f * dim, 0.3f, 1.0f, 0.5f));
      canvas.circle(px - 12, py - 12, 24);

      // Dot
      canvas.setColor(visage::Color(1.0f * dim, 0.4f, 1.0f, 0.9f));
      canvas.circle(px - 6, py - 6, 12);
    }

    // Title label at bottom
    if (label_ && label_[0]) {
      visage::Font font(10, resources::fonts::DroidSansMono_ttf);
      canvas.setColor(visage::Color(0.7f * dim, 0.6f, 0.65f, 0.7f));
      canvas.text(label_, font, visage::Font::kCenter, 0, h - label_h, w, label_h);
    }

    redraw();
  }

  void mouseDown(const visage::MouseEvent& event) override {
    if (!enabled_) return;
    dragging_ = true;
    updatePosition(static_cast<int>(event.position.x), static_cast<int>(event.position.y));
  }

  void mouseDrag(const visage::MouseEvent& event) override {
    if (!enabled_) return;
    if (dragging_)
      updatePosition(static_cast<int>(event.position.x), static_cast<int>(event.position.y));
  }

  void mouseUp(const visage::MouseEvent& event) override {
    dragging_ = false;
  }

  bool mouseWheel(const visage::MouseEvent& event) override {
    if (!enabled_) return false;
    if (event.isShiftDown()) {
      if (resonance_) {
        *resonance_ = std::clamp(*resonance_ + event.wheel_delta_y * 0.02f, 0.0f, 1.0f);
      }
    } else {
      if (cutoff_) {
        float mult = event.wheel_delta_y > 0 ? 1.05f : 0.95f;
        *cutoff_ = std::clamp(*cutoff_ * mult, 20.0f, 2000.0f);
      }
    }
    return true;
  }

private:
  void updatePosition(int mx, int my) {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float label_h = 12.0f;
    const float js_h = h - label_h;
    const float cx = w * 0.5f;
    const float cy = js_h * 0.5f;
    const float r = std::min(w, js_h) * 0.45f;

    float x = (mx - cx) / r;
    float y = -(my - cy) / r;

    // Constrain to circular boundary
    float dist = std::sqrt(x * x + y * y);
    if (dist > 1.0f) {
      x /= dist;
      y /= dist;
    }

    if (morpher_)
      morpher_->setPosition(x, y);
  }

  visage::Color getModeColor(float norm_angle) {
    if (norm_angle < 0.125f || norm_angle >= 0.875f)
      return visage::Color(1.0f, 0.2f, 0.9f, 1.0f);  // HP: cyan
    else if (norm_angle < 0.375f)
      return visage::Color(1.0f, 1.0f, 0.9f, 0.2f);  // BP: yellow
    else if (norm_angle < 0.625f)
      return visage::Color(1.0f, 0.2f, 1.0f, 0.3f);  // LP: green
    else
      return visage::Color(1.0f, 0.9f, 0.3f, 0.9f);  // BR: magenta
  }

  const char* label_;
  FilterMorpher* morpher_ = nullptr;
  float* cutoff_ = nullptr;
  float* resonance_ = nullptr;
  bool dragging_ = false;
  bool enabled_ = true;
};
