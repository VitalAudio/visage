#pragma once

#include "embedded/example_fonts.h"
#include "FilterMorpher.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <visage/app.h>

// Toggle switch (on/off) - sliding style
class ToggleSwitch : public visage::Frame {
public:
  using Callback = std::function<void(bool)>;

  ToggleSwitch(const char* label = "") : label_(label) { setIgnoresMouseEvents(false, false); }

  void setValue(bool v, bool send_callback = false) {
    value_ = v;
    if (send_callback && callback_)
      callback_(value_);
  }
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
      canvas.setColor(visage::Color(color_.alpha() * 0.5f * dim, color_.red(), color_.green(),
                                    color_.blue()));
    }
    else {
      canvas.setColor(visage::Color(0.5f * dim, 0.15f, 0.15f, 0.15f));
    }
    canvas.roundedRectangle(sx, sy, sw, sh, r);

    // Thumb
    float thumb_x = value_ ? (sx + sw - sh) : sx;
    if (value_) {
      canvas.setColor(visage::Color(color_.alpha() * dim, color_.red(), color_.green(), color_.blue()));
    }
    else {
      canvas.setColor(visage::Color(0.7f * dim, 0.3f, 0.3f, 0.3f));
    }
    canvas.circle(thumb_x + 2, sy + 2, sh - 4);

    redraw();
  }

  void mouseDown(const visage::MouseEvent&) override {
    if (!enabled_)
      return;
    value_ = !value_;
    if (callback_)
      callback_(value_);
  }

private:
  const char* label_;
  bool value_ = false;
  Callback callback_;
  visage::Color color_ { 1.0f, 0.5f, 0.9f, 0.8f };
  bool enabled_ = true;
};

// Retro square push button with LED indicator (oscilloscope style)
class PushButtonSwitch : public visage::Frame {
public:
  using Callback = std::function<void(bool)>;

  PushButtonSwitch(const char* label = "") : label_(label) { setIgnoresMouseEvents(false, false); }

  void setValue(bool v, bool send_callback = false) {
    value_ = v;
    if (send_callback && callback_)
      callback_(value_);
  }
  bool value() const { return value_; }
  void setCallback(Callback cb) { callback_ = cb; }
  void setColor(visage::Color c) { color_ = c; }
  void setEnabled(bool e) { enabled_ = e; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float label_h = 12.0f;
    const float btn_size = std::min(w * 0.6f, h - label_h - 10.0f);
    const float bx = (w - btn_size) * 0.5f;
    const float by = (h - label_h - btn_size) * 0.5f;
    const float dim = enabled_ ? 1.0f : 0.3f;
    const float led_r = 3.0f;
    const float led_y = by - led_r * 2 - 2;

    // LED indicator above button
    float led_x = w * 0.5f;
    if (value_) {
      // On: bright LED with bloom
      canvas.setColor(visage::Color(1.0f * dim, 0.3f, 1.0f, 0.3f));
      canvas.circle(led_x - led_r, led_y - led_r, led_r * 2);
      // Bloom core (HDR)
      float hdr = 3.0f;
      canvas.setColor(visage::Color(1.0f * dim, 0.5f, 1.0f, 0.4f, hdr));
      canvas.circle(led_x - led_r * 0.6f, led_y - led_r * 0.6f, led_r * 1.2f);
    }
    else {
      // Off: dim LED
      canvas.setColor(visage::Color(0.4f * dim, 0.15f, 0.15f, 0.1f));
      canvas.circle(led_x - led_r, led_y - led_r, led_r * 2);
    }

    // Button bezel (outer frame)
    canvas.setColor(visage::Color(0.8f * dim, 0.25f, 0.25f, 0.25f));
    canvas.roundedRectangle(bx - 2, by - 2, btn_size + 4, btn_size + 4, 3);

    // Button 3D effect - depends on pressed state
    if (value_) {
      // Pressed: inverted bevel (dark top-left, light bottom-right)
      canvas.setColor(visage::Color(0.7f * dim, 0.08f, 0.08f, 0.08f));
      canvas.fill(bx, by, btn_size, 2);  // top edge dark
      canvas.fill(bx, by, 2, btn_size);  // left edge dark
      canvas.setColor(visage::Color(0.5f * dim, 0.35f, 0.35f, 0.35f));
      canvas.fill(bx, by + btn_size - 2, btn_size, 2);  // bottom edge light
      canvas.fill(bx + btn_size - 2, by, 2, btn_size);  // right edge light

      // Button face (slightly darker when pressed)
      canvas.setColor(visage::Color(0.9f * dim, 0.12f, 0.12f, 0.12f));
      canvas.roundedRectangle(bx + 2, by + 2, btn_size - 4, btn_size - 4, 2);
    }
    else {
      // Not pressed: normal bevel (light top-left, dark bottom-right)
      canvas.setColor(visage::Color(0.7f * dim, 0.35f, 0.35f, 0.35f));
      canvas.fill(bx, by, btn_size, 2);  // top edge light
      canvas.fill(bx, by, 2, btn_size);  // left edge light
      canvas.setColor(visage::Color(0.5f * dim, 0.08f, 0.08f, 0.08f));
      canvas.fill(bx, by + btn_size - 2, btn_size, 2);  // bottom edge dark
      canvas.fill(bx + btn_size - 2, by, 2, btn_size);  // right edge dark

      // Button face
      canvas.setColor(visage::Color(0.9f * dim, 0.18f, 0.18f, 0.18f));
      canvas.roundedRectangle(bx + 2, by + 2, btn_size - 4, btn_size - 4, 2);
    }

    // Center marking (crosshair or square texture)
    float mark_size = btn_size * 0.25f;
    float mark_x = bx + (btn_size - mark_size) * 0.5f;
    float mark_y = by + (btn_size - mark_size) * 0.5f;
    if (value_) {
      canvas.setColor(visage::Color(color_.alpha() * 0.6f * dim, color_.red(), color_.green(),
                                    color_.blue()));
    }
    else {
      canvas.setColor(visage::Color(0.3f * dim, 0.25f, 0.25f, 0.25f));
    }
    canvas.fill(mark_x, mark_y, mark_size, mark_size);

    // Label at bottom
    if (label_ && label_[0]) {
      visage::Font font(10, resources::fonts::DroidSansMono_ttf);
      canvas.setColor(visage::Color(0.7f * dim, 0.6f, 0.65f, 0.7f));
      canvas.text(label_, font, visage::Font::kCenter, 0, h - label_h, w, label_h);
    }

    redraw();
  }

  void mouseDown(const visage::MouseEvent&) override {
    if (!enabled_)
      return;
    value_ = !value_;
    if (callback_)
      callback_(value_);
  }

private:
  const char* label_;
  bool value_ = false;
  Callback callback_;
  visage::Color color_ { 1.0f, 0.5f, 0.9f, 0.8f };
  bool enabled_ = true;
};

// Mode selector (cycles through options on click)
class ModeSelector : public visage::Frame {
public:
  using Callback = std::function<void(int)>;

  ModeSelector(const char* label = "") : label_(label) { setIgnoresMouseEvents(false, false); }

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
      canvas.text(options_[index_].c_str(), font, visage::Font::kCenter, 4, ctrl_y, w - 8, ctrl_h);
    }

    redraw();
  }

  void mouseDown(const visage::MouseEvent&) override {
    if (!enabled_ || options_.empty())
      return;
    index_ = (index_ + 1) % options_.size();
    if (callback_)
      callback_(index_);
  }

private:
  const char* label_;
  std::vector<std::string> options_;
  int index_ = 0;
  Callback callback_;
  visage::Color color_ { 1.0f, 0.5f, 0.9f, 0.8f };
  bool enabled_ = true;
};

// Simple rotary knob for filter parameters
class FilterKnob : public visage::Frame {
public:
  static constexpr float kPi = 3.14159265358979323846f;

  using Callback = std::function<void(float)>;

  FilterKnob(const char* label = "", bool logarithmic = false, bool bipolar_logarithmic = false,
             bool bidirectional = false) :
      label_(label),
      logarithmic_(logarithmic), bipolar_logarithmic_(bipolar_logarithmic),
      bidirectional_(bidirectional) {
    setIgnoresMouseEvents(false, false);
  }

  void setValue(float* v) { value_ = v; }
  void setLedIntensity(float* v) { led_intensity_ = v; }
  void setRange(float min, float max) {
    min_ = min;
    max_ = max;
  }
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
    const float range = 1.5f * kPi;  // 270 degrees

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
      canvas.setColor(visage::Color(color_.alpha() * 0.4f * dim, color_.red(), color_.green(),
                                    color_.blue()));

      if (bidirectional_) {
        // Fill from center (0.5) to current position
        float center_norm = 0.5f;
        float fill_start = std::min(norm, center_norm);
        float fill_end = std::max(norm, center_norm);
        float fill_range = fill_end - fill_start;
        const int segments = static_cast<int>(fill_range * 30) + 1;

        for (int i = 0; i < segments; ++i) {
          float t0 = fill_start + static_cast<float>(i) / segments * fill_range;
          float t1 = fill_start + static_cast<float>(i + 1) / segments * fill_range;
          float a0 = start_angle + t0 * range;
          float a1 = start_angle + t1 * range;
          float ri = r * 0.6f;
          float ro = r * 0.85f;
          canvas.triangle(cx + std::cos(a0) * ri, cy + std::sin(a0) * ri, cx + std::cos(a0) * ro,
                          cy + std::sin(a0) * ro, cx + std::cos(a1) * ri, cy + std::sin(a1) * ri);
          canvas.triangle(cx + std::cos(a1) * ri, cy + std::sin(a1) * ri, cx + std::cos(a0) * ro,
                          cy + std::sin(a0) * ro, cx + std::cos(a1) * ro, cy + std::sin(a1) * ro);
        }
      }
      else {
        // Fill from start to current position (original behavior)
        const int segments = static_cast<int>(norm * 30) + 1;
        for (int i = 0; i < segments; ++i) {
          float t0 = static_cast<float>(i) / segments * norm;
          float t1 = static_cast<float>(i + 1) / segments * norm;
          float a0 = start_angle + t0 * range;
          float a1 = start_angle + t1 * range;
          float ri = r * 0.6f;
          float ro = r * 0.85f;
          canvas.triangle(cx + std::cos(a0) * ri, cy + std::sin(a0) * ri, cx + std::cos(a0) * ro,
                          cy + std::sin(a0) * ro, cx + std::cos(a1) * ri, cy + std::sin(a1) * ri);
          canvas.triangle(cx + std::cos(a1) * ri, cy + std::sin(a1) * ri, cx + std::cos(a0) * ro,
                          cy + std::sin(a0) * ro, cx + std::cos(a1) * ro, cy + std::sin(a1) * ro);
        }
      }

      // Pointer as elongated dot
      canvas.setColor(visage::Color(color_.alpha() * dim, color_.red(), color_.green(), color_.blue()));
      float px = cx + std::cos(angle) * r * 0.5f;
      float py = cy + std::sin(angle) * r * 0.5f;
      canvas.circle(px - 5, py - 5, 10);

      // Center dot
      canvas.setColor(visage::Color(0.6f * dim, 0.2f, 0.2f, 0.2f));
      canvas.circle(cx - 6, cy - 6, 12);

      // LED Indicator (blooms if led_intensity_ is high)
      if (led_intensity_) {
        float intensity = *led_intensity_;
        // Position it at the top of the knob
        float lx = cx;
        float ly = cy - r * 0.7f;
        float led_r = 3.0f;

        // Base LED color
        canvas.setColor(visage::Color(1.0f * dim, 0.9f, 0.4f, 0.4f));
        canvas.circle(lx - led_r, ly - led_r, led_r * 2);

        // Blooming core (high HDR value triggers post-effect bloom)
        if (intensity > 0.05f) {
          float hdr = 1.0f + intensity * 8.0f;  // Scale up for bloom post effect
          canvas.setColor(visage::Color(1.0f * dim, 1.0f, 0.9f, 0.8f, hdr));
          canvas.circle(lx - led_r * 0.5f, ly - led_r * 0.5f, led_r);
        }
      }
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
    if (!enabled_)
      return;
    dragging_ = true;
    last_y_ = event.position.y;
  }

  void mouseDrag(const visage::MouseEvent& event) override {
    if (!enabled_ || !dragging_ || !value_)
      return;

    float delta = (last_y_ - event.position.y) / 100.0f;
    last_y_ = event.position.y;

    if (bipolar_logarithmic_) {
      // Bipolar logarithmic: symmetric log scaling around zero
      float norm = getNormalizedValue();
      norm = std::clamp(norm + delta, 0.0f, 1.0f);
      *value_ = denormalizeBipolarLog(norm);
    }
    else if (logarithmic_) {
      // Use exponential scaling: full drag range covers log ratio
      // For 20-2000Hz (100x), delta of 1.0 should cover full range
      float log_range = std::log(max_ / min_);
      float log_delta = delta * log_range;
      *value_ = std::clamp(*value_ * std::exp(log_delta), min_, max_);
    }
    else {
      *value_ = std::clamp(*value_ + delta * (max_ - min_), min_, max_);
    }
    if (callback_)
      callback_(*value_);
  }

  void mouseUp(const visage::MouseEvent&) override { dragging_ = false; }

  bool mouseWheel(const visage::MouseEvent& event) override {
    if (!enabled_ || !value_)
      return false;

    float wheel_delta = event.wheel_delta_y * 0.05f;
    if (bipolar_logarithmic_) {
      float norm = getNormalizedValue();
      norm = std::clamp(norm + wheel_delta, 0.0f, 1.0f);
      *value_ = denormalizeBipolarLog(norm);
    }
    else if (logarithmic_) {
      float log_range = std::log(max_ / min_);
      float log_delta = wheel_delta * log_range;
      *value_ = std::clamp(*value_ * std::exp(log_delta), min_, max_);
    }
    else {
      *value_ = std::clamp(*value_ + wheel_delta * (max_ - min_), min_, max_);
    }
    if (callback_)
      callback_(*value_);
    return true;
  }

private:
  float getNormalizedValue() const {
    if (!value_)
      return 0.0f;
    if (bipolar_logarithmic_) {
      // Bipolar log: map value to 0-1 range with log scaling on each side of zero
      // For -100 to +100 range where halfway is ±10:
      // Negative side: -100 to 0 maps to 0.0 to 0.5
      // Positive side: 0 to +100 maps to 0.5 to 1.0
      float v = *value_;
      if (v < 0) {
        // Negative side: use log scale from max_ (e.g., -100) to 0
        float abs_v = -v;
        float abs_max = -min_;  // min_ is negative, so -min_ is positive max
        float log_v = std::log(abs_v + 1.0f);  // +1 to handle zero
        float log_max = std::log(abs_max + 1.0f);
        return 0.5f * (1.0f - log_v / log_max);
      }
      else {
        // Positive side: use log scale from 0 to max_
        float log_v = std::log(v + 1.0f);
        float log_max = std::log(max_ + 1.0f);
        return 0.5f + 0.5f * (log_v / log_max);
      }
    }
    if (logarithmic_) {
      float log_min = std::log(min_);
      float log_max = std::log(max_);
      float log_val = std::log(*value_);
      return (log_val - log_min) / (log_max - log_min);
    }
    return (*value_ - min_) / (max_ - min_);
  }

  float denormalizeBipolarLog(float norm) const {
    // Convert normalized 0-1 value back to bipolar logarithmic range
    if (norm < 0.5f) {
      // Negative side
      float t = 1.0f - (norm / 0.5f);  // 0 at center, 1 at min
      float abs_max = -min_;
      float log_max = std::log(abs_max + 1.0f);
      float abs_v = std::exp(t * log_max) - 1.0f;
      return std::clamp(-abs_v, min_, 0.0f);
    }
    else {
      // Positive side
      float t = (norm - 0.5f) / 0.5f;  // 0 at center, 1 at max
      float log_max = std::log(max_ + 1.0f);
      float v = std::exp(t * log_max) - 1.0f;
      return std::clamp(v, 0.0f, max_);
    }
  }

  const char* label_;
  bool logarithmic_;
  bool bipolar_logarithmic_;
  bool bidirectional_;
  float* value_ = nullptr;
  float min_ = 0.0f, max_ = 1.0f;
  visage::Color color_ { 1.0f, 0.5f, 0.8f, 0.8f };
  Callback callback_;
  bool dragging_ = false;
  float last_y_ = 0.0f;
  bool enabled_ = true;
  float* led_intensity_ = nullptr;
};

// Retro vertical slider for military aesthetic
class FilterSlider : public visage::Frame {
public:
  using Callback = std::function<void(float)>;

  FilterSlider(const char* label = "") : label_(label) { setIgnoresMouseEvents(false, false); }

  void setValue(float* v) { value_ = v; }
  void setLedIntensity(float* v) { led_intensity_ = v; }
  void setRange(float min, float max) {
    min_ = min;
    max_ = max;
  }
  void setColor(visage::Color c) { color_ = c; }
  void setCallback(Callback cb) { callback_ = cb; }
  void setEnabled(bool e) { enabled_ = e; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float label_h = 12.0f;
    const float track_h = h - label_h - 10.0f;
    const float track_w = 4.0f;
    const float tx = (w - track_w) * 0.5f;
    const float ty = 5.0f;
    const float dim = enabled_ ? 1.0f : 0.3f;

    // Track background
    canvas.setColor(visage::Color(0.6f * dim, 0.05f, 0.05f, 0.05f));
    canvas.roundedRectangle(tx, ty, track_w, track_h, track_w * 0.5f);

    // Tick marks
    canvas.setColor(visage::Color(0.4f * dim, 0.3f, 0.3f, 0.3f));
    for (int i = 0; i <= 5; ++i) {
      float t = static_cast<float>(i) / 5.0f;
      float y = ty + (1.0f - t) * track_h;
      canvas.fill(tx - 6, y - 0.5f, 4, 1);
      canvas.fill(tx + track_w + 2, y - 0.5f, 4, 1);
    }

    if (value_) {
      float norm = (*value_ - min_) / (max_ - min_);
      float thumb_h = 16.0f;
      float thumb_w = w * 0.7f;
      float thumb_y = ty + (1.0f - norm) * track_h - thumb_h * 0.5f;
      float thumb_x = (w - thumb_w) * 0.5f;

      // Thumb Shadow
      canvas.setColor(visage::Color(0.4f, 0.0f, 0.0f, 0.0f));
      canvas.roundedRectangle(thumb_x + 2, thumb_y + 2, thumb_w, thumb_h, 2);

      // Thumb
      canvas.setColor(visage::Color(1.0f * dim, 0.15f, 0.15f, 0.15f));
      canvas.roundedRectangle(thumb_x, thumb_y, thumb_w, thumb_h, 2);

      // Center line on thumb
      canvas.setColor(visage::Color(color_.alpha() * dim, color_.red(), color_.green(), color_.blue()));
      canvas.fill(thumb_x + 2, thumb_y + thumb_h * 0.5f - 1, thumb_w - 4, 2);

      // LED Indicator on the slider thumb
      if (led_intensity_) {
        float intensity = *led_intensity_;
        float lx = w * 0.5f;
        float ly = thumb_y + thumb_h * 0.5f;
        float led_r = 2.5f;

        // Base LED
        canvas.setColor(visage::Color(1.0f * dim, 0.8f, 0.3f, 0.3f));
        canvas.circle(lx - led_r, ly - led_r, led_r * 2);

        // Bloom core
        if (intensity > 0.05f) {
          float hdr = 1.0f + intensity * 6.0f;
          canvas.setColor(visage::Color(1.0f * dim, 1.0f, 0.9f, 0.8f, hdr));
          canvas.circle(lx - led_r * 0.6f, ly - led_r * 0.6f, led_r * 1.2f);
        }
      }
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
    if (!enabled_)
      return;
    dragging_ = true;
    updateFromMouse(event.position.y);
  }

  void mouseDrag(const visage::MouseEvent& event) override {
    if (!enabled_ || !dragging_)
      return;
    updateFromMouse(event.position.y);
  }

  void mouseUp(const visage::MouseEvent&) override { dragging_ = false; }

  bool mouseWheel(const visage::MouseEvent& event) override {
    if (!enabled_ || !value_)
      return false;
    float wheel_delta = event.wheel_delta_y * 0.05f;
    *value_ = std::clamp(*value_ + wheel_delta * (max_ - min_), min_, max_);
    if (callback_)
      callback_(*value_);
    return true;
  }

private:
  void updateFromMouse(float mouse_y) {
    if (!value_)
      return;
    const float label_h = 12.0f;
    const float track_h = height() - label_h - 10.0f;
    const float ty = 5.0f;

    float norm = 1.0f - (mouse_y - ty) / track_h;
    *value_ = std::clamp(min_ + norm * (max_ - min_), min_, max_);
    if (callback_)
      callback_(*value_);
  }

  const char* label_;
  float* value_ = nullptr;
  float* led_intensity_ = nullptr;
  float min_ = 0.0f, max_ = 1.0f;
  visage::Color color_ { 1.0f, 0.5f, 0.8f, 0.8f };
  Callback callback_;
  bool dragging_ = false;
  bool enabled_ = true;
};

// Numeric display with digital font
class NumericDisplay : public visage::Frame {
public:
  NumericDisplay(const char* suffix = "") : suffix_(suffix) { }

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
      }
      else if (decimals_ == 1) {
        snprintf(buf, sizeof(buf), "%.1f%s", *value_, suffix_);
      }
      else if (decimals_ == 2) {
        snprintf(buf, sizeof(buf), "%.2f%s", *value_, suffix_);
      }
      else {
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
  visage::Color color_ { 1.0f, 0.4f, 1.0f, 0.8f };
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
    if (!enabled_)
      return;
    dragging_ = true;
    updatePosition(static_cast<int>(event.position.x), static_cast<int>(event.position.y));
  }

  void mouseDrag(const visage::MouseEvent& event) override {
    if (!enabled_)
      return;
    if (dragging_)
      updatePosition(static_cast<int>(event.position.x), static_cast<int>(event.position.y));
  }

  void mouseUp(const visage::MouseEvent& event) override { dragging_ = false; }

  bool mouseWheel(const visage::MouseEvent& event) override {
    if (!enabled_)
      return false;
    if (event.isShiftDown()) {
      if (resonance_) {
        *resonance_ = std::clamp(*resonance_ + event.wheel_delta_y * 0.02f, 0.0f, 1.0f);
      }
    }
    else {
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
