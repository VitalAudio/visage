#pragma once

#include <visage/app.h>
#include <cmath>
#include <algorithm>
#include "FilterMorpher.h"

// Visual Joystick Control for Filter Morphing
class FilterJoystick : public visage::Frame {
public:
  static constexpr float kPi = 3.14159265358979323846f;

  FilterJoystick() {
    setIgnoresMouseEvents(false, false);
  }

  void setMorpher(FilterMorpher* m) { morpher_ = m; }
  void setCutoff(float* c) { cutoff_ = c; }
  void setResonance(float* r) { resonance_ = r; }

  void draw(visage::Canvas& canvas) override {
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    const float cx = w * 0.5f;
    const float cy = h * 0.5f;
    const float r = std::min(w, h) * 0.45f;

    // Dark background circle
    canvas.setColor(visage::Color(0.7f, 0.06f, 0.08f, 0.12f));
    canvas.circle(cx - r, cy - r, r * 2);

    // Ring gradient for mode zones
    const int segments = 64;
    for (int i = 0; i < segments; ++i) {
      float a0 = kPi * 2.0f * i / segments;
      float a1 = kPi * 2.0f * (i + 1) / segments;
      float am = (a0 + a1) * 0.5f;

      float norm_a = am / (kPi * 2.0f);
      visage::Color color = getModeColor(norm_a);
      color = visage::Color(color.alpha() * 0.3f, color.red(), color.green(), color.blue());

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

    // Mode labels (small circles as placeholders)
    canvas.setColor(visage::Color(0.8f, 0.7f, 0.8f, 0.9f));
    float labelR = r * 0.65f;
    drawLabel(canvas, cx + labelR, cy);      // HP (right)
    drawLabel(canvas, cx, cy - labelR);      // BP (up)
    drawLabel(canvas, cx - labelR, cy);      // LP (left)
    drawLabel(canvas, cx, cy + labelR);      // BR (down)
    drawLabel(canvas, cx, cy);               // AP (center)

    // Current position indicator
    if (morpher_) {
      float px = cx + morpher_->posX() * r;
      float py = cy - morpher_->posY() * r;

      // Glow
      canvas.setColor(visage::Color(0.4f, 0.3f, 1.0f, 0.5f));
      canvas.circle(px - 12, py - 12, 24);

      // Dot
      canvas.setColor(visage::Color(1.0f, 0.4f, 1.0f, 0.9f));
      canvas.circle(px - 6, py - 6, 12);
    }

    redraw();
  }

  void mouseDown(const visage::MouseEvent& event) override {
    dragging_ = true;
    updatePosition(static_cast<int>(event.position.x), static_cast<int>(event.position.y));
  }

  void mouseDrag(const visage::MouseEvent& event) override {
    if (dragging_)
      updatePosition(static_cast<int>(event.position.x), static_cast<int>(event.position.y));
  }

  void mouseUp(const visage::MouseEvent& event) override {
    dragging_ = false;
  }

  bool mouseWheel(const visage::MouseEvent& event) override {
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
    const float cx = w * 0.5f;
    const float cy = h * 0.5f;
    const float r = std::min(w, h) * 0.45f;

    float x = (mx - cx) / r;
    float y = -(my - cy) / r;

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

  void drawLabel(visage::Canvas& canvas, float x, float y) {
    canvas.circle(x - 3, y - 3, 6);
  }

  FilterMorpher* morpher_ = nullptr;
  float* cutoff_ = nullptr;
  float* resonance_ = nullptr;
  bool dragging_ = false;
};
