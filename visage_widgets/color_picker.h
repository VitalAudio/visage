/* Copyright Vital Audio, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include "text_editor.h"
#include "visage_graphics/color.h"
#include "visage_ui/frame.h"

namespace visage {
  class HueEditor : public Frame {
  public:
    HueEditor() = default;

    void draw(Canvas& canvas) override;

    auto& onEdit() { return on_edit_; }

    void setHue(const MouseEvent& e) {
      static constexpr float kHueRange = Color::kHueRange;

      hue_ = std::max(0.0f, std::min(kHueRange, kHueRange * e.position.y / height()));
      on_edit_.callback(hue_);
      redraw();
    }

    void mouseDown(const MouseEvent& e) override { setHue(e); }
    void mouseDrag(const MouseEvent& e) override { setHue(e); }

    void setHue(float hue) {
      hue_ = hue;
      redraw();
    }
    float hue() const { return hue_; }

  private:
    float hue_ = 0.0f;
    CallbackList<void(float)> on_edit_;

    VISAGE_LEAK_CHECKER(HueEditor)
  };

  class ValueSaturationEditor : public Frame {
  public:
    ValueSaturationEditor() = default;

    void draw(Canvas& canvas) override;

    void setValueSaturation(const MouseEvent& e) {
      value_ = std::max(0.0f, std::min(1.0f, 1.0f - e.position.y * 1.0f / height()));
      saturation_ = std::max(0.0f, std::min(1.0f, e.position.x * 1.0f / width()));
      on_edit_.callback(value_, saturation_);

      redraw();
    }

    void mouseDown(const MouseEvent& e) override { setValueSaturation(e); }
    void mouseDrag(const MouseEvent& e) override { setValueSaturation(e); }

    void setValue(float value) {
      value_ = value;
      redraw();
    }

    float value() const { return value_; }

    void setSaturation(float saturation) {
      saturation_ = saturation;
      redraw();
    }
    float saturation() const { return saturation_; }

    void setHueColor(const Color& hue_color) {
      hue_color_ = hue_color;
      redraw();
    }

    auto& onEdit() { return on_edit_; }

  private:
    float value_ = 1.0f;
    float saturation_ = 1.0f;
    Color hue_color_ = Color(0xffff00ff);
    CallbackList<void(float, float)> on_edit_;

    VISAGE_LEAK_CHECKER(ValueSaturationEditor)
  };

  class ColorPicker : public Frame {
  public:
    static constexpr float kHueWidth = 24.0f;
    static constexpr float kPadding = 8.0f;
    static constexpr float kEditHeight = 40.0f;
    static constexpr int kDecimalSigFigs = 5;

    ColorPicker();
    ~ColorPicker() override = default;

    auto& onColorChange() { return on_color_change_; }

    void resized() override;
    void draw(Canvas& canvas) override;

    void updateColor();
    void setColor(const Color& color);
    void notifyNewColor() { on_color_change_.callback(color_); }

  private:
    CallbackList<void(const Color&)> on_color_change_;
    Color color_ = Color(0);
    HueEditor hue_;
    ValueSaturationEditor value_saturation_;
    TextEditor hex_text_;
    TextEditor alpha_text_;
    TextEditor hdr_text_;
    float alpha_ = 1.0f;
    float hdr_ = 1.0f;

    VISAGE_LEAK_CHECKER(ColorPicker)
  };
}