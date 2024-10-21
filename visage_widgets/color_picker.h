/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "text_editor.h"
#include "visage_graphics/color.h"
#include "visage_ui/frame.h"

namespace visage {
  class HueEditor : public Frame {
  public:
    HueEditor() = default;

    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void hueChanged(float hue) = 0;
    };

    void draw(Canvas& canvas) override;

    void setHue(const MouseEvent& e) {
      static constexpr float kHueRange = Color::kHueRange;

      hue_ = std::max(0.0f, std::min(kHueRange, kHueRange * e.position.y / height()));
      for (Listener* listener : listeners_)
        listener->hueChanged(hue_);

      redraw();
    }

    void mouseDown(const MouseEvent& e) override { setHue(e); }
    void mouseDrag(const MouseEvent& e) override { setHue(e); }

    void setHue(float hue) {
      hue_ = hue;
      redraw();
    }
    float hue() const { return hue_; }

    void addListener(Listener* listener) { listeners_.push_back(listener); }

  private:
    float hue_ = 0.0f;
    std::vector<Listener*> listeners_;

    VISAGE_LEAK_CHECKER(HueEditor)
  };

  class ValueSaturationEditor : public Frame {
  public:
    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void valueSaturationChanged(float value, float saturation) = 0;
    };

    ValueSaturationEditor() = default;

    void draw(Canvas& canvas) override;

    void setValueSaturation(const MouseEvent& e) {
      value_ = std::max(0.0f, std::min(1.0f, 1.0f - e.position.y * 1.0f / height()));
      saturation_ = std::max(0.0f, std::min(1.0f, e.position.x * 1.0f / width()));
      for (Listener* listener : listeners_)
        listener->valueSaturationChanged(value_, saturation_);

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

    void addListener(Listener* listener) { listeners_.push_back(listener); }

  private:
    float value_ = 1.0f;
    float saturation_ = 1.0f;
    Color hue_color_ = Color(0xffff00ff);
    std::vector<Listener*> listeners_;

    VISAGE_LEAK_CHECKER(ValueSaturationEditor)
  };

  class ColorPicker : public Frame,
                      public TextEditor::Listener,
                      public HueEditor::Listener,
                      public ValueSaturationEditor::Listener {
  public:
    static constexpr float kHueWidth = 24.0f;
    static constexpr float kPadding = 8.0f;
    static constexpr float kEditHeight = 40.0f;
    static constexpr int kDecimalSigFigs = 5;

    class Listener {
    public:
      virtual ~Listener() = default;
      virtual void colorChanged(ColorPicker* picker, Color color) = 0;
    };

    ColorPicker() {
      hue_.addListener(this);
      value_saturation_.addListener(this);
      hex_text_.addListener(this);
      hex_text_.setNumberEntry();
      hex_text_.setMargin(5, 0);
      hex_text_.setMaxCharacters(6);

      alpha_text_.addListener(this);
      alpha_text_.setNumberEntry();
      alpha_text_.setMargin(5, 0);
      alpha_text_.setMaxCharacters(kDecimalSigFigs + 1);

      hdr_text_.addListener(this);
      hdr_text_.setNumberEntry();
      hdr_text_.setMargin(5, 0);
      hdr_text_.setMaxCharacters(kDecimalSigFigs + 1);

      addChild(&hue_);
      addChild(&value_saturation_);
      addChild(&hex_text_);
      addChild(&alpha_text_);
      addChild(&hdr_text_);
      value_saturation_.setHueColor(Color::fromAHSV(1.0f, hue_.hue(), 1.0f, 1.0f));
      updateColor();
    }

    ~ColorPicker() override = default;

    void resized() override;
    void draw(Canvas& canvas) override;

    void textEditorEnterPressed(TextEditor* editor) override;
    void textEditorChanged(TextEditor* editor) override;

    void hueChanged(float hue) override {
      updateColor();
      notifyNewColor();
      value_saturation_.setHueColor(Color::fromAHSV(1.0f, hue, 1.0f, 1.0f));
      redraw();
    }

    void valueSaturationChanged(float value, float saturation) override {
      updateColor();
      notifyNewColor();
      redraw();
    }

    void addListener(Listener* listener) { listeners_.push_back(listener); }
    void updateColor();
    void setColor(const Color& color);
    void notifyNewColor() {
      for (Listener* listener : listeners_)
        listener->colorChanged(this, color_);
    }

  private:
    std::vector<Listener*> listeners_;
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