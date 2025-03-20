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

#include "color_picker.h"

#include "embedded/fonts.h"
#include "visage_utils/string_utils.h"

namespace visage {
  void HueEditor::draw(Canvas& canvas) {
    int w = width();
    int h = height();

    int yg_y = h / 6;
    int gc_y = h / 3;
    int cb_y = h / 2;
    int bp_y = 2 * h / 3;
    int pr_y = 5 * h / 6;

    canvas.setColor(Brush::vertical(0xffff0000, 0xffffff00));
    canvas.rectangle(0, 0, w, yg_y);

    canvas.setColor(Brush::vertical(0xffffff00, 0xff00ff00));
    canvas.rectangle(0, yg_y, w, gc_y - yg_y);

    canvas.setColor(Brush::vertical(0xff00ff00, 0xff00ffff));
    canvas.rectangle(0, gc_y, w, cb_y - gc_y);

    canvas.setColor(Brush::vertical(0xff00ffff, 0xff0000ff));
    canvas.rectangle(0, cb_y, w, bp_y - cb_y);

    canvas.setColor(Brush::vertical(0xff0000ff, 0xffff00ff));
    canvas.rectangle(0, bp_y, w, pr_y - bp_y);

    canvas.setColor(Brush::vertical(0xffff00ff, 0xffff0000));
    canvas.rectangle(0, pr_y, w, h - pr_y);

    float y = h * hue_ / Color::kHueRange;
    canvas.setColor(0xff000000);
    canvas.rectangle(0, y - 1.0f, w, 2.0f);
  }

  void ValueSaturationEditor::draw(Canvas& canvas) {
    canvas.setColor(Brush::horizontal(0xffffffff, hue_color_));
    canvas.rectangle(0, 0, width(), height());

    canvas.setColor(Brush::vertical(0x00000000, 0xff000000));
    canvas.rectangle(0, 0, width(), height());

    float x = width() * saturation_;
    float y = height() * (1.0f - value_);

    canvas.setColor(0xff000000);
    canvas.ring(x - 3.0f, y - 3.0f, 6.0f, 1.0f);
  }

  ColorPicker::ColorPicker() {
    Font font(kEditHeight / 2, fonts::DroidSansMono_ttf);

    hue_.onEdit() = [this](float hue) {
      updateColor();
      notifyNewColor();
      value_saturation_.setHueColor(Color::fromAHSV(1.0f, hue, 1.0f, 1.0f));
      redraw();
    };

    value_saturation_.onEdit() = [this](float, float) {
      updateColor();
      notifyNewColor();
      redraw();
    };

    hex_text_.setFont(font);
    alpha_text_.setFont(font);
    hdr_text_.setFont(font);

    hex_text_.setNumberEntry();
    hex_text_.setMargin(5, 0);
    hex_text_.setMaxCharacters(6);

    alpha_text_.setNumberEntry();
    alpha_text_.setMargin(5, 0);
    alpha_text_.setMaxCharacters(kDecimalSigFigs + 1);

    hdr_text_.setNumberEntry();
    hdr_text_.setMargin(5, 0);
    hdr_text_.setMaxCharacters(kDecimalSigFigs + 1);

    hex_text_.onEnterKey() += [this] { requestKeyboardFocus(); };
    alpha_text_.onEnterKey() += [this] { requestKeyboardFocus(); };
    hdr_text_.onEnterKey() += [this] { requestKeyboardFocus(); };

    hex_text_.onTextChange() += [this] {
      color_ = Color::fromHexString(hex_text_.text().toUtf8());
      hue_.setHue(color_.hue());
      value_saturation_.setValue(color_.value());
      value_saturation_.setSaturation(color_.saturation());
      value_saturation_.setHueColor(Color::fromAHSV(1.0f, hue_.hue(), 1.0f, 1.0f));
      notifyNewColor();
      redraw();
    };

    alpha_text_.onTextChange() += [this] {
      alpha_ = std::min(1.0f, std::max(0.0f, alpha_text_.text().toFloat()));
      color_.setAlpha(alpha_);
      notifyNewColor();
      redraw();
    };

    hdr_text_.onTextChange() += [this] {
      hdr_ = std::max(0.0f, hdr_text_.text().toFloat());
      color_.setHdr(hdr_);
      notifyNewColor();
      redraw();
    };

    addChild(&hue_);
    addChild(&value_saturation_);
    addChild(&hex_text_);
    addChild(&alpha_text_);
    addChild(&hdr_text_);
    value_saturation_.setHueColor(Color::fromAHSV(1.0f, hue_.hue(), 1.0f, 1.0f));
    updateColor();
  }

  void ColorPicker::resized() {
    Frame::resized();
    int widget_height = height() - kEditHeight - kPadding;
    hue_.setBounds(width() - kHueWidth, 0, kHueWidth, widget_height);
    value_saturation_.setBounds(0, 0, width() - kHueWidth - kPadding, widget_height);

    hex_text_.setNumberEntry();
    hex_text_.setBackgroundRounding(8);

    alpha_text_.setNumberEntry();
    alpha_text_.setBackgroundRounding(8);

    hdr_text_.setNumberEntry();
    hdr_text_.setBackgroundRounding(8);

    int edit_width = (width() - kEditHeight - 4 * kPadding) / 3;
    hex_text_.setBounds(kEditHeight + kPadding, widget_height + kPadding, edit_width, kEditHeight);
    alpha_text_.setBounds(hex_text_.right() + kPadding, widget_height + kPadding, edit_width, kEditHeight);
    hdr_text_.setBounds(alpha_text_.right() + kPadding, widget_height + kPadding, edit_width, kEditHeight);
  }

  void ColorPicker::draw(Canvas& canvas) {
    int widget_height = height() - kEditHeight;

    canvas.setColor(color_);
    canvas.roundedRectangle(0, widget_height, kEditHeight, kEditHeight, 8);
  }

  void ColorPicker::updateColor() {
    color_ = Color::fromAHSV(alpha_, hue_.hue(), value_saturation_.saturation(),
                             value_saturation_.value());
    color_.setHdr(hdr_);
    hex_text_.setText(color_.toRGBHexString());
    alpha_text_.setText(String(alpha_, kDecimalSigFigs).toUtf8());
    hdr_text_.setText(String(hdr_, kDecimalSigFigs).toUtf8());
    redraw();
  }

  void ColorPicker::setColor(const Color& color) {
    hue_.setHue(color.hue());
    value_saturation_.setHueColor(Color::fromAHSV(1.0f, hue_.hue(), 1.0f, 1.0f));
    value_saturation_.setValue(color.value());
    value_saturation_.setSaturation(color.saturation());
    alpha_ = color.alpha();
    hdr_ = color.hdr();
    updateColor();
  }
}