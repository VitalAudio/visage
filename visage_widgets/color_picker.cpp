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

#include "color_picker.h"

#include "embedded/fonts.h"
#include "visage_utils/string_utils.h"

namespace visage {
  void HueEditor::draw(Canvas& canvas) {
    int width = getWidth();
    int height = getHeight();

    int yg_y = height / 6;
    int gc_y = height / 3;
    int cb_y = height / 2;
    int bp_y = 2 * height / 3;
    int pr_y = 5 * height / 6;

    canvas.setColor(VerticalGradient(0xffff0000, 0xffffff00));
    canvas.rectangle(0, 0, width, yg_y);

    canvas.setColor(VerticalGradient(0xffffff00, 0xff00ff00));
    canvas.rectangle(0, yg_y, width, gc_y - yg_y);

    canvas.setColor(VerticalGradient(0xff00ff00, 0xff00ffff));
    canvas.rectangle(0, gc_y, width, cb_y - gc_y);

    canvas.setColor(VerticalGradient(0xff00ffff, 0xff0000ff));
    canvas.rectangle(0, cb_y, width, bp_y - cb_y);

    canvas.setColor(VerticalGradient(0xff0000ff, 0xffff00ff));
    canvas.rectangle(0, bp_y, width, pr_y - bp_y);

    canvas.setColor(VerticalGradient(0xffff00ff, 0xffff0000));
    canvas.rectangle(0, pr_y, width, height - pr_y);

    float y = getHeight() * hue_ / Color::kHueRange;
    canvas.setColor(0xff000000);
    canvas.rectangle(0, y - 1.0f, width, 2.0f);
  }

  void ValueSaturationEditor::draw(Canvas& canvas) {
    canvas.setColor(HorizontalGradient(0xffffffff, hue_color_));
    canvas.rectangle(0, 0, getWidth(), getHeight());

    canvas.setColor(VerticalGradient(0x00000000, 0xff000000));
    canvas.rectangle(0, 0, getWidth(), getHeight());

    float x = getWidth() * saturation_;
    float y = getHeight() * (1.0f - value_);

    canvas.setColor(0xff000000);
    canvas.ring(x - 3.0f, y - 3.0f, 6.0f, 1.0f);
  }

  void ColorPicker::resized() {
    DrawableComponent::resized();
    int width = getWidth();
    int widget_height = getHeight() - kEditHeight - kPadding;
    hue_.setBounds(width - kHueWidth, 0, kHueWidth, widget_height);
    value_saturation_.setBounds(0, 0, width - kHueWidth - kPadding, widget_height);

    hex_text_.setNumberEntry();
    hex_text_.setFont(Font(kEditHeight / 2, fonts::DroidSansMono_ttf));
    hex_text_.setBackgroundRounding(8);

    alpha_text_.setNumberEntry();
    alpha_text_.setFont(Font(kEditHeight / 2, fonts::DroidSansMono_ttf));
    alpha_text_.setBackgroundRounding(8);

    hdr_text_.setNumberEntry();
    hdr_text_.setFont(Font(kEditHeight / 2, fonts::DroidSansMono_ttf));
    hdr_text_.setBackgroundRounding(8);

    int edit_width = (getWidth() - kEditHeight - 4 * kPadding) / 3;
    hex_text_.setBounds(kEditHeight + kPadding, widget_height + kPadding, edit_width, kEditHeight);
    alpha_text_.setBounds(hex_text_.getRight() + kPadding, widget_height + kPadding, edit_width, kEditHeight);
    hdr_text_.setBounds(alpha_text_.getRight() + kPadding, widget_height + kPadding, edit_width, kEditHeight);
  }

  void ColorPicker::draw(Canvas& canvas) {
    int widget_height = getHeight() - kEditHeight;

    canvas.setColor(color_);
    canvas.roundedRectangle(0, widget_height, kEditHeight, kEditHeight, 8);
  }

  void ColorPicker::textEditorEnterPressed(TextEditor* editor) {
    requestKeyboardFocus();
  }

  void ColorPicker::textEditorChanged(TextEditor* editor) {
    if (editor == &hex_text_) {
      color_ = Color::fromHexString(editor->getText().toUtf8());
      hue_.setHue(color_.hue());
      value_saturation_.setValue(color_.value());
      value_saturation_.setSaturation(color_.saturation());
      value_saturation_.setHueColor(Color::fromAHSV(1.0f, hue_.hue(), 1.0f, 1.0f));
    }
    else if (editor == &alpha_text_) {
      alpha_ = std::min(1.0f, std::max(0.0f, String(editor->getText()).toFloat()));
      color_.setAlpha(alpha_);
    }
    else if (editor == &hdr_text_) {
      hdr_ = std::max(0.0f, String(editor->getText()).toFloat());
      color_.setHdr(hdr_);
    }

    notifyNewColor();
    redraw();
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