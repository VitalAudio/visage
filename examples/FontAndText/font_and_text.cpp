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

#include "embedded/example_fonts.h"

#include <iomanip>
#include <sstream>
#include <visage_app/application_editor.h>
#include <visage_windowing/windowing.h>

class ExampleEditor : public visage::WindowedEditor {
public:
  static constexpr float kFontHeightRatio = 0.05f;

  ExampleEditor() {
    addChild(&increment_);
    increment_.onDraw() = [this](visage::Canvas& canvas) {
      incrment_value_ += 0.01;
      canvas.setColor(0xffffffff);
      std::ostringstream format;
      format << "Monospace: " << std::fixed << std::setprecision(2) << incrment_value_;
      canvas.text(format.str(), increment_font_, visage::Font::kLeft, 0, 0, increment_.width(),
                  increment_.height());
      increment_.redraw();
    };
  }

  void draw(visage::Canvas& canvas) override {
    int font_size = kFontHeightRatio * height();
    visage::Font font(font_size, resources::fonts::Lato_Regular_ttf);
    canvas.setColor(0xff2a2a33);
    canvas.fill(0, 0, width(), height());

    canvas.setColor(0xff222222);
    canvas.rectangle(300, 300, 500, 100);

    canvas.setColor(0xffffffff);
    canvas.text("Hello, world!", font, visage::Font::kCenter, 300, 300, 500, 100, visage::Direction::Up);

    visage::Font emoji(font_size, resources::fonts::NotoEmoji_Medium_ttf);
    canvas.text(U"\U0001F525", emoji, visage::Font::kLeft, 0.0f, 0.0f, width(), height());
  }

  void resized() override {
    int font_size = kFontHeightRatio * height();

    increment_.setBounds(0, 0, width() - font_size, 2 * font_size);
    increment_font_ = visage::Font(font_size, resources::fonts::DroidSansMono_ttf);
  }

private:
  double incrment_value_ = 0.0;
  visage::Font increment_font_;
  visage::Frame increment_;
};

int runExample() {
  ExampleEditor editor;
  editor.show(visage::Dimension::widthPercent(70.0f), visage::Dimension::heightPercent(70.0f));

  return 0;
}
