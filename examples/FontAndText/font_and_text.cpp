/* Copyright Vital Audio, LLC
 *
 * va is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * va is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with va.  If not, see <http://www.gnu.org/licenses/>.
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
  editor.showWithEventLoop(0.5f);

  return 0;
}
