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

#include <visage/app.h>
#include <visage_widgets/button.h>

using namespace visage::dimension;

class SubWindow : public visage::ApplicationWindow {
public:
  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff222026);
    canvas.fill(0, 0, width(), height());

    canvas.setColor(0xffaa88ff);
    int radius = std::min(width(), height()) / 4;
    canvas.circle(width() / 2 - radius, height() / 2 - radius, 2 * radius);
  }
};

class ExampleEditor : public visage::ApplicationWindow {
public:
  static constexpr int kNumFrames = 10;

  ExampleEditor() {
    setFlexLayout(true);
    layout().setPadding(50_px);

    addChild(&button_);
    button_.setText("Open Window");
    button_.layout().setHeight(100_vh);
    button_.layout().setWidth(100_vw);
    button_.setFont(visage::Font(24, resources::fonts::Lato_Regular_ttf));

    sub_window_.onShow() += [this] { button_.setText("Close Window"); };
    sub_window_.onHide() += [this] { button_.setText("Open Window"); };

    button_.onToggle() += [this](visage::Button* button, bool is_toggled) {
      if (sub_window_.isShowing())
        sub_window_.hide();
      else
        sub_window_.show(10_vw, 10_vh, 400_px, 300_px);
    };

    onHide() += [this] {
      if (sub_window_.isShowing())
        sub_window_.hide();
    };
  }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff222026);
    canvas.fill(0, 0, width(), height());
  }

private:
  visage::UiButton button_;
  visage::Font font_ {};
  SubWindow sub_window_;
};

int runExample() {
  ExampleEditor editor;
  editor.show(500_px, 200_px);
  editor.runEventLoop();

  return 0;
}
