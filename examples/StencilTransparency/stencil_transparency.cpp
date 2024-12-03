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

#include <visage_app/application_editor.h>
#include <visage_windowing/windowing.h>

class StenciledFrame : public visage::Frame {
public:
  StenciledFrame() { setStenciled(true); }

  void draw(visage::Canvas& canvas) override {
    static constexpr int kBuffer = 5;
    static constexpr int kRows = 15;
    static constexpr float kRadiusRatio = 0.2f;

    float x_delta = width() / (kBuffer * 2.0f + kRows);
    float y_delta = height() / (kBuffer * 2.0f + kRows);
    float radius = std::min(x_delta, y_delta) * kRadiusRatio;
    float start_x = kBuffer * x_delta - radius;
    float start_y = kBuffer * y_delta - radius;
    canvas.setColor(0xff444444);
    canvas.fill(0, 0, width(), height());

    visage::Color color(0xffff8855);
    for (int i = 0; i < kRows; ++i) {
      for (int j = 0; j < kRows; ++j) {
        color.setHdr(sinf(0.4f * i + 0.2f * j - 3.0f * canvas.time()) + 1.5f);
        canvas.setColor(color);
        canvas.circle(i * x_delta + start_x, j * y_delta + start_y, 2.0f * radius);
      }
    }

    setAlphaTransparency(0.5f + 0.5f * sinf(canvas.time()));

    canvas.setBlendMode(visage::BlendMode::Multiply);

    canvas.setColor(0xffffffff);
    canvas.circle(0, 0, width());
    redraw();
  };

  void mouseMove(const visage::MouseEvent& e) override { }
};

class ExampleEditor : public visage::WindowedEditor {
public:
  ExampleEditor() { addChild(&stenciled_frame_); }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff222026);
    canvas.fill(0, 0, width(), height());
  };

  void resized() override { stenciled_frame_.setBounds(localBounds()); }

  void mouseMove(const visage::MouseEvent& e) override { }

private:
  StenciledFrame stenciled_frame_;
};

int runExample() {
  ExampleEditor editor;
  editor.showWithEventLoop(0.5f);

  return 0;
}
