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

#include "embedded/example_fonts.h"

#include <visage_app/application_editor.h>
#include <visage_windowing/windowing.h>

class ExampleEditor : public visage::WindowedEditor {
public:
  ExampleEditor() {
    blur_bloom_.setBlurAmount(1.0f);
    blur_bloom_.setBlurSize(40.0f);
    blur_bloom_.setBloomSize(40.0f);
    blur_bloom_.setBloomIntensity(10.0f);
    setPostEffect(&blur_bloom_);
  }

  void draw(visage::Canvas& canvas) override {
    static constexpr int kBuffer = 5;
    static constexpr int kRows = 15;
    static constexpr float kRadiusRatio = 0.2f;
    static constexpr float kBlurSpeed = 8.0f;

    canvas.setColor(0xff222026);
    canvas.fill(0, 0, width(), height());

    float blur_delta = canvas.deltaTime() * kBlurSpeed;
    if (!mouse_down_)
      blur_delta = -blur_delta;
    blur_amount_ = std::min(1.0f, std::max(0.0f, blur_amount_ + blur_delta));
    blur_bloom_.setBlurAmount(blur_amount_);

    float x_delta = width() / (kBuffer * 2.0f + kRows);
    float y_delta = height() / (kBuffer * 2.0f + kRows);
    float radius = std::min(x_delta, y_delta) * kRadiusRatio;
    float start_x = kBuffer * x_delta - radius;
    float start_y = kBuffer * y_delta - radius;

    visage::Color color(0xffff8855);
    for (int i = 0; i < kRows; ++i) {
      for (int j = 0; j < kRows; ++j) {
        color.setHdr(sinf(0.4f * i + 0.2f * j - 3.0f * canvas.time()) + 1.5f);
        canvas.setColor(color);
        canvas.circle(i * x_delta + start_x, j * y_delta + start_y, 2.0f * radius);
      }
    }

    canvas.setColor(0xffffffff);
    int text_area_height = kBuffer * y_delta;
    canvas.text("Click to blur", font_, visage::Font::kCenter, 0, height() - text_area_height,
                width(), text_area_height);
    redraw();
  };

  void resized() override {
    font_ = visage::Font(18.0f * heightScale(), resources::fonts::Lato_Regular_ttf);
  }

  void mouseDown(const visage::MouseEvent& e) override { mouse_down_ = true; }
  void mouseUp(const visage::MouseEvent& e) override { mouse_down_ = false; }

private:
  visage::Font font_;
  bool mouse_down_ = false;
  float blur_amount_ = 0.0f;
  visage::BlurBloomPostEffect blur_bloom_;
};

int runExample() {
  ExampleEditor editor;
  editor.show(visage::Dimension::widthPercent(40.0f), visage::Dimension::widthPercent(30.0f));
  editor.runEventLoop();

  return 0;
}
