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

#include "embedded/example_shaders.h"

#include <complex>
#include <visage/app.h>
#include <visage_widgets/shader_editor.h>

using namespace visage::dimension;

static void drawRing(visage::Canvas& canvas, int width, int height, float radius,
                     float circle_diameter, int num, float phase_offset) {
  static constexpr float kPi = 3.14159265358979323846f;

  float phase_inc = 2.0f * kPi / num;
  std::complex<float> tick(cosf(phase_inc), sinf(phase_inc));
  std::complex<float> position(radius * cosf(phase_offset), radius * sinf(phase_offset));
  float center_x = (width - circle_diameter) / 2.0f;
  float center_y = (height - circle_diameter) / 2.0f;

  for (int i = 0; i < num; ++i) {
    position *= tick;
    canvas.circle(center_x + position.real(), center_y + position.imag(), circle_diameter);
  }
}

static void drawRotatingCircles(visage::Canvas& canvas, int width, int height) {
  static constexpr int kIncrement = 6;
  static constexpr int kNumRings = 20;

  canvas.setColor(0xffffffff);
  float radius_increment = height * 0.5f / kNumRings;
  float circle_diameter = height * 0.4f / kNumRings;
  float phase_offset = canvas.time() * 0.03f;
  for (int i = 0; i < kNumRings; ++i) {
    drawRing(canvas, width, height, i * radius_increment, circle_diameter, i * kIncrement,
             phase_offset * (kNumRings - i));
  }
}

class ExampleEditor : public visage::ApplicationWindow {
public:
  ExampleEditor() {
    shapes_.onDraw() = [this](visage::Canvas& canvas) {
      drawRotatingCircles(canvas, shapes_.width(), shapes_.height());
      shapes_.redraw();
    };

    post_effect_ = std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_custom,
                                                              resources::shaders::fs_warp);

    shader_editor_.setShader(resources::shaders::fs_warp, resources::shaders::fs_warp_sc);

    shapes_.setPostEffect(post_effect_.get());
    addChild(&shapes_);
    addChild(&shader_editor_);
  }

  void draw(visage::Canvas& canvas) override {
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, shader_editor_.x(), height());
  }

  void resized() override {
    float center = width() / 2.0f;
    int shapes_width = std::min(center, height());
    shapes_.setBounds((center - shapes_width) / 2, (height() - shapes_width) / 2, shapes_width, shapes_width);
    shader_editor_.setBounds(center, 0, width() - center, height());
  }

private:
  Frame shapes_;
  std::unique_ptr<visage::ShaderPostEffect> post_effect_;
  visage::ShaderEditor shader_editor_;
};

int runExample() {
  ExampleEditor editor;
  editor.showMaximized();
  editor.runEventLoop();

  return 0;
}
