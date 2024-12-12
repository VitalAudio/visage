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

#include "embedded/example_shaders.h"

#include <complex>
#include <visage_app/application_editor.h>
#include <visage_widgets/shader_editor.h>
#include <visage_windowing/windowing.h>

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

class ExampleEditor : public visage::WindowedEditor {
public:
  ExampleEditor() {
    shapes_.onDraw() = [this](visage::Canvas& canvas) {
      canvas.setColor(0xff000000);
      canvas.fill(0, 0, shapes_.width(), shapes_.height());

      drawRotatingCircles(canvas, shapes_.width(), shapes_.height());
      shapes_.redraw();
    };

    post_effect_ = std::make_unique<visage::ShaderPostEffect>(resources::shaders::vs_warp,
                                                              resources::shaders::fs_warp);

    shader_editor_.setShader(resources::shaders::fs_warp, resources::shaders::fs_warp_sc);

    shapes_.setPostEffect(post_effect_.get());
    addChild(&shapes_);
    addChild(&shader_editor_);
  }

  void resized() override {
    int center = width() / 2;
    int shapes_width = std::min(center, height());
    shapes_.setBounds((center - shapes_width) / 2, (height() - shapes_width) / 2, shapes_width, shapes_width);
    shader_editor_.setBounds(center, 0, width() - center, height());
  }

  int defaultWidth() const override { return 1000; }
  int defaultHeight() const override { return 500; }

private:
  Frame shapes_;
  std::unique_ptr<visage::ShaderPostEffect> post_effect_;
  visage::ShaderEditor shader_editor_;
};

int runExample() {
  ExampleEditor editor;
  editor.showWithEventLoop(0.6f);

  return 0;
}
