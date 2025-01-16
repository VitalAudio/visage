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

static void drawRgbCircles(visage::Canvas& canvas, float width, float height) {
  static constexpr float kCircleRadiusRatio = 0.2f;
  static constexpr float kVennRadiusRatio = 0.13f;
  static constexpr float k60DegreeTriangleRatio = 0.866025403784f;

  float min_dimension = std::min(width, height);

  float circle_radius = kCircleRadiusRatio * min_dimension;
  float venn_radius = kVennRadiusRatio * min_dimension;
  float venn_offset = k60DegreeTriangleRatio * venn_radius;
  float start_x = width / 2.0f - circle_radius;
  float start_y = height / 2.0f - circle_radius;

  canvas.setColor(0xffff0000);
  canvas.circle(start_x, start_y + venn_radius, 2 * circle_radius);
  canvas.setColor(0xff00ff00);
  canvas.circle(start_x - venn_offset, start_y - venn_radius * 0.5f, 2 * circle_radius);
  canvas.setColor(0xff0000ff);
  canvas.circle(start_x + venn_offset, start_y - venn_radius * 0.5f, 2 * circle_radius);
}

class ExampleEditor : public visage::WindowedEditor {
public:
  ExampleEditor() {
    addChild(&additive_frame_);
    addChild(&subtractive_frame_);
    stenciled_frame_.setStenciled(true);
    addChild(&stenciled_frame_);
    transparent_frame_.setAlphaTransparency(0.5f);
    addChild(&transparent_frame_);

    additive_frame_.onDraw() = [this](visage::Canvas& canvas) {
      canvas.setBlendMode(visage::BlendMode::Add);
      canvas.setColor(0xffffffff);
      canvas.text("Additive", font_, visage::Font::kCenter, 0, 0, additive_frame_.width(),
                  font_.size() * 3);
      drawRgbCircles(canvas, additive_frame_.width(), additive_frame_.height());
    };

    subtractive_frame_.onDraw() = [this](visage::Canvas& canvas) {
      canvas.setColor(0xffeeeeee);
      canvas.fill(0, 0, subtractive_frame_.width(), subtractive_frame_.height());

      canvas.setBlendMode(visage::BlendMode::Sub);
      canvas.setColor(0xffffffff);
      canvas.text("Subtractive", font_, visage::Font::kCenter, 0, 0, subtractive_frame_.width(),
                  font_.size() * 3);
      drawRgbCircles(canvas, subtractive_frame_.width(), subtractive_frame_.height());
    };

    stenciled_frame_.onDraw() = [this](visage::Canvas& canvas) {
      static constexpr int kColumns = 12;
      int w = stenciled_frame_.width();
      int h = stenciled_frame_.height();
      int last_x = 0;
      int color1 = 0xffff00ff;
      int color2 = 0xffffffff;
      for (int i = 0; i < kColumns; ++i) {
        int x = (w * (i + 1)) / kColumns;
        canvas.setColor(color1);
        std::swap(color1, color2);
        canvas.fill(last_x, 0, x - last_x, h);
        last_x = x;
      }

      canvas.setBlendMode(visage::BlendMode::StencilRemove);
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, w, h);

      canvas.setBlendMode(visage::BlendMode::StencilAdd);
      canvas.setColor(0xffffffff);
      drawRgbCircles(canvas, w, h);

      canvas.setBlendMode(visage::BlendMode::Alpha);
      canvas.setColor(0xffffffff);
      canvas.text("Stencil", font_, visage::Font::kCenter, 0, 0, w, font_.size() * 3);
    };

    transparent_frame_.onDraw() = [this](visage::Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.text("Grouped Transparency", font_, visage::Font::kCenter, 0, 0,
                  transparent_frame_.width(), font_.size() * 3);
      drawRgbCircles(canvas, transparent_frame_.width(), transparent_frame_.height());
    };
  }

  void draw(visage::Canvas& canvas) override {
    static constexpr int kColumns = 12;

    canvas.setColor(0xff222026);
    canvas.fill(0, 0, width(), height());

    int w = transparent_frame_.width();
    int h = transparent_frame_.height();
    int last_x = transparent_frame_.x();
    int y = transparent_frame_.y();
    int color1 = 0xff666666;
    int color2 = 0xff888888;
    for (int i = 0; i < kColumns; ++i) {
      int x = transparent_frame_.x() + (w * (i + 1)) / kColumns;
      canvas.setColor(color1);
      std::swap(color1, color2);
      canvas.fill(last_x, y, x - last_x, h);
      last_x = x;
    }
  };

  void resized() override {
    int center_x = width() / 2;
    int center_y = height() / 2;

    additive_frame_.setBounds(0, 0, center_x, center_y);
    subtractive_frame_.setBounds(center_x, 0, width() - center_x, center_y);
    stenciled_frame_.setBounds(0, center_y, center_x, height() - center_y);
    transparent_frame_.setBounds(center_x, center_y, width() - center_x, height() - center_y);

    font_ = visage::Font(16.0f * dpiScale(), resources::fonts::Lato_Regular_ttf);
  }

private:
  Frame additive_frame_;
  Frame subtractive_frame_;
  Frame stenciled_frame_;
  Frame transparent_frame_;
  visage::Font font_;
};

int runExample() {
  ExampleEditor editor;
  editor.show(visage::Dimension::logicalPixels(800), visage::Dimension::logicalPixels(600));
  editor.runEventLoop();

  return 0;
}
