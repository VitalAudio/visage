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

class ExampleEditor : public visage::ApplicationWindow {
public:
  ExampleEditor() {
    addChild(&additive_frame_);
    addChild(&subtractive_frame_);
    masked_frame_.setMasked(true);
    addChild(&masked_frame_);
    transparent_frame_.setAlphaTransparency(0.5f);
    addChild(&transparent_frame_);

    additive_frame_.onDraw() = [this](visage::Canvas& canvas) {
      canvas.setBlendMode(visage::BlendMode::Add);
      canvas.setColor(0xffffffff);
      canvas.text("Additive", font_, visage::Font::kCenter, 0, 0, additive_frame_.width(),
                  0.2f * additive_frame_.height());
      drawRgbCircles(canvas, additive_frame_.width(), additive_frame_.height());
    };

    subtractive_frame_.onDraw() = [this](visage::Canvas& canvas) {
      canvas.setColor(0xffeeeeee);
      canvas.fill(0, 0, subtractive_frame_.width(), subtractive_frame_.height());

      canvas.setBlendMode(visage::BlendMode::Sub);
      canvas.setColor(0xffffffff);
      canvas.text("Subtractive", font_, visage::Font::kCenter, 0, 0, subtractive_frame_.width(),
                  0.2f * subtractive_frame_.height());
      drawRgbCircles(canvas, subtractive_frame_.width(), subtractive_frame_.height());
    };

    masked_frame_.onDraw() = [this](visage::Canvas& canvas) {
      static constexpr int kColumns = 12;
      int w = masked_frame_.width();
      int h = masked_frame_.height();
      int last_x = 0;
      int color1 = 0xffff00ff;
      int color2 = 0xffffffff;
      for (int i = 0; i < kColumns; ++i) {
        int x = w * (i + 1) / kColumns;
        canvas.setColor(color1);
        std::swap(color1, color2);
        canvas.fill(last_x, 0, x - last_x, h);
        last_x = x;
      }

      canvas.setBlendMode(visage::BlendMode::MaskRemove);
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, w, h);

      canvas.setBlendMode(visage::BlendMode::MaskAdd);
      canvas.setColor(0xffffffff);
      drawRgbCircles(canvas, w, h);

      canvas.setBlendMode(visage::BlendMode::Alpha);
      canvas.setColor(0xffffffff);
      canvas.text("Masked", font_, visage::Font::kCenter, 0, 0, w, 0.2f * masked_frame_.height());
    };

    transparent_frame_.onDraw() = [this](visage::Canvas& canvas) {
      canvas.setColor(0xffffffff);
      canvas.text("Grouped Transparency", font_, visage::Font::kCenter, 0, 0,
                  transparent_frame_.width(), 0.2f * transparent_frame_.height());
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
      int x = transparent_frame_.x() + w * (i + 1) / kColumns;
      canvas.setColor(color1);
      std::swap(color1, color2);
      canvas.fill(last_x, y, x - last_x, h);
      last_x = x;
    }
  }

  void resized() override {
    int center_x = width() / 2;
    int center_y = height() / 2;

    additive_frame_.setBounds(0, 0, center_x, center_y);
    subtractive_frame_.setBounds(center_x, 0, width() - center_x, center_y);
    masked_frame_.setBounds(0, center_y, center_x, height() - center_y);
    transparent_frame_.setBounds(center_x, center_y, width() - center_x, height() - center_y);

    font_ = visage::Font(16, resources::fonts::Lato_Regular_ttf);
  }

private:
  Frame additive_frame_;
  Frame subtractive_frame_;
  Frame masked_frame_;
  Frame transparent_frame_;
  visage::Font font_;
};

int runExample() {
  ExampleEditor editor;
  if (visage::isMobileDevice())
    editor.showMaximized();
  else
    editor.show(visage::Dimension::logicalPixels(800), visage::Dimension::logicalPixels(600));
  editor.runEventLoop();

  return 0;
}
