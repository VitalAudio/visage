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

#pragma once

#include "graphics_utils.h"
#include "visage_file_embed/embedded_file.h"

#include <memory>

namespace visage {
  struct DrawableState {
    Matrix transform;
    float opacity = 1.0f;

    Brush fill_brush = Brush::solid(0xff000000);
    float fill_opacity = 1.0f;

    Brush stroke_brush;
    float stroke_opacity = 1.0f;
    float stroke_width = 1.0f;
    Path::JoinType stroke_join = Path::JoinType::Miter;

    bool visible = true;
  };

  struct SvgDrawable {
    void draw(Canvas& canvas, float x, float y, float width, float height) const;
    void fill(Canvas& canvas, float x, float y, float width, float height) const;
    void stroke(Canvas& canvas, float x, float y, float width, float height) const;

    Path path;
    DrawableState state;
  };

  class Svg {
  public:
    struct ViewSettings {
      float width = 0.0f;
      float height = 0.0f;
      float view_box_x = 0.0f;
      float view_box_y = 0.0f;
      float view_box_width = 0.0f;
      float view_box_height = 0.0f;
      std::string align;
      std::string scale;
    };

    Svg() = default;

    Svg& operator=(const Svg& other) {
      drawables_.clear();
      drawables_.reserve(other.drawables_.size());
      view_ = other.view_;
      for (const auto& drawable : other.drawables_)
        drawables_.emplace_back(std::make_unique<SvgDrawable>(*drawable));
      return *this;
    }

    Svg(const Svg& other) { *this = other; }

    Svg(const unsigned char* data, int data_size) { parseData(data, data_size); }
    Svg(const EmbeddedFile& file) : Svg(file.data, file.size) { }

    void draw(Canvas& canvas, float x, float y, float width = 0.0f, float height = 0.0f) const {
      for (const auto& drawable : drawables_)
        drawable->draw(canvas, x, y, width ? width : view_.width, height ? height : view_.height);
    }

    void setDimensions(int width, int height) {
      this->width = width;
      this->height = height;
      view_.width = width;
      view_.height = height;
    }

    int width = 0;
    int height = 0;
    int blur_radius = 0;

  private:
    void parseData(const unsigned char* data, int data_size);

    std::vector<std::unique_ptr<SvgDrawable>> drawables_;
    ViewSettings view_;
  };
}