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

    Brush fill_brush;
    float fill_opacity = 1.0f;

    Brush stroke_brush;
    float stroke_opacity = 1.0f;
    float stroke_width = 1.0f;

    bool visible = true;
  };

  struct SvgDrawable {
    void draw(Canvas& canvas) const;
    void fill(Canvas& canvas) const;
    void stroke(Canvas& canvas) const;

    Path path;
    DrawableState state;
  };

  class Svg {
  public:
    Svg() = default;

    Svg& operator=(const Svg& other) {
      width = other.width;
      height = other.height;
      blur_radius = other.blur_radius;
      drawables_.clear();
      drawables_.reserve(other.drawables_.size());
      for (const auto& drawable : other.drawables_)
        drawables_.emplace_back(std::make_unique<SvgDrawable>(*drawable));
      return *this;
    }

    Svg(const Svg& other) { *this = other; }

    Svg(const unsigned char* data, int data_size, int width = 0, int height = 0, int blur_radius = 0) :
        width(width), height(height), blur_radius(blur_radius) {
      parseData(data, data_size);
    }
    Svg(const EmbeddedFile& file, int width = 0, int height = 0, int blur_radius = 0) :
        Svg(file.data, file.size, width, height, blur_radius) { }

    void draw(Canvas& canvas) const {
      for (const auto& drawable : drawables_)
        drawable->draw(canvas);
    }

    int width = 0;
    int height = 0;
    int blur_radius = 0;

  private:
    void parseData(const unsigned char* data, int data_size);

    std::vector<std::unique_ptr<SvgDrawable>> drawables_;
  };
}