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

#include "color.h"
#include "graphics_utils.h"

#include <map>
#include <vector>

namespace visage {
  struct GradientAtlasTexture;

  class Gradient {
  public:
    Gradient() = default;

    template<typename... Args>
    explicit Gradient(Args... args) {
      std::vector<Color> colors { args... };
      int resolution = colors.size();
      colors_.reserve(resolution);
      hdrs_.reserve(resolution);

      for (int i = 0; i < resolution; ++i) {
        colors_.emplace_back(colors[i].toARGB());
        hdrs_.emplace_back(colors[i].hdr());
      }
    }

    Gradient(int resolution, const std::function<Color(float)>& sample_function) {
      VISAGE_ASSERT(resolution > 1);
      colors_.reserve(resolution);
      hdrs_.reserve(resolution);

      for (int i = 0; i < resolution; ++i) {
        Color color = sample_function(i / (resolution - 1.0f));
        colors_.emplace_back(color.toARGB());
        hdrs_.emplace_back(color.hdr());
      }
    }

    int resolution() const { return colors_.size(); }

    static int compare(const Gradient& a, const Gradient& b) {
      if (a.resolution() < b.resolution())
        return -1;
      if (a.resolution() > b.resolution())
        return 1;

      for (int i = 0; i < a.resolution(); ++i) {
        if (a.colors_[i] < b.colors_[i])
          return -1;
        if (a.colors_[i] > b.colors_[i])
          return 1;
        if (a.hdrs_[i] < b.hdrs_[i])
          return -1;
        if (a.hdrs_[i] > b.hdrs_[i])
          return 1;
      }
      return 0;
    }

    bool operator<(const Gradient& other) const { return compare(*this, other) < 0; }

    const std::vector<unsigned char>& colors() const { return colors_; }
    const std::vector<float>& hdrs() const { return hdrs_; }

  private:
    std::vector<unsigned char> colors_;
    std::vector<float> hdrs_;
  };

  class HorizontalGradient {
  public:
    HorizontalGradient(const Color& left, const Color& right) : left_(left), right_(right) { }

    const Color& left() const { return left_; }
    const Color& right() const { return right_; }

  private:
    Color left_;
    Color right_;
  };

  class VerticalGradient {
  public:
    VerticalGradient(const Color& top, const Color& bottom) : top_(top), bottom_(bottom) { }

    const Color& top() const { return top_; }
    const Color& bottom() const { return bottom_; }

  private:
    Color top_;
    Color bottom_;
  };

  struct PackedGradient {
    explicit PackedGradient(Gradient g) : gradient(std::move(g)) { }

    Gradient gradient;
    int x = 0;
    int y = 0;
  };

  class GradientAtlas {
  public:
    void addGradient(const Gradient& gradient) {
      if (num_references_[gradient] == 0) {
        gradients_[gradient] = std::make_unique<PackedGradient>(gradient);
        if (!atlas_.addRect(gradients_[gradient].get(), gradient.resolution(), 1))
          resize();
      }
      num_references_[gradient]++;
    }

    void removeGradient(const Gradient& gradient) {
      VISAGE_ASSERT(num_references_[gradient] > 1);

      num_references_[gradient]--;
      if (num_references_[gradient] == 0) {
        atlas_.removeRect(gradients_[gradient].get());
        num_references_.erase(gradient);
        gradients_.erase(gradient);
      }
    }

    void updateGradient(const PackedGradient* gradient);
    void checkInit();
    void resize();

    const bgfx::TextureHandle& colorTextureHandle();
    const bgfx::TextureHandle& hdrTextureHandle();

  private:
    std::map<Gradient, std::unique_ptr<PackedGradient>> gradients_;
    std::map<Gradient, int> num_references_;

    PackedAtlas<const PackedGradient*> atlas_;
    std::unique_ptr<GradientAtlasTexture> texture_;
  };

  struct GradientPosition {
    enum class InterpolationShape {
      None,
      Horizontal,
      Vertical,
      PointsLinear,
      PointsRadial
    };

    explicit GradientPosition(InterpolationShape shape) : shape(shape) { }

    GradientPosition(FloatPoint from, FloatPoint to) :
        shape(InterpolationShape::PointsLinear), point_from(from), point_to(to) { }

    GradientPosition(FloatPoint center, float start_radius, float end_radius) :
        shape(InterpolationShape::PointsRadial), point_from(center),
        point_to(start_radius, end_radius) { }

    InterpolationShape shape = InterpolationShape::None;
    FloatPoint point_from;
    FloatPoint point_to;
  };
}