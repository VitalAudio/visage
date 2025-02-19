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

#include <functional>
#include <map>
#include <vector>

namespace visage {
  struct GradientAtlasTexture;

  class Gradient {
  public:
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

    static Gradient interpolate(const Gradient& from, const Gradient& to, float t) {
      auto sample_function = [&](float s) { return from.sample(s).interpolateWith(to.sample(s), t); };
      return Gradient(std::max(from.resolution(), to.resolution()), sample_function);
    }

    Gradient() = default;

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

    template<typename... Args, typename = std::enable_if_t<(std::is_same_v<Args, Color> && ...)>>
    explicit Gradient(const Args&... args) {
      colors_.reserve(sizeof...(args));
      hdrs_.reserve(sizeof...(args));

      (colors_.emplace_back(args.toARGB()), ...);
      (hdrs_.emplace_back(args.hdr()), ...);
    }

    Color sample(float t) const {
      if (colors_.size() <= 1)
        return Color(colors_[0], hdrs_[0]);

      float position = t * (resolution() - 1);
      int index = std::min(resolution() - 2, static_cast<int>(position));
      Color from(colors_[index], hdrs_[index]);
      return from.interpolateWith(Color(colors_[index + 1], hdrs_[index + 1]), position - index);
    }

    int resolution() const { return colors_.size(); }

    bool operator<(const Gradient& other) const { return compare(*this, other) < 0; }

    const std::vector<unsigned int>& colors() const { return colors_; }
    const std::vector<float>& hdrs() const { return hdrs_; }

    Gradient interpolateWith(const Gradient& other, float t) const {
      return interpolate(*this, other, t);
    }

    Gradient withMultipliedAlpha(float mult) const {
      Gradient result;
      result.colors_.reserve(colors_.size());
      result.hdrs_.reserve(hdrs_.size());

      for (int i = 0; i < colors_.size(); ++i) {
        unsigned int color = colors_[i];
        unsigned int alpha = std::round(mult * ((color & 0xff000000) >> 24));
        result.colors_.emplace_back((color & 0x00ffffff) | (alpha << 24));
        result.hdrs_.emplace_back(hdrs_[i]);
      }
      return result;
    }

  private:
    std::vector<unsigned int> colors_;
    std::vector<float> hdrs_;
  };

  class GradientAtlas {
  public:
    struct PackedGradient {
      explicit PackedGradient(Gradient g) : gradient(std::move(g)) { }

      Gradient gradient;
      int x = 0;
      int y = 0;
    };

    GradientAtlas();
    ~GradientAtlas();

    const PackedGradient* addGradient(const Gradient& gradient) {
      if (num_references_[gradient] == 0) {
        gradients_[gradient] = std::make_unique<PackedGradient>(gradient);
        if (!atlas_.addRect(gradients_[gradient].get(), gradient.resolution(), 1))
          resize();
      }
      num_references_[gradient]++;

      return gradients_[gradient].get();
    }

    void removeGradient(const Gradient& gradient) {
      VISAGE_ASSERT(num_references_[gradient] > 0);

      num_references_[gradient]--;
      if (num_references_[gradient] == 0) {
        atlas_.removeRect(gradients_[gradient].get());
        num_references_.erase(gradient);
        gradients_.erase(gradient);
      }
    }

    void removeGradient(const PackedGradient* packed_gradient) {
      removeGradient(packed_gradient->gradient);
    }

    void updateGradient(const PackedGradient* gradient);
    void checkInit();
    void resize();
    int width() { return atlas_.width(); }
    int height() { return atlas_.height(); }

    const bgfx::TextureHandle& colorTextureHandle();
    const bgfx::TextureHandle& hdrTextureHandle();

  private:
    std::map<Gradient, std::unique_ptr<PackedGradient>> gradients_;
    std::map<Gradient, int> num_references_;

    PackedAtlas<const PackedGradient*> atlas_;
    std::unique_ptr<GradientAtlasTexture> texture_;
  };

  struct GradientPosition {
    static GradientPosition interpolate(const GradientPosition& from, const GradientPosition& to, float t) {
      VISAGE_ASSERT(from.shape == to.shape);
      GradientPosition result;
      result.shape = from.shape;
      result.point_from = from.point_from + (to.point_from - from.point_from) * t;
      result.point_to = from.point_to + (to.point_to - from.point_to) * t;
      return result;
    }

    enum class InterpolationShape {
      Solid,
      Horizontal,
      Vertical,
      PointsLinear,
      PointsRadial
    };

    GradientPosition() = default;
    explicit GradientPosition(InterpolationShape shape) : shape(shape) { }

    GradientPosition(FloatPoint from, FloatPoint to) :
        shape(InterpolationShape::PointsLinear), point_from(from), point_to(to) { }

    GradientPosition(FloatPoint center, float start_radius, float end_radius) :
        shape(InterpolationShape::PointsRadial), point_from(center),
        point_to(start_radius, end_radius) { }

    InterpolationShape shape = InterpolationShape::Solid;
    FloatPoint point_from;
    FloatPoint point_to;

    GradientPosition interpolateWith(const GradientPosition& other, float t) const {
      return interpolate(*this, other, t);
    }
  };

  class Brush {
  public:
    static Brush solid(Color color) {
      return Brush(Gradient(color), GradientPosition(GradientPosition::InterpolationShape::Solid));
    }

    static Brush horizontal(Gradient gradient) {
      return Brush(std::move(gradient), GradientPosition(GradientPosition::InterpolationShape::Horizontal));
    }

    static Brush horizontal(const Color& left, const Color& right) {
      return horizontal(Gradient(left, right));
    }

    static Brush vertical(Gradient gradient) {
      return Brush(std::move(gradient), GradientPosition(GradientPosition::InterpolationShape::Vertical));
    }

    static Brush vertical(const Color& top, const Color& bottom) {
      return vertical(Gradient(top, bottom));
    }

    static Brush linear(Gradient gradient, const FloatPoint& from_position, const FloatPoint& to_position) {
      return Brush(std::move(gradient), GradientPosition(from_position, to_position));
    }

    static Brush linear(const Color& from_color, const Color& to_color,
                        const FloatPoint& from_position, const FloatPoint& to_position) {
      return linear(Gradient(from_color, to_color), from_position, to_position);
    }

    static Brush radial(Gradient gradient, const FloatPoint& center, float from_radius, float to_radius) {
      return Brush(std::move(gradient), GradientPosition(center, from_radius, to_radius));
    }

    static Brush radial(const Color& from_color, const Color& to_color, const FloatPoint& center,
                        float from_radius, float to_radius) {
      return radial(Gradient(from_color, to_color), center, from_radius, to_radius);
    }

    static Brush interpolate(const Brush& from, const Brush& to, float t) {
      VISAGE_ASSERT(from.position_.shape == to.position_.shape);
      return Brush(from.gradient_.interpolateWith(to.gradient_, t),
                   from.position_.interpolateWith(to.position_, t));
    }

    Brush interpolateWith(const Brush& other, float t) { return interpolate(*this, other, t); }

    Brush withMultipliedAlpha(float mult) {
      return Brush(gradient_.withMultipliedAlpha(mult), position_);
    }

    const Gradient& gradient() const { return gradient_; }
    const GradientPosition& position() const { return position_; }

    Brush() = default;

  private:
    Brush(Gradient gradient, GradientPosition position) :
        gradient_(std::move(gradient)), position_(std::move(position)) { }

    Gradient gradient_;
    GradientPosition position_;
  };

  class PackedBrush {
  public:
    template<typename V>
    static void setVertexGradientPositions(const PackedBrush* brush, V* vertices, int num_vertices,
                                           float origin_x, float origin_y, float left, float top,
                                           float right, float bottom) {
      float gradient_position_from_x = 0.0f;
      float gradient_position_from_y = 0.0f;
      float gradient_position_to_x = 0.0f;
      float gradient_position_to_y = 0.0f;
      float gradient_color_from_x = -1.0f;
      float gradient_color_to_x = -1.0f;
      float gradient_color_y = -1.0f;

      if (brush) {
        if (brush->position_.shape == GradientPosition::InterpolationShape::Horizontal) {
          gradient_position_from_x = origin_x + left;
          gradient_position_to_x = origin_x + right;
        }
        else if (brush->position_.shape == GradientPosition::InterpolationShape::Vertical) {
          gradient_position_from_y = origin_y + top;
          gradient_position_to_y = origin_y + bottom;
        }
        else if (brush->position_.shape == GradientPosition::InterpolationShape::PointsLinear) {
          gradient_position_from_x = origin_x + brush->position_.point_from.x;
          gradient_position_from_y = origin_y + brush->position_.point_from.y;
          gradient_position_to_x = origin_x + brush->position_.point_to.x;
          gradient_position_to_y = origin_y + brush->position_.point_to.y;
        }

        float atlas_x_scale = 2.0f / brush->atlasWidth();
        float atlas_y_scale = 2.0f / brush->atlasHeight();
        gradient_color_from_x += brush->gradient_->x / atlas_x_scale;
        gradient_color_to_x += gradient_color_from_x + brush->gradient_->gradient.resolution() / atlas_x_scale;
        gradient_color_y += brush->gradient_->y / atlas_y_scale - 1.0f;
      }

      for (int i = 0; i < num_vertices; ++i) {
        vertices[i].gradient_color_from_x = gradient_color_from_x;
        vertices[i].gradient_color_from_y = gradient_color_y;
        vertices[i].gradient_color_to_x = gradient_color_to_x;
        vertices[i].gradient_color_to_y = gradient_color_y;
        vertices[i].gradient_position_from_x = gradient_position_from_x;
        vertices[i].gradient_position_from_y = gradient_position_from_y;
        vertices[i].gradient_position_to_x = gradient_position_to_x;
        vertices[i].gradient_position_to_y = gradient_position_to_y;
      }
    }

    explicit PackedBrush(GradientAtlas* atlas, const Brush& brush) :
        atlas_(atlas), position_(brush.position()), gradient_(atlas->addGradient(brush.gradient())) { }

    PackedBrush::PackedBrush(const PackedBrush& other) :
        atlas_(other.atlas_), position_(other.position_),
        gradient_(atlas_->addGradient(other.gradient_->gradient)) { }

    PackedBrush& PackedBrush::operator=(const PackedBrush& other) {
      if (this == &other)
        return *this;

      auto old_gradient = gradient_;
      auto old_atlas = atlas_;
      atlas_ = other.atlas_;
      position_ = other.position_;
      gradient_ = atlas_->addGradient(other.gradient_->gradient);

      if (old_gradient)
        old_atlas->removeGradient(old_gradient);
      return *this;
    }

    PackedBrush::~PackedBrush() {
      if (gradient_)
        atlas_->removeGradient(gradient_);
    }

    const GradientAtlas::PackedGradient* gradient() const { return gradient_; }
    const GradientPosition& position() const { return position_; }
    int atlasWidth() const { return atlas_->width(); }
    int atlasHeight() const { return atlas_->height(); }

  private:
    GradientAtlas* atlas_ = nullptr;
    GradientPosition position_;
    const GradientAtlas::PackedGradient* gradient_ = nullptr;
  };
}