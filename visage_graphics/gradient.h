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
#include <iosfwd>
#include <map>
#include <utility>
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
        int comp = Color::compare(a.colors_[i], b.colors_[i]);
        if (comp)
          return comp;
      }
      return 0;
    }

    static Gradient fromSampleFunction(int resolution, const std::function<Color(float)>& sample_function) {
      VISAGE_ASSERT(resolution > 0);
      Gradient result;
      result.colors_.reserve(resolution);

      float normalization = 1.0f / std::max(1.0f, resolution - 1.0f);
      for (int i = 0; i < resolution; ++i)
        result.colors_.emplace_back(sample_function(i * normalization));

      return result;
    }

    static Gradient interpolate(const Gradient& from, const Gradient& to, float t) {
      auto sample_function = [&](float s) { return from.sample(s).interpolateWith(to.sample(s), t); };
      return fromSampleFunction(std::max(from.resolution(), to.resolution()), sample_function);
    }

    Gradient() = default;

    template<typename... Args>
    explicit Gradient(const Args&... args) {
      colors_.reserve(sizeof...(args));
      (colors_.emplace_back(Color(args)), ...);
    }

    Color sample(float t) const {
      if (colors_.empty())
        return {};
      if (colors_.size() <= 1)
        return colors_[0];

      float position = t * (resolution() - 1);
      int index = std::min(resolution() - 2, static_cast<int>(position));
      return colors_[index].interpolateWith(colors_[index + 1], position - index);
    }

    int resolution() const { return colors_.size(); }
    void setResolution(int resolution) {
      if (!colors_.empty())
        colors_.resize(resolution, colors_.back());
      else
        colors_.resize(resolution);
    }

    bool operator<(const Gradient& other) const { return compare(*this, other) < 0; }

    const std::vector<Color>& colors() const { return colors_; }
    void setColor(int index, const Color& color) {
      VISAGE_ASSERT(index < colors_.size());
      colors_[index] = color;
    }

    Gradient interpolateWith(const Gradient& other, float t) const {
      return interpolate(*this, other, t);
    }

    Gradient withMultipliedAlpha(float mult) const {
      Gradient result;
      result.colors_.reserve(colors_.size());

      for (const Color& color : colors_)
        result.colors_.emplace_back(color.withAlpha(color.alpha() * mult));

      return result;
    }

    std::string encode() const;
    void encode(std::ostringstream& stream) const;
    void decode(const std::string& data);
    void decode(std::istringstream& stream);

  private:
    std::vector<Color> colors_;
  };

  class GradientAtlas {
  public:
    struct PackedGradientRect {
      explicit PackedGradientRect(Gradient g) : gradient(std::move(g)) { }

      Gradient gradient;
      int x = 0;
      int y = 0;
    };

    struct PackedGradientReference {
      PackedGradientReference(std::weak_ptr<GradientAtlas*> atlas,
                              const PackedGradientRect* packed_gradient_rect) :
          atlas(std::move(atlas)), packed_gradient_rect(packed_gradient_rect) { }
      ~PackedGradientReference();

      std::weak_ptr<GradientAtlas*> atlas;
      const PackedGradientRect* packed_gradient_rect = nullptr;
    };

    class PackedGradient {
    public:
      int x() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_gradient_rect->x;
      }

      int y() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_gradient_rect->y;
      }

      const Gradient& gradient() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_gradient_rect->gradient;
      }

      explicit PackedGradient(std::shared_ptr<PackedGradientReference> reference) :
          reference_(std::move(reference)) { }

    private:
      std::shared_ptr<PackedGradientReference> reference_;
    };

    GradientAtlas();
    ~GradientAtlas();

    PackedGradient addGradient(const Gradient& gradient) {
      if (gradients_.count(gradient) == 0) {
        std::unique_ptr<PackedGradientRect> packed_gradient_rect = std::make_unique<PackedGradientRect>(gradient);
        if (!atlas_map_.addRect(packed_gradient_rect.get(), gradient.resolution(), 1))
          resize();

        const PackedRect& rect = atlas_map_.rectForId(packed_gradient_rect.get());
        packed_gradient_rect->x = rect.x;
        packed_gradient_rect->y = rect.y;
        updateGradient(packed_gradient_rect.get());
        gradients_[gradient] = std::move(packed_gradient_rect);
      }
      stale_gradients_.erase(gradient);

      if (auto reference = references_[gradient].lock())
        return PackedGradient(reference);

      auto reference = std::make_shared<PackedGradientReference>(reference_, gradients_[gradient].get());
      references_[gradient] = reference;
      return PackedGradient(reference);
    }

    void clearStaleGradients() {
      for (const auto& stale : stale_gradients_) {
        gradients_.erase(stale.first);
        atlas_map_.removeRect(stale.second);
      }
      stale_gradients_.clear();
    }

    void checkInit();
    void destroy();
    void setHdr(bool hdr) {
      hdr_ = hdr;
      destroy();
    }
    int width() const { return atlas_map_.width(); }
    int height() const { return atlas_map_.height(); }

    const bgfx::TextureHandle& colorTextureHandle();

  private:
    void updateGradient(const PackedGradientRect* gradient);
    void resize();

    void removeGradient(const Gradient& gradient) {
      VISAGE_ASSERT(gradients_.count(gradient));
      stale_gradients_[gradient] = gradients_[gradient].get();
    }

    void removeGradient(const PackedGradientRect* packed_gradient_rect) {
      removeGradient(packed_gradient_rect->gradient);
    }

    std::map<Gradient, std::weak_ptr<PackedGradientReference>> references_;
    std::map<Gradient, std::unique_ptr<PackedGradientRect>> gradients_;
    std::map<Gradient, const PackedGradientRect*> stale_gradients_;

    bool hdr_ = false;
    PackedAtlasMap<const PackedGradientRect*> atlas_map_;
    std::unique_ptr<GradientAtlasTexture> texture_;
    std::shared_ptr<GradientAtlas*> reference_;

    VISAGE_LEAK_CHECKER(GradientAtlas)
  };

  struct GradientPosition {
    enum class InterpolationShape {
      Solid,
      Horizontal,
      Vertical,
      PointsLinear,
    };

    static GradientPosition interpolate(const GradientPosition& from, const GradientPosition& to, float t) {
      VISAGE_ASSERT(from.shape == to.shape || from.shape == InterpolationShape::Solid ||
                    to.shape == InterpolationShape::Solid);
      GradientPosition result;
      result.shape = from.shape;
      result.point_from = from.point_from + (to.point_from - from.point_from) * t;
      result.point_to = from.point_to + (to.point_to - from.point_to) * t;
      return result;
    }

    GradientPosition() = default;
    explicit GradientPosition(InterpolationShape shape) : shape(shape) { }

    GradientPosition(Point from, Point to) :
        shape(InterpolationShape::PointsLinear), point_from(from), point_to(to) { }

    InterpolationShape shape = InterpolationShape::Solid;
    Point point_from;
    Point point_to;

    GradientPosition interpolateWith(const GradientPosition& other, float t) const {
      return interpolate(*this, other, t);
    }

    std::string encode() const;
    void encode(std::ostringstream& stream) const;
    void decode(const std::string& data);
    void decode(std::istringstream& stream);

    GradientPosition operator*(float mult) const {
      GradientPosition result = *this;
      result.point_from *= mult;
      result.point_to *= mult;
      return result;
    }

    VISAGE_LEAK_CHECKER(GradientPosition)
  };

  class Brush {
  public:
    static Brush none() {
      return { {}, GradientPosition(GradientPosition::InterpolationShape::Solid) };
    }

    static Brush solid(const Color& color) {
      return { Gradient(color), GradientPosition(GradientPosition::InterpolationShape::Solid) };
    }

    static Brush horizontal(const Gradient& gradient) {
      return { gradient, GradientPosition(GradientPosition::InterpolationShape::Horizontal) };
    }

    static Brush horizontal(const Color& left, const Color& right) {
      return horizontal(Gradient(left, right));
    }

    static Brush vertical(Gradient gradient) {
      return { std::move(gradient), GradientPosition(GradientPosition::InterpolationShape::Vertical) };
    }

    static Brush vertical(const Color& top, const Color& bottom) {
      return vertical(Gradient(top, bottom));
    }

    static Brush linear(Gradient gradient, const Point& from_position, const Point& to_position) {
      return { std::move(gradient), GradientPosition(from_position, to_position) };
    }

    static Brush linear(const Color& from_color, const Color& to_color, const Point& from_position,
                        const Point& to_position) {
      return linear(Gradient(from_color, to_color), from_position, to_position);
    }

    static Brush interpolate(const Brush& from, const Brush& to, float t) {
      return { from.gradient_.interpolateWith(to.gradient_, t),
               from.position_.interpolateWith(to.position_, t) };
    }

    Brush() = default;

    Brush interpolateWith(const Brush& other, float t) const {
      return interpolate(*this, other, t);
    }

    Brush withMultipliedAlpha(float mult) const {
      return { gradient_.withMultipliedAlpha(mult), position_ };
    }

    const Gradient& gradient() const { return gradient_; }
    Gradient& gradient() { return gradient_; }
    const GradientPosition& position() const { return position_; }
    GradientPosition& position() { return position_; }

    std::string encode() const;
    void encode(std::ostringstream& stream) const;
    void decode(const std::string& data);
    void decode(std::istringstream& stream);
    bool isNone() const { return gradient_.colors().empty(); }

  private:
    Brush(Gradient gradient, const GradientPosition& position) :
        gradient_(std::move(gradient)), position_(position) { }

    Gradient gradient_;
    GradientPosition position_;

    VISAGE_LEAK_CHECKER(Brush)
  };

  class PackedBrush {
  public:
    struct GradientTexturePosition {
      float gradient_position_from_x = 0.0f;
      float gradient_position_from_y = 0.0f;
      float gradient_position_to_x = 0.0f;
      float gradient_position_to_y = 0.0f;
      float gradient_color_from_x = 0.0f;
      float gradient_color_to_x = 0.0f;
      float gradient_color_y = 0.0f;
    };

    static GradientTexturePosition computeVertexGradientPositions(const PackedBrush* brush, float offset_x,
                                                                  float offset_y, float left, float top,
                                                                  float right, float bottom) {
      GradientTexturePosition result;

      if (brush) {
        if (brush->position_.shape == GradientPosition::InterpolationShape::Horizontal) {
          result.gradient_position_from_x = left + 0.5f;
          result.gradient_position_to_x = right - 0.5f;
        }
        else if (brush->position_.shape == GradientPosition::InterpolationShape::Vertical) {
          result.gradient_position_from_y = top + 0.5f;
          result.gradient_position_to_y = bottom - 0.5f;
        }
        else if (brush->position_.shape == GradientPosition::InterpolationShape::PointsLinear) {
          result.gradient_position_from_x = offset_x + brush->position_.point_from.x;
          result.gradient_position_from_y = offset_y + brush->position_.point_from.y;
          result.gradient_position_to_x = offset_x + brush->position_.point_to.x;
          result.gradient_position_to_y = offset_y + brush->position_.point_to.y;
        }

        float atlas_x_scale = 1.0f / brush->atlasWidth();
        float atlas_y_scale = 1.0f / brush->atlasHeight();
        result.gradient_color_from_x = (brush->gradient_.x() + 0.5f) * atlas_x_scale;
        result.gradient_color_to_x = result.gradient_color_from_x +
                                     (brush->gradient_.gradient().resolution() - 1) * atlas_x_scale;
        result.gradient_color_y = (brush->gradient_.y() + 0.5f) * atlas_y_scale;
      }

      return result;
    }

    template<typename V>
    static void setVertexGradientPositions(const PackedBrush* brush, V* vertices, int num_vertices,
                                           float offset_x, float offset_y, float left, float top,
                                           float right, float bottom) {
      GradientTexturePosition position = computeVertexGradientPositions(brush, offset_x, offset_y,
                                                                        left, top, right, bottom);

      for (int i = 0; i < num_vertices; ++i) {
        vertices[i].gradient_color_from_x = position.gradient_color_from_x;
        vertices[i].gradient_color_from_y = position.gradient_color_y;
        vertices[i].gradient_color_to_x = position.gradient_color_to_x;
        vertices[i].gradient_color_to_y = position.gradient_color_y;
        vertices[i].gradient_position_from_x = position.gradient_position_from_x;
        vertices[i].gradient_position_from_y = position.gradient_position_from_y;
        vertices[i].gradient_position_to_x = position.gradient_position_to_x;
        vertices[i].gradient_position_to_y = position.gradient_position_to_y;
      }
    }

    PackedBrush(GradientAtlas* atlas, const Gradient& gradient, const GradientPosition& position) :
        atlas_(atlas), position_(position), gradient_(atlas->addGradient(gradient)) { }

    PackedBrush(GradientAtlas* atlas, const Brush& brush) :
        atlas_(atlas), position_(brush.position()), gradient_(atlas->addGradient(brush.gradient())) { }

    const GradientAtlas::PackedGradient* gradient() const { return &gradient_; }
    const GradientPosition& position() const { return position_; }
    int atlasWidth() const { return atlas_->width(); }
    int atlasHeight() const { return atlas_->height(); }

  private:
    GradientAtlas* atlas_ = nullptr;
    GradientPosition position_;
    GradientAtlas::PackedGradient gradient_;

    VISAGE_LEAK_CHECKER(PackedBrush)
  };
}