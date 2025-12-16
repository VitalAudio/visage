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
#include <numeric>
#include <utility>
#include <vector>

namespace visage {
  struct GradientAtlasTexture;

  class Gradient {
  public:
    static constexpr int kMaxGradientResolution = 512;
    static Gradient kViridis;
    static Gradient kMagma;

    static int compare(const Gradient& a, const Gradient& b) {
      if (a.numColors() < b.numColors())
        return -1;
      if (a.numColors() > b.numColors())
        return 1;

      if (a.repeat_ != b.repeat_)
        return a.repeat_ ? -1 : 1;
      if (a.reflect_ != b.reflect_)
        return a.reflect_ ? -1 : 1;

      for (int i = 0; i < a.numColors(); ++i) {
        int comp = Color::compare(a.colors_[i], b.colors_[i]);
        if (comp)
          return comp;
        if (a.positions_[i] < b.positions_[i])
          return -1;
        if (a.positions_[i] > b.positions_[i])
          return 1;
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
      result.evenlySpace();
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
      evenlySpace();
    }

    bool isNone() const {
      for (auto& color : colors_) {
        if (color.alpha() > 0.0f)
          return false;
      }
      return true;
    }

    void evenlySpace() {
      positions_.resize(colors_.size(), 0.0f);
      if (colors_.size() > 1) {
        float step = 1.0f / (colors_.size() - 1);
        for (int i = 0; i < colors_.size(); ++i)
          positions_[i] = i * step;
      }
    }

    Color sample(float t) const {
      if (colors_.empty())
        return {};
      if (colors_.size() <= 1)
        return colors_[0];

      if (reflect_) {
        t *= 2.0f;
        if (t > 1.0f)
          t = 2.0f - t;
      }

      auto it = std::upper_bound(positions_.begin(), positions_.end(), t);
      if (it == positions_.begin())
        return colors_.front();
      if (it == positions_.end())
        return colors_.back();
      int index = std::distance(positions_.begin(), it);
      float t0 = positions_[index - 1];
      float t1 = positions_[index];
      float local_t = (t - t0) / std::max(0.000001f, t1 - t0);
      return colors_[index - 1].interpolateWith(colors_[index], local_t);
    }

    int numColors() const { return colors_.size(); }
    void setRepeat(bool repeat) { repeat_ = repeat; }
    void setReflect(bool reflect) { reflect_ = reflect; }
    bool repeat() const { return repeat_; }
    bool reflect() const { return reflect_; }

    int resolution() const {
      if (custom_stops_ || repeat_ || reflect_)
        return kMaxGradientResolution;
      return std::min<int>(kMaxGradientResolution, colors_.size());
    }

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

    void addColorStop(const Color& color, float position) {
      position = std::clamp(position, 0.0f, 1.0f);
      auto it = std::upper_bound(positions_.begin(), positions_.end(), position);
      int index = std::distance(positions_.begin(), it);
      positions_.insert(it, position);
      colors_.insert(colors_.begin() + index, color);
      custom_stops_ = true;
    }

    Gradient interpolateWith(const Gradient& other, float t) const {
      return interpolate(*this, other, t);
    }

    Gradient withMultipliedAlpha(float mult) const {
      Gradient result = *this;

      for (int i = 0; i < colors_.size(); ++i)
        result.colors_[i] = colors_[i].withAlpha(colors_[i].alpha() * mult);

      return result;
    }

    Gradient operator*(const Gradient& other) const {
      auto sample_function = [&](float s) { return sample(s) * other.sample(s); };
      return fromSampleFunction(std::max(resolution(), other.resolution()), sample_function);
    }

    std::string encode() const;
    void encode(std::ostringstream& stream) const;
    void decode(const std::string& data);
    void decode(std::istringstream& stream);

  private:
    void sort() {
      if (colors_.size() <= 1)
        return;

      std::vector<int> indices(colors_.size());
      std::iota(indices.begin(), indices.end(), 0);
      std::sort(indices.begin(), indices.end(),
                [&](int a, int b) { return Color::compare(colors_[a], colors_[b]) < 0; });
      std::vector<Color> sorted_colors(colors_.size());
      std::vector<float> sorted_positions(colors_.size());
      for (int i = 0; i < colors_.size(); ++i) {
        sorted_colors[i] = colors_[indices[i]];
        sorted_positions[i] = positions_[indices[i]];
      }
      colors_ = std::move(sorted_colors);
      positions_ = std::move(sorted_positions);
    }

    std::vector<Color> colors_;
    std::vector<float> positions_;
    bool custom_stops_ = false;
    bool repeat_ = false;
    bool reflect_ = false;
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
        references_.erase(stale.first);
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
    bool repacked_ = false;
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
      Radial,
    };

    static GradientPosition interpolate(const GradientPosition& from, const GradientPosition& to, float t) {
      VISAGE_ASSERT(from.shape == to.shape || from.shape == InterpolationShape::Solid ||
                    to.shape == InterpolationShape::Solid);
      GradientPosition result;
      result.shape = from.shape;
      result.point1 = from.point1 + (to.point1 - from.point1) * t;
      result.point2 = from.point2 + (to.point2 - from.point2) * t;
      result.coefficientx2 = from.coefficientx2 + (to.coefficientx2 - from.coefficientx2) * t;
      result.coefficienty2 = from.coefficienty2 + (to.coefficienty2 - from.coefficienty2) * t;
      result.coefficientxy = from.coefficientxy + (to.coefficientxy - from.coefficientxy) * t;
      result.focal_radius = from.focal_radius + (to.focal_radius - from.focal_radius) * t;
      return result;
    }

    GradientPosition() = default;
    explicit GradientPosition(InterpolationShape shape) : shape(shape) { }

    GradientPosition(Point from, Point to) :
        shape(InterpolationShape::PointsLinear), point1(from), point2(to) { }

    InterpolationShape shape = InterpolationShape::Solid;
    Point point1;
    Point point2;
    float focal_radius = 0.0f;
    float coefficientx2 = 0.0f;
    float coefficienty2 = 0.0f;
    float coefficientxy = 0.0f;

    GradientPosition interpolateWith(const GradientPosition& other, float t) const {
      return interpolate(*this, other, t);
    }

    GradientPosition transformed(const Transform& transform) const {
      GradientPosition result = *this;
      if (shape == InterpolationShape::Radial) {
        result.point1 = transform * point1;
        result.point2 = transform * point2;
        auto inverse = transform.matrix.inversed();
        float a = inverse.matrix[0][0];
        float b = inverse.matrix[0][1];
        float c = inverse.matrix[1][0];
        float d = inverse.matrix[1][1];

        result.coefficientx2 = coefficientx2 * a * a + coefficienty2 * c * c + coefficientxy * a * c;
        result.coefficienty2 = coefficientx2 * b * b + coefficienty2 * d * d + coefficientxy * b * d;
        result.coefficientxy = 2.0f * (coefficientx2 * a * b + coefficienty2 * c * d) +
                               coefficientxy * (a * d + b * c);
      }
      else if (shape == InterpolationShape::PointsLinear) {
        result.point1 = transform * point1;
        auto delta = point2 - point1;
        auto transformed_delta = transform.matrix * delta;
        auto dual = transform.matrix.transposed().inversed() * delta;
        auto new_delta = dual * (dual.dot(transformed_delta) / dual.dot(dual));
        result.point2 = result.point1 + new_delta;
      }
      return result;
    }

    std::string encode() const;
    void encode(std::ostringstream& stream) const;
    void decode(const std::string& data);
    void decode(std::istringstream& stream);

    GradientPosition operator*(float mult) const {
      GradientPosition result = *this;
      result.point1 *= mult;
      result.point2 *= mult;
      if (mult) {
        float coefficient_scale = 1.0f / (mult * mult);
        result.coefficientx2 *= coefficient_scale;
        result.coefficienty2 *= coefficient_scale;
        result.coefficientxy *= coefficient_scale;
      }
      return result;
    }

    static GradientPosition linear(const Point& from, const Point& to) {
      return GradientPosition(from, to);
    }

    static GradientPosition radial(const Point& center, float radius_x, float radius_y,
                                   Point focal_center, float focal_radius = 0.0f) {
      radius_x = std::max(0.0001f, radius_x);
      radius_y = std::max(0.0001f, radius_y);

      GradientPosition position;
      position.point1 = center;
      position.point2 = focal_center;
      position.shape = InterpolationShape::Radial;
      position.coefficientxy = 0.0f;
      position.coefficientx2 = 1.0f / (radius_x * radius_x);
      position.coefficienty2 = 1.0f / (radius_y * radius_y);
      position.focal_radius = focal_radius;
      return position;
    }

    static GradientPosition radial(const Point& center, float radius_x, float radius_y) {
      return radial(center, radius_x, radius_y, center);
    }

    static GradientPosition radial(const Point& center, float radius) {
      return radial(center, radius, radius);
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

    static Brush radial(Gradient gradient, const Point& center, float radius_x, float radius_y,
                        Point focal_center, float focal_radius = 0.0f) {
      return { std::move(gradient),
               GradientPosition::radial(center, radius_x, radius_y, focal_center, focal_radius) };
    }

    static Brush radial(const Color& from_color, const Color& to_color, const Point& center,
                        float radius_x, float radius_y, Point focal_center, float focal_radius = 0.0f) {
      return { Gradient(from_color, to_color),
               GradientPosition::radial(center, radius_x, radius_y, focal_center, focal_radius) };
    }

    static Brush radial(Gradient gradient, const Point& center, float radius_x, float radius_y) {
      return radial(std::move(gradient), center, radius_x, radius_y, center);
    }

    static Brush radial(const Color& from_color, const Color& to_color, const Point& center,
                        float radius_x, float radius_y) {
      return radial(Gradient(from_color, to_color), center, radius_x, radius_y, center);
    }

    static Brush radial(Gradient gradient, const Point& center, float radius) {
      return radial(std::move(gradient), center, radius, radius);
    }

    static Brush radial(const Color& from_color, const Color& to_color, const Point& center, float radius) {
      return radial(Gradient(from_color, to_color), center, radius, radius);
    }

    static Brush interpolate(const Brush& from, const Brush& to, float t) {
      return { from.gradient_.interpolateWith(to.gradient_, t),
               from.position_.interpolateWith(to.position_, t) };
    }

    Brush() = default;

    Brush(Gradient gradient, const GradientPosition& position) :
        gradient_(std::move(gradient)), position_(position) { }

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
    bool isNone() const { return gradient_.isNone(); }

    void transform(const Transform& transform) { position_ = position_.transformed(transform); }

  private:
    Gradient gradient_;
    GradientPosition position_;

    VISAGE_LEAK_CHECKER(Brush)
  };

  class PackedBrush {
  public:
    static void computeVertexGradientTexturePositions(GradientTexturePosition& result,
                                                      const PackedBrush* brush) {
      if (brush) {
        float atlas_x_scale = 1.0f / brush->atlasWidth();
        float atlas_y_scale = 1.0f / brush->atlasHeight();
        const auto& gradient = brush->gradient()->gradient();
        bool repeating = gradient.repeat() || gradient.reflect();
        int offset = repeating ? 0 : 1;
        result.from_x = (brush->gradient_.x() + offset * 0.5f) * atlas_x_scale;
        float span = (gradient.resolution() - offset) * atlas_x_scale;
        if (gradient.reflect())
          span *= 0.5f;
        result.to_x = result.from_x + span;
        result.from_y = (brush->gradient_.y() + 0.5f) * atlas_y_scale;
        result.to_y = result.from_y;
      }
    }

    static void computeVertexGradientPositions(GradientVertexPosition& result, const PackedBrush* brush,
                                               float offset_x, float offset_y, float left,
                                               float top, float right, float bottom) {
      result.from_x = -1.0f;
      result.to_x = 1.0f;
      result.from_y = -1.0f;
      result.to_y = 1.0f;
      result.coefficient1 = 1.0f;
      result.coefficient2 = 1.0f;
      result.coefficient3 = 1.0f;
      result.cone_height = 1.0f;

      if (brush == nullptr)
        return;

      if (brush->position_.shape == GradientPosition::InterpolationShape::Horizontal) {
        result.from_x = left + 0.5f;
        result.to_x = right - 0.5f;
        result.from_y = 0.0f;
        result.to_y = 0.0f;
      }
      else if (brush->position_.shape == GradientPosition::InterpolationShape::Vertical) {
        result.from_x = 0.0f;
        result.to_x = 0.0f;
        result.from_y = top + 0.5f;
        result.to_y = bottom - 0.5f;
      }
      else {
        result.from_x = offset_x + brush->position_.point1.x;
        result.from_y = offset_y + brush->position_.point1.y;
        if (brush->position_.shape == GradientPosition::InterpolationShape::Radial) {
          result.coefficient1 = brush->position_.coefficientx2;
          result.coefficient2 = brush->position_.coefficienty2;
          result.coefficient3 = brush->position_.coefficientxy;
          result.cone_height = 1.0f / (1.0f - brush->position_.focal_radius);
          auto delta = brush->position_.point2 - brush->position_.point1;
          auto cone_xy = delta * result.cone_height;
          result.to_x = cone_xy.x;
          result.to_y = cone_xy.y;
        }
        else {
          result.to_x = offset_x + brush->position_.point2.x;
          result.to_y = offset_y + brush->position_.point2.y;
        }
      }
    }

    template<typename V>
    static void setVertexGradientPositions(const PackedBrush* brush, V* vertices, int num_vertices,
                                           float offset_x, float offset_y, float left, float top,
                                           float right, float bottom) {
      if (num_vertices <= 0)
        return;

      computeVertexGradientTexturePositions(vertices[0].gradient_texture_position, brush);
      computeVertexGradientPositions(vertices[0].gradient, brush, offset_x, offset_y, left, top,
                                     right, bottom);

      for (int i = 1; i < num_vertices; ++i) {
        vertices[i].gradient_texture_position = vertices[0].gradient_texture_position;
        vertices[i].gradient = vertices[0].gradient;
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

    Brush originalBrush() const { return Brush(gradient_.gradient(), position_); }

  private:
    GradientAtlas* atlas_ = nullptr;
    GradientPosition position_;
    GradientAtlas::PackedGradient gradient_;

    VISAGE_LEAK_CHECKER(PackedBrush)
  };
}