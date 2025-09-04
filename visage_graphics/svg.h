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
#include "visage_graphics/gradient.h"
#include "visage_graphics/path.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace visage {
  class Canvas;

  enum class SvgClipBox {
    BorderBox,
    FillBox,
    StrokeBox,
    ViewBox,
  };

  struct TagData {
    std::string name;
    std::string text;
    std::map<std::string, std::string> attributes;
    bool is_closing = false;
    bool is_self_closing = false;
    bool ignored = false;
  };

  struct Tag {
    TagData data;
    std::vector<Tag> children;
  };

  struct CssSelector {
    std::string tag_name;
    std::string id;
    bool direct_child = false;
    std::vector<std::string> classes;
    std::vector<CssSelector> parents;

    bool matches(const Tag& tag) const;
  };

  struct GradientDef {
    GradientDef() = default;
    GradientDef(const Color& color) {
      gradient = Gradient(color);
      solid = true;
    }

    Brush toBrush(Transform current_transform, Bounds box) const {
      if (solid)
        return Brush::solid(gradient.colors().front());

      Transform scale_transform;
      if (!user_space) {
        scale_transform = Transform::translation(box.x(), box.y()) *
                          Transform::scale(box.width(), box.height());
      }

      GradientPosition position;
      if (radial) {
        float f_radius = focal_radius;
        if (radius)
          f_radius /= radius;
        else
          f_radius = radius;
        position = GradientPosition::radial(point1, radius, radius, point2, f_radius);
      }
      else
        position = GradientPosition::linear(point1, point2);

      position = position.transform(current_transform * scale_transform * transform);
      return Brush(gradient, position);
    }

    Gradient gradient;
    Transform transform;
    bool radial = false;
    bool solid = false;
    bool user_space = false;
    Point point1 = Point(0.0f, 0.0f);
    Point point2 = Point(1.0f, 0.0f);
    float focal_radius = 0.0f;
    float radius = 0.5f;
  };

  struct DrawableState {
    Transform local_transform;
    bool transform_ratio_x = false;
    bool transform_ratio_y = false;
    float tranform_origin_x = 0.0f;
    float tranform_origin_y = 0.0f;

    float opacity = 1.0f;

    Color current_color = Color(0xff000000);

    GradientDef fill_gradient = GradientDef(0xff000000);
    float fill_opacity = 1.0f;
    bool non_zero_fill = false;

    GradientDef stroke_gradient;
    float stroke_opacity = 1.0f;
    float stroke_width = 1.0f;
    Path::Join stroke_join = Path::Join::Miter;
    Path::EndCap stroke_end_cap = Path::EndCap::Butt;
    std::vector<std::pair<float, bool>> stroke_dasharray;
    float stroke_dashoffset = 0.0f;
    bool stroke_dashoffset_ratio = false;
    float stroke_miter_limit = 4.0f;

    bool visible = true;
  };

  struct SvgViewSettings {
    float width = 500.0f;
    float height = 500.0f;
    Bounds view_box;
    std::string align;
    std::string scale;
  };

  struct SvgDrawable {
    SvgDrawable() = default;
    SvgDrawable(const SvgDrawable& other) {
      state = other.state;
      fill_brush = other.fill_brush;
      stroke_brush = other.stroke_brush;
      path = other.path;
      stroke_path = other.stroke_path;
      command_list = other.command_list;
      children.reserve(other.children.size());
      for (const auto& child : other.children)
        children.emplace_back(std::make_unique<SvgDrawable>(*child));
    }

    void draw(Canvas& canvas, float x, float y, float width, float height) const;
    void fill(Canvas& canvas, float x, float y, float width, float height) const;
    void stroke(Canvas& canvas, float x, float y, float width, float height) const;
    bool hasFill() const { return !fill_brush.isNone() && state.fill_opacity > 0.0f; }
    bool hasStroke() const {
      return state.stroke_opacity > 0.0f && state.stroke_width > 0.0f && !stroke_brush.isNone();
    }

    Bounds boundingFillBox() {
      Bounds bounds;
      if (hasFill())
        bounds = path.boundingBox();

      for (auto& child : children)
        bounds = bounds.unioned(child->boundingFillBox());

      return bounds;
    }

    Bounds boundingStrokeBox() {
      Bounds bounds;
      if (hasStroke())
        bounds = stroke_path.boundingBox();

      for (auto& child : children)
        bounds = bounds.unioned(child->boundingStrokeBox());

      return bounds;
    }

    Bounds boundingBox() { return boundingFillBox().unioned(boundingStrokeBox()); }

    void transformPaths(const Transform& transform) {
      path = path.transformed(transform);
      stroke_path = stroke_path.transformed(transform);
      for (auto& child : children)
        child->transformPaths(transform);
    }

    void applyClipping(const Path* clip_path) {
      if (clip_path == nullptr)
        return;

      if (path.numPoints())
        path = clip_path->combine(path, Path::Operation::Intersection);

      if (stroke_path.numPoints())
        stroke_path = clip_path->combine(stroke_path, Path::Operation::Intersection);

      for (auto& child : children)
        child->applyClipping(clip_path);
    }

    void unionPaths(Path* result) {
      if (path.numPoints())
        *result = result->combine(path, Path::Operation::Union);

      for (auto& child : children)
        child->unionPaths(result);

      path.clear();
      stroke_path.clear();
    }

    void checkPathClipping(const Transform& total_transform, const Bounds& view_box,
                           std::map<std::string, SvgDrawable*>& clip_paths);
    void adjustPaths(const Transform& transform, const Bounds& view_box,
                     std::map<std::string, SvgDrawable*>& clip_paths);
    void setSize(const SvgViewSettings& view) {
      std::map<std::string, SvgDrawable*> clip_paths;
      auto initial_transform = initialTransform(view);
      adjustPaths(initial_transform, view.view_box, clip_paths);
      transformPaths(initial_transform);
    }

    Transform initialTransform(const SvgViewSettings& view) {
      Transform transform;

      float extra_width = 0.0f;
      float extra_height = 0.0f;
      if (view.width > 0 && view.height > 0 && view.view_box.width() > 0 && view.view_box.height() > 0) {
        float scale_x = view.width / view.view_box.width();
        float scale_y = view.height / view.view_box.height();
        if (view.scale == "meet")
          scale_x = scale_y = std::min(scale_x, scale_y);
        else if (view.scale == "slice")
          scale_x = scale_y = std::max(scale_x, scale_y);

        transform = Transform::scale(scale_x, scale_y) *
                    Transform::translation(-view.view_box.x(), -view.view_box.y());
        extra_width = view.width - (view.view_box.width() * scale_x);
        extra_height = view.height - (view.view_box.height() * scale_y);
      }

      if (view.align == "xMidYMid")
        transform = Transform::translation(extra_width / 2, extra_height / 2) * transform;
      else if (view.align == "xMaxYMax")
        transform = Transform::translation(extra_width, extra_height) * transform;
      else if (view.align == "xMinYMax")
        transform = Transform::translation(0, extra_height) * transform;
      else if (view.align == "xMaxYMin")
        transform = Transform::translation(extra_width, 0) * transform;
      else if (view.align == "xMidYMin")
        transform = Transform::translation(extra_width / 2, 0) * transform;
      else if (view.align == "xMidYMax")
        transform = Transform::translation(extra_width / 2, extra_height) * transform;
      else if (view.align == "xMinYMid")
        transform = Transform::translation(0, extra_height / 2) * transform;
      else if (view.align == "xMaxYMid")
        transform = Transform::translation(extra_width, extra_height / 2) * transform;

      return transform;
    }

    std::string id;
    Path::CommandList command_list;
    std::vector<std::unique_ptr<SvgDrawable>> children;
    DrawableState state;
    Brush fill_brush;
    Brush stroke_brush;
    Path path;
    Path stroke_path;

    bool is_clip_path = false;
    std::string clip_path_id;
    Path::CommandList clip_path_commands;
    SvgClipBox clip_box = SvgClipBox::BorderBox;
  };

  class Svg {
  public:
    Svg() = default;

    Svg& operator=(const Svg& other) {
      drawable_ = std::make_unique<SvgDrawable>(*other.drawable_);
      view_ = other.view_;
      return *this;
    }

    Svg(const Svg& other) { *this = other; }

    Svg(const unsigned char* data, int data_size) { parseData(data, data_size); }
    Svg(const EmbeddedFile& file) : Svg(file.data, file.size) { }

    void draw(Canvas& canvas, float x, float y, float width = 0.0f, float height = 0.0f) const {
      if (drawable_)
        drawable_->draw(canvas, x, y, width ? width : view_.width, height ? height : view_.height);
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

    std::unique_ptr<SvgDrawable> computeDrawables(Tag& tag, std::vector<DrawableState>& state_stack);
    void parseStyleAttribute(const std::string& style, DrawableState& state);
    void parseStyleDefinition(const std::string& key, const std::string& value, DrawableState& state);
    void loadDrawableStyle(const Tag& tag, std::vector<DrawableState>& state_stack);
    GradientDef parseGradient(const std::string& color_string);
    void parseClipPath(const Tag& tag, SvgDrawable* drawable);

    void collectDefs(std::vector<Tag>& tags);
    void collectGradients(std::vector<Tag>& tags);
    void resolveUses(std::vector<Tag>& tags);
    void parseCssStyle(std::string& style);
    void loadStyleTags(std::vector<Tag>& tags);

    std::unique_ptr<SvgDrawable> drawable_;
    std::map<std::string, Tag> defs_;
    std::map<std::string, GradientDef> gradients_;
    std::vector<std::pair<CssSelector, std::string>> style_lookup_;
    SvgViewSettings view_;
  };
}