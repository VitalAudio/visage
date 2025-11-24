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

#include "visage_file_embed/embedded_file.h"
#include "visage_graphics/gradient.h"
#include "visage_graphics/path.h"
#include "visage_utils/clone_ptr.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace visage {
  class Canvas;
  struct Marker;

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
    enum class Type {
      None,
      Solid,
      Linear,
      Radial,
      CurrentColor,
      ContextFill,
      ContextStroke
    };

    GradientDef() = default;
    GradientDef(Type t) : type(t) { }
    GradientDef(const Color& color) {
      gradient = Gradient(color);
      type = Type::Solid;
    }

    Brush toBrush(Bounds box) const {
      if (type == Type::Solid)
        return Brush::solid(gradient.colors().front());

      GradientPosition position;
      if (type == Type::Radial) {
        float f_radius = radius ? focal_radius / radius : 0.0f;
        position = GradientPosition::radial(point1, radius, radius, point2, f_radius);
      }
      else
        position = GradientPosition::linear(point1, point2);

      Transform scale_transform;
      if (!user_space) {
        scale_transform = Transform::translation(box.x(), box.y()) *
                          Transform::scale(box.width(), box.height());
      }
      position = position.transformed(scale_transform * transform);
      return { gradient, position };
    }

    bool isNone() const { return gradient.isNone(); }
    bool isCurrentColor() const { return type == Type::CurrentColor; }

    Gradient gradient;
    Transform transform;
    Type type = Type::None;
    bool user_space = false;
    Point point1 = Point(0.0f, 0.0f);
    Point point2 = Point(1.0f, 0.0f);
    float focal_radius = 0.0f;
    float radius = 0.5f;
  };

  struct DrawableState {
    GradientDef current_color = GradientDef(0xff000000);
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
    bool non_scaling_stroke = false;
    float stroke_miter_limit = 4.0f;

    bool visible = true;
  };

  struct SvgViewSettings {
    float width = 0.0f;
    float height = 0.0f;
    Bounds view_box;
    std::string align;
    std::string scale;
  };

  struct SvgDrawable {
    struct ColorContext {
      const Brush* current_color = nullptr;
      const Brush* fill_color = nullptr;
      const Brush* stroke_color = nullptr;
    };

    SvgDrawable() = default;
    SvgDrawable(const SvgDrawable& other) = default;
    SvgDrawable& operator=(const SvgDrawable& other) = default;

    void draw(Canvas& canvas, ColorContext* context, float x, float y, float width, float height) const;
    void drawAll(Canvas& canvas, ColorContext* context, float x, float y, float width, float height) const;
    bool setContextColor(Canvas& canvas, const ColorContext* context, const GradientDef& gradient,
                         float color_opacity) const;
    void fill(Canvas& canvas, ColorContext* context, float x, float y, float width, float height) const;
    void stroke(Canvas& canvas, ColorContext* context, float x, float y, float width, float height) const;
    bool hasFill() const { return !fill_brush.isNone() && state.fill_opacity > 0.0f; }
    bool hasStroke() const {
      return state.stroke_opacity > 0.0f && state.stroke_width > 0.0f && !stroke_brush.isNone();
    }

    void setAllFillBrush(const Brush& brush) {
      if ((!state.fill_gradient.isNone() || state.fill_gradient.isCurrentColor()) &&
          state.fill_opacity > 0.0f) {
        fill_brush = brush;
      }
      for (auto& child : children)
        child->setAllFillBrush(brush);
    }

    void setAllStrokeBrush(const Brush& brush) {
      if ((!state.stroke_gradient.isNone() || state.stroke_gradient.isCurrentColor()) &&
          state.stroke_opacity > 0.0f) {
        stroke_brush = brush;
      }
      for (auto& child : children)
        child->setAllStrokeBrush(brush);
    }

    void setAllCurrentColor(const Brush& brush) {
      if (state.fill_gradient.isCurrentColor() && state.fill_opacity > 0.0f)
        fill_brush = brush;
      if (state.stroke_gradient.isCurrentColor() && state.stroke_opacity > 0.0f)
        stroke_brush = brush;
      for (auto& child : children)
        child->setAllCurrentColor(brush);
    }

    Bounds boundingFillBox() const {
      Bounds bounds;
      if (hasFill())
        bounds = path.boundingBox();

      for (auto& child : children)
        bounds = bounds.unioned(child->boundingFillBox());

      return bounds;
    }

    Bounds boundingStrokeBox() const {
      Bounds bounds;
      if (hasStroke())
        bounds = stroke_path.boundingBox();

      for (auto& child : children)
        bounds = bounds.unioned(child->boundingStrokeBox());

      return bounds;
    }

    Bounds boundingBox() const { return boundingFillBox().unioned(boundingStrokeBox()); }

    void transformPaths(const Transform& transform) {
      post_bounding_box = {};

      if (path.numPoints()) {
        path = path.transformed(transform);
        fill_brush.transform(transform);
        post_bounding_box = path.boundingBox();

        if (stroke_path.numPoints()) {
          stroke_path = stroke_path.transformed(transform);
          stroke_brush.transform(transform);
          post_bounding_box = post_bounding_box.unioned(stroke_path.boundingBox());
        }
      }
      for (auto& child : children) {
        child->transformPaths(transform);
        post_bounding_box = post_bounding_box.unioned(child->post_bounding_box);
      }
      for (auto& clip_path : clipping_paths)
        clip_path = clip_path.transformed(transform);
    }

    void gatherPaths(std::vector<Path>& paths) {
      if (path.numPoints())
        paths.push_back(path);

      for (auto& child : children)
        child->gatherPaths(paths);

      path.clear();
      stroke_path.clear();
    }

    void checkPathClipping(const Matrix& scale_matrix, const Bounds& view_box,
                           std::map<std::string, SvgDrawable*>& clip_paths);
    void initPaths(const Matrix& scale_matrix, const Bounds& view_box);
    void adjustPaths(const Matrix& scale_matrix, const Bounds& view_box,
                     std::map<std::string, SvgDrawable*>& clip_paths);
    void strokePath(const Matrix& scale_matrix, const Bounds& view_box);
    void setSize(const SvgViewSettings& view, float width, float height, float scale = 1.0f) {
      if (width == 0.0f || height == 0.0f)
        return;

      std::map<std::string, SvgDrawable*> clip_paths;
      auto initial_transform = initialTransform(view, width, height);
      auto dpi_scale = Matrix::scale(scale, scale);
      initPaths(dpi_scale * initial_transform.matrix, view.view_box);
      adjustPaths(dpi_scale * initial_transform.matrix, view.view_box, clip_paths);
      transformPaths(initial_transform);
    }

    void setSize(const SvgViewSettings& view) { setSize(view, view.width, view.height); }

    Transform initialTransform(const SvgViewSettings& view, float width, float height) {
      Transform transform;

      float extra_width = 0.0f;
      float extra_height = 0.0f;
      if (width > 0 && height > 0 && view.view_box.width() > 0 && view.view_box.height() > 0) {
        float scale_x = width / view.view_box.width();
        float scale_y = height / view.view_box.height();
        if (view.scale == "meet")
          scale_x = scale_y = std::min(scale_x, scale_y);
        else if (view.scale == "slice")
          scale_x = scale_y = std::max(scale_x, scale_y);

        transform = Transform::scale(scale_x, scale_y) *
                    Transform::translation(-view.view_box.x(), -view.view_box.y());
        extra_width = width - view.view_box.width() * scale_x;
        extra_height = height - view.view_box.height() * scale_y;
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
    bool is_defines = false;
    Path::CommandList command_list;
    std::vector<clone_ptr<SvgDrawable>> children;

    Transform local_transform;
    bool transform_origin_x_ratio = false;
    bool transform_origin_y_ratio = false;
    float transform_origin_x = 0.0f;
    float transform_origin_y = 0.0f;

    float opacity = 1.0f;
    DrawableState state;
    Brush fill_brush;
    Brush stroke_brush;
    Path path;
    Path stroke_path;
    Bounds post_bounding_box;
    std::vector<Path> clipping_paths;
    Marker* marker_start = nullptr;
    Marker* marker_mid = nullptr;
    Marker* marker_end = nullptr;

    bool is_clip_path = false;
    bool is_clip_bounding_box = false;
    std::string clip_path_shape;
  };

  struct Marker {
    SvgDrawable drawable;
    bool reverse_start_marker = false;
    bool use_angle = false;
    float marker_angle = 0.0f;
  };

  class SvgParser {
  public:
    static std::unique_ptr<SvgDrawable> loadDrawable(const unsigned char* data, int data_size,
                                                     SvgViewSettings& view) {
      SvgParser parser(data, data_size);
      view = parser.view_;
      return std::move(parser.drawable_);
    }

  private:
    SvgParser() = default;

    SvgParser(const unsigned char* data, int data_size) { parseData(data, data_size); }
    explicit SvgParser(const EmbeddedFile& file) : SvgParser(file.data, file.size) { }

    void parseData(const unsigned char* data, int data_size);

    std::unique_ptr<SvgDrawable> computeDrawables(Tag& tag, std::vector<DrawableState>& state_stack);
    void parseStyleAttribute(const std::string& style, DrawableState& state, SvgDrawable* drawable);
    void parseStyleDefinition(const std::string& key, const std::string& value,
                              DrawableState& state, SvgDrawable* drawable);
    void loadDrawableTransform(const Tag& tag, SvgDrawable* drawable);
    bool loadDrawable(const Tag& tag, SvgDrawable* drawable) const;

    void loadDrawableStyle(const Tag& tag, std::vector<DrawableState>& state_stack, SvgDrawable* drawable);
    GradientDef parseGradient(const std::string& color_string);

    void collectDefs(std::vector<Tag>& tags);
    void collectGradients(std::vector<Tag>& tags);
    void collectMarkers(std::vector<Tag>& tags);
    void resolveUses(std::vector<Tag>& tags);
    void parseCssStyle(const std::string& style);
    void loadStyleTags(std::vector<Tag>& tags);

    std::unique_ptr<SvgDrawable> drawable_;
    std::map<std::string, Tag> defs_;
    std::map<std::string, GradientDef> gradients_;
    std::map<std::string, std::unique_ptr<Marker>> markers_;
    std::vector<std::pair<CssSelector, std::string>> style_lookup_;
    SvgViewSettings view_;
    float draw_width_ = 0.0f;
    float draw_height_ = 0.0f;
  };

  class Svg {
  public:
    Svg() = default;
    Svg(const Svg& other) = default;
    Svg& operator=(const Svg& other) = default;

    Svg(const unsigned char* data, int data_size) {
      drawable_ = clone_ptr(SvgParser::loadDrawable(data, data_size, view_));
    }

    explicit Svg(const EmbeddedFile& file) : Svg(file.data, file.size) { }

    void setDimensions(int width, int height, float scale) {
      setDrawableDimensions(width, height, scale);
    }

    SvgDrawable* drawable() const { return drawable_.get(); }

    float width() const { return draw_width_; }
    float height() const { return draw_height_; }

    void setFillBrush(const Brush& brush) {
      fill_brush_ = brush;
      if (drawable_)
        drawable_->setAllFillBrush(brush);
    }

    void resetFillBrush() {
      fill_brush_ = Brush::none();
      resetDrawable();
    }

    void setStrokeBrush(const Brush& brush) {
      stroke_brush_ = brush;
      if (drawable_)
        drawable_->setAllStrokeBrush(brush);
    }

    void resetStrokeBrush() {
      stroke_brush_ = Brush::none();
      resetDrawable();
    }

    void setCurrentColor(const Brush& brush) {
      current_color_ = brush;
      if (drawable_)
        drawable_->setAllCurrentColor(brush);
    }

  private:
    void setDrawableDimensions(int width, int height, float scale) {
      if (width != draw_width_ || height != draw_height_) {
        draw_width_ = width;
        draw_height_ = height;
        draw_scale_ = scale;
        resetDrawable();
      }
    }

    void resetDrawable() {
      if (!drawable_)
        return;

      drawable_->setSize(view_, draw_width_, draw_height_, draw_scale_);
      if (!fill_brush_.isNone())
        drawable_->setAllFillBrush(fill_brush_);
      if (!stroke_brush_.isNone())
        drawable_->setAllStrokeBrush(stroke_brush_);
      if (!current_color_.isNone())
        drawable_->setAllCurrentColor(current_color_);
    }

    SvgViewSettings view_;
    clone_ptr<SvgDrawable> drawable_;
    float draw_width_ = 0.0f;
    float draw_height_ = 0.0f;
    float draw_scale_ = 1.0f;

    Brush fill_brush_;
    Brush stroke_brush_;
    Brush current_color_;
  };
}