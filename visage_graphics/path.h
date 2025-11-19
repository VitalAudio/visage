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
#include "visage_utils/space.h"

#include <cfloat>
#include <complex>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace visage {
  template<typename T>
  static T orientation(const BasePoint<T>& source, const BasePoint<T>& target1, const BasePoint<T>& target2) {
    static constexpr T kEpsilon = 1.0e-10;
    BasePoint<T> delta1 = target1 - source;
    BasePoint<T> delta2 = target2 - source;
    T l = delta2.y * delta1.x;
    T r = delta2.x * delta1.y;
    T sum = std::abs(l + r);
    T diff = l - r;
    return std::abs(diff) >= kEpsilon * sum ? diff : 0.0;
  }

  template<typename T>
  static T stableOrientation(const BasePoint<T>& source, const BasePoint<T>& target1,
                             const BasePoint<T>& target2) {
    T result = orientation(source, target1, target2);
    if (result != 0.0)
      return result;

    result = orientation(target2, source, target1);
    if (result != 0.0)
      return result;

    return orientation(target1, target2, source);
  }

  class Path {
  public:
    static constexpr float kDefaultErrorTolerance = 0.1f;
    static constexpr float kDefaultMiterLimit = 4.0f;
    static constexpr float kPi = 3.14159265358979323846f;

    struct SubPath {
      std::vector<Point> points;
      bool closed = false;
    };

    struct Command {
      char type;
      Point end;
      Point control1 { FLT_MAX, FLT_MAX };
      Point control2 { FLT_MAX, FLT_MAX };
      int flags = 0;
    };

    struct CommandList : std::vector<Command> {
      enum Flags {
        kLargeArc = 1,
        kSweep = 1 << 1
      };

      Point adjustPoint(const Point& p, bool relative) const { return relative ? current + p : p; }
      Point adjustPoint(float x, float y, bool relative) const {
        return adjustPoint({ x, y }, relative);
      }

      void addCommand(const Command& command) {
        current = command.end;
        push_back(command);
      }

      void moveTo(float x, float y, bool relative = false) {
        addCommand({ 'M', adjustPoint(x, y, relative) });
        start = current;
      }

      void moveTo(Point p, bool relative = false) { moveTo(p.x, p.y, relative); }

      void lineTo(float x, float y, bool relative = false) {
        addCommand({ 'L', adjustPoint(x, y, relative) });
      }

      void horizontalTo(float x, bool relative = false) {
        addCommand({ 'L', { relative ? x + current.x : x, current.y } });
      }

      void verticalTo(float y, bool relative = false) {
        addCommand({ 'L', { current.x, relative ? y + current.y : y } });
      }

      void quadraticTo(float cx, float cy, float x, float y, bool relative = false) {
        addCommand({ 'Q', adjustPoint(x, y, relative), adjustPoint(cx, cy, relative) });
      }

      void smoothQuadraticTo(float x, float y, bool relative = false) {
        addCommand({ 'T', adjustPoint(x, y, relative) });
      }

      void bezierTo(float cx1, float cy1, float cx2, float cy2, float x, float y, bool relative = false) {
        addCommand({ 'C', adjustPoint(x, y, relative), adjustPoint(cx1, cy1, relative),
                     adjustPoint(cx2, cy2, relative) });
      }

      void smoothBezierTo(float cx, float cy, float x, float y, bool relative = false) {
        addCommand({ 'S', adjustPoint(x, y, relative), adjustPoint(cx, cy, relative) });
      }

      void arcTo(float rx, float ry, float rotation, bool large_arc, bool sweep, float x, float y,
                 bool relative = false) {
        int flags = (large_arc ? kLargeArc : 0) | (sweep ? kSweep : 0);
        addCommand({ 'A', adjustPoint(x, y, relative), { rx, ry }, { rotation, rotation }, flags });
      }

      void arcTo(float rx, float ry, float rotation, bool large_arc, bool sweep, Point p,
                 bool relative = false) {
        arcTo(rx, ry, rotation, large_arc, sweep, p.x, p.y, relative);
      }

      void close() { addCommand({ 'Z', start }); }

      void addRectangle(float x, float y, float width, float height);
      void addRoundedRectangle(float x, float y, float width, float height, float rx_top_left,
                               float ry_top_left, float rx_top_right, float ry_top_right,
                               float rx_bottom_left, float ry_bottom_left, float rx_bottom_right,
                               float ry_bottom_right);
      void addRoundedRectangle(float x, float y, float width, float height, float rx, float ry);
      void addEllipse(float cx, float cy, float rx, float ry);
      void addCircle(float cx, float cy, float r);

      Point direction(int index) const {
        auto check_delta = [](const Point& current, const Point& check) {
          return check != Point(FLT_MAX, FLT_MAX) && check != current;
        };

        index = std::clamp(index, 0, static_cast<int>(size()) - 1);
        Point command = at(index).end;
        Point prev_point = command;
        for (int i = index; i >= 0 && prev_point == command; --i) {
          const auto& prev = at(i);
          if (check_delta(command, prev.end))
            prev_point = prev.end;
          else if (check_delta(command, prev.control2))
            prev_point = prev.control2;
          else if (check_delta(command, prev.control1))
            prev_point = prev.control1;
        }

        Point next_point = command;
        for (int i = index + 1; i < size() && next_point == command; ++i) {
          const auto& next = at(i);
          if (check_delta(command, next.control1))
            next_point = next.control1;
          else if (check_delta(command, next.control2))
            next_point = next.control2;
          else if (check_delta(command, next.end))
            next_point = next.end;
        }

        Point prev_direction = (command - prev_point).normalized();
        Point next_direction = (next_point - command).normalized();
        Point direction = prev_direction + next_direction;
        if (direction == Point(0.0f, 0.0f))
          return { -prev_direction.y, prev_direction.x };

        return (prev_direction + next_direction).normalized();
      }

      Point start;
      Point current;
    };

    enum class FillRule {
      NonZero,
      Positive,
      EvenOdd
    };

    enum class ControlPoints {
      Linear,
      Quadratic,
      Cubic
    };

    enum class Join {
      Round,
      Miter,
      Bevel,
      Square
    };

    enum class EndCap {
      Round,
      Square,
      Butt
    };

    Path() = default;
    Path(const Path& other) = default;
    Path& operator=(const Path& other) = default;

    template<typename T>
    static std::optional<T> findIntersection(T start1, T end1, T start2, T end2) {
      auto delta1 = end1 - start1;
      auto delta2 = end2 - start2;
      auto det = delta1.cross(delta2);
      if (det == 0.0)
        return std::nullopt;

      auto start_delta = start2 - start1;
      auto t1 = start_delta.cross(delta2) / det;
      return start1 + delta1 * t1;
    }

    static CommandList parseSvgPath(const std::string& path);

    void moveTo(Point point, bool relative = false) {
      if (!paths_.empty() && !paths_.back().points.empty())
        startNewPath();

      if (relative)
        point += last_point_;

      last_point_ = point;
    }

    void moveTo(float x, float y, bool relative = false) { moveTo(Point(x, y), relative); }

    void lineTo(Point point, bool relative = false) {
      if (currentPath().points.empty())
        addPoint(last_point_);

      if (relative)
        point += last_point_;

      addPoint(point);
    }

    void lineTo(float x, float y, bool relative = false) { lineTo(Point(x, y), relative); }

    void verticalTo(float y, bool relative = false) {
      if (relative)
        y += last_point_.y;

      lineTo(last_point_.x, y);
    }

    void horizontalTo(float x, bool relative = false) {
      if (relative)
        x += last_point_.x;

      lineTo(x, last_point_.y);
    }

    void close() {
      static constexpr float kCloseEpsilon = 0.000001f;

      if (paths_.empty() || paths_.back().points.empty())
        return;

      if ((paths_.back().points.front() - paths_.back().points.back()).squareMagnitude() < kCloseEpsilon) {
        paths_.back().points.back() = paths_.back().points.front();
        last_point_ = paths_.back().points.front();
      }
      else if (paths_.back().points.front() != paths_.back().points.back())
        addPoint(paths_.back().points.front());

      currentPath().closed = true;
    }

    void quadraticTo(Point control, Point end, bool relative = false) {
      if (currentPath().points.empty())
        addPoint(last_point_);

      Point from = last_point_;
      if (relative) {
        control += from;
        end += from;
      }

      Point control1 = from + (2.0f / 3.0f) * (control - from);
      Point control2 = end + (2.0f / 3.0f) * (control - end);
      smooth_control_point_ = end + (end - control);
      recurseBezierTo(from, control1, control2, end);
      current_control_points_ = ControlPoints::Quadratic;
    }

    void quadraticTo(float control_x, float control_y, float end_x, float end_y, bool relative = false) {
      quadraticTo(Point(control_x, control_y), Point(end_x, end_y), relative);
    }

    void smoothQuadraticTo(Point end, bool relative = false) {
      if (current_control_points_ != ControlPoints::Quadratic)
        smooth_control_point_ = last_point_;

      if (relative)
        end += last_point_;

      quadraticTo(smooth_control_point_, end);
    }

    void smoothQuadraticTo(float end_x, float end_y, bool relative = false) {
      smoothQuadraticTo(Point(end_x, end_y), relative);
    }

    void bezierTo(Point control1, Point control2, Point end, bool relative = false) {
      if (currentPath().points.empty())
        addPoint(last_point_);

      Point from = last_point_;
      if (relative) {
        control1 += from;
        control2 += from;
        end += from;
      }

      recurseBezierTo(from, control1, control2, end);
      smooth_control_point_ = end + (end - control2);
      current_control_points_ = ControlPoints::Cubic;
    }

    void bezierTo(float x1, float y1, float x2, float y2, float x3, float y3, bool relative = false) {
      bezierTo(Point(x1, y1), Point(x2, y2), Point(x3, y3), relative);
    }

    void smoothBezierTo(Point end_control, Point end, bool relative = false) {
      if (relative) {
        end_control += last_point_;
        end += last_point_;
      }
      if (current_control_points_ != ControlPoints::Cubic)
        smooth_control_point_ = end_control;

      bezierTo(smooth_control_point_, end_control, end);
    }

    void smoothBezierTo(float end_control_x, float end_control_y, float end_x, float end_y,
                        bool relative = false) {
      smoothBezierTo(Point(end_control_x, end_control_y), Point(end_x, end_y), relative);
    }

    void arcTo(float rx, float ry, float x_axis_rotation, bool large_arc, bool sweep_flag,
               Point point, bool relative = false);

    int numPoints() const {
      int count = 0;
      for (const auto& path : paths_)
        count += path.points.size();
      return count;
    }

    std::vector<SubPath>& subPaths() { return paths_; }
    const std::vector<SubPath>& subPaths() const { return paths_; }

    void clear() {
      paths_.clear();
      last_point_ = {};
    }

    void loadSvgPath(const std::string& path);
    void loadCommands(const CommandList& commands);
    void addRectangle(float x, float y, float width, float height);
    void addRoundedRectangle(float x, float y, float width, float height, float rx_top_left,
                             float ry_top_left, float rx_top_right, float ry_top_right, float rx_bottom_right,
                             float ry_bottom_right, float rx_bottom_left, float ry_bottom_left);
    void addRoundedRectangle(float x, float y, float width, float height, float rx, float ry);
    void addRoundedRectangle(float x, float y, float width, float height, float r) {
      addRoundedRectangle(x, y, width, height, r, r);
    }

    void addEllipse(float cx, float cy, float rx, float ry);
    void addCircle(float cx, float cy, float r);

    Path combine(Path& other, FillRule fill_rule = FillRule::EvenOdd) const;
    SubPath singlePointOffset(Point point, float amount, EndCap end_cap);
    Path offset(float amount, Join join = Join::Square, EndCap end_cap = EndCap::Butt,
                float miter_limit = kDefaultMiterLimit);
    Path stroke(float stroke_width, Join join = Join::Round, EndCap end_cap = EndCap::Round,
                std::vector<float> dash_array = {}, float dash_offset = 0.0f,
                float miter_limit = kDefaultMiterLimit);

    Path scaled(float mult) const {
      Path result = *this;
      result.scale(mult);
      return result;
    }

    void scale(float mult) {
      for (auto& path : paths_) {
        for (Point& point : path.points)
          point *= mult;
      }
    }

    Path translated(const Point& offset) const {
      Path result = *this;
      result.translate(offset);
      return result;
    }

    Path translated(float x, float y) const { return translated(Point(x, y)); }

    void translate(const Point& offset) {
      for (auto& path : paths_) {
        for (Point& point : path.points)
          point += offset;
      }
    }

    void translate(float x, float y) { translate(Point(x, y)); }

    void rotate(float angle) {
      Point row1 = { cosf(angle), sinf(angle) };
      Point row2 = { -sinf(angle), cosf(angle) };
      for (auto& path : paths_) {
        for (Point& point : path.points) {
          float x = point.x;
          float y = point.y;
          point.x = row1.x * x + row1.y * y;
          point.y = row2.x * x + row2.y * y;
        }
      }
    }

    Path rotated(float angle) const {
      Path result = *this;
      result.rotate(angle);
      return result;
    }

    Path transformed(const Transform& transform) const {
      Path result = *this;
      result.transform(transform);
      return result;
    }

    void transform(const Transform& transform) {
      for (auto& path : paths_) {
        for (Point& point : path.points)
          point = transform * point;
      }
    }

    Path reversed() const {
      Path reversed_path = *this;
      reversed_path.reverse();
      return reversed_path;
    }

    void reverse() {
      for (auto& path : paths_)
        std::reverse(path.points.begin(), path.points.end());
    }

    void setFillRule(FillRule fill_rule) { fill_rule_ = fill_rule; }
    FillRule fillRule() const { return fill_rule_; }

    void setErrorTolerance(float tolerance) {
      VISAGE_ASSERT(tolerance > 0.0f);
      if (tolerance > 0.0f)
        error_tolerance_ = tolerance;
    }

    Bounds boundingBox() const {
      float min_x = std::numeric_limits<float>::max();
      float min_y = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float max_y = std::numeric_limits<float>::lowest();
      for (const auto& path : paths_) {
        for (const auto& point : path.points) {
          min_x = std::min(min_x, point.x);
          min_y = std::min(min_y, point.y);
          max_x = std::max(max_x, point.x);
          max_y = std::max(max_y, point.y);
        }
      }
      if (min_x > max_x || min_y > max_y)
        return { 0, 0, 0, 0 };
      return { min_x, min_y, max_x - min_x, max_y - min_y };
    }

    float errorTolerance() const { return error_tolerance_; }
    float length() const {
      float total_length = 0.0f;
      for (const auto& path : paths_) {
        for (size_t i = 1; i < path.points.size(); ++i)
          total_length += (path.points[i] - path.points[i - 1]).length();
        if (path.closed && path.points.size() > 2)
          total_length += (path.points.front() - path.points.back()).length();
      }
      return total_length;
    }

    void setResolutionMatrix(const Matrix& matrix) { resolution_matrix_ = matrix; }
    const Matrix& resolutionMatrix() const { return resolution_matrix_; }

  private:
    Path offset(float amount, Join join, Join inner_join, EndCap end_cap, float miter_limit);

    static Point deltaFromLine(const Point& point, const Point& line_from, const Point& line_to) {
      if (line_from == line_to)
        return point - line_from;

      Point line_delta = line_to - line_from;
      Point point_delta = point - line_from;
      float t = point_delta.dot(line_delta) / line_delta.dot(line_delta);
      t = std::clamp(t, 0.0f, 1.0f);
      Point closest_point = line_from + t * line_delta;
      return point - closest_point;
    }

    void recurseBezierTo(const Point& from, const Point& control1, const Point& control2, const Point& to) {
      float error_squared = error_tolerance_ * error_tolerance_;

      Point delta1 = resolution_matrix_ * deltaFromLine(control1, from, to);
      Point delta2 = resolution_matrix_ * deltaFromLine(control2, from, to);
      if (delta1.squareMagnitude() <= error_squared && delta2.squareMagnitude() <= error_squared) {
        addPoint(to);
        return;
      }

      Point mid1 = (from + control1) * 0.5f;
      Point mid2 = (control1 + control2) * 0.5f;
      Point mid3 = (control2 + to) * 0.5f;

      Point midmid1 = (mid1 + mid2) * 0.5f;
      Point midmid2 = (mid2 + mid3) * 0.5f;

      Point break_point = (midmid1 + midmid2) * 0.5f;

      recurseBezierTo(from, mid1, midmid1, break_point);
      recurseBezierTo(break_point, midmid2, mid3, to);
    }

    void startNewPath() {
      if (paths_.empty() || !paths_.back().points.empty())
        paths_.emplace_back();

      current_control_points_ = ControlPoints::Linear;
    }

    SubPath& currentPath() {
      if (paths_.empty() || paths_.back().closed)
        paths_.emplace_back();
      return paths_.back();
    }

    void addPoint(const Point& point) {
      if (!currentPath().points.empty() && point == currentPath().points.back())
        return;

      last_point_ = point;
      currentPath().points.push_back(point);
      current_control_points_ = ControlPoints::Linear;
    }

    void addPoint(float x, float y) { addPoint({ x, y }); }

    Matrix resolution_matrix_;
    std::vector<SubPath> paths_;
    FillRule fill_rule_ = FillRule::EvenOdd;
    Point smooth_control_point_;
    ControlPoints current_control_points_ = ControlPoints::Linear;
    Point last_point_;
    float error_tolerance_ = kDefaultErrorTolerance;
  };

  struct PathAtlasTexture;

  class PathAtlas {
  public:
    static constexpr int kBuffer = 1;

    struct PackedPathRect {
      explicit PackedPathRect(const Path& p) : path(p) { }

      Path path;
      int x = 0;
      int y = 0;
      int w = 0;
      int h = 0;
      bool needs_update = true;
    };

    struct PackedPathReference {
      PackedPathReference(std::weak_ptr<PathAtlas*> atlas, const PackedPathRect* packed_path_rect) :
          atlas(std::move(atlas)), packed_path_rect(packed_path_rect) { }
      ~PackedPathReference();

      std::weak_ptr<PathAtlas*> atlas;
      const PackedPathRect* packed_path_rect = nullptr;
    };

    class PackedPath {
    public:
      explicit PackedPath(std::shared_ptr<PackedPathReference> reference) :
          reference_(std::move(reference)) { }

      PackedPath() = default;
      ~PackedPath() = default;

      int x() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_path_rect->x;
      }

      int y() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_path_rect->y;
      }

      int w() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_path_rect->w;
      }

      int h() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_path_rect->h;
      }

      const Path& path() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_path_rect->path;
      }

      const PackedPathRect* packedImageRect() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_path_rect;
      }

    private:
      std::shared_ptr<PackedPathReference> reference_;
    };

    PathAtlas();
    ~PathAtlas();

    PackedPath addPath(const Path& path, int width, int height) {
      std::unique_ptr<PackedPathRect> packed_path_rect = std::make_unique<PackedPathRect>(path);
      if (!atlas_map_.addRect(packed_path_rect.get(), width, height))
        needs_packing_ = true;

      const PackedRect& rect = atlas_map_.rectForId(packed_path_rect.get());
      packed_path_rect->x = rect.x;
      packed_path_rect->y = rect.y;
      packed_path_rect->w = rect.w;
      packed_path_rect->h = rect.h;
      auto packed_rect = packed_path_rect.get();
      paths_.push_back(std::move(packed_path_rect));

      auto reference = std::make_shared<PackedPathReference>(reference_, packed_rect);
      references_[packed_rect] = reference;
      return PackedPath(reference);
    }

    int updatePaths(int submit_pass);
    void destroy();
    int width() const { return width_; }
    int height() const { return height_; }

    const bgfx::FrameBufferHandle& frameBufferHandle();

    void removePath(const PackedPathRect* packed_path_rect) {
      atlas_map_.removeRect(packed_path_rect);
      references_.erase(packed_path_rect);
      paths_.erase(std::remove_if(paths_.begin(), paths_.end(),
                                  [&](const std::unique_ptr<PackedPathRect>& ptr) {
                                    return ptr.get() == packed_path_rect;
                                  }),
                   paths_.end());
    }

    static void setPathAtlasCoordinates(TextureVertex* vertices, const PackedPath& rect) {
      float left = rect.x();
      float top = rect.y();
      float right = left + rect.w();
      float bottom = top + rect.h();

      vertices[0].texture_x = left;
      vertices[0].texture_y = top;
      vertices[1].texture_x = right;
      vertices[1].texture_y = top;
      vertices[2].texture_x = left;
      vertices[2].texture_y = bottom;
      vertices[3].texture_x = right;
      vertices[3].texture_y = bottom;

      for (int i = 0; i < kVerticesPerQuad; ++i) {
        vertices[i].direction_x = 1.0f;
        vertices[i].direction_y = 0.0f;
      }
    }

  private:
    bool clearUpdatedPathAreas(int submit_pass);
    void checkInit();
    void resize();

    std::map<const PackedPathRect*, std::weak_ptr<PackedPathReference>> references_;
    std::vector<std::unique_ptr<PackedPathRect>> paths_;
    PackedAtlasMap<const PackedPathRect*> atlas_map_;
    std::unique_ptr<PathAtlasTexture> frame_buffer_;
    int width_ = 0;
    int height_ = 0;
    bool needs_packing_ = false;
    std::shared_ptr<PathAtlas*> reference_;

    VISAGE_LEAK_CHECKER(PathAtlas)
  };
}
