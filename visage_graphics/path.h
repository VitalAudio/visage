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

#include "visage_utils/space.h"

#include <optional>
#include <string>

namespace visage {
  struct SubPath {
    std::vector<Point> points;
    std::vector<float> values;
    bool is_closed = false;
  };

  class Path {
  public:
    static constexpr float kDefaultErrorTolerance = 0.1f;

    enum class FillRule {
      NonZero,
      Positive,
      EvenOdd
    };

    enum class Operation {
      Union,
      Intersection,
      Difference,
      Xor,
    };

    enum class JoinType {
      Round,
      Miter,
      Bevel,
      Square
    };

    enum class EndType {
      Round,
      Square,
      Butt
    };

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

    struct Triangulation {
      std::vector<Point> points;
      std::vector<int> triangles;
    };

    void setPointValue(float value) { current_value_ = value; }

    void moveTo(Point point, bool relative = false) {
      if (!paths_.empty() && paths_.back().points.size() > 1)
        startNewPath();

      if (relative)
        point += last_point_;

      addPoint(point);
      smooth_control_point_ = {};
    }

    void moveTo(float x, float y, bool relative = false) { moveTo(Point(x, y), relative); }

    void lineTo(Point point, bool relative = false) {
      if (currentPath().points.empty())
        addPoint(last_point_);

      if (relative)
        point += last_point_;

      addPoint(point);
      smooth_control_point_ = {};
    }

    void lineTo(float x, float y, bool relative = false) { lineTo(Point(x, y), relative); }

    void verticalTo(float y, bool relative = false) {
      if (relative)
        y += last_point_.y;

      lineTo(last_point_.x, y);
      smooth_control_point_ = {};
    }

    void horizontalTo(float x, bool relative = false) {
      if (relative)
        x += last_point_.x;

      lineTo(x, last_point_.y);
      smooth_control_point_ = {};
    }

    void close() {
      if (paths_.empty() || paths_.back().points.empty())
        return;

      if (paths_.back().points.front() != paths_.back().points.back())
        addPoint(paths_.back().points.front());
      currentPath().is_closed = true;
      startNewPath();
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
      recurseBezierTo(from, control1, control2, end);
    }

    void quadraticTo(float control_x, float control_y, float end_x, float end_y, bool relative = false) {
      quadraticTo(Point(control_x, control_y), Point(end_x, end_y), relative);
    }

    void smoothQuadraticTo(Point end, bool relative = false) {
      if (relative)
        end += last_point_;

      quadraticTo(smooth_control_point_, end);
    }

    void smoothQuadraticTo(float end_x, float end_y, bool relative = false) {
      smoothQuadraticTo(Point(end_x, end_y), relative);
    }

    Point deltaFromLine(const Point& point, const Point& line_from, const Point& line_to) {
      if (line_from == line_to)
        return point - line_from;

      Point line_delta = line_to - line_from;
      Point point_delta = point - line_from;
      float t = point_delta.dot(line_delta) / line_delta.dot(line_delta);
      t = std::clamp(t, 0.0f, 1.0f);
      Point closest_point = line_from + t * line_delta;
      return point - closest_point;
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
    }

    void bezierTo(float x1, float y1, float x2, float y2, float x3, float y3, bool relative = false) {
      bezierTo(Point(x1, y1), Point(x2, y2), Point(x3, y3), relative);
    }

    void smoothBezierTo(Point end_control, Point end, bool relative = false) {
      if (relative) {
        end_control += last_point_;
        end += last_point_;
      }

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

    void parseSvgPath(const std::string& path);
    void addRectangle(float x, float y, float width, float height);
    void addRoundedRectangle(float x, float y, float width, float height, float rx, float ry);
    void addEllipse(float cx, float cy, float rx, float ry);
    void addCircle(float cx, float cy, float r);

    Triangulation triangulate() const;
    Path combine(const Path& other, Operation operation = Operation::Union) const;
    std::pair<Path, Path> offsetAntiAlias(float scale, std::vector<int>& inner_added_points,
                                          std::vector<int>& outer_added_points) const;
    Path offset(float offset, JoinType join_type = JoinType::Square) const;
    Path stroke(float stroke_width, JoinType join_type = JoinType::Round,
                EndType end_type = EndType::Round) const;
    Path breakIntoSimplePolygons() const;

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

    Path transformed(const Matrix& transform) const {
      Path result = *this;
      result.transform(transform);
      return result;
    }

    void transform(const Matrix& transform) {
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
      for (auto& path : paths_) {
        std::reverse(path.points.begin(), path.points.end());
        std::reverse(path.values.begin(), path.values.end());
      }
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

    void setResolutionTransform(const Matrix& transform) {
      resolution_transform_ = transform.withNoTranslation();
    }

  private:
    void recurseBezierTo(Point from, Point control1, Point control2, Point to) {
      float error_squared = error_tolerance_ * error_tolerance_;

      Point delta1 = resolution_transform_ * deltaFromLine(control1, from, to);
      Point delta2 = resolution_transform_ * deltaFromLine(control2, from, to);
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

    Path combine(const Path& other, Path::FillRule fill_rule, int num_cycles_needed, bool reverse_other) const;

    void startNewPath() {
      if (paths_.empty() || !paths_.back().points.empty())
        paths_.emplace_back();

      smooth_control_point_ = {};
      current_value_ = 0.0f;
    }

    SubPath& currentPath() {
      if (paths_.empty())
        paths_.emplace_back();
      return paths_.back();
    }

    void addPoint(const Point& point) {
      last_point_ = point;
      currentPath().points.push_back(point);
      currentPath().values.push_back(current_value_);
    }

    void addPoint(float x, float y) { addPoint({ x, y }); }

    Matrix resolution_transform_;
    std::vector<SubPath> paths_;
    FillRule fill_rule_ = FillRule::EvenOdd;
    Point smooth_control_point_;
    Point last_point_;
    float current_value_ = 0.0f;
    float error_tolerance_ = kDefaultErrorTolerance;
  };
}
