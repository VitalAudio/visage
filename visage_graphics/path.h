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
  };

  class Path {
  public:
    enum class FillRule {
      NonZero,
      EvenOdd
    };

    static constexpr int kMaxCurveResolution = 32;

    template<typename T>
    static std::optional<T> findIntersection(T start1, T end1, T start2, T end2) {
      if (start1 == start2 || end1 == end2)
        return std::nullopt;

      auto delta1 = end1 - start1;
      auto delta2 = end2 - start2;
      auto det = delta1.cross(delta2);
      if (det == 0.0)
        return std::nullopt;

      auto start_delta = start2 - start1;
      auto t1 = start_delta.cross(delta2) / det;
      auto t2 = start_delta.cross(delta1) / det;

      if (t1 < 0 || t2 < 0 || t1 > 1 || t2 > 1)
        return std::nullopt;

      return start1 + delta1 * t1;
    }

    struct Triangulation {
      std::vector<Point> points;
      std::vector<int> triangles;
    };

    void setPointValue(float value) { current_value_ = value; }

    Point lastPoint() const {
      if (paths_.empty() || paths_.back().points.empty())
        return { 0.0f, 0.0f };

      return paths_.back().points.back();
    }

    void moveTo(Point point, bool relative = false) {
      if (relative)
        point += lastPoint();

      addPoint(point);
      smooth_control_point_ = {};
    }

    void moveTo(float x, float y, bool relative = false) { moveTo(Point(x, y), relative); }

    void lineTo(Point point, bool relative = false) {
      if (relative)
        point += lastPoint();

      addPoint(point);
      smooth_control_point_ = {};
    }

    void lineTo(float x, float y, bool relative = false) { lineTo(Point(x, y), relative); }

    void verticalTo(float y, bool relative = false) {
      Point last_point = lastPoint();
      if (relative)
        y += lastPoint().y;

      lineTo(last_point.x, y);
      smooth_control_point_ = {};
    }

    void horizontalTo(float x, bool relative = false) {
      Point last_point = lastPoint();
      if (relative)
        x += last_point.x;

      lineTo(x, last_point.y);
      smooth_control_point_ = {};
    }

    void close() {
      if (!paths_.empty())
        addPoint(paths_.back().points.front());
      paths_.emplace_back();
      smooth_control_point_ = {};
      current_value_ = 0.0f;
    }

    void quadraticTo(Point control, Point end, bool relative = false) {
      Point p0 = lastPoint();
      if (relative) {
        control += p0;
        end += p0;
      }

      for (int i = 1; i <= kMaxCurveResolution; ++i) {
        float t = i / static_cast<float>(kMaxCurveResolution);
        float u = 1 - t;
        Point point = u * u * p0 + 2 * u * t * control + t * t * end;
        addPoint(point);
      }

      smooth_control_point_ = end + (end - control);
    }

    void quadraticTo(float control_x, float control_y, float end_x, float end_y, bool relative = false) {
      quadraticTo(Point(control_x, control_y), Point(end_x, end_y), relative);
    }

    void smoothQuadraticTo(Point end, bool relative = false) {
      if (relative)
        end += lastPoint();

      quadraticTo(smooth_control_point_, end);
    }

    void smoothQuadraticTo(float end_x, float end_y, bool relative = false) {
      smoothQuadraticTo(Point(end_x, end_y), relative);
    }

    void bezierTo(Point control1, Point control2, Point end, bool relative = false) {
      Point p0 = lastPoint();
      if (relative) {
        control1 += p0;
        control2 += p0;
        end += p0;
      }

      for (int i = 1; i <= kMaxCurveResolution; ++i) {
        float t = i / static_cast<float>(kMaxCurveResolution);
        float u = 1 - t;
        Point point = u * u * u * p0 + 3 * u * u * t * control1 + 3 * u * t * t * control2 + t * t * t * end;
        addPoint(point);
      }

      smooth_control_point_ = end + (end - control2);
    }

    void bezierTo(float x1, float y1, float x2, float y2, float x3, float y3, bool relative = false) {
      bezierTo(Point(x1, y1), Point(x2, y2), Point(x3, y3), relative);
    }

    void smoothBezierTo(Point end_control, Point end, bool relative = false) {
      if (relative) {
        end_control += lastPoint();
        end += lastPoint();
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

    const std::vector<SubPath>& subPaths() const { return paths_; }

    void clear() { paths_.clear(); }

    void parseSvgPath(const std::string& path);
    Triangulation triangulate() const;

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
      Point row1 = { cos(angle), sin(angle) };
      Point row2 = { -sin(angle), cos(angle) };
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

  private:
    SubPath& currentPath() {
      if (paths_.empty())
        paths_.emplace_back();
      return paths_.back();
    }

    void addPoint(const Point& point) {
      currentPath().points.push_back(point);
      currentPath().values.push_back(current_value_);
    }

    void addPoint(float x, float y) {
      currentPath().points.emplace_back(x, y);
      currentPath().values.push_back(current_value_);
    }

    std::vector<SubPath> paths_;
    FillRule fill_rule_ = FillRule::EvenOdd;
    Point smooth_control_point_;
    float current_value_ = 0.0f;
  };
}
