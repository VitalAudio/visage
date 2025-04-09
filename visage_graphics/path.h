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
  class Path {
  public:
    static std::optional<Point> findIntersection(Point start1, Point end1, Point start2, Point end2) {
      if (start1 == start2 || end1 == end2)
        return std::nullopt;

      Point delta1 = end1 - start1;
      Point delta2 = end2 - start2;
      float det = delta1.cross(delta2);
      if (det == 0.0f)
        return std::nullopt;

      Point start_delta = start2 - start1;
      float t1 = start_delta.cross(delta2) / det;
      float t2 = start_delta.cross(delta1) / det;

      if (t1 <= 0.0f || t2 <= 0.0f || t1 >= 1.0f || t2 >= 1.0f)
        return std::nullopt;

      return start1 + delta1 * t1;
    }

    struct Triangulation {
      std::vector<Point> points;
      std::vector<int> triangles;
    };

    static constexpr int kMaxCurveResolution = 32;

    Point lastPoint() const {
      if (paths_.empty() || paths_.back().empty())
        return Point(0.0f, 0.0f);

      return paths_.back().back();
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

    void closePath() {
      paths_.emplace_back();
      smooth_control_point_ = {};
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
        count += path.size();
      return count;
    }

    const std::vector<std::vector<Point>>& subPaths() const { return paths_; }

    void clear() {
      paths_.clear();
      values_.clear();
    }

    void parseSvgPath(const std::string& path);
    Triangulation triangulate() const;

    Path scaled(float mult) const {
      Path result = *this;
      result.scale(mult);
      return result;
    }

    void scale(float mult) {
      for (auto& path : paths_)
        for (Point& point : path)
          point *= mult;
    }

    Path reversed() const {
      Path reversed_path = *this;
      reversed_path.reverse();
      return reversed_path;
    }

    void reverse() {
      for (auto& path : paths_)
        std::reverse(path.begin(), path.end());
    }

  private:
    std::vector<Point>& currentPath() {
      if (paths_.empty())
        paths_.emplace_back();
      return paths_.back();
    }

    void addPoint(Point point) {
      currentPath().push_back(point);
      values_.push_back(0.0f);
    }

    void addPoint(float x, float y) {
      currentPath().emplace_back(x, y);
      values_.push_back(0.0f);
    }

    std::vector<std::vector<Point>> paths_;
    Point smooth_control_point_;
    std::vector<float> values_ {};
  };
}
