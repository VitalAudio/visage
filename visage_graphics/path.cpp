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

#include "path.h"

#include <optional>
#include <set>

namespace visage {
  void Path::arcTo(float rx, float ry, float x_axis_rotation, bool large_arc, bool sweep_flag,
                   Point point, bool relative) {
    static constexpr float kPi = 3.14159265358979323846f;

    if (current_path_.empty())
      return;

    Point p0 = current_path_.back();
    if (relative)
      point += p0;

    float phi = x_axis_rotation * kPi / 180.0f;
    float cos_phi = cos(phi);
    float sin_phi = sin(phi);

    float dx2 = (p0.x - point.x) / 2.0f;
    float dy2 = (p0.y - point.y) / 2.0f;
    float x1p = cos_phi * dx2 + sin_phi * dy2;
    float y1p = -sin_phi * dx2 + cos_phi * dy2;

    float rx_sq = rx * rx;
    float ry_sq = ry * ry;
    float x1p_sq = x1p * x1p;
    float y1p_sq = y1p * y1p;
    float radii_scale = x1p_sq / rx_sq + y1p_sq / ry_sq;
    if (radii_scale > 1.0f) {
      float scale = sqrt(radii_scale);
      rx *= scale;
      ry *= scale;
      rx_sq = rx * rx;
      ry_sq = ry * ry;
    }

    float sign = (large_arc != sweep_flag) ? 1.0f : -1.0f;
    float sq = ((rx_sq * ry_sq) - (rx_sq * y1p_sq) - (ry_sq * x1p_sq)) /
               ((rx_sq * y1p_sq) + (ry_sq * x1p_sq));
    sq = std::max(0.0f, sq);
    float coef = sign * sqrt(sq);
    float cxp = coef * ((rx * y1p) / ry);
    float cyp = coef * -((ry * x1p) / rx);

    float cx = cos_phi * cxp - sin_phi * cyp + (p0.x + point.x) / 2.0f;
    float cy = sin_phi * cxp + cos_phi * cyp + (p0.y + point.y) / 2.0f;

    auto vectorAngle = [](float ux, float uy, float vx, float vy) -> float {
      float dot = ux * vx + uy * vy;
      float len = sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy));
      float ang = acos(std::min(std::max(dot / len, -1.0f), 1.0f));
      if (ux * vy - uy * vx < 0.0f)
        ang = -ang;
      return ang;
    };

    float theta1 = vectorAngle(1.0f, 0.0f, (x1p - cxp) / rx, (y1p - cyp) / ry);
    float delta_theta = vectorAngle((x1p - cxp) / rx, (y1p - cyp) / ry, (-x1p - cxp) / rx,
                                    (-y1p - cyp) / ry);

    if (!sweep_flag && delta_theta > 0)
      delta_theta -= 2 * kPi;
    else if (sweep_flag && delta_theta < 0)
      delta_theta += 2 * kPi;

    for (int i = 1; i <= kMaxCurveResolution; ++i) {
      float t = theta1 + delta_theta * (float(i) / kMaxCurveResolution);
      float cos_t = cos(t);
      float sin_t = sin(t);
      float x = cos_phi * rx * cos_t - sin_phi * ry * sin_t + cx;
      float y = sin_phi * rx * cos_t + cos_phi * ry * sin_t + cy;
      addPoint(x, y);
    }

    smooth_control_point_ = {};
  }

  static std::vector<float> parseNumbers(const std::string& str, size_t& i, int num) {
    std::vector<float> numbers;
    std::string number;
    while (i < str.size() && numbers.size() < num) {
      bool sign = str[i] == '-' || str[i] == '+';
      if (std::isdigit(str[i]) || (number.empty() && sign) || str[i] == '.' || str[i] == 'e' || str[i] == 'E')
        number += str[i++];
      else if (str[i] == ',' || std::isspace(str[i]) || sign) {
        if (!number.empty()) {
          numbers.push_back(std::stof(number));
          number.clear();
        }
        if (!sign)
          ++i;
      }
      else if (std::isalpha(str[i]))
        break;
      else
        ++i;
    }
    if (!number.empty())
      numbers.push_back(std::stof(number));

    VISAGE_ASSERT(num == numbers.size());
    return numbers;
  }

  static int numbersForCommand(char command) {
    switch (std::toupper(command)) {
    case 'M':
    case 'L':
    case 'T': return 2;
    case 'H':
    case 'V': return 1;
    case 'Q':
    case 'S': return 4;
    case 'C': return 6;
    case 'A': return 7;
    case 'Z': return 0;
    default: VISAGE_ASSERT(false); return 0;
    }
  }

  void Path::parseSvgPath(const std::string& path) {
    clear();

    size_t i = 0;
    char command_char = 0;
    while (i < path.size()) {
      if (std::isspace(path[i])) {
        ++i;
        continue;
      }

      char new_command = path[i];
      if (std::isalpha(new_command)) {
        command_char = new_command;
        ++i;
      }

      bool relative = std::islower(command_char);
      std::vector<float> vals = parseNumbers(path, i, numbersForCommand(command_char));

      switch (std::toupper(command_char)) {
      case 'M': moveTo(vals[0], vals[1], relative); break;
      case 'L': lineTo(vals[0], vals[1], relative); break;
      case 'H': horizontalTo(vals[0], relative); break;
      case 'V': verticalTo(vals[0], relative); break;
      case 'Q': quadraticTo(vals[0], vals[1], vals[2], vals[3], relative); break;
      case 'T': smoothQuadraticTo(vals[0], vals[1], relative); break;
      case 'C': bezierTo(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], relative); break;
      case 'S': smoothBezierTo(vals[0], vals[1], vals[2], vals[3], relative); break;
      case 'A':
        arcTo(vals[0], vals[1], vals[2], vals[3], vals[4], Point(vals[5], vals[6]), relative);
        break;
      case 'Z': closePath(); break;
      default: VISAGE_ASSERT(false);
      }
    }
  }

  struct ScanLineArea {
    ScanLineArea(int from_index, Point from, int to_index, Point to, float* position) :
        from_index(from_index), from(from), to_index(to_index), to(to), position(position) { }

    bool operator<(const ScanLineArea& other) const {
      float sample1 = sample();
      float sample2 = other.sample();
      if (sample1 != sample2)
        return sample1 < sample2;

      float slope = (to.y - from.y) / (to.x - from.x);
      float other_slope = (other.to.y - other.from.y) / (other.to.x - other.from.x);
      if (slope != other_slope)
        return slope < other_slope;

      return to.x + from.x < other.to.x + other.from.x;
    }

    float sample() const {
      if (*position == to.x || to.x == from.x)
        return to.y;

      return from.y + (to.y - from.y) * (*position - from.x) / (to.x - from.x);
    }

    int from_index;
    Point from;
    int to_index;
    Point to;
    float* position = nullptr;
  };

  class TriangulationGraph {
  public:
    Path::Triangulation triangulate(const Path* path) {
      num_points_ = path->numPoints();
      points_ = path->points();

      prev_edge_.reserve(num_points_);
      next_edge_.reserve(num_points_);
      for (int i = 0; i < num_points_; ++i) {
        prev_edge_.push_back((i + num_points_ - 1) % num_points_);
        next_edge_.push_back((i + 1) % num_points_);
      }

      removeIntersections();
      breakIntoMonotonicPolygons();
      Path::Triangulation result;
      result.triangles = breakIntoTriangles();
      result.points = std::move(points_);
      return result;
    }

  private:
    int addAdditionalPoint(Point point) {
      points_.push_back(point);
      int new_index = points_.size() - 1;
      prev_edge_.push_back(new_index);
      next_edge_.push_back(new_index);
      return new_index;
    }

    bool connected(int a_index, int b_index) const {
      return prev_edge_[a_index] == b_index || next_edge_[a_index] == b_index;
    }

    void connect(int from, int to) {
      next_edge_[from] = to;
      prev_edge_[to] = from;
    }

    std::vector<int> sortedIndices() {
      std::vector<int> sorted_indices;
      sorted_indices.resize(prev_edge_.size());
      std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

      std::sort(sorted_indices.begin(), sorted_indices.end(),
                [this](const int a, const int b) { return compareIndices(a, b) < 0.0f; });
      return sorted_indices;
    }

    void removeIntersections() {
      auto area_compare = [](const ScanLineArea& area, float sample) {
        return area.sample() < sample;
      };

      std::set<ScanLineArea> current_areas;

      float position = 0.0f;
      auto add_area = [&](int index, Point point, int end_index, Point end) {
        auto adjacent = std::lower_bound(current_areas.begin(), current_areas.end(), point.y, area_compare);

        std::set<ScanLineArea> intersected_areas;
        int start_index = index;

        auto try_intersect = [&](auto& it) {
          std::optional<std::pair<int, int>> new_indices = breakIntersection(start_index, end_index,
                                                                             it->from_index, it->to_index);
          if (!new_indices.has_value())
            return false;

          start_index = connected(new_indices->first, end_index) ? new_indices->first : new_indices->second;
          position = points_[start_index].x;
          int intersected_index = connected(new_indices->first, it->to_index) ? new_indices->first :
                                                                                new_indices->second;
          intersected_areas.insert(ScanLineArea(intersected_index, points_[intersected_index],
                                                it->to_index, it->to, &position));
          it = current_areas.erase(it);
          return true;
        };

        auto it = adjacent;
        while (it != current_areas.end() && it->from == point)
          ++it;

        while (it != current_areas.end() && try_intersect(it))
          ;

        if (intersected_areas.empty() && adjacent != current_areas.begin()) {
          it = adjacent;
          while (it != current_areas.begin()) {
            it = std::prev(it);
            if (!try_intersect(it))
              break;
          }
        }

        current_areas.insert(intersected_areas.begin(), intersected_areas.end());
        current_areas.emplace(start_index, points_[start_index], end_index, end, &position);
      };

      auto sorted_indices = sortedIndices();

      for (int index : sorted_indices) {
        Point point = points_[index];
        position = point.x;
        int prev_index = prev_edge_[index];
        int next_index = next_edge_[index];
        Point prev = points_[prev_index];
        Point next = points_[next_index];
        if (prev.x == point.x && next.x == point.x)
          continue;

        bool begin_area = prev.x >= point.x && next.x >= point.x && prev.x != next.x;
        bool end_area = prev.x <= point.x && next.x <= point.x && prev.x != next.x;

        auto area = std::lower_bound(current_areas.begin(), current_areas.end(), point.y, area_compare);

        if (begin_area) {
          bool prev_higher = (prev - point).cross(next - point) > 0.0f;
          if (prev_higher == prev.y < point.y) {
            add_area(index, point, prev_index, prev);
            add_area(index, point, next_index, next);
          }
          else {
            add_area(index, point, next_index, next);
            add_area(index, point, prev_index, prev);
          }
        }
        else if (end_area) {
          area = current_areas.erase(area);
          if (area == current_areas.end() || area->to_index != index) {
            VISAGE_ASSERT(false);
            return;
          }

          current_areas.erase(area);
        }
        else {
          current_areas.erase(area);
          if (compareIndices(prev_index, next_index) < 0.0f)
            add_area(index, point, next_index, next);
          else
            add_area(index, point, prev_index, prev);
        }

        VISAGE_ASSERT(checkValidPolygons());
        VISAGE_ASSERT(current_areas.size() % 2 == 0);
      }
    }

    void breakIntoMonotonicPolygons() {
      auto area_compare = [](const ScanLineArea& area, float sample) {
        return area.sample() < sample;
      };

      std::set<ScanLineArea> current_areas;

      float position = 0.0f;
      auto add_area = [&](int index, Point point, int end_index, Point end) {
        current_areas.emplace(index, point, end_index, end, &position);
      };

      auto sorted_indices = sortedIndices();

      for (int index : sorted_indices) {
        Point point = points_[index];
        position = point.x;
        int prev_index = prev_edge_[index];
        int next_index = next_edge_[index];
        Point prev = points_[prev_index];
        Point next = points_[next_index];
        if (prev.x == point.x && next.x == point.x)
          continue;

        bool begin_area = prev.x >= point.x && next.x >= point.x && prev.x != next.x;
        bool end_area = prev.x <= point.x && next.x <= point.x && prev.x != next.x;

        auto area = std::lower_bound(current_areas.begin(), current_areas.end(), point.y, area_compare);

        if (begin_area) {
          bool start_hole = std::distance(current_areas.begin(), area) % 2 == 1;
          bool convex = (prev - point).cross(next - point) > 0.0f;
          if (start_hole == convex)
            reverseCycle(index);

          int diagonal_break_index = index;

          if (start_hole) {
            int diagonal_index = area->from_index;
            diagonal_break_index = addDiagonal(index, diagonal_index);
          }
          add_area(index, point, prev_index, prev);
          add_area(diagonal_break_index, point, next_index, next);
        }
        else if (end_area) {
          area = current_areas.erase(area);
          if (area == current_areas.end() || area->to_index != index) {
            VISAGE_ASSERT(false);
            return;
          }

          current_areas.erase(area);
        }
        else {
          while (area->to_index != index && area != current_areas.end())
            ++area;
          if (area == current_areas.end()) {
            VISAGE_ASSERT(false);
            return;
          }
          current_areas.erase(area);

          if (compareIndices(prev_index, next_index) < 0.0f)
            add_area(index, point, next_index, next);
          else
            add_area(index, point, prev_index, prev);
        }

        VISAGE_ASSERT(checkValidPolygons());
        VISAGE_ASSERT(current_areas.size() % 2 == 0);
      }
    }

    std::vector<int> breakIntoTriangles() {
      std::vector<int> triangles;
      auto sorted_indices = sortedIndices();

      for (int index : sorted_indices)
        cutEars(index, triangles);

      return triangles;
    }

    bool checkValidPolygons() const {
      for (int i = 0; i < points_.size(); ++i) {
        if (prev_edge_[next_edge_[i]] != i || next_edge_[prev_edge_[i]] != i)
          return false;
      }
      return true;
    }

    float compareIndices(int a_index, int b_index) {
      float comp = points_[a_index].x - points_[b_index].x;
      int index_offset = a_index - b_index;
      int a_prev = a_index;
      int a_next = a_index;
      int b_prev = b_index;
      int b_next = b_index;

      float y_comp = points_[a_index].y - points_[b_index].y;
      if (comp == 0.0f && y_comp)
        return y_comp;

      while (comp == 0.0f) {
        a_next = next_edge_[a_next];
        if (a_next == a_prev || a_next == prev_edge_[a_prev])
          return 0.0f;
        a_prev = prev_edge_[a_prev];

        b_next = next_edge_[b_next];
        b_prev = prev_edge_[b_prev];

        comp = points_[a_next].x + points_[a_prev].x - points_[b_next].x - points_[b_prev].x;
      }
      return comp;
    }

    bool tryCutEar(int index, bool forward, std::vector<int>& triangles) {
      auto& direction = forward ? next_edge_ : prev_edge_;
      auto& reverse = forward ? prev_edge_ : next_edge_;

      int intermediate_index = direction[index];
      int target_index = direction[intermediate_index];
      if (intermediate_index == index || target_index == index)
        return false;

      Point start = points_[index];
      Point intermediate = points_[intermediate_index];
      Point target = points_[target_index];

      if (intermediate.x >= start.x || target.x > start.x)
        return false;

      float cross = (intermediate - start).cross(target - intermediate);
      if ((cross < 0.0f) != forward && cross)
        return false;

      if (cross) {
        triangles.push_back(index);
        triangles.push_back(intermediate_index);
        triangles.push_back(target_index);
      }
      direction[index] = target_index;
      reverse[target_index] = index;

      direction[intermediate_index] = intermediate_index;
      reverse[intermediate_index] = intermediate_index;
      return true;
    }

    void cutEars(int index, std::vector<int>& triangles) {
      while (tryCutEar(index, true, triangles))
        ;
      while (tryCutEar(index, false, triangles))
        ;
    }

    void reverseCycle(int start_index) {
      std::swap(prev_edge_[start_index], next_edge_[start_index]);
      for (int i = next_edge_[start_index]; i != start_index; i = next_edge_[i])
        std::swap(prev_edge_[i], next_edge_[i]);
    }

    std::optional<Point> findIntersection(Point start1, Point end1, Point start2, Point end2) {
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

    std::optional<std::pair<int, int>> breakIntersection(int start_index1, int end_index1,
                                                         int start_index2, int end_index2) {
      Point start1 = points_[start_index1];
      Point end1 = points_[end_index1];
      Point start2 = points_[start_index2];
      Point end2 = points_[end_index2];

      std::optional<Point> intersection = findIntersection(start1, end1, start2, end2);
      if (!intersection.has_value())
        return std::nullopt;

      int new_index1 = addAdditionalPoint(intersection.value());
      int new_index2 = addAdditionalPoint(intersection.value());

      if (next_edge_[start_index1] != end_index1)
        std::swap(start_index1, end_index1);
      if (next_edge_[start_index2] != end_index2)
        std::swap(start_index2, end_index2);

      connect(start_index1, new_index1);
      connect(new_index1, end_index2);
      connect(start_index2, new_index2);
      connect(new_index2, end_index1);

      VISAGE_ASSERT(checkValidPolygons());
      return std::pair(new_index1, new_index2);
    }

    int addDiagonal(int index, int target) {
      int new_index = prev_edge_.size();
      int new_diagonal_index = new_index + 1;
      points_.push_back(points_[index]);
      points_.push_back(points_[target]);

      next_edge_[prev_edge_[target]] = new_diagonal_index;
      prev_edge_[next_edge_[index]] = new_index;

      prev_edge_.push_back(new_diagonal_index);
      next_edge_.push_back(next_edge_[index]);
      prev_edge_.push_back(prev_edge_[target]);
      next_edge_.push_back(new_index);

      connect(index, target);

      return new_index;
    }

    int num_points_ = 0;
    std::vector<Point> points_;
    std::vector<int> prev_edge_;
    std::vector<int> next_edge_;
  };

  Path::Triangulation Path::triangulate() const {
    TriangulationGraph graph;
    return graph.triangulate(this);
  }
}
