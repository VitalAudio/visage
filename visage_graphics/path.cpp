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
  struct PolygonArea {
    PolygonArea(int from_index, Point from, int to_index, Point to, float* position) :
        from_index(from_index), from(from), to_index(to_index), to(to), position(position) { }

    bool operator<(const PolygonArea& other) const {
      float sample1 = sample();
      float sample2 = other.sample();
      if (sample1 != sample2)
        return sample1 < sample2;

      float slope = (to.y - from.y) / (to.x - from.x);
      float other_slope = (other.to.y - other.from.y) / (other.to.x - other.from.x);
      return slope < other_slope;
    }

    float sample() const {
      if (*position == to.x)
        return to.y;

      VISAGE_ASSERT(to.x != from.x);
      return from.y + (to.y - from.y) * (*position - from.x) / (to.x - from.x);
    }

    int from_index;
    Point from;
    int to_index;
    Point to;
    float* position = nullptr;
  };

  struct EdgeGraph {
    static float compareIndices(const Path* path, int a_index, int b_index) {
      float comp = path->point(a_index).x - path->point(b_index).x;
      int index_offset = a_index - b_index;
      int a = a_index;
      int b = b_index;

      int offset = (path->numPoints() + b_index - a_index) % path->numPoints();
      bool move_a = offset > path->numPoints() / 2;
      while (comp == 0.0f && b != a_index && a != b_index) {
        if (move_a)
          a = (a + 1) % path->numPoints();
        else
          b = (b + 1) % path->numPoints();

        comp = path->point(a).x - path->point(b).x;
      }
      return comp;
    }

    EdgeGraph(const Path* path) : path(path) {
      int num_points = path->numPoints();
      prev_edge.reserve(num_points);
      next_edge.reserve(num_points);
      for (int i = 0; i < num_points; ++i) {
        prev_edge.push_back((i + num_points - 1) % num_points);
        next_edge.push_back((i + 1) % num_points);
      }

      sorted_indices.reserve(num_points);
      for (int i = 0; i < num_points; ++i)
        sorted_indices.push_back(i);

      std::sort(sorted_indices.begin(), sorted_indices.end(),
                [path](const int a, const int b) { return compareIndices(path, a, b) < 0.0f; });
    }

    bool tryCutEar(int index, bool forward, std::vector<int>& triangles) {
      auto& direction = forward ? next_edge : prev_edge;
      auto& reverse = forward ? prev_edge : next_edge;

      int intermediate_index = direction[index];
      int target_index = direction[intermediate_index];
      if (intermediate_index == index || target_index == index)
        return false;

      Point start = point(index);
      Point intermediate = point(intermediate_index);
      Point target = point(target_index);

      if (intermediate.x >= start.x || target.x > start.x)
        return false;

      float cross = (intermediate - start).cross(target - intermediate);
      if ((cross < 0.0f) != forward && cross)
        return false;

      if (cross) {
        triangles.push_back(originalIndex(index));
        triangles.push_back(originalIndex(intermediate_index));
        triangles.push_back(originalIndex(target_index));
      }
      direction[index] = target_index;
      reverse[target_index] = index;

      direction[intermediate_index] = intermediate_index;
      reverse[intermediate_index] = intermediate_index;
      return true;
    }

    void reverseCycle(int start_index) {
      std::swap(prev_edge[start_index], next_edge[start_index]);
      for (int i = next_edge[start_index]; i != start_index; i = next_edge[i])
        std::swap(prev_edge[i], next_edge[i]);
    }

    void cutEars(int index, std::vector<int>& triangles) {
      while (tryCutEar(index, true, triangles))
        ;
      while (tryCutEar(index, false, triangles))
        ;
    }

    int next(int index, bool forward = true) const {
      return forward ? next_edge[index] : prev_edge[index];
    }

    int prev(int index, bool forward = true) const {
      return forward ? prev_edge[index] : next_edge[index];
    }

    int originalIndex(int index) const {
      if (index >= path->numPoints())
        return additional_point_refs[index - sorted_indices.size()];
      return index;
    }

    Point point(int index) const { return path->point(originalIndex(index)); }

    int sortedIndex(int sorted_index) const { return sorted_indices[sorted_index]; }

    int addDiagonal(int index, int target) {
      int new_index = prev_edge.size();
      int new_diagonal_index = new_index + 1;
      additional_point_refs.push_back(originalIndex(index));
      additional_point_refs.push_back(originalIndex(target));

      next_edge[prev_edge[target]] = new_diagonal_index;
      prev_edge[next_edge[index]] = new_index;

      prev_edge.push_back(new_diagonal_index);
      next_edge.push_back(next_edge[index]);
      prev_edge.push_back(prev_edge[target]);
      next_edge.push_back(new_index);

      next_edge[index] = target;
      prev_edge[target] = index;

      return new_index;
    }

    const Path* path = nullptr;
    std::vector<int> prev_edge;
    std::vector<int> next_edge;
    std::vector<int> additional_point_refs;
    std::vector<int> sorted_indices;
  };

  std::vector<int> Path::triangulate() const {
    int num_points = numPoints();
    if (num_points < 3)
      return {};

    std::vector<int> triangles;
    triangles.reserve((num_points - 2) * 3);

    auto area_compare = [](const PolygonArea& area, float sample) { return area.sample() < sample; };
    EdgeGraph graph(this);

    float position = 0.0f;
    std::set<PolygonArea> current_areas;
    for (int i = 0; i < num_points; ++i) {
      int index = graph.sortedIndex(i);

      Point point = graph.point(index);
      position = point.x;
      int prev_index = graph.prev(index);
      int next_index = graph.next(index);
      Point prev = graph.point(prev_index);
      Point next = graph.point(next_index);
      bool begin_area = prev.x > point.x && next.x > point.x;
      bool end_area = prev.x < point.x && next.x < point.x;

      auto area = std::lower_bound(current_areas.begin(), current_areas.end(), point.y, area_compare);

      if (begin_area) {
        bool start_hole = std::distance(current_areas.begin(), area) % 2 == 1;
        bool convex = (prev - point).cross(next - point) > 0.0f;
        if (start_hole == convex)
          graph.reverseCycle(index);

        int diagonal_break_index = index;

        if (start_hole) {
          int diagonal_index = area->from_index;
          diagonal_break_index = graph.addDiagonal(index, diagonal_index);
          graph.cutEars(diagonal_break_index, triangles);
        }
        current_areas.emplace(index, point, prev_index, prev, &position);
        current_areas.emplace(diagonal_break_index, point, next_index, next, &position);
      }
      else if (end_area) {
        VISAGE_ASSERT(area != current_areas.end());
        if (area == current_areas.end())
          return {};

        VISAGE_ASSERT(area->to_index == index);
        VISAGE_ASSERT(std::next(area)->to_index == index);
        if (area->to_index != index || std::next(area)->to_index != index)
          return {};

        for (auto it = area; it != current_areas.end() && it->to_index == index;)
          it = current_areas.erase(it);
      }
      else {
        VISAGE_ASSERT(area != current_areas.end());
        if (area == current_areas.end())
          return {};

        VISAGE_ASSERT(area->to_index == index);
        if (area->to_index != index)
          return {};

        VISAGE_ASSERT(index != next_index);
        VISAGE_ASSERT(index != prev_index);
        current_areas.erase(area);
        if (EdgeGraph::compareIndices(this, graph.originalIndex(prev_index),
                                      graph.originalIndex(next_index)) < 0.0f) {
          current_areas.emplace(index, point, next_index, next, &position);
        }
        else
          current_areas.emplace(index, point, prev_index, prev, &position);
      }

      graph.cutEars(index, triangles);
      VISAGE_ASSERT(current_areas.size() % 2 == 0);
    }

    return triangles;
  }
}
