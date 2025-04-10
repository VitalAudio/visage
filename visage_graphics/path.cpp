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

#include "path.h"

#include <set>

namespace visage {
  void Path::arcTo(float rx, float ry, float x_axis_rotation, bool large_arc, bool sweep_flag,
                   Point point, bool relative) {
    static constexpr float kPi = 3.14159265358979323846f;

    Point p0 = lastPoint();
    if (relative)
      point += p0;

    float phi = x_axis_rotation * kPi / 180.0f;
    float cos_phi = cosf(phi);
    float sin_phi = sinf(phi);

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
      float scale = sqrtf(radii_scale);
      rx *= scale;
      ry *= scale;
      rx_sq = rx * rx;
      ry_sq = ry * ry;
    }

    float sign = (large_arc != sweep_flag) ? 1.0f : -1.0f;
    float sq = ((rx_sq * ry_sq) - (rx_sq * y1p_sq) - (ry_sq * x1p_sq)) /
               ((rx_sq * y1p_sq) + (ry_sq * x1p_sq));
    sq = std::max(0.0f, sq);
    float coef = sign * sqrtf(sq);
    float cxp = coef * ((rx * y1p) / ry);
    float cyp = coef * -((ry * x1p) / rx);

    float cx = cos_phi * cxp - sin_phi * cyp + (p0.x + point.x) / 2.0f;
    float cy = sin_phi * cxp + cos_phi * cyp + (p0.y + point.y) / 2.0f;

    auto vectorAngle = [](float ux, float uy, float vx, float vy) -> float {
      float dot = ux * vx + uy * vy;
      float len = sqrtf((ux * ux + uy * uy) * (vx * vx + vy * vy));
      float ang = acos(std::clamp(dot / len, -1.0f, 1.0f));
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
      float t = theta1 + delta_theta * (static_cast<float>(i) / kMaxCurveResolution);
      float cos_t = cosf(t);
      float sin_t = sinf(t);
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
    static bool compare(const ScanLineArea& area, Point point) {
      return area.sample(point) < point.y;
    }

    ScanLineArea(int from_index, Point from, int to_index, Point to, bool above_fill = false) :
        from_index(from_index), from(from), to_index(to_index), to(to), above_fill(above_fill) { }

    bool operator<(const ScanLineArea& other) const {
      Point position = from.x > other.from.x ? from : other.from;
      float sample1 = sample(position);
      float sample2 = other.sample(position);
      if (sample1 != sample2)
        return sample1 < sample2;

      float slope = (to.y - from.y) / (to.x - from.x);
      float other_slope = (other.to.y - other.from.y) / (other.to.x - other.from.x);
      if (slope != other_slope)
        return slope < other_slope;

      return to.y + from.y < other.to.y + other.from.y;
    }

    float sample(Point position) const {
      if (to.x == from.x) {
        if (position.x < from.x)
          return from.y;
        if (position.x > from.x)
          return to.y;
        float min = std::min(from.y, to.y);
        float max = std::max(from.y, to.y);
        return std::clamp(position.y, min, max);
      }
      if (position.x == to.x)
        return to.y;
      if (position.x == from.x)
        return from.y;

      float t = (position.x - from.x) / (to.x - from.x);
      return from.y + (to.y - from.y) * t;
    }

    int from_index;
    Point from;
    int to_index;
    Point to;
    bool above_fill;
  };

  struct IntersectionEvent {
    Point point;
    int a_to;
    bool a_forward;
    int b_to;
    bool b_forward;

    bool operator<(const IntersectionEvent& other) const {
      if (point.x != other.point.x)
        return point.x < other.point.x;
      if (point.y != other.point.y)
        return point.y < other.point.y;
      return a_to < other.a_to;
    }
  };

  class TriangulationGraph {
  public:
    Path::Triangulation triangulate(const Path* path) {
      num_points_ = path->numPoints();
      prev_edge_.reserve(num_points_);
      next_edge_.reserve(num_points_);

      points_.reserve(num_points_);
      int path_start = 0;
      for (const auto& sub_path : path->subPaths()) {
        int sub_path_size = sub_path.size();
        for (int i = 0; i < sub_path.size(); ++i) {
          points_.push_back(sub_path[i]);
          prev_edge_.push_back(path_start + ((i + sub_path_size - 1) % sub_path_size));
          next_edge_.push_back(path_start + ((i + 1) % sub_path_size));
        }
        path_start += sub_path_size;
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

    void removeFromCycle(int index) {
      int prev = prev_edge_[index];
      int next = next_edge_[index];
      prev_edge_[index] = index;
      next_edge_[index] = index;
      connect(prev, next);
    }

    void removeDuplicatePoints() {
      std::unique_ptr<bool[]> visited = std::make_unique<bool[]>(points_.size());
      for (int i = 0; i < points_.size(); ++i) {
        int prev_index = prev_edge_[i];
        int next_index = next_edge_[i];
        if (points_[i] == points_[prev_index] || points_[i] == points_[next_index])
          removeFromCycle(i);
      }
    }

    float compareIndices(int a_index, int b_index) const {
      float comp = points_[a_index].x - points_[b_index].x;
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

    bool checkValidPolygons() const {
      for (int i = 0; i < points_.size(); ++i) {
        if (prev_edge_[next_edge_[i]] != i || next_edge_[prev_edge_[i]] != i)
          return false;
      }
      return true;
    }

    std::vector<int> sortedIndices() const {
      std::vector<int> sorted_indices;
      sorted_indices.resize(prev_edge_.size());
      std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

      std::sort(sorted_indices.begin(), sorted_indices.end(),
                [this](const int a, const int b) { return compareIndices(a, b) < 0.0f; });
      return sorted_indices;
    }

    std::optional<std::pair<int, int>> breakIntersection(int start_index1, int end_index1,
                                                         int start_index2, int end_index2) {
      Point start1 = points_[start_index1];
      Point end1 = points_[end_index1];
      Point start2 = points_[start_index2];
      Point end2 = points_[end_index2];

      std::optional<Point> intersection = Path::findIntersection(start1, end1, start2, end2);
      VISAGE_ASSERT(intersection.has_value());
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

    std::optional<IntersectionEvent> adjacentIntersection(std::set<ScanLineArea>& areas,
                                                          std::set<ScanLineArea>::iterator it) const {
      if (it == areas.end())
        return std::nullopt;

      auto next = std::next(it);
      if (next == areas.end())
        return std::nullopt;

      auto intersection = Path::findIntersection(it->from, it->to, next->from, next->to);
      if (!intersection.has_value())
        return std::nullopt;

      return IntersectionEvent {
        intersection.value(),
        it->to_index,
        next_edge_[it->from_index] == it->to_index,
        next->to_index,
        next_edge_[next->from_index] == next->to_index,
      };
    }

    void checkAddIntersection(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas,
                              std::set<ScanLineArea>::iterator it) const {
      auto intersection = adjacentIntersection(areas, it);
      if (intersection.has_value())
        events.insert(intersection.value());
    };

    void checkRemoveIntersection(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas,
                                 std::set<ScanLineArea>::iterator it) const {
      auto intersection = adjacentIntersection(areas, it);
      if (intersection.has_value()) {
        VISAGE_ASSERT(events.count(intersection.value()));
        events.erase(intersection.value());
      }
    };

    void addArea(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas,
                 const ScanLineArea& area, bool check_remove = true) const {
      if (check_remove) {
        auto adjacent = areas.lower_bound(area);
        if (adjacent != areas.end() && adjacent != areas.begin())
          checkRemoveIntersection(events, areas, std::prev(adjacent));
      }

      auto it = areas.insert(area).first;
      checkAddIntersection(events, areas, it);
      checkAddIntersection(events, areas, std::prev(it));
    }

    void addArea(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas, int index,
                 Point point, int end_index1, Point end1, bool check_remove = true) const {
      addArea(events, areas, ScanLineArea(index, points_[index], end_index1, end1), check_remove);
    }

    void handleIntersectionEvent(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas,
                                 const IntersectionEvent& e) {
      int a_from = e.a_forward ? prev_edge_[e.a_to] : next_edge_[e.a_to];
      int b_from = e.b_forward ? prev_edge_[e.b_to] : next_edge_[e.b_to];
      std::optional<std::pair<int, int>> broken = breakIntersection(a_from, e.a_to, b_from, e.b_to);
      VISAGE_ASSERT(broken.has_value());

      ScanLineArea erase_area1(a_from, points_[a_from], e.a_to, points_[e.a_to]);
      ScanLineArea erase_area2(b_from, points_[b_from], e.b_to, points_[e.b_to]);
      if (erase_area2 < erase_area1)
        std::swap(erase_area1, erase_area2);

      auto erase1 = areas.find(erase_area1);
      checkRemoveIntersection(events, areas, std::prev(erase1));
      areas.erase(erase1);
      VISAGE_ASSERT(areas.size() % 2 == 1);

      auto erase2 = areas.find(erase_area2);
      checkRemoveIntersection(events, areas, erase2);
      VISAGE_ASSERT(areas.size() % 2 == 0);

      int a_index = connected(broken->first, e.a_to) ? broken->first : broken->second;
      int b_index = connected(broken->first, e.b_to) ? broken->first : broken->second;
      ScanLineArea area1(a_index, points_[a_index], e.a_to, points_[e.a_to]);
      ScanLineArea area2(b_index, points_[b_index], e.b_to, points_[e.b_to]);
      addArea(events, areas, area1, false);
      addArea(events, areas, area2);
    }

    void handlePointEvent(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas, int index) const {
      Point point = points_[index];
      int prev_index = prev_edge_[index];
      int next_index = next_edge_[index];
      Point prev = points_[prev_index];
      Point next = points_[next_index];

      if (prev == next)
        return;

      float compare_prev = compareIndices(index, prev_index);
      float compare_next = compareIndices(index, next_index);
      bool begin_area = compare_prev < 0.0f && compare_next < 0.0f;
      bool end_area = compare_prev > 0.0f && compare_next > 0.0f;

      auto area = std::lower_bound(areas.begin(), areas.end(), point, ScanLineArea::compare);

      if (begin_area) {
        addArea(events, areas, index, point, prev_index, prev);
        addArea(events, areas, index, point, next_index, next);
      }
      else if (end_area) {
        for (int i = 0; i < 2; ++i) {
          if (area == areas.end() || area->to_index != index) {
            VISAGE_ASSERT(false);
            return;
          }

          area = areas.erase(area);
        }
        if (area != areas.end() && area != areas.begin())
          checkAddIntersection(events, areas, std::prev(area));
      }
      else {
        areas.erase(area);
        if (compareIndices(prev_index, next_index) < 0.0f)
          addArea(events, areas, index, point, next_index, next, false);
        else
          addArea(events, areas, index, point, prev_index, prev, false);
      }

      VISAGE_ASSERT(checkValidPolygons());
      VISAGE_ASSERT(areas.size() % 2 == 0);
    }

    // Bentley-Ottmann algorithm for finding intersections
    void removeIntersections() {
      std::set<ScanLineArea> current_areas;
      auto sorted_indices = sortedIndices();

      std::set<IntersectionEvent> intersection_events;
      for (int index : sorted_indices)
        intersection_events.insert(IntersectionEvent { points_[index], index, true, index, true });

      while (!intersection_events.empty()) {
        auto it = intersection_events.begin();
        IntersectionEvent ev = *it;
        intersection_events.erase(it);

        if (ev.a_to == ev.b_to)
          handlePointEvent(intersection_events, current_areas, ev.a_to);
        else
          handleIntersectionEvent(intersection_events, current_areas, ev);
      }
    }

    void reverseCycle(int start_index) {
      std::swap(prev_edge_[start_index], next_edge_[start_index]);
      for (int i = next_edge_[start_index]; i != start_index; i = next_edge_[i])
        std::swap(prev_edge_[i], next_edge_[i]);
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

    // Seidel's algorithm for breaking simple polygon into monotonic polygons
    void breakIntoMonotonicPolygons() {
      std::map<ScanLineArea, int> current_areas;
      auto sorted_indices = sortedIndices();

      for (int index : sorted_indices) {
        Point point = points_[index];
        int prev_index = prev_edge_[index];
        int next_index = next_edge_[index];
        Point prev = points_[prev_index];
        Point next = points_[next_index];
        if (prev == next)
          continue;

        float compare_prev = compareIndices(index, prev_index);
        float compare_next = compareIndices(index, next_index);
        bool begin_area = compare_prev < 0.0f && compare_next < 0.0f;
        bool end_area = compare_prev > 0.0f && compare_next > 0.0f;

        auto area = std::lower_bound(current_areas.begin(), current_areas.end(), point,
                                     [](const auto& pair, const Point& p) {
                                       return ScanLineArea::compare(pair.first, p);
                                     });

        if (begin_area) {
          bool start_hole = area != current_areas.end() && !area->first.above_fill;
          bool convex = (prev - point).cross(next - point) > 0.0f;
          if (start_hole == convex)
            reverseCycle(index);

          int diagonal_index = index;

          if (start_hole)
            diagonal_index = addDiagonal(index, area->second);

          auto area1 = ScanLineArea(index, point, prev_index, prev, !start_hole);
          auto area2 = ScanLineArea(diagonal_index, point, next_index, next, start_hole);
          if (area2 < area1) {
            area1.above_fill = !area1.above_fill;
            area2.above_fill = !area2.above_fill;
          }
          current_areas[area1] = index;
          current_areas[area2] = diagonal_index;
        }
        else if (end_area) {
          for (int i = 0; i < 2; ++i) {
            if (area == current_areas.end() || area->first.to_index != index) {
              VISAGE_ASSERT(false);
              return;
            }

            area = current_areas.erase(area);
          }
          if (area != current_areas.end() && !area->first.above_fill) {
            area->second = index;
            std::prev(area)->second = index;
          }
        }
        else {
          while (area != current_areas.end() && area->first.to_index != index)
            ++area;
          if (area == current_areas.end()) {
            VISAGE_ASSERT(false);
            return;
          }
          bool above_fill = area->first.above_fill;
          current_areas.erase(area);

          if (compareIndices(prev_index, next_index) < 0.0f)
            current_areas[ScanLineArea(index, point, next_index, next, above_fill)] = index;
          else
            current_areas[ScanLineArea(index, point, prev_index, prev, above_fill)] = index;
        }

        VISAGE_ASSERT(checkValidPolygons());
        VISAGE_ASSERT(current_areas.size() % 2 == 0);
      }
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

      if (intermediate.x > start.x || target.x > start.x)
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

    std::vector<int> breakIntoTriangles() {
      removeDuplicatePoints();
      std::vector<int> triangles;
      auto sorted_indices = sortedIndices();

      for (int index : sorted_indices)
        cutEars(index, triangles);

      return triangles;
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
