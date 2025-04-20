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
      case 'Z': close(); break;
      default: VISAGE_ASSERT(false);
      }
    }
  }

  static double orientation(const DPoint& source, const DPoint& target1, const DPoint& target2) {
    static constexpr double kEpsilon = 3.3306690738754716e-16;
    double l = (target2.y - source.y) * (target1.x - source.x);
    double r = (target2.x - source.x) * (target1.y - source.y);
    double sum = std::abs(l + r);
    double diff = l - r;
    return std::abs(diff) >= kEpsilon * sum ? diff : 0.0;
  }

  double stableOrientation(const DPoint& source, const DPoint& target1, const DPoint& target2) {
    double result = orientation(source, target1, target2);
    if (result != 0.0)
      return result;

    result = orientation(target2, source, target1);
    if (result != 0.0)
      return result;

    return orientation(target1, target2, source);
  }

  struct ScanLineArea {
    static bool compare(const ScanLineArea& area, DPoint point) {
      return area.sample(point) < point.y;
    }

    ScanLineArea(int from_index, DPoint from, int to_index, DPoint to) :
        from_index(from_index), from(from), to_index(to_index), to(to) { }

    bool operator<(const ScanLineArea& other) const {
      double orientation = from.y - other.from.y;
      if (other.from < from)
        orientation = stableOrientation(other.from, other.to, from);
      else if (from < other.from)
        orientation = -stableOrientation(from, to, other.from);

      if (orientation)
        return orientation < 0.0;

      if (to == other.from && from != other.to)
        return true;
      if (from == other.to && to != other.from)
        return false;

      if (other.to < to)
        orientation = -stableOrientation(other.from, other.to, to);
      else if (to < other.to)
        orientation = stableOrientation(from, to, other.to);
      if (orientation)
        return orientation > 0.0;

      if (from_index != other.from_index)
        return from_index < other.from_index;
      return to_index < other.to_index;
    }

    double sample(DPoint position) const {
      if (to.x == from.x) {
        if (position.x < from.x)
          return from.y;
        if (position.x > from.x)
          return to.y;
        double min = std::min(from.y, to.y);
        double max = std::max(from.y, to.y);
        return std::clamp(position.y, min, max);
      }
      if (position.x == to.x)
        return to.y;
      if (position.x == from.x)
        return from.y;

      DPoint delta = to - from;
      return from.y + (delta.y * (position.x - from.x)) / delta.x;
    }

    int from_index;
    DPoint from;
    int to_index;
    DPoint to;
  };

  class TriangulationGraph {
  public:
    TriangulationGraph(const Path* path) {
      int num_points = path->numPoints();
      prev_edge_.reserve(num_points);
      next_edge_.reserve(num_points);

      points_.reserve(num_points);
      int path_start = 0;
      for (const auto& sub_path : path->subPaths()) {
        int sub_path_size = sub_path.points.size();
        for (int i = 0; i < sub_path.points.size(); ++i) {
          points_.emplace_back(sub_path.points[i]);
          prev_edge_.push_back(path_start + ((i + sub_path_size - 1) % sub_path_size));
          next_edge_.push_back(path_start + ((i + 1) % sub_path_size));
        }
        path_start += sub_path_size;
      }
    }

    struct IntersectionEvent {
      DPoint point;
      int a_to;
      bool a_forward;
      int b_to;
      bool b_forward;
      const TriangulationGraph* graph;

      bool operator<(const IntersectionEvent& other) const {
        if (point.x != other.point.x)
          return point.x < other.point.x;
        if (point.y != other.point.y)
          return point.y < other.point.y;
        if (a_to == b_to && other.a_to == other.b_to)
          return graph->compareIndices(a_to, other.a_to) < 0.0;
        return a_to < other.a_to;
      }
    };

    Path::Triangulation triangulate(Path::FillRule fill_rule, int minimum_cycles = 1) {
      simplify();
      breakIntersections();
      fixWindings(fill_rule, minimum_cycles);
      breakSimpleIntoMonotonicPolygons();
      Path::Triangulation result;
      result.triangles = breakIntoTriangles();
      for (const auto& point : points_)
        result.points.emplace_back(point);
      return result;
    }

    double compareIndices(int a_index, int b_index) const {
      DPoint delta = points_[a_index] - points_[b_index];
      if (delta.x)
        return delta.x;
      if (delta.y)
        return delta.y;

      int a_prev = a_index;
      int a_next = a_index;
      int b_prev = b_index;
      int b_next = b_index;

      do {
        a_next = next_edge_[a_next];
        a_prev = prev_edge_[a_prev];

        b_next = next_edge_[b_next];
        b_prev = prev_edge_[b_prev];

        double sum_a = points_[a_next].x + points_[a_prev].x;
        double sum_b = points_[b_next].x + points_[b_prev].x;
        if (sum_a < sum_b)
          return -1.0;
        if (sum_a > sum_b)
          return 1.0;
      } while (next_edge_[a_next] != a_prev && next_edge_[b_next] != b_prev &&
               next_edge_[a_next] != prev_edge_[a_prev] && next_edge_[b_next] != prev_edge_[b_prev]);
      return a_index - b_index;
    }

    // Bentley-Ottmann algorithm for finding intersections
    void breakIntersections() {
      std::set<ScanLineArea> areas;
      auto sorted_indices = sortedIndices();

      std::set<IntersectionEvent> intersection_events;
      for (int index : sorted_indices)
        intersection_events.insert(IntersectionEvent { points_[index], index, true, index, true, this });

      VISAGE_ASSERT(checkValidPolygons());

      while (!intersection_events.empty()) {
        auto it = intersection_events.begin();
        IntersectionEvent ev = *it;
        intersection_events.erase(it);

        if (ev.a_to == ev.b_to)
          handlePointEvent(intersection_events, areas, ev.a_to);
        else
          handleIntersectionEvent(intersection_events, areas, ev);
      }

      simplify();
    }

    void fixWindings(Path::FillRule fill_rule, int minimum_cycles = 1) {
      std::set<ScanLineArea> areas;
      auto sorted_indices = sortedIndices();
      auto windings = std::make_unique<int[]>(points_.size());
      auto reversed = std::make_unique<bool[]>(points_.size());
      auto winding_directions = std::make_unique<int[]>(points_.size());

      for (int index : sorted_indices) {
        if (next_edge_[index] == index || prev_edge_[index] == next_edge_[index])
          continue;

        DPoint point = points_[index];
        if (winding_directions[index] == 0) {
          bool convex = stableOrientation(point, points_[prev_edge_[index]],
                                          points_[next_edge_[index]]) > 0.0;

          auto area = std::lower_bound(areas.begin(), areas.end(), point, ScanLineArea::compare);

          int current_winding = 0;
          bool reverse = false;
          if (area != areas.end()) {
            bool area_fill_above = next_edge_[area->from_index] == area->to_index;
            bool fill = winding_directions[area->from_index] == 1;
            current_winding += windings[area->from_index];
            bool inside_polygon = area_fill_above == fill;

            if (inside_polygon) {
              if (reversed[area->from_index] && fill_rule != Path::FillRule::EvenOdd)
                reverse = true;
            }
            else
              current_winding -= winding_directions[area->from_index];
          }

          if (fill_rule == Path::FillRule::EvenOdd) {
            bool fill = (current_winding % 2) == 0;
            reverse = reverse || (convex != fill);
          }
          else if (fill_rule == Path::FillRule::NonZero && current_winding == 0)
            reverse = reverse || !convex;

          if (reverse) {
            reverseCycle(index);
            convex = !convex;
          }

          int winding_direction = convex ? 1 : -1;
          int winding = current_winding + winding_direction;
          for (int i = index; winding_directions[i] == 0; i = next_edge_[i]) {
            winding_directions[i] = winding_direction;
            windings[i] = winding;
            reversed[i] = reverse;
          }
        }

        int prev_index = prev_edge_[index];
        int next_index = next_edge_[index];
        DPoint prev = points_[prev_index];
        DPoint next = points_[next_index];

        PointEvent point_event = pointEvent(index);
        if (point_event == PointEvent::Begin) {
          areas.insert(ScanLineArea(index, point, prev_index, prev));
          areas.insert(ScanLineArea(index, point, next_index, next));
        }
        else if (point_event == PointEvent::End) {
          areas.erase(ScanLineArea(prev_index, prev, index, point));
          areas.erase(ScanLineArea(next_index, next, index, point));
        }
        else {
          if (compareIndices(prev_index, next_index) < 0.0) {
            areas.erase(ScanLineArea(prev_index, prev, index, point));
            areas.insert(ScanLineArea(index, point, next_index, next));
          }
          else {
            areas.erase(ScanLineArea(next_index, next, index, point));
            areas.insert(ScanLineArea(index, point, prev_index, prev));
          }
        }

        VISAGE_ASSERT(checkValidPolygons());
        VISAGE_ASSERT(areas.size() % 2 == 0);
      }

      if (fill_rule != Path::FillRule::EvenOdd) {
        for (int i = 0; i < points_.size(); ++i) {
          if (next_edge_[i] == i)
            continue;

          bool empty = windings[i] < minimum_cycles;
          bool was_empty = (windings[i] - winding_directions[i]) < minimum_cycles;

          if (empty == was_empty)
            removeCycle(i);
        }
      }
    }

    void reverse() {
      for (int i = 0; i < points_.size(); ++i)
        std::swap(prev_edge_[i], next_edge_[i]);
    }

    void breakSimpleIntoMonotonicPolygons() {
      std::map<ScanLineArea, int> current_areas;
      auto indices = sortedIndices();
      std::set<int> merge_vertices;

      for (int index : indices) {
        DPoint point = points_[index];
        if (index == next_edge_[index] || prev_edge_[index] == next_edge_[index])
          continue;

        auto area = std::lower_bound(current_areas.begin(), current_areas.end(), point,
                                     [](const auto& pair, const DPoint& p) {
                                       return ScanLineArea::compare(pair.first, p);
                                     });

        bool convex = stableOrientation(point, points_[prev_edge_[index]], points_[next_edge_[index]]) > 0.0;

        int prev_index = prev_edge_[index];
        int next_index = next_edge_[index];
        DPoint prev = points_[prev_index];
        DPoint next = points_[next_index];

        PointEvent point_event = pointEvent(index);
        if (point_event == PointEvent::Begin) {
          int diagonal_index = index;
          if (!convex) {
            diagonal_index = addDiagonal(index, area->second);
            if (area != current_areas.end())
              area->second = index;
            area = std::prev(area);
            if (area != current_areas.end())
              area->second = diagonal_index;
          }

          auto area1 = ScanLineArea(index, point, prev_index, prev);
          auto area2 = ScanLineArea(diagonal_index, point, next_index, next);
          current_areas[area1] = index;
          current_areas[area2] = diagonal_index;
        }
        else if (point_event == PointEvent::End) {
          for (int i = 0; i < 2 && area != current_areas.end();) {
            if (area->first.to_index == index) {
              ++i;
              if (i == 0 && merge_vertices.count(area->second))
                addDiagonal(index, area->second);
              area = current_areas.erase(area);
            }
            else
              ++area;
          }

          if (area != current_areas.end()) {
            if (!convex) {
              area->second = index;
              area = std::prev(area);
              if (area != current_areas.end())
                area->second = index;
              merge_vertices.insert(index);
            }
          }
        }
        else {
          while (area != current_areas.end() && area->first.to_index != index)
            ++area;

          if (compareIndices(prev_index, next_index) < 0.0)
            current_areas[ScanLineArea(index, point, next_index, next)] = index;
          else {
            if (merge_vertices.count(area->second))
              addDiagonal(index, area->second);
            current_areas[ScanLineArea(index, point, prev_index, prev)] = index;
          }
          current_areas.erase(area);
        }

        VISAGE_ASSERT(checkValidPolygons());
        VISAGE_ASSERT(checkValidOrder(current_areas));
        VISAGE_ASSERT(current_areas.size() % 2 == 0);
      }

      VISAGE_ASSERT(current_areas.empty());
      simplify();
    }

    std::vector<int> breakIntoTriangles() {
      // TODO switch to Delaunay triangulation
      std::vector<int> triangles;
      auto sorted_indices = sortedIndices();
      std::unique_ptr<bool[]> touched = std::make_unique<bool[]>(points_.size());

      for (int index : sorted_indices) {
        touched[index] = true;
        cutEars(index, triangles, touched);
      }

      return triangles;
    }

    void combine(const TriangulationGraph& other) {
      int offset = points_.size();
      for (int i = 0; i < other.points_.size(); ++i) {
        points_.push_back(other.points_[i]);
        prev_edge_.push_back(other.prev_edge_[i] + offset);
        next_edge_.push_back(other.next_edge_[i] + offset);
      }

      VISAGE_ASSERT(checkValidPolygons());
    }

    void simplify() {
      std::unique_ptr<bool[]> visited = std::make_unique<bool[]>(points_.size());
      for (int i = 0; i < points_.size(); ++i) {
        if (i == next_edge_[i])
          continue;

        if (points_[i] == points_[next_edge_[i]])
          removeFromCycle(i);
        else {
          while (i != next_edge_[i]) {
            double orientation = stableOrientation(points_[i], points_[next_edge_[i]],
                                                   points_[next_edge_[next_edge_[i]]]);
            if (orientation)
              break;

            removeFromCycle(next_edge_[i]);
          }
        }
      }
    }

    Path toPath() {
      std::unique_ptr<bool[]> visited = std::make_unique<bool[]>(points_.size());
      Path path;
      for (int i = 0; i < points_.size(); ++i) {
        if (i == next_edge_[i] || visited[i])
          continue;

        int index = i;
        path.moveTo(Point(points_[i]));
        while (!visited[index]) {
          visited[index] = true;
          index = next_edge_[index];
          path.lineTo(Point(points_[index]));
        }

        path.close();
      }
      return path;
    }

  private:
    enum class PointEvent {
      Continue,
      Begin,
      End,
    };

    PointEvent pointEvent(int index) const {
      double compare_prev = compareIndices(index, prev_edge_[index]);
      double compare_next = compareIndices(index, next_edge_[index]);
      VISAGE_ASSERT(compare_prev && compare_next);

      if (compare_prev < 0.0 && compare_next < 0.0)
        return PointEvent::Begin;
      if (compare_prev > 0.0 && compare_next > 0.0)
        return PointEvent::End;
      return PointEvent::Continue;
    }

    int addAdditionalPoint(DPoint point) {
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

    bool checkValidPolygons() const {
      for (int i = 0; i < points_.size(); ++i) {
        if (prev_edge_[next_edge_[i]] != i || next_edge_[prev_edge_[i]] != i)
          return false;
      }
      return true;
    }

    bool checkValidOrder(const std::map<ScanLineArea, int>& areas) const {
      if (areas.empty())
        return true;

      auto start = areas.begin();
      for (auto offset = std::next(start); offset != areas.end(); ++offset) {
        auto it = start;
        for (auto next = offset; next != areas.end(); ++it, ++next) {
          if (!(*it < *next))
            return false;
        }
      }
      return true;
    }

    std::vector<int> sortedIndices() const {
      std::vector<int> sorted_indices;
      sorted_indices.resize(prev_edge_.size());
      std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

      std::sort(sorted_indices.begin(), sorted_indices.end(),
                [this](const int a, const int b) { return compareIndices(a, b) < 0.0; });
      return sorted_indices;
    }

    std::optional<std::pair<int, int>> breakIntersection(int start_index1, int end_index1,
                                                         int start_index2, int end_index2) {
      DPoint start1 = points_[start_index1];
      DPoint end1 = points_[end_index1];
      DPoint start2 = points_[start_index2];
      DPoint end2 = points_[end_index2];

      std::optional<DPoint> intersection = Path::findIntersection(start1, end1, start2, end2);
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

      return IntersectionEvent { intersection.value(),
                                 it->to_index,
                                 next_edge_[it->from_index] == it->to_index,
                                 next->to_index,
                                 next_edge_[next->from_index] == next->to_index,
                                 this };
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

    void addIntersectionArea(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas,
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

    void addIntersectionArea(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas,
                             int index, DPoint point, int end_index1, DPoint end1,
                             bool check_remove = true) const {
      addIntersectionArea(events, areas, ScanLineArea(index, points_[index], end_index1, end1), check_remove);
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
      areas.erase(erase2);
      VISAGE_ASSERT(areas.size() % 2 == 0);

      int a_index = connected(broken->first, e.a_to) ? broken->first : broken->second;
      int b_index = connected(broken->first, e.b_to) ? broken->first : broken->second;
      ScanLineArea area1(a_index, points_[a_index], e.a_to, points_[e.a_to]);
      ScanLineArea area2(b_index, points_[b_index], e.b_to, points_[e.b_to]);
      addIntersectionArea(events, areas, area1, false);
      addIntersectionArea(events, areas, area2);
      VISAGE_ASSERT(areas.size() % 2 == 0);
    }

    void handlePointEvent(std::set<IntersectionEvent>& events, std::set<ScanLineArea>& areas,
                          int index) const {
      DPoint point = points_[index];
      int prev_index = prev_edge_[index];
      int next_index = next_edge_[index];
      if (next_index == index || prev_index == next_index)
        return;

      DPoint prev = points_[prev_index];
      DPoint next = points_[next_index];

      PointEvent point_event = pointEvent(index);
      if (point_event == PointEvent::Begin) {
        addIntersectionArea(events, areas, index, point, prev_index, prev);
        addIntersectionArea(events, areas, index, point, next_index, next);
      }
      else if (point_event == PointEvent::End) {
        areas.erase(ScanLineArea(prev_index, prev, index, point));
        auto area = areas.erase(areas.find(ScanLineArea(next_index, next, index, point)));
        if (area != areas.end() && area != areas.begin())
          checkAddIntersection(events, areas, std::prev(area));
      }
      else {
        if (compareIndices(prev_index, next_index) < 0.0) {
          areas.erase(ScanLineArea(prev_index, prev, index, point));
          addIntersectionArea(events, areas, index, point, next_index, next, false);
        }
        else {
          areas.erase(ScanLineArea(next_index, next, index, point));
          addIntersectionArea(events, areas, index, point, prev_index, prev, false);
        }
      }

      VISAGE_ASSERT(checkValidPolygons());
      VISAGE_ASSERT(areas.size() % 2 == 0);
    }

    void removeCycle(int start_index) {
      for (int i = start_index; next_edge_[i] != i;) {
        int next = next_edge_[i];
        prev_edge_[i] = i;
        next_edge_[i] = i;
        i = next;
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

    bool tryCutEar(int index, bool forward, std::vector<int>& triangles, std::unique_ptr<bool[]>& touched) {
      auto& direction = forward ? next_edge_ : prev_edge_;
      auto& reverse = forward ? prev_edge_ : next_edge_;

      int intermediate_index = direction[index];
      int target_index = direction[intermediate_index];
      if (intermediate_index == index || target_index == index || !touched[intermediate_index] ||
          !touched[target_index]) {
        return false;
      }

      DPoint start = points_[index];
      DPoint intermediate = points_[intermediate_index];
      DPoint target = points_[target_index];

      double orientation = stableOrientation(start, intermediate, target);
      if ((orientation < 0.0) != forward && orientation)
        return false;

      if (orientation) {
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

    void cutEars(int index, std::vector<int>& triangles, std::unique_ptr<bool[]>& touched) {
      while (tryCutEar(index, true, triangles, touched))
        ;
      while (tryCutEar(index, false, triangles, touched))
        ;
    }

    std::vector<DPoint> points_;
    std::vector<int> prev_edge_;
    std::vector<int> next_edge_;
  };

  Path::Triangulation Path::triangulate() const {
    TriangulationGraph graph(this);
    return graph.triangulate(fillRule());
  }

  Path Path::computeUnion(const Path& other) const {
    return computeCombo(other, Path::FillRule::NonZero, 1, false);
  }

  Path Path::computeIntersection(const Path& other) const {
    return computeCombo(other, Path::FillRule::NonZero, 2, false);
  }

  Path Path::computeDifference(const Path& other) const {
    return computeCombo(other, Path::FillRule::Positive, 1, true);
  }

  Path Path::computeXor(const Path& other) const {
    return computeCombo(other, Path::FillRule::EvenOdd, 1, false);
  }

  Path Path::computeCombo(const Path& other, Path::FillRule fill_rule, int num_cycles_needed,
                          bool reverse_other) const {
    TriangulationGraph graph(this);
    TriangulationGraph other_graph(&other);
    graph.simplify();
    graph.breakIntersections();
    graph.fixWindings(fillRule());

    other_graph.simplify();
    other_graph.breakIntersections();
    other_graph.fixWindings(other.fillRule());
    if (reverse_other)
      other_graph.reverse();

    graph.combine(other_graph);
    graph.breakIntersections();
    graph.fixWindings(fill_rule, num_cycles_needed);
    return graph.toPath();
  }
}
