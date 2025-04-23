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
    static double compare(const ScanLineArea& area, DPoint point) {
      return stableOrientation(area.from, area.to, point);
    }

    static bool lessThan(const ScanLineArea& area, DPoint point) {
      return compare(area, point) > 0.0;
    }

    ScanLineArea(int from_index, DPoint from, int to_index, DPoint to) :
        from_index(from_index), from(from), to_index(to_index), to(to) { }

    bool operator<(const ScanLineArea& other) const {
      double orientation = 0.0;
      if (other.from < from)
        orientation = stableOrientation(other.from, other.to, from);
      else if (from < other.from)
        orientation = -stableOrientation(from, to, other.from);

      if (orientation)
        return orientation < 0.0;

      if (from.y != other.from.y)
        return from.y < other.from.y;

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

    int from_index;
    DPoint from;
    int to_index;
    DPoint to;
  };

  class TriangulationGraph {
  public:
    enum class PointType {
      Continue,
      Begin,
      End
    };

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
      double compare = points_[a_index].compare(points_[b_index]);
      if (compare)
        return compare;

      int a_prev = a_index;
      int a_next = a_index;
      int b_prev = b_index;
      int b_next = b_index;

      do {
        a_prev = prev_edge_[a_prev];
        a_next = next_edge_[a_next];

        b_prev = prev_edge_[b_prev];
        b_next = next_edge_[b_next];

        double compare_prev = points_[a_prev].compare(points_[b_prev]);
        double compare_next = points_[a_next].compare(points_[b_next]);
        if (compare_prev == 0.0 && compare_next == 0.0)
          continue;
        if (compare_prev <= 0.0 && compare_next <= 0.0)
          return -1.0;
        if (compare_prev >= 0.0 && compare_next >= 0.0)
          return 1.0;
        if (compare_prev == 0.0 || compare_next == 0.0)
          return compare_prev + compare_next;

        double compare_a = points_[a_prev].compare(points_[a_next]);
        double compare_b = points_[b_prev].compare(points_[b_next]);

        if (compare_a < 0.0 && compare_b < 0.0)
          return compare_prev;
        if (compare_a > 0.0 && compare_b > 0.0)
          return compare_next;

      } while (next_edge_[a_next] != a_prev && next_edge_[b_next] != b_prev &&
               next_edge_[a_next] != prev_edge_[a_prev] && next_edge_[b_next] != prev_edge_[b_prev]);
      return a_index - b_index;
    }

    struct Degeneracy {
      DPoint from, to, point;
      bool operator<(const Degeneracy& other) const {
        if (from != other.from)
          return from < other.from;
        if (to != other.to)
          return to < other.to;
        return point < other.point;
      }
    };

    struct ScanLinePair {
      int scan_line_id1, scan_line_id2;

      ScanLinePair(int id1, int id2) {
        scan_line_id1 = std::min(id1, id2);
        scan_line_id2 = std::max(id1, id2);
      }

      bool operator<(const ScanLinePair& other) const {
        if (scan_line_id1 != other.scan_line_id1)
          return scan_line_id1 < other.scan_line_id1;
        return scan_line_id2 < other.scan_line_id2;
      }
    };

    class ScanLine {
    public:
      struct Event {
        PointType type;
        int index;
        DPoint point;
        int prev_index;
        DPoint prev;
        int next_index;
        DPoint next;
      };

      ScanLine(TriangulationGraph* graph, bool find_intersections = false) :
          graph_(graph), find_intersections_(find_intersections),
          sorted_indices_(graph->sortedIndices()) {
        progressToNextEvent();
      }

      bool hasNext() { return event_index_ < sorted_indices_.size(); }
      void progressToNextEvent() {
        while (event_index_ < sorted_indices_.size()) {
          int index = sorted_indices_[event_index_];
          if (graph_->next_edge_[index] != index)
            break;
          event_index_++;
        }
      }

      int resolveAlias(int index) const {
        if (aliases_.count(index))
          return aliases_.at(index);
        return index;
      }

      void addAlias(int alias, int original) {
        while (aliases_.count(original))
          original = aliases_[original];

        aliases_[alias] = original;
      }

      Event nextEvent() const {
        int index = sorted_indices_[event_index_];
        int prev_index = resolveAlias(graph_->prev_edge_[index]);
        int next_index = resolveAlias(graph_->next_edge_[index]);
        auto point = graph_->points_[index];
        auto prev = graph_->points_[prev_index];
        auto next = graph_->points_[next_index];
        auto type = graph_->pointType(index);
        return { type, index, point, prev_index, prev, next_index, next };
      }

      std::optional<IntersectionEvent> adjacentIntersection(const std::map<ScanLineArea, int>::iterator& it) const {
        if (it == areas_.end())
          return std::nullopt;

        auto next_it = std::next(it);
        if (next_it == areas_.end())
          return std::nullopt;

        const ScanLineArea& area = it->first;
        const ScanLineArea& next = next_it->first;
        auto intersection = Path::findIntersection(area.from, area.to, next.from, next.to);
        if (!intersection.has_value())
          return std::nullopt;

        return IntersectionEvent { intersection.value(),
                                   area.to_index,
                                   graph_->next_edge_[area.from_index] == area.to_index,
                                   next.to_index,
                                   graph_->next_edge_[next.from_index] == next.to_index,
                                   graph_ };
      }

      void checkRemoveIntersection(const std::map<ScanLineArea, int>::iterator& it) {
        auto intersection = adjacentIntersection(it);
        if (intersection.has_value()) {
          VISAGE_ASSERT(intersection_events_.count(intersection.value()));
          intersection_events_.erase(intersection.value());
        }
      };

      auto addArea(const ScanLineArea& area, int id, bool check_remove = true) {
        if (find_intersections_ && check_remove) {
          auto adjacent = areas_.lower_bound(area);
          if (adjacent != areas_.end() && adjacent != areas_.begin())
            checkRemoveIntersection(std::prev(adjacent));
        }

        auto it = areas_.insert({ area, id }).first;
        last_id_ = id;
        if (find_intersections_) {
          checkAddIntersection(it);
          checkAddIntersection(std::prev(it));
        }
        return it;
      }

      void checkAddIntersection(const std::map<ScanLineArea, int>::iterator& it) {
        auto intersection = adjacentIntersection(it);
        if (intersection.has_value()) {
          auto next = std::next(it);
          intersection_events_.insert(intersection.value());
        }
      };

      void updateBeginAreas(const Event& ev) {
        ScanLineArea to_insert1(ev.index, ev.point, ev.prev_index, ev.prev);
        ScanLineArea to_insert2(ev.index, ev.point, ev.next_index, ev.next);
        if (to_insert2 < to_insert1)
          std::swap(to_insert1, to_insert2);

        last_position1_ = addArea(to_insert1, -1);
        last_position2_ = addArea(to_insert2, -1);
      }

      void updateEndAreas(const Event& ev) {
        areas_.erase(ScanLineArea(ev.prev_index, ev.prev, ev.index, ev.point));
        ScanLineArea to_erase(ev.next_index, ev.next, ev.index, ev.point);
        last_id_ = areas_[to_erase];
        last_position1_ = areas_.erase(areas_.find(to_erase));
        last_position2_ = last_position1_;
        if (find_intersections_ && last_position1_ != areas_.end() && last_position1_ != areas_.begin())
          checkAddIntersection(std::prev(last_position1_));
      }

      void updateAreaIntersection(const IntersectionEvent& ev) {
        int a_from = ev.a_forward ? graph_->prev_edge_[ev.a_to] : graph_->next_edge_[ev.a_to];
        int b_from = ev.b_forward ? graph_->prev_edge_[ev.b_to] : graph_->next_edge_[ev.b_to];
        std::optional<std::pair<int, int>> broken = graph_->breakIntersection(a_from, ev.a_to,
                                                                              b_from, ev.b_to);
        VISAGE_ASSERT(broken.has_value());

        ScanLineArea erase_area1(a_from, graph_->points_[a_from], ev.a_to, graph_->points_[ev.a_to]);
        ScanLineArea erase_area2(b_from, graph_->points_[b_from], ev.b_to, graph_->points_[ev.b_to]);
        if (erase_area2 < erase_area1)
          std::swap(erase_area1, erase_area2);

        auto erase1 = areas_.find(erase_area1);
        checkRemoveIntersection(std::prev(erase1));
        areas_.erase(erase1);
        VISAGE_ASSERT(areas_.size() % 2 == 1);

        auto erase2 = areas_.find(erase_area2);
        checkRemoveIntersection(erase2);
        areas_.erase(erase2);
        VISAGE_ASSERT(areas_.size() % 2 == 0);

        int a_index = graph_->connected(broken->first, ev.a_to) ? broken->first : broken->second;
        int b_index = graph_->connected(broken->first, ev.b_to) ? broken->first : broken->second;
        ScanLineArea area1(a_index, graph_->points_[a_index], ev.a_to, graph_->points_[ev.a_to]);
        ScanLineArea area2(b_index, graph_->points_[b_index], ev.b_to, graph_->points_[ev.b_to]);
        last_position1_ = addArea(area1, -1, false);
        last_position2_ = addArea(area2, -1);
        VISAGE_ASSERT(areas_.size() % 2 == 0);
      }

      void updateContinueArea(const Event& ev) {
        int from_index = ev.prev_index;
        int to_index = ev.next_index;
        if (graph_->compareIndices(ev.prev_index, ev.next_index) > 0.0)
          std::swap(from_index, to_index);

        ScanLineArea to_erase(from_index, graph_->points_[from_index], ev.index, ev.point);
        ScanLineArea to_insert(ev.index, ev.point, to_index, graph_->points_[to_index]);
        last_id_ = areas_[to_erase];
        areas_.erase(to_erase);
        last_position1_ = addArea(to_insert, last_id_, false);
        last_position2_ = last_position1_;
      }

      void update() {
        Event ev = nextEvent();

        if (!intersection_events_.empty()) {
          auto it = intersection_events_.begin();
          IntersectionEvent intersection = *it;

          if (intersection.point < ev.point) {
            intersection_events_.erase(it);
            updateAreaIntersection(intersection);
            return;
          }
        }

        if (ev.type == PointType::Begin)
          updateBeginAreas(ev);
        else if (ev.type == PointType::End)
          updateEndAreas(ev);
        else
          updateContinueArea(ev);

        event_index_++;
        progressToNextEvent();

        VISAGE_ASSERT(graph_->checkValidPolygons());
        VISAGE_ASSERT(checkValidOrder(areas_));
        VISAGE_ASSERT(areas_.size() % 2 == 0);
      }

      auto lowerBound(DPoint point) {
        return std::lower_bound(areas_.begin(), areas_.end(), point, [](const auto& pair, const DPoint& p) {
          return ScanLineArea::lessThan(pair.first, p);
        });
      }

      auto end() const { return areas_.end(); }
      int lastId() const { return last_id_; }
      auto lastPosition1() const { return last_position1_; }
      auto lastPosition2() const { return last_position2_; }

    private:
      TriangulationGraph* graph_ = nullptr;
      std::map<ScanLineArea, int> areas_;
      std::vector<int> sorted_indices_;
      int event_index_ = 0;

      std::map<ScanLineArea, int>::iterator last_position1_;
      std::map<ScanLineArea, int>::iterator last_position2_;
      std::map<int, int> aliases_;

      bool find_intersections_ = false;
      std::set<IntersectionEvent> intersection_events_;
      int last_id_ = 0;
    };

    void breakIntersections() {
      VISAGE_ASSERT(checkValidPolygons());
      ScanLine scan_line(this, true);

      while (scan_line.hasNext())
        scan_line.update();

      simplify();
    }

    void fixWindings(Path::FillRule fill_rule, int minimum_cycles = 1) {
      ScanLine scan_line(this);
      auto windings = std::make_unique<int[]>(points_.size());
      auto reversed = std::make_unique<bool[]>(points_.size());
      auto winding_directions = std::make_unique<int[]>(points_.size());

      while (scan_line.hasNext()) {
        auto ev = scan_line.nextEvent();
        if (winding_directions[ev.index] == 0) {
          bool convex = stableOrientation(ev.point, ev.prev, ev.next) > 0.0;

          auto area_iterator = scan_line.lowerBound(ev.point);

          int current_winding = 0;
          bool reverse = false;
          if (area_iterator != scan_line.end()) {
            const auto& area = area_iterator->first;
            bool area_fill_above = next_edge_[area.from_index] == area.to_index;
            bool fill = winding_directions[area.from_index] == 1;
            current_winding += windings[area.from_index];
            bool inside_polygon = area_fill_above == fill;

            if (inside_polygon) {
              if (reversed[area.from_index] && fill_rule != Path::FillRule::EvenOdd)
                reverse = true;
            }
            else
              current_winding -= winding_directions[area.from_index];
          }

          if (fill_rule == Path::FillRule::EvenOdd) {
            bool fill = (current_winding % 2) == 0;
            reverse = reverse || (convex != fill);
          }
          else if (fill_rule == Path::FillRule::NonZero && current_winding == 0)
            reverse = reverse || !convex;

          if (reverse) {
            reverseCycle(ev.index);
            convex = !convex;
          }

          int winding_direction = convex ? 1 : -1;
          int winding = current_winding + winding_direction;
          for (int i = ev.index; winding_directions[i] == 0; i = next_edge_[i]) {
            winding_directions[i] = winding_direction;
            windings[i] = winding;
            reversed[i] = reverse;
          }
        }

        scan_line.update();
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
      ScanLine scan_line(this);
      std::set<int> merge_vertices;

      while (scan_line.hasNext()) {
        auto ev = scan_line.nextEvent();

        bool convex = stableOrientation(ev.point, ev.prev, ev.next) > 0.0;
        if (ev.type == PointType::Begin) {
          scan_line.update();

          auto after = scan_line.lastPosition2();
          after->second = ev.index;

          int diagonal_index = ev.index;
          if (!convex) {
            after = std::next(after);
            diagonal_index = addDiagonal(scan_line, ev.index, after->second);
            after->second = ev.index;

            auto before = std::prev(scan_line.lastPosition1());
            before->second = diagonal_index;
          }

          scan_line.lastPosition1()->second = diagonal_index;
        }
        else if (ev.type == PointType::End) {
          scan_line.update();
          auto& area = scan_line.lowerBound(ev.point);

          if (convex && merge_vertices.count(scan_line.lastId()))
            addDiagonal(scan_line, ev.index, scan_line.lastId());

          if (area != scan_line.end()) {
            if (!convex) {
              area->second = ev.index;
              area = std::prev(area);
              if (area != scan_line.end())
                area->second = ev.index;
              merge_vertices.insert(ev.index);
            }
          }
        }
        else {
          bool reversed = compareIndices(ev.prev_index, ev.next_index) > 0.0;
          scan_line.update();

          int diagonal_target = scan_line.lastId();
          if (reversed && merge_vertices.count(diagonal_target))
            addDiagonal(scan_line, ev.index, diagonal_target);

          scan_line.lastPosition1()->second = ev.index;
        }
      }

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
    PointType pointType(int index) const {
      double compare_prev = compareIndices(index, prev_edge_[index]);
      double compare_next = compareIndices(index, next_edge_[index]);
      VISAGE_ASSERT(compare_prev && compare_next);

      if (compare_prev < 0.0 && compare_next < 0.0)
        return PointType::Begin;
      if (compare_prev > 0.0 && compare_next > 0.0)
        return PointType::End;
      return PointType::Continue;
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

    static bool checkValidOrder(const std::map<ScanLineArea, int>& areas) {
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

    int addDiagonal(ScanLine& scan_line, int index, int target) {
      int new_index = prev_edge_.size();
      int new_diagonal_index = new_index + 1;
      scan_line.addAlias(new_index, index);
      scan_line.addAlias(new_diagonal_index, target);
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
