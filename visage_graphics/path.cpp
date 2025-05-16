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

#include <complex>
#include <map>
#include <memory>
#include <set>

namespace visage {
  void Path::arcTo(float x_radius, float y_radius, float x_axis_rotation, bool large_arc,
                   bool sweep_flag, Point to, bool relative) {
    static constexpr float kPi = 3.14159265358979323846f;

    smooth_control_point_ = {};

    Point from = lastPoint();
    if (relative)
      to += from;

    auto ellipse_rotation = Matrix::rotation(x_axis_rotation * kPi / 180.0f);
    Point delta = ellipse_rotation.transpose() * (to - from);
    float radius_ratio = x_radius / y_radius;
    delta.y *= radius_ratio;

    float length = delta.length();
    float radius = std::max(length * 0.5f, x_radius);
    float center_offset = std::sqrt(radius * radius - length * length * 0.25f);
    Point normal = Point(delta.y, -delta.x) / length;
    if (large_arc != sweep_flag)
      normal = -normal;

    Point center = delta * 0.5f + normal * center_offset;
    float arc_angle = 2.0f * std::asin(length * 0.5f / radius);

    if (large_arc)
      arc_angle = 2.0f * kPi - arc_angle;
    if (!sweep_flag)
      arc_angle = -arc_angle;

    float max_delta_radians = 2.0f * std::acos(1.0f - error_tolerance_ / std::max(x_radius, y_radius));
    int num_points = std::ceil(std::abs(arc_angle) / max_delta_radians);

    std::complex<double> position(-center.x, -center.y);
    double angle_delta = arc_angle / num_points;
    std::complex<double> rotation = std::polar(1.0, angle_delta);

    for (int i = 0; i < num_points; ++i) {
      position *= rotation;
      Point point = center + Point(position.real(), position.imag());
      point.y /= radius_ratio;
      point = ellipse_rotation * point + from;
      addPoint(point);
    }
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
    startNewPath();

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
    DPoint delta1 = target1 - source;
    DPoint delta2 = target2 - source;
    double l = delta2.y * delta1.x;
    double r = delta2.x * delta1.y;
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
    ScanLineArea(int from_index, const DPoint& from, int to_index, const DPoint& to, bool forward,
                 int id = -1) :
        from_index(from_index), from(from), to_index(to_index), to(to), forward(forward), id(id) { }

    bool operator==(const ScanLineArea& other) const {
      return from_index == other.from_index && to_index == other.to_index && from == other.from &&
             to == other.to;
    }

    bool operator<(const ScanLineArea& other) const {
      double orientation = 0.0;
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

      if (id != other.id)
        return id < other.id;
      if (from_index != other.from_index)
        return from_index < other.from_index;
      return to_index < other.to_index;
    }

    int from_index;
    DPoint from;
    int to_index;
    DPoint to;
    bool forward;
    int id;
  };

  class TriangulationGraph {
  public:
    enum class PointType {
      Begin,
      Continue,
      End
    };

    TriangulationGraph() = delete;

    explicit TriangulationGraph(const Path* path) {
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

    bool lessThan(int a_index, int b_index) const {
      double compare = points_[a_index].compare(points_[b_index]);
      if (compare)
        return compare < 0.0;

      PointType a_type = pointType(a_index);
      PointType b_type = pointType(b_index);
      if (a_type != b_type)
        return a_type == PointType::End || b_type == PointType::Begin;

      return a_index < b_index;
    }

    template<bool find_intersections = false>
    class ScanLine {
    public:
      struct Event {
        Event(PointType type, int index, const DPoint& point, int prev_index, const DPoint& prev,
              int next_index, const DPoint& next, bool degeneracy) :
            type(type), index(index), point(point), prev_index(prev_index), prev(prev),
            next_index(next_index), next(next), degeneracy(degeneracy) { }

        PointType type;
        int index;
        DPoint point;
        int prev_index;
        DPoint prev;
        int next_index;
        DPoint next;
        bool degeneracy;
      };

      ScanLine() = delete;

      explicit ScanLine(TriangulationGraph* graph) :
          graph_(graph), sorted_indices_(graph->sortedIndices()) {
        progressToNextEvent();
      }

      bool hasNext() const { return current_index_ < sorted_indices_.size(); }
      void progressToNextEvent() {
        while (current_index_ < sorted_indices_.size()) {
          int index = sorted_indices_[current_index_];
          if (graph_->next_edge_[index] != index)
            break;
          current_index_++;
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
        int index = sorted_indices_[current_index_];
        int prev_index = resolveAlias(graph_->prev_edge_[index]);
        int next_index = resolveAlias(graph_->next_edge_[index]);
        auto point = graph_->points_[index];
        auto prev = graph_->points_[prev_index];
        auto next = graph_->points_[next_index];
        auto type = graph_->pointType(index);
        bool degeneracy = current_index_ + 1 < sorted_indices_.size() &&
                          graph_->points_[sorted_indices_[current_index_ + 1]] == point;
        return { type, index, point, prev_index, prev, next_index, next, degeneracy };
      }

      struct Break {
        Break() = delete;
        DPoint point;
        int area1_new_index;
        int area2_new_index;
      };

      std::optional<Break> breakIntersection(const ScanLineArea& area1, const ScanLineArea& area2) {
        if (area1.from == area2.from || area1.to == area2.to)
          return std::nullopt;

        double compare1 = stableOrientation(area1.from, area1.to, area2.to);
        double compare2 = stableOrientation(area2.from, area2.to, area1.to);
        if (compare1 > 0.0 || compare2 < 0.0)
          return std::nullopt;

        int area1_prev = area1.forward ? area1.from_index : area1.to_index;
        int area1_next = area1.forward ? area1.to_index : area1.from_index;

        int area2_prev = area2.forward ? area2.from_index : area2.to_index;
        int area2_next = area2.forward ? area2.to_index : area2.from_index;

        if (compare1 == 0.0 && area2.to < area1.to) {
          int new_index = graph_->insertPointBetween(area1_prev, area1_next, area2.to);
          return Break { area2.to, new_index, area2.to_index };
        }
        if (compare2 == 0.0 && area1.to < area2.to) {
          int new_index = graph_->insertPointBetween(area2_prev, area2_next, area1.to);
          return Break { area1.to, area1.to_index, new_index };
        }

        std::optional<DPoint> intersection = Path::findIntersection(area1.from, area1.to,
                                                                    area2.from, area2.to);
        VISAGE_ASSERT(intersection.has_value());
        int new_index1 = area1.to_index;
        int new_index2 = area2.to_index;
        if (intersection.value() != area1.from && intersection.value() != area1.to)
          new_index1 = graph_->insertPointBetween(area1_prev, area1_next, intersection.value());
        if (intersection.value() != area2.from && intersection.value() != area2.to)
          new_index2 = graph_->insertPointBetween(area2_prev, area2_next, intersection.value());

        return Break { intersection.value(), new_index1, new_index2 };
      }

      struct IntersectionEvent {
        IntersectionEvent() = delete;
        DPoint point;
        int area1_from_index;
        int area1_to_index;
        int area2_from_index;
        int area2_to_index;

        bool operator<(const IntersectionEvent& other) const {
          if (point != other.point)
            return point < other.point;
          if (area1_from_index != other.area1_from_index)
            return area1_from_index < other.area1_from_index;
          return area2_from_index < other.area2_from_index;
        }
      };

      void addArea(const ScanLineArea& area) {
        VISAGE_ASSERT(area.from_index != area.to_index);

        auto it = areas_.insert(std::lower_bound(areas_.begin(), areas_.end(), area), area);
        auto before = safePrev(it);
        checkAddIntersection(before);
        checkAddIntersection(it);
        last_id_ = area.id;
        VISAGE_ASSERT(graph_->checkValidPolygons());
      }

      void checkAddIntersection(std::vector<ScanLineArea>::iterator it) {
        if (it == areas_.end())
          return;
        auto next = std::next(it);
        if (next == areas_.end())
          return;

        auto intersection = breakIntersection(*it, *next);
        if (!intersection.has_value())
          return;

        intersection_events_.insert(IntersectionEvent {
            intersection.value().point, intersection.value().area1_new_index, it->to_index,
            intersection.value().area2_new_index, next->to_index });

        it->to_index = intersection.value().area1_new_index;
        it->to = intersection.value().point;
        next->to_index = intersection.value().area2_new_index;
        next->to = intersection.value().point;
      }

      void checkForBeginIntersections(const Event& ev) {
        if (ev.type != PointType::Begin)
          return;

        auto lower_bound = std::lower_bound(areas_.begin(), areas_.end(), ev.point,
                                            [](const auto& area, const DPoint& point) {
                                              return stableOrientation(area.from, area.to, point) > 0.0;
                                            });

        while (lower_bound != areas_.end() &&
               stableOrientation(lower_bound->from, lower_bound->to, ev.point) == 0.0) {
          if (lower_bound->from != ev.point && lower_bound->to != ev.point) {
            int prev = lower_bound->forward ? lower_bound->from_index : lower_bound->to_index;
            int next = lower_bound->forward ? lower_bound->to_index : lower_bound->from_index;
            int new_index = graph_->insertPointBetween(prev, next, ev.point);

            intersection_events_.insert(IntersectionEvent { ev.point, new_index,
                                                            lower_bound->to_index, 0, 0 });
            lower_bound->to_index = new_index;
            lower_bound->to = ev.point;
          }
          ++lower_bound;
        }

        VISAGE_ASSERT(graph_->checkValidPolygons());
      }

      DPoint checkForIntersections(const DPoint& point) {
        DPoint result_point = point;
        while (!intersection_events_.empty() && intersection_events_.begin()->point <= result_point) {
          result_point = intersection_events_.begin()->point;
          auto it = intersection_events_.begin();
          if (it->area1_from_index != it->area1_to_index) {
            auto old = findAreaByEndIndex(it->area1_from_index);
            old_areas_.push_back(*old);
            new_areas_.emplace_back(old->to_index, old->to, it->area1_to_index,
                                    graph_->points_[it->area1_to_index],
                                    graph_->next_edge_[old->to_index] == it->area1_to_index, old->id);
            last_position1_ = areas_.erase(old);
          }
          if (it->area2_from_index != it->area2_to_index) {
            auto old = findAreaByEndIndex(it->area2_from_index);
            old_areas_.push_back(*old);
            new_areas_.emplace_back(old->to_index, old->to, it->area2_to_index,
                                    graph_->points_[it->area2_to_index],
                                    graph_->next_edge_[old->to_index] == it->area2_to_index, old->id);
            last_position1_ = areas_.erase(old);
          }
          intersection_events_.erase(it);
        }

        return result_point;
      }

      void processPointEvents(Event ev, const DPoint& point) {
        while (ev.point == point) {
          if (ev.type == PointType::Continue) {
            int from_index = ev.prev_index;
            int to_index = ev.next_index;
            bool forward = ev.prev < ev.next;
            if (!forward)
              std::swap(from_index, to_index);

            auto old = findAreaByEndIndex(ev.index);
            old_areas_.push_back(*old);
            new_areas_.emplace_back(ev.index, ev.point, to_index, graph_->points_[to_index],
                                    forward, old->id);
            areas_.erase(old);
          }
          else if (ev.type == PointType::Begin) {
            area_index_++;
            new_areas_.emplace_back(ev.index, ev.point, ev.prev_index, ev.prev, false, -area_index_);
            new_areas_.emplace_back(ev.index, ev.point, ev.next_index, ev.next, true, area_index_);
          }
          else {
            auto old = findAreaByEndIndex(ev.index);
            old_areas_.push_back(*old);
            areas_.erase(old);
            old = findAreaByEndIndex(ev.index);
            old_areas_.push_back(*old);
            last_position1_ = areas_.erase(old);
          }

          current_index_++;
          progressToNextEvent();
          if (!hasNext())
            break;

          ev = nextEvent();
        }
      }

      template<typename HandlePair>
      void pairInsOuts(std::vector<ScanLineArea>& areas, HandlePair handle_pair) {
        std::sort(areas.begin(), areas.end());
        areas.erase(std::unique(areas.begin(), areas.end()), areas.end());

        for (auto it = areas.begin(); it != areas.end();) {
          auto next = std::next(it);
          if (next != areas.end() && it->forward != next->forward) {
            handle_pair(*it, *next);

            it = areas.erase(it);
            it = areas.erase(it);
            if (it == areas.end())
              break;
            if (it != areas.begin())
              it = std::prev(it);
          }
          else
            ++it;
        }
      }

      auto findAreaByEndIndex(int index) {
        for (auto it = areas_.begin(); it != areas_.end(); ++it) {
          if (it->to_index == index)
            return it;
        }

        return areas_.end();
      }

      void updateNormalEvent(const Event& ev) {
        if (ev.type == PointType::Continue) {
          last_position1_ = findAreaByEndIndex(ev.index);
          last_position2_ = last_position1_;
          last_id_ = last_position1_->id;

          bool forward = last_position1_->from_index == ev.prev_index;
          last_position1_->from_index = ev.index;
          last_position1_->from = ev.point;

          if (forward) {
            last_position1_->to_index = ev.next_index;
            last_position1_->to = ev.next;
          }
          else {
            last_position1_->to_index = ev.prev_index;
            last_position1_->to = ev.prev;
          }
        }
        else if (ev.type == PointType::Begin) {
          area_index_++;
          ScanLineArea to_insert1(ev.index, ev.point, ev.prev_index, ev.prev, false, -area_index_);
          ScanLineArea to_insert2(ev.index, ev.point, ev.next_index, ev.next, true, area_index_);
          if (to_insert2 < to_insert1)
            std::swap(to_insert1, to_insert2);

          auto lower_bound = std::lower_bound(areas_.begin(), areas_.end(), to_insert1);
          last_position1_ = areas_.insert(lower_bound, to_insert2);
          last_position1_ = areas_.insert(last_position1_, to_insert1);
          last_position2_ = std::next(last_position1_);
        }
        else {
          areas_.erase(findAreaByEndIndex(ev.index));
          auto to_erase = findAreaByEndIndex(ev.index);
          last_id_ = to_erase->id;
          last_position1_ = areas_.erase(to_erase);
          last_position2_ = last_position1_;
        }

        current_index_++;
        progressToNextEvent();

        VISAGE_ASSERT(areas_.size() % 2 == 0);
      }

      void updateDegeneracy(const Event& ev, const DPoint& point) {
        processPointEvents(ev, point);

        std::sort(old_areas_.begin(), old_areas_.end());
        std::sort(new_areas_.begin(), new_areas_.end());

        degeneracies_.clear();
        for (auto& area : old_areas_) {
          if (area.forward)
            degeneracies_.push_back(area.to_index);
        }

        for (auto& area : new_areas_) {
          if (!area.forward)
            degeneracies_.push_back(area.from_index);
        }

        std::sort(degeneracies_.begin(), degeneracies_.end());

        int old_index = 0;
        pairInsOuts(old_areas_, [this, &old_index](const ScanLineArea& area1, const ScanLineArea& area2) {
          int index = degeneracies_[old_index++];
          if (area1.forward) {
            graph_->connect(area1.from_index, index);
            graph_->connect(index, area2.from_index);
          }
          else {
            graph_->connect(index, area1.from_index);
            graph_->connect(area2.from_index, index);
          }
        });

        next_areas_.clear();
        int new_index = degeneracies_.size() - 1;
        pairInsOuts(new_areas_, [this, &new_index](const ScanLineArea& area1, const ScanLineArea& area2) {
          int index = degeneracies_[new_index--];

          next_areas_.push_back(area1);
          next_areas_.back().from_index = index;
          next_areas_.push_back(area2);
          next_areas_.back().from_index = index;

          if (area1.forward) {
            graph_->connect(index, area1.to_index);
            graph_->connect(area2.to_index, index);
          }
          else {
            graph_->connect(area1.to_index, index);
            graph_->connect(index, area2.to_index);
          }
        });

        VISAGE_ASSERT(old_areas_.size() == new_areas_.size());
        auto new_area = new_areas_.begin();
        for (auto old_area = old_areas_.begin(); old_area != old_areas_.end(); ++old_area, ++new_area) {
          int index = degeneracies_[old_index++];
          next_areas_.push_back(*new_area);
          next_areas_.back().from_index = index;
          next_areas_.back().id = old_area->id;

          if (old_area->forward) {
            graph_->connect(old_area->from_index, index);
            graph_->connect(index, new_area->to_index);
          }
          else {
            graph_->connect(index, old_area->from_index);
            graph_->connect(new_area->to_index, index);
          }
        }

        VISAGE_ASSERT(graph_->checkValidPolygons());

        for (auto& next_area : next_areas_)
          addArea(next_area);
        if (next_areas_.empty())
          checkAddIntersection(safePrev(last_position1_));

        VISAGE_ASSERT(graph_->checkValidPolygons());
        VISAGE_ASSERT(areas_.size() % 2 == 0);

        old_areas_.clear();
        new_areas_.clear();
      }

      void update() {
        Event ev = nextEvent();

        if constexpr (!find_intersections) {
          updateNormalEvent(ev);
          return;
        }

        checkForBeginIntersections(ev);
        DPoint point = checkForIntersections(ev.point);
        if (!new_areas_.empty() || ev.degeneracy) {
          updateDegeneracy(ev, point);
          return;
        }

        updateNormalEvent(ev);
        if (ev.type == PointType::End)
          checkAddIntersection(safePrev(last_position1_));
        else {
          checkAddIntersection(safePrev(last_position1_));
          checkAddIntersection(last_position2_);
        }
      }

      auto lowerBound(const ScanLineArea& area) {
        return std::lower_bound(areas_.begin(), areas_.end(), area);
      }

      auto safePrev(const std::vector<ScanLineArea>::iterator it) {
        if (it != areas_.begin())
          return std::prev(it);
        return areas_.end();
      }

      auto safePrev(const std::vector<ScanLineArea>::const_iterator it) const {
        if (it != areas_.begin())
          return std::prev(it);
        return areas_.end();
      }

      auto end() const { return areas_.end(); }
      int lastId() const { return last_id_; }
      auto lastPosition1() const { return last_position1_; }
      auto lastPosition2() const { return last_position2_; }

    private:
      TriangulationGraph* graph_ = nullptr;
      std::vector<int> sorted_indices_;
      int current_index_ = 0;

      std::vector<ScanLineArea> areas_;
      std::vector<ScanLineArea>::iterator last_position1_ = areas_.end();
      std::vector<ScanLineArea>::iterator last_position2_ = areas_.end();
      std::map<int, int> aliases_;
      std::vector<ScanLineArea> new_areas_;
      std::vector<ScanLineArea> old_areas_;
      std::vector<ScanLineArea> next_areas_;
      std::vector<int> degeneracies_;
      int area_index_ = 0;

      std::set<IntersectionEvent> intersection_events_;
      int last_id_ = 0;
    };

    void breakIntersections() {
      VISAGE_ASSERT(checkValidPolygons());
      ScanLine<true> scan_line(this);

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

          auto area_iterator = scan_line.lowerBound(ScanLineArea(ev.index, ev.point, ev.next_index,
                                                                 ev.next, true, INT_MAX));

          int current_winding = 0;
          bool reverse = false;
          if (area_iterator != scan_line.end()) {
            const auto& area = *area_iterator;
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
            reverse = convex != fill;
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
        if (ev.type == PointType::Continue) {
          scan_line.update();

          int diagonal_target = scan_line.lastId();
          if (!scan_line.lastPosition1()->forward && merge_vertices.count(diagonal_target))
            addDiagonal(scan_line, ev.index, diagonal_target);

          scan_line.lastPosition1()->id = ev.index;
          if (!scan_line.lastPosition1()->forward) {
            auto after = std::next(scan_line.lastPosition1());
            if (after != scan_line.end())
              after->id = ev.index;
          }
        }
        else if (ev.type == PointType::Begin) {
          scan_line.update();

          auto after = scan_line.lastPosition2();
          after->id = ev.index;

          int diagonal_index = ev.index;
          if (!convex) {
            after = std::next(after);
            diagonal_index = addDiagonal(scan_line, ev.index, after->id);
            after->id = ev.index;

            auto before = scan_line.safePrev(scan_line.lastPosition1());
            before->id = diagonal_index;
          }

          scan_line.lastPosition1()->id = diagonal_index;
        }
        else {
          scan_line.update();
          ScanLineArea scan_area(ev.prev_index, ev.prev, ev.index, ev.point, true);
          auto area = scan_line.lowerBound(scan_area);

          if (convex && merge_vertices.count(scan_line.lastId()))
            addDiagonal(scan_line, ev.index, scan_line.lastId());

          if (area != scan_line.end()) {
            if (!convex) {
              area->id = ev.index;
              area = scan_line.safePrev(area);
              if (area != scan_line.end())
                area->id = ev.index;
              merge_vertices.insert(ev.index);
            }
          }
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

    template<Path::JointType joint_type>
    void offset(double amount) {
      if (amount == 0.0)
        return;

      double max_delta_radians = 2.0 * std::acos(1.0 - Path::kDefaultErrorTolerance / std::abs(amount));
      int start_points = points_.size();
      std::unique_ptr<bool[]> touched = std::make_unique<bool[]>(start_points);
      for (int i = 0; i < start_points; ++i) {
        if (i == next_edge_[i])
          continue;

        DPoint start = points_[i];
        int prev_index = prev_edge_[i];
        DPoint prev = points_[prev_index];
        DPoint point = start;
        DPoint prev_direction = (start - prev).normalized();
        DPoint prev_offset = DPoint(-prev_direction.y, prev_direction.x) * amount;
        int index = i;
        while (!touched[index]) {
          touched[index] = true;
          int next_index = next_edge_[index];

          DPoint next = next_index == i ? start : points_[next_index];
          DPoint direction = (next - point).normalized();
          auto offset = DPoint(-direction.y, direction.x) * amount;
          if constexpr (joint_type == Path::JointType::Bevel) {
            points_[index] += offset;
            insertPointBetween(index, next_index, next + offset);
          }
          else if constexpr (joint_type == Path::JointType::Miter) {
            auto intersection = Path::findIntersection(prev + prev_offset, point + prev_offset,
                                                       point + offset, next + offset);
            points_[index] = intersection.value();
          }
          else if constexpr (joint_type == Path::JointType::Square) {
            DPoint square_offset = (prev_direction - direction).normalized() * amount;
            DPoint square_center = point + square_offset;
            DPoint square_tangent = DPoint(-square_offset.y, square_offset.x);
            auto intersection_prev = Path::findIntersection(square_center, square_center + square_tangent,
                                                            prev + prev_offset, point + prev_offset);
            auto intersection = Path::findIntersection(square_center, square_center + square_tangent,
                                                       point + offset, next + offset);
            points_[index] = intersection_prev.value();
            insertPointBetween(index, next_index, intersection.value());
          }
          else if constexpr (joint_type == Path::JointType::Round) {
            bool convex = stableOrientation(prev, point, next) < 0.0;
            if (convex == (amount > 0.0)) {
              double arc_angle = std::acos(prev_offset.dot(offset) / (amount * amount));
              points_[index] += prev_offset;
              int num_points = std::ceil(arc_angle / max_delta_radians);
              std::complex<double> position(prev_offset.x, prev_offset.y);
              double angle_delta = arc_angle / num_points;
              std::complex<double> rotation = std::polar(1.0, (amount < 0.0) ? angle_delta : -angle_delta);
              int current_index = index;

              for (int i = 0; i < num_points; ++i) {
                position *= rotation;
                DPoint insert = point + DPoint(position.real(), position.imag());
                current_index = insertPointBetween(current_index, next_index, insert);
              }
            }
            else {
              if (amount < 0.0) {
                auto intersection = Path::findIntersection(prev + prev_offset, point + prev_offset,
                                                           point + offset, next + offset);
                points_[index] = intersection.value();
              }
              else {
                points_[index] += prev_offset;
                insertPointBetween(index, next_index, point + offset);
              }
            }
          }

          prev_index = index;
          index = next_index;
          prev = point;
          point = next;
          prev_direction = direction;
          prev_offset = offset;
        }
      }

      simplify();
      // TODO maybe use robust comparisons instead of this
      for (auto& point : points_)
        point = DPoint((float)point.x, (float)point.y);

      breakIntersections();
      fixWindings(Path::FillRule::Positive);
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
            if (stableOrientation(points_[i], points_[next_edge_[i]], points_[next_edge_[next_edge_[i]]]))
              break;

            removeFromCycle(next_edge_[i]);
          }
        }
      }
    }

    Path toPath() const {
      std::unique_ptr<bool[]> visited = std::make_unique<bool[]>(points_.size());
      Path path;
      for (int i = 0; i < points_.size(); ++i) {
        if (i == next_edge_[i] || visited[i])
          continue;

        int index = i;
        while (!visited[index]) {
          path.lineTo(Point(points_[index]));
          visited[index] = true;
          index = next_edge_[index];
        }

        path.close();
      }
      return path;
    }

  private:
    PointType pointType(int index) const {
      double compare_prev = points_[index].compare(points_[prev_edge_[index]]);
      double compare_next = points_[index].compare(points_[next_edge_[index]]);

      if (compare_prev < 0.0 && compare_next < 0.0)
        return PointType::Begin;
      if (compare_prev > 0.0 && compare_next > 0.0)
        return PointType::End;
      return PointType::Continue;
    }

    int addAdditionalPoint(const DPoint& point) {
      points_.push_back(point);
      int new_index = points_.size() - 1;
      prev_edge_.push_back(new_index);
      next_edge_.push_back(new_index);
      return new_index;
    }

    int insertPointBetween(int start_index, int end_index, const DPoint& point) {
      VISAGE_ASSERT(points_[start_index] != point);
      VISAGE_ASSERT(points_[end_index] != point);
      VISAGE_ASSERT(next_edge_[start_index] == end_index);
      VISAGE_ASSERT(prev_edge_[end_index] == start_index);

      int new_index = addAdditionalPoint(point);
      connect(start_index, new_index);
      connect(new_index, end_index);
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

    std::vector<int> sortedIndices() const {
      std::vector<int> sorted_indices;
      sorted_indices.resize(prev_edge_.size());
      std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

      std::sort(sorted_indices.begin(), sorted_indices.end(),
                [this](const int a, const int b) { return lessThan(a, b); });
      return sorted_indices;
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

    int addDiagonal(ScanLine<false>& scan_line, int index, int target) {
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

    bool tryCutEar(int index, bool forward, std::vector<int>& triangles,
                   const std::unique_ptr<bool[]>& touched) {
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

    void cutEars(int index, std::vector<int>& triangles, const std::unique_ptr<bool[]>& touched) {
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

  Path Path::computeOffset(float offset, JointType joint_type) const {
    TriangulationGraph graph(this);
    graph.simplify();
    graph.breakIntersections();
    graph.fixWindings(fillRule());
    switch (joint_type) {
    case JointType::Bevel: graph.offset<JointType::Bevel>(offset); break;
    case JointType::Miter: graph.offset<JointType::Miter>(offset); break;
    case JointType::Square: graph.offset<JointType::Square>(offset); break;
    case JointType::Round: graph.offset<JointType::Round>(offset); break;
    }
    return graph.toPath();
  }
}