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

    Point from = last_point_;
    if (relative)
      to += last_point_;

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

  static std::vector<float> parseNumbers(const std::string& str, size_t& i, int num, bool bit_flags = false) {
    std::vector<float> numbers;
    std::string number;
    while (i < str.size() && numbers.size() < num) {
      bool sign = str[i] == '-' || str[i] == '+';
      if (std::isdigit(str[i]) || (number.empty() && sign) || str[i] == '.' || str[i] == 'e' ||
          str[i] == 'E') {
        number += str[i++];
      }
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

      if (bit_flags && !number.empty()) {
        numbers.push_back(std::stof(number));
        number.clear();
      }
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

  void Path::parseSvgPath(const std::string& path, float scale) {
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
      std::vector<float> vals;
      if (std::toupper(command_char) == 'A') {
        vals = parseNumbers(path, i, 3);
        vals[0] *= scale;
        vals[1] *= scale;
        auto flags = parseNumbers(path, i, 2, true);
        auto end = parseNumbers(path, i, 2);
        end[0] *= scale;
        end[1] *= scale;
        vals.insert(vals.end(), flags.begin(), flags.end());
        vals.insert(vals.end(), end.begin(), end.end());
      }
      else {
        vals = parseNumbers(path, i, numbersForCommand(command_char));
        for (auto& val : vals)
          val *= scale;
      }

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
    static constexpr double kEpsilon = 1.0e-10;
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
    ScanLineArea(int from_index, const DPoint& from, int to_index, const DPoint& to, bool forward) :
        from_index(from_index), from(from), to_index(to_index), to(to), forward(forward) { }

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
        return stableOrientation(other.from, other.to, to) < 0.0;
      else if (to < other.to)
        return stableOrientation(from, to, other.to) > 0.0;

      return false;
    }

    int from_index = 0;
    DPoint from;
    int to_index = 0;
    DPoint to;
    bool forward = true;
    int data = 0;
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

      scan_line_ = std::make_unique<ScanLine>(this);
    }

    TriangulationGraph(const TriangulationGraph& other) {
      points_ = other.points_;
      prev_edge_ = other.prev_edge_;
      next_edge_ = other.next_edge_;
      scan_line_ = std::make_unique<ScanLine>(this);
    }

    Path::Triangulation triangulate(Path::FillRule fill_rule, int minimum_cycles = 1) {
      removeLinearPoints();
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

    class ScanLine {
    public:
      enum class IntersectionType {
        None,
        Cross,
        Colinear
      };

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
        edit_positions_.resize(sorted_indices_.size(), -1);
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
        degeneracy = degeneracy || (current_index_ - 1 >= 0 &&
                                    graph_->points_[sorted_indices_[current_index_ - 1]] == point);
        return { type, index, point, prev_index, prev, next_index, next, degeneracy };
      }

      struct IntersectionEvent {
        DPoint point;
        int area1_from_index;
        int area1_to_index;
        int area2_from_index;
        int area2_to_index;
      };

      void progressToNextIntersection() {
        next_intersection_ = -1;
        DPoint point;
        for (int i = 0; i < intersection_events_.size(); ++i) {
          if (next_intersection_ == -1 || intersection_events_[i].point < point) {
            next_intersection_ = i;
            point = intersection_events_[i].point;
          }
        }
      }

      auto findAreaByToIndex(int index, std::vector<ScanLineArea>::iterator it) {
        for (; it != areas_.end(); ++it) {
          if (it->to_index == index)
            return it;
        }

        VISAGE_ASSERT(false);
        return areas_.end();
      }

      auto findAreaByFromTo(int from, int to) {
        for (auto it = areas_.begin(); it != areas_.end(); ++it) {
          if (it->from_index == from && it->to_index == to)
            return it;
        }

        VISAGE_ASSERT(false);
        return areas_.end();
      }

      auto findAreaByToIndex(int index) { return findAreaByToIndex(index, areas_.begin()); }

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

      bool splitIntersection() {
        auto it = intersection_events_.begin() + next_intersection_;
        IntersectionEvent ev = *it;
        intersection_events_.erase(it);
        last_position1_ = findAreaByFromTo(ev.area1_from_index, ev.area1_to_index);
        last_position2_ = findAreaByFromTo(ev.area2_from_index, ev.area2_to_index);
        checkRemoveIntersection(safePrev(last_position1_));
        checkRemoveIntersection(last_position2_);

        bool added = false;

        if (ev.point != last_position1_->from && ev.point != last_position1_->to) {
          last_position1_->from_index = graph_->insertPointBetween(last_position1_->from_index,
                                                                   last_position1_->to_index, ev.point);
          last_position1_->from = ev.point;
          added = true;
        }
        if (ev.point != last_position2_->from && ev.point != last_position2_->to) {
          last_position2_->from_index = graph_->insertPointBetween(last_position2_->from_index,
                                                                   last_position2_->to_index, ev.point);
          last_position2_->from = ev.point;
          added = true;
        }

        progressToNextIntersection();
        if (*last_position2_ < *last_position1_)
          std::swap(*last_position1_, *last_position2_);

        checkAddIntersection(safePrev(last_position1_));
        checkAddIntersection(last_position1_);
        checkAddIntersection(last_position2_);

        return added;
      }

      IntersectionType intersectionType(std::vector<ScanLineArea>::iterator it) {
        if (it == areas_.end())
          return IntersectionType::None;
        auto next = std::next(it);
        if (next == areas_.end())
          return IntersectionType::None;

        const auto& area1 = *it;
        const auto& area2 = *next;

        if (it->from == next->to || next->from == it->to || it->to_index == next->to_index)
          return IntersectionType::None;

        double cross = (area1.to - area1.from).cross(area2.to - area2.from);
        double compare1 = stableOrientation(area1.from, area1.to, area2.to);
        double compare2 = stableOrientation(area2.from, area2.to, area1.to);
        if (compare1 == 0.0 && compare2 == 0.0) {
          if (it->from == next->from && it->to == next->to)
            return IntersectionType::None;

          if (cross == 0.0 || stableOrientation(area1.from, area1.to, area2.from) == 0.0)
            return IntersectionType::Colinear;

          if (it->to == next->to)
            return IntersectionType::None;

          return IntersectionType::Cross;
        }

        if (compare1 <= 0.0 && compare2 >= 0.0)
          return cross == 0.0 ? IntersectionType::Colinear : IntersectionType::Cross;

        if (area1.from == area2.from)
          return IntersectionType::None;

        compare1 = stableOrientation(area1.from, area1.to, area2.from);
        double min_y = std::min(area1.from.y, area1.to.y);
        double max_y = std::max(area1.from.y, area1.to.y);
        if (compare1 == 0.0 && area2.from.x >= area1.from.x && area2.from.x <= area1.to.x &&
            area2.from.y >= min_y && area2.from.y <= max_y)
          return IntersectionType::Cross;

        compare2 = stableOrientation(area2.from, area2.to, area1.from);
        min_y = std::min(area2.from.y, area2.to.y);
        max_y = std::max(area2.from.y, area2.to.y);
        if (compare2 == 0.0 && area1.from.x >= area2.from.x && area1.from.x <= area2.to.x &&
            area1.from.y >= min_y && area1.from.y <= max_y)
          return IntersectionType::Cross;

        return IntersectionType::None;
      }

      bool hasIntersection(int from1, int to1, int from2, int to2) const {
        return std::any_of(intersection_events_.begin(), intersection_events_.end(), [&](const auto& intersection) {
          return (intersection.area1_from_index == from1 && intersection.area1_to_index == to1 &&
                  intersection.area2_from_index == from2 && intersection.area2_to_index == to2) ||
                 (intersection.area1_from_index == from2 && intersection.area1_to_index == to2 &&
                  intersection.area2_from_index == from1 && intersection.area2_to_index == to1);
        });
      }

      void checkAddIntersection(std::vector<ScanLineArea>::iterator it) {
        double kEpsilon = 1.0e-8;

        auto intersection_type = intersectionType(it);
        if (intersection_type == IntersectionType::None)
          return;

        DPoint intersection;
        auto next = std::next(it);

        if (intersection_type == IntersectionType::Colinear) {
          if (it->from != next->from)
            intersection = it->from < next->from ? next->from : it->from;
          else
            intersection = it->to < next->to ? it->to : next->to;
        }
        else {
          auto intersection_option = Path::findIntersection(it->from, it->to, next->from, next->to);
          VISAGE_ASSERT(intersection_option.has_value());
          intersection = intersection_option.value();

          double min_x = std::max(it->from.x, next->from.x);
          double max_x = std::min(it->to.x, next->to.x);
          intersection.x = std::clamp(intersection.x, min_x, max_x);
          double min_y = std::max(std::min(it->from.y, it->to.y), std::min(next->from.y, next->to.y));
          double max_y = std::min(std::max(it->from.y, it->to.y), std::max(next->from.y, next->to.y));
          max_y = std::max(min_y, max_y);
          intersection.y = std::clamp(intersection.y, min_y, max_y);
          if ((intersection - it->from).squareMagnitude() < kEpsilon)
            intersection = it->from;
          else if ((intersection - it->to).squareMagnitude() < kEpsilon)
            intersection = it->to;
          else if ((intersection - next->from).squareMagnitude() < kEpsilon)
            intersection = next->from;
          else if ((intersection - next->to).squareMagnitude() < kEpsilon)
            intersection = next->to;
        }

        // TODO: this can possibly move the intersection point a lot if points are set with double precision
        // e.g. intersecting (0.0, 0.0) -> (std::nextafter(0.0), 2.0) and (-1.0, 0.0) -> (1.0, 0.0)
        // This doesn't happen when defining the path with floats since you get more space
        // between x positions when converting to double.
        intersection = std::clamp(intersection, it->from, it->to);
        intersection = std::clamp(intersection, next->from, next->to);

        if (next_intersection_ >= 0 && intersection < intersection_events_[next_intersection_].point)
          next_intersection_ = intersection_events_.size();
        else if (next_intersection_ < 0)
          next_intersection_ = 0;

        VISAGE_ASSERT(!hasIntersection(it->from_index, it->to_index, next->from_index, next->to_index));
        intersection_events_.push_back(IntersectionEvent { intersection, it->from_index, it->to_index,
                                                           next->from_index, next->to_index });
      }

      void checkRemoveIntersection(const std::vector<ScanLineArea>::iterator it) {
        if (intersectionType(it) == IntersectionType::None)
          return;

        for (int i = 0; i < intersection_events_.size(); ++i) {
          const auto& ev = intersection_events_[i];
          if (ev.area1_to_index == it->to_index && ev.area1_from_index == it->from_index &&
              ev.area2_to_index == std::next(it)->to_index) {
            intersection_events_.erase(intersection_events_.begin() + i);
            if (next_intersection_ == i)
              progressToNextIntersection();
            else if (next_intersection_ > i)
              next_intersection_--;
            return;
          }
        }
        VISAGE_ASSERT(false);
      }

      void processPointEvents(Event ev) {
        bool found_position = false;
        for (auto it = areas_.begin(); it != areas_.end();) {
          if (it->to == ev.point) {
            old_areas_.push_back(*it);
            found_position = true;
            it = areas_.erase(it);
            last_position1_ = it;
          }
          else
            ++it;
        }

        DPoint point = ev.point;
        while (ev.degeneracy && ev.point == point) {
          if (ev.type == PointType::Continue) {
            int from_index = ev.prev_index;
            int to_index = ev.next_index;
            bool forward = ev.prev < ev.next;
            if (!forward)
              std::swap(from_index, to_index);

            new_areas_.emplace_back(ev.index, point, to_index, graph_->points_[to_index], forward);
          }
          else if (ev.type == PointType::Begin) {
            new_areas_.emplace_back(ev.index, point, ev.prev_index, ev.prev, false);
            new_areas_.emplace_back(ev.index, point, ev.next_index, ev.next, true);
            if (!found_position) {
              found_position = true;
              last_position1_ = areas_.begin() + editPosition(ev.index);
            }
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

        auto pair = [&](std::vector<ScanLineArea>::iterator& it) {
          auto next = std::next(it);
          int index = it - areas.begin();
          handle_pair(*it, *next, index);

          it = areas.erase(it);
          it = areas.erase(it);
          if (it != areas.begin() && it != areas.end())
            it = std::prev(it);
        };

        for (auto it = areas.begin(); it != areas.end();) {
          auto next = std::next(it);
          if (next != areas.end() && it->forward != next->forward && it->from == next->from &&
              it->to == next->to) {
            pair(it);
          }
          else
            ++it;
        }

        for (auto it = areas.begin(); it != areas.end();) {
          auto next = std::next(it);
          if (next != areas.end() && it->forward != next->forward)
            pair(it);
          else
            ++it;
        }
      }

      void updateNormalEvent(const Event& ev) {
        if (ev.type == PointType::Continue) {
          last_position1_ = findAreaByToIndex(ev.index);
          last_position2_ = last_position1_;
          last_data_ = last_position1_->data;

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
          ScanLineArea to_insert1(ev.index, ev.point, ev.prev_index, ev.prev, false);
          ScanLineArea to_insert2(ev.index, ev.point, ev.next_index, ev.next, true);
          if (to_insert2 < to_insert1)
            std::swap(to_insert1, to_insert2);

          VISAGE_ASSERT(to_insert1.from != to_insert1.to);
          VISAGE_ASSERT(to_insert2.from != to_insert2.to);
          auto location = areas_.begin() + editPosition(ev.index);
          last_position1_ = areas_.insert(location, to_insert2);
          last_position1_ = areas_.insert(last_position1_, to_insert1);
          last_position2_ = std::next(last_position1_);
        }
        else {
          areas_.erase(findAreaByToIndex(ev.index));
          auto to_erase = findAreaByToIndex(ev.index);
          last_data_ = to_erase->data;
          last_position1_ = areas_.erase(to_erase);
          last_position2_ = last_position1_;
        }

        current_index_++;
        progressToNextEvent();

        VISAGE_ASSERT(areas_.size() % 2 == 0);
      }

      void updateDegeneracy(const Event& ev) {
        last_position1_ = areas_.end();
        processPointEvents(ev);

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
        pairInsOuts(old_areas_, [&, this](const ScanLineArea& area1, const ScanLineArea& area2, int) {
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

        next_areas_ = new_areas_;
        int new_index = degeneracies_.size() - 1;
        int edit_position = last_position1_ - areas_.begin();
        pairInsOuts(new_areas_, [&, this](const ScanLineArea& area1, const ScanLineArea& area2, int i) {
          int index = degeneracies_[new_index--];
          edit_positions_[index] = edit_position + i;

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
        int new_i = 0;
        for (auto old_area = old_areas_.begin(); old_area != old_areas_.end(); ++old_area, ++new_i) {
          int index = degeneracies_[old_index++];
          edit_positions_[index] = edit_position + new_i;

          if (old_area->forward) {
            graph_->connect(old_area->from_index, index);
            graph_->connect(index, new_areas_[new_i].to_index);
          }
          else {
            graph_->connect(index, old_area->from_index);
            graph_->connect(new_areas_[new_i].to_index, index);
          }
        }

        for (auto& area : next_areas_) {
          if (area.forward)
            area.from_index = graph_->prev_edge_[area.to_index];
          else
            area.from_index = graph_->next_edge_[area.to_index];
        }

        VISAGE_ASSERT(graph_->checkValidPolygons());

        areas_.insert(last_position1_, next_areas_.begin(), next_areas_.end());

        VISAGE_ASSERT(graph_->checkValidPolygons());
        VISAGE_ASSERT(areas_.size() % 2 == 0);

        old_areas_.clear();
        new_areas_.clear();
      }

      bool updateSplitIntersections() {
        Event ev = nextEvent();

        if (next_intersection_ >= 0 && intersection_events_[next_intersection_].point <= ev.point)
          return splitIntersection();

        if (ev.type == PointType::Continue) {
          last_position1_ = findAreaByToIndex(ev.index);
          edit_positions_[ev.index] = last_position1_ - areas_.begin();
          last_position2_ = last_position1_;

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

          checkAddIntersection(safePrev(last_position1_));
          checkAddIntersection(last_position1_);
        }
        else if (ev.type == PointType::Begin) {
          ScanLineArea to_insert1(ev.index, ev.point, ev.prev_index, ev.prev, false);
          ScanLineArea to_insert2(ev.index, ev.point, ev.next_index, ev.next, true);
          if (to_insert2 < to_insert1)
            std::swap(to_insert1, to_insert2);

          VISAGE_ASSERT(to_insert1.from != to_insert1.to);
          VISAGE_ASSERT(to_insert2.from != to_insert2.to);
          auto lower_bound = std::lower_bound(areas_.begin(), areas_.end(), to_insert1);
          edit_positions_[ev.index] = lower_bound - areas_.begin();
          checkRemoveIntersection(safePrev(lower_bound));

          last_position1_ = areas_.insert(lower_bound, to_insert2);
          last_position1_ = areas_.insert(last_position1_, to_insert1);
          last_position2_ = std::next(last_position1_);

          checkAddIntersection(safePrev(last_position1_));
          checkAddIntersection(last_position1_);
          checkAddIntersection(last_position2_);
        }
        else {
          auto end1 = findAreaByToIndex(ev.index);
          for (auto it = std::next(end1); it->to_index != ev.index; ++it) {
            if (it->from != ev.point && it->to != ev.point) {
              checkRemoveIntersection(it);
              checkRemoveIntersection(safePrev(it));
              it->from = ev.point;
              it->from_index = graph_->insertPointBetween(it->from_index, it->to_index, ev.point);
              checkAddIntersection(it);
              checkAddIntersection(safePrev(it));
            }
          }

          last_position1_ = areas_.erase(end1);
          if (last_position1_->to_index == ev.index) {
            last_data_ = last_position1_->data;
            last_position1_ = areas_.erase(last_position1_);
            last_position2_ = last_position1_;
          }
          else {
            auto end2 = findAreaByToIndex(ev.index, last_position1_);
            last_data_ = end2->data;
            last_position2_ = areas_.erase(end2);
            checkAddIntersection(safePrev(last_position2_));
          }
          checkAddIntersection(safePrev(last_position1_));
        }

        current_index_++;
        progressToNextEvent();
        return false;
      }

      void updateBreakIntersections() {
        Event ev = nextEvent();
        if (ev.degeneracy)
          updateDegeneracy(ev);
        else
          updateNormalEvent(ev);
      }

      void update() { updateNormalEvent(nextEvent()); }
      auto begin() { return areas_.begin(); }

      auto lowerBound(const ScanLineArea& area) {
        return std::lower_bound(areas_.begin(), areas_.end(), area);
      }

      int editPosition(int index) const {
        VISAGE_ASSERT(index < edit_positions_.size() && edit_positions_[index] >= 0);
        return edit_positions_[index];
      }

      auto end() const { return areas_.end(); }
      int lastData() const { return last_data_; }
      auto lastPosition1() const { return last_position1_; }
      auto lastPosition2() const { return last_position2_; }

      void reset() {
        sorted_indices_ = graph_->sortedIndices();
        edit_positions_.resize(sorted_indices_.size(), -1);
        areas_.clear();
        last_position1_ = areas_.end();
        last_position2_ = areas_.end();
        aliases_.clear();
        intersection_events_.clear();
        current_index_ = 0;
        progressToNextEvent();
      }

    private:
      TriangulationGraph* graph_ = nullptr;
      std::vector<int> sorted_indices_;
      int current_index_ = 0;
      int next_intersection_ = -1;

      std::vector<ScanLineArea> areas_;
      std::vector<int> edit_positions_;
      std::vector<ScanLineArea>::iterator last_position1_ = areas_.end();
      std::vector<ScanLineArea>::iterator last_position2_ = areas_.end();
      std::map<int, int> aliases_;
      std::vector<ScanLineArea> new_areas_;
      std::vector<ScanLineArea> old_areas_;
      std::vector<ScanLineArea> next_areas_;
      std::vector<int> degeneracies_;

      std::vector<IntersectionEvent> intersection_events_;
      int last_data_ = 0;
    };

    void breakIntersections() {
      VISAGE_ASSERT(checkValidPolygons());
      removeLinearPoints();

      bool intersection_added = true;
      while (intersection_added) {
        intersection_added = false;
        scan_line_->reset();
        while (scan_line_->hasNext()) {
          if (scan_line_->updateSplitIntersections())
            intersection_added = true;
        }
      }

      simplify();
      scan_line_->reset();
      while (scan_line_->hasNext())
        scan_line_->updateBreakIntersections();

      simplify();
      scan_line_->reset();
    }

    void fixWindings(Path::FillRule fill_rule, int minimum_cycles = 1) {
      scan_line_->reset();

      auto windings = std::make_unique<int[]>(points_.size());
      auto reversed = std::make_unique<bool[]>(points_.size());
      auto winding_directions = std::make_unique<int[]>(points_.size());

      while (scan_line_->hasNext()) {
        auto ev = scan_line_->nextEvent();
        if (winding_directions[ev.index] == 0) {
          bool convex = stableOrientation(ev.point, ev.prev, ev.next) >= 0.0;
          auto area_iterator = scan_line_->begin() + scan_line_->editPosition(ev.index);

          int current_winding = 0;
          bool reverse = false;
          if (area_iterator != scan_line_->end()) {
            const auto& area = *area_iterator;
            bool fill = winding_directions[area.from_index] == 1;
            current_winding += windings[area.from_index];

            bool inside_polygon = area.forward == fill;
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

        scan_line_->update();
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
      scan_line_->reset();
      std::set<int> merge_vertices;

      while (scan_line_->hasNext()) {
        auto ev = scan_line_->nextEvent();

        bool convex = stableOrientation(ev.point, ev.prev, ev.next) >= 0.0;
        if (ev.type == PointType::Continue) {
          scan_line_->update();

          int diagonal_target = scan_line_->lastData();
          if (!scan_line_->lastPosition1()->forward && merge_vertices.count(diagonal_target))
            addDiagonal(ev.index, diagonal_target);

          scan_line_->lastPosition1()->data = ev.index;
          if (!scan_line_->lastPosition1()->forward) {
            auto after = std::next(scan_line_->lastPosition1());
            if (after != scan_line_->end())
              after->data = ev.index;
          }
        }
        else if (ev.type == PointType::Begin) {
          scan_line_->update();

          auto after = scan_line_->lastPosition2();
          after->data = ev.index;

          int diagonal_index = ev.index;
          if (!convex) {
            after = std::next(after);
            diagonal_index = addDiagonal(ev.index, after->data);
            after->data = ev.index;

            auto before = scan_line_->safePrev(scan_line_->lastPosition1());
            before->data = diagonal_index;
          }

          scan_line_->lastPosition1()->data = diagonal_index;
        }
        else {
          scan_line_->update();
          ScanLineArea scan_area(ev.prev_index, ev.prev, ev.index, ev.point, true);
          auto area = scan_line_->lowerBound(scan_area);

          if (convex && merge_vertices.count(scan_line_->lastData()))
            addDiagonal(ev.index, scan_line_->lastData());

          if (area != scan_line_->end()) {
            if (!convex) {
              area->data = ev.index;
              area = scan_line_->safePrev(area);
              if (area != scan_line_->end())
                area->data = ev.index;
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

    template<Path::JoinType joint_type>
    void offset(double amount, bool post_simplify, std::vector<int>& points_created) {
      if (amount == 0.0)
        return;

      double max_delta_radians = 2.0 * std::acos(1.0 - Path::kDefaultErrorTolerance / std::abs(amount));
      int start_points = points_.size();
      std::unique_ptr<bool[]> touched = std::make_unique<bool[]>(start_points);
      for (int i = 0; i < start_points; ++i) {
        if (i == next_edge_[i])
          continue;

        DPoint start = points_[i];
        DPoint prev = points_[prev_edge_[i]];
        if (prev == start)
          prev = points_[prev_edge_[prev_edge_[i]]];
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
          if constexpr (joint_type == Path::JoinType::Bevel) {
            points_[index] += offset;
            insertPointBetween(index, next_index, next + offset);
            points_created.push_back(2);
          }
          else if constexpr (joint_type == Path::JoinType::Miter) {
            auto intersection = Path::findIntersection(prev + prev_offset, point + prev_offset,
                                                       point + offset, next + offset);
            if (intersection.has_value())
              points_[index] = intersection.value();
            else
              points_[index] = point;
            points_created.push_back(1);
          }
          else if constexpr (joint_type == Path::JoinType::Square) {
            DPoint square_offset = (prev_direction - direction).normalized() * amount;
            DPoint square_center = point + square_offset;
            DPoint square_tangent = DPoint(-square_offset.y, square_offset.x);
            auto intersection_prev = Path::findIntersection(square_center, square_center + square_tangent,
                                                            prev + prev_offset, point + prev_offset);
            auto intersection = Path::findIntersection(square_center, square_center + square_tangent,
                                                       point + offset, next + offset);
            VISAGE_ASSERT(intersection_prev.has_value());
            VISAGE_ASSERT(intersection.has_value());
            points_[index] = intersection_prev.value();
            insertPointBetween(index, next_index, intersection.value());
            points_created.push_back(2);
          }
          else if constexpr (joint_type == Path::JoinType::Round) {
            bool convex = stableOrientation(prev, point, next) < 0.0;
            if (convex == (amount > 0.0)) {
              double arc_angle = std::acos(prev_offset.dot(offset) / (amount * amount));
              points_[index] += prev_offset;
              int num_points = std::ceil(arc_angle / max_delta_radians);
              std::complex<double> position(prev_offset.x, prev_offset.y);
              double angle_delta = arc_angle / num_points;
              std::complex<double> rotation = std::polar(1.0, (amount < 0.0) ? angle_delta : -angle_delta);
              int current_index = index;

              for (int p = 0; p < num_points; ++p) {
                position *= rotation;
                DPoint insert = point + DPoint(position.real(), position.imag());
                current_index = insertPointBetween(current_index, next_index, insert);
              }
              points_created.push_back(num_points + 1);
            }
            else {
              if (amount < 0.0) {
                auto intersection = Path::findIntersection(prev + prev_offset, point + prev_offset,
                                                           point + offset, next + offset);
                VISAGE_ASSERT(intersection.has_value());
                points_[index] = intersection.value();
                points_created.push_back(1);
              }
              else {
                points_[index] += prev_offset;
                DPoint insert = point + offset;
                if (insert != points_[index] && insert != next) {
                  insertPointBetween(index, next_index, insert);
                  points_created.push_back(2);
                }
                else
                  points_created.push_back(1);
              }
            }
          }

          index = next_index;
          prev = point;
          point = next;
          prev_direction = direction;
          prev_offset = offset;
        }
      }

      if (post_simplify) {
        simplify();
        breakIntersections();
        fixWindings(Path::FillRule::Positive);
      }
    }

    template<Path::JoinType joint_type>
    void offset(double amount, bool post_simplify = true) {
      std::vector<int> points_created;
      offset<joint_type>(amount, post_simplify, points_created);
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

    void removeLinearPoints() {
      bool removed = true;
      while (removed) {
        removed = false;
        for (int i = 0; i < points_.size(); ++i) {
          if (i == next_edge_[i])
            continue;

          if (points_[i] == points_[next_edge_[i]]) {
            removed = true;
            removeFromCycle(i);
          }
          else {
            while (i != next_edge_[i]) {
              if (stableOrientation(points_[i], points_[next_edge_[i]], points_[next_edge_[next_edge_[i]]]))
                break;

              removed = true;
              removeFromCycle(next_edge_[i]);
            }
          }
        }
      }
    }

    void simplify() {
      for (int i = 0; i < points_.size(); ++i) {
        if (i == next_edge_[i])
          continue;

        if (points_[i] == points_[next_edge_[i]])
          removeFromCycle(i);
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
      if (prev_edge_[start_index] == end_index)
        std::swap(start_index, end_index);

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

    int addDiagonal(int index, int target) {
      int new_index = prev_edge_.size();
      int new_diagonal_index = new_index + 1;
      scan_line_->addAlias(new_index, index);
      scan_line_->addAlias(new_diagonal_index, target);
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

    std::unique_ptr<ScanLine> scan_line_;
    std::vector<DPoint> points_;
    std::vector<int> prev_edge_;
    std::vector<int> next_edge_;
  };

  Path::Triangulation Path::triangulate() const {
    TriangulationGraph graph(this);
    return graph.triangulate(fillRule());
  }

  Path Path::combine(const Path& other, Operation operation) const {
    switch (operation) {
    case Operation::Union: return combine(other, Path::FillRule::NonZero, 1, false);
    case Operation::Intersection: return combine(other, Path::FillRule::NonZero, 2, false);
    case Operation::Difference: return combine(other, Path::FillRule::Positive, 1, true);
    case Operation::Xor: return combine(other, Path::FillRule::EvenOdd, 1, false);
    default: return *this;
    }
  }

  Path Path::combine(const Path& other, Path::FillRule fill_rule, int num_cycles_needed,
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

  std::pair<Path, Path> Path::offsetAntiAlias(float scale, std::vector<int>& inner_added_points,
                                              std::vector<int>& outer_added_points) const {
    TriangulationGraph outer(this);
    outer.simplify();
    TriangulationGraph inner = outer;
    outer.offset<JoinType::Round>(0.5f / scale, false, outer_added_points);
    inner.offset<JoinType::Round>(-0.5f / scale, false, inner_added_points);
    return { inner.toPath(), outer.toPath() };
  }

  Path Path::offset(float offset, JoinType joint_type) const {
    TriangulationGraph graph(this);
    graph.simplify();
    graph.breakIntersections();
    graph.fixWindings(fillRule());
    switch (joint_type) {
    case JoinType::Bevel: graph.offset<JoinType::Bevel>(offset); break;
    case JoinType::Miter: graph.offset<JoinType::Miter>(offset); break;
    case JoinType::Square: graph.offset<JoinType::Square>(offset); break;
    case JoinType::Round: graph.offset<JoinType::Round>(offset); break;
    }
    return graph.toPath();
  }

  Path Path::breakIntoSimplePolygons() const {
    TriangulationGraph graph(this);
    graph.removeLinearPoints();
    graph.simplify();
    graph.breakIntersections();
    graph.fixWindings(fillRule());
    return graph.toPath();
  }
}