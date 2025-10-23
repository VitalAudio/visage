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
  template<typename T>
  static void roundedRectangle(T& t, float x, float y, float width, float height, float rx_top_left,
                               float ry_top_left, float rx_top_right, float ry_top_right,
                               float rx_bottom_right, float ry_bottom_right, float rx_bottom_left,
                               float ry_bottom_left) {
    float scale = 1.0f;
    scale = std::min(scale, width / (rx_top_left + rx_top_right));
    scale = std::min(scale, width / (rx_bottom_left + rx_bottom_right));
    scale = std::min(scale, height / (ry_top_left + ry_bottom_left));
    scale = std::min(scale, height / (ry_top_right + ry_bottom_right));
    rx_top_left *= scale;
    ry_top_left *= scale;
    rx_top_right *= scale;
    ry_top_right *= scale;
    rx_bottom_right *= scale;
    ry_bottom_right *= scale;
    rx_bottom_left *= scale;
    ry_bottom_left *= scale;

    t.moveTo(x + rx_top_left, y);
    t.lineTo(x + width - rx_top_right, y);
    t.arcTo(rx_top_right, ry_top_right, 0.0f, false, true, Point(x + width, y + ry_top_right), false);
    t.lineTo(x + width, y + height - ry_bottom_right);
    t.arcTo(rx_bottom_right, ry_bottom_right, 0.0f, false, true,
            Point(x + width - rx_bottom_right, y + height), false);
    t.lineTo(x + rx_bottom_left, y + height);
    t.arcTo(rx_bottom_left, ry_bottom_left, 0.0f, false, true, Point(x, y + height - ry_bottom_left), false);
    t.lineTo(x, y + ry_top_left);
    t.arcTo(rx_top_left, ry_top_left, 0.0f, false, true, Point(x + rx_top_left, y), false);
    t.close();
  }

  template<typename T>
  static void roundedRectangle(T& t, float x, float y, float width, float height, float rx, float ry) {
    rx = std::min(rx, width * 0.5f);
    ry = std::min(ry, height * 0.5f);
    t.moveTo(x + rx, y);
    t.lineTo(x + width - rx, y);
    t.arcTo(rx, ry, 0.0f, false, true, Point(x + width, y + ry), false);
    t.lineTo(x + width, y + height - ry);
    t.arcTo(rx, ry, 0.0f, false, true, Point(x + width - rx, y + height), false);
    t.lineTo(x + rx, y + height);
    t.arcTo(rx, ry, 0.0f, false, true, Point(x, y + height - ry), false);
    t.lineTo(x, y + ry);
    t.arcTo(rx, ry, 0.0f, false, true, Point(x + rx, y), false);
    t.close();
  }

  void Path::arcTo(float x_radius, float y_radius, float x_axis_rotation, bool large_arc,
                   bool sweep_flag, Point to, bool relative) {
    if (currentPath().points.empty())
      addPoint(last_point_);

    smooth_control_point_ = {};

    Point from = last_point_;
    if (relative)
      to += last_point_;

    auto ellipse_rotation = Matrix::rotation(x_axis_rotation);
    Point delta = ellipse_rotation.transposed() * (to - from);
    float radius_ratio = x_radius / y_radius;
    delta.y *= radius_ratio;

    float length = delta.length();
    if (length == 0.0f)
      return;

    float radius = std::max(length * 0.5f, x_radius);
    float center_offset = std::sqrt(radius * radius - length * length * 0.25f);
    Point normal = Point(delta.y, -delta.x) / length;
    if (large_arc != sweep_flag)
      normal = -normal;

    Point center = delta * 0.5f + normal * center_offset;
    float arc_angle = 2.0f * std::asin(length * 0.5f / radius);

    if (large_arc)
      arc_angle = 2.0f * TriangulationGraph::kPi - arc_angle;
    if (!sweep_flag)
      arc_angle = -arc_angle;

    Point adjusted_radius = resolution_matrix_ * Point(x_radius, y_radius);
    float max_radius = std::max(std::abs(adjusted_radius.x), std::abs(adjusted_radius.y));
    float max_delta_radians = 2.0f * std::acos(1.0f - error_tolerance_ / max_radius);
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

  static float parseNumber(const std::string& str, size_t& i, bool bit_flags = false) {
    std::string number;
    while (i < str.size()) {
      bool sign = str[i] == '-' || str[i] == '+';
      if (std::isdigit(str[i]) || (number.empty() && sign) || str[i] == '.' || str[i] == 'e' ||
          str[i] == 'E') {
        if (str[i] == '.') {
          if (number.find('.') != std::string::npos)
            return std::stof(number);
        }
        number += str[i++];
      }
      else if (str[i] == ',' || std::isspace(str[i]) || sign) {
        if (!number.empty())
          return std::stof(number);
        if (!sign)
          ++i;
      }
      else if (std::isalpha(str[i]))
        break;
      else
        ++i;

      if (bit_flags && !number.empty())
        return std::stof(number);
    }
    if (number.empty()) {
      VISAGE_ASSERT(false);
      return 0.0f;
    }
    return std::stof(number);
  }

  Path::CommandList Path::parseSvgPath(const std::string& path) {
    CommandList commands;
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

      char type = std::toupper(command_char);
      bool relative = std::islower(command_char);

      if (type == 'M') {
        float x = parseNumber(path, i);
        float y = parseNumber(path, i);
        commands.moveTo(x, y, relative);
      }
      else if (type == 'L') {
        float x = parseNumber(path, i);
        float y = parseNumber(path, i);
        commands.lineTo(x, y, relative);
      }
      else if (type == 'H')
        commands.horizontalTo(parseNumber(path, i), relative);
      else if (type == 'V')
        commands.verticalTo(parseNumber(path, i), relative);
      else if (type == 'Z')
        commands.close();
      else if (type == 'C') {
        float cx1 = parseNumber(path, i);
        float cy1 = parseNumber(path, i);
        float cx2 = parseNumber(path, i);
        float cy2 = parseNumber(path, i);
        float x = parseNumber(path, i);
        float y = parseNumber(path, i);
        commands.bezierTo(cx1, cy1, cx2, cy2, x, y, relative);
      }
      else if (type == 'S') {
        float cx = parseNumber(path, i);
        float cy = parseNumber(path, i);
        float x = parseNumber(path, i);
        float y = parseNumber(path, i);
        commands.smoothBezierTo(cx, cy, x, y, relative);
      }
      else if (type == 'Q') {
        float cx = parseNumber(path, i);
        float cy = parseNumber(path, i);
        float x = parseNumber(path, i);
        float y = parseNumber(path, i);
        commands.quadraticTo(cx, cy, x, y, relative);
      }
      else if (type == 'T') {
        float x = parseNumber(path, i);
        float y = parseNumber(path, i);
        commands.smoothQuadraticTo(x, y, relative);
      }
      else if (type == 'A') {
        float rx = parseNumber(path, i);
        float ry = parseNumber(path, i);
        float rotation = parseNumber(path, i);
        bool large_arc = parseNumber(path, i, true);
        bool sweep = parseNumber(path, i, true);
        float x = parseNumber(path, i);
        float y = parseNumber(path, i);
        commands.arcTo(rx, ry, rotation, large_arc, sweep, x, y, relative);
      }
    }
    return commands;
  }

  void Path::CommandList::addRectangle(float x, float y, float width, float height) {
    moveTo(x, y);
    lineTo(x + width, y);
    lineTo(x + width, y + height);
    lineTo(x, y + height);
    close();
  }

  void Path::CommandList::addRoundedRectangle(float x, float y, float width, float height,
                                              float rx_top_left, float ry_top_left,
                                              float rx_top_right, float ry_top_right,
                                              float rx_bottom_left, float ry_bottom_left,
                                              float rx_bottom_right, float ry_bottom_right) {
    roundedRectangle(*this, x, y, width, height, rx_top_left, ry_top_left, rx_top_right,
                     ry_top_right, rx_bottom_right, ry_bottom_right, rx_bottom_left, ry_bottom_left);
  }

  void Path::CommandList::addRoundedRectangle(float x, float y, float width, float height, float rx,
                                              float ry) {
    roundedRectangle(*this, x, y, width, height, rx, ry);
  }

  void Path::CommandList::addEllipse(float cx, float cy, float rx, float ry) {
    moveTo(cx + rx, cy);
    arcTo(rx, ry, 180.0f, false, true, Point(cx - rx, cy), false);
    arcTo(rx, ry, 180.0f, false, true, Point(cx + rx, cy), false);

    close();
  }

  void Path::CommandList::addCircle(float cx, float cy, float r) {
    addEllipse(cx, cy, r, r);
  }

  void Path::addRectangle(float x, float y, float width, float height) {
    moveTo(x, y);
    lineTo(x + width, y);
    lineTo(x + width, y + height);
    lineTo(x, y + height);
    close();
  }

  void Path::addRoundedRectangle(float x, float y, float width, float height, float rx_top_left,
                                 float ry_top_left, float rx_top_right, float ry_top_right,
                                 float rx_bottom_right, float ry_bottom_right, float rx_bottom_left,
                                 float ry_bottom_left) {
    roundedRectangle(*this, x, y, width, height, rx_top_left, ry_top_left, rx_top_right,
                     ry_top_right, rx_bottom_right, ry_bottom_right, rx_bottom_left, ry_bottom_left);
  }

  void Path::addRoundedRectangle(float x, float y, float width, float height, float rx, float ry) {
    roundedRectangle(*this, x, y, width, height, rx, ry);
  }

  void Path::addEllipse(float cx, float cy, float rx, float ry) {
    moveTo(cx + rx, cy);
    arcTo(rx, ry, 180.0f, false, true, Point(cx - rx, cy), false);
    arcTo(rx, ry, 180.0f, false, true, Point(cx + rx, cy), false);

    close();
  }

  void Path::addCircle(float cx, float cy, float r) {
    addEllipse(cx, cy, r, r);
  }

  void Path::loadSvgPath(const std::string& path) {
    loadCommands(parseSvgPath(path));
  }

  void Path::loadCommands(const CommandList& commands) {
    startNewPath();
    for (const auto& command : commands) {
      switch (command.type) {
      case 'M': moveTo(command.end); break;
      case 'L': lineTo(command.end); break;
      case 'H': horizontalTo(command.end.x); break;
      case 'V': verticalTo(command.end.y); break;
      case 'Q': quadraticTo(command.control1, command.end); break;
      case 'T': smoothQuadraticTo(command.end); break;
      case 'C': bezierTo(command.control1, command.control2, command.end); break;
      case 'S': smoothBezierTo(command.control1, command.end); break;
      case 'A':
        arcTo(command.control1.x, command.control1.y, command.control2.x,
              command.flags & CommandList::kLargeArc, command.flags & CommandList::kSweep, command.end);
        break;
      case 'Z': close(); break;
      default: VISAGE_ASSERT(false); break;
      }
    }
  }

  Path::TriangulationGraph::TriangulationGraph(const Path* path) {
    int num_points = path->numPoints();
    resolution_transform_ = path->resolutionMatrix();
    prev_edge_.reserve(num_points);
    next_edge_.reserve(num_points);

    points_.reserve(num_points);
    int path_start = 0;
    for (const auto& sub_path : path->subPaths()) {
      int sub_path_size = sub_path.points.size();
      if (sub_path_size == 0)
        continue;

      for (int i = 0; i < sub_path_size; ++i) {
        points_.emplace_back(sub_path.points[i]);
        prev_edge_.push_back(path_start + (i - 1 + sub_path_size) % sub_path_size);
        next_edge_.push_back(path_start + (i + 1) % sub_path_size);
      }

      path_start += sub_path_size;
      VISAGE_ASSERT(next_edge_.size() == points_.size());
    }

    scan_line_ = std::make_unique<ScanLine>(this);
    simplify();
  }

  Path::Triangulation Path::TriangulationGraph::triangulate(Path::FillRule fill_rule, int minimum_cycles) {
    removeLinearPoints();
    breakIntersections();
    fixWindings(fill_rule, minimum_cycles);
    breakSimpleIntoMonotonicPolygons();
    Triangulation result;
    result.triangles = breakIntoTriangles();
    for (const auto& point : points_)
      result.points.emplace_back(point);
    return result;
  }

  auto Path::TriangulationGraph::ScanLine::safePrev(const std::vector<ScanLineArea>::iterator& it) {
    if (it != areas_.begin())
      return std::prev(it);
    return areas_.end();
  }

  auto Path::TriangulationGraph::ScanLine::safePrev(const std::vector<ScanLineArea>::const_iterator& it) const {
    if (it != areas_.begin())
      return std::prev(it);
    return areas_.end();
  }

  bool Path::TriangulationGraph::ScanLine::splitIntersection() {
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

  Path::TriangulationGraph::ScanLine::IntersectionType Path::TriangulationGraph::ScanLine::
      intersectionType(std::vector<ScanLineArea>::iterator it) {
    if (it == areas_.end())
      return IntersectionType::None;
    auto next = std::next(it);
    if (next == areas_.end())
      return IntersectionType::None;

    if (it->from.y < next->from.y && it->to.y < next->from.y && it->from.y < next->to.y &&
        it->to.y < next->to.y)
      return IntersectionType::None;

    if (it->from == next->to || next->from == it->to || it->to_index == next->to_index)
      return IntersectionType::None;

    const auto& area1 = *it;
    const auto& area2 = *next;

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

  void Path::TriangulationGraph::ScanLine::checkAddIntersection(const std::vector<ScanLineArea>::iterator& it) {
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

      if ((intersection - it->from).squareMagnitude() < kEpsilon)
        intersection = it->from;
      else if ((intersection - it->to).squareMagnitude() < kEpsilon)
        intersection = it->to;
      else if ((intersection - next->from).squareMagnitude() < kEpsilon)
        intersection = next->from;
      else if ((intersection - next->to).squareMagnitude() < kEpsilon)
        intersection = next->to;

      double min_x = std::max(it->from.x, next->from.x);
      double max_x = std::min(it->to.x, next->to.x);
      intersection.x = std::clamp(intersection.x, min_x, max_x);
      double min_y = std::max(std::min(it->from.y, it->to.y), std::min(next->from.y, next->to.y));
      double max_y = std::min(std::max(it->from.y, it->to.y), std::max(next->from.y, next->to.y));
      max_y = std::max(min_y, max_y);
      intersection.y = std::clamp(intersection.y, min_y, max_y);
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

  void Path::TriangulationGraph::ScanLine::checkRemoveIntersection(const std::vector<ScanLineArea>::iterator& it) {
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

  void Path::TriangulationGraph::ScanLine::processPointEvents(Event ev) {
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

  void Path::TriangulationGraph::ScanLine::updateNormalEvent(const Event& ev) {
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

  void Path::TriangulationGraph::ScanLine::updateDegeneracy(const Event& ev) {
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

    areas_.insert(last_position1_, next_areas_.begin(), next_areas_.end());
    VISAGE_ASSERT(areas_.size() % 2 == 0);

    old_areas_.clear();
    new_areas_.clear();
  }

  bool Path::TriangulationGraph::ScanLine::updateSplitIntersections() {
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

  void Path::TriangulationGraph::ScanLine::updateBreakIntersections() {
    Event ev = nextEvent();
    if (ev.degeneracy)
      updateDegeneracy(ev);
    else
      updateNormalEvent(ev);
  }

  void Path::TriangulationGraph::breakIntersections() {
    if (intersections_broken_)
      return;
    intersections_broken_ = true;

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
  }

  void Path::TriangulationGraph::fixWindings(Path::FillRule fill_rule, int minimum_cycles) {
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

  void Path::TriangulationGraph::reverse() {
    for (int i = 0; i < points_.size(); ++i)
      std::swap(prev_edge_[i], next_edge_[i]);
  }

  void Path::TriangulationGraph::breakSimpleIntoMonotonicPolygons() {
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

  std::vector<int> Path::TriangulationGraph::breakIntoTriangles() {
    // TODO switch to Delaunay triangulation
    std::vector<int> triangles;
    auto sorted_indices = sortedIndices();
    std::unique_ptr<bool[]> touched = std::make_unique<bool[]>(points_.size());

    for (const auto& index : *sorted_indices) {
      touched[index.index] = true;
      cutEars(index.index, triangles, touched);
    }

    return triangles;
  }

  void Path::TriangulationGraph::singlePointOffset(double amount, int index, Path::EndCap end_cap,
                                                   std::vector<int>& points_created) {
    if (amount < 0.0)
      return;

    auto point = points_[index];
    auto next_index = next_edge_[index];
    if (end_cap == Path::EndCap::Square) {
      points_[index] += DPoint(amount, amount);
      int current_index = index;
      current_index = insertPointBetween(current_index, next_index, point + DPoint(amount, -amount));
      current_index = insertPointBetween(current_index, next_index, point + DPoint(-amount, -amount));
      current_index = insertPointBetween(current_index, next_index, point + DPoint(-amount, amount));
      points_created.push_back(4);
    }
    else if (end_cap == Path::EndCap::Round) {
      float adjusted_radius = (resolution_transform_ * Point(amount, 0.0f)).length();
      double max_delta_radians = 2.0 * std::acos(1.0 - Path::kDefaultErrorTolerance / adjusted_radius);
      int num_points = std::ceil(2.0 * kPi / max_delta_radians - 0.1);
      std::complex<double> position(amount, 0.0);
      double angle_delta = 2.0 * kPi / num_points;
      std::complex<double> rotation = std::polar(1.0, -angle_delta);
      int current_index = index;

      points_[index] += DPoint(position.real(), position.imag());
      for (int p = 1; p < num_points; ++p) {
        position *= rotation;
        DPoint insert = point + DPoint(position.real(), position.imag());
        current_index = insertPointBetween(current_index, next_index, insert);
      }
      points_created.push_back(num_points);
    }
  }

  void Path::TriangulationGraph::offset(double amount, bool post_simplify, Path::Join join,
                                        Path::EndCap end_cap, std::vector<int>& points_created,
                                        float miter_limit) {
    static constexpr double kMinOffset = 0.001;
    if (std::abs(amount) < kMinOffset)
      return;

    intersections_broken_ = false;

    float square_miter_limit = miter_limit * miter_limit;
    float adjusted_radius = (resolution_transform_ * Point(amount, 0.0f)).length();
    double max_delta_radians = 2.0 * std::acos(1.0 - Path::kDefaultErrorTolerance / adjusted_radius);
    int start_points = points_.size();
    std::unique_ptr<bool[]> touched = std::make_unique<bool[]>(start_points);
    for (int i = 0; i < start_points; ++i) {
      if (touched[i])
        continue;

      if (next_edge_[i] == i) {
        singlePointOffset(amount, i, end_cap, points_created);
        continue;
      }

      DPoint start = points_[i];
      int last_index = prev_edge_[i];
      DPoint prev = points_[last_index];
      DPoint point = start;
      DPoint prev_direction = (start - prev).normalized();
      DPoint prev_offset = DPoint(-prev_direction.y, prev_direction.x) * amount;
      int index = i;
      while (!touched[index]) {
        touched[index] = true;
        int next_index = next_edge_[index];
        DPoint next = index == last_index ? start : points_[next_index];
        DPoint direction = (next - point).normalized();
        auto offset = DPoint(-direction.y, direction.x) * amount;

        auto type = join;
        if (prev == next) {
          if (end_cap == Path::EndCap::Butt)
            type = Path::Join::Bevel;
          else if (end_cap == Path::EndCap::Square)
            type = Path::Join::Square;
          else if (end_cap == Path::EndCap::Round)
            type = Path::Join::Round;
        }
        if (type == Path::Join::Bevel) {
          points_[index] += prev_offset;
          insertPointBetween(index, next_index, point + offset);
          points_created.push_back(2);
        }
        else if (type == Path::Join::Square) {
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
        else if (type == Path::Join::Round) {
          bool convex = stableOrientation(prev, point, next) <= 0.0;
          if (convex == (amount > 0.0)) {
            double acos_param = std::clamp(prev_offset.dot(offset) / (amount * amount), -1.0, 1.0);
            double arc_angle = std::acos(acos_param);
            points_[index] += prev_offset;
            int num_points = std::max(0.0, std::ceil(arc_angle / max_delta_radians - 0.1));
            std::complex<double> position(prev_offset.x, prev_offset.y);
            double angle_delta = arc_angle / (num_points + 1);
            std::complex<double> rotation = std::polar(1.0, (amount < 0.0) ? angle_delta : -angle_delta);
            int current_index = index;

            for (int p = 0; p < num_points; ++p) {
              position *= rotation;
              DPoint insert = point + DPoint(position.real(), position.imag());
              current_index = insertPointBetween(current_index, next_index, insert);
            }
            points_created.push_back(num_points + 1);
          }
          else
            type = Path::Join::Miter;
        }
        if (type == Path::Join::Miter) {
          auto intersection = Path::findIntersection(prev + prev_offset, point + prev_offset,
                                                     point + offset, next + offset);
          if (intersection.has_value() &&
              (intersection.value() - point).squareMagnitude() / (amount * amount) < square_miter_limit) {
            points_[index] = intersection.value();
            points_created.push_back(1);
          }
          else {
            points_[index] += prev_offset;
            if (point + offset == points_[index])
              points_created.push_back(1);
            else {
              insertPointBetween(index, next_index, point + offset);
              points_created.push_back(2);
            }
          }
        }

        if (index == last_index)
          break;

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

  void Path::TriangulationGraph::combine(const TriangulationGraph& other) {
    int offset = points_.size();
    for (int i = 0; i < other.points_.size(); ++i) {
      points_.push_back(other.points_[i]);
      prev_edge_.push_back(other.prev_edge_[i] + offset);
      next_edge_.push_back(other.next_edge_[i] + offset);
    }
    intersections_broken_ = false;

    VISAGE_ASSERT(checkValidPolygons());
  }

  void Path::TriangulationGraph::removeLinearPoints() {
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

  void Path::TriangulationGraph::simplify() {
    for (int i = 0; i < points_.size(); ++i) {
      if (i == next_edge_[i])
        continue;

      if (next_edge_[i] != -1 && points_[i] == points_[next_edge_[i]])
        removeFromCycle(i);
    }
  }

  Path Path::TriangulationGraph::toPath() const {
    std::unique_ptr<bool[]> visited = std::make_unique<bool[]>(points_.size());
    Path path;
    for (int i = 0; i < points_.size(); ++i) {
      if (i == next_edge_[i] || visited[i])
        continue;

      path.moveTo(Point(points_[i]));

      int index = next_edge_[i];
      while (!visited[index]) {
        path.lineTo(Point(points_[index]));
        visited[index] = true;
        index = next_edge_[index];
      }

      path.close();
    }
    return path;
  }

  Path::TriangulationGraph::PointType Path::TriangulationGraph::pointType(int index) const {
    int prev_index = prev_edge_[index];
    if (prev_index == index)
      return PointType::None;

    int next_index = next_edge_[index];
    if (prev_index < 0)
      prev_index = index;
    if (next_index < 0)
      next_index = index;
    double compare_prev = points_[index].compare(points_[prev_index]);
    double compare_next = points_[index].compare(points_[next_index]);

    if (compare_prev < 0.0 && compare_next < 0.0)
      return PointType::Begin;
    if (compare_prev > 0.0 && compare_next > 0.0)
      return PointType::End;
    return PointType::Continue;
  }

  int Path::TriangulationGraph::addAdditionalPoint(const DPoint& point) {
    points_.push_back(point);
    int new_index = points_.size() - 1;
    prev_edge_.push_back(new_index);
    next_edge_.push_back(new_index);
    return new_index;
  }

  int Path::TriangulationGraph::insertPointBetween(int start_index, int end_index, const DPoint& point) {
    if (next_edge_[start_index] != end_index)
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

  bool Path::TriangulationGraph::connected(int a_index, int b_index) const {
    return prev_edge_[a_index] == b_index || next_edge_[a_index] == b_index;
  }

  void Path::TriangulationGraph::connect(int from, int to) {
    next_edge_[from] = to;
    prev_edge_[to] = from;
  }

  void Path::TriangulationGraph::removeFromCycle(int index) {
    int prev = prev_edge_[index];
    int next = next_edge_[index];
    prev_edge_[index] = index;
    next_edge_[index] = index;
    connect(prev, next);
  }

  bool Path::TriangulationGraph::checkValidPolygons() const {
    for (int i = 0; i < points_.size(); ++i) {
      if (prev_edge_[next_edge_[i]] != i || next_edge_[prev_edge_[i]] != i)
        return false;
    }
    return true;
  }

  const std::vector<Path::TriangulationGraph::IndexData>* Path::TriangulationGraph::sortedIndices() {
    sorted_indices_.reserve(prev_edge_.size());
    auto unchanged = [this](IndexData& data) {
      const auto& point = points_[data.index];
      auto type = pointType(data.index);
      if (data.point == point && data.type == type)
        return true;

      data.point = point;
      data.type = type;
      return false;
    };

    int updated_index = std::stable_partition(sorted_indices_.begin(), sorted_indices_.end(), unchanged) -
                        sorted_indices_.begin();

    for (int i = sorted_indices_.size(); i < prev_edge_.size(); ++i)
      sorted_indices_.emplace_back(i, points_[i], pointType(i));

    std::sort(sorted_indices_.begin() + updated_index, sorted_indices_.end());
    std::inplace_merge(sorted_indices_.begin(), sorted_indices_.begin() + updated_index,
                       sorted_indices_.end());

    return &sorted_indices_;
  }

  void Path::TriangulationGraph::removeCycle(int start_index) {
    for (int i = start_index; next_edge_[i] != i;) {
      int next = next_edge_[i];
      prev_edge_[i] = i;
      next_edge_[i] = i;
      i = next;
    }
  }

  void Path::TriangulationGraph::reverseCycle(int start_index) {
    std::swap(prev_edge_[start_index], next_edge_[start_index]);
    for (int i = next_edge_[start_index]; i != start_index; i = next_edge_[i])
      std::swap(prev_edge_[i], next_edge_[i]);
  }

  int Path::TriangulationGraph::addDiagonal(int index, int target) {
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

  bool Path::TriangulationGraph::tryCutEar(int index, bool forward, std::vector<int>& triangles,
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

  void Path::TriangulationGraph::cutEars(int index, std::vector<int>& triangles,
                                         const std::unique_ptr<bool[]>& touched) {
    while (tryCutEar(index, true, triangles, touched))
      ;
    while (tryCutEar(index, false, triangles, touched))
      ;
  }

  Path::Triangulation Path::triangulate() {
    return triangulationGraph()->triangulate(fillRule());
  }

  Path Path::combine(Path& other, Operation operation) {
    switch (operation) {
    case Operation::Union: return combine(other, Path::FillRule::NonZero, 1, false);
    case Operation::Intersection: return combine(other, Path::FillRule::NonZero, 2, false);
    case Operation::Difference: return combine(other, Path::FillRule::Positive, 1, true);
    case Operation::Xor: return combine(other, Path::FillRule::EvenOdd, 1, false);
    default: return *this;
    }
  }

  Path Path::combine(Path& other, Path::FillRule fill_rule, int num_cycles_needed, bool reverse_other) {
    triangulationGraph()->breakIntersections();
    other.triangulationGraph()->breakIntersections();

    TriangulationGraph graph = *triangulationGraph();
    TriangulationGraph other_graph = *other.triangulationGraph();
    graph.fixWindings(fillRule());
    other_graph.fixWindings(other.fillRule());
    if (reverse_other)
      other_graph.reverse();

    graph.combine(other_graph);
    graph.breakIntersections();
    graph.fixWindings(fill_rule, num_cycles_needed);
    return graph.toPath();
  }

  std::pair<Path, Path> Path::offsetAntiAlias(float scale, std::vector<int>& inner_added_points,
                                              std::vector<int>& outer_added_points) {
    TriangulationGraph outer(this);
    outer.simplify();
    TriangulationGraph inner = outer;
    outer.offset(0.5f / scale, false, Join::Round, EndCap::Butt, outer_added_points);
    inner.offset(-0.5f / scale, false, Join::Round, EndCap::Butt, inner_added_points);
    return { inner.toPath(), outer.toPath() };
  }

  Path Path::offset(float offset, Join join_type, float miter_limit) {
    TriangulationGraph graph = *triangulationGraph();
    std::vector<int> points_created;
    graph.offset(offset, true, join_type, EndCap::Butt, points_created, miter_limit);
    return graph.toPath();
  }

  Path Path::stroke(float stroke_width, Join join, EndCap end_cap, std::vector<float> dash_array,
                    float dash_offset, float miter_limit) {
    float dash_total = 0.0f;
    for (auto& dash : dash_array)
      dash_total += dash;

    if (dash_total <= 0.0f)
      dash_array.clear();

    if (dash_array.size() % 2 != 0)
      dash_total *= 2.0f;

    Path stroke_path;
    if (!dash_array.empty()) {
      if (dash_offset < 0.0f)
        dash_offset = dash_total - std::fmod(-dash_offset, dash_total);
      else
        dash_offset = std::fmod(dash_offset, dash_total);

      int dash_index = 0;
      bool fill = true;
      float dash_length = dash_array[0];
      while (dash_offset > dash_length) {
        dash_offset -= dash_length;
        dash_index = (dash_index + 1) % dash_array.size();
        dash_length = dash_array[dash_index];
        fill = !fill;
      }

      dash_length -= dash_offset;

      for (auto& path : paths_) {
        auto prev = path.points[0];
        stroke_path.moveTo(prev);
        for (int i = 1; i < path.points.size(); ++i) {
          stroke_path.setPointValue(path.values[i]);
          auto length = (path.points[i] - prev).length();
          while (length > dash_length) {
            auto ratio = dash_length / length;
            auto point = prev + (path.points[i] - prev) * ratio;

            if (fill)
              stroke_path.lineTo(point);
            else
              stroke_path.moveTo(point);

            prev = point;
            length -= dash_length;

            dash_index = (dash_index + 1) % dash_array.size();
            dash_length = dash_array[dash_index];
            fill = !fill;
          }
          if (fill)
            stroke_path.lineTo(path.points[i]);

          dash_length -= (path.points[i] - prev).length();
          prev = path.points[i];
        }
      }
    }
    else
      stroke_path = *this;

    for (auto& path : stroke_path.paths_) {
      if (path.points.size() > 2 && path.closed && path.points.front() == path.points.back()) {
        path.points.push_back(path.points.front());
        path.values.push_back(path.values.front());
        path.points.push_back(path.points[1]);
        path.values.push_back(path.values[1]);
      }

      int start_size = path.points.size();
      for (int i = start_size - 2; i > 0; --i) {
        path.points.push_back(path.points[i]);
        path.values.push_back(path.values[i]);
      }
    }

    TriangulationGraph graph(&stroke_path);
    graph.simplify();
    std::vector<int> points_created;
    graph.offset(stroke_width / 2.0f, true, join, end_cap, points_created, miter_limit);
    return graph.toPath();
  }

  Path Path::breakIntoSimplePolygons() {
    triangulationGraph()->breakIntersections();
    TriangulationGraph graph = *triangulationGraph();
    graph.fixWindings(fillRule());
    return graph.toPath();
  }
}