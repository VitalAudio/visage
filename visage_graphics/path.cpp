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

#include "embedded/shaders.h"
#include "graphics_caches.h"
#include "shape_batcher.h"
#include "uniforms.h"

#include <bgfx/bgfx.h>
#include <complex>
#include <memory>

namespace visage {
  static float toFloat(const std::string& str) {
    try {
      return std::stof(str);
    }
    catch (...) {
      return 0.0f;
    }
  }

  template<typename T>
  static T clampedACos(T value) {
    if (value <= T(-1.0))
      return T(3.14159265358979323846);
    if (value >= T(1.0))
      return T(0.0);
    return std::acos(value);
  }

  template<typename T>
  static void roundedRectangle(T& t, float x, float y, float width, float height, float rx_top_left,
                               float ry_top_left, float rx_top_right, float ry_top_right,
                               float rx_bottom_right, float ry_bottom_right, float rx_bottom_left,
                               float ry_bottom_left) {
    float scale = 1.0f;
    if (rx_top_left + rx_top_right)
      scale = std::min(scale, width / (rx_top_left + rx_top_right));
    if (rx_bottom_left + rx_bottom_right)
      scale = std::min(scale, width / (rx_bottom_left + rx_bottom_right));
    if (ry_top_left + ry_bottom_left)
      scale = std::min(scale, height / (ry_top_left + ry_bottom_left));
    if (ry_top_right + ry_bottom_right)
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

  void Path::arcTo(float rx, float ry, float x_axis_rotation, bool large_arc, bool sweep_flag,
                   Point point, bool relative) {
    rx = std::abs(rx);
    ry = std::abs(ry);
    if (rx == 0.0f || ry == 0.0f) {
      lineTo(point);
      return;
    }

    if (currentPath().points.empty())
      addPoint(last_point_);

    Point from = last_point_;
    if (relative)
      point += last_point_;

    auto ellipse_rotation = Matrix::rotation(x_axis_rotation);
    Point delta = ellipse_rotation.transposed() * (point - from);
    float radius_ratio = rx / ry;
    delta.y *= radius_ratio;

    float length = delta.length();
    if (length == 0.0f)
      return;

    float radius = std::max(length * 0.5f, rx);
    float center_offset = std::sqrt(std::max(0.0f, radius * radius - length * length * 0.25f));
    Point normal = Point(delta.y, -delta.x) / length;
    if (large_arc != sweep_flag)
      normal = -normal;

    Point center = delta * 0.5f + normal * center_offset;
    float arc_angle = 2.0f * std::asin(length * 0.5f / radius);

    if (large_arc)
      arc_angle = 2.0f * kPi - arc_angle;
    if (!sweep_flag)
      arc_angle = -arc_angle;

    Point adjusted_radius = resolution_matrix_ * Point(rx, ry);
    float max_radius = std::max(std::abs(adjusted_radius.x), std::abs(adjusted_radius.y));
    float max_delta_radians = 2.0f * clampedACos(1.0f - error_tolerance_ / max_radius);
    int num_points = std::ceil(std::abs(arc_angle) / max_delta_radians);

    std::complex<float> position(-center.x, -center.y);
    float angle_delta = arc_angle / num_points;
    std::complex<float> rotation = std::polar(1.0f, angle_delta);

    for (int i = 0; i < num_points; ++i) {
      position *= rotation;
      Point p = center + Point(position.real(), position.imag());
      p.y /= radius_ratio;
      p = ellipse_rotation * p + from;
      addPoint(p);
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
            return toFloat(number);
        }
        number += str[i++];
      }
      else if (str[i] == ',' || std::isspace(str[i]) || sign) {
        if (!number.empty())
          return toFloat(number);
        if (!sign)
          ++i;
      }
      else if (std::isalpha(str[i]))
        break;
      else
        ++i;

      if (bit_flags && !number.empty())
        return toFloat(number);
    }
    if (number.empty()) {
      VISAGE_ASSERT(false);
      return 0.0f;
    }
    return toFloat(number);
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

  Path Path::combine(Path& other, FillRule fill_rule) const {
    Path combined = *this;
    for (auto& path : other.paths_)
      combined.paths_.push_back(path);
    combined.fill_rule_ = fill_rule;
    return combined;
  }

  Path::SubPath Path::singlePointOffset(Point point, float amount, EndCap end_cap) {
    SubPath sub_path;
    if (amount < 0.0)
      return sub_path;

    sub_path.closed = true;
    if (end_cap == EndCap::Square) {
      sub_path.points.push_back(point + Point(amount, amount));
      sub_path.points.push_back(point + Point(amount, -amount));
      sub_path.points.push_back(point + Point(-amount, -amount));
      sub_path.points.push_back(point + Point(-amount, amount));
    }
    else if (end_cap == EndCap::Round) {
      float adjusted_radius = (resolution_matrix_ * Point(amount, 0.0f)).length();
      float max_delta_radians = 2.0 * clampedACos(1.0 - kDefaultErrorTolerance / adjusted_radius);
      int num_points = std::max<int>(1, std::ceil(2.0 * kPi / max_delta_radians - 0.1));
      std::complex<float> position(amount, 0.0);
      float angle_delta = 2.0 * kPi / num_points;
      std::complex<float> rotation = std::polar(1.0f, -angle_delta);

      sub_path.points.push_back(point + Point(position.real(), position.imag()));
      for (int p = 1; p < num_points; ++p) {
        position *= rotation;
        sub_path.points.push_back(point + Point(position.real(), position.imag()));
      }
    }

    return sub_path;
  }

  Path Path::offset(float amount, Join join, EndCap end_cap, float miter_limit) {
    return offset(amount, join, Join::Miter, end_cap, miter_limit);
  }

  Path Path::offset(float amount, Join join, Join inner_join, EndCap end_cap, float miter_limit) {
    static constexpr float kMinOffset = 0.001f;
    Path result;

    if (std::abs(amount) < kMinOffset)
      return result;

    float square_miter_limit = miter_limit * miter_limit;
    float adjusted_radius = (resolution_matrix_ * Point(amount, 0.0f)).length();
    float max_delta_radians = 2.0f * clampedACos(1.0 - kDefaultErrorTolerance / adjusted_radius);
    for (const auto& sub_path : paths_) {
      if (sub_path.points.empty())
        continue;

      bool closed_points = sub_path.points.front() == sub_path.points.back();
      if (sub_path.points.size() == 1 || (sub_path.points.size() == 2 && closed_points)) {
        result.paths_.push_back(singlePointOffset(Point(sub_path.points[0]), amount, end_cap));
        continue;
      }

      std::vector<Point> new_path;
      Point point = sub_path.points.back();
      Point prev = sub_path.points[sub_path.points.size() - 2];
      Point prev_direction = (point - prev).normalized();
      Point prev_offset = Point(-prev_direction.y, prev_direction.x) * amount;

      for (const Point& next : sub_path.points) {
        if (next == point)
          continue;

        Point direction = (next - point).normalized();
        auto offset = Point(-direction.y, direction.x) * amount;

        auto type = join;
        if (prev == next) {
          if (end_cap == EndCap::Butt)
            type = Join::Bevel;
          else if (end_cap == EndCap::Square)
            type = Join::Square;
          else if (end_cap == EndCap::Round)
            type = Join::Round;
        }
        bool convex = stableOrientation(prev, point, next) <= 0.0;
        if (convex == (amount < 0.0f))
          type = inner_join;

        if (type == Join::Bevel) {
          new_path.push_back(point + prev_offset);
          new_path.push_back(point + offset);
        }
        else if (type == Join::Square) {
          Point square_offset = (prev_direction - direction).normalized() * amount;
          Point square_center = point + square_offset;
          Point square_tangent = { -square_offset.y, square_offset.x };
          auto intersection_prev = findIntersection(square_center, square_center + square_tangent,
                                                    prev + prev_offset, point + prev_offset);
          auto intersection = findIntersection(square_center, square_center + square_tangent,
                                               point + offset, next + offset);
          VISAGE_ASSERT(intersection_prev.has_value());
          VISAGE_ASSERT(intersection.has_value());
          new_path.push_back(intersection_prev.value());
          new_path.push_back(intersection.value());
        }
        else if (type == Join::Round) {
          float arc_angle = clampedACos(prev_offset.dot(offset) / (amount * amount));
          new_path.push_back(point + prev_offset);
          int num_points = std::max(0.0, std::ceil(arc_angle / max_delta_radians - 0.1));
          std::complex<float> position(prev_offset.x, prev_offset.y);
          float angle_delta = arc_angle / (num_points + 1);
          std::complex<float> rotation = std::polar(1.0f, (amount < 0.0f) ? angle_delta : -angle_delta);

          for (int p = 0; p <= num_points; ++p) {
            position *= rotation;
            new_path.push_back(point + Point(position.real(), position.imag()));
          }
        }
        else if (type == Join::Miter) {
          auto intersection = findIntersection(prev + prev_offset, point + prev_offset,
                                               point + offset, next + offset);
          if (intersection.has_value() &&
              (intersection.value() - point).squareMagnitude() / (amount * amount) < square_miter_limit) {
            new_path.push_back(intersection.value());
          }
          else {
            new_path.push_back(point + prev_offset);
            if (point + offset != point)
              new_path.push_back(point + offset);
          }
        }

        prev = point;
        point = next;
        prev_direction = direction;
        prev_offset = offset;
      }

      result.paths_.push_back(SubPath { std::move(new_path), true });
    }

    return result;
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
        if (path.points.empty())
          continue;

        auto prev = path.points[0];
        stroke_path.moveTo(prev);
        for (int i = 1; i < path.points.size(); ++i) {
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

    std::vector<SubPath> inner_paths;
    for (auto& path : stroke_path.paths_) {
      if (path.points.size() > 1 && path.closed) {
        SubPath inner = path;
        std::reverse(inner.points.begin(), inner.points.end());
        inner_paths.push_back(inner);
      }

      else {
        int start_size = path.points.size();
        for (int i = start_size - 2; i > 0; --i)
          path.points.push_back(path.points[i]);
      }
    }

    stroke_path.paths_.insert(stroke_path.paths_.end(), inner_paths.begin(), inner_paths.end());
    stroke_path = stroke_path.offset(stroke_width / 2.0f, join, Join::Bevel, end_cap, miter_limit);
    stroke_path.fill_rule_ = FillRule::NonZero;
    return stroke_path;
  }

  struct PathAtlasTexture {
    bgfx::FrameBufferHandle handle = { bgfx::kInvalidHandle };

    ~PathAtlasTexture() {
      if (bgfx::isValid(handle))
        bgfx::destroy(handle);
    }
  };

  PathAtlas::PackedPathReference::~PackedPathReference() {
    if (auto atlas_pointer = atlas.lock())
      (*atlas_pointer)->removePath(packed_path_rect);
  }

  PathAtlas::PathAtlas() {
    reference_ = std::make_shared<PathAtlas*>(this);
    atlas_map_.setPadding(kBuffer);
  }

  PathAtlas::~PathAtlas() = default;

  template<const char* name>
  void setPathUniform(float value0, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f) {
    float values[4] = { value0, value1, value2, value3 };
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    bgfx::setUniform(uniform, values);
  }

  bool PathAtlas::clearUpdatedPathAreas(int submit_pass) {
    int total_need_update = 0;
    for (auto& path : paths_) {
      if (path->needs_update)
        ++total_need_update;
    }

    if (total_need_update == 0)
      return false;

    float width_scale = 2.0f / width_;
    float height_scale = 2.0f / height_;

    bgfx::setViewMode(submit_pass, bgfx::ViewMode::Sequential);
    bgfx::setViewRect(submit_pass, 0, 0, width_, height_);
    bgfx::setViewFrameBuffer(submit_pass, frame_buffer_->handle);
    bgfx::setState(BGFX_STATE_WRITE_R | BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ZERO));
    auto clear_vertices = initQuadVertices<UvVertex>(total_need_update);
    int vertex_index = 0;
    for (auto& path : paths_) {
      if (path->needs_update) {
        float left = path->x * width_scale - 1.0f;
        float top = 1.0f - path->y * height_scale;
        float right = left + path->w * width_scale;
        float bottom = top - path->h * height_scale;
        clear_vertices[vertex_index].x = left;
        clear_vertices[vertex_index].y = top;
        clear_vertices[vertex_index + 1].x = right;
        clear_vertices[vertex_index + 1].y = top;
        clear_vertices[vertex_index + 2].x = left;
        clear_vertices[vertex_index + 2].y = bottom;
        clear_vertices[vertex_index + 3].x = right;
        clear_vertices[vertex_index + 3].y = bottom;
        clear_vertices[vertex_index].u = 1.0f;
        clear_vertices[vertex_index].v = 0.0f;
        clear_vertices[vertex_index + 1].u = 1.0f;
        clear_vertices[vertex_index + 1].v = 0.0f;
        clear_vertices[vertex_index + 2].u = 0.0f;
        clear_vertices[vertex_index + 2].v = 0.0f;
        clear_vertices[vertex_index + 3].u = 0.0f;
        clear_vertices[vertex_index + 3].v = 0.0f;
        vertex_index += 4;
      }
    }

    if (bgfx::getCaps()->originBottomLeft) {
      for (int i = 0; i < vertex_index; ++i)
        clear_vertices[i].y = -clear_vertices[i].y;
    }

    setPathUniform<Uniforms::kColor>(0.0f);
    bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_clear, shaders::fs_clear));

    return true;
  }

  int PathAtlas::updatePaths(int submit_pass) {
    constexpr int kTriangleIndices[] = { 0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5 };
    constexpr int kConservativeVerticesPerTriangle = 3;
    constexpr int kRegularVerticesPerTriangle = 6;
    constexpr float kTriangleDrawOffset = 2.0f;

    checkInit();

    if (!clearUpdatedPathAreas(submit_pass))
      return submit_pass;

    int num_triangles = 0;
    for (auto& path : paths_) {
      if (path->needs_update) {
        for (const auto& sub_path : path->path.subPaths()) {
          if (sub_path.points.size() > 2)
            num_triangles += sub_path.points.size();
        }
      }
    }

    auto state = BGFX_STATE_WRITE_R | BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ONE);
    int vertices_per_triangle = kRegularVerticesPerTriangle;
    int indices_per_triangle = sizeof(kTriangleIndices) / sizeof(int);
    int vertices_per_point = 2;
    bool conservative_raster = bgfx::getCaps()->supported & BGFX_CAPS_CONSERVATIVE_RASTER;
    if (conservative_raster) {
      state |= BGFX_STATE_CONSERVATIVE_RASTER;
      vertices_per_triangle = kConservativeVerticesPerTriangle;
      indices_per_triangle = 3;
      vertices_per_point = 1;
    }

    bgfx::setState(state);
    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::TransientIndexBuffer index_buffer {};
    int num_vertices = num_triangles * vertices_per_triangle;
    int num_indices = num_triangles * indices_per_triangle;
    if (!bgfx::allocTransientBuffers(&vertex_buffer, PathVertex::layout(), num_vertices,
                                     &index_buffer, num_indices, true)) {
      VISAGE_LOG("PathAtlas::updatePaths: Failed to allocate transient buffers");
      return submit_pass + 1;
    }

    auto vertices = reinterpret_cast<PathVertex*>(vertex_buffer.data);
    auto indices = reinterpret_cast<uint32_t*>(index_buffer.data);
    if (indices == nullptr || vertices == nullptr) {
      VISAGE_LOG("PathAtlas::updatePaths: Failed to map transient buffers");
      return submit_pass + 1;
    }

    bgfx::setVertexBuffer(0, &vertex_buffer);
    bgfx::setIndexBuffer(&index_buffer);

    uint32_t vertex = 0;
    uint32_t triangle_index = 0;
    for (auto& path : paths_) {
      if (path->needs_update) {
        path->needs_update = false;

        for (const auto& sub_path : path->path.subPaths()) {
          if (sub_path.points.size() <= 2)
            continue;

          float average_x = 0.0f;
          float max_y = 0.0f;
          for (const auto& point : sub_path.points) {
            max_y = std::max(max_y, point.y);
            average_x += point.x;
          }
          average_x /= static_cast<float>(sub_path.points.size());

          float x = path->x;
          float y = path->y;
          float anchor_x = x + average_x;
          float anchor_y = y + max_y + kTriangleDrawOffset;
          float last_x = x + sub_path.points.back().x;
          float last_y = y + sub_path.points.back().y;
          for (int i = 0; i < sub_path.points.size(); ++i) {
            for (int k = 0; k < indices_per_triangle; ++k)
              indices[triangle_index++] = vertex + kTriangleIndices[k];

            float new_x = x + sub_path.points[i].x;
            float new_y = y + sub_path.points[i].y;
            float index = 0.0f;

            for (int v = vertex; v < vertex + vertices_per_triangle;) {
              float direction = 1.0f;
              for (int p = 0; p < vertices_per_point; ++p, ++v) {
                vertices[v].index = index;
                vertices[v].direction = direction;
                direction *= -1.0f;
              }
              index += 1.0f;
            }

            for (int v = 0; v < vertices_per_triangle; ++v) {
              vertices[vertex].x1 = anchor_x;
              vertices[vertex].y1 = anchor_y;
              vertices[vertex].x2 = last_x;
              vertices[vertex].y2 = last_y;
              vertices[vertex].x3 = new_x;
              vertices[vertex].y3 = new_y;
              ++vertex;
            }

            last_x = new_x;
            last_y = new_y;
          }
        }
      }
    }

    VISAGE_ASSERT(vertex == num_vertices);
    VISAGE_ASSERT(triangle_index == num_indices);

    bool origin_flip = bgfx::getCaps()->originBottomLeft;
    setPathUniform<Uniforms::kColor>(1.0f);
    setPathUniform<Uniforms::kOriginFlip>(origin_flip ? -1.0 : 1.0);

    if (origin_flip)
      setPathUniform<Uniforms::kBounds>(2.0f / width_, 2.0f / height_, -1.0f, -1.0f);
    else
      setPathUniform<Uniforms::kBounds>(2.0f / width_, -2.0f / height_, -1.0f, 1.0f);

    if (conservative_raster)
      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_conservative_path_fill,
                                                            shaders::fs_path_fill));
    else
      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_path_fill, shaders::fs_path_fill));
    return submit_pass + 1;
  }

  void PathAtlas::destroy() {
    frame_buffer_.reset();
  }

  void PathAtlas::checkInit() {
    constexpr uint64_t kFlags = BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP | BGFX_SAMPLER_V_CLAMP;
    if (needs_packing_) {
      resize();
      needs_packing_ = false;
    }

    if (frame_buffer_ == nullptr)
      frame_buffer_ = std::make_unique<PathAtlasTexture>();

    if (!bgfx::isValid(frame_buffer_->handle) && atlas_map_.width() && atlas_map_.height()) {
      frame_buffer_->handle = bgfx::createFrameBuffer(atlas_map_.width(), atlas_map_.height(),
                                                      bgfx::TextureFormat::R16F, kFlags);
      width_ = atlas_map_.width();
      height_ = atlas_map_.height();
    }
  }

  void PathAtlas::resize() {
    static constexpr float kShrinkFactor = 0.5f;
    atlas_map_.pack(width_, height_);
    if (atlas_map_.width() > width_ || atlas_map_.height() > height_ ||
        atlas_map_.width() < width_ * kShrinkFactor || atlas_map_.height() < height_ * kShrinkFactor)
      frame_buffer_.reset();

    for (auto& path : paths_) {
      const PackedRect& rect = atlas_map_.rectForId(path.get());
      path->x = rect.x;
      path->y = rect.y;
      path->w = rect.w;
      path->h = rect.h;
      path->needs_update = true;
    }
  }

  const bgfx::FrameBufferHandle& PathAtlas::frameBufferHandle() {
    return frame_buffer_->handle;
  }
}