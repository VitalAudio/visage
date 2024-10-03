/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <algorithm>

namespace visage {
  struct Point {
    int x = 0;
    int y = 0;

    Point() = default;
    Point(int initial_x, int initial_y) : x(initial_x), y(initial_y) { }

    Point operator+(const Point& other) const { return { x + other.x, y + other.y }; }

    Point operator+=(const Point& other) {
      x += other.x;
      y += other.y;
      return *this;
    }

    Point operator-(const Point& other) const { return { x - other.x, y - other.y }; }

    Point operator-=(const Point& other) {
      x -= other.x;
      y -= other.y;
      return *this;
    }

    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    bool operator!=(const Point& other) const { return !(*this == other); }
  };

  struct FloatPoint {
    float x = 0;
    float y = 0;

    FloatPoint() = default;
    FloatPoint(float initial_x, float initial_y) : x(initial_x), y(initial_y) { }
    explicit FloatPoint(const Point& point) : x(point.x), y(point.y) { }

    FloatPoint operator+(const FloatPoint& other) const { return { x + other.x, y + other.y }; }
    FloatPoint operator-(const FloatPoint& other) const { return { x - other.x, y - other.y }; }
    FloatPoint operator*(float scalar) const { return { x * scalar, y * scalar }; }
    bool operator==(const FloatPoint& other) const { return x == other.x && y == other.y; }
    float operator*(const FloatPoint& other) const { return x * other.x + y * other.y; }

    float squareMagnitude() const { return x * x + y * y; }
  };

  class Bounds {
  public:
    Bounds() = default;
    Bounds(int x, int y, int width, int height) : x_(x), y_(y), width_(width), height_(height) { }

    int x() const { return x_; }
    int y() const { return y_; }
    int width() const { return width_; }
    int height() const { return height_; }
    bool hasArea() const { return width_ > 0 && height_ > 0; }
    int right() const { return x_ + width_; }
    int bottom() const { return y_ + height_; }
    int xCenter() const { return x_ + width_ / 2; }
    int yCenter() const { return y_ + height_ / 2; }
    Point topLeft() const { return { x_, y_ }; }

    void setX(int x) { x_ = x; }
    void setY(int y) { y_ = y; }
    void setWidth(int width) { width_ = width; }
    void setHeight(int height) { height_ = height; }

    bool operator==(const Bounds& other) const {
      return x_ == other.x_ && y_ == other.y_ && width_ == other.width_ && height_ == other.height_;
    }

    bool operator!=(const Bounds& other) const { return !(*this == other); }
    bool contains(int x, int y) const { return x >= x_ && x < right() && y >= y_ && y < bottom(); }
    bool contains(const Point& point) const { return contains(point.x, point.y); }

    bool contains(const Bounds& other) const {
      return x_ <= other.x_ && y_ <= other.y_ && right() >= other.right() && bottom() >= other.bottom();
    }

    bool overlaps(const Bounds& other) const {
      return x_ < other.right() && right() > other.x_ && y_ < other.bottom() && bottom() > other.y_;
    }

    Bounds intersection(const Bounds& other) const {
      int x = std::max(x_, other.x_);
      int y = std::max(y_, other.y_);
      int r = std::min(right(), other.right());
      int b = std::min(bottom(), other.bottom());
      return { x, y, r - x, b - y };
    }

    // Returns true if subtracting the other rectangle result in only one rectangle
    // Stores that rectangle in _result_
    bool subtract(const Bounds& other, Bounds& result) const {
      bool left_edge_inside = x_ < other.x_ && other.x_ < right();
      bool right_edge_inside = x_ < other.right() && other.right() < right();
      bool top_edge_inside = y_ < other.y_ && other.y_ < bottom();
      bool bottom_edge_inside = y_ < other.bottom() && other.bottom() < bottom();
      int total_edges_inside = left_edge_inside + right_edge_inside + top_edge_inside + bottom_edge_inside;
      if (total_edges_inside > 1)
        return false;
      if (other.contains(*this)) {
        result = { x_, y_, 0, 0 };
        return true;
      }

      if (left_edge_inside)
        result = { x_, y_, other.x_ - x_, height_ };
      else if (right_edge_inside)
        result = { other.right(), y_, right() - other.right(), height_ };
      else if (top_edge_inside)
        result = { x_, y_, width_, other.y_ - y_ };
      else if (bottom_edge_inside)
        result = { x_, other.bottom(), width_, bottom() - other.bottom() };
      else
        result = *this;

      return true;
    }

    Bounds operator+(const Point& point) const {
      return { x_ + point.x, y_ + point.y, width_, height_ };
    }

  private:
    int x_ = 0;
    int y_ = 0;
    int width_ = 0;
    int height_ = 0;
  };

  static Point adjustBoundsForAspectRatio(Point current, Point min_bounds, Point max_bounds,
                                          float aspect_ratio, bool horizontal_resize,
                                          bool vertical_resize) {
    int width = std::max(min_bounds.x, std::min(max_bounds.x, current.x));
    int height = std::max(min_bounds.y, std::min(max_bounds.y, current.y));

    int width_from_height = std::max(min_bounds.x, std::min<int>(max_bounds.x, height * aspect_ratio));
    int height_from_width = std::max(min_bounds.y, std::min<int>(max_bounds.y, width / aspect_ratio));

    if (horizontal_resize && !vertical_resize)
      return { width, height_from_width };
    if (vertical_resize && !horizontal_resize)
      return { width_from_height, height };

    Point result = { width, height };
    if (width_from_height > width)
      result.x = width_from_height;
    if (height_from_width > height)
      result.y = height_from_width;
    return result;
  }
}
