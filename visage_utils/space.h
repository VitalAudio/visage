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

#include "defines.h"

#include <algorithm>
#include <vector>

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

    int squareMagnitude() const { return x * x + y * y; }
    float length() const { return sqrtf(x * x + y * y); }
  };

  struct FloatPoint {
    float x = 0;
    float y = 0;

    FloatPoint() = default;
    FloatPoint(float initial_x, float initial_y) : x(initial_x), y(initial_y) { }
    FloatPoint(const Point& point) : x(point.x), y(point.y) { }

    FloatPoint operator+(const FloatPoint& other) const { return { x + other.x, y + other.y }; }
    FloatPoint operator-(const FloatPoint& other) const { return { x - other.x, y - other.y }; }
    FloatPoint operator*(float scalar) const { return { x * scalar, y * scalar }; }
    FloatPoint operator+(const Point& other) const { return { x + other.x, y + other.y }; }
    FloatPoint operator-(const Point& other) const { return { x - other.x, y - other.y }; }
    bool operator==(const FloatPoint& other) const { return x == other.x && y == other.y; }
    float operator*(const FloatPoint& other) const { return x * other.x + y * other.y; }

    float squareMagnitude() const { return x * x + y * y; }
    float length() const { return sqrtf(squareMagnitude()); }
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
    Point clampPoint(const Point& point) const {
      return { std::max(x_, std::min(right(), point.x)), std::max(y_, std::min(bottom(), point.y)) };
    }

    void setX(int x) { x_ = x; }
    void setY(int y) { y_ = y; }
    void setWidth(int width) { width_ = width; }
    void setHeight(int height) { height_ = height; }
    void flipDimensions() {
      std::swap(x_, y_);
      std::swap(width_, height_);
    }

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

    // Create nonoverlapping rectangles that cover the same area as _rect1_ and _rect2_.
    // Input bounds are modified and additional rectangles needed are put into _pieces_.
    static void breakIntoNonOverlapping(Bounds& rect1, Bounds& rect2, std::vector<Bounds>& pieces) {
      Bounds original_new = rect1;
      Bounds original_old = rect2;
      if (!rect1.overlaps(rect2))
        return;

      Bounds subtraction;
      if (rect1.subtract(rect2, subtraction)) {
        rect1 = subtraction;
        return;
      }
      if (rect2.subtract(rect1, subtraction)) {
        rect2 = subtraction;
        return;
      }
      Bounds breaks[4];
      Bounds remaining = rect2;
      int index = 0;
      if (remaining.x() < rect1.x()) {
        breaks[index++] = { remaining.x(), remaining.y(), rect1.x() - remaining.x(), remaining.height() };
        remaining = { rect1.x(), remaining.y(), remaining.right() - rect1.x(), remaining.height() };
      }
      if (remaining.y() < rect1.y()) {
        breaks[index++] = { remaining.x(), remaining.y(), remaining.width(), rect1.y() - remaining.y() };
        remaining = { remaining.x(), rect1.y(), remaining.width(), remaining.bottom() - rect1.y() };
      }
      if (remaining.right() > rect1.right()) {
        breaks[index++] = { rect1.right(), remaining.y(), remaining.right() - rect1.right(),
                            remaining.height() };
        remaining = { remaining.x(), remaining.y(), rect1.right() - remaining.x(), remaining.height() };
      }
      if (remaining.bottom() > rect1.bottom()) {
        breaks[index++] = { remaining.x(), rect1.bottom(), remaining.width(),
                            remaining.bottom() - rect1.bottom() };
      }
      VISAGE_ASSERT(index == 2);

      rect2 = breaks[0];
      pieces.push_back(breaks[1]);
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
