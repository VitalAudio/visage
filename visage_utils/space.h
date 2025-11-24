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
#include <cmath>
#include <cstdint>
#include <vector>

namespace visage {
  template<typename T>
  struct BasePoint {
    T x = 0;
    T y = 0;

    BasePoint() = default;
    BasePoint(T initial_x, T initial_y) : x(initial_x), y(initial_y) { }
    template<typename Base>
    explicit BasePoint(const BasePoint<Base>& other) {
      x = other.x;
      y = other.y;
    }

    BasePoint& operator=(const BasePoint& other) = default;

    BasePoint operator+(const BasePoint& other) const { return { x + other.x, y + other.y }; }
    BasePoint& operator+=(const BasePoint& other) {
      x += other.x;
      y += other.y;
      return *this;
    }
    BasePoint operator-(const BasePoint& other) const { return { x - other.x, y - other.y }; }
    BasePoint& operator-=(const BasePoint& other) {
      x -= other.x;
      y -= other.y;
      return *this;
    }

    BasePoint operator-() const { return { -x, -y }; }

    T dot(const BasePoint& other) const { return x * other.x + y * other.y; }
    T operator*(const BasePoint& other) const { return dot(other); }

    BasePoint operator*(T scalar) const { return { x * scalar, y * scalar }; }
    BasePoint& operator*=(T scalar) {
      x *= scalar;
      y *= scalar;
      return *this;
    }
    friend BasePoint operator*(T scalar, const BasePoint& point) {
      return { point.x * scalar, point.y * scalar };
    }

    BasePoint operator/(T scalar) const { return { x / scalar, y / scalar }; }
    BasePoint& operator/=(T scalar) {
      x /= scalar;
      y /= scalar;
      return *this;
    }
    friend BasePoint operator/(T scalar, const BasePoint& point) {
      return { point.x / scalar, point.y / scalar };
    }

    bool operator==(const BasePoint& other) const noexcept { return x == other.x && y == other.y; }
    bool operator!=(const BasePoint& other) const noexcept { return !(*this == other); }
    bool operator<(const BasePoint& other) const noexcept {
      return x < other.x || (x == other.x && y < other.y);
    }
    bool operator>(const BasePoint& other) const noexcept {
      return x > other.x || (x == other.x && y > other.y);
    }
    bool operator<=(const BasePoint& other) const noexcept {
      return x < other.x || (x == other.x && y <= other.y);
    }
    bool operator>=(const BasePoint& other) const noexcept {
      return x > other.x || (x == other.x && y >= other.y);
    }
    T compare(const BasePoint& other) const noexcept {
      if (x != other.x)
        return x < other.x ? -1 : 1;
      if (y != other.y)
        return y < other.y ? -1 : 1;
      return 0;
    }

    T squareMagnitude() const noexcept { return x * x + y * y; }
    auto length() const noexcept {
      if constexpr (std::is_same_v<T, float>)
        return sqrtf(squareMagnitude());
      else
        return sqrt(squareMagnitude());
    }

    BasePoint normalized() const {
      T magnitude = length();
      if (magnitude)
        return BasePoint(x / magnitude, y / magnitude);
      return *this;
    }
    void normalize() { *this = normalized(); }

    T cross(const BasePoint& other) const noexcept { return x * other.y - y * other.x; }

    auto round() const {
      if constexpr (std::is_integral_v<T>)
        return *this;
      else if constexpr (std::is_same_v<T, float>)
        return BasePoint<int>(std::round(x), std::round(y));
      else
        return BasePoint<int64_t>(std::round(x), std::round(y));
    }
  };

  template<typename T>
  struct BaseMatrix {
    static constexpr auto kPi = std::is_same_v<T, float> ? 3.14159265358979323846f : 3.14159265358979323846;

    T matrix[2][2] = { { 1, 0 }, { 0, 1 } };

    BaseMatrix() = default;

    BaseMatrix(T a1, T a2, T b1, T b2) {
      matrix[0][0] = a1;
      matrix[0][1] = a2;
      matrix[1][0] = b1;
      matrix[1][1] = b2;
    }

    BaseMatrix operator*(const BaseMatrix& other) const {
      return { matrix[0][0] * other.matrix[0][0] + matrix[0][1] * other.matrix[1][0],
               matrix[0][0] * other.matrix[0][1] + matrix[0][1] * other.matrix[1][1],
               matrix[1][0] * other.matrix[0][0] + matrix[1][1] * other.matrix[1][0],
               matrix[1][0] * other.matrix[0][1] + matrix[1][1] * other.matrix[1][1] };
    }

    BasePoint<T> operator*(const BasePoint<T>& point) const {
      return { matrix[0][0] * point.x + matrix[0][1] * point.y,
               matrix[1][0] * point.x + matrix[1][1] * point.y };
    }

    BaseMatrix operator*(T scalar) const {
      return { matrix[0][0] * scalar, matrix[0][1] * scalar, matrix[1][0] * scalar, matrix[1][1] * scalar };
    }

    BaseMatrix operator/(T scalar) const {
      if (scalar == 0)
        return { 0, 0, 0, 0 };
      return { matrix[0][0] / scalar, matrix[0][1] / scalar, matrix[1][0] / scalar, matrix[1][1] / scalar };
    }

    static BaseMatrix identity() { return { 1, 0, 0, 1 }; }

    static BaseMatrix rotation(T angle) {
      T cos_angle = std::cos(angle * T(kPi) / T(180));
      T sin_angle = std::sin(angle * T(kPi) / T(180));
      return { cos_angle, -sin_angle, sin_angle, cos_angle };
    }

    static BaseMatrix scale(T scale_x, T scale_y) { return { scale_x, T(0), T(0), scale_y }; }

    static BaseMatrix skewX(T skew) {
      return { T(1), T(std::tan(skew * T(kPi) / T(180))), T(0), T(1) };
    }

    static BaseMatrix skewY(T skew) {
      return { T(1), T(0), T(std::tan(skew * T(kPi) / T(180))), T(1) };
    }

    BaseMatrix transposed() const {
      return { matrix[0][0], matrix[1][0], matrix[0][1], matrix[1][1] };
    }

    BaseMatrix inversed() const {
      T det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
      if (std::abs(det) < std::numeric_limits<T>::epsilon())
        return { 0, 0, 0, 0 };
      return { matrix[1][1] / det, -matrix[0][1] / det, -matrix[1][0] / det, matrix[0][0] / det };
    }

    T determinant() const { return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]; }
  };

  template<typename T>
  struct BaseTransform {
    BaseMatrix<T> matrix;
    BasePoint<T> translate;

    BaseTransform() = default;

    BaseTransform(BaseMatrix<T> matrix, BasePoint<T> translate) :
        matrix(matrix), translate(translate) { }

    BaseTransform(BaseMatrix<T> matrix) : matrix(matrix) { }

    BaseTransform(T a1, T a2, T a3, T b1, T b2, T b3) :
        matrix(a1, a2, b1, b2), translate(a3, b3) { }

    BaseTransform operator*(const BaseTransform& other) const {
      return { matrix * other.matrix, matrix * other.translate + translate };
    }

    BasePoint<T> operator*(const BasePoint<T>& point) const { return matrix * point + translate; }

    BaseTransform operator*(T scalar) const { return { matrix * scalar, translate * scalar }; }

    BaseTransform operator/(T scalar) const {
      if (scalar == 0)
        return { 0, 0, 0, 0, 0, 0 };
      return { matrix / scalar, translate / scalar };
    }

    BaseTransform inversed() const {
      BaseMatrix<T> inv_matrix = matrix.inversed();
      return { inv_matrix, inv_matrix * -translate };
    }

    bool isIdentity() const {
      return matrix.matrix[0][0] == 1 && matrix.matrix[0][1] == 0 && matrix.matrix[1][0] == 0 &&
             matrix.matrix[1][1] == 1 && translate.x == 0 && translate.y == 0;
    }

    static BaseTransform identity() { return { BaseMatrix<T>::identity() }; }

    static BaseTransform translation(T translate_x, T translate_y) {
      return { BaseMatrix<T>::identity(), { translate_x, translate_y } };
    }

    static BaseTransform translation(const BasePoint<T>& translate) {
      return { BaseMatrix<T>::identity(), translate };
    }

    static BaseTransform rotation(T angle) { return { BaseMatrix<T>::rotation(angle) }; }

    static BaseTransform rotation(T angle, const BasePoint<T>& center) {
      return translation(center) * rotation(angle) * translation(-center);
    }

    static BaseTransform scale(T scale_x, T scale_y) {
      return { BaseMatrix<T>::scale(scale_x, scale_y) };
    }

    static BaseTransform skewX(T skew) { return { BaseMatrix<T>::skewX(skew) }; }

    static BaseTransform skewY(T skew) { return { BaseMatrix<T>::skewY(skew) }; }
  };

  typedef BasePoint<int> IPoint;
  typedef BasePoint<float> Point;
  typedef BasePoint<double> DPoint;
  typedef BasePoint<int64_t> IPoint64;

  typedef BaseMatrix<int> IMatrix;
  typedef BaseMatrix<float> Matrix;
  typedef BaseMatrix<double> DMatrix;
  typedef BaseMatrix<int64_t> IMatrix64;

  typedef BaseTransform<int> ITransform;
  typedef BaseTransform<float> Transform;
  typedef BaseTransform<double> DTransform;
  typedef BaseTransform<int64_t> ITransform64;

  class IBounds {
  public:
    IBounds() = default;
    IBounds(int x, int y, int width, int height) : x_(x), y_(y), width_(width), height_(height) { }

    int x() const { return x_; }
    int y() const { return y_; }
    int width() const { return width_; }
    int height() const { return height_; }
    bool hasArea() const { return width_ > 0 && height_ > 0; }
    int right() const { return x_ + width_; }
    int bottom() const { return y_ + height_; }
    int xCenter() const { return x_ + width_ / 2; }
    int yCenter() const { return y_ + height_ / 2; }
    IPoint topLeft() const { return { x_, y_ }; }
    IPoint clampPoint(const IPoint& point) const {
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

    IBounds trimTop(int amount) {
      amount = std::min(amount, height_);
      const IBounds trimmed(x_, y_, width_, amount);
      y_ += amount;
      height_ -= amount;
      return trimmed;
    }

    IBounds trimBottom(int amount) {
      amount = std::min(amount, height_);
      const IBounds trimmed(x_, y_ + height_ - amount, width_, amount);
      height_ -= amount;
      return trimmed;
    }

    IBounds trimLeft(int amount) {
      amount = std::min(amount, width_);
      const IBounds trimmed(x_, y_, amount, height_);
      x_ += amount;
      width_ -= amount;
      return trimmed;
    }

    IBounds trimRight(int amount) {
      amount = std::min(amount, width_);
      const IBounds trimmed(x_ + width_ - amount, y_, amount, height_);
      width_ -= amount;
      return trimmed;
    }

    IBounds reduced(int amount) const {
      const auto newWidth = std::max(0, width_ - 2 * amount);
      const auto newHeight = std::max(0, height_ - 2 * amount);
      return { x_ + amount, y_ + amount, newWidth, newHeight };
    }

    IBounds reduced(int left, int right, int top, int bottom) const {
      const auto newWidth = std::max(0, width_ - left - right);
      const auto newHeight = std::max(0, height_ - top - bottom);
      return { x_ + left, y_ + top, newWidth, newHeight };
    }

    bool operator==(const IBounds& other) const {
      return x_ == other.x_ && y_ == other.y_ && width_ == other.width_ && height_ == other.height_;
    }

    bool operator!=(const IBounds& other) const { return !(*this == other); }
    bool contains(int x, int y) const { return x >= x_ && x < right() && y >= y_ && y < bottom(); }
    bool contains(const IPoint& point) const { return contains(point.x, point.y); }

    bool contains(const IBounds& other) const {
      return x_ <= other.x_ && y_ <= other.y_ && right() >= other.right() && bottom() >= other.bottom();
    }

    bool overlaps(const IBounds& other) const {
      return x_ < other.right() && right() > other.x_ && y_ < other.bottom() && bottom() > other.y_;
    }

    IBounds unioned(const IBounds& other) const {
      if (width() <= 0)
        return other;
      if (other.width() <= 0)
        return *this;

      auto x = std::min(x_, other.x_);
      auto y = std::min(y_, other.y_);
      auto r = std::max(right(), other.right());
      auto b = std::max(bottom(), other.bottom());
      return { x, y, r - x, b - y };
    }

    IBounds intersection(const IBounds& other) const {
      int x = std::max(x_, other.x_);
      int y = std::max(y_, other.y_);
      int r = std::min(right(), other.right());
      int b = std::min(bottom(), other.bottom());
      return { x, y, r - x, b - y };
    }

    // Returns true if subtracting the other rectangle result in only one rectangle
    // Stores that rectangle in _result_
    bool subtract(const IBounds& other, IBounds& result) const {
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

    IBounds operator+(const IPoint& point) const {
      return { x_ + point.x, y_ + point.y, width_, height_ };
    }

    // Create nonoverlapping rectangles that cover the same area as _rect1_ and _rect2_.
    // Input bounds are modified and additional rectangles needed are put into _pieces_.
    static void breakIntoNonOverlapping(IBounds& rect1, IBounds& rect2, std::vector<IBounds>& pieces) {
      if (!rect1.overlaps(rect2))
        return;

      IBounds subtraction;
      if (rect1.subtract(rect2, subtraction)) {
        rect1 = subtraction;
        return;
      }
      if (rect2.subtract(rect1, subtraction)) {
        rect2 = subtraction;
        return;
      }
      IBounds breaks[4];
      IBounds remaining = rect2;
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

  class Bounds {
  public:
    Bounds() = default;
    Bounds(float x, float y, float width, float height) :
        x_(x), y_(y), width_(width), height_(height) { }

    explicit Bounds(const IBounds& other) :
        x_(other.x()), y_(other.y()), width_(other.width()), height_(other.height()) { }

    IBounds round() const {
      return { static_cast<int>(std::round(x_)), static_cast<int>(std::round(y_)),
               static_cast<int>(std::round(width_)), static_cast<int>(std::round(height_)) };
    }

    float x() const { return x_; }
    float y() const { return y_; }
    float width() const { return width_; }
    float height() const { return height_; }
    bool hasArea() const { return width_ > 0.0f && height_ > 0.0f; }
    float right() const { return x_ + width_; }
    float bottom() const { return y_ + height_; }
    float xCenter() const { return x_ + width_ * 0.5f; }
    float yCenter() const { return y_ + height_ * 0.5f; }
    Point topLeft() const { return { x_, y_ }; }
    Point bottomRight() const { return { x_ + width_, y_ + height_ }; }
    Point clampPoint(const Point& point) const {
      return { std::max(x_, std::min(right(), point.x)), std::max(y_, std::min(bottom(), point.y)) };
    }

    void setX(float x) { x_ = x; }
    void setY(float y) { y_ = y; }
    void setWidth(float width) { width_ = width; }
    void setHeight(float height) { height_ = height; }
    void flipDimensions() {
      std::swap(x_, y_);
      std::swap(width_, height_);
    }

    Bounds trimTop(float amount) {
      amount = std::min(amount, height_);
      const Bounds trimmed(x_, y_, width_, amount);
      y_ += amount;
      height_ -= amount;
      return trimmed;
    }

    Bounds trimBottom(float amount) {
      amount = std::min(amount, height_);
      const Bounds trimmed(x_, y_ + height_ - amount, width_, amount);
      height_ -= amount;
      return trimmed;
    }

    Bounds trimLeft(float amount) {
      amount = std::min(amount, width_);
      const Bounds trimmed(x_, y_, amount, height_);
      x_ += amount;
      width_ -= amount;
      return trimmed;
    }

    Bounds trimRight(float amount) {
      amount = std::min(amount, width_);
      const Bounds trimmed(x_ + width_ - amount, y_, amount, height_);
      width_ -= amount;
      return trimmed;
    }

    Bounds reduced(float amount) const {
      const auto newWidth = std::max(0.0f, width_ - 2.0f * amount);
      const auto newHeight = std::max(0.0f, height_ - 2.0f * amount);
      return { x_ + amount, y_ + amount, newWidth, newHeight };
    }

    Bounds reduced(float left, float right, float top, float bottom) const {
      const auto newWidth = std::max(0.0f, width_ - left - right);
      const auto newHeight = std::max(0.0f, height_ - top - bottom);
      return { x_ + left, y_ + top, newWidth, newHeight };
    }

    bool operator==(const Bounds& other) const {
      return x_ == other.x_ && y_ == other.y_ && width_ == other.width_ && height_ == other.height_;
    }

    bool operator!=(const Bounds& other) const { return !(*this == other); }
    bool contains(float x, float y) const {
      return x >= x_ && x < right() && y >= y_ && y < bottom();
    }
    bool contains(const Point& point) const { return contains(point.x, point.y); }

    bool contains(const Bounds& other) const {
      return x_ <= other.x_ && y_ <= other.y_ && right() >= other.right() && bottom() >= other.bottom();
    }

    bool overlaps(const Bounds& other) const {
      return x_ < other.right() && right() > other.x_ && y_ < other.bottom() && bottom() > other.y_;
    }

    Bounds unioned(const Bounds& other) const {
      if (width() <= 0.0f)
        return other;
      if (other.width() <= 0.0f)
        return *this;

      float x = std::min(x_, other.x_);
      float y = std::min(y_, other.y_);
      float r = std::max(right(), other.right());
      float b = std::max(bottom(), other.bottom());
      return { x, y, r - x, b - y };
    }

    Bounds intersection(const Bounds& other) const {
      float x = std::max(x_, other.x_);
      float y = std::max(y_, other.y_);
      float r = std::min(right(), other.right());
      float b = std::min(bottom(), other.bottom());
      return { x, y, r - x, b - y };
    }

    friend Bounds operator*(float scalar, const Bounds& bounds) {
      return { bounds.x_ * scalar, bounds.y_ * scalar, bounds.width_ * scalar, bounds.height_ * scalar };
    }

    Bounds operator*(float scalar) const {
      return { x_ * scalar, y_ * scalar, width_ * scalar, height_ * scalar };
    }

    Bounds operator+(const Point& point) const {
      return { x_ + point.x, y_ + point.y, width_, height_ };
    }

  private:
    float x_ = 0.0f;
    float y_ = 0.0f;
    float width_ = 0.0f;
    float height_ = 0.0f;
  };

  static Point adjustBoundsForAspectRatio(Point current, Point min_bounds, Point max_bounds,
                                          float aspect_ratio, bool horizontal_resize,
                                          bool vertical_resize) {
    float min_x = std::max(min_bounds.x, min_bounds.y * aspect_ratio);
    float min_y = std::max(min_bounds.y, min_bounds.x / aspect_ratio);
    float max_x = std::min(max_bounds.x, max_bounds.y * aspect_ratio);
    float max_y = std::min(max_bounds.y, max_bounds.x / aspect_ratio);

    float width = std::max(min_x, std::min(max_x, current.x));
    float height = std::max(min_y, std::min(max_y, current.y));

    float width_from_height = std::max(min_x, std::min(max_x, height * aspect_ratio));
    float height_from_width = std::max(min_y, std::min(max_y, width / aspect_ratio));

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
