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

#include <cmath>
#include <functional>

namespace visage {

  struct Dimension {
    float amount = 0.0f;
    std::function<float(float, float, float, float)> compute_function = nullptr;

    float compute(float dpi_scale, float parent_width, float parent_height, float default_value = 0.0f) const {
      if (compute_function)
        return compute_function(amount, dpi_scale, parent_width, parent_height);
      return default_value;
    }

    int computeInt(float dpi_scale, float parent_width, float parent_height, int default_value = 0) const {
      if (compute_function)
        return std::round(compute_function(amount, dpi_scale, parent_width, parent_height));
      return default_value;
    }

    Dimension() = default;
    Dimension(float amount) { *this = logicalPixels(amount); }
    Dimension(float amount, std::function<float(float, float, float, float)> compute) :
        amount(amount), compute_function(std::move(compute)) { }

    static Dimension nativePixels(float pixels) {
      return Dimension(pixels, [](float amount, float, float, float) { return amount; });
    }

    static Dimension logicalPixels(float pixels) {
      return Dimension(pixels, [](float amount, float scale, float, float) { return scale * amount; });
    }

    static Dimension widthPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension(ratio, [](float amount, float, float parent_width, float) {
        return amount * parent_width;
      });
    }

    static Dimension heightPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension(ratio, [](float amount, float, float, float parent_height) {
        return amount * parent_height;
      });
    }

    static Dimension viewMinPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension(ratio, [](float amount, float, float parent_width, float parent_height) {
        return amount * std::min(parent_width, parent_height);
      });
    }

    static Dimension viewMaxPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension(ratio, [](float amount, float, float parent_width, float parent_height) {
        return amount * std::max(parent_width, parent_height);
      });
    }

    static Dimension min(const Dimension& a, const Dimension& b) {
      if (a.compute_function == nullptr)
        return b;
      if (b.compute_function == nullptr)
        return a;

      auto amount1 = a.amount;
      auto amount2 = b.amount;
      auto compute1 = a.compute_function;
      auto compute2 = b.compute_function;
      return Dimension(0.0f, [amount1, amount2, compute1,
                              compute2](float, float dpi_scale, float parent_width, float parent_height) {
        return std::min(compute1(amount1, dpi_scale, parent_width, parent_height),
                        compute2(amount2, dpi_scale, parent_width, parent_height));
      });
    }

    static Dimension max(const Dimension& a, const Dimension& b) {
      if (a.compute_function == nullptr)
        return b;
      if (b.compute_function == nullptr)
        return a;

      auto amount1 = a.amount;
      auto amount2 = b.amount;
      auto compute1 = a.compute_function;
      auto compute2 = b.compute_function;
      return Dimension(0.0f, [amount1, amount2, compute1,
                              compute2](float, float dpi_scale, float parent_width, float parent_height) {
        return std::max(compute1(amount1, dpi_scale, parent_width, parent_height),
                        compute2(amount2, dpi_scale, parent_width, parent_height));
      });
    }

    Dimension operator+(const Dimension& other) const {
      if (compute_function == nullptr)
        return other;
      if (other.compute_function == nullptr)
        return *this;

      auto amount1 = amount;
      auto amount2 = other.amount;
      auto compute1 = compute_function;
      auto compute2 = other.compute_function;
      return Dimension(0.0f, [amount1, amount2, compute1,
                              compute2](float, float dpi_scale, float parent_width, float parent_height) {
        return compute1(amount1, dpi_scale, parent_width, parent_height) +
               compute2(amount2, dpi_scale, parent_width, parent_height);
      });
    }

    Dimension& operator+=(const Dimension& other) {
      *this = *this + other;
      return *this;
    }

    Dimension operator-(const Dimension& other) const {
      if (compute_function == nullptr)
        return -other;
      if (other.compute_function == nullptr)
        return *this;

      auto amount1 = amount;
      auto amount2 = other.amount;
      auto compute1 = compute_function;
      auto compute2 = other.compute_function;
      return Dimension(0.0f, [amount1, amount2, compute1,
                              compute2](float, float dpi_scale, float parent_width, float parent_height) {
        return compute1(amount1, dpi_scale, parent_width, parent_height) -
               compute2(amount2, dpi_scale, parent_width, parent_height);
      });
    }

    Dimension& operator-=(const Dimension& other) {
      *this = *this - other;
      return *this;
    }

    Dimension operator*(float scalar) const {
      if (compute_function == nullptr)
        return *this;

      auto amount1 = amount;
      auto compute1 = compute_function;
      return Dimension(amount1, [compute1, scalar](float amount, float dpi_scale,
                                                   float parent_width, float parent_height) {
        return scalar * compute1(amount, dpi_scale, parent_width, parent_height);
      });
    }

    friend Dimension operator*(float scalar, const Dimension& dimension) {
      return dimension * scalar;
    }

    Dimension operator/(float scalar) const { return *this * (1.0f / scalar); }

    Dimension operator-() const { return *this * -1.0f; }

    Dimension min(const Dimension& other) const { return min(*this, other); }

    Dimension max(const Dimension& other) const { return max(*this, other); }
  };

  namespace dimension {
    inline Dimension operator""_npx(long double pixels) {
      return Dimension::nativePixels(pixels);
    }

    inline Dimension operator""_npx(unsigned long long pixels) {
      return Dimension::nativePixels(pixels);
    }

    inline Dimension operator""_px(long double pixels) {
      return Dimension::logicalPixels(pixels);
    }

    inline Dimension operator""_px(unsigned long long pixels) {
      return Dimension::logicalPixels(pixels);
    }

    inline Dimension operator""_vw(long double percent) {
      return Dimension::widthPercent(percent);
    }

    inline Dimension operator""_vw(unsigned long long percent) {
      return Dimension::widthPercent(percent);
    }

    inline Dimension operator""_vh(long double percent) {
      return Dimension::heightPercent(percent);
    }

    inline Dimension operator""_vh(unsigned long long percent) {
      return Dimension::heightPercent(percent);
    }

    inline Dimension operator""_vmin(long double percent) {
      return Dimension::viewMinPercent(percent);
    }

    inline Dimension operator""_vmin(unsigned long long percent) {
      return Dimension::viewMinPercent(percent);
    }

    inline Dimension operator""_vmax(long double percent) {
      return Dimension::viewMaxPercent(percent);
    }

    inline Dimension operator""_vmax(unsigned long long percent) {
      return Dimension::viewMaxPercent(percent);
    }
  }
}