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
    std::function<float(float dpi_scale, float parent_width, float parent_height)> compute = nullptr;

    int computeWithDefault(float dpi_scale, float parent_width, float parent_height,
                           float default_value = 0.0f) const {
      if (compute)
        return std::round(compute(dpi_scale, parent_width, parent_height));
      return default_value;
    }

    Dimension() = default;
    Dimension(float amount) { *this = devicePixels(amount); }
    Dimension(std::function<float(float, float, float)> compute) : compute(std::move(compute)) { }

    static Dimension devicePixels(float pixels) {
      return Dimension([pixels](float, float, float) { return pixels; });
    }

    static Dimension logicalPixels(float pixels) {
      return Dimension([pixels](float dpi_scale, float, float) { return dpi_scale * pixels; });
    }

    static Dimension widthPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float parent_width, float) { return ratio * parent_width; });
    }

    static Dimension heightPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float, float parent_height) { return ratio * parent_height; });
    }

    static Dimension viewMinPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float parent_width, float parent_height) {
        return ratio * std::min(parent_width, parent_height);
      });
    }

    static Dimension viewMaxPercent(float percent) {
      float ratio = percent * 0.01f;
      return Dimension([ratio](float, float parent_width, float parent_height) {
        return ratio * std::max(parent_width, parent_height);
      });
    }

    Dimension operator+(const Dimension& other) const {
      auto compute1 = compute;
      auto compute2 = other.compute;
      return Dimension([compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) +
               compute2(dpi_scale, parent_width, parent_height);
      });
    }

    Dimension& operator+=(const Dimension& other) {
      auto compute1 = compute;
      auto compute2 = other.compute;
      compute = [compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) +
               compute2(dpi_scale, parent_width, parent_height);
      };
      return *this;
    }

    Dimension operator-(const Dimension& other) const {
      auto compute1 = compute;
      auto compute2 = other.compute;
      return Dimension([compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) -
               compute2(dpi_scale, parent_width, parent_height);
      });
    }

    Dimension& operator-=(const Dimension& other) {
      auto compute1 = compute;
      auto compute2 = other.compute;
      compute = [compute1, compute2](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) -
               compute2(dpi_scale, parent_width, parent_height);
      };
      return *this;
    }

    Dimension operator*(float scalar) const {
      auto compute1 = compute;
      return Dimension([compute1, scalar](float dpi_scale, float parent_width, float parent_height) {
        return scalar * compute1(dpi_scale, parent_width, parent_height);
      });
    }

    friend Dimension operator*(float scalar, const Dimension& dimension) {
      return dimension * scalar;
    }

    Dimension operator/(float scalar) const {
      auto compute1 = compute;
      return Dimension([compute1, scalar](float dpi_scale, float parent_width, float parent_height) {
        return compute1(dpi_scale, parent_width, parent_height) / scalar;
      });
    }
  };

  namespace dimension {
    inline Dimension operator""_dpx(long double pixels) {
      return Dimension::devicePixels(pixels);
    }

    inline Dimension operator""_dpx(unsigned long long pixels) {
      return Dimension::devicePixels(pixels);
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