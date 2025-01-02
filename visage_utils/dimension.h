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