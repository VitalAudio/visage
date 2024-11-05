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

#include <cstring>
#include <memory>

namespace visage {
  struct Line {
    static constexpr int kLineVerticesPerPoint = 6;
    static constexpr int kFillVerticesPerPoint = 2;

    explicit Line(int points = 0) { setNumPoints(points); }

    void setNumPoints(int points) {
      num_line_vertices = kLineVerticesPerPoint * points;
      num_fill_vertices = kFillVerticesPerPoint * points;

      std::unique_ptr<float[]> old_x = std::move(x);
      std::unique_ptr<float[]> old_y = std::move(y);
      std::unique_ptr<float[]> old_values = std::move(values);
      if (points) {
        x = std::make_unique<float[]>(points);
        y = std::make_unique<float[]>(points);
        values = std::make_unique<float[]>(points);

        if (num_points) {
          std::memcpy(x.get(), old_x.get(), num_points * sizeof(float));
          std::memcpy(y.get(), old_y.get(), num_points * sizeof(float));
          std::memcpy(values.get(), old_values.get(), num_points * sizeof(float));
        }
      }

      num_points = points;
    }

    int num_points = 0;
    int num_line_vertices = 0;
    int num_fill_vertices = 0;

    std::unique_ptr<float[]> x;
    std::unique_ptr<float[]> y;
    std::unique_ptr<float[]> values;

    float line_value_scale = 1.0f;
    float fill_value_scale = 1.0f;
  };
}
