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
