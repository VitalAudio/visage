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

namespace visage {
  struct Line {
    static constexpr int kLineVerticesPerPoint = 6;
    static constexpr int kFillVerticesPerPoint = 2;

    explicit Line(int points = 0) { setNumPoints(points); }

    void setNumPoints(int points) {
      num_points = points;

      num_line_vertices = kLineVerticesPerPoint * num_points;
      num_fill_vertices = kFillVerticesPerPoint * num_points;
      x.resize(num_points, 0.0f);
      y.resize(num_points, 0.0f);
      values.resize(num_points, 0.0f);
    }

    int num_points = 0;
    int num_line_vertices = 0;
    int num_fill_vertices = 0;

    std::vector<float> x {};
    std::vector<float> y {};
    std::vector<float> values {};

    float line_value_scale = 1.0f;
    float fill_value_scale = 1.0f;
  };
}
