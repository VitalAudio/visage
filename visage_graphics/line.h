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

#include "graphics_utils.h"

namespace visage {
  struct LineIndexBuffers;

  struct Line {
    static constexpr int kLineVerticesPerPoint = 6;
    static constexpr int kFillVerticesPerPoint = 2;

    explicit Line(int num);
    ~Line();

    void init() const;
    void destroy() const;

    const bgfx::IndexBufferHandle& indexBuffer() const;
    const bgfx::IndexBufferHandle& fillIndexBuffer() const;

    int num_points = 0;
    int num_line_vertices = 0;
    int num_fill_vertices = 0;

    std::unique_ptr<float[]> x;
    std::unique_ptr<float[]> y;
    std::unique_ptr<float[]> values;

    float line_value_scale = 1.0f;
    float fill_value_scale = 1.0f;
    std::unique_ptr<uint16_t[]> index_data;
    std::unique_ptr<uint16_t[]> fill_index_data;

  private:
    std::unique_ptr<LineIndexBuffers> index_buffers_;
  };
}
