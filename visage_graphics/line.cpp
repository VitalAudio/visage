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

#include "line.h"

#include "graphics_libs.h"

namespace visage {
  struct LineIndexBuffers {
    bgfx::IndexBufferHandle line = { bgfx::kInvalidHandle };
    bgfx::IndexBufferHandle fill = { bgfx::kInvalidHandle };

    ~LineIndexBuffers() { destroy(); }

    void destroy() const {
      if (bgfx::isValid(line))
        bgfx::destroy(line);
      if (bgfx::isValid(fill))
        bgfx::destroy(fill);
    }
  };

  Line::Line(int num) {
    index_buffers_ = std::make_unique<LineIndexBuffers>();

    num_points = num;
    num_line_vertices = kLineVerticesPerPoint * num_points;
    num_fill_vertices = kFillVerticesPerPoint * num_points;

    x = std::make_unique<float[]>(num_points);
    y = std::make_unique<float[]>(num_points);
    values = std::make_unique<float[]>(num_points);

    index_data = std::make_unique<uint16_t[]>(num_line_vertices);
    for (int i = 0; i < num_line_vertices; ++i)
      index_data[i] = i;

    fill_index_data = std::make_unique<uint16_t[]>(num_fill_vertices);
    for (int i = 0; i < num_fill_vertices; ++i)
      fill_index_data[i] = i;
  }

  Line::~Line() = default;

  void Line::init() const {
    index_buffers_->line = bgfx::createIndexBuffer(bgfx::makeRef(index_data.get(),
                                                                 num_line_vertices * sizeof(uint16_t)));
    index_buffers_->fill = bgfx::createIndexBuffer(bgfx::makeRef(fill_index_data.get(),
                                                                 num_fill_vertices * sizeof(uint16_t)));
  }

  void Line::destroy() const {
    index_buffers_->destroy();
  }

  const bgfx::IndexBufferHandle& Line::indexBuffer() const {
    return index_buffers_->line;
  }

  const bgfx::IndexBufferHandle& Line::fillIndexBuffer() const {
    return index_buffers_->fill;
  }
}
