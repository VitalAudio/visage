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

#include "visage_utils/defines.h"

#include <memory>
#include <vector>

namespace bgfx {
  struct VertexLayout;
  struct TextureHandle;
  struct ShaderHandle;
  struct ProgramHandle;
  struct UniformHandle;
  struct IndexBufferHandle;
  struct FrameBufferHandle;
  struct TransientIndexBuffer;
  struct TransientVertexBuffer;
}

namespace visage {
  struct PackedAtlasData;

  enum class BlendState {
    Opaque,
    Clear,
    Alpha,
    Additive
  };

  static constexpr float kHdrColorRange = 4.0f;
  static constexpr float kHdrColorMultiplier = 1.0f / kHdrColorRange;
  static constexpr int kVerticesPerQuad = 4;
  static constexpr int kIndicesPerQuad = 6;

  const uint16_t kQuadTriangles[] = {
    0, 1, 2, 2, 1, 3,
  };

  class PackedAtlas {
  public:
    struct Rect {
      int x;
      int y;
      int w;
      int h;
    };

    PackedAtlas();
    ~PackedAtlas();

    int width() const { return width_; }
    void setPadding(int padding) { padding_ = padding; }
    bool packed() const { return packed_; }
    const Rect& getRect(int index) const {
      VISAGE_ASSERT(index >= 0 && index < packed_rects_.size());
      return packed_rects_[index];
    }
    int numRects() const;
    bool addPackedRect(int id, int width, int height);
    void pack();
    void clear();

  private:
    void loadPackedRects();
    void loadLastPackedRect();

    bool packed_ = false;
    int width_ = 0;
    int padding_ = 0;
    std::vector<Rect> packed_rects_;
    std::unique_ptr<PackedAtlasData> data_;
  };

  struct BasicVertex {
    float x;
    float y;

    static bgfx::VertexLayout& layout();
  };

  struct UvVertex {
    float x;
    float y;
    float u;
    float v;

    static bgfx::VertexLayout& layout();
  };

  struct LineVertex {
    float x;
    float y;
    float fill;
    float value;

    static bgfx::VertexLayout& layout();
  };

  struct ShapeVertex {
    float x;
    float y;
    float coordinate_x;
    float coordinate_y;
    float dimension_x;
    float dimension_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    uint32_t color;
    float hdr;
    float value_1;
    float value_2;
    float value_3;
    float value_4;

    static bgfx::VertexLayout& layout();
  };

  struct ComplexShapeVertex {
    float x;
    float y;
    float coordinate_x;
    float coordinate_y;
    float dimension_x;
    float dimension_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    uint32_t color;
    float hdr;
    float value_1;
    float value_2;
    float value_3;
    float value_4;
    float value_5;
    float value_6;
    float value_7;
    float value_8;

    static bgfx::VertexLayout& layout();
  };

  struct RotaryVertex {
    float x;
    float y;
    float coordinate_x;
    float coordinate_y;
    float dimension_x;
    float dimension_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    uint32_t color;
    uint32_t back_color;
    uint32_t thumb_color;
    float hdr;
    float back_hdr;
    float thumb_hdr;
    float value_1;
    float value_2;
    float value_3;
    float value_4;

    static bgfx::VertexLayout& layout();
  };
}