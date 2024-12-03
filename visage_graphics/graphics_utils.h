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

#include <map>
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
  enum class BlendMode {
    Opaque,
    Alpha,
    Additive,
    Multiply
  };

  static constexpr float kHdrColorRange = 4.0f;
  static constexpr float kHdrColorMultiplier = 1.0f / kHdrColorRange;
  static constexpr int kVerticesPerQuad = 4;
  static constexpr int kIndicesPerQuad = 6;

  const uint16_t kQuadTriangles[] = {
    0, 1, 2, 2, 1, 3,
  };

  struct PackedAtlasData;

  struct PackedRect {
    int x;
    int y;
    int w;
    int h;
  };

  class AtlasPacker {
  public:
    AtlasPacker();
    ~AtlasPacker();

    bool addRect(PackedRect& rect);
    void clear();
    bool pack(std::vector<PackedRect>& rects, int width);
    void setPadding(int padding) { padding_ = padding; }

    bool packed() const { return packed_; }

  private:
    std::unique_ptr<PackedAtlasData> data_;
    bool packed_ = false;
    int padding_ = 0;
    int rect_index_ = 0;
  };

  template<typename T = int>
  class PackedAtlas {
  public:
    bool addRect(T id, int width, int height) {
      VISAGE_ASSERT(lookup_.count(id) == 0);

      int index = packed_rects_.size();
      lookup_[id] = index;
      packed_rects_.push_back({ 0, 0, width, height });
      return packer_.addRect(packed_rects_.back());
    }

    void removeRect(T id) {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      int index = lookup_.at(id);
      lookup_.erase(id);
    }

    void pack() {
      static constexpr int kDefaultWidth = 256;
      static constexpr int kMaxMultiples = 6;

      checkRemovedRects();
      bool packed = false;
      for (int m = 0; !packed && m < kMaxMultiples; ++m) {
        width_ = kDefaultWidth << m;
        packed = packer_.pack(packed_rects_, width_);
      }

      VISAGE_ASSERT(packed);
    }

    void clear() {
      lookup_.clear();
      packer_.clear();
      packed_rects_.clear();
    }

    void setPadding(int padding) { packer_.setPadding(padding); }

    const PackedRect& rectAtIndex(int index) const {
      VISAGE_ASSERT(index >= 0 && index < packed_rects_.size());
      return packed_rects_[index];
    }

    template<typename V>
    void setTexturePositionsForIndex(int rect_index, V* vertices) const {
      const PackedRect& packed_rect = rectAtIndex(rect_index);

      vertices[0].texture_x = packed_rect.x;
      vertices[0].texture_y = packed_rect.y;
      vertices[1].texture_x = packed_rect.x + packed_rect.w;
      vertices[1].texture_y = packed_rect.y;
      vertices[2].texture_x = packed_rect.x;
      vertices[2].texture_y = packed_rect.y + packed_rect.h;
      vertices[3].texture_x = packed_rect.x + packed_rect.w;
      vertices[3].texture_y = packed_rect.y + packed_rect.h;
    }

    const PackedRect& rectForId(T id) const {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      return rectAtIndex(lookup_.at(id));
    }

    template<typename V>
    void setTexturePositionsForId(T id, V* vertices) const {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      setTexturePositionsForIndex(lookup_.at(id), vertices);
    }

    int width() const { return width_; }
    bool packed() const { return packer_.packed(); }
    int numRects() const { return packed_rects_.size(); }

  private:
    void checkRemovedRects() {
      if (packed_rects_.size() == lookup_.size())
        return;

      std::vector<PackedRect> old_rects = std::move(packed_rects_);
      packed_rects_.reserve(lookup_.size());
      for (auto& packed : lookup_) {
        int index = packed_rects_.size();
        packed_rects_.push_back(old_rects[packed.second]);
        packed.second = index;
      }
    }

    int width_ = 0;
    int padding_ = 0;
    std::vector<PackedRect> packed_rects_;
    AtlasPacker packer_;
    std::map<T, int> lookup_;
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
    float thickness;
    float fade;
    float value_1;
    float value_2;

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
    float thickness;
    float fade;
    float value_1;
    float value_2;
    float value_3;
    float value_4;
    float value_5;
    float value_6;

    static bgfx::VertexLayout& layout();
  };

  struct TextureVertex {
    float x;
    float y;
    float dimension_x;
    float dimension_y;
    float texture_x;
    float texture_y;
    float direction_x;
    float direction_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    uint32_t color;
    float hdr;

    static bgfx::VertexLayout& layout();
  };

  struct PostEffectVertex {
    float x;
    float y;
    float dimension_x;
    float dimension_y;
    float texture_x;
    float texture_y;
    float uv_x;
    float uv_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    uint32_t color;
    float hdr;

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