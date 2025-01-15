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
    Add,
    Sub,
    Mult,
    StencilAdd,
    StencilRemove,
  };

  static constexpr float kHdrColorRange = 4.0f;
  static constexpr float kHdrColorMultiplier = 1.0f / kHdrColorRange;
  static constexpr int kVerticesPerQuad = 4;
  static constexpr int kIndicesPerQuad = 6;

  bool preprocessWebGlShader(std::string& result, const std::string& code,
                             const std::string& utils_source, const std::string& varying_source);

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
    void setTexturePositionsForIndex(int rect_index, V* vertices, bool bottom_left_origin = false) const {
      const PackedRect& packed_rect = rectAtIndex(rect_index);

      vertices[0].texture_x = packed_rect.x;
      vertices[0].texture_y = packed_rect.y;
      vertices[1].texture_x = packed_rect.x + packed_rect.w;
      vertices[1].texture_y = packed_rect.y;
      vertices[2].texture_x = packed_rect.x;
      vertices[2].texture_y = packed_rect.y + packed_rect.h;
      vertices[3].texture_x = packed_rect.x + packed_rect.w;
      vertices[3].texture_y = packed_rect.y + packed_rect.h;

      if (bottom_left_origin) {
        for (int i = 0; i < kVerticesPerQuad; ++i)
          vertices[i].texture_y = width_ - vertices[i].texture_y;
      }
    }

    const PackedRect& rectForId(T id) const {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      return rectAtIndex(lookup_.at(id));
    }

    template<typename V>
    void setTexturePositionsForId(T id, V* vertices, bool bottom_left_origin = false) const {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      setTexturePositionsForIndex(lookup_.at(id), vertices, bottom_left_origin);
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