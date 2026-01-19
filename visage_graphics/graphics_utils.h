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

#include <cstdint>
#include <map>
#include <memory>
#include <string>
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
    Composite,
    Alpha,
    Add,
    Sub,
    Mult,
    MaskAdd,
    MaskRemove,
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

  struct TextureRect {
    int left;
    int top;
    int right;
    int bottom;
  };

  class AtlasPacker {
  public:
    AtlasPacker();
    ~AtlasPacker();

    bool addRect(PackedRect& rect);
    void clear();
    bool pack(std::vector<PackedRect>& rects, int width, int height);
    void setPadding(int padding) { padding_ = padding; }
    int padding() const { return padding_; }

    bool packed() const { return packed_; }

  private:
    std::unique_ptr<PackedAtlasData> data_;
    bool packed_ = false;
    int padding_ = 1;
    int rect_index_ = 0;
  };

  template<typename T = int>
  class PackedAtlasMap {
  public:
    static constexpr int kDefaultWidth = 64;

    PackedAtlasMap() = default;

    bool addRect(T id, int width, int height) {
      VISAGE_ASSERT(lookup_.count(id) == 0);

      int index = packed_rects_.size();
      lookup_[id] = index;
      packed_rects_.push_back({ 0, 0, std::max(0, width), std::max(0, height) });
      return packer_.addRect(packed_rects_.back());
    }

    bool hasId(T id) const { return lookup_.count(id) > 0; }

    void removeRect(T id) {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      lookup_.erase(id);
    }

    void pack(int start_width = kDefaultWidth, int start_height = kDefaultWidth) {
      static constexpr int kMaxDimension = 1 << 14;

      checkRemovedRects();
      if (packed_rects_.size() == 1) {
        width_ = std::max(1, packed_rects_[0].w + packer_.padding());
        height_ = std::max(1, packed_rects_[0].h + packer_.padding());
        if (!packer_.pack(packed_rects_, width_, height_))
          VISAGE_ASSERT(false);
      }
      else if (!packed_rects_.empty()) {
        width_ = std::max(kDefaultWidth, start_width);
        height_ = std::max(kDefaultWidth, start_height);

        while (width_ < kMaxDimension * 2 || height_ < kMaxDimension * 2) {
          width_ = fixed_width_ ? fixed_width_ : std::min(kMaxDimension, width_);
          height_ = std::min(kMaxDimension, height_);
          if (packer_.pack(packed_rects_, width_, height_))
            return;

          width_ *= 2;
          height_ *= 2;
        }
        VISAGE_ASSERT(false);
      }
    }

    void clear() {
      lookup_.clear();
      packer_.clear();
      packed_rects_.clear();
    }

    void setPadding(int padding) { packer_.setPadding(padding); }
    int padding() const { return packer_.padding(); }

    const PackedRect& rectAtIndex(int index) const {
      VISAGE_ASSERT(index >= 0 && index < packed_rects_.size());
      return packed_rects_[index];
    }

    TextureRect texturePositionsForIndex(int rect_index, bool bottom_left_origin = false) const {
      const PackedRect& packed_rect = rectAtIndex(rect_index);
      TextureRect result = { packed_rect.x, packed_rect.y, packed_rect.x + packed_rect.w,
                             packed_rect.y + packed_rect.h };

      if (bottom_left_origin) {
        result.top = height_ - result.top;
        result.bottom = height_ - result.bottom;
      }
      return result;
    }

    const PackedRect& rectForId(T id) const {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      return rectAtIndex(lookup_.at(id));
    }

    TextureRect texturePositionsForId(T id, bool bottom_left_origin = false) const {
      VISAGE_ASSERT(lookup_.count(id) > 0);
      return texturePositionsForIndex(lookup_.at(id), bottom_left_origin);
    }

    void fixWidth(int width) { fixed_width_ = width; }
    int width() const { return width_; }
    int height() const { return height_; }
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

    int fixed_width_ = 0;
    int width_ = 0;
    int height_ = 0;
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

  struct PathVertex {
    float index;
    float direction;
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;

    static bgfx::VertexLayout& layout();
  };

  struct GradientTexturePosition {
    float from_x;
    float from_y;
    float to_x;
    float to_y;
  };

  struct GradientVertexPosition {
    float from_x;
    float from_y;
    float to_x;
    float to_y;
    float coefficient1;
    float coefficient2;
    float coefficient3;
    float cone_height;

    float* position1() { return &from_x; }
    float* position2() { return &coefficient1; }
  };

  struct ShapeVertex {
    float x;
    float y;
    float garbage1;
    float garbage2;
    GradientTexturePosition gradient_texture_position;
    GradientVertexPosition gradient;
    float coordinate_x;
    float coordinate_y;
    float dimension_x;
    float dimension_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    float thickness;
    float fade;
    float value1;
    float value2;

    static bgfx::VertexLayout& layout();
  };

  struct ComplexShapeVertex {
    float x;
    float y;
    float garbage1;
    float garbage2;
    GradientTexturePosition gradient_texture_position;
    GradientVertexPosition gradient;
    float coordinate_x;
    float coordinate_y;
    float dimension_x;
    float dimension_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    float thickness;
    float fade;
    float value1;
    float value2;
    float value3;
    float value4;
    float value5;
    float value6;

    static bgfx::VertexLayout& layout();
  };

  struct TextureVertex {
    float x;
    float y;
    float dimension_x;
    float dimension_y;
    GradientTexturePosition gradient_texture_position;
    GradientVertexPosition gradient;
    float texture_x;
    float texture_y;
    float direction_x;
    float direction_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;

    static bgfx::VertexLayout& layout();
  };

  struct PostEffectVertex {
    float x;
    float y;
    float dimension_x;
    float dimension_y;
    GradientTexturePosition gradient_texture_position;
    GradientVertexPosition gradient;
    float texture_x;
    float texture_y;
    float value1;
    float value2;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;

    static bgfx::VertexLayout& layout();
  };
}
