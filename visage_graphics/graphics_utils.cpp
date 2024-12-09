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

#include "graphics_utils.h"

#include <bgfx/bgfx.h>
#include <vector>

#define STB_RECT_PACK_IMPLEMENTATION
#include <stb/stb_rect_pack.h>

namespace visage {
  struct PackedAtlasData {
    stbrp_context pack_context {};
    std::unique_ptr<stbrp_node[]> pack_nodes;
  };

  AtlasPacker::AtlasPacker() : data_(std::make_unique<PackedAtlasData>()) { }

  AtlasPacker::~AtlasPacker() = default;

  bool AtlasPacker::addRect(PackedRect& rect) {
    if (!packed_)
      return false;

    stbrp_rect r;
    r.w = rect.w + padding_;
    r.h = rect.h + padding_;
    r.id = rect_index_++;

    packed_ = stbrp_pack_rects(&data_->pack_context, &r, 1);
    if (packed_) {
      rect.x = r.x;
      rect.y = r.y;
    }
    return packed_;
  }

  void AtlasPacker::clear() {
    packed_ = false;
  }

  bool AtlasPacker::pack(std::vector<PackedRect>& rects, int width) {
    data_->pack_nodes = std::make_unique<stbrp_node[]>(width);
    std::vector<stbrp_rect> packed_rects;
    rect_index_ = 0;
    for (const PackedRect& rect : rects) {
      stbrp_rect r;
      r.w = rect.w + padding_;
      r.h = rect.h + padding_;
      r.id = rect_index_++;
      packed_rects.push_back(r);
    }

    stbrp_init_target(&data_->pack_context, width, width, data_->pack_nodes.get(), width);
    packed_ = stbrp_pack_rects(&data_->pack_context, packed_rects.data(), packed_rects.size());
    if (packed_) {
      for (int i = 0; i < packed_rects.size(); ++i) {
        rects[i].x = packed_rects[i].x;
        rects[i].y = packed_rects[i].y;
      }
    }
    return packed_;
  }

  bgfx::VertexLayout& UvVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& LineVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& ShapeVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color1, 1, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord2, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& ComplexShapeVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color1, 1, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord2, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord3, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& TextureVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color1, 1, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& PostEffectVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color1, 1, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  bgfx::VertexLayout& RotaryVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color1, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color2, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color3, 3, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord2, 4, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }
}