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

#include "graphics_libs.h"
#include "visage_utils/defines.h"

#include <vector>

namespace visage {
  struct PackedAtlasData {
    stbrp_context pack_context {};
    std::vector<stbrp_rect> packed_rects;
    std::unique_ptr<stbrp_node[]> pack_nodes;
  };

  PackedAtlas::PackedAtlas() : data_(std::make_unique<PackedAtlasData>()) { }

  PackedAtlas::~PackedAtlas() = default;

  inline int PackedAtlas::numRects() const {
    return data_->packed_rects.size();
  }

  void PackedAtlas::loadPackedRects() {
    packed_rects_.clear();
    packed_rects_.reserve(data_->packed_rects.size());
    for (const stbrp_rect& rect : data_->packed_rects)
      packed_rects_.push_back({ rect.x, rect.y, rect.w, rect.h });
  }

  void PackedAtlas::loadLastPackedRect() {
    packed_rects_.push_back({ data_->packed_rects.back().x, data_->packed_rects.back().y,
                              data_->packed_rects.back().w, data_->packed_rects.back().h });
  }

  bool PackedAtlas::addPackedRect(int id, int width, int height) {
    stbrp_rect rect;
    rect.id = id;
    rect.w = width + padding_;
    rect.h = height + padding_;
    data_->packed_rects.push_back(rect);

    if (packed_) {
      packed_ = stbrp_pack_rects(&data_->pack_context, data_->packed_rects.data() + rect.id, 1);
      loadLastPackedRect();
    }
    return packed_;
  }

  void PackedAtlas::pack() {
    static constexpr int kDefaultWidth = 256;
    static constexpr int kMaxMultiples = 6;

    packed_ = false;
    for (int m = 0; !packed_ && m < kMaxMultiples; ++m) {
      width_ = kDefaultWidth << m;
      data_->pack_nodes = std::make_unique<stbrp_node[]>(width_);

      stbrp_init_target(&data_->pack_context, width_, width_, data_->pack_nodes.get(), width_);
      packed_ = stbrp_pack_rects(&data_->pack_context, data_->packed_rects.data(),
                                 data_->packed_rects.size());
    }

    if (packed_)
      loadPackedRects();

    VISAGE_ASSERT(packed_);
  }

  void PackedAtlas::clear() {
    data_->packed_rects.clear();
    packed_rects_.clear();
  }

  bgfx::VertexLayout& BasicVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin().add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float).end();
    }

    return layout;
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