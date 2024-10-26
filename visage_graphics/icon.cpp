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

#include "icon.h"

#include "graphics_libs.h"

#include <cstring>
#include <nanosvg/src/nanosvg.h>
#include <nanosvg/src/nanosvgrast.h>

namespace visage {
  static void boxBlur(unsigned char* dest, unsigned char* cache, int width, int blur_radius, int stride) {
    int value = 0;
    int sample_index = 0;
    int write_index = 0;
    for (; sample_index < blur_radius / 2; ++sample_index) {
      cache[sample_index] = dest[sample_index * stride];
      value += cache[sample_index];
    }

    for (; sample_index < blur_radius; ++sample_index) {
      cache[sample_index] = dest[sample_index * stride];
      value += cache[sample_index];
      dest[write_index * stride] = value / blur_radius;
      write_index++;
    }

    for (; sample_index < width; ++sample_index) {
      int cache_index = sample_index % blur_radius;
      int cached = cache[cache_index];
      int sample = dest[sample_index * stride];
      cache[cache_index] = sample;
      value += sample - cached;
      dest[write_index * stride] = value / blur_radius;
      write_index++;
    }

    for (; sample_index < width + blur_radius; ++sample_index) {
      value -= cache[sample_index % blur_radius];
      dest[write_index * stride] = value / blur_radius;
      write_index++;
    }
  }

  class Rasterizer {
  public:
    static Rasterizer& instance() {
      static Rasterizer instance;
      return instance;
    }

    ~Rasterizer() { nsvgDeleteRasterizer(rasterizer_); }

    std::unique_ptr<unsigned char[]> rasterize(const Icon& icon) const {
      std::unique_ptr<char[]> copy = std::make_unique<char[]>(icon.svg_size + 1);
      memcpy(copy.get(), icon.svg, icon.svg_size);

      NSVGimage* image = nsvgParse(copy.get(), "px", 96);
      std::unique_ptr<unsigned char[]> data = std::make_unique<unsigned char[]>(icon.width *
                                                                                icon.height * 4);

      float width_scale = icon.width / image->width;
      float height_scale = icon.height / image->height;
      float scale = std::min(width_scale, height_scale);
      float x_offset = (icon.width - image->width * scale) * 0.5f;
      float y_offset = (icon.height - image->height * scale) * 0.5f;

      nsvgRasterize(rasterizer_, image, x_offset, y_offset, scale, data.get(), icon.width,
                    icon.height, icon.width * 4);
      nsvgDelete(image);
      return data;
    }

  private:
    Rasterizer() : rasterizer_(nsvgCreateRasterizer()) { }

    NSVGrasterizer* rasterizer_ = nullptr;
  };

  bgfx::VertexLayout& IconVertex::layout() {
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

  class IconGroupTexture {
  public:
    explicit IconGroupTexture(int width) : width_(width) {
      texture_ = std::make_unique<unsigned char[]>(width * width);
    }

    ~IconGroupTexture() { destroyHandle(); }

    void destroyHandle() {
      if (bgfx::isValid(texture_handle_))
        bgfx::destroy(texture_handle_);

      bgfx::frame();
      bgfx::frame();
      texture_handle_ = BGFX_INVALID_HANDLE;
    }

    void createHandle() {
      if (bgfx::isValid(texture_handle_)) {
        bgfx::destroy(texture_handle_);
        bgfx::frame();
        bgfx::frame();
      }

      const bgfx::Memory* texture_ref = bgfx::makeRef(texture_.get(),
                                                      width_ * width_ * sizeof(unsigned char));
      texture_handle_ = bgfx::createTexture2D(width_, width_, false, 1, bgfx::TextureFormat::A8,
                                              BGFX_TEXTURE_NONE | BGFX_SAMPLER_NONE, texture_ref);
    }

    bgfx::TextureHandle& handle() { return texture_handle_; }
    unsigned char* data() const { return texture_.get(); }

  private:
    int width_ = 0;
    std::unique_ptr<unsigned char[]> texture_;
    bgfx::TextureHandle texture_handle_ = BGFX_INVALID_HANDLE;
  };

  IconGroup::IconGroup(const std::set<Icon>& icons) {
    atlas_.setPadding(kIconBuffer);
    addIcons(icons);
  }

  void IconGroup::setNewSize() {
    atlas_.pack();
    texture_ = std::make_unique<IconGroupTexture>(atlas_.width());
    for (int i = 0; i < icons_.size(); ++i)
      drawIcon(i);
  }

  IconGroup::~IconGroup() = default;

  bool IconGroup::addIcon(const Icon& icon) {
    int index = icons_.size();
    icons_.push_back(icon);
    icon_lookup_[icon] = index;
    return atlas_.addPackedRect(index, icon.width + icon.blur_radius * 2,
                                icon.height + icon.blur_radius * 2);
  }

  void IconGroup::addIcons(const std::set<Icon>& icons) {
    std::set<Icon> filtered;
    for (const Icon& icon : icons) {
      if (icon_lookup_.count(icon) == 0)
        filtered.insert(icon);
    }

    if (filtered.empty())
      return;

    int start_draw_index = icons_.size();
    bool pack_succeeded = atlas_.packed();
    for (const Icon& icon : filtered) {
      bool packed = addIcon(icon);
      pack_succeeded = pack_succeeded && packed;
    }

    if (!pack_succeeded) {
      setNewSize();
      start_draw_index = 0;
    }

    for (int i = start_draw_index; i < icons_.size(); ++i)
      drawIcon(i);

    texture_->createHandle();
  }

  void IconGroup::drawIcon(int index) {
    Icon& icon = icons_[index];
    if (icon.width == 0)
      return;

    std::unique_ptr<unsigned char[]> data = Rasterizer::instance().rasterize(icon);

    PackedAtlas::Rect packed_rect = atlas_.rectAtIndex(index);
    int atlas_offset = packed_rect.x + packed_rect.y * atlas_.width();
    unsigned char* texture_ref = texture_->data() + atlas_offset;
    for (int y = 0; y < icon.height; ++y) {
      uint8_t* row_ref = data.get() + y * icon.width * 4 + 3;
      for (int x = 0; x < icon.width; ++x)
        texture_ref[x] = row_ref[4 * x];

      texture_ref += atlas_.width();
    }

    if (icon.blur_radius)
      blurIcon(texture_->data() + atlas_offset, icon.width, icon.blur_radius);
  }

  void IconGroup::blurIcon(unsigned char* location, int width, int blur_radius) const {
    static constexpr int kBoxBlurIterations = 3;

    int radius = std::min(blur_radius, width - 1);
    radius = radius + ((radius + 1) % 2);

    std::unique_ptr<unsigned char[]> cache = std::make_unique<unsigned char[]>(radius);

    for (int r = 0; r < width; ++r) {
      for (int i = 0; i < kBoxBlurIterations; ++i)
        boxBlur(location + r * atlas_.width(), cache.get(), width, radius, 1);
    }

    for (int c = 0; c < width; ++c) {
      for (int i = 0; i < kBoxBlurIterations; ++i)
        boxBlur(location + c, cache.get(), width, radius, atlas_.width());
    }
  }

  const bgfx::TextureHandle& IconGroup::textureHandle() const {
    return texture_->handle();
  }

  void IconGroup::setIconCoordinates(IconVertex* vertices, const Icon& icon) const {
    int index = iconIndex(icon);
    if (index < 0)
      return;

    atlas_.setQuadCoordinates(vertices, index);

    for (int i = 0; i < 4; ++i) {
      vertices[i].direction_x = 1.0f;
      vertices[i].direction_y = 0.0f;
    }
  }
}