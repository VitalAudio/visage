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

#include "image.h"

#include "graphics_libs.h"

#include <bimg/decode.h>
#include <bx/allocator.h>
#include <cstring>
#include <nanosvg/src/nanosvg.h>
#include <nanosvg/src/nanosvgrast.h>

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <stb_image_resize2.h>

namespace visage {
  static bx::DefaultAllocator* allocator() {
    static bx::DefaultAllocator allocator;
    return &allocator;
  }

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

  class SvgRasterizer {
  public:
    static SvgRasterizer& instance() {
      static SvgRasterizer instance;
      return instance;
    }

    ~SvgRasterizer() { nsvgDeleteRasterizer(rasterizer_); }

    std::unique_ptr<unsigned int[]> rasterize(const ImageFile& svg) const {
      VISAGE_ASSERT(svg.svg);
      std::unique_ptr<char[]> copy = std::make_unique<char[]>(svg.data_size + 1);
      memcpy(copy.get(), svg.data, svg.data_size);

      NSVGimage* image = nsvgParse(copy.get(), "px", 96);
      std::unique_ptr<unsigned int[]> data = std::make_unique<unsigned int[]>(svg.width * svg.height);

      float width_scale = svg.width / image->width;
      float height_scale = svg.height / image->height;
      float scale = std::min(width_scale, height_scale);
      float x_offset = (svg.width - image->width * scale) * 0.5f;
      float y_offset = (svg.height - image->height * scale) * 0.5f;

      nsvgRasterize(rasterizer_, image, x_offset, y_offset, scale, (unsigned char*)data.get(),
                    svg.width, svg.height, svg.width * 4);
      nsvgDelete(image);
      return data;
    }

  private:
    SvgRasterizer() : rasterizer_(nsvgCreateRasterizer()) { }

    NSVGrasterizer* rasterizer_ = nullptr;
  };

  class ImageGroupTexture {
  public:
    explicit ImageGroupTexture(int width) : width_(width) { }

    ~ImageGroupTexture() { destroyHandle(); }

    void destroyHandle() {
      if (bgfx::isValid(texture_handle_))
        bgfx::destroy(texture_handle_);

      texture_handle_ = BGFX_INVALID_HANDLE;
    }

    bool hasHandle() const { return bgfx::isValid(texture_handle_); }

    bgfx::TextureHandle& handle() { return texture_handle_; }

    void checkHandle() {
      if (!bgfx::isValid(texture_handle_))
        texture_handle_ = bgfx::createTexture2D(width_, width_, false, 1, bgfx::TextureFormat::BGRA8);
    }

    void updateTexture(unsigned int* data, int x, int y, int width, int height) {
      VISAGE_ASSERT(bgfx::isValid(texture_handle_));
      bgfx::updateTexture2D(texture_handle_, 0, 0, x, y, width, height,
                            bgfx::copy(data, width * height * sizeof(unsigned int)));
    }

  private:
    int width_ = 0;
    bgfx::TextureHandle texture_handle_ = BGFX_INVALID_HANDLE;
  };

  ImageGroup::ImageGroup() {
    atlas_.setPadding(kImageBuffer);
  }

  void ImageGroup::setNewSize() {
    for (auto count = image_count_.begin(); count != image_count_.end();) {
      if (count->second == 0) {
        atlas_.removeRect(count->first);
        count = image_count_.erase(count);
      }
      else
        ++count;
    }

    atlas_.pack();
    texture_ = std::make_unique<ImageGroupTexture>(atlas_.width());
  }

  ImageGroup::~ImageGroup() = default;

  bool ImageGroup::packImage(const ImageFile& image) {
    int width = image.width;
    int height = image.height;
    if (image.width == 0 && !image.svg) {
      bimg::ImageContainer* image_container = bimg::imageParse(allocator(), image.data, image.data_size);
      if (image_container) {
        width = image_container->m_width;
        height = image_container->m_height;
      }
      bimg::imageFree(image_container);
    }
    return atlas_.addRect(image, width, height);
  }

  void ImageGroup::decrementImage(const ImageFile& image) {
    image_count_[image]--;
    VISAGE_ASSERT(image_count_[image] >= 0);
  }

  Point ImageGroup::incrementImage(const ImageFile& image) {
    bool added = image_count_.count(image);
    image_count_[image]++;
    if (added) {
      PackedRect packed_rect = atlas_.rectForId(image);
      return { packed_rect.w, packed_rect.h };
    }

    if (!packImage(image))
      setNewSize();
    else if (texture_->hasHandle())
      drawImage(image);

    PackedRect packed_rect = atlas_.rectForId(image);
    return { packed_rect.w, packed_rect.h };
  }

  void ImageGroup::drawImage(const ImageFile& image) const {
    if (image.width == 0 && image.svg)
      return;

    PackedRect packed_rect = atlas_.rectForId(image);
    if (image.svg) {
      std::unique_ptr<unsigned int[]> data = SvgRasterizer::instance().rasterize(image);

      // TODO if (image.blur_radius)
      // TODO   blurImage(data.get(), image.width, image.blur_radius);

      texture_->updateTexture(data.get(), packed_rect.x, packed_rect.y, packed_rect.w, packed_rect.h);
    }
    else {
      bimg::ImageContainer* image_container = bimg::imageParse(allocator(), image.data, image.data_size,
                                                               bimg::TextureFormat::BGRA8);
      if (image_container) {
        if (image_container->m_width == packed_rect.w && image_container->m_height == packed_rect.h) {
          texture_->updateTexture((unsigned int*)image_container->m_data, packed_rect.x,
                                  packed_rect.y, packed_rect.w, packed_rect.h);
        }
        else {
          std::unique_ptr<unsigned char[]> resampled = std::make_unique<unsigned char[]>(packed_rect.w *
                                                                                         packed_rect.h * 4);
          stbir_resize_uint8_srgb((unsigned char*)image_container->m_data, image_container->m_width,
                                  image_container->m_height, image_container->m_width * 4,
                                  resampled.get(), packed_rect.w, packed_rect.h, packed_rect.w * 4,
                                  STBIR_BGRA);
          texture_->updateTexture((unsigned int*)resampled.get(), packed_rect.x, packed_rect.y,
                                  packed_rect.w, packed_rect.h);
        }
        bimg::imageFree(image_container);
      }
    }
  }

  void ImageGroup::blurImage(unsigned int* location, int width, int blur_radius) const {
    static constexpr int kBoxBlurIterations = 3;

    int radius = std::min(blur_radius, width - 1);
    radius = radius + ((radius + 1) % 2);

    std::unique_ptr<unsigned char[]> cache = std::make_unique<unsigned char[]>(radius);

    unsigned char* location_char = reinterpret_cast<unsigned char*>(location);

    for (int r = 0; r < width; ++r) {
      for (int i = 0; i < kBoxBlurIterations; ++i) {
        for (int channel = 0; channel < 4; ++channel)
          boxBlur(location_char + r * width * 4 + channel, cache.get(), width, radius, 4);
      }
    }

    for (int c = 0; c < width * 4; ++c) {
      for (int i = 0; i < kBoxBlurIterations; ++i)
        boxBlur(location_char + c, cache.get(), width, radius, width * 4);
    }
  }

  const bgfx::TextureHandle& ImageGroup::textureHandle() const {
    if (!texture_->hasHandle()) {
      texture_->checkHandle();
      for (auto& icon : image_count_)
        drawImage(icon.first);
    }
    return texture_->handle();
  }

  void ImageGroup::setImageCoordinates(TextureVertex* vertices, const ImageFile& image) const {
    VISAGE_ASSERT(image_count_.count(image) && image_count_.at(image) > 0);
    atlas_.setTexturePositionsForId(image, vertices);

    for (int i = 0; i < 4; ++i) {
      vertices[i].direction_x = 1.0f;
      vertices[i].direction_y = 0.0f;
    }
  }
}