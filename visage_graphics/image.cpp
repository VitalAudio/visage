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

#include "image.h"

#include <bgfx/bgfx.h>
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
    for (int i = 0; i < blur_radius; ++i)
      cache[i] = 0;

    int value = 0;
    int sample_index = 0;
    int write_index = 0;
    for (; sample_index < blur_radius / 2; ++sample_index) {
      cache[sample_index] = dest[sample_index * stride];
      value += cache[sample_index];
    }

    for (; sample_index < width - blur_radius / 2; ++sample_index) {
      int cache_index = sample_index % blur_radius;
      int cached = cache[cache_index];
      int sample = dest[sample_index * stride];
      cache[cache_index] = sample;
      value += sample - cached;
      dest[write_index * stride] = value / blur_radius;
      write_index++;
    }

    for (; sample_index < width; ++sample_index) {
      value -= cache[sample_index % blur_radius];
      dest[write_index * stride] = value / blur_radius;
      write_index++;
    }
  }

  static void blurImage(unsigned char* location, int width, int height, int blur_radius) {
    static constexpr int kBoxBlurIterations = 3;

    int radius = std::min(blur_radius, width - 1);
    radius = radius + ((radius + 1) % 2);

    std::unique_ptr<unsigned char[]> cache = std::make_unique<unsigned char[]>(radius);

    for (int r = 0; r < height; ++r) {
      for (int channel = 0; channel < ImageAtlas::kChannels; ++channel) {
        for (int i = 0; i < kBoxBlurIterations; ++i)
          boxBlur(location + r * width * ImageAtlas::kChannels + channel, cache.get(), width,
                  radius, ImageAtlas::kChannels);
      }
    }

    for (int c = 0; c < width * ImageAtlas::kChannels; ++c) {
      for (int i = 0; i < kBoxBlurIterations; ++i)
        boxBlur(location + c, cache.get(), width, radius, width * ImageAtlas::kChannels);
    }
  }

  class SvgRasterizer {
  public:
    static SvgRasterizer& instance() {
      static SvgRasterizer instance;
      return instance;
    }

    ~SvgRasterizer() { nsvgDeleteRasterizer(rasterizer_); }

    std::unique_ptr<unsigned char[]> rasterize(const ImageFile& svg) const {
      VISAGE_ASSERT(svg.svg);
      std::unique_ptr<char[]> copy = std::make_unique<char[]>(svg.data_size + 1);
      memcpy(copy.get(), svg.data, svg.data_size);

      NSVGimage* image = nsvgParse(copy.get(), "px", 96);
      std::unique_ptr<unsigned char[]> data = std::make_unique<unsigned char[]>(svg.width * svg.height *
                                                                                ImageAtlas::kChannels);

      float width_scale = svg.width / image->width;
      float height_scale = svg.height / image->height;
      float scale = std::min(width_scale, height_scale);
      float x_offset = (svg.width - image->width * scale) * 0.5f;
      float y_offset = (svg.height - image->height * scale) * 0.5f;

      nsvgRasterize(rasterizer_, image, x_offset, y_offset, scale, data.get(), svg.width,
                    svg.height, svg.width * ImageAtlas::kChannels);
      nsvgDelete(image);
      return data;
    }

  private:
    SvgRasterizer() : rasterizer_(nsvgCreateRasterizer()) { }

    NSVGrasterizer* rasterizer_ = nullptr;
  };

  class ImageAtlasTexture {
  public:
    explicit ImageAtlasTexture(int width, int height) : width_(width), height_(height) { }

    ~ImageAtlasTexture() { destroyHandle(); }

    void destroyHandle() {
      if (bgfx::isValid(texture_handle_))
        bgfx::destroy(texture_handle_);

      texture_handle_ = BGFX_INVALID_HANDLE;
    }

    bool hasHandle() const { return bgfx::isValid(texture_handle_); }

    bgfx::TextureHandle& handle() { return texture_handle_; }

    void checkHandle() {
      if (!bgfx::isValid(texture_handle_))
        texture_handle_ = bgfx::createTexture2D(width_, height_, false, 1, bgfx::TextureFormat::RGBA8);
    }

    void updateTexture(const unsigned char* data, int x, int y, int width, int height) {
      VISAGE_ASSERT(bgfx::isValid(texture_handle_));
      bgfx::updateTexture2D(texture_handle_, 0, 0, x, y, width, height,
                            bgfx::copy(data, width * height * ImageAtlas::kChannels));
    }

  private:
    int width_ = 0;
    int height_ = 0;
    bgfx::TextureHandle texture_handle_ = BGFX_INVALID_HANDLE;
  };

  ImageAtlas::PackedImageReference::~PackedImageReference() {
    if (auto atlas_pointer = atlas.lock())
      (*atlas_pointer)->removeImage(packed_image_rect);
  }

  ImageAtlas::ImageAtlas() {
    reference_ = std::make_shared<ImageAtlas*>(this);
    atlas_map_.setPadding(kImageBuffer);
  }

  ImageAtlas::~ImageAtlas() = default;

  ImageAtlas::PackedImage ImageAtlas::addImage(const ImageFile& image) {
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

    if (images_.count(image) == 0) {
      std::unique_ptr<PackedImageRect> packed_image_rect = std::make_unique<PackedImageRect>(image);
      if (!atlas_map_.addRect(packed_image_rect.get(), width, height))
        resize();

      const PackedRect& rect = atlas_map_.rectForId(packed_image_rect.get());
      packed_image_rect->x = rect.x;
      packed_image_rect->y = rect.y;
      packed_image_rect->w = rect.w;
      packed_image_rect->h = rect.h;
      updateImage(packed_image_rect.get());
      images_[image] = std::move(packed_image_rect);
    }
    stale_images_.erase(image);

    if (auto reference = references_[image].lock())
      return PackedImage(reference);

    auto reference = std::make_shared<PackedImageReference>(reference_, images_[image].get());
    references_[image] = reference;
    return PackedImage(reference);
  }

  void ImageAtlas::resize() {
    clearStaleImages();

    atlas_map_.pack();
    texture_ = std::make_unique<ImageAtlasTexture>(atlas_map_.width(), atlas_map_.height());
  }

  void ImageAtlas::updateImage(const PackedImageRect* image) const {
    if (image->w == 0 && image->image.svg)
      return;

    if (texture_ == nullptr || !bgfx::isValid(texture_->handle()))
      return;

    PackedRect packed_rect = atlas_map_.rectForId(image);
    if (image->image.svg) {
      std::unique_ptr<unsigned char[]> data = SvgRasterizer::instance().rasterize(image->image);

      if (image->image.blur_radius)
        blurImage(data.get(), image->image.width, image->image.height, image->image.blur_radius);

      texture_->updateTexture(data.get(), packed_rect.x, packed_rect.y, packed_rect.w, packed_rect.h);
    }
    else {
      bimg::ImageContainer* image_container = bimg::imageParse(allocator(), image->image.data,
                                                               image->image.data_size,
                                                               bimg::TextureFormat::RGBA8);
      if (image_container) {
        unsigned char* image_data = static_cast<unsigned char*>(image_container->m_data);
        if (image_container->m_width == packed_rect.w && image_container->m_height == packed_rect.h) {
          texture_->updateTexture(image_data, packed_rect.x, packed_rect.y, packed_rect.w,
                                  packed_rect.h);
        }
        else {
          int size = packed_rect.w * packed_rect.h * kChannels;
          std::unique_ptr<unsigned char[]> resampled = std::make_unique<unsigned char[]>(size);
          stbir_resize_uint8_srgb(image_data, image_container->m_width, image_container->m_height,
                                  image_container->m_width * kChannels, resampled.get(),
                                  packed_rect.w, packed_rect.h, packed_rect.w * kChannels, STBIR_BGRA);
          texture_->updateTexture(resampled.get(), packed_rect.x, packed_rect.y, packed_rect.w,
                                  packed_rect.h);
        }
        bimg::imageFree(image_container);
      }
    }
  }

  const bgfx::TextureHandle& ImageAtlas::textureHandle() const {
    if (!texture_->hasHandle()) {
      texture_->checkHandle();
      for (auto& image : images_)
        updateImage(image.second.get());
    }
    return texture_->handle();
  }

  void ImageAtlas::setImageCoordinates(TextureVertex* vertices, const PackedImage& image) const {
    TextureRect rect = atlas_map_.texturePositionsForId(image.packedImageRect());

    vertices[0].texture_x = rect.left;
    vertices[0].texture_y = rect.top;
    vertices[1].texture_x = rect.right;
    vertices[1].texture_y = rect.top;
    vertices[2].texture_x = rect.left;
    vertices[2].texture_y = rect.bottom;
    vertices[3].texture_x = rect.right;
    vertices[3].texture_y = rect.bottom;

    for (int i = 0; i < kVerticesPerQuad; ++i) {
      vertices[i].direction_x = 1.0f;
      vertices[i].direction_y = 0.0f;
    }
  }
}