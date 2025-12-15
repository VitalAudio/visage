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

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <stb_image_resize2.h>

namespace visage {
  static bx::DefaultAllocator* allocator() {
    static bx::DefaultAllocator allocator;
    return &allocator;
  }

  class ImageAtlasTexture {
  public:
    explicit ImageAtlasTexture(int width, int height, ImageAtlas::DataType data_type) :
        width_(width), height_(height), data_type_(data_type) { }

    ~ImageAtlasTexture() { destroyHandle(); }

    void destroyHandle() {
      if (bgfx::isValid(texture_handle_))
        bgfx::destroy(texture_handle_);

      texture_handle_ = BGFX_INVALID_HANDLE;
    }

    bool hasHandle() const { return bgfx::isValid(texture_handle_); }

    bgfx::TextureHandle& handle() { return texture_handle_; }

    void checkHandle() {
      if (!bgfx::isValid(texture_handle_)) {
        if (data_type_ == ImageAtlas::DataType::Float32)
          texture_handle_ = bgfx::createTexture2D(width_, height_, false, 1, bgfx::TextureFormat::R32F);
        else
          texture_handle_ = bgfx::createTexture2D(width_, height_, false, 1, bgfx::TextureFormat::RGBA8);
      }
    }

    void updateTexture(const unsigned char* data, int x, int y, int width, int height) {
      VISAGE_ASSERT(bgfx::isValid(texture_handle_));
      bgfx::updateTexture2D(texture_handle_, 0, 0, x, y, width, height,
                            bgfx::copy(data, width * height * 4));
    }

  private:
    int width_ = 0;
    int height_ = 0;
    ImageAtlas::DataType data_type_ = ImageAtlas::DataType::RGBA8;
    bgfx::TextureHandle texture_handle_ = BGFX_INVALID_HANDLE;
  };

  ImageAtlas::PackedImageReference::~PackedImageReference() {
    if (auto atlas_pointer = atlas.lock())
      (*atlas_pointer)->removeImage(packed_image_rect);
  }

  ImageAtlas::ImageAtlas(DataType data_type) : data_type_(data_type) {
    reference_ = std::make_shared<ImageAtlas*>(this);
    atlas_map_.setPadding(kImageBuffer);
  }

  ImageAtlas::~ImageAtlas() = default;

  ImageAtlas::PackedImage ImageAtlas::addImage(const Image& image, bool force_update) {
    if (images_.count(image) == 0) {
      int width = image.width;
      int height = image.height;
      if (image.width == 0) {
        bimg::ImageContainer* image_container = bimg::imageParse(allocator(), image.data, image.data_size);
        if (image_container) {
          width = image_container->m_width;
          height = image_container->m_height;
          bimg::imageFree(image_container);
        }
      }

      std::unique_ptr<PackedImageRect> packed_image_rect = std::make_unique<PackedImageRect>(image);
      if (!atlas_map_.addRect(packed_image_rect.get(), width, height))
        resize();

      loadImageRect(packed_image_rect.get());
      updateImage(packed_image_rect.get());
      images_[image] = std::move(packed_image_rect);
    }
    else if (force_update)
      updateImage(images_[image].get());

    stale_images_.erase(image);

    if (auto reference = references_[image].lock())
      return PackedImage(reference);

    auto reference = std::make_shared<PackedImageReference>(reference_, images_[image].get());
    references_[image] = reference;
    return PackedImage(reference);
  }

  ImageAtlas::PackedImage ImageAtlas::addData(const unsigned char* data, int width, int height) {
    Image image(data, width * height * 4, width, height);
    image.raw = true;
    return addImage(image, true);
  }

  void ImageAtlas::resize() {
    clearStaleImages();

    int last_width = atlas_map_.width();
    int last_height = atlas_map_.height();
    atlas_map_.pack();
    for (auto& image : images_)
      loadImageRect(image.second.get());

    if (atlas_map_.width() != last_width || atlas_map_.height() != last_height)
      texture_ = std::make_unique<ImageAtlasTexture>(atlas_map_.width(), atlas_map_.height(), data_type_);
    else
      repacked_ = true;
  }

  void ImageAtlas::loadImageRect(PackedImageRect* image) const {
    const PackedRect& rect = atlas_map_.rectForId(image);
    image->x = rect.x;
    image->y = rect.y;
    image->w = rect.w;
    image->h = rect.h;
  }

  void ImageAtlas::updateImage(const PackedImageRect* image) const {
    if (texture_ == nullptr || !bgfx::isValid(texture_->handle()))
      return;

    PackedRect packed_rect = atlas_map_.rectForId(image);
    if (image->image.raw) {
      texture_->updateTexture(image->image.data, packed_rect.x, packed_rect.y, packed_rect.w,
                              packed_rect.h);
      return;
    }
    bimg::ImageContainer* image_container = bimg::imageParse(allocator(), image->image.data,
                                                             image->image.data_size,
                                                             bimg::TextureFormat::RGBA8);
    if (image_container) {
      unsigned char* image_data = static_cast<unsigned char*>(image_container->m_data);
      if (image_container->m_width == packed_rect.w && image_container->m_height == packed_rect.h)
        texture_->updateTexture(image_data, packed_rect.x, packed_rect.y, packed_rect.w, packed_rect.h);
      else {
        int size = packed_rect.w * packed_rect.h * numChannels();
        std::unique_ptr<unsigned char[]> resampled = std::make_unique<unsigned char[]>(size);
        stbir_resize_uint8_srgb(image_data, image_container->m_width, image_container->m_height,
                                image_container->m_width * numChannels(), resampled.get(),
                                packed_rect.w, packed_rect.h, packed_rect.w * numChannels(), STBIR_RGBA);
        texture_->updateTexture(resampled.get(), packed_rect.x, packed_rect.y, packed_rect.w,
                                packed_rect.h);
      }
      bimg::imageFree(image_container);
    }
    else
      VISAGE_ASSERT(false);
  }

  const bgfx::TextureHandle& ImageAtlas::textureHandle() {
    VISAGE_ASSERT(texture_.get());
    if (!texture_->hasHandle() || repacked_) {
      texture_->checkHandle();
      for (auto& image : images_) {
        if (stale_images_.count(image.first) == 0)
          updateImage(image.second.get());
      }
      repacked_ = false;
    }
    return texture_->handle();
  }

  void ImageAtlas::setImageCoordinates(TextureVertex* vertices, const PackedImage& image) const {
    float left = image.x();
    float top = image.y();
    float right = left + image.w();
    float bottom = top + image.h();

    vertices[0].texture_x = left;
    vertices[0].texture_y = top;
    vertices[1].texture_x = right;
    vertices[1].texture_y = top;
    vertices[2].texture_x = left;
    vertices[2].texture_y = bottom;
    vertices[3].texture_x = right;
    vertices[3].texture_y = bottom;

    for (int i = 0; i < kVerticesPerQuad; ++i) {
      vertices[i].direction_x = 1.0f;
      vertices[i].direction_y = 0.0f;
    }
  }
}