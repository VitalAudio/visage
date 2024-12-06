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

#include <bimg/bimg.h>
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

    std::unique_ptr<unsigned int[]> rasterize(const ImageFile& image) const {
      if (image.svg)
        return rasterizeSvg(image);

      bimg::ImageContainer image_container;
      bimg::imageParse(image_container, image.data, image.data_size);

      return nullptr;
    }

  private:
    Rasterizer() : rasterizer_(nsvgCreateRasterizer()) { }

    std::unique_ptr<unsigned int[]> rasterizeSvg(const ImageFile& svg) const {
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

    NSVGrasterizer* rasterizer_ = nullptr;
  };

  class ImageGroupTexture {
  public:
    explicit ImageGroupTexture(int width) : width_(width) {
      texture_ = std::make_unique<unsigned int[]>(width * width);
    }

    ~ImageGroupTexture() { destroyHandle(); }

    void destroyHandle() {
      if (bgfx::isValid(texture_handle_))
        bgfx::destroy(texture_handle_);

      texture_handle_ = BGFX_INVALID_HANDLE;
    }

    bgfx::TextureHandle& handle() {
      if (!bgfx::isValid(texture_handle_)) {
        const bgfx::Memory* texture_ref = bgfx::makeRef(texture_.get(),
                                                        width_ * width_ * sizeof(unsigned int));
        texture_handle_ = bgfx::createTexture2D(width_, width_, false, 1, bgfx::TextureFormat::BGRA8,
                                                BGFX_TEXTURE_NONE | BGFX_SAMPLER_NONE, texture_ref);
      }
      return texture_handle_;
    }

    unsigned int* data() const { return texture_.get(); }

  private:
    int width_ = 0;
    std::unique_ptr<unsigned int[]> texture_;
    bgfx::TextureHandle texture_handle_ = BGFX_INVALID_HANDLE;
  };

  ImageGroup::ImageGroup() {
    atlas_.setPadding(kIconBuffer);
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
    for (auto& icon : image_count_)
      drawImage(icon.first);
  }

  ImageGroup::~ImageGroup() = default;

  bool ImageGroup::packImage(const ImageFile& image) {
    return atlas_.addRect(image, image.width + image.blur_radius * 2, image.height + image.blur_radius * 2);
  }

  void ImageGroup::decrementImage(const ImageFile& image) {
    image_count_[image]--;
    VISAGE_ASSERT(image_count_[image] >= 0);
  }

  void ImageGroup::incrementImage(const ImageFile& image) {
    bool added = image_count_.count(image);
    image_count_[image]++;
    if (added)
      return;

    if (!packImage(image))
      setNewSize();
    else
      drawImage(image);

    texture_->destroyHandle();
  }

  void ImageGroup::drawImage(const ImageFile& image) {
    if (image.width == 0)
      return;

    std::unique_ptr<unsigned int[]> data = Rasterizer::instance().rasterize(image);

    PackedRect packed_rect = atlas_.rectForId(image);
    int atlas_offset = packed_rect.x + packed_rect.y * atlas_.width();
    unsigned int* texture_ref = texture_->data() + atlas_offset;
    for (int y = 0; y < image.height; ++y) {
      unsigned int* row_ref = data.get() + y * image.width;
      for (int x = 0; x < image.width; ++x)
        texture_ref[x] = row_ref[x];

      texture_ref += atlas_.width();
    }

    if (image.blur_radius)
      blurImage(texture_->data() + atlas_offset, image.width, image.blur_radius);
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
          boxBlur(location_char + r * atlas_.width() * 4 + channel, cache.get(), width, radius, 4);
      }
    }

    for (int c = 0; c < width * 4; ++c) {
      for (int i = 0; i < kBoxBlurIterations; ++i)
        boxBlur(location_char + c, cache.get(), width, radius, atlas_.width() * 4);
    }
  }

  const bgfx::TextureHandle& ImageGroup::textureHandle() const {
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