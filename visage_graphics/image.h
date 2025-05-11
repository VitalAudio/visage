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

#include "graphics_utils.h"

#include <map>
#include <utility>

namespace visage {
  struct ImageFile {
    ImageFile() = default;
    ImageFile(bool svg, const unsigned char* data, int data_size, int width = 0, int height = 0,
              int blur_radius = 0) :
        svg(svg), data(data), data_size(data_size), width(width), height(height),
        blur_radius(blur_radius) { }

    bool svg = false;
    const unsigned char* data = nullptr;
    int data_size = 0;
    int width = 0;
    int height = 0;
    int blur_radius = 0;

    bool operator==(const ImageFile& other) const {
      return data == other.data && data_size == other.data_size && width == other.width &&
             height == other.height && blur_radius == other.blur_radius;
    }

    bool operator<(const ImageFile& other) const {
      return data < other.data || (data == other.data && data_size < other.data_size) ||
             (data == other.data && data_size == other.data_size && width < other.width) ||
             (data == other.data && data_size == other.data_size && width == other.width &&
              height < other.height) ||
             (data == other.data && data_size == other.data_size && width == other.width &&
              height == other.height && blur_radius < other.blur_radius);
    }
  };

  struct Svg : ImageFile {
    Svg() = default;
    Svg(const unsigned char* data, int data_size, int width, int height, int blur_radius = 0) :
        ImageFile(true, data, data_size, width, height, blur_radius) { }
  };

  struct Image : ImageFile {
    Image() = default;
    Image(const unsigned char* data, int data_size) : ImageFile(false, data, data_size) { }
    Image(const unsigned char* data, int data_size, int width, int height) :
        ImageFile(false, data, data_size, width, height) { }
  };

  class ImageAtlasTexture;

  class ImageAtlas {
  public:
    static constexpr int kImageBuffer = 1;
    static constexpr int kChannels = 4;

    static void blurImage(unsigned char* location, int width, int height, int blur_radius);

    struct PackedImageRect {
      explicit PackedImageRect(const ImageFile& image) : image(image) { }

      ImageFile image;
      int x = 0;
      int y = 0;
      int w = 0;
      int h = 0;
    };

    struct PackedImageReference {
      PackedImageReference(std::weak_ptr<ImageAtlas*> atlas, const PackedImageRect* packed_image_rect) :
          atlas(std::move(atlas)), packed_image_rect(packed_image_rect) { }
      ~PackedImageReference();

      std::weak_ptr<ImageAtlas*> atlas;
      const PackedImageRect* packed_image_rect = nullptr;
    };

    class PackedImage {
    public:
      int x() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_image_rect->x;
      }

      int y() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_image_rect->y;
      }

      int w() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_image_rect->w;
      }

      int h() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_image_rect->h;
      }

      const ImageFile& image() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_image_rect->image;
      }

      const PackedImageRect* packedImageRect() const {
        VISAGE_ASSERT(reference_->atlas.lock().get());
        return reference_->packed_image_rect;
      }

      explicit PackedImage(std::shared_ptr<PackedImageReference> reference) :
          reference_(std::move(reference)) { }

    private:
      std::shared_ptr<PackedImageReference> reference_;
    };

    ImageAtlas();
    virtual ~ImageAtlas();

    PackedImage addImage(const ImageFile& image);
    void clearStaleImages() {
      for (const auto& stale : stale_images_) {
        images_.erase(stale.first);
        atlas_map_.removeRect(stale.second);
      }
      stale_images_.clear();
    }

    int width() const { return atlas_map_.width(); }
    int height() const { return atlas_map_.height(); }
    const bgfx::TextureHandle& textureHandle() const;
    void setImageCoordinates(TextureVertex* vertices, const PackedImage& image) const;

  private:
    void resize();
    void loadImageRect(PackedImageRect* image) const;
    void updateImage(const PackedImageRect* image) const;

    void removeImage(const ImageFile& image) {
      VISAGE_ASSERT(images_.count(image));
      stale_images_[image] = images_[image].get();
    }

    void removeImage(const PackedImageRect* packed_image_rect) {
      removeImage(packed_image_rect->image);
    }

    std::map<ImageFile, std::weak_ptr<PackedImageReference>> references_;
    std::map<ImageFile, std::unique_ptr<PackedImageRect>> images_;
    std::map<ImageFile, const PackedImageRect*> stale_images_;

    PackedAtlasMap<const PackedImageRect*> atlas_map_;
    std::unique_ptr<ImageAtlasTexture> texture_;
    std::shared_ptr<ImageAtlas*> reference_;
  };
}