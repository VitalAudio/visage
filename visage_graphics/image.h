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
  struct Image {
    Image() = default;
    Image(const unsigned char* data, int data_size, int width = 0, int height = 0) :
        data(data), data_size(data_size), width(width), height(height) { }

    const unsigned char* data = nullptr;
    int data_size = 0;
    int width = 0;
    int height = 0;
    bool raw = false;

    bool operator==(const Image& other) const {
      return data == other.data && data_size == other.data_size && width == other.width &&
             height == other.height;
    }

    bool operator<(const Image& other) const {
      return data < other.data || (data == other.data && data_size < other.data_size) ||
             (data == other.data && data_size == other.data_size && width < other.width) ||
             (data == other.data && data_size == other.data_size && width == other.width &&
              height < other.height);
    }
  };

  class GraphData {
  public:
    GraphData(int num_points = 0) : num_points_(num_points), y_values_(num_points, 0.0f) { }

    void setNumPoints(int num_points) {
      num_points_ = num_points;
      y_values_.resize(num_points_, 0.0f);
    }

    int numPoints() const { return num_points_; }

    void clear() { std::fill(y_values_.begin(), y_values_.end(), 0.0f); }

    float& operator[](int index) {
      VISAGE_ASSERT(index >= 0 && index < num_points_);
      return y_values_[index];
    }

    const float& operator[](int index) const {
      VISAGE_ASSERT(index >= 0 && index < num_points_);
      return y_values_[index];
    }

    const unsigned char* data() const { return (const unsigned char*)y_values_.data(); }

  private:
    int num_points_ = 0;
    std::vector<float> y_values_;
  };

  class HeatMapData {
  public:
    HeatMapData(int width = 0, int height = 0) :
        width_(width), height_(height), values_(width * height, 0.0f) { }

    void setDimensions(int width, int height) {
      width_ = width;
      height_ = height;
      values_.resize(width * height, 0.0f);
    }

    float maxValue() const {
      float max_value = 0.0f;
      for (const auto& value : values_) {
        if (value > max_value)
          max_value = value;
      }
      return max_value;
    }

    float minValue() const {
      if (values_.empty())
        return 0.0f;

      float min_value = values_[0];
      for (const auto& value : values_) {
        if (value < min_value)
          min_value = value;
      }
      return min_value;
    }

    void scale(float scale) {
      for (auto& value : values_)
        value *= scale;
    }

    void normalize() {
      float max_value = maxValue();

      if (max_value > 0.0f)
        scale(1.0f / max_value);
    }

    void setOctaves(float octaves) { octaves_ = octaves; }
    float octaves() const { return octaves_; }

    int width() const { return width_; }
    int height() const { return height_; }

    void clear() { std::fill(values_.begin(), values_.end(), 0.0f); }

    const unsigned char* data() const { return (const unsigned char*)values_.data(); }

    void set(int x, int y, float value) {
      VISAGE_ASSERT(x >= 0 && x < width_ && y >= 0 && y < height_);
      values_[y * width_ + x] = value;
    }

    float& at(int x, int y) {
      VISAGE_ASSERT(x >= 0 && x < width_ && y >= 0 && y < height_);
      return values_[y * width_ + x];
    }

    const float& at(int x, int y) const {
      VISAGE_ASSERT(x >= 0 && x < width_ && y >= 0 && y < height_);
      return values_[y * width_ + x];
    }

  private:
    int width_ = 0;
    int height_ = 0;
    float octaves_ = 0.0f;
    std::vector<float> values_;
  };

  class ImageAtlasTexture;

  class ImageAtlas {
  public:
    static constexpr int kImageBuffer = 1;

    enum class DataType {
      RGBA8,
      Float32,
    };

    struct PackedImageRect {
      explicit PackedImageRect(const Image& image) : image(image) { }

      Image image;
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

      const Image& image() const {
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

    ImageAtlas(DataType data_type);
    virtual ~ImageAtlas();

    PackedImage addImage(const Image& image, bool force_update = false);
    PackedImage addData(const unsigned char* data, int width, int height = 1);
    void clearStaleImages() {
      for (const auto& stale : stale_images_) {
        images_.erase(stale.first);
        atlas_map_.removeRect(stale.second);
        references_.erase(stale.first);
      }
      stale_images_.clear();
    }

    int width() const { return atlas_map_.width(); }
    int height() const { return atlas_map_.height(); }
    const bgfx::TextureHandle& textureHandle();
    void setImageCoordinates(TextureVertex* vertices, const PackedImage& image) const;
    int numChannels() const { return data_type_ == DataType::Float32 ? 1 : 4; }
    int bytesPerChannel() const { return data_type_ == DataType::Float32 ? 4 : 1; }

  private:
    void resize();
    void loadImageRect(PackedImageRect* image) const;
    void updateImage(const PackedImageRect* image) const;

    void removeImage(const Image& image) {
      VISAGE_ASSERT(images_.count(image));
      stale_images_[image] = images_[image].get();
    }

    void removeImage(const PackedImageRect* packed_image_rect) {
      removeImage(packed_image_rect->image);
    }

    std::map<Image, std::weak_ptr<PackedImageReference>> references_;
    std::map<Image, std::unique_ptr<PackedImageRect>> images_;
    std::map<Image, const PackedImageRect*> stale_images_;

    DataType data_type_ = DataType::RGBA8;
    bool repacked_ = false;
    PackedAtlasMap<const PackedImageRect*> atlas_map_;
    std::unique_ptr<ImageAtlasTexture> texture_;
    std::shared_ptr<ImageAtlas*> reference_;
  };
}