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

#pragma once

#include "graphics_utils.h"

#include <map>

namespace visage {
  struct ImageFile {
    ImageFile() = default;
    ImageFile(bool svg, const char* data, int data_size, int width = 0, int height = 0,
              int blur_radius = 0) :
        svg(svg), data(data), data_size(data_size), width(width), height(height),
        blur_radius(blur_radius) { }

    bool svg = false;
    const char* data = nullptr;
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
    Svg(const char* data, int data_size, int width, int height, int blur_radius = 0) :
        ImageFile(true, data, data_size, width, height, blur_radius) { }
  };

  struct Image : ImageFile {
    Image() = default;
    Image(const char* data, int data_size) : ImageFile(false, data, data_size) { }
    Image(const char* data, int data_size, int width, int height) :
        ImageFile(false, data, data_size, width, height) { }
  };

  class ImageGroupTexture;

  class ImageGroup {
  public:
    static constexpr int kImageBuffer = 1;

    ImageGroup();
    virtual ~ImageGroup();

    void clear() {
      image_count_.clear();
      atlas_.clear();
    }

    Point incrementImage(const ImageFile& image);
    void decrementImage(const ImageFile& image);
    int atlasWidth() const { return atlas_.width(); }
    const bgfx::TextureHandle& textureHandle() const;
    void setImageCoordinates(TextureVertex* vertices, const ImageFile& image) const;

  private:
    void setNewSize();
    bool packImage(const ImageFile& image);
    void drawImage(const ImageFile& image) const;
    void blurImage(unsigned int* location, int width, int height, int blur_radius) const;

    PackedAtlas<ImageFile> atlas_;
    std::map<ImageFile, int> image_count_;
    std::unique_ptr<ImageGroupTexture> texture_;
  };
}