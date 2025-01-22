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
#include "visage_utils/space.h"

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
    static constexpr int kChannels = 4;

    ImageGroup();
    virtual ~ImageGroup();

    void clear() {
      image_count_.clear();
      atlas_.clear();
    }

    Point incrementImage(const ImageFile& image);
    void decrementImage(const ImageFile& image);
    int atlasWidth() const { return atlas_.width(); }
    int atlasHeight() const { return atlas_.height(); }
    const bgfx::TextureHandle& textureHandle() const;
    void setImageCoordinates(TextureVertex* vertices, const ImageFile& image) const;

  private:
    void setNewSize();
    bool packImage(const ImageFile& image);
    void drawImage(const ImageFile& image) const;

    PackedAtlas<ImageFile> atlas_;
    std::map<ImageFile, int> image_count_;
    std::unique_ptr<ImageGroupTexture> texture_;
  };
}