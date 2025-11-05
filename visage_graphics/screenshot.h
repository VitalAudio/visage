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

#include "visage_utils/space.h"

#include <cstdint>
#include <string>
#include <vector>

namespace visage {
  class Screenshot {
  public:
    Screenshot() = default;
    Screenshot(const uint8_t* data, int width, int height, bool blue_red = false) :
        width_(width), height_(height) {
      data_.resize(width * height * 4);
      std::copy_n(data, width * height * 4, data_.data());
      if (blue_red)
        flipBlueRed();
    }

    Screenshot(const uint8_t* data, int width, int height, int pitch, bool blue_red = false) :
        width_(width), height_(height) {
      VISAGE_ASSERT(pitch >= width * 4);

      data_.resize(width * height * 4);
      if (pitch == width * 4)
        std::copy_n(data, width * height * 4, data_.data());
      else {
        for (int y = 0; y < height; ++y)
          std::copy_n(data + y * pitch, width * 4, data_.data() + y * width * 4);
      }

      if (blue_red)
        flipBlueRed();
    }

    Screenshot(const Screenshot& other) = default;
    Screenshot& operator=(const Screenshot& other) = default;

    void save(const char* path) const;
    void save(const std::string& path) const;

    void setDimensions(int width, int height) {
      width_ = width;
      height_ = height;
      data_.clear();
      data_.resize(width * height * 4, 0);
    }

    const uint8_t* data() const { return data_.data(); }
    uint8_t* data() { return data_.data(); }
    int width() const { return width_; }
    int height() const { return height_; }
    Color sample(int x, int y) const {
      VISAGE_ASSERT(x >= 0 && x < width_ && y >= 0 && y < height_);

      int index = (y * width_ + x) * 4;
      uint32_t color = (data_[index + 3] << 24) | (data_[index] << 16) | (data_[index + 1] << 8) |
                       data_[index + 2];
      return color;
    }

    Color sample(Point point) const { return sample(point.x, point.y); }

  private:
    void flipBlueRed() {
      for (int i = 0; i < width_ * height_ * 4; i += 4) {
        uint8_t temp = data_[i];
        data_[i] = data_[i + 2];
        data_[i + 2] = temp;
      }
    }

    int width_ = 0;
    int height_ = 0;
    std::vector<uint8_t> data_;
  };
}
