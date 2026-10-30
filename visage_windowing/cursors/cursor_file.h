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

#include <cstdint>
#include <vector>

namespace visage::cursor_file {
  struct Entry {
    int width = 0;
    int height = 0;
    int hot_x = 0;
    int hot_y = 0;
    int bits_per_pixel = 0;
    const unsigned char* image = nullptr;
    int image_size = 0;
  };

  inline uint32_t read16(const unsigned char* data) {
    return data[0] | (data[1] << 8);
  }

  inline uint32_t read32(const unsigned char* data) {
    return read16(data) | (read16(data + 2) << 16);
  }

  inline bool closerSize(const Entry& candidate, const Entry& current, int target_size) {
    bool candidate_fits = candidate.width >= target_size;
    bool current_fits = current.width >= target_size;
    if (candidate_fits != current_fits)
      return candidate_fits;
    if (candidate.width != current.width)
      return candidate_fits ? candidate.width < current.width : candidate.width > current.width;
    return candidate.bits_per_pixel > current.bits_per_pixel;
  }

  inline bool bestEntry(const unsigned char* data, int size, int target_size, Entry& result) {
    static constexpr int kHeaderSize = 6;
    static constexpr int kEntrySize = 16;
    static constexpr int kBitmapHeaderSize = 40;

    if (data == nullptr || size < kHeaderSize || read16(data + 2) != 2)
      return false;

    int count = read16(data + 4);
    bool found = false;
    for (int i = 0; i < count && kHeaderSize + (i + 1) * kEntrySize <= size; ++i) {
      const unsigned char* entry = data + kHeaderSize + i * kEntrySize;
      uint32_t image_size = read32(entry + 8);
      uint32_t offset = read32(entry + 12);
      if (offset > static_cast<uint32_t>(size) || image_size > size - offset ||
          image_size < kBitmapHeaderSize) {
        continue;
      }

      Entry candidate;
      candidate.width = entry[0] ? entry[0] : 256;
      candidate.height = entry[1] ? entry[1] : 256;
      candidate.hot_x = read16(entry + 4);
      candidate.hot_y = read16(entry + 6);
      candidate.bits_per_pixel = read16(data + offset + 14);
      candidate.image = data + offset;
      candidate.image_size = image_size;
      if (!found || closerSize(candidate, result, target_size)) {
        result = candidate;
        found = true;
      }
    }
    return found;
  }

  inline std::vector<uint32_t> decode(const Entry& entry) {
    int width = entry.width;
    int height = entry.height;
    int bits = entry.bits_per_pixel;
    if (bits != 1 && bits != 4 && bits != 8 && bits != 24 && bits != 32)
      return {};

    uint32_t header_size = read32(entry.image);
    uint32_t palette_count = read32(entry.image + 32);
    if (palette_count == 0 && bits <= 8)
      palette_count = 1 << bits;

    int color_stride = ((width * bits + 31) / 32) * 4;
    int mask_stride = ((width + 31) / 32) * 4;
    size_t palette_offset = header_size;
    size_t color_offset = palette_offset + palette_count * 4;
    size_t mask_offset = color_offset + static_cast<size_t>(color_stride) * height;
    if (mask_offset + static_cast<size_t>(mask_stride) * height > static_cast<size_t>(entry.image_size))
      return {};

    const unsigned char* palette = entry.image + palette_offset;
    std::vector<uint32_t> pixels(width * height, 0);
    for (int y = 0; y < height; ++y) {
      const unsigned char* color_row = entry.image + color_offset + (height - 1 - y) * color_stride;
      const unsigned char* mask_row = entry.image + mask_offset + (height - 1 - y) * mask_stride;
      for (int x = 0; x < width; ++x) {
        bool transparent = (mask_row[x / 8] >> (7 - x % 8)) & 1;
        uint32_t pixel = 0;
        if (bits == 32)
          pixel = read32(color_row + x * 4);
        else if (bits == 24)
          pixel = 0xff000000 | color_row[x * 3] | (color_row[x * 3 + 1] << 8) |
                  (color_row[x * 3 + 2] << 16);
        else {
          int pixels_per_byte = 8 / bits;
          int shift = (pixels_per_byte - 1 - x % pixels_per_byte) * bits;
          uint32_t index = (color_row[x / pixels_per_byte] >> shift) & ((1 << bits) - 1);
          if (index < palette_count)
            pixel = 0xff000000 | (read32(palette + index * 4) & 0xffffff);
        }

        if (bits != 32 && transparent)
          pixel = 0;
        pixels[y * width + x] = pixel;
      }
    }
    return pixels;
  }
}
