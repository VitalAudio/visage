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

#include "screenshot.h"

#include <algorithm>
#include <cstdio>
#include <vector>

namespace visage {
  static uint32_t crc32(const uint8_t* data, size_t size, uint32_t crc = 0) {
    static const auto table = [] {
      std::vector<uint32_t> result(256);
      for (uint32_t i = 0; i < 256; ++i) {
        uint32_t value = i;
        for (int bit = 0; bit < 8; ++bit)
          value = (value & 1) ? 0xedb88320u ^ (value >> 1) : value >> 1;
        result[i] = value;
      }
      return result;
    }();

    crc = ~crc;
    for (size_t i = 0; i < size; ++i)
      crc = table[(crc ^ data[i]) & 0xff] ^ (crc >> 8);
    return ~crc;
  }

  static void appendBigEndian(std::vector<uint8_t>& data, uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8)
      data.push_back((value >> shift) & 0xff);
  }

  static void appendChunk(std::vector<uint8_t>& png, const char* type, const std::vector<uint8_t>& data) {
    appendBigEndian(png, static_cast<uint32_t>(data.size()));
    size_t start = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), data.begin(), data.end());
    appendBigEndian(png, crc32(png.data() + start, png.size() - start));
  }

  static std::vector<uint8_t> encodePng(const uint8_t* pixels, int width, int height) {
    static constexpr uint8_t kSignature[] = { 0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n' };
    static constexpr size_t kMaxStoredBlock = 65535;

    std::vector<uint8_t> raw;
    size_t row_bytes = static_cast<size_t>(width) * 4;
    raw.reserve((row_bytes + 1) * height);
    for (int y = 0; y < height; ++y) {
      raw.push_back(0);
      raw.insert(raw.end(), pixels + y * row_bytes, pixels + (y + 1) * row_bytes);
    }

    std::vector<uint8_t> zlib = { 0x78, 0x01 };
    uint32_t adler_a = 1, adler_b = 0;
    for (uint8_t byte : raw) {
      adler_a = (adler_a + byte) % 65521;
      adler_b = (adler_b + adler_a) % 65521;
    }
    size_t position = 0;
    do {
      size_t block = std::min(kMaxStoredBlock, raw.size() - position);
      bool last = position + block == raw.size();
      zlib.push_back(last ? 1 : 0);
      zlib.push_back(block & 0xff);
      zlib.push_back((block >> 8) & 0xff);
      zlib.push_back(~block & 0xff);
      zlib.push_back((~block >> 8) & 0xff);
      zlib.insert(zlib.end(), raw.begin() + position, raw.begin() + position + block);
      position += block;
    } while (position < raw.size());
    appendBigEndian(zlib, (adler_b << 16) | adler_a);

    std::vector<uint8_t> header;
    appendBigEndian(header, width);
    appendBigEndian(header, height);
    header.insert(header.end(), { 8, 6, 0, 0, 0 });

    std::vector<uint8_t> png(std::begin(kSignature), std::end(kSignature));
    appendChunk(png, "IHDR", header);
    appendChunk(png, "IDAT", zlib);
    appendChunk(png, "IEND", {});
    return png;
  }

  void Screenshot::save(const char* path) const {
    if (width_ <= 0 || height_ <= 0 || data_.empty())
      return;

    std::vector<uint8_t> png = encodePng(data_.data(), width_, height_);
    if (FILE* file = std::fopen(path, "wb")) {
      std::fwrite(png.data(), 1, png.size(), file);
      std::fclose(file);
    }
  }

  void Screenshot::save(const std::string& path) const {
    save(path.c_str());
  }
}