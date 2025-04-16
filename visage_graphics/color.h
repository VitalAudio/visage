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

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iosfwd>
#include <string>

namespace visage {
  class Color {
  public:
    enum {
      kBlue,
      kGreen,
      kRed,
      kAlpha,
      kNumChannels
    };
    static constexpr int kBitsPerColor = 8;
    static constexpr float kFloatScale = 1.0f / 0xff;
    static constexpr float kFloatScale16 = 1.0f / 0xffff;
    static constexpr float kHueRange = 360.0f;
    static constexpr float kGradientNormalization = 64.0f;

    static int compare(const Color& a, const Color& b) {
      for (int i = 0; i < kNumChannels; ++i) {
        if (a.values_[i] < b.values_[i])
          return -1;
        if (a.values_[i] > b.values_[i])
          return 1;
      }
      if (a.hdr_ < b.hdr_)
        return -1;
      if (a.hdr_ > b.hdr_)
        return 1;
      return 0;
    }

    static Color fromAHSV(float alpha, float hue, float saturation, float value) {
      static constexpr float kHueCutoff = kHueRange / 6.0f;
      Color result;

      hue = std::fmod(hue, kHueRange);
      result.values_[kAlpha] = alpha;
      float range = value * saturation;
      float minimum = value - range;
      result.values_[kRed] = result.values_[kGreen] = result.values_[kBlue] = minimum;

      float hue_offset = hue * (0.5f / kHueCutoff);
      hue_offset = (hue_offset - std::floor(hue_offset)) * 2.0f;
      float conversion = range * (1.0f - std::abs(hue_offset - 1.0f));
      int max_index = kRed;
      int middle_index = kGreen;

      if (hue > 5 * kHueCutoff)
        middle_index = kBlue;
      else if (hue > 4 * kHueCutoff) {
        max_index = kBlue;
        middle_index = kRed;
      }
      else if (hue > 3 * kHueCutoff) {
        max_index = kBlue;
        middle_index = kGreen;
      }
      else if (hue > 2 * kHueCutoff) {
        max_index = kGreen;
        middle_index = kBlue;
      }
      else if (hue > kHueCutoff) {
        max_index = kGreen;
        middle_index = kRed;
      }

      result.values_[max_index] += range;
      result.values_[middle_index] += conversion;
      return result;
    }

    static Color fromABGR16(uint64_t abgr) {
      Color result;
      result.loadABGR16(abgr);
      return result;
    }

    static Color fromARGB16(uint64_t argb) {
      Color result;
      result.loadARGB16(argb);
      return result;
    }

    static Color fromABGR(unsigned int abgr) {
      Color result;
      result.loadABGR(abgr);
      return result;
    }

    static Color fromARGB(unsigned int argb) {
      Color result;
      result.loadARGB(argb);
      return result;
    }

    static Color fromHexString(const std::string& color_string) {
      if (color_string.empty())
        return 0;

      std::string hex = color_string[0] == '#' ? color_string.substr(1) : color_string;

      if (hex.size() < 8) {
        unsigned int value = std::stoul(hex, nullptr, 16);
        return value | 0xff000000;
      }
      return fromARGB(std::stoul(hex, nullptr, 16));
    }

    Color() : values_() { }
    Color(float alpha, float red, float green, float blue, float hdr = 1.0f) {
      values_[kAlpha] = alpha;
      values_[kRed] = red;
      values_[kGreen] = green;
      values_[kBlue] = blue;
      hdr_ = hdr;
    }
    Color(const Color&) = default;

    Color(unsigned int argb, float hdr = 1.0f) noexcept {
      loadARGB(argb);
      hdr_ = hdr;
    }

    void loadARGB16(uint64_t abgr) {
      for (int i = 0; i < kNumChannels; ++i) {
        int shift = kBitsPerColor * i * 2;
        values_[i] = ((abgr >> shift) & 0xffff) * kFloatScale16;
      }
    }

    void loadABGR16(uint64_t abgr) {
      loadARGB16(abgr);
      std::swap(values_[kBlue], values_[kRed]);
    }

    void loadARGB(unsigned int abgr) {
      for (int i = 0; i < kNumChannels; ++i) {
        int shift = kBitsPerColor * i;
        values_[i] = ((abgr >> shift) & 0xff) * kFloatScale;
      }
    }

    void loadABGR(unsigned int abgr) {
      loadARGB(abgr);
      std::swap(values_[kBlue], values_[kRed]);
    }

    void setAlpha(float alpha) { values_[kAlpha] = std::max(0.0f, std::min(1.0f, alpha)); }

    void setHdr(float hdr) { hdr_ = std::max(0.0f, hdr); }

    void multRgb(float amount) {
      for (int i = 0; i < kAlpha; ++i)
        values_[i] *= amount;
    }

    uint64_t toABGR16() const {
      float mult = hdr_ / kGradientNormalization;
      uint64_t value = floatToHex16(values_[kAlpha]) << (6 * kBitsPerColor);
      value += floatToHex16(values_[kBlue]) << (4 * kBitsPerColor);
      value += floatToHex16(values_[kGreen]) << (2 * kBitsPerColor);
      return value + floatToHex16(values_[kRed]);
    }

    uint64_t toARGB16() const {
      float mult = hdr_ / kGradientNormalization;
      uint64_t value = floatToHex16(values_[kAlpha]) << (6 * kBitsPerColor);
      value += floatToHex16(values_[kRed]) << (4 * kBitsPerColor);
      value += floatToHex16(values_[kGreen]) << (2 * kBitsPerColor);
      return value + floatToHex16(values_[kBlue]);
    }

    uint64_t toABGR16F() const {
      float mult = hdr_ / kGradientNormalization;
      uint64_t value = floatToHalf(values_[kAlpha]) << (6 * kBitsPerColor);
      value += floatToHalf(values_[kBlue] * mult) << (4 * kBitsPerColor);
      value += floatToHalf(values_[kGreen] * mult) << (2 * kBitsPerColor);
      return value + floatToHalf(values_[kRed] * mult);
    }

    uint64_t toARGB16F() const {
      float mult = hdr_ / kGradientNormalization;
      uint64_t value = floatToHalf(values_[kAlpha]) << (6 * kBitsPerColor);
      value += floatToHalf(values_[kRed] * mult) << (4 * kBitsPerColor);
      value += floatToHalf(values_[kGreen] * mult) << (2 * kBitsPerColor);
      return value + floatToHalf(values_[kBlue] * mult);
    }

    unsigned int toABGR() const {
      unsigned int value = floatToHex(values_[kAlpha]) << (3 * kBitsPerColor);
      value += floatToHex(values_[kBlue]) << (2 * kBitsPerColor);
      value += floatToHex(values_[kGreen]) << kBitsPerColor;
      return value + floatToHex(values_[kRed]);
    }

    unsigned int toARGB() const {
      unsigned int value = floatToHex(values_[kAlpha]) << (3 * kBitsPerColor);
      value += floatToHex(values_[kRed]) << (2 * kBitsPerColor);
      value += floatToHex(values_[kGreen]) << kBitsPerColor;
      return value + floatToHex(values_[kBlue]);
    }

    unsigned int toRGB() const {
      unsigned int value = floatToHex(values_[kRed]) << (2 * kBitsPerColor);
      value += floatToHex(values_[kGreen]) << kBitsPerColor;
      return value + floatToHex(values_[kBlue]);
    }

    float alpha() const { return values_[kAlpha]; }
    float red() const { return values_[kRed]; }
    float green() const { return values_[kGreen]; }
    float blue() const { return values_[kBlue]; }
    float hdr() const { return hdr_; }

    float value() const {
      return std::max(values_[kRed], std::max(values_[kGreen], values_[kBlue]));
    }

    float saturation() const {
      float val = value();
      if (val <= 0.0f)
        return 0.0f;
      float range = val - minColor();
      return range / val;
    }

    float hue() const {
      float min = minColor();
      float max = value();
      float range = max - min;
      if (range <= 0.0f)
        return 0.0f;

      float color_range = kHueRange / 6.0f;

      if (values_[kRed] == max) {
        if (values_[kGreen] == min) {
          float delta = color_range * (values_[kBlue] - min) / range;
          if (delta == 0.0f)
            return 0.0f;
          return kHueRange - delta;
        }
        return color_range * (values_[kGreen] - min) / range;
      }
      if (values_[kGreen] == max) {
        if (values_[kBlue] == min)
          return 2.0f * color_range - color_range * (values_[kRed] - min) / range;
        return 2.0f * color_range + color_range * (values_[kBlue] - min) / range;
      }
      if (values_[kRed] == min)
        return 4.0f * color_range - color_range * (values_[kGreen] - min) / range;
      return 4.0f * color_range + color_range * (values_[kRed] - min) / range;
    }

    float hexAlpha() const { return floatToHex(values_[kAlpha]); }
    float hexRed() const { return floatToHex(values_[kRed]); }
    float hexGreen() const { return floatToHex(values_[kGreen]); }
    float hexBlue() const { return floatToHex(values_[kBlue]); }

    Color& operator=(const Color&) = default;
    Color& operator=(unsigned int argb) {
      loadARGB(argb);
      return *this;
    }
    Color operator*(float mult) const {
      return { values_[kAlpha] * mult, values_[kRed] * mult, values_[kGreen] * mult,
               values_[kBlue] * mult, hdr_ };
    }
    Color operator-(const Color& other) const {
      return { values_[kAlpha] - other.values_[kAlpha], values_[kRed] - other.values_[kRed],
               values_[kGreen] - other.values_[kGreen], values_[kBlue] - other.values_[kBlue], hdr_ };
    }
    Color operator+(const Color& other) const {
      return { values_[kAlpha] + other.values_[kAlpha], values_[kRed] + other.values_[kRed],
               values_[kGreen] + other.values_[kGreen], values_[kBlue] + other.values_[kBlue], hdr_ };
    }
    bool operator==(const Color& other) const {
      return values_[kAlpha] == other.values_[kAlpha] && values_[kRed] == other.values_[kRed] &&
             values_[kGreen] == other.values_[kGreen] && values_[kBlue] == other.values_[kBlue] &&
             hdr_ == other.hdr_;
    }

    bool operator<(const Color& other) const { return compare(*this, other) < 0; }
    bool operator>(const Color& other) const { return compare(*this, other) > 0; }

    std::string encode() const;
    void encode(std::ostringstream& stream) const;
    void decode(const std::string& data);
    void decode(std::istringstream& data);

    Color interpolateWith(const Color& other, float t) const {
      Color result;
      for (int i = 0; i < kNumChannels; ++i)
        result.values_[i] = values_[i] + (other.values_[i] - values_[i]) * t;
      result.hdr_ = hdr_ + (other.hdr_ - hdr_) * t;
      return result;
    }

    Color withAlpha(float alpha) const {
      return { alpha, values_[kRed], values_[kGreen], values_[kBlue], hdr_ };
    }

    std::string toARGBHexString() const {
      return floatToHexString(values_[kAlpha]) + floatToHexString(values_[kRed]) +
             floatToHexString(values_[kGreen]) + floatToHexString(values_[kBlue]);
    }

    std::string toRGBHexString() const {
      return floatToHexString(values_[kRed]) + floatToHexString(values_[kGreen]) +
             floatToHexString(values_[kBlue]);
    }

  private:
    static unsigned int floatToHex(float value) {
      return std::round(std::max(0.0f, std::min(1.0f, value)) * 0xff);
    }

    static uint64_t floatToHex16(float value) {
      return std::round(std::max(0.0f, std::min(1.0f, value)) * 0xffff);
    }

    static uint64_t floatToHalf(float value) {
      uint32_t f;
      std::memcpy(&f, &value, sizeof(f));
      uint32_t sign = (f >> 16) & 0x8000;  // Sign bit
      uint32_t exponent = (f >> 23) & 0xFF;  // Extract exponent (8-bit)
      uint32_t mantissa = f & 0x007FFFFF;  // Extract mantissa (23-bit)

      if (exponent == 255) {  // Handle NaN or Infinity
        if (mantissa)
          return sign | 0x7FFF;  // NaN (all exponent bits 1 and mantissa nonzero)
        return sign | 0x7C00;  // Infinity
      }

      if (exponent > 112) {  // Normalized case
        exponent -= 112;  // Adjust bias from 127 (float) to 15 (half-float)
        if (exponent > 30)  // Overflow (set to Infinity)
          return sign | 0x7C00;
        return sign | (exponent << 10) | (mantissa >> 13);
      }

      if (exponent > 103) {  // Subnormal case
        mantissa |= 0x00800000;  // Add implicit leading 1 bit
        return sign | ((mantissa >> (113 - exponent)) & 0x3FF);
      }

      return sign;  // Underflow to zero
    }

    static char hexCharacter(int value) {
      if (value < 10)
        return '0' + value;
      return 'A' + value - 10;
    }

    static std::string floatToHexString(float value) {
      unsigned int hex_value = floatToHex(value);
      char first_digit = hexCharacter(hex_value & 0xf);
      char second_digit = hexCharacter(hex_value >> 4);
      return std::string(1, second_digit) + std::string(1, first_digit);
    }

    float minColor() const {
      return std::min(values_[kRed], std::min(values_[kGreen], values_[kBlue]));
    }

    float values_[kNumChannels] {};
    float hdr_ = 1.0f;
  };
}