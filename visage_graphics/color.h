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

#include <cmath>
#include <iosfwd>
#include <string>

namespace visage {
  class Color {
  public:
    enum {
      kRed,
      kGreen,
      kBlue,
      kAlpha,
      kNumChannels
    };
    static constexpr int kBitsPerColor = 8;
    static constexpr float kFloatScale = 1.0f / 0xff;
    static constexpr float kHueRange = 360.0f;

    static unsigned int floatToHex(float value) {
      return std::round(std::max(0.0f, std::min(1.0f, value)) * 0xff);
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

    static Color fromAHSV(float alpha, float hue, float saturation, float value) {
      static constexpr float kHueCutoff = kHueRange / 6.0f;
      Color result;

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

      if (color_string.size() <= 8) {
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

    void loadABGR(unsigned int abgr) {
      for (int i = 0; i < kNumChannels; ++i) {
        int shift = kBitsPerColor * i;
        values_[i] = ((abgr >> shift) & 0xff) * kFloatScale;
      }
    }

    void loadARGB(unsigned int argb) {
      loadABGR(argb);
      std::swap(values_[kBlue], values_[kRed]);
    }

    void loadRGB(unsigned int rgb) {
      for (int i = 0; i < kAlpha; ++i) {
        int shift = kBitsPerColor * i;
        values_[i] = ((rgb >> shift) & 0xff) * kFloatScale;
      }
      std::swap(values_[kBlue], values_[kRed]);
    }

    void setAlpha(float alpha) { values_[kAlpha] = std::max(0.0f, std::min(1.0f, alpha)); }

    void setHdr(float hdr) { hdr_ = std::max(0.0f, hdr); }

    void multRgb(float amount) {
      for (int i = 0; i < kAlpha; ++i)
        values_[i] *= amount;
    }

    unsigned int toABGR() const {
      unsigned int value = 0;
      for (int i = kNumChannels - 1; i >= 0; --i) {
        value = value << kBitsPerColor;
        int color_value = floatToHex(values_[i]);
        value += color_value;
      }
      return value;
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

    float minColor() const {
      return std::min(values_[kRed], std::min(values_[kGreen], values_[kBlue]));
    }

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
        if (values_[kGreen] == min)
          return kHueRange - color_range * (values_[kBlue] - min) / range;
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
      return { values_[kAlpha] * mult, values_[kRed] * mult, values_[kGreen] * mult, values_[kBlue] * mult };
    }
    Color operator-(const Color& other) const {
      return { values_[kAlpha] - other.values_[kAlpha], values_[kRed] - other.values_[kRed],
               values_[kGreen] - other.values_[kGreen], values_[kBlue] - other.values_[kBlue] };
    }
    Color operator+(const Color& other) const {
      return { values_[kAlpha] + other.values_[kAlpha], values_[kRed] + other.values_[kRed],
               values_[kGreen] + other.values_[kGreen], values_[kBlue] + other.values_[kBlue] };
    }

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
    float values_[kNumChannels] {};
    float hdr_ = 1.0f;
  };

  class HorizontalGradient {
  public:
    HorizontalGradient(Color left, Color right) : left_(left), right_(right) { }

    const Color& left() const { return left_; }
    const Color& right() const { return right_; }

  private:
    Color left_;
    Color right_;
  };

  class VerticalGradient {
  public:
    VerticalGradient(Color top, Color bottom) : top_(top), bottom_(bottom) { }

    const Color& top() const { return top_; }
    const Color& bottom() const { return bottom_; }

  private:
    Color top_;
    Color bottom_;
  };

  struct QuadColor {
    enum {
      kTopLeft,
      kTopRight,
      kBottomLeft,
      kBottomRight,
      kNumCorners
    };

    unsigned int corners[kNumCorners] {};
    float hdr[kNumCorners] {};

    static unsigned int toABGR(unsigned int argb) {
      int abgr = argb;
      char* chars = reinterpret_cast<char*>(&abgr);
      std::swap(chars[0], chars[2]);
      return abgr;
    }

    static int interpolateColor(unsigned int hex_color1, unsigned int hex_color2, float t) {
      unsigned int result = 0;
      for (int i = 0; i < Color::kNumChannels; ++i) {
        int hex_shift = i * Color::kBitsPerColor;
        int channel1 = (hex_color1 >> hex_shift) & 0xff;
        int channel2 = (hex_color2 >> hex_shift) & 0xff;
        unsigned int result_channel = channel1 + (channel2 - channel1) * t;
        result_channel = std::min<int>(0xff, result_channel);
        result = result | (result_channel << hex_shift);
      }
      return result;
    }

    static float interpolateHdr(float from, float to, float t) { return from + (to - from) * t; }

    QuadColor() : corners() { }

    QuadColor(const Color& color) {
      unsigned int value = color.toABGR();
      float hdr_value = color.hdr();
      corners[kTopLeft] = value;
      corners[kTopRight] = value;
      corners[kBottomLeft] = value;
      corners[kBottomRight] = value;
      hdr[kTopLeft] = hdr_value;
      hdr[kTopRight] = hdr_value;
      hdr[kBottomLeft] = hdr_value;
      hdr[kBottomRight] = hdr_value;
    }

    QuadColor(unsigned int top_left, unsigned int top_right, unsigned int bottom_left,
              unsigned int bottom_right, float top_left_hdr, float top_right_hdr,
              float bottom_left_hdr, float bottom_right_hdr) {
      corners[kTopLeft] = top_left;
      corners[kTopRight] = top_right;
      corners[kBottomLeft] = bottom_left;
      corners[kBottomRight] = bottom_right;
      hdr[kTopLeft] = top_left_hdr;
      hdr[kTopRight] = top_right_hdr;
      hdr[kBottomLeft] = bottom_left_hdr;
      hdr[kBottomRight] = bottom_right_hdr;
    }

    QuadColor(const HorizontalGradient& gradient) {
      unsigned int left = gradient.left().toABGR();
      unsigned int right = gradient.right().toABGR();
      float left_hdr = gradient.left().hdr();
      float right_hdr = gradient.right().hdr();
      corners[kTopLeft] = left;
      corners[kTopRight] = right;
      corners[kBottomLeft] = left;
      corners[kBottomRight] = right;
      hdr[kTopLeft] = left_hdr;
      hdr[kTopRight] = right_hdr;
      hdr[kBottomLeft] = left_hdr;
      hdr[kBottomRight] = right_hdr;
    }

    QuadColor(const VerticalGradient& gradient) {
      unsigned int top = gradient.top().toABGR();
      unsigned int bottom = gradient.bottom().toABGR();
      float top_hdr = gradient.top().hdr();
      float bottom_hdr = gradient.bottom().hdr();
      corners[kTopLeft] = top;
      corners[kTopRight] = top;
      corners[kBottomLeft] = bottom;
      corners[kBottomRight] = bottom;
      hdr[kTopLeft] = top_hdr;
      hdr[kTopRight] = top_hdr;
      hdr[kBottomLeft] = bottom_hdr;
      hdr[kBottomRight] = bottom_hdr;
    }

    QuadColor(int argb, float hdr_value = 1.0f) {
      unsigned int abgr = toABGR(argb);
      corners[kTopLeft] = abgr;
      corners[kTopRight] = abgr;
      corners[kBottomLeft] = abgr;
      corners[kBottomRight] = abgr;
      hdr[kTopLeft] = hdr_value;
      hdr[kTopRight] = hdr_value;
      hdr[kBottomLeft] = hdr_value;
      hdr[kBottomRight] = hdr_value;
    }

    QuadColor withAlpha(float alpha) const {
      int alpha_value = alpha * 0xff;
      alpha_value = alpha_value << (Color::kBitsPerColor * 3);
      QuadColor result;
      result.corners[kTopLeft] = alpha_value + (corners[kTopLeft] & 0xffffff);
      result.corners[kTopRight] = alpha_value + (corners[kTopRight] & 0xffffff);
      result.corners[kBottomLeft] = alpha_value + (corners[kBottomLeft] & 0xffffff);
      result.corners[kBottomRight] = alpha_value + (corners[kBottomRight] & 0xffffff);
      result.hdr[kTopLeft] = hdr[kTopLeft];
      result.hdr[kTopRight] = hdr[kTopRight];
      result.hdr[kBottomLeft] = hdr[kBottomLeft];
      result.hdr[kBottomRight] = hdr[kBottomRight];
      return result;
    }

    QuadColor withMultipliedAlpha(float multiply) const {
      QuadColor result;
      result.corners[kTopLeft] = interpolateColor(corners[kTopLeft] & 0xffffff, corners[kTopLeft], multiply);
      result.corners[kTopRight] = interpolateColor(corners[kTopRight] & 0xffffff,
                                                   corners[kTopRight], multiply);
      result.corners[kBottomLeft] = interpolateColor(corners[kBottomLeft] & 0xffffff,
                                                     corners[kBottomLeft], multiply);
      result.corners[kBottomRight] = interpolateColor(corners[kBottomRight] & 0xffffff,
                                                      corners[kBottomRight], multiply);
      result.hdr[kTopLeft] = hdr[kTopLeft];
      result.hdr[kTopRight] = hdr[kTopRight];
      result.hdr[kBottomLeft] = hdr[kBottomLeft];
      result.hdr[kBottomRight] = hdr[kBottomRight];
      return result;
    }

    QuadColor withMultipliedHdr(float multiply) const {
      QuadColor result = *this;
      result.hdr[kTopLeft] = multiply * hdr[kTopLeft];
      result.hdr[kTopRight] = multiply * hdr[kTopRight];
      result.hdr[kBottomLeft] = multiply * hdr[kBottomLeft];
      result.hdr[kBottomRight] = multiply * hdr[kBottomRight];
      return result;
    }

    QuadColor interpolate(const QuadColor& other, float t) const {
      QuadColor result;
      result.corners[kTopLeft] = interpolateColor(corners[kTopLeft], other.corners[kTopLeft], t);
      result.corners[kTopRight] = interpolateColor(corners[kTopRight], other.corners[kTopRight], t);
      result.corners[kBottomLeft] = interpolateColor(corners[kBottomLeft], other.corners[kBottomLeft], t);
      result.corners[kBottomRight] = interpolateColor(corners[kBottomRight],
                                                      other.corners[kBottomRight], t);
      result.hdr[kTopLeft] = interpolateHdr(hdr[kTopLeft], other.hdr[kTopLeft], t);
      result.hdr[kTopRight] = interpolateHdr(hdr[kTopRight], other.hdr[kTopRight], t);
      result.hdr[kBottomLeft] = interpolateHdr(hdr[kBottomLeft], other.hdr[kBottomLeft], t);
      result.hdr[kBottomRight] = interpolateHdr(hdr[kBottomRight], other.hdr[kBottomRight], t);
      return result;
    }

    QuadColor sampleQuad(float x, float y, float width, float height) const {
      QuadColor result;
      result.corners[kTopLeft] = sampleColor(x, y);
      result.corners[kBottomLeft] = sampleColor(x, y + height);
      result.corners[kTopRight] = sampleColor(x + width, y);
      result.corners[kBottomRight] = sampleColor(x + width, y + height);
      result.hdr[kTopLeft] = sampleHdr(x, y);
      result.hdr[kBottomLeft] = sampleHdr(x, y + height);
      result.hdr[kTopRight] = sampleHdr(x + width, y);
      result.hdr[kBottomRight] = sampleHdr(x + width, y + height);
      return result;
    }

    QuadColor clipRight(float t) const {
      QuadColor result;
      result.corners[kTopLeft] = corners[kTopLeft];
      result.corners[kBottomLeft] = corners[kBottomLeft];
      result.corners[kTopRight] = interpolateColor(corners[kTopLeft], corners[kTopRight], t);
      result.corners[kBottomRight] = interpolateColor(corners[kBottomLeft], corners[kBottomRight], t);
      result.hdr[kTopLeft] = hdr[kTopLeft];
      result.hdr[kBottomLeft] = hdr[kBottomLeft];
      result.hdr[kTopRight] = interpolateHdr(hdr[kTopLeft], hdr[kTopRight], t);
      result.hdr[kBottomRight] = interpolateHdr(hdr[kBottomLeft], hdr[kBottomRight], t);
      return result;
    }

    unsigned int sampleColor(float x, float y) const {
      unsigned int top = interpolateColor(corners[kTopLeft], corners[kTopRight], x);
      unsigned int bottom = interpolateColor(corners[kBottomLeft], corners[kBottomRight], x);
      return interpolateColor(top, bottom, y);
    }

    float sampleAlpha(float x, float y) const {
      unsigned int top = interpolateColor(corners[kTopLeft], corners[kTopRight], x);
      unsigned int bottom = interpolateColor(corners[kBottomLeft], corners[kBottomRight], x);
      return (0xff & (interpolateColor(top, bottom, y) >> (3 * Color::kBitsPerColor))) * (1.0f / 0xff);
    }

    float sampleHdr(float x, float y) const {
      float top = interpolateHdr(hdr[kTopLeft], hdr[kTopRight], x);
      float bottom = interpolateHdr(hdr[kBottomLeft], hdr[kBottomRight], x);
      return interpolateHdr(top, bottom, y);
    }
  };
}