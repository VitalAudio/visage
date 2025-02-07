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
    HorizontalGradient(const Color& left, const Color& right) : left_(left), right_(right) { }

    const Color& left() const { return left_; }
    const Color& right() const { return right_; }

  private:
    Color left_;
    Color right_;
  };

  class VerticalGradient {
  public:
    VerticalGradient(const Color& top, const Color& bottom) : top_(top), bottom_(bottom) { }

    const Color& top() const { return top_; }
    const Color& bottom() const { return bottom_; }

  private:
    Color top_;
    Color bottom_;
  };

  struct ColorGradient {
    enum class InterpolationShape {
      None,
      Horizontal,
      Vertical,
      PointsLinear,
      PointsRadial
    };

    unsigned int color_from = 0;
    unsigned int color_to = 0;
    float hdr_from = 1.0f;
    float hdr_to = 1.0f;
    FloatPoint point_from;
    FloatPoint point_to;
    InterpolationShape interpolation_shape = InterpolationShape::None;

    static unsigned int toABGR(unsigned int argb) {
      int abgr = argb;
      char* chars = reinterpret_cast<char*>(&abgr);
      std::swap(chars[0], chars[2]);
      return abgr;
    }

    static unsigned int interpolateColor(unsigned int hex_color1, unsigned int hex_color2, float t) {
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

    ColorGradient() = default;

    ColorGradient(const Color& color) {
      color_from = color_to = color.toABGR();
      hdr_from = hdr_to = color.hdr();
    }

    ColorGradient(const Color& from, const Color& to) {
      color_from = from.toABGR();
      color_to = to.toABGR();
      hdr_from = from.hdr();
      hdr_to = to.hdr();
    }

    ColorGradient(const Color& from, const Color& to, FloatPoint start, FloatPoint end) {
      color_from = from.toABGR();
      color_to = to.toABGR();
      hdr_from = from.hdr();
      hdr_to = to.hdr();
      point_from = start;
      point_to = end;
    }

    ColorGradient(const Color& from, const Color& to, float start_x, float start_y, float end_x,
                  float end_y) {
      color_from = from.toABGR();
      color_to = to.toABGR();
      hdr_from = from.hdr();
      hdr_to = to.hdr();
      point_from = { start_x, start_y };
      point_to = { end_x, end_y };
      interpolation_shape = InterpolationShape::PointsLinear;
    }

    ColorGradient(const HorizontalGradient& gradient) :
        ColorGradient(gradient.left(), gradient.right()) {
      interpolation_shape = InterpolationShape::Horizontal;
    }

    ColorGradient(const VerticalGradient& gradient) :
        ColorGradient(gradient.top(), gradient.bottom()) {
      interpolation_shape = InterpolationShape::Vertical;
    }

    ColorGradient(int argb, float hdr_value = 1.0f) : ColorGradient(Color(argb, hdr_value)) { }

    ColorGradient withAlpha(float alpha) const {
      int alpha_value = alpha * 0xff;
      alpha_value = alpha_value << (Color::kBitsPerColor * 3);
      ColorGradient result = *this;
      result.color_from = (color_from & 0xffffff) | alpha_value;
      result.color_to = (color_to & 0xffffff) | alpha_value;
      return result;
    }

    ColorGradient withMultipliedAlpha(float multiply) const {
      ColorGradient result = *this;
      result.color_from = interpolateColor(color_from & 0xffffff, color_from, multiply);
      result.color_to = interpolateColor(color_to & 0xffffff, color_to, multiply);
      return result;
    }

    ColorGradient withMultipliedHdr(float multiply) const {
      ColorGradient result = *this;
      result.hdr_from *= multiply;
      result.hdr_to *= multiply;
      return result;
    }

    ColorGradient interpolateWith(const ColorGradient& other, float t) const {
      ColorGradient result;
      result.color_from = interpolateColor(color_from, other.color_from, t);
      result.color_to = interpolateColor(color_to, other.color_to, t);
      result.interpolation_shape = interpolation_shape;
      if (interpolation_shape == InterpolationShape::None)
        result.interpolation_shape = other.interpolation_shape;

      result.point_from = point_from + (other.point_from - point_from) * t;
      result.point_to = point_to + (other.point_to - point_to) * t;
      return result;
    }
  };
}