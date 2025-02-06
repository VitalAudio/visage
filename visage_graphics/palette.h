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

#include "color.h"

#include <iosfwd>
#include <map>
#include <vector>

namespace visage {
  class Palette {
  public:
    static constexpr int kInvalidId = -2;
    static constexpr unsigned int kInvalidColor = 0xffff00ff;
    static constexpr float kNotSetValue = -99999.0f;
    static constexpr int kNotSetId = -1;
    static constexpr char kEncodingSeparator = '@';

    struct EditColor {
      enum Style {
        kSingle,
        kHorizontal,
        kVertical,
        kFourCorners,
        kNumStyles
      };

      EditColor() = default;
      explicit EditColor(const Color& color) {
        for (auto& i : colors)
          i = color;
      }

      Color colors[4];
      Style style = kSingle;

      void toggleStyle() { style = static_cast<Style>((style + 1) % kNumStyles); }

      int numActiveCorners() const {
        switch (style) {
        case kVertical:
        case kHorizontal: return 2;
        case kFourCorners: return 4;
        default: return 1;
        }
      }

      QuadColor toQuadColor() const {
        if (style == kSingle)
          return colors[QuadColor::kTopLeft];
        if (style == kHorizontal) {
          unsigned int left = colors[QuadColor::kTopLeft].toABGR();
          unsigned int right = colors[QuadColor::kTopRight].toABGR();
          float left_hdr = colors[QuadColor::kTopLeft].hdr();
          float right_hdr = colors[QuadColor::kTopRight].hdr();
          QuadColor result;
          result.corners[QuadColor::kTopLeft] = left;
          result.corners[QuadColor::kTopRight] = right;
          result.corners[QuadColor::kBottomLeft] = left;
          result.corners[QuadColor::kBottomRight] = right;
          result.hdr[QuadColor::kTopLeft] = left_hdr;
          result.hdr[QuadColor::kTopRight] = right_hdr;
          result.hdr[QuadColor::kBottomLeft] = left_hdr;
          result.hdr[QuadColor::kBottomRight] = right_hdr;
          return result;
        }
        else if (style == kVertical) {
          unsigned int top = colors[QuadColor::kTopLeft].toABGR();
          unsigned int bottom = colors[QuadColor::kTopRight].toABGR();
          float top_hdr = colors[QuadColor::kTopLeft].hdr();
          float bottom_hdr = colors[QuadColor::kTopRight].hdr();
          QuadColor result;
          result.corners[QuadColor::kTopLeft] = top;
          result.corners[QuadColor::kTopRight] = top;
          result.corners[QuadColor::kBottomLeft] = bottom;
          result.corners[QuadColor::kBottomRight] = bottom;
          result.hdr[QuadColor::kTopLeft] = top_hdr;
          result.hdr[QuadColor::kTopRight] = top_hdr;
          result.hdr[QuadColor::kBottomLeft] = bottom_hdr;
          result.hdr[QuadColor::kBottomRight] = bottom_hdr;
          return result;
        }

        QuadColor result;
        for (int i = 0; i < QuadColor::kNumCorners; ++i) {
          result.corners[i] = colors[i].toABGR();
          result.hdr[i] = colors[i].hdr();
        }

        return result;
      }

      std::string encode() const;
      void encode(std::ostringstream& stream) const;
      void decode(const std::string& data);
      void decode(std::istringstream& stream);
    };

    Palette() = default;

    int numColors() const { return colors_.size(); }

    EditColor colorIndex(int index) const {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      return colors_[index];
    }

    std::vector<EditColor> colorList() const { return colors_; }

    void initWithDefaults();
    void sortColors();

    std::map<std::string, std::vector<theme::ColorId>> colorIdList(theme::OverrideId override_id);
    std::map<std::string, std::vector<theme::ValueId>> valueIdList(theme::OverrideId override_id);

    void setEditColor(int index, const EditColor& color) {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      colors_[index] = color;
      computed_colors_[index] = colors_[index].toQuadColor();
    }

    void setColorIndex(int index, int corner, const Color& color) {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      colors_[index].colors[corner] = color;
      computed_colors_[index] = colors_[index].toQuadColor();
    }

    void toggleColorIndexStyle(int index) {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      colors_[index].toggleStyle();
      computed_colors_[index] = colors_[index].toQuadColor();
    }

    bool color(theme::OverrideId override_id, theme::ColorId color_id, QuadColor& color) {
      if (color_map_[override_id].count(color_id) == 0)
        color_map_[override_id][color_id] = kNotSetId;

      if (color_map_[override_id][color_id] == kNotSetId)
        return false;

      if (color_map_[override_id][color_id] == kInvalidId)
        color = kInvalidColor;
      else
        color = computed_colors_[color_map_[override_id][color_id]];
      return true;
    }

    void setColorMap(theme::OverrideId override_id, theme::ColorId color_id, int index) {
      color_map_[override_id][color_id] = index;
    }

    void setColor(theme::OverrideId override_id, theme::ColorId color_id, const Color& color) {
      int index = addColor(color);
      color_map_[override_id][color_id] = index;
    }

    void setColor(theme::ColorId color_id, const Color& color) { setColor({}, color_id, color); }

    void setValue(theme::OverrideId override_id, theme::ValueId value_id, float value) {
      value_map_[override_id][value_id] = value;
    }

    void setValue(theme::ValueId value_id, float value) { setValue({}, value_id, value); }

    void removeValue(theme::OverrideId override_id, theme::ValueId value_id) {
      if (value_map_[override_id].count(value_id))
        value_map_[override_id].erase(value_id);
    }

    void removeValue(theme::ValueId value_id) { removeValue({}, value_id); }

    int colorMap(theme::OverrideId override_id, theme::ColorId color_id) {
      return color_map_[override_id][color_id];
    }

    bool value(theme::OverrideId override_id, theme::ValueId value_id, float& result) {
      if (value_map_[override_id].count(value_id) == 0)
        value_map_[override_id][value_id] = kNotSetValue;

      result = value_map_[override_id][value_id];
      return result != kNotSetValue;
    }

    int addColor(const Color& color = 0xffff00ff) { return addColorInternal(color); }

    int addVerticalGradient(const VerticalGradient& gradient) {
      return addVerticalGradientInternal(gradient);
    }

    void clear() {
      color_map_.clear();
      value_map_.clear();
      colors_.clear();
      computed_colors_.clear();
    }

    void removeColor(int index);

    std::string encode() const;
    void decode(const std::string& data);

  private:
    int addColorInternal(const Color& color = 0xffff00ff) {
      colors_.emplace_back(color);
      computed_colors_.push_back(color);
      return colors_.size() - 1;
    }

    int addVerticalGradientInternal(const VerticalGradient& gradient) {
      EditColor color(gradient.top());
      color.style = EditColor::kVertical;
      color.colors[1] = gradient.bottom();
      color.colors[2] = gradient.bottom();
      color.colors[3] = gradient.bottom();

      colors_.emplace_back(color);
      computed_colors_.push_back(color.toQuadColor());
      return colors_.size() - 1;
    }

    std::vector<EditColor> colors_;
    std::vector<QuadColor> computed_colors_;
    std::map<theme::OverrideId, std::map<theme::ColorId, int>> color_map_;
    std::map<theme::OverrideId, std::map<theme::ValueId, float>> value_map_;
  };
}