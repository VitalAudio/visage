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

#include "color.h"

#include <iosfwd>
#include <map>
#include <vector>

namespace visage {
  class Palette {
  public:
    static constexpr int kMaxColors = 256;
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
      EditColor(const Color& color) {
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

    Palette() : colors_() { }

    int numColors() const { return num_colors_; }

    EditColor colorIndex(int index) const { return colors_[index]; }

    std::vector<EditColor> colorList() const {
      std::vector<EditColor> results;
      results.reserve(num_colors_);
      for (int i = 0; i < num_colors_; ++i)
        results.push_back(colors_[i]);
      return results;
    }

    void initWithDefaults();
    void sortColors();

    std::map<std::string, std::vector<unsigned int>> colorIdList(int override_id);
    std::map<std::string, std::vector<unsigned int>> valueIdList(int override_id);

    void setEditColor(int index, const EditColor& color) {
      colors_[index] = color;
      computed_colors_[index] = colors_[index].toQuadColor();
    }

    void setColorIndex(int index, int corner, const Color& color) {
      colors_[index].colors[corner] = color;
      computed_colors_[index] = colors_[index].toQuadColor();
    }

    void toggleColorIndexStyle(int index) {
      colors_[index].toggleStyle();
      computed_colors_[index] = colors_[index].toQuadColor();
    }

    bool color(unsigned int override_id, int color_id, QuadColor& color) {
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

    void setColorMap(unsigned int override_id, int color_id, int index) {
      color_map_[override_id][color_id] = index;
    }

    void setColor(unsigned int override_id, int color_id, Color color) {
      int index = addColor(color);
      color_map_[override_id][color_id] = index;
    }

    void setValue(unsigned int override_id, int value_id, float value) {
      value_map_[override_id][value_id] = value;
    }

    void removeValue(unsigned int override_id, int value_id) {
      if (value_map_[override_id].count(value_id))
        value_map_[override_id].erase(value_id);
    }

    void setName(std::string name) { name_ = std::move(name); }

    std::string name() { return name_; }

    void removeValue(int value_id) { value_map_.erase(value_id); }

    int colorMap(unsigned int override_id, unsigned int color_id) {
      return color_map_[override_id][color_id];
    }

    bool value(unsigned int override_id, unsigned int value_id, float& result) {
      if (value_map_[override_id].count(value_id) == 0)
        value_map_[override_id][value_id] = kNotSetValue;

      result = value_map_[override_id][value_id];
      return result != kNotSetValue;
    }

    int addColor(Color color = 0xffff00ff) { return addColorInternal(color); }

    int addVerticalGradient(const VerticalGradient& gradient) {
      return addVerticalGradientInternal(gradient);
    }

    void clear() {
      color_map_.clear();
      value_map_.clear();
      num_colors_ = 0;
    }

    void removeColor(int index);

    std::string encode() const;
    void decode(const std::string& data);

  private:
    int addColorInternal(Color color = 0xffff00ff) {
      colors_[num_colors_] = Color(color);
      computed_colors_[num_colors_] = color;
      return num_colors_++;
    }

    int addVerticalGradientInternal(const VerticalGradient& gradient) {
      EditColor color = gradient.top();
      color.style = EditColor::kVertical;
      color.colors[1] = gradient.bottom();
      color.colors[2] = gradient.bottom();
      color.colors[3] = gradient.bottom();
      colors_[num_colors_] = color;
      computed_colors_[num_colors_] = color.toQuadColor();
      return num_colors_++;
    }

    int num_colors_ = 0;
    std::string name_;
    EditColor colors_[kMaxColors];
    QuadColor computed_colors_[kMaxColors];
    std::map<unsigned int, std::map<unsigned int, int>> color_map_;
    std::map<unsigned int, std::map<unsigned int, float>> value_map_;
  };
}