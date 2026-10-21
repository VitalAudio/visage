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
#include "gradient.h"
#include "theme.h"

#include <map>
#include <set>
#include <vector>

namespace visage {
  class Palette {
  public:
    static constexpr int kInvalidId = -2;
    static constexpr unsigned int kInvalidColor = 0xffff00ff;
    static constexpr float kNotSetValue = -99999.0f;
    static constexpr int kNotSetId = -1;
    static constexpr char kEncodingSeparator = '@';

    Palette() = default;

    int numColors() const { return colors_.size(); }

    Brush colorIndex(int index) const {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      return colors_[index];
    }

    const std::vector<Brush>& colorList() const { return colors_; }

    void initWithDefaults();
    void sortColors();

    std::map<std::string, std::vector<theme::ColorId>> colorIdList(theme::OverrideId override_id) const;
    std::map<std::string, std::vector<theme::ValueId>> valueIdList(theme::OverrideId override_id) const;

    void setEditColor(int index, const Brush& color) {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      colors_[index] = color;
    }

    void setColorIndexFrom(int index, const Color& color) {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      colors_[index].gradient().setColor(0, color);
    }

    void setColorIndexTo(int index, const Color& color) {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      colors_[index].gradient().setColor(1, color);
    }

    void toggleColorIndexStyle(int index) {
      VISAGE_ASSERT(index >= 0 && index < colors_.size());
      if (colors_[index].position().shape == GradientPosition::InterpolationShape::Solid) {
        colors_[index].gradient().setResolution(2);
        colors_[index].position().shape = GradientPosition::InterpolationShape::Vertical;
      }
      else if (colors_[index].position().shape == GradientPosition::InterpolationShape::Vertical) {
        colors_[index].gradient().setResolution(2);
        colors_[index].position().shape = GradientPosition::InterpolationShape::Horizontal;
      }
      else {
        colors_[index].gradient().setResolution(1);
        colors_[index].position().shape = GradientPosition::InterpolationShape::Solid;
      }
    }

    bool color(theme::OverrideId override_id, theme::ColorId color_id, Brush& color) const {
      int index = colorMap(override_id, color_id);
      if (index == kNotSetId) {
        if (known_color_ids_[override_id].insert(color_id).second)
          color_id_lists_.erase(override_id);
        return false;
      }

      if (index < 0 || index >= colors_.size())
        color = Brush::solid(kInvalidColor);
      else
        color = colors_[index];
      return true;
    }

    void setColorMap(theme::OverrideId override_id, theme::ColorId color_id, int index) {
      if (color_map_[override_id].insert_or_assign(color_id, index).second)
        color_id_lists_.erase(override_id);
    }

    void setColor(theme::OverrideId override_id, theme::ColorId color_id, const Color& color) {
      setColorMap(override_id, color_id, addColor(color));
    }

    void setColor(theme::OverrideId override_id, theme::ColorId color_id, const Brush& color) {
      setColorMap(override_id, color_id, addBrush(color));
    }

    void setColor(theme::ColorId color_id, const Color& color) { setColor({}, color_id, color); }
    void setColor(theme::ColorId color_id, const Brush& color) { setColor({}, color_id, color); }

    void setValue(theme::OverrideId override_id, theme::ValueId value_id, float value) {
      if (value_map_[override_id].insert_or_assign(value_id, value).second)
        value_id_lists_.erase(override_id);
    }

    void setValue(theme::ValueId value_id, float value) { setValue({}, value_id, value); }

    void removeValue(theme::OverrideId override_id, theme::ValueId value_id) {
      auto values = value_map_.find(override_id);
      if (values == value_map_.end() || values->second.erase(value_id) == 0)
        return;

      known_value_ids_[override_id].insert(value_id);
      if (values->second.empty())
        value_map_.erase(values);
    }

    void removeValue(theme::ValueId value_id) { removeValue({}, value_id); }

    int colorMap(theme::OverrideId override_id, theme::ColorId color_id) const {
      auto colors = color_map_.find(override_id);
      if (colors == color_map_.end())
        return kNotSetId;
      auto index = colors->second.find(color_id);
      return index == colors->second.end() ? kNotSetId : index->second;
    }

    bool value(theme::OverrideId override_id, theme::ValueId value_id, float& result) const {
      result = kNotSetValue;
      auto values = value_map_.find(override_id);
      if (values != value_map_.end()) {
        auto value = values->second.find(value_id);
        if (value != values->second.end())
          result = value->second;
      }
      if (result == kNotSetValue && known_value_ids_[override_id].insert(value_id).second)
        value_id_lists_.erase(override_id);
      return result != kNotSetValue;
    }

    int addColor(const Color& color = 0xffff00ff) {
      colors_.emplace_back(Brush::solid(color));
      return colors_.size() - 1;
    }

    int addBrush(const Brush& color) {
      colors_.emplace_back(color);
      return colors_.size() - 1;
    }

    void clear() {
      color_map_.clear();
      value_map_.clear();
      colors_.clear();
      known_color_ids_.clear();
      known_value_ids_.clear();
      clearIdLists();
    }

    void removeColor(int index);

    std::string encode() const;
    void decode(const std::string& data);

  private:
    void clearIdLists() {
      color_id_lists_.clear();
      value_id_lists_.clear();
    }

    std::vector<Brush> colors_;
    std::map<theme::OverrideId, std::map<theme::ColorId, int>> color_map_;
    std::map<theme::OverrideId, std::map<theme::ValueId, float>> value_map_;
    mutable std::map<theme::OverrideId, std::set<theme::ColorId>> known_color_ids_;
    mutable std::map<theme::OverrideId, std::set<theme::ValueId>> known_value_ids_;
    mutable std::map<theme::OverrideId, std::map<std::string, std::vector<theme::ColorId>>> color_id_lists_;
    mutable std::map<theme::OverrideId, std::map<std::string, std::vector<theme::ValueId>>> value_id_lists_;
  };
}