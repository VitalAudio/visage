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

#include "palette.h"

#include "gradient.h"
#include "theme.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace visage {
  void Palette::initWithDefaults() {
    value_map_.clear();
    int num_value_ids = theme::ValueId::numValueIds();
    for (int i = 0; i < num_value_ids; ++i) {
      theme::ValueId value_id(i);
      value_map_[{}][value_id] = theme::ValueId::defaultValue(value_id);
    }

    color_map_.clear();

    int num_ids = theme::ColorId::numColorIds();
    std::map<unsigned int, int> existing_colors;
    for (int i = 0; i < num_ids; ++i) {
      theme::ColorId color_id(i);
      unsigned int default_color = theme::ColorId::defaultColor(color_id);
      if (existing_colors.count(default_color) == 0)
        existing_colors[default_color] = existing_colors.size();

      color_map_[{}][color_id] = existing_colors[default_color];
    }

    colors_.clear();

    for (const auto& color : existing_colors) {
      while (colors_.size() <= color.second)
        colors_.emplace_back();

      colors_[color.second] = Brush::solid(color.first);
    }

    sortColors();
  }

  void Palette::sortColors() {
    std::vector<std::pair<Brush, int>> sorted;
    sorted.reserve(colors_.size());
    for (auto& color : colors_)
      sorted.emplace_back(color, sorted.size());

    auto color_sort = [](const std::pair<Brush, int>& one, const std::pair<Brush, int>& two) {
      static constexpr float kSaturationCutoff = 0.2f;
      Color color1 = one.first.gradient().sample(0.0f);
      Color color2 = two.first.gradient().sample(0.0f);
      float saturation1 = color1.saturation();
      float saturation2 = color2.saturation();
      if (saturation1 >= kSaturationCutoff && saturation2 >= kSaturationCutoff)
        return color1.hue() < color2.hue();
      if (saturation1 < kSaturationCutoff && saturation2 < kSaturationCutoff)
        return color1.value() < color2.value();
      return saturation2 < kSaturationCutoff;
    };
    std::sort(sorted.begin(), sorted.end(), color_sort);

    std::vector<int> color_movement(colors_.size());
    for (int i = 0; i < colors_.size(); ++i) {
      colors_[i] = sorted[i].first;
      color_movement[sorted[i].second] = i;
    }

    for (auto&& override_map : color_map_) {
      for (auto&& mapped : override_map.second) {
        if (mapped.second >= 0)
          mapped.second = color_movement[mapped.second];
      }
    }
  }

  std::map<std::string, std::vector<theme::ColorId>> Palette::colorIdList(theme::OverrideId override_id) {
    std::map<std::string, std::vector<theme::ColorId>> results;
    for (const auto& color : color_map_[override_id])
      results[theme::ColorId::groupName(color.first)].push_back(color.first);
    return results;
  }

  std::map<std::string, std::vector<theme::ValueId>> Palette::valueIdList(theme::OverrideId override_id) {
    std::map<std::string, std::vector<theme::ValueId>> results;
    for (const auto& value : value_map_[override_id])
      results[theme::ValueId::groupName(value.first)].push_back(value.first);
    return results;
  }

  void Palette::removeColor(int index) {
    colors_.erase(colors_.begin() + index);

    for (auto& override_map : color_map_) {
      for (auto& color : override_map.second) {
        if (color.second == index)
          color.second = -1;
        else if (color.second > index)
          color.second--;
      }
    }
  }

  std::string Palette::encode() const {
    std::ostringstream stream;
    stream << std::setprecision(std::numeric_limits<float>::max_digits10);

    for (const auto& override_group : color_map_) {
      stream << theme::OverrideId::name(override_group.first) << std::endl;
      for (const auto& color_assignment : override_group.second) {
        stream << theme::ColorId::name(color_assignment.first) << kEncodingSeparator
               << color_assignment.second << std::endl;
      }
      stream << std::endl;
    }
    stream << std::endl;

    for (const auto& override_group : value_map_) {
      stream << theme::OverrideId::name(override_group.first) << std::endl;
      for (const auto& value_assignment : override_group.second) {
        stream << theme::ValueId::name(value_assignment.first) << kEncodingSeparator
               << value_assignment.second << std::endl;
      }
      stream << std::endl;
    }
    stream << std::endl;

    stream << colors_.size() << std::endl;
    for (auto& color : colors_)
      color.encode(stream);
    return stream.str();
  }

  void Palette::decode(const std::string& data) {
    std::istringstream stream(data);
    std::map<std::string, theme::OverrideId> override_name_map = theme::OverrideId::nameIdMap();
    std::map<std::string, theme::ColorId> color_name_map = theme::ColorId::nameIdMap();
    std::map<std::string, theme::ValueId> value_name_map = theme::ValueId::nameIdMap();

    color_map_.clear();
    std::string override_name;
    std::getline(stream, override_name);
    while (!override_name.empty()) {
      theme::OverrideId override_id(theme::OverrideId::kInvalidId);
      if (override_name_map.count(override_name))
        override_id = override_name_map[override_name];

      std::string color_mapping;
      std::getline(stream, color_mapping);
      while (!color_mapping.empty()) {
        std::size_t split_position = color_mapping.find(kEncodingSeparator);
        if (split_position != std::string::npos) {
          std::string key = color_mapping.substr(0, split_position);

          try {
            int color_index = std::stoi(color_mapping.substr(split_position + 1));
            color_map_[override_id][color_name_map[key]] = color_index;
          }
          catch (...) {
          }
        }

        std::getline(stream, color_mapping);
      }

      std::getline(stream, override_name);
    }

    value_map_.clear();
    std::getline(stream, override_name);
    while (!override_name.empty()) {
      theme::OverrideId override_id(theme::OverrideId::kInvalidId);
      if (override_name_map.count(override_name))
        override_id = override_name_map[override_name];

      std::string value_mapping;
      std::getline(stream, value_mapping);
      while (!value_mapping.empty()) {
        std::size_t split_position = value_mapping.find(kEncodingSeparator);
        if (split_position != std::string::npos) {
          std::string key = value_mapping.substr(0, split_position);
          float value = 0.0f;
          try {
            value = std::stof(value_mapping.substr(split_position + 1));
          }
          catch (...) {
          }
          value_map_[override_id][value_name_map[key]] = value;
        }

        std::getline(stream, value_mapping);
      }

      std::getline(stream, override_name);
    }

    std::string line;
    std::getline(stream, line);
    int num_colors = 0;
    try {
      num_colors = std::stoi(line);
    }
    catch (...) {
    }

    colors_.clear();
    colors_.reserve(num_colors);

    for (int i = 0; i < num_colors; ++i) {
      colors_.emplace_back();
      colors_[i].decode(stream);
    }
  }
}