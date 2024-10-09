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

#include "palette.h"

#include "theme.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace visage {
  std::string Palette::EditColor::encode() const {
    std::ostringstream stream;
    encode(stream);
    return stream.str();
  }

  void Palette::EditColor::encode(std::ostringstream& stream) const {
    for (const auto& color : colors)
      color.encode(stream);
    stream << static_cast<int>(style) << std::endl;
  }

  void Palette::EditColor::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void Palette::EditColor::decode(std::istringstream& stream) {
    std::string line;

    for (auto& color : colors)
      color.decode(stream);

    int int_style = 0;
    stream >> int_style;
    style = static_cast<Style>(int_style);
  }

  void Palette::initWithDefaults() {
    value_map_.clear();
    int num_value_ids = theme::ValueId::numValueIds();
    for (int i = 0; i < num_value_ids; ++i)
      value_map_[0][i] = theme::ValueId::defaultValue(i);

    color_map_.clear();

    int num_ids = theme::ColorId::numColorIds();
    std::map<unsigned int, int> existing_colors;
    for (int i = 0; i < num_ids; ++i) {
      unsigned int default_color = theme::ColorId::defaultColor(i);
      if (existing_colors.count(default_color) == 0)
        existing_colors[default_color] = existing_colors.size();

      color_map_[0][i] = existing_colors[default_color];
    }

    for (const auto& color : existing_colors) {
      colors_[color.second] = Color(color.first);
      computed_colors_[color.second] = colors_[color.second].toQuadColor();
    }
    num_colors_ = existing_colors.size();

    sortColors();
  }

  void Palette::sortColors() {
    std::vector<std::pair<EditColor, int>> sorted(num_colors_);
    for (int i = 0; i < num_colors_; ++i)
      sorted[i] = { colors_[i], i };

    auto color_sort = [](const std::pair<EditColor, int>& one, const std::pair<EditColor, int>& two) {
      static constexpr float kSaturationCutoff = 0.2f;
      float saturation1 = one.first.colors[0].saturation();
      float saturation2 = two.first.colors[0].saturation();
      if (saturation1 >= kSaturationCutoff && saturation2 >= kSaturationCutoff)
        return one.first.colors[0].hue() < two.first.colors[0].hue();
      if (saturation1 < kSaturationCutoff && saturation2 < kSaturationCutoff)
        return one.first.colors[0].value() < two.first.colors[0].value();
      return saturation2 < kSaturationCutoff;
    };
    std::sort(sorted.begin(), sorted.end(), color_sort);

    std::vector<int> color_movement(num_colors_);
    for (int i = 0; i < num_colors_; ++i) {
      colors_[i] = sorted[i].first;
      computed_colors_[i] = sorted[i].first.toQuadColor();
      color_movement[sorted[i].second] = i;
    }

    for (auto&& override_map : color_map_) {
      for (auto&& mapped : override_map.second) {
        if (mapped.second >= 0)
          mapped.second = color_movement[mapped.second];
      }
    }
  }

  std::map<std::string, std::vector<unsigned int>> Palette::colorIdList(int override_id) {
    std::map<std::string, std::vector<unsigned int>> results;
    for (const auto& color : color_map_[override_id])
      results[theme::ColorId::groupName(color.first)].push_back(color.first);
    return results;
  }

  std::map<std::string, std::vector<unsigned int>> Palette::valueIdList(int override_id) {
    std::map<std::string, std::vector<unsigned int>> results;
    for (const auto& value : value_map_[override_id])
      results[theme::ValueId::groupName(value.first)].push_back(value.first);
    return results;
  }

  void Palette::removeColor(int index) {
    num_colors_--;
    for (int i = index; i < num_colors_; ++i) {
      colors_[i] = colors_[i + 1];
      computed_colors_[i] = computed_colors_[i + 1];
    }

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

    stream << num_colors_ << std::endl;
    for (int i = 0; i < num_colors_; ++i)
      colors_[i].encode(stream);
    return stream.str();
  }

  void Palette::decode(const std::string& data) {
    std::istringstream stream(data);
    std::map<std::string, unsigned int> override_name_map = theme::OverrideId::nameIdMap();
    std::map<std::string, unsigned int> color_name_map = theme::ColorId::nameIdMap();
    std::map<std::string, unsigned int> value_name_map = theme::ValueId::nameIdMap();

    color_map_.clear();
    std::string override_name;
    std::getline(stream, override_name);
    while (!override_name.empty()) {
      unsigned int override_id = -1;
      if (override_name_map.count(override_name))
        override_id = override_name_map[override_name];

      std::string color_mapping;
      std::getline(stream, color_mapping);
      while (!color_mapping.empty()) {
        std::size_t split_position = color_mapping.find(kEncodingSeparator);
        if (split_position != std::string::npos) {
          std::string key = color_mapping.substr(0, split_position);
          int color_index = std::stoi(color_mapping.substr(split_position + 1));
          color_map_[override_id][color_name_map[key]] = color_index;
        }

        std::getline(stream, color_mapping);
      }

      std::getline(stream, override_name);
    }

    value_map_.clear();
    std::getline(stream, override_name);
    while (!override_name.empty()) {
      unsigned int override_id = -1;
      if (override_name_map.count(override_name))
        override_id = override_name_map[override_name];

      std::string value_mapping;
      std::getline(stream, value_mapping);
      while (!value_mapping.empty()) {
        std::size_t split_position = value_mapping.find(kEncodingSeparator);
        if (split_position != std::string::npos) {
          std::string key = value_mapping.substr(0, split_position);
          int value = std::stof(value_mapping.substr(split_position + 1));
          value_map_[override_id][value_name_map[key]] = value;
        }

        std::getline(stream, value_mapping);
      }

      std::getline(stream, override_name);
    }

    std::string line;
    std::getline(stream, line);
    num_colors_ = std::stoi(line);
    for (int i = 0; i < num_colors_; ++i)
      colors_[i].decode(stream);

    for (int i = 0; i < num_colors_; ++i)
      computed_colors_[i] = colors_[i].toQuadColor();
  }
}