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
#include "visage_utils/string_utils.h"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace visage {
  void Palette::initWithDefaults() {
    version_++;
    known_color_ids_.clear();
    known_value_ids_.clear();
    clearIdLists();
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
    version_++;
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

  const ResolvedPalette* Palette::resolve(const ResolvedPalette* parent, theme::OverrideId override_id) {
    if (parent == nullptr || parent->palette_ != this)
      parent = root();
    if (override_id.isDefault() || override_id == parent->override_id_)
      return parent;

    auto& resolved = resolved_[{ parent, override_id }];
    if (resolved == nullptr) {
      resolved = std::make_unique<ResolvedPalette>();
      resolved->palette_ = this;
      resolved->parent_ = parent;
      resolved->override_id_ = override_id;
    }
    return resolved.get();
  }

  Brush Palette::color(const ResolvedPalette* resolved, theme::ColorId color_id) const {
    const ResolvedPalette& table = refreshed(resolved);
    if (color_id.id >= table.color_indices_.size())
      return Brush::solid(theme::ColorId::defaultColor(color_id));

    if (!table.colors_looked_up_[color_id.id]) {
      table.colors_looked_up_[color_id.id] = true;
      Brush result;
      const ResolvedPalette* level = &table;
      while (level && !color(level->override_id_, color_id, result))
        level = level->parent_;
    }

    int index = table.color_indices_[color_id.id];
    if (index == kNotSetId)
      return Brush::solid(theme::ColorId::defaultColor(color_id));
    if (index < 0 || index >= colors_.size())
      return Brush::solid(kInvalidColor);
    return colors_[index];
  }

  float Palette::value(const ResolvedPalette* resolved, theme::ValueId value_id) const {
    const ResolvedPalette& table = refreshed(resolved);
    if (value_id.id >= table.values_.size())
      return theme::ValueId::defaultValue(value_id);

    if (!table.values_looked_up_[value_id.id]) {
      table.values_looked_up_[value_id.id] = true;
      float result = 0.0f;
      const ResolvedPalette* level = &table;
      while (level && !value(level->override_id_, value_id, result))
        level = level->parent_;
    }

    float result = table.values_[value_id.id];
    return result == kNotSetValue ? theme::ValueId::defaultValue(value_id) : result;
  }

  const ResolvedPalette* Palette::root() const {
    auto& root = resolved_[{ nullptr, theme::OverrideId() }];
    if (root == nullptr) {
      root = std::make_unique<ResolvedPalette>();
      root->palette_ = this;
    }
    return root.get();
  }

  const ResolvedPalette& Palette::refreshed(const ResolvedPalette* resolved) const {
    if (resolved == nullptr || resolved->palette_ != this)
      resolved = root();
    if (resolved->version_ == version_)
      return *resolved;

    if (resolved->parent_) {
      const ResolvedPalette& parent = refreshed(resolved->parent_);
      resolved->color_indices_ = parent.color_indices_;
      resolved->values_ = parent.values_;
    }
    else {
      resolved->color_indices_.assign(theme::ColorId::numColorIds(), kNotSetId);
      resolved->values_.assign(theme::ValueId::numValueIds(), kNotSetValue);
    }

    auto colors = color_map_.find(resolved->override_id_);
    if (colors != color_map_.end()) {
      for (const auto& [color_id, index] : colors->second) {
        if (color_id.id < resolved->color_indices_.size())
          resolved->color_indices_[color_id.id] = index;
      }
    }

    auto values = value_map_.find(resolved->override_id_);
    if (values != value_map_.end()) {
      for (const auto& [value_id, value] : values->second) {
        if (value != kNotSetValue && value_id.id < resolved->values_.size())
          resolved->values_[value_id.id] = value;
      }
    }

    resolved->colors_looked_up_.assign(resolved->color_indices_.size(), false);
    resolved->values_looked_up_.assign(resolved->values_.size(), false);
    resolved->version_ = version_;
    return *resolved;
  }

  std::map<std::string, std::vector<theme::ColorId>> Palette::colorIdList(theme::OverrideId override_id) const {
    auto cached = color_id_lists_.find(override_id);
    if (cached != color_id_lists_.end())
      return cached->second;

    std::set<theme::ColorId> ids;
    auto known = known_color_ids_.find(override_id);
    if (known != known_color_ids_.end())
      ids = known->second;
    auto colors = color_map_.find(override_id);
    if (colors != color_map_.end()) {
      for (const auto& color : colors->second)
        ids.insert(color.first);
    }

    auto& results = color_id_lists_[override_id];
    for (theme::ColorId id : ids)
      results[theme::ColorId::groupName(id)].push_back(id);
    return results;
  }

  std::map<std::string, std::vector<theme::ValueId>> Palette::valueIdList(theme::OverrideId override_id) const {
    auto cached = value_id_lists_.find(override_id);
    if (cached != value_id_lists_.end())
      return cached->second;

    std::set<theme::ValueId> ids;
    auto known = known_value_ids_.find(override_id);
    if (known != known_value_ids_.end())
      ids = known->second;
    auto values = value_map_.find(override_id);
    if (values != value_map_.end()) {
      for (const auto& value : values->second)
        ids.insert(value.first);
    }

    auto& results = value_id_lists_[override_id];
    for (theme::ValueId id : ids)
      results[theme::ValueId::groupName(id)].push_back(id);
    return results;
  }

  void Palette::removeColor(int index) {
    version_++;
    colors_.erase(colors_.begin() + index);

    std::vector<std::pair<theme::OverrideId, theme::ColorId>> unmapped;
    for (auto& override_map : color_map_) {
      for (auto& color : override_map.second) {
        if (color.second == index)
          unmapped.emplace_back(override_map.first, color.first);
        else if (color.second > index)
          color.second--;
      }
    }

    for (const auto& [override_id, color_id] : unmapped)
      removeColorMap(override_id, color_id);
  }

  std::string Palette::encode() const {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
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
    version_++;
    known_color_ids_.clear();
    known_value_ids_.clear();
    clearIdLists();
    std::istringstream stream(data);
    stream.imbue(std::locale::classic());
    std::map<std::string, theme::OverrideId> override_name_map = theme::OverrideId::nameIdMap();
    std::map<std::string, theme::ColorId> color_name_map = theme::ColorId::nameIdMap();
    std::map<std::string, theme::ValueId> value_name_map = theme::ValueId::nameIdMap();

    color_map_.clear();
    std::string override_name;
    std::getline(stream, override_name);
    while (stream && !override_name.empty()) {
      bool known_override = override_name_map.count(override_name);
      theme::OverrideId override_id = override_name_map[override_name];

      std::string color_mapping;
      std::getline(stream, color_mapping);
      while (stream && !color_mapping.empty()) {
        std::size_t split_position = color_mapping.find(kEncodingSeparator);
        std::string key = color_mapping.substr(0, split_position);
        if (known_override && split_position != std::string::npos && color_name_map.count(key)) {
          try {
            int color_index = std::stoi(color_mapping.substr(split_position + 1));
            if (color_index < 0)
              removeColorMap(override_id, color_name_map[key]);
            else
              setColorMap(override_id, color_name_map[key], color_index);
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
    while (stream && !override_name.empty()) {
      bool known_override = override_name_map.count(override_name);
      theme::OverrideId override_id = override_name_map[override_name];

      std::string value_mapping;
      std::getline(stream, value_mapping);
      while (stream && !value_mapping.empty()) {
        std::size_t split_position = value_mapping.find(kEncodingSeparator);
        std::string key = value_mapping.substr(0, split_position);
        if (known_override && split_position != std::string::npos && value_name_map.count(key)) {
          float value = String(value_mapping.substr(split_position + 1)).toFloat();
          if (value == kNotSetValue)
            known_value_ids_[override_id].insert(value_name_map[key]);
          else
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
    colors_.resize(std::max(0, num_colors));
    for (Brush& color : colors_)
      color.decode(stream);
  }
}