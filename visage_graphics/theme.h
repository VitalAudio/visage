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

#include "visage_utils/defines.h"

#include <map>
#include <string>

#define THEME_COLOR(color, default_color) \
  const unsigned int k##color = theme::ColorId::nextId(#color, __FILE__, default_color)

#define THEME_DEFINE_COLOR(color) static const unsigned int k##color

#define THEME_IMPLEMENT_COLOR(container, color, default_color) \
  const unsigned int container::k##color = theme::ColorId::nextId(#color, __FILE__, default_color)

#define THEME_VALUE(value, default_value, scale_type, round_to_pixel)                   \
  const unsigned int k##value = theme::ValueId::nextId(#value, __FILE__, default_value, \
                                                       theme::ValueId::k##scale_type, round_to_pixel)

#define THEME_DEFINE_VALUE(value) static const unsigned int k##value

#define THEME_IMPLEMENT_VALUE(container, value, default_value, scale_type, round_to_pixel)         \
  const unsigned int container::k##value = theme::ValueId::nextId(#value, __FILE__, default_value, \
                                                                  theme::ValueId::k##scale_type,   \
                                                                  round_to_pixel)

#define THEME_PALETTE_OVERRIDE(override_name) \
  const unsigned int k##override_name = theme::OverrideId::nextId(#override_name)

namespace theme {
  static std::string nameFromPath(const std::string& file_path) {
    size_t start = file_path.find_last_of("\\/");
    size_t end = file_path.find_last_of('.');
    if (start == std::string::npos)
      start = 0;
    if (end == std::string::npos)
      end = file_path.length();
    return file_path.substr(start + 1, end - start - 1);
  }

  class ColorId {
  public:
    struct ColorIdInfo {
      std::string name;
      std::string group;
      unsigned int default_color = 0;
    };

    static ColorId* instance() {
      static ColorId instance;
      return &instance;
    }

    static unsigned int nextId(std::string name, const std::string& file_path, unsigned int default_color) {
      ColorId* id = instance();
      id->info_map_[id->next_id_] = { std::move(name), nameFromPath(file_path), default_color };
      return id->next_id_++;
    }

    static inline unsigned int defaultColor(unsigned int color_id) {
      return instance()->info_map_[color_id].default_color;
    }

    static inline const std::string& groupName(unsigned int color_id) {
      return instance()->info_map_[color_id].group;
    }

    static inline const std::string& name(unsigned int color_id) {
      return instance()->info_map_[color_id].name;
    }

    static std::map<std::string, unsigned int> nameIdMap() {
      std::map<std::string, unsigned int> result;
      for (const auto& assignment : instance()->info_map_)
        result[assignment.second.name] = assignment.first;

      return result;
    }

    static int numColorIds() { return instance()->next_id_; }

  private:
    ColorId() = default;

    unsigned int next_id_ = 0;
    std::map<unsigned int, ColorIdInfo> info_map_;
  };

  class ValueId {
  public:
    enum ScaleType {
      kConstant,
      kScaledDpi,
      kScaledWidth,
      kScaledHeight,
      kNumScaleTypes
    };

    struct ValueIdInfo {
      std::string name;
      std::string group;
      float default_value = 0.0f;
      ScaleType scale_type = kConstant;
      bool round_to_pixel = false;
    };

    static ValueId* instance() {
      static ValueId instance;
      return &instance;
    }

    static unsigned int nextId(std::string name, const std::string& file_path, float default_value,
                               ScaleType scale_type, bool round_to_pixel) noexcept {
      ValueId* id = instance();
      id->info_map_[id->next_id_] = { std::move(name), nameFromPath(file_path), default_value,
                                      scale_type, round_to_pixel };
      return id->next_id_++;
    }

    static inline float defaultValue(unsigned int value_id) {
      return instance()->info_map_[value_id].default_value;
    }

    static inline ValueIdInfo info(unsigned int value_id) {
      return instance()->info_map_[value_id];
    }

    static inline const std::string& groupName(unsigned int value_id) {
      return instance()->info_map_[value_id].group;
    }

    static inline const std::string& name(unsigned int value_id) {
      return instance()->info_map_[value_id].name;
    }

    static std::map<std::string, unsigned int> nameIdMap() {
      std::map<std::string, unsigned int> result;
      for (const auto& assignment : instance()->info_map_)
        result[assignment.second.name] = assignment.first;

      return result;
    }

    static int numValueIds() { return instance()->next_id_; }

  private:
    ValueId() = default;

    unsigned int next_id_ = 0;
    std::map<unsigned int, ValueIdInfo> info_map_;
  };

  class OverrideId {
  public:
    static OverrideId* instance() {
      static OverrideId instance;
      return &instance;
    }

    static unsigned int nextId(std::string name) noexcept {
      OverrideId* id = instance();
      id->name_map_[id->next_id_] = std::move(name);
      return id->next_id_++;
    }

    static inline const std::string& name(unsigned int value_id) {
      return instance()->name_map_[value_id];
    }

    static inline unsigned int id(const std::string& name) {
      OverrideId* id = instance();
      for (const auto& assignment : id->name_map_) {
        if (assignment.second == name)
          return assignment.first;
      }
      VISAGE_ASSERT(false);
      return -1;
    }

    static std::map<std::string, unsigned int> nameIdMap() {
      std::map<std::string, unsigned int> result;
      for (const auto& assignment : instance()->name_map_)
        result[assignment.second] = assignment.first;

      return result;
    }

    static int numOverrideIds() { return instance()->next_id_; }

  private:
    OverrideId() = default;

    unsigned int next_id_ = 1;
    std::map<unsigned int, std::string> name_map_ = { { 0, "Global" } };
  };
}