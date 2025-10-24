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

#include "visage_utils/defines.h"

#include <map>
#include <string>

#define VISAGE_THEME_COLOR(color, default_color) \
  const ::visage::theme::ColorId color = ::visage::theme::ColorId::nextId(#color, __FILE__, default_color)

#define VISAGE_THEME_DEFINE_COLOR(color) static const ::visage::theme::ColorId color

#define VISAGE_THEME_IMPLEMENT_COLOR(container, color, default_color)                                  \
  const ::visage::theme::ColorId container::color = ::visage::theme::ColorId::nextId(#color, __FILE__, \
                                                                                     default_color)

#define VISAGE_THEME_VALUE(value, default_value) \
  const ::visage::theme::ValueId value = ::visage::theme::ValueId::nextId(#value, __FILE__, default_value)

#define VISAGE_THEME_DEFINE_VALUE(value) static const ::visage::theme::ValueId value

#define VISAGE_THEME_IMPLEMENT_VALUE(container, value, default_value)                                  \
  const ::visage::theme::ValueId container::value = ::visage::theme::ValueId::nextId(#value, __FILE__, \
                                                                                     default_value)

#define VISAGE_THEME_PALETTE_OVERRIDE(override_name) \
  const ::visage::theme::OverrideId override_name = ::visage::theme::OverrideId::nextId(#override_name)

namespace visage::theme {
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
    static constexpr unsigned int kInvalidId = 0xFFFFFFFF;

    struct ColorIdInfo {
      std::string name;
      std::string group;
      unsigned int default_color = 0;
    };

    explicit ColorId(unsigned int id) : id(id) { }
    ColorId() = default;

    unsigned int id = kInvalidId;

    bool operator==(const ColorId& other) const { return id == other.id; }
    bool operator!=(const ColorId& other) const { return id != other.id; }
    bool operator<(const ColorId& other) const { return id < other.id; }

    bool isValid() const { return id != kInvalidId; }

    static ColorId nextId(std::string name, const std::string& file_path, unsigned int default_color) {
      Map* id = Map::instance();
      id->info_map_[ColorId(id->next_id_)] = { std::move(name), nameFromPath(file_path), default_color };
      return ColorId(id->next_id_++);
    }

    static unsigned int defaultColor(ColorId color_id) {
      VISAGE_ASSERT(Map::instance()->info_map_.count(color_id));
      return Map::instance()->info_map_[color_id].default_color;
    }

    static const std::string& groupName(ColorId color_id) {
      VISAGE_ASSERT(Map::instance()->info_map_.count(color_id));
      return Map::instance()->info_map_[color_id].group;
    }

    static const std::string& name(ColorId color_id) {
      VISAGE_ASSERT(Map::instance()->info_map_.count(color_id));
      return Map::instance()->info_map_[color_id].name;
    }

    static std::map<std::string, ColorId> nameIdMap() {
      std::map<std::string, ColorId> result;
      for (const auto& assignment : Map::instance()->info_map_)
        result[assignment.second.name] = assignment.first;

      return result;
    }

    static int numColorIds() { return Map::instance()->next_id_; }

  private:
    struct Map {
      static Map* instance() {
        static Map instance;
        return &instance;
      }

      unsigned int next_id_ = 0;
      std::map<ColorId, ColorIdInfo> info_map_;
    };
  };

  class ValueId {
  public:
    static constexpr unsigned int kInvalidId = 0xFFFFFFFF;

    struct ValueIdInfo {
      std::string name;
      std::string group;
      float default_value = 0.0f;
    };

    explicit ValueId(unsigned int id) : id(id) { }
    ValueId() = default;

    unsigned int id = kInvalidId;

    bool operator==(const ValueId& other) const { return id == other.id; }
    bool operator!=(const ValueId& other) const { return id != other.id; }
    bool operator<(const ValueId& other) const { return id < other.id; }

    static ValueId nextId(std::string name, const std::string& file_path, float default_value) noexcept {
      Map* id = Map::instance();
      id->info_map_[ValueId(id->next_id_)] = { std::move(name), nameFromPath(file_path), default_value };
      return ValueId(id->next_id_++);
    }

    static float defaultValue(ValueId value_id) {
      return Map::instance()->info_map_[value_id].default_value;
    }

    static ValueIdInfo info(ValueId value_id) { return Map::instance()->info_map_[value_id]; }

    static const std::string& groupName(ValueId value_id) {
      return Map::instance()->info_map_[value_id].group;
    }

    static const std::string& name(ValueId value_id) {
      return Map::instance()->info_map_[value_id].name;
    }

    static std::map<std::string, ValueId> nameIdMap() {
      std::map<std::string, ValueId> result;
      for (const auto& assignment : Map::instance()->info_map_)
        result[assignment.second.name] = assignment.first;

      return result;
    }

    static int numValueIds() { return Map::instance()->next_id_; }

  private:
    struct Map {
      static Map* instance() {
        static Map instance;
        return &instance;
      }

      unsigned int next_id_ = 0;
      std::map<ValueId, ValueIdInfo> info_map_;
    };
  };

  class OverrideId {
  public:
    static constexpr unsigned int kInvalidId = 0xFFFFFFFF;
    static constexpr unsigned int kDefaultId = 0;

    bool operator==(const OverrideId& other) const { return id == other.id; }
    bool operator!=(const OverrideId& other) const { return id != other.id; }
    bool operator<(const OverrideId& other) const { return id < other.id; }

    explicit OverrideId(unsigned int id) : id(id) { }
    OverrideId() = default;

    unsigned int id = kDefaultId;

    bool isDefault() const { return id == kDefaultId; }

    static OverrideId nextId(std::string name) noexcept {
      Map* id = Map::instance();
      id->name_map_[OverrideId(id->next_id_)] = std::move(name);
      return OverrideId(id->next_id_++);
    }

    static const std::string& name(OverrideId value_id) {
      return Map::instance()->name_map_[value_id];
    }

    static OverrideId idFromName(const std::string& name) {
      Map* id = Map::instance();
      for (const auto& assignment : id->name_map_) {
        if (assignment.second == name)
          return assignment.first;
      }
      VISAGE_ASSERT(false);
      return OverrideId(kInvalidId);
    }

    static std::map<std::string, OverrideId> nameIdMap() {
      std::map<std::string, OverrideId> result;
      for (const auto& assignment : Map::instance()->name_map_)
        result[assignment.second] = assignment.first;

      return result;
    }

    static int numOverrideIds() { return Map::instance()->next_id_; }

  private:
    struct Map {
      static Map* instance() {
        static Map instance;
        return &instance;
      }

      unsigned int next_id_ = 0;
      std::map<OverrideId, std::string> name_map_ = { { OverrideId(0), "Global" } };
    };
  };
}