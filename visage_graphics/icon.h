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

#include "graphics_utils.h"

#include <map>

namespace visage {
  struct Icon {
    Icon() = default;
    Icon(const char* svg, int svg_size, int width, int height, int blur_radius = 0) :
        svg(svg), svg_size(svg_size), width(width), height(height), blur_radius(blur_radius) { }

    const char* svg = nullptr;
    int svg_size = 0;
    int width = 0;
    int height = 0;
    int blur_radius = 0;

    bool operator==(const Icon& other) const {
      return svg == other.svg && svg_size == other.svg_size && width == other.width &&
             height == other.height && blur_radius == other.blur_radius;
    }

    bool operator<(const Icon& other) const {
      return svg < other.svg || (svg == other.svg && svg_size < other.svg_size) ||
             (svg == other.svg && svg_size == other.svg_size && width < other.width) ||
             (svg == other.svg && svg_size == other.svg_size && width == other.width &&
              height < other.height) ||
             (svg == other.svg && svg_size == other.svg_size && width == other.width &&
              height == other.height && blur_radius < other.blur_radius);
    }
  };

  class IconGroupTexture;

  class IconGroup {
  public:
    static constexpr int kIconBuffer = 1;

    IconGroup();
    virtual ~IconGroup();

    void clear() {
      icon_count_.clear();
      atlas_.clear();
    }

    void incrementIcon(const Icon& icon);
    void decrementIcon(const Icon& icon);
    int atlasWidth() const { return atlas_.width(); }
    const bgfx::TextureHandle& textureHandle() const;
    void setIconCoordinates(TextureVertex* vertices, const Icon& icon) const;

  private:
    void setNewSize();
    bool packIcon(const Icon& icon);
    void drawIcon(const Icon& icon);
    void blurIcon(unsigned int* location, int width, int blur_radius) const;

    PackedAtlas<Icon> atlas_;
    std::map<Icon, int> icon_count_;
    std::unique_ptr<IconGroupTexture> texture_;
  };
}