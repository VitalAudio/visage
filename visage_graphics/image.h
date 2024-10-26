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
#include <set>

namespace visage {
  class Canvas;

  struct ImageVertex {
    float x;
    float y;
    float coordinate_x;
    float coordinate_y;
    float dimension_x;
    float dimension_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    uint32_t color;
    float hdr;

    static bgfx::VertexLayout& layout();
  };

  class Image {
  public:
    virtual ~Image() = default;
    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual bool needsRedraw() const = 0;
    virtual void draw(Canvas& canvas) = 0;
  };

  class ImageGroup {
  public:
    explicit ImageGroup();
    virtual ~ImageGroup();

    void clear() {
      images_.clear();
      image_lookup_.clear();
      atlas_.clear();
    }

    int addImages(const std::set<Image*>& images, int submit_pass);
    int atlasWidth() const { return atlas_.width(); }
    bgfx::TextureHandle atlasTexture() const;

    int imageIndex(const Image* image) const {
      if (image_lookup_.count(image))
        return image_lookup_.at(image);
      return -1;
    }

    void setImageCoordinates(ImageVertex* vertices, const Image* image) const;

  private:
    void setNewSize();
    bool addImage(Image* image);
    int drawImages(int submit_pass, int start_index = 0, int end_index = -1) const;
    void drawImage(Canvas& canvas, int index) const;

    PackedAtlas atlas_;
    std::unique_ptr<Canvas> canvas_;

    std::vector<Image*> images_;
    std::map<const Image*, int> image_lookup_;
  };
}