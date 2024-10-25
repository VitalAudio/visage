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

#include "image.h"

#include "canvas.h"
#include "graphics_libs.h"

namespace visage {
  bgfx::VertexLayout& ImageVertex::layout() {
    static bgfx::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin()
          .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
          .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
          .add(bgfx::Attrib::Color1, 1, bgfx::AttribType::Float)
          .end();
    }

    return layout;
  }

  ImageGroup::ImageGroup() {
    canvas_ = std::make_unique<Canvas>();
    setNewSize();
  }

  ImageGroup::~ImageGroup() = default;

  void ImageGroup::setNewSize() {
    atlas_.pack();
    canvas_->setDimensions(atlas_.width(), atlas_.width());
  }

  bool ImageGroup::addImage(Image* image) {
    int index = images_.size();
    images_.push_back(image);
    image_lookup_[image] = index;
    return atlas_.addPackedRect(index, image->width(), image->height());
  }

  int ImageGroup::addImages(const std::set<Image*>& images, int submit_pass) {
    std::set<Image*> filtered;
    for (Image* image : images) {
      if (image->needsRedraw() || image_lookup_.count(image) == 0)
        filtered.insert(image);
    }

    if (filtered.empty())
      return 0;

    int start_draw_index = images_.size();
    bool pack_succeeded = true;
    for (Image* image : filtered)
      pack_succeeded = pack_succeeded && addImage(image);

    if (pack_succeeded)
      return drawImages(submit_pass, start_draw_index, images_.size());

    setNewSize();
    return drawImages(submit_pass);
  }

  int ImageGroup::drawImages(int submit_pass, int start_index, int end_index) const {
    int end = end_index < 0 ? images_.size() : end_index;
    if (start_index >= end)
      return 0;

    for (int i = start_index; i < end; ++i)
      drawImage(*canvas_, i);

    int new_submit_pass = canvas_->submit(submit_pass);
    canvas_->clearDrawnShapes();
    return new_submit_pass;
  }

  void ImageGroup::drawImage(Canvas& canvas, int index) const {
    Image* image = images_[index];
    int image_width = image->width();
    int image_height = image->height();

    if (image_width == 0 || image_height == 0)
      return;

    canvas.saveState();
    PackedAtlas::Rect rect = atlas_.rectAtIndex(index);
    canvas.setPosition(rect.x, rect.y);
    canvas.setClampBounds(0, 0, image_width, image_height);
    canvas.clearArea(0, 0, image_width, image_height);
    image->draw(canvas);
    canvas.restoreState();
  }

  bgfx::TextureHandle ImageGroup::atlasTexture() const {
    return bgfx::getTexture(canvas_->frameBuffer());
  }

  void ImageGroup::setImageCoordinates(ImageVertex* vertices, const Image* image) const {
    int index = imageIndex(image);

    VISAGE_ASSERT(index >= 0);
    if (index < 0)
      return;

    atlas_.setQuadCoordinates(vertices, index);

    if (bgfx::getCaps()->originBottomLeft) {
      for (int i = 0; i < 4; ++i)
        vertices[i].coordinate_y = 1.0f - vertices[i].coordinate_y;
    }
  }
}