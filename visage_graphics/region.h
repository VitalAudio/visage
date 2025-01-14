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

#include "shape_batcher.h"
#include "visage_utils/space.h"

namespace visage {
  class Palette;
  class Shader;

  class Region {
  public:
    friend class Canvas;

    Region() = default;

    SubmitBatch* submitBatchAtPosition(int position) const {
      return shape_batcher_.batchAtIndex(position);
    }
    int numSubmitBatches() const { return shape_batcher_.numBatches(); }
    bool isEmpty() const { return shape_batcher_.isEmpty(); }
    const std::vector<Region*>& subRegions() const { return sub_regions_; }
    int numRegions() const { return sub_regions_.size(); }

    void addRegion(Region* region) {
      VISAGE_ASSERT(region->parent_ == nullptr);
      sub_regions_.push_back(region);
      region->parent_ = this;

      if (canvas_)
        region->setCanvas(canvas_);

      region->setLayerIndex(layer_index_);
    }

    void removeRegion(Region* region) {
      region->parent_ = nullptr;
      region->setCanvas(nullptr);
      sub_regions_.erase(std::find(sub_regions_.begin(), sub_regions_.end(), region));
    }

    void setCanvas(Canvas* canvas) {
      if (canvas_ == canvas)
        return;

      canvas_ = canvas;
      for (auto& sub_region : sub_regions_)
        sub_region->setCanvas(canvas);
    }

    void setBounds(int x, int y, int width, int height) {
      invalidate();
      x_ = x;
      y_ = y;
      width_ = width;
      height_ = height;
      setupIntermediateRegion();
      invalidate();
    }

    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }
    bool overlaps(const Region* other) const {
      return x_ < other->x_ + other->width_ && x_ + width_ > other->x_ &&
             y_ < other->y_ + other->height_ && y_ + height_ > other->y_;
    }

    int x() const { return x_; }
    int y() const { return y_; }
    int width() const { return width_; }
    int height() const { return height_; }

    void invalidateRect(Bounds rect);

    void invalidate() {
      if (width_ > 0 && height_ > 0)
        invalidateRect({ 0, 0, width_, height_ });
    }

    Layer* layer() const;

    void clear() {
      shape_batcher_.clear();
      text_store_.clear();
    }

    void setupIntermediateRegion();
    void setNeedsLayer(bool needs_layer);
    void setPostEffect(PostEffect* post_effect) {
      post_effect_ = post_effect;
      setupIntermediateRegion();
    }
    PostEffect* postEffect() const { return post_effect_; }
    bool needsLayer() const { return intermediate_region_.get(); }
    Region* intermediateRegion() const { return intermediate_region_.get(); }

  private:
    void setLayerIndex(int layer_index);
    void incrementLayer() { setLayerIndex(layer_index_ + 1); }
    void decrementLayer() { setLayerIndex(layer_index_ - 1); }

    Text* addText(const String& string, const Font& font, Font::Justification justification) {
      text_store_.push_back(std::make_unique<Text>(string, font, justification));
      return text_store_.back().get();
    }

    void clearSubRegions() { sub_regions_.clear(); }

    void clearAll() {
      clear();
      clearSubRegions();
    }

    int x_ = 0;
    int y_ = 0;
    int width_ = 0;
    int height_ = 0;
    int palette_override_ = 0;
    bool visible_ = true;
    int layer_index_ = 0;

    Canvas* canvas_ = nullptr;
    Region* parent_ = nullptr;
    PostEffect* post_effect_ = nullptr;
    ShapeBatcher shape_batcher_;
    std::vector<std::unique_ptr<Text>> text_store_;
    std::vector<Region*> sub_regions_;
    std::unique_ptr<Region> intermediate_region_;
  };
}