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

#include "region.h"

#include "canvas.h"

namespace visage {
  void Region::invalidateRect(Bounds rect) {
    if (canvas_ == nullptr)
      return;

    int layer_index = layer_index_;
    Region* region = this;
    while (region->parent_) {
      if (region->needsLayer()) {
        canvas_->invalidateRectInRegion(rect, region, layer_index);
        --layer_index;

        if (region->post_effect_)
          rect = { 0, 0, region->width_, region->height_ };
      }

      rect = rect + Point(region->x_, region->y_);
      region = region->parent_;
    }

    canvas_->invalidateRectInRegion(rect, region, region->layer_index_);
  }

  Layer* Region::layer() const {
    return canvas_->layer(layer_index_);
  }

  void Region::setupIntermediateRegion() {
    if (intermediate_region_) {
      intermediate_region_->setBounds(x_, y_, width_, height_);
      intermediate_region_->clearAll();
      SampleRegion sample_layer({ 0.0f, 0.0f, width_ * 1.0f, height_ * 1.0f }, 0xffffffff, 0, 0,
                                width_, height_, this, post_effect_);
      intermediate_region_->shape_batcher_.addShape(sample_layer);
      canvas_->changePackedLayer(this, layer_index_, layer_index_);
    }
  }

  void Region::setNeedsLayer(bool needs_layer) {
    if (needsLayer() == needs_layer)
      return;

    if (needs_layer) {
      incrementLayer();
      intermediate_region_ = std::make_unique<Region>();
      canvas_->addToPackedLayer(this, layer_index_);
      setupIntermediateRegion();
    }
    else {
      canvas_->removeFromPackedLayer(this, layer_index_);
      intermediate_region_ = nullptr;
      decrementLayer();
    }

    invalidate();
  }

  void Region::setLayerIndex(int layer_index) {
    if (needsLayer())
      canvas_->changePackedLayer(this, layer_index_, layer_index);

    layer_index_ = layer_index;
    for (auto& sub_region : sub_regions_) {
      if (sub_region->needsLayer())
        sub_region->setLayerIndex(layer_index + 1);
      else
        sub_region->setLayerIndex(layer_index);
    }
  }
}
