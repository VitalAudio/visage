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

#include "region.h"

#include "canvas.h"

namespace visage {
  void Region::invalidateRect(IBounds rect) {
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

      rect = rect + IPoint(region->x_, region->y_);
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
      const PackedBrush* brush = intermediate_region_->addBrush(canvas_->gradientAtlas(),
                                                                Brush::solid(0xffffffff));
      SampleRegion sample_region({ 0.0f, 0.0f, width_ * 1.0f, height_ * 1.0f }, brush, 0, 0, width_,
                                 height_, this, post_effect_);
      intermediate_region_->shape_batcher_.addShape(sample_region);
      canvas_->changePackedLayer(this, layer_index_, layer_index_);
    }
  }

  void Region::setNeedsLayer(bool needs_layer) {
    if (needsLayer() == needs_layer || canvas_ == nullptr)
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
