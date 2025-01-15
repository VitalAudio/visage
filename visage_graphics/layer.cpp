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

#include "layer.h"

#include "canvas.h"
#include "palette.h"
#include "region.h"
#include "theme.h"

#include <bgfx/bgfx.h>

namespace visage {
  struct FrameBufferData {
    bgfx::FrameBufferHandle handle = BGFX_INVALID_HANDLE;
    bgfx::TextureFormat::Enum format = bgfx::TextureFormat::RGBA8;
  };

  struct RegionPosition {
    RegionPosition(Region* region, std::vector<Bounds> invalid_rects, int position, int x = 0, int y = 0) :
        region(region), invalid_rects(std::move(invalid_rects)), position(position), x(x), y(y) { }
    RegionPosition() = default;

    Region* region = nullptr;
    std::vector<Bounds> invalid_rects;
    int position = 0;
    int x = 0;
    int y = 0;

    SubmitBatch* currentBatch() const { return region->submitBatchAtPosition(position); }
    bool isDone() const { return position >= region->numSubmitBatches(); }
  };

  inline void moveToVector(std::vector<Bounds>& rects, std::vector<Bounds>& pieces) {
    rects.insert(rects.end(), pieces.begin(), pieces.end());
    pieces.clear();
  }

  static void addSubRegions(std::vector<RegionPosition>& positions, std::vector<RegionPosition>& overlapping,
                            const RegionPosition& done_position) {
    auto begin = done_position.region->subRegions().cbegin();
    auto end = done_position.region->subRegions().cend();
    for (auto it = begin; it != end; ++it) {
      Region* sub_region = *it;
      if (!sub_region->isVisible())
        continue;

      if (sub_region->needsLayer())
        sub_region = sub_region->intermediateRegion();

      bool overlaps = std::any_of(begin, it, [sub_region](const Region* other) {
        return other->isVisible() && sub_region->overlaps(other);
      });

      Bounds bounds(done_position.x + sub_region->x(), done_position.y + sub_region->y(),
                    sub_region->width(), sub_region->height());

      std::vector<Bounds> invalid_rects;
      for (const Bounds& invalid_rect : done_position.invalid_rects) {
        if (bounds.overlaps(invalid_rect))
          invalid_rects.push_back(invalid_rect.intersection(bounds));
      }

      if (invalid_rects.empty())
        continue;

      if (overlaps)
        overlapping.emplace_back(sub_region, std::move(invalid_rects), 0, bounds.x(), bounds.y());
      else if (sub_region->isEmpty())
        addSubRegions(positions, overlapping,
                      { sub_region, std::move(invalid_rects), 0, bounds.x(), bounds.y() });
      else
        positions.emplace_back(sub_region, std::move(invalid_rects), 0, bounds.x(), bounds.y());
    }
  }

  static void checkOverlappingRegions(std::vector<RegionPosition>& positions,
                                      std::vector<RegionPosition>& overlapping) {
    for (auto it = overlapping.begin(); it != overlapping.end();) {
      bool overlaps = std::any_of(positions.begin(), positions.end(), [it](const RegionPosition& other) {
        return it->x < other.x + other.region->width() && it->x + it->region->width() > other.x &&
               it->y < other.y + other.region->height() && it->y + it->region->height() > other.y;
      });

      if (!overlaps) {
        if (it->isDone())
          addSubRegions(positions, overlapping, *it);
        else
          positions.push_back(*it);
        it = overlapping.erase(it);
      }
      else
        ++it;
    }
  }

  static const SubmitBatch* nextBatch(const std::vector<RegionPosition>& positions,
                                      const void* current_batch_id, BlendMode current_blend_mode) {
    const SubmitBatch* next_batch = positions[0].currentBatch();
    for (auto& position : positions) {
      const SubmitBatch* batch = position.currentBatch();
      if (next_batch->compare(batch) > 0) {
        if (batch->compare(current_batch_id, current_blend_mode) > 0 ||
            next_batch->compare(current_batch_id, current_blend_mode) < 0) {
          next_batch = position.currentBatch();
        }
      }
      else if (next_batch->compare(current_batch_id, current_blend_mode) < 0 &&
               batch->compare(current_batch_id, current_blend_mode) > 0) {
        next_batch = position.currentBatch();
      }
    }

    return next_batch;
  }

  Layer::Layer() {
    frame_buffer_data_ = std::make_unique<FrameBufferData>();
  }

  Layer::~Layer() {
    destroyFrameBuffer();
  }

  void Layer::checkFrameBuffer() {
    static constexpr uint64_t kFrameBufferFlags = BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP |
                                                  BGFX_SAMPLER_V_CLAMP;

    if (bgfx::isValid(frame_buffer_data_->handle))
      return;

    if (hdr_) {
      if (bgfx::getRendererType() == bgfx::RendererType::Vulkan)
        frame_buffer_data_->format = bgfx::TextureFormat::RGBA16F;
      else
        frame_buffer_data_->format = bgfx::TextureFormat::RGB10A2;
    }
    else
      frame_buffer_data_->format = bgfx::TextureFormat::RGBA8;

    if (window_handle_) {
      frame_buffer_data_->handle = bgfx::createFrameBuffer(window_handle_, width_, height_,
                                                           frame_buffer_data_->format);
    }
    else {
      frame_buffer_data_->handle = bgfx::createFrameBuffer(width_, height_, frame_buffer_data_->format,
                                                           kFrameBufferFlags);
    }

    bottom_left_origin_ = bgfx::getCaps()->originBottomLeft;
  }

  void Layer::destroyFrameBuffer() const {
    if (bgfx::isValid(frame_buffer_data_->handle)) {
      bgfx::destroy(frame_buffer_data_->handle);
      frame_buffer_data_->handle = BGFX_INVALID_HANDLE;

      bgfx::frame();
      bgfx::frame();
    }
  }

  bgfx::FrameBufferHandle& Layer::frameBuffer() const {
    return frame_buffer_data_->handle;
  }

  int Layer::frameBufferFormat() const {
    return frame_buffer_data_->format;
  }

  void Layer::invalidateRect(Bounds rect) {
    for (auto it = invalid_rects_.begin(); it != invalid_rects_.end();) {
      Bounds& invalid_rect = *it;
      if (invalid_rect.contains(rect)) {
        moveToVector(invalid_rects_, invalid_rect_pieces_);
        return;
      }

      if (rect.contains(invalid_rect)) {
        it = invalid_rects_.erase(it);
        continue;
      }
      Bounds::breakIntoNonOverlapping(rect, invalid_rect, invalid_rect_pieces_);
      ++it;
    }

    invalid_rects_.push_back(rect);
    moveToVector(invalid_rects_, invalid_rect_pieces_);
  }

  void Layer::invalidateRectInRegion(Bounds rect, const Region* region) {
    invalidateRect(rect + coordinatesForRegion(region));
  }

  void Layer::clearInvalidRectAreas(int submit_pass) {
    ShapeBatch<Fill> clear_batch(BlendMode::Opaque);
    QuadColor color;
    for (const Bounds& rect : invalid_rects_) {
      float x = rect.x();
      float y = rect.y();
      float width = rect.width();
      float height = rect.height();
      clear_batch.addShape(Fill({ x, y, x + width, y + height }, color, x, y, width, height));
    }

    PositionedBatch positioned_clear = { &clear_batch, &invalid_rects_, 0, 0 };
    clear_batch.submit(*this, submit_pass, { positioned_clear });
  }

  int Layer::submit(int submit_pass) {
    if (!anyInvalidRects())
      return submit_pass;

    checkFrameBuffer();
    bgfx::setViewMode(submit_pass, bgfx::ViewMode::Sequential);
    bgfx::setViewRect(submit_pass, 0, 0, width_, height_);
    if (bgfx::isValid(frame_buffer_data_->handle))
      bgfx::setViewFrameBuffer(submit_pass, frame_buffer_data_->handle);

    if (intermediate_layer_)
      clearInvalidRectAreas(submit_pass);

    std::vector<RegionPosition> region_positions;
    std::vector<RegionPosition> overlapping_regions;
    for (Region* region : regions_) {
      Point point = coordinatesForRegion(region);
      if (region->isEmpty()) {
        addSubRegions(region_positions, overlapping_regions,
                      { region, invalid_rects_, 0, point.x, point.y });
      }
      else
        region_positions.emplace_back(region, invalid_rects_, 0, point.x, point.y);
    }

    invalid_rects_.clear();

    const void* current_batch_id = nullptr;
    BlendMode current_blend_mode = BlendMode::Opaque;
    std::vector<PositionedBatch> batches;
    std::vector<RegionPosition> done_regions;

    while (!region_positions.empty()) {
      const SubmitBatch* next_batch = nextBatch(region_positions, current_batch_id, current_blend_mode);
      for (auto& region_position : region_positions) {
        SubmitBatch* batch = region_position.currentBatch();
        if (batch->id() != next_batch->id() || batch->blendMode() != next_batch->blendMode())
          continue;

        batches.push_back({ batch, &region_position.invalid_rects, region_position.x,
                            region_position.y });
        region_position.position++;
      }

      batches.front().batch->submit(*this, submit_pass, batches);
      batches.clear();

      auto done_it = std::partition(region_positions.begin(), region_positions.end(),
                                    [](const RegionPosition& position) { return position.isDone(); });
      done_regions.insert(done_regions.end(), std::make_move_iterator(region_positions.begin()),
                          std::make_move_iterator(done_it));
      region_positions.erase(region_positions.begin(), done_it);

      for (auto& region_position : done_regions)
        addSubRegions(region_positions, overlapping_regions, region_position);

      if (!done_regions.empty())
        checkOverlappingRegions(region_positions, overlapping_regions);

      done_regions.clear();
      current_batch_id = next_batch->id();
      current_blend_mode = next_batch->blendMode();
    }

    submit_pass = submit_pass + 1;
    for (Region* region : regions_) {
      if (region->postEffect())
        submit_pass = region->postEffect()->preprocess(region, submit_pass);
    }

    return submit_pass;
  }

  void Layer::addRegion(Region* region) {
    if (!hdr_ && region->postEffect() && region->postEffect()->hdr())
      setHdr(true);

    regions_.push_back(region);
  }

  void Layer::addPackedRegion(Region* region) {
    addRegion(region);
    if (!atlas_.addRect(region, region->width(), region->height())) {
      invalidate();
      atlas_.pack();
      setDimensions(atlas_.width(), atlas_.width());
    }
  }

  void Layer::removePackedRegion(Region* region) {
    removeRegion(region);
    atlas_.removeRect(region);
  }

  Point Layer::coordinatesForRegion(const Region* region) const {
    if (intermediate_layer_) {
      const PackedRect& rect = atlas_.rectForId(region);
      return { rect.x, rect.y };
    }
    return { region->x(), region->y() };
  }
}
