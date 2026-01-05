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
#include "region.h"
#include "renderer.h"

#include <bgfx/bgfx.h>

namespace visage {
  struct FrameBufferData {
    bgfx::TextureHandle read_back_handle = BGFX_INVALID_HANDLE;
    bgfx::FrameBufferHandle handle = BGFX_INVALID_HANDLE;
    bgfx::TextureFormat::Enum format = bgfx::TextureFormat::RGBA8;
  };

  struct RegionPosition {
    RegionPosition(Region* region, std::vector<IBounds> invalid_rects, int position, int x = 0, int y = 0) :
        region(region), invalid_rects(std::move(invalid_rects)), position(position), x(x), y(y) { }
    RegionPosition() = default;

    Region* region = nullptr;
    std::vector<IBounds> invalid_rects;
    int position = 0;
    int x = 0;
    int y = 0;

    SubmitBatch* currentBatch() const { return region->submitBatchAtPosition(position); }
    bool isDone() const { return position >= region->numSubmitBatches(); }
  };

  inline void moveToVector(std::vector<IBounds>& rects, std::vector<IBounds>& pieces) {
    rects.insert(rects.end(), pieces.begin(), pieces.end());
    pieces.clear();
  }

  static void addSubRegions(std::vector<RegionPosition>& positions, std::vector<RegionPosition>& overlapping,
                            const RegionPosition& done_position, int backdrop_count) {
    auto begin = done_position.region->subRegions().cbegin();
    auto end = done_position.region->subRegions().cend();
    if (begin == end)
      return;

    std::vector<RegionPosition> new_overlapping;
    bool on_top = false;
    for (auto it = begin; it != end || !on_top; ++it) {
      if (it == end) {
        on_top = true;
        it = begin;
      }

      Region* sub_region = *it;
      if (!sub_region->isVisible() || sub_region->isOnTop() != on_top)
        continue;

      if (sub_region->backdropCount() > backdrop_count || sub_region->backdropCountChildren() < backdrop_count)
        continue;

      bool should_draw = sub_region->shouldDraw(backdrop_count);
      if (sub_region->needsLayer())
        sub_region = sub_region->intermediateRegion();

      IBounds bounds(done_position.x + sub_region->x(), done_position.y + sub_region->y(),
                     sub_region->width(), sub_region->height());

      auto overlap_check = [&bounds](const RegionPosition& other) {
        IBounds other_bounds(other.x, other.y, other.region->width(), other.region->height());
        return bounds.overlaps(other_bounds);
      };

      bool overlaps = std::any_of(positions.begin(), positions.end(), overlap_check) ||
                      std::any_of(new_overlapping.begin(), new_overlapping.end(), overlap_check);

      std::vector<IBounds> invalid_rects;
      for (const IBounds& invalid_rect : done_position.invalid_rects) {
        if (bounds.overlaps(invalid_rect))
          invalid_rects.push_back(invalid_rect.intersection(bounds));
      }

      if (invalid_rects.empty())
        continue;

      if (overlaps)
        new_overlapping.emplace_back(sub_region, std::move(invalid_rects), 0, bounds.x(), bounds.y());
      else if (sub_region->isEmpty() || !should_draw) {
        std::vector<RegionPosition> child_overlap;
        RegionPosition region(sub_region, std::move(invalid_rects), 0, bounds.x(), bounds.y());
        addSubRegions(positions, child_overlap, region, backdrop_count);
        new_overlapping.insert(new_overlapping.end(), child_overlap.rbegin(), child_overlap.rend());
      }
      else
        positions.emplace_back(sub_region, std::move(invalid_rects), 0, bounds.x(), bounds.y());
    }
    overlapping.insert(overlapping.end(), new_overlapping.rbegin(), new_overlapping.rend());
  }

  static void checkOverlappingRegions(std::vector<RegionPosition>& positions,
                                      std::vector<RegionPosition>& overlapping, int backdrop_count) {
    for (int i = overlapping.size() - 1; i >= 0; --i) {
      auto it = overlapping.begin() + i;
      auto overlap_check = [it](const RegionPosition& other) {
        return it->x < other.x + other.region->width() && it->x + it->region->width() > other.x &&
               it->y < other.y + other.region->height() && it->y + it->region->height() > other.y;
      };
      bool overlaps = std::any_of(positions.begin(), positions.end(), overlap_check) ||
                      std::any_of(it + 1, overlapping.end(), overlap_check);

      if (!overlaps) {
        auto position = *it;
        overlapping.erase(it);
        if (position.isDone())
          addSubRegions(positions, overlapping, position, backdrop_count);
        else
          positions.push_back(position);
      }
    }
  }

  static const SubmitBatch* nextBatch(const std::vector<RegionPosition>& positions,
                                      const SubmitBatch* current) {
    const SubmitBatch* next_batch = positions[0].currentBatch();
    for (auto& position : positions) {
      const SubmitBatch* batch = position.currentBatch();
      if (next_batch->compare(batch) > 0) {
        if (batch->compare(current) > 0 || next_batch->compare(current) < 0)
          next_batch = position.currentBatch();
      }
      else if (next_batch->compare(current) < 0 && batch->compare(current) > 0)
        next_batch = position.currentBatch();
    }

    return next_batch;
  }

  Layer::Layer(GradientAtlas* gradient_atlas) : gradient_atlas_(gradient_atlas) {
    frame_buffer_data_ = std::make_unique<FrameBufferData>();
    clear_brush_ = std::make_unique<const PackedBrush>(gradient_atlas, Brush::solid(0));
  }

  Layer::~Layer() {
    destroyFrameBuffer();
  }

  void Layer::checkFrameBuffer() {
    static constexpr uint64_t kFrameBufferFlags = BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP |
                                                  BGFX_SAMPLER_V_CLAMP;

    if (bgfx::isValid(frame_buffer_data_->handle))
      return;

    if (hdr_)
      frame_buffer_data_->format = bgfx::TextureFormat::RGB10A2;
    else
      frame_buffer_data_->format = bgfx::TextureFormat::RGBA8;

    if (window_handle_) {
      frame_buffer_data_->handle = bgfx::createFrameBuffer(window_handle_, width_, height_,
                                                           frame_buffer_data_->format);
    }
    else {
      bool read_back = (bgfx::getCaps()->supported & BGFX_CAPS_TEXTURE_BLIT) &&
                       (bgfx::getCaps()->supported & BGFX_CAPS_TEXTURE_READ_BACK);
      if (headless_render_ && read_back) {
        uint64_t flags = BGFX_TEXTURE_BLIT_DST | BGFX_TEXTURE_READ_BACK;
        frame_buffer_data_->read_back_handle = bgfx::createTexture2D(width_, height_, false, 1,
                                                                     bgfx::TextureFormat::RGBA8, flags);
      }
      frame_buffer_data_->handle = bgfx::createFrameBuffer(width_, height_, frame_buffer_data_->format,
                                                           kFrameBufferFlags);
    }

    bottom_left_origin_ = bgfx::getCaps()->originBottomLeft;
  }

  void Layer::destroyFrameBuffer() {
    if (bgfx::isValid(frame_buffer_data_->handle)) {
      bgfx::destroy(frame_buffer_data_->handle);
      frame_buffer_data_->handle = BGFX_INVALID_HANDLE;
    }
  }

  bgfx::FrameBufferHandle& Layer::frameBuffer() const {
    return frame_buffer_data_->handle;
  }

  int Layer::frameBufferFormat() const {
    return frame_buffer_data_->format;
  }

  void Layer::invalidateRectInRegion(IBounds rect, const Region* region) {
    IBounds region_bounds = boundsForRegion(region);
    rect = rect + IPoint(region_bounds.x(), region_bounds.y());
    rect = rect.intersection(region_bounds);

    std::vector<IBounds>& invalid_rects = invalid_rects_[region];

    for (auto it = invalid_rects.begin(); it != invalid_rects.end();) {
      IBounds& invalid_rect = *it;
      if (invalid_rect.contains(rect)) {
        moveToVector(invalid_rects, invalid_rect_pieces_);
        return;
      }

      if (rect.contains(invalid_rect)) {
        it = invalid_rects.erase(it);
        continue;
      }
      IBounds::breakIntoNonOverlapping(rect, invalid_rect, invalid_rect_pieces_);
      ++it;
    }

    invalid_rects.push_back(rect);
    moveToVector(invalid_rects, invalid_rect_pieces_);
  }

  void Layer::checkBackdropInvalidation(const std::vector<IBounds>& top_level_invalid_rects) {
    for (Region* region : regions_) {
      if (region->backdropEffect()) {
        IBounds bounds(0, 0, region->width(), region->height());
        Region* parent = region;
        while (parent) {
          bounds = bounds + IPoint(parent->x(), parent->y());
          parent = parent->parent();
        }

        if (std::any_of(top_level_invalid_rects.begin(), top_level_invalid_rects.end(),
                        [&bounds](const auto& invalid_rect) { return bounds.overlaps(invalid_rect); })) {
          region->invalidate();
        }
      }
    }
  }

  bool Layer::hasBackdropEffect() const {
    return std::any_of(regions_.begin(), regions_.end(),
                       [](const auto& region) { return region->backdropEffect(); });
  }

  void Layer::clearInvalidRectAreas(int submit_pass) {
    ShapeBatch<Fill> clear_batch(BlendMode::Opaque);
    std::vector<IBounds> invalid_rects;
    for (auto& region_invalid_rects : invalid_rects_) {
      for (const IBounds& rect : region_invalid_rects.second) {
        invalid_rects.push_back(rect);
        float x = rect.x();
        float y = rect.y();
        float width = rect.width();
        float height = rect.height();
        clear_batch.addShape(Fill({ x, y, x + width, y + height }, clear_brush_.get(), x, y, width, height));
      }
    }

    PositionedBatch positioned_clear = { &clear_batch, &invalid_rects, 0, 0 };
    clear_batch.submit(*this, submit_pass, { positioned_clear });
  }

  int Layer::submit(int submit_pass, int backdrop_count) {
    if (!anyInvalidRects() && !(hasBackdropEffect() && backdrop_count > 0))
      return submit_pass;

    checkFrameBuffer();

    std::vector<RegionPosition> region_positions;
    std::vector<RegionPosition> overlapping_regions;
    for (Region* region : regions_) {
      if (region->backdropCount() > backdrop_count || region->backdropCountChildren() < backdrop_count)
        continue;

      if (region->backdropEffect() && region->backdropCount() == backdrop_count) {
        auto parent = region->parent();
        auto backdrop_region = parent;
        while (parent->parent()) {
          parent = parent->parent();
          if (parent->needsLayer())
            backdrop_region = parent;
        }

        if (backdrop_region && backdrop_region->needsLayer())
          submit_pass = region->backdropEffect()->preprocess(backdrop_region, submit_pass);
      }

      IPoint point = coordinatesForRegion(region);
      if (region->isEmpty() || !region->shouldDraw(backdrop_count)) {
        addSubRegions(region_positions, overlapping_regions,
                      { region, invalid_rects_[region], 0, point.x, point.y }, backdrop_count);
      }
      else
        region_positions.emplace_back(region, invalid_rects_[region], 0, point.x, point.y);
    }
    if (region_positions.empty())
      return submit_pass;

    const SubmitBatch* current_batch = nullptr;
    std::vector<PositionedBatch> batches;
    std::vector<RegionPosition> done_regions;

    bgfx::setViewMode(submit_pass, bgfx::ViewMode::Sequential);
    bgfx::setViewRect(submit_pass, 0, 0, width_, height_);

    if (bgfx::isValid(frame_buffer_data_->handle))
      bgfx::setViewFrameBuffer(submit_pass, frame_buffer_data_->handle);

    if (intermediate_layer_ && backdrop_count == 0)
      clearInvalidRectAreas(submit_pass);

    while (!region_positions.empty()) {
      const SubmitBatch* next_batch = nextBatch(region_positions, current_batch);
      for (auto& region_position : region_positions) {
        SubmitBatch* batch = region_position.currentBatch();
        if (!batch->match(next_batch))
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
        addSubRegions(region_positions, overlapping_regions, region_position, backdrop_count);

      if (!done_regions.empty())
        checkOverlappingRegions(region_positions, overlapping_regions, backdrop_count);

      done_regions.clear();
      current_batch = next_batch;
    }

    if (screenshot_requested_ && bgfx::isValid(frame_buffer_data_->read_back_handle)) {
      screenshot_requested_ = false;
      bgfx::blit(submit_pass, frame_buffer_data_->read_back_handle, 0, 0,
                 bgfx::getTexture(frame_buffer_data_->handle), 0, 0, width_, height_);

      screenshot_.setDimensions(width_, height_);
      bgfx::readTexture(frame_buffer_data_->read_back_handle, screenshot_.data());
      bgfx::frame();
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
    if (!atlas_map_.addRect(region, region->width(), region->height())) {
      atlas_map_.pack();
      invalidate();
      setDimensions(atlas_map_.width(), atlas_map_.height());
    }
  }

  void Layer::removePackedRegion(const Region* region) {
    removeRegion(region);
    atlas_map_.removeRect(region);
  }

  IBounds Layer::boundsForRegion(const Region* region) const {
    if (intermediate_layer_) {
      const PackedRect& rect = atlas_map_.rectForId(region);
      return { rect.x, rect.y, rect.w, rect.h };
    }
    return { region->x(), region->y(), region->width(), region->height() };
  }

  IPoint Layer::coordinatesForRegion(const Region* region) const {
    if (intermediate_layer_) {
      const PackedRect& rect = atlas_map_.rectForId(region);
      return { rect.x, rect.y };
    }
    return { region->x(), region->y() };
  }

  void Layer::requestScreenshot() {
    if (headless_render_)
      screenshot_requested_ = true;
    else {
      Renderer::resetResolution(width_, height_);
      bgfx::requestScreenShot(frame_buffer_data_->handle, "screenshot.png");
    }
    invalidate();
  }

  const Screenshot& Layer::screenshot() const {
    if (headless_render_)
      return screenshot_;
    return Renderer::instance().screenshot();
  }
}
