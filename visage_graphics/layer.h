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
#include "visage_utils/space.h"

namespace visage {
  class Region;
  struct FrameBufferData;

  class Layer {
  public:
    static constexpr int kInvalidRectMemory = 2;

    Layer();
    ~Layer();

    void checkFrameBuffer();
    void destroyFrameBuffer() const;

    bgfx::FrameBufferHandle& frameBuffer() const;
    int frameBufferFormat() const;

    void clearInvalidRectAreas(int submit_pass);
    int submit(int submit_pass);

    void setIntermediateLayer(bool intermediate_layer) { intermediate_layer_ = intermediate_layer; }
    void addRegion(Region* region);
    void removeRegion(Region* region) {
      regions_.erase(std::find(regions_.begin(), regions_.end(), region));
    }
    void addPackedRegion(Region* region);
    void removePackedRegion(Region* region);
    Point coordinatesForRegion(const Region* region) const;

    template<typename V>
    void setTexturePositionsForRegion(const Region* region, V* vertices) const {
      atlas_.setTexturePositionsForId(region, vertices);
    }

    void invalidate() {
      invalid_rects_.clear();
      invalid_rects_.emplace_back(0, 0, width_, height_);
    }

    void invalidateRect(Bounds rect);
    void invalidateRectInRegion(Bounds rect, Region* region);

    void setDimensions(int width, int height) {
      if (width == width_ && height == height_)
        return;

      width_ = width;
      height_ = height;
      destroyFrameBuffer();
      invalidate();
    }
    int width() const { return width_; }
    int height() const { return height_; }
    bool bottomLeftOrigin() const { return bottom_left_origin_; }

    double time() const { return render_time_; }
    void setTime(double time) { render_time_ = time; }

    void setHdr(bool hdr) {
      hdr_ = hdr;
      destroyFrameBuffer();
    }
    bool hdr() const { return hdr_; }

    void pairToWindow(void* window_handle, int width, int height) {
      window_handle_ = window_handle;
      setDimensions(width, height);
      destroyFrameBuffer();
    }

    void removeFromWindow() {
      window_handle_ = nullptr;
      destroyFrameBuffer();
    }

    void clear() { regions_.clear(); }

  private:
    bool bottom_left_origin_ = false;
    bool hdr_ = false;
    int width_ = 0;
    int height_ = 0;
    double render_time_ = 0.0;
    bool intermediate_layer_ = false;

    void* window_handle_ = nullptr;
    std::unique_ptr<FrameBufferData> frame_buffer_data_;
    PackedAtlas<const Region*> atlas_;
    std::vector<Region*> regions_;
    std::vector<Bounds> invalid_rects_;
    std::vector<Bounds> prev_invalid_rects_[kInvalidRectMemory];
    std::vector<Bounds> invalid_rect_pieces_;
  };
}