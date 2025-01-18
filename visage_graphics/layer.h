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
    void removeRegion(const Region* region) {
      regions_.erase(std::find(regions_.begin(), regions_.end(), region));
    }
    void addPackedRegion(Region* region);
    void removePackedRegion(Region* region);
    Bounds boundsForRegion(const Region* region) const;
    Point coordinatesForRegion(const Region* region) const;

    template<typename V>
    void setTexturePositionsForRegion(const Region* region, V* vertices) const {
      atlas_.setTexturePositionsForId(region, vertices, bottom_left_origin_);
    }

    void invalidate() {
      invalid_rects_.clear();
      for (const auto& region : regions_)
        invalid_rects_[region].push_back(boundsForRegion(region));
    }

    void invalidateRectInRegion(Bounds rect, const Region* region);
    bool anyInvalidRects() { return !invalid_rects_.empty(); }

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

    void clear() {
      regions_.clear();
      atlas_.clear();
    }

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
    std::map<const Region*, std::vector<Bounds>> invalid_rects_;
    std::vector<Bounds> invalid_rect_pieces_;
    std::vector<Region*> regions_;
  };
}