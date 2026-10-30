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

#include "canvas.h"

#include "gpu/gpu.h"
#include "palette.h"
#include "renderer.h"
#include "theme.h"

namespace visage {
  bool Canvas::swapChainSupported() {
    return gpu::getCaps()->swap_chain;
  }

  Canvas::Canvas() :
      default_brush_(&gradient_atlas_, Brush::solid(0xffffffff)),
      image_atlas_(ImageAtlas::DataType::RGBA8), data_atlas_(ImageAtlas::DataType::Float32),
      composite_layer_(&gradient_atlas_) {
    VISAGE_ASSERT(!hasStaticStorage(this));
    Renderer::instance().watchForLeaks();
    state_.brush = &default_brush_;
    state_.current_region = &default_region_;
    layers_.push_back(&composite_layer_);
    composite_layer_.addRegion(&window_region_);

    window_region_.setCanvas(this);
    window_region_.addRegion(&default_region_);
    default_region_.setLayerBlendMode(BlendMode::Opaque);
    default_region_.setCanvas(this);
    default_region_.setNeedsLayer(true);
  }

  void Canvas::clearDrawnShapes() {
    default_region_.clear();
    default_region_.invalidate();
    composite_layer_.clear();
    composite_layer_.addRegion(&window_region_);
  }

  void Canvas::removeFromWindow() {
    if (composite_layer_.removeFromWindow())
      advanceFrame();
  }

  void Canvas::setWindowless(int width, int height) {
    setDimensions(width, height);
    composite_layer_.setWindowlessRender(width, height);
    Renderer::instance().initializeWindowless();
  }

  void Canvas::setDimensions(int width, int height) {
    VISAGE_ASSERT(state_memory_.empty());
    width = std::max(1, width);
    height = std::max(1, height);
    composite_layer_.setDimensions(width, height);
    window_region_.setBounds(0, 0, width, height);
    default_region_.setBounds(0, 0, width, height);
    setClampBounds(0, 0, width, height);
  }

  void Canvas::restoreLostContents() {
    static uint32_t font_generation = 0;
    uint32_t generation = gpu::contentsGeneration();
    if (font_generation != generation) {
      font_generation = generation;
      FontCache::releaseTextures();
    }

    gradient_atlas_.destroy();
    image_atlas_.destroyTexture();
    data_atlas_.destroyTexture();
    path_atlas_.invalidateAll();
    for (Layer* layer : layers_)
      layer->invalidateContents();
  }

  int Canvas::submit(int submit_pass) {
    if (gpu::deviceLost())
      advanceFrame();

    if (contents_generation_ != gpu::contentsGeneration()) {
      contents_generation_ = gpu::contentsGeneration();
      restoreLostContents();
    }

    beginBufferPoolFrame();
    for (int i = 1; i < layers_.size(); ++i)
      layers_[i]->updateHdr(*layers_[i - 1]);

    int backdrop_count = window_region_.computeBackdropCount();
    int submission = path_atlas_.updatePaths(submit_pass);

    for (int i = 2; i < layers_.size(); ++i)
      layers_[i]->invalidateBackdropsIfAllInvalidated();
    for (int i = 2; i < layers_.size(); ++i) {
      if (!layers_[1]->invalidRects().empty())
        layers_[i]->checkBackdropInvalidation(layers_[1]->invalidRects().begin()->second);
    }

    int layers_start = submission;
    int start_flushes = Layer::viewFlushes();
    for (int backdrop = 0; backdrop <= backdrop_count; ++backdrop) {
      for (int i = layers_.size() - 1; i > 0; --i)
        submission = layers_[i]->submit(submission, backdrop);
      if (submission == layers_start && Layer::viewFlushes() == start_flushes)
        break;
    }

    for (int i = 1; i < layers_.size(); ++i) {
      layers_[i]->clearInvalidRects();
      layers_[i]->destroyIdleTemporaryFrameBuffers();
    }

    if (submission > submit_pass || Layer::viewFlushes() != start_flushes) {
      composite_layer_.invalidate();
      submission = composite_layer_.submit(submission, backdrop_count);
      advanceFrame();
      endBufferPoolFrame();

      render_frame_++;
      FontCache::clearStaleFonts();
      gradient_atlas_.clearStaleGradients();
      image_atlas_.clearStaleImages();
      data_atlas_.clearStaleImages();
    }
    else if (last_skipped_frame_ != render_frame_) {
      last_skipped_frame_ = render_frame_;
      advanceFrame();
      endBufferPoolFrame();
    }
    return submission;
  }

  int Canvas::frameBufferId() const {
    gpu::FrameBufferHandle handle = composite_layer_.frameBuffer();
    return gpu::isValid(handle) ? handle.idx : -1;
  }

  bool Canvas::waitForFrame(int frame_buffer_id, int timeout_ms) {
    gpu::FrameBufferHandle handle = { static_cast<uint16_t>(frame_buffer_id) };
    return frame_buffer_id < 0 || gpu::waitForFrame(handle, timeout_ms);
  }

  const Screenshot& Canvas::takeScreenshot() {
    composite_layer_.requestScreenshot();
    default_region_.invalidate();
    submit();
    return composite_layer_.screenshot();
  }

  const Screenshot& Canvas::screenshot() const {
    return composite_layer_.screenshot();
  }

  void Canvas::ensureLayerExists(int layer) {
    int layers_to_add = layer + 1 - layers_.size();
    for (int i = 0; i < layers_to_add; ++i) {
      intermediate_layers_.push_back(std::make_unique<Layer>(&gradient_atlas_));
      intermediate_layers_.back()->setIntermediateLayer(true);
      layers_.push_back(intermediate_layers_.back().get());
    }
  }

  void Canvas::invalidateRectInRegion(IBounds rect, const Region* region, int layer) {
    ensureLayerExists(layer);
    layers_[layer]->invalidateRectInRegion(rect, region);
  }

  void Canvas::addToPackedLayer(Region* region, int layer_index) {
    if (layer_index == 0)
      return;

    ensureLayerExists(layer_index);
    layers_[layer_index]->addPackedRegion(region);
  }

  void Canvas::removeFromPackedLayer(const Region* region, int layer_index) {
    if (layer_index == 0)
      return;

    layers_[layer_index]->removePackedRegion(region);
  }

  void Canvas::changePackedLayer(Region* region, int from, int to) {
    removeFromPackedLayer(region, from);
    addToPackedLayer(region, to);
  }

  Brush Canvas::color(theme::ColorId color_id) {
    if (palette_) {
      Brush result;
      theme::OverrideId last_check;
      for (auto it = state_memory_.rbegin(); it != state_memory_.rend(); ++it) {
        theme::OverrideId override_id = it->palette_override;
        if (override_id.id != last_check.id && palette_->color(override_id, color_id, result))
          return result;
        last_check = override_id;
      }
      if (palette_->color({}, color_id, result))
        return result;
    }

    return Brush::solid(theme::ColorId::defaultColor(color_id));
  }

  float Canvas::value(theme::ValueId value_id) {
    if (palette_) {
      float result = 0.0f;
      theme::OverrideId last_check;
      for (auto it = state_memory_.rbegin(); it != state_memory_.rend(); ++it) {
        theme::OverrideId override_id = it->palette_override;
        if (override_id.id != last_check.id && palette_->value(override_id, value_id, result))
          return result;

        last_check = override_id;
      }
      if (palette_->value({}, value_id, result))
        return result;
    }

    return theme::ValueId::defaultValue(value_id);
  }

  std::vector<std::string> Canvas::debugInfo() const {
    const gpu::Caps* caps = gpu::getCaps();
    std::vector<std::string> result;
    result.push_back(std::string("Graphics API: ") + gpu::rendererName(caps->renderer));
    float hz = 1.0f / std::max(0.001f, refresh_time_);
    result.push_back("Refresh Rate : " + std::to_string(hz) + " Hz");

    const gpu::Stats* stats = gpu::getStats();
    result.push_back("Frame submit: " + std::to_string(stats->cpu_frame_ms) + " ms");
    result.push_back("Draw number: " + std::to_string(stats->num_draws));
    result.push_back("Num views: " + std::to_string(stats->num_views));
    result.push_back(std::string(caps->compute ? "YES" : "    ") +
                     " - Compute shaders are supported.");
    result.push_back(std::string(caps->swap_chain ? "YES" : "    ") +
                     " - Multiple windows are supported.");
    return result;
  }

  void Canvas::updateTime(double time) {
    static constexpr float kRefreshRateSlew = 0.3f;
    delta_time_ = std::max(0.0, time - render_time_);
    render_time_ = time;
    refresh_time_ = (std::min(delta_time_, 1.0) - refresh_time_) * kRefreshRateSlew + refresh_time_;

    for (Layer* layer : layers_)
      layer->setTime(time);
  }
}
