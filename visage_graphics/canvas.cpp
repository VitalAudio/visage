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

#include "palette.h"
#include "renderer.h"
#include "theme.h"

#include <bgfx/bgfx.h>

namespace visage {
  bool Canvas::swapChainSupported() {
    return bgfx::getCaps()->supported & BGFX_CAPS_SWAP_CHAIN;
  }

  Canvas::Canvas() :
      image_atlas_(ImageAtlas::DataType::RGBA8), data_atlas_(ImageAtlas::DataType::Float32),
      composite_layer_(&gradient_atlas_) {
    state_.current_region = &default_region_;
    layers_.push_back(&composite_layer_);
    composite_layer_.addRegion(&window_region_);

    window_region_.setCanvas(this);
    window_region_.addRegion(&default_region_);
    default_region_.setCanvas(this);
    default_region_.setNeedsLayer(true);
  }

  void Canvas::clearDrawnShapes() {
    default_region_.clear();
    default_region_.invalidate();
    composite_layer_.clear();
    composite_layer_.addRegion(&window_region_);
  }

  void Canvas::setWindowless(int width, int height) {
    setDimensions(width, height);
    composite_layer_.setWindowlessRender(width, height);
    Renderer::instance().initializeWindowless();
    bgfx::reset(0, 0, BGFX_RESET_FLUSH_AFTER_RENDER | BGFX_RESET_FLIP_AFTER_RENDER);
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

  int Canvas::submit(int submit_pass) {
    default_region_.computeBackdropCount();
    int submission = submit_pass;
    int last_submission = submission - 1;

    submission = path_atlas_.updatePaths(submission);

    for (int i = 2; i < layers_.size(); ++i) {
      if (!layers_[1]->invalidRects().empty())
        layers_[i]->checkBackdropInvalidation(layers_[1]->invalidRects().begin()->second);
    }

    for (int backdrop = 0; submission != last_submission; backdrop++) {
      last_submission = submission;
      for (int i = layers_.size() - 1; i > 0; --i)
        submission = layers_[i]->submit(submission, backdrop);
    }

    for (int i = 1; i < layers_.size(); ++i)
      layers_[i]->clearInvalidRects();

    if (submission > submit_pass) {
      composite_layer_.invalidate();
      submission = composite_layer_.submit(submission, 0);
      bgfx::frame();
      if (render_frame_ == 0)
        bgfx::frame();

      render_frame_++;
      FontCache::clearStaleFonts();
      gradient_atlas_.clearStaleGradients();
      image_atlas_.clearStaleImages();
      data_atlas_.clearStaleImages();
    }
    else if (last_skipped_frame_ != render_frame_) {
      last_skipped_frame_ = render_frame_;
      bgfx::frame();
    }
    return submission;
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
    static const std::vector<std::pair<unsigned long long, std::string>> caps_list {
      { BGFX_CAPS_ALPHA_TO_COVERAGE, "Alpha to coverage is supported." },
      { BGFX_CAPS_BLEND_INDEPENDENT, "Blend independent is supported." },
      { BGFX_CAPS_COMPUTE, "Compute shaders are supported." },
      { BGFX_CAPS_CONSERVATIVE_RASTER, "Conservative rasterization is supported." },
      { BGFX_CAPS_DRAW_INDIRECT, "Draw indirect is supported." },
      { BGFX_CAPS_FRAGMENT_DEPTH, "Fragment depth is available in fragment shader." },
      { BGFX_CAPS_FRAGMENT_ORDERING, "Fragment ordering is available in fragment shader." },
      { BGFX_CAPS_GRAPHICS_DEBUGGER, "Graphics debugger is present." },
      { BGFX_CAPS_HDR10, "HDR10 rendering is supported." },
      { BGFX_CAPS_HIDPI, "HiDPI rendering is supported." },
      { BGFX_CAPS_IMAGE_RW, "Image Read/Write is supported." },
      { BGFX_CAPS_INDEX32, "32-bit indices are supported." },
      { BGFX_CAPS_INSTANCING, "Instancing is supported." },
      { BGFX_CAPS_OCCLUSION_QUERY, "Occlusion query is supported." },
      { BGFX_CAPS_RENDERER_MULTITHREADED, "Renderer is on separate thread." },
      { BGFX_CAPS_SWAP_CHAIN, "Multiple windows are supported." },
      { BGFX_CAPS_TEXTURE_2D_ARRAY, "2D texture array is supported." },
      { BGFX_CAPS_TEXTURE_3D, "3D textures are supported." },
      { BGFX_CAPS_TEXTURE_BLIT, "Texture blit is supported." },
      { BGFX_CAPS_TEXTURE_COMPARE_LEQUAL, "Texture compare less equal mode is supported." },
      { BGFX_CAPS_TEXTURE_CUBE_ARRAY, "Cubemap texture array is supported." },
      { BGFX_CAPS_TEXTURE_DIRECT_ACCESS, "CPU direct access to GPU texture memory." },
      { BGFX_CAPS_TEXTURE_READ_BACK, "Read-back texture is supported." },
      { BGFX_CAPS_VERTEX_ATTRIB_HALF, "Vertex attribute half-float is supported." },
      { BGFX_CAPS_VERTEX_ATTRIB_UINT10, "Vertex attribute 10_10_10_2 is supported." },
      { BGFX_CAPS_VERTEX_ID, "Rendering with VertexID only is supported." },
      { BGFX_CAPS_VIEWPORT_LAYER_ARRAY, "Viewport layer is available in vertex shader." },
    };

    const bgfx::Caps* caps = bgfx::getCaps();
    std::vector<std::string> result;
    result.push_back(std::string("Graphics API: ") + bgfx::getRendererName(caps->rendererType));
    float hz = 1.0f / std::max(0.001f, refresh_time_);
    result.push_back("Refresh Rate : " + std::to_string(hz) + " Hz");

    const bgfx::Stats* stats = bgfx::getStats();
    result.push_back("Render wait: " + std::to_string(stats->waitRender));
    result.push_back("Submit wait: " + std::to_string(stats->waitSubmit));
    result.push_back("Draw number: " + std::to_string(stats->numDraw));
    result.push_back("Num views: " + std::to_string(stats->numViews));

    for (auto& cap : caps_list) {
      if (caps->supported & cap.first)
        result.push_back("YES - " + cap.second);
      else
        result.push_back("    - " + cap.second);
    }

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
