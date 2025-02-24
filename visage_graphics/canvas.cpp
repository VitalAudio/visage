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
#include "theme.h"

#include <bgfx/bgfx.h>

namespace visage {
  Canvas::Canvas() : composite_layer_(&gradient_atlas_) {
    state_.current_region = &default_region_;
    layers_.push_back(&composite_layer_);
    composite_layer_.addRegion(&window_region_);

    window_region_.setCanvas(this);
    window_region_.addRegion(&default_region_);
    default_region_.setCanvas(this);
    default_region_.setNeedsLayer(true);
  }

  void Canvas::clearDrawnShapes() {
    composite_layer_.clear();
    composite_layer_.addRegion(&window_region_);
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
    int submission = submit_pass;
    for (int i = layers_.size() - 1; i > 0; --i)
      submission = layers_[i]->submit(submission);

    if (submission > submit_pass) {
      composite_layer_.invalidate();
      submission = composite_layer_.submit(submission);
      render_frame_++;
      bgfx::frame();
      FontCache::clearStaleFonts();
      gradient_atlas_.clearStaleGradients();
      image_atlas_.clearStaleImages();
    }
    return submission;
  }

  void Canvas::takeScreenshot(const std::string& filename) {
    bgfx::requestScreenShot(composite_layer_.frameBuffer(), filename.c_str());
  }

  void Canvas::ensureLayerExists(int layer) {
    int layers_to_add = layer + 1 - layers_.size();
    for (int i = 0; i < layers_to_add; ++i) {
      intermediate_layers_.push_back(std::make_unique<Layer>(&gradient_atlas_));
      intermediate_layers_.back()->setIntermediateLayer(true);
      layers_.push_back(intermediate_layers_.back().get());
    }
  }

  void Canvas::invalidateRectInRegion(Bounds rect, Region* region, int layer) {
    ensureLayerExists(layer);
    layers_[layer]->invalidateRectInRegion(rect, region);
  }

  void Canvas::addToPackedLayer(Region* region, int layer_index) {
    if (layer_index == 0)
      return;

    ensureLayerExists(layer_index);
    layers_[layer_index]->addPackedRegion(region);
  }

  void Canvas::removeFromPackedLayer(Region* region, int layer_index) {
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
    float scale = 1.0f;
    theme::ValueId::ValueIdInfo info = theme::ValueId::info(value_id);
    if (info.scale_type == theme::ValueId::ScaleType::ScaledWidth)
      scale = width_scale_;
    else if (info.scale_type == theme::ValueId::ScaleType::ScaledHeight)
      scale = height_scale_;
    else if (info.scale_type == theme::ValueId::ScaleType::ScaledDpi)
      scale = dpi_scale_;

    if (palette_) {
      float result = 0.0f;
      theme::OverrideId last_check;
      for (auto it = state_memory_.rbegin(); it != state_memory_.rend(); ++it) {
        theme::OverrideId override_id = it->palette_override;
        if (override_id.id != last_check.id && palette_->value(override_id, value_id, result))
          return scale * result;

        last_check = override_id;
      }
      if (palette_->value({}, value_id, result))
        return scale * result;
    }

    return scale * theme::ValueId::defaultValue(value_id);
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
    result.push_back(std::string("Grapihcs API: ") + bgfx::getRendererName(caps->rendererType));
    float hz = 1.0f / std::max(0.001f, refresh_rate_);
    result.push_back("Refresh Rate : " + std::to_string(hz) + " Hz");

    result.push_back("UI Scaling: " + std::to_string(width_scale_) + " : " + std::to_string(height_scale_));

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
    refresh_rate_ = (std::min(delta_time_, 1.0) - refresh_rate_) * kRefreshRateSlew + refresh_rate_;

    for (Layer* layer : layers_)
      layer->setTime(time);
  }
}
