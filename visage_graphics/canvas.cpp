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

#include "canvas.h"

#include "graphics_libs.h"
#include "palette.h"
#include "renderer.h"
#include "theme.h"

namespace visage {
  struct FrameBufferData {
    bgfx::FrameBufferHandle handle = BGFX_INVALID_HANDLE;
    bgfx::TextureFormat::Enum format = bgfx::TextureFormat::RGBA8;
  };

  struct RegionPosition {
    RegionPosition(Canvas::Region* region, std::vector<Bounds> invalid_rects, int position,
                   int x = 0, int y = 0) :
        region(region), invalid_rects(std::move(invalid_rects)), position(position), x(x), y(y) { }
    RegionPosition() = default;

    Canvas::Region* region = nullptr;
    std::vector<Bounds> invalid_rects;
    int position = 0;
    int x = 0;
    int y = 0;

    SubmitBatch* currentBatch() const { return region->submitBatchAtPosition(position); }
    bool isDone() const { return position >= region->numSubmitBatches(); }
  };

  Canvas::Canvas() {
    frame_buffer_data_ = std::make_unique<FrameBufferData>();
    base_region_.setCanvas(this);
    state_.current_region = &base_region_;
  }

  Canvas::~Canvas() {
    destroyFrameBuffer();
  }

  void Canvas::clearDrawnShapes() {
    base_region_.clearAll();
    invalidate();
  }

  void Canvas::pairToWindow(void* window_handle, int width, int height) {
    window_handle_ = window_handle;
    setDimensions(width, height);
    invalidate();
    destroyFrameBuffer();
  }

  void Canvas::removeFromWindow() {
    window_handle_ = nullptr;
    destroyFrameBuffer();
  }

  void Canvas::setHdr(bool hdr) {
    hdr_ = hdr;
    destroyFrameBuffer();
  }

  inline void moveToVector(std::vector<Bounds>& rects, std::vector<Bounds>& pieces) {
    rects.insert(rects.end(), pieces.begin(), pieces.end());
    pieces.clear();
  }

  void Canvas::invalidateRect(Bounds rect) {
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

  void Canvas::setDimensions(int width, int height) {
    VISAGE_ASSERT(state_memory_.empty());
    width = std::max(1, width);
    height = std::max(1, height);
    if (width == width_ && height == height_)
      return;

    base_region_.width_ = width;
    base_region_.height_ = height;
    width_ = width;
    height_ = height;
    setClampBounds(0, 0, width, height);

    if (icon_group_)
      icon_group_->clear();
    if (image_group_)
      image_group_->clear();

    images_.clear();
    destroyFrameBuffer();
    invalidate();
  }

  bgfx::FrameBufferHandle& Canvas::frameBuffer() const {
    return frame_buffer_data_->handle;
  }

  int Canvas::frameBufferFormat() const {
    return frame_buffer_data_->format;
  }

  const void* nextBatchId(const std::vector<RegionPosition>& positions, const void* current_batch_id) {
    const void* next_batch_id = positions[0].currentBatch()->id();
    for (auto& position : positions) {
      const void* batch_id = position.currentBatch()->id();
      if (batch_id < next_batch_id) {
        if (batch_id > current_batch_id || next_batch_id < current_batch_id)
          next_batch_id = batch_id;
      }
      else if (next_batch_id < current_batch_id && batch_id > current_batch_id)
        next_batch_id = batch_id;
    }

    return next_batch_id;
  }

  void addSubRegions(std::vector<RegionPosition>& positions,
                     std::vector<RegionPosition>& overlapping, const RegionPosition& done_position) {
    auto begin = done_position.region->subRegions().cbegin();
    auto end = done_position.region->subRegions().cend();
    for (auto it = begin; it != end; ++it) {
      Canvas::Region* sub_region = *it;
      if (!sub_region->isVisible())
        continue;

      bool overlaps = std::any_of(begin, it, [sub_region](const Canvas::Region* other) {
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

  void checkOverlappingRegions(std::vector<RegionPosition>& positions,
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

  int Canvas::submit(int submit_pass) {
    submit_pass = submitExternalCanvases(submit_pass);

    if (invalid_rects_.empty())
      return submit_pass;

    render_frame_++;
    checkFrameBuffer();
    submit_pass = redrawImages(submit_pass);

    bgfx::setViewMode(submit_pass, bgfx::ViewMode::Sequential);
    bgfx::setViewRect(submit_pass, 0, 0, width_, height_);
    if (bgfx::isValid(frame_buffer_data_->handle))
      bgfx::setViewFrameBuffer(submit_pass, frame_buffer_data_->handle);

    std::vector<RegionPosition> region_positions;
    std::vector<RegionPosition> overlapping_regions;
    if (base_region_.isEmpty()) {
      addSubRegions(region_positions, overlapping_regions,
                    { &base_region_, std::move(invalid_rects_), 0, 0, 0 });
    }
    else
      region_positions.emplace_back(&base_region_, std::move(invalid_rects_), 0);

    const void* current_batch_id = nullptr;
    std::vector<PositionedBatch> batches;
    std::vector<RegionPosition> done_regions;

    while (!region_positions.empty()) {
      const void* next_batch_id = nextBatchId(region_positions, current_batch_id);
      for (auto& region_position : region_positions) {
        SubmitBatch* batch = region_position.currentBatch();
        if (batch->id() != next_batch_id)
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
      current_batch_id = next_batch_id;
    }

    return submit_pass + 1;
  }

  void Canvas::render() {
    bgfx::frame();
  }

  QuadColor Canvas::color(unsigned int color_id) {
    if (palette_) {
      QuadColor result;
      int last_check = -1;
      for (auto it = state_memory_.rbegin(); it != state_memory_.rend(); ++it) {
        int override_id = it->palette_override;
        if (override_id != last_check && palette_->color(override_id, color_id, result))
          return result;
        last_check = override_id;
      }
      if (palette_->color(0, color_id, result))
        return result;
    }

    return theme::ColorId::defaultColor(color_id);
  }

  float Canvas::value(unsigned int value_id) {
    float scale = 1.0f;
    theme::ValueId::ValueIdInfo info = theme::ValueId::info(value_id);
    if (info.scale_type == theme::ValueId::kScaledWidth)
      scale = width_scale_;
    else if (info.scale_type == theme::ValueId::kScaledHeight)
      scale = height_scale_;
    else if (info.scale_type == theme::ValueId::kScaledDpi)
      scale = dpi_scale_;

    if (palette_) {
      float result = 0.0f;
      int last_check = -1;
      for (auto it = state_memory_.rbegin(); it != state_memory_.rend(); ++it) {
        int override_id = it->palette_override;
        if (override_id != last_check && palette_->value(override_id, value_id, result))
          return scale * result;

        last_check = override_id;
      }
      if (palette_->value(0, value_id, result))
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

  int Canvas::redrawImages(int submit_pass) {
    if (images_.empty())
      return submit_pass;

    return imageGroup()->addImages(images_, submit_pass);
  }

  int Canvas::submitExternalCanvases(int submit_pass, Canvas::Region* region) {
    for (auto& external : region->external_canvases_) {
      external.canvas->updateTime(render_time_);
      int new_submit_pass = external.canvas->submit(submit_pass);
      if (new_submit_pass != submit_pass) {
        submit_pass = external.post_effect->preprocess(*external.canvas, new_submit_pass);
        region->invalidateRect(Bounds(external.x, external.y, external.width, external.height));
      }
    }

    for (Region* sub_region : region->subRegions())
      submit_pass = submitExternalCanvases(submit_pass, sub_region);

    return submit_pass;
  }

  int Canvas::submitExternalCanvases(int submit_pass) {
    return submitExternalCanvases(submit_pass, &base_region_);
  }

  void Canvas::updateTime(double time) {
    static constexpr float kRefreshRateSlew = 0.3f;
    delta_time_ = std::max(0.0, time - render_time_);
    render_time_ = time;
    refresh_rate_ = (std::min(delta_time_, 1.0) - refresh_rate_) * kRefreshRateSlew + refresh_rate_;
  }

  void Canvas::checkFrameBuffer() {
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

  void Canvas::destroyFrameBuffer() const {
    if (bgfx::isValid(frame_buffer_data_->handle)) {
      bgfx::destroy(frame_buffer_data_->handle);
      frame_buffer_data_->handle = BGFX_INVALID_HANDLE;

      bgfx::frame();
      bgfx::frame();
    }
  }
}
