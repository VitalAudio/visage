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

#include "post_effects.h"

#include "canvas.h"
#include "embedded/shaders.h"
#include "graphics_caches.h"
#include "uniforms.h"

#include <bgfx/bgfx.h>
#include <optional>

namespace visage {
  template<const char* name>
  void setPostEffectUniform(float value0, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f) {
    float values[4] = { value0, value1, value2, value3 };
    const bgfx::UniformHandle& uniform = UniformCache::uniformHandle(name);
    bgfx::setUniform(uniform, values);
  }

  template<const char* name>
  void setPostEffectTexture(int stage, bgfx::TextureHandle handle, uint32_t flags = UINT32_MAX) {
    const bgfx::UniformHandle& uniform = UniformCache::uniformHandle(name, UniformCache::Sampler);
    bgfx::setTexture(stage, uniform, handle, flags);
  }

  static constexpr UvVertex kScreenVertices[kVerticesPerQuad] = { { -1.0f, 1.0f, 0.0f, 0.0f },
                                                                  { 1.0f, 1.0f, 1.0f, 0.0f },
                                                                  { -1.0f, -1.0f, 0.0f, 1.0f },
                                                                  { 1.0f, -1.0f, 1.0f, 1.0f } };
  static constexpr UvVertex kInvScreenVertices[kVerticesPerQuad] = { { -1.0f, 1.0f, 0.0f, 1.0f },
                                                                     { 1.0f, 1.0f, 1.0f, 1.0f },
                                                                     { -1.0f, -1.0f, 0.0f, 0.0f },
                                                                     { 1.0f, -1.0f, 1.0f, 0.0f } };

  struct ScreenBuffers {
    bgfx::IndexBufferHandle index_buffer = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle vertex_buffer = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle inv_vertex_buffer = BGFX_INVALID_HANDLE;
  };

  static ScreenBuffers screen_buffers;

  static const ScreenBuffers& screenBuffers() {
    if (!bgfx::isValid(screen_buffers.index_buffer)) {
      screen_buffers = {
        bgfx::createIndexBuffer(bgfx::makeRef(kQuadTriangles, sizeof(kQuadTriangles))),
        bgfx::createVertexBuffer(bgfx::makeRef(kScreenVertices, sizeof(kScreenVertices)),
                                 UvVertex::layout()),
        bgfx::createVertexBuffer(bgfx::makeRef(kInvScreenVertices, sizeof(kInvScreenVertices)),
                                 UvVertex::layout()),
      };
    }
    return screen_buffers;
  }

  void PostEffect::releaseScreenBuffers() {
    if (!bgfx::isValid(screen_buffers.index_buffer))
      return;

    bgfx::destroy(screen_buffers.index_buffer);
    bgfx::destroy(screen_buffers.vertex_buffer);
    bgfx::destroy(screen_buffers.inv_vertex_buffer);
    screen_buffers = {};
  }

  struct DownsampleHandles {
    std::optional<Renderer::Reference> renderer_reference;
    bgfx::FrameBufferHandle result = BGFX_INVALID_HANDLE;
    bgfx::FrameBufferHandle backdrop = BGFX_INVALID_HANDLE;

    ~DownsampleHandles() { destroyFrameBuffers(); }

    void destroyFrameBuffers() {
      if (bgfx::isValid(result))
        bgfx::destroy(result);
      result = BGFX_INVALID_HANDLE;
      renderer_reference.reset();
    }
  };

  struct BackdropHandles {
    bgfx::FrameBufferHandle copy = BGFX_INVALID_HANDLE;
  };

  void PostEffect::submitPassthrough(const BatchVector<SampleRegion>& batches,
                                     const Layer& destination, int submit_pass) const {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setPostEffectUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setBlendMode(BlendMode::Alpha);
    setPostEffectUniform<Uniforms::kTime>(destination.time());

    Layer* source_layer = batches[0].shapes->front().region->layer();
    float width_scale = 1.0f / source_layer->width();
    float height_scale = 1.0f / source_layer->height();
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
    setUniformDimensions(destination.width(), destination.height());
    setOriginFlipUniform(destination.bottomLeftOrigin());
    bgfx::submit(submit_pass, ProgramCache::programHandle(SampleRegion::vertexShader(),
                                                          SampleRegion::fragmentShader()));
  }

  DownsamplePostEffect::~DownsamplePostEffect() = default;

  DownsamplePostEffect::DownsamplePostEffect(bool hdr) : PostEffect(hdr) {
    handles_ = std::make_unique<DownsampleHandles>();
  }

  void DownsamplePostEffect::checkBuffers(const Region* region) {
    static constexpr uint64_t kFrameBufferFlags = BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP |
                                                  BGFX_SAMPLER_V_CLAMP;

    VISAGE_ASSERT(region_ == nullptr || region_ == region);
    region_ = region;

    int full_width = region->width();
    int full_height = region->height();
    bgfx::TextureFormat::Enum format = static_cast<bgfx::TextureFormat::Enum>(region->layer()->frameBufferFormat());

    if (full_width != full_width_ || full_height != full_height_ || format_ != format) {
      setLevelDimensions(full_width, full_height);
      format_ = format;
      handles_->destroyFrameBuffers();
    }

    if (!bgfx::isValid(handles_->result)) {
      handles_->renderer_reference.emplace();
      handles_->result = bgfx::createFrameBuffer(widths_[1], heights_[1], format, kFrameBufferFlags);
    }
  }

  void DownsamplePostEffect::setLevelDimensions(int width, int height) {
    full_width_ = width;
    full_height_ = height;
    for (int i = 0; i <= kMaxDownsamples; ++i) {
      int scale = 1 << i;
      widths_[i] = std::max(1, (width + scale - 1) / scale);
      heights_[i] = std::max(1, (height + scale - 1) / scale);
    }
  }

  static void setInitialVertices(Region* region, const IBounds& rect) {
    UvVertex* uv_data = initVertices<UvVertex>(kVerticesPerQuad);
    if (uv_data == nullptr)
      return;

    for (int i = 0; i < kVerticesPerQuad; ++i) {
      uv_data[i].x = kScreenVertices[i].x;
      uv_data[i].y = kScreenVertices[i].y;
    }

    float width_scale = 1.0f / region->layer()->width();
    float height_scale = 1.0f / region->layer()->height();
    IPoint position = region->layer()->coordinatesForRegion(region) + rect.topLeft();
    float left = position.x * width_scale;
    float top = position.y * height_scale;
    float right = left + rect.width() * width_scale;
    float bottom = top + rect.height() * height_scale;
    uv_data[0].u = left;
    uv_data[0].v = top;
    uv_data[1].u = right;
    uv_data[1].v = top;
    uv_data[2].u = left;
    uv_data[2].v = bottom;
    uv_data[3].u = right;
    uv_data[3].v = bottom;

    if (region->layer()->bottomLeftOrigin()) {
      uv_data[0].v = 1.0f - bottom;
      uv_data[1].v = 1.0f - bottom;
      uv_data[2].v = 1.0f - top;
      uv_data[3].v = 1.0f - top;
    }
  }

  void DownsamplePostEffect::setScreenVertexBuffer(bool inverted) {
    bgfx::setVertexBuffer(0, inverted ? screenBuffers().inv_vertex_buffer : screenBuffers().vertex_buffer);
  }

  static void setRegionTexturePositions(const BatchVector<SampleRegion>& batches,
                                        PostEffectVertex* vertices, const Region* source,
                                        IPoint origin, float scale_x, float scale_y) {
    for (const auto& batch : batches) {
      for (const SampleRegion& shape : *batch.shapes) {
        IPoint position = -origin;
        for (const Region* region = shape.region; region && region != source; region = region->parent())
          position += IPoint(region->x(), region->y());

        float left = position.x * scale_x;
        float top = position.y * scale_y;
        float right = (position.x + shape.width) * scale_x;
        float bottom = (position.y + shape.height) * scale_y;
        int pieces = numShapePieces(shape, batch.x, batch.y, *batch.invalid_rects);
        for (int i = 0; i < pieces; ++i) {
          vertices[0].texture_x = left;
          vertices[0].texture_y = top;
          vertices[1].texture_x = right;
          vertices[1].texture_y = top;
          vertices[2].texture_x = left;
          vertices[2].texture_y = bottom;
          vertices[3].texture_x = right;
          vertices[3].texture_y = bottom;
          vertices += kVerticesPerQuad;
        }
      }
    }
  }

  void DownsamplePostEffect::setTexturePositions(const BatchVector<SampleRegion>& batches,
                                                 PostEffectVertex* vertices, int level) const {
    setRegionTexturePositions(batches, vertices, region_, {}, widths_[level] * 1.0f / full_width_,
                              heights_[level] * 1.0f / full_height_);
  }

  static int compositeLayeredAncestors(const Region* region, const Region* source,
                                       const IBounds& target, bgfx::FrameBufferHandle destination,
                                       bgfx::IndexBufferHandle index_buffer, int submit_pass) {
    std::vector<const Region*> ancestors;
    for (const Region* parent = region->parent(); parent && parent != source; parent = parent->parent()) {
      if (parent->needsLayer())
        ancestors.push_back(parent);
    }

    for (auto it = ancestors.rbegin(); it != ancestors.rend(); ++it) {
      const Region* ancestor = *it;
      const Layer* layer = ancestor->layer();
      IBounds visible(0, 0, ancestor->width(), ancestor->height());
      IPoint position;
      for (const Region* child = ancestor; child != source; child = child->parent()) {
        IPoint offset(child->x(), child->y());
        position += offset;
        visible = (visible + offset).intersection({ 0, 0, child->parent()->width(), child->parent()->height() });
      }
      visible = visible.intersection(target);
      if (visible.width() <= 0 || visible.height() <= 0 || !bgfx::isValid(layer->frameBuffer()))
        continue;

      UvVertex* vertices = initVertices<UvVertex>(kVerticesPerQuad);
      if (vertices == nullptr)
        return submit_pass;

      bool flip = layer->bottomLeftOrigin();
      IPoint slot = layer->coordinatesForRegion(ancestor) - position;
      auto vertex_y = [&](int y) {
        float ratio = (y - target.y()) * 2.0f / target.height();
        return flip ? ratio - 1.0f : 1.0f - ratio;
      };
      auto texture_v = [&](int y) {
        float v = (slot.y + y) * 1.0f / layer->height();
        return flip ? 1.0f - v : v;
      };

      for (int i = 0; i < kVerticesPerQuad; ++i) {
        int x = i % 2 ? visible.right() : visible.x();
        int y = i / 2 ? visible.bottom() : visible.y();
        vertices[i].x = (x - target.x()) * 2.0f / target.width() - 1.0f;
        vertices[i].y = vertex_y(y);
        vertices[i].u = (slot.x + x) * 1.0f / layer->width();
        vertices[i].v = texture_v(y);
      }

      setBlendMode(BlendMode::Alpha);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(layer->frameBuffer()));
      setPostEffectUniform<Uniforms::kResampleValues>(1.0f, 1.0f);
      bgfx::setIndexBuffer(index_buffer);

      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, target.width(), target.height());
      bgfx::setViewClear(submit_pass, BGFX_CLEAR_NONE);
      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
      submit_pass++;
    }
    return submit_pass;
  }

  BackdropCopy::BackdropCopy() {
    handles_ = std::make_unique<BackdropHandles>();
  }

  BackdropCopy::~BackdropCopy() = default;

  int BackdropCopy::copy(Region* source, const Region* region, int submit_pass) {
    source_ = source;
    handles_->copy = BGFX_INVALID_HANDLE;

    IPoint position;
    for (const Region* child = region; child != source; child = child->parent())
      position += IPoint(child->x(), child->y());
    bounds_ = IBounds(0, 0, source->width(), source->height())
                  .intersection({ position.x, position.y, region->width(), region->height() });
    if (bounds_.width() <= 0 || bounds_.height() <= 0)
      return submit_pass;

    Layer* layer = source->layer();
    handles_->copy = layer->temporaryFrameBuffer(bounds_.width(), bounds_.height());
    setBlendMode(BlendMode::Opaque);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(layer->frameBuffer()));
    setPostEffectUniform<Uniforms::kPixelSize>(1.0f / bounds_.width(), 1.0f / bounds_.height());
    bgfx::setIndexBuffer(screenBuffers().index_buffer);

    bgfx::setViewFrameBuffer(submit_pass, handles_->copy);
    bgfx::setViewRect(submit_pass, 0, 0, bounds_.width(), bounds_.height());
    setInitialVertices(source, bounds_);
    setPostEffectUniform<Uniforms::kResampleValues>(1.0f, 1.0f);
    bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
    submit_pass++;

    return compositeLayeredAncestors(region, source, bounds_, handles_->copy,
                                     screenBuffers().index_buffer, submit_pass);
  }

  void BackdropCopy::release() {
    if (hasCopy())
      layer()->releaseTemporaryFrameBuffer(handles_->copy);
  }

  bool BackdropCopy::hasCopy() const {
    return bgfx::isValid(handles_->copy);
  }

  Layer* BackdropCopy::layer() const {
    return source_->layer();
  }

  bgfx::FrameBufferHandle BackdropCopy::frameBuffer() const {
    return handles_->copy;
  }

  void BackdropCopy::setUniforms(const BatchVector<SampleRegion>& batches,
                                 PostEffectVertex* vertices, bgfx::FrameBufferHandle texture) const {
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(texture));
    setRegionTexturePositions(batches, vertices, source_, bounds_.topLeft(), 1.0f, 1.0f);
    setOriginFlipUniform(false);
  }

  BlurPostEffect::BlurPostEffect() : DownsamplePostEffect(false) { }

  BlurPostEffect::~BlurPostEffect() = default;

  static void setRegionView(int submit_pass, Region* region) {
    Layer* layer = region->layer();
    IBounds bounds = layer->boundsForRegion(region);
    bgfx::setViewFrameBuffer(submit_pass, layer->frameBuffer());
    bgfx::setViewRect(submit_pass, bounds.x(), bounds.y(), bounds.width(), bounds.height());
    bgfx::setViewClear(submit_pass, BGFX_CLEAR_NONE);
  }

  int BlurPostEffect::preprocess(Region* region, int submit_pass) {
    return blur(region, submit_pass);
  }

  int BlurPostEffect::preprocessBackdrop(Region* source, const Region* region, int submit_pass) {
    handles_->backdrop = BGFX_INVALID_HANDLE;
    submit_pass = backdrop_.copy(source, region, submit_pass);
    if (!backdrop_.hasCopy())
      return submit_pass;

    submit_pass = blur(nullptr, submit_pass);
    backdrop_.release();
    return submit_pass;
  }

  int BlurPostEffect::blur(Region* region, int submit_pass) {
    static constexpr float kKernelSigma = 3.3f;
    static constexpr uint32_t kMirror = BGFX_SAMPLER_U_MIRROR | BGFX_SAMPLER_V_MIRROR;

    bool in_place = region != nullptr;
    float sigma = blur_radius_;
    if (in_place && sigma < kMinSigma)
      return submit_pass;

    IBounds rect(0, 0, backdrop_.width(), backdrop_.height());
    if (in_place)
      rect = IBounds(0, 0, region->width(), region->height());
    uint32_t sampler = in_place ? UINT32_MAX : kMirror;

    Layer* layer = in_place ? region->layer() : backdrop_.layer();
    bool inverted = layer->bottomLeftOrigin();
    setLevelDimensions(rect.width(), rect.height());

    auto kernel_sigma = [sigma](int stages) {
      float resample_variance = ((1 << (2 * stages + 2)) - 4) * 2.0f / 9.0f;
      return std::sqrt(std::max(0.0f, sigma * sigma - resample_variance)) / (1 << stages);
    };

    int stages = 0;
    while (sigma >= kMinSigma && stages < kMaxDownsamples && kernel_sigma(stages) > kKernelSigma)
      stages++;
    float transition = std::min(1.0f, kernel_sigma(stages) / kKernelSigma);

    bgfx::FrameBufferHandle levels[kMaxDownsamples + 1];
    int last_width = full_width_;
    int last_height = full_height_;

    bgfx::FrameBufferHandle source = layer->frameBuffer();
    if (!in_place) {
      levels[0] = backdrop_.frameBuffer();
      source = levels[0];
    }

    for (int i = in_place ? 0 : 1; i <= stages; ++i) {
      int downsample_width = widths_[i];
      int downsample_height = heights_[i];
      float x_downsample_scale = downsample_width * 2.0f / last_width;
      float y_downsample_scale = downsample_height * 2.0f / last_height;
      last_width = downsample_width;
      last_height = downsample_height;

      levels[i] = layer->temporaryFrameBuffer(downsample_width, downsample_height);
      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source), i ? sampler : UINT32_MAX);
      setPostEffectUniform<Uniforms::kPixelSize>(1.0f / last_width, 1.0f / last_height);
      bgfx::setIndexBuffer(screenBuffers().index_buffer);

      bgfx::setViewFrameBuffer(submit_pass, levels[i]);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);

      if (i == 0) {
        setInitialVertices(region, rect);
        setPostEffectUniform<Uniforms::kResampleValues>(1.0f, 1.0f);
        bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
      }
      else {
        setScreenVertexBuffer(inverted);
        setPostEffectUniform<Uniforms::kResampleValues>(x_downsample_scale, y_downsample_scale);
        bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_blur_sample));
      }

      submit_pass++;
      source = levels[i];
    }

    if (sigma >= kMinSigma) {
      bgfx::FrameBufferHandle scratch = layer->temporaryFrameBuffer(last_width, last_height);
      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source), sampler);
      setScreenVertexBuffer(inverted);

      bgfx::setIndexBuffer(screenBuffers().index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, scratch);
      bgfx::setViewRect(submit_pass, 0, 0, last_width, last_height);
      setPostEffectUniform<Uniforms::kPixelSize>(transition / last_width, 0.0f);
      bgfx::submit(submit_pass,
                   ProgramCache::programHandle(shaders::vs_full_screen_texture, shaders::fs_blur));
      submit_pass++;

      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(scratch), sampler);
      setScreenVertexBuffer(inverted && !(in_place && stages == 0));

      bgfx::setIndexBuffer(screenBuffers().index_buffer);
      if (in_place && stages == 0)
        setRegionView(submit_pass, region);
      else {
        bgfx::setViewFrameBuffer(submit_pass, levels[stages]);
        bgfx::setViewRect(submit_pass, 0, 0, last_width, last_height);
      }
      setPostEffectUniform<Uniforms::kPixelSize>(0.0f, transition / last_height);
      bgfx::submit(submit_pass,
                   ProgramCache::programHandle(shaders::vs_full_screen_texture, shaders::fs_blur));
      submit_pass++;
      layer->releaseTemporaryFrameBuffer(scratch);

      for (int i = stages; i > (in_place ? 0 : 1); --i) {
        int dest_width = widths_[i - 1];
        int dest_height = heights_[i - 1];

        setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(levels[i]), sampler);
        setPostEffectUniform<Uniforms::kResampleValues>(dest_width * 0.5f / widths_[i],
                                                        dest_height * 0.5f / heights_[i]);
        setScreenVertexBuffer(inverted && !(i == 1 && in_place));
        bgfx::setIndexBuffer(screenBuffers().index_buffer);
        if (i == 1 && in_place)
          setRegionView(submit_pass, region);
        else {
          bgfx::setViewFrameBuffer(submit_pass, levels[i - 1]);
          bgfx::setViewRect(submit_pass, 0, 0, dest_width, dest_height);
        }

        setBlendMode(BlendMode::Opaque);
        bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
        submit_pass++;
      }
    }

    handles_->backdrop = levels[stages ? 1 : 0];
    for (int i = in_place ? 0 : 1; i <= stages; ++i)
      layer->releaseTemporaryFrameBuffer(levels[i]);
    return submit_pass;
  }

  void BlurPostEffect::submitBackdrop(const BatchVector<SampleRegion>& batches, Layer& destination,
                                      int submit_pass) {
    if (!bgfx::isValid(handles_->backdrop))
      return;

    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setPostEffectUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setBlendMode(BlendMode::Alpha);
    setPostEffectUniform<Uniforms::kTime>(destination.time());
    setPostEffectUniform<Uniforms::kAtlasScale>(1.0f / backdrop_.width(), 1.0f / backdrop_.height());
    backdrop_.setUniforms(batches, quads.vertices, handles_->backdrop);
    setUniformDimensions(destination.width(), destination.height());
    bgfx::submit(submit_pass, ProgramCache::programHandle(SampleRegion::vertexShader(),
                                                          SampleRegion::fragmentShader()));
  }

  BloomPostEffect::BloomPostEffect() : DownsamplePostEffect(true) {
    VISAGE_ASSERT(!hasStaticStorage(this));
  }

  BloomPostEffect::~BloomPostEffect() = default;

  int BloomPostEffect::preprocess(Region* region, int submit_pass) {
    checkBuffers(region);

    float stages = std::max(std::floor(bloom_size_) + 0.99f, 0.0f);
    stages = std::max(1.0f, std::min(stages, kMaxDownsamples + 0.99f));
    downsamples_ = stages;

    Layer* layer = region->layer();
    bgfx::FrameBufferHandle levels[kMaxDownsamples + 1];
    bgfx::FrameBufferHandle scratch[kMaxDownsamples + 1];
    levels[1] = handles_->result;
    for (int i = 2; i <= downsamples_; ++i) {
      levels[i] = layer->temporaryFrameBuffer(widths_[i], heights_[i]);
      scratch[i] = layer->temporaryFrameBuffer(widths_[i], heights_[i]);
    }

    setBlendMode(BlendMode::Opaque);
    setInitialVertices(region, { 0, 0, region->width(), region->height() });
    setPostEffectUniform<Uniforms::kResampleValues>(1.0f, 1.0f);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(region->layer()->frameBuffer()));

    bgfx::setIndexBuffer(screenBuffers().index_buffer);
    bgfx::setViewFrameBuffer(submit_pass, levels[1]);
    bgfx::setViewRect(submit_pass, 0, 0, widths_[1], heights_[1]);
    setPostEffectUniform<Uniforms::kMult>(bloom_intensity_, bloom_intensity_, bloom_intensity_, 1.0f);
    setPostEffectUniform<Uniforms::kThreshold>(1.0f);

    bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_mult_threshold));
    submit_pass++;

    bgfx::FrameBufferHandle source = levels[1];
    for (int i = 1; i < downsamples_; ++i) {
      int downsample_width = widths_[i + 1];
      int downsample_height = heights_[i + 1];
      float x_downsample_scale = downsample_width * 2.0f / widths_[i];
      float y_downsample_scale = downsample_height * 2.0f / heights_[i];

      bgfx::FrameBufferHandle destination = levels[i + 1];
      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source));
      bgfx::setIndexBuffer(screenBuffers().index_buffer);
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      setPostEffectUniform<Uniforms::kResampleValues>(x_downsample_scale, y_downsample_scale);

      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);

      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
      submit_pass++;

      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(destination));
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      bgfx::setIndexBuffer(screenBuffers().index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, scratch[i + 1]);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
      setPostEffectUniform<Uniforms::kPixelSize>(1.0f / downsample_width);

      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_full_screen_texture,
                                                            shaders::fs_small_blur));
      submit_pass++;

      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(scratch[i + 1]));
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      bgfx::setIndexBuffer(screenBuffers().index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
      setPostEffectUniform<Uniforms::kPixelSize>(0.0f, 1.0f / downsample_height);
      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_full_screen_texture,
                                                            shaders::fs_small_blur));
      submit_pass++;

      source = destination;
    }

    for (int i = downsamples_ - 1; i > 0; --i) {
      bgfx::FrameBufferHandle destination = levels[i];
      int dest_width = widths_[i];
      int dest_height = heights_[i];

      bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_BLEND_ADD);

      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(levels[i + 1]));
      setPostEffectUniform<Uniforms::kResampleValues>(dest_width * 0.5f / widths_[i + 1],
                                                      dest_height * 0.5f / heights_[i + 1]);
      setPostEffectUniform<Uniforms::kMult>(2.0f, 2.0f, 2.0f, 1.0f);
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      bgfx::setIndexBuffer(screenBuffers().index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, dest_width, dest_height);

      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_mult));
      submit_pass++;
    }

    for (int i = 2; i <= downsamples_; ++i) {
      layer->releaseTemporaryFrameBuffer(levels[i]);
      layer->releaseTemporaryFrameBuffer(scratch[i]);
    }
    return submit_pass;
  }

  void BloomPostEffect::submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) {
    submitPassthrough(batches, destination, submit_pass);
    submitBloom(batches, destination, submit_pass);
  }

  void BloomPostEffect::submitBloom(const BatchVector<SampleRegion>& batches,
                                    const Layer& destination, int submit_pass) const {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setTexturePositions(batches, quads.vertices, 0);
    for (int i = 0; i < quads.num_shapes * kVerticesPerQuad; ++i) {
      quads.vertices[i].value1 = 1.0f;
      quads.vertices[i].value2 = 0.0f;
    }

    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_BLEND_ADD);

    float width_scale = 1.0f / widths_[0];
    float height_scale = 1.0f / heights_[0];
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    float mult = bloom_intensity_ * Color::kGradientNormalization;
    setPostEffectUniform<Uniforms::kColorMult>(mult, mult, mult, 1.0f);
    setAlphaClamp(destination);
    setPostEffectTexture<Uniforms::kGradient>(0, destination.gradientAtlas()->colorTextureHandle());
    setPostEffectTexture<Uniforms::kTexture>(1, bgfx::getTexture(handles_->result));
    setUniformDimensions(destination.width(), destination.height());
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shaders::vs_tinted_texture, shaders::fs_tinted_texture));
  }

  int ShaderPostEffect::preprocessBackdrop(Region* source, const Region* region, int submit_pass) {
    submit_pass = backdrop_.copy(source, region, submit_pass);
    backdrop_.release();
    return submit_pass;
  }

  void ShaderPostEffect::submit(const BatchVector<SampleRegion>& batches, Layer& destination,
                                int submit_pass) {
    submitShader(batches, destination, submit_pass, false);
  }

  void ShaderPostEffect::submitBackdrop(const BatchVector<SampleRegion>& batches,
                                        Layer& destination, int submit_pass) {
    if (backdrop_.hasCopy())
      submitShader(batches, destination, submit_pass, true);
  }

  void ShaderPostEffect::submitShader(const BatchVector<SampleRegion>& batches, Layer& destination,
                                      int submit_pass, bool backdrop) {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setPostEffectUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setPostEffectUniform<Uniforms::kTime>(destination.time());

    Layer* source_layer = batches[0].shapes->front().region->layer();
    float width_scale = 1.0f / (backdrop ? backdrop_.width() : source_layer->width());
    float height_scale = 1.0f / (backdrop ? backdrop_.height() : source_layer->height());
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    setPostEffectUniform<Uniforms::kColorMult>(1.0f, 1.0f, 1.0f, 1.0f);
    if (backdrop)
      backdrop_.setUniforms(batches, quads.vertices, backdrop_.frameBuffer());
    else {
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
      setOriginFlipUniform(destination.bottomLeftOrigin());
    }

    float min_x = (std::min(quads.vertices[0].texture_x, quads.vertices[3].texture_x) + 0.5f) * width_scale;
    float min_y = (std::min(quads.vertices[0].texture_y, quads.vertices[3].texture_y) + 0.5f) * height_scale;
    float max_x = (std::max(quads.vertices[0].texture_x, quads.vertices[3].texture_x) - 0.5f) * width_scale;
    float max_y = (std::max(quads.vertices[0].texture_y, quads.vertices[3].texture_y) - 0.5f) * height_scale;
    setPostEffectUniform<Uniforms::kTextureClamp>(min_x, min_y, max_x, max_y);
    float center_x = (quads.vertices[0].texture_x + quads.vertices[3].texture_x) * 0.5f;
    float center_y = (quads.vertices[0].texture_y + quads.vertices[3].texture_y) * 0.5f;
    setPostEffectUniform<Uniforms::kCenterPosition>(center_x, center_y);
    float width = std::abs(quads.vertices[3].texture_x - quads.vertices[0].texture_x);
    float height = std::abs(quads.vertices[3].texture_y - quads.vertices[0].texture_y);
    setPostEffectUniform<Uniforms::kDimensions>(width, height);
    setUniformDimensions(destination.width(), destination.height());

    for (const auto& uniform : uniforms_)
      bgfx::setUniform(UniformCache::uniformHandle(uniform.first.c_str()), uniform.second.data);

    submitBlended(state_, submit_pass, ProgramCache::programHandle(vertexShader(), fragmentShader()));
  }

  LuminanceMaskPostEffect::LuminanceMaskPostEffect() :
      ShaderPostEffect(shaders::vs_post_effect, shaders::fs_luminance_mask) {
    setState(BlendMode::MaskMult);
  }
}