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

namespace visage {
  template<const char* name>
  void setPostEffectUniform(float value0, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f) {
    float values[4] = { value0, value1, value2, value3 };
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    bgfx::setUniform(uniform, values);
  }

  template<const char* name>
  void setPostEffectTexture(int stage, bgfx::TextureHandle handle) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Sampler, 1);
    bgfx::setTexture(stage, uniform, handle);
  }

  struct DownsampleHandles {
    bgfx::IndexBufferHandle screen_index_buffer = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle screen_vertex_buffer = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle inv_screen_vertex_buffer = BGFX_INVALID_HANDLE;
    bgfx::FrameBufferHandle downsample_buffers1[DownsamplePostEffect::kMaxDownsamples + 1] {};
    bgfx::FrameBufferHandle downsample_buffers2[DownsamplePostEffect::kMaxDownsamples + 1] {};

    ~DownsampleHandles() { destroy(); }

    void destroy() {
      if (bgfx::isValid(screen_index_buffer))
        bgfx::destroy(screen_index_buffer);
      if (bgfx::isValid(screen_vertex_buffer))
        bgfx::destroy(screen_vertex_buffer);
      destroyFrameBuffers();
    }

    void destroyFrameBuffers() {
      for (auto& buffer : downsample_buffers1) {
        if (bgfx::isValid(buffer))
          bgfx::destroy(buffer);
        buffer = BGFX_INVALID_HANDLE;
      }
      for (auto& buffer : downsample_buffers2) {
        if (bgfx::isValid(buffer))
          bgfx::destroy(buffer);
        buffer = BGFX_INVALID_HANDLE;
      }

      bgfx::frame();
      bgfx::frame();
    }
  };

  void PostEffect::submitPassthrough(const BatchVector<SampleRegion>& batches,
                                     const Layer& destination, int submit_pass) const {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setPostEffectUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setBlendMode(BlendMode::Composite);
    setPostEffectUniform<Uniforms::kTime>(destination.time());

    Layer* source_layer = batches[0].shapes->front().region->layer();
    float width_scale = 1.0f / source_layer->width();
    float height_scale = 1.0f / source_layer->height();
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
    setUniformDimensions(destination.width(), destination.height());
    float value = hdr() ? 1.0f / kHdrColorMultiplier : 1.0f;
    setPostEffectUniform<Uniforms::kColorMult>(value, value, value, 1.0f);
    setOriginFlipUniform(destination.bottomLeftOrigin());
    bgfx::submit(submit_pass, ProgramCache::programHandle(SampleRegion::vertexShader(),
                                                          SampleRegion::fragmentShader()));
  }

  DownsamplePostEffect::~DownsamplePostEffect() = default;

  DownsamplePostEffect::DownsamplePostEffect(bool hdr) : PostEffect(hdr) {
    handles_ = std::make_unique<DownsampleHandles>();

    for (auto& buffer : handles_->downsample_buffers1)
      buffer = BGFX_INVALID_HANDLE;

    for (auto& buffer : handles_->downsample_buffers2)
      buffer = BGFX_INVALID_HANDLE;

    screen_vertices_[0].x = -1.0f;
    screen_vertices_[0].y = 1.0f;
    screen_vertices_[1].x = 1.0f;
    screen_vertices_[1].y = 1.0f;
    screen_vertices_[2].x = -1.0f;
    screen_vertices_[2].y = -1.0f;
    screen_vertices_[3].x = 1.0f;
    screen_vertices_[3].y = -1.0f;

    for (int i = 0; i < kVerticesPerQuad; ++i) {
      screen_vertices_[i].u = screen_vertices_[i].x * 0.5f + 0.5f;
      screen_vertices_[i].v = screen_vertices_[i].y * -0.5f + 0.5f;
      inv_screen_vertices_[i].x = screen_vertices_[i].x;
      inv_screen_vertices_[i].y = screen_vertices_[i].y;
      inv_screen_vertices_[i].u = screen_vertices_[i].u;
      inv_screen_vertices_[i].v = screen_vertices_[i].y * 0.5f + 0.5f;
    }
  }

  void DownsamplePostEffect::checkBuffers(const Region* region, bool full_resolution) {
    static constexpr uint64_t kFrameBufferFlags = BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP |
                                                  BGFX_SAMPLER_V_CLAMP;

    int full_width = region->width();
    int full_height = region->height();
    bgfx::TextureFormat::Enum format = static_cast<bgfx::TextureFormat::Enum>(region->layer()->frameBufferFormat());

    if (!bgfx::isValid(handles_->screen_index_buffer)) {
      handles_->screen_index_buffer = bgfx::createIndexBuffer(bgfx::makeRef(visage::kQuadTriangles,
                                                                            sizeof(visage::kQuadTriangles)));
      const bgfx::Memory* vertex_memory = bgfx::makeRef(screen_vertices_, sizeof(screen_vertices_));
      handles_->screen_vertex_buffer = bgfx::createVertexBuffer(vertex_memory, UvVertex::layout());
      const bgfx::Memory* inv_vertex_memory = bgfx::makeRef(inv_screen_vertices_,
                                                            sizeof(inv_screen_vertices_));
      handles_->inv_screen_vertex_buffer = bgfx::createVertexBuffer(inv_vertex_memory, UvVertex::layout());
    }

    if (full_width != full_width_ || full_height != full_height_ || format_ != format) {
      full_width_ = full_width;
      full_height_ = full_height;
      format_ = format;
      handles_->destroyFrameBuffers();
    }

    if (!bgfx::isValid(handles_->downsample_buffers1[1])) {
      int start_index = full_resolution ? 0 : 1;
      for (int i = start_index; i <= kMaxDownsamples; ++i) {
        int scale = 1 << i;
        widths_[i] = std::max(1, (full_width + scale - 1) / scale);
        heights_[i] = std::max(1, (full_height + scale - 1) / scale);
        handles_->downsample_buffers1[i] = bgfx::createFrameBuffer(widths_[i], heights_[i], format,
                                                                   kFrameBufferFlags);
        handles_->downsample_buffers2[i] = bgfx::createFrameBuffer(widths_[i], heights_[i], format,
                                                                   kFrameBufferFlags);
      }
    }
  }

  void DownsamplePostEffect::setInitialVertices(Region* region) {
    bgfx::TransientVertexBuffer first_sample_buffer {};
    bgfx::allocTransientVertexBuffer(&first_sample_buffer, 4, UvVertex::layout());
    UvVertex* uv_data = reinterpret_cast<UvVertex*>(first_sample_buffer.data);
    if (uv_data == nullptr)
      return;

    for (int i = 0; i < kVerticesPerQuad; ++i) {
      uv_data[i].x = screen_vertices_[i].x;
      uv_data[i].y = screen_vertices_[i].y;
    }

    float width_scale = 1.0f / region->layer()->width();
    float height_scale = 1.0f / region->layer()->height();
    IPoint position = region->layer()->coordinatesForRegion(region);
    float left = position.x * width_scale;
    float top = position.y * height_scale;
    float right = left + region->width() * width_scale;
    float bottom = top + region->height() * height_scale;
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

    bgfx::setVertexBuffer(0, &first_sample_buffer);
  }

  void DownsamplePostEffect::setScreenVertexBuffer(bool inverted) {
    bgfx::setVertexBuffer(0, inverted ? handles_->inv_screen_vertex_buffer : handles_->screen_vertex_buffer);
  }

  BlurPostEffect::BlurPostEffect() : DownsamplePostEffect(false) { }

  BlurPostEffect::~BlurPostEffect() = default;

  int BlurPostEffect::preprocess(Region* region, int submit_pass) {
    static constexpr float kMaxSigma = 4.0f;
    checkBuffers(region, true);

    sigma_ = blur_radius_;
    if (sigma_ < kMinSigma)
      return submit_pass;

    float adjusted_sigma = sigma_;
    downsample_stages_ = 0;
    while (adjusted_sigma > kMaxSigma) {
      adjusted_sigma *= 0.5f;
      downsample_stages_ += 1;
    }
    downsample_stages_ = std::min(kMaxDownsamples, downsample_stages_);
    float transition = adjusted_sigma / kMaxSigma;

    int last_width = full_width_;
    int last_height = full_height_;

    bgfx::FrameBufferHandle source = region->layer()->frameBuffer();
    for (int i = 0; i <= downsample_stages_; ++i) {
      int downsample_width = widths_[i];
      int downsample_height = heights_[i];
      float x_downsample_scale = downsample_width * 2.0f / last_width;
      float y_downsample_scale = downsample_height * 2.0f / last_height;
      last_width = downsample_width;
      last_height = downsample_height;

      bgfx::FrameBufferHandle destination = handles_->downsample_buffers1[i];
      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source));
      setPostEffectUniform<Uniforms::kPixelSize>(1.0f / last_width, 1.0f / last_height);
      bgfx::setIndexBuffer(handles_->screen_index_buffer);

      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);

      if (i == 0) {
        setInitialVertices(region);
        setPostEffectUniform<Uniforms::kResampleValues>(1.0f, 1.0f);
        bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
      }
      else {
        setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
        setPostEffectUniform<Uniforms::kResampleValues>(x_downsample_scale, y_downsample_scale);
        bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_blur_sample));
      }

      submit_pass++;
      source = destination;
    }

    setBlendMode(BlendMode::Opaque);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source));
    setScreenVertexBuffer(region->layer()->bottomLeftOrigin());

    bgfx::setIndexBuffer(handles_->screen_index_buffer);
    bgfx::setViewFrameBuffer(submit_pass, handles_->downsample_buffers2[downsample_stages_]);
    bgfx::setViewRect(submit_pass, 0, 0, last_width, last_height);
    setPostEffectUniform<Uniforms::kPixelSize>(transition / last_width, 0.0f);
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shaders::vs_full_screen_texture, shaders::fs_blur));
    submit_pass++;

    setBlendMode(BlendMode::Opaque);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers2[downsample_stages_]));
    setScreenVertexBuffer(region->layer()->bottomLeftOrigin());

    bgfx::setIndexBuffer(handles_->screen_index_buffer);
    bgfx::setViewFrameBuffer(submit_pass, handles_->downsample_buffers1[downsample_stages_]);
    bgfx::setViewRect(submit_pass, 0, 0, last_width, last_height);
    setPostEffectUniform<Uniforms::kPixelSize>(0.0f, transition / last_height);
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shaders::vs_full_screen_texture, shaders::fs_blur));
    submit_pass++;

    for (int i = downsample_stages_; i > 1; --i) {
      int dest_width = widths_[i - 1];
      int dest_height = heights_[i - 1];

      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers1[i]));
      setPostEffectUniform<Uniforms::kResampleValues>(dest_width * 0.5f / widths_[i],
                                                      dest_height * 0.5f / heights_[i]);
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, handles_->downsample_buffers1[i - 1]);
      bgfx::setViewRect(submit_pass, 0, 0, dest_width, dest_height);

      setBlendMode(BlendMode::Opaque);
      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
      submit_pass++;
    }

    return submit_pass;
  }

  void BlurPostEffect::submit(const BatchVector<SampleRegion>& batches, Layer& destination, int submit_pass) {
    if (sigma_ < kMinSigma) {
      submitPassthrough(batches, destination, submit_pass);
      return;
    }
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setPostEffectUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setBlendMode(BlendMode::Composite);
    setPostEffectUniform<Uniforms::kTime>(destination.time());

    float width_scale = 1.0f / widths_[0];
    float height_scale = 1.0f / heights_[0];
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);

    int source_index = downsample_stages_ ? 1 : 0;
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers1[source_index]));

    for (int i = 0; i < quads.num_shapes; ++i) {
      quads.vertices[i * kVerticesPerQuad].texture_x = 0.0f;
      quads.vertices[i * kVerticesPerQuad].texture_y = 0.0f;
      quads.vertices[i * kVerticesPerQuad + 1].texture_x = widths_[0];
      quads.vertices[i * kVerticesPerQuad + 1].texture_y = 0.0f;
      quads.vertices[i * kVerticesPerQuad + 2].texture_x = 0.0f;
      quads.vertices[i * kVerticesPerQuad + 2].texture_y = heights_[0];
      quads.vertices[i * kVerticesPerQuad + 3].texture_x = widths_[0];
      quads.vertices[i * kVerticesPerQuad + 3].texture_y = heights_[0];
    }
    setUniformDimensions(destination.width(), destination.height());

    float value = destination.hdr() ? kHdrColorMultiplier : 1.0f;
    setPostEffectUniform<Uniforms::kColorMult>(value, value, value, 1.0f);
    setOriginFlipUniform(false);
    bgfx::submit(submit_pass, ProgramCache::programHandle(SampleRegion::vertexShader(),
                                                          SampleRegion::fragmentShader()));
  }

  BloomPostEffect::BloomPostEffect() : DownsamplePostEffect(true) { }

  BloomPostEffect::~BloomPostEffect() = default;

  int BloomPostEffect::preprocess(Region* region, int submit_pass) {
    checkBuffers(region, false);

    float hdr_range = hdr() ? kHdrColorRange : 1.0f;
    float stages = std::max(std::floor(bloom_size_) + 0.99f, 0.0f);
    stages = std::max(1.0f, std::min(stages, kMaxDownsamples + 0.99f));
    downsamples_ = stages;

    setBlendMode(BlendMode::Opaque);
    setInitialVertices(region);
    setPostEffectUniform<Uniforms::kResampleValues>(1.0f, 1.0f);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(region->layer()->frameBuffer()));

    bgfx::setIndexBuffer(handles_->screen_index_buffer);
    bgfx::setViewFrameBuffer(submit_pass, handles_->downsample_buffers1[1]);
    bgfx::setViewRect(submit_pass, 0, 0, widths_[1], heights_[1]);
    float mult_val = hdr_range * bloom_intensity_;
    setPostEffectUniform<Uniforms::kMult>(mult_val, mult_val, mult_val, 1.0f);
    float hdr_mult = hdr() ? kHdrColorMultiplier : 1.0f;
    setPostEffectUniform<Uniforms::kThreshold>(hdr_mult);

    bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_mult_threshold));
    submit_pass++;

    bgfx::FrameBufferHandle source = handles_->downsample_buffers1[1];
    for (int i = 1; i < downsamples_; ++i) {
      int downsample_width = widths_[i + 1];
      int downsample_height = heights_[i + 1];
      float x_downsample_scale = downsample_width * 2.0f / widths_[i];
      float y_downsample_scale = downsample_height * 2.0f / heights_[i];

      bgfx::FrameBufferHandle destination = handles_->downsample_buffers1[i + 1];
      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source));
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      setPostEffectUniform<Uniforms::kResampleValues>(x_downsample_scale, y_downsample_scale);

      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);

      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_sample));
      submit_pass++;

      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(destination));
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, handles_->downsample_buffers2[i + 1]);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
      setPostEffectUniform<Uniforms::kPixelSize>(1.0f / downsample_width);

      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_full_screen_texture,
                                                            shaders::fs_small_blur));
      submit_pass++;

      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers2[i + 1]));
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
      setPostEffectUniform<Uniforms::kPixelSize>(0.0f, 1.0f / downsample_height);
      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_full_screen_texture,
                                                            shaders::fs_small_blur));
      submit_pass++;

      source = destination;
    }

    for (int i = downsamples_ - 1; i > 0; --i) {
      bgfx::FrameBufferHandle destination = handles_->downsample_buffers1[i];
      int dest_width = widths_[i];
      int dest_height = heights_[i];

      setBlendMode(BlendMode::Add);

      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers1[i + 1]));
      setPostEffectUniform<Uniforms::kResampleValues>(dest_width * 0.5f / widths_[i + 1],
                                                      dest_height * 0.5f / heights_[i + 1]);
      setPostEffectUniform<Uniforms::kMult>(2.0f, 2.0f, 2.0f, 1.0f);
      setScreenVertexBuffer(region->layer()->bottomLeftOrigin());
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, dest_width, dest_height);

      bgfx::submit(submit_pass, ProgramCache::programHandle(shaders::vs_sample, shaders::fs_mult));
      submit_pass++;
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

    for (int i = 0; i < quads.num_shapes; ++i) {
      quads.vertices[i * kVerticesPerQuad].texture_x = 0.0f;
      quads.vertices[i * kVerticesPerQuad].texture_y = 0.0f;
      quads.vertices[i * kVerticesPerQuad + 1].texture_x = widths_[1];
      quads.vertices[i * kVerticesPerQuad + 1].texture_y = 0.0f;
      quads.vertices[i * kVerticesPerQuad + 2].texture_x = 0.0f;
      quads.vertices[i * kVerticesPerQuad + 2].texture_y = heights_[1];
      quads.vertices[i * kVerticesPerQuad + 3].texture_x = widths_[1];
      quads.vertices[i * kVerticesPerQuad + 3].texture_y = heights_[1];
    }

    setBlendMode(BlendMode::Add);

    float width_scale = 1.0f / widths_[1];
    float height_scale = 1.0f / heights_[1];
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    float mult = bloom_intensity_ * Color::kGradientNormalization;
    setPostEffectUniform<Uniforms::kColorMult>(mult, mult, mult, 1.0f);
    setPostEffectTexture<Uniforms::kGradient>(0, destination.gradientAtlas()->colorTextureHandle());
    setPostEffectTexture<Uniforms::kTexture>(1, bgfx::getTexture(handles_->downsample_buffers1[1]));
    setUniformDimensions(destination.width(), destination.height());
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shaders::vs_tinted_texture, shaders::fs_tinted_texture));
  }

  void ShaderPostEffect::submit(const BatchVector<SampleRegion>& batches, Layer& destination,
                                int submit_pass) {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setPostEffectUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setBlendMode(BlendMode::Composite);
    setPostEffectUniform<Uniforms::kTime>(destination.time());

    Layer* source_layer = batches[0].shapes->front().region->layer();
    float width_scale = 1.0f / source_layer->width();
    float height_scale = 1.0f / source_layer->height();
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
    float value = hdr() ? 1.0f / kHdrColorMultiplier : 1.0f;
    setPostEffectUniform<Uniforms::kColorMult>(value, value, value, 1.0f);
    setOriginFlipUniform(destination.bottomLeftOrigin());

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

    bgfx::ProgramHandle program = ProgramCache::programHandle(vertexShader(), fragmentShader());
    bgfx::submit(submit_pass, program);
  }
}