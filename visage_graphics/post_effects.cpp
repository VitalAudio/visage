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

#include "post_effects.h"

#include "canvas.h"
#include "embedded/shaders.h"
#include "graphics_caches.h"
#include "uniforms.h"

#include <bgfx/bgfx.h>

namespace visage {
  template<const char* name>
  inline void setPostEffectUniform(const void* values) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    bgfx::setUniform(uniform, values);
  }

  template<const char* name>
  inline void setPostEffectUniform(float value0, float value1 = 0.0f, float value2 = 0.0f,
                                   float value3 = 0.0f) {
    float values[4] = { value0, value1, value2, value3 };
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    bgfx::setUniform(uniform, values);
  }

  template<const char* name>
  inline void setPostEffectTexture(int stage, bgfx::TextureHandle handle) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Sampler, 1);
    bgfx::setTexture(stage, uniform, handle);
  }

  struct BlurBloomHandles {
    bgfx::IndexBufferHandle screen_index_buffer = BGFX_INVALID_HANDLE;
    bgfx::VertexBufferHandle screen_vertex_buffer = BGFX_INVALID_HANDLE;
    bgfx::FrameBufferHandle downsample_buffers1[BlurBloomPostEffect::kMaxDownsamples] {};
    bgfx::FrameBufferHandle downsample_buffers2[BlurBloomPostEffect::kMaxDownsamples] {};

    ~BlurBloomHandles() { destroy(); }

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

  BlurBloomPostEffect::BlurBloomPostEffect() : PostEffect(true) {
    handles_ = std::make_unique<BlurBloomHandles>();

    for (int i = 0; i < kMaxDownsamples; ++i) {
      handles_->downsample_buffers1[i] = BGFX_INVALID_HANDLE;
      handles_->downsample_buffers2[i] = BGFX_INVALID_HANDLE;
    }

    screen_vertices_[0].x = -1.0f;
    screen_vertices_[0].y = 1.0f;
    screen_vertices_[1].x = 1.0f;
    screen_vertices_[1].y = 1.0f;
    screen_vertices_[2].x = -1.0f;
    screen_vertices_[2].y = -1.0f;
    screen_vertices_[3].x = 1.0f;
    screen_vertices_[3].y = -1.0f;

    for (auto& screen_vertex : screen_vertices_) {
      screen_vertex.u = screen_vertex.x * 0.5f + 0.5f;
      screen_vertex.v = screen_vertex.y * -0.5f + 0.5f;
    }
  }

  BlurBloomPostEffect::~BlurBloomPostEffect() = default;

  void BlurBloomPostEffect::checkBuffers(const Region* region) {
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
    }

    if (full_width != full_width_ || full_height != full_height_ || format_ != format) {
      full_width_ = full_width;
      full_height_ = full_height;
      format_ = format;
      handles_->destroyFrameBuffers();
    }

    if (!bgfx::isValid(handles_->downsample_buffers1[0])) {
      for (int i = 0; i < kMaxDownsamples; ++i) {
        int scale = 1 << (i + 1);
        int width = (full_width + scale - 1) / scale;
        int height = (full_height + scale - 1) / scale;
        if (height > 0 && width > 0) {
          widths_[i] = width;
          heights_[i] = height;
          handles_->downsample_buffers1[i] = bgfx::createFrameBuffer(width, height, format,
                                                                     kFrameBufferFlags);
          handles_->downsample_buffers2[i] = bgfx::createFrameBuffer(width, height, format,
                                                                     kFrameBufferFlags);
        }
      }
    }
  }

  void BlurBloomPostEffect::setInitialVertices(Region* region) {
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
    Point position = region->layer()->coordinatesForRegion(region);
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
      for (int i = 0; i < kVerticesPerQuad; ++i)
        uv_data[i].v = 1.0f - uv_data[i].v;
    }

    bgfx::setVertexBuffer(0, &first_sample_buffer);
  }

  int BlurBloomPostEffect::preprocess(Region* region, int submit_pass) {
    checkBuffers(region);

    float hdr_range = hdr() ? kHdrColorRange : 1.0f;
    float blur_stages = std::max(std::floor(blur_size_) + 0.99f, 0.0f);
    float bloom_stages = std::max(std::floor(bloom_size_) + 0.99f, 0.0f);
    float stages = bloom_stages + (blur_stages - bloom_stages) * blur_amount_;
    stages = std::max(1.0f, std::min(stages, kMaxDownsamples + 0.99f));
    int downsample_index = stages;
    int last_width = full_width_;
    int last_height = full_height_;

    bgfx::FrameBufferHandle source = region->layer()->frameBuffer();
    cutoff_ = downsample_index * blur_amount_;
    for (int i = 0; i < downsample_index; ++i) {
      int downsample_width = widths_[i];
      int downsample_height = heights_[i];
      float x_downsample_scale = downsample_width * 2.0f / last_width;
      float y_downsample_scale = downsample_height * 2.0f / last_height;
      last_width = downsample_width;
      last_height = downsample_height;

      bgfx::FrameBufferHandle destination = handles_->downsample_buffers1[i];
      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source));
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      if (i == 0) {
        setInitialVertices(region);
        setPostEffectUniform<Uniforms::kResampleValues>(1.0f, 1.0f);
      }
      else {
        bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
        setPostEffectUniform<Uniforms::kResampleValues>(x_downsample_scale, y_downsample_scale);
      }

      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);

      float hdr_mult = hdr() ? visage::kHdrColorMultiplier : 1.0f;
      setPostEffectUniform<Uniforms::kThreshold>(hdr_mult);
      float limit_mult = cutoff_ - i;

      if ((limit_mult >= 0.0f && limit_mult < 1.0f) || (i == 0 && cutoff_ > 0.0f)) {
        float mult_val = 1.0f / (downsample_index - i);
        if (i == 0)
          mult_val *= hdr_range * bloom_intensity_;
        setPostEffectUniform<Uniforms::kMult>(mult_val, mult_val, mult_val, 1.0f);

        if (limit_mult) {
          float limit_mult_val = std::min(limit_mult, 1.0f);
          setPostEffectUniform<Uniforms::kLimitMult>(limit_mult_val, limit_mult_val, limit_mult_val, 1.0f);
          auto handle = visage::ProgramCache::programHandle(visage::shaders::vs_resample,
                                                            visage::shaders::fs_split_threshold);
          bgfx::submit(submit_pass, handle);
        }
        else {
          auto handle = visage::ProgramCache::programHandle(visage::shaders::vs_resample,
                                                            visage::shaders::fs_mult_threshold);
          bgfx::submit(submit_pass, handle);
        }
      }
      else {
        bgfx::submit(submit_pass, visage::ProgramCache::programHandle(visage::shaders::vs_resample,
                                                                      visage::shaders::fs_sample));
      }

      submit_pass++;

      if (i > 0) {
        setBlendMode(BlendMode::Opaque);
        setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(destination));
        bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
        bgfx::setIndexBuffer(handles_->screen_index_buffer);
        bgfx::setViewFrameBuffer(submit_pass, handles_->downsample_buffers2[i]);
        bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
        setPostEffectUniform<Uniforms::kPixelSize>(1.0f / downsample_width);

        bgfx::submit(submit_pass, visage::ProgramCache::programHandle(visage::shaders::vs_full_screen_texture,
                                                                      visage::shaders::fs_blur));
        submit_pass++;

        setBlendMode(BlendMode::Opaque);
        setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers2[i]));
        bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
        bgfx::setIndexBuffer(handles_->screen_index_buffer);
        bgfx::setViewFrameBuffer(submit_pass, destination);
        bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
        setPostEffectUniform<Uniforms::kPixelSize>(0.0f, 1.0f / downsample_height);
        bgfx::submit(submit_pass, visage::ProgramCache::programHandle(visage::shaders::vs_full_screen_texture,
                                                                      visage::shaders::fs_blur));
        submit_pass++;
      }

      source = destination;
    }

    for (int i = downsample_index - 1; i > 0; --i) {
      bgfx::FrameBufferHandle destination = handles_->downsample_buffers1[i - 1];
      int dest_width = widths_[i - 1];
      int dest_height = heights_[i - 1];
      float mult_amount = 1.0f;

      if (i < cutoff_) {
        mult_amount *= hdr_range;
        setBlendMode(BlendMode::Opaque);
      }
      else
        setBlendMode(BlendMode::Add);

      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers1[i]));
      setPostEffectUniform<Uniforms::kResampleValues>(dest_width * 0.5f / widths_[i],
                                                      dest_height * 0.5f / heights_[i]);
      setPostEffectUniform<Uniforms::kMult>(mult_amount, mult_amount, mult_amount, 1.0f);
      bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, dest_width, dest_height);
      if (i < cutoff_ - 1.0f)
        bgfx::submit(submit_pass, visage::ProgramCache::programHandle(visage::shaders::vs_resample,
                                                                      visage::shaders::fs_sample));
      else
        bgfx::submit(submit_pass, visage::ProgramCache::programHandle(visage::shaders::vs_resample,
                                                                      visage::shaders::fs_mult));
      submit_pass++;
    }

    return submit_pass;
  }

  void BlurBloomPostEffect::submit(const SampleRegion& source, Layer& destination, int submit_pass,
                                   int x, int y) {
    submitPassthrough(source, destination, submit_pass, x, y);
    submitBloom(source, destination, submit_pass, x, y);
  }

  void BlurBloomPostEffect::submitPassthrough(const SampleRegion& source, const Layer& destination,
                                              int submit_pass, int x, int y) const {
    auto vertices = initQuadVertices<PostEffectVertex>(1);
    if (vertices == nullptr)
      return;

    setQuadPositions(vertices, source, source.clamp.withOffset(x, y), x, y);
    source.region->layer()->setTexturePositionsForRegion(source.region, vertices);

    float hdr_range = hdr() ? visage::kHdrColorRange : 1.0f;
    float passthrough_mult = std::max(0.0f, 1.0f - cutoff_) * hdr_range;
    if (passthrough_mult) {
      setBlendMode(BlendMode::Opaque);
      setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(source.region->layer()->frameBuffer()));
      setPostEffectUniform<Uniforms::kColorMult>(passthrough_mult);
      setUniformDimensions(destination.width(), destination.height());
      float width_scale = 1.0f / source.region->layer()->width();
      float height_scale = 1.0f / source.region->layer()->height();
      setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
      bgfx::submit(submit_pass, visage::ProgramCache::programHandle(visage::shaders::vs_tinted_texture,
                                                                    visage::shaders::fs_tinted_texture));
    }
  }

  void BlurBloomPostEffect::submitBloom(const SampleRegion& source, const Layer& destination,
                                        int submit_pass, int x, int y) const {
    auto vertices = initQuadVertices<PostEffectVertex>(1);
    if (vertices == nullptr)
      return;

    setQuadPositions(vertices, source, source.clamp.withOffset(x, y), x, y);
    vertices[0].texture_x = 0.0f;
    vertices[0].texture_y = 0.0f;
    vertices[1].texture_x = widths_[0];
    vertices[1].texture_y = 0.0f;
    vertices[2].texture_x = 0.0f;
    vertices[2].texture_y = heights_[0];
    vertices[3].texture_x = widths_[0];
    vertices[3].texture_y = heights_[0];

    if (destination.bottomLeftOrigin()) {
      for (int i = 0; i < kVerticesPerQuad; ++i)
        vertices[i].texture_y = heights_[0] - vertices[i].texture_y;
    }

    float hdr_range = hdr() ? visage::kHdrColorRange : 1.0f;
    float mult_amount = 1.0f;

    if (cutoff_) {
      if (cutoff_ == 1)
        mult_amount *= hdr_range;
      setBlendMode(BlendMode::Opaque);
    }
    else
      setBlendMode(BlendMode::Add);

    float width_scale = 1.0f / widths_[0];
    float height_scale = 1.0f / heights_[0];
    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    setPostEffectUniform<Uniforms::kColorMult>(mult_amount, mult_amount, mult_amount, 1.0f);
    setPostEffectTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers1[0]));
    setUniformDimensions(destination.width(), destination.height());
    bgfx::submit(submit_pass, visage::ProgramCache::programHandle(visage::shaders::vs_tinted_texture,
                                                                  visage::shaders::fs_tinted_texture));
  }

  void ShaderPostEffect::submit(const SampleRegion& source, Layer& destination, int submit_pass,
                                int x, int y) {
    auto vertices = initQuadVertices<PostEffectVertex>(1);
    if (vertices == nullptr)
      return;

    float hdr_range = source.region->layer()->hdr() ? visage::kHdrColorRange : 1.0f;
    setPostEffectUniform<Uniforms::kColorMult>(hdr_range, hdr_range, hdr_range, 1.0f);

    setQuadPositions(vertices, source, source.clamp.withOffset(x, y), x, y);
    source.setVertexData(vertices);

    Layer* source_layer = source.region->layer();
    float width_scale = 1.0f / source_layer->width();
    float height_scale = 1.0f / source_layer->height();

    setPostEffectUniform<Uniforms::kAtlasScale>(width_scale, height_scale);
    setPostEffectUniform<Uniforms::kTextureClamp>(vertices[0].texture_x, vertices[0].texture_y,
                                                  vertices[3].texture_x, vertices[3].texture_y);
    float center_x = (vertices[0].texture_x + vertices[3].texture_x) * 0.5f;
    float center_y = (vertices[0].texture_y + vertices[3].texture_y) * 0.5f;
    setPostEffectUniform<Uniforms::kCenterPosition>(center_x, center_y);
    float width = std::abs(vertices[3].texture_x - vertices[0].texture_x);
    float height = std::abs(vertices[3].texture_y - vertices[0].texture_y);
    setPostEffectUniform<Uniforms::kDimensions>(width, height);

    bgfx::TextureHandle texture = bgfx::getTexture(source_layer->frameBuffer());
    setPostEffectTexture<Uniforms::kTexture>(0, texture);
    setUniformDimensions(destination.width(), destination.height());

    for (const auto& uniform : uniforms_)
      bgfx::setUniform(visage::UniformCache::uniformHandle(uniform.first.c_str()), uniform.second.data);

    bgfx::ProgramHandle program = ProgramCache::programHandle(vertexShader(), fragmentShader());
    bgfx::submit(submit_pass, program);
  }
}