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
#include "graphics_libs.h"

namespace visage {
  struct Uniforms {
    static constexpr char kTime[] = "u_time";
    static constexpr char kMult[] = "u_mult";
    static constexpr char kBounds[] = "u_bounds";
    static constexpr char kColorMult[] = "u_color_mult";
    static constexpr char kLimitMult[] = "u_limit_mult";
    static constexpr char kOriginFlip[] = "u_origin_flip";
    static constexpr char kAtlasScale[] = "u_atlas_scale";
    static constexpr char kCenterPosition[] = "u_center_position";
    static constexpr char kScale[] = "u_scale";
    static constexpr char kDimensions[] = "u_dimensions";
    static constexpr char kTopLeftColor[] = "u_top_left_color";
    static constexpr char kTopRightColor[] = "u_top_right_color";
    static constexpr char kBottomLeftColor[] = "u_bottom_left_color";
    static constexpr char kBottomRightColor[] = "u_bottom_right_color";
    static constexpr char kLineWidth[] = "u_line_width";
    static constexpr char kResampleValues[] = "u_resample_values";
    static constexpr char kThreshold[] = "u_threshold";
    static constexpr char kPixelSize[] = "u_pixel_size";
    static constexpr char kTexture[] = "s_texture";
  };

  template<const char* name>
  inline void setUniform(const void* value) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    bgfx::setUniform(uniform, value);
  }

  template<const char* name>
  inline void setTexture(int stage, bgfx::TextureHandle handle) {
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
    screen_vertices_[0].y = -1.0f;
    screen_vertices_[1].x = 1.0f;
    screen_vertices_[1].y = -1.0f;
    screen_vertices_[2].x = -1.0f;
    screen_vertices_[2].y = 1.0f;
    screen_vertices_[3].x = 1.0f;
    screen_vertices_[3].y = 1.0f;

    for (auto& screen_vertex : screen_vertices_) {
      screen_vertex.u = screen_vertex.x * 0.5f + 0.5f;
      screen_vertex.v = screen_vertex.y * -0.5f + 0.5f;
    }
  }

  BlurBloomPostEffect::~BlurBloomPostEffect() = default;

  void BlurBloomPostEffect::checkBuffers(const Canvas& canvas) {
    static constexpr uint64_t kFrameBufferFlags = BGFX_TEXTURE_RT | BGFX_SAMPLER_U_CLAMP |
                                                  BGFX_SAMPLER_V_CLAMP;

    int full_width = canvas.width();
    int full_height = canvas.height();
    bgfx::TextureFormat::Enum format = static_cast<bgfx::TextureFormat::Enum>(canvas.frameBufferFormat());

    if (!bgfx::isValid(handles_->screen_index_buffer)) {
      handles_->screen_index_buffer = bgfx::createIndexBuffer(bgfx::makeRef(visage::kQuadTriangles,
                                                                            sizeof(visage::kQuadTriangles)));
      handles_->screen_vertex_buffer = bgfx::createVertexBuffer(bgfx::makeRef(screen_vertices_,
                                                                              sizeof(screen_vertices_)),
                                                                UvVertex::layout());
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
          handles_->downsample_buffers1[i] = bgfx::createFrameBuffer(width, height, format,
                                                                     kFrameBufferFlags);
          handles_->downsample_buffers2[i] = bgfx::createFrameBuffer(width, height, format,
                                                                     kFrameBufferFlags);
        }
      }
    }
  }

  int BlurBloomPostEffect::preprocess(Canvas& canvas, int submit_pass) {
    checkBuffers(canvas);

    float hdr_range = hdr() ? kHdrColorRange : 1.0f;
    float blur_stages = std::max(std::floor(blur_size_) + 0.99f, 0.0f);
    float bloom_stages = std::max(std::floor(bloom_size_) + 0.99f, 0.0f);
    float stages = bloom_stages + (blur_stages - bloom_stages) * blur_amount_;
    stages = std::max(1.0f, std::min(stages, kMaxDownsamples + 0.99f));
    int downsample_index = stages;

    bgfx::FrameBufferHandle source = canvas.frameBuffer();
    int last_width = full_width_;
    int last_height = full_height_;

    cutoff_ = static_cast<int>(downsample_index * blur_amount_);
    cutoff_index_ = std::min<int>(downsample_index - 1, cutoff_);
    for (int i = 0; i < downsample_index; ++i) {
      int downsample_width = (last_width + 1) / 2;
      int downsample_height = (last_height + 1) / 2;
      widths_[i] = downsample_width;
      heights_[i] = downsample_height;

      float x_downsample_scale = downsample_width * 2.0f / last_width;
      float y_downsample_scale = downsample_height * 2.0f / last_height;
      last_width = downsample_width;
      last_height = downsample_height;

      float downsample_values[] = { x_downsample_scale, y_downsample_scale, 0.0f, 0.0f };

      bgfx::FrameBufferHandle destination = handles_->downsample_buffers1[i];
      setBlendState(BlendState::Opaque);
      setTexture<Uniforms::kTexture>(0, bgfx::getTexture(source));
      bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);

      setUniform<Uniforms::kResampleValues>(downsample_values);
      float hdr_mult = hdr() ? visage::kHdrColorMultiplier : 1.0f;
      float thresholds[4] = { hdr_mult, hdr_mult, hdr_mult, hdr_mult };
      setUniform<Uniforms::kThreshold>(thresholds);
      float limit_mult = cutoff_ - i;

      if ((limit_mult >= 0.0f && limit_mult < 1.0f) || (i == 0 && cutoff_ > 0.0f)) {
        float mult_val = 1.0f / (downsample_index - i);
        if (i == 0)
          mult_val *= hdr_range * bloom_intensity_;
        float mult_values[4] = { mult_val, mult_val, mult_val, mult_val };
        setUniform<Uniforms::kMult>(mult_values);

        if (limit_mult) {
          float limit_mult_val = std::min(limit_mult, 1.0f);
          float limit_mults[4] = { limit_mult_val, limit_mult_val, limit_mult_val, limit_mult_val };
          setUniform<Uniforms::kLimitMult>(limit_mults);
          bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_resample,
                                                                     visage::shaders::fs_split_threshold));
        }
        else
          bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_resample,
                                                                     visage::shaders::fs_mult_threshold));
      }
      else
        bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_resample,
                                                                   visage::shaders::fs_sample));

      submit_pass++;

      if (i > 0) {
        float blur_scale1[4] = { 1.0f / downsample_width, 0.0f, 0.0f, 0.0f };
        float blur_scale2[4] = { 0.0f, 1.0f / downsample_height, 0.0f, 0.0f };
        setBlendState(BlendState::Opaque);
        setTexture<Uniforms::kTexture>(0, bgfx::getTexture(destination));
        bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
        bgfx::setIndexBuffer(handles_->screen_index_buffer);
        bgfx::setViewFrameBuffer(submit_pass, handles_->downsample_buffers2[i]);
        bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
        setUniform<Uniforms::kResampleValues>(downsample_values);
        setUniform<Uniforms::kPixelSize>(blur_scale1);

        bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_full_screen_texture,
                                                                   visage::shaders::fs_blur));
        submit_pass++;

        setBlendState(BlendState::Opaque);
        setTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers2[i]));
        bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
        bgfx::setIndexBuffer(handles_->screen_index_buffer);
        bgfx::setViewFrameBuffer(submit_pass, destination);
        bgfx::setViewRect(submit_pass, 0, 0, downsample_width, downsample_height);
        setUniform<Uniforms::kResampleValues>(downsample_values);
        setUniform<Uniforms::kPixelSize>(blur_scale2);
        bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_full_screen_texture,
                                                                   visage::shaders::fs_blur));
        submit_pass++;
      }

      source = destination;
    }

    float cutoff_transition = cutoff_ - cutoff_index_;
    for (int i = downsample_index - 1; i > 0; --i) {
      bgfx::FrameBufferHandle destination = handles_->downsample_buffers1[i - 1];
      int dest_width = widths_[i - 1];
      int dest_height = heights_[i - 1];
      float mult_amount = 1.0f;
      if (i + 1 == cutoff_index_)
        mult_amount *= (1.0f - cutoff_transition) + cutoff_transition;

      if (i < cutoff_) {
        mult_amount *= hdr_range;
        setBlendState(BlendState::Opaque);
      }
      else
        setBlendState(BlendState::Additive);

      float mult[] = { mult_amount, mult_amount, mult_amount, 1.0f };
      float resample_values[] = { dest_width * 0.5f / widths_[i], dest_height * 0.5f / heights_[i],
                                  0.0f, 0.0f };

      setTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers1[i]));
      setUniform<Uniforms::kResampleValues>(resample_values);
      setUniform<Uniforms::kMult>(mult);
      bgfx::setVertexBuffer(0, handles_->screen_vertex_buffer);
      bgfx::setIndexBuffer(handles_->screen_index_buffer);
      bgfx::setViewFrameBuffer(submit_pass, destination);
      bgfx::setViewRect(submit_pass, 0, 0, dest_width, dest_height);
      if (i < cutoff_ - 1.0f)
        bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_resample,
                                                                   visage::shaders::fs_sample));
      else
        bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_resample,
                                                                   visage::shaders::fs_mult));
      submit_pass++;
    }

    return submit_pass;
  }

  void BlurBloomPostEffect::submit(const CanvasWrapper& source, Canvas& destination, int submit_pass) {
    submitPassthrough(source, destination, submit_pass);
    submitBloom(source, destination, submit_pass);
  }

  void BlurBloomPostEffect::submitPassthrough(const CanvasWrapper& source,
                                              const Canvas& destination, int submit_pass) const {
    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::TransientIndexBuffer index_buffer {};
    if (!initTransientQuadBuffers(1, ShapeVertex::layout(), &vertex_buffer, &index_buffer))
      return;

    auto vertices = reinterpret_cast<ShapeVertex*>(vertex_buffer.data);
    setShapeQuadVertices(vertices, source.x, source.y, source.width, source.height, source.clamp,
                         source.color);
    float flip = destination.bottomLeftOrigin() ? 1.0f : 0.0f;
    vertices[0].coordinate_x = 0.0f;
    vertices[0].coordinate_y = flip;
    vertices[1].coordinate_x = 1.0f;
    vertices[1].coordinate_y = flip;
    vertices[2].coordinate_x = 0.0f;
    vertices[2].coordinate_y = 1.0f - flip;
    vertices[3].coordinate_x = 1.0f;
    vertices[3].coordinate_y = 1.0f - flip;

    float hdr_range = hdr() ? visage::kHdrColorRange : 1.0f;
    float passthrough_mult = std::max(0.0f, 1.0f - cutoff_) * hdr_range;
    if (passthrough_mult) {
      setBlendState(BlendState::Opaque);
      setTexture<Uniforms::kTexture>(0, bgfx::getTexture(source.canvas->frameBuffer()));
      bgfx::setVertexBuffer(0, &vertex_buffer);
      bgfx::setIndexBuffer(&index_buffer);
      float mult_values[4] = { passthrough_mult, passthrough_mult, passthrough_mult, passthrough_mult };
      setUniform<Uniforms::kColorMult>(mult_values);
      setUniformDimensions(destination.width(), destination.height());
      bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_image_sample,
                                                                 visage::shaders::fs_image_sample));
    }
  }

  void BlurBloomPostEffect::submitBloom(const CanvasWrapper& source, const Canvas& destination,
                                        int submit_pass) const {
    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::TransientIndexBuffer index_buffer {};
    if (!initTransientQuadBuffers(1, ShapeVertex::layout(), &vertex_buffer, &index_buffer))
      return;

    auto vertices = reinterpret_cast<ShapeVertex*>(vertex_buffer.data);
    setShapeQuadVertices(vertices, source.x, source.y, source.width, source.height, source.clamp,
                         source.color);

    int dest_width = source.canvas->width();
    int dest_height = source.canvas->height();
    float resample_width = dest_width * 0.5f / widths_[0];
    float resample_height = dest_height * 0.5f / heights_[0];
    vertices[0].coordinate_x = 0.0f;
    vertices[0].coordinate_y = 0.0f;
    vertices[1].coordinate_x = resample_width;
    vertices[1].coordinate_y = 0.0f;
    vertices[2].coordinate_x = 0.0f;
    vertices[2].coordinate_y = resample_height;
    vertices[3].coordinate_x = resample_width;
    vertices[3].coordinate_y = resample_height;

    float hdr_range = hdr() ? visage::kHdrColorRange : 1.0f;
    float mult_amount = 1.0f;
    float cutoff_transition = cutoff_ - cutoff_index_;
    if (cutoff_index_ == 1)
      mult_amount *= (1.0f - cutoff_transition) + cutoff_transition;

    if (cutoff_ > 0) {
      mult_amount *= hdr_range;
      setBlendState(BlendState::Opaque);
    }
    else
      setBlendState(BlendState::Additive);

    if (cutoff_ - 1.0f > 0)
      mult_amount = 1.0f;

    float mult[] = { mult_amount, mult_amount, mult_amount, 1.0f };
    setTexture<Uniforms::kTexture>(0, bgfx::getTexture(handles_->downsample_buffers1[0]));
    setUniform<Uniforms::kColorMult>(mult);
    bgfx::setVertexBuffer(0, &vertex_buffer);
    bgfx::setIndexBuffer(&index_buffer);
    setUniformDimensions(destination.width(), destination.height());
    bgfx::submit(submit_pass, visage::ProgramCache::getProgram(visage::shaders::vs_image_sample,
                                                               visage::shaders::fs_image_sample));
  }

  void ShaderPostEffect::submit(const CanvasWrapper& source, Canvas& destination, int submit_pass) {
    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::TransientIndexBuffer index_buffer {};
    if (!initTransientQuadBuffers(1, ShapeVertex::layout(), &vertex_buffer, &index_buffer))
      return;

    auto vertices = reinterpret_cast<ShapeVertex*>(vertex_buffer.data);
    setShapeQuadVertices(vertices, source.x, source.y, source.width, source.height, source.clamp,
                         source.color);
    float flip = destination.bottomLeftOrigin() ? 1.0f : 0.0f;
    vertices[0].coordinate_x = 0.0f;
    vertices[0].coordinate_y = flip;
    vertices[1].coordinate_x = 1.0f;
    vertices[1].coordinate_y = flip;
    vertices[2].coordinate_x = 0.0f;
    vertices[2].coordinate_y = 1.0f - flip;
    vertices[3].coordinate_x = 1.0f;
    vertices[3].coordinate_y = 1.0f - flip;

    bgfx::TextureHandle texture = bgfx::getTexture(source.canvas->frameBuffer());
    setBlendState(BlendState::Alpha);
    setTexture<Uniforms::kTexture>(0, texture);
    bgfx::setVertexBuffer(0, &vertex_buffer);
    bgfx::setIndexBuffer(&index_buffer);
    setUniformDimensions(destination.width(), destination.height());

    for (const auto& uniform : uniforms_)
      bgfx::setUniform(visage::UniformCache::getUniform(uniform.first.c_str()), uniform.second.data);

    float mult_values[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    setUniform<Uniforms::kColorMult>(mult_values);
    bgfx::ProgramHandle program = ProgramCache::getProgram(vertexShader(), fragmentShader());
    bgfx::submit(submit_pass, program);
  }
}