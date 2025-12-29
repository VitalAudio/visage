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

#include "shape_batcher.h"

#include "canvas.h"
#include "embedded/shaders.h"
#include "font.h"
#include "graphics_caches.h"
#include "layer.h"
#include "path.h"
#include "region.h"
#include "shader.h"
#include "uniforms.h"
#include "visage_utils/space.h"

#include <bgfx/bgfx.h>

namespace visage {
  static constexpr uint64_t blendModeValue(BlendMode blend_mode) {
    switch (blend_mode) {
    case BlendMode::Opaque:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ZERO);
    case BlendMode::Composite:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_INV_SRC_ALPHA);
    case BlendMode::Alpha:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC_SEPARATE(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA,
                                            BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_INV_SRC_ALPHA);
    case BlendMode::Add:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_ONE);
    case BlendMode::Sub:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_ONE) |
             BGFX_STATE_BLEND_EQUATION_SEPARATE(BGFX_STATE_BLEND_EQUATION_REVSUB,
                                                BGFX_STATE_BLEND_EQUATION_ADD);
    case BlendMode::Mult:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_MULTIPLY;
    case BlendMode::MaskAdd:
      return BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_INV_SRC_ALPHA);
    case BlendMode::MaskRemove:
      return BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_ONE) |
             BGFX_STATE_BLEND_EQUATION(BGFX_STATE_BLEND_EQUATION_REVSUB);
    }

    VISAGE_ASSERT(false);
    return 0;
  }

  void setBlendMode(BlendMode blend_mode) {
    bgfx::setState(blendModeValue(blend_mode));
  }

  template<const char* name>
  void setUniform(const void* value) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    bgfx::setUniform(uniform, value);
  }

  template<const char* name>
  void setUniform(float value0, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    float vec[4] = { value0, value1, value2, value3 };
    bgfx::setUniform(uniform, vec);
  }

  template<const char* name>
  void setTexture(int stage, bgfx::TextureHandle handle) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Sampler, 1);
    bgfx::setTexture(stage, uniform, handle);
  }

  inline void setUniformBounds(int x, int y, int width, int height) {
    float scale_x = 2.0f / width;
    float scale_y = -2.0f / height;
    float view_bounds[4] = { scale_x, scale_y, x * scale_x - 1.0f, y * scale_y + 1.0f };
    setUniform<Uniforms::kBounds>(view_bounds);
  }

  inline void setTimeUniform(float time) {
    float time_values[] = { time, time, time, time };
    setUniform<Uniforms::kTime>(time_values);
  }

  void setUniformDimensions(int width, int height) {
    float view_bounds[4] = { 2.0f / width, -2.0f / height, -1.0f, 1.0f };
    setUniform<Uniforms::kBounds>(view_bounds);
  }

  inline void setColorMult(bool hdr) {
    float value = (hdr ? kHdrColorMultiplier : 1.0f) * Color::kGradientNormalization;
    setUniform<Uniforms::kColorMult>(value, value, value, 1.0f);
  }

  void setOriginFlipUniform(bool origin_flip) {
    setUniform<Uniforms::kOriginFlip>(origin_flip ? -1.0 : 1.0, origin_flip ? 1.0 : 0.0);
  }

  bool initTransientQuadBuffers(int num_quads, const bgfx::VertexLayout& layout,
                                bgfx::TransientVertexBuffer* vertex_buffer,
                                bgfx::TransientIndexBuffer* index_buffer) {
    int num_vertices = num_quads * kVerticesPerQuad;
    int num_indices = num_quads * kIndicesPerQuad;
    if (!bgfx::allocTransientBuffers(vertex_buffer, layout, num_vertices, index_buffer, num_indices)) {
      VISAGE_LOG("Not enough transient buffer memory for %d quads", num_quads);
      return false;
    }

    uint16_t* indices = reinterpret_cast<uint16_t*>(index_buffer->data);
    for (int i = 0; i < num_quads; ++i) {
      int vertex_index = i * kVerticesPerQuad;
      int index = i * kIndicesPerQuad;
      for (int v = 0; v < kIndicesPerQuad; ++v)
        indices[index + v] = vertex_index + kQuadTriangles[v];
    }

    return true;
  }

  uint8_t* initQuadVerticesWithLayout(int num_quads, const bgfx::VertexLayout& layout) {
    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::TransientIndexBuffer index_buffer {};
    if (!initTransientQuadBuffers(num_quads, layout, &vertex_buffer, &index_buffer))
      return nullptr;

    bgfx::setVertexBuffer(0, &vertex_buffer);
    bgfx::setIndexBuffer(&index_buffer);
    return vertex_buffer.data;
  }

  void submitShapes(const Layer& layer, const EmbeddedFile& vertex_shader,
                    const EmbeddedFile& fragment_shader, bool radial_gradient, int submit_pass) {
    setTimeUniform(layer.time());
    setUniformDimensions(layer.width(), layer.height());
    setColorMult(layer.hdr());
    setOriginFlipUniform(layer.bottomLeftOrigin());
    GradientAtlas* gradient_atlas = layer.gradientAtlas();
    setUniform<Uniforms::kRadialGradient>(radial_gradient ? 1.0f : 0.0f);
    setTexture<Uniforms::kGradient>(0, gradient_atlas->colorTextureHandle());
    bgfx::submit(submit_pass, ProgramCache::programHandle(vertex_shader, fragment_shader));
  }

  void setImageAtlasUniform(ImageAtlas* atlas) {
    setTexture<Uniforms::kTexture>(1, atlas->textureHandle());
    setUniform<Uniforms::kAtlasScale>(1.0f / atlas->width(), 1.0f / atlas->height());
  }

  void setPathAtlasUniform(PathAtlas* atlas) {
    setTexture<Uniforms::kTexture>(1, bgfx::getTexture(atlas->frameBufferHandle()));
    setUniform<Uniforms::kAtlasScale>(1.0f / atlas->width(), 1.0f / atlas->height());
  }

  void setImageAtlasUniform(const BatchVector<ImageWrapper>& batches) {
    if (!batches.empty() && !batches[0].shapes->empty())
      setImageAtlasUniform(batches[0].shapes->front().image_atlas);
  }

  void setGraphDataUniform(const BatchVector<GraphLineWrapper>& batches) {
    if (!batches.empty() && !batches[0].shapes->empty())
      setImageAtlasUniform(batches[0].shapes->front().data_atlas);
  }

  void setGraphDataUniform(const BatchVector<GraphFillWrapper>& batches) {
    if (!batches.empty() && !batches[0].shapes->empty())
      setImageAtlasUniform(batches[0].shapes->front().data_atlas);
  }

  void setHeatMapDataUniform(const BatchVector<HeatMapWrapper>& batches) {
    if (!batches.empty() && !batches[0].shapes->empty())
      setImageAtlasUniform(batches[0].shapes->front().data_atlas);
  }

  void setPathDataUniform(const BatchVector<PathFillWrapper>& batches) {
    if (!batches.empty() && !batches[0].shapes->empty())
      setPathAtlasUniform(batches[0].shapes->front().path_atlas);
  }

  inline int numTextPieces(const TextBlock& text, int x, int y, const std::vector<IBounds>& invalid_rects) {
    auto count_pieces = [x, y, &text](int sum, IBounds invalid_rect) {
      ClampBounds clamp = text.clamp.clamp(invalid_rect.x() - x, invalid_rect.y() - y,
                                           invalid_rect.width(), invalid_rect.height());
      if (text.totallyClamped(clamp))
        return sum;

      auto overlaps = [&clamp, &text](const FontAtlasQuad& quad) {
        return quad.x + text.x < clamp.right && quad.x + quad.width + text.x > clamp.left &&
               quad.y + text.y < clamp.bottom && quad.y + quad.height + text.y > clamp.top;
      };
      int num_pieces = std::count_if(text.quads.begin(), text.quads.end(), overlaps);
      return sum + num_pieces;
    };
    return std::accumulate(invalid_rects.begin(), invalid_rects.end(), 0, count_pieces);
  }

  void submitText(const BatchVector<TextBlock>& batches, const Layer& layer, int submit_pass) {
    if (batches.empty() || batches[0].shapes->empty())
      return;

    const Font& font = batches[0].shapes->front().font;
    int total_length = 0;
    for (const auto& batch : batches) {
      auto count_pieces = [&batch](int sum, const TextBlock& text_block) {
        return sum + numTextPieces(text_block, batch.x, batch.y, *batch.invalid_rects);
      };
      total_length += std::accumulate(batch.shapes->begin(), batch.shapes->end(), 0, count_pieces);
    }

    if (total_length == 0)
      return;

    TextureVertex* vertices = initQuadVertices<TextureVertex>(total_length);
    if (vertices == nullptr)
      return;

    int vertex_index = 0;
    for (const auto& batch : batches) {
      for (const TextBlock& text_block : *batch.shapes) {
        int length = text_block.quads.size();
        if (length == 0)
          continue;

        float x = text_block.x + batch.x;
        float y = text_block.y + batch.y;
        for (const IBounds& invalid_rect : *batch.invalid_rects) {
          ClampBounds clamp = text_block.clamp.clamp(invalid_rect.x() - batch.x,
                                                     invalid_rect.y() - batch.y,
                                                     invalid_rect.width(), invalid_rect.height());
          if (text_block.totallyClamped(clamp))
            continue;

          auto overlaps = [&clamp, &text_block](const FontAtlasQuad& quad) {
            return quad.x + text_block.x < clamp.right &&
                   quad.x + quad.width + text_block.x > clamp.left &&
                   quad.y + text_block.y < clamp.bottom &&
                   quad.y + quad.height + text_block.y > clamp.top;
          };

          ClampBounds positioned_clamp = clamp.withOffset(batch.x, batch.y);
          float direction_x = 1.0f;
          float direction_y = 0.0f;
          int coordinate_index0 = 0;
          int coordinate_index1 = 1;
          int coordinate_index2 = 2;
          int coordinate_index3 = 3;

          if (text_block.direction == Direction::Down) {
            direction_x = -1.0f;
            direction_y = 0.0f;
            coordinate_index0 = 3;
            coordinate_index1 = 2;
            coordinate_index2 = 1;
            coordinate_index3 = 0;
          }
          else if (text_block.direction == Direction::Left) {
            direction_x = 0.0f;
            direction_y = -1.0f;
            coordinate_index0 = 2;
            coordinate_index1 = 0;
            coordinate_index2 = 3;
            coordinate_index3 = 1;
          }
          else if (text_block.direction == Direction::Right) {
            direction_x = 0.0f;
            direction_y = 1.0f;
            coordinate_index0 = 1;
            coordinate_index1 = 3;
            coordinate_index2 = 0;
            coordinate_index3 = 2;
          }

          PackedBrush::setVertexGradientPositions(text_block.brush, vertices + vertex_index,
                                                  length * kVerticesPerQuad, x, y, batch.x, batch.y,
                                                  x + text_block.width, y + text_block.height);

          for (int i = 0; i < length; ++i) {
            if (!overlaps(text_block.quads[i]))
              continue;

            float left = x + text_block.quads[i].x - 0.5f;
            float right = left + text_block.quads[i].width + 1.0f;
            float top = y + text_block.quads[i].y;
            float bottom = top + text_block.quads[i].height;

            float texture_x = text_block.quads[i].packed_glyph->atlas_left - 0.5f;
            float texture_y = text_block.quads[i].packed_glyph->atlas_top;
            float texture_width = text_block.quads[i].packed_glyph->width + 1.0f;
            float texture_height = text_block.quads[i].packed_glyph->height;

            vertices[vertex_index].x = left;
            vertices[vertex_index].y = top;
            vertices[vertex_index + 1].x = right;
            vertices[vertex_index + 1].y = top;
            vertices[vertex_index + 2].x = left;
            vertices[vertex_index + 2].y = bottom;
            vertices[vertex_index + 3].x = right;
            vertices[vertex_index + 3].y = bottom;

            vertices[vertex_index + coordinate_index0].texture_x = texture_x;
            vertices[vertex_index + coordinate_index0].texture_y = texture_y;
            vertices[vertex_index + coordinate_index1].texture_x = texture_x + texture_width;
            vertices[vertex_index + coordinate_index1].texture_y = texture_y;
            vertices[vertex_index + coordinate_index2].texture_x = texture_x;
            vertices[vertex_index + coordinate_index2].texture_y = texture_y + texture_height;
            vertices[vertex_index + coordinate_index3].texture_x = texture_x + texture_width;
            vertices[vertex_index + coordinate_index3].texture_y = texture_y + texture_height;

            for (int v = 0; v < kVerticesPerQuad; ++v) {
              int index = vertex_index + v;
              vertices[index].clamp_left = positioned_clamp.left;
              vertices[index].clamp_top = positioned_clamp.top;
              vertices[index].clamp_right = positioned_clamp.right;
              vertices[index].clamp_bottom = positioned_clamp.bottom;
              vertices[index].direction_x = direction_x;
              vertices[index].direction_y = direction_y;
            }

            vertex_index += kVerticesPerQuad;
          }
        }
      }
    }

    VISAGE_ASSERT(vertex_index == total_length * kVerticesPerQuad);

    setUniform<Uniforms::kAtlasScale>(1.0f / font.atlasWidth(), 1.0f / font.atlasHeight());
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());
    setTexture<Uniforms::kTexture>(1, font.textureHandle());
    setUniformDimensions(layer.width(), layer.height());
    setColorMult(layer.hdr());
    setUniform<Uniforms::kRadialGradient>(batches[0].shapes->front().radialGradient() ? 1.0f : 0.0f);
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shaders::vs_tinted_texture, shaders::fs_tinted_texture));
  }

  void submitShader(const BatchVector<ShaderWrapper>& batches, const Layer& layer, int submit_pass) {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);
    setBlendMode(BlendMode::Alpha);
    setTimeUniform(layer.time());
    setUniformDimensions(layer.width(), layer.height());
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());
    setColorMult(layer.hdr());
    setOriginFlipUniform(layer.bottomLeftOrigin());
    Shader* shader = batches[0].shapes->front().shader;
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shader->vertexShader(), shader->fragmentShader()));
  }

  void submitSampleRegions(const BatchVector<SampleRegion>& batches, const Layer& layer, int submit_pass) {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setBlendMode(BlendMode::Alpha);
    setTimeUniform(layer.time());
    Layer* source_layer = batches[0].shapes->front().region->layer();
    setUniform<Uniforms::kAtlasScale>(1.0f / source_layer->width(), 1.0f / source_layer->height());

    setTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
    setUniformDimensions(layer.width(), layer.height());
    float value = layer.hdr() ? kHdrColorMultiplier : 1.0f;
    setUniform<Uniforms::kColorMult>(value, value, value, 1.0f);
    setOriginFlipUniform(layer.bottomLeftOrigin());
    bgfx::submit(submit_pass, ProgramCache::programHandle(SampleRegion::vertexShader(),
                                                          SampleRegion::fragmentShader()));
  }
}