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
  inline void setUniform(const void* value) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    bgfx::setUniform(uniform, value);
  }

  template<const char* name>
  inline void setUniform(float value) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Vec4, 1);
    float vec[4] = { value, value, value, value };
    bgfx::setUniform(uniform, vec);
  }

  template<const char* name>
  inline void setTexture(int stage, bgfx::TextureHandle handle) {
    static const bgfx::UniformHandle uniform = bgfx::createUniform(name, bgfx::UniformType::Sampler, 1);
    bgfx::setTexture(stage, uniform, handle);
  }

  inline void setUniformBounds(int x, int y, int width, int height) {
    float scale_x = 2.0f / width;
    float scale_y = -2.0f / height;
    float view_bounds[4] = { scale_x, scale_y, x * scale_x - 1.0f, y * scale_y + 1.0f };
    setUniform<Uniforms::kBounds>(view_bounds);
  }

  inline void setScissor(const BaseShape& shape, int full_width, int full_height) {
    const ClampBounds& clamp = shape.clamp;
    int width = std::min<int>(shape.width, clamp.right - clamp.left);
    int height = std::min<int>(shape.height, clamp.bottom - clamp.top);
    int x = std::max<int>(shape.x, clamp.left);
    int y = std::max<int>(shape.y, clamp.top);

    int scissor_x = std::min(full_width, std::max<int>(0, x));
    int scissor_y = std::min(full_height, std::max<int>(0, y));
    int scissor_right = std::min(full_width, std::max<int>(0, x + width));
    int scissor_bottom = std::min(full_height, std::max<int>(0, y + height));
    if (scissor_x < scissor_right && scissor_y < scissor_bottom)
      bgfx::setScissor(scissor_x, scissor_y, scissor_right - scissor_x, scissor_bottom - scissor_y);
  }

  inline float inverseSqrt(float value) {
    static constexpr float kThreeHalves = 1.5f;

    float x2 = value * 0.5f;
    float y = value;
    int i = *reinterpret_cast<int*>(&y);
    i = 0x5f3759df - (i >> 1);
    y = *reinterpret_cast<float*>(&i);
    y = y * (kThreeHalves - (x2 * y * y));
    y = y * (kThreeHalves - (x2 * y * y));
    return y;
  }

  inline float inverseMagnitude(Point point) {
    return inverseSqrt(point.squareMagnitude());
  }

  inline Point normalize(Point point) {
    return point * inverseMagnitude(point);
  }

  static inline void setTimeUniform(float time) {
    float time_values[] = { time, time, time, time };
    setUniform<Uniforms::kTime>(time_values);
  }

  void setUniformDimensions(int width, int height) {
    float view_bounds[4] = { 2.0f / width, -2.0f / height, -1.0f, 1.0f };
    setUniform<Uniforms::kBounds>(view_bounds);
  }

  inline void setColorMult(bool hdr) {
    float value = (hdr ? kHdrColorMultiplier : 1.0f) * Color::kGradientNormalization;
    float color_mult[] = { value, value, value, 1.0f };
    setUniform<Uniforms::kColorMult>(color_mult);
  }

  inline void setOriginFlipUniform(bool origin_flip) {
    float flip_value = origin_flip ? -1.0 : 1.0;
    float true_value = origin_flip ? 1.0 : 0.0;
    float flip[4] = { flip_value, true_value, 0.0f, 0.0f };
    setUniform<Uniforms::kOriginFlip>(flip);
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
    float radial[] = { radial_gradient ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kRadialGradient>(radial);
    setTexture<Uniforms::kGradient>(0, gradient_atlas->colorTextureHandle());
    bgfx::submit(submit_pass, ProgramCache::programHandle(vertex_shader, fragment_shader));
  }

  void submitPathAntiAliasStrip(const PathFillWrapper& path_fill_wrapper, const Layer& layer,
                                int submit_pass) {
    int num_vertices = path_fill_wrapper.antiAliasStripLength();
    if (num_vertices == 0)
      return;

    if (bgfx::getAvailTransientVertexBuffer(num_vertices, LineVertex::layout()) != num_vertices)
      return;

    auto next_path_index = [&path_fill_wrapper](int path_index) {
      path_index++;
      while (path_index < path_fill_wrapper.path.subPaths().size() &&
             path_fill_wrapper.path.subPaths()[path_index].points.empty())
        path_index++;

      return path_index;
    };

    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::allocTransientVertexBuffer(&vertex_buffer, num_vertices, LineVertex::layout());
    LineVertex* line_data = reinterpret_cast<LineVertex*>(vertex_buffer.data);
    if (line_data == nullptr)
      return;

    int outer_index = 0;
    int inner_index = 0;
    int path_index = next_path_index(-1);

    int point_index = 0;
    int vertex_index = 2;

    float scale = path_fill_wrapper.scale;
    Point inner = path_fill_wrapper.anti_alias.first.subPaths()[0].points[0];
    line_data[0].x = inner.x * scale;
    line_data[0].y = inner.y * scale;
    line_data[1].x = inner.x * scale;
    line_data[1].y = inner.y * scale;
    Point outer;
    for (int i = 0; i < path_fill_wrapper.outer_added_points.size(); ++i) {
      int num_outer = path_fill_wrapper.outer_added_points[i];
      int num_inner = path_fill_wrapper.inner_added_points[i];

      while (point_index >= path_fill_wrapper.path.subPaths()[path_index].points.size() - 1) {
        inner = path_fill_wrapper.anti_alias.first.subPaths()[path_index].points.back();
        outer = path_fill_wrapper.anti_alias.second.subPaths()[path_index].points.back();

        path_index = next_path_index(path_index);
        point_index = 0;
        inner_index = 0;
        outer_index = 0;

        line_data[vertex_index].x = inner.x * scale;
        line_data[vertex_index].y = inner.y * scale;
        line_data[vertex_index + 1].x = outer.x * scale;
        line_data[vertex_index + 1].y = outer.y * scale;
        line_data[vertex_index + 2].x = outer.x * scale;
        line_data[vertex_index + 2].y = outer.y * scale;
        inner = path_fill_wrapper.anti_alias.first.subPaths()[path_index].points[0];
        outer = path_fill_wrapper.anti_alias.second.subPaths()[path_index].points[0];
        line_data[vertex_index + 3].x = inner.x * scale;
        line_data[vertex_index + 3].y = inner.y * scale;
        line_data[vertex_index + 4].x = inner.x * scale;
        line_data[vertex_index + 4].y = inner.y * scale;
        line_data[vertex_index + 5].x = outer.x * scale;
        line_data[vertex_index + 5].y = outer.y * scale;
        vertex_index += 6;
      }

      auto& inner_path = path_fill_wrapper.anti_alias.first.subPaths()[path_index];
      auto& outer_path = path_fill_wrapper.anti_alias.second.subPaths()[path_index];

      inner = inner_path.points[inner_index++ % inner_path.points.size()];
      outer = outer_path.points[outer_index++ % outer_path.points.size()];
      line_data[vertex_index].x = inner.x * scale;
      line_data[vertex_index].y = inner.y * scale;
      line_data[vertex_index + 1].x = outer.x * scale;
      line_data[vertex_index + 1].y = outer.y * scale;
      vertex_index += 2;

      for (int out = 1; out < num_outer; ++out) {
        outer = outer_path.points[outer_index++ % outer_path.points.size()];
        line_data[vertex_index].x = inner.x * scale;
        line_data[vertex_index].y = inner.y * scale;
        line_data[vertex_index + 1].x = outer.x * scale;
        line_data[vertex_index + 1].y = outer.y * scale;
        vertex_index += 2;
      }

      for (int in = 1; in < num_inner; ++in) {
        inner = inner_path.points[inner_index++ % inner_path.points.size()];
        line_data[vertex_index].x = inner.x * scale;
        line_data[vertex_index].y = inner.y * scale;
        line_data[vertex_index + 1].x = outer.x * scale;
        line_data[vertex_index + 1].y = outer.y * scale;
        vertex_index += 2;
      }

      point_index++;
    }

    inner = path_fill_wrapper.anti_alias.first.subPaths()[path_index].points.back();
    outer = path_fill_wrapper.anti_alias.second.subPaths()[path_index].points.back();
    line_data[vertex_index].x = inner.x * scale;
    line_data[vertex_index].y = inner.y * scale;
    line_data[vertex_index + 1].x = outer.x * scale;
    line_data[vertex_index + 1].y = outer.y * scale;
    vertex_index += 2;

    VISAGE_ASSERT(vertex_index == num_vertices);

    for (int i = 0; i < num_vertices; i += 2) {
      line_data[i].fill = 0.5f;
      line_data[i + 1].fill = 0.0f;
      line_data[i].value = 0.0f;
      line_data[i + 1].value = 0.0f;
    }

    bgfx::setState(blendModeValue(BlendMode::Alpha) | BGFX_STATE_PT_TRISTRIP);

    float dimensions[4] = { path_fill_wrapper.width, path_fill_wrapper.height, 1.0f, 1.0f };
    float time[] = { static_cast<float>(layer.time()), 0.0f, 0.0f, 0.0f };
    GradientTexturePosition texture_pos;
    GradientVertexPosition gradient_pos;
    PackedBrush::computeVertexGradientTexturePositions(texture_pos, path_fill_wrapper.brush);
    PackedBrush::computeVertexGradientPositions(gradient_pos, path_fill_wrapper.brush, 0, 0, 0, 0,
                                                path_fill_wrapper.width, path_fill_wrapper.height);
    float line_width[] = { 4.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kDimensions>(dimensions);
    setUniform<Uniforms::kTime>(time);
    setUniform<Uniforms::kGradientTexturePosition>(&texture_pos);
    setUniform<Uniforms::kGradientPosition>(gradient_pos.position1());
    setUniform<Uniforms::kGradientPosition2>(gradient_pos.position2());
    float radial_gradient[] = { path_fill_wrapper.radialGradient() ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kRadialGradient>(radial_gradient);
    setUniform<Uniforms::kLineWidth>(line_width);
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());

    bgfx::setVertexBuffer(0, &vertex_buffer);
    setUniformBounds(path_fill_wrapper.x, path_fill_wrapper.y, layer.width(), layer.height());
    setColorMult(layer.hdr());
    auto program = ProgramCache::programHandle(PathFillWrapper::vertexShader(),
                                               PathFillWrapper::fragmentShader());
    bgfx::submit(submit_pass, program);
  }

  void submitPathFill(const PathFillWrapper& path_fill_wrapper, const Layer& layer, int submit_pass) {
    int num_vertices = path_fill_wrapper.numVertices();
    int num_indices = path_fill_wrapper.triangulation.triangles.size();

    if (num_vertices == 0 || num_indices == 0)
      return;

    if (bgfx::getAvailTransientVertexBuffer(num_vertices, LineVertex::layout()) != num_vertices)
      return;

    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::TransientIndexBuffer index_buffer {};
    bgfx::allocTransientBuffers(&vertex_buffer, LineVertex::layout(), num_vertices, &index_buffer,
                                num_indices);
    LineVertex* line_data = reinterpret_cast<LineVertex*>(vertex_buffer.data);
    uint16_t* indices = reinterpret_cast<uint16_t*>(index_buffer.data);
    if (line_data == nullptr || indices == nullptr)
      return;

    float scale = path_fill_wrapper.scale;
    for (int i = 0; i < num_vertices; ++i) {
      line_data[i].x = path_fill_wrapper.triangulation.points[i].x * scale;
      line_data[i].y = path_fill_wrapper.triangulation.points[i].y * scale;
      line_data[i].fill = 0.5f;
      line_data[i].value = 0.0f;
    }

    for (int i = 0; i < num_indices; ++i) {
      int index = path_fill_wrapper.triangulation.triangles[i];
      indices[i] = static_cast<uint16_t>(index);
    }

    float dimensions[4] = { path_fill_wrapper.width, path_fill_wrapper.height, 1.0f, 1.0f };
    float time[] = { static_cast<float>(layer.time()), 0.0f, 0.0f, 0.0f };

    GradientTexturePosition texture_pos;
    GradientVertexPosition gradient_pos;
    PackedBrush::computeVertexGradientTexturePositions(texture_pos, path_fill_wrapper.brush);
    PackedBrush::computeVertexGradientPositions(gradient_pos, path_fill_wrapper.brush, 0, 0, 0, 0,
                                                path_fill_wrapper.width, path_fill_wrapper.height);
    float line_width[] = { 4.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kDimensions>(dimensions);
    setUniform<Uniforms::kTime>(time);
    setUniform<Uniforms::kGradientTexturePosition>(&texture_pos);
    setUniform<Uniforms::kGradientPosition>(gradient_pos.position1());
    setUniform<Uniforms::kGradientPosition2>(gradient_pos.position2());
    float radial_gradient[] = { path_fill_wrapper.radialGradient() ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kRadialGradient>(radial_gradient);
    setUniform<Uniforms::kLineWidth>(line_width);
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());

    bgfx::setVertexBuffer(0, &vertex_buffer);
    bgfx::setIndexBuffer(&index_buffer);
    setUniformBounds(path_fill_wrapper.x, path_fill_wrapper.y, layer.width(), layer.height());
    setColorMult(layer.hdr());
    auto program = ProgramCache::programHandle(PathFillWrapper::vertexShader(),
                                               PathFillWrapper::fragmentShader());
    bgfx::submit(submit_pass, program);

    submitPathAntiAliasStrip(path_fill_wrapper, layer, submit_pass);
  }

  void setImageAtlasUniform(const BatchVector<ImageWrapper>& batches) {
    if (batches.empty() || batches[0].shapes->empty())
      return;

    const ImageAtlas* image_atlas = batches[0].shapes->front().image_atlas;
    setTexture<Uniforms::kTexture>(1, image_atlas->textureHandle());
    float atlas_scale[] = { 1.0f / image_atlas->width(), 1.0f / image_atlas->height(), 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);
  }

  void setGraphDataUniform(const BatchVector<GraphLineWrapper>& batches) {
    if (batches.empty() || batches[0].shapes->empty())
      return;

    const ImageAtlas* data_atlas = batches[0].shapes->front().data_atlas;
    setTexture<Uniforms::kTexture>(1, data_atlas->textureHandle());
    float atlas_scale[] = { 1.0f / data_atlas->width(), 1.0f / data_atlas->height(), 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);
  }

  void setGraphDataUniform(const BatchVector<GraphFillWrapper>& batches) {
    if (batches.empty() || batches[0].shapes->empty())
      return;

    const ImageAtlas* data_atlas = batches[0].shapes->front().data_atlas;
    setTexture<Uniforms::kTexture>(1, data_atlas->textureHandle());
    float atlas_scale[] = { 1.0f / data_atlas->width(), 1.0f / data_atlas->height(), 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);
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

        int x = text_block.x + batch.x;
        int y = text_block.y + batch.y;
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

            float left = x + text_block.quads[i].x;
            float right = left + text_block.quads[i].width;
            float top = y + text_block.quads[i].y;
            float bottom = top + text_block.quads[i].height;

            float texture_x = text_block.quads[i].packed_glyph->atlas_left;
            float texture_y = text_block.quads[i].packed_glyph->atlas_top;
            float texture_width = text_block.quads[i].packed_glyph->width;
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

    float atlas_scale_uniform[] = { 1.0f / font.atlasWidth(), 1.0f / font.atlasHeight(), 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale_uniform);
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());
    setTexture<Uniforms::kTexture>(1, font.textureHandle());
    setUniformDimensions(layer.width(), layer.height());
    setColorMult(layer.hdr());
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shaders::vs_tinted_texture, shaders::fs_tinted_texture));
  }

  void submitShader(const BatchVector<ShaderWrapper>& batches, const Layer& layer, int submit_pass) {
    bool radial_gradient = false;
    if (!setupQuads(batches, radial_gradient))
      return;

    float radial[] = { radial_gradient ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kRadialGradient>(radial);

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
    bool radial_gradient = false;
    if (!setupQuads(batches, radial_gradient))
      return;

    float radial[] = { radial_gradient ? 1.0f : 0.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kRadialGradient>(radial);

    Layer* source_layer = batches[0].shapes->front().region->layer();
    float width_scale = 1.0f / source_layer->width();
    float height_scale = 1.0f / source_layer->height();

    setBlendMode(BlendMode::Alpha);
    setTimeUniform(layer.time());
    float atlas_scale[] = { width_scale, height_scale, 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);

    setTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
    setUniformDimensions(layer.width(), layer.height());
    float value = layer.hdr() ? kHdrColorMultiplier : 1.0f;
    float color_mult[] = { value, value, value, 1.0f };
    setUniform<Uniforms::kColorMult>(color_mult);
    setOriginFlipUniform(layer.bottomLeftOrigin());
    bgfx::submit(submit_pass, ProgramCache::programHandle(SampleRegion::vertexShader(),
                                                          SampleRegion::fragmentShader()));
  }
}