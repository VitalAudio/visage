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

#include "embedded/shaders.h"
#include "font.h"
#include "graphics_caches.h"
#include "line.h"
#include "renderer.h"
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

  inline float inverseMagnitudeOfPoint(FloatPoint point) {
    return inverseSqrt(point.x * point.x + point.y * point.y);
  }

  inline FloatPoint normalize(FloatPoint point) {
    return point * inverseMagnitudeOfPoint(point);
  }

  static void setLineVertices(const LineWrapper& line_wrapper, bgfx::TransientVertexBuffer vertex_buffer) {
    LineVertex* line_data = reinterpret_cast<LineVertex*>(vertex_buffer.data);
    Line* line = line_wrapper.line;

    for (int i = 0; i < line->num_line_vertices; i += 2) {
      line_data[i].fill = 0.0f;
      line_data[i + 1].fill = 1.0f;
    }

    FloatPoint prev_normalized_delta;
    for (int i = 0; i < line->num_points - 1; ++i) {
      if (line->x[i] != line->x[i + 1] || line->y[i] != line->y[i + 1]) {
        prev_normalized_delta = normalize(FloatPoint(line->x[i + 1] - line->x[i],
                                                     line->y[i + 1] - line->y[i]));
        break;
      }
    }

    FloatPoint prev_delta_normal(-prev_normalized_delta.y, prev_normalized_delta.x);
    float radius = line_wrapper.line_width * 0.5f + 0.5f;
    float prev_magnitude = radius;

    for (int i = 0; i < line->num_points; ++i) {
      FloatPoint point(line->x[i], line->y[i]);
      int next_index = i + 1;
      int clamped_next_index = std::min(next_index, line->num_points - 1);

      FloatPoint next_point(line->x[clamped_next_index], line->y[clamped_next_index]);
      FloatPoint delta = next_point - point;
      if (point == next_point)
        delta = prev_normalized_delta;

      float inverse_magnitude = inverseMagnitudeOfPoint(delta);
      float magnitude = 1.0f / std::max(0.00001f, inverse_magnitude);
      FloatPoint normalized_delta(delta.x * inverse_magnitude, delta.y * inverse_magnitude);
      FloatPoint delta_normal(-normalized_delta.y, normalized_delta.x);

      FloatPoint angle_bisect_delta = normalized_delta - prev_normalized_delta;
      FloatPoint bisect_line;
      bool straight = angle_bisect_delta.x < 0.001f && angle_bisect_delta.x > -0.001f &&
                      angle_bisect_delta.y < 0.001f && angle_bisect_delta.y > -0.001f;
      if (straight)
        bisect_line = delta_normal;
      else
        bisect_line = normalize(angle_bisect_delta);

      float x1, x2, x3, x4, x5, x6;
      float y1, y2, y3, y4, y5, y6;

      float max_inner_radius = std::max(radius, 0.5f * (magnitude + prev_magnitude));
      prev_magnitude = magnitude;

      float bisect_normal_dot_product = bisect_line * delta_normal;
      float inner_mult = 1.0f / std::max(0.1f, std::fabs(bisect_normal_dot_product));
      FloatPoint inner_point = point + bisect_line * std::min(inner_mult * radius, max_inner_radius);
      FloatPoint outer_point = point - bisect_line * radius;

      if (bisect_normal_dot_product < 0.0f) {
        FloatPoint outer_point_start = outer_point;
        FloatPoint outer_point_end = outer_point;
        if (!straight) {
          outer_point_start = point + prev_delta_normal * radius;
          outer_point_end = point + delta_normal * radius;
        }
        x1 = outer_point_start.x;
        y1 = outer_point_start.y;
        x3 = outer_point.x;
        y3 = outer_point.y;
        x5 = outer_point_end.x;
        y5 = outer_point_end.y;
        x2 = x4 = x6 = inner_point.x;
        y2 = y4 = y6 = inner_point.y;
      }
      else {
        FloatPoint outer_point_start = outer_point;
        FloatPoint outer_point_end = outer_point;
        if (!straight) {
          outer_point_start = point - prev_delta_normal * radius;
          outer_point_end = point - delta_normal * radius;
        }
        x2 = outer_point_start.x;
        y2 = outer_point_start.y;
        x4 = outer_point.x;
        y4 = outer_point.y;
        x6 = outer_point_end.x;
        y6 = outer_point_end.y;
        x1 = x3 = x5 = inner_point.x;
        y1 = y3 = y5 = inner_point.y;
      }

      int index = i * Line::kLineVerticesPerPoint;

      float value = line->values[i] * line->line_value_scale;
      line_data[index].x = x1;
      line_data[index].y = y1;
      line_data[index].value = value;

      line_data[index + 1].x = x2;
      line_data[index + 1].y = y2;
      line_data[index + 1].value = value;

      line_data[index + 2].x = x3;
      line_data[index + 2].y = y3;
      line_data[index + 2].value = value;

      line_data[index + 3].x = x4;
      line_data[index + 3].y = y4;
      line_data[index + 3].value = value;

      line_data[index + 4].x = x5;
      line_data[index + 4].y = y5;
      line_data[index + 4].value = value;

      line_data[index + 5].x = x6;
      line_data[index + 5].y = y6;
      line_data[index + 5].value = value;

      prev_delta_normal = delta_normal;
      prev_normalized_delta = normalized_delta;
    }
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
    float value = hdr ? kHdrColorMultiplier : 1.0f;
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
                    const EmbeddedFile& fragment_shader, int submit_pass) {
    setTimeUniform(layer.time());
    setUniformDimensions(layer.width(), layer.height());
    setColorMult(layer.hdr());
    setOriginFlipUniform(layer.bottomLeftOrigin());
    GradientAtlas* gradient_atlas = layer.gradientAtlas();
    setTexture<Uniforms::kGradient>(0, gradient_atlas->colorTextureHandle());
    bgfx::submit(submit_pass, ProgramCache::programHandle(vertex_shader, fragment_shader));
  }

  void submitLine(const LineWrapper& line_wrapper, const Layer& layer, int submit_pass) {
    Line* line = line_wrapper.line;
    if (bgfx::getAvailTransientVertexBuffer(line->num_line_vertices, LineVertex::layout()) !=
        line->num_line_vertices)
      return;

    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::allocTransientVertexBuffer(&vertex_buffer, line->num_line_vertices, LineVertex::layout());
    setLineVertices(line_wrapper, vertex_buffer);

    bgfx::setState(blendModeValue(BlendMode::Alpha) | BGFX_STATE_PT_TRISTRIP);

    float dimensions[4] = { line_wrapper.width, line_wrapper.height, 1.0f, 1.0f };
    float time[] = { static_cast<float>(layer.time()), 0.0f, 0.0f, 0.0f };

    auto pos = PackedBrush::computeVertexGradientPositions(line_wrapper.brush, 0, 0,
                                                           line_wrapper.width, line_wrapper.height);
    float gradient_color_pos[] = { pos.gradient_color_from_x, pos.gradient_color_y,
                                   pos.gradient_color_to_x, pos.gradient_color_y };
    float gradient_pos[] = { pos.gradient_position_from_x, pos.gradient_position_from_y,
                             pos.gradient_position_to_x, pos.gradient_position_to_y };
    float line_width[] = { line_wrapper.line_width * 2.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kDimensions>(dimensions);
    setUniform<Uniforms::kTime>(time);
    setUniform<Uniforms::kGradientColorPosition>(gradient_color_pos);
    setUniform<Uniforms::kGradientPosition>(gradient_pos);
    setUniform<Uniforms::kLineWidth>(line_width);
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());

    bgfx::setVertexBuffer(0, &vertex_buffer);
    setUniformBounds(line_wrapper.x, line_wrapper.y, layer.width(), layer.height());
    setColorMult(layer.hdr());
    setScissor(line_wrapper, layer.width(), layer.height());
    auto program = ProgramCache::programHandle(LineWrapper::vertexShader(), LineWrapper::fragmentShader());
    bgfx::submit(submit_pass, program);
  }

  static void setFillVertices(const LineFillWrapper& line_fill_wrapper,
                              const bgfx::TransientVertexBuffer& vertex_buffer) {
    LineVertex* fill_data = reinterpret_cast<LineVertex*>(vertex_buffer.data);
    Line* line = line_fill_wrapper.line;

    int fill_location = line_fill_wrapper.fill_center;
    for (int i = 0; i < line->num_points; ++i) {
      int index_top = i * Line::kFillVerticesPerPoint;
      int index_bottom = index_top + 1;
      float x = line->x[i];
      float y = line->y[i];
      float value = line->values[i] * line->fill_value_scale;
      fill_data[index_top].x = x;
      fill_data[index_top].y = y;
      fill_data[index_top].value = value;
      fill_data[index_bottom].x = x;
      fill_data[index_bottom].y = fill_location;
      fill_data[index_bottom].value = value;
    }
  }

  void submitLineFill(const LineFillWrapper& line_fill_wrapper, const Layer& layer, int submit_pass) {
    Line* line = line_fill_wrapper.line;
    if (bgfx::getAvailTransientVertexBuffer(line->num_fill_vertices, LineVertex::layout()) !=
        line->num_fill_vertices)
      return;

    float dimension_y_scale = line_fill_wrapper.fill_center / line_fill_wrapper.height;
    float dimensions[4] = { line_fill_wrapper.width, line_fill_wrapper.height * dimension_y_scale,
                            1.0f, 1.0f };
    float time[] = { static_cast<float>(layer.time()), 0.0f, 0.0f, 0.0f };
    auto pos = PackedBrush::computeVertexGradientPositions(line_fill_wrapper.brush, 0, 0,
                                                           line_fill_wrapper.width,
                                                           line_fill_wrapper.height);
    float gradient_color_pos[] = { pos.gradient_color_from_x, pos.gradient_color_y,
                                   pos.gradient_color_to_x, pos.gradient_color_y };
    float gradient_pos[] = { pos.gradient_position_from_x, pos.gradient_position_from_y,
                             pos.gradient_position_to_x, pos.gradient_position_to_y };

    float fill_location = static_cast<int>(line_fill_wrapper.fill_center);
    float center[] = { 0.0f, fill_location, 0.0f, 0.0f };

    bgfx::TransientVertexBuffer fill_vertex_buffer {};
    bgfx::allocTransientVertexBuffer(&fill_vertex_buffer, line->num_fill_vertices, LineVertex::layout());
    setFillVertices(line_fill_wrapper, fill_vertex_buffer);

    bgfx::setState(blendModeValue(BlendMode::Alpha) | BGFX_STATE_PT_TRISTRIP);
    setUniform<Uniforms::kDimensions>(dimensions);
    setUniform<Uniforms::kTime>(time);
    setUniform<Uniforms::kGradientColorPosition>(gradient_color_pos);
    setUniform<Uniforms::kGradientPosition>(gradient_pos);
    setUniform<Uniforms::kCenterPosition>(center);

    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());

    bgfx::setVertexBuffer(0, &fill_vertex_buffer);
    setUniformBounds(line_fill_wrapper.x, line_fill_wrapper.y, layer.width(), layer.height());
    setScissor(line_fill_wrapper, layer.width(), layer.height());
    setColorMult(layer.hdr());
    auto program = ProgramCache::programHandle(LineFillWrapper::vertexShader(),
                                               LineFillWrapper::fragmentShader());
    bgfx::submit(submit_pass, program);
  }

  void submitImages(const BatchVector<ImageWrapper>& batches, const Layer& layer, int submit_pass) {
    if (!setupQuads(batches))
      return;

    const ImageGroup* image_group = batches[0].shapes->front().image_group;
    setBlendMode(BlendMode::Alpha);
    float atlas_scale[] = { 1.0f / image_group->atlasWidth(), 1.0f / image_group->atlasHeight(),
                            0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());
    setTexture<Uniforms::kTexture>(1, image_group->textureHandle());
    setUniformDimensions(layer.width(), layer.height());
    setColorMult(layer.hdr());

    auto program = ProgramCache::programHandle(ImageWrapper::vertexShader(),
                                               ImageWrapper::fragmentShader());
    bgfx::submit(submit_pass, program);
  }

  inline int numTextPieces(const TextBlock& text, int x, int y, const std::vector<Bounds>& invalid_rects) {
    auto count_pieces = [x, y, &text](int sum, Bounds invalid_rect) {
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

    const Font& font = batches[0].shapes->front().text->font();
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
        for (const Bounds& invalid_rect : *batch.invalid_rects) {
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
                                                  length * kVerticesPerQuad, x, y,
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
    if (!setupQuads(batches))
      return;

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
    if (!setupQuads(batches))
      return;

    Layer* source_layer = batches[0].shapes->front().region->layer();
    float width_scale = 1.0f / source_layer->width();
    float height_scale = 1.0f / source_layer->height();

    setBlendMode(BlendMode::Alpha);
    setTimeUniform(layer.time());
    float atlas_scale[] = { width_scale, height_scale, 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);

    setTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
    setUniformDimensions(layer.width(), layer.height());
    setColorMult(layer.hdr());
    setOriginFlipUniform(layer.bottomLeftOrigin());
    bgfx::submit(submit_pass, ProgramCache::programHandle(SampleRegion::vertexShader(),
                                                          SampleRegion::fragmentShader()));
  }
}