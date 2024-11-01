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

#include "shape_batcher.h"

#include "canvas.h"
#include "embedded/shaders.h"
#include "font.h"
#include "graphics_caches.h"
#include "graphics_libs.h"
#include "line.h"
#include "renderer.h"
#include "shader.h"
#include "uniforms.h"
#include "visage_utils/space.h"

namespace visage {
  struct GlyphVertex {
    float x;
    float y;
    float coordinate_x;
    float coordinate_y;
    float direction_x;
    float direction_y;
    float clamp_left;
    float clamp_top;
    float clamp_right;
    float clamp_bottom;
    uint32_t color;
    float hdr;

    static bgfx::VertexLayout& layout() {
      static bgfx::VertexLayout layout;
      static bool initialized = false;

      if (!initialized) {
        initialized = true;
        layout.begin()
            .add(bgfx::Attrib::Position, 2, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord0, 4, bgfx::AttribType::Float)
            .add(bgfx::Attrib::TexCoord1, 4, bgfx::AttribType::Float)
            .add(bgfx::Attrib::Color0, 4, bgfx::AttribType::Uint8, true)
            .add(bgfx::Attrib::Color1, 1, bgfx::AttribType::Float)
            .end();
      }

      return layout;
    }
  };

  static constexpr uint64_t blendStateValue(BlendState draw_state) {
    switch (draw_state) {
    case BlendState::Opaque: return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A;
    case BlendState::Additive:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ADD;
    case BlendState::Clear:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ZERO);
    case BlendState::Alpha:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC_SEPARATE(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA,
                                            BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_INV_SRC_ALPHA);
    }

    VISAGE_ASSERT(false);
    return 0;
  }

  void setBlendState(BlendState draw_state) {
    bgfx::setState(blendStateValue(draw_state));
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
    float color_mult[] = { value, value, value, value };
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
    if (!bgfx::allocTransientBuffers(vertex_buffer, layout, num_vertices, index_buffer, num_indices))
      return false;

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

  void submitShapes(const Canvas& canvas, const EmbeddedFile& vertex_shader,
                    const EmbeddedFile& fragment_shader, BlendState state, int submit_pass) {
    setBlendState(state);

    setTimeUniform(canvas.time());
    setUniformDimensions(canvas.width(), canvas.height());
    setColorMult(canvas.hdr());
    setOriginFlipUniform(canvas.bottomLeftOrigin());
    bgfx::submit(submit_pass, ProgramCache::programHandle(vertex_shader, fragment_shader));
  }

  void submitLine(const LineWrapper& line_wrapper, const Canvas& canvas, int submit_pass) {
    Line* line = line_wrapper.line;
    if (bgfx::getAvailTransientVertexBuffer(line->num_line_vertices, LineVertex::layout()) !=
        line->num_line_vertices)
      return;

    QuadColor line_color = line_wrapper.color;
    Color top_left_line = Color::fromABGR(line_color.corners[0]);
    top_left_line.multRgb(line_color.hdr[0]);
    Color top_right_line = Color::fromABGR(line_color.corners[1]);
    top_right_line.multRgb(line_color.hdr[1]);
    Color bottom_left_line = Color::fromABGR(line_color.corners[2]);
    bottom_left_line.multRgb(line_color.hdr[2]);
    Color bottom_right_line = Color::fromABGR(line_color.corners[3]);
    bottom_right_line.multRgb(line_color.hdr[3]);

    float scale[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    float dimensions[4] = { line_wrapper.width, line_wrapper.height, 1.0f, 1.0f };
    float time[] = { static_cast<float>(canvas.time()), 0.0f, 0.0f, 0.0f };

    bgfx::TransientVertexBuffer vertex_buffer {};
    bgfx::allocTransientVertexBuffer(&vertex_buffer, line->num_line_vertices, LineVertex::layout());
    setLineVertices(line_wrapper, vertex_buffer);

    bgfx::setState(blendStateValue(BlendState::Alpha) | BGFX_STATE_PT_TRISTRIP);
    setUniform<Uniforms::kScale>(scale);
    setUniform<Uniforms::kDimensions>(dimensions);
    setUniform<Uniforms::kTopLeftColor>(&top_left_line);
    setUniform<Uniforms::kTopRightColor>(&top_right_line);
    setUniform<Uniforms::kBottomLeftColor>(&bottom_left_line);
    setUniform<Uniforms::kBottomRightColor>(&bottom_right_line);
    setUniform<Uniforms::kTime>(time);

    float line_width[] = { line_wrapper.line_width * 2.0f, 0.0f, 0.0f, 0.0f };
    setUniform<Uniforms::kLineWidth>(line_width);

    bgfx::setVertexBuffer(0, &vertex_buffer);
    setUniformBounds(line_wrapper.x, line_wrapper.y, canvas.width(), canvas.height());
    setScissor(line_wrapper, canvas.width(), canvas.height());
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

  void submitLineFill(const LineFillWrapper& line_fill_wrapper, const Canvas& canvas, int submit_pass) {
    Line* line = line_fill_wrapper.line;
    if (bgfx::getAvailTransientVertexBuffer(line->num_fill_vertices, LineVertex::layout()) !=
        line->num_fill_vertices)
      return;

    float scale[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    float dimension_y_scale = line_fill_wrapper.fill_center / line_fill_wrapper.height;
    float dimensions[4] = { line_fill_wrapper.width, line_fill_wrapper.height * dimension_y_scale,
                            1.0f, 1.0f };
    float time[] = { static_cast<float>(canvas.time()), 0.0f, 0.0f, 0.0f };

    float fill_location = static_cast<int>(line_fill_wrapper.fill_center);
    float center[] = { 0.0f, fill_location, 0.0f, 0.0f };

    QuadColor fill_color = line_fill_wrapper.color;
    Color top_left_fill = Color::fromABGR(fill_color.corners[0]);
    top_left_fill.multRgb(fill_color.hdr[0]);
    Color top_right_fill = Color::fromABGR(fill_color.corners[1]);
    top_right_fill.multRgb(fill_color.hdr[1]);
    Color bottom_left_fill = Color::fromABGR(fill_color.corners[2]);
    bottom_left_fill.multRgb(fill_color.hdr[2]);
    Color bottom_right_fill = Color::fromABGR(fill_color.corners[3]);
    bottom_right_fill.multRgb(fill_color.hdr[3]);

    bgfx::TransientVertexBuffer fill_vertex_buffer {};
    bgfx::allocTransientVertexBuffer(&fill_vertex_buffer, line->num_fill_vertices, LineVertex::layout());
    setFillVertices(line_fill_wrapper, fill_vertex_buffer);

    bgfx::setState(blendStateValue(BlendState::Alpha) | BGFX_STATE_PT_TRISTRIP);
    setUniform<Uniforms::kScale>(scale);
    setUniform<Uniforms::kDimensions>(dimensions);
    setUniform<Uniforms::kTopLeftColor>(&top_left_fill);
    setUniform<Uniforms::kTopRightColor>(&top_right_fill);
    setUniform<Uniforms::kBottomLeftColor>(&bottom_left_fill);
    setUniform<Uniforms::kBottomRightColor>(&bottom_right_fill);
    setUniform<Uniforms::kBottomRightColor>(&bottom_right_fill);
    setUniform<Uniforms::kTime>(time);
    setUniform<Uniforms::kCenterPosition>(center);

    bgfx::setVertexBuffer(0, &fill_vertex_buffer);
    setUniformBounds(line_fill_wrapper.x, line_fill_wrapper.y, canvas.width(), canvas.height());
    setScissor(line_fill_wrapper, canvas.width(), canvas.height());
    auto program = ProgramCache::programHandle(LineFillWrapper::vertexShader(),
                                               LineFillWrapper::fragmentShader());
    bgfx::submit(submit_pass, program);
  }

  void submitIcons(const BatchVector<IconWrapper>& batches, Canvas& canvas, int submit_pass) {
    if (!setupQuads(batches))
      return;

    setBlendState(BlendState::Alpha);
    float atlas_scale[] = { 1.0f / canvas.iconGroup()->atlasWidth(),
                            1.0f / canvas.iconGroup()->atlasWidth(), 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);
    setTexture<Uniforms::kTexture>(0, canvas.iconGroup()->textureHandle());
    setUniformDimensions(canvas.width(), canvas.height());
    setColorMult(canvas.hdr());

    auto program = ProgramCache::programHandle(IconWrapper::vertexShader(), IconWrapper::fragmentShader());
    bgfx::submit(submit_pass, program);
  }

  void submitImages(const BatchVector<ImageWrapper>& batches, Canvas& canvas, int submit_pass) {
    if (!setupQuads(batches))
      return;

    setBlendState(BlendState::Alpha);
    float atlas_scale[] = { 1.0f / canvas.imageGroup()->atlasWidth(),
                            1.0f / canvas.imageGroup()->atlasWidth(), 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);
    setTexture<Uniforms::kTexture>(0, canvas.imageGroup()->atlasTexture());
    setUniformDimensions(canvas.width(), canvas.height());
    setColorMult(canvas.hdr());
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

  void submitText(const BatchVector<TextBlock>& batches, const Canvas& canvas, int submit_pass) {
    if (batches.empty() || batches[0].shapes->empty())
      return;

    const Font& font = batches[0].shapes->at(0).text->font();
    int total_length = 0;
    for (const auto& batch : batches) {
      auto count_pieces = [&batch](int sum, const TextBlock& text_block) {
        return sum + numTextPieces(text_block, batch.x, batch.y, *batch.invalid_rects);
      };
      total_length += std::accumulate(batch.shapes->begin(), batch.shapes->end(), 0, count_pieces);
    }

    if (total_length == 0)
      return;

    GlyphVertex* vertices = initQuadVertices<GlyphVertex>(total_length);
    int vertex_index = 0;
    for (const auto& batch : batches) {
      for (const TextBlock& text_block : *batch.shapes) {
        int length = text_block.quads.size();
        if (length == 0)
          continue;

        int start_vertex_index = vertex_index;
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
          float direction_x = 0.0f;
          float direction_y = 0.0f;

          if (text_block.direction == Direction::Down) {
            direction_x = -1.0f;
            direction_y = 0.0f;
          }
          else if (text_block.direction == Direction::Left) {
            direction_x = 0.0f;
            direction_y = -1.0f;
          }
          else if (text_block.direction == Direction::Right) {
            direction_x = 0.0f;
            direction_y = 1.0f;
          }

          for (int i = 0; i < length; ++i) {
            if (!overlaps(text_block.quads[i]))
              continue;

            float left = x + text_block.quads[i].x;
            float right = left + text_block.quads[i].width;
            float top = y + text_block.quads[i].y;
            float bottom = top + text_block.quads[i].height;

            vertices[vertex_index].x = left;
            vertices[vertex_index].y = top;
            vertices[vertex_index].color = text_block.color.corners[0];
            vertices[vertex_index].hdr = text_block.color.hdr[0];

            vertices[vertex_index + 1].x = right;
            vertices[vertex_index + 1].y = top;
            vertices[vertex_index + 1].color = text_block.color.corners[1];
            vertices[vertex_index + 1].hdr = text_block.color.hdr[1];

            vertices[vertex_index + 2].x = left;
            vertices[vertex_index + 2].y = bottom;
            vertices[vertex_index + 2].color = text_block.color.corners[2];
            vertices[vertex_index + 2].hdr = text_block.color.hdr[2];

            vertices[vertex_index + 3].x = right;
            vertices[vertex_index + 3].y = bottom;
            vertices[vertex_index + 3].color = text_block.color.corners[3];
            vertices[vertex_index + 3].hdr = text_block.color.hdr[3];

            for (int v = 0; v < kVerticesPerQuad; ++v) {
              int index = vertex_index + v;
              vertices[index].coordinate_x = text_block.quads[i].x_coordinates[v];
              vertices[index].coordinate_y = text_block.quads[i].y_coordinates[v];
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

    setBlendState(BlendState::Alpha);
    float atlas_scale[] = { 1.0f / font.atlasWidth(), 1.0f / font.atlasWidth(), 0.0f, 0.0f };
    setUniform<Uniforms::kAtlasScale>(atlas_scale);
    setTexture<Uniforms::kTexture>(0, font.textureHandle());
    setUniformDimensions(canvas.width(), canvas.height());
    setColorMult(canvas.hdr());
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shaders::vs_tinted_texture, shaders::fs_tinted_texture));
  }

  void submitShader(const BatchVector<ShaderWrapper>& batches, const Canvas& canvas, int submit_pass) {
    if (!setupQuads(batches))
      return;

    setBlendState(BlendState::Alpha);
    setTimeUniform(canvas.time());
    setUniformDimensions(canvas.width(), canvas.height());
    setColorMult(canvas.hdr());
    setOriginFlipUniform(canvas.bottomLeftOrigin());
    Shader* shader = batches[0].shapes->at(0).shader;
    bgfx::submit(submit_pass,
                 ProgramCache::programHandle(shader->vertexShader(), shader->fragmentShader()));
  }
}