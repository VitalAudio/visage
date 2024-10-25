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

#pragma once

#include "color.h"
#include "icon.h"
#include "text.h"

#include <algorithm>

#define VISAGE_CREATE_BATCH_ID \
  static void* batchId() {     \
    static int batch_id = 0;   \
    return &batch_id;          \
  }

namespace visage {
  class Canvas;
  class Image;
  class Font;
  class PostEffect;
  class Shader;
  struct Line;

  enum class Direction {
    Left,
    Up,
    Right,
    Down,
  };

  enum class Corner {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
  };

  struct ClampBounds {
    float left = 1.0f;
    float top = 1.0f;
    float right = 0.0f;
    float bottom = 0.0f;

    bool totallyClamped() const { return bottom <= top || right <= left; }

    ClampBounds withOffset(int x, int y) const {
      return { left + x, top + y, right + x, bottom + y };
    }

    ClampBounds clamp(float x, float y, float width, float height) const {
      float new_top = std::max(top, y);
      float new_left = std::max(left, x);
      return { new_left, new_top, std::max(new_left, std::min(right, x + width)),
               std::max(new_top, std::min(bottom, y + height)) };
    }
  };

  template<typename T>
  static void setShapeQuadVertices(T* vertices, float x, float y, float width, float height,
                                   const ClampBounds& clamp, const QuadColor& color) {
    float left = x;
    float top = y;
    float right = x + width;
    float bottom = y + height;

    for (int i = 0; i < kVerticesPerQuad; ++i) {
      vertices[i].dimension_x = width;
      vertices[i].dimension_y = height;
      vertices[i].color = color.corners[i];
      vertices[i].hdr = color.hdr[i];
      vertices[i].clamp_left = clamp.left;
      vertices[i].clamp_top = clamp.top;
      vertices[i].clamp_right = clamp.right;
      vertices[i].clamp_bottom = clamp.bottom;
    }

    vertices[0].x = left;
    vertices[0].y = top;
    vertices[0].coordinate_x = -1.0f;
    vertices[0].coordinate_y = -1.0f;
    vertices[1].x = right;
    vertices[1].y = top;
    vertices[1].coordinate_x = 1.0f;
    vertices[1].coordinate_y = -1.0f;
    vertices[2].x = left;
    vertices[2].y = bottom;
    vertices[2].coordinate_x = -1.0f;
    vertices[2].coordinate_y = 1.0f;
    vertices[3].x = right;
    vertices[3].y = bottom;
    vertices[3].coordinate_x = 1.0f;
    vertices[3].coordinate_y = 1.0f;
  }

  struct BaseShape {
    BaseShape(const void* batch_id, const ClampBounds& clamp, const QuadColor& color, float x,
              float y, float width, float height) :
        batch_id(batch_id), clamp(clamp), color(color), x(x), y(y), width(width), height(height) { }

    const void* batch_id = nullptr;
    ClampBounds clamp;
    QuadColor color;
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;

    bool overlapsShape(const BaseShape& other) const {
      return x < other.x + other.width && x + width > other.x && y < other.y + other.height &&
             y + height > other.y;
    }

    bool totallyClamped(const ClampBounds& clamp) const {
      return clamp.totallyClamped() || clamp.left >= x + width || clamp.right <= x ||
             clamp.top >= y + height || clamp.bottom <= y;
    }
  };

  template<typename VertexType = ShapeVertex, BlendState Blend = BlendState::Alpha>
  struct Shape : public BaseShape {
    typedef VertexType Vertex;
    static constexpr BlendState kState = Blend;

    Shape(const void* batch_id, const ClampBounds& clamp, const QuadColor& color, float x, float y,
          float width, float height) : BaseShape(batch_id, clamp, color, x, y, width, height) { }
  };

  struct Clear : Shape<ShapeVertex, BlendState::Clear> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    Clear(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
          float height) : Shape(batchId(), clamp, color, x, y, width, height) { }

    static void setVertexData(Vertex* vertices) { }
  };

  struct Fill : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    Fill(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
         float height) : Shape(batchId(), clamp, color, x, y, width, height) { }

    static void setVertexData(Vertex* vertices) { }
  };

  struct RectangleShadow : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RectangleShadow(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                    float height, float thickness) :
        Shape(batchId(), clamp, color, x, y, width, height), thickness(thickness) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v)
        vertices[v].value_1 = 0.5f * thickness;
    }

    float thickness = 0.0f;
  };

  struct RoundedRectangleShadow : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RoundedRectangleShadow(const ClampBounds& clamp, const QuadColor& color, float x, float y,
                           float width, float height, float rounding, float thickness) :
        Shape(batchId(), clamp, color, x, y, width, height), rounding(rounding), thickness(thickness) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = rounding;
        vertices[v].value_2 = thickness;
      }
    }

    float rounding = 0.0f;
    float thickness = 0.0f;
  };

  struct Rectangle : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    Rectangle(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
              float height) : Shape(batchId(), clamp, color, x, y, width, height) { }

    static void setVertexData(Vertex*) { }
  };

  struct RoundedRectangle : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RoundedRectangle(const ClampBounds& clamp, const QuadColor& color, float x, float y,
                     float width, float height, float rounding, float pixel_width) :
        Shape(batchId(), clamp, color, x, y, width, height), rounding(rounding),
        pixel_width(pixel_width) { }

    void setVertexData(Vertex* vertices) const {
      float fade_scale = 1.0f / pixel_width;
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = rounding;
        vertices[v].value_2 = fade_scale;
      }
    }

    float rounding = 0.0f;
    float pixel_width = 1.0f;
  };

  struct RectangleBorder : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RectangleBorder(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                    float height, float thickness) :
        Shape(batchId(), clamp, color, x, y, width, height), thickness(thickness) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v)
        vertices[v].value_1 = thickness;
    }

    float thickness = 0.0f;
  };

  struct RoundedRectangleBorder : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RoundedRectangleBorder(const ClampBounds& clamp, const QuadColor& color, float x, float y,
                           float width, float height, float rounding, float thickness) :
        Shape(batchId(), clamp, color, x, y, width, height), rounding(rounding), thickness(thickness) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = rounding;
        vertices[v].value_2 = thickness;
      }
    }

    float rounding = 0.0f;
    float thickness = 0.0f;
  };

  struct Circle : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    Circle(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
           float pixel_width) :
        Shape(batchId(), clamp, color, x, y, width, width), pixel_width(pixel_width) { }

    void setVertexData(Vertex* vertices) const {
      float fade_scale = 1.0f / pixel_width;
      for (int v = 0; v < kVerticesPerQuad; ++v)
        vertices[v].value_1 = fade_scale;
    }

    float pixel_width = 1.0f;
  };

  struct Triangle : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    Triangle(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
             float height, Direction direction) :
        Shape(batchId(), clamp, color, x, y, width, height), direction(direction) { }

    void setVertexData(Vertex* vertices) const {
      float value_1 = (direction == Direction::Right || direction == Direction::Down) ? 1.0f : -1.0f;
      float value_2 = (direction == Direction::Right || direction == Direction::Left) ? 0.0f : 1.0f;
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = value_1;
        vertices[v].value_2 = value_2;
      }
    }

    Direction direction = Direction::Right;
  };

  struct RoundedCorner : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RoundedCorner(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                  Corner corner) :
        Shape(batchId(), clamp, color, x, y, width, width), corner(corner) { }

    void setVertexData(Vertex* vertices) const {
      bool shift_left = corner == Corner::TopRight || corner == Corner::BottomRight;
      bool shift_up = corner == Corner::BottomLeft || corner == Corner::BottomRight;

      if (shift_left) {
        vertices[0].coordinate_x = 0.0f;
        vertices[2].coordinate_x = 0.0f;
      }
      else {
        vertices[1].coordinate_x = 0.0f;
        vertices[3].coordinate_x = 0.0f;
      }
      if (shift_up) {
        vertices[0].coordinate_y = 0.0f;
        vertices[1].coordinate_y = 0.0f;
      }
      else {
        vertices[2].coordinate_y = 0.0f;
        vertices[3].coordinate_y = 0.0f;
      }
    }

    Corner corner = Corner::TopLeft;
  };

  struct Ring : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    Ring(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width, float thickness) :
        Shape(batchId(), clamp, color, x, y, width, width), thickness(thickness) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v)
        vertices[v].value_1 = thickness;
    }

    float thickness = 0.0f;
  };

  struct FlatArc : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    FlatArc(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
            float height, float thickness, float center_radians, float radians, float pixel_width) :
        Shape(batchId(), clamp, color, x, y, width, height), thickness(thickness),
        center_radians(center_radians), radians(radians), pixel_width(pixel_width) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = thickness;
        vertices[v].value_2 = center_radians;
        vertices[v].value_3 = radians;
        vertices[v].value_4 = pixel_width;
      }
    }

    float thickness = 0.0f;
    float center_radians = 0.0f;
    float radians = 0.0f;
    float pixel_width = 1.0f;
  };

  struct RoundedArc : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RoundedArc(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
               float height, float thickness, float center_radians, float radians, float pixel_width) :
        Shape(batchId(), clamp, color, x, y, width, height), thickness(thickness),
        center_radians(center_radians), radians(radians), pixel_width(pixel_width) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = thickness;
        vertices[v].value_2 = center_radians;
        vertices[v].value_3 = radians;
        vertices[v].value_4 = pixel_width;
      }
    }

    float thickness = 0.0f;
    float center_radians = 0.0f;
    float radians = 0.0f;
    float pixel_width = 1.0f;
  };

  struct FlatSegment : Shape<ComplexShapeVertex> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    FlatSegment(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                float height, float a_x, float a_y, float b_x, float b_y, float thickness,
                float pixel_width) :
        Shape(batchId(), clamp, color, x, y, width, height), a_x(a_x), a_y(a_y), b_x(b_x), b_y(b_y),
        thickness(thickness), pixel_width(pixel_width) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = a_x;
        vertices[v].value_2 = a_y;
        vertices[v].value_3 = b_x;
        vertices[v].value_4 = b_y;
        vertices[v].value_5 = thickness;
        vertices[v].value_6 = pixel_width;
      }
    }

    float a_x = 0.0f;
    float a_y = 0.0f;
    float b_x = 0.0f;
    float b_y = 0.0f;
    float thickness = 0.0f;
    float pixel_width = 1.0f;
  };

  struct RoundedSegment : Shape<ComplexShapeVertex> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    RoundedSegment(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                   float height, float a_x, float a_y, float b_x, float b_y, float thickness,
                   float pixel_width) :
        Shape(batchId(), clamp, color, x, y, width, height), a_x(a_x), a_y(a_y), b_x(b_x), b_y(b_y),
        thickness(thickness), pixel_width(pixel_width) { }

    void setVertexData(Vertex* vertices) const {
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = a_x;
        vertices[v].value_2 = a_y;
        vertices[v].value_3 = b_x;
        vertices[v].value_4 = b_y;
        vertices[v].value_5 = thickness;
        vertices[v].value_6 = pixel_width;
      }
    }

    float a_x = 0.0f;
    float a_y = 0.0f;
    float b_x = 0.0f;
    float b_y = 0.0f;
    float thickness = 0.0f;
    float pixel_width = 1.0f;
  };

  struct Rotary : Shape<RotaryVertex> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    static constexpr float kDefaultMaxRadians = 2.5f;

    Rotary(const ClampBounds& clamp, const QuadColor& color, const QuadColor& back_color,
           const QuadColor& thumb_color, float x, float y, float width, float value, bool bipolar,
           float hover_amount, float arc_thickness, float max_radians = kDefaultMaxRadians) :
        Shape(batchId(), clamp, color, x, y, width, width), back_color(back_color),
        thumb_color(thumb_color), value(value), bipolar(bipolar), hover_amount(hover_amount),
        arc_thickness(arc_thickness), max_radians(max_radians) { }

    void setVertexData(Vertex* vertices) const {
      float bipolar_adjust = bipolar ? 0.0f : -10.0f;
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = value + bipolar_adjust;
        vertices[v].value_2 = hover_amount;
        vertices[v].value_3 = arc_thickness;
        vertices[v].value_4 = max_radians;

        vertices[v].back_color = back_color.corners[v];
        vertices[v].thumb_color = thumb_color.corners[v];
        vertices[v].back_hdr = back_color.hdr[v];
        vertices[v].thumb_hdr = thumb_color.hdr[v];
      }
    }

    QuadColor back_color;
    QuadColor thumb_color;
    float value = 0.0f;
    bool bipolar = false;
    float hover_amount = 0.0f;
    float arc_thickness = 0.0f;
    float max_radians = 0.0f;
  };

  struct Diamond : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    Diamond(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
            float height, float rounding, float pixel_width) :
        Shape(batchId(), clamp, color, x, y, width, height), rounding(rounding),
        pixel_width(pixel_width) { }

    void setVertexData(Vertex* vertices) const {
      float fade_scale = 1.0f / pixel_width;
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].value_1 = rounding;
        vertices[v].value_2 = fade_scale;
      }
    }

    float rounding = 0.0f;
    float pixel_width = 1.0f;
  };

  struct IconWrapper : Shape<IconVertex> {
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    IconWrapper(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                float height, const Icon& icon, IconGroup const* icon_group) :
        Shape(icon_group, clamp, color, x, y, width, height), icon(icon), icon_group(icon_group) { }

    void setVertexData(IconVertex* vertices) const {
      icon_group->setIconCoordinates(vertices, icon);
    }

    Icon icon;
    IconGroup const* icon_group = nullptr;
  };

  struct ImageWrapper : Shape<ImageVertex> {
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    ImageWrapper(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                 float height, Image* image, ImageGroup const* image_group) :
        Shape(image_group, clamp, color, x, y, width, height), image(image), image_group(image_group) { }

    void setVertexData(ImageVertex* vertices) const {
      image_group->setImageCoordinates(vertices, image);
    }

    Image* image = nullptr;
    ImageGroup const* image_group = nullptr;
  };

  struct LineWrapper : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    LineWrapper(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                float height, Line* line, float line_width) :
        Shape(batchId(), clamp, color, x, y, width, height), line(line), line_width(line_width) { }

    Line* line = nullptr;
    float line_width = 0.0f;
  };

  struct LineFillWrapper : Shape<> {
    VISAGE_CREATE_BATCH_ID
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    LineFillWrapper(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                    float height, Line* line, float fill_center) :
        Shape(batchId(), clamp, color, x, y, width, height), line(line), fill_center(fill_center) { }

    Line* line = nullptr;
    float fill_center = 0.0f;
  };

  template<typename T>
  class VectorPool {
  public:
    static VectorPool<T>& instance() {
      static VectorPool instance;
      return instance;
    }

    std::vector<T> vector(int size) {
      std::vector<T> vector = removeVector(size);
      vector.resize(size);
      return vector;
    }

    void returnVector(std::vector<T>&& vector) {
      if (vector.capacity() == 0)
        return;

      vector.clear();
      auto pos = std::lower_bound(pool_.begin(), pool_.end(), vector,
                                  [](const std::vector<T>& vector, const std::vector<T>& insert) {
                                    return vector.capacity() < insert.capacity();
                                  });
      pool_.insert(pos, std::move(vector));
    }

  private:
    std::vector<T> removeVector(int minimum_capacity) {
      if (!pool_.empty()) {
        auto it = std::lower_bound(pool_.begin(), pool_.end(), minimum_capacity,
                                   [](const std::vector<T>& vector, int capacity) {
                                     return vector.capacity() < capacity;
                                   });
        if (it == pool_.end()) {
          std::vector<T> vector = std::move(pool_.back());
          pool_.pop_back();
          return vector;
        }

        std::vector<T> vector = std::move(*it);
        pool_.erase(it);
        return vector;
      }

      return {};
    }

    std::vector<std::vector<T>> pool_;
  };

  struct TextBlock : Shape<> {
    TextBlock(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
              float height, Text* text, Direction direction) :
        Shape(text->font().packedFont(), clamp, color, x, y, width, height), text(text),
        direction(direction) {
      quads = VectorPool<FontAtlasQuad>::instance().vector(text->text().length() * kVerticesPerQuad);

      const char32_t* c_str = text->text().c_str();
      int length = text->text().length();
      float w = width;
      float h = height;
      if (direction == Direction::Left || direction == Direction::Right)
        std::swap(w, h);
      if (text->multiLine()) {
        text->font().setMultiLineVertexPositions(quads.data(), c_str, length, 0, 0, w, h,
                                                 text->justification());
      }
      else {
        text->font().setVertexPositions(quads.data(), c_str, length, 0, 0, w, h,
                                        text->justification(), text->characterOverride());
      }
      if (direction == Direction::Down) {
        for (int i = 0; i < length; ++i) {
          quads[i].top_position = height - quads[i].top_position;
          quads[i].bottom_position = height - quads[i].bottom_position;
          quads[i].left_position = width - quads[i].left_position;
          quads[i].right_position = width - quads[i].right_position;
        }
      }
      else if (direction == Direction::Left) {
        for (int i = 0; i < length; ++i) {
          std::swap(quads[i].left_position, quads[i].top_position);
          std::swap(quads[i].right_position, quads[i].bottom_position);
          quads[i].top_position = height - quads[i].top_position;
          quads[i].bottom_position = height - quads[i].bottom_position;
        }
      }
      else if (direction == Direction::Right) {
        for (int i = 0; i < length; ++i) {
          std::swap(quads[i].left_position, quads[i].top_position);
          std::swap(quads[i].right_position, quads[i].bottom_position);
          quads[i].left_position = width - quads[i].left_position;
          quads[i].right_position = width - quads[i].right_position;
        }
      }

      float clamp_left = clamp.left - x;
      float clamp_right = clamp.right - x;
      float clamp_top = clamp.top - y;
      float clamp_bottom = clamp.bottom - y;
      auto check = [&](const FontAtlasQuad& quad) {
        return quad.right_position < clamp_left || quad.left_position > clamp_right ||
               quad.bottom_position < clamp_top || quad.top_position > clamp_bottom ||
               quad.left_position == quad.right_position;
      };

      auto it = std::remove_if(quads.begin(), quads.end(), check);
      quads.erase(it, quads.end());
    }

    TextBlock(const TextBlock&) = delete;
    TextBlock& operator=(const TextBlock&) = delete;
    TextBlock(TextBlock&&) = default;
    TextBlock& operator=(TextBlock&&) = default;

    ~TextBlock() { VectorPool<FontAtlasQuad>::instance().returnVector(std::move(quads)); }

    std::vector<FontAtlasQuad> quads;
    Text* text = nullptr;
    Direction direction = Direction::Up;
  };

  struct ShaderWrapper : Shape<> {
    ShaderWrapper(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                  float height, Shader* shader) :
        Shape(shader, clamp, color, x, y, width, height), shader(shader) { }

    static void setVertexData(Vertex* vertices) { }

    Shader* shader = nullptr;
  };

  struct CanvasWrapper : Shape<> {
    static const EmbeddedFile& vertexShader();
    static const EmbeddedFile& fragmentShader();

    CanvasWrapper(const ClampBounds& clamp, const QuadColor& color, float x, float y, float width,
                  float height, Canvas* canvas, PostEffect* post_effect) :
        Shape(canvas, clamp, color, x, y, width, height), canvas(canvas), post_effect(post_effect) { }

    Canvas* canvas = nullptr;
    PostEffect* post_effect = nullptr;
  };
}