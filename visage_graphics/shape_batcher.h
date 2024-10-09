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

#include "graphics_utils.h"
#include "post_effects.h"
#include "shapes.h"

#include <algorithm>

namespace visage {
  class Shader;

  template<typename T>
  struct DrawBatch {
    DrawBatch(const std::vector<T>* shapes, int x, int y) : shapes(shapes), x(x), y(y) { }

    const std::vector<T>* shapes;
    int x = 0;
    int y = 0;
  };

  template<typename T>
  using BatchVector = std::vector<DrawBatch<T>>;

  template<typename T>
  int numShapes(const BatchVector<T>& batches) {
    int total_size = 0;
    for (const auto& batch : batches)
      total_size += batch.shapes->size();
    return total_size;
  }

  void setUniformDimensions(int width, int height);
  void setColorMult(bool hdr);
  void setOriginFlipUniform(bool origin_flip);
  void setBlendState(BlendState draw_state);

  bool initTransientQuadBuffers(int num_quads, const bgfx::VertexLayout& layout,
                                bgfx::TransientVertexBuffer* vertex_buffer,
                                bgfx::TransientIndexBuffer* index_buffer);
  uint8_t* initQuadVertices(int num_quads, const bgfx::VertexLayout& layout);
  void submitShapes(const Canvas& canvas, const EmbeddedFile& vertex_shader,
                    const EmbeddedFile& fragment_shader, BlendState state, int submit_pass);

  void submitLine(const LineWrapper& line_wrapper, const Canvas& canvas, int submit_pass);
  void submitLineFill(const LineFillWrapper& line_fill_wrapper, const Canvas& canvas, int submit_pass);
  void submitIcons(const BatchVector<IconWrapper>& batches, Canvas& canvas, int submit_pass);
  void submitImages(const BatchVector<ImageWrapper>& batches, Canvas& canvas, int submit_pass);
  void submitText(const BatchVector<TextBlock>& batches, const Canvas& canvas, int submit_pass);
  void submitShader(const BatchVector<ShaderWrapper>& batches, const Canvas& canvas, int submit_pass);

  template<class T>
  static void debugVertices(T* vertices, int num_vertices) {
    int random_offset = rand() & 0xff;
    for (int i = 0; i < num_vertices; ++i)
      vertices[i].color = (vertices[i].color & ~0xff) + random_offset;
  }

  template<class T>
  static void submitShapes(const BatchVector<T>& batches, Canvas& canvas, int submit_pass) {
    auto vertices = reinterpret_cast<typename T::Vertex*>(initQuadVertices(numShapes(batches),
                                                                           T::Vertex::layout()));
    if (vertices == nullptr)
      return;

    int vertex_index = 0;
    for (const auto& batch : batches) {
      for (const T& shape : *batch.shapes) {
        setShapeQuadVertices(vertices + vertex_index, batch.x + shape.x, batch.y + shape.y, shape.width,
                             shape.height, shape.clamp.withOffset(batch.x, batch.y), shape.color);
        shape.setVertexData(vertices + vertex_index);
        vertex_index += kVerticesPerQuad;
      }
    }

    // debugVertices(vertices, vertex_index);

    submitShapes(canvas, T::vertexShader(), T::fragmentShader(), T::kState, submit_pass);
  }

  template<>
  inline void submitShapes<LineWrapper>(const BatchVector<LineWrapper>& batches, Canvas& canvas,
                                        int submit_pass) {
    for (const auto& batch : batches) {
      for (const LineWrapper& line_wrapper : *batch.shapes) {
        LineWrapper line = line_wrapper;
        line.x = batch.x + line_wrapper.x;
        line.y = batch.y + line_wrapper.y;
        submitLine(line, canvas, submit_pass);
      }
    }
  }

  template<>
  inline void submitShapes<LineFillWrapper>(const BatchVector<LineFillWrapper>& batches,
                                            Canvas& canvas, int submit_pass) {
    for (const auto& batch : batches) {
      for (const LineFillWrapper& line_fill_wrapper : *batch.shapes) {
        LineFillWrapper line_fill = line_fill_wrapper;
        line_fill.x = batch.x + line_fill.x;
        line_fill.y = batch.y + line_fill.y;
        submitLineFill(line_fill, canvas, submit_pass);
      }
    }
  }

  template<>
  inline void submitShapes<ImageWrapper>(const BatchVector<ImageWrapper>& batches, Canvas& canvas,
                                         int submit_pass) {
    submitImages(batches, canvas, submit_pass);
  }

  template<>
  inline void submitShapes<IconWrapper>(const BatchVector<IconWrapper>& batches, Canvas& canvas,
                                        int submit_pass) {
    submitIcons(batches, canvas, submit_pass);
  }

  template<>
  inline void submitShapes<ShaderWrapper>(const BatchVector<ShaderWrapper>& batches, Canvas& canvas,
                                          int submit_pass) {
    submitShader(batches, canvas, submit_pass);
  }

  template<>
  inline void submitShapes<TextBlock>(const BatchVector<TextBlock>& batches, Canvas& canvas, int submit_pass) {
    submitText(batches, canvas, submit_pass);
  }

  template<>
  inline void submitShapes<CanvasWrapper>(const BatchVector<CanvasWrapper>& batches, Canvas& canvas,
                                          int submit_pass) {
    for (const auto& batch : batches) {
      for (const CanvasWrapper& canvas_wrapper : *batch.shapes)
        canvas_wrapper.post_effect->submit(canvas_wrapper, canvas, submit_pass);
    }
  }

  struct FrameBufferData;

  class SubmitBatch;

  struct PositionedBatch {
    SubmitBatch* batch = nullptr;
    int x = 0;
    int y = 0;
  };

  class SubmitBatch {
  public:
    virtual ~SubmitBatch() = default;
    virtual void clear() = 0;
    virtual void submit(Canvas& canvas, int submit_pass, const std::vector<PositionedBatch>& others) = 0;

    bool overlapsShape(const BaseShape& shape) const {
      int x = shape.x;
      int y = shape.y;
      int right = shape.x + shape.width;
      int bottom = shape.y + shape.height;
      return std::any_of(areas_.begin(), areas_.end(), [x, y, right, bottom](auto& area) {
        return x < area.right && right > area.x && y < area.bottom && bottom > area.y;
      });
    }

    const void* id() const { return id_; }
    void clearAreas() { areas_.clear(); }
    void addShapeArea(const BaseShape& shape) {
      VISAGE_ASSERT(id_ == nullptr || id_ == shape.batch_id);
      id_ = shape.batch_id;
      areas_.push_back({ shape.x, shape.y, shape.x + shape.width, shape.y + shape.height });
    }

  protected:
    void setId(const void* id) {
      VISAGE_ASSERT(id_ == nullptr || id_ == id);
      id_ = id;
    }

  private:
    struct Area {
      float x, y, right, bottom;
    };

    const void* id_ = nullptr;
    std::vector<Area> areas_;
  };

  template<class T>
  class ShapeBatch : public SubmitBatch {
  public:
    ShapeBatch() = default;
    ~ShapeBatch() override = default;

    void clear() override {
      clearAreas();
      shapes_.clear();
    }

    void submit(Canvas& canvas, int submit_pass, const std::vector<PositionedBatch>& batches) override {
      BatchVector<T> batch_list;
      batch_list.reserve(batches.size());
      for (const PositionedBatch& batch : batches) {
        VISAGE_ASSERT(batch.batch->id() == id());
        const std::vector<T>* shapes = &reinterpret_cast<ShapeBatch<T>*>(batch.batch)->shapes_;
        batch_list.emplace_back(shapes, batch.x, batch.y);
      }
      submitShapes(batch_list, canvas, submit_pass);
    }

    void addShape(T shape) {
      addShapeArea(shape);
      shapes_.push_back(std::move(shape));
    }

  private:
    std::vector<T> shapes_;
  };

  class ShapeBatcher {
  public:
    void clear() {
      for (auto& batch : batches_) {
        batch->clear();
        unused_batches_[batch->id()].push_back(std::move(batch));
      }
      batches_.clear();
    }

    void submit(Canvas& canvas, int submit_pass) {
      for (auto& batch : batches_)
        batch->submit(canvas, submit_pass, {});
    }

    int autoBatchIndex(const BaseShape& shape) {
      int match = batches_.size();
      int insert = batches_.size();
      for (int i = batches_.size() - 1; i >= 0; --i) {
        SubmitBatch* batch = batches_[i].get();
        if (batch->id() == shape.batch_id)
          match = i;
        if (batch->overlapsShape(shape))
          break;
        if (batch->id() > shape.batch_id)
          insert = i;
      }
      if (match < batches_.size())
        return match;
      return insert;
    }

    int manualBatchIndex(const BaseShape& shape) {
      if (batches_.empty())
        return 0;

      return batches_.size() - 1;
    }

    int batchIndex(const BaseShape& shape) {
      if (manual_batching_)
        return manualBatchIndex(shape);
      return autoBatchIndex(shape);
    }

    template<class T>
    ShapeBatch<T>* createNewBatch(const void* id, int insert_index) {
      if (!unused_batches_[id].empty()) {
        auto batch = std::move(unused_batches_[id].back());
        unused_batches_[id].pop_back();
        batches_.insert(batches_.begin() + insert_index, std::move(batch));
      }
      else
        batches_.insert(batches_.begin() + insert_index, std::make_unique<ShapeBatch<T>>());

      return reinterpret_cast<ShapeBatch<T>*>(batches_[insert_index].get());
    }

    template<class T>
    void addShape(T shape) {
      int batch_index = batchIndex(shape);
      ShapeBatch<T>* batch = nullptr;
      if (batch_index < batches_.size() && batches_[batch_index]->id() == shape.batch_id)
        batch = reinterpret_cast<ShapeBatch<T>*>(batches_[batch_index].get());
      else
        batch = createNewBatch<T>(shape.batch_id, batch_index);

      batch->addShape(std::move(shape));
    }

    void setManualBatching(bool manual) { manual_batching_ = manual; }

    int numBatches() const { return batches_.size(); }
    bool isEmpty() const { return batches_.empty(); }
    SubmitBatch* batchAtIndex(int index) const { return batches_[index].get(); }

  private:
    std::vector<std::unique_ptr<SubmitBatch>> batches_;
    std::map<const void*, std::vector<std::unique_ptr<SubmitBatch>>> unused_batches_;
    bool manual_batching_ = false;
  };
}