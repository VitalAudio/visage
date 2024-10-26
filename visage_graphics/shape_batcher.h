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
#include "visage_utils/space.h"

#include <algorithm>
#include <numeric>

#ifndef NDEBUG
#include <random>

inline int randomInt(int min, int max) {
  static std::random_device random_device;
  static std::mt19937 generator(random_device());
  std::uniform_int_distribution distribution(min, max);
  return distribution(generator);
}

template<typename T>
static void debugVertices(T* vertices, int num_vertices) {
  for (int i = 0; i < num_vertices; ++i)
    vertices[i].color = (vertices[i].color & ~0xff) + randomInt(0, 0xff);
}

#endif

namespace visage {
  class Shader;

  template<typename T>
  struct DrawBatch {
    DrawBatch(const std::vector<T>* shapes, std::vector<Bounds>* invalid_rects, int x, int y) :
        shapes(shapes), invalid_rects(invalid_rects), x(x), y(y) { }

    const std::vector<T>* shapes;
    std::vector<Bounds>* invalid_rects;
    int x = 0;
    int y = 0;
  };

  template<typename T>
  using BatchVector = std::vector<DrawBatch<T>>;

  inline int numShapePieces(const BaseShape& shape, int x, int y, const std::vector<Bounds>& invalid_rects) {
    auto check_overlap = [x, y, &shape](Bounds invalid_rect) {
      ClampBounds clamp = shape.clamp.clamp(invalid_rect.x() - x, invalid_rect.y() - y,
                                            invalid_rect.width(), invalid_rect.height());
      return !shape.totallyClamped(clamp);
    };
    return std::count_if(invalid_rects.begin(), invalid_rects.end(), check_overlap);
  }

  template<typename T>
  int numShapes(const BatchVector<T>& batches) {
    int total_size = 0;
    for (const auto& batch : batches) {
      auto count_pieces = [&batch](int sum, const T& shape) {
        return sum + numShapePieces(shape, batch.x, batch.y, *batch.invalid_rects);
      };
      total_size += std::accumulate(batch.shapes->begin(), batch.shapes->end(), 0, count_pieces);
    }
    return total_size;
  }

  void setUniformDimensions(int width, int height);
  void setColorMult(bool hdr);
  void setOriginFlipUniform(bool origin_flip);
  void setBlendState(BlendState draw_state);

  bool initTransientQuadBuffers(int num_quads, const bgfx::VertexLayout& layout,
                                bgfx::TransientVertexBuffer* vertex_buffer,
                                bgfx::TransientIndexBuffer* index_buffer);
  uint8_t* initQuadVerticesWithLayout(int num_quads, const bgfx::VertexLayout& layout);
  template<typename T>
  T* initQuadVertices(int num_quads) {
    return reinterpret_cast<T*>(initQuadVerticesWithLayout(num_quads, T::layout()));
  }

  void submitShapes(const Canvas& canvas, const EmbeddedFile& vertex_shader,
                    const EmbeddedFile& fragment_shader, BlendState state, int submit_pass);

  void submitLine(const LineWrapper& line_wrapper, const Canvas& canvas, int submit_pass);
  void submitLineFill(const LineFillWrapper& line_fill_wrapper, const Canvas& canvas, int submit_pass);
  void submitIcons(const BatchVector<IconWrapper>& batches, Canvas& canvas, int submit_pass);
  void submitImages(const BatchVector<ImageWrapper>& batches, Canvas& canvas, int submit_pass);
  void submitText(const BatchVector<TextBlock>& batches, const Canvas& canvas, int submit_pass);
  void submitShader(const BatchVector<ShaderWrapper>& batches, const Canvas& canvas, int submit_pass);

  template<typename T>
  bool setupQuads(const BatchVector<T>& batches) {
    int num_shapes = numShapes(batches);
    if (num_shapes == 0)
      return false;

    auto vertices = initQuadVertices<typename T::Vertex>(num_shapes);
    if (vertices == nullptr)
      return false;

    int vertex_index = 0;
    for (const auto& batch : batches) {
      for (const T& shape : *batch.shapes) {
        for (const Bounds& invalid_rect : *batch.invalid_rects) {
          ClampBounds clamp = shape.clamp.clamp(invalid_rect.x() - batch.x, invalid_rect.y() - batch.y,
                                                invalid_rect.width(), invalid_rect.height());
          if (shape.totallyClamped(clamp))
            continue;

          clamp = clamp.withOffset(batch.x, batch.y);
          setShapeQuadVertices(vertices + vertex_index, batch.x + shape.x, batch.y + shape.y,
                               shape.width, shape.height, clamp, shape.color);
          shape.setVertexData(vertices + vertex_index);
          vertex_index += kVerticesPerQuad;
        }
      }
    }

    VISAGE_ASSERT(vertex_index == num_shapes * kVerticesPerQuad);
    return true;
  }

  template<typename T>
  static void submitShapes(const BatchVector<T>& batches, Canvas& canvas, int submit_pass) {
    if (!setupQuads(batches))
      return;
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
    std::vector<Bounds>* invalid_rects {};
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

  template<typename T>
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
        batch_list.emplace_back(shapes, batch.invalid_rects, batch.x, batch.y);
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

    int autoBatchIndex(const BaseShape& shape) const {
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

    int manualBatchIndex(const BaseShape& shape) const {
      if (batches_.empty())
        return 0;

      return batches_.size() - 1;
    }

    int batchIndex(const BaseShape& shape) const {
      if (manual_batching_)
        return manualBatchIndex(shape);
      return autoBatchIndex(shape);
    }

    template<typename T>
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

    template<typename T>
    void addShape(T shape) {
      int batch_index = batchIndex(shape);
      bool match = batch_index < batches_.size() && batches_[batch_index]->id() == shape.batch_id;
      ShapeBatch<T>* batch = match ? reinterpret_cast<ShapeBatch<T>*>(batches_[batch_index].get()) :
                                     createNewBatch<T>(shape.batch_id, batch_index);

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