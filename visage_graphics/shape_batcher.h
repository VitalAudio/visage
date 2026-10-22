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

#pragma once

#include "graphics_utils.h"
#include "post_effects.h"
#include "shapes.h"
#include "visage_utils/space.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace visage {
  class Shader;

  inline int numShapePieces(const BaseShape& shape, int x, int y, const std::vector<IBounds>& invalid_rects) {
    auto check_overlap = [x, y, &shape](IBounds invalid_rect) {
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
  void setOriginFlipUniform(bool origin_flip);
  void setBlendMode(BlendMode draw_state);
  void submitBlended(BlendMode blend_mode, int submit_pass, bgfx::ProgramHandle program);
  void setAlphaClamp(const Layer& layer);

  static constexpr uint64_t kBufferPoolChunkBytes = 4 << 20;

  void beginBufferPoolFrame();
  void endBufferPoolFrame();
  uint64_t bufferPoolCapacity(const bgfx::VertexLayout& layout);
  uint64_t bufferPoolStagingBytes(const bgfx::VertexLayout& layout);
  int maxQuadsPerChunk(int vertex_size);
  void reserveBufferPoolBytes(uint64_t bytes);
  uint8_t* initQuadVerticesWithLayout(int num_quads, const bgfx::VertexLayout& layout);
  uint8_t* initVerticesWithLayout(int num_vertices, const bgfx::VertexLayout& layout);
  bool initIndexedVerticesWithLayout(int num_vertices, const bgfx::VertexLayout& layout,
                                     int num_indices, uint8_t** vertices, uint32_t** indices);

  template<typename T>
  T* initQuadVertices(int num_quads) {
    return reinterpret_cast<T*>(initQuadVerticesWithLayout(num_quads, T::layout()));
  }

  template<typename T>
  T* initVertices(int num_vertices) {
    return reinterpret_cast<T*>(initVerticesWithLayout(num_vertices, T::layout()));
  }

  template<typename T>
  bool initIndexedVertices(int num_vertices, int num_indices, T** vertices, uint32_t** indices) {
    uint8_t* data = nullptr;
    bool result = initIndexedVerticesWithLayout(num_vertices, T::layout(), num_indices, &data, indices);
    *vertices = reinterpret_cast<T*>(data);
    return result;
  }

  template<typename V>
  class QuadChunker {
  public:
    explicit QuadChunker(int num_quads) : remaining_(num_quads) {
      int max_quads = maxQuadsPerChunk(sizeof(V));
      chunks_ = num_quads / max_quads + (num_quads % max_quads != 0);
    }

    template<typename Submit>
    V* nextQuad(Submit&& submit) {
      if (quad_ == num_quads_) {
        VISAGE_ASSERT(chunks_ > 0);
        if (vertices_)
          submit();

        num_quads_ = (remaining_ + chunks_ - 1) / chunks_;
        remaining_ -= num_quads_;
        chunks_--;
        quad_ = 0;
        reserveBufferPoolBytes(uint64_t(num_quads_) * kVerticesPerQuad * sizeof(V));
        vertices_ = initQuadVertices<V>(num_quads_);
        if (vertices_ == nullptr)
          return nullptr;
      }
      return vertices_ + kVerticesPerQuad * quad_++;
    }

    template<typename Submit>
    void finish(Submit&& submit) {
      VISAGE_ASSERT(quad_ == num_quads_ && remaining_ == 0);
      if (vertices_)
        submit();
    }

  private:
    V* vertices_ = nullptr;
    int remaining_ = 0;
    int chunks_ = 0;
    int num_quads_ = 0;
    int quad_ = 0;
  };

  void submitShapes(const Layer& layer, BlendMode blend_mode, const EmbeddedFile& vertex_shader,
                    const EmbeddedFile& fragment_shader, bool radial_gradient, int submit_pass);

  void setImageAtlasUniform(const BatchVector<ImageWrapper>& batches);
  void setGraphDataUniform(const BatchVector<GraphLineWrapper>& batches);
  void setGraphDataUniform(const BatchVector<GraphFillWrapper>& batches);
  void setHeatMapDataUniform(const BatchVector<HeatMapWrapper>& batches);
  void setPathDataUniform(const BatchVector<PathFillWrapper>& batches);

  void submitText(const BatchVector<TextBlock>& batches, BlendMode blend_mode, const Layer& layer,
                  int submit_pass);
  void submitShader(const BatchVector<ShaderWrapper>& batches, const Layer& layer, int submit_pass);
  void submitSampleRegions(const BatchVector<SampleRegion>& batches, BlendMode blend_mode,
                           const Layer& layer, int submit_pass);

  template<typename V>
  struct QuadVertices {
    V* vertices = nullptr;
    int num_shapes = 0;
    bool radial_gradient = false;
  };

  template<typename T, typename Callback>
  bool forEachShapePiece(const BatchVector<T>& batches, Callback&& callback) {
    for (const auto& batch : batches) {
      for (const T& shape : *batch.shapes) {
        for (const IBounds& invalid_rect : *batch.invalid_rects) {
          ClampBounds clamp = shape.clamp.clamp(invalid_rect.x() - batch.x, invalid_rect.y() - batch.y,
                                                invalid_rect.width(), invalid_rect.height());
          if (!shape.totallyClamped(clamp) &&
              !callback(batch, shape, clamp.withOffset(batch.x, batch.y)))
            return false;
        }
      }
    }
    return true;
  }

  template<typename T>
  QuadVertices<typename T::Vertex> setupQuads(const BatchVector<T>& batches) {
    QuadVertices<typename T::Vertex> results;
    results.num_shapes = numShapes(batches);
    if (results.num_shapes == 0)
      return results;

    results.vertices = initQuadVertices<typename T::Vertex>(results.num_shapes);
    if (results.vertices == nullptr)
      return results;

    int vertex_index = 0;
    forEachShapePiece(batches, [&](const auto& batch, const T& shape, const ClampBounds& clamp) {
      setQuadPositions(results.vertices + vertex_index, shape, clamp, batch.x, batch.y);
      shape.setVertexData(results.vertices + vertex_index);
      results.radial_gradient = shape.radialGradient();
      vertex_index += kVerticesPerQuad;
      return true;
    });

    VISAGE_ASSERT(vertex_index == results.num_shapes * kVerticesPerQuad);
    return results;
  }

  template<typename T>
  static void submitBaseShapes(const BatchVector<T>& batches, BlendMode state, Layer& layer, int submit_pass,
                               void (*set_data_uniform)(const BatchVector<T>&) = nullptr) {
    bool radial_gradient = false;
    auto submit = [&] {
      if (set_data_uniform)
        set_data_uniform(batches);
      submitShapes(layer, state, T::vertexShader(), T::fragmentShader(), radial_gradient, submit_pass);
    };

    QuadChunker<typename T::Vertex> chunker(numShapes(batches));
    auto write = [&](const auto& batch, const T& shape, const ClampBounds& clamp) {
      auto vertices = chunker.nextQuad(submit);
      if (vertices == nullptr)
        return false;

      setQuadPositions(vertices, shape, clamp, batch.x, batch.y);
      shape.setVertexData(vertices);
      radial_gradient = shape.radialGradient();
      return true;
    };

    if (forEachShapePiece(batches, write))
      chunker.finish(submit);
  }

  template<typename T>
  static void submitShapes(const BatchVector<T>& batches, BlendMode state, Layer& layer, int submit_pass) {
    submitBaseShapes(batches, state, layer, submit_pass);
  }

  template<>
  inline void submitShapes<PathFillWrapper>(const BatchVector<PathFillWrapper>& batches,
                                            BlendMode state, Layer& layer, int submit_pass) {
    submitBaseShapes(batches, state, layer, submit_pass, setPathDataUniform);
  }

  template<>
  inline void submitShapes<ImageWrapper>(const BatchVector<ImageWrapper>& batches, BlendMode state,
                                         Layer& layer, int submit_pass) {
    submitBaseShapes(batches, state, layer, submit_pass, setImageAtlasUniform);
  }

  template<>
  inline void submitShapes<GraphLineWrapper>(const BatchVector<GraphLineWrapper>& batches,
                                             BlendMode state, Layer& layer, int submit_pass) {
    submitBaseShapes(batches, state, layer, submit_pass, setGraphDataUniform);
  }

  template<>
  inline void submitShapes<GraphFillWrapper>(const BatchVector<GraphFillWrapper>& batches,
                                             BlendMode state, Layer& layer, int submit_pass) {
    submitBaseShapes(batches, state, layer, submit_pass, setGraphDataUniform);
  }

  template<>
  inline void submitShapes<HeatMapWrapper>(const BatchVector<HeatMapWrapper>& batches,
                                           BlendMode state, Layer& layer, int submit_pass) {
    submitBaseShapes(batches, state, layer, submit_pass, setHeatMapDataUniform);
  }

  template<>
  inline void submitShapes<ShaderWrapper>(const BatchVector<ShaderWrapper>& batches,
                                          BlendMode state, Layer& layer, int submit_pass) {
    submitShader(batches, layer, submit_pass);
  }

  template<>
  inline void submitShapes<TextBlock>(const BatchVector<TextBlock>& batches, BlendMode state,
                                      Layer& layer, int submit_pass) {
    submitText(batches, state, layer, submit_pass);
  }

  template<>
  inline void submitShapes<SampleRegion>(const BatchVector<SampleRegion>& batches, BlendMode state,
                                         Layer& layer, int submit_pass) {
    PostEffect* post_effect = batches[0].shapes->front().post_effect;
    bool backdrop = batches[0].shapes->front().backdrop;

    if (post_effect && backdrop)
      post_effect->submitBackdrop(batches, layer, submit_pass);
    else if (post_effect)
      post_effect->submit(batches, layer, submit_pass);
    else
      submitSampleRegions(batches, state, layer, submit_pass);
  }

  class SubmitBatch;

  struct PositionedBatch {
    SubmitBatch* batch = nullptr;
    std::vector<IBounds>* invalid_rects {};
    int x = 0;
    int y = 0;
  };

  class SubmitBatch {
  public:
    explicit SubmitBatch(BlendMode blend_mode) : blend_mode_(blend_mode) { }
    virtual ~SubmitBatch() = default;
    virtual const void* type() const = 0;
    virtual void clear() = 0;
    virtual void submit(Layer& layer, int submit_pass, const std::vector<PositionedBatch>& others) = 0;

    bool overlapsShape(const BaseShape& shape) const {
      int x = shape.x;
      int y = shape.y;
      int right = shape.x + shape.width;
      int bottom = shape.y + shape.height;
      Area area = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(right),
                    static_cast<float>(bottom) };
      for (int chunk = 0; chunk < chunk_bounds_.size(); ++chunk) {
        if (!chunk_bounds_[chunk].overlaps(area))
          continue;

        auto begin = areas_.begin() + chunk * kChunkSize;
        auto end = areas_.begin() + std::min<size_t>(areas_.size(), (chunk + 1) * kChunkSize);
        if (std::any_of(begin, end, [&area](const Area& other) { return other.overlaps(area); }))
          return true;
      }
      return false;
    }

    const void* id() const { return id_; }
    bool match(const void* shape_type, const void* id, BlendMode blend_mode, bool radial_gradient) const {
      return type() == shape_type && id_ == id && blend_mode_ == blend_mode &&
             radial_gradient_ == radial_gradient;
    }
    bool sortsAfter(const void* shape_type, const void* id) const {
      return id_ > id || (id_ == id && type() > shape_type);
    }
    bool match(const SubmitBatch* other) const { return compare(other) == 0; }

    void setBlendMode(BlendMode blend_mode) { blend_mode_ = blend_mode; }
    BlendMode blendMode() const { return blend_mode_; }
    bool radialGradient() const { return radial_gradient_; }

    int compare(const SubmitBatch* other) const {
      if (other == nullptr)
        return 1;

      if (id_ < other->id_)
        return -1;
      if (id_ > other->id_)
        return 1;
      if (type() < other->type())
        return -1;
      if (type() > other->type())
        return 1;
      if (blend_mode_ < other->blend_mode_)
        return -1;
      if (blend_mode_ > other->blend_mode_)
        return 1;
      if (radial_gradient_ < other->radial_gradient_)
        return -1;
      if (radial_gradient_ > other->radial_gradient_)
        return 1;
      return 0;
    }

    void clearAreas() {
      areas_.clear();
      chunk_bounds_.clear();
    }

    void addShapeArea(const BaseShape& shape) {
      VISAGE_ASSERT(id_ == nullptr || id_ == shape.batch_id);
      id_ = shape.batch_id;
      radial_gradient_ = shape.radialGradient();
      Area area = { shape.x, shape.y, shape.x + shape.width, shape.y + shape.height };
      if (areas_.size() % kChunkSize == 0)
        chunk_bounds_.push_back(area);
      else
        chunk_bounds_.back().expand(area);
      areas_.push_back(area);
    }

  private:
    static constexpr int kChunkSize = 64;

    struct Area {
      float x, y, right, bottom;

      bool overlaps(const Area& other) const {
        return x < other.right && right > other.x && y < other.bottom && bottom > other.y;
      }

      void expand(const Area& other) {
        x = std::fmin(x, other.x);
        y = std::fmin(y, other.y);
        right = std::fmax(right, other.right);
        bottom = std::fmax(bottom, other.bottom);
      }
    };

    const void* id_ = nullptr;
    std::vector<Area> areas_;
    std::vector<Area> chunk_bounds_;
    BlendMode blend_mode_;
    bool radial_gradient_ = false;
  };

  template<typename T>
  class ShapeBatch : public SubmitBatch {
  public:
    static const void* typeId() {
      static int type = 0;
      return &type;
    }

    explicit ShapeBatch(BlendMode blend_mode) : SubmitBatch(blend_mode) { }
    ~ShapeBatch() override = default;

    const void* type() const override { return typeId(); }

    void clear() override {
      clearAreas();
      shapes_.clear();
    }

    void submit(Layer& layer, int submit_pass, const std::vector<PositionedBatch>& batches) override {
      BatchVector<T> batch_list;
      batch_list.reserve(batches.size());
      for (const PositionedBatch& batch : batches) {
        VISAGE_ASSERT(batch.batch->type() == type() && batch.batch->id() == id());
        const std::vector<T>* shapes = &static_cast<ShapeBatch<T>*>(batch.batch)->shapes_;
        batch_list.emplace_back(shapes, batch.invalid_rects, batch.x, batch.y);
      }
      submitShapes(batch_list, blendMode(), layer, submit_pass);
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
      unused_batches_.clear();
      for (auto& batch : batches_) {
        batch->clear();
        unused_batches_[{ batch->id(), batch->type() }].push_back(std::move(batch));
      }
      batches_.clear();
    }

    void submit(Layer& layer, int submit_pass) {
      for (auto& batch : batches_)
        batch->submit(layer, submit_pass, {});
    }

    struct BatchPosition {
      int index = 0;
      bool match = false;
    };

    BatchPosition autoBatchPosition(const void* shape_type, const BaseShape& shape, BlendMode blend) const {
      auto matches = [&](const std::unique_ptr<SubmitBatch>& batch) {
        return batch->match(shape_type, shape.batch_id, blend, shape.radialGradient());
      };
      int first_match = std::find_if(batches_.begin(), batches_.end(), matches) - batches_.begin();
      int match = batches_.size();
      int insert = batches_.size();
      for (int i = batches_.size() - 1; i >= 0; --i) {
        SubmitBatch* batch = batches_[i].get();
        if (matches(batches_[i])) {
          match = i;
          if (i == first_match)
            break;
        }
        if (batch->overlapsShape(shape))
          break;
        if (batch->sortsAfter(shape_type, shape.batch_id))
          insert = i;
      }
      if (match < batches_.size())
        return { match, true };
      return { insert, false };
    }

    BatchPosition manualBatchPosition(const void* shape_type, const BaseShape& shape, BlendMode blend) const {
      int last = batches_.size() - 1;
      if (last >= 0 && batches_[last]->match(shape_type, shape.batch_id, blend, shape.radialGradient()))
        return { last, true };
      return { last + 1, false };
    }

    BatchPosition batchPosition(const void* shape_type, const BaseShape& shape, BlendMode blend) const {
      if (manual_batching_)
        return manualBatchPosition(shape_type, shape, blend);
      return autoBatchPosition(shape_type, shape, blend);
    }

    template<typename T>
    ShapeBatch<T>* createNewBatch(const void* id, BlendMode blend, int insert_index) {
      auto& unused = unused_batches_[{ id, ShapeBatch<T>::typeId() }];
      if (!unused.empty()) {
        auto batch = std::move(unused.back());
        unused.pop_back();
        batch->setBlendMode(blend);
        batches_.insert(batches_.begin() + insert_index, std::move(batch));
      }
      else
        batches_.insert(batches_.begin() + insert_index, std::make_unique<ShapeBatch<T>>(blend));

      return static_cast<ShapeBatch<T>*>(batches_[insert_index].get());
    }

    template<typename T>
    void addShape(T shape, BlendMode blend = BlendMode::Alpha) {
      const void* shape_type = ShapeBatch<T>::typeId();
      BatchPosition position = batchPosition(shape_type, shape, blend);
      ShapeBatch<T>* batch = position.match ?
                                 static_cast<ShapeBatch<T>*>(batches_[position.index].get()) :
                                 createNewBatch<T>(shape.batch_id, blend, position.index);

      batch->addShape(std::move(shape));
    }

    void setManualBatching(bool manual) { manual_batching_ = manual; }

    int numBatches() const { return batches_.size(); }
    bool isEmpty() const { return batches_.empty(); }
    SubmitBatch* batchAtIndex(int index) const { return batches_[index].get(); }

  private:
    std::vector<std::unique_ptr<SubmitBatch>> batches_;
    std::map<std::pair<const void*, const void*>, std::vector<std::unique_ptr<SubmitBatch>>> unused_batches_;
    bool manual_batching_ = false;
  };
}