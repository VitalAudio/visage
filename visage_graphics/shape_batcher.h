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
static void debugVertices(T* vertices, int num_random, int spacing) {
  for (int r = 0; r < num_random; ++r) {
    int rand = randomInt(0, 0xff);
    for (int s = 0; s < spacing; ++s)
      vertices[r * spacing + s].color = (vertices[r * spacing + s].color & ~0xff) | rand;
  }
}

#endif

namespace visage {
  class Shader;

  template<typename T>
  struct DrawBatch {
    DrawBatch(const std::vector<T>* shapes, std::vector<IBounds>* invalid_rects, int x, int y) :
        shapes(shapes), invalid_rects(invalid_rects), x(x), y(y) { }

    const std::vector<T>* shapes;
    std::vector<IBounds>* invalid_rects;
    int x = 0;
    int y = 0;
  };

  template<typename T>
  using BatchVector = std::vector<DrawBatch<T>>;

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

  bool initTransientQuadBuffers(int num_quads, const bgfx::VertexLayout& layout,
                                bgfx::TransientVertexBuffer* vertex_buffer,
                                bgfx::TransientIndexBuffer* index_buffer);
  uint8_t* initQuadVerticesWithLayout(int num_quads, const bgfx::VertexLayout& layout);
  template<typename T>
  T* initQuadVertices(int num_quads) {
    return reinterpret_cast<T*>(initQuadVerticesWithLayout(num_quads, T::layout()));
  }

  void submitShapes(const Layer& layer, const EmbeddedFile& vertex_shader,
                    const EmbeddedFile& fragment_shader, bool radial_gradient, int submit_pass);

  void submitPathFill(const PathFillWrapper& path_fill_wrapper, const Layer& layer, int submit_pass);
  void submitLine(const LineWrapper& line_wrapper, const Layer& layer, int submit_pass);
  void submitLineFill(const LineFillWrapper& line_fill_wrapper, const Layer& layer, int submit_pass);
  void submitImages(const BatchVector<ImageWrapper>& batches, const Layer& layer, int submit_pass);
  void submitText(const BatchVector<TextBlock>& batches, const Layer& layer, int submit_pass);
  void submitShader(const BatchVector<ShaderWrapper>& batches, const Layer& layer, int submit_pass);
  void submitSampleRegions(const BatchVector<SampleRegion>& batches, const Layer& layer, int submit_pass);

  template<typename T>
  bool setupQuads(const BatchVector<T>& batches, bool& radial_gradient) {
    int num_shapes = numShapes(batches);
    if (num_shapes == 0)
      return false;

    auto vertices = initQuadVertices<typename T::Vertex>(num_shapes);
    if (vertices == nullptr)
      return false;
    int vertex_index = 0;

    for (const auto& batch : batches) {
      for (const T& shape : *batch.shapes) {
        for (const IBounds& invalid_rect : *batch.invalid_rects) {
          ClampBounds clamp = shape.clamp.clamp(invalid_rect.x() - batch.x, invalid_rect.y() - batch.y,
                                                invalid_rect.width(), invalid_rect.height());
          if (shape.totallyClamped(clamp))
            continue;

          clamp = clamp.withOffset(batch.x, batch.y);
          setQuadPositions(vertices + vertex_index, shape, clamp, batch.x, batch.y);
          shape.setVertexData(vertices + vertex_index);
          radial_gradient = shape.radialGradient();
          vertex_index += kVerticesPerQuad;
        }
      }
    }

    // debugVertices(vertices, num_shapes, kVerticesPerQuad);
    VISAGE_ASSERT(vertex_index == num_shapes * kVerticesPerQuad);
    return true;
  }

  template<typename T>
  static void submitShapes(const BatchVector<T>& batches, BlendMode state, Layer& layer, int submit_pass) {
    bool radial_gradient = false;
    if (!setupQuads(batches, radial_gradient))
      return;

    setBlendMode(state);
    submitShapes(layer, T::vertexShader(), T::fragmentShader(), radial_gradient, submit_pass);
  }

  template<>
  inline void submitShapes<PathFillWrapper>(const BatchVector<PathFillWrapper>& batches,
                                            BlendMode state, Layer& layer, int submit_pass) {
    for (const auto& batch : batches) {
      for (const PathFillWrapper& path_wrapper : *batch.shapes) {
        PathFillWrapper path = path_wrapper;
        path.x = batch.x + path_wrapper.x;
        path.y = batch.y + path_wrapper.y;
        setBlendMode(state);
        submitPathFill(path, layer, submit_pass);
      }
    }
  }

  template<>
  inline void submitShapes<LineWrapper>(const BatchVector<LineWrapper>& batches, BlendMode state,
                                        Layer& layer, int submit_pass) {
    for (const auto& batch : batches) {
      for (const LineWrapper& line_wrapper : *batch.shapes) {
        LineWrapper line = line_wrapper;
        line.x = batch.x + line_wrapper.x;
        line.y = batch.y + line_wrapper.y;
        setBlendMode(state);
        submitLine(line, layer, submit_pass);
      }
    }
  }

  template<>
  inline void submitShapes<LineFillWrapper>(const BatchVector<LineFillWrapper>& batches,
                                            BlendMode state, Layer& layer, int submit_pass) {
    for (const auto& batch : batches) {
      for (const LineFillWrapper& line_fill_wrapper : *batch.shapes) {
        LineFillWrapper line_fill = line_fill_wrapper;
        line_fill.x = batch.x + line_fill.x;
        line_fill.y = batch.y + line_fill.y;
        setBlendMode(state);
        submitLineFill(line_fill, layer, submit_pass);
      }
    }
  }

  template<>
  inline void submitShapes<ImageWrapper>(const BatchVector<ImageWrapper>& batches, BlendMode state,
                                         Layer& layer, int submit_pass) {
    setBlendMode(state);
    submitImages(batches, layer, submit_pass);
  }

  template<>
  inline void submitShapes<ShaderWrapper>(const BatchVector<ShaderWrapper>& batches,
                                          BlendMode state, Layer& layer, int submit_pass) {
    setBlendMode(state);
    submitShader(batches, layer, submit_pass);
  }

  template<>
  inline void submitShapes<TextBlock>(const BatchVector<TextBlock>& batches, BlendMode state,
                                      Layer& layer, int submit_pass) {
    setBlendMode(state);
    submitText(batches, layer, submit_pass);
  }

  template<>
  inline void submitShapes<SampleRegion>(const BatchVector<SampleRegion>& batches, BlendMode state,
                                         Layer& layer, int submit_pass) {
    PostEffect* post_effect = batches[0].shapes->front().post_effect;

    if (post_effect) {
      for (const auto& batch : batches) {
        for (const SampleRegion& sample_layer : *batch.shapes) {
          setBlendMode(state);
          sample_layer.post_effect->submit(sample_layer, layer, submit_pass, batch.x, batch.y);
        }
      }
    }
    else {
      setBlendMode(state);
      submitSampleRegions(batches, layer, submit_pass);
    }
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
    virtual void clear() = 0;
    virtual void submit(Layer& layer, int submit_pass, const std::vector<PositionedBatch>& others) = 0;

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
    bool match(const void* id, BlendMode blend_mode, bool radial_gradient) const {
      return id_ == id && blend_mode_ == blend_mode && radial_gradient_ == radial_gradient;
    }

    void setBlendMode(BlendMode blend_mode) { blend_mode_ = blend_mode; }
    BlendMode blendMode() const { return blend_mode_; }
    bool radialGradient() const { return radial_gradient_; }

    int compare(const void* other_id, BlendMode other_blend_mode) const {
      if (id_ < other_id)
        return -1;
      if (id_ > other_id)
        return 1;
      if (blend_mode_ < other_blend_mode)
        return -1;
      if (blend_mode_ > other_blend_mode)
        return 1;
      return 0;
    }

    int compare(const SubmitBatch* other) const { return compare(other->id_, other->blend_mode_); }

    void clearAreas() { areas_.clear(); }
    void addShapeArea(const BaseShape& shape) {
      VISAGE_ASSERT(id_ == nullptr || id_ == shape.batch_id);
      id_ = shape.batch_id;
      radial_gradient_ = shape.radialGradient();
      areas_.push_back({ shape.x, shape.y, shape.x + shape.width, shape.y + shape.height });
    }

  private:
    struct Area {
      float x, y, right, bottom;
    };

    const void* id_ = nullptr;
    std::vector<Area> areas_;
    BlendMode blend_mode_;
    bool radial_gradient_ = false;
  };

  template<typename T>
  class ShapeBatch : public SubmitBatch {
  public:
    explicit ShapeBatch(BlendMode blend_mode) : SubmitBatch(blend_mode) { }
    ~ShapeBatch() override = default;

    void clear() override {
      clearAreas();
      shapes_.clear();
    }

    void submit(Layer& layer, int submit_pass, const std::vector<PositionedBatch>& batches) override {
      BatchVector<T> batch_list;
      batch_list.reserve(batches.size());
      for (const PositionedBatch& batch : batches) {
        VISAGE_ASSERT(batch.batch->id() == id());
        const std::vector<T>* shapes = &reinterpret_cast<ShapeBatch<T>*>(batch.batch)->shapes_;
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
      for (auto& batch : batches_) {
        batch->clear();
        unused_batches_[batch->id()].push_back(std::move(batch));
      }
      batches_.clear();
    }

    void submit(Layer& layer, int submit_pass) {
      for (auto& batch : batches_)
        batch->submit(layer, submit_pass, {});
    }

    int autoBatchIndex(const BaseShape& shape, BlendMode blend) const {
      int match = batches_.size();
      int insert = batches_.size();
      for (int i = batches_.size() - 1; i >= 0; --i) {
        SubmitBatch* batch = batches_[i].get();
        if (batch->match(shape.batch_id, blend, shape.radialGradient()))
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

    int batchIndex(const BaseShape& shape, BlendMode blend) const {
      if (manual_batching_)
        return manualBatchIndex(shape);
      return autoBatchIndex(shape, blend);
    }

    template<typename T>
    ShapeBatch<T>* createNewBatch(const void* id, BlendMode blend, int insert_index) {
      if (!unused_batches_[id].empty()) {
        auto batch = std::move(unused_batches_[id].back());
        unused_batches_[id].pop_back();
        batch->setBlendMode(blend);
        batches_.insert(batches_.begin() + insert_index, std::move(batch));
      }
      else
        batches_.insert(batches_.begin() + insert_index, std::make_unique<ShapeBatch<T>>(blend));

      return reinterpret_cast<ShapeBatch<T>*>(batches_[insert_index].get());
    }

    template<typename T>
    void addShape(T shape, BlendMode blend = BlendMode::Alpha) {
      int batch_index = batchIndex(shape, blend);
      bool match = batch_index < batches_.size() &&
                   batches_[batch_index]->match(shape.batch_id, blend, shape.radialGradient());
      ShapeBatch<T>* batch = match ? reinterpret_cast<ShapeBatch<T>*>(batches_[batch_index].get()) :
                                     createNewBatch<T>(shape.batch_id, blend, batch_index);

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