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
#include <map>
#include <memory>

namespace visage {
  static constexpr uint64_t blendModeValue(BlendMode blend_mode) {
    switch (blend_mode) {
    case BlendMode::Opaque:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ZERO);
    case BlendMode::Alpha:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_INV_SRC_ALPHA);
    case BlendMode::Add:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ONE);
    case BlendMode::Sub:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC_SEPARATE(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ONE,
                                            BGFX_STATE_BLEND_ZERO, BGFX_STATE_BLEND_ONE) |
             BGFX_STATE_BLEND_EQUATION_SEPARATE(BGFX_STATE_BLEND_EQUATION_REVSUB,
                                                BGFX_STATE_BLEND_EQUATION_ADD);
    case BlendMode::Mult:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC_SEPARATE(BGFX_STATE_BLEND_DST_COLOR, BGFX_STATE_BLEND_INV_SRC_ALPHA,
                                            BGFX_STATE_BLEND_ZERO, BGFX_STATE_BLEND_ONE);
    case BlendMode::MaskAdd:
      return BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_INV_SRC_ALPHA);
    case BlendMode::MaskRemove:
      return BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ONE, BGFX_STATE_BLEND_ONE) |
             BGFX_STATE_BLEND_EQUATION(BGFX_STATE_BLEND_EQUATION_REVSUB);
    case BlendMode::MaskMult:
      return BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
             BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_ZERO, BGFX_STATE_BLEND_SRC_ALPHA);
    }

    VISAGE_ASSERT(false);
    return 0;
  }

  void setBlendMode(BlendMode blend_mode) {
    bgfx::setState(blendModeValue(blend_mode));
  }

  void submitBlended(BlendMode blend_mode, int submit_pass, bgfx::ProgramHandle program) {
    static constexpr uint64_t kMultiplyUncovered = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A |
                                                   BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_INV_DST_ALPHA,
                                                                         BGFX_STATE_BLEND_ONE);

    setBlendMode(blend_mode);
    if (blend_mode == BlendMode::Mult) {
      bgfx::submit(submit_pass, program, 0, BGFX_DISCARD_NONE);
      bgfx::setState(kMultiplyUncovered);
    }
    bgfx::submit(submit_pass, program);
  }

  template<const char* name>
  void setUniform(const void* value) {
    const bgfx::UniformHandle& uniform = UniformCache::uniformHandle(name);
    bgfx::setUniform(uniform, value);
  }

  template<const char* name>
  void setUniform(float value0, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f) {
    const bgfx::UniformHandle& uniform = UniformCache::uniformHandle(name);
    float vec[4] = { value0, value1, value2, value3 };
    bgfx::setUniform(uniform, vec);
  }

  template<const char* name>
  void setTexture(int stage, bgfx::TextureHandle handle) {
    const bgfx::UniformHandle& uniform = UniformCache::uniformHandle(name, UniformCache::Sampler);
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

  void setAlphaClamp(const Layer& layer) {
    bool hdr = layer.frameBufferFormat() == bgfx::TextureFormat::RGBA16F;
    setUniform<Uniforms::kAlphaClamp>(hdr ? 0.0f : 1.0f);
  }

  inline void setColorMult(const Layer& layer) {
    float value = Color::kGradientNormalization;
    setUniform<Uniforms::kColorMult>(value, value, value, 1.0f);
    setAlphaClamp(layer);
  }

  void setOriginFlipUniform(bool origin_flip) {
    setUniform<Uniforms::kOriginFlip>(origin_flip ? -1.0 : 1.0, origin_flip ? 1.0 : 0.0);
  }

  static void setQuadIndices(uint32_t* indices, int num_quads) {
    for (int i = 0; i < num_quads; ++i) {
      int vertex_index = i * kVerticesPerQuad;
      int index = i * kIndicesPerQuad;
      for (int v = 0; v < kIndicesPerQuad; ++v)
        indices[index + v] = vertex_index + kQuadTriangles[v];
    }
  }

  class BufferPool {
  public:
    static constexpr uint64_t kMinSize = 1 << 12;
    static constexpr uint64_t kMaxBytes = 1 << 30;
    static constexpr int kUsageFrames = 120;
#if VISAGE_BACKGROUND_GRAPHICS_THREAD
    static constexpr uint64_t kFrameLatency = 2;
#else
    static constexpr uint64_t kFrameLatency = 1;
#endif

    static void endFrame() { frame_++; }

    explicit BufferPool(const bgfx::VertexLayout* layout) :
        indices_(layout == nullptr), stride_(layout ? layout->getStride() : sizeof(uint32_t)) {
      if (layout)
        layout_ = *layout;
    }

    BufferPool(const BufferPool&) = delete;

    uint64_t capacity() const { return capacity_; }
    uint64_t usedBytes() const { return used_ * stride_; }

    uint64_t stagingBytes() const {
      uint64_t bytes = 0;
      for (const auto& staging : staging_blocks_)
        bytes += staging->size;
      return bytes;
    }

    uint8_t* allocate(uint64_t count, uint32_t* start) {
      if (used_ + count > capacity_) {
        upload();
        if (!resize(grownCapacity(frame_used_ + count)))
          return nullptr;
      }

      if (staging_ == nullptr)
        staging_ = idleStaging();

      uint8_t* data = staging_->data.get() + used_ * stride_;
      staging_->last_frame = frame_;

      *start = used_;
      used_ += count;
      frame_used_ += count;
      return data;
    }

    void upload() {
      if (staging_ == nullptr || used_ == uploaded_)
        return;

      const bgfx::Memory* memory = bgfx::makeRef(staging_->data.get() + uploaded_ * stride_,
                                                 (used_ - uploaded_) * stride_);
      if (indices_)
        bgfx::update(index_buffer_, uploaded_, memory);
      else
        bgfx::update(vertex_buffer_, uploaded_, memory);
      uploaded_ = used_;
    }

    void split() {
      split_peak_ = std::max(split_peak_, frame_used_);
      uploaded_ = 0;
      used_ = 0;
      frame_used_ = 0;
      staging_ = nullptr;
    }

    void setVertexBuffer(uint32_t start, uint32_t count) const {
      bgfx::setVertexBuffer(0, vertex_buffer_, start, count);
    }

    void setIndexBuffer(uint32_t start, uint32_t count) const {
      bgfx::setIndexBuffer(index_buffer_, start, count);
    }

    bool setQuadIndexBuffer(int num_quads) {
      if (!bgfx::isValid(quad_index_buffer_)) {
        std::vector<uint32_t> indices(capacity_ / kVerticesPerQuad * kIndicesPerQuad);
        setQuadIndices(indices.data(), capacity_ / kVerticesPerQuad);
        quad_index_buffer_ = bgfx::createIndexBuffer(bgfx::copy(indices.data(),
                                                                indices.size() * sizeof(uint32_t)),
                                                     BGFX_BUFFER_INDEX32);
        if (!bgfx::isValid(quad_index_buffer_))
          return false;
      }

      bgfx::setIndexBuffer(quad_index_buffer_, 0, num_quads * kIndicesPerQuad);
      return true;
    }

    void beginFrame() {
      uint64_t frame_peak = std::max(frame_used_, split_peak_);
      split_peak_ = 0;
      if (capacity_) {
        idle_frames_ = frame_peak ? 0 : idle_frames_ + 1;
        peak_ = std::max(peak_, frame_peak);
        if (idle_frames_ >= kUsageFrames)
          destroyBuffers();
        else if (++usage_frames_ >= kUsageFrames) {
          if (peak_ * 4 < capacity_ && shrunkCapacity() < capacity_)
            resize(shrunkCapacity());
          usage_frames_ = 0;
          peak_ = 0;
        }
      }
      uploaded_ = 0;
      used_ = 0;
      frame_used_ = 0;
      staging_ = nullptr;

      uint64_t kept_spares = 0;
      auto unneeded = [&](const std::unique_ptr<Staging>& staging) {
        if (!isIdle(*staging))
          return false;
        if (kept_spares < kFrameLatency && staging->size == capacity_ * stride_) {
          ++kept_spares;
          return false;
        }
        return true;
      };
      staging_blocks_.erase(std::remove_if(staging_blocks_.begin(), staging_blocks_.end(), unneeded),
                            staging_blocks_.end());
    }

    void destroyBuffers() {
      if (bgfx::isValid(vertex_buffer_))
        bgfx::destroy(vertex_buffer_);
      if (bgfx::isValid(index_buffer_))
        bgfx::destroy(index_buffer_);
      if (bgfx::isValid(quad_index_buffer_))
        bgfx::destroy(quad_index_buffer_);
      vertex_buffer_ = BGFX_INVALID_HANDLE;
      index_buffer_ = BGFX_INVALID_HANDLE;
      quad_index_buffer_ = BGFX_INVALID_HANDLE;
      staging_ = nullptr;
      capacity_ = 0;
      uploaded_ = 0;
      used_ = 0;
    }

    void releaseStaging() {
      staging_blocks_.clear();
      staging_ = nullptr;
    }

  private:
    struct Staging {
      explicit Staging(uint64_t size) : data(new uint8_t[size]), size(size) { }

      std::unique_ptr<uint8_t[]> data;
      uint64_t size = 0;
      uint64_t last_frame = 0;
    };

    static bool isIdle(const Staging& staging) {
      return frame_ >= staging.last_frame + kFrameLatency;
    }

    Staging* idleStaging() {
      for (auto& staging : staging_blocks_) {
        if (isIdle(*staging) && staging->size == capacity_ * stride_)
          return staging.get();
      }
      staging_blocks_.push_back(std::make_unique<Staging>(capacity_ * stride_));
      return staging_blocks_.back().get();
    }

    uint64_t maxCapacity() const { return kMaxBytes / stride_; }

    uint64_t grownCapacity(uint64_t needed) const {
      uint64_t capacity = std::max(kMinSize, 2 * capacity_);
      while (capacity < needed)
        capacity *= 2;
      capacity = std::min(capacity, std::max(needed, kBufferPoolChunkBytes / stride_));
      capacity = std::min(capacity, maxCapacity());
      return capacity < needed ? 0 : capacity;
    }

    uint64_t shrunkCapacity() const {
      uint64_t capacity = kMinSize;
      while (capacity < 2 * peak_)
        capacity *= 2;
      return std::min(capacity, maxCapacity());
    }

    bool resize(uint64_t capacity) {
      destroyBuffers();
      if (capacity == 0)
        return false;

      if (indices_)
        index_buffer_ = bgfx::createDynamicIndexBuffer(capacity, BGFX_BUFFER_INDEX32);
      else
        vertex_buffer_ = bgfx::createDynamicVertexBuffer(capacity, layout_);
      if (indices_ ? !bgfx::isValid(index_buffer_) : !bgfx::isValid(vertex_buffer_)) {
        destroyBuffers();
        return false;
      }

      capacity_ = capacity;
      usage_frames_ = 0;
      idle_frames_ = 0;
      peak_ = 0;
      return true;
    }

    static inline uint64_t frame_ = kFrameLatency;

    bool indices_ = false;
    bgfx::VertexLayout layout_;
    uint64_t stride_ = 0;
    bgfx::DynamicVertexBufferHandle vertex_buffer_ = BGFX_INVALID_HANDLE;
    bgfx::DynamicIndexBufferHandle index_buffer_ = BGFX_INVALID_HANDLE;
    bgfx::IndexBufferHandle quad_index_buffer_ = BGFX_INVALID_HANDLE;
    std::vector<std::unique_ptr<Staging>> staging_blocks_;
    Staging* staging_ = nullptr;
    uint64_t capacity_ = 0;
    uint64_t uploaded_ = 0;
    uint64_t used_ = 0;
    uint64_t frame_used_ = 0;
    uint64_t split_peak_ = 0;
    uint64_t peak_ = 0;
    int usage_frames_ = 0;
    int idle_frames_ = 0;
  };

  static std::map<uint32_t, std::unique_ptr<BufferPool>>& vertexBufferPools() {
    static auto* pools = new std::map<uint32_t, std::unique_ptr<BufferPool>>();
    return *pools;
  }

  static BufferPool& vertexBufferPool(const bgfx::VertexLayout& layout) {
    std::unique_ptr<BufferPool>& pool = vertexBufferPools()[layout.m_hash];
    if (pool == nullptr)
      pool = std::make_unique<BufferPool>(&layout);
    return *pool;
  }

  static BufferPool& indexBufferPool() {
    static auto* pool = new BufferPool(nullptr);
    return *pool;
  }

  void beginBufferPoolFrame() {
    for (auto& pool : vertexBufferPools())
      pool.second->beginFrame();
    indexBufferPool().beginFrame();
  }

  void endBufferPoolFrame() {
    BufferPool::endFrame();
  }

  uint32_t advanceFrame(uint8_t flags) {
    for (auto& pool : vertexBufferPools())
      pool.second->upload();
    indexBufferPool().upload();
    return bgfx::frame(flags);
  }

  void releaseBufferPools() {
    for (auto& pool : vertexBufferPools())
      pool.second->destroyBuffers();
    indexBufferPool().destroyBuffers();
  }

  void releaseBufferPoolStaging() {
    for (auto& pool : vertexBufferPools())
      pool.second->releaseStaging();
    indexBufferPool().releaseStaging();
  }

  uint64_t bufferPoolCapacity(const bgfx::VertexLayout& layout) {
    auto pool = vertexBufferPools().find(layout.m_hash);
    return pool == vertexBufferPools().end() ? 0 : pool->second->capacity();
  }

  uint64_t bufferPoolStagingBytes(const bgfx::VertexLayout& layout) {
    auto pool = vertexBufferPools().find(layout.m_hash);
    return pool == vertexBufferPools().end() ? 0 : pool->second->stagingBytes();
  }

  int maxQuadsPerChunk(int vertex_size) {
    return std::max<uint64_t>(1, kBufferPoolChunkBytes / (vertex_size * kVerticesPerQuad));
  }

  void flushBufferPoolFrame() {
    advanceFrame(BGFX_FRAME_FLUSH);
    for (auto& pool : vertexBufferPools())
      pool.second->split();
    indexBufferPool().split();
    BufferPool::endFrame();
  }

  void reserveBufferPoolBytes(uint64_t bytes) {
    uint64_t used = indexBufferPool().usedBytes();
    for (auto& pool : vertexBufferPools())
      used += pool.second->usedBytes();
    if (used == 0 || used + bytes <= kBufferPoolChunkBytes)
      return;

    flushBufferPoolFrame();
  }

  uint8_t* initQuadVerticesWithLayout(int num_quads, const bgfx::VertexLayout& layout) {
    BufferPool& pool = vertexBufferPool(layout);
    uint32_t start = 0;
    uint8_t* vertices = pool.allocate(num_quads * kVerticesPerQuad, &start);
    if (vertices == nullptr || !pool.setQuadIndexBuffer(num_quads))
      return nullptr;

    pool.setVertexBuffer(start, num_quads * kVerticesPerQuad);
    return vertices;
  }

  uint8_t* initVerticesWithLayout(int num_vertices, const bgfx::VertexLayout& layout) {
    BufferPool& pool = vertexBufferPool(layout);
    uint32_t start = 0;
    uint8_t* vertices = pool.allocate(num_vertices, &start);
    if (vertices)
      pool.setVertexBuffer(start, num_vertices);
    return vertices;
  }

  bool initIndexedVerticesWithLayout(int num_vertices, const bgfx::VertexLayout& layout,
                                     int num_indices, uint8_t** vertices, uint32_t** indices) {
    BufferPool& vertex_pool = vertexBufferPool(layout);
    BufferPool& index_pool = indexBufferPool();
    uint32_t vertex_start = 0;
    uint32_t index_start = 0;
    *vertices = vertex_pool.allocate(num_vertices, &vertex_start);
    *indices = reinterpret_cast<uint32_t*>(index_pool.allocate(num_indices, &index_start));
    if (*vertices == nullptr || *indices == nullptr)
      return false;

    vertex_pool.setVertexBuffer(vertex_start, num_vertices);
    index_pool.setIndexBuffer(index_start, num_indices);
    return true;
  }

  void submitShapes(const Layer& layer, BlendMode blend_mode, const EmbeddedFile& vertex_shader,
                    const EmbeddedFile& fragment_shader, bool radial_gradient, int submit_pass) {
    setTimeUniform(layer.time());
    setUniformDimensions(layer.width(), layer.height());
    setColorMult(layer);
    setOriginFlipUniform(layer.bottomLeftOrigin());
    GradientAtlas* gradient_atlas = layer.gradientAtlas();
    setUniform<Uniforms::kRadialGradient>(radial_gradient ? 1.0f : 0.0f);
    setTexture<Uniforms::kGradient>(0, gradient_atlas->colorTextureHandle());
    submitBlended(blend_mode, submit_pass, ProgramCache::programHandle(vertex_shader, fragment_shader));
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

  void submitText(const BatchVector<TextBlock>& batches, BlendMode blend_mode, const Layer& layer,
                  int submit_pass) {
    if (batches.empty() || batches[0].shapes->empty())
      return;

    const Font& font = batches[0].shapes->front().font;
    if (font.packedFont() == nullptr)
      return;

    int total_length = 0;
    for (const auto& batch : batches) {
      auto count_pieces = [&batch](int sum, const TextBlock& text_block) {
        return sum + numTextPieces(text_block, batch.x, batch.y, *batch.invalid_rects);
      };
      total_length += std::accumulate(batch.shapes->begin(), batch.shapes->end(), 0, count_pieces);
    }

    if (total_length == 0)
      return;

    auto submit = [&] {
      setUniform<Uniforms::kAtlasScale>(1.0f / font.atlasWidth(), 1.0f / font.atlasHeight());
      setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());
      setTexture<Uniforms::kTexture>(1, font.textureHandle());
      setUniformDimensions(layer.width(), layer.height());
      setColorMult(layer);
      setUniform<Uniforms::kRadialGradient>(batches[0].shapes->front().radialGradient() ? 1.0f : 0.0f);
      submitBlended(blend_mode, submit_pass,
                    ProgramCache::programHandle(shaders::vs_tinted_texture, shaders::fs_text));
    };

    QuadChunker<TextureVertex> chunker(total_length);
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

          bool sideways = text_block.direction == Direction::Left ||
                          text_block.direction == Direction::Right;
          float padding_x = sideways ? 0.0f : 0.5f;
          float padding_y = sideways ? 0.5f : 0.0f;

          TextureVertex gradient;
          PackedBrush::setVertexGradientPositions(text_block.brush, &gradient, 1, x, y, batch.x,
                                                  batch.y, x + text_block.width, y + text_block.height);
          for (int i = 0; i < length; ++i) {
            if (!overlaps(text_block.quads[i]))
              continue;

            TextureVertex* vertices = chunker.nextQuad(submit);
            if (vertices == nullptr)
              return;

            float left = x + text_block.quads[i].x - padding_x;
            float right = left + text_block.quads[i].width + 2.0f * padding_x;
            float top = y + text_block.quads[i].y - padding_y;
            float bottom = top + text_block.quads[i].height + 2.0f * padding_y;

            float texture_x = text_block.quads[i].packed_glyph->atlas_left - 0.5f;
            float texture_y = text_block.quads[i].packed_glyph->atlas_top;
            float texture_width = text_block.quads[i].packed_glyph->width + 1.0f;
            float texture_height = text_block.quads[i].packed_glyph->height;

            vertices[0].x = left;
            vertices[0].y = top;
            vertices[1].x = right;
            vertices[1].y = top;
            vertices[2].x = left;
            vertices[2].y = bottom;
            vertices[3].x = right;
            vertices[3].y = bottom;

            vertices[coordinate_index0].texture_x = texture_x;
            vertices[coordinate_index0].texture_y = texture_y;
            vertices[coordinate_index1].texture_x = texture_x + texture_width;
            vertices[coordinate_index1].texture_y = texture_y;
            vertices[coordinate_index2].texture_x = texture_x;
            vertices[coordinate_index2].texture_y = texture_y + texture_height;
            vertices[coordinate_index3].texture_x = texture_x + texture_width;
            vertices[coordinate_index3].texture_y = texture_y + texture_height;

            for (int v = 0; v < kVerticesPerQuad; ++v) {
              vertices[v].clamp_left = positioned_clamp.left;
              vertices[v].clamp_top = positioned_clamp.top;
              vertices[v].clamp_right = positioned_clamp.right;
              vertices[v].clamp_bottom = positioned_clamp.bottom;
              vertices[v].direction_x = direction_x;
              vertices[v].direction_y = direction_y;
              vertices[v].gradient_texture_position = gradient.gradient_texture_position;
              vertices[v].gradient = gradient.gradient;
            }
          }
        }
      }
    }

    chunker.finish(submit);
  }

  void submitShader(const BatchVector<ShaderWrapper>& batches, const Layer& layer, int submit_pass) {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    Shader* shader = batches[0].shapes->front().shader;
    setUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);
    setTimeUniform(layer.time());
    setUniformDimensions(layer.width(), layer.height());
    setTexture<Uniforms::kGradient>(0, layer.gradientAtlas()->colorTextureHandle());
    setColorMult(layer);
    setOriginFlipUniform(layer.bottomLeftOrigin());
    submitBlended(shader->state(), submit_pass,
                  ProgramCache::programHandle(shader->vertexShader(), shader->fragmentShader()));
  }

  void submitSampleRegions(const BatchVector<SampleRegion>& batches, BlendMode blend_mode,
                           const Layer& layer, int submit_pass) {
    auto quads = setupQuads(batches);
    if (quads.vertices == nullptr)
      return;

    setUniform<Uniforms::kRadialGradient>(quads.radial_gradient ? 1.0f : 0.0f);

    setTimeUniform(layer.time());
    Layer* source_layer = batches[0].shapes->front().region->layer();
    setUniform<Uniforms::kAtlasScale>(1.0f / source_layer->width(), 1.0f / source_layer->height());

    setTexture<Uniforms::kTexture>(0, bgfx::getTexture(source_layer->frameBuffer()));
    setUniformDimensions(layer.width(), layer.height());
    setOriginFlipUniform(layer.bottomLeftOrigin());
    submitBlended(blend_mode, submit_pass,
                  ProgramCache::programHandle(SampleRegion::vertexShader(),
                                              SampleRegion::fragmentShader()));
  }
}