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

#include "path_tiles.h"

#include "embedded/shaders.h"
#include "gpu/gpu.h"
#include "graphics_caches.h"
#include "shape_batcher.h"
#include "uniforms.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

namespace visage {
  static constexpr float kPathTile = PathTileBinner::kTileSize;
  static constexpr int kMaxPathTileTexels = 1 << 22;
  static constexpr int kMaxExactTexelIndex = 1 << 23;

  static int pathTileCell(float value, int count) {
    float cell = std::floor(value * (1.0f / kPathTile));
    if (cell < 0.0f)
      return -1;
    if (cell >= count)
      return count;
    return static_cast<int>(cell);
  }

  template<typename Visitor>
  static void walkPathTiles(Point from, Point to, int columns, int rows, Visitor& visitor) {
    int column = pathTileCell(from.x, columns);
    int row = pathTileCell(from.y, rows);
    int end_column = pathTileCell(to.x, columns);
    int end_row = pathTileCell(to.y, rows);
    auto add_piece = [&](Point start, Point end) {
      if (start != end && column >= 0 && column < columns && row >= 0 && row < rows)
        visitor.piece(column, row, start, end);
    };

    int x_step = end_column > column ? 1 : -1;
    int y_step = end_row > row ? 1 : -1;
    int x_line = column + (x_step > 0 ? 1 : 0);
    int y_line = row + (y_step > 0 ? 1 : 0);
    int x_remaining = std::abs(end_column - column);
    int y_remaining = std::abs(end_row - row);
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    Point previous = from;
    while (x_remaining || y_remaining) {
      float x_t = x_remaining ? (x_line * kPathTile - from.x) / dx : FLT_MAX;
      float y_t = y_remaining ? (y_line * kPathTile - from.y) / dy : FLT_MAX;
      if (x_t <= y_t) {
        Point point(x_line * kPathTile, from.y + x_t * dy);
        add_piece(previous, point);
        previous = point;
        visitor.columnCrossing(x_line, row, x_step);
        column += x_step;
        x_line += x_step;
        --x_remaining;
      }
      else {
        Point point(from.x + y_t * dx, y_line * kPathTile);
        add_piece(previous, point);
        previous = point;
        visitor.rowCrossing(column, std::min(row, row + y_step), point.x, y_step);
        row += y_step;
        y_line += y_step;
        --y_remaining;
      }
    }

    add_piece(previous, to);
  }

  int PathTileBinner::addPath(const Path& path, int width, int height) {
    int columns = tilesFor(width);
    int rows = tilesFor(height);
    int tile_base = tiles_.size();
    tiles_.resize(tile_base + columns * rows);
    entries_.clear();
    backdrop_deltas_.assign(columns * rows, 0);

    struct Visitor {
      void add(int tile, const PathTileSegment& segment) {
        entries.push_back({ tile, segment });
        tiles[tile].count++;
      }

      void piece(int column, int row, Point start, Point end) {
        float left = column * kPathTile;
        float top = row * kPathTile;
        add(row * columns + column, { start.x - left, start.y - top, end.x - left, end.y - top });
      }

      void columnCrossing(int column, int row, int direction) {
        if (column >= 0 && column < columns && row > 0)
          backdrop_deltas[(std::min(row, rows) - 1) * columns + column] += direction;
      }

      void rowCrossing(int column, int row, float x, int direction) {
        if (column < 0 || column >= columns || row < 0 || row >= rows)
          return;

        float local_x = std::clamp(x - column * kPathTile, 0.0f, kPathTile);
        if (local_x == kPathTile)
          return;

        PathTile& tile = tiles[row * columns + column];
        if (tile.cover < 0) {
          tile.cover = covers.size();
          covers.resize(covers.size() + kTileSize);
        }

        int pixel = static_cast<int>(local_x);
        float fraction = local_x - pixel;
        covers[tile.cover + pixel] += direction * (1.0f - fraction);
        if (pixel + 1 < kTileSize)
          covers[tile.cover + pixel + 1] += direction * fraction;
      }

      int columns = 0;
      int rows = 0;
      PathTile* tiles = nullptr;
      std::vector<Entry>& entries;
      std::vector<int>& backdrop_deltas;
      std::vector<float>& covers;
    };

    int cover_base = covers_.size();
    Visitor visitor { columns, rows, tiles_.data() + tile_base, entries_, backdrop_deltas_, covers_ };
    for (const auto& sub_path : path.subPaths()) {
      const std::vector<Point>& points = sub_path.points;
      if (points.size() <= 2)
        continue;

      bool finite = std::all_of(points.begin(), points.end(), [](const Point& point) {
        return std::isfinite(point.x) && std::isfinite(point.y);
      });
      if (!finite)
        continue;

      Point previous = points.back();
      for (const Point& point : points) {
        walkPathTiles(previous, point, columns, rows, visitor);
        previous = point;
      }
    }

    int start = segments_.size();
    for (int i = tile_base; i < tiles_.size(); ++i) {
      tiles_[i].start = start;
      start += tiles_[i].count;
      tiles_[i].count = 0;
    }

    segments_.resize(start);
    for (const Entry& entry : entries_) {
      PathTile& tile = tiles_[tile_base + entry.tile];
      segments_[tile.start + tile.count++] = entry.segment;
    }

    for (int i = cover_base; i < covers_.size(); i += kTileSize) {
      for (int pixel = 1; pixel < kTileSize; ++pixel)
        covers_[i + pixel] += covers_[i + pixel - 1];
    }

    for (int column = 0; column < columns; ++column) {
      int backdrop = 0;
      for (int row = rows - 1; row >= 0; --row) {
        backdrop += backdrop_deltas_[row * columns + column];
        tiles_[tile_base + row * columns + column].backdrop = backdrop;
      }
    }

    return tile_base;
  }

  static int pathTileSegmentBound(Point from, Point to, int columns, int rows) {
    int column_crossings = std::abs(pathTileCell(to.x, columns) - pathTileCell(from.x, columns));
    int row_crossings = std::abs(pathTileCell(to.y, rows) - pathTileCell(from.y, rows));
    return column_crossings + row_crossings + 1;
  }

  template<const char* name>
  static void setTileUniform(float value0, float value1 = 0.0f, float value2 = 0.0f, float value3 = 0.0f) {
    float values[4] = { value0, value1, value2, value3 };
    const gpu::UniformHandle& uniform = UniformCache::uniformHandle(name);
    gpu::setUniform(uniform, values);
  }

  template<const char* name>
  static void setTileTexture(int stage, gpu::TextureHandle handle) {
    const gpu::UniformHandle& uniform = UniformCache::uniformHandle(name, UniformCache::Sampler);
    gpu::setTexture(stage, uniform, handle);
  }

  static void setTileBounds(int width, int height) {
    if (gpu::getCaps()->origin_bottom_left)
      setTileUniform<Uniforms::kBounds>(2.0f / width, 2.0f / height, -1.0f, -1.0f);
    else
      setTileUniform<Uniforms::kBounds>(2.0f / width, -2.0f / height, -1.0f, 1.0f);
  }

  struct PathTileVertex {
    float x;
    float y;
    float origin_x;
    float origin_y;
    float start;
    float count;
    float backdrop;
    float cover;
    float even_odd;

    static gpu::VertexLayout& layout() {
      static gpu::VertexLayout layout;
      static bool initialized = false;

      if (!initialized) {
        initialized = true;
        layout.begin()
            .add(gpu::Attrib::Position, 4)
            .add(gpu::Attrib::TexCoord0, 4)
            .add(gpu::Attrib::TexCoord1, 1)
            .end();
      }

      return layout;
    }
  };

  static gpu::VertexLayout& pathTileVec4Layout() {
    static gpu::VertexLayout layout;
    static bool initialized = false;

    if (!initialized) {
      initialized = true;
      layout.begin().add(gpu::Attrib::TexCoord0, 4).end();
    }

    return layout;
  }

  template<typename T>
  static void destroyTileHandle(T& handle) {
    if (gpu::isValid(handle))
      gpu::destroy(handle);
    handle = {};
  }

  static uint32_t tileBufferCapacity(uint32_t needed) {
    uint32_t capacity = 1024;
    while (capacity < needed)
      capacity *= 2;
    return capacity;
  }

  static bool ensureTileBuffer(gpu::BufferHandle& handle, uint32_t& capacity, uint32_t needed,
                               uint16_t flags) {
    if (gpu::isValid(handle) && capacity >= needed)
      return true;

    destroyTileHandle(handle);
    capacity = tileBufferCapacity(needed);
    handle = gpu::createDynamicVertexBuffer(capacity, pathTileVec4Layout(), flags);
    return gpu::isValid(handle);
  }

  static bool ensureTileIndexBuffer(gpu::BufferHandle& handle, uint32_t& capacity, uint32_t needed,
                                    uint16_t flags) {
    if (gpu::isValid(handle) && capacity >= needed)
      return true;

    destroyTileHandle(handle);
    capacity = tileBufferCapacity(needed);
    handle = gpu::createDynamicIndexBuffer(capacity, flags | gpu::kBufferIndex32);
    return gpu::isValid(handle);
  }

  static bool ensureTileTexture(gpu::TextureHandle& handle, int& rows, int width, int needed_texels,
                                uint32_t flags) {
    int max_size = gpu::getCaps()->max_texture_size;
    int needed_rows = std::max(1, (needed_texels + width - 1) / width);
    if (needed_rows > max_size)
      return false;
    if (gpu::isValid(handle) && rows >= needed_rows)
      return true;

    destroyTileHandle(handle);
    rows = 16;
    while (rows < needed_rows)
      rows *= 2;
    rows = std::min(rows, max_size);
    handle = gpu::createTexture2D(width, rows, gpu::TextureFormat::RGBA32F,
                                  flags | gpu::kSamplerPoint | gpu::kSamplerClamp);
    return gpu::isValid(handle);
  }

  static int tileTextureWidth() {
    static constexpr int kTextureWidth = 2048;
    return std::min<int>(kTextureWidth, gpu::getCaps()->max_texture_size);
  }

  struct PathTileRenderer::Resources {
    ~Resources() {
      destroyTileHandle(segment_texture);
      destroyTileHandle(gpu_segment_texture);
      destroyTileHandle(gpu_tile_texture);
      destroyTileHandle(edges);
      destroyTileHandle(edge_paths);
      destroyTileHandle(paths);
      destroyTileHandle(tiles);
      destroyTileHandle(counter);
      destroyTileHandle(covers);
      destroyTileHandle(clear_program);
      destroyTileHandle(bin_program);
      destroyTileHandle(columns_program);
    }

    gpu::TextureHandle segment_texture = {};
    int segment_texture_rows = 0;
    std::vector<int> tile_bases;
    std::vector<int> segment_bases;
    std::vector<int> cover_bases;

    gpu::TextureHandle gpu_segment_texture = {};
    gpu::TextureHandle gpu_tile_texture = {};
    int gpu_segment_texture_rows = 0;
    int gpu_tile_texture_rows = 0;
    gpu::BufferHandle edges = {};
    gpu::BufferHandle edge_paths = {};
    gpu::BufferHandle paths = {};
    gpu::BufferHandle tiles = {};
    gpu::BufferHandle counter = {};
    gpu::BufferHandle covers = {};
    uint32_t edge_capacity = 0;
    uint32_t edge_path_capacity = 0;
    uint32_t path_capacity = 0;
    uint32_t tile_capacity = 0;
    uint32_t counter_capacity = 0;
    uint32_t cover_capacity = 0;
    gpu::ProgramHandle clear_program = {};
    gpu::ProgramHandle bin_program = {};
    gpu::ProgramHandle columns_program = {};
    std::vector<float> edge_data;
    std::vector<uint32_t> edge_path_data;
    std::vector<float> path_data;
  };

  bool PathTileRenderer::computeSupported() {
#if VISAGE_EMSCRIPTEN
    return false;
#else
    const gpu::Caps* caps = gpu::getCaps();
    return caps->compute &&
           (caps->formats[static_cast<int>(gpu::TextureFormat::RGBA32F)] & gpu::kFormatStorage);
#endif
  }

  PathTileRenderer::PathTileRenderer() : resources_(std::make_unique<Resources>()) { }
  PathTileRenderer::~PathTileRenderer() = default;

  bool PathTileRenderer::submitCpu(const std::vector<Cell>& cells, int submit_pass, int atlas_width,
                                   int atlas_height) {
    Resources& resources = *resources_;
    binner_.clear();
    resources.tile_bases.clear();
    resources.segment_bases.clear();
    resources.cover_bases.clear();
    for (const Cell& cell : cells) {
      resources.segment_bases.push_back(binner_.segments().size());
      resources.cover_bases.push_back(binner_.covers().size());
      resources.tile_bases.push_back(binner_.addPath(*cell.path, cell.width, cell.height));
    }
    resources.segment_bases.push_back(binner_.segments().size());
    resources.cover_bases.push_back(binner_.covers().size());

    auto texels = [&](int begin, int end) {
      return (resources.segment_bases[end] - resources.segment_bases[begin]) +
             (resources.cover_bases[end] - resources.cover_bases[begin]) / 4;
    };

    bool succeeded = true;
    for (int begin = 0; begin < cells.size();) {
      int end = begin + 1;
      while (end < cells.size() && texels(begin, end + 1) <= kMaxPathTileTexels)
        ++end;

      if (begin > 0)
        flushBufferPoolFrame();
      succeeded = submitCpuBatch(cells, begin, end, submit_pass, atlas_width, atlas_height) && succeeded;
      begin = end;
    }
    return succeeded;
  }

  bool PathTileRenderer::submitCpuBatch(const std::vector<Cell>& cells, int begin, int end,
                                        int submit_pass, int atlas_width, int atlas_height) {
    Resources& resources = *resources_;
    const std::vector<PathTileSegment>& segments = binner_.segments();
    const std::vector<float>& covers = binner_.covers();
    int segment_base = resources.segment_bases[begin];
    int cover_base = resources.cover_bases[begin];
    int num_segments = resources.segment_bases[end] - segment_base;
    int num_covers = resources.cover_bases[end] - cover_base;
    int texels = num_segments + num_covers / 4;
    bool oversized = texels > kMaxPathTileTexels;
    if (oversized) {
      VISAGE_LOG("PathTileRenderer::submitCpu: A path has too many segments to rasterize");
      texels = 0;
    }

    int texture_width = tileTextureWidth();
    if (!ensureTileTexture(resources.segment_texture, resources.segment_texture_rows, texture_width,
                           texels, 0)) {
      VISAGE_LOG("PathTileRenderer::submitCpu: Failed to allocate the segment texture");
      return false;
    }

    if (texels) {
      int texture_rows = (texels + texture_width - 1) / texture_width;
      uint32_t bytes = texture_width * texture_rows * sizeof(PathTileSegment);
      const gpu::Memory* memory = gpu::alloc(bytes);
      size_t segment_bytes = num_segments * sizeof(PathTileSegment);
      size_t cover_bytes = num_covers * sizeof(float);
      std::memcpy(memory->data, segments.data() + segment_base, segment_bytes);
      std::memcpy(memory->data + segment_bytes, covers.data() + cover_base, cover_bytes);
      std::memset(memory->data + segment_bytes + cover_bytes, 0, bytes - segment_bytes - cover_bytes);
      gpu::updateTexture2D(resources.segment_texture, 0, 0, texture_width, texture_rows, memory);
    }

    const std::vector<PathTile>& tiles = binner_.tiles();
    const PathTile empty_tile;
    auto for_each_quad = [&](auto&& callback) {
      for (int i = begin; i < end; ++i) {
        const Cell& cell = cells[i];
        bool even_odd = cell.path->fillRule() == Path::FillRule::EvenOdd;
        int columns = PathTileBinner::tilesFor(cell.width);
        int rows = PathTileBinner::tilesFor(cell.height);
        for (int row = 0; row < rows; ++row) {
          const PathTile* tile_row = tiles.data() + resources.tile_bases[i] + row * columns;
          auto tile_at = [&](int column) -> const PathTile& {
            return oversized ? empty_tile : tile_row[column];
          };
          int top = row * PathTileBinner::kTileSize;
          int bottom = std::min(top + PathTileBinner::kTileSize, cell.height);
          for (int column = 0; column < columns;) {
            const PathTile& tile = tile_at(column);
            int run_end = column + 1;
            if (tile.count == 0 && tile.cover < 0) {
              while (run_end < columns && tile_at(run_end).count == 0 &&
                     tile_at(run_end).cover < 0 && tile_at(run_end).backdrop == tile.backdrop)
                ++run_end;
            }

            int left = column * PathTileBinner::kTileSize;
            int right = std::min(run_end * PathTileBinner::kTileSize, cell.width);
            callback(cell.x + left, cell.y + top, cell.x + right, cell.y + bottom, tile, even_odd);
            column = run_end;
          }
        }
      }
    };

    int num_quads = 0;
    for_each_quad([&](int, int, int, int, const PathTile&, bool) { ++num_quads; });
    if (num_quads == 0)
      return true;

    auto submit = [&] {
      gpu::setState(gpu::RenderState::write(gpu::kWriteR));
      setTileBounds(atlas_width, atlas_height);
      setTileUniform<Uniforms::kDimensions>(texture_width);
      setTileTexture<Uniforms::kTexture>(0, resources.segment_texture);
      gpu::submit(submit_pass, ProgramCache::programHandle(shaders::vs_path_tile, shaders::fs_path_tile));
    };

    QuadChunker<PathTileVertex> chunker(num_quads);
    bool failed = false;
    for_each_quad([&](int left, int top, int right, int bottom, const PathTile& tile, bool even_odd) {
      if (failed)
        return;

      PathTileVertex* vertices = chunker.nextQuad(submit);
      if (vertices == nullptr) {
        failed = true;
        return;
      }

      float positions[kVerticesPerQuad][2] = { { static_cast<float>(left), static_cast<float>(top) },
                                               { static_cast<float>(right), static_cast<float>(top) },
                                               { static_cast<float>(left), static_cast<float>(bottom) },
                                               { static_cast<float>(right), static_cast<float>(bottom) } };
      for (int v = 0; v < kVerticesPerQuad; ++v) {
        vertices[v].x = positions[v][0];
        vertices[v].y = positions[v][1];
        vertices[v].origin_x = left;
        vertices[v].origin_y = top;
        vertices[v].start = tile.start - segment_base;
        vertices[v].count = tile.count;
        vertices[v].backdrop = tile.backdrop;
        vertices[v].cover = tile.cover < 0 ? -1.0f : num_segments + (tile.cover - cover_base) / 4;
        vertices[v].even_odd = even_odd ? 1.0f : 0.0f;
      }
    });

    if (failed) {
      VISAGE_LOG("PathTileRenderer::submitCpu: Failed to allocate vertex buffers");
      return false;
    }

    chunker.finish(submit);
    return true;
  }

  bool PathTileRenderer::submitGpu(const std::vector<Cell>& cells, int compute_pass,
                                   int raster_pass, int atlas_width, int atlas_height) {
#if VISAGE_EMSCRIPTEN
    return false;
#else
    static constexpr uint32_t kThreadGroupSize = 64;
    static constexpr uint32_t kMaxGroups = 65535;

    Resources& resources = *resources_;
    resources.edge_data.clear();
    resources.edge_path_data.clear();
    resources.path_data.clear();

    uint32_t num_tiles = 0;
    uint32_t num_columns = 0;
    uint64_t segment_bound = 0;
    for (int i = 0; i < cells.size(); ++i) {
      const Cell& cell = cells[i];
      int columns = PathTileBinner::tilesFor(cell.width);
      int rows = PathTileBinner::tilesFor(cell.height);
      float even_odd = cell.path->fillRule() == Path::FillRule::EvenOdd ? 1.0f : 0.0f;
      float path_values[] = { static_cast<float>(cell.x),
                              static_cast<float>(cell.y),
                              static_cast<float>(cell.width),
                              static_cast<float>(cell.height),
                              static_cast<float>(num_tiles),
                              static_cast<float>(columns),
                              static_cast<float>(rows),
                              static_cast<float>(num_columns),
                              even_odd,
                              0.0f,
                              0.0f,
                              0.0f };
      resources.path_data.insert(resources.path_data.end(), std::begin(path_values), std::end(path_values));

      for (const auto& sub_path : cell.path->subPaths()) {
        const std::vector<Point>& points = sub_path.points;
        if (points.size() <= 2)
          continue;

        bool finite = std::all_of(points.begin(), points.end(), [](const Point& point) {
          return std::isfinite(point.x) && std::isfinite(point.y);
        });
        if (!finite)
          continue;

        Point previous = points.back();
        for (const Point& point : points) {
          if (previous != point) {
            float edge[] = { previous.x, previous.y, point.x, point.y };
            resources.edge_data.insert(resources.edge_data.end(), std::begin(edge), std::end(edge));
            resources.edge_path_data.push_back(i);
            segment_bound += pathTileSegmentBound(previous, point, columns, rows);
          }
          previous = point;
        }
      }

      num_tiles += columns * rows;
      num_columns += columns;
    }

    if (num_tiles == 0)
      return true;

    uint32_t num_edges = resources.edge_path_data.size();
    uint32_t num_paths = cells.size();
    if (segment_bound > kMaxPathTileTexels || segment_bound + 4 * uint64_t(num_tiles) >= kMaxExactTexelIndex)
      return false;

    if (!gpu::isValid(resources.clear_program)) {
      resources.clear_program = gpu::createProgram(ShaderCache::shaderHandle(shaders::cs_path_tile_clear),
                                                   false);
      resources.bin_program = gpu::createProgram(ShaderCache::shaderHandle(shaders::cs_path_tile_bin), false);
      resources.columns_program = gpu::createProgram(ShaderCache::shaderHandle(shaders::cs_path_tile_columns),
                                                     false);
    }

    int texture_width = tileTextureWidth();
    uint32_t segment_capacity = std::max<uint32_t>(1, segment_bound);
    uint16_t read = gpu::kBufferComputeRead;
    uint16_t read_write = gpu::kBufferComputeReadWrite;
    bool allocated = ensureTileBuffer(resources.edges, resources.edge_capacity,
                                      std::max(1u, num_edges), read) &&
                     ensureTileIndexBuffer(resources.edge_paths, resources.edge_path_capacity,
                                           std::max(1u, num_edges), read) &&
                     ensureTileBuffer(resources.paths, resources.path_capacity, 3 * num_paths, read) &&
                     ensureTileIndexBuffer(resources.tiles, resources.tile_capacity, 4 * num_tiles,
                                           read_write) &&
                     ensureTileIndexBuffer(resources.counter, resources.counter_capacity, 4, read_write) &&
                     ensureTileIndexBuffer(resources.covers, resources.cover_capacity,
                                           16 * num_tiles, read_write) &&
                     ensureTileTexture(resources.gpu_segment_texture,
                                       resources.gpu_segment_texture_rows, texture_width,
                                       segment_capacity + 4 * num_tiles, gpu::kTextureStorage) &&
                     ensureTileTexture(resources.gpu_tile_texture, resources.gpu_tile_texture_rows,
                                       texture_width, 3 * num_tiles, gpu::kTextureStorage);
    if (!allocated || !gpu::isValid(resources.columns_program)) {
      VISAGE_LOG("PathTileRenderer::submitGpu: Failed to allocate compute resources");
      return false;
    }

    if (num_edges) {
      gpu::update(resources.edges, 0,
                  gpu::copy(resources.edge_data.data(), resources.edge_data.size() * sizeof(float)));
      gpu::update(resources.edge_paths, 0,
                  gpu::copy(resources.edge_path_data.data(), num_edges * sizeof(uint32_t)));
    }
    gpu::update(resources.paths, 0,
                gpu::copy(resources.path_data.data(), resources.path_data.size() * sizeof(float)));

    auto dispatch = [&](gpu::ProgramHandle program, uint32_t count, float value) {
      uint32_t groups = (count + kThreadGroupSize - 1) / kThreadGroupSize;
      uint32_t groups_x = std::min(groups, kMaxGroups);
      uint32_t groups_y = (groups + groups_x - 1) / groups_x;
      setTileUniform<Uniforms::kTileParams>(count, groups_x * kThreadGroupSize, value, segment_capacity);
      setTileUniform<Uniforms::kTileLayout>(texture_width, segment_capacity);
      gpu::dispatch(compute_pass, program, groups_x, groups_y, 1);
    };

    gpu::setBuffer(0, resources.tiles, gpu::Access::ReadWrite);
    gpu::setBuffer(1, resources.counter, gpu::Access::ReadWrite);
    gpu::setBuffer(2, resources.covers, gpu::Access::ReadWrite);
    dispatch(resources.clear_program, 16 * num_tiles, 0.0f);

    auto bind_bin_inputs = [&] {
      gpu::setBuffer(0, resources.edges, gpu::Access::Read);
      gpu::setBuffer(1, resources.edge_paths, gpu::Access::Read);
      gpu::setBuffer(2, resources.paths, gpu::Access::Read);
      gpu::setBuffer(3, resources.tiles, gpu::Access::ReadWrite);
      gpu::setImage(4, resources.gpu_segment_texture, gpu::Access::Write);
      gpu::setBuffer(5, resources.covers, gpu::Access::ReadWrite);
    };

    if (num_edges) {
      bind_bin_inputs();
      dispatch(resources.bin_program, num_edges, 0.0f);
    }

    gpu::setBuffer(0, resources.paths, gpu::Access::Read);
    gpu::setBuffer(1, resources.tiles, gpu::Access::ReadWrite);
    gpu::setBuffer(2, resources.counter, gpu::Access::ReadWrite);
    gpu::setBuffer(3, resources.covers, gpu::Access::Read);
    gpu::setImage(4, resources.gpu_segment_texture, gpu::Access::Write);
    gpu::setImage(5, resources.gpu_tile_texture, gpu::Access::Write);
    dispatch(resources.columns_program, num_columns, num_paths);

    if (num_edges) {
      bind_bin_inputs();
      dispatch(resources.bin_program, num_edges, 1.0f);
    }

    gpu::setVertexCount(6 * num_tiles);
    gpu::setState(gpu::RenderState::write(gpu::kWriteR));
    setTileBounds(atlas_width, atlas_height);
    setTileUniform<Uniforms::kDimensions>(texture_width);
    setTileTexture<Uniforms::kTexture>(0, resources.gpu_segment_texture);
    setTileTexture<Uniforms::kTexture2>(1, resources.gpu_tile_texture);
    gpu::submit(raster_pass, ProgramCache::programHandle(shaders::vs_path_tile_gpu, shaders::fs_path_tile));
    return true;
#endif
  }
}
