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

#include "path.h"

#include <memory>
#include <vector>

namespace visage {
  struct PathTileSegment {
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
  };

  struct PathTile {
    int start = 0;
    int count = 0;
    int backdrop = 0;
    int cover = -1;
  };

  class PathTileBinner {
  public:
    static constexpr int kTileSize = 16;

    static int tilesFor(int pixels) { return (pixels + kTileSize - 1) / kTileSize; }

    void clear() {
      tiles_.clear();
      segments_.clear();
      covers_.clear();
    }

    int addPath(const Path& path, int width, int height);

    const std::vector<PathTile>& tiles() const { return tiles_; }
    const std::vector<PathTileSegment>& segments() const { return segments_; }
    const std::vector<float>& covers() const { return covers_; }

  private:
    struct Entry {
      int tile = 0;
      PathTileSegment segment;
    };

    std::vector<PathTile> tiles_;
    std::vector<PathTileSegment> segments_;
    std::vector<float> covers_;
    std::vector<Entry> entries_;
    std::vector<int> backdrop_deltas_;
  };

  class PathTileRenderer {
  public:
    struct Cell {
      const Path* path = nullptr;
      int x = 0;
      int y = 0;
      int width = 0;
      int height = 0;
    };

    static bool computeSupported();

    PathTileRenderer();
    ~PathTileRenderer();

    bool submitCpu(const std::vector<Cell>& cells, int submit_pass, int atlas_width, int atlas_height);
    bool submitGpu(const std::vector<Cell>& cells, int compute_pass, int raster_pass,
                   int atlas_width, int atlas_height);

  private:
    struct Resources;

    bool submitCpuBatch(const std::vector<Cell>& cells, int begin, int end, int submit_pass,
                        int atlas_width, int atlas_height);

    PathTileBinner binner_;
    std::unique_ptr<Resources> resources_;
  };
}
