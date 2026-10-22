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

#include "visage_graphics/canvas.h"
#include "visage_graphics/path.h"
#include "visage_graphics/path_tiles.h"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <random>
#include <set>

using namespace visage;

static float edgeArea(float x1, float y1, float x2, float y2) {
  auto clamped_area = [](float y) {
    float c = std::clamp(y, 0.0f, 1.0f);
    return 0.5f * c * c + std::max(y - 1.0f, 0.0f);
  };

  float dx = x2 - x1;
  if (dx == 0.0f)
    return 0.0f;

  float x_min = std::clamp(std::min(x1, x2), 0.0f, 1.0f);
  float x_max = std::clamp(std::max(x1, x2), 0.0f, 1.0f);
  float slope = (y2 - y1) / dx;
  float y_min = y1 + slope * (x_min - x1);
  float y_max = y1 + slope * (x_max - x1);
  float delta = y_max - y_min;
  float average = std::abs(delta) > 0.0001f ? (clamped_area(y_max) - clamped_area(y_min)) / delta :
                                              std::clamp(0.5f * (y_min + y_max), 0.0f, 1.0f);
  return (dx > 0.0f ? 1.0f : -1.0f) * (x_max - x_min) * average;
}

static float tiledCoverage(const PathTileBinner& binner, int tile_base, int width, int x, int y) {
  static constexpr int kTile = PathTileBinner::kTileSize;
  int column = x / kTile;
  int row = y / kTile;
  const PathTile& tile = binner.tiles()[tile_base + row * PathTileBinner::tilesFor(width) + column];
  float pixel_x = x - column * kTile;
  float pixel_y = y - row * kTile;
  float result = tile.backdrop;
  if (tile.cover >= 0)
    result += binner.covers()[tile.cover + x - column * kTile];
  for (int i = tile.start; i < tile.start + tile.count; ++i) {
    const PathTileSegment& segment = binner.segments()[i];
    result += edgeArea(segment.x1 - pixel_x, segment.y1 - pixel_y, segment.x2 - pixel_x,
                       segment.y2 - pixel_y);
  }
  return result;
}

static double directCoverage(const Path& path, int x, int y) {
  double result = 0.0;
  for (const auto& sub_path : path.subPaths()) {
    if (sub_path.points.size() <= 2)
      continue;

    Point previous = sub_path.points.back();
    for (const Point& point : sub_path.points) {
      result += edgeArea(previous.x - x, previous.y - y, point.x - x, point.y - y);
      previous = point;
    }
  }
  return result;
}

static void checkTiledCoverage(const Path& path, int width, int height) {
  PathTileBinner binner;
  binner.addPath(Path(), width, height);
  int tile_base = binner.addPath(path, width, height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double expected = directCoverage(path, x, y);
      double tiled = tiledCoverage(binner, tile_base, width, x, y);
      INFO("pixel " << x << ", " << y);
      REQUIRE(std::abs(tiled - expected) < 2e-3);
    }
  }
}

TEST_CASE("Tiled path coverage matches direct edge coverage on grid-aligned shapes", "[graphics]") {
  Path rectangle;
  rectangle.addRectangle(0.0f, 0.0f, 48.0f, 32.0f);
  checkTiledCoverage(rectangle, 48, 32);

  Path inner;
  inner.addRectangle(16.0f, 16.0f, 16.0f, 16.0f);
  checkTiledCoverage(inner, 64, 64);

  Path diamond;
  diamond.moveTo(32.0f, 0.0f);
  diamond.lineTo(64.0f, 32.0f);
  diamond.lineTo(32.0f, 64.0f);
  diamond.lineTo(0.0f, 32.0f);
  diamond.close();
  checkTiledCoverage(diamond, 64, 64);

  Path corners;
  corners.moveTo(16.0f, 16.0f);
  corners.lineTo(48.0f, 48.0f);
  corners.lineTo(16.0f, 48.0f);
  corners.lineTo(48.0f, 16.0f);
  corners.close();
  checkTiledCoverage(corners, 64, 64);

  Path outside;
  outside.moveTo(-40.0f, -8.0f);
  outside.lineTo(90.0f, 20.0f);
  outside.lineTo(30.0f, 120.0f);
  outside.lineTo(-10.0f, 70.0f);
  outside.close();
  checkTiledCoverage(outside, 50, 37);
}

TEST_CASE("Tiled path coverage matches direct edge coverage on random paths", "[graphics]") {
  std::mt19937 random(5);
  std::uniform_real_distribution<float> position(-30.0f, 110.0f);
  std::uniform_int_distribution<int> grid(-2, 7);
  std::uniform_int_distribution<int> kind(0, 3);
  std::uniform_int_distribution<int> count(3, 12);

  for (int i = 0; i < 300; ++i) {
    Path path;
    path.setFillRule(i % 2 ? Path::FillRule::EvenOdd : Path::FillRule::NonZero);
    int sub_paths = 1 + i % 3;
    for (int s = 0; s < sub_paths; ++s) {
      int points = count(random);
      Point last;
      for (int p = 0; p < points; ++p) {
        Point point(position(random), position(random));
        int k = kind(random);
        if (k == 1)
          point = Point(grid(random) * 16.0f, grid(random) * 16.0f);
        else if (k == 2)
          point.x = grid(random) * 16.0f;
        else if (k == 3 && p > 0)
          point.y = last.y;

        if (p == 0)
          path.moveTo(point);
        else
          path.lineTo(point);
        last = point;
      }
      path.close();
    }

    INFO("path " << i);
    checkTiledCoverage(path, 80, 80);
  }
}

static std::vector<uint8_t> renderPaths(PathAtlas::Binning binning, const std::vector<Path>& paths,
                                        unsigned int color, int width, int height) {
  PathAtlas::setBinning(binning);
  Canvas canvas;
  canvas.setWindowless(width, height);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, width, height);
  canvas.setColor(color);
  for (const Path& path : paths)
    canvas.fill(path);
  canvas.submit();
  PathAtlas::setBinning(PathAtlas::Binning::Gpu);

  const Screenshot& screenshot = canvas.takeScreenshot();
  return std::vector<uint8_t>(screenshot.data(), screenshot.data() + width * height * 4);
}

static Path randomPolygon(std::mt19937& random, Point center, float radius, int points) {
  std::uniform_real_distribution<float> offset(-radius, radius);
  Path path;
  for (int p = 0; p < points; ++p) {
    Point point = center + Point(offset(random), offset(random));
    if (p == 0)
      path.moveTo(point);
    else
      path.lineTo(point);
  }
  path.close();
  return path;
}

TEST_CASE("CPU and GPU tile binning render the same paths", "[graphics]") {
  static constexpr int kWidth = 300;
  static constexpr int kHeight = 220;

  std::mt19937 random(11);
  std::uniform_real_distribution<float> position(-60.0f, 360.0f);
  std::uniform_int_distribution<int> count(3, 40);

  std::vector<Path> paths;
  Path circle;
  circle.addCircle(150.0f, 110.0f, 95.5f);
  paths.push_back(circle);
  for (int i = 0; i < 24; ++i) {
    float radius = i < 6 ? 400.0f : 40.0f;
    Path path = randomPolygon(random, Point(position(random), position(random) * 0.7f), radius,
                              count(random));
    path.setFillRule(i % 2 ? Path::FillRule::EvenOdd : Path::FillRule::NonZero);
    paths.push_back(path);
  }

  std::vector<uint8_t> cpu = renderPaths(PathAtlas::Binning::Cpu, paths, 0x60ffffff, kWidth, kHeight);
  std::vector<uint8_t> gpu = renderPaths(PathAtlas::Binning::Gpu, paths, 0x60ffffff, kWidth, kHeight);
  INFO("GPU binning active " << (PathAtlas::activeBinning() == PathAtlas::Binning::Gpu));

  int max_difference = 0;
  std::set<int> levels;
  for (int i = 0; i < cpu.size(); ++i) {
    max_difference = std::max(max_difference, std::abs(cpu[i] - gpu[i]));
    levels.insert(cpu[i]);
  }
  REQUIRE(levels.size() > 200);
  REQUIRE(max_difference <= 1);
}

TEST_CASE("Tile binned rendering matches direct edge coverage", "[graphics]") {
  static constexpr int kWidth = 160;
  static constexpr int kHeight = 120;

  std::mt19937 random(23);
  for (auto rule : { Path::FillRule::NonZero, Path::FillRule::EvenOdd }) {
    Path path = randomPolygon(random, Point(80.0f, 60.0f), 90.0f, 24);
    path.setFillRule(rule);
    for (auto binning : { PathAtlas::Binning::Cpu, PathAtlas::Binning::Gpu }) {
      std::vector<uint8_t> pixels = renderPaths(binning, { path }, 0xffffffff, kWidth, kHeight);
      for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
          double coverage = std::abs(directCoverage(path, x, y));
          double alpha = rule == Path::FillRule::EvenOdd ?
                             1.0 - std::abs(std::fmod(coverage, 2.0) - 1.0) :
                             std::min(coverage, 1.0);
          INFO("binning " << static_cast<int>(binning) << " pixel " << x << ", " << y);
          REQUIRE(std::abs(pixels[(y * kWidth + x) * 4] - 255.0 * alpha) <= 1.0);
        }
      }
    }
  }
}

TEST_CASE("CPU tile binning renders a frame that splits the vertex buffer pool", "[graphics]") {
  static constexpr int kWidth = 1200;
  static constexpr int kHeight = 900;
  static constexpr int kRings = 1200;

  std::vector<Path> rings;
  for (int i = 0; i < kRings; ++i) {
    float x = 45.0f + (i % 40) * 27.7f;
    float y = 45.0f + (i / 40) * 27.3f;
    Path ring;
    ring.setFillRule(Path::FillRule::EvenOdd);
    ring.addCircle(x, y, 40.0f);
    ring.addCircle(x, y, 25.0f);
    rings.push_back(ring);
  }

  std::vector<uint8_t> cpu = renderPaths(PathAtlas::Binning::Cpu, rings, 0x10ffffff, kWidth, kHeight);
  std::vector<uint8_t> gpu = renderPaths(PathAtlas::Binning::Gpu, rings, 0x10ffffff, kWidth, kHeight);
  int max_difference = 0;
  std::set<int> levels;
  for (int i = 0; i < cpu.size(); ++i) {
    max_difference = std::max(max_difference, std::abs(cpu[i] - gpu[i]));
    levels.insert(cpu[i]);
  }
  REQUIRE(levels.size() > 50);
  REQUIRE(max_difference <= 1);
}
