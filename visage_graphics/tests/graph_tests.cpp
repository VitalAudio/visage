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

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <functional>

using namespace visage;

static Screenshot render(int width, int height, const std::function<void(Canvas&)>& draw) {
  Canvas canvas;
  canvas.setWindowless(width, height);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, width, height);
  canvas.setColor(0xffffffff);
  draw(canvas);
  canvas.submit();
  return canvas.takeScreenshot();
}

static GraphData flatData(int num_points, float value) {
  GraphData data(num_points);
  for (int i = 0; i < num_points; ++i)
    data[i] = value;
  return data;
}

static int coverage(const Screenshot& screenshot, int x, int y) {
  return screenshot.sample(x, y).hexRed();
}

TEST_CASE("Graph line is centered on its data with area coverage at its edges", "[graphics]") {
  Screenshot screenshot = render(100, 100, [](Canvas& canvas) {
    canvas.graphLine(flatData(2, 0.25f), 0, 0, 100, 40, 2.0f);
    canvas.graphLine(flatData(2, 0.25f + 0.25f / 40.0f), 0, 50, 100, 40, 2.0f);
  });

  for (int x : { 10, 50, 90 }) {
    INFO("x " << x);
    REQUIRE(coverage(screenshot, x, 8) <= 2);
    REQUIRE(coverage(screenshot, x, 9) >= 0xfd);
    REQUIRE(coverage(screenshot, x, 10) >= 0xfd);
    REQUIRE(coverage(screenshot, x, 11) <= 2);

    REQUIRE(std::abs(coverage(screenshot, x, 59) - 0xbf) <= 3);
    REQUIRE(coverage(screenshot, x, 60) >= 0xfd);
    REQUIRE(std::abs(coverage(screenshot, x, 61) - 0x40) <= 3);
  }
}

TEST_CASE("Graph line draws every point on dense data", "[graphics]") {
  static constexpr int kPoints = 4001;
  GraphData data = flatData(kPoints, 0.75f);
  data[kPoints / 2] = 0.25f;
  Screenshot screenshot = render(100, 40, [&data](Canvas& canvas) {
    canvas.graphLine(data, 0, 0, 100, 40, 6.0f);
  });

  REQUIRE(coverage(screenshot, 49, 9) >= 0xf0);
  REQUIRE(coverage(screenshot, 50, 9) >= 0xf0);
  REQUIRE(coverage(screenshot, 50, 20) >= 0xf0);
}

TEST_CASE("Graph line stays inside its bounds", "[graphics]") {
  Screenshot screenshot = render(100, 40, [](Canvas& canvas) {
    canvas.graphLine(flatData(2, 0.0f), 10, 10, 80, 20, 4.0f);
  });

  for (int x : { 10, 50, 89 }) {
    INFO("x " << x);
    REQUIRE(coverage(screenshot, x, 9) <= 2);
    REQUIRE(coverage(screenshot, x, 10) >= 0xfd);
    REQUIRE(coverage(screenshot, x, 11) >= 0xfd);
    REQUIRE(coverage(screenshot, x, 12) <= 2);
  }
  REQUIRE(coverage(screenshot, 9, 10) <= 2);
  REQUIRE(coverage(screenshot, 90, 10) <= 2);
}

TEST_CASE("Graph fill covers its bounds down to the fill center", "[graphics]") {
  Screenshot screenshot = render(120, 60, [](Canvas& canvas) {
    canvas.graphFill(flatData(2, 0.25f), 10, 5, 100, 40, 1.0f);
  });

  for (int x : { 10, 60, 109 }) {
    INFO("x " << x);
    REQUIRE(coverage(screenshot, x, 14) <= 2);
    REQUIRE(coverage(screenshot, x, 15) >= 0xfd);
    REQUIRE(coverage(screenshot, x, 44) >= 0xfd);
    REQUIRE(coverage(screenshot, x, 45) <= 2);
  }
  for (int y : { 15, 30, 44 }) {
    INFO("y " << y);
    REQUIRE(coverage(screenshot, 9, y) <= 2);
    REQUIRE(coverage(screenshot, 110, y) <= 2);
  }
}

TEST_CASE("Graph fill covers the exact area between dense data and the fill center", "[graphics]") {
  static constexpr int kPoints = 4001;
  GraphData data(kPoints);
  for (int i = 0; i < kPoints; ++i)
    data[i] = i % 2 ? 0.75f : 0.25f;
  Screenshot screenshot = render(100, 40, [&data](Canvas& canvas) {
    canvas.graphFill(data, 0, 0, 100, 40, 0.5f);
  });

  for (int x : { 20, 50, 80 }) {
    for (int y = 6; y < 34; ++y) {
      INFO("x " << x << " y " << y);
      float distance = std::abs(y + 0.5f - 20.0f);
      float expected = std::max(0.0f, 0.5f - distance / 20.0f);
      REQUIRE(std::abs(coverage(screenshot, x, y) - expected * 255.0f) <= 3.0f);
    }
  }
}

TEST_CASE("Graph fill covers both sides of a fill center inside a pixel", "[graphics]") {
  static constexpr int kPoints = 4001;
  GraphData data(kPoints);
  for (int i = 0; i < kPoints; ++i)
    data[i] = i % 2 ? 0.75f : 0.25f;
  Screenshot screenshot = render(100, 41, [&data](Canvas& canvas) {
    canvas.graphFill(data, 0, 0, 100, 41, 0.5f);
  });

  for (int x : { 20, 50, 80 }) {
    INFO("x " << x);
    REQUIRE(std::abs(coverage(screenshot, x, 19) - (0.5f - 1.0f / 20.5f) * 255.0f) <= 3.0f);
    REQUIRE(std::abs(coverage(screenshot, x, 20) - (0.5f - 0.25f / 20.5f) * 255.0f) <= 3.0f);
    REQUIRE(std::abs(coverage(screenshot, x, 21) - (0.5f - 1.0f / 20.5f) * 255.0f) <= 3.0f);
  }
}

TEST_CASE("Graph line with no thickness draws nothing", "[graphics]") {
  Screenshot screenshot = render(100, 40, [](Canvas& canvas) {
    canvas.graphLine(flatData(5, 0.5f), 0, 0, 100, 40, 0.0f);
  });

  REQUIRE(coverage(screenshot, 50, 20) == 0);
}
