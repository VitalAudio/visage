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

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <complex>
#include <random>
#include <set>

using namespace visage;

inline float randomFloat(float min, float max) {
  static std::random_device random_device;
  static std::mt19937 generator(random_device());
  std::uniform_real_distribution distribution(min, max);
  return distribution(generator);
}

struct PathTriangle {
  PathTriangle(const Point& a, const Point& b, const Point& c) : points { a, b, c } { }

  bool operator<(const PathTriangle& other) const { return points < other.points; }
  bool operator==(const PathTriangle& other) const {
    static constexpr float kEpsilon = 1e-5f;
    return points.size() == other.points.size() &&
           std::equal(points.begin(), points.end(), other.points.begin(),
                      [](const Point& p1, const Point& p2) {
                        if ((p1 - p2).squareMagnitude() < kEpsilon)
                          return true;
                        return false;
                      });
  }

  std::set<Point> points;
};

TEST_CASE("Degeneracies", "[graphics]") {
  static constexpr int kWidth = 100;

  SECTION("Infinity path with one path having point at intersection") {
    Path path;
    path.moveTo(10, 10);
    path.lineTo(90, 90);
    path.lineTo(20, 80);
    path.lineTo(50, 50);
    path.lineTo(80, 30);

    Canvas canvas;
    canvas.setWindowless(kWidth, kWidth);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(path, 0, 0, kWidth, kWidth);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();
    screenshot.save("infinity.png");

    for (int i = 0; i < path.subPaths()[0].points.size() - 2; i += 3) {
      Point p0 = path.subPaths()[0].points[i];
      Point p1 = path.subPaths()[0].points[i + 1];
      Point p2 = path.subPaths()[0].points[i + 2];
      Point inside = (p0 + p1 + p2) / 3.0f;
      Color sample = screenshot.sample(inside);
      REQUIRE(sample.hexRed() == 0xff);
    }

    Color sample_left = screenshot.sample(45, 50);
    Color sample_right = screenshot.sample(55, 50);
    REQUIRE(sample_left.hexRed() == 0);
    REQUIRE(sample_right.hexRed() == 0);
  }

  SECTION("Infinity path with points at intersection") {
    Path path;
    path.moveTo(15, 10);
    path.lineTo(50, 50);
    path.lineTo(90, 90);
    path.lineTo(20, 80);
    path.lineTo(50, 50);
    path.lineTo(80, 30);

    Canvas canvas;
    canvas.setWindowless(kWidth, kWidth);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(path, 0, 0, kWidth, kWidth);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();

    Color sample_top = screenshot.sample(50, 45);
    REQUIRE(sample_top.hexRed() == 0xff);
    Color sample_bottom = screenshot.sample(50, 55);
    REQUIRE(sample_bottom.hexRed() == 0xff);

    Color sample_left = screenshot.sample(45, 50);
    REQUIRE(sample_left.hexRed() == 0);
    Color sample_right = screenshot.sample(55, 50);
    REQUIRE(sample_right.hexRed() == 0);
  }

  SECTION("Degeneracy rectangle in rectangle corner") {
    Path path;
    path.moveTo(10, 10);
    path.lineTo(40, 10);
    path.lineTo(40, 40);
    path.lineTo(10, 40);
    path.close();

    path.moveTo(10, 10);
    path.lineTo(30, 10);
    path.lineTo(30, 30);
    path.lineTo(10, 30);
    path.close();

    Canvas canvas;
    canvas.setWindowless(50, 50);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(path, 0, 0, kWidth, kWidth);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();

    REQUIRE(screenshot.sample(10, 10).hexRed() == 0);
    REQUIRE(screenshot.sample(29, 29).hexRed() == 0);
    REQUIRE(screenshot.sample(10, 10).hexRed() == 0);
  }

  SECTION("Degeneracy embedded rectangles sharing two points") {
    Path path;
    path.moveTo(10, 10);
    path.lineTo(40, 10);
    path.lineTo(40, 40);
    path.lineTo(10, 40);
    path.close();

    path.moveTo(10, 10);
    path.lineTo(30, 10);
    path.lineTo(30, 40);
    path.lineTo(10, 40);
    path.close();

    Canvas canvas;
    canvas.setWindowless(50, 50);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(path, 0, 0, kWidth, kWidth);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();

    REQUIRE(screenshot.sample(10, 10).hexRed() == 0);
    REQUIRE(screenshot.sample(29, 29).hexRed() == 0);
    REQUIRE(screenshot.sample(10, 10).hexRed() == 0);
  }

  SECTION("Degeneracy rectangle in rectangle middle") {
    Path path;
    path.moveTo(10, 10);
    path.lineTo(40, 10);
    path.lineTo(40, 40);
    path.lineTo(10, 40);
    path.close();

    path.moveTo(20, 10);
    path.lineTo(30, 10);
    path.lineTo(30, 40);
    path.lineTo(20, 40);
    path.close();

    Canvas canvas;
    canvas.setWindowless(50, 50);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(path, 0, 0, kWidth, kWidth);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();

    REQUIRE(screenshot.sample(10, 10).hexRed() == 0xff);
    REQUIRE(screenshot.sample(10, 25).hexRed() == 0xff);
    REQUIRE(screenshot.sample(10, 39).hexRed() == 0xff);
    REQUIRE(screenshot.sample(21, 10).hexRed() == 0);
    REQUIRE(screenshot.sample(21, 25).hexRed() == 0);
    REQUIRE(screenshot.sample(21, 40).hexRed() == 0);
    REQUIRE(screenshot.sample(29, 10).hexRed() == 0);
    REQUIRE(screenshot.sample(29, 20).hexRed() == 0);
    REQUIRE(screenshot.sample(29, 40).hexRed() == 0);
    REQUIRE(screenshot.sample(31, 10).hexRed() == 0xff);
    REQUIRE(screenshot.sample(31, 20).hexRed() == 0xff);
    REQUIRE(screenshot.sample(31, 39).hexRed() == 0xff);
  }

  SECTION("Degeneracy begin point on existing line") {
    Path path;
    path.moveTo(10, 10);
    path.lineTo(40, 10);
    path.lineTo(40, 40);
    path.lineTo(10, 40);
    path.close();

    path.moveTo(20, 10);
    path.lineTo(30, 0);
    path.lineTo(30, 20);
    path.close();

    Canvas canvas;
    canvas.setWindowless(50, 50);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(path, 0, 0, kWidth, kWidth);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();

    REQUIRE(screenshot.sample(10, 10).hexRed() == 0xff);
    REQUIRE(screenshot.sample(25, 8).hexRed() == 0xff);
    REQUIRE(screenshot.sample(25, 12).hexRed() == 0x00);
  }

  SECTION("Degeneracy begin point on two existing lines") {
    Path path;
    path.moveTo(0, 0);
    path.lineTo(0, 20);
    path.lineTo(10, 10);
    path.lineTo(90, 10);
    path.lineTo(100, 20);
    path.lineTo(100, 0);
    path.lineTo(90, 10);
    path.lineTo(10, 10);
    path.close();

    path.moveTo(20, 10);
    path.lineTo(30, 0);
    path.lineTo(30, 20);
    path.close();

    Canvas canvas;
    canvas.setWindowless(100, 50);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(path, 0, 0, kWidth, kWidth);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();

    REQUIRE(screenshot.sample(5, 10).hexRed() == 0xff);
    REQUIRE(screenshot.sample(15, 10).hexRed() == 0x00);
    REQUIRE(screenshot.sample(25, 10).hexRed() == 0xff);
    REQUIRE(screenshot.sample(35, 10).hexRed() == 0x00);
    REQUIRE(screenshot.sample(95, 10).hexRed() == 0xff);
  }

  SECTION("Degeneracy center point star") {
    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr int kStarPoints = 10;
    static constexpr float kRadius = 40.0f;

    Path star;
    float phase = randomFloat(0.0f, 1.0f);
    std::complex<float> position(std::cos(-2.0f * kPi * phase / kStarPoints),
                                 std::sin(-2.0f * kPi * phase / kStarPoints));

    float center = kWidth * 0.5f;
    star.moveTo(center, center);
    std::complex<float> delta(std::cos(2.0f * kPi / kStarPoints), std::sin(2.0f * kPi / kStarPoints));

    for (int i = 0; i < kStarPoints; ++i) {
      position = position * delta;
      star.lineTo(center + kRadius * position.real(), center + kRadius * position.imag());
      if (i % 2)
        star.lineTo(center, center);
    }
    star.close();
    Canvas canvas;
    canvas.setWindowless(kWidth, kWidth);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, kWidth, kWidth);
    canvas.setColor(0xffff0000);
    canvas.fill(star);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();

    for (int i = 0; i < star.subPaths()[0].points.size() - 2; i += 3) {
      Point p0 = star.subPaths()[0].points[i];
      Point p1 = star.subPaths()[0].points[i + 1];
      Point p2 = star.subPaths()[0].points[i + 2];
      Point inside = (p0 + p1 + p2) / 3.0f;
      Color sample = screenshot.sample(inside);
      REQUIRE(sample.hexRed() == 0xff);
    }

    for (int i = 2; i < star.subPaths()[0].points.size() - 2; i += 3) {
      Point p0 = star.subPaths()[0].points[i];
      Point p1 = star.subPaths()[0].points[i + 1];
      Point p2 = star.subPaths()[0].points[i + 2];
      Point outside = (p0 + p1 + p2) / 3.0f;
      Color sample = screenshot.sample(outside);
      REQUIRE(sample.hexRed() == 0);
    }
  }
  //
  // SECTION("Vertical cross line degeneracy") {
  //   Path path;
  //   path.moveTo(10, 10);
  //   path.lineTo(40, 10);
  //   path.lineTo(40, 30);
  //   path.lineTo(50, 25);
  //   path.lineTo(40, 20);
  //   path.lineTo(40, 40);
  //   path.lineTo(10, 40);
  //
  //   Canvas canvases[2];
  //   for (auto& canvas : canvases) {
  //     canvas.setWindowless(50, 40);
  //     canvas.setColor(0xff000000);
  //     canvas.fill(0, 0, canvas.width(), canvas.height());
  //     canvas.setColor(0xffff0000);
  //   }
  //
  //   canvases[0].fill(path, 0, 0, canvases[0].width(), canvases[0].height());
  //   canvases[0].submit();
  //   path.reverse();
  //   canvases[1].fill(path, 0, 0, canvases[1].width(), canvases[1].height());
  //   canvases[1].submit();
  //   Screenshot screenshots[] = { canvases[0].takeScreenshot(), canvases[1].takeScreenshot() };
  //
  //   for (const auto& screenshot : screenshots) {
  //     REQUIRE(screenshot.sample(5, 20).hexRed() == 0x00);
  //     REQUIRE(screenshot.sample(20, 5).hexRed() == 0x00);
  //     REQUIRE(screenshot.sample(20, 20).hexRed() == 0xff);
  //     REQUIRE(screenshot.sample(45, 25).hexRed() == 0xff);
  //     REQUIRE(screenshot.sample(45, 30).hexRed() == 0x00);
  //     REQUIRE(screenshot.sample(45, 20).hexRed() == 0x00);
  //     REQUIRE(screenshot.sample(35, 15).hexRed() == 0xff);
  //     REQUIRE(screenshot.sample(35, 35).hexRed() == 0xff);
  //   }
  // }
}