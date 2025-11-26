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

    REQUIRE(screenshot.sample(10, 10).hexRed() <= 1);
    REQUIRE(screenshot.sample(29, 29).hexRed() <= 1);
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

    REQUIRE(screenshot.sample(10, 10).hexRed() <= 1);
    REQUIRE(screenshot.sample(29, 29).hexRed() <= 1);
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

    REQUIRE(screenshot.sample(10, 10).hexRed() >= 0xfe);
    REQUIRE(screenshot.sample(25, 8).hexRed() >= 0xfe);
    REQUIRE(screenshot.sample(25, 12).hexRed() <= 0x01);
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

    REQUIRE(screenshot.sample(5, 10).hexRed() >= 0xfe);
    REQUIRE(screenshot.sample(15, 10).hexRed() <= 0x01);
    REQUIRE(screenshot.sample(25, 10).hexRed() >= 0xfe);
    REQUIRE(screenshot.sample(35, 10).hexRed() <= 0x01);
    REQUIRE(screenshot.sample(95, 10).hexRed() >= 0xfe);
  }

  SECTION("Vertical cross line degeneracy") {
    Path path;
    path.moveTo(10, 10);
    path.lineTo(40, 10);
    path.lineTo(40, 30);
    path.lineTo(50, 25);
    path.lineTo(40, 20);
    path.lineTo(40, 40);
    path.lineTo(10, 40);

    Canvas canvas;
    canvas.setWindowless(50, 40);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);

    canvas.fill(path, 0, 0, canvas.width(), canvas.height());
    canvas.submit();
    Screenshot screenshot = canvas.takeScreenshot();

    REQUIRE(screenshot.sample(5, 20).hexRed() == 0x00);
    REQUIRE(screenshot.sample(20, 5).hexRed() == 0x00);
    REQUIRE(screenshot.sample(20, 20).hexRed() == 0xff);
    REQUIRE(screenshot.sample(45, 25).hexRed() == 0xff);
    REQUIRE(screenshot.sample(45, 30).hexRed() == 0x00);
    REQUIRE(screenshot.sample(45, 20).hexRed() == 0x00);
    REQUIRE(screenshot.sample(35, 15).hexRed() == 0xff);
    REQUIRE(screenshot.sample(35, 35).hexRed() == 0xff);
  }
}
