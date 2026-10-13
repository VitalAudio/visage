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

TEST_CASE("Paths larger than the path atlas", "[graphics]") {
  static constexpr int kWidth = 16;
  static constexpr float kPathSize = 20000.0f;

  Path path;
  path.addRectangle(-kPathSize, -kPathSize, 2.0f * kPathSize, 2.0f * kPathSize);

  Canvas canvas;
  canvas.setWindowless(kWidth, kWidth);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kWidth, kWidth);
  canvas.setColor(0xffff0000);
  canvas.fill(path, 0, 0, kWidth, kWidth);
  canvas.fill(path, 0, 0, kWidth, kWidth);
  canvas.submit();
  const auto& screenshot = canvas.takeScreenshot();

  REQUIRE(screenshot.sample(0, 0).hexRed() == 0xff);
  REQUIRE(screenshot.sample(kWidth / 2, kWidth / 2).hexRed() == 0xff);
  REQUIRE(screenshot.sample(kWidth - 1, kWidth - 1).hexRed() == 0xff);
}

TEST_CASE("More paths than fit in transient buffers", "[graphics]") {
  static constexpr int kWidth = 64;
  static constexpr int kNumPaths = 120000;

  Path path;
  path.addRectangle(0, 0, 4, 4);

  Canvas canvas;
  canvas.setWindowless(kWidth, kWidth);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kWidth, kWidth);
  canvas.setColor(0xffff0000);
  for (int i = 0; i < kNumPaths; ++i)
    canvas.fill(path, i % 60, (i / 60) % 60, 4, 4);
  canvas.submit();
}

TEST_CASE("Path atlas after removing paths", "[graphics]") {
  static constexpr int kWidth = 32;
  static constexpr int kHalf = kWidth / 2;

  Path square;
  square.addRectangle(2, 2, kHalf - 4, kHalf - 4);
  Path triangle;
  triangle.moveTo(2, 2);
  triangle.lineTo(kHalf - 2, 2);
  triangle.lineTo(2, kHalf - 2);
  triangle.close();

  Canvas canvas;
  canvas.setWindowless(kWidth, kWidth);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kWidth, kWidth);
  canvas.setColor(0xffff0000);
  canvas.fill(square, 0, 0, kHalf, kHalf);
  canvas.fill(triangle, kHalf, 0, kHalf, kHalf);
  canvas.fill(square, 0, kHalf, kHalf, kHalf);
  canvas.fill(triangle, kHalf, kHalf, kHalf, kHalf);
  canvas.submit();

  canvas.clearDrawnShapes();
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kWidth, kWidth);
  canvas.setColor(0xffff0000);
  canvas.fill(triangle, 0, 0, kHalf, kHalf);
  canvas.fill(square, kHalf, kHalf, kHalf, kHalf);
  const Screenshot& screenshot = canvas.takeScreenshot();

  REQUIRE(screenshot.sample(4, 4).hexRed() == 0xff);
  REQUIRE(screenshot.sample(kHalf - 4, kHalf - 4).hexRed() == 0x00);
  REQUIRE(screenshot.sample(kHalf + 4, 4).hexRed() == 0x00);
  REQUIRE(screenshot.sample(4, kHalf + 4).hexRed() == 0x00);
  REQUIRE(screenshot.sample(kHalf + kHalf / 2, kHalf + kHalf / 2).hexRed() == 0xff);
}

TEST_CASE("SVG path parsing", "[graphics]") {
  SECTION("Implicit line to after move") {
    auto commands = Path::parseSvgPath("M10 10 20 20 30 10");
    REQUIRE(commands.size() == 3);
    REQUIRE(commands[0].type == 'M');
    REQUIRE(commands[1].type == 'L');
    REQUIRE(commands[1].end == Point(20, 20));
    REQUIRE(commands[2].type == 'L');
    REQUIRE(commands[2].end == Point(30, 10));

    commands = Path::parseSvgPath("M10,10,20,20,30,30");
    REQUIRE(commands.size() == 3);
    REQUIRE(commands[1].type == 'L');
    REQUIRE(commands[2].type == 'L');
    REQUIRE(commands[2].end == Point(30, 30));
  }

  SECTION("Relative implicit line to after move") {
    auto commands = Path::parseSvgPath("m10 10 20 20");
    REQUIRE(commands.size() == 2);
    REQUIRE(commands[0].type == 'M');
    REQUIRE(commands[1].type == 'L');
    REQUIRE(commands[1].end == Point(30, 30));
  }

  SECTION("Exponent with sign") {
    auto commands = Path::parseSvgPath("M0 0L1e-1 5l2E+1,-2e-1");
    REQUIRE(commands.size() == 3);
    REQUIRE(commands[1].end.x == Catch::Approx(0.1f));
    REQUIRE(commands[1].end.y == Catch::Approx(5.0f));
    REQUIRE(commands[2].end.x == Catch::Approx(20.1f));
    REQUIRE(commands[2].end.y == Catch::Approx(4.8f));
  }

  SECTION("Invalid characters don't hang") {
    REQUIRE(Path::parseSvgPath("10 10").empty());
    REQUIRE(Path::parseSvgPath("M0 0 X1 1").size() == 1);
    REQUIRE(Path::parseSvgPath("M0 0L5 5z,").back().type == 'Z');
    REQUIRE(Path::parseSvgPath("M0 0L5 5Z;").back().type == 'Z');
  }
}

static bool samePoints(const std::string& a, const std::string& b) {
  Path path_a, path_b;
  path_a.loadSvgPath(a);
  path_b.loadSvgPath(b);
  return path_a.subPaths()[0].points == path_b.subPaths()[0].points;
}

TEST_CASE("Smooth curves without a previous curve", "[graphics]") {
  REQUIRE(samePoints("M0 0 S10 10 10 0", "M0 0 C0 0 10 10 10 0"));
  REQUIRE(samePoints("M0 0 C0 10 10 10 10 0 L20 0 S30 10 30 0",
                     "M0 0 C0 10 10 10 10 0 L20 0 C20 0 30 10 30 0"));
  REQUIRE(samePoints("M0 0 Q5 10 10 0 L20 0 T30 0", "M0 0 Q5 10 10 0 L20 0 Q20 0 30 0"));
}

TEST_CASE("Arc with a huge radius", "[graphics]") {
  Path path;
  path.moveTo(0, 0);
  path.arcTo(2e7f, 2e7f, 0.0f, false, true, Point(1000, 0));
  path.lineTo(1000, 50);

  const std::vector<Point>& points = path.subPaths().back().points;
  REQUIRE(points.size() == 3);
  REQUIRE((points[1] - Point(1000, 0)).length() < 0.01f);
}

TEST_CASE("Square join on a straight vertex", "[graphics]") {
  Path path;
  path.moveTo(0, 0);
  path.lineTo(5, 0);
  path.lineTo(10, 0);
  path.lineTo(10, 10);
  path.lineTo(0, 10);
  path.close();

  Path offset = path.offset(1.0f);
  REQUIRE(offset.numPoints() > 0);
}

TEST_CASE("Stroke with dashes smaller than float precision", "[graphics]") {
  Path path;
  path.moveTo(0, 0);
  path.lineTo(1000, 0);

  Path stroke = path.stroke(1.0f, Path::Join::Round, Path::EndCap::Round, { 1e-5f });
  REQUIRE(stroke.numPoints() > 0);
}
