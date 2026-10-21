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

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <complex>
#include <limits>
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

  SECTION("Errors keep the segments before them") {
    REQUIRE(Path::parseSvgPath("M30 40 L110 160 L150").size() == 2);
    REQUIRE(Path::parseSvgPath("M30 40 L110 160 L150#80").size() == 2);
    REQUIRE(Path::parseSvgPath("L10 10 M0 0").empty());
    REQUIRE(Path::parseSvgPath("M0 0 L10 10 Z 5 5").size() == 3);
  }

  SECTION("Arc flags") {
    auto compact = Path::parseSvgPath("M100 100 a25 25 0 1025 -25");
    REQUIRE(compact.size() == 2);
    REQUIRE(compact[1].end == Point(125, 75));

    REQUIRE(Path::parseSvgPath("M100 100 a25 25 0 -1 0 25 -25").size() == 1);
    REQUIRE(Path::parseSvgPath("M100 100 a25 25 0 1 2 25 -25").size() == 1);
    REQUIRE(Path::parseSvgPath("M100 100 a25 2501 025 -25").size() == 1);
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

TEST_CASE("Stroke dash pattern restarts for each subpath", "[graphics]") {
  Path path;
  path.moveTo(0, 0);
  path.lineTo(7, 0);
  path.moveTo(0, 10);
  path.lineTo(7, 10);

  Path stroke = path.stroke(1.0f, Path::Join::Miter, Path::EndCap::Butt, { 5.0f });
  float second_min = std::numeric_limits<float>::max();
  float second_max = std::numeric_limits<float>::lowest();
  for (const auto& sub_path : stroke.subPaths()) {
    for (const auto& point : sub_path.points) {
      if (point.y > 5.0f) {
        second_min = std::min(second_min, point.x);
        second_max = std::max(second_max, point.x);
      }
    }
  }
  REQUIRE(second_min == Catch::Approx(0.0f).margin(1e-4f));
  REQUIRE(second_max == Catch::Approx(5.0f).margin(1e-4f));
}

TEST_CASE("Relative arc with zero radius", "[graphics]") {
  Path path;
  path.moveTo(5, 5);
  path.arcTo(0.0f, 0.0f, 0.0f, false, false, Point(1, 2), true);
  REQUIRE(path.subPaths().back().points.back() == Point(6, 7));
}

TEST_CASE("Path offset stroke and combine work on const paths", "[graphics]") {
  Path source;
  source.moveTo(0, 0);
  source.lineTo(10, 0);
  source.lineTo(10, 10);
  source.close();

  const Path& path = source;
  REQUIRE(path.offset(1.0f).numPoints() > 0);
  REQUIRE(path.stroke(1.0f).numPoints() > 0);
  REQUIRE(path.combine(path).subPaths().size() == 2);
}

TEST_CASE("Path rotation matches Transform rotation", "[graphics]") {
  Path path;
  path.moveTo(1, 0);
  path.lineTo(2, 1);

  Path rotated = path.rotated(90.0f);
  Path transformed = path.transformed(Transform::rotation(90.0f));
  REQUIRE(rotated.subPaths()[0].points[0].x == Catch::Approx(0.0f).margin(1e-5f));
  REQUIRE(rotated.subPaths()[0].points[0].y == Catch::Approx(1.0f));
  for (int i = 0; i < 2; ++i) {
    REQUIRE(rotated.subPaths()[0].points[i].x ==
            Catch::Approx(transformed.subPaths()[0].points[i].x).margin(1e-5f));
    REQUIRE(rotated.subPaths()[0].points[i].y ==
            Catch::Approx(transformed.subPaths()[0].points[i].y).margin(1e-5f));
  }
}

TEST_CASE("Path command directions", "[graphics]") {
  auto arc = Path::parseSvgPath("M0 0 A10 10 0 0 1 20 0");
  REQUIRE(arc.direction(0).x == Catch::Approx(0.0f).margin(1e-5f));
  REQUIRE(arc.direction(0).y == Catch::Approx(-1.0f));
  REQUIRE(arc.direction(1).x == Catch::Approx(0.0f).margin(1e-5f));
  REQUIRE(arc.direction(1).y == Catch::Approx(1.0f));

  auto smooth_cubic = Path::parseSvgPath("M0 0 C0 10 10 10 10 0 S20 -10 20 0");
  REQUIRE(smooth_cubic.direction(1).x == Catch::Approx(0.0f).margin(1e-5f));
  REQUIRE(smooth_cubic.direction(1).y == Catch::Approx(-1.0f));

  auto smooth_quadratic = Path::parseSvgPath("M0 0 Q5 10 10 0 T20 0");
  REQUIRE(smooth_quadratic.direction(2).x == Catch::Approx(1.0f / std::sqrt(5.0f)));
  REQUIRE(smooth_quadratic.direction(2).y == Catch::Approx(2.0f / std::sqrt(5.0f)));
}

TEST_CASE("Enlarged arc radii keep their curve resolution", "[graphics]") {
  auto arc_points = [](float radius) {
    Path path;
    path.moveTo(0, 0);
    path.arcTo(radius, radius, 0.0f, false, true, Point(100, 0));
    return path.numPoints();
  };

  REQUIRE(arc_points(1.0f) == arc_points(50.0f));
}

TEST_CASE("Arcs end exactly at their end point", "[graphics]") {
  Path arc;
  arc.moveTo(140, 70);
  arc.arcTo(30, 30, 0.0f, false, true, Point(170, 100));
  arc.arcTo(30, 30, 0.0f, false, true, Point(140, 130));
  arc.arcTo(50, 20, 30.0f, true, false, Point(60, 110));
  REQUIRE(arc.subPaths()[0].points.back() == Point(60, 110));

  Path rectangle;
  rectangle.addRoundedRectangle(30, 70, 140, 60, 30);
  Path pill;
  pill.moveTo(60, 70);
  pill.lineTo(140, 70);
  pill.arcTo(30, 30, 0.0f, false, true, Point(170, 100));
  pill.arcTo(30, 30, 0.0f, false, true, Point(140, 130));
  pill.lineTo(60, 130);
  pill.arcTo(30, 30, 0.0f, false, true, Point(30, 100));
  pill.arcTo(30, 30, 0.0f, false, true, Point(60, 70));
  pill.close();

  Path dashed_rectangle = rectangle.stroke(8.0f, Path::Join::Miter, Path::EndCap::Butt, { 3.0f, 3.0f });
  Path dashed_pill = pill.stroke(8.0f, Path::Join::Miter, Path::EndCap::Butt, { 3.0f, 3.0f });
  REQUIRE(dashed_rectangle.subPaths().size() == dashed_pill.subPaths().size());
  for (int i = 0; i < dashed_pill.subPaths().size(); ++i) {
    const auto& expected = dashed_pill.subPaths()[i].points;
    const auto& points = dashed_rectangle.subPaths()[i].points;
    REQUIRE(points.size() == expected.size());
    for (int p = 0; p < points.size(); ++p)
      REQUIRE((points[p] - expected[p]).length() < 0.001f);
  }
}

static std::vector<Point> clipToHalfPlane(const std::vector<Point>& polygon, Point normal, float offset) {
  std::vector<Point> result;
  for (int i = 0; i < polygon.size(); ++i) {
    Point from = polygon[i];
    Point to = polygon[(i + 1) % polygon.size()];
    float from_distance = normal.dot(from) - offset;
    float to_distance = normal.dot(to) - offset;
    if (from_distance <= 0.0f)
      result.push_back(from);
    if ((from_distance < 0.0f) != (to_distance < 0.0f))
      result.push_back(from + (to - from) * (from_distance / (from_distance - to_distance)));
  }
  return result;
}

static float pixelCoverage(const std::vector<std::vector<Point>>& polygons, int x, int y) {
  float coverage = 0.0f;
  for (std::vector<Point> polygon : polygons) {
    polygon = clipToHalfPlane(polygon, { -1.0f, 0.0f }, -x);
    polygon = clipToHalfPlane(polygon, { 1.0f, 0.0f }, x + 1);
    polygon = clipToHalfPlane(polygon, { 0.0f, -1.0f }, -y);
    polygon = clipToHalfPlane(polygon, { 0.0f, 1.0f }, y + 1);
    float area = 0.0f;
    for (int i = 0; i < polygon.size(); ++i)
      area += polygon[i].cross(polygon[(i + 1) % polygon.size()]);
    coverage += std::abs(area) * 0.5f;
  }
  return coverage;
}

TEST_CASE("Path fill coverage is the area of each pixel inside the path", "[graphics]") {
  static constexpr int kWidth = 100;
  static constexpr int kHeight = 40;

  std::vector<std::vector<std::vector<Point>>> shapes = {
    { { { 10.25f, 10.4f }, { 90.75f, 10.4f }, { 90.75f, 30.6f }, { 10.25f, 30.6f } } },
    { { { 20.3f, 8.2f }, { 80.3f, 8.2f }, { 50.3f, 32.2f } } },
    { { { 50.26f, 2.4f }, { 54.2f, 37.4f }, { 46.3f, 37.4f } } },
    { { { 10, 10 }, { 20, 10 }, { 20, 20 }, { 10, 20 } },
      { { 30, 10 }, { 30, 20 }, { 40, 20 }, { 40, 10 } },
      { { 50, 10 }, { 60, 10 }, { 60, 20 }, { 50, 20 } },
      { { 60, 10 }, { 70, 10 }, { 70, 20 }, { 60, 20 } } },
  };

  for (int s = 0; s < shapes.size(); ++s) {
    Path path;
    for (const auto& polygon : shapes[s]) {
      path.moveTo(polygon[0]);
      for (int i = 1; i < polygon.size(); ++i)
        path.lineTo(polygon[i]);
      path.close();
    }

    Canvas canvas;
    canvas.setWindowless(kWidth, kHeight);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, kWidth, kHeight);
    canvas.setColor(0xffffffff);
    canvas.fill(path, 0, 0, kWidth, kHeight);

    const Screenshot& screenshot = canvas.takeScreenshot();
    float max_error = 0.0f;
    int worst_x = 0;
    int worst_y = 0;
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x) {
        float expected = std::min(1.0f, pixelCoverage(shapes[s], x, y));
        float error = std::abs(screenshot.sample(x, y).hexRed() / 255.0f - expected);
        if (error > max_error) {
          max_error = error;
          worst_x = x;
          worst_y = y;
        }
      }
    }

    INFO("shape " << s << ", worst pixel " << worst_x << ", " << worst_y << ", error " << max_error * 255.0f);
    REQUIRE(max_error < 0.02f);
  }
}

TEST_CASE("Path fill coverage stays exact next to nearly straight corners", "[graphics]") {
  static constexpr int kSize = 256;
  static constexpr int kStars[] = { 18, 57, 73, 96 };

  uint32_t state = 1;
  auto random = [&state]() {
    state = state * 1664525u + 1013904223u;
    return (state >> 8) / 16777216.0f;
  };

  for (int star = 0; star <= kStars[3]; ++star) {
    int num_points = 20 + static_cast<int>(random() * 180);
    float center_x = 128.0f + random();
    float center_y = 128.0f + random();
    Point center(center_x, center_y);
    std::vector<Point> polygon;
    for (int i = 0; i < num_points; ++i) {
      float angle = i * 2.0f * kPi / num_points;
      float radius = 20.0f + random() * 100.0f;
      polygon.push_back(center + Point(std::cos(angle), std::sin(angle)) * radius);
    }

    if (std::find(std::begin(kStars), std::end(kStars), star) == std::end(kStars))
      continue;

    std::vector<std::vector<Point>> polygons = { polygon };
    Path path;
    path.moveTo(polygon[0]);
    for (int i = 1; i < polygon.size(); ++i)
      path.lineTo(polygon[i]);
    path.close();

    Canvas canvas;
    canvas.setWindowless(kSize, kSize);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, kSize, kSize);
    canvas.setColor(0xffffffff);
    canvas.fill(path, 0, 0, kSize, kSize);

    const Screenshot& screenshot = canvas.takeScreenshot();
    float max_error = 0.0f;
    int worst_x = 0;
    int worst_y = 0;
    for (int y = 0; y < kSize; ++y) {
      for (int x = 0; x < kSize; ++x) {
        float expected = std::min(1.0f, pixelCoverage(polygons, x, y));
        float error = std::abs(screenshot.sample(x, y).hexRed() / 255.0f - expected);
        if (error > max_error) {
          max_error = error;
          worst_x = x;
          worst_y = y;
        }
      }
    }

    INFO("star " << star << ", worst pixel " << worst_x << ", " << worst_y << ", error "
                 << max_error * 255.0f);
    REQUIRE(max_error < 0.02f);
  }
}

TEST_CASE("Path fill coverage stays exact with many overlapping even-odd contours", "[graphics]") {
  static constexpr int kSize = 120;
  static constexpr int kContours = 8;
  static constexpr int kSides = 600;

  std::vector<std::vector<Point>> polygons;
  Path path;
  for (int c = 0; c < kContours; ++c) {
    Point center(60.0f + 16.5f * std::cos(c * 2.4f), 60.0f + 16.5f * std::sin(c * 2.4f));
    float radius = 22.5f + 1.875f * c;
    std::vector<Point> polygon;
    for (int i = 0; i < kSides; ++i) {
      float angle = i * 2.0f * kPi / kSides;
      polygon.push_back(center + Point(std::cos(angle), std::sin(angle)) * radius);
    }
    path.moveTo(polygon[0]);
    for (int i = 1; i < polygon.size(); ++i)
      path.lineTo(polygon[i]);
    path.close();
    polygons.push_back(polygon);
  }
  path.setFillRule(Path::FillRule::EvenOdd);

  Canvas canvas;
  canvas.setWindowless(kSize, kSize);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kSize, kSize);
  canvas.setColor(0xffffffff);
  canvas.fill(path, 0, 0, kSize, kSize);

  const Screenshot& screenshot = canvas.takeScreenshot();
  float max_error = 0.0f;
  int worst_x = 0;
  int worst_y = 0;
  for (int y = 0; y < kSize; ++y) {
    for (int x = 0; x < kSize; ++x) {
      float winding = pixelCoverage(polygons, x, y);
      float expected = 1.0f - std::abs(std::fmod(winding, 2.0f) - 1.0f);
      float error = std::abs(screenshot.sample(x, y).hexRed() / 255.0f - expected);
      if (error > max_error) {
        max_error = error;
        worst_x = x;
        worst_y = y;
      }
    }
  }

  INFO("worst pixel " << worst_x << ", " << worst_y << ", error " << max_error * 255.0f);
  REQUIRE(max_error < 0.02f);
}

TEST_CASE("Stroke joins at curve ends use the curve tangent", "[graphics]") {
  Path arc;
  arc.moveTo(0, 0);
  arc.lineTo(10, 0);
  arc.arcTo(10, 10, 0.0f, false, true, Point(0, 10));
  Bounds arc_bounds = arc.stroke(4.0f, Path::Join::Miter, Path::EndCap::Butt).boundingBox();
  REQUIRE(arc_bounds.right() == Catch::Approx(12.0f).margin(0.05f));
  REQUIRE(arc_bounds.y() == Catch::Approx(-2.0f).margin(0.05f));

  Path cubic;
  cubic.moveTo(0, 0);
  cubic.lineTo(10, 0);
  cubic.bezierTo(10, 5, 5, 10, 0, 10);
  Bounds cubic_bounds = cubic.stroke(4.0f, Path::Join::Miter, Path::EndCap::Butt).boundingBox();
  REQUIRE(cubic_bounds.right() == Catch::Approx(12.0f).margin(0.05f));
  REQUIRE(cubic_bounds.y() == Catch::Approx(-2.0f).margin(0.05f));

  for (float scale : { 1.0f, 1.5f, 2.0f, 4.0f }) {
    INFO("resolution scale " << scale);
    Path flat;
    flat.setResolutionMatrix(Matrix::scale(scale, scale));
    flat.moveTo(0, 0);
    flat.bezierTo(0.1f, 0, 100, 100, 100.1f, 100);
    Path line;
    line.setResolutionMatrix(Matrix::scale(scale, scale));
    line.moveTo(0, 0);
    line.lineTo(100.1f, 100);
    Bounds flat_bounds = flat.stroke(20.0f, Path::Join::Miter, Path::EndCap::Butt).boundingBox();
    Bounds line_bounds = line.stroke(20.0f, Path::Join::Miter, Path::EndCap::Butt).boundingBox();
    REQUIRE(flat_bounds.x() == Catch::Approx(line_bounds.x()).margin(0.05f));
    REQUIRE(flat_bounds.y() == Catch::Approx(line_bounds.y()).margin(0.05f));
    REQUIRE(flat_bounds.right() == Catch::Approx(line_bounds.right()).margin(0.05f));
    REQUIRE(flat_bounds.bottom() == Catch::Approx(line_bounds.bottom()).margin(0.05f));
  }
}

TEST_CASE("Curve end tangents survive reversing transforming and dashing", "[graphics]") {
  Path arc;
  arc.moveTo(0, 0);
  arc.lineTo(10, 0);
  arc.arcTo(10, 10, 0.0f, false, true, Point(0, 10));
  auto stroke_bounds = [](const Path& path) {
    return path.stroke(4.0f, Path::Join::Miter, Path::EndCap::Butt).boundingBox();
  };

  Bounds reversed = stroke_bounds(arc.reversed());
  REQUIRE(reversed.right() == Catch::Approx(12.0f).margin(0.05f));
  REQUIRE(reversed.y() == Catch::Approx(-2.0f).margin(0.05f));

  Bounds scaled = stroke_bounds(arc.scaled(2.0f));
  REQUIRE(scaled.right() == Catch::Approx(22.0f).margin(0.05f));
  REQUIRE(scaled.y() == Catch::Approx(-2.0f).margin(0.05f));

  Bounds rotated = stroke_bounds(arc.transformed(Transform::rotation(90.0f)));
  REQUIRE(rotated.right() == Catch::Approx(2.0f).margin(0.05f));
  REQUIRE(rotated.bottom() == Catch::Approx(12.0f).margin(0.05f));

  Path dashed = arc.stroke(4.0f, Path::Join::Miter, Path::EndCap::Butt, { 1000.0f, 1.0f });
  REQUIRE(dashed.boundingBox().right() == Catch::Approx(12.0f).margin(0.05f));
  REQUIRE(dashed.boundingBox().y() == Catch::Approx(-2.0f).margin(0.05f));

  Path closed = arc;
  closed.close();
  Path joined = closed.stroke(4.0f, Path::Join::Miter, Path::EndCap::Butt, { 12.0f, 1.0f });
  REQUIRE(joined.boundingBox().right() == Catch::Approx(12.0f).margin(0.05f));
  REQUIRE(joined.boundingBox().y() == Catch::Approx(-2.0f).margin(0.05f));
}

TEST_CASE("Dashes ending inside curves are cut along the curve normal", "[graphics]") {
  auto check_caps = [](const Path& circle) {
    Path dashed = circle.stroke(10.0f, Path::Join::Miter, Path::EndCap::Butt, { 20.0f, 7.0f });
    int caps = 0;
    for (const auto& sub_path : dashed.subPaths()) {
      for (int i = 0; i < sub_path.points.size(); ++i) {
        Point from = sub_path.points[i];
        Point to = sub_path.points[(i + 1) % sub_path.points.size()];
        if (std::abs((to - from).length() - 10.0f) > 0.1f)
          continue;

        float sine = (to - from).normalized().cross(((from + to) * 0.5f).normalized());
        if (std::abs(sine) > 0.5f)
          continue;

        caps++;
        REQUIRE(sine == Catch::Approx(0.0f).margin(0.002f));
      }
    }
    REQUIRE(caps == 14);
  };

  Path circle;
  circle.addCircle(0, 0, 30);
  check_caps(circle);

  static constexpr float k = 0.5522847f * 30.0f;
  Path cubic_circle;
  cubic_circle.moveTo(30, 0);
  cubic_circle.bezierTo(30, k, k, 30, 0, 30);
  cubic_circle.bezierTo(-k, 30, -30, k, -30, 0);
  cubic_circle.bezierTo(-30, -k, -k, -30, 0, -30);
  cubic_circle.bezierTo(k, -30, 30, -k, 30, 0);
  cubic_circle.close();
  check_caps(cubic_circle);
}

TEST_CASE("Dash ends at line and curve junctions keep the incoming direction", "[graphics]") {
  static constexpr float kWidth = 4.0f;
  auto end_cap = [](const Path& path, Point end) {
    Path dashed = path.stroke(kWidth, Path::Join::Miter, Path::EndCap::Butt, { 10.0f, 100.0f });
    for (const auto& sub_path : dashed.subPaths()) {
      for (int i = 0; i < sub_path.points.size(); ++i) {
        Point from = sub_path.points[i];
        Point to = sub_path.points[(i + 1) % sub_path.points.size()];
        if (std::abs((to - from).length() - kWidth) < 0.01f && ((from + to) * 0.5f - end).length() < 0.01f)
          return (to - from).normalized();
      }
    }
    return Point();
  };
  auto rotate = [](Point direction, float degrees) {
    float radians = degrees * kPi / 180.0f;
    return Point(direction.x * std::cos(radians) - direction.y * std::sin(radians),
                 direction.x * std::sin(radians) + direction.y * std::cos(radians));
  };

  Point corner(10, 0);
  Point curve_end = Point(1, -1).normalized();
  for (float degrees : { 30.0f, 60.0f, 90.0f, 120.0f, 150.0f }) {
    INFO("turn " << degrees);
    Point turn = rotate(Point(1, 0), degrees);
    Point side = rotate(turn, 90.0f);
    Path line_to_curve;
    line_to_curve.moveTo(0, 0);
    line_to_curve.lineTo(corner);
    line_to_curve.bezierTo(corner + turn * 5.0f, corner + turn * 10.0f + side * 5.0f,
                           corner + turn * 15.0f + side * 5.0f);
    Point cap = end_cap(line_to_curve, corner);
    REQUIRE(cap.length() > 0.5f);
    REQUIRE(cap.dot(Point(1, 0)) == Catch::Approx(0.0f).margin(0.002f));

    Path curve_to_line;
    curve_to_line.setResolutionMatrix(Matrix::scale(0.01f, 0.01f));
    curve_to_line.moveTo(0, 0);
    curve_to_line.bezierTo(3, 3, 7, 3, 10, 0);
    curve_to_line.lineTo(corner + rotate(curve_end, degrees) * 10.0f);
    cap = end_cap(curve_to_line, corner);
    REQUIRE(cap.length() > 0.5f);
    REQUIRE(cap.dot(curve_end) == Catch::Approx(0.0f).margin(0.002f));
  }
}
