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

std::set<PathTriangle> createTriangles(const Path::Triangulation& triangulation) {
  std::set<PathTriangle> triangles;
  for (size_t i = 0; i < triangulation.triangles.size(); i += 3) {
    Point a = triangulation.points[triangulation.triangles[i]];
    Point b = triangulation.points[triangulation.triangles[i + 1]];
    Point c = triangulation.points[triangulation.triangles[i + 2]];
    triangles.insert(PathTriangle(a, b, c));
  }
  return triangles;
}

bool matchPathTriangles(Path& path, const std::set<PathTriangle>& expected) {
  std::set<PathTriangle> triangles = createTriangles(path.triangulate());
  return triangles == expected;
}

TEST_CASE("Path triangulate nothing", "[graphics]") {
  Path path0;
  Path::Triangulation triangulation = path0.triangulate();
  REQUIRE(triangulation.triangles.empty());

  Path path1;
  path1.moveTo(0, 0);
  triangulation = path1.triangulate();
  REQUIRE(triangulation.triangles.empty());

  Path path2;
  path2.moveTo(0, 0);
  path2.lineTo(1, 0);
  triangulation = path2.triangulate();
  REQUIRE(triangulation.triangles.empty());

  Path path3;
  path3.moveTo(0, 0);
  path3.lineTo(0, 1);
  triangulation = path3.triangulate();
  REQUIRE(triangulation.triangles.empty());
}

TEST_CASE("Path triangulate single", "[graphics]") {
  Path path0;
  path0.moveTo(0, 0);
  path0.lineTo(0, 1);
  path0.lineTo(1, 1);
  path0.reverse();
  std::set expected = { PathTriangle(Point(0, 0), Point(0, 1), Point(1, 1)) };
  REQUIRE(matchPathTriangles(path0, expected));
}

TEST_CASE("Path triangulate intersection", "[graphics]") {
  Path path0;
  path0.moveTo(0, 0);
  path0.lineTo(0, 1);
  path0.lineTo(1, 0);
  path0.lineTo(1, 1);
  std::set expected = { PathTriangle(Point(0, 0), Point(0, 1), Point(0.5f, 0.5f)),
                        PathTriangle(Point(1, 0), Point(1, 1), Point(0.5f, 0.5f)) };
  REQUIRE(matchPathTriangles(path0, expected));

  Path path1;
  path1.moveTo(0, 0);
  path1.lineTo(1, 0);
  path1.lineTo(0, 1);
  path1.lineTo(1, 1);
  expected = { PathTriangle(Point(0, 0), Point(1, 0), Point(0.5f, 0.5f)),
               PathTriangle(Point(0, 1), Point(1, 1), Point(0.5f, 0.5f)) };
  REQUIRE(matchPathTriangles(path1, expected));
}

TEST_CASE("Colinear test", "[graphics]") {
  Path path;
  path.moveTo(0, 0);
  path.lineTo(1, 0);
  path.lineTo(2, 0);
  path.lineTo(3, 0);
  path.lineTo(3, 1);
  path.lineTo(3, 2);
  path.lineTo(3, 3);
  path.lineTo(2, 3);
  path.lineTo(1, 3);
  path.lineTo(0, 3);
  path.lineTo(0, 2);
  path.lineTo(0, 1);
  auto result = path.triangulate();
  REQUIRE(!result.points.empty());
}

TEST_CASE("Path triangulate multiple intersection", "[graphics]") {
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kStarPoints = 5;

  Path star;

  float radius = 100.0f;
  float phase = randomFloat(0.0f, 1.0f);
  std::complex<float> position(std::cos(-2.0f * kPi * phase / kStarPoints),
                               std::sin(-2.0f * kPi * phase / kStarPoints));

  star.moveTo(position.real() * radius, position.imag() * radius);
  std::complex<float> delta(std::cos(-2.0f * kPi * 2.0f / kStarPoints),
                            std::sin(-2.0f * kPi * 2.0f / kStarPoints));

  for (int i = 1; i < kStarPoints; ++i) {
    position = position * delta;
    star.lineTo(radius * position.real(), radius * position.imag());
  }

  std::vector<Point> intersections;
  for (int i = 0; i < kStarPoints; ++i) {
    Point start = star.subPaths()[0].points[i];
    Point end = star.subPaths()[0].points[(i + 1) % kStarPoints];
    Point start1 = star.subPaths()[0].points[(i + 2) % kStarPoints];
    Point end1 = star.subPaths()[0].points[(i + 3) % kStarPoints];

    auto intersection = Path::findIntersection(start, end, start1, end1);
    REQUIRE(intersection.has_value());
    intersections.push_back(intersection.value());
  }

  std::set<PathTriangle> expected;
  for (int i = 0; i < kStarPoints; ++i)
    expected.insert(PathTriangle(star.subPaths()[0].points[i], intersections[i],
                                 intersections[(i + 2) % kStarPoints]));

  REQUIRE(matchPathTriangles(star, expected));
}

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

  SECTION("Degeneracy point star") {
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
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffff0000);
    canvas.fill(star, 0, 0, kWidth, kWidth);
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

  SECTION("Vertical cross line degeneracy") {
    Path path;
    path.moveTo(10, 10);
    path.lineTo(40, 10);
    path.lineTo(40, 30);
    path.lineTo(50, 25);
    path.lineTo(40, 20);
    path.lineTo(40, 40);
    path.lineTo(10, 40);

    Canvas canvases[2];
    for (auto& canvas : canvases) {
      canvas.setWindowless(50, 40);
      canvas.setColor(0xff000000);
      canvas.fill(0, 0, canvas.width(), canvas.height());
      canvas.setColor(0xffff0000);
    }

    canvases[0].fill(path, 0, 0, canvases[0].width(), canvases[0].height());
    canvases[0].submit();
    path.reverse();
    canvases[1].fill(path, 0, 0, canvases[1].width(), canvases[1].height());
    canvases[1].submit();
    Screenshot screenshots[] = { canvases[0].takeScreenshot(), canvases[1].takeScreenshot() };

    for (const auto& screenshot : screenshots) {
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

  SECTION("Found degeneracy tests") {
    std::vector<std::vector<DPoint>> paths = {
      { { 0, 2 }, { 0, 4 }, { 4, 2 }, { 3, 4 }, { 3, 2 }, { 4, 3 }, { 2, 0 }, { 4, 0 }, { 2, 4 }, { 0, 4 }, { 2, 3 }, { 1, 4 } },
      { { 2, 3 }, { 2, 1 }, { 1, 4 }, { 3, 2 }, { 4, 0 }, { 1, 4 }, { 4, 1 }, { 1, 2 }, { 2, 4 }, { 0, 4 } },
      { { 2, 1 }, { 0, 3 }, { 4, 4 }, { 1, 3 }, { 3, 2 }, { 2, 0 }, { 0, 1 }, { 4, 1 }, { 3, 3 }, { 3, 0 }, { 3, 2 }, { 2, 4 } },
      { { 4, 3 }, { 3, 0 }, { 2, 2 }, { 3, 1 }, { 3, 2 }, { 0, 2 }, { 4, 3 }, { 4, 4 }, { 2, 2 } },
      { { 1, 2 }, { 4, 2 }, { 3, 4 }, { 4, 0 }, { 4, 4 }, { 3, 3 }, { 4, 1 } },
      { { 0, 4 }, { 4, 0 }, { 0, 3 }, { 1, 4 }, { 3, 1 }, { 1, 3 }, { 4, 4 } },
      { { 4, 1 }, { 4, 4 }, { 3, 3 }, { 0, 1 }, { 0, 2 }, { 4, 3 }, { 1, 4 } },
      { { 1, 2 }, { 2, 2 }, { 0, 1 }, { 4, 1 }, { 4, 0 }, { 0, 4 }, { 4, 1 } },
      { { 0, 2 }, { 4, 4 }, { 4, 0 }, { 1, 2 }, { 4, 4 }, { 4, 2 }, { 0, 3 } },
      { { 3, 4 }, { 3, 1 }, { 1, 2 }, { 3, 0 }, { 3, 2 }, { 4, 2 }, { 2, 0 } },
      { { 2, 2 }, { 4, 4 }, { 0, 4 }, { 1, 0 }, { 0, 0 }, { 0, 4 }, { 3, 4 } },
      { { 4, 1 }, { 2, 2 }, { 2, 1 }, { 0, 3 }, { 4, 1 }, { 1, 3 }, { 0, 1 } },
      { { 1, 0 }, { 3, 2 }, { 2, 0 }, { 4, 3 }, { 2, 1 }, { 4, 0 }, { 4, 1 } },
      { { 1, 3 }, { 0, 3 }, { 3, 2 }, { 2, 0 }, { 0, 3 }, { 0, 3 }, { 2, 2 } },
      { { 1, 1 }, { 1, 0 }, { 3, 4 }, { 4, 3 }, { 1, 0 }, { 1, 2 }, { 2, 0 } },
      { { 0, 3 }, { 2, 2 }, { 3, 0 }, { 1, 3 }, { 0, 3 }, { 4, 1 } },
      { { 4, 4 }, { 2, 0 }, { 2, 4 }, { 3, 4 }, { 4, 4 }, { 2, 0 }, { 0, 4 } },
      { { 0, 0 }, { 3, 4 }, { 4, 3 }, { 0, 3 }, { 4, 4 }, { 3, 3 }, { 4, 3 } },
      { { 2, 4 }, { 2, 2 }, { 1, 3 }, { 1, 2 }, { 1, 2 }, { 0, 4 }, { 3, 1 } },
      { { 1, 2 }, { 1, 1 }, { 1, 1 }, { 3, 0 }, { 1, 1 }, { 0, 4 }, { 4, 1 } },
      { { 3, 4 }, { 4, 2 }, { 3, 1 }, { 0, 3 }, { 4, 2 }, { 3, 1 } },
      { { 2, 0 }, { 3, 2 }, { 4, 2 }, { 1, 1 }, { 2, 4 }, { 3, 1 } },
      { { 1, 4 }, { 4, 4 }, { 2, 2 }, { 2, 3 }, { 1, 0 }, { 3, 0 } },
      { { 1, 0 }, { 3, 4 }, { 0, 1 }, { 4, 4 }, { 2, 3 }, { 4, 2 } },
      { { 4, 4 }, { 4, 0 }, { 0, 2 }, { 1, 0 }, { 3, 2 }, { 0, 2 }, { 2, 1 } },
      { { 4, 4 }, { 2, 0 }, { 4, 1 }, { 0, 1 }, { 0, 3 }, { 4, 0 }, { 3, 1 } },
      { { 0, 3 }, { 3, 1 }, { 0, 2 }, { 4, 4 }, { 1, 4 }, { 1, 1 }, { 2, 2 } },
      { { 2, 2 }, { 0, 4 }, { 0, 2 }, { 1, 4 }, { 1, 0 }, { 2, 4 }, { 0, 2 } },
      { { 1, 4 }, { 2, 3 }, { 4, 2 }, { 1, 1 }, { 2, 1 }, { 4, 4 }, { 3, 0 } },
      { { 4, 0 }, { 0, 2 }, { 3, 1 }, { 3, 3 }, { 4, 4 }, { 0, 3 }, { 0, 4 } },
      { { 4, 4 }, { 1, 1 }, { 2, 3 }, { 2, 2 }, { 3, 0 }, { 4, 0 }, { 1, 3 } },
      { { 2, 1 }, { 3, 0 }, { 1, 4 }, { 2, 3 }, { 2, 1 }, { 4, 2 }, { 2, 2 } },
      { { 3, 3 }, { 2, 2 }, { 4, 3 }, { 5, -2 }, { 0, 4 }, { 4, 2 }, { 3, 1 } },
    };

    for (const auto& path_points : paths) {
      Path path;
      for (const auto& point : path_points)
        path.lineTo(point.x, point.y);

      auto result = path.triangulate();
      REQUIRE(!result.points.empty());
    }
  }
}

TEST_CASE("Random path triangulation", "[graphics]") {
  static constexpr float kWidth = 1000.0f;
  static constexpr float kHeight = 1000.0f;
  static constexpr int kNumPoints = 20;
  static constexpr int kNumPaths = 50;

  for (int p = 0; p < kNumPaths; ++p) {
    Path path;
    path.moveTo(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));

    for (int i = 1; i < kNumPoints; ++i)
      path.lineTo(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));

    auto result = path.triangulate();
    REQUIRE(!result.points.empty());
  }
}

TEST_CASE("Random path triangulation with integer position degeneracies", "[graphics]") {
  static constexpr float kWidth = 5.0f;
  static constexpr float kHeight = 5.0f;
  static constexpr int kNumPoints = 40;
  static constexpr int kNumPaths = 50;

  for (int p = 0; p < kNumPaths; ++p) {
    Path path;
    int x = static_cast<int>(randomFloat(0.0f, kWidth));
    int y = static_cast<int>(randomFloat(0.0f, kHeight));
    path.moveTo(x, y);

    for (int i = 1; i < kNumPoints; ++i)
      path.lineTo(static_cast<int>(randomFloat(0, kWidth)), static_cast<int>(randomFloat(0, kHeight)));

    auto result = path.triangulate();
    REQUIRE(!result.points.empty());
  }
}

TEST_CASE("Random line degeneracy", "[graphics]") {
  static constexpr float kWidth = 1000.0f;
  static constexpr float kHeight = 1000.0f;
  static constexpr int kNumPoints = 20;
  static constexpr int kNumPaths = 50;

  for (int p = 0; p < kNumPaths; ++p) {
    Path path;
    Point point1(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));
    Point point2(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));
    path.moveTo(point1);

    for (int i = 1; i < kNumPoints; ++i) {
      float t = randomFloat(0.0f, kWidth);
      if (randomFloat(0.0f, 1.0f) < 0.5f) {
        Point point = point1 + (point2 - point1) * t;
        path.lineTo(point.x, point.y);
      }
      else
        path.lineTo(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));
    }

    auto result = path.triangulate();
    REQUIRE(!result.points.empty());
  }
}

TEST_CASE("Random point degeneracy", "[graphics]") {
  static constexpr float kWidth = 800.0f;
  static constexpr float kHeight = 800.0f;
  static constexpr int kNumPoints = 20;
  static constexpr int kNumPaths = 50;

  for (int p = 0; p < kNumPaths; ++p) {
    Path path;
    path.moveTo(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));

    for (int i = 1; i < kNumPoints; ++i) {
      if (randomFloat(0.0f, 1.0f) < 0.3f)
        path.lineTo(kWidth * 0.5f, kHeight * 0.5f);
      else
        path.lineTo(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));
    }

    auto result = path.triangulate();
    REQUIRE(!result.points.empty());
  }
}

TEST_CASE("Path difference", "[graphics]") {
  SECTION("Exact") {
    Path path1;
    path1.moveTo(1, 1);
    path1.lineTo(4, 1);
    path1.lineTo(4, 4);
    path1.lineTo(1, 4);
    path1.close();

    Path path2;
    path2.moveTo(1, 1);
    path2.lineTo(4, 1);
    path2.lineTo(4, 4);
    path2.lineTo(1, 4);
    path2.close();

    Path difference = path1.combine(path2, Path::Operation::Difference);

    Canvas canvas;
    canvas.setWindowless(5, 5);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffffffff);
    canvas.fill(difference, 0, 0, 5, 5);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();
    for (int r = 0; r < 5; ++r) {
      for (int c = 0; c < 5; ++c)
        REQUIRE(screenshot.sample(c, r).hexRed() == 0x0);
    }
  }

  SECTION("Embedded") {
    Path path1;
    path1.moveTo(1, 1);
    path1.lineTo(4, 1);
    path1.lineTo(4, 4);
    path1.lineTo(1, 4);
    path1.close();

    Path path2;
    path2.moveTo(0, 0);
    path2.lineTo(5, 0);
    path2.lineTo(5, 5);
    path2.lineTo(0, 5);
    path2.close();

    Path difference = path1.combine(path2, Path::Operation::Difference);

    Canvas canvas;
    canvas.setWindowless(5, 5);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffffffff);
    canvas.fill(difference, 0, 0, 5, 5);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();
    for (int r = 0; r < 5; ++r) {
      for (int c = 0; c < 5; ++c)
        REQUIRE(screenshot.sample(c, r).hexRed() == 0x0);
    }
  }
}

TEST_CASE("Path interesection", "[graphics]") {
  SECTION("Bug 1") {
    Path path1;
    path1.moveTo(0.00000000, 0);
    path1.lineTo(-3.33066907e-6, 10);
    path1.lineTo(10, 10);
    path1.lineTo(10, 0);
    path1.close();

    Path path2;
    path2.addRectangle(0, 0, 100, 100);
    Path intersection = path1.combine(path2, Path::Operation::Intersection);

    Canvas canvas;
    canvas.setWindowless(10, 10);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.setColor(0xffffffff);
    canvas.fill(intersection, 0, 0, 10, 10);
    canvas.submit();
    const auto& screenshot = canvas.takeScreenshot();
    for (int r = 0; r < 10; ++r) {
      for (int c = 0; c < 10; ++c)
        REQUIRE(screenshot.sample(c, r).hexRed() == 0xff);
    }
  }
}
