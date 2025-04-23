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
  PathTriangle(Point a, Point b, Point c) : points { a, b, c } { }

  bool operator<(const PathTriangle& other) const { return points < other.points; }
  bool operator==(const PathTriangle& other) const {
    static constexpr float kEpsilon = 1e-5f;
    return (points.size() == other.points.size()) &&
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
  std::set<PathTriangle> PathTriangles;
  for (size_t i = 0; i < triangulation.triangles.size(); i += 3) {
    Point a = triangulation.points[triangulation.triangles[i]];
    Point b = triangulation.points[triangulation.triangles[i + 1]];
    Point c = triangulation.points[triangulation.triangles[i + 2]];
    PathTriangles.insert(PathTriangle(a, b, c));
  }
  return PathTriangles;
}

bool matchPathTriangles(const Path& path, const std::set<PathTriangle>& expected) {
  std::set<PathTriangle> PathTriangles = createTriangles(path.triangulate());
  return PathTriangles == expected;
}

TEST_CASE("Path triangulate nothing", "[graphics]") {
  Path path0;
  Path::Triangulation triangulation = path0.triangulate();
  REQUIRE(triangulation.triangles.size() == 0);

  Path path1;
  path1.moveTo(0, 0);
  triangulation = path1.triangulate();
  REQUIRE(triangulation.triangles.size() == 0);

  Path path2;
  path2.moveTo(0, 0);
  path2.lineTo(1, 0);
  triangulation = path2.triangulate();
  REQUIRE(triangulation.triangles.size() == 0);

  Path path3;
  path3.moveTo(0, 0);
  path3.lineTo(0, 1);
  triangulation = path3.triangulate();
  REQUIRE(triangulation.triangles.size() == 0);
}

TEST_CASE("Path triangulate single", "[graphics]") {
  Path path0;
  path0.moveTo(0, 0);
  path0.lineTo(0, 1);
  path0.lineTo(1, 1);
  path0.reverse();
  std::set<PathTriangle> expected = { PathTriangle(Point(0, 0), Point(0, 1), Point(1, 1)) };
  REQUIRE(matchPathTriangles(path0, expected));
}

TEST_CASE("Path triangulate intersection", "[graphics]") {
  Path path0;
  path0.moveTo(0, 0);
  path0.lineTo(0, 1);
  path0.lineTo(1, 0);
  path0.lineTo(1, 1);
  std::set<PathTriangle> expected = { PathTriangle(Point(0, 0), Point(0, 1), Point(0.5f, 0.5f)),
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
  path.triangulate();
}

TEST_CASE("Path triangulate multiple intersection", "[graphics]") {
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr int kStarPoints = 5;

  Path star;

  float radius = 100.0f;
  float phase = randomFloat(0.0f, 1.0f);
  std::complex<float> position(cos(-2.0f * kPi * phase / kStarPoints),
                               sin(-2.0f * kPi * phase / kStarPoints));

  star.moveTo(position.real() * radius, position.imag() * radius);
  std::complex<float> delta(cos(-2.0f * kPi * 2.0f / kStarPoints), sin(-2.0f * kPi * 2.0f / kStarPoints));

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

TEST_CASE("Random path triangulation", "[graphics]") {
  // { x = 6752.79492 y = 567.126526 }
  // { x = 8581.74023 y = 7906.08105 }
  // { x = 1424.30957 y = 9965.66309 }
  // { x = 4595.61963 y = 14.3991947 }
  // { x = 5547.47363 y = 5059.93115 }
  // { x = 9685.39941 y = 3157.83105 }
  // { x = 6433.23340 y = 4173.01416 }
  // { x = 7474.98877 y = 5467.82324 }
  // { x = 9212.18262 y = 8411.53125 }
  // { x = 7426.63721 y = 2736.23535 }
  // { x = 9080.00098 y = 5840.46289 }
  // { x = 2113.48511 y = 3726.24365 }
  // { x = 9228.61230 y = 4507.67822 }
  // { x = 9178.29004 y = 6089.29102 }
  // { x = 1792.58777 y = 6839.50439 }
  // { x = 3473.21777 y = 8707.61426 }
  // { x = 8544.15234 y = 8513.67480 }
  // { x = 5927.23096 y = 9246.49121 }
  // { x = 9264.05273 y = 2850.45825 }
  // { x = 7787.21338 y = 8386.79883 }
  static constexpr float kWidth = 10000.0f;
  static constexpr float kHeight = 10000.0f;
  static constexpr int kNumPoints = 5;
  static constexpr int kNumPaths = 1000;

  for (int p = 0; p < kNumPaths; ++p) {
    Path path;
    path.moveTo(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));

    for (int i = 1; i < kNumPoints; ++i)
      path.lineTo(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));

    path.triangulate();
  }
}

TEST_CASE("Random robust degeneracy triangulation", "[graphics]") {
  // static constexpr float kWidth = 10000.0f;
  // static constexpr float kHeight = 10000.0f;
  // static constexpr int kNumPoints = 20;
  // static constexpr int kNumPaths = 50;
  //
  // for (int p = 0; p < kNumPaths; ++p) {
  //   Path path;
  //   Point point1(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));
  //   Point point2(randomFloat(0.0f, kWidth), randomFloat(0.0f, kHeight));
  //   path.moveTo(point1);
  //
  //   for (int i = 1; i < kNumPoints; ++i) {
  //     float t = randomFloat(0.0f, kWidth);
  //     Point point = point1 + (point2 - point1) * t;
  //     path.lineTo(point.x, point.y);
  //   }
  //
  //   path.triangulate();
  // }
}

TEST_CASE("Test path filling integration", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(1000, 1000);

  canvas.submit();
}