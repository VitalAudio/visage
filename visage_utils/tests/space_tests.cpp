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

#include "visage_utils/space.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("Bounds intersection", "[utils]") {
  Bounds bounds1(-1, -2, 10, 10);
  Bounds bounds2(7, 5, 15, 15);
  Bounds intersection = bounds1.intersection(bounds2);
  REQUIRE(intersection.x() == 7);
  REQUIRE(intersection.y() == 5);
  REQUIRE(intersection.width() == 2);
  REQUIRE(intersection.height() == 3);
}

TEST_CASE("Bounds subtract failed corner intersect", "[utils]") {
  Bounds bounds1(1, 10, 10, 10);
  Bounds bounds2(5, 15, 15, 15);
  Bounds bounds3(0, 15, 5, 15);
  Bounds result;

  REQUIRE(!bounds1.subtract(bounds2, result));
  REQUIRE(!bounds2.subtract(bounds1, result));
  REQUIRE(!bounds1.subtract(bounds3, result));
  REQUIRE(!bounds3.subtract(bounds1, result));
}

TEST_CASE("Bounds subtract containment", "[utils]") {
  Bounds bounds1(1, 10, 10, 10);
  Bounds bounds2(5, 15, 6, 5);
  Bounds bounds3(5, 15, 3, 2);
  Bounds bounds4(1, 10, 10, 7);
  Bounds bounds5(1, 13, 10, 7);
  Bounds bounds6(1, 10, 9, 10);
  Bounds bounds7(2, 10, 9, 10);
  Bounds result;

  REQUIRE(bounds1.subtract(bounds1, result));
  REQUIRE(result.width() == 0);
  REQUIRE(result.height() == 0);

  REQUIRE(!bounds1.subtract(bounds2, result));
  REQUIRE(!bounds1.subtract(bounds3, result));

  REQUIRE(bounds2.subtract(bounds1, result));
  REQUIRE(result.width() == 0);
  REQUIRE(result.height() == 0);

  REQUIRE(bounds3.subtract(bounds2, result));
  REQUIRE(result.width() == 0);
  REQUIRE(result.height() == 0);

  REQUIRE(bounds1.subtract(bounds4, result));
  REQUIRE(result.x() == 1);
  REQUIRE(result.y() == 17);
  REQUIRE(result.width() == 10);
  REQUIRE(result.height() == 3);

  REQUIRE(bounds1.subtract(bounds5, result));
  REQUIRE(result.x() == 1);
  REQUIRE(result.y() == 10);
  REQUIRE(result.width() == 10);
  REQUIRE(result.height() == 3);

  REQUIRE(bounds1.subtract(bounds6, result));
  REQUIRE(result.x() == 10);
  REQUIRE(result.y() == 10);
  REQUIRE(result.width() == 1);
  REQUIRE(result.height() == 10);

  REQUIRE(bounds1.subtract(bounds7, result));
  REQUIRE(result.x() == 1);
  REQUIRE(result.y() == 10);
  REQUIRE(result.width() == 1);
  REQUIRE(result.height() == 10);
}

TEST_CASE("Bounds subtract failed criss cross", "[utils]") {
  Bounds bounds1(1, 2, 5, 10);
  Bounds bounds2(0, 3, 10, 5);
  Bounds result;

  REQUIRE(!bounds1.subtract(bounds2, result));
  REQUIRE(!bounds2.subtract(bounds1, result));
}

TEST_CASE("Bounds subtract side overlap", "[utils]") {
  Bounds bounds1(1, 2, 5, 10);
  Bounds bounds2(0, 3, 5, 5);
  Bounds bounds3(0, 2, 15, 5);
  Bounds result;

  REQUIRE(!bounds1.subtract(bounds2, result));
  REQUIRE(bounds2.subtract(bounds1, result));
  REQUIRE(result.x() == 0);
  REQUIRE(result.y() == 3);
  REQUIRE(result.width() == 1);
  REQUIRE(result.height() == 5);

  REQUIRE(!bounds3.subtract(bounds1, result));
  REQUIRE(bounds1.subtract(bounds3, result));
  REQUIRE(result.x() == 1);
  REQUIRE(result.y() == 7);
  REQUIRE(result.width() == 5);
  REQUIRE(result.height() == 5);
}

TEST_CASE("Breaking rectangles", "[utils]") {
  Bounds bounds1(0, 0, 2312, 1161);
  Bounds bounds2(0, 1154, 1126, 156);
  std::vector<Bounds> pieces;
  Bounds::breakIntoNonOverlapping(bounds1, bounds2, pieces);
  REQUIRE(!bounds1.overlaps(bounds2));
  REQUIRE(pieces.size() == 0);
}
