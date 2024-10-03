/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
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
