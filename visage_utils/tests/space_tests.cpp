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
  IBounds bounds1(-1, -2, 10, 10);
  IBounds bounds2(7, 5, 15, 15);
  IBounds intersection = bounds1.intersection(bounds2);
  REQUIRE(intersection.x() == 7);
  REQUIRE(intersection.y() == 5);
  REQUIRE(intersection.width() == 2);
  REQUIRE(intersection.height() == 3);
}

TEST_CASE("Bounds subtract failed corner intersect", "[utils]") {
  IBounds bounds1(1, 10, 10, 10);
  IBounds bounds2(5, 15, 15, 15);
  IBounds bounds3(0, 15, 5, 15);
  IBounds result;

  REQUIRE(!bounds1.subtract(bounds2, result));
  REQUIRE(!bounds2.subtract(bounds1, result));
  REQUIRE(!bounds1.subtract(bounds3, result));
  REQUIRE(!bounds3.subtract(bounds1, result));
}

TEST_CASE("Bounds subtract containment", "[utils]") {
  IBounds bounds1(1, 10, 10, 10);
  IBounds bounds2(5, 15, 6, 5);
  IBounds bounds3(5, 15, 3, 2);
  IBounds bounds4(1, 10, 10, 7);
  IBounds bounds5(1, 13, 10, 7);
  IBounds bounds6(1, 10, 9, 10);
  IBounds bounds7(2, 10, 9, 10);
  IBounds result;

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
  IBounds bounds1(1, 2, 5, 10);
  IBounds bounds2(0, 3, 10, 5);
  IBounds result;

  REQUIRE(!bounds1.subtract(bounds2, result));
  REQUIRE(!bounds2.subtract(bounds1, result));
}

TEST_CASE("Bounds subtract side overlap", "[utils]") {
  IBounds bounds1(1, 2, 5, 10);
  IBounds bounds2(0, 3, 5, 5);
  IBounds bounds3(0, 2, 15, 5);
  IBounds result;

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
  IBounds bounds1(0, 0, 2312, 1161);
  IBounds bounds2(0, 1154, 1126, 156);
  std::vector<IBounds> pieces;
  IBounds::breakIntoNonOverlapping(bounds1, bounds2, pieces);
  REQUIRE(!bounds1.overlaps(bounds2));
  REQUIRE(pieces.empty());
}

TEST_CASE("Bounds copy constructor", "[utils]") {
  IBounds original(10, 20, 100, 200);
  REQUIRE(IBounds(original) == original);
}

TEST_CASE("IBounds trimTop", "[utils]") {
  IBounds original(10, 20, 100, 200);

  SECTION("Trim partial height") {
    IBounds removed = original.trimTop(50);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 50);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 70);
    REQUIRE(original.width() == 100);
    REQUIRE(original.height() == 150);
  }

  SECTION("Trim full height") {
    IBounds removed = original.trimTop(200);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 220);
    REQUIRE(original.width() == 100);
    REQUIRE(original.height() == 0);
  }

  SECTION("Trim exceeding height") {
    IBounds removed = original.trimTop(250);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 220);
    REQUIRE(original.width() == 100);
    REQUIRE(original.height() == 0);
  }
}

TEST_CASE("IBounds trimBottom", "[utils]") {
  IBounds original(10, 20, 100, 200);

  SECTION("Trim partial height") {
    IBounds removed = original.trimBottom(50);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 170);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 50);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 100);
    REQUIRE(original.height() == 150);
  }

  SECTION("Trim full height") {
    IBounds removed = original.trimBottom(200);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 100);
    REQUIRE(original.height() == 0);
  }

  SECTION("Trim exceeding height") {
    IBounds removed = original.trimBottom(250);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 100);
    REQUIRE(original.height() == 0);
  }
}

TEST_CASE("IBounds trimLeft", "[utils]") {
  IBounds original(10, 20, 100, 200);

  SECTION("Trim partial width") {
    IBounds removed = original.trimLeft(30);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 30);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 40);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 70);
    REQUIRE(original.height() == 200);
  }

  SECTION("Trim full width") {
    IBounds removed = original.trimLeft(100);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 110);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 0);
    REQUIRE(original.height() == 200);
  }

  SECTION("Trim exceeding width") {
    IBounds removed = original.trimLeft(150);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 110);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 0);
    REQUIRE(original.height() == 200);
  }
}

TEST_CASE("IBounds trimRight", "[utils]") {
  IBounds original(10, 20, 100, 200);

  SECTION("Trim partial width") {
    IBounds removed = original.trimRight(30);

    REQUIRE(removed.x() == 80);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 30);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 70);
    REQUIRE(original.height() == 200);
  }

  SECTION("Trim full width") {
    IBounds removed = original.trimRight(100);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 0);
    REQUIRE(original.height() == 200);
  }

  SECTION("Trim exceeding width") {
    IBounds removed = original.trimRight(150);

    REQUIRE(removed.x() == 10);
    REQUIRE(removed.y() == 20);
    REQUIRE(removed.width() == 100);
    REQUIRE(removed.height() == 200);

    REQUIRE(original.x() == 10);
    REQUIRE(original.y() == 20);
    REQUIRE(original.width() == 0);
    REQUIRE(original.height() == 200);
  }
}

TEST_CASE("IBounds reduced uniform", "[utils]") {
  IBounds original(10, 20, 100, 200);

  SECTION("Reduce by small amount") {
    IBounds reduced = original.reduced(10);

    REQUIRE(reduced.x() == 20);
    REQUIRE(reduced.y() == 30);
    REQUIRE(reduced.width() == 80);
    REQUIRE(reduced.height() == 180);
  }

  SECTION("Reduce beyond IBounds") {
    IBounds reduced = original.reduced(100);

    REQUIRE(reduced.x() == 110);
    REQUIRE(reduced.y() == 120);
    REQUIRE(reduced.width() == 0);
    REQUIRE(reduced.height() == 0);
  }
}

TEST_CASE("IBounds reduced asymmetric", "[utils]") {
  IBounds original(10, 20, 100, 200);

  SECTION("Reduce with different values") {
    IBounds reduced = original.reduced(10, 20, 30, 40);

    REQUIRE(reduced.x() == 20);
    REQUIRE(reduced.y() == 50);
    REQUIRE(reduced.width() == 70);
    REQUIRE(reduced.height() == 130);
  }

  SECTION("Reduce to zero") {
    IBounds reduced = original.reduced(50, 50, 100, 100);

    REQUIRE(reduced.x() == 60);
    REQUIRE(reduced.y() == 120);
    REQUIRE(reduced.width() == 0);
    REQUIRE(reduced.height() == 0);
  }

  SECTION("Reduce beyond IBounds") {
    IBounds reduced = original.reduced(60, 60, 110, 110);

    REQUIRE(reduced.x() == 70);
    REQUIRE(reduced.y() == 130);
    REQUIRE(reduced.width() == 0);
    REQUIRE(reduced.height() == 0);
  }
}
