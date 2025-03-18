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

#include "visage_utils/dimension.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;
using namespace visage::dimension;

TEST_CASE("Dimension defaults", "[utils]") {
  Dimension dimension;
  REQUIRE(dimension.compute(1.0f, 100.0f, 100.0f, 99.0f) == 99.0f);
}

TEST_CASE("Dimension native pixels", "[utils]") {
  Dimension dim1 = 99_npx;
  REQUIRE(dim1.compute(2, 100, 100) == 99.0f);
  Dimension dim2 = 0_npx;
  REQUIRE(dim2.compute(2, 100, 100) == 0.0f);
}

TEST_CASE("Dimension logical pixels", "[utils]") {
  Dimension dim1 = 99_px;
  REQUIRE(dim1.compute(1, 100, 100) == 99.0f);
  REQUIRE(dim1.compute(2, 100, 100) == 198.0f);
  REQUIRE(dim1.compute(3, 100, 100) == 297.0f);

  Dimension dim2 = 0_px;
  REQUIRE(dim2.compute(1, 100, 100) == 0.0f);
  REQUIRE(dim2.compute(2, 100, 100) == 0.0f);
}

TEST_CASE("Dimension width/height percentages", "[utils]") {
  Dimension dim1 = 0_vw;
  REQUIRE(dim1.compute(1, 198, 100) == 0.0f);
  REQUIRE(dim1.compute(2, 500, 100) == 0.0f);

  Dimension dim2 = 50_vw;
  REQUIRE(dim2.compute(1, 198, 100) == 99.0f);
  REQUIRE(dim2.compute(2, 500, 100) == 250.0f);

  Dimension dim3 = 50_vh;
  REQUIRE(dim3.compute(1, 100, 198) == 99.0f);
  REQUIRE(dim3.compute(2, 100, 500) == 250.0f);

  Dimension dim4 = 50_vmin;
  REQUIRE(dim3.compute(1, 1000, 198) == 99.0f);
  REQUIRE(dim3.compute(2, 1000, 500) == 250.0f);

  Dimension dim5 = 50_vmax;
  REQUIRE(dim3.compute(1, 100, 198) == 99.0f);
  REQUIRE(dim3.compute(2, 100, 500) == 250.0f);
}

TEST_CASE("Dimension combination", "[utils]") {
  Dimension device_pixels = 99_npx;
  Dimension zero = 0_npx;
  Dimension logical_pixels = 99_px;
  Dimension half_view_width = 50_vw;
  Dimension half_view_height = 50_vh;
  Dimension view_min = 100_vmin;
  Dimension view_max = 100_vmax;

  REQUIRE((half_view_height + half_view_width).compute(2, 100, 198) == 149.0f);
  REQUIRE((half_view_height - half_view_width).compute(2, 100, 198) == 49.0f);
  REQUIRE((view_max + view_min).compute(2, 100, 198) == 298.0f);
  REQUIRE((view_max - view_min).compute(2, 100, 198) == 98.0f);
  REQUIRE((view_max - view_min).compute(2, 198, 100) == 98.0f);
  REQUIRE((logical_pixels - device_pixels + zero).compute(2, 198, 100) == 99.0f);
  REQUIRE((2.0f * (logical_pixels - view_min)).compute(2, 198, 100) == 196.0f);
}