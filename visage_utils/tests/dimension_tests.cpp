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

#include "visage_utils/dimension.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;
using namespace visage::dimension;

TEST_CASE("Dimension defaults", "[utils]") {
  Dimension dimension;
  REQUIRE(dimension.computeWithDefault(1.0f, 100.0f, 100.0f, 99.0f) == 99.0f);
}

TEST_CASE("Dimension device pixels", "[utils]") {
  Dimension dim1 = 99_dpx;
  REQUIRE(dim1.compute(2, 100, 100) == 99.0f);
  Dimension dim2 = 0_dpx;
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
  Dimension device_pixels = 99_dpx;
  Dimension zero = 0_dpx;
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