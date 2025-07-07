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
  REQUIRE(dim4.compute(1, 1000, 198) == 99.0f);
  REQUIRE(dim4.compute(2, 1000, 500) == 250.0f);

  Dimension dim5 = 50_vmax;
  REQUIRE(dim5.compute(1, 100, 198) == 99.0f);
  REQUIRE(dim5.compute(2, 100, 500) == 250.0f);
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

TEST_CASE("Dimension computeInt function", "[utils]") {
  Dimension dim1 = 99.7_px;
  REQUIRE(dim1.computeInt(1.0f, 100.0f, 100.0f) == 100);
  REQUIRE(dim1.computeInt(2.0f, 100.0f, 100.0f) == 199);
  
  Dimension dim2 = 50.4_vw;
  REQUIRE(dim2.computeInt(1.0f, 200.0f, 100.0f) == 101);
  
  Dimension dim3;
  REQUIRE(dim3.computeInt(1.0f, 100.0f, 100.0f, 42) == 42);
}

TEST_CASE("Dimension constructors", "[utils]") {
  Dimension dim1(50.0f);
  REQUIRE(dim1.compute(2.0f, 100.0f, 100.0f) == 100.0f);
  
  Dimension dim2(25.0f, [](float amount, float scale, float, float) { 
    return amount * scale * 2.0f; 
  });
  REQUIRE(dim2.compute(3.0f, 100.0f, 100.0f) == 150.0f);
}

TEST_CASE("Dimension static min and max functions", "[utils]") {
  Dimension a = 100_px;
  Dimension b = 50_px;
  Dimension c = 200_npx;
  
  Dimension min_result = Dimension::min(a, b);
  REQUIRE(min_result.compute(2.0f, 100.0f, 100.0f) == 100.0f);
  
  Dimension max_result = Dimension::max(a, b);
  REQUIRE(max_result.compute(2.0f, 100.0f, 100.0f) == 200.0f);
  
  Dimension min_mixed = Dimension::min(a, c);
  REQUIRE(min_mixed.compute(2.0f, 100.0f, 100.0f) == 200.0f);
  
  Dimension max_mixed = Dimension::max(a, c);
  REQUIRE(max_mixed.compute(2.0f, 100.0f, 100.0f) == 200.0f);
}

TEST_CASE("Dimension compound assignment operators", "[utils]") {
  Dimension a = 100_px;
  Dimension b = 50_px;
  
  a += b;
  REQUIRE(a.compute(1.0f, 100.0f, 100.0f) == 150.0f);
  
  a -= b;
  REQUIRE(a.compute(1.0f, 100.0f, 100.0f) == 100.0f);
  
  Dimension c = 25_vw;
  Dimension d = 25_vh;
  c += d;
  REQUIRE(c.compute(1.0f, 200.0f, 400.0f) == 150.0f);
}

TEST_CASE("Dimension multiplication and division operators", "[utils]") {
  Dimension a = 100_px;
  
  Dimension scaled = a * 2.5f;
  REQUIRE(scaled.compute(1.0f, 100.0f, 100.0f) == 250.0f);
  
  Dimension friend_scaled = 3.0f * a;
  REQUIRE(friend_scaled.compute(1.0f, 100.0f, 100.0f) == 300.0f);
  
  Dimension divided = a / 2.0f;
  REQUIRE(divided.compute(1.0f, 100.0f, 100.0f) == 50.0f);
  
  Dimension view_scaled = 50_vw * 0.5f;
  REQUIRE(view_scaled.compute(1.0f, 200.0f, 100.0f) == 50.0f);
}

TEST_CASE("Dimension instance min and max methods", "[utils]") {
  Dimension a = 100_px;
  Dimension b = 50_px;
  Dimension c = 200_npx;
  
  REQUIRE(a.min(b).compute(2.0f, 100.0f, 100.0f) == 100.0f);
  REQUIRE(a.max(b).compute(2.0f, 100.0f, 100.0f) == 200.0f);
  REQUIRE(b.min(c).compute(2.0f, 100.0f, 100.0f) == 100.0f);
  REQUIRE(b.max(c).compute(2.0f, 100.0f, 100.0f) == 200.0f);
}

TEST_CASE("Dimension user-defined literal variants", "[utils]") {
  Dimension int_npx = 50_npx;
  Dimension float_npx = 50.5_npx;
  REQUIRE(int_npx.compute(2.0f, 100.0f, 100.0f) == 50.0f);
  REQUIRE(float_npx.compute(2.0f, 100.0f, 100.0f) == 50.5f);
  
  Dimension int_px = 25_px;
  Dimension float_px = 25.5_px;
  REQUIRE(int_px.compute(2.0f, 100.0f, 100.0f) == 50.0f);
  REQUIRE(float_px.compute(2.0f, 100.0f, 100.0f) == 51.0f);
  
  Dimension int_vw = 10_vw;
  Dimension float_vw = 10.5_vw;
  REQUIRE(int_vw.compute(1.0f, 100.0f, 200.0f) == Catch::Approx(10.0f));
  REQUIRE(float_vw.compute(1.0f, 100.0f, 200.0f) == Catch::Approx(10.5f));
  
  Dimension int_vh = 20_vh;
  Dimension float_vh = 20.5_vh;
  REQUIRE(int_vh.compute(1.0f, 100.0f, 200.0f) == Catch::Approx(40.0f));
  REQUIRE(float_vh.compute(1.0f, 100.0f, 200.0f) == Catch::Approx(41.0f));
  
  Dimension int_vmin = 15_vmin;
  Dimension float_vmin = 15.5_vmin;
  REQUIRE(int_vmin.compute(1.0f, 200.0f, 100.0f) == Catch::Approx(15.0f));
  REQUIRE(float_vmin.compute(1.0f, 200.0f, 100.0f) == Catch::Approx(15.5f));
  
  Dimension int_vmax = 30_vmax;
  Dimension float_vmax = 30.5_vmax;
  REQUIRE(int_vmax.compute(1.0f, 200.0f, 100.0f) == Catch::Approx(60.0f));
  REQUIRE(float_vmax.compute(1.0f, 200.0f, 100.0f) == Catch::Approx(61.0f));
}