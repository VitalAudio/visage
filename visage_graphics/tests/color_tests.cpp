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

#include "visage_graphics/color.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <random>

using namespace visage;
using namespace Catch;

TEST_CASE("Color initialization", "[graphics]") {
  Color color1;
  REQUIRE(color1.alpha() == 0.0f);
  REQUIRE(color1.red() == 0.0f);
  REQUIRE(color1.green() == 0.0f);
  REQUIRE(color1.blue() == 0.0f);

  Color color2(0.5f, 0.25f, 0.75f, 0.125f);
  REQUIRE(color2.alpha() == 0.5f);
  REQUIRE(color2.red() == 0.25f);
  REQUIRE(color2.green() == 0.75f);
  REQUIRE(color2.blue() == 0.125f);

  Color color3(0xffffffff);
  REQUIRE(color3.alpha() == 1.0f);
  REQUIRE(color3.red() == 1.0f);
  REQUIRE(color3.green() == 1.0f);
  REQUIRE(color3.blue() == 1.0f);

  Color color4(0xf1d1a181);
  REQUIRE(color4 == 0xf1d1a181);
  REQUIRE(!(color4 == 0xf2d1a181));
  REQUIRE(!(color4 == 0xf1d2a181));
  REQUIRE(!(color4 == 0xf1d1a281));
  REQUIRE(!(color4 == 0xf1d1a182));
  REQUIRE(color4.toARGB() == 0xf1d1a181);
  REQUIRE(color4.hexAlpha() == 0xf1);
  REQUIRE(color4.hexRed() == 0xd1);
  REQUIRE(color4.hexGreen() == 0xa1);
  REQUIRE(color4.hexBlue() == 0x81);
}

TEST_CASE("Color default constructor initializes to zero values", "[graphics]") {
  Color color;
  REQUIRE(color.alpha() == 0.0f);
  REQUIRE(color.red() == 0.0f);
  REQUIRE(color.green() == 0.0f);
  REQUIRE(color.blue() == 0.0f);
}

TEST_CASE("Color fromARGB correctly initializes from ARGB integer", "[graphics]") {
  Color color = Color::fromARGB(0x55FF0000);
  REQUIRE(color.alpha() == Approx(1.0f / 3.0f));
  REQUIRE(color.red() == Approx(1.0f));
  REQUIRE(color.green() == Approx(0.0f));
  REQUIRE(color.blue() == Approx(0.0f));
}

TEST_CASE("Color fromHexString correctly initializes", "[graphics]") {
  REQUIRE(Color(0x12345678) == Color::fromHexString("#12345678"));
  REQUIRE(Color(0x12345678) == Color::fromHexString("12345678"));
}

TEST_CASE("Color toARGBHexString converts correctly", "[graphics]") {
  REQUIRE(Color(0x12345678).toARGBHexString() == "12345678");
  REQUIRE(Color(0x12345678).toRGBHexString() == "345678");
}

TEST_CASE("Color fromABGR correctly initializes from ABGR integer", "[graphics]") {
  Color color = Color::fromABGR(0x550000FF);
  REQUIRE(color.alpha() == Approx(1.0f / 3.0f));
  REQUIRE(color.red() == Approx(1.0f));
  REQUIRE(color.green() == Approx(0.0f));
  REQUIRE(color.blue() == Approx(0.0f));
}

TEST_CASE("Color toARGB correctly converts to ARGB integer", "[graphics]") {
  Color color(1.0f / 3.0f, 1.0f, 0.0f, 0.0f);
  REQUIRE(color.toARGB() == 0x55FF0000);
}

TEST_CASE("Color toABGR correctly converts to ABGR integer", "[graphics]") {
  Color color(1.0f / 3.0f, 1.0f, 2.0f / 3.0f, 0.0f);
  REQUIRE(color.toABGR() == 0x5500aaFF);
}

TEST_CASE("Color arithmetic operations work correctly", "[graphics]") {
  Color c1(1.0f, 0.5f, 0.5f, 0.5f);
  Color c2(0.5f, 0.2f, 0.2f, 0.2f);

  Color c_add = c1 + c2;
  REQUIRE(c_add.alpha() == Approx(1.5f));
  REQUIRE(c_add.red() == Approx(0.7f));
  REQUIRE(c_add.green() == Approx(0.7f));
  REQUIRE(c_add.blue() == Approx(0.7f));

  Color c_sub = c1 - c2;
  REQUIRE(c_sub.alpha() == Approx(0.5f));
  REQUIRE(c_sub.red() == Approx(0.3f));
  REQUIRE(c_sub.green() == Approx(0.3f));
  REQUIRE(c_sub.blue() == Approx(0.3f));
}

TEST_CASE("Color interpolation works correctly", "[graphics]") {
  Color c1(1.0f, 0.5f, 0.0f, 0.0f, 2.0f);
  Color c2(1.0f, 0.0f, 1.0f, 0.4f, 3.0f);

  Color mid = c1.interpolateWith(c2, 0.25f);
  REQUIRE(mid.alpha() == Approx(1.0f));
  REQUIRE(mid.red() == Approx(0.375f));
  REQUIRE(mid.green() == Approx(0.25f));
  REQUIRE(mid.blue() == Approx(0.1f));
  REQUIRE(mid.hdr() == Approx(2.25f));
}

TEST_CASE("Color hue, saturation, and value calculations are correct", "[graphics]") {
  Color color(1.0f, 1.0f, 0.5f, 0.0f, 2.0f);
  REQUIRE(color.hue() == Approx(30.0f).margin(1.0f));
  REQUIRE(color.saturation() == Approx(1.0f));
  REQUIRE(color.value() == Approx(1.0f));
  REQUIRE(color.hdr() == Approx(2.0f));
}

TEST_CASE("Color fromAHSV", "[graphics]") {
  Color color = Color::fromAHSV(1.0f, 0.0f, 1.0f, 1.0f);
  REQUIRE(color.alpha() == Approx(1.0f));
  REQUIRE(color.red() == Approx(1.0f));
  REQUIRE(color.green() == Approx(0.0f));
  REQUIRE(color.blue() == Approx(0.0f));
  REQUIRE(color.hue() == Approx(0.0f));
  REQUIRE(color.saturation() == Approx(1.0f));
  REQUIRE(color.value() == Approx(1.0f));

  color = Color::fromAHSV(0.75f, 60.0f, 1.0f, 0.5f);
  REQUIRE(color.alpha() == Approx(0.75f));
  REQUIRE(color.red() == Approx(0.5f));
  REQUIRE(color.green() == Approx(0.5f));
  REQUIRE(color.blue() == Approx(0.0f));
  REQUIRE(color.hue() == Approx(60.0f));
  REQUIRE(color.saturation() == Approx(1.0f));
  REQUIRE(color.value() == Approx(0.5f));

  color = Color::fromAHSV(1.0f, 120.0f, 1.0f / 3.0f, 0.75f);
  REQUIRE(color.alpha() == Approx(1.0f));
  REQUIRE(color.red() == Approx(0.5f));
  REQUIRE(color.green() == Approx(0.75f));
  REQUIRE(color.blue() == Approx(0.5f));
  REQUIRE(color.hue() == Approx(120.0f));
  REQUIRE(color.saturation() == Approx(1.0f / 3.0f));
  REQUIRE(color.value() == Approx(0.75f));

  color = Color::fromAHSV(1.0f, 180.0f, 0.5f, 1.0f);
  REQUIRE(color.alpha() == Approx(1.0f));
  REQUIRE(color.red() == Approx(0.5f));
  REQUIRE(color.green() == Approx(1.0f));
  REQUIRE(color.blue() == Approx(1.0f));
  REQUIRE(color.hue() == Approx(180.0f));
  REQUIRE(color.saturation() == Approx(0.5f));
  REQUIRE(color.value() == Approx(1.0f));

  color = Color::fromAHSV(1.0f, 240.0f, 0.25f, 1.0f);
  REQUIRE(color.alpha() == Approx(1.0f));
  REQUIRE(color.red() == Approx(0.75f));
  REQUIRE(color.green() == Approx(0.75f));
  REQUIRE(color.blue() == Approx(1.0f));
  REQUIRE(color.hue() == Approx(240.0f));
  REQUIRE(color.saturation() == Approx(0.25f));
  REQUIRE(color.value() == Approx(1.0f));

  color = Color::fromAHSV(1.0f, 300.0f, 1.0f, 1.0f);
  REQUIRE(color.alpha() == Approx(1.0f));
  REQUIRE(color.red() == Approx(1.0f));
  REQUIRE(color.green() == Approx(0.0f));
  REQUIRE(color.blue() == Approx(1.0f));
  REQUIRE(color.hue() == Approx(300.0f));
  REQUIRE(color.saturation() == Approx(1.0f));
  REQUIRE(color.value() == Approx(1.0f));

  color = Color::fromAHSV(1.0f, 360.0f, 1.0f, 1.0f);
  REQUIRE(Color::fromAHSV(1.0f, 0.0f, 1.0f, 1.0f).toARGB() ==
          Color::fromAHSV(1.0f, 360.0f, 1.0f, 1.0f).toARGB());
  REQUIRE(color.hue() == 0.0f);

  color = Color::fromAHSV(1.0f, 420.0f, 1.0f, 1.0f);
  REQUIRE(color.hue() == 60.0f);
}

TEST_CASE("Color encode/decode", "[graphics]") {
  Color color;
  Color result(1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  result.decode(color.encode());
  REQUIRE(color == result);

  color = Color(0.5f, 0.25f, 0.75f, 0.125f, 2.0f);
  result.decode(color.encode());
  REQUIRE(color == result);
}