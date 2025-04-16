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
#include <sstream>

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
  REQUIRE(Color(0xff123456) == Color::fromHexString("123456"));
  REQUIRE(Color(0xff123456) == Color::fromHexString("#123456"));
  REQUIRE(Color(0) == Color::fromHexString(""));
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

TEST_CASE("Color fromARGB16 and fromABGR16", "[graphics]") {
  Color color1 = Color::fromARGB16(0x0000555500000000ULL);
  REQUIRE(color1.alpha() == 0.0f);
  REQUIRE(color1.red() == Approx(1.0f / 3.0f));
  REQUIRE(color1.green() == 0.0f);
  REQUIRE(color1.blue() == 0.0f);

  Color color2 = Color::fromABGR16(0x0000555500000000ULL);
  REQUIRE(color2.alpha() == 0.0f);
  REQUIRE(color2.red() == 0.0f);
  REQUIRE(color2.green() == 0.0f);
  REQUIRE(color2.blue() == Approx(1.0f / 3.0f));
}

TEST_CASE("Color toARGB correctly converts to ARGB integer", "[graphics]") {
  Color color(1.0f / 3.0f, 1.0f, 0.0f, 0.0f);
  REQUIRE(color.toARGB() == 0x55FF0000);
}

TEST_CASE("Color toABGR correctly converts to ABGR integer", "[graphics]") {
  Color color(1.0f / 3.0f, 1.0f, 2.0f / 3.0f, 0.0f);
  REQUIRE(color.toABGR() == 0x5500aaFF);
}

TEST_CASE("Color toRGB", "[graphics]") {
  Color color(1.0f, 0.5f, 1.0f, 0.75f);
  REQUIRE(color.toRGB() == 0x80ffbf);
}

TEST_CASE("Color to 16-bit formats", "[graphics]") {
  Color color(1.0f, 1.0f / 15.0f, 1.0f / 3.0f, 2.0f / 3.0f);
  REQUIRE((color.toARGB16() >> 48) == 0xffff);
  REQUIRE(((color.toARGB16() >> 32) & 0xffff) == 0x1111);
  REQUIRE(((color.toARGB16() >> 16) & 0xffff) == 0x5555);
  REQUIRE((color.toARGB16() & 0xffff) == 0xAAAA);
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

  Color c_mult = c1 * 0.5f;
  REQUIRE(c_mult.alpha() == Approx(0.5f));
  REQUIRE(c_mult.red() == Approx(0.25f));
  REQUIRE(c_mult.green() == Approx(0.25f));
  REQUIRE(c_mult.blue() == Approx(0.25f));
}

TEST_CASE("Color comparison operators", "[graphics]") {
  Color c1(1.0f, 0.5f, 0.5f, 0.5f);
  Color c2(1.0f, 0.5f, 0.5f, 0.5f);
  Color c3(0.5f, 0.5f, 0.5f, 0.5f);
  Color c4(1.0f, 0.6f, 0.5f, 0.5f);

  REQUIRE(c1 == c2);
  REQUIRE(c1 < c4);
  REQUIRE(c4 > c1);
  REQUIRE(c3 < c1);
  REQUIRE(Color::compare(c1, c2) == 0);
  REQUIRE(Color::compare(c1, c3) > 0);
  REQUIRE(Color::compare(c3, c1) < 0);
  REQUIRE(Color::compare(c1, c4) < 0);

  Color c5(1.0f, 0.5f, 0.5f, 0.5f, 1.0f);
  Color c6(1.0f, 0.5f, 0.5f, 0.5f, 2.0f);
  REQUIRE(c5 < c6);
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

  REQUIRE(c1.interpolateWith(c2, 0.0f) == c1);
  REQUIRE(c1.interpolateWith(c2, 1.0f) == c2);
}

TEST_CASE("Color withAlpha returns new color with modified alpha", "[graphics]") {
  Color c1(0.8f, 0.5f, 0.25f, 0.1f);
  Color c2 = c1.withAlpha(0.5f);

  REQUIRE(c2.alpha() == Approx(0.5f));
  REQUIRE(c2.red() == c1.red());
  REQUIRE(c2.green() == c1.green());
  REQUIRE(c2.blue() == c1.blue());
}

TEST_CASE("Color setAlpha and setHdr modify color in place", "[graphics]") {
  Color color(0.8f, 0.5f, 0.25f, 0.1f);

  color.setAlpha(0.5f);
  REQUIRE(color.alpha() == Approx(0.5f));

  color.setHdr(2.0f);
  REQUIRE(color.hdr() == Approx(2.0f));

  color.setAlpha(1.5f);
  REQUIRE(color.alpha() == Approx(1.0f));

  color.setAlpha(-0.5f);
  REQUIRE(color.alpha() == Approx(0.0f));

  color.setHdr(-1.0f);
  REQUIRE(color.hdr() == Approx(0.0f));
}

TEST_CASE("Color multRgb modifies RGB values proportionally", "[graphics]") {
  Color color(1.0f, 0.8f, 0.6f, 0.4f);

  color.multRgb(0.5f);

  REQUIRE(color.alpha() == Approx(1.0f));
  REQUIRE(color.red() == Approx(0.4f));
  REQUIRE(color.green() == Approx(0.3f));
  REQUIRE(color.blue() == Approx(0.2f));
}

TEST_CASE("Color hue, saturation, and value calculations are correct", "[graphics]") {
  Color color(1.0f, 1.0f, 0.5f, 0.0f, 2.0f);
  REQUIRE(color.hue() == Approx(30.0f).margin(1.0f));
  REQUIRE(color.saturation() == Approx(1.0f));
  REQUIRE(color.value() == Approx(1.0f));
  REQUIRE(color.hdr() == Approx(2.0f));

  Color black(1.0f, 0.0f, 0.0f, 0.0f);
  REQUIRE(black.value() == Approx(0.0f));
  REQUIRE(black.saturation() == Approx(0.0f));
  REQUIRE(black.hue() == Approx(0.0f));

  Color white(1.0f, 1.0f, 1.0f, 1.0f);
  REQUIRE(white.value() == Approx(1.0f));
  REQUIRE(white.saturation() == Approx(0.0f));
  REQUIRE(white.hue() == Approx(0.0f));

  Color gray(1.0f, 0.5f, 0.5f, 0.5f);
  REQUIRE(gray.value() == Approx(0.5f));
  REQUIRE(gray.saturation() == Approx(0.0f));
  REQUIRE(gray.hue() == Approx(0.0f));
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

  color = Color::fromAHSV(1.0f, 0.0f, 0.0f, 0.0f);
  REQUIRE(color.red() == Approx(0.0f));
  REQUIRE(color.green() == Approx(0.0f));
  REQUIRE(color.blue() == Approx(0.0f));
}

TEST_CASE("Color encode/decode", "[graphics]") {
  Color color;
  Color result(1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
  result.decode(color.encode());
  REQUIRE(color == result);

  color = Color(0.5f, 0.25f, 0.75f, 0.125f, 2.0f);
  result.decode(color.encode());
  REQUIRE(color == result);

  color = Color(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  result.decode(color.encode());
  REQUIRE(color == result);

  color = Color(1.0f, 1.0f, 1.0f, 1.0f, 10.0f);
  result.decode(color.encode());
  REQUIRE(color == result);

  color = Color(0.5f, 0.25f, 0.75f, 0.125f, 2.0f);
  std::ostringstream ostream;
  color.encode(ostream);

  std::istringstream istream(ostream.str());
  result.decode(istream);
  REQUIRE(color == result);
}