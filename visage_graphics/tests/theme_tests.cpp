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

#include "visage_graphics/gradient.h"
#include "visage_graphics/palette.h"
#include "visage_graphics/theme.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

VISAGE_THEME_COLOR(TestThemeColor, 0xff000000);
VISAGE_THEME_PALETTE_OVERRIDE(TestThemeOverride);

TEST_CASE("Palette override is separate from global", "[graphics]") {
  REQUIRE_FALSE(TestThemeOverride.isDefault());
  REQUIRE(TestThemeOverride != theme::OverrideId());
  REQUIRE(theme::OverrideId::name(theme::OverrideId()) == "Global");
  REQUIRE(theme::OverrideId::name(TestThemeOverride) == "TestThemeOverride");

  Palette palette;
  palette.setColor(TestThemeOverride, TestThemeColor, Color(0xffff0000));

  Brush override_brush;
  REQUIRE(palette.color(TestThemeOverride, TestThemeColor, override_brush));
  Brush global_brush;
  REQUIRE_FALSE(palette.color(theme::OverrideId(), TestThemeColor, global_brush));
}

TEST_CASE("Palette decode bad data", "[graphics]") {
  Palette palette;
  palette.decode("Global\nTestThemeColor@5\nUnknownColor@0\n\nUnknownOverride\nTestThemeColor@"
                 "0\n\n\n\n-1\n");
  REQUIRE(palette.numColors() == 0);

  Brush brush;
  REQUIRE(palette.color(theme::OverrideId(), TestThemeColor, brush));
  REQUIRE(brush.gradient().sample(0.0f) == Palette::kInvalidColor);

  std::string encoded = palette.encode();
  REQUIRE(encoded.find("Unknown") == std::string::npos);
  Palette decoded;
  decoded.decode(encoded);
  REQUIRE(decoded.encode() == encoded);

  palette.decode("Global\nTestThemeColor@0\n");
  REQUIRE(palette.color(theme::OverrideId(), TestThemeColor, brush));
}

TEST_CASE("Gradient decode bad data", "[graphics]") {
  Gradient gradient;
  gradient.decode("");
  REQUIRE(gradient.resolution() == 0);
  gradient.decode("0 0 -3");
  REQUIRE(gradient.resolution() == 0);
}
