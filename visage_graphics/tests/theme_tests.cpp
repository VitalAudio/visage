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
#include <clocale>
#include <string>
#include <vector>

using namespace visage;

VISAGE_THEME_COLOR(TestThemeColor, 0xff000000);
VISAGE_THEME_VALUE(TestThemeValue, 1.0f);
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

TEST_CASE("Palette values decode under a comma decimal locale", "[graphics]") {
  Palette palette;
  palette.setValue(TestThemeValue, 0.25f);
  std::string encoded = palette.encode();

  std::string previous_locale = setlocale(LC_NUMERIC, nullptr);
  for (const char* locale : { "de_DE.UTF-8", "de_DE.utf8", "en_DK.utf8", "fr_FR.UTF-8", "German" }) {
    if (setlocale(LC_NUMERIC, locale))
      break;
  }
  Palette decoded;
  decoded.decode(encoded);
  setlocale(LC_NUMERIC, previous_locale.c_str());

  float value = 0.0f;
  REQUIRE(decoded.value(theme::OverrideId(), TestThemeValue, value));
  REQUIRE(value == 0.25f);
}

TEST_CASE("Theme group names come from the file name", "[graphics]") {
  REQUIRE(theme::nameFromPath("/src/widgets/button.cpp") == "button");
  REQUIRE(theme::nameFromPath("C:\\src\\button.cpp") == "button");
  REQUIRE(theme::nameFromPath("button.cpp") == "button");
  REQUIRE(theme::nameFromPath("button") == "button");
  REQUIRE(theme::nameFromPath("widgets.v2/button") == "button");
}

TEST_CASE("Palette and theme lookups don't add entries", "[graphics]") {
  Palette palette;
  Brush brush;
  float value = 0.0f;
  REQUIRE_FALSE(palette.color(TestThemeOverride, TestThemeColor, brush));
  REQUIRE_FALSE(palette.value(TestThemeOverride, TestThemeValue, value));
  REQUIRE(palette.colorMap(TestThemeOverride, TestThemeColor) == Palette::kNotSetId);
  palette.removeValue(TestThemeOverride, TestThemeValue);
  REQUIRE(palette.encode() == Palette().encode());

  int value_names = theme::ValueId::nameIdMap().size();
  int override_names = theme::OverrideId::nameIdMap().size();
  REQUIRE(theme::ValueId::name(theme::ValueId()).empty());
  REQUIRE(theme::OverrideId::name(theme::OverrideId(theme::OverrideId::kInvalidId)).empty());
  REQUIRE(theme::ValueId::nameIdMap().size() == value_names);
  REQUIRE(theme::OverrideId::nameIdMap().size() == override_names);
}

TEST_CASE("Palette lists ids looked up or cleared without a saved value", "[graphics]") {
  Palette palette;
  Brush brush;
  float value = 0.0f;
  palette.color(TestThemeOverride, TestThemeColor, brush);
  REQUIRE(palette.colorIdList(TestThemeOverride)["theme_tests"] ==
          std::vector<theme::ColorId> { TestThemeColor });

  palette.setValue(TestThemeOverride, TestThemeValue, 0.5f);
  REQUIRE(palette.value(TestThemeOverride, TestThemeValue, value));
  palette.removeValue(TestThemeOverride, TestThemeValue);
  REQUIRE_FALSE(palette.value(TestThemeOverride, TestThemeValue, value));
  REQUIRE(palette.valueIdList(TestThemeOverride)["theme_tests"] ==
          std::vector<theme::ValueId> { TestThemeValue });

  REQUIRE(palette.encode() == Palette().encode());
  palette.clear();
  REQUIRE(palette.colorIdList(TestThemeOverride).empty());
  REQUIRE(palette.valueIdList(TestThemeOverride).empty());
}

TEST_CASE("Gradient decode bad data", "[graphics]") {
  Gradient gradient;
  gradient.decode("");
  REQUIRE(gradient.resolution() == 0);
  gradient.decode("0 0 -3");
  REQUIRE(gradient.resolution() == 0);
}
