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

#include "visage_widgets/color_picker.h"

#include <catch2/catch_test_macros.hpp>
#include <vector>

using namespace visage;

static TextEditor* hexEditor(ColorPicker& picker) {
  return dynamic_cast<TextEditor*>(picker.children()[2]);
}

TEST_CASE("Color picker hex entry keeps alpha and HDR", "[widgets]") {
  ColorPicker picker;
  picker.setColor(Color(0x80ff0000, 2.0f));
  std::vector<Color> colors;
  picker.onColorChange() += [&colors](const Color& color) { colors.push_back(color); };

  TextEditor* hex = hexEditor(picker);
  REQUIRE(hex);
  hex->selectAll();
  hex->processTextInput("00ff00");

  REQUIRE(colors.size() == 1);
  REQUIRE((colors.back().toARGB() & 0xffffff) == 0x00ff00);
  REQUIRE(colors.back().alpha() == Color(0x80ff0000).alpha());
  REQUIRE(colors.back().hdr() == 2.0f);
}

TEST_CASE("Color picker ignores partial hex entry", "[widgets]") {
  ColorPicker picker;
  picker.setColor(Color(0xff123456));
  std::vector<Color> colors;
  picker.onColorChange() += [&colors](const Color& color) { colors.push_back(color); };

  TextEditor* hex = hexEditor(picker);
  REQUIRE(hex);
  hex->selectAll();
  for (char c : std::string("00ff00"))
    hex->processTextInput(std::string(1, c));

  REQUIRE(colors.size() == 1);
  REQUIRE(colors.back().toARGB() == 0xff00ff00);

  hex->selectAll();
  hex->processTextInput("12");
  REQUIRE(colors.size() == 1);

  hex->processFocusChanged(false, false);
  REQUIRE(hex->text().toUtf8() == colors.back().toRGBHexString());
}
