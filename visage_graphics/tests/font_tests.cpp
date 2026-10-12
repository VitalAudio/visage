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

#include "lato_regular.h"
#include "visage_graphics/canvas.h"
#include "visage_graphics/emoji.h"
#include "visage_graphics/font.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;
using namespace Catch;

TEST_CASE("Font embedded test", "[graphics]") {
  Font font(16, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  Canvas canvas;
  canvas.setWindowless(100, 50);
  canvas.text("Test", font, Font::Justification::kCenter, 0, 0, 100, 50);
  canvas.submit();
  canvas.takeScreenshot();
  canvas.submit();
}
TEST_CASE("Font invalid data", "[graphics]") {
  static constexpr unsigned char kJunk[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };

  SECTION("Missing file") {
    Font font(16, std::string("/missing/font.ttf"), 1.0f);
    REQUIRE(font.stringWidth(U"Test") == 0.0f);
  }

  SECTION("Junk data") {
    Font font(16, kJunk, sizeof(kJunk), 1.0f);
    REQUIRE(font.stringWidth(U"Test") == 0.0f);

    Canvas canvas;
    canvas.setWindowless(100, 50);
    canvas.text("Test", font, Font::Justification::kCenter, 0, 0, 100, 50);
    canvas.submit();
  }
}

TEST_CASE("Font with 1-bit bitmap glyphs", "[graphics]") {
  static constexpr char kBdf[] = "STARTFONT 2.1\n"
                                 "FONT -test-mono-medium-r-normal--8-80-75-75-c-80-iso10646-1\n"
                                 "SIZE 8 75 75\n"
                                 "FONTBOUNDINGBOX 8 8 0 0\n"
                                 "STARTPROPERTIES 2\n"
                                 "FONT_ASCENT 8\n"
                                 "FONT_DESCENT 0\n"
                                 "ENDPROPERTIES\n"
                                 "CHARS 1\n"
                                 "STARTCHAR A\n"
                                 "ENCODING 65\n"
                                 "SWIDTH 500 0\n"
                                 "DWIDTH 8 0\n"
                                 "BBX 8 8 0 0\n"
                                 "BITMAP\n"
                                 "80\n80\n80\n80\n80\n80\n80\n80\n"
                                 "ENDCHAR\n"
                                 "ENDFONT\n";

  Font font(8, reinterpret_cast<const unsigned char*>(kBdf), sizeof(kBdf) - 1);
  Canvas canvas;
  canvas.setWindowless(20, 20);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 20, 20);
  canvas.setColor(0xffffffff);
  canvas.text("A", font, Font::Justification::kCenter, 0, 0, 20, 20);
  canvas.submit();
  canvas.takeScreenshot();

  const Screenshot& screenshot = canvas.screenshot();
  int lit = 0;
  for (int i = 0; i < screenshot.width() * screenshot.height(); ++i)
    lit += screenshot.data()[i * 4] > 0x80;
  REQUIRE(lit == 8);
}

#if VISAGE_LINUX
TEST_CASE("Emoji rasterizer character missing from emoji font", "[graphics]") {
  static constexpr int kSize = 32;
  std::vector<unsigned int> buffer(kSize * kSize, 0);
  EmojiRasterizer::instance().drawIntoBuffer(U'\u4e00', kSize, kSize, buffer.data(), kSize, 0, 0);

  int drawn = 0;
  for (unsigned int pixel : buffer) {
    if (pixel == 0)
      continue;
    ++drawn;
    REQUIRE((pixel & 0xffffff) == 0xffffff);
  }
  REQUIRE(drawn > 0);
}
#endif
