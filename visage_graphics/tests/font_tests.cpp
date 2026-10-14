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

TEST_CASE("Font variation selectors take no space", "[graphics]") {
  Font font(16, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data), 1.0f);
  FontAtlasQuad plain[2];
  FontAtlasQuad selected[3];
  font.setVertexPositions(plain, U"AB", 2, 0, 0, 100, 20);
  font.setVertexPositions(selected, U"A\uFE0FB", 3, 0, 0, 100, 20);

  REQUIRE(font.stringWidth(U"A\uFE0FB") == font.stringWidth(U"AB"));
  REQUIRE(selected[0].x == plain[0].x);
  REQUIRE(selected[1].width == 0.0f);
  REQUIRE(selected[2].x == plain[1].x);

  Canvas canvas;
  canvas.setWindowless(100, 20);
  canvas.setColor(Brush::horizontal(0xffff0000, 0xff0000ff));
  canvas.text(U"A\uFE0FB", font, Font::Justification::kCenter, 0, 0, 100, 20);
  canvas.submit();
}

TEST_CASE("Text edges have matching color and alpha", "[graphics]") {
  auto render = [](unsigned int background, unsigned int text_color) {
    Canvas canvas;
    canvas.setWindowless(120, 40);
    canvas.setColor(background);
    canvas.fill(0, 0, 120, 40);
    Font font(24, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
    canvas.setColor(text_color);
    canvas.text("Masked", font, Font::kCenter, 0, 0, 120, 40);
    return canvas.takeScreenshot();
  };

  Screenshot white_text = render(0xff000000, 0xffffffff);
  Screenshot black_text = render(0xffffffff, 0xff000000);
  int edges = 0;
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 120; ++x) {
      int coverage = white_text.sample(x, y).hexRed();
      int alpha = 0xff - black_text.sample(x, y).hexRed();
      if (alpha > 0 && alpha < 0xff) {
        ++edges;
        REQUIRE(std::abs(coverage - alpha) <= 1);
      }
    }
  }
  REQUIRE(edges > 0);
}

#if VISAGE_LINUX
TEST_CASE("Emoji edges are not darkened", "[graphics]") {
  auto render = [](unsigned int background) {
    Canvas canvas;
    canvas.setWindowless(80, 80);
    canvas.setColor(background);
    canvas.fill(0, 0, 80, 80);
    Font font(48, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
    canvas.setColor(0xffffffff);
    canvas.text(U"\U0001F600", font, Font::kCenter, 0, 0, 80, 80);
    return canvas.takeScreenshot();
  };

  Screenshot on_black = render(0xff000000);
  Screenshot on_white = render(0xffffffff);
  int edges = 0;
  for (int y = 0; y < 80; ++y) {
    for (int x = 0; x < 80; ++x) {
      Color black = on_black.sample(x, y);
      float alpha = 1.0f - (on_white.sample(x, y).red() - black.red());
      bool yellow = black.green() > 0.6f * black.red() && black.blue() < 0.5f * black.red() + 0.05f;
      if (alpha > 0.15f && alpha < 0.85f && yellow) {
        ++edges;
        REQUIRE(black.red() == Catch::Approx(alpha).margin(0.05f));
      }
    }
  }
  REQUIRE(edges > 0);
}

TEST_CASE("Emoji rasterizer character missing from emoji font", "[graphics]") {
  static constexpr int kSize = 32;
  std::vector<unsigned int> buffer(kSize * kSize, 0);
  EmojiRasterizer::instance().drawIntoBuffer(U'\u4e00', kSize, kSize, buffer.data(), kSize, 0, 0);

  int drawn = 0;
  for (unsigned int pixel : buffer) {
    if (pixel == 0)
      continue;
    ++drawn;
    REQUIRE(pixel == (pixel >> 24) * 0x01010101u);
  }
  REQUIRE(drawn > 0);
}
#endif
