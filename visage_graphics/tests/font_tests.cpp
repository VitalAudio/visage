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
#include "visage_utils/file_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>

#if VISAGE_WINDOWS
#include <process.h>
#else
#include <unistd.h>
#endif

using namespace visage;
using namespace Catch;

static std::filesystem::path uniqueTempPath(const std::string& name) {
#if VISAGE_WINDOWS
  int process_id = _getpid();
#else
  int process_id = getpid();
#endif
  return std::filesystem::temp_directory_path() / (std::to_string(process_id) + "_" + name);
}

TEST_CASE("Font size keeps fractional sizes", "[graphics]") {
  Font font(10.5f, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  REQUIRE(font.size() == 10.5f);
  REQUIRE(font.withSize(12.25f).size() == 12.25f);
}

static Font global_font;

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

TEST_CASE("Default font measures and draws as empty", "[graphics]") {
  Font font = Font().withDpiScale(1.0f);
  REQUIRE(font.packedFont() == nullptr);
  REQUIRE(font.stringWidth(U"Test") == 0.0f);
  REQUIRE(font.stringWidth(U"Test", 4, U'*') == 0.0f);
  REQUIRE(font.lineHeight() == 0.0f);
  REQUIRE(font.capitalHeight() == 0.0f);
  REQUIRE(font.widthOverflowIndex(U"Test", 4, 10.0f) == 4);
  REQUIRE(font.lineBreaks(U"Test\ntest", 9, 10.0f).size() == 1);

  Canvas canvas;
  canvas.setWindowless(100, 50);
  canvas.text("Test", Font(), Font::Justification::kCenter, 0, 0, 100, 50);
  canvas.submit();
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

TEST_CASE("Fonts with the same names but different data don't share a cache entry", "[graphics]") {
  auto bdf = [](int advance) {
    return std::string("STARTFONT 2.1\n"
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
                       "DWIDTH ") +
           std::to_string(advance) +
           " 0\n"
           "BBX 8 8 0 0\n"
           "BITMAP\n"
           "80\n80\n80\n80\n80\n80\n80\n80\n"
           "ENDCHAR\n"
           "ENDFONT\n";
  };

  std::string narrow = bdf(8);
  std::string wide = bdf(12);
  Font narrow_font(8, reinterpret_cast<const unsigned char*>(narrow.data()), narrow.size(), 1.0f);
  Font wide_font(8, reinterpret_cast<const unsigned char*>(wide.data()), wide.size(), 1.0f);
  REQUIRE(narrow_font.stringWidth(U"A") == 8.0f);
  REQUIRE(wide_font.stringWidth(U"A") == 12.0f);
}

TEST_CASE("Resized fonts share the cache with fonts loaded at that size", "[graphics]") {
  File path = uniqueTempPath("visage_font_cache_test.ttf");
  REQUIRE(replaceFileWithData(path, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data)));

  Font file_font(16, path.string(), 1.0f);
  Font data_font(16, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data), 1.0f);
  REQUIRE(file_font.withSize(20).packedFont() == Font(20, path.string(), 1.0f).packedFont());
  REQUIRE(file_font.withDpiScale(2.0f).packedFont() == Font(32, path.string(), 1.0f).packedFont());
  REQUIRE(data_font.withSize(20).packedFont() ==
          Font(20, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data), 1.0f).packedFont());
  REQUIRE(file_font.withSize(20).stringWidth(U"Test") ==
          data_font.withSize(20).stringWidth(U"Test"));

  std::filesystem::remove(path);
}

TEST_CASE("Font lower dip height is the descent below the baseline", "[graphics]") {
  Font font(20, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data), 1.0f);
  REQUIRE(font.lowerDipHeight() > 0.0f);
  REQUIRE(font.lowerDipHeight() < font.capitalHeight() / 2.0f);
  REQUIRE(font.withDpiScale(2.0f).lowerDipHeight() == Approx(font.lowerDipHeight()).margin(0.5f));
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

TEST_CASE("A Font held in a global is released after the font cache", "[graphics]") {
  global_font = Font(10, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  REQUIRE(global_font.packedFont());
}

TEST_CASE("Clamps that cut text glyphs keep each glyph pixel in place", "[graphics]") {
  Font font(24, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  auto render = [&font](Direction direction, float clamp_left, float clamp_width) {
    Canvas canvas;
    canvas.setWindowless(80, 80);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, 80, 80);
    canvas.setClampBounds(clamp_left, 0, clamp_width, 80);
    canvas.setColor(0xffffffff);
    canvas.text("Wg", font, Font::Justification::kCenter, 0, 0, 80, 80, direction);
    return canvas.takeScreenshot();
  };

  for (Direction direction : { Direction::Up, Direction::Down, Direction::Left, Direction::Right }) {
    INFO("direction " << static_cast<int>(direction));
    Screenshot full = render(direction, 0, 80);
    Screenshot clamped = render(direction, 34, 12);
    int lit = 0;
    int max_difference = 0;
    for (int y = 0; y < 80; ++y) {
      for (int x = 34; x < 46; ++x) {
        int full_red = static_cast<int>(full.sample(x, y).hexRed());
        int clamped_red = static_cast<int>(clamped.sample(x, y).hexRed());
        lit += full_red > 0x80;
        max_difference = std::max(max_difference, std::abs(full_red - clamped_red));
      }
    }
    REQUIRE(lit > 20);
    REQUIRE(max_difference <= 2);
  }
}
