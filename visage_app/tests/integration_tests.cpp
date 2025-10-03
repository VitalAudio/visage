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

#include "visage/app.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <visage/ui.h>
#include <visage/widgets.h>

using namespace visage;
using namespace Catch;

TEST_CASE("Screenshot solid color", "[integration]") {
  ApplicationEditor editor;
  editor.onDraw() = [&editor](Canvas& canvas) {
    canvas.setColor(0xffddaa88);
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  editor.setWindowless(10, 5);
  Screenshot screenshot = editor.takeScreenshot();
  REQUIRE(screenshot.width() == 10);
  REQUIRE(screenshot.height() == 5);
  uint8_t* data = screenshot.data();
  int i = 0;
  for (int y = 0; y < 5; ++y) {
    for (int x = 0; x < 10; ++x) {
      REQUIRE(data[i++] == 0xdd);
      REQUIRE(data[i++] == 0xaa);
      REQUIRE(data[i++] == 0x88);
      REQUIRE(data[i++] == 0xff);
    }
  }
}

TEST_CASE("Screenshot vertical gradient", "[integration]") {
  Color source = 0xff345678;
  Color destination = 0xff88aacc;
  ApplicationEditor editor;
  editor.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(Brush::vertical(source, destination));
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  editor.setWindowless(10, 5);
  Screenshot screenshot = editor.takeScreenshot();
  REQUIRE(screenshot.width() == 10);
  REQUIRE(screenshot.height() == 5);
  uint8_t* data = screenshot.data();
  int i = 0;
  for (int y = 0; y < 5; ++y) {
    float t = y / 4.0f;
    Color sample = source.interpolateWith(destination, t);

    for (int x = 0; x < 10; ++x) {
      REQUIRE(data[i++] == sample.hexRed());
      REQUIRE(data[i++] == sample.hexGreen());
      REQUIRE(data[i++] == sample.hexBlue());
      REQUIRE(data[i++] == 0xff);
    }
  }
}

TEST_CASE("Screenshot horizontal gradient", "[integration]") {
  Color source = 0xff123456;
  Color destination = 0xff88aacc;
  ApplicationEditor editor;
  editor.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(Brush::horizontal(source, destination));
    canvas.fill(0, 0, editor.width(), editor.height());
  };

  editor.setWindowless(10, 5);
  Screenshot screenshot = editor.takeScreenshot();
  REQUIRE(screenshot.width() == 10);
  REQUIRE(screenshot.height() == 5);
  uint8_t* data = screenshot.data();
  for (int x = 0; x < 10; ++x) {
    float t = x / 9.0f;
    Color sample = source.interpolateWith(destination, t);

    for (int y = 0; y < 5; ++y) {
      int index = (y * 10 + x) * 4;
      REQUIRE(std::abs(data[index] - (int)sample.hexRed()) <= 1);
      REQUIRE(std::abs(data[index + 1] - (int)sample.hexGreen()) <= 1);
      REQUIRE(std::abs(data[index + 2] - (int)sample.hexBlue()) <= 1);
      REQUIRE(std::abs(data[index + 3] - 0xff) <= 1);
    }
  }
}