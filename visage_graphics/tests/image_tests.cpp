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

#include "visage_graphics/canvas.h"
#include "visage_graphics/image.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <random>

using namespace visage;
using namespace Catch;

static std::vector<unsigned char> solidTga(int width, int height, unsigned int argb) {
  std::vector<unsigned char> tga(18, 0);
  tga[2] = 2;
  tga[12] = width;
  tga[13] = width >> 8;
  tga[14] = height;
  tga[15] = height >> 8;
  tga[16] = 32;
  tga[17] = 0x28;
  for (int i = 0; i < width * height; ++i) {
    tga.push_back(argb);
    tga.push_back(argb >> 8);
    tga.push_back(argb >> 16);
    tga.push_back(argb >> 24);
  }
  return tga;
}

static void requireTransparentEdges(const Screenshot& screenshot, int x, int y, int size) {
  for (int i = 2; i < size - 2; ++i) {
    REQUIRE(screenshot.sample(x + size, y + i).hexRed() == 0);
    REQUIRE(screenshot.sample(x + i, y + size).hexRed() == 0);
  }
}

TEST_CASE("Image atlas repack leaves no old pixels around images", "[graphics]") {
  std::vector<unsigned char> red = solidTga(40, 40, 0xffff0000);
  std::vector<unsigned char> blue = solidTga(10, 10, 0xff0000ff);
  std::vector<unsigned char> green = solidTga(30, 30, 0xff00ff00);

  Canvas canvas;
  canvas.setWindowless(128, 128);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, canvas.width(), canvas.height());
  canvas.setColor(0xffffffff);
  canvas.image(red.data(), red.size(), 0, 0, 40, 40);
  canvas.image(blue.data(), blue.size(), 60, 0, 10, 10);
  canvas.submit();

  canvas.clearDrawnShapes();
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, canvas.width(), canvas.height());
  canvas.setColor(0xffffffff);
  canvas.image(blue.data(), blue.size(), 100, 0, 10, 10);
  canvas.image(green.data(), green.size(), 40.75f, 40.75f, 30, 30);
  canvas.submit();

  requireTransparentEdges(canvas.takeScreenshot(), 40, 40, 30);
}

TEST_CASE("Image atlas ignores leftover GPU memory", "[graphics]") {
  std::vector<unsigned char> red = solidTga(63, 63, 0xffff0000);
  std::vector<unsigned char> blue = solidTga(10, 10, 0xff0000ff);
  std::vector<unsigned char> green = solidTga(30, 30, 0xff00ff00);
  {
    Canvas previous;
    previous.setWindowless(128, 128);
    previous.setColor(0xffffffff);
    previous.image(red.data(), red.size(), 0, 0, 63, 63);
    previous.submit();
    previous.takeScreenshot();
  }

  Canvas canvas;
  canvas.setWindowless(128, 128);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, canvas.width(), canvas.height());
  canvas.setColor(0xffffffff);
  canvas.image(green.data(), green.size(), 40.75f, 40.75f, 30, 30);
  canvas.image(blue.data(), blue.size(), 100, 0, 10, 10);
  canvas.submit();

  requireTransparentEdges(canvas.takeScreenshot(), 40, 40, 30);
}

TEST_CASE("Canvas skips images without image data", "[graphics]") {
  static const unsigned char kNoData[] = { 0 };
  std::vector<unsigned char> empty;
  Canvas canvas;
  canvas.setWindowless(30, 10);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 30, 10);
  canvas.setColor(0xffffffff);
  canvas.image(nullptr, 0, 0, 0, 10, 10);
  canvas.image(empty.data(), static_cast<int>(empty.size()), 10, 0, 10, 10);
  canvas.image(kNoData, 0, 20, 0, 10, 10);

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(5, 5).hexRed() == 0);
  REQUIRE(screenshot.sample(15, 5).hexRed() == 0);
  REQUIRE(screenshot.sample(25, 5).hexRed() == 0);
}

TEST_CASE("Regions draw untinted images and white shapes until a color is set", "[graphics]") {
  static constexpr int kSize = 20;
  std::vector<unsigned char> magenta = solidTga(kSize, kSize, 0xffff00ff);

  Canvas canvas;
  canvas.setWindowless(3 * kSize, kSize);
  Region colored;
  Region image;
  Region shape;
  canvas.addRegion(&colored);
  canvas.addRegion(&image);
  canvas.addRegion(&shape);
  colored.setBounds(0, 0, kSize, kSize);
  image.setBounds(kSize, 0, kSize, kSize);
  shape.setBounds(2 * kSize, 0, kSize, kSize);

  for (int frame = 0; frame < 3; ++frame) {
    canvas.beginRegion(&colored);
    canvas.setColor(0xff00ff00);
    canvas.fill(0, 0, kSize, kSize);
    canvas.endRegion();

    canvas.beginRegion(&image);
    canvas.image(magenta.data(), magenta.size(), 0, 0, kSize, kSize);
    canvas.endRegion();

    canvas.beginRegion(&shape);
    canvas.fill(0, 0, kSize, kSize);
    canvas.endRegion();

    const Screenshot& screenshot = canvas.takeScreenshot();
    INFO("frame " << frame);
    Color image_pixel = screenshot.sample(kSize + kSize / 2, kSize / 2);
    REQUIRE(image_pixel.hexRed() == 0xff);
    REQUIRE(image_pixel.hexGreen() == 0);
    REQUIRE(image_pixel.hexBlue() == 0xff);
    Color shape_pixel = screenshot.sample(2 * kSize + kSize / 2, kSize / 2);
    REQUIRE(shape_pixel.hexRed() == 0xff);
    REQUIRE(shape_pixel.hexGreen() == 0xff);
    REQUIRE(shape_pixel.hexBlue() == 0xff);
  }
}
