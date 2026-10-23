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
#include "visage_graphics/post_effects.h"
#include "visage_graphics/renderer.h"
#include "visage_graphics/shape_batcher.h"
#include "visage_utils/thread_utils.h"

#include <catch2/catch_test_macros.hpp>
#include <filesystem>

using namespace visage;

static std::vector<unsigned int> drawScene(const Font& font) {
  static constexpr int kSize = 64;
  static constexpr unsigned char kImage[] = { 0, 0, 2, 0, 0, 0,  0,    0,    0,    0,    0,
                                              0, 1, 0, 1, 0, 32, 0x28, 0xff, 0x00, 0xff, 0xff };

  Canvas canvas;
  canvas.setWindowless(kSize, kSize);
  BlurPostEffect blur;
  blur.setBlurRadius(4.0f);
  Region region;
  canvas.addRegion(&region);
  region.setNeedsLayer(true);
  region.setPostEffect(&blur);
  region.setBounds(kSize / 2, 0, kSize / 2, kSize / 2);

  Path path;
  path.moveTo(4, 60);
  path.lineTo(30, 34);
  path.lineTo(30, 60);
  path.close();

  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kSize, kSize);
  canvas.setColor(0xffff8800);
  canvas.circle(2, 2, 28);
  canvas.setColor(0xff00ff00);
  canvas.fill(path);
  canvas.image(kImage, sizeof(kImage), 4, 36, 10, 10);
  canvas.setColor(0xffffffff);
  canvas.text("Hi", font, Font::Justification::kCenter, kSize / 2, kSize / 2, kSize / 2, kSize / 2);
  canvas.beginRegion(&region);
  canvas.setColor(0xff0088ff);
  canvas.fill(8, 8, 16, 16);
  canvas.endRegion();

  const Screenshot& screenshot = canvas.takeScreenshot();
  std::vector<unsigned int> pixels;
  for (int y = 0; y < kSize; ++y) {
    for (int x = 0; x < kSize; ++x)
      pixels.push_back(screenshot.sample(x, y).toARGB());
  }
  return pixels;
}

TEST_CASE("Graphics draw the same after the last canvas shuts down the renderer", "[graphics]") {
  Font font(16, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  std::vector<unsigned int> before = drawScene(font);

  REQUIRE_FALSE(Renderer::instance().initialized());
  REQUIRE(bufferPoolStagingBytes(ShapeVertex::layout()) == 0);
  REQUIRE(drawScene(font) == before);
  REQUIRE_FALSE(Renderer::instance().initialized());
}

TEST_CASE("A bloom effect keeps the renderer running until it's destroyed", "[graphics]") {
  {
    BloomPostEffect bloom;
    {
      Region region;
      Canvas canvas;
      canvas.setWindowless(32, 32);
      canvas.addRegion(&region);
      region.setNeedsLayer(true);
      region.setPostEffect(&bloom);
      region.setBounds(0, 0, 32, 32);
      canvas.beginRegion(&region);
      canvas.setColor(0xffffffff);
      canvas.fill(8, 8, 16, 16);
      canvas.endRegion();
      REQUIRE(canvas.takeScreenshot().sample(16, 16).hexRed() == 0xff);
    }

    REQUIRE(Renderer::instance().initialized());
  }

  REQUIRE_FALSE(Renderer::instance().initialized());
}

#if VISAGE_LINUX
static long numThreads() {
  std::filesystem::directory_iterator tasks("/proc/self/task");
  return std::distance(std::filesystem::begin(tasks), std::filesystem::end(tasks));
}

TEST_CASE("Destroying the last canvas stops the graphics driver's threads", "[graphics]") {
  REQUIRE_FALSE(Renderer::instance().initialized());
  long threads = numThreads();
  {
    Font font(16, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
    drawScene(font);
  }

  for (int i = 0; i < 100 && numThreads() > threads; ++i)
    Thread::sleep(10);
  REQUIRE(numThreads() == threads);
}
#endif
