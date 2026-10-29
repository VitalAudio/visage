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
#include "visage_graphics/gpu/gpu.h"
#include "visage_graphics/post_effects.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

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

static void requireSameImage(const Screenshot& expected, const Screenshot& actual) {
  REQUIRE(expected.width() == actual.width());
  REQUIRE(expected.height() == actual.height());
  int differences = 0;
  for (int y = 0; y < expected.height(); ++y) {
    for (int x = 0; x < expected.width(); ++x) {
      Color a = expected.sample(x, y);
      Color b = actual.sample(x, y);
      if (a.hexRed() != b.hexRed() || a.hexGreen() != b.hexGreen() || a.hexBlue() != b.hexBlue())
        differences++;
    }
  }
  REQUIRE(differences == 0);
}

struct GpuCounts {
  int textures = 0;
  int frame_buffers = 0;
  int buffers = 0;
  int shaders = 0;
  int programs = 0;

  static GpuCounts current() {
    const gpu::Stats* stats = gpu::getStats();
    return { stats->num_textures, stats->num_frame_buffers, stats->num_buffers, stats->num_shaders,
             stats->num_programs };
  }

  bool operator==(const GpuCounts& other) const {
    return textures == other.textures && frame_buffers == other.frame_buffers &&
           buffers == other.buffers && shaders == other.shaders && programs == other.programs;
  }
};

TEST_CASE("Canvas contents come back the same after the GPU device is lost", "[graphics]") {
  static constexpr int kWidth = 200;
  static constexpr int kHeight = 160;

  Font font(24, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  std::vector<unsigned char> image = solidTga(20, 20, 0xff40c060);
  GraphData graph(32);
  for (int i = 0; i < graph.numPoints(); ++i)
    graph[i] = 0.5f + 0.4f * ((i % 5) - 2) / 2.0f;

  Canvas canvas;
  canvas.setWindowless(kWidth, kHeight);
  canvas.setColor(0xff101820);
  canvas.fill(0, 0, kWidth, kHeight);
  canvas.setColor(Brush::linear(Color(0xffff0000), Color(0xff0000ff), Point(0, 0), Point(kWidth, 0)));
  canvas.fill(10, 10, 80, 30);
  canvas.setColor(0xffffffff);
  canvas.text("Lost", font, Font::Justification::kCenter, 100, 10, 90, 30);
  canvas.image(image.data(), image.size(), 10, 50, 20, 20);

  Path path;
  path.moveTo(50, 50);
  path.lineTo(90, 60);
  path.lineTo(60, 90);
  path.close();
  canvas.setColor(0xffffc000);
  canvas.fill(path);
  canvas.setColor(0xff00ffff);
  canvas.graphLine(graph, 100, 50, 90, 40, 2.0f);

  BlurPostEffect blur;
  blur.setBlurRadius(6.0f);
  Region blurred;
  canvas.addRegion(&blurred);
  blurred.setNeedsLayer(true);
  blurred.setPostEffect(&blur);
  blurred.setBounds(10, 100, 80, 50);
  canvas.beginRegion(&blurred);
  canvas.setColor(0xffff00ff);
  canvas.fill(20, 15, 40, 20);
  canvas.endRegion();

  BlurPostEffect backdrop;
  backdrop.setBlurRadius(4.0f);
  Region glass;
  canvas.addRegion(&glass);
  glass.setNeedsLayer(true);
  glass.setBackdropEffect(&backdrop);
  glass.setBounds(110, 100, 80, 50);
  canvas.beginRegion(&glass);
  canvas.endRegion();

  Screenshot before = canvas.takeScreenshot();
  canvas.takeScreenshot();
  GpuCounts counts = GpuCounts::current();
  uint32_t generation = gpu::contentsGeneration();

  gpu::simulateDeviceLoss();
  canvas.submit();
  REQUIRE(gpu::contentsGeneration() != generation);

  Screenshot after = canvas.takeScreenshot();
  requireSameImage(before, after);
  requireSameImage(before, canvas.takeScreenshot());
  REQUIRE(GpuCounts::current() == counts);
}

TEST_CASE("Repeated GPU device losses leave nothing behind", "[graphics]") {
  Font font(16, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  Canvas canvas;
  canvas.setWindowless(80, 40);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 80, 40);
  canvas.setColor(0xffffffff);
  canvas.text("Again", font, Font::Justification::kCenter, 0, 0, 80, 40);

  Screenshot before = canvas.takeScreenshot();
  canvas.takeScreenshot();
  GpuCounts counts = GpuCounts::current();
  for (int i = 0; i < 4; ++i) {
    INFO("loss " << i);
    gpu::simulateDeviceLoss();
    canvas.submit();
    requireSameImage(before, canvas.takeScreenshot());
  }
  REQUIRE(GpuCounts::current() == counts);
}

TEST_CASE("Canvases created after a GPU device loss render normally", "[graphics]") {
  Canvas first;
  first.setWindowless(20, 20);
  first.takeScreenshot();
  gpu::simulateDeviceLoss();
  first.submit();

  Canvas second;
  second.setWindowless(20, 20);
  second.setColor(0xff00ff00);
  second.fill(0, 0, 20, 20);
  REQUIRE(second.takeScreenshot().sample(10, 10).hexGreen() == 0xff);
}
