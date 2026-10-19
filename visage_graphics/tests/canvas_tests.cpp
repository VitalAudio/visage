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
#include "visage_graphics/color.h"
#include "visage_graphics/gradient.h"
#include "visage_graphics/graphics_caches.h"
#include "visage_graphics/post_effects.h"
#include "visage_graphics/renderer.h"
#include "visage_graphics/shader.h"
#include "visage_graphics/shape_batcher.h"

#include <bgfx/bgfx.h>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace visage;
using namespace Catch;

class CanvasTestFixture {
public:
  CanvasTestFixture() { canvas_.setWindowless(200, 200); }

  Canvas& canvas() { return canvas_; }

private:
  Canvas canvas_;
};

TEST_CASE("Canvas basic setup", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("Canvas dimensions") {
    REQUIRE(canvas.width() == 200);
    REQUIRE(canvas.height() == 200);
  }

  SECTION("Canvas state initialization") {
    REQUIRE(canvas.state() != nullptr);
    REQUIRE(canvas.dpiScale() == 1.0f);
  }
}

TEST_CASE("Canvas color setting", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("setColor with unsigned int") {
    REQUIRE_NOTHROW(canvas.setColor(0xffff0000));
    REQUIRE_NOTHROW(canvas.setColor(0xff00ff00));
    REQUIRE_NOTHROW(canvas.setColor(0xff0000ff));
  }

  SECTION("setColor with Color object") {
    Color red(1.0f, 0.0f, 0.0f, 1.0f);
    Color green(0.0f, 1.0f, 0.0f, 1.0f);
    Color blue(0.0f, 0.0f, 1.0f, 1.0f);

    REQUIRE_NOTHROW(canvas.setColor(red));
    REQUIRE_NOTHROW(canvas.setColor(green));
    REQUIRE_NOTHROW(canvas.setColor(blue));
  }

  SECTION("setColor with Brush") {
    Brush solid_brush = Brush::solid(0xffff0000);
    REQUIRE_NOTHROW(canvas.setColor(solid_brush));
  }
}

TEST_CASE("Canvas basic shapes", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("Rectangle drawing") {
    canvas.setColor(0xffff0000);
    REQUIRE_NOTHROW(canvas.rectangle(10, 10, 50, 30));
    REQUIRE_NOTHROW(canvas.rectangle(0, 0, 200, 200));
    REQUIRE_NOTHROW(canvas.rectangle(100, 100, 1, 1));
  }

  SECTION("Circle drawing") {
    canvas.setColor(0xff00ff00);
    REQUIRE_NOTHROW(canvas.circle(50, 50, 30));
    REQUIRE_NOTHROW(canvas.circle(100, 100, 80));
    REQUIRE_NOTHROW(canvas.circle(150, 150, 5));
  }

  SECTION("Fill operations") {
    canvas.setColor(0xff0000ff);
    REQUIRE_NOTHROW(canvas.fill(0, 0, 200, 200));
    REQUIRE_NOTHROW(canvas.fill(25, 25, 150, 150));
    REQUIRE_NOTHROW(canvas.fill(10, 10, 5, 5));
  }
}

TEST_CASE("Canvas advanced shapes", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("Rounded rectangle") {
    canvas.setColor(0xffff00ff);
    REQUIRE_NOTHROW(canvas.roundedRectangle(10, 10, 100, 50, 5));
    REQUIRE_NOTHROW(canvas.roundedRectangle(50, 50, 80, 80, 20));
    REQUIRE_NOTHROW(canvas.roundedRectangle(100, 100, 50, 30, 1));
  }

  SECTION("Triangle drawing") {
    canvas.setColor(0xffffff00);
    REQUIRE_NOTHROW(canvas.triangle(50, 20, 80, 70, 20, 70));
    REQUIRE_NOTHROW(canvas.triangleUp(100, 100, 20));
    REQUIRE_NOTHROW(canvas.triangleDown(130, 100, 20));
    REQUIRE_NOTHROW(canvas.triangleLeft(160, 100, 20));
    REQUIRE_NOTHROW(canvas.triangleRight(190, 100, 20));
  }

  SECTION("Squircle drawing") {
    canvas.setColor(0xff00ffff);
    REQUIRE_NOTHROW(canvas.squircle(50, 50, 40));
    REQUIRE_NOTHROW(canvas.squircle(100, 100, 30, 2.0f));
    REQUIRE_NOTHROW(canvas.squircle(150, 150, 20, 8.0f));
  }

  SECTION("Diamond drawing") {
    canvas.setColor(0xffaabbcc);
    REQUIRE_NOTHROW(canvas.diamond(75, 75, 50, 5));
    REQUIRE_NOTHROW(canvas.diamond(125, 125, 40, 1));
  }
}

TEST_CASE("Canvas borders and strokes", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("Rectangle border") {
    canvas.setColor(0xffff0000);
    REQUIRE_NOTHROW(canvas.rectangleBorder(10, 10, 80, 60, 1));
    REQUIRE_NOTHROW(canvas.rectangleBorder(100, 100, 50, 50, 5));
  }

  SECTION("Ring (circle border)") {
    canvas.setColor(0xff00ff00);
    REQUIRE_NOTHROW(canvas.ring(50, 50, 40, 2));
    REQUIRE_NOTHROW(canvas.ring(150, 150, 30, 8));
  }

  SECTION("Segment (line) drawing") {
    canvas.setColor(0xff0000ff);
    REQUIRE_NOTHROW(canvas.segment(10, 50, 100, 50, 2, false));
    REQUIRE_NOTHROW(canvas.segment(50, 10, 50, 100, 2, false));
    REQUIRE_NOTHROW(canvas.segment(20, 20, 80, 80, 3, false));
    REQUIRE_NOTHROW(canvas.segment(120, 120, 180, 180, 4, true));
  }
}

TEST_CASE("Canvas state management", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("Save and restore state") {
    canvas.setColor(0xffff0000);
    REQUIRE_NOTHROW(canvas.saveState());
    canvas.setColor(0xff00ff00);
    canvas.setPosition(10, 10);
    REQUIRE_NOTHROW(canvas.restoreState());
    REQUIRE_NOTHROW(canvas.rectangle(0, 0, 10, 10));
  }

  SECTION("Position setting") {
    canvas.setColor(0xff0000ff);
    REQUIRE_NOTHROW(canvas.setPosition(50, 50));
    REQUIRE_NOTHROW(canvas.rectangle(0, 0, 20, 20));
    REQUIRE_NOTHROW(canvas.circle(10, 10, 15));
  }
}

TEST_CASE("Canvas coordinate and bounds", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("Clamp bounds") {
    canvas.setColor(0xffaabbcc);
    REQUIRE_NOTHROW(canvas.setClampBounds(25, 25, 150, 150));
    REQUIRE_NOTHROW(canvas.rectangle(0, 0, 200, 200));
    REQUIRE_NOTHROW(canvas.circle(100, 100, 80));
    REQUIRE_FALSE(canvas.totallyClamped());
  }

  SECTION("Trim clamp bounds") {
    canvas.setColor(0xffdddddd);
    canvas.setClampBounds(0, 0, 200, 200);
    REQUIRE_NOTHROW(canvas.trimClampBounds(50, 50, 100, 100));
    REQUIRE_NOTHROW(canvas.rectangle(0, 0, 200, 200));
  }
}

TEST_CASE("Canvas visual validation", "[graphics]") {
  static constexpr int kTestWidth = 200;
  static constexpr int kTestHeight = 200;

  SECTION("Simple red rectangle") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setColor(0xffff0000);
    canvas.rectangle(50, 50, 100, 100);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color center = screenshot.sample(100, 100);
    REQUIRE(center.hexRed() == 0xff);
    REQUIRE(center.hexGreen() == 0x00);
    REQUIRE(center.hexBlue() == 0x00);
    Color top_edge = screenshot.sample(100, 50);
    REQUIRE(top_edge.hexRed() == 0xff);
    Color bottom_edge = screenshot.sample(100, 149);
    REQUIRE(bottom_edge.hexRed() == 0xff);
    Color left_edge = screenshot.sample(50, 100);
    REQUIRE(left_edge.hexRed() == 0xff);
    Color right_edge = screenshot.sample(149, 100);
    REQUIRE(right_edge.hexRed() == 0xff);

    Color background = screenshot.sample(25, 25);
    REQUIRE(background.hexRed() == 0x00);
    REQUIRE(background.hexGreen() == 0x00);
    REQUIRE(background.hexBlue() == 0x00);
  }

  SECTION("Circle drawing validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setColor(0xff00ff00);
    canvas.circle(25, 25, 50);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color center = screenshot.sample(50, 50);
    REQUIRE(center.hexGreen() == 0xff);
    REQUIRE(center.hexRed() == 0x00);
    REQUIRE(center.hexBlue() == 0x00);

    Color inside = screenshot.sample(40, 50);
    REQUIRE(inside.hexGreen() == 0xff);

    Color outside = screenshot.sample(100, 100);
    REQUIRE(outside.hexGreen() == 0x00);
    REQUIRE(outside.hexRed() == 0x00);
    REQUIRE(outside.hexBlue() == 0x00);
  }

  SECTION("Fill operation validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff0000ff);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color top_left = screenshot.sample(0, 0);
    REQUIRE(top_left.hexBlue() == 0xff);
    Color top_right = screenshot.sample(199, 0);
    REQUIRE(top_right.hexBlue() == 0xff);
    Color bottom_left = screenshot.sample(0, 199);
    REQUIRE(bottom_left.hexBlue() == 0xff);
    Color bottom_right = screenshot.sample(199, 199);
    REQUIRE(bottom_right.hexBlue() == 0xff);

    Color center = screenshot.sample(100, 100);
    REQUIRE(center.hexBlue() == 0xff);
    REQUIRE(center.hexRed() == 0x00);
    REQUIRE(center.hexGreen() == 0x00);
  }

  SECTION("Multiple shapes composition") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setColor(0xffff0000);
    canvas.rectangle(10, 10, 50, 50);

    canvas.setColor(0xff00ff00);
    canvas.circle(70, 70, 60);

    canvas.setColor(0xff0000ff);
    canvas.triangle(150, 20, 180, 70, 120, 70);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color red_sample = screenshot.sample(35, 35);
    REQUIRE(red_sample.hexRed() == 0xff);
    REQUIRE(red_sample.hexGreen() == 0x00);

    Color green_sample = screenshot.sample(100, 100);
    REQUIRE(green_sample.hexGreen() == 0xff);
    REQUIRE(green_sample.hexRed() == 0x00);

    Color blue_sample = screenshot.sample(150, 45);
    REQUIRE(blue_sample.hexBlue() == 0xff);
    REQUIRE(blue_sample.hexRed() == 0x00);

    Color background = screenshot.sample(5, 5);
    REQUIRE(background.hexRed() == 0x00);
    REQUIRE(background.hexGreen() == 0x00);
    REQUIRE(background.hexBlue() == 0x00);
  }

  SECTION("Clipping validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setClampBounds(50, 50, 100, 100);

    canvas.setColor(0xffff0000);
    canvas.rectangle(0, 0, 200, 200);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color inside = screenshot.sample(100, 100);
    REQUIRE(inside.hexRed() == 0xff);

    Color outside_left = screenshot.sample(25, 100);
    REQUIRE(outside_left.hexRed() == 0x00);
    Color outside_right = screenshot.sample(175, 100);
    REQUIRE(outside_right.hexRed() == 0x00);
    Color outside_top = screenshot.sample(100, 25);
    REQUIRE(outside_top.hexRed() == 0x00);
    Color outside_bottom = screenshot.sample(100, 175);
    REQUIRE(outside_bottom.hexRed() == 0x00);
  }
}

TEST_CASE("Canvas advanced shape validation", "[graphics]") {
  static constexpr int kTestWidth = 200;
  static constexpr int kTestHeight = 200;

  SECTION("Rounded rectangle validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setColor(0xffffff00);
    canvas.roundedRectangle(50, 50, 100, 100, 10);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color center = screenshot.sample(100, 100);
    REQUIRE(center.hexRed() == 0xff);
    REQUIRE(center.hexGreen() == 0xff);
    REQUIRE(center.hexBlue() == 0x00);

    Color corner = screenshot.sample(50, 50);
    REQUIRE(corner.hexRed() == 0x00);

    Color edge = screenshot.sample(100, 50);
    REQUIRE(edge.hexRed() >= 0xfe);
    REQUIRE(edge.hexGreen() >= 0xfe);
  }

  SECTION("Rectangle border validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setColor(0xff00ffff);
    canvas.rectangleBorder(50, 50, 100, 100, 5);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color top_border = screenshot.sample(100, 52);
    REQUIRE(top_border.hexGreen() == 0xff);
    REQUIRE(top_border.hexBlue() == 0xff);
    REQUIRE(top_border.hexRed() == 0x00);

    Color left_border = screenshot.sample(52, 100);
    REQUIRE(left_border.hexGreen() == 0xff);
    REQUIRE(left_border.hexBlue() == 0xff);

    Color interior = screenshot.sample(100, 100);
    REQUIRE(interior.hexRed() == 0x00);
    REQUIRE(interior.hexGreen() == 0x00);
    REQUIRE(interior.hexBlue() == 0x00);
  }

  SECTION("Line segment validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setColor(0xffff00ff);
    canvas.segment(50, 100, 150, 100, 3, false);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color line_start = screenshot.sample(50, 100);
    REQUIRE(line_start.hexRed() >= 0xfe);
    REQUIRE(line_start.hexBlue() >= 0xfe);
    REQUIRE(line_start.hexGreen() == 0x00);

    Color line_middle = screenshot.sample(100, 100);
    REQUIRE(line_middle.hexRed() == 0xff);
    REQUIRE(line_middle.hexBlue() == 0xff);

    Color line_end = screenshot.sample(149, 100);
    REQUIRE(line_end.hexRed() >= 0xfe);
    REQUIRE(line_end.hexBlue() >= 0xfe);

    Color off_line = screenshot.sample(100, 90);
    REQUIRE(off_line.hexRed() == 0x00);
    REQUIRE(off_line.hexGreen() == 0x00);
    REQUIRE(off_line.hexBlue() == 0x00);
  }
}

TEST_CASE("Canvas state and position validation", "[graphics]") {
  static constexpr int kTestWidth = 200;
  static constexpr int kTestHeight = 200;

  SECTION("Position offset validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    // Set position offset
    canvas.setPosition(50, 50);

    // Draw rectangle at (0,0) which should appear at (50,50)
    canvas.setColor(0xffff0000);
    canvas.rectangle(0, 0, 50, 50);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color offset_center = screenshot.sample(75, 75);
    REQUIRE(offset_center.hexRed() == 0xff);
    REQUIRE(offset_center.hexGreen() == 0x00);
    REQUIRE(offset_center.hexBlue() == 0x00);

    Color original_pos = screenshot.sample(25, 25);
    REQUIRE(original_pos.hexRed() == 0x00);
    REQUIRE(original_pos.hexGreen() == 0x00);
    REQUIRE(original_pos.hexBlue() == 0x00);
  }

  SECTION("Save/restore state validation") {
    Canvas canvas;
    canvas.setWindowless(kTestWidth, kTestHeight);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());

    canvas.setPosition(25, 25);
    canvas.saveState();

    canvas.setPosition(75, 75);

    canvas.restoreState();

    canvas.setColor(0xffff0000);
    canvas.rectangle(0, 0, 30, 30);

    canvas.submit();
    const Screenshot& screenshot = canvas.takeScreenshot();

    Color red_rect = screenshot.sample(40, 40);
    REQUIRE(red_rect.hexRed() == 0xff);
    REQUIRE(red_rect.hexGreen() == 0x00);
    REQUIRE(red_rect.hexBlue() == 0x00);

    Color wrong_pos = screenshot.sample(90, 90);
    REQUIRE(wrong_pos.hexRed() == 0x00);  // Should be background (black)
  }
}

TEST_CASE("Canvas edge cases", "[graphics]") {
  CanvasTestFixture fixture;
  Canvas& canvas = fixture.canvas();

  SECTION("Zero dimensions") {
    canvas.setColor(0xffff0000);

    REQUIRE_NOTHROW(canvas.rectangle(50, 50, 0, 50));
    REQUIRE_NOTHROW(canvas.rectangle(50, 50, 50, 0));
    REQUIRE_NOTHROW(canvas.rectangle(50, 50, 0, 0));

    REQUIRE_NOTHROW(canvas.circle(50, 50, 0));
  }

  SECTION("Negative positions") {
    canvas.setColor(0xff00ff00);

    REQUIRE_NOTHROW(canvas.rectangle(-10, -10, 50, 50));
    REQUIRE_NOTHROW(canvas.circle(-20, -20, 30));
  }

  SECTION("Large dimensions") {
    canvas.setColor(0xff0000ff);

    REQUIRE_NOTHROW(canvas.rectangle(0, 0, 1000, 1000));
    REQUIRE_NOTHROW(canvas.circle(100, 100, 500));
  }
}

TEST_CASE("Canvas gradient atlas growth", "[graphics]") {
  static constexpr int kTestWidth = 200;
  static constexpr int kTestHeight = 200;
  static constexpr int kNumGradients = 100;
  static constexpr int kRowHeight = kTestHeight / kNumGradients;

  auto row_color = [](int i) -> unsigned int {
    return 0xff000000 | ((i * 2) << 16) | (0xff - i * 2);
  };
  auto row_brush = [&row_color](int i) {
    Gradient gradient(row_color(i), row_color(i));
    gradient.setRepeat(true);
    return Brush::horizontal(gradient);
  };

  Canvas canvas;
  canvas.setWindowless(kTestWidth, kTestHeight);

  canvas.setColor(row_brush(0));
  canvas.fill(0, 0, kTestWidth, kTestHeight);
  canvas.submit();

  canvas.clearDrawnShapes();
  for (int i = 0; i < kNumGradients; ++i) {
    canvas.setColor(row_brush(i));
    canvas.fill(0, i * kRowHeight, kTestWidth, kRowHeight);
  }
  canvas.submit();
  const Screenshot& screenshot = canvas.takeScreenshot();

  for (int i = 0; i < kNumGradients; ++i) {
    Color expected = row_color(i);
    Color sample = screenshot.sample(kTestWidth / 2, i * kRowHeight + kRowHeight / 2);
    INFO("Gradient row " << i);
    REQUIRE(std::abs(static_cast<int>(sample.hexRed()) - static_cast<int>(expected.hexRed())) <= 1);
    REQUIRE(std::abs(static_cast<int>(sample.hexBlue()) - static_cast<int>(expected.hexBlue())) <= 1);
  }
}

TEST_CASE("Recycled shape batches keep their shape type", "[graphics]") {
  int freed_owner = 0;
  ShapeBatcher batcher;
  Fill fill({}, nullptr, 0, 0, 10, 10);
  fill.batch_id = &freed_owner;
  batcher.addShape(fill);
  batcher.clear();

  Rectangle rectangle({}, nullptr, 0, 0, 10, 10);
  rectangle.batch_id = &freed_owner;
  batcher.addShape(rectangle);
  REQUIRE(dynamic_cast<ShapeBatch<Rectangle>*>(batcher.batchAtIndex(0)) != nullptr);
}

TEST_CASE("Windowless canvas resizes don't leak textures", "[graphics]") {
  Canvas canvas;
  auto screenshot = [&canvas](int size) {
    canvas.setWindowless(size, size);
    canvas.setColor(0xffff0000);
    canvas.fill(0, 0, size, size);
    canvas.takeScreenshot();
  };

  screenshot(16);
  int textures = bgfx::getStats()->numTextures;
  for (int i = 0; i < 8; ++i)
    screenshot(17 + i);
  REQUIRE(bgfx::getStats()->numTextures <= textures);
}

TEST_CASE("Destroying blur and bloom effects doesn't leak vertex buffers", "[graphics]") {
  auto draw_effects = [] {
    Canvas canvas;
    canvas.setWindowless(64, 32);
    BlurPostEffect blur;
    blur.setBlurRadius(4.0f);
    BloomPostEffect bloom;
    Region blurred;
    Region bloomed;
    canvas.addRegion(&blurred);
    canvas.addRegion(&bloomed);
    blurred.setNeedsLayer(true);
    blurred.setPostEffect(&blur);
    blurred.setBounds(0, 0, 32, 32);
    bloomed.setNeedsLayer(true);
    bloomed.setPostEffect(&bloom);
    bloomed.setBounds(32, 0, 32, 32);

    for (Region* region : { &blurred, &bloomed }) {
      canvas.beginRegion(region);
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, region->width(), region->height());
      canvas.endRegion();
    }
    canvas.takeScreenshot();
  };

  draw_effects();
  bgfx::frame();
  int vertex_buffers = bgfx::getStats()->numVertexBuffers;
  for (int i = 0; i < 4; ++i)
    draw_effects();
  bgfx::frame();
  REQUIRE(bgfx::getStats()->numVertexBuffers <= vertex_buffers);
}

TEST_CASE("Canvas screenshot shows the current frame", "[graphics]") {
  static constexpr int kSize = 16;

  Canvas canvas;
  canvas.setWindowless(kSize, kSize);

  SECTION("Without submitting first") {
    canvas.setColor(0xffff0000);
    canvas.fill(0, 0, kSize, kSize);
    const Screenshot& screenshot = canvas.takeScreenshot();
    REQUIRE(screenshot.sample(kSize / 2, kSize / 2).hexRed() == 0xff);
  }

  SECTION("After changing what was drawn") {
    canvas.setColor(0xffff0000);
    canvas.fill(0, 0, kSize, kSize);
    canvas.submit();

    canvas.clearDrawnShapes();
    canvas.setColor(0xff0000ff);
    canvas.fill(0, 0, kSize, kSize);
    const Screenshot& screenshot = canvas.takeScreenshot();
    REQUIRE(screenshot.sample(kSize / 2, kSize / 2).hexRed() == 0x00);
    REQUIRE(screenshot.sample(kSize / 2, kSize / 2).hexBlue() == 0xff);
  }
}

TEST_CASE("Resizing a post effect region doesn't render extra frames", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 64);
  BlurPostEffect blur;
  blur.setBlurRadius(10.0f);
  Region region;
  canvas.addRegion(&region);
  region.setNeedsLayer(true);
  region.setPostEffect(&blur);
  region.setBounds(0, 0, 32, 32);

  auto frames = [&] {
    uint32_t start = bgfx::frame();
    region.clear();
    region.invalidate();
    canvas.beginRegion(&region);
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, region.width(), region.height());
    canvas.endRegion();
    canvas.submit();
    return bgfx::frame() - start - 1;
  };

  frames();
  frames();
  uint32_t steady = frames();
  region.setBounds(0, 0, 48, 48);
  REQUIRE(frames() <= steady);
  REQUIRE(canvas.takeScreenshot().sample(24, 24).hexRed() == 0xff);
}

TEST_CASE("Translucent frames don't accumulate on the composite", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(16, 16);

  auto alpha = [&canvas] {
    canvas.clearDrawnShapes();
    canvas.setColor(0x80ffffff);
    canvas.fill(0, 0, 16, 16);
    return canvas.takeScreenshot().sample(8, 8).hexAlpha();
  };

  unsigned int first = alpha();
  REQUIRE(first > 0x70);
  REQUIRE(first < 0x90);
  for (int i = 0; i < 3; ++i)
    REQUIRE(alpha() == first);
}

TEST_CASE("Blur radius matches the blurred spread", "[graphics]") {
  static constexpr int kWidth = 512;
  static constexpr int kHeight = 16;
  static constexpr int kEdge = kWidth / 2;

  Canvas canvas;
  canvas.setWindowless(kWidth, kHeight);
  BlurPostEffect blur;
  Region region;
  canvas.addRegion(&region);
  region.setNeedsLayer(true);
  region.setPostEffect(&blur);
  region.setBounds(0, 0, kWidth, kHeight);

  auto measured_sigma = [&](float radius) {
    blur.setBlurRadius(radius);
    region.clear();
    region.invalidate();
    canvas.beginRegion(&region);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, kWidth, kHeight);
    canvas.setColor(0xffffffff);
    canvas.fill(kEdge, 0, kWidth - kEdge, kHeight);
    canvas.endRegion();
    const Screenshot& screenshot = canvas.takeScreenshot();

    auto crossing = [&screenshot](float level) {
      float last = 0.0f;
      for (int x = 0; x < kWidth; ++x) {
        float value = screenshot.sample(x, kHeight / 2).hexRed() / 255.0f;
        if (x > 0 && last < level && value >= level)
          return x - 0.5f + (level - last) / (value - last);
        last = value;
      }
      return 0.0f;
    };
    return (crossing(0.8413f) - crossing(0.1587f)) / 2.0f;
  };

  for (float radius : { 6.0f, 16.0f, 40.0f })
    REQUIRE(measured_sigma(radius) == Approx(radius).epsilon(0.05f));

  REQUIRE(measured_sigma(8.1f) / measured_sigma(7.9f) == Approx(8.1f / 7.9f).epsilon(0.02f));
}

TEST_CASE("Layered region with a backdrop child keeps its other children", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect blur;
  blur.setBlurRadius(2.0f);
  Region panel;
  Region child;
  Region glass;
  canvas.addRegion(&panel);
  panel.setNeedsLayer(true);
  panel.setBounds(0, 0, 64, 32);
  panel.addRegion(&child);
  child.setBounds(0, 0, 16, 16);
  panel.addRegion(&glass);
  glass.setNeedsLayer(true);
  glass.setBackdropEffect(&blur);
  glass.setBounds(40, 0, 16, 16);

  auto fill = [&canvas](Region& region, unsigned int color) {
    canvas.beginRegion(&region);
    canvas.setColor(color);
    canvas.fill(0, 0, region.width(), region.height());
    canvas.endRegion();
  };

  fill(panel, 0xffff0000);
  fill(child, 0xff00ff00);
  canvas.beginRegion(&glass);
  canvas.endRegion();
  REQUIRE(canvas.takeScreenshot().sample(8, 8).hexGreen() == 0xff);
  REQUIRE(canvas.screenshot().sample(8, 8).hexRed() == 0x00);

  child.invalidate();
  fill(child, 0xff00ff00);
  REQUIRE(canvas.takeScreenshot().sample(8, 8).hexGreen() == 0xff);
  REQUIRE(canvas.screenshot().sample(8, 8).hexRed() == 0x00);
}

TEST_CASE("Backdrop follows its region when an ancestor moves", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect blur;
  blur.setBlurRadius(2.0f);
  Region background;
  Region parent;
  Region glass;
  canvas.addRegion(&background);
  background.setBounds(0, 0, 64, 32);
  canvas.addRegion(&parent);
  parent.setBounds(0, 0, 32, 32);
  parent.addRegion(&glass);
  glass.setNeedsLayer(true);
  glass.setBackdropEffect(&blur);
  glass.setBounds(8, 8, 16, 16);

  canvas.beginRegion(&background);
  canvas.setColor(0xffff0000);
  canvas.fill(0, 0, 32, 32);
  canvas.setColor(0xff0000ff);
  canvas.fill(32, 0, 32, 32);
  canvas.endRegion();
  canvas.beginRegion(&glass);
  canvas.endRegion();
  REQUIRE(canvas.takeScreenshot().sample(16, 16).hexRed() == 0xff);

  parent.setBounds(32, 0, 32, 32);
  REQUIRE(canvas.takeScreenshot().sample(48, 16).hexBlue() == 0xff);
  REQUIRE(canvas.screenshot().sample(48, 16).hexRed() == 0x00);
}

TEST_CASE("On-top region draws above a backdrop sibling added after it", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect blur;
  blur.setBlurRadius(2.0f);
  Region group;
  Region popup;
  Region glass;
  canvas.addRegion(&group);
  group.setBounds(0, 0, 64, 32);
  group.addRegion(&popup);
  popup.setOnTop(true);
  popup.setBounds(16, 8, 32, 16);
  group.addRegion(&glass);
  glass.setNeedsLayer(true);
  glass.setBackdropEffect(&blur);
  glass.setBounds(0, 0, 64, 32);

  canvas.beginRegion(&popup);
  canvas.setColor(0xff00ff00);
  canvas.fill(0, 0, popup.width(), popup.height());
  canvas.endRegion();
  canvas.beginRegion(&glass);
  canvas.setColor(0xffff0000);
  canvas.fill(0, 0, glass.width(), glass.height());
  canvas.endRegion();
  REQUIRE(canvas.takeScreenshot().sample(32, 16).hexGreen() == 0xff);
  REQUIRE(canvas.screenshot().sample(32, 16).hexRed() == 0x00);
}

TEST_CASE("Layered region keeps its content when an ancestor changes layering", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  Region group;
  Region group_child1;
  Region group_child2;
  Region panel;
  Region child;
  canvas.addRegion(&group);
  group.setNeedsLayer(true);
  group.setBounds(0, 0, 32, 32);
  group.addRegion(&group_child1);
  group_child1.setNeedsLayer(true);
  group_child1.setBounds(0, 0, 4, 4);
  group.addRegion(&group_child2);
  group_child2.setNeedsLayer(true);
  group_child2.setBounds(8, 0, 4, 4);
  canvas.addRegion(&panel);
  panel.setBounds(32, 0, 32, 32);
  panel.addRegion(&child);
  child.setNeedsLayer(true);
  child.setBounds(4, 4, 8, 8);

  auto fill = [&canvas](Region& region, unsigned int color) {
    canvas.beginRegion(&region);
    canvas.setColor(color);
    canvas.fill(0, 0, region.width(), region.height());
    canvas.endRegion();
  };

  fill(group, 0xff404040);
  fill(group_child1, 0xffffffff);
  fill(group_child2, 0xffffffff);
  fill(panel, 0xff0000ff);
  fill(child, 0xff00ff00);
  REQUIRE(canvas.takeScreenshot().sample(40, 8).hexGreen() == 0xff);

  panel.setNeedsLayer(true);
  REQUIRE(canvas.takeScreenshot().sample(40, 8).hexGreen() == 0xff);

  panel.setNeedsLayer(false);
  REQUIRE(canvas.takeScreenshot().sample(40, 8).hexGreen() == 0xff);
}

TEST_CASE("Cached region next to a bloom region keeps its brightness", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BloomPostEffect bloom;
  Region glowing;
  Region cached;
  canvas.addRegion(&glowing);
  glowing.setNeedsLayer(true);
  glowing.setPostEffect(&bloom);
  glowing.setBounds(0, 0, 32, 32);
  canvas.addRegion(&cached);
  cached.setNeedsLayer(true);
  cached.setBounds(32, 0, 32, 32);

  canvas.beginRegion(&glowing);
  canvas.setColor(0xff202020);
  canvas.fill(0, 0, glowing.width(), glowing.height());
  canvas.endRegion();
  canvas.beginRegion(&cached);
  canvas.setColor(0xffffffff);
  canvas.fill(0, 0, cached.width(), cached.height());
  canvas.endRegion();
  REQUIRE(canvas.takeScreenshot().sample(48, 16).hexRed() == 0xff);
}

TEST_CASE("HDR layer edges ignore leftover GPU memory", "[graphics]") {
  {
    Canvas previous;
    previous.setWindowless(128, 128);
    previous.setColor(0xffffffff);
    previous.fill(0, 0, previous.width(), previous.height());
    previous.takeScreenshot();
  }

  Canvas canvas;
  canvas.setWindowless(128, 64);
  BloomPostEffect bloom;
  bloom.setTranslucent(true);
  bloom.setBloomIntensity(0.0f);
  Region glowing;
  Region plain;
  canvas.addRegion(&glowing);
  glowing.setNeedsLayer(true);
  glowing.setPostEffect(&bloom);
  glowing.setBounds(4, 4, 30, 30);
  canvas.addRegion(&plain);
  plain.setNeedsLayer(true);
  plain.setBounds(70, 4, 30, 30);

  canvas.setColor(0xff808080);
  canvas.fill(0, 0, canvas.width(), canvas.height());
  for (Region* region : { &glowing, &plain }) {
    canvas.beginRegion(region);
    canvas.setColor(0xffff0000);
    canvas.fill(0, 0, region->width(), region->height());
    canvas.endRegion();
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (Region* region : { &glowing, &plain }) {
    for (int i = 0; i < region->width(); i += 7) {
      REQUIRE(screenshot.sample(region->x() + region->width() - 1, region->y() + i).toARGB() == 0xffff0000);
      REQUIRE(screenshot.sample(region->x() + i, region->y() + region->height() - 1).toARGB() == 0xffff0000);
    }
  }
}

TEST_CASE("Cached region keeps its content when a sibling adds a bloom effect", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BloomPostEffect bloom;
  Region glowing;
  Region cached;
  canvas.addRegion(&glowing);
  glowing.setNeedsLayer(true);
  glowing.setBounds(0, 0, 32, 32);
  canvas.addRegion(&cached);
  cached.setNeedsLayer(true);
  cached.setBounds(32, 0, 32, 32);

  canvas.beginRegion(&glowing);
  canvas.setColor(0xff202020);
  canvas.fill(0, 0, glowing.width(), glowing.height());
  canvas.endRegion();
  canvas.beginRegion(&cached);
  canvas.setColor(0xffffffff);
  canvas.fill(0, 0, cached.width(), cached.height());
  canvas.endRegion();
  REQUIRE(canvas.takeScreenshot().sample(48, 16).hexRed() == 0xff);

  glowing.setPostEffect(&bloom);
  REQUIRE(canvas.takeScreenshot().sample(48, 16).hexRed() == 0xff);
}

TEST_CASE("Backdrop only blurs the content under it", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(160, 32);
  BlurPostEffect blur;
  blur.setBlurRadius(20.0f);
  Region background;
  Region neighbor;
  Region glass;
  canvas.addRegion(&background);
  background.setBounds(0, 0, 160, 32);
  canvas.addRegion(&neighbor);
  neighbor.setBounds(80, 0, 18, 32);
  canvas.addRegion(&glass);
  glass.setNeedsLayer(true);
  glass.setBackdropEffect(&blur);
  glass.setBounds(100, 0, 60, 32);

  auto fill = [&canvas](Region& region, unsigned int color) {
    canvas.beginRegion(&region);
    canvas.setColor(color);
    canvas.fill(0, 0, region.width(), region.height());
    canvas.endRegion();
  };

  fill(background, 0xff000000);
  fill(neighbor, 0xffffffff);
  canvas.beginRegion(&glass);
  canvas.endRegion();
  REQUIRE(canvas.takeScreenshot().sample(101, 16).hexRed() < 0x10);

  neighbor.setBounds(100, 0, 18, 32);
  REQUIRE(canvas.takeScreenshot().sample(125, 16).hexRed() > 0x30);
}

TEST_CASE("Layered region with a backdrop child draws translucent content once", "[graphics]") {
  for (bool with_background : { false, true }) {
    Canvas canvas;
    canvas.setWindowless(64, 32);
    BlurPostEffect blur;
    blur.setBlurRadius(2.0f);
    Region background;
    Region panel;
    Region child;
    Region glass;
    if (with_background) {
      canvas.addRegion(&background);
      background.setBounds(0, 0, 64, 32);
    }
    canvas.addRegion(&panel);
    panel.setNeedsLayer(true);
    panel.setBounds(0, 0, 64, 32);
    panel.addRegion(&child);
    child.setBounds(0, 0, 16, 16);
    panel.addRegion(&glass);
    glass.setNeedsLayer(true);
    glass.setBackdropEffect(&blur);
    glass.setBounds(40, 0, 16, 16);

    canvas.beginRegion(&background);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, 64, 32);
    canvas.endRegion();
    canvas.beginRegion(&child);
    canvas.setColor(0x80ffffff);
    canvas.fill(0, 0, 16, 16);
    canvas.endRegion();
    canvas.beginRegion(&glass);
    canvas.endRegion();

    unsigned int red = canvas.takeScreenshot().sample(8, 8).hexRed();
    REQUIRE(red > 0x78);
    REQUIRE(red < 0x88);
  }
}

TEST_CASE("Backdrop inside layered regions blurs their content behind it", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect inner_blur;
  inner_blur.setBlurRadius(2.0f);
  BlurPostEffect outer_blur;
  outer_blur.setBlurRadius(2.0f);
  Region outer;
  Region inner;
  Region inner_glass;
  Region outer_glass;
  canvas.addRegion(&outer);
  outer.setNeedsLayer(true);
  outer.setBounds(0, 0, 64, 32);
  outer.addRegion(&inner);
  inner.setNeedsLayer(true);
  inner.setBounds(0, 0, 32, 32);
  inner.addRegion(&inner_glass);
  inner_glass.setNeedsLayer(true);
  inner_glass.setBackdropEffect(&inner_blur);
  inner_glass.setBounds(8, 8, 16, 16);
  outer.addRegion(&outer_glass);
  outer_glass.setNeedsLayer(true);
  outer_glass.setBackdropEffect(&outer_blur);
  outer_glass.setBounds(40, 8, 16, 16);

  auto split = [&canvas](Region& region, int edge, unsigned int left, unsigned int right) {
    canvas.beginRegion(&region);
    canvas.setColor(left);
    canvas.fill(0, 0, edge, region.height());
    canvas.setColor(right);
    canvas.fill(edge, 0, region.width() - edge, region.height());
    canvas.endRegion();
  };

  split(outer, 48, 0xff0000ff, 0xff00ff00);
  split(inner, 16, 0xffff0000, 0xff0000ff);
  canvas.beginRegion(&inner_glass);
  canvas.endRegion();
  canvas.beginRegion(&outer_glass);
  canvas.endRegion();

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(15, 16).hexRed() < 0xe0);
  REQUIRE(screenshot.sample(15, 16).hexBlue() > 0x20);
  REQUIRE(screenshot.sample(15, 16).hexGreen() == 0x00);
  REQUIRE(screenshot.sample(47, 16).hexBlue() < 0xe0);
  REQUIRE(screenshot.sample(47, 16).hexGreen() > 0x20);
  REQUIRE(screenshot.sample(47, 16).hexRed() == 0x00);
}

TEST_CASE("Backdrop sees through cached ancestors but not translucent ones", "[graphics]") {
  for (bool translucent : { false, true }) {
    Canvas canvas;
    canvas.setWindowless(64, 32);
    BlurPostEffect blur;
    blur.setBlurRadius(4.0f);
    Region background;
    Region group;
    Region glass;
    canvas.addRegion(&background);
    background.setBounds(0, 0, 64, 32);
    canvas.addRegion(&group);
    group.setNeedsLayer(true);
    group.setBounds(0, 0, 64, 32);
    if (translucent)
      group.setAlpha(0.5f);
    group.addRegion(&glass);
    glass.setNeedsLayer(true);
    glass.setBackdropEffect(&blur);
    glass.setBounds(16, 8, 32, 16);

    canvas.beginRegion(&background);
    canvas.setColor(0xffff0000);
    canvas.fill(0, 0, 32, 32);
    canvas.setColor(0xff0000ff);
    canvas.fill(32, 0, 32, 32);
    canvas.endRegion();
    canvas.beginRegion(&group);
    canvas.endRegion();
    canvas.beginRegion(&glass);
    canvas.endRegion();

    Color edge = canvas.takeScreenshot().sample(31, 16);
    if (translucent) {
      REQUIRE(edge.hexRed() > 0xf0);
      REQUIRE(edge.hexBlue() < 0x10);
    }
    else {
      REQUIRE(edge.hexRed() < 0xe0);
      REQUIRE(edge.hexBlue() > 0x20);
    }
  }
}

TEST_CASE("Backdrop passes run when an earlier backdrop region draws nothing", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect hidden_blur;
  hidden_blur.setBlurRadius(2.0f);
  BlurPostEffect blur;
  blur.setBlurRadius(2.0f);
  Region background;
  Region hidden;
  Region glass;
  canvas.addRegion(&background);
  background.setBounds(0, 0, 64, 32);
  canvas.addRegion(&hidden);
  hidden.setBounds(0, 0, 16, 16);
  hidden.setBackdropEffect(&hidden_blur);
  hidden.setVisible(false);
  canvas.addRegion(&glass);
  glass.setNeedsLayer(true);
  glass.setBackdropEffect(&blur);
  glass.setBounds(32, 0, 32, 32);

  canvas.beginRegion(&background);
  canvas.setColor(0xff0000ff);
  canvas.fill(0, 0, 64, 32);
  canvas.endRegion();
  canvas.beginRegion(&glass);
  canvas.setColor(0xffff0000);
  canvas.fill(8, 8, 16, 16);
  canvas.endRegion();

  REQUIRE(canvas.takeScreenshot().sample(48, 16).hexRed() == 0xff);
}

TEST_CASE("Layered region whose backdrop children draw nothing still composites", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect outer_blur;
  outer_blur.setBlurRadius(2.0f);
  BlurPostEffect inner_blur;
  inner_blur.setBlurRadius(2.0f);
  Region container;
  Region hidden;
  Region inner;
  canvas.addRegion(&container);
  container.setNeedsLayer(true);
  container.setBounds(0, 0, 64, 32);
  container.addRegion(&hidden);
  hidden.setBounds(8, 8, 32, 16);
  hidden.setBackdropEffect(&outer_blur);
  hidden.setVisible(false);
  hidden.addRegion(&inner);
  inner.setBounds(0, 0, 16, 16);
  inner.setBackdropEffect(&inner_blur);

  canvas.beginRegion(&container);
  canvas.setColor(0xffff0000);
  canvas.fill(0, 0, 64, 32);
  canvas.endRegion();

  REQUIRE(canvas.takeScreenshot().sample(56, 16).hexRed() == 0xff);
}

TEST_CASE("Layer blend modes handle translucent content", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(128, 32);
  Region background;
  Region add;
  Region subtract;
  Region multiply;
  Region group;
  Region grouped_multiply;
  canvas.addRegion(&background);
  background.setBounds(0, 0, 128, 32);
  for (Region* region : { &add, &subtract, &multiply, &group }) {
    canvas.addRegion(region);
    region->setNeedsLayer(true);
  }
  add.setBounds(0, 0, 32, 32);
  add.setLayerBlendMode(BlendMode::Add);
  add.setAlpha(0.5f);
  subtract.setBounds(32, 0, 32, 32);
  subtract.setLayerBlendMode(BlendMode::Sub);
  subtract.setAlpha(0.5f);
  multiply.setBounds(64, 0, 32, 32);
  multiply.setLayerBlendMode(BlendMode::Mult);
  group.setBounds(96, 0, 32, 32);
  group.addRegion(&grouped_multiply);
  grouped_multiply.setNeedsLayer(true);
  grouped_multiply.setLayerBlendMode(BlendMode::Mult);
  grouped_multiply.setBounds(0, 0, 32, 32);

  auto fill = [&canvas](Region& region, unsigned int color, int width) {
    canvas.beginRegion(&region);
    canvas.setColor(color);
    canvas.fill(0, 0, width, region.height());
    canvas.endRegion();
  };

  fill(background, 0xff406080, 128);
  fill(add, 0xff806040, 32);
  fill(subtract, 0xff202020, 32);
  fill(multiply, 0xff808080, 16);
  fill(grouped_multiply, 0xff808080, 32);

  const Screenshot& screenshot = canvas.takeScreenshot();
  auto expect = [&screenshot](int x, unsigned int color) {
    Color sample = screenshot.sample(x, 16);
    REQUIRE(std::abs(static_cast<int>(sample.hexRed()) - static_cast<int>((color >> 16) & 0xff)) <= 2);
    REQUIRE(std::abs(static_cast<int>(sample.hexGreen()) - static_cast<int>((color >> 8) & 0xff)) <= 2);
    REQUIRE(std::abs(static_cast<int>(sample.hexBlue()) - static_cast<int>(color & 0xff)) <= 2);
  };
  expect(16, 0x8090a0);
  expect(48, 0x305070);
  expect(72, 0x203040);
  expect(88, 0x406080);
  expect(112, 0x808080);
}

TEST_CASE("Shape blend modes handle translucent content", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(128, 32);
  Region background;
  Region multiply_layer;
  Region add_layer;
  canvas.addRegion(&background);
  background.setBounds(0, 0, 128, 32);
  for (Region* region : { &multiply_layer, &add_layer }) {
    canvas.addRegion(region);
    region->setNeedsLayer(true);
  }
  multiply_layer.setBounds(64, 0, 32, 32);
  add_layer.setBounds(96, 0, 32, 32);

  canvas.beginRegion(&background);
  canvas.setColor(0xff406080);
  canvas.fill(0, 0, 128, 32);
  canvas.setBlendMode(BlendMode::Mult);
  canvas.setColor(0xff808080);
  canvas.circle(0, 0, 32);
  canvas.setColor(0x80808080);
  canvas.fill(32, 0, 32, 32);
  canvas.endRegion();

  canvas.beginRegion(&multiply_layer);
  canvas.setBlendMode(BlendMode::Mult);
  canvas.setColor(0xff808080);
  canvas.fill(0, 0, 32, 32);
  canvas.endRegion();

  canvas.beginRegion(&add_layer);
  canvas.setBlendMode(BlendMode::Add);
  canvas.setColor(0x80ff0000);
  canvas.fill(0, 0, 32, 32);
  canvas.endRegion();

  const Screenshot& screenshot = canvas.takeScreenshot();
  auto expect = [&screenshot](int x, int y, unsigned int color) {
    Color sample = screenshot.sample(x, y);
    REQUIRE(std::abs(static_cast<int>(sample.hexRed()) - static_cast<int>((color >> 16) & 0xff)) <= 2);
    REQUIRE(std::abs(static_cast<int>(sample.hexGreen()) - static_cast<int>((color >> 8) & 0xff)) <= 2);
    REQUIRE(std::abs(static_cast<int>(sample.hexBlue()) - static_cast<int>(color & 0xff)) <= 2);
  };
  expect(2, 2, 0x406080);
  expect(16, 16, 0x203040);
  expect(48, 16, 0x304860);
  expect(80, 16, 0x808080);
  expect(112, 16, 0xa03040);
}

TEST_CASE("Blurred regions share temporary frame buffers", "[graphics]") {
  static constexpr int kNumBlurred = 8;

  Canvas canvas;
  canvas.setWindowless(256, 32);
  BlurPostEffect blurs[kNumBlurred];
  Region regions[kNumBlurred];
  for (int i = 0; i < kNumBlurred; ++i) {
    canvas.addRegion(&regions[i]);
    regions[i].setNeedsLayer(true);
    regions[i].setBounds(i * 32, 0, 16, 16);
  }

  auto frame_buffers = [&] {
    for (Region& region : regions) {
      region.invalidate();
      canvas.beginRegion(&region);
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, region.width(), region.height());
      canvas.endRegion();
    }
    canvas.submit();
    return bgfx::getStats()->numFrameBuffers;
  };

  int without_blur = frame_buffers();
  for (int i = 0; i < kNumBlurred; ++i) {
    blurs[i].setBlurRadius(3.0f);
    regions[i].setPostEffect(&blurs[i]);
  }
  REQUIRE(frame_buffers() - without_blur <= 2);
}

TEST_CASE("Blurred regions only blur again when their content changes", "[graphics]") {
  auto draws_per_frame = [](int num_blurred) {
    Canvas canvas;
    canvas.setWindowless(256, 32);
    std::vector<BlurPostEffect> blurs(num_blurred);
    std::vector<Region> regions(num_blurred);
    Region animated;
    canvas.addRegion(&animated);
    animated.setNeedsLayer(true);
    animated.setBounds(0, 0, 16, 16);
    for (int i = 0; i < num_blurred; ++i) {
      canvas.addRegion(&regions[i]);
      regions[i].setNeedsLayer(true);
      regions[i].setBounds(32 + i * 32, 0, 16, 16);
      blurs[i].setBlurRadius(10.0f);
      regions[i].setPostEffect(&blurs[i]);
      canvas.beginRegion(&regions[i]);
      canvas.setColor(0xffffffff);
      canvas.fill(0, 0, 16, 16);
      canvas.endRegion();
    }

    for (int frame = 0; frame < 3; ++frame) {
      animated.invalidate();
      canvas.beginRegion(&animated);
      canvas.setColor(frame % 2 ? 0xffff0000 : 0xff0000ff);
      canvas.fill(0, 0, 16, 16);
      canvas.endRegion();
      canvas.submit();
    }
    return bgfx::getStats()->numDraw;
  };

  REQUIRE(draws_per_frame(5) == draws_per_frame(0));
}

TEST_CASE("One blur effect can be shared by several regions", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect blur;
  blur.setBlurRadius(2.0f);
  Region left;
  Region right;
  canvas.addRegion(&left);
  left.setNeedsLayer(true);
  left.setPostEffect(&blur);
  left.setBounds(0, 0, 32, 32);
  canvas.addRegion(&right);
  right.setNeedsLayer(true);
  right.setPostEffect(&blur);
  right.setBounds(32, 0, 32, 32);

  auto fill = [&canvas](Region& region, unsigned int color) {
    canvas.beginRegion(&region);
    canvas.setColor(color);
    canvas.fill(0, 0, region.width(), region.height());
    canvas.endRegion();
  };

  fill(left, 0xffff0000);
  fill(right, 0xff0000ff);
  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(16, 16).hexRed() == 0xff);
  REQUIRE(screenshot.sample(16, 16).hexBlue() == 0x00);
  REQUIRE(screenshot.sample(48, 16).hexBlue() == 0xff);
  REQUIRE(screenshot.sample(48, 16).hexRed() == 0x00);
}

TEST_CASE("Blurred region with a hidden backdrop child still blurs", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BlurPostEffect blur;
  blur.setBlurRadius(4.0f);
  BlurPostEffect backdrop;
  backdrop.setBlurRadius(4.0f);
  Region blurred;
  Region glass;
  canvas.addRegion(&blurred);
  blurred.setNeedsLayer(true);
  blurred.setPostEffect(&blur);
  blurred.setBounds(0, 0, 64, 32);
  blurred.addRegion(&glass);
  glass.setNeedsLayer(true);
  glass.setBackdropEffect(&backdrop);
  glass.setBounds(0, 0, 8, 8);
  glass.setVisible(false);

  canvas.beginRegion(&blurred);
  canvas.setColor(0xffffffff);
  canvas.fill(0, 0, 32, 32);
  canvas.setColor(0xff000000);
  canvas.fill(32, 0, 32, 32);
  canvas.endRegion();
  REQUIRE(canvas.takeScreenshot().sample(30, 16).hexRed() < 0xf0);
}

TEST_CASE("Post effects destroyed before rendering", "[graphics]") {
  { BlurPostEffect blur; }
  { BloomPostEffect bloom; }
}

TEST_CASE("Translucent bloom keeps the alpha of its content", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(110, 10);
  canvas.setColor(0xffffffff);
  canvas.fill(0, 0, 110, 10);
  BloomPostEffect bloom;
  bloom.setBloomIntensity(0.0f);
  bloom.setTranslucent(true);
  Region region;
  canvas.addRegion(&region);
  region.setNeedsLayer(true);
  region.setPostEffect(&bloom);
  region.setBounds(0, 0, 110, 10);
  region.clear();
  region.invalidate();
  canvas.beginRegion(&region);
  for (int i = 0; i <= 10; ++i) {
    canvas.setColor(Color(0xff000000).withAlpha(i / 10.0f));
    canvas.fill(i * 10, 0, 10, 10);
  }
  canvas.endRegion();

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (int i = 0; i <= 10; ++i)
    REQUIRE(screenshot.sample(i * 10 + 5, 5).hexRed() == Approx(255 * (1.0f - i / 10.0f)).margin(2));
}

TEST_CASE("Translucent bloom glows like opaque bloom", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(200, 100);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 200, 100);
  BloomPostEffect opaque_bloom;
  BloomPostEffect translucent_bloom;
  translucent_bloom.setTranslucent(true);
  for (BloomPostEffect* bloom : { &opaque_bloom, &translucent_bloom }) {
    bloom->setBloomSize(16.0f);
    bloom->setBloomIntensity(0.6f);
  }
  Region opaque;
  Region outer;
  Region inner;
  Region translucent;
  canvas.addRegion(&opaque);
  canvas.addRegion(&outer);
  outer.addRegion(&inner);
  inner.addRegion(&translucent);
  for (Region* region : { &opaque, &outer, &inner, &translucent })
    region->setNeedsLayer(true);
  opaque.setPostEffect(&opaque_bloom);
  translucent.setPostEffect(&translucent_bloom);
  opaque.setBounds(0, 0, 100, 100);
  outer.setBounds(100, 0, 100, 100);
  inner.setBounds(0, 0, 100, 100);
  translucent.setBounds(0, 0, 100, 100);

  for (Region* region : { &opaque, &translucent }) {
    canvas.beginRegion(region);
    canvas.setColor(Color(0xffff8040, 1.5f));
    canvas.circle(40, 40, 20);
    canvas.endRegion();
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(62, 50).hexRed() > 0x40);
  for (int x : { 62, 70, 80 }) {
    Color opaque_glow = screenshot.sample(x, 50);
    Color translucent_glow = screenshot.sample(x + 100, 50);
    REQUIRE(translucent_glow.hexRed() == Approx(opaque_glow.hexRed()).margin(6));
    REQUIRE(translucent_glow.hexGreen() == Approx(opaque_glow.hexGreen()).margin(6));
    REQUIRE(translucent_glow.hexBlue() == Approx(opaque_glow.hexBlue()).margin(6));
  }
}

TEST_CASE("Bloom glow stays in place when part of its region redraws", "[graphics]") {
  auto render = [](bool redraw_part) -> Screenshot {
    Canvas canvas;
    canvas.setWindowless(96, 64);
    BloomPostEffect bloom;
    bloom.setBloomSize(16.0f);
    bloom.setBloomIntensity(0.6f);
    Region glowing;
    Region overlap;
    canvas.addRegion(&glowing);
    glowing.setNeedsLayer(true);
    glowing.setPostEffect(&bloom);
    glowing.setBounds(16, 0, 64, 64);
    canvas.addRegion(&overlap);
    overlap.setBounds(0, 0, 48, 64);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&glowing);
    canvas.setColor(Color(0xffff8040, 1.5f));
    canvas.circle(16, 16, 32);
    canvas.endRegion();

    auto draw_stripe = [&](unsigned int color) {
      overlap.clear();
      overlap.invalidate();
      canvas.beginRegion(&overlap);
      canvas.setColor(color);
      canvas.fill(0, 0, 8, overlap.height());
      canvas.endRegion();
    };

    if (!redraw_part) {
      draw_stripe(0xff00ff00);
      return canvas.takeScreenshot();
    }

    draw_stripe(0xff0000ff);
    canvas.submit();
    draw_stripe(0xff00ff00);
    canvas.layer(0)->requestScreenshot();
    canvas.submit();
    return canvas.layer(0)->screenshot();
  };

  Screenshot full = render(false);
  Screenshot partial = render(true);
  for (int y = 0; y < 64; y += 3) {
    for (int x = 16; x < 80; x += 3) {
      INFO("x " << x << " y " << y);
      REQUIRE(partial.sample(x, y).hexRed() == Approx(full.sample(x, y).hexRed()).margin(2));
      REQUIRE(partial.sample(x, y).hexGreen() == Approx(full.sample(x, y).hexGreen()).margin(2));
    }
  }
}

TEST_CASE("Bloom inside a transparent layered region keeps what's behind it", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  canvas.setColor(0xffff0000);
  canvas.fill(0, 0, 64, 32);
  BloomPostEffect bloom;
  Region container;
  Region glowing;
  canvas.addRegion(&container);
  container.setNeedsLayer(true);
  container.setBounds(0, 0, 64, 32);
  container.addRegion(&glowing);
  glowing.setNeedsLayer(true);
  glowing.setPostEffect(&bloom);
  glowing.setBounds(16, 0, 32, 32);

  canvas.beginRegion(&glowing);
  canvas.setColor(0xffffffff);
  canvas.fill(14, 14, 4, 4);
  canvas.endRegion();

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (int x : { 18, 24, 40, 44 }) {
    INFO("x " << x);
    REQUIRE(screenshot.sample(x, 2).hexRed() == 0xff);
  }
}

TEST_CASE("Canvas strokes a path at its logical width", "[graphics]") {
  for (float scale : { 1.0f, 2.0f }) {
    Canvas canvas;
    canvas.setDpiScale(scale);
    canvas.setWindowless(20 * scale, 20 * scale);
    canvas.setLogicalPixelScale();
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, 20, 20);

    Path path;
    path.moveTo(2, 10);
    path.lineTo(18, 10);
    canvas.setColor(0xffffffff);
    canvas.stroke(path, 0, 0, 20, 20, 4.0f, Path::Join::Miter, Path::EndCap::Butt);

    const Screenshot& screenshot = canvas.takeScreenshot();
    int covered = 0;
    for (int y = 0; y < 20 * scale; ++y)
      covered += screenshot.sample(10 * scale, y).hexRed();
    REQUIRE(covered / 255.0f == Catch::Approx(4.0f * scale).margin(0.5f));
  }
}

TEST_CASE("Fill with no bounds fills the clamp bounds", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(10, 10);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 10, 10);
  canvas.setClampBounds(5, 2, 5, 6);
  canvas.setColor(0xffffffff);
  canvas.fill();

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(7, 5).hexRed() == 0xff);
  REQUIRE(screenshot.sample(9, 7).hexRed() == 0xff);
  REQUIRE(screenshot.sample(2, 5).hexRed() == 0x00);
  REQUIRE(screenshot.sample(7, 1).hexRed() == 0x00);
  REQUIRE(screenshot.sample(7, 9).hexRed() == 0x00);
}

TEST_CASE("Super ellipse shape does not change with DPI scale", "[graphics]") {
  for (float scale : { 1.0f, 2.0f }) {
    Canvas canvas;
    canvas.setDpiScale(scale);
    canvas.setWindowless(100 * scale, 100 * scale);
    canvas.setLogicalPixelScale();
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, 100, 100);
    canvas.setColor(0xffffffff);
    canvas.superEllipse(0, 0, 100, 100, 4.0f);

    const Screenshot& screenshot = canvas.takeScreenshot();
    REQUIRE(screenshot.sample(30 * scale, 50 * scale).hexRed() == 0xff);
    REQUIRE(screenshot.sample(92.5f * scale, 92.5f * scale).hexRed() == 0x00);
  }
}

TEST_CASE("Squircle centers are filled and squircle border centers are empty", "[graphics]") {
  for (int size : { 100, 101 }) {
    Canvas canvas;
    canvas.setWindowless(2 * size, size);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, 2 * size, size);
    canvas.setColor(0xffffffff);
    canvas.squircle(0, 0, size);
    canvas.squircleBorder(size, 0, size, 4.0f, 8.0f);

    const Screenshot& screenshot = canvas.takeScreenshot();
    for (int offset = -2; offset <= 2; ++offset) {
      REQUIRE(screenshot.sample(size / 2 + offset, size / 2).hexRed() == 0xff);
      REQUIRE(screenshot.sample(size + size / 2 + offset, size / 2).hexRed() == 0x00);
    }
    REQUIRE(screenshot.sample(size + 3, size / 2).hexRed() == 0xff);
  }
}

TEST_CASE("Custom shaders draw with their own blend mode", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(20, 10);
  canvas.setColor(0xff0000ff);
  canvas.fill(0, 0, 20, 10);

  Shader alpha(Fill::vertexShader(), Fill::fragmentShader(), BlendMode::Alpha);
  Shader add(Fill::vertexShader(), Fill::fragmentShader(), BlendMode::Add);
  canvas.setColor(0xffff0000);
  canvas.shader(&alpha, 0, 0, 10, 10);
  canvas.shader(&add, 10, 0, 10, 10);

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(5, 5).hexRed() == 0xff);
  REQUIRE(screenshot.sample(5, 5).hexBlue() == 0x00);
  REQUIRE(screenshot.sample(15, 5).hexRed() == 0xff);
  REQUIRE(screenshot.sample(15, 5).hexBlue() == 0xff);
}

TEST_CASE("Moving a layered region keeps its atlas slot", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(200, 50);
  Region moving;
  Region still;
  for (Region* region : { &moving, &still }) {
    canvas.addRegion(region);
    region->setNeedsLayer(true);
  }
  moving.setBounds(0, 0, 40, 40);
  still.setBounds(100, 0, 40, 40);
  for (Region* region : { &moving, &still }) {
    canvas.beginRegion(region);
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, 40, 40);
    canvas.endRegion();
  }
  canvas.takeScreenshot();

  Layer* layer = moving.layer();
  IPoint slot = layer->coordinatesForRegion(&moving);
  for (int x = 1; x <= 20; ++x) {
    moving.setBounds(x, 0, 40, 40);
    REQUIRE(layer->coordinatesForRegion(&moving).x == slot.x);
    REQUIRE(layer->coordinatesForRegion(&moving).y == slot.y);
    REQUIRE(layer->invalidRects().count(&still) == 0);
  }
  REQUIRE(canvas.takeScreenshot().sample(55, 20).hexRed() == 0xff);
}

TEST_CASE("Rounded triangle with collinear points records no shapes", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(80, 20);
  Region region;
  canvas.addRegion(&region);
  region.setBounds(0, 0, 3010, 3010);

  canvas.beginRegion(&region);
  canvas.setColor(0xffffffff);
  canvas.roundedTriangle(5, 10, 20, 10, 35, 10, 3);
  canvas.roundedTriangleBorder(45, 10, 60, 10, 75, 10, 3, 4);
  canvas.roundedTriangle(0.1f, 0.1f, 1000.3f, 1000.3f, 3000.7f, 3000.7f, 3);
  canvas.endRegion();
  REQUIRE(region.numSubmitBatches() == 0);
}

TEST_CASE("Zero-length linear gradients paint a solid color like Chrome", "[graphics]") {
  std::string svg_data = R"svg(<svg viewBox="0 0 60 20" width="60" height="20">
    <linearGradient id="pad" x1="0.5" y1="0.5" x2="0.5" y2="0.5">
      <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
    </linearGradient>
    <linearGradient id="repeat" x1="0.5" y1="0.5" x2="0.5" y2="0.5" spreadMethod="repeat">
      <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
    </linearGradient>
    <linearGradient id="reflect" x1="0.5" y1="0.5" x2="0.5" y2="0.5" spreadMethod="reflect">
      <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
    </linearGradient>
    <rect width="20" height="20" fill="url(#pad)"/>
    <rect x="20" width="20" height="20" fill="url(#repeat)"/>
    <rect x="40" width="20" height="20" fill="url(#reflect)"/>
  </svg>)svg";
  Canvas canvas;
  canvas.setWindowless(80, 20);
  Svg svg(reinterpret_cast<const unsigned char*>(svg_data.data()), static_cast<int>(svg_data.size()));
  canvas.svg(svg, 0, 0, 60, 20);
  canvas.setColor(Brush::linear(Color(0xffff0000), Color(0xff0000ff), Point(70, 10), Point(70, 10)));
  canvas.fill(60, 0, 20, 20);

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (int x : { 10, 70 }) {
    INFO("x " << x);
    REQUIRE(screenshot.sample(x, 10).hexRed() <= 0x02);
    REQUIRE(screenshot.sample(x, 10).hexBlue() >= 0xfd);
  }
  for (int x : { 30, 50 }) {
    INFO("x " << x);
    REQUIRE(screenshot.sample(x, 10).hexRed() == Approx(0x80).margin(2));
    REQUIRE(screenshot.sample(x, 10).hexBlue() == Approx(0x80).margin(2));
  }
}

TEST_CASE("Gradient average weighs each stop by the span it covers", "[graphics]") {
  Gradient gradient(0xffff0000, 0xff0000ff);
  REQUIRE(gradient.average().hexRed() == Approx(0x80).margin(1));
  REQUIRE(gradient.average().hexBlue() == Approx(0x80).margin(1));

  Gradient offset;
  offset.addColorStop(Color(0xffff0000), 0.25f);
  offset.addColorStop(Color(0xff0000ff), 1.0f);
  REQUIRE(offset.average().red() == Approx(0.625f).margin(0.001f));
  REQUIRE(offset.average().blue() == Approx(0.375f).margin(0.001f));
}

TEST_CASE("Zero-length segments draw a dot only with rounded ends", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(40, 20);
  Region flat;
  Region rounded;
  for (Region* region : { &flat, &rounded })
    canvas.addRegion(region);
  flat.setBounds(0, 0, 20, 20);
  rounded.setBounds(20, 0, 20, 20);

  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 40, 20);
  canvas.beginRegion(&flat);
  canvas.setColor(0xffffffff);
  canvas.segment(10, 10, 10, 10, 4, false);
  canvas.endRegion();
  canvas.beginRegion(&rounded);
  canvas.setColor(0xffffffff);
  canvas.segment(10, 10, 10, 10, 4, true);
  canvas.endRegion();

  REQUIRE(flat.numSubmitBatches() == 0);
  REQUIRE(canvas.takeScreenshot().sample(30, 10).hexRed() == 0xff);
}

TEST_CASE("Radial gradients with focal radii at or beyond the radius render like Chrome",
          "[graphics]") {
  std::string svg_data = R"svg(<svg viewBox="0 0 100 40" width="100" height="40">
    <radialGradient id="same" r="0.5" fr="0.5">
      <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
    </radialGradient>
    <radialGradient id="offset" r="0.5" fx="0.3" fr="0.5">
      <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
    </radialGradient>
    <radialGradient id="wide" r="0.25" fr="0.5">
      <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
    </radialGradient>
    <radialGradient id="offset_wide" r="0.25" fx="0.4" fr="0.5">
      <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
    </radialGradient>
    <rect width="20" height="20" fill="url(#same)"/>
    <rect x="20" width="20" height="20" fill="url(#offset)"/>
    <rect x="40" width="20" height="20" fill="url(#wide)"/>
    <rect x="60" width="40" height="40" fill="url(#offset_wide)"/>
  </svg>)svg";
  Canvas canvas;
  canvas.setWindowless(100, 40);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 100, 40);
  Svg svg(reinterpret_cast<const unsigned char*>(svg_data.data()), static_cast<int>(svg_data.size()));
  canvas.svg(svg, 0, 0, 100, 40);

  const Screenshot& screenshot = canvas.takeScreenshot();
  auto require_color = [&](int x, int y, unsigned int expected) {
    INFO("x " << x << " y " << y);
    Color color = screenshot.sample(x, y);
    REQUIRE(color.hexRed() == Approx((expected >> 16) & 0xff).margin(3));
    REQUIRE(color.hexBlue() == Approx(expected & 0xff).margin(3));
  };

  require_color(10, 10, 0xff0000);
  require_color(1, 10, 0xff0000);
  require_color(1, 1, 0x0000ff);

  Color offset = screenshot.sample(30, 10);
  REQUIRE(offset.hexRed() + offset.hexBlue() > 0xf0);

  require_color(50, 10, 0x0000ff);
  Color ramp = screenshot.sample(57, 10);
  REQUIRE(ramp.hexRed() > 0x40);
  REQUIRE(ramp.hexBlue() > 0x40);
  require_color(41, 1, 0xff0000);

  require_color(65, 20, 0x5200ad);
  require_color(61, 20, 0x9b0064);
  require_color(61, 1, 0xff0000);
  require_color(95, 20, 0xeb0015);
}

TEST_CASE("Renderer reads captured screenshots in their format and row order", "[graphics]") {
  struct Capture {
    const void* data;
    int format;
    bool y_flip;
  };

  static const uint8_t kBgraBottomUp[] = { 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff };
  static const uint8_t kRgbaTopDown[] = { 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff };
  static const uint32_t kRgb10A2TopDown[] = { 0xc00003ff, 0xfff00000 };
  for (const Capture& capture : { Capture { kBgraBottomUp, bgfx::TextureFormat::BGRA8, true },
                                  Capture { kRgbaTopDown, bgfx::TextureFormat::RGBA8, false },
                                  Capture { kRgb10A2TopDown, bgfx::TextureFormat::RGB10A2, false } }) {
    INFO("format " << capture.format);
    Renderer::instance().setScreenshotData(static_cast<const uint8_t*>(capture.data), 1, 2, 4,
                                           capture.format, capture.y_flip);
    const Screenshot& screenshot = Renderer::instance().screenshot();
    REQUIRE(screenshot.sample(0, 0).hexRed() == 0xff);
    REQUIRE(screenshot.sample(0, 0).hexBlue() == 0x00);
    REQUIRE(screenshot.sample(0, 1).hexRed() == 0x00);
    REQUIRE(screenshot.sample(0, 1).hexBlue() == 0xff);
  }
}

TEST_CASE("Layer atlas grows one side at a time", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 64);
  Region first;
  Region second;
  for (Region* region : { &first, &second }) {
    canvas.addRegion(region);
    region->setNeedsLayer(true);
    region->setBounds(0, 0, 3840, 2160);
  }

  Layer* layer = first.layer();
  REQUIRE(layer->width() * layer->height() <= 4096 * 8192);
  REQUIRE(layer->width() <= bgfx::getCaps()->limits.maxTextureSize);
  REQUIRE(layer->height() <= bgfx::getCaps()->limits.maxTextureSize);
}

TEST_CASE("Over a hundred bloom regions all glow in their own color", "[graphics]") {
  static constexpr int kNumRegions = 120;
  static constexpr int kColumns = 12;
  static constexpr int kSize = 24;
  static constexpr unsigned int kColors[] = { 0xffff0000, 0xff00ff00, 0xff0000ff };

  Canvas canvas;
  canvas.setWindowless(kColumns * kSize, (kNumRegions / kColumns) * kSize);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kColumns * kSize, (kNumRegions / kColumns) * kSize);

  std::vector<std::unique_ptr<BloomPostEffect>> blooms;
  std::vector<std::unique_ptr<Region>> regions;
  for (int i = 0; i < kNumRegions; ++i) {
    blooms.push_back(std::make_unique<BloomPostEffect>());
    blooms.back()->setBloomSize(64.0f);
    regions.push_back(std::make_unique<Region>());
    Region* region = regions.back().get();
    canvas.addRegion(region);
    region->setNeedsLayer(true);
    region->setPostEffect(blooms.back().get());
    region->setBounds((i % kColumns) * kSize, (i / kColumns) * kSize, kSize, kSize);
    canvas.beginRegion(region);
    canvas.setColor(Color(kColors[i % 3], 1.5f));
    canvas.fill(kSize / 2 - 2, kSize / 2 - 2, 4, 4);
    canvas.endRegion();
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (int i = 0; i < kNumRegions; ++i) {
    INFO("region " << i);
    Color glow = screenshot.sample((i % kColumns) * kSize + kSize / 2 + 5,
                                   (i / kColumns) * kSize + kSize / 2);
    unsigned int channels[] = { glow.hexRed(), glow.hexGreen(), glow.hexBlue() };
    REQUIRE(channels[i % 3] > 0x10);
    REQUIRE(channels[(i + 1) % 3] < 0x08);
    REQUIRE(channels[(i + 2) % 3] < 0x08);
  }
}

TEST_CASE("Blur resized every frame doesn't pile up frame buffers", "[graphics]") {
  static constexpr int kFrames = 400;

  Canvas canvas;
  canvas.setWindowless(260, 140);
  BlurPostEffect blur;
  blur.setBlurRadius(300.0f);
  Region region;
  canvas.addRegion(&region);
  region.setNeedsLayer(true);
  region.setPostEffect(&blur);

  for (int i = 0; i < kFrames; ++i) {
    region.setBounds(0, 0, 50 + (i * 4) % 200, 50 + 4 * (i / 50));
    canvas.beginRegion(&region);
    canvas.setColor(0xffff0000);
    canvas.fill(0, 0, region.width() / 2, region.height());
    canvas.setColor(0xff0000ff);
    canvas.fill(region.width() / 2, 0, region.width() - region.width() / 2, region.height());
    canvas.endRegion();
    canvas.submit();
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(bgfx::getStats()->numFrameBuffers < 200);
  Color center = screenshot.sample(region.width() / 2, region.height() / 2);
  REQUIRE(center.hexRed() > 0x20);
  REQUIRE(center.hexBlue() > 0x20);
}

TEST_CASE("Temporary frame buffers are reused when their sizes come back", "[graphics]") {
  static constexpr int kSizes = 64;
  static constexpr int kPeriod = 4;

  Canvas canvas;
  canvas.setWindowless(64, 64);
  Layer* layer = canvas.layer(1);
  bgfx::FrameBufferHandle handles[kSizes];

  for (int submit = 0; submit < 4 * kPeriod; ++submit) {
    for (int i = submit % kPeriod; i < kSizes; i += kPeriod) {
      bgfx::FrameBufferHandle handle = layer->temporaryFrameBuffer(16 + i, 16);
      if (submit >= kPeriod)
        REQUIRE(handle.idx == handles[i].idx);
      handles[i] = handle;
      layer->releaseTemporaryFrameBuffer(handle);
    }
    layer->destroyIdleTemporaryFrameBuffers();
  }
}

TEST_CASE("Bloom resized every frame doesn't pile up frame buffers", "[graphics]") {
  static constexpr int kFrames = 400;

  Canvas canvas;
  canvas.setWindowless(260, 140);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 260, 140);
  BloomPostEffect bloom;
  bloom.setBloomSize(64.0f);
  Region region;
  canvas.addRegion(&region);
  region.setNeedsLayer(true);
  region.setPostEffect(&bloom);

  for (int i = 0; i < kFrames; ++i) {
    region.setBounds(0, 0, 50 + (i * 4) % 200, 50 + 4 * (i / 50));
    canvas.beginRegion(&region);
    canvas.setColor(Color(0xffff0000, 1.5f));
    canvas.fill(region.width() / 2 - 8, region.height() / 2 - 8, 16, 16);
    canvas.endRegion();
    canvas.submit();
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(bgfx::getStats()->numFrameBuffers < 200);
  REQUIRE(screenshot.sample(region.width() / 2 + 12, region.height() / 2).hexRed() > 0x40);
}

TEST_CASE("Shape batches find overlaps in every chunk of shapes", "[graphics]") {
  ShapeBatch<Rectangle> batch(BlendMode::Alpha);
  for (int i = 0; i < 200; ++i)
    batch.addShape(Rectangle({}, nullptr, i * 10.0f, 0.0f, 5.0f, 5.0f));

  auto overlaps = [&batch](float x, float width) {
    return batch.overlapsShape(Rectangle({}, nullptr, x, 1.0f, width, 1.0f));
  };
  REQUIRE(overlaps(1.0f, 1.0f));
  REQUIRE(overlaps(1500.0f, 2.0f));
  REQUIRE(overlaps(1994.0f, 1.0f));
  REQUIRE_FALSE(overlaps(1506.0f, 2.0f));
  REQUIRE_FALSE(overlaps(2000.0f, 10.0f));
  REQUIRE_FALSE(overlaps(9.5f, 1.0f));

  ShapeBatch<Rectangle> nan_first(BlendMode::Alpha);
  nan_first.addShape(Rectangle({}, nullptr, std::nanf(""), 0.0f, 5.0f, 5.0f));
  nan_first.addShape(Rectangle({}, nullptr, 20.0f, 0.0f, 5.0f, 5.0f));
  REQUIRE(nan_first.overlapsShape(Rectangle({}, nullptr, 21.0f, 1.0f, 1.0f, 1.0f)));
}

TEST_CASE("Thousands of live blur effects all blur", "[graphics]") {
  static constexpr int kNumCanvases = 4;
  static constexpr int kRegionsPerCanvas = 600;
  static constexpr int kColumns = 30;
  static constexpr int kSpacing = 8;
  static constexpr int kSize = 6;

  std::vector<std::unique_ptr<Canvas>> canvases;
  std::vector<std::unique_ptr<BlurPostEffect>> blurs;
  std::vector<std::unique_ptr<Region>> regions;
  for (int c = 0; c < kNumCanvases; ++c) {
    canvases.push_back(std::make_unique<Canvas>());
    Canvas& canvas = *canvases.back();
    canvas.setWindowless(kColumns * kSpacing, (kRegionsPerCanvas / kColumns) * kSpacing);
    for (int i = 0; i < kRegionsPerCanvas; ++i) {
      blurs.push_back(std::make_unique<BlurPostEffect>());
      blurs.back()->setBlurRadius(2.0f);
      regions.push_back(std::make_unique<Region>());
      Region* region = regions.back().get();
      canvas.addRegion(region);
      region->setNeedsLayer(true);
      region->setPostEffect(blurs.back().get());
      region->setBounds((i % kColumns) * kSpacing, (i / kColumns) * kSpacing, kSize, kSize);
      canvas.beginRegion(region);
      canvas.setColor(0xffff0000);
      canvas.fill(0, 0, kSize / 2, kSize);
      canvas.setColor(0xff0000ff);
      canvas.fill(kSize / 2, 0, kSize - kSize / 2, kSize);
      canvas.endRegion();
    }

    const Screenshot& screenshot = canvas.takeScreenshot();
    for (int i = 0; i < kRegionsPerCanvas; ++i) {
      INFO("canvas " << c << ", region " << i);
      Color center = screenshot.sample((i % kColumns) * kSpacing + kSize / 2,
                                       (i / kColumns) * kSpacing + kSize / 2);
      REQUIRE(center.hexRed() > 0x20);
      REQUIRE(center.hexBlue() < 0xe0);
    }
  }
}

TEST_CASE("Shader data that bgfx rejects leaves an invalid cached handle", "[graphics]") {
  static constexpr unsigned char kNotAShader[] = { 'b', 'a', 'd', ' ', 'd', 'a', 't', 'a' };
  static const EmbeddedFile kFile = { "not_a_shader", kNotAShader, sizeof(kNotAShader) };

  Canvas canvas;
  canvas.setWindowless(8, 8);
  REQUIRE_FALSE(bgfx::isValid(ShaderCache::shaderHandle(kFile)));
  REQUIRE_FALSE(bgfx::isValid(ShaderCache::shaderHandle(kFile)));
}

#ifdef NDEBUG
TEST_CASE("Uniform names that bgfx rejects leave an invalid cached handle", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(8, 8);
  REQUIRE_FALSE(bgfx::isValid(UniformCache::uniformHandle("bad name")));
}
#endif
