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
#include "visage_graphics/color.h"
#include "visage_graphics/gradient.h"
#include "visage_graphics/gpu/gpu.h"
#include "visage_graphics/graphics_caches.h"
#include "visage_graphics/post_effects.h"
#include "visage_graphics/renderer.h"
#include "visage_graphics/shader.h"
#include "visage_graphics/shape_batcher.h"
#include "visage_utils/child_process.h"
#include "visage_utils/file_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>

#if VISAGE_LINUX
#include <dlfcn.h>
#endif

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

TEST_CASE("Filled rectangles follow Canvas::setPosition", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(200, 40);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 200, 40);
  canvas.setPosition(100, 0);
  canvas.setColor(0xffff0000);

  SECTION("Without clamp bounds") {
    canvas.fill(0, 0, 50, 40);
    const Screenshot& screenshot = canvas.takeScreenshot();
    REQUIRE(screenshot.sample(25, 20).hexRed() == 0);
    REQUIRE(screenshot.sample(125, 20).hexRed() == 0xff);
    REQUIRE(screenshot.sample(175, 20).hexRed() == 0);
  }

  SECTION("Inside clamp bounds set after the position") {
    canvas.setClampBounds(0, 0, 25, 40);
    canvas.fill(0, 0, 50, 40);
    const Screenshot& screenshot = canvas.takeScreenshot();
    REQUIRE(screenshot.sample(110, 20).hexRed() == 0xff);
    REQUIRE(screenshot.sample(140, 20).hexRed() == 0);
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

TEST_CASE("Solid colors that change every frame draw correctly from recycled atlas slots", "[graphics]") {
  static constexpr int kColumns = 30;
  static constexpr int kRows = 20;
  static constexpr int kCellWidth = 6;
  static constexpr int kCellHeight = 10;
  static constexpr int kTestWidth = kColumns * kCellWidth;
  static constexpr int kTestHeight = kRows * kCellHeight + kCellHeight;
  static constexpr int kFrames = 6;

  auto cell_color = [](int frame, int cell) -> unsigned int {
    int value = frame * kColumns * kRows + cell;
    return 0xff000000 | ((value & 0xff) << 16) | (((value >> 8) & 0xff) << 8) | 0x40;
  };
  Gradient stripe(0xffff0000, 0xff0000ff);
  stripe.setRepeat(true);

  Canvas canvas;
  canvas.setWindowless(kTestWidth, kTestHeight);

  for (int frame = 0; frame < kFrames; ++frame) {
    canvas.clearDrawnShapes();
    canvas.setColor(Brush::horizontal(stripe));
    canvas.fill(0, kRows * kCellHeight, kTestWidth, kCellHeight);
    for (int r = 0; r < kRows; ++r) {
      for (int c = 0; c < kColumns; ++c) {
        canvas.setColor(cell_color(frame, r * kColumns + c));
        canvas.fill(c * kCellWidth, r * kCellHeight, kCellWidth, kCellHeight);
      }
    }
    canvas.submit();
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (int r = 0; r < kRows; ++r) {
    for (int c = 0; c < kColumns; ++c) {
      Color expected = cell_color(kFrames - 1, r * kColumns + c);
      Color sample = screenshot.sample(c * kCellWidth + kCellWidth / 2, r * kCellHeight + kCellHeight / 2);
      INFO("Cell " << r << ", " << c);
      REQUIRE(std::abs(static_cast<int>(sample.hexRed()) - static_cast<int>(expected.hexRed())) <= 1);
      REQUIRE(std::abs(static_cast<int>(sample.hexGreen()) - static_cast<int>(expected.hexGreen())) <= 1);
      REQUIRE(std::abs(static_cast<int>(sample.hexBlue()) - static_cast<int>(expected.hexBlue())) <= 1);
    }
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

TEST_CASE("Manual batching adds a new batch after the last one", "[graphics]") {
  ShapeBatcher batcher;
  batcher.setManualBatching(true);
  batcher.addShape(Fill({}, nullptr, 0, 0, 10, 10));
  batcher.addShape(Rectangle({}, nullptr, 0, 0, 10, 10));
  batcher.addShape(Rectangle({}, nullptr, 20, 0, 10, 10));
  batcher.addShape(Fill({}, nullptr, 0, 20, 10, 10));

  REQUIRE(batcher.numBatches() == 3);
  REQUIRE(dynamic_cast<ShapeBatch<Fill>*>(batcher.batchAtIndex(0)) != nullptr);
  REQUIRE(dynamic_cast<ShapeBatch<Rectangle>*>(batcher.batchAtIndex(1)) != nullptr);
  REQUIRE(dynamic_cast<ShapeBatch<Fill>*>(batcher.batchAtIndex(2)) != nullptr);
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
  int textures = gpu::getStats()->num_textures;
  for (int i = 0; i < 8; ++i)
    screenshot(17 + i);
  REQUIRE(gpu::getStats()->num_textures <= textures);
}

TEST_CASE("Destroying blur and bloom effects doesn't leak vertex buffers", "[graphics]") {
  Canvas keep_renderer_running;
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
  gpu::frame();
  int vertex_buffers = gpu::getStats()->num_buffers;
  for (int i = 0; i < 4; ++i)
    draw_effects();
  gpu::frame();
  REQUIRE(gpu::getStats()->num_buffers <= vertex_buffers);
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
    uint32_t start = gpu::frame();
    region.clear();
    region.invalidate();
    canvas.beginRegion(&region);
    canvas.setColor(0xffffffff);
    canvas.fill(0, 0, region.width(), region.height());
    canvas.endRegion();
    canvas.submit();
    return gpu::frame() - start - 1;
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
  Canvas canvas;
  {
    Canvas previous;
    previous.setWindowless(128, 128);
    previous.setColor(0xffffffff);
    previous.fill(0, 0, previous.width(), previous.height());
    previous.takeScreenshot();
  }

  canvas.setWindowless(128, 64);
  BloomPostEffect bloom;
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

TEST_CASE("Layer stops being HDR when its bloom region moves to another layer", "[graphics]") {
  struct Result {
    unsigned int translucent = 0;
    unsigned int bright = 0;
    int format = 0;
  };

  auto render = [](bool layer_parent_later) -> Result {
    Canvas canvas;
    canvas.setWindowless(64, 32);
    BloomPostEffect bloom;
    Region parent;
    Region glowing;
    canvas.addRegion(&parent);
    parent.setBounds(0, 0, 64, 32);
    if (!layer_parent_later)
      parent.setNeedsLayer(true);
    parent.addRegion(&glowing);
    glowing.setNeedsLayer(true);
    glowing.setPostEffect(&bloom);
    glowing.setBounds(40, 0, 24, 32);

    canvas.setColor(0xff808080);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&parent);
    canvas.setColor(0x80000000);
    canvas.fill(0, 0, 32, 32);
    canvas.setColor(Color(0x80ffffff, 2.0f));
    canvas.fill(32, 0, 8, 32);
    canvas.endRegion();
    canvas.beginRegion(&glowing);
    canvas.setColor(0xff202020);
    canvas.fill(0, 0, glowing.width(), glowing.height());
    canvas.endRegion();
    canvas.submit();

    if (layer_parent_later)
      parent.setNeedsLayer(true);
    const Screenshot& screenshot = canvas.takeScreenshot();
    return { screenshot.sample(16, 16).hexRed(), screenshot.sample(36, 16).hexRed(),
             canvas.layer(2)->frameBufferFormat() };
  };

  Result layered_later = render(true);
  Result layered_first = render(false);
  REQUIRE(layered_later.translucent == Approx(layered_first.translucent).margin(2));
  REQUIRE(layered_later.bright == Approx(layered_first.bright).margin(2));
  REQUIRE(layered_later.format == layered_first.format);
}

TEST_CASE("Regions sharing a layer recomposite when the layer changes HDR", "[graphics]") {
  auto render = [](bool parent_layered, bool change) {
    Canvas canvas;
    canvas.setWindowless(96, 64);
    BloomPostEffect bloom;
    Region parent;
    Region glowing;
    Region sibling;
    canvas.addRegion(&parent);
    parent.setBounds(0, 0, 40, 64);
    parent.setNeedsLayer(parent_layered);
    parent.addRegion(&glowing);
    glowing.setNeedsLayer(true);
    glowing.setPostEffect(&bloom);
    glowing.setBounds(0, 0, 40, 64);
    canvas.addRegion(&sibling);
    sibling.setNeedsLayer(true);
    sibling.setBounds(44, 0, 48, 64);

    canvas.setColor(0xff808080);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&glowing);
    canvas.setColor(0xff202020);
    canvas.fill(0, 0, glowing.width(), glowing.height());
    canvas.endRegion();
    canvas.beginRegion(&sibling);
    canvas.setColor(Color(0x80ffffff, 2.0f));
    canvas.fill(0, 0, sibling.width(), sibling.height());
    canvas.endRegion();

    if (!change)
      return canvas.takeScreenshot().sample(68, 32).hexRed();

    canvas.submit();
    parent.setNeedsLayer(!parent_layered);
    canvas.layer(0)->requestScreenshot();
    canvas.submit();
    return canvas.layer(0)->screenshot().sample(68, 32).hexRed();
  };

  REQUIRE(render(false, true) == Approx(render(true, false)).margin(2));
  REQUIRE(render(true, true) == Approx(render(false, false)).margin(2));
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
    return gpu::getStats()->num_frame_buffers;
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
    return gpu::getStats()->num_draws;
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

TEST_CASE("Bloom keeps the alpha of its content", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(110, 10);
  canvas.setColor(0xffffffff);
  canvas.fill(0, 0, 110, 10);
  BloomPostEffect bloom;
  bloom.setBloomIntensity(0.0f);
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

TEST_CASE("Nested bloom glows like top-level bloom", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(200, 100);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, 200, 100);
  BloomPostEffect top_bloom;
  BloomPostEffect nested_bloom;
  for (BloomPostEffect* bloom : { &top_bloom, &nested_bloom }) {
    bloom->setBloomSize(16.0f);
    bloom->setBloomIntensity(0.6f);
  }
  Region top;
  Region outer;
  Region inner;
  Region nested;
  canvas.addRegion(&top);
  canvas.addRegion(&outer);
  outer.addRegion(&inner);
  inner.addRegion(&nested);
  for (Region* region : { &top, &outer, &inner, &nested })
    region->setNeedsLayer(true);
  top.setPostEffect(&top_bloom);
  nested.setPostEffect(&nested_bloom);
  top.setBounds(0, 0, 100, 100);
  outer.setBounds(100, 0, 100, 100);
  inner.setBounds(0, 0, 100, 100);
  nested.setBounds(0, 0, 100, 100);

  for (Region* region : { &top, &nested }) {
    canvas.beginRegion(region);
    canvas.setColor(Color(0xffff8040, 1.5f));
    canvas.circle(40, 40, 20);
    canvas.endRegion();
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(62, 50).hexRed() > 0x40);
  for (int x : { 62, 70, 80 }) {
    Color top_glow = screenshot.sample(x, 50);
    Color nested_glow = screenshot.sample(x + 100, 50);
    REQUIRE(nested_glow.hexRed() == Approx(top_glow.hexRed()).margin(6));
    REQUIRE(nested_glow.hexGreen() == Approx(top_glow.hexGreen()).margin(6));
    REQUIRE(nested_glow.hexBlue() == Approx(top_glow.hexBlue()).margin(6));
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

TEST_CASE("Bloom glow keeps its brightness when its parent layer is HDR", "[graphics]") {
  auto render = [](bool hdr_parent) -> Screenshot {
    Canvas canvas;
    canvas.setWindowless(96, 32);
    BloomPostEffect bloom;
    bloom.setBloomSize(16.0f);
    bloom.setBloomIntensity(0.3f);
    BloomPostEffect neighbor_bloom;
    neighbor_bloom.setBloomIntensity(0.0f);
    Region container;
    Region glowing;
    Region neighbor;
    canvas.addRegion(&container);
    container.setNeedsLayer(true);
    container.setBounds(0, 0, 64, 32);
    container.addRegion(&glowing);
    glowing.setNeedsLayer(true);
    glowing.setPostEffect(&bloom);
    glowing.setBounds(24, 8, 16, 16);
    canvas.addRegion(&neighbor);
    neighbor.setNeedsLayer(true);
    neighbor.setBounds(80, 0, 16, 32);
    if (hdr_parent)
      neighbor.setPostEffect(&neighbor_bloom);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&container);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, container.width(), container.height());
    canvas.endRegion();
    canvas.beginRegion(&glowing);
    canvas.setColor(Color(0xffffffff, 2.0f));
    canvas.fill(4, 4, 8, 8);
    canvas.endRegion();
    canvas.beginRegion(&neighbor);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, neighbor.width(), neighbor.height());
    canvas.endRegion();
    return canvas.takeScreenshot();
  };

  Screenshot sdr_parent = render(false);
  Screenshot hdr_parent = render(true);
  REQUIRE(sdr_parent.sample(26, 16).hexRed() > 0);
  for (int x : { 25, 26, 27, 37, 38 }) {
    INFO("x " << x);
    REQUIRE(hdr_parent.sample(x, 16).hexRed() == Approx(sdr_parent.sample(x, 16).hexRed()).margin(2));
  }
}

TEST_CASE("Translucent region sharing a layer with bloom keeps its alpha", "[graphics]") {
  auto render = [](bool with_bloom) {
    Canvas canvas;
    canvas.setWindowless(64, 32);
    BloomPostEffect bloom;
    Region soft;
    Region glowing;
    canvas.addRegion(&soft);
    soft.setNeedsLayer(true);
    soft.setBounds(0, 0, 32, 32);
    canvas.addRegion(&glowing);
    glowing.setNeedsLayer(true);
    glowing.setBounds(32, 0, 32, 32);
    if (with_bloom)
      glowing.setPostEffect(&bloom);

    canvas.setColor(0xff808080);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&soft);
    canvas.setColor(0x80000000);
    canvas.fill(0, 0, soft.width(), soft.height());
    canvas.endRegion();
    canvas.beginRegion(&glowing);
    canvas.setColor(0xff202020);
    canvas.fill(0, 0, glowing.width(), glowing.height());
    canvas.endRegion();
    return canvas.takeScreenshot().sample(16, 16).hexRed();
  };

  REQUIRE(render(true) == Approx(render(false)).margin(2));
}

TEST_CASE("Multiplied region keeps its strength in an HDR layer", "[graphics]") {
  auto render = [](bool hdr_layer) {
    Canvas canvas;
    canvas.setWindowless(64, 32);
    BloomPostEffect bloom;
    bloom.setBloomIntensity(0.0f);
    Region background;
    Region multiplied;
    Region neighbor;
    canvas.addRegion(&background);
    background.setNeedsLayer(true);
    background.setBounds(0, 0, 32, 32);
    background.addRegion(&multiplied);
    multiplied.setNeedsLayer(true);
    multiplied.setLayerBlendMode(BlendMode::Mult);
    multiplied.setBounds(8, 8, 16, 16);
    canvas.addRegion(&neighbor);
    neighbor.setNeedsLayer(true);
    neighbor.setBounds(40, 0, 24, 32);
    if (hdr_layer)
      neighbor.setPostEffect(&bloom);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&background);
    canvas.setColor(0xffc0c0c0);
    canvas.fill(0, 0, background.width(), background.height());
    canvas.endRegion();
    canvas.beginRegion(&multiplied);
    canvas.setColor(0xff808080);
    canvas.fill(0, 0, multiplied.width(), multiplied.height());
    canvas.endRegion();
    canvas.beginRegion(&neighbor);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, neighbor.width(), neighbor.height());
    canvas.endRegion();
    return canvas.takeScreenshot().sample(16, 16).hexRed();
  };

  REQUIRE(render(false) == Approx(0x60).margin(2));
  REQUIRE(render(true) == Approx(0x60).margin(2));
}

TEST_CASE("Bloom glows brighter for colors above four times white", "[graphics]") {
  auto glow = [](float hdr) {
    Canvas canvas;
    canvas.setWindowless(64, 32);
    BloomPostEffect bloom;
    bloom.setBloomSize(8.0f);
    bloom.setBloomIntensity(0.2f);
    Region glowing;
    canvas.addRegion(&glowing);
    glowing.setNeedsLayer(true);
    glowing.setPostEffect(&bloom);
    glowing.setBounds(0, 0, 64, 32);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&glowing);
    canvas.setColor(0xff000000);
    canvas.fill(0, 0, glowing.width(), glowing.height());
    canvas.setColor(Color(0xffffffff, hdr));
    canvas.fill(28, 12, 8, 8);
    canvas.endRegion();
    return canvas.takeScreenshot().sample(38, 16).hexRed();
  };

  int four_times_white = glow(4.0f);
  int eight_times_white = glow(8.0f);
  REQUIRE(four_times_white > 8);
  REQUIRE(eight_times_white < 0xff);
  REQUIRE(eight_times_white > four_times_white + 10);
}

TEST_CASE("Canvas draws a batch too big for 16-bit indices", "[graphics]") {
  static constexpr int kWidth = 200;
  static constexpr int kHeight = 100;
  Canvas canvas;
  canvas.setWindowless(kWidth, kHeight);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kWidth, kHeight);
  canvas.setColor(0xffffffff);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x)
      canvas.rectangle(x, y, 1, 1);
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(screenshot.sample(0, 0).hexRed() == 0xff);
  REQUIRE(screenshot.sample(kWidth - 1, kHeight - 1).hexRed() == 0xff);
}

TEST_CASE("Buffer pools fail cleanly when the GPU runs out of buffer handles", "[graphics]") {
  static constexpr int kIdleFrames = 130;
  Canvas canvas;
  canvas.setWindowless(20, 20);
  canvas.takeScreenshot();
  for (int i = 0; i < kIdleFrames; ++i)
    canvas.submit();
  REQUIRE(bufferPoolCapacity(UvVertex::layout()) == 0);

  std::vector<gpu::BufferHandle> handles;
  for (gpu::BufferHandle handle = gpu::createDynamicVertexBuffer(1, UvVertex::layout());
       gpu::isValid(handle); handle = gpu::createDynamicVertexBuffer(1, UvVertex::layout()))
    handles.push_back(handle);

  REQUIRE(initQuadVertices<UvVertex>(1) == nullptr);
  for (gpu::BufferHandle handle : handles)
    gpu::destroy(handle);
  canvas.takeScreenshot();

  canvas.setColor(0xffffffff);
  canvas.fill(0, 0, 20, 20);
  REQUIRE(canvas.takeScreenshot().sample(10, 10).hexRed() == 0xff);
}

TEST_CASE("Buffer pools persist across frames and are released when idle", "[graphics]") {
  static constexpr int kWidth = 200;
  static constexpr int kHeight = 100;
  static constexpr int kIdleFrames = 130;
  auto num_dynamic_buffers = []() {
    gpu::frame();
    gpu::frame();
    return gpu::getStats()->num_buffers;
  };

  Canvas canvas;
  canvas.setWindowless(kWidth, kHeight);
  for (int i = 0; i < kIdleFrames; ++i)
    canvas.submit();
  int baseline = num_dynamic_buffers();

  canvas.setColor(0xffffffff);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x)
      canvas.rectangle(x, y, 1, 1);
  }
  canvas.takeScreenshot();
  int with_pools = num_dynamic_buffers();
  REQUIRE(with_pools > baseline);
  for (int i = 0; i < 5; ++i)
    REQUIRE(canvas.takeScreenshot().sample(kWidth - 1, kHeight - 1).hexRed() == 0xff);
  REQUIRE(num_dynamic_buffers() == with_pools);

  canvas.clearDrawnShapes();
  for (int i = 0; i < kIdleFrames; ++i)
    canvas.submit();
  REQUIRE(num_dynamic_buffers() == baseline);
}

TEST_CASE("Buffer pools shrink to recent use and are released when unused", "[graphics]") {
  static constexpr int kUsageFrames = 120;
  Canvas canvas;
  canvas.setWindowless(20, 20);
  canvas.submit();

  auto frame = [](int num_quads) {
    beginBufferPoolFrame();
    if (num_quads)
      REQUIRE(initQuadVertices<UvVertex>(num_quads) != nullptr);
    gpu::discard();
    gpu::frame();
    endBufferPoolFrame();
  };
  auto capacity = [] { return bufferPoolCapacity(UvVertex::layout()); };
  auto staging = [] { return bufferPoolStagingBytes(UvVertex::layout()); };
  uint64_t stride = UvVertex::layout().stride();

  for (int i = 0; i < 3 * kUsageFrames && capacity(); ++i)
    frame(0);
  REQUIRE(capacity() == 0);

  frame(200000);
  REQUIRE(capacity() == 200000 * kVerticesPerQuad);
  for (int i = 0; i < 2 * kUsageFrames; ++i)
    frame(20000);
  REQUIRE(capacity() == 1 << 18);
  for (int i = 0; i < kUsageFrames; ++i)
    frame(20000);
  REQUIRE(capacity() == 1 << 18);
  REQUIRE(staging() > 0);
  REQUIRE(staging() <= 2 * capacity() * stride);

  for (int i = 0; i < 2 * kUsageFrames; ++i)
    frame(0);
  REQUIRE(capacity() == 0);
  REQUIRE(staging() == 0);
}

TEST_CASE("Shapes drawn before a buffer pool grows mid-frame still render", "[graphics]") {
  static constexpr int kWidth = 200;
  static constexpr int kBand = 100;
  Canvas canvas;
  canvas.setWindowless(kWidth, 3 * kBand);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kWidth, 3 * kBand);
  canvas.setColor(0xffffffff);
  for (int y = 0; y < kBand; ++y) {
    for (int x = 0; x < kWidth; ++x)
      canvas.rectangle(x, y, 1, 1);
  }
  for (int y = 0; y < kBand; ++y) {
    for (int x = 0; x < kWidth; ++x)
      canvas.circle(x, kBand + y, 1);
  }
  for (int y = 0; y < kBand; ++y) {
    for (int x = 0; x < kWidth; ++x)
      canvas.roundedRectangle(x, 2 * kBand + y, 1, 1, 0.25f);
  }

  for (int frame = 0; frame < 3; ++frame) {
    const Screenshot& screenshot = canvas.takeScreenshot();
    for (int band = 0; band < 3; ++band) {
      for (int y : { 0, kBand / 2, kBand - 1 }) {
        for (int x : { 0, kWidth / 2, kWidth - 1 }) {
          INFO("frame " << frame << " band " << band << " x " << x << " y " << y);
          REQUIRE(screenshot.sample(x, band * kBand + y).hexRed() > 0x80);
        }
      }
    }
  }
}

TEST_CASE("A batch drawn before its buffer pool grows mid-frame keeps its vertices", "[graphics]") {
  static constexpr int kCircles = 2000;
  static constexpr int kMaxIdleFrames = 400;
  Canvas canvas;
  canvas.setWindowless(100, 100);
  auto capacity = [] { return bufferPoolCapacity(ShapeVertex::layout()); };
  for (int i = 0; i < kMaxIdleFrames && capacity(); ++i)
    canvas.submit();
  REQUIRE(capacity() == 0);

  canvas.setColor(0xffff0000);
  canvas.rectangle(0, 0, 20, 20);
  canvas.setColor(0xff0000ff);
  for (int i = 0; i < kCircles; ++i)
    canvas.circle(10 + i % 80, 10 + i / 80, 2);

  const Screenshot& screenshot = canvas.takeScreenshot();
  REQUIRE(capacity() >= (kCircles + 1) * kVerticesPerQuad);
  REQUIRE(screenshot.sample(5, 5).hexRed() == 0xff);
  REQUIRE(screenshot.sample(5, 5).hexBlue() == 0);
}

TEST_CASE("Every batch in a frame reaches the GPU when the frame is submitted", "[graphics]") {
  static constexpr int kBatches = 40;
  static constexpr int kSize = 8;
  Canvas canvas;
  canvas.setWindowless(kBatches * kSize, kSize);
  Region region;
  canvas.addRegion(&region);
  region.setNeedsLayer(true);
  region.setBounds(0, 0, kBatches * kSize, kSize);

  auto red = [](int batch, int frame) { return (batch + frame) % 2 == 0; };
  for (int frame = 0; frame < 4; ++frame) {
    region.clear();
    region.invalidate();
    canvas.beginRegion(&region);
    for (int i = 0; i < kBatches; ++i) {
      canvas.setColor(red(i, frame) ? 0xffff0000 : 0xff0000ff);
      if (i % 2)
        canvas.circle(i * kSize, 0, kSize);
      else
        canvas.rectangle(i * kSize, 0, kSize, kSize);
    }
    canvas.endRegion();
    canvas.submit();

    const Screenshot& screenshot = canvas.takeScreenshot();
    for (int i = 0; i < kBatches; ++i) {
      INFO("frame " << frame << " batch " << i);
      Color sample = screenshot.sample(i * kSize + kSize / 2, kSize / 2);
      REQUIRE(sample.hexRed() == (red(i, frame) ? 0xff : 0));
      REQUIRE(sample.hexBlue() == (red(i, frame) ? 0 : 0xff));
    }
  }
}

TEST_CASE("Buffer pools keep staging when later splits leave every block idle", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(20, 20);
  canvas.submit();

  int shape_quads = maxQuadsPerChunk(sizeof(ShapeVertex));
  int text_quads = maxQuadsPerChunk(sizeof(TextureVertex));
  auto draw_chunks = [&] {
    for (int i = 0; i < 3; ++i) {
      gpu::discard();
      reserveBufferPoolBytes(uint64_t(shape_quads) * kVerticesPerQuad * sizeof(ShapeVertex));
      REQUIRE(initQuadVertices<ShapeVertex>(shape_quads) != nullptr);
    }
    for (int i = 0; i < 2; ++i) {
      gpu::discard();
      reserveBufferPoolBytes(uint64_t(text_quads) * kVerticesPerQuad * sizeof(TextureVertex));
      REQUIRE(initQuadVertices<TextureVertex>(text_quads) != nullptr);
    }
    gpu::discard();
    gpu::frame();
    endBufferPoolFrame();
  };
  auto staging = [] { return bufferPoolStagingBytes(ShapeVertex::layout()); };

  for (int i = 0; i < 5; ++i) {
    beginBufferPoolFrame();
    draw_chunks();
  }
  for (int i = 0; i < 5; ++i) {
    uint64_t before = staging();
    beginBufferPoolFrame();
    REQUIRE(staging() == before);
    draw_chunks();
  }
}

TEST_CASE("Shapes past a buffer pool chunk split the frame and all render", "[graphics]") {
  static constexpr int kWidth = 200;
  static constexpr int kHeight = 100;
  REQUIRE(kWidth * kHeight * kVerticesPerQuad * sizeof(ShapeVertex) > 2 * kBufferPoolChunkBytes);

  Canvas canvas;
  canvas.setWindowless(kWidth, kHeight);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kWidth, kHeight);
  canvas.setColor(0xffffffff);
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x)
      canvas.rectangle(x, y, 1, 1);
  }

  for (int frame = 0; frame < 3; ++frame) {
    const Screenshot& screenshot = canvas.takeScreenshot();
    int unfilled = 0;
    for (int y = 0; y < kHeight; ++y) {
      for (int x = 0; x < kWidth; ++x)
        unfilled += screenshot.sample(x, y).hexRed() != 0xff;
    }
    REQUIRE(unfilled == 0);
  }
  REQUIRE(bufferPoolCapacity(ShapeVertex::layout()) * sizeof(ShapeVertex) <= kBufferPoolChunkBytes);
}

TEST_CASE("Text past a buffer pool chunk splits the frame and all renders", "[graphics]") {
  static constexpr int kColumns = 200;
  static constexpr int kRows = 120;
  static constexpr int kCell = 8;
  REQUIRE(kColumns * kRows * kVerticesPerQuad * sizeof(TextureVertex) > 2 * kBufferPoolChunkBytes);

  Font font(kCell, fonts::Lato_Regular_ttf_data, sizeof(fonts::Lato_Regular_ttf_data));
  Canvas canvas;
  canvas.setWindowless(kColumns * kCell, kRows * kCell);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kColumns * kCell, kRows * kCell);
  canvas.setColor(0xffffffff);
  for (int i = 0; i < kColumns * kRows; ++i) {
    canvas.text("W", font, Font::Justification::kCenter, (i % kColumns) * kCell,
                (i / kColumns) * kCell, kCell, kCell);
  }

  const Screenshot& screenshot = canvas.takeScreenshot();
  int ink = 0;
  for (int y = 0; y < kCell; ++y) {
    for (int x = 0; x < kCell; ++x)
      ink += screenshot.sample(x, y).hexRed();
  }
  REQUIRE(ink > 0);

  int mismatched_cells = 0;
  for (int i = 1; i < kColumns * kRows; ++i) {
    int cell_x = (i % kColumns) * kCell;
    int cell_y = (i / kColumns) * kCell;
    bool matches = true;
    for (int y = 0; y < kCell && matches; ++y) {
      for (int x = 0; x < kCell && matches; ++x)
        matches = screenshot.sample(cell_x + x, cell_y + y).hexRed() == screenshot.sample(x, y).hexRed();
    }
    mismatched_cells += !matches;
  }
  REQUIRE(mismatched_cells == 0);
  REQUIRE(bufferPoolCapacity(TextureVertex::layout()) * sizeof(TextureVertex) <= kBufferPoolChunkBytes);
}

TEST_CASE("A path with 60000 points fills correctly", "[graphics]") {
  static constexpr int kSize = 200;
  static constexpr int kPoints = 60000;
  static constexpr float kRadius = 80.0f;
  static constexpr float kCenter = kSize / 2.0f;

  Canvas canvas;
  canvas.setWindowless(kSize, kSize);
  canvas.setColor(0xff000000);
  canvas.fill(0, 0, kSize, kSize);

  Path path;
  for (int i = 0; i < kPoints; ++i) {
    float phase = i * 2.0f * kPi / kPoints;
    float x = kCenter + kRadius * std::cos(phase);
    float y = kCenter + kRadius * std::sin(phase);
    if (i == 0)
      path.moveTo(x, y);
    else
      path.lineTo(x, y);
  }
  path.close();
  canvas.setColor(0xffffffff);
  canvas.fill(path);

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (int i = 0; i < 8; ++i) {
    float phase = i * kPi / 4.0f;
    for (float radius : { 0.0f, 40.0f, 70.0f }) {
      int x = kCenter + radius * std::cos(phase);
      int y = kCenter + radius * std::sin(phase);
      INFO("inside " << x << ", " << y);
      REQUIRE(screenshot.sample(x, y).hexRed() == 0xff);
    }
    int x = kCenter + 90.0f * std::cos(phase);
    int y = kCenter + 90.0f * std::sin(phase);
    INFO("outside " << x << ", " << y);
    REQUIRE(screenshot.sample(x, y).hexRed() == 0);
  }
}

#if VISAGE_LINUX
TEST_CASE("Large frame before exit child", "[.large_frame_exit_child]") {
  static constexpr int kWidth = 400;
  static constexpr int kHeight = 200;
  {
    Canvas canvas;
    canvas.setWindowless(kWidth, kHeight);
    auto draw = [&] {
      canvas.setColor(0xffffffff);
      for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x)
          canvas.rectangle(x, y, 1, 1);
      }
    };
    draw();
    canvas.takeScreenshot();
    canvas.clearDrawnShapes();
    draw();
    canvas.submit();
  }
  std::exit(0);
}

class ScopedEnvironmentVariable {
public:
  ScopedEnvironmentVariable(const char* name, const std::string& value) : name_(name) {
    const char* previous = std::getenv(name);
    had_previous_ = previous != nullptr;
    previous_ = had_previous_ ? previous : "";
    setenv(name, value.c_str(), 1);
  }

  ~ScopedEnvironmentVariable() {
    if (had_previous_)
      setenv(name_, previous_.c_str(), 1);
    else
      unsetenv(name_);
  }

private:
  const char* name_ = nullptr;
  bool had_previous_ = false;
  std::string previous_;
};

TEST_CASE("Exiting right after a large frame doesn't crash", "[graphics]") {
  const char* asan_options = std::getenv("ASAN_OPTIONS");
  std::string child_asan_options = std::string(asan_options ? asan_options : "") +
                                   ":detect_leaks=0";
  ScopedEnvironmentVariable no_leak_check("ASAN_OPTIONS", child_asan_options);
  ScopedEnvironmentVariable no_vulkan_layers("VK_LOADER_LAYERS_DISABLE", "~all~");
  std::string output;
  bool exited_cleanly = spawnChildProcess(hostExecutable().string(), "[.large_frame_exit_child]",
                                          output, 60000);
  INFO(output);
  REQUIRE(exited_cleanly);
}

#ifdef VISAGE_GRAPHICS_TEST_MODULE
static void callTestModuleBeforeExit(const char* function_name) {
  void* module = dlopen(VISAGE_GRAPHICS_TEST_MODULE, RTLD_NOW | RTLD_LOCAL);
  REQUIRE(module);
  auto function = reinterpret_cast<void (*)()>(dlsym(module, function_name));
  REQUIRE(function);
  function();
  dlclose(module);
  std::exit(0);
}

TEST_CASE("Test module canvas child", "[.test_module_canvas_child]") {
  callTestModuleBeforeExit("createAndDestroyCanvas");
}

TEST_CASE("Test module leaked canvas child", "[.test_module_leaked_canvas_child]") {
  callTestModuleBeforeExit("leakCanvas");
}

TEST_CASE("Leaked canvas child", "[.leaked_canvas_child]") {
  new Canvas();
  std::exit(0);
}

TEST_CASE("Unloading or exiting with a canvas still alive trips a debug assert", "[graphics]") {
  const char* asan_options = std::getenv("ASAN_OPTIONS");
  std::string child_asan_options = std::string(asan_options ? asan_options : "") +
                                   ":detect_leaks=0";
  ScopedEnvironmentVariable no_leak_check("ASAN_OPTIONS", child_asan_options);
  std::string output;
  bool exited_cleanly = spawnChildProcess(hostExecutable().string(), "[.test_module_canvas_child]",
                                          output, 60000);
  INFO(output);
  REQUIRE(exited_cleanly);

  for (const char* child : { "[.test_module_leaked_canvas_child]", "[.leaked_canvas_child]" }) {
    std::string output;
    bool exited_cleanly = spawnChildProcess(hostExecutable().string(), child, output, 60000);
    INFO(child << "\n" << output);
#ifdef NDEBUG
    REQUIRE(exited_cleanly);
#else
    REQUIRE_FALSE(exited_cleanly);
    REQUIRE(output.find("renderer.cpp") != std::string::npos);
    REQUIRE(output.find("Assertion failed") != std::string::npos);
    REQUIRE(output.find("defines.h") == std::string::npos);
#endif
  }
}
#endif

TEST_CASE("Static canvas child", "[.static_canvas_child]") {
  static Canvas canvas;
  std::exit(0);
}

TEST_CASE("Static bloom effect child", "[.static_bloom_child]") {
  static BloomPostEffect bloom;
  std::exit(0);
}

TEST_CASE("Static canvases and bloom effects trip a debug assert", "[graphics]") {
  for (auto [child, file] : { std::pair { "[.static_canvas_child]", "canvas.cpp" },
                              std::pair { "[.static_bloom_child]", "post_effects.cpp" } }) {
    std::string output;
    bool exited_cleanly = spawnChildProcess(hostExecutable().string(), child, output, 60000);
    INFO(child << "\n" << output);
#ifdef NDEBUG
    REQUIRE(exited_cleanly);
#else
    REQUIRE_FALSE(exited_cleanly);
    REQUIRE(output.find(file) != std::string::npos);
    REQUIRE(output.find("Assertion failed") != std::string::npos);
#endif
  }
}
#endif

#if !VISAGE_EMSCRIPTEN
static int zeroed_static_value = 0;
static int initialized_static_value = 1;

TEST_CASE("Only static addresses count as static storage", "[graphics]") {
  static int local_static_value = 0;
  REQUIRE(hasStaticStorage(&zeroed_static_value));
  REQUIRE(hasStaticStorage(&initialized_static_value));
  REQUIRE(hasStaticStorage(&local_static_value));
#if VISAGE_LINUX
  REQUIRE(hasStaticStorage(dlsym(RTLD_DEFAULT, "gnu_get_libc_version")));
#endif

  int stack_value = 0;
  auto heap_value = std::make_unique<int>(0);
  REQUIRE_FALSE(hasStaticStorage(&stack_value));
  REQUIRE_FALSE(hasStaticStorage(heap_value.get()));
}
#endif

struct CanvasHolder {
  int value = 0;
  Canvas canvas;
};

TEST_CASE("Stack and heap canvases don't trip the static canvas assert", "[graphics]") {
  Canvas stack_canvas;
  auto heap_canvas = std::make_unique<Canvas>();
  auto holder = std::make_unique<CanvasHolder>();
  REQUIRE_FALSE(hasStaticStorage(&stack_canvas));
  REQUIRE_FALSE(hasStaticStorage(heap_canvas.get()));
  REQUIRE_FALSE(hasStaticStorage(&holder->canvas));
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

TEST_CASE("Removing a region invalidates the area it covered in its parent", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(100, 100);
  Region parent;
  Region child;
  canvas.addRegion(&parent);
  parent.setNeedsLayer(true);
  parent.setBounds(0, 0, 100, 100);
  parent.addRegion(&child);
  child.setBounds(10, 20, 30, 40);
  canvas.takeScreenshot();

  Layer* layer = parent.layer();
  REQUIRE(layer->invalidRects().count(&parent) == 0);

  parent.removeRegion(&child);
  REQUIRE(layer->invalidRects().count(&parent) == 1);

  IBounds covered;
  for (const IBounds& rect : layer->invalidRects().at(&parent))
    covered = covered.width() ? covered.unioned(rect) : rect;
  REQUIRE(covered.contains(IBounds(10, 20, 30, 40)));
}

TEST_CASE("Rounded triangle with nearly collinear points draws nothing", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(80, 80);
  Region region;
  canvas.addRegion(&region);
  region.setBounds(0, 0, 3010, 3010);

  canvas.beginRegion(&region);
  canvas.setColor(0xffffffff);
  canvas.roundedTriangle(0.1f, 0.1f, 1000.3f, 1000.3f, 3000.7f, 3000.7f, 3);
  canvas.endRegion();

  const Screenshot& screenshot = canvas.takeScreenshot();
  int max_red = 0;
  for (int i = 0; i < 80; ++i) {
    for (int x = std::max(0, i - 3); x <= std::min(79, i + 3); ++x)
      max_red = std::max<int>(max_red, screenshot.sample(x, i).hexRed());
  }
  REQUIRE(max_red == 0);
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

TEST_CASE("Gradient brush points follow Canvas::setPosition like shapes", "[graphics]") {
  Brush linear = Brush::linear(Color(0xffff0000), Color(0xff0000ff), Point(0, 0), Point(100, 0));
  Canvas canvas;
  canvas.setWindowless(200, 40);

  auto require_red_to_blue = [&](int left) {
    const Screenshot& screenshot = canvas.takeScreenshot();
    INFO("left " << left);
    REQUIRE(screenshot.sample(left + 5, 20).hexRed() >= 0xe0);
    REQUIRE(screenshot.sample(left + 5, 20).hexBlue() <= 0x20);
    REQUIRE(screenshot.sample(left + 50, 20).hexRed() == Approx(0x80).margin(4));
    REQUIRE(screenshot.sample(left + 50, 20).hexBlue() == Approx(0x80).margin(4));
    REQUIRE(screenshot.sample(left + 95, 20).hexRed() <= 0x20);
    REQUIRE(screenshot.sample(left + 95, 20).hexBlue() >= 0xe0);
  };

  SECTION("Brush set after the position") {
    canvas.setPosition(100, 0);
    canvas.setColor(linear);
    canvas.rectangle(0, 0, 100, 40);
    require_red_to_blue(100);
  }

  SECTION("Brush set before the position keeps the position it was set at") {
    canvas.setColor(linear);
    canvas.setPosition(100, 0);
    canvas.rectangle(-100, 0, 100, 40);
    require_red_to_blue(0);
  }

  SECTION("Radial brush") {
    canvas.setPosition(100, 0);
    canvas.setColor(Brush::radial(Color(0xffff0000), Color(0xff0000ff), Point(50, 20), 50.0f));
    canvas.rectangle(0, 0, 100, 40);
    const Screenshot& screenshot = canvas.takeScreenshot();
    REQUIRE(screenshot.sample(150, 20).hexRed() >= 0xe0);
    REQUIRE(screenshot.sample(150, 20).hexBlue() <= 0x20);
    REQUIRE(screenshot.sample(198, 20).hexRed() <= 0x20);
    REQUIRE(screenshot.sample(198, 20).hexBlue() >= 0xe0);
  }

  SECTION("SVG gradient") {
    std::string svg_data = R"svg(<svg viewBox="0 0 100 40" width="100" height="40">
      <linearGradient id="g" gradientUnits="userSpaceOnUse" x1="0" y1="0" x2="100" y2="0">
        <stop offset="0" stop-color="#ff0000"/><stop offset="1" stop-color="#0000ff"/>
      </linearGradient>
      <rect width="100" height="40" fill="url(#g)"/>
    </svg>)svg";
    Svg svg(reinterpret_cast<const unsigned char*>(svg_data.data()), static_cast<int>(svg_data.size()));
    canvas.setPosition(100, 0);
    canvas.svg(svg, 0, 0, 100, 40);
    require_red_to_blue(100);
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

TEST_CASE("Strokes at a DPI scale match the same stroke drawn at the scaled size", "[graphics]") {
  auto testPath = [](float k) {
    Path path;
    path.moveTo(4 * k, 36 * k);
    path.lineTo(20 * k, 2 * k);
    path.quadraticTo(40 * k, 10 * k, 36 * k, 36 * k);
    path.lineTo(20 * k, 24 * k);
    path.close();
    return path;
  };

  for (float dpi : { 2.0f, 1.5f }) {
    for (bool dashed : { false, true }) {
      INFO("DPI " << dpi << (dashed ? ", dashed" : ""));
      int size = std::round(60 * dpi);
      auto render = [&](float canvas_dpi, float k) {
        Canvas canvas;
        canvas.setDpiScale(canvas_dpi);
        canvas.setWindowless(size, size);
        canvas.setNativePixelScale();
        canvas.setColor(0xff000000);
        canvas.fill(0, 0, size, size);
        canvas.setLogicalPixelScale();
        canvas.setColor(0xffffffff);
        std::vector<float> dashes;
        if (dashed)
          dashes = { 6 * k, 4 * k };
        canvas.stroke(testPath(k), 10 * k, 10 * k, 40 * k, 40 * k, 3 * k, Path::Join::Round,
                      Path::EndCap::Round, dashes, 2 * k);
        return canvas.takeScreenshot();
      };

      Screenshot expected = render(1.0f, dpi);
      Screenshot actual = render(dpi, 1.0f);
      int max_difference = 0;
      for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
          int difference = std::abs(static_cast<int>(expected.sample(x, y).hexRed()) -
                                    static_cast<int>(actual.sample(x, y).hexRed()));
          max_difference = std::max(max_difference, difference);
        }
      }
      REQUIRE(max_difference <= 2);
    }
  }
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
    bool bgra;
    bool y_flip;
  };

  static const uint8_t kBgraBottomUp[] = { 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff };
  static const uint8_t kRgbaTopDown[] = { 0xff, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff };
  for (const Capture& capture : { Capture { kBgraBottomUp, true, true }, Capture { kRgbaTopDown, false, false } }) {
    INFO("bgra " << capture.bgra);
    Renderer::instance().setScreenshotData(static_cast<const uint8_t*>(capture.data), 1, 2, 4,
                                           capture.bgra, capture.y_flip);
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
  REQUIRE(layer->width() <= gpu::getCaps()->max_texture_size);
  REQUIRE(layer->height() <= gpu::getCaps()->max_texture_size);
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

TEST_CASE("Graph fill with a single point has a sharp top edge", "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(120, 80);
  GraphData data(1);
  data[0] = 0.5f;
  canvas.setColor(0xffffffff);
  canvas.graphFill(data, 3, 2, 97, 71, 1.0f);
  canvas.graphLine(data, 3, 2, 97, 71, 1.5f);
  canvas.submit();

  const Screenshot& screenshot = canvas.takeScreenshot();
  for (int x : { 10, 50, 90 }) {
    INFO("x " << x);
    int partial = 0;
    for (int y = 0; y < 80; ++y) {
      int red = screenshot.sample(x, y).hexRed();
      if (red > 8 && red < 247)
        ++partial;
    }
    REQUIRE(partial <= 2);
    REQUIRE(screenshot.sample(x, 10).hexRed() == 0);
    REQUIRE(screenshot.sample(x, 70).hexRed() == 0xff);
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
  REQUIRE(gpu::getStats()->num_frame_buffers < 200);
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
  gpu::FrameBufferHandle handles[kSizes];

  for (int submit = 0; submit < 4 * kPeriod; ++submit) {
    for (int i = submit % kPeriod; i < kSizes; i += kPeriod) {
      gpu::FrameBufferHandle handle = layer->temporaryFrameBuffer(16 + i, 16);
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
  REQUIRE(gpu::getStats()->num_frame_buffers < 200);
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

TEST_CASE("Shader data the GPU layer rejects leaves an invalid cached handle", "[graphics]") {
  static constexpr unsigned char kNotAShader[] = { 'b', 'a', 'd', ' ', 'd', 'a', 't', 'a' };
  static const EmbeddedFile kFile = { "not_a_shader", kNotAShader, sizeof(kNotAShader) };

  Canvas canvas;
  canvas.setWindowless(8, 8);
  REQUIRE_FALSE(gpu::isValid(ShaderCache::shaderHandle(kFile)));
  REQUIRE_FALSE(gpu::isValid(ShaderCache::shaderHandle(kFile)));
}

TEST_CASE("Subtracted shapes over transparent pixels don't darken the HDR layer below",
          "[graphics]") {
  Canvas canvas;
  canvas.setWindowless(64, 32);
  BloomPostEffect outer_bloom;
  outer_bloom.setBloomIntensity(0.0f);
  BloomPostEffect inner_bloom;
  inner_bloom.setBloomIntensity(0.0f);
  Region outer;
  Region inner;
  canvas.addRegion(&outer);
  outer.setNeedsLayer(true);
  outer.setPostEffect(&outer_bloom);
  outer.setBounds(0, 0, 64, 32);
  outer.addRegion(&inner);
  inner.setNeedsLayer(true);
  inner.setPostEffect(&inner_bloom);
  inner.setBounds(0, 0, 64, 32);

  canvas.setColor(0xff000000);
  canvas.fill(0, 0, canvas.width(), canvas.height());
  canvas.beginRegion(&outer);
  canvas.setColor(0xff808080);
  canvas.fill(0, 0, outer.width(), outer.height());
  canvas.endRegion();
  canvas.beginRegion(&inner);
  canvas.setBlendMode(BlendMode::Sub);
  canvas.setColor(0xffffffff);
  canvas.fill(16, 8, 32, 16);
  canvas.endRegion();

  REQUIRE(canvas.takeScreenshot().sample(32, 16).hexRed() == Approx(0x80).margin(2));
}

TEST_CASE("Bloom doesn't glow dark around subtracted shapes", "[graphics]") {
  auto render = [](bool subtract) {
    Canvas canvas;
    canvas.setWindowless(96, 32);
    BloomPostEffect outer_bloom;
    outer_bloom.setBloomIntensity(0.0f);
    BloomPostEffect bloom;
    bloom.setBloomSize(8.0f);
    bloom.setBloomIntensity(0.5f);
    Region outer;
    Region glowing;
    canvas.addRegion(&outer);
    outer.setNeedsLayer(true);
    outer.setPostEffect(&outer_bloom);
    outer.setBounds(0, 0, 96, 32);
    outer.addRegion(&glowing);
    glowing.setNeedsLayer(true);
    glowing.setPostEffect(&bloom);
    glowing.setBounds(0, 0, 96, 32);

    canvas.setColor(0xff000000);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.beginRegion(&outer);
    canvas.setColor(0xff808080);
    canvas.fill(0, 0, outer.width(), outer.height());
    canvas.endRegion();
    canvas.beginRegion(&glowing);
    canvas.setColor(0xffeeeeee);
    canvas.fill(0, 0, 48, 32);
    if (subtract) {
      canvas.setBlendMode(BlendMode::Sub);
      canvas.setColor(0xffffffff);
      canvas.fill(32, 8, 16, 16);
    }
    canvas.endRegion();
    return canvas.takeScreenshot().sample(52, 16).hexRed();
  };

  REQUIRE(render(false) == Approx(0x80).margin(2));
  REQUIRE(render(true) == Approx(render(false)).margin(2));
}
