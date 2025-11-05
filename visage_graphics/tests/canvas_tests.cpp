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