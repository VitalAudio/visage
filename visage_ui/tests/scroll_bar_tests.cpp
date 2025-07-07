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

#include "visage_ui/scroll_bar.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

TEST_CASE("ScrollBar construction and initialization", "[ui]") {
  ScrollBar scroll_bar;

  SECTION("Default state") {
    REQUIRE(scroll_bar.viewRange() == 0);
    REQUIRE(scroll_bar.viewHeight() == 0);
  }

  SECTION("Initial dimensions") {
    scroll_bar.setBounds(0, 0, 20, 100);
    REQUIRE(scroll_bar.width() == 20);
    REQUIRE(scroll_bar.height() == 100);
  }
}

TEST_CASE("ScrollBar position and view management", "[ui]") {
  ScrollBar scroll_bar;

  SECTION("Setting position") {
    scroll_bar.setPosition(50);
  }

  SECTION("Setting view position with range smaller than view height") {
    scroll_bar.setViewPosition(100, 150, 0);
    REQUIRE(scroll_bar.viewRange() == 100);
    REQUIRE(scroll_bar.viewHeight() == 150);
    REQUIRE(scroll_bar.ignoresMouseEvents());
  }

  SECTION("Setting view position with range larger than view height") {
    scroll_bar.setViewPosition(200, 100, 25);
    REQUIRE(scroll_bar.viewRange() == 200);
    REQUIRE(scroll_bar.viewHeight() == 100);
    REQUIRE_FALSE(scroll_bar.ignoresMouseEvents());
  }

  SECTION("View position edge cases") {
    scroll_bar.setViewPosition(0, 0, 0);
    REQUIRE(scroll_bar.viewRange() == 0);
    REQUIRE(scroll_bar.viewHeight() == 0);
  }
}

TEST_CASE("ScrollBar callbacks", "[ui]") {
  ScrollBar scroll_bar;
  float last_callback_value = -1.0f;
  bool callback_called = false;

  scroll_bar.addScrollCallback([&](float value) {
    last_callback_value = value;
    callback_called = true;
  });

  SECTION("Multiple callbacks") {
    float second_callback_value = -1.0f;
    bool second_callback_called = false;

    scroll_bar.addScrollCallback([&](float value) {
      second_callback_value = value;
      second_callback_called = true;
    });
  }
}

TEST_CASE("ScrollBar appearance properties", "[ui]") {
  ScrollBar scroll_bar;

  SECTION("Setting rounding") {
    scroll_bar.setRounding(5.0f);
  }

  SECTION("Setting left side") {
    scroll_bar.setLeftSide(true);
    scroll_bar.setLeftSide(false);
  }
}

TEST_CASE("ScrollBar mouse events", "[ui]") {
  ScrollBar scroll_bar;
  scroll_bar.setBounds(0, 0, 20, 100);
  scroll_bar.setViewPosition(200, 100, 0);

  SECTION("Mouse enter event") {
    MouseEvent event;
    scroll_bar.mouseEnter(event);
  }

  SECTION("Mouse exit event") {
    MouseEvent event;
    scroll_bar.mouseExit(event);
  }

  SECTION("Mouse down event") {
    MouseEvent event;
    event.relative_position = { 10, 50 };
    scroll_bar.mouseDown(event);
  }

  SECTION("Mouse up event") {
    MouseEvent event;
    scroll_bar.mouseUp(event);
  }

  SECTION("Mouse drag event") {
    MouseEvent event;
    event.relative_position = { 10, 60 };
    scroll_bar.mouseDrag(event);
  }
}

TEST_CASE("ScrollBar resize handling", "[ui]") {
  ScrollBar scroll_bar;

  SECTION("Resize triggers width animation setup") {
    scroll_bar.setBounds(0, 0, 30, 100);
    scroll_bar.resized();
  }
}

TEST_CASE("ScrollableFrame construction", "[ui]") {
  ScrollableFrame scrollable_frame("test_scrollable");

  SECTION("Default state") {
    REQUIRE(scrollable_frame.name() == "test_scrollable");
    REQUIRE(scrollable_frame.yPosition() == 0.0f);
    REQUIRE(scrollable_frame.scrollableHeight() == 0.0f);
  }

  SECTION("Children added correctly") {
    REQUIRE(scrollable_frame.children().size() == 2);
  }
}

TEST_CASE("ScrollableFrame scroll operations", "[ui]") {
  ScrollableFrame scrollable_frame;
  scrollable_frame.setBounds(0, 0, 100, 200);

  SECTION("Setting scrollable height") {
    scrollable_frame.setScrollableHeight(400, 200);
    REQUIRE(scrollable_frame.scrollableHeight() == 400);
  }

  SECTION("Setting Y position") {
    scrollable_frame.setScrollableHeight(400, 200);
    scrollable_frame.setYPosition(50);
    REQUIRE(scrollable_frame.yPosition() == 50);
  }

  SECTION("Scroll up") {
    scrollable_frame.setScrollableHeight(400, 200);
    scrollable_frame.setYPosition(100);
    bool result = scrollable_frame.scrollUp();
    REQUIRE(result);
    REQUIRE(scrollable_frame.yPosition() < 100);
  }

  SECTION("Scroll down") {
    scrollable_frame.setScrollableHeight(400, 200);
    scrollable_frame.setYPosition(50);
    bool result = scrollable_frame.scrollDown();
    REQUIRE(result);
    REQUIRE(scrollable_frame.yPosition() > 50);
  }

  SECTION("Scroll up at top boundary") {
    scrollable_frame.setScrollableHeight(400, 200);
    scrollable_frame.setYPosition(0);
    float initial_position = scrollable_frame.yPosition();
    scrollable_frame.scrollUp();
    REQUIRE(scrollable_frame.yPosition() == initial_position);
  }
}

TEST_CASE("ScrollableFrame scrollable children", "[ui]") {
  ScrollableFrame scrollable_frame;
  Frame child_frame("child");

  SECTION("Adding scrolled child") {
    scrollable_frame.addScrolledChild(&child_frame);
    REQUIRE(child_frame.parent() != &scrollable_frame);
    REQUIRE(child_frame.isVisible());
  }

  SECTION("Adding scrolled child with visibility control") {
    scrollable_frame.addScrolledChild(&child_frame, false);
    REQUIRE_FALSE(child_frame.isVisible());
  }
}

TEST_CASE("ScrollableFrame scroll bar configuration", "[ui]") {
  ScrollableFrame scrollable_frame;

  SECTION("Setting scroll bar rounding") {
    scrollable_frame.setScrollBarRounding(8.0f);
  }

  SECTION("Setting scroll bar bounds") {
    scrollable_frame.setScrollBarBounds(90, 0, 10, 200);
  }

  SECTION("Setting scroll bar side") {
    scrollable_frame.setScrollBarLeft(true);
    scrollable_frame.setScrollBarLeft(false);
  }
}

TEST_CASE("ScrollableFrame scroll sensitivity and smoothing", "[ui]") {
  ScrollableFrame scrollable_frame;

  SECTION("Setting sensitivity") {
    scrollable_frame.setSensitivity(150.0f);
  }

  SECTION("Setting smooth time") {
    scrollable_frame.setSmoothTime(0.2f);
  }

  SECTION("Default values") {
    ScrollableFrame default_frame;
    default_frame.setSensitivity(ScrollableFrame::kDefaultWheelSensitivity);
    default_frame.setSmoothTime(ScrollableFrame::kDefaultSmoothTime);
  }
}

TEST_CASE("ScrollableFrame mouse wheel handling", "[ui]") {
  ScrollableFrame scrollable_frame;
  scrollable_frame.setBounds(0, 0, 100, 200);
  scrollable_frame.setScrollableHeight(400, 200);

  SECTION("Mouse wheel without momentum") {
    MouseEvent wheel_event;
    wheel_event.precise_wheel_delta_y = 10.0f;
    wheel_event.wheel_momentum = false;

    bool result = scrollable_frame.mouseWheel(wheel_event);
    REQUIRE(result);
  }

  SECTION("Mouse wheel with momentum") {
    MouseEvent wheel_event;
    wheel_event.precise_wheel_delta_y = 5.0f;
    wheel_event.wheel_momentum = true;

    bool result = scrollable_frame.mouseWheel(wheel_event);
    REQUIRE_FALSE(result);
  }

  SECTION("Mouse wheel with no scroll range") {
    ScrollableFrame no_scroll_frame;
    no_scroll_frame.setBounds(0, 0, 100, 200);
    no_scroll_frame.setScrollableHeight(150, 200);

    MouseEvent wheel_event;
    wheel_event.precise_wheel_delta_y = 10.0f;
    wheel_event.wheel_momentum = false;

    bool result = no_scroll_frame.mouseWheel(wheel_event);
    REQUIRE_FALSE(result);
  }

  SECTION("Mouse wheel at scroll boundaries with momentum") {
    scrollable_frame.setYPosition(200);

    MouseEvent wheel_event;
    wheel_event.precise_wheel_delta_y = -10.0f;
    wheel_event.wheel_momentum = true;

    bool result = scrollable_frame.mouseWheel(wheel_event);
    REQUIRE_FALSE(result);
  }
}

TEST_CASE("ScrollableFrame callback system", "[ui]") {
  ScrollableFrame scrollable_frame;
  bool scroll_callback_called = false;
  ScrollableFrame* callback_frame = nullptr;

  scrollable_frame.onScroll().add([&](ScrollableFrame* frame) {
    scroll_callback_called = true;
    callback_frame = frame;
  });

  SECTION("Scroll callback triggered by position change") {
    scrollable_frame.setBounds(0, 0, 100, 200);
    scrollable_frame.setScrollableHeight(400, 200);
    scrollable_frame.setYPosition(50);

    REQUIRE(scroll_callback_called);
    REQUIRE(callback_frame == &scrollable_frame);
  }
}

TEST_CASE("ScrollableFrame layout access", "[ui]") {
  ScrollableFrame scrollable_frame;

  SECTION("Scrollable layout access") {
    Layout& layout = scrollable_frame.scrollableLayout();
    layout.setFlex(true);
    REQUIRE(layout.flex());
  }
}

TEST_CASE("ScrollableFrame scroll bar access", "[ui]") {
  ScrollableFrame scrollable_frame;

  SECTION("Scroll bar access") {
    ScrollBar& scroll_bar = scrollable_frame.scrollBar();
    scroll_bar.setRounding(10.0f);
  }
}