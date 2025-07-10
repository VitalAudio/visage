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
#include "visage_ui/frame.h"
#include "visage_utils/events.h"

#include <catch2/catch_test_macros.hpp>

using namespace visage;

class TestFrame : public Frame {
public:
  TestFrame(std::string name = "") : Frame(std::move(name)) { }

  int draw_count = 0;
  int resize_count = 0;
  int mouse_enter_count = 0;
  int mouse_exit_count = 0;
  int mouse_down_count = 0;
  int mouse_up_count = 0;
  bool last_key_press_result = false;
  bool last_mouse_wheel_result = false;

  void draw(Canvas& canvas) override {
    draw_count++;
    Frame::draw(canvas);
  }

  void resized() override {
    resize_count++;
    Frame::resized();
  }

  void mouseEnter(const MouseEvent& e) override {
    mouse_enter_count++;
    Frame::mouseEnter(e);
  }

  void mouseExit(const MouseEvent& e) override {
    mouse_exit_count++;
    Frame::mouseExit(e);
  }

  void mouseDown(const MouseEvent& e) override {
    mouse_down_count++;
    Frame::mouseDown(e);
  }

  void mouseUp(const MouseEvent& e) override {
    mouse_up_count++;
    Frame::mouseUp(e);
  }

  bool keyPress(const KeyEvent& e) override { return last_key_press_result; }

  bool mouseWheel(const MouseEvent& e) override { return last_mouse_wheel_result; }
};

TEST_CASE("Frame construction and naming", "[ui]") {
  Frame frame;
  REQUIRE(frame.name().empty());
  REQUIRE(frame.parent() == nullptr);
  REQUIRE(frame.children().empty());

  Frame named_frame("test_frame");
  REQUIRE(named_frame.name() == "test_frame");

  frame.setName("updated_name");
  REQUIRE(frame.name() == "updated_name");
}

TEST_CASE("Frame bounds and positioning", "[ui]") {
  Frame frame;

  SECTION("Default bounds") {
    REQUIRE(frame.x() == 0.0f);
    REQUIRE(frame.y() == 0.0f);
    REQUIRE(frame.width() == 0.0f);
    REQUIRE(frame.height() == 0.0f);
  }

  SECTION("Setting bounds") {
    frame.setBounds(10.0f, 20.0f, 100.0f, 200.0f);
    REQUIRE(frame.x() == 10.0f);
    REQUIRE(frame.y() == 20.0f);
    REQUIRE(frame.width() == 100.0f);
    REQUIRE(frame.height() == 200.0f);
    REQUIRE(frame.right() == 110.0f);
    REQUIRE(frame.bottom() == 220.0f);
  }

  SECTION("Setting bounds with Bounds object") {
    Bounds bounds { 5.0f, 15.0f, 50.0f, 75.0f };
    frame.setBounds(bounds);
    REQUIRE(frame.bounds() == bounds);
  }

  SECTION("Setting top-left position") {
    frame.setBounds(0.0f, 0.0f, 100.0f, 200.0f);
    frame.setTopLeft(25.0f, 35.0f);
    REQUIRE(frame.x() == 25.0f);
    REQUIRE(frame.y() == 35.0f);
    REQUIRE(frame.width() == 100.0f);
    REQUIRE(frame.height() == 200.0f);
  }

  SECTION("Native bounds") {
    frame.setNativeBounds(5, 10, 150, 250);
    REQUIRE(frame.nativeX() == 5);
    REQUIRE(frame.nativeY() == 10);
    REQUIRE(frame.nativeWidth() == 150);
    REQUIRE(frame.nativeHeight() == 250);
    REQUIRE(frame.nativeRight() == 155);
    REQUIRE(frame.nativeBottom() == 260);
  }

  SECTION("Local bounds") {
    frame.setBounds(10.0f, 20.0f, 100.0f, 200.0f);
    auto local = frame.localBounds();
    REQUIRE(local.x() == 0.0f);
    REQUIRE(local.y() == 0.0f);
    REQUIRE(local.width() == 100.0f);
    REQUIRE(local.height() == 200.0f);
  }

  SECTION("Aspect ratio") {
    frame.setBounds(0.0f, 0.0f, 100.0f, 50.0f);
    REQUIRE(frame.aspectRatio() == 2.0f);

    frame.setBounds(0.0f, 0.0f, 50.0f, 100.0f);
    REQUIRE(frame.aspectRatio() == 0.5f);
  }

  SECTION("Point containment") {
    frame.setBounds(10.0f, 20.0f, 100.0f, 200.0f);
    REQUIRE(frame.containsPoint({ 50.0f, 100.0f }));
    REQUIRE(frame.containsPoint({ 10.0f, 20.0f }));
    REQUIRE_FALSE(frame.containsPoint({ 5.0f, 15.0f }));
    REQUIRE_FALSE(frame.containsPoint({ 115.0f, 225.0f }));
  }
}

TEST_CASE("Frame hierarchy management", "[ui]") {
  Frame parent;
  auto child1 = std::make_unique<Frame>("child1");
  auto child2 = std::make_unique<Frame>("child2");
  Frame* child1_ptr = child1.get();
  Frame* child2_ptr = child2.get();

  SECTION("Adding children") {
    parent.addChild(std::move(child1));
    REQUIRE(parent.children().size() == 1);
    REQUIRE(parent.children()[0] == child1_ptr);
    REQUIRE(child1_ptr->parent() == &parent);

    parent.addChild(std::move(child2));
    REQUIRE(parent.children().size() == 2);
    REQUIRE(parent.children()[1] == child2_ptr);
    REQUIRE(child2_ptr->parent() == &parent);
  }

  SECTION("Finding child index") {
    parent.addChild(std::move(child1));
    parent.addChild(std::move(child2));

    REQUIRE(parent.indexOfChild(child1_ptr) == 0);
    REQUIRE(parent.indexOfChild(child2_ptr) == 1);

    Frame unrelated_frame;
    REQUIRE(parent.indexOfChild(&unrelated_frame) == -1);
  }

  SECTION("Removing children") {
    parent.addChild(std::move(child1));
    parent.addChild(std::move(child2));

    parent.removeChild(child1_ptr);
    REQUIRE(parent.children().size() == 1);
    REQUIRE(parent.children()[0] == child2_ptr);
    REQUIRE(parent.indexOfChild(child1_ptr) == -1);
  }

  SECTION("Removing all children") {
    parent.addChild(std::move(child1));
    parent.addChild(std::move(child2));

    parent.removeAllChildren();
    REQUIRE(parent.children().empty());
  }

  SECTION("Parent-child relationships") {
    Frame grandparent;
    grandparent.addChild(&parent);
    parent.addChild(std::move(child1));

    REQUIRE(child1_ptr->parent() == &parent);
    REQUIRE(parent.parent() == &grandparent);
  }
}

TEST_CASE("Frame visibility and drawing", "[ui]") {
  TestFrame frame;

  SECTION("Default visibility") {
    REQUIRE(frame.isVisible());
    REQUIRE(frame.isDrawing());
  }

  SECTION("Setting visibility") {
    frame.setVisible(false);
    REQUIRE_FALSE(frame.isVisible());

    frame.setVisible(true);
    REQUIRE(frame.isVisible());
  }

  SECTION("Setting drawing state") {
    frame.setDrawing(false);
    REQUIRE_FALSE(frame.isDrawing());

    frame.setDrawing(true);
    REQUIRE(frame.isDrawing());
  }
}

TEST_CASE("Frame event handling", "[ui]") {
  TestFrame frame;

  SECTION("Mouse events") {
    MouseEvent mouse_event {};

    frame.processMouseEnter(mouse_event);
    REQUIRE(frame.mouse_enter_count == 1);

    frame.processMouseExit(mouse_event);
    REQUIRE(frame.mouse_exit_count == 1);

    frame.processMouseDown(mouse_event);
    REQUIRE(frame.mouse_down_count == 1);

    frame.processMouseUp(mouse_event);
    REQUIRE(frame.mouse_up_count == 1);
  }

  SECTION("Key events") {
    KeyEvent key_event(KeyCode::A, 0, true);

    frame.last_key_press_result = true;
    REQUIRE(frame.processKeyPress(key_event));

    frame.last_key_press_result = false;
    REQUIRE_FALSE(frame.processKeyPress(key_event));
  }

  SECTION("Mouse wheel events") {
    MouseEvent wheel_event {};

    frame.last_mouse_wheel_result = true;
    REQUIRE(frame.processMouseWheel(wheel_event));

    frame.last_mouse_wheel_result = false;
    REQUIRE_FALSE(frame.processMouseWheel(wheel_event));
  }
}

TEST_CASE("Frame focus and keyboard handling", "[ui]") {
  Frame frame;

  SECTION("Default focus state") {
    REQUIRE_FALSE(frame.hasKeyboardFocus());
    REQUIRE_FALSE(frame.acceptsKeystrokes());
  }

  SECTION("Setting accepts keystrokes") {
    frame.setAcceptsKeystrokes(true);
    REQUIRE(frame.acceptsKeystrokes());

    frame.setAcceptsKeystrokes(false);
    REQUIRE_FALSE(frame.acceptsKeystrokes());
  }

  SECTION("Focus processing") {
    frame.setAcceptsKeystrokes(true);
    frame.processFocusChanged(true, false);
    REQUIRE(frame.hasKeyboardFocus());

    frame.processFocusChanged(false, false);
    REQUIRE_FALSE(frame.hasKeyboardFocus());
  }

  SECTION("Focus with non-accepting frame") {
    frame.setAcceptsKeystrokes(false);
    frame.processFocusChanged(true, false);
    REQUIRE_FALSE(frame.hasKeyboardFocus());
  }
}

TEST_CASE("Frame mouse event handling properties", "[ui]") {
  Frame frame;

  SECTION("Default mouse event handling") {
    REQUIRE_FALSE(frame.ignoresMouseEvents());
  }

  SECTION("Setting mouse event ignore") {
    frame.setIgnoresMouseEvents(true, false);
    REQUIRE(frame.ignoresMouseEvents());

    frame.setIgnoresMouseEvents(false, true);
    REQUIRE_FALSE(frame.ignoresMouseEvents());
  }
}

TEST_CASE("Frame DPI handling", "[ui]") {
  TestFrame parent;
  auto child = std::make_unique<TestFrame>("child");
  TestFrame* child_ptr = child.get();
  parent.addChild(std::move(child));

  SECTION("Default DPI scale") {
    REQUIRE(parent.dpiScale() == 1.0f);
    REQUIRE(child_ptr->dpiScale() == 1.0f);
  }

  SECTION("Setting DPI scale propagates to children") {
    parent.setDpiScale(2.0f);
    REQUIRE(parent.dpiScale() == 2.0f);
    REQUIRE(child_ptr->dpiScale() == 2.0f);
  }
}

TEST_CASE("Frame transparency and effects", "[ui]") {
  Frame frame;

  SECTION("Default alpha transparency") {
    frame.setAlphaTransparency(0.5f);
    frame.removeAlphaTransparency();
  }

  SECTION("Setting cached state") {
    frame.setCached(true);
    frame.setCached(false);
  }

  SECTION("Setting masked state") {
    frame.setMasked(true);
    frame.setMasked(false);
  }
}

TEST_CASE("Frame on-top handling", "[ui]") {
  Frame frame;

  SECTION("Default on-top state") {
    REQUIRE_FALSE(frame.isOnTop());
  }

  SECTION("Setting on-top state") {
    frame.setOnTop(true);
    REQUIRE(frame.isOnTop());

    frame.setOnTop(false);
    REQUIRE_FALSE(frame.isOnTop());
  }
}

TEST_CASE("Frame parent finding", "[ui]") {
  class TestParentA : public Frame { };
  class TestParentB : public Frame { };

  TestParentA grandparent;
  TestParentB parent;
  Frame child;

  grandparent.addChild(&parent);
  parent.addChild(&child);

  SECTION("Finding specific parent type") {
    auto found_a = child.findParent<TestParentA>();
    REQUIRE(found_a == &grandparent);

    auto found_b = child.findParent<TestParentB>();
    REQUIRE(found_b == &parent);
  }

  SECTION("Parent type not found") {
    class TestParentC : public Frame { };
    auto found_c = child.findParent<TestParentC>();
    REQUIRE(found_c == nullptr);
  }
}

class MockEventHandler : public FrameEventHandler {
public:
  int redraw_count = 0;
  int focus_count = 0;
  int remove_count = 0;
  std::string last_clipboard_text;

  MockEventHandler() {
    request_redraw = [this](Frame*) { redraw_count++; };
    request_keyboard_focus = [this](Frame*) { focus_count++; };
    remove_from_hierarchy = [this](Frame*) { remove_count++; };
    read_clipboard_text = []() -> std::string { return "test_clipboard"; };
    set_clipboard_text = [this](std::string text) { last_clipboard_text = std::move(text); };
  }
};

TEST_CASE("Frame event handler integration", "[ui]") {
  Frame frame;
  MockEventHandler handler;
  frame.setEventHandler(&handler);

  SECTION("Redraw requests") {
    frame.setVisible(true);
    frame.setDrawing(true);
    bool result = frame.requestRedraw();
    REQUIRE(result);
    REQUIRE(handler.redraw_count >= 1);
  }

  SECTION("Keyboard focus requests") {
    frame.requestKeyboardFocus();
    REQUIRE(handler.focus_count == 1);
  }

  SECTION("Hierarchy removal notification") {
    frame.notifyRemoveFromHierarchy();
    REQUIRE(handler.remove_count == 1);
  }

  SECTION("Clipboard operations") {
    std::string text = frame.readClipboardText();
    REQUIRE(text == "test_clipboard");

    frame.setClipboardText("new_text");
    REQUIRE(handler.last_clipboard_text == "new_text");
  }

  SECTION("Event handler propagation to children") {
    auto child = std::make_unique<Frame>();
    Frame* child_ptr = child.get();
    frame.addChild(std::move(child));

    REQUIRE(child_ptr->eventHandler() == &handler);
  }

  frame.setEventHandler(nullptr);
}

TEST_CASE("Frame layout management", "[ui]") {
  Frame frame;

  SECTION("Layout creation on demand") {
    Layout& layout = frame.layout();
    REQUIRE(&layout == &frame.layout());
  }

  SECTION("Clearing layout") {
    frame.layout();
    frame.clearLayout();
  }

  SECTION("Setting flex layout") {
    frame.setFlexLayout(true);
    REQUIRE(frame.layout().flex());

    frame.setFlexLayout(false);
    REQUIRE_FALSE(frame.layout().flex());
  }
}