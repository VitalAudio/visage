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
#include "visage_ui/svg_frame.h"
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
  std::string last_text_input;

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

  void textInput(const std::string& text) override { last_text_input = text; }
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

  SECTION("Text input events") {
    std::string callback_text;
    frame.onTextInput() += [&](const std::string& text) { callback_text = text; };

    frame.processTextInput("abc");
    REQUIRE(frame.last_text_input == "abc");
    REQUIRE(callback_text == "abc");
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

TEST_CASE("Frame listen", "[ui]") {
  SECTION("Stops listening when the listener is destroyed") {
    Frame parent;
    int calls = 0;
    auto child = std::make_unique<Frame>();
    child->listen(parent.onResize(), [&calls] { calls++; });

    parent.setBounds(0, 0, 10, 10);
    REQUIRE(calls == 1);
    child.reset();
    parent.setBounds(0, 0, 20, 20);
    REQUIRE(calls == 1);
  }

  SECTION("Source destroyed first") {
    auto parent = std::make_unique<Frame>();
    Frame child;
    child.listen(parent->onResize(), [] {});
    parent.reset();
  }
}

TEST_CASE("Frame removed by its own mouse event", "[ui]") {
  Frame parent;
  auto owned_child = std::make_unique<Frame>();
  Frame* child = owned_child.get();
  parent.addChild(std::move(owned_child));
  parent.setReceiveChildMouseEvents(true);

  int parent_mouse_downs = 0;
  parent.onMouseDown() += [&](const MouseEvent&) { parent_mouse_downs++; };
  child->onMouseDown() += [&](const MouseEvent&) { parent.removeChild(child); };

  MouseEvent mouse_event {};
  child->processMouseDown(mouse_event);
  REQUIRE(parent_mouse_downs == 1);
}

TEST_CASE("Frame removed with its parent by its own mouse event", "[ui]") {
  Frame list;
  auto owned_row = std::make_unique<Frame>();
  Frame* row = owned_row.get();
  list.addChild(std::move(owned_row));
  auto owned_button = std::make_unique<Frame>();
  Frame* button = owned_button.get();
  row->addChild(std::move(owned_button));

  button->onMouseUp() += [&](const MouseEvent&) { list.removeChild(row); };

  MouseEvent mouse_event {};
  button->processMouseUp(mouse_event);
  REQUIRE(list.children().empty());
}

TEST_CASE("Frame remove child that isn't a child", "[ui]") {
  Frame parent;
  Frame other_parent;
  Frame child;
  Frame other_child;
  parent.addChild(child);
  other_parent.addChild(other_child);

  parent.removeChild(other_child);
  REQUIRE(parent.children().size() == 1);
  REQUIRE(other_child.parent() == &other_parent);
}

TEST_CASE("Frame added to a new parent", "[ui]") {
  Frame first_parent;
  Frame second_parent;

  SECTION("Unowned child") {
    Frame child;
    first_parent.addChild(child);
    second_parent.addChild(child);
    REQUIRE(first_parent.children().empty());
    REQUIRE(second_parent.children().size() == 1);
    REQUIRE(child.parent() == &second_parent);
  }

  SECTION("Owned child") {
    auto old_parent = std::make_unique<Frame>();
    auto owned_child = std::make_unique<Frame>();
    Frame* child = owned_child.get();
    old_parent->addChild(std::move(owned_child));
    second_parent.addChild(child);
    old_parent.reset();
    REQUIRE(second_parent.children().size() == 1);
    REQUIRE(child->parent() == &second_parent);
  }

  SECTION("Same parent") {
    Frame child;
    first_parent.addChild(child);
    first_parent.addChild(child);
    REQUIRE(first_parent.children().size() == 1);
  }
}

TEST_CASE("Frame added to a new parent draws only when the new parent draws", "[ui]") {
  Frame first_parent;
  Frame second_parent;
  Frame child;
  Frame grandchild;
  child.addChild(grandchild);

  first_parent.addChild(child);
  first_parent.setVisible(false);
  REQUIRE_FALSE(child.isDrawing());

  second_parent.addChild(child);
  REQUIRE(child.isDrawing());
  REQUIRE(grandchild.isDrawing());

  first_parent.addChild(child);
  REQUIRE_FALSE(child.isDrawing());
  REQUIRE_FALSE(grandchild.isDrawing());

  second_parent.addChild(child, false);
  REQUIRE_FALSE(child.isVisible());
  REQUIRE_FALSE(child.isDrawing());
  REQUIRE_FALSE(grandchild.isDrawing());
}

TEST_CASE("Frame DPI change updates children before notifying", "[ui]") {
  Frame parent;
  Frame child;
  parent.setBounds(0, 0, 400, 300);
  parent.addChild(child);

  float child_dpi_in_callback = 0.0f;
  parent.onDpiChange() += [&] {
    child_dpi_in_callback = child.dpiScale();
    child.setBounds(0, 0, 75, 10);
  };

  parent.setDpiScale(2.0f);
  REQUIRE(child_dpi_in_callback == 2.0f);
  REQUIRE(child.nativeWidth() == 150);
}

TEST_CASE("Frame DPI change updates native bounds of children", "[ui]") {
  Frame parent;
  Frame child;
  parent.setBounds(0, 0, 400, 300);
  parent.addChild(child);
  child.setBounds(10, 20, 100, 50);

  parent.setDpiScale(2.0f);
  REQUIRE(IBounds(child.nativeX(), child.nativeY(), child.nativeWidth(), child.nativeHeight()) ==
          IBounds(20, 40, 200, 100));
}

TEST_CASE("Frame sized before being added uses the parent's DPI", "[ui]") {
  Frame parent;
  parent.setDpiScale(2.0f);
  Frame child;
  child.setBounds(10, 20, 100, 50);

  parent.addChild(child);
  REQUIRE(IBounds(child.nativeX(), child.nativeY(), child.nativeWidth(), child.nativeHeight()) ==
          IBounds(20, 40, 200, 100));
}

TEST_CASE("SvgFrame ignores mouse events however it is constructed", "[ui]") {
  static constexpr char kSvg[] = "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 10 10\">"
                                 "<rect width=\"10\" height=\"10\"/></svg>";
  const auto* data = reinterpret_cast<const uint8_t*>(kSvg);

  SvgFrame from_nothing;
  SvgFrame from_data(data, sizeof(kSvg) - 1);
  SvgFrame from_file(EmbeddedFile { "icon.svg", data, sizeof(kSvg) - 1 });
  REQUIRE(from_nothing.ignoresMouseEvents());
  REQUIRE(from_data.ignoresMouseEvents());
  REQUIRE(from_file.ignoresMouseEvents());
}

TEST_CASE("Frame undo helpers use the frame itself when it is the undo history", "[ui]") {
  class HistoryFrame : public Frame,
                       public UndoHistory { };

  HistoryFrame frame;
  Frame child;
  frame.addChild(child);
  int value = 0;

  frame.addUndoableAction(std::make_unique<LambdaAction>([&value] { value = 1; },
                                                         [&value] { value = 2; }));
  REQUIRE(frame.Frame::canUndo());
  REQUIRE(child.canUndo());

  frame.triggerUndo();
  REQUIRE(value == 1);
  REQUIRE(frame.Frame::canRedo());
  frame.triggerRedo();
  REQUIRE(value == 2);
}

TEST_CASE("Masked frames are destroyed cleanly while attached to a parent", "[ui]") {
  Frame parent;
  auto frame = std::make_unique<Frame>();
  frame->onDrawMask() = [](Canvas&) {};
  parent.addChild(frame.get());
  frame.reset();
  REQUIRE(parent.children().empty());
}
