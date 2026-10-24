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
#include "visage/widgets.h"
#include "visage_ui/popup_menu.h"

#include <catch2/catch_test_macros.hpp>
#include <functional>
#include <thread>

using namespace visage;

namespace {
  class TestWindow : public Window {
  public:
    TestWindow(int width, int height) : Window(width, height) { }

    void runEventLoop() override { }
    void* nativeHandle() const override { return nullptr; }
    void windowContentsResized(int width, int height) override { }
    void show() override { }
    void showMaximized() override { }
    void hide() override { }
    void close() override { }
    bool isShowing() const override { return true; }
    void setWindowTitle(const std::string& title) override { }
    IPoint maxWindowDimensions() const override { return { 1000, 1000 }; }
  };
}

TEST_CASE("Mouse up over another frame sends it an enter relative to itself", "[integration]") {
  ApplicationEditor app;
  Frame top;
  Frame bottom;
  Point bottom_enter_position;
  bottom.onMouseEnter() = [&](const MouseEvent& e) { bottom_enter_position = e.position; };

  app.addChild(top);
  app.addChild(bottom);
  app.setBounds(0, 0, 40, 40);
  top.setBounds(0, 0, 40, 20);
  bottom.setBounds(0, 20, 40, 20);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseMove(10, 10, 0, 0);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 15, 25, 0, 0);
  REQUIRE(bottom_enter_position == Point(15, 5));
  app.removeFromWindow();
}

TEST_CASE("Windows without an event file descriptor report -1", "[integration]") {
  TestWindow window(40, 40);
  REQUIRE(window.posixFd() == -1);
}

TEST_CASE("Repeat clicks count only the same button near the last click", "[integration]") {
  ApplicationEditor app;
  Frame frame;
  std::vector<int> counts;
  frame.onMouseDown() = [&](const MouseEvent& e) { counts.push_back(e.repeatClickCount()); };

  app.addChild(frame);
  app.setBounds(0, 0, 100, 100);
  frame.setBounds(0, 0, 100, 100);

  TestWindow window(100, 100);
  app.addToWindow(&window);
  auto click = [&](MouseButton button, int x, int y, bool move) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (move)
      window.handleMouseMove(x, y, 0, 0);
    window.handleMouseDown(button, x, y, button, 0);
    window.handleMouseUp(button, x, y, 0, 0);
  };

  click(kMouseButtonLeft, 10, 10, true);
  click(kMouseButtonLeft, 11, 10, true);
  click(kMouseButtonLeft, 12, 11, true);
  click(kMouseButtonRight, 12, 11, true);
  click(kMouseButtonLeft, 40, 40, false);
  click(kMouseButtonLeft, 40, 40, false);
  REQUIRE(counts == std::vector<int> { 1, 2, 3, 1, 1, 2 });

  counts.clear();
  click(kMouseButtonLeft, 60, 60, true);
  click(kMouseButtonLeft, 62, 60, true);
  click(kMouseButtonLeft, 64, 60, true);
  click(kMouseButtonLeft, 64, 63, true);
  REQUIRE(counts == std::vector<int> { 1, 2, 1, 1 });
  app.removeFromWindow();
}

TEST_CASE("Editors can be deleted while attached or inside their draw callback", "[integration]") {
  class ActionTimer : public EventTimer {
  public:
    void timerCallback() override {
      stopTimer();
      action();
    }

    std::function<void()> action;
  };

  TestWindow window(40, 40);
  auto app = std::make_unique<ApplicationEditor>();
  app->setBounds(0, 0, 40, 40);
  app->addToWindow(&window);

  SECTION("By a timer during the draw callback") {
    ActionTimer timer;
    timer.action = [&app] { app.reset(); };
    timer.startTimer(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    window.drawCallback(0.0);
    REQUIRE(app == nullptr);
  }

  SECTION("While attached") {
    app.reset();
  }

  window.drawCallback(0.0);
  window.handleMouseMove(10, 10, 0, 0);
}

TEST_CASE("Editors take a window DPI change that keeps the native size", "[integration]") {
  ApplicationEditor app;
  TestWindow window(600, 400);
  window.setDpiScale(2.0f);
  app.addToWindow(&window);
  REQUIRE(app.width() == 300);

  window.setDpiScale(1.0f);
  window.handleResized(600, 400);
  CHECK(app.dpiScale() == 1.0f);
  CHECK(app.width() == 600);
  CHECK(app.height() == 400);
  app.removeFromWindow();
}

TEST_CASE("Minimum window size uses the window's current DPI", "[integration]") {
  ApplicationEditor app;
  app.setMinimumDimensions(300, 200);
  TestWindow window(600, 400);
  window.setDpiScale(2.0f);
  app.addToWindow(&window);

  window.setDpiScale(1.0f);
  int width = 300;
  int height = 200;
  window.handleAdjustResize(&width, &height, true, true);
  CHECK(width == 300);
  CHECK(height == 200);
  app.removeFromWindow();
}

TEST_CASE("Dragging files back over their source frame exits the previous target",
          "[integration]") {
  class DropFrame : public Frame {
  public:
    bool receivesDragDropFiles() override { return true; }
    void dragFilesEnter(const std::vector<std::string>&) override { enters++; }
    void dragFilesExit() override { exits++; }
    void dropFiles(const std::vector<std::string>&) override { drops++; }

    int enters = 0;
    int exits = 0;
    int drops = 0;
  };

  ApplicationEditor app;
  DropFrame source;
  DropFrame target;
  app.addChild(source);
  app.addChild(target);
  app.setBounds(0, 0, 40, 40);
  source.setBounds(0, 0, 40, 20);
  target.setBounds(0, 20, 40, 20);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  std::vector<std::string> files = { "/tmp/file.wav" };
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);

  REQUIRE(window.handleFileDrag(10, 30, files));
  REQUIRE(target.enters == 1);

  window.handleFileDrag(10, 10, files);
  REQUIRE(target.exits == 1);

  REQUIRE_FALSE(window.handleFileDrop(10, 10, files));
  REQUIRE(target.exits == 1);
  REQUIRE(source.drops == 0);
  REQUIRE(target.drops == 0);

  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  REQUIRE(window.handleFileDrag(10, 30, files));
  REQUIRE(target.enters == 2);
  REQUIRE(window.handleFileDrop(10, 30, files));
  REQUIRE(target.drops == 1);
  app.removeFromWindow();
}

TEST_CASE("Dragging files over a frame with an invalid extension regex doesn't throw",
          "[integration]") {
  class BadRegexFrame : public Frame {
  public:
    bool receivesDragDropFiles() override { return true; }
    std::string dragDropFileExtensionRegex() override { return "(wav"; }
  };

  ApplicationEditor app;
  BadRegexFrame frame;
  app.addChild(frame);
  app.setBounds(0, 0, 40, 40);
  frame.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  std::vector<std::string> files = { "/tmp/file.wav" };
  REQUIRE_NOTHROW(window.handleFileDrag(10, 10, files));
  REQUIRE_FALSE(window.handleFileDrag(10, 10, files));
  REQUIRE_FALSE(window.handleFileDrop(10, 10, files));
  app.removeFromWindow();
}

TEST_CASE("Losing focus while a button is held doesn't click it", "[integration]") {
  ApplicationEditor app;
  UiButton button("Button");
  int toggles = 0;
  int ups = 0;
  button.onToggle() += [&toggles](Button*, bool) { toggles++; };
  button.onMouseUp() += [&ups](const MouseEvent&) { ups++; };
  app.addChild(button);
  app.setBounds(0, 0, 40, 40);
  button.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseMove(10, 10, 0, 0);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleFocusLost();
  REQUIRE(ups == 1);
  REQUIRE(toggles == 0);

  window.handleFocusGained();
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  REQUIRE(toggles == 1);
  app.removeFromWindow();
}

TEST_CASE("Window doesn't draw an editor after it's removed from the window", "[integration]") {
  ApplicationEditor app;
  Frame child;
  int draws = 0;
  child.onDraw() = [&draws](Canvas&) { draws++; };
  app.addChild(child);
  app.setBounds(0, 0, 40, 40);
  child.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  int draws_while_attached = draws;
  app.removeFromWindow();

  child.redraw();
  window.drawCallback(1.0);
  REQUIRE(draws == draws_while_attached);
}

TEST_CASE("Removing an editor from its window during the window's draw doesn't crash",
          "[integration]") {
  ApplicationEditor app;
  Frame child;
  int draws = 0;
  child.onDraw() = [&draws](Canvas&) { draws++; };
  app.addChild(child);
  app.setBounds(0, 0, 40, 40);
  child.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  int draws_while_attached = draws;

  runOnEventThread([&app] { app.removeFromWindow(); });
  child.redraw();
  window.drawCallback(1.0);
  REQUIRE(draws == draws_while_attached);
}

namespace {
  class FocusFrame : public Frame {
  public:
    FocusFrame() { setAcceptsKeystrokes(true); }

    void focusChanged(bool is_focused, bool) override {
      focus_changes.push_back(is_focused);
      if (!is_focused && on_blur)
        on_blur();
    }
    bool keyPress(const KeyEvent&) override {
      key_presses++;
      return true;
    }

    std::vector<bool> focus_changes;
    std::function<void()> on_blur;
    int key_presses = 0;
  };
}

TEST_CASE("Hiding or removing the focused frame takes its keyboard focus away", "[integration]") {
  ApplicationEditor app;
  Frame panel;
  FocusFrame field;
  app.addChild(panel);
  panel.addChild(field);
  app.setBounds(0, 0, 40, 40);
  panel.setBounds(0, 0, 40, 40);
  field.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);

  SECTION("Hiding a parent") {
    window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
    window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
    REQUIRE(field.focus_changes == std::vector<bool> { true });

    panel.setVisible(false);
    REQUIRE(field.focus_changes == std::vector<bool> { true, false });
    window.handleKeyDown(KeyCode::A, 0, false);
    REQUIRE(field.key_presses == 0);
  }

  SECTION("Removing from the parent") {
    window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
    window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
    panel.removeChild(field);
    REQUIRE(field.focus_changes == std::vector<bool> { true, false });
    window.handleKeyDown(KeyCode::A, 0, false);
    REQUIRE(field.key_presses == 0);
  }

  app.removeFromWindow();
}

TEST_CASE("A blur handler can remove the frame losing focus", "[integration]") {
  ApplicationEditor app;
  Frame panel;
  Frame sibling;
  auto owned_field = std::make_unique<FocusFrame>();
  FocusFrame* field = owned_field.get();
  app.addChild(panel);
  panel.addChild(std::move(owned_field));
  panel.addChild(sibling);
  app.setBounds(0, 0, 40, 40);
  panel.setBounds(0, 0, 40, 40);
  field->setBounds(0, 0, 40, 20);
  sibling.setBounds(0, 20, 40, 20);
  field->on_blur = [&panel, field] { panel.removeChild(field); };

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);

  SECTION("Removing it") {
    panel.removeChild(field);
    REQUIRE(panel.children() == std::vector<Frame*> { &sibling });
  }

  SECTION("Hiding it") {
    field->setVisible(false);
    REQUIRE(panel.children() == std::vector<Frame*> { &sibling });
  }

  app.removeFromWindow();
}

TEST_CASE("Hiding a frame doesn't take focus from another frame", "[integration]") {
  ApplicationEditor app;
  Frame panel;
  FocusFrame first;
  FocusFrame second;
  app.addChild(panel);
  panel.addChild(first);
  app.addChild(second);
  app.setBounds(0, 0, 40, 40);
  panel.setBounds(0, 0, 40, 20);
  first.setBounds(0, 0, 40, 20);
  second.setBounds(0, 20, 40, 20);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  panel.removeChild(first);
  panel.addChild(first);
  window.handleMouseDown(kMouseButtonLeft, 10, 30, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 30, 0, 0);
  REQUIRE(second.focus_changes == std::vector<bool> { true });

  first.setVisible(false);
  REQUIRE(second.focus_changes == std::vector<bool> { true });
  window.handleKeyDown(KeyCode::A, 0, false);
  REQUIRE(second.key_presses == 1);
  app.removeFromWindow();
}

TEST_CASE("A blur handler can delete the parent of the frame losing focus", "[integration]") {
  ApplicationEditor app;
  auto panel = std::make_unique<Frame>();
  FocusFrame field;
  app.addChild(panel.get());
  panel->addChild(field);
  app.setBounds(0, 0, 40, 40);
  panel->setBounds(0, 0, 40, 40);
  field.setBounds(0, 0, 40, 40);
  field.on_blur = [&panel] { panel.reset(); };

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);

  panel->removeChild(&field);
  REQUIRE(panel == nullptr);
  REQUIRE(field.focus_changes == std::vector<bool> { true, false });
  app.removeFromWindow();
}

TEST_CASE("Removing or hiding the focused frame in an unfocused window blurs it once",
          "[integration]") {
  ApplicationEditor app;
  Frame panel;
  FocusFrame field;
  app.addChild(panel);
  panel.addChild(field);
  app.setBounds(0, 0, 40, 40);
  panel.setBounds(0, 0, 40, 40);
  field.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  window.handleFocusLost();
  REQUIRE(field.focus_changes == std::vector<bool> { true, false });

  SECTION("Removing") {
    panel.removeChild(field);
  }

  SECTION("Hiding") {
    field.setVisible(false);
  }

  REQUIRE(field.focus_changes == std::vector<bool> { true, false });
  window.handleFocusGained();
  REQUIRE(field.focus_changes == std::vector<bool> { true, false });
  app.removeFromWindow();
}

TEST_CASE("Keys go back to the frame focused before a removed or hidden one", "[integration]") {
  ApplicationEditor app;
  FocusFrame field;
  FocusFrame popup;
  app.addChild(field);
  app.addChild(popup);
  app.setBounds(0, 0, 40, 40);
  field.setBounds(0, 0, 40, 20);
  popup.setBounds(0, 20, 40, 20);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  popup.requestKeyboardFocus();
  REQUIRE(field.focus_changes == std::vector<bool> { true, false });

  SECTION("Removing it") {
    app.removeChild(popup);
  }

  SECTION("Hiding it") {
    popup.setVisible(false);
  }

  REQUIRE(popup.focus_changes == std::vector<bool> { true, false });
  window.handleKeyDown(KeyCode::A, 0, false);
  REQUIRE(field.key_presses == 1);
  REQUIRE(field.focus_changes == std::vector<bool> { true, false, true });
  REQUIRE(popup.key_presses == 0);
  app.removeFromWindow();
}

TEST_CASE("Keys don't go back to a focused frame that was removed, hidden or clicked away from",
          "[integration]") {
  ApplicationEditor app;
  auto field = std::make_unique<FocusFrame>();
  FocusFrame popup;
  Frame background;
  app.addChild(field.get());
  app.addChild(popup);
  app.addChild(background);
  app.setBounds(0, 0, 60, 40);
  field->setBounds(0, 0, 40, 20);
  popup.setBounds(0, 20, 40, 20);
  background.setBounds(40, 0, 20, 40);

  TestWindow window(60, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);

  SECTION("Deleted while another frame has focus") {
    popup.requestKeyboardFocus();
    field.reset();
    app.removeChild(popup);
  }

  SECTION("Hidden while another frame has focus") {
    popup.requestKeyboardFocus();
    field->setVisible(false);
    app.removeChild(popup);
    REQUIRE(field->key_presses == 0);
  }

  SECTION("Clicked away from") {
    window.handleMouseDown(kMouseButtonLeft, 50, 10, kMouseButtonLeft, 0);
    window.handleMouseUp(kMouseButtonLeft, 50, 10, 0, 0);
    window.handleKeyDown(KeyCode::A, 0, false);
    REQUIRE(field->key_presses == 0);
    REQUIRE(field->focus_changes == std::vector<bool> { true, false });
  }

  window.handleKeyDown(KeyCode::A, 0, false);
  REQUIRE(popup.key_presses == 0);
  if (field)
    REQUIRE(field->key_presses == 0);
  app.removeFromWindow();
}

TEST_CASE("Mouse buttons pressed while another is held go to the first pressed frame",
          "[integration]") {
  ApplicationEditor app;
  Frame top;
  Frame bottom;
  int top_downs = 0, top_ups = 0, bottom_downs = 0, bottom_ups = 0;
  top.onMouseDown() = [&](const MouseEvent&) { top_downs++; };
  top.onMouseUp() = [&](const MouseEvent&) { top_ups++; };
  bottom.onMouseDown() = [&](const MouseEvent&) { bottom_downs++; };
  bottom.onMouseUp() = [&](const MouseEvent&) { bottom_ups++; };

  app.addChild(top);
  app.addChild(bottom);
  app.setBounds(0, 0, 40, 40);
  top.setBounds(0, 0, 40, 20);
  bottom.setBounds(0, 20, 40, 20);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseDown(kMouseButtonRight, 10, 30, kMouseButtonLeft | kMouseButtonRight, 0);
  REQUIRE(top_downs == 2);
  REQUIRE(bottom_downs == 0);

  window.handleMouseUp(kMouseButtonLeft, 10, 30, kMouseButtonRight, 0);
  REQUIRE(top_ups == 1);
  window.handleMouseUp(kMouseButtonRight, 10, 30, 0, 0);
  REQUIRE(top_ups == 2);
  REQUIRE(bottom_ups == 0);

  window.handleMouseDown(kMouseButtonLeft, 10, 30, kMouseButtonLeft, 0);
  REQUIRE(bottom_downs == 1);
  window.handleMouseUp(kMouseButtonLeft, 10, 30, 0, 0);
  REQUIRE(bottom_ups == 1);
  app.removeFromWindow();
}

TEST_CASE("Pressing a second mouse button on a toggle button toggles it once", "[integration]") {
  ApplicationEditor app;
  ToggleButton button;
  int toggles = 0;
  button.onToggle() += [&toggles](Button*, bool) { toggles++; };
  app.addChild(button);
  app.setBounds(0, 0, 40, 40);
  button.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseDown(kMouseButtonRight, 10, 10, kMouseButtonLeft | kMouseButtonRight, 0);
  window.handleMouseUp(kMouseButtonRight, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  REQUIRE(toggles == 1);
  REQUIRE(button.toggled());

  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  REQUIRE(toggles == 2);
  REQUIRE_FALSE(button.toggled());
  app.removeFromWindow();
}

TEST_CASE("Pressing a second mouse button on a popup option selects it once", "[integration]") {
  ApplicationEditor app;
  Frame source;
  app.addChild(source);
  app.setBounds(0, 0, 200, 200);
  source.setBounds(0, 0, 200, 20);

  TestWindow window(200, 200);
  app.addToWindow(&window);

  int selections = 0;
  PopupMenu menu;
  menu.addOption(1, "First");
  menu.addOption(2, "Second");
  menu.onSelection() += [&selections](int) { selections++; };
  menu.show(&source);

  window.handleMouseDown(kMouseButtonLeft, 10, 30, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 30, 0, 0);
  REQUIRE(selections == 0);

  window.handleMouseDown(kMouseButtonLeft, 10, 30, kMouseButtonLeft, 0);
  window.handleMouseDown(kMouseButtonRight, 10, 30, kMouseButtonLeft | kMouseButtonRight, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 30, kMouseButtonRight, 0);
  window.handleMouseUp(kMouseButtonRight, 10, 30, 0, 0);
  REQUIRE(selections == 1);
  app.removeFromWindow();
}

TEST_CASE("A toggle on press button removed during a press still takes the next press",
          "[integration]") {
  ApplicationEditor app;
  ToggleButton button;
  button.setToggleOnMouseDown(true);
  int toggles = 0;
  button.onToggle() += [&toggles](Button*, bool) { toggles++; };
  app.addChild(button);
  app.setBounds(0, 0, 40, 40);
  button.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  REQUIRE(toggles == 1);

  app.removeChild(button);
  app.addChild(button);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  window.handleMouseDown(kMouseButtonLeft, 10, 10, kMouseButtonLeft, 0);
  window.handleMouseUp(kMouseButtonLeft, 10, 10, 0, 0);
  REQUIRE(toggles == 2);
  app.removeFromWindow();
}

TEST_CASE("Window resizes don't report the editor's contents as resized but editor resizes do",
          "[integration]") {
  ApplicationEditor app;
  app.setBounds(0, 0, 40, 40);

  TestWindow window(40, 40);
  app.addToWindow(&window);
  int reports = 0;
  app.onWindowContentsResized() = [&reports] { reports++; };

  window.handleResized(60, 50);
  window.setNativeWindowSize(70, 55);
  CHECK(app.nativeWidth() == 70);
  CHECK(app.nativeHeight() == 55);
  CHECK(reports == 0);

  app.setNativeBounds(0, 0, 80, 60);
  CHECK(window.clientWidth() == 80);
  CHECK(window.clientHeight() == 60);
  CHECK(reports == 1);
  app.removeFromWindow();
}

TEST_CASE("Host sizes set before a window exists don't report the contents as resized",
          "[integration]") {
  ApplicationWindow app;
  int reports = 0;
  app.onWindowContentsResized() = [&reports] { reports++; };
  app.setNativeWindowDimensionsFromHost(120, 90);
  CHECK(app.nativeWidth() == 120);
  CHECK(app.nativeHeight() == 90);
  CHECK(reports == 0);
}
