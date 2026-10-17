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

#include <catch2/catch_test_macros.hpp>
#include <functional>

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
