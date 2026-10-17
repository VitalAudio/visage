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
#include "visage_widgets/text_editor.h"

#include <catch2/catch_test_macros.hpp>
#include <string>

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

  std::string keyName(KeyCode key_code) {
    if (key_code == KeyCode::Return)
      return "Return";
    if (key_code == KeyCode::LShift)
      return "Shift";
    if (key_code == KeyCode::Left)
      return "Left";
    return std::to_string(static_cast<int>(key_code));
  }

  class LoggingFrame : public Frame {
  public:
    LoggingFrame() { setAcceptsKeystrokes(true); }

    bool keyPress(const KeyEvent& e) override {
      log += "P" + keyName(e.keyCode()) + " ";
      return on_key_press ? on_key_press(e) : true;
    }

    bool keyRelease(const KeyEvent& e) override {
      log += "R" + keyName(e.keyCode()) + " ";
      return true;
    }

    std::function<bool(const KeyEvent&)> on_key_press;
    std::string log;
  };

  struct EditorInWindow {
    ApplicationEditor app;
    LoggingFrame parent;
    Frame cell_a;
    Frame cell_b;
    TextEditor editor;
    TestWindow window { 600, 400 };

    EditorInWindow() {
      editor.setText("hello world");
      app.addChild(parent);
      app.addToWindow(&window);
      parent.setBounds(0, 0, 600, 400);
      parent.addChild(cell_a);
      parent.addChild(cell_b);
      cell_a.setBounds(0, 0, 300, 100);
      cell_b.setBounds(300, 0, 300, 100);
      cell_a.addChild(editor);
      editor.setBounds(0, 0, 300, 100);
    }

    ~EditorInWindow() { app.removeFromWindow(); }

    void click(int x, int y, int modifiers) {
      window.handleMouseDown(kMouseButtonLeft, x, y, kMouseButtonLeft, modifiers);
      window.handleMouseUp(kMouseButtonLeft, x, y, 0, modifiers);
    }
  };
}

TEST_CASE("Text editor removed by its Enter handler passes on later Return releases",
          "[integration]") {
  EditorInWindow setup;
  bool focus_before_remove = false;
  SECTION("Focus moves after the removal") {
    focus_before_remove = false;
  }
  SECTION("Focus moves before the removal") {
    focus_before_remove = true;
  }

  setup.cell_a.removeChild(setup.editor);
  setup.parent.on_key_press = [&](const KeyEvent& e) {
    if (e.keyCode() == KeyCode::Return) {
      setup.cell_a.addChild(setup.editor);
      setup.editor.setBounds(0, 0, 300, 100);
      setup.editor.requestKeyboardFocus();
    }
    return true;
  };
  setup.editor.onEnterKey() += [&] {
    if (focus_before_remove)
      setup.parent.requestKeyboardFocus();
    setup.cell_a.removeChild(setup.editor);
    if (!focus_before_remove)
      setup.parent.requestKeyboardFocus();
  };
  setup.parent.requestKeyboardFocus();

  for (int round = 0; round < 3; ++round) {
    setup.window.handleKeyDown(KeyCode::Return, 0, false);
    setup.window.handleKeyUp(KeyCode::Return, 0);
    setup.window.handleTextInput("x");
    setup.window.handleKeyDown(KeyCode::Return, 0, false);
    setup.window.handleKeyUp(KeyCode::Return, 0);
  }
  CHECK(setup.parent.log ==
        "PReturn RReturn RReturn PReturn RReturn RReturn PReturn RReturn RReturn ");
}

TEST_CASE("Text editor removed by its Enter handler passes on later Shift releases",
          "[integration]") {
  EditorInWindow setup;
  setup.editor.onEnterKey() += [&] {
    setup.cell_a.removeChild(setup.editor);
    setup.parent.requestKeyboardFocus();
  };
  setup.editor.requestKeyboardFocus();
  setup.window.handleKeyDown(KeyCode::LShift, kModifierShift, false);
  setup.window.handleKeyDown(KeyCode::Return, kModifierShift, false);
  setup.window.handleKeyUp(KeyCode::Return, kModifierShift);
  setup.window.handleKeyUp(KeyCode::LShift, 0);
  setup.parent.log.clear();

  setup.cell_a.addChild(setup.editor);
  setup.editor.setBounds(0, 0, 300, 100);
  setup.window.handleKeyDown(KeyCode::LShift, kModifierShift, false);
  setup.click(50, 20, kModifierShift);
  setup.window.handleKeyUp(KeyCode::LShift, 0);
  CHECK(setup.parent.log == "PShift RShift ");
}

TEST_CASE("Text editor moved to another frame by its Enter handler passes on a held Return's "
          "release",
          "[integration]") {
  EditorInWindow setup;
  setup.editor.onEnterKey() += [&] {
    setup.cell_b.addChild(setup.editor);
    setup.editor.setBounds(0, 0, 300, 100);
    setup.parent.requestKeyboardFocus();
  };
  setup.editor.requestKeyboardFocus();
  setup.window.handleKeyDown(KeyCode::Return, 0, false);
  setup.window.handleKeyUp(KeyCode::Return, 0);
  setup.parent.log.clear();

  setup.window.handleKeyDown(KeyCode::Return, 0, false);
  setup.click(350, 20, 0);
  setup.window.handleKeyUp(KeyCode::Return, 0);
  CHECK(setup.parent.log == "PReturn RReturn ");
}

TEST_CASE("Clicking a focused text editor keeps a held key's release in it", "[integration]") {
  EditorInWindow setup;
  KeyCode key_code = KeyCode::LShift;
  int modifiers = kModifierShift;
  SECTION("Shift") { }
  SECTION("Arrow") {
    key_code = KeyCode::Left;
    modifiers = 0;
  }

  setup.editor.requestKeyboardFocus();
  setup.window.handleKeyDown(key_code, modifiers, false);
  setup.click(50, 20, modifiers);
  setup.window.handleKeyUp(key_code, 0);
  CHECK(setup.editor.hasKeyboardFocus());
  CHECK(setup.parent.log.empty());
}

TEST_CASE("Text editor keeps a held key's release across a window refocus", "[integration]") {
  EditorInWindow setup;
  setup.editor.requestKeyboardFocus();
  setup.window.handleKeyDown(KeyCode::A, 0, false);
  setup.window.handleFocusGained();
  setup.window.handleKeyUp(KeyCode::A, 0);
  CHECK(setup.parent.log.empty());
}
