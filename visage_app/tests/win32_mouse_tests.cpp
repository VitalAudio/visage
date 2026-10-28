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
#include <memory>
#include <string>
#include <vector>

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

using namespace visage;

namespace {
  class MouseRecorder : public Window::EventHandler {
  public:
    HitTestResult handleHitTest(int, int) override { return hit_test; }
    HitTestResult currentHitTest() const override { return hit_test; }
    void handleMouseMove(int, int, int button_state, int) override {
      move_button_states.push_back(button_state);
    }
    void handleMouseDown(MouseButton, int, int, int, int, int) override { }
    void handleMouseUp(MouseButton, int, int, int, int, int) override { }
    void handleMouseEnter(int, int) override { }
    void handleMouseLeave(int, int, int, int) override { leaves++; }
    void handleMouseWheel(float, float, float, float, int, int, int, int, bool) override { }

    bool handleKeyDown(KeyCode, int, bool) override { return true; }
    bool handleKeyUp(KeyCode, int) override { return true; }

    bool handleTextInput(const std::string&) override { return false; }
    bool hasActiveTextEntry() override { return false; }

    void handleFocusLost() override { }
    void handleFocusGained() override { }
    void handleResized(int, int) override { }
    void handleWindowShown() override { }
    void handleWindowHidden() override { }
    bool handleCloseRequested() override { return true; }

    bool handleFileDrag(int, int, const std::vector<std::string>&) override { return false; }
    void handleFileDragLeave() override { }
    bool handleFileDrop(int, int, const std::vector<std::string>&) override { return false; }

    bool isDragDropSource() override { return false; }
    std::string startDragDropSource() override { return {}; }
    void cleanupDragDropSource() override { }

    HitTestResult hit_test = HitTestResult::Client;
    std::vector<int> move_button_states;
    int leaves = 0;
  };

  void runMessages() {
    MSG message;
    while (PeekMessageW(&message, nullptr, 0, 0, PM_REMOVE)) {
      TranslateMessage(&message);
      DispatchMessageW(&message);
    }
  }

  LPARAM screenPointLParam(int x, int y) {
    return MAKELPARAM(static_cast<WORD>(static_cast<short>(x)), static_cast<WORD>(static_cast<short>(y)));
  }

  struct ClientDecoratedWindow {
    ClientDecoratedWindow() {
      window = createWindow(Dimension::nativePixels(200), Dimension::nativePixels(100),
                            Window::Decoration::Client);
      if (window) {
        window->setEventHandler(&recorder);
        hwnd = static_cast<HWND>(window->nativeHandle());
      }
    }

    ~ClientDecoratedWindow() {
      if (window)
        window->clearEventHandler();
      window.reset();
      runMessages();
    }

    LPARAM titleBarPosition() const {
      RECT rect {};
      GetWindowRect(hwnd, &rect);
      return screenPointLParam(rect.left + 20, rect.top + 5);
    }

    void clientMove() { SendMessageW(hwnd, WM_MOUSEMOVE, 0, MAKELPARAM(20, 60)); }
    void titleBarMove() { SendMessageW(hwnd, WM_NCMOUSEMOVE, HTCAPTION, titleBarPosition()); }

    MouseRecorder recorder;
    std::unique_ptr<Window> window;
    HWND hwnd = nullptr;
  };

  class ScopedMouseButtonsUp {
  public:
    ScopedMouseButtonsUp() {
      GetKeyboardState(saved_);
      BYTE state[256];
      GetKeyboardState(state);
      for (int key : { VK_LBUTTON, VK_RBUTTON, VK_MBUTTON, VK_XBUTTON1, VK_XBUTTON2 })
        state[key] = 0;
      SetKeyboardState(state);
    }

    ~ScopedMouseButtonsUp() { SetKeyboardState(saved_); }

  private:
    BYTE saved_[256] {};
  };

  struct HitTestCase {
    const char* name;
    WPARAM hit_test;
  };
}

TEST_CASE("Win32 title bar mouse moves don't report the hit-test code as buttons",
          "[integration]") {
  ScopedMouseButtonsUp buttons_up;
  ClientDecoratedWindow setup;
  setup.recorder.hit_test = HitTestResult::TitleBar;
  REQUIRE(setup.window);
  REQUIRE(setup.hwnd);

  RECT rect {};
  REQUIRE(GetWindowRect(setup.hwnd, &rect));
  LPARAM position = screenPointLParam(rect.left + 20, rect.top + 5);

  for (HitTestCase hit_test :
       { HitTestCase { "HTCAPTION", HTCAPTION }, HitTestCase { "HTMAXBUTTON", HTMAXBUTTON },
         HitTestCase { "HTCLOSE", HTCLOSE }, HitTestCase { "HTLEFT", HTLEFT },
         HitTestCase { "HTSYSMENU", HTSYSMENU }, HitTestCase { "HTMINBUTTON", HTMINBUTTON } }) {
    INFO(hit_test.name);
    setup.recorder.move_button_states.clear();
    SendMessageW(setup.hwnd, WM_NCMOUSEMOVE, hit_test.hit_test, position);
    REQUIRE(setup.recorder.move_button_states.size() == 1);
    CHECK(setup.recorder.move_button_states[0] == 0);
  }
}

TEST_CASE("Win32 title bar mouse moves report buttons that are really held", "[integration]") {
  BYTE saved[256];
  GetKeyboardState(saved);
  BYTE state[256];
  GetKeyboardState(state);
  state[VK_LBUTTON] = 0;
  state[VK_MBUTTON] = 0;
  state[VK_RBUTTON] = 0x80;
  SetKeyboardState(state);

  ClientDecoratedWindow setup;
  setup.recorder.hit_test = HitTestResult::TitleBar;
  REQUIRE(setup.window);

  SendMessageW(setup.hwnd, WM_NCMOUSEMOVE, HTMINBUTTON, setup.titleBarPosition());
  SetKeyboardState(saved);

  REQUIRE(setup.recorder.move_button_states.size() == 1);
  CHECK(setup.recorder.move_button_states[0] == kMouseButtonRight);
}

TEST_CASE("Win32 client mouse moves read buttons from wParam", "[integration]") {
  ScopedMouseButtonsUp buttons_up;
  ClientDecoratedWindow setup;
  setup.recorder.hit_test = HitTestResult::TitleBar;
  REQUIRE(setup.window);

  SendMessageW(setup.hwnd, WM_MOUSEMOVE, MK_LBUTTON | MK_RBUTTON, MAKELPARAM(20, 50));
  SendMessageW(setup.hwnd, WM_MOUSEMOVE, 0, MAKELPARAM(21, 50));
  REQUIRE(setup.recorder.move_button_states.size() == 2);
  CHECK(setup.recorder.move_button_states[0] == (kMouseButtonLeft | kMouseButtonRight));
  CHECK(setup.recorder.move_button_states[1] == 0);
}

TEST_CASE("Win32 content moves re-arm leave tracking after leaving into the title bar",
          "[integration]") {
  ClientDecoratedWindow setup;
  runMessages();
  REQUIRE(setup.hwnd);
  REQUIRE_FALSE(IsWindowVisible(setup.hwnd));

  setup.recorder.hit_test = HitTestResult::Client;
  setup.clientMove();

  setup.recorder.hit_test = HitTestResult::TitleBar;
  runMessages();

  setup.recorder.hit_test = HitTestResult::Client;
  setup.recorder.leaves = 0;
  setup.clientMove();
  runMessages();
  CHECK(setup.recorder.leaves == 1);
}

TEST_CASE("Win32 content moves re-arm leave tracking after a content leave", "[integration]") {
  ClientDecoratedWindow setup;
  runMessages();
  REQUIRE(setup.hwnd);

  for (int round = 0; round < 3; ++round) {
    INFO("round " << round);
    setup.recorder.leaves = 0;
    setup.clientMove();
    runMessages();
    CHECK(setup.recorder.leaves == 1);
  }
}

TEST_CASE("Win32 leaving through the title bar reports a leave", "[integration]") {
  ClientDecoratedWindow setup;
  runMessages();
  REQUIRE(setup.hwnd);

  setup.recorder.hit_test = HitTestResult::TitleBar;
  setup.recorder.leaves = 0;
  setup.titleBarMove();
  runMessages();
  CHECK(setup.recorder.leaves >= 1);
}

TEST_CASE("Win32 leaving the content after a trip through the title bar reports the leave",
          "[integration]") {
  ClientDecoratedWindow setup;
  runMessages();
  REQUIRE(setup.hwnd);

  setup.recorder.hit_test = HitTestResult::Client;
  setup.clientMove();
  setup.recorder.hit_test = HitTestResult::TitleBar;
  runMessages();
  setup.titleBarMove();
  setup.recorder.hit_test = HitTestResult::Client;
  runMessages();

  setup.recorder.leaves = 0;
  setup.clientMove();
  setup.clientMove();
  runMessages();
  CHECK(setup.recorder.leaves == 1);
}

#endif
