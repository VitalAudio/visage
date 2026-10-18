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

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#if VISAGE_LINUX
#include <X11/Xlib.h>
#include <X11/Xproto.h>

using namespace visage;

namespace {
  bool hasDisplay() {
    const char* display_name = std::getenv("DISPLAY");
    return display_name && display_name[0];
  }
}

static std::vector<int> error_request_codes;

static int recordErrorHandler(Display* display, XErrorEvent* error_event) {
  error_request_codes.push_back(error_event->request_code);
  return 0;
}

namespace {
  class ScopedErrorHandler {
  public:
    explicit ScopedErrorHandler(XErrorHandler handler) : previous_(XSetErrorHandler(handler)) { }
    ~ScopedErrorHandler() { XSetErrorHandler(previous_); }

  private:
    XErrorHandler previous_;
  };
}

static int requestCount(int request_code) {
  return std::count(error_request_codes.begin(), error_request_codes.end(), request_code);
}

class TitleBarEventHandler : public visage::Window::EventHandler {
public:
  HitTestResult handleHitTest(int x, int y) override {
    return y < ApplicationEditor::kDefaultClientTitleBarHeight ? HitTestResult::TitleBar :
                                                                 HitTestResult::Client;
  }
  HitTestResult currentHitTest() const override { return HitTestResult::Client; }
  void handleMouseMove(int x, int y, int button_state, int modifiers) override { }
  void handleMouseDown(MouseButton button_id, int x, int y, int button_state, int modifiers,
                       int repeat_clicks) override { }
  void handleMouseUp(MouseButton button_id, int x, int y, int button_state, int modifiers,
                     int repeat_clicks) override { }
  void handleMouseEnter(int x, int y) override { }
  void handleMouseLeave(int last_x, int last_y, int button_state, int modifiers) override { }
  void handleMouseWheel(float delta_x, float delta_y, float precise_x, float precise_y, int mouse_x,
                        int mouse_y, int button_state, int modifiers, bool momentum) override { }
  bool handleKeyDown(visage::KeyCode key_code, int modifiers, bool repeat) override {
    return false;
  }
  bool handleKeyUp(visage::KeyCode key_code, int modifiers) override { return false; }
  bool handleTextInput(const std::string& text) override { return false; }
  bool hasActiveTextEntry() override { return false; }
  void handleFocusLost() override { }
  void handleFocusGained() override { }
  void handleResized(int width, int height) override { }
  void handleWindowShown() override { }
  void handleWindowHidden() override { }
  bool handleCloseRequested() override { return true; }
  bool handleFileDrag(int x, int y, const std::vector<std::string>& files) override {
    return false;
  }
  void handleFileDragLeave() override { }
  bool handleFileDrop(int x, int y, const std::vector<std::string>& files) override {
    return false;
  }
  bool isDragDropSource() override { return false; }
  std::string startDragDropSource() override { return ""; }
  void cleanupDragDropSource() override { }
};

TEST_CASE("X11 windows stop a drag resize or move when the window is already destroyed",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  auto drag_destroyed_window = [](IPoint press) {
    error_request_codes.clear();
    TitleBarEventHandler handler;
    std::unique_ptr<visage::Window> window = createWindow(Dimension::nativePixels(200),
                                                          Dimension::nativePixels(200),
                                                          visage::Window::Decoration::Client);
    REQUIRE(window);
    Display* window_display = (Display*)window->globalDisplay();
    ScopedErrorHandler recorder(recordErrorHandler);
    window->setEventHandler(&handler);
    window->show();

    Display* display = XOpenDisplay(nullptr);
    REQUIRE(display);
    ::Window handle = (::Window)window->nativeHandle();
    XEvent button = {};
    button.xbutton.type = ButtonPress;
    button.xbutton.window = handle;
    button.xbutton.button = Button1;
    button.xbutton.x = press.x;
    button.xbutton.y = press.y;
    button.xbutton.x_root = press.x;
    button.xbutton.y_root = press.y;
    button.xbutton.same_screen = True;
    XSendEvent(display, handle, False, ButtonPressMask, &button);

    XEvent motion = {};
    motion.xmotion.type = MotionNotify;
    motion.xmotion.window = handle;
    motion.xmotion.x = 20;
    motion.xmotion.y = 20;
    motion.xmotion.x_root = 20;
    motion.xmotion.y_root = 20;
    motion.xmotion.same_screen = True;
    XSendEvent(display, handle, False, PointerMotionMask, &motion);

    XDestroyWindow(display, handle);
    XSync(display, False);
    window->runEventLoop();
    XCloseDisplay(display);
    XSync(window_display, False);
  };

  SECTION("Resizing from the border") {
    drag_destroyed_window({ 1, 1 });
    CHECK(requestCount(X_GetGeometry) > 0);
    REQUIRE(requestCount(X_ConfigureWindow) == 0);
  }

  SECTION("Moving by the title bar") {
    drag_destroyed_window({ 30, 15 });
    CHECK(requestCount(X_TranslateCoords) > 0);
    REQUIRE(requestCount(X_ConfigureWindow) == 0);
  }
}

#endif
