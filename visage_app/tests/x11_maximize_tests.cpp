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
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#if VISAGE_LINUX
#include <X11/Xlib.h>

using namespace visage;

namespace {
  bool hasDisplay() {
    const char* display_name = std::getenv("DISPLAY");
    return display_name && display_name[0];
  }

  class MaximizeButtonEventHandler : public visage::Window::EventHandler {
  public:
    HitTestResult handleHitTest(int x, int y) override { return HitTestResult::MaximizeButton; }
    HitTestResult currentHitTest() const override { return HitTestResult::MaximizeButton; }
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

  std::vector<long> maximizeActions(Display* display, ::Window window) {
    Atom wm_state = XInternAtom(display, "_NET_WM_STATE", False);
    Atom max_horizontal = XInternAtom(display, "_NET_WM_STATE_MAXIMIZED_HORZ", False);
    Atom max_vertical = XInternAtom(display, "_NET_WM_STATE_MAXIMIZED_VERT", False);
    XSync(display, False);

    std::vector<long> actions;
    XEvent event;
    while (XCheckTypedEvent(display, ClientMessage, &event)) {
      if (event.xclient.window == window && event.xclient.message_type == wm_state &&
          event.xclient.data.l[1] == static_cast<long>(max_horizontal) &&
          event.xclient.data.l[2] == static_cast<long>(max_vertical)) {
        actions.push_back(event.xclient.data.l[0]);
      }
    }
    return actions;
  }

  void sendClick(Display* display, ::Window window) {
    XEvent button = {};
    button.xbutton.window = window;
    button.xbutton.button = Button1;
    button.xbutton.x = 100;
    button.xbutton.y = 100;
    button.xbutton.x_root = 100;
    button.xbutton.y_root = 100;
    button.xbutton.same_screen = True;
    button.xbutton.type = ButtonPress;
    XSendEvent(display, window, False, ButtonPressMask, &button);
    button.xbutton.type = ButtonRelease;
    XSendEvent(display, window, False, ButtonReleaseMask, &button);
  }
}

TEST_CASE("X11 maximize button toggles and showMaximized only maximizes", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* observer = XOpenDisplay(nullptr);
  REQUIRE(observer);
  XSelectInput(observer, DefaultRootWindow(observer), SubstructureNotifyMask);
  XSync(observer, False);

  MaximizeButtonEventHandler handler;
  std::unique_ptr<visage::Window> window = createWindow(Dimension::nativePixels(200),
                                                        Dimension::nativePixels(200),
                                                        visage::Window::Decoration::Client);
  REQUIRE(window);
  ::Window handle = (::Window)window->nativeHandle();
  window->setEventHandler(&handler);
  window->show();

  sendClick(observer, handle);
  sendClick(observer, handle);
  XSync(observer, False);
  window->close();
  window->runEventLoop();
  REQUIRE(maximizeActions(observer, handle) == std::vector<long> { 2, 2 });

  window->showMaximized();
  window->showMaximized();
  REQUIRE(maximizeActions(observer, handle) == std::vector<long> { 1, 1 });

  window.reset();
  XCloseDisplay(observer);
}

#endif
