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
#include <functional>
#include <memory>
#include <vector>

#if VISAGE_LINUX
#include <X11/Xlib.h>
#include <X11/Xproto.h>

using namespace visage;

namespace {
  class Steps : public EventTimer {
  public:
    explicit Steps(std::vector<std::function<bool()>> steps) : steps_(std::move(steps)) {
      startTimer(20);
    }

    void timerCallback() override {
      if (next_ < steps_.size() && steps_[next_]())
        next_++;
    }

  private:
    std::vector<std::function<bool()>> steps_;
    size_t next_ = 0;
  };

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

TEST_CASE("X11 windows stop a drag resize when the window is already destroyed", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  error_request_codes.clear();
  {
    ApplicationWindow app;
    app.setWindowDecoration(visage::Window::Decoration::Client);
    app.show(Dimension::nativePixels(200), Dimension::nativePixels(200));
    Display* window_display = (Display*)app.window()->globalDisplay();
    ScopedErrorHandler recorder(recordErrorHandler);

    Display* display = XOpenDisplay(nullptr);
    REQUIRE(display);
    ::Window window = (::Window)app.window()->nativeHandle();
    Steps steps({ [&] {
      XEvent press = {};
      press.xbutton.type = ButtonPress;
      press.xbutton.window = window;
      press.xbutton.button = Button1;
      press.xbutton.x = 1;
      press.xbutton.y = 1;
      press.xbutton.same_screen = True;
      XSendEvent(display, window, False, ButtonPressMask, &press);

      XEvent motion = {};
      motion.xmotion.type = MotionNotify;
      motion.xmotion.window = window;
      motion.xmotion.x = 20;
      motion.xmotion.y = 20;
      motion.xmotion.x_root = 20;
      motion.xmotion.y_root = 20;
      motion.xmotion.same_screen = True;
      XSendEvent(display, window, False, PointerMotionMask, &motion);

      XDestroyWindow(display, window);
      XSync(display, False);
      return true;
    } });
    app.runEventLoop();
    XCloseDisplay(display);
    XSync(window_display, False);
  }

  CHECK(requestCount(X_GetGeometry) > 0);
  REQUIRE(requestCount(X_ConfigureWindow) == 0);
}

#endif
