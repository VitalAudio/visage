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
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#if VISAGE_LINUX
#include <X11/Xlib.h>

using namespace visage;

namespace {
  bool hasDisplay() {
    const char* display_name = std::getenv("DISPLAY");
    return display_name && display_name[0];
  }
}

static int ignoreXErrors(Display* display, XErrorEvent* error_event) {
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

TEST_CASE("X11 plugin windows keep their size when resized just before their parent is destroyed",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  ScopedErrorHandler ignore_errors(ignoreXErrors);
  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 200, 0, 0, 0);
  XMapWindow(host, parent);
  XSync(host, False);

  std::unique_ptr<visage::Window> window = createPluginWindow(Dimension::nativePixels(100),
                                                              Dimension::nativePixels(100), (void*)parent);
  REQUIRE(window);
  window->show();
  window->processPluginFdEvents();
  REQUIRE(window->clientWidth() == 100);
  REQUIRE(window->clientHeight() == 100);

  SECTION("The parent is resized") {
    XResizeWindow(host, parent, 150, 150);
  }

  SECTION("The plugin window is resized") {
    XResizeWindow(host, (::Window)window->nativeHandle(), 150, 150);
  }

  XDestroyWindow(host, parent);
  XSync(host, False);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  window->processPluginFdEvents();
  CHECK(window->clientWidth() == 100);
  CHECK(window->clientHeight() == 100);

  window.reset();
  XCloseDisplay(host);
}

#endif
