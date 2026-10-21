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
#include <cmath>
#include <cstdlib>
#include <memory>
#include <thread>

#if VISAGE_LINUX
#include <X11/Xlib.h>
#include <X11/Xutil.h>

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

TEST_CASE("X11 plugin windows leave the host window's size hints alone", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 300, 200, 0, 0, 0);
  XSizeHints* hints = XAllocSizeHints();
  hints->flags = PMinSize;
  hints->min_width = 300;
  hints->min_height = 200;
  XSetWMNormalHints(host, parent, hints);
  XMapWindow(host, parent);
  XSync(host, False);

  std::unique_ptr<visage::Window> window = createPluginWindow(Dimension::nativePixels(100),
                                                              Dimension::nativePixels(100), (void*)parent);
  REQUIRE(window);
  window->show();
  window->setFixedAspectRatio(true);
  window->processPluginFdEvents();
  XSync(host, False);

  long supplied_return;
  REQUIRE(XGetWMNormalHints(host, parent, hints, &supplied_return));
  CHECK(hints->flags == PMinSize);
  CHECK(hints->min_width == 300);
  CHECK(hints->min_height == 200);
  XFree(hints);

  window.reset();
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 plugin windows report contents resizes only for resizes they start",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 100, 100, 0, 0, 0);
  XMapWindow(host, parent);
  XSync(host, False);

  auto app = std::make_unique<ApplicationWindow>();
  int reports = 0;
  app->onWindowContentsResized() = [&reports] { reports++; };
  app->setWindowDimensions(Dimension::nativePixels(80), Dimension::nativePixels(80));
  app->setNativeWindowDimensionsFromHost(100, 100);
  CHECK(app->nativeWidth() == 100);
  CHECK(reports == 0);

  app->show((void*)parent);
  REQUIRE(app->window());
  app->window()->processPluginFdEvents();
  CHECK(reports == 0);

  XResizeWindow(host, parent, 150, 120);
  XSync(host, False);
  for (int i = 0; i < 200 && app->nativeWidth() != 150; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    app->window()->processPluginFdEvents();
  }
  CHECK(app->nativeWidth() == 150);
  CHECK(app->nativeHeight() == 120);
  CHECK(reports == 0);

  app->setNativeWindowDimensionsFromHost(160, 130);
  CHECK(app->window()->clientWidth() == 160);
  CHECK(app->nativeWidth() == 160);
  CHECK(reports == 0);

  app->setWindowDimensionsFromHost(85, 70);
  CHECK(app->window()->clientWidth() == std::round(85 * app->window()->dpiScale()));
  CHECK(reports == 0);

  app->setNativeWindowDimensions(180, 140);
  CHECK(app->window()->clientWidth() == 180);
  CHECK(reports == 1);

  app->setWindowDimensions(Dimension::nativePixels(190), Dimension::nativePixels(150));
  CHECK(app->window()->clientWidth() == 190);
  CHECK(reports == 2);

  app.reset();
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 plugin windows draw at their new size when the host resizes them before a frame",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 150, 0, 0, 0);
  XMapWindow(host, parent);
  XSync(host, False);

  ApplicationWindow app;
  app.onDraw() = [&app](Canvas& canvas) {
    canvas.setColor(0xff336699);
    canvas.fill(0, 0, app.width(), app.height());
  };
  app.setWindowDimensions(Dimension::nativePixels(200), Dimension::nativePixels(150));
  app.show((void*)parent);
  REQUIRE(app.window());

  auto process_events = [&](int milliseconds) {
    for (int i = 0; i < milliseconds / 5; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      app.window()->processPluginFdEvents();
    }
  };
  process_events(200);

  XResizeWindow(host, parent, 260, 190);
  XSync(host, False);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  app.window()->processPluginFdEvents();
  process_events(200);
  XSync(host, False);

  ::Window plugin = (::Window)app.window()->nativeHandle();
  XWindowAttributes attributes;
  REQUIRE(XGetWindowAttributes(host, plugin, &attributes));
  REQUIRE(attributes.width == 260);
  REQUIRE(attributes.height == 190);
  XImage* image = XGetImage(host, plugin, 0, 0, 260, 190, AllPlanes, ZPixmap);
  REQUIRE(image);
  unsigned long top_left = XGetPixel(image, 0, 0) & 0xffffff;
  unsigned long bottom_right = XGetPixel(image, 259, 189) & 0xffffff;
  XDestroyImage(image);
  CHECK(top_left != 0);
  CHECK(bottom_right == top_left);

  app.close();
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

#endif
