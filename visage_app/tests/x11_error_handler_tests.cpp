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

static int hostErrorHandler(Display* display, XErrorEvent* error_event) {
  return 0;
}

static XErrorHandler currentErrorHandler() {
  XErrorHandler current = XSetErrorHandler(nullptr);
  XSetErrorHandler(current);
  return current;
}

TEST_CASE("X11 plugin windows survive the host destroying their parent first", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

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

  XDestroyWindow(host, parent);
  XSync(host, False);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  window->processPluginFdEvents();
  window.reset();

  XCloseDisplay(host);
  SUCCEED("Destroying the plugin window after its parent didn't end the process");
}

TEST_CASE("X11 plugin windows keep the host's X error handler", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 200, 0, 0, 0);
  XSync(host, False);

  XErrorHandler previous = XSetErrorHandler(hostErrorHandler);
  std::unique_ptr<visage::Window> window = createPluginWindow(Dimension::nativePixels(100),
                                                              Dimension::nativePixels(100), (void*)parent);
  REQUIRE(window);
  REQUIRE(XSetErrorHandler(previous) == hostErrorHandler);

  window.reset();
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 plugin windows restore the X error handler when the last one closes",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 200, 0, 0, 0);
  XSync(host, False);

  XErrorHandler previous = XSetErrorHandler(nullptr);
  XErrorHandler default_handler = currentErrorHandler();
  std::unique_ptr<visage::Window> first = createPluginWindow(Dimension::nativePixels(100),
                                                             Dimension::nativePixels(100), (void*)parent);
  std::unique_ptr<visage::Window> second = createPluginWindow(Dimension::nativePixels(100),
                                                              Dimension::nativePixels(100), (void*)parent);
  REQUIRE(first);
  REQUIRE(second);
  REQUIRE(currentErrorHandler() != default_handler);

  first.reset();
  REQUIRE(currentErrorHandler() != default_handler);
  second.reset();
  REQUIRE(currentErrorHandler() == default_handler);

  XSetErrorHandler(previous);
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 standalone and plugin windows share the X error handler", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 200, 0, 0, 0);
  XSync(host, False);

  XErrorHandler previous = XSetErrorHandler(nullptr);
  XErrorHandler default_handler = currentErrorHandler();
  auto create_plugin = [&] {
    return createPluginWindow(Dimension::nativePixels(100), Dimension::nativePixels(100), (void*)parent);
  };
  auto create_standalone = [] {
    return createWindow(Dimension::nativePixels(100), Dimension::nativePixels(100));
  };

  SECTION("A standalone window closing last") {
    std::unique_ptr<visage::Window> plugin = create_plugin();
    std::unique_ptr<visage::Window> standalone = create_standalone();
    REQUIRE(plugin);
    REQUIRE(standalone);
    plugin.reset();
    REQUIRE(currentErrorHandler() != default_handler);
    standalone.reset();
    REQUIRE(currentErrorHandler() == default_handler);
  }

  SECTION("A plugin window closing last") {
    std::unique_ptr<visage::Window> standalone = create_standalone();
    std::unique_ptr<visage::Window> plugin = create_plugin();
    REQUIRE(standalone);
    REQUIRE(plugin);
    standalone.reset();
    REQUIRE(currentErrorHandler() != default_handler);
    plugin.reset();
    REQUIRE(currentErrorHandler() == default_handler);
  }

  SECTION("A standalone window keeps the host's handler") {
    XSetErrorHandler(hostErrorHandler);
    std::unique_ptr<visage::Window> standalone = create_standalone();
    REQUIRE(standalone);
    REQUIRE(currentErrorHandler() == hostErrorHandler);
    standalone.reset();
    REQUIRE(currentErrorHandler() == hostErrorHandler);
  }

  XSetErrorHandler(previous);
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

#endif
