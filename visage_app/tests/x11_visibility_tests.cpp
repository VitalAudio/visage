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

TEST_CASE("X11 windows stop drawing while they or their parent are unmapped", "[integration]") {
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
  auto settles_to = [&](bool visible) {
    for (int i = 0; i < 200; ++i) {
      window->processPluginFdEvents();
      if (window->isVisible() == visible)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  };
  auto stays = [&](bool visible) {
    for (int i = 0; i < 40; ++i) {
      window->processPluginFdEvents();
      if (window->isVisible() != visible)
        return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return true;
  };

  window->hide();
  REQUIRE(settles_to(false));
  window->show();
  REQUIRE(settles_to(true));

  XUnmapWindow(host, parent);
  XSync(host, False);
  REQUIRE(settles_to(false));
  XMapWindow(host, parent);
  XSync(host, False);
  REQUIRE(settles_to(true));

  window->hide();
  REQUIRE(settles_to(false));
  XUnmapWindow(host, parent);
  XSync(host, False);
  XMapWindow(host, parent);
  XSync(host, False);
  REQUIRE(stays(false));

  XUnmapWindow(host, parent);
  XSync(host, False);
  window->show();
  REQUIRE(stays(false));
  XMapWindow(host, parent);
  XSync(host, False);
  REQUIRE(settles_to(true));

  window.reset();
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 windows become visible when an ancestor is mapped after them", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window top = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 200, 0, 0, 0);
  ::Window parent = XCreateSimpleWindow(host, top, 0, 0, 200, 200, 0, 0, 0);
  XMapWindow(host, parent);
  XSync(host, False);

  std::unique_ptr<visage::Window> window = createPluginWindow(Dimension::nativePixels(100),
                                                              Dimension::nativePixels(100), (void*)parent);
  REQUIRE(window);
  auto settles_to = [&](bool visible) {
    for (int i = 0; i < 200; ++i) {
      window->processPluginFdEvents();
      if (window->isVisible() == visible)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  };

  window->show();
  REQUIRE(settles_to(false));
  XMapWindow(host, top);
  XSync(host, False);
  REQUIRE(settles_to(true));

  window.reset();
  XDestroyWindow(host, top);
  XCloseDisplay(host);
}

#endif
