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
#include <vector>

#if VISAGE_LINUX
#include <X11/Xlib.h>

using namespace visage;

namespace {
  bool hasDisplay() {
    const char* display_name = std::getenv("DISPLAY");
    return display_name && display_name[0];
  }

  std::vector<unsigned int> pixels(const Screenshot& screenshot) {
    std::vector<unsigned int> result;
    for (int y = 0; y < screenshot.height(); ++y) {
      for (int x = 0; x < screenshot.width(); ++x)
        result.push_back(screenshot.sample(x, y).toARGB());
    }
    return result;
  }
}

TEST_CASE("Plugin windows draw the same after the renderer restarts between opens",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 200, 0, 0, 0);
  XSync(host, False);

  auto open = [parent]() {
    ApplicationWindow app;
    app.onDraw() = [&app](Canvas& canvas) {
      canvas.setColor(0xff203040);
      canvas.fill(0, 0, app.width(), app.height());
      canvas.setColor(0xffc08040);
      canvas.circle(10, 10, 30);
    };
    app.show(Dimension::nativePixels(100), Dimension::nativePixels(80), (void*)parent);
    REQUIRE(Renderer::instance().initialized());
    app.drawWindow();
    return pixels(app.takeScreenshot());
  };

  std::vector<unsigned int> first = open();
  REQUIRE_FALSE(Renderer::instance().initialized());
  REQUIRE(first.size() == 100 * 80);
  REQUIRE(open() == first);
  REQUIRE_FALSE(Renderer::instance().initialized());

  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("Canvases paired after the renderer shuts down initialize it again", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 200, 200, 0, 0, 0);
  XSync(host, False);

  std::unique_ptr<visage::Window> window = createPluginWindow(Dimension::nativePixels(100),
                                                              Dimension::nativePixels(80), (void*)parent);
  REQUIRE(window);
  Renderer::instance().initialize(window->initWindow(), window->globalDisplay());
  window->show();

  auto draw = [&window]() {
    Canvas canvas;
    canvas.pairToWindow(window->graphicsHandle(), window->clientWidth(), window->clientHeight());
    canvas.setColor(0xff306090);
    canvas.fill(0, 0, canvas.width(), canvas.height());
    canvas.submit();
    return canvas.takeScreenshot().sample(50, 40).toARGB();
  };

  REQUIRE(draw() == 0xff306090);
  REQUIRE_FALSE(Renderer::instance().initialized());
  REQUIRE(draw() == 0xff306090);
  REQUIRE_FALSE(Renderer::instance().initialized());

  window = nullptr;
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

#endif
