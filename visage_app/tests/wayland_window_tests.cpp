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
#include <cmath>
#include <cstdlib>
#include <vector>

#if VISAGE_LINUX
using namespace visage;

namespace {
  bool useWaylandWindowing() {
    setenv("VISAGE_WINDOWING", "wayland", 1);
    std::unique_ptr<Window> window = createWindow(Dimension::logicalPixels(100),
                                                  Dimension::logicalPixels(100));
    return window && window->waylandDisplay();
  }
}

TEST_CASE("Wayland windows map, present frames and close", "[integration]") {
  if (!useWaylandWindowing())
    SKIP("No Wayland compositor");

  ApplicationWindow app;
  int frames = 0;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff00ff00);
    canvas.fill(0, 0, app.width(), app.height());
    if (++frames == 10)
      app.close();
    else
      app.redraw();
  };

  app.show(Dimension::logicalPixels(200), Dimension::logicalPixels(100));
  Window* window = app.window();
  REQUIRE(window);
  REQUIRE(window->waylandDisplay());
  REQUIRE(window->dpiScale() == defaultDpiScale());
  REQUIRE(window->clientWidth() == std::round(200 * window->dpiScale()));
  REQUIRE(window->clientHeight() == std::round(100 * window->dpiScale()));

  app.runEventLoop();
  REQUIRE(frames >= 10);
  REQUIRE(window->isClosed());
  REQUIRE_FALSE(window->isShowing());
}

TEST_CASE("Wayland windows describe the frame they need drawn around the client area",
          "[integration]") {
  if (!useWaylandWindowing())
    SKIP("No Wayland compositor");

  std::unique_ptr<Window> window = createWindow(Dimension::logicalPixels(300),
                                                Dimension::logicalPixels(200));
  REQUIRE(window);
  if (window->frameDecorationHandle() == nullptr)
    SKIP("The compositor draws window decorations");

  std::vector<Window::FrameDecoration> decorations;
  window->setFrameDecorationCallback([&](const Window::FrameDecoration& decoration) {
    decorations.push_back(decoration);
  });
  window->setWindowTitle("Frame");
  int ticks = 0;
  window->setDrawCallback([&](double) {
    if (++ticks == 5)
      window->close();
  });
  window->show();
  window->runEventLoop();

  REQUIRE_FALSE(decorations.empty());
  const Window::FrameDecoration& decoration = decorations.back();
  REQUIRE(decoration.title == "Frame");
  REQUIRE(decoration.dpi_scale == window->dpiScale());
  REQUIRE(std::abs(decoration.window.width() - window->clientWidth()) <= 1);
  REQUIRE(decoration.title_bar.height() > 0);
  REQUIRE(std::abs(decoration.window.height() - decoration.title_bar.height() - window->clientHeight()) <= 1);
  REQUIRE(decoration.window.x() >= 0);
  REQUIRE(decoration.window.right() <= decoration.width);
  REQUIRE(decoration.window.bottom() <= decoration.height);
  REQUIRE(decoration.close_button.width() > 0);
  REQUIRE(decoration.close_button.right() <= decoration.window.right());
  REQUIRE(decoration.close_button.bottom() <= decoration.title_bar.bottom());
  REQUIRE(decoration.title_font_size > 0.0f);
  REQUIRE((decoration.header_color >> 24) == 0xff);
}

TEST_CASE("Wayland windows can be shown again after hiding", "[integration]") {
  if (!useWaylandWindowing())
    SKIP("No Wayland compositor");

  ApplicationWindow app;
  int frames = 0;
  bool hidden = false;
  app.onDraw() = [&](Canvas& canvas) {
    canvas.setColor(0xff0000ff);
    canvas.fill(0, 0, app.width(), app.height());
    ++frames;
    if (frames == 5 && !hidden) {
      hidden = true;
      app.window()->hide();
      app.window()->show();
    }
    if (frames == 10)
      app.close();
    else
      app.redraw();
  };

  app.show(Dimension::logicalPixels(120), Dimension::logicalPixels(80));
  app.runEventLoop();
  REQUIRE(hidden);
  REQUIRE(frames >= 10);
}
#endif
