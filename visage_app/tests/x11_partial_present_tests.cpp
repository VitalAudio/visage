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
#include <iterator>
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

  int ignoreXErrors(Display*, XErrorEvent*) {
    return 0;
  }

  class ScopedErrorHandler {
  public:
    explicit ScopedErrorHandler(XErrorHandler handler) : previous_(XSetErrorHandler(handler)) { }
    ~ScopedErrorHandler() { XSetErrorHandler(previous_); }

  private:
    XErrorHandler previous_;
  };

  class ColorFrame : public Frame {
  public:
    void draw(Canvas& canvas) override {
      canvas.setColor(color);
      canvas.fill(0, 0, width(), height());
    }

    unsigned int color = 0;
  };

  bool windowShows(Display* display, ::Window window, int width, int height, unsigned int left,
                   unsigned int right) {
    XImage* image = XGetImage(display, window, 0, 0, width, height, AllPlanes, ZPixmap);
    if (image == nullptr)
      return false;

    bool result = (XGetPixel(image, width / 4, height / 2) & 0xffffff) == (left & 0xffffff) &&
                  (XGetPixel(image, 3 * width / 4, height / 2) & 0xffffff) == (right & 0xffffff);
    XDestroyImage(image);
    return result;
  }

  bool waitForWindow(Display* display, ::Window window, int width, int height, unsigned int left,
                     unsigned int right) {
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < end) {
      if (windowShows(display, window, width, height, left, right))
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }
}

TEST_CASE("X11 windows show every change when only part of the window redraws", "[integration]") {
  if (!hasDisplay())
    SKIP("No X display");

  ScopedErrorHandler ignore_errors(ignoreXErrors);
  Display* display = XOpenDisplay(nullptr);
  REQUIRE(display);

  ApplicationEditor app;
  ColorFrame left;
  ColorFrame right;
  app.addChild(left);
  app.addChild(right);

  std::unique_ptr<visage::Window> window = createWindow(Dimension::nativePixels(40),
                                                        Dimension::nativePixels(40));
  app.addToWindow(window.get());
  left.setBounds(0, 0, app.width() / 2, app.height());
  right.setBounds(app.width() / 2, 0, app.width() - app.width() / 2, app.height());
  window->show();

  ::Window handle = static_cast<::Window>(reinterpret_cast<uintptr_t>(window->nativeHandle()));
  int width = app.nativeWidth();
  int height = app.nativeHeight();

  struct Colors {
    unsigned int left = 0;
    unsigned int right = 0;
  };
  Colors steps[] = { { 0xffff0000, 0xff0000ff }, { 0xff00ff00, 0xff0000ff },
                     { 0xff00ff00, 0xffffff00 }, { 0xffffffff, 0xffffff00 },
                     { 0xffffffff, 0xffff00ff }, { 0xff00ffff, 0xffff00ff },
                     { 0xff00ffff, 0xff808080 }, { 0xff404040, 0xff808080 } };

  int failed_step = -1;
  for (int i = 0; i < static_cast<int>(std::size(steps)) && failed_step < 0; ++i) {
    const Colors& colors = steps[i];
    if (left.color != colors.left) {
      left.color = colors.left;
      left.redraw();
    }
    if (right.color != colors.right) {
      right.color = colors.right;
      right.redraw();
    }
    app.drawWindow();
    if (!waitForWindow(display, handle, width, height, colors.left, colors.right))
      failed_step = i;
  }

  app.removeFromWindow();
  XCloseDisplay(display);
  REQUIRE(failed_step == -1);
}

#endif
