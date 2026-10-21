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
#include "visage/widgets.h"

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

TEST_CASE("X11 plugin windows only cancel a held click when they really lose focus",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  int detail = NotifyPointer;
  int expected_toggles = 1;
  SECTION("Focus moves only because the pointer is over the window") { }
  SECTION("The window really loses focus") {
    detail = NotifyNonlinear;
    expected_toggles = 0;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window parent = XCreateSimpleWindow(host, DefaultRootWindow(host), 0, 0, 100, 100, 0, 0, 0);
  XMapWindow(host, parent);
  XSync(host, False);

  auto app = std::make_unique<ApplicationWindow>();
  UiButton button("Button");
  int toggles = 0;
  button.onToggle() += [&toggles](Button*, bool) { toggles++; };
  app->addChild(button);
  app->setWindowDimensions(Dimension::nativePixels(100), Dimension::nativePixels(100));
  app->show((void*)parent);
  REQUIRE(app->window());
  button.setBounds(app->localBounds());
  ::Window window = (::Window)app->window()->nativeHandle();

  auto send = [&](XEvent event) {
    event.xany.window = window;
    XSendEvent(host, window, False, NoEventMask, &event);
    XSync(host, False);
    for (int i = 0; i < 10; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      app->window()->processPluginFdEvents();
    }
  };

  XEvent press = {};
  press.xbutton.type = ButtonPress;
  press.xbutton.button = Button1;
  press.xbutton.x = 20;
  press.xbutton.y = 20;
  press.xbutton.same_screen = True;
  send(press);

  XEvent focus_out = {};
  focus_out.xfocus.type = FocusOut;
  focus_out.xfocus.mode = NotifyNormal;
  focus_out.xfocus.detail = detail;
  send(focus_out);

  XEvent release = press;
  release.xbutton.type = ButtonRelease;
  release.xbutton.state = Button1Mask;
  send(release);
  CHECK(toggles == expected_toggles);

  app.reset();
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

#endif
