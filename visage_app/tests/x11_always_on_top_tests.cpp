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
#include <vector>

#if VISAGE_LINUX
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

using namespace visage;

namespace {
  bool hasDisplay() {
    const char* display_name = std::getenv("DISPLAY");
    return display_name && display_name[0];
  }

  std::vector<Atom> windowStates(Display* display, ::Window window, Atom wm_state) {
    Atom type = None;
    int format = 0;
    unsigned long count = 0, remaining = 0;
    unsigned char* data = nullptr;
    std::vector<Atom> states;
    if (XGetWindowProperty(display, window, wm_state, 0, 32, False, XA_ATOM, &type, &format, &count,
                           &remaining, &data) == Success &&
        data) {
      Atom* atoms = reinterpret_cast<Atom*>(data);
      states.assign(atoms, atoms + count);
      XFree(data);
    }
    return states;
  }

  bool nextStateMessage(Display* display, ::Window window, Atom wm_state, XEvent& event) {
    for (int i = 0; i < 200; ++i) {
      while (XCheckTypedEvent(display, ClientMessage, &event)) {
        if (event.xclient.window == window && event.xclient.message_type == wm_state)
          return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }
}

TEST_CASE("X11 windows ask to stay on top", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Display* observer = XOpenDisplay(nullptr);
  REQUIRE(observer);
  Atom wm_state = XInternAtom(observer, "_NET_WM_STATE", False);
  Atom above = XInternAtom(observer, "_NET_WM_STATE_ABOVE", False);
  XSelectInput(observer, DefaultRootWindow(observer), SubstructureNotifyMask);
  XSync(observer, False);

  std::unique_ptr<visage::Window> window = createWindow(Dimension::nativePixels(100),
                                                        Dimension::nativePixels(100));
  REQUIRE(window);
  ::Window handle = (::Window)window->nativeHandle();
  Display* window_display = (Display*)window->globalDisplay();

  window->setAlwaysOnTop(true);
  XSync(window_display, False);
  REQUIRE(windowStates(observer, handle, wm_state) == std::vector<Atom> { above });
  window->setAlwaysOnTop(false);
  XSync(window_display, False);
  REQUIRE(windowStates(observer, handle, wm_state).empty());

  Atom maximized = XInternAtom(observer, "_NET_WM_STATE_MAXIMIZED_VERT", False);
  XChangeProperty(observer, handle, wm_state, XA_ATOM, 32, PropModeReplace, (unsigned char*)&maximized, 1);
  XSync(observer, False);
  window->setAlwaysOnTop(true);
  XSync(window_display, False);
  REQUIRE(windowStates(observer, handle, wm_state) == std::vector<Atom> { maximized, above });
  window->setAlwaysOnTop(false);
  XSync(window_display, False);
  REQUIRE(windowStates(observer, handle, wm_state) == std::vector<Atom> { maximized });

  XEvent event;
  Atom icccm_state = XInternAtom(observer, "WM_STATE", False);
  Atom hidden = XInternAtom(observer, "_NET_WM_STATE_HIDDEN", False);
  long iconic[] = { IconicState, None };
  XChangeProperty(observer, handle, icccm_state, icccm_state, 32, PropModeReplace,
                  (unsigned char*)iconic, 2);
  XChangeProperty(observer, handle, wm_state, XA_ATOM, 32, PropModeReplace, (unsigned char*)&hidden, 1);
  XSync(observer, False);
  window->setAlwaysOnTop(true);
  REQUIRE(nextStateMessage(observer, handle, wm_state, event));
  CHECK(event.xclient.data.l[0] == 1);
  XSync(window_display, False);
  REQUIRE(windowStates(observer, handle, wm_state) == std::vector<Atom> { hidden });
  XDeleteProperty(observer, handle, icccm_state);
  XSync(observer, False);

  window->show();
  window->setAlwaysOnTop(true);
  REQUIRE(nextStateMessage(observer, handle, wm_state, event));
  CHECK(event.xclient.data.l[0] == 1);
  CHECK(event.xclient.data.l[1] == static_cast<long>(above));

  window->setAlwaysOnTop(false);
  REQUIRE(nextStateMessage(observer, handle, wm_state, event));
  CHECK(event.xclient.data.l[0] == 0);
  CHECK(event.xclient.data.l[1] == static_cast<long>(above));

  window.reset();
  XCloseDisplay(observer);
}

#endif
