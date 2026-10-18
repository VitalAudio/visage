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

#if VISAGE_LINUX
#include "visage/app.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <functional>
#include <vector>
#include <X11/keysym.h>
#include <X11/Xlib.h>

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

  struct KeyRecord {
    visage::KeyCode key_code = visage::KeyCode::Unknown;
    bool down = false;
    int modifiers = 0;

    bool operator==(const KeyRecord& other) const {
      return key_code == other.key_code && down == other.down && modifiers == other.modifiers;
    }
  };
}

TEST_CASE("X11 windows send key events for modifier keys", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  ApplicationWindow app;
  Frame target;
  app.setFlexLayout(true);
  target.layout().setFlexGrow(1.0f);
  target.setAcceptsKeystrokes(true);
  app.addChild(target);

  std::vector<KeyRecord> records;
  target.onKeyPress() = [&](const KeyEvent& e) {
    records.push_back({ e.keyCode(), true, e.modifierMask() });
    return true;
  };
  target.onKeyRelease() = [&](const KeyEvent& e) {
    records.push_back({ e.keyCode(), false, e.modifierMask() });
    return true;
  };

  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));
  target.requestKeyboardFocus();

  Display* display = XOpenDisplay(nullptr);
  REQUIRE(display);
  ::Window window = (::Window)app.window()->nativeHandle();
  auto send = [&](int type, KeySym keysym, unsigned int state) {
    XEvent event = {};
    event.xkey.type = type;
    event.xkey.display = display;
    event.xkey.window = window;
    event.xkey.root = DefaultRootWindow(display);
    event.xkey.keycode = XKeysymToKeycode(display, keysym);
    event.xkey.state = state;
    event.xkey.same_screen = True;
    XSendEvent(display, window, False, type == KeyPress ? KeyPressMask : KeyReleaseMask, &event);
    XFlush(display);
  };

  Steps steps({ [&] {
                 send(KeyPress, XK_Shift_L, 0);
                 send(KeyPress, XK_Control_R, ShiftMask);
                 send(KeyRelease, XK_Control_R, ShiftMask | ControlMask);
                 send(KeyRelease, XK_Shift_L, ShiftMask);
                 return true;
               },
                [&, ticks = 0]() mutable { return records.size() >= 4 || ++ticks > 100; },
                [&] {
                  app.close();
                  return true;
                } });
  app.runEventLoop();
  XCloseDisplay(display);

  std::vector<KeyRecord> expected = { { visage::KeyCode::LShift, true, kModifierShift },
                                      { visage::KeyCode::RCtrl, true, kModifierShift | kModifierRegCtrl },
                                      { visage::KeyCode::RCtrl, false, kModifierShift },
                                      { visage::KeyCode::LShift, false, 0 } };
  CHECK(records == expected);
}

#endif
