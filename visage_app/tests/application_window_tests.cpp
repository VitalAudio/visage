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
#include "visage_utils/child_process.h"
#include "visage_utils/file_system.h"

#include <catch2/catch_test_macros.hpp>
#include <cstdlib>
#include <functional>
#include <vector>

#if VISAGE_LINUX
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
}

TEST_CASE("A window opened after windowless rendering still draws", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  Canvas windowless;
  windowless.setWindowless(10, 10);
  windowless.takeScreenshot();

  ApplicationWindow app;
  app.onDraw() = [&app](Canvas& canvas) {
    canvas.setColor(0xff20c040);
    canvas.fill(0, 0, app.width(), app.height());
  };
  app.show(Dimension::nativePixels(60), Dimension::nativePixels(40));

  Screenshot screenshot;
  int ticks = 0;
  Steps steps({ [&] { return ++ticks > 5; },
                [&] {
                  screenshot = app.takeScreenshot();
                  app.close();
                  return true;
                } });
  app.runEventLoop();

  REQUIRE(screenshot.width() == 60);
  REQUIRE(screenshot.sample(30, 20).hexGreen() == 0xc0);
}

TEST_CASE("An ApplicationWindow can be shown again from its own draw callback", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  ApplicationWindow app;
  int shows = 0;
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));
  Steps steps({ [&] {
                 app.show(Dimension::nativePixels(120), Dimension::nativePixels(90));
                 shows++;
                 return true;
               },
                [&] {
                  app.close();
                  return true;
                } });
  app.runEventLoop();
  REQUIRE(shows == 1);
  REQUIRE(app.nativeWidth() == 120);
  REQUIRE(app.nativeHeight() == 90);
}

TEST_CASE("An ApplicationWindow can be shown again from its own mouse handler", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  ApplicationWindow app;
  Frame button;
  int shows = 0;
  button.onMouseDown() = [&](const MouseEvent&) {
    app.show(Dimension::nativePixels(120), Dimension::nativePixels(90));
    shows++;
  };
  app.setFlexLayout(true);
  button.layout().setFlexGrow(1.0f);
  app.addChild(button);
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));

  Display* display = XOpenDisplay(nullptr);
  REQUIRE(display);
  Steps steps({ [&] {
                 XEvent event = {};
                 event.xbutton.type = ButtonPress;
                 event.xbutton.window = (::Window)app.window()->nativeHandle();
                 event.xbutton.button = Button1;
                 event.xbutton.x = 10;
                 event.xbutton.y = 10;
                 event.xbutton.same_screen = True;
                 XSendEvent(display, event.xbutton.window, False, ButtonPressMask, &event);
                 XFlush(display);
                 return true;
               },
                [&] { return shows > 0; },
                [&] {
                  app.close();
                  return true;
                } });
  app.runEventLoop();
  XCloseDisplay(display);
  REQUIRE(shows == 1);
}

TEST_CASE("A hidden ApplicationWindow can be shown again from its own event loop",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  ApplicationWindow app;
  int shows = 0;
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));
  void* handle = app.window()->nativeHandle();
  Steps steps({ [&] {
                 app.hide();
                 return true;
               },
                [&] {
                  app.show(Dimension::nativePixels(120), Dimension::nativePixels(90));
                  shows++;
                  return true;
                },
                [&] { return app.isShowing(); },
                [&] {
                  app.close();
                  return true;
                } });
  app.runEventLoop();
  REQUIRE(shows == 1);
  REQUIRE(app.nativeWidth() == 120);
  REQUIRE(app.nativeHeight() == 90);
  REQUIRE(app.window()->nativeHandle() == handle);
}

TEST_CASE("A closed ApplicationWindow can be shown again while another window is open",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  ApplicationWindow app;
  ApplicationWindow other;
  int shows = 0;
  app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));
  other.show(Dimension::nativePixels(60), Dimension::nativePixels(40));
  void* handle = app.window()->nativeHandle();
  Steps steps({ [&] {
                 app.close();
                 return true;
               },
                [&] { return !app.isShowing(); },
                [&] {
                  app.showMaximized();
                  shows++;
                  return true;
                },
                [&] { return app.isShowing(); },
                [&] {
                  app.close();
                  other.close();
                  return true;
                } });
  app.runEventLoop();
  REQUIRE(shows == 1);
  REQUIRE(app.window()->nativeHandle() != handle);
}

TEST_CASE("Static editor child", "[.static_editor_child]") {
  static ApplicationEditor editor;
  std::exit(0);
}

TEST_CASE("Static editors trip a debug assert", "[integration]") {
  std::string output;
  bool exited_cleanly = spawnChildProcess(hostExecutable().string(), "[.static_editor_child]",
                                          output, 60000);
  INFO(output);
#ifdef NDEBUG
  REQUIRE(exited_cleanly);
#else
  REQUIRE_FALSE(exited_cleanly);
  REQUIRE(output.find("application_editor.cpp") != std::string::npos);
  REQUIRE(output.find("Assertion failed") != std::string::npos);
#endif
}
#endif
