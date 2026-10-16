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

#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
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

  bool displayIsUnlocked(Display* display, std::vector<std::thread>& threads,
                         std::chrono::milliseconds timeout = std::chrono::seconds(1)) {
    auto locked = std::make_shared<std::promise<void>>();
    std::future<void> future = locked->get_future();
    threads.emplace_back([display, locked] {
      XLockDisplay(display);
      XUnlockDisplay(display);
      locked->set_value();
    });
    return future.wait_for(timeout) == std::future_status::ready;
  }

  ::Window findMappedWindow(Display* display, const char* name) {
    ::Window root = 0, parent = 0;
    ::Window* children = nullptr;
    unsigned int num_children = 0;
    if (XQueryTree(display, DefaultRootWindow(display), &root, &parent, &children, &num_children) == 0)
      return 0;

    ::Window result = 0;
    for (unsigned int i = 0; i < num_children && result == 0; ++i) {
      char* window_name = nullptr;
      XWindowAttributes attributes;
      if (XFetchName(display, children[i], &window_name) && std::string(window_name) == name &&
          XGetWindowAttributes(display, children[i], &attributes) && attributes.map_state == IsViewable)
        result = children[i];
      if (window_name)
        XFree(window_name);
    }
    if (children)
      XFree(children);
    return result;
  }
}

TEST_CASE("X11 windows release the display lock before calling the app", "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  class Target : public Frame {
  public:
    bool receivesDragDropFiles() override { return true; }
    void dragFilesEnter(const std::vector<std::string>& paths) override { on_drag(); }
    void dropFiles(const std::vector<std::string>& paths) override { on_drop(); }

    std::function<void()> on_drag;
    std::function<void()> on_drop;
  };

  ApplicationWindow app;
  Target target;
  app.setFlexLayout(true);
  target.layout().setFlexGrow(1.0f);
  app.addChild(target);

  std::vector<std::thread> threads;
  std::vector<bool> unlocked;
  auto check = [&] {
    unlocked.push_back(displayIsUnlocked((Display*)app.window()->globalDisplay(), threads));
  };

  SECTION("onShow and onHide") {
    app.onShow() = check;
    app.onHide() = check;
    app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));
    app.hide();
    CHECK(unlocked == std::vector<bool> { true, true });
  }

  SECTION("Mouse move") {
    target.onMouseMove() = [&](const MouseEvent&) {
      if (unlocked.empty())
        check();
    };
    app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));

    Display* display = XOpenDisplay(nullptr);
    REQUIRE(display);
    auto move = [&](int x, int y) {
      XEvent event = {};
      event.xmotion.type = MotionNotify;
      event.xmotion.window = (::Window)app.window()->nativeHandle();
      event.xmotion.x = x;
      event.xmotion.y = y;
      event.xmotion.same_screen = True;
      XSendEvent(display, event.xmotion.window, False, PointerMotionMask, &event);
      XFlush(display);
    };
    Steps steps({ [&] {
                   move(10, 10);
                   return true;
                 },
                  [&] {
                    move(20, 20);
                    return true;
                  },
                  [&] { return !unlocked.empty(); },
                  [&] {
                    app.close();
                    return true;
                  } });
    app.runEventLoop();
    XCloseDisplay(display);
    CHECK(unlocked == std::vector<bool> { true });
  }

  SECTION("File drag and drop") {
    target.on_drag = check;
    target.on_drop = check;
    app.show(Dimension::nativePixels(100), Dimension::nativePixels(80));

    Display* source = XOpenDisplay(nullptr);
    REQUIRE(source);
    ::Window source_window = XCreateSimpleWindow(source, DefaultRootWindow(source), 0, 0, 1, 1, 0, 0, 0);
    Atom uri_list = XInternAtom(source, "text/uri-list", False);
    XSetSelectionOwner(source, XInternAtom(source, "XdndSelection", False), source_window, CurrentTime);
    XSync(source, False);

    ::Window window = (::Window)app.window()->nativeHandle();
    auto send = [&](const char* type, long data1, long data2, long data4) {
      XEvent event = {};
      event.xclient.type = ClientMessage;
      event.xclient.window = window;
      event.xclient.message_type = XInternAtom(source, type, False);
      event.xclient.format = 32;
      event.xclient.data.l[0] = source_window;
      event.xclient.data.l[1] = data1;
      event.xclient.data.l[2] = data2;
      event.xclient.data.l[3] = CurrentTime;
      event.xclient.data.l[4] = data4;
      XSendEvent(source, window, False, NoEventMask, &event);
      XSync(source, False);
    };
    auto answer_selection = [&] {
      while (XPending(source)) {
        XEvent event;
        XNextEvent(source, &event);
        if (event.type != SelectionRequest)
          continue;

        const XSelectionRequestEvent& request = event.xselectionrequest;
        std::string files = "file:///tmp/visage-drop.wav\r\n";
        XChangeProperty(source, request.requestor, request.property, uri_list, 8, PropModeReplace,
                        reinterpret_cast<const unsigned char*>(files.data()), files.size());
        XSelectionEvent notify = {};
        notify.type = SelectionNotify;
        notify.requestor = request.requestor;
        notify.selection = request.selection;
        notify.target = request.target;
        notify.property = request.property;
        notify.time = request.time;
        XSendEvent(source, request.requestor, False, NoEventMask, reinterpret_cast<XEvent*>(&notify));
        XSync(source, False);
        return true;
      }
      return false;
    };

    Steps steps({ [&] {
                   send("XdndEnter", 5 << 24, uri_list, 0);
                   return true;
                 },
                  answer_selection,
                  [&] {
                    int x = 0, y = 0;
                    ::Window child;
                    XTranslateCoordinates(source, window, DefaultRootWindow(source), 50, 40, &x, &y, &child);
                    send("XdndPosition", 0, (x << 16) | y, XInternAtom(source, "XdndActionCopy", False));
                    return true;
                  },
                  [&] { return unlocked.size() == 1; },
                  [&] {
                    send("XdndDrop", 0, CurrentTime, 0);
                    return true;
                  },
                  [&] { return unlocked.size() == 2; },
                  [&] {
                    app.close();
                    return true;
                  } });
    app.runEventLoop();
    XDestroyWindow(source, source_window);
    XCloseDisplay(source);
    CHECK(unlocked == std::vector<bool> { true, true });
  }

  for (std::thread& thread : threads)
    thread.join();
}

TEST_CASE("X11 clipboard reads and message boxes release the display lock while waiting",
          "[integration]") {
  if (!hasDisplay()) {
    SUCCEED("No X display");
    return;
  }

  std::unique_ptr<visage::Window> window = createWindow(Dimension::nativePixels(100),
                                                        Dimension::nativePixels(100));
  REQUIRE(window);
  Display* display = (Display*)window->globalDisplay();
  std::vector<std::thread> threads;
  std::atomic<bool> unlocked = false;

  SECTION("Reading the clipboard from a slow owner") {
    Display* source = XOpenDisplay(nullptr);
    REQUIRE(source);
    ::Window source_window = XCreateSimpleWindow(source, DefaultRootWindow(source), 0, 0, 1, 1, 0, 0, 0);
    Atom utf8 = XInternAtom(source, "UTF8_STRING", False);
    XSetSelectionOwner(source, XInternAtom(source, "CLIPBOARD", False), source_window, CurrentTime);
    XSync(source, False);

    std::thread owner([&] {
      for (int i = 0; i < 400; ++i) {
        XEvent event;
        if (!XCheckTypedEvent(source, SelectionRequest, &event)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }

        unlocked = displayIsUnlocked(display, threads, std::chrono::milliseconds(100));
        const XSelectionRequestEvent& request = event.xselectionrequest;
        std::string text = "pasted";
        XChangeProperty(source, request.requestor, request.property, utf8, 8, PropModeReplace,
                        reinterpret_cast<const unsigned char*>(text.data()), text.size());
        XSelectionEvent notify = {};
        notify.type = SelectionNotify;
        notify.requestor = request.requestor;
        notify.selection = request.selection;
        notify.target = request.target;
        notify.property = request.target == utf8 ? request.property : None;
        notify.time = request.time;
        XSendEvent(source, request.requestor, False, NoEventMask, reinterpret_cast<XEvent*>(&notify));
        XSync(source, False);
        return;
      }
    });

    std::string text = readClipboardText();
    owner.join();
    CHECK(text == "pasted");
    XDestroyWindow(source, source_window);
    XCloseDisplay(source);
  }

  SECTION("A message box waiting to be dismissed") {
    std::thread user([&] {
      Display* user_display = XOpenDisplay(nullptr);
      ::Window box = 0;
      for (int i = 0; i < 400 && box == 0; ++i) {
        box = findMappedWindow(user_display, "visage lock test");
        if (box == 0)
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      unlocked = displayIsUnlocked(display, threads, std::chrono::milliseconds(100));

      XEvent event = {};
      event.xkey.type = KeyPress;
      event.xkey.window = box;
      event.xkey.same_screen = True;
      XSendEvent(user_display, box, False, KeyPressMask, &event);
      XCloseDisplay(user_display);
    });

    showMessageBox("visage lock test", "Press a key");
    user.join();
  }

  for (std::thread& thread : threads)
    thread.join();
  CHECK(unlocked);
}

#endif
