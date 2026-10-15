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

#if VISAGE_LINUX
#include <thread>
#include <X11/Xatom.h>
#include <X11/Xlib.h>

namespace {
  struct SelectionReply {
    bool replied = false;
    Atom type = None;
    std::string data;
  };

  SelectionReply requestSelection(Display* display, ::Window requestor,
                                  visage::Window* plugin_window, Atom target) {
    Atom clipboard = XInternAtom(display, "CLIPBOARD", False);
    Atom property = XInternAtom(display, "VISAGE_TEST_SELECTION", False);
    XConvertSelection(display, clipboard, target, property, requestor, CurrentTime);
    XFlush(display);

    SelectionReply reply;
    XEvent event;
    for (int i = 0; i < 400 && !reply.replied; ++i) {
      plugin_window->processPluginFdEvents();
      reply.replied = XCheckTypedWindowEvent(display, requestor, SelectionNotify, &event);
      if (!reply.replied)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (!reply.replied || event.xselection.property == None)
      return reply;

    int format = 0;
    unsigned long num_items = 0, bytes_after = 0;
    unsigned char* data = nullptr;
    XGetWindowProperty(display, requestor, property, 0, ~0, True, AnyPropertyType, &reply.type,
                       &format, &num_items, &bytes_after, &data);
    if (data) {
      int item_size = format == 32 ? sizeof(long) : format / 8;
      reply.data = std::string(reinterpret_cast<char*>(data), num_items * item_size);
      XFree(data);
    }
    return reply;
  }
}

TEST_CASE("X11 plugin windows serve clipboard requests", "[integration]") {
  const char* display_name = std::getenv("DISPLAY");
  if (display_name == nullptr || display_name[0] == 0) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window root = DefaultRootWindow(host);
  ::Window parent = XCreateSimpleWindow(host, root, 0, 0, 200, 200, 0, 0, 0);
  ::Window requestor = XCreateSimpleWindow(host, root, 0, 0, 1, 1, 0, 0, 0);
  XMapWindow(host, parent);
  XFlush(host);

  std::unique_ptr<visage::Window> plugin_window =
      visage::createPluginWindow(visage::Dimension::logicalPixels(100),
                                 visage::Dimension::logicalPixels(100), (void*)parent);
  REQUIRE(plugin_window);

  visage::setClipboardText("h\xc3\xa9llo \xe2\x82\xac");
  REQUIRE(visage::readClipboardText() == "h\xc3\xa9llo \xe2\x82\xac");

  Atom utf8_string = XInternAtom(host, "UTF8_STRING", False);
  SelectionReply utf8 = requestSelection(host, requestor, plugin_window.get(), utf8_string);
  REQUIRE(utf8.replied);
  REQUIRE(utf8.type == utf8_string);
  REQUIRE(utf8.data == "h\xc3\xa9llo \xe2\x82\xac");

  SelectionReply latin1 = requestSelection(host, requestor, plugin_window.get(), XA_STRING);
  REQUIRE(latin1.replied);
  REQUIRE(latin1.type == XA_STRING);
  REQUIRE(latin1.data == "h\xe9llo ?");

  Atom targets = XInternAtom(host, "TARGETS", False);
  SelectionReply supported = requestSelection(host, requestor, plugin_window.get(), targets);
  REQUIRE(supported.replied);
  REQUIRE(supported.type == XA_ATOM);
  REQUIRE(supported.data.size() == 3 * sizeof(long));

  plugin_window.reset();
  REQUIRE(XGetSelectionOwner(host, XInternAtom(host, "CLIPBOARD", False)) == None);

  XDestroyWindow(host, requestor);
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}

TEST_CASE("X11 clipboard moves to another plugin window when its owner closes", "[integration]") {
  const char* display_name = std::getenv("DISPLAY");
  if (display_name == nullptr || display_name[0] == 0) {
    SUCCEED("No X display");
    return;
  }

  Display* host = XOpenDisplay(nullptr);
  REQUIRE(host);
  ::Window root = DefaultRootWindow(host);
  ::Window parent = XCreateSimpleWindow(host, root, 0, 0, 200, 200, 0, 0, 0);
  ::Window requestor = XCreateSimpleWindow(host, root, 0, 0, 1, 1, 0, 0, 0);
  XMapWindow(host, parent);
  XFlush(host);

  std::unique_ptr<visage::Window> windows[] = {
    visage::createPluginWindow(visage::Dimension::logicalPixels(100),
                               visage::Dimension::logicalPixels(100), (void*)parent),
    visage::createPluginWindow(visage::Dimension::logicalPixels(100),
                               visage::Dimension::logicalPixels(100), (void*)parent)
  };
  REQUIRE(windows[0]);
  REQUIRE(windows[1]);

  visage::setClipboardText("copied");
  Atom clipboard = XInternAtom(host, "CLIPBOARD", False);
  ::Window owner = XGetSelectionOwner(host, clipboard);
  int owner_index = owner == (::Window)windows[0]->nativeHandle() ? 0 : 1;
  REQUIRE(owner == (::Window)windows[owner_index]->nativeHandle());

  XSetSelectionOwner(host, XA_PRIMARY, requestor, CurrentTime);
  XSync(host, False);

  windows[owner_index].reset();
  REQUIRE(XGetSelectionOwner(host, XA_PRIMARY) == requestor);
  visage::Window* remaining = windows[1 - owner_index].get();
  REQUIRE(XGetSelectionOwner(host, clipboard) == (::Window)remaining->nativeHandle());
  REQUIRE(visage::readClipboardText() == "copied");

  SelectionReply reply = requestSelection(host, requestor, remaining,
                                          XInternAtom(host, "UTF8_STRING", False));
  REQUIRE(reply.replied);
  REQUIRE(reply.data == "copied");

  windows[1 - owner_index].reset();
  REQUIRE(XGetSelectionOwner(host, clipboard) == None);

  XDestroyWindow(host, requestor);
  XDestroyWindow(host, parent);
  XCloseDisplay(host);
}
#endif
